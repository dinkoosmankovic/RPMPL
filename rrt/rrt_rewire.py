from rrt import RRT
import numpy as np
import logging
import kdtree
import time
from rrt.node import Node


class RRTRewire(RRT):
    def __init__(self, start, goal, args):
        super().__init__(start, goal, args)

    def resolve_with_check(self, start) -> bool:
        for i in range(1, len(self.path)):
            if not self.robot.is_valid(self.path[i-1], self.path[i]):
                return self.resolve(start)

        for q in self.path:
            if not self.robot.is_valid(q):
                return self.resolve(start)

        # logging.info("using last tree")
        return False

    def reverse_parent(self, q_start, q_old_parent):
        q = q_start
        q_old_parent.children.append(q_start)
        q_start.children.remove(q_old_parent)

        while q is not None:
            print("reversing nodes: ", q)
            if q.parent is None:
                q.parent = q_old_parent
                break
            q.children.append(q.parent)
            q.parent.children.remove(q)

            q_temp = q.parent
            q.parent = q_old_parent
            q_old_parent = q
            q = q_temp

    def dfs(self, q_start):
        q_start.visited = True
        for q in q_start.children:
            if self.robot.is_valid(q_start.position, q.position):
                self.dfs(q)

    def resolve(self, start) -> bool:
        # add new start node
        print("number of nodes: ", len(self.nodes))
        self.start = start
        # add new start node to tree and update it so it points from start
        q_near = self.tree.search_nn(self.start)
        if np.linalg.norm(start - q_near[0].data) > 1e-05:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            q_start = Node(self.start)
            q_goal = self.nodes[-1]
            self.nodes.pop()
            self.nodes.append(q_start)
            self.nodes.append(q_goal)

            q_start.children.append(q_near_node)
        else:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            if q_near_node.parent is not None:
                q_near_node.parent.children.remove(q_near_node)
                q_near_node.children.append(q_near_node.parent)
                q_near_node.parent = None
            q_start = q_near_node

        self.reverse_parent_v2(q_start)

        # remove nodes that are no longer in free space
        del self.tree
        self.tree = kdtree.create(dimensions=len(start))
        for q in self.nodes[:]:
            if not self.robot.is_valid(q.position):
                q_parent = q.parent
                if q_parent is not None:
                    q_parent.children.remove(q)
                for child in q.children:
                    child.parent = None
                self.nodes.remove(q)
            else:
                self.tree.add(q.position)

        # self.start = start
        # q_near = self.tree.search_nn(self.start)
        # if np.linalg.norm(start-q_near[0].data) > 1e-05:
        #     q_near_node = self.get_parent_node(Node(q_near[0].data))
        #     q_old_parent = q_near_node.parent
        #
        #     q_new = Node(self.start)
        #     self.nodes.append(q_new)
        #
        #     q_near_node.parent = q_new
        #     q_new.children.append(q_near_node)
        #     q_start = q_new
        # else:
        #     q_near_node = self.get_parent_node(Node(q_near[0].data))
        #     q_old_parent = q_near_node.parent
        #     q_near_node.parent = None
        #     q_start = q_near_node
        #     self.start = q_start.position
        #
        # # rotate rest of the tree to point from new start node
        # if q_old_parent is not None:
        #     self.reverse_parent(q_start=q_old_parent, q_old_parent=q_near_node)

        # remove nodes that are not connected to start node
        self.dfs(q_start)
        i = 0
        del self.tree
        self.tree = kdtree.create([self.start])
        print("before: ", len(self.nodes))
        for q in self.nodes[:]:
            if not q.visited:
                self.nodes.remove(q)
            else:
                self.tree.add(q.position)
                q.visited = False
        print("after: ", len(self.nodes))

        return self.solve()

    # def update_path(self):
    #     self.path = self.robot.traj[::-1]

    def update_path(self):
        if len(self.path) < 2 or self.robot.curr_q is None:
            return
        # print(self.robot.curr_q, " ", self.path[0], " ", self.path[1])

        if np.linalg.norm(self.robot.curr_q-self.path[1]) < 1e-05:
            self.path = self.path[1:]

    def get_root_node(self, q):
        if q.parent is None:
            return q
        return self.get_root_node(q.parent)

    def dfs_quick(self, q, tree):
        if not self.robot.is_valid(q.position):
            return

        for qq in q.children:
            if self.robot.is_valid(q.position, qq.position, self.num_checks*2):
                self.dfs_quick(qq, tree)
        tree.append(q)

    def closest(self, q, tree):
        dist = float('inf')
        q_ret = None
        for qq in tree:
            if np.linalg.norm(q.position - qq.position) < dist:
                q_ret = qq
                dist = np.linalg.norm(q.position-qq.position)
        return q_ret

    def connect(self, q_from, q_to):
        from_tree = []
        self.dfs_quick(self.get_root_node(q_from), from_tree)

        to_tree = []
        self.dfs_quick(self.get_root_node(q_to), to_tree)

        q_1 = self.closest(q_from, to_tree)
        q_2 = self.closest(q_to, from_tree)

        if self.robot.is_valid(q_from.position, q_1.position, self.num_checks*2):
            q_from.children.append(q_1)
            self.reverse_parent_v2(q_from)
            return True

        if self.robot.is_valid(q_2.position, q_to.position, self.num_checks*2):
            q_2.children.append(q_to)
            if q_to.parent is not None:
                q_to.parent.children.remove(q_to)
            q_to.parent = q_2
            return True

        return False

    def fix_hanging_node(self, q_start):
        if len(q_start.children) == 0:
            return
        for q in q_start.children:
            if q.parent is None:
                q.parent = q_start
            self.fix_hanging_node(q)

    def resolve_rewire(self, start) -> bool:
        for i in range(1, len(self.path)):
            if not self.robot.is_valid(self.path[i-1], self.path[i]):
                return self.resolve_with_rewire(start)

        return False

    def reverse_parent_v2(self, q_start):
        for q in q_start.children:
            if q.parent is None:
                q.parent = q_start
                continue
            elif q.parent is q_start:
                continue
            q.parent.children.remove(q)
            q.children.append(q.parent)
            q.parent = q_start
            self.reverse_parent_v2(q)

    def resolve_with_rewire(self, start):
        self.start = start
        # add new start node to tree and update it so it points from start
        q_near = self.tree.search_nn(self.start)
        if np.linalg.norm(start - q_near[0].data) > 1e-05:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            q_start = Node(self.start)
            q_goal = self.nodes[-1]
            self.nodes.pop()
            self.nodes.append(q_start)
            self.nodes.append(q_goal)

            q_start.children.append(q_near_node)
        else:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            if q_near_node.parent is not None:
                q_near_node.parent.children.remove(q_near_node)
                q_near_node.children.append(q_near_node.parent)
                q_near_node.parent = None
            q_start = q_near_node

        self.reverse_parent_v2(q_start)

        # find nodes on path that are not available and try to connect without using them
        q_goal_node = self.nodes[-1]
        q_curr = q_goal_node
        nodes_to_connect = []
        q_last = q_goal_node

        while q_curr is not None:
            if q_curr.parent is None:
                break
            valid_curr = self.robot.is_valid(q_curr.position)
            valid_parent = self.robot.is_valid(q_curr.parent.position)
            valid_path = self.robot.is_valid(q_curr.parent.position, q_curr.position)

            if valid_curr and valid_parent:
                if valid_path:
                    q_last = q_curr
                    q_curr = q_curr.parent
                else:
                    q_last = q_curr
                    nodes_to_connect.append(q_curr)
                    nodes_to_connect.append(q_curr.parent)
                    q_curr.parent.children.remove(q_curr)
                    q_curr.parent = None

            if valid_curr and not valid_parent:
                q_last = q_curr
                q_curr.parent.children.remove(q_curr)
                q_temp = q_curr
                q_curr = q_curr.parent
                q_temp.parent = None

            if not valid_curr and valid_parent:
                nodes_to_connect.append(q_last)
                nodes_to_connect.append(q_curr.parent)
                q_curr.parent.children.remove(q_curr)
                q_temp = q_curr
                q_curr = q_curr.parent
                q_temp.parent = None

            if not valid_curr and not valid_parent:
                q_curr = q_curr.parent

        res = True
        if len(nodes_to_connect) == 0:
            res = False

        for i in range(0, len(nodes_to_connect), 2):
            ress = self.connect(nodes_to_connect[i + 1], nodes_to_connect[i])
            res = (ress and res)

        # remove nodes that are not connected to start node
        self.dfs(q_start)
        del self.tree
        self.tree = kdtree.create([self.start])
        for q in self.nodes[:]:
            if not q.visited:
                self.nodes.remove(q)
            else:
                self.tree.add(q.position)
                q.visited = False

        if res:
            print("connected tree")
            return True
        return self.solve()
