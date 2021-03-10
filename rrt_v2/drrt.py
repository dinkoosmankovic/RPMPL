from rrt_v2.rrt import RRT
import numpy as np
import kdtree
from rrt_v2.node import Node
from timeit import default_timer as timer


class DRRT(RRT):
    def __init__(self, start, goal, args):
        super().__init__(start, goal, args)

    def replan(self, start):
        print(start, self.goal)
        if not(self.robot.is_valid(q=start) and self.robot.is_valid(q=self.goal)):
            raise ValueError("Robot is in collision in start or goal position!")

        for i in range(1, len(self.path)):
            if not self.robot.is_valid(self.path[i-1], self.path[i]):
                s = timer()
                res = self.resolve(start)
                diff = timer() - s
                return res, diff
        return False, 0

    def dfs(self, q_start):
        q_start.visited = True
        for q in q_start.children:
            if self.robot.is_valid(q_start.position, q.position):
                self.dfs(q)

    def update_path(self):
        if len(self.path) < 2 or self.robot.curr_q is None:
            return

        if np.linalg.norm(self.robot.curr_q-self.path[1]) < 1e-05:
            self.path = self.path[1:]

    def reverse_parent(self, q_start):
        for q in q_start.children:
            if q.parent is None:
                q.parent = q_start
                continue
            elif q.parent is q_start:
                continue
            q.parent.children.remove(q)
            q.children.append(q.parent)
            q.parent = q_start
            self.reverse_parent(q)

    def resolve(self, start) -> bool:
        # add new start node
        self.start = start
        # add new start node to tree and update it so it points from start
        q_near = self.tree.search_nn(self.start)
        if np.linalg.norm(start - q_near[0].data) > 1e-05:
            q_near_node = self.get_parent_node(q_near[0])
            # q_near_node = self.get_parent_node(Node(q_near[0].data))
            q_start = Node(self.start)
            q_goal = self.nodes[-1]
            self.nodes.pop()

            q_new_kdnode = self.tree.add(self.start)
            self.nodes_index[q_new_kdnode] = q_start
            self.nodes.append(q_start)

            self.nodes.append(q_goal)

            q_start.children.append(q_near_node)
        else:
            q_near_node = self.get_parent_node(q_near[0])
            # q_near_node = self.get_parent_node(Node(q_near[0].data))
            if q_near_node.parent is not None:
                q_near_node.parent.children.remove(q_near_node)
                q_near_node.children.append(q_near_node.parent)
                q_near_node.parent = None
            q_start = q_near_node

        self.reverse_parent(q_start)

        # remove nodes that are no longer in free space
        for q in self.nodes[:]:
            if not self.robot.is_valid(q.position):
                q_parent = q.parent
                if q_parent is not None:
                    q_parent.children.remove(q)
                for child in q.children:
                    child.parent = None
                self.nodes.remove(q)

        self.dfs(q_start)
        i = 0
        del self.tree
        self.tree = kdtree.create([self.start])
        self.nodes_index = dict()
        self.nodes_index[self.tree] = q_start

        for q in self.nodes[:]:
            if not q.visited:
                self.nodes.remove(q)
            else:
                qq = self.tree.add(q.position)
                self.nodes_index[qq] = q

                q.visited = False

        return self.solve()

