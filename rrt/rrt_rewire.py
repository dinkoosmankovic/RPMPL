from rrt import RRT
import numpy as np
import logging
import kdtree
from rrt.node import Node


class RRTRewire(RRT):
    def __init__(self, start, goal, args):
        super().__init__(start, goal, args)

    # check if we can still hold the same trajectory
    def resolve_simple(self) -> bool:
        for q in self.path:
            if not self.robot.is_valid(q):
                self.clear()
                return self.resolve()
        # logging.info("Using last tree")
        return False

    def resolve_with_check(self, start) -> bool:
        for q in self.path:
            if not self.robot.is_valid(q):
                return self.resolve(start)
        # logging.info("using last tree")
        return False

    def reverse_parent(self, q_start, q_old_parent):
        q = q_start
        q_old_parent.children.append(q_start)
        q_start.children.remove(q_old_parent)

        while q.parent is not None:
            q.children.append(q.parent)
            q.parent.children.remove(q)

            q_temp = q.parent
            q.parent = q_old_parent
            q_old_parent = q
            q = q_temp

    def dfs(self, q_start):
        q_start.visited = True
        if len(q_start.children) == 0:
            return
        for q in q_start.children:
            self.dfs(q)

    def resolve(self, start) -> bool:
        i = 0
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

        # add new start node
        self.start = start
        q_near = self.tree.search_nn(self.start)
        if np.linalg.norm(start-q_near[0].data) > 1e-05:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            q_old_parent = q_near_node.parent

            q_new = Node(self.start)
            self.nodes.append(q_new)

            q_near_node.parent = q_new
            q_new.children.append(q_near_node)
            q_start = q_new
        else:
            q_near_node = self.get_parent_node(Node(q_near[0].data))
            q_old_parent = q_near_node.parent
            q_near_node.parent = None
            q_start = q_near_node


        # rotate rest of the tree to point from new start node
        if q_old_parent is not None:
            self.reverse_parent(q_start=q_old_parent, q_old_parent=q_near_node)

        # remove nodes that are not connected to start node
        self.dfs(q_start)
        i = 0
        del self.tree
        self.tree = kdtree.create([self.start])
        for q in self.nodes[:]:
            if not q.visited:
                self.nodes.remove(q)
            else:
                self.tree.add(q.position)
                q.visited = False
        return self.solve()

