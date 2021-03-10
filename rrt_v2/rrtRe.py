from rrt_v2.rrt import RRT
import numpy as np
import kdtree
from rrt_v2.node import Node
from timeit import default_timer as timer


class RRTre(RRT):
    def __init__(self, start, goal, args):
        super().__init__(start, goal, args)

    def replan(self):
        if not (self.robot.is_valid(q=self.start) and self.robot.is_valid(q=self.goal)):
            raise ValueError("Robot is in collision in start or goal position!")

        for i in range(1, len(self.path)):
            if not self.robot.is_valid(self.path[i - 1], self.path[i]):
                s = timer()
                self.clear()
                res = self.solve()
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
