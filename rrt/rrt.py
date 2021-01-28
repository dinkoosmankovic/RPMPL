from .node import Node
from state_spaces.real_vector_space_2d import RealVectorSpace2D
import kdtree
from kdtree import KDNode
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure


class RRT:
    def __init__(self, start, goal, args = None) -> None:
        self.eps = 1.0
        self.max_iter = 1000
        self.start = start
        self.goal = goal
        self.tree = kdtree.create([start])
        self.nodes = [Node(start)]
        self.gamma = 0.1
        self.space_state = RealVectorSpace2D(args["obstacles"])

    def __str__(self) -> str:
        return "Tree size: " + str(len(self.nodes))

    def update_obstacles(self, obstacles) -> None:
        self.space_state.update_obstacles(obstacles)

    def get_parent_node(self, node) -> Node:
        return self.nodes[self.nodes.index(node)]

    def solve(self) -> bool:
        for i in range(self.max_iter):
            q_rand = None
            if random.random() < self.gamma:
                q_rand = self.goal
            else:
                q_rand = self.space_state.get_qrand(0, 25, 0, 25)
            q_near = self.tree.search_nn(q_rand)
            q_new = self.space_state.get_qnew(q_near[0].data, q_rand, self.eps)
            if (self.space_state.is_valid(q_near[0].data, q_new)):
                self.tree.add(q_new)
                q_new_parent = self.get_parent_node(Node(q_near[0].data))
                q_new_node = Node(q_new, q_new_parent)
                self.nodes.append(q_new_node)
                if np.linalg.norm(q_new - self.goal) < self.eps:
                    return True
        return False

    def get_solution_path(self) -> list:
        current_node = self.nodes[-1]
        path = []
        while(current_node is not None):
            path.append(current_node.position)
            current_node = current_node.parent
        return path

    def visualize(self) -> None:
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.axis('equal')
        xs = []
        ys = []

        for o in self.space_state.obs:
            circle = plt.Circle(o["p"], o["r"], color='k')
            ax.add_patch(circle)

        for n in self.nodes:
            # print(n)
            if n.parent is not None:
                p1, p2 = n.position, n.parent.position
                xs = [p1[0], p2[0]]
                ys = [p1[1], p2[1]]
                ax.plot(xs, ys, marker='o', color="green")
        ax.scatter(self.start[0], self.start[1],
                   marker="o", color="blue", s=100, zorder=3)
        ax.scatter(self.goal[0], self.goal[1], marker="o",
                   color="red", s=100, zorder=3)
        ax.scatter(self.goal[0], self.goal[1], marker="o",
                   color="red", s=3000, zorder=3, alpha=0.1)

        pxs = []
        pys = []
        for p in self.get_solution_path():
            pxs.append(p[0])
            pys.append(p[1])

        ax.plot(pxs, pys, marker='o', color="magenta", linewidth=3)
        plt.show()
