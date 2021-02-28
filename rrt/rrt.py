from .node import Node
import kdtree
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy.spatial import Delaunay

import logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                    datefmt="%H:%M:%S")


class RRT:
    def __init__(self, start, goal, args=None) -> None:
        self.eps = args["eps"]
        self.max_iter = args["max_iter"]
        self.start = start
        self.goal = goal
        self.tree = kdtree.create([start])
        self.nodes = [Node(start)]
        self.gamma = args["gamma"]
        self.state_space = args["state_space"]
        self.robot = args["robot"]
        self.num_checks = args["num_checks"]
        if not(self.robot.is_valid(q=start) and self.robot.is_valid(q=goal)):
            raise Exception("Robot is in collision in start or goal position!")

        self.path = []

    def clear(self):
        self.tree = kdtree.create([self.start])
        self.nodes = [Node(self.start)]

    def __str__(self) -> str:
        return "Tree size: " + str(len(self.nodes))

    def get_parent_node(self, node) -> Node:
        return self.nodes[self.nodes.index(node)]

    def solve(self) -> bool:
        for i in range(self.max_iter):
            if (i % 100 == 0):
                logging.info("Iteration %d with %d nodes", i, len(self.nodes))
            q_rand = None
            if random.random() < self.gamma:
                q_rand = self.goal
            else:
                q_rand = self.state_space.get_qrand()

            q_near = self.tree.search_nn(q_rand)
            q_new = self.state_space.get_qnew(q_near[0].data, q_rand, self.eps)
            # print("qnear: ", q_near[0].data)
            # print("qnew: ", q_new)
            if self.robot.is_valid(q_near[0].data, q_new, self.num_checks):
                self.tree.add(q_new)
                q_new_parent = self.get_parent_node(Node(q_near[0].data))
                q_new_node = Node(q_new, q_new_parent)
                q_new_parent.children.append(q_new_node)
                self.nodes.append(q_new_node)
                if self.state_space.equal(q_new, self.goal, self.eps):
                    return True
        return False

    def get_solution_path(self) -> list:
        current_node = self.nodes[-1]
        if np.linalg.norm(current_node.position - self.goal) > self.eps:
            return []
        path = []
        while (current_node is not None):
            path.append(current_node.position)
            current_node = current_node.parent
        self.path = path
        return path

    def get_obstacles(self) -> tuple:
        if self.state_space.get_dimensions() != 2:
            raise Exception("This is only for 2D spaces")
        state_space = self.state_space
        obs = []
        # print("range: ", self.space_state.range)
        step_x = np.abs(
            state_space.range[0]-state_space.range[1])/100
        step_y = np.abs(
            state_space.range[0]-state_space.range[1])/100
        range_x = np.arange(
            state_space.range[0], state_space.range[1], step=step_x)

        range_y = np.arange(
            state_space.range[0], state_space.range[1], step=step_y)

        for i in range_x:
            for j in range_y:
                res = self.robot.is_valid(q=[i, j])
                # print([i,j], res)
                if not(res):
                    obs.append([i, j])

        return np.array(obs)

    def visualize(self) -> None:
        if len(self.start) != 2:
            raise Exception("Cannot visualize non 2D planning")

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.axis('equal')
        xs = []
        ys = []

        obs = self.get_obstacles()
        tri = Delaunay(obs)
        triangles = np.empty((0, 3), dtype=int)
        for i in range(0, tri.simplices.shape[0]):
            simplex = tri.simplices[i]
            x = tri.points[simplex[0]]
            y = tri.points[simplex[1]]
            z = tri.points[simplex[2]]
            d0 = np.sqrt(np.dot(x-y, x-y))
            d1 = np.sqrt(np.dot(x-z, x-z))
            d2 = np.sqrt(np.dot(z-y, z-y))
            max_edge = max([d0, d1, d2])
            if max_edge <= 0.5:
                triangles = np.vstack((triangles, simplex))
        zFaces = np.ones(triangles.shape[0])
        cmap = colors.LinearSegmentedColormap.from_list(
            "", [(0.8, 0.8, 0.8), "grey", "grey"])
        ax.tripcolor(obs[:, 0], obs[:, 1], triangles, cmap=cmap,
                     facecolors=zFaces, edgecolors='none')

        # print(len(obs_xs))
        # ax.scatter(obs_xs, obs_ys, marker=".", color="black")

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
        ax.grid(True)
        ax.set_xlim(self.state_space.range)
        ax.set_ylim(self.state_space.range)
        plt.ion()
        plt.show()
        plt.pause(0.001)
