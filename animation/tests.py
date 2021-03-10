from math import pi
from trimesh.creation import box, cylinder
from data.two_planar.two_dof import TwoDOF
from animation.animation import Animation
from animation.myEvent import Event
from animation.localtime import LocalTime
from timeit import default_timer as timer
import threading
import logging
import time
import numpy as np
import math
import random


def easy_obstacles():
    start = np.array([-pi / 4, pi / 4])
    goal = np.array([pi / 4, -pi / 4])
    dim_box = np.array([[0.3, 0.6, 0.3], [0.3, 0.3, 0.3]])

    obstacles = [box([0.3, 0.6, 0.3]), box([0.3, 0.3, 0.3])]
    obstacles[0].apply_translation([1.4, 0, 0])
    obstacles[1].apply_translation([-0.5, 0, 0])
    return start, goal, dim_box, obstacles


def hard_obstacles():
    sign_matrix = [[1, 1], [1, -1], [-1, 1], [-1, -1]]

    start = np.array([-pi / 2, 0])
    goal = np.array([pi / 2, 0])
    d = 0.3
    dim_box = np.array([[d, d, d], [d, d, d],
                        [d, d, d], [d, d, d]])

    obstacles = [box(dim_box[i]) for i in range(0, 4)]

    theta = []
    c = 1.0
    for i in range(0, 4):
        random.seed()
        theta.append(random.randint(0, 360))
    r = 0.2

    for i in range(0, 4):
        s1 = sign_matrix[i][0]
        s2 = sign_matrix[i][1]

        obstacles[i].apply_translation([c * s1 + r * math.cos(math.radians(theta[i])),
                                        c * s2 + r * math.sin(math.radians(theta[i])), 0])
    return start, goal, dim_box, obstacles, theta


def xarm_easy_obstacles():
    random.seed()
    x = random.randrange(300, 500)
    x /= 1000

    start = np.array([-pi / 4, pi / 2.5, -pi / 1.5, pi / 2, 0, 0])
    goal = np.array([pi / 4, pi / 2.5, -pi / 1.5, pi / 2, 0, 0])

    dim_box = np.array([[0.1, 0.1, 0.5]])
    obstacles = [box(dim_box[0])]
    obstacles[0].apply_translation([x, 0, 0.25])

    table = cylinder(radius=0.7, height=0.02)
    table.apply_translation([0, 0, -0.015])
    obstacles.append(table)

    return start, goal, dim_box, obstacles, x


def xarm_hard_obstacles():
    x, z = 0.6, 1.
    start = np.array([-pi / 4, pi / 4, -pi / 2, 0, 0, 0])
    goal = np.array([pi / 4, pi / 4, -pi / 2, 0, 0, 0])

    dim_box = [[0.1, 0.1, 0.5], [0.1, 0.1, 0.1],
               [0.1, 0.6, 0.6], [0.1, 0.3, 0.1]]

    obstacles = [box(dim_box[i]) for i in range(0, 4)]
    obstacles[0].apply_translation([x, 0, 0.25])
    obstacles[1].apply_translation([0.3, 0, z])
    obstacles[2].apply_translation([-0.2, 0, 0.3])
    obstacles[3].apply_translation([x, 0, 0.5])

    table = cylinder(radius=0.7, height=0.02)
    table.apply_translation([0, 0, -0.015])
    obstacles.append(table)

    return start, goal, dim_box, obstacles, x, z
