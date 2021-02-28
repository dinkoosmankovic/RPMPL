from rrt.rrt import RRT
from rrt.rrt_rewire import RRTRewire
from math import pi
from trimesh.creation import box
from data.two_planar.two_dof import TwoDOF
from animation.animation import Animation
from animation.myEvent import Event
from animation.myEvent import Key
from animation.localtime import LocalTime
import threading
import logging
import time
import numpy as np
import random
import math

from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise

theta = []
sign_matrix = [[1, 1], [1, -1], [-1, 1], [-1, -1]]


def move_obstacles(obstacles_l, anim):
    global theta, sign_matrix

    d, r = 0.3, 0.2
    c = 1.0
    # move obstacles at constant speed relative to FPS
    with threading.Lock():
        for i in range(0, len(obstacles_l)):
            theta[i] += 2
            if theta[i] > 360:
                theta[i] -= 360
            t_obstacle = box([d, d, d])
            s1 = sign_matrix[i][0]
            s2 = sign_matrix[i][1]
            t_obstacle.apply_translation([c * s1 + r * math.cos(math.radians(theta[i])),
                                              c * s2 + r * math.sin(math.radians(theta[i])), 0])
            obstacles_l[i] = t_obstacle
        ev = Event(Event.ENV_CHANGE)
        anim.events.append(ev)


def move_thread(obstacles_l, anim):
    while True:
        move_obstacles(obstacles_l, anim)
        time.sleep(0.1)


def single_move_obstacle(obstacles_l, direction):
    # move all objects in specified direction
    # custom move to test manipulator execution
    with threading.Lock():
        obstacles_l[0].apply_translation([direction[0], direction[1], 0])


def fix_path(path, max_q=1.0):
    new_path = []
    for i in range(0, len(path)-1):
        new_path.append(path[i])
        curr = path[i]
        while np.linalg.norm(path[i+1]-curr) > max_q:
            vecdir = path[i+1] - path[i]
            nor = np.linalg.norm(vecdir)
            vecdir = [q / nor * max_q for q in vecdir]
            curr = curr + vecdir
            new_path.append(curr)
    new_path.append(path[-1])
    return new_path


def get_3D_tracker(difft):
    R_std = 0.1
    Q_std = 0.05

    tracker = KalmanFilter(dim_x=6, dim_z=3)
    dt = difft

    tracker.F = np.array([[1, dt, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, dt, 0, 0],
                         [0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 1, dt],
                         [0, 0, 0, 0, 0, 1]])
    tracker.u = 0.
    tracker.H = np.array([[1., 0, 0, 0, 0, 0],
                         [0, 0, 1., 0, 0, 0],
                         [0, 0, 0, 0, 1., 0]])
    tracker.R = np.eye(3) * R_std**2
    q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_std**2)
    tracker.Q = block_diag(q, q)
    tracker.x = np.array([[0, 0, 0, 0, 0, 0]]).T
    tracker.P = np.eye(6) * 500.
    tracker.B = 0.
    return tracker


def get_predictions_box(dim, trackers):
    global obstacles
    pred_box = []

    for i, tr in enumerate(trackers):
        tr.update(obstacles[i].center_mass)
        pr, _ = tr.get_prediction()
        pred_box.append(box(dim[i]))
        pred_box[i].apply_translation([pr[0][0], pr[2][0], pr[4][0]])
    return pred_box

_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=_format, level=logging.INFO,
                    datefmt="%H:%M:%S")

start = np.array([-pi/2, 0])
goal = np.array([pi/2, 0])
d = 0.3
dim_box = np.array([[d, d, d], [d, d, d],
                    [d, d, d], [d, d, d]])

obstacles = [box(dim_box[i]) for i in range(0, 4)]

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

# example: two dof robot so the animation is relatively simple
robot = TwoDOF(obstacles)
robot.update_predictions(obstacles)
robot.start_config = start
eps = pi/10.0
args = {
    "max_iter": 1000,
    "eps": pi/10.0,
    "gamma": 0.2,
    "state_space": robot.spaces,
    "robot": robot,
    "num_checks": 10
}

ms = 50

trackers = [get_3D_tracker(2*ms/1000) for i in range(0, len(obstacles))]
for i in range(0, len(obstacles)):
    c = obstacles[i].center_mass
    x = c.T
    trackers[i].predict()
    trackers[i].update(x)

anim = Animation(robot, obstacles)
t = LocalTime()
moveThread = threading.Thread(target=move_thread, args=(obstacles, anim, ))
moveThread.start()

with threading.Lock():
    planner = RRTRewire(start, goal, args)
    res = planner.solve()
    if res:
        path = planner.get_solution_path()
        path = path[::-1]
        path.append(goal)

        path = fix_path(path, max_q=0.05)
        anim.robot.set_trajectory(path)

i = 0
while True:
    # pick up events
    predictions = get_predictions_box(dim=dim_box, trackers=trackers)
    i += 1
    # if i % 50 == 0:
    #     print("visualize")
    #     planner.visualize()

    # for i, ob in enumerate(obstacles):
    #     print("obstacle i: ", i , " at: ", ob.center_mass,
    #           " prediction at: ", predictions[i].center_mass)

    for event in anim.events:
        # if there was autonomous obstacle movement, do the re-planning
        if event.type == Event.ENV_CHANGE:
            # update environment for collision
            anim.robot.update_env(obstacles)
            anim.robot.update_predictions(predictions)
            start = anim.robot.curr_q

            if np.linalg.norm(goal - start) > eps:
                # planner = RRT(start, goal, args)
                with threading.Lock():
                    res = planner.resolve_with_check(start)
                    if res:
                        # planner.visualize()
                        path = planner.get_solution_path()
                        path = path[::-1]
                        path.append(goal)

                        path = fix_path(path, max_q=0.05)
                        anim.robot.set_trajectory(path)

        # if there was pressed key, move the obstacle man>ally
        # *NOTE* STILL NOT IMPLEMENTED
        elif event.type == Event.KEY_PRESSED:
            if event.key == Key.UP:
                single_move_obstacle(obstacles, [0, 1], anim)
            elif event.key == Key.RIGHT:
                single_move_obstacle(obstacles, [1, 0], anim)
            elif event.key == Key.LEFT:
                single_move_obstacle(obstacles, [-1, 0], anim)
            elif event.key == Key.DOWN:
                single_move_obstacle(obstacles, [0, -1], anim)
    anim.events.clear()

    # update scene - obstacles
    scene_thread = threading.Thread(target=anim.update_scene, args=())
    scene_thread.start()

    # update scene - manipulator
    robot_thread = threading.Thread(target=anim.update_robot, args=())
    robot_thread.start()

    scene_thread.join()
    robot_thread.join()
    planner.update_path()

    # time advance - miliseconds
    # time.sleep(0.5)
    t.tick(ms)
