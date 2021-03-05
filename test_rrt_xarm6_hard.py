from rrt.rrt import RRT
import numpy as np
from rrt.rrt_rewire import RRTRewire
from data.xarm6.xarm6 import Xarm6
from math import pi
from trimesh.creation import box, cylinder
from animation.animation import Animation, AnimationXarm
from animation.myEvent import Event, Key
from animation.localtime import LocalTime
import time
import logging
import threading

from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise


pos = 0
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                    datefmt="%H:%M:%S")

x = 0.6
z = 1.

sign_x = -1
sign_z = -1


def move_obstacles(obstacles_l, anim):
    global x, z, sign_x, sign_z
    with threading.Lock():
        if x < 0.4:
            sign_x = 1
        elif x > 0.5:
            sign_x = -1
        if z < 0.7:
            sign_z = 1
        elif z > 1:
            sign_z = -1
        x += sign_x * 0.01
        z += sign_z * 0.01

        dim_box = [[0.1, 0.1, 0.5], [0.1, 0.1, 0.1],
                   [0.1, 0.6, 0.6], [0.1, 0.3, 0.1]]

        tobstacles = [box(dim_box[i]) for i in range(0, 4)]
        tobstacles[0].apply_translation([x, 0, 0.25])
        tobstacles[1].apply_translation([0.3, 0, z])
        tobstacles[2].apply_translation([-0.2, 0, 0.3])
        tobstacles[3].apply_translation([x, 0, 0.5])

        obstacles_l[0:4] = tobstacles[0:4]

        ev = Event(Event.ENV_CHANGE)
        anim.events.append(ev)


def move_thread(obstacles_l, anim):
    while True:
        move_obstacles(obstacles_l, anim)
        time.sleep(0.5)


def fix_path(path, max_q=1.0):
    if len(path) == 0:
        return []
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
                         [0, 0, 0, 1, 0, 0],
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


start = np.array([-pi/4, pi/4, -pi/2, 0, 0, 0])
goal = np.array([pi/4, pi/4, -pi/2, 0, 0, 0])

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

robot = Xarm6(obstacles)
robot.update_predictions(obstacles[:-1])
robot.start_config = start

eps = pi/5.0
args = {
    "max_iter": 5000,
    "eps": eps,
    "gamma": 0.15,
    "state_space": robot.spaces,
    "robot": robot,
    "num_checks": 10
}

ms = 100

trackers = [get_3D_tracker(2*ms/1000) for i in range(0, len(obstacles)-1)]

for i in range(0, len(obstacles)-1):
    c = obstacles[i].center_mass
    xx = c.T
    trackers[i].predict()
    trackers[i].update(xx)

anim = AnimationXarm(robot, obstacles)
t = LocalTime()
moveThread = threading.Thread(target=move_thread, args = (obstacles, anim))
# moveThread.start()

with threading.Lock():
    planner = RRTRewire(start, goal, args)
    res = planner.solve()
    if res:
        print("found path")
        path = planner.get_solution_path()
        path = path[::-1]
        path = fix_path(path, max_q =0.05)
        anim.robot.set_trajectory(path)

moveThread.start()
i = 0
check = True
while check:
    predictions = get_predictions_box(dim=dim_box, trackers=trackers)
    i += 1

    for event in anim.events:
        if event.type == Event.ENV_CHANGE:
            anim.robot.update_env(obstacles)
            anim.robot.update_predictions(predictions)
            start = anim.robot.curr_q

            print(np.linalg.norm(goal-start))

            if np.linalg.norm(goal - start) > eps:
                with threading.Lock():
                    # res = planner.resolve_with_check(start)
                    res = planner.resolve_with_rewire(start)
                    if res:
                        path = planner.get_solution_path()
                        path = path[::-1]
                        path = fix_path(path, max_q=0.05)
                        anim.robot.set_trajectory(path)
            else:
                check = False
    anim.events.clear()

    scene_thread = threading.Thread(target=anim.update_scene, args=())
    scene_thread.start()

    robot_thread = threading.Thread(target=anim.update_robot, args=())
    robot_thread.start()

    scene_thread.join()
    robot_thread.join()
    planner.update_path()

    t.tick(ms)
logging.info("Done.")