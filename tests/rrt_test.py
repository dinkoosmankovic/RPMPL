from math import pi
from trimesh.creation import box
from data.two_planar.two_dof import TwoDOF
from data.xarm6.xarm6 import Xarm6
from animation.animation import Animation, AnimationXarm
from animation.myEvent import Event
from animation.localtime import LocalTime
from timeit import default_timer as timer
import rrt_v2.rrtRe as rrtRe
import threading
import logging
import time
import numpy as np
import math
import animation.tests as at
from rrt_v2.node import Node


def move_ob_easy(ob, anim):
    try:
        move_ob_easy.i += 1
        if move_ob_easy.i > 30:
            move_ob_easy.i = 0
            move_ob_easy.sign *= -1
    except AttributeError:
        move_ob_easy.i = 15
        move_ob_easy.sign = 1

    s = move_ob_easy.sign
    with threading.Lock():
        ob[0].apply_translation([0.01*s, 0., 0.])
        if move_ob_easy.i % 2 == 0:
            ev = Event(Event.ENV_CHANGE)
            anim.events.append(ev)


def move_ob_hard(ob, anim, theta):
    # fix to retrieve from tests.py
    d, r = 0.3, 0.2
    c = 1.0
    sign_matrix = [[1, 1], [1, -1], [-1, 1], [-1, -1]]

    with threading.Lock():
        for i in range(0, len(ob)):
            theta[i] += 2
            if theta[i] > 360:
                theta[i] -= 360
            t_ob = box([d, d, d])
            s1 = sign_matrix[i][0]
            s2 = sign_matrix[i][1]
            t_ob.apply_translation([c*s1 + r*math.cos(math.radians(theta[i])),
                                    c*s2 + r*math.sin(math.radians(theta[i])), 0])
            ob[i] = t_ob
        ev = Event(Event.ENV_CHANGE)
        anim.events.append(ev)


def move_ob_easy_xarm(ob, anim):
    with threading.Lock():
        if move_ob_easy_xarm.x < 0.3:
            move_ob_easy_xarm.sign = 1
        elif move_ob_easy_xarm.x > 0.5:
            move_ob_easy_xarm.sign = -1
        move_ob_easy_xarm.x += move_ob_easy_xarm.sign * 0.01

        dim_box = np.array([[0.1, 0.1, 0.5]])
        t_obstacles = box(dim_box[0])
        t_obstacles.apply_translation([move_ob_easy_xarm.x, 0, 0.25])
        ob[0] = t_obstacles
        ev = Event(Event.ENV_CHANGE)
        anim.events.append(ev)


def move_ob_hard_xarm(ob, anim):
    x = move_ob_hard_xarm.x
    z = move_ob_hard_xarm.z
    sign_x = move_ob_hard_xarm.sign_x
    sign_z = move_ob_hard_xarm.sign_z

    with threading.Lock():
        if move_ob_hard_xarm.x < 0.4:
            move_ob_hard_xarm.sign_x = 1
        elif move_ob_hard_xarm.x > 0.5:
            move_ob_hard_xarm.sign_x = -1
        if move_ob_hard_xarm.z < 0.7:
            move_ob_hard_xarm.sign_z = 1
        elif move_ob_hard_xarm.z > 1:
            move_ob_hard_xarm.sign_z = -1
        move_ob_hard_xarm.x += move_ob_hard_xarm.sign_x * 0.01
        move_ob_hard_xarm.z += move_ob_hard_xarm.sign_z * 0.01

        dim_box = [[0.1, 0.1, 0.5], [0.1, 0.1, 0.1],
                   [0.1, 0.6, 0.6], [0.1, 0.3, 0.1]]

        tobstacles = [box(dim_box[i]) for i in range(0, 4)]
        tobstacles[0].apply_translation([move_ob_hard_xarm.x, 0, 0.25])
        tobstacles[1].apply_translation([0.3, 0, move_ob_hard_xarm.z])
        tobstacles[2].apply_translation([-0.2, 0, 0.3])
        tobstacles[3].apply_translation([move_ob_hard_xarm.x, 0, 0.5])

        ob[0:4] = tobstacles[0:4]

        ev = Event(Event.ENV_CHANGE)
        anim.events.append(ev)


def move_thread(ob, anim, finish_ev, pause_ev, choice, theta=None, x=0, z=0):
    while True:
        if finish_ev.isSet():
            return
        if not pause_ev.isSet():
            if choice == 0:  # easy 2dof
                move_ob_easy(ob, anim)
            elif choice == 1:  # hard 2dof
                move_ob_hard(ob, anim, theta)
            elif choice == 2:  # easy xarm
                move_ob_easy_xarm(ob, anim)
            elif choice == 3:  # hard xarm
                move_ob_hard_xarm(ob, anim)
        time.sleep(0.3)


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


def run(run_number=0, animate=False, choice=0, ms=50):
    Node.count = 0
    _format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=_format, level=logging.INFO,
                        datefmt="%H:%M:%S")
    logging.info("Starting...")

    if choice == 0:
        start, goal, dim_box, obstacles = at.easy_obstacles()
    elif choice == 1:
        start, goal, dim_box, obstacles, theta = at.hard_obstacles()
    elif choice == 2:
        start, goal, dim_box, obstacles, x = at.xarm_easy_obstacles()
        move_ob_easy_xarm.x = x
        move_ob_easy_xarm.sign = -1
    else:
        start, goal, dim_box, obstacles, x, z = at.xarm_hard_obstacles()
        move_ob_hard_xarm.x = x
        move_ob_hard_xarm.z = z
        move_ob_hard_xarm.sign_x = -1
        move_ob_hard_xarm.sign_z = -1

    if choice == 0 or choice == 1:
        robot = TwoDOF(obstacles)
    else:
        robot = Xarm6(obstacles)

    robot.start_config = start
    eps = pi/5.0
    args = {
        "max_iter": 15000,
        "eps": eps,
        "gamma": 0.15,
        "state_space": robot.spaces,
        "robot": robot,
        "num_checks": 20
    }

    if choice == 0 or choice == 1:
        anim = Animation(robot, obstacles, do_anim=animate)
    else:
        anim = AnimationXarm(robot, obstacles, do_anim=animate)

    t = LocalTime()

    finish_ev = threading.Event()
    pause_ev = threading.Event()
    if choice == 0:
        moveThread = threading.Thread(target=move_thread, args=(obstacles, anim,
                                                                finish_ev, pause_ev,
                                                                choice))
    elif choice == 1:
        moveThread = threading.Thread(target=move_thread, args=(obstacles, anim,
                                                                finish_ev, pause_ev,
                                                                choice, theta))
    elif choice == 2:
        moveThread = threading.Thread(target=move_thread, args=(obstacles, anim,
                                                                finish_ev, pause_ev,
                                                                choice))
    else:
        moveThread = threading.Thread(target=move_thread, args=(obstacles, anim,
                                                                finish_ev, pause_ev,
                                                                choice))

    total_nodes = 0
    initial_planning = 0.0

    with threading.Lock():
        t_s = timer()
        planner = rrtRe.RRTre(start, goal, args)
        res = planner.solve()
        if res:
            path = planner.get_solution_path()
            path = path[::-1]
            path.append(goal)

            path = fix_path(path, max_q=0.1)
            anim.robot.set_trajectory(path)
        else:
            logging.info("Could not find RRT solution in time.")
            return None, None, None, -2

        total_nodes += len(planner.nodes)
        initial_planning = timer() - t_s

    moveThread.start()
    i = 0
    check = True
    t_start = timer()
    replan_counter = 0
    total_time = 0

    while check:
        if timer() - t_start > 30:
            logging.info("Stuck.")
            finish_ev.set()
            return replan_counter, total_time, initial_planning, -1
        i += 1

        for event in anim.events:
            if event.type == Event.ENV_CHANGE:
                anim.robot.update_env(obstacles)
                start = anim.robot.curr_q

                pause_ev.set()
                if np.linalg.norm(goal - start) > eps:
                    planner.start = start
                    res, diff = planner.replan()
                    total_time += diff
                    if res:
                        total_nodes += len(planner.nodes)
                        replan_counter += 1
                        path = planner.get_solution_path()
                        path = path[::-1]
                        path.append(goal)
                        path = fix_path(path, max_q = 0.1)
                        anim.robot.set_trajectory(path)
                else:
                    check = False
                pause_ev.clear()
        anim.events.clear()

        scene_thread = threading.Thread(target=anim.update_scene, args=())
        scene_thread.start()

        robot_thread = threading.Thread(target=anim.update_robot, args=())
        robot_thread.start()

        scene_thread.join()
        robot_thread.join()
        planner.update_path()
        t.tick(ms)

    logging.info("Done: " + str(run_number))
    finish_ev.set()
    return replan_counter, total_time, initial_planning, Node.count
