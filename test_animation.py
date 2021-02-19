from rrt.rrt import RRT
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


def move_obstacles(obstacles_l, anim):
    # loop through pre-defined movement (right-left)
    try:
        move_obstacles.i += 1
        if move_obstacles.i > 50:
            move_obstacles.i = 0
            move_obstacles.sign *= -1
    except AttributeError:
        move_obstacles.i = 0
        move_obstacles.sign = -1

    s = move_obstacles.sign
    # move obstacles at constant speed relative to FPS
    with threading.Lock():
        obstacles_l[0].apply_translation([0.005 * s, 0, 0])
        if move_obstacles.i % 5 == 0:
            ev = Event(Event.ENV_CHANGE)
            anim.events.append(ev)


def move_thread(obstacles_l, anim):
    while True:
        move_obstacles(obstacles_l, anim)
        time.sleep(0.2)


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

_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=_format, level=logging.INFO,
                    datefmt="%H:%M:%S")

start = np.array([-pi/4, pi/4])
goal = np.array([pi/4, -pi/4])
obstacles = [box([0.3, 0.6, 0.3]), box([0.3, 0.3, 0.3])]
obstacles[0].apply_translation([1.3, 0, 0])
obstacles[1].apply_translation([-0.5, 0, 0])

# example: two dof robot so the animation is relatively simple
robot = TwoDOF(obstacles)
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

# specify Animation
# - collect robot information
# - collect obstacles information
# - create scene inside Animation rather than inside of robot

anim = Animation(robot, obstacles)
t = LocalTime()
moveThread = threading.Thread(target=move_thread, args=(obstacles, anim, ))
moveThread.start()

with threading.Lock():
    planner = RRT(start, goal, args)
    res = planner.solve()
    if res:
        path = planner.get_solution_path()
        path = path[::-1]
        path.append(goal)

        path = fix_path(path, max_q=0.05)
        anim.robot.set_trajectory(path)

while True:
    # pick up events
    for event in anim.events:
        # if there was autonomous obstacle movement, do the re-planning
        if event.type == Event.ENV_CHANGE:
            # update enviroment for collision
            anim.robot.update_env(obstacles)
            start = anim.robot.curr_q

            if np.linalg.norm(goal - start) > eps:
                planner = RRT(start, goal, args)
                res = planner.solve()
                if res:
                    path = planner.get_solution_path()
                    path = path[::-1]
                    path.append(goal)

                    path = fix_path(path, max_q=0.05)
                    anim.robot.set_trajectory(path)
            anim.events.clear()

        # if there was pressed key, move the obstacle manually
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

    # update scene - obstacles
    scene_thread = threading.Thread(target=anim.update_scene, args=())
    scene_thread.start()

    # update scene - manipulator
    robot_thread = threading.Thread(target=anim.update_robot, args=())
    robot_thread.start()

    scene_thread.join()
    robot_thread.join()

    # time advance - miliseconds
    # time.sleep(0.5)
    t.tick(50)