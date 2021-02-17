from rrt.rrt import RRT
import numpy as np
from data.ur5.ur5 import UR5
from math import pi
from trimesh.creation import box
import time
import logging
import threading

pos = 0
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                    datefmt="%H:%M:%S")


def update_obstacles(obstacles):
    global pos
    logging.info("New pos %.2f: ", 1.3 + pos)
    threading.Lock()
    obstacles[0].apply_translation([1.3 + pos, 0, 0])
    time.sleep(0.1)


start = np.array([pi/4, -pi/3, 2.0, 0, 0, 0])
goal = np.array([-pi/4, -pi/3, 2.0, 0, 0, 0])
obstacles = [box([0.1, 0.1, 0.3]), box([0.1, 0.1, 0.1])]
obstacles[0].apply_translation([0.3, 0, 0.15])
obstacles[1].apply_translation([-0.5, 0, 0.1])
robot = UR5(obstacles)

x = threading.Thread(target=update_obstacles, args=(obstacles,))
# robot.show(q=start, obstacles=obstacles)

args = {
    "max_iter": 2000,
    "eps": pi/10.0,
    "gamma": 0.01,
    "state_space": robot.spaces,
    "robot": robot,
    "num_checks": 10
}

planner = RRT(start, goal, args)

# x.start()
res = planner.solve()
#planner.visualize()
if res:
    # planner.visualize()
    path = planner.get_solution_path()
    robot.animate(q_traj=path, obstacles=obstacles)

# planner.update_obstacles( [{"p" : np.array([12, 10]), "r": 2.0 }] )
# planner.visualize()
