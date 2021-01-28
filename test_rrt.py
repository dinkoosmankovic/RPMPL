from rrt.node import Node
from rrt.rrt import RRT
import numpy as np

start = np.array([2, 2])
goal = np.array([20, 20])

args = {
    "obstacles" : [ {"p" : np.array([10, 10]), "r": 2.0 } ]
}

planner = RRT(start, goal, args)

res = planner.solve()

print(res, planner)
planner.visualize()

#planner.update_obstacles( [{"p" : np.array([12, 10]), "r": 2.0 }] )
#planner.visualize()