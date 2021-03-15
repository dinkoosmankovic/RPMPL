import tests.rrt_test as rt
import tests.drrt_test as drt
import tests.ekf_rrt_test as ekf

import statistics
from numpy import mean

# rt.run(animate=True, choice=1)

d = []
t = []
for i in range(0, 100):
    try:
        # choice = 0, 2dof easy
        # choice = 1, 2dof hard
        # choice = 2, xarm easy
        # choice = 3, xarm hard
        min_dist, _, total_time, _, _ = ekf.run(animate=False, choice=0)
        d.append(min_dist)
        t.append(total_time)
    except ValueError:
        continue

print("Time: ", mean(t))
print("Time stdev: ", statistics.stdev(t))
print("Distance: ", mean(d))
print("Distance stdev: ", statistics.stdev(d))

