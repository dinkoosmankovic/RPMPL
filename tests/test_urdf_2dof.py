from trimesh.creation import box
from data.two_planar.two_dof import TwoDOF
from math import pi
import time


obstacles = [box([0.3, 0.3, 0.3])]
obstacles[0].apply_translation([0.6, 0, 0])

two_dof = TwoDOF(obstacles)
robot = two_dof.robot


for link in robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.actuated_joints))

fk = robot.link_fk()
print(fk[robot.links[0]])

print(type(obstacles[0]))

#two_dof.show(obstacles=obstacles, q=[-pi/4, pi/6] )

ret = two_dof.is_valid(q=[-pi/4, pi/6])
print(ret)

start = time.time()
for i in range(100):
    ret = two_dof.is_valid(q=[0, 0])  

end = 100*(time.time()-start)
print("ms: %s" % end)
