from trimesh.creation import uv_sphere
from data.two_planar.two_dof import TwoDOF
from math import pi

two_dof = TwoDOF()
robot = two_dof.robot


for link in robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.actuated_joints))

fk = robot.link_fk()
print(fk[robot.links[0]])

obstacles = [uv_sphere(0.1)]
obstacles[0].apply_translation([0.6, 0, 0])

print(type(obstacles[0]))

two_dof.show(obstacles=obstacles, q=[pi/4, pi/6] )
