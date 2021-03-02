from trimesh.creation import uv_sphere, box
from data.xarm6.xarm6 import Xarm6
from math import pi

obstacles = [box([0.1, 0.1, 0.5]), box([0.1, 0.1, 0.1])]
obstacles[0].apply_translation([0.4, 0, 0.25])
obstacles[1].apply_translation([0.4, 0, 0.8])
robot = Xarm6(obstacles)


for link in robot.robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.robot.actuated_joints))

fk = robot.robot.link_fk()
print(fk[robot.robot.links[0]])

fk = robot.robot.link_fk(cfg={'joint2': 1.0})
print(fk[robot.robot.links[1]])

robot.show(obstacles=obstacles, q=[0, 0, 0, 0, 0, 0] )
print(robot.is_in_collision(q=[0, 0, 0, 0, 0, 0]))
print(robot.distance(q=[0, 0, 0, 0, 0, 0]))

