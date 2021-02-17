from trimesh.creation import uv_sphere, box
from data.ur5.ur5 import UR5
from math import pi

obstacles = [box([0.1, 0.1, 0.3]), box([0.1, 0.1, 0.1])]
obstacles[0].apply_translation([0.3, 0, 0.15])
obstacles[1].apply_translation([-0.5, 0, 0.1])
robot = UR5(obstacles)


for link in robot.robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.robot.actuated_joints))

fk = robot.robot.link_fk()
print(fk[robot.robot.links[0]])

fk = robot.robot.link_fk(cfg={'shoulder_pan_joint': 1.0})
print(fk[robot.robot.links[1]])

robot.show(obstacles=obstacles, q=[pi/4, -pi/3, 2.0, 0, 0, 0] )
print(robot.is_in_collision([0, -2.0, 2.0, 0, 0, 0]))
