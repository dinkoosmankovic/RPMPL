from trimesh.creation import uv_sphere
from data.ur5.ur5 import UR5

ur5 = UR5()
robot = ur5.robot


for link in robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.actuated_joints))

fk = robot.link_fk()
print(fk[robot.links[0]])

fk = robot.link_fk(cfg={'shoulder_pan_joint': 1.0})
print(fk[robot.links[1]])

obstacles = [uv_sphere(0.1)]
obstacles[0].apply_translation([0.4, 0, 0.3])

print(type(obstacles[0]))

ur5.show(obstacles=obstacles, q=[0, -2.0, 2.0, 0, 0, 0] )
