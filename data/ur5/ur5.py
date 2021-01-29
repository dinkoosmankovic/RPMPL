from urdfpy import URDF
import pyrender
from trimesh.creation import box

class UR5:
    def __init__(self) -> None:
        self.robot = URDF.load('data/ur5/ur5.urdf')

    def robot(self):
        return self.robot

    def get_config(self, q):
        if len(self.robot.actuated_joints) != len(q):
            raise Exception("Wrong dimensions of q")

        cfg = {}    
        for i in range(len(q)):
            cfg[self.robot.actuated_joints[i].name] = q[i]

        return cfg


    def show(self, q=None, obstacles=None, use_collision=False):
        cfg = self.get_config(q)
        #print(cfg)
        if use_collision:
            fk = self.robot.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.robot.visual_trimesh_fk(cfg=cfg)

        scene = pyrender.Scene()
        # adding robot to the scene
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            scene.add(mesh, pose=pose)

        # adding base box to the scene
        table = box([0.5, 0.5, 0.02])
        table.apply_translation([0, 0, -0.015])

        scene.add(pyrender.Mesh.from_trimesh(table))

        # adding obstacles to the scene
        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        pyrender.Viewer(scene, use_raymond_lighting=True)
