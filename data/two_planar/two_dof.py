from urdfpy import URDF
import pyrender
from trimesh.creation import box, cylinder
import numpy as np

class TwoDOF:
    def __init__(self) -> None:
        self.robot = URDF.load('data/two_planar/2dof_planar.urdf')

    def robot(self):
        return self.robot

    def get_config(self, q):
        if len(self.robot.actuated_joints) != len(q):
            raise Exception("Wrong dimensions of q")

        cfg = {}    
        for i in range(len(q)):
            cfg[self.robot.actuated_joints[i].name] = q[i]

        return cfg

    def get_link_mesh(self, tm):
        print(tm.visuals[0].material.color)
        init_pose = tm.visuals[0].origin
        if tm.visuals[0].geometry.cylinder is not None:
            length = tm.visuals[0].geometry.cylinder.length
            radius = tm.visuals[0].geometry.cylinder.radius
            mesh = cylinder(radius, length)
            mesh.visual.face_color = tm.visuals[0].material.color
            return init_pose, mesh
        else:
            ext = tm.visuals[0].geometry.box.size
            mesh = box(ext)
            mesh.visual.face_colors = tm.visuals[0].material.color
            return init_pose, mesh



    def show(self, q=None, obstacles=None, use_collision=False):
        cfg = self.get_config(q)
        #print(cfg)
        if use_collision:
            fk = self.robot.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.robot.link_fk(cfg=cfg)

        scene = pyrender.Scene()
        # adding robot to the scene
        for tm in fk:
            pose = fk[tm]
            init_pose, link_mesh = self.get_link_mesh(tm)
            mesh = pyrender.Mesh.from_trimesh(link_mesh, smooth=False)
            scene.add(mesh, pose=np.matmul(pose, init_pose))

        # adding base box to the scene
        table = box([0.5, 0.5, -0.4])
        table.apply_translation([0, 0, -0.015])

        #scene.add(pyrender.Mesh.from_trimesh(table))

        # adding obstacles to the scene
        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        pyrender.Viewer(scene, use_raymond_lighting=True)