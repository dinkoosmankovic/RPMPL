from urdfpy import URDF
import pyrender
from trimesh.creation import box, cylinder
from state_spaces.real_vector_space import RealVectorSpace
from trimesh.creation import box, cylinder
from trimesh.collision import CollisionManager
import numpy as np


class Xarm6(RealVectorSpace):
    def __init__(self, obstacles) -> None:
        self.robot = URDF.load('data/xarm6/xarm6.urdf')
        self.spaces = RealVectorSpace(6)
        self.robot_cm = CollisionManager()
        self.env_cm = CollisionManager()
        cfg = self.get_config([0, 0, 0, 0, 0, 0])
        self.init_poses = [0 for i in range(6)]
        #print(cfg)
        fk = self.robot.collision_trimesh_fk(cfg=cfg)
        #fk = self.robot.visual_trimesh_fk(cfg=cfg)

        # adding robot to the scene
        for i, tm in enumerate(fk):
            pose = fk[tm]
            name = "link_" + str(i+1) 
            self.init_poses.append(pose)
            self.robot_cm.add_object(name, tm, pose)
        
        table = cylinder(radius=0.7, height=0.02)
        table.apply_translation([0, 0, -0.015])
        obstacles.append(table)
        for i, ob in enumerate(obstacles):
            self.env_cm.add_object("obstacle_" + str(i), ob)

    def spaces(self):
        return self.spaces

    def robot(self):
        return self.robot

    def is_in_collision(self, q):
        cfg = self.get_config(q)
        fk = self.robot.collision_trimesh_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            pose = fk[tm]
            self.robot_cm.set_transform("link_"+ str(i+1), pose)

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        return self.robot_cm.in_collision_other(self.env_cm)

    def is_valid(self, q, qe=None, num_checks=None):
        if qe is None or num_checks is None:
            res = self.is_in_collision(q)
            return not(res)
        else:
            for i in range(num_checks):
                q_i = self.get_qnew(q, qe, i/num_checks)
                res = self.is_in_collision(q_i)
                if res:
                    return False
        return True

    def get_config(self, q):
        if len(self.robot.actuated_joints) != len(q):
            raise Exception("Wrong dimensions of q")

        cfg = {}
        for i in range(len(q)):
            cfg[self.robot.actuated_joints[i].name] = q[i]

        return cfg

    def get_link_mesh(self, tm):
        print(tm.visuals)
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
            fk = self.robot.visual_trimesh_fk(cfg=cfg)

        scene = pyrender.Scene()
        # adding robot to the scene
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            scene.add(mesh, pose=pose)

        # adding base box to the scene
        table = cylinder(radius=0.7, height=0.02) #([0.5, 0.5, 0.02])
        table.apply_translation([0, 0, -0.015])
        table.visual.vertex_colors = [205, 243, 8, 255]

        scene.add(pyrender.Mesh.from_trimesh(table))

        # adding obstacles to the scene
        for i, ob in enumerate(obstacles):
            if i < len(obstacles)-1:
                ob.visual.vertex_colors = [255, 0, 0, 255]
            else:
                ob.visual.vertex_colors = [205, 243, 8, 255]
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        pyrender.Viewer(scene, use_raymond_lighting=True)

    def animate(self, q_traj=None, obstacles=None):
        import time
        fps = 10.0
        cfgs = [self.get_config(q) for q in q_traj]

        # Create the scene
        fk = self.robot.visual_trimesh_fk(cfg=cfgs[0])

        node_map = {}
        init_pose_map = {}
        scene = pyrender.Scene()
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            node = scene.add(mesh, pose=pose)
            node_map[tm] = node

        # adding base box to the scene
        #table = cylinder(radius=0.7, height=0.02) #([0.5, 0.5, 0.02])
        #table.apply_translation([0, 0, -0.015])
        #table.visual.vertex_colors = [205, 243, 8, 255]
        #scene.add(pyrender.Mesh.from_trimesh(table))

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = 2.5
        scene.add(cam, pose=init_cam_pose)

        # adding obstacles to the scene
        for i, ob in enumerate(obstacles):
            if i < len(obstacles)-1:
                ob.visual.vertex_colors = [255, 0, 0, 255]
            else:
                ob.visual.vertex_colors = [205, 243, 8, 255]
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        # Pop the visualizer asynchronously
        v = pyrender.Viewer(scene, run_in_thread=True,
                            use_raymond_lighting=True)
        time.sleep(1.0)
        # Now, run our loop
        i = 0
        while v.is_active:
            cfg = cfgs[i]
            fk = self.robot.visual_trimesh_fk(cfg=cfg)
            if i < len(cfgs) - 1:
                i += 1
            else:
                i = 0
                time.sleep(1.0)
            v.render_lock.acquire()
            for mesh in fk:
                pose = fk[mesh]
                node_map[mesh].matrix = pose
            v.render_lock.release()
            time.sleep(1.0 / fps)