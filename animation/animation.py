import pyrender
from state_spaces.real_vector_space import RealVectorSpace
from trimesh.creation import box, cylinder
from trimesh.collision import CollisionManager
import numpy as np


class Animation:
    def __init__(self, robot, obstacles, do_anim=True) -> None:
        self.robot = robot
        self.obstacles = obstacles

        self.spaces = RealVectorSpace(2)
        self.scene = pyrender.Scene()

        self.do_anim = do_anim

        if do_anim:
            cam = pyrender.PerspectiveCamera(yfov=np.pi/3.0, aspectRatio=1.414)
            init_cam_pose = np.eye(4)
            init_cam_pose[2, 3] = 2.5
            self.scene.add(cam, pose=init_cam_pose)

            self.viewer = pyrender.Viewer(self.scene, run_in_thread=True,
                                          use_raymond_lighting=True)

            # add obstacles to scene
            self.ob_map = {}
            c = 0
            for ob in obstacles:
                self.viewer.render_lock.acquire()
                node = self.scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))
                self.ob_map[c] = node
                self.viewer.render_lock.release()
                c += 1

            # add robot to scene
            cfg = self.robot.get_config(self.robot.start_config)
            fk = self.robot.robot.link_fk(cfg=cfg)

            self.node_map = {}
            self.init_pose_map = {}
            for i, tm in enumerate(fk):
                self.viewer.render_lock.acquire()
                pose = fk[tm]
                init_pose, link_mesh = self.robot.get_link_mesh(tm)
                mesh = pyrender.Mesh.from_trimesh(link_mesh, smooth=False)
                node = self.scene.add(mesh, pose=np.matmul(pose, init_pose))
                self.node_map[tm] = node
                self.init_pose_map[tm] = init_pose
                self.viewer.render_lock.release()

        # create event array
        self.events = []

    # update obstacles in scene
    def update_scene(self):
        if not self.do_anim:
            return
        c = 0
        for ob in self.obstacles:
            self.viewer.render_lock.acquire()
            self.scene.remove_node(self.ob_map[c])
            self.ob_map[c] = self.scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))
            self.viewer.render_lock.release()
            c += 1

    def update_robot(self):
        cfg = self.robot.get_next_q()
        if not self.do_anim:
            return

        if cfg is None:
            return
        cfg = self.robot.get_config(cfg)
        fk = self.robot.robot.link_fk(cfg=cfg)
        self.viewer.render_lock.acquire()
        for mesh in fk:
            pose = fk[mesh]
            self.node_map[mesh].matrix = np.matmul(pose, self.init_pose_map[mesh])
        self.viewer.render_lock.release()


class AnimationXarm():
    def __init__(self, robot, obstacles, do_anim=True) -> None:
        self.do_anim = do_anim
        self.robot = robot
        self.obstacles = obstacles

        self.spaces = RealVectorSpace(2)
        self.scene = pyrender.Scene()

        if do_anim:
            cam = pyrender.PerspectiveCamera(yfov=np.pi/3.0, aspectRatio=1.414)
            init_cam_pose = np.eye(4)
            init_cam_pose[2, 3] = 2.5
            self.scene.add(cam, pose=init_cam_pose)

            self.viewer = pyrender.Viewer(self.scene, run_in_thread=True,
                                          use_raymond_lighting=True)

            # add obstacles to scene
            self.ob_map = {}
            c = 0
            for ob in obstacles:
                self.viewer.render_lock.acquire()
                node = self.scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))
                self.ob_map[c] = node
                self.viewer.render_lock.release()
                c += 1

            # add robot to scene
            cfg = self.robot.get_config(self.robot.start_config)
            # fk = self.robot.robot.link_fk(cfg=cfg)
            fk = self.robot.robot.visual_trimesh_fk(cfg=cfg)

            self.node_map = {}
            self.init_pose_map = {}
            for i, tm in enumerate(fk):
                self.viewer.render_lock.acquire()
                pose = fk[tm]
                mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
                node = self.scene.add(mesh, pose=pose)
                self.node_map[tm] = node
                self.viewer.render_lock.release()

        # create event array
        self.events = []

    # update obstacles in scene
    def update_scene(self):
        if not self.do_anim:
            return
        c = 0
        for ob in self.obstacles:
            self.viewer.render_lock.acquire()
            self.scene.remove_node(self.ob_map[c])
            self.ob_map[c] = self.scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))
            self.viewer.render_lock.release()
            c += 1

    def update_robot(self):
        cfg = self.robot.get_next_q()
        if not self.do_anim:
            return
        if cfg is None:
            return
        cfg = self.robot.get_config(cfg)
        fk = self.robot.robot.visual_trimesh_fk(cfg=cfg)
        self.viewer.render_lock.acquire()
        for mesh in fk:
            pose = fk[mesh]
            self.node_map[mesh].matrix = pose
        self.viewer.render_lock.release()
