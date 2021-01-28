import pyrender
from trimesh.creation import box


def get_config(robot, q):
    if len(robot.actuated_joints) != len(q):
        raise "Wrong dimensions of q"

    cfg = {}    
    for i in range(len(q)):
        cfg[robot.actuated_joints[i].name] = q[i]

    return cfg


def show(robot, q=None, obstacles=None, use_collision=False):
    cfg = get_config(robot, q)
    if use_collision:
        fk = robot.collision_trimesh_fk(cfg=cfg)
    else:
        fk = robot.visual_trimesh_fk(cfg=cfg)

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
