from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import sys
sys.path.insert(0, sys.path.insert(0, "/isaac-sim/y/envs/gsplat/lib/python3.10/site-packages"))

from omni.isaac.core import World
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera

import numpy as np

from ow_robot import OwlBot
from gsplat_manager import IsaacSimGSplatManager
from controllers import TeleopIKController

from utils import _get_robot_link_transforms

BASE_QUAT = [1, 0, 0, 0]
BASE_POS = [0, 0, 0]
ROBOT_LINK_PLYS = [
    "/isaac-sim/assets/base_link.ply",
    "/isaac-sim/assets/shoulder_link.ply",
    "/isaac-sim/assets/link1.ply",
    "/isaac-sim/assets/elbow_link.ply",
    "/isaac-sim/assets/link2.ply",
    "/isaac-sim/assets/w2w3_link.ply",
    "/isaac-sim/assets/end_effector_link.ply",
    "/isaac-sim/assets/gripper_base.ply",
    "/isaac-sim/assets/gripper_left.ply",
    "/isaac-sim/assets/gripper_right.ply"
]

def setup_scene(world, gsplat_manager):
    # Robot specific class that provides extra functionalities
    # such as having gripper and end_effector instances.
    # World boundaries 
    robot = world.scene.add(
        OwlBot(
            prim_path="/World/owl", 
            name="owl",
            position=np.array([0.0, 0.0, 0.87]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
            # position=np.array([-0.84, 0.0, 0.9398])
        )
    )

    pot_base = DynamicCuboid(
        "/World/Stand",
        name="stand",
        position=[-1.21,-0.058, 0.6],
        size=1.2,
        mass=100
    )
    world.scene.add(pot_base)

    cam = Camera(
        prim_path="/World/Camera", 
        name="camera", 
        resolution=(640, 480)
    )
    
    cam.set_world_pose(
        position=[-0.29, 2.81, 1.51],
        orientation=[0.023, 0.028, -0.65, -0.76],
        camera_axes="usd"
    )

    cam.set_focal_length(1.33)
    # cam.set_focus_distance(3.0)
    world.scene.add(cam)

    pot_usd_path = "/isaac-sim/assets/pot.usd"
    add_reference_to_stage(usd_path=pot_usd_path, prim_path="/World/Pot")
    pot_obj = RigidPrim("/World/Pot", name="pot", position=[-0.75, -0.018, 1.2], mass=2.0, scale=(0.34, 0.34, 0.34))
    # pot_obj = GeometryPrim("/World/Pot", "pot", position=[-0.75, -0.018, 1.2], mass=0.250, scale=(0.34, 0.34, 0.34), collision=True)
    # phys_mat = PhysicsMaterial("/World/pot/PhysicsMaterial", static_friction=0.8, dynamic_friction=0.8)
    # pot_obj.apply_physics_material(phys_mat)
    world.scene.add(pot_obj)
    return robot

if __name__ == "__main__":
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
    print("GP added")
    
    my_world.reset()

    gsplat_manager = IsaacSimGSplatManager(
        point_clouds=[
            "/isaac-sim/assets/ow_bg.ply",
            "/isaac-sim/assets/ow_pot.ply",
            *ROBOT_LINK_PLYS,
        ],
        transforms= # transforms with respect to background splat
        [
            (BASE_POS, BASE_QUAT),
            (BASE_POS, BASE_QUAT),
        ] +
        [
            (BASE_POS, BASE_QUAT)
        ] * len(ROBOT_LINK_PLYS),
        transforms_to_world=[
            True,
            False,
        ] + 
        [False] * len(ROBOT_LINK_PLYS)
    )
    
    robot = setup_scene(my_world, gsplat_manager=None)
    
    print("Set up scene")
    my_world.initialize_physics()
    my_world.reset()
    print("Physics intialized")
    controller = TeleopIKController(robot)

    cam = my_world.scene.get_object("camera")
    cam.prim.GetAttribute("verticalAperture").Set(1.833 * 10.0)
    cam.prim.GetAttribute("horizontalAperture").Set(1.026 * 10.0)
    gsplat_manager.register_camera(cam)

    pot = my_world.scene.get_object("pot")

    print("\n\n TEST \n\n")

    i = 0
    while simulation_app.is_running():
        print("LOOP")
        my_world.step(render=True)
        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                print("resetting")
                my_world.reset()
            controller.step(i)
            i += 1
            pot_position, pot_orientation = pot.get_world_pose()

            print("\npot position", pot_position)
            
            robot_link_transforms = _get_robot_link_transforms(robot, first_n_links=len(ROBOT_LINK_PLYS), robot_prim_path="/World/owl/")
            print("Transforms : \n", robot_link_transforms)
            
            gsplat_manager.update_transforms(
                [
                    (BASE_POS, BASE_QUAT),
                    (pot_position, pot_orientation),
                    *robot_link_transforms
                ]
            )

            gsplat_manager.update_cameras()
            im = gsplat_manager.render_frame()
            print("IM DATATYPE : \n",type(im))

    # controller._cleanup()
    simulation_app.close()
