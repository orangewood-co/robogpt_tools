#!/usr/bin/env python
import numpy as np
import rospy
import ast
from geometry_msgs.msg import Pose
from dalus_sim.ros_noetic.adapter import ROSNoeticAdapter
import dalus_sim.utils as utils 

# Initial default camera pose values
DEFAULT_TRANS_POSE = [-0.19317988, 2.25949238, 1.41142992]
DEFAULT_ORIENT_POSE = [0.02248325947618654, -0.023770599992951346, 0.726116714378276, -0.6867925296039543]

#Define your custom background pose (position and quaternion)
CUSTOM_BG_POSE = [0.0, 0.0, 0.0]  
CUSTOM_BG_QUAT = [0.0, 0.0, 0.0, 0.0]

BASE_QUAT = [1, 0, 0, 0] # Zero Rotation Quaternion
BASE_POS = [0, 0, 0] # Zero Position

# Map from Gaussian Splat to Child Frame Link ID
PLY_TO_LINK_MAP = {
    "/root/assets/base_link.ply": "base_link",
    "/root/assets/shoulder_link.ply": "shoulder_link",
    "/root/assets/link1.ply": "link1",
    "/root/assets/elbow_link.ply": "elbow_link",
    "/root/assets/link2.ply": "link2",
    "/root/assets/w2w3_link.ply": "w2w3_link",
    "/root/assets/end_effector_link.ply": "end_effector_link",
    "/root/assets/gripper_base.ply": "end_effector_link",
    "/root/assets/gripper_left.ply": "gripper_finger1_finger_tip_link",
    "/root/assets/gripper_right.ply": "gripper_finger2_finger_tip_link"
}

ROBOT_LINK_PLYS = list(PLY_TO_LINK_MAP.keys())
transforms = [(CUSTOM_BG_POSE, CUSTOM_BG_QUAT)] + [(BASE_POS, BASE_QUAT) for _ in range(len(ROBOT_LINK_PLYS))]

# Global variables to store state
dalus_manager = None
last_bg_path = None  # Tracks the last used background file
last_bg_pose = [0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0]

def scene_setup(bg_plot: str, robot_links: list, transforms):
    # Construct a list of point clouds: background + each robot link.
    point_clouds = [bg_plot] + robot_links
    # First element rotates to the world frame; others do not.
    rotate_to_world_frame = [True,] + [False] * len(robot_links)
    dm = ROSNoeticAdapter(
        point_clouds=point_clouds,
        transforms=transforms,
        rotate_to_world_frame=rotate_to_world_frame,
        joints_to_track=PLY_TO_LINK_MAP
    )
    return dm

def camera_setup(trans_pose: list, orient_pose: list):
    # Define camera intrinsics
    camera_K = utils.camera_K_from_im_size_fov(img_wh=(1080, 720), fov=np.deg2rad(45.0))
    # Define camera extrinsics using the provided pose
    T = utils.T_from_pos_and_quat(
        pos=trans_pose,
        quat=orient_pose,
        quat_scalar_first=True
    )
    return camera_K, T

def update_dalus_manager():
    """
    Checks the rosparam and updates the dalus_manager if the background file has changed.
    Also registers the camera using the latest camera pose.
    """
    global last_bg_path,last_bg_pose, dalus_manager, DEFAULT_TRANS_POSE, DEFAULT_ORIENT_POSE

    # Retrieve the background file path parameter
    bg_path = rospy.get_param("/updated_ply_file_path", "/root/assets/ow_bg.ply")
    bg_pose = rospy.get_param("/bg_pose",[0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0])
    
    # If we have not created a manager yet, or if the background file changed, re-create it.
    if last_bg_path is None or last_bg_pose is None or bg_path != last_bg_path or bg_pose != last_bg_pose:
        #Define your custom background pose (position and quaternion)
        CUSTOM_BG_POSE = [bg_pose[0],bg_pose[1],bg_pose[2]]  
        CUSTOM_BG_QUAT = [bg_pose[3],bg_pose[4],bg_pose[5],bg_pose[6]]

        transforms = [(CUSTOM_BG_POSE, CUSTOM_BG_QUAT)] + [(BASE_POS, BASE_QUAT) for _ in range(len(ROBOT_LINK_PLYS))]
        rospy.loginfo("Creating a new ROSNoeticAdapter with background: %s", bg_path)
        camera_K, T = camera_setup(DEFAULT_TRANS_POSE, DEFAULT_ORIENT_POSE)
        dalus_manager = scene_setup(bg_plot=bg_path, robot_links=ROBOT_LINK_PLYS, transforms=transforms)
        # Set up the camera using the (possibly updated) pose.
        dalus_manager.register_camera(
            camera_K=camera_K,
            camera_T=T
        )
        last_bg_pose = bg_pose
        last_bg_path = bg_path

def camera_pose_callback(msg):
    global DEFAULT_TRANS_POSE, DEFAULT_ORIENT_POSE
    # Update the default camera pose based on incoming messages.
    DEFAULT_TRANS_POSE = [
        msg.position.x,
        msg.position.y,
        msg.position.z
    ]
    DEFAULT_ORIENT_POSE = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]
    rospy.loginfo("Updated camera pose: trans=%s, orient=%s", DEFAULT_TRANS_POSE, DEFAULT_ORIENT_POSE)
    bg_path = rospy.get_param("/updated_ply_file_path", "/root/assets/ow_bg.ply")
    camera_K, T = camera_setup(DEFAULT_TRANS_POSE, DEFAULT_ORIENT_POSE)
    dalus_manager = scene_setup(bg_plot=bg_path, robot_links=ROBOT_LINK_PLYS)
    # Set up the camera using the (possibly updated) pose.
    dalus_manager.register_camera(
        camera_K=camera_K,
        camera_T=T
    )
# def setup_callback(msg):


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('dalus_manager_node', anonymous=True)
    
    # Subscribe to camera_pose topic
    rospy.Subscriber('camera_pose', Pose, camera_pose_callback)
    
    rate = rospy.Rate(10)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        try:
            update_dalus_manager()
            rate.sleep()
        except rospy.ROSInterruptException:
            break
