#!/usr/bin/env python3

from owl_client import Pose

def create_pose_from_point(point):
    """
    Create a pose from a surface normal and the topmost point of a point cloud.
    Parameters:
        normal (numpy.ndarray): The average normal vector of the point cloud surface.
        topmost_point (numpy.ndarray): The coordinates of the topmost point in the point cloud.
    Returns:
        geometry_msgs.msg.Pose: The pose with the position set to the topmost point and orientation set based on the normal.
    """
    pose = Pose()
    pose.x = point[0]
    pose.y = point[1]
    pose.z = point[2]
    pose.x = point[3]
    pose.y = point[4]
    pose.z = point[5]

    return pose

