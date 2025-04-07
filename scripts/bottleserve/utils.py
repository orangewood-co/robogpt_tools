import rospy
import json
import os

class PoseManager:
    def __init__(self, poses_file="saved_joint_angles.json"):
        self.poses_file = poses_file
        self.poses = self.load_poses()

    def load_poses(self):
        """Load saved poses from a file."""
        if os.path.isfile(self.poses_file):
            try:
                with open(self.poses_file, "r") as file:
                    rospy.loginfo("Loading poses from file...")
                    return json.load(file)
            except Exception as e:
                rospy.logerr(f"Error loading poses from file: {str(e)}")
                return {}
        rospy.logwarn("Poses file not found. No poses loaded.")
        return {}

    def pose_exists(self, pose_name):
        """Check if a pose exists."""
        return pose_name in self.poses

    def get_pose(self, pose_name):
        """Retrieve joint angles for a pose."""
        return self.poses.get(pose_name, None)
