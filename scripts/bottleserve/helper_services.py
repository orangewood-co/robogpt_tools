import rospy
from owl_client.client.robot import OwlClient, Joint

class RobotMovementService:
    def __init__(self, host="10.42.0.52", joint_speed=100):
        self.host = host
        self.joint_speed = joint_speed
        self.robot = self.connect_to_robot()

    def connect_to_robot(self):
        """Establish connection to the robot."""
        try:
            rospy.loginfo("Connecting to robot...")
            robot = OwlClient(self.host)
            rospy.loginfo("Robot connected successfully.")
            return robot
        except Exception as e:
            rospy.logerr(f"Error connecting to robot: {str(e)}")
            return None

    def move_to_pose(self, pose_name):
        """Move the robot to the given pose."""
        try:
            rospy.loginfo(f"Moving to pose: {pose_name}")
            joint_angles = rospy.get_param(f"/poses/{pose_name}")
            joint_pose = Joint(*joint_angles)
            self.robot.move_to_joint(jointPose=joint_pose, toolSpeed=self.joint_speed)
            rospy.loginfo(f"Successfully moved to pose: {pose_name}")
        except Exception as e:
            rospy.logerr(f"Error moving to pose '{pose_name}': {str(e)}")


class GripperService:
    def __init__(self):
        self.robot = RobotMovementService().robot

    def close_gripper(self):
        """Close the robot's gripper."""
        try:
            rospy.loginfo("Closing gripper...")
            self.robot.gripper_close()
        except Exception as e:
            rospy.logerr(f"Error closing gripper: {str(e)}")

    def open_gripper(self):
        """Open the robot's gripper."""
        try:
            rospy.loginfo("Opening gripper...")
            self.robot.gripper_open()
        except Exception as e:
            rospy.logerr(f"Error opening gripper: {str(e)}")
