#!/usr/bin/env python3

import rospy
from helper_services import RobotMovementService, GripperService
from utils import PoseManager


class BottleServingController:
    def __init__(self):
        self.robot_service = RobotMovementService()
        self.gripper_service = GripperService()
        self.pose_manager = PoseManager()

    def execute_serving(self):
        """Execute bottle serving sequence."""
        try:
            rospy.loginfo("Starting bottle serving sequence...")

            # Sequence for serving bottle 1
            self.move_to_pose("home")
            self.move_to_pose("target1")
            self.move_to_pose("grab1")
            self.control_gripper(True)  # Grab the bottle
            self.move_to_pose("waypoint1")
            self.move_to_pose("waypoint1.1")
            self.move_to_pose("serve1")  # Serve the bottle
            self.control_gripper(False)  # Release the bottle
            self.move_to_pose("home2")

            # Sequence for serving bottle 2
            self.move_to_pose("target2")
            self.move_to_pose("grab2")
            self.control_gripper(True)
            self.move_to_pose("waypoint2")
            self.move_to_pose("waypoint2.1")
            self.move_to_pose("serve2")
            self.control_gripper(False)
            self.move_to_pose("home3")

            # Final return to rest position
            self.move_to_pose("lasthome")
            rospy.loginfo("Bottle serving sequence completed.")

        except Exception as e:
            rospy.logerr(f"Error during bottle serving: {str(e)}")

    def move_to_pose(self, pose_name):
        """Move the robot to a specified pose."""
        if not self.pose_manager.pose_exists(pose_name):
            rospy.logerr(f"Pose '{pose_name}' not found.")
            return
        self.robot_service.move_to_pose(pose_name)

    def control_gripper(self, close):
        """Control the robot's gripper."""
        if close:
            self.gripper_service.close_gripper()
        else:
            self.gripper_service.open_gripper()


if __name__ == "__main__":
    rospy.init_node("bottle_serving_controller", anonymous=True)
    try:
        controller = BottleServingController()
        controller.execute_serving()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")
