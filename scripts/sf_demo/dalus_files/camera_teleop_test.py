#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from keyboard import is_pressed

# Default starting poses
DEFAULT_TRANS_POSE = [-0.19317988, 2.25949238, 1.41142992]
DEFAULT_ORIENT_POSE = [0.02248325947618654, -0.023770599992951346, 0.726116714378276, -0.6867925296039543]

class CameraTeleopNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_teleop', anonymous=True)
        # Publisher for sending camera pose messages
        self.pub = rospy.Publisher('/camera_pose', Pose, queue_size=10)

        # Set up the initial camera pose using the default starting values
        self.camera_pose = Pose()
        self.camera_pose.position.x = DEFAULT_TRANS_POSE[0]
        self.camera_pose.position.y = DEFAULT_TRANS_POSE[1]
        self.camera_pose.position.z = DEFAULT_TRANS_POSE[2]

        self.camera_pose.orientation.x = DEFAULT_ORIENT_POSE[0]
        self.camera_pose.orientation.y = DEFAULT_ORIENT_POSE[1]
        self.camera_pose.orientation.z = DEFAULT_ORIENT_POSE[2]
        self.camera_pose.orientation.w = DEFAULT_ORIENT_POSE[3]

        # Loop rate in Hz
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        rospy.loginfo("Camera Teleop Node started.")
        rospy.loginfo("Controls:")
        rospy.loginfo("  Movement: W (forward), S (backward), A (left), D (right), R (up), F (down)")
        rospy.loginfo("  Rotation: Q (rotate left), E (rotate right)")
        rospy.loginfo("  Exit: ESC")
        
        try:
            while not rospy.is_shutdown():
                self.handle_keyboard_input()
                self.pub.publish(self.camera_pose)
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Camera Teleop Node shutting down.")

    def handle_keyboard_input(self):
        # Define step sizes for position and rotation adjustments
        step = 0.1           # Position step size (units)
        rotation_step = 0.1  # Rotation step size (radians)

        # Position Controls:
        if is_pressed('w'):  # Move forward (increase z)
            self.camera_pose.position.z += step
        if is_pressed('s'):  # Move backward (decrease z)
            self.camera_pose.position.z -= step
        if is_pressed('a'):  # Move left (increase y)
            self.camera_pose.position.y += step
        if is_pressed('d'):  # Move right (decrease y)
            self.camera_pose.position.y -= step
        if is_pressed('r'):  # Move up (increase x)
            self.camera_pose.position.x += step
        if is_pressed('f'):  # Move down (decrease x)
            self.camera_pose.position.x -= step

        # Orientation Controls (simple yaw adjustments):
        if is_pressed('q'):  # Rotate left (increase yaw)
            self.camera_pose.orientation.z += rotation_step
        if is_pressed('e'):  # Rotate right (decrease yaw)
            self.camera_pose.orientation.z -= rotation_step

        # Exit if the ESC key is pressed
        if is_pressed('esc'):
            rospy.signal_shutdown("ESC pressed. Shutting down.")

if __name__ == '__main__':
    node = CameraTeleopNode()
    node.run()
