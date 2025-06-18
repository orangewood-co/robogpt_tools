#!/usr/bin/env python3

"""
Python wrapper for Orangewood simulation SDK APIs
"""
import time
import sys
import signal
import rospy

from owl_robot_sdk import OwlSimClient

class bot_wrapper:
    def __init__(self, robot_group, gripper_group, time_out, gripper_enable):
        print("Successfully loaded the SDK for Robots in Simulation")
        
        # Check if ROS node is already initialized
        try:
            # This will fail if a node hasn't been initialized
            rospy.get_name()
            initialized = True
        except rospy.exceptions.ROSException:
            initialized = False
            
        # Using monkey patching to prevent OwlSimClient from initializing ROS
        if initialized:
            # Save the original init_node function
            original_init_node = rospy.init_node
            
            # Create a dummy function that does nothing
            def dummy_init_node(*args, **kwargs):
                rospy.loginfo("Skipping rospy.init_node() as it's already initialized")
                return Noneimport rospy

            
            # Replace the init_node function temporarily
            rospy.init_node = dummy_init_node
            
            try:
                # Initialize the client with the monkey-patched function
                self.client = OwlSimClient(robot_group, gripper_group, time_out, gripper_enable)
            finally:
                # Restore the original function
                rospy.init_node = original_init_node
        else:
            # If no node is initialized, normal initialization
            self.client = OwlSimClient(robot_group, gripper_group, time_out, gripper_enable)

    def get_tcp(self):
        '''
        Function to get the tcp coordinates in Castersian space
        '''
        return self.client.get_tcp()

    def get_joints(self):
        '''        
        Function to get the current joint angles of robot
        '''
        return self.client.get_joint()
    
    def set_hand_teach(self,switch):
        '''        
        Function to enable hand drag 
        '''        
        print("Can't enable hand drag in Simulation")
    
    def move_to_pose(self,pose):
        '''
        Function to move robot in cartersian space
        '''
        return self.client.move_to_pose(goalPose=pose)

    def move_to_joint(self, joints):
        '''
        Function to move the robot to desired joint angles
        '''
        return self.client.move_to_joint(jointPose=joints)
        
    def move_trajectory(self,waypoints):
        '''
        Function to move robot in a trajactory of waypoints
        '''
        return self.client.move_trajectory(trajectory=waypoints)

    def move_translate(self,x,y,z):
        '''
        Function to make robot in a particular axis or plane
        '''
        return self.client.move_translate(x,y,z)

    def set_gripper(self,switch: bool):
        '''
        Function to control the gripper in the Simulation 
        '''
        if switch:
            self.client.set_gripper(goal_state="open",mode="state")
        if not switch:
            self.client.set_gripper(goal_state="close",mode="state")
            
        return 
