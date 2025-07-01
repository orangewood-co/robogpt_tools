#!/usr/bin/env python3

import sys
import getpass
import numpy as np

driver_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools/robot_sdk"
sys.path.append(driver_path)
from SDK.owl_ec_sdk.client.robot import Owl_Ec_client # type: ignore

class bot_wrapper:

    def __init__(self,ip) -> None:
        print("Successfully loaded the SDK for Elite EC series")
        self.client = Owl_Ec_client(ip=ip)
        print("Connected to the Robot Controller")

    def test(self):
        print("Testing IP")
        ip = self.client.ip_test()
        return ip
    
    def get_tcp(self):
        print("getting tcp pose")
        # Function to get the tcp coordinates in Castersian space
        tcp_pose = self.client.get_tcp_position()
        #tcp_pose = [x/1000 for x in tcp_pose]
        return tcp_pose

    def get_joints(self):
        # Function to get the current joint angles of robot
        joint_angles = self.client.getjointangles()
        if joint_angles is not None:
           joint_values = np.deg2rad(joint_angles) # Converting degree to radians
           return joint_values
        else:
           print("Error: joint_angles is None. Check robot connection or function output.")
           return joint_angles

    def set_hand_teach(self,switch):
        # Function to enable hand drag 
        result = self.client.set_hand_drag(switch=switch)
        return result

    def move_to_pose(self, pose):
        # Function to move the robot to a desired pose
        #pose = [x*1000 for x in pose]
        self.client.movel(points=pose)
        return True
    
    def move_to_joint(self,joint_angles):
        # Function to move the robot to desired joint angles
        # joint angles in radians
        self.client.movej(joint_angles=joint_angles)
        return True
    
    def get_gripper(self,pin_no):
        response = self.client.get_digital_io(io_number=pin_no)
        return response
    
    def get_joint_speed(self):
        response=self.client.getjointspeed()
        return response
    
    def get_joint_acc(self):
        response=self.client.getjointacc()
        return response

    def move_in_arc(self,point_1,point_2):
        # Function to move robot in an arc consisting of three points
        self.client.movec(points=point_1,nexpoints=point_2)
        return True
    
    def set_gripper(self,pin_no,switch):
        # Function to control any I/O pin from the control box
        response = self.client.set_digital_io(io_number=pin_no,turn_on=switch)
        return response
    

    def move_translate(self,x,y,z):
        # Function to make robot in a particular axis or plane
        tcp_pose = self.client.get_tcp_position()

        tcp_pose[0] = tcp_pose[0] + x
        tcp_pose[1] = tcp_pose[1] + y
        tcp_pose[2] = tcp_pose[2] + z
        self.client.movel(points=tcp_pose)
        print(f"Moving robot in x:{x}, y:{y}, z:{z} direction")
        print(f"New TCP pose: {tcp_pose}")
        return True
    
    def move_in_trajactory(self,waypoints):
        # Function to move robot in a particular trajactory
        # To-DO
        pass

if __name__=="__main__":
    wrap = bot_wrapper(ip="192.168.1.200")
    # wrap.connect_robot()
    # print(wrap.get_tcp())
    # print(wrap.set_hand_teach(switch=True))
