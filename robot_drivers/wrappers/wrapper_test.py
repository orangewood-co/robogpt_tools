#!/usr/bin/env python3

import sys
import time
import rospkg
import signal

rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_drivers')    
sys.path.append(package_path)

from cs_wrapper import OwlCsClientWrapper  # Updated import to use CS66 wrapper

tcp_pose=[0.580, 0.140, 0.672, 1.740, -1.385, 1.851]


class TestWrapper:

    def __init__(self, ip) -> None:
        self.ip = ip

        signal.signal(signal.SIGINT, self.signal_handler)
        print("Application is running. Press Ctrl+C to exit.")
        self.wrapper = OwlCsClientWrapper(ip=self.ip)

        self.test_connection()
        self.test_get_functions()
        self.test_move_functions()

    def signal_handler(self, sig, frame):
        print("Interrupt received, closing application.")
        sys.exit(0)

    def test_connection(self):
        # Test the connection with the robot
        connection_status = self.wrapper.test_connection()
        print("Connection test result:", connection_status)
    
    def test_get_functions(self):
        # Retrieve the TCP position
        current_pose = self.wrapper.get_tcp_position()
        print("Current cartesian position:", current_pose)

        # Retrieve the joint angles
        current_joints = self.wrapper.get_joint_angles()
        print("Current joint position of robot:", current_joints)

    def test_move_functions(self):
        # Test moving in linear motion
        print("Testing the linear move function")
        self.wrapper.move_linear(position=tcp_pose,acceleration=1.3,speed=0.2)  # Example move down by 0.1 m in Z-axis
        # time.sleep(5.0)
        # Test joint movements
        # print("Testing the joint movements")
        
        # try:
        #     current_joints = self.wrapper.get_joint_angles()
        #     if current_joints:
        #         print(current_joints)
        #         current_joints[2] -= 0.5  # Example: adjust a joint angle
        #         current_joints[5] -= 0.5
        #         print(current_joints)
        #         self.wrapper.move_joint(joint_angles=current_joints,acceleration=1.3,speed=0.2)
        # except Exception as e:
        #     print(f"Error encountered: {e}")
        


        # # Test cartesian pose movement
        # print("Testing the robot motion in cartesian space")
        # current_pose = self.wrapper.get_tcp_position()
        # if current_pose:
        #     current_pose[2] += 0.5  # Move up by 0.1 m in Z-axis
        #     print("Target pose given:", current_pose)
        #     self.wrapper.move_linear(position=current_pose,acceleration=1.3,speed=0.2)

        # # Test setting digital I/O pin
        # print("Testing the I/O pins")
        # self.wrapper.set_digital_io(io_number=0, turn_on=True)

        # # Test collision detection status
        # print("Testing collision detection status")
        # collision_status = self.wrapper.get_collision_detection_status()
        # print(f"Collision detection status: {'Enabled' if collision_status else 'Disabled'}")

if __name__ == "__main__":
    test = TestWrapper(ip="192.168.1.200")#!/usr/bin/env python3