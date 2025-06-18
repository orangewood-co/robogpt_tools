#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from std_srvs.srv import Empty
from robotiq_gripper.srv import CommandRobotiqGripperMM, CommandRobotiqGripperPose


class RobotiqGripperClient:
    """Client class to interact with Robotiq gripper services."""
    
    def __init__(self, node=None):
        """Initialize the Robotiq gripper client."""
        
        if node is not None:
            self.node = node
   
        # Create service clients
        self.reset_client = self.node.create_client(
            Empty, '/robotiq/gripper/reset')
        
        self.activate_client = self.node.create_client(
            Empty, '/robotiq/gripper/activate')
            
        self.open_client = self.node.create_client(
            Empty, '/robotiq/gripper/open')
            
        self.close_client = self.node.create_client(
            Empty, '/robotiq/gripper/close')
            
        self.pose_client = self.node.create_client(
            CommandRobotiqGripperPose, '/robotiq/gripper/command/pose')
            
        self.mm_client = self.node.create_client(
            CommandRobotiqGripperMM, '/robotiq/gripper/command/mm')
    
    def _wait_for_service(self, client: Client, timeout_sec=5.0):
        """Wait for service to be available"""

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                f'Service {client.srv_name} not available after waiting {timeout_sec} seconds')
            return False
        return True
    
    def _call_service(self, client: Client, request, timeout_sec=5.0):
        """Call a service and wait for the response."""

        if not self._wait_for_service(client, timeout_sec):
            return None
            
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        if future.done():
            return future.result()
        else:
            self.node.get_logger().error('Service call timed out')
            return None
    
    def reset(self, timeout_sec=5.0):
        """Reset the gripper. """ 

        request = Empty.Request()
        response = self._call_service(self.reset_client, request, timeout_sec)
        return response is not None
    
    def activate(self, timeout_sec=5.0):
        """Activate the gripper."""

        request = Empty.Request()
        response = self._call_service(self.activate_client, request, timeout_sec)
        return response is not None
    
    def open(self, timeout_sec=5.0):
        """Open the gripper fully."""

        request = Empty.Request()
        response = self._call_service(self.open_client, request, timeout_sec)
        return response is not None
    
    def close(self, timeout_sec=5.0):
        """Close the gripper fully.""" 

        request = Empty.Request()
        response = self._call_service(self.close_client, request, timeout_sec)
        return response is not None
    
    def go_to_position(self, position, speed=255, force=255, timeout_sec=5.0):
        """Move the gripper to a specific position."""  

        request = CommandRobotiqGripperPose.Request()
        request.speed = speed
        request.force = force
        request.gripperpose = position
        
        response = self._call_service(self.pose_client, request, timeout_sec)
        if response is None:
            return None
        
        return (response.success, response.obj_detected, response.gripperpose)
    
    def go_to_width(self, width_mm, speed=255, force=255, timeout_sec=5.0):
        """Move the gripper to a specific width in millimeters."""

        request = CommandRobotiqGripperMM.Request()
        request.speed = speed
        request.force = force
        request.gripperwidth = width_mm
        
        response = self._call_service(self.mm_client, request, timeout_sec)
        if response is None:
            return None
        
        return (response.success, response.obj_detected, response.gripperwidth)
    
    def shutdown(self):
        """Shutdown the client and clean up resources."""

        if self._owns_node:
            self.node.destroy_node()
            rclpy.shutdown()
