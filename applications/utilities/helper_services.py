#!/usr/bin/python3 -u

import sys
import rclpy
from rclpy.node import Node
import getpass
import os, random
import time
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_srvs.srv import Empty
from robogpt_perception.srv import PoseCalculator, WorldContext
from robogpt.core_stack.robogpt_agents.scripts import prompt
from robogpt_perception.action import Autotrain

# Define status codes for readability
STATUS_CODES = {
    0: "Image Capture Starts",
    1: "Image Capture process Ends",
    2: "Augmentation Started",
    3: "Augmentation Ended",
    4: "Training Started",
    5: "Training Completed"
    # Add more status codes if needed
}
###########################################################################

class ExternalServices:
    """A class to interact with external ROS services and actions."""

    def __init__(self, node=None):
        """Initializes the ROS node if not already initialized."""
        if node is not None:
            self.node = node

    ###########################################################################
    # Service Call Methods

    def call_get_world_context(self, object_name, parent_frame, camera_name, include_ort=True):
        """
        Calls the 'get_world_context' ROS service and returns the transformation.

        Args:
            object_name (str): The name of the object whose world context is to be retrieved.
            parent_frame (str): The reference frame from which the transformation is calculated.
            camera_name (str): The camera frame used for the calculation.
            include_ort (bool, optional): Whether or not to include orientation in the response. Defaults to True.

        Returns:
            Xbase: The transformation of the object in the base frame.
            None: If the service call fails.
        """
        # Create a client for the 'get_world_context' service
        client = self.node.create_client(PoseCalculator, 'get_world_context')
        
        # Wait for the service to become available
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('get_world_context service not available, waiting...')
        
        try:
            # Create and populate the service request object
            req = PoseCalculator.Request()
            req.object_name = object_name
            req.parent_frame = parent_frame
            req.camera_name = camera_name
            req.include_ort = include_ort
            
            # Call the service and get the future
            future = client.call_async(req)
            # Wait for the future to complete
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info(f"Received transformation for '{object_name}': {response.Xbase}")
                return response.xbase
            else:
                self.node.get_logger().error('Failed to call get_world_context service')
                return None
        
        except Exception as e:
            # Print an error message if the service call fails
            self.node.get_logger().error(f"Service call failed: {e}")
            return None

    def switch_sim_gripper(self, state: bool):
        """
        Calls the gripper service to switch it on or off.

        Args:
            state (bool): True to switch the gripper on, False to switch it off.
        """
        # Determine the service name based on the state
        service_name = '/owl/vacuum_gripper/off' if state else '/owl/vacuum_gripper/on'

        client = self.node.create_client(Empty, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{service_name} service not available, waiting...')
            
        try:
            # Create a request (Empty has no fields)
            request = Empty.Request()
            
            # Call the service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                self.node.get_logger().info(f"Successfully called the service: {service_name}")
            else:
                self.node.get_logger().error(f"Failed to call {service_name} service")
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")

    def analyze_image(self, prompt, perform_ocr=False):
        """
        Analyzes an image using the GPT-4o model.

        Args:
            image_path (str): The path to the image file.
            prompt_text (str): The prompt text for analysis.
            perform_ocr (bool): Whether to perform OCR on the image.

        Returns:
            str: The result of the analysis.
        """

        client = self.node.create_client(WorldContext, 'analyze_image')

        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('analyze_image service not available, waiting...')

        try:
            req = WorldContext.Request()
            req.prompt = prompt
            req.perform_ocr = perform_ocr
            req.compare_images = False

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info(f"Received analysis result: {response.result}")
                return response
        except Exception as e:
            self.node.get_logger().error(f"Error analyzing image: {e}")
            return "Error in analysis"
        
    def compare_images(self, prompt):
        """
        Compares two images using the GPT-4o model.

        Args:
            prompt (str): The prompt text for image comparison.

        Returns:
            str: The result of the image comparison.
        """
        client = self.node.create_client(WorldContext, 'analyze_image')

        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('analyze_image service not available, waiting...')

        try:
            req = WorldContext.Request()
            req.prompt = prompt
            req.perform_ocr = False
            req.compare_images = True

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info(f"Received comparison result: {response.result}")
                return response
        except Exception as e:
            self.node.get_logger().error(f"Error comparing images: {e}")
            return "Error in comparison"



    ###########################################################################
    # Action Client Method for Autotrain 
    ###########################################################################

    # def feedback_cb(self, feedback_msg):
    #     """
    #     Callback function to handle feedback from the action server.
    #     """
    #     feedback = feedback_msg.feedback
    #     status_code = feedback.status
    #     status_message = STATUS_CODES.get(status_code, "Unknown Status")
    #     self.get_logger().info(f"Feedback Received: Status {status_code} - {status_message}")

    # def send_auto_train_goal(
    #     self,
    #     data_folder=None,
    #     image_topic="/camera/color/image_raw",
    #     object_name="human",
    #     object_label="kaleshi",
    #     prev_data_folder="/path/to/prev_data_folder",
    #     new_weights=True,
    #     abs_yaml_file="/path/to/train.yaml",
    #     draw_bb=False,
    #     image_threshold=100,
    #     number_aug=3,
    #     epochs=65,
    #     map_threshold=0.5
    # ):
    #     """
    #     Sends a goal to the AutoTrain action server with specified parameters.
    #     """
    #     try:
    #         # Initialize the action client
    #         self._action_client = ActionClient(self, AutoTrain, 'auto_train')
    #         self.get_logger().info("Waiting for AutoTrain Action Server to start...")
            
    #         if not self._action_client.wait_for_server(timeout_sec=10.0):
    #             self.get_logger().error("AutoTrain Action Server not available after waiting")
    #             return
                
    #         self.get_logger().info("AutoTrain Action Server started, sending goal.")

    #         if data_folder is None:
    #             base_dir = f"/home/{getpass.getuser()}/autotrain_data/"
    #             rand_num = random.randint(100000, 999999)
    #             dir_name = f"train{rand_num}"
    #             new_dir_path = os.path.join(base_dir, dir_name)

    #         # Define the goal with provided arguments and default values for the rest
    #         goal_msg = AutoTrain.Goal()
    #         goal_msg.data_folder = new_dir_path
    #         goal_msg.image_topic = image_topic
    #         goal_msg.object_name = object_name
    #         goal_msg.object_label = object_label
    #         goal_msg.prev_data_folder = prev_data_folder
    #         goal_msg.new_weights = new_weights
    #         goal_msg.abs_yaml_file = abs_yaml_file
    #         goal_msg.draw_bb = draw_bb
    #         goal_msg.image_threshold = image_threshold
    #         goal_msg.number_aug = number_aug
    #         goal_msg.epochs = epochs
    #         goal_msg.map_threshold = map_threshold

    #         # Send the goal with feedback callback
    #         send_goal_future = self._action_client.send_goal_async(
    #             goal_msg, 
    #             feedback_callback=self.feedback_cb
    #         )
            
    #         # Add a done callback
    #         send_goal_future.add_done_callback(self.goal_response_callback)
            
    #         self.get_logger().info("Goal sent to AutoTrain Action Server")

    #     except Exception as e:
    #         self.get_logger().error(f"An error occurred in send_auto_train_goal: {e}")

    # def goal_response_callback(self, future):
    #     """
    #     Callback for when a goal response is received.
    #     """
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected')
    #         return

    #     self.get_logger().info('Goal accepted')
        
    #     # Get the result
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.get_result_callback)
        
    # def get_result_callback(self, future):
    #     """
    #     Callback for when the result of a goal is received.
    #     """
    #     result = future.result().result
    #     status = future.result().status
        
    #     if status == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info('Goal succeeded!')
    #     else:
    #         self.get_logger().info(f'Goal failed with status: {status}')


