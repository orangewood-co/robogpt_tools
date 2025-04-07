#!/usr/bin/python3

import sys
import rospy
import rospkg
import os, random
import actionlib
from std_srvs.srv import Empty

##########################################################################
# Appending and importing the pkg path and services

rospack = rospkg.RosPack()
vision_path = rospack.get_path("robogpt_vision")
sys.path.append(vision_path)
from robogpt_vision.srv import GetWorldContext, GetWorldContextRequest

# Import AutoTrain action messages
from auto_train_ros.msg import AutoTrainAction, AutoTrainGoal, AutoTrainFeedback

#Import Tour action messages
from robogpt_agents.msg import TourAction, TourGoal

###########################################################################

class ExternalServices:
    """A class to interact with external ROS services and actions."""

    def __init__(self):
        """Initializes the ROS node if not already initialized."""
        # Check if the node is already initialized. If not, initialize it.
        if not rospy.core.is_initialized():
            rospy.init_node('external_services_node', anonymous=True)
            rospy.loginfo("Initialized ROS node 'external_services_node'.")

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
        # Wait for the 'get_world_context' service to become available
        rospy.wait_for_service('get_world_context')
        try:
            # Create a proxy for the 'get_world_context' service
            get_world_context = rospy.ServiceProxy('get_world_context', GetWorldContext)
            
            # Create and populate the service request object
            req = GetWorldContextRequest()
            req.object_name = object_name
            req.parent_frame = parent_frame
            req.camera_name = camera_name
            req.include_ort = include_ort
            
            # Call the service and return the response
            resp = get_world_context(req)
            rospy.loginfo(f"Received transformation for '{object_name}': {resp.Xbase}")
            return resp.Xbase
        
        except rospy.ServiceException as e:
            # Print an error message if the service call fails
            rospy.logerr(f"Service call failed: {e}")
            return None

    def switch_sim_gripper(self, state: bool):
        """
        Calls the gripper service to switch it on or off.

        Args:
            state (bool): True to switch the gripper on, False to switch it off.
        """
        # Determine the service name based on the state
        service_name = '/owl/vacuum_gripper/off' if state else '/owl/vacuum_gripper/on'

        rospy.wait_for_service(service_name)
        try:
            # Create a proxy to the service
            gripper_service = rospy.ServiceProxy(service_name, Empty)
            
            # Call the service
            gripper_service()
            rospy.loginfo(f"Successfully called the service: {service_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    ###########################################################################
    # Action Client Method for Autotrain 
    ###########################################################################

    def feedback_cb(self, feedback):
        """
        Callback function to handle feedback from the AutoTrain action server.

        Args:
            feedback (AutoTrainFeedback): Feedback message from the action server.
        """
        # Assuming AutoTrainFeedback has fields like 'current_epoch' and 'progress'
        # Adjust the field names based on your actual AutoTrainFeedback message definition
        rospy.loginfo(f"[AutoTrain Feedback] Epoch: {feedback.current_epoch}, Progress: {feedback.progress * 100:.2f}%")

    def done_cb(self, status, result):
        """
        Callback function to handle the result of the AutoTrain action.

        Args:
            status (int): Status of the action.
            result (AutoTrainResult): Result message from the action server.
        """
        if result.new_weights_path:
            rospy.loginfo(f"[AutoTrain Result] New Weights Path: {result.new_weights_path}, Status: {result.status}")
        else:
            rospy.logwarn("[AutoTrain Result] No new weights were generated. Check the server.")

    def send_auto_train_goal(
        self,
        combined_folder="/home/sakabato/test11",
        object_name = "reb box",
        object_label = "duster",
        prev_data_folder="/path/to/prev_data_folder",
        new_weights=True,
        abs_yaml_file="/path/to/train.yaml",
        draw_bb=False,
        image_threshold=10,
        number_aug=3,
        epochs=10,
        map_threshold=0.5,            # Default value
        inference=False,               # Default value
        inference_threshold=0.3,       # Default value
        camera_range=10                # Default value
    ):
        """
        Sends a goal to the AutoTrain action server with specified parameters.

        Args:
            combined_folder (str): Path to the combined folder.
            prev_data_folder (str): Path to the previous data folder.
            new_weights (bool): Flag to generate new weights.
            abs_yaml_file (str): Path to the YAML configuration file.
            draw_bb (bool): Flag to draw bounding boxes.
            image_threshold (int): Image threshold value.
            number_aug (int): Number of augmentations.
            epochs (int): Number of training epochs.
            map_threshold (float, optional): Map threshold value. Defaults to 0.5.
            inference (bool, optional): Flag to enable inference. Defaults to False.
            inference_threshold (float, optional): Inference threshold value. Defaults to 0.3.
            camera_range (int, optional): Camera range value. Defaults to 10.

        Returns:
            None
        """
        try:
            # Initialize the action client
            client = actionlib.SimpleActionClient('auto_train', AutoTrainAction)
            rospy.loginfo("Waiting for AutoTrain Action Server to start...")
            client.wait_for_server()
            rospy.loginfo("AutoTrain Action Server started, sending goal.")
            
            base_dir = "/home/sakabato"
            rand_num = random.randint(100000, 999999)
            dir_name = f"train{rand_num}"
            new_dir_path = os.path.join(base_dir, dir_name)

            # Define the goal with provided arguments and default values for the rest
            goal = AutoTrainGoal(
                combined_folder=new_dir_path,
                object_name=object_name,
                object_label = object_label,
                prev_data_folder=prev_data_folder,
                new_weights=new_weights,
                abs_yaml_file=abs_yaml_file,
                draw_bb=draw_bb,
                image_threshold=image_threshold,
                number_aug=number_aug,
                epochs=epochs,
                map_threshold=map_threshold,
                inference=inference,
                inference_threshold=inference_threshold,
                camera_range=camera_range
            )

            # Send the goal with feedback and done callbacks
            client.send_goal(goal, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
            rospy.loginfo("Goal sent to AutoTrain Action Server")

            # Optionally, you can implement asynchronous behavior here
            # For simplicity, we'll wait for the result
            client.wait_for_result()
            result = client.get_result()
            return result
            # Handle the result in the done_cb
            # Since done_cb is already handling it, no need to process it here
            # However, if you want to perform additional actions, you can do so here

        except rospy.ROSInterruptException:
            rospy.loginfo("AutoTrain client interrupted before completion.")
        except Exception as e:
            rospy.logerr(f"An error occurred in send_auto_train_goal: {e}")

# ###########################################################################
# Action Client for use case tours
# ###########################################################################

    def run_tour(self,tour_name: str):
        """
        Sends a tour request to the Tour Action Server.

        This method creates and initializes an action client for the 
        "tour_action" action server, waits for the server to become available,
        and then sends a goal containing the specified tour name. The tour action 
        server is expected to execute the corresponding tour sequence based on the tour name.

        Args:
            tour_name (str): The name identifier of the tour to be executed.

        Returns:
            None
        """
        client = actionlib.SimpleActionClient("tour_action", TourAction)
    
        rospy.loginfo("Waiting for server...")
        client.wait_for_server()

        goal = TourGoal()
        goal.tour_name = tour_name

        rospy.loginfo(f"Sending tour request: {tour_name}")
        client.send_goal(goal)

# ###########################################################################
