#!/usr/bin/python3 -u

import random
import getpass
import numpy as np
import time
import sys, os
import json
import rclpy
import rclpy
from rclpy.node import Node
import tf2_ros
import ament_index_python.packages
import pusher  
import importlib
import geometry_msgs.msg
import tf2_geometry_msgs
from difflib import SequenceMatcher
from typing import Type, List, Dict, Optional
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from robogpt_tools.applications.utilities.helper_services import ExternalServices
from robogpt_tools.applications.utilities.robotiqgripper import RobotiqGripperClient
from robogpt_tools.applications.utilities.param_read_write import ParameterReader, ParameterWriter
from robogpt.core_stack.robogpt_agents.scripts.prompt import RobogptAgentNode

# Color Coding for print statements for Debugging
class Colors:
    # Reset
    RESET = "\033[0m"
    # Regular colors
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    
# Json paths used in skills
tools_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools"
vision_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_perception"
agent_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt/core_stack/robogpt_agents"
robot_joint_file_path=os.path.join(tools_path,"robot_config/robot_joints.json")
robot_home_file_path=os.path.join(tools_path,"robot_config/robot_pose.json")
object_details=os.path.join(vision_path,"vision_config/vision_config.json")
tour_paths = os.path.join(vision_path,"config/tour_scripts")

# parameter instances
param_reader = ParameterReader(RobogptAgentNode())
param_writer = ParameterWriter(RobogptAgentNode())

