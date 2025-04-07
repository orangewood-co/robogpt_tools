#!/usr/bin/python3

import random
import numpy as np
import time
import sys, os
import json
import rospy
import tf2_ros
import rospkg
import pusher  
import importlib
import geometry_msgs.msg
import tf2_geometry_msgs
from difflib import SequenceMatcher
from robogpt_apps.scripts.utilities import utils
from langchain.tools import BaseTool
from typing import Type, List, Dict, Optional
from pydantic import BaseModel, Field
from robogpt_apps.scripts.utilities.helper_services import ExternalServices
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, euler_from_matrix


# Define file paths for configuration and results
rospack = rospkg.RosPack()
base_dir = rospack.get_path('robogpt_agents')
driver_path = rospack.get_path('robot_drivers')
vision_path = rospack.get_path('robogpt_vision')

robot_home_file_path = os.path.join(base_dir,"config/robot_config/robot_pose.json")
robot_joint_file_path = os.path.join(base_dir,"config/robot_config/robot_joints.json")
robotgpt_config = os.path.join(base_dir,"config/robot_config/robogpt.json")
tour_paths = os.path.join(base_dir, "config/tour_scripts")
object_details = os.path.join(vision_path,"config/vision_config.json")