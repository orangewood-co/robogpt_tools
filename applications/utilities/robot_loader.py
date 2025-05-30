#!/usr/bin/python3
print("loading robots")
import json
import os
import sys
import getpass
import importlib
from difflib import SequenceMatcher
from robogpt_tools.applications.utilities.skill_initializers import Colors
from robogpt_tools.applications.utilities.utils import *

class RobotLoader():
    def __init__(self):
        print(f"{Colors.GREEN}Initializing robot loader{Colors.RESET}")
        self.robots = []
        self.robot_list = []
        self.config_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools/robot_config/bot_details.json"
        self.robotgpt_config = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools/robot_config/robogpt.json"
        self.driver_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools/robot_drivers"

    def is_robot_connected(self, ip: str) -> bool:
        # Ping the IP address with 1 packet and wait for a response
        command = f"ping -c 1 -W 1 {ip} > /dev/null 2>&1"
        response = os.system(command)
        return response == 0

    def get_matched_robot(self, robot_name: str) -> str:
        with open(self.config_path, 'r') as file:
            bot_dict = json.load(file)
        bot_list = bot_dict["bot_list"]
        best_match = max(bot_list, key=lambda obj: SequenceMatcher(None, robot_name.lower(), obj).ratio(), default=None)
        return best_match

    def load_robots(self):
        try:
            # Use ROS2 parameter API
            config_data = get_single_message(topic_name="robot_config")
            robot_model = config_data.robot_name
            print(f"Robot name is {robot_model}")
            print("Robot model is ",robot_model)
            sim_flag = config_data.use_sim
            print(f"{Colors.GREEN}Loading robot model: {robot_model}{Colors.RESET}")
            with open(self.config_path, "r") as file:
                data = json.load(file)

            bot = self.get_matched_robot(robot_name=robot_model)
            self.robot_list.append(robot_model)
            print(f"Updating the wrapper of {bot}")
            wrapper_path = "robot_drivers.wrappers.sim_wrapper" if sim_flag else f"robot_drivers.wrappers.{bot}_wrapper"

            # Append the parent directory of 'wrappers' to sys.path
            wrappers_parent_dir = os.path.dirname(self.driver_path)
            if wrappers_parent_dir not in sys.path:
                sys.path.append(wrappers_parent_dir)

            bot_control = importlib.import_module(wrapper_path)
            wrapper = getattr(bot_control, 'bot_wrapper')

            if sim_flag:
                robot_config = data[robot_model]
                self.robots.append(wrapper(
                    robot_config["robot_group"],
                    robot_config["gripper_group"],
                    robot_config["time_out"],
                    robot_config["gripper_enable"]
                ))
            else:
                if self.is_robot_connected(data[robot_model]["robot_ip"]):
                    self.robots.append(wrapper(data[robot_model]["robot_ip"]))
                else:
                    print("Please check the robot connection and try again")

        except Exception as err:
            print(f"Could not load robot due to {err}")
        
        return self.robot_list, self.robots