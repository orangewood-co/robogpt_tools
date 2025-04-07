#!/usr/bin/python3

from robogpt_apps.scripts.utilities import utils
from langchain.tools import BaseTool
from typing import Type, List, Dict
from pydantic import BaseModel, Field
from robogpt_apps.scripts.base.skills import *

# Data model for the 'place_object' tool
class place_object_definition(BaseModel):
    drop_name : str = Field(default=None,description="The name of the pose on which robot needs to place the object")
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")
    direction: str = Field(default=None,description="The direction in which robot needs to place the object")
    drop_object:str = Field(default=None,description="the object on which we need to place the object")

# Implementation of the 'place_object' tool
class place_object_implementation(BaseTool):
    name = "place_object"
    description="Tthe tool to place any object with robot"
    args_schema: Type[BaseModel] = place_object_definition

    def _run(self,drop_name: str = None, drop_object: str = None, robot_to_use: int = 1,direction: str = None):

        home = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="home")

        if drop_object is not None:
            
            try: 
                target = check_pose_implementation()._run(object=drop_object)
                if target is None:
                    send_message_to_webapp_implementation()._run(f"Could Not find the {drop_object}. Placing Object in trash")
                    trash = get_zone_pose_implementation()._run(zone_name="trash",robot_to_use=robot_to_use)
                    move_to_pose_implementation()._run(goal_pose=trash)
                    control_gripper_implementation()._run(switch=True)
                    move_to_pose_implementation()._run(goal_pose=home)

                target = target.copy()
                target[2] = target[2] + 0.10
                move_to_pose_implementation()._run(goal_pose=target)
                control_gripper_implementation()._run(switch=True)
                return True
            except Exception as e:
                send_message_to_webapp_implementation()._run(f"Error in placing due to {e}")

        if drop_name is not None:
            try: 
                pose = get_zone_pose_implementation()._run(zone_name=drop_name,robot_to_use=robot_to_use)
                move_to_pose_implementation()._run(goal_pose=pose)
                control_gripper_implementation()._run(switch=True)
                move_to_pose_implementation()._run(goal_pose=home)

                return True
            except Exception as e:
                send_message_to_webapp_implementation()._run(f"Error in placing due to {e}")

        if direction is not None:   
            send_message_to_webapp_implementation()._run(message=f"Currently I don't have a sense of direction. Could you please specify an object or dropping pose name.")
            move_to_pose_implementation()._run(goal_pose=home)

            return False
        

# Data model for the 'pick_object' tool
class pick_object_definition(BaseModel):
    object: str = Field(description="object which the robot needs to pick up")
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")

# Implementation of the 'pick_object' tool
class pick_object_implementation(BaseTool):
    '''Tool to pick objects'''
    name = "pick_object"
    description = "Tool to pick up the object mentioned in the prompt"
    args_schema: Type[BaseModel] = pick_object_definition

    def _run(self, object: str, robot_to_use: int = 1):

        try:
            home = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="home")
            int_pose = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="int_pose")

            move_to_pose_implementation()._run(goal_pose=home)
            pose = check_pose_implementation()._run(object=object)
            control_gripper_implementation()._run(switch=True)

            if pose is None:
                utils.send_msg(message="Unable to detect the object. Please try again!")
                move_to_pose_implementation()._run(goal_pose=home, robot_to_use=1)

            move_to_pose_implementation()._run(goal_pose=pose)
            control_gripper_implementation()._run(switch=False)

            if int_pose is not None:
                move_to_pose_implementation()._run(goal_pose=int_pose)

            else:
                send_message_to_webapp_implementation()._run(f"Intermediate pose not defined taking it to home position.")
                move_to_pose_implementation()._run(goal_pose=home)


        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Unable to pick {object} due to {e}")

         
class clean_table_definition(BaseModel):
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")
    drop_pose: str = Field(default="bin",description="The name of the pose on which robot needs to place the object")
class clean_table_implementation(BaseTool):
    name = "clean_table"
    description="The tool to clean or pick up all the stuff and place it in the bin or bucket"
    args_schema: Type[BaseModel] = clean_table_definition

    def _run(self,robot_to_use: str = 1,drop_pose: str = "bin"):
        objects = get_object_list_implementation()._run()
        print(objects)
        for object in objects:
            print(object)
            if object != "bucket":
                pick_object_implementation()._run(object=object)
                place_object_implementation()._run(drop_name=drop_pose,robot_to_use=robot_to_use)

        home = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="home")
        move_to_pose_implementation()._run(goal_pose=home)


