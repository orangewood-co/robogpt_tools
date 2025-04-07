#!/usr/bin/python3

import rospy
import getpass
from gazebo_msgs.srv import SpawnModel, GetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from robogpt_apps.scripts.utilities import utils
from langchain.tools import BaseTool
from typing import Type, List, Dict
from pydantic import BaseModel, Field
from difflib import SequenceMatcher
from robogpt_apps.scripts.base.skills import *

#################### Utility Functions and Constants ###################
model_dir = f'/home/{getpass.getuser()}/orangewood_ws/src/robogpt_apps/scripts/gazebo_skills/models'

def get_best_model_match(model_name, list):
        best_match = None
        best_ratio = 0
        for obj in list:
            ratio = SequenceMatcher(None, model_name, obj).ratio()
            if ratio > best_ratio:
                best_ratio = ratio
                best_match = obj
                    
        return best_match

def list_folders(directory):
    try:
        # List only directories in the given directory
        folders = [folder for folder in os.listdir(directory) if os.path.isdir(os.path.join(directory, folder))]
        return folders
    except FileNotFoundError:
        print(f"The directory '{directory}' does not exist.")
        return []
    except Exception as e:
        print(f"An error occurred: {e}")
        return []

def get_current_model_names():
    """
    Function to get the names of all models present in the current Gazebo simulation.
    
    :return: List of model names.
    """
    # Wait for the /gazebo/model_states topic to be published
    rospy.loginfo("Waiting for /gazebo/model_states topic...")
    rospy.wait_for_message("/gazebo/model_states", ModelStates)
    
    # Get the model states message
    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    
    # Extract the model names
    model_names = model_states.name
    rospy.loginfo(f"Model names in Gazebo: {model_names}")
    return model_names
#####################################################



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
                target[2] = target[2] + 0.12
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
    model_name: str = Field(description="The name of model which robogpt needs to spawn in gazebo")
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")

# Implementation of the 'pick_object' tool
class pick_object_implementation(BaseTool):
    '''Tool to pick objects'''
    name = "pick_object"
    description = "Tool to pick up the object mentioned in the prompt"
    args_schema: Type[BaseModel] = pick_object_definition

    def _run(self, model_name: str, robot_to_use: int = 1):

        try:
            home = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="home")
            int_pose = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="int_pose")

            move_to_pose_implementation()._run(goal_pose=home)
            pose = check_model_pose_implementation()._run(model_name=model_name)
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


class check_model_implementation(BaseTool):
    name = "check_model_in_gazebo"
    description= "The tool to give the list of all the models robogpt have in it's arsenal or all the saved models"

    def _run(self):
    
        models_dir = f'/home/{getpass.getuser()}/orangewood_ws/src/robogpt_apps/scripts/gazebo_skills/models'

        def list_folders(directory):
            try:
                # List only directories in the given directory
                folders = [folder for folder in os.listdir(directory) if os.path.isdir(os.path.join(directory, folder))]
                return folders
            except FileNotFoundError:
                print(f"The directory '{directory}' does not exist.")
                return []
            except Exception as e:
                print(f"An error occurred: {e}")
                return []
            

        try:
            return list_folders(models_dir)
        except:
            return Exception

class get_current_sim_models_definition(BaseModel):
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")

class get_current_sim_models_implementation(BaseTool):
    name = "get_current_models"
    description= "The tool to give the list of all the models which are currently present in the simulation"

    def _run(self,robot_to_use: int = 1):
        
        def get_current_model_names():
            """
            Function to get the names of all models present in the current Gazebo simulation.
            
            :return: List of model names.
            """
            # Wait for the /gazebo/model_states topic to be published
            rospy.loginfo("Waiting for /gazebo/model_states topic...")
            rospy.wait_for_message("/gazebo/model_states", ModelStates)
            
            # Get the model states message
            model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
            
            # Extract the model names
            model_names = model_states.name
            rospy.loginfo(f"Model names in Gazebo: {model_names}")
            return model_names

        try:
            return get_current_model_names()
        except:
            return Exception


class check_model_pose_defination(BaseModel):
    model_name: str = Field(description="The name of model which robogpt needs to spawn in gazebo")

class check_model_pose_implementation(BaseTool):
    name = "check_model_position_in_gazebo"
    description= "The tool to give the position of model in gazebo"
    args_schema: Type[BaseModel] = check_model_pose_defination

    def _run(self,model_name: str):

        def get_model_pose_as_list(model_name):
            """
            Function to get the pose of a specific model from the /gazebo/model_states topic as a list.
            
            :param model_name: Name of the model to get the pose for.
            :return: Pose of the model as a list [x, y, z, qx, qy, qz, qw], or None if not found.
            """
            # Wait for the /gazebo/model_states topic to be published
            rospy.loginfo("Waiting for /gazebo/model_states topic...")
            rospy.wait_for_message("/gazebo/model_states", ModelStates)
            
            # Get the model states message
            model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
            
            # Check if the model is in the list
            if model_name in model_states.name:
                index = model_states.name.index(model_name)
                pose = model_states.pose[index]
                
                # Convert pose to a list
                pose_list = [
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                ]
                pose_list[0] = pose_list[0] - 0.4
                pose_list[2] = pose_list[2] - 0.95 

                rospy.loginfo(f"Pose of {model_name}: {pose_list}")
                return pose_list
            else:
                rospy.logwarn(f"Model '{model_name}' not found in /gazebo/model_states")
                return None

        model_list = get_current_model_names()
        model_name = get_best_model_match(model_name=model_name,list=model_list)
        print(model_name)
        return get_model_pose_as_list(model_name)

class spawn_model_defination(BaseModel):
    model_name: str = Field(description="The name of model which robogpt needs to spawn in gazebo")
    model_pose: List[float] = Field(description="the pose on which the model is to be spawned. This pose is a list of x,y,z") 

class spawn_model_implementation(BaseTool):
    name = "spawn_model_in_gazebo"
    description= "The tool to spawn, add or include models in gazebo simulation or any other simulation"
    args_schema:Type[BaseModel] = spawn_model_defination

    def _run(self,model_name: str, model_pose: List[float]):
    

        def spawn_model(model_dir,model_name,model_pose):

            model_path = f'{model_dir}/{model_name}/model.sdf'
            model_path = rospy.get_param('model_path', model_path)

            with open(model_path, 'r') as file:
                model_xml = file.read()

            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                pose = Pose()
                pose.position.x = model_pose[0]
                pose.position.y = model_pose[1]
                pose.position.z = model_pose[2]
                response = spawn_model(model_name, model_xml, "", pose, "world")
                return response.status_message
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        
        try:
            model_list = list_folders(model_dir)
            model_name = get_best_model_match(model_name,model_list)

            if model_name:
                return spawn_model(model_dir,model_name,model_pose)
            else:
                err = "No suitable match found."
                return err

        except Exception as e:
            return e
        

class scan_area_definition(BaseModel):
    pose_list: List[str] = Field(default=[""],description="The list of poses robot needs to go to scan the area around it")

class scan_area_implementation(BaseTool):
    name = "scanning_area"
    description = "The tool to scan the surroundings or near objects for making a octamap for moviet"
    args_schema:Type[BaseModel] = scan_area_definition

    def _run(self,pose_list: List[str] = [""]):
        send_message_to_webapp_implementation()._run(f"Scanning the area using {pose_list} poses")
        if pose_list[0] == "":
            pose_list = ["scan1","scan2","scan3","home"]
        try:
            for i in range(len(pose_list)):
                response = move_to_joint_implementation()._run(pose_name=pose_list[i])

            return response
        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Unable to Scan area due to {e}")

# class pick_up_object_definition(BaseModel):
#     model_name: str = Field(description="The name of model which robogpt needs to spawn in gazebo")

# class pick_up_object_implementation(BaseTool):
#     name = "pick_up_object"
#     description="The tool to pick up objects or models in the simulation"
#     args_schema: Type[BaseModel] = pick_up_object_definition

#     def _run(self,model_name: str):
#         model_pose = check_model_pose_implementation()._run(model_name=model_name)
#         home = get_zone_pose_implementation()._run(zone_name="home")
#         move_to_pose_implementation()._run(goal_pose=home)
#         move_to_pose_implementation()._run(goal_pose=model_pose)
#         control_gripper_implementation()._run(switch=False)
#         move_to_pose_implementation()._run(goal_pose=home)
