#!/usr/bin/python3

from robogpt_apps.scripts.utilities import utils
from langchain.tools import BaseTool
from typing import Type, List, Dict
from pydantic import BaseModel, Field
from robogpt_apps.scripts.base.skills import *

class auto_train_definition(BaseModel):
    object_to_look: str = Field(description="the object which it needs to look or needs to train the model at")
    object_name: str = Field(description="the name which needs to be alloted or labeled to the object on which the model is being trained")
    forget_old :bool = Field(default= True, description="the boolean which means robot should forget all other objects then True and if it should remember old objects also then false")
    images_to_train: int = Field(default=10,description="The Number of images the robot needs to click/take to make a dataset for autotraining.")
    number_of_epochs: int = Field(default=3,description="It is the number of epochs the robot is allowed to use while training the dataset")

class auto_train_implementation(BaseTool):
    '''Tool to train the new objects via webapp'''
    name = "auto_train_objects"
    description= "The tool wchich trains a object detection model to remember the object once shown to it"
    args_schema:Type[BaseModel] = auto_train_definition


    def _run(self,object_to_look: str, object_name: str, forget_old: bool = True, images_to_train: int = 10,number_of_epochs: int = 3 ) -> str:
        object_to_look = object_to_look + "."
        print("label of the object::", object_name)
        print("Object to detect::", object_to_look)
        print("Deleting old objects:", forget_old)

        def update_new_weights(json_file, new_value,object_name,old_data_path):
            # Read the existoing JSON data
            with open(json_file, 'r') as file:
                data = json.load(file)
            
            # Update the value for the key 'new_weights'
            data['new_weights'] = new_value
            data['objects_to_detect'].append(object_name)
            data['old_data_set'] = old_data_path

            # Write the updated JSON data back to the file
            with open(json_file, 'w') as file:
                json.dump(data, file, indent=4)
        
        def read_old_path(json_file):
            # Read the JSON data from the file
            with open(json_file, 'r') as file:
                data = json.load(file)
            
            # Return the value for the key 'new_weights'
            return data.get('old_data_set')
        previous_data_set = read_old_path(object_details)
        new_weights_path = ExternalServices().send_auto_train_goal(combined_folder="home/aion/train30",prev_data_folder=previous_data_set, object_name=object_to_look, object_label=object_name, new_weights=forget_old, image_threshold=images_to_train, epochs=number_of_epochs)
        
        return new_weights_path
    
class scan_object_definition(BaseModel):
    pose_list: List[str] = Field(default=[""],description="The list of poses robot needs to go to scan the object from different angles around it")

class scan_object_implementation(BaseTool):
    name = "scanning_object"
    description = "The tool to scan the surroundings or near objects for making a octamap for moviet"
    args_schema:Type[BaseModel] = scan_object_definition

    def _run(self,pose_list: List[str] = [""]):
        send_message_to_webapp_implementation()._run(f"Scanning the area using {pose_list} poses")
        if pose_list[0] == "":
            pose_list = ["scan1","scan2","scan3","home"]
        try:
            for i in range(len(pose_list)):
                response = move_to_pose_implementation()._run(pose_name=pose_list[i])

            return response
        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Unable to Scan area due to {e}")


class update_ply_file_defination(BaseModel):
    url: str = Field(default="",description="The url of the file which needs to be used")
    pose: list = Field(default=[0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0], description=" the pose of the the background you need to fix in envoirment")
class update_ply_file_implementation(BaseTool):
    name = "updating_ply_file"
    description = "The tool to update the background projection or simulation envoirment for the dalus viewer"
    args_schema:Type[BaseModel] = update_ply_file_defination

    def _run(self,url: str = "", pose: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        ply_path = rospy.get_param("/attachment_path",default="/home/owl/assets/ow_bg.ply")

        print(ply_path)
        filename = os.path.basename(ply_path)
        print(filename)
        rospy.set_param("/updated_ply_file_path",f"/root/owl_assets/{filename}")
        rospy.set_param("bg_pose", pose)
        return "Changing envoirment can take some time. Till then you can ask robogpt for other task. When change is done it will reflect in live window"