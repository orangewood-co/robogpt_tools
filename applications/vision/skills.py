import time
import rclpy
import difflib
import yaml
from rclpy.parameter import Parameter
from typing import ClassVar, Optional, List, Type
from pydantic import BaseModel, Field, ConfigDict
from langchain_core.tools import BaseTool
from robogpt_tools.applications.utilities.skill_initializers import *
from robogpt_tools.applications.utilities.robot_loader import RobotLoader
from robogpt_tools.applications.utilities import utils

from robogpt_tools.applications.base.skills import *

robot_list,robots = RobotLoader().load_robots()


####################
# ArUco Marker Tools
####################
class active_marker_definition(BaseModel):
    aruco_num: int = Field(
        default=1,
        description="the number of the aruco marker to use for the saving and calculation of offset poses"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class active_marker_implementation(BaseTool):
    """Set the active ArUco marker number for saving and calculating offset poses."""
    name: ClassVar[str] = "active_marker"
    description: ClassVar[str] = "the tool to set the aruco marker number to use for the saving and calculation of offset poses"
    args_schema: ClassVar[Type[BaseModel]] = active_marker_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, aruco_num: int = 1):
        MasterAgentNode.set_parameters([rclpy.parameter.Parameter("active_marker", rclpy.Parameter.Type.INTEGER, aruco_num)])


class get_aruco_pose_definition(BaseModel):
    cam_name: str = Field(
        default="camera",
        description="the camera name from which we need the pose of the object"
    )
    parent_frame: str = Field(
        default="base_link",
        description="the link with respect to which we need to get the pose of an object"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class get_aruco_pose_implementation(BaseTool):
    """Get the pose of the aruco marker with respect to the robot base."""
    name: ClassVar[str] = "get_aruco_pose"
    description: ClassVar[str] = "Get the pose of the aruco marker with respect to the robot base."
    args_schema: ClassVar[Type[BaseModel]] = get_aruco_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, cam_name: str = "camera", parent_frame: str = "base_link"):
        """
        Get and save ArUco marker pose with marker ID and position data.
        The pose is saved as aruco_1, aruco_2, etc. with marker ID and position.
        """
        try:
            # Get paths
            package_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_perception"
            aruco_results_path = os.path.join(package_path, "vision_config/detection_results.json")
            aruco_pose_path = os.path.join(package_path, "vision_config/aruco_pose.json")

            # Get marker ID from results
            try:
                with open(aruco_results_path, "r") as file:
                    aruco_results = json.load(file)
                    marker_id = None
                    
                    # Check in each camera's data
                    for camera in aruco_results:
                        if "aruco" in aruco_results[camera]:
                            aruco_data = aruco_results[camera]["aruco"]
                            if "marker_id" in aruco_data:
                                marker_id = aruco_data["marker_id"]
                                self.logger.info(f"Found ArUco marker {marker_id} in camera {camera}")
                                break
                    
                    if marker_id is None:
                        self.logger.warn("No ArUco markers with ID found in any camera")
                        return None

            except Exception as e:
                self.logger.error(f"Error reading ArUco results: {str(e)}")
                return None

            # Get the ArUco marker pose
            aruco_pose = ExternalServices().call_get_world_context(
                object_name="aruco", 
                parent_frame=parent_frame, 
                camera_name=cam_name
            )

            if aruco_pose is None:
                raise ValueError("Failed to retrieve ArUco pose.")
            
            try:
                # Load existing pose data
                try:
                    with open(aruco_pose_path, "r") as file:
                        aruco_pose_data = json.load(file)
                except FileNotFoundError:
                    aruco_pose_data = {}

                # Check if this marker_id exists in any saved poses
                existing_entry = None
                for key, data in aruco_pose_data.items():
                    if data.get("marker_id") == marker_id:
                        existing_entry = key
                        break

                if existing_entry:
                    # Update existing entry
                    aruco_name = existing_entry
                else:
                    # Create new entry with next available number
                    i = 1
                    while f"aruco_{i}" in aruco_pose_data:
                        i += 1
                    aruco_name = f"aruco_{i}"

                # Save/update the pose data with marker ID
                aruco_pose_data[aruco_name] = {
                    "marker_id": marker_id,
                    "pose": {
                        "x": aruco_pose[0],
                        "y": aruco_pose[1],
                        "z": aruco_pose[2]
                    }
                }

                # Save updated data
                with open(aruco_pose_path, "w") as file:
                    json.dump(aruco_pose_data, file, indent=4)

                self.logger.info(f"ArUco marker {marker_id} pose saved as {aruco_name}: {aruco_pose_data[aruco_name]}")
                send_message_to_webapp_implementation()._run(success_msg)
                return aruco_pose

            except Exception as e:
                self.logger.error(f"Error saving ArUco pose: {str(e)}")
                send_message_to_webapp_implementation()._run(f"Could not save ArUco pose: {str(e)}")
                return None

        except Exception as e:
            self.logger.error(f"Error in get_aruco_pose: {str(e)}")
            send_message_to_webapp_implementation()._run(f"Error in get_aruco_pose: {str(e)}")
            return None


class save_offset_pose_definition(BaseModel):
    pose_name: Optional[str] = Field(
        default=None,
        description="The name of the pose to which we need to save the offset"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class save_offset_pose_implementation(BaseTool):
    """Calculate and save the offset of the current pose from ArUco pose with respect to the robot base."""
    name: ClassVar[str] = "save_offset_pose"
    description: ClassVar[str] = "calculate and save the offset of the current pose from aruco pose with respect to the robot base"
    args_schema: ClassVar[Type[BaseModel]] = save_offset_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, pose_name: str = None):

        package_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_perception"
        offset_path = os.path.join(package_path, "vision_config/offset_poses.json")
        
        if MasterAgentNode.has_parameter("active_marker"):
            aruco_number = MasterAgentNode.get_parameter("active_marker").value
        else:
            aruco_number = 1
        
        aruco_name = f"aruco_{aruco_number}"
        try:
            # Initialize an empty list for the pose
            offset_pose = []

            # Get the current TCP pose
            current_pose = get_pose_implementation()._run(robot_to_use=1)
            print(current_pose)

            aruco_pose_path = os.path.join(package_path, "vision_config/aruco_pose.json")

            try:
                with open(aruco_pose_path, "r") as file:
                    aruco_pose_data = json.load(file)
                    if aruco_name not in aruco_pose_data:
                        raise ValueError(f"No pose data found for {aruco_name}")
                    self.logger.info(f"ArUco marker id: {aruco_pose_data[aruco_name]['marker_id']}")
                    aruco_pose = [
                        aruco_pose_data[aruco_name]["pose"]["x"],
                        aruco_pose_data[aruco_name]["pose"]["y"],
                        aruco_pose_data[aruco_name]["pose"]["z"]
                    ]
            except Exception as e:
                self.logger.error(f"Error reading ArUco pose from JSON file: {str(e)}")
                send_message_to_webapp_implementation()._run(
                    f"Could not read the ArUco pose from JSON file due to: {str(e)}"
                )
                return None

            # Ensure both poses are valid
            if current_pose is None or aruco_pose is None:
                raise ValueError("Failed to retrieve current pose or ArUco pose.")

            # Calculate the offset (x, y, z)
            offset_pose = [
                current_pose[0] - aruco_pose[0],
                current_pose[1] - aruco_pose[1],
                current_pose[2] - aruco_pose[2]
            ]

            # Prepare the data to write to the JSON file
            data_to_save = {
                "pose_name": pose_name if pose_name else "unnamed_pose",
                "offset_pose": {
                    "x": offset_pose[0],
                    "y": offset_pose[1],
                    "z": offset_pose[2],
                    "rx": current_pose[3],
                    "ry": current_pose[4],
                    "rz": current_pose[5]
                }
            }

            print(f"Offset pose data to save: {data_to_save}")

            # Load existing data if the file exists
            try:
                with open(offset_path, "r") as file:
                    existing_data = json.load(file)
            except FileNotFoundError:
                existing_data = {}

            # Update the data with the new pose
            existing_data[data_to_save["pose_name"]] = data_to_save["offset_pose"]

            # Write the updated data back to the JSON file
            with open(offset_path, "w") as file:
                json.dump(existing_data, file, indent=4)

            # Log success and return the offset pose
            self.logger.info(f"Offset pose saved successfully: {data_to_save}")
            send_message_to_webapp_implementation()._run(f"Offset pose named {pose_name} has been saved successfully.")
            return offset_pose

        except Exception as e:
            # Handle errors and log them
            self.logger.error(f"Error in calculating or saving offset pose: {str(e)}")
            send_message_to_webapp_implementation()._run(
                f"Could not calculate or save the offset pose due to: {str(e)}"
            )
            return None

    
class calculate_offset_pose_definition(BaseModel):
    target_position: str = Field(
        default="all",
        description="the target position for the robot to calculate pose of"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class calculate_offset_pose_implementation(BaseTool):
    """Calculate the offset pose of a target position with respect to the ArUco marker pose."""
    name: ClassVar[str] = "calculate_offset_pose"
    description: ClassVar[str] = "This tool is used to detect the aruco marker and get the relative position of the target position with respect to the aruco marker"
    args_schema: ClassVar[Type[BaseModel]] = calculate_offset_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, target_position:str = "all"):
        try:
            package_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_perception"
            aruco_pose_path = os.path.join(package_path, "vision_config/aruco_pose.json")
            
            if MasterAgentNode.has_parameter("active_marker"):
                aruco_number = MasterAgentNode.get_parameter("active_marker").value
            else:
                aruco_number = 1

            aruco_name = f"aruco_{aruco_number}"

            try:
                with open(aruco_pose_path, "r") as file:
                    aruco_pose_data = json.load(file)
                    if aruco_name not in aruco_pose_data:
                        raise ValueError(f"No pose data found for {aruco_name}")
                    self.logger.info(f"ArUco marker id: {aruco_pose_data[aruco_name]['marker_id']}")
                    aruco_pose = [
                        aruco_pose_data[aruco_name]["pose"]["x"],
                        aruco_pose_data[aruco_name]["pose"]["y"],
                        aruco_pose_data[aruco_name]["pose"]["z"]
                    ]
            except Exception as e:
                self.logger.error(f"Error reading ArUco pose from JSON file: {str(e)}")
                send_message_to_webapp_implementation()._run(
                    f"Could not read the ArUco pose from JSON file due to: {str(e)}"
                )
                return None
            
            delta_poses = utils.get_delta_poses(aruco_pose, target_position=target_position)
            self.logger.info(f"ArUco offset pose for {target_position} read successfully")
            send_message_to_webapp_implementation()._run(f"ArUco offset pose for {target_position} read successfully")
            return delta_poses

        except Exception as e:
            self.logger.error(f"Error in calculate_offset_pose: {str(e)}")
            send_message_to_webapp_implementation()._run(f"Error in calculate_offset_pose: {str(e)}")
            return None



################################################
# Analyze, compare images and set context camera
################################################

class analyze_camera_frame_definition(BaseModel):
    prompt: Optional[str] = Field(
        default=None,
        description="A natural language question or instruction describing what to analyze in the camera feed. Also check if user want answer in one word or number."
    )
    perform_ocr: bool = Field(
        default=False,
        description="True if you want the 4o model to read/analyze the text in the camera feed."
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class analyze_camera_frame_implementation(BaseTool):
    """Analyze the camera frame and return the result based on the prompt."""
    name: ClassVar[str] = "analyze_camera_frame"
    description: ClassVar[str] = "The tool to see what is around the robot from camera"
    args_schema: ClassVar[Type[BaseModel]] = analyze_camera_frame_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, prompt: str = None, perform_ocr: bool = False):
        new_prompt = f"{prompt}, answer in one word or number"
        success, result, elapsed_time = ExternalServices().analyze_image(prompt, perform_ocr)
        if success:
            self.loggerinfo(f"Analysis result: {result}")
            return result
        else:
            self.logger.error(result)
            send_message_to_webapp_implementation()._run("Failed to analyze the camera frame.")
            return result


class set_context_camera_definition(BaseModel):
    camera_name: Optional[str] = Field(
        default=None,
        description="The name of the camera to set as context."
    )
    purpose: str = Field(
        default="context",
        description="The purpose for which camera is being used. You only have two option for value context or detection"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class set_context_camera_implementation(BaseTool):
    """Set the camera name which robot will use to get the context / understanding of the surroundings."""
    name: ClassVar[str] = "set_context_camera"
    description: ClassVar[str] = "The tool to set the camera name which robot will use to get the context / understanding of the surroundings"
    args_schema: ClassVar[Type[BaseModel]] = set_context_camera_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, camera_name: str = None, purpose: str = "context"):
        
        def get_camera_names_from_yaml(yaml_file_path):
            if not os.path.exists(yaml_file_path):
                print(f"YAML file not found: {yaml_file_path}")
                return []
                
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file) or {}
            
            camera_names = []
            
            num_cameras = data.get("number_of_cams", 0)
            for i in range(1, num_cameras + 1):
                camera_key = f"camera_{i}"
                if camera_key in data:
                    camera_names.append(data[camera_key])
                    
            i = 1
            while True:
                rgb_cam_key = f"rgb_cam_{i}"
                if rgb_cam_key in data:
                    camera_names.append(data[rgb_cam_key])
                    i += 1
                else:
                    break
            
            return camera_names
        
        def find_closest_camera_match(search_string, camera_list):
            if not camera_list:
                return None
            matches = difflib.get_close_matches(search_string, camera_list, n=1, cutoff=0.1)
            if matches:
                return matches[0]
            return None
        
        if get_camera_names_from_yaml == []:
            return "No cameras found in the setup config. Please first configure the cameras."
        else:
            camera_list = get_camera_names_from_yaml(camera_config)
            if camera_name is not None:
                matched_camera = find_closest_camera_match(camera_name, camera_list)
                if matched_camera:
                    param_name = f"/{purpose}_camera"
                    print(param_name)
                    camera_name = f"{camera_name}/color/image_raw"
                    MasterAgentNode.set_parameters([rclpy.parameter.Parameter("param_name", rclpy.Parameter.Type.STRING, camera_name)])
                    return True


class compare_images_definition(BaseModel):
    prompt: Optional[str] = Field(
        default="Compare these two images and check if an object is attached to the robot gripper.",
        description="Natural language prompt describing what to compare between the images"
    )
    model_config = ConfigDict(arbitrary_types_allowed=True)

class compare_images_implementation(BaseTool):
    """Compare two images and return the difference between them based on the given prompt."""
    name: ClassVar[str] = "compare_images"
    description: ClassVar[str] = "Tool to compare two images and return the difference between them based on the given prompt"
    args_schema: ClassVar[Type[BaseModel]] = compare_images_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, prompt: str = None) -> str:
        """
        Compare two images and return analysis based on the prompt.
        Args:
            prompt (str): Comparison prompt. Uses default if None.
        Returns:
            str: Comparison result or error message
        """
        try:
            # Validate prompt
            if prompt is None:
                prompt = self.parameters["properties"]["prompt"]["default"]
            
            self.logger.info(f"Comparing images with prompt: {prompt}")

            # Call comparison service
            success, result, elapsed_time = ExternalServices().compare_images(prompt=prompt)

            if not success:
                error_msg = f"Image comparison failed: {result}"
                self.logger.error(error_msg)
                send_message_to_webapp_implementation()._run(error_msg)
                return error_msg

            # Log and return success result
            self.logger.info(f"Comparison complete in {elapsed_time:.2f}s: {result}")
            send_message_to_webapp_implementation()._run(result)
            return result

        except Exception as e:
            error_msg = f"Error during image comparison: {str(e)}"
            self.logger.error(error_msg)
            send_message_to_webapp_implementation()._run(error_msg)
            return error_msg
