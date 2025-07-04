#!/usr/bin/python3

import os
import rclpy
import zipfile
import getpass
import json
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor


def extract_xml_from_zip(zip_path, extract_to_folder):
    """
    Extracts XML files from a given zip file into a subfolder named after the zip file.
    Saves their paths in a list.

    :param zip_path: Path to the zip file.
    :param extract_to_folder: Base folder where the files will be extracted.
    :return: A list of paths to the extracted XML files.
    """
    xml_paths = []  # To store paths of extracted XML files

    zip_file_name = os.path.basename(zip_path)  # Get the name of the zip file
    zip_file_name_without_ext = os.path.splitext(zip_file_name)[0]  # Remove the extension
    # Full path for the new subdirectory
    full_extract_path = os.path.join(extract_to_folder, zip_file_name_without_ext)

    # Create the subdirectory if it doesn't exist
    if not os.path.exists(full_extract_path):
        os.makedirs(full_extract_path)

    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        # Extract all files in the zip to the subdirectory
        zip_ref.extractall(full_extract_path)
        # Loop through the file names
        for file_name in zip_ref.namelist():
            # Check if the file is an XML
            if file_name.endswith('.xml'):
                # Save the full path of the extracted XML file
                xml_paths.append(os.path.join(full_extract_path, file_name))

    print("All the program files extracted")

    return xml_paths


def is_robot_connected(ip: str) -> bool:
    """
    Check if the robot is connected via Ethernet by pinging its IP address.

    Args:
        ip (str): IP address of the robot.

    Returns:
        bool: True if the robot is connected, False otherwise.
    """
    # Ping the IP address with 1 packet and wait for a response
    command = f"ping -c 1 -W 1 {ip} > /dev/null 2>&1"
    # Execute the command
    response = os.system(command)
    # Return True if the ping was successful (exit code 0), False otherwise
    return response == 0

def get_delta_poses(aruco_pose, target_position="all"):
    """
    Calculate multiple poses relative to an ArUco marker position using predefined offsets.
    
    Args:
        aruco_pose (list): List containing [x, y, z] coordinates of the ArUco marker
        target_position (str): Specific position to get offset for (e.g., "button", "pick").
                             Default "all" returns all positions.
        
    Returns:
        list: List of geometry_msgs/Pose objects, each representing a target pose
              calculated by applying the offsets to the ArUco marker position
    """
    if MasterAgentNode.has_parameter("robot_model"):
        robot_name = MasterAgentNode.get_parameter("robot_model").value
    else:
        robot_name = "owl65"

    # Get the path to the package containing the JSON file
    package_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_perception"
    
    # Construct the full path to the JSON file
    json_path = os.path.join(package_path, 'vision_config/offset_poses.json')
    
    try:        
        # Load offsets from JSON file
        with open(json_path, 'r') as f:
            offsets = json.load(f)
        
        # Handle named positions
        if target_position != "all":
            if target_position in offsets:
                offset = offsets[target_position]
                delta_pose = [0.0] * 6
                delta_pose[0] = aruco_pose[0] + offset['x']     # Add gripper offsets
                delta_pose[1] = aruco_pose[1] + offset['y']     # Add gripper offsets
                delta_pose[2] = aruco_pose[2] + offset['z']     # Add gripper offsets
                delta_pose[3] = offset['rx']
                delta_pose[4] = offset['ry']
                delta_pose[5] = offset['rz']
                return delta_pose
            else:
                self.logger.warn(f"Position '{target_position}' not found in offsets")
        else:
            # Process all positions
            delta_poses = []
            for position_name, offset in offsets.items():
                delta_pose = [
                    aruco_pose[0] + offset['x'],    # Add gripper offsets  
                    aruco_pose[1] + offset['y'],    # Add gripper offsets        
                    aruco_pose[2] + offset['z']     # Add gripper offsets    
                ]
                delta_pose[3] = offset['rx']
                delta_pose[4] = offset['ry']
                delta_pose[5] = offset['rz']
                delta_poses.append(delta_pose)
        
            return delta_poses
    
    except Exception as e:
        self.logger.info(f"Error processing poses: {str(e)}")
        return []

