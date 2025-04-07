import os
import sys
import cv2
import json
import time
import rospy
import pusher
import logging
import zipfile
import rospkg
import datetime
from std_msgs.msg import Bool


def set_logger():
    '''
    Configures and initializes two loggers for logging robot state and operation information. 
    It creates separate log files for each logger and sets up the log format.

    Returns:
    robotlogger (logging.Logger): A logger for robot state information.
    logger (logging.Logger): A logger for operation information.
    '''

    # Creates a folder in /log based on current date
    folder = datetime.datetime.now().strftime('log_%d_%m_%Y')
    os.makedirs(os.getcwd()+"/logs/"+folder, exist_ok=True)

    robotlogger = logging.getLogger("robot state")
    # Configure the log file and format for this script
    file_handler = logging.FileHandler(os.getcwd()+"/logs/"+folder+"/robot_state.log")
    formatter = logging.Formatter('%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s')
    file_handler.setFormatter(formatter)
    robotlogger.setLevel(logging.INFO)
    robotlogger.addHandler(file_handler)

    logger = logging.getLogger("operation")
    # Configure the log file and format for this script
    file_handler = logging.FileHandler(os.getcwd()+"/logs/"+folder+"/operation.log")
    formatter = logging.Formatter('%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s')
    logger.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return robotlogger,logger

robot_logger, opt_logger = set_logger()

# def send_image_once(image):
#     # Generate a unique name for the image
#     image_name = f"image_{int(time.time())}.jpg"
#     # Upload the image to Azure Blob Storage
#     image_url = upload_image_to_azure(image, image_name)
#     # Send the image URL via Pusher
#     send_msg(image_url)
#     return image_url

def get_current_objects():

    detection_file = "config/owl/detection_results.json"  
    # Load detection results
    with open(detection_file, "r") as file:
        detection_results = json.load(file)
    objects = []

    # Extract object names
    for key, detections in detection_results.items():
        for detection_key, detection in detections.items():
            objects.append(detection["detected_object"])

    # Create a formatted string
    if not objects:
        return "No objects detected in the frame."
    
    unique_objects = set(objects)  # Remove duplicates if needed
    object_list_str = ", ".join(unique_objects)
    formatted_string = f"I see the following object(s) in the frame: {object_list_str}."
    return object_list_str

def publish_pass_bool(val: bool, topic: str = "/pass_topic", queue_size: int = 1) -> None:
    """
    Publishes a boolean message on a specified ROS topic.

    This function creates a ROS publisher for the given topic and publishes
    the boolean value. A brief delay is included to allow the publisher to
    register with the ROS network before sending the message.

    Args:
        val (bool): The boolean value to publish.
        topic (str): The ROS topic to which the message is published.
                     Defaults to "/boolean_topic".
        queue_size (int): The publisher queue size. Defaults to 10.
    """
    pub = rospy.Publisher(topic, Bool, queue_size=queue_size,latch=True)
    # Allow time for the publisher to register with any subscribers
    rospy.sleep(0.5)
    rospy.loginfo("Publishing bool: %s on topic %s", val, topic)
    pub.publish(Bool(data=val))


def get_skill_template():
    return '''
class {{ class_name }}_definition(BaseModel):
    object: str = Field(default = None, descrption = "the object or device to which robot needs to press button")
class {{ class_name }}_implementation(BaseTool):
    """Tool to tell the position of the object"""
    name = "{{ tool_name }}"
    description = "{{ tool_description }}"
    args_schema: Type[BaseModel] = {{ class_name }}_definition

    def _run(self, object:str = None):
        robot_ip = {{ robot_ip }}
        send_message_to_webapp_implementation()._run(message="Sure! I have instructed the robot to make popcorn")
'''

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


