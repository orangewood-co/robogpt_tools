# This file contains the implementation of the skills that are used in the RoboGPT application.

from robogpt_apps.scripts.utilities.skill_initializers import *
from robogpt_apps.scripts.utilities.robot_loader import RobotLoader

ROBOT_LIST,robots = RobotLoader().load_robots()
# Data model for the 'get_joint' tool
class get_joint_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    wait: bool = Field(default=True, description="True will allow to return the latest data received from the robot.")

# Implementation of the 'get_joint' tool
class get_joint_implementation(BaseTool):
    """Tool for retrieving joint values of the robot in radians"""
    name = "get_joint"
    description = "returns the current joint values of the robot"
    args_schema: Type[BaseModel] = get_joint_definition

    def _run(self, robot_to_use: int = 1, wait: bool = True, flag: bool = True) -> list:
        try:
            # robots[robot_to_use - 1].test()
            curr_joint_rads = robots[robot_to_use-1].get_joints()
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return curr_joint_rads
        
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_joint_implementation failed " + str(e))
            return None

    def _arun(self, robot_to_use: int, wait: bool = True):
        return("get_joint does not support async")


# Data model for the 'get_pose' tool
class get_pose_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    wait: bool = Field(default=True, description="True will allow to return the latest data received from the robot.")

# Implementation of the 'get_pose' tool
class get_pose_implementation(BaseTool):
    """Tool for retrieving end effector pose in meters"""
    name = "get_position"
    description = "returns the current pose of robot end effector"
    args_schema: Type[BaseModel] = get_pose_definition

    def _run(self, robot_to_use: int = 1, wait: bool = True, flag: bool = True) -> list:
        print("get_pose_implementation")
        try:
            curr_pose = robots[robot_to_use-1].get_tcp()
            print(f"current pose:: {curr_pose}")
 
            return curr_pose
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_pose_implementation failed " + str(e))
            return None

    def _arun(self, robot_to_use: int, wait: bool = True):
        return("get_joint does not support async")


# Data model for the 'get_zone_pose' tool
class get_zone_pose_definition(BaseModel):
    zone_name: str = Field(description="Target pose name where the robot needs to move.")
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    joint_space: bool = Field(default=False,description="Boolean to decide wheather to use joint space or cartesian space")
# Implementation of the 'get_zone_pose' tool
class get_zone_pose_implementation(BaseTool):
    """Tool to get the robot pose at a specific zone"""
    name = "get_zone_pose"
    description = "requests the pose of the robot at a particular zone"
    args_schema: Type[BaseModel] = get_zone_pose_definition

    def _run(self, zone_name: str, robot_to_use: int = 1, joint_space: bool = False, flag: bool = True) -> list:
        # Helper function to find the best matching zone name
        def get_best_zone_match(zone_name, robot_dict):
            best_match = None
            best_ratio = 0
            for obj in robot_dict.keys():
                ratio = SequenceMatcher(None, zone_name, obj).ratio()
                if ratio > best_ratio:
                    best_ratio = ratio
                    best_match = obj
            return best_match
        
        if joint_space:
           robot_dict = json.loads(open(robot_joint_file_path).read())
        else:
            robot_dict = json.loads(open(robot_home_file_path).read())
        
        try:
            robot_name = ROBOT_LIST[robot_to_use - 1]
            robot_pose = robot_dict[robot_name]
            if zone_name in robot_pose:
                zone_match = get_best_zone_match(zone_name, robot_pose)
                pose = robot_pose[zone_match]
            else:
                pose = None
                
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return pose
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_zone_pose_implementation failed " + str(e))
            return None

    def _arun(self, zone_name: str):
        print("move_to_pose does not support async")


# -----------------------------------------------------------------------------
#                       ROBOT TOOL CONTROL
# -----------------------------------------------------------------------------


# Data model for the 'hand_teach' tool
class hand_teach_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    config: bool = Field(default=False, description="The boolean which decides the robot is in gravity or hand teach mode or not. true puts the robot in hand teach and false switches the gravity /hand teach off")

# Implementation of the 'hand_teach' tool
class hand_teach_implementation(BaseTool):
    '''Tool to put robot in hand teach mode'''
    name = "hand_teach_init"
    description = "It switches the hand teach/gravity mode on or off based on the config bool value"
    args_schema: Type[BaseModel] = hand_teach_definition

    def _run(self, robot_to_use: int = 1, config: bool = False, flag: bool = True):
        try:
            response = robots[robot_to_use-1].set_hand_teach(config)
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return response

        except Exception as e:
            print("Robot is unable to switch in Hand teach")

# Data model for the 'control_gripper' tool
class control_gripper_definition(BaseModel):
    '''True to activate or open the gripper and False to deactivate or close the gripper.'''
    switch: bool = Field(description="True to activate or close the gripper and False to deactivate or open the gripper.")
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    model: str = Field(default="robotiq", description="the gripper model robot is using for applications")
    span : float = Field(default=None, description="The stroke lenght of the gripper to open if using robotiq gripper")

##########################################################
######     NEED TO ADD ROBOTIQ GRIPPER DRIVER AND WRAPPER
##########################################################  

# Implementation of the 'control_gripper' tool
class control_gripper_implementation(BaseTool):
    """Tool to activate/deactivate the gripper."""
    name = "control_gripper"
    description = "Activates/Deactivates the gripper."
    args_schema: Type[BaseModel] = control_gripper_definition

    def _run(self, switch: bool = True, robot_to_use: int = 1, model: str = "robotiq",span: float = None, flag: bool = True) -> None:
        print("control_gripper_implementation")
        
        ''' This method for robotiq gripper is a test method but 
            not the right way to control robotiq gripper. Till the robotiq package
            is integrated we will be using this
            Here True closes the gripper and False will open the gripper'''
        
        sim_status = rospy.get_param("/use_sim", default="false")
        if sim_status:
            response = ExternalServices().switch_sim_gripper(switch)

        else:
            if model == "robotiq":
                command_close = "rosservice call /robotiq/gripper/close"
                command_source = "source ~/workspace/robogpt/robogpt_ws/devel/setup.bash"
                command_open = "rosservice call /robotiq/gripper/open"
                os.system(command=command_source)

                if switch:
                    response = os.system(command=command_open)
                elif not switch:
                    response = os.system(command=command_close)
                else:
                    pass
                # response = robots[robot_to_use-1].set_robotiq(switch,span)

            else:
                response = robots[robot_to_use-1].set_gripper(3, switch)

        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)
        return response
    
    def _arun(self, switch: bool):
        print("activate_gripper does not support async")



# -----------------------------------------------------------------------------
#                      MISC
# -----------------------------------------------------------------------------

# Data model for the 'delay' tool
class delay_definition(BaseModel):
    delay: float = Field(description="Delay in seconds.")

# Implementation of the 'delay' tool
class delay_implementation(BaseTool):
    """Tool to add delay in the script."""
    name = "delay"
    description = "Adds delay in the script."
    args_schema: Type[BaseModel] = delay_definition

    def _run(self, delay: float, flag: bool = True):
        print("delay_implementation")
        time.sleep(delay)
        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)
        return ("added delay for " + str(delay) + " seconds")
    
    def _arun(self, delay: float):
        time.sleep(delay)
        return ("added delay for " + str(delay) + " seconds")
    
    
# Data model for the 'send_to_webapp' tool
class send_message_to_webapp_definition(BaseModel):
    message: str = Field(description="The message we need to send to the webapp")
 
# Implementation of the 'send_to_webapp' tool
class send_message_to_webapp_implementation(BaseTool):
    name = "send_to_webapp"
    description = "Send a particular message to webapp"
    args_schema: Type[BaseModel] = send_message_to_webapp_definition

    def _run(self, message: str, flag: bool = True):
        app_id = "1828565"
        key = "41c6b76d716c60427fa2"
        secret = "a74221a78df96bd79386"
        cluster = "ap2"

        pusher_client = pusher.Pusher(
            app_id=app_id, key=key, secret=secret, cluster=cluster)

        pusher_client.trigger('private-chat', 'evt::test', {'message': message})
        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)
# -----------------------------------------------------------------------------
#                   ROBOT AND MOVEIT CONTROL
# -----------------------------------------------------------------------------

# Data model for the 'move_translate' tool
class move_translate_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    x: float = Field(default=0.0, description="the distance in m to which robot needs to translate in x axis")
    y: float = Field(default=0.0, description="the distance in m to which robot needs to translate in y axis")
    z: float = Field(default=0.0, description="the distance in m to which robot needs to translate in z axis")
    toolspeed: int = Field(default=100, description="the speed of robot when it executes the translation")

# Implementation of the 'move_translate' tool
class move_translate_implementation(BaseTool):
    '''Tool to translate the robot in a particular plane or axis'''
    name = "move_translate"
    description = "Tool to translate or move the robot in a particular plane or axis"
    args_schema: Type[BaseModel] = move_translate_definition

    def _run(self, robot_to_use: int = 1, x: float = 0.0, y: float = 0.0, z: float = 0.0, toolspeed: int = 100, flag: bool = True):
        try:
            response = robots[robot_to_use-1].move_translate(x, y, z, toolspeed)
            utils.robot_logger.info("Move Translate SUCCEEDED")
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return response
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("Move Translate FAILED " + str(e))



# Data model for the 'move_to_joint' tool
class move_to_joint_definition(BaseModel):
    pose_name : str = Field(default= None, description="The name of the zone / pose or position to where robot needs to go")
    wait: bool = Field(default=True, description="True will make the move call synchronous and wait till move is completed.")
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    relative: bool = Field(default=False, description="Move relative to the current robot joint. It will move the joints by the amount specified in jointPose from the current joint position.")

# Implementation of the 'move_to_joint' tool
class move_to_joint_implementation(BaseTool):
    """Tool to move the robot to a specific joint pose"""
    name = "move_to_joint"
    description = "requests the robot server to move to a specific joint pose."
    args_schema: Type[BaseModel] = move_to_joint_definition

    def _run(self, pose_name: str=None, robot_to_use: int = 1, wait: bool = True, flag: bool = True) -> None:
        print("Moving robot to desired pose")
        if pose_name is not None:
                try:
                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose_name,joint_space=True)
                    if goal_pose is not None:
                        response = robots[robot_to_use - 1].move_to_joint(goal_pose)
                        utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")

                    elif goal_pose is None:
                        send_message_to_webapp_implementation()._run(f"Failed to move robot to '{pose_name}'. Pose is not available.")
                        response = False

                except Exception as e:
                    send_message_to_webapp_implementation()._run(f"Failed to retrieve pose '{pose_name}'. Error: {str(e)}")

# Data model for the 'move_to_pose' tool
class move_to_pose_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first robot model in the list, robot 2 will be the second robot model in the list and so on.")
    use_moveit: bool = Field(default=False, description="the boolean which checks if robot has to use moveit or not")
    pose_name : str = Field(default= None, description="The name of the zone / pose or position to where robot needs to go")


class move_to_pose_implementation(BaseTool):
    """Tool to move the robot to a specific pose"""
    
    name = "move_to_pose"
    description = "requests the robot server to move to a specific pose"
    
    args_schema: Type[BaseModel] = move_to_pose_definition
 
    def _run(self, goal_pose: list=None, pose_name: str=None, use_moveit: bool=False, robot_to_use:int=1, flag: str = True) -> None:
        print("Moving robot to desired pose")
        if pose_name is not None:
                try:
                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose_name,joint_space=False)
                    print(goal_pose)
                    if goal_pose is not None:
                        response = robots[robot_to_use - 1].move_to_pose(goal_pose)
                        utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")
                        flag = rospy.get_param("/tour_flag", default=False)
                        if flag:
                            utils.publish_pass_bool(val=True)

                    elif goal_pose is None:
                        print(f"Failed to move robot to '{pose_name}'. Pose is not available.")
                        response = False

                except Exception as e:
                    print(f"Failed to retrieve pose '{pose_name}'. Error: {str(e)}")
                    response = e

        elif goal_pose is not None:

                try:
                    response = robots[robot_to_use - 1].move_to_pose(goal_pose)
                    utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")

                except Exception as e:
                    send_message_to_webapp_implementation()._run(f"Failed to retrieve position'. Error: {str(e)}")
                    response = e

        if not flag:
            return response



# Data model for the 'move_in_trajactory' tool
class move_in_trajactory_definition(BaseModel):
    waypoints: List[list] = Field(description="list of the waypoints through which robot needs to make a trajactory")
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")

# Implementation of the 'move_in_trajactory' tool
class move_in_trajactory_implementation(BaseTool):
    "tool to run robot in a trajactory"
    name = "move_in_trajactroy"
    description = "the tool to run robot in a particular trajactory"
    args_schema: Type[BaseModel] = move_in_trajactory_definition

    def _run(self, waypoints: List[list], robot_to_use: int = 1, flag: bool = True):
        try:
            utils.robot_logger.info(f"Motion planning SUCCEEDED")
            response = robots[robot_to_use-1].move_trajectory(waypoints)
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return response
        except Exception as e:
            utils.robot_logger.error("Motion planning FAILED " + str(e))

# Data model for the 'save_pose' tool
class save_pose_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    zone_name: str = Field(description="Target pose name where the robot needs to move.")

# Implementation of the 'save_pose' tool
class save_pose_implementation(BaseTool):
    """Tool to add zones in the robot environment"""
    name = "saving_pose"
    description = "Saves the current position or robot for further use"
    args_schema: Type[BaseModel] = save_pose_definition

    def _run(self,zone_name: str,robot_to_use: int = 1, flag:bool = True) -> None:
        robot_model = ROBOT_LIST[robot_to_use - 1]
        print(f"This is the {robot_model} from list of {ROBOT_LIST}")
        robot_dict = json.loads(open(robot_home_file_path).read())
        if robot_model not in robot_dict:
            robot_dict[robot_model] = {}
        zone_pose = get_pose_implementation()._run(robot_to_use)
        print("zone pose:::::", zone_pose)
        robot_dict[robot_model][zone_name] = zone_pose

        try:
            with open(f"{robot_home_file_path}", 'w') as f:
                json.dump(robot_dict, f)
            flag = rospy.get_param("/tour_flag",default=False)
            if flag:
                utils.publish_pass_bool(val=True)
            return True
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("write to file failed - setting home pose " + str(e))
            return False

class save_joint_angles_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="the robot number to use. robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    zone_name: str = Field(description="Target pose name where the robot needs to move.")
    zone_pose: List[float] = Field(default=None, description="Target pose of the robot at the zone")

class save_joint_angles_implementation(BaseTool):
    """Tool to add zones in the robot environment"""
    name = "saving_angles"
    description = "Saves the current joint angles or joint states of the robot "
    args_schema: Type[BaseModel] = save_joint_angles_definition

    def _run(self,zone_name: str, zone_pose: List[float] = None,robot_to_use: int = 1) -> None:
        robot_model = ROBOT_LIST[robot_to_use - 1]
        robot_dict = json.loads(open(robot_joint_file_path).read())
        if robot_model not in robot_dict:
            robot_dict[robot_model] = {}
        if zone_pose == None:
            zone_pose = get_joint_implementation()._run(robot_to_use=1)
        robot_dict[robot_model][zone_name] = zone_pose
        print("zone pose:::::", zone_pose)

        try:
            with open(f"{robot_joint_file_path}", 'w') as f:
                json.dump(robot_dict, f)
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("write to file failed - setting home pose " + str(e)) 


class get_best_match_definition(BaseModel):
    object: str = Field(description="The name of the object with which the best match is to be detected")

# Implementation of the 'get_best_match' tool
class get_best_match_implementation(BaseTool):
    name = "get_best_match"
    description = "Function to get the best match of the object"
    args_schema: Type[BaseModel] = get_best_match_definition

    def _run(self, object: str, flag: bool = True):


        with open(object_details, 'r') as file:
            robogpt_obj_dict = json.load(file)

        obj_list = robogpt_obj_dict["objects_to_detect"]

        # Find the best match by comparing similarity ratios
        best_match = None
        best_ratio = 0
        for obj in obj_list:
            ratio = SequenceMatcher(None, object, obj).ratio()
            if ratio > best_ratio:
                best_ratio = ratio
                best_match = obj

        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)
        return best_match  

class get_object_list_implementation(BaseTool):
    name = "get_object_list"
    description = "Function to get the best match of the object"

    def _run(self, flag: bool = True):

        with open(object_details, 'r') as file:
            robogpt_obj_dict = json.load(file)

        obj_list = robogpt_obj_dict["objects_to_detect"]

        # Find the best match by comparing similarity ratio
        if flag:
            utils.publish_pass_bool(val=True)
        return obj_list

# Data model for the 'check_pose' tool
class check_pose_definition(BaseModel):
    object: str = Field(default=None, description="the object name whose position is to be determined")
    cam_name: str = Field(default="camera",description="the camera name from which we need the pose of the object")
    parent_frame: str = Field(default="base_link",description="the link with respect to which we need to get the pose of an object")
    gripper_frame: str = Field(default="None",description="the frame name of the gripper link from which robot needs to pick object")

# Implementation of the 'check_pose' tool
class check_pose_implementation(BaseTool):
    """Tool to tell the position of the object"""
    name = "check_pose"
    description = "To calculate the given object's position in cartesian plane"
    args_schema: Type[BaseModel] = check_pose_definition

    def _run(self, object: str = None,cam_name: str = "camera",parent_frame: str = "base_link",gripper_frame: str = None) -> list:
        
        try:
            if object is not None:
                object = get_best_match_implementation()._run(object=object)
                object_pose = ExternalServices().call_get_world_context(object_name=object,camera_name=cam_name,parent_frame=parent_frame)
                print(f"The position of {object} is {object_pose}")
                return object_pose
        
        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Could Not Find the pose of {object} due to {e}")

class industry_test_definition(BaseModel):
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")
    pose_name_list: list = Field(default=None,description="the list of the name of poses robot needs to go")
    cycles: int = Field(default=1,description="The number times robot needs to go to these points or number cycles robot needs to perfrom this task")

class industry_test_implementation(BaseTool):
    name = "industry_test"
    description="The tool to test specific waypoints in multiple cycles"
    args_schema: Type[BaseModel] = industry_test_definition

    def _run(self,robot_to_use: int = 1,pose_name_list: list = None, cycles: int = 1, flag: bool = True):
        try:
            for i in range(cycles):
                for pose in pose_name_list:

                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose,robot_to_use=robot_to_use)
                    move_to_pose_implementation()._run(goal_pose=goal_pose)

            home = get_zone_pose_implementation()._run(robot_to_use=1,zone_name="home")
            move_to_pose_implementation()._run(goal_pose=home)
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)

        except Exception as e:
            send_message_to_webapp_implementation()._run(message=f"Unable to run this test due to {e}")


class recording_object_definition(BaseModel):
    robot_to_use: int = Field(default=1,description="Robot number we need to use for this task")
    pose_name_list: list = Field(default=None,description="the list of the name of poses robot needs to go")

class recording_object_implementation(BaseTool):
    name = "recording_object"
    description="The record the object using specific waypoints"
    args_schema: Type[BaseModel] = recording_object_definition

    def _run(self, robot_to_use: int = 1, flag: bool = True):
        try:
            # Save poses in a dictionary for better management and reusability
            poses = {}
            required_poses = ["pose1", "pose2", "pose3", "pose4", "pose5", "pose6"]
            
            for pose_name in required_poses:
                try:
                    pose = get_zone_pose_implementation()._run(robot_to_use=robot_to_use, zone_name=pose_name)
                    if pose is None:
                        send_message_to_webapp_implementation()._run(
                            f"Pose '{pose_name}' not found. Please save the required poses before running this test.")
                        return
                    poses[pose_name] = pose
                except Exception as e:
                    send_message_to_webapp_implementation()._run(
                        f"Error retrieving pose '{pose_name}'. Error: {str(e)}")
                    return

            # Execute motion to each pose
            for pose_name, pose in poses.items():
                try:
                    move_to_pose_implementation()._run(goal_pose=pose)
                except Exception as e:
                    send_message_to_webapp_implementation()._run(
                        f"Failed to move robot to {pose_name}. Error: {str(e)}")
                    return

            # Move back to the home position
            try:
                home = get_zone_pose_implementation()._run(robot_to_use=robot_to_use, zone_name="home")
                if home is None:
                    send_message_to_webapp_implementation()._run("Failed to retrieve 'home' pose. Pose not found.")
                    return
                move_to_pose_implementation()._run(goal_pose=home)
            except Exception as e:
                send_message_to_webapp_implementation()._run(
                    f"Failed to move robot to 'home'. Error: {str(e)}")
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)

        except Exception as e:
            send_message_to_webapp_implementation()._run(
                f"Unable to complete the cycle due to an unexpected error: {str(e)}")

class custom_tour_defination(BaseModel):
    tour_name: str = Field(description="The name of the tour which robogpt needs to run or show")

class custom_tour_implementation(BaseTool):
    name = "custom_tour"
    description="the tool to run the custom tours for partiuclar use-case or skill"
    args_schema:Type[BaseModel] = custom_tour_defination

    def _run(self,tour_name: str, flag: bool = True):

        def get_best_tour_match(input_tour: str) -> str:
            """
            Returns the best matched tour name from the tour list defined in the robogpt.json file.

            Args:
                input_tour (str): The tour name for which a best match is to be found.
                config_file (str): Path to the JSON configuration file containing the ideal tour names.
                                Defaults to "robogpt_v3/robogpt_agents/config/robot_config/robogpt.json".

            Returns:
                str: The best matched tour name. Returns None if no tours are found or in case of an error.
            """
            try:
                with open(robotgpt_config, "r") as f:
                    config = json.load(f)
            except Exception as e:
                print(f"Error reading config file: {e}")
                return None

            tour_list = config.get("tour_list", [])
            if not tour_list:
                print("No tour names found in the configuration file.")
                return None

            best_match = None
            best_ratio = 0
            for candidate in tour_list:
                # Compare lower-case versions for case-insensitive matching
                ratio = SequenceMatcher(None, input_tour.lower(), candidate.lower()).ratio()
                if ratio > best_ratio:
                    best_ratio = ratio
                    best_match = candidate

            return best_match
        def get_tour_path(tour: str):
            tour_script = os.path.join(tour_paths, f"{tour}.txt")
            return tour_script

        tour = get_best_tour_match(tour_name)
        tour_script = get_tour_path(tour)
        print(f"Running the tour of {tour}")

        ExternalServices().run_tour(tour_name=tour_script)
        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)

        return f"Running tour of {tour}"
class confirmation_defination(BaseModel):
    confirmation: bool = Field(description="The boolena to check is there is confirmation or not")

class confirmation_implementation(BaseTool):
    name = "confirmation"
    description="run this if user says okay, done or anyhting releated to confirmation"
    args_schema:Type[BaseModel] = confirmation_defination
    def _run(self, confirmation: bool = True):
        utils.publish_pass_bool(val=True)
        return f"Just reply with great or nothing"