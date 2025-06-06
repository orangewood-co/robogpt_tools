import time
from typing import ClassVar
from robogpt_tools.applications.utilities.skill_initializers import *
from robogpt_tools.applications.utilities.robot_loader import RobotLoader
from robogpt_tools.applications.utilities import utils

robot_list,robots = RobotLoader().load_robots()

# Data model for the 'delay' tool
class delay_definition(BaseModel):
    delay: float = Field(description="Delay in seconds.")
    
    # Add this for Pydantic v2 compatibility
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'delay' tool
class delay_implementation(BaseTool):
    """Tool to add delay in the script."""
    # Add proper type annotations to all class attributes
    name: ClassVar[str] = "delay"
    description: ClassVar[str] = "Adds delay in the script."
    args_schema: ClassVar[Type[BaseModel]] = delay_definition
    return_direct: ClassVar[bool] = False  # Add this if BaseTool has it
    verbose: ClassVar[bool] = False  # Add this if BaseTool has it
    
    # If there are any other attributes from BaseTool, add them with ClassVar annotations

    def _run(self, delay: float, flag: bool = True) -> str:
        print("delay_implementation")
        time.sleep(delay)
        return f"added delay for {delay} seconds"
    
    def _arun(self, delay: float) -> str:
        time.sleep(delay)
        return f"added delay for {delay} seconds"
    
    
# Data model for the 'delay' tool
class move_robot_definition(BaseModel):
    pose_name: str = Field(description="Name of the pose to move to")
    # Add this for Pydantic v2 compatibility
    model_config = ConfigDict(arbitrary_types_allowed=True)


# Implementation of the 'move_robot' tool
class move_robot_implementation(BaseTool):
    """Tool to add delay in the script."""
    # Add proper type annotations to all class attributes
    name: ClassVar[str] = "move_robot"
    description: ClassVar[str] = "This is toll to move the robot to a particular position or pose which is saved. Run this function if user wants to move the robot to a particular position or pose"
    args_schema: ClassVar[Type[BaseModel]] = move_robot_definition
    return_direct: ClassVar[bool] = False  # Add this if BaseTool has it
    verbose: ClassVar[bool] = False  # Add this if BaseTool has it
    
    # If there are any other attributes from BaseTool, add them with ClassVar annotations
    def _run(self,pose_name: str) -> str:
        print("Running the move command")
        print(f"{Colors.GREEN}ROBOT IS MOVING to {pose_name}:{Colors.RESET}")

    

# Data model for the 'get_joint' tool
class get_joint_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    wait: bool = Field(default=True, description="True will allow to return the latest data received from the robot.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'get_joint' tool
class get_joint_implementation(BaseTool):
    """Tool to get joint values of the robot."""
    name: ClassVar[str] = "get_joint"
    description: ClassVar[str] = "Returns the current joint values of the robot"
    args_schema: ClassVar[Type[BaseModel]] = get_joint_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, wait: bool = True) -> list:
        try:
            print("get_joint implementation")
            print(f"robot_to_use: {robot_to_use}")
            # robots[robot_to_use - 1].test()
            curr_joint_rads = robots[robot_to_use-1].get_joints()

        
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_joint failed " + str(e))
            return None

# Data model for the 'get_pose' tool
class get_pose_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="The robot number to use")
    wait: bool = Field(default=True, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'get_pose' tool
class get_pose_implementation(BaseTool):
    """Tool to get current pose of robot end effector."""
    name: ClassVar[str] = "get_position"
    description: ClassVar[str] = "Returns the current pose of robot end effector"
    args_schema: ClassVar[Type[BaseModel]] = get_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, wait: bool = True) -> list:
        print("get_pose")
        try:
            curr_pose = robots[robot_to_use-1].get_tcp()
            print(f"current pose:: {curr_pose}")

            return curr_pose
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_pose failed " + str(e))
            return None

# Data model for the 'get_zone_pose' tool
class get_zone_pose_definition(BaseModel):
    zone_name: str = Field(default=None, description="Target pose name where the robot needs to move.")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    joint_space: bool = Field(default=False, description="Boolean to decide whether to use joint space or cartesian space")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'get_zone_pose' tool
class get_zone_pose_implementation(BaseTool):
    """Tool to get pose of the robot at a particular zone."""
    name: ClassVar[str] = "get_zone_pose"
    description: ClassVar[str] = "Requests the pose of the robot at a particular zone"
    args_schema: ClassVar[Type[BaseModel]] = get_zone_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, zone_name: str, robot_to_use: int = 1, joint_space: bool = False) -> list:
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
            robot_name = robot_list[robot_to_use - 1]
            robot_pose = robot_dict[robot_name]
            if zone_name in robot_pose:
                zone_match = get_best_zone_match(zone_name, robot_pose)
                pose = robot_pose[zone_match]
            else:
                pose = None
                
            return pose
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("get_zone_pose failed" + str(e))
            return None

# Data model for the 'hand_teach' tool
class hand_teach_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    config: bool = Field(default=False, description="The boolean which decides the robot is in gravity or hand teach mode or not. True puts the robot in hand teach and false switches the gravity/hand teach off")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'hand_teach' tool
class hand_teach_implementation(BaseTool):
    """Tool to switch hand teach/gravity mode."""
    name: ClassVar[str] = "hand_teach_init"
    description: ClassVar[str] = "It switches the hand teach/gravity mode on or off based on the config bool value"
    args_schema: ClassVar[Type[BaseModel]] = hand_teach_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, config: bool = False):
        try:
            response = robots[robot_to_use-1].set_hand_teach(config)
            return response

        except Exception as e:
            print("Robot is unable to switch in Hand teach")

# Data model for the 'control_gripper' tool
class control_gripper_definition(BaseModel):
    switch: bool = Field(description="True to activate or close the gripper and False to deactivate or open the gripper.")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    model: str = Field(default="pneumatic", description="The gripper model robot is using for applications")
    span: Optional[float] = Field(default=None, description="The stroke length of the gripper to open if using robotiq gripper")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'control_gripper' tool
class control_gripper_implementation(BaseTool):
    """Tool to control robot gripper."""
    name: ClassVar[str] = "control_gripper"
    description: ClassVar[str] = "Activates/Deactivates the gripper."
    args_schema: ClassVar[Type[BaseModel]] = control_gripper_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, switch: bool = True, robot_to_use: int = 1, model: str = "pneumatic", span: float = None) -> None:
        print("control_gripper")
        
        ''' This method for robotiq gripper is a test method but 
            not the right way to control robotiq gripper. Till the robotiq package
            is integrated we will be using this
            Here True closes the gripper and False will open the gripper
        '''
        
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
                response = robots[robot_to_use-1].set_gripper(4, switch)

        return response

# Data model for the 'delay' tool
class delay_definition(BaseModel):
    delay: float = Field(default=0.0, description="Delay in seconds.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'delay' tool
class delay_implementation(BaseTool):
    """Tool to add delay in the script."""
    name: ClassVar[str] = "delay"
    description: ClassVar[str] = "Adds delay in the script."
    args_schema: ClassVar[Type[BaseModel]] = delay_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, delay: float):
        print("delay")
        time.sleep(delay)

        return ("added delay for " + str(delay) + " seconds")

# Data model for the 'send_message_to_webapp' tool
class send_message_to_webapp_definition(BaseModel):
    message: str = Field(description="The message we need to send to the webapp")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'send_message_to_webapp' tool
class send_message_to_webapp_implementation(BaseTool):
    """Tool to send messages to webapp."""
    name: ClassVar[str] = "send_to_webapp"
    description: ClassVar[str] = "Send a particular message to webapp"
    args_schema: ClassVar[Type[BaseModel]] = send_message_to_webapp_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, message: str):
        pusher_client = pusher.Pusher(
            app_id=os.environ.get('PUSHER_APP_ID'), 
            key=os.environ.get('NEXT_PUBLIC_PUSHER_KEY'), 
            secret=os.environ.get('PUSHER_SECRET'), 
            cluster=os.environ.get('NEXT_PUBLIC_PUSHER_CLUSTER')
        )
        user = rospy.get_param("client_id", default="user")
        pusher_client.trigger('private-chat', 'evt::test', {'message': message,'userId': user})

# Data model for the 'move_translate' tool
class move_translate_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    x: float = Field(default=0.0, description="The distance in m to which robot needs to translate in x axis")
    y: float = Field(default=0.0, description="The distance in m to which robot needs to translate in y axis")
    z: float = Field(default=0.0, description="The distance in m to which robot needs to translate in z axis")
    toolspeed: int = Field(default=100, description="The speed of robot when it executes the translation")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'move_translate' tool
class move_translate_implementation(BaseTool):
    """Tool to translate robot in a particular plane or axis."""
    name: ClassVar[str] = "move_translate"
    description: ClassVar[str] = "Tool to translate or move the robot in a particular plane or axis"
    args_schema: ClassVar[Type[BaseModel]] = move_translate_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, x: float = 0.0, y: float = 0.0, z: float = 0.0, toolspeed: int = 100):
        try:
            response = robots[robot_to_use-1].move_translate(x, y, z, toolspeed)
            utils.robot_logger.info("Move Translate SUCCEEDED")

            return response
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("Move Translate FAILED " + str(e))

# Data model for the 'move_to_joint' tool
class move_to_joint_definition(BaseModel):
    goal_pose: Optional[List[float]] = Field(default=None, description="The joint angles to which the robot should move.")
    pose_name: Optional[str] = Field(default=None, description="The name of the zone / pose or position to where robot needs to go")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    wait: bool = Field(default=True, description="True will make the move call synchronous and wait till move is completed.")
    relative: bool = Field(default=False, description="Move relative to the current robot joint. It will move the joints by the amount specified in jointPose from the current joint position.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'move_to_joint' tool
class move_to_joint_implementation(BaseTool):
    """Tool to move robot to a specific joint pose."""
    name: ClassVar[str] = "move_to_joint"
    description: ClassVar[str] = "Requests the robot server to move to a specific joint pose."
    args_schema: ClassVar[Type[BaseModel]] = move_to_joint_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, goal_pose: list=None, pose_name: str=None, robot_to_use: int = 1, wait: bool = True) -> None:
        print("Moving robot to desired pose")
        if pose_name is not None:
                try:
                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose_name, joint_space=True)
                    if goal_pose is not None:
                        response = robots[robot_to_use - 1].move_to_joint(goal_pose)
                        utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")

                    elif goal_pose is None:
                        send_message_to_webapp_implementation()._run(f"Failed to move robot to '{pose_name}'. Pose is not available.")
                        response = False

                except Exception as e:
                    send_message_to_webapp_implementation()._run(f"Failed to retrieve pose '{pose_name}'. Error: {str(e)}")

        elif goal_pose is not None:
                try:
                    response = robots[robot_to_use - 1].move_to_joint(goal_pose)
                    utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")

                except Exception as e:
                    send_message_to_webapp_implementation()._run(f"Failed to retrieve position'. Error: {str(e)}")

        return response

# Data model for the 'move_to_pose' tool
class move_to_pose_definition(BaseModel):
    goal_pose: Optional[List[float]] = Field(default=None, description="The pose to which the robot should move.")
    pose_name: Optional[str] = Field(default=None, description="The name of the zone / pose or position to where robot needs to go")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    use_moveit: bool = Field(default=False, description="True will make the move call synchronous and wait till move is completed.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'move_to_pose' tool
class move_to_pose_implementation(BaseTool):
    """Tool to move robot to a specific pose."""
    name: ClassVar[str] = "move_to_pose"
    description: ClassVar[str] = "Request the robot server to move to a specific pose"
    args_schema: ClassVar[Type[BaseModel]] = move_to_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, goal_pose: list=None, pose_name: str=None, use_moveit: bool=False, robot_to_use:int=1) -> None:
        print("Moving robot to desired pose")
        if pose_name is not None:
                try:
                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose_name, joint_space=False)
                    print(goal_pose)
                    if goal_pose is not None:
                        response = robots[robot_to_use - 1].move_to_pose(goal_pose)
                        utils.robot_logger.info(f"Motion planning SUCCEEDED : goal pose - {goal_pose}")

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

        return response

# Data model for the 'move_in_trajectory' tool
class move_in_trajectory_definition(BaseModel):
    waypoints: List[List[float]] = Field(description="List of the waypoints through which robot needs to make a trajectory")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'move_in_trajectory' tool
class move_in_trajectory_implementation(BaseTool):
    """Tool to run robot in a particular trajectory."""
    name: ClassVar[str] = "move_in_trajectory"
    description: ClassVar[str] = "The tool to run robot in a particular trajectory"
    args_schema: ClassVar[Type[BaseModel]] = move_in_trajectory_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, waypoints: List[list], robot_to_use: int = 1):
        try:
            utils.robot_logger.info(f"Motion planning SUCCEEDED")
            response = robots[robot_to_use-1].move_trajectory(waypoints)
            return response
        except Exception as e:
            utils.robot_logger.error("Motion planning FAILED " + str(e))

# Data model for the 'save_pose' tool
class save_pose_definition(BaseModel):
    zone_name: str = Field(default=None, description="Target pose name where the robot needs to move.")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'save_pose' tool
class save_pose_implementation(BaseTool):
    """Tool to save current position of the robot."""
    name: ClassVar[str] = "save_pose"
    description: ClassVar[str] = "Saves the current position of the robot in a json file"
    args_schema: ClassVar[Type[BaseModel]] = save_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, zone_name: str, robot_to_use: int = 1) -> None:
        robot_model = robot_list[robot_to_use - 1]
        print(f"This is the {robot_model} from list of {robot_list}")
        robot_dict = json.loads(open(robot_home_file_path).read())
        if robot_model not in robot_dict:
            robot_dict[robot_model] = {}
        zone_pose = get_pose_implementation()._run(robot_to_use)
        print("zone pose:::::", zone_pose)
        robot_dict[robot_model][zone_name] = zone_pose

        try:
            with open(f"{robot_home_file_path}", 'w') as f:
                json.dump(robot_dict, f)

            return True
        except Exception as e:
            self.return_direct = True
            utils.robot_logger.error("write to file failed - setting home pose " + str(e))
            return False

# Data model for the 'save_joint_angles' tool
class save_joint_angles_definition(BaseModel):
    zone_name: str = Field(default=None, description="Target pose name where the robot needs to move.")
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    zone_pose: Optional[List[float]] = Field(default=None, description="The joint angles to which the robot should move.")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'save_joint_angles' tool
class save_joint_angles_implementation(BaseTool):
    """Tool to save joint angles of the robot."""
    name: ClassVar[str] = "saving_angles"
    description: ClassVar[str] = "Saves the current position of the robot in a json file"
    args_schema: ClassVar[Type[BaseModel]] = save_joint_angles_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, zone_name: str, zone_pose: List[float] = None, robot_to_use: int = 1) -> None:
        robot_model = robot_list[robot_to_use - 1]
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

# Data model for the 'get_best_match' tool
class get_best_match_definition(BaseModel):
    object: str = Field(description="The name of the object with which the best match is to be detected")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'get_best_match' tool
class get_best_match_implementation(BaseTool):
    """Tool to get best match of an object."""
    name: ClassVar[str] = "get_best_match"
    description: ClassVar[str] = "Function to get the best match of the object"
    args_schema: ClassVar[Type[BaseModel]] = get_best_match_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, object: str):
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

# Data model for the 'get_object_list' tool
class get_object_list_definition(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'get_object_list' tool
class get_object_list_implementation(BaseTool):
    """Tool to get list of objects."""
    name: ClassVar[str] = "get_object_list"
    description: ClassVar[str] = "Function to get the best match of the object"
    args_schema: ClassVar[Type[BaseModel]] = get_object_list_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self):
        with open(object_details, 'r') as file:
            robogpt_obj_dict = json.load(file)

        obj_list = robogpt_obj_dict["objects_to_detect"]

        flag = rospy.get_param("/tour_flag", default=False)
        if flag:
            utils.publish_pass_bool(val=True)
        return obj_list

# Data model for the 'check_pose' tool
class check_pose_definition(BaseModel):
    object: Optional[str] = Field(default=None, description="The object name whose position is to be determined")
    cam_name: str = Field(default="camera", description="The camera name from which we need the pose of the object")
    parent_frame: str = Field(default="base_link", description="The link with respect to which we need to get the pose of an object")
    gripper_frame: Optional[str] = Field(default=None, description="The frame name of the gripper link from which robot needs to pick object")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'check_pose' tool
class check_pose_implementation(BaseTool):
    """Tool to calculate object position in cartesian plane."""
    name: ClassVar[str] = "check_pose"
    description: ClassVar[str] = "To calculate the given object's position in cartesian plane"
    args_schema: ClassVar[Type[BaseModel]] = check_pose_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, object: str = None, cam_name: str = "camera", parent_frame: str = "base_link", gripper_frame: str = None) -> list:
        try:
            if object is not None:
                object = get_best_match_implementation()._run(object=object)
                object_pose = ExternalServices().call_get_world_context(object_name=object, camera_name=cam_name, parent_frame=parent_frame, aruco_detect=False)
                print(f"The position of {object} is {object_pose}")
                return object_pose
        
        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Could Not Find the pose of {object} due to {e}")

# Data model for the 'industry_test' tool
class industry_test_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="The robot number to use. Robot 1 will be the first IP address in the list, robot 2 will be the second IP address in the list and so on.")
    pose_name_list: Optional[List[str]] = Field(default=None, description="The list of the name of poses robot needs to go")
    cycles: int = Field(default=1, description="The number times robot needs to go to these points or number cycles robot needs to perform this task")
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'industry_test' tool
class industry_test_implementation(BaseTool):
    """Tool to test specific waypoints in multiple cycles."""
    name: ClassVar[str] = "industry_test"
    description: ClassVar[str] = "The tool to test specific waypoints in multiple cycles"
    args_schema: ClassVar[Type[BaseModel]] = industry_test_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, pose_name_list: list = None, cycles: int = 1):
        try:
            for i in range(cycles):
                for pose in pose_name_list:
                    goal_pose = get_zone_pose_implementation()._run(zone_name=pose, robot_to_use=robot_to_use)
                    move_to_pose_implementation()._run(goal_pose=goal_pose)

            home = get_zone_pose_implementation()._run(robot_to_use=1, zone_name="home")
            move_to_pose_implementation()._run(goal_pose=home)
            flag = rospy.get_param("/tour_flag", default=False)
            if flag:
                utils.publish_pass_bool(val=True)

        except Exception as e:
            send_message_to_webapp_implementation()._run(message=f"Unable to run this test due to {e}")

