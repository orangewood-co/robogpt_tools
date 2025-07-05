from robogpt_tools.applications.base.skills import *

# Place Object Definition and Implementation
class place_object_definition(BaseModel):
    drop_name: Optional[str] = Field(default=None, description="The name of the pose on which robot needs to place the object")
    robot_to_use: int = Field(default=1, description="Robot number we need to use for this task")
    direction: Optional[str] = Field(default=None, description="The direction in which robot needs to place the object")
    drop_object: Optional[str] = Field(default=None, description="The object on which we need to place the object")
    model_config = ConfigDict(arbitrary_types_allowed=True)

class place_object_implementation(BaseTool):
    """Tool to place any object with robot"""
    name: ClassVar[str] = "place_object"
    description: ClassVar[str] = "The tool to place any object with robot"
    args_schema: ClassVar[Type[BaseModel]] = place_object_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, drop_name: Optional[str] = None, drop_object: Optional[str] = None, robot_to_use: int = 1, direction: Optional[str] = None):
        home = get_zone_pose_implementation()._run(robot_to_use=1, zone_name="home")

        if drop_object is not None:
            try: 
                target = check_pose_implementation()._run(object=drop_object)
                if target is None:
                    send_message_to_webapp_implementation()._run(f"Could Not find the {drop_object}. Placing Object in trash")
                    trash = get_zone_pose_implementation()._run(zone_name="trash", robot_to_use=robot_to_use)
                    move_to_pose_implementation()._run(goal_pose=trash)
                    control_IO_implementation()._run(switch=True)
                    move_to_pose_implementation()._run(goal_pose=home)

                target = target.copy()
                target[2] = target[2] + 0.10
                move_to_pose_implementation()._run(goal_pose=target)
                control_IO_implementation()._run(switch=True)
                return True
            except Exception as e:
                send_message_to_webapp_implementation()._run(f"Error in placing due to {e}")

        if drop_name is not None:
            try: 
                pose = get_zone_pose_implementation()._run(zone_name=drop_name, robot_to_use=robot_to_use)
                move_to_pose_implementation()._run(goal_pose=pose)
                control_IO_implementation()._run(switch=True)
                move_to_pose_implementation()._run(goal_pose=home)

                return True
            except Exception as e:
                send_message_to_webapp_implementation()._run(f"Error in placing due to {e}")

        if direction is not None:   
            send_message_to_webapp_implementation()._run(message=f"Currently I don't have a sense of direction. Could you please specify an object or dropping pose name.")
            move_to_pose_implementation()._run(goal_pose=home)

            return False


# Pick Object Definition and Implementation
class pick_definition(BaseModel):
    object: str = Field(description="Object which the robot needs to pick up")
    robot_to_use: int = Field(default=1, description="Robot number we need to use for this task")
    model_config = ConfigDict(arbitrary_types_allowed=True)

class pick_implementation(BaseTool):
    """Tool to pick any object with robot"""
    name: ClassVar[str] = "pick_implementation"
    description: ClassVar[str] = "The tool to pick any object with robot"
    args_schema: ClassVar[Type[BaseModel]] = pick_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, object: str, robot_to_use: int = 1):
        try:
            home = get_zone_pose_implementation()._run(robot_to_use=1, zone_name="home")
            int_pose = get_zone_pose_implementation()._run(robot_to_use=1, zone_name="int_pose")

            move_to_pose_implementation()._run(goal_pose=home)
            pose = check_pose_implementation()._run(object=object)
            control_IO_implementation()._run(switch=True)

            if pose is None:
                utils.send_msg(message="Unable to detect the object. Please try again!")
                move_to_pose_implementation()._run(goal_pose=home, robot_to_use=1)

            move_to_pose_implementation()._run(goal_pose=pose)
            control_IO_implementation()._run(switch=False)

            if int_pose is not None:
                move_to_pose_implementation()._run(goal_pose=int_pose)
            else:
                send_message_to_webapp_implementation()._run(f"Intermediate pose not defined taking it to home position.")
                move_to_pose_implementation()._run(goal_pose=home)

        except Exception as e:
            send_message_to_webapp_implementation()._run(f"Unable to pick {object} due to {e}")


# Clean Table Definition and Implementation
class clean_table_definition(BaseModel):
    robot_to_use: int = Field(default=1, description="Robot number we need to use for this task")
    drop_pose: str = Field(default="bin", description="The name of the pose on which robot needs to place the object")
    model_config = ConfigDict(arbitrary_types_allowed=True)

class clean_table_implementation(BaseTool):
    """Tool to clean or pick up all the stuff and place it in the bin or bucket"""
    name: ClassVar[str] = "clean_table"
    description: ClassVar[str] = "The tool to clean or pick up all the stuff and place it in the bin or bucket"
    args_schema: ClassVar[Type[BaseModel]] = clean_table_definition
    return_direct: ClassVar[bool] = False
    verbose: ClassVar[bool] = False

    def _run(self, robot_to_use: int = 1, drop_pose: str = "bin"):
        objects = get_object_list_implementation()._run()
        print(objects)
        for object in objects:
            print(object)
            if object != "bucket":
                pick_implementation()._run(object=object)
                place_object_implementation()._run(drop_name=drop_pose, robot_to_use=robot_to_use)

        home = get_zone_pose_implementation()._run(robot_to_use=1, zone_name="home")
        move_to_pose_implementation()._run(goal_pose=home)