import time
from typing import ClassVar
from robogpt_tools.applications.utilities.skill_initializers import *
from robogpt_tools.applications.utilities.robot_loader import RobotLoader

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

# Implementation of the 'delay' tool

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

    
