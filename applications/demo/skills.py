import time
from typing import ClassVar
from pydantic import BaseModel, Field
from langchain_core.tools import BaseTool
from robogpt_tools.applications.utilities.skill_initializers import *
from robogpt_tools.applications.utilities.robot_loader import RobotLoader
from robogpt_tools.applications.utilities import utils
robot_list,robots = RobotLoader().load_robots()


