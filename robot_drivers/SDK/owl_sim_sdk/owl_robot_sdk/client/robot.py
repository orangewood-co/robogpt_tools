"""
OwlSimClient to control a MoveIt based Robot
"""
__version__ = "0.12"
__author__ = 'Lentin Joesph'

import sys
import copy

import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
import geometry_msgs.msg
import tf
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



class OwlSimClient(object):
    """
    Class acting as a client interface to control a MoveIt! based robot through ROS. It handles initializing,
    controlling, and interacting with the robot within a simulated environment. This includes managing movement,
    adding obstacles, and handling the robot's gripper.
    """

    def __init__(self, arm_group_name="arm",gripper_group_name="gripper",wait_for_servers=10,gripper_enable=True):
        """
        Initializes the robot environment, including the MoveIt! commander, ROS node, and publishers.
        Sets up arm and gripper groups for robot manipulation.

        Args:
            arm_group_name (str): Name of the arm move group.
            gripper_group_name (str): Name of the gripper move group.
            wait_for_servers (int): Time to wait for move group servers to be available.
            gripper_enable (bool): Flag to decide gripper to enable or not.

        """

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('owl_sim_client',
                        anonymous=True)

        self.gripper_enable = gripper_enable
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:

        self.group_name = arm_group_name
        self.gripper_group_name = gripper_group_name

        self.robot = None
        self.scene = None
        self.group = None
        self.gripper_group = None

        try:
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()

            self.group = moveit_commander.MoveGroupCommander(self.group_name,wait_for_servers=wait_for_servers)
            if(self.gripper_enable):
                self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name,wait_for_servers=wait_for_servers)
        except:
            print("Exception, Initialization of movegroup failed, Check MoveIt is started and planning group is available")
            sys.exit(0)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   DisplayTrajectory,
                                                   queue_size=20)

    def __repr__(self):
        return "Robot Object (Move Group name=%s)" % (self.group_name)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __del__(self):
        self.close()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
    
    def is_running(self) -> bool:
        """
        Checks if the robot's move group is currently available.

        Returns:
            bool: True if the move group is available, False otherwise.
        """
        result = False
        result = self.robot.has_group(self.group_name)	
        return result
  
    def get_version(self) -> str:
        """
        Return the current robot version.
        
        """
        version = self.group.get_name()	
        return version

    def get_joint(self):
        """
        Return the current joint values of robot.
        """
        current_joints = self.group.get_current_joint_values()
        return current_joints
    
    def get_tcp(self):
        """
        Return the current tcp values of robot.
        """
        current_pose = pose_to_list(self.group.get_current_pose().pose)

        return current_pose

    def get_tcp_orientation(self,mode="quat"):
        """
        Return the current tcp orientation in euler or queternion values
        mode: euler
        mode: quat
        """
        current_pose = pose_to_list(self.group.get_current_pose().pose)

        if(mode == "euler"):
            euler = tf.transformations.euler_from_quaternion((current_pose[3],current_pose[4],current_pose[5],current_pose[6]))
            return euler
        elif(mode == "quat"):
            quat = [0,0,0,0]
            quat[0] = current_pose[3]
            quat[1] = current_pose[4]
            quat[2] = current_pose[5]
            quat[3] = current_pose[6]
            return quat
        else:
            null = []
            return null


    def get_tcp_position(self):
        """
        Return the current tcp position x,y,z
        """
        current_pose = pose_to_list(self.group.get_current_pose().pose)

        position = [0,0,0]

        position[0] = current_pose[0]
        position[1] = current_pose[1]
        position[2] = current_pose[2]

        return position


    def move_to_pose( self, goalPose, toolSpeed=1, wait=True, relative=False):
        """
        Request robot server to move to goal pose with desired tool speed in cartesian space.

        Parameters:
            goalPose (Pose)   : Goal pose [x,y,z,rx,ry,rz] robot need to achieve with desired tool speed.\n
            toolSpeed(float) : Tool speed with which robot need to do move.\n
            wait     (bool)      : True will make the move call synchronouse and wait till move is completed.\n
            relative (bool)    : Move relative to current robot pose.\n
            moveType (int)     : Type of move plan need to generate for move.\n
        """
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = goalPose[0]
        pose_goal.position.y = goalPose[1]
        pose_goal.position.z = goalPose[2]

        if(len(goalPose) == 6):
        #Roll, Pitch and Yaw
            quaternion = tf.transformations.quaternion_from_euler(goalPose[3], goalPose[4], goalPose[5])
            pose_goal.orientation = quaternion

        elif(len(goalPose) == 7):
            self.group.set_pose_target(goalPose)


        self.plan = self.group.go(wait=wait)

        return True



    def move_to_joint(self, jointPose, toolSpeed=1, wait=True, relative=False):
        """
        Request robot server to move to goal joint with desired tool speed in joint space.

        Parameters:
            joalPose (Joint)   : Goal joint robot need to achieve with desired tool speed in radians.
            toolSpeed (float) : Tool speed with which robot need to do move.
            wait (bool)       : True will make the move call synchronouse and wait till move is completed.
            relative(bool)    : Move relative to current robot joint.
        """
        joint_goal = self.group.get_current_joint_values()
        no_of_joint = len(joint_goal)

        if(no_of_joint > len(jointPose)):
            print("Number of joint values great than actual joints")
            return False
        elif(no_of_joint < len(jointPose)):
            print("Number of joint values less than actual joints")
            return False
        elif(no_of_joint == len(jointPose)):
            joint_goal = list(jointPose)


            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.group.go(joint_goal, wait=wait)

            # Calling ``stop()`` ensures that there is no residual movement
            self.group.stop()

            return True

    def display_path(self):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)        


    def move_trajectory(self, trajectory=[]):
        """
        Request robot server to follow a joint trajectory.

        Parameters:
            trajectory (Trajectory)     : Desired trajectory need to follow by robot.\n
        """
        for traj in trajectory:
            self.move_translate(traj[0],traj[1],traj[2])

        return False


    def move_translate(self, x=0.0, y=0.0, z=0.0, eef_step=0.01, jump_threshold=0, scale=1, wait_exec=True):
        """
        Request robot server to translate in cartesian space.

        Parameters:
            x(float) : Translate in x direction.\n
            y(float) : Translate in y direction.\n
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """

        waypoints = []

        wpose = self.group.get_current_pose().pose

        wpose.position.x += scale * x  # Second move forward/backwards in (x)
        wpose.position.y += scale * y  # and sideways (y)
        wpose.position.z += scale * z  # First move up (z)

        waypoints.append(copy.deepcopy(wpose))


        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        eef_step,        # eef_step
                                        jump_threshold)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:

        if(fraction == 0):
            return False
        else:
            self.group.execute(plan, wait=wait_exec)
            self.plan = plan
            return True




    def move_up(self, z=0.05, toolSpeed=100):
        """
        Request robot server to move in up.

        Parameters:
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """
        self.move_translate(0,0,z)
        return True
    
    def move_down(self, z=-0.05, toolSpeed=100):
        """
        Request robot server to move in down.
        
        Parameters:
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """
        self.move_translate(0,0,z)
        return True

#TODO
    '''
    def move_pause(self):
        """
        Request MoveIt server to pause the current move.
        """
        pass

    def move_resume(self):
        """
        Request robot server to resume the paused move.
        """
        pass
    '''

    def move_abort(self):
        """
        Request MoveIt to abort the current move.
        """
        self.group.stop()
        pass

    def change_speed_fraction(self, speedFraction : float):
        """
        Change the speed fraction setting for move [0 -> 1].

        """
        try:
            self.group.set_max_velocity_scaling_factor(speedFraction)
            print("Set Velocity scaling factor:= ",speedFraction)
            return True
        except:
            print("Exception in velocity scaling")
            return False 
    
    #TODO
    def enable(self, access_code : str):
        """
        Request to enable the robot. This command is admin privileged, an access code is required.
        
        Parameters:
            access_code (str): 
        """
        pass

    #TODO
    def disable(self, access_code : str):
        """
        Request to disable the robot. This command is admin privileged, an access code is required.
        
        Parameters:
            access_code (str): 
        """
        pass

    #TODO
    def close(self):
        moveit_commander.roscpp_shutdown()


    def add_obstacle(self,name="box1", length=1,width=1, height=1, radius=1, pose=[1,1,1,0,0,0], object_type="box", file_name="/home/user/mesh.stl", frame_id="base_link"):
        """
        Adds an obstacle to the planning scene based on the specified parameters.

        Args:
            object_type (str): Type of the obstacle ('plane' or 'sphere').
            name (str): Identifier for the obstacle.
            pose (list): Position and orientation of the obstacle.
            frame_id (str): The reference frame for the obstacle's position.

        Returns:
            bool: True if the obstacle was successfully added, False otherwise.
        """

        if(object_type =="box"):

            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = frame_id

            box_pose.pose.position.x = pose[0]
            box_pose.pose.position.y = pose[1]
            box_pose.pose.position.z = pose[2]

            box_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            box_pose.pose.orientation.x = quat[0]
            box_pose.pose.orientation.y = quat[1]
            box_pose.pose.orientation.z = quat[2]
            box_pose.pose.orientation.w = quat[3]

            box_name = name
            self.scene.add_box(box_name, box_pose, size=(length,width ,height))

            return True

            

        elif(object_type =="cone"):

            cone_pose = geometry_msgs.msg.PoseStamped()
            cone_pose.header.frame_id = frame_id

            cone_pose.pose.position.x = pose[0]
            cone_pose.pose.position.y = pose[1]
            cone_pose.pose.position.z = pose[2]

            cone_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            cone_pose.pose.orientation.x = quat[0]
            cone_pose.pose.orientation.y = quat[1]
            cone_pose.pose.orientation.z = quat[2]
            cone_pose.pose.orientation.w = quat[3]

            cone_name = name
            self.scene.add_cone(cone_name, cone_pose, height,radius)

            return True



        elif(object_type == "cylinder"):

            cylinder_pose = geometry_msgs.msg.PoseStamped()
            cylinder_pose.header.frame_id = frame_id

            cylinder_pose.pose.position.x = pose[0]
            cylinder_pose.pose.position.y = pose[1]
            cylinder_pose.pose.position.z = pose[2]

            cylinder_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            cylinder_pose.pose.orientation.x = quat[0]
            cylinder_pose.pose.orientation.y = quat[1]
            cylinder_pose.pose.orientation.z = quat[2]
            cylinder_pose.pose.orientation.w = quat[3]

            cylinder_name = name
            self.scene.add_cylinder(cylinder_name, cylinder_pose, height,radius)

            return True

        elif(object_type == "mesh"):

            mesh_pose = geometry_msgs.msg.PoseStamped()
            mesh_pose.header.frame_id = frame_id

            mesh_pose.pose.position.x = pose[0]
            mesh_pose.pose.position.y = pose[1]
            mesh_pose.pose.position.z = pose[2]

            mesh_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            mesh_pose.pose.orientation.x = quat[0]
            mesh_pose.pose.orientation.y = quat[1]
            mesh_pose.pose.orientation.z = quat[2]
            mesh_pose.pose.orientation.w = quat[3]

            mesh_name = name
            self.scene.add_mesh(mesh_name, mesh_pose, file_name)

            return True


        elif(object_type == "plane"):

            plane_pose = geometry_msgs.msg.PoseStamped()
            plane_pose.header.frame_id = frame_id

            plane_pose.pose.position.x = pose[0]
            plane_pose.pose.position.y = pose[1]
            plane_pose.pose.position.z = pose[2]

            plane_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            plane_pose.pose.orientation.x = quat[0]
            plane_pose.pose.orientation.y = quat[1]
            plane_pose.pose.orientation.z = quat[2]
            plane_pose.pose.orientation.w = quat[3]

            plane_name = name
            self.scene.add_plane(plane_name, plane_pose)

            return True


        elif(object_type == "sphere"):


            sphere_pose = geometry_msgs.msg.PoseStamped()
            sphere_pose.header.frame_id = frame_id

            sphere_pose.pose.position.x = pose[0]
            sphere_pose.pose.position.y = pose[1]
            sphere_pose.pose.position.z = pose[2]

            sphere_pose.pose.orientation.w = 1.0

            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

            sphere_pose.pose.orientation.x = quat[0]
            sphere_pose.pose.orientation.y = quat[1]
            sphere_pose.pose.orientation.z = quat[2]
            sphere_pose.pose.orientation.w = quat[3]

            sphere_name = name
            self.scene.add_sphere(sphere_name, sphere_pose,radius)
            return True



    def get_obstacles_list(self):
        """
        Retrieves a list of all obstacles currently present in the planning scene.

        Returns:
            list: A list of obstacle names.
        """        
        return list(self.scene.get_objects().keys())
        
    def remove_obstacle(self,name=None):
        """
        Removes a specified obstacle from the planning scene.

        Args:
            name (str, optional): Name of the obstacle to remove. Defaults to None.
        """

        self.scene.remove_world_object(name)
        return

    def set_home(self,pose="home",wait=True):
        """
        Sets the robot's position to a predefined home position.

        Args:
            pose (str): The name of the home position.
            wait (bool): Whether to wait for the move to complete.
        """

        self.group.set_named_target(pose)
        self.plan = self.group.go(wait=wait)
        print("Set home position")

    #Get signs of a joints in a list
    def get_signs(self,input_list):
        signs_list = []
        for num in input_list:
            if num > 0:
                signs_list.append(1)
            elif num < 0:
                signs_list.append(-1)
            else:
                signs_list.append(0)  # Assuming you want to handle zero as well
        return signs_list
    #Mulitply each element by element
    def multiply_lists(self,list1, list2):
        return [a * b for a, b in zip(list1, list2)]
    #Operate gripper based on value or fixed joint state
    def set_gripper(self,goal_state="open",goal_value=1,mode="state",wait=True):
        """
        Configures the gripper to a specific state or value.

        Args:
            goal_state (str): Target state for the gripper ('open' or 'close').
            goal_value (int): Target value for the gripper position.
            mode (str): Mode of setting the gripper ('state' or 'value').
            wait (bool): Whether to wait for the move to complete.
        """
        if self.gripper_enable == True:
            if(mode == "state"):
                self.gripper_group.set_named_target(goal_state)
                plan = self.gripper_group.go(wait=wait)
                self.gripper_group.stop()
                return True
            elif(mode == "value"):

                current_gripper_goal = self.gripper_group.get_current_joint_values()
                no_of_joints = len(current_gripper_goal)
                get_sign_list = self.get_signs(current_gripper_goal)
                new_joints_val = [goal_value] * no_of_joints
                final_joint_val = self.multiply_lists(new_joints_val,get_sign_list)

                self.gripper_group.set_joint_value_target(final_joint_val)
                
                # Plan and execute the motion
                plan = self.gripper_group.go(wait=True)
                # Calling `stop()` ensures that there is no residual movement
                self.gripper_group.stop()
                # It is always good to clear your targets after planning with poses.
                # Note: there is no equivalent function for clear_joint_value_targets().
                self.gripper_group.clear_pose_targets()

                return True
            
        else:
            return False
        
    #Get current value of gripper
    def get_gripper_val(self,wait=True):
        """
        Get the current value of gripper. Return joint values

        Args:
            wait (bool): Whether to wait for the move to complete.
        """        
        if self.gripper_enable == True:

            current_gripper_goal = self.gripper_group.get_current_joint_values()
            return current_gripper_goal
        
        else:
            current_gripper_goal = []
            return current_gripper_goal
        
