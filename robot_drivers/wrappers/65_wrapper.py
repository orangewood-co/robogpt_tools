#!/usr/bin/env python3

"""
Python wrapper for Orangewood simulation SDK APIs
"""
import time
import sys
import signal
import rospkg
import json
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_drivers')    
sys.path.append(package_path)

from owl_client import OwlClient, Pose, Joint

# This class deals with moveit wrapper specially for owl65. Using Compute cartesian method
# for Owl65
class MoveitHelperFunct():

    def __init__(self):
        try:
            self._robot_ip = rospy.get_param("robot_ip",default="10.42.0.52")
        except:
            rospy.logwarn("Unable to get the robot_ip...")
            sys.exit(-1)
        self.client = OwlClient(self._robot_ip)
        self.joint_speed = 50
        self.tool_speed = 100
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a RobotCommander object. This object is the outer-level interface to the robot:
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. 
        
        # In this case, the group is the primary arm of the robot:
        self.group_name = "OwlArm"  # change this to your group name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planner_id("PTP")
        
        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    
    
    def get_ros_pose(self,pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]

        if len(pose) == 6:
            quaternion = quaternion_from_euler(pose[3],pose[4],pose[5])
        elif len(pose) == 7:
            quaternion = pose[3:]

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        return pose_goal
    
    def pose_conversion(self,waypoint_list):
        pose_list = []
        for point in waypoint_list:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = point[0]
            pose_goal.position.y = point[1]
            pose_goal.position.z = point[2]

            quaternion = quaternion_from_euler(point[3], point[4], point[5])
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]

            pose_list.append(pose_goal)
        return pose_list
        
    def move_trajectory(self, poses: list):
        pose_list = self.pose_conversion(poses)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
                             pose_list,   # waypoints to follow
                             0.01,        # eef_step
                             False)         # jump_threshold

        for i in range(20):
            self.move_group.execute(plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement:
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

#------------------------------------------------------------------------------#
# Main wrapper class

class bot_wrapper:

    def __init__(self,ip):
        
        print("Successfully loaded the SDK for OWL 6.5 series")
        self.client = OwlClient(ip)
        self.moveit_helper = MoveitHelperFunct()

    def get_tcp(self):
        '''
        Function to get the tcp coordinates in Castersian space
        '''
        return self.client.get_tcp().get_pose()

    def get_joints(self):
        '''
        Function to get the current joint angles of robot
        '''
        return self.client.get_joint().get_joints()
    
    def set_hand_teach(self,switch):
        '''
        Function to enable hand drag 
        '''
        if switch:
            return self.client.enter_teach_mode()
        if not switch:
            return self.client.end_teach_mode()
    
    def move_to_pose(self,pose):
        '''
        Function to move robot in cartersian space. This uses the moveit
        wrapper for planning using compute cartersian method. Useing a sigle 
        goal as a list of poses for trajactory poses
        '''
        poses = [pose]
        print("Moving to pose", poses)
        return self.moveit_helper.move_trajectory(poses)
    
    def set_gripper(self,pin_no,switch):
        '''
        Function to control any I/O pin from the control box
        '''
        return self.client.set_digital_output(digital_pin=pin_no,digital_status=switch)

    def move_to_joint(self, joints):
        '''
        Function to move the robot to desired joint angles
        '''
        joint_goal = Joint(*joints)
        return self.client.move_to_joint(jointPose=joint_goal,toolSpeed=50)
        
    def move_trajectory(self,waypoints):
        '''
        # Function to move robot in a trajactory of waypoints
        '''
        return self.moveit_helper.move_trajectory(poses=waypoints)

    def move_translate(self,x,y,z):
        '''
        # Function to make robot in a particular axis or plane
        '''
        return self.client.move_translate(x,y,z)

    def test(self):
        '''
        Function to test the instances
        '''
        print("WORKING FINE")
