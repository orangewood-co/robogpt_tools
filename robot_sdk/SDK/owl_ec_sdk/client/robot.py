#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
Created on Fri Apr 19 16:41:50 2024
@author: Orangewood Labs Inc.
"""

import socket
import json
import math

class Owl_Ec_client:

    def __init__(self, ip, port=8055, tool_num = 2, user_num = 0):
        self.ip_address = ip
        self.port = port
        self.connect_success = False
        self.sock = None
        self.tool_num = tool_num
        self.user_num = user_num

############### Primary functions ########################
    def ip_test(self):
        print(f"Ip address being used in below functions is {self.ip_address}")
        return self.ip_address
    
    def connect(self):
        '''
        This function is used to establish a connection with the robot.

        Returns:
            list: [bool, str] -> [True, "Robot connected at IP (ACTUAL IP)"] or [False, "Cannot connect at IP (ACTUAL IP)"]
        '''
        def connectETController(ip, port=8055):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect((ip, port))
                return (True, sock)
            except Exception as e:
                sock.close()
                return (False,)

        try:
            conSuc, sock = connectETController(self.ip_address)
            print(f"Connection status:: {conSuc}")

        except:
            return [False, f"Cannot connect at IP:{self.ip_address}, check if robot is powered ON and connected to this system."]

        if conSuc:
            self.connect_success = conSuc
            self.sock = sock
            return [True, f"Robot connected at IP:{self.ip_address}"]
        else:
            return [False, f"Cannot connect at IP:{self.ip_address}"]
    
    def disconnect(self):
        '''
        This function is used to disconnect from the robot.

        Returns:
            list: [bool, str] -> [True, "Robot Disconnect Success"]
        '''
        if self.sock:
            self.sock.close()
            self.sock = None
            return [True, "Robot Disconnect Success"]
        else:
            self.sock = None
            return [True, "Robot Disconnect Success, Robot was already disconected."]


    def sendCMD(self, cmd, params=None, id=1):
        '''
        Note: This function should not be used directly, instead use the execute function.

        Internal Function to send commands to the robot.
        This function is essentially the same as mentioned in the 
        official Programming Manual.
        All commands are mentioned in the Programming Manual.

        Attributes:
            cmd (string): The command that we want to send.
            params (dict): If the commands requires additional parameters, specify here.

        '''
        sock = self.sock

        if (not params):
            params = []
        else:
            params = json.dumps(params)
        sendStr = "{{\"method\":\"{0}\",\"params\":{1} ,\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(
            cmd, params, id)+"\n"

        try:
            sock.sendall(bytes(sendStr, "utf-8"))
            ret = sock.recv(1024)
            jdata = json.loads(str(ret, "utf-8"))
            if ("result" in jdata.keys()):
                return (True, json.loads(jdata["result"]), jdata["id"])
            elif ("error" in jdata.keys()):
                return (False, jdata["error"], jdata["id"])
            else:
                return (False, None, None)
        except Exception as e:
            return (False, None, None)

    def calculate_inverse_kinematics(self, target_pose):
        """
        Calculate inverse kinematics for a given target pose with respect to a reference pose.

        Parameters:
            robot_ip (str): The IP address of the robot.
            target_pose (list): The target pose to calculate inverse kinematics for.

        Returns:
            list or None: The calculated joint angles if successful, None otherwise.
        """
        # Reference point
        ref = []
        ref = self.getjointangles()

        self.connect()
        
        if self.connect_success:
            try:
                # Perform inverse kinematics calculation
                suc, result, id = self.sendCMD("inverseKinematic", {"targetPose": target_pose, "referencePos": ref})
                if suc:
                    return result
                else:
                    print("Inverse kinematics failed for this pose:", result)
                    return None

            except Exception as e:
                print("Error:", e)
            finally:
                # Disconnect from the robot controller
                self.disconnect()
        else:
            print("Connection to the robot failed.")
            return None

    def movel(self,points):
        '''
        This function is used to make robot move in linear motion to a goal pose

        Attributes:
            points (List[float]): The goal pose where robot needs to go

        Returns:
            bool: True if the the robot has reached the desired pose and False otherwise
        '''    

        self.connect()

        if not self.connect_success:
            return None  # Connection failed

        try:
            suc, result, id = self.sendCMD("set_servo_status", {"status": 1})
            angle_point = self.calculate_inverse_kinematics(points)
            self.connect()

            # Linear motion
            print("Moving to point linearly:", angle_point)

            # Check if angle_point is valid
            if angle_point is None:
                print("Error: Target position for linear motion is invalid.")
                return

            suc, result, id = self.sendCMD("moveByLine", {
                "targetPos": angle_point,
                "speed": 120,
                "acc":10,
                "dec": 10,
                "cond_type": 0,
                "cond_num": 7,
                "cond_value": 1})

            if not suc:
                print("Error in moveByLine:", result)
                return

            while True:
                # Get robot status
                suc, result, id = self.sendCMD("getRobotState")
                if result == 0:
                    break

        except Exception as e:
            print("Error in moveRobotToPoint:", e)
        finally:
            self.disconnect()

    def movej(self,joint_angles):
        '''
        This function is used to make robot move in joint_space to a goal pose

        Attributes:
            points (List[float]): The goal joint angles where robot needs to go should be in radian

        Returns:
            bool: True if the the robot has reached the desired pose and False otherwise
        '''    
        # Converting the joint angles from radian to degree format 
        joint_angles = [math.degrees(angle) for angle in joint_angles]

        self.connect()

        if not self.connect_success:
            return None  # Connection failed
        
        try:
            suc, result, id = self.sendCMD("set_servo_status", {"status": 1})

            # Motion in joint space
            print("Moving to point in joint space:", joint_angles)

            # Check if angle_point is valid
            if joint_angles is None:
                print("Error: Target position for move by joint is invalid.")
                return
            
            suc, result, id = self.sendCMD("moveByJoint", {
                "targetPos": joint_angles,
                "speed": 120,
                "acc":10,
                "dec": 10,
                "cond_type": 0,
                "cond_num": 7,
                "cond_value": 1})

            if not suc:
                print("Error in moveByJoint:", result)
                return

            while True:
                # Get robot status
                suc, result, id = self.sendCMD("getRobotState")
                if result == 0:
                    break

        except Exception as e:
            print("Error in moveRobotToPoint:", e)
        finally:
            self.disconnect()


    def movec(self, points, nexpoints):
        '''
        This function is used to make robot move in an arc motion having three waypoints.

        Attributes:
            points (List[float]): The first waypoint were robot needs to go for arc motion
            nexpoints (List[Float]): the next waypoint in the arc motion

        Returns:
            bool: True if the the robot has reached the desired pose and False otherwise
        '''        
        self.connect()

        if not self.connect_success:
            return None  # Connection failed

        try:
            suc, result, id = self.sendCMD("set_servo_status", {"status": 1})
            P000 = self.calculate_inverse_kinematics(points)
            P001 = self.calculate_inverse_kinematics(nexpoints)
            self.connect()

            # Angular motion
            print("Moving to point angular:", P000)
            print("Target point angular:", P001)

            suc, result, id = self.sendCMD("moveByArc", {
                "midPos": P000,
                "targetPos": P001,
                "speed_type": 0,
                "speed": 150,
                "cond_type": 0,
                "cond_num": 7,
                "cond_value": 1
            })

            if not suc:
                print("Error in moveByArc:", result)
                return

            while True:
                # Get robot status
                suc, result, id = self.sendCMD("getRobotState")
                if result == 0:
                    break

        except Exception as e:
            print("Error in moveRobotToPoint:", e)
        finally:
            self.disconnect()

    def getjointangles(self):
        '''
        This function is used to get the tcp position of the robot.

        Returns:
            List: list of the coordinates in cartesian space
        '''

        # Connecting Robot Controller
        self.connect()

        if self.connect_success:
            ret, result, id = self.sendCMD("get_joint_pos")
            return result
        else:
            print("Connection Failed")
            # Disconnecting from Robot controller
            self.disconnect()
            return None
            
    
    def get_tcp_position(self):
        '''
        This function is used to get the tcp position of the robot.

        Returns:
            List: list of the coordinates in cartesian space
        '''

        # Connecting Robot Controller
        self.connect()
        print(f"Second check for connection status::{self.connect_success}")

        if self.connect_success:
            # Get the actual tcp pose of the tool 1 in the user 1 coordinate system
            suc, result , id = self.sendCMD("get_actual_tcp",{"tool_num":self.tool_num,"user_num":self.user_num})
            return result
        else :
            print("Connection Failed")
            # Disconnecting from Robot controller
            self.disconnect()
            return None

    def get_digital_io(self, io_number):
        '''
        This function is used to get the digital Input Output pins High or Low.
        Attributes:
            io_number (Int): The Io pin number.
        Returns:
            bool: 1 if the Io is high, 0 if io if low and 2 otherwise.
        '''
        # Connecting Robot Controller
        self.connect()
        if self.connect_success:
            try:
                suc, result, id = self.sendCMD("getOutput", {"addr": io_number})
                if result:
                    message = "Digital I/O {} is on.".format(io_number)
                    print(message)
                    return 1
                else:
                    message = "Digital I/O {} is off.".format(io_number)
                    print(message)
                    return 0
            except Exception as e:
                print("Error:", e)
                return -1
                
            finally:
                # Disconnect from the robot controller
                self.disconnect()
        else:
            print("Connection to the robot failed.")
            return -1
    
    def getjointacc(self):
        '''
        This function is used to obtain the joint acceleration
        Returns:
            List: Joint motion acceleration double joint_acc[6], unit: degree/s2
        '''
        # Connecting Robot Controller
        self.connect()
        if self.connect_success:
            while 1:
                suc, result , id = self.sendCMD("get_joint_acc")
                return result
        else :
            print("Connection Failed")
            # Disconnecting from Robot controller
            
            self.disconnect()
            return None
        
    def getjointspeed(self):
        '''
        This function is used to obtain the joint speed
        Returns:
            List: Joint movement speed double speed[6], unit: degree/s
        '''
        # Connecting Robot Controller
        self.connect()
        if self.connect_success:
            while 1:
                suc, result , id = self.sendCMD("get_joint_speed")
                print (suc,result,id)
                return result
        else :
            print("Connection Failed")
            # Disconnecting from Robot controller
            
            self.disconnect()
            return None

    def set_digital_io(self, io_number, turn_on):
        '''
        This function is used to set the digital Input Output pins High or Low.

        Attributes:
            io_number (Int): The Io pin number.
            turn_on (bool) : Switch of io High or low

        Returns:
            bool: True if the Io is set high, False otherwise.
        '''

        # Connecting Robot Controller
        self.connect()

        if self.connect_success:
            try:
                # Set output IO status based on turn_on argument
                status = 1 if turn_on else 0
                suc, result, id = self.sendCMD("setOutput", {"addr": io_number, "status": status})
                if suc:
                    message = "Digital I/O {} turned on.".format(io_number) if turn_on else "Digital I/O {} turned off.".format(io_number)
                    print(message)
                    return True
                else:
                    print("Failed to set digital I/O {}:".format(io_number), result)
                    return False

            except Exception as e:
                print("Error:", e)
                return False
                
            finally:
                # Disconnect from the robot controller
                self.disconnect()
        else:
            print("Connection to the robot failed.")
            return False


    def set_hand_drag(self, switch):
        '''
        This function is used to set the hand drag mode.

        Attributes:
            swtich (bool) : switch to set the hand drag mode. 

        Returns:
            bool: True if the drag mode is on, False otherwise.
        '''

        # connecting to robot controller
        self.connect()

        if self.connect_success():
            if switch:
                suc, result, id = self.sendCMD("drag_teach_switch",{"switch":1})
            if not switch:
                suc, result, id = self.sendCMD("drag_teach_switch",{"switch":0})
            
            if suc:
                print("Enabled Hand drag mode Successfully")
                return result
            if not suc:
                print("Error in Enabling Hand drag mode")
        else:
            return None 
            print("Connection Failed")
            # Disconnecting from Robot controller
            self.disconnect()


############## Secondary Fuctions ##############################
#     
    def execute(self, command: str, params: dict=None) -> list:
        '''
        This function is used to execute commands on the robot. Better version of the official sendCMD function.

        Attributes:
            command (string): The command that we want to execute.
            params (dict): If the commands requires additional parameters, specify here.

        Returns:
            list: [int, str] -> [ID, "Result of the command"]
        '''
        if self.connect_success:
            if (not params):
                suc, result, id = self.sendCMD(command)
            else:
                suc, result, id = self.sendCMD(command, params)

            if suc:
                return [id, result]
            else:
                # Check if the not executed due to remote mode not enabled
                if result["code"] == -32693:
                    raise Exception(
                        f"Robot must be in Remote Mode to execute {command}. Enable remote mode from the teach pendant.")
                else:
                    raise Exception(f"Can not execute: {result}")
        else:
            return [False, "Robot not connected."]


    def getStatus(self) -> str:
            '''
            Get the status of the robot. The status can be one of the following:
            - STOP
            - PAUSE
            - EMERGENCY_STOP
            - RUNNING
            - ALARM
            - COLLISION

            Returns:
                str: The status of the robot.

            '''
            STATES = [
                "STOP",
                "PAUSE",
                "EMERGENCY_STOP",
                "RUNNING",
                "ALARM",
                "COLLISION",
            ]

            if self.connect_success:
                _, result = self.execute("getRobotState")
                return STATES[result]
            else:
                raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def getRobotMode(self) -> str:
        '''
        Get the mode of the robot. The mode can be one of the following:
        - TEACHING
        - OPERATING
        - REMOTE

        Returns:
            str: The mode of the robot.

        '''

        MODES = [
            "TEACHING",
            "OPERATING",
            "REMOTE"
        ]

        if self.connect_success:
            _, result = self.execute("getRobotMode")
            return MODES[result]
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    # Collision Options
    def getCollisionStatus(self) -> bool:
        '''
        Get the collision status of the robot.

        Returns:
            bool: True if collision is detected, False otherwise.

        '''

        if self.connect_success:
            _, result = self.execute("getCollisionEnable")
            return result == 1
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")


    def setCollisionDetection(self, status: bool) -> bool:
        '''
        Set the collision detection status of the robot.

        Attributes:
            status (bool): True to enable collision detection, False to disable.

        Returns:
            bool: True if collision detection status is set, False otherwise.

        '''
        if self.connect_success:
            _, result = self.execute("setCollisionEnable", {"enable": status})
            return result
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")
        

    def getVariable(self, variable_type: str, variable_address: int) -> list:
        '''
        This function is used to get the value of a variable from the robot.

        Attributes:
            variable_type (string): The type of variable we want to get, for example: SysVarI, SysVarB, etc.
            variable_address (int): The address of the variable we want to get.

        Returns:
            list: [int, str] -> [ID, Value of the variable]
        '''

        if self.connect_success:
            suc, result, id = self.sendCMD(self.sock, f"get{variable_type}", {
                                           "addr": variable_address})
            if suc:
                return [id, result]
            else:
                raise Exception(f"Can not get variable: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def setVariable(self, variable_type: str, variable_address: int, variable_value: int) -> list:
        '''
        This function is used to set the value of a variable from the robot.

        Attributes:
            variable_type (string): The type of variable we want to set, for example: SysVarI, SysVarB, etc.
            variable_address (int): The address of the variable we want to set.
            variable_value (int): The value of the variable we want to set.

        Returns:
            list: [int, str] -> [ID, Result of the command]
        '''
        if self.connect_success:
            suc, result, id = self.sendCMD(f"set{variable_type}", {
                "addr": variable_address, "value": variable_value})
            if suc:
                return [id, result]
            else:
                raise Exception(f"Can not set variable: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def setServoStatus(self, status: int) -> list:
        '''
        This function is used to set the servo status of the robot.

        Attributes:
            status (int): The status of the servo, 0 for OFF and 1 for ON.

        Returns:
            list: [int, str] -> [ID, Result of the command]
        '''
        if self.connect_success:
            suc, result, id = self.sendCMD(
                self.sock, "set_servo_status", {"status": status})
            if suc:
                return [id, result]
            else:
                raise Exception(f"Can not set servo status: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def stopOperation(self) -> bool:
        '''
        This function is used to stop the robot operation.

        Returns:
            bool: True if the robot is stopped, False otherwise.
        '''
        if self.connect_success:
            suc, result, _ = self.execute("stop")
            if suc:
                return result
            else:
                raise Exception(f"Can not stop robot: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")
    
    def pause(self) -> bool:
        '''
        This function is used to pause the robot operation.

        Returns:
            bool: True if the robot is paused, False otherwise.
        '''
        if self.connect_success:
            suc, result, _ = self.execute("pause")
            if suc:
                return result
            else:
                raise Exception(f"Can not pause robot: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")
    
    def run(self) -> bool:
        '''
        This function is used to run the robot operation.

        Returns:
            bool: True if the robot is running, False otherwise.
        '''
        if self.connect_success:
            suc, result, _ = self.execute("run")
            if suc:
                return result
            else:
                raise Exception(f"Can not run robot: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")
        
    def setSpeed(self, speed) -> bool:
        '''
        This function is used to set the robot running speed.

        Attributes:
            speed (double [0.05,100]): The speed value.

        Returns:
            bool: True if the speed is set, False otherwise.
        '''
        if self.connect_success:
            id, result = self.execute("setSpeed", {"value": speed})
            return result
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")
        
    # JBI File Operations
    def checkJbiExist(self, jbi_filename: str) -> bool:
        '''
        This function is used to check if a jbi file exists on the robot.

        Attributes:
            jbi_filename (string): The name of the jbi file we want to check.

        Returns:
            bool: True if the jbi file exists, False otherwise.
        '''
        if self.connect_success:
            id, result = self.execute(
                "checkJbiExist", {"filename": jbi_filename})
            return result == 1
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def runJbi(self, jbi_filename) -> list:
        '''
        This function is used to run a jbi file on the robot.

        Attributes:
            jbi_filename (string): The name of the jbi file we want to run.

        Returns:
            list: [int, str] -> [ID, Result of the command]
        '''

        if self.connect_success:
            if self.checkJbiExist(jbi_filename):
                id, result = self.execute(
                    "runJbi", {"filename": jbi_filename})
                return [id, result]
            else:
                raise Exception(f"Jbi file {jbi_filename} does not exist.")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def getJbiState(self) -> list:
        '''
        This function is used to get the state of the jbi file running on the robot.

        Returns:
            list: [int, str] -> [ID, Result of the command]
        '''
        if self.connect_success:
            suc, result, id = self.sendCMD(self.sock, "getJbiState")
            if suc:
                return [id, result]
            else:
                raise Exception(f"Can not get jbi state: {result}")
        else:
            raise Exception(f"Robot not connected at IP:{self.ip_address}")

    def __repr__(self):
        return f'Robot Object at IP:{self.ip_address}, Connection:{self.connect_success}'


if __name__ == "__main__":
    print("This is a library file, not a standalone script.")
    