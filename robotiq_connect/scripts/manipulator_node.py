#!/usr/bin/env python3
import os
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Int8
import numpy as np
import math
import time
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


HOST = "192.168.20.35" # The remote host
PORT = 30002 # The same port as used by the server

# Get the path to the package's share directory
package_share_directory = ament_index_python.packages.get_package_share_directory('robotiq_connect')

# Build the full path to the gripper_activate.script file
script_path1 = os.path.join(package_share_directory, 'scripts', 'gripper_activate.script')
script_path2 = os.path.join(package_share_directory, 'scripts', 'gripper_close.script')
script_path3 = os.path.join(package_share_directory, 'scripts', 'gripper_open.script')

home_joint = [4.71, -1.57, 0, -1.57, -1.57, 0] #Set home for UR3e

set_case1 = [np.radians(270.34),np.radians(-107.72),np.radians(101.11),np.radians(-173.41),np.radians(-90.01),np.radians(0)] #Set UR3e in case z > 0 [m]
set_case2 = [4.71, -1.57, 1.57, -1.57, -1.57, 0] #Set UR3e in case z < 0 [m]

offset = [0, 0, 0.075, 1.5708, 0, 0] #array 0,1,2 for set offset in z axis 0.075 [m] | array 3,4,5 for set end-effect axis


def activated_gripper():
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Call Socket
    s.connect((HOST, PORT)) # Connect socket port
    f = open(script_path1,"rb") # Open UR-script **Not forget to change path of UR-script** 
    l = f.read() 
    s.send(l) # Send UR-script to UR3e
    print('ok')
    time.sleep(2)
    s.close()

def close_gripper(force,speed,range):

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    f = open(script_path2,"rb")
    add = "rq_set_force({})\r\n  rq_set_speed({})\r\n  rq_move_and_wait({})\r\n  \r\nend\r\n".format(force,speed,range)
    l = f.read()
    l2 = bytes(add, 'utf-8')
    send = l + l2
    s.send(send)
    time.sleep(1)
    s.close()

def open_gripper():

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    f = open(script_path3,"rb")
    l = f.read()
    s.send(l)
    time.sleep(1)
    s.close()

def activated_rtde():
    global rtde_c
    global rtde_r
    time.sleep(1)
    rtde_c = RTDEControl("192.168.20.35") # Connect ur_rtde via ip address
    rtde_r = RTDEReceive("192.168.20.35") # Connect ur_rtde via ip address

activated_gripper()
activated_rtde()

class Ability(Node):
    def __init__(self):
        super().__init__('Manipulation')
        self.recieve_timer = self.create_timer(0.008,self.ur_rtde_recieve_callback) #Timer for recieve value from UR3e
        self.timer = self.create_timer(0.1,self.timer_callback) #Timer for send status to system integration
        self.enble_service1 = self.create_service(Empty_srv,'/mani_grab/enable',self.mani_grab_enable_callback) #Create service for mani_grab ability
        self.enble_service2 = self.create_service(Empty_srv,'/mani_release/enable',self.mani_release_enable_callback) #Create service mani_release ability
        self.mani_grab_publisher = self.create_publisher(Int8,'mani_grab/status',10) #Create publisher for send status to system integration
        self.mani_release_publisher = self.create_publisher(Int8,'mani_release/status',10) #Create publisher for send status to system integration
        self.pose_subscription = self.create_subscription(Point32,'/position_from_rs2',self.reciceve_pose,10) #Topic for recieve value from object detection
        self.nav_publisher = self.create_publisher(Point32,'/mani_to_nav',10) #Topic for recieve value from object detection

        #Variable for service and status
        self.mani_grab_isEnable = False
        self.mani_release_isEnable = False
        self.mani_grab_status = Int8()
        self.mani_grab_status.data = 0
        self.mani_release_status = Int8()
        self.mani_release_status.data = 0

        #Set Home
        rtde_c.moveJ(home_joint,1.2,1.2,asynchronous = False)

        #Variable for check work space and move to goal pose
        self.pose = np.zeros((6,))
        self.moving_flag = 0
        self.new_pose = 0

        #Variable for calculate position in x,y axis to navigation
        #Navigate CACAO robot to these value in x,y axis in UR3e frame
        self.desired_x = 0.0 
        self.desired_y = 0.5
        # activated_gripper()
        # close_gripper(10,10,10)
        print("yeah")
        self.gripper_control(True)
        self.gripper_control(False)


    def timer_callback(self):
        self.mani_grab_publisher.publish(self.mani_grab_status)
        self.mani_release_publisher.publish(self.mani_release_status)

    def mani_grab_enable_callback(self,request,response):
        print("call service")
        activated_gripper()
        self.mani_grab_status.data = 0
        self.mani_grab_isEnable = False
        # self.move_to_goal_pose()
        close_gripper(10,10,10)
        return response

    def mani_release_enable_callback(self,request,response):
        activated_gripper()
        self.mani_release_status.data = 0
        print(self.mani_release_status.data)
        self.mani_release_isEnable = True
        # self.release_object()
        open_gripper()
        return response

    def ur_rtde_recieve_callback(self):
        self.current_pose = rtde_r.getActualTCPPose() #recieve current pose from UR3e
        self.joint_state = rtde_r.getActualQ() #recieve current joint config from UR3e

    def reciceve_pose(self,msg:Point32):
        
        # Reset flag
        self.new_pose = 1
        self.flag_worksapce = True

        # Change camera frame in rotation and translation to UR3e frame
        self.pose[0] = (msg.x*math.pow(10,-3) - 0.2)*-1
        self.pose[1] = msg.z*math.pow(10,-3) - 0.5 - 0.05 
        self.pose[2] = msg.y*math.pow(10,-3) +  0.34

        # Object offset of luggage [In this test we need to grab at handle. So pose from Object detection is middle point of luggage]
        self.pose[2] += 0.225

        print(self.pose)

        rtde_c.setTcp(offset) #Set TCP of end-effector

    def move_to_goal_pose(self): #Start this function when service /mani_grab/enable has been called and pose from object detection has sent
        if self.new_pose == 1:
            self.flag_worksapce = self.check_workspace(self.pose[0],self.pose[1],self.pose[2])

            if self.flag_worksapce: 
                # Control UR3e to goal pose using moveJ_IK
                rtde_c.moveJ_IK([self.pose[0],0.35,self.pose[2],self.pose[3],self.pose[4],self.pose[5]],0.5,0.5,asynchronous = False)
                rtde_c.moveJ_IK(self.pose,0.5,0.5,asynchronous = False)
                self.gripper_control(True)
            else:
                #Call PosToNav function when pose is over worksapce
                self.PosToNav(self.pose[0],self.pose[1])
                rtde_c.moveJ_IK([0,0.35,0.45,self.pose[3],self.pose[4],self.pose[5]],0.5,0.5,asynchronous = False) #***for test****#
                rtde_c.moveJ_IK([0,0.55,0.45,self.pose[3],self.pose[4],self.pose[5]],0.5,0.5,asynchronous = False) #***for test****#
                self.gripper_control(True)

    def check_workspace(self,x,y,z):

            flag = True
            if z > 0.0:
                if z < 0.0:
                    rtde_c.moveJ(set_case1,1,1,asynchronous = False)
                self.pose[3] = 0
                self.pose[4] = 1.569
                self.pose[5] = 0
                if y < 0.35: # Define y axis of UR3e not less than this value
                    flag = False
                    print("poseY fail")
            else:
                if z >= 0.0:
                    rtde_c.moveJ(set_case2,1,1,asynchronous = False)
                self.pose[3] = 2.4
                self.pose[4] = -2.4
                self.pose[5] = 2.416
                if y < 0.35:
                    flag = False
                    print("poseY fail")

            if z < -0.32: # Define z axis of UR3e not less than this value
                flag = False
                print("poseZ fail")

            # Check possibility to move to goal pose of UR3e
            if rtde_c.isPoseWithinSafetyLimits(self.pose):

                # Check joint config from inverse solve will collide with CACAO robot or not
                if rtde_c.getInverseKinematics(self.pose):
                    self.joint = rtde_c.getInverseKinematics(self.pose)
                    if np.radians(90.0)-np.radians(45.0) < self.joint[0] < np.radians(270.0)+np.radians(45.0):
                        pass
                    else:
                        flag = False
                        print("joint1 fail",self.joint[0])

                    if self.joint[1] > np.radians(30.0):
                        flag = False
                        print("joint2 fail",self.joint[1])

            else:
                flag = False
                print("pose fail")

            return flag

    def PosToNav(self,X,Y):
        # Calculate pose to navigation node
        send_nav = Point32()
        self.moveX = X - self.desired_x
        self.moveY = Y - self.desired_y

        send_nav.x = self.moveX
        send_nav.y = self.moveY
        send_nav.z = 0.0

        self.nav_publisher.publish(send_nav)
        print("To Nav is:",self.moveX, self.moveY)
        self.new_pose = 0

    def release_object(self): #Start this function when service /mani_release/enable has been called
        self.mani_release_status.data = 0
        rtde_c.moveJ(set_case1,1,1,asynchronous = False)
        rtde_c.moveJ_IK([0,0.55,0.45,0,1.569,0],0.5,0.5,asynchronous = False)
        self.gripper_control(False)

    def gripper_control(self,con):

        #When you use 2F-gripper you need to change from ur_rtde to socket package. (this is one of problem that we enconter now)
        rtde_c.disconnect()
        rtde_r.disconnect()

        if con: #grab
            print('start grab')
            close_gripper(50,50,200) #force,speed,pose
            activated_rtde()
            print('finish grab')
            rtde_c.moveJ(home_joint,1.2,1.2,asynchronous = False)

            ### Send status to system integration and reset value ###
            self.mani_grab_status.data = 1
            self.mani_grab_isEnable = False
            self.mani_grab_publisher.publish(self.mani_grab_status)
            self.new_pose = 0
   
        else: #release
            open_gripper()
            activated_rtde()
            rtde_c.moveJ_IK([0,0.35,0.45,0,1.569,0],0.5,0.5,asynchronous = False)
            rtde_c.moveJ(home_joint,1.2,1.2,asynchronous = False)

            ### Send status to system integration and reset value ###
            self.mani_release_status.data = 1
            self.mani_release_isEnable = False
            self.mani_release_publisher.publish(self.mani_release_status)
            self.new_pose = 0

def main(args=None):
    rclpy.init(args=args)
    
    ability = Ability()
    rclpy.spin(ability)
    ability.destroy_node()
    ability.shutdown()
    rclpy.shutdown()


if __name__=='__main__':
    main()
