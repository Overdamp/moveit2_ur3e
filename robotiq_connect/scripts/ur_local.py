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

# Network configuration for Local Mode
HOST = "192.168.20.35"  # UR3e IP in LAN
PORT = 30002  # Socket port for UR-script

# Get the path to the package's share directory
package_share_directory = ament_index_python.packages.get_package_share_directory('robotiq_connect')

# Build paths to UR-script files
script_path1 = os.path.join(package_share_directory, 'scripts', 'gripper_activate.script')
script_path2 = os.path.join(package_share_directory, 'scripts', 'gripper_close.script')
script_path3 = os.path.join(package_share_directory, 'scripts', 'gripper_open.script')

# Robot configuration
home_joint = [4.71, -1.57, 0, -1.57, -1.57, 0]  # Home position for UR3e
set_case1 = [np.radians(270.34), np.radians(-107.72), np.radians(101.11), np.radians(-173.41), np.radians(-90.01), np.radians(0)]  # Case z > 0 [m]
set_case2 = [4.71, -1.57, 1.57, -1.57, -1.57, 0]  # Case z < 0 [m]
offset = [0, 0, 0.075, 1.5708, 0, 0]  # TCP offset

class Ability(Node):
    def __init__(self):
        super().__init__('Manipulation')
        # Initialize RTDE and socket connections
        try:
            self.rtde_c = RTDEControl(HOST)
            self.rtde_r = RTDEReceive(HOST)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((HOST, PORT))
            print("Connected to UR3e at", HOST)
        except Exception as e:
            print(f"Failed to connect to UR3e: {e}")
            raise

        # Activate gripper at startup
        self._activate_gripper()

        # ROS2 setup
        self.recieve_timer = self.create_timer(0.008, self.ur_rtde_recieve_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.enble_service1 = self.create_service(Empty_srv, '/mani_grab/enable', self.mani_grab_enable_callback)
        self.enble_service2 = self.create_service(Empty_srv, '/mani_release/enable', self.mani_release_enable_callback)
        self.mani_grab_publisher = self.create_publisher(Int8, 'mani_grab/status', 10)
        self.mani_release_publisher = self.create_publisher(Int8, 'mani_release/status', 10)
        self.pose_subscription = self.create_subscription(Point32, '/position_from_rs2', self.reciceve_pose, 10)
        self.nav_publisher = self.create_publisher(Point32, '/mani_to_nav', 10)

        # Variables for service and status
        self.mani_grab_isEnable = False
        self.mani_release_isEnable = False
        self.mani_grab_status = Int8()
        self.mani_grab_status.data = 0
        self.mani_release_status = Int8()
        self.mani_release_status.data = 0

        # Move to home position
        self.rtde_c.moveJ(home_joint, 1.2, 1.2, asynchronous=False)

        # Variables for pose and movement
        self.pose = np.zeros((6,))
        self.moving_flag = 0
        self.new_pose = 0
        self.desired_x = 0.0
        self.desired_y = 0.5

    def _activate_gripper(self):
        """Activate gripper using UR-script"""
        try:
            with open(script_path1, "rb") as f:
                script = f.read()
                self.socket.send(script)
            print("Gripper activated")
            time.sleep(1)  # Reduced sleep for LAN
        except Exception as e:
            print(f"Gripper activation failed: {e}")

    def _close_gripper(self, force=50, speed=50, range=200):
        """Close gripper with specified parameters"""
        try:
            with open(script_path2, "rb") as f:
                script = f.read()
                add = f"rq_set_force({force})\r\nrq_set_speed({speed})\r\nrq_move_and_wait({range})\r\nend\r\n"
                send = script + bytes(add, 'utf-8')
                self.socket.send(send)
            print("Gripper closed")
            time.sleep(0.5)  # Reduced sleep for LAN
        except Exception as e:
            print(f"Gripper close failed: {e}")

    def _open_gripper(self):
        """Open gripper using UR-script"""
        try:
            with open(script_path3, "rb") as f:
                script = f.read()
                self.socket.send(script)
            print("Gripper opened")
            time.sleep(0.5)  # Reduced sleep for LAN
        except Exception as e:
            print(f"Gripper open failed: {e}")

    def timer_callback(self):
        self.mani_grab_publisher.publish(self.mani_grab_status)
        self.mani_release_publisher.publish(self.mani_release_status)

    def mani_grab_enable_callback(self, request, response):
        print("Mani grab service called")
        self.mani_grab_status.data = 0
        self.mani_grab_isEnable = True
        self.move_to_goal_pose()
        return response

    def mani_release_enable_callback(self, request, response):
        print("Mani release service called")
        self.mani_release_status.data = 0
        self.mani_release_isEnable = True
        self.release_object()
        return response

    def ur_rtde_recieve_callback(self):
        try:
            self.current_pose = self.rtde_r.getActualTCPPose()
            self.joint_state = self.rtde_r.getActualQ()
        except Exception as e:
            print(f"RTDE receive failed: {e}")

    def reciceve_pose(self, msg: Point32):
        self.new_pose = 1
        self.flag_workspace = True

        # Transform camera frame to UR3e frame
        self.pose[0] = (msg.x * math.pow(10, -3) - 0.2) * -1
        self.pose[1] = msg.z * math.pow(10, -3) - 0.5 - 0.05
        self.pose[2] = msg.y * math.pow(10, -3) + 0.34
        self.pose[2] += 0.225  # Luggage handle offset

        print("Received pose:", self.pose)
        self.rtde_c.setTcp(offset)

    def move_to_goal_pose(self):
        if self.new_pose == 1:
            self.flag_workspace = self.check_workspace(self.pose[0], self.pose[1], self.pose[2])
            if self.flag_workspace:
                self.rtde_c.moveJ_IK([self.pose[0], 0.35, self.pose[2], self.pose[3], self.pose[4], self.pose[5]], 0.5, 0.5, asynchronous=False)
                self.rtde_c.moveJ_IK(self.pose, 0.5, 0.5, asynchronous=False)
                self.gripper_control(True)
            else:
                self.PosToNav(self.pose[0], self.pose[1])
                self.rtde_c.moveJ_IK([0, 0.35, 0.45, self.pose[3], self.pose[4], self.pose[5]], 0.5, 0.5, asynchronous=False)
                self.rtde_c.moveJ_IK([0, 0.55, 0.45, self.pose[3], self.pose[4], self.pose[5]], 0.5, 0.5, asynchronous=False)
                self.gripper_control(True)

    def check_workspace(self, x, y, z):
        flag = True
        if z > 0.0:
            if z < 0.0:
                self.rtde_c.moveJ(set_case1, 1, 1, asynchronous=False)
            self.pose[3] = 0
            self.pose[4] = 1.569
            self.pose[5] = 0
            if y < 0.35:
                flag = False
                print("poseY fail")
        else:
            if z >= 0.0:
                self.rtde_c.moveJ(set_case2, 1, 1, asynchronous=False)
            self.pose[3] = 2.4
            self.pose[4] = -2.4
            self.pose[5] = 2.416
            if y < 0.35:
                flag = False
                print("poseY fail")

        if z < -0.32:
            flag = False
            print("poseZ fail")

        if self.rtde_c.isPoseWithinSafetyLimits(self.pose):
            joint = self.rtde_c.getInverseKinematics(self.pose)
            if joint and (np.radians(90.0) - np.radians(45.0) < joint[0] < np.radians(270.0) + np.radians(45.0)):
                if joint[1] <= np.radians(30.0):
                    return flag
                else:
                    print("joint2 fail", joint[1])
            else:
                print("joint1 fail", joint[0] if joint else "No solution")
            flag = False
        else:
            print("pose fail")
            flag = False
        return flag

    def PosToNav(self, X, Y):
        send_nav = Point32()
        self.moveX = X - self.desired_x
        self.moveY = Y - self.desired_y
        send_nav.x = self.moveX
        send_nav.y = self.moveY
        send_nav.z = 0.0
        self.nav_publisher.publish(send_nav)
        print("To Nav is:", self.moveX, self.moveY)
        self.new_pose = 0

    def release_object(self):
        self.mani_release_status.data = 0
        self.rtde_c.moveJ(set_case1, 1, 1, asynchronous=False)
        self.rtde_c.moveJ_IK([0, 0.55, 0.45, 0, 1.569, 0], 0.5, 0.5, asynchronous=False)
        self.gripper_control(False)

    def gripper_control(self, con):
        if con:  # Grab
            print("Start grab")
            self._close_gripper(force=50, speed=50, range=200)
            self.rtde_c.moveJ(home_joint, 1.2, 1.2, asynchronous=False)
            self.mani_grab_status.data = 1
            self.mani_grab_isEnable = False
            self.mani_grab_publisher.publish(self.mani_grab_status)
            self.new_pose = 0
        else:  # Release
            self._open_gripper()
            self.rtde_c.moveJ_IK([0, 0.35, 0.45, 0, 1.569, 0], 0.5, 0.5, asynchronous=False)
            self.rtde_c.moveJ(home_joint, 1.2, 1.2, asynchronous=False)
            self.mani_release_status.data = 1
            self.mani_release_isEnable = False
            self.mani_release_publisher.publish(self.mani_release_status)
            self.new_pose = 0

    def __del__(self):
        """Cleanup connections on node destruction"""
        try:
            self.socket.close()
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
            print("Connections closed")
        except Exception as e:
            print(f"Error closing connections: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        ability = Ability()
        rclpy.spin(ability)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        ability.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()