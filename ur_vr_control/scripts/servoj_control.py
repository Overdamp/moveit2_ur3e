#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import yaml
import time
import os
import math
from ament_index_python.packages import get_package_share_directory
from quaternion_utils import quaternion_to_axis_angle, quaternion_multiply, quaternion_inverse

class UR3eVRControl(Node):
    """ROS 2 node to control UR3e robot using relative VR controller data with TCP pose from ur_robot_driver."""
    
    def __init__(self):
        super().__init__('ur3e_vr_control')

        # Load configuration from ur3e_config.yaml
        try:
            config_path = os.path.join(
                get_package_share_directory('ur_vr_control'),
                'config',
                'ur3e_config.yaml'
            )
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            if self.config is None:
                raise ValueError("Failed to load ur3e_config.yaml: File is empty or malformed")
        except FileNotFoundError:
            self.get_logger().error(f"❌ ur3e_config.yaml not found at {config_path}")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load ur3e_config.yaml: {e}")
            raise
        
        # Robot connection parameters
        try:
            self.robot_ip = self.config["robot_control"]["robot_ip"]
            self.robot_port = self.config["robot_control"].get("robot_port", 30003)
            self.reconnect_interval = self.config["robot_control"].get("reconnect_interval", 5.0)
            self.socket_timeout = self.config["robot_control"].get("socket_timeout", 5.0)
        except KeyError as e:
            self.get_logger().error(f"❌ Missing key in ur3e_config.yaml: {e}")
            raise
        
        # Servoj parameters
        self.servoj_params = {
            'a': self.config["robot_control"]["acceleration"] * 1.0,
            'v': self.config["robot_control"]["max_speed"] * 1.0,
            't': self.config["servoj"]["time"],
            'lookahead_time': self.config["servoj"]["lookahead_time"],
            'gain': self.config["servoj"]["gain"]
        }
        self.get_logger().info(f'🛠️ Servoj params: a={self.servoj_params["a"]:.3f}, v={self.servoj_params["v"]:.3f}, '
                              f't={self.servoj_params["t"]:.3f}, lookahead_time={self.servoj_params["lookahead_time"]:.3f}, '
                              f'gain={self.servoj_params["gain"]:.3f}')
        
        # VR to robot mapping parameters
        self.vr_workspace = {
            'x_min': -1.0, 'x_max': 1.0,
            'y_min': -1.0, 'y_max': 1.0,
            'z_min': -1.0, 'z_max': 1.0
        }
        self.robot_limits = self.config["robot_control"]["workspace_limits"]
        
        # Calculate scaling
        self.scale_x = (self.robot_limits["x_max"] - self.robot_limits["x_min"]) / (self.vr_workspace["x_max"] - self.vr_workspace["x_min"])
        self.scale_y = (self.robot_limits["y_max"] - self.robot_limits["y_min"]) / (self.vr_workspace["y_max"] - self.vr_workspace["y_min"])
        self.scale_z = (self.robot_limits["z_max"] - self.robot_limits["z_min"]) / (self.vr_workspace["z_max"] - self.vr_workspace["z_min"])
        self.get_logger().info(f'🛠️ Position mapping: scale=[{self.scale_x:.3f}, {self.scale_y:.3f}, {self.scale_z:.3f}]')
        
        # Initialize socket
        self.socket = None
        self.connect_socket()
        
        # Subscribe to TCP pose from ur_robot_driver
        self.create_subscription(
            Pose,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
        self.get_logger().info('📥 Subscribed to /tcp_pose_broadcaster/pose')

        # Subscribe to VR controller topics
        self.create_subscription(
            Pose,
            'vr_controller/pose',
            self.vr_pose_callback,
            10
        )
        self.create_subscription(
            Bool,
            'vr_controller/gripper',
            self.gripper_callback,
            10
        )
        self.create_subscription(
            Bool,
            'vr_controller/move_enable',
            self.motion_enable_callback,
            10
        )
        self.get_logger().info('📥 Subscribed to /vr_controller/pose, /vr_controller/gripper, /vr_controller/move_enable')
        
        # State variables
        self.prev_gripper_state = False
        self.motion_enabled = False
        self.robot_pose = None  # From /tcp_pose_broadcaster/pose
        self.vr_prev_pose = None
        self.last_sent_pose = None
        self.current_vr_pose = None
        self.last_motion_enable_time = 0.0

    def connect_socket(self):
        """Establish TCP connection to UR3e robot."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(self.socket_timeout)
        try:
            self.get_logger().info(f'Attempting to connect to UR3e at {self.robot_ip}:{self.robot_port}')
            self.socket.connect((self.robot_ip, self.robot_port))
            self.get_logger().info(f'✅ Connected to UR3e at {self.robot_ip}:{self.robot_port}')
            # Clear socket buffer
            self.socket.settimeout(0.2)
            try:
                while True:
                    data = self.socket.recv(1024)
                    self.get_logger().info(f'Cleared buffer data: {repr(data)}')
            except socket.timeout:
                self.get_logger().info('Buffer cleared successfully')
            self.socket.settimeout(self.socket_timeout)

            # Unlock protective stop
            self.get_logger().info('📡 Unlocking protective stop...')
            self.send_urscript('unlock_protective_stop()\n')
            time.sleep(1.0)

            # Initialize TCP
            self.get_logger().info('🛠️ Setting default TCP...')
            self.send_urscript('set_tcp(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])\n')
            time.sleep(1.0)

            return True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to UR3e: {e}')
            return False

    def reconnect_socket(self):
        """Attempt to reconnect to robot if connection is lost."""
        self.get_logger().warn('⚠️ Attempting to reconnect to UR3e...')
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        while rclpy.ok() and not self.connect_socket():
            time.sleep(self.reconnect_interval)

    def tcp_pose_callback(self, msg):
        """Callback to store the current TCP pose from ur_robot_driver."""
        self.robot_pose = msg
        self.get_logger().info(f'📍 Received TCP pose: x={msg.position.x:.5f}, y={msg.position.y:.5f}, z={msg.position.z:.5f}, '
                              f'qx={msg.orientation.x:.5f}, qy={msg.orientation.y:.5f}, qz={msg.orientation.z:.5f}, qw={msg.orientation.w:.5f}')

    def vr_pose_callback(self, msg):
        """Process relative VR controller pose and send servoj command to UR3e."""
        if not self.socket:
            self.get_logger().error("❌ Socket not connected.")
            self.reconnect_socket()
            return

        self.current_vr_pose = msg
        self.get_logger().info(f'📡 Received VR pose: x={msg.position.x:.5f}, y={msg.position.y:.5f}, z={msg.position.z:.5f}, '
                              f'qx={msg.orientation.x:.5f}, qy={msg.orientation.y:.5f}, qz={msg.orientation.z:.5f}, qw={msg.orientation.w:.5f}')

        if not self.motion_enabled:
            self.get_logger().warn("🔒 Motion disabled, sending stopj command.")
            self.send_urscript("stopj(1.0)\n")
            return
        if self.robot_pose is None:
            self.get_logger().error("❌ No robot pose received from /tcp_pose_broadcaster/pose.")
            return
        if self.vr_prev_pose is None:
            self.vr_prev_pose = msg
            self.get_logger().info("📍 Initial VR pose set for delta calculation.")
            return

        # Map VR delta to robot workspace (relative movement)
        delta_x = self.scale_x * msg.position.x
        delta_y = self.scale_y * msg.position.y
        delta_z = self.scale_z * msg.position.z
        self.get_logger().info(f'🛠️ Computed deltas: dx={delta_x:.5f}, dy={delta_y:.5f}, dz={delta_z:.5f}')

        # Limit delta for safety
        max_delta = 0.1
        if abs(delta_x) > max_delta or abs(delta_y) > max_delta or abs(delta_z) > max_delta:
            self.get_logger().warn(f'⚠️ Robot delta too large: dx={delta_x:.5f}, dy={delta_y:.5f}, dz={delta_z:.5f}')
            self.vr_prev_pose = msg
            return

        # Compute target position relative to current robot pose
        x = self.robot_pose.position.x + delta_x
        y = self.robot_pose.position.y + delta_y
        z = self.robot_pose.position.z + delta_z
        self.get_logger().info(f'🎯 Target pose: x={x:.5f}, y={y:.5f}, z={z:.5f}')

        # Smooth target position
        alpha = 0.05
        if self.last_sent_pose is None:
            self.last_sent_pose = [x, y, z]
        else:
            x = alpha * x + (1 - alpha) * self.last_sent_pose[0]
            y = alpha * y + (1 - alpha) * self.last_sent_pose[1]
            z = alpha * z + (1 - alpha) * self.last_sent_pose[2]
            self.last_sent_pose = [x, y, z]
        self.get_logger().info(f'📏 Smoothed pose: x={x:.5f}, y={y:.5f}, z={z:.5f}')

        # Validate target position
        if not self.validate_pose(x, y, z):
            return

        # Compute quaternion delta for orientation
        q_vr_current = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_vr_prev = [self.vr_prev_pose.orientation.x, self.vr_prev_pose.orientation.y,
                     self.vr_prev_pose.orientation.z, self.vr_prev_pose.orientation.w]
        q_robot_current = [self.robot_pose.orientation.x, self.robot_pose.orientation.y,
                           self.robot_pose.orientation.z, self.robot_pose.orientation.w]
        
        try:
            q_delta = quaternion_multiply(q_vr_current, quaternion_inverse(q_vr_prev))
            q_robot_new = quaternion_multiply(q_delta, q_robot_current)
            rx, ry, rz = quaternion_to_axis_angle(q_robot_new[0], q_robot_new[1], q_robot_new[2], q_robot_new[3])
            self.get_logger().info(f'🔄 Computed axis-angle: rx={rx:.5f}, ry={ry:.5f}, rz={rz:.5f}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to process quaternion: {e}')
            self.vr_prev_pose = msg
            return

        # Create servoj command
        servoj_cmd = (
            f'servoj([{x:.5f},{y:.5f},{z:.5f},{rx:.5f},{ry:.5f},{rz:.5f}], '
            f'a={self.servoj_params["a"]:.5f}, v={self.servoj_params["v"]:.5f}, '
            f't={self.servoj_params["t"]:.5f}, lookahead_time={self.servoj_params["lookahead_time"]:.5f}, '
            f'gain={self.servoj_params["gain"]:.5f})\n'
        )

        # Update robot pose for next iteration
        self.robot_pose.position.x = x
        self.robot_pose.position.y = y
        self.robot_pose.position.z = z
        self.robot_pose.orientation.x = q_robot_new[0]
        self.robot_pose.orientation.y = q_robot_new[1]
        self.robot_pose.orientation.z = q_robot_new[2]
        self.robot_pose.orientation.w = q_robot_new[3]
        self.vr_prev_pose = msg

        self.get_logger().info(f'🚀 Sending servoj command: {servoj_cmd.strip()}')
        self.send_urscript(servoj_cmd)

    def validate_pose(self, x, y, z):
        """Check if position is within workspace limits."""
        valid = (self.robot_limits["x_min"] <= x <= self.robot_limits["x_max"] and
                 self.robot_limits["y_min"] <= y <= self.robot_limits["y_max"] and
                 self.robot_limits["z_min"] <= z <= self.robot_limits["z_max"])
        if not valid:
            self.get_logger().warn(f'⚠️ Target pose out of workspace: x={x:.5f}, y={y:.5f}, z={z:.5f}')
        return valid

    def gripper_callback(self, msg):
        """Control gripper based on VR controller button 1 with debounce."""
        if not self.socket:
            self.get_logger().error("❌ Socket not connected.")
            self.reconnect_socket()
            return

        if msg.data and not self.prev_gripper_state:
            script = 'set_standard_digital_out(0, True)\n'
            self.get_logger().info('🖐️ Gripper closing')
            self.send_urscript(script)
        elif not msg.data and self.prev_gripper_state:
            script = 'set_standard_digital_out(0, False)\n'
            self.get_logger().info('🖐️ Gripper opening')
            self.send_urscript(script)

        self.prev_gripper_state = msg.data

    def motion_enable_callback(self, msg):
        """Enable or disable motion based on VR controller button 2 with debounce."""
        current_time = time.time()
        if current_time - self.last_motion_enable_time < 0.2:
            return

        self.motion_enabled = msg.data
        self.last_motion_enable_time = current_time

        if self.motion_enabled:
            self.last_sent_pose = None
            self.vr_prev_pose = None
            self.get_logger().info('🔄 Motion enabled')
        else:
            self.get_logger().info('🔒 Motion disabled, sending stopj command.')
            self.send_urscript("stopj(1.0)\n")

    def send_urscript(self, script):
        """Send URScript command to UR3e."""
        try:
            self.socket.send(script.encode('utf-8'))
            self.get_logger().debug(f'📤 Sent URScript: {script.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send URScript: {e}')
            self.reconnect_socket()

    def destroy_node(self):
        """Clean up resources on shutdown."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.get_logger().info('❎ Disconnected from UR3e')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UR3eVRControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()