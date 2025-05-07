#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
import yaml
import time
import os
import re
from ament_index_python.packages import get_package_share_directory
from quaternion_utils import axis_angle_to_quaternion

class UR3ePoseReader(Node):
    """ROS 2 node to read and display the current pose of UR3e robot."""
    
    def __init__(self):
        super().__init__('ur3e_pose_reader')

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

        # Initialize socket
        self.socket = None
        self.connect_socket()

        # Timer to read pose periodically (ทุก 1 วินาที)
        self.timer = self.create_timer(1.0, self.read_pose)

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
                    self.get_logger().debug(f'Cleared buffer data: {repr(data)}')
            except socket.timeout:
                self.get_logger().info('Buffer cleared successfully')
            self.socket.settimeout(self.socket_timeout)
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

    def send_urscript(self, script):
        """Send URScript command to UR3e."""
        try:
            self.socket.send(script.encode('utf-8'))
            self.get_logger().debug(f'📤 Sent URScript: {script.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send URScript: {e}')
            self.reconnect_socket()

    def read_pose(self):
        """Read and display the current pose of UR3e."""
        if not self.socket:
            self.get_logger().error("❌ Socket not connected.")
            self.reconnect_socket()
            return

        try:
            self.get_logger().info('📍 Requesting current robot pose...')
            self.send_urscript('get_actual_tcp_pose()\n')
            self.socket.settimeout(5.0)

            # Receive data
            data = b''
            start_time = time.time()
            while time.time() - start_time < 5.0:
                try:
                    chunk = self.socket.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                except socket.timeout:
                    break

            # Decode and parse data
            decoded_data = data.decode('utf-8', errors='ignore')
            self.get_logger().debug(f'Received pose data: {decoded_data}')

            # Parse pose data (format: [x, y, z, rx, ry, rz])
            match = re.search(r'\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\]', decoded_data)
            if match:
                x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
                rx, ry, rz = float(match.group(4)), float(match.group(5)), float(match.group(6))

                # Convert axis-angle to quaternion
                qx, qy, qz, qw = axis_angle_to_quaternion(rx, ry, rz)

                # Display pose on terminal
                self.get_logger().info(
                    f'📍 Current UR3e Pose:\n'
                    f'Position: x={x:.5f}, y={y:.5f}, z={z:.5f}\n'
                    f'Orientation (quaternion): qx={qx:.5f}, qy={qy:.5f}, qz={qz:.5f}, qw={qw:.5f}\n'
                    f'Orientation (axis-angle): rx={rx:.5f}, ry={ry:.5f}, rz={rz:.5f}'
                )
            else:
                self.get_logger().error(f'❌ Failed to parse pose data: {decoded_data}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to read pose: {e}')
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
    node = UR3ePoseReader()
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