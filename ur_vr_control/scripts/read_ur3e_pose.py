#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
import yaml
import time
import os
import re
import struct
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
                    self.get_logger().info(f'Cleared buffer data: {repr(data)}')
            except socket.timeout:
                self.get_logger().info('Buffer cleared successfully')

            # Unlock protective stop
            self.get_logger().info('📡 Unlocking protective stop...')
            self.socket.settimeout(self.socket_timeout)
            self.send_urscript('unlock_protective_stop()\n')
            time.sleep(1.0)

            # Check robot status
            self.get_logger().info('📡 Checking robot status...')
            self.send_urscript('get_robot_status()\n')
            time.sleep(1.0)
            data = b''
            self.socket.settimeout(2.0)
            try:
                while True:
                    chunk = self.socket.recv(1024)
                    if not chunk:
                        break
                    data += chunk
            except socket.timeout:
                pass
            self.get_logger().info(f'Robot status: {data.decode("utf-8", errors="ignore")}')

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
            self.get_logger().info(f'📤 Sent URScript: {script.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send URScript: {e}')
            self.reconnect_socket()

    def parse_rtde_packet(self, data):
        """Parse RTDE packet to extract TCP pose."""
        try:
            # RTDE packet starts with a 2-byte length field
            if len(data) < 8:
                self.get_logger().warn('⚠️ Data too short to parse RTDE packet')
                return None

            # Parse packet length (first 2 bytes)
            packet_length = struct.unpack('!H', data[:2])[0]
            self.get_logger().info(f'RTDE packet length: {packet_length}')

            # Check if we have the full packet
            if len(data) < packet_length:
                self.get_logger().warn(f'⚠️ Incomplete RTDE packet: expected {packet_length} bytes, got {len(data)} bytes')
                return None

            # Parse message type (byte 2)
            message_type = data[2]
            self.get_logger().info(f'RTDE message type: {message_type}')

            # We are expecting a data message (type 16 for RTDE_DATA_PACKAGE)
            if message_type != 16:
                self.get_logger().warn(f'⚠️ Unexpected RTDE message type: {message_type}')
                return None

            # Parse the actual data (starting from byte 3)
            offset = 3
            # RTDE data package contains multiple variables; we need to parse until we find the TCP pose
            # For simplicity, assume TCP pose (6 doubles: x, y, z, rx, ry, rz) is near the beginning
            if len(data) < offset + 48:  # 6 doubles = 48 bytes
                self.get_logger().warn('⚠️ RTDE packet too short to contain TCP pose')
                return None

            # Parse TCP pose (6 doubles: x, y, z, rx, ry, rz)
            x, y, z, rx, ry, rz = struct.unpack('!dddddd', data[offset:offset+48])
            return x, y, z, rx, ry, rz
        except Exception as e:
            self.get_logger().error(f'❌ Failed to parse RTDE packet: {e}')
            return None

    def read_pose(self, retries=3):
        """Read and display the current pose of UR3e with retries."""
        if not self.socket:
            self.get_logger().error("❌ Socket not connected.")
            self.reconnect_socket()
            return

        for attempt in range(retries):
            try:
                self.get_logger().info(f'📍 Requesting current robot pose (attempt {attempt + 1}/{retries})...')
                
                # Clear socket buffer before sending command
                self.socket.settimeout(0.1)
                try:
                    while True:
                        data = self.socket.recv(1024)
                        self.get_logger().info(f'Cleared buffer data before request: {repr(data)}')
                except socket.timeout:
                    self.get_logger().info('Buffer cleared before sending request')

                # Send command to get pose
                self.send_urscript('get_actual_tcp_pose()\n')
                self.socket.settimeout(10.0)  # เพิ่ม timeout เป็น 10 วินาที

                # Receive data
                data = b''
                start_time = time.time()
                while time.time() - start_time < 10.0:
                    try:
                        chunk = self.socket.recv(1024)
                        if not chunk:
                            break
                        data += chunk
                    except socket.timeout:
                        break

                # Log raw data for debugging
                self.get_logger().info(f'Raw pose data (bytes): {repr(data)}')
                self.get_logger().info(f'Raw pose data (hex): {data.hex()}')
                
                # Try parsing as URScript text first
                decoded_data = data.decode('utf-8', errors='ignore')
                self.get_logger().info(f'Decoded pose data: {decoded_data}')

                match = re.search(r'\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\]', decoded_data)
                if match:
                    x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
                    rx, ry, rz = float(match.group(4)), float(match.group(5)), float(match.group(6))

                    # Convert axis-angle to quaternion
                    qx, qy, qz, qw = axis_angle_to_quaternion(rx, ry, rz)

                    # Display pose on terminal
                    self.get_logger().info(
                        f'📍 Current UR3e Pose (URScript):\n'
                        f'Position: x={x:.5f}, y={y:.5f}, z={z:.5f}\n'
                        f'Orientation (quaternion): qx={qx:.5f}, qy={qy:.5f}, qz={qz:.5f}, qw={qw:.5f}\n'
                        f'Orientation (axis-angle): rx={rx:.5f}, ry={ry:.5f}, rz={rz:.5f}'
                    )
                    return
                else:
                    self.get_logger().warn('⚠️ Could not parse as URScript, trying RTDE packet...')

                # Try parsing as RTDE packet
                pose = self.parse_rtde_packet(data)
                if pose:
                    x, y, z, rx, ry, rz = pose
                    qx, qy, qz, qw = axis_angle_to_quaternion(rx, ry, rz)
                    self.get_logger().info(
                        f'📍 Current UR3e Pose (RTDE):\n'
                        f'Position: x={x:.5f}, y={y:.5f}, z={z:.5f}\n'
                        f'Orientation (quaternion): qx={qx:.5f}, qy={qy:.5f}, qz={qz:.5f}, qw={qw:.5f}\n'
                        f'Orientation (axis-angle): rx={rx:.5f}, ry={ry:.5f}, rz={rz:.5f}'
                    )
                    return
                else:
                    self.get_logger().error(f'❌ Failed to parse pose data: {decoded_data}')
                    if attempt < retries - 1:
                        self.get_logger().warn('⚠️ Retrying...')
                        time.sleep(1.0)
                    else:
                        self.get_logger().error('❌ Failed to parse pose data after all retries.')
            except Exception as e:
                self.get_logger().error(f'❌ Failed to read pose: {e}')
                if attempt < retries - 1:
                    self.get_logger().warn('⚠️ Retrying...')
                    time.sleep(1.0)
                else:
                    self.get_logger().error('❌ Failed to read pose after all retries.')
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