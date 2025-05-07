#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import yaml
import time
import os
import math
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import select

class VRClient(Node):
    """ROS 2 node to connect to VR server and publish relative controller data."""
    
    def __init__(self):
        super().__init__('vr_client')

        # Load configuration from vr_config.yaml
        try:
            config_path = os.path.join(
                get_package_share_directory('ur_vr_control'),
                'config',
                'vr_config.yaml'
            )
            self.get_logger().info(f'Loading config from: {config_path}')
            
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            if self.config is None:
                raise ValueError("Failed to load vr_config.yaml: File is empty or malformed")
        
        except FileNotFoundError:
            self.get_logger().error(f"❌ vr_config.yaml not found at {config_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"❌ YAML parsing error in vr_config.yaml: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load vr_config.yaml: {e}")
            raise
        
        # VR server connection parameters
        try:
            self.vr_server_ip = self.config["vr_client"]["vr_server_ip"]
            self.vr_server_port = self.config["vr_client"]["vr_server_port"]
            self.reconnect_interval = self.config["vr_client"]["reconnect_interval"]
            self.socket_timeout = self.config["vr_client"]["socket_timeout"]
            self.get_logger().info(f'VR server config: ip={self.vr_server_ip}, port={self.vr_server_port}, timeout={self.socket_timeout}')
        except KeyError as e:
            self.get_logger().error(f"❌ Missing key in vr_config.yaml: {e}")
            raise

        # Publishers for VR controller data
        self.pose_pub = self.create_publisher(Pose, 'vr_controller/pose', 10)
        self.gripper_pub = self.create_publisher(Bool, 'vr_controller/gripper', 10)
        self.move_enable_pub = self.create_publisher(Bool, 'vr_controller/move_enable', 10)
        
        # Initialize socket
        self.socket = None
        self.connect_socket()
        
        # Timer for reading VR data
        self.timer = self.create_timer(0.01, self.read_vr_data)

        # State for relative pose
        self.initial_vr_pose = None
        self.move_enabled = False
        self.last_move_enable_time = 0.0
        self.create_subscription(
            Bool,
            'vr_controller/move_enable',
            self.move_enable_callback,
            10
        )

    def connect_socket(self):
        """Establish TCP connection to VR server."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setblocking(False)
        try:
            self.socket.connect((self.vr_server_ip, self.vr_server_port))
            self.socket.settimeout(0.5)
            self.get_logger().info(f'✅ Connected to VR server at {self.vr_server_ip}:{self.vr_server_port}')
            return True
        except socket.error as e:
            if e.errno == 115 or e.errno == 36:
                r, w, x = select.select([], [self.socket], [], self.socket_timeout)
                if w:
                    error = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                    if error == 0:
                        self.socket.settimeout(0.5)
                        self.get_logger().info(f'✅ Connected to VR server at {self.vr_server_ip}:{self.vr_server_port}')
                        return True
                    else:
                        self.get_logger().error(f'❌ Connection failed: {error}')
                        return False
                else:
                    self.get_logger().error(f'❌ Connection timed out')
                    return False
            else:
                self.get_logger().error(f'❌ Failed to connect to VR server: {e}')
                return False

    def reconnect_socket(self):
        """Attempt to reconnect to VR server if connection is lost."""
        self.get_logger().warn('⚠️ Attempting to reconnect to VR server...')
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        while rclpy.ok() and not self.connect_socket():
            time.sleep(self.reconnect_interval)

    def move_enable_callback(self, msg):
        """Handle move_enable state changes with debounce."""
        current_time = time.time()
        if current_time - self.last_move_enable_time < 0.5:  # Debounce 0.5 seconds
            return
        
        if self.move_enabled != msg.data:
            self.move_enabled = msg.data
            self.last_move_enable_time = current_time
            if self.move_enabled:
                self.get_logger().info('🔄 Move enabled, waiting for first VR pose to set initial pose')
                self.initial_vr_pose = None
            else:
                self.get_logger().info('🔄 Move disabled')
                self.initial_vr_pose = None
                # Publish zero pose when disabled
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

    def read_vr_data(self):
        """Read data from VR server and publish relative pose to ROS 2 topics."""
        if not self.socket:
            self.reconnect_socket()
            return
        
        try:
            r, _, _ = select.select([self.socket], [], [], 0.1)
            if not r:
                return
            
            data = self.socket.recv(4096).decode('utf-8', errors='ignore')
            if not data:
                self.get_logger().warn('⚠️ Connection closed by VR server')
                self.reconnect_socket()
                return
            
            self.get_logger().debug(f'Raw data: {repr(data)}')
            
            lines = data.splitlines()
            for line in lines:
                line = line.strip().replace('\r', '')
                if not line:
                    continue
                
                try:
                    values = [v.strip() for v in line.split(',')]
                    self.get_logger().debug(f'Parsed values: {values}, count={len(values)}')
                    
                    if len(values) != 9:
                        self.get_logger().warn(f'⚠️ Invalid VR data: {line}, expected 9 values, got {len(values)}')
                        continue
                    
                    # Validate float values
                    for i in range(7):
                        try:
                            float(values[i])
                        except ValueError:
                            self.get_logger().warn(f'⚠️ Invalid float value in VR data: {values[i]} at index {i}')
                            continue
                    
                    # Parse raw VR pose
                    raw_pose = Pose()
                    raw_pose.position.x = float(values[0])
                    raw_pose.position.y = float(values[1])
                    raw_pose.position.z = float(values[2])
                    raw_pose.orientation.x = float(values[3])
                    raw_pose.orientation.y = float(values[4])
                    raw_pose.orientation.z = float(values[5])
                    raw_pose.orientation.w = float(values[6])
                    
                    gripper = Bool()
                    try:
                        gripper.data = bool(int(values[7]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ Invalid gripper value: {values[7]}')
                        continue
                    
                    move_enable = Bool()
                    try:
                        move_enable.data = bool(int(values[8]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ Invalid move_enable value: {values[8]}')
                        continue
                    
                    # Handle relative pose
                    pose = Pose()
                    if move_enable.data:
                        if self.initial_vr_pose is None:
                            self.initial_vr_pose = raw_pose
                            self.get_logger().info(
                                f'Initial VR pose set: x={raw_pose.position.x:.5f}, y={raw_pose.position.y:.5f}, z={raw_pose.position.z:.5f}, '
                                f'qx={raw_pose.orientation.x:.5f}, qy={raw_pose.orientation.y:.5f}, qz={raw_pose.orientation.z:.5f}, qw={raw_pose.orientation.w:.5f}'
                            )
                            pose.position.x = 0.0
                            pose.position.y = 0.0
                            pose.position.z = 0.0
                            pose.orientation.w = 1.0
                        else:
                            pose.position.x = raw_pose.position.x - self.initial_vr_pose.position.x
                            pose.position.y = raw_pose.position.y - self.initial_vr_pose.position.y
                            pose.position.z = raw_pose.position.z - self.initial_vr_pose.position.z
                            # Compute relative quaternion
                            from quaternion_utils import quaternion_multiply, quaternion_inverse
                            q_current = [raw_pose.orientation.x, raw_pose.orientation.y, raw_pose.orientation.z, raw_pose.orientation.w]
                            q_initial = [self.initial_vr_pose.orientation.x, self.initial_vr_pose.orientation.y,
                                        self.initial_vr_pose.orientation.z, self.initial_vr_pose.orientation.w]
                            # Check for valid quaternion
                            if any(not math.isfinite(v) for v in q_current + q_initial):
                                self.get_logger().warn(f'⚠️ Invalid quaternion: q_current={q_current}, q_initial={q_initial}')
                                pose.orientation.w = 1.0
                            else:
                                q_delta = quaternion_multiply(q_current, quaternion_inverse(q_initial))
                                pose.orientation.x = q_delta[0]
                                pose.orientation.y = q_delta[1]
                                pose.orientation.z = q_delta[2]
                                pose.orientation.w = q_delta[3]
                    else:
                        pose.position.x = 0.0
                        pose.position.y = 0.0
                        pose.position.z = 0.0
                        pose.orientation.w = 1.0
                    
                    # Publish data
                    self.pose_pub.publish(pose)
                    self.gripper_pub.publish(gripper)
                    self.move_enable_pub.publish(move_enable)
                    
                    self.get_logger().info(
                        f'Published VR data: pose=[{pose.position.x:.5f}, {pose.position.y:.5f}, {pose.position.z:.5f}], '
                        f'qx={pose.orientation.x:.5f}, qy={pose.orientation.y:.5f}, qz={pose.orientation.z:.5f}, qw={pose.orientation.w:.5f}, '
                        f'gripper={gripper.data}, move_enable={move_enable.data}'
                    )
                
                except Exception as e:
                    self.get_logger().error(f'❌ Failed to parse line: {line}, error: {e}')
                    continue
        
        except Exception as e:
            self.get_logger().error(f'❌ Failed to read VR data: {e}')
            self.reconnect_socket()

    def destroy_node(self):
        """Clean up resources on shutdown."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.get_logger().info('❎ Disconnected from VR server')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VRClient()
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