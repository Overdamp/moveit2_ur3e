#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import yaml
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import logging

class VRClient(Node):
    """
    ROS 2 node to receive VR controller data via TCP and publish to ROS topics.
    """
    def __init__(self):
        super().__init__('vr_client_node')
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        with open("config.yaml", "r") as f:
            self.config = yaml.safe_load(f)
        
        self.server_ip = self.config["vr_client"]["server_ip"]
        self.server_port = self.config["vr_client"]["server_port"]
        self.reconnect_interval = self.config["vr_client"]["reconnect_interval"]
        self.socket_timeout = self.config["vr_client"]["socket_timeout"]
        self.pose_alpha = self.config["smoothing"]["pose_alpha"]
        
        # Workspace limits
        self.limits = self.config["robot_control"]["workspace_limits"]
        
        # Create publishers
        self.pose_pub = self.create_publisher(Pose, 'vr_controller/pose', 10)
        self.gripper_pub = self.create_publisher(Bool, 'vr_controller/gripper', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, 'diagnostics', 10)
        
        # Initialize socket
        self.client_socket = None
        self.last_pose = None
        
        # Start diagnostics timer
        self.create_timer(self.config["diagnostics"]["publish_rate"], self.publish_diagnostics)
        
        # Start client
        self.run_client()
    
    def connect_socket(self):
        """Establish TCP connection to VR server."""
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.settimeout(self.socket_timeout)
        try:
            self.client_socket.connect((self.server_ip, self.server_port))
            self.get_logger().info(f'✅ Connected to server at {self.server_ip}:{self.server_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to server: {e}')
            return False
    
    def validate_pose(self, x, y, z):
        """Check if position is within workspace limits."""
        return (self.limits["x_min"] <= x <= self.limits["x_max"] and
                self.limits["y_min"] <= y <= self.limits["y_max"] and
                self.limits["z_min"] <= z <= self.limits["z_max"])
    
    def smooth_pose(self, new_pose):
        """Apply exponential moving average to smooth pose data."""
        if self.last_pose is None:
            self.last_pose = new_pose
            return new_pose
        
        smoothed_pose = Pose()
        alpha = self.pose_alpha
        smoothed_pose.position.x = alpha * new_pose.position.x + (1 - alpha) * self.last_pose.position.x
        smoothed_pose.position.y = alpha * new_pose.position.y + (1 - alpha) * self.last_pose.position.y
        smoothed_pose.position.z = alpha * new_pose.position.z + (1 - alpha) * self.last_pose.position.z
        smoothed_pose.orientation = new_pose.orientation  # Orientation not smoothed
        self.last_pose = smoothed_pose
        return smoothed_pose
    
    def run_client(self):
        """Main loop to receive and process VR data."""
        buffer = ""
        while rclpy.ok():
            if self.client_socket is None:
                if not self.connect_socket():
                    time.sleep(self.reconnect_interval)
                    continue
            
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.get_logger().warn('⚠️ No data received. Reconnecting...')
                    self.client_socket.close()
                    self.client_socket = None
                    continue
                
                buffer += data.decode()
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    
                    # Validate data format
                    values = line.split(',')
                    if len(values) != 9:
                        self.get_logger().warn(f'⚠️ Invalid data: {line}')
                        continue
                    
                    self.get_logger().info(f'📨 Received: {line}')
                    
                    # Process pose
                    try:
                        x, y, z, qx, qy, qz, qw = map(float, values[:7])
                        if not self.validate_pose(x, y, z):
                            self.get_logger().warn(f'⚠️ Pose out of workspace: x={x}, y={y}, z={z}')
                            continue
                        
                        pose_msg = Pose()
                        pose_msg.position.x = x
                        pose_msg.position.y = y
                        pose_msg.position.z = z
                        pose_msg.orientation.x = qx
                        pose_msg.orientation.y = qy
                        pose_msg.orientation.z = qz
                        pose_msg.orientation.w = qw
                        
                        # Smooth pose
                        pose_msg = self.smooth_pose(pose_msg)
                        
                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info(f'📤 Published pose: {pose_msg}')
                    
                    except ValueError as e:
                        self.get_logger().warn(f'⚠️ Error parsing pose: {e}')
                        continue
                    
                    # Process gripper
                    try:
                        gripper_state = bool(int(values[8]))
                        gripper_msg = Bool()
                        gripper_msg.data = gripper_state
                        self.gripper_pub.publish(gripper_msg)
                        self.get_logger().info(f'📤 Published gripper: {gripper_msg.data}')
                    
                    except ValueError as e:
                        self.get_logger().warn(f'⚠️ Error parsing gripper: {e}')
                        continue
            
            except socket.timeout:
                self.get_logger().warn('⚠️ Socket timeout. Reconnecting...')
                self.client_socket.close()
                self.client_socket = None
                continue
            except Exception as e:
                self.get_logger().error(f'❌ Error receiving data: {e}')
                self.client_socket.close()
                self.client_socket = None
                continue
    
    def publish_diagnostics(self):
        """Publish diagnostic information."""
        status = DiagnosticStatus()
        status.name = "VR Client"
        status.hardware_id = f"{self.server_ip}:{self.server_port}"
        status.level = DiagnosticStatus.OK if self.client_socket else DiagnosticStatus.ERROR
        status.message = "Connected" if self.client_socket else "Disconnected"
        status.values.append(KeyValue(key="Server IP", value=self.server_ip))
        status.values.append(KeyValue(key="Server Port", value=str(self.server_port)))
        self.diag_pub.publish(status)
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.client_socket:
            self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VRClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()