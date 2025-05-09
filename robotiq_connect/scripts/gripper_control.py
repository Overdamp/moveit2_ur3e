#!/usr/bin/env python3
import os
import socket
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_share_directory
from custom_interfaces.srv import GripperControl  # Assuming custom_interfaces is the package for GripperControl.srv

class GripperControlNode(Node):
    """ROS 2 node to control Robotiq 2F-140 gripper via socket and handle service calls."""

    def __init__(self):
        super().__init__('gripper_control')
        self.get_logger().info('Initializing GripperControlNode')

        # Gripper connection parameters
        self.host = "192.168.20.35"
        self.port = 63352
        self.script_port = 30002  # For sending URScript
        self.socket_timeout = 2.0

        # Load URScript files
        package_share_directory = get_package_share_directory('robotiq_connect')
        self.script_path_activate = os.path.join(package_share_directory, 'scripts', 'gripper_activate.script')
        self.script_path_open = os.path.join(package_share_directory, 'scripts', 'gripper_open.script')
        self.script_path_close = os.path.join(package_share_directory, 'scripts', 'gripper_close.script')

        # Initialize socket
        self.socket = None
        self.script_socket = None
        self.connect_socket()

        # Service servers
        self.activate_srv = self.create_service(Empty, 'gripper/activate', self.activate_gripper_callback)
        self.open_srv = self.create_service(Empty, 'gripper/open', self.open_gripper_callback)
        self.close_srv = self.create_service(Empty, 'gripper/close', self.close_gripper_callback)
        self.control_srv = self.create_service(GripperControl, 'gripper/control', self.control_gripper_callback)

    def connect_socket(self):
        """Establish socket connections for gripper control and URScript."""
        # Control socket (port 63352)
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.socket_timeout)
            self.socket.connect((self.host, self.port))
            self.get_logger().info(f'Connected to gripper at {self.host}:{self.port}')
        except socket.error as e:
            self.get_logger().error(f'Failed to connect to gripper: {e}')
            self.socket = None

        # Script socket (port 30002)
        try:
            self.script_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.script_socket.settimeout(self.socket_timeout)
            self.script_socket.connect((self.host, self.script_port))
            self.get_logger().info(f'Connected to script socket at {self.host}:{self.script_port}')
        except socket.error as e:
            self.get_logger().error(f'Failed to connect to script socket: {e}')
            self.script_socket = None

    def reconnect_socket(self):
        """Reconnect to gripper if connection is lost."""
        self.get_logger().warn('Attempting to reconnect to gripper...')
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        if self.script_socket:
            try:
                self.script_socket.close()
            except:
                pass
        self.socket = None
        self.script_socket = None
        self.connect_socket()

    def send_script(self, script_path, additional_commands=None):
        """Send URScript to gripper via script socket."""
        if not self.script_socket:
            self.get_logger().error('Script socket not connected')
            return False

        try:
            with open(script_path, 'rb') as f:
                script_content = f.read()

            if additional_commands:
                script_content += bytes(additional_commands, 'utf-8')

            self.script_socket.send(script_content)
            time.sleep(1)  # Wait for script execution
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send script: {e}')
            self.reconnect_socket()
            return False

    def activate_gripper_callback(self, request, response):
        """Handle gripper activation service call."""
        self.get_logger().info('Activating gripper')
        success = self.send_script(self.script_path_activate)
        if success:
            self.get_logger().info('Gripper activated successfully')
        else:
            self.get_logger().error('Failed to activate gripper')
        return response

    def open_gripper_callback(self, request, response):
        """Handle gripper open service call."""
        self.get_logger().info('Opening gripper')
        success = self.send_script(self.script_path_open)
        if success:
            self.get_logger().info('Gripper opened successfully')
        else:
            self.get_logger().error('Failed to open gripper')
        return response

    def close_gripper_callback(self, request, response):
        """Handle gripper close service call."""
        self.get_logger().info('Closing gripper')
        success = self.send_script(self.script_path_close)
        if success:
            self.get_logger().info('Gripper closed successfully')
        else:
            self.get_logger().error('Failed to close gripper')
        return response

    def control_gripper_callback(self, request, response):
        """Handle custom gripper control service call."""
        self.get_logger().info(f'Controlling gripper: force={request.force}, speed={request.speed}, position={request.position}')
        force = max(0, min(255, int(request.force)))
        speed = max(0, min(255, int(request.speed)))
        position = max(0, min(255, int(request.position)))

        additional_commands = (
            f"rq_set_force({force})\r\n"
            f"rq_set_speed({speed})\r\n"
            f"rq_move_and_wait({position})\r\n"
        )

        success = self.send_script(self.script_path_close, additional_commands)
        response.success = success
        if success:
            response.message = "Gripper control executed successfully"
            self.get_logger().info(response.message)
        else:
            response.message = "Failed to execute gripper control"
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        """Clean up resources on shutdown."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        if self.script_socket:
            try:
                self.script_socket.close()
            except:
                pass
        self.get_logger().info('Disconnected from gripper')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()