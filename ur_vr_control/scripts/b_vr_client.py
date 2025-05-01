#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class VRClient(Node):
    def __init__(self):
        super().__init__('vr_client_node')

        # Server configuration
        self.server_ip = '10.9.164.219'
        self.server_port = 5555

        # Create TCP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.client_socket.connect((self.server_ip, self.server_port))
            self.get_logger().info(f'✅ Connected to server at {self.server_ip}:{self.server_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to server: {e}')
            rclpy.shutdown()
            return

        # Create publishers for pose and gripper
        self.pose_pub = self.create_publisher(Pose, 'vr_controller/pose', 10)
        self.gripper_pub = self.create_publisher(Bool, 'vr_controller/gripper', 10)

        # Start reading data
        self.run_client()

    def run_client(self):
        buffer = ""

        while rclpy.ok():
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.get_logger().warn('⚠️ No data received. Server may have closed the connection.')
                    break

                buffer += data.decode()

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    if not line:
                        continue

                    # Validate data format (expecting 9 comma-separated values)
                    values = line.split(',')
                    if len(values) != 9:
                        self.get_logger().warn(f'⚠️ Incomplete or invalid data received: {line}')
                        continue

                    self.get_logger().info(f'📨 Received: {line}')
                    print(line)

                    # Process pose data (x, y, z, qx, qy, qz, qw)
                    try:
                        x, y, z, qx, qy, qz, qw = map(float, values[:7])
                        pose_msg = Pose()
                        pose_msg.position.x = x
                        pose_msg.position.y = y
                        pose_msg.position.z = z
                        pose_msg.orientation.x = qx
                        pose_msg.orientation.y = qy
                        pose_msg.orientation.z = qz
                        pose_msg.orientation.w = qw
                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info(f'📤 Published pose: {pose_msg}')

                    except ValueError as e:
                        self.get_logger().warn(f'⚠️ Error parsing pose values: {e}')
                        continue

                    # Process gripper state (last value in the data, should be either 0 or 1)
                    try:
                        gripper_state = bool(int(values[8]))
                        gripper_msg = Bool()
                        gripper_msg.data = gripper_state
                        self.gripper_pub.publish(gripper_msg)
                        self.get_logger().info(f'📤 Published gripper state: {gripper_msg.data}')

                    except ValueError as e:
                        self.get_logger().warn(f'⚠️ Error parsing gripper state: {e}')
                        continue

            except Exception as e:
                self.get_logger().error(f'❌ Error while receiving data: {e}')
                break

        self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    vr_client = VRClient()
    vr_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
