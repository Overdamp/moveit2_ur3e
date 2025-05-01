#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
from quaternion_utils import quaternion_to_axis_angle

class URVRSpeedControl(Node):
    def __init__(self):
        super().__init__('ur_vr_speed_control')

        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.20.35')
        self.declare_parameter('max_speed', 0.3)  # 20% of UR3e's max speed
        self.declare_parameter('acceleration', 0.05)

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value

        # Setup TCP connection
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.robot_ip, 30002))
            self.get_logger().info(f"✅ Connected to UR3e at {self.robot_ip}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not connect to UR3e: {e}")
            rclpy.shutdown()
            return

        # Subscribe to VR controller pose topic
        self.subscription = self.create_subscription(
            Pose,
            'vr_controller/pose',
            self.listener_callback,
            10
        )

        self.last_pose = None
        self.last_time = None

    def listener_callback(self, msg):
        now = time.time()
        if self.last_pose is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt == 0:
                return

            # Linear velocities
            vx = (msg.position.x - self.last_pose.position.x) / dt
            vy = (msg.position.y - self.last_pose.position.y) / dt
            vz = (msg.position.z - self.last_pose.position.z) / dt

            # Orientation (convert quaternion to axis-angle)
            rx, ry, rz = quaternion_to_axis_angle(
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w
            )

            # URScript speedl command
            script = (
                f"speedl([{vx:.5f},{vy:.5f},{vz:.5f},{rx:.5f},{ry:.5f},{rz:.5f}], "
                f"{self.max_speed}, {self.acceleration})\n"
            )

            try:
                self.socket.send(script.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to send data: {e}")

        self.last_pose = msg
        self.last_time = now

    def destroy_node(self):
        # Close socket before shutting down node
        self.socket.close()
        self.get_logger().info("❎ Disconnected from UR3e")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = URVRSpeedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
