#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from quaternion_utils import quaternion_to_axis_angle


class URVRServoControl(Node):
    def __init__(self):
        super().__init__('ur_vr_servoj_control')

        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.20.35')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # Setup TCP connection to robot
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.robot_ip, 30002))
            self.get_logger().info(f"✅ Connected to UR3e at {self.robot_ip}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not connect to UR3e: {e}")
            rclpy.shutdown()
            return

        # Subscribe to VR controller pose topic
        self.create_subscription(
            Pose,
            'vr_controller/pose',
            self.pose_callback,
            10
        )

        # Subscribe to VR controller gripper control topic
        self.create_subscription(
            Bool,
            'vr_controller/gripper',
            self.gripper_callback,
            10
        )

    def pose_callback(self, msg):
        """Callback for VR controller pose"""
        x, y, z = msg.position.x, msg.position.y, msg.position.z

        rx, ry, rz = quaternion_to_axis_angle(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )

        # URScript servoj command (ปรับ speed เหลือ 20% ของ max)
        servoj_cmd = f'servoj(p[{x},{y},{z},{rx},{ry},{rz}], 1.2, 0.05, 0.008)\n'
        self.send_urscript(servoj_cmd)

    def gripper_callback(self, msg):
        """Callback for VR controller gripper control"""
        if msg.data:
            script = 'set_digital_out(0, True)\n'  # สมมุติ digital out 0 คือ gripper on
        else:
            script = 'set_digital_out(0, False)\n'

        self.send_urscript(script)

    def send_urscript(self, script):
        """Send URScript command to robot controller"""
        try:
            self.socket.send(script.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send URScript: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.socket.close()
        self.get_logger().info("❎ Disconnected from UR3e")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = URVRServoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
