#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from b_quaternion_utils import quaternion_to_axis_angle

class URVRServoControl(Node):
    def __init__(self):
        super().__init__('ur_vr_servoj_control')

        self.declare_parameter('robot_ip', '192.168.20.35')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.robot_ip, 30002))
            self.get_logger().info(f"✅ Connected to UR3e at {self.robot_ip}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not connect to UR3e: {e}")
            rclpy.shutdown()
            return

        self.current_tcp_pose = None
        self.vr_reference_pose = None
        self.tcp_reference_pose = None
        self.motion_enabled = False
        self.vr_pose = None

        self.create_subscription(Pose, 'vr_controller/pose', self.vr_pose_callback, 10)
        self.create_subscription(Bool, 'vr_controller/gripper', self.gripper_callback, 10)
        self.create_subscription(Bool, 'vr_controller/move_enable', self.move_enable_callback, 10)
        self.create_subscription(PoseStamped, 'tcp_broadcast/pose', self.tcp_callback, 10)

    def tcp_callback(self, msg):
        self.current_tcp_pose = msg.pose

    def move_enable_callback(self, msg):
        self.motion_enabled = msg.data
        if self.motion_enabled and self.current_tcp_pose and self.vr_pose:
            self.vr_reference_pose = self.vr_pose
            self.tcp_reference_pose = self.current_tcp_pose
            self.get_logger().info("🔄 Reference poses set.")

    def vr_pose_callback(self, msg):
        self.vr_pose = msg
        if not self.motion_enabled or not self.vr_reference_pose or not self.tcp_reference_pose:
            return

        dx = msg.position.x - self.vr_reference_pose.position.x
        dy = msg.position.y - self.vr_reference_pose.position.y
        dz = msg.position.z - self.vr_reference_pose.position.z

        x = self.tcp_reference_pose.position.x + dx
        y = self.tcp_reference_pose.position.y + dy
        z = self.tcp_reference_pose.position.z + dz

        rx, ry, rz = quaternion_to_axis_angle(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )

        servoj_cmd = f'servoj([{x:.5f},{y:.5f},{z:.5f},{rx:.5f},{ry:.5f},{rz:.5f}], a=1.2, v=0.2, t=0.008, lookahead_time=0.1, gain=300)\n'
        self.send_urscript(servoj_cmd)

    def gripper_callback(self, msg):
        script = 'set_digital_out(0, True)\n' if msg.data else 'set_digital_out(0, False)\n'
        self.send_urscript(script)

    def send_urscript(self, script):
        try:
            self.socket.send(script.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send URScript: {e}")

    def destroy_node(self):
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
