#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import time
from rclpy.time import Time
from quaternion_utils import quaternion_multiply, quaternion_inverse

class VRToServoNode(Node):
    def __init__(self):
        super().__init__('vr_to_servo_node')

        # Declare parameters
        self.declare_parameter('publish_period', 0.004)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 0.5)
        self.declare_parameter('frame_id', 'tool0')
        self.declare_parameter('joint_names', [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"])

        # Get parameters
        self.publish_period = self.get_parameter('publish_period').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joint_names = self.get_parameter('joint_names').value

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, '/vr_controller/pose', self.pose_callback, 10)
        self.move_enable_sub = self.create_subscription(
            Bool, '/vr_controller/move_enable', self.move_enable_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Publishers
        self.servo_twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.servo_joint_pub = self.create_publisher(
            JointState, '/servo_node/delta_joint_cmds', 10)

        # State variables
        self.prev_position = None
        self.prev_orientation = None
        self.prev_time = None
        self.move_enable = False
        self.current_joint_state = None
        self.last_move_enable_time = 0.0
        self.prev_linear_velocity = np.zeros(3)
        self.prev_angular_velocity = np.zeros(3)
        self.filter_alpha = 0.3

        self.get_logger().info('VRToServoNode initialized')

    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.get_logger().debug(f'Received joint state: {msg.name}')

    def move_enable_callback(self, msg: Bool):
        current_time = time.time()
        if current_time - self.last_move_enable_time < 0.5:
            return
        self.move_enable = msg.data
        self.last_move_enable_time = current_time
        if not self.move_enable:
            self.prev_position = None
            self.prev_orientation = None
            self.prev_time = None
            self.get_logger().info('Movement disabled')
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
            self.servo_twist_pub.publish(twist_msg)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.joint_names
            joint_msg.velocity = [0.0] * len(self.joint_names)
            self.servo_joint_pub.publish(joint_msg)
        else:
            self.get_logger().info('Movement enabled')

    def pose_callback(self, msg: Pose):
        if not self.move_enable:
            return

        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        orientation = orientation / np.linalg.norm(orientation)

        current_time = self.get_clock().now()

        twist_msg = TwistStamped()
        joint_msg = JointState()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = self.frame_id
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = self.joint_names

        if self.prev_position is None or self.prev_orientation is None or self.prev_time is None:
            self.prev_position = position
            self.prev_orientation = orientation
            self.prev_time = current_time
            self.get_logger().info('Initialized pose state')
            return

        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().warn('Invalid time difference, skipping')
            return

        # Calculate linear and angular velocity
        delta_pos = position - self.prev_position
        linear_velocity = delta_pos / dt * self.linear_scale

        q1 = self.prev_orientation
        q2 = orientation
        q1_inv = quaternion_inverse(q1)
        q_diff = quaternion_multiply(q2, q1_inv)
        angular_velocity = np.array([q_diff[0], q_diff[1], q_diff[2]]) * 2.0 / dt * self.angular_scale

        # Apply low-pass filter
        linear_velocity = (1 - self.filter_alpha) * self.prev_linear_velocity + self.filter_alpha * linear_velocity
        angular_velocity = (1 - self.filter_alpha) * self.prev_angular_velocity + self.filter_alpha * angular_velocity
        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity

        max_linear_vel = 1.0
        max_angular_vel = 1.0
        linear_velocity = np.clip(linear_velocity, -max_linear_vel, max_linear_vel)
        angular_velocity = np.clip(angular_velocity, -max_angular_vel, max_angular_vel)

        # Publish TwistStamped
        twist_msg.twist.linear.x = float(linear_velocity[0])
        twist_msg.twist.linear.y = float(linear_velocity[1])
        twist_msg.twist.linear.z = float(linear_velocity[2])
        twist_msg.twist.angular.x = float(angular_velocity[0])
        twist_msg.twist.angular.y = float(angular_velocity[1])
        twist_msg.twist.angular.z = float(angular_velocity[2])
        self.servo_twist_pub.publish(twist_msg)
        self.get_logger().info(f'Twist published: lx={twist_msg.twist.linear.x:.3f}, ax={twist_msg.twist.angular.x:.3f}')

        # Calculate joint velocities
        if self.current_joint_state is not None and len(self.current_joint_state.name) == len(self.joint_names):
            try:
                jacobian = self.compute_jacobian(self.current_joint_state)
                twist = np.concatenate([linear_velocity, angular_velocity])
                condition_number = np.linalg.cond(jacobian)
                if condition_number > 1000:
                    self.get_logger().warn(f'Jacobian near singularity, condition number: {condition_number}')
                    self.get_logger().warn(f'Joint positions: {[self.current_joint_state.position[i] for i in range(len(self.current_joint_state.name))]}')
                    joint_msg.velocity = [0.0] * len(self.joint_names)
                else:
                    joint_velocities = np.dot(np.linalg.pinv(jacobian), twist)
                    joint_velocities = np.clip(joint_velocities, -1.0, 1.0)
                    joint_msg.velocity = joint_velocities.tolist()
                    self.servo_joint_pub.publish(joint_msg)
                    self.get_logger().info(f'Joint velocities published: {joint_msg.velocity}')
            except Exception as e:
                self.get_logger().error(f'Failed to compute joint velocities: {e}')
        else:
            self.get_logger().warn('No valid joint state available, skipping joint command')

        self.prev_position = position
        self.prev_orientation = orientation
        self.prev_time = current_time

    def compute_jacobian(self, joint_state):
        # UR3e DH parameters
        d = [0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921]
        a = [0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0]
        alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]

        theta = np.array([joint_state.position[i] for i in range(len(joint_state.name))])
        num_joints = len(theta)
        if num_joints != 6:
            self.get_logger().error(f'Invalid number of joints: expected 6, got {num_joints}')
            return np.zeros((6, 6))

        jacobian = np.zeros((6, num_joints))

        # Compute transformation matrices
        T = np.eye(4)
        transforms = []
        for i in range(num_joints):
            ct = np.cos(theta[i])
            st = np.sin(theta[i])
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ A
            transforms.append(T.copy())
            self.get_logger().debug(f'Transform {i}: {T}')

        # End-effector position
        p_ee = transforms[-1][:3, 3]
        self.get_logger().info(f'End-effector position: {p_ee}')

        # Compute Jacobian
        for i in range(num_joints):
            z_i = transforms[i][:3, 2]
            p_i = transforms[i][:3, 3]
            jacobian[:3, i] = np.cross(z_i, p_ee - p_i)
            jacobian[3:6, i] = z_i

        condition_number = np.linalg.cond(jacobian)
        self.get_logger().info(f'Jacobian condition number: {condition_number}')
        return jacobian

    def destroy_node(self):
        self.get_logger().info('Shutting down VRToServoNode')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VRToServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()