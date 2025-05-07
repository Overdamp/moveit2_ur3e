#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped, Vector3
from std_msgs.msg import Bool, Header
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from utils import quat_multiply, quat_conjugate  # จาก utils.py เดิม
import numpy as np

class URVRMoveItServoControl(Node):
    """ROS 2 node to control UR3e robot using relative VR controller data with MoveIt Servo."""

    def __init__(self):
        super().__init__('ur_vr_moveit_servo_control')

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

        # Robot workspace limits and scaling
        self.robot_limits = self.config["robot_control"]["workspace_limits"]
        self.vr_workspace = {'x_min': -1.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': 1.0, 'z_min': -1.0, 'z_max': 1.0}
        self.scale_x = (self.robot_limits["x_max"] - self.robot_limits["x_min"]) / (self.vr_workspace["x_max"] - self.vr_workspace["x_min"])
        self.scale_y = (self.robot_limits["y_max"] - self.robot_limits["y_min"]) / (self.vr_workspace["y_max"] - self.vr_workspace["y_min"])
        self.scale_z = (self.robot_limits["z_max"] - self.robot_limits["z_min"]) / (self.vr_workspace["z_max"] - self.vr_workspace["z_min"])
        
        self.get_logger().info(f'🛠️ Position mapping: scale=[{self.scale_x:.3f}, {self.scale_y:.3f}, {self.scale_z:.3f}]')

        # MoveIt Servo parameters
        self.max_linear_speed = self.config["robot_control"].get("max_speed", 0.1)  # m/s
        self.max_angular_speed = self.config["robot_control"].get("max_angular_speed", 0.5)  # rad/s
        self.max_delta = 0.05  # Maximum allowed position delta per update (m)

        # Publishers and Subscribers
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.create_subscription(Pose, 'vr_controller/pose', self.pose_callback, 10)
        self.create_subscription(Bool, 'vr_controller/move_enable', self.motion_enable_callback, 10)

        # State variables
        self.motion_enabled = False
        self.vr_prev_pose = None
        self.last_motion_enable_time = 0.0
        self.last_sent_twist = None

    def validate_twist(self, linear, angular):
        """Check if twist command is within safe limits."""
        valid = (
            abs(linear.x) <= self.max_linear_speed and
            abs(linear.y) <= self.max_linear_speed and
            abs(linear.z) <= self.max_linear_speed and
            abs(angular.x) <= self.max_angular_speed and
            abs(angular.y) <= self.max_angular_speed and
            abs(angular.z) <= self.max_angular_speed
        )
        if not valid:
            self.get_logger().warn(f'⚠️ Twist command exceeds limits: linear=[{linear.x:.3f}, {linear.y:.3f}, {linear.z:.3f}], '
                                  f'angular=[{angular.x:.3f}, {angular.y:.3f}, {angular.z:.3f}]')
        return valid

    def smooth_twist(self, new_linear, new_angular):
        """Apply exponential moving average to smooth twist commands."""
        if self.last_sent_twist is None:
            self.last_sent_twist = [new_linear.x, new_linear.y, new_linear.z, new_angular.x, new_angular.y, new_angular.z]
            return new_linear, new_angular
        
        alpha = 0.1
        smoothed_linear = Vector3()
        smoothed_angular = Vector3()
        smoothed_linear.x = alpha * new_linear.x + (1 - alpha) * self.last_sent_twist[0]
        smoothed_linear.y = alpha * new_linear.y + (1 - alpha) * self.last_sent_twist[1]
        smoothed_linear.z = alpha * new_linear.z + (1 - alpha) * self.last_sent_twist[2]
        smoothed_angular.x = alpha * new_angular.x + (1 - alpha) * self.last_sent_twist[3]
        smoothed_angular.y = alpha * new_angular.y + (1 - alpha) * self.last_sent_twist[4]
        smoothed_angular.z = alpha * new_angular.z + (1 - alpha) * self.last_sent_twist[5]
        self.last_sent_twist = [smoothed_linear.x, smoothed_linear.y, smoothed_linear.z,
                               smoothed_angular.x, smoothed_angular.y, smoothed_angular.z]
        return smoothed_linear, smoothed_angular

    def pose_callback(self, msg):
        """Process relative VR controller pose and send TwistStamped to MoveIt Servo."""
        if not self.motion_enabled:
            self.get_logger().debug("🔒 Motion disabled, skipping pose processing.")
            return
        if self.vr_prev_pose is None:
            self.vr_prev_pose = msg
            self.get_logger().info("📍 Initial VR pose set for delta calculation.")
            return

        # Calculate position delta (relative movement)
        delta_x = self.scale_x * msg.position.x
        delta_y = self.scale_y * msg.position.y
        delta_z = self.scale_z * msg.position.z

        # Limit position delta for safety
        if abs(delta_x) > self.max_delta or abs(delta_y) > self.max_delta or abs(delta_z) > self.max_delta:
            self.get_logger().warn(f'⚠️ Position delta too large: dx={delta_x:.3f}, dy={delta_y:.3f}, dz={delta_z:.3f}')
            self.vr_prev_pose = msg
            return

        # Calculate orientation delta
        q_vr_current = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_vr_prev = [self.vr_prev_pose.orientation.x, self.vr_prev_pose.orientation.y,
                     self.vr_prev_pose.orientation.z, self.vr_prev_pose.orientation.w]
        
        try:
            # Compute relative rotation (delta quaternion)
            q_delta = quat_multiply(q_vr_current, quat_conjugate(q_vr_prev))
            # Convert quaternion to angular velocity (approximation)
            # For small rotations, q_delta ≈ [wx*dt/2, wy*dt/2, wz*dt/2, 1]
            dt = 0.1  # Assume 10 Hz update rate
            angular_vel = np.array([q_delta[0], q_delta[1], q_delta[2]]) * 2.0 / dt
        except Exception as e:
            self.get_logger().error(f'❌ Failed to process quaternion: {e}')
            self.vr_prev_pose = msg
            return

        # Create TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        twist_msg.twist.linear.x = delta_x / dt  # Convert to velocity (m/s)
        twist_msg.twist.linear.y = delta_y / dt
        twist_msg.twist.linear.z = delta_z / dt
        twist_msg.twist.angular.x = angular_vel[0]
        twist_msg.twist.angular.y = angular_vel[1]
        twist_msg.twist.angular.z = angular_vel[2]

        # Smooth twist command
        smoothed_linear = Vector3(x=twist_msg.twist.linear.x, y=twist_msg.twist.linear.y, z=twist_msg.twist.linear.z)
        smoothed_angular = Vector3(x=twist_msg.twist.angular.x, y=twist_msg.twist.angular.y, z=twist_msg.twist.angular.z)
        smoothed_linear, smoothed_angular = self.smooth_twist(smoothed_linear, smoothed_angular)
        twist_msg.twist.linear = smoothed_linear
        twist_msg.twist.angular = smoothed_angular

        # Validate twist command
        if not self.validate_twist(twist_msg.twist.linear, twist_msg.twist.angular):
            self.vr_prev_pose = msg
            return

        # Publish twist command
        self.twist_pub.publish(twist_msg)
        self.get_logger().info(f'🚀 Sent TwistStamped: linear=[{twist_msg.twist.linear.x:.3f}, {twist_msg.twist.linear.y:.3f}, {twist_msg.twist.linear.z:.3f}], '
                              f'angular=[{twist_msg.twist.angular.x:.3f}, {twist_msg.twist.angular.y:.3f}, {twist_msg.twist.angular.z:.3f}]')
        self.vr_prev_pose = msg

    def motion_enable_callback(self, msg):
        """Enable or disable motion based on VR controller button."""
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        if current_time - self.last_motion_enable_time < 0.2:  # Debounce
            return
        
        self.motion_enabled = msg.data
        self.last_motion_enable_time = current_time
        
        if self.motion_enabled:
            self.vr_prev_pose = None
            self.last_sent_twist = None
            self.get_logger().info('🔄 Motion enabled')
        else:
            self.get_logger().info('🔒 Motion disabled')

    def destroy_node(self):
        """Clean up resources on shutdown."""
        self.get_logger().info('❎ Shutting down URVRMoveItServoControl node')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = URVRMoveItServoControl()
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