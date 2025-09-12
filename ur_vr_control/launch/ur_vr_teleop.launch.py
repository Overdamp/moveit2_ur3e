import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    ur_type = LaunchConfiguration('ur_type', default='ur3e')
    launch_servo = LaunchConfiguration('launch_servo', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    # Define launch arguments
    declare_ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3e',
        description='Type/series of used UR robot.'
    )

    declare_launch_servo_arg = DeclareLaunchArgument(
        'launch_servo',
        default_value='true',
        description='Launch servo node.'
    )

    declare_launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz.'
    )

    # Path to the ur_moveit.launch.py
    ur_moveit_launch = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    # Launch ur_moveit.launch.py (first instance)
    moveit_launch = ExecuteProcess(
        cmd=['ros2', 'launch', ur_moveit_launch,
             'ur_type:={}'.format(ur_type),
             'launch_servo:={}'.format(launch_servo),
             'launch_rviz:={}'.format(launch_rviz)],
        output='screen'
    )

    # Switch controllers: activate forward_position_controller, deactivate scaled_joint_trajectory_controller
    switch_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers',
             '--activate', 'forward_position_controller',
             '--deactivate', 'scaled_joint_trajectory_controller'],
        output='screen'
    )

    # Call service /servo_node/start_servo
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo',
             'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # Run urvr_moveit_control.py node
    urvr_control_node = Node(
        package='ur_vr_control',
        executable='urvr_moveit_control.py',
        name='urvr_moveit_control',
        output='screen'
    )

    return LaunchDescription([
        declare_ur_type_arg,
        declare_launch_servo_arg,
        declare_launch_rviz_arg,
        moveit_launch,
        switch_controllers,
        start_servo_service,
        urvr_control_node
    ])