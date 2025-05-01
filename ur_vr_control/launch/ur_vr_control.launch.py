from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    control_mode = LaunchConfiguration('control_mode')
    server_ip = LaunchConfiguration('server_ip')
    server_port = LaunchConfiguration('server_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='speedl',
            description='Control mode: speedl or servoj'
        ),

        DeclareLaunchArgument(
            'server_ip',
            default_value='10.9.164.219',
            description='IP address of the Unity VR server'
        ),

        DeclareLaunchArgument(
            'server_port',
            default_value='5555',
            description='Port number of the Unity VR server'
        ),

        Node(
            package='ur_vr_control',
            executable='speedl_control',
            name='speedl_control',
            condition=IfCondition(
                PythonExpression(["'", control_mode, "' == 'speedl'"])
            ),
            output='screen'
        ),

        Node(
            package='ur_vr_control',
            executable='servoj_control',
            name='servoj_control',
            condition=IfCondition(
                PythonExpression(["'", control_mode, "' == 'servoj'"])
            ),
            output='screen'
        ),

        Node(
            package='ur_vr_control',
            executable='vr_client',
            name='vr_client',
            output='screen',
            parameters=[
                {'server_ip': server_ip},
                {'server_port': server_port}
            ]
        ),
    ])
