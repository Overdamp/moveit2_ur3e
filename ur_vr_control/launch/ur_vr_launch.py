
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate a launch description for running vr_client and servoj_control nodes.
    """
    # Define package name and namespace
    package_name = 'ur_vr_control'
    namespace = 'vr_ur'

    # Define vr_client node
    vr_client_node = Node(
        package=package_name,
        executable='vr_client.py',
        name='vr_client',
        namespace=namespace,
        output='screen',
        emulate_tty=True
    )

    # Define servoj_control node
    servoj_control_node = Node(
        package=package_name,
        executable='servoj_control.py',
        name='servoj_control',
        namespace=namespace,
        output='screen',
        emulate_tty=True
    )

    # Create launch description
    return LaunchDescription([
        vr_client_node,
        servoj_control_node
    ])