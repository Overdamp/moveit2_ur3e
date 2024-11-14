#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import xacro


def generate_launch_description():

    package_name = 'ur3e_robot_description'
    rviz_file_name = 'ur3e_robot_config.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')
    
    robot_description_path = os.path.join(get_package_share_directory(
                                    'ur3e_robot_description'), 
                                    'urdf','ur3e',
                                    'robot.urdf.xacro')
    
    robot_description = xacro.process_file(robot_description_path).toxml()

    robot_namespace = ''
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    {'robot_description': robot_description},
                                    {'frame_prefix': robot_namespace+'/'}
                                  ],
                                  namespace=robot_namespace,
                                  
    )
    
    joint_state_publisher = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    name='joint_state_publisher',
                                    namespace=robot_namespace
    )
    
    # mecanum_wheel_joint_state =Node(
    #         package='dynamixel_workbench_controllers',
    #         executable='dynamixel_controllers',
    #         name='dynamixel_controllers',
    #         output='screen',
    #         parameters=[{
    #             'use_sim_time': False,
    #             'dynamixel_info': '/home/Ros2_Directory/recon_ws/src/recon_bot_description/config/dynamixel_controllers.yaml',  # Full path to the config file
    #         }]
    #     )
    # joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
    #                                 executable='joint_state_publisher_gui',
    #                                 name='joint_state_publisher_gui',
    #                                 namespace=robot_namespace
    # )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    # launch_description.add_action(joint_state_publisher_gui)
    # launch_description.add_action(mecanum_wheel_joint_state)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    