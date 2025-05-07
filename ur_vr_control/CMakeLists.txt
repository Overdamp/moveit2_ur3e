cmake_minimum_required(VERSION 3.8)
project(ur_vr_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/vr_client.py
  scripts/servoj_control.py
  scripts/speedl_control.py
  scripts/quaternion_utils.py

  scripts/b_vr_client.py
  scripts/b_servoj_control.py
  scripts/b_speedl_control.py
  scripts/b_quaternion_utils.py

  scripts/read_ur3e_pose.py
  scripts/urvr_moveit_control.py
DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()