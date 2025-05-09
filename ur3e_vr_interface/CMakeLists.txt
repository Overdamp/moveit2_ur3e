cmake_minimum_required(VERSION 3.8)
project(ur3e_vr_interface)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PosRot.msg"
  "srv/GripperControl.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
 )

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS

DESTINATION lib/${PROJECT_NAME}
)


################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  msg
  srv
  DESTINATION share/${PROJECT_NAME})

ament_package()

