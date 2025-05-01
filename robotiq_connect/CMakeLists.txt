cmake_minimum_required(VERSION 3.8)
project(robotiq_connect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS

scripts/manipulator_node.py  
scripts/gripper_activate.script
scripts/gripper_close.script
scripts/gripper_open.script

DESTINATION lib/${PROJECT_NAME}
)

# Install the 'scripts' directory
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)


################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
