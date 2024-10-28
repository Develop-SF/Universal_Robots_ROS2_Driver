cmake_minimum_required(VERSION 3.5)
project(ur_moveit_config)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  std_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  rclcpp
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
  trajectory_msgs
  std_srvs
  control_msgs
  vision_msgs
  moveit_core
  moveit_ros_planning
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(teleop_command_publisher src/teleop_command_publisher.cpp)
ament_target_dependencies(teleop_command_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(non_servo_teleop src/non_servo_teleop.cpp)
ament_target_dependencies(non_servo_teleop ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS teleop_command_publisher non_servo_teleop
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_common.py)

ament_package()
