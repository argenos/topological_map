cmake_minimum_required(VERSION 2.8.3)
project(topological_map_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
)

catkin_python_setup()

## Generate services in the 'ros/srv' folder
add_service_files(
  DIRECTORY ros/srv
  FILES
  TopologicalPath.srv
  TopologicalPosition.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
)

install(PROGRAMS
  ros/scripts/generate_map
  ros/scripts/robot_pose_test
  ros/scripts/topological_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
