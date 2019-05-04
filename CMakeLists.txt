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
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

#############
## Install ##
#############

## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  ros/scripts/topological_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

