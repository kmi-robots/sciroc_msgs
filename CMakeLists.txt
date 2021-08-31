cmake_minimum_required(VERSION 3.0.2)
project(sciroc_msgs)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
    actionlib
    message_generation
    sensor_msgs
    moveit_msgs
    )

## Add actions
add_action_files(DIRECTORY action FILES Perception.action)
add_service_files(FILES DetectGrasps.srv) 

## Generate messages
generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs moveit_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## Install scripts and executables
install(PROGRAMS
  scripts/fibonacci_client.py
  scripts/fibonacci_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

