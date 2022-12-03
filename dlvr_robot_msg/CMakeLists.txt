cmake_minimum_required(VERSION 3.0.2)
project(dlvr_robot_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  )

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 motor_stat.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 )

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES dlvr_robot_msg
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  #  DEPENDS system_lib
  )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 #include
 ${catkin_INCLUDE_DIRS}
 )
