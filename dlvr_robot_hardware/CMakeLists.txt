cmake_minimum_required(VERSION 3.0.2)
project(dlvr_robot_hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  nav_msgs
  dlvr_robot_msg # add this when depend on other package
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dlvr_robot_hardware
  CATKIN_DEPENDS roscpp rospy dlvr_robot_msg # add this when depend on other package
  #  DEPENDS system_lib
  )

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

add_executable(twist_to_wheel src/twist_to_wheel.cpp)
target_link_libraries(twist_to_wheel ${catkin_LIBRARIES})

add_executable(twist_to_wheel_sim src/twist_to_wheel_sim.cpp)
target_link_libraries(twist_to_wheel_sim ${catkin_LIBRARIES})

add_executable(wheel_to_odom src/wheel_to_odom.cpp)
target_link_libraries(wheel_to_odom ${catkin_LIBRARIES})
add_dependencies(wheel_to_odom dlvr_robot_msg_generate_messages_cpp) # add this when depend on other package