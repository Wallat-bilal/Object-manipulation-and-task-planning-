cmake_minimum_required(VERSION 3.0.2)
project(omtp_lecture4)
cmake_minimum_required(VERSION 3.0.2)
project(omtp_lecture4)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ToolControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

