cmake_minimum_required(VERSION 3.0.2)
project(omtp_lecture5)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and other necessary packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  image_transport
  cv_bridge
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_msgs image_transport cv_bridge
)

## Add your executable and link libraries
# add_executable(${PROJECT_NAME}_node src/omtp_lecture5_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

