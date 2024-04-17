cmake_minimum_required(VERSION 3.0.2)
project(omtp_lecture3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs 
  message_generation  # Add this if you are using custom messages
)

# Specify the directory containing the message files
# This assumes CMakeLists.txt is in the root of omtp_lecture3
set(MSG_DIR logical_camera/msg)

# Uncomment this if you are using custom message or service files
add_message_files(
  DIRECTORY ${MSG_DIR}
  FILES
  LogicalCameraImage.msg  # Ensure this file exists in the logical_camera/msg folder
)

# Uncomment this if you are generating messages
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other dependencies your messages/services need
  geometry_msgs 
)

# Define catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omtp_lecture3
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime  # Add message_runtime if you have custom messages
#  DEPENDS system_lib
)

# Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Python scripts installation
catkin_install_python(PROGRAMS 
  scripts/transform_detected_objects.py
  # scripts/transform_object_pose.py  # Make sure your script is listed here if it needs to be installed
  scripts/tf_object_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

