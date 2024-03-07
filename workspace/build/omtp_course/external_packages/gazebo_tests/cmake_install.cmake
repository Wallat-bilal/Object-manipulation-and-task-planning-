# Install script for directory: /home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/src/omtp_course/external_packages/gazebo_tests

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gazebo_tests" TYPE FILE FILES "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/devel/include/gazebo_tests/ModelSpawnConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/gazebo_tests" TYPE FILE FILES "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/devel/lib/python3/dist-packages/gazebo_tests/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/devel/lib/python3/dist-packages/gazebo_tests/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/gazebo_tests" TYPE DIRECTORY FILES "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/devel/lib/python3/dist-packages/gazebo_tests/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/build/omtp_course/external_packages/gazebo_tests/catkin_generated/installspace/gazebo_tests.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gazebo_tests/cmake" TYPE FILE FILES
    "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/build/omtp_course/external_packages/gazebo_tests/catkin_generated/installspace/gazebo_testsConfig.cmake"
    "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/build/omtp_course/external_packages/gazebo_tests/catkin_generated/installspace/gazebo_testsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gazebo_tests" TYPE FILE FILES "/home/wallat/Desktop/Object-manipulation-and-task-planning-/workspace/src/omtp_course/external_packages/gazebo_tests/package.xml")
endif()

