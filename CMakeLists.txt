cmake_minimum_required(VERSION 3.0.2)
project(amrl_consensus)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c17>
  -pthread
  -O3
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  tf2_ros
  std_msgs
  geometry_msgs
  amrl_logging
  amrl_common
)

## Specify additional locations of header files
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################


# roslaunch_add_file_check(ros_launch)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amrl_consensus
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  py_plotting/pose_plots.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#########################
## Project directories ##
#########################

add_subdirectory(libs)  
add_subdirectory(nodes)

