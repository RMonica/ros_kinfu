cmake_minimum_required(VERSION 2.8.3)
project(kinfu_output)

find_package(catkin REQUIRED COMPONENTS
  kinfu_msgs
  roscpp
  pcl_conversions
  sensor_msgs
  pcl_msgs
)

find_package(PCL REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS kinfu_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(kinfu_output
  src/kinfu_output.cpp
  )

target_link_libraries(kinfu_output
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

#############
## Testing ##
#############

