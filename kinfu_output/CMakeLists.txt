cmake_minimum_required(VERSION 2.8.3)
project(kinfu_output)

find_package(catkin REQUIRED COMPONENTS
  kinfu_msgs
  roscpp
  pcl_conversions
  sensor_msgs
  pcl_msgs
  actionlib
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS kinfu_msgs roscpp actionlib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(kinfu_output
  src/kinfu_output.cpp
  src/request_action_manager.cpp
  src/kinfu_output_action_conversions.cpp
  )

target_link_libraries(kinfu_output
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  )

add_dependencies(kinfu_output kinfu_msgs_generate_messages_cpp)

#############
## Install ##
#############

#############
## Testing ##
#############

