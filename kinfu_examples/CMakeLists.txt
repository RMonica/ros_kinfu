cmake_minimum_required(VERSION 2.8.3)
project(kinfu_examples)

find_package(catkin REQUIRED COMPONENTS
  kinfu_msgs
  roscpp
  pcl_conversions
  actionlib
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinfu_examples
#  CATKIN_DEPENDS kinfu_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(save_mesh src/save_mesh.cpp)

add_dependencies(save_mesh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(save_mesh
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

