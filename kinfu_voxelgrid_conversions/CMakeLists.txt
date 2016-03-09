cmake_minimum_required(VERSION 2.8.3)
project(kinfu_voxelgrid_conversions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

# hack for optional dependency
find_package(moveit_msgs QUIET)
if(${moveit_msgs_FOUND})
  add_definitions("-DWITH_MOVEIT_MSGS")
  message(STATUS "${PROJECT_NAME}: moveit_msgs extension will be built.")
else()
  message(STATUS "${PROJECT_NAME}: moveit_msgs extension will NOT be built.")
endif()

# super-hack for optional dependency
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/WithArmNavigationMsgs.cmake")
  find_path(arm_navigation_msgs_INCLUDE_DIRS_include CollisionMap.h PATH_SUFFIXES arm_navigation_msgs)
  get_filename_component(arm_navigation_msgs_INCLUDE_DIRS "${arm_navigation_msgs_INCLUDE_DIRS_include}" DIRECTORY)
  add_definitions("-DWITH_ARM_NAVIGATION_MSGS")
  message(STATUS "${PROJECT_NAME}: arm_navigation_msgs extension will be built.")
else()
  message(STATUS "${PROJECT_NAME}: arm_navigation_msgs extension will NOT be built.")
endif()

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
#  LIBRARIES kinfu_voxelgrid_conversions
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${arm_navigation_msgs_INCLUDE_DIRS}
)

add_executable(kinfu_voxelgrid_conversions src/kinfu_voxelgrid_conversions.cpp)

add_dependencies(kinfu_voxelgrid_conversions ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_msgs_EXPORTED_TARGETS}
  )

target_link_libraries(kinfu_voxelgrid_conversions
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_msgs_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

