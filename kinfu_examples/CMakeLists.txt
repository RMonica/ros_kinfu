cmake_minimum_required(VERSION 2.8.3)
project(kinfu_examples)

find_package(catkin REQUIRED COMPONENTS
  kinfu_msgs
  roscpp
  pcl_conversions
  actionlib
  eigen_conversions
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

add_executable(save_voxelgrid src/save_voxelgrid.cpp)

add_dependencies(save_voxelgrid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(save_voxelgrid
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(shift_checker_service src/shift_checker_service.cpp)

add_dependencies(shift_checker_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(shift_checker_service
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(save_tsdf_volume src/save_tsdf_volume.cpp)

add_dependencies(save_tsdf_volume ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(save_tsdf_volume
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(load_tsdf_volume src/load_tsdf_volume.cpp)

add_dependencies(load_tsdf_volume ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(load_tsdf_volume
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

