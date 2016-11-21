cmake_minimum_required(VERSION 2.8.3)
project(kinfu_tf_feeder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kinfu_msgs
  roscpp
  tf
  eigen_conversions
  tf_conversions
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs kinfu_msgs roscpp tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinfu_tf_feeder
  src/kinfu_tf_feeder.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(kinfu_tf_feeder
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS kinfu_tf_feeder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

