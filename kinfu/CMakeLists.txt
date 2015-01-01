cmake_minimum_required(VERSION 2.8.3)
project(kinfu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  kinfu_msgs
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
)

include(cmake/CustomPCLConfig.cmake)
message(STATUS "Looking for PCL configuration file at " ${PCL_DIR})
find_package(PCL CONFIG REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)

if (CUDA_FOUND) 
    message(" * CUDA ${CUDA_VERSION} was found")
    include_directories(${CUDA_INCLUDE_DIRS})
else(CUDA_FOUND)
    message(" * CUDA is not found")
    message(FATAL_ERROR "Not all CUDA libraries are found")
endif(CUDA_FOUND)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs image_transport kinfu_msgs message_filters roscpp sensor_msgs std_msgs tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinfu
  src/commandsubscriber.cpp
  src/kinfuLS.cpp
  src/weightcubelistener.cpp
  src/worlddownloadmanager.cpp
  )

add_dependencies(kinfu kinfu_msgs_generate_messages_cpp)

target_link_libraries(kinfu
  boost_system boost_signals
  ${CUDA_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

