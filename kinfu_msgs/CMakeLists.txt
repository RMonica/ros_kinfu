cmake_minimum_required(VERSION 2.8.3)
project(kinfu_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
    FILES
    KinfuCloudPoint.msg
    KinfuCommand.msg
    KinfuMeshTriangle.msg
    KinfuRequestHeader.msg
    KinfuTsdfPoint.msg
    KinfuTsdfRequest.msg
    KinfuTsdfResponse.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############

