cmake_minimum_required(VERSION 2.8.3)
project(kinfu_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  pcl_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
    FILES
    KinfuCommand.msg
    KinfuRequestHeader.msg
    KinfuCloudPoint.msg
    KinfuTsdfRequest.msg
    KinfuPose.msg
    KinfuCameraIntrinsics.msg
  )

add_action_files(
  DIRECTORY
    action
  FILES
    Request.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  pcl_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs
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

