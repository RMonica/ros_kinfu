/*
 * Written in 2021 by Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * To the extent possible under law, the author(s) have dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along
 * with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

#include <kinfu_msgs/RequestAction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <cstdio>

#define FILENAME "tsdf_volume.pcd"

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"save_tsdf_volume");

  typedef pcl::PointCloud<pcl::PointXYZI> TSDFCloud;

  actionlib::SimpleActionClient<kinfu_msgs::RequestAction> action_client("/kinfu_output/actions/request",true);
  ROS_INFO("Waiting for server...");
  action_client.waitForServer();

  ROS_INFO("Sending goal...");
  kinfu_msgs::RequestGoal req_goal;
  kinfu_msgs::KinfuTsdfRequest & req_publish = req_goal.request;
  req_publish.tsdf_header.request_type = req_publish.tsdf_header.REQUEST_TYPE_GET_TSDF;
  req_publish.request_remove_duplicates = true;
  action_client.sendGoal(req_goal);

  ROS_INFO("Waiting for result...");
  action_client.waitForResult(ros::Duration(60.0));
  actionlib::SimpleClientGoalState state = action_client.getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Action did not succeed.");
    return 1;
  }
  ROS_INFO("Action succeeded.");

  TSDFCloud cloud;
  kinfu_msgs::RequestResultConstPtr result = action_client.getResult();
  // check if the point cloud was too large, and it was saved to file
  if (!result->file_name.empty())
  {
    ROS_INFO("Loading point cloud from file '%s'...",result->file_name.c_str());
    pcl::io::loadPCDFile(result->file_name, cloud);
    std::remove(result->file_name.c_str()); // cleanup
  }
  else
  {
    // else, get the point cloud from the message
    ROS_INFO("Extracting point cloud from message...");
    sensor_msgs::PointCloud2 cloud_msg = result->pointcloud;
    pcl::fromROSMsg(cloud_msg, cloud);
  }

  ROS_INFO("Saving cloud to '%s'...",FILENAME);
  if (pcl::io::savePCDFileBinary(FILENAME, cloud))
  {
    ROS_ERROR("TSDF volume could not be saved.");
    return 2;
  }

  ROS_INFO("TSDF volume saved.");
  return 0;
}
