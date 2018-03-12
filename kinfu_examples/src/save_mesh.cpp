/*
 * Written in 2018 by Riccardo Monica
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

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

#define FILENAME "mesh.ply"

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"save_mesh");

  actionlib::SimpleActionClient<kinfu_msgs::RequestAction> action_client("/kinfu_output/actions/request",true);
  ROS_INFO("Waiting for server...");
  action_client.waitForServer();

  ROS_INFO("Sending goal...");
  kinfu_msgs::RequestGoal req_goal;
  kinfu_msgs::KinfuTsdfRequest & req_publish = req_goal.request;
  req_publish.tsdf_header.request_type = req_publish.tsdf_header.REQUEST_TYPE_GET_MESH;
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

  ROS_INFO("Saving mesh to '%s'...",FILENAME);
  kinfu_msgs::RequestResultConstPtr result = action_client.getResult();
  const pcl_msgs::PolygonMesh & mesh_msg = result->mesh;
  pcl::PolygonMesh mesh;
  pcl_conversions::toPCL(mesh_msg,mesh);
  if (pcl::io::savePLYFileBinary(FILENAME,mesh))
  {
    ROS_ERROR("Mesh could not be saved.");
    return 2;
  }

  ROS_INFO("Mesh saved.");
  return 0;
}
