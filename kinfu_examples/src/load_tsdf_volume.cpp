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

#include <kinfu_msgs/WorldUploadAction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <cstdio>
#include <stdint.h>

#define FILENAME           "tsdf_volume.pcd"
#define VOXELGRID_FILENAME "voxelgrid.dat"

typedef int8_t int8;
typedef std::vector<int8> Int8Vector;

typedef uint64_t uint64;

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"load_tsdf_volume");

  typedef pcl::PointCloud<pcl::PointXYZI> TSDFCloud;

  TSDFCloud cloud;

  ROS_INFO("Loading file %s.", FILENAME);
  if (pcl::io::loadPCDFile(FILENAME, cloud))
  {
    ROS_FATAL("Could not load file %s", FILENAME);
    return 1;
  }

  actionlib::SimpleActionClient<kinfu_msgs::WorldUploadAction> action_client("/kinfu/world_upload",true);
  ROS_INFO("Waiting for server...");
  action_client.waitForServer();

  ROS_INFO("Sending goal...");
  kinfu_msgs::WorldUploadGoal req_goal;
  req_goal.reset = false;                // kinfu must clear TSDF volume before loading
  req_goal.bbox_min.x = -1.0;
  req_goal.bbox_min.y = -1.0;
  req_goal.bbox_min.z = 0.0;
  req_goal.bbox_max.x = 1.0;
  req_goal.bbox_max.y = 1.0;
  req_goal.bbox_max.z = 4.0;
  pcl::toROSMsg(cloud, req_goal.tsdf_volume);

  {
    Int8Vector voxelgrid;
    uint64 sizes[3];

    std::ifstream ifile(VOXELGRID_FILENAME);
    ifile.read((char *)(sizes), sizeof(sizes));
    if (ifile)
    {
      ROS_INFO("Voxelgrid has size %u %u %u", unsigned(sizes[0]), unsigned(sizes[1]), unsigned(sizes[2]));

      const uint64 total_size = sizes[0] * sizes[1] * sizes[2];
      voxelgrid.resize(total_size, 0);
      ifile.read((char *)(voxelgrid.data()), total_size);

      std_msgs::MultiArrayDimension dim;
      dim.label = "x";
      dim.size = sizes[0];
      dim.stride = 1;
      req_goal.known_voxelgrid.layout.dim.push_back(dim);

      dim.label = "y";
      dim.size = sizes[1];
      dim.stride = sizes[0];
      req_goal.known_voxelgrid.layout.dim.push_back(dim);

      dim.label = "z";
      dim.size = sizes[2];
      dim.stride = sizes[1] * sizes[0];
      req_goal.known_voxelgrid.layout.dim.push_back(dim);

      req_goal.known_voxelgrid.data.resize(total_size, 0);
      for (uint64 i = 0; i < total_size; i++)
        if (voxelgrid[i] != 0)
          req_goal.known_voxelgrid.data[i] = 1;
    }
  }

  action_client.sendGoal(req_goal);

  ROS_INFO("Waiting for result...");
  action_client.waitForResult(ros::Duration(60.0));
  actionlib::SimpleClientGoalState state = action_client.getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Action did not succeed.");
    return 1;
  }
  kinfu_msgs::WorldUploadResult result = *action_client.getResult();

  ROS_INFO("Action succeeded: %s", (result.ok ? "OK" : "NOT OK"));

  return 0;
}
