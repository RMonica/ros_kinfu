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
#include <actionlib/client/simple_action_client.h>

#define FILENAME "voxelgrid.dat"

#include <stdint.h>
#include <vector>
#include <fstream>

typedef unsigned int uint;
typedef uint64_t uint64;

typedef int8_t int8;
typedef std::vector<int8> Int8Vector;

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"save_mesh");

  kinfu_msgs::RequestGoal req_goal;
  kinfu_msgs::KinfuTsdfRequest & req_publish = req_goal.request;
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::SUCCEEDED;

  req_publish.request_bounding_box = true;
  req_publish.bounding_box_min.x = -1.0;
  req_publish.bounding_box_min.y = -1.0;
  req_publish.bounding_box_min.z = 0.0;
  req_publish.bounding_box_max.x = 1.0;
  req_publish.bounding_box_max.y = 1.0;
  req_publish.bounding_box_max.z = 4.0;

  actionlib::SimpleActionClient<kinfu_msgs::RequestAction> action_client("/kinfu_output/actions/request",true);
  ROS_INFO("Waiting for server...");
  action_client.waitForServer();

  ROS_INFO("Sending goal...");
  req_publish.tsdf_header.request_type = req_publish.tsdf_header.REQUEST_TYPE_GET_VOXELGRID;
  action_client.sendGoal(req_goal);
  ROS_INFO("Waiting for result...");
  action_client.waitForResult(ros::Duration(60.0));
  state = action_client.getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Action did not succeed.");
    return 1;
  }
  ROS_INFO("Action succeeded.");

  kinfu_msgs::RequestResultConstPtr result = action_client.getResult();
  std_msgs::Float32MultiArray array_msg = result->float_values;

  uint64 sizes[3];
  for (uint i = 0; i < 3; i++)
    sizes[i] = array_msg.layout.dim[i].size;
  uint64 steps[3] = {1,sizes[0],sizes[1] * sizes[0]};
  const uint64 size = sizes[0] * sizes[1] * sizes[2];

  // message too large
  if (!result->file_name.empty())
  {
    ROS_INFO("Kinfu message too large, loading file %s.", result->file_name.c_str());
    std::ifstream ifile(result->file_name.c_str(), std::ios::binary);

    array_msg.data.resize(size);

    ifile.read((char *)(array_msg.data.data()), size * sizeof(float));
    if (!ifile)
    {
      ROS_ERROR("Could not load file.");
      return 1;
    }
    ifile.close();
    std::remove(result->file_name.c_str());
  }

  ROS_INFO_STREAM("Grid has size: " << size << " (" << sizes[0] << " " << sizes[1] << " " << sizes[2] << ")");

  Int8Vector grid(size);
  for (uint64 i = 0; i < size; i++)
    grid[i] = (array_msg.data[i] > 0.5) ? 1 : ((array_msg.data[i] < -0.5) ? -1 : 0);

  uint64 occupied_count = 0;
  uint64 empty_count = 0;
  uint64 unknown_count = 0;
  for (uint64 x = 0; x < sizes[0]; x++)
    for (uint64 y = 0; y < sizes[1]; y++)
      for (uint64 z = 0; z < sizes[2]; z++)
      {
        uint64 i = x * steps[0] + y * steps[1] + z * steps[2];
        if (grid[i] == 0)
          unknown_count++;
        else if (grid[i] == 1)
          occupied_count++;
        else
          empty_count++;
      }
  ROS_INFO("\nThere are %d occupied"
           "\n          %d empty"
           "\n      and %d unknown voxels.",int(occupied_count),int(empty_count),int(unknown_count));

  ROS_INFO("Saving grid to '%s'...",FILENAME);
  std::ofstream ofile(FILENAME);
  ofile.write((char *)(sizes),sizeof(uint64) * 3);
  ofile.write((char *)(grid.data()),sizeof(int8) * grid.size());
  if (!ofile)
  {
    ROS_ERROR("Grid could not be saved.");
    return 2;
  }

  ROS_INFO("Grid saved.");

  return 0;
}
