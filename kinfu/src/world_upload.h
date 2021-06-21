#ifndef WORLD_UPLOAD_H
#define WORLD_UPLOAD_H

// STL
#include <iostream>
#include <vector>
#include <list>
#include <stdint.h>
#include <memory>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// ROS custom messages
#include <kinfu_msgs/WorldUploadAction.h>

class WorldUploadManager
{
  public:
  typedef actionlib::SimpleActionServer<kinfu_msgs::WorldUploadAction> ActionServer;
  typedef std::shared_ptr<ActionServer> ActionServerPtr;
  typedef pcl::PointCloud<pcl::PointXYZI> TSDFCloud;
  typedef boost::shared_ptr<TSDFCloud> TSDFCloudPtr;
  typedef std::vector<short> ShortVector;

  WorldUploadManager(ros::NodeHandle & nh, boost::mutex &mutex, boost::condition_variable &cond);

  void SetInitialTransformation(const Eigen::Affine3f & initial_transformation)
    {m_initial_transformation = initial_transformation; }

  void onAction(const kinfu_msgs::WorldUploadGoalConstPtr & goal);

  bool IsActionWaiting() {return m_is_action_waiting; }

  void Execute(pcl::gpu::kinfuLS::KinfuTracker * kinfu);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  bool ParseKnownVoxelgrid(const std_msgs::UInt8MultiArray & voxelgrid,
                           Eigen::Vector3i & size,
                           ShortVector & weights);

  ros::NodeHandle & m_nh;

  ActionServerPtr m_action_server;

  Eigen::Affine3f m_initial_transformation;

  bool m_is_action_waiting;
  bool m_is_action_done;
  bool m_is_action_succeeded;
  kinfu_msgs::WorldUploadGoalConstPtr m_current_goal;

  boost::mutex & m_shared_mutex;
  boost::condition_variable & m_shared_cond;
};

#endif // WORLD_UPLOAD_H
