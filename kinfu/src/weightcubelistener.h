/*
 * Copyright (c) 2013-2015, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WEIGHTCUBELISTENER_H
#define WEIGHTCUBELISTENER_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// STL
#include <deque>
#include <stdint.h>
#include <vector>

// ROS
#include <ros/ros.h>

// custom
#include "kinfu/bitmaskoctree.h"

class TWeightCubeListener: public pcl::gpu::kinfuLS::CyclicalBuffer::WeightCubeListener
  {
  public:
  typedef pcl::gpu::kinfuLS::CyclicalBuffer::WeightCubeListener Listener;
  typedef pcl::BitmaskOctree<3> OccupancyOctree;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::PointNormal> PointXYZNormalCloud;
  typedef boost::shared_ptr<TWeightCubeListener> Ptr;
  typedef unsigned int uint;
  typedef uint64_t uint64;
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > Vector3iVector;

  TWeightCubeListener();
  ~TWeightCubeListener();

  void onNewCube(const Listener::WeightVectorPtr weights,const Eigen::Vector3f& cube_size,const Eigen::Vector3i& nb_voxels,
    const Eigen::Vector3i& cyclical_shifted_origin,const Eigen::Vector3i& grid_origin);

  void onClearSphere(const Eigen::Vector3f & center,float radius,
                     const bool set_to_known,PointXYZNormalCloud::Ptr cleared_frontier);
  void onClearBBox(const Eigen::Vector3f & min,const Eigen::Vector3f & max,
                   const bool set_to_known,PointXYZNormalCloud::Ptr cleared_frontier);

  void onReset();

  bool retrieveOldCube(WeightVectorPtr& weights, const Eigen::Vector3f& cube_size,
    const Eigen::Vector3i& nb_voxels, const Eigen::Vector3i& cyclical_shifted_origin, const Eigen::Vector3i& grid_global_origin);

  struct NewCubeInfo
    {
    Listener::WeightVectorPtr weights;
    Eigen::Vector3f cube_size;
    Eigen::Vector3i nb_voxels;
    Eigen::Vector3i cyclical_shifted_origin;
    Eigen::Vector3i grid_global_origin;

    enum Task
      {
      TASK_NOOP          = 0,
      TASK_WRITE_CUBE    = 1,
      TASK_RETRIEVE_CUBE = 2,
      MAX_TASKS
      };

    Task task;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef boost::shared_ptr<NewCubeInfo> Ptr;
    };

  void CubeWorker();
  void NewCubeWorker(NewCubeInfo::Ptr new_info);
  void RetrieveCubeWorker(NewCubeInfo::Ptr new_info);

  PointCloud::Ptr GetOccupiedVoxelCenters();
  uint64 CountOccupiedVoxelsInSphere(const Eigen::Vector3f & center,float radius,bool inverse = false);
  uint64 CountOccupiedVoxelsInBBox(const Eigen::Vector3f & bbox_min, const Eigen::Vector3f &bbox_max, bool inverse = false);
  uint64 CountOccupiedVoxels() {return m_octree.GetPointCount(); }

  private:
  OccupancyOctree m_octree;
  float m_voxel_size;

  bool m_is_terminating;

  boost::thread * m_cube_thread;
  std::deque<NewCubeInfo::Ptr> m_cube_queue; // cubes to be transferred into the octree
  boost::condition_variable m_cube_cond;
  boost::mutex m_cube_mutex;
  uint m_currently_working_cubes; // counts the number of cubes that have been removed from the queue
                                  // but are still under processing.

  WeightVectorPtr m_retrieved_weights;
  uint m_queued_retrieving_cubes; // increased every time a retrieving action is added to the queue, so
                                  // we can return only the last requested cube
  NewCubeInfo::Ptr m_last_requested_cube;
  };

#endif // WEIGHTCUBELISTENER_H
