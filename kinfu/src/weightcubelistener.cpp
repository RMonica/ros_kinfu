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

#include "weightcubelistener.h"

template <typename T>
  T SQR(const T & t) {return t * t; }

TWeightCubeListener::TWeightCubeListener()
  {
  m_voxel_size = 1.0;
  m_currently_working_cubes = 0;
  m_is_terminating = false;
  m_queued_retrieving_cubes = 0;

  m_cube_thread = new boost::thread(&TWeightCubeListener::CubeWorker,this);
  }

TWeightCubeListener::~TWeightCubeListener()
  {
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    // signal the worker thread to terminate
    m_is_terminating = true;
    m_cube_cond.notify_all();
    }

  // join the thread
  m_cube_thread->join();
  delete m_cube_thread;
  }

void TWeightCubeListener::onNewCube(const WeightVectorPtr weights, const Eigen::Vector3f &cube_size, const Eigen::Vector3i &nb_voxels,
  const Eigen::Vector3i &cyclical_shifted_origin, const Eigen::Vector3i &grid_global_origin)
  {
  NewCubeInfo::Ptr new_info(new NewCubeInfo);
  new_info->weights = weights;
  new_info->cube_size = cube_size;
  new_info->nb_voxels = nb_voxels;
  new_info->cyclical_shifted_origin = cyclical_shifted_origin;
  new_info->grid_global_origin = grid_global_origin;
  new_info->task = NewCubeInfo::TASK_WRITE_CUBE;

  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    m_cube_queue.push_back(new_info);
    m_cube_cond.notify_all();
    }
  }

bool TWeightCubeListener::retrieveOldCube(WeightVectorPtr& weights,const Eigen::Vector3f& cube_size,
  const Eigen::Vector3i& nb_voxels,const Eigen::Vector3i& cyclical_shifted_origin,const Eigen::Vector3i& grid_global_origin)
  {
  if (m_last_requested_cube
    && m_last_requested_cube->cyclical_shifted_origin == cyclical_shifted_origin
    && m_last_requested_cube->grid_global_origin == grid_global_origin)
    {
    // extraction already in progress, do not schedule
    if (!m_queued_retrieving_cubes)
      {
      // extraction complete!!
      if (!m_retrieved_weights)
        return false; // error occurred

      weights = m_retrieved_weights;
      m_retrieved_weights = WeightVectorPtr();
      ROS_INFO("kinfu: retrieveOldCube: returned old cube.");
      return true;
      }

    return true;
    }

  NewCubeInfo::Ptr new_info(new NewCubeInfo);
  new_info->weights = weights;
  new_info->cube_size = cube_size;
  new_info->nb_voxels = nb_voxels;
  new_info->cyclical_shifted_origin = cyclical_shifted_origin;
  new_info->grid_global_origin = grid_global_origin;
  new_info->task = NewCubeInfo::TASK_RETRIEVE_CUBE;
  m_last_requested_cube = new_info;

  ROS_INFO("kinfu: retrieveOldCube: scheduled old cube.");

  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    m_cube_queue.push_back(new_info);
    m_queued_retrieving_cubes++;
    m_cube_cond.notify_all();
    }

  return true;
  }

void TWeightCubeListener::onClearSphere(const Eigen::Vector3f & center, float radius,
                                        const bool set_to_known,PointXYZNormalCloud::Ptr cleared_frontier)
  {
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return;
      }

    Eigen::Vector3i min = (center - Eigen::Vector3f::Ones() * (radius + 1.0)).cast<int>();
    Eigen::Vector3i max = (center + Eigen::Vector3f::Ones() * (radius + 1.0)).cast<int>();

    if (cleared_frontier)
      {
      ROS_INFO("kinfu: onClearSphere: building frontier...");
      Eigen::Vector3i t;
      OccupancyOctree::Cache octree_cache;
      for (t.x() = min.x(); t.x() <= max.x(); t.x()++)
        for (t.y() = min.y(); t.y() <= max.y(); t.y()++)
          for (t.z() = min.z(); t.z() <= max.z(); t.z()++)
          {
            if ((t.cast<float>() - center).squaredNorm() > SQR(radius))
              continue;

            if (m_octree.GetIntCached(t.x(),t.y(),t.z(),octree_cache) == set_to_known)
              continue;

            for (uint ax = 0; ax < 3; ax++)
              for (int dx = -1; dx <= 1; dx++)
                {
                if (!dx)
                  continue;
                const Eigen::Vector3i dt = Eigen::Vector3i::Unit(ax) * dx;
                const Eigen::Vector3i nt = t + dt;
                bool create_t = false;
                if ((nt.cast<float>() - center).squaredNorm() >= SQR(radius) &&
                    (m_octree.GetIntCached(nt.x(),nt.y(),nt.z(),octree_cache) != set_to_known))
                  {
                  if (set_to_known)
                    create_t = true;
                  else
                    {
                    pcl::PointNormal pt;
                    pt.x = nt.x(); pt.y = nt.y(); pt.z = nt.z();
                    Eigen::Vector3f normal = (nt.cast<float>() - center).normalized();
                    pt.normal_x = normal.x(); pt.normal_y = normal.y(); pt.normal_z = normal.z();
                    cleared_frontier->push_back(pt);
                    }
                  }

                if (create_t)
                  {
                  pcl::PointNormal pt;
                  pt.x = t.x(); pt.y = t.y(); pt.z = t.z();
                  Eigen::Vector3f normal = -(t.cast<float>() - center).normalized();
                  pt.normal_x = normal.x(); pt.normal_y = normal.y(); pt.normal_z = normal.z();
                  cleared_frontier->push_back(pt);
                  }
                }
          }
      ROS_INFO("kinfu: onClearSphere: frontier built.");
      }

    // clear the sphere in its bounding box only
    Eigen::Vector3i t;
    for (t.x() = min.x(); t.x() <= max.x(); t.x()++)
      for (t.y() = min.y(); t.y() <= max.y(); t.y()++)
        for (t.z() = min.z(); t.z() <= max.z(); t.z()++)
          if ((t.cast<float>() - center).squaredNorm() < SQR(radius))
            m_octree.SetInt(t.x(),t.y(),t.z(),set_to_known);

    m_cube_cond.notify_all();
    }
  }

void TWeightCubeListener::onClearBBox(const Eigen::Vector3f & m,const Eigen::Vector3f & M,
                                      const bool set_to_known,PointXYZNormalCloud::Ptr cleared_frontier)
  {
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return;
      }

    const Eigen::Vector3i min = (m + Eigen::Vector3f::Ones() * 0.5).cast<int>();
    const Eigen::Vector3i max = (M + Eigen::Vector3f::Ones() * 0.5).cast<int>();

    if (cleared_frontier)
      {
      ROS_INFO("kinfu: onClearBBox: building frontier...");
      Eigen::Vector3i t;
      OccupancyOctree::Cache octree_cache;
      for (t.x() = min.x(); t.x() <= max.x(); t.x()++)
        for (t.y() = min.y(); t.y() <= max.y(); t.y()++)
          for (t.z() = min.z(); t.z() <= max.z(); t.z()++)
            {
            if ((t.array() != min.array()).any() && (t.array() != max.array()).any())
              continue;

            if (m_octree.GetIntCached(t.x(),t.y(),t.z(),octree_cache) == set_to_known)
              continue;

            Eigen::Vector3i inormal = Eigen::Vector3i::Zero();
            for (uint i = 0; i < 3; i++)
              if (min[i] == t[i])
                inormal[i] = -1;
            for (uint i = 0; i < 3; i++)
              if (max[i] == t[i])
                inormal[i] = 1;
            Eigen::Vector3f normal = inormal.cast<float>().normalized();

            Eigen::Vector3i nt = t + inormal;
            if (m_octree.GetIntCached(nt.x(),nt.y(),nt.z(),octree_cache) != set_to_known)
              {
              pcl::PointNormal pt;
              if (set_to_known)
                { pt.x = t.x(); pt.y = t.y(); pt.z = t.z(); }
              else
                { pt.x = nt.x(); pt.y = nt.y(); pt.z = nt.z(); }
              if (set_to_known)
                normal *= -1.0;
              pt.normal_x = normal.x(); pt.normal_y = normal.y(); pt.normal_z = normal.z();
              cleared_frontier->push_back(pt);
              }
            }
      ROS_INFO("kinfu: onClearBBox: frontier built.");
      }

    // clear the bounding box
    Eigen::Vector3i t;
    for (t.x() = min.x(); t.x() <= max.x(); t.x()++)
      for (t.y() = min.y(); t.y() <= max.y(); t.y()++)
        for (t.z() = min.z(); t.z() <= max.z(); t.z()++)
          m_octree.SetInt(t.x(),t.y(),t.z(),set_to_known);

    m_cube_cond.notify_all();
    }
  }

void TWeightCubeListener::onReset()
  {
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return;
      }

    m_octree.Clear();

    m_cube_cond.notify_all();
    }
  }

TWeightCubeListener::PointCloud::Ptr TWeightCubeListener::GetOccupiedVoxelCenters()
  {
  PointCloud::Ptr result(new PointCloud);
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return result;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return result;
      }

    m_octree.GetOccupiedVoxelsCenters(*result,m_voxel_size);

    m_cube_cond.notify_all();
    }
  return result;
  }

TWeightCubeListener::uint64 TWeightCubeListener::CountOccupiedVoxelsInSphere(const Eigen::Vector3f & center,float radius,bool inverse)
  {
  const Eigen::Vector3f expanded_center = center / m_voxel_size;
  const float expanded_radius = radius / m_voxel_size;
  const Eigen::Vector3f bbox_min = expanded_center - Eigen::Vector3f::Ones() * expanded_radius;
  const Eigen::Vector3f bbox_max = expanded_center + Eigen::Vector3f::Ones() * expanded_radius;
  const Eigen::Vector3i bbox_min_i = (bbox_min - Eigen::Vector3f::Ones() * 0.5).cast<int>();
  const Eigen::Vector3i bbox_max_i = (bbox_max + Eigen::Vector3f::Ones() * 0.5).cast<int>();
  const float sqr_e_radius = expanded_radius * expanded_radius;

  uint64 count = 0;
  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return 0;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return 0;
      }

    for (int x = bbox_min_i.x(); x <= bbox_max_i.x(); x++)
      for (int y = bbox_min_i.y(); y <= bbox_max_i.y(); y++)
        for (int z = bbox_min_i.z(); z <= bbox_max_i.z(); z++)
          if ((Eigen::Vector3f(x,y,z) - expanded_center).squaredNorm() <= sqr_e_radius)
            if (m_octree.GetInt(x,y,z) != inverse)
              count++;

    m_cube_cond.notify_all();
    }

  return count;
  }

TWeightCubeListener::uint64 TWeightCubeListener::CountOccupiedVoxelsInBBox(
  const Eigen::Vector3f & bbox_min,const Eigen::Vector3f & bbox_max,bool inverse)
  {
  const Eigen::Vector3i bbox_min_i = (bbox_min / m_voxel_size).cast<int>();
  const Eigen::Vector3i bbox_max_i = (bbox_max / m_voxel_size + Eigen::Vector3f::Ones() * 0.5).cast<int>();
  uint64 count = 0;

  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    if (m_is_terminating)
      return 0;
    // wait for an empty queue
    while ((!m_cube_queue.empty()) || (m_currently_working_cubes > 0))
      {
      m_cube_cond.wait(lock);
      if (m_is_terminating)
        return 0;
      }

    for (int x = bbox_min_i.x(); x <= bbox_max_i.x(); x++)
      for (int y = bbox_min_i.y(); y <= bbox_max_i.y(); y++)
        for (int z = bbox_min_i.z(); z <= bbox_max_i.z(); z++)
          if (m_octree.GetInt(x,y,z) != inverse)
            count++;

    m_cube_cond.notify_all();
    }

  return count;
  }

void TWeightCubeListener::CubeWorker()
  {
  while (true)
    {
    NewCubeInfo::Ptr new_info;
    // scope only
      {
      boost::mutex::scoped_lock lock(m_cube_mutex);
      if (m_is_terminating)
        return;
      while (m_cube_queue.empty())
        {
        m_cube_cond.wait(lock);
        if (m_is_terminating)
          return; // terminate if required.
        }

      new_info = m_cube_queue.front();
      m_cube_queue.pop_front();

      m_currently_working_cubes++;
      m_cube_cond.notify_all();
      }

    switch (new_info->task)
      {
      case NewCubeInfo::TASK_WRITE_CUBE:
        NewCubeWorker(new_info);
        break;
      case NewCubeInfo::TASK_RETRIEVE_CUBE:
        RetrieveCubeWorker(new_info);
        break;
      default:
        ROS_INFO("kinfu: CubeWorker: noop.");
        break;
      }

    // scope only
      {
      boost::mutex::scoped_lock lock(m_cube_mutex);
      if (m_is_terminating)
        return;
      m_currently_working_cubes--;
      m_cube_cond.notify_all();
      }
    }
  }

void TWeightCubeListener::NewCubeWorker(NewCubeInfo::Ptr new_info)
  {
  ROS_INFO("kinfu: NewCubeWorker: Setting known voxels...\n");
  unsigned int leaves_count = 0;
  const unsigned int z_edge = new_info->nb_voxels.y() * new_info->nb_voxels.x();
  const unsigned int y_edge = new_info->nb_voxels.x();
  const unsigned int x_edge = 1;

  const Eigen::Vector3i cyclical_shifted_origin = new_info->cyclical_shifted_origin;
  const Eigen::Vector3i nb_voxels = new_info->nb_voxels;
  const Eigen::Vector3i grid_global_origin = new_info->grid_global_origin;

  OccupancyOctree::Cache cache;

  for (int z = 0; z < nb_voxels.z(); z++)
    {
    const int wz = (int(z + cyclical_shifted_origin.z()) % nb_voxels.z()) * z_edge;
    for (int y = 0; y < nb_voxels.y(); y++)
      {
      const int wy = (int(y + cyclical_shifted_origin.y()) % nb_voxels.y()) * y_edge;
      for (int x = 0; x < nb_voxels.x(); x++)
        {
        const int wx = (int(x + cyclical_shifted_origin.x()) % nb_voxels.x()) * x_edge;

        if ((*(new_info->weights))[wx + wy + wz] > 0)
          {
          int tx,ty,tz;
          tx = x + int(grid_global_origin.x());
          ty = y + int(grid_global_origin.y());
          tz = z + int(grid_global_origin.z());
          m_octree.SetIntCached(tx,ty,tz,true,cache);
          leaves_count++;
          }
        }
      }
    }
  ROS_INFO("kinfu: NewCubeWorker: Done: set %u leaves.",leaves_count);
  ROS_INFO("kinfu: NewCubeWorker: points are %u, bitmasks are %u, full bitmasks are %u.",
    uint(m_octree.GetPointCount()),uint(m_octree.GetLeafCount()),uint(m_octree.GetFullLeafCount()));

  m_voxel_size = new_info->cube_size.x() / float(new_info->nb_voxels.x());
  }

void TWeightCubeListener::RetrieveCubeWorker(NewCubeInfo::Ptr new_info)
  {
  ROS_INFO("kinfu: RetrieveCubeWorker: working...");

  const uint size = new_info->nb_voxels.z() * new_info->nb_voxels.y() * new_info->nb_voxels.x();
  m_retrieved_weights = WeightVectorPtr(new WeightVector(size,0));
  uint leaves_count = 0;

  const unsigned int z_edge = new_info->nb_voxels.y() * new_info->nb_voxels.x();
  const unsigned int y_edge = new_info->nb_voxels.x();
  const unsigned int x_edge = 1;

  const Eigen::Vector3i cyclical_shifted_origin = new_info->cyclical_shifted_origin;
  const Eigen::Vector3i nb_voxels = new_info->nb_voxels;
  const Eigen::Vector3i grid_global_origin = new_info->grid_global_origin;

  if (m_octree.GetPointCount() > 0) // skip if empty
    {
    OccupancyOctree::Cache octree_cache;
    for (int z = 0; z < nb_voxels.z(); z++)
      {
      const int wz = (int(z + cyclical_shifted_origin.z()) % nb_voxels.z()) * z_edge;
      for (int y = 0; y < nb_voxels.y(); y++)
        {
        const int wy = (int(y + cyclical_shifted_origin.y()) % nb_voxels.y()) * y_edge;
        for (int x = 0; x < nb_voxels.x(); x++)
          {
          const int wx = (int(x + cyclical_shifted_origin.x()) % nb_voxels.x()) * x_edge;

          int tx,ty,tz;
          tx = x + int(grid_global_origin.x());
          ty = y + int(grid_global_origin.y());
          tz = z + int(grid_global_origin.z());
          if (m_octree.GetIntCached(tx,ty,tz,octree_cache))
            {
            (*m_retrieved_weights)[wx + wy + wz] = 16;
            leaves_count++;
            }
          }
        }
      }
    }

  // scope only
    {
    boost::mutex::scoped_lock lock(m_cube_mutex);
    m_queued_retrieving_cubes--;
    }

  ROS_INFO("kinfu: RetrieveCubeWorker: retrieved %u voxels.",uint(leaves_count));
  }
