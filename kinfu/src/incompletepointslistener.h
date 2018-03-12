/*
 * Copyright (c) 2015, Riccardo Monica
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

#ifndef INCOMPLETEPOINTSLISTENER_H
#define INCOMPLETEPOINTSLISTENER_H

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// STL
#include <set>
#include <stdint.h>

// Boost
#include <boost/shared_ptr.hpp>

class TIncompletePointsListener: public pcl::gpu::kinfuLS::CyclicalBuffer::IncompletePointsListener
  {
  public:
  typedef pcl::gpu::kinfuLS::CyclicalBuffer::IncompletePointsListener IncompletePointsListener;
  typedef IncompletePointsListener::Type ListenerType;
  typedef pcl::PointCloud<pcl::PointNormal> PointXYZNormalCloud;
  typedef unsigned int uint;
  typedef boost::shared_ptr<TIncompletePointsListener> Ptr;
  typedef boost::shared_ptr<const TIncompletePointsListener> ConstPtr;

  bool acceptsType(const ListenerType type) const;

  void onAddSlice(const PointCloudXYZNormal::ConstPtr cloud, const ListenerType type);

  void onClearBBox(const Eigen::Vector3f & bbox_min, const Eigen::Vector3f & bbox_max, const ListenerType type);
  void onClearSphere(const Eigen::Vector3f & center, const float radius, const ListenerType type);

  void AddAcceptedType(const ListenerType type);

  PointXYZNormalCloud::ConstPtr GetBorderCloud() {return m_border_cloud; }
  PointXYZNormalCloud::ConstPtr GetFrontierCloud() {return m_frontiers_cloud; }
  PointXYZNormalCloud::ConstPtr GetSurfacesCloud() {return m_surfaces_cloud; }

  private:
  PointXYZNormalCloud::Ptr ClearSphereCloud(const PointXYZNormalCloud::ConstPtr cloud,
    const Eigen::Vector3f & center, const float radius);
  PointXYZNormalCloud::Ptr ClearBBoxCloud(const PointXYZNormalCloud::ConstPtr cloud,
    const Eigen::Vector3f & bbox_min, const Eigen::Vector3f & bbox_max);

  std::set<ListenerType> m_accepted_types;
  PointXYZNormalCloud::Ptr m_border_cloud;
  PointXYZNormalCloud::Ptr m_frontiers_cloud;
  PointXYZNormalCloud::Ptr m_surfaces_cloud;
  };

#endif // INCOMPLETEPOINTSLISTENER_H
