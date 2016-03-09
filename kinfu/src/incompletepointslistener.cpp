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

#include "incompletepointslistener.h"

// ROS
#include <ros/ros.h>

void TIncompletePointsListener::AddAcceptedType(const ListenerType type)
  {
  m_accepted_types.insert(type);
  switch (type)
    {
    case IncompletePointsListener::TYPE_BORDERS:
      if (!m_border_cloud)
        m_border_cloud = PointXYZNormalCloud::Ptr(new PointXYZNormalCloud);
      break;
    case IncompletePointsListener::TYPE_FRONTIERS:
      if (!m_frontiers_cloud)
        m_frontiers_cloud = PointXYZNormalCloud::Ptr(new PointXYZNormalCloud);
      break;
    default:
      ROS_WARN("kinfu: incomplete points listener: ignored unknown type: %u",uint(type));
      break;
    }
  }

void TIncompletePointsListener::onAddSlice(const PointCloudXYZNormal::ConstPtr cloud, const ListenerType type)
  {
  switch (type)
    {
    case IncompletePointsListener::TYPE_BORDERS:
      if (m_border_cloud && cloud)
        (*m_border_cloud) += (*cloud);
      break;
    case IncompletePointsListener::TYPE_FRONTIERS:
      if (!m_frontiers_cloud)
        m_frontiers_cloud = PointXYZNormalCloud::Ptr(new PointXYZNormalCloud);
      break;
    default:
      ROS_WARN("kinfu: incomplete points listener: ignored unknown type: %u",uint(type));
      return;
    }

  ROS_INFO("kinfu: incomplete points listener: added slice.");
  }

void TIncompletePointsListener::onClearBBox(const Eigen::Vector3f & bbox_min, const Eigen::Vector3f & bbox_max, const ListenerType type)
  {
  if (type != IncompletePointsListener::TYPE_ANY)
    {
    ROS_WARN("kinfu: incomplete points listener: clear type is not TYPE_ANY, not supported.");
    return;
    }

  ROS_INFO("kinfu: incomplete points listener: clearing bounding box.");

  if (m_border_cloud)
    m_border_cloud = ClearBBoxCloud(m_border_cloud,bbox_min,bbox_max);
  if (m_frontiers_cloud)
    m_frontiers_cloud = ClearBBoxCloud(m_frontiers_cloud,bbox_min,bbox_max);
  }

void TIncompletePointsListener::onClearSphere(const Eigen::Vector3f & center, const float radius, const ListenerType type)
  {
  if (type != IncompletePointsListener::TYPE_ANY)
    {
    ROS_WARN("kinfu: incomplete points listener: clear type is not TYPE_ANY, not supported.");
    return;
    }

  ROS_INFO("kinfu: incomplete points listener: clearing sphere.");

  if (m_border_cloud)
    m_border_cloud = ClearSphereCloud(m_border_cloud,center,radius);
  if (m_frontiers_cloud)
    m_frontiers_cloud = ClearSphereCloud(m_frontiers_cloud,center,radius);
  }

TIncompletePointsListener::PointXYZNormalCloud::Ptr TIncompletePointsListener::ClearSphereCloud(
  const PointXYZNormalCloud::ConstPtr cloud, const Eigen::Vector3f & center, const float radius)
  {
  PointXYZNormalCloud::Ptr result(new PointXYZNormalCloud);
  const size_t size = cloud->size();
  const float sq_radius = radius*radius;
  result->reserve(size);
  for (uint i = 0; i < size; i++)
    {
    const pcl::PointNormal & pt = (*cloud)[i];
    const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
    if ((ept - center).squaredNorm() > sq_radius)
      result->push_back(pt);
    }
  return result;
  }

TIncompletePointsListener::PointXYZNormalCloud::Ptr TIncompletePointsListener::ClearBBoxCloud(
  const PointXYZNormalCloud::ConstPtr cloud, const Eigen::Vector3f & bbox_min, const Eigen::Vector3f & bbox_max)
  {
  PointXYZNormalCloud::Ptr result(new PointXYZNormalCloud);
  const size_t size = cloud->size();
  result->reserve(size);
  for (uint i = 0; i < size; i++)
    {
    const pcl::PointNormal & pt = (*cloud)[i];
    const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
    if ((ept.array() < bbox_min.array()).any() || (ept.array() > bbox_max.array()).any())
      result->push_back(pt);
    }
  return result;
  }
