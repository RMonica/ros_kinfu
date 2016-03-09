/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>
#include <pcl/common/distances.h>
#include "internal.h"



bool
pcl::gpu::kinfuLS::CyclicalBuffer::checkForShift (const TsdfVolume::Ptr volume, const Eigen::Affine3f &cam_pose, const double distance_camera_target, const bool perform_shift, const bool last_shift, const bool force_shift)
{
  bool result = false;

  // project the target point in the cube
  pcl::PointXYZ targetPoint;
  targetPoint.x = 0.0f;
  targetPoint.y = 0.0f;
  targetPoint.z = distance_camera_target; // place the point at camera position + distance_camera_target on Z
  targetPoint = pcl::transformPoint (targetPoint, cam_pose);

  // check distance from the cube's center
  pcl::PointXYZ center_cube;
  center_cube.x = buffer_.origin_metric.x + buffer_.volume_size.x/2.0f;
  center_cube.y = buffer_.origin_metric.y + buffer_.volume_size.y/2.0f;
  center_cube.z = buffer_.origin_metric.z + buffer_.volume_size.z/2.0f;

  checkOldCubeRetrieval(volume);

  if (pcl::euclideanDistance (targetPoint, center_cube) > distance_threshold_)
    result = true;

  if (!perform_shift && !force_shift)
    return (result);

  // perform shifting operations
  if (result || force_shift)
    performShift (volume, targetPoint, last_shift);

  return (result);
}

void pcl::gpu::kinfuLS::CyclicalBuffer::checkOldCubeRetrieval(const TsdfVolume::Ptr volume)
{
  if (!old_cube_retrieved_) {
    if (!weight_cube_listener_) {
      old_cube_retrieved_ = true;
      return;
    }

    WeightCubeListener::WeightVectorPtr old_weights;
    if (extract_known_points_ && weight_cube_listener_) {
      Eigen::Vector3i nb_voxels(buffer_.voxels_size.x,buffer_.voxels_size.y,buffer_.voxels_size.z);
      Eigen::Vector3i cyclical_shifted_origin(buffer_.origin_GRID.x,buffer_.origin_GRID.y,buffer_.origin_GRID.z);
      Eigen::Vector3i grid_origin(buffer_.origin_GRID_global.x,buffer_.origin_GRID_global.y,buffer_.origin_GRID_global.z);
      Eigen::Vector3f cube_size(buffer_.volume_size.x,buffer_.volume_size.y,buffer_.volume_size.z);
      if (!weight_cube_listener_->retrieveOldCube(old_weights,cube_size,nb_voxels,cyclical_shifted_origin,grid_origin)) {
        old_cube_retrieved_ = true; // feature not implemented
        PCL_INFO ("known: feature not implemented by listener.\n");
      }
    }

    // clear buffer slice and update the world model
    if (old_weights && !old_cube_retrieved_) {
      DeviceArray2D<short> device_weights;
      device_weights.create(buffer_.voxels_size.y * buffer_.voxels_size.z, buffer_.voxels_size.x);
      device_weights.upload(*old_weights,buffer_.voxels_size.x);

      PCL_INFO ("known: uploading weights...\n");
      pcl::device::kinfuLS::uploadKnownToTSDFSlice (volume->data (), &buffer_,
        last_shifting_offset_.x(), last_shifting_offset_.y(), last_shifting_offset_.z(), device_weights);
      PCL_INFO ("known: upload complete.\n");
      old_cube_retrieved_ = true;
    }
  }
}

void pcl::gpu::kinfuLS::CyclicalBuffer::getCurrentWorldBoundingBox(Eigen::Vector3f& bbox_min,Eigen::Vector3f& bbox_max) const
{
  getCurrentCubeBoundingBox(bbox_min,bbox_max);
  Eigen::Vector3f world_bbox_min,world_bbox_max;
  if (!world_model_.getWorldBoundingBox(world_bbox_min,world_bbox_max))
    return; // cube bounding box is all we have, world is empty
  const Eigen::Vector3f resize = Eigen::Vector3f(buffer_.volume_size.x,buffer_.volume_size.y,buffer_.volume_size.z).array() /
    Eigen::Vector3f(buffer_.voxels_size.x,buffer_.voxels_size.y,buffer_.voxels_size.z).array();
  world_bbox_min = world_bbox_min.array() * resize.array();
  world_bbox_max = world_bbox_max.array() * resize.array();
  bbox_min = bbox_min.array().min(world_bbox_min.array());
  bbox_max = bbox_max.array().max(world_bbox_max.array());
}

void pcl::gpu::kinfuLS::CyclicalBuffer::getCurrentCubeBoundingBox(Eigen::Vector3f& bbox_min,Eigen::Vector3f& bbox_max) const
{
  bbox_min = Eigen::Vector3f(buffer_.origin_metric.x,buffer_.origin_metric.y,buffer_.origin_metric.z);
  bbox_max = bbox_min + Eigen::Vector3f(buffer_.volume_size.x,buffer_.volume_size.y,buffer_.volume_size.z);
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::performShift (const TsdfVolume::Ptr volume, const pcl::PointXYZ &target_point, const bool last_shift)
{
  // compute new origin and offsets
  int offset_x, offset_y, offset_z;
  computeAndSetNewCubeMetricOrigin (target_point, offset_x, offset_y, offset_z);
  last_shifting_offset_ = Eigen::Vector3i(offset_x,offset_y,offset_z);

  // extract current slice from the TSDF volume (coordinates are in indices! (see fetchSliceAsCloud() )
  PointCloudXYZI::Ptr current_slice;
  if(!last_shift)
  {
    current_slice = volume->fetchSliceAsPointCloud (cloud_buffer_device_xyz_, cloud_buffer_device_intensities_,
      last_data_transfer_matrix_device_, &buffer_, offset_x, offset_y, offset_z);
  }
  else
  {
    current_slice = volume->fetchSliceAsPointCloud (cloud_buffer_device_xyz_, cloud_buffer_device_intensities_,
      last_data_transfer_matrix_device_, &buffer_, buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1);
  }

  // save the known points
  Eigen::Vector3i onNewCube_cyclical_shifted_origin(buffer_.origin_GRID.x,buffer_.origin_GRID.y,buffer_.origin_GRID.z);
  Eigen::Vector3i onNewCube_grid_origin(buffer_.origin_GRID_global.x,buffer_.origin_GRID_global.y,buffer_.origin_GRID_global.z);
  WeightCubeListener::WeightVectorPtr onNewCube_current_weights_ptr;
  if (extract_known_points_)
  {
    // download weights
    PCL_INFO("shift: downloading weights...\n");
    onNewCube_current_weights_ptr = WeightCubeListener::WeightVectorPtr(new WeightCubeListener::WeightVector);
    volume->downloadWeights(*onNewCube_current_weights_ptr);
    PCL_INFO("shift: weights downloaded.\n");
  }

  // transform the slice from local to global coordinates
  Eigen::Affine3f global_cloud_transformation;
  global_cloud_transformation.translation ()[0] = buffer_.origin_GRID_global.x;
  global_cloud_transformation.translation ()[1] = buffer_.origin_GRID_global.y;
  global_cloud_transformation.translation ()[2] = buffer_.origin_GRID_global.z;
  global_cloud_transformation.linear () = Eigen::Matrix3f::Identity ();
  transformPointCloud (*current_slice, *current_slice, global_cloud_transformation);

  if (incomplete_points_listener_)
  {
    // clear the old cube position
    const Eigen::Vector3f origin(buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z);
    const Eigen::Vector3f voxels_size(buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1);
    const Eigen::Vector3f origin_max = origin + voxels_size;
    incomplete_points_listener_->onClearBBox(origin, origin_max, IncompletePointsListener::TYPE_ANY);

    if (incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_BORDERS))
    {
      PointCloudXYZNormal::Ptr old_cloud = volume->fetchIncompletePointsAsPointCloud(
        cloud_buffer_device_xyz_, cloud_buffer_device_normals_,
        true, last_data_transfer_matrix_device_, &buffer_);
      transformPointCloud (*old_cloud, *old_cloud, global_cloud_transformation);
      incomplete_points_listener_->onAddSlice(old_cloud, IncompletePointsListener::TYPE_BORDERS);
    }

    if (incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_FRONTIERS))
    {
      PointCloudXYZNormal::Ptr old_cloud = volume->fetchIncompletePointsAsPointCloud(
        cloud_buffer_device_xyz_, cloud_buffer_device_normals_,
        false, last_data_transfer_matrix_device_, &buffer_);
      transformPointCloud (*old_cloud, *old_cloud, global_cloud_transformation);
      incomplete_points_listener_->onAddSlice(old_cloud, IncompletePointsListener::TYPE_FRONTIERS);
    }
  }

  // retrieve existing data from the world model
  PointCloud<PointXYZI>::Ptr previously_existing_slice (new  PointCloud<PointXYZI>);

  world_model_.getExistingData (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                                offset_x, offset_y, offset_z,
                                buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1,
                                *previously_existing_slice);

  //replace world model data with values extracted from the TSDF buffer slice
  world_model_.setSliceAsNans (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                               offset_x, offset_y, offset_z,
                               buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);


  PCL_INFO ("world contains %d points after update\n", world_model_.getWorldSize ());
  world_model_.cleanWorldFromNans ();
  PCL_INFO ("world contains %d points after cleaning\n", world_model_.getWorldSize ());

  // clear buffer slice and update the world model
  pcl::device::kinfuLS::clearTSDFSlice (volume->data (), &buffer_, offset_x, offset_y, offset_z);

  // insert current slice in the world if it contains any points
  if (current_slice->points.size () != 0) {
    world_model_.addSlice(current_slice);
  }

  // shift buffer addresses
  shiftOrigin (volume, offset_x, offset_y, offset_z);

  // push existing data in the TSDF buffer
  if (previously_existing_slice->points.size () != 0 ) {
    volume->pushSlice(previously_existing_slice, getBuffer () );
  }

  if (extract_known_points_) {
    old_cube_retrieved_ = false;
    checkOldCubeRetrieval(volume);

    if (weight_cube_listener_)
      {
      PCL_INFO("shift: calling weight listener...\n");
      Eigen::Vector3i nb_voxels(buffer_.voxels_size.x,buffer_.voxels_size.y,buffer_.voxels_size.z);
      Eigen::Vector3f cube_size(buffer_.volume_size.x,buffer_.volume_size.y,buffer_.volume_size.z);
      weight_cube_listener_->onNewCube(onNewCube_current_weights_ptr,cube_size,
        nb_voxels,onNewCube_cyclical_shifted_origin,onNewCube_grid_origin);
      PCL_INFO("shift: done.\n");
      }
  }
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::computeAndSetNewCubeMetricOrigin (const pcl::PointXYZ &target_point, int &shiftX, int &shiftY, int &shiftZ)
{
  // compute new origin for the cube, based on the target point
  float3 new_cube_origin_meters;
  new_cube_origin_meters.x = target_point.x - buffer_.volume_size.x/2.0f;
  new_cube_origin_meters.y = target_point.y - buffer_.volume_size.y/2.0f;
  new_cube_origin_meters.z = target_point.z - buffer_.volume_size.z/2.0f;
  PCL_INFO ("The old cube's metric origin was    (%f, %f, %f).\n", buffer_.origin_metric.x, buffer_.origin_metric.y, buffer_.origin_metric.z);
  PCL_INFO ("The new cube's metric origin is now (%f, %f, %f).\n", new_cube_origin_meters.x, new_cube_origin_meters.y, new_cube_origin_meters.z);

  // deduce each shift in indices
  shiftX = (int)( (new_cube_origin_meters.x - buffer_.origin_metric.x) * ( buffer_.voxels_size.x / (float) (buffer_.volume_size.x) ) );
  shiftY = (int)( (new_cube_origin_meters.y - buffer_.origin_metric.y) * ( buffer_.voxels_size.y / (float) (buffer_.volume_size.y) ) );
  shiftZ = (int)( (new_cube_origin_meters.z - buffer_.origin_metric.z) * ( buffer_.voxels_size.z / (float) (buffer_.volume_size.z) ) );

  // update the cube's metric origin
  buffer_.origin_metric = new_cube_origin_meters;
}
