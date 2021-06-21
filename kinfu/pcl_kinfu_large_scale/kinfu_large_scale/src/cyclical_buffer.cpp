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

pcl::gpu::kinfuLS::CyclicalBuffer::DefaultShiftChecker::
  DefaultShiftChecker(const double distance_camera_target, const double distance_threshold)
{
  m_distance_threshold = distance_threshold;
  m_distance_camera_target = distance_camera_target;
}

bool pcl::gpu::kinfuLS::CyclicalBuffer::DefaultShiftChecker::CheckForShift(const Eigen::Affine3f & cam_pose,
                                                                           const Eigen::Vector3f & center_cube)
{
  Eigen::Vector3f targetPoint;
  targetPoint.x() = 0.0f;
  targetPoint.y() = 0.0f;
  targetPoint.z() = m_distance_camera_target; // place the point at camera position + distance_camera_target on Z
  targetPoint = cam_pose * targetPoint;

  return (targetPoint - center_cube).norm() > m_distance_threshold;
}

Eigen::Vector3f pcl::gpu::kinfuLS::CyclicalBuffer::DefaultShiftChecker::GetNewCubeOrigin(const Eigen::Affine3f & cam_pose)
{
  Eigen::Vector3f targetPoint;
  targetPoint.x() = 0.0f;
  targetPoint.y() = 0.0f;
  targetPoint.z() = m_distance_camera_target; // place the point at camera position + distance_camera_target on Z
  targetPoint = cam_pose * targetPoint;

  return targetPoint;
}

bool
pcl::gpu::kinfuLS::CyclicalBuffer::checkForShift (const TsdfVolume::Ptr volume,
                                                  const Eigen::Affine3f & initial_cam_pose,
                                                  const Eigen::Affine3f & cam_pose,
                                                  IShiftChecker & shift_checker,
                                                  const bool perform_shift,
                                                  const bool last_shift,
                                                  const bool force_shift)
{
  // check distance from the cube's center
  pcl::PointXYZ center_cube;
  center_cube.x = buffer_.origin_metric.x + buffer_.volume_size.x/2.0f;
  center_cube.y = buffer_.origin_metric.y + buffer_.volume_size.y/2.0f;
  center_cube.z = buffer_.origin_metric.z + buffer_.volume_size.z/2.0f;

  checkOldCubeRetrieval(volume);

  const Eigen::Affine3f inv_initial_cam_pose = initial_cam_pose.inverse();
  const Eigen::Affine3f global_cam_pose = inv_initial_cam_pose * cam_pose;
  const Eigen::Vector3f global_center_cube = inv_initial_cam_pose * Eigen::Vector3f(center_cube.x, center_cube.y, center_cube.z);
  const bool distance_shift = shift_checker.CheckForShift(global_cam_pose, global_center_cube);

  if (!perform_shift && !force_shift)
    return (distance_shift);

  if (distance_shift && !old_cube_retrieved_ && !force_shift)
    return false;
    // delaying shift while previous weight cube is uploaded.

  if (force_shift && !old_cube_retrieved_)
    PCL_WARN("checkForShift: forced shifting, but previous weight cube was not uploaded!");

  // perform shifting operations
  if (distance_shift || force_shift)
  {
    const Eigen::Vector3f global_new_center_cube = shift_checker.GetNewCubeOrigin(global_cam_pose);
    const Eigen::Vector3f new_center_cube = initial_cam_pose * global_new_center_cube;
    pcl::PointXYZ npt;
    npt.x = new_center_cube.x();
    npt.y = new_center_cube.y();
    npt.z = new_center_cube.z();
    performShift (volume, npt);
  }

  return (distance_shift);
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
pcl::gpu::kinfuLS::CyclicalBuffer::pushTSDFCloudToTSDF(const TsdfVolume::Ptr volume,
                                                       const pcl::PointCloud<pcl::PointXYZI>::ConstPtr tsdf_cloud,
                                                       const Eigen::Vector3i & bbox_min,
                                                       const Eigen::Vector3i & bbox_max)
{
  const Eigen::Vector3i origin_GRID_global(buffer_.origin_GRID_global.x,
                                           buffer_.origin_GRID_global.y,
                                           buffer_.origin_GRID_global.z);
  const Eigen::Vector3i origin_GRID(buffer_.origin_GRID.x,
                                    buffer_.origin_GRID.y,
                                    buffer_.origin_GRID.z);
  const Eigen::Vector3i voxels_size(buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);
  const Eigen::Vector3i max_GRID_global = origin_GRID_global + voxels_size;
  const Eigen::Vector3i GRID_min = bbox_min.array().max(origin_GRID_global.array());
  const Eigen::Vector3i GRID_max = bbox_max.array().min(max_GRID_global.array());

  pcl::PointCloud<pcl::PointXYZI>::Ptr existing_slice(new pcl::PointCloud<pcl::PointXYZI>);
  existing_slice->reserve(tsdf_cloud->size());
  {
    const size_t world_size = tsdf_cloud->size();
    for (size_t i = 0; i < world_size; i++)
    {
      const pcl::PointXYZI & ipt = (*tsdf_cloud)[i];
      const Eigen::Vector3i ept(ipt.x, ipt.y, ipt.z);
      if ((ept.array() >= GRID_min.array()).all() &&
          (ept.array() < GRID_max.array()).all())
        existing_slice->push_back(ipt);
    }
    existing_slice->height = 1;
    existing_slice->width = existing_slice->size();
  }

  volume->clearBBox(origin_GRID, (GRID_min - origin_GRID_global).cast<float>(),
                    (GRID_max - origin_GRID_global).cast<float>(), false);

  PCL_INFO("pushWorldModelToTSDF: pushing %u points.\n", unsigned(existing_slice->size ()));

  {
    Eigen::Affine3f transf = Eigen::Affine3f::Identity();
    transf.translation() = -origin_GRID_global.cast<float>();
    pcl::transformPointCloud(*existing_slice, *existing_slice, transf);
  }

  // push existing data in the TSDF buffer
  if (existing_slice->size () != 0 ) {
    volume->pushSlice(existing_slice, getBuffer () );
  }
}

/** \brief adds weights data in bounding box
 */
void pcl::gpu::kinfuLS::CyclicalBuffer::pushWeightsToTSDF(const TsdfVolume::Ptr volume,
                                                          const std::vector<short> & weights,
                                                          const Eigen::Vector3i & bbox_min,
                                                          const Eigen::Vector3i & bbox_max)
{
  const Eigen::Vector3i origin_GRID_global(buffer_.origin_GRID_global.x,
                                           buffer_.origin_GRID_global.y,
                                           buffer_.origin_GRID_global.z);
  const Eigen::Vector3i origin_GRID(buffer_.origin_GRID.x,
                                    buffer_.origin_GRID.y,
                                    buffer_.origin_GRID.z);
  const Eigen::Vector3i voxels_size(buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);
  const Eigen::Vector3i max_GRID_global = origin_GRID_global + voxels_size;
  const Eigen::Vector3i GRID_min = bbox_min.array().max(origin_GRID_global.array());
  const Eigen::Vector3i GRID_max = bbox_max.array().min(max_GRID_global.array());
  const Eigen::Vector3i bbox_size = bbox_max - bbox_min;
  const Eigen::Vector3i GRID_size = GRID_max - GRID_min;

  if ((GRID_size.array() > 0).all()) // there is intersection between bounding box and TSDF
  {
    DeviceArray2D<short> device_weights;
    {
      std::vector<short> dw(size_t(GRID_size.x()) * GRID_size.y() * GRID_size.z());

      Eigen::Vector3i i;
      for (i.z() = 0; i.z() < GRID_size.z(); i.z()++)
        for (i.y() = 0; i.y() < GRID_size.y(); i.y()++)
          for (i.x() = 0; i.x() < GRID_size.x(); i.x()++)
          {
            const size_t di = i.x() + size_t(i.y()) * GRID_size.x() + size_t(i.z()) * GRID_size.x() * GRID_size.y();
            const Eigen::Vector3i ni = i + GRID_min - bbox_min;
            const size_t si = ni.x() + size_t(ni.y()) * bbox_size.x() + size_t(ni.z()) * bbox_size.x() * bbox_size.y();
            dw[di] = weights[si];
          }

      device_weights.create(GRID_size.y() * GRID_size.z(), GRID_size.x());
      device_weights.upload(dw, GRID_size.x());
    }


    PCL_INFO ("pushWeightsToTSDF: known: uploading weights...\n");
    volume->uploadKnownToBBox(origin_GRID, GRID_min, GRID_max, device_weights);
    PCL_INFO ("known: upload complete.\n");
  }

  // notify the listeners
  PCL_INFO ("pushWeightsToTSDF: known: notifying listeners...\n");
  PointCloudXYZNormal::Ptr cleared_frontier_cloud;
  if (incomplete_points_listener_ &&
      incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_CLEARED_FRONTIERS))
    cleared_frontier_cloud.reset(new PointCloudXYZNormal);

  if (weight_cube_listener_)
    weight_cube_listener_->onReplaceBBox(weights,
                                         bbox_min, bbox_max,
                                         cleared_frontier_cloud);
  if (incomplete_points_listener_)
    incomplete_points_listener_->onClearBBox(bbox_min.cast<float>(), bbox_max.cast<float>(),
                                             IncompletePointsListener::TYPE_ANY);
  if (incomplete_points_listener_ && cleared_frontier_cloud)
    incomplete_points_listener_->onAddSlice(cleared_frontier_cloud, IncompletePointsListener::TYPE_CLEARED_FRONTIERS);
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::performShift (const TsdfVolume::Ptr volume, const pcl::PointXYZ &target_point)
{
  // compute new origin and offsets
  int offset_x, offset_y, offset_z;
  computeAndSetNewCubeMetricOrigin (target_point, offset_x, offset_y, offset_z);
  last_shifting_offset_ = Eigen::Vector3i(offset_x,offset_y,offset_z);

  // extract current slice from the TSDF volume (coordinates are in indices! (see fetchSliceAsCloud() )
  // note: offset is set to maximum, so the whole TSDF volume is always downloaded (1)
  PointCloudXYZI::Ptr current_slice;
  current_slice = volume->fetchSliceAsPointCloud (cloud_buffer_device_xyz_, cloud_buffer_device_intensities_,
    last_data_transfer_matrix_device_, &buffer_, buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1);

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
    const Eigen::Vector3f voxels_size(buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);
    const Eigen::Vector3f origin_max = origin + voxels_size - Eigen::Vector3f::Ones();
    const Eigen::Vector3f origin_min = origin + Eigen::Vector3f::Ones();
    incomplete_points_listener_->onClearBBox(origin_min, origin_max, IncompletePointsListener::TYPE_ANY);

    if (incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_BORDERS))
    {
      PointCloudXYZNormal::Ptr old_cloud = volume->fetchIncompletePointsAsPointCloud(
        cloud_buffer_device_xyz_, cloud_buffer_device_normals_,
        1, last_data_transfer_matrix_device_, &buffer_);
      transformPointCloud (*old_cloud, *old_cloud, global_cloud_transformation);
      incomplete_points_listener_->onAddSlice(old_cloud, IncompletePointsListener::TYPE_BORDERS);
    }

    if (incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_FRONTIERS))
    {
      PointCloudXYZNormal::Ptr old_cloud = volume->fetchIncompletePointsAsPointCloud(
        cloud_buffer_device_xyz_, cloud_buffer_device_normals_,
        0, last_data_transfer_matrix_device_, &buffer_);
      transformPointCloud (*old_cloud, *old_cloud, global_cloud_transformation);
      incomplete_points_listener_->onAddSlice(old_cloud, IncompletePointsListener::TYPE_FRONTIERS);
    }

    if (incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_SURFACES))
    {
      PointCloudXYZNormal::Ptr old_cloud = volume->fetchIncompletePointsAsPointCloud(
        cloud_buffer_device_xyz_, cloud_buffer_device_normals_,
        2, last_data_transfer_matrix_device_, &buffer_);
      transformPointCloud (*old_cloud, *old_cloud, global_cloud_transformation);
      incomplete_points_listener_->onAddSlice(old_cloud, IncompletePointsListener::TYPE_SURFACES);
    }
  }

  // retrieve existing data from the world model
  PointCloud<PointXYZI>::Ptr previously_existing_slice (new  PointCloud<PointXYZI>);

  world_model_.getExistingData (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                                offset_x, offset_y, offset_z,
                                buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z,
                                *previously_existing_slice);

  //replace world model data with values extracted from the TSDF buffer slice
  // clear full cube, as we are downloading the full TSDF volume (see (1) above)
  world_model_.setFullCubeAsNans (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                                  buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);
//  world_model_.setSliceAsNans (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
//                               offset_x, offset_y, offset_z,
//                               buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);


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
pcl::gpu::kinfuLS::CyclicalBuffer::clearSphere(const Eigen::Vector3f & center,float radius,bool set_to_empty)
{
  world_model_.clearSphere(center,radius);

  PointCloudXYZNormal::Ptr cleared_frontiers_cloud;
  if (incomplete_points_listener_ &&
      incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_CLEARED_FRONTIERS))
    cleared_frontiers_cloud.reset(new PointCloudXYZNormal);

  if (weight_cube_listener_)
    weight_cube_listener_->onClearSphere(center,radius,set_to_empty,cleared_frontiers_cloud);
  if (incomplete_points_listener_)
    incomplete_points_listener_->onClearSphere(center, radius, IncompletePointsListener::TYPE_ANY);
  if (incomplete_points_listener_ && cleared_frontiers_cloud)
    incomplete_points_listener_->onAddSlice(cleared_frontiers_cloud,IncompletePointsListener::TYPE_CLEARED_FRONTIERS);
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::clearCylinder(const Eigen::Vector3f & center,const Eigen::Vector3f & height_bearing,
                                                 float radius,float half_height,bool set_to_empty)
{
  world_model_.clearCylinder(center,height_bearing,radius,half_height);

  PointCloudXYZNormal::Ptr cleared_frontiers_cloud;
  if (incomplete_points_listener_ &&
      incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_CLEARED_FRONTIERS))
    cleared_frontiers_cloud.reset(new PointCloudXYZNormal);

  if (weight_cube_listener_)
    weight_cube_listener_->onClearCylinder(center, height_bearing, radius, half_height,
                                           set_to_empty, cleared_frontiers_cloud);
  if (incomplete_points_listener_)
    incomplete_points_listener_->onClearCylinder(center, height_bearing, radius,
                                                 half_height, IncompletePointsListener::TYPE_ANY);
  if (incomplete_points_listener_ && cleared_frontiers_cloud)
    incomplete_points_listener_->onAddSlice(cleared_frontiers_cloud,IncompletePointsListener::TYPE_CLEARED_FRONTIERS);
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::clearBBox(const Eigen::Vector3f & min,const Eigen::Vector3f & max,bool set_to_empty)
{
  world_model_.clearBBox(min,max);

  PointCloudXYZNormal::Ptr cleared_frontier_cloud;
  if (incomplete_points_listener_ &&
      incomplete_points_listener_->acceptsType(IncompletePointsListener::TYPE_CLEARED_FRONTIERS))
    cleared_frontier_cloud.reset(new PointCloudXYZNormal);

  if (weight_cube_listener_)
    weight_cube_listener_->onClearBBox(min,max, set_to_empty, cleared_frontier_cloud);
  if (incomplete_points_listener_)
    incomplete_points_listener_->onClearBBox(min, max, IncompletePointsListener::TYPE_ANY);
  if (incomplete_points_listener_ && cleared_frontier_cloud)
    incomplete_points_listener_->onAddSlice(cleared_frontier_cloud, IncompletePointsListener::TYPE_CLEARED_FRONTIERS);
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
