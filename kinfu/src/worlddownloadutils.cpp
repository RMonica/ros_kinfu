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

#include "worlddownloadmanager.h"

#include <pcl/gpu/kinfu_large_scale/internal.h>
#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>
#include <kinfu/bitmaskoctree.h>
#include <sensor_msgs/fill_image.h>
#include <Eigen/StdVector>

void WorldDownloadManager::separateMesh(Mesh::ConstPtr mesh,PointCloud::Ptr points,TrianglesPtr triangles)
{
  // convert the point cloud
  pcl::fromPCLPointCloud2(mesh->cloud,*points);

  if (triangles)
  {
    // convert the triangles
    const uint mesh_size = mesh->polygons.size();
    triangles->reserve(mesh_size);

    for (uint triangle_i = 0; triangle_i < mesh_size; triangle_i++)
    {
      const pcl::Vertices &v = mesh->polygons[triangle_i];
      if (v.vertices.size() != 3)
      {
        ROS_ERROR("WARNING: polygon %u has %u vertices, only triangles are supported.",triangle_i,uint(v.vertices.size()));
        continue;
      }

      kinfu_msgs::KinfuMeshTriangle tri;

      for (uint i = 0; i < 3; i++)
        tri.vertex_id[i] = v.vertices[i];

      triangles->push_back(tri);
    }
  }
}

void WorldDownloadManager::mergePointCloudsAndMesh(std::vector<PointCloud::Ptr> &pointclouds,
  PointCloud::Ptr out_cloud, std::vector<TrianglesPtr> * meshes,Triangles * out_mesh)
{
  uint offset = 0;
  const uint pointcloud_count = pointclouds.size();

  out_cloud->clear();

  if (out_mesh)
    out_mesh->clear();

  for (uint pointcloud_i = 0; pointcloud_i < pointcloud_count; pointcloud_i++)
  {
    const uint pointcloud_size = pointclouds[pointcloud_i]->size();

    // copy the points
    (*out_cloud) += *(pointclouds[pointcloud_i]);

    if (out_mesh)
    {
      // copy the triangles, shifting vertex id by an offset
      const uint mesh_size = (*meshes)[pointcloud_i]->size();
      out_mesh->reserve(out_mesh->size() + mesh_size);

      for (uint triangle_i = 0; triangle_i < mesh_size; triangle_i++)
      {
        kinfu_msgs::KinfuMeshTriangle tri;
        const kinfu_msgs::KinfuMeshTriangle & v = (*(*meshes)[pointcloud_i])[triangle_i];

        for (uint i = 0; i < 3; i++)
          tri.vertex_id[i] = v.vertex_id[i] + offset;

        out_mesh->push_back(tri);
      }

      offset += pointcloud_size;
    }
  }
}

void WorldDownloadManager::cropMesh(const kinfu_msgs::KinfuCloudPoint & min,
  const kinfu_msgs::KinfuCloudPoint & max,PointCloud::ConstPtr cloud,
  TrianglesConstPtr triangles,PointCloud::Ptr out_cloud,TrianglesPtr out_triangles)
{
  const uint triangles_size = triangles->size();
  const uint cloud_size = cloud->size();

  std::vector<bool> valid_points(cloud_size,true);

  std::vector<uint> valid_points_remap(cloud_size,0);

  std::cout << "Starting with " << cloud_size << " points and " << triangles_size << " triangles.\n";

  uint offset;

  // check the points
  for (uint i = 0; i < cloud_size; i++)
  {
    const pcl::PointXYZ & pt = (*cloud)[i];

    if (pt.x > max.x || pt.y > max.y || pt.z > max.z ||
      pt.x < min.x || pt.y < min.y || pt.z < min.z)
      valid_points[i] = false;
  }

  // discard invalid points
  out_cloud->clear();
  out_cloud->reserve(cloud_size);
  offset = 0;

  for (uint i = 0; i < cloud_size; i++)
    if (valid_points[i])
    {
      out_cloud->push_back((*cloud)[i]);

      // save new position for triangles remap
      valid_points_remap[i] = offset;

      offset++;
    }
  out_cloud->resize(offset);

  // discard invalid triangles
  out_triangles->clear();
  out_triangles->reserve(triangles_size);
  offset = 0;

  for (uint i = 0; i < triangles_size; i++)
  {
    const kinfu_msgs::KinfuMeshTriangle & tri = (*triangles)[i];
    bool is_valid = true;

    // validate all the vertices
    for (uint h = 0; h < 3; h++)
      if (!valid_points[tri.vertex_id[h]])
      {
        is_valid = false;
        break;
      }

    if (is_valid)
    {
      kinfu_msgs::KinfuMeshTriangle out_tri;

      // remap the triangle
      for (uint h = 0; h < 3; h++)
        out_tri.vertex_id[h] = valid_points_remap[(*triangles)[i].vertex_id[h]];

      out_triangles->push_back(out_tri);
      offset++;
    }

  }
  out_triangles->resize(offset);

  std::cout << "Ended with " << out_cloud->size() << " points and " << out_triangles->size() << " triangles.\n";
}

void WorldDownloadManager::pclPointCloudToMessage(PointCloud::Ptr pcl_cloud,
  std::vector<kinfu_msgs::KinfuCloudPoint> & message)
{
  uint cloud_size = pcl_cloud->size();
  message.resize(cloud_size);

  for (uint i = 0; i < cloud_size; i++)
  {
    message[i].x = (*pcl_cloud)[i].x;
    message[i].y = (*pcl_cloud)[i].y;
    message[i].z = (*pcl_cloud)[i].z;
  }
}

bool WorldDownloadManager::marchingCubes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<Mesh::Ptr> &output_meshes) const
{
  try
  {
   // Creating world model object
    pcl::kinfuLS::WorldModel<pcl::PointXYZI> wm;

    //Adding current cloud to the world model
    wm.addSlice (cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > transforms;

    //Get world as a vector of cubes
    wm.getWorldAsCubes (pcl::device::kinfuLS::VOLUME_X, clouds, transforms, 0.025); // 2.5% overlapp (12 cells with a 512-wide cube)

    //Creating the standalone marching cubes instance
    float volume_size = pcl::device::kinfuLS::VOLUME_SIZE;

    std::cout << "Processing world with volume size set to " << volume_size << "meters\n";

    pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> m_cubes (pcl::device::kinfuLS::VOLUME_X,
      pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z, volume_size);

    m_cubes.getMeshesFromTSDFVectorMemory(clouds,transforms,output_meshes);

    std::cout << "Done!\n";
    return true;
  }
  catch (const pcl::PCLException& /*e*/) { PCL_ERROR ("PCLException... Exiting...\n"); return false; }
  catch (const std::bad_alloc& /*e*/) { PCL_ERROR ("Bad alloc... Exiting...\n"); return false; }
  catch (const std::exception& /*e*/) { PCL_ERROR ("Exception... Exiting...\n"); return false; }
}

template<typename T1,typename T2>
  Eigen::Affine3f WorldDownloadManager::toEigenAffine(const T1 & linear,const T2 & translation)
{
  Eigen::Matrix3f l;
  Eigen::Vector3f t;

  for (uint r = 0; r < 3; r++)
    for (uint c = 0; c < 3; c++)
      l(r,c) = linear[r * 3 + c];

  for (uint r = 0; r < 3; r++)
    t[r] = translation[r];

  Eigen::Affine3f result;
  result.linear() = l;
  result.translation() = t;
  return result;
}

Eigen::Affine3f WorldDownloadManager::toEigenAffine(const kinfu_msgs::KinfuPose & pose)
{
  return toEigenAffine(pose.linear,pose.translation);
}

bool WorldDownloadManager::lockKinfu()
{
  boost::mutex::scoped_lock wlock(m_kinfu_waiting_mutex);
  m_kinfu_waiting_count++;

  // wake up the kinfu thread, so "respond" may be called
  m_shared_cond.notify_one();

  while (!m_kinfu_available && !m_is_shutting_down)
    m_kinfu_waiting_cond.wait(wlock);

  if (m_is_shutting_down)
    return false;

  m_kinfu_available = false;
  return true;
}

void WorldDownloadManager::unlockKinfu()
{
  boost::mutex::scoped_lock wlock(m_kinfu_waiting_mutex);
  m_kinfu_available = true;
  m_kinfu_waiting_count--;
  m_kinfu_waiting_cond.notify_all();
}

bool WorldDownloadManager::shiftNear(const Eigen::Affine3f & pose, float distance)
{
  ROS_INFO("kinfu: shiftNear...");
  m_kinfu->shiftNear(pose,distance);
  if (!m_kinfu->isShiftComplete()) // shifting is waiting for cube
  {
    ros::Rate rate(10);
    do
    {
      rate.sleep();
      m_kinfu->shiftNear(pose,distance);
    }
    while (!m_kinfu->isShiftComplete() || ros::isShuttingDown());
  }

  if (ros::isShuttingDown())
    return false;
  return true;
}

void WorldDownloadManager::initRaycaster(bool has_intrinsics,const kinfu_msgs::KinfuCameraIntrinsics & intr,
  bool has_bounding_box_view,const kinfu_msgs::KinfuCloudPoint & bbox_min,const kinfu_msgs::KinfuCloudPoint & bbox_max)
{
  const uint rows = has_intrinsics ? intr.size_y : m_kinfu->rows();
  const uint cols = has_intrinsics ? intr.size_x : m_kinfu->cols();

  float cx,cy,fx,fy;

  if (has_intrinsics)
  {
    ROS_INFO("kinfu: custom intrinsics will be used.");
    cx = intr.center_x;
    cy = intr.center_y;
    fx = intr.focal_x;
    fy = intr.focal_y;
  }
  else
    m_kinfu->getDepthIntrinsics(fx,fy,cx,cy);

  if (!m_raycaster || (m_raycaster->rows != rows) || (m_raycaster->cols != cols))
  {
    ROS_INFO("kinfu: initializing raycaster...");
    m_raycaster = RayCaster::Ptr(new RayCaster(rows,cols));
    m_raycaster->setRaycastStep(m_kinfu->volume().getTsdfTruncDist() * 0.6);
  }

  m_raycaster->setIntrinsics(fx,fy,cx,cy);

  if (has_bounding_box_view)
    {
    const Eigen::Vector3f im(bbox_min.x,bbox_min.y,bbox_min.z);
    const Eigen::Vector3f iM(bbox_max.x,bbox_max.y,bbox_max.z);
    ROS_INFO("kinfu: raycaster will be limited to bounding box: %f %f %f - %f %f %f",im.x(),im.y(),im.z(),iM.x(),iM.y(),iM.z());
    const Eigen::Vector3f m = m_reverse_initial_transformation.inverse() * im;
    const Eigen::Vector3f M = m_reverse_initial_transformation.inverse() * iM;
    const Eigen::Vector3f mmin = m.array().min(M.array());
    const Eigen::Vector3f mmax = m.array().max(M.array());

    m_raycaster->setBoundingBox(mmin,mmax);
    }
  else
    m_raycaster->clearBoundingBox();
}

// this must be called by the kinfu thread
void WorldDownloadManager::respond(KinfuTracker * kinfu)
{
  boost::mutex::scoped_lock wlock(m_kinfu_waiting_mutex);
  if (m_kinfu_waiting_count == 0)
    return; // nothing to do

  m_kinfu = kinfu;

  m_kinfu_available = true;
  m_kinfu_waiting_cond.notify_one();

  while (m_kinfu_waiting_count > 0)
    m_kinfu_waiting_cond.wait(wlock);

  m_kinfu_available = false;
}

bool WorldDownloadManager::hasRequests()
{
  boost::mutex::scoped_lock wlock(m_kinfu_waiting_mutex);
  return m_kinfu_waiting_count > 0;
}
