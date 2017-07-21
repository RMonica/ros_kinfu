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

#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>
#include <kinfu/bitmaskoctree.h>
#include <sensor_msgs/fill_image.h>
#include <vector>
#include <algorithm>
#include <cstring>

template <class PointT>
void WorldDownloadManager::separateMesh(Mesh::ConstPtr mesh,
                                        typename pcl::PointCloud<PointT>::Ptr points,
                                        TriangleVectorPtr triangles)
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

      Triangle tri;

      for (uint i = 0; i < 3; i++)
        tri[i] = v.vertices[i];

      triangles->push_back(tri);
    }
  }
}

template void WorldDownloadManager::separateMesh<pcl::PointXYZ>
  (Mesh::ConstPtr mesh,typename pcl::PointCloud<pcl::PointXYZ>::Ptr points,TriangleVectorPtr triangles);
template void WorldDownloadManager::separateMesh<pcl::PointNormal>
  (Mesh::ConstPtr mesh,typename pcl::PointCloud<pcl::PointNormal>::Ptr points,TriangleVectorPtr triangles);

template <class PointT>
void WorldDownloadManager::mergePointCloudsAndMesh(std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointclouds,
  typename pcl::PointCloud<PointT>::Ptr out_cloud,std::vector<TriangleVectorPtr> * meshes,TriangleVector * out_mesh)
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
        Triangle tri;
        const Triangle & v = (*(*meshes)[pointcloud_i])[triangle_i];

        for (uint i = 0; i < 3; i++)
          tri[i] = v[i] + offset;

        out_mesh->push_back(tri);
      }

      offset += pointcloud_size;
    }
  }
}

template void WorldDownloadManager::mergePointCloudsAndMesh<pcl::PointXYZ>(
  std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> &pointclouds,
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, std::vector<TriangleVectorPtr> * meshes,TriangleVector * out_mesh);
template void WorldDownloadManager::mergePointCloudsAndMesh<pcl::PointNormal>(
  std::vector<typename pcl::PointCloud<pcl::PointNormal>::Ptr> &pointclouds,
  typename pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud, std::vector<TriangleVectorPtr> * meshes,TriangleVector * out_mesh);

template <class PointT>
void WorldDownloadManager::cropMesh(const kinfu_msgs::KinfuCloudPoint & min,
  const kinfu_msgs::KinfuCloudPoint & max,typename pcl::PointCloud<PointT>::ConstPtr cloud,
  TriangleVectorConstPtr triangles,typename pcl::PointCloud<PointT>::Ptr out_cloud,TriangleVectorPtr out_triangles)
{
  const uint triangles_size = triangles->size();
  const uint cloud_size = cloud->size();

  std::vector<bool> valid_points(cloud_size,true);

  std::vector<uint> valid_points_remap(cloud_size,0);

  uint offset;

  // check the points
  for (uint i = 0; i < cloud_size; i++)
  {
    const PointT & pt = (*cloud)[i];

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
    const Triangle & tri = (*triangles)[i];
    bool is_valid = true;

    // validate all the vertices
    for (uint h = 0; h < 3; h++)
      if (!valid_points[tri[h]])
      {
        is_valid = false;
        break;
      }

    if (is_valid)
    {
      Triangle out_tri;

      // remap the triangle
      for (uint h = 0; h < 3; h++)
        out_tri[h] = valid_points_remap[(*triangles)[i][h]];

      out_triangles->push_back(out_tri);
      offset++;
    }

  }
  out_triangles->resize(offset);
}

template void WorldDownloadManager::cropMesh<pcl::PointXYZ>(const kinfu_msgs::KinfuCloudPoint & min,
  const kinfu_msgs::KinfuCloudPoint & max,typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  TriangleVectorConstPtr triangles,typename pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, TriangleVectorPtr out_triangles);
template void WorldDownloadManager::cropMesh<pcl::PointNormal>(const kinfu_msgs::KinfuCloudPoint & min,
  const kinfu_msgs::KinfuCloudPoint & max,typename pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
  TriangleVectorConstPtr triangles,typename pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud, TriangleVectorPtr out_triangles);

template <class PointT>
  void WorldDownloadManager::cropCloud(const Eigen::Vector3f & min,const Eigen::Vector3f & max,
  typename pcl::PointCloud<PointT>::ConstPtr cloud,typename pcl::PointCloud<PointT>::Ptr out_cloud)
{
  const uint cloud_size = cloud->size();

  std::vector<bool> valid_points(cloud_size,true);

  // check the points
  for (uint i = 0; i < cloud_size; i++)
  {
    const PointT & pt = (*cloud)[i];

    if (pt.x > max.x() || pt.y > max.y() || pt.z > max.z() ||
      pt.x < min.x() || pt.y < min.y() || pt.z < min.z())
      valid_points[i] = false;
  }

  // discard invalid points
  out_cloud->clear();
  out_cloud->reserve(cloud_size);
  uint count = 0;

  for (uint i = 0; i < cloud_size; i++)
    if (valid_points[i])
    {
      out_cloud->push_back((*cloud)[i]);
      count++;
    }
  out_cloud->resize(count);
}

template void WorldDownloadManager::cropCloud<pcl::PointNormal>(const Eigen::Vector3f & min,const Eigen::Vector3f & max,
  typename pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,typename pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud);
template void WorldDownloadManager::cropCloud<pcl::PointXYZ>(const Eigen::Vector3f & min,const Eigen::Vector3f & max,
  typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,typename pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);

template <class PointT>
  void WorldDownloadManager::cropCloudWithSphere(const Eigen::Vector3f & center,const float radius,
  typename pcl::PointCloud<PointT>::ConstPtr cloud,typename pcl::PointCloud<PointT>::Ptr out_cloud)
{
  const uint cloud_size = cloud->size();

  std::vector<bool> valid_points(cloud_size,true);

  // check the points
  for (uint i = 0; i < cloud_size; i++)
  {
    const PointT & pt = (*cloud)[i];
    const Eigen::Vector3f ept(pt.x,pt.y,pt.z);

    if ((ept - center).squaredNorm() > radius * radius)
      valid_points[i] = false;
  }

  // discard invalid points
  out_cloud->clear();
  out_cloud->reserve(cloud_size);
  uint count = 0;

  for (uint i = 0; i < cloud_size; i++)
    if (valid_points[i])
    {
      out_cloud->push_back((*cloud)[i]);
      count++;
    }
  out_cloud->resize(count);
}

template void WorldDownloadManager::cropCloudWithSphere<pcl::PointNormal>(const Eigen::Vector3f & center,const float radius,
  typename pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,typename pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud);
template void WorldDownloadManager::cropCloudWithSphere<pcl::PointXYZ>(const Eigen::Vector3f & center,const float radius,
  typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,typename pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);

bool WorldDownloadManager::marchingCubes(TsdfCloud::Ptr cloud, std::vector<Mesh::Ptr> &output_meshes) const
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
    wm.getWorldAsCubes (m_marching_cubes_volume_size, clouds, transforms, 0.025); // 2.5% overlapp (12 cells with a 512-wide cube)

    //Creating the standalone marching cubes instance
    float volume_size = pcl::device::kinfuLS::VOLUME_SIZE / pcl::device::kinfuLS::VOLUME_X * m_marching_cubes_volume_size;

    std::cout << "Processing world with volume size set to " << volume_size << "meters\n";

    pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> marching_cubes (m_marching_cubes_volume_size,
      m_marching_cubes_volume_size, m_marching_cubes_volume_size, volume_size);

    marching_cubes.getMeshesFromTSDFVectorMemory(clouds,transforms,output_meshes);

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

void WorldDownloadManager::transformBoundingBoxAndExpand(const Eigen::Vector3f& bbox_min,const Eigen::Vector3f& bbox_max,
  const Eigen::Affine3f& transform,Eigen::Vector3f& bbox_min_out,Eigen::Vector3f& bbox_max_out)
{
  const int SIZE = 2;
  Eigen::Vector3f inv[SIZE];
  inv[0] = bbox_min;
  inv[1] = bbox_max;
  Eigen::Vector3f comb;
  for (uint ix = 0; ix < SIZE; ix++)
  {
    comb.x() = inv[ix].x();
    for (uint iy = 0; iy < SIZE; iy++)
    {
      comb.y() = inv[iy].y();
      for (uint iz = 0; iz < SIZE; iz++)
      {
        comb.z() = inv[iz].z();
        const Eigen::Vector3f t_comb = transform * comb;
        if (!ix && !iy && !iz) // first iteration
          bbox_min_out = bbox_max_out = t_comb;
        else
        {
          bbox_min_out = bbox_min_out.array().min(t_comb.array());
          bbox_max_out = bbox_max_out.array().max(t_comb.array());
        }
      }
    }
  }
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
  float min_range = 0.0;

  if (has_intrinsics)
  {
    ROS_INFO("kinfu: custom intrinsics will be used.");
    cx = intr.center_x;
    cy = intr.center_y;
    fx = intr.focal_x;
    fy = intr.focal_y;
    min_range = intr.min_range;
  }
  else
    m_kinfu->getDepthIntrinsics(fx,fy,cx,cy);

  if (!m_raycaster || (uint(m_raycaster->rows) != rows) || (uint(m_raycaster->cols) != cols))
  {
    ROS_INFO("kinfu: initializing raycaster...");
    m_raycaster = RayCaster::Ptr(new RayCaster(rows,cols));
  }

  m_raycaster->setRaycastStep(m_kinfu->volume().getTsdfTruncDist() * 0.6);
  m_raycaster->setMinRange(min_range);
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

void WorldDownloadManager::cropTsdfCloud(const TsdfCloud & cloud_in,TsdfCloud & cloud_out,
  const kinfu_msgs::KinfuCloudPoint & min,const kinfu_msgs::KinfuCloudPoint & max)
{
  cloud_out.clear();

  uint in_size = cloud_in.size();

  cloud_out.points.reserve(in_size);

  for (uint i = 0; i < in_size; i++)
  {
    const pcl::PointXYZI & s = cloud_in[i];

    if (s.x > max.x || s.y > max.y || s.z > max.z ||
      s.x < min.x || s.y < min.y || s.z < min.z)
      continue;

    pcl::PointXYZI d;

    d.x = s.x;
    d.y = s.y;
    d.z = s.z;
    d.intensity = s.intensity;

    cloud_out.push_back(d);
  }
}

template <class PointT>
class WorldDownloadManager_mergeMeshTrianglesComparator
{
  public:
  WorldDownloadManager_mergeMeshTrianglesComparator(const pcl::PointCloud<PointT> & cloud): m_cloud(cloud) {}

  bool operator() (uint64_t a,uint64_t b)
  {
    const PointT & apt = m_cloud[a];
    const PointT & bpt = m_cloud[b];
    int result = std::memcmp(apt.data,bpt.data,sizeof(float) * 3);
    if (result != 0)
      return result < 0;
    return a < b;
  }

  const pcl::PointCloud<PointT> & m_cloud;
};

template <class PointT>
class WorldDownloadManager_mergeMeshTrianglesAverageClass
{
};

template <>
class WorldDownloadManager_mergeMeshTrianglesAverageClass<pcl::PointXYZ>
{
  public:
  WorldDownloadManager_mergeMeshTrianglesAverageClass() {}

  void insert(const pcl::PointXYZ & pt) { m_pt = pt; }
  pcl::PointXYZ get() { return m_pt; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  pcl::PointXYZ m_pt;
};

template <>
class WorldDownloadManager_mergeMeshTrianglesAverageClass<pcl::PointNormal>
{
  public:
  WorldDownloadManager_mergeMeshTrianglesAverageClass()
  {
    m_counter = 0;
    m_pt.normal_x = 0.0;
    m_pt.normal_y = 0.0;
    m_pt.normal_z = 0.0;
    m_pt.curvature = 0.0;
  }

  void insert(const pcl::PointNormal & pt)
  {
    m_pt.x = pt.x; m_pt.y = pt.y; m_pt.z = pt.z;

    m_pt.normal_x += pt.normal_x;
    m_pt.normal_y += pt.normal_y;
    m_pt.normal_z += pt.normal_z;
    m_pt.curvature += pt.curvature;
    m_counter++;
  }
  pcl::PointNormal get()
  {
    pcl::PointNormal result;
    result.x = m_pt.x; result.y = m_pt.y; result.z = m_pt.z;
    result.curvature = m_pt.curvature / float(m_counter);
    const float normal_norm =
      std::sqrt((m_pt.normal_x * m_pt.normal_x) + (m_pt.normal_y * m_pt.normal_y) + (m_pt.normal_z * m_pt.normal_z));
    result.normal_x = m_pt.normal_x / normal_norm;
    result.normal_y = m_pt.normal_y / normal_norm;
    result.normal_z = m_pt.normal_z / normal_norm;
    return result;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  uint m_counter;
  pcl::PointNormal m_pt;
};

template <class PointT>
  void WorldDownloadManager::removeDuplicatePoints(typename pcl::PointCloud<PointT>::ConstPtr cloud,
  TriangleVectorConstPtr triangles, typename pcl::PointCloud<PointT>::Ptr out_cloud, TriangleVectorPtr out_triangles)
{
  const uint64 input_size = cloud->size();
  std::vector<uint64> ordered(input_size);
  for (uint64 i = 0; i < input_size; i++)
    ordered[i] = i;

  std::sort(ordered.begin(),ordered.end(),WorldDownloadManager_mergeMeshTrianglesComparator<PointT>(*cloud));

  typedef WorldDownloadManager_mergeMeshTrianglesAverageClass<PointT> AverageClass;
  std::vector<uint64> re_association_vector(input_size);
  std::vector<AverageClass, Eigen::aligned_allocator<AverageClass> > average_vector;

  uint64 output_i = 0;
  re_association_vector[ordered[0]] = 0;

  for (uint64 src_i = 1; src_i < input_size; src_i++)
  {
    const PointT & prev = (*cloud)[ordered[src_i - 1]];
    const PointT & current = (*cloud)[ordered[src_i]];
    const int not_equal = std::memcmp(prev.data,current.data,sizeof(float) * 3);
    if (not_equal)
      output_i++;
    re_association_vector[ordered[src_i]] = output_i;
  }
  const uint64 output_size = output_i + 1;

  out_cloud->resize(output_size);
  average_vector.resize(output_size);
  for (uint64 i = 0; i < input_size; i++)
    average_vector[re_association_vector[i]].insert((*cloud)[i]);
  for (uint64 i = 0; i < output_size; i++)
    (*out_cloud)[i] = average_vector[i].get();

  if (out_triangles && triangles)
  {
    const uint64 triangles_size = triangles->size();
    uint64 out_triangles_size = 0;
    out_triangles->resize(triangles_size);
    for (uint64 i = 0; i < triangles_size; i++)
    {
      Triangle new_tri;
      for (uint h = 0; h < 3; h++)
        new_tri[h] = re_association_vector[(*triangles)[i][h]];

      bool degenerate = false;
      for (uint h = 0; h < 3; h++)
        if (new_tri[h] == new_tri[(h + 1) % 3])
          degenerate = true;

      if (!degenerate)
      {
        (*out_triangles)[out_triangles_size] = new_tri;
        out_triangles_size++;
      }
    }

    out_triangles->resize(out_triangles_size);
  }
}

template void WorldDownloadManager::removeDuplicatePoints<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  TriangleVectorConstPtr triangles,typename pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud,TriangleVectorPtr out_triangles);
template void WorldDownloadManager::removeDuplicatePoints<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
  TriangleVectorConstPtr triangles,typename pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud,TriangleVectorPtr out_triangles);

void WorldDownloadManager::findExtraCubesForBoundingBox(const Eigen::Vector3f& current_cube_min,
  const Eigen::Vector3f& current_cube_max,const Eigen::Vector3f& bbox_min,const Eigen::Vector3f& bbox_max,Vector3fVector& cubes_centers,
  bool& extract_current)
{
  const Eigen::Vector3f & cube_size = current_cube_max - current_cube_min;
  cubes_centers.clear();
  extract_current = false;

  const Eigen::Vector3f relative_act_bbox_min = bbox_min - current_cube_min;
  const Eigen::Vector3f relative_act_bbox_max = bbox_max - current_cube_min;
  const Eigen::Vector3i num_cubes_plus = Eigen::Vector3f(floor3f(Eigen::Vector3f(relative_act_bbox_max.array() / cube_size.array())
    - (Eigen::Vector3f::Ones() * 0.0001))).cast<int>();
  const Eigen::Vector3i num_cubes_minus = Eigen::Vector3f(floor3f(Eigen::Vector3f(relative_act_bbox_min.array() / cube_size.array())
    + (Eigen::Vector3f::Ones() * 0.0001))).cast<int>();
  for (int z = num_cubes_minus.z(); z <= num_cubes_plus.z(); z++)
    for (int y = num_cubes_minus.y(); y <= num_cubes_plus.y(); y++)
      for (int x = num_cubes_minus.x(); x <= num_cubes_plus.x(); x++)
      {
        const Eigen::Vector3i cube_index(x,y,z);
        if ((cube_index.array() == Eigen::Vector3i::Zero().array()).all())
        {
          extract_current = true;
          continue;
        }

        const Eigen::Vector3f relative_cube_origin = cube_index.cast<float>().array() * cube_size.array();
        const Eigen::Vector3f cube_center = relative_cube_origin + current_cube_min + (cube_size * 0.5);
        cubes_centers.push_back(cube_center);
      }
}

Eigen::Vector3f WorldDownloadManager::floor3f(const Eigen::Vector3f & v)
{
  Eigen::Vector3f r;
  for (uint i = 0; i < 3; i++)
    r[i] = std::floor(v[i]);
  return r;
}
