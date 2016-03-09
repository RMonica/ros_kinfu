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
#include <Eigen/StdVector>

#define VOXEL_EPSILON 0.0058
#define SQR(x) ((x)*(x))

WorldDownloadManager::WorldDownloadManager(ros::NodeHandle &nhandle,boost::mutex &shared_mutex,boost::condition_variable & cond):
  m_nh(nhandle),m_shared_mutex(shared_mutex),m_shared_cond(cond)
{
  m_nh.param<std::string>(PARAM_NAME_REQUEST_TOPIC,m_req_topic_name,PARAM_DEFAULT_REQUEST_TOPIC);
  m_subReq = m_nh.subscribe(m_req_topic_name, 10,&WorldDownloadManager::requestCallback,this);

  m_nh.param<std::string>(PARAM_NAME_RESPONSE_TOPIC,m_resp_topic_name,PARAM_DEFAULT_RESPONSE_TOPIC);
  m_pub = m_nh.advertise<kinfu_msgs::KinfuTsdfResponse>(m_resp_topic_name,10);

  m_kinfu_waiting_count = 0;
  m_kinfu_available = false;
  m_is_shutting_down = false;

  m_cube_listener = TWeightCubeListener::Ptr(new TWeightCubeListener);
  m_incomplete_points_listener = TIncompletePointsListener::Ptr(new TIncompletePointsListener);

  m_reverse_initial_transformation = Eigen::Affine3f::Identity();
}

WorldDownloadManager::~WorldDownloadManager()
{
  {
    boost::mutex::scoped_lock thread_lock(m_thread_mutex);

    // every thread will now start shutting down
    m_is_shutting_down = true;
    m_kinfu_waiting_cond.notify_all();

    // wait for all threads to terminate
    while (!m_threads.empty())
      m_thread_cond.wait(thread_lock);

    for (uint i = 0; i < m_terminated_threads.size(); i++)
    {
      m_terminated_threads[i]->join();
      delete m_terminated_threads[i];
    }
    m_terminated_threads.clear();
  }
}

void WorldDownloadManager::requestCallback(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  // scope only
  {
    boost::mutex::scoped_lock thread_lock(m_thread_mutex);

    // while we're at it, let's cleanup a little the terminated threads queue
    for (uint i = 0; i < m_terminated_threads.size(); i++)
    {
      m_terminated_threads[i]->join();
      delete m_terminated_threads[i];
    }
    m_terminated_threads.clear();

    // create a new place in the active threads queue and an iterator to it
    m_threads.push_back(NULL);
    std::list<boost::thread *>::iterator iter = m_threads.end();
    iter--;

    // create the thread, with the iterator as a parameter
    (*iter) = new boost::thread(&WorldDownloadManager::requestWorker,this,req,iter);
  }
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

void WorldDownloadManager::requestWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req,
  std::list<boost::thread *>::iterator thread_iterator)
{
  if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_PING))
    pingWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_TSDF))
    extractTsdfWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_MESH))
    extractMeshWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_CLOUD))
    extractCloudWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_KNOWN))
    extractKnownWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_VIEW))
    extractViewWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_VIEW_CLOUD))
    extractViewCloudWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_VOXEL_COUNT))
  {
    if (!req->request_view_poses)
      extractVoxelCountGenericWorker(req);
    else
      extractVoxelCountViewsWorker(req);
  }
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_VOXELGRID))
    extractVoxelGridWorker(req);
  else if (uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_FRONTIER_POINTS) ||
    uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_BORDER_POINTS))
    extractBorderPointsWorker(req);
  else
  {
    ROS_ERROR("Request type %u (id: %u) is unknown.",
      uint(req->tsdf_header.request_type),uint(req->tsdf_header.request_id));
  }

  // when finished, transfer itself to the terminated threads queue
  {
    boost::mutex::scoped_lock thread_lock(m_thread_mutex);

    m_terminated_threads.push_back(*thread_iterator);

    m_threads.erase(thread_iterator);

    m_thread_cond.notify_one();
  }
}

void WorldDownloadManager::pingWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  // just a ping: answer immediately with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());

  resp->tsdf_header = req->tsdf_header;
  resp->reference_frame_id = m_reference_frame_name;

  // if reset is required, reset the kinfu
  if (req->request_reset)
  {
    if (!lockKinfu())
      return;

    m_kinfu->reset();

    unlockKinfu();
  }

  m_pub.publish(resp);
}

void WorldDownloadManager::extractTsdfWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Tsdf Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;
  resp->reference_frame_id = m_reference_frame_name;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  TsdfCloud::Ptr cloud = req->request_reset ?
    m_kinfu->extractWorldAndReset() : m_kinfu->extractWorld();

  unlockKinfu();

  // is crop required?
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    TsdfCloud temp;
    cropTsdfCloud(*cloud,temp,req->bounding_box_min,req->bounding_box_max);
    temp.swap(*cloud);
  }

  ROS_INFO("kinfu: Sending message...");
  fromTsdfToMessage(*cloud,resp);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Tsdf Worker complete.");
}

void WorldDownloadManager::extractCloudWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Cloud Worker started.");

  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;
  resp->reference_frame_id = m_reference_frame_name;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  ROS_INFO("kinfu: Locked.");

  TsdfCloud::Ptr kinfu_cloud = req->request_reset ?
    m_kinfu->extractWorldAndReset() : m_kinfu->extractWorld();

  unlockKinfu();

  extractCloudWorkerMCWithNormals(req,resp,kinfu_cloud);

  ROS_INFO("kinfu: Publishing...");
  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Cloud Worker complete.");
}

void WorldDownloadManager::extractCloudWorkerMCWithNormals(kinfu_msgs::KinfuTsdfRequestConstPtr req,
    kinfu_msgs::KinfuTsdfResponsePtr resp,TsdfCloud::Ptr kinfu_cloud)
{
  Eigen::Affine3f transformation = m_reverse_initial_transformation;

  std::vector<Mesh::Ptr> meshes;

  ROS_INFO("kinfu: Marching cubes...");
  if (!marchingCubes(kinfu_cloud,meshes))
    return; // ERROR

  kinfu_cloud->clear(); // save some memory

  std::vector<PointCloudXYZNormal::Ptr> clouds;

  ROS_INFO("kinfu: Extracting only points from mesh...");
  for (uint i = 0; i < meshes.size(); i++)
  {
    clouds.push_back(PointCloudXYZNormal::Ptr(new PointCloudXYZNormal()));
    separateMesh<pcl::PointNormal>(meshes[i],clouds[i]);
  }
  meshes.clear(); // save some memory

  PointCloudXYZNormal::Ptr cloud(new PointCloudXYZNormal());

  ROS_INFO("kinfu: Merging points...");
  mergePointCloudsAndMesh<pcl::PointNormal>(clouds,cloud);

  // if needed, transform
  if (req->request_transformation)
  {
    ROS_INFO("kinfu: A custom transformation will be applied.");
    Eigen::Affine3f tr = toEigenAffine(req->transformation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    PointCloudXYZNormal::Ptr out_cloud(new PointCloudXYZNormal());
    TrianglesPtr empty(new Triangles());
    cropMesh<pcl::PointNormal>(req->bounding_box_min,req->bounding_box_max,cloud,empty,out_cloud,empty);
    cloud = out_cloud;
  }

  if (req->request_remove_duplicates)
  {
    ROS_INFO("kinfu: Removing duplicate points...");
    PointCloudXYZNormal::Ptr new_cloud(new PointCloudXYZNormal());
    removeDuplicatePoints<pcl::PointNormal>(cloud,TrianglesConstPtr(),new_cloud,TrianglesPtr());
    cloud = new_cloud;
  }

  pclXYZNormalCloudToMessage(cloud,resp->point_cloud,resp->normal_cloud,resp->curvature_cloud);
}

void WorldDownloadManager::extractKnownWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Known Worker started.");

  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;
  resp->reference_frame_id = m_reference_frame_name;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  ROS_INFO("kinfu: Locked.");

  m_kinfu->syncKnownPoints();
  if (req->request_reset)
    m_kinfu->reset();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_cube_listener->GetOccupiedVoxelCenters();

  unlockKinfu();

  Eigen::Affine3f transformation = m_reverse_initial_transformation;

  if (req->request_subsample)
  {
    ROS_INFO("kinfu: Subsampling to voxel size %f",float(req->subsample_voxel_size));

    pcl::BitmaskOctree<3> octree;
    octree.SetOccupiedVoxelsCenters(*cloud,req->subsample_voxel_size);
    cloud->clear();
    octree.GetOccupiedVoxelsCenters(*cloud,req->subsample_voxel_size);
  }

  // if needed, transform
  if (req->request_transformation)
  {
    ROS_INFO("kinfu: A custom transformation will be applied.");
    Eigen::Affine3f tr = toEigenAffine(req->transformation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    PointCloudXYZ::Ptr out_cloud(new PointCloudXYZ());
    TrianglesPtr empty(new Triangles());
    cropMesh<pcl::PointXYZ>(req->bounding_box_min,req->bounding_box_max,cloud,empty,out_cloud,empty);
    cloud = out_cloud;
  }

  ROS_INFO("kinfu: Publishing...");
  pclPointCloudToMessage(cloud,resp->point_cloud);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Known Worker complete.");
}

void WorldDownloadManager::extractMeshWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Mesh Worker started.");

  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;
  resp->reference_frame_id = m_reference_frame_name;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  ROS_INFO("kinfu: Locked.");
  pcl::PointCloud<pcl::PointXYZI>::Ptr kinfu_cloud = req->request_reset ?
    m_kinfu->extractWorldAndReset() : m_kinfu->extractWorld();

  unlockKinfu();

  Eigen::Affine3f transformation = m_reverse_initial_transformation;

  std::vector<Mesh::Ptr> meshes;

  ROS_INFO("kinfu: Marching cubes...");
  if (!marchingCubes(kinfu_cloud,meshes))
    return; // ERROR

  kinfu_cloud->clear(); // save some memory

  std::vector<PointCloudXYZNormal::Ptr> clouds;
  std::vector<TrianglesPtr> clouds_triangles;

  ROS_INFO("kinfu: Divide triangles and points...");
  for (uint i = 0; i < meshes.size(); i++)
  {
    clouds.push_back(PointCloudXYZNormal::Ptr(new PointCloudXYZNormal()));
    clouds_triangles.push_back(TrianglesPtr(new Triangles()));

    separateMesh<pcl::PointNormal>(meshes[i],clouds[i],clouds_triangles[i]);
  }
  meshes.clear(); // save some memory

  TrianglesPtr triangles(new Triangles());
  PointCloudXYZNormal::Ptr cloud(new PointCloudXYZNormal());

  ROS_INFO("kinfu: Merging points...");
  mergePointCloudsAndMesh<pcl::PointNormal>(clouds,cloud,&clouds_triangles,&(*triangles));

  // if needed, transform
  if (req->request_transformation)
  {
    ROS_INFO("kinfu: A custom transformation will be applied.");
    Eigen::Affine3f tr = toEigenAffine(req->transformation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    TrianglesPtr out_triangles(new Triangles());
    PointCloudXYZNormal::Ptr out_cloud(new PointCloudXYZNormal());
    cropMesh<pcl::PointNormal>(req->bounding_box_min,req->bounding_box_max,cloud,triangles,out_cloud,out_triangles);
    cloud = out_cloud;
    triangles = out_triangles;
  }

  if (req->request_remove_duplicates)
  {
    ROS_INFO("kinfu: Removing duplicate points...");
    PointCloudXYZNormal::Ptr new_cloud(new PointCloudXYZNormal());
    TrianglesPtr new_triangles(new Triangles());
    removeDuplicatePoints<pcl::PointNormal>(cloud,triangles,new_cloud,new_triangles);
    cloud = new_cloud;
    triangles = new_triangles;
  }

  ROS_INFO("kinfu: Publishing...");
  pclXYZNormalCloudToMessage(cloud,resp->point_cloud,resp->normal_cloud,resp->curvature_cloud);
  resp->triangles.swap(*triangles);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Mesh Worker complete.");
}

void WorldDownloadManager::extractViewWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract View Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;

  Eigen::Affine3f view_pose = m_reverse_initial_transformation.inverse();
  if (req->request_transformation)
  {
    const Eigen::Affine3f tr = toEigenAffine(req->transformation);
    view_pose = view_pose * tr;
  }

  const bool has_intrinsics = req->request_camera_intrinsics;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  initRaycaster(has_intrinsics,req->camera_intrinsics,
    req->request_bounding_box_view,req->bounding_box_view_min,req->bounding_box_view_max);
  const uint rows = has_intrinsics ? req->camera_intrinsics.size_x : m_kinfu->rows();
  const uint cols = has_intrinsics ? req->camera_intrinsics.size_y : m_kinfu->cols();

  if (!shiftNear(view_pose,req->tsdf_center_distance))
    return; // error or shutting down

  ROS_INFO("kinfu: generating scene...");
  m_raycaster->run ( m_kinfu->volume (), view_pose, m_kinfu->getCyclicalBufferStructure () );
  m_raycaster->generateSceneView(m_view_device);

  int useless_cols;
  std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host;
  m_view_device.download (view_host, useless_cols);

  unlockKinfu();

  ROS_INFO("kinfu: Sending message...");
  sensor_msgs::fillImage(resp->image, "rgb8", rows, cols, cols * 3, &view_host[0]);
  resp->image.header.frame_id = m_reference_frame_name;

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract View Worker complete.");
}

void WorldDownloadManager::extractViewCloudWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract View Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;

  Eigen::Affine3f view_pose = m_reverse_initial_transformation.inverse();
  if (req->request_transformation)
  {
    const Eigen::Affine3f tr = toEigenAffine(req->transformation);
    view_pose = view_pose * tr;
  }

  const bool has_intrinsics = req->request_camera_intrinsics;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  initRaycaster(has_intrinsics,req->camera_intrinsics,
    req->request_bounding_box_view,req->bounding_box_view_min,req->bounding_box_view_max);
  const uint rows = has_intrinsics ? req->camera_intrinsics.size_x : m_kinfu->rows();
  const uint cols = has_intrinsics ? req->camera_intrinsics.size_y : m_kinfu->cols();
  const uint size = rows * cols;

  if (!shiftNear(view_pose,req->tsdf_center_distance))
    return; // error or shutting down

  ROS_INFO("kinfu: generating scene...");
  m_raycaster->run ( m_kinfu->volume (), view_pose, m_kinfu->getCyclicalBufferStructure (), true);

  ROS_INFO("kinfu: downloading data...");
  int useless_cols;

  std::vector<float> view_host(size * 3);
  m_raycaster->getVertexMap().download (view_host,useless_cols);

  std::vector<float> intensity_host(size);
  m_raycaster->getKnownMap().download (intensity_host,useless_cols);

  const Eigen::Vector3f cycl_o(m_kinfu->getCyclicalBufferStructure()->origin_metric.x,
                               m_kinfu->getCyclicalBufferStructure()->origin_metric.y,
                               m_kinfu->getCyclicalBufferStructure()->origin_metric.z);

  unlockKinfu();

  const bool request_bounding_box = req->request_bounding_box;
  const bool request_sphere = req->request_sphere;

  ROS_INFO("kinfu: Applying transform...");
  PointCloudXYZ::Ptr cloud(new PointCloudXYZ());
  if (!request_bounding_box && !request_sphere)
  {
    resp->float_values.resize(size);
    cloud->resize(size);
    for (uint i = 0; i < size; i++)
    {
      if (intensity_host[i] < -0.5)
      {
        (*cloud)[i].x = NAN;
        (*cloud)[i].y = NAN;
        (*cloud)[i].z = NAN;
        continue;
      }

      const Eigen::Vector3f ept(view_host[i],view_host[i + size],view_host[i + size * 2]);
      const Eigen::Vector3f tpt = m_reverse_initial_transformation * (ept + cycl_o);
      (*cloud)[i].x = tpt.x();
      (*cloud)[i].y = tpt.y();
      (*cloud)[i].z = tpt.z();
      }

    for (uint i = 0; i < size; i++)
      resp->float_values[i] = intensity_host[i];

    cloud->width = cols;
    cloud->height = rows;
    cloud->is_dense = false;
  }

  if (request_bounding_box || request_sphere)
  {
    ROS_INFO("kinfu: Applying bounding box...");
    const Eigen::Vector3f bbox_min(req->bounding_box_min.x,req->bounding_box_min.y,req->bounding_box_min.z);
    const Eigen::Vector3f bbox_max(req->bounding_box_max.x,req->bounding_box_max.y,req->bounding_box_max.z);

    const Eigen::Vector3f sphere_center(req->sphere_center.x,req->sphere_center.y,req->sphere_center.z);
    const float sphere_radius = req->sphere_radius;

    resp->float_values.reserve(size);
    cloud->reserve(size);
    for (uint i = 0; i < size; i++)
    {
      if (intensity_host[i] < -0.5 || std::isnan(intensity_host[i]))
        continue;

      const Eigen::Vector3f ept(view_host[i],view_host[i + size],view_host[i + size * 2]);
      if (std::isnan(ept.x()) || std::isnan(ept.y()) || std::isnan(ept.z()))
        continue;

      const Eigen::Vector3f tpt = m_reverse_initial_transformation * (ept + cycl_o);
      if (request_bounding_box)
        if ((tpt.array() < bbox_min.array()).any() || (tpt.array() > bbox_max.array()).any())
          continue;

      if (request_sphere)
        if ((tpt - sphere_center).squaredNorm() > sphere_radius * sphere_radius)
          continue;

      pcl::PointXYZ ppt;
      ppt.x = tpt.x(); ppt.y = tpt.y(); ppt.z = tpt.z();
      cloud->push_back(ppt);
      resp->float_values.push_back(intensity_host[i]);
      }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
  }

  ROS_INFO("kinfu: Sending message...");
  pclPointCloudToMessage(cloud,resp->point_cloud);

  resp->reference_frame_id = m_reference_frame_name;

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract View Worker complete.");
}

void WorldDownloadManager::extractVoxelCountGenericWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Voxel Count Generic Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;

  if (req->request_view_poses)
  {
    ROS_ERROR("kinfu: Extract Voxel Count Generic worker: did you mean the View version?");
    return;
  }

  const bool has_bounding_box = req->request_bounding_box;
  Eigen::Vector3f bbox_min(req->bounding_box_min.x,req->bounding_box_min.y,req->bounding_box_min.z);
  Eigen::Vector3f bbox_max(req->bounding_box_max.x,req->bounding_box_max.y,req->bounding_box_max.z);
  {
    const Eigen::Vector3f tmin = m_reverse_initial_transformation.inverse() * bbox_min;
    const Eigen::Vector3f tmax = m_reverse_initial_transformation.inverse() * bbox_max;
    bbox_min = tmin.array().min(tmax.array());
    bbox_max = tmin.array().max(tmax.array());
  }

  const bool has_sphere = req->request_sphere;
  Eigen::Vector3f sphere_center(req->sphere_center.x,req->sphere_center.y,req->sphere_center.z);
  sphere_center = m_reverse_initial_transformation.inverse() * sphere_center;
  const float sphere_radius = req->sphere_radius;

  resp->uint_values.resize(2); // [0]: occupied, [1] unknown

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  ROS_INFO("kinfu: sync points...");
  m_kinfu->syncKnownPoints();

  const float voxel_size = m_kinfu->getVoxelSize();

  TsdfCloud::Ptr points = m_kinfu->getCyclicalBuffer().getWorldModel()->getWorld();
  const uint64 size = points ? points->size() : 0;

  ROS_INFO("kinfu: counting voxels...");
  const Eigen::Vector3f expanded_min = bbox_min / voxel_size;
  const Eigen::Vector3f expanded_max = bbox_max / voxel_size;
  const Eigen::Vector3f expanded_center = sphere_center / voxel_size;
  const float expanded_radius = sphere_radius / voxel_size;
  const float sqr_expanded_radius = expanded_radius * expanded_radius;

  uint64 occupied_count = 0;
  for (uint64 i = 0; i < size; i++)
  {
    const pcl::PointXYZI & ppt((*points)[i]);
    if (ppt.intensity > 0.0)
      continue; // empty

    const Eigen::Vector3f ept(ppt.x,ppt.y,ppt.z);
    if (has_bounding_box)
      if ((ept.array() < expanded_min.array()).any() && (ept.array() > expanded_max.array()).any())
        continue;

    if (has_sphere)
      if ((ept - expanded_center).squaredNorm() > sqr_expanded_radius)
        continue;

    occupied_count++;
  }
  resp->uint_values[0] = occupied_count;

  // these count the unknown voxels
  if (has_sphere)
  {
    ROS_INFO("kinfu: counting unknown voxels (sphere)...");
    resp->uint_values[1] = m_cube_listener->CountOccupiedVoxelsInSphere(sphere_center,sphere_radius,true);
  }
  else if (has_bounding_box)
  {
    ROS_INFO("kinfu: counting unknown voxels (bounding box)...");
    resp->uint_values[1] = m_cube_listener->CountOccupiedVoxelsInBBox(bbox_min,bbox_max,true);
  }
  else
    resp->uint_values[1] = 0;

  unlockKinfu();

  ROS_INFO("kinfu: Sending message...");
  resp->reference_frame_id = m_reference_frame_name;

  m_pub.publish(resp);
}

void WorldDownloadManager::extractVoxelCountViewsWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Voxel Count Views Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse);
  resp->tsdf_header = req->tsdf_header;

  if (!req->request_view_poses)
    {
    ROS_ERROR("kinfu: Extract Voxel Count Views worker: did you mean the Generic version?");
    return;
    }

  Eigen::Affine3f center_pose = m_reverse_initial_transformation.inverse();

  ROS_INFO("kinfu: Computing poses...");
  std::vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > view_poses;
  if (req->request_view_poses)
  {
    view_poses.resize(req->view_poses.size());
    for (uint i = 0; i < view_poses.size(); i++)
    {
      const Eigen::Affine3f p = toEigenAffine(req->view_poses[i]);
      view_poses[i] = center_pose * p;
    }
  }

  if (req->request_transformation)
  {
    const Eigen::Affine3f tr = toEigenAffine(req->transformation);
    center_pose = center_pose * tr;
  }

  const bool has_intrinsics = req->request_camera_intrinsics;

  resp->uint_values.resize(view_poses.size() * 2,0);

  const bool has_bounding_box = req->request_bounding_box;
  Eigen::Vector3f bbox_min(req->bounding_box_min.x,req->bounding_box_min.y,req->bounding_box_min.z);
  Eigen::Vector3f bbox_max(req->bounding_box_max.x,req->bounding_box_max.y,req->bounding_box_max.z);

  const bool has_sphere = req->request_sphere;
  Eigen::Vector3f sphere_center(req->sphere_center.x,req->sphere_center.y,req->sphere_center.z);
  const float sphere_radius = req->sphere_radius;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  const uint rows = has_intrinsics ? req->camera_intrinsics.size_x : m_kinfu->rows();
  const uint cols = has_intrinsics ? req->camera_intrinsics.size_y : m_kinfu->cols();
  const uint size = rows * cols;

  const Eigen::Vector3f cycl_o(m_kinfu->getCyclicalBufferStructure()->origin_metric.x,
                               m_kinfu->getCyclicalBufferStructure()->origin_metric.y,
                               m_kinfu->getCyclicalBufferStructure()->origin_metric.z);

  if (has_bounding_box)
  {
    Eigen::Vector3f t_bbox_min = m_reverse_initial_transformation.inverse() * bbox_min - cycl_o;
    Eigen::Vector3f t_bbox_max = m_reverse_initial_transformation.inverse() * bbox_max - cycl_o;
    bbox_min = t_bbox_min.array().min(t_bbox_max.array());
    bbox_max = t_bbox_min.array().max(t_bbox_max.array());
    bbox_min -= Eigen::Vector3f::Ones() * VOXEL_EPSILON;
    bbox_min += Eigen::Vector3f::Ones() * VOXEL_EPSILON;
  }
  if (has_sphere)
    sphere_center = m_reverse_initial_transformation.inverse() * sphere_center - cycl_o;

  initRaycaster(has_intrinsics,req->camera_intrinsics,
    req->request_bounding_box_view,req->bounding_box_view_min,req->bounding_box_view_max);

  if (!shiftNear(center_pose,req->tsdf_center_distance))
    return; // error or shutting down

  ROS_INFO("kinfu: evaluating poses...");
  for (uint i = 0; i < view_poses.size(); i++)
  {
    m_raycaster->run ( m_kinfu->volume (), view_poses[i], m_kinfu->getCyclicalBufferStructure (), true);

    int useless_cols;
    std::vector<float> intensity_host(size);
    m_raycaster->getKnownMap().download (intensity_host,useless_cols);

    std::vector<float> view_host;

    if (has_bounding_box || has_sphere)
    {
      view_host.resize(size * 3);
      m_raycaster->getVertexMap().download (view_host,useless_cols);
    }

    const float sqr_sphere_radius = SQR(sphere_radius + VOXEL_EPSILON);

    for (uint h = 0; h < size; h++)
    {
      if (std::isnan(intensity_host[h]) || intensity_host[h] < -0.5)
        continue;

      if (has_bounding_box || has_sphere)
      {
        const Eigen::Vector3f ept(view_host[h],view_host[h + size],view_host[h + size * 2]);
        if (std::isnan(ept.x()) || std::isnan(ept.y()) || std::isnan(ept.z()))
          continue;

        if (has_bounding_box)
          if ((ept.array() < bbox_min.array()).any() || (ept.array() > bbox_max.array()).any())
            continue;

        if (has_sphere)
          if ((ept - sphere_center).squaredNorm() > sqr_sphere_radius)
            continue;
      }

      if (intensity_host[h] > 0.5)
        resp->uint_values[i * 2] += 1;       // occupied
      else
        resp->uint_values[(i * 2) + 1] += 1; // unknown
    }
  }

  unlockKinfu();

  ROS_INFO("kinfu: Sending message...");
  resp->reference_frame_id = m_reference_frame_name;

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Voxel Count complete.");
}

void WorldDownloadManager::extractVoxelGridWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  ROS_INFO("kinfu: Extract Voxel Grid Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;

  const Eigen::Vector3f bbox_min(req->bounding_box_min.x,req->bounding_box_min.y,req->bounding_box_min.z);
  const Eigen::Vector3f bbox_max(req->bounding_box_max.x,req->bounding_box_max.y,req->bounding_box_max.z);

  const bool has_subsample = req->request_subsample;
  const float subsample_voxel_size = has_subsample ? req->subsample_voxel_size : 0.0;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  ROS_INFO("kinfu: sync points...");
  m_kinfu->syncKnownPoints();

  const float kinfu_voxel_size = m_kinfu->getVoxelSize();
  const float voxel_size = std::max(kinfu_voxel_size,subsample_voxel_size);

  const Eigen::Vector3i voxel_count = ((bbox_max - bbox_min) / voxel_size + Eigen::Vector3f::Ones() * 0.5).cast<int>();
  if ((voxel_count.array() < 1).any())
  {
    ROS_ERROR("kinfu: extractVoxelGridWorker: invalid bounding box.");
    unlockKinfu();
    return;
  }

  const uint64 grid_x_step = 1;
  const uint64 grid_y_step = grid_x_step * voxel_count.x();
  const uint64 grid_z_step = grid_y_step * voxel_count.y();
  const uint64 grid_size = grid_z_step * voxel_count.z();

  std::vector<float> voxelgrid(grid_size,0.0); // initialize all unknown

  TsdfCloud::Ptr points = m_kinfu->getCyclicalBuffer().getWorldModel()->getWorld();
  const uint64 world_size = points ? points->size() : 0;

  ROS_INFO("kinfu: extracting points from world model (world size is %lu)...",(long unsigned int)world_size);

  for (uint64 i = 0; i < world_size; i++)
  {
    const pcl::PointXYZI & ppt((*points)[i]);
    const Eigen::Vector3f ept(ppt.x,ppt.y,ppt.z);

    const Eigen::Vector3f point = m_reverse_initial_transformation * (ept * kinfu_voxel_size);
    const Eigen::Vector3i ipoint = ((point - bbox_min) / voxel_size + Eigen::Vector3f::Ones() * 0.5).cast<int>();

    if ((ipoint.array() < 0).any() || (ipoint.array() >= voxel_count.array()).any())
      continue; // outside bounding box

    const uint64 addr = ipoint.x() * grid_x_step + ipoint.y() * grid_y_step + ipoint.z() * grid_z_step;

    if (ppt.intensity < 0.0)
      voxelgrid[addr] = 1.0; // occupied

    if (ppt.intensity > 0.0 && voxelgrid[addr] < 0.5) // do not override occupied
      voxelgrid[addr] = -1.0; // empty
  }

  ROS_INFO("kinfu: extracting empty voxels from knowledge octree...");
  {
    PointCloudXYZ::Ptr centers = m_cube_listener->GetOccupiedVoxelCenters();
    uint64 centers_size = centers->size();
    for (uint64 i = 0; i < centers_size; i++)
    {
      const pcl::PointXYZ & ppt((*centers)[i]);
      const Eigen::Vector3f ept(ppt.x,ppt.y,ppt.z);

      const Eigen::Vector3f point = m_reverse_initial_transformation * ept;
      const Eigen::Vector3i ipoint = ((point - bbox_min) / voxel_size + Eigen::Vector3f::Ones() * 0.5).cast<int>();

      if ((ipoint.array() < 0).any() || (ipoint.array() >= voxel_count.array()).any())
        continue; // outside bounding box

      const uint64 addr = ipoint.x() * grid_x_step + ipoint.y() * grid_y_step + ipoint.z() * grid_z_step;

      if (voxelgrid[addr] > -0.5 && voxelgrid[addr] < 0.5) // do not override occupied
        voxelgrid[addr] = -1.0; // empty
    }
  }

  unlockKinfu();

  ROS_INFO("kinfu: Sending message...");
  resp->reference_frame_id = m_reference_frame_name;
  resp->float_values = voxelgrid;
  resp->uint_values.resize(3);
  for (uint i = 0; i < 3; i++)
    resp->uint_values[i] = voxel_count[i];

  m_pub.publish(resp);
}

void WorldDownloadManager::extractBorderPointsWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req)
{
  const bool edges_only = uint(req->tsdf_header.request_type) == uint(req->tsdf_header.REQUEST_TYPE_GET_BORDER_POINTS);

  ROS_INFO("kinfu: Extract Border Points Worker started.");
  // prepare with same header
  kinfu_msgs::KinfuTsdfResponsePtr resp(new kinfu_msgs::KinfuTsdfResponse());
  resp->tsdf_header = req->tsdf_header;

  Eigen::Affine3f center_pose = m_reverse_initial_transformation.inverse();
  if (req->request_transformation)
  {
    const Eigen::Affine3f tr = toEigenAffine(req->transformation);
    center_pose = center_pose * tr;
  }

  const bool req_bbox = req->request_bounding_box;
  const Eigen::Vector3f req_bbox_min(req->bounding_box_min.x,req->bounding_box_min.y,req->bounding_box_min.z);
  const Eigen::Vector3f req_bbox_max(req->bounding_box_max.x,req->bounding_box_max.y,req->bounding_box_max.z);

  const bool req_sphere = req->request_sphere;
  const Eigen::Vector3f req_sphere_center(req->sphere_center.x,req->sphere_center.y,req->sphere_center.z);
  const float req_sphere_radius = req->sphere_radius;

  ROS_INFO("kinfu: Locking kinfu...");
  if (!lockKinfu())
    return; // shutting down or synchronization error

  if (!m_kinfu->isShiftComplete())
  {
    ROS_INFO("kinfu: waiting for current shift to terminate...");
    while (!m_kinfu->isShiftComplete())
      m_kinfu->updateShift();
  }
  m_kinfu->syncKnownPoints();

  PointCloudXYZNormal::ConstPtr cloud;
  if (edges_only)
    cloud = m_incomplete_points_listener->GetBorderCloud();
  else
    cloud = m_incomplete_points_listener->GetFrontierCloud();

  Eigen::Affine3f scale_down = Eigen::Affine3f::Identity();
  scale_down.linear() *= m_kinfu->getVoxelSize();

  unlockKinfu();

  if (!cloud)
  {
    ROS_ERROR("kinfu: incomplete point listener returned a NULL cloud, is incomplete point extraction enabled?");
    return;
  }

  PointCloudXYZNormal::Ptr total_cloud(new PointCloudXYZNormal);
  pcl::transformPointCloud(*cloud,*total_cloud,scale_down);

  ROS_INFO("kinfu: extracted %u incomplete points.",uint(total_cloud->size()));

  pcl::transformPointCloudWithNormals(*total_cloud,*total_cloud,center_pose.inverse());
  if (req_bbox) // refine the bounding box
  {
    ROS_INFO("kinfu: applying bounding box...");
    cropCloud<pcl::PointNormal>(req_bbox_min,req_bbox_max,total_cloud,total_cloud);
  }
  if (req_sphere)
  {
    ROS_INFO("kinfu: applying sphere...");
    cropCloudWithSphere<pcl::PointNormal>(req_sphere_center,req_sphere_radius,total_cloud,total_cloud);
  }

  ROS_INFO("kinfu: Sending message...");
  resp->reference_frame_id = m_reference_frame_name;
  pclXYZNormalCloudToMessage(total_cloud,resp->point_cloud,resp->normal_cloud,resp->curvature_cloud);

  m_pub.publish(resp);
}
