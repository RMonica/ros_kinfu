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

void WorldDownloadManager::requestWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req,
  std::list<boost::thread *>::iterator thread_iterator)
{
  if (req->tsdf_header.request_type == req->tsdf_header.REQUEST_TYPE_PING)
    pingWorker(req);
  else if (req->tsdf_header.request_type == req->tsdf_header.REQUEST_TYPE_GET_TSDF)
    extractTsdfWorker(req);
  else if (req->tsdf_header.request_type == req->tsdf_header.REQUEST_TYPE_GET_MESH)
    extractMeshWorker(req);
  else if (req->tsdf_header.request_type == req->tsdf_header.REQUEST_TYPE_GET_CLOUD)
    extractCloudWorker(req);
  else if (req->tsdf_header.request_type == req->tsdf_header.REQUEST_TYPE_GET_KNOWN)
    extractKnownWorker(req);
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

void WorldDownloadManager::fromTsdfToMessage(const TsdfCloud & in,kinfu_msgs::KinfuTsdfResponse::Ptr & resp)
{
  resp->tsdf_cloud.clear();

  uint in_size = in.size();

  resp->tsdf_cloud.reserve(in_size);

  for (uint i = 0; i < in_size; i++)
  {
    const pcl::PointXYZI & s = in[i];

    kinfu_msgs::KinfuTsdfPoint d;

    d.x = s.x;
    d.y = s.y;
    d.z = s.z;
    d.i = s.intensity;

    resp->tsdf_cloud.push_back(d);
  }
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr kinfu_cloud = req->request_reset ?
    m_kinfu->extractWorldAndReset() : m_kinfu->extractWorld();

  unlockKinfu();

  Eigen::Affine3f transformation = m_reverse_initial_transformation;

  std::vector<Mesh::Ptr> meshes;

  ROS_INFO("kinfu: Marching cubes...");
  if (!marchingCubes(kinfu_cloud,meshes))
    return; // ERROR

  kinfu_cloud->clear(); // save some memory

  std::vector<PointCloud::Ptr> clouds;

  ROS_INFO("kinfu: Extracting only points from mesh...");
  for (uint i = 0; i < meshes.size(); i++)
  {
    clouds.push_back(PointCloud::Ptr(new PointCloud()));
    separateMesh(meshes[i],clouds[i]);
  }
  meshes.clear(); // save some memory

  PointCloud::Ptr cloud(new PointCloud());

  ROS_INFO("kinfu: Merging points...");
  mergePointCloudsAndMesh(clouds,cloud);

  // if needed, transform
  if (req->request_transformation)
  {
    ROS_INFO("kinfu: A custom transformation will be applied.");
    Eigen::Affine3f tr = toEigenAffine(req->transformation_linear,req->transformation_translation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    PointCloud::Ptr out_cloud(new PointCloud());
    TrianglesPtr empty(new Triangles());
    cropMesh(req->bounding_box_min,req->bounding_box_max,cloud,empty,out_cloud,empty);
    cloud = out_cloud;
  }

  ROS_INFO("kinfu: Publishing...");
  pclPointCloudToMessage(cloud,resp->point_cloud);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Cloud Worker complete.");
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
    Eigen::Affine3f tr = toEigenAffine(req->transformation_linear,req->transformation_translation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    PointCloud::Ptr out_cloud(new PointCloud());
    TrianglesPtr empty(new Triangles());
    cropMesh(req->bounding_box_min,req->bounding_box_max,cloud,empty,out_cloud,empty);
    cloud = out_cloud;
  }

  ROS_INFO("kinfu: Publishing...");
  pclPointCloudToMessage(cloud,resp->point_cloud);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Known Worker complete.");
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

  std::vector<PointCloud::Ptr> clouds;
  std::vector<TrianglesPtr> clouds_triangles;

  ROS_INFO("kinfu: Divide triangles and points...");
  for (uint i = 0; i < meshes.size(); i++)
  {
    clouds.push_back(PointCloud::Ptr(new PointCloud()));
    clouds_triangles.push_back(TrianglesPtr(new Triangles()));

    separateMesh(meshes[i],clouds[i],clouds_triangles[i]);
  }
  meshes.clear(); // save some memory

  TrianglesPtr triangles(new Triangles());
  PointCloud::Ptr cloud(new PointCloud());

  ROS_INFO("kinfu: Merging points...");
  mergePointCloudsAndMesh(clouds,cloud,&clouds_triangles,&(*triangles));

  // if needed, transform
  if (req->request_transformation)
  {
    ROS_INFO("kinfu: A custom transformation will be applied.");
    Eigen::Affine3f tr = toEigenAffine(req->transformation_linear,req->transformation_translation);
    transformation = tr * transformation;
  }

  ROS_INFO("kinfu: Applying transformation...");
  pcl::transformPointCloud(*cloud,*cloud,transformation);

  // if needed, crop
  if (req->request_bounding_box)
  {
    ROS_INFO("kinfu: Cropping...");
    TrianglesPtr out_triangles(new Triangles());
    PointCloud::Ptr out_cloud(new PointCloud());
    cropMesh(req->bounding_box_min,req->bounding_box_max,cloud,triangles,out_cloud,out_triangles);
    cloud = out_cloud;
    triangles = out_triangles;
  }

  ROS_INFO("kinfu: Publishing...");
  pclPointCloudToMessage(cloud,resp->point_cloud);
  resp->triangles.swap(*triangles);

  m_pub.publish(resp);
  ROS_INFO("kinfu: Extract Mesh Worker complete.");
}

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
    std::vector<Eigen::Vector3f> transforms;

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
  Eigen::Affine3f WorldDownloadManager::toEigenAffine(T1 linear,T2 translation)
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
