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

#ifndef WORLDDOWNLOADMANAGER_H
#define WORLDDOWNLOADMANAGER_H

// STL
#include <iostream>
#include <vector>
#include <list>
#include <stdint.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// Custom
#include "parameters.h"
#include "weightcubelistener.h"
#include "incompletepointslistener.h"
#include "kinfu_output_request_manager.h"

// ROS custom messages
#include <kinfu_msgs/RequestAction.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>
#include <kinfu_msgs/KinfuCameraIntrinsics.h>

class WorldDownloadManager: private KinfuOutputIAnswerer
{
  public:
  typedef boost::shared_ptr<ros::Publisher> PublisherPtr;
  typedef pcl::gpu::kinfuLS::KinfuTracker KinfuTracker;
  typedef unsigned int uint;
  typedef uint64_t uint64;
  typedef pcl::PolygonMesh Mesh;
  typedef pcl::PointNormal PointXYZNormal;
  typedef pcl::PointCloud<pcl::PointXYZI> TsdfCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
  typedef pcl::PointCloud<pcl::PointNormal> PointCloudXYZNormal;
  typedef pcl::gpu::kinfuLS::RayCaster RayCaster;
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i> > Vector4iVector;
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > Vector3iVector;
  typedef std::vector<uint64> UInt64Vector;
  typedef std::vector<bool> BoolVector;
  struct Triangle
  {
    uint64 & operator[](const std::size_t index) {return indices[index]; }
    uint64 operator[](const std::size_t index) const {return indices[index]; }

    pcl_msgs::Vertices toVertices() const
    {
      pcl_msgs::Vertices v;
      v.vertices.resize(3);
      for (uint64 index = 0; index < 3; index++)
        v.vertices[index] = indices[index];
      return v;
    }

    private:
    uint64 indices[3];
  };
  typedef std::vector<Triangle> TriangleVector;
  typedef boost::shared_ptr<TriangleVector> TriangleVectorPtr;
  typedef boost::shared_ptr<const TriangleVector> TriangleVectorConstPtr;

  WorldDownloadManager(ros::NodeHandle &nhandle,boost::mutex &shared_mutex,boost::condition_variable & cond);

  ~WorldDownloadManager();

  // this must be called by the kinfu thread when the kinfu is ready
  void respond(KinfuTracker * kinfu);

  bool hasRequests();

  void setReverseInitialTransformation(Eigen::Affine3f t) {m_reverse_initial_transformation = t; }

  void setReferenceFrameName(std::string n) {m_reference_frame_name = n; }

  void setMarchingCubesVolumeSize(int size) {m_marching_cubes_volume_size = size; }

  TWeightCubeListener::Ptr getWeightCubeListener() {return m_cube_listener; }

  TIncompletePointsListener::Ptr getIncompletePointsListener() {return m_incomplete_points_listener; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:

  // called by the request manager on request
  void requestCallback(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void requestWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req,
    std::list<boost::thread *>::iterator thread_iterator);

  // PING
  void pingWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  // TSDF
  void extractTsdfWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);
  static void cropTsdfCloud(const TsdfCloud & in,TsdfCloud & out,
    const kinfu_msgs::KinfuCloudPoint & min,const kinfu_msgs::KinfuCloudPoint & max);

  void extractCloudWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);
  void extractCloudWorkerMCWithNormals(kinfu_msgs::KinfuTsdfRequestConstPtr req,
    kinfu_msgs::RequestResultPtr resp, TsdfCloud::Ptr tsdf_cloud, const float scale);

  template <class PointT>
  static void separateMesh(Mesh::ConstPtr mesh,typename pcl::PointCloud<PointT>::Ptr points,
    TriangleVectorPtr triangles = TriangleVectorPtr());

  template <class PointT>
  static void mergePointCloudsAndMesh(std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointclouds,
    typename pcl::PointCloud<PointT>::Ptr out_cloud,
    std::vector<TriangleVectorPtr> *meshes = NULL,
    TriangleVector *out_mesh = NULL);

  void extractMeshWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractKnownWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractViewWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractViewCloudWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractVoxelCountViewsWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);
  void extractVoxelCountGenericWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractVoxelGridWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  void extractBorderPointsWorker(kinfu_msgs::KinfuTsdfRequestConstPtr req);

  bool marchingCubes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const float scale, std::vector<Mesh::Ptr> & output_meshes) const;

  // returns false on error
  bool lockKinfu();

  void unlockKinfu();

  bool shiftNear(const Eigen::Affine3f & pose, const float distance, const float distance_threshold); // false on error

  void initRaycaster(bool has_intrinsics, const kinfu_msgs::KinfuCameraIntrinsics & intr,
    bool has_bounding_box_view,const kinfu_msgs::KinfuCloudPoint & bbox_min,const kinfu_msgs::KinfuCloudPoint & bbox_max);

  template <class PointT>
  static void cropMesh(const kinfu_msgs::KinfuCloudPoint & min,
    const kinfu_msgs::KinfuCloudPoint & max,typename pcl::PointCloud<PointT>::ConstPtr cloud,
    TriangleVectorConstPtr triangles,typename pcl::PointCloud<PointT>::Ptr out_cloud,TriangleVectorPtr out_triangles);

  template <class PointT>
  static void removeDuplicatePoints(typename pcl::PointCloud<PointT>::ConstPtr cloud,
    TriangleVectorConstPtr triangles,typename pcl::PointCloud<PointT>::Ptr out_cloud,TriangleVectorPtr out_triangles);

  template <class PointT>
  static void cropCloud(const Eigen::Vector3f & min,const Eigen::Vector3f & max,
    typename pcl::PointCloud<PointT>::ConstPtr cloud,typename pcl::PointCloud<PointT>::Ptr out_cloud);

  template <class PointT>
  static void cropCloudWithSphere(const Eigen::Vector3f & sphere_center,const float sphere_radius,
    typename pcl::PointCloud<PointT>::ConstPtr cloud,typename pcl::PointCloud<PointT>::Ptr out_cloud);

  // many kinds of array and vectors may be used here
  // (the main reason being that ROS uses std::vector
  // OR ros::array, "if available")
  template<typename T1,typename T2>
    static Eigen::Affine3f toEigenAffine(const T1 & linear,const T2 & translation);

  static Eigen::Affine3f toEigenAffine(const kinfu_msgs::KinfuPose & pose);

  // transforms the bounding box, and generates a bigger axis oriented bounding box, if needed
  static void transformBoundingBoxAndExpand(const Eigen::Vector3f& bbox_min,const Eigen::Vector3f& bbox_max,
    const Eigen::Affine3f& transform,Eigen::Vector3f& bbox_min_out,Eigen::Vector3f& bbox_max_out);

  static void findExtraCubesForBoundingBox(const Eigen::Vector3f& current_cube_min, const Eigen::Vector3f& current_cube_max,
    const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max, Vector3fVector& cubes_centers, bool& extract_current);
  static Eigen::Vector3f floor3f(const Eigen::Vector3f & v);

  ros::NodeHandle &m_nh;

  RayCaster::Ptr m_raycaster;
  KinfuTracker::View m_view_device;

  boost::mutex &m_shared_mutex;
  boost::condition_variable & m_shared_cond;

  uint m_kinfu_waiting_count;
  bool m_kinfu_available;
  boost::mutex m_kinfu_waiting_mutex;
  boost::condition_variable m_kinfu_waiting_cond;

  bool m_is_shutting_down;

  int m_marching_cubes_volume_size;

  KinfuTracker * m_kinfu;

  boost::mutex m_thread_mutex;
  boost::condition_variable m_thread_cond;

  std::list<boost::thread *> m_threads;
  std::vector<boost::thread *> m_terminated_threads;

  Eigen::Affine3f m_reverse_initial_transformation;

  TWeightCubeListener::Ptr m_cube_listener;

  TIncompletePointsListener::Ptr m_incomplete_points_listener;

  std::string m_reference_frame_name;

  RequestManager m_request_manager;
};

#endif // WORLDDOWNLOADMANAGER_H
