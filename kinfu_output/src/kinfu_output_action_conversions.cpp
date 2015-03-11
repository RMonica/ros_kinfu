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

#include "kinfu_output_action_conversions.h"

typedef unsigned int uint;
typedef uint64_t uint64;

void ConvertTsdfToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data)
  {
  uint cloud_size = resp.tsdf_cloud.size();
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  pointcloud.reserve(cloud_size);
  for (uint i = 0; i < cloud_size; i++)
    {
    pcl::PointXYZI p;
    const kinfu_msgs::KinfuTsdfPoint & tp = resp.tsdf_cloud[i];

    p.x = tp.x;
    p.y = tp.y;
    p.z = tp.z;
    p.intensity = tp.i;

    pointcloud.push_back(p);
    }

  pcl::toROSMsg(pointcloud,data);

  data.header.seq = resp.tsdf_header.request_id;
  data.header.stamp = ros::Time::now();
  data.header.frame_id = resp.reference_frame_id;
  }

void ConvertCloudToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data)
  {
  uint cloud_size = resp.point_cloud.size();
  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  pointcloud.reserve(cloud_size);
  for (uint i = 0; i < cloud_size; i++)
    {
    pcl::PointXYZ p;
    const kinfu_msgs::KinfuCloudPoint & tp = resp.point_cloud[i];

    p.x = tp.x;
    p.y = tp.y;
    p.z = tp.z;

    pointcloud.push_back(p);
    }

  pcl::toROSMsg(pointcloud,data);

  data.header.seq = resp.tsdf_header.request_id;
  data.header.stamp = ros::Time::now();
  data.header.frame_id = resp.reference_frame_id;
  }

void ConvertMeshToMeshMsg(const kinfu_msgs::KinfuTsdfResponse & resp, pcl_msgs::PolygonMesh &data)
  {
  uint points_size = resp.point_cloud.size();
  uint triangles_size = resp.triangles.size();
  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  pointcloud.reserve(points_size);
  for (uint i = 0; i < points_size; i++)
    {
    pcl::PointXYZ p;
    const kinfu_msgs::KinfuCloudPoint & tp = resp.point_cloud[i];

    p.x = tp.x;
    p.y = tp.y;
      p.z = tp.z;

    pointcloud.push_back(p);
    }

  pcl::toROSMsg(pointcloud,data.cloud);

  data.polygons.reserve(triangles_size);
  for (uint i = 0; i < triangles_size; i++)
    {
    pcl_msgs::Vertices v;
    const kinfu_msgs::KinfuMeshTriangle & t = resp.triangles[i];

    v.vertices.resize(3);
    for (uint h = 0; h < 3; h++)
      v.vertices[h] = t.vertex_id[h];

    data.polygons.push_back(v);
    }

  data.header.seq = resp.tsdf_header.request_id;
  data.header.stamp = ros::Time::now();
  data.header.frame_id = resp.reference_frame_id;
  }

void ConvertIntensityCloudToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data)
  {
  uint cloud_size = resp.point_cloud.size();
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  pointcloud.reserve(cloud_size);
  for (uint i = 0; i < cloud_size; i++)
    {
    pcl::PointXYZI p;
    const kinfu_msgs::KinfuCloudPoint & tp = resp.point_cloud[i];

    p.x = tp.x;
    p.y = tp.y;
    p.z = tp.z;
    p.intensity = resp.float_values[i];

    pointcloud.push_back(p);
    }

  pcl::toROSMsg(pointcloud,data);

  data.header.seq = resp.tsdf_header.request_id;
  data.header.stamp = ros::Time::now();
  data.header.frame_id = resp.reference_frame_id;
  }

void ConvertUintArrayToMsg(const kinfu_msgs::KinfuTsdfResponse & resp, std_msgs::UInt64MultiArray & data,
  const std::vector<std::string> &dims, const std::vector<uint64> &sizes)
  {
  data.data = resp.uint_values;
  data.layout.data_offset = 0;

  uint stride_inc = 1;
  for (uint i = 0; i < dims.size(); i++)
    {
    std_msgs::MultiArrayDimension dim;
    dim.label = dims[i];
    dim.size = sizes[i];
    dim.stride = stride_inc;
    stride_inc *= sizes[i];
    data.layout.dim.push_back(dim);
    }
  }

void ConvertGridToMsg(const kinfu_msgs::KinfuTsdfResponse & resp,std_msgs::Float32MultiArray & data)
  {
  if (resp.uint_values.size() != 3)
    {
    ROS_ERROR("kinfu_output: malformed REQUEST_TYPE_GET_VOXELGRID response, expected 3 uint_values!");
    return;
    }

  static const char * const dims[3] = {"x","y","z"};

  uint64 sizes[3];
  for (uint i = 0; i < 3; i++)
    sizes[i] = resp.uint_values[i];
  const uint64 size = sizes[0] * sizes[1] * sizes[2];
  const uint64 actual_size = resp.float_values.size();
  if (size != actual_size)
    {
    ROS_ERROR("kinfu_output: malformed REQUEST_TYPE_GET_VOXELGRID response, expected size is %u, but actual size is %u",
      uint(size),uint(actual_size));
    return;
    }

  data.data = resp.float_values;
  data.layout.data_offset = 0;

  uint stride_inc = 1;
  for (uint i = 0; i < 3; i++)
    {
    std_msgs::MultiArrayDimension dim;
    dim.label = dims[i];
    dim.size = sizes[i];
    dim.stride = stride_inc;
    stride_inc *= sizes[i];
    data.layout.dim.push_back(dim);
    }
  }

void ConvertToActionResponse(const kinfu_msgs::KinfuTsdfResponse & response,kinfu_msgs::RequestResult & result)
  {
  if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_PING)
    {
    result.header.seq = response.tsdf_header.request_id;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = response.reference_frame_id;
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_TSDF)
    {
    ConvertTsdfToPointCloud2(response,result.pointcloud);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_MESH)
    {
    ConvertMeshToMeshMsg(response,result.mesh);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_CLOUD)
    {
    ConvertCloudToPointCloud2(response,result.pointcloud);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_KNOWN)
    {
    ConvertCloudToPointCloud2(response,result.pointcloud);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VIEW)
    {
    result.image = response.image;
    result.image.header.stamp = ros::Time::now();
    result.image.header.seq = response.tsdf_header.request_id;
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VIEW_CLOUD)
    {
    ConvertIntensityCloudToPointCloud2(response,result.pointcloud);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VOXEL_COUNT)
    {
    std::vector<std::string> dims;
    dims.push_back("Voxel type");
    dims.push_back("View number");
    std::vector<uint64> sizes;
    sizes.push_back(2);
    sizes.push_back(response.uint_values.size() / 2);
    ConvertUintArrayToMsg(response,result.uint_values,dims,sizes);
    }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VOXELGRID)
    {
    ConvertGridToMsg(response,result.float_values);
    }
  else
    ROS_ERROR("kinfu_output: unknown request type %u.",uint(response.tsdf_header.request_type));
  }
