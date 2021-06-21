/*
 * Copyright (c) 2020, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
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

#include "kinfu_output_save_file.h"

#include <cstdio>
#include <string>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "parameters.h"

KinfuOutputSaveFile::KinfuOutputSaveFile(ros::NodeHandle & nh): m_nh(nh)
{
  m_nh.param<std::string>(PARAM_NAME_TEMPORARY_FILE_DIR, m_temp_file_dir, PARAM_DEFAULT_TEMPORARY_FILE_DIR);
}

bool KinfuOutputSaveFile::IsLargeResponse(const kinfu_msgs::RequestResult & response) const
{
  // messages longer than 1 GB are not supported by ROS
  // use 900 MB to be safe
  return (ros::serialization::serializationLength(response) > 900 * 1000 * 1000);
}

std::string KinfuOutputSaveFile::SaveFile(kinfu_msgs::RequestResult & response)
{
  bool pointcloud = false;
  bool mesh = false;
  bool float_values = false;

  std::string filename;

  if (!m_temp_file_dir.empty())
  {
    char * c_filename = tempnam(m_temp_file_dir.c_str(), NULL);
    filename = c_filename;
    free(c_filename);
  }
  else
  {
    filename = std::tmpnam(NULL);
  }

  switch (response.tsdf_header.request_type)
  {
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_TSDF:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_CLOUD:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_KNOWN:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_VIEW_CLOUD:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_FRONTIER_POINTS:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_BORDER_POINTS:
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_SURFACE_POINTS:
      pointcloud = true;
      filename += ".pcd";
      break;
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_MESH:
      mesh = true;
      filename += ".ply";
      break;
    case kinfu_msgs::KinfuRequestHeader::REQUEST_TYPE_GET_VOXELGRID:
      float_values = true;
      filename += ".dat";
      break;
    default:
      ROS_ERROR("kinfu_output: could not save file for response type %u.",uint(response.tsdf_header.request_type));
      return "";
  }

  if (pointcloud)
  {
    pcl::PCLPointCloud2 pcloud;
    pcl_conversions::toPCL(response.pointcloud, pcloud);

    ROS_INFO("kinfu_output: saving file %s", filename.c_str());
    if (!pcloud.data.empty())
    {
      if (pcl::io::savePCDFile(filename, pcloud, Eigen::Vector4f::Zero(),
                               Eigen::Quaternionf::Identity(), true))
      {
        ROS_ERROR("kinfu_output: could not save file.");
        return "";
      }
    }
    else
      ROS_WARN("kinfu_output: file not saved: point cloud is empty.");

    response.pointcloud = sensor_msgs::PointCloud2();
  }

  if (mesh)
  {
    pcl::PolygonMesh pmesh;
    pcl_conversions::toPCL(response.mesh, pmesh);

    ROS_INFO("kinfu_output: saving file %s", filename.c_str());
    if (pcl::io::savePLYFileBinary(filename, pmesh))
    {
      ROS_ERROR("kinfu_output: could not save file.");
      return "";
    }

    response.mesh = pcl_msgs::PolygonMesh();
  }

  if (float_values)
  {
    ROS_INFO("kinfu_output: saving file %s", filename.c_str());
    std::ofstream ofile(filename.c_str(), std::ios::binary);
    ofile.write((const char *)(response.float_values.data.data()), response.float_values.data.size() *
                sizeof(float));

    if (!ofile)
    {
      ROS_ERROR("kinfu_output: could not save file.");
      return "";
    }

    response.float_values.data.clear();
  }

  return filename;
}

