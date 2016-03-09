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

#ifndef KINFU_OUTPUT_ACTION_CONVERSIONS_H
#define KINFU_OUTPUT_ACTION_CONVERSIONS_H

#include <kinfu_msgs/KinfuTsdfResponse.h>
#include <kinfu_msgs/RequestAction.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/Image.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <string>
#include <stdint.h>

void ConvertToActionResponse(const kinfu_msgs::KinfuTsdfResponse &response, kinfu_msgs::RequestResult &result);

void ConvertTsdfToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data);
void ConvertCloudToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data);
void ConvertMeshToMeshMsg(const kinfu_msgs::KinfuTsdfResponse & resp,pcl_msgs::PolygonMesh & data);
void ConvertIntensityCloudToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data);
void ConvertXYZNormalCloudToPointCloud2(const kinfu_msgs::KinfuTsdfResponse & resp,sensor_msgs::PointCloud2 & data);
void ConvertUintArrayToMsg(const kinfu_msgs::KinfuTsdfResponse & resp,std_msgs::UInt64MultiArray &data,
  const std::vector<std::string> & dims,const std::vector<uint64_t> & sizes);
void ConvertGridToMsg(const kinfu_msgs::KinfuTsdfResponse & resp,std_msgs::Float32MultiArray & data);

#endif // KINFU_OUTPUT_ACTION_CONVERSIONS_H
