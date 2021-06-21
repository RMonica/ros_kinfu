/*
 * Copyright (c) 2013-2016, Riccardo Monica
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

#ifndef KINFU_OUTPUT_REQUEST_MANAGER_H
#define KINFU_OUTPUT_REQUEST_MANAGER_H

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/Image.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

// ROS custom messages
#include <kinfu_msgs/RequestAction.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>

// custom
#include "parameters.h"
#include "kinfu_output_ianswerer.h"
#include "kinfu_output_save_file.h"

class RequestActionManager;

class RequestManager
{
  public:
  class IQueuedResponse
  {
    public:
    virtual void Update() = 0;
    virtual bool IsEnded() = 0;
    virtual ~IQueuedResponse() {}
  };
  typedef boost::shared_ptr<IQueuedResponse> PQueuedResponse;
  typedef boost::shared_ptr<pcl_msgs::PolygonMesh> MeshPtr;
  typedef std::list<PQueuedResponse> PQueuedResponseList;

  typedef boost::shared_ptr<RequestActionManager> RequestActionManagerPtr;

  RequestManager(ros::NodeHandle & nh,KinfuOutputIAnswerer & answerer);
  ~RequestManager();

  void SendResponse(const kinfu_msgs::RequestResultPtr & response_ptr);

  private:
  void requestCallback(const kinfu_msgs::KinfuTsdfRequestConstPtr & request);
  void run();
  void Update();

  ros::NodeHandle & m_nh;

  ros::Subscriber m_req_sub;
  KinfuOutputIAnswerer & m_answerer;

  KinfuOutputSaveFile m_save_file;

  boost::mutex m_mutex;
  boost::thread m_update_thread;
  bool m_is_terminating;

  std::list<PQueuedResponse> m_queue;

  RequestActionManagerPtr m_ramgr;
};

#endif // KINFU_OUTPUT_REQUEST_MANAGER_H
