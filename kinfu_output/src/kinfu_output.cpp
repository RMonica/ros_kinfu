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

// ROS custom messages
#include <kinfu_msgs/KinfuTsdfResponse.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>

// custom
#include "parameters.h"
#include "request_action_manager.h"
#include "kinfu_output_action_conversions.h"

typedef unsigned int uint;

#define LOOP_RATE 10
#define TIMEOUT (60 * LOOP_RATE) // after TIMEOUT cycles, the messages will be discarded.

class IQueuedResponse
{
  public:
  virtual void Update() = 0;
  virtual bool IsEnded() = 0;
  virtual ~IQueuedResponse() {}
};

template <class MT>
class TQueuedResponse
{
  public:
  TQueuedResponse(ros::NodeHandle & nh,std::string output_topic)
  {
    m_sent = false;
    m_prepare = 0;
    m_pub = nh.advertise<MT>(output_topic,1);
    m_cycle_count = 0;
    m_topic_name = output_topic;
  }

  virtual ~TQueuedResponse() {}

  protected:
  void _Update()
  {
    if (m_sent)
      return;

    if (m_cycle_count++ > TIMEOUT)
    {
      m_sent = true;
      return; // discard the message after the timeout
    }

    if (m_prepare > 0)
      m_prepare++;

    // wait until at least a subscriber is present
    // FIXME: what if i'm recording a rosbag on this channel?
    //   getNumSubscribers would be > 0 immediately...
    if (m_pub.getNumSubscribers() > 0 && m_prepare == 0)
    {
      m_pub.publish(m_data);
      ROS_INFO("kinfu_output: Publishing topic %s the first time.",m_topic_name.c_str());
      m_prepare++;
    }

    // wait, then write again (sometimes the first one is ignored). FIXME
    if (m_prepare > 2)
    {
      m_pub.publish(m_data);
      m_sent = true;
      ROS_INFO("kinfu_output: Publishing topic %s the second time",m_topic_name.c_str());
    }
  }

  bool _IsEnded() {return m_sent; }
  MT m_data;

  private:
  ros::Publisher m_pub;
  std::string m_topic_name;

  bool m_sent;
  uint m_prepare;
  uint m_cycle_count;
};

typedef boost::shared_ptr<IQueuedResponse> PQueuedResponse;

class TQueuedTsdfResponse: public TQueuedResponse<sensor_msgs::PointCloud2>,
  public IQueuedResponse
{
  public:
  TQueuedTsdfResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertTsdfToPointCloud2(resp,m_data);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse::_Update(); }
  bool IsEnded() {return TQueuedResponse::_IsEnded(); }
};

class TQueuedCloudResponse: public TQueuedResponse<sensor_msgs::PointCloud2>,
  public IQueuedResponse
{
  public:
  TQueuedCloudResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertCloudToPointCloud2(resp,m_data);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<sensor_msgs::PointCloud2>::_Update(); }
  bool IsEnded() {return TQueuedResponse<sensor_msgs::PointCloud2>::_IsEnded(); }
};

class TQueuedMeshResponse: public TQueuedResponse<pcl_msgs::PolygonMesh>,
  public IQueuedResponse
{
  public:
  TQueuedMeshResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertMeshToMeshMsg(resp,m_data);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<pcl_msgs::PolygonMesh>::_Update(); }
  bool IsEnded() {return TQueuedResponse<pcl_msgs::PolygonMesh>::_IsEnded(); }
};

class TQueuedPingResponse: public TQueuedResponse<std_msgs::Header>,
  public IQueuedResponse
{
  public:
  TQueuedPingResponse(ros::NodeHandle & nh,const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data.seq = resp.tsdf_header.request_id;
    m_data.stamp = ros::Time::now();
    m_data.frame_id = resp.reference_frame_id;
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<std_msgs::Header>::_Update(); }
  bool IsEnded() {return TQueuedResponse<std_msgs::Header>::_IsEnded(); }
};

class TQueuedImageResponse: public TQueuedResponse<sensor_msgs::Image>,
  public IQueuedResponse
{
  public:
  TQueuedImageResponse(ros::NodeHandle & nh,const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.image;
    m_data.header.stamp = ros::Time::now();
    m_data.header.seq = resp.tsdf_header.request_id;
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<sensor_msgs::Image>::_Update(); }
  bool IsEnded() {return TQueuedResponse<sensor_msgs::Image>::_IsEnded(); }
};

class TQueuedIntensityCloudResponse: public TQueuedResponse<sensor_msgs::PointCloud2>,
  public IQueuedResponse
{
  public:
  TQueuedIntensityCloudResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertIntensityCloudToPointCloud2(resp,m_data);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<sensor_msgs::PointCloud2>::_Update(); }
  bool IsEnded() {return TQueuedResponse<sensor_msgs::PointCloud2>::_IsEnded(); }
};

class TQueuedUintArrayResponse: public TQueuedResponse<std_msgs::UInt64MultiArray>,
  public IQueuedResponse
{
  public:
  typedef typename std::vector<std::string> TStringVector;
  typedef typename std::vector<long unsigned int> TUintVector;

  TQueuedUintArrayResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp,const TStringVector & dims,const TUintVector & sizes):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertUintArrayToMsg(resp,m_data,dims,sizes);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<std_msgs::UInt64MultiArray>::_Update(); }
  bool IsEnded() {return TQueuedResponse<std_msgs::UInt64MultiArray>::_IsEnded(); }
};

class TQueuedGridResponse: public TQueuedResponse<std_msgs::Float32MultiArray>,
  public IQueuedResponse
{
  public:
  TQueuedGridResponse(ros::NodeHandle & nh,
    const kinfu_msgs::KinfuTsdfResponse & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    ConvertGridToMsg(resp,m_data);
  }

  // solve multi-inheritance conflict
  void Update() {TQueuedResponse<std_msgs::Float32MultiArray>::_Update(); }
  bool IsEnded() {return TQueuedResponse<std_msgs::Float32MultiArray>::_IsEnded(); }
};

class ResponseConverter
{
  public:
  typedef boost::shared_ptr<pcl_msgs::PolygonMesh> MeshPtr;

  ResponseConverter(ros::NodeHandle & nh): m_nh(nh), m_ramgr(nh)
  {
    nh.param<std::string>(PARAM_NAME_RESPONSE_TOPIC,m_response_topic_name,PARAM_DEFAULT_RESPONSE_TOPIC);
    m_sub = nh.subscribe(m_response_topic_name,10,&ResponseConverter::callback,this);
  }

  void callback(const kinfu_msgs::KinfuTsdfResponse & response)
  {
    const uint tsdf_size = response.tsdf_cloud.size();
    const uint cloud_size = response.point_cloud.size();
    const uint triangles_size = response.triangles.size();
    const uint pixels = response.image.height * response.image.width;
    const uint uint64_values = response.uint_values.size();
    const uint float32_values = response.float_values.size();

    ROS_INFO("kinfu_output: \nReceived response with\n    %u tsdf points\n"
      "    %u point cloud points\n    %u triangles\n    %u pixels\n    %u uint64 values\n    %u float32 values\n",
      tsdf_size,cloud_size,triangles_size,pixels,uint64_values,float32_values);

    if (m_ramgr.HandleResponse(response))
      return; // handled by the action manager

    if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_PING)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedPingResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_TSDF)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedTsdfResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_MESH)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedMeshResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_CLOUD)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedCloudResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_KNOWN)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedCloudResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VIEW)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedImageResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VIEW_CLOUD)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedIntensityCloudResponse(m_nh,response)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VOXEL_COUNT)
    {
      TQueuedUintArrayResponse::TStringVector dims;
      dims.push_back("Voxel type");
      dims.push_back("View number");
      TQueuedUintArrayResponse::TUintVector sizes;
      sizes.push_back(2);
      sizes.push_back(response.uint_values.size() / 2);
      m_queue.push_back(PQueuedResponse(new TQueuedUintArrayResponse(m_nh,response,dims,sizes)));
    }
    else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VOXELGRID)
    {
      m_queue.push_back(PQueuedResponse(new TQueuedGridResponse(m_nh,response)));
    }
    else
      ROS_ERROR("kinfu_output: received unknown response type %u.",uint(response.tsdf_header.request_type));

  }

  void update()
  {
    // update all the queue elements
    for (std::list<PQueuedResponse>::iterator iter = m_queue.begin(); iter != m_queue.end(); iter++)
      (*iter)->Update();

    // erase the ended elements
    bool erased;
    do
    {
      erased = false;
      for (std::list<PQueuedResponse>::iterator iter = m_queue.begin(); iter != m_queue.end(); iter++)
        if ((*iter)->IsEnded())
        {
          m_queue.erase(iter);
          erased = true;
          break;
        }
    }
    while (erased);
  }

  private:
  ros::NodeHandle & m_nh;

  std::string m_response_topic_name;

  ros::Subscriber m_sub;

  RequestActionManager m_ramgr;

  std::list<PQueuedResponse> m_queue;
};

int main(int argc,char ** argv)
{
  ros::init(argc, argv, "kinfu_output");

  ros::NodeHandle nh("~");

  ResponseConverter conv(nh);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    conv.update();

    loop_rate.sleep();
  }

  return 1;
}
