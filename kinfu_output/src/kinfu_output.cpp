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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>

// ROS custom messages
#include <kinfu_msgs/KinfuTsdfResponse.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>

// custom
#include "parameters.h"

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

    pcl::toROSMsg(pointcloud,m_data);

    m_data.header.seq = resp.tsdf_header.request_id;
    m_data.header.stamp = ros::Time::now();
    m_data.header.frame_id = resp.reference_frame_id;
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

    pcl::toROSMsg(pointcloud,m_data);

    m_data.header.seq = resp.tsdf_header.request_id;
    m_data.header.stamp = ros::Time::now();
    m_data.header.frame_id = resp.reference_frame_id;
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

    pcl::toROSMsg(pointcloud,m_data.cloud);

    m_data.polygons.reserve(triangles_size);
    for (uint i = 0; i < triangles_size; i++)
    {
      pcl_msgs::Vertices v;
      const kinfu_msgs::KinfuMeshTriangle & t = resp.triangles[i];

      v.vertices.resize(3);
      for (uint h = 0; h < 3; h++)
        v.vertices[h] = t.vertex_id[h];

      m_data.polygons.push_back(v);
    }

    m_data.header.seq = resp.tsdf_header.request_id;
    m_data.header.stamp = ros::Time::now();
    m_data.header.frame_id = resp.reference_frame_id;
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

class ResponseConverter
{
  public:
  typedef boost::shared_ptr<pcl_msgs::PolygonMesh> MeshPtr;

  ResponseConverter(ros::NodeHandle & nh): m_nh(nh)
  {
    nh.param<std::string>(PARAM_NAME_RESPONSE_TOPIC,m_response_topic_name,PARAM_DEFAULT_RESPONSE_TOPIC);
    m_sub = nh.subscribe<kinfu_msgs::KinfuTsdfResponse>(m_response_topic_name,10,&ResponseConverter::callback,this);
  }

  void callback(const kinfu_msgs::KinfuTsdfResponse response)
  {
    uint tsdf_size = response.tsdf_cloud.size();
    uint cloud_size = response.point_cloud.size();
    uint triangles_size = response.triangles.size();

    ROS_INFO("kinfu_output: \nReceived response with\n    %u tsdf points,\n"
      "    %u point cloud points\nand %u triangles.\n",
      tsdf_size,cloud_size,triangles_size);

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
