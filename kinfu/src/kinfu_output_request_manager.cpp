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

#include "kinfu_output_request_manager.h"
#include "kinfu_output_action_manager.h"

typedef unsigned int uint;

#define LOOP_RATE 10
#define TIMEOUT (30 * LOOP_RATE) // after TIMEOUT cycles, the messages will be discarded.
#define CONNECTED_TIMEOUT (1 * LOOP_RATE)
  // if a node is connected for CONNECTED_TIMEOUT cycles, the message is considered sent

template <class MT>
class TQueuedResponse: public RequestManager::IQueuedResponse
{
  public:
  TQueuedResponse(ros::NodeHandle & nh,std::string output_topic)
  {
    m_pub = nh.advertise<MT>(output_topic,1,true);
    m_topic_name = output_topic;
    m_sent = false;
    m_connected = false;
    m_cycle_count = 0;
    m_connected_cycle_count = 0;
  }

  virtual ~TQueuedResponse() {}

  protected:
  void Update()
  {
    if (m_sent)
      return;

    if (m_cycle_count == 0)
    {
      m_pub.publish(m_data);
      m_cycle_count++;
      return;
    }

    m_cycle_count++;

    if (m_cycle_count > TIMEOUT)
    {
      m_sent = true;
      return; // discard the message after the timeout
    }

    // FIXME: we may have to wait for more than one subscriber
    if (!m_connected && m_pub.getNumSubscribers() > 0)
    {
      m_connected = true; // the receiver may disconnect immediately after receiving the message
                          // so we save the connected status here
      ROS_INFO("kinfu_output: Connection detected for topic %s.",m_topic_name.c_str());
    }

    if (m_connected)
      m_connected_cycle_count++;

    if (m_connected_cycle_count > CONNECTED_TIMEOUT)
    {
      m_sent = true; // probably, the node received the message
      ROS_INFO("kinfu_output: Message to topic %s sent.",m_topic_name.c_str());
      return;
    }
  }

  bool IsEnded() {return m_sent; }
  MT m_data;

  private:
  ros::Publisher m_pub;
  std::string m_topic_name;

  bool m_sent;
  uint m_cycle_count;
  bool m_connected;
  uint m_connected_cycle_count;
};

class TQueuedTsdfResponse: public TQueuedResponse<sensor_msgs::PointCloud2>
{
  public:
  TQueuedTsdfResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.pointcloud;
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedCloudResponse: public TQueuedResponse<sensor_msgs::PointCloud2>
{
  public:
  TQueuedCloudResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.pointcloud;
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedMeshResponse: public TQueuedResponse<pcl_msgs::PolygonMesh>
{
  public:
  TQueuedMeshResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.mesh;
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedPingResponse: public TQueuedResponse<std_msgs::Header>
{
  public:
  TQueuedPingResponse(ros::NodeHandle & nh,const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data.seq = resp.tsdf_header.request_id;
    m_data.stamp = ros::Time::now();
    m_data.frame_id = resp.header.frame_id;
  }
};

class TQueuedImageResponse: public TQueuedResponse<sensor_msgs::Image>
{
  public:
  TQueuedImageResponse(ros::NodeHandle & nh,const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.image;
    m_data.header.stamp = ros::Time::now();
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedIntensityCloudResponse: public TQueuedResponse<sensor_msgs::PointCloud2>
{
  public:
  TQueuedIntensityCloudResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.pointcloud;
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedXYZNormalCloudResponse: public TQueuedResponse<sensor_msgs::PointCloud2>
{
  public:
  TQueuedXYZNormalCloudResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.pointcloud;
    m_data.header.seq = resp.tsdf_header.request_id;
  }
};

class TQueuedUintArrayResponse: public TQueuedResponse<std_msgs::UInt64MultiArray>
{
  public:
  typedef typename std::vector<std::string> TStringVector;
  typedef typename std::vector<long unsigned int> TUintVector;

  TQueuedUintArrayResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.uint_values;
  }
};

class TQueuedGridResponse: public TQueuedResponse<std_msgs::Float32MultiArray>
{
  public:
  TQueuedGridResponse(ros::NodeHandle & nh,
    const kinfu_msgs::RequestResult & resp):
    TQueuedResponse(nh,resp.tsdf_header.request_source_name)
  {
    m_data = resp.float_values;
  }
};

void RequestManager::SendResponse(const kinfu_msgs::RequestResultPtr & response_ptr)
{
  boost::mutex::scoped_lock lock(m_mutex);

  kinfu_msgs::RequestResult & response = *response_ptr;

  const uint cloud_size = response.pointcloud.height * response.pointcloud.width;
  const uint mesh_cloud_size = response.mesh.cloud.height * response.mesh.cloud.width;
  const uint triangles_size = response.mesh.polygons.size();
  const uint pixels = response.image.height * response.image.width;
  const uint uint64_values = response.uint_values.data.size();
  const uint float32_values = response.float_values.data.size();

  ROS_INFO("kinfu_output: \nReceived response with\n    %u cloud points\n"
    "    %u mesh cloud points\n    %u triangles\n    %u pixels\n    %u uint64 values\n    %u float32 values\n",
    cloud_size,mesh_cloud_size,triangles_size,pixels,uint64_values,float32_values);

  if (m_ramgr->HandleResponse(response))
    return; // request was an action

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
    m_queue.push_back(PQueuedResponse(new TQueuedUintArrayResponse(m_nh,response)));
  }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_VOXELGRID)
  {
    m_queue.push_back(PQueuedResponse(new TQueuedGridResponse(m_nh,response)));
  }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_FRONTIER_POINTS)
  {
    m_queue.push_back(PQueuedResponse(new TQueuedXYZNormalCloudResponse(m_nh,response)));
  }
  else if (response.tsdf_header.request_type == response.tsdf_header.REQUEST_TYPE_GET_BORDER_POINTS)
  {
    m_queue.push_back(PQueuedResponse(new TQueuedXYZNormalCloudResponse(m_nh,response)));
  }
  else
    ROS_ERROR("kinfu_output: received unknown response type %u.",uint(response.tsdf_header.request_type));
}

void RequestManager::Update()
{
  // update all the queue elements
  for (PQueuedResponseList::iterator iter = m_queue.begin(); iter != m_queue.end(); iter++)
    (*iter)->Update();

  // erase the ended elements
  for (PQueuedResponseList::iterator iter = m_queue.begin(); iter != m_queue.end(); )
  {
    PQueuedResponseList::iterator td = iter;
    ++iter;
    if ((*td)->IsEnded())
      m_queue.erase(td);
  }

  m_ramgr->Update();
}

void RequestManager::requestCallback(const kinfu_msgs::KinfuTsdfRequestConstPtr & request)
{
  m_answerer.requestCallback(request);
}

RequestManager::RequestManager(ros::NodeHandle & nh,KinfuOutputIAnswerer & answerer): m_nh(nh), m_answerer(answerer)
{
  boost::mutex::scoped_lock lock(m_mutex);
  std::string param_string;

  m_nh.param<std::string>(PARAM_NAME_REQUEST_TOPIC,param_string,PARAM_DEFAULT_REQUEST_TOPIC);
  m_req_sub = m_nh.subscribe(param_string, 10,&RequestManager::requestCallback,this);

  m_ramgr = RequestActionManagerPtr(new RequestActionManager(m_nh,m_answerer));

  m_is_terminating = false;
  m_update_thread = boost::thread(&RequestManager::run,this);
}

void RequestManager::run()
{
  ROS_INFO("kinfu_output: main thread started.");
  boost::mutex::scoped_lock lock(m_mutex);
  ros::Rate rate(LOOP_RATE);
  while (ros::ok() && !m_is_terminating)
  {
    Update();
    lock.unlock();
    rate.sleep();
    lock.lock();
  }
  ROS_INFO("kinfu_output: main thread terminated.");
}

RequestManager::~RequestManager()
{
  {
    boost::mutex::scoped_lock lock(m_mutex);
    m_is_terminating = true;
  }
  ROS_INFO("kinfu_output: joining main thread.");
  m_update_thread.join();
}
