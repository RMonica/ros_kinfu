/*
 * Copyright (c) 2014-2015, Riccardo Monica
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

#include "kinfu_tf_feeder.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/UInt32.h>

// Boost
#include <boost/shared_ptr.hpp>

// custom
#include <kinfu_msgs/KinfuCommand.h>

class KinfuTFFeeder
  {
  public:
  typedef typename boost::shared_ptr<tf::TransformListener> TransformListenerPtr;
  typedef unsigned int uint;

  KinfuTFFeeder(ros::NodeHandle &nh): m_nh(nh)
    {
    std::string param_string;
    int param_int;

    m_nh.param<double>(PARAM_NAME_POSE_TIMEOUT,m_pose_timeout,PARAM_DEFAULT_POSE_TIMEOUT);
    m_transform_listener = TransformListenerPtr(new tf::TransformListener(ros::Duration(m_pose_timeout)));

    m_nh.param<std::string>(PARAM_NAME_REFERENCE_FRAME,m_reference_frame,PARAM_DEFAULT_REFERENCE_FRAME);
    m_nh.param<std::string>(PARAM_NAME_POSE_FRAME,m_pose_frame,PARAM_DEFAULT_POSE_FRAME);

    m_nh.param<int>(PARAM_NAME_RATE,param_int,PARAM_DEFAULT_RATE);
    m_rate = (param_int > 0) ? param_int : PARAM_DEFAULT_RATE;

    m_nh.param<std::string>(PARAM_NAME_KINFU_COMMAND_TOPIC,param_string,PARAM_DEFAULT_KINFU_COMMAND_TOPIC);
    m_command_publisher = m_nh.advertise<kinfu_msgs::KinfuCommand>(param_string,1);

    m_nh.param<bool>(PARAM_NAME_START_KINFU,m_start_kinfu,PARAM_DEFAULT_START_KINFU);
    m_nh.param<bool>(PARAM_NAME_FORCED,m_forced,PARAM_DEFAULT_FORCED);
    m_nh.param<bool>(PARAM_NAME_TRIGGERED,m_triggered,PARAM_DEFAULT_TRIGGERED);
    m_nh.param<bool>(PARAM_NAME_AUTOSTART,m_running,PARAM_DEFAULT_AUTOSTART);

    m_nh.param<std::string>(PARAM_NAME_START_TOPIC,param_string,PARAM_DEFAULT_START_TOPIC);
    m_start_subscriber = m_nh.subscribe(param_string,5,&KinfuTFFeeder::onStart,this);
    m_nh.param<std::string>(PARAM_NAME_STOP_TOPIC,param_string,PARAM_DEFAULT_STOP_TOPIC);
    m_stop_subscriber = m_nh.subscribe(param_string,5,&KinfuTFFeeder::onStop,this);
    m_nh.param<std::string>(PARAM_NAME_ACK_TOPIC,param_string,PARAM_DEFAULT_ACK_TOPIC);
    m_ack_publisher = m_nh.advertise<std_msgs::UInt32>(param_string,1);
    }

  void onStart(const std_msgs::UInt32 & id)
    {
    m_running = true;
    m_ack_publisher.publish(id);
    }

  void onStop(const std_msgs::UInt32 & id)
    {
    m_running = false;
    m_ack_publisher.publish(id);
    }

  void Update()
    {
    if (!m_running)
      return;

    Eigen::Affine3d epose;

    try
      {
      tf::StampedTransform tpose;
      m_transform_listener->lookupTransform(m_reference_frame,m_pose_frame,ros::Time(0),tpose);
      tf::poseTFToEigen(tpose,epose);
      }
    catch (tf::TransformException ex)
      {
      return;
      }

    kinfu_msgs::KinfuCommandPtr command(new kinfu_msgs::KinfuCommand);
    if (m_start_kinfu)
      {
      command->command_type = command->COMMAND_TYPE_RESUME;
      m_start_kinfu = false;
      ROS_INFO("kinfu_tf_feeder: got first pose, starting kinfu.");
      }
      else
        command->command_type = m_triggered ? uint(command->COMMAND_TYPE_TRIGGER) : uint(command->COMMAND_TYPE_NOOP);

    command->hint_expiration_time = ros::Time::now() + ros::Duration(m_pose_timeout);
    command->hint_forced = m_forced;

    for (uint i = 0; i < 3; i++)
      for (uint h = 0; h < 3; h++)
        command->pose_hint_rotation[i * 3 + h] = epose.linear()(i,h);
    for (uint i = 0; i < 3; i++)
      command->pose_hint_translation[i] = epose.translation()[i];

    m_command_publisher.publish(command);
    }

  uint GetRate() const {return m_rate; }

  private:
  ros::NodeHandle & m_nh;

  TransformListenerPtr m_transform_listener;

  ros::Publisher m_command_publisher;
  ros::Publisher m_ack_publisher;

  ros::Subscriber m_start_subscriber;
  ros::Subscriber m_stop_subscriber;

  std::string m_reference_frame;
  std::string m_pose_frame;

  double m_pose_timeout;

  uint m_rate;

  bool m_forced;
  bool m_triggered;
  bool m_running;

  bool m_start_kinfu;
  };

int main(int argc,char ** argv)
  {
  ros::init(argc,argv,"kinfu_tf_feeder");
  ros::NodeHandle nh("~");

  KinfuTFFeeder kttf(nh);

  ros::Rate rate(kttf.GetRate());

  while (ros::ok())
    {
    kttf.Update();
    ros::spinOnce();
    rate.sleep();
    }

  return 0;
  }
