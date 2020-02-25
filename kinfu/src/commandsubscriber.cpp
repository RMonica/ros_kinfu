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

#include "commandsubscriber.h"

#include <std_msgs/String.h>

CommandSubscriber::CommandSubscriber(ros::NodeHandle &nh, tf::TransformListener &tf_listener,
  boost::mutex & shared_mutex, boost::condition_variable & cond):
  m_nh(nh),m_tf_listener(tf_listener),m_shared_mutex(shared_mutex),m_cond(cond)
  {
  m_nh.param<std::string>(PARAM_NAME_COMMAND_TOPIC,m_command_topic_name,PARAM_DEFAULT_COMMAND_TOPIC);
  m_nh.param<bool>(PARAM_NAME_AUTOSTART,m_is_running,PARAM_DEFAULT_AUTOSTART);

  m_nh.param<bool>(PARAM_NAME_ENABLE_MIN_MOTION,m_enable_minimum_movement,PARAM_DEFAULT_ENABLE_MIN_MOTION);
  m_nh.param<std::string>(PARAM_NAME_FORCED_CURRENT_FRAME,m_forced_tf_frame_name,PARAM_DEFAULT_FORCED_CURRENT_FRAME);
  m_nh.param<std::string>(PARAM_NAME_FORCED_REFERENCE_FRAME,m_forced_tf_frame_reference,PARAM_DEFAULT_FORCED_REFERENCE_FRAME);

  m_nh.param<std::string>(PARAM_NAME_COMMAND_ACK_TOPIC,m_ack_topic_name,PARAM_DEFAULT_COMMAND_ACK_TOPIC);
  m_ack_publisher = m_nh.advertise<std_msgs::String>(m_ack_topic_name,5);

  m_request_reset = false;
  m_forced_tf_frames = false;
  m_is_triggered = false;
  m_hint_expiration = ros::Time(0);

  m_nh.param<bool>(PARAM_NAME_FORCED_TF_POSITION,m_forced_tf_frames,PARAM_DEFAULT_FORCED_TF_POSITION);

  m_initial_transformation = Eigen::Affine3f::Identity();

  m_command_subscriber = m_nh.subscribe(m_command_topic_name,5,&CommandSubscriber::commandCallback,this);
  }

void CommandSubscriber::commandCallback(const kinfu_msgs::KinfuCommand & cmd)
  {
  boost::mutex::scoped_lock shared_lock(m_shared_mutex);

  if (cmd.command_type == cmd.COMMAND_TYPE_NOOP)
    {
    // does nothing
    ack(cmd.command_id,true);
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_RESUME)
    {
    m_is_running = true;
    ack(cmd.command_id,true);
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_SUSPEND)
    {
    m_is_running = false;
    ack(cmd.command_id,true);
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_RESET)
    {
    if (m_request_reset)
      ack(m_request_reset_command_id,false);
    m_request_reset = true;
    m_request_reset_command_id = cmd.command_id;
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_SET_ENABLED_MIN_MOVEMENT)
    {
    if (cmd.boolean_data.size() < 1)
      {
      ROS_ERROR("kinfu: command: set enabled min movement: one boolean required in boolean_data.");
      ack(cmd.command_id,false);
      }
    else
      {
      m_enable_minimum_movement = cmd.boolean_data[0];
      ROS_INFO("kinfu: command: set enable min movement: %u.",uint(m_enable_minimum_movement));
      ack(cmd.command_id,true);
      }
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_SET_FORCED_TF_FRAMES)
    {
    if (cmd.boolean_data.size() < 1)
      {
      ROS_ERROR("kinfu: command: set forced tf frame: one boolean required in boolean_data.");
      ack(cmd.command_id,false);
      }
    else
      {
      m_forced_tf_frames = cmd.boolean_data[0];
      ROS_INFO("kinfu: command: set forced tf frame: %u.",uint(m_forced_tf_frames));
      if (m_forced_tf_frames)
        m_hint_expiration = ros::Time(0); // clear the hint: now forced from tf
      ack(cmd.command_id,true);
      }
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_CLEAR_SPHERE)
    {
    ROS_INFO("kinfu: command: clear sphere.");
    if (cmd.float_data.size() < 4)
      ROS_ERROR("kinfu: command: a sphere requires 4 float_data params.");
    else
      {
      Eigen::Vector3f center,kinfu_center;
      float radius;
      for (uint i = 0; i < 3; i++)
        center[i] = cmd.float_data[i];
      radius = cmd.float_data[3];

      bool set_to_empty = false;
      if (cmd.boolean_data.size() >= 1 && cmd.boolean_data[0])
        set_to_empty = true;

      kinfu_center = m_initial_transformation * center;

      m_clear_sphere = Sphere::Ptr(new Sphere(kinfu_center,radius,cmd.command_id,set_to_empty));
      }
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_CLEAR_BOUNDING_BOX)
    {
    ROS_INFO("kinfu: command: clear bbox.");
    if (cmd.float_data.size() < 6)
      ROS_ERROR("kinfu: command: a bounding box requires 6 float_data params.");
    else
      {
      Eigen::Vector3f w_min,w_max;
      Eigen::Vector3f kinfu_min,kinfu_max;
      for (uint i = 0; i < 3; i++)
        w_min[i] = cmd.float_data[i];
      for (uint i = 0; i < 3; i++)
        w_max[i] = cmd.float_data[i + 3];

      w_min = m_initial_transformation * w_min;
      w_max = m_initial_transformation * w_max;
      kinfu_min = w_min.array().min(w_max.array());
      kinfu_max = w_min.array().max(w_max.array());

      bool set_to_empty = false;
      if (cmd.boolean_data.size() >= 1 && cmd.boolean_data[0])
        set_to_empty = true;

      m_clear_bbox = BBox::Ptr(new BBox(kinfu_min,kinfu_max,cmd.command_id,set_to_empty));
      }
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_CLEAR_CYLINDER)
    {
    ROS_INFO("kinfu: command: clear cylinder.");
    if (cmd.float_data.size() < 7)
      ROS_ERROR("kinfu: command: clear cylinder requires 7 float_data params.");
    else
      {
      Eigen::Vector3f b_min,b_max;
      float r;
      for (uint i = 0; i < 3; i++)
        b_min[i] = cmd.float_data[i];
      for (uint i = 0; i < 3; i++)
        b_max[i] = cmd.float_data[i + 3];
      r = cmd.float_data[6];;

      b_min = m_initial_transformation * b_min;
      b_max = m_initial_transformation * b_max;

      if ((b_min - b_max).squaredNorm() < 1e-7f)
        ROS_WARN("kinfu: cylinder height is very small, division by zero may occur later.");

      bool set_to_empty = false;
      if (cmd.boolean_data.size() >= 1 && cmd.boolean_data[0])
        set_to_empty = true;

      m_clear_cylinder = Cylinder::Ptr(new Cylinder(b_min,b_max,r,cmd.command_id,set_to_empty));
      }
    }
  else if (cmd.command_type == cmd.COMMAND_TYPE_TRIGGER)
    {
    if (m_is_running || m_is_triggered)
      ROS_WARN("kinfu: trigger request ignored: kinfu is already running.");
    else
      {
      m_is_triggered = true;
      m_is_triggered_command_id = cmd.command_id;
      }
    }
  else
    {
    ROS_ERROR("kinfu: command: unknown command type: %u",uint(cmd.command_type));
    ack(cmd.command_id,false);
    }

  // if the hint comes from tf frames, do not force hint otherwise
  if (!m_forced_tf_frames)
    {
    if (cmd.hint_expiration_time >= ros::Time::now())
      {
      m_hint_expiration = cmd.hint_expiration_time;

      for (uint r = 0; r < 3; r++)
        for (uint c = 0; c < 3; c++)
          m_hint.linear()(r,c) = cmd.pose_hint_rotation[r * 3 + c];
      for (uint i = 0; i < 3; i++)
        m_hint.translation()[i] = cmd.pose_hint_translation[i];

      m_hint = m_initial_transformation * m_hint;

      m_hint_forced = cmd.hint_forced;
      }
    }

  m_cond.notify_all();
  }

bool CommandSubscriber::hasHint() const
  {
  return m_hint_expiration >= ros::Time::now() || m_forced_tf_frames;
  }

bool CommandSubscriber::hasForcedHint() const
  {
  return m_hint_forced || m_forced_tf_frames;
  }

Eigen::Affine3f CommandSubscriber::getHintTransform(bool & ok) const
  {
  if (m_forced_tf_frames)
    {
    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    ok = getForcedTfFrame(result);
    return m_initial_transformation * result;
    }

  ok = true;
  return m_hint;
  }

void CommandSubscriber::clearHint()
  {
  m_hint_expiration = ros::Time(0);
  }

bool CommandSubscriber::isResetRequired() const
  {
  return m_request_reset;
  }

void CommandSubscriber::ack(const std::string &id, bool success) const
  {
  std_msgs::String id_msg;
  id_msg.data = success ? (id + COMMAND_ACK_STRING) : (id + COMMAND_NACK_STRING);
  m_ack_publisher.publish(id_msg);
  }

bool CommandSubscriber::getForcedTfFrame(Eigen::Affine3f & result) const
  {
  tf::StampedTransform transform;
  try
    {
    m_tf_listener.lookupTransform(m_forced_tf_frame_reference,m_forced_tf_frame_name,ros::Time(0),transform);
    }
    catch (tf::TransformException ex)
      {
      ROS_ERROR("kinfu: hint was forced by TF, but retrieval failed because: %s",ex.what());
      return false;
      }

  Eigen::Vector3f vec;
  vec.x() = transform.getOrigin().x();
  vec.y() = transform.getOrigin().y();
  vec.z() = transform.getOrigin().z();
  Eigen::Quaternionf quat;
  quat.x() = transform.getRotation().x();
  quat.y() = transform.getRotation().y();
  quat.z() = transform.getRotation().z();
  quat.w() = transform.getRotation().w();

  result.linear() = Eigen::AngleAxisf(quat).matrix();
  result.translation() = vec;
  return true;
  }
