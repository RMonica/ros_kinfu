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

#ifndef COMMANDSUBSCRIBER_H
#define COMMANDSUBSCRIBER_H

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// custom messages
#include <kinfu_msgs/KinfuCommand.h>

// STL
#include <string>

// custom
#include "parameters.h"

#define COMMAND_ACK_STRING ":OK"
#define COMMAND_NACK_STRING ":NO"

class CommandSubscriber
  {
  public:
  typedef unsigned int uint;

  struct Sphere
  {
    typedef boost::shared_ptr<Sphere> Ptr;
    Eigen::Vector3f center;
    float radius;

    std::string command_id;

    Sphere(const Eigen::Vector3f & c,float r,const std::string &id):
      center(c),radius(r),command_id(id) {}
    Sphere(): center(Eigen::Vector3f::Zero()),radius(0),command_id(0) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct BBox
  {
    typedef boost::shared_ptr<BBox> Ptr;
    Eigen::Vector3f min;
    Eigen::Vector3f max;

    std::string command_id;

    BBox(const Eigen::Vector3f & m,const Eigen::Vector3f & M,const std::string &id):
      min(m),max(M),command_id(id) {}
    BBox(): min(Eigen::Vector3f::Zero()), max(Eigen::Vector3f::Zero()) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  CommandSubscriber(ros::NodeHandle & nh,tf::TransformListener & tf_listener,
    boost::mutex & shared_mutex,boost::condition_variable & cond);

  void commandCallback(const kinfu_msgs::KinfuCommand &cmd);

  bool hasHint() const;
  bool hasForcedHint() const;
  Eigen::Affine3f getHintTransform() const;
  void clearHint();

  bool isResetRequired() const;
  void clearResetRequired() {m_request_reset = false; }
  const std::string & getResetCommandId() const {return m_request_reset_command_id; }

  bool isRunning() const {return m_is_running; }

  bool isTriggered() const {return m_is_triggered; }
  bool clearTriggered() {m_is_triggered = false; }
  const std::string & getIsTriggeredCommandId() const {return m_is_triggered_command_id; }

  void setInitialTransformation(Eigen::Affine3f initial) {m_initial_transformation = initial; }

  // is a minimum kinect movement required to update the kinfu?
  bool isEnabledMinimumMovement() const {return m_enable_minimum_movement; }

  bool getForcedTfFrame(Eigen::Affine3f & result) const;

  bool hasClearSphere() {return bool(getClearSphere()); }
  Sphere::Ptr getClearSphere() {return m_clear_sphere; }
  void clearClearSphere() {m_clear_sphere = Sphere::Ptr(); }

  bool hasClearBBox() {return bool(getClearBBox()); }
  BBox::Ptr getClearBBox() {return m_clear_bbox; }
  void clearClearBBox() {m_clear_bbox = BBox::Ptr(); }

  // WARNING: this must be thread safe!
  void ack(const std::string & id,bool success) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  ros::NodeHandle & m_nh;
  tf::TransformListener & m_tf_listener;

  boost::mutex & m_shared_mutex;
  boost::condition_variable & m_cond;

  std::string m_command_topic_name;
  ros::Subscriber m_command_subscriber;

  std::string m_ack_topic_name;
  ros::Publisher m_ack_publisher;

  bool m_is_running;

  bool m_is_triggered;
  std::string m_is_triggered_command_id;

  bool m_request_reset;
  std::string m_request_reset_command_id;

  Eigen::Affine3f m_initial_transformation;

  Sphere::Ptr m_clear_sphere;
  BBox::Ptr m_clear_bbox;

  Eigen::Affine3f m_hint;
  ros::Time m_hint_expiration;
  bool m_hint_forced;

  bool m_enable_minimum_movement;

  bool m_forced_tf_frames;
  std::string m_forced_tf_frame_name;
  std::string m_forced_tf_frame_reference;
  };

#endif // COMMANDSUBSCRIBER_H
