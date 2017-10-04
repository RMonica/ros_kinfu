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

#ifndef KINFU_OUTPUT_ACTION_MANAGER_H
#define KINFU_OUTPUT_ACTION_MANAGER_H

// ROS
#include <ros/ros.h>
#include <actionlib/server/action_server.h>

// ROS custom
#include <kinfu_msgs/RequestAction.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>

// custom
#include "parameters.h"
#include "kinfu_output_ianswerer.h"

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// STL
#include <string>
#include <list>
#include <map>

class RequestActionManager
  {
  public:
  typedef unsigned int uint;
  typedef typename kinfu_msgs::RequestAction ActionType;
  typedef typename actionlib::ActionServer<ActionType> ActionServer;
  typedef typename boost::shared_ptr<ActionServer> ActionServerPtr;
  typedef typename ActionServer::GoalHandle GoalHandle;
  typedef typename std::list<GoalHandle> TGoalHandleList;
  typedef typename actionlib_msgs::GoalID TGoalID;
  typedef typename std::map<uint,TGoalID> TGoalIDUintMap;
  typedef typename std::pair<uint,TGoalID> TGoalIDUintPair;

  RequestActionManager(ros::NodeHandle & nh,KinfuOutputIAnswerer & answerer);

  void Update();

  // returns true if the response is handled
  bool HandleResponse(const kinfu_msgs::RequestResult & resp);

  private:
  void onNewGoal(GoalHandle goal_handle);
  uint getNewId();

  ros::NodeHandle & m_nh;

  ActionServerPtr m_server;
  TGoalHandleList m_active_goals;
  boost::mutex m_mutex;

  TGoalIDUintMap m_id_relations;
  uint m_next_id;

  KinfuOutputIAnswerer & m_answerer;

  std::string m_request_magic;
  };

#endif // KINFU_OUTPUT_ACTION_MANAGER_H
