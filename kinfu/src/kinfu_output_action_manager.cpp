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

#include "kinfu_output_action_manager.h"

RequestActionManager::RequestActionManager(ros::NodeHandle & nh, KinfuOutputIAnswerer & answerer,
                                           KinfuOutputSaveFile &save_file):
  m_nh(nh), m_answerer(answerer), m_save_file(save_file)
  {
  std::string temp_string;
  m_nh.param<std::string>(PARAM_NAME_REQUEST_ACTION_NAME,temp_string,PARAM_DEFAULT_REQUEST_ACTION_NAME);
  m_server = ActionServerPtr(
    new ActionServer(m_nh,temp_string,boost::bind(&RequestActionManager::onNewGoal,this,_1),false));

  m_nh.param<std::string>(PARAM_NAME_REQUEST_ACTION_MAGIC,m_request_magic,PARAM_DEFAULT_REQUEST_ACTION_MAGIC);

  m_next_id = 0;

  boost::mutex::scoped_lock lock(m_mutex);
  m_server->start();
  }

void RequestActionManager::onNewGoal(GoalHandle goal_handle)
  {
    kinfu_msgs::KinfuTsdfRequestPtr request_msg;

    {
    boost::mutex::scoped_lock lock(m_mutex);
    goal_handle.setAccepted();

    const uint id = getNewId();

    boost::shared_ptr<const kinfu_msgs::RequestGoal> goal_msg = goal_handle.getGoal();
    request_msg = kinfu_msgs::KinfuTsdfRequestPtr(new kinfu_msgs::KinfuTsdfRequest(goal_msg->request));
    request_msg->tsdf_header.request_source_name = m_request_magic;
    request_msg->tsdf_header.request_id = id;

    ROS_INFO("kinfu_output: executing new request action, id %u.",id);

    m_id_relations.insert(TGoalIDUintPair(id,goal_handle.getGoalID()));
    m_active_goals.push_back(goal_handle);
    }

  m_answerer.requestCallback(request_msg);
  }

bool RequestActionManager::HandleResponse(kinfu_msgs::RequestResult & resp)
  {
  boost::mutex::scoped_lock lock(m_mutex);

  if (resp.tsdf_header.request_source_name != m_request_magic)
    return false;

  const uint id = resp.tsdf_header.request_id;
  TGoalIDUintMap::iterator rel_i = m_id_relations.find(id);
  if (rel_i == m_id_relations.end())
    {
    ROS_WARN("kinfu_output: received response for action, but id %u was not found.",id);
    return true;
    }

  TGoalHandleList::iterator goal_i = m_active_goals.begin();
  for (goal_i = m_active_goals.begin(); goal_i != m_active_goals.end(); ++goal_i)
    if (goal_i->getGoalID().id == rel_i->second.id)
      break;

  if (m_save_file.IsLargeResponse(resp))
    {
    ROS_WARN("kinfu_output: response for action with id %u is too large, saving to file.", unsigned(id));
    const std::string filename = m_save_file.SaveFile(resp);
    if (filename.empty())
      {
      ROS_ERROR("kinfu_output: could not save file, action %u failed.", unsigned(id));
      goal_i->setAborted(resp);
      return true;
      }
    resp.file_name = filename;
    }

  if (goal_i->getGoalStatus().status == actionlib_msgs::GoalStatus().ACTIVE)
    {
    ROS_INFO("kinfu_output: result sent for id %u", unsigned(id));
    goal_i->setSucceeded(resp);
    }

  return true;
  }

void RequestActionManager::Update()
  {
  boost::mutex::scoped_lock lock(m_mutex);

  // cleanup
  for (TGoalHandleList::iterator goal_i = m_active_goals.begin(); goal_i != m_active_goals.end(); )
    {
    if (goal_i->getGoalStatus().status != actionlib_msgs::GoalStatus().ACTIVE)
      {
      for (TGoalIDUintMap::iterator rel_i = m_id_relations.begin(); rel_i != m_id_relations.end(); ++rel_i)
        if (rel_i->second.id == goal_i->getGoalID().id)
          {
          m_id_relations.erase(rel_i);
          break;
          }

      TGoalHandleList::iterator erase_i = goal_i++;
      m_active_goals.erase(erase_i);
      }
      else
        ++goal_i;
    }
  }

RequestActionManager::uint RequestActionManager::getNewId()
  {
  return m_next_id++;
  }
