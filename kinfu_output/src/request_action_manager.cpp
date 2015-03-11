#include "request_action_manager.h"

#include "kinfu_output_action_conversions.h"

RequestActionManager::RequestActionManager(ros::NodeHandle & nh): m_nh(nh)
  {
  std::string temp_string;
  m_nh.param<std::string>(PARAM_NAME_REQUEST_ACTION_NAME,temp_string,PARAM_DEFAULT_REQUEST_ACTION_NAME);
  m_server = ActionServerPtr(
    new ActionServer(m_nh,temp_string,boost::bind(&RequestActionManager::onNewGoal,this,_1),false));

  m_nh.param<std::string>(PARAM_NAME_REQUEST_ACTION_MAGIC,m_request_magic,PARAM_DEFAULT_REQUEST_ACTION_MAGIC);

  m_nh.param<std::string>(PARAM_NAME_KINFU_REQUEST_TOPIC,temp_string,PARAM_DEFAULT_KINFU_REQUEST_TOPIC);
  m_request_publisher = m_nh.advertise<kinfu_msgs::KinfuTsdfRequest>(temp_string,5);

  m_next_id = 0;

  boost::mutex::scoped_lock lock(m_mutex);
  m_server->start();
  }

void RequestActionManager::onNewGoal(GoalHandle & goal_handle)
  {
  boost::mutex::scoped_lock lock(m_mutex);

  goal_handle.setAccepted();

  const uint id = getNewId();

  boost::shared_ptr<const kinfu_msgs::RequestGoal> goal_msg = goal_handle.getGoal();
  boost::shared_ptr<kinfu_msgs::KinfuTsdfRequest> request_msg(new kinfu_msgs::KinfuTsdfRequest(goal_msg->request));
  request_msg->tsdf_header.request_source_name = m_request_magic;
  request_msg->tsdf_header.request_id = id;

  ROS_INFO("kinfu_output: executing new request action, id %u.",id);
  m_request_publisher.publish(request_msg);

  m_id_relations.insert(TGoalIDUintPair(id,goal_handle.getGoalID()));
  m_active_goals.push_back(goal_handle);
  }

bool RequestActionManager::HandleResponse(const kinfu_msgs::KinfuTsdfResponse & resp)
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

  kinfu_msgs::RequestResultPtr result(new kinfu_msgs::RequestResult);
  ConvertToActionResponse(resp,*result);

  for (TGoalHandleList::iterator goal_i = m_active_goals.begin(); goal_i != m_active_goals.end(); ++goal_i)
    if (goal_i->getGoalID().id == rel_i->second.id)
      {
      if (goal_i->getGoalStatus().status == actionlib_msgs::GoalStatus().ACTIVE)
        {
        ROS_INFO("kinfu_output: result sent for id %u",id);
        goal_i->setSucceeded(*result);
        }
      break;
      }
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
