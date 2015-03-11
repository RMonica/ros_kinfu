#ifndef REQUEST_ACTION_MANAGER_H
#define REQUEST_ACTION_MANAGER_H

// ROS
#include <ros/ros.h>
#include <actionlib/server/action_server.h>

// ROS custom
#include <kinfu_msgs/RequestAction.h>
#include <kinfu_msgs/KinfuTsdfResponse.h>
#include <kinfu_msgs/KinfuTsdfRequest.h>

// custom
#include "parameters.h"

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

  RequestActionManager(ros::NodeHandle & nh);

  void Update();

  // returns true if the response is handled
  bool HandleResponse(const kinfu_msgs::KinfuTsdfResponse & resp);

  private:
  void onNewGoal(GoalHandle & goal_handle);
  uint getNewId();

  ros::NodeHandle & m_nh;

  ActionServerPtr m_server;
  TGoalHandleList m_active_goals;
  boost::mutex m_mutex;

  TGoalIDUintMap m_id_relations;
  uint m_next_id;

  std::string m_request_magic;
  ros::Publisher m_request_publisher;
  };

#endif // REQUEST_ACTION_MANAGER_H
