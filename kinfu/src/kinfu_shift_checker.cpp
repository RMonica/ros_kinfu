#include "kinfu_shift_checker.h"

#include <sstream>
#include <float.h>
#include <cmath>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include <kinfu_msgs/CheckForShift.h>

#include <Eigen/Dense>

typedef pcl::gpu::kinfuLS::KinfuTracker KinfuTracker;
typedef KinfuTracker::IShiftChecker IShiftChecker;

class NeverShiftChecker: public IShiftChecker
{
  public:
  bool CheckForShift(const Eigen::Affine3f &cam_pose, const Eigen::Vector3f & center_cube) override
  {
    return false;
  }

  Eigen::Vector3f GetNewCubeOrigin(const Eigen::Affine3f &cam_pose) override
  {
    return Eigen::Vector3f::Zero();
  }
};

class FixedShiftChecker: public IShiftChecker
{
  public:
  FixedShiftChecker(const float x, const float y, const float z)
  {
    m_pt = Eigen::Vector3f(x, y, z);
  }

  bool CheckForShift(const Eigen::Affine3f & cam_pose,
                     const Eigen::Vector3f & center_cube) override
  {
    if ((center_cube - m_pt).squaredNorm() > FLT_EPSILON)
      return true;
    return false;
  }

  Eigen::Vector3f GetNewCubeOrigin(const Eigen::Affine3f &cam_pose) override
  {
    return m_pt;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  Eigen::Vector3f m_pt;
};

class RosserviceShiftChecker: public IShiftChecker
{
  public:
  RosserviceShiftChecker(const float time, const std::string service_name):
    m_nh("~")
  {
    m_last_pt = Eigen::Vector3f::Zero();

    m_last_check_time = ros::Time(0.0);

    m_check_duration = ros::Duration(time);

    ROS_INFO("kinfu: RosserviceShiftChecker: connecting to service %s", service_name.c_str());

    m_service_client = m_nh.serviceClient<kinfu_msgs::CheckForShift>(service_name);
  }

  bool CheckForShift(const Eigen::Affine3f &cam_pose, const Eigen::Vector3f & center_cube) override
  {
    if (ros::Time::now() - m_last_check_time < m_check_duration)
      return false;

    m_last_check_time = ros::Time::now();

    kinfu_msgs::CheckForShift srv;
    tf::poseEigenToMsg(cam_pose.cast<double>(), srv.request.cam_pose);
    srv.request.current_center.x = center_cube.x();
    srv.request.current_center.y = center_cube.y();
    srv.request.current_center.z = center_cube.z();

    if (!m_service_client.call(srv))
    {
      ROS_ERROR("kinfu: RosserviceShiftChecker: unable to call service!");
      return false;
    }

    const bool do_shift = srv.response.do_shift;
    if (do_shift)
    {
      m_last_pt.x() = srv.response.new_center.x;
      m_last_pt.y() = srv.response.new_center.y;
      m_last_pt.z()= srv.response.new_center.z;
    }

    return do_shift;
  }

  Eigen::Vector3f GetNewCubeOrigin(const Eigen::Affine3f &cam_pose) override
  {
    return m_last_pt;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  Eigen::Vector3f m_last_pt;
  ros::Time m_last_check_time;

  ros::NodeHandle m_nh;
  ros::ServiceClient m_service_client;
  ros::Duration m_check_duration;
};

IShiftChecker::Ptr KinfuShiftCheckerFactory::BuildShiftChecker(const std::string description)
{
  std::istringstream istr(description);
  std::string name;
  istr >> name;
  if (!istr)
  {
    ROS_ERROR("kinfu: BuildShiftChecker: unable to parse description string \"%s\", using default.", description.c_str());
    return IShiftChecker::Ptr();
  }

  if (name == PARAM_VALUE_SHIFT_CHECKER_DEFAULT)
    return IShiftChecker::Ptr();
  else if (name == PARAM_VALUE_SHIFT_CHECKER_NEVER)
  {
    ROS_INFO("kinfu: BuildShiftChecker: kinfu will never shift.");
    return IShiftChecker::Ptr(new NeverShiftChecker);
  }
  else if (name == PARAM_VALUE_SHIFT_CHECKER_FIXED)
  {
    float x, y, z;
    istr >> x >> y >> z;
    if (!istr)
    {
      ROS_ERROR("kinfu: BuildShiftChecker: %s: three float values expected (\"%s\").", name.c_str(), description.c_str());
      return IShiftChecker::Ptr();
    }

    ROS_INFO("kinfu: BuildShiftChecker: TSDF volume always centered at (%f, %f, %f).", float(x), float(y), float(z));

    return IShiftChecker::Ptr(new FixedShiftChecker(x, y, z));
  }
  else if (name == PARAM_VALUE_SHIFT_CHECKER_ROS)
  {
    float time;
    istr >> time;
    std::string service_name;
    istr >> service_name;

    if (!istr)
    {
      ROS_ERROR("kinfu: BuildShiftChecker: %s: float and string expected (\"%s\").", name.c_str(), description.c_str());
      return IShiftChecker::Ptr();
    }

    return IShiftChecker::Ptr(new RosserviceShiftChecker(time, service_name));
  }
  else
  {
    ROS_ERROR("kinfu: BuildShiftChecker: unknown name %s in description string \"%s\", using default.",
              name.c_str(), description.c_str());
    return IShiftChecker::Ptr();
  }
}
