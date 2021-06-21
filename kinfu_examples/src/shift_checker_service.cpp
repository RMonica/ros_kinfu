#include <ros/ros.h>
#include <kinfu_msgs/CheckForShift.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Dense>

bool onCheckForShift(kinfu_msgs::CheckForShift::Request  &req,
                     kinfu_msgs::CheckForShift::Response &res)
{
  std::cout << "onCheckForShift" << std::endl;
  std::cout << "cam_pose:\n" << req.cam_pose << "\n";
  std::cout << "current_center:\n" << req.current_center << "\n";

  Eigen::Vector3f center_cube(req.current_center.x, req.current_center.y, req.current_center.z);

  Eigen::Affine3d cam_pose_d;
  tf::poseMsgToEigen(req.cam_pose, cam_pose_d);
  Eigen::Affine3f cam_pose = cam_pose_d.cast<float>();

  Eigen::Vector3f targetPoint;
  targetPoint.x() = 0.0f;
  targetPoint.y() = 0.0f;
  targetPoint.z() = 1.5f;
  targetPoint = cam_pose * targetPoint;

  const float distance = (targetPoint - center_cube).norm();
  res.do_shift = distance > 1.0f;
  std::cout << "distance " << distance << " do shift: " << (res.do_shift ? "TRUE": "FALSE") << std::endl;;

  res.new_center.x = targetPoint.x();
  res.new_center.y = targetPoint.y();
  res.new_center.z = targetPoint.z();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_for_shift_server");
  ros::NodeHandle n("~");

  ros::ServiceServer service = n.advertiseService("/shift_checker_service", onCheckForShift);
  ros::spin();

  return 0;
}
