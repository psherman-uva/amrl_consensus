
#include <libs/interface/RobotInterface.hpp>


namespace amrl {


RobotInterface::RobotInterface(
  ros::NodeHandle &nh, 
  const std::string &robot_str)
  : _nh(nh),
  _robot_str(robot_str)
{
  std::string cmd_topic   = "/" + robot_str + "/cmd";
  std::string pose_topic  = "/" + robot_str + "/pose";
  std::string vel_topic   = "/" + robot_str + "/vel";

  ROS_INFO("Pose Topic: %s", pose_topic.c_str());

  _cmd_pub  = _nh.advertise<geometry_msgs::Vector3>(cmd_topic, 10);
  _pose_sub = _nh.subscribe(pose_topic, 2, &RobotInterface::pose_callback, this);
  _vel_sub  = _nh.subscribe(vel_topic, 2, &RobotInterface::velocity_callback, this);
}

void RobotInterface::command_publish(const Eigen::VectorXd &u)
{
  geometry_msgs::Vector3 msg;
  msg.x = u[0];
  msg.y = u[1];
  msg.z = u[2];

  _cmd_pub.publish(msg);
}

bool RobotInterface::pose_ready(void) const
{
  return _pose_received;
}

bool RobotInterface::velocity_ready(void) const
{
  return _vel_received;
}

Eigen::VectorXd RobotInterface::pose_get(void) const
{
  return Eigen::Vector<double, 3>({_pose.x, _pose.y, _pose.z});
}

Eigen::VectorXd RobotInterface::velocity_get(void) const
{
  return Eigen::Vector<double, 3>({_vel.x, _vel.y, _vel.z});
}

void RobotInterface::pose_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  _pose          = *msg;
  _pose_received = true;
}

void RobotInterface::velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  _vel          = *msg;
  _vel_received = true;
}

}