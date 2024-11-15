
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

namespace amrl {

class RobotInterface
{ 
public:

  RobotInterface(
    ros::NodeHandle &nh,
    const std::string &robot_str);
  ~RobotInterface(void) = default;

  void command_publish(const Eigen::VectorXd &u);

  bool pose_ready(void) const;
  bool velocity_ready(void) const;
  Eigen::VectorXd pose_get(void) const;
  Eigen::VectorXd velocity_get(void) const;

private:

  void pose_callback(const geometry_msgs::Vector3::ConstPtr &msg);
  void velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg);

  ros::NodeHandle _nh;

  ros::Subscriber _pose_sub;
  ros::Subscriber _vel_sub;
  ros::Publisher _cmd_pub;

  geometry_msgs::Vector3 _pose;
  geometry_msgs::Vector3 _vel;

  bool _pose_received;
  bool _vel_received;

  std::string _robot_str;
};
 
}
