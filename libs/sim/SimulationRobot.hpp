
#pragma once

#include <libs/sim/Plant.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

#include <memory>

namespace amrl {

class SimulationRobot
{
public:

  SimulationRobot(
    ros::NodeHandle &nh,
    const std::string &robot_str,
    const Eigen::Vector<double, 6> &x0);

  ~SimulationRobot(void) = default;

  void cycle(double dt);
  Eigen::Vector<double, 3> pos(void) const;
  Eigen::Vector<double, 3> vel(void) const;
  
private:

  void control_callback(const geometry_msgs::Vector3::ConstPtr &msg);

  ros::NodeHandle _nh;
  ros::Subscriber _control_sub;
  ros::Publisher _pose_pub;
  ros::Publisher _vel_pub;

  geometry_msgs::Vector3 _pose_msg;
  geometry_msgs::Vector3 _vel_msg;

  std::shared_ptr<Plant<6, 3>> _plant;
  Eigen::Vector<double, 6> _x;
  Eigen::Vector<double, 3> _u;
};


}
