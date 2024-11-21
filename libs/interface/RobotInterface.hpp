
#pragma once

#include <libs/display/DisplayFormation.hpp>
#include <libs/formation/FormationSupervisor.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

namespace amrl {

class RobotInterface
{ 
public:

  RobotInterface(
    uint32_t id,
    ros::NodeHandle &nh,
    std::shared_ptr<FormationSupervisor> formation,
    std::shared_ptr<DisplayFormation> display);
  
  ~RobotInterface(void) = default;

  void command_publish(const Eigen::VectorXd &u);
  void update_display(void);
  void setup_display(void);

  bool pose_ready(void) const;
  bool velocity_ready(void) const;
  
  Eigen::VectorXd pose_get(void) const;
  Eigen::VectorXd velocity_get(void) const;
  
  Eigen::VectorXd formation_pose_get(void);

private:

  void pose_callback(const geometry_msgs::Vector3::ConstPtr &msg);
  void velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg);


  uint32_t _id;

  ros::NodeHandle _nh;

  ros::Subscriber _pose_sub;
  ros::Subscriber _vel_sub;
  ros::Publisher _cmd_pub;

  Eigen::Vector3d _pose;
  Eigen::Vector3d _vel;

  bool _pose_received;
  bool _vel_received;

  std::string _robot_str;

  std::shared_ptr<FormationSupervisor> _formation;
  std::shared_ptr<DisplayFormation> _display;

  static const std::vector<std::array<float, 3>> kDisplayColors;
};
 
}
