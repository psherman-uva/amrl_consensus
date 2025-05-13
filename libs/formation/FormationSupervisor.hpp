/*
  @file:      RobotFormation.hpp
  @author:    psherman
  @date       Nov. 2024
  
  @brief Class defining robots desired position/velocity around formation center
*/

#pragma once

#include <libs/formation/Formation.hpp>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <vector>
#include <memory>
#include <map>

namespace amrl {

class FormationSupervisor
{
public:

  FormationSupervisor(ros::NodeHandle &nh, const uint32_t num_robots);
  ~FormationSupervisor(void) = default;

  Eigen::Vector3d vel_desired(double t);
  Eigen::Vector3d vel_int_desired(double t);
  Eigen::Vector3d vel_dot_desired(double t);

  Eigen::VectorXd formation_pose_vector(const double t);
  Eigen::VectorXd formation_velocity_vector(const double t);

  Eigen::Vector3d robot_formation_pose_vector(uint32_t id);
  Eigen::Vector3d robot_formation_velocity_vector(uint32_t id);

private:

  void setup_formation_plan(ros::NodeHandle &nh); 


  Eigen::VectorXd _r_F;
  Eigen::VectorXd _r_F_dot;

  std::vector<std::pair<double, std::unique_ptr<Formation>>> _formations;

  uint32_t _n;
  
  size_t _idx_curr;

  double _full_cycle_duration = 0.0;

};


}