/*
  @file:      RobotFormation.hpp
  @author:    psherman
  @date       Nov. 2024
  
  @brief Class defining robots desired position/velocity around formation center
*/

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace amrl {

class FormationSupervisor
{
public:

  FormationSupervisor(const uint32_t num_robots);
  ~FormationSupervisor(void) = default;

  Eigen::Vector3d vel_desired(double t);
  Eigen::Vector3d vel_int_desired(double t);
  Eigen::Vector3d vel_dot_desired(double t);

  Eigen::VectorXd formation_pose_vector(const double t);
  Eigen::VectorXd formation_velocity_vector(const double t);

  Eigen::Vector3d robot_formation_pose_vector(uint32_t id);
  Eigen::Vector3d robot_formation_velocity_vector(uint32_t id);

private:

  Eigen::VectorXd _r_F;
  Eigen::VectorXd _r_F_dot;

  uint32_t _n;
};


}