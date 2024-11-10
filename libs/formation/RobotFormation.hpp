/*
  @file:      RobotFormation.hpp
  @author:    psherman
  @date       Nov. 2024
  
  @brief Class defining robots desired position/velocity around formation center
*/

#pragma once

#include <Eigen/Dense>

namespace amrl
{

class FormationRobot
{
public:

  /// Constructor
  /// @param [in] num_robots Number of robots to be included in formation
  FormationRobot(uint8_t num_robots);

  /// Destructor
  ~FormationRobot(void) = default;

  /// Get vector containing each robot's desired position relative
  /// to the formation center. Assuming x,y,z positions, the return vector
  /// will be in form: r_F = [x1, ..., xn, y1, ..., yn, z1, ..., zn]^T
  /// @return Position vector r_F defining target postions around formation center
  Eigen::VectorXd position_from_center(const double t);

  /// Get vector containing each robot's desired velocity relative
  /// to the formation center. Assuming x,y,z state, the return vector
  /// will be in form: v_F = [v_x1, ..., v_xn, v_y1, ..., v_yn, v_z1, ..., v_zn]^T
  /// @return Position vector r_F defining target velocities around formation center
  Eigen::VectorXd velocity_from_center(const double t);

private:

  uint8_t _n; /// Number of robot agents in formation
};


}