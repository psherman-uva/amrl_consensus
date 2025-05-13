/*
  @file:      Formation.hpp
  @author:    psherman
  @date       Dec. 2024
  
  @brief Abstract class defining interface for a desired formation
*/

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace amrl {

class Formation
{
public:

  Formation(const uint32_t num_robots);
  virtual ~Formation(void) = default;

  virtual Eigen::Vector3d robot_pose_vector(uint32_t id, double t);
  virtual Eigen::Vector3d robot_velocity_vector(uint32_t id, double t);

protected:

  uint32_t _n;
};

}