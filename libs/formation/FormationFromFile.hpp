/*
  @file:      FormationFromFile.hpp
  @author:    psherman
  @date       Dec. 2024
  
  @brief Abstract class defining interface for a desired formation
*/

#pragma once

#include <libs/formation/Formation.hpp>
#include <map>

namespace amrl {

class FormationFromFile : public Formation
{
public:

  FormationFromFile(const uint32_t num_robots, const std::string &formation_file);
  ~FormationFromFile(void) = default;

  Eigen::Vector3d robot_pose_vector(uint32_t id, double t) override;
  Eigen::Vector3d robot_velocity_vector(uint32_t id, double t) override;

private:

  std::map<uint32_t, Eigen::Vector3d> _poses;
};


}