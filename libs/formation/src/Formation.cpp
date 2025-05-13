#include <libs/formation/Formation.hpp>

namespace amrl {

Formation::Formation(const uint32_t num_robots)
  : _n(num_robots)
{
}

Eigen::Vector3d Formation::robot_pose_vector(uint32_t id, double t)
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d Formation::robot_velocity_vector(uint32_t id, double t)
{
  return Eigen::Vector3d::Zero();
}

}