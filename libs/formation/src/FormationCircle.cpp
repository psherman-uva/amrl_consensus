
#include <libs/formation/FormationCircle.hpp>

namespace amrl {


FormationCircle::FormationCircle(const uint32_t num_robots)
 : Formation(num_robots)
{

}

Eigen::Vector3d FormationCircle::robot_pose_vector(uint32_t id, double t)
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d FormationCircle::robot_velocity_vector(uint32_t id, double t)
{
  return Eigen::Vector3d::Zero();
}

}