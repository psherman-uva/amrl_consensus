#include <libs/formation/FormationSupervisor.hpp>


namespace amrl {

FormationSupervisor::FormationSupervisor(const uint32_t num_robots)
  : _n(num_robots)
{
  _r_F     = Eigen::VectorXd::Zero(3 * _n);
  _r_F_dot = Eigen::VectorXd::Zero(3 * _n);

  formation_pose_vector(0.0);
}

Eigen::Vector3d FormationSupervisor::vel_desired(double t)
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d FormationSupervisor::vel_int_desired(double t)
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d FormationSupervisor::vel_dot_desired(double t)
{
  return Eigen::Vector3d::Zero();
}

Eigen::VectorXd FormationSupervisor::formation_pose_vector(const double t)
{
  // X - components
  _r_F[0] = -1.0;
  _r_F[1] = -1.0;
  _r_F[2] =  1.0;
  _r_F[3] =  1.0;

  // Y - components
  _r_F[4] =  1.0;
  _r_F[5] = -1.0;
  _r_F[6] = -1.0;
  _r_F[7] =  1.0;

  // Z - components
  // Keep zero for now

  return _r_F;
}

Eigen::VectorXd FormationSupervisor::formation_velocity_vector(const double t)
{
  return _r_F_dot; // Keep all zeros for now.
}

Eigen::Vector3d FormationSupervisor::robot_formation_pose_vector(uint32_t id)
{
  return Eigen::Vector3d({_r_F[id], _r_F[_n + id], _r_F[2*_n + id]});
}

Eigen::Vector3d FormationSupervisor::robot_formation_velocity_vector(uint32_t id)
{
  return Eigen::Vector3d({_r_F_dot[id], _r_F_dot[_n + id], _r_F_dot[2*_n + id]});
}

}