
#include <libs/formation/RobotFormation.hpp>


namespace amrl {

FormationRobot::FormationRobot(uint8_t num_robots)
  : _n(num_robots)
{
}

Eigen::VectorXd FormationRobot::position_from_center(const double t)
{
  Eigen::VectorXd r = Eigen::VectorXd::Zero(3*_n);

  // X - components
  r[0] = 0.5;
  r[1] = 0.5;
  r[2] = -0.5;
  r[3] = -0.5;

  // Y - components
  r[4] = 0.5;
  r[5] = -0.5;
  r[6] = -0.5;
  r[7] = 0.5;

  // Z - components
  // Keep zero for now

  return r;
}

Eigen::VectorXd FormationRobot::velocity_from_center(const double t)
{
  Eigen::VectorXd v = Eigen::VectorXd::Zero(3*_n);

  return v;
}


}