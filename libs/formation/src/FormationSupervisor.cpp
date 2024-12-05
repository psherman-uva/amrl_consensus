#include <libs/formation/FormationSupervisor.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace amrl {

FormationSupervisor::FormationSupervisor(const uint32_t num_robots)
  : _n(num_robots)
{
  _r_F     = Eigen::VectorXd::Zero(3 * _n);
  _r_F_dot = Eigen::VectorXd::Zero(3 * _n);

  formation_pose_vector(0.0);
}

void FormationSupervisor::initialize_from_json(const std::string &jsonfile)
{
  using json = nlohmann::json;
  std::ifstream f(jsonfile);
  if (f.is_open()) {
    json data = json::parse(f);
    json formation = data["formation"];
    uint32_t idx = 0; 
    
    for(json::iterator it = formation.begin(); it != formation.end(); ++it) {
      double x = it.value()["x"];
      double y = it.value()["y"];
      double z = it.value()["z"];
      if(idx < _n) {
        _r_F[idx]        = x;
        _r_F[idx + _n]   = y;
        _r_F[idx + 2*_n] = z;
      }
      ++idx;
    }
  } else {
    std::cout << jsonfile << ": File Not Open" << std::endl;
  }
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