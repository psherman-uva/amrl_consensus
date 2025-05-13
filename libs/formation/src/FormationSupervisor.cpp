#include <libs/formation/FormationSupervisor.hpp>
#include <libs/formation/FormationFromFile.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace amrl {

FormationSupervisor::FormationSupervisor(
    ros::NodeHandle &nh,
    const uint32_t num_robots)
: _n(num_robots),
  _full_cycle_duration(0.0)
{
  _r_F     = Eigen::VectorXd::Zero(3 * _n);
  _r_F_dot = Eigen::VectorXd::Zero(3 * _n);

  setup_formation_plan(nh);
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
  for(uint32_t i = 0; i < _n; ++i) {
    Eigen::Vector3d r_i = _formations[0].second->robot_pose_vector(i, t);
    _r_F[i]      = r_i[0];
    _r_F[i+_n]   = r_i[1];
    _r_F[i+2*_n] = r_i[2];
  }

  return _r_F;
}

Eigen::VectorXd FormationSupervisor::formation_velocity_vector(const double t)
{
  for(uint32_t i = 0; i < _n; ++i) {
    Eigen::Vector3d r_i_dot = _formations[0].second->robot_velocity_vector(i, t);
    _r_F_dot[i]      = r_i_dot[0];
    _r_F_dot[i+_n]   = r_i_dot[1];
    _r_F_dot[i+2*_n] = r_i_dot[2];
  }
  return _r_F_dot;
}

Eigen::Vector3d FormationSupervisor::robot_formation_pose_vector(uint32_t id)
{
  return _formations[0].second->robot_pose_vector(id, 0.0);
}

Eigen::Vector3d FormationSupervisor::robot_formation_velocity_vector(uint32_t id)
{
  return _formations[0].second->robot_velocity_vector(id, 0.0);
}

void FormationSupervisor::setup_formation_plan(ros::NodeHandle &nh)
{
  std::string formation_dir = nh.param<std::string>("/config/formation_directory", "");
  std::string plan_dir      = nh.param<std::string>("/config/plan_directory", "");
  std::string plan          = nh.param<std::string>("/consensus/plan", "");
  std::string plan_file     = plan_dir + plan + ".json";

  using json = nlohmann::json;
  std::ifstream f(plan_file);

  if (f.is_open()) {
    json data = json::parse(f)["plan"];
   
    for(json::iterator it = data.begin(); it != data.end(); ++it) {
      std::string ftype = it.value()["type"];
      double duration   = it.value()["duration"];
      
      std::unique_ptr<Formation> f;
      if(ftype == "file") {
        std::string filename = it.value()["filename"];
        filename = formation_dir + filename + ".json";
        f = std::make_unique<FormationFromFile>(_n, filename);
      } else {
        f = std::make_unique<Formation>(_n);
      }

      ROS_INFO("Adding formation to plan: %s", ftype.c_str());
      _formations.push_back({duration, std::move(f)});
      _full_cycle_duration += duration;
    }
  } else {
    ROS_WARN("Unable to open plan file: %s", plan_file.c_str());
  }
}


}