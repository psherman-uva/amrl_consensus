
#include <libs/formation/FormationFromFile.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace amrl {

FormationFromFile::FormationFromFile(const uint32_t num_robots, const std::string &formation_file) 
  : Formation(num_robots)
{
  using json = nlohmann::json;
  std::ifstream f(formation_file);
  if (f.is_open()) {
    json data = json::parse(f);
    json formation = data["formation"];
    uint32_t idx = 0; 
    
    for(json::iterator it = formation.begin(); it != formation.end(); ++it) {
      double x = it.value()["x"];
      double y = it.value()["y"];
      double z = it.value()["z"];
      if(idx < _n) {
        Eigen::Vector3d pose(x, y, z);
        _poses[idx] = pose;
      }
      ++idx;
    }
  } else {
    std::cout << formation_file << ": File Not Open" << std::endl;
  }
}

Eigen::Vector3d FormationFromFile::robot_pose_vector(uint32_t id, double t)
{
  if(_poses.find(id) != _poses.end()) {
    return _poses.at(id);
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d FormationFromFile::robot_velocity_vector(uint32_t id, double t)
{
  return Eigen::Vector3d::Zero();
}


}