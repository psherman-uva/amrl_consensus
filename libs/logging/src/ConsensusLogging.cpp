
#include <libs/logging/ConsensusLogging.hpp>

#include <amrl_common/util/util.hpp>
#include <amrl_logging_util/util.hpp>



namespace amrl {


ConsensusLogging::ConsensusLogging(ros::NodeHandle &nh)
  : _nh(nh)
{
}

bool ConsensusLogging::setup(    
  const uint32_t num_robots,
  const uint32_t num_states, 
  const std::string &formation)
{
  const std::string kDataTopic = ros::this_node::getName() + "/consensus_data";

  bool drop_table = _nh.param<bool>("/logging/drop_table", true);
  _table          = _nh.param<std::string>("/logging/table", "");

  if(_table.empty() || (drop_table && !amrl::logging_delete_table(_nh, _table))) { return false; }

  std::vector<std::string> label_header({"formation"});
  std::vector<std::string> int_header({"idx"});
  std::vector<std::string> real_header({"time"});
  for(uint32_t i = 0; i < num_robots; ++i) {
    real_header.push_back("x" + std::to_string(i));
    real_header.push_back("y" + std::to_string(i));
    real_header.push_back("z" + std::to_string(i));
    real_header.push_back("Fx" + std::to_string(i));
    real_header.push_back("Fy" + std::to_string(i));
    real_header.push_back("Fz" + std::to_string(i));
    real_header.push_back("ux" + std::to_string(i));
    real_header.push_back("uy" + std::to_string(i));
    real_header.push_back("uz" + std::to_string(i));
  }
  _robot_items = 9; // Number of items to log per robot

  // Consensus Result
  _consensus_idx = _robot_items * num_robots + 1;
  uint32_t k_zeta = 0;
  uint32_t k_xi = 0;
  for(uint32_t i = 0; i < num_states; ++i) {
    for (uint32_t j = 0; j < num_robots; ++j) {
      real_header.push_back("xi" + std::to_string(k_xi));
      ++k_xi;  
    }
    for (uint32_t j = 0; j < num_robots; ++j) {
      real_header.push_back("zeta" + std::to_string(k_zeta));
      ++k_zeta;  
    }
  }

  if(!amrl::logging_setup(_nh, _table, kDataTopic, label_header, int_header, real_header)) {
    return false;
  }

  _data.labels.resize(label_header.size());
  _data.labels[0] = formation;

  _data.nums.resize(int_header.size(), 0);
  _data.reals.resize(real_header.size(), 1.0);

  _pub = _nh.advertise<amrl_logging::LoggingData>(kDataTopic, 100);

  return true;
}

void ConsensusLogging::publish(void)
{
  _pub.publish(_data);
  ++_data.nums[0];
}

void ConsensusLogging::update_time(double time)
{
  _data.reals[0] = time;
}

void ConsensusLogging::update_robot_position(uint32_t idx, const Eigen::VectorXd &pose)
{
  _data.reals[(idx * _robot_items) + 1] = pose[0];
  _data.reals[(idx * _robot_items) + 2] = pose[1];
  _data.reals[(idx * _robot_items) + 3] = pose[2];
}

void ConsensusLogging::update_formation_position(uint32_t idx, const Eigen::VectorXd &pose)
{
  _data.reals[(idx * _robot_items) + 4] = pose[0];
  _data.reals[(idx * _robot_items) + 5] = pose[1];
  _data.reals[(idx * _robot_items) + 6] = pose[2];
}

void ConsensusLogging::update_robot_control(uint32_t idx, const Eigen::VectorXd &u)
{
  _data.reals[(idx * _robot_items) + 7] = u[0];
  _data.reals[(idx * _robot_items) + 8] = u[1];
  _data.reals[(idx * _robot_items) + 9] = u[2];
}

void ConsensusLogging::update_consensus(const Eigen::VectorXd &xi_zeta)
{
  uint32_t N = xi_zeta.size();
  for(uint32_t i = 0; i < N; ++i) {
    _data.reals[_consensus_idx + i] = xi_zeta[i];
  }
}



}