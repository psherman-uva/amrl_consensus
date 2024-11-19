
#include <libs/logging/ConsensusLogging.hpp>
#include <amrl_logging_util/util.hpp>

namespace amrl {


ConsensusLogging::ConsensusLogging(ros::NodeHandle &nh)
  : _nh(nh)
{
}

bool ConsensusLogging::setup(const uint32_t num_robots)
{
  const std::string kDataTopic = ros::this_node::getName() + "/consensus_data";

  bool drop_table = _nh.param<bool>("/logging/drop_table", true);
  _table          = _nh.param<std::string>("/logging/table", "");


  if(_table.empty() || (drop_table && !amrl::logging_delete_table(_nh, _table))) { return false; }

  std::vector<std::string> label_header({"trial"});
  std::vector<std::string> int_header({"idx"});
  std::vector<std::string> real_header({"time"});
  for(uint32_t i = 0; i < num_robots; ++i) {
    real_header.push_back("x" + std::to_string(i));
    real_header.push_back("y" + std::to_string(i));
    real_header.push_back("z" + std::to_string(i));
    real_header.push_back("Fx" + std::to_string(i));
    real_header.push_back("Fy" + std::to_string(i));
    real_header.push_back("Fz" + std::to_string(i));
  }

  if(!amrl::logging_setup(_nh, _table, kDataTopic, label_header, int_header, real_header)) {
    return false;
  }

  _data.labels.resize(label_header.size(), "test");
  _data.nums.resize(int_header.size(), 0);
  _data.reals.resize(real_header.size(), 1.0);

  _pub = _nh.advertise<amrl_logging::LoggingData>(kDataTopic, 100);

  return true;
}

void ConsensusLogging::publish(void)
{
  _data.nums[0] += 1;
  _pub.publish(_data);
}

void ConsensusLogging::update_time(double time)
{
  _data.reals[0] = time;
}

void ConsensusLogging::update_robot_position(uint32_t idx, const Eigen::VectorXd &pose)
{
  _data.reals[(idx * 6) + 1] = pose[0];
  _data.reals[(idx * 6) + 2] = pose[1];
  _data.reals[(idx * 6) + 3] = pose[2];
}

void ConsensusLogging::update_formation_position(uint32_t idx, const Eigen::VectorXd &pose)
{
  _data.reals[(idx * 6) + 4] = pose[0];
  _data.reals[(idx * 6) + 5] = pose[1];
  _data.reals[(idx * 6) + 6] = pose[2];
}


}