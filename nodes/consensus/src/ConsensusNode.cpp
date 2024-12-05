
#include <nodes/consensus/ConsensusNode.hpp>

#include <cmath>
#include <algorithm>

namespace amrl {

ConsensusNode::ConsensusNode(void)
  : _display(nullptr),
  _rbt_setup_done(false),
  _logger(nullptr),
  _logging_setup_done(false),
  _logging_setup_cnt(0),
  _start_time(0.0),
  _time(0.0)
{
  std::string config_dir = _nh.param<std::string>("/config/directory", "");

  _num_robots      = _nh.param<int>("/consensus/num_robots", 4);
  _display_enabled = _nh.param<bool>("/display/enabled", false);
  _logging_enabled = _nh.param<bool>("/logging/enabled", false);
  _formation_label = _nh.param<std::string>("/consensus/formation", "none");

  if(_display_enabled) {
    _display = std::make_shared<DisplayFormation>(_nh, "origin_frame", ros::this_node::getName());
  }

  _formation = std::make_shared<FormationSupervisor>(_num_robots );
  if((!config_dir.empty()) && _formation_label != "none") {
    std::string formation_file = config_dir + _formation_label + ".json";
    _formation->initialize_from_json(formation_file);
  }

  for(size_t i = 0; i < _num_robots; ++i) {
    std::shared_ptr<RobotInterface> rbt_int(std::make_shared<RobotInterface>(i, _nh, _formation, _display));
    _rbt_inter.push_back(rbt_int);
  }
  Eigen::VectorXd r = Eigen::VectorXd::Zero(kNumStates * _num_robots);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNumStates * _num_robots);

  // Start looping setup function.
  _setup_tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::setup, this);
}

void ConsensusNode::control_loop_callback(const ros::TimerEvent&)
{  
  _time = (ros::Time::now() - _start_time).toSec();

  // Consensus
  Eigen::VectorXd r_i(Eigen::VectorXd::Zero(kNumStates * _num_robots));
  Eigen::VectorXd v_i(Eigen::VectorXd::Zero(kNumStates * _num_robots));
  for(size_t i = 0; i < _num_robots; ++i) {
    Eigen::VectorXd r_temp = _rbt_inter[i]->pose_get();
    Eigen::VectorXd v_temp = _rbt_inter[i]->velocity_get();
    for (size_t j = 0; j < kNumStates; ++j) {
      r_i[_num_robots*j + i] = r_temp[j];
      v_i[_num_robots*j + i] = v_temp[j];
    }
  }

  Eigen::VectorXd x_dot = _consensus->xi_zeta_dot(r_i, v_i, _time);
  Eigen::VectorXd zeta_dot(kNumStates * _num_robots);
  for(int i = 0; i < kNumStates; ++i) {
    zeta_dot(Eigen::seqN(i*_num_robots, _num_robots)) = x_dot(Eigen::seqN(i*2*_num_robots + _num_robots, _num_robots));
  }

  for(size_t i = 0; i < _num_robots; ++i) {
    Eigen::VectorXd u_i(Eigen::VectorXd::Zero(kNumStates));
    for(size_t j = 0; j < kNumStates; ++j) {
      u_i[j] = zeta_dot[_num_robots*j + i];
    }
    _rbt_inter[i]->command_publish(u_i);
  }

  logging_update();

}

void ConsensusNode::display_loop_callback(const ros::TimerEvent&)
{
  for(auto rbt : _rbt_inter) { rbt->update_display(); }
  _display->publish_markers();
}

void ConsensusNode::setup(const ros::TimerEvent&)
{ 
  if (!_logging_setup_done) {
    if (_logging_enabled) {
      if (!_logger) {
        _logger = std::make_shared<ConsensusLogging>(_nh);
        if(!_logger->setup(_num_robots, kNumStates, _formation_label)) { _logger = nullptr; }
      } else if(_logging_setup_cnt < 10) {
        ++_logging_setup_cnt;
      } else {
        _logging_setup_done = true;  
        ROS_INFO("Logger Setup Complete");
      }
    } else {
      ROS_INFO("Skipping logging setup");
      _logging_setup_done = true;  
    }
  }

  if(!_rbt_setup_done && std::all_of(_rbt_inter.begin(), _rbt_inter.end(), 
    [](const std::shared_ptr<RobotInterface> &ri) { return ri->pose_ready(); })) {

    // Initialize consensus object
    std::vector<std::pair<int, int>> conns({
      {0, 1}, {0, 3},
      {1, 2},
      {2, 1}, {2, 3},
      {3, 0}
    });
    double alpha = _nh.param<double>("/consensus/alpha", 1.5);
    double gamma = _nh.param<double>("/consensus/gamma", 2.0);

    Eigen::VectorXd r_init(Eigen::VectorXd::Zero(_num_robots * kNumStates));
    for(uint32_t i = 0; i < _num_robots; ++i) {
      Eigen::VectorXd pose = _rbt_inter[i]->pose_get();
      r_init[i]                 = pose[0];
      r_init[i + _num_robots]   = pose[1];
      r_init[i + 2*_num_robots] = pose[2];
    }
    _consensus = std::make_shared<FormationConsensus>(
      _num_robots,
      kNumStates,
      r_init,
      alpha,
      gamma,
      conns,
      _formation
    );

    // Initialize Display robots
    if(_display) { for(auto rbt : _rbt_inter) { rbt->setup_display(); } }
    
    _rbt_setup_done = true;
  }

  if(_logging_setup_done && _rbt_setup_done) {
    _setup_tmr.stop();
    logging_update();

    if(_display) {
      _display_tmr = _nh.createTimer(ros::Duration(kCmdLoopPeriod_s), &ConsensusNode::display_loop_callback, this);
    }
    _cmd_tmr     = _nh.createTimer(ros::Duration(kCmdLoopPeriod_s), &ConsensusNode::control_loop_callback, this);
    _start_time  = ros::Time::now();

    ROS_INFO("Consensus Setup Complete");
  }
}

void ConsensusNode::logging_update(void)
{
  if(_logging_enabled) {
    _logger->update_time(_time);
    _logger->update_consensus(_consensus->full_state());
    for(uint32_t i = 0; i < _num_robots; ++i) {
      Eigen::Vector3d pos(_rbt_inter[i]->pose_get());
      Eigen::Vector3d rF(_formation->robot_formation_pose_vector(i));

      _logger->update_robot_position(i, pos);
      _logger->update_formation_position(i, pos - rF);
    }
    _logger->publish();
  }
}

}
