
#include <nodes/consensus/ConsensusNode.hpp>

#include <cmath>
#include <algorithm>

namespace amrl {

ConsensusNode::ConsensusNode(void)
  : _js(_nh),
  _display(nullptr),
  _active(false),
  _rbt_setup_done(false),
  _logger(nullptr),
  _logging_setup_done(false),
  _logging_setup_cnt(0),
  _start_time(0.0),
  _time(0.0)
{
  _num_robots      = _nh.param<int>("/consensus/num_robots", 4);
  _display_enabled = _nh.param<bool>("/display/enabled", false);
  _logging_enabled = _nh.param<bool>("/logging/enabled", false);

  if(_display_enabled) {
    _display = std::make_shared<DisplayFormation>(_nh, "origin_frame", ros::this_node::getName());
  }

  _formation = std::make_shared<FormationSupervisor>(_num_robots );

  for(size_t i = 0; i < _num_robots; ++i) {
    std::shared_ptr<RobotInterface> rbt_int(std::make_shared<RobotInterface>(i, _nh, _formation, _display));
    _rbt_inter.push_back(rbt_int);
  }
  Eigen::VectorXd r = Eigen::VectorXd::Zero(kNuStates * _num_robots);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNuStates * _num_robots);

  boost::function<void(const sensor_msgs::Joy::ConstPtr&)> joy_cb = boost::bind(&ConsensusNode::joy_callback, this, _1);
  _js.subscribe_custom_callback(_nh, joy_cb, "/joy");

  // Start looping setup function.
  _setup_tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::setup, this);
}

void ConsensusNode::control_loop_callback(const ros::TimerEvent&)
{  
  _time = (ros::Time::now() - _start_time).toSec();

  // Consensus
  Eigen::VectorXd r = Eigen::Vector<double, 12>::Zero();
  Eigen::VectorXd v = Eigen::Vector<double, 12>::Zero();
  Eigen::VectorXd a = Eigen::Vector<double, 12>::Zero();
  for(size_t i = 0; i < _num_robots; ++i) {
    Eigen::Vector3d pos = _rbt_inter[i]->pose_get();
    Eigen::Vector3d vel = _rbt_inter[i]->velocity_get();
    
    r[]
  }

  _consensus->control_update(r, v, a, kCmdLoopPeriod_s);

  // Log Data if enabled
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
        if(!_logger->setup(_num_robots)) { _logger = nullptr; }
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
    double alpha     = _nh.param<double>("/consensus/alpha", 1.5);
    double gamma     = _nh.param<double>("/consensus/gamma", 2.0);

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
      Eigen::Vector3d pos = _rbt_inter[i]->pose_get();
      _logger->update_robot_position(i, pos);
    }
    _logger->publish();
  }
}

void ConsensusNode::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
  _js.update_data(msg->axes, msg->buttons);
  
  if ((!_active) && _js.button_A()) {
    ROS_INFO("Button A");
    _active = true;
  } 

  if (_js.button_B()) {
    ROS_INFO("Shutting down node");
    ros::shutdown();
  } 
}

}
