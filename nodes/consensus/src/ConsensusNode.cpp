
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
  _logging_setup_done(false)
{
  double alpha     = _nh.param<double>("/consensus/alpha", 1.5);
  double gamma     = _nh.param<double>("/consensus/gamme", 2.0);
  _num_robots      = _nh.param<int>("/consensus/num_robots", 4);
  _display_enabled = _nh.param<bool>("/display/enabled", false);
  _logging_enabled = _nh.param<bool>("/logging/enabled", false);

  if(_display_enabled) {
    _display = std::make_shared<DisplayFormation>(_nh, "origin_frame", ros::this_node::getName());
  }

  _formation = std::make_shared<FormationSupervisor>(_num_robots  );

  for(size_t i = 0; i < _num_robots; ++i) {
    std::shared_ptr<RobotInterface> rbt_int(std::make_shared<RobotInterface>(i, _nh, _formation, _display));
    _rbt_inter.push_back(rbt_int);
  }

  boost::function<void(const sensor_msgs::Joy::ConstPtr&)> joy_cb = boost::bind(&ConsensusNode::joy_callback, this, _1);
  _js.subscribe_custom_callback(_nh, joy_cb, "/joy");

  _setup_tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::setup, this);

  // Consensus

}

void ConsensusNode::control_loop_callback(const ros::TimerEvent&)
{
  static constexpr double freq = 2*M_PI / 5.0;
  
  ros::Duration dt = ros::Time::now() - _start_time;
  double tm        = dt.toSec();

  // Log Data if enabled
  if(_logging_enabled) { 
    for(uint32_t i = 0; i < _num_robots; ++i) {
      Eigen::Vector3d pos = _rbt_inter[i]->pose_get();
      _logger->update_robot_position(i, pos);
    }
    _logger->publish();
  }
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
        _logging_setup_done = true;  
        ROS_INFO("Logger Setup Complete");
      }
    } else {
      _logging_setup_done = true;
      ROS_INFO("Logger Not Enabled");
    }
  }

  if(!_rbt_setup_done && std::all_of(_rbt_inter.begin(), _rbt_inter.end(), 
    [](const std::shared_ptr<RobotInterface> &ri) { return ri->pose_ready(); })) {
    
    if(_display) {
      for(auto rbt : _rbt_inter) { rbt->setup_display(); }
      _display_tmr = _nh.createTimer(ros::Duration(kCmdLoopPeriod_s), &ConsensusNode::display_loop_callback, this);
    }
    
    _rbt_setup_done = true;
  }

  if(_logging_setup_done && _rbt_setup_done) {
    _setup_tmr.stop();
    _cmd_tmr     = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::control_loop_callback, this);
    _start_time  = ros::Time::now() + ros::Duration(kLoopPeriod_s);

    ROS_INFO("Consensus Setup Complete");
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
