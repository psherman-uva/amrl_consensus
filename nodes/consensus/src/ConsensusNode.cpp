
#include <nodes/consensus/ConsensusNode.hpp>

#include <cmath>
#include <algorithm>

namespace amrl {

ConsensusNode::ConsensusNode(void)
  : _js(_nh),
  _display(_nh, "origin_frame", "formation", "agents"),
  _num_robots(4),
  _active(false)
{
  boost::function<void(const sensor_msgs::Joy::ConstPtr&)> joy_cb = boost::bind(&ConsensusNode::joy_callback, this, _1);
  _js.subscribe_custom_callback(_nh, joy_cb, "/joy");

  for(size_t i = 0; i < _num_robots; ++i) {
    std::string rbt_str = "robot" + std::to_string(i);
    std::shared_ptr<RobotInterface> rbt_int(std::make_shared<RobotInterface>(_nh, rbt_str));
    _rbt_inter.push_back(rbt_int);
  }

  _setup_tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::setup, this);
}

void ConsensusNode::control_loop_callback(const ros::TimerEvent&)
{

}

void ConsensusNode::display_loop_callback(const ros::TimerEvent&)
{
  static constexpr double freq = 2*M_PI / 5.0;
  
  ros::Duration dt = ros::Time::now() - _start_time;
  double tm        = dt.toSec();

  std::vector<Eigen::Vector3d> form({
    {0.4, 0.0, 0.9},
    {0.0, 0.4, 1.1},
    {-0.4, 0.0, 0.7},
    {0.0, -0.4, 1.4}});

  for(size_t i = 0; i < _num_robots; ++i) {
    _display.update_markers(_rbt_inter[i]->pose_get(), form[i], i*2);
  }
  _display.publish_markers();
}

void ConsensusNode::setup(const ros::TimerEvent&)
{ 
  if(std::all_of(_rbt_inter.begin(), _rbt_inter.end(), 
    [](const std::shared_ptr<RobotInterface> &ri) { return ri->pose_ready(); })) {
    
    Eigen::Vector3d pos0 = _rbt_inter[0]->pose_get();
    Eigen::Vector3d form0({0.4, 0.0, 0.9});
    std::array<float, 3> clr0({1.0, 0.2, 0.4});
    _display.add_markers(pos0, form0, clr0);

    Eigen::Vector3d pos1 = _rbt_inter[1]->pose_get();
    Eigen::Vector3d form1({0.0, 0.4, 1.1});
    std::array<float, 3> clr1({0.2, 0.7, 0.3});
    _display.add_markers(pos1, form1, clr1);

    Eigen::Vector3d pos2 = _rbt_inter[2]->pose_get();
    Eigen::Vector3d form2({-0.4, 0.0, 0.7});
    std::array<float, 3> clr2({0.0, 0.4, 0.9});
    _display.add_markers(pos2, form2, clr2);

    Eigen::Vector3d pos3 = _rbt_inter[3]->pose_get();
    Eigen::Vector3d form3({0.0, -0.4, 1.4});
    std::array<float, 3> clr3({0.9, 0.5, 0.2});
    _display.add_markers(pos3, form3, clr3);

    _setup_tmr.stop();
    _cmd_tmr     = _nh.createTimer(ros::Duration(kLoopPeriod_s), &ConsensusNode::control_loop_callback, this);
    _display_tmr = _nh.createTimer(ros::Duration(kCmdLoopPeriod_s), &ConsensusNode::display_loop_callback, this);
    _start_time  = ros::Time::now();

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
