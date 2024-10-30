
#include <nodes/ConsensusNode.hpp>


namespace amrl {

ConsensusNode::ConsensusNode(void)
  : _js(_nh),
  _active(false)
{
  _tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), 
    &ConsensusNode::control_callback, this);

  boost::function<void(const sensor_msgs::Joy::ConstPtr&)> joy_cb = 
    boost::bind(&ConsensusNode::joy_callback, this, _1);
  _js.subscribe_custom_callback(_nh, joy_cb, "/joy");

}

void ConsensusNode::control_callback(const ros::TimerEvent&)
{

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
