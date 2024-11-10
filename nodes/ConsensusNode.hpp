
#pragma once

#include <libs/consensus/FormationConsensus.hpp>

#include <amrl_common/joystick/Joystick.hpp>


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>

namespace amrl {

class ConsensusNode
{
public:
  /// Constructor
  ConsensusNode(void);

  /// Default Constructor
  ~ConsensusNode(void) = default;

private:

  void control_callback(const ros::TimerEvent&);

  void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
  
  // ROS Object
  ros::NodeHandle _nh;

  // Subscribe to joystick object
  ros::Subscriber _joy_sub;

  // Timer for control loop
  ros::Timer _tmr;

  // Custom wrapper for using joystick
  Joystick _js;

  // Formation control
  // FormationConsensus _consensus;


  bool _active;

  static constexpr double kLoopPeriod_s = 0.5;
};

}