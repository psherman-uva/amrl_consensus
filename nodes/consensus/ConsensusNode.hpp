
#pragma once

#include <libs/formation/FormationSupervisor.hpp>
#include <libs/display/DisplayFormation.hpp>
#include <libs/interface/RobotInterface.hpp>
#include <libs/logging/ConsensusLogging.hpp>

#include <amrl_common/joystick/Joystick.hpp>


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace amrl {


class ConsensusNode
{
public:
  /// Constructor
  ConsensusNode(void);

  /// Default Constructor
  ~ConsensusNode(void) = default;

private:

  void setup(const ros::TimerEvent&);

  void control_loop_callback(const ros::TimerEvent&);

  void display_loop_callback(const ros::TimerEvent&);

  void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
  
  // ROS Object
  ros::NodeHandle _nh;

  // Subscribe to joystick object
  ros::Subscriber _joy_sub;

  // Timer for control loop
  ros::Timer _cmd_tmr;

  // Timer for setup loop
  ros::Timer _setup_tmr;

  // Timer for display loop
  ros::Timer _display_tmr;

  // Custom wrapper for using joystick
  Joystick _js;

  // Formation definition
  std::shared_ptr<FormationSupervisor> _formation;

  // Interface between consensus node and real robots
  std::vector<std::shared_ptr<RobotInterface>> _rbt_inter;
  bool _rbt_setup_done;

  // Logging Object [nullptr if logging not needed]
  std::shared_ptr<ConsensusLogging> _logger;
  bool _logging_enabled;
  bool _logging_setup_done;

  // RViz Display
  std::shared_ptr<DisplayFormation> _display;
  std::vector<std::array<float, 3>> _display_clrs;
  bool _display_enabled;

  ros::Time _start_time;
  uint8_t _num_robots;
  bool _active;

  static constexpr double kLoopPeriod_s    = 0.2;
  static constexpr double kCmdLoopPeriod_s = 1.0; // 0.05;
};

}