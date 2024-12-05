
#pragma once

#include <libs/consensus/FormationConsensus.hpp>
#include <libs/formation/FormationSupervisor.hpp>
#include <libs/display/DisplayFormation.hpp>
#include <libs/interface/RobotInterface.hpp>
#include <libs/logging/ConsensusLogging.hpp>

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

  void logging_update(void);
  
  // ROS Object
  ros::NodeHandle _nh;

  // Timer for control loop
  ros::Timer _cmd_tmr;

  // Timer for setup loop
  ros::Timer _setup_tmr;

  // Timer for display loop
  ros::Timer _display_tmr;

  // Formation definition
  std::shared_ptr<FormationSupervisor> _formation;

  // Interface between consensus node and real robots
  std::vector<std::shared_ptr<RobotInterface>> _rbt_inter;
  bool _rbt_setup_done;

  // Consensus Algorithm
  std::shared_ptr<FormationConsensus> _consensus;
  Eigen::VectorXd _r; // Robot posisitions
  Eigen::VectorXd _v; // Robot velocities

  // Logging Object [nullptr if logging not needed]
  std::shared_ptr<ConsensusLogging> _logger;
  bool _logging_enabled;
  bool _logging_setup_done;
  uint8_t _logging_setup_cnt;

  // RViz Display
  std::shared_ptr<DisplayFormation> _display;
  std::vector<std::array<float, 3>> _display_clrs;
  bool _display_enabled;

  ros::Time _start_time;
  double _time;
  uint8_t _num_robots;
  std::string _formation_label;

  static constexpr uint32_t kNumStates     = 3;   //< x, y, z
  static constexpr double kLoopPeriod_s    = 0.2;
  static constexpr double kCmdLoopPeriod_s = 0.1; // 0.05;
};

}