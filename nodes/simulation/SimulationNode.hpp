
#pragma once

#include <libs/sim/SimulationRobot.hpp>

#include <Eigen/Dense>
#include <ros/ros.h>

namespace amrl {

class SimulationNode 
{
public:
  SimulationNode(void);
  ~SimulationNode(void) = default;

private:

  void control_callback(const ros::TimerEvent&);

  ros::NodeHandle _nh;   ///< ROS node handle object
  ros::Timer _tmr;       ///< Timer for control loop

  uint8_t _num_robots;                               ///< Number of robots to simulate
  std::vector<std::shared_ptr<SimulationRobot>> _robots;

  static constexpr double kLoopPeriod_s = 0.05; ///< Control loop period
};


}