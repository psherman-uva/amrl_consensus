
#pragma once

#include <amrl_logging/LoggingData.h>

#include <ros/ros.h>
#include <Eigen/Dense>

namespace amrl {

class ConsensusLogging 
{
public:

  /// Constructor
  /// @param _nh ROS node handle obejct
  ConsensusLogging(ros::NodeHandle &nh); 

  /// Destructor
  ~ConsensusLogging(void) = default;

  /// Setup Logging
  bool setup(
    const uint32_t num_robots,
    const uint32_t num_states, 
    const std::string &formation);

  /// Log Data
  void publish(void);

  void update_time(double time);
  void update_robot_position(uint32_t idx, const Eigen::VectorXd &pose);
  void update_formation_position(uint32_t idx, const Eigen::VectorXd &pose);
  void update_robot_control(uint32_t idx, const Eigen::VectorXd &u);
  void update_consensus(const Eigen::VectorXd &xi_zeta);
  
private:

  // ROS Node Object
  ros::NodeHandle _nh;

  // ROS Publisher
  ros::Publisher _pub;

  // Logging topic publisher
  amrl_logging::LoggingData _data;

  // Database table name
  std::string _table;

  // Number of different items per robot to log
  uint32_t _robot_items;

  // Consensus starting idx
  uint32_t _consensus_idx;
};


}