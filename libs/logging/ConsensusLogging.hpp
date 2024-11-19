
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
  bool setup(const uint32_t num_robots);

  /// Log Data
  void publish(void);

  void update_time(double time);
  void update_robot_position(uint32_t idx, const Eigen::VectorXd &pose);
  void update_formation_position(uint32_t idx, const Eigen::VectorXd &pose);

private:

  // ROS Node Object
  ros::NodeHandle _nh;

  // ROS Publisher
  ros::Publisher _pub;

  // Logging topic publisher
  amrl_logging::LoggingData _data;

  // Database table name
  std::string _table;
};


}