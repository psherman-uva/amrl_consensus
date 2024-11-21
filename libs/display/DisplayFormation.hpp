/*
  @file:      DisplayObjects.hpp
  @author:    psherman
  @date       Nov. 2024
  
  @brief Class defining robots desired position/velocity around formation center
*/

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

namespace amrl
{

class DisplayFormation
{
public:

  DisplayFormation(
    ros::NodeHandle &nh, 
    const std::string &frame_id,
    const std::string &pub_namespace);

  ~DisplayFormation(void) = default;

  void add_markers(
    uint32_t idx,
    const Eigen::Vector3d &rbt_pos,
    const Eigen::Vector3d &formation_pos,
    const std::array<float, 3> &bdy_clr);

  void update_markers(
    uint32_t idx,
    const Eigen::Vector3d &rbt_pos,
    const Eigen::Vector3d &formation_pos);

  void publish_markers(void);

private:

  ros::NodeHandle _nh;
  std::string _frame_id;
  std::string _pub_namespace;

  // RViz visualization of robot body
  std::vector<visualization_msgs::Marker> _rbt_markers;
  std::vector<visualization_msgs::Marker> _frm_markers;
  ros::Publisher _rviz_pub;

};

}