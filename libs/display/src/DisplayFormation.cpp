
#include <libs/display/DisplayFormation.hpp>


namespace amrl {

DisplayFormation::DisplayFormation(
  ros::NodeHandle &nh, 
  const std::string &frame_id, 
  const std::string &pub_namespace)
: _nh(nh),
  _frame_id(frame_id),
  _pub_namespace(pub_namespace)
{
  _rviz_pub = _nh.advertise<visualization_msgs::Marker>(pub_namespace + "/consensus_markers", 100);
}

void DisplayFormation::publish_markers(void)
{
  for(const auto &mkr : _rbt_markers) { _rviz_pub.publish(mkr); }
  for(const auto &mkr : _frm_markers) { _rviz_pub.publish(mkr); }
}

void DisplayFormation::add_markers(
  uint32_t idx,
  const Eigen::Vector3d &rbt_pos,
  const Eigen::Vector3d &formation_pos,
  const std::array<float, 3> &bdy_clr)
{
  visualization_msgs::Marker mkr;

  // Common traits
  mkr.header.frame_id = _frame_id;
  mkr.header.stamp    = ros::Time::now();
  mkr.id              = idx;
  mkr.type            = visualization_msgs::Marker::SPHERE;
  mkr.action          = visualization_msgs::Marker::ADD;
  mkr.lifetime        = ros::Duration();

  mkr.color.r = bdy_clr[0];
  mkr.color.g = bdy_clr[1];
  mkr.color.b = bdy_clr[2];

  mkr.pose.orientation.x = 0.0;
  mkr.pose.orientation.y = 0.0;
  mkr.pose.orientation.z = 0.0;
  mkr.pose.orientation.w = 1.0;

  // -------------------- //
  // --  Robot Marker  -- //
  // -------------------- //
  mkr.ns = "robot_mkr";

  // Set the pose of the marker
  mkr.pose.position.x = rbt_pos[0];
  mkr.pose.position.y = rbt_pos[1];
  mkr.pose.position.z = rbt_pos[2];

  // Scale size
  mkr.scale.x = 0.4;
  mkr.scale.y = 0.4;
  mkr.scale.z = 0.4;

  mkr.color.a = 1.0f;

  _rbt_markers.push_back(mkr);

  // -------------------- //
  // --  Robot Marker  -- //
  // -------------------- //
  mkr.ns = "formation_mkr";

  mkr.pose.position.x = formation_pos[0];
  mkr.pose.position.y = formation_pos[1];
  mkr.pose.position.z = formation_pos[2];

  mkr.scale.x = 0.1;
  mkr.scale.y = 0.1;
  mkr.scale.z = 0.1;

  mkr.color.a = 0.5f;

  _frm_markers.push_back(mkr);
}

void DisplayFormation::update_markers(
    uint32_t idx,
    const Eigen::Vector3d &rbt_pos,
    const Eigen::Vector3d &formation_pos)
{
  _rbt_markers[idx].pose.position.x = rbt_pos[0];
  _rbt_markers[idx].pose.position.y = rbt_pos[1];
  _rbt_markers[idx].pose.position.z = rbt_pos[2];

  _frm_markers[idx].pose.position.x = formation_pos[0];
  _frm_markers[idx].pose.position.y = formation_pos[1];
  _frm_markers[idx].pose.position.z = formation_pos[2];
}

}