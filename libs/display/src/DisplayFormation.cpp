
#include <libs/display/DisplayFormation.hpp>


namespace amrl {

DisplayFormation::DisplayFormation(
ros::NodeHandle &nh, 
  const std::string &frame_id, 
  const std::string &marker_ns,
  const std::string &pub_namespace)
:
  _nh(nh),
  _frame_id(frame_id),
  _marker_ns(marker_ns),
  _pub_namespace(pub_namespace)
{
  _rviz_pub = _nh.advertise<visualization_msgs::Marker>(pub_namespace + "/consensus_markers", 10);
}

void DisplayFormation::publish_markers(void)
{
  for(const auto &mkr : _markers) {
    _rviz_pub.publish(mkr);
  }
}

uint32_t DisplayFormation::add_markers(
  const Eigen::Vector3d &rbt_pos,
  const Eigen::Vector3d &formation_pos,
  const std::array<float, 3> &bdy_clr)
{
  uint32_t n = _markers.size();

  visualization_msgs::Marker mkr;
  mkr.header.frame_id = _frame_id;
  mkr.header.stamp    = ros::Time::now();
  mkr.ns              = _marker_ns;
  mkr.id              = n;
  mkr.type            = visualization_msgs::Marker::SPHERE;
  mkr.action          = visualization_msgs::Marker::ADD;
  mkr.lifetime        = ros::Duration();

  // Set the pose of the marker
  mkr.pose.position.x = rbt_pos[0];
  mkr.pose.position.y = rbt_pos[1];
  mkr.pose.position.z = rbt_pos[2];
  mkr.pose.orientation.x = 0.0;
  mkr.pose.orientation.y = 0.0;
  mkr.pose.orientation.z = 0.0;
  mkr.pose.orientation.w = 1.0;

  // Scale size
  mkr.scale.x = 0.4;
  mkr.scale.y = 0.4;
  mkr.scale.z = 0.4;

  // Color
  mkr.color.r = bdy_clr[0];
  mkr.color.g = bdy_clr[1];
  mkr.color.b = bdy_clr[2];
  mkr.color.a = 1.0f;

  _markers.push_back(mkr);

  //
  // Formation Center
  //
  mkr.id = n + 1;

  mkr.pose.position.x = formation_pos[0];
  mkr.pose.position.y = formation_pos[1];
  mkr.pose.position.z = formation_pos[2];

  mkr.scale.x = 0.1;
  mkr.scale.y = 0.1;
  mkr.scale.z = 0.1;

  mkr.color.a = 0.5f;

  _markers.push_back(mkr);

  return n;
}

void DisplayFormation::update_markers(
    const Eigen::Vector3d &rbt_pos,
    const Eigen::Vector3d &formation_pos,
    uint32_t idx)
{
  if(idx+1 < _markers.size()) {
    _markers[idx].pose.position.x = rbt_pos[0];
    _markers[idx].pose.position.y = rbt_pos[1];
    _markers[idx].pose.position.z = rbt_pos[2];

    _markers[idx+1].pose.position.x = formation_pos[0];
    _markers[idx+1].pose.position.y = formation_pos[1];
    _markers[idx+1].pose.position.z = formation_pos[2];
  }
}

}