
#include <libs/interface/RobotInterface.hpp>


namespace amrl {

const std::vector<std::array<float, 3>> RobotInterface::kDisplayColors({
  {0.89, 0.10, 0.11},
  {0.22, 0.50, 0.72},
  {0.30, 0.69, 0.29},
  {0.60, 0.31, 0.64}});


RobotInterface::RobotInterface(
  uint32_t id,
  ros::NodeHandle &nh,
  std::shared_ptr<FormationSupervisor> formation,
  std::shared_ptr<DisplayFormation> display)
  : _id(id), 
  _nh(nh),
  _pose(Eigen::Vector3d::Zero()),
  _vel(Eigen::Vector3d::Zero()),
  _formation(formation),
  _display(display)
{
  _robot_str = "robot" + std::to_string(_id);
  std::string cmd_topic   = "/" + _robot_str + "/cmd";
  std::string pose_topic  = "/" + _robot_str + "/pose";
  std::string vel_topic   = "/" + _robot_str + "/vel";

  ROS_INFO("Pose Topic: %s", pose_topic.c_str());

  _cmd_pub  = _nh.advertise<geometry_msgs::Vector3>(cmd_topic, 10);
  _pose_sub = _nh.subscribe(pose_topic, 2, &RobotInterface::pose_callback, this);
  _vel_sub  = _nh.subscribe(vel_topic, 2, &RobotInterface::velocity_callback, this);
}

void RobotInterface::command_publish(const Eigen::VectorXd &u)
{
  geometry_msgs::Vector3 msg;
  msg.x = u[0];
  msg.y = u[1];
  msg.z = u[2];

  _cmd_pub.publish(msg);
}

void RobotInterface::update_display(void)
{
  if(_display) {
    _display->update_markers(_id, _pose, formation_pose_get());
  }
}

void RobotInterface::setup_display(void)
{
  if(_display) {
    _display->add_markers(_id, _pose, formation_pose_get(), kDisplayColors[_id]);
  }
}

bool RobotInterface::pose_ready(void) const
{
  return _pose_received;
}

bool RobotInterface::velocity_ready(void) const
{
  return _vel_received;
}

Eigen::VectorXd RobotInterface::pose_get(void) const
{
  return _pose;
}

Eigen::VectorXd RobotInterface::velocity_get(void) const
{
  return _vel;
}

Eigen::VectorXd RobotInterface::formation_pose_get(void)
{
  return _pose - _formation->robot_formation_pose_vector(_id);
}

void RobotInterface::pose_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  _pose[0] = msg->x;
  _pose[1] = msg->y;
  _pose[2] = msg->z;
  _pose_received = true;
}

void RobotInterface::velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  _vel[0] = msg->x;
  _vel[1] = msg->y;
  _vel[2] = msg->z;
  _vel_received = true;
}

}