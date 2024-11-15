
#include <libs/sim/SimulationRobot.hpp>

namespace amrl {

SimulationRobot::SimulationRobot(
  ros::NodeHandle &nh,
  const std::string &robot_str,
  const Eigen::Vector<double, 6> &x0)
  : _nh(nh), 
  _plant(nullptr),
  _x(x0),
  _u(Eigen::Vector<double, 3>::Zero())
{
  Eigen::Matrix<double, 6, 6> A(Eigen::Matrix<double, 6, 6>::Zero());
  A(0, 3) = 1.0;
  A(1, 4) = 1.0;
  A(2, 5) = 1.0;

  Eigen::Matrix<double, 6, 3> B(Eigen::Matrix<double, 6, 3>::Zero());
  B(3, 0) = 1.0;
  B(4, 1) = 1.0;
  B(5, 2) = 1.0;

  _plant = std::make_shared<Plant<6, 3>>(A, B);
  _plant->state_set(_x);

  std::string cmd_topic = "/" + robot_str + "/cmd";
  _control_sub = _nh.subscribe(cmd_topic, 2, &SimulationRobot::control_callback, this);

  std::string pose_topic = "/" + robot_str + "/pose";
  _pose_pub = _nh.advertise<geometry_msgs::Vector3>(pose_topic, 10);  

  std::string vel_topic = "/" + robot_str + "/vel";
  _vel_pub = _nh.advertise<geometry_msgs::Vector3>(vel_topic, 10);
}

void SimulationRobot::cycle(double dt)
{
  _x = _plant->cycle(_u, dt);

  _pose_msg.x = _x[0];
  _pose_msg.y = _x[1];
  _pose_msg.z = _x[2];
  _pose_pub.publish(_pose_msg);

  _vel_msg.x = _x[3];
  _vel_msg.y = _x[4];
  _vel_msg.z = _x[5];
  _vel_pub.publish(_vel_msg);
}

Eigen::Vector<double, 3> SimulationRobot::pos(void) const
{
  return _x(Eigen::seqN(0, 3));
}

Eigen::Vector<double, 3> SimulationRobot::vel(void) const
{
  return _x(Eigen::seqN(3, 3));
}

void SimulationRobot::control_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  _u[0] = msg->x;
  _u[1] = msg->y;
  _u[2] = msg->z;
}

}