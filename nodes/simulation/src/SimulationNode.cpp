
#include <nodes/simulation/SimulationNode.hpp>

namespace amrl {

SimulationNode::SimulationNode(void)
  : _num_robots(4)
{
  Eigen::Vector<double, 6> x0(Eigen::Vector<double, 6>::Zero());

  for(uint8_t i = 0; i < _num_robots; ++i) {
    std::string rbt_str = "robot" + std::to_string(i);
    
    // if (i == 0) {
    //   x0[0] = -2.1;
    //   x0[1] =  1.48;
    //   x0[2] =  1.0;
    // } else if (i == 1) {
    //   x0[0] = -0.3;
    //   x0[1] = -0.6;
    //   x0[2] =  1.0;
    // } else if (i == 2) {
    //   x0[0] =  2.4;
    //   x0[1] =  0.4;
    //   x0[2] =  1.0;
    // } else if (i == 3) {
    //   x0[0] = 1.9;
    //   x0[1] = 0.8;
    //   x0[2] = 1.0;
    // } 

    if (i == 0) {
      x0[0] = -2.1;
      x0[1] =  1.48;
      x0[2] =  1.7;
    } else if (i == 1) {
      x0[0] = -0.3;
      x0[1] = -0.6;
      x0[2] =  0.3;
    } else if (i == 2) {
      x0[0] =  2.4;
      x0[1] =  0.4;
      x0[2] =  1.1;
    } else if (i == 3) {
      x0[0] = 1.9;
      x0[1] = 0.8;
      x0[2] = 0.4;
    } 

    

    std::shared_ptr<SimulationRobot> rbt(std::make_shared<SimulationRobot>(_nh, rbt_str, x0));
    _robots.push_back(rbt);
  }

  _tmr = _nh.createTimer(ros::Duration(kLoopPeriod_s), &SimulationNode::control_callback, this);
}

void SimulationNode::control_callback(const ros::TimerEvent&)
{
  for(auto &rbt : _robots) { rbt->cycle(kLoopPeriod_s); }
}

}