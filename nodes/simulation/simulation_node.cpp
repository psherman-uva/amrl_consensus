
#include <nodes/simulation/SimulationNode.hpp>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simulation");
  amrl::SimulationNode node;
  ros::spin();
  return 0;
}