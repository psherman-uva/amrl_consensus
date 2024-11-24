
#include <nodes/consensus/ConsensusNode.hpp>

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char* argv[])
{
  using namespace amrl;
  // ros::init(argc, argv, "consensus");
  // amrl::ConsensusNode node;
  // ros::spin();


  int num_robots = 4;
  int num_states = 3;
  double alpha   = 1.5;
  double gamma   = 2.0;
  std::vector<std::pair<int, int>> conns({
    {0, 1}, {0, 3},
    {1, 2},
    {2, 1}, {2, 3},
    {3, 0}
  });

  Eigen::VectorXd r_init(Eigen::Vector<double, 6>::Zero());

  std::shared_ptr<FormationSupervisor> formation;
  std::shared_ptr<FormationConsensus>  consensus;

  formation = std::make_shared<FormationSupervisor>(num_robots);
  consensus = std::make_shared<FormationConsensus>(
    num_robots, num_states, r_init, alpha, gamma, conns
  );

  return 0;
}



