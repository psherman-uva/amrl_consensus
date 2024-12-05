
#include <nodes/consensus/ConsensusNode.hpp>

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "consensus");
  amrl::ConsensusNode node;
  ros::spin();

  return 0;
}
