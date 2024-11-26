


#include <nodes/consensus/ConsensusNode.hpp>

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

// Eigen::VectorXd test_script()



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "consensus");
  amrl::ConsensusNode node;
  ros::spin();

  // using namespace amrl;

  // uint32_t num_robots = 4;
  // uint32_t kNumStates = 3;

  // Eigen::VectorXd r_init(Eigen::VectorXd::Zero(num_robots * kNumStates));
  // r_init[0]  = -1.25;
  // r_init[1]  = -1.25;
  // r_init[2]  =  1.25;
  // r_init[3]  =  1.25;
  // r_init[4]  =  1.25;
  // r_init[5]  = -1.25;
  // r_init[6]  = -1.25;
  // r_init[7]  =  1.25;
  // r_init[8]  =  0.60;
  // r_init[9]  =  0.80;
  // r_init[10] =  1.00;
  // r_init[11] =  1.20;

  // std::vector<std::pair<int, int>> conns({
  //   {0, 1}, {0, 3}, {1, 2},
  //   {2, 1}, {2, 3}, {3, 0}
  // });
  // double alpha = 1.5;
  // double gamma = 2.0;

  // std::shared_ptr<FormationSupervisor> formation = 
  //   std::make_shared<FormationSupervisor>(num_robots );
  // std::shared_ptr<FormationConsensus> consensus = 
  //   std::make_shared<FormationConsensus>(
  //     num_robots,
  //     kNumStates,
  //     r_init,
  //     alpha,
  //     gamma,
  //     conns,
  //     formation
  //   );

  // double dt = 0.05;
  // Eigen::VectorXd r(Eigen::VectorXd::Zero(num_robots * kNumStates));
  // Eigen::VectorXd v(Eigen::VectorXd::Zero(num_robots * kNumStates));
  // Eigen::VectorXd a(Eigen::VectorXd::Zero(num_robots * kNumStates));

  // Eigen::VectorXd x0 = consensus->full_state();
  // for(size_t i = 0; i < 200; ++i) {
  //   consensus->control_update(r, v, a, dt);
  // }
  // Eigen::VectorXd x1 = consensus->full_state();
  
  
  // std::cout << "x(0): \n" << x0 << std::endl;
  // std::cout << "x(1): \n" << x1 << std::endl;

  // ros::Time::init();
  // ros::Rate(5).sleep();
  // ros::Rate(5).sleep();
  // ros::Rate(5).sleep();
  // ros::Rate(5).sleep();
  // ros::Rate(5).sleep();




  return 0;
}
