
#include <libs/consensus/FormationConsensus.hpp>
#include <nodes/ConsensusNode.hpp>

#include <amrl_logging_util/util.hpp>
#include <amrl_logging/LoggingData.h>

#include <amrl_display/AddPoint.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>

// ---------------------------------- //
//     Local Function Declarations    //
// ---------------------------------- //

// void combine_robot_states(
//   const std::map<uint8_t, amrl::MultiRobot> &robots,
//   Eigen::Matrix<double, 8, 1> &x_rbt,
//   Eigen::Matrix<double, 8, 1> &v_rbt,
//   Eigen::Matrix<double, 8, 1> &a_rbt);

// void update_connections(
//   const std::map<int, std::map<int, ros::Publisher>> &connections,
//   const std::map<uint8_t, amrl::MultiRobot> &robots);

// std::map<int, ros::Publisher> setup_formation_center(
//   ros::NodeHandle &nh,
//   const amrl::FormationConsensus &consensus,
//   Eigen::Matrix<double, 8, 1> &x_rbt,
//   Eigen::Matrix<double, 8, 1> &r0);

// void update_formation_center(
//   const std::map<int, ros::Publisher> &pubs,
//   const Eigen::Matrix<double, 8, 1> &r0);

// void log_data_publish(
//   const double t,
//   const Eigen::Matrix<double, 8, 1> &r0,
//   const ros::Publisher &data_pub,
//   amrl_logging::LoggingData &data);

// ---------------------------------- //
// --      Local Variables         -- //
// ---------------------------------- //

static bool active   = false;
static bool done     = false;
static bool logging  = true;

static std::map<int, std::string> COLORS(
  {{0, "#af14de"},
   {1, "#187d2f"},
   {2, "#9c2308"},
   {3, "#0f5eb8"}});

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "consensus");

  amrl::ConsensusNode node;
  ros::spin();

  return 0;
  

  // //-----   Initialize Robot Objects   -----//

  // std::map<uint8_t, amrl::MultiRobot> robots;
  // std::vector<std::pair<int, int>> conns;
  // amrl::robot_only_setup_and_display(nh, robots, conns);
  // std::map<int, std::map<int, ros::Publisher>> conn_pubs = amrl::display_connections(nh, robots, conns);


  // // ----------------------------------------- //
  // // --         Setup logging Stuff         -- //
  // // ----------------------------------------- //
  // const std::string kDbTable  = "ConsensusTable";
  // const std::string kDbTopic  = "consensus_data";

  // ros::Publisher data_pub;
  // amrl_logging::LoggingData data;

  // if (logging) {
  //   const std::vector<std::string> kHeader(
  //     {"t", "x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3"});

  //   if(!amrl::logging_delete_table(nh, kDbTable)) { return 0; }
  //   if(!amrl::logging_setup(nh, kDbTable, kDbTopic, {}, kHeader)) { return 0; }

  //   data_pub = nh.advertise<amrl_logging::LoggingData>(kDbTopic, 10);
  
  //   data.labels = {};
  //   data.values = std::vector<double>(kHeader.size());

  //   for (int i = 0; i < 15; ++i) { ros::Rate(10).sleep(); } // Takes a second for logging to get setup
  // }



  // //-----   Main Loop   -----//

  // Eigen::Matrix<double, 8, 1> x_rbt; // Robot Positions
  // Eigen::Matrix<double, 8, 1> v_rbt; // Robot Velocities
  // Eigen::Matrix<double, 8, 1> a_rbt; // Robot Accelerations
  // Eigen::Matrix<double, 8, 1> r0(Eigen::Matrix<double, 8, 1>::Zero());

  // combine_robot_states(robots, x_rbt, v_rbt, a_rbt);

  // amrl::FormationConsensus consensus(x_rbt, conns);
  // Eigen::Matrix<double, 8, 1> u(Eigen::Matrix<double, 8, 1>::Zero());
  // Eigen::Vector2d u_rbt(Eigen::Vector2d::Zero());

  // std::map<int, ros::Publisher> r0_pubs = setup_formation_center(nh, consensus, x_rbt, r0);

  // r0 = consensus.center();
  // log_data_publish(0.0, r0, data_pub, data);


  // const double dt = 0.05;
  // ros::Rate loop_rate(1/dt);
  // double t = 0.0;
  // while (ros::ok() && !done) {
  //   ros::spinOnce(); // Run any subscriber callbacks in queue

  //   if (active) {
  //     u = consensus.control_update(x_rbt, v_rbt, a_rbt, dt);
  //   } else {
  //     u = Eigen::Matrix<double, 8, 1>::Zero();
  //   }

  //   // Update Robot Positions
  //   for(int i = 0; i < robots.size(); ++i)
  //   {
  //     u_rbt = u(Eigen::seq(i*2, i*2+1));
    
  //     robots.at(i).cycle(u_rbt, dt);
  //     robots.at(i).publish();
  //   }
  //   combine_robot_states(robots, x_rbt, v_rbt, a_rbt);
    
  //   r0 = consensus.center();
  //   update_connections(conn_pubs, robots);
  //   update_formation_center(r0_pubs, r0);




  //   if(active) {
  //     t += dt;
  //     log_data_publish(t, r0, data_pub, data);
  //   }

  //   loop_rate.sleep(); // Sleep until next period
  // }

  // if (active) { amrl::logging_finish(nh, kDbTable); }
}



// ---------------------------------- //
//     Local Function Definitions     //
// ---------------------------------- //

// void combine_robot_states(
//   const std::map<uint8_t, amrl::MultiRobot> &robots,
//   Eigen::Matrix<double, 8, 1> &x_rbt,
//   Eigen::Matrix<double, 8, 1> &v_rbt,
//   Eigen::Matrix<double, 8, 1> &a_rbt)
// {
//   x_rbt(Eigen::seq(0,1)) = robots.at(0).get_pos_state();
//   x_rbt(Eigen::seq(2,3)) = robots.at(1).get_pos_state();
//   x_rbt(Eigen::seq(4,5)) = robots.at(2).get_pos_state();
//   x_rbt(Eigen::seq(6,7)) = robots.at(3).get_pos_state();

//   v_rbt(Eigen::seq(0,1)) = robots.at(0).get_vel_state();
//   v_rbt(Eigen::seq(2,3)) = robots.at(1).get_vel_state();
//   v_rbt(Eigen::seq(4,5)) = robots.at(2).get_vel_state();
//   v_rbt(Eigen::seq(6,7)) = robots.at(3).get_vel_state();

//   a_rbt(Eigen::seq(0,1)) = robots.at(0).get_accel();
//   a_rbt(Eigen::seq(2,3)) = robots.at(1).get_accel();
//   a_rbt(Eigen::seq(4,5)) = robots.at(2).get_accel();
//   a_rbt(Eigen::seq(6,7)) = robots.at(3).get_accel();
// }

// void log_data_publish(
//   const double t,
//   const Eigen::Matrix<double, 8, 1> &r0,
//   const ros::Publisher &data_pub,
//   amrl_logging::LoggingData &data)
// {
//     data.values[0] = t;
//     data.values[1] = r0(0);
//     data.values[2] = r0(1);
//     data.values[3] = r0(2);
//     data.values[4] = r0(3);
//     data.values[5] = r0(4);
//     data.values[6] = r0(5);
//     data.values[7] = r0(6);
//     data.values[8] = r0(7);
//     data_pub.publish(data);
// }

// // void update_connections(
// //   const std::map<int, std::map<int, ros::Publisher>> &connections,
// //   const std::map<uint8_t, amrl::MultiRobot> &robots)
// // {
// //   amrl_display::LineSegment msg;
// //   for (const auto &conn1 : connections) {
// //     int idx1 = conn1.first;
// //     for(const auto &conn2 : conn1.second) {
// //       int idx2 = conn2.first;
// //       msg.pt1 = robots.at(idx1).get_position();
// //       msg.pt2 = robots.at(idx2).get_position();

// //       conn2.second.publish(msg);
// //     }
// //   }
// // }

// // std::map<int, ros::Publisher> setup_formation_center(
// //   ros::NodeHandle &nh,
// //   const amrl::FormationConsensus &consensus,
// //   Eigen::Matrix<double, 8, 1> &x_rbt,
// //   Eigen::Matrix<double, 8, 1> &r0)
// // {
// //   std::map<int, ros::Publisher> pt_pubs;

// //   r0 = consensus.center();

// //   ros::ServiceClient add_point = nh.serviceClient<amrl_display::AddPoint>("/display/add_point");

// //   if(add_point.waitForExistence(ros::Duration(5.0))) {
// //     for (int i = 0; i < 4; ++i) {
// //       amrl_display::AddPoint clt;
// //       clt.request.color      = COLORS[i];
// //       clt.request.topic_name = "/multi/center" + std::to_string(i);
// //       clt.request.pt_name    = "center" + std::to_string(i);
// //       clt.request.radius     = 0.1;
// //       clt.request.alpha      = 0.5;
// //       clt.request.pos.x      = r0(2*i);
// //       clt.request.pos.y      = r0(2*i+1);

// //       add_point.call(clt);

// //       pt_pubs[i] = nh.advertise<geometry_msgs::Point>(clt.request.topic_name, 1, false);
// //     }
// //   }

// //   return pt_pubs;
// // }

// // void update_formation_center(
// //   const std::map<int, ros::Publisher> &pubs,
// //   const Eigen::Matrix<double, 8, 1> &r0)
// // {
// //   geometry_msgs::Point msg;
// //   for (int i = 0; i < 4; ++i) {
// //     msg.x = r0(2*i);
// //     msg.y = r0(2*i + 1);
// //     pubs.at(i).publish(msg);
// //   }
// // }

