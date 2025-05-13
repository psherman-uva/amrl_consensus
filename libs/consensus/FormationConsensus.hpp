/*
    File: FormationConsensus.hpp
    Authors: pdsherman
    Date: March 2024

    Description: Implementation of Formation consensus algorithm
*/

#pragma once

#include <libs/formation/FormationSupervisor.hpp>

#include <Eigen/Dense>
#include <memory>

namespace amrl {

class FormationConsensus
{
public:
  /// Convenience aliases
  using Vec_t = Eigen::VectorXd;
  using Mat_t = Eigen::MatrixXd;

  /// Constructor
  /// @param x0 Initial information state
  /// @param conns Connections between robots in multi-robot system
  FormationConsensus(
    const uint8_t num_robots,
    const uint8_t num_states,
    const Vec_t &r_init,
    const double alpha,
    const double gamma,
    const std::vector<std::pair<int, int>> &conns,
    std::shared_ptr<FormationSupervisor> formation);

  /// Default constructor
  // FormationConsensus(void);

  // Default destructor
  ~FormationConsensus(void) = default;

  Vec_t xi_zeta_dot(
    const Vec_t &r, 
    const Vec_t &v,
    const double t);

  Vec_t full_state(void) const;
  Vec_t xi(void) const;
  Vec_t zeta(void) const;

private:
  void update_sub_states(void);
  Vec_t x_dot(const Vec_t &x, const Eigen::Matrix<double, 0, 1> &u) const;

  // ---------------------------------- //
  // ------ End of Class Methods ------ //
  // ---------------------------------- //

  double _t; // Simulation Time
  const uint8_t _n;  // Number of robots
  const uint8_t _m;  // Number of states (e.g. x,y,z => 3)
  const uint8_t _nm; // (Number of robots) * (number of states)

  Vec_t _x;       ///< Full information state [xi, zeta]'
  Vec_t _xi;      ///< Xi   = r_i - r_iF
  Vec_t _zeta;    ///< Zeta = v_i - r'_iF

  Vec_t _rdot_F;  ///< r_iF dot
  Vec_t _rddot_F; ///< r_iF double-dot

  Mat_t _L;    ///< Laplacian
  Mat_t _sig;   
  Mat_t _Sigma;

  Mat_t _In;
  Mat_t _Im;

  const double _gamma;
  const double _alpha;

  /// Object that defines "desired" formation
  std::shared_ptr<FormationSupervisor> _formation;
};

}