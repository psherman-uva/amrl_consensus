/*
    File: FormationConsensus.hpp
    Authors: pdsherman
    Date: March 2024

    Description: Implementation of Formation consensus algorithm
*/

#pragma once

#include <Eigen/Dense>
#include <amrl_common/ode_solver/RungeKutta.hpp>

namespace amrl {

class FormationConsensus
{
public:
  using R_t = Eigen::Matrix<double, 8, 1>;
  using V_t = Eigen::Matrix<double, 8, 1>;
  using A_t = Eigen::Matrix<double, 8, 1>;
  using X_t = Eigen::Matrix<double, 16, 1>;

  /// Constructor
  /// @param x0 Initial information state
  /// @param conns Connections between robots in multi-robot system
  FormationConsensus(const R_t &r_init, const std::vector<std::pair<int, int>> &conns);

  /// Default constructor
  FormationConsensus(void);

  // Default destructor
  ~FormationConsensus(void) = default;

  Eigen::Matrix<double, 8, 1> control_update(
    const R_t &r, 
    const V_t &v, 
    const A_t &a, 
    const double dt);

  Eigen::Matrix<double, 8, 1> center(void) const;

private:

  X_t x_dot(const X_t &x, const Eigen::Matrix<double, 0, 1> &u) const;
  void update_sub_states(void);

  // ---------------------------------- //
  // ------ End of Class Methods ------ //
  // ---------------------------------- //

  double _t;  // Simulation Time

  X_t _x;        // Full information state [xi, zeta]'
  R_t _xi;       // Xi   = r_i - r_iF
  V_t _zeta;     // Zeta = v_i - r'_iF
  V_t _rdot_F;  // r_iF dot
  A_t _rddot_F; // r_iF double-dot

  Eigen::Matrix<double, 4, 4>   _L;     ///< Laplacian
  Eigen::Matrix<double, 8, 8>   _sig; 
  Eigen::Matrix<double, 16, 16> _Sigma;


  static constexpr double kGamma = 2.5;
  static constexpr double kAlpha = 1.5;
  static constexpr int _n = 4;
  static constexpr int _m = 2;

  static const R_t r_iF;

  static const Eigen::Matrix<double, _n, _n> In;
  static const Eigen::Matrix<double, _m, _m> Im;
  static const Eigen::Matrix<double, 0, 1> kU0;


  /// ODE Solver for system
  RungeKutta<16, 0> _solver;
};


}