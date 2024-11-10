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
  // using R_t = Eigen::Matrix<double, 8, 1>;
  // using V_t = Eigen::Matrix<double, 8, 1>;
  // using A_t = Eigen::Matrix<double, 8, 1>;
  // using X_t = Eigen::Matrix<double, 16, 1>;

  /// Constructor
  /// @param x0 Initial information state
  /// @param conns Connections between robots in multi-robot system
  FormationConsensus(const Eigen::VectorXd &r_init, const std::vector<std::pair<int, int>> &conns);

  /// Default constructor
  // FormationConsensus(void);

  // Default destructor
  ~FormationConsensus(void) = default;

  Eigen::VectorXd control_update(
    const Eigen::VectorXd &r, 
    const Eigen::VectorXd &v, 
    const Eigen::VectorXd &a, 
    const double dt);

  Eigen::VectorXd center(void) const;

private:

  Eigen::VectorXd x_dot(const Eigen::VectorXd &x, const Eigen::Matrix<double, 0, 1> &u) const;
  void update_sub_states(void);

  // ---------------------------------- //
  // ------ End of Class Methods ------ //
  // ---------------------------------- //

  double _t;  // Simulation Time

  Eigen::VectorXd _x;       // Full information state [xi, zeta]'
  Eigen::VectorXd _xi;      // Xi   = r_i - r_iF
  Eigen::VectorXd _zeta;    // Zeta = v_i - r'_iF
  Eigen::VectorXd _rdot_F;  // r_iF dot
  Eigen::VectorXd _rddot_F; // r_iF double-dot

  Eigen::MatrixXd _L;     ///< Laplacian
  Eigen::MatrixXd _sig; 
  Eigen::MatrixXd _Sigma;
  
  // X_t _x;        // Full information state [xi, zeta]'
  // R_t _xi;       // Xi   = r_i - r_iF
  // V_t _zeta;     // Zeta = v_i - r'_iF
  // V_t _rdot_F;  // r_iF dot
  // A_t _rddot_F; // r_iF double-dot

  // Eigen::Matrix<double, 4, 4>   _L;     ///< Laplacian
  // Eigen::Matrix<double, 8, 8>   _sig; 
  // Eigen::Matrix<double, 16, 16> _Sigma;
  

  const uint8_t _n = 4;
  static const Eigen::MatrixXd In;
  
  static constexpr int _m = 3;
  static const Eigen::Matrix<double, _m, _m> Im;

  static constexpr double kGamma = 2.5;
  static constexpr double kAlpha = 1.5;

  static const Eigen::VectorXd r_iF;

  static const Eigen::Matrix<double, 0, 1> kU0;

  /// ODE Solver for system
  RungeKutta<16, 0> _solver;
};


}