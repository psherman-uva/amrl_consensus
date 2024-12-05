/*
 * @file:   RungeKutta.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class implementing basic 3rd and 4th order runge-kutta ODE solver
 */

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <iostream>

namespace amrl {

/// Numerical ODE solver using Runge-Kutta method
/// Class assumes system of first order ODEs
/// The solver format has been slightly modified from general RK formula as it's intended
/// to be used for simulation of controlled dynamic systems where
/// ODE input is a state variable (x) AND an input force (u) --> dx/dt = f(x, u)
/// Given:
///     State variable:             x
///     Input force:                u
///     Time-step:                  dt
///     Initial condition:          x0    = x(t0)
///     System of first order ODEs: dx/dt = f(x, u)
/// Result of RK method:
///     State one step forward:     x1    = x(t0 + dt)
///
/// Explicit RK formula used:
///    x1 = x0 + dt*SUM(b_i*f_i) for i = [1, ..., s]
/// Where
///    f_1 = f(x0, u)
///    f_i = f(x0 + dt*SUM(a_ij*f_j), u) for j = [1, ..., i-1]
/// 
/// Values of s, b_i, a_ij depend on choice of RK solver type and
/// were found in reference text listed below.
/// 
/// Reference:
///  Book Title: "Numerical Methods for Differential Equations: A Computational Approach"
///  Author: John R. Dormand
class RungeKutta
{
public:
  // Syntax Convenience for state and input value
  using X_t = Eigen::VectorXd;
  using U_t = Eigen::VectorXd;

  /// Options for RK solver
  enum class SolverType {
    kThirdOrder,
    kFourthOrderClassic,
    kFourthOrderOptimal
  };

  /// Constructor
  /// @param  ode First order diffential equation for x'.(x'=f(x, u))
  /// @param  type Option for what order of solver and solver coefficients
  RungeKutta(
    int num_states,
    std::function<X_t(const X_t&, const U_t&)> &&ode,
    const SolverType type = SolverType::kFourthOrderClassic);

  // Delete copy constructor. Can't guarentee
  // ODE function can always be copied/moved
  RungeKutta(const RungeKutta &rhs) = delete;

  /// Default destructor
  ~RungeKutta(void) = default;

  /// Calculate system one time step forward
  /// @param  x0 Current state of the system. x(t0)
  /// @param  u  Input command at current time. u(t0)
  /// @param  dt Time step to calculate system forward. dt
  /// @return State of the system one time step forward. x(t0+dt)
  Eigen::VectorXd step(const X_t &x0, const U_t &u, const double dt) const;

private:

  // State differential equation. x'(t) = f(x, u)
  std::function<X_t(const X_t&, const U_t&)> _ode_func;

  /// Number of stages. Depends on RK solver type
  int _s;

  /// Number of states
  int _N;

  /// Coefficients for RK formula
  Eigen::MatrixXd _a;
  Eigen::VectorXd _b;
};

} // namespace amrl