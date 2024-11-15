/*
  @file:      Plant.hpp
  @author:    psherman
  @date       Feb 2023
*/
#pragma once

#include <amrl_common/ode_solver/RungeKutta.hpp>

#include <Eigen/Dense>

namespace amrl {

template <size_t N, size_t M>
class Plant {
public:
  // Syntax Convenience for state and input value
  using X_t = Eigen::Vector<double, N>;
  using U_t = Eigen::Vector<double, M>;

  /// Constructor
  /// @param A 
  /// @param B 
  Plant(const Eigen::Matrix<double, N, N> &A,
    const Eigen::Matrix<double, N, M> &B);

  /// Default destructor
  ~Plant(void) = default;

  /// Simulate the system forward in time by one time-step
  /// @param u  Commanded input to the system during the cycle
  /// @param dt Time step to calculate system forward
  /// @return Output y of the system after the cycle
  X_t cycle(const U_t &u, const double dt);

  /// @return Current state y of the system
  X_t state(void) const;

  /// @return Last commanded input
  U_t input(void) const;

  /// Manually set the current value for the state
  /// @param x Value of state to set as current plant state
  void state_set(const X_t &x);

private:
  
  /// ODE for the system. dx/dt = f(x0, u)
  /// @param x0 state of the system
  /// @param u  commanded input to the system
  /// @return Value of the system differential
  X_t x_dot(const X_t &x0, const U_t &u) const;

  //
  // Class Members
  //
  X_t _x; ///< Current state of the system
  U_t _u; ///< Last commanded input to the system

  /// State Space Matrices
  Eigen::Matrix<double, N, N> _A; ///< System Description
  Eigen::Matrix<double, N, M> _B; ///< Input matrix

  /// ODE Solver for system
  RungeKutta<N, M> _solver;
};

template <size_t N, size_t M>
Plant<N, M>::Plant(const Eigen::Matrix<double, N, N> &A,
    const Eigen::Matrix<double, N, M> &B)
: _A(A), _B(B),
  _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
    RungeKutta<N, M>::SolverType::kFourthOrderOptimal)
{
  _x = X_t::Zero();
  _u = U_t::Zero();
}

template <size_t N, size_t M>
typename Plant<N, M>::X_t Plant<N, M>::cycle(const U_t &u, const double dt)
{
  _x = _solver.step(_x, u, dt);
  _u = u;

  return _x;
}

template <size_t N, size_t M>
typename Plant<N, M>::X_t Plant<N, M>::state(void) const
{
  return _x;
}

template <size_t N, size_t M>
typename Plant<N, M>::U_t Plant<N, M>::input(void) const
{
  return _u;
}

template <size_t N, size_t M>
void Plant<N, M>::state_set(const X_t &x)
{
  _x = x;
}

template <size_t N, size_t M>
typename Plant<N, M>::X_t Plant<N, M>::x_dot(const X_t &x0, const U_t &u) const
{
  return _A*x0 + _B*u;
}

} // namespace amrl