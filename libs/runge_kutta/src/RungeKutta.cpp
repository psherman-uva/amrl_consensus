
#include <libs/runge_kutta/RungeKutta.hpp>

namespace amrl {

RungeKutta::RungeKutta(
    int num_states,
    std::function<X_t(const X_t&, const U_t&)> &&ode,
    const SolverType type)
  : _ode_func(std::move(ode)),
    _N(num_states)
{
  switch(type) {
    case SolverType::kThirdOrder:
      _s = 3;
      _b.resize(3);
      _b(0) = 1.0/6.0;
      _b(1) = 2.0/3.0;
      _b(2) = 1.0/6.0;

      _a = Eigen::MatrixXd::Zero(3, 2);
      _a(1, 0) =  0.5;
      _a(2, 0) = -1.0;
      _a(2, 1) =  2.0;
      break;
    case SolverType::kFourthOrderClassic:
      _s = 4;
      _b.resize(4);
      _b(0) = 1.0/6.0;
      _b(1) = 1.0/3.0;
      _b(2) = 1.0/3.0;
      _b(3) = 1.0/6.0;

      _a = Eigen::MatrixXd::Zero(4, 3);
      _a(1, 0) = 0.5;
      _a(2, 1) = 0.5;
      _a(3, 2) = 1.0;
      break;
    case SolverType::kFourthOrderOptimal:
      _s = 4;
      _b.resize(4);
      _b(0) =  0.17476028;
      _b(1) = -0.55148066;
      _b(2) =  1.20553560;
      _b(3) =  0.17118478;

      _a = Eigen::MatrixXd::Zero(4, 3);
      _a(1, 0) =  0.4;
      _a(2, 0) =  0.29697761;
      _a(2, 1) =  0.15875964;
      _a(3, 0) =  0.21810040;
      _a(3, 1) = -3.05096516;
      _a(3, 2) =  3.83286476;
      break;
  }
}

Eigen::Vector<double, 24> RungeKutta::step(
    const X_t &x0, const U_t &u, const double dt) const
{
  std::vector<X_t> func_evals; // Store evaluation of x' function evaluations f_j
  X_t dx = X_t::Zero(_N);      // Delta from RK step (1 for each state variable)

  // Loop number of stages for specified RK type
  for(int i = 0; i < _s; ++i) {
    X_t x_stage = x0; // State to plug into ODE for this stage
    for(int j = 0; j < i; ++j) {
      if(_a(i, j) != 0.0) {
        for(int k = 0; k < _N; ++k)
          x_stage[k] += dt*_a(i, j)*func_evals[j][k];
      }
    }

    // Calculate state derivative from ode
    func_evals.push_back(_ode_func(x_stage, u));

    // Add stage function evaluation to dx
    for(int j = 0; j < _N; ++j) {
      dx[j] += dt*_b[i]*func_evals[i][j];
    }
  }

  return x0 + dx;
}


}