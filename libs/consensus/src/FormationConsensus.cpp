
#include <libs/consensus/FormationConsensus.hpp>

namespace amrl {

// const FormationConsensus::Vec_t FormationConsensus::r_iF(
//   { -1.5,  0.0,  0.0,  1.5, 1.5,  0.0,  0.0, -1.5});

const Eigen::Matrix<double, 0, 1> FormationConsensus::kU0(Eigen::Matrix<double, 0, 1>::Zero());


FormationConsensus::FormationConsensus(
    const uint8_t num_robots,
    const uint8_t num_states,
    const Vec_t &r_init,
    const double alpha,
    const double gamma,
    const std::vector<std::pair<int, int>> &conns,
    std::shared_ptr<FormationSupervisor> formation) 
: 
_t(0.0),
_n(num_robots),
_m(num_states),
_nm(num_robots * num_states), 
_x(Vec_t::Zero(_nm * 2)),
_xi(Vec_t::Zero(_nm)),
_zeta(Vec_t::Zero(_nm)),
_rdot_F(Vec_t::Zero(_nm)),
_rddot_F(Vec_t::Zero(_nm)),
_L(Mat_t::Zero(_n, _n)),
_sig(Mat_t::Zero(2*_n, 2*_n)),
_Sigma(Mat_t::Zero(_nm * 2, _nm * 2)),
_In(Mat_t::Identity(_n, _n)),
_Im(Mat_t::Identity(_m, _m)),
_gamma(gamma),
_alpha(alpha),
_formation(formation),
_solver(
  2*_nm,
  [&](const Vec_t &x0, const Eigen::Matrix<double, 0, 1> &u) { return this->x_dot(x0, u); },
  RungeKutta::SolverType::kFourthOrderOptimal)
{

  for(const auto &c : conns)     { _L(c.first, c.second) = -1.0; }
  for(size_t i = 0; i < _n; ++i) { _L(i,i) = -_L.row(i).sum(); }

  _sig(Eigen::seqN(0,_n), Eigen::seqN(_n,_n))  = _In;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(0,_n))  = -_L;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(_n,_n)) = -(_alpha*_In) -(_gamma*_L);

  // Kronecker Product (kinda)
  for (int i = 0; i < _m; ++i) {
    _Sigma(Eigen::seqN(2*_n*i, 2*_n), Eigen::seqN(2*_n*i, 2*_n)) = _sig;
  }

  // Initialize _x with Xi (Zeta(0) is all zeros)
  Eigen::VectorXd r_iF    = _formation->formation_pose_vector(0.0); 
  _x(Eigen::seqN(0, _nm)) = r_init - r_iF;
  update_sub_states();
}

void FormationConsensus::update_sub_states(void)
{
  _xi   = _x(Eigen::seqN(0, _n*_m));
  _zeta = _x(Eigen::seqN(_n*_m, _n*_m));
}

FormationConsensus::Vec_t FormationConsensus::control_update(
  const Vec_t &r, 
  const Vec_t &v,
  const Vec_t &a,
  const double dt)
{
  _x = _solver.step(_x, kU0, dt);
  update_sub_states();

  // // Reinitialize _x with updated robot information
  // _x(Eigen::seqN(0, 8)) = r - r_iF;     // Xi(0)
  // _x(Eigen::seqN(8, 8)) = v - _rdot_F;  // Zeta(0)

  // Vec_t x_dot = _Sigma*_x;
  // Eigen::Matrix<double, 8, 1> xi_dot   = x_dot(Eigen::seqN(0, 8));
  // Eigen::Matrix<double, 8, 1> zeta_dot = x_dot(Eigen::seqN(8, 8));
  // Eigen::Matrix<double, 8, 1> u        = zeta_dot + _rddot_F;
  Vec_t u = Vec_t::Zero(_n);
  return u;
}

FormationConsensus::Vec_t FormationConsensus::full_state(void) const
{
  return _x;
}

FormationConsensus::Vec_t FormationConsensus::center(void) const
{
  return _xi;
}

FormationConsensus::Vec_t FormationConsensus::x_dot(const Vec_t &x, const Eigen::Matrix<double, 0, 1> &u) const
{
  return _Sigma*x;
}

}
