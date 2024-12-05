
#include <libs/consensus/FormationConsensus.hpp>

namespace amrl {

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

  // Replicate sigma matrix into diagonal blocks. One for each state
  for (int i = 0; i < _m; ++i) {
    _Sigma(Eigen::seqN(2*_n*i, 2*_n), Eigen::seqN(2*_n*i, 2*_n)) = _sig;
  }

  // Initialize _x with Xi (Zeta(0) is all zeros)
  Eigen::VectorXd r_iF = _formation->formation_pose_vector(0.0);
  for (int i = 0; i < _m; ++i) {
    _x(Eigen::seqN(i*2*_n, _n)) = r_init(Eigen::seqN(i*_n, _n)) - r_iF(Eigen::seqN(i*_n, _n));
  }

  update_sub_states();
}

void FormationConsensus::update_sub_states(void)
{
  for(int i = 0; i < _m; ++i) {
    _xi(Eigen::seqN(i*_n, _n))   = _x(Eigen::seqN(i*2*_n, _n));
    _zeta(Eigen::seqN(i*_n, _n)) = _x(Eigen::seqN(i*2*_n + _n, _n));
  }
}

void FormationConsensus::update(
  const Vec_t &r, 
  const Vec_t &v,
  const double t,
  const double dt)
{
  Eigen::VectorXd r_iF     = _formation->formation_pose_vector(t);
  Eigen::VectorXd r_dot_iF = _formation->formation_velocity_vector(t);


  // Eigen::VectorXd x(Vec_t::Zero(_nm * 2)),
  // for (int i = 0; i < _m; ++i) {
  //   _x(Eigen::seqN(i*2*_n, _n))      = r(Eigen::seqN(i*_n, _n)) - r_iF(Eigen::seqN(i*_n, _n));
  //   _x(Eigen::seqN(i*2*_n + _n, _n)) = v(Eigen::seqN(i*_n, _n)) - r_dot_iF(Eigen::seqN(i*_n, _n));
  // }

  _x = _solver.step(_x, kU0, dt);
  update_sub_states();
}

FormationConsensus::Vec_t FormationConsensus::xi_zeta_dot(
  const Vec_t &r, 
  const Vec_t &v,
  const double t)
{
  Eigen::VectorXd r_iF     = _formation->formation_pose_vector(t);
  Eigen::VectorXd r_dot_iF = _formation->formation_velocity_vector(t);

  Eigen::VectorXd x(Vec_t::Zero(_nm * 2));
  for (int i = 0; i < _m; ++i) {
    x(Eigen::seqN(i*2*_n, _n))      = r(Eigen::seqN(i*_n, _n)) - r_iF(Eigen::seqN(i*_n, _n));
    x(Eigen::seqN(i*2*_n + _n, _n)) = v(Eigen::seqN(i*_n, _n)) - r_dot_iF(Eigen::seqN(i*_n, _n));
  }

  return _Sigma*x;
}

FormationConsensus::Vec_t FormationConsensus::full_state(void) const
{
  return _x;
}

FormationConsensus::Vec_t FormationConsensus::xi(void) const
{
  return _xi;
}

FormationConsensus::Vec_t FormationConsensus::zeta(void) const
{
  return _zeta;
}

FormationConsensus::Vec_t FormationConsensus::x_dot(const Vec_t &x, const Eigen::Matrix<double, 0, 1> &u) const
{
  return _Sigma*x;
}

}
