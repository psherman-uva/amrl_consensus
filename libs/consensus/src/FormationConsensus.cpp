
#include <libs/consensus/FormationConsensus.hpp>

namespace amrl {

const FormationConsensus::R_t FormationConsensus::r_iF(
  { -1.5,  0.0,  0.0,  1.5, 1.5,  0.0,  0.0, -1.5});

const Eigen::Matrix<double, FormationConsensus::_n, FormationConsensus::_n> FormationConsensus::In(
  Eigen::Matrix<double, FormationConsensus::_n, FormationConsensus::_n>::Identity());
const Eigen::Matrix<double, FormationConsensus::_m, FormationConsensus::_m> FormationConsensus::Im(
  Eigen::Matrix<double, FormationConsensus::_m, FormationConsensus::_m>::Identity());

const Eigen::Matrix<double, 0, 1> FormationConsensus::kU0(Eigen::Matrix<double, 0, 1>::Zero());



// FormationConsensus(void)
// {

// }

FormationConsensus::FormationConsensus(
  const R_t &r_init, const std::vector<std::pair<int, int>> &conns) : 
_t(0.0),
_x(X_t::Zero()),
_xi(R_t::Zero()),
_zeta(V_t::Zero()),
_rdot_F(V_t::Zero()),
_rddot_F(A_t::Zero()),
_L(Eigen::Matrix<double, 4, 4>::Zero()),
_sig(Eigen::Matrix<double, 8, 8>::Zero()),
_Sigma(Eigen::Matrix<double, 16, 16>::Zero()),
_solver([&](const X_t &x0, const Eigen::Matrix<double, 0, 1> &u) { return this->x_dot(x0, u); },
  RungeKutta<16, 0>::SolverType::kFourthOrderOptimal)
{
  for (const auto &c : conns) {
    _L(c.second, c.first) = -2.0;
  }

  _L(0,0) = -_L.row(0).sum();
  _L(1,1) = -_L.row(1).sum();
  _L(2,2) = -_L.row(2).sum();
  _L(3,3) = -_L.row(3).sum();

  _sig(Eigen::seqN(0,_n), Eigen::seqN(_n,_n))  = In;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(0,_n))  = -_L;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(_n,_n)) = -(kAlpha*In) -(kGamma*_L);

  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      _Sigma(Eigen::seqN(i*2,_m), Eigen::seqN(j*2,_m)) = _sig(i,j)*Im;
    }
  }

  // Initialize Xi (Zeta(0) is all zeros)
  for (int i = 0; i < 4; ++i) {
    _x(Eigen::seqN(i*2,_m)) = r_init(Eigen::seqN(i*2,_m)) - r_iF(Eigen::seqN(i*2,_m));
  }
  update_sub_states();
}

void FormationConsensus::update_sub_states(void)
{
  _xi   = _x(Eigen::seqN(0, 8));
  _zeta = _x(Eigen::seqN(8, 8));
}

Eigen::Matrix<double, 8, 1> FormationConsensus::control_update(
  const R_t &r, 
  const V_t &v,
  const A_t &a,
  const double dt)
{
  // Reinitialize _x with updated robot information
  _x(Eigen::seqN(0, 8)) = r - r_iF;     // Xi(0)
  _x(Eigen::seqN(8, 8)) = v - _rdot_F;  // Zeta(0)
  update_sub_states();

  X_t x_dot = _Sigma*_x;
  Eigen::Matrix<double, 8, 1> xi_dot   = x_dot(Eigen::seqN(0, 8));
  Eigen::Matrix<double, 8, 1> zeta_dot = x_dot(Eigen::seqN(8, 8));
  Eigen::Matrix<double, 8, 1> u = zeta_dot + _rddot_F;

  return u;
}

FormationConsensus::R_t FormationConsensus::center(void) const
{
  return _xi;
}

FormationConsensus::X_t FormationConsensus::x_dot(const X_t &x, const Eigen::Matrix<double, 0, 1> &u) const
{
  return _Sigma*x;
}

}
