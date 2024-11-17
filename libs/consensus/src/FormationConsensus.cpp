
#include <libs/consensus/FormationConsensus.hpp>

namespace amrl {

// const FormationConsensus::Vec_t FormationConsensus::r_iF(
//   { -1.5,  0.0,  0.0,  1.5, 1.5,  0.0,  0.0, -1.5});

const Eigen::Matrix<double, 0, 1> FormationConsensus::kU0(Eigen::Matrix<double, 0, 1>::Zero());



// FormationConsensus(void)
// {

// }

FormationConsensus::FormationConsensus(
    const uint8_t num_robots,
    const uint8_t num_states,
    const Vec_t &r_init, 
    const std::vector<std::pair<int, int>> &conns) 
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
_Im(Mat_t::Identity(_m, _m))
// _solver([&](const Vec_t &x0, const Eigen::Matrix<double, 0, 1> &u) { return this->x_dot(x0, u); },
//   RungeKutta<16, 0>::SolverType::kFourthOrderOptimal)
{
  // for (const auto &c : conns) {
  //   _L(c.second, c.first) = -2.0;
  // }

  for(size_t i = 0; i < _n; ++i) {
    _L(i,i) = -_L.row(i).sum();
  }

  _sig(Eigen::seqN(0,_n), Eigen::seqN(_n,_n))  = _In;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(0,_n))  = -_L;
  _sig(Eigen::seqN(_n,_n), Eigen::seqN(_n,_n)) = -(kAlpha*_In) -(kGamma*_L);

  // for (int i = 0; i < _; ++i) {
  //   for (int j = 0; j < 8; ++j) {
  //     _Sigma(Eigen::seqN(i*2,_m), Eigen::seqN(j*2,_m)) = _sig(i,j)*_Im;
  //   }
  // }

  // // Initialize Xi (Zeta(0) is all zeros)
  // for (int i = 0; i < 4; ++i) {
  //   _x(Eigen::seqN(i*2,_m)) = r_init(Eigen::seqN(i*2,_m)) - r_iF(Eigen::seqN(i*2,_m));
  // }
  // update_sub_states();
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
  // // Reinitialize _x with updated robot information
  // _x(Eigen::seqN(0, 8)) = r - r_iF;     // Xi(0)
  // _x(Eigen::seqN(8, 8)) = v - _rdot_F;  // Zeta(0)
  // update_sub_states();

  // Vec_t x_dot = _Sigma*_x;
  // Eigen::Matrix<double, 8, 1> xi_dot   = x_dot(Eigen::seqN(0, 8));
  // Eigen::Matrix<double, 8, 1> zeta_dot = x_dot(Eigen::seqN(8, 8));
  // Eigen::Matrix<double, 8, 1> u        = zeta_dot + _rddot_F;
  Vec_t u = Vec_t::Zero(_n);
  return u;
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
