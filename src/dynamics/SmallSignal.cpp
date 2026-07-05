#include "hacdcpf/dynamics/SmallSignal.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>

#include <Eigen/Dense>

#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"

namespace hacdcpf::dynamics {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

struct DaeLayout {
  int n_x{0};
  int n_ac{0};
  int n_dc{0};
  int ac_off{0};
  int ai_off{0};
  int dc_off{0};
  int n{0};
};

DaeLayout make_layout(const DynamicSystem& sys) {
  DaeLayout L;
  L.n_x = sys.x.size();
  L.n_ac = sys.network.acPhaseNodeCount();
  L.n_dc = sys.network.dcBusCount();
  L.ac_off = L.n_x;
  L.ai_off = L.n_x + L.n_ac;
  L.dc_off = L.n_x + 2 * L.n_ac;
  L.n = L.n_x + 2 * L.n_ac + L.n_dc;
  return L;
}

void pack(const DynamicSystem& sys, const DaeLayout& L, Eigen::VectorXd& u) {
  u.resize(L.n);
  for (int i = 0; i < L.n_x; ++i) u[i] = sys.x.x[i];
  for (int i = 0; i < L.n_ac; ++i) {
    u[L.ac_off + i] = sys.y.Vac_abc[i].real();
    u[L.ai_off + i] = sys.y.Vac_abc[i].imag();
  }
  for (int i = 0; i < L.n_dc; ++i) u[L.dc_off + i] = sys.y.Vdc[i];
}

// Evaluate F(u) = [ f(x,V) ; g(x,V) ] with g = I_inj(x,V) - Y_eff*V (the DAE RHS,
// NOT the backward-Euler residual). Uses the same device stamp / derivative model
// as the mass-matrix DAE stepper.
bool eval_F(DynamicSystem& sys, const DaeLayout& L, double t, const Eigen::VectorXd& u,
            Eigen::VectorXd& F) {
  for (int i = 0; i < L.n_x; ++i) sys.x.x[i] = u[i];
  for (int i = 0; i < L.n_ac; ++i) {
    sys.y.Vac_abc[i] = std::complex<double>(u[L.ac_off + i], u[L.ai_off + i]);
  }
  for (int i = 0; i < L.n_dc; ++i) sys.y.Vdc[i] = u[L.dc_off + i];

  DynamicStamp stamp(L.n_ac, L.n_dc);
  for (const auto& device : sys.devices) device->stamp(t, sys.x, sys.y, stamp);
  Eigen::SparseMatrix<std::complex<double>> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  sys.network.assembleEffectiveMatrices(stamp, sys.options.singular_regularization_pu,
                                        Yac, Iac, Gdc, Idc);

  Eigen::VectorXd f = Eigen::VectorXd::Zero(L.n_x);
  for (const auto& device : sys.devices) device->computeDerivatives(t, sys.x, sys.y, f);

  F.resize(L.n);
  for (int i = 0; i < L.n_x; ++i) F[i] = f[i];
  if (L.n_ac > 0) {
    Eigen::VectorXcd Vac(L.n_ac);
    for (int i = 0; i < L.n_ac; ++i) {
      Vac[i] = std::complex<double>(u[L.ac_off + i], u[L.ai_off + i]);
    }
    const Eigen::VectorXcd g = Iac - Yac * Vac;
    for (int i = 0; i < L.n_ac; ++i) {
      F[L.ac_off + i] = g[i].real();
      F[L.ai_off + i] = g[i].imag();
    }
  }
  if (L.n_dc > 0) {
    Eigen::VectorXd Vdc(L.n_dc);
    for (int i = 0; i < L.n_dc; ++i) Vdc[i] = u[L.dc_off + i];
    const Eigen::VectorXd g = Idc - Gdc * Vdc;
    for (int i = 0; i < L.n_dc; ++i) F[L.dc_off + i] = g[i];
  }
  return F.allFinite();
}

// Recover the per-device differential-state ranges. assignStateIndices is
// idempotent (it re-derives the same offsets from the fixed device order), so
// re-running it does not perturb the state vector.
std::vector<SmallSignalStateInfo> build_state_info(DynamicSystem& sys, int n_x) {
  std::vector<SmallSignalStateInfo> info(static_cast<std::size_t>(std::max(0, n_x)));
  int offset = 0;
  for (auto& device : sys.devices) {
    const int before = offset;
    device->assignStateIndices(offset);
    const int size = offset - before;
    for (int k = 0; k < size; ++k) {
      const int gidx = before + k;
      if (gidx < 0 || gidx >= n_x) continue;
      SmallSignalStateInfo si;
      si.index = gidx;
      si.device_name = device->name();
      si.device_type = device->type();
      si.component_index = device->componentIndex();
      si.local_index = k;
      si.label = device->type() + "#" + std::to_string(device->componentIndex()) + ":s" +
                 std::to_string(k);
      info[static_cast<std::size_t>(gidx)] = si;
    }
  }
  for (int i = 0; i < n_x; ++i) {
    if (info[static_cast<std::size_t>(i)].label.empty()) {
      info[static_cast<std::size_t>(i)].index = i;
      info[static_cast<std::size_t>(i)].label = "x" + std::to_string(i);
    }
  }
  return info;
}

}  // namespace

SmallSignalResult small_signal_analysis(DynamicSystem& system) {
  SmallSignalResult result;
  const DaeLayout L = make_layout(system);
  result.n_differential = L.n_x;
  result.n_algebraic = 2 * L.n_ac + L.n_dc;

  if (L.n_x == 0) {
    result.success = true;
    result.message = "No differential states to analyze";
    return result;
  }

  const double t = system.options.t_start_s;
  const Eigen::VectorXd x_save = system.x.x;
  const NetworkState y_save = system.y;

  Eigen::VectorXd u0;
  pack(system, L, u0);
  Eigen::VectorXd F0;
  if (!eval_F(system, L, t, u0, F0)) {
    system.x.x = x_save;
    system.y = y_save;
    result.success = false;
    result.message = "Small-signal: base residual is non-finite at the operating point";
    return result;
  }

  // Full Jacobian J = dF/du by CENTRAL finite differences with a deliberately
  // large step. Machine swing equations carry a small equilibrium dead-band
  // (|power imbalance| <= 1e-4 -> derivative forced to 0), so a tiny sqrt(eps)
  // step never escapes it and would linearize to zero. A central step of ~1e-3
  // straddles the dead-band and recovers the smooth-model slope, while the
  // network block (linear in V) is captured exactly at any step.
  Eigen::MatrixXd J(L.n, L.n);
  const double fd_step = 1e-3;
  Eigen::VectorXd Fp;
  Eigen::VectorXd Fm;
  bool ok = true;
  for (int j = 0; j < L.n && ok; ++j) {
    const double h = fd_step * std::max(1.0, std::abs(u0[j]));
    Eigen::VectorXd up = u0;
    Eigen::VectorXd um = u0;
    up[j] += h;
    um[j] -= h;
    if (!eval_F(system, L, t, up, Fp) || !eval_F(system, L, t, um, Fm)) {
      ok = false;
      break;
    }
    J.col(j) = (Fp - Fm) / (2.0 * h);
  }
  system.x.x = x_save;
  system.y = y_save;
  if (!ok) {
    result.success = false;
    result.message = "Small-signal: Jacobian evaluation produced a non-finite value";
    return result;
  }

  const int nd = L.n_x;
  const int na = L.n - L.n_x;

  // Partition and eliminate the algebraic (network) block by Schur complement.
  // All device states are differential (mass M_dd = I), so A = f_x - f_y g_y^-1 g_x.
  const Eigen::MatrixXd f_x = J.topLeftCorner(nd, nd);
  Eigen::MatrixXd A = f_x;
  if (na > 0) {
    const Eigen::MatrixXd f_y = J.topRightCorner(nd, na);
    const Eigen::MatrixXd g_x = J.bottomLeftCorner(na, nd);
    const Eigen::MatrixXd g_y = J.bottomRightCorner(na, na);
    const Eigen::FullPivLU<Eigen::MatrixXd> lu(g_y);
    if (!lu.isInvertible()) {
      result.success = false;
      result.message = "Small-signal: algebraic (network) Jacobian block is singular";
      return result;
    }
    A.noalias() -= f_y * lu.solve(g_x);
  }
  result.reduced_jacobian = A;
  result.states = build_state_info(system, nd);

  // Eigen-decomposition of the reduced state matrix.
  Eigen::EigenSolver<Eigen::MatrixXd> es(A, /*computeEigenvectors=*/true);
  if (es.info() != Eigen::Success) {
    result.success = false;
    result.message = "Small-signal: eigen-decomposition failed";
    return result;
  }
  const Eigen::VectorXcd lambda = es.eigenvalues();
  const Eigen::MatrixXcd R = es.eigenvectors();          // right eigenvectors (columns)
  const Eigen::MatrixXcd Linv = R.inverse();             // left eigenvectors (rows)

  result.participation.resize(nd, nd);
  const double osc_threshold = 1e-6;
  result.stable = true;
  for (int i = 0; i < nd; ++i) {
    SmallSignalMode m;
    m.eigen_real = lambda[i].real();
    m.eigen_imag = lambda[i].imag();
    const double mag = std::abs(lambda[i]);
    m.frequency_hz = std::abs(m.eigen_imag) / kTwoPi;
    m.damping_ratio = mag > 1e-12 ? -m.eigen_real / mag : (m.eigen_real < 0 ? 1.0 : -1.0);
    m.oscillatory = std::abs(m.eigen_imag) > osc_threshold;
    if (m.eigen_real > result.stability_margin) result.stable = false;

    // Participation p_{ki} = |R(k,i)| * |Linv(i,k)|, normalized over k.
    double sum = 0.0;
    Eigen::VectorXd p(nd);
    for (int k = 0; k < nd; ++k) {
      p[k] = std::abs(R(k, i)) * std::abs(Linv(i, k));
      sum += p[k];
    }
    if (sum > 0.0) p /= sum;
    int dom = 0;
    for (int k = 1; k < nd; ++k) {
      if (p[k] > p[dom]) dom = k;
    }
    m.dominant_state_index = dom;
    m.dominant_state = result.states[static_cast<std::size_t>(dom)].label;
    for (int k = 0; k < nd; ++k) result.participation(i, k) = p[k];
    result.modes.push_back(std::move(m));
  }

  // Sort modes (and their participation rows) by damping ratio, most critical
  // (least damped) first.
  std::vector<int> order(result.modes.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return result.modes[a].damping_ratio < result.modes[b].damping_ratio;
  });
  std::vector<SmallSignalMode> sorted_modes;
  sorted_modes.reserve(order.size());
  Eigen::MatrixXd sorted_part(nd, nd);
  for (std::size_t r = 0; r < order.size(); ++r) {
    sorted_modes.push_back(result.modes[order[r]]);
    sorted_part.row(static_cast<int>(r)) = result.participation.row(order[r]);
  }
  result.modes = std::move(sorted_modes);
  result.participation = std::move(sorted_part);

  result.success = true;
  result.message = "Small-signal analysis completed";
  return result;
}

}  // namespace hacdcpf::dynamics
