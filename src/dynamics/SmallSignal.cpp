#include "hacdcpf/dynamics/SmallSignal.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"

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

Eigen::SparseMatrix<double> dense_to_sparse(const Eigen::MatrixXd& dense) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(dense.size()));
  for (Eigen::Index j = 0; j < dense.cols(); ++j) {
    for (Eigen::Index i = 0; i < dense.rows(); ++i) {
      const double v = dense(i, j);
      if (v != 0.0) triplets.emplace_back(i, j, v);
    }
  }
  Eigen::SparseMatrix<double> sparse(dense.rows(), dense.cols());
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  return sparse;
}

// Assemble the raw DAE Jacobian J = dF/du = [f_x f_y; g_x g_y] ANALYTICALLY,
// reusing the same model the mass-matrix DAE stepper uses: each device's
// addJacobian is called with a dt=1, theta=-1 context so its differential block
// (-dt*theta*df) stamps +f directly and its current-injection block stamps
// d(I_inj)/d(.); the network contributes the -Y_eff block of dg/dV. This avoids
// the O(n) full-system finite-difference sweep (one assembly instead of ~2n RHS
// evaluations). Returns false if the result is non-finite (caller falls back to
// finite differences). Does not modify the system state.
bool assemble_analytic_jacobian(DynamicSystem& sys, const DaeLayout& L, double t,
                                Eigen::MatrixXd& J, std::vector<int>& diff_fd_columns) {
  if (L.n <= 0) return false;
  DynamicStamp stamp(L.n_ac, L.n_dc);
  for (const auto& device : sys.devices) device->stamp(t, sys.x, sys.y, stamp);
  Eigen::SparseMatrix<std::complex<double>> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  sys.network.assembleEffectiveMatrices(stamp, sys.options.singular_regularization_pu,
                                        Yac, Iac, Gdc, Idc);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(L.n) * 8);

  // Algebraic network block: g = I_inj - Y_eff*V, so dg/dV carries -Y_eff. The
  // real/imag split mirrors dae_add_analytic_network_block in the DAE stepper.
  for (int col = 0; col < Yac.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(Yac, col); it; ++it) {
      const int row = static_cast<int>(it.row());
      const auto y = it.value();
      if (y.real() != 0.0) {
        triplets.emplace_back(L.ac_off + row, L.ac_off + col, -y.real());
        triplets.emplace_back(L.ai_off + row, L.ai_off + col, -y.real());
      }
      if (y.imag() != 0.0) {
        triplets.emplace_back(L.ac_off + row, L.ai_off + col, y.imag());
        triplets.emplace_back(L.ai_off + row, L.ac_off + col, -y.imag());
      }
    }
  }
  for (int col = 0; col < Gdc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Gdc, col); it; ++it) {
      if (it.value() != 0.0) {
        triplets.emplace_back(L.dc_off + static_cast<int>(it.row()), L.dc_off + col,
                              -it.value());
      }
    }
  }

  // Per-device blocks: differential (+f, because -dt*theta = -1*-1 = +1) and the
  // analytic/local current-injection derivatives. Stamped into a separate vector
  // so we can record which columns any device couples to (its own states + its
  // terminal bus). setFromTriplets sums the device d(I_inj)/dV with the -Y_eff
  // network entries to form the full dg/dV.
  DynamicJacobianContext context;
  context.n_x = L.n_x;
  context.n_ac = L.n_ac;
  context.n_dc = L.n_dc;
  context.ac_real_offset = L.ac_off;
  context.ac_imag_offset = L.ai_off;
  context.dc_offset = L.dc_off;
  context.total_size = L.n;
  context.dt = 1.0;
  context.theta = -1.0;
  context.include_differential_derivatives = true;
  std::vector<Eigen::Triplet<double>> device_triplets;
  device_triplets.reserve(static_cast<std::size_t>(sys.devices.size()) * 16);
  for (const auto& device : sys.devices) {
    device->addJacobian(t, sys.x, sys.y, context, device_triplets);
  }

  // Differential-FD column set: the differential RHS f depends only on states
  // (all n_x — cross-device coupling flows through controller states) and on the
  // terminal-bus voltage columns that some device couples to. Non-device voltage
  // columns leave f unchanged, so they are skipped in the differential refinement.
  std::vector<char> col_touched(static_cast<std::size_t>(L.n), 0);
  for (const auto& tr : device_triplets) {
    if (tr.col() >= 0 && tr.col() < L.n) col_touched[static_cast<std::size_t>(tr.col())] = 1;
  }
  diff_fd_columns.clear();
  for (int j = 0; j < L.n_x; ++j) diff_fd_columns.push_back(j);
  for (int j = L.n_x; j < L.n; ++j) {
    if (col_touched[static_cast<std::size_t>(j)]) diff_fd_columns.push_back(j);
  }

  triplets.insert(triplets.end(), device_triplets.begin(), device_triplets.end());
  Eigen::SparseMatrix<double> Jsp(L.n, L.n);
  Jsp.setFromTriplets(triplets.begin(), triplets.end());
  J = Eigen::MatrixXd(Jsp);
  return J.allFinite();
}

// Read/write a single packed unknown (state, AC voltage real/imag, or DC voltage)
// directly in the system, so the differential finite difference can perturb one
// component at a time instead of re-unpacking the whole vector each column.
double get_packed_unknown(const DynamicSystem& sys, const DaeLayout& L, int j) {
  if (j < L.n_x) return sys.x.x[j];
  if (j < L.ai_off) return sys.y.Vac_abc[j - L.ac_off].real();
  if (j < L.dc_off) return sys.y.Vac_abc[j - L.ai_off].imag();
  return sys.y.Vdc[j - L.dc_off];
}

void set_packed_unknown(DynamicSystem& sys, const DaeLayout& L, int j, double v) {
  if (j < L.n_x) {
    sys.x.x[j] = v;
  } else if (j < L.ai_off) {
    auto& z = sys.y.Vac_abc[j - L.ac_off];
    z = std::complex<double>(v, z.imag());
  } else if (j < L.dc_off) {
    auto& z = sys.y.Vac_abc[j - L.ai_off];
    z = std::complex<double>(z.real(), v);
  } else {
    sys.y.Vdc[j - L.dc_off] = v;
  }
}

// Differential RHS f(x, V) via device computeDerivatives only (no network
// assembly), reading the current system state. See the differential-block note
// in small_signal_analysis for why a large FD step is used by the caller.
Eigen::VectorXd compute_differential_rhs(DynamicSystem& sys, const DaeLayout& L,
                                         double t) {
  Eigen::VectorXd f = Eigen::VectorXd::Zero(L.n_x);
  for (const auto& device : sys.devices) device->computeDerivatives(t, sys.x, sys.y, f);
  return f;
}

}  // namespace

SmallSignalResult small_signal_analysis(DynamicSystem& system,
                                        bool use_analytic_jacobian) {
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

  // Preferred path: assemble the raw Jacobian J = dF/du analytically from the
  // device addJacobian model + the network block (one assembly, no O(n) sweep).
  // Falls back to central finite differences if the analytic assembly is
  // unavailable/non-finite. eval_F above left the system state at u0.
  Eigen::MatrixXd J(L.n, L.n);
  bool built = false;
  if (use_analytic_jacobian) {
    std::vector<int> diff_fd_columns;
    built = assemble_analytic_jacobian(system, L, t, J, diff_fd_columns);
    if (built && L.n_x > 0) {
      // The analytic algebraic block (currents + network -Y_eff) is exact, but
      // the per-device addJacobian DIFFERENTIAL block reads as zero for machine
      // swing equations: their equilibrium dead-band (|imbalance| <= 1e-4) traps
      // the tiny (1e-6) local-FD step. Recompute the differential rows f = [f_x
      // f_y] with a large-step computeDerivatives-only FD (no network re-assembly),
      // which straddles the dead-band and captures the synchronizing/network
      // coupling as well as cross-device (machine <-> AVR/PSS/governor) coupling.
      // Only the columns f actually depends on are swept: every state column (the
      // controller coupling flows through states) plus the terminal-bus voltage
      // columns any device couples to (diff_fd_columns from the assembly). The
      // remaining (device-free) voltage columns leave f unchanged, so their
      // differential rows stay zero from the analytic assembly. The algebraic rows
      // are kept analytic. The system is at u0 here (eval_F above); perturb one
      // packed unknown at a time in place (no full re-unpack) and restore it.
      Eigen::VectorXd f0 = compute_differential_rhs(system, L, t);
      if (!f0.allFinite()) {
        built = false;
      } else {
        const double fd_step = 1e-3;
        for (int j : diff_fd_columns) {
          const double base = get_packed_unknown(system, L, j);
          const double h = fd_step * std::max(1.0, std::abs(base));
          set_packed_unknown(system, L, j, base + h);
          const Eigen::VectorXd fp = compute_differential_rhs(system, L, t);
          set_packed_unknown(system, L, j, base - h);
          const Eigen::VectorXd fm = compute_differential_rhs(system, L, t);
          set_packed_unknown(system, L, j, base);  // restore
          if (!fp.allFinite() || !fm.allFinite()) {
            built = false;
            break;
          }
          J.block(0, j, L.n_x, 1) = (fp - fm) / (2.0 * h);
        }
      }
    }
  }

  // Fallback: full Jacobian J = dF/du by CENTRAL finite differences with a
  // deliberately large step. Machine swing equations carry a small equilibrium
  // dead-band (|power imbalance| <= 1e-4 -> derivative forced to 0), so a tiny
  // sqrt(eps) step never escapes it and would linearize to zero. A central step
  // of ~1e-3 straddles the dead-band and recovers the smooth-model slope, while
  // the network block (linear in V) is captured exactly at any step.
  bool ok = true;
  if (!built) {
    const double fd_step = 1e-3;
    Eigen::VectorXd Fp;
    Eigen::VectorXd Fm;
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
    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
    lu.compute(dense_to_sparse(g_y));
    if (lu.info() != Eigen::Success) {
      result.success = false;
      result.message = "Small-signal: algebraic (network) Jacobian block is singular";
      return result;
    }
    const Eigen::MatrixXd gy_solve = lu.solve(g_x);
    if (lu.info() != Eigen::Success || !gy_solve.allFinite()) {
      result.success = false;
      result.message = "Small-signal: algebraic (network) sparse solve failed";
      return result;
    }
    A.noalias() -= f_y * gy_solve;
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
  // DY-01: R is singular for a defective (non-diagonalizable) reduced Jacobian
  // — e.g. repeated eigenvalues from identical parallel machines — so R.inverse()
  // yields non-finite left eigenvectors. Detect this and fall back to a finite
  // right-eigenvector-magnitude participation instead of emitting NaN factors.
  const bool participation_degraded = !Linv.allFinite();

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

    // Participation p_{ki} = |R(k,i)| * |Linv(i,k)|, normalized over k. DY-01:
    // when R is defective (Linv non-finite) fall back to |R(k,i)|^2 so the
    // factors stay finite instead of NaN.
    double sum = 0.0;
    Eigen::VectorXd p(nd);
    for (int k = 0; k < nd; ++k) {
      const double rk = std::abs(R(k, i));
      p[k] = participation_degraded ? rk * rk : rk * std::abs(Linv(i, k));
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
  result.message =
      participation_degraded
          ? "Small-signal analysis completed; reduced Jacobian is defective "
            "(repeated eigenvalues), participation uses a right-eigenvector "
            "magnitude fallback"
          : "Small-signal analysis completed";
  return result;
}

}  // namespace hacdcpf::dynamics
