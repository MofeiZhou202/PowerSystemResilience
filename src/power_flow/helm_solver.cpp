/// Holomorphic Embedding Load-flow Method (HELM) — AC power flow solver.
///
/// Ported from the HELMpy open-source reference implementation:
///   HELMpy, open source package of power flow solvers developed on Python 3
///   Copyright (C) 2019 Tulio Molina (tuliojose8@gmail.com) and
///                      Juan José Ortega (juanjoseop10@gmail.com)
///   Licensed under the GNU Affero General Public License v3.
///   https://github.com/TulioMolina/HELMpy
///
/// Enhancements over the original port:
///   - Sparse Ytrans via Eigen::SparseMatrix<Cplx> (O(nnz) instead of O(N²)).
///   - Multiple SLACK buses: all receive identity rows; multi-area supported.
///   - Distributed slack via K-factor participation factors.
///   - Warm start: DCPF-angle baseline or Gauss–Seidel pre-iterations.

#include "hacdcpf/power_flow/helm_solver.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/power_flow/residual_evaluator.hpp"

namespace hacdcpf::powerflow {

namespace {

using Cplx       = std::complex<double>;
using VecCplx    = std::vector<Cplx>;
using MatCplx    = std::vector<VecCplx>;           // [bus][coef_index]
using SparseCplx = Eigen::SparseMatrix<Cplx, Eigen::RowMajor>;

constexpr double kPi = 3.14159265358979323846;

// ─────────────────────────────────────────────────────────────────────────────
// Padé approximant — matrix method (HELMpy analytic_continuation.py: Pade)
//
// Evaluates the [L/L] Padé approximant of the power series {serie[0..n-1]}
// at s = 1.  The series must have an odd number of terms (2L+1).
// Returns a complex scalar approximation of the value at s = 1.
// ─────────────────────────────────────────────────────────────────────────────
Cplx pade_at_one(const VecCplx& serie, int n_terms) {
  const int L = (n_terms - 1) / 2;
  if (L < 1) return serie[0];  // Degenerate: return the constant term.

  // Build the L×L Toeplitz-like system for the denominator coefficients.
  Eigen::MatrixXcd mat_c(L, L);
  for (int i = 1; i <= L; ++i) {
    for (int j = i; j < i + L; ++j) {
      mat_c(i - 1, j - i) = serie[static_cast<size_t>(j)];
    }
  }

  // RHS: -serie[L+1 .. 2L]
  Eigen::VectorXcd rhs_b(L);
  for (int k = 0; k < L; ++k) {
    rhs_b(k) = -serie[static_cast<size_t>(L + 1 + k)];
  }

  // Solve for denominator coefficients β_1 … β_L.
  Eigen::VectorXcd vec_b = mat_c.lu().solve(rhs_b);

  // Assemble b-polynomial (b[0] = 1, b[i] = vec_b[L-i] reversed).
  VecCplx b(static_cast<size_t>(L + 1));
  b[0] = Cplx{1.0, 0.0};
  for (int i = 1; i <= L; ++i) {
    b[static_cast<size_t>(i)] = vec_b(L - i);
  }

  // Assemble a-polynomial via convolution: a[i] = Σ_{k=0}^{i} serie[k]*b[i-k].
  VecCplx a(static_cast<size_t>(L + 1), Cplx{0.0, 0.0});
  a[0] = serie[0];
  for (int i = 1; i <= L; ++i) {
    Cplx aux{0.0, 0.0};
    for (int k = 0; k <= i; ++k) {
      aux += serie[static_cast<size_t>(k)] * b[static_cast<size_t>(i - k)];
    }
    a[static_cast<size_t>(i)] = aux;
  }

  // Evaluate at s = 1: P(1)/Q(1) = Σa / Σb.
  Cplx sum_a{0.0, 0.0}, sum_b{0.0, 0.0};
  for (int i = 0; i <= L; ++i) {
    sum_a += a[static_cast<size_t>(i)];
    sum_b += b[static_cast<size_t>(i)];
  }
  if (std::abs(sum_b) < 1e-300) return Cplx{0.0, 0.0};
  return sum_a / sum_b;
}

// ─────────────────────────────────────────────────────────────────────────────
// Build sparse Ytrans and Yshunt.
//
// Ytrans — series-admittance part of Ybus stored as a RowMajor sparse matrix.
// Yshunt[i] — shunt admittance at bus i (line charging + bus Gs+jBs).
// ─────────────────────────────────────────────────────────────────────────────
void build_ytrans_yshunt(const SolverData& data,
                         SparseCplx& Ytrans,
                         VecCplx&    Yshunt) {
  const int    N    = static_cast<int>(data.ac_buses.size());
  const double base = data.base_mva;

  Yshunt.assign(static_cast<size_t>(N), Cplx{});
  std::vector<Eigen::Triplet<Cplx>> trip;
  trip.reserve(data.ac_branches.size() * 4 + static_cast<size_t>(N));

  for (const auto& br : data.ac_branches) {
    if (!br.in_service) continue;
    const int i = br.from_bus - 1;
    const int j = br.to_bus  - 1;
    if (i < 0 || j < 0 || i >= N || j >= N) continue;

    const Cplx z(br.r_pu, br.x_pu);
    if (std::abs(z) < 1e-20) continue;

    const Cplx   ys      = 1.0 / z;
    const double tap_mag = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double shift   = br.shift_deg * (kPi / 180.0);
    const Cplx   tap     = std::polar(tap_mag, shift);
    const double tap2    = std::norm(tap);
    if (tap2 <= 0.0) continue;

    const Cplx yff =  ys / tap2;
    const Cplx yft = -ys / std::conj(tap);
    const Cplx ytf = -ys / tap;
    const Cplx ytt =  ys;

    trip.emplace_back(i, i, yff);
    trip.emplace_back(j, j, ytt);
    trip.emplace_back(i, j, yft);
    trip.emplace_back(j, i, ytf);

    const Cplx b_half(0.0, br.b_pu / 2.0);
    Yshunt[static_cast<size_t>(i)] += b_half / tap2;
    Yshunt[static_cast<size_t>(j)] += b_half;
  }

  Ytrans.resize(N, N);
  Ytrans.setFromTriplets(trip.begin(), trip.end());

  for (int i = 0; i < N; ++i)
    Yshunt[static_cast<size_t>(i)] +=
        Cplx(data.ac_buses[static_cast<size_t>(i)].gs_mw,
             data.ac_buses[static_cast<size_t>(i)].bs_mvar) / base;

  for (const auto& sh : data.shunts) {
    if (!sh.in_service) continue;
    const int i = sh.bus - 1;
    if (i < 0 || i >= N) continue;
    const double bs = sh.switchable && sh.n_steps > 0
                          ? sh.bs_per_step * sh.current_step
                          : sh.bs_mvar;
    Yshunt[static_cast<size_t>(i)] += Cplx(sh.gs_mw, bs) / base;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Build the 2N×2N real modified admittance matrix for the HELM embedding.
// Uses sparse Ytrans InnerIterator — O(nnz) per bus instead of O(N).
// ─────────────────────────────────────────────────────────────────────────────
Eigen::SparseMatrix<double> build_germ_matrix(
    const SparseCplx& Ytrans, const std::vector<BusType>& btype, int N) {
  std::vector<Eigen::Triplet<double>> trip;
  trip.reserve(static_cast<size_t>(Ytrans.nonZeros() * 4 + 2 * N));

  for (int i = 0; i < N; ++i) {
    if (btype[static_cast<size_t>(i)] == BusType::SLACK) {
      trip.emplace_back(2 * i, 2 * i, 1.0);
      trip.emplace_back(2 * i + 1, 2 * i + 1, 1.0);
      continue;
    }
    for (SparseCplx::InnerIterator it(Ytrans, i); it; ++it) {
      const int j = static_cast<int>(it.col());
      const Cplx y = it.value();
      trip.emplace_back(2 * i, 2 * j, y.real());
      trip.emplace_back(2 * i, 2 * j + 1, -y.imag());
      trip.emplace_back(2 * i + 1, 2 * j, y.imag());
      trip.emplace_back(2 * i + 1, 2 * j + 1, y.real());
    }
  }

  Eigen::SparseMatrix<double> Y(2 * N, 2 * N);
  Y.setFromTriplets(trip.begin(), trip.end());
  return Y;
}

Eigen::SparseMatrix<double> build_ymod(const SparseCplx&           Ytrans,
                                       const std::vector<BusType>& btype,
                                       const VecCplx&              germ,
                                       int N) {
  std::vector<Eigen::Triplet<double>> trip;
  trip.reserve(static_cast<size_t>(Ytrans.nonZeros() * 4 + 2 * N));

  for (int i = 0; i < N; ++i) {
    const BusType bt = btype[static_cast<size_t>(i)];

    if (bt == BusType::SLACK) {
      trip.emplace_back(2 * i,     2 * i,     1.0);
      trip.emplace_back(2 * i + 1, 2 * i + 1, 1.0);

    } else if (bt == BusType::PQ) {
      for (SparseCplx::InnerIterator it(Ytrans, i); it; ++it) {
        const int  j = static_cast<int>(it.col());
        const Cplx y = it.value();
        trip.emplace_back(2 * i,     2 * j,      y.real());
        trip.emplace_back(2 * i,     2 * j + 1, -y.imag());
        trip.emplace_back(2 * i + 1, 2 * j,      y.imag());
        trip.emplace_back(2 * i + 1, 2 * j + 1,  y.real());
      }

    } else {  // PV — Model 2 about the no-load germ.
      const Cplx v0_conj = std::conj(germ[static_cast<size_t>(i)]);
      for (SparseCplx::InnerIterator it(Ytrans, i); it; ++it) {
        const int  j = static_cast<int>(it.col());
        const Cplx y = v0_conj * it.value();
        trip.emplace_back(2 * i, 2 * j,      y.real());
        trip.emplace_back(2 * i, 2 * j + 1, -y.imag());
      }
      trip.emplace_back(2 * i + 1, 2 * i, germ[static_cast<size_t>(i)].real());
      trip.emplace_back(2 * i + 1, 2 * i + 1,
                        germ[static_cast<size_t>(i)].imag());
    }
  }

  Eigen::SparseMatrix<double> Y(2 * N, 2 * N);
  Y.setFromTriplets(trip.begin(), trip.end());
  return Y;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute per-bus complex power injection S = P + jQ (p.u.) for all buses.
// Uses aggregated pg/qg from SolverData, and subtracts load demand.
// ─────────────────────────────────────────────────────────────────────────────
VecCplx compute_injections(const SolverData& data, int N) {
  VecCplx S(static_cast<size_t>(N), Cplx{});
  const double inv = 1.0 / data.base_mva;

  for (int i = 0; i < N; ++i) {
    // Generation (pre-aggregated in data.pg / data.qg).
    S[static_cast<size_t>(i)] += Cplx(data.pg[i], data.qg[i]);

    // Load demand.
    if (data.has_component_loads) {
      S[static_cast<size_t>(i)] -= Cplx(data.pd_pu[i], data.qd_pu[i]);
    } else {
      const auto& bus = data.ac_buses[static_cast<size_t>(i)];
      S[static_cast<size_t>(i)] -= Cplx(bus.pd_mw, bus.qd_mvar) * inv;
    }
  }
  return S;
}

// ─────────────────────────────────────────────────────────────────────────────
// Q injection at bus i from the final voltage profile (sparse row product).
// ─────────────────────────────────────────────────────────────────────────────
double compute_q_injection(int i, const VecCplx& Vf,
                           const SparseCplx& Ytrans,
                           const VecCplx& Yshunt, int /*N*/) {
  Cplx I_i{};
  for (SparseCplx::InnerIterator it(Ytrans, i); it; ++it)
    I_i += it.value() * Vf[static_cast<size_t>(it.col())];
  I_i += Yshunt[static_cast<size_t>(i)] * Vf[static_cast<size_t>(i)];
  return (Vf[static_cast<size_t>(i)] * std::conj(I_i)).imag();
}

// ─────────────────────────────────────────────────────────────────────────────
// Warm start: DCPF.
// Solves B'·θ = P (reduced B' from imaginary part of data.ybus) and returns
// V_dc[i] = 1∠θ_i for all non-slack buses.
// ─────────────────────────────────────────────────────────────────────────────
VecCplx dcpf_warmstart(const SolverData& data,
                       const std::vector<BusType>& btype, int N) {
  VecCplx V_dc(static_cast<size_t>(N), Cplx{1.0, 0.0});

  std::vector<int> free_idx;
  free_idx.reserve(static_cast<size_t>(N));
  std::vector<int> local(static_cast<size_t>(N), -1);
  for (int i = 0; i < N; ++i)
    if (btype[static_cast<size_t>(i)] != BusType::SLACK) {
      local[static_cast<size_t>(i)] = static_cast<int>(free_idx.size());
      free_idx.push_back(i);
    }
  const int M = static_cast<int>(free_idx.size());
  if (M == 0) return V_dc;

  // Build reduced B' = −Im(Ybus) for free-bus submatrix.
  // data.ybus is Eigen::SparseMatrix<complex<double>> in CCS (col-major) format.
  std::vector<Eigen::Triplet<double>> trip;
  trip.reserve(static_cast<size_t>(data.ybus.nonZeros()));
  for (int k = 0; k < data.ybus.outerSize(); ++k) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator
           it(data.ybus, k); it; ++it) {
      const int r  = static_cast<int>(it.row());
      const int c  = static_cast<int>(it.col());
      const int rl = (r < N) ? local[static_cast<size_t>(r)] : -1;
      const int cl = (c < N) ? local[static_cast<size_t>(c)] : -1;
      if (rl < 0 || cl < 0) continue;
      trip.emplace_back(rl, cl, -it.value().imag());
    }
  }
  Eigen::SparseMatrix<double> Bp(M, M);
  Bp.setFromTriplets(trip.begin(), trip.end());
  Bp.makeCompressed();

  // P injection for free buses.
  const double inv = 1.0 / data.base_mva;
  Eigen::VectorXd P_inj(M);
  P_inj.setZero();
  for (int k = 0; k < M; ++k) {
    const int   i   = free_idx[static_cast<size_t>(k)];
    const auto& bus = data.ac_buses[static_cast<size_t>(i)];
    double p = data.pg[i];
    if (data.has_component_loads) p -= data.pd_pu[i];
    else                          p -= bus.pd_mw * inv;
    P_inj(k) = p;
  }

  Eigen::SparseLU<Eigen::SparseMatrix<double>> dc_lu;
  dc_lu.analyzePattern(Bp);
  dc_lu.factorize(Bp);
  if (dc_lu.info() != Eigen::Success) return V_dc;

  const Eigen::VectorXd theta = dc_lu.solve(P_inj);
  for (int k = 0; k < M; ++k)
    V_dc[static_cast<size_t>(free_idx[static_cast<size_t>(k)])] =
        std::polar(1.0, theta(k));
  return V_dc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Warm start: Gauss–Seidel AC power flow pre-iterations.
// Returns improved voltage estimates; PV magnitudes are enforced at Vsp.
// ─────────────────────────────────────────────────────────────────────────────
VecCplx gauss_seidel_warmstart(const SolverData& data,
                               const std::vector<BusType>& btype,
                               const std::vector<double>& Vsp,
                               const VecCplx& S_inj,
                               int N, int n_iter) {
  VecCplx V(static_cast<size_t>(N), Cplx{1.0, 0.0});
  // Initialise slack buses to exact setpoints.
  for (int i = 0; i < N; ++i)
    if (btype[static_cast<size_t>(i)] == BusType::SLACK)
      V[static_cast<size_t>(i)] =
          std::polar(Vsp[static_cast<size_t>(i)],
                     data.ac_buses[static_cast<size_t>(i)].va_deg * (kPi / 180.0));

  // data.ybus is column-major; build a row-map for efficient per-bus iteration.
  // We use the column-major Ybus directly: for each non-slack bus i,
  // scan column i of Ybus to find Y[r][i] — but we need row i entries.
  // Because Ybus is symmetric for passive networks, col i = row i transposed.
  // Use a row-major copy for O(nnz) Gauss–Seidel.
  Eigen::SparseMatrix<std::complex<double>, Eigen::RowMajor> Ybus_rm =
      data.ybus.topLeftCorner(N, N);

  for (int iter = 0; iter < n_iter; ++iter) {
    for (int i = 0; i < N; ++i) {
      const BusType bt = btype[static_cast<size_t>(i)];
      if (bt == BusType::SLACK) continue;
      const Cplx Vi = V[static_cast<size_t>(i)];
      if (std::abs(Vi) < 1e-12) continue;

      Cplx Yii{};
      Cplx sum_off{};
      for (Eigen::SparseMatrix<std::complex<double>,
                               Eigen::RowMajor>::InnerIterator
             it(Ybus_rm, i); it; ++it) {
        const int j = static_cast<int>(it.col());
        if (j == i) { Yii = it.value(); continue; }
        sum_off += it.value() * V[static_cast<size_t>(j)];
      }
      if (std::abs(Yii) < 1e-20) continue;

      const Cplx I_sched = std::conj(S_inj[static_cast<size_t>(i)] / Vi);
      Cplx V_new = (I_sched - sum_off) / Yii;

      if (bt == BusType::PV && std::abs(V_new) > 1e-12)
        V_new = std::polar(Vsp[static_cast<size_t>(i)], std::arg(V_new));
      V[static_cast<size_t>(i)] = V_new;
    }
  }
  return V;
}

// ─────────────────────────────────────────────────────────────────────────────
// Distributed slack: adjust active-power injections of PV+SLACK buses via
// K-factor participation factors to balance P_load = P_gen before HELM runs.
// ─────────────────────────────────────────────────────────────────────────────
void apply_distributed_slack(VecCplx& S_inj,
                             const std::vector<BusType>& btype,
                             const std::vector<double>& K,
                             int N) {
  double P_load{}, P_gen{};
  for (int i = 0; i < N; ++i) {
    const double p = S_inj[static_cast<size_t>(i)].real();
    if (p < 0.0) P_load -= p;
    else         P_gen  += p;
  }
  const double imbalance = P_load - P_gen;

  double K_sum = 0.0;
  for (int i = 0; i < N; ++i)
    if (btype[static_cast<size_t>(i)] == BusType::PV)
      K_sum += K[static_cast<size_t>(i)];
  if (K_sum < 1e-12) return;

  for (int i = 0; i < N; ++i) {
    const double ki = K[static_cast<size_t>(i)];
    if (ki <= 0.0) continue;
    const BusType bt = btype[static_cast<size_t>(i)];
    if (bt != BusType::PV) continue;
    const double delta_P = imbalance * ki / K_sum;
    S_inj[static_cast<size_t>(i)] =
        Cplx(S_inj[static_cast<size_t>(i)].real() + delta_P,
             S_inj[static_cast<size_t>(i)].imag());
  }
}

double magnitude_squared_coefficient(const VecCplx& series, int order) {
  Cplx value{};
  for (int k = 0; k <= order; ++k) {
    value += series[static_cast<size_t>(k)] *
             std::conj(series[static_cast<size_t>(order - k)]);
  }
  return value.real();
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// HelmSolver::solve
// ─────────────────────────────────────────────────────────────────────────────
PowerFlowResult HelmSolver::solve(const SolverData& data,
                                  const PowerFlowOptions& opt) const {
  PowerFlowResult result;
  const int N = static_cast<int>(data.ac_buses.size());
  result.vm.assign(static_cast<size_t>(N), 1.0);
  result.va.assign(static_cast<size_t>(N), 0.0);

  auto fail = [&](std::string reason, std::string warning) {
    result.converged = false;
    result.residual = std::numeric_limits<double>::infinity();
    result.diagnostics.converged = false;
    result.diagnostics.iterations = result.iterations;
    result.diagnostics.final_mismatch_norm = result.residual;
    result.diagnostics.termination_reason = std::move(reason);
    if (!warning.empty()) {
      result.diagnostics.warnings.push_back(std::move(warning));
    }
  };

  const SolverData* verification_data = &data;
  SolverData distributed_slack_data;

  auto verify_physical_residual = [&]() {
    if (!result.converged) return;
    Eigen::Map<const Eigen::VectorXd> vm(result.vm.data(),
                                         static_cast<Eigen::Index>(result.vm.size()));
    Eigen::Map<const Eigen::VectorXd> va(result.va.data(),
                                         static_cast<Eigen::Index>(result.va.size()));
    const Eigen::VectorXd vdc;
    const auto residual =
        evaluate_power_flow_residual(*verification_data, vm, va, vdc);
    result.residual = residual.full.size() > 0
                          ? residual.full.lpNorm<Eigen::Infinity>()
                          : 0.0;
    const double tolerance = std::max(opt.tol, 1e-12);
    if (!std::isfinite(result.residual) || result.residual > tolerance) {
      result.converged = false;
      result.diagnostics.warnings.push_back(
          "HELM Pade coefficients stabilized, but the physical AC power-flow "
          "residual exceeds the requested tolerance.");
    }
    result.diagnostics.converged = result.converged;
    result.diagnostics.iterations = result.iterations;
    result.diagnostics.final_mismatch_norm = result.residual;
    result.diagnostics.termination_reason = result.converged
        ? "HELM converged with physical residual verification"
        : "HELM physical residual verification failed";
  };

  if (N == 0) {
    result.converged = true;
    verify_physical_residual();
    return result;
  }

  // ── Bus classification ──────────────────────────────────────────────────────
  std::vector<BusType> btype(static_cast<size_t>(N));
  for (int i = 0; i < N; ++i)
    btype[static_cast<size_t>(i)] = data.ac_buses[static_cast<size_t>(i)].bus_type;

  for (int i = 0; i < N; ++i) {
    if (btype[static_cast<size_t>(i)] == BusType::ISOLATED) {
      fail("HELM does not support in-model isolated AC buses",
           "Project/de-energize isolated buses before HELM; they must not be "
           "interpreted as PV equations.");
      return result;
    }
  }

  // Require at least one SLACK bus. Multiple references are opt-in.
  int slack_count = 0;
  for (int i = 0; i < N; ++i)
    if (btype[static_cast<size_t>(i)] == BusType::SLACK) ++slack_count;
  if (slack_count == 0) {
    fail("HELM requires an AC slack bus",
         "HELM was not run because the AC subsystem has no voltage-angle reference.");
    return result;
  }
  if (!helm_opts.allow_multi_slack && slack_count > 1) {
    fail("HELM multiple-slack input rejected by options",
         "The AC subsystem has multiple slack buses while allow_multi_slack=false.");
    return result;
  }

  if (helm_opts.max_coef < 4 || !std::isfinite(helm_opts.mismatch) ||
      helm_opts.mismatch <= 0.0) {
    fail("HELM options are invalid",
         "max_coef must be at least 4 and mismatch must be finite and positive.");
    return result;
  }

  // Voltage setpoints (pu).
  std::vector<double> Vsp(static_cast<size_t>(N));
  for (int i = 0; i < N; ++i)
    Vsp[static_cast<size_t>(i)] = data.ac_buses[static_cast<size_t>(i)].vm_pu;

  // ── Admittance decomposition (sparse) ──────────────────────────────────────
  SparseCplx Ytrans;
  VecCplx    Yshunt;
  build_ytrans_yshunt(data, Ytrans, Yshunt);

  // The no-load germ solves Ytrans*V0=0 at every non-reference bus. Unlike a
  // flat all-ones germ, this remains an exact order-zero solution with ideal
  // transformers and phase shifters.
  Eigen::SparseMatrix<double> germ_matrix =
      build_germ_matrix(Ytrans, btype, N);
  germ_matrix.makeCompressed();
  Eigen::SparseLU<Eigen::SparseMatrix<double>> germ_lu;
  germ_lu.analyzePattern(germ_matrix);
  germ_lu.factorize(germ_matrix);
  if (germ_lu.info() != Eigen::Success) {
    fail("HELM no-load germ factorization failed",
         "The series-admittance network is singular after applying AC slack references.");
    return result;
  }
  Eigen::VectorXd germ_rhs = Eigen::VectorXd::Zero(2 * N);
  for (int i = 0; i < N; ++i) {
    if (btype[static_cast<size_t>(i)] == BusType::SLACK) {
      germ_rhs[2 * i] = 1.0;
    }
  }
  const Eigen::VectorXd germ_solution = germ_lu.solve(germ_rhs);
  if (germ_lu.info() != Eigen::Success || !germ_solution.allFinite()) {
    fail("HELM no-load germ solve failed",
         "The no-load transformer/phase-shifter germ could not be computed.");
    return result;
  }
  VecCplx germ(static_cast<size_t>(N));
  for (int i = 0; i < N; ++i) {
    germ[static_cast<size_t>(i)] =
        Cplx(germ_solution[2 * i], germ_solution[2 * i + 1]);
    if (std::abs(germ[static_cast<size_t>(i)]) < 1e-12) {
      fail("HELM no-load germ contains a zero voltage",
           "The reciprocal-voltage series cannot be initialized at a zero-voltage germ.");
      return result;
    }
  }

  // ── Q limits ───────────────────────────────────────────────────────────────
  const double inv_base = 1.0 / data.base_mva;
  const double kInf     = std::numeric_limits<double>::max();
  std::vector<double> Qgmax(static_cast<size_t>(N), 0.0);
  std::vector<double> Qgmin(static_cast<size_t>(N), 0.0);
  std::vector<bool> has_q_capability(static_cast<size_t>(N), false);
  for (const auto& g : data.generators) {
    if (!g.in_service) continue;
    const int bi = g.bus - 1;
    if (bi < 0 || bi >= N) continue;
    Qgmax[static_cast<size_t>(bi)] += g.qmax_mvar * inv_base;
    Qgmin[static_cast<size_t>(bi)] += g.qmin_mvar * inv_base;
    has_q_capability[static_cast<size_t>(bi)] = true;
  }
  for (int i = 0; i < N; ++i) {
    if (!has_q_capability[static_cast<size_t>(i)]) {
      Qgmax[static_cast<size_t>(i)] = kInf;
      Qgmin[static_cast<size_t>(i)] = -kInf;
    }
  }

  // ── Power injections ────────────────────────────────────────────────────────
  VecCplx S_inj = compute_injections(data, N);

  // Distributed slack (K-factor): re-balance P before HELM runs.
  if (!helm_opts.participation_factors.empty()) {
    if (static_cast<int>(helm_opts.participation_factors.size()) != N) {
      fail("HELM distributed-slack factors have the wrong size",
           "participation_factors must contain one entry per AC bus.");
      return result;
    }
    double eligible_sum = 0.0;
    for (int i = 0; i < N; ++i) {
      const double k = helm_opts.participation_factors[static_cast<size_t>(i)];
      if (!std::isfinite(k) || k < 0.0) {
        fail("HELM distributed-slack factors are invalid",
             "Participation factors must be finite and non-negative.");
        return result;
      }
      if (btype[static_cast<size_t>(i)] == BusType::PV) {
        eligible_sum += k;
      } else if (k > 0.0) {
        fail("HELM distributed-slack factor targets a non-PV bus",
             "HELM can prescribe K-factor active-power changes only at PV buses; "
             "slack and PQ entries must be zero.");
        return result;
      }
    }
    if (eligible_sum <= 1e-12) {
      fail("HELM distributed-slack factors have no participating PV bus",
           "At least one PV bus must have a positive participation factor.");
      return result;
    }
    const VecCplx original_injections = S_inj;
    apply_distributed_slack(S_inj, btype, helm_opts.participation_factors, N);
    distributed_slack_data = data;
    for (int i = 0; i < N; ++i) {
      distributed_slack_data.pg[i] +=
          S_inj[static_cast<size_t>(i)].real() -
          original_injections[static_cast<size_t>(i)].real();
    }
    verification_data = &distributed_slack_data;
  }

  // Load Q for Q-limit bookkeeping.
  std::vector<double> Qd_pu(static_cast<size_t>(N), 0.0);
  for (int i = 0; i < N; ++i)
    Qd_pu[static_cast<size_t>(i)] = data.has_component_loads
        ? data.qd_pu[i]
        : data.ac_buses[static_cast<size_t>(i)].qd_mvar * inv_base;

  // ── Solver parameters ───────────────────────────────────────────────────────
  const int    max_coef = helm_opts.max_coef;
  const double mis_tol  = helm_opts.mismatch;
  const double pade_stop_tol =
      std::min(mis_tol, std::max(opt.tol, 1e-12));

  // ── Warm start: compute initial voltage estimate for Padé baseline ──────────
  VecCplx V_warm(static_cast<size_t>(N), Cplx{1.0, 0.0});
  if (helm_opts.warm_start == HelmWarmStart::DCPF) {
    V_warm = dcpf_warmstart(data, btype, N);
  } else if (helm_opts.warm_start == HelmWarmStart::GaussSeidel &&
             helm_opts.gauss_iter > 0) {
    V_warm = gauss_seidel_warmstart(data, btype, Vsp, S_inj, N,
                                    helm_opts.gauss_iter);
  }
  // Snap slack-bus warm-start entries to exact setpoints.
  for (int i = 0; i < N; ++i)
    if (btype[static_cast<size_t>(i)] == BusType::SLACK)
      V_warm[static_cast<size_t>(i)] =
          std::polar(Vsp[static_cast<size_t>(i)],
                     data.ac_buses[static_cast<size_t>(i)].va_deg * (kPi / 180.0));

  // ── Outer Q-limit loop (up to 10 PV→PQ switches) ───────────────────────────
  for (int q_outer = 0; q_outer < 10; ++q_outer) {

    // ── Build and factorize the 2N×2N modified admittance matrix ─────────────
    Eigen::SparseMatrix<double> Ymod = build_ymod(Ytrans, btype, germ, N);
    Ymod.makeCompressed();
    const bool use_sparse =
        helm_opts.sparse_threshold <= 0 || N >= helm_opts.sparse_threshold;
    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
    Eigen::FullPivLU<Eigen::MatrixXd> dense_lu;
    if (use_sparse) {
      lu.analyzePattern(Ymod);
      lu.factorize(Ymod);
      if (lu.info() != Eigen::Success) {
        fail("HELM coefficient matrix factorization failed",
             "Sparse LU could not factor the HELM Model-2 coefficient matrix.");
        return result;
      }
    } else {
      dense_lu.compute(Eigen::MatrixXd(Ymod));
      if (!dense_lu.isInvertible()) {
        fail("HELM coefficient matrix factorization failed",
             "Dense LU found the HELM Model-2 coefficient matrix singular.");
        return result;
      }
    }

    // ── Allocate series storage ───────────────────────────────────────────────
    MatCplx V  (static_cast<size_t>(N), VecCplx(static_cast<size_t>(max_coef), Cplx{}));
    MatCplx W  (static_cast<size_t>(N), VecCplx(static_cast<size_t>(max_coef), Cplx{}));
    MatCplx CC (static_cast<size_t>(N), VecCplx(static_cast<size_t>(max_coef), Cplx{}));

    // Exact no-load initialisation at s = 0.
    for (int i = 0; i < N; ++i) {
      V[static_cast<size_t>(i)][0] = germ[static_cast<size_t>(i)];
      W[static_cast<size_t>(i)][0] = Cplx{1.0, 0.0} / germ[static_cast<size_t>(i)];
    }

    // Padé baseline: use warm-start estimate (1+0j when warm_start == None).
    VecCplx Vpade_prev = V_warm;

    bool converged_helm = false;
    bool diverged       = false;
    int  n_computed     = 0;

    // ── Main coefficient loop ──────────────────────────────────────────────────
    for (int n = 1; n < max_coef; ++n) {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(2 * N);

      for (int i = 0; i < N; ++i) {
        const size_t si = static_cast<size_t>(i);
        const BusType bt = btype[si];

        if (bt == BusType::SLACK) {
          // Each SLACK bus gets its specific Vsp·e^{jθsp}.
          // At n=1 the identity row moves the germ to the requested reference.
          // At n≥2: zero RHS enforces c^[n]=0 for all higher coefficients.
          if (n == 1) {
            const Cplx Vref = std::polar(Vsp[si],
                data.ac_buses[si].va_deg * (kPi / 180.0));
            rhs[2 * i]     = Vref.real() - germ[si].real();
            rhs[2 * i + 1] = Vref.imag() - germ[si].imag();
          }

        } else if (bt == BusType::PQ) {
          // ── PQ bus ─────────────────────────────────────────────────────────
          // rhs = conj(S) · conj(W[i][n-1]) − Ysh[i] · V[i][n-1]
          const Cplx conj_S = std::conj(S_inj[si]);
          const Cplx conj_W = std::conj(W[si][static_cast<size_t>(n - 1)]);
          const Cplx val    = conj_S * conj_W
                              - Yshunt[si] * V[si][static_cast<size_t>(n - 1)];
          rhs[2 * i]     = val.real();
          rhs[2 * i + 1] = val.imag();

        } else {
          // ── PV bus (Model 2) ───────────────────────────────────────────────
          // Row 2i+1 — |V|² constraint: V_re[i][n] = VV/2.
          double VV;
          if (n == 1) {
            VV = Vsp[si] * Vsp[si] - std::norm(germ[si]);
          } else {
            VV = 0.0;
            for (int k = 1; k < n; ++k) {
              VV += (V[si][static_cast<size_t>(k)] *
                     std::conj(V[si][static_cast<size_t>(n - k)])).real();
            }
            VV = -VV;
          }
          rhs[2 * i + 1] = VV / 2.0;

          // Row 2i — P-balance equation: CC term.
          double p_cc;
          if (n == 1) {
            // CC = P_inject − Re(Ysh)  (first coefficient initialisation).
            p_cc = S_inj[si].real() -
                   Yshunt[si].real() * std::norm(germ[si]);
          } else {
            // Cache CC[i][n-1] = Σ_j Ytrans[i][j] · V[j][n-1] (sparse row).
            Cplx PP{};
            for (SparseCplx::InnerIterator it(Ytrans, i); it; ++it)
              PP += it.value() * V[static_cast<size_t>(it.col())]
                                  [static_cast<size_t>(n - 1)];
            CC[si][static_cast<size_t>(n - 1)] = PP;

            // Accumulate P loss: −Re( Σ_{x=1}^{n-1} conj(V[i][n−x]) · CC[i][x] )
            Cplx PPP{};
            for (int x = 1; x < n; ++x) {
              PPP += std::conj(V[si][static_cast<size_t>(n - x)]) *
                     CC[si][static_cast<size_t>(x)];
            }
            p_cc = -PPP.real();

            // Shunt conductance correction (Re(Ysh) ≠ 0).
            if (std::abs(Yshunt[si].real()) > 1e-20) {
              p_cc -= Yshunt[si].real() *
                      magnitude_squared_coefficient(V[si], n - 1);
            }
          }
          rhs[2 * i] = p_cc;
        }
      }  // per-bus RHS loop

      // ── Solve Y_mod · coef[:,n] = rhs ────────────────────────────────────
      Eigen::VectorXd coef_n;
      if (use_sparse) {
        coef_n = lu.solve(rhs);
      } else {
        coef_n = dense_lu.solve(rhs);
      }
      if ((use_sparse && lu.info() != Eigen::Success) || !coef_n.allFinite()) {
        diverged = true;
        break;
      }

      // ── Update V and W ─────────────────────────────────────────────────────
      for (int i = 0; i < N; ++i) {
        V[static_cast<size_t>(i)][static_cast<size_t>(n)] =
            Cplx(coef_n[2 * i], coef_n[2 * i + 1]);
      }
      // W[i][n] = − Σ_{k=0}^{n-1} W[i][k] · V[i][n−k]
      for (int i = 0; i < N; ++i) {
        Cplx aux{};
        for (int k = 0; k < n; ++k) {
          aux += W[static_cast<size_t>(i)][static_cast<size_t>(k)] *
                 V[static_cast<size_t>(i)][static_cast<size_t>(n - k)];
        }
        W[static_cast<size_t>(i)][static_cast<size_t>(n)] =
            -aux / germ[static_cast<size_t>(i)];
      }

      n_computed = n;

      // ── Padé convergence check (every 2 steps, from series length ≥ 4) ────
      const int slen = n + 1;
      if (slen >= 4 && (slen % 2) == 0) {
        double max_delta = 0.0;
        bool all_ok = true;
        for (int i = 0; i < N; ++i) {
          const Cplx v_new = pade_at_one(V[static_cast<size_t>(i)], slen);
          const Cplx v_old = Vpade_prev[static_cast<size_t>(i)];
          const double dm = std::abs(std::abs(v_new) - std::abs(v_old));
          const double da = std::abs(std::arg(v_new)  - std::arg(v_old));
          const double d  = std::max(dm, da);
          if (d > pade_stop_tol) all_ok = false;
          if (d > max_delta) max_delta = d;
          Vpade_prev[static_cast<size_t>(i)] = v_new;
        }
        if (helm_opts.pade_trace_out) helm_opts.pade_trace_out->push_back(max_delta);
        if (all_ok) { converged_helm = true; break; }
      }
    }  // coefficient loop

    if (diverged) {
      result.iterations = n_computed;
      fail("HELM coefficient solve failed",
           "LU solve failed or produced non-finite power-series coefficients.");
      return result;
    }

    // ── Extract final voltage profile via Padé ─────────────────────────────
    const int use_len = (n_computed % 2 == 1) ? (n_computed + 1) : n_computed;
    VecCplx Vfinal(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) {
      const int len = std::min(use_len, n_computed + 1);
      if (len >= 3) {
        Vfinal[static_cast<size_t>(i)] =
            pade_at_one(V[static_cast<size_t>(i)], len);
      } else {
        // Fallback: partial sum of series.
        Cplx acc{};
        for (int k = 0; k <= n_computed; ++k) {
          acc += V[static_cast<size_t>(i)][static_cast<size_t>(k)];
        }
        Vfinal[static_cast<size_t>(i)] = acc;
      }
      result.vm[static_cast<size_t>(i)] = std::abs(Vfinal[static_cast<size_t>(i)]);
      result.va[static_cast<size_t>(i)] = std::arg(Vfinal[static_cast<size_t>(i)]);
    }

    // ── Q-limit check (PV → PQ switching) ─────────────────────────────────
    if (!helm_opts.enforce_q_limits) {
      result.converged  = converged_helm;
      result.iterations = n_computed;
      verify_physical_residual();
      return result;
    }

    bool q_violated = false;
    for (int i = 0; i < N; ++i) {
      if (btype[static_cast<size_t>(i)] != BusType::PV) continue;
      const size_t si = static_cast<size_t>(i);

      const double Q_gen =
          compute_q_injection(i, Vfinal, Ytrans, Yshunt, N) + Qd_pu[si];

      if (Q_gen > Qgmax[si] + 1e-6) {
        btype[si]  = BusType::PQ;
        // Fix Q injection at the limit: S_inj_Q = Qgmax − Qd.
        S_inj[si]  = Cplx(S_inj[si].real(), Qgmax[si] - Qd_pu[si]);
        q_violated = true;
      } else if (Q_gen < Qgmin[si] - 1e-6) {
        btype[si]  = BusType::PQ;
        S_inj[si]  = Cplx(S_inj[si].real(), Qgmin[si] - Qd_pu[si]);
        q_violated = true;
      }
    }

    if (!q_violated) {
      result.converged  = converged_helm;
      result.iterations = n_computed;
      verify_physical_residual();
      return result;
    }
    // If Q limits were violated the outer loop will rebuild Y_mod and re-solve.
  }

  // Reached here only if Q-limit outer loop exhausted without a clean solution.
  fail("HELM reactive-power limit outer loop exhausted",
       "PV-to-PQ switching did not reach a stable active set within 10 outer iterations.");
  return result;
}

}  // namespace hacdcpf::powerflow
