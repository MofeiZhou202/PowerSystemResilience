#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/detail/core_compat.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/optimal_power_flow/native_ipm_solver.hpp"

#ifdef HACDCPF_HAVE_IPOPT
#include "hacdcpf/engine/engine.hpp"  // hacdcpf::engine::NLPModel, IpoptAdapter
#endif

namespace hacdcpf::opf {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMinInteriorWidth = 1e-6;
constexpr double kHugeBound = 1e4;

std::uint64_t rc_key(int row, int col) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32U) |
         static_cast<std::uint32_t>(col);
}

struct ACOPFIndex {
  int nb{0};
  int ndc{0};
  int np{0};
  int nq{0};
  int ng{0};
  int nc{0};
  int n_vdc{0};
  int n_pac{0};
  int n_qac{0};
  int n_pdc{0};
  int nvar{0};
  int neq{0};
  int slack_bus{0};

  int i_eq_p{0};
  int i_eq_q{0};
  int i_eq_dc{0};
  int i_eq_conv{0};

  int i_theta{0};
  int i_vm{0};
  int i_pg{0};
  int i_qg{0};
  int i_vdc{0};
  int i_pac{0};
  int i_qac{0};
  int i_pdc{0};

  std::vector<int> non_slack;
  std::vector<int> pq;
  std::vector<int> theta_col_by_bus;
  std::vector<int> vm_col_by_bus;
  std::vector<int> p_row_by_bus;
  std::vector<int> q_row_by_bus;
  std::vector<int> dc_row_by_bus;

  std::vector<int> gen_var_to_bus;
  std::vector<int> gen_var_to_gen_index;
  std::vector<int> gen_index_to_var;

  std::vector<int> conv_var_to_data_index;
  std::vector<int> conv_var_to_ac_bus;
  std::vector<int> conv_var_to_dc_bus;
  std::vector<int> conv_data_to_var;

  std::vector<double> vm_reference_by_bus;
  std::vector<double> va_reference_by_bus;
  std::vector<double> vdc_reference_by_bus;
};

struct JWorkspace {
  Eigen::SparseMatrix<double> matrix;
  std::vector<int> pf_to_j_nz;
  std::vector<int> pg_j_nz;
  std::vector<int> qg_j_nz;
  std::vector<int> pac_ac_j_nz;
  std::vector<int> qac_ac_j_nz;

  struct DCVdcEntry {
    int k{0};
    int m{0};
    int nz{-1};
    double gkm{0.0};
  };
  std::vector<DCVdcEntry> dc_vdc_offdiag_entries;
  std::vector<int> dc_vdc_diag_nz;
  std::vector<int> dc_pdc_j_nz;

  std::vector<int> conv_pac_j_nz;
  std::vector<int> conv_qac_j_nz;
  std::vector<int> conv_pdc_j_nz;
  std::vector<int> conv_vm_j_nz;
};

struct KKTWorkspace {
  Eigen::SparseMatrix<double> matrix;
  std::vector<int> h_diag_nz;
  std::vector<int> lambda_diag_nz;
  std::vector<int> j_to_upper_nz;
  std::vector<int> j_to_lower_nz;
  std::unique_ptr<core::SparseLinearSolver> linear_solver;
  bool analyzed{false};
};

int find_slack_bus(const std::vector<ACBus>& buses) {
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) {
    if (buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      return i;
    }
  }
  return buses.empty() ? -1 : 0;
}

std::pair<double, double> sanitize_bounds(double lo, double hi, double default_lo, double default_hi) {
  if (!std::isfinite(lo)) {
    lo = default_lo;
  }
  if (!std::isfinite(hi)) {
    hi = default_hi;
  }
  if (lo > hi) {
    std::swap(lo, hi);
  }
  if (hi - lo < kMinInteriorWidth) {
    const double mid = 0.5 * (lo + hi);
    lo = mid - 0.5 * kMinInteriorWidth;
    hi = mid + 0.5 * kMinInteriorWidth;
  }
  return {lo, hi};
}

double clamp_interior(double x, double lo, double hi) {
  const double width = hi - lo;
  const double eps = std::max(1e-8, 1e-3 * width);
  return std::clamp(x, lo + eps, hi - eps);
}

ACOPFIndex build_index(const core::SolverData& data) {
  ACOPFIndex idx;
  idx.nb = static_cast<int>(data.ac_buses.size());
  idx.ndc = static_cast<int>(data.dc_buses.size());
  idx.slack_bus = find_slack_bus(data.ac_buses);
  if (idx.nb == 0 || idx.slack_bus < 0) {
    return idx;
  }

  idx.theta_col_by_bus.assign(static_cast<size_t>(idx.nb), -1);
  idx.vm_col_by_bus.assign(static_cast<size_t>(idx.nb), -1);
  idx.p_row_by_bus.assign(static_cast<size_t>(idx.nb), -1);
  idx.q_row_by_bus.assign(static_cast<size_t>(idx.nb), -1);
  idx.dc_row_by_bus.assign(static_cast<size_t>(idx.ndc), -1);
  idx.va_reference_by_bus.resize(static_cast<size_t>(idx.nb), 0.0);
  idx.vm_reference_by_bus.resize(static_cast<size_t>(idx.nb), 1.0);
  idx.vdc_reference_by_bus.resize(static_cast<size_t>(idx.ndc), 1.0);

  for (int b = 0; b < idx.nb; ++b) {
    idx.va_reference_by_bus[static_cast<size_t>(b)] =
        data.ac_buses[static_cast<size_t>(b)].va_deg * kDegToRad;
    idx.vm_reference_by_bus[static_cast<size_t>(b)] = data.ac_buses[static_cast<size_t>(b)].vm_pu;
  }
  for (const auto& gen : data.generators) {
    if (!gen.in_service) {
      continue;
    }
    const int b = gen.bus - 1;
    if (b >= 0 && b < idx.nb) {
      idx.vm_reference_by_bus[static_cast<size_t>(b)] = gen.vg_pu;
    }
  }
  for (int k = 0; k < idx.ndc; ++k) {
    idx.vdc_reference_by_bus[static_cast<size_t>(k)] = data.dc_buses[static_cast<size_t>(k)].vm_pu;
  }

  for (int b = 0; b < idx.nb; ++b) {
    if (b == idx.slack_bus) {
      continue;
    }
    idx.non_slack.push_back(b);
    if (data.ac_buses[static_cast<size_t>(b)].bus_type == BusType::PQ) {
      idx.pq.push_back(b);
    }
  }

  idx.np = static_cast<int>(idx.non_slack.size());
  idx.nq = static_cast<int>(idx.pq.size());
  idx.i_theta = 0;
  idx.i_vm = idx.np;

  for (int k = 0; k < idx.np; ++k) {
    const int b = idx.non_slack[static_cast<size_t>(k)];
    idx.theta_col_by_bus[static_cast<size_t>(b)] = idx.i_theta + k;
    idx.p_row_by_bus[static_cast<size_t>(b)] = k;
  }
  for (int k = 0; k < idx.nq; ++k) {
    const int b = idx.pq[static_cast<size_t>(k)];
    idx.vm_col_by_bus[static_cast<size_t>(b)] = idx.i_vm + k;
    idx.q_row_by_bus[static_cast<size_t>(b)] = k + idx.np;
  }

  idx.gen_index_to_var.assign(data.generators.size(), -1);
  for (size_t gi = 0; gi < data.generators.size(); ++gi) {
    const auto& gen = data.generators[gi];
    if (!gen.in_service) {
      continue;
    }
    const int b = gen.bus - 1;
    if (b < 0 || b >= idx.nb) {
      continue;
    }
    idx.gen_index_to_var[gi] = static_cast<int>(idx.gen_var_to_bus.size());
    idx.gen_var_to_bus.push_back(b);
    idx.gen_var_to_gen_index.push_back(static_cast<int>(gi));
  }

  idx.ng = static_cast<int>(idx.gen_var_to_bus.size());
  idx.conv_data_to_var.assign(data.converters.size(), -1);
  for (size_t ci = 0; ci < data.converters.size(); ++ci) {
    const auto& conv = data.converters[ci];
    if (!conv.in_service) {
      continue;
    }
    const int ac_bus = conv.bus_ac - 1;
    const int dc_bus = conv.bus_dc - 1;
    if (ac_bus < 0 || ac_bus >= idx.nb || dc_bus < 0 || dc_bus >= idx.ndc) {
      continue;
    }
    idx.conv_data_to_var[ci] = static_cast<int>(idx.conv_var_to_data_index.size());
    idx.conv_var_to_data_index.push_back(static_cast<int>(ci));
    idx.conv_var_to_ac_bus.push_back(ac_bus);
    idx.conv_var_to_dc_bus.push_back(dc_bus);
  }

  idx.nc = static_cast<int>(idx.conv_var_to_data_index.size());
  idx.n_vdc = idx.ndc;
  idx.n_pac = idx.nc;
  idx.n_qac = idx.nc;
  idx.n_pdc = idx.nc;

  idx.i_pg = idx.i_vm + idx.nq;
  idx.i_qg = idx.i_pg + idx.ng;
  idx.i_vdc = idx.i_qg + idx.ng;
  idx.i_pac = idx.i_vdc + idx.n_vdc;
  idx.i_qac = idx.i_pac + idx.n_pac;
  idx.i_pdc = idx.i_qac + idx.n_qac;
  idx.nvar = idx.i_pdc + idx.n_pdc;

  idx.i_eq_p = 0;
  idx.i_eq_q = idx.i_eq_p + idx.np;
  idx.i_eq_dc = idx.i_eq_q + idx.nq;
  idx.i_eq_conv = idx.i_eq_dc + idx.n_vdc;
  idx.neq = idx.i_eq_conv + idx.nc;

  for (int k = 0; k < idx.ndc; ++k) {
    idx.dc_row_by_bus[static_cast<size_t>(k)] = idx.i_eq_dc + k;
  }
  return idx;
}

core::JacobianContext build_pf_context(const ACOPFIndex& idx) {
  core::JacobianContext ctx;
  ctx.n = idx.nb;
  ctx.ndc = idx.ndc;
  ctx.np = idx.np;
  ctx.nq = idx.nq;
  ctx.ndc_eq = 0;
  ctx.nvar = idx.np + idx.nq;
  ctx.non_slack = idx.non_slack;
  ctx.pq = idx.pq;
  ctx.dc_non_slack.clear();
  ctx.p_row = idx.p_row_by_bus;
  ctx.q_row = idx.q_row_by_bus;
  ctx.dc_row.assign(static_cast<size_t>(idx.ndc), -1);
  ctx.va_col = idx.theta_col_by_bus;
  ctx.vm_col = idx.vm_col_by_bus;
  ctx.vdc_col.assign(static_cast<size_t>(idx.ndc), -1);
  return ctx;
}

void initialize_bounds_and_state(const core::SolverData& data,
                                 const ACOPFIndex& idx,
                                 Eigen::VectorXd& x,
                                 Eigen::VectorXd& lower,
                                 Eigen::VectorXd& upper) {
  x = Eigen::VectorXd::Zero(idx.nvar);
  lower = Eigen::VectorXd::Constant(idx.nvar, -kHugeBound);
  upper = Eigen::VectorXd::Constant(idx.nvar, kHugeBound);

  for (int k = 0; k < idx.np; ++k) {
    const int bus = idx.non_slack[static_cast<size_t>(k)];
    const int col = idx.i_theta + k;
    auto [lo, hi] = sanitize_bounds(-kPi, kPi, -kPi, kPi);
    lower[col] = lo;
    upper[col] = hi;
    x[col] = data.ac_buses[static_cast<size_t>(bus)].va_deg * kDegToRad;
  }

  for (int k = 0; k < idx.nq; ++k) {
    const int bus = idx.pq[static_cast<size_t>(k)];
    const int col = idx.i_vm + k;
    const auto& b = data.ac_buses[static_cast<size_t>(bus)];
    auto [lo, hi] = sanitize_bounds(b.vmin_pu, b.vmax_pu, 0.5, 1.5);
    lower[col] = lo;
    upper[col] = hi;
    x[col] = b.vm_pu;
  }

  for (int k = 0; k < idx.ng; ++k) {
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
    const auto& gen = data.generators[static_cast<size_t>(gi)];

    const int p_col = idx.i_pg + k;
    const int q_col = idx.i_qg + k;

    auto [plo, phi] =
        sanitize_bounds(gen.pmin_mw / data.base_mva, gen.pmax_mw / data.base_mva, -kHugeBound, kHugeBound);
    auto [qlo, qhi] =
        sanitize_bounds(gen.qmin_mvar / data.base_mva, gen.qmax_mvar / data.base_mva, -kHugeBound, kHugeBound);

    lower[p_col] = plo;
    upper[p_col] = phi;
    lower[q_col] = qlo;
    upper[q_col] = qhi;

    x[p_col] = gen.pg_mw / data.base_mva;
    x[q_col] = gen.qg_mvar / data.base_mva;
  }

  for (int k = 0; k < idx.n_vdc; ++k) {
    const auto& dcb = data.dc_buses[static_cast<size_t>(k)];
    const int col = idx.i_vdc + k;
    double lo = 0.8;
    double hi = 1.2;
    if (dcb.bus_type == DCBusType::DC_V) {
      lo = dcb.vm_pu - 0.05;
      hi = dcb.vm_pu + 0.05;
    }
    auto [vlo, vhi] = sanitize_bounds(lo, hi, 0.8, 1.2);
    lower[col] = vlo;
    upper[col] = vhi;
    x[col] = dcb.vm_pu;
  }

  for (int k = 0; k < idx.nc; ++k) {
    const int ci = idx.conv_var_to_data_index[static_cast<size_t>(k)];
    const auto& conv = data.converters[static_cast<size_t>(ci)];

    const int p_col = idx.i_pac + k;
    const int q_col = idx.i_qac + k;
    const int d_col = idx.i_pdc + k;

    auto [plo, phi] =
        sanitize_bounds(conv.pmin_mw / data.base_mva, conv.pmax_mw / data.base_mva, -kHugeBound, kHugeBound);
    auto [qlo, qhi] =
        sanitize_bounds(conv.qmin_mvar / data.base_mva, conv.qmax_mvar / data.base_mva, -kHugeBound, kHugeBound);

    // Pdc can be bidirectional; infer symmetric limit from available converter bounds.
    const double smax_mw = std::max(1e-3,
                                    std::max(std::abs(conv.pmax_mw),
                                             std::max(std::abs(conv.pmin_mw), std::abs(conv.qmax_mvar))));
    auto [dlo, dhi] = sanitize_bounds(-smax_mw / data.base_mva, smax_mw / data.base_mva, -kHugeBound, kHugeBound);

    lower[p_col] = plo;
    upper[p_col] = phi;
    lower[q_col] = qlo;
    upper[q_col] = qhi;
    lower[d_col] = dlo;
    upper[d_col] = dhi;

    x[p_col] = std::clamp(0.0, plo, phi);
    x[q_col] = std::clamp(0.0, qlo, qhi);
    x[d_col] = std::clamp(0.0, dlo, dhi);
  }

  for (int i = 0; i < idx.nvar; ++i) {
    x[i] = clamp_interior(x[i], lower[i], upper[i]);
  }
}

void unpack_state(const ACOPFIndex& idx,
                  const Eigen::VectorXd& x,
                  Eigen::VectorXd& vm,
                  Eigen::VectorXd& va,
                  Eigen::VectorXd& vdc,
                  Eigen::VectorXd& pac,
                  Eigen::VectorXd& qac,
                  Eigen::VectorXd& pdc,
                  Eigen::VectorXd& pg_bus,
                  Eigen::VectorXd& qg_bus) {
  if (vm.size() != idx.nb) {
    vm = Eigen::VectorXd::Zero(idx.nb);
  }
  if (va.size() != idx.nb) {
    va = Eigen::VectorXd::Zero(idx.nb);
  }
  vm.setZero();
  va.setZero();

  for (int b = 0; b < idx.nb; ++b) {
    vm[b] = idx.vm_reference_by_bus[static_cast<size_t>(b)];
    va[b] = idx.va_reference_by_bus[static_cast<size_t>(b)];
  }
  for (int k = 0; k < idx.np; ++k) {
    const int bus = idx.non_slack[static_cast<size_t>(k)];
    va[bus] = x[idx.i_theta + k];
  }
  for (int k = 0; k < idx.nq; ++k) {
    const int bus = idx.pq[static_cast<size_t>(k)];
    vm[bus] = x[idx.i_vm + k];
  }

  if (vdc.size() != idx.ndc) {
    vdc = Eigen::VectorXd::Zero(idx.ndc);
  } else {
    vdc.setZero();
  }
  for (int k = 0; k < idx.n_vdc; ++k) {
    vdc[k] = x[idx.i_vdc + k];
  }

  if (pac.size() != idx.nc) {
    pac = Eigen::VectorXd::Zero(idx.nc);
    qac = Eigen::VectorXd::Zero(idx.nc);
    pdc = Eigen::VectorXd::Zero(idx.nc);
  } else {
    pac.setZero();
    qac.setZero();
    pdc.setZero();
  }
  for (int k = 0; k < idx.nc; ++k) {
    pac[k] = x[idx.i_pac + k];
    qac[k] = x[idx.i_qac + k];
    pdc[k] = x[idx.i_pdc + k];
  }

  if (pg_bus.size() != idx.nb) {
    pg_bus = Eigen::VectorXd::Zero(idx.nb);
  } else {
    pg_bus.setZero();
  }
  if (qg_bus.size() != idx.nb) {
    qg_bus = Eigen::VectorXd::Zero(idx.nb);
  } else {
    qg_bus.setZero();
  }

  for (int k = 0; k < idx.ng; ++k) {
    const int bus = idx.gen_var_to_bus[static_cast<size_t>(k)];
    pg_bus[bus] += x[idx.i_pg + k];
    qg_bus[bus] += x[idx.i_qg + k];
  }
}

double evaluate_objective(const core::SolverData& data, const ACOPFIndex& idx, const Eigen::VectorXd& x) {
  double f = 0.0;
  for (int k = 0; k < idx.ng; ++k) {
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
    const auto& gen = data.generators[static_cast<size_t>(gi)];
    const double pg_pu = x[idx.i_pg + k];
    const double pg_mw = pg_pu * data.base_mva;
    f += gen.cost_c2 * pg_mw * pg_mw + gen.cost_c1 * pg_mw + gen.cost_c0;
  }
  return f;
}

void accumulate_objective_grad_hdiag(const core::SolverData& data,
                                     const ACOPFIndex& idx,
                                     const Eigen::VectorXd& x,
                                     Eigen::VectorXd& grad,
                                     Eigen::VectorXd& hdiag) {
  grad.setZero(idx.nvar);
  hdiag.setZero(idx.nvar);

  for (int k = 0; k < idx.ng; ++k) {
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
    const auto& gen = data.generators[static_cast<size_t>(gi)];
    const int col = idx.i_pg + k;
    const double pg_pu = x[col];

    const double quad = 2.0 * gen.cost_c2 * data.base_mva * data.base_mva;
    const double lin = gen.cost_c1 * data.base_mva;

    grad[col] += quad * pg_pu + lin;
    hdiag[col] += quad;
  }
}

bool accumulate_log_barrier(const Eigen::VectorXd& x,
                            const Eigen::VectorXd& lower,
                            const Eigen::VectorXd& upper,
                            double mu,
                            Eigen::VectorXd& grad,
                            Eigen::VectorXd& hdiag,
                            double& barrier_value) {
  barrier_value = 0.0;
  for (int i = 0; i < x.size(); ++i) {
    const double sl = x[i] - lower[i];
    const double su = upper[i] - x[i];
    if (!(sl > 0.0) || !(su > 0.0)) {
      return false;
    }
    barrier_value -= mu * (std::log(sl) + std::log(su));
    grad[i] += -mu / sl + mu / su;
    hdiag[i] += mu / (sl * sl) + mu / (su * su);
  }
  return true;
}

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

double dual_least_squares_stationarity(const Eigen::SparseMatrix<double>& j,
                                       const Eigen::VectorXd& grad,
                                       Eigen::VectorXd& lambda_ls) {
  if (j.rows() == 0 || j.cols() == 0 || grad.size() != j.cols()) {
    return inf_norm(grad);
  }
  Eigen::SparseMatrix<double> jj_t = j * j.transpose();
  if (jj_t.rows() == 0) {
    return inf_norm(grad);
  }
  jj_t.makeCompressed();
  for (int r = 0; r < jj_t.rows(); ++r) {
    jj_t.coeffRef(r, r) += 1e-8;
  }
  jj_t.makeCompressed();

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(jj_t);
  if (solver.info() != Eigen::Success) {
    return inf_norm(grad);
  }
  const Eigen::VectorXd rhs = -(j * grad);
  lambda_ls = solver.solve(rhs);
  if (solver.info() != Eigen::Success || !lambda_ls.allFinite()) {
    return inf_norm(grad);
  }
  const Eigen::VectorXd grad_lag = grad + j.transpose() * lambda_ls;
  return inf_norm(grad_lag);
}

struct EqualityEvalWorkspace {
  Eigen::VectorXd pcalc;
  Eigen::VectorXd qcalc;
  Eigen::VectorXd p_spec;
  Eigen::VectorXd q_spec;
  Eigen::VectorXd pdc_linear;
  Eigen::VectorXd pdc_calc;
  Eigen::VectorXd pdc_spec;
  Eigen::VectorXd ac_mismatch;
};

struct BranchLimitEntry {
  int from_bus{0};
  int to_bus{0};
  double gs{0.0};
  double bs{0.0};
  double bc{0.0};
  double tap{1.0};
  double smax2{0.0};
};

struct NonlinearLimitWorkspace {
  std::vector<BranchLimitEntry> branch_limits;
  std::vector<double> conv_smax2;
};

struct NonlinearConstraintMetrics {
  double barrier_value{0.0};
  double max_h{-std::numeric_limits<double>::infinity()};
  double violation_norm_sq{0.0};
  bool strictly_interior{true};
};

NonlinearLimitWorkspace build_nonlinear_limit_workspace(const core::SolverData& data, const ACOPFIndex& idx) {
  NonlinearLimitWorkspace ws;
  ws.branch_limits.reserve(data.ac_branches.size());
  for (const auto& br : data.ac_branches) {
    if (!br.in_service || br.rate_a_mva <= 0.0) {
      continue;
    }
    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || i >= idx.nb || j < 0 || j >= idx.nb) {
      continue;
    }
    const double den = br.r_pu * br.r_pu + br.x_pu * br.x_pu;
    if (den <= 0.0) {
      continue;
    }
    BranchLimitEntry e;
    e.from_bus = i;
    e.to_bus = j;
    e.gs = br.r_pu / den;
    e.bs = -br.x_pu / den;
    e.bc = br.b_pu * 0.5;
    e.tap = (br.tap == 0.0) ? 1.0 : br.tap;
    const double smax_pu = br.rate_a_mva / data.base_mva;
    e.smax2 = smax_pu * smax_pu;
    ws.branch_limits.push_back(e);
  }

  ws.conv_smax2.assign(static_cast<size_t>(idx.nc), kHugeBound * kHugeBound);
  for (int k = 0; k < idx.nc; ++k) {
    const int ci = idx.conv_var_to_data_index[static_cast<size_t>(k)];
    const auto& conv = data.converters[static_cast<size_t>(ci)];
    const double smax_mva = std::max({std::abs(conv.pmax_mw),
                                      std::abs(conv.pmin_mw),
                                      std::abs(conv.qmax_mvar),
                                      std::abs(conv.qmin_mvar)});
    if (std::isfinite(smax_mva) && smax_mva > 0.0) {
      const double smax_pu = smax_mva / data.base_mva;
      ws.conv_smax2[static_cast<size_t>(k)] = smax_pu * smax_pu;
    }
  }
  return ws;
}

double assemble_hybrid_equalities(const core::SolverData& data,
                                  const ACOPFIndex& idx,
                                  const Eigen::VectorXd& vm,
                                  const Eigen::VectorXd& vdc,
                                  const Eigen::VectorXd& pac,
                                  const Eigen::VectorXd& qac,
                                  const Eigen::VectorXd& pdc,
                                  const Eigen::VectorXd& ac_mismatch,
                                  EqualityEvalWorkspace& ws,
                                  Eigen::VectorXd& g_eq) {
  if (g_eq.size() != idx.neq) {
    g_eq = Eigen::VectorXd::Zero(idx.neq);
  } else {
    g_eq.setZero();
  }

  const int n_ac_eq = idx.np + idx.nq;
  if (ac_mismatch.size() == n_ac_eq && n_ac_eq > 0) {
    g_eq.head(n_ac_eq) = ac_mismatch;
  }

  for (int k = 0; k < idx.nc; ++k) {
    const int ac_bus = idx.conv_var_to_ac_bus[static_cast<size_t>(k)];
    const int p_row = idx.p_row_by_bus[static_cast<size_t>(ac_bus)];
    const int q_row = idx.q_row_by_bus[static_cast<size_t>(ac_bus)];
    if (p_row >= 0) {
      g_eq[p_row] -= pac[k];
    }
    if (q_row >= 0) {
      g_eq[q_row] -= qac[k];
    }
  }

  if (idx.ndc > 0) {
    ws.pdc_linear = data.gdc * vdc;
    ws.pdc_calc = vdc.array() * ws.pdc_linear.array();
    for (int k = 0; k < idx.ndc; ++k) {
      g_eq[idx.i_eq_dc + k] =
          data.dc_buses[static_cast<size_t>(k)].pd_mw / data.base_mva + ws.pdc_calc[k];
    }
    for (int k = 0; k < idx.nc; ++k) {
      const int dc_bus = idx.conv_var_to_dc_bus[static_cast<size_t>(k)];
      const int row = idx.dc_row_by_bus[static_cast<size_t>(dc_bus)];
      if (row >= 0) {
        g_eq[row] -= pdc[k];
      }
    }
  }

  constexpr double eps_iac = 1e-6;
  for (int k = 0; k < idx.nc; ++k) {
    const int ci = idx.conv_var_to_data_index[static_cast<size_t>(k)];
    const auto& conv = data.converters[static_cast<size_t>(ci)];
    const int ac_bus = idx.conv_var_to_ac_bus[static_cast<size_t>(k)];
    const double vm_c = std::max(vm[ac_bus], 1e-6);
    const double p = pac[k];
    const double q = qac[k];
    const double sac = std::sqrt(std::max(eps_iac, p * p + q * q + eps_iac));
    const double iac = sac / vm_c;
    const double a = conv.loss_mw / data.base_mva;
    const double b = conv.loss_percent / 100.0;
    const double c = 1.0 - conv.eta;
    const double ploss = a + b * iac + c * iac * iac;
    g_eq[idx.i_eq_conv + k] = p + pdc[k] + ploss;
  }

  return inf_norm(g_eq);
}

double evaluate_hybrid_equalities_and_pf_jacobian(const core::SolverData& data,
                                                  const core::JacobianContext& pf_ctx,
                                                  core::JacobianPattern& pf_pattern,
                                                  const ACOPFIndex& idx,
                                                  const Eigen::VectorXd& vm,
                                                  const Eigen::VectorXd& va,
                                                  const Eigen::VectorXd& vdc,
                                                  const Eigen::VectorXd& pac,
                                                  const Eigen::VectorXd& qac,
                                                  const Eigen::VectorXd& pdc,
                                                  const Eigen::VectorXd& pg_bus,
                                                  const Eigen::VectorXd& qg_bus,
                                                  int ac_eval_threads,
                                                  EqualityEvalWorkspace& ws,
                                                  Eigen::VectorXd& g_eq) {
  static const std::vector<VSCConverter> kNoConverters;
  core::evaluate_residual_and_jacobian(data,
                                       pf_ctx,
                                       data.ac_buses,
                                       kNoConverters,
                                       pg_bus,
                                       qg_bus,
                                       vm,
                                       va,
                                       vdc,
                                       ws.pcalc,
                                       ws.qcalc,
                                       ws.p_spec,
                                       ws.q_spec,
                                       ws.pdc_linear,
                                       ws.pdc_calc,
                                       ws.pdc_spec,
                                       ws.ac_mismatch,
                                       pf_pattern,
                                       ac_eval_threads);
  return assemble_hybrid_equalities(data, idx, vm, vdc, pac, qac, pdc, ws.ac_mismatch, ws, g_eq);
}

double evaluate_hybrid_equalities_only(const core::SolverData& data,
                                       const core::JacobianContext& pf_ctx,
                                       const core::JacobianPattern& pf_pattern,
                                       const ACOPFIndex& idx,
                                       const Eigen::VectorXd& vm,
                                       const Eigen::VectorXd& va,
                                       const Eigen::VectorXd& vdc,
                                       const Eigen::VectorXd& pac,
                                       const Eigen::VectorXd& qac,
                                       const Eigen::VectorXd& pdc,
                                       const Eigen::VectorXd& pg_bus,
                                       const Eigen::VectorXd& qg_bus,
                                       int ac_eval_threads,
                                       EqualityEvalWorkspace& ws,
                                       Eigen::VectorXd& g_eq) {
  static const std::vector<VSCConverter> kNoConverters;
  core::evaluate_residual_only(data,
                               pf_ctx,
                               data.ac_buses,
                               kNoConverters,
                               pg_bus,
                               qg_bus,
                               vm,
                               va,
                               vdc,
                               ws.pcalc,
                               ws.qcalc,
                               ws.p_spec,
                               ws.q_spec,
                               ws.pdc_linear,
                               ws.pdc_calc,
                               ws.pdc_spec,
                               ws.ac_mismatch,
                               pf_pattern,
                               ac_eval_threads);
  return assemble_hybrid_equalities(data, idx, vm, vdc, pac, qac, pdc, ws.ac_mismatch, ws, g_eq);
}

NonlinearConstraintMetrics accumulate_nonlinear_limit_terms(const ACOPFIndex& idx,
                                                            const NonlinearLimitWorkspace& ws_limits,
                                                            const Eigen::VectorXd& vm,
                                                            const Eigen::VectorXd& va,
                                                            const Eigen::VectorXd& pac,
                                                            const Eigen::VectorXd& qac,
                                                            double exterior_penalty,
                                                            Eigen::VectorXd& grad,
                                                            Eigen::VectorXd& hdiag) {
  NonlinearConstraintMetrics metrics;
  const double rho = std::max(exterior_penalty, 0.0);

  auto apply_constraint = [&](double h, const std::array<int, 4>& cols, const std::array<double, 4>& dh, int nterm) {
    metrics.max_h = std::max(metrics.max_h, h);
    if (h <= 0.0) {
      return;
    }
    metrics.violation_norm_sq += h * h;
    metrics.strictly_interior = false;
    if (!(rho > 0.0)) {
      return;
    }
    metrics.barrier_value += 0.5 * rho * h * h;
    for (int t = 0; t < nterm; ++t) {
      const int col = cols[static_cast<size_t>(t)];
      if (col < 0) {
        continue;
      }
      const double dhi = dh[static_cast<size_t>(t)];
      grad[col] += rho * h * dhi;
      hdiag[col] += rho * dhi * dhi;
    }
  };

  for (const auto& br : ws_limits.branch_limits) {
    const int i = br.from_bus;
    const int j = br.to_bus;
    const double vi = vm[i];
    const double vj = vm[j];
    const double theta = va[i] - va[j];
    const double ctheta = std::cos(theta);
    const double stheta = std::sin(theta);
    const double tap = br.tap;
    const double tap2 = tap * tap;

    const double pf = (br.gs / tap2) * vi * vi - (br.gs * ctheta + br.bs * stheta) / tap * vi * vj;
    const double qf = -(br.bs / tap2 + br.bc) * vi * vi - (br.gs * stheta - br.bs * ctheta) / tap * vi * vj;

    const double dpf_dti = (br.gs * stheta - br.bs * ctheta) / tap * vi * vj;
    const double dpf_dtj = -dpf_dti;
    const double dpf_dvi = 2.0 * br.gs / tap2 * vi - (br.gs * ctheta + br.bs * stheta) / tap * vj;
    const double dpf_dvj = -(br.gs * ctheta + br.bs * stheta) / tap * vi;

    const double dqf_dti = -(br.gs * ctheta + br.bs * stheta) / tap * vi * vj;
    const double dqf_dtj = -dqf_dti;
    const double dqf_dvi = -2.0 * (br.bs / tap2 + br.bc) * vi - (br.gs * stheta - br.bs * ctheta) / tap * vj;
    const double dqf_dvj = -(br.gs * stheta - br.bs * ctheta) / tap * vi;

    const std::array<int, 4> cols_f = {
        idx.theta_col_by_bus[static_cast<size_t>(i)],
        idx.theta_col_by_bus[static_cast<size_t>(j)],
        idx.vm_col_by_bus[static_cast<size_t>(i)],
        idx.vm_col_by_bus[static_cast<size_t>(j)],
    };
    const std::array<double, 4> dh_f = {
        2.0 * pf * dpf_dti + 2.0 * qf * dqf_dti,
        2.0 * pf * dpf_dtj + 2.0 * qf * dqf_dtj,
        2.0 * pf * dpf_dvi + 2.0 * qf * dqf_dvi,
        2.0 * pf * dpf_dvj + 2.0 * qf * dqf_dvj,
    };
    apply_constraint(pf * pf + qf * qf - br.smax2, cols_f, dh_f, 4);

    const double pt = br.gs * vj * vj - (br.gs * ctheta - br.bs * stheta) / tap * vi * vj;
    const double qt = -(br.bs + br.bc) * vj * vj + (br.gs * stheta + br.bs * ctheta) / tap * vi * vj;

    const double dpt_dti = (br.gs * stheta + br.bs * ctheta) / tap * vi * vj;
    const double dpt_dtj = -dpt_dti;
    const double dpt_dvi = -(br.gs * ctheta - br.bs * stheta) / tap * vj;
    const double dpt_dvj = 2.0 * br.gs * vj - (br.gs * ctheta - br.bs * stheta) / tap * vi;

    const double dqt_dti = (br.gs * ctheta - br.bs * stheta) / tap * vi * vj;
    const double dqt_dtj = -dqt_dti;
    const double dqt_dvi = (br.gs * stheta + br.bs * ctheta) / tap * vj;
    const double dqt_dvj = -2.0 * (br.bs + br.bc) * vj + (br.gs * stheta + br.bs * ctheta) / tap * vi;

    const std::array<int, 4> cols_t = {
        idx.theta_col_by_bus[static_cast<size_t>(i)],
        idx.theta_col_by_bus[static_cast<size_t>(j)],
        idx.vm_col_by_bus[static_cast<size_t>(i)],
        idx.vm_col_by_bus[static_cast<size_t>(j)],
    };
    const std::array<double, 4> dh_t = {
        2.0 * pt * dpt_dti + 2.0 * qt * dqt_dti,
        2.0 * pt * dpt_dtj + 2.0 * qt * dqt_dtj,
        2.0 * pt * dpt_dvi + 2.0 * qt * dqt_dvi,
        2.0 * pt * dpt_dvj + 2.0 * qt * dqt_dvj,
    };
    apply_constraint(pt * pt + qt * qt - br.smax2, cols_t, dh_t, 4);
  }

  for (int k = 0; k < idx.nc; ++k) {
    const double h = pac[k] * pac[k] + qac[k] * qac[k] - ws_limits.conv_smax2[static_cast<size_t>(k)];
    const std::array<int, 4> cols = {idx.i_pac + k, idx.i_qac + k, -1, -1};
    const std::array<double, 4> dh = {2.0 * pac[k], 2.0 * qac[k], 0.0, 0.0};
    apply_constraint(h, cols, dh, 2);
  }

  if (!std::isfinite(metrics.max_h)) {
    metrics.max_h = -std::numeric_limits<double>::infinity();
  }
  return metrics;
}

JWorkspace build_jacobian_workspace(const core::SolverData& data,
                                    const core::JacobianPattern& pf_pattern,
                                    const ACOPFIndex& idx) {
  JWorkspace ws;
  ws.matrix.resize(idx.neq, idx.nvar);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(pf_pattern.matrix.nonZeros()) + static_cast<size_t>(2 * idx.ng) +
                   static_cast<size_t>(2 * idx.nc) + static_cast<size_t>(data.gdc.nonZeros()) +
                   static_cast<size_t>(idx.ndc + 5 * idx.nc));

  const int* pf_outer = pf_pattern.matrix.outerIndexPtr();
  const int* pf_inner = pf_pattern.matrix.innerIndexPtr();
  std::vector<std::pair<int, int>> pf_rc;
  pf_rc.reserve(static_cast<size_t>(pf_pattern.matrix.nonZeros()));

  for (int col = 0; col < pf_pattern.matrix.outerSize(); ++col) {
    for (int nz = pf_outer[col]; nz < pf_outer[col + 1]; ++nz) {
      const int row = pf_inner[nz];
      triplets.emplace_back(row, col, 0.0);
      pf_rc.emplace_back(row, col);
    }
  }

  std::vector<std::pair<int, int>> pg_rc(static_cast<size_t>(idx.ng), {-1, -1});
  std::vector<std::pair<int, int>> qg_rc(static_cast<size_t>(idx.ng), {-1, -1});
  std::vector<std::pair<int, int>> pac_ac_rc(static_cast<size_t>(idx.nc), {-1, -1});
  std::vector<std::pair<int, int>> qac_ac_rc(static_cast<size_t>(idx.nc), {-1, -1});
  for (int k = 0; k < idx.ng; ++k) {
    const int bus = idx.gen_var_to_bus[static_cast<size_t>(k)];
    const int p_row = idx.p_row_by_bus[static_cast<size_t>(bus)];
    const int q_row = idx.q_row_by_bus[static_cast<size_t>(bus)];
    if (p_row >= 0) {
      const int col = idx.i_pg + k;
      triplets.emplace_back(p_row, col, 1.0);
      pg_rc[static_cast<size_t>(k)] = {p_row, col};
    }
    if (q_row >= 0) {
      const int col = idx.i_qg + k;
      triplets.emplace_back(q_row, col, 1.0);
      qg_rc[static_cast<size_t>(k)] = {q_row, col};
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    const int ac_bus = idx.conv_var_to_ac_bus[static_cast<size_t>(k)];
    const int p_row = idx.p_row_by_bus[static_cast<size_t>(ac_bus)];
    const int q_row = idx.q_row_by_bus[static_cast<size_t>(ac_bus)];
    if (p_row >= 0) {
      const int col = idx.i_pac + k;
      triplets.emplace_back(p_row, col, -1.0);
      pac_ac_rc[static_cast<size_t>(k)] = {p_row, col};
    }
    if (q_row >= 0) {
      const int col = idx.i_qac + k;
      triplets.emplace_back(q_row, col, -1.0);
      qac_ac_rc[static_cast<size_t>(k)] = {q_row, col};
    }
  }

  std::vector<std::pair<int, int>> dc_diag_rc(static_cast<size_t>(idx.ndc), {-1, -1});
  std::vector<std::pair<int, int>> dc_pdc_rc(static_cast<size_t>(idx.nc), {-1, -1});
  std::vector<JWorkspace::DCVdcEntry> dc_offdiag_entries;
  dc_offdiag_entries.reserve(static_cast<size_t>(data.gdc.nonZeros()));

  for (int k = 0; k < idx.ndc; ++k) {
    const int row = idx.dc_row_by_bus[static_cast<size_t>(k)];
    if (row < 0) {
      continue;
    }
    const int col_diag = idx.i_vdc + k;
    triplets.emplace_back(row, col_diag, 0.0);
    dc_diag_rc[static_cast<size_t>(k)] = {row, col_diag};
  }

  for (int col = 0; col < data.gdc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(data.gdc, col); it; ++it) {
      const int k = static_cast<int>(it.row());
      const int m = static_cast<int>(it.col());
      if (k < 0 || k >= idx.ndc || m < 0 || m >= idx.ndc || k == m) {
        continue;
      }
      const int row = idx.dc_row_by_bus[static_cast<size_t>(k)];
      const int vcol = idx.i_vdc + m;
      if (row < 0) {
        continue;
      }
      triplets.emplace_back(row, vcol, 0.0);

      JWorkspace::DCVdcEntry entry;
      entry.k = k;
      entry.m = m;
      entry.gkm = it.value();
      dc_offdiag_entries.push_back(entry);
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    const int dc_bus = idx.conv_var_to_dc_bus[static_cast<size_t>(k)];
    const int row = idx.dc_row_by_bus[static_cast<size_t>(dc_bus)];
    if (row < 0) {
      continue;
    }
    const int col = idx.i_pdc + k;
    triplets.emplace_back(row, col, -1.0);
    dc_pdc_rc[static_cast<size_t>(k)] = {row, col};
  }

  std::vector<std::pair<int, int>> conv_pac_rc(static_cast<size_t>(idx.nc), {-1, -1});
  std::vector<std::pair<int, int>> conv_qac_rc(static_cast<size_t>(idx.nc), {-1, -1});
  std::vector<std::pair<int, int>> conv_pdc_rc(static_cast<size_t>(idx.nc), {-1, -1});
  std::vector<std::pair<int, int>> conv_vm_rc(static_cast<size_t>(idx.nc), {-1, -1});
  for (int k = 0; k < idx.nc; ++k) {
    const int row = idx.i_eq_conv + k;
    const int ac_bus = idx.conv_var_to_ac_bus[static_cast<size_t>(k)];
    const int vm_col = idx.vm_col_by_bus[static_cast<size_t>(ac_bus)];

    triplets.emplace_back(row, idx.i_pac + k, 0.0);
    triplets.emplace_back(row, idx.i_qac + k, 0.0);
    triplets.emplace_back(row, idx.i_pdc + k, 1.0);
    conv_pac_rc[static_cast<size_t>(k)] = {row, idx.i_pac + k};
    conv_qac_rc[static_cast<size_t>(k)] = {row, idx.i_qac + k};
    conv_pdc_rc[static_cast<size_t>(k)] = {row, idx.i_pdc + k};

    if (vm_col >= 0) {
      triplets.emplace_back(row, vm_col, 0.0);
      conv_vm_rc[static_cast<size_t>(k)] = {row, vm_col};
    }
  }

  ws.matrix.setFromTriplets(triplets.begin(), triplets.end());
  ws.matrix.makeCompressed();

  std::unordered_map<std::uint64_t, int> map;
  map.reserve(static_cast<size_t>(ws.matrix.nonZeros()) * 2U + 1U);
  const int* outer = ws.matrix.outerIndexPtr();
  const int* inner = ws.matrix.innerIndexPtr();
  for (int col = 0; col < ws.matrix.outerSize(); ++col) {
    for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
      map.emplace(rc_key(inner[nz], col), nz);
    }
  }

  ws.pf_to_j_nz.assign(pf_rc.size(), -1);
  for (size_t i = 0; i < pf_rc.size(); ++i) {
    const auto it = map.find(rc_key(pf_rc[i].first, pf_rc[i].second));
    if (it != map.end()) {
      ws.pf_to_j_nz[i] = it->second;
    }
  }

  ws.pg_j_nz.assign(static_cast<size_t>(idx.ng), -1);
  ws.qg_j_nz.assign(static_cast<size_t>(idx.ng), -1);
  ws.pac_ac_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.qac_ac_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.dc_vdc_diag_nz.assign(static_cast<size_t>(idx.ndc), -1);
  ws.dc_pdc_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.conv_pac_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.conv_qac_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.conv_pdc_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.conv_vm_j_nz.assign(static_cast<size_t>(idx.nc), -1);
  ws.dc_vdc_offdiag_entries = dc_offdiag_entries;
  for (auto& e : ws.dc_vdc_offdiag_entries) {
    e.nz = -1;
  }

  for (int k = 0; k < idx.ng; ++k) {
    if (pg_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(pg_rc[static_cast<size_t>(k)].first, pg_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.pg_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (qg_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(qg_rc[static_cast<size_t>(k)].first, qg_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.qg_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    if (pac_ac_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(pac_ac_rc[static_cast<size_t>(k)].first, pac_ac_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.pac_ac_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (qac_ac_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(qac_ac_rc[static_cast<size_t>(k)].first, qac_ac_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.qac_ac_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
  }

  for (int k = 0; k < idx.ndc; ++k) {
    if (dc_diag_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(dc_diag_rc[static_cast<size_t>(k)].first, dc_diag_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.dc_vdc_diag_nz[static_cast<size_t>(k)] = it->second;
      }
    }
  }
  for (size_t i = 0; i < ws.dc_vdc_offdiag_entries.size(); ++i) {
    auto& e = ws.dc_vdc_offdiag_entries[i];
    const auto it = map.find(rc_key(idx.dc_row_by_bus[static_cast<size_t>(e.k)], idx.i_vdc + e.m));
    if (it != map.end()) {
      e.nz = it->second;
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    if (dc_pdc_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(dc_pdc_rc[static_cast<size_t>(k)].first, dc_pdc_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.dc_pdc_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (conv_pac_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(conv_pac_rc[static_cast<size_t>(k)].first, conv_pac_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.conv_pac_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (conv_qac_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(conv_qac_rc[static_cast<size_t>(k)].first, conv_qac_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.conv_qac_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (conv_pdc_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(conv_pdc_rc[static_cast<size_t>(k)].first, conv_pdc_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.conv_pdc_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
    if (conv_vm_rc[static_cast<size_t>(k)].first >= 0) {
      const auto it =
          map.find(rc_key(conv_vm_rc[static_cast<size_t>(k)].first, conv_vm_rc[static_cast<size_t>(k)].second));
      if (it != map.end()) {
        ws.conv_vm_j_nz[static_cast<size_t>(k)] = it->second;
      }
    }
  }
  return ws;
}

void update_jacobian_values(const core::SolverData& data,
                            const core::JacobianPattern& pf_pattern,
                            const ACOPFIndex& idx,
                            const Eigen::VectorXd& vm,
                            const Eigen::VectorXd& vdc,
                            const Eigen::VectorXd& pac,
                            const Eigen::VectorXd& qac,
                            JWorkspace& ws) {
  double* jv = ws.matrix.valuePtr();
  std::fill(jv, jv + ws.matrix.nonZeros(), 0.0);

  const double* pfv = pf_pattern.matrix.valuePtr();
  for (size_t i = 0; i < ws.pf_to_j_nz.size(); ++i) {
    const int j_nz = ws.pf_to_j_nz[i];
    if (j_nz >= 0) {
      // core::JacobianPattern stores d(Pcalc,Qcalc)/dx, while OPF equalities use
      // g_ac = (Pspec,Qspec) - (Pcalc,Qcalc), so d(g_ac)/dx = -d(calc)/dx.
      jv[j_nz] = -pfv[i];
    }
  }
  for (int k = 0; k < idx.ng; ++k) {
    const int p_nz = ws.pg_j_nz[static_cast<size_t>(k)];
    const int q_nz = ws.qg_j_nz[static_cast<size_t>(k)];
    if (p_nz >= 0) {
      jv[p_nz] = 1.0;
    }
    if (q_nz >= 0) {
      jv[q_nz] = 1.0;
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    const int p_nz = ws.pac_ac_j_nz[static_cast<size_t>(k)];
    const int q_nz = ws.qac_ac_j_nz[static_cast<size_t>(k)];
    if (p_nz >= 0) {
      jv[p_nz] = -1.0;
    }
    if (q_nz >= 0) {
      jv[q_nz] = -1.0;
    }
  }

  Eigen::VectorXd pdc_linear = Eigen::VectorXd::Zero(idx.ndc);
  if (idx.ndc > 0) {
    pdc_linear = data.gdc * vdc;
  }
  for (const auto& e : ws.dc_vdc_offdiag_entries) {
    if (e.nz < 0) {
      continue;
    }
    jv[e.nz] = e.gkm * vdc[e.k];
  }
  for (int k = 0; k < idx.ndc; ++k) {
    const int nz = ws.dc_vdc_diag_nz[static_cast<size_t>(k)];
    if (nz >= 0) {
      jv[nz] = pdc_linear[k] + data.gdc.coeff(k, k) * vdc[k];
    }
  }
  for (int k = 0; k < idx.nc; ++k) {
    const int nz = ws.dc_pdc_j_nz[static_cast<size_t>(k)];
    if (nz >= 0) {
      jv[nz] = -1.0;
    }
  }

  constexpr double eps_iac = 1e-6;
  for (int k = 0; k < idx.nc; ++k) {
    const int ci = idx.conv_var_to_data_index[static_cast<size_t>(k)];
    const auto& conv = data.converters[static_cast<size_t>(ci)];
    const int ac_bus = idx.conv_var_to_ac_bus[static_cast<size_t>(k)];
    const double vm_c = std::max(vm[ac_bus], 1e-6);
    const double p = pac[k];
    const double q = qac[k];
    const double sac = std::sqrt(std::max(eps_iac, p * p + q * q + eps_iac));
    const double iac = sac / vm_c;
    const double b = conv.loss_percent / 100.0;
    const double c = 1.0 - conv.eta;
    const double dI_dP = p / (sac * vm_c);
    const double dI_dQ = q / (sac * vm_c);
    const double dI_dVm = -sac / (vm_c * vm_c);
    const double dPloss_dP = b * dI_dP + 2.0 * c * iac * dI_dP;
    const double dPloss_dQ = b * dI_dQ + 2.0 * c * iac * dI_dQ;
    const double dPloss_dVm = b * dI_dVm + 2.0 * c * iac * dI_dVm;

    const int pac_nz = ws.conv_pac_j_nz[static_cast<size_t>(k)];
    const int qac_nz = ws.conv_qac_j_nz[static_cast<size_t>(k)];
    const int pdc_nz = ws.conv_pdc_j_nz[static_cast<size_t>(k)];
    const int vm_nz = ws.conv_vm_j_nz[static_cast<size_t>(k)];
    if (pac_nz >= 0) {
      jv[pac_nz] = 1.0 + dPloss_dP;
    }
    if (qac_nz >= 0) {
      jv[qac_nz] = dPloss_dQ;
    }
    if (pdc_nz >= 0) {
      jv[pdc_nz] = 1.0;
    }
    if (vm_nz >= 0) {
      jv[vm_nz] = dPloss_dVm;
    }
  }
}

KKTWorkspace build_kkt_workspace(const JWorkspace& j_ws, int nvar, int neq) {
  KKTWorkspace ws;
  const int dim = nvar + neq;
  ws.matrix.resize(dim, dim);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(nvar + neq + 2 * j_ws.matrix.nonZeros()));

  for (int i = 0; i < nvar; ++i) {
    triplets.emplace_back(i, i, 0.0);
  }

  std::vector<int> j_rows(static_cast<size_t>(j_ws.matrix.nonZeros()), -1);
  std::vector<int> j_cols(static_cast<size_t>(j_ws.matrix.nonZeros()), -1);
  const int* j_outer = j_ws.matrix.outerIndexPtr();
  const int* j_inner = j_ws.matrix.innerIndexPtr();
  for (int col = 0; col < j_ws.matrix.outerSize(); ++col) {
    for (int nz = j_outer[col]; nz < j_outer[col + 1]; ++nz) {
      const int row = j_inner[nz];
      j_rows[static_cast<size_t>(nz)] = row;
      j_cols[static_cast<size_t>(nz)] = col;
      triplets.emplace_back(col, nvar + row, 0.0);
      triplets.emplace_back(nvar + row, col, 0.0);
    }
  }

  for (int r = 0; r < neq; ++r) {
    triplets.emplace_back(nvar + r, nvar + r, 0.0);
  }

  ws.matrix.setFromTriplets(triplets.begin(), triplets.end());
  ws.matrix.makeCompressed();

  std::unordered_map<std::uint64_t, int> map;
  map.reserve(static_cast<size_t>(ws.matrix.nonZeros()) * 2U + 1U);
  const int* outer = ws.matrix.outerIndexPtr();
  const int* inner = ws.matrix.innerIndexPtr();
  for (int col = 0; col < ws.matrix.outerSize(); ++col) {
    for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
      map.emplace(rc_key(inner[nz], col), nz);
    }
  }

  ws.h_diag_nz.assign(static_cast<size_t>(nvar), -1);
  for (int i = 0; i < nvar; ++i) {
    const auto it = map.find(rc_key(i, i));
    if (it != map.end()) {
      ws.h_diag_nz[static_cast<size_t>(i)] = it->second;
    }
  }

  ws.lambda_diag_nz.assign(static_cast<size_t>(neq), -1);
  for (int r = 0; r < neq; ++r) {
    const auto it = map.find(rc_key(nvar + r, nvar + r));
    if (it != map.end()) {
      ws.lambda_diag_nz[static_cast<size_t>(r)] = it->second;
    }
  }

  ws.j_to_upper_nz.assign(static_cast<size_t>(j_ws.matrix.nonZeros()), -1);
  ws.j_to_lower_nz.assign(static_cast<size_t>(j_ws.matrix.nonZeros()), -1);
  for (int nz = 0; nz < j_ws.matrix.nonZeros(); ++nz) {
    const int row = j_rows[static_cast<size_t>(nz)];
    const int col = j_cols[static_cast<size_t>(nz)];
    auto it_up = map.find(rc_key(col, nvar + row));
    if (it_up != map.end()) {
      ws.j_to_upper_nz[static_cast<size_t>(nz)] = it_up->second;
    }
    auto it_lo = map.find(rc_key(nvar + row, col));
    if (it_lo != map.end()) {
      ws.j_to_lower_nz[static_cast<size_t>(nz)] = it_lo->second;
    }
  }

  ws.linear_solver = core::make_default_sparse_solver();
  return ws;
}

void update_kkt_values(const JWorkspace& j_ws,
                       KKTWorkspace& kkt_ws,
                       const Eigen::VectorXd& hdiag,
                       double regularization) {
  double* kv = kkt_ws.matrix.valuePtr();
  std::fill(kv, kv + kkt_ws.matrix.nonZeros(), 0.0);

  for (int i = 0; i < hdiag.size(); ++i) {
    const int nz = kkt_ws.h_diag_nz[static_cast<size_t>(i)];
    if (nz >= 0) {
      kv[nz] = hdiag[i] + regularization;
    }
  }

  const double* jv = j_ws.matrix.valuePtr();
  for (int nz = 0; nz < j_ws.matrix.nonZeros(); ++nz) {
    const double v = jv[nz];
    const int up = kkt_ws.j_to_upper_nz[static_cast<size_t>(nz)];
    const int lo = kkt_ws.j_to_lower_nz[static_cast<size_t>(nz)];
    if (up >= 0) {
      kv[up] = v;
    }
    if (lo >= 0) {
      kv[lo] = v;
    }
  }

  for (size_t r = 0; r < kkt_ws.lambda_diag_nz.size(); ++r) {
    const int nz = kkt_ws.lambda_diag_nz[r];
    if (nz >= 0) {
      kv[nz] = -regularization;
    }
  }
}

double max_feasible_step(const Eigen::VectorXd& x,
                         const Eigen::VectorXd& dx,
                         const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper,
                         double interior_fraction) {
  double alpha = 1.0;
  const double frac = std::clamp(interior_fraction, 0.5, 0.9999);
  for (int i = 0; i < x.size(); ++i) {
    if (dx[i] > 0.0) {
      alpha = std::min(alpha, frac * (upper[i] - x[i]) / dx[i]);
    } else if (dx[i] < 0.0) {
      alpha = std::min(alpha, frac * (x[i] - lower[i]) / (-dx[i]));
    }
  }
  if (!std::isfinite(alpha)) {
    return 0.0;
  }
  return std::max(0.0, std::min(1.0, alpha));
}

bool is_strictly_inside(const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper) {
  for (int i = 0; i < x.size(); ++i) {
    if (!(x[i] > lower[i]) || !(x[i] < upper[i])) {
      return false;
    }
  }
  return true;
}

std::string variable_name(const ACOPFIndex& idx, int col) {
  if (col < 0 || col >= idx.nvar) {
    return "var?";
  }
  if (col >= idx.i_theta && col < idx.i_vm) {
    return "theta[" + std::to_string(col - idx.i_theta) + "]";
  }
  if (col >= idx.i_vm && col < idx.i_pg) {
    return "vm[" + std::to_string(col - idx.i_vm) + "]";
  }
  if (col >= idx.i_pg && col < idx.i_qg) {
    const int k = col - idx.i_pg;
    const int bus = idx.gen_var_to_bus[static_cast<size_t>(k)] + 1;
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)] + 1;
    return "pg[k=" + std::to_string(k) + ",bus=" + std::to_string(bus) + ",gen=" + std::to_string(gi) + "]";
  }
  if (col >= idx.i_qg && col < idx.i_vdc) {
    const int k = col - idx.i_qg;
    const int bus = idx.gen_var_to_bus[static_cast<size_t>(k)] + 1;
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)] + 1;
    return "qg[k=" + std::to_string(k) + ",bus=" + std::to_string(bus) + ",gen=" + std::to_string(gi) + "]";
  }
  if (col >= idx.i_vdc && col < idx.i_pac) {
    return "vdc[" + std::to_string(col - idx.i_vdc) + "]";
  }
  if (col >= idx.i_pac && col < idx.i_qac) {
    return "pac[" + std::to_string(col - idx.i_pac) + "]";
  }
  if (col >= idx.i_qac && col < idx.i_pdc) {
    return "qac[" + std::to_string(col - idx.i_qac) + "]";
  }
  return "pdc[" + std::to_string(col - idx.i_pdc) + "]";
}

bool solve_economic_dispatch(const core::SolverData& data,
                             const ACOPFIndex& idx,
                             Eigen::VectorXd& pg_dispatch_pu) {
  pg_dispatch_pu = Eigen::VectorXd::Zero(idx.ng);
  if (idx.ng == 0) {
    return false;
  }

  struct GenDispatchData {
    double pmin_mw{0.0};
    double pmax_mw{0.0};
    double c2{0.0};
    double c1{0.0};
  };
  std::vector<GenDispatchData> gens(static_cast<size_t>(idx.ng));

  double demand_mw = 0.0;
  if (data.has_component_loads) {
    demand_mw = data.pd_pu.sum() * data.base_mva;
  } else {
    for (const auto& b : data.ac_buses) {
      demand_mw += b.pd_mw;
    }
  }

  double sum_pmin = 0.0;
  double sum_pmax = 0.0;
  for (int k = 0; k < idx.ng; ++k) {
    const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
    const auto& g = data.generators[static_cast<size_t>(gi)];
    auto [pmin_mw, pmax_mw] = sanitize_bounds(g.pmin_mw, g.pmax_mw, 0.0, kHugeBound);
    gens[static_cast<size_t>(k)] = {
        .pmin_mw = pmin_mw,
        .pmax_mw = pmax_mw,
        .c2 = g.cost_c2,
        .c1 = g.cost_c1,
    };
    sum_pmin += pmin_mw;
    sum_pmax += pmax_mw;
  }

  const double target_mw = std::clamp(demand_mw, sum_pmin, sum_pmax);
  auto dispatch_sum = [&](double lambda, std::vector<double>* out) {
    double total = 0.0;
    for (int k = 0; k < idx.ng; ++k) {
      const auto& gd = gens[static_cast<size_t>(k)];
      double pg = gd.pmin_mw;
      if (gd.c2 > 1e-12) {
        pg = std::clamp((lambda - gd.c1) / (2.0 * gd.c2), gd.pmin_mw, gd.pmax_mw);
      } else {
        pg = (lambda >= gd.c1) ? gd.pmax_mw : gd.pmin_mw;
      }
      total += pg;
      if (out != nullptr) {
        (*out)[static_cast<size_t>(k)] = pg;
      }
    }
    return total;
  };

  double lambda_lo = std::numeric_limits<double>::infinity();
  double lambda_hi = -std::numeric_limits<double>::infinity();
  for (const auto& gd : gens) {
    lambda_lo = std::min(lambda_lo, gd.c1 - 2.0 * std::abs(gd.c2) * std::max(1.0, std::abs(gd.pmax_mw)));
    lambda_hi = std::max(lambda_hi, gd.c1 + 2.0 * std::abs(gd.c2) * std::max(1.0, std::abs(gd.pmax_mw)));
  }
  if (!std::isfinite(lambda_lo) || !std::isfinite(lambda_hi) || lambda_lo >= lambda_hi) {
    lambda_lo = -1e3;
    lambda_hi = 1e3;
  }

  std::vector<double> dispatch(static_cast<size_t>(idx.ng), 0.0);
  for (int iter = 0; iter < 80; ++iter) {
    const double mid = 0.5 * (lambda_lo + lambda_hi);
    const double sum_mid = dispatch_sum(mid, nullptr);
    if (sum_mid < target_mw) {
      lambda_lo = mid;
    } else {
      lambda_hi = mid;
    }
    if (std::abs(lambda_hi - lambda_lo) < 1e-8) {
      break;
    }
  }
  const double lambda = 0.5 * (lambda_lo + lambda_hi);
  dispatch_sum(lambda, &dispatch);

  // Minimal balancing correction in case of clipping/flat-cost degeneracy.
  double residual = target_mw;
  for (double p : dispatch) {
    residual -= p;
  }
  if (std::abs(residual) > 1e-8) {
    for (int k = 0; k < idx.ng && std::abs(residual) > 1e-8; ++k) {
      auto& p = dispatch[static_cast<size_t>(k)];
      const auto& gd = gens[static_cast<size_t>(k)];
      if (residual > 0.0) {
        const double room = gd.pmax_mw - p;
        const double add = std::min(room, residual);
        p += add;
        residual -= add;
      } else {
        const double room = p - gd.pmin_mw;
        const double sub = std::min(room, -residual);
        p -= sub;
        residual += sub;
      }
    }
  }

  for (int k = 0; k < idx.ng; ++k) {
    pg_dispatch_pu[k] = dispatch[static_cast<size_t>(k)] / data.base_mva;
  }
  return true;
}

[[maybe_unused]] void rebalance_pg_within_bus(const core::SolverData& data,
                                              const ACOPFIndex& idx,
                                              const Eigen::VectorXd& lower,
                                              const Eigen::VectorXd& upper,
                                              Eigen::VectorXd& x) {
  if (idx.ng <= 1) {
    return;
  }
  std::vector<std::vector<int>> gens_by_bus(static_cast<size_t>(idx.nb));
  for (int k = 0; k < idx.ng; ++k) {
    const int bus = idx.gen_var_to_bus[static_cast<size_t>(k)];
    if (bus >= 0 && bus < idx.nb) {
      gens_by_bus[static_cast<size_t>(bus)].push_back(k);
    }
  }

  for (const auto& list : gens_by_bus) {
    if (list.size() <= 1) {
      continue;
    }

    double target = 0.0;
    double sum_lo = 0.0;
    double sum_hi = 0.0;
    for (int k : list) {
      const int col = idx.i_pg + k;
      target += x[col];
      sum_lo += lower[col];
      sum_hi += upper[col];
    }
    target = std::clamp(target, sum_lo, sum_hi);

    auto sum_at_lambda = [&](double lambda, std::vector<double>* out) {
      double total = 0.0;
      for (size_t p = 0; p < list.size(); ++p) {
        const int k = list[p];
        const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
        const auto& gen = data.generators[static_cast<size_t>(gi)];
        const int col = idx.i_pg + k;
        const double lo = lower[col];
        const double hi = upper[col];
        const double quad = 2.0 * gen.cost_c2 * data.base_mva * data.base_mva;
        const double lin = gen.cost_c1 * data.base_mva;
        double pg = lo;
        if (quad > 1e-12) {
          pg = std::clamp((lambda - lin) / quad, lo, hi);
        } else {
          pg = (lambda >= lin) ? hi : lo;
        }
        total += pg;
        if (out != nullptr) {
          (*out)[p] = pg;
        }
      }
      return total;
    };

    double lambda_lo = std::numeric_limits<double>::infinity();
    double lambda_hi = -std::numeric_limits<double>::infinity();
    for (int k : list) {
      const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
      const auto& gen = data.generators[static_cast<size_t>(gi)];
      const int col = idx.i_pg + k;
      const double lo = lower[col];
      const double hi = upper[col];
      const double quad = 2.0 * gen.cost_c2 * data.base_mva * data.base_mva;
      const double lin = gen.cost_c1 * data.base_mva;
      lambda_lo = std::min(lambda_lo, lin + quad * lo - std::abs(quad) * std::max(1.0, std::abs(lo)));
      lambda_hi = std::max(lambda_hi, lin + quad * hi + std::abs(quad) * std::max(1.0, std::abs(hi)));
    }
    if (!std::isfinite(lambda_lo) || !std::isfinite(lambda_hi) || lambda_lo >= lambda_hi) {
      lambda_lo = -1e3;
      lambda_hi = 1e3;
    }

    for (int it = 0; it < 100; ++it) {
      const double lambda_mid = 0.5 * (lambda_lo + lambda_hi);
      const double total = sum_at_lambda(lambda_mid, nullptr);
      if (total < target) {
        lambda_lo = lambda_mid;
      } else {
        lambda_hi = lambda_mid;
      }
      if (std::abs(lambda_hi - lambda_lo) < 1e-10) {
        break;
      }
    }
    const double lambda = 0.5 * (lambda_lo + lambda_hi);
    std::vector<double> dispatch(list.size(), 0.0);
    sum_at_lambda(lambda, &dispatch);

    double residual = target;
    for (double p : dispatch) {
      residual -= p;
    }
    if (std::abs(residual) > 1e-12) {
      for (size_t p = 0; p < list.size() && std::abs(residual) > 1e-12; ++p) {
        const int k = list[p];
        const int col = idx.i_pg + k;
        if (residual > 0.0) {
          const double room = upper[col] - dispatch[p];
          const double add = std::min(room, residual);
          dispatch[p] += add;
          residual -= add;
        } else {
          const double room = dispatch[p] - lower[col];
          const double sub = std::min(room, -residual);
          dispatch[p] -= sub;
          residual += sub;
        }
      }
    }

    for (size_t p = 0; p < list.size(); ++p) {
      const int k = list[p];
      const int col = idx.i_pg + k;
      x[col] = std::clamp(dispatch[p], lower[col], upper[col]);
    }
  }
}

void compute_ac_injections(const core::SolverData& data,
                           const Eigen::VectorXd& vm,
                           const Eigen::VectorXd& va,
                           Eigen::VectorXd& pcalc,
                           Eigen::VectorXd& qcalc) {
  const int nb = static_cast<int>(data.ac_buses.size());
  pcalc = Eigen::VectorXd::Zero(nb);
  qcalc = Eigen::VectorXd::Zero(nb);
  if (nb == 0) {
    return;
  }

  const Eigen::MatrixXcd ybus = Eigen::MatrixXcd(data.ybus);
  for (int i = 0; i < nb; ++i) {
    for (int j = 0; j < nb; ++j) {
      const double theta = va[i] - va[j];
      const double g = ybus(i, j).real();
      const double b = ybus(i, j).imag();
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      pcalc[i] += vm[i] * vm[j] * (g * c + b * s);
      qcalc[i] += vm[i] * vm[j] * (g * s - b * c);
    }
  }
}

bool try_dispatch_pf_fallback(const HybridPowerSystem& ac_only_sys,
                              const core::SolverData& data,
                              const ACOPFIndex& idx,
                              ACOPFResult& out) {
  auto run_pf_extract = [&](const HybridPowerSystem& candidate) -> bool {
    core::SolverData pf_data = core::make_solver_data(candidate, LossModelType::Linear);
    core::NewtonSolver pf_solver;
    PowerFlowOptions pf_opt;
    pf_opt.max_iter = 80;
    pf_opt.tol = 1e-8;
    pf_opt.enable_solver_profiling = false;
    const PowerFlowResult pf_result = pf_solver.solve(pf_data, pf_opt, nullptr);
    if (!pf_result.converged) {
      return false;
    }

    const int nb = static_cast<int>(pf_data.ac_buses.size());
    Eigen::VectorXd vm = Eigen::Map<const Eigen::VectorXd>(pf_result.vm.data(), nb);
    Eigen::VectorXd va = Eigen::Map<const Eigen::VectorXd>(pf_result.va.data(), nb);
    Eigen::VectorXd pcalc;
    Eigen::VectorXd qcalc;
    compute_ac_injections(pf_data, vm, va, pcalc, qcalc);

    Eigen::VectorXd pg_bus = pcalc;
    Eigen::VectorXd qg_bus = qcalc;
    for (int b = 0; b < nb; ++b) {
      const double pd = pf_data.has_component_loads
                            ? pf_data.pd_pu[b]
                            : pf_data.ac_buses[static_cast<size_t>(b)].pd_mw / pf_data.base_mva;
      const double qd = pf_data.has_component_loads
                            ? pf_data.qd_pu[b]
                            : pf_data.ac_buses[static_cast<size_t>(b)].qd_mvar / pf_data.base_mva;
      pg_bus[b] += pd;
      qg_bus[b] += qd;
    }

    out.vm = pf_result.vm;
    out.va = pf_result.va;
    out.pg_mw.assign(pf_data.generators.size(), 0.0);
    out.qg_mvar.assign(pf_data.generators.size(), 0.0);

    std::vector<std::vector<int>> gens_by_bus(static_cast<size_t>(nb));
    for (size_t gi = 0; gi < pf_data.generators.size(); ++gi) {
      const auto& g = pf_data.generators[gi];
      if (!g.in_service) {
        continue;
      }
      const int b = g.bus - 1;
      if (b >= 0 && b < nb) {
        gens_by_bus[static_cast<size_t>(b)].push_back(static_cast<int>(gi));
      }
    }

    for (int b = 0; b < nb; ++b) {
      const auto& list = gens_by_bus[static_cast<size_t>(b)];
      if (list.empty()) {
        continue;
      }
      const double pg_each_mw = pg_bus[b] * pf_data.base_mva / static_cast<double>(list.size());
      const double qg_each_mvar = qg_bus[b] * pf_data.base_mva / static_cast<double>(list.size());
      for (int gi : list) {
        const auto& g = pf_data.generators[static_cast<size_t>(gi)];
        out.pg_mw[static_cast<size_t>(gi)] = std::clamp(pg_each_mw, g.pmin_mw, g.pmax_mw);
        out.qg_mvar[static_cast<size_t>(gi)] = std::clamp(qg_each_mvar, g.qmin_mvar, g.qmax_mvar);
      }
    }

    out.objective = 0.0;
    for (size_t gi = 0; gi < pf_data.generators.size(); ++gi) {
      const auto& g = pf_data.generators[gi];
      const double pg = out.pg_mw[gi];
      out.objective += g.cost_c2 * pg * pg + g.cost_c1 * pg + g.cost_c0;
    }
    out.max_constraint_violation = pf_result.residual;
    out.max_stationarity = 0.0;
    out.converged = true;
    return true;
  };

  // Fastest safe path: use existing dispatch and project to AC-PF feasibility.
  if (run_pf_extract(ac_only_sys)) {
    return true;
  }

  // If original dispatch fails PF, try a simple economic dispatch reshaping.
  Eigen::VectorXd pg_dispatch_pu;
  if (solve_economic_dispatch(data, idx, pg_dispatch_pu)) {
    HybridPowerSystem dispatch_sys = ac_only_sys;
    for (int k = 0; k < idx.ng; ++k) {
      const int gi = idx.gen_var_to_gen_index[static_cast<size_t>(k)];
      dispatch_sys.ac.generators[static_cast<size_t>(gi)].pg_mw = pg_dispatch_pu[k] * data.base_mva;
    }
    if (run_pf_extract(dispatch_sys)) {
      return true;
    }
  }

  return false;
}

// Inner nonlinear solver used by the shared parity full-space formulation.
//   Auto      : native parity primal-dual IPM first, Ipopt as a fallback
//   NativeIPM : self-developed parity primal-dual IPM only
//   Ipopt     : embedded Ipopt (filter line-search) only
enum class ParityInnerSolver { Auto, NativeIPM, Ipopt };

// Solve the assembled parity OPF nonlinear program with the embedded Ipopt
// filter line-search NLP solver.  The parity `Problem` already exposes every
// callback Ipopt needs (objective, gradient, equality/inequality residuals and
// Jacobians, and the Lagrangian Hessian), so the model maps 1:1 onto
// engine::NLPModel.  Box bounds are passed as variable bounds; only the
// nonlinear inequalities (`h(x) <= 0`) become general constraints.
//
// `available` is set false when Ipopt is not compiled in; callers then keep the
// native-IPM result.  Ipopt does not return constraint multipliers through this
// adapter, so `lambda_eq` is left empty (LMP extraction is skipped downstream).
parity::IPMResult solve_parity_with_ipopt(const parity::Problem& prob,
                                          const parity::IPMOptions& ipm_opt,
                                          bool& available) {
  parity::IPMResult res;
#ifdef HACDCPF_HAVE_IPOPT
  // The embedded Ipopt links MUMPS 5.6.2 (homebrew).  The Ipopt MUMPS interface
  // struct in MIPSolvers was corrected to the matching 5.6.2 ABI, so the former
  // heap corruption is resolved and this path is enabled by default.  It can
  // still be force-disabled with HACDCPF_DISABLE_IPOPT_OPF if a future toolchain
  // reintroduces an Ipopt/MUMPS ABI mismatch.
  if (std::getenv("HACDCPF_DISABLE_IPOPT_OPF") != nullptr) {
    available = false;
    res.converged = false;
    res.status = "Ipopt disabled (HACDCPF_DISABLE_IPOPT_OPF set)";
    return res;
  }
  available = true;

  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  Eigen::VectorXd x0;
  parity::build_variable_bounds(prob, xmin, xmax);
  parity::build_initial_point(prob, xmin, xmax, x0);

  const int n = prob.vidx.n_total;

  engine::NLPModel nlp;
  nlp.sense = engine::Sense::Minimize;
  nlp.vars.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    engine::VariableMeta vm;
    vm.type = engine::VarType::Continuous;
    vm.lb = std::isfinite(xmin[i]) ? xmin[i] : -1e20;
    vm.ub = std::isfinite(xmax[i]) ? xmax[i] : 1e20;
    nlp.vars[static_cast<size_t>(i)] = vm;
  }
  nlp.x0 = x0;

  nlp.f = [&prob](const Eigen::VectorXd& x) -> double {
    return parity::objective(prob, x);
  };
  nlp.grad = [&prob](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    Eigen::VectorXd hdiag;
    parity::objective_gradient_hessian_diag(prob, x, g, hdiag);
  };
  nlp.g = [&prob](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    parity::EvalWorkspace ws;
    parity::equality_constraints(prob, x, ws, g);
  };
  nlp.jac_g = [&prob](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) {
    // equality_jacobian reads intermediate quantities (p_calc/q_calc, ...) that
    // equality_constraints populates in the shared workspace, so it must run
    // first on the same workspace and point.
    parity::EvalWorkspace ws;
    Eigen::VectorXd g_tmp;
    parity::equality_constraints(prob, x, ws, g_tmp);
    parity::equality_jacobian(prob, x, ws, J);
  };
  nlp.h = [&prob](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    parity::nonlinear_inequality_constraints(prob, x, h);
  };
  nlp.jac_h = [&prob](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) {
    parity::nonlinear_inequality_jacobian(prob, x, J);
  };
  nlp.lagrangian_hess = [&prob](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& lambda,
                                const Eigen::VectorXd* nu,
                                Eigen::SparseMatrix<double>& H) {
    parity::lagrangian_hessian(prob, x, lambda, nu, H, 0.0);
  };

  engine::IpoptAdapter ipopt;
  const engine::SolveResult sol = ipopt.solve_nlp(nlp);

  res.x = (sol.x.size() == n) ? sol.x : x0;
  res.converged = sol.stats.success;
  res.iterations = sol.stats.iterations;
  res.primal_inf = sol.stats.primal_feas;
  res.dual_inf = sol.stats.dual_feas;
  res.complementarity = sol.stats.complementarity;
  res.status = std::string("Ipopt: ") + sol.stats.status;
#else
  (void)prob;
  (void)ipm_opt;
  available = false;
  res.converged = false;
  res.status = "Ipopt not compiled in";
#endif
  return res;
}

ACOPFResult solve_with_parity_ipm(const HybridPowerSystem& sys, const ACOPFOptions& opt,
                                  ParityInnerSolver inner = ParityInnerSolver::Auto) {
  ACOPFResult out;
  // Parity-IPM uses the AML hybrid OPF builder, which enforces the VDC_Q DC-bus
  // voltage equality and the converter capacity circle / quadratic loss.
  {
    auto& sc = out.converter_model_scope;
    // Build the scope tag from the features actually enforced for this run so it
    // stays consistent with the per-feature validity flags below (a constraint
    // family the caller disabled must not appear in the tag).
    std::string tag = "ac-opf-parity-ipm:vsc-free-pq";
    if (opt.enforce_converter_capacity) tag += "+capacity-circle";
    tag += "+vdc-equality";  // always enforced by the parity formulation
    if (opt.enforce_converter_current_limits) tag += "+iac";
    if (opt.enforce_converter_modulation_limits) tag += "+modulation+dcdc-duty";
    if (opt.enforce_branch_limits) tag += "+branch-limits";
    sc.model_scope = tag;
    sc.validity.vsc_loss_modelled = true;
    sc.validity.vsc_capacity_circle_enforced = opt.enforce_converter_capacity;
    sc.validity.vsc_current_limits_enforced = opt.enforce_converter_current_limits;
    sc.validity.vsc_modulation_limits_enforced = opt.enforce_converter_modulation_limits;
    sc.validity.dcdc_duty_ratio_enforced = opt.enforce_converter_modulation_limits;
    sc.validity.vsc_vdc_control_modelled = true;
  }

  parity::ParityOptions form_opt;
  form_opt.load_shedding = true;
  form_opt.voll = 0.0;
  form_opt.eps_iac = 1e-6;
  form_opt.enforce_branch_limits = opt.enforce_branch_limits;
  form_opt.enforce_converter_capacity = opt.enforce_converter_capacity;
  form_opt.enforce_converter_current_limits = opt.enforce_converter_current_limits;
  form_opt.enforce_converter_modulation_limits = opt.enforce_converter_modulation_limits;
  const parity::Problem prob = parity::build_problem(sys, form_opt);
  const auto& vidx = prob.vidx;
  const auto& cidx = prob.cidx;

  parity::IPMOptions ipm_opt;
  ipm_opt.max_iter = std::max(1, opt.max_inner_iterations * std::max(1, opt.max_outer_iterations));
  ipm_opt.tol_primal = std::max(opt.feasibility_tol, 1e-10);
  ipm_opt.tol_dual = std::max(opt.stationarity_tol, 1e-10);
  ipm_opt.tol_complementarity = std::max(opt.barrier_mu_min, 1e-10);
  ipm_opt.regularization = std::max(opt.regularization, 1e-12);
  ipm_opt.alpha_max = std::clamp(opt.interior_fraction, 0.5, 0.9999);
  ipm_opt.verbose = opt.verbose;

  // ── Inner nonlinear solver selection ──────────────────────────────────────
  // Auto: native parity IPM first; if it does not converge, retry the SAME
  // assembled problem with Ipopt (filter line-search) when available.
  parity::IPMResult ipm_res;
  std::string backend_label;
  if (inner == ParityInnerSolver::Ipopt) {
    bool ipopt_ok = false;
    ipm_res = solve_parity_with_ipopt(prob, ipm_opt, ipopt_ok);
    backend_label = "ipopt_filter_linesearch";
    if (!ipopt_ok) {
      // Ipopt explicitly requested but not compiled in — fall back to native.
      ipm_res = parity::solve_primal_dual_ipm(prob, ipm_opt);
      backend_label = "parity_ipm:" + ipm_res.linear_solver + " (ipopt unavailable)";
    }
  } else if (inner == ParityInnerSolver::NativeIPM) {
    ipm_res = parity::solve_primal_dual_ipm(prob, ipm_opt);
    backend_label = "parity_ipm:" + ipm_res.linear_solver;
  } else {  // Auto: native parity IPM, then Ipopt fallback on non-convergence
    ipm_res = parity::solve_primal_dual_ipm(prob, ipm_opt);
    backend_label = "parity_ipm:" + ipm_res.linear_solver;
    if (!ipm_res.converged && opt.allow_fallback) {
      bool ipopt_ok = false;
      const parity::IPMResult ipopt_res =
          solve_parity_with_ipopt(prob, ipm_opt, ipopt_ok);
      if (ipopt_ok && ipopt_res.converged) {
        ipm_res = ipopt_res;
        backend_label = "ipopt_filter_linesearch (auto-fallback)";
      }
    }
  }

  out.converged = ipm_res.converged;
  out.iterations = std::max(0, ipm_res.iterations + 1);
  out.outer_iterations = 1;
  out.max_stationarity = ipm_res.dual_inf;
  out.profiling.linear_solver_backend = backend_label;
  out.solver_path = OPFSolverPath::ParityIPM;
  out.profiling.total_iterations = out.iterations;
  out.profiling.accepted_steps = out.iterations;
  out.profiling.rejected_steps = 0;
  out.profiling.final_barrier_mu = ipm_res.complementarity;

  out.vm.assign(static_cast<size_t>(vidx.n_vm), 1.0);
  out.va.assign(static_cast<size_t>(vidx.n_va), 0.0);
  out.pg_mw.assign(prob.data.generators.size(), 0.0);
  out.qg_mvar.assign(prob.data.generators.size(), 0.0);
  for (size_t gi = 0; gi < prob.data.generators.size(); ++gi) {
    out.pg_mw[gi] = prob.data.generators[gi].pg_mw;
    out.qg_mvar[gi] = prob.data.generators[gi].qg_mvar;
  }

  if (ipm_res.x.size() == vidx.n_total) {
    for (int i = 0; i < vidx.n_vm; ++i) {
      out.vm[static_cast<size_t>(i)] = ipm_res.x[vidx.i_vm + i];
    }
    for (int i = 0; i < vidx.n_va; ++i) {
      out.va[static_cast<size_t>(i)] = ipm_res.x[vidx.i_va + i];
    }
    // Extract DC bus voltages
    if (vidx.n_vdc > 0) {
      out.vdc.resize(static_cast<size_t>(vidx.n_vdc), 1.0);
      for (int i = 0; i < vidx.n_vdc; ++i) {
        out.vdc[static_cast<size_t>(i)] = ipm_res.x[vidx.i_vdc + i];
      }
    }
    for (int k = 0; k < vidx.n_pg; ++k) {
      const int gi = prob.gen_var_to_data[static_cast<size_t>(k)];
      out.pg_mw[static_cast<size_t>(gi)] = ipm_res.x[vidx.i_pg + k] * prob.data.base_mva;
      out.qg_mvar[static_cast<size_t>(gi)] = ipm_res.x[vidx.i_qg + k] * prob.data.base_mva;
    }

    // Extract per-bus load shedding
    if (vidx.n_dpd > 0) {
      out.dpd_mw.resize(static_cast<size_t>(vidx.n_dpd), 0.0);
      out.dqd_mvar.resize(static_cast<size_t>(vidx.n_dqd), 0.0);
      for (int i = 0; i < vidx.n_dpd; ++i) {
        out.dpd_mw[static_cast<size_t>(i)] = ipm_res.x[vidx.i_dpd + i] * prob.data.base_mva;
      }
      for (int i = 0; i < vidx.n_dqd; ++i) {
        out.dqd_mvar[static_cast<size_t>(i)] = ipm_res.x[vidx.i_dqd + i] * prob.data.base_mva;
      }
    }

    // Extract converter AC-side operating points
    if (vidx.n_pac > 0) {
      out.pac_mw.resize(static_cast<size_t>(vidx.n_pac), 0.0);
      out.qac_mvar.resize(static_cast<size_t>(vidx.n_pac), 0.0);
      for (int k = 0; k < vidx.n_pac; ++k) {
        out.pac_mw[static_cast<size_t>(k)] = ipm_res.x[vidx.i_pac + k] * prob.data.base_mva;
        out.qac_mvar[static_cast<size_t>(k)] = ipm_res.x[vidx.i_qac + k] * prob.data.base_mva;
      }
    }

    // Extract enhanced DER variable results + build identity maps
    if (vidx.n_pren > 0) {
      const size_t nren = static_cast<size_t>(vidx.n_pren);
      out.pren_mw.resize(nren, 0.0);
      out.qren_mvar.resize(nren, 0.0);
      out.ren_map.resize(nren);
      for (int k = 0; k < vidx.n_pren; ++k) {
        const size_t ku = static_cast<size_t>(k);
        out.pren_mw[ku] = ipm_res.x[vidx.i_pren + k] * prob.data.base_mva;
        out.ren_map[ku].original_index = prob.ren_var_to_data[ku];
        out.ren_map[ku].source_type    = prob.ren_source[ku];
      }
      for (int k = 0; k < vidx.n_qren; ++k) {
        out.qren_mvar[static_cast<size_t>(k)] = ipm_res.x[vidx.i_qren + k] * prob.data.base_mva;
      }
    }
    if (vidx.n_pstor > 0 || vidx.n_pstordc > 0) {
      const size_t nstor_ac = static_cast<size_t>(vidx.n_pstor);
      const size_t nstor_dc = static_cast<size_t>(vidx.n_pstordc);
      const size_t nstor_total = nstor_ac + nstor_dc;
      out.pstor_mw.resize(nstor_total, 0.0);
      out.qstor_mvar.resize(nstor_total, 0.0);
      out.stor_map.resize(nstor_total);

      for (int k = 0; k < vidx.n_pstor; ++k) {
        const size_t ku = static_cast<size_t>(k);
        out.pstor_mw[ku] = ipm_res.x[vidx.i_pstor + k] * prob.data.base_mva;
        out.stor_map[ku].original_index = prob.stor_var_to_data[ku];
        out.stor_map[ku].source_type    = 0;  // AC storage
      }
      for (int k = 0; k < vidx.n_qstor; ++k) {
        out.qstor_mvar[static_cast<size_t>(k)] =
            ipm_res.x[vidx.i_qstor + k] * prob.data.base_mva;
      }

      for (int k = 0; k < vidx.n_pstordc; ++k) {
        const size_t ku = nstor_ac + static_cast<size_t>(k);
        out.pstor_mw[ku] = ipm_res.x[vidx.i_pstordc + k] * prob.data.base_mva;
        out.qstor_mvar[ku] = 0.0;
        out.stor_map[ku].original_index = prob.stor_dc_var_to_data[static_cast<size_t>(k)];
        out.stor_map[ku].source_type    = 1;  // DC storage
      }
    }
    if (vidx.n_pdcdc > 0) {
      const size_t ndcdc = static_cast<size_t>(vidx.n_pdcdc);
      out.pdcdc_mw.resize(ndcdc, 0.0);
      out.dcdc_map.resize(ndcdc);
      for (int k = 0; k < vidx.n_pdcdc; ++k) {
        const size_t ku = static_cast<size_t>(k);
        out.pdcdc_mw[ku] = ipm_res.x[vidx.i_pdcdc + k] * prob.data.base_mva;
        out.dcdc_map[ku].original_index = prob.dcdc_var_to_data[ku];
        out.dcdc_map[ku].source_type    = 0;
      }
    }
    if (vidx.n_pflex > 0) {
      const size_t nflex = static_cast<size_t>(vidx.n_pflex);
      out.pflex_mw.resize(nflex, 0.0);
      out.flex_map.resize(nflex);
      for (int k = 0; k < vidx.n_pflex; ++k) {
        const size_t ku = static_cast<size_t>(k);
        out.pflex_mw[ku] = ipm_res.x[vidx.i_pflex + k] * prob.data.base_mva;
        out.flex_map[ku].original_index = prob.flex_var_to_data[ku];
        out.flex_map[ku].source_type    = 0;
      }
    }

    out.objective = parity::objective(prob, ipm_res.x);

    // Extract LMP from dual variables of power balance constraints
    if (ipm_res.lambda_eq.size() >= cidx.i_qbal_ac + cidx.n_qbal_ac) {
      const double base = prob.data.base_mva;
      out.lmp_p.resize(static_cast<size_t>(cidx.n_pbal_ac), 0.0);
      out.lmp_q.resize(static_cast<size_t>(cidx.n_qbal_ac), 0.0);
      for (int i = 0; i < cidx.n_pbal_ac; ++i) {
        out.lmp_p[static_cast<size_t>(i)] =
            ipm_res.lambda_eq[cidx.i_pbal_ac + i] * prob.scale_p / base;
      }
      for (int i = 0; i < cidx.n_qbal_ac; ++i) {
        out.lmp_q[static_cast<size_t>(i)] =
            ipm_res.lambda_eq[cidx.i_qbal_ac + i] * prob.scale_q / base;
      }
    }

    parity::EvalWorkspace eq_ws;
    Eigen::VectorXd g_eq;
    parity::equality_constraints(prob, ipm_res.x, eq_ws, g_eq);
    Eigen::VectorXd h_nonlin;
    parity::nonlinear_inequality_constraints(prob, ipm_res.x, h_nonlin);
    Eigen::VectorXd xmin;
    Eigen::VectorXd xmax;
    parity::build_variable_bounds(prob, xmin, xmax);

    double max_bound = 0.0;
    for (int i = 0; i < vidx.n_total; ++i) {
      if (std::isfinite(xmin[i])) {
        max_bound = std::max(max_bound, xmin[i] - ipm_res.x[i]);
      }
      if (std::isfinite(xmax[i])) {
        max_bound = std::max(max_bound, ipm_res.x[i] - xmax[i]);
      }
    }
    const double max_hplus =
        (h_nonlin.size() > 0) ? std::max(0.0, h_nonlin.maxCoeff()) : 0.0;
    out.max_constraint_violation = std::max({inf_norm(g_eq), max_hplus, max_bound});
  } else {
    out.objective = std::numeric_limits<double>::infinity();
    out.max_constraint_violation = std::numeric_limits<double>::infinity();
  }

  if (out.converged) {
    out.status = "converged (" + backend_label + ")";
  } else {
    out.status = "not converged: " + ipm_res.status;
  }

  // Un-project merged results back to original bus count
  if (prob.data.bus_merge_map && prob.data.bus_merge_map->has_merges()) {
    out.vm = unproject_bus_vector(out.vm, *prob.data.bus_merge_map);
    out.va = unproject_bus_vector(out.va, *prob.data.bus_merge_map);
    if (!out.lmp_p.empty())
      out.lmp_p = unproject_bus_vector(out.lmp_p, *prob.data.bus_merge_map);
    if (!out.lmp_q.empty())
      out.lmp_q = unproject_bus_vector(out.lmp_q, *prob.data.bus_merge_map);
  }

  return out;
}

bool contains_hybrid_acdc_components(const HybridPowerSystem& sys) {
  return !sys.dc.buses.empty()               ||
         !sys.dc.branches.empty()            ||
         !sys.dc.loads.empty()               ||
         !sys.vsc_converters.empty()         ||
         !sys.dc.dcdc_converters.empty()     ||
         !sys.dc.dc_circuit_breakers.empty() ||
         !sys.dc.storage.empty()             ||
         !sys.dc.pv_arrays.empty()           ||
         !sys.dc.static_generators.empty()   ||
         !sys.dc.dc_static_generators.empty();
}

}  // namespace

ACOPFResult solve_ac_opf(const HybridPowerSystem& sys, const ACOPFOptions& opt_in) {
  ACOPFResult out;
  ACOPFOptions opt = opt_in;
  // Native AC-OPF models converters as free P_ac/Q_ac/P_dc box-bounded variables
  // with the capacity circle and quadratic loss, but is control-mode-agnostic:
  // it does NOT pin VDC_Q DC-bus voltages (unlike the parity-IPM / AML path).
  {
    auto& sc = out.converter_model_scope;
    sc.model_scope = "ac-opf-native:vsc-free-pq+capacity-circle";
    sc.validity.vsc_loss_modelled = true;
    sc.validity.vsc_capacity_circle_enforced = true;
    sc.validity.vsc_vdc_control_modelled = false;
  }

  if (opt.max_inner_iterations <= 0) {
    opt.max_inner_iterations = 80;
  }
  if (opt.max_outer_iterations <= 0) {
    opt.max_outer_iterations = 8;
  }
  if (opt.max_line_search_steps <= 0) {
    opt.max_line_search_steps = 20;
  }
  if (opt.feasibility_tol <= 0.0) {
    opt.feasibility_tol = 1e-6;
  }
  if (opt.stationarity_tol <= 0.0) {
    opt.stationarity_tol = 1e-6;
  }
  if (opt.barrier_mu0 <= 0.0) {
    opt.barrier_mu0 = 1e-2;
  }
  if (opt.barrier_mu_reduction <= 0.0 || opt.barrier_mu_reduction >= 1.0) {
    opt.barrier_mu_reduction = 0.2;
  }
  if (opt.barrier_mu_min <= 0.0) {
    opt.barrier_mu_min = 1e-8;
  }
  if (opt.regularization <= 0.0) {
    opt.regularization = 1e-6;
  }
  if (opt.merit_penalty <= 0.0) {
    opt.merit_penalty = 10.0;
  }
  if (opt.step_backoff <= 0.0 || opt.step_backoff >= 1.0) {
    opt.step_backoff = 0.5;
  }
  if (opt.interior_fraction <= 0.0 || opt.interior_fraction >= 1.0) {
    opt.interior_fraction = 0.995;
  }

  if (sys.ac.buses.empty()) {
    out.status = "AC OPF failed: empty AC bus set.";
    return out;
  }

  // Working copy: promote external grids with OPF cost into generator variables.
  // Allows nighttime timesteps (no PV → sys.ac.generators is empty) to be
  // dispatched through the grid-tie connection rather than falling back to PF.
  HybridPowerSystem sys_work = sys;
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    if (eg.cost_c2 == 0.0 && eg.cost_c1 == 0.0) continue;
    bool already_on_bus = false;
    for (const auto& g : sys_work.ac.generators)
      if (g.bus == eg.bus) { already_on_bus = true; break; }
    if (already_on_bus) continue;
    const double pmax = (eg.s_sc_max_mva > 0.0) ? eg.s_sc_max_mva : 1000.0;
    Generator eg_gen;
    eg_gen.index      = static_cast<int>(sys_work.ac.generators.size());
    eg_gen.bus        = eg.bus;
    eg_gen.in_service = true;
    eg_gen.name       = eg.name.empty() ? ("EG_opf_" + std::to_string(eg.index)) : eg.name;
    eg_gen.vg_pu      = eg.vm_pu;
    eg_gen.pmax_mw    =  pmax;
    eg_gen.pmin_mw    = -pmax;
    eg_gen.qmax_mvar  =  pmax;
    eg_gen.qmin_mvar  = -pmax;
    eg_gen.cost_c2    = eg.cost_c2;
    eg_gen.cost_c1    = eg.cost_c1;
    eg_gen.cost_c0    = eg.cost_c0;
    sys_work.ac.generators.push_back(std::move(eg_gen));
  }

  const bool has_hybrid_acdc = contains_hybrid_acdc_components(sys);
  const bool dropped_dc = has_hybrid_acdc;
  auto append_hybrid_fallback_suppression = [&]() {
    if (!dropped_dc || !opt.allow_fallback) {
      return;
    }
    const std::string hint =
        "AC-only economic-dispatch fallback suppressed because the case contains DC/VSC components.";
    const auto already_present = std::any_of(
        out.infeasibility_hints.begin(), out.infeasibility_hints.end(),
        [&](const std::string& existing) { return existing.find(hint) != std::string::npos; });
    if (!already_present) {
      out.infeasibility_hints.push_back(hint);
    }
  };

  // ── Graph topology pre-check ────────────────────────────────────────────────
  // Detect topology problems before building the full IPM formulation.
  {
    namespace gr = hacdcpf::graph;
    const auto g    = gr::build_power_system_graph(sys);
    const auto topo = gr::analyze_topology(g);
    const bool has_valid = std::any_of(
        topo.islands.begin(), topo.islands.end(),
        [](const gr::IslandInfo& i) { return i.status == gr::IslandStatus::Valid; });
    if (!has_valid) {
      out.status = "AC OPF infeasible: no island with slack bus";
      for (const auto& diag : topo.diagnostics)
        out.infeasibility_hints.push_back(diag.message);
      if (out.infeasibility_hints.empty())
        out.infeasibility_hints.push_back("No AC island contains a slack bus");
      append_hybrid_fallback_suppression();
      return out;
    }
    for (const auto& isl : topo.islands) {
      if (isl.status == gr::IslandStatus::IsolatedLoad ||
          isl.status == gr::IslandStatus::NoSlack) {
        out.infeasibility_hints.push_back(
            "Island " + std::to_string(isl.island_id) +
            " (" + std::to_string(isl.bus_ids.size()) +
            " buses) has no slack — buses will be unservable");
      }
    }
  }

  // ── High-level solver-backend selection ───────────────────────────────────
  // Translate ACOPFOptions::ac_solver_backend into the legacy primal-dual flags
  // and the inner nonlinear solver.  The parity full-space formulation is shared
  // by the native IPM and Ipopt; only the inner Newton engine differs.
  ParityInnerSolver inner = ParityInnerSolver::Auto;
  switch (opt.ac_solver_backend) {
    case ACOPFSolverBackend::ParityIPM:
      opt.enable_primal_dual = true;
      opt.use_parity_ipm = true;
      inner = ParityInnerSolver::NativeIPM;
      break;
    case ACOPFSolverBackend::Ipopt:
      opt.enable_primal_dual = true;
      opt.use_parity_ipm = true;
      inner = ParityInnerSolver::Ipopt;
      break;
    case ACOPFSolverBackend::EconomicDispatch:
      // Force the fast economic-dispatch + AC PF path (pure-AC only).
      opt.enable_primal_dual = false;
      opt.use_parity_ipm = false;
      break;
    case ACOPFSolverBackend::Auto:
    default:
      inner = ParityInnerSolver::Auto;
      break;
  }

  // Auto-enable a hybrid-capable primal-dual path when any DC/VSC subsystem is
  // present (unless the caller explicitly chose the economic-dispatch backend).
  if (has_hybrid_acdc && !opt.enable_primal_dual &&
      opt.ac_solver_backend != ACOPFSolverBackend::EconomicDispatch) {
    opt.enable_primal_dual = true;
    opt.use_parity_ipm = true;
  }

  if (opt.enable_primal_dual && opt.use_parity_ipm) {
    // The parity path internally applies the Ipopt fallback when inner == Auto
    // and opt.allow_fallback is set, so no separate recursive economic-dispatch
    // fallback is needed here.
    return solve_with_parity_ipm(sys, opt, inner);
  }

  HybridPowerSystem ac_only = sys_work;
  ac_only.dc.buses.clear();
  ac_only.dc.branches.clear();
  ac_only.vsc_converters.clear();

  if (!opt.enable_primal_dual) {
    if (dropped_dc) {
      out.status = "AC OPF failed: hybrid AC/DC cases require a hybrid-capable primal-dual solver; "
                   "AC-only economic-dispatch fallback was not used.";
      append_hybrid_fallback_suppression();
      out.infeasibility_hints.push_back(
          "Hybrid AC/DC fallback must preserve DC/VSC equations; enable_primal_dual/use_parity_ipm is required.");
      return out;
    }
    core::SolverData fallback_data = core::make_solver_data(ac_only, LossModelType::Linear);
    ACOPFIndex fallback_idx = build_index(fallback_data);
    if (fallback_idx.nb == 0 || fallback_idx.nvar == 0 || fallback_idx.neq == 0) {
      out.status = "AC OPF failed: degenerate model dimensions.";
      return out;
    }
    if (fallback_idx.ng == 0) {
      out.status = "AC OPF failed: no in-service generators.";
      return out;
    }
    if (try_dispatch_pf_fallback(ac_only, fallback_data, fallback_idx, out)) {
      out.status = "converged (fast economic-dispatch + AC PF path)";
      return out;
    }
    out.status = "AC OPF failed: fast fallback path did not converge.";
    return out;
  }

  core::SolverData data = core::make_solver_data(sys_work, LossModelType::Linear);
  ACOPFIndex idx = build_index(data);
  if (idx.nb == 0 || idx.nvar == 0 || idx.neq == 0) {
    out.status = "AC OPF failed: degenerate model dimensions.";
    append_hybrid_fallback_suppression();
    return out;
  }
  if (idx.ng == 0) {
    out.status = "AC OPF failed: no in-service generators.";
    append_hybrid_fallback_suppression();
    return out;
  }

  core::JacobianContext pf_ctx = build_pf_context(idx);
  core::JacobianPattern pf_pattern = core::build_jacobian_pattern(data, pf_ctx);
  JWorkspace j_ws = build_jacobian_workspace(data, pf_pattern, idx);
  NonlinearLimitWorkspace nl_ws = build_nonlinear_limit_workspace(data, idx);
  KKTWorkspace kkt_ws = build_kkt_workspace(j_ws, idx.nvar, idx.neq);
  std::vector<bool> constrained_var_col(static_cast<size_t>(idx.nvar), false);
  const int* j_outer = j_ws.matrix.outerIndexPtr();
  for (int col = 0; col < idx.nvar; ++col) {
    constrained_var_col[static_cast<size_t>(col)] = (j_outer[col] < j_outer[col + 1]);
  }
  out.profiling.linear_solver_backend = kkt_ws.linear_solver->backend_name();
  out.solver_path = OPFSolverPath::NativeAC;

  Eigen::VectorXd x;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  initialize_bounds_and_state(data, idx, x, lower, upper);
  if (!is_strictly_inside(x, lower, upper)) {
    out.status = "AC OPF failed: unable to initialize a strictly interior point.";
    append_hybrid_fallback_suppression();
    return out;
  }

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(idx.neq);
  Eigen::VectorXd lambda_ls = Eigen::VectorXd::Zero(idx.neq);
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(idx.nvar);
  Eigen::VectorXd hdiag = Eigen::VectorXd::Zero(idx.nvar);
  Eigen::VectorXd grad_lag = Eigen::VectorXd::Zero(idx.nvar);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(idx.nvar + idx.neq);
  Eigen::VectorXd delta = Eigen::VectorXd::Zero(idx.nvar + idx.neq);

  Eigen::VectorXd vm;
  Eigen::VectorXd va;
  Eigen::VectorXd vdc;
  Eigen::VectorXd pac;
  Eigen::VectorXd qac;
  Eigen::VectorXd pdc;
  Eigen::VectorXd pg_bus;
  Eigen::VectorXd qg_bus;
  EqualityEvalWorkspace eq_ws;
  Eigen::VectorXd g_eq;
  NonlinearConstraintMetrics nl_metrics;

  Eigen::VectorXd x_trial = x;
  Eigen::VectorXd vm_trial;
  Eigen::VectorXd va_trial;
  Eigen::VectorXd vdc_trial;
  Eigen::VectorXd pac_trial;
  Eigen::VectorXd qac_trial;
  Eigen::VectorXd pdc_trial;
  Eigen::VectorXd pg_bus_trial;
  Eigen::VectorXd qg_bus_trial;
  EqualityEvalWorkspace eq_ws_trial;
  Eigen::VectorXd g_eq_trial;

  bool converged = false;
  std::string failure_reason = "maximum iterations reached";
  double mu = std::max(opt.barrier_mu0, opt.barrier_mu_min);
  double regularization_dynamic = std::max(opt.regularization, 1e-9);
  double merit_penalty_dynamic = std::max(opt.merit_penalty, 10.0);
  double objective_weight_dynamic = 0.05;
  int stagnation_count = 0;
  double max_eq_p_final = 0.0;
  double max_eq_q_final = 0.0;
  double max_eq_dc_final = 0.0;
  double max_eq_conv_final = 0.0;
  double max_ineq_final = 0.0;

  for (int outer = 0; outer < opt.max_outer_iterations; ++outer) {
    out.outer_iterations = outer + 1;
    bool inner_converged_at_mu = false;

    for (int it = 0; it < opt.max_inner_iterations; ++it) {
      out.iterations += 1;
      out.profiling.total_iterations += 1;

      unpack_state(idx, x, vm, va, vdc, pac, qac, pdc, pg_bus, qg_bus);
      const double max_eq_mismatch = evaluate_hybrid_equalities_and_pf_jacobian(data,
                                                                                 pf_ctx,
                                                                                 pf_pattern,
                                                                                 idx,
                                                                                 vm,
                                                                                 va,
                                                                                 vdc,
                                                                                 pac,
                                                                                 qac,
                                                                                 pdc,
                                                                                 pg_bus,
                                                                                 qg_bus,
                                                                                 opt.ac_eval_threads,
                                                                                 eq_ws,
                                                                                 g_eq);
      update_jacobian_values(data, pf_pattern, idx, vm, vdc, pac, qac, j_ws);

      const double objective = evaluate_objective(data, idx, x);
      accumulate_objective_grad_hdiag(data, idx, x, grad, hdiag);
      grad *= objective_weight_dynamic;
      hdiag *= objective_weight_dynamic;
      const double objective_scaled = objective_weight_dynamic * objective;

      double var_barrier_value = 0.0;
      if (!accumulate_log_barrier(x, lower, upper, mu, grad, hdiag, var_barrier_value)) {
        failure_reason = "left the interior of variable bounds";
        converged = false;
        goto finalize;
      }
      const double ineq_gate = std::clamp(1e-2 / std::max(max_eq_mismatch, 1e-12), 1e-3, 1.0);
      const double ineq_penalty_scale = 100.0 * merit_penalty_dynamic * ineq_gate;
      nl_metrics = accumulate_nonlinear_limit_terms(
          idx, nl_ws, vm, va, pac, qac, ineq_penalty_scale, grad, hdiag);
      const double max_ineq_violation = std::max(0.0, nl_metrics.max_h);
      const double max_mismatch = std::max(max_eq_mismatch, max_ineq_violation);
      const double barrier_value = var_barrier_value + nl_metrics.barrier_value;

      const double eq_norm_sq = g_eq.squaredNorm();
      const double infeas_current = std::sqrt(eq_norm_sq + nl_metrics.violation_norm_sq);
      grad_lag = grad;
      double max_stationarity = inf_norm(grad_lag);
      if (infeas_current <= 1e-3) {
        max_stationarity = std::min(max_stationarity, dual_least_squares_stationarity(j_ws.matrix, grad, lambda_ls));
      }
      out.max_constraint_violation = max_mismatch;
      out.max_stationarity = max_stationarity;
      out.objective = objective;
      out.profiling.final_barrier_mu = mu;

      if (max_mismatch <= opt.feasibility_tol && max_stationarity <= opt.stationarity_tol) {
        inner_converged_at_mu = true;
        if (mu <= opt.barrier_mu_min * 1.01) {
          converged = true;
          goto finalize;
        }
        break;
      }

      const double merit_current =
          objective_scaled + barrier_value +
          0.5 * merit_penalty_dynamic * (eq_norm_sq + nl_metrics.violation_norm_sq);
      bool accepted = false;
      bool accepted_via_restoration = false;
      double accepted_alpha = 0.0;
      double accepted_infeas = std::numeric_limits<double>::infinity();
      Eigen::VectorXd accepted_dlambda = Eigen::VectorXd::Zero(idx.neq);
      double reg_trial = std::max(regularization_dynamic, 1e-12);

      constexpr int kTrustAttempts = 6;
      constexpr double kMaxDxInf = 0.25;
      constexpr double kMaxDlambdaAbs = 1e3;
      const bool force_restoration =
          (stagnation_count >= 3) && (infeas_current > std::max(1e-3, 10.0 * opt.feasibility_tol));
      const int trust_attempts = force_restoration ? 0 : kTrustAttempts;
      for (int trust = 0; trust < trust_attempts && !accepted; ++trust) {
        update_kkt_values(j_ws, kkt_ws, hdiag, reg_trial);
        if (!kkt_ws.analyzed) {
          kkt_ws.linear_solver->analyze_pattern(kkt_ws.matrix);
          kkt_ws.analyzed = true;
          out.profiling.analyze_calls += 1;
        }
        if (!kkt_ws.linear_solver->factorize(kkt_ws.matrix)) {
          out.profiling.rejected_steps += 1;
          reg_trial *= 10.0;
          continue;
        }
        out.profiling.factorization_calls += 1;

        rhs.head(idx.nvar) = -grad_lag;
        rhs.tail(idx.neq) = -g_eq;
        if (!kkt_ws.linear_solver->solve(rhs, delta)) {
          out.profiling.rejected_steps += 1;
          reg_trial *= 10.0;
          continue;
        }
        out.profiling.linear_solve_calls += 1;

        Eigen::VectorXd dx = delta.head(idx.nvar);
        Eigen::VectorXd dlambda = delta.tail(idx.neq);
        if (!dx.allFinite() || !dlambda.allFinite()) {
          out.profiling.rejected_steps += 1;
          reg_trial *= 10.0;
          continue;
        }
        for (int i = 0; i < idx.nvar; ++i) {
          const double limit = 0.25 * (upper[i] - lower[i]);
          if (std::isfinite(limit) && limit > 0.0) {
            dx[i] = std::clamp(dx[i], -limit, limit);
          }
        }
        const double dx_inf = inf_norm(dx);
        if (std::isfinite(dx_inf) && dx_inf > kMaxDxInf) {
          dx *= (kMaxDxInf / dx_inf);
        }
        for (int i = 0; i < idx.neq; ++i) {
          dlambda[i] = std::clamp(dlambda[i], -kMaxDlambdaAbs, kMaxDlambdaAbs);
        }

        double alpha = max_feasible_step(x, dx, lower, upper, opt.interior_fraction);
        double best_alpha = 0.0;
        double best_infeas = std::numeric_limits<double>::infinity();
        double best_merit = std::numeric_limits<double>::infinity();
        Eigen::VectorXd best_x = x;

        for (int ls = 0; ls < opt.max_line_search_steps; ++ls) {
          if (!(alpha > 0.0)) {
            break;
          }
          x_trial = x + alpha * dx;
          if (!is_strictly_inside(x_trial, lower, upper)) {
            alpha *= opt.step_backoff;
            continue;
          }

          unpack_state(idx,
                       x_trial,
                       vm_trial,
                       va_trial,
                       vdc_trial,
                       pac_trial,
                       qac_trial,
                       pdc_trial,
                       pg_bus_trial,
                       qg_bus_trial);
          evaluate_hybrid_equalities_only(data,
                                          pf_ctx,
                                          pf_pattern,
                                          idx,
                                          vm_trial,
                                          va_trial,
                                          vdc_trial,
                                          pac_trial,
                                          qac_trial,
                                          pdc_trial,
                                          pg_bus_trial,
                                          qg_bus_trial,
                                          opt.ac_eval_threads,
                                          eq_ws_trial,
                                          g_eq_trial);

          const double objective_trial = evaluate_objective(data, idx, x_trial);
          const double objective_trial_scaled = objective_weight_dynamic * objective_trial;
          Eigen::VectorXd grad_tmp = Eigen::VectorXd::Zero(idx.nvar);
          Eigen::VectorXd hdiag_tmp = Eigen::VectorXd::Zero(idx.nvar);
          accumulate_objective_grad_hdiag(data, idx, x_trial, grad_tmp, hdiag_tmp);
          grad_tmp *= objective_weight_dynamic;
          hdiag_tmp *= objective_weight_dynamic;
          double var_barrier_trial = 0.0;
          if (!accumulate_log_barrier(x_trial, lower, upper, mu, grad_tmp, hdiag_tmp, var_barrier_trial)) {
            alpha *= opt.step_backoff;
            continue;
          }
          const NonlinearConstraintMetrics nl_trial = accumulate_nonlinear_limit_terms(
              idx, nl_ws, vm_trial, va_trial, pac_trial, qac_trial, ineq_penalty_scale, grad_tmp, hdiag_tmp);

          const double eq_norm_sq_trial = g_eq_trial.squaredNorm();
          const double infeas_trial = std::sqrt(eq_norm_sq_trial + nl_trial.violation_norm_sq);
          const double merit_trial = objective_trial_scaled + var_barrier_trial + nl_trial.barrier_value +
                                     0.5 * merit_penalty_dynamic * (eq_norm_sq_trial + nl_trial.violation_norm_sq);
          if (infeas_trial < best_infeas || (std::abs(infeas_trial - best_infeas) < 1e-12 && merit_trial < best_merit)) {
            best_infeas = infeas_trial;
            best_merit = merit_trial;
            best_alpha = alpha;
            best_x = x_trial;
          }
          const bool merit_improved = std::isfinite(merit_trial) && merit_trial < merit_current;
          const bool infeas_improved = infeas_trial < (1.0 - 5e-3 * alpha) * infeas_current;
          const bool near_feasible_for_merit =
              infeas_current <= std::max(1e-3, 10.0 * opt.feasibility_tol);
          if (infeas_improved || (near_feasible_for_merit && merit_improved)) {
            accepted = true;
            accepted_alpha = alpha;
            accepted_infeas = infeas_trial;
            accepted_dlambda = dlambda;
            break;
          }
          alpha *= opt.step_backoff;
        }

        if (!accepted && best_alpha > 0.0) {
          const bool best_improves_infeas = best_infeas < 0.995 * infeas_current;
          const bool best_improves_merit = best_merit < merit_current;
          const bool close_to_feasible = infeas_current <= std::max(1e-4, 10.0 * opt.feasibility_tol);
          const bool near_neutral_step =
              close_to_feasible && (best_infeas <= 1.005 * infeas_current) && (best_merit <= 1.005 * merit_current);
          if (!(best_improves_infeas || (close_to_feasible && best_improves_merit) || near_neutral_step)) {
            out.profiling.rejected_steps += 1;
            reg_trial *= 10.0;
            continue;
          }
          accepted = true;
          accepted_alpha = near_neutral_step ? std::min(best_alpha, 5e-3) : best_alpha;
          accepted_infeas = best_infeas;
          accepted_dlambda = dlambda;
          x_trial = best_x;
        }

        if (accepted) {
          regularization_dynamic = std::max(opt.regularization, reg_trial * 0.5);
          break;
        }
        out.profiling.rejected_steps += 1;
        reg_trial *= 10.0;
      }

      if (!accepted) {
        constexpr int kRestorationAttempts = 4;
        constexpr double kMaxRestoreDxInf = 0.12;
        Eigen::VectorXd hdiag_restore = Eigen::VectorXd::Zero(idx.nvar);
        double restore_reg = std::max(regularization_dynamic, 1e-6);
        for (int restore = 0; restore < kRestorationAttempts && !accepted; ++restore) {
          out.profiling.rejected_steps += 1;
          update_kkt_values(j_ws, kkt_ws, hdiag_restore, restore_reg);
          if (!kkt_ws.analyzed) {
            kkt_ws.linear_solver->analyze_pattern(kkt_ws.matrix);
            kkt_ws.analyzed = true;
            out.profiling.analyze_calls += 1;
          }
          if (!kkt_ws.linear_solver->factorize(kkt_ws.matrix)) {
            restore_reg *= 10.0;
            continue;
          }
          out.profiling.factorization_calls += 1;

          rhs.head(idx.nvar).setZero();
          rhs.tail(idx.neq) = -g_eq;
          if (!kkt_ws.linear_solver->solve(rhs, delta)) {
            restore_reg *= 10.0;
            continue;
          }
          out.profiling.linear_solve_calls += 1;

          Eigen::VectorXd dx = delta.head(idx.nvar);
          if (!dx.allFinite()) {
            restore_reg *= 10.0;
            continue;
          }
          for (int i = 0; i < idx.nvar; ++i) {
            const double limit = 0.10 * (upper[i] - lower[i]);
            if (std::isfinite(limit) && limit > 0.0) {
              dx[i] = std::clamp(dx[i], -limit, limit);
            }
          }
          const double dx_inf = inf_norm(dx);
          if (std::isfinite(dx_inf) && dx_inf > kMaxRestoreDxInf) {
            dx *= (kMaxRestoreDxInf / dx_inf);
          }

          double alpha = max_feasible_step(x, dx, lower, upper, opt.interior_fraction);
          double best_alpha = 0.0;
          double best_infeas = std::numeric_limits<double>::infinity();
          Eigen::VectorXd best_x = x;
          for (int ls = 0; ls < opt.max_line_search_steps; ++ls) {
            if (!(alpha > 0.0)) {
              break;
            }
            x_trial = x + alpha * dx;
            if (!is_strictly_inside(x_trial, lower, upper)) {
              alpha *= opt.step_backoff;
              continue;
            }
            unpack_state(idx,
                         x_trial,
                         vm_trial,
                         va_trial,
                         vdc_trial,
                         pac_trial,
                         qac_trial,
                         pdc_trial,
                         pg_bus_trial,
                         qg_bus_trial);
            evaluate_hybrid_equalities_only(data,
                                            pf_ctx,
                                            pf_pattern,
                                            idx,
                                            vm_trial,
                                            va_trial,
                                            vdc_trial,
                                            pac_trial,
                                            qac_trial,
                                            pdc_trial,
                                            pg_bus_trial,
                                            qg_bus_trial,
                                            opt.ac_eval_threads,
                                            eq_ws_trial,
                                            g_eq_trial);
            Eigen::VectorXd grad_restore = Eigen::VectorXd::Zero(idx.nvar);
            Eigen::VectorXd hdiag_restore_trial = Eigen::VectorXd::Zero(idx.nvar);
            const NonlinearConstraintMetrics nl_trial = accumulate_nonlinear_limit_terms(
                idx, nl_ws, vm_trial, va_trial, pac_trial, qac_trial, 0.0, grad_restore, hdiag_restore_trial);
            const double infeas_trial = std::sqrt(g_eq_trial.squaredNorm() + nl_trial.violation_norm_sq);
            if (infeas_trial < best_infeas) {
              best_infeas = infeas_trial;
              best_alpha = alpha;
              best_x = x_trial;
            }
            if (infeas_trial < (1.0 - 1e-3 * alpha) * infeas_current) {
              accepted = true;
              accepted_via_restoration = true;
              accepted_alpha = alpha;
              accepted_infeas = infeas_trial;
              accepted_dlambda.setZero();
              break;
            }
            alpha *= opt.step_backoff;
          }
          if (!accepted && best_alpha > 0.0 && best_infeas < 0.995 * infeas_current) {
            accepted = true;
            accepted_via_restoration = true;
            accepted_alpha = best_alpha;
            accepted_infeas = best_infeas;
            accepted_dlambda.setZero();
            x_trial = best_x;
          }
          if (!accepted) {
            restore_reg *= 10.0;
          } else {
            regularization_dynamic = std::max(opt.regularization, restore_reg);
          }
        }
      }

      if (!accepted) {
        Eigen::VectorXd dx = -j_ws.matrix.transpose() * g_eq;
        if (dx.allFinite()) {
          constexpr double kMaxGradFallbackInf = 0.08;
          const double dx_inf = inf_norm(dx);
          if (std::isfinite(dx_inf) && dx_inf > kMaxGradFallbackInf) {
            dx *= (kMaxGradFallbackInf / dx_inf);
          }
          double alpha = max_feasible_step(x, dx, lower, upper, opt.interior_fraction);
          double best_alpha = 0.0;
          double best_infeas = std::numeric_limits<double>::infinity();
          Eigen::VectorXd best_x = x;
          for (int ls = 0; ls < opt.max_line_search_steps; ++ls) {
            if (!(alpha > 0.0)) {
              break;
            }
            x_trial = x + alpha * dx;
            if (!is_strictly_inside(x_trial, lower, upper)) {
              alpha *= opt.step_backoff;
              continue;
            }
            unpack_state(idx,
                         x_trial,
                         vm_trial,
                         va_trial,
                         vdc_trial,
                         pac_trial,
                         qac_trial,
                         pdc_trial,
                         pg_bus_trial,
                         qg_bus_trial);
            evaluate_hybrid_equalities_only(data,
                                            pf_ctx,
                                            pf_pattern,
                                            idx,
                                            vm_trial,
                                            va_trial,
                                            vdc_trial,
                                            pac_trial,
                                            qac_trial,
                                            pdc_trial,
                                            pg_bus_trial,
                                            qg_bus_trial,
                                            opt.ac_eval_threads,
                                            eq_ws_trial,
                                            g_eq_trial);
            Eigen::VectorXd grad_dummy = Eigen::VectorXd::Zero(idx.nvar);
            Eigen::VectorXd hdiag_dummy = Eigen::VectorXd::Zero(idx.nvar);
            const NonlinearConstraintMetrics nl_trial = accumulate_nonlinear_limit_terms(
                idx, nl_ws, vm_trial, va_trial, pac_trial, qac_trial, 0.0, grad_dummy, hdiag_dummy);
            const double infeas_trial = std::sqrt(g_eq_trial.squaredNorm() + nl_trial.violation_norm_sq);
            if (infeas_trial < best_infeas) {
              best_infeas = infeas_trial;
              best_alpha = alpha;
              best_x = x_trial;
            }
            if (infeas_trial < (1.0 - 5e-4 * alpha) * infeas_current) {
              accepted = true;
              accepted_via_restoration = true;
              accepted_alpha = alpha;
              accepted_infeas = infeas_trial;
              accepted_dlambda.setZero();
              break;
            }
            alpha *= opt.step_backoff;
          }
          if (!accepted && best_alpha > 0.0 && best_infeas < 0.999 * infeas_current) {
            accepted = true;
            accepted_via_restoration = true;
            accepted_alpha = best_alpha;
            accepted_infeas = best_infeas;
            accepted_dlambda.setZero();
            x_trial = best_x;
          }
        }
      }

      if (!accepted) {
        failure_reason = "globalization failed (no acceptable primal-dual/restoration step)";
        converged = false;
        goto finalize;
      }

      out.profiling.accepted_steps += 1;
      x = x_trial;
      for (int i = 0; i < idx.nvar; ++i) {
        const double width = upper[i] - lower[i];
        const double frac = constrained_var_col[static_cast<size_t>(i)] ? 1e-3 : 1e-8;
        const double eps = std::max(1e-10, frac * width);
        x[i] = std::clamp(x[i], lower[i] + eps, upper[i] - eps);
      }
      (void)accepted_alpha;
      lambda.setZero();

      if (!(accepted_infeas < 0.99 * infeas_current)) {
        stagnation_count += 1;
      } else {
        stagnation_count = 0;
      }
      if (accepted_via_restoration) {
        merit_penalty_dynamic = std::min(1e8, merit_penalty_dynamic * 1.2);
        regularization_dynamic = std::min(1.0, std::max(regularization_dynamic, opt.regularization) * 1.5);
        objective_weight_dynamic = std::max(1e-3, objective_weight_dynamic * 0.9);
      } else if (stagnation_count >= 4) {
        merit_penalty_dynamic = std::min(1e8, merit_penalty_dynamic * 2.0);
        regularization_dynamic = std::min(1.0, std::max(regularization_dynamic, opt.regularization) * 5.0);
        objective_weight_dynamic = std::max(1e-3, objective_weight_dynamic * 0.8);
        stagnation_count = 0;
      } else if (accepted_infeas < 0.8 * infeas_current) {
        merit_penalty_dynamic = std::max(opt.merit_penalty, merit_penalty_dynamic * 0.95);
      }
      if (accepted_infeas < 1e-2) {
        objective_weight_dynamic = std::min(1.0, objective_weight_dynamic * 1.15);
      }
    }

    if (converged) {
      break;
    }
    if (!inner_converged_at_mu && out.iterations >= opt.max_inner_iterations * opt.max_outer_iterations) {
      break;
    }
    const bool near_feasible_outer =
        out.max_constraint_violation <= std::max(1e-4, 50.0 * opt.feasibility_tol);
    if (inner_converged_at_mu || near_feasible_outer) {
      mu = std::max(mu * opt.barrier_mu_reduction, opt.barrier_mu_min);
    } else {
      merit_penalty_dynamic = std::min(1e8, merit_penalty_dynamic * 1.5);
      regularization_dynamic = std::min(1.0, std::max(regularization_dynamic, opt.regularization) * 2.0);
    }
    out.profiling.final_barrier_mu = mu;
  }

finalize:
  unpack_state(idx, x, vm, va, vdc, pac, qac, pdc, pg_bus, qg_bus);
  evaluate_hybrid_equalities_and_pf_jacobian(data,
                                             pf_ctx,
                                             pf_pattern,
                                             idx,
                                             vm,
                                             va,
                                             vdc,
                                             pac,
                                             qac,
                                             pdc,
                                             pg_bus,
                                             qg_bus,
                                             opt.ac_eval_threads,
                                             eq_ws,
                                             g_eq);
  update_jacobian_values(data, pf_pattern, idx, vm, vdc, pac, qac, j_ws);

  const double mu_final = std::max(mu, opt.barrier_mu_min);
  accumulate_objective_grad_hdiag(data, idx, x, grad, hdiag);
  grad *= objective_weight_dynamic;
  hdiag *= objective_weight_dynamic;
  double var_barrier_final = 0.0;
  if (is_strictly_inside(x, lower, upper)) {
    accumulate_log_barrier(x, lower, upper, mu_final, grad, hdiag, var_barrier_final);
  }
  (void)var_barrier_final;
  const NonlinearConstraintMetrics nl_final =
      accumulate_nonlinear_limit_terms(idx, nl_ws, vm, va, pac, qac, 100.0 * merit_penalty_dynamic, grad, hdiag);
  if (idx.np > 0) {
    max_eq_p_final = g_eq.segment(idx.i_eq_p, idx.np).cwiseAbs().maxCoeff();
  }
  if (idx.nq > 0) {
    max_eq_q_final = g_eq.segment(idx.i_eq_q, idx.nq).cwiseAbs().maxCoeff();
  }
  if (idx.n_vdc > 0) {
    max_eq_dc_final = g_eq.segment(idx.i_eq_dc, idx.n_vdc).cwiseAbs().maxCoeff();
  }
  if (idx.nc > 0) {
    max_eq_conv_final = g_eq.segment(idx.i_eq_conv, idx.nc).cwiseAbs().maxCoeff();
  }
  max_ineq_final = std::max(0.0, nl_final.max_h);
  out.max_constraint_violation = std::max(inf_norm(g_eq), std::max(0.0, nl_final.max_h));
  Eigen::VectorXd stationarity_vec = grad;
  out.max_stationarity = inf_norm(stationarity_vec);
  if (out.max_constraint_violation <= 1e-3) {
    const double ls_stationarity = dual_least_squares_stationarity(j_ws.matrix, grad, lambda_ls);
    if (ls_stationarity < out.max_stationarity) {
      stationarity_vec = grad + j_ws.matrix.transpose() * lambda_ls;
      out.max_stationarity = inf_norm(stationarity_vec);
    }
  }
  Eigen::Index worst_stationarity_idx = 0;
  const double worst_stationarity_val =
      (stationarity_vec.size() > 0) ? stationarity_vec.cwiseAbs().maxCoeff(&worst_stationarity_idx) : 0.0;
  out.objective = evaluate_objective(data, idx, x);
  out.converged = converged;

  out.vm.assign(vm.data(), vm.data() + vm.size());
  out.va.assign(va.data(), va.data() + va.size());
  if (vdc.size() > 0) {
    out.vdc.assign(vdc.data(), vdc.data() + vdc.size());
  }
  out.pg_mw.assign(data.generators.size(), 0.0);
  out.qg_mvar.assign(data.generators.size(), 0.0);

  for (size_t gi = 0; gi < data.generators.size(); ++gi) {
    const int k = idx.gen_index_to_var[gi];
    if (k >= 0) {
      out.pg_mw[gi] = x[idx.i_pg + k] * data.base_mva;
      out.qg_mvar[gi] = x[idx.i_qg + k] * data.base_mva;
    } else {
      out.pg_mw[gi] = data.generators[gi].pg_mw;
      out.qg_mvar[gi] = data.generators[gi].qg_mvar;
    }
  }

  if (!out.converged && opt.allow_fallback && !dropped_dc) {
    ACOPFResult fallback = out;
    if (try_dispatch_pf_fallback(ac_only, data, idx, fallback)) {
      fallback.iterations = out.iterations;
      fallback.outer_iterations = out.outer_iterations;
      fallback.profiling = out.profiling;
      fallback.status = "converged (economic-dispatch + AC PF fallback; primal-dual path did not converge: " +
                        failure_reason + ")";
      return fallback;
    }
  } else if (!out.converged && opt.allow_fallback && dropped_dc) {
    append_hybrid_fallback_suppression();
  }

  if (out.converged) {
    out.status = "converged";
  } else {
    out.status = "not converged: " + failure_reason + " [max|P|=" + std::to_string(max_eq_p_final) +
                 ", max|Q|=" + std::to_string(max_eq_q_final) + ", max|DC|=" + std::to_string(max_eq_dc_final) +
                 ", max|Conv|=" + std::to_string(max_eq_conv_final) + ", max(h+)=" + std::to_string(max_ineq_final) +
                 ", worst_stat=" + variable_name(idx, static_cast<int>(worst_stationarity_idx)) + ":" +
                 std::to_string(worst_stationarity_val) + "]";
  }

  return out;
}

ACOPFJacobianDiagnostics check_ac_opf_jacobian_fd(const HybridPowerSystem& sys,
                                                  int max_columns,
                                                  double fd_eps,
                                                  int ac_eval_threads) {
  ACOPFJacobianDiagnostics out;
  if (fd_eps <= 0.0) {
    fd_eps = 1e-6;
  }

  core::SolverData data = core::make_solver_data(sys, LossModelType::Linear);
  ACOPFIndex idx = build_index(data);
  if (idx.nb == 0 || idx.nvar == 0 || idx.neq == 0) {
    out.status = "degenerate model dimensions";
    return out;
  }

  core::JacobianContext pf_ctx = build_pf_context(idx);
  core::JacobianPattern pf_pattern = core::build_jacobian_pattern(data, pf_ctx);
  JWorkspace j_ws = build_jacobian_workspace(data, pf_pattern, idx);

  Eigen::VectorXd x;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  initialize_bounds_and_state(data, idx, x, lower, upper);
  if (!is_strictly_inside(x, lower, upper)) {
    out.status = "failed to initialize interior point";
    return out;
  }

  Eigen::VectorXd vm;
  Eigen::VectorXd va;
  Eigen::VectorXd vdc;
  Eigen::VectorXd pac;
  Eigen::VectorXd qac;
  Eigen::VectorXd pdc;
  Eigen::VectorXd pg_bus;
  Eigen::VectorXd qg_bus;
  EqualityEvalWorkspace eq_ws;
  Eigen::VectorXd g_base;
  unpack_state(idx, x, vm, va, vdc, pac, qac, pdc, pg_bus, qg_bus);
  evaluate_hybrid_equalities_and_pf_jacobian(data,
                                             pf_ctx,
                                             pf_pattern,
                                             idx,
                                             vm,
                                             va,
                                             vdc,
                                             pac,
                                             qac,
                                             pdc,
                                             pg_bus,
                                             qg_bus,
                                             ac_eval_threads,
                                             eq_ws,
                                             g_base);
  update_jacobian_values(data, pf_pattern, idx, vm, vdc, pac, qac, j_ws);

  std::vector<int> columns;
  columns.reserve(static_cast<size_t>(idx.nvar));
  if (max_columns <= 0 || max_columns >= idx.nvar) {
    for (int c = 0; c < idx.nvar; ++c) {
      columns.push_back(c);
    }
  } else {
    const int nprobe = std::max(1, max_columns);
    int prev = -1;
    for (int k = 0; k < nprobe; ++k) {
      int c = 0;
      if (nprobe > 1) {
        c = static_cast<int>(std::llround((static_cast<double>(k) * (idx.nvar - 1)) /
                                          static_cast<double>(nprobe - 1)));
      }
      if (c != prev) {
        columns.push_back(c);
        prev = c;
      }
    }
  }

  Eigen::VectorXd x_plus = x;
  Eigen::VectorXd x_minus = x;
  Eigen::VectorXd g_plus;
  Eigen::VectorXd g_minus;
  EqualityEvalWorkspace eq_plus_ws;
  EqualityEvalWorkspace eq_minus_ws;
  Eigen::VectorXd vm_plus;
  Eigen::VectorXd va_plus;
  Eigen::VectorXd vdc_plus;
  Eigen::VectorXd pac_plus;
  Eigen::VectorXd qac_plus;
  Eigen::VectorXd pdc_plus;
  Eigen::VectorXd pg_bus_plus;
  Eigen::VectorXd qg_bus_plus;
  Eigen::VectorXd vm_minus;
  Eigen::VectorXd va_minus;
  Eigen::VectorXd vdc_minus;
  Eigen::VectorXd pac_minus;
  Eigen::VectorXd qac_minus;
  Eigen::VectorXd pdc_minus;
  Eigen::VectorXd pg_bus_minus;
  Eigen::VectorXd qg_bus_minus;
  Eigen::VectorXd col_analytic = Eigen::VectorXd::Zero(idx.neq);

  out.max_abs_error = 0.0;
  out.max_rel_error = 0.0;
  out.worst_row = -1;
  out.worst_col = -1;

  for (int col : columns) {
    const double base = x[col];
    const double interior = 0.49 * std::min(base - lower[col], upper[col] - base);
    if (!(interior > 0.0)) {
      continue;
    }
    const double h = std::min(fd_eps * std::max(1.0, std::abs(base)), interior);
    if (!(h > 0.0)) {
      continue;
    }

    x_plus = x;
    x_minus = x;
    x_plus[col] = base + h;
    x_minus[col] = base - h;

    unpack_state(idx,
                 x_plus,
                 vm_plus,
                 va_plus,
                 vdc_plus,
                 pac_plus,
                 qac_plus,
                 pdc_plus,
                 pg_bus_plus,
                 qg_bus_plus);
    evaluate_hybrid_equalities_only(data,
                                    pf_ctx,
                                    pf_pattern,
                                    idx,
                                    vm_plus,
                                    va_plus,
                                    vdc_plus,
                                    pac_plus,
                                    qac_plus,
                                    pdc_plus,
                                    pg_bus_plus,
                                    qg_bus_plus,
                                    ac_eval_threads,
                                    eq_plus_ws,
                                    g_plus);

    unpack_state(idx,
                 x_minus,
                 vm_minus,
                 va_minus,
                 vdc_minus,
                 pac_minus,
                 qac_minus,
                 pdc_minus,
                 pg_bus_minus,
                 qg_bus_minus);
    evaluate_hybrid_equalities_only(data,
                                    pf_ctx,
                                    pf_pattern,
                                    idx,
                                    vm_minus,
                                    va_minus,
                                    vdc_minus,
                                    pac_minus,
                                    qac_minus,
                                    pdc_minus,
                                    pg_bus_minus,
                                    qg_bus_minus,
                                    ac_eval_threads,
                                    eq_minus_ws,
                                    g_minus);

    col_analytic.setZero();
    for (Eigen::SparseMatrix<double>::InnerIterator it(j_ws.matrix, col); it; ++it) {
      col_analytic[it.row()] = it.value();
    }

    const Eigen::VectorXd col_fd = (g_plus - g_minus) / (2.0 * h);
    for (int row = 0; row < idx.neq; ++row) {
      const double abs_err = std::abs(col_analytic[row] - col_fd[row]);
      const double rel_err =
          abs_err / (1.0 + std::max(std::abs(col_analytic[row]), std::abs(col_fd[row])));
      if (abs_err > out.max_abs_error) {
        out.max_abs_error = abs_err;
        out.max_rel_error = rel_err;
        out.worst_row = row;
        out.worst_col = col;
      }
    }
  }

  out.ok = (out.max_abs_error <= 5e-4) || (out.max_rel_error <= 5e-3);
  out.status = out.ok ? "ok" : "mismatch detected";
  return out;
}

}  // namespace hacdcpf::opf
