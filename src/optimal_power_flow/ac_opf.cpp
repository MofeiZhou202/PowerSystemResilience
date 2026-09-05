#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/power_flow/ac.hpp"
#include "hacdcpf/power_flow/hybrid.hpp"
#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/detail/core_compat.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/native_ipm_solver.hpp"
#include "hacdcpf/validation/validate_system.hpp"

#ifdef HACDCPF_HAVE_IPOPT
#include "hacdcpf/engine/engine.hpp"  // hacdcpf::engine::NLPModel, IpoptAdapter
#endif

namespace hacdcpf::opf {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMinInteriorWidth = 1e-6;
constexpr double kHugeBound = 1e4;

struct PreparedSessionState {
  ACOPFOptions options;
  std::optional<parity::Problem> problem;
  std::optional<parity::ParityOptions> formulation_options;
  std::string system_snapshot;
  std::uint64_t layout_signature{0};
  parity::IPMPreparedState ipm_state;
  ACOPFResult previous_result;
  bool has_previous_result{false};
};

bool same_formulation_options(const parity::ParityOptions& lhs,
                              const parity::ParityOptions& rhs) {
  return lhs.load_shedding == rhs.load_shedding && lhs.voll == rhs.voll &&
         lhs.eps_iac == rhs.eps_iac && lhs.objective == rhs.objective &&
         lhs.voltage_target_pu == rhs.voltage_target_pu &&
         lhs.voltage_deviation_weight == rhs.voltage_deviation_weight &&
         lhs.active_loss_weight == rhs.active_loss_weight &&
         lhs.enforce_branch_limits == rhs.enforce_branch_limits &&
         lhs.enforce_converter_capacity == rhs.enforce_converter_capacity &&
         lhs.enforce_converter_current_limits ==
             rhs.enforce_converter_current_limits &&
         lhs.enforce_converter_modulation_limits ==
             rhs.enforce_converter_modulation_limits;
}

void unproject_per_bus_ac_opf_result(ACOPFResult& out, const BusMergeMap& map) {
  if (map.ext_to_int.empty() || map.n_original <= 0) {
    return;
  }
  out.vm = unproject_bus_vector(out.vm, map, BusVectorSemantics::Intensive);
  out.va = unproject_bus_vector(out.va, map, BusVectorSemantics::Intensive);
  if (!out.dpd_mw.empty()) {
    out.dpd_mw = unproject_bus_vector(
        out.dpd_mw, map, BusVectorSemantics::Extensive);
  }
  if (!out.dqd_mvar.empty()) {
    out.dqd_mvar = unproject_bus_vector(
        out.dqd_mvar, map, BusVectorSemantics::Extensive);
  }
  if (!out.lmp_p.empty()) {
    out.lmp_p = unproject_bus_vector(
        out.lmp_p, map, BusVectorSemantics::Intensive);
  }
  if (!out.lmp_q.empty()) {
    out.lmp_q = unproject_bus_vector(
        out.lmp_q, map, BusVectorSemantics::Intensive);
  }
}

void trim_internal_dc_bus_results(
    ACOPFResult& out,
    const std::optional<ProjectionCertificate>& certificate) {
  if (!certificate || certificate->n_authored_dc_buses < 0) return;
  // Recover DC voltages stripped as dead islands to authored positions (0 pu)
  // before dropping ER-internal buses (R-01/R-02).
  if (certificate->has_dc_strip())
    out.vdc = unproject_dc_bus_vector(out.vdc, *certificate);
  const size_t authored =
      static_cast<size_t>(certificate->n_authored_dc_buses);
  if (out.vdc.size() > authored) out.vdc.resize(authored);
}

void set_generator_result_map(ACOPFResult& out,
                              const powerflow::SolverData& data) {
  out.gen_map.resize(data.generators.size());
  for (size_t i = 0; i < data.generators.size(); ++i) {
    out.gen_map[i].original_index = data.generators[i].index;
    out.gen_map[i].source_type = 0;
  }
}

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
  if (!std::isfinite(x)) {
    x = 0.0;
  }
  if (!std::isfinite(lo) || !std::isfinite(hi)) {
    if (std::isfinite(lo) && x < lo) return lo;
    if (std::isfinite(hi) && x > hi) return hi;
    return x;
  }
  if (lo > hi) {
    std::swap(lo, hi);
  }
  const double width = hi - lo;
  if (!(width > 0.0)) {
    return 0.5 * (lo + hi);
  }
  const double eps = std::min(std::max(1e-8, 1e-3 * width), 0.49 * width);
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
  double shift_rad{0.0};
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
    e.shift_rad = br.shift_deg * kDegToRad;
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
    // Match the complex off-nominal tap used by Ybus/branch-flow assembly:
    // tap = |t| exp(j*shift), hence the from-end angle is θi-θj-shift.
    const double theta = va[i] - va[j] - br.shift_rad;
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
    set_generator_result_map(out, pf_data);

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
    if (pf_data.bus_merge_map) {
      unproject_per_bus_ac_opf_result(out, *pf_data.bus_merge_map);
    }
    trim_internal_dc_bus_results(out, pf_data.projection_certificate);
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

// Map a previous public ACOPFResult into the current compact parity variable
// layout.  Start from the normal physics-informed point so a missing or
// incompatible component family degrades gracefully instead of rejecting the
// whole warm start.
Eigen::VectorXd build_parity_primal_warm_start(const parity::Problem& prob,
                                               const ACOPFResult& warm,
                                               bool& mapped_any) {
  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  Eigen::VectorXd x0;
  parity::build_variable_bounds(prob, xmin, xmax);
  parity::build_initial_point(prob, xmin, xmax, x0);
  const auto& idx = prob.vidx;
  const double base = prob.data.base_mva;
  mapped_any = false;

  if (warm.ipm_primal_state.size() == static_cast<size_t>(idx.n_total)) {
    Eigen::Map<const Eigen::VectorXd> raw(warm.ipm_primal_state.data(),
                                          idx.n_total);
    if (raw.allFinite()) {
      // Clamp into the bounds interior: warm/extrapolated states (e.g.
      // Davidenko-predicted points) can overshoot variable bounds, which
      // otherwise start the IPM with floored slacks on those rows.
      Eigen::VectorXd clipped = raw;
      for (int i = 0; i < idx.n_total; ++i) {
        if (std::isfinite(xmin[i])) clipped[i] = std::max(clipped[i], xmin[i] + 1e-8);
        if (std::isfinite(xmax[i])) clipped[i] = std::min(clipped[i], xmax[i] - 1e-8);
      }
      mapped_any = true;
      return clipped;
    }
  }

  const auto assign_block = [&](int offset, int count,
                                const std::vector<double>& values,
                                double divisor = 1.0) {
    if (count <= 0 || values.size() != static_cast<size_t>(count)) return;
    for (int k = 0; k < count; ++k) {
      const double value = values[static_cast<size_t>(k)] / divisor;
      if (std::isfinite(value)) x0[offset + k] = value;
    }
    mapped_any = true;
  };

  assign_block(idx.i_vm, idx.n_vm, warm.vm);
  assign_block(idx.i_va, idx.n_va, warm.va);
  assign_block(idx.i_vdc, idx.n_vdc, warm.vdc);
  assign_block(idx.i_pac, idx.n_pac, warm.pac_mw, base);
  assign_block(idx.i_qac, idx.n_qac, warm.qac_mvar, base);
  if (idx.n_pdc == idx.n_pac &&
      warm.pac_mw.size() == static_cast<size_t>(idx.n_pac) &&
      warm.qac_mvar.size() == static_cast<size_t>(idx.n_qac)) {
    for (int k = 0; k < idx.n_pac; ++k) {
      const auto& conv = prob.data.converters[static_cast<size_t>(
          prob.conv_var_to_data[static_cast<size_t>(k)])];
      const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
      const double vm = (warm.vm.size() == static_cast<size_t>(idx.n_vm))
                            ? std::max(warm.vm[static_cast<size_t>(ac)], 1e-8)
                            : std::max(x0[idx.i_vm + ac], 1e-8);
      const double p = warm.pac_mw[static_cast<size_t>(k)] / base;
      const double q = warm.qac_mvar[static_cast<size_t>(k)] / base;
      const double iac = std::sqrt(p * p + q * q +
                                  std::max(prob.options.eps_iac, 1e-12)) / vm;
      const double loss = conv.loss_mw / base +
                          conv.loss_percent / 100.0 * iac +
                          (1.0 - conv.eta) * iac * iac;
      x0[idx.i_pdc + k] = -(p + loss);
    }
    mapped_any = true;
  }
  assign_block(idx.i_dpd, idx.n_dpd, warm.dpd_mw, base);
  assign_block(idx.i_dqd, idx.n_dqd, warm.dqd_mvar, base);
  assign_block(idx.i_pren, idx.n_pren, warm.pren_mw, base);
  assign_block(idx.i_qren, idx.n_qren, warm.qren_mvar, base);
  assign_block(idx.i_pdcdc, idx.n_pdcdc, warm.pdcdc_mw, base);
  assign_block(idx.i_pflex, idx.n_pflex, warm.pflex_mw, base);
  assign_block(idx.i_erp, idx.n_erp, warm.er_port_p_mw, base);

  // Generator rows in the current problem may include synthetic external-grid
  // generators appended after authored generators.  Map both public result
  // families without relying on unstable compact-row positions.
  for (int k = 0; k < idx.n_pg; ++k) {
    const int data_index = prob.gen_var_to_data[static_cast<size_t>(k)];
    if (data_index >= 0 && data_index < static_cast<int>(warm.pg_mw.size())) {
      if (std::isfinite(warm.pg_mw[static_cast<size_t>(data_index)]))
        x0[idx.i_pg + k] = warm.pg_mw[static_cast<size_t>(data_index)] / base;
      if (data_index < static_cast<int>(warm.qg_mvar.size()) &&
          std::isfinite(warm.qg_mvar[static_cast<size_t>(data_index)]))
        x0[idx.i_qg + k] = warm.qg_mvar[static_cast<size_t>(data_index)] / base;
      mapped_any = true;
      continue;
    }
    const int external = data_index - static_cast<int>(warm.pg_mw.size());
    if (external >= 0 &&
        external < static_cast<int>(warm.external_grid_p_mw.size())) {
      x0[idx.i_pg + k] =
          warm.external_grid_p_mw[static_cast<size_t>(external)] / base;
      if (external < static_cast<int>(warm.external_grid_q_mvar.size()))
        x0[idx.i_qg + k] =
            warm.external_grid_q_mvar[static_cast<size_t>(external)] / base;
      mapped_any = true;
    }
  }

  if (idx.n_pstor > 0 &&
      warm.pstor_mw.size() >= static_cast<size_t>(idx.n_pstor)) {
    for (int k = 0; k < idx.n_pstor; ++k) {
      x0[idx.i_pstor + k] = warm.pstor_mw[static_cast<size_t>(k)] / base;
      if (warm.qstor_mvar.size() > static_cast<size_t>(k))
        x0[idx.i_qstor + k] = warm.qstor_mvar[static_cast<size_t>(k)] / base;
    }
    mapped_any = true;
  }
  const size_t dc_storage_offset = static_cast<size_t>(idx.n_pstor);
  if (idx.n_pstordc > 0 &&
      warm.pstor_mw.size() >= dc_storage_offset +
                                static_cast<size_t>(idx.n_pstordc)) {
    for (int k = 0; k < idx.n_pstordc; ++k)
      x0[idx.i_pstordc + k] =
          warm.pstor_mw[dc_storage_offset + static_cast<size_t>(k)] / base;
    mapped_any = true;
  }
  if (idx.n_erq > 0 &&
      warm.er_port_q_mvar.size() >= static_cast<size_t>(idx.n_erq)) {
    for (const auto& port : prob.er_ports) {
      if (port.qvar >= 0 &&
          static_cast<size_t>(port.pvar) < warm.er_port_q_mvar.size())
        x0[idx.i_erq + port.qvar] =
            warm.er_port_q_mvar[static_cast<size_t>(port.pvar)] / base;
    }
    mapped_any = true;
  }
  return x0;
}

std::vector<double> map_power_flow_ac_voltage_to_parity(
    const parity::Problem& prob, const HybridPowerSystem& sys,
    const std::vector<double>& authored_values, double default_value) {
  const int n = prob.vidx.n_vm;
  std::vector<double> values(static_cast<size_t>(n), default_value);
  if (n == 0 || authored_values.empty()) return values;

  std::unordered_map<int, double> by_bus;
  by_bus.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size() && i < authored_values.size(); ++i)
    by_bus[sys.ac.buses[i].index] = authored_values[i];

  if (prob.data.bus_merge_map &&
      prob.data.bus_merge_map->int_to_ext.size() == static_cast<size_t>(n)) {
    for (int i = 0; i < n; ++i) {
      const int external = prob.data.bus_merge_map->int_to_ext[static_cast<size_t>(i)];
      const auto it = by_bus.find(external);
      if (it != by_bus.end() && std::isfinite(it->second)) values[static_cast<size_t>(i)] = it->second;
    }
    return values;
  }

  for (int i = 0; i < n && i < static_cast<int>(prob.data.ac_buses.size()); ++i) {
    const auto it = by_bus.find(prob.data.ac_buses[static_cast<size_t>(i)].index);
    if (it != by_bus.end() && std::isfinite(it->second)) values[static_cast<size_t>(i)] = it->second;
  }
  return values;
}

std::vector<double> map_power_flow_dc_voltage_to_parity(
    const parity::Problem& prob, const HybridPowerSystem& sys,
    const std::vector<double>& authored_values) {
  const int n = prob.vidx.n_vdc;
  std::vector<double> values(static_cast<size_t>(n), 1.0);
  if (n == 0 || authored_values.empty()) return values;

  std::unordered_map<int, double> by_bus;
  by_bus.reserve(sys.dc.buses.size());
  for (size_t i = 0; i < sys.dc.buses.size() && i < authored_values.size(); ++i)
    by_bus[sys.dc.buses[i].index] = authored_values[i];
  for (int i = 0; i < n && i < static_cast<int>(prob.data.dc_buses.size()); ++i) {
    const auto it = by_bus.find(prob.data.dc_buses[static_cast<size_t>(i)].index);
    if (it != by_bus.end() && std::isfinite(it->second)) values[static_cast<size_t>(i)] = it->second;
  }
  return values;
}

void recover_power_flow_generator_dispatch(const parity::Problem& prob,
                                           ACOPFResult& warm) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const double base = prob.data.base_mva;
  if (!(base > 0.0) || idx.n_pg == 0 ||
      warm.vm.size() != static_cast<size_t>(idx.n_vm) ||
      warm.va.size() != static_cast<size_t>(idx.n_va) ||
      warm.vdc.size() != static_cast<size_t>(idx.n_vdc)) {
    return;
  }

  // Re-evaluate the same nodal balances used by Parity at the converged PF
  // voltage state. PowerFlowResult intentionally carries voltages and device
  // transfers rather than mutable generator rows, so the balance residual is
  // the authoritative way to recover the slack/PV dispatch.
  Eigen::VectorXd x = Eigen::VectorXd::Zero(idx.n_total);
  for (int i = 0; i < idx.n_va; ++i)
    x[idx.i_va + i] = warm.va[static_cast<size_t>(i)];
  for (int i = 0; i < idx.n_vm; ++i)
    x[idx.i_vm + i] = warm.vm[static_cast<size_t>(i)];
  for (int i = 0; i < idx.n_vdc; ++i)
    x[idx.i_vdc + i] = warm.vdc[static_cast<size_t>(i)];

  for (int k = 0; k < idx.n_pg; ++k) {
    const int data_index = prob.gen_var_to_data[static_cast<size_t>(k)];
    if (data_index < 0 ||
        data_index >= static_cast<int>(prob.data.generators.size()))
      continue;
    const auto& gen = prob.data.generators[static_cast<size_t>(data_index)];
    const double pg = data_index < static_cast<int>(warm.pg_mw.size())
                          ? warm.pg_mw[static_cast<size_t>(data_index)]
                          : gen.pg_mw;
    const double qg = data_index < static_cast<int>(warm.qg_mvar.size())
                          ? warm.qg_mvar[static_cast<size_t>(data_index)]
                          : gen.qg_mvar;
    x[idx.i_pg + k] = std::isfinite(pg) ? pg / base : 0.0;
    x[idx.i_qg + k] = std::isfinite(qg) ? qg / base : 0.0;
  }
  for (int k = 0; k < idx.n_pac; ++k) {
    if (k < static_cast<int>(warm.pac_mw.size()))
      x[idx.i_pac + k] = warm.pac_mw[static_cast<size_t>(k)] / base;
    if (k < static_cast<int>(warm.qac_mvar.size()))
      x[idx.i_qac + k] = warm.qac_mvar[static_cast<size_t>(k)] / base;
  }

  // Seed every other AC-side decision component at its PF operating value.
  // Load-shedding variables deliberately remain zero: PF did not shed load.
  for (int k = 0; k < idx.n_pren; ++k) {
    const int data_index = prob.ren_var_to_data[static_cast<size_t>(k)];
    if (prob.ren_source[static_cast<size_t>(k)] == 0) {
      const auto& renewable =
          prob.data.renewable_gens[static_cast<size_t>(data_index)];
      x[idx.i_pren + k] = renewable.p_mw / base;
      x[idx.i_qren + k] = renewable.q_mvar / base;
    } else {
      const auto& pv = prob.data.pv_systems[static_cast<size_t>(data_index)];
      x[idx.i_pren + k] = pv.p_mw / base;
      x[idx.i_qren + k] = pv.q_mvar / base;
    }
  }
  for (int k = 0; k < idx.n_pstor; ++k) {
    const auto& storage = prob.data.storage_units[static_cast<size_t>(
        prob.stor_var_to_data[static_cast<size_t>(k)])];
    x[idx.i_pstor + k] = storage.p_mw / base;
    x[idx.i_qstor + k] = storage.q_mvar / base;
  }
  for (int k = 0; k < idx.n_pflex; ++k) {
    const auto& load = prob.data.flexible_loads[static_cast<size_t>(
        prob.flex_var_to_data[static_cast<size_t>(k)])];
    x[idx.i_pflex + k] = load.p_mw / base;
  }
  for (const auto& port : prob.er_ports) {
    const auto& authored =
        prob.data.energy_routers[static_cast<size_t>(port.router_idx)]
            .ports[static_cast<size_t>(port.port_idx)];
    x[idx.i_erp + port.pvar] = authored.p_mw / base;
    if (port.qvar >= 0)
      x[idx.i_erq + port.qvar] = authored.q_mvar / base;
  }

  parity::EvalWorkspace ws;
  Eigen::VectorXd residual;
  parity::equality_constraints(prob, x, ws, residual);
  if (!residual.allFinite() ||
      residual.size() != prob.cidx.n_eq_total) {
    return;
  }

  std::vector<std::vector<int>> generators_by_bus(
      static_cast<size_t>(idx.n_vm));
  for (int k = 0; k < idx.n_pg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    if (bus >= 0 && bus < idx.n_vm)
      generators_by_bus[static_cast<size_t>(bus)].push_back(k);
  }

  auto distribute_at_bus = [&](const std::vector<int>& compact_rows,
                               double target_mw, bool reactive) {
    if (compact_rows.empty() || !std::isfinite(target_mw)) return;
    auto& values = reactive ? warm.qg_mvar : warm.pg_mw;
    std::vector<int> order = compact_rows;
    std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      const int li = prob.gen_var_to_data[static_cast<size_t>(lhs)];
      const int ri = prob.gen_var_to_data[static_cast<size_t>(rhs)];
      return prob.data.generators[static_cast<size_t>(li)].is_slack &&
             !prob.data.generators[static_cast<size_t>(ri)].is_slack;
    });

    double assigned = 0.0;
    for (const int k : order) {
      const int data_index = prob.gen_var_to_data[static_cast<size_t>(k)];
      const auto& gen = prob.data.generators[static_cast<size_t>(data_index)];
      double lo = reactive ? gen.qmin_mvar : gen.pmin_mw;
      double hi = reactive ? gen.qmax_mvar : gen.pmax_mw;
      if (lo > hi) std::swap(lo, hi);
      double value = values[static_cast<size_t>(data_index)];
      if (!std::isfinite(value)) value = 0.0;
      value = std::clamp(value, lo, hi);
      values[static_cast<size_t>(data_index)] = value;
      assigned += value;
    }

    double remaining = target_mw - assigned;
    for (const int k : order) {
      if (std::abs(remaining) <= 1e-9) break;
      const int data_index = prob.gen_var_to_data[static_cast<size_t>(k)];
      const auto& gen = prob.data.generators[static_cast<size_t>(data_index)];
      double lo = reactive ? gen.qmin_mvar : gen.pmin_mw;
      double hi = reactive ? gen.qmax_mvar : gen.pmax_mw;
      if (lo > hi) std::swap(lo, hi);
      double& value = values[static_cast<size_t>(data_index)];
      if (remaining > 0.0) {
        const double step = std::min(remaining, hi - value);
        value += step;
        remaining -= step;
      } else {
        const double step = std::min(-remaining, value - lo);
        value -= step;
        remaining += step;
      }
    }
  };

  const double p_scale = std::max(std::abs(prob.scale_p), 1e-12);
  const double q_scale = std::max(std::abs(prob.scale_q), 1e-12);
  for (int bus = 0; bus < idx.n_vm; ++bus) {
    const auto& rows = generators_by_bus[static_cast<size_t>(bus)];
    if (rows.empty()) continue;
    double pg_current_pu = 0.0;
    double qg_current_pu = 0.0;
    for (const int k : rows) {
      pg_current_pu += x[idx.i_pg + k];
      qg_current_pu += x[idx.i_qg + k];
    }
    const double pg_target_mw =
        (pg_current_pu + residual[cidx.i_pbal_ac + bus] / p_scale) *
        base;
    const double qg_target_mvar =
        (qg_current_pu + residual[cidx.i_qbal_ac + bus] / q_scale) *
        base;
    distribute_at_bus(rows, pg_target_mw, false);
    distribute_at_bus(rows, qg_target_mvar, true);
  }
}

ACOPFResult build_hybrid_power_flow_warm_start(
    const parity::Problem& prob, const HybridPowerSystem& sys,
    const PowerFlowResult& pf) {
  ACOPFResult warm;
  warm.vm = map_power_flow_ac_voltage_to_parity(prob, sys, pf.vm, 1.0);
  warm.va = map_power_flow_ac_voltage_to_parity(prob, sys, pf.va, 0.0);
  warm.vdc = map_power_flow_dc_voltage_to_parity(prob, sys, pf.vdc);
  std::unordered_map<int, const Generator*> authored_generators;
  authored_generators.reserve(sys.ac.generators.size());
  for (const auto& gen : sys.ac.generators)
    authored_generators.emplace(gen.index, &gen);
  warm.pg_mw.reserve(prob.data.generators.size());
  warm.qg_mvar.reserve(prob.data.generators.size());
  for (const auto& gen : prob.data.generators) {
    const auto authored = authored_generators.find(gen.index);
    const Generator& seed = authored == authored_generators.end()
        ? gen
        : *authored->second;
    warm.pg_mw.push_back(seed.pg_mw);
    warm.qg_mvar.push_back(seed.qg_mvar);
  }

  const auto find_vsc = [&](int index) -> const VSCTransfer* {
    for (const auto& transfer : pf.vsc_transfers)
      if (transfer.index == index) return &transfer;
    return nullptr;
  };
  warm.pac_mw.assign(static_cast<size_t>(prob.vidx.n_pac), 0.0);
  warm.qac_mvar.assign(static_cast<size_t>(prob.vidx.n_qac), 0.0);
  for (int k = 0; k < prob.vidx.n_pac; ++k) {
    const int data_index = prob.conv_var_to_data[static_cast<size_t>(k)];
    const auto* transfer = find_vsc(prob.data.converters[static_cast<size_t>(data_index)].index);
    if (transfer == nullptr) continue;
    warm.pac_mw[static_cast<size_t>(k)] = transfer->p_ac_mw;
    warm.qac_mvar[static_cast<size_t>(k)] = transfer->q_ac_mvar;
  }

  const auto find_dcdc = [&](int index) -> const DCDCTransfer* {
    for (const auto& transfer : pf.dcdc_transfers)
      if (transfer.index == index) return &transfer;
    return nullptr;
  };
  warm.pdcdc_mw.assign(static_cast<size_t>(prob.vidx.n_pdcdc), 0.0);
  for (int k = 0; k < prob.vidx.n_pdcdc; ++k) {
    const int data_index = prob.dcdc_var_to_data[static_cast<size_t>(k)];
    const auto* transfer = find_dcdc(prob.data.dcdc_converters[static_cast<size_t>(data_index)].index);
    if (transfer != nullptr) warm.pdcdc_mw[static_cast<size_t>(k)] = transfer->p_in_mw;
  }
  recover_power_flow_generator_dispatch(prob, warm);
  return warm;
}

// Reuse the immutable Parity topology/formulation while advancing only the
// authored operating point that build_initial_point and result fallback read.
// DC Phase I changes no bounds, costs, admittances, or compact index maps, so
// rebuilding Problem would duplicate O(n + nnz) storage without changing the
// NLP. This update is equivalent to the pg/qg/vm/va seed fields of a rebuild.
void update_parity_operating_point(parity::Problem& prob,
                                   const HybridPowerSystem& sys) {
  std::unordered_map<int, const ACBus*> buses;
  buses.reserve(sys.ac.buses.size());
  for (const auto& bus : sys.ac.buses) buses.emplace(bus.index, &bus);
  for (auto& bus : prob.data.ac_buses) {
    const auto authored = buses.find(bus.index);
    if (authored == buses.end()) continue;
    bus.vm_pu = authored->second->vm_pu;
    bus.va_deg = authored->second->va_deg;
  }

  std::unordered_map<int, const Generator*> generators;
  generators.reserve(sys.ac.generators.size());
  for (const auto& gen : sys.ac.generators)
    generators.emplace(gen.index, &gen);
  for (auto& gen : prob.data.generators) {
    const auto authored = generators.find(gen.index);
    if (authored == generators.end()) continue;
    gen.pg_mw = authored->second->pg_mw;
    gen.qg_mvar = authored->second->qg_mvar;
  }
}

#ifdef HACDCPF_HAVE_IPOPT
struct ParityNLPInput {
  engine::NLPModel model;
  Eigen::VectorXd x0;
  bool warm_start_used{false};
  double initial_primal_inf{0.0};
};

ParityNLPInput build_parity_nlp_input(const parity::Problem& prob,
                                      const parity::IPMOptions& ipm_opt) {
  ParityNLPInput input;
  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  parity::build_variable_bounds(prob, xmin, xmax);
  if (ipm_opt.primal_start != nullptr &&
      ipm_opt.primal_start->size() == prob.vidx.n_total &&
      ipm_opt.primal_start->allFinite()) {
    input.x0 = *ipm_opt.primal_start;
    for (int i = 0; i < input.x0.size(); ++i) {
      if (std::isfinite(xmin[i])) {
        input.x0[i] = std::max(input.x0[i], xmin[i] + 1e-8);
      }
      if (std::isfinite(xmax[i])) {
        input.x0[i] = std::min(input.x0[i], xmax[i] - 1e-8);
      }
    }
    input.warm_start_used = true;
  } else {
    parity::build_initial_point(prob, xmin, xmax, input.x0);
  }

  {
    parity::EvalWorkspace ws0;
    Eigen::VectorXd g0;
    Eigen::VectorXd h0;
    parity::equality_constraints(prob, input.x0, ws0, g0);
    parity::nonlinear_inequality_constraints(prob, input.x0, h0);
    const double eq0 = g0.size() ? g0.cwiseAbs().maxCoeff() : 0.0;
    const double ineq0 = h0.size() ? std::max(0.0, h0.maxCoeff()) : 0.0;
    input.initial_primal_inf = std::max(eq0, ineq0);
  }

  const int n = prob.vidx.n_total;
  auto& nlp = input.model;
  nlp.sense = engine::Sense::Minimize;
  nlp.vars.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    engine::VariableMeta vm;
    vm.type = engine::VarType::Continuous;
    vm.lb = std::isfinite(xmin[i]) ? xmin[i] : -1e20;
    vm.ub = std::isfinite(xmax[i]) ? xmax[i] : 1e20;
    nlp.vars[static_cast<size_t>(i)] = vm;
  }
  nlp.x0 = input.x0;
  nlp.solver_options.max_iterations = std::max(1, ipm_opt.max_iter);
  nlp.solver_options.tolerance = std::min(
      {ipm_opt.tol_primal, ipm_opt.tol_dual,
       ipm_opt.tol_complementarity});
  nlp.solver_options.acceptable_tolerance = std::max(
      {ipm_opt.tol_primal, ipm_opt.tol_dual,
       ipm_opt.tol_complementarity});
  const double backend_primal_tol = std::min(
      ipm_opt.tol_primal, nlp.solver_options.tolerance);
  nlp.solver_options.constraint_violation_tolerance = backend_primal_tol;
  nlp.solver_options.dual_infeasibility_tolerance = ipm_opt.tol_dual;
  nlp.solver_options.complementarity_tolerance = ipm_opt.tol_complementarity;
  nlp.solver_options.acceptable_constraint_violation_tolerance =
      backend_primal_tol;
  nlp.solver_options.acceptable_dual_infeasibility_tolerance =
      100.0 * ipm_opt.tol_dual;
  nlp.solver_options.acceptable_complementarity_tolerance = std::max(
      1e-2, 100.0 * ipm_opt.tol_complementarity);

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
  nlp.jac_g = [&prob](const Eigen::VectorXd& x,
                       Eigen::SparseMatrix<double>& J) {
    parity::EvalWorkspace ws;
    Eigen::VectorXd g_tmp;
    parity::equality_constraints(prob, x, ws, g_tmp);
    parity::equality_jacobian(prob, x, ws, J);
  };
  nlp.h = [&prob](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    parity::nonlinear_inequality_constraints(prob, x, h);
  };
  nlp.jac_h = [&prob](const Eigen::VectorXd& x,
                       Eigen::SparseMatrix<double>& J) {
    parity::nonlinear_inequality_jacobian(prob, x, J);
  };
  // Wächter and Biegler (2006), Sec. 2: Ipopt's limited-memory mode requires
  // consistent first derivatives but does not consume an exact Lagrangian
  // Hessian. Keep hybrid DC/VSC models on that path until every converter and
  // DC-network second-order block has a complete directional-derivative
  // certificate; pure-AC models retain the audited exact Hessian.
  const bool hybrid_problem = prob.vidx.n_vdc > 0 || prob.vidx.n_pac > 0 ||
                              prob.vidx.n_pdcdc > 0;
  if (!hybrid_problem) {
    nlp.lagrangian_hess = [&prob](const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& lambda,
                                  const Eigen::VectorXd* nu,
                                  Eigen::SparseMatrix<double>& H) {
      parity::lagrangian_hessian(prob, x, lambda, nu, H, 0.0);
    };
  }
  return input;
}
#endif

// Solve the assembled parity OPF nonlinear program with the embedded Ipopt
// filter line-search NLP solver.  The parity `Problem` already exposes every
// callback Ipopt needs (objective, gradient, equality/inequality residuals and
// Jacobians, and the Lagrangian Hessian), so the model maps 1:1 onto
// engine::NLPModel.  Box bounds are passed as variable bounds; only the
// nonlinear inequalities (`h(x) <= 0`) become general constraints.
//
// `available` is set false when Ipopt is not compiled in; callers then keep the
// native-IPM result.
parity::IPMResult solve_parity_with_ipopt(const parity::Problem& prob,
                                          const parity::IPMOptions& ipm_opt,
                                          bool& available) {
  parity::IPMResult res;
#ifdef HACDCPF_HAVE_IPOPT
  // Ipopt and MUMPS are built from the pinned MIPSolvers source tree. The path
  // can still be force-disabled for backend isolation and diagnostics.
  if (std::getenv("HACDCPF_DISABLE_IPOPT_OPF") != nullptr) {
    available = false;
    res.converged = false;
    res.status = "Ipopt disabled (HACDCPF_DISABLE_IPOPT_OPF set)";
    return res;
  }
  available = true;

  ParityNLPInput input = build_parity_nlp_input(prob, ipm_opt);
  res.warm_start_used = input.warm_start_used;
  res.initial_primal_inf = input.initial_primal_inf;
  const int n = prob.vidx.n_total;
  engine::IpoptAdapter ipopt;
  const engine::SolveResult sol = ipopt.solve_nlp(input.model);

  res.x = (sol.x.size() == n) ? sol.x : input.x0;
  // Ipopt can return Maximum_Iterations_Exceeded after reaching a highly
  // usable point on large hybrid models.  Keep the termination code in the
  // status, but accept the point when feasibility is strict and the remaining
  // KKT residual is within the same practical acceptable band used by the
  // native IPM's best-iterate certification.
  const double acceptable_scale = 1000.0;
  const double acceptable_dual = acceptable_scale *
      std::max(ipm_opt.tol_dual, 1.0e-12);
  // Ipopt's complementarity includes bound multipliers and can remain at its
  // documented acceptable floor after the physical equality and stationarity
  // residuals have settled.  Keep a bounded engineering floor consistent with
  // the native IPM's relaxed complementarity test.
  const double acceptable_complementarity = std::max(
      1.0e-2, acceptable_scale * std::max(ipm_opt.tol_complementarity, 1.0e-12));
  const bool acceptable_residuals =
      std::isfinite(sol.stats.primal_feas) &&
      std::isfinite(sol.stats.dual_feas) &&
      std::isfinite(sol.stats.complementarity) &&
      sol.stats.primal_feas <= ipm_opt.tol_primal &&
      sol.stats.dual_feas <= acceptable_dual &&
      sol.stats.complementarity <= acceptable_complementarity;
  res.converged = sol.stats.success || acceptable_residuals;
  res.iterations = sol.stats.iterations;
  res.primal_inf = sol.stats.primal_feas;
  res.dual_inf = sol.stats.dual_feas;
  res.complementarity = sol.stats.complementarity;
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  const int meq = prob.cidx.n_eq_total;
  if (sol.constraint_duals.size() == m_nonlin + meq &&
      sol.box_dual_lb.size() == n && sol.box_dual_ub.size() == n) {
    Eigen::VectorXd xmin;
    Eigen::VectorXd xmax;
    parity::build_variable_bounds(prob, xmin, xmax);
    std::vector<int> lb_cols;
    std::vector<int> ub_cols;
    for (int i = 0; i < n; ++i) {
      if (std::isfinite(xmin[i])) lb_cols.push_back(i);
      if (std::isfinite(xmax[i])) ub_cols.push_back(i);
    }
    const int niq = m_nonlin + static_cast<int>(lb_cols.size()) +
                    static_cast<int>(ub_cols.size());
    constexpr double kInteriorFloor = 2e-12;
    res.lambda_eq = sol.constraint_duals.tail(meq);
    res.mu = Eigen::VectorXd::Constant(niq, kInteriorFloor);
    res.z = Eigen::VectorXd::Constant(niq, kInteriorFloor);
    Eigen::VectorXd h_nonlin;
    parity::nonlinear_inequality_constraints(prob, res.x, h_nonlin);
    if (m_nonlin > 0) {
      res.mu.head(m_nonlin) =
          sol.constraint_duals.head(m_nonlin).array().max(kInteriorFloor);
      res.z.head(m_nonlin) =
          (-h_nonlin.array()).max(kInteriorFloor);
    }
    int row = m_nonlin;
    for (int col : lb_cols) {
      res.mu[row] = std::max(sol.box_dual_lb[col], kInteriorFloor);
      res.z[row] = std::max(res.x[col] - xmin[col], kInteriorFloor);
      ++row;
    }
    for (int col : ub_cols) {
      res.mu[row] = std::max(sol.box_dual_ub[col], kInteriorFloor);
      res.z[row] = std::max(xmax[col] - res.x[col], kInteriorFloor);
      ++row;
    }
  }
  res.status = std::string("Ipopt: ") + sol.stats.status;
  if (!sol.stats.success && acceptable_residuals) {
    res.status = "Ipopt: acceptable residuals after " + sol.stats.status;
  }
#else
  (void)prob;
  (void)ipm_opt;
  available = false;
  res.converged = false;
  res.status = "Ipopt not compiled in";
#endif
  return res;
}

ACOPFResult solve_with_parity_ipm(
    const HybridPowerSystem& sys, const ACOPFOptions& opt,
    ParityInnerSolver inner = ParityInnerSolver::Auto,
    PreparedSessionState* prepared = nullptr) {
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
    const bool has_lcc = std::any_of(
        sys.lcc_converters.begin(), sys.lcc_converters.end(),
        [](const LCCConverter& lcc) { return lcc.in_service; });
    if (has_lcc) tag += "+lcc-quasi-steady";
    sc.model_scope = tag;
    sc.validity.vsc_loss_modelled = true;
    sc.validity.vsc_capacity_circle_enforced = opt.enforce_converter_capacity;
    sc.validity.vsc_current_limits_enforced = opt.enforce_converter_current_limits;
    sc.validity.vsc_modulation_limits_enforced = opt.enforce_converter_modulation_limits;
    sc.validity.dcdc_duty_ratio_enforced = opt.enforce_converter_modulation_limits;
    sc.validity.vsc_vdc_control_modelled = true;
    sc.validity.lcc_quasi_steady_modelled = has_lcc;
    if (has_lcc) {
      out.model_limitations.push_back(
          "LCC P/I/alpha/gamma orders and converter-transformer taps are fixed inputs, not economic dispatch variables.");
      out.model_limitations.push_back(
          "LCC rated-current saturation is modelled, while alpha/gamma limits are post-solve audits rather than hard OPF inequalities.");
      out.model_limitations.push_back(
          "LCC commutation-overlap iteration, tap-changer control, and active converter losses are not modelled.");
      out.model_limitations.push_back(
          "LCC equality second derivatives use a local finite difference of the analytical injection Jacobian; the rated-current transition is nonsmooth.");
    }
  }

  parity::ParityOptions form_opt;
  // RPO objectives must not improve voltage/loss by dropping demand.  Ordinary
  // economic OPF retains its existing VOLL-backed load-shedding recourse.
  form_opt.load_shedding = opt.objective == ACOPFObjective::Economic;
  form_opt.voll = opt.voll;
  form_opt.eps_iac = 1e-6;
  form_opt.objective = opt.objective;
  form_opt.voltage_target_pu = opt.voltage_target_pu;
  form_opt.voltage_deviation_weight = opt.voltage_deviation_weight;
  form_opt.active_loss_weight = opt.active_loss_weight;
  form_opt.enforce_branch_limits = opt.enforce_branch_limits;
  form_opt.enforce_converter_capacity = opt.enforce_converter_capacity;
  form_opt.enforce_converter_current_limits = opt.enforce_converter_current_limits;
  form_opt.enforce_converter_modulation_limits = opt.enforce_converter_modulation_limits;
  bool dc_phase_one_requested = false;
  bool dc_phase_one_accepted = false;
  int dc_phase_one_iterations = 0;
  double dc_phase_one_runtime_ms = 0.0;
  double dc_phase_one_time_limit_ms =
      opt.ac_pf_dc_phase_one_time_limit_ms;
  bool dc_phase_one_budget_exhausted = false;
  double dc_phase_one_budget_overshoot_ms = 0.0;
  int dc_phase_one_symbolic_analyze_calls = 0;
  int parity_formulation_builds = 0;
  double dc_phase_one_residual = std::numeric_limits<double>::infinity();
  double dc_phase_one_candidate_primal =
      std::numeric_limits<double>::infinity();
  double dc_phase_one_candidate_dual =
      std::numeric_limits<double>::infinity();
  double dc_phase_one_baseline_primal =
      std::numeric_limits<double>::infinity();
  double dc_phase_one_baseline_dual =
      std::numeric_limits<double>::infinity();
  std::string dc_phase_one_status = "not-requested";
  const bool pure_ac = sys.dc.buses.empty() && sys.dc.branches.empty() &&
      sys.vsc_converters.empty() && sys.lcc_converters.empty() &&
      sys.dc.dcdc_converters.empty() && sys.energy_routers.empty();
  const bool dc_phase_one_eligible =
      opt.ac_pf_warm_start && opt.warm_start == nullptr &&
      opt.ac_pf_dc_phase_one && pure_ac &&
      static_cast<int>(sys.ac.buses.size()) >=
          std::max(0, opt.ac_pf_dc_phase_one_min_buses);
  bool formulation_reused = false;
  bool mapping_reused = false;
  bool prepared_structure_invalidated = false;
  std::string prepared_invalidation_reason = "not-prepared";
  parity::Problem local_problem;
  parity::Problem* problem_ptr = nullptr;
  if (prepared != nullptr) {
    const std::string snapshot = io::to_json(sys, -1);
    const bool same_model = prepared->problem.has_value() &&
        prepared->formulation_options.has_value() &&
        same_formulation_options(*prepared->formulation_options, form_opt) &&
        prepared->system_snapshot == snapshot;
    if (same_model) {
      problem_ptr = &*prepared->problem;
      formulation_reused = true;
      mapping_reused = true;
      prepared_invalidation_reason = "none";
    } else if (prepared->problem.has_value() &&
               prepared->formulation_options.has_value() &&
               same_formulation_options(*prepared->formulation_options,
                                        form_opt) &&
               parity::refresh_problem_numeric_data(*prepared->problem, sys)) {
      const std::uint64_t refreshed_layout =
          parity::problem_layout_signature(*prepared->problem);
      if (refreshed_layout != prepared->layout_signature) {
        throw std::logic_error(
            "prepared OPF numeric refresh changed the audited layout");
      }
      prepared->system_snapshot = std::move(snapshot);
      problem_ptr = &*prepared->problem;
      formulation_reused = true;
      mapping_reused = true;
      prepared_invalidation_reason =
          "numeric-parameters-refreshed: formulation, mapping, and symbolic retained";
    } else {
      const bool had_prepared_problem = prepared->problem.has_value();
      parity::Problem rebuilt = parity::build_problem(sys, form_opt);
      ++parity_formulation_builds;
      const std::uint64_t rebuilt_layout =
          parity::problem_layout_signature(rebuilt);
      if (prepared->layout_signature == 0) {
        prepared_invalidation_reason = "initial-build";
      } else if (prepared->layout_signature == rebuilt_layout &&
                 !had_prepared_problem) {
        prepared_invalidation_reason = "initial-build";
      } else {
        prepared_structure_invalidated = had_prepared_problem;
        prepared_invalidation_reason =
            prepared->layout_signature == rebuilt_layout
                ? "structural-compatibility-check-failed"
                : "layout-signature-changed";
        prepared->ipm_state.reset();
      }
      prepared->problem = std::move(rebuilt);
      prepared->formulation_options = form_opt;
      prepared->system_snapshot = std::move(snapshot);
      prepared->layout_signature = rebuilt_layout;
      problem_ptr = &*prepared->problem;
    }
  } else {
    local_problem = parity::build_problem(sys, form_opt);
    ++parity_formulation_builds;
    problem_ptr = &local_problem;
  }
  parity::Problem& prob = *problem_ptr;
  const std::uint64_t layout_signature =
      prepared != nullptr ? prepared->layout_signature
                          : parity::problem_layout_signature(prob);

  parity::IPMOptions ipm_opt;
  ipm_opt.max_iter = std::max(1, opt.max_inner_iterations * std::max(1, opt.max_outer_iterations));
  ipm_opt.tol_primal = std::max(opt.feasibility_tol, 1e-10);
  ipm_opt.tol_dual = std::max(opt.stationarity_tol, 1e-10);
  ipm_opt.tol_complementarity = std::max(opt.barrier_mu_min, 1e-10);
  ipm_opt.regularization = std::max(opt.regularization, 1e-12);
  ipm_opt.alpha_max = std::clamp(opt.interior_fraction, 0.5, 0.9999);
  ipm_opt.verbose = opt.verbose;
  ipm_opt.enable_phase_one = opt.enable_phase_one;
  ipm_opt.phase_one_time_limit_ms = opt.phase_one_time_limit_ms;
  ipm_opt.phase_one_max_iterations = opt.phase_one_max_iterations;
  ipm_opt.phase_one_max_factorizations = opt.phase_one_max_factorizations;
  ipm_opt.phase_one_max_backtracks = opt.phase_one_max_backtracks;
  ipm_opt.phase_one_barrier_mu = opt.phase_one_barrier_mu;
  ipm_opt.phase_one_admission_mu_factor =
      opt.phase_one_admission_mu_factor;
  ipm_opt.phase_one_primal_mu_factor = opt.phase_one_primal_mu_factor;
  ipm_opt.phase_one_centrality_tolerance =
      opt.phase_one_centrality_tolerance;
  ipm_opt.phase_one_dispatch_dual_predictor =
      opt.phase_one_dispatch_dual_predictor;
  ipm_opt.phase_one_dispatch_dual_min_improvement =
      opt.phase_one_dispatch_dual_min_improvement;
  ipm_opt.prepared_state = prepared != nullptr ? &prepared->ipm_state : nullptr;
  ipm_opt.prepared_numeric_refactor = opt.prepared_numeric_refactor;
  ipm_opt.prepared_numeric_max_relative_drift =
      opt.prepared_numeric_max_relative_drift;
  ipm_opt.prepared_numeric_backward_error_tolerance =
      opt.prepared_numeric_backward_error_tolerance;
  Eigen::VectorXd primal_warm_start;
  Eigen::VectorXd equality_dual_warm_start;
  Eigen::VectorXd inequality_dual_warm_start;
  Eigen::VectorXd slack_warm_start;
  bool warm_start_mapped = false;
  bool complete_continuation_state_supplied = false;
  // Hybrid-PF warm start (opt-in): seed AC voltages, DC voltages, VSC powers,
  // and DC/DC powers from the same coupled physical equations that the OPF
  // enforces.  A plain AC PF only supplies (vm, va) and leaves hybrid cases in
  // a large DC-balance mismatch basin.
  ACOPFResult pf_warm;
  if (opt.ac_pf_warm_start && opt.warm_start == nullptr) {
    hacdcpf::PowerFlowOptions pf_opt;
    pf_opt.max_iter = std::max(100, opt.max_inner_iterations);
    pf_opt.tol = std::min(opt.feasibility_tol, 1.0e-8);
    pf_opt.enable_converter_mode_switching = true;
    pf_opt.enable_pv_pq_conversion = true;
    pf_opt.enable_auto_swing_selection = true;
    const hacdcpf::PowerFlowResult baseline_pf =
        hacdcpf::powerflow::solve_hybrid(sys, pf_opt);
    if (baseline_pf.converged) {
      pf_warm = build_hybrid_power_flow_warm_start(prob, sys, baseline_pf);
      primal_warm_start = build_parity_primal_warm_start(
          prob, pf_warm, warm_start_mapped);
    }
    const auto baseline_diagnostics = warm_start_mapped
        ? parity::evaluate_ipm_initial_point(prob, primal_warm_start)
        : parity::IPMInitialPointDiagnostics{};
    dc_phase_one_baseline_primal = baseline_diagnostics.primal_inf;
    dc_phase_one_baseline_dual = baseline_diagnostics.dual_inf;

    const bool baseline_needs_dispatch_phase =
        !baseline_diagnostics.valid ||
        baseline_diagnostics.dual_inf > std::max(
            0.0, opt.ac_pf_dc_phase_one_baseline_dual_threshold);
    if (dc_phase_one_eligible) {
      dc_phase_one_requested = true;
      if (!baseline_needs_dispatch_phase) {
        dc_phase_one_status =
            "skipped: baseline AC-PF dual residual already acceptable";
      } else {
        DCOPFOptions dc_options;
        dc_options.solver = DCOPFSolverBackend::NativeQP;
        dc_options.max_iterations =
            std::max(1, opt.ac_pf_dc_phase_one_max_iterations);
        dc_options.phase_one_time_limit_ms =
            opt.ac_pf_dc_phase_one_time_limit_ms;
        dc_options.compute_lmp = false;
        dc_options.load_shedding = false;
        dc_options.structural_warm_start = true;
        dc_options.compact_quadratic_model = true;
        dc_options.accept_phase_one_iterate = true;
        dc_options.phase_one_iterate_tolerance =
            std::max(0.0, opt.ac_pf_dc_phase_one_tolerance);
        const DCOPFResult dc = solve_dc_opf(sys, dc_options);
        dc_phase_one_iterations = dc.iterations;
        dc_phase_one_runtime_ms = 1000.0 * dc.runtime_sec;
        dc_phase_one_budget_exhausted = dc.phase_one_budget_exhausted;
        dc_phase_one_budget_overshoot_ms = dc.phase_one_budget_overshoot_ms;
        dc_phase_one_symbolic_analyze_calls =
            dc.native_qp_symbolic_analyze_calls;
        dc_phase_one_residual = dc.phase_one_warm_start_only
            ? dc.phase_one_iterate_residual
            : dc.solver_initial_primal_residual;
        dc_phase_one_status = dc.status;
        const bool usable = dc.converged || dc.phase_one_warm_start_only;
        const bool mapping_matches =
            dc.pg_mw.size() == sys.ac.generators.size() &&
            dc.va.size() == sys.ac.buses.size();
        if (usable && mapping_matches) {
          HybridPowerSystem candidate_system = sys;
          for (std::size_t i = 0;
               i < candidate_system.ac.generators.size(); ++i)
            candidate_system.ac.generators[i].pg_mw = dc.pg_mw[i];
          for (std::size_t i = 0; i < candidate_system.ac.buses.size(); ++i)
            candidate_system.ac.buses[i].va_deg = dc.va[i] / kDegToRad;
          const hacdcpf::PowerFlowResult candidate_pf =
              hacdcpf::powerflow::solve_hybrid(candidate_system, pf_opt);
          bool candidate_mapped = false;
          bool candidate_operating_point_loaded = false;
          Eigen::VectorXd candidate_primal;
          ACOPFResult candidate_warm;
          if (candidate_pf.converged) {
            // build_parity_primal_warm_start preserves Phase-I load-shedding
            // slacks from build_initial_point. Refresh the mutable operating
            // point first so those un-mapped variables match the candidate,
            // exactly as a full Problem rebuild would, while all formulation
            // structure and sparse matrices remain shared.
            update_parity_operating_point(prob, candidate_system);
            candidate_operating_point_loaded = true;
            candidate_warm = build_hybrid_power_flow_warm_start(
                prob, candidate_system, candidate_pf);
            candidate_primal = build_parity_primal_warm_start(
                prob, candidate_warm, candidate_mapped);
          }
          const auto candidate_diagnostics = candidate_mapped
              ? parity::evaluate_ipm_initial_point(
                    prob, candidate_primal)
              : parity::IPMInitialPointDiagnostics{};
          dc_phase_one_candidate_primal = candidate_diagnostics.primal_inf;
          dc_phase_one_candidate_dual = candidate_diagnostics.dual_inf;
          const double required_improvement = std::clamp(
              opt.ac_pf_dc_phase_one_min_dual_improvement, 0.0, 1.0);
          const bool improves_dual = candidate_diagnostics.valid &&
              baseline_diagnostics.valid &&
              candidate_diagnostics.dual_inf <=
                  (1.0 - required_improvement) *
                      baseline_diagnostics.dual_inf;
          const bool improves_primal = candidate_diagnostics.valid &&
              baseline_diagnostics.valid &&
              candidate_diagnostics.primal_inf <=
                  baseline_diagnostics.primal_inf;
          if (candidate_diagnostics.valid &&
              (improves_dual || improves_primal ||
               !baseline_diagnostics.valid)) {
            dc_phase_one_accepted = true;
            pf_warm = std::move(candidate_warm);
            primal_warm_start = std::move(candidate_primal);
            warm_start_mapped = true;
          } else {
            if (candidate_operating_point_loaded)
              update_parity_operating_point(prob, sys);
            dc_phase_one_status +=
                "; rejected: no primal/dual initialization improvement";
          }
        } else if (usable) {
          dc_phase_one_status +=
              "; rejected: authored result mapping mismatch";
        }
      }
    }
    if (warm_start_mapped) ipm_opt.primal_start = &primal_warm_start;
  }
  if (opt.warm_start != nullptr) {
    const bool exact_layout =
        !prepared_structure_invalidated &&
        opt.warm_start->ipm_layout_signature != 0 &&
        opt.warm_start->ipm_layout_signature == layout_signature;
    const bool complete_native_state = exact_layout &&
        opt.warm_start->ipm_primal_state.size() ==
            static_cast<size_t>(prob.vidx.n_total) &&
        opt.warm_start->ipm_equality_dual_state.size() ==
            static_cast<size_t>(prob.cidx.n_eq_total) &&
        !opt.warm_start->ipm_inequality_dual_state.empty() &&
        opt.warm_start->ipm_inequality_dual_state.size() ==
            opt.warm_start->ipm_slack_state.size();
    ACOPFResult physical_only;
    const ACOPFResult* warm_source = opt.warm_start;
    if (!complete_native_state &&
        !opt.warm_start->ipm_primal_state.empty()) {
      // Dimension compatibility alone does not establish semantic row/column
      // identity. Preserve the public physical mapping while removing every
      // opaque native block as one all-or-nothing unit.
      physical_only = *opt.warm_start;
      physical_only.ipm_primal_state.clear();
      physical_only.ipm_equality_dual_state.clear();
      physical_only.ipm_inequality_dual_state.clear();
      physical_only.ipm_slack_state.clear();
      warm_source = &physical_only;
    }
    primal_warm_start =
        build_parity_primal_warm_start(prob, *warm_source, warm_start_mapped);
    if (warm_start_mapped) {
      ipm_opt.primal_start = &primal_warm_start;
      const auto copy_state = [](const std::vector<double>& source,
                                 Eigen::VectorXd& destination) {
        if (source.empty()) {
          destination.resize(0);
          return;
        }
        destination = Eigen::Map<const Eigen::VectorXd>(
            source.data(), static_cast<Eigen::Index>(source.size()));
      };
      if (complete_native_state) {
        copy_state(opt.warm_start->ipm_equality_dual_state,
                   equality_dual_warm_start);
        copy_state(opt.warm_start->ipm_inequality_dual_state,
                   inequality_dual_warm_start);
        copy_state(opt.warm_start->ipm_slack_state, slack_warm_start);
        ipm_opt.equality_dual_start = &equality_dual_warm_start;
        ipm_opt.inequality_dual_start = &inequality_dual_warm_start;
        ipm_opt.slack_start = &slack_warm_start;
        complete_continuation_state_supplied = true;
      }
    }
  }

  const auto& vidx = prob.vidx;
  const auto& cidx = prob.cidx;

  // ── Inner nonlinear solver selection ──────────────────────────────────────
  // Auto with a coupled PF start uses standalone Ipopt first on difficult
  // hybrid cases, then optionally gives its primal point to Native IPM for
  // refinement.  Objective homotopy is kept on the Native-only branch below.
  parity::IPMResult ipm_res;
  std::string backend_label;
  if (inner == ParityInnerSolver::Auto && opt.ac_pf_warm_start &&
      !opt.objective_homotopy) {
    // A converged hybrid PF point is an excellent primal start for Ipopt on
    // difficult hybrid cases.  Refine Ipopt's last primal iterate with the
    // native IPM when Ipopt stops at its acceptable KKT limit; this avoids
    // losing a nearly feasible point just because the external adapter reports
    // a maximum-iteration termination.
    bool ipopt_ok = false;
    ipm_res = solve_parity_with_ipopt(prob, ipm_opt, ipopt_ok);
    backend_label = "ipopt_filter_linesearch";
    if (ipopt_ok && !ipm_res.converged &&
        ipm_res.x.size() == prob.vidx.n_total && ipm_res.x.allFinite()) {
      parity::IPMOptions refine_opt = ipm_opt;
      refine_opt.primal_start = &ipm_res.x;
      refine_opt.equality_dual_start = &ipm_res.lambda_eq;
      refine_opt.inequality_dual_start = &ipm_res.mu;
      refine_opt.slack_start = &ipm_res.z;
      const parity::IPMResult refined =
          parity::solve_primal_dual_ipm(prob, refine_opt);
      if (refined.converged) {
        ipm_res = refined;
        backend_label = "parity_ipm:" + ipm_res.linear_solver +
                        " (Ipopt warm-start)";
      }
    }
    if (!ipopt_ok) {
      ipm_res = parity::solve_primal_dual_ipm(prob, ipm_opt);
      backend_label = "parity_ipm:" + ipm_res.linear_solver +
                      " (ipopt unavailable)";
    }
  } else if (inner == ParityInnerSolver::Ipopt) {
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
  out.profiling.analyze_calls = ipm_res.symbolic_analyze_calls;
  out.profiling.factorization_calls = ipm_res.factorization_calls;
  out.profiling.linear_solve_calls = ipm_res.linear_solve_calls;
  out.profiling.accepted_steps = ipm_res.accepted_steps;
  out.profiling.rejected_steps = ipm_res.rejected_steps;
  out.profiling.backend_escalations = ipm_res.backend_escalations;
  out.profiling.scaling_rebuilds = ipm_res.scaling_rebuilds;
  out.profiling.warm_start_used = ipm_res.warm_start_used;
  out.profiling.prepared_session_used = prepared != nullptr;
  out.profiling.formulation_reused = formulation_reused;
  out.profiling.mapping_reused = mapping_reused;
  out.profiling.symbolic_reused = ipm_res.symbolic_reused;
  out.profiling.continuation_state_reused =
      complete_continuation_state_supplied &&
      ipm_res.phase_two_start_accepted;
  out.profiling.numeric_refactor_attempted =
      ipm_res.numeric_refactor_attempted;
  out.profiling.numeric_refactor_accepted =
      ipm_res.numeric_refactor_accepted;
  out.profiling.numeric_refactor_relative_drift =
      ipm_res.numeric_refactor_relative_drift;
  out.profiling.numeric_refactor_backward_error =
      ipm_res.numeric_refactor_backward_error;
  out.profiling.numeric_refactor_status = ipm_res.numeric_refactor_status;
  out.profiling.prepared_session_invalidation_reason =
      std::move(prepared_invalidation_reason);
  out.profiling.initial_primal_residual = ipm_res.initial_primal_inf;
  out.profiling.initial_dual_residual = ipm_res.initial_dual_inf;
  out.profiling.dc_phase_one_requested = dc_phase_one_requested;
  out.profiling.dc_phase_one_accepted = dc_phase_one_accepted;
  out.profiling.dc_phase_one_iterations = dc_phase_one_iterations;
  out.profiling.dc_phase_one_runtime_ms = dc_phase_one_runtime_ms;
  out.profiling.dc_phase_one_time_limit_ms = dc_phase_one_time_limit_ms;
  out.profiling.dc_phase_one_budget_exhausted =
      dc_phase_one_budget_exhausted;
  out.profiling.dc_phase_one_budget_overshoot_ms =
      dc_phase_one_budget_overshoot_ms;
  out.profiling.dc_phase_one_symbolic_analyze_calls =
      dc_phase_one_symbolic_analyze_calls;
  out.profiling.parity_formulation_builds = parity_formulation_builds;
  out.profiling.dc_phase_one_residual = dc_phase_one_residual;
  out.profiling.dc_phase_one_candidate_primal =
      dc_phase_one_candidate_primal;
  out.profiling.dc_phase_one_candidate_dual =
      dc_phase_one_candidate_dual;
  out.profiling.dc_phase_one_baseline_primal =
      dc_phase_one_baseline_primal;
  out.profiling.dc_phase_one_baseline_dual =
      dc_phase_one_baseline_dual;
  out.profiling.dc_phase_one_status = std::move(dc_phase_one_status);
  out.profiling.phase_one_initial_violation =
      ipm_res.phase_one_initial_violation;
  out.profiling.phase_one_constraint_violation =
      ipm_res.phase_one_constraint_violation;
  out.profiling.phase_one_dual_fit_residual =
      ipm_res.phase_one_dual_fit_residual;
  out.profiling.phase_one_primal_feasible =
      ipm_res.phase_one_primal_feasible;
  out.profiling.phase_one_in_handoff_corridor =
      ipm_res.phase_one_in_handoff_corridor;
  out.profiling.phase_one_dual_initialized =
      ipm_res.phase_one_dual_initialized;
  out.profiling.phase_one_handoff_primal_tolerance =
      ipm_res.phase_one_handoff_primal_tolerance;
  out.profiling.phase_one_perturbed_primal_residual =
      ipm_res.phase_one_perturbed_primal_residual;
  out.profiling.phase_one_centrality = ipm_res.phase_one_centrality;
  out.profiling.dispatch_dual_predictor_attempted =
      ipm_res.dispatch_dual_predictor_attempted;
  out.profiling.dispatch_dual_predictor_accepted =
      ipm_res.dispatch_dual_predictor_accepted;
  out.profiling.dispatch_dual_predictor_runtime_ms =
      ipm_res.dispatch_dual_predictor_runtime_ms;
  out.profiling.dispatch_dual_predictor_baseline_raw =
      ipm_res.dispatch_dual_predictor_baseline_raw;
  out.profiling.dispatch_dual_predictor_candidate_raw =
      ipm_res.dispatch_dual_predictor_candidate_raw;
  out.profiling.dispatch_dual_predictor_baseline_normalized =
      ipm_res.dispatch_dual_predictor_baseline_normalized;
  out.profiling.dispatch_dual_predictor_candidate_normalized =
      ipm_res.dispatch_dual_predictor_candidate_normalized;
  out.profiling.dispatch_dual_predictor_status =
      std::move(ipm_res.dispatch_dual_predictor_status);
  out.profiling.phase_one_barrier_mu = ipm_res.phase_one_barrier_mu;
  out.profiling.phase_one_budget_exhausted =
      ipm_res.phase_one_budget_exhausted;
  out.profiling.phase_one_iterations = ipm_res.phase_one_iterations;
  out.profiling.phase_one_factorizations = ipm_res.phase_one_factorizations;
  out.profiling.phase_one_backtracks = ipm_res.phase_one_backtracks;
  out.profiling.phase_one_structural_step_attempted =
      ipm_res.phase_one_structural_step_attempted;
  out.profiling.phase_one_structural_step_accepted =
      ipm_res.phase_one_structural_step_accepted;
  out.profiling.phase_one_structural_factorizations =
      ipm_res.phase_one_structural_factorizations;
  out.profiling.phase_one_structural_violation =
      ipm_res.phase_one_structural_violation;
  out.profiling.phase_one_structure = ipm_res.phase_one_structure;
  out.profiling.phase_one_runtime_ms = ipm_res.phase_one_runtime_ms;
  out.profiling.phase_one_termination = ipm_res.phase_one_termination;
  out.profiling.phase_one_linear_solver = ipm_res.phase_one_linear_solver;
  out.profiling.phase_two_start_accepted =
      ipm_res.phase_two_start_accepted;
  out.profiling.phase_two_start_rejection_reason =
      ipm_res.phase_two_start_rejection_reason;
  out.profiling.final_barrier_mu = ipm_res.complementarity;
  const auto save_state = [](const Eigen::VectorXd& source,
                             std::vector<double>& destination) {
    if (source.size() == 0 || !source.allFinite()) {
      destination.clear();
    } else {
      destination.assign(source.data(), source.data() + source.size());
    }
  };
  save_state(ipm_res.x, out.ipm_primal_state);
  save_state(ipm_res.lambda_eq, out.ipm_equality_dual_state);
  save_state(ipm_res.mu, out.ipm_inequality_dual_state);
  save_state(ipm_res.z, out.ipm_slack_state);
  out.ipm_layout_signature = layout_signature;

  // Optional Davidenko homotopy tangent at the returned point (one extra
  // inertia-controlled KKT solve) for objective-continuation drivers.
  const bool have_finite_primal =
      ipm_res.x.size() == vidx.n_total && ipm_res.x.allFinite();
  if (opt.compute_homotopy_tangent && have_finite_primal) {
    Eigen::VectorXd dxdt;
    Eigen::VectorXd dzdt;
    Eigen::VectorXd dldt;
    Eigen::VectorXd dmdt;
    if (parity::homotopy_tangent(prob, ipm_res.x, ipm_res.z, ipm_res.lambda_eq,
                                 ipm_res.mu, opt.homotopy_t,
                                 dxdt, dzdt, dldt, dmdt)) {
      save_state(dxdt, out.ipm_tangent_primal);
      save_state(dzdt, out.ipm_tangent_slack);
      save_state(dldt, out.ipm_tangent_equality_dual);
      save_state(dmdt, out.ipm_tangent_inequality_dual);
    }
  }

  out.vm.assign(static_cast<size_t>(vidx.n_vm), 1.0);
  out.va.assign(static_cast<size_t>(vidx.n_va), 0.0);
  out.pg_mw.assign(prob.data.generators.size(), 0.0);
  out.qg_mvar.assign(prob.data.generators.size(), 0.0);
  set_generator_result_map(out, prob.data);
  for (size_t gi = 0; gi < prob.data.generators.size(); ++gi) {
    out.pg_mw[gi] = prob.data.generators[gi].pg_mw;
    out.qg_mvar[gi] = prob.data.generators[gi].qg_mvar;
  }

  if (have_finite_primal) {
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
    if (vidx.n_erp > 0) {
      const size_t ner = static_cast<size_t>(vidx.n_erp);
      out.er_port_p_mw.assign(ner, 0.0);
      out.er_port_q_mvar.assign(ner, 0.0);
      out.er_port_map.resize(ner);
      for (const auto& port : prob.er_ports) {
        if (port.pvar < 0 || port.pvar >= vidx.n_erp) continue;
        const size_t ku = static_cast<size_t>(port.pvar);
        out.er_port_p_mw[ku] =
            ipm_res.x[vidx.i_erp + port.pvar] * prob.data.base_mva;
        if (port.qvar >= 0 && port.qvar < vidx.n_erq) {
          out.er_port_q_mvar[ku] =
              ipm_res.x[vidx.i_erq + port.qvar] * prob.data.base_mva;
        }
        const auto& er = prob.data.energy_routers[static_cast<size_t>(port.router_idx)];
        const auto& er_port = er.ports[static_cast<size_t>(port.port_idx)];
        out.er_port_map[ku].original_index = er.index;
        out.er_port_map[ku].source_type = er_port.index;
      }
    }

    // Re-evaluate every in-service LCC at the optimized voltage state. The
    // canonical formulation may renumber or merge buses, so restore authored
    // bus identifiers by stable LCCConverter.index before exposing the rows.
    if (!prob.data.lcc_converters.empty()) {
      const Eigen::Map<const Eigen::VectorXd> vm_state(
          ipm_res.x.data() + vidx.i_vm, vidx.n_vm);
      const Eigen::Map<const Eigen::VectorXd> vdc_state(
          ipm_res.x.data() + vidx.i_vdc, vidx.n_vdc);
      out.lcc_transfers.reserve(prob.data.lcc_converters.size());
      for (const auto& lcc : prob.data.lcc_converters) {
        if (!lcc.in_service) continue;
        const int ac = lcc.ac_bus - 1;
        const int dc = lcc.dc_bus - 1;
        if (ac < 0 || ac >= vm_state.size() ||
            dc < 0 || dc >= vdc_state.size()) {
          continue;
        }
        const double e_kv = vm_state[ac] * lcc.vn_ac_kv;
        const double commutation_e_kv =
            powerflow::lcc_commutation_voltage_kv(
                prob.data, lcc, vm_state);
        const double ud_kv =
            vdc_state[dc] * powerflow::lcc_dc_base_kv(prob.data, lcc);
        const auto operating_point = powerflow::lcc_operating_point(
            lcc, e_kv, commutation_e_kv, ud_kv);
        if (!operating_point.valid) continue;

        LCCTransfer transfer;
        transfer.index = lcc.index;
        transfer.bus_ac = lcc.ac_bus;
        transfer.bus_dc = lcc.dc_bus;
        const auto authored = std::find_if(
            sys.lcc_converters.begin(), sys.lcc_converters.end(),
            [&](const LCCConverter& candidate) {
              return candidate.index == lcc.index;
            });
        if (authored != sys.lcc_converters.end()) {
          transfer.bus_ac = authored->ac_bus;
          transfer.bus_dc = authored->dc_bus;
        }
        transfer.station_role = static_cast<int>(lcc.station_role);
        transfer.control_mode = static_cast<int>(lcc.control_mode);
        transfer.alpha_deg = operating_point.alpha_deg;
        transfer.gamma_deg = operating_point.gamma_deg;
        transfer.ud0_kv = operating_point.ud0_kv;
        transfer.ud_kv = operating_point.ud_kv;
        transfer.id_ka = operating_point.id_ka;
        transfer.p_ac_mw = operating_point.p_ac_mw;
        transfer.q_ac_mvar = operating_point.q_ac_mvar;
        transfer.p_dc_mw = operating_point.p_dc_mw;
        transfer.id_at_limit = operating_point.id_at_limit;
        transfer.alpha_within_limits =
            operating_point.alpha_deg >= lcc.alpha_min_deg - 1e-6 &&
            operating_point.alpha_deg <= lcc.alpha_stop_deg + 1e-6;
        transfer.gamma_within_limits =
            lcc.gamma_min_deg <= 0.0 ||
            operating_point.gamma_deg >= lcc.gamma_min_deg - 1e-6;
        out.lcc_transfers.push_back(transfer);

        if (lcc.station_role == LCCStationRole::Rectifier &&
            !transfer.alpha_within_limits) {
          out.model_limitations.push_back(
              "LCC rectifier " + std::to_string(lcc.index) +
              " violates its post-solve firing-angle window at the fixed tap.");
        }
        if (lcc.station_role == LCCStationRole::Inverter &&
            !transfer.gamma_within_limits) {
          out.model_limitations.push_back(
              "LCC inverter " + std::to_string(lcc.index) +
              " violates its post-solve minimum extinction angle at the fixed tap.");
        }
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
      out.lmp_valid = true;
      out.lmp_validity_reason =
          "AC nodal prices were extracted from the solved formulation's equality multipliers.";
    } else {
      out.lmp_valid = false;
      out.lmp_validity_reason =
          backend_label.find("ipopt") != std::string::npos
              ? "The AC OPF Ipopt result mapping does not publish constraint multipliers; LMPs are unavailable."
              : "The selected solve path did not return a complete equality-dual vector; LMPs are unavailable.";
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
    const auto block_inf = [&](int offset, int count) {
      if (count <= 0 || offset < 0 || offset + count > g_eq.size()) return 0.0;
      return g_eq.segment(offset, count).cwiseAbs().maxCoeff();
    };
    out.profiling.max_ac_p_balance_residual_pu =
        block_inf(cidx.i_pbal_ac, cidx.n_pbal_ac) /
        std::max(prob.scale_p, 1e-16);
    out.profiling.max_ac_q_balance_residual_pu =
        block_inf(cidx.i_qbal_ac, cidx.n_qbal_ac) /
        std::max(prob.scale_q, 1e-16);
    out.profiling.max_dc_balance_residual_pu =
        block_inf(cidx.i_pbal_dc, cidx.n_pbal_dc);
    out.profiling.max_converter_balance_residual_pu =
        block_inf(cidx.i_conv_bal, cidx.n_conv_bal);
    double other_eq = 0.0;
    other_eq = std::max(other_eq,
                        block_inf(cidx.i_dcdc_bal, cidx.n_dcdc_bal));
    other_eq = std::max(other_eq, block_inf(cidx.i_er_bal, cidx.n_er_bal));
    other_eq = std::max(other_eq, block_inf(cidx.i_ac_ref, cidx.n_ac_ref));
    other_eq = std::max(other_eq, block_inf(cidx.i_dc_ref, cidx.n_dc_ref));
    out.profiling.max_other_equality_residual_pu = other_eq;
    out.profiling.max_nonlinear_inequality_violation_pu = max_hplus;
    out.max_constraint_violation = std::max({inf_norm(g_eq), max_hplus, max_bound});
  } else {
    out.objective = std::numeric_limits<double>::infinity();
    out.max_constraint_violation = std::numeric_limits<double>::infinity();
  }

  if (out.converged) {
    const bool acceptable_ipopt =
        ipm_res.status.find("acceptable residuals") != std::string::npos;
    out.status = "converged (" + backend_label + ")";
    if (acceptable_ipopt) out.status += " [acceptable residuals]";
  } else {
    out.status = "not converged: " + ipm_res.status;
  }

  // Expand per-bus results back to the original user-facing AC bus order.
  // Projection may create a map for pure reindexing or dead-island stripping
  // even when no physical bus merge occurred, so map presence is the gate.
  if (prob.data.bus_merge_map) {
    unproject_per_bus_ac_opf_result(out, *prob.data.bus_merge_map);
  }
  trim_internal_dc_bus_results(out, prob.data.projection_certificate);

  return out;
}

bool contains_hybrid_acdc_components(const HybridPowerSystem& sys) {
  return !sys.dc.buses.empty()               ||
         !sys.dc.branches.empty()            ||
         !sys.dc.loads.empty()               ||
         !sys.vsc_converters.empty()         ||
         !sys.lcc_converters.empty()         ||
         !sys.dc.dcdc_converters.empty()     ||
         !sys.dc.dc_circuit_breakers.empty() ||
         !sys.dc.storage.empty()             ||
         !sys.dc.pv_arrays.empty()           ||
         !sys.dc.static_generators.empty()   ||
         !sys.dc.dc_static_generators.empty();
}

}  // namespace

struct PreparedACOPFSession::Impl {
  explicit Impl(ACOPFOptions options) {
    state.options = std::move(options);
  }
  PreparedSessionState state;
};

ACOPFResult solve_ac_opf_impl(const HybridPowerSystem& sys,
                              const ACOPFOptions& opt_in,
                              PreparedSessionState* prepared) {
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

  // The LCC injection helpers deliberately return zero for unusable station
  // configurations so PF can emit diagnostics without producing NaNs. OPF
  // must be stricter: silently dropping a fixed transfer changes feasibility
  // and the optimum, so reject unsupported controls and missing characteristic
  // data before canonical projection.
  for (const auto& lcc : sys.lcc_converters) {
    if (!lcc.in_service) continue;
    const auto reject_lcc = [&](const std::string& reason) {
      out.status = "AC OPF failed: LCC readiness check failed for converter " +
                   std::to_string(lcc.index) + ": " + reason;
      out.infeasibility_hints.push_back(out.status);
    };
    if (!powerflow::lcc_control_supported(lcc)) {
      reject_lcc("station role and control mode are not a supported pair.");
      return out;
    }
    if (lcc.n_bridges <= 0 || !(lcc.vn_ac_kv > 0.0)) {
      reject_lcc("n_bridges and vn_ac_kv must both be positive.");
      return out;
    }
    const bool characteristic =
        lcc.control_mode == LCCControlMode::ConstantAlpha ||
        lcc.control_mode == LCCControlMode::ConstantGamma;
    if (characteristic && !(lcc.x_comm_ohm > 0.0)) {
      reject_lcc(
          "constant-alpha/gamma control requires positive x_comm_ohm; the numerical reactance floor is not an OPF model.");
      return out;
    }
    const bool ac_bus_exists = std::any_of(
        sys.ac.buses.begin(), sys.ac.buses.end(),
        [&](const ACBus& bus) {
          return bus.in_service && bus.index == lcc.ac_bus;
        });
    const auto dc_bus = std::find_if(
        sys.dc.buses.begin(), sys.dc.buses.end(),
        [&](const DCBus& bus) {
          return bus.in_service && bus.index == lcc.dc_bus;
        });
    if (!ac_bus_exists || dc_bus == sys.dc.buses.end()) {
      reject_lcc("the in-service AC or DC terminal bus cannot be resolved.");
      return out;
    }
    if (!(dc_bus->base_kv > 0.0) && !(lcc.rated_dc_kv > 0.0)) {
      reject_lcc("a positive DC bus base_kv or rated_dc_kv is required.");
      return out;
    }
  }

  const auto reference_validation =
      validation::validate_reference_bus_eligibility(sys);
  const bool bare_ideal_dc_boundary = std::any_of(
      reference_validation.issues.begin(), reference_validation.issues.end(),
      [](const validation::ValidationIssue& issue) {
        return issue.severity == validation::Severity::Warning &&
               issue.component_type == "DCBus" && issue.field == "bus_type";
      });
  if (reference_validation.has_errors() || bare_ideal_dc_boundary) {
    out.status = "AC OPF failed: reference-bus eligibility check failed: " +
                 reference_validation.summary();
    if (bare_ideal_dc_boundary) {
      out.status +=
          " Optimisation requires an explicit controllable DC balancing "
          "device because an ideal DC_V boundary has no dispatch-cost model.";
    }
    for (const auto& issue : reference_validation.issues)
      out.infeasibility_hints.push_back(issue.message);
    return out;
  }

  // Working copy: every in-service external grid is an independent dispatchable
  // source. Cost-free grids and grids sharing a bus with a local generator must
  // still participate; both are physical sources in PF/dynamic simulation.
  HybridPowerSystem sys_work = sys;
  const size_t original_generator_count = sys.ac.generators.size();
  std::unordered_map<int, size_t> external_grid_by_synthetic_id;
  for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
    const auto& eg = sys.ac.external_grids[i];
    if (!eg.in_service) continue;
    // Keep the promoted variable on the physical scale of the case.  A huge
    // synthetic bound makes the parity KKT system ill-conditioned for small
    // distribution cases (for example a 1.5 MVA feeder supplied by a grid with
    // s_sc_max_mva=100).  The short-circuit rating is not an import limit, but
    // it is a useful finite scale; fall back to one system-base unit when it is
    // unavailable.
    const double external_grid_scale_mw =
        std::max({1.0, eg.s_sc_max_mva, sys.base_mva});
    Generator eg_gen;
    eg_gen.index = std::numeric_limits<int>::min() + static_cast<int>(i);
    eg_gen.bus        = eg.bus;
    eg_gen.in_service = true;
    eg_gen.name       = eg.name.empty() ? ("EG_opf_" + std::to_string(eg.index)) : eg.name;
    eg_gen.vg_pu      = eg.vm_pu;
    eg_gen.pmax_mw    =  external_grid_scale_mw;
    eg_gen.pmin_mw    = -external_grid_scale_mw;
    eg_gen.qmax_mvar  =  external_grid_scale_mw;
    eg_gen.qmin_mvar  = -external_grid_scale_mw;
    eg_gen.cost_c2    = eg.cost_c2;
    eg_gen.cost_c1    = eg.cost_c1;
    eg_gen.cost_c0    = eg.cost_c0;
    sys_work.ac.generators.push_back(std::move(eg_gen));
    external_grid_by_synthetic_id[std::numeric_limits<int>::min() +
                                  static_cast<int>(i)] = i;
  }

  std::unordered_map<int, size_t> authored_generator_by_id;
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    authored_generator_by_id.emplace(sys.ac.generators[i].index, i);
  }

  auto separate_external_grid_dispatch =
      [&](ACOPFResult result) -> ACOPFResult {
    result.model_limitations.push_back(
        "AC OPF is a non-convex nonlinear program; convergence certifies a local KKT point, not a global optimum.");
    if (!sys.ac.storage.empty() || !sys.dc.storage.empty()) {
      result.model_limitations.push_back(
          "Storage is optimized as a single-period power injection; intertemporal SOC dynamics are not modelled by snapshot OPF.");
    }
    if (!sys.energy_routers.empty()) {
      result.model_limitations.push_back(
          "Energy-router port balance is lossless; internal router conversion losses are not represented.");
    }
    if (std::any_of(sys.vsc_converters.begin(), sys.vsc_converters.end(),
                    [](const VSCConverter& converter) {
                      return converter.in_service &&
                             converter.control_mode ==
                                 ConverterMode::AC_GRID_FORMING;
                    })) {
      result.model_limitations.push_back(
          "Balanced AC OPF treats grid-forming VSCs as free P/Q converter "
          "ports with the selected engineering inequalities; internal voltage, "
          "virtual impedance, and priority NCP are certified only by the "
          "post-dispatch power-flow replay and are not OPF KKT constraints.");
    }
    if (!result.lmp_valid && result.lmp_validity_reason.empty()) {
      result.lmp_validity_reason =
          "The selected OPF path did not provide certified nodal-price multipliers.";
    }
    if (result.solver_path == OPFSolverPath::NativeAC) {
      result.model_limitations.push_back(
          "The legacy Native AC path handles nonlinear engineering limits through an external barrier/penalty loop; prefer ParityIPM for the production hybrid formulation.");
    }
    result.external_grid_p_mw.assign(sys.ac.external_grids.size(), 0.0);
    result.external_grid_q_mvar.assign(sys.ac.external_grids.size(), 0.0);
    std::vector<double> authored_pg(original_generator_count, 0.0);
    std::vector<double> authored_qg(original_generator_count, 0.0);
    std::vector<ACOPFResult::ComponentRef> authored_map(original_generator_count);
    for (size_t i = 0; i < original_generator_count; ++i) {
      authored_map[i].original_index = static_cast<int>(i);
      authored_map[i].source_type = 0;
    }
    if ((!result.pg_mw.empty() || !result.qg_mvar.empty()) &&
        result.gen_map.size() != result.pg_mw.size()) {
      throw std::runtime_error(
          "AC OPF generator result map is not aligned with dispatch rows");
    }
    for (size_t row = 0; row < result.gen_map.size(); ++row) {
      const int component_id = result.gen_map[row].original_index;
      if (const auto external =
              external_grid_by_synthetic_id.find(component_id);
          external != external_grid_by_synthetic_id.end()) {
        if (row < result.pg_mw.size())
          result.external_grid_p_mw[external->second] = result.pg_mw[row];
        if (row < result.qg_mvar.size())
          result.external_grid_q_mvar[external->second] = result.qg_mvar[row];
        continue;
      }
      const auto authored = authored_generator_by_id.find(component_id);
      if (authored == authored_generator_by_id.end()) continue;
      if (row < result.pg_mw.size())
        authored_pg[authored->second] = result.pg_mw[row];
      if (row < result.qg_mvar.size())
        authored_qg[authored->second] = result.qg_mvar[row];
    }
    result.pg_mw = std::move(authored_pg);
    result.qg_mvar = std::move(authored_qg);
    result.gen_map = std::move(authored_map);
    return result;
  };

  const bool has_hybrid_acdc = contains_hybrid_acdc_components(sys);
  const bool has_lcc = std::any_of(
      sys.lcc_converters.begin(), sys.lcc_converters.end(),
      [](const LCCConverter& lcc) { return lcc.in_service; });
  const bool dropped_dc = has_hybrid_acdc;
  auto append_hybrid_fallback_suppression = [&]() {
    if (!dropped_dc || !opt.allow_fallback) {
      return;
    }
    const std::string hint =
        "AC-only economic-dispatch fallback suppressed because the case contains DC/VSC/LCC components.";
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
  // The legacy native formulation does not have Vdc columns for LCC coupling.
  // Force every non-EconomicDispatch LCC solve through the shared
  // Parity/Ipopt formulation even when legacy flags were set inconsistently.
  if (has_lcc &&
      opt.ac_solver_backend != ACOPFSolverBackend::EconomicDispatch) {
    opt.enable_primal_dual = true;
    opt.use_parity_ipm = true;
  }

  if (opt.enable_primal_dual && opt.use_parity_ipm) {
    // Objective homotopy is implemented only by the native parity IPM.  An
    // explicit Ipopt request must remain a standalone solve; never put Ipopt
    // into the continuation state chain.
    if (opt.objective_homotopy && inner != ParityInnerSolver::Ipopt) {
      // Theory-guided two-phase solve (docs/numerical_methods.md §12–13):
      // follow the objective homotopy P(t) = min t·f from t = 0 (feasibility)
      // to t = 1 (full cost), carrying the full primal-dual state between
      // steps with Davidenko tangent prediction (parameter-metric trust
      // radius) and the adaptive step rule (fast steps double, failures
      // halve; collapse → honest minimal-violation report).
      std::vector<ACOPFResult> chain;
      ACOPFResult predicted;
      bool have_prediction = false;
      double t_ok = 0.0;
      double dt = (opt.homotopy_dt0 > 0.0) ? opt.homotopy_dt0 : 0.10;
      for (int step = 0; step < 120 && t_ok < 1.0; ++step) {
        const double t = (step == 0) ? 0.0 : std::min(1.0, t_ok + dt);
        HybridPowerSystem sys_t = sys_work;
        for (auto& g : sys_t.ac.generators) {
          g.cost_c0 *= t;
          g.cost_c1 *= t;
          g.cost_c2 *= t;
        }
        ACOPFOptions opt_t = opt;
        opt_t.compute_homotopy_tangent = true;
        opt_t.homotopy_t = t;
        // Generalize the homotopy to non-economic objectives (RPO): the
        // voltage-deviation and active-loss objective weights are scaled by t
        // alongside the generation costs, so t = 0 is a pure feasibility
        // problem for EVERY objective class and the path is meaningful for
        // the RPO baseline, not only for economic dispatch.
        opt_t.voltage_deviation_weight *= t;
        opt_t.active_loss_weight *= t;
        if (step == 0) {
          opt_t.ac_pf_warm_start = true;
        } else if (have_prediction) {
          opt_t.warm_start = &predicted;
        } else if (!chain.empty()) {
          opt_t.warm_start = &chain.back();
        }
        ACOPFResult r_t =
            solve_with_parity_ipm(sys_t, opt_t, inner, prepared);
        if (r_t.converged) {
          const bool fast = r_t.iterations <= 20;
          chain.push_back(std::move(r_t));
          t_ok = t;
          if (fast) dt = std::min(dt * 2.0, 0.5);
          // Davidenko predictor for the next step: w_pred = w(t) + h·w′(t),
          // h capped by the parameter-metric trust radius κ/‖dx/dt‖∞.
          predicted = ACOPFResult();
          have_prediction = false;
          const ACOPFResult& rb = chain.back();
          if (!rb.ipm_tangent_primal.empty() && t < 1.0) {
            double w_norm = 0.0;
            for (double v : rb.ipm_tangent_primal) {
              w_norm = std::max(w_norm, std::abs(v));
            }
            constexpr double kTrustRadius = 0.05;
            const double h = std::min(dt, kTrustRadius / std::max(w_norm, 1e-30));
            const auto adv = [&](const std::vector<double>& w,
                                 const std::vector<double>& dw, double floor) {
              std::vector<double> out_v(w.size());
              for (size_t i = 0; i < w.size(); ++i) {
                out_v[i] = std::max(w[i] + h * dw[i], floor);
              }
              return out_v;
            };
            const auto adv_free = [&](const std::vector<double>& w,
                                      const std::vector<double>& dw) {
              std::vector<double> out_v(w.size());
              for (size_t i = 0; i < w.size(); ++i) {
                out_v[i] = w[i] + h * dw[i];
              }
              return out_v;
            };
            predicted.ipm_primal_state =
                adv_free(rb.ipm_primal_state, rb.ipm_tangent_primal);
            predicted.ipm_slack_state =
                adv(rb.ipm_slack_state, rb.ipm_tangent_slack, 1e-8);
            predicted.ipm_equality_dual_state =
                adv_free(rb.ipm_equality_dual_state, rb.ipm_tangent_equality_dual);
            predicted.ipm_inequality_dual_state =
                adv(rb.ipm_inequality_dual_state, rb.ipm_tangent_inequality_dual, 1e-8);
            have_prediction = true;
          }
        } else {
          have_prediction = false;
          dt *= 0.5;
          if (dt < 0.005) {
            break;  // minimal-violation region reached — report honestly below
          }
        }
      }
      if (chain.empty()) {
        out.status =
            "AC OPF failed: objective homotopy could not find a feasible point "
            "(Phase I diverged).";
        return out;
      }
      if (t_ok < 1.0) {
        ACOPFResult best = std::move(chain.back());
        // A feasible Phase-I point is not an optimal solution for the
        // requested objective.  Give the full target one final chance from
        // that point before reporting the homotopy as incomplete.
        if (opt.allow_fallback) {
          ACOPFOptions full_target_opt = opt;
          full_target_opt.objective_homotopy = false;
          // This is outside the Native-only homotopy loop.  Run the standalone
          // recovery from a fresh coupled PF point; carrying the Phase-I
          // primal into Ipopt can pin it to the zero-objective face and is not
          // needed for the independent full-objective solve.
          full_target_opt.ac_pf_warm_start = true;
          full_target_opt.warm_start = nullptr;
          ACOPFResult full_target =
              solve_with_parity_ipm(sys_work, full_target_opt, inner, prepared);
          if (full_target.converged) {
            full_target.status =
                "converged (full objective after incomplete objective homotopy)";
            return separate_external_grid_dispatch(std::move(full_target));
          }
        }
        best.converged = false;
        best.status =
            "not converged: objective homotopy stopped at t=" +
            std::to_string(t_ok) +
            " (minimal-violation region; full objective not solved)";
        return separate_external_grid_dispatch(std::move(best));
      }
      // t = 1 was solved in-loop as a normal step; its result is the answer.
      return separate_external_grid_dispatch(std::move(chain.back()));
    }
    if (opt.objective_homotopy && inner == ParityInnerSolver::Ipopt) {
      ACOPFOptions standalone_opt = opt;
      standalone_opt.objective_homotopy = false;
      return separate_external_grid_dispatch(
          solve_with_parity_ipm(sys_work, standalone_opt, inner, prepared));
    }
    // The parity path internally applies the Ipopt fallback when inner == Auto
    // and opt.allow_fallback is set, so no separate recursive economic-dispatch
    // fallback is needed here.
    return separate_external_grid_dispatch(
        solve_with_parity_ipm(sys_work, opt, inner, prepared));
  }

  HybridPowerSystem ac_only = sys_work;
  ac_only.dc.buses.clear();
  ac_only.dc.branches.clear();
  ac_only.vsc_converters.clear();

  if (!opt.enable_primal_dual) {
    if (dropped_dc) {
      out.status = "AC OPF failed: hybrid AC/DC/LCC cases require a hybrid-capable primal-dual solver; "
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
      return separate_external_grid_dispatch(std::move(out));
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
        if (!std::isfinite(width)) {
          if (std::isfinite(lower[i]) && x[i] < lower[i]) x[i] = lower[i];
          if (std::isfinite(upper[i]) && x[i] > upper[i]) x[i] = upper[i];
        } else if (!(width > 0.0)) {
          x[i] = 0.5 * (lower[i] + upper[i]);
        } else {
          const double frac = constrained_var_col[static_cast<size_t>(i)] ? 1e-3 : 1e-8;
          const double eps = std::min(std::max(1e-10, frac * width), 0.49 * width);
          x[i] = std::clamp(x[i], lower[i] + eps, upper[i] - eps);
        }
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
  set_generator_result_map(out, data);

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
      return separate_external_grid_dispatch(std::move(fallback));
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

  if (data.bus_merge_map) {
    unproject_per_bus_ac_opf_result(out, *data.bus_merge_map);
  }
  trim_internal_dc_bus_results(out, data.projection_certificate);

  return separate_external_grid_dispatch(std::move(out));
}

ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                         const ACOPFOptions& options) {
  return solve_ac_opf_impl(sys, options, nullptr);
}

PreparedACOPFSession::PreparedACOPFSession(ACOPFOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
PreparedACOPFSession::~PreparedACOPFSession() = default;
PreparedACOPFSession::PreparedACOPFSession(PreparedACOPFSession&&) noexcept =
    default;
PreparedACOPFSession& PreparedACOPFSession::operator=(
    PreparedACOPFSession&&) noexcept = default;

ACOPFResult PreparedACOPFSession::solve(const HybridPowerSystem& sys) {
  ACOPFOptions options = impl_->state.options;
  if (options.warm_start == nullptr && impl_->state.has_previous_result) {
    options.warm_start = &impl_->state.previous_result;
  }
  ACOPFResult result = solve_ac_opf_impl(sys, options, &impl_->state);
  if (result.ipm_layout_signature != 0) {
    impl_->state.previous_result = result;
    impl_->state.has_previous_result = true;
  }
  return result;
}

void PreparedACOPFSession::reset() {
  ACOPFOptions options = impl_->state.options;
  impl_ = std::make_unique<Impl>(std::move(options));
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
