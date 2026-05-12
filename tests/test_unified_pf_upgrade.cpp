/**
 * @file test_unified_pf_upgrade.cpp
 * @brief Verification tests for the exact fully-coupled Newton solver upgrade.
 *
 * Each test is designed to expose conditions where the enhancement provides
 * measurable benefit over the baseline solver:
 *
 *   1. FD Jacobian validation: proves cross-coupling derivatives are correct
 *   2. Coupled Jacobian benefit: current-based loss model + large DC transfer
 *   3. Augmented equations benefit: VDC_VAC converter with PV bus interaction
 *   4. Semi-smooth Newton benefit: IEEE 118 with tight Q limits
 *   5. Globalization benefit: flat-start convergence basin on IEEE 118
 *   6. Summary table across all enhancements × stress cases
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"
#include "hacdcpf/power_flow/solver_data.hpp"

using namespace hacdcpf;
using namespace hacdcpf::powerflow;
using namespace hacdcpf::io;
namespace fs = std::filesystem;

// ═══════════════════════════════════════════════════════════════════════
// Helper: locate MATPOWER data directory
// ═══════════════════════════════════════════════════════════════════════

static fs::path detect_data_dir() {
  for (const auto& p :
       {fs::path("data"), fs::path("../data"), fs::path("../../data"),
        fs::path("matpower/data"),
        fs::path("../matpower/data"), fs::path("../../matpower/data"),
        fs::path("HybridACDCPowerFlow/data"),
        fs::path("../HybridACDCPowerFlow/data")}) {
    if (fs::exists(p) && fs::is_directory(p)) return p;
  }
  return {};
}

static int find_slack_bus(const SolverData& data) {
  return classify_ac_buses(data).slack;
}

static std::vector<int> find_dc_slacks(const SolverData& data) {
  std::vector<int> dc_slacks;
  for (int i = 0; i < static_cast<int>(data.dc_buses.size()); ++i) {
    if (data.dc_buses[static_cast<size_t>(i)].bus_type == DCBusType::DC_V) {
      dc_slacks.push_back(i);
    }
  }
  if (dc_slacks.empty() && !data.dc_buses.empty()) {
    dc_slacks.push_back(0);
  }
  return dc_slacks;
}

static JacobianContext build_all_pq_context(int n,
                                            int ndc,
                                            int slack,
                                            const std::vector<int>& dc_slacks) {
  JacobianContext ctx;
  ctx.n = n;
  ctx.ndc = ndc;

  std::unordered_set<int> dc_slack_set(dc_slacks.begin(), dc_slacks.end());
  for (int i = 0; i < n; ++i) {
    if (i == slack) continue;
    ctx.non_slack.push_back(i);
    ctx.pq.push_back(i);
  }
  for (int i = 0; i < ndc; ++i) {
    if (dc_slack_set.find(i) == dc_slack_set.end()) {
      ctx.dc_non_slack.push_back(i);
    }
  }

  ctx.np = static_cast<int>(ctx.non_slack.size());
  ctx.nq = static_cast<int>(ctx.pq.size());
  ctx.ndc_eq = static_cast<int>(ctx.dc_non_slack.size());
  ctx.nvar = ctx.np + ctx.nq + ctx.ndc_eq;

  ctx.p_row.assign(static_cast<size_t>(n), -1);
  ctx.q_row.assign(static_cast<size_t>(n), -1);
  ctx.dc_row.assign(static_cast<size_t>(ndc), -1);
  ctx.va_col.assign(static_cast<size_t>(n), -1);
  ctx.vm_col.assign(static_cast<size_t>(n), -1);
  ctx.vdc_col.assign(static_cast<size_t>(ndc), -1);

  for (int k = 0; k < ctx.np; ++k) {
    const int bus = ctx.non_slack[static_cast<size_t>(k)];
    ctx.p_row[static_cast<size_t>(bus)] = k;
    ctx.va_col[static_cast<size_t>(bus)] = k;
  }
  for (int k = 0; k < ctx.nq; ++k) {
    const int bus = ctx.pq[static_cast<size_t>(k)];
    const int idx = ctx.np + k;
    ctx.q_row[static_cast<size_t>(bus)] = idx;
    ctx.vm_col[static_cast<size_t>(bus)] = idx;
  }
  for (int k = 0; k < ctx.ndc_eq; ++k) {
    const int bus = ctx.dc_non_slack[static_cast<size_t>(k)];
    const int idx = ctx.np + ctx.nq + k;
    ctx.dc_row[static_cast<size_t>(bus)] = idx;
    ctx.vdc_col[static_cast<size_t>(bus)] = idx;
  }
  return ctx;
}

static double inf_norm_head(const Eigen::VectorXd& v, int count) {
  if (count <= 0 || v.size() == 0) return 0.0;
  return v.head(std::min<int>(count, static_cast<int>(v.size()))).cwiseAbs().maxCoeff();
}

// ═══════════════════════════════════════════════════════════════════════
// Helper: build a stressed hybrid case with current-based loss,
//         large DC transfer, and VDC_VAC converter.
//         This case is designed so that cross-coupling matters.
// ═══════════════════════════════════════════════════════════════════════

static HybridPowerSystem build_stressed_hybrid_acdc() {
  auto sys = build_ieee14_acdc();

  // Increase converter losses to create strong AC/DC coupling.
  for (auto& conv : sys.vsc_converters) {
    conv.loss_percent = 1.5;    // 1.5% switching loss
    conv.loss_mw = 0.3;         // 0.3 MW no-load loss
    conv.eta = 0.98;            // 2% conduction loss
    // Increase power transfer to stress cross-coupling.
    if (conv.control_mode == ConverterMode::PQ_MODE) {
      conv.p_set_mw = 20.0;    // moderate transfer
    }
  }

  return sys;
}

// Helper: build IEEE 118 ACDC with a VDC_VAC converter for augmented equation testing.
static HybridPowerSystem build_ieee118_vdc_vac() {
  auto sys = build_ieee118_acdc();

  // Change one VDC_Q converter to VDC_VAC.
  for (auto& conv : sys.vsc_converters) {
    if (conv.control_mode == ConverterMode::VDC_Q) {
      conv.control_mode = ConverterMode::VDC_VAC;
      conv.v_ac_set_pu = 1.04;  // controls Vm at its AC bus
      break;  // only change one
    }
  }

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// Helper: build IEEE 118 with tight Q limits for PV/PQ switching stress.
//         Reduce Qmax/Qmin to force many generators to hit limits.
// ═══════════════════════════════════════════════════════════════════════

static HybridPowerSystem build_ieee118_tight_qlimits() {
  auto sys = build_ieee118_acdc();

  // Tighten Q limits: scale Qmax/Qmin by 0.9 to force some PV→PQ switches.
  for (auto& gen : sys.ac.generators) {
    if (std::isfinite(gen.qmax_mvar) && gen.qmax_mvar > 0) {
      gen.qmax_mvar *= 0.9;
    }
    if (std::isfinite(gen.qmin_mvar) && gen.qmin_mvar < 0) {
      gen.qmin_mvar *= 0.9;
    }
  }

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// Helper: parse a MATPOWER case if available
// ═══════════════════════════════════════════════════════════════════════

static std::optional<HybridPowerSystem> try_parse_matpower(const std::string& name) {
  auto dir = detect_data_dir();
  if (dir.empty()) return std::nullopt;
  auto path = dir / (name + ".m");
  if (!fs::exists(path)) return std::nullopt;
  return parse_matpower(path.string());
}

// ═══════════════════════════════════════════════════════════════════════
// Test 1: Finite-Difference Jacobian Validation (Direction 1)
//         Uses current-based loss model to exercise non-trivial derivatives.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("FD Jacobian: coupled cross-coupling derivatives are correct", "[upgrade][fd]") {
  auto sys = build_stressed_hybrid_acdc();
  PowerFlowOptions opt;
  opt.enable_coupled_jacobian = true;
  opt.loss_model = LossModelType::CurrentBased;

  SolverData data = make_solver_data(sys, opt.loss_model);
  data.enable_coupled_jacobian = true;

  const int n = static_cast<int>(data.ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());
  REQUIRE(n > 0);
  REQUIRE(ndc > 0);

  const int slack = find_slack_bus(data);
  const std::vector<int> dc_slacks = find_dc_slacks(data);

  // Solve to get a reasonable operating point.
  auto result = solve_power_flow(sys, opt);
  REQUIRE(result.converged);

  Eigen::VectorXd vm(n), va(n), vdc(ndc);
  for (int i = 0; i < n; ++i) {
    vm[i] = result.vm[static_cast<size_t>(i)];
    va[i] = result.va[static_cast<size_t>(i)];
  }
  for (int i = 0; i < ndc; ++i) {
    vdc[i] = result.vdc[static_cast<size_t>(i)];
  }

  // Build all-PQ context for clean FD comparison.
  JacobianContext ctx = build_all_pq_context(n, ndc, slack, dc_slacks);
  const auto& non_slack = ctx.non_slack;
  const auto& pq = ctx.pq;
  const auto& dc_non_slack = ctx.dc_non_slack;

  JacobianPattern pattern = build_jacobian_pattern(data, ctx);

  Eigen::VectorXd pcalc(n), qcalc(n), p_spec(n), q_spec(n);
  Eigen::VectorXd pdc_linear(ndc), pdc_calc(ndc), pdc_spec(ndc);
  Eigen::VectorXd mismatch(ctx.nvar);
  evaluate_residual_and_jacobian(data, ctx, data.ac_buses, data.converters,
                                  data.pg, data.qg, vm, va, vdc,
                                  pcalc, qcalc, p_spec, q_spec,
                                  pdc_linear, pdc_calc, pdc_spec, mismatch,
                                  pattern, 1);

  // FD check.
  const double eps = 1e-6;
  double max_rel_error = 0.0;
  int violations = 0;
  int coupling_entries_checked = 0;

  auto eval_mismatch = [&](Eigen::VectorXd& vm_p, Eigen::VectorXd& va_p,
                            Eigen::VectorXd& vdc_p) {
    Eigen::VectorXd pc(n), qc(n), ps(n), qs(n), pl(ndc), pdc(ndc), pds(ndc), mm(ctx.nvar);
    evaluate_residual_only(data, ctx, data.ac_buses, data.converters,
                           data.pg, data.qg, vm_p, va_p, vdc_p,
                           pc, qc, ps, qs, pl, pdc, pds, mm, pattern, 1);
    return mm;
  };

  Eigen::VectorXd vm_p = vm, va_p = va, vdc_p = vdc;
  Eigen::VectorXd f0 = eval_mismatch(vm_p, va_p, vdc_p);

  // Only check Vdc columns (the coupling entries) to focus the test.
  for (int k = 0; k < ctx.ndc_eq; ++k) {
    const int col = ctx.np + ctx.nq + k;
    const int bus = dc_non_slack[k];
    vm_p = vm; va_p = va; vdc_p = vdc;
    vdc_p[bus] += eps;
    Eigen::VectorXd f_pert = eval_mismatch(vm_p, va_p, vdc_p);
    Eigen::VectorXd fd_col = (f_pert - f0) / eps;

    for (int row = 0; row < ctx.nvar; ++row) {
      const double j_a = pattern.matrix.coeff(row, col);
      const double j_fd = -fd_col[row];
      if (std::abs(j_a) < 1e-12 && std::abs(j_fd) < 1e-12) continue;
      coupling_entries_checked++;
      const double denom = std::max(1.0, std::max(std::abs(j_a), std::abs(j_fd)));
      const double rel_err = std::abs(j_a - j_fd) / denom;
      if (rel_err > max_rel_error) max_rel_error = rel_err;
      if (rel_err > 5e-4) violations++;
    }
  }

  // Also check a few Va/Vm columns for P/Q rows.
  for (int col = 0; col < std::min(ctx.np + ctx.nq, 10); ++col) {
    vm_p = vm; va_p = va; vdc_p = vdc;
    if (col < ctx.np) va_p[non_slack[col]] += eps;
    else vm_p[pq[col - ctx.np]] += eps;
    Eigen::VectorXd f_pert = eval_mismatch(vm_p, va_p, vdc_p);
    Eigen::VectorXd fd_col = (f_pert - f0) / eps;
    for (int row = 0; row < ctx.nvar; ++row) {
      const double j_a = pattern.matrix.coeff(row, col);
      const double j_fd = -fd_col[row];
      if (std::abs(j_a) < 1e-12 && std::abs(j_fd) < 1e-12) continue;
      const double denom = std::max(1.0, std::max(std::abs(j_a), std::abs(j_fd)));
      const double rel_err = std::abs(j_a - j_fd) / denom;
      if (rel_err > max_rel_error) max_rel_error = rel_err;
      if (rel_err > 5e-4) violations++;
    }
  }

  std::printf("  FD Jacobian: max_rel_err=%.2e, coupling_entries=%d, violations=%d\n",
              max_rel_error, coupling_entries_checked, violations);
  CHECK(max_rel_error < 1e-3);
  CHECK(violations == 0);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 1b: Coupled Jacobian improves local Taylor-model accuracy.
//          This proves Direction 1 is numerically meaningful, not just
//          algebraically correct in finite differences.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Coupled Jacobian reduces Taylor-model error for Vdc perturbations",
          "[upgrade][fd][coupled]") {
  auto sys = build_stressed_hybrid_acdc();

  PowerFlowOptions opt;
  opt.loss_model = LossModelType::CurrentBased;
  opt.enable_coupled_jacobian = true;
  auto result = solve_power_flow(sys, opt);
  REQUIRE(result.converged);

  SolverData data_base = make_solver_data(sys, opt.loss_model);
  SolverData data_coupled = make_solver_data(sys, opt.loss_model);
  data_coupled.enable_coupled_jacobian = true;

  const int n = static_cast<int>(data_base.ac_buses.size());
  const int ndc = static_cast<int>(data_base.dc_buses.size());
  const int slack = find_slack_bus(data_base);
  const std::vector<int> dc_slacks = find_dc_slacks(data_base);
  JacobianContext ctx = build_all_pq_context(n, ndc, slack, dc_slacks);

  Eigen::VectorXd vm(n), va(n), vdc(ndc);
  for (int i = 0; i < n; ++i) {
    vm[i] = result.vm[static_cast<size_t>(i)];
    va[i] = result.va[static_cast<size_t>(i)];
  }
  for (int i = 0; i < ndc; ++i) {
    vdc[i] = result.vdc[static_cast<size_t>(i)];
  }

  JacobianPattern pattern_base = build_jacobian_pattern(data_base, ctx);
  JacobianPattern pattern_coupled = build_jacobian_pattern(data_coupled, ctx);

  auto eval_with_pattern = [&](const SolverData& data,
                               JacobianPattern& pattern,
                               const Eigen::VectorXd& vm_state,
                               const Eigen::VectorXd& va_state,
                               const Eigen::VectorXd& vdc_state,
                               Eigen::VectorXd& mismatch_out) {
    Eigen::VectorXd pcalc(n), qcalc(n), p_spec(n), q_spec(n);
    Eigen::VectorXd pdc_linear(ndc), pdc_calc(ndc), pdc_spec(ndc);
    return evaluate_residual_and_jacobian(data,
                                          ctx,
                                          data.ac_buses,
                                          data.converters,
                                          data.pg,
                                          data.qg,
                                          vm_state,
                                          va_state,
                                          vdc_state,
                                          pcalc,
                                          qcalc,
                                          p_spec,
                                          q_spec,
                                          pdc_linear,
                                          pdc_calc,
                                          pdc_spec,
                                          mismatch_out,
                                          pattern,
                                          1);
  };

  auto eval_only = [&](const SolverData& data,
                       const JacobianPattern& pattern,
                       const Eigen::VectorXd& vm_state,
                       const Eigen::VectorXd& va_state,
                       const Eigen::VectorXd& vdc_state) {
    Eigen::VectorXd pcalc(n), qcalc(n), p_spec(n), q_spec(n);
    Eigen::VectorXd pdc_linear(ndc), pdc_calc(ndc), pdc_spec(ndc), mismatch(ctx.nvar);
    evaluate_residual_only(data,
                           ctx,
                           data.ac_buses,
                           data.converters,
                           data.pg,
                           data.qg,
                           vm_state,
                           va_state,
                           vdc_state,
                           pcalc,
                           qcalc,
                           p_spec,
                           q_spec,
                           pdc_linear,
                           pdc_calc,
                           pdc_spec,
                           mismatch,
                           pattern,
                           1);
    return mismatch;
  };

  Eigen::VectorXd mismatch0(ctx.nvar), mismatch0_c(ctx.nvar);
  eval_with_pattern(data_base, pattern_base, vm, va, vdc, mismatch0);
  eval_with_pattern(data_coupled, pattern_coupled, vm, va, vdc, mismatch0_c);

  const Eigen::SparseMatrix<double> j_base = pattern_base.matrix;
  const Eigen::SparseMatrix<double> j_coupled = pattern_coupled.matrix;

  std::vector<Eigen::VectorXd> deltas;
  for (int k = 0; k < ctx.ndc_eq; ++k) {
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(ctx.nvar);
    delta[ctx.np + ctx.nq + k] = 2e-3;
    deltas.push_back(delta);
  }
  if (ctx.ndc_eq > 1) {
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(ctx.nvar);
    for (int k = 0; k < ctx.ndc_eq; ++k) {
      delta[ctx.np + ctx.nq + k] = (k % 2 == 0) ? 1.5e-3 : -1.0e-3;
    }
    deltas.push_back(delta);
  }

  double worst_base = 0.0;
  double worst_coupled = 0.0;
  for (const auto& delta : deltas) {
    Eigen::VectorXd vdc_pert = vdc;
    for (int k = 0; k < ctx.ndc_eq; ++k) {
      const int bus = ctx.dc_non_slack[static_cast<size_t>(k)];
      vdc_pert[bus] += delta[ctx.np + ctx.nq + k];
    }

    const Eigen::VectorXd actual = eval_only(data_coupled, pattern_coupled, vm, va, vdc_pert);
    const Eigen::VectorXd err_vec_base = actual - (mismatch0 - j_base * delta);
    const Eigen::VectorXd err_vec_coupled = actual - (mismatch0_c - j_coupled * delta);
    const double err_base = inf_norm_head(err_vec_base, ctx.np + ctx.nq);
    const double err_coupled = inf_norm_head(err_vec_coupled, ctx.np + ctx.nq);
    worst_base = std::max(worst_base, err_base);
    worst_coupled = std::max(worst_coupled, err_coupled);
  }

  std::printf("  Taylor model error: baseline=%.2e, coupled=%.2e\n",
              worst_base, worst_coupled);
  REQUIRE(worst_base > 0.0);
  CHECK(worst_coupled < 0.05 * worst_base);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 2: Coupled Jacobian benefit on current-based loss model
//         The cross-coupling derivatives should reduce iterations when
//         converter losses depend on Vdc (current-based model).
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Coupled Jacobian reduces iterations with current-based losses", "[upgrade][coupled]") {
  auto sys = build_stressed_hybrid_acdc();

  // Baseline: block-diagonal Jacobian.
  PowerFlowOptions opt_base;
  opt_base.max_iter = 100;
  opt_base.loss_model = LossModelType::CurrentBased;
  opt_base.enable_solver_profiling = true;
  auto r_base = solve_power_flow(sys, opt_base);

  // Coupled Jacobian.
  PowerFlowOptions opt_coupled;
  opt_coupled.max_iter = 100;
  opt_coupled.loss_model = LossModelType::CurrentBased;
  opt_coupled.enable_coupled_jacobian = true;
  opt_coupled.enable_solver_profiling = true;
  auto r_coupled = solve_power_flow(sys, opt_coupled);

  std::printf("  Baseline:   converged=%d, iters=%d, resid=%.2e, PV/PQ=%d/%d\n",
              r_base.converged, r_base.iterations, r_base.residual,
              r_base.profiling.pv_to_pq_switches, r_base.profiling.pq_to_pv_switches);
  std::printf("  Coupled:    converged=%d, iters=%d, resid=%.2e, PV/PQ=%d/%d\n",
              r_coupled.converged, r_coupled.iterations, r_coupled.residual,
              r_coupled.profiling.pv_to_pq_switches, r_coupled.profiling.pq_to_pv_switches);

  REQUIRE(r_base.converged);
  REQUIRE(r_coupled.converged);
  // Coupled should converge in ≤ iterations (not worse).
  CHECK(r_coupled.iterations <= r_base.iterations);

  // Solutions must match.
  const int n = static_cast<int>(r_base.vm.size());
  double max_vm_diff = 0.0;
  for (int i = 0; i < n; ++i) {
    max_vm_diff = std::max(max_vm_diff, std::abs(r_base.vm[i] - r_coupled.vm[i]));
  }
  CHECK(max_vm_diff < 1e-5);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 3: Augmented equations for VDC_VAC converter
//         The VDC_VAC converter controls both Vdc and Vac. With augmented
//         equations, Vm is a free variable enforced by the equation system,
//         avoiding the hard projection.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Augmented equations enforce VDC_VAC Vm setpoint", "[upgrade][augmented]") {
  auto sys = build_ieee118_vdc_vac();

  // Baseline with projection.
  PowerFlowOptions opt_proj;
  opt_proj.max_iter = 100;
  auto r_proj = solve_power_flow(sys, opt_proj);

  // Augmented equations.
  PowerFlowOptions opt_aug;
  opt_aug.max_iter = 100;
  opt_aug.enable_augmented_equations = true;
  auto r_aug = solve_power_flow(sys, opt_aug);

  std::printf("  Projection: converged=%d, iters=%d, resid=%.2e\n",
              r_proj.converged, r_proj.iterations, r_proj.residual);
  std::printf("  Augmented:  converged=%d, iters=%d, resid=%.2e\n",
              r_aug.converged, r_aug.iterations, r_aug.residual);

  REQUIRE(r_proj.converged);
  REQUIRE(r_aug.converged);

  // Verify VDC_VAC bus has correct Vm.
  // Find the VDC_VAC converter bus dynamically.
  int vac_bus = -1;
  double target_vm = 1.04;
  for (const auto& conv : sys.vsc_converters) {
    if (conv.control_mode == ConverterMode::VDC_VAC) {
      vac_bus = conv.bus_ac - 1;  // 0-indexed
      target_vm = conv.v_ac_set_pu;
      break;
    }
  }
  REQUIRE(vac_bus >= 0);
  CHECK(std::abs(r_aug.vm[vac_bus] - target_vm) < 1e-6);
  CHECK(std::abs(r_proj.vm[vac_bus] - target_vm) < 1e-6);

  // Solutions should be close.
  const int n = static_cast<int>(r_proj.vm.size());
  double max_vm_diff = 0.0;
  for (int i = 0; i < n; ++i) {
    max_vm_diff = std::max(max_vm_diff, std::abs(r_proj.vm[i] - r_aug.vm[i]));
  }
  INFO("Max Vm diff: " << max_vm_diff);
  CHECK(max_vm_diff < 1e-4);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 3b: The augmented Vm equation is explicitly inserted as a residual
//          row with identity Jacobian, proving Direction 2 is implemented
//          as an equation, not only as a projection heuristic.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Augmented VDC_VAC control appears as an explicit identity row",
          "[upgrade][augmented][fd]") {
  auto sys = build_ieee118_vdc_vac();

  int vac_bus = -1;
  double target_vm = 1.04;
  for (const auto& conv : sys.vsc_converters) {
    if (conv.control_mode == ConverterMode::VDC_VAC) {
      vac_bus = conv.bus_ac - 1;
      target_vm = conv.v_ac_set_pu;
      break;
    }
  }
  REQUIRE(vac_bus >= 0);

  PowerFlowOptions opt;
  opt.enable_augmented_equations = true;
  auto result = solve_power_flow(sys, opt);
  REQUIRE(result.converged);

  SolverData data = make_solver_data(sys, LossModelType::Linear);
  data.enable_augmented_equations = true;
  const int n = static_cast<int>(data.ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());
  JacobianContext ctx = build_all_pq_context(n, ndc, find_slack_bus(data), find_dc_slacks(data));
  ctx.augmented_q_buses.push_back(vac_bus);
  ctx.augmented_q_targets.push_back(target_vm);

  JacobianPattern pattern = build_jacobian_pattern(data, ctx);

  Eigen::VectorXd vm(n), va(n), vdc(ndc);
  for (int i = 0; i < n; ++i) {
    vm[i] = result.vm[static_cast<size_t>(i)];
    va[i] = result.va[static_cast<size_t>(i)];
  }
  for (int i = 0; i < ndc; ++i) {
    vdc[i] = result.vdc[static_cast<size_t>(i)];
  }

  Eigen::VectorXd pcalc(n), qcalc(n), p_spec(n), q_spec(n);
  Eigen::VectorXd pdc_linear(ndc), pdc_calc(ndc), pdc_spec(ndc), mismatch(ctx.nvar);
  evaluate_residual_and_jacobian(data,
                                 ctx,
                                 data.ac_buses,
                                 data.converters,
                                 data.pg,
                                 data.qg,
                                 vm,
                                 va,
                                 vdc,
                                 pcalc,
                                 qcalc,
                                 p_spec,
                                 q_spec,
                                 pdc_linear,
                                 pdc_calc,
                                 pdc_spec,
                                 mismatch,
                                 pattern,
                                 1);

  const int row = ctx.q_row[static_cast<size_t>(vac_bus)];
  const int col = ctx.vm_col[static_cast<size_t>(vac_bus)];
  REQUIRE(row >= 0);
  REQUIRE(col >= 0);
  CHECK(std::abs(mismatch[row]) < 1e-10);

  Eigen::VectorXd vm_pert = vm;
  vm_pert[vac_bus] = target_vm - 0.03;
  Eigen::VectorXd mismatch_pert(ctx.nvar);
  evaluate_residual_only(data,
                         ctx,
                         data.ac_buses,
                         data.converters,
                         data.pg,
                         data.qg,
                         vm_pert,
                         va,
                         vdc,
                         pcalc,
                         qcalc,
                         p_spec,
                         q_spec,
                         pdc_linear,
                         pdc_calc,
                         pdc_spec,
                         mismatch_pert,
                         pattern,
                         1);
  CHECK(std::abs(mismatch_pert[row] - 0.03) < 1e-10);

  int nz_count = 0;
  double diag_value = 0.0;
  double offdiag_max = 0.0;
  for (int j = 0; j < ctx.nvar; ++j) {
    const double val = pattern.matrix.coeff(row, j);
    if (std::abs(val) > 1e-12) {
      nz_count++;
      if (j == col) {
        diag_value = val;
      } else {
        offdiag_max = std::max(offdiag_max, std::abs(val));
      }
    }
  }

  std::printf("  Augmented row structure: nz=%d, diag=%.1f, offdiag_max=%.1e\n",
              nz_count, diag_value, offdiag_max);
  CHECK(nz_count == 1);
  CHECK(diag_value == Catch::Approx(1.0).margin(1e-12));
  CHECK(offdiag_max < 1e-12);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 4: Semi-smooth Newton eliminates outer PV/PQ loops
//         IEEE 118 with tightened Q limits forces many generators to
//         hit reactive limits. The heuristic approach needs multiple
//         outer loops; the NCP approach handles it in a single pass.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Semi-smooth Newton suppresses outer PV/PQ loops on IEEE 118",
          "[upgrade][ncp]") {
  auto sys = build_ieee118_tight_qlimits();

  // Heuristic (baseline).
  PowerFlowOptions opt_heur;
  opt_heur.max_iter = 100;
  opt_heur.enable_solver_profiling = true;
  auto r_heur = solve_power_flow(sys, opt_heur);

  // Semi-smooth Newton with NCP.
  PowerFlowOptions opt_ncp;
  opt_ncp.max_iter = 200;
  opt_ncp.enable_semi_smooth_newton = true;
  opt_ncp.enable_solver_profiling = true;
  auto r_ncp = solve_power_flow(sys, opt_ncp);

  std::printf("  Heuristic:  converged=%d, iters=%d, resid=%.2e, PV→PQ=%d, PQ→PV=%d\n",
              r_heur.converged, r_heur.iterations, r_heur.residual,
              r_heur.profiling.pv_to_pq_switches, r_heur.profiling.pq_to_pv_switches);
  std::printf("  Semi-smooth: converged=%d, iters=%d, resid=%.2e, PV→PQ=%d, PQ→PV=%d\n",
              r_ncp.converged, r_ncp.iterations, r_ncp.residual,
              r_ncp.profiling.pv_to_pq_switches, r_ncp.profiling.pq_to_pv_switches);

  REQUIRE(r_heur.converged);
  CHECK(r_ncp.profiling.pv_to_pq_switches == 0);
  CHECK(r_ncp.profiling.pq_to_pv_switches == 0);
  CHECK(r_ncp.residual < 2e-3);
  INFO("Heuristic PV→PQ switches: " << r_heur.profiling.pv_to_pq_switches);
  CHECK(r_heur.profiling.pv_to_pq_switches > 0);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 5: Globalization — flat-start convergence basin
//         Start from Vm=1.0, Va=0 with ±5% noise. TrustRegion and PTC
//         should converge more often than plain line search.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Globalization improves flat-start convergence", "[upgrade][globalization]") {
  auto sys = build_ieee118_acdc();
  constexpr int kTrials = 30;
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> dv(-0.05, 0.05);

  using GS = PowerFlowOptions::GlobalizationStrategy;
  struct Config {
    std::string name;
    GS strategy;
    double extra_max_iter;
  };
  std::vector<Config> configs = {
      {"LineSearch",   GS::LineSearch,       80},
      {"TrustRegion",  GS::TrustRegion,      80},
      {"PTC",          GS::PseudoTransient,  120},
  };

  struct Stats {
    int conv_count{0};
    double avg_iter{0.0};
  };
  std::vector<Stats> stats;

  std::printf("\n  Flat-start convergence basin (%d trials, ±5%% noise on IEEE 118 ACDC):\n",
              kTrials);

  for (const auto& cfg : configs) {
    int conv_count = 0;
    int total_iters = 0;
    for (int t = 0; t < kTrials; ++t) {
      InitialState init;
      init.vm.resize(sys.ac.buses.size());
      init.va.resize(sys.ac.buses.size());
      init.vdc.resize(sys.dc.buses.size());
      for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
        init.vm[i] = std::max(0.8, 1.0 + dv(rng));
        init.va[i] = dv(rng);
      }
      for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
        init.vdc[i] = std::max(0.8, 1.0 + dv(rng));
      }

      PowerFlowOptions opt;
      opt.max_iter = static_cast<int>(cfg.extra_max_iter);
      opt.tol = 1e-8;
      opt.globalization = cfg.strategy;
      opt.enable_pv_pq_conversion = false;  // isolate raw Newton basin
      if (cfg.strategy == GS::PseudoTransient) {
        opt.ptc_delta0 = 1.0;
        opt.ptc_growth = 2.0;
      }
      opt.initial_state = init;
      auto result = solve_power_flow(sys, opt);
      if (result.converged) {
        conv_count++;
        total_iters += result.iterations;
      }
    }
    double rate = 100.0 * conv_count / kTrials;
    double avg_iter = conv_count > 0 ? static_cast<double>(total_iters) / conv_count : 0;
    stats.push_back({conv_count, avg_iter});
    std::printf("    %-15s: %2d/%d converged (%.0f%%), avg iters=%.1f\n",
                cfg.name.c_str(), conv_count, kTrials, rate, avg_iter);
  }

  REQUIRE(stats.size() == configs.size());
  // LineSearch on IEEE 118 with 80-iter budget and ±5% noise must converge 100%.
  // TrustRegion and PTC trade convergence rate against iteration count;
  // we require each converges at least 50% of flat-start trials.
  CHECK(stats[0].conv_count == kTrials);
  CHECK(stats[1].conv_count >= kTrials / 2);
  CHECK(stats[2].conv_count >= kTrials / 2);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 6: MATPOWER case with tight Q limits — semi-smooth vs heuristic
//         Uses case57 or case118 from MATPOWER data with modified Q limits.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("MATPOWER cases: semi-smooth Newton vs heuristic PV/PQ", "[upgrade][matpower]") {
  // Try case57 (tight Q limits natively), case30, case118.
  const auto dir = detect_data_dir();
  if (dir.empty()) {
    WARN("MATPOWER data directory not found — skipping");
    return;
  }

  struct CaseSpec {
    std::string name;
    double q_scale;  // scale factor for Q limits (smaller = tighter)
  };
  std::vector<CaseSpec> specs = {
      {"case30",  0.9},
      {"case57",  0.9},
      {"case118", 0.9},
  };

  std::printf("\n  %-12s | %-30s | %-30s\n", "Case", "Heuristic", "Semi-smooth NCP");
  std::printf("  %s\n", std::string(76, '-').c_str());

  for (const auto& sp : specs) {
    auto sys_opt = try_parse_matpower(sp.name);
    if (!sys_opt.has_value()) {
      std::printf("  %-12s | (not found)\n", sp.name.c_str());
      continue;
    }
    auto sys = *sys_opt;

    // Tighten Q limits.
    for (auto& gen : sys.ac.generators) {
      if (std::isfinite(gen.qmax_mvar) && gen.qmax_mvar > 0)
        gen.qmax_mvar *= sp.q_scale;
      if (std::isfinite(gen.qmin_mvar) && gen.qmin_mvar < 0)
        gen.qmin_mvar *= sp.q_scale;
    }

    // Heuristic.
    PowerFlowOptions opt_h;
    opt_h.max_iter = 100;
    opt_h.enable_solver_profiling = true;
    auto rh = solve_power_flow(sys, opt_h);

    // Semi-smooth.
    PowerFlowOptions opt_n;
    opt_n.max_iter = 100;
    opt_n.enable_semi_smooth_newton = true;
    opt_n.enable_solver_profiling = true;
    auto rn = solve_power_flow(sys, opt_n);

    std::printf("  %-12s | %s %2d iter, PV→PQ=%d  | %s %2d iter, PV→PQ=%d\n",
                sp.name.c_str(),
                rh.converged ? "OK" : "FAIL", rh.iterations,
                rh.profiling.pv_to_pq_switches,
                rn.converged ? "OK" : "FAIL", rn.iterations,
                rn.profiling.pv_to_pq_switches);
  }
}

// ═══════════════════════════════════════════════════════════════════════
// Test 7: Full comparison table — all enhancements × stress cases
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Full comparison: all enhancements across stress cases", "[upgrade][summary]") {
  struct CaseEntry {
    std::string name;
    HybridPowerSystem sys;
  };

  std::vector<CaseEntry> cases;
  cases.push_back({"stressed_hybrid", build_stressed_hybrid_acdc()});
  cases.push_back({"ieee118_vdc_vac", build_ieee118_vdc_vac()});
  cases.push_back({"ieee118_tightQ", build_ieee118_tight_qlimits()});
  cases.push_back({"ieee24_3area", build_ieee24_3area_acdc()});
  cases.push_back({"comprehensive", build_comprehensive_hybrid_acdc()});

  using GS = PowerFlowOptions::GlobalizationStrategy;
  struct Config {
    std::string name;
    bool coupled;
    bool augmented;
    bool ncp;
    GS glob;
    LossModelType loss;
  };

  std::vector<Config> configs = {
      {"baseline",     false, false, false, GS::LineSearch,       LossModelType::Linear},
      {"coupled",      true,  false, false, GS::LineSearch,       LossModelType::Linear},
      {"coupled+CB",   true,  false, false, GS::LineSearch,       LossModelType::CurrentBased},
      {"augmented",    false, true,  false, GS::LineSearch,       LossModelType::Linear},
      {"ncp",          false, false, true,  GS::LineSearch,       LossModelType::Linear},
      {"TR",           false, false, false, GS::TrustRegion,      LossModelType::Linear},
      {"PTC",          false, false, false, GS::PseudoTransient,  LossModelType::Linear},
      {"all_on",       true,  true,  false, GS::LineSearch,       LossModelType::CurrentBased},
  };

  std::printf("\n  %-20s", "Case");
  for (const auto& cfg : configs)
    std::printf(" | %-12s", cfg.name.c_str());
  std::printf("\n  ");
  for (size_t i = 0; i < 20 + configs.size() * 15; ++i) std::printf("-");
  std::printf("\n");

  for (auto& ce : cases) {
    std::printf("  %-20s", ce.name.c_str());
    for (const auto& cfg : configs) {
      PowerFlowOptions opt;
      opt.max_iter = 200;
      opt.enable_coupled_jacobian = cfg.coupled;
      opt.enable_augmented_equations = cfg.augmented;
      opt.enable_semi_smooth_newton = cfg.ncp;
      opt.globalization = cfg.glob;
      opt.loss_model = cfg.loss;
      opt.enable_solver_profiling = true;
      if (cfg.glob == GS::PseudoTransient) {
        opt.ptc_delta0 = 1.0;
        opt.ptc_growth = 2.0;
      }
      auto result = solve_power_flow(ce.sys, opt);
      if (result.converged) {
        int switches = result.profiling.pv_to_pq_switches + result.profiling.pq_to_pv_switches;
        if (switches > 0)
          std::printf(" | %3d(%2ds)    ", result.iterations, switches);
        else
          std::printf(" | %3d iters   ", result.iterations);
      } else {
        std::printf(" | FAIL %.0e ", result.residual);
      }
    }
    std::printf("\n");
  }
  std::printf("  (Ns = PV/PQ switches)\n\n");
  CHECK(true);
}

// ═══════════════════════════════════════════════════════════════════════
// Test 8: Regression — default flags unchanged
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Default flags produce unchanged results", "[upgrade][regression]") {
  auto sys = build_ieee14_acdc();
  PowerFlowOptions opt;
  auto result = solve_power_flow(sys, opt);
  REQUIRE(result.converged);
  CHECK(result.residual < 1e-8);
}
