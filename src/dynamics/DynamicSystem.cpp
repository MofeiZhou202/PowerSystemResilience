#include "hacdcpf/dynamics/DynamicSystem.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>

#include "hacdcpf/dynamics/solvers/SparseLinearSolver.hpp"

namespace hacdcpf::dynamics {

namespace {

using Complex = std::complex<double>;

double finite_or(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

// Exact structural + numeric equality of two compressed sparse matrices, used to
// decide whether a cached factorization can be reused. Cheap (O(nnz)) relative to
// a re-factorization.
template <typename T>
bool sparse_matrices_equal(const Eigen::SparseMatrix<T>& a,
                           const Eigen::SparseMatrix<T>& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols() || a.nonZeros() != b.nonZeros()) {
    return false;
  }
  const Eigen::Index nnz = a.nonZeros();
  const Eigen::Index outer = a.outerSize();
  for (Eigen::Index i = 0; i <= outer; ++i) {
    if (a.outerIndexPtr()[i] != b.outerIndexPtr()[i]) return false;
  }
  for (Eigen::Index i = 0; i < nnz; ++i) {
    if (a.innerIndexPtr()[i] != b.innerIndexPtr()[i]) return false;
    if (a.valuePtr()[i] != b.valuePtr()[i]) return false;
  }
  return true;
}

void mask_slow_residuals(const std::vector<std::unique_ptr<DynamicDevice>>& devices,
                         Eigen::VectorXd& dxdt) {
  for (const auto& device : devices) {
    device->maskSlowStateResidual(dxdt);
  }
}

std::vector<DynamicResidualDiagnostic> collect_residual_diagnostics(
    const std::vector<std::unique_ptr<DynamicDevice>>& devices,
    double t,
    const DynamicState& x,
    const NetworkState& y,
    double min_abs_residual) {
  std::vector<DynamicResidualDiagnostic> diagnostics;
  if (x.x.size() == 0) return diagnostics;
  for (const auto& device : devices) {
    Eigen::VectorXd contribution = Eigen::VectorXd::Zero(x.x.size());
    device->computeDerivatives(t, x, y, contribution);
    mask_slow_residuals(devices, contribution);
    if (contribution.size() == 0) continue;
    Eigen::Index idx = 0;
    const double max_abs = contribution.cwiseAbs().maxCoeff(&idx);
    if (!std::isfinite(max_abs) || max_abs < min_abs_residual) continue;
    DynamicResidualDiagnostic diag;
    diag.device_name = device->name();
    diag.device_type = device->type();
    diag.component_index = device->componentIndex();
    diag.state_index = static_cast<int>(idx);
    diag.residual = contribution[idx];
    diagnostics.push_back(std::move(diag));
  }
  std::sort(diagnostics.begin(),
            diagnostics.end(),
            [](const DynamicResidualDiagnostic& a, const DynamicResidualDiagnostic& b) {
              return std::abs(a.residual) > std::abs(b.residual);
            });
  if (diagnostics.size() > 8) diagnostics.resize(8);
  return diagnostics;
}

double inf_norm(const Eigen::VectorXd& v) {
  return v.size() > 0 ? v.lpNorm<Eigen::Infinity>() : 0.0;
}

bool evaluate_masked_dynamic_residual(DynamicSystem& sys,
                                      double t,
                                      const Eigen::VectorXd& state,
                                      Eigen::VectorXd& residual,
                                      std::string& error) {
  if (!sys.evaluateDerivatives(t, state, residual, error)) return false;
  mask_slow_residuals(sys.devices, residual);
  if (!residual.allFinite()) {
    error = "Dynamic residual contains non-finite values";
    return false;
  }
  return true;
}

bool numerical_masked_dynamic_jacobian(DynamicSystem& sys,
                                       double t,
                                       const Eigen::VectorXd& state,
                                       const Eigen::VectorXd& residual0,
                                       Eigen::MatrixXd& jac,
                                       std::string& error) {
  const Eigen::Index n = state.size();
  jac = Eigen::MatrixXd::Zero(n, n);
  if (n == 0) return true;
  const double eps0 = std::sqrt(std::numeric_limits<double>::epsilon());
  for (Eigen::Index col = 0; col < n; ++col) {
    Eigen::VectorXd trial = state;
    const double h = eps0 * std::max(1.0, std::abs(state[col]));
    trial[col] += h;
    Eigen::VectorXd residual_p;
    if (!evaluate_masked_dynamic_residual(sys, t, trial, residual_p, error)) return false;
    jac.col(col) = (residual_p - residual0) / h;
  }
  Eigen::VectorXd restored;
  return evaluate_masked_dynamic_residual(sys, t, state, restored, error);
}

bool is_machine_controller(const DynamicDevice& device) {
  const std::string type = device.type();
  return type == "Governor" || type == "Exciter" || type == "PSS";
}

bool reanchor_machine_controllers(
    const std::vector<std::unique_ptr<DynamicDevice>>& devices,
    DynamicState& x,
    NetworkState& y) {
  bool changed = false;
  for (const auto& device : devices) {
    if (!is_machine_controller(*device)) continue;
    changed = device->trimToNetworkEquilibrium(x, y) || changed;
  }
  return changed;
}

struct ConsistentInitializationResult {
  bool converged{false};
  int iterations{0};
  double residual_norm{0.0};
  std::string message;
};

ConsistentInitializationResult solve_consistent_dynamic_initial_state(
    DynamicSystem& sys,
    double t,
    const Eigen::VectorXd& initial_state,
    double tolerance,
    int max_iterations) {
  ConsistentInitializationResult result;
  std::string error;
  Eigen::VectorXd state = initial_state;
  Eigen::VectorXd residual;
  if (!evaluate_masked_dynamic_residual(sys, t, state, residual, error)) {
    result.message = error;
    result.residual_norm = std::numeric_limits<double>::infinity();
    return result;
  }
  double norm = inf_norm(residual);
  result.residual_norm = norm;
  if (norm <= tolerance) {
    result.converged = true;
    sys.x.x = state;
    return result;
  }

  double lambda = 1e-8;
  const double min_alpha = std::max(1e-9, sys.options.newton_damping_min);
  const int max_iters = std::max(1, max_iterations);
  for (int iter = 0; iter < max_iters; ++iter) {
    Eigen::MatrixXd jac;
    if (!numerical_masked_dynamic_jacobian(sys, t, state, residual, jac, error)) {
      result.message = error;
      break;
    }
    auto try_delta = [&](Eigen::VectorXd delta) {
      if (!delta.allFinite()) return false;
      const double delta_inf = inf_norm(delta);
      if (delta_inf > 5.0) {
        delta *= 5.0 / delta_inf;
      }
      double alpha = 1.0;
      while (alpha >= min_alpha) {
        const Eigen::VectorXd trial = state + alpha * delta;
        if (!trial.allFinite()) {
          alpha *= 0.5;
          continue;
        }
        Eigen::VectorXd trial_residual;
        if (!evaluate_masked_dynamic_residual(sys, t, trial, trial_residual, error)) {
          alpha *= 0.5;
          continue;
        }
        const double trial_norm = inf_norm(trial_residual);
        if (trial_norm <=
            (1.0 - 1e-4 * alpha) * std::max(norm, tolerance)) {
          state = trial;
          residual = trial_residual;
          norm = trial_norm;
          return true;
        }
        alpha *= 0.5;
      }
      return false;
    };

    bool accepted =
        try_delta(jac.completeOrthogonalDecomposition().solve(-residual));
    if (accepted) {
      lambda = std::max(1e-12, lambda * 0.1);
      result.iterations = iter + 1;
      result.residual_norm = norm;
      if (norm <= tolerance) {
        result.converged = true;
        sys.x.x = state;
        return result;
      }
      continue;
    }

    const Eigen::MatrixXd jt = jac.transpose();
    const Eigen::MatrixXd jtj = jt * jac;
    const Eigen::VectorXd rhs = -jt * residual;
    const double diag_scale = std::max(1.0, jtj.diagonal().cwiseAbs().maxCoeff());

    for (int damp_try = 0; damp_try < 8 && !accepted; ++damp_try) {
      Eigen::MatrixXd a = jtj;
      a.diagonal().array() += lambda * diag_scale;
      Eigen::VectorXd delta = a.ldlt().solve(rhs);
      accepted = try_delta(delta);
      if (accepted) lambda = std::max(1e-12, lambda * 0.1);
      if (!accepted) lambda *= 10.0;
    }

    result.iterations = iter + 1;
    result.residual_norm = norm;
    if (norm <= tolerance) {
      result.converged = true;
      sys.x.x = state;
      return result;
    }
    if (!accepted) {
      result.message = "DAE consistent-initialization Newton line search failed";
      break;
    }
  }

  sys.x.x = state;
  Eigen::VectorXd final_residual;
  if (evaluate_masked_dynamic_residual(sys, t, state, final_residual, error)) {
    result.residual_norm = inf_norm(final_residual);
  }
  if (result.message.empty()) {
    result.message = "DAE consistent-initialization Newton did not converge";
  }
  return result;
}

}  // namespace

int DynamicNetwork::acBusPosition(int bus_id) const {
  const auto it = ac_bus_pos_by_id.find(bus_id);
  return it == ac_bus_pos_by_id.end() ? -1 : it->second;
}

int DynamicNetwork::dcBusPosition(int bus_id) const {
  const auto it = dc_bus_pos_by_id.find(bus_id);
  return it == dc_bus_pos_by_id.end() ? -1 : it->second;
}

void DynamicNetwork::rebuildBaseMatrices(double singular_regularization_pu) {
  std::vector<Eigen::Triplet<Complex>> y_triplets;
  y_triplets.reserve(ac_branches.size() * 36 + ac_bus_loads.size() * 3 + ac_bus_ids.size() * 3);

  for (const auto& branch : ac_branches) {
    if (!branch.in_service || branch.from_pos < 0 || branch.to_pos < 0) continue;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        const int fr = acPhaseNode(branch.from_pos, r);
        const int fc = acPhaseNode(branch.from_pos, c);
        const int tr = acPhaseNode(branch.to_pos, r);
        const int tc = acPhaseNode(branch.to_pos, c);
        if (std::abs(branch.y_ff(r, c)) != 0.0) y_triplets.emplace_back(fr, fc, branch.y_ff(r, c));
        if (std::abs(branch.y_ft(r, c)) != 0.0) y_triplets.emplace_back(fr, tc, branch.y_ft(r, c));
        if (std::abs(branch.y_tf(r, c)) != 0.0) y_triplets.emplace_back(tr, fc, branch.y_tf(r, c));
        if (std::abs(branch.y_tt(r, c)) != 0.0) y_triplets.emplace_back(tr, tc, branch.y_tt(r, c));
      }
    }
  }

  for (const auto& fault : fault_shunts) {
    if (!fault.active || !fault.is_ac || fault.bus_pos < 0) continue;
    const Complex y_fault(fault.g_pu, fault.b_pu);
    for (int phase = 0; phase < 3; ++phase) {
      const int node = acPhaseNode(fault.bus_pos, phase);
      y_triplets.emplace_back(node, node, y_fault);
    }
  }

  for (const auto& load : ac_bus_loads) {
    if (load.bus_pos < 0 || base_mva <= 0.0 || load.scale <= 0.0) continue;
    const Complex s_pu(load.p_mw / base_mva * load.scale,
                       load.q_mvar / base_mva * load.scale);
    const double vm0 = std::max(1e-6, std::abs(load.nominal_voltage_pu));
    const Complex y_load = std::conj(s_pu) / (3.0 * vm0 * vm0);
    for (int phase = 0; phase < 3; ++phase) {
      const int node = acPhaseNode(load.bus_pos, phase);
      y_triplets.emplace_back(node, node, y_load);
    }
  }

  if (singular_regularization_pu > 0.0) {
    for (int i = 0; i < acPhaseNodeCount(); ++i) {
      y_triplets.emplace_back(i, i, Complex(singular_regularization_pu, 0.0));
    }
  }

  Yac_base.resize(acPhaseNodeCount(), acPhaseNodeCount());
  Yac_base.setFromTriplets(y_triplets.begin(), y_triplets.end());

  std::vector<Eigen::Triplet<double>> g_triplets;
  g_triplets.reserve(dc_branches.size() * 4 + dc_bus_loads.size() + dc_bus_ids.size());
  for (const auto& branch : dc_branches) {
    if (!branch.in_service || branch.from_pos < 0 || branch.to_pos < 0 ||
        branch.conductance_pu == 0.0) {
      continue;
    }
    const double g = branch.conductance_pu;
    g_triplets.emplace_back(branch.from_pos, branch.from_pos, g);
    g_triplets.emplace_back(branch.to_pos, branch.to_pos, g);
    g_triplets.emplace_back(branch.from_pos, branch.to_pos, -g);
    g_triplets.emplace_back(branch.to_pos, branch.from_pos, -g);
  }

  for (const auto& fault : fault_shunts) {
    if (!fault.active || fault.is_ac || fault.bus_pos < 0) continue;
    g_triplets.emplace_back(fault.bus_pos, fault.bus_pos, fault.g_pu);
  }

  for (const auto& load : dc_bus_loads) {
    if (load.bus_pos < 0 || base_mva <= 0.0 || load.scale <= 0.0) continue;
    const double p_pu = load.p_mw / base_mva * load.scale;
    g_triplets.emplace_back(load.bus_pos, load.bus_pos, std::max(0.0, p_pu));
  }

  if (singular_regularization_pu > 0.0) {
    for (int i = 0; i < dcBusCount(); ++i) {
      g_triplets.emplace_back(i, i, singular_regularization_pu);
    }
  }

  Gdc_base.resize(dcBusCount(), dcBusCount());
  Gdc_base.setFromTriplets(g_triplets.begin(), g_triplets.end());
}

void DynamicNetwork::assembleEffectiveMatrices(
    const DynamicStamp& stamp,
    double singular_regularization_pu,
    Eigen::SparseMatrix<Complex>& Yac_eff,
    Eigen::VectorXcd& Iac_eff,
    Eigen::SparseMatrix<double>& Gdc_eff,
    Eigen::VectorXd& Idc_eff) const {
  std::vector<Eigen::Triplet<Complex>> y_triplets;
  y_triplets.reserve(static_cast<std::size_t>(Yac_base.nonZeros()) +
                     stamp.Yac_triplets.size() + static_cast<std::size_t>(acPhaseNodeCount()));
  for (int col = 0; col < Yac_base.outerSize(); ++col) {
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(Yac_base, col); it; ++it) {
      y_triplets.emplace_back(it.row(), it.col(), it.value());
    }
  }
  y_triplets.insert(y_triplets.end(), stamp.Yac_triplets.begin(), stamp.Yac_triplets.end());
  if (singular_regularization_pu > 0.0 && Yac_base.nonZeros() == 0) {
    for (int i = 0; i < acPhaseNodeCount(); ++i) {
      y_triplets.emplace_back(i, i, Complex(singular_regularization_pu, 0.0));
    }
  }
  Yac_eff.resize(acPhaseNodeCount(), acPhaseNodeCount());
  Yac_eff.setFromTriplets(y_triplets.begin(), y_triplets.end());
  Iac_eff = stamp.Iac;

  std::vector<Eigen::Triplet<double>> g_triplets;
  g_triplets.reserve(static_cast<std::size_t>(Gdc_base.nonZeros()) +
                     stamp.Gdc_triplets.size() + static_cast<std::size_t>(dcBusCount()));
  for (int col = 0; col < Gdc_base.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Gdc_base, col); it; ++it) {
      g_triplets.emplace_back(it.row(), it.col(), it.value());
    }
  }
  g_triplets.insert(g_triplets.end(), stamp.Gdc_triplets.begin(), stamp.Gdc_triplets.end());
  if (singular_regularization_pu > 0.0 && Gdc_base.nonZeros() == 0) {
    for (int i = 0; i < dcBusCount(); ++i) {
      g_triplets.emplace_back(i, i, singular_regularization_pu);
    }
  }
  Gdc_eff.resize(dcBusCount(), dcBusCount());
  Gdc_eff.setFromTriplets(g_triplets.begin(), g_triplets.end());
  Idc_eff = stamp.Idc;
}

void DynamicSystem::assignStateIndices() {
  int offset = 0;
  for (auto& device : devices) {
    device->assignStateIndices(offset);
  }
  x.resize(static_cast<std::size_t>(offset));
  y.resize(network.acPhaseNodeCount(), network.dcBusCount());
}

void DynamicSystem::initializeStatesFromPowerFlow() {
  if (x.size() == 0 && !devices.empty()) {
    assignStateIndices();
  }
  for (auto& device : devices) {
    device->initializeFromPowerFlow(initial_power_flow, x, y);
  }

  if (!options.trim_dynamic_initial_conditions) {
    std::string error;
    Eigen::VectorXd dxdt;
    if (evaluateDerivatives(options.t_start_s, x.x, dxdt, error)) {
      initialization.dynamic_initial_dxdt_inf_norm =
          dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
      mask_slow_residuals(devices, dxdt);
      initialization.dynamic_fast_dxdt_inf_norm =
          dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
      initialization.dynamic_residual_diagnostics =
          collect_residual_diagnostics(devices,
                                       options.t_start_s,
                                       x,
                                       y,
                                       std::max(1e-12, options.dynamic_trim_tol * 0.1));
    } else {
      initialization.warnings.push_back(error);
    }
    return;
  }

  std::string error;
  bool trimmed = false;
  double fast_norm = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < std::max(1, options.max_dynamic_trim_iters); ++iter) {
    if (!solveNetwork(options.t_start_s, error)) {
      initialization.warnings.push_back("Dynamic equilibrium trim skipped: " + error);
      break;
    }

    bool changed = false;
    for (auto& device : devices) {
      changed = device->trimToNetworkEquilibrium(x, y) || changed;
    }
    if (changed && !solveNetwork(options.t_start_s, error)) {
      initialization.warnings.push_back("Dynamic equilibrium trim network refresh failed: " +
                                       error);
      break;
    }
    reanchor_machine_controllers(devices, x, y);

    Eigen::VectorXd dxdt;
    if (!evaluateDerivatives(options.t_start_s, x.x, dxdt, error)) {
      initialization.warnings.push_back("Dynamic equilibrium trim residual unavailable: " + error);
      break;
    }
    x.dxdt = dxdt;
    initialization.dynamic_initial_dxdt_inf_norm =
        dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
    mask_slow_residuals(devices, dxdt);
    fast_norm = inf_norm(dxdt);
    initialization.dynamic_residual_diagnostics =
        collect_residual_diagnostics(devices,
                                     options.t_start_s,
                                     x,
                                     y,
                                     std::max(1e-12, options.dynamic_trim_tol * 0.1));
    initialization.dynamic_trim_iterations = iter + 1;
    if (fast_norm <= options.dynamic_trim_tol || !changed) {
      trimmed = fast_norm <= options.dynamic_trim_tol;
      break;
    }
  }

  if (!trimmed && std::isfinite(fast_norm)) {
    const ConsistentInitializationResult consistent =
        solve_consistent_dynamic_initial_state(*this,
                                               options.t_start_s,
                                               x.x,
                                               options.dynamic_trim_tol,
                                               options.max_dynamic_trim_iters);
    initialization.dynamic_trim_iterations += consistent.iterations;
    fast_norm = consistent.residual_norm;
    trimmed = consistent.converged || fast_norm <= options.dynamic_trim_tol;
    if (!trimmed && !consistent.message.empty()) {
      initialization.warnings.push_back(consistent.message +
                                       "; residual ||M f||_inf=" +
                                       std::to_string(fast_norm));
    }
  }

  Eigen::VectorXd final_dxdt;
  if (evaluateDerivatives(options.t_start_s, x.x, final_dxdt, error)) {
    x.dxdt = final_dxdt;
    initialization.dynamic_initial_dxdt_inf_norm =
        final_dxdt.size() > 0 ? final_dxdt.lpNorm<Eigen::Infinity>() : 0.0;
    mask_slow_residuals(devices, final_dxdt);
    fast_norm = inf_norm(final_dxdt);
    trimmed = fast_norm <= options.dynamic_trim_tol;
    initialization.dynamic_residual_diagnostics =
        collect_residual_diagnostics(devices,
                                     options.t_start_s,
                                     x,
                                     y,
                                     std::max(1e-12, options.dynamic_trim_tol * 0.1));
  } else if (!std::isfinite(fast_norm)) {
    fast_norm = derivativeInfinityNorm(options.t_start_s, error);
  }
  initialization.dynamic_fast_dxdt_inf_norm = std::isfinite(fast_norm) ? fast_norm : 0.0;
  initialization.dynamic_trim_converged = trimmed ||
      initialization.dynamic_fast_dxdt_inf_norm <= options.dynamic_trim_tol;
  if (!initialization.dynamic_trim_converged) {
    initialization.warnings.push_back(
        "Dynamic equilibrium trim did not fully converge; initial fast-state residual ||dx/dt||_inf=" +
        std::to_string(initialization.dynamic_fast_dxdt_inf_norm));
  }
}

bool DynamicSystem::acSolveCached(const Eigen::SparseMatrix<Complex>& a,
                                  const Eigen::VectorXcd& b, Eigen::VectorXcd& x,
                                  std::string& error) {
  if (options.linear_solver != DynamicLinearSolverType::EigenSparseLU) {
    SparseLinearSolver solver(options.linear_solver);
    const auto result = solver.solve(a, b, x);
    if (!result.success) error = result.message;
    return result.success && x.allFinite();
  }
  if (!network_cache) network_cache = std::make_unique<NetworkSolveCache>();
  auto& c = *network_cache;
  if (!c.ac_valid || !sparse_matrices_equal(a, c.ac_matrix)) {
    c.ac_matrix = a;
    c.ac_matrix.makeCompressed();
    c.ac_lu.compute(c.ac_matrix);
    if (c.ac_lu.info() != Eigen::Success) {
      c.ac_valid = false;
      error = "Complex sparse factorization failed";
      return false;
    }
    c.ac_valid = true;
  }
  x = c.ac_lu.solve(b);
  if (c.ac_lu.info() != Eigen::Success || !x.allFinite()) {
    error = "Complex sparse solve failed";
    return false;
  }
  return true;
}

bool DynamicSystem::dcSolveCached(const Eigen::SparseMatrix<double>& a,
                                  const Eigen::VectorXd& b, Eigen::VectorXd& x,
                                  std::string& error) {
  if (options.linear_solver != DynamicLinearSolverType::EigenSparseLU) {
    SparseLinearSolver solver(options.linear_solver);
    const auto result = solver.solve(a, b, x);
    if (!result.success) error = result.message;
    return result.success && x.allFinite();
  }
  if (!network_cache) network_cache = std::make_unique<NetworkSolveCache>();
  auto& c = *network_cache;
  if (!c.dc_valid || !sparse_matrices_equal(a, c.dc_matrix)) {
    c.dc_matrix = a;
    c.dc_matrix.makeCompressed();
    c.dc_lu.compute(c.dc_matrix);
    if (c.dc_lu.info() != Eigen::Success) {
      c.dc_valid = false;
      error = "Real sparse factorization failed";
      return false;
    }
    c.dc_valid = true;
  }
  x = c.dc_lu.solve(b);
  if (c.dc_lu.info() != Eigen::Success || !x.allFinite()) {
    error = "Real sparse solve failed";
    return false;
  }
  return true;
}

bool DynamicSystem::solveNetwork(double t, std::string& error) {
  const int max_iters = std::max(1, options.algebraic_network_max_iters);
  const double tol = std::max(0.0, options.algebraic_network_tol);
  double final_delta = std::numeric_limits<double>::infinity();
  double final_ac_delta = 0.0;
  double final_dc_delta = 0.0;
  bool converged = false;
  for (int iter = 0; iter < max_iters; ++iter) {
    const Eigen::VectorXcd vac_prev = y.Vac_abc;
    const Eigen::VectorXd vdc_prev = y.Vdc;

    DynamicStamp stamp(network.acPhaseNodeCount(), network.dcBusCount());
    for (const auto& device : devices) {
      device->stamp(t, x, y, stamp);
    }

    Eigen::SparseMatrix<Complex> Yac_eff;
    Eigen::SparseMatrix<double> Gdc_eff;
    Eigen::VectorXcd Iac_eff;
    Eigen::VectorXd Idc_eff;
    network.assembleEffectiveMatrices(
        stamp,
        options.singular_regularization_pu,
        Yac_eff,
        Iac_eff,
        Gdc_eff,
        Idc_eff);

    if (network.acPhaseNodeCount() > 0) {
      Eigen::VectorXcd v;
      std::string solve_error;
      if (!acSolveCached(Yac_eff, Iac_eff, v, solve_error)) {
        error = "AC transient network solve failed: " + solve_error;
        return false;
      }
      y.Vac_abc = v;
      y.Iac_abc = Iac_eff;
    }

    if (network.dcBusCount() > 0) {
      Eigen::VectorXd v;
      std::string solve_error;
      if (!dcSolveCached(Gdc_eff, Idc_eff, v, solve_error)) {
        error = "DC transient network solve failed: " + solve_error;
        return false;
      }
      for (Eigen::Index i = 0; i < v.size(); ++i) {
        if (!std::isfinite(v[i])) {
          error = "DC transient network solve produced non-finite voltages";
          return false;
        }
        y.Vdc[i] = finite_or(v[i], 1.0);
      }
      y.Idc = Idc_eff;
    }

    double delta = 0.0;
    double ac_delta = 0.0;
    double dc_delta = 0.0;
    if (vac_prev.size() == y.Vac_abc.size() && y.Vac_abc.size() > 0) {
      ac_delta = (y.Vac_abc - vac_prev).cwiseAbs().maxCoeff();
      delta = std::max(delta, ac_delta);
    }
    if (vdc_prev.size() == y.Vdc.size() && y.Vdc.size() > 0) {
      dc_delta = (y.Vdc - vdc_prev).cwiseAbs().maxCoeff();
      delta = std::max(delta, dc_delta);
    }
    final_delta = delta;
    final_ac_delta = ac_delta;
    final_dc_delta = dc_delta;
    if (!std::isfinite(final_delta)) {
      error = "Transient algebraic network iteration produced a non-finite residual";
      return false;
    }
    if (delta <= tol) {
      converged = true;
      break;
    }
  }

  if (!converged && final_delta > tol) {
    std::ostringstream os;
    os << "Transient algebraic network solve did not converge at t=" << t
       << "s; voltage fixed-point residual=" << std::scientific << final_delta
       << " > " << tol << " (AC=" << final_ac_delta
       << ", DC=" << final_dc_delta << ")";
    error = os.str();
    return false;
  }

  for (auto& device : devices) {
    device->updateAlgebraicOutputs(x, y);
  }
  error.clear();
  return true;
}

bool DynamicSystem::evaluateDerivatives(double t,
                                        const Eigen::VectorXd& state,
                                        Eigen::VectorXd& dxdt,
                                        std::string& error) {
  x.x = state;
  if (!solveNetwork(t, error)) return false;
  dxdt = Eigen::VectorXd::Zero(state.size());
  for (const auto& device : devices) {
    device->computeDerivatives(t, x, y, dxdt);
  }
  error.clear();
  return true;
}

double DynamicSystem::derivativeInfinityNorm(double t, std::string& error) {
  Eigen::VectorXd dxdt;
  if (!evaluateDerivatives(t, x.x, dxdt, error)) {
    return std::numeric_limits<double>::infinity();
  }
  x.dxdt = dxdt;
  return dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
}

}  // namespace hacdcpf::dynamics
