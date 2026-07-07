#include "hacdcpf/dynamics/DynamicSystem.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

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

template <typename T>
bool triplets_equal(const std::vector<Eigen::Triplet<T>>& a,
                    const std::vector<Eigen::Triplet<T>>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].row() != b[i].row() || a[i].col() != b[i].col() ||
        a[i].value() != b[i].value()) {
      return false;
    }
  }
  return true;
}

void assemble_ac_effective_matrix(const DynamicNetwork& network,
                                  const DynamicStamp& stamp,
                                  double singular_regularization_pu,
                                  Eigen::SparseMatrix<Complex>& Yac_eff,
                                  Eigen::VectorXcd& Iac_eff) {
  std::vector<Eigen::Triplet<Complex>> y_triplets;
  y_triplets.reserve(static_cast<std::size_t>(network.Yac_base.nonZeros()) +
                     stamp.Yac_triplets.size() +
                     static_cast<std::size_t>(network.acPhaseNodeCount()));
  for (int col = 0; col < network.Yac_base.outerSize(); ++col) {
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(network.Yac_base, col); it; ++it) {
      y_triplets.emplace_back(it.row(), it.col(), it.value());
    }
  }
  y_triplets.insert(y_triplets.end(), stamp.Yac_triplets.begin(), stamp.Yac_triplets.end());
  if (singular_regularization_pu > 0.0 && network.Yac_base.nonZeros() == 0) {
    for (int i = 0; i < network.acPhaseNodeCount(); ++i) {
      y_triplets.emplace_back(i, i, Complex(singular_regularization_pu, 0.0));
    }
  }
  Yac_eff.resize(network.acPhaseNodeCount(), network.acPhaseNodeCount());
  Yac_eff.setFromTriplets(y_triplets.begin(), y_triplets.end());
  Iac_eff = stamp.Iac;
}

void assemble_dc_effective_matrix(const DynamicNetwork& network,
                                  const DynamicStamp& stamp,
                                  double singular_regularization_pu,
                                  Eigen::SparseMatrix<double>& Gdc_eff,
                                  Eigen::VectorXd& Idc_eff) {
  std::vector<Eigen::Triplet<double>> g_triplets;
  g_triplets.reserve(static_cast<std::size_t>(network.Gdc_base.nonZeros()) +
                     stamp.Gdc_triplets.size() +
                     static_cast<std::size_t>(network.dcBusCount()));
  for (int col = 0; col < network.Gdc_base.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(network.Gdc_base, col); it; ++it) {
      g_triplets.emplace_back(it.row(), it.col(), it.value());
    }
  }
  g_triplets.insert(g_triplets.end(), stamp.Gdc_triplets.begin(), stamp.Gdc_triplets.end());
  if (singular_regularization_pu > 0.0 && network.Gdc_base.nonZeros() == 0) {
    for (int i = 0; i < network.dcBusCount(); ++i) {
      g_triplets.emplace_back(i, i, singular_regularization_pu);
    }
  }
  Gdc_eff.resize(network.dcBusCount(), network.dcBusCount());
  Gdc_eff.setFromTriplets(g_triplets.begin(), g_triplets.end());
  Idc_eff = stamp.Idc;
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

bool sparse_solve_dense_system(const Eigen::MatrixXd& dense,
                               const Eigen::VectorXd& rhs,
                               Eigen::VectorXd& x) {
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  lu.compute(dense_to_sparse(dense));
  if (lu.info() != Eigen::Success) return false;
  x = lu.solve(rhs);
  return lu.info() == Eigen::Success && x.allFinite();
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

    Eigen::VectorXd sparse_delta;
    bool accepted = sparse_solve_dense_system(jac, -residual, sparse_delta) &&
                    try_delta(sparse_delta);
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
      Eigen::VectorXd delta;
      accepted = sparse_solve_dense_system(a, rhs, delta) && try_delta(delta);
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
      if (fault.phase >= 0 && fault.phase < 3 && phase != fault.phase) continue;
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
  assemble_ac_effective_matrix(*this, stamp, singular_regularization_pu, Yac_eff, Iac_eff);
  assemble_dc_effective_matrix(*this, stamp, singular_regularization_pu, Gdc_eff, Idc_eff);
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

  const bool has_dynamic_rl_line =
      std::any_of(devices.begin(),
                  devices.end(),
                  [](const std::unique_ptr<DynamicDevice>& device) {
                    return device->type() == "DynamicRLLine";
                  });
  if (options.solver_type == DynamicSolverType::MassMatrixDae &&
      has_dynamic_rl_line) {
    const int max_iters = options.trim_dynamic_initial_conditions
                              ? std::max(1, options.max_dynamic_trim_iters)
                              : 1;
    bool trimmed = false;
    double fast_norm = std::numeric_limits<double>::infinity();
    for (int iter = 0; iter < max_iters; ++iter) {
      bool changed = false;
      if (options.trim_dynamic_initial_conditions) {
        for (auto& device : devices) {
          changed = device->trimToNetworkEquilibrium(x, y) || changed;
        }
        changed = reanchor_machine_controllers(devices, x, y) || changed;
      }

      Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(x.x.size());
      for (const auto& device : devices) {
        device->computeDerivatives(options.t_start_s, x, y, dxdt);
      }
      x.dxdt = dxdt;
      initialization.dynamic_initial_dxdt_inf_norm =
          dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
      mask_slow_residuals(devices, dxdt);
      fast_norm = inf_norm(dxdt);
      initialization.dynamic_fast_dxdt_inf_norm = fast_norm;
      initialization.dynamic_residual_diagnostics =
          collect_residual_diagnostics(devices,
                                       options.t_start_s,
                                       x,
                                       y,
                                       std::max(1e-12, options.dynamic_trim_tol * 0.1));
      initialization.dynamic_trim_iterations = iter + 1;
      if (!options.trim_dynamic_initial_conditions ||
          fast_norm <= options.dynamic_trim_tol ||
          !changed) {
        trimmed = fast_norm <= options.dynamic_trim_tol;
        break;
      }
    }

    initialization.dynamic_fast_dxdt_inf_norm =
        std::isfinite(fast_norm) ? fast_norm : 0.0;
    initialization.dynamic_trim_converged =
        trimmed || initialization.dynamic_fast_dxdt_inf_norm <= options.dynamic_trim_tol;
    if (options.trim_dynamic_initial_conditions &&
        !initialization.dynamic_trim_converged) {
      initialization.warnings.push_back(
          "Mass-matrix DAE initialization kept the power-flow algebraic state; "
          "initial fast-state residual ||dx/dt||_inf=" +
          std::to_string(initialization.dynamic_fast_dxdt_inf_norm));
    }
    return;
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

  if (!trimmed && std::isfinite(fast_norm) &&
      options.use_consistent_dynamic_initialization) {
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
  } else if (!trimmed && std::isfinite(fast_norm)) {
    initialization.warnings.push_back(
        "Global consistent dynamic initialization was skipped; set "
        "use_consistent_dynamic_initialization=true to run the full Newton trim "
        "if the initial fast-state residual must meet dynamic_trim_tol");
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
  // Anderson(1) acceleration state for the Picard fixed-point iteration below.
  // The plain Gauss/Picard map converges only linearly and can stall badly when
  // the device mix presents negative incremental resistance (constant-power
  // loads / current-source IBRs), leaving the tightest tolerances unreachable in
  // a bounded iteration budget. Anderson(1) extrapolates from the two most
  // recent residuals; it converges to the *same* network fixed point (the loop
  // only ever returns a clean Picard image taken at convergence, never an
  // extrapolated vector), so it accelerates without changing the solution.
  constexpr double kAndersonGammaMax = 4.0;
  Eigen::VectorXcd ac_picard_prev;
  Eigen::VectorXcd ac_resid_prev;
  bool ac_anderson_ready = false;
  Eigen::VectorXd dc_picard_prev;
  Eigen::VectorXd dc_resid_prev;
  bool dc_anderson_ready = false;
  for (int iter = 0; iter < max_iters; ++iter) {
    const Eigen::VectorXcd vac_prev = y.Vac_abc;
    const Eigen::VectorXd vdc_prev = y.Vdc;

    DynamicStamp stamp(network.acPhaseNodeCount(), network.dcBusCount());
    for (const auto& device : devices) {
      device->stamp(t, x, y, stamp);
    }

    if (network.acPhaseNodeCount() > 0) {
      Eigen::VectorXcd Iac_eff = stamp.Iac;
      Eigen::SparseMatrix<Complex> Yac_eff;
      const Eigen::SparseMatrix<Complex>* Yac_solve = nullptr;
      NetworkSolveCache* cache = nullptr;
      if (options.linear_solver == DynamicLinearSolverType::EigenSparseLU) {
        if (!network_cache) network_cache = std::make_unique<NetworkSolveCache>();
        cache = network_cache.get();
      }
      if (cache != nullptr && cache->ac_valid &&
          triplets_equal(stamp.Yac_triplets, cache->ac_stamp_triplets) &&
          cache->ac_matrix.rows() == network.acPhaseNodeCount() &&
          cache->ac_matrix.cols() == network.acPhaseNodeCount()) {
        Yac_solve = &cache->ac_matrix;
      } else {
        assemble_ac_effective_matrix(network,
                                     stamp,
                                     options.singular_regularization_pu,
                                     Yac_eff,
                                     Iac_eff);
        if (cache != nullptr) cache->ac_stamp_triplets = stamp.Yac_triplets;
        Yac_solve = &Yac_eff;
      }
      Eigen::VectorXcd v;
      std::string solve_error;
      if (!acSolveCached(*Yac_solve, Iac_eff, v, solve_error)) {
        error = "AC transient network solve failed: " + solve_error;
        return false;
      }
      y.Vac_abc = v;
      y.Iac_abc = Iac_eff;
    }

    if (network.dcBusCount() > 0) {
      Eigen::VectorXd Idc_eff = stamp.Idc;
      Eigen::SparseMatrix<double> Gdc_eff;
      const Eigen::SparseMatrix<double>* Gdc_solve = nullptr;
      NetworkSolveCache* cache = nullptr;
      if (options.linear_solver == DynamicLinearSolverType::EigenSparseLU) {
        if (!network_cache) network_cache = std::make_unique<NetworkSolveCache>();
        cache = network_cache.get();
      }
      if (cache != nullptr && cache->dc_valid &&
          triplets_equal(stamp.Gdc_triplets, cache->dc_stamp_triplets) &&
          cache->dc_matrix.rows() == network.dcBusCount() &&
          cache->dc_matrix.cols() == network.dcBusCount()) {
        Gdc_solve = &cache->dc_matrix;
      } else {
        assemble_dc_effective_matrix(network,
                                     stamp,
                                     options.singular_regularization_pu,
                                     Gdc_eff,
                                     Idc_eff);
        if (cache != nullptr) cache->dc_stamp_triplets = stamp.Gdc_triplets;
        Gdc_solve = &Gdc_eff;
      }
      Eigen::VectorXd v;
      std::string solve_error;
      if (!dcSolveCached(*Gdc_solve, Idc_eff, v, solve_error)) {
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

    // Not yet converged: extrapolate the next iterate with Anderson(1). The
    // current network voltages are the Picard image G(y_prev); the change from
    // the previous iterate is the residual r_k. Overwriting the voltages with
    // the accelerated estimate only affects the *next* iteration's device
    // stamps; the convergence test above always measures the true Picard
    // residual, so the returned solution is unchanged (only reached faster).
    if (network.acPhaseNodeCount() > 0 &&
        vac_prev.size() == y.Vac_abc.size() && y.Vac_abc.size() > 0) {
      const Eigen::VectorXcd picard = y.Vac_abc;
      const Eigen::VectorXcd resid = picard - vac_prev;
      if (ac_anderson_ready && ac_resid_prev.size() == resid.size()) {
        const Eigen::VectorXcd dr = resid - ac_resid_prev;
        const double denom = dr.squaredNorm();
        if (denom > 1e-30) {
          double gamma = (dr.dot(resid)).real() / denom;
          gamma = std::clamp(gamma, -kAndersonGammaMax, kAndersonGammaMax);
          const Eigen::VectorXcd accel =
              picard - gamma * (picard - ac_picard_prev);
          if (accel.allFinite()) y.Vac_abc = accel;
        }
      }
      ac_picard_prev = picard;
      ac_resid_prev = resid;
      ac_anderson_ready = true;
    }
    if (network.dcBusCount() > 0 && vdc_prev.size() == y.Vdc.size() &&
        y.Vdc.size() > 0) {
      const Eigen::VectorXd picard = y.Vdc;
      const Eigen::VectorXd resid = picard - vdc_prev;
      if (dc_anderson_ready && dc_resid_prev.size() == resid.size()) {
        const Eigen::VectorXd dr = resid - dc_resid_prev;
        const double denom = dr.squaredNorm();
        if (denom > 1e-30) {
          double gamma = dr.dot(resid) / denom;
          gamma = std::clamp(gamma, -kAndersonGammaMax, kAndersonGammaMax);
          const Eigen::VectorXd accel =
              picard - gamma * (picard - dc_picard_prev);
          if (accel.allFinite()) y.Vdc = accel;
        }
      }
      dc_picard_prev = picard;
      dc_resid_prev = resid;
      dc_anderson_ready = true;
    }
  }

  if (!converged && final_delta > tol) {
    // The Gauss/Picard fixed-point stalled short of tolerance. Before declaring
    // failure, try a finite-difference Newton solve of the same algebraic
    // network residual, which converges quadratically where Picard is only
    // linear (design doc §15.2, §20 item 1).
    std::string newton_error;
    if (options.network_newton_fallback && solveNetworkNewton(t, newton_error)) {
      converged = true;
    } else {
      std::ostringstream os;
      os << "Transient algebraic network solve did not converge at t=" << t
         << "s; voltage fixed-point residual=" << std::scientific << final_delta
         << " > " << tol << " (AC=" << final_ac_delta
         << ", DC=" << final_dc_delta << ")";
      if (!newton_error.empty()) os << "; Newton fallback: " << newton_error;
      error = os.str();
      return false;
    }
  }

  for (auto& device : devices) {
    device->updateAlgebraicOutputs(x, y);
  }
  error.clear();
  return true;
}

bool DynamicSystem::solveNetworkNewton(double t, std::string& error) {
  const int n_ac = network.acPhaseNodeCount();
  const int n_dc = network.dcBusCount();
  const int m = 2 * n_ac + n_dc;
  if (m == 0) {
    error.clear();
    return true;
  }
  const int lac = 0;
  const int lai = n_ac;
  const int ldc = 2 * n_ac;

  // Evaluate the algebraic network residual g(V) = I_inj(V) - Y_eff*V at the
  // current voltages (device differential states are held fixed).
  const auto eval_residual = [&](Eigen::VectorXd& R) -> bool {
    DynamicStamp stamp(n_ac, n_dc);
    for (const auto& device : devices) device->stamp(t, x, y, stamp);
    Eigen::SparseMatrix<Complex> Yac;
    Eigen::VectorXcd Iac;
    Eigen::SparseMatrix<double> Gdc;
    Eigen::VectorXd Idc;
    network.assembleEffectiveMatrices(stamp, options.singular_regularization_pu,
                                      Yac, Iac, Gdc, Idc);
    R.resize(m);
    if (n_ac > 0) {
      const Eigen::VectorXcd g = Iac - Yac * y.Vac_abc;
      for (int i = 0; i < n_ac; ++i) {
        R[lac + i] = g[i].real();
        R[lai + i] = g[i].imag();
      }
    }
    if (n_dc > 0) {
      const Eigen::VectorXd g = Idc - Gdc * y.Vdc;
      for (int i = 0; i < n_dc; ++i) R[ldc + i] = g[i];
    }
    return R.allFinite();
  };

  const auto perturb = [&](int j, double h) {
    if (j < n_ac) {
      y.Vac_abc[j] += Complex(h, 0.0);
    } else if (j < 2 * n_ac) {
      y.Vac_abc[j - n_ac] += Complex(0.0, h);
    } else {
      y.Vdc[j - 2 * n_ac] += h;
    }
  };
  const auto value_of = [&](int j) -> double {
    if (j < n_ac) return y.Vac_abc[j].real();
    if (j < 2 * n_ac) return y.Vac_abc[j - n_ac].imag();
    return y.Vdc[j - 2 * n_ac];
  };

  const double tol = std::max(1e-12, options.algebraic_network_tol);
  const double fd_eps = std::sqrt(std::numeric_limits<double>::epsilon());
  const int max_iters = 25;
  Eigen::VectorXd R0;
  Eigen::VectorXd Rp;
  bool converged = false;
  for (int iter = 0; iter < max_iters; ++iter) {
    if (!eval_residual(R0)) {
      error = "network Newton residual is non-finite";
      return false;
    }
    if (R0.cwiseAbs().maxCoeff() <= tol) {
      converged = true;
      break;
    }

    // Finite-difference Jacobian of the network residual, one column per network
    // unknown, restoring the voltages exactly after each perturbation.
    const Eigen::VectorXcd vac0 = y.Vac_abc;
    const Eigen::VectorXd vdc0 = y.Vdc;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(m) * 8);
    for (int j = 0; j < m; ++j) {
      const double h = fd_eps * std::max(1.0, std::abs(value_of(j)));
      perturb(j, h);
      const bool ok = eval_residual(Rp);
      y.Vac_abc = vac0;
      y.Vdc = vdc0;
      if (!ok) {
        error = "network Newton finite-difference produced a non-finite residual";
        return false;
      }
      const Eigen::VectorXd col = (Rp - R0) / h;
      for (int i = 0; i < m; ++i) {
        if (std::abs(col[i]) > 1e-14) triplets.emplace_back(i, j, col[i]);
      }
    }

    Eigen::SparseMatrix<double> J(m, m);
    J.setFromTriplets(triplets.begin(), triplets.end());
    J.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
    lu.compute(J);
    if (lu.info() != Eigen::Success) {
      error = "network Newton Jacobian factorization failed";
      return false;
    }
    const Eigen::VectorXd dv = lu.solve(-R0);
    if (lu.info() != Eigen::Success || !dv.allFinite()) {
      error = "network Newton linear solve failed";
      return false;
    }
    // Trust region: cap the raw step so Newton stays near the current iterate,
    // preventing a jump to a non-physical (e.g. low-voltage) solution branch on
    // networks with constant-power devices (which admit multiple roots).
    Eigen::VectorXd step = dv;
    const double step_inf = step.size() > 0 ? step.cwiseAbs().maxCoeff() : 0.0;
    constexpr double kTrustRadius = 0.25;
    if (step_inf > kTrustRadius) step *= kTrustRadius / step_inf;
    // Backtracking line search: accept the largest step length (<= 1) that
    // strictly reduces the residual. This globalizes Newton so it converges to
    // the physical solution near the stalled Picard iterate instead of
    // overshooting into a non-physical region on stiff machine networks.
    const double rnorm0 = R0.cwiseAbs().maxCoeff();
    double lambda = 1.0;
    bool accepted = false;
    for (int bt = 0; bt < 24; ++bt) {
      for (int i = 0; i < n_ac; ++i) {
        y.Vac_abc[i] = vac0[i] + lambda * Complex(step[lac + i], step[lai + i]);
      }
      for (int i = 0; i < n_dc; ++i) {
        y.Vdc[i] = vdc0[i] + lambda * step[ldc + i];
      }
      if (eval_residual(Rp) && Rp.cwiseAbs().maxCoeff() < rnorm0) {
        accepted = true;
        break;
      }
      lambda *= 0.5;
    }
    if (!accepted) {
      y.Vac_abc = vac0;
      y.Vdc = vdc0;
      error = "network Newton line search failed to reduce the residual";
      return false;
    }
  }

  if (!converged) {
    error = "network Newton did not converge within the iteration budget";
    return false;
  }

  // Refresh the injected-current vectors so downstream consumers see values
  // consistent with the Newton-converged voltages.
  {
    DynamicStamp stamp(n_ac, n_dc);
    for (const auto& device : devices) device->stamp(t, x, y, stamp);
    Eigen::SparseMatrix<Complex> Yac;
    Eigen::VectorXcd Iac;
    Eigen::SparseMatrix<double> Gdc;
    Eigen::VectorXd Idc;
    network.assembleEffectiveMatrices(stamp, options.singular_regularization_pu,
                                      Yac, Iac, Gdc, Idc);
    if (n_ac > 0) y.Iac_abc = Iac;
    if (n_dc > 0) y.Idc = Idc;
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
