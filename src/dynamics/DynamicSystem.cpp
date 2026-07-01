#include "hacdcpf/dynamics/DynamicSystem.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>

#include <Eigen/SparseLU>

namespace hacdcpf::dynamics {

namespace {

using Complex = std::complex<double>;

double finite_or(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
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
  y_triplets.reserve(ac_branches.size() * 36 + ac_bus_ids.size() * 3);

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

  if (singular_regularization_pu > 0.0) {
    for (int i = 0; i < acPhaseNodeCount(); ++i) {
      y_triplets.emplace_back(i, i, Complex(singular_regularization_pu, 0.0));
    }
  }

  Yac_base.resize(acPhaseNodeCount(), acPhaseNodeCount());
  Yac_base.setFromTriplets(y_triplets.begin(), y_triplets.end());

  std::vector<Eigen::Triplet<double>> g_triplets;
  g_triplets.reserve(dc_branches.size() * 4 + dc_bus_ids.size());
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
    const double norm = derivativeInfinityNorm(options.t_start_s, error);
    if (error.empty()) {
      initialization.dynamic_initial_dxdt_inf_norm = norm;
      initialization.dynamic_fast_dxdt_inf_norm = norm;
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

    Eigen::VectorXd dxdt;
    if (!evaluateDerivatives(options.t_start_s, x.x, dxdt, error)) {
      initialization.warnings.push_back("Dynamic equilibrium trim residual unavailable: " + error);
      break;
    }
    x.dxdt = dxdt;
    fast_norm = dxdt.size() > 0 ? dxdt.lpNorm<Eigen::Infinity>() : 0.0;
    initialization.dynamic_trim_iterations = iter + 1;
    if (fast_norm <= options.dynamic_trim_tol || !changed) {
      trimmed = fast_norm <= options.dynamic_trim_tol;
      break;
    }
  }

  if (!std::isfinite(fast_norm)) {
    fast_norm = derivativeInfinityNorm(options.t_start_s, error);
  }
  initialization.dynamic_fast_dxdt_inf_norm = std::isfinite(fast_norm) ? fast_norm : 0.0;
  initialization.dynamic_initial_dxdt_inf_norm = initialization.dynamic_fast_dxdt_inf_norm;
  initialization.dynamic_trim_converged = trimmed ||
      initialization.dynamic_fast_dxdt_inf_norm <= options.dynamic_trim_tol;
  if (!initialization.dynamic_trim_converged) {
    initialization.warnings.push_back(
        "Dynamic equilibrium trim did not fully converge; initial fast-state residual ||dx/dt||_inf=" +
        std::to_string(initialization.dynamic_fast_dxdt_inf_norm));
  }
}

bool DynamicSystem::solveNetwork(double t, std::string& error) {
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
    Eigen::SparseLU<Eigen::SparseMatrix<Complex>> solver;
    solver.compute(Yac_eff);
    if (solver.info() != Eigen::Success) {
      error = "AC transient admittance factorization failed";
      return false;
    }
    const Eigen::VectorXcd v = solver.solve(Iac_eff);
    if (solver.info() != Eigen::Success) {
      error = "AC transient network solve failed";
      return false;
    }
    y.Vac_abc = v;
    y.Iac_abc = Iac_eff;
  }

  if (network.dcBusCount() > 0) {
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(Gdc_eff);
    if (solver.info() != Eigen::Success) {
      error = "DC transient conductance factorization failed";
      return false;
    }
    const Eigen::VectorXd v = solver.solve(Idc_eff);
    if (solver.info() != Eigen::Success) {
      error = "DC transient network solve failed";
      return false;
    }
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      y.Vdc[i] = finite_or(v[i], 1.0);
    }
    y.Idc = Idc_eff;
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
