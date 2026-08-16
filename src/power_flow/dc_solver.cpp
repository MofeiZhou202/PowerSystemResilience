#include "hacdcpf/power_flow/dc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/SparseLU>

#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr double kMinVdc = 0.05;

}  // namespace

Eigen::SparseMatrix<double> build_dc_newton_jacobian(
    const SolverData& data,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    const Eigen::VectorXd& vdc,
    const std::vector<int>& dc_non_slack) {
  (void)va;
  const int ndc = static_cast<int>(data.dc_buses.size());
  const int neq = static_cast<int>(dc_non_slack.size());
  std::vector<int> reduced(static_cast<size_t>(ndc), -1);
  for (int k = 0; k < neq; ++k) {
    reduced[static_cast<size_t>(dc_non_slack[static_cast<size_t>(k)])] = k;
  }

  // Pcalc = diag(V) G V, so dPcalc/dV = diag(GV) + diag(V)G.
  // Device terms then apply J = d(Pcalc - Pspec)/dV exactly.
  const Eigen::VectorXd gdc_v = data.gdc * vdc;
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data.gdc.nonZeros() + neq * 3));
  for (int col = 0; col < data.gdc.outerSize(); ++col) {
    const int reduced_col = reduced[static_cast<size_t>(col)];
    if (reduced_col < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(data.gdc, col); it; ++it) {
      const int reduced_row = reduced[static_cast<size_t>(it.row())];
      if (reduced_row < 0) continue;
      double value = vdc[it.row()] * it.value();
      if (it.row() == col) value += gdc_v[it.row()];
      triplets.emplace_back(reduced_row, reduced_col, value);
    }
  }

  const auto add = [&](int row_bus, int col_bus, double value) {
    if (row_bus < 0 || row_bus >= ndc || col_bus < 0 || col_bus >= ndc ||
        value == 0.0) {
      return;
    }
    const int row = reduced[static_cast<size_t>(row_bus)];
    const int col = reduced[static_cast<size_t>(col_bus)];
    if (row >= 0 && col >= 0) triplets.emplace_back(row, col, value);
  };

  for (const auto& conv : data.converters) {
    if (!conv.in_service) continue;
    const int bus = conv.bus_dc - 1;
    const double dspec = converter_dc_jacobian_vdc(
        conv, vdc, data.base_mva, data.loss_model).dpdc_dvdc;
    add(bus, bus, -dspec);
  }
  for (const auto& lcc : data.lcc_converters) {
    if (!lcc.in_service) continue;
    const int bus = lcc.dc_bus - 1;
    add(bus, bus, -lcc_dc_jacobian_vdc(data, lcc, vm, vdc));
  }
  for (const auto& dcdc : data.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int bin = dcdc.bus_in - 1;
    const int bout = dcdc.bus_out - 1;
    const auto jac = dcdc_jacobian_vdc(dcdc, vdc, data.base_mva);
    // Pspec,in = -Pin and Pspec,out = +Pout.
    add(bin, bin, jac.dpin_dvdc_in);
    add(bin, bout, jac.dpin_dvdc_out);
    add(bout, bin, -jac.dpout_dvdc_in);
    add(bout, bout, -jac.dpout_dvdc_out);
  }

  Eigen::SparseMatrix<double> jacobian(neq, neq);
  jacobian.setFromTriplets(triplets.begin(), triplets.end());
  jacobian.makeCompressed();
  return jacobian;
}

DCPowerFlowResult DCSolver::solve(const SolverData& data,
                                  const PowerFlowOptions& opt,
                                  const InitialState* init) const {
  DCPowerFlowResult out;

  const int ndc = static_cast<int>(data.dc_buses.size());
  out.vdc.assign(static_cast<size_t>(ndc), 1.0);
  out.converged = false;
  out.iterations = 0;
  out.residual = 0.0;

  if (ndc == 0) {
    out.converged = true;
    return out;
  }

  const int n = static_cast<int>(data.ac_buses.size());

  const int dc_slack = first_dc_slack_or_default(data);
  if (dc_slack < 0) {
    throw std::invalid_argument("Invalid DC slack bus detection.");
  }

  std::vector<int> dc_non_slack;
  dc_non_slack.reserve(static_cast<size_t>(std::max(0, ndc - 1)));
  for (int i = 0; i < ndc; ++i) {
    if (i == dc_slack) continue;
    // Isolated DC buses are de-energized: hold them at their fixed voltage and
    // exclude them from the solved equation set (otherwise an unconnected bus
    // contributes a zero row/column and makes the Jacobian singular).
    if (data.dc_buses[static_cast<size_t>(i)].bus_type == DCBusType::DC_ISOLATED) {
      continue;
    }
    dc_non_slack.push_back(i);
  }

  const int ndc_eq = static_cast<int>(dc_non_slack.size());
  if (ndc_eq == 0) {
    for (int i = 0; i < ndc; ++i) {
      out.vdc[static_cast<size_t>(i)] = data.dc_buses[static_cast<size_t>(i)].vm_pu;
    }
    out.converged = true;
    out.iterations = 0;
    out.residual = 0.0;
    return out;
  }

  Eigen::VectorXd vdc = Eigen::VectorXd::Ones(ndc);
  load_dc_initial_state(init, data.dc_buses, vdc);

  Eigen::VectorXd vm_dummy = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd va_dummy = Eigen::VectorXd::Zero(n);

  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      vm_dummy[i] = data.ac_buses[static_cast<size_t>(i)].vm_pu;
      va_dummy[i] = 0.0;
    }
  }

  Eigen::VectorXd mismatch = Eigen::VectorXd::Zero(ndc_eq);
  Eigen::VectorXd pdc_linear = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_calc = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_spec = Eigen::VectorXd::Zero(ndc);

  auto eval = [&](const Eigen::VectorXd& vdc_state, Eigen::VectorXd& mismatch_out) {
    assemble_dc_injections(data, vm_dummy, va_dummy, vdc_state, pdc_spec);

    pdc_linear = data.gdc * vdc_state;
    pdc_calc = vdc_state.array() * pdc_linear.array();
    for (int k = 0; k < ndc_eq; ++k) {
      const int i = dc_non_slack[static_cast<size_t>(k)];
      mismatch_out[k] = pdc_spec[i] - pdc_calc[i];
    }
    return inf_norm(mismatch_out);
  };

  for (int iter = 1; iter <= opt.max_iter; ++iter) {
    const double res = eval(vdc, mismatch);
    out.iterations = iter;
    out.residual = res;
    if (res < opt.tol) {
      out.converged = true;
      break;
    }

    const Eigen::SparseMatrix<double> jac = build_dc_newton_jacobian(
        data, vm_dummy, va_dummy, vdc, dc_non_slack);
    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
    lu.analyzePattern(jac);
    lu.factorize(jac);
    if (lu.info() != Eigen::Success) {
      out.converged = false;
      break;
    }

    const Eigen::VectorXd dx = lu.solve(mismatch);
    if (lu.info() != Eigen::Success || !dx.allFinite()) {
      out.converged = false;
      break;
    }

    Eigen::VectorXd vdc_best = vdc;
    double best_res = res;
    bool improved = false;
    double alpha = 1.0;

    for (int ls = 0; ls < 8; ++ls) {
      Eigen::VectorXd trial = vdc;
      for (int k = 0; k < ndc_eq; ++k) {
        const int b = dc_non_slack[static_cast<size_t>(k)];
        trial[b] = std::max(trial[b] + alpha * dx[k], kMinVdc);
      }
      const double trial_res = eval(trial, mismatch);
      if (trial_res < best_res) {
        best_res = trial_res;
        vdc_best = trial;
        improved = true;
      }
      if (trial_res < res) {
        break;
      }
      alpha *= 0.5;
    }

    if (improved) {
      vdc = vdc_best;
    } else {
      for (int k = 0; k < ndc_eq; ++k) {
        const int b = dc_non_slack[static_cast<size_t>(k)];
        vdc[b] = std::max(vdc[b] + dx[k], kMinVdc);
      }
    }
  }

  (void)eval(vdc, mismatch);
  out.residual = inf_norm(mismatch);
  if (out.residual < opt.tol) {
    out.converged = true;
  }

  for (int i = 0; i < ndc; ++i) {
    out.vdc[static_cast<size_t>(i)] = vdc[i];
  }

  return out;
}

}  // namespace hacdcpf::powerflow
