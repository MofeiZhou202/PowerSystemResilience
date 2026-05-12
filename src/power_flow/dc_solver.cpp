#include "hacdcpf/power_flow/dc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/LU>

#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr double kMinVdc = 0.05;

}  // namespace

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
    if (i != dc_slack) {
      dc_non_slack.push_back(i);
    }
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
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(ndc_eq, ndc_eq);

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

    jac.setZero();
    for (int rk = 0; rk < ndc_eq; ++rk) {
      const int k = dc_non_slack[static_cast<size_t>(rk)];
      for (int rl = 0; rl < ndc_eq; ++rl) {
        const int l = dc_non_slack[static_cast<size_t>(rl)];
        if (k == l) {
          jac(rk, rl) = pdc_linear[k] + data.gdc.coeff(k, k) * vdc[k];
        } else {
          jac(rk, rl) = data.gdc.coeff(k, l) * vdc[k];
        }
      }
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu(jac);
    if (!lu.isInvertible()) {
      out.converged = false;
      out.failure_reason = SolverFailureReason::SingularJacobian;
      break;
    }

    const Eigen::VectorXd dx = lu.solve(mismatch);
    if (!dx.allFinite()) {
      out.converged = false;
      out.failure_reason = SolverFailureReason::NonFiniteSolution;
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
