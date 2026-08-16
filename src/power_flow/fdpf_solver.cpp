#include "hacdcpf/power_flow/fdpf_solver.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/power_flow/admittance_builder.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr double kPi = 3.14159265358979323846;

void compute_power_injections(
    const Eigen::SparseMatrix<std::complex<double>>& ybus,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    Eigen::VectorXd& pcalc,
    Eigen::VectorXd& qcalc) {
  const int n = static_cast<int>(vm.size());
  Eigen::VectorXcd voltage(n);
  for (int i = 0; i < n; ++i) {
    voltage[i] = std::polar(vm[i], va[i]);
  }
  const Eigen::VectorXcd current = ybus * voltage;
  for (int i = 0; i < n; ++i) {
    const std::complex<double> injection = voltage[i] * std::conj(current[i]);
    pcalc[i] = injection.real();
    qcalc[i] = injection.imag();
  }
}

void compute_specified_injections(const SolverData& data,
                                  const Eigen::VectorXd& vm,
                                  Eigen::VectorXd& p_spec,
                                  Eigen::VectorXd& q_spec) {
  const int n = static_cast<int>(data.ac_buses.size());
  for (int i = 0; i < n; ++i) {
    const double v = vm[i];
    double pd = 0.0, qd = 0.0;
    double pw0 = 1.0, pw1 = 0.0, pw2 = 0.0;
    double qw0 = 1.0, qw1 = 0.0, qw2 = 0.0;
    if (data.has_component_loads) {
      pd = data.pd_pu[i];
      qd = data.qd_pu[i];
      pw0 = data.bus_zip_pp[i];
      pw1 = data.bus_zip_ip[i];
      pw2 = data.bus_zip_zp[i];
      qw0 = data.bus_zip_pq[i];
      qw1 = data.bus_zip_iq[i];
      qw2 = data.bus_zip_zq[i];
    } else {
      const auto& bus = data.ac_buses[static_cast<size_t>(i)];
      pd = bus.pd_mw / data.base_mva;
      qd = bus.qd_mvar / data.base_mva;
      pw0 = data.zip_pw[0];
      pw1 = data.zip_pw[1];
      pw2 = data.zip_pw[2];
      qw0 = data.zip_qw[0];
      qw1 = data.zip_qw[1];
      qw2 = data.zip_qw[2];
    }
    p_spec[i] = data.pg[i] - pd * (pw0 + pw1 * v + pw2 * v * v);
    q_spec[i] = data.qg[i] - qd * (qw0 + qw1 * v + qw2 * v * v);
  }
}

double raw_power_flow_residual(const SolverData& data,
                               const Eigen::VectorXd& vm,
                               const Eigen::VectorXd& va,
                               int slack) {
  const int n = static_cast<int>(data.ac_buses.size());
  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qcalc = Eigen::VectorXd::Zero(n);
  compute_power_injections(data.ybus, vm, va, pcalc, qcalc);
  Eigen::VectorXd p_spec = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd q_spec = Eigen::VectorXd::Zero(n);
  compute_specified_injections(data, vm, p_spec, q_spec);
  double residual = 0.0;
  for (int i = 0; i < n; ++i) {
    if (i == slack) continue;
    residual = std::max(residual, std::abs(p_spec[i] - pcalc[i]));
    if (data.ac_buses[static_cast<size_t>(i)].bus_type == BusType::PQ) {
      residual = std::max(residual, std::abs(q_spec[i] - qcalc[i]));
    }
  }
  return residual;
}

}  // namespace

PowerFlowResult FDPFSolver::solve(const SolverData& data,
                                   const PowerFlowOptions& opt) const {
  PowerFlowResult out;
  const bool has_hybrid_assets =
      !data.dc_buses.empty() || !data.dc_branches.empty() ||
      !data.converters.empty() || !data.lcc_converters.empty() ||
      !data.dcdc_converters.empty() || !data.energy_routers.empty();
  if (has_hybrid_assets) {
    out.diagnostics.termination_reason =
        "FDPF supports the balanced AC subsystem only";
    out.diagnostics.warnings.push_back(
        "FDPF was not run because the model contains DC buses or converter "
        "assets; use unified Newton or Newton-Krylov.");
    out.converter_model_scope.model_scope =
        "ac-only-fdpf:not-applicable-to-hybrid";
    return out;
  }
  out.converter_model_scope.model_scope = "ac-only-fdpf:fdxb";
  const int n = static_cast<int>(data.ac_buses.size());
  if (n == 0) {
    out.converged = true;
    return out;
  }

  // Classify bus types.
  const AcBusSets ac = classify_ac_buses(data);
  const int slack = ac.slack;
  const std::vector<int>& pq = ac.pq;
  const std::vector<int>& non_slack = ac.non_slack;

  const int npvpq = static_cast<int>(non_slack.size());
  const int npq = static_cast<int>(pq.size());

  // Initialize voltages from bus data.
  Eigen::VectorXd vm = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd va = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    vm[i] = data.ac_buses[static_cast<size_t>(i)].vm_pu;
    va[i] = data.ac_buses[static_cast<size_t>(i)].va_deg * kPi / 180.0;
  }

  // Build and factorize B' (reduced to non_slack × non_slack).
  Eigen::SparseMatrix<double> Bp_full =
      build_susceptance_matrix(data, make_bp_flags(opt.robust_nonlinear.min_branch_x_pu));
  Eigen::SparseMatrix<double> Bpp_full = build_susceptance_matrix(data, make_bpp_flags());

  // Reduce B' to non_slack rows/cols and B'' to pq rows/cols.
  std::vector<int> remap_ns(static_cast<size_t>(n), -1);
  for (int k = 0; k < npvpq; ++k) {
    remap_ns[static_cast<size_t>(non_slack[static_cast<size_t>(k)])] = k;
  }
  std::vector<int> remap_pq(static_cast<size_t>(n), -1);
  for (int k = 0; k < npq; ++k) {
    remap_pq[static_cast<size_t>(pq[static_cast<size_t>(k)])] = k;
  }

  auto extract_submatrix = [&](const Eigen::SparseMatrix<double>& M,
                                const std::vector<int>& remap,
                                int dim) {
    std::vector<Eigen::Triplet<double>> t;
    for (int col = 0; col < M.outerSize(); ++col) {
      const int rc = remap[static_cast<size_t>(col)];
      if (rc < 0) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, col); it; ++it) {
        const int rr = remap[static_cast<size_t>(it.row())];
        if (rr < 0) continue;
        t.emplace_back(rr, rc, it.value());
      }
    }
    Eigen::SparseMatrix<double> sub(dim, dim);
    sub.setFromTriplets(t.begin(), t.end());
    sub.makeCompressed();
    return sub;
  };

  Eigen::SparseMatrix<double> Bp_red = extract_submatrix(Bp_full, remap_ns, npvpq);
  Eigen::SparseMatrix<double> Bpp_red = extract_submatrix(Bpp_full, remap_pq, npq);

  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_bp, lu_bpp;
  lu_bp.compute(Bp_red);
  if (lu_bp.info() != Eigen::Success) {
    out.converged = false;
    return out;
  }
  if (npq > 0) {
    lu_bpp.compute(Bpp_red);
    if (lu_bpp.info() != Eigen::Success) {
      out.converged = false;
      return out;
    }
  }

  const int max_iter = opt.fdpf_max_iter;
  const double tol = opt.tol;
  int iter = 0;

  for (; iter < max_iter; ++iter) {
    // Compute power mismatch: mis = V .* conj(Ybus * V) - Sbus
    // then P = real(mis)/Vm, Q = imag(mis)/Vm
    Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd qcalc = Eigen::VectorXd::Zero(n);
    compute_power_injections(data.ybus, vm, va, pcalc, qcalc);

    Eigen::VectorXd p_spec = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd q_spec = Eigen::VectorXd::Zero(n);
    compute_specified_injections(data, vm, p_spec, q_spec);

    // P mismatch (divided by Vm, per FDPF formulation).
    Eigen::VectorXd P_mis(npvpq);
    for (int k = 0; k < npvpq; ++k) {
      const int i = non_slack[static_cast<size_t>(k)];
      const double vm_safe =
          std::max(std::abs(vm[i]), opt.robust_nonlinear.min_vm_pu);
      P_mis[k] = (p_spec[i] - pcalc[i]) / vm_safe;
    }

    // Q mismatch (divided by Vm).
    Eigen::VectorXd Q_mis(npq);
    for (int k = 0; k < npq; ++k) {
      const int i = pq[static_cast<size_t>(k)];
      const double vm_safe =
          std::max(std::abs(vm[i]), opt.robust_nonlinear.min_vm_pu);
      Q_mis[k] = (q_spec[i] - qcalc[i]) / vm_safe;
    }

    const double normP = (npvpq > 0) ? P_mis.cwiseAbs().maxCoeff() : 0.0;
    const double normQ = (npq > 0) ? Q_mis.cwiseAbs().maxCoeff() : 0.0;

    if (normP < tol && normQ < tol) {
      out.converged = true;
      break;
    }

    // P-step: update Va.  B'*Δθ = ΔP/V  →  Δθ = (B')^{-1} * P_mis
    if (npvpq > 0) {
      const Eigen::VectorXd dVa = lu_bp.solve(P_mis);
      for (int k = 0; k < npvpq; ++k) {
        va[non_slack[static_cast<size_t>(k)]] += dVa[k];
      }
    }

    // Q-step: update Vm.  B''*ΔV = ΔQ/V  →  ΔV = (B'')^{-1} * Q_mis
    if (npq > 0) {
      const Eigen::VectorXd dVm = lu_bpp.solve(Q_mis);
      for (int k = 0; k < npq; ++k) {
        vm[pq[static_cast<size_t>(k)]] += dVm[k];
      }
    }
  }

  out.iterations = iter;
  // Use one residual contract on both success and failure: the raw physical
  // P/Q mismatch in pu, without the FDPF 1/V preconditioning.
  out.residual = raw_power_flow_residual(data, vm, va, slack);

  out.vm.resize(static_cast<size_t>(n));
  out.va.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    out.vm[static_cast<size_t>(i)] = vm[i];
    out.va[static_cast<size_t>(i)] = va[i];
  }

  return out;
}

}  // namespace hacdcpf::powerflow
