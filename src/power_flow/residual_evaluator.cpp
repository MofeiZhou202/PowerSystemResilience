#include "hacdcpf/power_flow/residual_evaluator.hpp"

#include <cmath>
#include <vector>

#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

void compute_ac_power_injections(const Eigen::MatrixXcd& ybus,
                                 const Eigen::VectorXd& vm,
                                 const Eigen::VectorXd& va,
                                 Eigen::VectorXd& pcalc,
                                 Eigen::VectorXd& qcalc) {
  const int n = static_cast<int>(vm.size());
  pcalc = Eigen::VectorXd::Zero(n);
  qcalc = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
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

}  // namespace

ResidualBlocks evaluate_power_flow_residual(const SolverData& data,
                                            const Eigen::VectorXd& vm,
                                            const Eigen::VectorXd& va,
                                            const Eigen::VectorXd& vdc) {
  ResidualBlocks residual;

  const int n = static_cast<int>(data.ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());

  const AcBusSets ac = classify_ac_buses(data);
  const int dc_slack = first_dc_slack_or_default(data);

  std::vector<int> dc_non_slack;
  dc_non_slack.reserve(static_cast<size_t>(std::max(0, ndc - 1)));
  for (int i = 0; i < ndc; ++i) {
    if (i != dc_slack) dc_non_slack.push_back(i);
  }

  const int np = static_cast<int>(ac.non_slack.size());
  const int nq = static_cast<int>(ac.pq.size());
  const int ndc_eq = static_cast<int>(dc_non_slack.size());

  residual.ac = Eigen::VectorXd::Zero(np + nq);
  residual.dc = Eigen::VectorXd::Zero(ndc_eq);
  residual.full = Eigen::VectorXd::Zero(np + nq + ndc_eq);

  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qcalc = Eigen::VectorXd::Zero(n);
  if (n > 0) {
    const Eigen::MatrixXcd ybus_dense = Eigen::MatrixXcd(data.ybus);
    compute_ac_power_injections(ybus_dense, vm, va, pcalc, qcalc);
  }

  Eigen::VectorXd p_spec = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd q_spec = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    const double v = vm[i];
    double pd, qd, pw0, pw1, pw2, qw0, qw1, qw2;
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
      const auto& b = data.ac_buses[static_cast<size_t>(i)];
      pd = b.pd_mw / data.base_mva;
      qd = b.qd_mvar / data.base_mva;
      pw0 = data.zip_pw[0]; pw1 = data.zip_pw[1]; pw2 = data.zip_pw[2];
      qw0 = data.zip_qw[0]; qw1 = data.zip_qw[1]; qw2 = data.zip_qw[2];
    }
    p_spec[i] = data.pg[i] - (pd * pw0 + pd * pw1 * v + pd * pw2 * v * v);
    q_spec[i] = data.qg[i] - (qd * qw0 + qd * qw1 * v + qd * qw2 * v * v);
  }

  for (const auto& conv : data.converters) {
    if (!conv.in_service) {
      continue;
    }
    const int ac_bus = conv.bus_ac - 1;
    if (ac_bus >= 0 && ac_bus < n) {
      const auto [pac, qac] =
          converter_ac_injection(conv, vm, va, vdc, data.base_mva, data.loss_model);
      p_spec[ac_bus] += pac;
      q_spec[ac_bus] += qac;
    }
  }

  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service || p.port_type != ERPortType::AC) continue;
      const int ac_bus = p.bus - 1;
      if (ac_bus >= 0 && ac_bus < n) {
        p_spec[ac_bus] += p.p_mw / data.base_mva;
        q_spec[ac_bus] += p.q_mvar / data.base_mva;
      }
    }
  }

  for (int k = 0; k < np; ++k) {
    const int i = ac.non_slack[static_cast<size_t>(k)];
    residual.ac[k] = p_spec[i] - pcalc[i];
  }
  for (int k = 0; k < nq; ++k) {
    const int i = ac.pq[static_cast<size_t>(k)];
    residual.ac[np + k] = q_spec[i] - qcalc[i];
  }

  if (ndc_eq > 0) {
    Eigen::VectorXd pdc_spec = Eigen::VectorXd::Zero(ndc);
    assemble_dc_injections(data, vm, va, vdc, pdc_spec);

    const Eigen::VectorXd pdc_linear = data.gdc * vdc;
    const Eigen::VectorXd pdc_calc = vdc.array() * pdc_linear.array();
    for (int k = 0; k < ndc_eq; ++k) {
      const int i = dc_non_slack[static_cast<size_t>(k)];
      residual.dc[k] = pdc_spec[i] - pdc_calc[i];
    }
  }

  if (residual.ac.size() > 0) {
    residual.full.head(residual.ac.size()) = residual.ac;
  }
  if (residual.dc.size() > 0) {
    residual.full.tail(residual.dc.size()) = residual.dc;
  }

  return residual;
}

}  // namespace hacdcpf::powerflow
