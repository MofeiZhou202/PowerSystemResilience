#include "hacdcpf/power_flow/pf_injection_assembly.hpp"

#include <algorithm>
#include <cmath>

#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

void assemble_ac_injections(const SolverData& data,
                            const Eigen::VectorXd& vm,
                            const Eigen::VectorXd& va,
                            const Eigen::VectorXd& vdc,
                            Eigen::VectorXd& p_spec,
                            Eigen::VectorXd& q_spec) {
  const int n = static_cast<int>(data.ac_buses.size());
  p_spec = Eigen::VectorXd::Zero(n);
  q_spec = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    if (!data.ac_buses[static_cast<size_t>(i)].in_service) {
      continue;
    }
    const double v = vm[i];
    double pd = 0.0;
    double qd = 0.0;
    double pw0 = 0.0;
    double pw1 = 0.0;
    double pw2 = 0.0;
    double qw0 = 0.0;
    double qw1 = 0.0;
    double qw2 = 0.0;
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

  for (const auto& conv : data.converters) {
    if (!conv.in_service) continue;
    const int ac_bus = conv.bus_ac - 1;
    if (ac_bus < 0 || ac_bus >= n ||
        !data.ac_buses[static_cast<size_t>(ac_bus)].in_service) continue;
    const auto [pac, qac] = converter_ac_injection(
        conv, vm, va, vdc, data.base_mva, data.loss_model);
    p_spec[ac_bus] += pac;
    q_spec[ac_bus] += qac;
  }

  for (const auto& lcc : data.lcc_converters) {
    if (!lcc.in_service) continue;
    const int ac_bus = lcc.ac_bus - 1;
    if (ac_bus < 0 || ac_bus >= n ||
        !data.ac_buses[static_cast<size_t>(ac_bus)].in_service) continue;
    const auto [pac, qac] = lcc_ac_injection(data, lcc, vm, vdc);
    p_spec[ac_bus] += pac;
    q_spec[ac_bus] += qac;
  }

  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& port : er.ports) {
      if (!port.in_service || port.port_type != ERPortType::AC) continue;
      const int ac_bus = port.bus - 1;
      if (ac_bus < 0 || ac_bus >= n ||
          !data.ac_buses[static_cast<size_t>(ac_bus)].in_service) continue;
      p_spec[ac_bus] += port.p_mw / data.base_mva;
      q_spec[ac_bus] += port.q_mvar / data.base_mva;
    }
  }
}

void assemble_dc_injections(const SolverData& data,
                            const Eigen::VectorXd& vm,
                            const Eigen::VectorXd& va,
                            const Eigen::VectorXd& vdc,
                            Eigen::VectorXd& pdc_spec) {
  const int ndc = static_cast<int>(data.dc_buses.size());
  pdc_spec.setZero();

  // DC bus demand and explicit DCLoad records are additive.
  // pdc_spec is the NET power injection at a bus (generation positive,
  // consumption negative), matching pdc_calc = V .* (gdc * V) where gdc is the
  // positive-diagonal nodal conductance Laplacian. A load therefore enters as a
  // negative injection.
  for (int i = 0; i < ndc; ++i) {
    pdc_spec[i] = -data.dc_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
  }
  for (const auto& ld : data.dc_loads) {
    if (!ld.in_service) continue;
    const int dc_bus = ld.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] -= model::effective_load_p_mw(ld) / data.base_mva;
    }
  }

  // DC-side storage: positive p_mw = discharge = generation at DC bus.
  for (const auto& st : data.dc_storage) {
    if (!st.in_service) continue;
    const int dc_bus = st.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += st.p_mw / data.base_mva;
    }
  }

  // DC-side static generators.
  for (const auto& sg : data.dc_static_generators) {
    if (!sg.in_service) continue;
    const int dc_bus = sg.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] +=
          sg.p_mw * model::sanitize_scaling(sg.scaling) / data.base_mva;
    }
  }

  // DC-side PV arrays: generation enters as a positive net injection.
  for (const auto& pv : data.dc_pv_arrays) {
    if (!pv.in_service) continue;
    const int dc_bus = pv.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += pv.p_set_mw / data.base_mva;
    }
  }

  // VSC converter DC-side injections.
  for (const auto& conv : data.converters) {
    if (!conv.in_service) continue;
    const int dc_bus = conv.bus_dc - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += converter_dc_injection(
          conv, vm, va, vdc, data.base_mva, data.loss_model);
    }
  }

  // LCC converter DC-side injections (quasi-steady operating point evaluated
  // at the current iterate; rectifier positive, inverter negative).
  for (const auto& lcc : data.lcc_converters) {
    if (!lcc.in_service) continue;
    const int dc_bus = lcc.dc_bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += lcc_dc_injection(data, lcc, vm, vdc);
    }
  }

  // DCDC converter contributions.
  // Sign convention: positive p_ref_mw = forward transfer (bus_in → bus_out).
  // p_ref_mw is OUTPUT-referenced: the power delivered to bus_out.
  for (const auto& dcdc : data.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int bin = dcdc.bus_in - 1;
    const int bout = dcdc.bus_out - 1;
    if (bin < 0 || bin >= ndc || bout < 0 || bout >= ndc) continue;

    const DCDCPowerTransfer transfer = dcdc_power_transfer(dcdc, vdc, data.base_mva);
    pdc_spec[bin] -= transfer.p_in_pu;
    pdc_spec[bout] += transfer.p_out_pu;
  }

  // EnergyRouter DC ports.
  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service || p.port_type != ERPortType::DC) continue;
      const int dc_bus = p.bus - 1;
      if (dc_bus >= 0 && dc_bus < ndc) {
        pdc_spec[dc_bus] += p.p_mw / data.base_mva;
      }
    }
  }
}

}  // namespace hacdcpf::powerflow
