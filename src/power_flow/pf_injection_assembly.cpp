#include "hacdcpf/power_flow/pf_injection_assembly.hpp"

#include <algorithm>
#include <cmath>

#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

void assemble_dc_injections(const SolverData& data,
                            const Eigen::VectorXd& vm,
                            const Eigen::VectorXd& va,
                            const Eigen::VectorXd& vdc,
                            Eigen::VectorXd& pdc_spec) {
  const int ndc = static_cast<int>(data.dc_buses.size());
  pdc_spec.setZero();

  // DC loads (or bus pd_mw when the load table is absent).
  if (data.dc_loads.empty()) {
    for (int i = 0; i < ndc; ++i) {
      pdc_spec[i] = data.dc_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
    }
  } else {
    for (const auto& ld : data.dc_loads) {
      if (!ld.in_service) continue;
      const int dc_bus = ld.bus - 1;
      if (dc_bus >= 0 && dc_bus < ndc) {
        pdc_spec[dc_bus] += ld.p_mw / data.base_mva;
      }
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
      pdc_spec[dc_bus] += sg.p_mw * sg.scaling / data.base_mva;
    }
  }

  // DC-side PV arrays (generation → negative demand convention).
  for (const auto& pv : data.dc_pv_arrays) {
    if (!pv.in_service) continue;
    const int dc_bus = pv.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] -= pv.p_set_mw / data.base_mva;
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
