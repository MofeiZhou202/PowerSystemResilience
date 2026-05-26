#include "hacdcpf/assembly/solver_data.hpp"

#include <atomic>
#include <cmath>

#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/model/enums/grid_enums.hpp"
#include "hacdcpf/model/enums/storage_enums.hpp"
#include "hacdcpf/power_flow/admittance_builder.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

namespace hacdcpf::powerflow {

void aggregate_generation(SolverData& data) {
  const int nac = static_cast<int>(data.ac_buses.size());
  data.pg = Eigen::VectorXd::Zero(nac);
  data.qg = Eigen::VectorXd::Zero(nac);

  // Standard generators.
  for (const auto& gen : data.generators) {
    if (!gen.in_service) continue;
    const int idx = gen.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += gen.pg_mw / data.base_mva;
    data.qg[idx] += gen.qg_mvar / data.base_mva;
  }

  // Static generators (distributed generation).
  for (const auto& sg : data.static_generators) {
    if (!sg.in_service) continue;
    const int idx = sg.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += sg.p_mw * sg.scaling / data.base_mva;
    data.qg[idx] += sg.q_mvar * sg.scaling / data.base_mva;
  }

  // Renewable generators.
  for (const auto& rg : data.renewable_gens) {
    if (!rg.in_service) continue;
    const int idx = rg.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += rg.p_mw / data.base_mva;
    data.qg[idx] += rg.q_mvar / data.base_mva;
  }

  // PV systems (use power curve if array parameters available).
  for (const auto& pv : data.pv_systems) {
    if (!pv.in_service) continue;
    const int idx = pv.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    const double p_mw = compute_pv_power_mw(pv);
    data.pg[idx] += p_mw / data.base_mva;
    data.qg[idx] += pv.q_mvar / data.base_mva;
  }

  // Storage (positive p_mw = discharge = generation).
  for (const auto& st : data.storage_units) {
    if (!st.in_service) continue;
    const int idx = st.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += st.p_mw / data.base_mva;
    data.qg[idx] += st.q_mvar / data.base_mva;
  }

  // Virtual Power Plants (aggregated DER output at aggregation bus).
  for (const auto& vpp : data.vpps) {
    if (!vpp.in_service) continue;
    const int idx = vpp.pcc_bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += vpp.p_output_mw / data.base_mva;
    data.qg[idx] += vpp.q_output_mvar / data.base_mva;
  }

  // Microgrids (power exchange at PCC bus when grid-connected).
  // Positive p_exchange_mw = export to grid = generation injection.
  for (const auto& mg : data.microgrids) {
    if (!mg.in_service) continue;
    if (mg.operating_mode != MicrogridMode::GridConnected) continue;
    const int idx = mg.pcc_bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += mg.p_exchange_mw / data.base_mva;
  }

  // Mobile Storage (inject when not in transit).
  for (const auto& ms : data.mobile_storage) {
    if (!ms.in_service) continue;
    if (ms.status == MobileStorageStatus::InTransit) continue;
    const int idx = ms.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pg[idx] += ms.p_mw / data.base_mva;
    data.qg[idx] += ms.q_mvar / data.base_mva;
  }
}

void aggregate_load_demand(SolverData& data) {
  const int nac = static_cast<int>(data.ac_buses.size());
  // Component-based loads are used when Load table is non-empty.
  // ChargingStations are additive to either bus-level or Load-based demand.
  if (data.loads.empty() && data.charging_stations.empty()) {
    data.has_component_loads = false;
    return;
  }

  data.has_component_loads = true;
  data.pd_pu = Eigen::VectorXd::Zero(nac);
  data.qd_pu = Eigen::VectorXd::Zero(nac);
  data.bus_zip_zp = Eigen::VectorXd::Zero(nac);
  data.bus_zip_ip = Eigen::VectorXd::Zero(nac);
  data.bus_zip_pp = Eigen::VectorXd::Zero(nac);
  data.bus_zip_zq = Eigen::VectorXd::Zero(nac);
  data.bus_zip_iq = Eigen::VectorXd::Zero(nac);
  data.bus_zip_pq = Eigen::VectorXd::Zero(nac);

  // H1: Always initialise pd_pu/qd_pu from bus-level demand so that the PF
  // solver uses the same additive model as DC-OPF (bus.pd_mw + scaled loads).
  // When no component loads are present this is the sole demand source.
  for (int i = 0; i < nac; ++i) {
    if (!data.ac_buses[static_cast<size_t>(i)].in_service) continue;
    data.pd_pu[i] = data.ac_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
    data.qd_pu[i] = data.ac_buses[static_cast<size_t>(i)].qd_mvar / data.base_mva;
    // Default: constant power (overwritten by ZIP normalisation when loads present).
    data.bus_zip_pp[i] = 1.0;
    data.bus_zip_pq[i] = 1.0;
  }

  // Accumulate per-bus load and track total P/Q weight for averaging ZIP.
  Eigen::VectorXd total_p_weight = Eigen::VectorXd::Zero(nac);
  Eigen::VectorXd total_q_weight = Eigen::VectorXd::Zero(nac);

  // H1: When component loads are present, seed ZIP numerators and weights with
  // bus-level demand (constant power, fraction=1.0) so that the weighted-average
  // normalisation accounts for the base demand alongside component-load ZIP models.
  if (!data.loads.empty()) {
    for (int i = 0; i < nac; ++i) {
      if (!data.ac_buses[static_cast<size_t>(i)].in_service) continue;
      const double abs_p = std::abs(data.ac_buses[static_cast<size_t>(i)].pd_mw);
      const double abs_q = std::abs(data.ac_buses[static_cast<size_t>(i)].qd_mvar);
      data.bus_zip_pp[i] = 1.0 * abs_p;  // fraction=1.0, weight=abs_p
      data.bus_zip_pq[i] = 1.0 * abs_q;
      total_p_weight[i]  = abs_p;
      total_q_weight[i]  = abs_q;
    }
  }

  for (const auto& ld : data.loads) {
    if (!ld.in_service) continue;
    const int idx = ld.bus - 1;
    if (idx < 0 || idx >= nac) continue;

    const double pd = ld.p_mw * ld.scaling / data.base_mva;   // H1: apply scaling
    const double qd = ld.q_mvar * ld.scaling / data.base_mva; // H1: apply scaling
    data.pd_pu[idx] += pd;
    data.qd_pu[idx] += qd;

    const double abs_p = std::abs(ld.p_mw * ld.scaling);  // H1: scale ZIP weight
    const double abs_q = std::abs(ld.q_mvar * ld.scaling);

    // Weighted sum of ZIP coefficients (weight = |P| or |Q|).
    data.bus_zip_pp[idx] += (ld.p_percent_p / 100.0) * abs_p;
    data.bus_zip_ip[idx] += (ld.i_percent_p / 100.0) * abs_p;
    data.bus_zip_zp[idx] += (ld.z_percent_p / 100.0) * abs_p;
    total_p_weight[idx] += abs_p;

    data.bus_zip_pq[idx] += (ld.p_percent_q / 100.0) * abs_q;
    data.bus_zip_iq[idx] += (ld.i_percent_q / 100.0) * abs_q;
    data.bus_zip_zq[idx] += (ld.z_percent_q / 100.0) * abs_q;
    total_q_weight[idx] += abs_q;
  }

  // Aggregate charging station power as constant-power loads (kW/kvar → MW/Mvar).
  // ChargingStations are always constant-power, added to pd_pu/qd_pu directly.
  for (const auto& cs : data.charging_stations) {
    if (!cs.in_service) continue;
    const int idx = cs.bus - 1;
    if (idx < 0 || idx >= nac) continue;
    data.pd_pu[idx] += (cs.p_total_kw / 1000.0) / data.base_mva;
    data.qd_pu[idx] += (cs.q_total_kvar / 1000.0) / data.base_mva;
    // When Load table is present, include CS in ZIP weighted average.
    if (!data.loads.empty()) {
      const double abs_p = std::abs(cs.p_total_kw / 1000.0);
      const double abs_q = std::abs(cs.q_total_kvar / 1000.0);
      data.bus_zip_pp[idx] += 1.0 * abs_p;
      total_p_weight[idx] += abs_p;
      data.bus_zip_pq[idx] += 1.0 * abs_q;
      total_q_weight[idx] += abs_q;
    }
  }

  // Normalize to get weighted-average ZIP coefficients per bus.
  // Only needed when Load entries exist (they have non-trivial ZIP coefficients).
  // When only charging stations / bus-level demand are present, ZIP is constant-power (1,0,0).
  if (!data.loads.empty()) {
    for (int i = 0; i < nac; ++i) {
      if (total_p_weight[i] > 1e-20) {
        data.bus_zip_pp[i] /= total_p_weight[i];
        data.bus_zip_ip[i] /= total_p_weight[i];
        data.bus_zip_zp[i] /= total_p_weight[i];
      } else {
        // Default: constant power.
        data.bus_zip_pp[i] = 1.0;
      }
      if (total_q_weight[i] > 1e-20) {
        data.bus_zip_pq[i] /= total_q_weight[i];
        data.bus_zip_iq[i] /= total_q_weight[i];
        data.bus_zip_zq[i] /= total_q_weight[i];
      } else {
        data.bus_zip_pq[i] = 1.0;
      }
    }
  }
}

void rebuild_matrices(SolverData& data) {
  aggregate_generation(data);
  aggregate_load_demand(data);
  data.ybus = build_admittance_matrix(data);
  data.gdc = build_dc_conductance(data);
}

SolverData make_solver_data(const HybridPowerSystem& sys, LossModelType loss_model) {
  static std::atomic<std::uint64_t> next_build_id{1};

  HybridPowerSystem projected = project_to_canonical_models(sys);

  SolverData data;
  data.ac_buses = std::move(projected.ac.buses);
  data.ac_branches = std::move(projected.ac.branches);
  data.dc_buses = std::move(projected.dc.buses);
  data.dc_branches = std::move(projected.dc.branches);
  data.converters = std::move(projected.vsc_converters);
  data.dcdc_converters = std::move(projected.dc.dcdc_converters);
  data.energy_routers = std::move(projected.energy_routers);
  data.generators = std::move(projected.ac.generators);
  data.loads = std::move(projected.ac.loads);
  data.static_generators = std::move(projected.ac.static_generators);
  data.renewable_gens = std::move(projected.ac.renewable_gens);
  data.pv_systems = std::move(projected.ac.pv_systems);
  data.storage_units = std::move(projected.ac.storage);
  data.dc_loads = std::move(projected.dc.loads);
  data.dc_storage = std::move(projected.dc.storage);
  data.dc_static_generators = std::move(projected.dc.static_generators);
  // Also convert StaticGeneratorDC entries into StaticGenerator format
  for (auto& sgdc : projected.dc.dc_static_generators) {
    StaticGenerator sg;
    sg.index = sgdc.index;
    sg.bus = sgdc.bus;
    sg.in_service = sgdc.in_service;
    sg.name = std::move(sgdc.name);
    sg.p_mw = sgdc.p_set_mw;
    sg.scaling = sgdc.scaling;
    sg.pmax_mw = sgdc.pmax_mw;
    sg.pmin_mw = sgdc.pmin_mw;
    sg.controllable = sgdc.controllable;
    data.dc_static_generators.push_back(std::move(sg));
  }
  data.dc_pv_arrays = std::move(projected.dc.pv_arrays);
  data.shunts = std::move(projected.ac.shunts);
  data.charging_stations = std::move(projected.ac.charging_stations);
  data.chargers = std::move(projected.ac.chargers);
  data.external_grids = std::move(projected.ac.external_grids);
  data.flexible_loads = std::move(projected.ac.flexible_loads);
  data.switches = std::move(projected.ac.switches);
  data.vpps = std::move(projected.vpps);
  data.microgrids = std::move(projected.microgrids);
  data.mobile_storage = std::move(projected.mobile_storage);
  data.base_mva = projected.base_mva;
  data.loss_model = loss_model;
  data.build_id = next_build_id.fetch_add(1, std::memory_order_relaxed);

  // Propagate bus merge map if present.
  data.bus_merge_map = std::move(projected.bus_merge_map);

  // Apply ExternalGrid settings: set bus voltage setpoints and bus type.
  for (const auto& eg : data.external_grids) {
    if (!eg.in_service) continue;
    const int idx = eg.bus - 1;
    if (idx < 0 || idx >= static_cast<int>(data.ac_buses.size())) continue;
    data.ac_buses[static_cast<size_t>(idx)].vm_pu = eg.vm_pu;
    data.ac_buses[static_cast<size_t>(idx)].va_deg = eg.va_deg;
    // ExternalGrid implies slack bus.
    if (data.ac_buses[static_cast<size_t>(idx)].bus_type == BusType::PQ) {
      data.ac_buses[static_cast<size_t>(idx)].bus_type = BusType::SLACK;
    }
  }

  rebuild_matrices(data);
  return data;
}

SolverData make_solver_data(HybridPowerSystem&& sys, LossModelType loss_model) {
  HybridPowerSystem projected = project_to_canonical_models(std::move(sys));
  return make_solver_data_projected(std::move(projected), loss_model);
}

SolverData make_solver_data_projected(HybridPowerSystem&& projected, LossModelType loss_model) {
  static std::atomic<std::uint64_t> next_build_id{1};

  SolverData data;
  data.ac_buses = std::move(projected.ac.buses);
  data.ac_branches = std::move(projected.ac.branches);
  data.dc_buses = std::move(projected.dc.buses);
  data.dc_branches = std::move(projected.dc.branches);
  data.converters = std::move(projected.vsc_converters);
  data.dcdc_converters = std::move(projected.dc.dcdc_converters);
  data.energy_routers = std::move(projected.energy_routers);
  data.generators = std::move(projected.ac.generators);
  data.loads = std::move(projected.ac.loads);
  data.static_generators = std::move(projected.ac.static_generators);
  data.renewable_gens = std::move(projected.ac.renewable_gens);
  data.pv_systems = std::move(projected.ac.pv_systems);
  data.storage_units = std::move(projected.ac.storage);
  data.dc_loads = std::move(projected.dc.loads);
  data.dc_storage = std::move(projected.dc.storage);
  data.dc_static_generators = std::move(projected.dc.static_generators);
  for (auto& sgdc : projected.dc.dc_static_generators) {
    StaticGenerator sg;
    sg.index = sgdc.index;
    sg.bus = sgdc.bus;
    sg.in_service = sgdc.in_service;
    sg.name = std::move(sgdc.name);
    sg.p_mw = sgdc.p_set_mw;
    sg.scaling = sgdc.scaling;
    sg.pmax_mw = sgdc.pmax_mw;
    sg.pmin_mw = sgdc.pmin_mw;
    sg.controllable = sgdc.controllable;
    data.dc_static_generators.push_back(std::move(sg));
  }
  data.dc_pv_arrays = std::move(projected.dc.pv_arrays);
  data.shunts = std::move(projected.ac.shunts);
  data.charging_stations = std::move(projected.ac.charging_stations);
  data.chargers = std::move(projected.ac.chargers);
  data.external_grids = std::move(projected.ac.external_grids);
  data.flexible_loads = std::move(projected.ac.flexible_loads);
  data.switches = std::move(projected.ac.switches);
  data.vpps = std::move(projected.vpps);
  data.microgrids = std::move(projected.microgrids);
  data.mobile_storage = std::move(projected.mobile_storage);
  data.base_mva = projected.base_mva;
  data.loss_model = loss_model;
  data.build_id = next_build_id.fetch_add(1, std::memory_order_relaxed);
  data.bus_merge_map = std::move(projected.bus_merge_map);

  for (const auto& eg : data.external_grids) {
    if (!eg.in_service) continue;
    const int idx = eg.bus - 1;
    if (idx < 0 || idx >= static_cast<int>(data.ac_buses.size())) continue;
    data.ac_buses[static_cast<size_t>(idx)].vm_pu = eg.vm_pu;
    data.ac_buses[static_cast<size_t>(idx)].va_deg = eg.va_deg;
    if (data.ac_buses[static_cast<size_t>(idx)].bus_type == BusType::PQ) {
      data.ac_buses[static_cast<size_t>(idx)].bus_type = BusType::SLACK;
    }
  }

  rebuild_matrices(data);
  return data;
}

}  // namespace hacdcpf::powerflow
