#include "hacdcpf/assembly/solver_data.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <stdexcept>

#include "hacdcpf/projection/result_attribution.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
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
    const double scaling = model::sanitize_scaling(sg.scaling);
    data.pg[idx] += sg.p_mw * scaling / data.base_mva;
    data.qg[idx] += sg.q_mvar * scaling / data.base_mva;
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
    if (ld.model == LoadModel::Exponential) {
      throw std::invalid_argument(
          "aggregate_load_demand: Load " + std::to_string(ld.index) +
          " uses Exponential model without represented voltage exponents; "
          "use ZIP or ConstantPower");
    }
    const int idx = ld.bus - 1;
    if (idx < 0 || idx >= nac) continue;

    const double scaling = model::sanitize_scaling(ld.scaling);
    const double pd = ld.p_mw * scaling / data.base_mva;
    const double qd = ld.q_mvar * scaling / data.base_mva;
    data.pd_pu[idx] += pd;
    data.qd_pu[idx] += qd;

    const double abs_p = std::abs(ld.p_mw * scaling);
    const double abs_q = std::abs(ld.q_mvar * scaling);

    const auto normalized_zip = [](double z, double i, double p) {
      z = std::isfinite(z) ? std::max(0.0, z) : 0.0;
      i = std::isfinite(i) ? std::max(0.0, i) : 0.0;
      p = std::isfinite(p) ? std::max(0.0, p) : 0.0;
      const double sum = z + i + p;
      if (sum <= 1e-20) return std::array<double, 3>{0.0, 0.0, 1.0};
      return std::array<double, 3>{z / sum, i / sum, p / sum};
    };
    const auto zip_p = normalized_zip(
        ld.z_percent_p, ld.i_percent_p, ld.p_percent_p);
    const auto zip_q = normalized_zip(
        ld.z_percent_q, ld.i_percent_q, ld.p_percent_q);

    // Weighted sum of per-load normalized ZIP coefficients.
    data.bus_zip_pp[idx] += zip_p[2] * abs_p;
    data.bus_zip_ip[idx] += zip_p[1] * abs_p;
    data.bus_zip_zp[idx] += zip_p[0] * abs_p;
    total_p_weight[idx] += abs_p;

    data.bus_zip_pq[idx] += zip_q[2] * abs_q;
    data.bus_zip_iq[idx] += zip_q[1] * abs_q;
    data.bus_zip_zq[idx] += zip_q[0] * abs_q;
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
  // JacobianPattern caches the numerical AC/DC admittance entries as well as
  // their sparsity. Advance the data revision even when Eigen reuses the same
  // matrix storage, otherwise a solver can silently retain pre-rebuild values
  // after an OLTC/tap update.
  ++data.build_id;
}

bool refresh_solver_data_values(SolverData& data,
                                const HybridPowerSystem& sys) {
  // Rich-device expansion and bus merging require authored-to-canonical
  // mapping updates. Keep those cases on the conservative full rebuild path;
  // the direct MATPOWER/time-series layout below is one-to-one.
  if (data.bus_merge_map || !sys.ac.transformers_2w.empty() ||
      !sys.ac.transformers_3w.empty() || !sys.ac.switches.empty() ||
      !sys.ac.circuit_breakers.empty() || !sys.energy_routers.empty() ||
      !sys.ac.external_grids.empty() || !sys.vsc_converters.empty() ||
      !sys.lcc_converters.empty() || !sys.dc.dcdc_converters.empty() ||
      !sys.dc.dc_storage.empty() || sys.three_phase_ac.has_value()) {
    return false;
  }
  if (data.base_mva != sys.base_mva ||
      data.ac_buses.size() != sys.ac.buses.size() ||
      data.ac_branches.size() != sys.ac.branches.size() ||
      data.generators.size() != sys.ac.generators.size() ||
      data.loads.size() != sys.ac.loads.size() ||
      data.flexible_loads.size() != sys.ac.flexible_loads.size() ||
      data.static_generators.size() != sys.ac.static_generators.size() ||
      data.renewable_gens.size() != sys.ac.renewable_gens.size() ||
      data.pv_systems.size() != sys.ac.pv_systems.size() ||
      data.storage_units.size() != sys.ac.storage.size() ||
      data.shunts.size() != sys.ac.shunts.size() ||
      data.charging_stations.size() != sys.ac.charging_stations.size() ||
      data.chargers.size() != sys.ac.chargers.size() ||
      data.dc_buses.size() != sys.dc.buses.size() ||
      data.dc_branches.size() != sys.dc.branches.size() ||
      data.dc_loads.size() != sys.dc.loads.size() ||
      data.dc_storage.size() != sys.dc.storage.size() ||
      data.dc_static_generators.size() != sys.dc.static_generators.size() ||
      data.dc_pv_arrays.size() != sys.dc.pv_arrays.size() ||
      data.vpps.size() != sys.vpps.size() ||
      data.microgrids.size() != sys.microgrids.size() ||
      data.mobile_storage.size() != sys.mobile_storage.size()) {
    return false;
  }

  for (size_t i = 0; i < data.ac_buses.size(); ++i) {
    const auto& old_bus = data.ac_buses[i];
    const auto& new_bus = sys.ac.buses[i];
    if (old_bus.index != new_bus.index || old_bus.bus_type != new_bus.bus_type ||
        old_bus.in_service != new_bus.in_service) {
      return false;
    }
  }
  for (size_t i = 0; i < data.dc_buses.size(); ++i) {
    const auto& old_bus = data.dc_buses[i];
    const auto& new_bus = sys.dc.buses[i];
    if (old_bus.index != new_bus.index || old_bus.bus_type != new_bus.bus_type ||
        old_bus.in_service != new_bus.in_service) {
      return false;
    }
  }
  for (size_t i = 0; i < data.ac_branches.size(); ++i) {
    const auto& old_branch = data.ac_branches[i];
    const auto& new_branch = sys.ac.branches[i];
    if (old_branch.index != new_branch.index ||
        old_branch.from_bus != new_branch.from_bus ||
        old_branch.to_bus != new_branch.to_bus ||
        old_branch.in_service != new_branch.in_service ||
        old_branch.r_pu != new_branch.r_pu ||
        old_branch.x_pu != new_branch.x_pu ||
        old_branch.b_pu != new_branch.b_pu ||
        old_branch.tap != new_branch.tap ||
        old_branch.shift_deg != new_branch.shift_deg) {
      return false;
    }
  }
  for (size_t i = 0; i < data.dc_branches.size(); ++i) {
    const auto& old_branch = data.dc_branches[i];
    const auto& new_branch = sys.dc.branches[i];
    if (old_branch.index != new_branch.index ||
        old_branch.from_bus != new_branch.from_bus ||
        old_branch.to_bus != new_branch.to_bus ||
        old_branch.in_service != new_branch.in_service ||
        old_branch.r_pu != new_branch.r_pu) {
      return false;
    }
  }
  for (size_t i = 0; i < data.shunts.size(); ++i) {
    const auto& old_shunt = data.shunts[i];
    const auto& new_shunt = sys.ac.shunts[i];
    if (old_shunt.index != new_shunt.index || old_shunt.bus != new_shunt.bus ||
        old_shunt.in_service != new_shunt.in_service ||
        old_shunt.gs_mw != new_shunt.gs_mw ||
        old_shunt.bs_mvar != new_shunt.bs_mvar ||
        old_shunt.current_step != new_shunt.current_step) {
      return false;
    }
  }
  for (size_t i = 0; i < data.generators.size(); ++i) {
    const auto& old_gen = data.generators[i];
    const auto& new_gen = sys.ac.generators[i];
    if (old_gen.index != new_gen.index || old_gen.bus != new_gen.bus ||
        old_gen.is_slack != new_gen.is_slack ||
        old_gen.in_service != new_gen.in_service) {
      return false;
    }
  }

  // In-place assignments preserve vector and sparse-matrix storage, so the
  // Newton pattern and symbolic factorization remain valid. Only aggregated
  // specified injections are recomputed.
  std::copy(sys.ac.buses.begin(), sys.ac.buses.end(), data.ac_buses.begin());
  std::copy(sys.ac.branches.begin(), sys.ac.branches.end(), data.ac_branches.begin());
  std::copy(sys.ac.generators.begin(), sys.ac.generators.end(), data.generators.begin());
  std::copy(sys.ac.loads.begin(), sys.ac.loads.end(), data.loads.begin());
  std::copy(sys.ac.flexible_loads.begin(), sys.ac.flexible_loads.end(),
            data.flexible_loads.begin());
  std::copy(sys.ac.static_generators.begin(), sys.ac.static_generators.end(),
            data.static_generators.begin());
  std::copy(sys.ac.renewable_gens.begin(), sys.ac.renewable_gens.end(),
            data.renewable_gens.begin());
  std::copy(sys.ac.pv_systems.begin(), sys.ac.pv_systems.end(),
            data.pv_systems.begin());
  std::copy(sys.ac.storage.begin(), sys.ac.storage.end(), data.storage_units.begin());
  std::copy(sys.ac.charging_stations.begin(), sys.ac.charging_stations.end(),
            data.charging_stations.begin());
  std::copy(sys.ac.chargers.begin(), sys.ac.chargers.end(), data.chargers.begin());
  std::copy(sys.dc.buses.begin(), sys.dc.buses.end(), data.dc_buses.begin());
  std::copy(sys.dc.branches.begin(), sys.dc.branches.end(), data.dc_branches.begin());
  std::copy(sys.dc.loads.begin(), sys.dc.loads.end(), data.dc_loads.begin());
  std::copy(sys.dc.storage.begin(), sys.dc.storage.end(), data.dc_storage.begin());
  std::copy(sys.dc.static_generators.begin(), sys.dc.static_generators.end(),
            data.dc_static_generators.begin());
  std::copy(sys.dc.pv_arrays.begin(), sys.dc.pv_arrays.end(), data.dc_pv_arrays.begin());
  std::copy(sys.vpps.begin(), sys.vpps.end(), data.vpps.begin());
  std::copy(sys.microgrids.begin(), sys.microgrids.end(), data.microgrids.begin());
  std::copy(sys.mobile_storage.begin(), sys.mobile_storage.end(),
            data.mobile_storage.begin());
  aggregate_generation(data);
  aggregate_load_demand(data);
  return true;
}

// AC-side voltage-forming converters (multi-converter model r1 §1/§2/§4.2/§4.7).
// Three control modes pin the AC terminal voltage of their bus:
//   * AC_PV (Mode 2) and DC_V_DROOP_AC_V (Mode 6) hold the AC voltage magnitude,
//     so their AC bus becomes a PV bus at v_ac_set_pu — the Newton solver fixes
//     Vm there and releases the bus reactive balance (the converter's reactive
//     power becomes the free balancing device unknown).
//   * AC_GRID_FORMING (Mode 1) forms the full AC reference (angle + magnitude),
//     so its AC bus becomes a SLACK bus at v_ac_set_pu / va_set_deg — the
//     converter both fixes the angle reference and balances the AC island's
//     active and reactive power.
// A bus that is already SLACK is left untouched (an existing rigid reference
// wins); a grid-forming converter promotes a PQ/PV bus to SLACK. Roles are read
// through resolve_device_control_role so the AC-side treatment matches the
// converter-coordination checks and honors the ac_grid_forming opt-in flag (not
// just the raw control_mode enum).
static void apply_acpv_voltage_control(SolverData& data) {
  for (const auto& conv : data.converters) {
    if (!conv.in_service) continue;
    const DeviceControlRole role = resolve_device_control_role(conv);
    const bool forms_ac = role.is_ac_grid_forming;
    const bool holds_vmag = !forms_ac && role.controls_ac_v;
    if (!holds_vmag && !forms_ac) continue;
    const int idx = conv.bus_ac - 1;
    if (idx < 0 || idx >= static_cast<int>(data.ac_buses.size())) continue;
    auto& bus = data.ac_buses[static_cast<size_t>(idx)];

    if (forms_ac) {
      // Grid-forming: promote to SLACK and pin the reference angle + magnitude.
      if (bus.bus_type != BusType::SLACK) {
        bus.bus_type = BusType::SLACK;
        bus.va_deg = conv.v_ac_angle_set_deg;
      }
      if (conv.v_ac_set_pu > 0.0) bus.vm_pu = conv.v_ac_set_pu;
      continue;
    }

    // Voltage-magnitude hold (AC_PV / Mode 6): promote PQ/PV to PV, never demote
    // an existing SLACK reference.
    if (bus.bus_type == BusType::SLACK) continue;
    bus.bus_type = BusType::PV;
    if (conv.v_ac_set_pu > 0.0) bus.vm_pu = conv.v_ac_set_pu;
  }
}

SolverData make_solver_data(const HybridPowerSystem& sys, LossModelType loss_model) {
  static std::atomic<std::uint64_t> next_build_id{1};

  HybridPowerSystem projected =
      projection::RichToCanonicalOperator::apply(sys).canonical;

  // Fold DC-side `DCStorage` (active-power-only) into the engine's existing
  // `Storage`-typed dc.storage path so it injects into the DC power flow.
  materialize_dc_storage(projected);

  SolverData data;
  data.ac_buses = std::move(projected.ac.buses);
  data.ac_branches = std::move(projected.ac.branches);
  data.dc_buses = std::move(projected.dc.buses);
  data.dc_branches = std::move(projected.dc.branches);
  data.converters = std::move(projected.vsc_converters);
  data.lcc_converters = std::move(projected.lcc_converters);
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
    sg.co2_emission_rate = sgdc.emission_factor_tco2_mwh;
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
  data.projection_certificate = std::move(projected.projection_certificate);
  data.projection_report = std::move(projected.projection_report);

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

  apply_acpv_voltage_control(data);
  rebuild_matrices(data);
  return data;
}

SolverData make_solver_data(HybridPowerSystem&& sys, LossModelType loss_model) {
  HybridPowerSystem projected =
      projection::RichToCanonicalOperator::apply(std::move(sys)).canonical;
  return make_solver_data_projected(std::move(projected), loss_model);
}

SolverData make_solver_data_projected(HybridPowerSystem&& projected, LossModelType loss_model) {
  static std::atomic<std::uint64_t> next_build_id{1};

  // Fold DC-side `DCStorage` (active-power-only) into the engine's existing
  // `Storage`-typed dc.storage path so it injects into the DC power flow.
  materialize_dc_storage(projected);

  SolverData data;
  data.ac_buses = std::move(projected.ac.buses);
  data.ac_branches = std::move(projected.ac.branches);
  data.dc_buses = std::move(projected.dc.buses);
  data.dc_branches = std::move(projected.dc.branches);
  data.converters = std::move(projected.vsc_converters);
  data.lcc_converters = std::move(projected.lcc_converters);
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
    sg.co2_emission_rate = sgdc.emission_factor_tco2_mwh;
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
  data.projection_certificate = std::move(projected.projection_certificate);
  data.projection_report = std::move(projected.projection_report);

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

  apply_acpv_voltage_control(data);
  rebuild_matrices(data);
  return data;
}

}  // namespace hacdcpf::powerflow
