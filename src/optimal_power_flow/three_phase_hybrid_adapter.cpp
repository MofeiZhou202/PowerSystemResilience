#include "hacdcpf/optimal_power_flow/three_phase_hybrid_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Sparse>

#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace hacdcpf::opf::phase_hybrid {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::array<double, 3> phase_values(double a, double b, double c) {
  return {a, b, c};
}

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

const Generator* matching_rich_generator(const HybridPowerSystem& system,
                                         const ThreePhaseGenerator& phase_gen) {
  const auto by_name = std::find_if(
      system.ac.generators.begin(), system.ac.generators.end(),
      [&](const Generator& generator) {
        return generator.in_service && !phase_gen.name.empty() &&
               generator.name == phase_gen.name;
      });
  if (by_name != system.ac.generators.end()) return &*by_name;
  const auto by_bus = std::find_if(
      system.ac.generators.begin(), system.ac.generators.end(),
      [&](const Generator& generator) {
        return generator.in_service && generator.bus == phase_gen.bus;
      });
  return by_bus == system.ac.generators.end() ? nullptr : &*by_bus;
}

}  // namespace

int ThreePhaseHybridModel::phase_node(int ac_bus_id, int phase) const {
  if (phase < 0 || phase >= 3) return -1;
  const auto it = std::find(ac_bus_ids.begin(), ac_bus_ids.end(), ac_bus_id);
  if (it == ac_bus_ids.end()) return -1;
  const auto pos = static_cast<std::size_t>(
      std::distance(ac_bus_ids.begin(), it));
  return bus_phase_nodes[pos][static_cast<std::size_t>(phase)];
}

ThreePhaseHybridModel build_three_phase_hybrid_model(
    const HybridPowerSystem& system,
    const ThreePhaseHybridAdapterOptions& options) {
  if (!system.three_phase_ac.has_value()) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF requires a three_phase_ac subsystem");
  }
  if (system.dc.buses.empty() || system.vsc_converters.empty()) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF requires a DC network and VSC converters");
  }
  if (!system.dc.dcdc_converters.empty()) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF adapter does not support DC/DC converters");
  }
  if (!system.energy_routers.empty()) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF adapter does not expand energy routers");
  }

  const auto& phase_system = *system.three_phase_ac;
  ThreePhaseHybridModel model;
  auto& opf = model.opf;
  opf.name = system.name + "_three_phase_hybrid_opf";
  opf.base_mva = phase_system.base_mva > 0.0
                     ? phase_system.base_mva
                     : (system.base_mva > 0.0 ? system.base_mva : 100.0);
  opf.vuf_max = std::clamp(options.vuf_max, 0.0, 1.0);

  const int full_node_count =
      static_cast<int>(phase_system.buses.size()) * 3;
  std::vector<int> full_to_node(
      static_cast<std::size_t>(full_node_count), -1);
  model.ac_bus_ids.reserve(phase_system.buses.size());
  model.bus_phase_nodes.reserve(phase_system.buses.size());
  int node_count = 0;
  for (std::size_t bus_pos = 0; bus_pos < phase_system.buses.size(); ++bus_pos) {
    const auto& bus = phase_system.buses[bus_pos];
    model.ac_bus_ids.push_back(bus.index);
    model.bus_phase_nodes.push_back({-1, -1, -1});
    if (!bus.in_service) continue;
    for (int phase = 0; phase < 3; ++phase) {
      if (!bus.phase_mask.has(phase)) continue;
      full_to_node[bus_pos * 3 + static_cast<std::size_t>(phase)] = node_count;
      model.bus_phase_nodes[bus_pos][static_cast<std::size_t>(phase)] =
          node_count++;
    }
  }
  if (node_count == 0) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF found no in-service phase nodes");
  }

  auto network_for_ybus = phase_system;
  // Reference voltages and dispatchable source powers represent external grids
  // in the OPF. Stamping their Norton equivalents as well would double count.
  network_for_ybus.external_grids.clear();
  std::vector<Eigen::Triplet<std::complex<double>>> ac_triplets;
  for (const auto& entry : analysis::build_full_ybus_phase_entries(
           network_for_ybus, options.include_shunts)) {
    if (entry.row < 0 || entry.row >= full_node_count || entry.col < 0 ||
        entry.col >= full_node_count) {
      continue;
    }
    const int row = full_to_node[static_cast<std::size_t>(entry.row)];
    const int col = full_to_node[static_cast<std::size_t>(entry.col)];
    if (row >= 0 && col >= 0) {
      ac_triplets.emplace_back(row, col, entry.value);
    }
  }
  opf.y_ac.resize(node_count, node_count);
  opf.y_ac.setFromTriplets(ac_triplets.begin(), ac_triplets.end());
  opf.y_ac.makeCompressed();
  opf.i_ac_fixed = Eigen::VectorXcd::Zero(node_count);
  opf.p_load_pu = Eigen::VectorXd::Zero(node_count);
  opf.q_load_pu = Eigen::VectorXd::Zero(node_count);
  opf.v_min_pu = Eigen::VectorXd::Constant(node_count, 0.90);
  opf.v_max_pu = Eigen::VectorXd::Constant(node_count, 1.10);
  opf.voltage_start = Eigen::VectorXcd::Ones(node_count);
  opf.ac_phase_index.assign(static_cast<std::size_t>(node_count), 0);

  std::unordered_map<int, int> phase_bus_position;
  for (std::size_t pos = 0; pos < phase_system.buses.size(); ++pos) {
    const auto& bus = phase_system.buses[pos];
    phase_bus_position[bus.index] = static_cast<int>(pos);
    const auto vm = phase_values(bus.vm_a_pu, bus.vm_b_pu, bus.vm_c_pu);
    const auto va = phase_values(bus.va_a_deg, bus.va_b_deg, bus.va_c_deg);
    const auto pd = phase_values(bus.pd_a_mw, bus.pd_b_mw, bus.pd_c_mw);
    const auto qd = phase_values(bus.qd_a_mvar, bus.qd_b_mvar, bus.qd_c_mvar);
    std::vector<int> complete_bus;
    for (int phase = 0; phase < 3; ++phase) {
      const int node = model.bus_phase_nodes[pos][static_cast<std::size_t>(phase)];
      if (node < 0) continue;
      opf.p_load_pu[node] += pd[static_cast<std::size_t>(phase)] / opf.base_mva;
      opf.q_load_pu[node] += qd[static_cast<std::size_t>(phase)] / opf.base_mva;
      opf.v_min_pu[node] = bus.vmin_pu > 0.0 ? bus.vmin_pu : 0.90;
      opf.v_max_pu[node] = bus.vmax_pu > 0.0 ? bus.vmax_pu : 1.10;
      opf.voltage_start[node] = std::polar(
          vm[static_cast<std::size_t>(phase)] > 0.0
              ? vm[static_cast<std::size_t>(phase)]
              : 1.0,
          va[static_cast<std::size_t>(phase)] * kPi / 180.0);
      opf.ac_phase_index[static_cast<std::size_t>(node)] = phase;
      complete_bus.push_back(node);
    }
    if (complete_bus.size() == 3) {
      opf.three_phase_bus_nodes.push_back(std::move(complete_bus));
    }
  }

  bool used_load_equivalent = false;
  for (const auto& load : phase_system.loads) {
    if (!load.in_service) continue;
    const bool non_wye = lowercase(load.connection) == "delta";
    const bool voltage_dependent =
        load.const_z_percent > 1e-9 || load.const_i_percent > 1e-9 ||
        load.p_const_z_percent > 1e-9 || load.p_const_i_percent > 1e-9 ||
        load.q_const_z_percent > 1e-9 || load.q_const_i_percent > 1e-9;
    if ((non_wye || voltage_dependent) &&
        !options.allow_constant_power_load_equivalent) {
      throw std::invalid_argument(
          "Three-phase hybrid OPF requires a constant-power equivalent for "
          "delta or voltage-dependent OpenDSS loads");
    }
    used_load_equivalent = used_load_equivalent || non_wye || voltage_dependent;
    const auto bus_it = phase_bus_position.find(load.bus);
    if (bus_it == phase_bus_position.end()) continue;
    const auto p = phase_values(load.p_a_mw, load.p_b_mw, load.p_c_mw);
    const auto q = phase_values(load.q_a_mvar, load.q_b_mvar, load.q_c_mvar);
    for (int phase = 0; phase < 3; ++phase) {
      if (!load.phase_mask.has(phase)) continue;
      const int node = model.bus_phase_nodes[
          static_cast<std::size_t>(bus_it->second)]
          [static_cast<std::size_t>(phase)];
      if (node < 0) continue;
      opf.p_load_pu[node] += p[static_cast<std::size_t>(phase)] / opf.base_mva;
      opf.q_load_pu[node] += q[static_cast<std::size_t>(phase)] / opf.base_mva;
    }
  }
  if (used_load_equivalent) {
    model.model_limitations.push_back(
        "Delta or voltage-dependent OpenDSS loads use their imported "
        "per-phase constant-power operating-point equivalent");
  }

  const double total_load_mw = std::max(
      1.0, opf.p_load_pu.cwiseMax(0.0).sum() * opf.base_mva);
  for (const auto& generator : phase_system.generators) {
    if (!generator.in_service) continue;
    const auto bus_it = phase_bus_position.find(generator.bus);
    if (bus_it == phase_bus_position.end()) continue;
    const auto p = phase_values(generator.p_a_mw, generator.p_b_mw,
                                generator.p_c_mw);
    const auto q = phase_values(generator.q_a_mvar, generator.q_b_mvar,
                                generator.q_c_mvar);
    const bool has_explicit_phase_dispatch =
        std::abs(generator.p_a_mw) + std::abs(generator.p_b_mw) +
            std::abs(generator.p_c_mw) + std::abs(generator.q_a_mvar) +
            std::abs(generator.q_b_mvar) + std::abs(generator.q_c_mvar) >
        1e-12;
    const double phase_count = std::max(1, generator.phase_mask.count());
    const Generator* rich = matching_rich_generator(system, generator);
    for (int phase = 0; phase < 3; ++phase) {
      if (!generator.phase_mask.has(phase)) continue;
      const int node = model.bus_phase_nodes[
          static_cast<std::size_t>(bus_it->second)]
          [static_cast<std::size_t>(phase)];
      if (node < 0) continue;
      const double pg = has_explicit_phase_dispatch
                            ? p[static_cast<std::size_t>(phase)]
                            : generator.p_mw / phase_count;
      const double qg = has_explicit_phase_dispatch
                            ? q[static_cast<std::size_t>(phase)]
                            : generator.q_mvar / phase_count;
      PhaseGenerator item;
      item.phase_node = node;
      const bool has_p_bounds = generator.pmax_mw > generator.pmin_mw;
      const bool has_q_bounds = generator.qmax_mvar > generator.qmin_mvar;
      item.p_min_pu =
          (has_p_bounds ? generator.pmin_mw / phase_count : pg) / opf.base_mva;
      item.p_max_pu =
          (has_p_bounds ? generator.pmax_mw / phase_count : pg) / opf.base_mva;
      item.q_min_pu =
          (has_q_bounds ? generator.qmin_mvar / phase_count : qg) / opf.base_mva;
      item.q_max_pu =
          (has_q_bounds ? generator.qmax_mvar / phase_count : qg) / opf.base_mva;
      item.cost_c2 = rich ? rich->cost_c2 * phase_count : 0.0;
      item.cost_c1 = rich ? rich->cost_c1 : 30.0;
      opf.generators.push_back(item);
      model.generator_bindings.push_back(
          {generator.index,
           generator.bus,
           phase,
           generator.name,
           "three_phase_generator",
           rich ? rich->ramp_up_mw_min : 0.0,
           rich ? rich->ramp_dn_mw_min : 0.0});
    }
  }

  std::unordered_set<int> reference_node_set;
  const auto add_reference = [&](int bus_pos, PhaseMask mask,
                                 const std::array<double, 3>& vm,
                                 const std::array<double, 3>& va) {
    for (int phase = 0; phase < 3; ++phase) {
      if (!mask.has(phase)) continue;
      const int node = model.bus_phase_nodes[
          static_cast<std::size_t>(bus_pos)][static_cast<std::size_t>(phase)];
      if (node < 0 || !reference_node_set.insert(node).second) continue;
      opf.reference_nodes.push_back(node);
      opf.reference_voltage.conservativeResize(opf.reference_voltage.size() + 1);
      opf.reference_voltage[opf.reference_voltage.size() - 1] =
          std::polar(vm[static_cast<std::size_t>(phase)],
                     va[static_cast<std::size_t>(phase)] * kPi / 180.0);
    }
  };
  for (const auto& grid : phase_system.external_grids) {
    if (!grid.in_service) continue;
    const auto bus_it = phase_bus_position.find(grid.bus);
    if (bus_it == phase_bus_position.end()) continue;
    const auto vm = grid.use_phase_voltage_setpoint
                        ? phase_values(grid.vm_a_pu, grid.vm_b_pu, grid.vm_c_pu)
                        : phase_values(grid.vm_pu, grid.vm_pu, grid.vm_pu);
    const auto va = grid.use_phase_voltage_setpoint
                        ? phase_values(grid.va_a_deg, grid.va_b_deg, grid.va_c_deg)
                        : phase_values(grid.va_deg, grid.va_deg - 120.0,
                                       grid.va_deg + 120.0);
    add_reference(bus_it->second, grid.phase_mask, vm, va);
    const auto rich = std::find_if(
        system.ac.external_grids.begin(), system.ac.external_grids.end(),
        [&](const ExternalGrid& source) {
          return source.in_service && source.bus == grid.bus;
        });
    for (int phase = 0; phase < 3; ++phase) {
      if (!grid.phase_mask.has(phase)) continue;
      const int node = model.bus_phase_nodes[
          static_cast<std::size_t>(bus_it->second)]
          [static_cast<std::size_t>(phase)];
      if (node < 0) continue;
      PhaseGenerator source;
      source.phase_node = node;
      source.p_min_pu = -5.0 * total_load_mw / opf.base_mva;
      source.p_max_pu = 5.0 * total_load_mw / opf.base_mva;
      source.q_min_pu = -5.0 * total_load_mw / opf.base_mva;
      source.q_max_pu = 5.0 * total_load_mw / opf.base_mva;
      source.cost_c2 = rich != system.ac.external_grids.end()
                           ? rich->cost_c2 * 3.0
                           : 0.0;
      source.cost_c1 = rich != system.ac.external_grids.end()
                           ? rich->cost_c1
                           : 30.0;
      opf.generators.push_back(source);
      model.generator_bindings.push_back(
          {grid.index, grid.bus, phase, grid.name,
           "three_phase_external_grid", 0.0, 0.0});
    }
  }
  if (opf.reference_nodes.empty()) {
    for (std::size_t pos = 0; pos < phase_system.buses.size(); ++pos) {
      const auto& bus = phase_system.buses[pos];
      if (!bus.in_service || bus.bus_type != BusType::SLACK) continue;
      add_reference(static_cast<int>(pos), bus.phase_mask,
                    phase_values(bus.vm_a_pu, bus.vm_b_pu, bus.vm_c_pu),
                    phase_values(bus.va_a_deg, bus.va_b_deg, bus.va_c_deg));
      break;
    }
  }
  if (opf.reference_nodes.empty()) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF requires an AC reference source");
  }

  std::unordered_map<int, int> dc_bus_position;
  for (const auto& bus : system.dc.buses) {
    if (!bus.in_service) continue;
    dc_bus_position[bus.index] = static_cast<int>(model.dc_bus_ids.size());
    model.dc_bus_ids.push_back(bus.index);
  }
  const int dc_count = static_cast<int>(model.dc_bus_ids.size());
  if (dc_count == 0) {
    throw std::invalid_argument(
        "Three-phase hybrid OPF found no in-service DC buses");
  }
  std::vector<Eigen::Triplet<double>> dc_triplets;
  for (const auto& branch : system.dc.branches) {
    if (!branch.in_service) continue;
    const auto from = dc_bus_position.find(branch.from_bus);
    const auto to = dc_bus_position.find(branch.to_bus);
    if (from == dc_bus_position.end() || to == dc_bus_position.end()) continue;
    if (!(branch.r_pu > 0.0)) {
      throw std::invalid_argument(
          "Three-phase hybrid OPF requires positive DC branch resistance");
    }
    const double conductance = std::max(1, branch.n_parallel) / branch.r_pu;
    dc_triplets.emplace_back(from->second, from->second, conductance);
    dc_triplets.emplace_back(to->second, to->second, conductance);
    dc_triplets.emplace_back(from->second, to->second, -conductance);
    dc_triplets.emplace_back(to->second, from->second, -conductance);
  }
  opf.g_dc.resize(dc_count, dc_count);
  opf.g_dc.setFromTriplets(dc_triplets.begin(), dc_triplets.end());
  opf.g_dc.makeCompressed();
  opf.p_dc_load_pu = Eigen::VectorXd::Zero(dc_count);
  opf.v_dc_start = Eigen::VectorXd::Ones(dc_count);
  opf.v_dc_min_pu = Eigen::VectorXd::Constant(dc_count, 0.90);
  opf.v_dc_max_pu = Eigen::VectorXd::Constant(dc_count, 1.10);
  for (const auto& bus : system.dc.buses) {
    const auto it = dc_bus_position.find(bus.index);
    if (it == dc_bus_position.end()) continue;
    opf.p_dc_load_pu[it->second] += bus.pd_mw / opf.base_mva;
    opf.v_dc_start[it->second] = bus.vm_pu > 0.0 ? bus.vm_pu : 1.0;
    opf.v_dc_min_pu[it->second] = bus.vmin_pu > 0.0 ? bus.vmin_pu : 0.90;
    opf.v_dc_max_pu[it->second] = bus.vmax_pu > 0.0 ? bus.vmax_pu : 1.10;
    if (bus.bus_type == DCBusType::DC_V) {
      opf.dc_reference_terminals.push_back(it->second);
      opf.dc_reference_voltage_pu.conservativeResize(
          opf.dc_reference_voltage_pu.size() + 1);
      opf.dc_reference_voltage_pu[opf.dc_reference_voltage_pu.size() - 1] =
          opf.v_dc_start[it->second];
    }
  }
  for (const auto& load : system.dc.loads) {
    if (!load.in_service) continue;
    const auto it = dc_bus_position.find(load.bus);
    if (it != dc_bus_position.end()) {
      opf.p_dc_load_pu[it->second] +=
          load.p_mw * load.scaling / opf.base_mva;
    }
  }
  for (const auto& source : system.dc.dc_static_generators) {
    if (!source.in_service) continue;
    if (source.controllable) {
      throw std::invalid_argument(
          "Three-phase hybrid OPF adapter does not dispatch controllable "
          "DC static generators");
    }
    const auto it = dc_bus_position.find(source.bus);
    if (it != dc_bus_position.end()) {
      opf.p_dc_load_pu[it->second] -=
          source.p_set_mw * source.scaling / opf.base_mva;
    }
  }
  for (const auto& source : system.dc.pv_arrays) {
    if (!source.in_service) continue;
    const auto it = dc_bus_position.find(source.bus);
    if (it != dc_bus_position.end()) {
      opf.p_dc_load_pu[it->second] -= source.p_set_mw / opf.base_mva;
    }
  }
  for (const auto& storage : system.dc.dc_storage) {
    if (!storage.in_service) continue;
    const auto it = dc_bus_position.find(storage.bus);
    if (it != dc_bus_position.end()) {
      opf.p_dc_load_pu[it->second] -= storage.p_mw / opf.base_mva;
    }
  }

  for (const auto& converter : system.vsc_converters) {
    if (!converter.in_service) continue;
    const auto ac_bus = phase_bus_position.find(converter.bus_ac);
    const auto dc_bus = dc_bus_position.find(converter.bus_dc);
    if (ac_bus == phase_bus_position.end() || dc_bus == dc_bus_position.end()) {
      continue;
    }
    PhaseVSC item;
    for (int phase = 0; phase < 3; ++phase) {
      const int node = model.bus_phase_nodes[
          static_cast<std::size_t>(ac_bus->second)]
          [static_cast<std::size_t>(phase)];
      if (node >= 0) item.phase_nodes.push_back(node);
    }
    if (item.phase_nodes.size() != 3) {
      throw std::invalid_argument(
          "Sequence-aware VSC terminals require phases A, B, and C");
    }
    item.dc_terminal = dc_bus->second;
    item.efficiency = std::clamp(converter.eta, 0.01, 1.0);
    const double rated_mva = std::max(
        {converter.p_rated_mw, std::abs(converter.pmax_mw),
         std::abs(converter.pmin_mw), std::abs(converter.qmax_mvar),
         std::abs(converter.qmin_mvar), 1e-6});
    item.s_max_pu = options.enforce_converter_capacity
                        ? rated_mva / opf.base_mva
                        : 1e6;
    item.phase_current_max_pu =
        options.enforce_converter_current && converter.i_ac_max_pu > 0.0
            ? converter.i_ac_max_pu
            : 1e6;
    item.control_mode =
        converter.ac_grid_forming ||
                converter.control_mode == ConverterMode::AC_GRID_FORMING
            ? PhaseVSCControlMode::GridFormingDroop
            : PhaseVSCControlMode::EqualPhasePower;
    item.virtual_r_pu =
        converter.r_conv_ac_pu > 0.0 ? converter.r_conv_ac_pu : 0.01;
    item.virtual_x_pu =
        converter.x_sc_pu > 0.0 ? converter.x_sc_pu : 0.10;
    item.voltage_reference_pu =
        converter.v_ac_set_pu > 0.0 ? converter.v_ac_set_pu : 1.0;
    opf.converters.push_back(std::move(item));
    model.vsc_component_indices.push_back(converter.index);
  }
  if (opf.converters.empty()) {
    throw std::invalid_argument(
        "No VSC has both an in-service three-phase AC and DC terminal");
  }
  if (opf.dc_reference_terminals.empty()) {
    const int terminal = opf.converters.front().dc_terminal;
    opf.dc_reference_terminals.push_back(terminal);
    opf.dc_reference_voltage_pu =
        Eigen::VectorXd::Constant(1, opf.v_dc_start[terminal]);
    model.model_limitations.push_back(
        "No DC_V bus was declared; the first VSC terminal anchors DC voltage");
  }
  model.model_limitations.push_back(
      "The adapter supports phase-domain AC admittance, resistive DC branches, "
      "direct VSC ports, and constant-power nodal load equations");
  return model;
}

}  // namespace hacdcpf::opf::phase_hybrid
