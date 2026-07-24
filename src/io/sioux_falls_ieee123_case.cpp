#include "hacdcpf/io/sioux_falls_ieee123_case.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

#include "hacdcpf/ev_power_traffic/route_generation.hpp"
#include "hacdcpf/io/external_grid_io.hpp"
#include "hacdcpf/io/tntp_parser.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace hacdcpf::io {
namespace {

using namespace hacdcpf::evpt;

struct StationSeed {
  int traffic_node;
  const char* opendss_bus;
  std::array<double, 3> phase_power_share;
};

constexpr StationSeed kStationSeeds[] = {
    {2, "13", {0.50, 0.30, 0.20}},
    {5, "35", {0.20, 0.50, 0.30}},
    {8, "60", {0.30, 0.20, 0.50}},
    {10, "76", {0.45, 0.35, 0.20}},
    {15, "83", {0.20, 0.45, 0.35}},
    {18, "97", {0.35, 0.20, 0.45}},
    {20, "105", {0.40, 0.35, 0.25}},
    {24, "114", {1.00, 0.00, 0.00}}};

int power_bus_named(const HybridPowerSystem& system, const std::string& name) {
  const auto it = std::find_if(
      system.ac.buses.begin(), system.ac.buses.end(),
      [&](const ACBus& bus) { return bus.name == name; });
  if (it == system.ac.buses.end()) {
    throw std::runtime_error(
        "Sioux Falls--IEEE 123: OpenDSS bus is absent: " + name);
  }
  return it->index;
}

int three_phase_bus_named(const ThreePhaseACSystem& system,
                          const std::string& name) {
  const auto it = std::find_if(
      system.buses.begin(), system.buses.end(),
      [&](const ThreePhaseACBus& bus) { return bus.name == name; });
  if (it == system.buses.end()) {
    throw std::runtime_error(
        "Sioux Falls--IEEE 123: three-phase OpenDSS bus is absent: " + name);
  }
  return it->index;
}

void prepare_power_system(SiouxFallsIEEE123Case& result) {
  auto& system = result.problem.system;
  if (!system.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "Sioux Falls--IEEE 123: native three-phase system was not loaded");
  }

  // The positive-sequence projection remains available only for screening.
  // Closed OpenDSS switches are represented as low-impedance branches there;
  // the paper's main recourse model uses the native phase-domain subsystem.
  int next_branch = 1;
  for (const auto& branch : system.ac.branches) {
    next_branch = std::max(next_branch, branch.index + 1);
  }
  for (const auto& sw : system.ac.switches) {
    if (!sw.in_service || !sw.closed) continue;
    ACBranch branch;
    branch.index = next_branch++;
    branch.name = "closed-switch-" + sw.name;
    branch.from_bus = sw.bus_from;
    branch.to_bus = sw.bus_to;
    branch.r_pu = 0.0;
    branch.x_pu = 1e-4;
    branch.rate_a_mva = 5.0;
    branch.in_service = true;
    system.ac.branches.push_back(branch);
  }
  system.ac.switches.clear();
  for (auto& branch : system.ac.branches) {
    if (branch.rate_a_mva <= 0.0) branch.rate_a_mva = 5.0;
    if (std::abs(branch.x_pu) < 1e-6) branch.x_pu = 1e-4;
  }

  for (auto& generator : system.ac.generators) {
    generator.pmin_mw = 0.0;
    generator.pmax_mw = 6.0;
    generator.cost_c2 = 4.0;
    generator.cost_c1 = 35.0;
    generator.cost_c0 = 0.0;
    generator.ramp_up_mw_min = 0.020;
    generator.ramp_dn_mw_min = 0.020;
  }

  const std::vector<std::pair<std::string, double>> dg = {
      {"35", 72.0}, {"60", 84.0}, {"97", 96.0}, {"105", 108.0}};
  int next_generator = static_cast<int>(system.ac.generators.size()) + 1;
  for (const auto& [bus_name, marginal_cost] : dg) {
    Generator generator;
    generator.index = next_generator++;
    generator.bus = power_bus_named(system, bus_name);
    generator.name = "scenario-DG-" + bus_name;
    generator.in_service = true;
    generator.pmin_mw = 0.0;
    generator.pmax_mw = 0.45;
    generator.qmin_mvar = -0.25;
    generator.qmax_mvar = 0.25;
    generator.cost_c2 = 8.0;
    generator.cost_c1 = marginal_cost;
    generator.ramp_up_mw_min = 0.008;
    generator.ramp_dn_mw_min = 0.008;
    system.ac.generators.push_back(generator);

    ThreePhaseGenerator phase_generator;
    phase_generator.index = generator.index;
    phase_generator.bus =
        three_phase_bus_named(*system.three_phase_ac, bus_name);
    phase_generator.name = generator.name;
    phase_generator.in_service = true;
    const auto phase_bus = std::find_if(
        system.three_phase_ac->buses.begin(),
        system.three_phase_ac->buses.end(),
        [&](const ThreePhaseACBus& bus) {
          return bus.index == phase_generator.bus;
        });
    if (phase_bus == system.three_phase_ac->buses.end()) {
      throw std::runtime_error(
          "Sioux Falls--IEEE 123: DG phase bus is absent: " + bus_name);
    }
    phase_generator.phase_mask = phase_bus->phase_mask;
    phase_generator.p_mw = 0.0;
    phase_generator.q_mvar = 0.0;
    phase_generator.pmin_mw = generator.pmin_mw;
    phase_generator.pmax_mw = generator.pmax_mw;
    phase_generator.qmin_mvar = generator.qmin_mvar;
    phase_generator.qmax_mvar = generator.qmax_mvar;
    system.three_phase_ac->generators.push_back(std::move(phase_generator));
  }

  int station_id = 1001;
  for (const auto& seed : kStationSeeds) {
    const int bus = power_bus_named(system, seed.opendss_bus);
    const int phase_bus =
        three_phase_bus_named(*system.three_phase_ac, seed.opendss_bus);
    ChargingStation station;
    station.index = station_id;
    station.name = "SF-node-" + std::to_string(seed.traffic_node);
    station.location = "Sioux Falls traffic node " +
                       std::to_string(seed.traffic_node);
    station.bus = bus;
    station.in_service = true;
    station.n_fast = 12;
    station.num_chargers = 12;
    station.p_fast_max_kw = 50.0;
    station.max_power_kw = result.options.station_power_kw;
    station.power_factor = 0.98;
    system.ac.charging_stations.push_back(station);
    result.station_mapping.push_back(
        {station_id, seed.traffic_node, seed.opendss_bus, bus, phase_bus,
         seed.phase_power_share});
    ++station_id;
  }

  // Four-terminal DC overlay used by the monolithic three-phase AC/DC OPF.
  // The overlay and VSC locations are study assumptions, not IEEE feeder data.
  for (int i = 0; i < 4; ++i) {
    DCBus bus;
    bus.index = 5001 + i;
    bus.name = "scenario-DC-" + std::to_string(i + 1);
    bus.bus_type = i == 0 ? DCBusType::DC_V : DCBusType::DC_P;
    bus.vm_pu = 1.0;
    bus.vmin_pu = 0.90;
    bus.vmax_pu = 1.10;
    bus.base_kv = 1.5;
    bus.in_service = true;
    system.dc.buses.push_back(bus);
  }
  for (int i = 0; i < 3; ++i) {
    DCBranch branch;
    branch.index = 6001 + i;
    branch.name = "scenario-DC-link-" + std::to_string(i + 1);
    branch.from_bus = 5001 + i;
    branch.to_bus = 5002 + i;
    branch.r_pu = 0.02;
    branch.rate_a_mva = 1.5;
    branch.s_max_mva = 1.5;
    branch.in_service = true;
    system.dc.branches.push_back(branch);
  }
  const std::vector<std::string> converter_ac_buses = {"35", "60", "97", "105"};
  for (int i = 0; i < 4; ++i) {
    VSCConverter converter;
    converter.index = 7001 + i;
    converter.name = "scenario-VSC-" + std::to_string(i + 1);
    converter.bus_ac = three_phase_bus_named(
        *system.three_phase_ac,
        converter_ac_buses[static_cast<std::size_t>(i)]);
    converter.bus_dc = 5001 + i;
    converter.in_service = true;
    converter.control_mode =
        i == 0 ? ConverterMode::VDC_Q : ConverterMode::PQ_MODE;
    converter.grid_forming = i == 0;
    converter.p_is_hard_constraint = false;
    converter.eta = 0.98;
    converter.pmax_mw = result.options.converter_capacity_scale;
    converter.pmin_mw = -result.options.converter_capacity_scale;
    converter.qmin_mvar = -0.5 * result.options.converter_capacity_scale;
    converter.qmax_mvar = 0.5 * result.options.converter_capacity_scale;
    converter.p_rated_mw = result.options.converter_capacity_scale;
    converter.i_ac_max_pu = 1.2 * result.options.converter_capacity_scale;
    converter.r_conv_ac_pu = 0.01;
    converter.x_sc_pu = 0.10;
    converter.v_dc_set_pu = 1.0;
    converter.v_ac_set_pu = 1.0;
    system.vsc_converters.push_back(converter);
  }
}

void validate_options(const SiouxFallsIEEE123Options& options) {
  if (options.selected_od_pairs <= 0 || options.paths_per_od <= 0 ||
      options.num_steps <= 0 || options.time_step_hr <= 0.0 ||
      options.ctm_substeps_per_power_step <= 0 ||
      options.od_demand_scale <= 0.0 || options.road_capacity_scale <= 0.0 ||
      options.ev_penetration < 0.0 || options.ev_penetration > 1.0 ||
      options.station_power_kw <= 0.0 || options.charging_dwell_steps <= 0 ||
      options.base_load_scale <= 0.0 || options.phase_voltage_min_pu <= 0.0 ||
      options.phase_voltage_min_pu >= options.phase_voltage_max_pu ||
      options.phase_voltage_max_pu <= 0.0 || options.vuf_limit <= 0.0 ||
      options.vuf_limit >= 1.0 || options.converter_capacity_scale <= 0.0 ||
      options.departure_steps.empty() ||
      options.departure_steps.size() != options.departure_shares.size()) {
    throw std::runtime_error("Sioux Falls--IEEE 123: invalid case options");
  }
  const double share_sum = std::accumulate(
      options.departure_shares.begin(), options.departure_shares.end(), 0.0);
  if (std::abs(share_sum - 1.0) > 1e-9 ||
      std::any_of(options.departure_shares.begin(), options.departure_shares.end(),
                  [](double value) { return value < 0.0; })) {
    throw std::runtime_error(
        "Sioux Falls--IEEE 123: departure shares must be nonnegative and sum to one");
  }
  for (int step : options.departure_steps) {
    if (step < 0 || step >= options.num_steps) {
      throw std::runtime_error(
          "Sioux Falls--IEEE 123: departure step is outside horizon");
    }
  }
}

}  // namespace

SiouxFallsIEEE123Case build_sioux_falls_ieee123_case(
    const SiouxFallsIEEE123Options& options) {
  validate_options(options);
  SiouxFallsIEEE123Case result;
  result.options = options;

  const auto sioux_dir =
      options.external_data_root / "transportation" / "SiouxFalls";
  const auto dss_master =
      options.external_data_root / "opendss_ieee_pes" / "opendss_reference" /
      "123_node" / "IEEE123Master.dss";
  result.traffic_source = (sioux_dir / "SiouxFalls_net.tntp").string();
  result.power_source = dss_master.string();
  result.power_model_scope =
      "native OpenDSS three-phase unbalanced IEEE 123 feeder with a "
      "four-terminal study DC overlay and four VSCs for monolithic "
      "three-phase AC/DC hybrid OPF";
  result.screening_power_model_scope =
      "balanced positive-sequence OpenDSS projection retained only for "
      "fast screening and ablation";

  const auto network =
      parse_tntp_network((sioux_dir / "SiouxFalls_net.tntp").string());
  const auto nodes =
      parse_tntp_nodes((sioux_dir / "SiouxFalls_node.tntp").string());
  const auto trips =
      parse_tntp_trips((sioux_dir / "SiouxFalls_trips.tntp").string());
  TNTPImportOptions traffic_options;
  traffic_options.time_unit = TNTPTimeUnit::Minutes;
  traffic_options.length_unit = TNTPLengthUnit::Miles;
  result.problem.traffic = make_traffic_graph(network, nodes, traffic_options);
  for (auto& link : result.problem.traffic.links) {
    link.capacity_veh_per_hr *= options.road_capacity_scale;
    link.jam_vehicles *= options.road_capacity_scale;
  }

  OpenDSSImportOptions power_options;
  power_options.mode = ImportMode::Permissive;
  power_options.default_base_mva = 10.0;
  power_options.default_base_kv_ll = 4.16;
  power_options.default_frequency_hz = 60.0;
  power_options.convert_actual_to_per_unit = true;
  power_options.create_slack_generator = true;
  auto power_report = load_opendss_with_report(dss_master, power_options);
  result.problem.system = std::move(power_report.system);
  result.problem.system.three_phase_ac =
      analysis::load_three_phase_system_from_opendss(dss_master, 10.0);
  result.problem.system.name = "Sioux Falls--OpenDSS IEEE 123 coupled case";
  result.power_import_warnings = std::move(power_report.warnings);
  result.power_import_skipped = std::move(power_report.skipped);
  prepare_power_system(result);
  opf::phase_hybrid::ThreePhaseHybridAdapterOptions adapter_options;
  adapter_options.vuf_max = options.vuf_limit;
  result.phase_hybrid_model = opf::phase_hybrid::build_three_phase_hybrid_model(
      result.problem.system, adapter_options);
  result.phase_hybrid_model.opf.v_min_pu =
      result.phase_hybrid_model.opf.v_min_pu.cwiseMax(
          options.phase_voltage_min_pu);
  result.phase_hybrid_model.opf.v_max_pu =
      result.phase_hybrid_model.opf.v_max_pu.cwiseMin(
          options.phase_voltage_max_pu);

  std::unordered_map<int, int> destination_station;
  for (const auto& mapping : result.station_mapping) {
    destination_station[mapping.traffic_node] = mapping.station_id;
  }
  struct RankedOD {
    int origin;
    int destination;
    double flow;
  };
  std::vector<RankedOD> ranked;
  for (const auto& [origin, destinations] : trips.demand) {
    for (const auto& [destination, flow] : destinations) {
      if (origin != destination && flow > 0.0 &&
          destination_station.count(destination)) {
        ranked.push_back({origin, destination, flow});
      }
    }
  }
  std::sort(ranked.begin(), ranked.end(), [](const RankedOD& lhs,
                                             const RankedOD& rhs) {
    if (lhs.flow != rhs.flow) return lhs.flow > rhs.flow;
    if (lhs.origin != rhs.origin) return lhs.origin < rhs.origin;
    return lhs.destination < rhs.destination;
  });

  int next_route = 1;
  int next_ev_demand = 1;
  int next_icv_demand = 10001;
  for (const auto& od : ranked) {
    if (static_cast<int>(result.selected_od.size()) >= options.selected_od_pairs) {
      break;
    }
    const auto paths = k_shortest_loopless_paths(
        result.problem.traffic, od.origin, od.destination,
        options.paths_per_od);
    if (paths.empty()) continue;

    SiouxFallsIEEE123ODPair selected;
    selected.origin_node = od.origin;
    selected.destination_node = od.destination;
    selected.tntp_flow = od.flow;
    for (const auto& path : paths) {
      RouteAlternative ev_route;
      ev_route.index = next_route++;
      ev_route.origin_node = od.origin;
      ev_route.destination_node = od.destination;
      ev_route.link_indices = path.link_indices;
      RouteChargingStop stop;
      stop.station_id = destination_station.at(od.destination);
      stop.requested_energy_kwh_per_vehicle =
          options.charging_energy_kwh_per_vehicle;
      stop.dwell_steps = options.charging_dwell_steps;
      stop.max_charge_kw_per_vehicle =
          options.charging_power_kw_per_vehicle;
      ev_route.charging_stops.push_back(stop);
      selected.ev_route_indices.push_back(ev_route.index);
      result.problem.routes.push_back(std::move(ev_route));

      RouteAlternative icv_route;
      icv_route.index = next_route++;
      icv_route.origin_node = od.origin;
      icv_route.destination_node = od.destination;
      icv_route.link_indices = path.link_indices;
      selected.icv_route_indices.push_back(icv_route.index);
      result.problem.routes.push_back(std::move(icv_route));
    }

    for (std::size_t cohort = 0; cohort < options.departure_steps.size(); ++cohort) {
      const double cohort_flow = od.flow * options.od_demand_scale *
                                 options.departure_shares[cohort];
      EVDemand ev;
      ev.index = next_ev_demand++;
      ev.origin_node = od.origin;
      ev.destination_node = od.destination;
      ev.departure_step = options.departure_steps[cohort];
      ev.vehicles = cohort_flow * options.ev_penetration;
      ev.candidate_route_indices = selected.ev_route_indices;
      ev.initial_energy_kwh = 52.0;
      ev.energy_min_kwh = 6.0;
      ev.energy_max_kwh = 78.0;
      ev.reserve_energy_kwh = 5.0;
      ev.willingness_to_pay_per_vehicle = 80.0;
      ev.value_of_time_per_hr = 24.0;
      result.problem.demands.push_back(std::move(ev));

      ICVDemand icv;
      icv.index = next_icv_demand++;
      icv.origin_node = od.origin;
      icv.destination_node = od.destination;
      icv.departure_step = options.departure_steps[cohort];
      icv.vehicles = cohort_flow * (1.0 - options.ev_penetration);
      icv.candidate_route_indices = selected.icv_route_indices;
      icv.value_of_time_per_hr = 20.0;
      icv.fuel_cost_per_km = 0.10;
      result.problem.icv_demands.push_back(std::move(icv));
    }
    result.selected_od.push_back(std::move(selected));
  }
  if (static_cast<int>(result.selected_od.size()) < options.selected_od_pairs) {
    throw std::runtime_error(
        "Sioux Falls--IEEE 123: insufficient connected OD pairs");
  }

  const std::vector<double> daily_base_load_multiplier = {
      0.82, 0.82, 0.84, 0.86, 0.90, 0.95, 1.00, 1.04,
      1.07, 1.10, 1.08, 1.05, 1.02, 1.00, 0.98, 0.97,
      0.96, 0.95, 0.94, 0.92, 0.90, 0.88, 0.86, 0.84};
  result.base_load_multiplier.reserve(static_cast<std::size_t>(options.num_steps));
  for (int step = 0; step < options.num_steps; ++step) {
    result.base_load_multiplier.push_back(
        daily_base_load_multiplier[static_cast<std::size_t>(
            step % static_cast<int>(daily_base_load_multiplier.size()))]);
  }
  for (const auto& mapping : result.station_mapping) {
    StationPriceProfile price;
    price.station_id = mapping.station_id;
    price.price_per_kwh.resize(static_cast<std::size_t>(options.num_steps));
    for (int k = 0; k < options.num_steps; ++k) {
      price.price_per_kwh[static_cast<std::size_t>(k)] =
          0.035 + 0.055 * result.base_load_multiplier[static_cast<std::size_t>(k)];
    }
    result.problem.station_prices.push_back(std::move(price));
  }
  return result;
}

SiouxFallsIEEE123Recourse build_sioux_falls_ieee123_recourse(
    const SiouxFallsIEEE123Case& data,
    const evpt::CTMForwardResult& forward,
    const SiouxFallsIEEE123RecourseOptions& options) {
  if (!forward.valid || forward.simulation.steps_per_sim_step <= 0 ||
      !(options.charging_efficiency > 0.0) ||
      options.charging_efficiency > 1.0 ||
      !(options.charging_power_factor > 0.0) ||
      options.charging_power_factor > 1.0) {
    throw std::invalid_argument(
        "Sioux Falls--IEEE 123 recourse: invalid CTM result or options");
  }

  const int horizon = data.options.num_steps;
  const int substeps = forward.simulation.steps_per_sim_step;
  const double dt_hr = data.options.time_step_hr;
  SiouxFallsIEEE123Recourse result;
  result.source_entry_shortfall_vehicles =
      forward.source_entry_shortfall_vehicles;
  result.scheduling_scope =
      options.arrival_opens_until_horizon
          ? "arrival-monotone station dispatch with a common recovery "
            "horizon, followed by monolithic phase-domain AC/DC OPF recourse"
          : "earliest-deadline-first station dispatch with fixed dwell "
            "windows, followed by monolithic phase-domain AC/DC OPF recourse";
  result.aggregate_backlog_kwh.assign(static_cast<std::size_t>(horizon), 0.0);

  struct ChargingCohort {
    int deadline_step{0};
    double remaining_battery_kwh{0.0};
  };
  for (const auto& mapping : data.station_mapping) {
    auto& arrivals = result.station_arrivals_vehicles[mapping.station_id];
    arrivals.assign(static_cast<std::size_t>(horizon), 0.0);
    const auto raw = forward.simulation.station_arrivals.find(mapping.station_id);
    if (raw != forward.simulation.station_arrivals.end()) {
      for (int k = 0; k < static_cast<int>(raw->second.size()); ++k) {
        // A CTM flow recorded in substep k reaches the station at the end of
        // that substep. Charging can therefore start only at the next power-
        // interval boundary, not retroactively within the containing interval.
        const int charging_step = (k + substeps) / substeps;
        if (charging_step >= 0 && charging_step < horizon) {
          arrivals[static_cast<std::size_t>(charging_step)] +=
              raw->second[static_cast<std::size_t>(k)];
        }
      }
    }

    auto& station_load = result.station_grid_load_kw[mapping.station_id];
    station_load.assign(static_cast<std::size_t>(horizon), 0.0);
    auto& station_backlog = result.station_backlog_kwh[mapping.station_id];
    station_backlog.assign(static_cast<std::size_t>(horizon), 0.0);
    std::deque<ChargingCohort> active;
    for (int step = 0; step < horizon; ++step) {
      while (!active.empty() && active.front().deadline_step <= step) {
        result.charging_window_shortfall_kwh +=
            active.front().remaining_battery_kwh;
        active.pop_front();
      }
      const double arriving_vehicles = arrivals[static_cast<std::size_t>(step)];
      if (arriving_vehicles > 0.0) {
        const double energy = arriving_vehicles *
                              data.options.charging_energy_kwh_per_vehicle;
        active.push_back(
            {options.arrival_opens_until_horizon
                 ? horizon
                 : std::min(horizon,
                            step + data.options.charging_dwell_steps),
             energy});
        result.total_arrivals_vehicles += arriving_vehicles;
        result.requested_battery_energy_kwh += energy;
      }

      double available_grid_kw = data.options.station_power_kw;
      for (auto& cohort : active) {
        if (available_grid_kw <= 1e-9) break;
        const double grid_kw = std::min(
            available_grid_kw,
            cohort.remaining_battery_kwh /
                (options.charging_efficiency * dt_hr));
        station_load[static_cast<std::size_t>(step)] += grid_kw;
        available_grid_kw -= grid_kw;
        const double delivered =
            grid_kw * options.charging_efficiency * dt_hr;
        cohort.remaining_battery_kwh =
            std::max(0.0, cohort.remaining_battery_kwh - delivered);
        result.delivered_battery_energy_kwh += delivered;
      }
      while (!active.empty() &&
             active.front().remaining_battery_kwh <= 1e-9) {
        active.pop_front();
      }
      const double backlog = std::accumulate(
          active.begin(), active.end(), 0.0,
          [](double sum, const ChargingCohort& cohort) {
            return sum + cohort.remaining_battery_kwh;
          });
      station_backlog[static_cast<std::size_t>(step)] = backlog;
      result.aggregate_backlog_kwh[static_cast<std::size_t>(step)] += backlog;
      result.maximum_station_backlog_kwh =
          std::max(result.maximum_station_backlog_kwh, backlog);
      result.backlog_time_integral_kwh_hr += backlog * dt_hr;
      if (backlog > 1e-9 &&
          station_load[static_cast<std::size_t>(step)] >=
              data.options.station_power_kw - 1e-7) {
        ++result.station_saturated_intervals;
      }
    }
    for (const auto& cohort : active) {
      result.charging_window_shortfall_kwh +=
          cohort.remaining_battery_kwh;
    }
  }

  result.opf_cases.reserve(static_cast<std::size_t>(horizon));
  const auto& base = data.phase_hybrid_model.opf;
  const double reactive_ratio =
      std::tan(std::acos(options.charging_power_factor));
  for (int step = 0; step < horizon; ++step) {
    auto problem = base;
    problem.name = base.name + "_step_" + std::to_string(step);
    const double multiplier = data.options.base_load_scale *
        ((options.scale_base_load_profile &&
          step < static_cast<int>(data.base_load_multiplier.size()))
             ? data.base_load_multiplier[static_cast<std::size_t>(step)]
             : 1.0);
    problem.p_load_pu = base.p_load_pu * multiplier;
    problem.q_load_pu = base.q_load_pu * multiplier;
    for (const auto& mapping : data.station_mapping) {
      const double station_mw =
          result.station_grid_load_kw.at(mapping.station_id)
              [static_cast<std::size_t>(step)] /
          1000.0;
      int available_phases = 0;
      if (data.options.balanced_station_phase_allocation) {
        for (int phase = 0; phase < 3; ++phase) {
          available_phases += data.phase_hybrid_model.phase_node(
                                  mapping.three_phase_bus_index, phase) >= 0
              ? 1
              : 0;
        }
      }
      for (int phase = 0; phase < 3; ++phase) {
        const double share = data.options.balanced_station_phase_allocation
            ? (data.phase_hybrid_model.phase_node(
                   mapping.three_phase_bus_index, phase) >= 0 &&
               available_phases > 0
                   ? 1.0 / static_cast<double>(available_phases)
                   : 0.0)
            : mapping.phase_power_share[static_cast<std::size_t>(phase)];
        if (share <= 0.0) continue;
        const int node = data.phase_hybrid_model.phase_node(
            mapping.three_phase_bus_index, phase);
        if (node < 0) {
          throw std::runtime_error(
              "Sioux Falls--IEEE 123 recourse: station phase is absent");
        }
        problem.p_load_pu[node] += station_mw * share / problem.base_mva;
        problem.q_load_pu[node] +=
            station_mw * reactive_ratio * share / problem.base_mva;
      }
    }
    result.opf_cases.push_back(std::move(problem));
  }
  return result;
}

}  // namespace hacdcpf::io
