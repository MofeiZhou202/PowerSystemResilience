#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"
#include "hacdcpf/model/typical_parameters.hpp"
#include "hacdcpf/model/typed_ids.hpp"
#include "hacdcpf/model/unit_conversion.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/projection/result_attribution.hpp"
#include "hacdcpf/validation/validate_system.hpp"

using Catch::Matchers::WithinAbs;
using namespace hacdcpf;

namespace {

ACBus make_bus(int index, BusType type, double base_kv = 10.0) {
  ACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.base_kv = base_kv;
  return bus;
}

HybridPowerSystem make_two_bus_system() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};

  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.05;
  branch.rate_a_mva = 100.0;
  sys.ac.branches.push_back(branch);

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.pmax_mw = 200.0;
  generator.qmax_mvar = 100.0;
  generator.qmin_mvar = -100.0;
  sys.ac.generators.push_back(generator);
  return sys;
}

bool has_validation_field(const validation::ValidationReport& report,
                          const std::string& field) {
  return std::any_of(report.issues.begin(), report.issues.end(),
                     [&](const auto& issue) { return issue.field == field; });
}

const projection::RichComponentResult& attributed_row(
    const projection::RichResultAttribution& result, const std::string& type,
    int index) {
  const auto it = std::find_if(
      result.components.begin(), result.components.end(), [&](const auto& row) {
        return row.component_type == type && row.component_index == index;
      });
  REQUIRE(it != result.components.end());
  return *it;
}

double attributed_value(const projection::RichComponentResult& row,
                        const std::string& name) {
  const auto it = std::find_if(row.values.begin(), row.values.end(),
                               [&](const auto& value) {
                                 return value.name == name;
                               });
  REQUIRE(it != row.values.end());
  return it->value;
}

}  // namespace

TEST_CASE("A1 flexible demand replaces its projected active-power baseline",
          "[model_audit][opf][flexible_load]") {
  auto sys = make_two_bus_system();

  Load fixed;
  fixed.index = 1;
  fixed.bus = 2;
  fixed.p_mw = 6.0;
  fixed.q_mvar = 3.0;
  fixed.z_percent_p = 20.0;
  fixed.i_percent_p = 30.0;
  fixed.p_percent_p = 50.0;
  sys.ac.loads.push_back(fixed);

  FlexibleLoad flexible;
  flexible.index = 2;
  flexible.bus = 2;
  flexible.p_mw = 4.0;
  flexible.q_mvar = 1.0;
  flexible.flex_up_mw = 1.0;
  flexible.flex_down_mw = 2.0;
  sys.ac.flexible_loads.push_back(flexible);

  const auto problem = opf::parity::build_problem(sys);
  REQUIRE(problem.pd_demand_pu.size() == 2);
  REQUIRE(problem.flex_var_to_data.size() == 1);
  CHECK_THAT(problem.pd_demand_pu[1], WithinAbs(0.06, 1e-12));
  CHECK_THAT(problem.qd_demand_pu[1], WithinAbs(0.04, 1e-12));
  CHECK_THAT(problem.zip_pp[1], WithinAbs(0.50, 1e-12));
  CHECK_THAT(problem.zip_ip[1], WithinAbs(0.30, 1e-12));
  CHECK_THAT(problem.zip_zp[1], WithinAbs(0.20, 1e-12));
}

TEST_CASE("A2 charger aggregation is idempotent and requires a qualified station",
          "[model_audit][charger]") {
  auto sys = make_two_bus_system();
  ChargingStation station;
  station.index = 42;
  station.bus = 2;
  station.power_factor = 0.95;
  station.p_total_kw = 999.0;
  station.q_total_kvar = 999.0;
  station.num_chargers = 99;
  sys.ac.charging_stations.push_back(station);

  Charger fast;
  fast.index = 1;
  fast.station_id = 42;
  fast.p_ch_max_kw = 60.0;
  Charger slow = fast;
  slow.index = 2;
  slow.p_ch_max_kw = 40.0;
  sys.ac.chargers = {fast, slow};

  const auto projected = project_to_canonical_models(sys);
  REQUIRE(projected.ac.charging_stations.size() == 1);
  const auto& aggregate = projected.ac.charging_stations.front();
  CHECK_THAT(aggregate.p_total_kw, WithinAbs(100.0, 1e-12));
  CHECK_THAT(aggregate.q_total_kvar,
             WithinAbs(100.0 * std::tan(std::acos(0.95)), 1e-12));
  CHECK(aggregate.num_chargers == 2);
  CHECK(projected.ac.chargers.empty());

  auto charger_only = make_two_bus_system();
  charger_only.ac.chargers.push_back(fast);
  CHECK_THROWS_AS(project_to_canonical_models(charger_only),
                  std::invalid_argument);
}

TEST_CASE("A3 DC energy-router ports reuse DC buses without synthetic VSCs",
          "[model_audit][energy_router][domain]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.dc.base_mva = 100.0;
  DCBus a;
  a.index = 10;
  a.base_kv = 0.75;
  DCBus b = a;
  b.index = 20;
  sys.dc.buses = {a, b};

  EnergyRouter router;
  router.index = 7;
  router.p_rated_mw = 5.0;
  EnergyRouterPort pa;
  pa.index = 1;
  pa.bus = 10;
  pa.side = 0;
  pa.port_type = ERPortType::DC;
  EnergyRouterPort pb = pa;
  pb.index = 2;
  pb.bus = 20;
  pb.side = 1;
  router.ports = {pa, pb};
  sys.energy_routers.push_back(router);

  const auto projected = project_to_canonical_models(sys, false);
  CHECK(projected.vsc_converters.empty());
  REQUIRE(projected.dc.dcdc_converters.size() == 1);
  CHECK(projected.dc.buses.size() == 2);
  CHECK(projected.dc.dcdc_converters.front().bus_in == 1);
  CHECK(projected.dc.dcdc_converters.front().bus_out == 2);

  HybridPowerSystem dispatched;
  dispatched.base_mva = dispatched.dc.base_mva = 100.0;
  dispatched.ac.buses = {make_bus(1, BusType::SLACK),
                         make_bus(2, BusType::PQ),
                         make_bus(3, BusType::PQ)};
  dispatched.dc.buses.push_back(a);
  EnergyRouter signed_router;
  signed_router.index = 8;
  signed_router.p_rated_mw = 10.0;
  EnergyRouterPort dc_port = pa;
  EnergyRouterPort vf;
  vf.index = 2;
  vf.bus = 1;
  vf.side = 1;
  vf.port_type = ERPortType::AC;
  vf.control_mode = ERControlMode::VF;
  EnergyRouterPort export_port = vf;
  export_port.index = 3;
  export_port.bus = 2;
  export_port.control_mode = ERControlMode::PQ;
  export_port.p_set_mw = 2.0;
  EnergyRouterPort import_port = export_port;
  import_port.index = 4;
  import_port.bus = 3;
  import_port.p_set_mw = -0.5;
  signed_router.ports = {dc_port, vf, export_port, import_port};
  dispatched.energy_routers.push_back(signed_router);
  const auto signed_projection = project_to_canonical_models(dispatched, false);
  REQUIRE(signed_projection.dc.dcdc_converters.size() == 1);
  CHECK_THAT(signed_projection.dc.dcdc_converters.front().p_ref_mw,
             WithinAbs(1.5, 1e-12));
}

TEST_CASE("parallel transformer projection aggregates all sequence impedances and ratings",
          "[model_audit][projection][transformer]") {
  auto sys = make_two_bus_system();
  sys.ac.branches.clear();
  Transformer2W transformer;
  transformer.index = 9;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.sn_mva = 10.0;
  transformer.vn_hv_kv = 10.0;
  transformer.vn_lv_kv = 10.0;
  transformer.vk_percent = 10.0;
  transformer.vkr_percent = 0.0;
  transformer.fixed_tap_pu = 1.05;
  transformer.n_parallel = 2;
  sys.ac.transformers_2w.push_back(transformer);

  const auto projected = project_to_canonical_models(sys);
  REQUIRE(projected.ac.branches.size() == 1);
  const auto& branch = projected.ac.branches.front();
  CHECK_THAT(branch.x_pu, WithinAbs(0.5, 1e-12));
  CHECK_THAT(branch.x0_pu, WithinAbs(0.5, 1e-12));
  CHECK_THAT(branch.tap, WithinAbs(1.05, 1e-12));
  CHECK_THAT(branch.rate_a_mva, WithinAbs(20.0, 1e-12));
  CHECK_THAT(branch.sn_mva, WithinAbs(20.0, 1e-12));
  CHECK(branch.n_parallel == 1);
}

TEST_CASE("motor projection preserves motor-base per-unit short-circuit data",
          "[model_audit][projection][motor]") {
  auto sys = make_two_bus_system();
  AsynchronousMotor motor;
  motor.index = 8;
  motor.bus = 2;
  motor.sn_mva = 2.0;
  motor.cos_phi = 0.8;
  motor.r_pu = 0.02;
  motor.x_pu = 0.20;
  sys.ac.motors.push_back(motor);

  const auto projected = project_to_canonical_models(sys);
  const auto it = std::find_if(
      projected.ac.loads.begin(), projected.ac.loads.end(),
      [](const Load& load) { return load.sc_source_type == "AsynchronousMotor"; });
  REQUIRE(it != projected.ac.loads.end());
  CHECK_THAT(it->sn_mva, WithinAbs(2.0, 1e-12));
  CHECK_THAT(it->r_sc_pu, WithinAbs(0.02, 1e-12));
  CHECK_THAT(it->x_sub_pu, WithinAbs(0.20, 1e-12));
  CHECK_THAT(it->motor_percent, WithinAbs(1.0, 1e-12));
}

TEST_CASE("three-winding projection applies winding voltage bases and ratio taps",
          "[model_audit][projection][transformer3w]") {
  const auto project = [](double vn_mv_kv, double vn_lv_kv) {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;
    sys.ac.buses = {make_bus(1, BusType::SLACK, 110.0),
                    make_bus(2, BusType::PQ, 33.0),
                    make_bus(3, BusType::PQ, 11.0)};
    Transformer3W transformer;
    transformer.index = 1;
    transformer.hv_bus = 1;
    transformer.mv_bus = 2;
    transformer.lv_bus = 3;
    transformer.sn_hv_mva = 100.0;
    transformer.sn_mv_mva = 100.0;
    transformer.sn_lv_mva = 100.0;
    transformer.vn_hv_kv = 110.0;
    transformer.vn_mv_kv = vn_mv_kv;
    transformer.vn_lv_kv = vn_lv_kv;
    transformer.vk_hv_mv_percent = 10.0;
    transformer.vk_hv_lv_percent = 10.0;
    transformer.vk_mv_lv_percent = 10.0;
    sys.ac.transformers_3w.push_back(transformer);
    return project_to_canonical_models(sys, false);
  };

  const auto aligned = project(33.0, 11.0);
  const auto mismatched = project(30.0, 10.0);
  REQUIRE(aligned.ac.branches.size() == 3);
  REQUIRE(mismatched.ac.branches.size() == 3);
  const double expected_scale = (30.0 / 33.0) * (30.0 / 33.0);
  for (size_t i = 0; i < aligned.ac.branches.size(); ++i) {
    REQUIRE(std::abs(aligned.ac.branches[i].x_pu) > 1e-12);
    CHECK_THAT(mismatched.ac.branches[i].x_pu /
                   aligned.ac.branches[i].x_pu,
               WithinAbs(expected_scale, 1e-12));
  }
  CHECK_THAT(mismatched.ac.branches[0].tap, WithinAbs(1.1, 1e-12));
  CHECK_THAT(mismatched.ac.branches[1].tap, WithinAbs(1.1, 1e-12));
  CHECK_THAT(mismatched.ac.branches[2].tap, WithinAbs(1.0, 1e-12));
}

TEST_CASE("three-phase fallback projection keeps one base and motor/tap semantics",
          "[model_audit][projection][three_phase]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  ThreePhaseACSystem phase;
  phase.base_mva = 50.0;
  ThreePhaseACBus bus1;
  bus1.index = 1;
  bus1.bus_type = BusType::SLACK;
  bus1.base_kv = 10.0;
  ThreePhaseACBus bus2 = bus1;
  bus2.index = 2;
  bus2.bus_type = BusType::PQ;
  phase.buses = {bus1, bus2};
  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.length_km = 1.0;
  line.r1_ohm_per_km = 0.1;
  line.x1_ohm_per_km = 0.2;
  phase.lines.push_back(line);
  ThreePhaseTransformer transformer;
  transformer.index = 2;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.sn_mva = 10.0;
  transformer.vn_hv_kv = 10.0;
  transformer.vn_lv_kv = 10.0;
  transformer.vk_percent = 5.0;
  transformer.tap_side = 1;
  transformer.tap_min = -4;
  transformer.tap_max = 6;
  phase.transformers.push_back(transformer);
  ThreePhaseLoad load;
  load.index = 3;
  load.bus = 2;
  load.p_a_mw = load.p_b_mw = load.p_c_mw = 0.3;
  load.motor_percent = 50.0;
  load.lrc_pu = 5.0;
  load.x_r_ratio = 4.0;
  phase.loads.push_back(load);
  sys.three_phase_ac = phase;

  const auto projected = project_to_canonical_models(sys, false);
  CHECK_THAT(projected.ac.base_mva, WithinAbs(50.0, 1e-12));
  REQUIRE_FALSE(projected.ac.branches.empty());
  CHECK_THAT(projected.ac.branches.front().r_pu, WithinAbs(0.05, 1e-12));
  CHECK_THAT(projected.ac.branches.front().x_pu, WithinAbs(0.10, 1e-12));
  REQUIRE(projected.ac.transformers_2w.size() == 1);
  CHECK(projected.ac.transformers_2w.front().tap_side == 1);
  CHECK(projected.ac.transformers_2w.front().tap_min == -4);
  CHECK(projected.ac.transformers_2w.front().tap_max == 6);
  REQUIRE(projected.ac.loads.size() == 1);
  const double expected_r = 0.2 / std::sqrt(17.0);
  CHECK_THAT(projected.ac.loads.front().sn_mva, WithinAbs(0.9, 1e-12));
  CHECK_THAT(projected.ac.loads.front().r_sc_pu,
             WithinAbs(expected_r, 1e-12));
  CHECK_THAT(projected.ac.loads.front().x_sub_pu,
             WithinAbs(4.0 * expected_r, 1e-12));
}

TEST_CASE("zero-impedance merge protects ideal transformers and preserves shunt charging",
          "[model_audit][projection][merge]") {
  HybridPowerSystem protected_system;
  protected_system.base_mva = 100.0;
  protected_system.ac.buses = {make_bus(1, BusType::SLACK),
                               make_bus(2, BusType::PQ)};
  ACBranch tapped;
  tapped.index = 1;
  tapped.from_bus = 1;
  tapped.to_bus = 2;
  tapped.r_pu = 1e-6;
  tapped.x_pu = 1e-6;
  tapped.tap = 1.05;
  protected_system.ac.branches.push_back(tapped);
  merge_zero_impedance_buses(protected_system);
  CHECK(protected_system.ac.buses.size() == 2);

  HybridPowerSystem merged;
  merged.base_mva = 100.0;
  merged.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  ACBranch ideal = tapped;
  ideal.tap = 1.0;
  ACBranch charged = ideal;
  charged.index = 2;
  charged.r_pu = 0.01;
  charged.x_pu = 0.05;
  charged.b_pu = 0.20;
  merged.ac.branches = {ideal, charged};
  const auto original = merged;
  merge_zero_impedance_buses(merged);
  REQUIRE(merged.ac.buses.size() == 1);
  CHECK(merged.ac.branches.empty());
  CHECK_THAT(merged.ac.buses.front().bs_mvar, WithinAbs(20.0, 1e-12));
  REQUIRE(merged.bus_merge_map.has_value());
  const auto attribution = evaluate_attribution(
      original, merged, ObservableKind::ACBranchTerminalFlow);
  CHECK_FALSE(attribution.total());
}

TEST_CASE("provenance-marked numerical ties form a canonical quotient without merging short lines",
          "[model_audit][projection][merge][power_flow]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  auto slack = make_bus(1, BusType::SLACK);
  auto pv1 = make_bus(2, BusType::PV);
  auto pv2 = make_bus(3, BusType::PV);
  slack.vm_pu = 1.0;
  pv1.vm_pu = 0.98;
  pv2.vm_pu = 1.02;
  pv1.pd_mw = 20.0;
  pv1.qd_mvar = 5.0;
  sys.ac.buses = {slack, pv1, pv2};

  ACBranch feeder;
  feeder.index = 1;
  feeder.from_bus = 1;
  feeder.to_bus = 2;
  feeder.r_pu = 0.01;
  feeder.x_pu = 0.10;
  ACBranch numerical_tie;
  numerical_tie.index = 2;
  numerical_tie.from_bus = 2;
  numerical_tie.to_bus = 3;
  numerical_tie.x_pu = 1e-4;
  numerical_tie.ideal_connectivity = true;
  sys.ac.branches = {feeder, numerical_tie};

  Generator grid;
  grid.index = 1;
  grid.bus = 1;
  grid.is_slack = true;
  grid.qmin_mvar = -100.0;
  grid.qmax_mvar = 100.0;
  Generator g1;
  g1.index = 2;
  g1.bus = 2;
  g1.pg_mw = 10.0;
  g1.vg_pu = 0.98;
  g1.qmin_mvar = -20.0;
  g1.qmax_mvar = 20.0;
  Generator g2 = g1;
  g2.index = 3;
  g2.bus = 3;
  g2.vg_pu = 1.02;
  sys.ac.generators = {grid, g1, g2};

  const auto projected = project_to_canonical_models(sys, false);
  REQUIRE(projected.ac.buses.size() == 2);
  REQUIRE(projected.ac.branches.size() == 1);
  REQUIRE(projected.bus_merge_map.has_value());
  CHECK(projected.bus_merge_map->merge_records.size() == 1);

  PowerFlowOptions options;
  options.enable_pv_pq_conversion = true;
  options.tol = 1e-10;
  const auto result = solve_power_flow(sys, options);
  REQUIRE(result.converged);
  CHECK(result.reactive_limits.certified);
  CHECK(result.reactive_limits.max_violation_pu <= 1e-10);

  auto physical_short_line = sys;
  physical_short_line.ac.branches.back().ideal_connectivity = false;
  const auto unmerged = project_to_canonical_models(physical_short_line, false);
  CHECK(unmerged.ac.buses.size() == 3);
  CHECK(unmerged.ac.branches.size() == 2);
}

TEST_CASE("device residual is complete and shared across ambiguous parallel devices",
          "[model_audit][attribution][device_flow]") {
  HybridPowerSystem sys;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 4.0;
  load.q_mvar = 2.0;
  sys.ac.loads.push_back(load);
  RenewableGen renewable;
  renewable.index = 1;
  renewable.bus = 2;
  renewable.p_mw = 1.0;
  renewable.q_mvar = 0.6;
  sys.ac.renewable_gens.push_back(renewable);
  for (int index = 1; index <= 2; ++index) {
    CircuitBreaker breaker;
    breaker.index = index;
    breaker.bus_from = 1;
    breaker.bus_to = 2;
    breaker.closed = true;
    sys.ac.circuit_breakers.push_back(breaker);
  }

  const auto flows = compute_device_terminal_flows(sys, {});
  REQUIRE(flows.ac_circuit_breakers.size() == 2);
  for (const auto& flow : flows.ac_circuit_breakers) {
    CHECK_THAT(flow.pf_mw, WithinAbs(1.5, 1e-12));
    CHECK_THAT(flow.qf_mvar, WithinAbs(0.7, 1e-12));
  }
}

TEST_CASE("rich attribution applies scaling, zeros outages, and reports unsupported rows",
          "[model_audit][attribution][coverage]") {
  HybridPowerSystem rich;
  rich.base_mva = rich.ac.base_mva = 100.0;
  rich.ac.buses = {make_bus(1, BusType::SLACK)};
  AsymmetricLoad asymmetric;
  asymmetric.index = 11;
  asymmetric.bus = 1;
  asymmetric.pa_mw = 1.0;
  asymmetric.scaling = 2.0;
  rich.ac.asymmetric_loads.push_back(asymmetric);
  RenewableGen stopped;
  stopped.index = 12;
  stopped.bus = 1;
  stopped.p_mw = 5.0;
  stopped.q_mvar = 2.0;
  stopped.in_service = false;
  rich.ac.renewable_gens.push_back(stopped);
  ChargingStation unsupported;
  unsupported.index = 13;
  unsupported.bus = 1;
  rich.ac.charging_stations.push_back(unsupported);

  ProjectionOptions options;
  options.strip_dead_islands = false;
  const auto bundle = projection::RichToCanonicalOperator::apply(rich, options);
  const auto result = projection::CanonicalToRichOperator::apply(
      rich, bundle, nullptr, nullptr, nullptr);
  CHECK_THAT(attributed_value(attributed_row(result, "asymmetric_load", 11),
                              "p_mw"),
             WithinAbs(2.0, 1e-12));
  CHECK_THAT(attributed_value(attributed_row(result, "renewable_generator", 12),
                              "p_mw"),
             WithinAbs(0.0, 1e-12));
  CHECK_FALSE(result.coverage.total());
  CHECK(result.coverage.unsupported_components > 0);
}

TEST_CASE("model semantic helpers fail closed and round-trip Hydro",
          "[model_audit][contract]") {
  CHECK(renewable_type_str(RenewableType::Hydro) == "Hydro");
  CHECK(renewable_type_from_str("Hydro") == RenewableType::Hydro);

  VSCConverter converter;
  converter.control_mode = ConverterMode::VDC_Q;
  converter.k_vdc = 0.0;
  CHECK_FALSE(resolve_device_control_role(converter).provides_dc_v_reference);
  converter.k_vdc = 1.0;
  CHECK(resolve_device_control_role(converter).provides_dc_v_reference);

  StaticGeneratorDC source;
  source.p_set_mw = 0.0;
  source.pmax_mw = 10.0;
  source.controllable = false;
  CHECK(model::effective_capacity_mw(source) == 0.0);
  source.controllable = true;
  CHECK(model::effective_capacity_mw(source) == 10.0);
  CHECK(model::sanitize_scaling(std::numeric_limits<double>::infinity()) == 0.0);
}

TEST_CASE("capacitance conversion and DC result reporting use canonical units",
          "[model_audit][unit_conversion][dc_rating]") {
  HybridPowerSystem ac;
  ac.base_mva = ac.ac.base_mva = 100.0;
  ac.ac.freq_hz = 50.0;
  ac.ac.buses = {make_bus(1, BusType::SLACK, 110.0)};
  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 1;
  line.length_km = 1.0;
  line.r_pu = 0.1;
  line.x_pu = 0.2;
  line.c_nf_per_km = 10.0;
  ac.ac.branches.push_back(line);
  CHECK(convert_actual_to_per_unit(ac) == 1);
  const double expected_b =
      2.0 * 3.14159265358979323846 * 50.0 * 10.0e-9 * 121.0;
  CHECK_THAT(ac.ac.branches.front().b_pu, WithinAbs(expected_b, 1e-15));
  CHECK_THAT(ac.ac.branches.front().r_pu, WithinAbs(0.1, 1e-15));

  HybridPowerSystem dc;
  dc.base_mva = dc.dc.base_mva = 100.0;
  DCBus from;
  from.index = 1;
  from.base_kv = 1.0;
  DCBus to = from;
  to.index = 2;
  dc.dc.buses = {from, to};
  DCBranch dc_line;
  dc_line.index = 1;
  dc_line.from_bus = 1;
  dc_line.to_bus = 2;
  dc_line.r_pu = 0.1;
  dc_line.rate_a_mva = 10.0;
  dc_line.s_max_mva = 5.0;
  dc.dc.branches.push_back(dc_line);
  PowerFlowResult result;
  result.vdc = {1.0, 0.99};
  const auto json = nlohmann::json::parse(io::power_flow_result_to_json(dc, result));
  CHECK_THAT(json["dc_branch_flows"][0]["loading_pct"].get<double>(),
             WithinAbs(100.0, 1e-9));
}

TEST_CASE("validation rejects unrepresented loads and invalid topology",
          "[model_audit][validation]") {
  auto sys = make_two_bus_system();
  Load load;
  load.index = 4;
  load.bus = 2;
  load.p_mw = 1.0;
  load.model = LoadModel::Exponential;
  load.z_percent_p = 30.0;
  load.i_percent_p = 30.0;
  load.p_percent_p = 30.0;
  sys.ac.loads.push_back(load);
  sys.ac.buses.front().index = 0;
  sys.ac.branches.front().from_bus = 2;
  sys.ac.branches.front().to_bus = 2;

  const auto report = validation::validate(sys);
  CHECK(report.has_errors());
  CHECK(has_validation_field(report, "model"));
  CHECK(has_validation_field(report, "ZIP-P"));
  CHECK(has_validation_field(report, "index"));
  CHECK(has_validation_field(report, "from_bus/to_bus"));
}

TEST_CASE("typical and standard parameter paths preserve authored operating states",
          "[model_audit][parameters]") {
  HybridPowerSystem sys;
  Load load;
  load.index = 1;
  load.scaling = 0.0;
  sys.ac.loads.push_back(load);
  Shunt shunt;
  shunt.index = 1;
  shunt.n_steps = 5;
  shunt.current_step = 0;
  sys.ac.shunts.push_back(shunt);
  ChargingStation station;
  station.index = 1;
  station.n_fast = 2;
  station.n_slow = 1;
  sys.ac.charging_stations.push_back(station);
  DCStorage storage;
  storage.index = 1;
  storage.soc_min = 0.9;
  storage.soc_max = 0.2;
  sys.dc.dc_storage.push_back(storage);
  apply_typical_parameters(sys);
  CHECK(sys.ac.loads.front().scaling == 0.0);
  CHECK(sys.ac.shunts.front().current_step == 0);
  CHECK_THAT(sys.ac.charging_stations.front().max_power_kw,
             WithinAbs(127.0, 1e-12));
  CHECK(sys.dc.dc_storage.front().soc_min < sys.dc.dc_storage.front().soc_max);
  const auto library = make_standard_parameter_library();
  REQUIRE(library.find("dc_bus.base_kv") != nullptr);
  CHECK_THAT(library.find("dc_bus.base_kv")->default_value,
             WithinAbs(TypicalParameters::kDCDistributionKv, 1e-12));

  HybridPowerSystem invalid;
  VSCConverter vsc;
  vsc.index = 1;
  vsc.eta = 0.98;
  vsc.r_sc_pu = 2.0;
  vsc.x_sc_pu = 0.15;
  vsc.i_max_pu = 10.0;
  invalid.vsc_converters.push_back(vsc);
  const auto diagnostics = validate_component_parameters(invalid);
  CHECK(std::any_of(diagnostics.diagnostics.begin(),
                    diagnostics.diagnostics.end(), [](const auto& item) {
                      return item.parameter == "r_sc_pu";
                    }));
  CHECK(std::any_of(diagnostics.diagnostics.begin(),
                    diagnostics.diagnostics.end(), [](const auto& item) {
                      return item.parameter == "i_max_pu";
                    }));

  const auto handbook_projection = [](int parallel) {
    HybridPowerSystem model;
    model.base_mva = model.ac.base_mva = 1.0;
    model.ac.buses = {make_bus(1, BusType::SLACK, 0.4),
                      make_bus(2, BusType::PQ, 0.4)};
    ACBranch branch;
    branch.index = 1;
    branch.from_bus = 1;
    branch.to_bus = 2;
    branch.length_km = 0.1;
    branch.n_parallel = parallel;
    branch.conductor_model = "ZRC-YJV22-";
    branch.cross_section_mm2 = 240.0;
    branch.line_type = "LV cable";
    branch.parameter_source = "cim_model_cross_section_estimate";
    model.ac.branches.push_back(branch);
    return complete_design_handbook_parameters(model).suggestions.front();
  };
  const auto single = handbook_projection(1);
  const auto parallel = handbook_projection(2);
  CHECK_THAT(parallel.new_r_pu, WithinAbs(single.new_r_pu / 2.0, 1e-12));
  CHECK_THAT(parallel.new_x_pu, WithinAbs(single.new_x_pu / 2.0, 1e-12));
  CHECK_THAT(parallel.new_rate_a_mva,
             WithinAbs(single.new_rate_a_mva * 2.0, 1e-12));

  HybridPowerSystem bad_matrix;
  bad_matrix.three_phase_ac.emplace();
  ThreePhaseACLine matrix_line;
  matrix_line.index = 21;
  matrix_line.from_bus = 1;
  matrix_line.to_bus = 2;
  matrix_line.use_phase_matrix = true;
  matrix_line.r_matrix_pu[0] = std::numeric_limits<double>::quiet_NaN();
  matrix_line.x_matrix_pu[0] = 0.1;
  bad_matrix.three_phase_ac->lines.push_back(matrix_line);
  const auto matrix_report = validate_component_parameters(bad_matrix);
  CHECK(std::any_of(matrix_report.diagnostics.begin(),
                    matrix_report.diagnostics.end(), [](const auto& item) {
                      return item.code == "three_phase_line_zero_series_impedance";
                    }));
}

TEST_CASE("R-01/R-02: DC dead islands are detected and reported in the certificate",
          "[model_audit][projection][dc][dead_island]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
  // A healthy AC island with a slack source keeps AC strip from firing.
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.01;
  line.x_pu = 0.10;
  sys.ac.branches = {line};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  sys.ac.external_grids = {grid};

  // A DC bus with no source, no branch, and no converter feeding it: a DC dead
  // island the projection reports (but does not strip).
  DCBus orphan;
  orphan.index = 5;
  orphan.bus_type = DCBusType::DC_P;
  orphan.base_kv = 0.75;
  sys.dc.buses = {orphan};

  const auto projected = project_to_canonical_models(sys);
  REQUIRE(projected.projection_certificate.has_value());
  const auto& diags = projected.projection_certificate->diagnostics;
  const bool reported =
      std::any_of(diags.begin(), diags.end(), [](const std::string& d) {
        return d.find("DC dead-island") != std::string::npos;
      });
  CHECK(reported);
}

TEST_CASE("R-04: standalone merge honors ideal_connectivity above the numeric threshold",
          "[model_audit][projection][merge]") {
  // Impedance well ABOVE kBusMergeZThreshold (1e-4): only the authored
  // ideal_connectivity flag — not the magnitude — may trigger contraction.
  HybridPowerSystem ideal;
  ideal.base_mva = ideal.ac.base_mva = 100.0;
  ideal.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  ACBranch tie;
  tie.index = 1;
  tie.from_bus = 1;
  tie.to_bus = 2;
  tie.r_pu = 0.02;
  tie.x_pu = 0.05;
  tie.ideal_connectivity = true;
  ideal.ac.branches = {tie};
  merge_zero_impedance_buses(ideal);
  CHECK(ideal.ac.buses.size() == 1);

  // Same impedance but NOT marked ideal: a real short line must remain a branch.
  HybridPowerSystem control;
  control.base_mva = control.ac.base_mva = 100.0;
  control.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  ACBranch real_line = tie;
  real_line.ideal_connectivity = false;
  control.ac.branches = {real_line};
  merge_zero_impedance_buses(control);
  CHECK(control.ac.buses.size() == 2);
}

TEST_CASE("typed IDs keep the three index spaces separate at compile time",
          "[model_audit][typed_ids]") {
  // Iron rule #2: the three integer index spaces must not be interchangeable.
  static_assert(!std::is_convertible_v<StableBusId, VectorPos>);
  static_assert(!std::is_convertible_v<VectorPos, NodeIdx>);
  static_assert(!std::is_convertible_v<NodeIdx, StableBusId>);
  static_assert(!std::is_convertible_v<int, StableBusId>);  // explicit only

  const StableBusId a{3};
  const StableBusId b{3};
  const VectorPos p{2};
  CHECK(a == b);
  CHECK(a.value() == 3);
  CHECK(p.value() == 2);
  CHECK_FALSE(StableBusId{}.valid());  // default is the invalid sentinel

  std::unordered_map<StableBusId, int> by_id;
  by_id[a] = 7;
  CHECK(by_id.at(b) == 7);
}
