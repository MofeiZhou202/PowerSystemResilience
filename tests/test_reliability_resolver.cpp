/// @file test_reliability_resolver.cpp
/// @brief Unit tests for the unified reliability parameter resolver and the
///        hybrid AC/DC customer-metric fix (docs/reliability_assessment_code_review.md).
///
/// Coverage:
///   - resolve_reliability_params() conversion rules (lambda+MTTR, MTBF(+/-MTTR),
///     FOR+MTTR, FOR-only, lambda-only, missing under strict vs default policy).
///   - compute_distribution_indices() now weights DC bus customers so DC load
///     interruptions reach SAIFI/SAIDI/ASAI (code-review Finding 3), while
///     AC-only callers are unaffected (backward compatibility).
///   - summarize_reliability_data_quality() reports case-data coverage.
///   - run_distribution_fmea() populates the data-quality summary and honours
///     the StrictCaseDataOnly policy (templates are opt-in — Finding 2).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/reliability/failure_mode.hpp"
#include "hacdcpf/reliability/protection_frt.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Approx;

namespace {
constexpr double kHoursPerYear = 8760.0;

HybridPowerSystem make_linked_transformer_case() {
  HybridPowerSystem sys;
  sys.ac.base_mva = 100.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.in_service = true;
  load_bus.pd_mw = 10.0;
  sys.ac.buses = {slack, load_bus};

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.in_service = true;
  gen.is_slack = true;
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 50.0;
  sys.ac.generators = {gen};

  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.x_pu = 0.1;
  branch.r_pu = 0.01;
  branch.tap = 1.05;
  branch.rate_a_mva = 100.0;
  branch.in_service = true;
  sys.ac.branches = {branch};

  Transformer2W metadata;
  metadata.index = 1;
  metadata.hv_bus = 1;
  metadata.lv_bus = 2;
  metadata.in_service = true;
  metadata.source_branch_idx = branch.index;
  sys.ac.transformers_2w = {metadata};
  return sys;
}

HybridPowerSystem make_two_island_source_case() {
  HybridPowerSystem sys;
  sys.ac.base_mva = 100.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus local;
  local.index = 2;
  local.bus_type = BusType::PQ;
  local.in_service = true;
  local.pd_mw = 10.0;
  sys.ac.buses = {slack, local};

  Generator grid_source;
  grid_source.index = 1;
  grid_source.bus = 1;
  grid_source.in_service = true;
  grid_source.is_slack = true;
  grid_source.pmax_mw = 50.0;
  Generator island_source;
  island_source.index = 2;
  island_source.bus = 2;
  island_source.in_service = true;
  island_source.pmax_mw = 20.0;
  sys.ac.generators = {grid_source, island_source};
  return sys;
}
}  // namespace

TEST_CASE("linked MATPOWER transformer metadata never masks a failed branch",
          "[reliability][topology][transformer-link]") {
  auto healthy = make_linked_transformer_case();
  const auto healthy_graph = graph::build_power_system_graph(healthy);
  const auto healthy_topology = graph::analyze_topology(healthy_graph);
  REQUIRE(healthy_topology.is_connected);

  auto failed = healthy;
  failed.ac.branches.front().in_service = false;
  // The importer intentionally leaves linked Transformer2W metadata online.
  // It must not become a second electrical edge when the source branch fails.
  const auto failed_graph = graph::build_power_system_graph(failed);
  const auto failed_topology = graph::analyze_topology(failed_graph);
  CHECK_FALSE(failed_topology.is_connected);
  CHECK(failed_topology.n_ac_islands == 2);

  const auto load_island = std::find_if(
      failed_topology.islands.begin(), failed_topology.islands.end(),
      [](const graph::IslandInfo& island) {
        return std::find(island.ac_bus_ids.begin(), island.ac_bus_ids.end(), 2) !=
               island.ac_bus_ids.end();
      });
  REQUIRE(load_island != failed_topology.islands.end());
  CHECK(load_island->status == graph::IslandStatus::NoSlack);

  opf::DCOPFOptions opf_options;
  opf_options.load_shedding = true;
  opf_options.compute_lmp = false;
  const auto opf = opf::solve_dc_opf(failed, opf_options);
  CHECK(opf.converged);
  CHECK(opf.total_load_shedding_mw == Approx(10.0).margin(1e-6));
}

TEST_CASE("hybrid reliability LP uses physical branches and one angle reference per island",
          "[reliability][hybrid][topology][transformer-link]") {
  auto sys = make_linked_transformer_case();
  sys.ac.branches.front().in_service = false;
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.in_service = true;
  sys.dc.buses = {dc_bus};

  FMEAOptions options;
  options.opf_options.compute_lmp = false;
  const auto result = evaluate_failed_network_state(sys, options);
  CHECK(result.model_scope == "hybrid-acdc-network-lp");
  CHECK(result.is_loss);
  CHECK(result.total_shed_mw == Approx(10.0).margin(1e-6));
}

TEST_CASE("hybrid reliability redispatch admits zero converter transfer",
          "[reliability][hybrid][constructive-feasibility]") {
  HybridPowerSystem sys;
  ACBus ac_bus;
  ac_bus.index = 1;
  ac_bus.in_service = true;
  sys.ac.buses = {ac_bus};
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.in_service = true;
  sys.dc.buses = {dc_bus};
  VSCConverter converter;
  converter.index = 1;
  converter.bus_ac = 1;
  converter.bus_dc = 1;
  converter.in_service = true;
  converter.controllable = false;
  converter.p_set_mw = 5.0;
  converter.pmax_mw = 5.0;
  converter.pmin_mw = 5.0;
  sys.vsc_converters = {converter};

  const auto result = evaluate_failed_network_state(sys, FMEAOptions{});
  CHECK(result.model_scope == "hybrid-acdc-network-lp");
  CHECK_FALSE(result.is_loss);
  CHECK(result.total_shed_mw == Approx(0.0).margin(1e-8));
}

TEST_CASE("DC OPF distinguishes angle reference from island power source",
          "[reliability][topology][island-source]") {
  auto sys = make_two_island_source_case();
  const auto topology = graph::analyze_topology(
      graph::build_power_system_graph(sys));
  REQUIRE(topology.n_ac_islands == 2);
  REQUIRE(std::any_of(topology.islands.begin(), topology.islands.end(),
      [](const graph::IslandInfo& island) {
        return island.status == graph::IslandStatus::NoSlack &&
               std::find(island.ac_bus_ids.begin(), island.ac_bus_ids.end(), 2) !=
                   island.ac_bus_ids.end();
      }));

  opf::DCOPFOptions options;
  options.load_shedding = true;
  options.compute_lmp = false;
  const auto supplied = opf::solve_dc_opf(sys, options);
  REQUIRE(supplied.converged);
  CHECK(supplied.total_load_shedding_mw == Approx(0.0).margin(1e-7));

  sys.ac.generators[1].in_service = false;
  const auto unsupplied = opf::solve_dc_opf(sys, options);
  REQUIRE(unsupplied.converged);
  CHECK(unsupplied.total_load_shedding_mw == Approx(10.0).margin(1e-6));
}

TEST_CASE("DC OPF load shedding keeps every generator-less state feasible",
          "[reliability][dc-opf][constructive-feasibility]") {
  HybridPowerSystem sys;
  sys.ac.base_mva = 100.0;
  ACBus bus;
  bus.index = 7;
  bus.bus_type = BusType::PQ;
  bus.in_service = true;
  bus.pd_mw = 10.0;
  sys.ac.buses = {bus};

  for (const auto backend : {opf::DCOPFSolverBackend::Auto,
                             opf::DCOPFSolverBackend::HiGHS,
                             opf::DCOPFSolverBackend::Native}) {
    opf::DCOPFOptions options;
    options.solver = backend;
    options.load_shedding = true;
    options.lexicographic_load_shedding = true;
    options.compute_lmp = false;
    const auto result = opf::solve_dc_opf(sys, options);
    INFO("status=" << result.status);
    INFO("solver=" << result.solver_name);
    REQUIRE(result.converged);
    REQUIRE(result.load_shedding_mw.size() == 1);
    CHECK(result.load_shedding_mw[0] == Approx(10.0).margin(1e-6));
    CHECK(result.total_load_shedding_mw == Approx(10.0).margin(1e-6));
  }
}

TEST_CASE("Reliability DC OPF curtails fixed injection instead of becoming infeasible",
          "[reliability][dc-opf][constructive-feasibility][pgc]") {
  HybridPowerSystem sys;
  sys.ac.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::PQ;
  bus.in_service = true;
  bus.pd_mw = 2.0;
  sys.ac.buses = {bus};

  StaticGenerator static_generator;
  static_generator.index = 1;
  static_generator.bus = 1;
  static_generator.p_mw = 5.0;
  sys.ac.static_generators = {static_generator};

  RenewableGen renewable;
  renewable.index = 1;
  renewable.bus = 1;
  renewable.p_mw = 3.0;
  sys.ac.renewable_gens = {renewable};

  PVSystem pv;
  pv.index = 1;
  pv.bus = 1;
  pv.p_mw = 4.0;
  sys.ac.pv_systems = {pv};

  Storage storage;
  storage.index = 1;
  storage.bus = 1;
  storage.p_mw = 1.0;
  sys.ac.storage = {storage};

  for (const auto backend : {opf::DCOPFSolverBackend::HiGHS,
                             opf::DCOPFSolverBackend::Native}) {
    opf::DCOPFOptions options;
    options.solver = backend;
    options.load_shedding = true;
    options.lexicographic_load_shedding = true;
    options.full_redispatch_from_zero = true;
    options.allow_fixed_generation_curtailment = true;
    options.compute_lmp = false;
    const auto result = opf::solve_dc_opf(sys, options);
    INFO("status=" << result.status);
    INFO("solver=" << result.solver_name);
    REQUIRE(result.converged);
    REQUIRE(result.load_shedding_mw.size() == 1);
    REQUIRE(result.fixed_generation_curtailment_mw.size() == 1);
    CHECK(result.total_load_shedding_mw == Approx(0.0).margin(1e-7));
    CHECK(result.fixed_generation_curtailment_mw[0] ==
          Approx(11.0).margin(1e-6));
    CHECK(result.total_fixed_generation_curtailment_mw ==
          Approx(11.0).margin(1e-6));
    const auto [feasible, violation_mw, reason] =
        opf::check_dc_opf_feasibility(sys, result, options.feasibility_tol);
    INFO("feasibility reason=" << reason);
    CHECK(feasible);
    CHECK(violation_mw <= options.feasibility_tol * sys.ac.base_mva);
  }
}

TEST_CASE("Reliability HL-II uses zero Pmin and exact shed-first dispatch",
          "[reliability][dc-opf][lexicographic][pmin]") {
  HybridPowerSystem sys;
  sys.ac.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::PQ;
  bus.in_service = true;
  bus.pd_mw = 10.0;
  sys.ac.buses = {bus};

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.in_service = true;
  gen.pmin_mw = 15.0;
  gen.pmax_mw = 20.0;
  gen.cost_c1 = 100.0;
  sys.ac.generators = {gen};

  FMEAOptions options;
  options.opf_options.voll = 0.01;
  options.opf_options.compute_lmp = false;
  const auto result = evaluate_failed_network_state(sys, options);
  CHECK(result.model_scope == "ac-only-dcopf");
  CHECK(result.total_shed_mw == Approx(0.0).margin(1e-7));
  CHECK_FALSE(result.is_loss);
}

TEST_CASE("IEEE RTS-24 reliability data is mapped by unit and branch row",
          "[reliability][rts24][data]") {
  auto sys = io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case24_ieee_rts.m");
  REQUIRE(sys.ac.generators.size() == 33);
  REQUIRE(sys.ac.branches.size() == 38);
  REQUIRE_NOTHROW(apply_ieee24_reliability_data(sys));

  const auto check_generator = [&](size_t position, double expected_for,
                                   double expected_mttr) {
    INFO("generator row=" << position + 1);
    CHECK(sys.ac.generators[position].forced_outage_rate ==
          Approx(expected_for).margin(1e-12));
    CHECK(sys.ac.generators[position].mttr_hr ==
          Approx(expected_mttr).margin(1e-12));
  };
  check_generator(0, 0.10, 50.0);       // U20
  check_generator(2, 0.02, 40.0);       // U76
  check_generator(8, 0.04, 50.0);       // U100
  check_generator(11, 0.05, 50.0);      // U197
  check_generator(14, 0.1 / 10000.1, 0.1);  // synchronous condenser
  check_generator(15, 0.02, 60.0);      // U12
  check_generator(20, 0.04, 40.0);      // U155
  check_generator(22, 0.12, 150.0);     // U400
  check_generator(24, 0.01, 20.0);      // U50
  check_generator(32, 0.08, 100.0);     // U350

  CHECK(sys.ac.branches[5].failure_rate == Approx(0.38));
  CHECK(sys.ac.branches[5].mttr_hr == Approx(10.0));
  CHECK(sys.ac.branches[6].failure_rate == Approx(0.02));
  CHECK(sys.ac.branches[6].mttr_hr == Approx(768.0));
  CHECK(sys.ac.branches[11].failure_rate == Approx(0.44));
  CHECK(sys.ac.branches[33].failure_rate == Approx(0.38));
  CHECK(sys.ac.branches[35].failure_rate == Approx(0.34));

  auto malformed = sys;
  malformed.ac.generators[3].pmax_mw += 1.0;
  CHECK_THROWS_AS(apply_ieee24_reliability_data(malformed),
                  std::invalid_argument);
}

TEST_CASE("IEEE RTS-24 every N-0 N-1 and N-2 HL-II state is feasible",
          "[reliability][rts24][state-scan]") {
  auto sys = io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case24_ieee_rts.m");
  apply_ieee24_reliability_data(sys);

  opf::DCOPFOptions options;
  options.solver = opf::DCOPFSolverBackend::HiGHS;
  options.feasibility_tol = 1e-7;
  const auto scan = scan_ac_hlii_n2_states(sys, options);
  INFO("first failure state=" << scan.first_failure_state);
  INFO("first failure reason=" << scan.first_failure_reason);
  CHECK(scan.component_count == 71);
  CHECK(scan.states_evaluated == 2557);
  CHECK(scan.failed_states == 0);
  CHECK(scan.nonfinite_states == 0);
  CHECK(scan.infeasible_states == 0);
  CHECK(scan.negative_shed_states == 0);
  CHECK(scan.shed_above_load_states == 0);
  CHECK(scan.total_load_mw == Approx(2850.0).margin(1e-9));
  CHECK(scan.maximum_shed_mw <= scan.total_load_mw + 1e-5);
  CHECK(scan.passed());
}

// ─────────────────────────────────────────────────────────────────────────
// Resolver conversion rules
// ─────────────────────────────────────────────────────────────────────────

TEST_CASE("resolver: lambda + MTTR (AC-branch style)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = 2.0;
  raw.mttr_hr = 10.0;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK_FALSE(p.used_default);
  CHECK(p.data_source == "case");
  CHECK(p.lambda_per_year == Approx(2.0));
  CHECK(p.calendar_frequency_per_year ==
        Approx((1.0 - p.unavailability) * p.lambda_per_year));
  CHECK(p.repair_hr == Approx(10.0));
  // U = lambda / (lambda + 8760/repair)
  CHECK(p.unavailability == Approx(2.0 / (2.0 + kHoursPerYear / 10.0)));
  CHECK(p.mttf_hr == Approx(kHoursPerYear / 2.0));
}

TEST_CASE("resolver: calendar failure frequency is inverted without rare-event approximation",
          "[reliability][resolver][calendar]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = 2.0;
  raw.mttr_hr = 438.0;
  ReliabilityDataPolicy policy;
  policy.failure_rate_basis = FailureRateBasis::CalendarTime;

  const auto resolved = resolve_reliability_params(raw, policy);
  const double expected_u = 2.0 * 438.0 / 8760.0;
  REQUIRE(resolved.has_data);
  CHECK(resolved.calendar_frequency_per_year == Approx(2.0));
  CHECK(resolved.unavailability == Approx(expected_u));
  CHECK(resolved.lambda_per_year == Approx(2.0 / (1.0 - expected_u)));
  CHECK((1.0 - resolved.unavailability) * resolved.lambda_per_year ==
        Approx(resolved.calendar_frequency_per_year));

  policy.hours_per_year = 8736.0;
  const auto non_leap_reporting_year = resolve_reliability_params(raw, policy);
  const double expected_8736_u = 2.0 * 438.0 / 8736.0;
  CHECK(non_leap_reporting_year.unavailability == Approx(expected_8736_u));
  CHECK(non_leap_reporting_year.lambda_per_year ==
        Approx(2.0 / (1.0 - expected_8736_u)));

  raw.failure_rate_per_year = 20.0;
  CHECK_THROWS_AS(resolve_reliability_params(raw, policy),
                  std::invalid_argument);
}

TEST_CASE("resolver: MTBF + MTTR (transformer style)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.mtbf_hours = 8760.0;
  raw.mttr_hours = 24.0;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.data_source == "case");
  CHECK(p.lambda_per_year == Approx(1.0));            // 8760 / 8760
  CHECK(p.repair_hr == Approx(24.0));
  CHECK(p.unavailability == Approx(24.0 / (8760.0 + 24.0)));  // steady-state form
}

TEST_CASE("resolver: FOR + MTTR (generator style)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.forced_outage_rate = 0.02;
  raw.mttr_hr = 50.0;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.unavailability == Approx(0.02));
  CHECK(p.repair_hr == Approx(50.0));
  // lambda = FOR / ((1-FOR) * MTTR) * 8760
  CHECK(p.lambda_per_year ==
        Approx(0.02 / ((1.0 - 0.02) * 50.0) * kHoursPerYear));
  // mttf = MTTR * (1-FOR) / FOR
  CHECK(p.mttf_hr == Approx(50.0 * (1.0 - 0.02) / 0.02));
}

TEST_CASE("resolver: FOR only is partial (no frequency)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.forced_outage_rate = 0.05;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.unavailability == Approx(0.05));
  CHECK(p.lambda_per_year == Approx(0.0));   // undetermined without MTTR
  CHECK_FALSE(p.warnings.empty());
}

TEST_CASE("resolver: lambda only is partial (no repair)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = 1.5;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.lambda_per_year == Approx(1.5));
  CHECK(p.mttf_hr == Approx(kHoursPerYear / 1.5));
  CHECK(p.repair_hr == Approx(0.0));
  CHECK_FALSE(p.warnings.empty());
}

TEST_CASE("resolver: MTBF alone yields frequency (switch style)", "[reliability][resolver]") {
  ReliabilityRawFields raw;
  raw.mtbf_hours = 17520.0;  // 2 years

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.lambda_per_year == Approx(0.5));   // 8760 / 17520
}

TEST_CASE("resolver: missing data under StrictCaseDataOnly invents nothing",
          "[reliability][resolver]") {
  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;

  ReliabilityParams p = resolve_reliability_params(
      ReliabilityRawFields{}, strict, /*def_lambda=*/0.35, /*def_repair=*/10.0);

  CHECK_FALSE(p.has_data);
  CHECK(p.data_source == "missing");
  CHECK(p.lambda_per_year == Approx(0.0));
  CHECK(p.repair_hr == Approx(0.0));
  CHECK_FALSE(p.warnings.empty());
}

TEST_CASE("resolver: missing data fills per-kind defaults under default policy",
          "[reliability][resolver]") {
  ReliabilityDataPolicy pol;  // default = UseNamedTemplateForMissingOnly
  ReliabilityParams p = resolve_reliability_params(
      ReliabilityRawFields{}, pol, /*def_lambda=*/0.35, /*def_repair=*/10.0);

  CHECK(p.used_default);
  CHECK(p.data_source == "default");
  CHECK(p.lambda_per_year == Approx(0.35));
  CHECK(p.repair_hr == Approx(10.0));
  CHECK(p.unavailability == Approx(0.35 / (0.35 + kHoursPerYear / 10.0)));
}

TEST_CASE("resolver: lambda+MTTR takes priority over MTBF when both present",
          "[reliability][resolver][edge]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = 3.0;
  raw.mttr_hr = 8.0;
  raw.mtbf_hours = 8760.0;   // would imply lambda=1.0 if used — must NOT be used

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  CHECK(p.lambda_per_year == Approx(3.0));   // from failure_rate, not MTBF
  CHECK(p.repair_hr == Approx(8.0));
}

TEST_CASE("resolver: FOR >= 1 is rejected (not a valid steady-state U)",
          "[reliability][resolver][edge]") {
  ReliabilityRawFields raw;
  raw.forced_outage_rate = 1.0;   // out of (0,1)

  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  ReliabilityParams p = resolve_reliability_params(raw, strict);

  CHECK_FALSE(p.has_data);
  CHECK(p.data_source == "missing");
}

TEST_CASE("resolver: negative fields are treated as not provided",
          "[reliability][resolver][edge]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = -2.0;
  raw.mttr_hr = -5.0;

  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  ReliabilityParams p = resolve_reliability_params(raw, strict);

  CHECK_FALSE(p.has_data);
  CHECK(p.lambda_per_year == Approx(0.0));
}

TEST_CASE("resolver: overwrite-template policy also fills defaults when missing",
          "[reliability][resolver][edge]") {
  ReliabilityDataPolicy pol;
  pol.default_policy = ReliabilityDefaultPolicy::OverwriteWithNamedTemplate;
  ReliabilityParams p = resolve_reliability_params(
      ReliabilityRawFields{}, pol, /*def_lambda=*/0.5, /*def_repair=*/20.0);

  CHECK(p.used_default);
  CHECK(p.lambda_per_year == Approx(0.5));
  CHECK(p.repair_hr == Approx(20.0));
}

TEST_CASE("resolver: MTBF + MTTR equals lambda+repair round trip",
          "[reliability][resolver][edge]") {
  // A component specified as MTBF/MTTR and the same component specified as
  // lambda/repair must resolve to identical frequency and repair time.
  ReliabilityRawFields a;            // MTBF/MTTR form
  a.mtbf_hours = 4380.0;             // lambda = 2.0/yr
  a.mttr_hours = 12.0;

  ReliabilityRawFields b;            // lambda/repair form
  b.failure_rate_per_year = 2.0;
  b.mttr_hr = 12.0;

  ReliabilityParams pa = resolve_reliability_params(a, ReliabilityDataPolicy{});
  ReliabilityParams pb = resolve_reliability_params(b, ReliabilityDataPolicy{});

  CHECK(pa.lambda_per_year == Approx(pb.lambda_per_year));
  CHECK(pa.repair_hr == Approx(pb.repair_hr));
}

// ─────────────────────────────────────────────────────────────────────────
// Active-on-demand, explicit MTTF, MTBF convention, configurable H
// ─────────────────────────────────────────────────────────────────────────

TEST_CASE("resolver: active-on-demand frequency = demand_freq * p_demand",
          "[reliability][resolver][active]") {
  ReliabilityRawFields raw;
  raw.is_active = true;
  raw.probability_per_demand = 0.01;
  raw.demand_frequency_per_year = 2.0;
  raw.cyber_recovery_hr = 1.0;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.is_active);
  CHECK(p.lambda_active_per_year == Approx(0.02));   // 2.0 * 0.01
  CHECK(p.lambda_per_year == Approx(0.02));
  CHECK(p.repair_hr == Approx(1.0));                  // cyber recovery time
  CHECK(p.unavailability == Approx(0.02 * 1.0 / (8760.0 + 0.02 * 1.0)));
}

TEST_CASE("resolver: active mode without repair is frequency-only",
          "[reliability][resolver][active]") {
  ReliabilityRawFields raw;
  raw.is_active = true;
  raw.probability_per_demand = 0.005;
  raw.demand_frequency_per_year = 1.0;
  // no repair / recovery time

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.lambda_active_per_year == Approx(0.005));
  CHECK(p.repair_hr == Approx(0.0));
  CHECK_FALSE(p.warnings.empty());
}

TEST_CASE("resolver: explicit MTTF + MTTR", "[reliability][resolver][mttf]") {
  ReliabilityRawFields raw;
  raw.mttf_hours = 4380.0;   // lambda = 2.0/yr
  raw.mttr_hours = 12.0;

  ReliabilityParams p = resolve_reliability_params(raw, ReliabilityDataPolicy{});

  REQUIRE(p.has_data);
  CHECK(p.lambda_per_year == Approx(2.0));
  CHECK(p.repair_hr == Approx(12.0));
  CHECK(p.unavailability == Approx(12.0 / (4380.0 + 12.0)));
}

TEST_CASE("resolver: MTBF cycle-time convention subtracts MTTR",
          "[reliability][resolver][mtbf]") {
  ReliabilityRawFields raw;
  raw.mtbf_hours = 4392.0;   // cycle time
  raw.mttr_hours = 12.0;     // MTTF = 4392 - 12 = 4380

  ReliabilityDataPolicy pol;
  pol.mtbf_convention = MtbfConvention::MtbfAsCycleTime;

  ReliabilityParams p = resolve_reliability_params(raw, pol);

  CHECK(p.lambda_per_year == Approx(8760.0 / 4380.0));
  CHECK(p.mtbf_convention_applied == MtbfConvention::MtbfAsCycleTime);
}

TEST_CASE("resolver: configurable hours-per-year (8736)",
          "[reliability][resolver][hours]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = 2.0;
  raw.mttr_hr = 10.0;

  ReliabilityDataPolicy pol;
  pol.hours_per_year = 8736.0;

  ReliabilityParams p = resolve_reliability_params(raw, pol);

  CHECK(p.unavailability == Approx(2.0 / (2.0 + 8736.0 / 10.0)));
  CHECK(p.mttf_hr == Approx(8736.0 / 2.0));
}

TEST_CASE("resolver: non-finite inputs are rejected",
          "[reliability][resolver][edge]") {
  ReliabilityRawFields raw;
  raw.failure_rate_per_year = std::numeric_limits<double>::infinity();
  raw.mttr_hr = std::nan("");

  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  ReliabilityParams p = resolve_reliability_params(raw, strict);

  CHECK_FALSE(p.has_data);
  CHECK(p.data_source == "missing");
}

// -------------------------------------------------------------------------
// Analytical reliability theory kernels
// -------------------------------------------------------------------------

TEST_CASE("analytical F&D: exact series and parallel CTMC reductions",
          "[reliability][analytical][frequency-duration]") {
  const double lambda = 8760.0 / 1900.0;
  const TwoStateReliabilityComponent unit{lambda, 100.0, 0.05};

  const auto series = reduce_series_frequency_duration({unit, unit});
  CHECK(series.availability == Approx(0.95 * 0.95).margin(1e-14));
  CHECK(series.unavailability == Approx(1.0 - 0.95 * 0.95).margin(1e-14));
  CHECK(series.failure_frequency_per_year ==
        Approx(0.95 * 0.95 * 2.0 * lambda).margin(1e-12));
  CHECK(series.mean_failure_duration_hr ==
        Approx(series.unavailability * 8760.0 /
               series.failure_frequency_per_year).margin(1e-12));

  const auto parallel = reduce_parallel_frequency_duration({unit, unit});
  CHECK(parallel.unavailability == Approx(0.05 * 0.05).margin(1e-14));
  CHECK(parallel.failure_frequency_per_year ==
        Approx(2.0 * lambda * 0.95 * 0.05).margin(1e-12));
  CHECK(parallel.mean_failure_duration_hr == Approx(50.0).margin(1e-10));
}

TEST_CASE("analytical COPT: two 60 MW units match the closed-form table",
          "[reliability][analytical][copt]") {
  HybridPowerSystem sys;
  Generator g1;
  g1.index = 1;
  g1.in_service = true;
  g1.pmax_mw = 60.0;
  g1.forced_outage_rate = 0.05;
  g1.mttr_hr = 100.0;
  Generator g2 = g1;
  g2.index = 2;
  sys.ac.generators = {g1, g2};

  const auto result = run_frequency_duration_analysis(sys, 50.0);
  REQUIRE(result.capacity_outage_levels.size() == 3);
  REQUIRE(result.state_probability.size() == 3);
  CHECK(result.exact_capacity_states);
  CHECK(result.probability_valid);
  CHECK(result.frequency_valid);
  CHECK(result.capacity_outage_levels[0] == Approx(0.0));
  CHECK(result.capacity_outage_levels[1] == Approx(60.0));
  CHECK(result.capacity_outage_levels[2] == Approx(120.0));
  CHECK(result.state_probability[0] == Approx(0.9025).margin(1e-14));
  CHECK(result.state_probability[1] == Approx(0.095).margin(1e-14));
  CHECK(result.state_probability[2] == Approx(0.0025).margin(1e-14));
  CHECK(std::accumulate(result.state_probability.begin(),
                        result.state_probability.end(), 0.0) ==
        Approx(1.0).margin(1e-14));
  CHECK(result.lolp == Approx(0.0025).margin(1e-14));
  CHECK(result.lole_fd == Approx(21.9).margin(1e-12));
  CHECK(result.lolf_fd == Approx(0.438).margin(1e-12));
  CHECK(result.lold == Approx(50.0).margin(1e-10));

  sys.ac.generators.front().mttr_hr = 0.0;
  const auto probability_only = run_frequency_duration_analysis(sys, 50.0);
  CHECK(probability_only.probability_valid);
  CHECK_FALSE(probability_only.frequency_valid);
  CHECK(probability_only.lolp == Approx(0.0025).margin(1e-14));
  CHECK(probability_only.lolf_fd == Approx(0.0));
  CHECK(probability_only.lold == Approx(0.0));
  REQUIRE_FALSE(probability_only.warnings.empty());
}

TEST_CASE("analytical PFD: exact proof-test average and low-rate limit",
          "[reliability][analytical][pfd]") {
  const auto result = compute_low_demand_pfd(1e-6, 1000.0);
  const double expected = 1.0 - (1.0 - std::exp(-0.001)) / 0.001;
  CHECK(result.pfd_average == Approx(expected).margin(1e-13));
  CHECK(result.first_order_pfd_average == Approx(0.0005));
  CHECK(result.approximation_relative_error < 0.001);

  const auto tiny = compute_low_demand_pfd(1e-12, 1.0);
  CHECK(tiny.pfd_average == Approx(5e-13).margin(1e-24));
  CHECK_THROWS_AS(compute_low_demand_pfd(-1.0, 1.0), std::invalid_argument);
}

TEST_CASE("tail risk: empirical expected shortfall fractionally weights VaR atom",
          "[reliability][analytical][tail-risk]") {
  const std::vector<double> losses{0.0, 0.0, 10.0, 20.0};
  const auto risk = compute_tail_risk(losses, {}, 0.60);
  CHECK(risk.eens_var == Approx(10.0));
  // Integral of empirical quantiles on [0.6,1]: 0.15*10 + 0.25*20,
  // divided by the 0.4 tail probability.
  CHECK(risk.eens_cvar == Approx(16.25).margin(1e-12));
  CHECK_THROWS_AS(compute_tail_risk(losses, {}, 1.0), std::invalid_argument);
}

TEST_CASE("information reliability: shared infrastructure is counted once",
          "[reliability][analytical][cyber][cut-set]") {
  const std::vector<InformationServiceComponent> components{
      {"control-center", 0.9, 1.0, 1.0, 0.1},
      {"radio-a", 0.8, 1.0, 1.0, 0.1},
      {"radio-b", 0.7, 1.0, 1.0, 0.1}};
  InformationFunctionDefinition function;
  function.name = "FLISR";
  function.alternative_paths = {{{0, 1}}, {{0, 2}}};

  const auto result =
      evaluate_information_function_reliability(components, function);
  CHECK(result.valid_path_count == 2);
  CHECK(result.availability == Approx(0.9 * (1.0 - 0.2 * 0.3)).margin(1e-12));
  CHECK(result.availability != Approx(1.0 - (1.0 - 0.72) * (1.0 - 0.63)));
  REQUIRE(result.minimal_cut_sets.size() == 2);
  CHECK(std::find(result.minimal_cut_sets.begin(), result.minimal_cut_sets.end(),
                  std::vector<size_t>{0}) != result.minimal_cut_sets.end());
  CHECK(std::find(result.minimal_cut_sets.begin(), result.minimal_cut_sets.end(),
                  (std::vector<size_t>{1, 2})) != result.minimal_cut_sets.end());

  function.max_latency_ms = 1.5;
  const auto no_paths =
      evaluate_information_function_reliability(components, function);
  CHECK(no_paths.valid_path_count == 0);
  CHECK(no_paths.availability == Approx(0.0));
}

TEST_CASE("physical network reliability: meshed success paths yield exact minimal cuts",
          "[reliability][analytical][physical][cut-set]") {
  const std::vector<PhysicalReliabilityComponent> components{
      {"ac_branch:10", 0.9},
      {"ac_branch:20", 0.8},
      {"ac_branch:30", 0.7}};
  const std::vector<PhysicalSuccessPath> paths{{{0, 1}}, {{0, 2}}};

  const auto result = evaluate_physical_network_reliability(components, paths);
  CHECK(result.exact_independent_path_model);
  CHECK(result.reduced_success_path_count == 2);
  CHECK(result.availability == Approx(0.9 * (1.0 - 0.2 * 0.3)).margin(1e-12));
  CHECK(result.loss_probability == Approx(1.0 - result.availability).margin(1e-12));
  REQUIRE(result.minimal_cut_set_stable_ids.size() == 2);
  CHECK(std::find(result.minimal_cut_set_stable_ids.begin(),
                  result.minimal_cut_set_stable_ids.end(),
                  std::vector<std::string>{"ac_branch:10"}) !=
        result.minimal_cut_set_stable_ids.end());
  CHECK(std::find(result.minimal_cut_set_stable_ids.begin(),
                  result.minimal_cut_set_stable_ids.end(),
                  (std::vector<std::string>{"ac_branch:20", "ac_branch:30"})) !=
        result.minimal_cut_set_stable_ids.end());

  CHECK_THROWS_AS(evaluate_physical_network_reliability(
                      {{"duplicate", 0.9}, {"duplicate", 0.8}}, {{{0}}}),
                  std::invalid_argument);
  CHECK_THROWS_AS(evaluate_physical_network_reliability(components, {{{3}}}),
                  std::out_of_range);
}

TEST_CASE("information reliability: joint functions retain shared dependencies",
          "[reliability][analytical][cyber][joint]") {
  const std::vector<InformationServiceComponent> components{
      {"shared-center", 0.9}, {"detector", 0.8}, {"restorer", 0.7}};
  InformationFunctionDefinition detection;
  detection.name = "detection";
  detection.alternative_paths = {{{0, 1}}};
  InformationFunctionDefinition restoration;
  restoration.name = "restoration";
  restoration.alternative_paths = {{{0, 2}}};

  const auto joint = evaluate_joint_information_reliability(
      components, {detection, restoration});
  CHECK(joint.availability == Approx(0.9 * 0.8 * 0.7).margin(1e-12));
  REQUIRE(joint.minimal_cut_sets.size() == 3);
  for (size_t index = 0; index < 3; ++index) {
    CHECK(std::find(joint.minimal_cut_sets.begin(), joint.minimal_cut_sets.end(),
                    std::vector<size_t>{index}) !=
          joint.minimal_cut_sets.end());
  }
}

TEST_CASE("protection FRT: inverse-time integration and event classes are exact",
          "[reliability][protection-frt][event-tree]") {
  InverseTimeRelaySettings primary_curve;
  primary_curve.pickup_current = 1.0;
  primary_curve.time_multiplier = 1.0;
  primary_curve.curve_a = 1.0;
  primary_curve.curve_p = 1.0;
  primary_curve.reset_time_s = 1.0;
  const std::vector<RelayCurrentPoint> current{{0.0, 2.0}, {1.1, 2.0}};
  const auto relay = evaluate_inverse_time_relay(current, primary_curve);
  REQUIRE(relay.operated);
  CHECK(relay.command_time_s == Approx(1.0).margin(1e-12));
  CHECK(relay.terminal_action_integral == Approx(1.0));

  const auto reset_relay = evaluate_inverse_time_relay(
      {{0.0, 2.0}, {0.5, 0.0}, {1.5, 2.0}, {2.5, 2.0}},
      primary_curve);
  REQUIRE(reset_relay.operated);
  CHECK(reset_relay.command_time_s ==
        Approx(1.5 + 1.0 - 0.5 * std::exp(-1.0)).margin(1e-12));

  ProtectionFRTEventInput input;
  input.primary.current_trajectory = current;
  input.primary.relay = primary_curve;
  input.primary.relay_success_probability = 0.9;
  input.primary.breaker_success_probability = 0.95;
  input.backup.current_trajectory = {{0.0, 2.0}, {2.1, 2.0}};
  input.backup.relay = primary_curve;
  input.backup.relay.time_multiplier = 2.0;
  input.backup.relay_success_probability = 0.8;
  input.backup.breaker_success_probability = 1.0;
  input.coordination_margin_s = 0.5;
  input.uncleared_terminal_time_s = 3.0;

  ProtectionFRTDERInput der;
  der.stable_id = "ac_pv_system:7";
  der.settings.enabled = true;
  der.settings.v_filter_t_s = 1e-9;
  der.settings.f_filter_t_s = 1e-9;
  der.settings.allow_reconnect = false;
  der.settings.undervoltage_trip = {{0.9, 0.2}};
  der.trajectory = {{0.0, 0.4, 0.0}, {0.1, 0.4, 0.0},
                    {0.2, 0.4, 0.0}, {3.0, 0.4, 0.0}};
  input.ders = {der};

  const auto generated = generate_protection_frt_classes(input);
  REQUIRE(generated.class_probabilities_normalized);
  CHECK(generated.primary_clear_time_s == Approx(1.0).margin(1e-12));
  CHECK(generated.backup_clear_time_s == Approx(2.0).margin(1e-12));
  double primary_probability = 0.0;
  double backup_probability = 0.0;
  double uncleared_probability = 0.0;
  bool saw_breaker_failure = false;
  for (const auto& event_class : generated.classes) {
    REQUIRE(event_class.der_results.size() == 1);
    CHECK(event_class.der_results.front().terminal_class ==
          DERFRTTerminalClass::Tripped);
    switch (event_class.protection_outcome) {
      case ProtectionClearingOutcome::PrimaryCleared:
        primary_probability += event_class.conditional_probability;
        break;
      case ProtectionClearingOutcome::BackupCleared:
        backup_probability += event_class.conditional_probability;
        saw_breaker_failure |= event_class.failure_cause ==
            ProtectionFailureCause::PrimaryBreakerFailed;
        break;
      case ProtectionClearingOutcome::Uncleared:
        uncleared_probability += event_class.conditional_probability;
        break;
    }
  }
  CHECK(primary_probability == Approx(0.855).margin(1e-12));
  CHECK(backup_probability == Approx(0.116).margin(1e-12));
  CHECK(uncleared_probability == Approx(0.029).margin(1e-12));
  CHECK(saw_breaker_failure);

  ProtectionFRTReliabilityScenario scenario;
  scenario.scenario_id = "fault-1";
  scenario.initiating_frequency_per_year = 2.0;
  scenario.trace_classes_validated = true;
  for (const auto& event_class : generated.classes) {
    double shed = 0.0;
    switch (event_class.protection_outcome) {
      case ProtectionClearingOutcome::PrimaryCleared: shed = 1.0; break;
      case ProtectionClearingOutcome::BackupCleared: shed = 2.0; break;
      case ProtectionClearingOutcome::Uncleared: shed = 3.0; break;
    }
    scenario.classes.push_back({event_class, {{1.0, shed}}});
  }
  const auto reliability = aggregate_protection_frt_reliability({scenario});
  CHECK(reliability.eens_mwh_yr == Approx(2.348).margin(1e-12));
  CHECK(reliability.lole_hr_yr == Approx(2.0).margin(1e-12));
  CHECK(reliability.lolf_occ_yr == Approx(2.0).margin(1e-12));
  CHECK(reliability.validity.protection_frt_reliability_coupled);
  CHECK(reliability.validity.der_ride_through_modelled);
  CHECK(reliability.validity.protection_coordination_modelled);
  CHECK(reliability.validity.breaker_failure_modelled);
  CHECK_FALSE(reliability.validity.online_dae_coupled);
}

TEST_CASE("protection coordination: relay criteria and clearing margins are explicit",
          "[reliability][protection-frt][coordination]") {
  ProtectionRelayModel definite;
  definite.relay_id = "50/51-primary";
  definite.characteristic =
      ProtectionRelayCharacteristic::DefiniteTimeOvercurrent;
  definite.pickup_current = 1.0;
  definite.definite_time_delay_s = 1.0;
  definite.reset_time_s = 1.0;
  const auto reset = evaluate_protection_relay(
      {{0.0, 2.0}, {0.4, 0.0}, {1.4, 2.0}, {2.4, 2.0}}, definite);
  REQUIRE(reset.operated);
  CHECK(reset.command_time_s ==
        Approx(1.4 + 1.0 - 0.4 * std::exp(-1.0)).margin(1e-12));

  ProtectionRelayModel distance;
  distance.relay_id = "21-line";
  distance.characteristic = ProtectionRelayCharacteristic::Distance;
  distance.distance_zones = {{"Z1", 6.0, 0.1}, {"Z2", 10.0, 0.5}};
  auto zone = evaluate_protection_relay(
      {{0.0, 1.0, 1.0, 5.0}, {0.2, 1.0, 1.0, 5.0}}, distance);
  REQUIRE(zone.operated);
  CHECK(zone.command_time_s == Approx(0.1).margin(1e-12));
  CHECK(zone.operated_zone == 0);
  CHECK(zone.operated_zone_name == "Z1");
  zone = evaluate_protection_relay(
      {{0.0, 1.0, 1.0, 8.0}, {0.6, 1.0, 1.0, 8.0}}, distance);
  REQUIRE(zone.operated);
  CHECK(zone.command_time_s == Approx(0.5).margin(1e-12));
  CHECK(zone.operated_zone == 1);

  ProtectionRelayModel differential;
  differential.relay_id = "87-transformer";
  differential.characteristic = ProtectionRelayCharacteristic::Differential;
  differential.differential_pickup = 0.2;
  differential.differential_slope = 0.5;
  differential.differential_high_set = 4.0;
  differential.definite_time_delay_s = 0.0;
  const auto restrained = evaluate_protection_relay(
      {{0.0, 0.0, 1.0, 0.0, 1.0, 2.0},
       {0.1, 0.0, 1.0, 0.0, 1.0, 2.0}}, differential);
  CHECK_FALSE(restrained.operated);
  const auto high_set = evaluate_protection_relay(
      {{0.0, 0.0, 1.0, 0.0, 5.0, 20.0},
       {0.1, 0.0, 1.0, 0.0, 5.0, 20.0}}, differential);
  CHECK(high_set.operated);
  CHECK(high_set.command_time_s == Approx(0.0));

  ProtectionRelayModel primary = definite;
  primary.definite_time_delay_s = 0.1;
  ProtectionRelayModel backup = definite;
  backup.relay_id = "50/51-backup";
  backup.definite_time_delay_s = 0.5;
  BreakerClearingChain breaker;
  breaker.mechanical_delay_s = 0.05;
  const std::vector<ProtectionMeasurementPoint> fault{
      {0.0, 2.0}, {1.0, 2.0}};
  auto report = evaluate_protection_coordination(
      {primary, backup}, {fault, fault}, {breaker, breaker}, {{0, 1, 0.3}});
  REQUIRE(report.checks.size() == 1);
  CHECK(report.checks[0].primary_clear_time_s == Approx(0.15).margin(1e-12));
  CHECK(report.checks[0].backup_clear_time_s == Approx(0.55).margin(1e-12));
  CHECK(report.checks[0].actual_margin_s == Approx(0.4).margin(1e-12));
  CHECK(report.all_selective);
  report = evaluate_protection_coordination(
      {primary, backup}, {fault, fault}, {breaker, breaker}, {{0, 1, 0.5}});
  CHECK_FALSE(report.all_selective);
}

TEST_CASE("protection measurement: CT PT dynamics and saturation match closed form",
          "[reliability][protection-frt][measurement]") {
  InstrumentTransformerSettings settings;
  settings.ct_ratio = 10.0;
  settings.pt_ratio = 100.0;
  settings.ct_time_constant_s = 1.0;
  settings.pt_time_constant_s = 1.0;
  settings.ct_saturation_secondary_a = 5.0;
  settings.pt_saturation_secondary_v = 2.0;
  const auto measured = simulate_instrument_transformers(
      {{0.0, {100.0, 0.0}, {100.0, 0.0}},
       {1.0, {100.0, 0.0}, {100.0, 0.0}},
       {2.0, {100.0, 0.0}, {100.0, 0.0}}},
      settings);
  REQUIRE(measured.size() == 3);
  CHECK(std::abs(measured[1].voltage_phasor_v) ==
        Approx(1.0 - std::exp(-1.0)).margin(1e-12));
  CHECK(std::abs(measured[1].current_phasor_a) == Approx(5.0));
  CHECK(std::abs(measured[2].current_phasor_a) == Approx(5.0));
}

TEST_CASE("complex distance protection evaluates mho and quadrilateral geometry",
          "[reliability][protection-frt][distance]") {
  const auto point = [](double impedance) {
    ProtectionMeasurementPoint sample;
    sample.time_s = 0.0;
    sample.directional_current = 1.0;
    sample.phasor_measurement_valid = true;
    sample.current_phasor_a = {1.0, 0.0};
    sample.voltage_phasor_v = {impedance, 0.0};
    return sample;
  };
  auto end = point(0.0);
  end.time_s = 1.0;

  ProtectionRelayModel mho;
  mho.relay_id = "mho";
  mho.characteristic = ProtectionRelayCharacteristic::Distance;
  DistanceProtectionZone mho_zone;
  mho_zone.name = "Z1";
  mho_zone.reach_ohm = 10.0;
  mho_zone.shape = DistanceZoneShape::Mho;
  mho.distance_zones = {mho_zone};
  CHECK(evaluate_protection_relay({point(5.0), end}, mho).operated);
  CHECK_FALSE(evaluate_protection_relay({point(-1.0), end}, mho).operated);

  ProtectionRelayModel quad = mho;
  quad.relay_id = "quad";
  quad.distance_zones[0].shape = DistanceZoneShape::Quadrilateral;
  quad.distance_zones[0].forward_resistance_ohm = 6.0;
  quad.distance_zones[0].reverse_resistance_ohm = 1.0;
  quad.distance_zones[0].forward_reactance_ohm = 8.0;
  quad.distance_zones[0].reverse_reactance_ohm = 2.0;
  CHECK(evaluate_protection_relay({point(5.5), end}, quad).operated);
  CHECK_FALSE(evaluate_protection_relay({point(7.0), end}, quad).operated);
}

TEST_CASE("adaptive protection changes settings only with communication",
          "[reliability][protection-frt][adaptive]") {
  ProtectionRelayModel base;
  base.pickup_current = 2.0;
  base.definite_time_delay_s = 1.0;
  base.distance_zones = {{"Z1", 10.0, 0.5}};
  AdaptiveProtectionContext context;
  context.pickup_scale = 0.5;
  context.distance_reach_scale = 1.2;
  context.time_delay_scale = 0.8;
  const auto adapted = adapt_protection_settings(base, context);
  CHECK(adapted.pickup_current == Approx(1.0));
  CHECK(adapted.distance_zones[0].reach_ohm == Approx(12.0));
  CHECK(adapted.distance_zones[0].delay_s == Approx(0.4));
  context.communication_available = false;
  const auto retained = adapt_protection_settings(base, context);
  CHECK(retained.pickup_current == Approx(base.pickup_current));
  CHECK(retained.distance_zones[0].reach_ohm ==
        Approx(base.distance_zones[0].reach_ohm));
}

TEST_CASE("recloser fuse sectionalizer automatic sequence reaches physical terminal states",
          "[reliability][protection-frt][sequence]") {
  RecloserFuseSectionalizerInput temporary;
  temporary.shot_trip_times_s = {0.1, 0.2, 0.3};
  temporary.reclose_intervals_s = {0.5, 1.0};
  temporary.maximum_shots = 3;
  temporary.fault_clears_after_shot = 1;
  auto result = simulate_recloser_fuse_sectionalizer(temporary);
  CHECK(result.fault_cleared);
  CHECK(result.recloser_closed);
  CHECK_FALSE(result.recloser_locked_out);
  REQUIRE(result.events.size() == 2);
  CHECK(result.events[0].action == ProtectionSequenceAction::TripOpen);
  CHECK(result.events[1].action == ProtectionSequenceAction::Reclose);

  RecloserFuseSectionalizerInput sectionalizer = temporary;
  sectionalizer.fault_clears_after_shot = -1;
  sectionalizer.sectionalizer_count_to_open = 2;
  result = simulate_recloser_fuse_sectionalizer(sectionalizer);
  CHECK(result.sectionalizer_open);
  CHECK(result.fault_cleared);
  CHECK(result.recloser_closed);

  RecloserFuseSectionalizerInput fuse = temporary;
  fuse.fault_clears_after_shot = -1;
  fuse.fuse_total_clearing_time_s = 0.25;
  result = simulate_recloser_fuse_sectionalizer(fuse);
  CHECK(result.fuse_open);
  CHECK(result.fault_cleared);
  CHECK(result.fuse_melting_fraction == Approx(1.0));

  RecloserFuseSectionalizerInput lockout = temporary;
  lockout.fault_clears_after_shot = -1;
  lockout.instantaneous_lockout = true;
  result = simulate_recloser_fuse_sectionalizer(lockout);
  CHECK(result.recloser_locked_out);
  CHECK_FALSE(result.recloser_closed);
  REQUIRE(result.events.size() == 2);
  CHECK(result.events[1].action == ProtectionSequenceAction::Lockout);
}

TEST_CASE("DER momentary cessation and synchronization gates reach explicit terminal states",
          "[reliability][protection-frt][momentary-cessation][synchronization]") {
  dynamics::IEEE1547Settings ieee;
  ieee.enabled = true;
  ieee.nominal_frequency_hz = 50.0;
  ieee.v_continuous_min_pu = 0.88;
  ieee.v_continuous_max_pu = 1.10;
  ieee.f_continuous_min_hz = 49.0;
  ieee.f_continuous_max_hz = 51.0;
  ieee.undervoltage_trip.clear();
  ieee.overvoltage_trip.clear();
  ieee.underfrequency_trip.clear();
  ieee.overfrequency_trip.clear();

  DERMomentaryCessationSettings cessation;
  cessation.enabled = true;
  cessation.enter_below_voltage_pu = 0.8;
  cessation.exit_above_voltage_pu = 0.9;
  cessation.exit_dwell_s = 0.05;
  auto result = classify_der_frt_trajectory(
      ieee, cessation,
      {{0.0, 1.0, 0.0}, {0.05, 0.7, 0.0}, {0.10, 0.7, 0.0}},
      0.10);
  CHECK(result.ever_momentary_ceased);
  CHECK(result.terminal_momentary_ceased);
  CHECK(result.terminal_class == DERFRTTerminalClass::MomentaryCessation);
  CHECK(result.terminal_restore_scale == Approx(0.0));

  result = classify_der_frt_trajectory(
      ieee, cessation,
      {{0.0, 1.0, 0.0}, {0.05, 0.7, 0.0}, {0.10, 0.95, 0.0},
       {0.15, 0.95, 0.0}},
      0.15);
  CHECK(result.ever_momentary_ceased);
  CHECK_FALSE(result.terminal_momentary_ceased);
  CHECK(result.terminal_class == DERFRTTerminalClass::RideThrough);
  CHECK(result.first_momentary_recovery_time_s == Approx(0.15));

  cessation.maximum_duration_s = 0.04;
  result = classify_der_frt_trajectory(
      ieee, cessation,
      {{0.0, 1.0, 0.0}, {0.05, 0.7, 0.0}, {0.10, 0.7, 0.0}},
      0.10);
  CHECK(result.terminal_class == DERFRTTerminalClass::Tripped);
  CHECK(result.trip_reason == "momentary cessation maximum duration exceeded");

  MicrogridSynchronizationInput synchronization;
  synchronization.voltage_difference_pu = 0.03;
  synchronization.frequency_difference_hz = 0.05;
  synchronization.angle_difference_rad = 0.1;
  auto synchronized = evaluate_microgrid_synchronization(synchronization);
  CHECK(synchronized.close_permitted);
  synchronization.angle_difference_rad = 0.3;
  synchronized = evaluate_microgrid_synchronization(synchronization);
  CHECK_FALSE(synchronized.close_permitted);
  CHECK_FALSE(synchronized.angle_within_window);
  synchronization.angle_difference_rad = 0.0;
  synchronization.close_command_channel_available = false;
  synchronized = evaluate_microgrid_synchronization(synchronization);
  CHECK_FALSE(synchronized.close_permitted);
}

TEST_CASE("automatic protection topology derives stable-id load consequence",
          "[reliability][protection-frt][topology]") {
  HybridPowerSystem system;
  ACBus source;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  ACBus downstream;
  downstream.index = 2;
  downstream.bus_type = BusType::PQ;
  downstream.pd_mw = 1.5;
  downstream.n_customers = 10;
  system.ac.buses = {source, downstream};
  ACBranch branch;
  branch.index = 101;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.in_service = true;
  branch.x_pu = 0.1;
  system.ac.branches = {branch};
  Load load;
  load.index = 201;
  load.bus = 2;
  load.p_mw = 2.0;
  load.scaling = 0.5;
  load.n_customers = 5;
  system.ac.loads = {load};

  AutomaticProtectionTopologyInput action;
  action.opened_ac_branch_indices = {101};
  const auto result = evaluate_automatic_protection_topology(system, action);
  CHECK(result.stable_ids_resolved);
  CHECK(result.domain_qualified_topology_used);
  REQUIRE(result.deenergized_ac_bus_ids == std::vector<int>{2});
  CHECK(result.deenergized_dc_bus_ids.empty());
  CHECK(result.shed_mw == Approx(2.5));
  CHECK(result.interrupted_customers == 15);
  CHECK(result.islands_without_source == 1);
  CHECK_THROWS_AS(evaluate_automatic_protection_topology(
                      system, AutomaticProtectionTopologyInput{{}, {999}, {}}),
                  std::invalid_argument);
}

TEST_CASE("protection misoperation maps false alarms to annual reliability indices",
          "[reliability][protection-frt][misoperation]") {
  ProtectionMisoperationInput input;
  input.no_fault_decision_windows_per_year = 1000.0;
  input.false_trip_probability_per_window = 0.01;
  input.trip_channel_success_probability = 0.8;
  input.breaker_success_probability = 0.9;
  input.disconnected_load_mw = 5.0;
  input.restoration_duration_hr = 2.0;
  const auto result = evaluate_protection_misoperation(input);
  CHECK(result.false_trip_frequency_per_year == Approx(7.2));
  CHECK(result.eens_mwh_yr == Approx(72.0));
  CHECK(result.lole_hr_yr == Approx(14.4));
  CHECK(result.lolf_occ_yr == Approx(7.2));
}

TEST_CASE("protection cyber: shared topology common cause and battery are exact",
          "[reliability][protection-frt][cyber]") {
  std::vector<ProtectionCyberComponent> components{
      {"shared-center", 0.9, 1.0, 2.0, 0.5, "dc-1", 10.0, 10.0,
       "control-room"},
      {"detector", 0.8, 1.0, 1.0, 0.2},
      {"restorer", 0.7, 1.0, 1.0, 0.2}};
  ProtectionCyberFunction detection{
      "detection", {{{0, 1}}}, 5.0, 1.0, 0.99};
  ProtectionCyberFunction restoration{
      "restoration", {{{0, 2}}}, 5.0, 1.0, 0.99};
  const auto nominal = enumerate_protection_cyber_states(
      components, {detection, restoration});
  REQUIRE(nominal.probabilities_normalized);
  CHECK(nominal.shared_dependencies_modelled);
  REQUIRE(nominal.function_availability.size() == 2);
  CHECK(nominal.function_availability[0] == Approx(0.72).margin(1e-12));
  CHECK(nominal.function_availability[1] == Approx(0.63).margin(1e-12));

  const std::vector<ProtectionCyberEnvironment> common_cause{
      {"normal", 0.8, {}, {}, 0.0},
      {"control-room-fire", 0.2, {"control-room"}, {}, 0.0}};
  const auto conditioned = enumerate_protection_cyber_states(
      components, {detection, restoration}, common_cause);
  CHECK(conditioned.common_cause_conditioned);
  CHECK(conditioned.function_availability[0] == Approx(0.8 * 0.72).margin(1e-12));
  CHECK(conditioned.function_availability[1] == Approx(0.8 * 0.63).margin(1e-12));

  components[0].intrinsic_availability = 1.0;
  components[1].intrinsic_availability = 1.0;
  components[2].intrinsic_availability = 1.0;
  const std::vector<ProtectionCyberEnvironment> power_states{
      {"short-outage", 0.5, {}, {"dc-1"}, 0.5},
      {"long-outage", 0.5, {}, {"dc-1"}, 2.0}};
  const auto power = enumerate_protection_cyber_states(
      components, {detection}, power_states);
  CHECK(power.power_dependency_modelled);
  CHECK(power.function_availability[0] == Approx(0.5).margin(1e-12));

  auto missing_power = components;
  missing_power[0].power_draw_w = 0.0;
  CHECK_THROWS_AS(enumerate_protection_cyber_states(
                      missing_power, {detection}, power_states),
                  std::invalid_argument);
  std::vector<ProtectionCyberComponent> too_many(21);
  for (size_t i = 0; i < too_many.size(); ++i)
    too_many[i].id = "component-" + std::to_string(i);
  CHECK_THROWS_AS(enumerate_protection_cyber_states(too_many, {}),
                  std::invalid_argument);
}

namespace {
ProtectionCyberReliabilityScenario make_protection_cyber_oracle(
    double center_availability) {
  ProtectionCyberReliabilityScenario scenario;
  scenario.scenario_id = "line-fault-oracle";
  scenario.initiating_frequency_per_year = 2.0;
  auto& event = scenario.event;
  event.primary.relay.relay_id = "primary";
  event.primary.relay.characteristic =
      ProtectionRelayCharacteristic::DefiniteTimeOvercurrent;
  event.primary.relay.pickup_current = 1.0;
  event.primary.relay.definite_time_delay_s = 0.1;
  event.primary.trajectory = {{0.0, 2.0}, {1.0, 2.0}};
  event.primary.relay_success_probability = 0.9;
  event.primary.breaker_success_probability = 1.0;
  event.backup = event.primary;
  event.backup.relay.relay_id = "backup";
  event.backup.relay.definite_time_delay_s = 0.5;
  event.backup.relay_success_probability = 1.0;
  event.coordination_margin_s = 0.3;
  event.uncleared_terminal_time_s = 2.0;
  event.information_components = {
      {"shared-center", center_availability, 1.0}};
  ProtectionCyberFunction function;
  function.name = "shared-control";
  function.alternative_paths = {{{0}}};
  event.information_functions = {function};
  event.function_bindings.detection_function = 0;
  event.function_bindings.isolation_function = 0;
  event.function_bindings.restoration_function = 0;

  scenario.consequence.primary_clearing_shed_mw = 10.0;
  scenario.consequence.backup_clearing_shed_mw = 10.0;
  scenario.consequence.uncleared_shed_mw = 10.0;
  scenario.consequence.isolated_shed_mw = 2.0;
  scenario.consequence.isolation_failed_shed_mw = 8.0;
  scenario.consequence.restored_shed_mw = 0.0;
  scenario.consequence.restoration_failed_shed_mw = 5.0;
  scenario.consequence.automatic_restoration_hr = 0.1;
  scenario.consequence.manual_restoration_hr = 1.0;
  scenario.consequence.repair_hr = 10.0;
  return scenario;
}
}  // namespace

TEST_CASE("protection cyber: three-method reliability comparison matches oracle",
          "[reliability][protection-frt][cyber][comparison]") {
  const auto comparison = compare_protection_cyber_reliability(
      {make_protection_cyber_oracle(0.8)});
  // Protection-only per event:
  // 0.9*(10*0.1/3600+2*0.1)+0.1*(10*0.5/3600+2*0.1).
  const double protection_per_event =
      0.9 * (10.0 * 0.1 / 3600.0 + 0.2) +
      0.1 * (10.0 * 0.5 / 3600.0 + 0.2);
  // Center-down class: 10*2/3600 + 8*1 +
  // 5*(10-1-2/3600) MWh per initiating event.
  const double center_down_per_event =
      10.0 * 2.0 / 3600.0 + 8.0 +
      5.0 * (10.0 - 1.0 - 2.0 / 3600.0);
  const double cyber_per_event =
      0.8 * protection_per_event + 0.2 * center_down_per_event;
  const double protection_lole_per_event =
      0.9 * (0.1 / 3600.0 + 0.1) +
      0.1 * (0.5 / 3600.0 + 0.1);
  const double cyber_lole_per_event =
      0.8 * protection_lole_per_event + 0.2 * 10.0;
  CHECK(comparison.static_fmea_eens_mwh_yr == Approx(200.0).margin(1e-12));
  CHECK(comparison.protection_only_eens_mwh_yr ==
        Approx(2.0 * protection_per_event).margin(1e-12));
  CHECK(comparison.cyber_conditioned_eens_mwh_yr ==
        Approx(2.0 * cyber_per_event).margin(1e-12));
  CHECK(comparison.protection_only_eens_mwh_yr <
        comparison.cyber_conditioned_eens_mwh_yr);
  CHECK(comparison.cyber_conditioned_eens_mwh_yr <
        comparison.static_fmea_eens_mwh_yr);
  CHECK(comparison.static_fmea_lole_hr_yr == Approx(20.0));
  CHECK(comparison.protection_only_lole_hr_yr ==
        Approx(2.0 * protection_lole_per_event).margin(1e-12));
  CHECK(comparison.cyber_conditioned_lole_hr_yr ==
        Approx(2.0 * cyber_lole_per_event).margin(1e-12));
  CHECK(comparison.static_fmea_lolf_occ_yr == Approx(2.0));
  CHECK(comparison.protection_only_lolf_occ_yr == Approx(2.0));
  CHECK(comparison.cyber_conditioned_lolf_occ_yr == Approx(2.0));
  CHECK(comparison.validity.information_topology_modelled);
  CHECK_FALSE(comparison.validity.online_network_dae_coupled);

  const auto degraded = compare_protection_cyber_reliability(
      {make_protection_cyber_oracle(0.4)});
  CHECK(degraded.cyber_conditioned_eens_mwh_yr >
        comparison.cyber_conditioned_eens_mwh_yr);
  CHECK(degraded.cyber_conditioned_lole_hr_yr >
        comparison.cyber_conditioned_lole_hr_yr);
}

// ─────────────────────────────────────────────────────────────────────────
// Failure-mode catalog
// ─────────────────────────────────────────────────────────────────────────

namespace {
HybridPowerSystem make_switch_breaker_vsc_system() {
  HybridPowerSystem sys;
  ACBus a;
  a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true;
  sys.ac.buses = {a};

  Switch sw;
  sw.index = 1; sw.bus_from = 1; sw.bus_to = 1; sw.in_service = true;
  sw.p_sw_fail = 0.02;
  sys.ac.switches = {sw};

  CircuitBreaker cb;
  cb.index = 1; cb.bus_from = 1; cb.bus_to = 1; cb.in_service = true;
  sys.ac.circuit_breakers = {cb};

  VSCConverter v;
  v.index = 1; v.bus_ac = 1; v.bus_dc = 101; v.in_service = true;
  v.forced_outage_rate = 0.01; v.mttr_hr = 24.0;
  sys.vsc_converters = {v};

  return sys;
}
}  // namespace

TEST_CASE("catalog: switch emits active + passive + cyber modes",
          "[reliability][failure_mode][catalog]") {
  auto sys = make_switch_breaker_vsc_system();
  FailureModeCatalogOptions opt;
  auto cat = build_failure_mode_catalog(sys, opt, ReliabilityDataPolicy{});

  // Collect the switch mode ids.
  std::vector<std::string> sw_modes;
  for (const auto& e : cat)
    if (e.mode.ref.component.kind == ReliabilityComponentKind::ACSwitch)
      sw_modes.push_back(e.mode.ref.mode_id);

  auto has_suffix = [&](const std::string& suf) {
    for (const auto& m : sw_modes)
      if (m.size() >= suf.size() && m.compare(m.size() - suf.size(), suf.size(), suf) == 0)
        return true;
    return false;
  };
  CHECK(has_suffix("/hardware_outage"));
  CHECK(has_suffix("/fail_to_open"));
  CHECK(has_suffix("/fail_to_close"));
  CHECK(has_suffix("/comm_loss"));

  // The active fail-to-open mode must carry the case p_sw_fail as p_demand.
  for (const auto& e : cat) {
    if (e.mode.ref.consequence == FailureConsequenceKind::FailToOpen) {
      CHECK(e.mode.ref.activation == FailureActivation::ActiveOnDemand);
      CHECK(e.mode.probability_per_demand == Approx(0.02));
      CHECK(e.mode.params.lambda_active_per_year == Approx(2.0 * 0.02));
    }
  }
}

TEST_CASE("catalog: fuse uses one-shot clearing failure and has no close command",
          "[reliability][failure_mode][catalog][fuse]") {
  HybridPowerSystem sys;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  sys.ac.buses = {bus};
  Switch fuse;
  fuse.index = 5;
  fuse.name = "F5";
  fuse.bus_from = 1;
  fuse.bus_to = 1;
  fuse.switch_type = SwitchType::Fuse;
  fuse.p_fail_to_open = 0.003;
  fuse.fuse_protection.replace_after_operation = true;
  fuse.fuse_protection.replacement_time_hr = 1.5;
  sys.ac.switches = {fuse};

  const auto catalog = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  bool saw_fail_to_clear = false;
  bool saw_fail_to_close = false;
  for (const auto& entry : catalog) {
    if (entry.mode.ref.component.kind != ReliabilityComponentKind::ACSwitch)
      continue;
    saw_fail_to_clear |= entry.mode.ref.mode_id.find("/fail_to_clear") !=
                         std::string::npos;
    saw_fail_to_close |= entry.mode.ref.mode_id.find("/fail_to_close") !=
                         std::string::npos;
    if (entry.mode.ref.mode_id.find("/fail_to_clear") != std::string::npos)
      CHECK(entry.mode.probability_per_demand == Approx(0.003));
  }
  CHECK(saw_fail_to_clear);
  CHECK_FALSE(saw_fail_to_close);
}

TEST_CASE("catalog: breaker emits fail-to-trip protection mode + VSC grid-forming",
          "[reliability][failure_mode][catalog]") {
  auto sys = make_switch_breaker_vsc_system();
  FailureModeCatalogOptions opt;
  auto cat = build_failure_mode_catalog(sys, opt, ReliabilityDataPolicy{});

  bool saw_fail_to_trip = false, saw_grid_forming = false;
  for (const auto& e : cat) {
    if (e.mode.ref.consequence == FailureConsequenceKind::FailToTrip &&
        e.mode.ref.cause == FailureCause::ProtectionLogic)
      saw_fail_to_trip = true;
    if (e.mode.ref.consequence == FailureConsequenceKind::GridFormingUnavailable)
      saw_grid_forming = true;
  }
  CHECK(saw_fail_to_trip);
  CHECK(saw_grid_forming);
}

TEST_CASE("catalog: coverage summary splits active/passive and physical/cyber",
          "[reliability][failure_mode][coverage]") {
  auto sys = make_switch_breaker_vsc_system();
  FailureModeCatalogOptions opt;
  auto cat = build_failure_mode_catalog(sys, opt, ReliabilityDataPolicy{});
  auto cov = summarize_failure_mode_coverage(cat);

  CHECK(cov.modes_total == (int)cat.size());
  CHECK(cov.modes_active > 0);
  CHECK(cov.modes_passive > 0);
  CHECK(cov.modes_physical > 0);
  CHECK(cov.modes_cyber_control > 0);
  CHECK(cov.components_total == 4);  // AC bus + switch + breaker + VSC
}

TEST_CASE("catalog: activation filter excludes active modes",
          "[reliability][failure_mode][catalog]") {
  auto sys = make_switch_breaker_vsc_system();
  FailureModeCatalogOptions opt;
  opt.include_active_on_demand = false;
  auto cat = build_failure_mode_catalog(sys, opt, ReliabilityDataPolicy{});

  for (const auto& e : cat)
    CHECK(e.mode.ref.activation == FailureActivation::Passive);
}

// ─────────────────────────────────────────────────────────────────────────
// Rich-component catalog coverage (every rich model is represented)
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_rich_component_system() {
  HybridPowerSystem sys;
  ACBus a; a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true;
  sys.ac.buses = {a};

  ExternalGrid eg; eg.index = 1; eg.bus = 1; eg.in_service = true; eg.name = "Grid";
  sys.ac.external_grids = {eg};

  Load ld; ld.index = 1; ld.bus = 1; ld.in_service = true; ld.p_mw = 5.0;
  ld.n_customers = 80; ld.controllable = true;
  sys.ac.loads = {ld};

  Charger ch; ch.index = 1; ch.in_service = true; ch.mtbf_hours = 8760.0; ch.mttr_hours = 6.0;
  sys.ac.chargers = {ch};

  EnergyRouterPort port; port.index = 1; port.in_service = true;
  EnergyRouter er; er.index = 1; er.in_service = true;
  er.mtbf_hours = 4380.0; er.mttr_hours = 48.0; er.ports = {port};
  sys.energy_routers = {er};

  MobileStorage ms; ms.index = 1; ms.bus = 1; ms.in_service = true;
  ms.mtbf_hours = 4380.0; ms.mttr_hours = 24.0;
  sys.mobile_storage = {ms};

  VirtualPowerPlant vpp; vpp.index = 1; vpp.pcc_bus = 1; vpp.in_service = true;
  vpp.mtbf_hours = 4380.0; vpp.mttr_hours = 4.0;
  sys.vpps = {vpp};

  Microgrid mg; mg.index = 1; mg.in_service = true;
  mg.mtbf_hours = 8760.0; mg.mttr_hours = 4.0;
  sys.microgrids = {mg};
  return sys;
}
}  // namespace

TEST_CASE("catalog: rich components (grid/load/router/mobile/vpp/microgrid) are catalogued",
          "[reliability][failure_mode][catalog]") {
  auto sys = make_rich_component_system();
  FailureModeCatalogOptions opt;
  // Allow defaulting so template-only components (external grid, loads) are enabled.
  ReliabilityDataPolicy policy;
  policy.default_policy = ReliabilityDefaultPolicy::UseNamedTemplateForMissingOnly;
  auto cat = build_failure_mode_catalog(sys, opt, policy);

  auto saw_kind = [&](ReliabilityComponentKind k) {
    for (const auto& e : cat)
      if (e.mode.ref.component.kind == k) return true;
    return false;
  };
  CHECK(saw_kind(ReliabilityComponentKind::ExternalGrid));
  CHECK(saw_kind(ReliabilityComponentKind::ACLoad));
  CHECK(saw_kind(ReliabilityComponentKind::Charger));
  CHECK(saw_kind(ReliabilityComponentKind::EnergyRouter));
  CHECK(saw_kind(ReliabilityComponentKind::EnergyRouterPort));
  CHECK(saw_kind(ReliabilityComponentKind::MobileStorage));
  CHECK(saw_kind(ReliabilityComponentKind::VirtualPowerPlant));
  CHECK(saw_kind(ReliabilityComponentKind::Microgrid));

  // The energy-router outage resolves its MTBF/MTTR into a real lambda (case data).
  for (const auto& e : cat) {
    if (e.mode.ref.component.kind == ReliabilityComponentKind::EnergyRouter &&
        e.mode.ref.consequence == FailureConsequenceKind::ForcedOutage) {
      CHECK(e.mode.params.lambda_per_year == Approx(8760.0 / 4380.0));
      CHECK(e.mode.params.repair_hr == Approx(48.0));
    }
  }
}

TEST_CASE("catalog: LCC, dedicated DC storage, regulator and three-phase models are catalogued",
          "[reliability][failure_mode][catalog][complete]") {
  HybridPowerSystem sys;
  ACBus ac_bus;
  ac_bus.index = 9;
  ac_bus.name = "ACBus9";
  sys.ac.buses = {ac_bus};
  DCBus dc_bus;
  dc_bus.index = 10;
  dc_bus.name = "DCBus10";
  sys.dc.buses = {dc_bus};
  RegulatorControl regulator;
  regulator.index = 11;
  regulator.name = "Reg11";
  regulator.enabled = true;
  sys.ac.regulator_controls = {regulator};

  DCStorage storage;
  storage.index = 12;
  storage.name = "DCESS12";
  storage.in_service = true;
  storage.forced_outage_rate = 0.03;
  storage.mttr_hr = 6.0;
  sys.dc.dc_storage = {storage};

  LCCConverter lcc;
  lcc.index = 13;
  lcc.name = "LCC13";
  lcc.in_service = true;
  sys.lcc_converters = {lcc};

  ThreePhaseACSystem tp;
  ThreePhaseACBus tp_bus;
  tp_bus.index = 20;
  tp_bus.name = "TPBus20";
  tp.buses = {tp_bus};
  ThreePhaseACLine line;
  line.index = 21;
  line.name = "TPLine21";
  line.failure_rate = 0.2;
  line.mttr_hr = 5.0;
  tp.lines = {line};
  ThreePhaseTransformer transformer;
  transformer.index = 22;
  transformer.name = "TPT22";
  transformer.mtbf_hr = 8760.0;
  transformer.mttr_hr = 24.0;
  tp.transformers = {transformer};
  ThreePhaseLoad load;
  load.index = 23;
  load.name = "TPLoad23";
  tp.loads = {load};
  ThreePhaseGenerator generator;
  generator.index = 24;
  generator.name = "TPGen24";
  tp.generators = {generator};
  ThreePhaseExternalGrid grid;
  grid.index = 25;
  grid.name = "TPGrid25";
  tp.external_grids = {grid};
  ThreePhaseRegulatorControl tp_regulator;
  tp_regulator.index = 26;
  tp_regulator.name = "TPReg26";
  tp_regulator.enabled = true;
  tp.regulator_controls = {tp_regulator};
  sys.three_phase_ac = tp;

  const auto catalog = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  std::set<ReliabilityComponentKind> kinds;
  for (const auto& entry : catalog) kinds.insert(entry.mode.ref.component.kind);
  CHECK(kinds.count(ReliabilityComponentKind::ACBus) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::DCBus) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ACRegulatorControl) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::DCDedicatedStorage) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::LCCConverter) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseACBus) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseACLine) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseTransformer) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseLoad) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseGenerator) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseExternalGrid) == 1);
  CHECK(kinds.count(ReliabilityComponentKind::ThreePhaseRegulatorControl) == 1);
}

TEST_CASE("configuration: transformer override has precedence and validates",
          "[reliability][configuration][transformer]") {
  HybridPowerSystem sys;
  Transformer2W transformer;
  transformer.index = 7;
  transformer.name = "MainTransformer";
  transformer.in_service = true;
  transformer.mtbf_hours = 87600.0;
  transformer.mttr_hours = 100.0;
  sys.ac.transformers_2w = {transformer};

  const auto baseline = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  const auto it = std::find_if(baseline.begin(), baseline.end(), [](const auto& entry) {
    return entry.mode.ref.consequence == FailureConsequenceKind::ForcedOutage;
  });
  REQUIRE(it != baseline.end());

  ReliabilityConfiguration configuration;
  FailureModeParameterOverride custom;
  custom.mode_id = it->mode.ref.mode_id;
  custom.failure_rate_per_year = 2.0;
  custom.repair_hr = 12.0;
  custom.isolation_hr = 0.25;
  configuration.mode_overrides = {custom};
  CHECK(validate_reliability_configuration(sys, configuration).ok());

  FailureModeCatalogOptions options;
  options.configuration = &configuration;
  const auto configured = build_failure_mode_catalog(
      sys, options, ReliabilityDataPolicy{});
  const auto configured_it = std::find_if(
      configured.begin(), configured.end(), [&](const auto& entry) {
        return entry.mode.ref.mode_id == custom.mode_id;
      });
  REQUIRE(configured_it != configured.end());
  CHECK(configured_it->mode.params.lambda_per_year == Approx(2.0));
  CHECK(configured_it->mode.repair_hr == Approx(12.0));
  CHECK(configured_it->mode.isolation_hr == Approx(0.25));
  CHECK(configured_it->mode.params.data_source == "user_override");
}

TEST_CASE("configuration: explicit zero hazards and repair durations override defaults",
          "[reliability][configuration][precedence]") {
  HybridPowerSystem sys;
  ACBranch branch;
  branch.index = 4;
  branch.name = "Line4";
  branch.in_service = true;
  branch.failure_rate = 0.8;
  branch.mttr_hr = 6.0;
  sys.ac.branches = {branch};
  CircuitBreaker breaker;
  breaker.index = 9;
  breaker.name = "CB9";
  breaker.in_service = true;
  sys.ac.circuit_breakers = {breaker};

  const auto baseline = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  const auto branch_mode = std::find_if(
      baseline.begin(), baseline.end(), [](const auto& entry) {
        return entry.mode.ref.component.kind == ReliabilityComponentKind::ACBranch &&
               entry.mode.ref.consequence == FailureConsequenceKind::ForcedOutage;
      });
  const auto trip_mode = std::find_if(
      baseline.begin(), baseline.end(), [](const auto& entry) {
        return entry.mode.ref.component.kind ==
                   ReliabilityComponentKind::ACCircuitBreaker &&
               entry.mode.ref.consequence == FailureConsequenceKind::FailToTrip;
      });
  REQUIRE(branch_mode != baseline.end());
  REQUIRE(trip_mode != baseline.end());

  FailureModeParameterOverride branch_override;
  branch_override.mode_id = branch_mode->mode.ref.mode_id;
  branch_override.failure_rate_per_year = 0.0;
  branch_override.mttr_hours = 0.0;
  FailureModeParameterOverride trip_override;
  trip_override.mode_id = trip_mode->mode.ref.mode_id;
  trip_override.demand_frequency_per_year = 12.0;
  trip_override.probability_per_demand = 0.0;
  ReliabilityConfiguration configuration;
  configuration.mode_overrides = {branch_override, trip_override};
  REQUIRE(validate_reliability_configuration(sys, configuration).ok());

  FailureModeCatalogOptions options;
  options.configuration = &configuration;
  const auto configured = build_failure_mode_catalog(
      sys, options, ReliabilityDataPolicy{});
  const auto configured_branch = std::find_if(
      configured.begin(), configured.end(), [&](const auto& entry) {
        return entry.mode.ref.mode_id == branch_override.mode_id;
      });
  const auto configured_trip = std::find_if(
      configured.begin(), configured.end(), [&](const auto& entry) {
        return entry.mode.ref.mode_id == trip_override.mode_id;
      });
  REQUIRE(configured_branch != configured.end());
  REQUIRE(configured_trip != configured.end());
  CHECK(configured_branch->mode.params.lambda_per_year == 0.0);
  CHECK(configured_branch->mode.params.repair_hr == 0.0);
  CHECK(configured_branch->mode.params.unavailability == 0.0);
  CHECK(configured_trip->mode.params.probability_per_demand == 0.0);
  CHECK(configured_trip->mode.params.lambda_active_per_year == 0.0);
  CHECK(configured_trip->mode.params.lambda_per_year == 0.0);
}

TEST_CASE("configuration: invalid probabilities and dangling protection references reject",
          "[reliability][configuration][validation]") {
  HybridPowerSystem sys;
  CircuitBreaker breaker;
  breaker.index = 1;
  breaker.name = "CB1";
  sys.ac.circuit_breakers = {breaker};
  const auto catalog = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  REQUIRE_FALSE(catalog.empty());

  ReliabilityConfiguration configuration;
  FailureModeParameterOverride invalid_mode;
  invalid_mode.mode_id = catalog.front().mode.ref.mode_id;
  invalid_mode.probability_per_demand = 1.1;
  configuration.mode_overrides = {invalid_mode};
  ProtectionConfiguration invalid_protection;
  invalid_protection.protection_id = "P1";
  invalid_protection.protective_device_id = "missing:device";
  invalid_protection.protected_component_id = "missing:asset";
  invalid_protection.zone_component_ids = {"missing:zone"};
  configuration.protection = {invalid_protection};
  const auto report = validate_reliability_configuration(sys, configuration);
  CHECK_FALSE(report.ok());
  CHECK(report.errors.size() >= 4);
}

TEST_CASE("configuration: duplicate component indices reject ambiguous stable IDs",
          "[reliability][configuration][validation]") {
  HybridPowerSystem sys;
  CircuitBreaker first;
  first.index = 7;
  first.name = "CB-A";
  CircuitBreaker second;
  second.index = 7;
  second.name = "CB-B";
  sys.ac.circuit_breakers = {first, second};

  const auto report = validate_reliability_configuration(
      sys, ReliabilityConfiguration{});
  CHECK_FALSE(report.ok());
  CHECK(std::any_of(report.errors.begin(), report.errors.end(),
                    [](const std::string& message) {
                      return message.find("duplicate stable component ID") !=
                             std::string::npos;
                    }));
}

TEST_CASE("configuration: protection ownership, uniqueness and zones are validated",
          "[reliability][configuration][protection][validation]") {
  HybridPowerSystem sys;
  ACBranch branch;
  branch.index = 1;
  branch.name = "ProtectedLine";
  branch.in_service = true;
  sys.ac.branches = {branch};
  CircuitBreaker breaker;
  breaker.index = 2;
  breaker.name = "MainBreaker";
  breaker.in_service = true;
  sys.ac.circuit_breakers = {breaker};
  Generator generator;
  generator.index = 3;
  generator.name = "NotAProtectionDevice";
  generator.in_service = true;
  sys.ac.generators = {generator};

  const auto catalog = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  auto component_id = [&](ReliabilityComponentKind kind) {
    const auto it = std::find_if(catalog.begin(), catalog.end(),
                                 [&](const auto& entry) {
      return entry.mode.ref.component.kind == kind;
    });
    REQUIRE(it != catalog.end());
    return it->mode.ref.component.stable_id;
  };
  const std::string branch_id = component_id(ReliabilityComponentKind::ACBranch);
  const std::string breaker_id =
      component_id(ReliabilityComponentKind::ACCircuitBreaker);
  const std::string generator_id =
      component_id(ReliabilityComponentKind::ACGenerator);

  ProtectionConfiguration valid;
  valid.protection_id = "P-main";
  valid.protective_device_id = breaker_id;
  valid.protected_component_id = branch_id;
  valid.zone_component_ids = {branch_id};

  SECTION("non-switching device cannot own protection") {
    valid.protective_device_id = generator_id;
    ReliabilityConfiguration configuration;
    configuration.protection = {valid};
    const auto report = validate_reliability_configuration(sys, configuration);
    CHECK_FALSE(report.ok());
    CHECK(std::any_of(report.errors.begin(), report.errors.end(),
                      [](const std::string& message) {
      return message.find("protective_device_id must reference a switch or circuit breaker") !=
             std::string::npos;
    }));
  }

  SECTION("one device cannot have two enabled protection rows") {
    auto duplicate = valid;
    duplicate.protection_id = "P-duplicate";
    ReliabilityConfiguration configuration;
    configuration.protection = {valid, duplicate};
    const auto report = validate_reliability_configuration(sys, configuration);
    CHECK_FALSE(report.ok());
    CHECK(std::any_of(report.errors.begin(), report.errors.end(),
                      [](const std::string& message) {
      return message.find("only one enabled protection row") != std::string::npos;
    }));
  }

  SECTION("enabled zone must include its protected component") {
    valid.zone_component_ids = {breaker_id};
    ReliabilityConfiguration configuration;
    configuration.protection = {valid};
    const auto report = validate_reliability_configuration(sys, configuration);
    CHECK_FALSE(report.ok());
    CHECK(std::any_of(report.errors.begin(), report.errors.end(),
                      [](const std::string& message) {
      return message.find("zone_component_ids must include protected_component_id") !=
             std::string::npos;
    }));
  }
}

TEST_CASE("configuration: custom protection zone targets explicit components",
          "[reliability][configuration][protection]") {
  HybridPowerSystem sys;
  ACBranch protected_branch;
  protected_branch.index = 1;
  protected_branch.name = "ProtectedLine";
  protected_branch.in_service = true;
  sys.ac.branches = {protected_branch};
  CircuitBreaker breaker;
  breaker.index = 2;
  breaker.name = "MainBreaker";
  breaker.in_service = true;
  sys.ac.circuit_breakers = {breaker};

  const auto baseline = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  const FailureModeCatalogEntry* trip = nullptr;
  std::string branch_id;
  for (const auto& entry : baseline) {
    if (entry.mode.ref.component.kind == ReliabilityComponentKind::ACCircuitBreaker &&
        entry.mode.ref.consequence == FailureConsequenceKind::FailToTrip)
      trip = &entry;
    if (entry.mode.ref.component.kind == ReliabilityComponentKind::ACBranch)
      branch_id = entry.mode.ref.component.stable_id;
  }
  REQUIRE(trip != nullptr);
  REQUIRE_FALSE(branch_id.empty());

  ReliabilityConfiguration configuration;
  ProtectionConfiguration protection;
  protection.protection_id = "P-main";
  protection.protective_device_id = trip->mode.ref.component.stable_id;
  protection.protected_component_id = branch_id;
  protection.zone_component_ids = {branch_id};
  protection.fail_to_trip_probability = 0.0;
  configuration.protection = {protection};
  REQUIRE(validate_reliability_configuration(sys, configuration).ok());
  configuration = resolve_reliability_configuration(sys, std::move(configuration));

  FailureModeCatalogOptions configured_options;
  configured_options.configuration = &configuration;
  const auto configured_catalog = build_failure_mode_catalog(
      sys, configured_options, ReliabilityDataPolicy{});
  const auto configured_trip = std::find_if(
      configured_catalog.begin(), configured_catalog.end(), [&](const auto& entry) {
        return entry.mode.ref.mode_id == trip->mode.ref.mode_id;
      });
  REQUIRE(configured_trip != configured_catalog.end());
  CHECK(configured_trip->mode.probability_per_demand == 0.0);
  CHECK(configured_trip->mode.params.lambda_active_per_year == 0.0);
  CHECK(configured_trip->mode.params.lambda_per_year == 0.0);

  ConsequenceModelCapabilities capabilities;
  capabilities.supports_protection_modeling = true;
  const auto patch = build_consequence_patch(
      sys, trip->mode, capabilities, &configuration);
  REQUIRE(patch.representable_by_selected_model);
  REQUIRE(patch.mutations.size() == 1);
  CHECK(patch.mutations.front().kind == MutationKind::ForceOutOfService);
  CHECK(patch.mutations.front().target_kind == ReliabilityComponentKind::ACBranch);
  CHECK(patch.mutations.front().target_index == 0);
}

TEST_CASE("configuration: every supported protective device consumes nuisance frequency",
          "[reliability][configuration][protection][catalog]") {
  HybridPowerSystem sys;
  Switch ac_switch;
  ac_switch.index = 1;
  ac_switch.name = "SW1";
  ac_switch.in_service = true;
  sys.ac.switches = {ac_switch};
  CircuitBreaker ac_breaker;
  ac_breaker.index = 2;
  ac_breaker.name = "ACCB2";
  ac_breaker.in_service = true;
  sys.ac.circuit_breakers = {ac_breaker};
  DCCircuitBreaker dc_breaker;
  dc_breaker.index = 3;
  dc_breaker.name = "DCCB3";
  dc_breaker.in_service = true;
  sys.dc.dc_circuit_breakers = {dc_breaker};

  const auto baseline = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  const std::array<ReliabilityComponentKind, 3> protective_kinds = {
      ReliabilityComponentKind::ACSwitch,
      ReliabilityComponentKind::ACCircuitBreaker,
      ReliabilityComponentKind::DCCircuitBreaker};
  ReliabilityConfiguration configuration;
  for (std::size_t i = 0; i < protective_kinds.size(); ++i) {
    const auto it = std::find_if(baseline.begin(), baseline.end(),
                                 [&](const auto& entry) {
      return entry.mode.ref.component.kind == protective_kinds[i];
    });
    REQUIRE(it != baseline.end());
    ProtectionConfiguration protection;
    protection.protection_id = "P" + std::to_string(i);
    protection.protective_device_id = it->mode.ref.component.stable_id;
    protection.protected_component_id = it->mode.ref.component.stable_id;
    protection.zone_component_ids = {it->mode.ref.component.stable_id};
    protection.nuisance_trip_frequency_per_year = 1.25 + static_cast<double>(i);
    configuration.protection.push_back(std::move(protection));
  }
  REQUIRE(validate_reliability_configuration(sys, configuration).ok());

  FailureModeCatalogOptions options;
  options.configuration = &configuration;
  const auto configured = build_failure_mode_catalog(
      sys, options, ReliabilityDataPolicy{});
  for (std::size_t i = 0; i < protective_kinds.size(); ++i) {
    const auto nuisance = std::find_if(configured.begin(), configured.end(),
                                       [&](const auto& entry) {
      return entry.mode.ref.component.kind == protective_kinds[i] &&
             entry.mode.ref.consequence == FailureConsequenceKind::NuisanceTrip;
    });
    REQUIRE(nuisance != configured.end());
    const double configured_calendar_frequency =
        1.25 + static_cast<double>(i);
    CHECK(nuisance->mode.params.calendar_frequency_per_year ==
          Approx(configured_calendar_frequency));
    CHECK((1.0 - nuisance->mode.params.unavailability) *
              nuisance->mode.params.lambda_per_year ==
          Approx(configured_calendar_frequency));
    CHECK(nuisance->mode.params.data_source == "user_override");
  }
}

TEST_CASE("configuration: unsupported protection-zone targets reject consequence execution",
          "[reliability][configuration][protection][consequence]") {
  HybridPowerSystem sys;
  CircuitBreaker breaker;
  breaker.index = 1;
  breaker.name = "MainBreaker";
  breaker.in_service = true;
  sys.ac.circuit_breakers = {breaker};
  LCCConverter lcc;
  lcc.index = 2;
  lcc.name = "UnsupportedLCC";
  lcc.in_service = true;
  sys.lcc_converters = {lcc};

  const auto baseline = build_failure_mode_catalog(
      sys, FailureModeCatalogOptions{}, ReliabilityDataPolicy{});
  const auto trip = std::find_if(baseline.begin(), baseline.end(),
                                 [](const auto& entry) {
    return entry.mode.ref.component.kind ==
               ReliabilityComponentKind::ACCircuitBreaker &&
           entry.mode.ref.consequence == FailureConsequenceKind::FailToTrip;
  });
  const auto lcc_mode = std::find_if(baseline.begin(), baseline.end(),
                                     [](const auto& entry) {
    return entry.mode.ref.component.kind == ReliabilityComponentKind::LCCConverter;
  });
  REQUIRE(trip != baseline.end());
  REQUIRE(lcc_mode != baseline.end());

  ReliabilityConfiguration configuration;
  ProtectionConfiguration protection;
  protection.protection_id = "P-LCC";
  protection.protective_device_id = trip->mode.ref.component.stable_id;
  protection.protected_component_id = lcc_mode->mode.ref.component.stable_id;
  protection.zone_component_ids = {lcc_mode->mode.ref.component.stable_id};
  configuration.protection = {protection};
  REQUIRE(validate_reliability_configuration(sys, configuration).ok());
  configuration = resolve_reliability_configuration(sys, std::move(configuration));

  ConsequenceModelCapabilities capabilities;
  capabilities.supports_protection_modeling = true;
  const auto patch = build_consequence_patch(
      sys, trip->mode, capabilities, &configuration);
  CHECK_FALSE(patch.representable_by_selected_model);
  CHECK(patch.mutations.empty());
  CHECK(patch.unsupported_reason.find("lcc_converter:2") != std::string::npos);
}

TEST_CASE("catalog: aggregated sources support forced-outage; control effects + unmodeled kinds unsupported",
          "[reliability][failure_mode][consequence]") {
  auto sys = make_rich_component_system();
  FailureModeCatalogOptions opt;
  ReliabilityDataPolicy policy;
  policy.default_policy = ReliabilityDefaultPolicy::UseNamedTemplateForMissingOnly;
  auto cat = build_failure_mode_catalog(sys, opt, policy);

  ConsequenceModelCapabilities caps;  // steady-state AC/DC shed defaults
  bool saw_grid_outage = false, saw_grid_control = false, saw_charger_outage = false;
  for (const auto& e : cat) {
    auto patch = build_consequence_patch(sys, e.mode, caps);
    const auto kind = e.mode.ref.component.kind;
    const auto cons = e.mode.ref.consequence;
    if (kind == ReliabilityComponentKind::ExternalGrid &&
        cons == FailureConsequenceKind::ForcedOutage) {
      saw_grid_outage = true;
      // External grid is now wired as a dispatchable source -> outage is modelled.
      CHECK(patch.representable_by_selected_model);
    }
    if (kind == ReliabilityComponentKind::ExternalGrid &&
        cons == FailureConsequenceKind::ControlUnavailable) {
      saw_grid_control = true;
      // Only forced-outage of an aggregated source is modelled by the shed engine.
      CHECK_FALSE(patch.representable_by_selected_model);
      CHECK_FALSE(patch.unsupported_reason.empty());
    }
    if (kind == ReliabilityComponentKind::Charger &&
        cons == FailureConsequenceKind::ForcedOutage) {
      saw_charger_outage = true;
      // Chargers are not represented by the steady-state shed engine.
      CHECK_FALSE(patch.representable_by_selected_model);
    }
  }
  CHECK(saw_grid_outage);
  CHECK(saw_grid_control);
  CHECK(saw_charger_outage);
}

// ─────────────────────────────────────────────────────────────────────────
// Active-failure parameter data-policy honesty
// ─────────────────────────────────────────────────────────────────────────
TEST_CASE("resolver: template active params resolve to default, missing under strict",
          "[reliability][resolver][active]") {
  ReliabilityRawFields raw;
  raw.is_active = true;
  raw.probability_per_demand = 0.005;
  raw.demand_frequency_per_year = 1.0;
  raw.active_params_are_template = true;  // catalog default, not case data

  // Non-strict: template active resolves but is flagged "default" (not case).
  ReliabilityDataPolicy lenient;
  lenient.default_policy = ReliabilityDefaultPolicy::UseNamedTemplateForMissingOnly;
  auto p1 = resolve_reliability_params(raw, lenient);
  CHECK(p1.data_source == "default");
  CHECK_FALSE(p1.has_data);
  CHECK(p1.lambda_active_per_year == Approx(0.005));

  // Strict: template active params are NOT case data -> missing.
  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  auto p2 = resolve_reliability_params(raw, strict);
  CHECK(p2.data_source == "missing");
  CHECK_FALSE(p2.has_data);

  // Case-sourced active params (template flag false) are "case" even under strict.
  raw.active_params_are_template = false;
  auto p3 = resolve_reliability_params(raw, strict);
  CHECK(p3.data_source == "case");
  CHECK(p3.has_data);
}

// ─────────────────────────────────────────────────────────────────────────
// Forced-shed-at-load: load-point interruption counts as shed
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_single_load_system() {
  HybridPowerSystem sys;
  ACBus a; a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true;
  sys.ac.buses = {a};
  Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 50.0; g.pmin_mw = 0.0; g.cost_c1 = 10.0; g.is_slack = true;
  sys.ac.generators = {g};
  Load ld; ld.index = 1; ld.bus = 1; ld.in_service = true; ld.p_mw = 5.0;
  ld.n_customers = 40;
  sys.ac.loads = {ld};
  return sys;
}
}  // namespace

TEST_CASE("fmea: load-point interruption counts the load MW as shed",
          "[reliability][failure_mode][forced_shed]") {
  auto sys = make_single_load_system();
  FailureModeFMEAOptions opt;  // default policy fills template defaults
  auto res = run_failure_mode_fmea(sys, opt);

  bool found = false;
  for (const auto& c : res.contingencies) {
    if (c.ref.component.kind == ReliabilityComponentKind::ACLoad &&
        c.ref.consequence == FailureConsequenceKind::ForcedOutage) {
      found = true;
      CHECK(c.supported);
      CHECK(c.total_shed_mw == Approx(5.0).margin(1e-6));
      CHECK(c.causes_loss);
      CHECK(c.eens_contribution > 0.0);
    }
  }
  CHECK(found);
  CHECK(res.eens_mwh_yr > 0.0);
}

// ─────────────────────────────────────────────────────────────────────────
// External grid wired as a source: its outage sheds the load it supplied
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_external_grid_load_system() {
  HybridPowerSystem sys;
  ACBus a; a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true;
  sys.ac.buses = {a};
  ExternalGrid eg; eg.index = 1; eg.bus = 1; eg.in_service = true; eg.name = "MainGrid";
  sys.ac.external_grids = {eg};
  Load ld; ld.index = 1; ld.bus = 1; ld.in_service = true; ld.p_mw = 10.0;
  ld.n_customers = 100;
  sys.ac.loads = {ld};
  return sys;
}
}  // namespace

TEST_CASE("fmea: external grid outage sheds the load it supplied",
          "[reliability][failure_mode][external_grid]") {
  auto sys = make_external_grid_load_system();
  FailureModeFMEAOptions opt;  // default policy enables the template-filled grid mode
  auto res = run_failure_mode_fmea(sys, opt);

  bool found = false;
  for (const auto& c : res.contingencies) {
    if (c.ref.component.kind == ReliabilityComponentKind::ExternalGrid &&
        c.ref.consequence == FailureConsequenceKind::ForcedOutage) {
      found = true;
      CHECK(c.supported);
      CHECK(c.total_shed_mw == Approx(10.0).margin(0.5));
      CHECK(c.causes_loss);
    }
  }
  CHECK(found);
}

// ─────────────────────────────────────────────────────────────────────────
// F15: cyber/control converter modes now move EENS (communication loss freezes
// the dispatch setpoint; derating severity is data-driven, not a flat 0.5).
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_vsc_cyber_system() {
  HybridPowerSystem sys;
  ACBus a; a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true;
  sys.ac.buses = {a};
  Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 20.0; g.pmin_mw = 0.0; g.is_slack = true; g.forced_outage_rate = 0.0;
  sys.ac.generators = {g};
  DCBus d; d.index = 101; d.in_service = true;
  sys.dc.buses = {d};
  DCLoad dl; dl.index = 1; dl.bus = 101; dl.in_service = true; dl.p_mw = 8.0;
  dl.n_customers = 40;
  sys.dc.loads = {dl};
  // The controllable VSC is the DC island's only infeed, with setpoint 0, so a
  // frozen / dropped control channel removes the infeed entirely.
  VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 101; v.in_service = true;
  v.pmax_mw = 10.0; v.pmin_mw = -10.0; v.p_rated_mw = 10.0; v.p_set_mw = 0.0;
  v.controllable = true;
  sys.vsc_converters = {v};
  return sys;
}
}  // namespace

TEST_CASE("fmea: cyber/control converter modes move EENS (F15)",
          "[reliability][failure_mode][f15]") {
  auto sys = make_vsc_cyber_system();
  FailureModeFMEAOptions opt;  // default policy fills template frequencies
  auto res = run_failure_mode_fmea(sys, opt);

  const FailureModeContingency* comm = nullptr;
  const FailureModeContingency* frozen = nullptr;
  const FailureModeContingency* derate = nullptr;
  const FailureModeContingency* meas = nullptr;
  for (const auto& c : res.contingencies) {
    if (c.ref.component.kind != ReliabilityComponentKind::VSCConverter) continue;
    switch (c.ref.consequence) {
      case FailureConsequenceKind::CommunicationLoss: comm = &c; break;
      case FailureConsequenceKind::SetpointFrozen:    frozen = &c; break;
      case FailureConsequenceKind::Derating:          derate = &c; break;
      case FailureConsequenceKind::MeasurementBias:   meas = &c; break;
      default: break;
    }
  }
  REQUIRE(comm != nullptr);
  REQUIRE(frozen != nullptr);
  REQUIRE(derate != nullptr);
  REQUIRE(meas != nullptr);

  // Communication loss freezes the dispatch setpoint (pinned at 0) -> the DC
  // island loses its only infeed -> full 8 MW shed, and it now moves EENS.
  CHECK(comm->supported);
  CHECK(comm->total_shed_mw == Approx(8.0).margin(0.5));
  CHECK(comm->eens_contribution > 0.0);

  // Setpoint-frozen is the same control-loss consequence.
  CHECK(frozen->supported);
  CHECK(frozen->total_shed_mw == Approx(8.0).margin(0.5));

  // Data-driven derating: surviving fraction 0.7 -> pmax 10->7 -> 1 MW shed.
  // A hard-coded 0.5 would derate to 5 MW and shed 3 MW instead.
  CHECK(derate->supported);
  CHECK(derate->total_shed_mw == Approx(1.0).margin(0.5));

  // Measurement bias has no steady-state shed effect (honest unsupported).
  CHECK_FALSE(meas->supported);
  CHECK(meas->total_shed_mw == Approx(0.0).margin(1e-6));

  CHECK(res.eens_mwh_yr > 0.0);
}

// ─────────────────────────────────────────────────────────────────────────
// Multi-mode (N-2) co-failure enumeration: two parallel branches each carry
// the load alone (no single-mode shed), but their co-failure islands the load.
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_n2_branch_system() {
  HybridPowerSystem sys;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.in_service = true;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.in_service = true;
  b2.pd_mw = 8.0;
  sys.ac.buses = {b1, b2};
  // Generator carries no reliability data -> its modes are disabled under the
  // strict policy, leaving a perfectly reliable source (only the branches fail).
  Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 20.0; g.pmin_mw = 0.0; g.is_slack = true;
  sys.ac.generators = {g};
  // Two parallel 1-2 branches, each rated 10 MVA with explicit failure data.
  ACBranch a; a.index = 1; a.from_bus = 1; a.to_bus = 2; a.in_service = true;
  a.x_pu = 0.1; a.rate_a_mva = 10.0; a.failure_rate = 0.5; a.mttr_hr = 10.0;
  ACBranch bb; bb.index = 2; bb.from_bus = 1; bb.to_bus = 2; bb.in_service = true;
  bb.x_pu = 0.1; bb.rate_a_mva = 10.0; bb.failure_rate = 0.5; bb.mttr_hr = 10.0;
  sys.ac.branches = {a, bb};
  return sys;
}
}  // namespace

TEST_CASE("fmea: multi-mode co-failure enumeration captures N-2 risk",
          "[reliability][failure_mode][n2]") {
  auto sys = make_n2_branch_system();
  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;

  // Single-mode (default max_order = 1): either branch alone leaves the parallel
  // branch (rated 10) to carry the 8 MW load -> no shed, no co-contingencies.
  FailureModeFMEAOptions opt1;
  opt1.data_policy = strict;
  auto r1 = run_failure_mode_fmea(sys, opt1);
  CHECK(r1.eens_mwh_yr == Approx(0.0).margin(1e-6));
  CHECK(r1.co_contingencies.empty());
  CHECK(r1.n_pairs_evaluated == 0);

  // Second-order (max_order = 2): the double branch outage islands bus 2 -> the
  // full 8 MW is shed, and this N-2 state now moves EENS.
  FailureModeFMEAOptions opt2;
  opt2.data_policy = strict;
  opt2.max_order = 2;
  auto r2 = run_failure_mode_fmea(sys, opt2);
  CHECK(r2.n_pairs_evaluated >= 1);
  CHECK(r2.eens_mwh_yr > 0.0);
  CHECK(r2.second_order_expansion_complete);
  REQUIRE_FALSE(r2.co_contingencies.empty());
  CHECK(r2.co_contingencies.front().total_shed_mw == Approx(8.0).margin(0.5));
  CHECK(r2.co_contingencies.front().eens_contribution > 0.0);
  CHECK(r2.co_contingencies.front().interaction_eens_correction ==
        Approx(r2.co_contingencies.front().eens_contribution).margin(1e-9));
  CHECK(r2.eens_mwh_yr ==
        Approx(r2.second_order_interaction_eens_mwh_yr).margin(1e-9));
  CHECK(r2.co_contingencies.front().joint_unavailability > 0.0);
}

TEST_CASE("fmea: exact N-2 interaction vanishes for additive island consequences",
          "[reliability][failure_mode][n2][interaction]") {
  HybridPowerSystem sys;
  ACBus source;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  source.in_service = true;
  ACBus load_a;
  load_a.index = 2;
  load_a.bus_type = BusType::PQ;
  load_a.pd_mw = 3.0;
  load_a.in_service = true;
  ACBus load_b = load_a;
  load_b.index = 3;
  load_b.pd_mw = 5.0;
  sys.ac.buses = {source, load_a, load_b};

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  generator.pmax_mw = 20.0;
  sys.ac.generators = {generator};

  ACBranch branch_a;
  branch_a.index = 1;
  branch_a.from_bus = 1;
  branch_a.to_bus = 2;
  branch_a.in_service = true;
  branch_a.x_pu = 0.1;
  branch_a.rate_a_mva = 10.0;
  branch_a.failure_rate = 0.5;
  branch_a.mttr_hr = 10.0;
  ACBranch branch_b = branch_a;
  branch_b.index = 2;
  branch_b.to_bus = 3;
  sys.ac.branches = {branch_a, branch_b};

  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  FailureModeFMEAOptions first_order_options;
  first_order_options.data_policy = strict;
  const auto first_order =
      run_failure_mode_fmea(sys, first_order_options);

  FailureModeFMEAOptions second_order_options = first_order_options;
  second_order_options.max_order = 2;
  const auto second_order =
      run_failure_mode_fmea(sys, second_order_options);

  REQUIRE(second_order.second_order_expansion_complete);
  REQUIRE_FALSE(second_order.co_contingencies.empty());
  CHECK(second_order.co_contingencies.front().total_shed_mw ==
        Approx(8.0).margin(0.5));
  CHECK(second_order.co_contingencies.front().eens_contribution > 0.0);
  CHECK(second_order.co_contingencies.front().interaction_eens_correction ==
        Approx(0.0).margin(1e-9));
  CHECK(second_order.second_order_interaction_eens_mwh_yr ==
        Approx(0.0).margin(1e-9));
  CHECK(second_order.eens_mwh_yr ==
        Approx(first_order.eens_mwh_yr).margin(1e-9));
}

// ─────────────────────────────────────────────────────────────────────────
// Active switching: fail-to-open/stuck-closed expand the isolation zone;
// fail-to-close is deferred to the three-stage restoration engine.
// ─────────────────────────────────────────────────────────────────────────
TEST_CASE("consequence: switch fail-to-open expands the isolation zone; fail-to-close deferred",
          "[reliability][failure_mode][protection]") {
  HybridPowerSystem sys;
  ACBus b1; b1.index = 1; b1.in_service = true;
  ACBus b2; b2.index = 2; b2.in_service = true;
  sys.ac.buses = {b1, b2};
  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.in_service = true;
  sys.ac.branches = {br};
  Switch sw; sw.index = 1; sw.bus_from = 2; sw.bus_to = 2; sw.in_service = true;
  sw.closed = true; sw.p_sw_fail = 0.02;
  sys.ac.switches = {sw};

  FailureModeCatalogOptions opt;
  auto cat = build_failure_mode_catalog(sys, opt, ReliabilityDataPolicy{});
  ConsequenceModelCapabilities caps;
  caps.supports_protection_modeling = true;  // FMEA engine enables this

  bool saw_open = false, saw_close = false;
  for (const auto& e : cat) {
    if (e.mode.ref.component.kind != ReliabilityComponentKind::ACSwitch) continue;
    if (e.mode.ref.consequence == FailureConsequenceKind::FailToOpen) {
      saw_open = true;
      auto patch = build_consequence_patch(sys, e.mode, caps);
      CHECK(patch.representable_by_selected_model);
      REQUIRE(patch.mutations.size() == 1);
      CHECK(patch.mutations[0].kind == MutationKind::ProtectionZoneExpansion);
      // Applying it de-energizes the branch incident to the switch's bus_from.
      auto sys_m = apply_consequence_patch(sys, patch);
      CHECK_FALSE(sys_m.ac.branches[0].in_service);
    }
    if (e.mode.ref.consequence == FailureConsequenceKind::FailToClose) {
      saw_close = true;
      auto patch = build_consequence_patch(sys, e.mode, caps);
      CHECK_FALSE(patch.representable_by_selected_model);  // deferred to three-stage
    }
  }
  CHECK(saw_open);
  CHECK(saw_close);
}

// ─────────────────────────────────────────────────────────────────────────
// End-to-end: non-sequential MC on a hybrid AC/DC system includes DC load
// curtailment in EENS (MC now routes hybrid states through the hybrid LP).
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_hybrid_mc_system() {
  HybridPowerSystem sys;
  // AC bus 1: slack with a reliable generator (the only physical source).
  ACBus a; a.index = 1; a.bus_type = BusType::SLACK; a.in_service = true; a.pd_mw = 0.0;
  sys.ac.buses = {a};
  Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 20.0; g.pmin_mw = 0.0; g.is_slack = true; g.forced_outage_rate = 0.0;
  sys.ac.generators = {g};
  // DC bus 101 with a 5 MW DC load, fed only through the VSC.
  DCBus d; d.index = 101; d.in_service = true; d.pd_mw = 0.0;
  sys.dc.buses = {d};
  DCLoad dl; dl.index = 1; dl.bus = 101; dl.in_service = true; dl.p_mw = 5.0;
  dl.n_customers = 50;
  sys.dc.loads = {dl};
  // VSC AC<->DC: when it fails the DC load loses its only source.
  VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 101; v.in_service = true;
  v.pmax_mw = 10.0; v.pmin_mw = -10.0; v.p_rated_mw = 10.0; v.controllable = true;
  v.forced_outage_rate = 0.1; v.mttr_hr = 24.0;
  sys.vsc_converters = {v};
  return sys;
}
}  // namespace

TEST_CASE("nsq MC: hybrid system includes DC load curtailment in EENS",
          "[reliability][mc][hybrid][e2e]") {
  auto sys = make_hybrid_mc_system();
  ReliabilityOptions opt;
  opt.max_iterations = 4000;
  opt.seed = 42;
  opt.compute_tail_risk = false;
  auto r = run_nonsequential_mc(sys, opt);

  CHECK(r.model_scope == "hybrid-acdc-network-lp");
  CHECK(r.validity.dc_load_curtailment_included);
  CHECK(r.validity.vsc_dc_power_flow_modelled);
  // VSC unavailability ~0.1 -> the 5 MW DC load is shed in those states, so the
  // hybrid-LP-based MC reports non-trivial EENS (previously zero under AC-only).
  CHECK(r.eens_mwh_yr > 100.0);
}

TEST_CASE("nsq MC importance sampling preserves the lambda-one identity",
          "[reliability][mc][importance]") {
  const auto sys = make_hybrid_mc_system();
  ReliabilityOptions plain;
  plain.max_iterations = 500;
  plain.seed = 31415;
  plain.compute_tail_risk = false;
  plain.enable_parallel = false;
  const auto reference = run_nonsequential_mc(sys, plain);

  ReliabilityOptions importance = plain;
  importance.use_importance_sampling = true;
  importance.importance_lambda = 1.0;
  const auto twisted = run_nonsequential_mc(sys, importance);

  CHECK(twisted.importance_sampling_used);
  CHECK(twisted.importance_twisting_factor == Approx(1.0));
  CHECK(twisted.importance_mean_likelihood_ratio == Approx(1.0));
  CHECK(twisted.importance_effective_sample_size == Approx(500.0));
  CHECK(twisted.eens_mwh_yr == Approx(reference.eens_mwh_yr));
  CHECK(twisted.lole_hr_yr == Approx(reference.lole_hr_yr));
}

TEST_CASE("nsq MC importance sampling reports likelihood diagnostics",
          "[reliability][mc][importance]") {
  const auto sys = make_hybrid_mc_system();
  ReliabilityOptions options;
  options.max_iterations = 2000;
  options.seed = 2718;
  options.compute_tail_risk = false;
  options.enable_parallel = false;
  options.use_importance_sampling = true;
  options.importance_lambda = 4.0;
  const auto result = run_nonsequential_mc(sys, options);

  CHECK(result.importance_sampling_used);
  CHECK(result.importance_twisting_factor == Approx(4.0));
  CHECK(result.importance_effective_sample_size > 0.0);
  CHECK(result.importance_effective_sample_size <= result.iterations_used);
  CHECK(result.importance_mean_likelihood_ratio == Approx(1.0).margin(0.08));
  CHECK(result.incremental_eens_mwh_yr > 100.0);
}

TEST_CASE("importance sampling rejects invalid or unsupported configurations",
          "[reliability][mc][importance]") {
  const auto sys = make_hybrid_mc_system();
  ReliabilityOptions invalid;
  invalid.max_iterations = 1;
  invalid.compute_tail_risk = false;
  invalid.use_importance_sampling = true;
  invalid.importance_lambda = 0.0;
  CHECK_THROWS_AS(run_nonsequential_mc(sys, invalid), std::invalid_argument);

  ReliabilityOptions sequential = invalid;
  sequential.importance_lambda = 2.0;
  sequential.hours_per_year = 1;
  LoadProfile profile;
  profile.factors = {1.0};
  CHECK_THROWS_AS(run_sequential_mc(sys, profile, sequential),
                  std::invalid_argument);
}

TEST_CASE("exact Birnbaum Fussell-Vesely and EENS derivative match hybrid oracle",
          "[reliability][sensitivity][exact]") {
  auto sys = make_hybrid_mc_system();
  sys.vsc_converters.front().index = 77;
  ReliabilityOptions options;
  options.compute_tail_risk = false;
  options.load_scale_factor = 1.0;
  options.data_policy.default_policy =
      ReliabilityDefaultPolicy::StrictCaseDataOnly;
  const auto result = compute_exact_reliability_sensitivity(sys, options);

  CHECK(result.exact_independent_binary_model);
  CHECK(result.states_evaluated == 2);
  CHECK(result.baseline_eens_mwh_yr == Approx(0.0).margin(1e-9));
  CHECK(result.expected_incremental_eens_mwh_yr ==
        Approx(0.1 * 5.0 * 8760.0).margin(1e-8));
  const auto converter = std::find_if(
      result.components.begin(), result.components.end(),
      [](const auto& item) { return item.component_type == "VSCConverter"; });
  REQUIRE(converter != result.components.end());
  CHECK(converter->component_position == 0);
  CHECK(converter->component_index == 77);
  CHECK(converter->eens_if_forced_down_mwh_yr ==
        Approx(5.0 * 8760.0).margin(1e-8));
  CHECK(converter->eens_if_forced_up_mwh_yr == Approx(0.0).margin(1e-9));
  CHECK(converter->birnbaum_mwh_yr_per_unit_unavailability ==
        Approx(5.0 * 8760.0).margin(1e-8));
  CHECK(converter->eens_derivative_mwh_yr_per_unit_unavailability ==
        Approx(converter->birnbaum_mwh_yr_per_unit_unavailability));
  CHECK(converter->fussell_vesely == Approx(1.0).margin(1e-12));
  CHECK(compute_exact_reliability_sensitivity(sys, options, 1)
            .states_evaluated == 2);

  auto two_stochastic_components = sys;
  two_stochastic_components.ac.generators.front().forced_outage_rate = 0.2;
  two_stochastic_components.ac.generators.front().mttr_hr = 24.0;
  CHECK_THROWS_AS(compute_exact_reliability_sensitivity(
                      two_stochastic_components, options, 1),
                  std::invalid_argument);
}

TEST_CASE("nsq MC: microgrid supervisory outage removes island support",
          "[reliability][mc][microgrid][e2e]") {
  HybridPowerSystem sys;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::PQ;
  bus.in_service = true;
  bus.pd_mw = 0.5;
  bus.n_customers = 10;
  sys.ac.buses = {bus};

  Microgrid microgrid;
  microgrid.index = 7;
  microgrid.name = "Island controller";
  microgrid.pcc_bus = 1;
  microgrid.in_service = true;
  microgrid.islanding_capability = true;
  microgrid.capacity_mw = 0.5;
  microgrid.mtbf_hours = 1.0;
  microgrid.mttr_hours = 8760.0;
  sys.microgrids = {microgrid};

  ReliabilityOptions options;
  options.max_iterations = 200;
  options.seed = 19;
  options.compute_tail_risk = false;
  const auto result = run_nonsequential_mc(sys, options);

  CHECK(result.eens_mwh_yr > 4000.0);
  const auto critical = std::find_if(
      result.critical_components.begin(), result.critical_components.end(),
      [](const auto& component) {
        return component.component_type == "Microgrid";
      });
  REQUIRE(critical != result.critical_components.end());
  CHECK(critical->component_name.find("Island controller") !=
        std::string::npos);
}

// ─────────────────────────────────────────────────────────────────────────
// F9: the hybrid LP now enforces DC power flow (Kirchhoff), so meshed loop
// flow is constrained by reactance instead of split freely (transport).
// ─────────────────────────────────────────────────────────────────────────
namespace {
HybridPowerSystem make_meshed_hybrid_system() {
  HybridPowerSystem sys;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.in_service = true;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.in_service = true; b2.pd_mw = 10.0;
  sys.ac.buses = {b1, b2};
  Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 50.0; g.pmin_mw = 0.0; g.is_slack = true;
  sys.ac.generators = {g};
  // Two parallel 1-2 branches with a 2:1 susceptance ratio (x=0.1 vs 0.2), each
  // rated 6 MVA.  Serving 10 MW forces the low-x branch to ~6.67 MW under
  // Kirchhoff -> overloaded -> ~1 MW shed even healthy.  Transport would split
  // 5/5 and shed nothing.
  ACBranch a; a.index = 1; a.from_bus = 1; a.to_bus = 2; a.in_service = true;
  a.x_pu = 0.1; a.r_pu = 0.0; a.rate_a_mva = 6.0; a.failure_rate = 0.0;
  ACBranch bb; bb.index = 2; bb.from_bus = 1; bb.to_bus = 2; bb.in_service = true;
  bb.x_pu = 0.2; bb.r_pu = 0.0; bb.rate_a_mva = 6.0; bb.failure_rate = 0.0;
  sys.ac.branches = {a, bb};
  // A trivial DC bus routes the evaluation through the hybrid AC/DC network LP.
  DCBus d; d.index = 101; d.in_service = true;
  sys.dc.buses = {d};
  return sys;
}
}  // namespace

TEST_CASE("hybrid LP: DC power flow constrains meshed loop flow (F9)",
          "[reliability][hybrid][f9]") {
  auto sys = make_meshed_hybrid_system();
  ReliabilityOptions opt;
  opt.max_iterations = 50;
  opt.seed = 3;
  opt.compute_tail_risk = false;
  opt.data_policy.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
  auto r = run_nonsequential_mc(sys, opt);

  CHECK(r.model_scope == "hybrid-acdc-network-lp");
  // Kirchhoff forces the 2:1 split, overloading the low-reactance branch and
  // shedding ~1 MW even in the healthy state (~1 MW * 8760 h).  A pure transport
  // LP would split freely and report ~0 EENS.
  CHECK(r.eens_mwh_yr > 1000.0);
  CHECK(r.baseline_eens_mwh_yr > 1000.0);
  CHECK(r.incremental_eens_mwh_yr == Approx(0.0).margin(1e-9));
  CHECK(r.model_limitations.find("N-0 state already curtails") != std::string::npos);
}

// ─────────────────────────────────────────────────────────────────────────
// Hybrid AC/DC customer metrics (Finding 3)
// ─────────────────────────────────────────────────────────────────────────

namespace {
// 1 AC bus (100 customers) + 1 DC bus (50 customers).
HybridPowerSystem make_ac_dc_customer_system() {
  HybridPowerSystem sys;

  ACBus a;
  a.index = 1;
  a.bus_type = BusType::SLACK;
  a.in_service = true;
  sys.ac.buses = {a};

  Load ld;
  ld.index = 1;
  ld.bus = 1;
  ld.in_service = true;
  ld.p_mw = 1.0;
  ld.n_customers = 100;
  sys.ac.loads = {ld};

  DCBus d;
  d.index = 101;
  d.in_service = true;
  d.is_load = true;
  d.pd_mw = 0.5;
  d.n_customers = 50;
  sys.dc.buses = {d};

  return sys;
}
}  // namespace

TEST_CASE("metrics: DC bus customers contribute to SAIFI/SAIDI", "[reliability][metrics][dc]") {
  HybridPowerSystem sys = make_ac_dc_customer_system();

  // Nodal layout is [AC bus | DC bus]; only the DC node is interrupted.
  std::vector<double> nodal_cif = {0.0, 2.0};   // 2 interruptions/yr at DC bus
  std::vector<double> nodal_cid = {0.0, 6.0};   // 6 hr/yr at DC bus

  DistributionIndices idx = compute_distribution_indices(sys, nodal_cif, nodal_cid, 8760);

  // total customers = 100 (AC) + 50 (DC) = 150
  // SAIFI = (2.0 * 50) / 150 ; SAIDI = (6.0 * 50) / 150
  CHECK(idx.saifi == Approx(100.0 / 150.0));
  CHECK(idx.saidi == Approx(300.0 / 150.0));
  CHECK(idx.caidi == Approx(idx.saidi / idx.saifi));
  CHECK(idx.asai == Approx(1.0 - idx.saidi / 8760.0));
}

TEST_CASE("metrics: AC-only caller is unaffected by DC presence (backward compat)",
          "[reliability][metrics][dc]") {
  HybridPowerSystem sys = make_ac_dc_customer_system();

  // AC-only nodal vectors (length == AC bus count) → DC bus is NOT weighted.
  std::vector<double> nodal_cif = {3.0};
  std::vector<double> nodal_cid = {12.0};

  DistributionIndices idx = compute_distribution_indices(sys, nodal_cif, nodal_cid, 8760);

  // total customers = 100 (AC only)
  CHECK(idx.saifi == Approx(3.0));
  CHECK(idx.saidi == Approx(12.0));
}

TEST_CASE("RL-02: ASAI is clamped to [0,1] when SAIDI exceeds a reporting year",
          "[reliability][metrics][regression]") {
  HybridPowerSystem sys = make_ac_dc_customer_system();
  std::vector<double> nodal_cif = {1.0};
  std::vector<double> nodal_cid = {20000.0};  // > 8760 hr/customer
  DistributionIndices idx =
      compute_distribution_indices(sys, nodal_cif, nodal_cid, 8760);
  CHECK(idx.asai >= 0.0);
  CHECK(idx.asai <= 1.0);
  CHECK(idx.asui >= 0.0);
  CHECK(idx.asui <= 1.0);
}

TEST_CASE("RL-01: compute_tail_risk tolerates a shorter LOLE series without OOB",
          "[reliability][tail_risk][regression]") {
  std::vector<double> eens(100);
  for (int i = 0; i < 100; ++i)
    eens[static_cast<size_t>(i)] = static_cast<double>(i);
  std::vector<double> lole(50, 1.0);  // deliberately shorter than eens
  const TailRiskMetrics m = compute_tail_risk(eens, lole, 0.95);
  CHECK(std::isfinite(m.eens_var));
  CHECK(std::isfinite(m.lole_var));
  CHECK(std::isfinite(m.eens_cvar));
  CHECK(std::isfinite(m.lole_cvar));
  CHECK(m.eens_percentiles.size() == 5);
  CHECK(m.lole_percentiles.size() == 5);
}

TEST_CASE("metrics: explicit DCLoad.n_customers weights DC interruptions",
          "[reliability][metrics][dc]") {
  HybridPowerSystem sys;

  ACBus a;
  a.index = 1;
  a.bus_type = BusType::SLACK;
  a.in_service = true;
  sys.ac.buses = {a};

  Load ld;
  ld.index = 1; ld.bus = 1; ld.in_service = true; ld.p_mw = 1.0; ld.n_customers = 100;
  sys.ac.loads = {ld};

  DCBus d;
  d.index = 101; d.in_service = true; d.is_load = true; d.pd_mw = 0.5;
  d.n_customers = 0;        // no bus-level count → must fall back to DC load count
  sys.dc.buses = {d};

  DCLoad dl;
  dl.index = 1; dl.bus = 101; dl.in_service = true; dl.p_mw = 0.5;
  dl.n_customers = 40;      // explicit DC load customers
  sys.dc.loads = {dl};

  std::vector<double> nodal_cif = {0.0, 1.0};
  std::vector<double> nodal_cid = {0.0, 3.0};

  DistributionIndices idx = compute_distribution_indices(sys, nodal_cif, nodal_cid, 8760);

  // total customers = 100 (AC) + 40 (DC load) = 140; only DC interrupted.
  CHECK(idx.saifi == Approx(1.0 * 40.0 / 140.0));
  CHECK(idx.saidi == Approx(3.0 * 40.0 / 140.0));
}

// ─────────────────────────────────────────────────────────────────────────
// Data-quality summary + opt-in policy through FMEA
// ─────────────────────────────────────────────────────────────────────────

TEST_CASE("data quality: summarize counts case vs missing", "[reliability][data_quality]") {
  HybridPowerSystem sys;

  ACBus a;
  a.index = 1;
  a.bus_type = BusType::SLACK;
  a.in_service = true;
  sys.ac.buses = {a};

  Generator g;          // has FOR + MTTR → case data
  g.index = 1; g.bus = 1; g.in_service = true;
  g.forced_outage_rate = 0.02; g.mttr_hr = 50.0;
  sys.ac.generators = {g};

  ACBranch br;          // no reliability fields → missing
  br.index = 1; br.from_bus = 1; br.to_bus = 1; br.in_service = true;
  sys.ac.branches = {br};

  ReliabilityDataQuality dq = summarize_reliability_data_quality(sys, ReliabilityDataPolicy{});

  CHECK(dq.components_total == 2);
  CHECK(dq.components_with_reliability_data == 1);   // the generator
  CHECK(dq.missing_required_data.size() == 1);       // the branch
}

TEST_CASE("FMEA: data-quality populated and strict policy flags missing data",
          "[reliability][fmea][data_quality]") {
  HybridPowerSystem sys;

  ACBus b1, b2;
  b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0; b1.in_service = true;
  b2.index = 2; b2.bus_type = BusType::PQ;    b2.pd_mw = 1.0; b2.in_service = true;
  sys.ac.buses = {b1, b2};

  Generator g;
  g.index = 1; g.bus = 1; g.in_service = true;
  g.pg_mw = 5.0; g.pmax_mw = 5.0; g.pmin_mw = 0.0;
  sys.ac.generators = {g};

  ACBranch br;   // NO failure_rate / mttr → reliability data missing
  br.index = 1; br.from_bus = 1; br.to_bus = 2; br.in_service = true;
  br.r_pu = 0.01; br.x_pu = 0.01;
  sys.ac.branches = {br};

  SECTION("default policy fills per-kind defaults") {
    FMEAOptions opts;  // default = UseNamedTemplateForMissingOnly
    auto result = run_distribution_fmea(sys, opts);
    CHECK(result.data_quality.components_total > 0);
    CHECK(result.data_quality.components_defaulted > 0);
    CHECK(result.data_quality.missing_required_data.empty());
  }

  SECTION("strict policy reports missing reliability data") {
    FMEAOptions opts;
    opts.data_policy.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;
    auto result = run_distribution_fmea(sys, opts);
    CHECK_FALSE(result.data_quality.missing_required_data.empty());
  }
}

TEST_CASE("FMEA: upstream protection trip precedes disconnector isolation",
          "[reliability][fmea][switch_sequence]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 10.0;
  ACBus source, load_bus;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  sys.ac.buses = {source, load_bus};
  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.pmax_mw = 5.0;
  generator.forced_outage_rate = 0.01;
  generator.mttr_hr = 2.0;
  sys.ac.generators = {generator};
  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 1.0;
  load.n_customers = 10;
  sys.ac.loads = {load};
  ACBranch feeder;
  feeder.index = 17;
  feeder.from_bus = 1;
  feeder.to_bus = 2;
  feeder.r_pu = 0.01;
  feeder.x_pu = 0.02;
  feeder.rate_a_mva = 5.0;
  feeder.failure_rate = 1.0;
  feeder.mttr_hr = 4.0;
  sys.ac.branches = {feeder};
  Switch breaker;
  breaker.index = 21;
  breaker.name = "CB-17";
  breaker.bus_from = 1;
  breaker.bus_to = 2;
  breaker.switch_type = SwitchType::CircuitBreaker;
  breaker.role = SwitchRole::Protection;
  breaker.capabilities_explicit = true;
  breaker.capabilities.can_interrupt_fault_current = true;
  breaker.capabilities.can_interrupt_load_current = true;
  breaker.t_open_s = 0.08;
  Switch disconnector;
  disconnector.index = 22;
  disconnector.name = "DS-17";
  disconnector.bus_from = 1;
  disconnector.bus_to = 2;
  disconnector.switch_type = SwitchType::Disconnector;
  disconnector.role = SwitchRole::Isolation;
  disconnector.controlled_element_type = "ac_branch";
  disconnector.controlled_element_index = 17;
  disconnector.controlled_branch_index = 17;
  disconnector.upstream_protective_switch_index = 21;
  disconnector.capabilities_explicit = true;
  disconnector.capabilities.requires_deenergized_operation = true;
  disconnector.t_open_s = 1.0;
  sys.ac.switches = {breaker, disconnector};

  FMEAOptions options;
  options.enable_parallel = false;
  const auto result = run_distribution_fmea(sys, options);
  const auto detail = std::find_if(
      result.contingencies.begin(), result.contingencies.end(),
      [](const auto& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(detail != result.contingencies.end());
  CHECK(detail->fault_isolation_explicit);
  CHECK(detail->repair_switch_sequence_valid);
  REQUIRE(detail->repair_switch_actions.size() == 2);
  const auto& trip = detail->repair_switch_actions[0];
  CHECK(trip.sequence_order == 1);
  CHECK(trip.switch_index == 21);
  CHECK(trip.action == "trip");
  CHECK(trip.validated);
  const auto& isolate = detail->repair_switch_actions[1];
  CHECK(isolate.sequence_order == 2);
  CHECK(isolate.switch_index == 22);
  CHECK(isolate.action == "open");
  CHECK(isolate.validated);
}

TEST_CASE("FMEA: Level-1 cyber conditioning preserves bounds and decomposition",
          "[reliability][fmea][cyber_physical]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 10.0;

  ACBus b1, b2;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.in_service = true;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.in_service = true;
  generator.is_slack = true;
  generator.pmax_mw = 10.0;
  generator.forced_outage_rate = 0.01;
  generator.mttr_hr = 2.0;
  sys.ac.generators = {generator};

  Load load;
  load.index = 1;
  load.bus = 2;
  load.in_service = true;
  load.p_mw = 1.0;
  load.n_customers = 100;
  sys.ac.loads = {load};

  ACBranch feeder;
  feeder.index = 1;
  feeder.from_bus = 1;
  feeder.to_bus = 2;
  feeder.in_service = true;
  feeder.r_pu = 0.01;
  feeder.x_pu = 0.1;
  feeder.rate_a_mva = 10.0;
  feeder.failure_rate = 1.0;
  feeder.mttr_hr = 4.0;
  ACBranch tie = feeder;
  tie.index = 2;
  tie.in_service = false;
  tie.failure_rate = 0.0;
  sys.ac.branches = {feeder, tie};

  FMEAOptions options;
  options.enable_parallel = false;
  options.enable_switch_reconfiguration = true;
  options.max_repair_switch_actions = 1;
  options.cyber_physical.enabled = true;
  options.cyber_physical.automation_availability = 0.75;
  options.cyber_physical.automatic_switching_time_hr = 0.05;
  options.cyber_physical.manual_switching_time_hr = 1.0;
  options.cyber_physical.freeze_der_on_automation_loss = false;

  const auto result = run_distribution_fmea(sys, options);
  REQUIRE(result.cyber_physical.enabled);
  CHECK(result.validity.restoration_duration_cyber_conditioned);
  CHECK_FALSE(result.validity.cyber_topology_modelled);
  CHECK_FALSE(result.validity.cyber_power_coupling_modelled);
  CHECK(result.cyber_physical.eens_perfect_cyber_mwh_yr <=
        result.eens_mwh_yr + 1e-9);
  CHECK(result.eens_mwh_yr <=
        result.cyber_physical.eens_no_automation_mwh_yr + 1e-9);
  CHECK(result.cyber_physical.automation_efficacy == Approx(0.75).margin(1e-8));
  CHECK(result.cyber_physical.information_service_availability ==
        Approx(0.75).margin(1e-8));
  CHECK_FALSE(result.cyber_physical.intelligent_enabled);

  const auto branch = std::find_if(
      result.contingencies.begin(), result.contingencies.end(),
      [](const FMEAContingencyDetail& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(branch != result.contingencies.end());
  const double branch_calendar_frequency = 8760.0 / (8760.0 + 4.0);
  CHECK(branch->failure_rate == Approx(branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_perfect_cyber_contribution ==
        Approx(0.05 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_no_automation_contribution ==
        Approx(1.0 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_cyber_duration_increment ==
        Approx(0.2375 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_cyber_control_increment == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_contribution ==
        Approx(0.2875 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->tau_sw_hr == Approx(0.2875).margin(1e-8));

  // Level-1+ three-dimensional screening multiplies the information service
  // probability by explicitly enabled intelligent-function probabilities.
  options.cyber_physical.intelligent.enabled = true;
  options.cyber_physical.intelligent.detection_success_probability = 0.8;
  options.cyber_physical.intelligent.isolation_success_probability = 1.0;
  options.cyber_physical.intelligent.restoration_decision_valid_probability = 1.0;
  options.cyber_physical.intelligent.restoration_execution_success_probability = 1.0;
  options.cyber_physical.intelligent.protection_success_probability = 1.0;
  const auto three_dimension = run_distribution_fmea(sys, options);
  CHECK(three_dimension.cyber_physical.intelligent_enabled);
  CHECK(three_dimension.cyber_physical.independent_factorization);
  CHECK(three_dimension.cyber_physical.intelligent_function_success_probability ==
        Approx(0.8).margin(1e-8));
  CHECK(three_dimension.cyber_physical.effective_automation_probability ==
        Approx(0.6).margin(1e-8));
  CHECK(three_dimension.cyber_physical.automation_efficacy ==
        Approx(0.6).margin(1e-8));
  CHECK(three_dimension.validity.information_service_conditioned);
  CHECK(three_dimension.validity.intelligent_function_probabilities_modelled);
  CHECK_FALSE(three_dimension.validity.joint_class_probability_modelled);
  CHECK_FALSE(three_dimension.validity.protection_logic_modelled);
  CHECK_FALSE(three_dimension.validity.protection_frt_reliability_coupled);

  // An explicit mutually-exclusive state table preserves dependence and
  // replaces the scalar product.  Here information is always available but
  // detection and every downstream function succeed together only 40% of time.
  using JointState = CyberPhysicalFMEAOptions::IntelligentFunctionOptions::
      JointFunctionState;
  JointState joint_success;
  joint_success.probability = 0.4;
  JointState joint_failure;
  joint_failure.probability = 0.6;
  joint_failure.detection_success = false;
  options.cyber_physical.intelligent.joint_states =
      {joint_success, joint_failure};
  const auto joint_dimension = run_distribution_fmea(sys, options);
  CHECK(joint_dimension.validity.joint_class_probability_modelled);
  CHECK_FALSE(joint_dimension.cyber_physical.independent_factorization);
  CHECK(joint_dimension.cyber_physical.information_service_availability ==
        Approx(1.0));
  CHECK(joint_dimension.cyber_physical.detection_success_probability ==
        Approx(0.4));
  CHECK(joint_dimension.cyber_physical.effective_automation_probability ==
        Approx(0.4));
  CHECK(joint_dimension.cyber_physical.automation_efficacy ==
        Approx(0.4).margin(1e-8));

  options.cyber_physical.intelligent.joint_states.back().probability = 0.5;
  CHECK_THROWS_AS(run_distribution_fmea(sys, options), std::invalid_argument);
  options.cyber_physical.intelligent.joint_states.clear();
  options.cyber_physical.intelligent.enabled = false;

  // Cyber conditioning parallelizes exactly like the physical path (no serial
  // guard) and the parallel run reproduces the serial numbers.
  options.enable_parallel = true;
  options.parallel_threads = 2;
  const auto parallel_run = run_distribution_fmea(sys, options);
  CHECK(parallel_run.parallel_effective);
  CHECK(parallel_run.parallel_mode == "parallel-fmea-contingencies");
  CHECK(parallel_run.eens_mwh_yr == Approx(result.eens_mwh_yr).margin(1e-9));

  options.cyber_physical.automation_availability = 1.0;
  const auto perfect_cyber = run_distribution_fmea(sys, options);
  const auto perfect_branch = std::find_if(
      perfect_cyber.contingencies.begin(), perfect_cyber.contingencies.end(),
      [](const FMEAContingencyDetail& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(perfect_branch != perfect_cyber.contingencies.end());
  CHECK(perfect_branch->eens_contribution ==
        Approx(0.05 * branch_calendar_frequency).margin(1e-12));
  CHECK(perfect_branch->eens_no_automation_contribution ==
        Approx(1.0 * branch_calendar_frequency).margin(1e-12));
  CHECK(perfect_cyber.cyber_physical.eens_no_automation_mwh_yr >=
        perfect_cyber.cyber_physical.eens_perfect_cyber_mwh_yr);

  options.cyber_physical.enabled = false;
  options.enable_parallel = false;
  options.switching_time_hr = 0.5;
  const auto physical = run_distribution_fmea(sys, options);
  const auto physical_branch = std::find_if(
      physical.contingencies.begin(), physical.contingencies.end(),
      [](const FMEAContingencyDetail& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(physical_branch != physical.contingencies.end());
  CHECK_FALSE(physical.cyber_physical.enabled);
  CHECK(physical_branch->eens_contribution ==
        Approx(0.5 * branch_calendar_frequency).margin(1e-12));
}

TEST_CASE("FMEA: Level-1 cyber conditioning attributes DER control loss",
          "[reliability][fmea][cyber_physical][control]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 10.0;

  ACBus b1, b2;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.in_service = true;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  Generator grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  grid.is_slack = true;
  grid.pmax_mw = 10.0;
  grid.forced_outage_rate = 0.0;
  sys.ac.generators = {grid};

  Storage backup;
  backup.index = 1;
  backup.bus = 2;
  backup.in_service = true;
  backup.controllable = true;
  backup.grid_forming = true;
  backup.pmax_mw = 1.0;
  backup.p_rated_mw = 1.0;
  backup.e_rated_mwh = 10.0;
  backup.e_mwh = 10.0;
  backup.soc_min = 0.0;
  backup.eta_discharge = 1.0;
  sys.ac.storage = {backup};

  Microgrid microgrid;
  microgrid.index = 1;
  microgrid.pcc_bus = 2;
  microgrid.internal_buses = {2};
  microgrid.in_service = true;
  microgrid.islanding_capability = true;
  sys.microgrids = {microgrid};

  Load load;
  load.index = 1;
  load.bus = 2;
  load.in_service = true;
  load.p_mw = 1.0;
  load.n_customers = 100;
  sys.ac.loads = {load};

  ACBranch feeder;
  feeder.index = 1;
  feeder.from_bus = 1;
  feeder.to_bus = 2;
  feeder.in_service = true;
  feeder.r_pu = 0.01;
  feeder.x_pu = 0.1;
  feeder.rate_a_mva = 10.0;
  feeder.failure_rate = 1.0;
  feeder.mttr_hr = 4.0;
  sys.ac.branches = {feeder};

  FMEAOptions options;
  options.enable_parallel = false;
  options.cyber_physical.enabled = true;
  options.cyber_physical.automation_availability = 0.5;
  options.cyber_physical.automatic_switching_time_hr = 0.1;
  options.cyber_physical.manual_switching_time_hr = 0.1;
  options.cyber_physical.freeze_der_on_automation_loss = true;

  const auto result = run_distribution_fmea(sys, options);
  const auto branch = std::find_if(
      result.contingencies.begin(), result.contingencies.end(),
      [](const FMEAContingencyDetail& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(branch != result.contingencies.end());
  const double branch_calendar_frequency = 8760.0 / (8760.0 + 4.0);
  CHECK(branch->failure_rate == Approx(branch_calendar_frequency).margin(1e-12));

  CHECK(result.validity.cyber_control_consequence_modelled);
  CHECK(branch->shed_rep_automatic_mw == Approx(0.0).margin(1e-8));
  CHECK(branch->shed_rep_manual_mw == Approx(1.0).margin(1e-8));
  CHECK(branch->eens_perfect_cyber_contribution == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_no_automation_contribution ==
        Approx(4.0 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_cyber_duration_increment == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_cyber_control_increment ==
        Approx(2.0 * branch_calendar_frequency).margin(1e-12));
  CHECK(branch->eens_contribution ==
        Approx(2.0 * branch_calendar_frequency).margin(1e-12));
}

TEST_CASE("built-in cyber-physical demo exposes automation value",
          "[reliability][fmea][cyber_physical][case_builder]") {
  const HybridPowerSystem sys =
      hacdcpf::io::build_cyber_physical_reliability_demo();

  REQUIRE(sys.ac.buses.size() == 3);
  REQUIRE(sys.ac.loads.size() == 2);
  REQUIRE(sys.ac.branches.size() == 3);
  REQUIRE(sys.ac.storage.size() == 1);
  CHECK_FALSE(sys.ac.branches[2].in_service);
  CHECK_FALSE(sys.ac.storage[0].grid_forming);

  FMEAOptions options;
  options.enable_parallel = false;
  options.enable_switch_reconfiguration = true;
  options.max_repair_switch_actions = 1;
  options.cyber_physical.enabled = true;
  options.cyber_physical.automation_availability = 0.75;
  options.cyber_physical.automatic_switching_time_hr = 0.05;
  options.cyber_physical.manual_switching_time_hr = 1.0;
  options.cyber_physical.freeze_der_on_automation_loss = true;

  const auto result = run_distribution_fmea(sys, options);
  const auto feeder = std::find_if(
      result.contingencies.begin(), result.contingencies.end(),
      [](const FMEAContingencyDetail& item) {
        return item.component_type == "ac_branch" && item.component_index == 0;
      });
  REQUIRE(feeder != result.contingencies.end());
  const double feeder_calendar_frequency = feeder->failure_rate;

  // Automatic class: FLISR closes the tie and dispatches the battery to cover
  // the 0.8 MW tie shortfall -> repair shed 0.  ens = 2 MW * 0.05 h = 0.1.
  CHECK(feeder->shed_rep_automatic_mw == Approx(0.0).margin(1e-8));
  CHECK(feeder->eens_perfect_cyber_contribution ==
        Approx(0.1 * feeder_calendar_frequency).margin(1e-12));
  // Manual class: the crew still closes the tie during the repair stage, but
  // the battery is frozen, so the 1.2 MVA tie limit leaves 0.8 MW shed.
  // ens = 2 MW * 1 h + 0.8 MW * 3 h = 4.4.
  CHECK(feeder->shed_rep_manual_mw == Approx(0.8).margin(1e-6));
  CHECK(feeder->eens_no_automation_contribution ==
        Approx(4.4 * feeder_calendar_frequency).margin(1e-12));
  // Increments at A = 0.75: duration 0.25*(2.0-0.1), control 0.25*(4.4-2.0).
  CHECK(feeder->eens_cyber_duration_increment ==
        Approx(0.475 * feeder_calendar_frequency).margin(1e-12));
  CHECK(feeder->eens_cyber_control_increment ==
        Approx(0.6 * feeder_calendar_frequency).margin(1e-12));
  CHECK(feeder->eens_contribution ==
        Approx(1.175 * feeder_calendar_frequency).margin(1e-12));
  CHECK(result.cyber_physical.delta_cyber_duration_mwh_yr > 0.0);
  CHECK(result.cyber_physical.delta_cyber_control_mwh_yr > 0.0);
  CHECK(result.cyber_physical.eens_perfect_cyber_mwh_yr < result.eens_mwh_yr);
  CHECK(result.eens_mwh_yr <
        result.cyber_physical.eens_no_automation_mwh_yr);

  auto misoperation_options = options;
  auto& misoperation =
      misoperation_options.cyber_physical.protection_misoperation;
  misoperation.enabled = true;
  misoperation.no_fault_decision_windows_per_year = 100.0;
  misoperation.false_trip_probability_per_window = 0.01;
  misoperation.trip_channel_success_probability = 0.5;
  misoperation.breaker_success_probability = 0.8;
  misoperation.disconnected_load_mw = 2.0;
  misoperation.restoration_duration_hr = 0.25;
  const auto with_misoperation =
      run_distribution_fmea(sys, misoperation_options);
  CHECK(with_misoperation.validity.protection_logic_modelled);
  CHECK(with_misoperation.validity.protection_misoperation_modelled);
  CHECK(with_misoperation.cyber_physical
            .protection_misoperation_frequency_per_year == Approx(0.4));
  CHECK(with_misoperation.cyber_physical
            .delta_protection_misoperation_mwh_yr == Approx(0.2));
  CHECK(with_misoperation.eens_mwh_yr ==
        Approx(result.eens_mwh_yr + 0.2).margin(1e-8));
  CHECK(with_misoperation.lole_hr_yr ==
        Approx(result.lole_hr_yr + 0.1).margin(1e-8));
  CHECK(with_misoperation.lolf_occ_yr ==
        Approx(result.lolf_occ_yr + 0.4).margin(1e-8));
  CHECK(with_misoperation.cyber_physical.eens_decomposition_residual_mwh_yr ==
        Approx(0.0).margin(1e-8));
}
