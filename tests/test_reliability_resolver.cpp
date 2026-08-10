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
#include <set>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/reliability/failure_mode.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Approx;

namespace {
constexpr double kHoursPerYear = 8760.0;
}  // namespace

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
  CHECK(p.repair_hr == Approx(10.0));
  // U = lambda / (lambda + 8760/repair)
  CHECK(p.unavailability == Approx(2.0 / (2.0 + kHoursPerYear / 10.0)));
  CHECK(p.mttf_hr == Approx(kHoursPerYear / 2.0));
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
    CHECK(nuisance->mode.params.lambda_per_year ==
          Approx(1.25 + static_cast<double>(i)));
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
  REQUIRE_FALSE(r2.co_contingencies.empty());
  CHECK(r2.co_contingencies.front().total_shed_mw == Approx(8.0).margin(0.5));
  CHECK(r2.co_contingencies.front().eens_contribution > 0.0);
  CHECK(r2.co_contingencies.front().joint_unavailability > 0.0);
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
  CHECK(branch->eens_perfect_cyber_contribution == Approx(0.05).margin(1e-6));
  CHECK(branch->eens_no_automation_contribution == Approx(1.0).margin(1e-6));
  CHECK(branch->eens_cyber_duration_increment == Approx(0.2375).margin(1e-6));
  CHECK(branch->eens_cyber_control_increment == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_contribution == Approx(0.2875).margin(1e-6));
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
  CHECK(perfect_branch->eens_contribution == Approx(0.05).margin(1e-6));
  CHECK(perfect_branch->eens_no_automation_contribution ==
        Approx(1.0).margin(1e-6));
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
  CHECK(physical_branch->eens_contribution == Approx(0.5).margin(1e-6));
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

  CHECK(result.validity.cyber_control_consequence_modelled);
  CHECK(branch->shed_rep_automatic_mw == Approx(0.0).margin(1e-8));
  CHECK(branch->shed_rep_manual_mw == Approx(1.0).margin(1e-8));
  CHECK(branch->eens_perfect_cyber_contribution == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_no_automation_contribution == Approx(4.0).margin(1e-6));
  CHECK(branch->eens_cyber_duration_increment == Approx(0.0).margin(1e-8));
  CHECK(branch->eens_cyber_control_increment == Approx(2.0).margin(1e-6));
  CHECK(branch->eens_contribution == Approx(2.0).margin(1e-6));
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

  // Automatic class: FLISR closes the tie and dispatches the battery to cover
  // the 0.8 MW tie shortfall -> repair shed 0.  ens = 2 MW * 0.05 h = 0.1.
  CHECK(feeder->shed_rep_automatic_mw == Approx(0.0).margin(1e-8));
  CHECK(feeder->eens_perfect_cyber_contribution == Approx(0.1).margin(1e-6));
  // Manual class: the crew still closes the tie during the repair stage, but
  // the battery is frozen, so the 1.2 MVA tie limit leaves 0.8 MW shed.
  // ens = 2 MW * 1 h + 0.8 MW * 3 h = 4.4.
  CHECK(feeder->shed_rep_manual_mw == Approx(0.8).margin(1e-6));
  CHECK(feeder->eens_no_automation_contribution == Approx(4.4).margin(1e-6));
  // Increments at A = 0.75: duration 0.25*(2.0-0.1), control 0.25*(4.4-2.0).
  CHECK(feeder->eens_cyber_duration_increment == Approx(0.475).margin(1e-6));
  CHECK(feeder->eens_cyber_control_increment == Approx(0.6).margin(1e-6));
  CHECK(feeder->eens_contribution == Approx(1.175).margin(1e-6));
  CHECK(result.cyber_physical.delta_cyber_duration_mwh_yr > 0.0);
  CHECK(result.cyber_physical.delta_cyber_control_mwh_yr > 0.0);
  CHECK(result.cyber_physical.eens_perfect_cyber_mwh_yr < result.eens_mwh_yr);
  CHECK(result.eens_mwh_yr <
        result.cyber_physical.eens_no_automation_mwh_yr);
}
