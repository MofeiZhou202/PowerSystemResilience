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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
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
  CHECK(cov.components_total == 3);  // switch + breaker + vsc
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
  auto r = run_nonsequential_mc(sys, opt);

  CHECK(r.model_scope == "hybrid-acdc-network-lp");
  // Kirchhoff forces the 2:1 split, overloading the low-reactance branch and
  // shedding ~1 MW even in the healthy state (~1 MW * 8760 h).  A pure transport
  // LP would split freely and report ~0 EENS.
  CHECK(r.eens_mwh_yr > 1000.0);
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
