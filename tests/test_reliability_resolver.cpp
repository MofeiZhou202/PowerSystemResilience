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

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

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
