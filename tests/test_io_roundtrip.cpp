// Round-trip / IO conformance contract (§7).
#include <catch2/catch_test_macros.hpp>

#include <string>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/component_io_mapping.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/roundtrip.hpp"

TEST_CASE("JSON round-trip is lossless for a built-in case",
          "[io][roundtrip]") {
  const auto sys = hacdcpf::io::build_ieee14_acdc();
  const auto ev = hacdcpf::io::json_roundtrip(sys);
  INFO("adapter=" << ev.adapter << " checked=" << ev.fields_checked
                  << " mismatched=" << ev.fields_mismatched);
  if (!ev.mismatches.empty()) {
    INFO("first mismatch: " << ev.mismatches.front().path << " '"
                            << ev.mismatches.front().before << "' -> '"
                            << ev.mismatches.front().after << "'");
  }
  CHECK(ev.fields_checked > 0);
  CHECK(ev.passed);
}

TEST_CASE("diff_systems detects an injected mismatch", "[io][roundtrip]") {
  auto a = hacdcpf::io::build_ieee14_acdc();
  auto b = a;
  REQUIRE_FALSE(b.ac.branches.empty());
  b.ac.branches.front().x_pu += 1.0;  // perturb one field
  const auto ev = hacdcpf::io::diff_systems(a, b);
  CHECK_FALSE(ev.passed);
  CHECK(ev.fields_mismatched >= 1);
}

TEST_CASE("AC branch dynamic RL selector round-trips through JSON",
          "[io][roundtrip][dynamics]") {
  auto sys = hacdcpf::io::build_ieee14_acdc();
  REQUIRE_FALSE(sys.ac.branches.empty());
  sys.ac.branches.front().dynamic_rl = true;

  const auto restored = hacdcpf::io::from_json(hacdcpf::io::to_json(sys));
  REQUIRE_FALSE(restored.ac.branches.empty());
  CHECK(restored.ac.branches.front().dynamic_rl);
}

TEST_CASE("Round-trip evidence is stored and consistent with the F3 gate",
          "[io][roundtrip][maturity]") {
  const auto sys = hacdcpf::io::build_ieee14_acdc();
  const auto report = hacdcpf::io::analyze_digital_twin_readiness(sys);
  REQUIRE_FALSE(report.round_trip_evidence.empty());
  const auto& rt = report.round_trip_evidence.front();
  for (const auto& g : report.gates) {
    if (g.gate_id == "F3") {
      // F3 can only pass with a lossless stored round-trip (§7.2).
      if (g.passed) CHECK(rt.passed);
    }
  }
}

TEST_CASE("Schema version is stamped and gated by mode (§3.3)",
          "[io][schema]") {
  using namespace hacdcpf::io;
  const auto sys = build_ieee14_acdc();
  const std::string js = to_json(sys);
  CHECK(js.find("schema_version") != std::string::npos);
  CHECK(check_schema_version(js).has_value());  // self-consistent stamp

  // A document with an incompatible MAJOR version is rejected in Strict and
  // tolerated (best-effort) in Permissive (§3.3).
  const std::string bad =
      R"({"schema_version":"9.0","name":"x","base_mva":100.0})";
  CHECK_FALSE(check_schema_version(bad).has_value());
  CHECK_FALSE(try_from_json(bad, ImportMode::Strict).has_value());
  CHECK(try_from_json(bad, ImportMode::Permissive).has_value());
}

TEST_CASE("Telemetry section round-trips and drives the I1 gate (§10)",
          "[io][telemetry]") {
  using namespace hacdcpf;
  auto sys = io::build_ieee14_acdc();

  TelemetrySection tel;
  TelemetryStream st;
  st.stream_id = "scada-1";
  st.time_base.epoch_utc = "2026-07-03T00:00:00Z";
  TelemetryBinding b;
  b.component_ref = "ac.buses[1]";
  b.measurement_type = MeasurementType::Voltage;
  b.unit = "pu";
  b.source_system = "SCADA";
  b.tag = "BUS1.V";
  b.constrains_state = "vm_pu@bus1";
  st.bindings.push_back(b);
  tel.streams.push_back(st);
  StateSeed seed;
  seed.timestamp = "2026-07-03T00:00:00Z";
  seed.component_ref = "ac.buses[1]";
  seed.quantity = "vm_pu";
  seed.value = 1.02;
  tel.state_seeds.push_back(seed);
  sys.telemetry = tel;

  // Telemetry survives the JSON round-trip losslessly.
  const auto ev = io::json_roundtrip(sys);
  INFO("mismatched=" << ev.fields_mismatched);
  CHECK(ev.passed);

  const auto restored = io::from_json(io::to_json(sys));
  REQUIRE(restored.telemetry.has_value());
  CHECK(restored.telemetry->binding_count() == 1);
  REQUIRE(restored.telemetry->streams.size() == 1);
  CHECK(restored.telemetry->streams.front().bindings.front().tag == "BUS1.V");
  CHECK(restored.telemetry->state_seeds.size() == 1);

  // The I1 integration gate now reads real telemetry bindings, not names.
  const auto report = io::analyze_digital_twin_readiness(sys);
  for (const auto& g : report.gates) {
    if (g.gate_id == "I1") {
      CHECK(g.evidence.find("1 telemetry binding") != std::string::npos);
    }
  }
}
