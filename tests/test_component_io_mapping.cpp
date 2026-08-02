#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/io/component_io_mapping.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/powersimulationsdynamics_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

using Catch::Matchers::ContainsSubstring;

namespace {

hacdcpf::HybridPowerSystem make_system_with_all_io_component_tables() {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "component_io_mapping_all_tables";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.dc.base_mva = 10.0;

  hacdcpf::ACBus ac1;
  ac1.index = 1;
  ac1.name = "ac_source";
  ac1.bus_type = hacdcpf::BusType::SLACK;
  ac1.base_kv = 12.47;
  sys.ac.buses.push_back(ac1);

  hacdcpf::ACBus ac2;
  ac2.index = 2;
  ac2.name = "ac_load";
  ac2.bus_type = hacdcpf::BusType::PQ;
  ac2.base_kv = 12.47;
  sys.ac.buses.push_back(ac2);

  hacdcpf::ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.03;
  br.name = "ac_line";
  sys.ac.branches.push_back(br);

  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  gen.name = "slack";
  sys.ac.generators.push_back(gen);

  hacdcpf::StaticGenerator sgen;
  sgen.index = 1;
  sgen.bus = 2;
  sgen.p_mw = 0.1;
  sgen.name = "static_gen";
  sys.ac.static_generators.push_back(sgen);

  hacdcpf::Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 0.4;
  load.q_mvar = 0.1;
  load.name = "load";
  sys.ac.loads.push_back(load);

  hacdcpf::FlexibleLoad flex;
  flex.index = 1;
  flex.bus = 2;
  flex.p_mw = 0.2;
  flex.name = "flex";
  sys.ac.flexible_loads.push_back(flex);

  hacdcpf::AsymmetricLoad asym;
  asym.index = 1;
  asym.bus = 2;
  asym.pa_mw = 0.01;
  asym.pb_mw = 0.02;
  asym.pc_mw = 0.03;
  asym.name = "asym";
  sys.ac.asymmetric_loads.push_back(asym);

  hacdcpf::Shunt shunt;
  shunt.index = 1;
  shunt.bus = 2;
  shunt.bs_mvar = 0.02;
  shunt.name = "shunt";
  sys.ac.shunts.push_back(shunt);

  hacdcpf::Storage storage;
  storage.index = 1;
  storage.bus = 2;
  storage.p_mw = 0.05;
  storage.name = "storage";
  sys.ac.storage.push_back(storage);

  hacdcpf::RenewableGen renewable;
  renewable.index = 1;
  renewable.bus = 2;
  renewable.p_mw = 0.06;
  renewable.name = "renewable";
  sys.ac.renewable_gens.push_back(renewable);

  hacdcpf::PVSystem pv;
  pv.index = 1;
  pv.bus = 2;
  pv.p_mw = 0.07;
  pv.name = "pv";
  sys.ac.pv_systems.push_back(pv);

  hacdcpf::ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.name = "external_grid";
  sys.ac.external_grids.push_back(eg);

  hacdcpf::Transformer2W tr2;
  tr2.index = 1;
  tr2.hv_bus = 1;
  tr2.lv_bus = 2;
  tr2.sn_mva = 1.0;
  tr2.vk_percent = 5.0;
  tr2.name = "tr2";
  sys.ac.transformers_2w.push_back(tr2);

  hacdcpf::Transformer3W tr3;
  tr3.index = 1;
  tr3.hv_bus = 1;
  tr3.mv_bus = 2;
  tr3.lv_bus = 2;
  tr3.sn_hv_mva = 1.0;
  tr3.sn_mv_mva = 1.0;
  tr3.sn_lv_mva = 1.0;
  tr3.name = "tr3";
  sys.ac.transformers_3w.push_back(tr3);

  hacdcpf::RegulatorControl reg;
  reg.index = 1;
  reg.name = "reg";
  reg.transformer_index = 1;
  reg.monitored_bus = 2;
  reg.vreg_volts = 120.0;
  reg.band_volts = 2.0;
  reg.enabled = true;
  sys.ac.regulator_controls.push_back(reg);

  hacdcpf::Switch sw;
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sw.name = "switch";
  sys.ac.switches.push_back(sw);

  hacdcpf::CircuitBreaker cb;
  cb.index = 1;
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.name = "breaker";
  sys.ac.circuit_breakers.push_back(cb);

  hacdcpf::ChargingStation station;
  station.index = 1;
  station.bus = 2;
  station.p_total_kw = 50.0;
  station.name = "station";
  sys.ac.charging_stations.push_back(station);

  hacdcpf::Charger charger;
  charger.index = 1;
  charger.station_id = 1;
  charger.p_ch_max_kw = 7.0;
  charger.name = "charger";
  sys.ac.chargers.push_back(charger);

  hacdcpf::AsynchronousMotor motor;
  motor.index = 1;
  motor.bus = 2;
  motor.sn_mva = 0.1;
  motor.name = "motor";
  sys.ac.motors.push_back(motor);

  hacdcpf::DCBus dc1;
  dc1.index = 1;
  dc1.bus_type = hacdcpf::DCBusType::DC_V;
  dc1.base_kv = 0.75;
  sys.dc.buses.push_back(dc1);

  hacdcpf::DCBus dc2;
  dc2.index = 2;
  dc2.bus_type = hacdcpf::DCBusType::DC_P;
  dc2.base_kv = 0.75;
  sys.dc.buses.push_back(dc2);

  hacdcpf::DCBranch dcbr;
  dcbr.index = 1;
  dcbr.from_bus = 1;
  dcbr.to_bus = 2;
  dcbr.r_pu = 0.02;
  sys.dc.branches.push_back(dcbr);

  hacdcpf::DCLoad dcload;
  dcload.index = 1;
  dcload.bus = 2;
  dcload.p_mw = 0.1;
  sys.dc.loads.push_back(dcload);

  hacdcpf::Storage dc_storage_legacy;
  dc_storage_legacy.index = 1;
  dc_storage_legacy.bus = 2;
  dc_storage_legacy.p_mw = 0.03;
  sys.dc.storage.push_back(dc_storage_legacy);

  hacdcpf::DCStorage dc_storage;
  dc_storage.index = 1;
  dc_storage.bus = 2;
  dc_storage.p_mw = 0.03;
  sys.dc.dc_storage.push_back(dc_storage);

  hacdcpf::StaticGenerator dc_sgen_legacy;
  dc_sgen_legacy.index = 1;
  dc_sgen_legacy.bus = 2;
  dc_sgen_legacy.p_mw = 0.04;
  sys.dc.static_generators.push_back(dc_sgen_legacy);

  hacdcpf::StaticGeneratorDC dc_sgen;
  dc_sgen.index = 1;
  dc_sgen.bus = 2;
  dc_sgen.p_set_mw = 0.04;
  sys.dc.dc_static_generators.push_back(dc_sgen);

  hacdcpf::PVArrayDC pvdc;
  pvdc.index = 1;
  pvdc.bus = 2;
  pvdc.p_set_mw = 0.05;
  sys.dc.pv_arrays.push_back(pvdc);

  hacdcpf::DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 1;
  dcdc.bus_out = 2;
  sys.dc.dcdc_converters.push_back(dcdc);

  hacdcpf::DCCircuitBreaker dccb;
  dccb.index = 1;
  dccb.bus_from = 1;
  dccb.bus_to = 2;
  sys.dc.dc_circuit_breakers.push_back(dccb);

  hacdcpf::VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.bus_dc = 1;
  vsc.p_set_mw = 0.1;
  sys.vsc_converters.push_back(vsc);

  hacdcpf::EnergyRouter er;
  er.index = 1;
  er.name = "er";
  er.num_ports = 1;
  hacdcpf::EnergyRouterPort port;
  port.index = 1;
  port.bus = 2;
  port.port_type = hacdcpf::ERPortType::AC;
  er.ports.push_back(port);
  sys.energy_routers.push_back(er);

  hacdcpf::MobileStorage mobile;
  mobile.index = 1;
  mobile.bus = 2;
  mobile.p_mw = 0.02;
  sys.mobile_storage.push_back(mobile);

  hacdcpf::VirtualPowerPlant vpp;
  vpp.index = 1;
  vpp.pcc_bus = 2;
  vpp.p_output_mw = 0.03;
  sys.vpps.push_back(vpp);

  hacdcpf::Microgrid microgrid;
  microgrid.index = 1;
  microgrid.pcc_bus = 2;
  microgrid.p_exchange_mw = 0.04;
  sys.microgrids.push_back(microgrid);

  hacdcpf::ThreePhaseACSystem tp;
  tp.base_mva = 10.0;
  hacdcpf::ThreePhaseACBus tpb;
  tpb.index = 1;
  tpb.bus_type = hacdcpf::BusType::SLACK;
  tp.buses.push_back(tpb);
  hacdcpf::ThreePhaseACLine tpl;
  tpl.index = 1;
  tpl.from_bus = 1;
  tpl.to_bus = 1;
  tp.lines.push_back(tpl);
  hacdcpf::ThreePhaseTransformer tpt;
  tpt.index = 1;
  tpt.hv_bus = 1;
  tpt.lv_bus = 1;
  tp.transformers.push_back(tpt);
  hacdcpf::ThreePhaseLoad tpld;
  tpld.index = 1;
  tpld.bus = 1;
  tpld.p_a_mw = 0.01;
  tp.loads.push_back(tpld);
  hacdcpf::ThreePhaseGenerator tpg;
  tpg.index = 1;
  tpg.bus = 1;
  tpg.is_slack = true;
  tp.generators.push_back(tpg);
  hacdcpf::ThreePhaseExternalGrid tpeg;
  tpeg.index = 1;
  tpeg.bus = 1;
  tp.external_grids.push_back(tpeg);
  hacdcpf::ThreePhaseRegulatorControl tpreg;
  tpreg.index = 1;
  tpreg.transformer_index = 1;
  tpreg.vreg_volts = 120.0;
  tp.regulator_controls.push_back(tpreg);
  sys.three_phase_ac = tp;

  return sys;
}

}  // namespace

TEST_CASE("Component IO registry covers every rich component collection",
          "[io][mapping][coverage]") {
  const auto& mappings = hacdcpf::io::component_io_mappings();
  REQUIRE(mappings.size() >= 40);

  std::set<std::string> paths;
  for (const auto& mapping : mappings) {
    CHECK_FALSE(mapping.component_type.empty());
    CHECK_FALSE(mapping.collection_path.empty());
    CHECK(paths.insert(mapping.collection_path).second);
    CHECK(hacdcpf::io::policy_for_format(
              mapping, hacdcpf::io::ComponentIOFormat::InternalJSON) ==
          hacdcpf::io::ComponentIOPolicy::Exact);
  }

  CHECK(hacdcpf::io::find_component_io_mapping("ACBus").has_value());
  CHECK(hacdcpf::io::find_component_io_mapping("VSCConverter").has_value());
  CHECK(hacdcpf::io::find_component_io_mapping("NoSuchComponent").has_value() ==
        false);

  const auto vsc = hacdcpf::io::find_component_io_mapping("VSCConverter");
  REQUIRE(vsc.has_value());
  CHECK(std::any_of(vsc->standard_profiles.begin(),
                    vsc->standard_profiles.end(),
                    [](const hacdcpf::io::ComponentStandardProfile& profile) {
                      return profile.family == hacdcpf::io::ComponentStandardFamily::NERC &&
                             profile.model_name.find("REGC") != std::string::npos;
                    }));
  const auto gen = hacdcpf::io::find_component_io_mapping("Generator");
  REQUIRE(gen.has_value());
  CHECK(std::any_of(gen->standard_profiles.begin(),
                    gen->standard_profiles.end(),
                    [](const hacdcpf::io::ComponentStandardProfile& profile) {
                      return profile.family ==
                             hacdcpf::io::ComponentStandardFamily::IEEE4215;
                    }));
  CHECK(hacdcpf::io::to_string(
            hacdcpf::io::ComponentStandardFamily::IEC61970CIM) ==
        "IEC61970CIM");
  CHECK(hacdcpf::io::to_string(
            hacdcpf::io::ComponentStandardFamily::
                PowerSimulationsDynamics) ==
        "PowerSimulationsDynamics.jl");
}

TEST_CASE("Component IO coverage report classifies populated systems",
          "[io][mapping][coverage][all_components]") {
  const auto sys = make_system_with_all_io_component_tables();
  const auto report = hacdcpf::io::analyze_component_io_coverage(sys);

  CHECK(report.total_instances() > 40);
  CHECK(report.unrepresented_instances(
            hacdcpf::io::ComponentIOFormat::InternalJSON) == 0);
  CHECK(report.unrepresented_instances(
            hacdcpf::io::ComponentIOFormat::CanonicalModel) == 0);
  CHECK(report.unrepresented_instances(
            hacdcpf::io::ComponentIOFormat::GridLABD) > 0);
  CHECK(report.unrepresented_instances(
            hacdcpf::io::ComponentIOFormat::OpenDSS) > 0);
  CHECK(report.unrepresented_instances(
            hacdcpf::io::ComponentIOFormat::
                PowerSimulationsDynamicsJulia) > 0);

  const auto gridlabd_diagnostics = hacdcpf::io::external_io_diagnostics(
      report, hacdcpf::io::ComponentIOFormat::GridLABD);
  const auto opendss_diagnostics = hacdcpf::io::external_io_diagnostics(
      report, hacdcpf::io::ComponentIOFormat::OpenDSS);
  const auto psd_diagnostics = hacdcpf::io::external_io_diagnostics(
      report,
      hacdcpf::io::ComponentIOFormat::PowerSimulationsDynamicsJulia);

  REQUIRE_FALSE(gridlabd_diagnostics.empty());
  REQUIRE_FALSE(opendss_diagnostics.empty());
  REQUIRE_FALSE(psd_diagnostics.empty());
  CHECK_THAT(gridlabd_diagnostics.front(), ContainsSubstring("policy is"));
  CHECK(std::any_of(gridlabd_diagnostics.begin(), gridlabd_diagnostics.end(),
                    [](const std::string& line) {
                      return line.find("dc.buses") != std::string::npos;
                    }));
  CHECK(std::any_of(opendss_diagnostics.begin(), opendss_diagnostics.end(),
                    [](const std::string& line) {
                      return line.find("vsc_converters") != std::string::npos;
                    }));
  CHECK(std::any_of(psd_diagnostics.begin(), psd_diagnostics.end(),
                    [](const std::string& line) {
                      return line.find("dc.buses") != std::string::npos;
                    }));
}

TEST_CASE("Digital twin conversion capabilities summarize conversion risk",
          "[io][mapping][digital_twin][conversion]") {
  const auto sys = make_system_with_all_io_component_tables();
  const auto capabilities =
      hacdcpf::io::analyze_digital_twin_conversion_capabilities(sys);
  REQUIRE(capabilities.size() == 5);

  auto find_capability =
      [&](hacdcpf::io::ComponentIOFormat format)
          -> const hacdcpf::io::DigitalTwinConversionCapability* {
    for (const auto& cap : capabilities) {
      if (cap.format == format) return &cap;
    }
    return nullptr;
  };

  const auto* json = find_capability(
      hacdcpf::io::ComponentIOFormat::InternalJSON);
  const auto* psd = find_capability(
      hacdcpf::io::ComponentIOFormat::PowerSimulationsDynamicsJulia);
  const auto* gridlabd = find_capability(
      hacdcpf::io::ComponentIOFormat::GridLABD);

  REQUIRE(json != nullptr);
  REQUIRE(psd != nullptr);
  REQUIRE(gridlabd != nullptr);

  CHECK(json->coverage_ratio > 0.99);
  CHECK(json->round_trip_available);
  CHECK(json->twin_path_safe);
  CHECK(json->risk_level == "low");

  CHECK(psd->binding_level == "dynamic-profile");
  CHECK(psd->represented_instances > 0);
  CHECK(psd->unrepresented_instances > 0);
  CHECK_FALSE(psd->risks.empty());
  CHECK(psd->risk_score > json->risk_score);

  CHECK(gridlabd->unsupported_instances > 0);
  CHECK_FALSE(gridlabd->blocking_collections.empty());
  CHECK_FALSE(gridlabd->next_actions.empty());
}

TEST_CASE("Digital twin evidence ledger stores adapter validation evidence",
          "[io][mapping][digital_twin][ledger]") {
  const auto sys = make_system_with_all_io_component_tables();
  const auto ledger = hacdcpf::io::analyze_digital_twin_evidence_ledger(sys);

  REQUIRE(ledger.model_name == sys.name);
  REQUIRE_FALSE(ledger.generated_at_utc.empty());
  REQUIRE(ledger.entries.size() == 5);

  auto find_entry =
      [&](hacdcpf::io::ComponentIOFormat format)
          -> const hacdcpf::io::DigitalTwinEvidenceLedgerEntry* {
    for (const auto& entry : ledger.entries) {
      if (entry.conversion_target == format) return &entry;
    }
    return nullptr;
  };

  const auto* json = find_entry(
      hacdcpf::io::ComponentIOFormat::InternalJSON);
  const auto* psd = find_entry(
      hacdcpf::io::ComponentIOFormat::PowerSimulationsDynamicsJulia);

  REQUIRE(json != nullptr);
  REQUIRE(psd != nullptr);

  CHECK(json->model_name == sys.name);
  CHECK(json->validation_method.find("round-trip") != std::string::npos);
  CHECK(json->passed);
  CHECK(json->residual == 0.0);
  CHECK(json->tolerance > 0.0);
  CHECK(json->confidence_score > 0.80);
  CHECK(json->timestamp_utc == ledger.generated_at_utc);

  CHECK(psd->adapter == "PowerSimulationsDynamics.jl");
  CHECK_FALSE(psd->passed);
  CHECK(psd->residual > psd->tolerance);
  CHECK_FALSE(psd->blocking_collections.empty());
  CHECK((psd->risk_level == "medium" || psd->risk_level == "high"));
  CHECK_THAT(psd->evidence_summary, ContainsSubstring("coverage"));
}

TEST_CASE("PowerSimulationsDynamics Julia IO exports a neutral snapshot",
          "[io][mapping][psd][export]") {
  auto sys = make_system_with_all_io_component_tables();
  REQUIRE_FALSE(sys.ac.generators.empty());
  sys.ac.generators.front().dynamic_model.standard =
      "PowerSimulationsDynamics";
  sys.ac.generators.front().dynamic_model.model_name = "GENROU";
  sys.ac.generators.front().dynamic_model.parameter_set = "unit-test";
  sys.ac.generators.front().dynamic_model.parameters["H"] = 3.2;
  sys.ac.generators.front().dynamic_model.components.push_back(
      {"machine", "RoundRotorQuadratic", "PowerSimulationsDynamics", "",
       {{"Td0_p", 8.0}}});

  REQUIRE_FALSE(sys.vsc_converters.empty());
  sys.vsc_converters.front().dynamic_model.standard =
      "PowerSimulationsDynamics";
  sys.vsc_converters.front().dynamic_model.model_name =
      "REGC_REEC_GFL_Subset";
  sys.vsc_converters.front().dynamic_model.components.push_back(
      {"frequency_estimator", "ReducedOrderPLL", "PowerSimulationsDynamics",
       "", {{"kp_pll", 0.1}}});

  const auto text = hacdcpf::io::to_powersimulationsdynamics_json(sys);
  const auto doc = nlohmann::json::parse(text);

  CHECK(doc.at("format") == "hacdcpf_psd_snapshot.v1");
  CHECK(doc.at("source").at("kind") == "hacdcpf_rich_model");
  REQUIRE(doc.at("components").contains("dynamic_injections"));
  const auto& dynamic = doc.at("components").at("dynamic_injections");
  REQUIRE(dynamic.size() >= 2);
  CHECK(doc.at("system")
            .at("component_counts")
            .at("dynamic_injections")
            .get<std::size_t>() == dynamic.size());

  const auto has_genrou = std::any_of(
      dynamic.begin(), dynamic.end(), [](const nlohmann::json& row) {
        return row.value("type", "") == "Generator" &&
               row.at("fields")
                       .at("dynamic_model")
                       .value("model_name", "") == "GENROU" &&
               row.contains("slots") && !row.at("slots").empty();
      });
  CHECK(has_genrou);

  const auto notes = doc.at("conversion_notes").dump();
  CHECK(notes.find("PowerSimulationsDynamics.jl") != std::string::npos);

  const auto julia = hacdcpf::io::to_powersimulationsdynamics_julia(sys);
  CHECK_THAT(julia, ContainsSubstring("HACDCPF_PSD_SNAPSHOT_JSON"));
  CHECK_THAT(julia, ContainsSubstring("HACDCPF_RICH_MODEL_JSON"));
  CHECK_THAT(julia, ContainsSubstring("hacdcpf_psd_julia.v1"));
  const auto restored =
      hacdcpf::io::from_powersimulationsdynamics_julia(julia);
  CHECK(restored.name == sys.name);
  CHECK(restored.ac.generators.size() == sys.ac.generators.size());
  CHECK(restored.vsc_converters.size() == sys.vsc_converters.size());

  REQUIRE(doc.at("components").contains("diagnostic_components"));
  const auto& diagnostic = doc.at("components").at("diagnostic_components");
  const auto has_dc_bus = std::any_of(
      diagnostic.begin(), diagnostic.end(), [](const nlohmann::json& row) {
        return row.value("type", "") == "DCBus";
      });
  const auto has_three_phase_bus = std::any_of(
      diagnostic.begin(), diagnostic.end(), [](const nlohmann::json& row) {
        return row.value("type", "") == "ThreePhaseACBus";
      });
  CHECK(has_dc_bus);
  CHECK(has_three_phase_bus);
}

TEST_CASE("Regulator controls are preserved by internal JSON",
          "[io][json][mapping][roundtrip]") {
  const auto sys = make_system_with_all_io_component_tables();
  const auto restored = hacdcpf::io::from_json(hacdcpf::io::to_json(sys));

  REQUIRE(restored.ac.regulator_controls.size() == 1);
  CHECK(restored.ac.regulator_controls.front().name == "reg");
  CHECK(restored.ac.regulator_controls.front().transformer_index == 1);
  CHECK(restored.ac.regulator_controls.front().monitored_bus == 2);
  CHECK(restored.ac.regulator_controls.front().enabled);

  REQUIRE(restored.three_phase_ac.has_value());
  REQUIRE(restored.three_phase_ac->regulator_controls.size() == 1);
  CHECK(restored.three_phase_ac->regulator_controls.front().transformer_index ==
        1);
  CHECK(restored.three_phase_ac->regulator_controls.front().vreg_volts ==
        120.0);
}

TEST_CASE("Dynamic profiles survive rich-to-canonical projection",
          "[io][mapping][dynamic][canonical]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 10.0;

  hacdcpf::ACBus ac1;
  ac1.index = 1;
  ac1.bus_type = hacdcpf::BusType::SLACK;
  hacdcpf::ACBus ac2;
  ac2.index = 2;
  ac2.bus_type = hacdcpf::BusType::PQ;
  sys.ac.buses = {ac1, ac2};

  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  sys.ac.generators.push_back(gen);

  hacdcpf::DynamicModelProfile vpp_profile;
  vpp_profile.standard = "NERC";
  vpp_profile.model_name = "DER_Aggregate";
  vpp_profile.parameters["fleet_droop"] = 0.04;
  hacdcpf::VirtualPowerPlant vpp;
  vpp.index = 12;
  vpp.pcc_bus = 2;
  vpp.p_output_mw = 0.3;
  vpp.dynamic_model = vpp_profile;
  sys.vpps.push_back(vpp);

  hacdcpf::DynamicModelProfile er_profile;
  er_profile.standard = "IEC";
  er_profile.model_name = "EnergyRouterAverageModel";
  er_profile.parameters["dc_link_capacitance_s"] = 0.15;
  hacdcpf::EnergyRouter er;
  er.index = 4;
  er.name = "er";
  er.in_service = true;
  er.p_rated_mw = 1.0;
  er.vn_dc_kv = 0.75;
  er.dynamic_model = er_profile;

  hacdcpf::EnergyRouterPort port;
  port.index = 1;
  port.bus = 2;
  port.side = 0;
  port.port_type = hacdcpf::ERPortType::AC;
  port.control_mode = hacdcpf::ERControlMode::PQ;
  port.p_set_mw = 0.1;
  port.dynamic_model.standard = "NERC";
  port.dynamic_model.model_name = "REGC_A";
  port.dynamic_model.components.push_back(
      {"pll", "ReducedOrderPLL", "PSD", "", {{"kp_pll", 0.02}}});
  er.ports.push_back(port);

  hacdcpf::EnergyRouterPort peer_port;
  peer_port.index = 2;
  peer_port.bus = 2;
  peer_port.side = 1;
  peer_port.port_type = hacdcpf::ERPortType::AC;
  peer_port.control_mode = hacdcpf::ERControlMode::PQ;
  peer_port.p_set_mw = -0.1;
  er.ports.push_back(peer_port);
  sys.energy_routers.push_back(er);

  const auto projected = hacdcpf::project_to_canonical_models(sys, false);

  REQUIRE_FALSE(projected.ac.static_generators.empty());
  const auto vpp_eq = std::find_if(
      projected.ac.static_generators.begin(),
      projected.ac.static_generators.end(),
      [](const hacdcpf::StaticGenerator& sg) {
        return sg.dynamic_model.source_id == "VirtualPowerPlant:12";
      });
  REQUIRE(vpp_eq != projected.ac.static_generators.end());
  CHECK(vpp_eq->dynamic_model.model_name == "DER_Aggregate");
  CHECK(std::abs(vpp_eq->dynamic_model.parameters.at("fleet_droop") - 0.04) <=
        1e-12);

  REQUIRE_FALSE(projected.vsc_converters.empty());
  const auto er_vsc = std::find_if(
      projected.vsc_converters.begin(),
      projected.vsc_converters.end(),
      [](const hacdcpf::VSCConverter& c) {
        return c.dynamic_model.source_id == "EnergyRouterPort:4:1";
      });
  REQUIRE(er_vsc != projected.vsc_converters.end());
  CHECK(er_vsc->dynamic_model.model_name == "REGC_A");
  REQUIRE(er_vsc->dynamic_model.components.size() == 1);
  CHECK(er_vsc->dynamic_model.components.front().model == "ReducedOrderPLL");

  REQUIRE_FALSE(projected.dc.dcdc_converters.empty());
  const auto er_dcdc = std::find_if(
      projected.dc.dcdc_converters.begin(),
      projected.dc.dcdc_converters.end(),
      [](const hacdcpf::DCDCConverter& c) {
        return c.dynamic_model.source_id == "EnergyRouter:4";
      });
  REQUIRE(er_dcdc != projected.dc.dcdc_converters.end());
  CHECK(er_dcdc->dynamic_model.model_name == "EnergyRouterAverageModel");
  CHECK(std::abs(er_dcdc->dynamic_model.parameters.at("dc_link_capacitance_s") -
                 0.15) <= 1e-12);
}

TEST_CASE("Component parameter rule catalog spans standards and categories",
          "[io][mapping][parameters][standards]") {
  const auto& rules = hacdcpf::io::component_parameter_rules();
  REQUIRE(rules.size() >= 120);

  auto has_rule = [&](const std::string& component,
                      const std::string& parameter,
                      hacdcpf::io::ComponentParameterCategory category) {
    return std::any_of(
        rules.begin(), rules.end(),
        [&](const hacdcpf::io::ComponentParameterRule& rule) {
          return rule.component_type == component &&
                 rule.parameter_path == parameter &&
                 rule.category == category;
        });
  };

  CHECK(has_rule("ACBus", "base_kv",
                 hacdcpf::io::ComponentParameterCategory::Static));
  CHECK(has_rule("Generator", "dynamic_model",
                 hacdcpf::io::ComponentParameterCategory::Dynamic));
  CHECK(has_rule("VSCConverter", "i_max_pu",
                 hacdcpf::io::ComponentParameterCategory::Transient));
  CHECK(has_rule("Storage", "forced_outage_rate",
                 hacdcpf::io::ComponentParameterCategory::Failure));
  CHECK(has_rule("ACBranch", "mttr_hr",
                 hacdcpf::io::ComponentParameterCategory::Reliability));

  CHECK(std::any_of(
      rules.begin(), rules.end(),
      [](const hacdcpf::io::ComponentParameterRule& rule) {
        return rule.standard_family ==
               hacdcpf::io::ComponentStandardFamily::IEEE1547;
      }));
  CHECK(std::any_of(
      rules.begin(), rules.end(),
      [](const hacdcpf::io::ComponentParameterRule& rule) {
        return rule.standard_family ==
               hacdcpf::io::ComponentStandardFamily::IEC61970CIM;
      }));
  CHECK(hacdcpf::io::to_string(
            hacdcpf::io::ComponentParameterSeverity::Warning) == "Warning");
  CHECK(hacdcpf::io::to_string(
            hacdcpf::io::ComponentParameterCategory::Transient) ==
        "Transient");
}

TEST_CASE("Component parameter audit flags invalid rich-model values",
          "[io][mapping][parameters][audit]") {
  hacdcpf::HybridPowerSystem sys;

  hacdcpf::ACBus bus;
  bus.index = 1;
  bus.name = "bad_bus";
  bus.base_kv = -12.47;
  bus.vm_pu = 1.8;
  bus.vmin_pu = 1.2;
  bus.vmax_pu = 0.8;
  sys.ac.buses.push_back(bus);

  hacdcpf::ACBranch branch;
  branch.index = 1;
  branch.name = "bad_branch";
  branch.r_pu = -0.01;
  branch.x_pu = 0.02;
  branch.failure_rate = -1.0;
  sys.ac.branches.push_back(branch);

  hacdcpf::Transformer2W transformer;
  transformer.index = 1;
  transformer.name = "bad_transformer";
  transformer.sn_mva = 0.0;
  transformer.vn_hv_kv = 12.47;
  transformer.vn_lv_kv = 0.48;
  transformer.vk_percent = 4.0;
  transformer.vkr_percent = 8.0;
  sys.ac.transformers_2w.push_back(transformer);

  hacdcpf::Generator generator;
  generator.index = 1;
  generator.name = "bad_generator";
  generator.vg_pu = 1.0;
  generator.pmin_mw = 20.0;
  generator.pmax_mw = 10.0;
  generator.dynamic_model.standard = "IEEE";
  generator.dynamic_model.model_name = "GENROU";
  generator.dynamic_model.parameters["H"] = 0.01;
  generator.dynamic_model.parameters["pll_kp"] = 200.0;
  sys.ac.generators.push_back(generator);

  hacdcpf::Storage storage;
  storage.index = 1;
  storage.name = "bad_storage";
  storage.soc_min = 0.2;
  storage.soc_init = 1.2;
  storage.soc_max = 0.9;
  storage.eta_charge = 1.1;
  storage.forced_outage_rate = 1.2;
  sys.ac.storage.push_back(storage);

  hacdcpf::VSCConverter vsc;
  vsc.index = 1;
  vsc.name = "bad_vsc";
  vsc.eta = 1.2;
  vsc.i_max_pu = 5.0;
  vsc.pmin_mw = 5.0;
  vsc.pmax_mw = -1.0;
  vsc.dynamic_model.components.push_back(
      {"pll", "", "", "", {{"ki_pll", -0.1}}});
  sys.vsc_converters.push_back(vsc);

  const auto report =
      hacdcpf::io::analyze_component_parameter_quality(sys);

  CHECK(report.component_instances_checked >= 6);
  CHECK(report.checked_parameters > 20);
  CHECK(report.count(hacdcpf::io::ComponentParameterSeverity::Error) >= 4);
  CHECK(report.count(hacdcpf::io::ComponentParameterSeverity::Warning) >= 6);
  CHECK(report.count(hacdcpf::io::ComponentParameterCategory::Static) > 0);
  CHECK(report.count(hacdcpf::io::ComponentParameterCategory::Dynamic) > 0);
  CHECK(report.count(hacdcpf::io::ComponentParameterCategory::Transient) > 0);
  CHECK(report.count(hacdcpf::io::ComponentParameterCategory::Failure) > 0);

  auto has_finding = [&](const std::string& component,
                         const std::string& parameter_substring) {
    return std::any_of(
        report.findings.begin(), report.findings.end(),
        [&](const hacdcpf::io::ComponentParameterFinding& finding) {
          return finding.component_type == component &&
                 finding.parameter_path.find(parameter_substring) !=
                     std::string::npos;
        });
  };

  CHECK(has_finding("ACBus", "base_kv"));
  CHECK(has_finding("ACBus", "vmin_pu <= vmax_pu"));
  CHECK(has_finding("Transformer2W", "vkr_percent <= vk_percent"));
  CHECK(has_finding("Generator", "dynamic_model.parameters.H"));
  CHECK(has_finding("Storage", "soc_init <= soc_max"));
  CHECK(has_finding("Storage", "eta_charge"));
  CHECK(has_finding("VSCConverter", "i_max_pu"));
  CHECK(has_finding("VSCConverter", "dynamic_model.components.parameters.ki_pll"));
}

TEST_CASE("Digital twin readiness criteria cover the IO maturity dimensions",
          "[io][mapping][digital_twin]") {
  const auto& criteria = hacdcpf::io::digital_twin_readiness_criteria();
  REQUIRE(criteria.size() >= 11);

  std::set<hacdcpf::io::DigitalTwinDimension> dimensions;
  for (const auto& criterion : criteria) {
    CHECK_FALSE(criterion.criterion_id.empty());
    CHECK_FALSE(criterion.title.empty());
    CHECK(criterion.weight > 0.0);
    dimensions.insert(criterion.dimension);
  }

  CHECK(dimensions.count(hacdcpf::io::DigitalTwinDimension::AssetIdentity) ==
        1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::TopologyConnectivity) == 1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::ElectricalParameters) == 1);
  CHECK(dimensions.count(hacdcpf::io::DigitalTwinDimension::DynamicBehavior) ==
        1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::TelemetryObservability) == 1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::StateSynchronization) == 1);
  CHECK(dimensions.count(hacdcpf::io::DigitalTwinDimension::ScenarioEvents) ==
        1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::ReliabilityLifecycle) == 1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::StandardsInteroperability) == 1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::NumericalValidation) == 1);
  CHECK(dimensions.count(
            hacdcpf::io::DigitalTwinDimension::ProvenanceGovernance) == 1);
  CHECK(hacdcpf::io::to_string(
            hacdcpf::io::DigitalTwinDimension::NumericalValidation) ==
        "NumericalValidation");
}

TEST_CASE("Digital twin readiness reports low maturity for empty systems",
          "[io][mapping][digital_twin]") {
  const hacdcpf::HybridPowerSystem empty;
  const auto report = hacdcpf::io::analyze_digital_twin_readiness(empty);

  CHECK(report.max_score > 0.0);
  CHECK(report.score == 0.0);
  CHECK(report.readiness_ratio == 0.0);
  CHECK(report.maturity_level == 0);
  CHECK(report.count(hacdcpf::io::ComponentParameterSeverity::Error) >= 1);
  CHECK_FALSE(report.findings.empty());
}

TEST_CASE("Digital twin readiness improves with dynamic and reliability metadata",
          "[io][mapping][digital_twin]") {
  auto base = make_system_with_all_io_component_tables();
  auto richer = base;

  auto add_profile = [](hacdcpf::DynamicModelProfile& profile,
                        std::string standard,
                        std::string model,
                        std::string source) {
    profile.standard = std::move(standard);
    profile.model_name = std::move(model);
    profile.parameter_set = "validated-default";
    profile.source_id = std::move(source);
    profile.parameters["H"] = 3.0;
  };

  REQUIRE_FALSE(richer.ac.generators.empty());
  add_profile(richer.ac.generators.front().dynamic_model,
              "IEEE",
              "GENROU",
              "unit-test-generator");
  richer.ac.generators.front().forced_outage_rate = 0.03;
  richer.ac.generators.front().mttr_hr = 12.0;
  REQUIRE_FALSE(richer.ac.loads.empty());
  add_profile(richer.ac.loads.front().dynamic_model,
              "GridLABD",
              "ZIP",
              "unit-test-load");
  richer.ac.loads.front().controllable = true;
  REQUIRE_FALSE(richer.ac.storage.empty());
  add_profile(richer.ac.storage.front().dynamic_model,
              "IEEE1547",
              "BESS_GFM",
              "unit-test-storage");
  richer.ac.storage.front().mtbf_battery_hr = 20000.0;
  richer.ac.storage.front().mttr_battery_hr = 8.0;
  REQUIRE_FALSE(richer.vsc_converters.empty());
  add_profile(richer.vsc_converters.front().dynamic_model,
              "NERC",
              "REGC_A",
              "unit-test-vsc");
  richer.vsc_converters.front().forced_outage_rate = 0.02;
  richer.vsc_converters.front().mttr_hr = 6.0;

  const auto base_report =
      hacdcpf::io::analyze_digital_twin_readiness(base);
  const auto rich_report =
      hacdcpf::io::analyze_digital_twin_readiness(richer);

  CHECK(rich_report.score > base_report.score);
  CHECK(rich_report.readiness_ratio > base_report.readiness_ratio);
  CHECK(rich_report.count(hacdcpf::io::ComponentParameterSeverity::Error) <=
        base_report.count(hacdcpf::io::ComponentParameterSeverity::Error));

  const auto has_dynamic_finding = std::any_of(
      rich_report.findings.begin(),
      rich_report.findings.end(),
      [](const hacdcpf::io::DigitalTwinReadinessFinding& finding) {
        return finding.criterion_id == "DT-DYNAMIC-01" &&
               finding.max_score > 0.0;
      });
  CHECK(has_dynamic_finding);
}

TEST_CASE("Digital twin maturity is gated over two axes, not averaged",
          "[io][mapping][digital_twin][maturity]") {
  auto gate = [](const hacdcpf::io::DigitalTwinReadinessReport& r,
                 const std::string& id)
      -> const hacdcpf::io::DigitalTwinMaturityGate* {
    for (const auto& g : r.gates)
      if (g.gate_id == id) return &g;
    return nullptr;
  };

  SECTION("empty system sits at F0/I0 with all gates failing") {
    const hacdcpf::HybridPowerSystem empty;
    const auto report = hacdcpf::io::analyze_digital_twin_readiness(empty);
    CHECK(report.fidelity_level == 0);
    CHECK(report.integration_level == 0);
    CHECK(report.maturity_level == 0);
    // F1, F2, F3, I1, I2 are all reported.
    REQUIRE(report.gates.size() == 5);
    for (const auto& g : report.gates) CHECK_FALSE(g.passed);
  }

  SECTION("a populated, well-formed system cannot reach F3 without stored "
          "round-trip evidence, so maturity is capped below L4") {
    const auto sys = make_system_with_all_io_component_tables();
    const auto report = hacdcpf::io::analyze_digital_twin_readiness(sys);

    // F3 is gated on round-trip evidence (§7), which does not exist yet.
    const auto* f3 = gate(report, "F3");
    REQUIRE(f3 != nullptr);
    CHECK_FALSE(f3->passed);
    CHECK(report.fidelity_level <= 2);

    // Weakest-link projection: without F3, no L4/L5 regardless of other scores.
    CHECK(report.maturity_level < 4);
    // maturity_level is the projection of (F,I), not the weighted average.
    const int expected =
        (report.fidelity_level >= 3 && report.integration_level >= 2)   ? 5
        : (report.fidelity_level >= 3 && report.integration_level >= 1) ? 4
                                                                        : report.fidelity_level;
    CHECK(report.maturity_level == expected);
  }
}

TEST_CASE("Cross-field invariants flag electrically inconsistent nameplates",
          "[io][mapping][parameters][cross_field]") {
  using hacdcpf::io::ComponentParameterSeverity;
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;

  hacdcpf::ACBus b1;
  b1.index = 1;
  b1.base_kv = 110.0;
  hacdcpf::ACBus b2;
  b2.index = 2;
  b2.base_kv = 10.0;  // different voltage level than b1
  sys.ac.buses = {b1, b2};

  // A plain line spanning two voltage levels, with an implausible X/R ratio.
  hacdcpf::ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.0005;
  line.x_pu = 1.0;
  line.rate_a_mva = 50.0;
  sys.ac.branches = {line};

  // A generator whose P/Q operating box exceeds its MVA rating.
  hacdcpf::Generator g;
  g.bus = 1;
  g.pmin_mw = 0.0;
  g.pmax_mw = 100.0;
  g.qmin_mvar = -80.0;
  g.qmax_mvar = 80.0;
  g.mbase_mva = 50.0;
  sys.ac.generators = {g};

  const auto report = hacdcpf::io::analyze_component_parameter_quality(sys);

  const auto has = [&](const std::string& needle) {
    return std::any_of(
        report.findings.begin(), report.findings.end(),
        [&](const hacdcpf::io::ComponentParameterFinding& f) {
          return f.message.find(needle) != std::string::npos;
        });
  };
  CHECK(has("capability infeasible"));       // sqrt(100^2+80^2)=128 > 50 MVA
  CHECK(has("X/R ratio"));                    // 1.0/0.0005 = 2000
  CHECK(has("different base voltages"));      // 110 kV vs 10 kV
  CHECK(report.count(ComponentParameterSeverity::Warning) >= 3);
}
