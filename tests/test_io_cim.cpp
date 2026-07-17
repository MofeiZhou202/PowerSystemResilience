// Bounded CIM / CGMES 3.0 (EQ+SSH) import/export contract (§5).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/cim_dist_io.hpp"
#include "hacdcpf/io/cim_io.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace {
bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
}  // namespace

TEST_CASE("CIM CGMES bounded round-trip preserves the EQ+SSH subset",
          "[io][cim]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus b1; b1.index = 1; b1.base_kv = 110.0; b1.name = "B1";
  ACBus b2; b2.index = 2; b2.base_kv = 110.0; b2.name = "B2";
  ACBus b3; b3.index = 3; b3.base_kv = 20.0;  b3.name = "B3";
  sys.ac.buses = {b1, b2, b3};

  ACBranch l1;
  l1.index = 1; l1.from_bus = 1; l1.to_bus = 2;
  l1.r_pu = 0.01; l1.x_pu = 0.10; l1.b_pu = 0.02; l1.name = "L1";
  ACBranch l2;
  l2.index = 2; l2.from_bus = 2; l2.to_bus = 3;
  l2.r_pu = 0.02; l2.x_pu = 0.20; l2.b_pu = 0.0; l2.name = "L2";
  sys.ac.branches = {l1, l2};

  Generator g;
  g.index = 1; g.bus = 1; g.name = "G1";
  g.pmin_mw = 0.0; g.pmax_mw = 100.0;
  g.qmin_mvar = -50.0; g.qmax_mvar = 50.0;
  g.mbase_mva = 120.0; g.pg_mw = 40.0;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1; ld.bus = 3; ld.name = "LD1"; ld.p_mw = 30.0; ld.q_mvar = 10.0;
  sys.ac.loads = {ld};

  const std::string xml = io::to_cim(sys);
  INFO(xml);
  const auto res = io::from_cim(xml);
  const auto& r = res.system;

  REQUIRE(r.ac.buses.size() == 3);
  REQUIRE(r.ac.branches.size() == 2);
  REQUIRE(r.ac.generators.size() == 1);
  REQUIRE(r.ac.loads.size() == 1);

  auto kv = [&](int idx) {
    for (const auto& b : r.ac.buses)
      if (b.index == idx) return b.base_kv;
    return -1.0;
  };
  CHECK(near(kv(1), 110.0));
  CHECK(near(kv(3), 20.0));

  const auto& rl1 = r.ac.branches.front();
  CHECK(near(rl1.r_pu, 0.01));
  CHECK(near(rl1.x_pu, 0.10));
  CHECK(near(rl1.b_pu, 0.02));
  CHECK(rl1.from_bus == 1);
  CHECK(rl1.to_bus == 2);

  CHECK(near(r.ac.generators.front().pmax_mw, 100.0));
  CHECK(near(r.ac.generators.front().mbase_mva, 120.0));
  CHECK(r.ac.generators.front().bus == 1);

  CHECK(near(r.ac.loads.front().p_mw, 30.0));
  CHECK(near(r.ac.loads.front().q_mvar, 10.0));
  CHECK(r.ac.loads.front().bus == 3);

  CHECK(res.report.binding_level == io::ImportBindingLevel::Rich);
  CHECK_FALSE(res.report.has_errors());
}

TEST_CASE("CIM import drops out-of-scope classes with a structural-loss record",
          "[io][cim]") {
  const std::string xml =
      "<?xml version=\"1.0\"?>\n"
      "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" "
      "xmlns:cim=\"http://iec.ch/TC57/CIM100#\">\n"
      "  <cim:PowerTransformer rdf:ID=\"_t1\">\n"
      "    <cim:IdentifiedObject.name>T1</cim:IdentifiedObject.name>\n"
      "  </cim:PowerTransformer>\n"
      "</rdf:RDF>\n";
  const auto res = hacdcpf::io::from_cim(xml);
  bool structural_loss = false;
  for (const auto& rec : res.report.records)
    if (rec.reason_code == hacdcpf::io::ImportReasonCode::StructuralLoss)
      structural_loss = true;
  CHECK(structural_loss);
}

TEST_CASE("CIM parser rejects DOCTYPE/ENTITY (XXE hardening, §12)",
          "[io][cim][security]") {
  const std::string evil =
      "<?xml version=\"1.0\"?>\n"
      "<!DOCTYPE foo [ <!ENTITY xxe SYSTEM \"file:///etc/passwd\"> ]>\n"
      "<rdf:RDF></rdf:RDF>\n";
  const auto res = hacdcpf::io::from_cim(evil);
  CHECK(res.report.has_errors());
}

TEST_CASE("Distribution CIM fixtures import and round-trip",
          "[io][cim][distribution][roundtrip]") {
  namespace fs = std::filesystem;
  const fs::path fixture_dir =
      fs::path(__FILE__).parent_path().parent_path() / "external_data" / "xml";
  REQUIRE(fs::is_directory(fixture_dir));

  std::vector<fs::path> fixtures;
  for (const auto& entry : fs::directory_iterator(fixture_dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".xml") {
      fixtures.push_back(entry.path());
    }
  }
  std::sort(fixtures.begin(), fixtures.end());
  REQUIRE(fixtures.size() == 3);

  for (const auto& fixture : fixtures) {
    CAPTURE(fixture.string());
    const auto imported = hacdcpf::io::load_cim_dist(fixture);
    CHECK_FALSE(imported.report.has_errors());
    CHECK(imported.report.binding_level ==
          hacdcpf::io::ImportBindingLevel::Canonical);
    CHECK_FALSE(imported.system.ac.buses.empty());
    CHECK_FALSE(imported.system.ac.branches.empty());
    CHECK(imported.system.ac.transformers_2w.size() == 1);
    CHECK(imported.system.ac.external_grids.size() == 1);
    CHECK_FALSE(imported.system.ac.loads.empty());

    const std::string exported = hacdcpf::io::to_cim_dist(imported.system);
    CHECK(exported.find("<rdf:RDF") != std::string::npos);
    CHECK(exported.find("<cim:PowerTransformer") != std::string::npos);
    const auto round_trip = hacdcpf::io::from_cim_dist(exported);
    CHECK_FALSE(round_trip.report.has_errors());
    CHECK(round_trip.system.ac.buses.size() == imported.system.ac.buses.size());
    CHECK(round_trip.system.ac.branches.size() ==
          imported.system.ac.branches.size());
    CHECK(round_trip.system.ac.loads.size() == imported.system.ac.loads.size());
  }
}

TEST_CASE("Distribution CIM parser rejects DOCTYPE and ENTITY declarations",
          "[io][cim][distribution][security]") {
  const std::string evil =
      "<?xml version=\"1.0\"?>\n"
      "<!DOCTYPE foo [ <!ENTITY xxe SYSTEM \"file:///etc/passwd\"> ]>\n"
      "<rdf:RDF></rdf:RDF>\n";
  const auto result = hacdcpf::io::from_cim_dist(evil);
  CHECK(result.report.has_errors());
  CHECK(result.system.ac.buses.empty());
}

TEST_CASE("Distribution CIM stitches documents and adds one source per feeder",
          "[io][cim][distribution][multifile]") {
  const std::string first = R"xml(<?xml version="1.0"?>
<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV10"><cim:BaseVoltage.nominalVoltage>10000</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:BaseVoltage rdf:ID="BV04"><cim:BaseVoltage.nominalVoltage>400</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:PowerTransformer rdf:ID="T1"><cim:Naming.name>T1</cim:Naming.name><cim:PowerTransformer.ratedCapacity>0.4</cim:PowerTransformer.ratedCapacity></cim:PowerTransformer>
 <cim:ConnectivityNode rdf:ID="N1"/>
</rdf:RDF>)xml";
  const std::string second = R"xml(<?xml version="1.0"?>
<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV10"><cim:BaseVoltage.nominalVoltage>10000</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:BaseVoltage rdf:ID="BV04"><cim:BaseVoltage.nominalVoltage>400</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:TransformerWinding rdf:ID="T1P"><cim:TransformerWinding.windingType>primary</cim:TransformerWinding.windingType><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV10"/><cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource="#T1"/></cim:TransformerWinding>
 <cim:Terminal rdf:ID="T1PT"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#T1P"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:TransformerWinding rdf:ID="T1S"><cim:TransformerWinding.windingType>secondary</cim:TransformerWinding.windingType><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV04"/><cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource="#T1"/></cim:TransformerWinding>
 <cim:Terminal rdf:ID="T1ST"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#T1S"/><cim:Terminal.ConnectivityNode rdf:resource=""/></cim:Terminal>
 <cim:PowerTransformer rdf:ID="T2"><cim:Naming.name>T2</cim:Naming.name><cim:PowerTransformer.ratedCapacity>0.63</cim:PowerTransformer.ratedCapacity></cim:PowerTransformer>
 <cim:TransformerWinding rdf:ID="T2P"><cim:TransformerWinding.windingType>primary</cim:TransformerWinding.windingType><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV10"/><cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource="#T2"/></cim:TransformerWinding>
 <cim:Terminal rdf:ID="T2PT"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#T2P"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
 <cim:TransformerWinding rdf:ID="T2S"><cim:TransformerWinding.windingType>secondary</cim:TransformerWinding.windingType><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV04"/><cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource="#T2"/></cim:TransformerWinding>
 <cim:Terminal rdf:ID="T2ST"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#T2S"/><cim:Terminal.ConnectivityNode rdf:resource=""/></cim:Terminal>
 <cim:ACLineSegment rdf:ID="L1"><cim:Conductor.length>100</cim:Conductor.length><cim:Conductor.crossSectionArea>50</cim:Conductor.crossSectionArea><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV10"/></cim:ACLineSegment>
 <cim:Terminal rdf:ID="L1T1"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:Terminal rdf:ID="L1T2"><cim:Terminal.sequenceNumber>2</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
 <cim:ConnectivityNode rdf:ID="N2"/>
</rdf:RDF>)xml";

  const auto imported =
      hacdcpf::io::from_cim_dist(std::vector<std::string>{first, second});
  CHECK_FALSE(imported.report.has_errors());
  CHECK(imported.system.ac.buses.size() == 4);
  CHECK(imported.system.ac.branches.size() == 1);
  CHECK(imported.system.ac.transformers_2w.size() == 2);
  CHECK(imported.system.ac.external_grids.size() == 1);
  REQUIRE(imported.system.ac.external_grids.size() == 1);
  const int source_bus = imported.system.ac.external_grids.front().bus;
  const auto source = std::find_if(
      imported.system.ac.buses.begin(), imported.system.ac.buses.end(),
      [source_bus](const auto& bus) { return bus.index == source_bus; });
  REQUIRE(source != imported.system.ac.buses.end());
  CHECK(source->bus_type == hacdcpf::BusType::SLACK);
  CHECK(near(source->base_kv, 10.0));
  CHECK(std::any_of(imported.warnings.begin(), imported.warnings.end(),
                    [](const std::string& warning) {
                      return warning.find("合并 2 条重复对象声明") !=
                             std::string::npos;
                    }));
}

TEST_CASE("Distribution CIM folder stitching completes a physical name from a duplicate object",
          "[io][cim][distribution][multifile][names]") {
  const std::string topology = R"xml(<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV"><cim:BaseVoltage.nominalVoltage>10000</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:ConnectivityNode rdf:ID="N1"/>
 <cim:ConnectivityNode rdf:ID="N2"><cim:Naming.name>受端节点</cim:Naming.name></cim:ConnectivityNode>
 <cim:ACLineSegment rdf:ID="L1"><cim:Conductor.length>100</cim:Conductor.length><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV"/></cim:ACLineSegment>
 <cim:Terminal rdf:ID="L1T1"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:Terminal rdf:ID="L1T2"><cim:Terminal.sequenceNumber>2</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
</rdf:RDF>)xml";
  const std::string names = R"xml(<rdf:RDF>
 <cim:ConnectivityNode rdf:about="#N1"><cim:Naming.name>馈线首端节点</cim:Naming.name></cim:ConnectivityNode>
 <cim:ACLineSegment rdf:about="#L1"><cim:Naming.name>人民路一回线</cim:Naming.name></cim:ACLineSegment>
</rdf:RDF>)xml";

  const auto imported =
      hacdcpf::io::from_cim_dist(std::vector<std::string>{topology, names});
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.ac.buses.size() == 2);
  REQUIRE(imported.system.ac.branches.size() == 1);
  CHECK(imported.system.ac.buses[0].name == "馈线首端节点");
  CHECK(imported.system.ac.buses[1].name == "受端节点");
  CHECK(imported.system.ac.branches[0].name == "人民路一回线");
}

TEST_CASE("Distribution CIM real feeder creates transformer-side fallback loads",
          "[io][cim][distribution][fixture]") {
  namespace fs = std::filesystem;
  const fs::path fixture =
      fs::path(__FILE__).parent_path().parent_path() / "data" / "test.xml";
  REQUIRE(fs::is_regular_file(fixture));

  const auto imported = hacdcpf::io::load_cim_dist(fixture);
  CHECK_FALSE(imported.report.has_errors());
  CHECK(imported.source_load_objects == 0);
  CHECK(imported.source_generator_objects == 0);
  REQUIRE(imported.system.ac.transformers_2w.size() == 41);
  CHECK(std::any_of(imported.system.ac.transformers_2w.begin(),
                    imported.system.ac.transformers_2w.end(),
                    [](const auto& transformer) {
                      return transformer.name == "大刀沙水厂#2变压器";
                    }));
  CHECK(std::any_of(imported.system.ac.switches.begin(),
                    imported.system.ac.switches.end(), [](const auto& sw) {
                      return sw.name ==
                             "10kV草河F27大刀沙水厂支#04杆04T02RD跌落式熔断器";
                    }));
  CHECK(imported.inferred_load_objects == 41);
  CHECK(imported.system.ac.loads.size() == 41);
  CHECK(imported.system.ac.generators.empty());
  CHECK(std::count_if(imported.system.ac.switches.begin(),
                      imported.system.ac.switches.end(), [](const auto& sw) {
          return sw.switch_type == hacdcpf::SwitchType::Fuse;
        }) == 28);
  CHECK(std::count_if(imported.system.ac.switches.begin(),
                      imported.system.ac.switches.end(), [](const auto& sw) {
          return sw.switch_type == hacdcpf::SwitchType::LoadBreakSwitch;
        }) == 55);
  CHECK(std::all_of(imported.system.ac.switches.begin(),
                    imported.system.ac.switches.end(), [](const auto& sw) {
          return sw.switch_type != hacdcpf::SwitchType::Fuse ||
                 (sw.role == hacdcpf::SwitchRole::Protection &&
                  !hacdcpf::effective_switch_capabilities(sw)
                       .can_close_for_restoration);
        }));
  auto completion_system = imported.system;
  const auto binding_preview =
      hacdcpf::complete_design_handbook_parameters(completion_system);
  CHECK(binding_preview.switches_scanned ==
        static_cast<int>(imported.system.ac.switches.size()));
  CHECK(binding_preview.switch_bindings.size() ==
        imported.system.ac.switches.size());
  CHECK(binding_preview.switch_binding_candidates > 0);
  CHECK(std::any_of(binding_preview.switch_bindings.begin(),
                    binding_preview.switch_bindings.end(), [](const auto& row) {
        return row.controlled_element_index >= 0 || row.ambiguous;
      }));
  REQUIRE(imported.system.three_phase_ac.has_value());
  CHECK(imported.system.three_phase_ac->loads.size() == 41);
  CHECK(imported.has_explicit_phase_data);
  CHECK_FALSE(imported.is_unbalanced);
}

TEST_CASE("Distribution CIM folder preserves loads and unbalanced meter phases",
          "[io][cim][distribution][fixture][three_phase]") {
  namespace fs = std::filesystem;
  const fs::path fixture_dir =
      fs::path(__FILE__).parent_path().parent_path() / "data" / "xmls";
  REQUIRE(fs::is_directory(fixture_dir));
  std::vector<fs::path> fixtures;
  for (const auto& entry : fs::directory_iterator(fixture_dir))
    if (entry.is_regular_file() && entry.path().extension() == ".xml")
      fixtures.push_back(entry.path());
  std::sort(fixtures.begin(), fixtures.end());
  REQUIRE(fixtures.size() == 7);

  const auto imported = hacdcpf::io::load_cim_dist(fixtures);
  CHECK_FALSE(imported.report.has_errors());
  CHECK(imported.system.ac.transformers_2w.size() == 7);
  const std::vector<std::string> expected_transformer_names = {
      "丹山新村3A公变房#1变压器", "丹山新村3B公变房#2变压器",
      "丹山新村3C公变房#3变压器", "丹山新村3D公变房#4变压器",
      "丹山新村3E公变房#5变压器", "丹山新村5A公变房#1变压器",
      "丹山新村5B公变房#2变压器"};
  for (const auto& expected_name : expected_transformer_names) {
    CHECK(std::any_of(imported.system.ac.transformers_2w.begin(),
                      imported.system.ac.transformers_2w.end(),
                      [&](const auto& transformer) {
                        return transformer.name == expected_name;
                      }));
  }
  CHECK(imported.source_load_objects == 0);
  // 138 LVBuilding loads in the newer exports plus one transformer-side
  // fallback load for each of the four older exports with no LVBuilding data.
  CHECK(imported.inferred_load_objects == 142);
  CHECK(imported.system.ac.loads.size() == 142);
  CHECK(imported.source_generator_objects == 0);
  CHECK(imported.system.ac.generators.empty());
  // Each transformer-area export omits its upstream 10 kV feeder. The two 5A
  // and 5B LV networks are tied, but that must not suppress the 5B HV source.
  CHECK(imported.system.ac.external_grids.size() == 7);
  CHECK(imported.has_explicit_phase_data);
  CHECK(imported.is_unbalanced);
  REQUIRE(imported.system.three_phase_ac.has_value());
  const auto& phase = *imported.system.three_phase_ac;
  CHECK(phase.loads.size() == imported.system.ac.loads.size());
  CHECK(std::any_of(phase.loads.begin(), phase.loads.end(), [](const auto& load) {
    return load.phase_mask.bits == hacdcpf::PhaseMask::a().bits;
  }));
  CHECK(std::any_of(phase.loads.begin(), phase.loads.end(), [](const auto& load) {
    return load.phase_mask.bits == hacdcpf::PhaseMask::c().bits;
  }));

  hacdcpf::PowerFlowOptions pf_options;
  pf_options.max_iter = 200;
  pf_options.tol = 1e-8;
  pf_options.enable_converter_coordination_check = false;
  const auto pf = hacdcpf::solve_power_flow(imported.system, pf_options);
  REQUIRE(pf.converged);
  REQUIRE_FALSE(pf.vm.empty());
  CHECK(*std::min_element(pf.vm.begin(), pf.vm.end()) > 0.9);

  const std::string json = hacdcpf::io::to_json(imported.system);
  const auto restored = hacdcpf::io::from_json(json);
  REQUIRE(restored.three_phase_ac.has_value());
  REQUIRE(restored.three_phase_ac->loads.size() == phase.loads.size());
  CHECK(restored.three_phase_ac->loads.front().phase_mask.bits ==
        phase.loads.front().phase_mask.bits);
}

TEST_CASE("Distribution CIM maps explicit loads and generators",
          "[io][cim][distribution][equipment]") {
  const std::string xml = R"xml(<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV"><cim:BaseVoltage.nominalVoltage>400</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:ConnectivityNode rdf:ID="N1"/><cim:ConnectivityNode rdf:ID="N2"/>
 <cim:EnergyConsumer rdf:ID="LD"><cim:Naming.name>Load A</cim:Naming.name><cim:EnergyConsumer.p>0.12</cim:EnergyConsumer.p><cim:EnergyConsumer.q>0.04</cim:EnergyConsumer.q><cim:ConductingEquipment.phases>A相</cim:ConductingEquipment.phases><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV"/></cim:EnergyConsumer>
 <cim:Terminal rdf:ID="LDT"><cim:Terminal.ConductingEquipment rdf:resource="#LD"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:SynchronousMachine rdf:ID="GEN"><cim:Naming.name>Generator C</cim:Naming.name><cim:RotatingMachine.p>0.08</cim:RotatingMachine.p><cim:SynchronousMachine.maxP>0.1</cim:SynchronousMachine.maxP><cim:ConductingEquipment.phases>C相</cim:ConductingEquipment.phases><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV"/></cim:SynchronousMachine>
 <cim:Terminal rdf:ID="GENT"><cim:Terminal.ConductingEquipment rdf:resource="#GEN"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
</rdf:RDF>)xml";
  const auto imported = hacdcpf::io::from_cim_dist(xml);
  REQUIRE(imported.system.ac.loads.size() == 1);
  REQUIRE(imported.system.ac.generators.size() == 1);
  CHECK(near(imported.system.ac.loads.front().p_mw, 0.12));
  CHECK(near(imported.system.ac.generators.front().pg_mw, 0.08));
  CHECK(imported.source_load_objects == 1);
  CHECK(imported.source_generator_objects == 1);
  REQUIRE(imported.system.three_phase_ac.has_value());
  CHECK(imported.system.three_phase_ac->loads.front().phase_mask.bits ==
        hacdcpf::PhaseMask::a().bits);
  CHECK(imported.system.three_phase_ac->generators.front().phase_mask.bits ==
        hacdcpf::PhaseMask::c().bits);
  CHECK(imported.is_unbalanced);
}

TEST_CASE("Distribution CIM uses MV conductor fallback before PSR geometry",
          "[io][cim][distribution][parameters]") {
  const std::string xml = R"xml(<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV10"><cim:BaseVoltage.nominalVoltage>10000</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:PSRType rdf:ID="CABLE"><cim:Naming.name>配电电缆段</cim:Naming.name></cim:PSRType>
 <cim:ConnectivityNode rdf:ID="N1"/><cim:ConnectivityNode rdf:ID="N2"/>
 <cim:ACLineSegment rdf:ID="L1"><cim:Naming.name>10 kV cable without size</cim:Naming.name><cim:Conductor.length>100</cim:Conductor.length><cim:PowerSystemResource.PSRType rdf:resource="#CABLE"/><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV10"/></cim:ACLineSegment>
 <cim:Terminal rdf:ID="L1T1"><cim:Terminal.sequenceNumber>1</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:Terminal rdf:ID="L1T2"><cim:Terminal.sequenceNumber>2</cim:Terminal.sequenceNumber><cim:Terminal.ConductingEquipment rdf:resource="#L1"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
</rdf:RDF>)xml";

  hacdcpf::io::CimDistImportOptions options;
  options.default_mv_cross_section_mm2 = 300.0;
  const auto imported = hacdcpf::io::from_cim_dist(
      xml, hacdcpf::io::ImportMode::Permissive, options);
  REQUIRE(imported.system.ac.branches.size() == 1);
  const auto& branch = imported.system.ac.branches.front();
  CHECK(branch.cross_section_inferred);
  CHECK_THAT(branch.cross_section_mm2,
             Catch::Matchers::WithinAbs(300.0, 1e-12));
  CHECK_THAT(branch.rate_a_mva,
             Catch::Matchers::WithinAbs(std::sqrt(3.0) * 10.0 * 0.6, 1e-12));
  CHECK_THAT(imported.system.ac.buses.front().base_kv,
             Catch::Matchers::WithinAbs(10.0, 1e-12));
}

TEST_CASE("Distribution CIM preserves switching equipment classes and roles",
          "[io][cim][distribution][switchgear]") {
  const std::string xml = R"xml(<rdf:RDF>
 <cim:BaseVoltage rdf:ID="BV"><cim:BaseVoltage.nominalVoltage>10000</cim:BaseVoltage.nominalVoltage></cim:BaseVoltage>
 <cim:ConnectivityNode rdf:ID="N1"/><cim:ConnectivityNode rdf:ID="N2"/><cim:ConnectivityNode rdf:ID="N3"/>
 <cim:Fuse rdf:ID="F1"><cim:Naming.name>dropout fuse</cim:Naming.name><cim:Switch.normalOpen>true</cim:Switch.normalOpen><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV"/></cim:Fuse>
 <cim:Terminal rdf:ID="F1T1"><cim:Terminal.ConductingEquipment rdf:resource="#F1"/><cim:Terminal.ConnectivityNode rdf:resource="#N1"/></cim:Terminal>
 <cim:Terminal rdf:ID="F1T2"><cim:Terminal.ConductingEquipment rdf:resource="#F1"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
 <cim:LoadBreakSwitch rdf:ID="T1"><cim:Naming.name>ring tie</cim:Naming.name><cim:Switch.normalOpen>true</cim:Switch.normalOpen><cim:Switch.isRing>true</cim:Switch.isRing><cim:PowerSystemResource.BaseVoltage rdf:resource="#BV"/></cim:LoadBreakSwitch>
 <cim:Terminal rdf:ID="T1T1"><cim:Terminal.ConductingEquipment rdf:resource="#T1"/><cim:Terminal.ConnectivityNode rdf:resource="#N2"/></cim:Terminal>
 <cim:Terminal rdf:ID="T1T2"><cim:Terminal.ConductingEquipment rdf:resource="#T1"/><cim:Terminal.ConnectivityNode rdf:resource="#N3"/></cim:Terminal>
</rdf:RDF>)xml";

  const auto imported = hacdcpf::io::from_cim_dist(xml);
  REQUIRE(imported.system.ac.switches.size() == 2);
  const auto& fuse = imported.system.ac.switches[0];
  const auto& tie = imported.system.ac.switches[1];
  CHECK(fuse.switch_type == hacdcpf::SwitchType::Fuse);
  CHECK(fuse.role == hacdcpf::SwitchRole::Protection);
  CHECK_FALSE(hacdcpf::effective_switch_capabilities(fuse).can_close_for_restoration);
  CHECK(tie.switch_type == hacdcpf::SwitchType::LoadBreakSwitch);
  CHECK(tie.role == hacdcpf::SwitchRole::Tie);
  CHECK(hacdcpf::effective_switch_capabilities(tie).can_close_for_restoration);

  hacdcpf::ProjectionOptions projection_options;
  projection_options.strip_dead_islands = false;
  projection_options.preserve_switch_branches = true;
  const auto projected = hacdcpf::project_to_canonical_models(
      imported.system, projection_options);
  CHECK(std::any_of(projected.ac.branches.begin(), projected.ac.branches.end(),
                    [](const auto& branch) {
                      return branch.name == "dropout fuse";
                    }));
  CHECK(std::any_of(projected.ac.branches.begin(), projected.ac.branches.end(),
                    [](const auto& branch) {
                      return branch.name == "ring tie";
                    }));

  const std::string exported = hacdcpf::io::to_cim_dist(imported.system);
  CHECK(exported.find("<cim:Fuse") != std::string::npos);
  CHECK(exported.find("<cim:LoadBreakSwitch") != std::string::npos);
}
