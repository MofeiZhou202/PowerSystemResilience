#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "hacdcpf/analysis/scenario_generation.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include <algorithm>
#include <filesystem>
#include <numeric>
#include <limits>

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Approx;
namespace {
HybridPowerSystem network() {
  HybridPowerSystem s;
  ACBus from; from.index=1; from.base_kv=12.66;
  ACBus to; to.index=2; to.base_kv=12.66;
  s.ac.buses={from,to};
  ACBranch a; a.index=25; a.from_bus=1; a.to_bus=2; a.length_km=2; a.in_service=true;
  a.line_type="overhead"; a.weather_insulator_wet_ref_kv=8.5;
  DCBranch d; d.index=25; d.from_bus=1; d.to_bus=2; d.length_km=2; d.in_service=true;
  d.line_type="cable"; d.weather_cable_entry_height_m=.20;
  s.ac.branches.push_back(a); s.dc.branches.push_back(d);
  return s;
}
WeatherHazardOptions options(const char* type) {
  auto o=weather_hazard_options_from_json({{"hazard_type",type}});
  o.rainstorm.severity_variation=0; o.lightning.severity_variation=0;
  return o;
}
}

TEST_CASE("Weather schema round trips defaults and rejects invalid intent", "[weather]") {
  const auto catalog=weather_hazard_schema(); REQUIRE(catalog.size()==3);
  for(const auto& hazard:catalog) {
    const std::string type=hazard.at("id");
    if(type=="typhoon") continue;
    nlohmann::json params=nlohmann::json::object();
    for(const auto& f:hazard.at("fields")) params[f.at("key").get<std::string>()]=f.at("default");
    auto o=weather_hazard_options_from_json({{"hazard_type",type},{type,params}});
    const auto result=generate_weather_hazard(network(),o,48,17);
    CHECK(result.evidence.at("parameters")==params);
  }
  CHECK_THROWS(weather_hazard_options_from_json({{"hazard_type","earthquake"}}));
  CHECK_THROWS(weather_hazard_options_from_json({{"rainstorm",{{"typo",1}}}}));
  CHECK_THROWS(weather_hazard_options_from_json({{"rainstorm",{{"total_mm",-1}}}}));
  CHECK_THROWS(weather_hazard_options_from_json({{"lightning",{{"permanent_fraction",2}}}}));
  CHECK_THROWS(weather_hazard_options_from_json({{"lightning",{{"density_km2_hr","2"}}}}));
  auto o=options("rainstorm"); o.rainstorm.duration_hr=48;
  CHECK_THROWS(generate_weather_hazard(network(),o,48,1));
  o=options("lightning"); o.lightning.density_km2_hr=std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS(generate_weather_hazard(network(),o,48,1));
}

TEST_CASE("Rainfall conserves prescribed volume and runoff storage", "[weather]") {
  auto o=options("rainstorm");
  o.rainstorm.drainage_mm_hr=0; o.rainstorm.runoff=.8; o.rainstorm.total_mm=200;
  const auto r=generate_weather_hazard(network(),o,48,8);
  const auto rain=r.evidence.at("profiles").at(0).at("values").get<std::vector<double>>();
  const auto water=r.evidence.at("profiles").at(1).at("values").get<std::vector<double>>();
  CHECK(std::accumulate(rain.begin(),rain.end(),0.0)==Approx(200).epsilon(1e-10));
  CHECK(water.back()==Approx(.160).epsilon(1e-10));
  CHECK(r.peak_intensity==Approx(.160));
  CHECK(*std::max_element(rain.begin(),rain.end())>200/6.0);
  for(int t=0;t<4;++t) CHECK(rain[t]==0);
  for(int t=10;t<48;++t) CHECK(rain[t]==0);
  auto unsafe=o; unsafe.rainstorm.total_mm=1000;
  CHECK_THROWS(generate_weather_hazard(network(),unsafe,48,8));
  o.rainstorm.total_mm=0;
  const auto dry=generate_weather_hazard(network(),o,48,8);
  CHECK(dry.faults.empty()); CHECK(dry.peak_intensity==0); CHECK(dry.peak_failure_probability==0);
  o.rainstorm.total_mm=200; o.rainstorm.runoff=0;
  const auto no_runoff=generate_weather_hazard(network(),o,48,8);
  CHECK(std::none_of(no_runoff.faults.begin(),no_runoff.faults.end(),[](const auto& f) {
    return f.equipment_type=="cable_accessory";
  }));
}

TEST_CASE("Rainfall safe access and domain-qualified damage are reproducible", "[weather]") {
  auto o=options("rainstorm"); o.rainstorm.total_mm=1000;
  o.rainstorm.drainage_mm_hr=25;
  const auto r=generate_weather_hazard(network(),o,48,9);
  REQUIRE(r.faults.size()>=1);
  const auto& equipment=r.evidence.at("affected_equipment");
  CHECK(equipment.at("exposed_ac_branches")==1);
  CHECK(equipment.at("exposed_dc_branches")==1);
  CHECK(equipment.at("failed_ac_branches")==r.faults.size()-1);
  CHECK(equipment.at("failed_dc_branches")==1);
  CHECK(equipment.at("cable_accessory_shutdowns")==1);
  CHECK(equipment.at("fault_count")==static_cast<int>(r.faults.size()));
  CHECK(equipment.at("protective_shutdown_branches")==1);
  CHECK(equipment.at("permanent_failure_branches")==0);
  CHECK(r.faults.back().branch_kind==ResilienceBranchKind::DC);
  for(const auto& e:r.evidence.at("fault_effects")) {
    if(e.at("failure_mode")=="protective_shutdown") {
      CHECK(e.at("safe_access_hr").get<double>()>=10);
      CHECK(e.at("restoration_time_hr").get<double>()>=e.at("safe_access_hr").get<double>()+o.rainstorm.repair_hr);
    }
  }
  CHECK(r.evidence==generate_weather_hazard(network(),o,48,9).evidence);
  auto sys=network(); sys.ac.branches.push_back(sys.ac.branches[0]);
  CHECK_THROWS(generate_weather_hazard(sys,o,48,9));
  sys.ac.branches[1].index=99;
  const auto before=generate_weather_hazard(sys,o,48,9);
  std::reverse(sys.ac.branches.begin(),sys.ac.branches.end());
  CHECK(before.evidence==generate_weather_hazard(sys,o,48,9).evidence);
  sys.ac.branches[0].in_service=false;
  CHECK(generate_weather_hazard(sys,o,48,9).evidence.at("asset_risks").size()==2);
  auto unknown=network(); unknown.ac.branches[0].line_type="unknown-demo-type";
  CHECK(generate_weather_hazard(unknown,o,48,9).evidence.at("affected_equipment").at("assumed_line_types")==1);
}

TEST_CASE("Lightning trip probability matches thinned Poisson exposure", "[weather]") {
  auto o=options("lightning"); o.lightning.critical_current_ka=o.lightning.median_current_ka;
  o.lightning.density_km2_hr=1;
  const auto r=generate_weather_hazard(network(),o,48,1);
  // length 2 km * width 0.1 km * 1 strike/km2/h * P(I>median)=0.1/h
  CHECK(r.peak_failure_probability==Approx(1-std::exp(-.1*6)).epsilon(1e-12));
  int faults=0;
  for(unsigned seed=0;seed<1000;++seed) faults+=static_cast<int>(generate_weather_hazard(network(),o,48,seed).faults.size());
  CHECK(faults/1000.0==Approx(1-std::exp(-.6)).margin(.05));
  o.lightning.density_km2_hr=0;
  CHECK(generate_weather_hazard(network(),o,48,1).faults.empty());
  o.lightning.density_km2_hr=1; o.lightning.collection_width_m=0;
  CHECK(generate_weather_hazard(network(),o,48,1).peak_failure_probability==0);
}

TEST_CASE("Lightning temporary trips and permanent faults have distinct recovery windows", "[weather]") {
  auto o=options("lightning"); o.lightning.density_km2_hr=1000;
  o.lightning.permanent_fraction=0;
  auto sys=network(); sys.ac.branches[0].length_km=0;
  const auto temporary=generate_weather_hazard(sys,o,48,12);
  REQUIRE(temporary.faults.size()==1);
  CHECK(temporary.evidence.at("fallback_length_count")==1);
  for(const auto& e:temporary.evidence.at("fault_effects")) {
    CHECK(e.at("failure_mode")=="temporary_trip");
    CHECK(e.at("hands_on_repair_hr")==0);
    CHECK(e.at("restoration_time_hr").get<double>()<10);
  }
  o.lightning.permanent_fraction=1;
  const auto permanent=generate_weather_hazard(sys,o,48,12);
  for(const auto& e:permanent.evidence.at("fault_effects")) {
    CHECK(e.at("failure_mode")=="permanent"); CHECK(e.at("safe_access_hr")==10);
    CHECK(e.at("restoration_time_hr")==14);
  }
}

TEST_CASE("Rainstorm transformer moisture has a distinct equipment identity and electrical outage", "[weather]") {
  auto sys=network();
  sys.ac.branches[0].line_type="transformer_equivalent";
  Transformer2W tr; tr.index=9001; tr.hv_bus=1; tr.lv_bus=2;
  tr.source_branch_idx=25; tr.weather_moisture_vulnerable=true;
  sys.ac.transformers_2w.push_back(tr);
  auto o=options("rainstorm"); o.rainstorm.total_mm=1000;
  o.rainstorm.transformer_paper_a=.01;
  const auto r=generate_weather_hazard(sys,o,48,1);
  REQUIRE(r.faults.size()>=1);
  const auto it=std::find_if(r.faults.begin(),r.faults.end(),[](const auto& f) {
    return f.equipment_type=="transformer_2w";
  });
  REQUIRE(it!=r.faults.end());
  CHECK(it->equipment_index==9001);
  CHECK(it->branch_kind==ResilienceBranchKind::AC);
  CHECK(it->branch_index==25);
  CHECK(r.evidence.at("affected_equipment").at("transformer_moisture_shutdowns")==1);
}

TEST_CASE("Mixed weather case assigns asset classes and lightning excludes cables", "[weather]") {
  io::set_case_data_root(std::filesystem::path(__FILE__).parent_path().parent_path()/"external_data"/"matpower");
  const auto sys=io::build_dist33_weather_mixed();
  CHECK(sys.ac.buses.size()==33);
  CHECK(std::count_if(sys.ac.branches.begin(),sys.ac.branches.end(),[](const auto& b){return b.line_type=="cable";})>0);
  CHECK(std::count_if(sys.ac.branches.begin(),sys.ac.branches.end(),[](const auto& b){return b.line_type=="overhead";})>0);
  REQUIRE(sys.ac.transformers_2w.size()>=1);
  CHECK(sys.ac.transformers_2w.back().weather_moisture_vulnerable);
  auto o=options("lightning"); o.lightning.density_km2_hr=1000;
  const auto r=generate_weather_hazard(sys,o,48,11);
  REQUIRE(!r.faults.empty());
  CHECK(std::all_of(r.faults.begin(),r.faults.end(),[](const auto& f){return f.equipment_type=="overhead_line" && f.branch_kind==ResilienceBranchKind::AC;}));
  const auto rain=generate_weather_hazard(sys,options("rainstorm"),48,11);
  CHECK(rain.evidence.at("affected_equipment").at("exposed_transformers")==1);
  auto severe=options("rainstorm"); severe.rainstorm.total_mm=1000;
  severe.rainstorm.transformer_paper_a=.01;
  const auto saturated=generate_weather_hazard(sys,severe,48,11);
  CHECK(std::any_of(saturated.faults.begin(),saturated.faults.end(),[](const auto& f) {
    return f.equipment_type=="transformer_2w" && f.equipment_index==9001 && f.branch_index==1;
  }));
}

TEST_CASE("Weather candidates cluster and serialize without typhoon catalog", "[weather]") {
  for(const std::string type:{"rainstorm","lightning"}) {
    ScenarioGenerationOptions o;
    o.regular.enabled=false; o.reliability.enabled=false;
    o.resilience.weather=options(type.c_str());
    o.resilience.candidates_per_intensity=6; o.resilience.default_cluster_count=2;
    const auto response=scenario_generation_result_to_json(generate_scenarios(network(),o));
    const auto& group=response.at("resilience").at("intensities").at(0);
    CHECK(group.at("hazard_type")==type); CHECK(group.at("intensity")==type);
    REQUIRE(group.at("clusters").size()==2);
    double probability=0;
    for(const auto& cluster:group.at("clusters")) {
      probability+=cluster.at("probability").get<double>();
      const auto& r=cluster.at("representative");
      CHECK(r.at("resilience_event").at("hazard_type")==type);
      CHECK(r.at("resilience_event").at("selected_track_max_vmax_ms").is_null());
      CHECK(r.at("resilience_event").at("track").empty());
      CHECK(r.contains("standard_time_series"));
      CHECK(r.at("resilience_event").at("hazard_evidence").at("parameters").is_object());
    }
    CHECK(probability==Approx(1));
  }
}
