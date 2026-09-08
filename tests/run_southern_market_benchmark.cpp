#include "hacdcpf/market/southern_market.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/resource.h>

int main(int argc, char** argv) {
  using J = nlohmann::json;
  using namespace hacdcpf::market;
  if (argc < 3 || argc > 11) { std::cerr << "Usage: run_southern_market_benchmark example|demo|118|2000 output.json [seconds] [compact|reference] [auto|solver_default|barrier|dual_simplex] [gurobi|highs|native] [thermal_limit=-1] [solve|inspect] [none|enabled row_presolve] [default|enhanced native_root_cuts]\n"; return 2; }
  const auto start = std::chrono::steady_clock::now();
  J evidence;
  try {
    const std::string kind = argv[1];
    const std::string mode = argc >= 9 ? argv[8] : "solve";
    if (mode != "solve" && mode != "inspect") throw std::invalid_argument("mode must be solve or inspect");
    if (kind != "example" && kind != "demo" && kind != "118" && kind != "118-mixed" && kind != "2000") throw std::invalid_argument("case must be example, demo, 118, 118-mixed or 2000");
    const int thermal_limit=argc>=8 ? std::stoi(argv[7]) : -1;
    if (kind != "2000" && thermal_limit != -1) throw std::invalid_argument("thermal_limit requires the 2000-bus research case");
    auto boundary = kind == "example" ? make_southern_market_example() : kind == "demo" ? make_southern_market_demo() :
      kind == "118-mixed" ? make_southern_market_ieee118_mixed(hacdcpf::io::parse_matpower((std::filesystem::path(HACDCPF_TEST_DATA_DIR).parent_path()/"external_data"/"matpower"/"case118.m").string())) :
      kind == "118" ? make_southern_market_ieee118(hacdcpf::io::parse_matpower((std::filesystem::path(HACDCPF_TEST_DATA_DIR).parent_path()/"external_data"/"matpower"/"case118.m").string())) :
      southern_market_from_system(hacdcpf::io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR)+"/case_ACTIVSg2000.m"),true,thermal_limit);
    boundary["execution"]["solver"]=argc>=7 ? argv[6] : "gurobi";
    boundary["execution"]["threads"]=boundary["execution"]["solver"]=="gurobi" ? 4 : 0;
    boundary["execution"]["time_limit_sec"]=argc >= 4 ? std::stod(argv[3]) : 120;
    boundary["execution"]["formulation"]=argc >= 5 ? argv[4] : "compact";
    boundary["execution"]["row_presolve"]=argc >= 10 ? argv[9] : "none";
    boundary["execution"]["native_root_cuts"]=argc >= 11 ? argv[10] : "default";
    boundary["execution"]["gurobi_method"]=argc >= 6 ? argv[5] : "auto";
    boundary["execution"]["mip_gap"]=.01;
    boundary["execution"]["ac_security"]="schedule_only";
    boundary["execution"]["balance_policy"]="diagnostic";
    boundary["execution"]["balance_penalty_per_mwh"]=100000;
    evidence={{"case",kind},{"thermal_limit",thermal_limit},{"source",boundary["source"]},{"execution",boundary["execution"]},{"entities",J::object()}};
    for(const auto& g:boundary["generators"]) {
      const std::string k=g.at("kind");
      if (!evidence["fleet"].contains(k)) evidence["fleet"][k]=J::object();
      evidence["fleet"][k]["units"]=evidence["fleet"][k].value("units",0)+1;
      evidence["fleet"][k]["capacity_mw"]=evidence["fleet"][k].value("capacity_mw",0.0)+g.at("pmax_mw")[0].get<double>();
    }
    for(const auto* table : {"buses","branches","generators","reservoirs","storage","controllable_loads"}) evidence["entities"][table]=boundary.at(table).size();
    std::cout << evidence.dump() << std::endl;
    const auto result=mode=="inspect" ? inspect_southern_market_model(boundary) : run_southern_day_ahead_market(boundary);
    if (mode=="inspect") evidence["inspection"] = result;
    for(const auto* field : {"status","schedule_feasible","prices_valid","runtime_sec","model_scope","error"}) if(result.contains(field))evidence[field]=result[field];
    for(const auto* stage : {"scuc","sced","lmp"}) if(result.contains(stage)) {
      evidence[stage]=J::object();
      for(const auto* field : {"solver","solver_status","solution_quality","variables","binary_variables","equalities","inequalities","nonzeros","assembly_sec","runtime_sec","audit_sec","mip_gap","objective","max_residual","optimality_proven","objective_terms","compact_units","formulation","reconstructed_max_residual","lp_algorithm","reservoir_scaling","primal_start","model_size","native_diagnostics"})
        if(result[stage].contains(field))evidence[stage][field]=result[stage][field];
    }
  } catch(const std::exception& e) { evidence["error"]=e.what(); }
  evidence["wall_sec"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  rusage usage{}; getrusage(RUSAGE_SELF,&usage);
#ifdef __APPLE__
  evidence["peak_rss_bytes"]=usage.ru_maxrss;
#else
  evidence["peak_rss_bytes"]=usage.ru_maxrss*1024;
#endif
  const std::filesystem::path output=argv[2];
  if(output.has_parent_path())std::filesystem::create_directories(output.parent_path());
  std::ofstream file(output); file << evidence.dump(2) << '\n';
  if(!file) { std::cerr << "Cannot write evidence\n"; return 2; }
  std::cout << evidence.dump(2) << std::endl;
  if (evidence.contains("inspection") && evidence["inspection"].value("status",std::string())=="model_inspected") return 0;
  return evidence.value("schedule_feasible",false) && evidence.value("prices_valid",false) ? 0 : 1;
}
