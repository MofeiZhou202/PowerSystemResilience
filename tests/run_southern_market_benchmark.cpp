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
  if (argc < 3 || argc > 5) { std::cerr << "Usage: run_southern_market_benchmark example|2000 output.json [seconds] [compact|reference]\n"; return 2; }
  const auto start = std::chrono::steady_clock::now();
  J evidence;
  try {
    const std::string kind = argv[1];
    if (kind != "example" && kind != "2000") throw std::invalid_argument("case must be example or 2000");
    auto boundary = kind == "example" ? make_southern_market_example() :
      southern_market_from_system(hacdcpf::io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR)+"/case_ACTIVSg2000.m"),true);
    boundary["execution"]["solver"]="gurobi";
    boundary["execution"]["threads"]=4;
    boundary["execution"]["time_limit_sec"]=argc >= 4 ? std::stod(argv[3]) : 120;
    boundary["execution"]["formulation"]=argc == 5 ? argv[4] : "compact";
    boundary["execution"]["mip_gap"]=.01;
    boundary["execution"]["ac_security"]="schedule_only";
    boundary["execution"]["balance_policy"]="diagnostic";
    boundary["execution"]["balance_penalty_per_mwh"]=100000;
    evidence={{"case",kind},{"execution",boundary["execution"]},{"entities",J::object()}};
    for(const auto* table : {"buses","branches","generators","reservoirs","storage","controllable_loads"}) evidence["entities"][table]=boundary.at(table).size();
    std::cout << evidence.dump() << std::endl;
    const auto result=run_southern_day_ahead_market(boundary);
    for(const auto* field : {"status","schedule_feasible","prices_valid","runtime_sec","model_scope","error"}) if(result.contains(field))evidence[field]=result[field];
    for(const auto* stage : {"scuc","sced","lmp"}) if(result.contains(stage)) {
      evidence[stage]=J::object();
      for(const auto* field : {"solver","solver_status","solution_quality","variables","binary_variables","equalities","inequalities","nonzeros","assembly_sec","runtime_sec","audit_sec","mip_gap","objective","max_residual","optimality_proven","objective_terms","compact_units","formulation","reconstructed_max_residual"})
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
  return evidence.value("schedule_feasible",false) && evidence.value("prices_valid",false) ? 0 : 1;
}
