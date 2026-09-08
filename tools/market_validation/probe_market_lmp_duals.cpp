// Single-assembly LP diagnostic; see performance.md, price repeatability audit.
// Include the implementation to inspect its internal Build without expanding
// the production API. This executable replaces, never links, its market object.
#include "../../src/market/southern_market.cpp"
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  using namespace hacdcpf::market;
  nlohmann::json request;
  std::ifstream(argv[1]) >> request;
  auto input = preview_market_operation_boundary(request.at("boundary"),request.at("config"),0).at("effective");
  std::map<std::string,double> dispatch;
  for (const std::string stage : {"scuc","sced"}) {
    auto lease = build_model(input,stage,std::move(dispatch),{});
    const auto solved = solve(*lease,input);
    if (!stage_result(*lease,solved,input).at("feasible").get<bool>()) return 3;
    dispatch = solution_map(*lease,solved);
  }
  auto lease = build_model(input,"lmp",std::move(dispatch),{});
  nlohmann::json results = nlohmann::json::object();
  Eigen::VectorXd expected;
  for (const std::string method : {"auto","dual_simplex","barrier"}) {
    input["execution"]["gurobi_method"] = method;
    input["execution"]["threads"] = method=="auto" ? 1 : 2;
    const auto solved = solve(*lease,input);
    auto result = stage_result(*lease,solved,input);
    if (!result.at("prices_valid").get<bool>() || result.at("max_residual").get<double>() > 1e-6) return 4;
    if(expected.size()==0)expected=solved.constraint_duals;
    else if(expected.size()!=solved.constraint_duals.size() ||
        !(expected.array()==solved.constraint_duals.array()).all())return 5;
    // Exercise the production fail-closed price export without a test hook.
    const auto audit=lease->price_consistency;
    lease->price_consistency["passed"]=false;
    const auto rejected=stage_result(*lease,solved,input);
    if(rejected.at("prices_valid").get<bool>())return 6;
    for(const auto& bus:rejected.at("buses"))for(const auto& price:bus.at("lmp_per_mwh"))
      if(!price.is_null())return 7;
    lease->price_consistency=audit;
    std::cout << method << " objective=" << result.at("objective") << " residual=" << result.at("max_residual") << '\n';
    results[method] = std::move(result);
  }
  results["_audit"]={{"full_row_duals",std::vector<double>(expected.data(),expected.data()+expected.size())},
    {"requested_methods_exact",true},{"failed_check_exports_null_prices",true}};
  std::ofstream(argv[2]) << results.dump();
}
