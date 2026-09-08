// Cross-process, identical-dispatch full-dual audit without a production hook.
#include "../../src/market/southern_market.cpp"
#include <fstream>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=4&&argc!=5)return 2;
  using namespace hacdcpf::market;
  J input;std::ifstream(argv[1])>>input;input=validate_southern_market(input);
  J state;
  std::ifstream saved(argv[2]);
  if(saved.good()) {
    saved>>state;
    if(state.at("input")!=input)throw std::runtime_error("Pricing snapshot input differs");
  } else {
    auto lease=build_model(input,"scuc");
    const auto solved=solve(*lease,input);
    if(!stage_result(*lease,solved,input).at("feasible").get<bool>())return 3;
    auto dispatch=solution_map(*lease,solved);
    if(!derive_sced(*lease,input,solved,dispatch,{})||primal_residual(*lease,solved.x,input)>1e-6)return 4;
    state={{"input",input},{"dispatch",lease->fixed}};
    std::ofstream(argv[2])<<state.dump();
  }
  auto dispatch=state.at("dispatch").get<std::map<std::string,double>>();
  BuildLease lease;
  if(argc==5) {
    if(std::string(argv[4])!="derive")return 2;
    lease=build_model(input,"sced",dispatch);
    if(!derive_lmp(*lease,input,dispatch,{}))return 8;
    auto original=assemble_model(input,"lmp",lease->fixed,{},true,true);
    compare_assembly(*lease,*original);
  } else lease=build_model(input,"lmp",std::move(dispatch));
  const auto solved=solve(*lease,input);
  const auto result=stage_result(*lease,solved,input);
  if(!result.at("prices_valid").get<bool>())return 5;
  const std::string dual_path=std::string(argv[3])+".duals";
  std::ofstream binary(dual_path,std::ios::binary);
  binary.write(reinterpret_cast<const char*>(solved.constraint_duals.data()),solved.constraint_duals.size()*sizeof(double));
  if(!binary.good())throw std::runtime_error("Cannot write complete dual vector");
  lease->price_consistency["passed"]=false;
  const auto failed=stage_result(*lease,solved,input);
  if(failed.at("prices_valid").get<bool>())return 6;
  for(const auto& bus:failed.at("buses"))for(const auto& price:bus.at("lmp_per_mwh"))if(!price.is_null())return 7;
  J report={{"price_consistency",result.at("price_consistency")},{"objective",result.at("objective")},
    {"assembly",argc==5?"derived_with_exact_original_comparison":"original"},
    {"max_residual",result.at("max_residual")},{"dual_rows",solved.constraint_duals.size()},
    {"dual_file",dual_path},{"failed_check_exports_null_prices",true}};
  std::ofstream(argv[3])<<report.dump(2);
  std::cout<<report.dump(2)<<std::endl;
}
