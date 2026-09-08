// Diagnostic only: full original LP relaxation and fixed-integer repair.
// Rationale and predeclared numerical/performance gates: performance.md.
#include "../../src/market/southern_market.cpp"
#include <fstream>
#include <iostream>


int main(int argc,char** argv) {
  if(argc!=3)return 2;
  using namespace hacdcpf::market;
  J input;std::ifstream(argv[1])>>input;input=validate_southern_market(input);
  input["execution"].erase("network_bounds");
  const auto started=std::chrono::steady_clock::now();
  const auto wall=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
  J evidence;
  const auto save=[&]{evidence["wall_sec"]=wall();std::ofstream(argv[2])<<evidence.dump(2);};
  auto lease=build_model(input,"scuc");auto& b=*lease;
  evidence["assembly_sec"]=wall();save();
  hacdcpf::engine::GurobiOptions options;options.method=2;options.crossover=0;options.threads=8;options.time_limit_sec=180;
  const double cert_start=wall();const auto box=certificate_box(b,input);
  evidence["box_sec"]=wall()-cert_start;evidence["box_available"]=box.has_value();save();
  if(!box)return 6;
  auto relaxed=hacdcpf::engine::GurobiAdapter(options).solve_relaxation_lp(b.model.linear_part,.005);
  const auto lower_bound=detail::box_dual_lower_bound(b.model.linear_part,relaxed.constraint_duals,box->first,box->second);
  evidence["relaxation"]={{"status",relaxed.stats.status},{"runtime_sec",relaxed.stats.runtime_sec},
    {"objective",relaxed.stats.objective},{"bound",lower_bound?J(*lower_bound):J(nullptr)},
    {"dual_violation",relaxed.stats.dual_feas},{"primal_violation",relaxed.stats.primal_feas},
    {"fractional",J::array()}};
  std::cout<<"relaxation "<<evidence["relaxation"].dump()<<std::endl;save();
  if(!relaxed.stats.success || relaxed.stats.status!="Optimal")return 3;
  std::vector<std::tuple<int,double,double>> bounds;
  for(int c:b.model.binary_idx) {
    const double value=relaxed.x[c];auto& var=b.model.linear_part.vars[c];
    if(std::abs(value-std::round(value))>1e-6)evidence["relaxation"]["fractional"].push_back({{"name",var.name},{"value",value}});
    bounds.push_back({c,var.lb,var.ub});var.lb=var.ub=std::clamp(std::round(value),var.lb,var.ub);
  }
  save();
  auto repaired=hacdcpf::engine::GurobiAdapter(options).solve_lp(b.model.linear_part);
  for(const auto& [c,lo,hi]:bounds){b.model.linear_part.vars[c].lb=lo;b.model.linear_part.vars[c].ub=hi;}
  const double residual=primal_residual(b,repaired.x,input);
  const double upper=repaired.stats.objective,lower=lower_bound.value_or(-std::numeric_limits<double>::infinity());
  const double gap=std::abs(upper-lower)/std::max(1e-10,std::abs(upper));
  evidence["repair"]={{"status",repaired.stats.status},{"runtime_sec",repaired.stats.runtime_sec},
    {"objective",upper},{"max_residual",residual},{"gap",gap}};save();
  std::cout<<"repair "<<evidence["repair"].dump()<<std::endl;
  if(!repaired.stats.success||residual>1e-6||!std::isfinite(lower)||gap>num(input.at("execution"),"mip_gap"))return 4;
  auto dispatch=solution_map(b,repaired);
  auto sced_lease=build_model(input,"sced",dispatch,{});auto& sced=*sced_lease;
  const auto& old=b.model.linear_part;const auto& next=sced.model.linear_part;
  const auto same_matrix=[](const auto& a,const auto& z){return a.rows()==z.rows()&&a.cols()==z.cols()&&
    a.nonZeros()==z.nonZeros()&&std::equal(a.outerIndexPtr(),a.outerIndexPtr()+a.outerSize()+1,z.outerIndexPtr())&&
    std::equal(a.innerIndexPtr(),a.innerIndexPtr()+a.nonZeros(),z.innerIndexPtr())&&
    std::equal(a.valuePtr(),a.valuePtr()+a.nonZeros(),z.valuePtr());};
  const bool matrices=same_matrix(old.A,next.A)&&same_matrix(old.Aeq,next.Aeq)&&
    old.b.size()==next.b.size()&&(old.b.array()==next.b.array()).all()&&
    old.beq.size()==next.beq.size()&&(old.beq.array()==next.beq.array()).all()&&
    old.c.size()==next.c.size()&&(old.c.array()==next.c.array()).all();
  bool subset=old.vars.size()==next.vars.size();
  for(size_t c=0;subset&&c<old.vars.size();++c)subset=old.vars[c].name==next.vars[c].name&&
    next.vars[c].lb>=old.vars[c].lb&&next.vars[c].ub<=old.vars[c].ub;
  Eigen::VectorXd candidate(next.vars.size());for(size_t c=0;c<next.vars.size();++c)candidate[c]=dispatch.at(next.vars[c].name);
  evidence["sced"]={{"same_matrices_cost_rhs",matrices},{"bound_subset",subset},
    {"candidate_residual",primal_residual(sced,candidate,input)}};save();
  std::cout<<"sced "<<evidence["sced"].dump()<<std::endl;
  if(!matrices||!subset||primal_residual(sced,candidate,input)>1e-6)return 5;
  auto pricing_lease=build_model(input,"lmp",dispatch,{});auto& pricing=*pricing_lease;
  for(int method:{2,2}) {
    auto pricing_options=options;pricing_options.method=method;
    const auto price=hacdcpf::engine::GurobiAdapter(pricing_options).solve_lp(pricing.model.linear_part);
    evidence["pricing"][std::to_string(method)]={{"status",price.stats.status},{"runtime_sec",price.stats.runtime_sec},
      {"objective",price.stats.objective},{"max_residual",primal_residual(pricing,price.x,input)}};save();
    std::cout<<"pricing "<<method<<" "<<evidence["pricing"][std::to_string(method)].dump()<<std::endl;
  }
}
