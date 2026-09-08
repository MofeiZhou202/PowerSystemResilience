// Read-only original-matrix diagnosis. No model reduction or solve-policy changes.
// Normal-equation structural proxy: sum_j choose(degree(A[:,j]),2).
// This counts edge multiplicity before ordering, NOT actual factor fill or FLOPs.
#include "../../src/market/southern_market.cpp"
#include <fstream>
#include <iostream>

namespace {
using namespace hacdcpf::market;
struct Stats {
  size_t count{0}, fixed{0}, nnz{0}, max_degree{0}, cross_time{0};
  uint64_t edge_multiplicity{0};
  J json() const {return {{"count",count},{"fixed",fixed},{"nonzeros",nnz},
    {"max_degree",max_degree},{"cross_time_rows",cross_time},
    {"normal_graph_edge_multiplicity_proxy",edge_multiplicity}};}
};
std::string family(const std::string& name) {
  const auto end=name.rfind('/');
  const auto id=end==std::string::npos?end:name.rfind('/',end-1);
  return id==std::string::npos?name:name.substr(0,id);
}
int time_index(const std::string& name) {
  const auto end=name.rfind('/');
  if(end==std::string::npos)return -1;
  const std::string tail=name.substr(end+1);
  if(tail.empty()||!std::all_of(tail.begin(),tail.end(),[](char c){return c>='0'&&c<='9';}))return -1;
  return std::stoi(tail);
}
J inspect(const Build& b) {
  const auto& lp=b.model.linear_part;
  std::map<std::string,Stats> cols,rows;
  std::vector<size_t> degrees(lp.vars.size());
  std::vector<int> times;times.reserve(lp.vars.size());
  for(const auto& v:lp.vars)times.push_back(time_index(v.name));
  J broad=J::array();
  for(const auto* source:{&b.eq,&b.le})for(const auto& row:*source) {
    const auto first=row.name.find('/');
    const auto second=first==std::string::npos?first:row.name.find('/',first+1);
    auto& s=rows[row.name.substr(0,second)];++s.count;
    size_t degree=0;int low=std::numeric_limits<int>::max(),high=-1;
    row.expr.each([&](int c,double a){
      if(a==0||lp.vars[c].lb==lp.vars[c].ub)return;
      ++degree;++degrees[c];
      if(times[c]>=0){low=std::min(low,times[c]);high=std::max(high,times[c]);}
    });
    s.nnz+=degree;s.max_degree=std::max(s.max_degree,degree);
    s.cross_time+=high>low;
    if(degree>=100) broad.push_back({{"name",row.name},{"unfixed_degree",degree},
      {"min_name_time_index",low==std::numeric_limits<int>::max()?J(nullptr):J(low)},
      {"max_name_time_index",high<0?J(nullptr):J(high)}});
  }
  for(size_t c=0;c<lp.vars.size();++c) {
    auto& s=cols[family(lp.vars[c].name)];++s.count;s.fixed+=lp.vars[c].lb==lp.vars[c].ub;
    const auto degree=degrees[c];s.nnz+=degree;s.max_degree=std::max(s.max_degree,degree);
    s.edge_multiplicity+=degree*(degree?degree-1:0)/2;
  }
  J result={{"stage",b.stage},{"columns",lp.vars.size()},{"equalities",b.eq.size()},
    {"inequalities",b.le.size()},{"stored_nonzeros",lp.A.nonZeros()+lp.Aeq.nonZeros()},
    {"scope","original expressions, numerical zeros and authored fixed columns excluded; no solver presolve"},
    {"time_scope","last name suffix, hourly storage mode uses hour index; not an exact temporal partition"},
    {"proxy_scope","edge multiplicity before ordering, not factor fill or operation count"},
    {"broad_rows",std::move(broad)}};
  for(const auto& [name,s]:cols)result["column_families"][name]=s.json();
  for(const auto& [name,s]:rows)result["row_families"][name]=s.json();
  return result;
}
}
int main(int argc,char** argv) {
  if(argc!=4)return 2;
  J input,state;std::ifstream(argv[1])>>input;input=validate_southern_market(input);
  std::ifstream(argv[2])>>state;
  if(state.at("input")!=input)throw std::runtime_error("Snapshot input differs");
  J result;
  {auto lease=build_model(input,"scuc");result["scuc_relaxation"]=inspect(*lease);}
  {auto lease=build_model(input,"lmp",state.at("dispatch").get<std::map<std::string,double>>());
    result["lmp"]=inspect(*lease);}
  std::ofstream output(argv[3]);output<<result.dump(2);
  if(!output.good())throw std::runtime_error("Cannot write structural diagnosis");
  std::cout<<"Original SCUC and LMP structural diagnosis written\n";
}
