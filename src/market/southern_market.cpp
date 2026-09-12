#include "hacdcpf/market/southern_market.hpp"
#include "southern_solver_status.hpp"
#include "southern_price_consistency.hpp"
#include "southern_bound_certificate.hpp"
#include "yunnan_rules_workflow.hpp"
#include "southern_real_time.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <future>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <charconv>
#include <utility>
#include <vector>
#include <Eigen/Sparse>
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/engine/engine.hpp"

namespace hacdcpf::market {
namespace {
using J = nlohmann::json;
constexpr int T = 98;
constexpr double inf = 1e20;
// Acceptance gate fixed in southern_execution_contract.md, in original units.
constexpr double tolerance = 1e-6;
thread_local bool reference_terms = false;
struct AssemblyScope {
  bool previous{reference_terms};
  explicit AssemblyScope(bool reference) { reference_terms = reference; }
  ~AssemblyScope() { reference_terms = previous; }
};
struct Expr {
  using Flat = std::vector<std::pair<int,double>>;
  std::variant<Flat,std::map<int,double>> terms;
  double constant{0};
  Expr() { if (reference_terms) terms.emplace<1>(); }
  explicit Expr(double c) : Expr() { constant = c; }
  template<class F> void each(F&& f) const {
    std::visit([&](const auto& items) { for (const auto& [i,v] : items) f(i,v); },terms);
  }
  Expr& add(int index, double coefficient = 1) {
    // Preserve map order and per-column accumulation exactly; assembly proof in performance.md.
    if (auto* flat = std::get_if<Flat>(&terms)) {
      const auto pos = std::lower_bound(flat->begin(),flat->end(),index,
        [](const auto& item,int i) { return item.first<i; });
      if (pos!=flat->end() && pos->first==index) pos->second += coefficient;
      else flat->insert(pos,{index,0.0+coefficient});
    } else std::get<1>(terms)[index] += coefficient;
    return *this;
  }
  Expr& add(const Expr& rhs, double scale = 1) {
    constant += scale*rhs.constant;
    rhs.each([&](int i,double v) { add(i,scale*v); });
    return *this;
  }
};
Expr variable(int id, double coefficient = 1) { Expr e; return e.add(id, coefficient); }
double value(const Expr& e, const Eigen::VectorXd& x) {
  double v = e.constant;
  e.each([&](int i,double c) { v += c*x[i]; });
  return v;
}
double num(const J& row, const char* key) { return row.at(key).get<double>(); }
double at(const J& row, const char* key, int t) { return row.at(key)[t].get<double>(); }
std::string key(const char* family, int id, int t) {
  return std::string(family) + "/" + std::to_string(id) + "/" + std::to_string(t);
}
struct Row { Expr expr; double rhs; std::string name; double solver_scale{1}; };
struct SecurityCut {
  std::string name;
  std::map<std::string, double> coefficients;
  double rhs{0};
};
struct Build {
  engine::MIPModel model;
  std::vector<double> costs;
  std::vector<std::string> cost_categories;
  std::vector<int> audited_binary_columns;
  std::map<std::string, int> columns;
  std::vector<Row> eq, le;
  std::map<std::string, int> balance_rows;
  std::map<std::pair<int,int>, std::pair<int,int>> reserve_rows;
  std::map<std::string, Expr> metrics;
  double constant_cost{0};
  std::map<std::string, double> fixed;
  std::string stage;
  std::set<int> compact_units;
  std::set<int> compact_storage;
  std::set<int> projected_commitment;
  bool compact{false};
  bool reservoir_scaling{false};
  bool prune_rows{false};
  std::vector<bool> redundant_rows;
  std::vector<size_t> submitted_rows;
  J size_report;
  J native_diagnostics = nullptr;
  J solver_timing = nullptr;
  J price_consistency = nullptr;
  J gap_certificate = nullptr;
  J primal_start{{"status","not_requested"},{"runtime_sec",0.0},{"accepted",false}};
  bool reference{false}, layout_match{false}, column_layout_match{false}, reusable{false};
  std::map<std::string,std::map<std::string,int>::const_iterator> column_hints;
  size_t eq_count{0}, le_count{0}, reused_matrices{0};
  std::map<std::string,std::unordered_map<int,std::vector<int>>,std::less<>> column_slots;
  struct Pattern { std::vector<size_t> row_offsets; std::vector<std::pair<int,int>> slots; };
  Pattern le_pattern, eq_pattern;
  J assembly_template;
  void prepare(bool old_path) {
    reference=old_path; column_layout_match=layout_match=reusable; reusable=false; eq_count=le_count=reused_matrices=0;
    costs.clear(); cost_categories.clear(); audited_binary_columns.clear();
    model.binary_idx.clear(); model.initial_solution.resize(0);
    balance_rows.clear(); reserve_rows.clear(); metrics.clear(); fixed.clear(); constant_cost=0;
    compact_units.clear(); compact_storage.clear(); projected_commitment.clear();
    redundant_rows.clear(); submitted_rows.clear();
    native_diagnostics=nullptr;
    solver_timing=nullptr;
    price_consistency=nullptr;
    gap_certificate=nullptr;
    primal_start={{"status","not_requested"},{"runtime_sec",0.0},{"accepted",false}};
  }
  void register_column(const std::string& name,int col) {
    if (reference) {columns.emplace(name,col);return;}
    const size_t last=name.rfind('/'), first=last==std::string::npos ? last : name.rfind('/',last-1);
    if (first==std::string::npos) {columns.emplace(name,col);return;}
    const auto family=name.substr(0,first);
    // Consecutive family slots are usually adjacent in lexicographic order.
    // std::map verifies the hint and falls back at numeric-name wrap points;
    // performance.md, 2000-node index/export cost model.
    auto [hint,inserted]=column_hints.try_emplace(family,columns.end());
    hint->second=columns.emplace_hint(inserted?columns.end():std::next(hint->second),name,col);
    int id=0,t=0;
    const auto a=std::from_chars(name.data()+first+1,name.data()+last,id);
    const auto z=std::from_chars(name.data()+last+1,name.data()+name.size(),t);
    if (a.ec!=std::errc{} || a.ptr!=name.data()+last || z.ec!=std::errc{} || z.ptr!=name.data()+name.size() || t<0 || t>=98) return;
    auto& slots=column_slots[family][id];
    if(slots.size()<=static_cast<size_t>(t))slots.resize(t+1,-1);
    slots[t]=col;
  }
  int var(std::string name, double lo, double hi, double cost = 0,
          const char* category = "energy", bool binary = false, bool freeze = false,
          bool implied_integer = false) {
    if (freeze) {
      const auto found = fixed.find(name);
      if (found == fixed.end()) throw std::logic_error("missing preceding-stage variable: " + name);
      lo = hi = binary ? std::round(found->second) : found->second;
    }
    if (lo > hi + tolerance) throw std::invalid_argument(name + ": incompatible effective bounds");
    const int id = static_cast<int>(costs.size());
    const auto type=binary && !implied_integer ? engine::VarType::Binary : engine::VarType::Continuous;
    auto& vars=model.linear_part.vars;
    const bool same=id<static_cast<int>(vars.size()) && vars[id].name==name;
    if (!same && column_layout_match) {
      column_layout_match=layout_match=false; columns.clear(); column_slots.clear(); column_hints.clear();
      for(int i=0;i<id;++i)register_column(vars[i].name,i);
    }
    if (!column_layout_match) register_column(name,id);
    if (same) { if(vars[id].type!=type)layout_match=false; vars[id].lb=lo;vars[id].ub=hi;vars[id].type=type; }
    else if(id<static_cast<int>(vars.size()))vars[id]={type,lo,hi,std::move(name)};
    else vars.push_back({type,lo,hi,std::move(name)});
    costs.push_back(cost); cost_categories.emplace_back(category);
    if (binary) audited_binary_columns.push_back(id);
    if (binary && !freeze && !implied_integer) model.binary_idx.push_back(id);
    return id;
  }
  int col(const char* family, int id, int t) const {
    if(reference)return columns.at(key(family,id,t));
    const auto found=column_slots.find(family);
    if(found==column_slots.end())throw std::logic_error("missing column family");
    const auto& slots=found->second.at(id);
    const int col=slots.at(t);
    if(col<0 || static_cast<size_t>(col)>=costs.size())throw std::logic_error("missing column time slot");
    return col;
  }
  Expr expr(const char* family, int id, int t) const { return variable(col(family, id, t)); }
  int equal(std::string name, Expr e, double rhs = 0) {
    put_row(eq,eq_count++,std::move(name),std::move(e),rhs); return static_cast<int>(eq_count)-1;
  }
  void put_row(std::vector<Row>& rows,size_t pos,std::string name,Expr e,double rhs) {
    if(pos<rows.size()) {
      if(rows[pos].name!=name)layout_match=false;
      rows[pos]={std::move(e),rhs,std::move(name)};
    } else {layout_match=false;rows.push_back({std::move(e),rhs,std::move(name)});}
  }
  void upper(std::string name, Expr e, double rhs = 0) { put_row(le,le_count++,std::move(name),std::move(e),rhs); }
  void range(const std::string& name, const Expr& e, double lo, double hi) {
    upper(name + "/upper", e, hi); Expr negative; negative.add(e, -1); upper(name + "/lower", negative, -lo);
  }
  bool bound_redundant(const Row& row) const {
    // Box support certificate with outward rounding; execution contract,
    // "Certified inequality omission". No feasibility tolerance is used.
    double upper = row.expr.constant;
    if (!std::isfinite(upper)) return false;
    bool bounded=true;
    row.expr.each([&](int col,double a) {
      if (a == 0 || !bounded) return;
      const auto& var = model.linear_part.vars[col];
      const double bound = a > 0 ? var.ub : var.lb;
      if (!std::isfinite(bound) || std::abs(bound) >= inf) {bounded=false;return;}
      if (bound == 0) return;
      const double term = std::nextafter(a*bound,std::numeric_limits<double>::infinity());
      upper = std::nextafter(upper+term,std::numeric_limits<double>::infinity());
      if (!std::isfinite(upper)) bounded=false;
    });
    return bounded && upper <= row.rhs;
  }
  void finish() {
    auto& lp = model.linear_part;
    if(eq_count!=eq.size() || le_count!=le.size() || costs.size()!=lp.vars.size())layout_match=false;
    eq.resize(eq_count);le.resize(le_count);lp.vars.resize(costs.size());
    if(columns.size()!=costs.size()) {
      columns.clear();column_slots.clear();column_hints.clear();
      for(size_t i=0;i<lp.vars.size();++i)register_column(lp.vars[i].name,static_cast<int>(i));
    }
    lp.c = Eigen::Map<Eigen::VectorXd>(costs.data(), static_cast<Eigen::Index>(costs.size()));
    lp.sense = engine::Sense::Minimize;
    redundant_rows.reserve(le.size());
    for (size_t r=0;r<le.size();++r) {
      redundant_rows.push_back(bound_redundant(le[r]));
      if (!prune_rows || !redundant_rows.back()) submitted_rows.push_back(r);
    }
    const auto matrix = [&](const std::vector<Row>& rows, auto& A, auto& b, bool inequality,Pattern& pattern) {
      const size_t count = inequality ? submitted_rows.size() : rows.size();
      b.resize(static_cast<Eigen::Index>(count));
      bool matching=!reference && layout_match && A.rows()==static_cast<int>(count) && A.cols()==static_cast<int>(costs.size()) && pattern.row_offsets.size()==count+1;
      size_t offset=0;
      for(size_t index=0;index<count && matching;++index) {
        const auto& row=rows[inequality ? submitted_rows[index] : index];
        if(pattern.row_offsets[index]!=offset) {matching=false;break;}
        row.expr.each([&](int c,double v) {
          if(v==0)return;
          if(offset>=pattern.slots.size() || pattern.slots[offset].first!=c)matching=false;
          ++offset;
        });
        if(pattern.row_offsets[index+1]!=offset)matching=false;
      }
      if(matching && offset==pattern.slots.size()) {
        offset=0;
        for(size_t index=0;index<count;++index) {
          const auto& row=rows[inequality ? submitted_rows[index] : index];
          b[index]=(row.rhs-row.expr.constant)*row.solver_scale;
          row.expr.each([&](int,double v) {if(v!=0)A.valuePtr()[pattern.slots[offset++].second]=v*row.solver_scale;});
        }
        ++reused_matrices;return;
      }
      std::vector<Eigen::Triplet<double>> entries;
      if(!reference)entries.reserve(pattern.slots.size());
      for (size_t index = 0; index < count; ++index) {
        const size_t r = inequality ? submitted_rows[index] : index;
        b[static_cast<Eigen::Index>(index)] = (rows[r].rhs - rows[r].expr.constant)*rows[r].solver_scale;
        rows[r].expr.each([&](int c,double v) {if(v!=0)entries.emplace_back(static_cast<int>(index),c,v*rows[r].solver_scale);});
      }
      A.resize(static_cast<int>(count), static_cast<int>(costs.size()));
      A.setFromTriplets(entries.begin(), entries.end());
      if(!reference) {
        pattern.row_offsets.assign(count+1,0);pattern.slots.resize(entries.size());
        for(const auto& entry:entries)++pattern.row_offsets[entry.row()+1];
        std::partial_sum(pattern.row_offsets.begin(),pattern.row_offsets.end(),pattern.row_offsets.begin());
        auto next=pattern.row_offsets;
        for(int col=0;col<A.outerSize();++col)for(int slot=A.outerIndexPtr()[col];slot<A.outerIndexPtr()[col+1];++slot)
          pattern.slots[next[A.innerIndexPtr()[slot]]++]={col,slot};
      }
    };
    matrix(le, lp.A, lp.b, true,le_pattern); matrix(eq, lp.Aeq, lp.beq, false,eq_pattern);
    assembly_template={{"mode",reference ? "reference" : "cached"},{"reused_matrices",reused_matrices},
      {"layout_reused",layout_match},{"matrix_comparison","not_requested"}};
    reusable=true;
  }
};

struct NetworkBounds {
  bool certified{false};
  std::map<int,double> angle, flow, power;
};

NetworkBounds certified_network_bounds(const Build& b, const J& j, int t) {
  // Cut-set maximum principle and outward-rounded path bounds; derivation and
  // excluded nonpositive/phase-shifting networks: performance.md, 2000-node goal.
  NetworkBounds out;
  std::map<int,int> parent;
  std::map<int,std::vector<std::pair<int,double>>> graph;
  for(const auto& node:j.at("buses"))parent[node.at("id")]=node.at("id");
  const auto root=[&](int n){while(parent.at(n)!=n)n=parent.at(n);return n;};
  const double infinity=std::numeric_limits<double>::infinity();
  const auto up=[&](double v){return std::nextafter(v,infinity);};
  for(const auto& line:j.at("branches"))if(at(line,"available",t)!=0) {
    const double susceptance=num(j,"base_mva")/(num(line,"x_pu")*num(line,"tap"));
    if(!(susceptance>0) || !std::isfinite(susceptance) || num(line,"shift_deg")!=0)return out;
    const int from=line.at("from_bus"),to=line.at("to_bus");
    const double distance=up(1/susceptance);
    graph[from].push_back({to,distance});graph[to].push_back({from,distance});
    parent[root(to)]=root(from);
  }
  std::map<int,double> positive;
  for(const auto& node:j.at("buses")) {
    const int id=node.at("id");const auto& e=b.eq.at(b.balance_rows.at(key("bus",id,t))).expr;
    double upper=e.constant;bool finite=true;
    e.each([&](int col,double a){
      if(a==0)return;
      const auto& var=b.model.linear_part.vars[col];
      if(var.name.starts_with("flow/"))return;
      const double bound=a>0?var.ub:var.lb;
      if(!std::isfinite(bound)||std::abs(bound)>=inf){finite=false;return;}
      if(bound!=0)upper=up(upper+up(a*bound));
    });
    if(!finite||!std::isfinite(upper))return out;
    auto& sum=positive[root(id)];if(upper>0)sum=up(sum+upper);
  }
  std::map<int,double> distance;
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  for(const auto& [id,p]:parent){distance[id]=id==p?0:infinity;if(id==p)queue.push({0,id});}
  while(!queue.empty()) {
    const auto [d,node]=queue.top();queue.pop();if(d!=distance.at(node))continue;
    for(const auto& [next,weight]:graph[node]) {
      const double candidate=up(d+weight);
      if(candidate<distance.at(next)){distance[next]=candidate;queue.push({candidate,next});}
    }
  }
  for(const auto& [id,d]:distance) {
    const double power=positive.at(root(id));
    const double bound=power==0||d==0?0:up(power*d);
    if(!std::isfinite(bound)||bound>=inf)return NetworkBounds{};
    out.angle[id]=bound;
    out.power[id]=power;
  }
  for(const auto& line:j.at("branches"))out.flow[line.at("id")]=
    at(line,"available",t)==0?0:positive.at(root(line.at("from_bus")));
  out.certified=true;return out;
}

std::optional<std::pair<Eigen::VectorXd,Eigen::VectorXd>> certificate_box(const Build& b,const J& input) {
  // A finite optimal-representative box, never passed to the solver. Proof and
  // restrictions: performance.md, Lagrangian box lower bound.
  if(input.contains("_rt"))return std::nullopt;
  const auto& lp=b.model.linear_part;
  Eigen::VectorXd lower(lp.c.size()),upper(lp.c.size());
  for(size_t c=0;c<lp.vars.size();++c){lower[c]=lp.vars[c].lb;upper[c]=lp.vars[c].ub;}
  std::vector<NetworkBounds> network(T);
  for(int t=0;t<T;++t){network[t]=certified_network_bounds(b,input,t);if(!network[t].certified)return std::nullopt;}
  const auto up=[](double x){return std::nextafter(x,std::numeric_limits<double>::infinity());};
  const auto set=[&](const char* family,int id,int t,double lo,double hi){
    const int c=b.col(family,id,t);lower[c]=std::max(lower[c],lo);upper[c]=std::min(upper[c],hi);
  };
  std::vector<std::string> slack_owner(lp.c.size());
  const auto slack=[&](const char* family,const char* row_family,int id,int t,double power,double lo,double hi){
    const auto plus=std::string(family)+"_slack_plus",minus=std::string(family)+"_slack_minus";
    const int p=b.col(plus.c_str(),id,t),n=b.col(minus.c_str(),id,t);
    if(lp.c[p]<0||lp.c[n]<0||lp.vars[p].lb!=0||lp.vars[n].lb!=0)return false;
    slack_owner[p]=slack_owner[n]=key(row_family,id,t);
    upper[p]=std::min(upper[p],std::max(0.0,up(power-hi)));
    upper[n]=std::min(upper[n],std::max(0.0,up(lo+power)));
    return true;
  };
  for(int t=0;t<T;++t) {
    for(const auto& node:input.at("buses")) {
      const int id=node.at("id");const double angle=network[t].angle.at(id);
      set("theta",id,t,-angle,angle);
      if(b.columns.contains(key("surplus",id,t)))set("surplus",id,t,0,network[t].power.at(id));
    }
    for(const auto& line:input.at("branches")) {
      const int id=line.at("id");const double power=network[t].flow.at(id);
      set("flow",id,t,-power,power);
      if(!slack("line","2.6.3.14/line",id,t,power,at(line,"min_mw",t),at(line,"max_mw",t)))return std::nullopt;
    }
    for(const auto& section:input.at("sections")) {
      double power=0;
      for(const auto& member:section.at("members"))power=up(power+up(std::abs(num(member,"coefficient"))*network[t].flow.at(member.at("branch"))));
      if(!slack("section","2.6.3.15/section",section.at("id"),t,power,at(section,"min_mw",t),at(section,"max_mw",t)))return std::nullopt;
    }
  }
  bool valid=true;
  for(const auto* rows:{&b.eq,&b.le})for(const auto& row:*rows)row.expr.each([&](int c,double a){
    if(a==0)return;
    const auto& owner=slack_owner[c];
    if(!owner.empty()&&row.name!=owner+"/upper"&&row.name!=owner+"/lower")valid=false;
  });
  for(Eigen::Index c=0;c<lp.c.size();++c)if(!std::isfinite(lower[c])||!std::isfinite(upper[c])||
      lower[c]<=-inf||upper[c]>=inf||lower[c]>upper[c]) {
    return std::nullopt;
  }
  if(!valid)return std::nullopt;
  return std::pair{std::move(lower),std::move(upper)};
}

J model_size_report(const Build& b, const J& input) {
  // Box support function max(a*x+c) = c + sum(a>=0 ? a*ub : a*lb).
  // Outward rounding and the scope of this read-only certificate are documented
  // in southern_execution_contract.md, "Model size and bound redundancy audit".
  struct Counts { size_t columns{0}, binaries{0}, fixed{0}, equalities{0}, inequalities{0}, nonzeros{0}, redundant{0}; };
  std::map<std::string, Counts> variables, rules;
  std::vector<bool> binaries(b.costs.size(),false);
  for (int col : b.model.binary_idx) binaries[col] = true;
  for (size_t i = 0; i < b.costs.size(); ++i) {
    const auto& var = b.model.linear_part.vars[i];
    auto& count = variables[var.name.substr(0, var.name.find('/'))];
    ++count.columns; count.binaries += binaries[i];
    count.fixed += var.lb == var.ub;
  }
  const auto count_rows = [&](const std::vector<Row>& rows, bool equality) {
    for (size_t r=0;r<rows.size();++r) {
      const auto& row=rows[r];
      auto& count = rules[row.name.substr(0, row.name.find('/'))];
      if (equality) ++count.equalities; else ++count.inequalities;
      row.expr.each([&](int,double a) {if(a!=0)++count.nonzeros;});
      if (!equality && b.redundant_rows[r]) ++count.redundant;
    }
  };
  count_rows(b.eq, true); count_rows(b.le, false);
  J out = {{"scope", "assembled-before-solver-presolve"}, {"constraints_removed", b.le.size()-b.submitted_rows.size()},
    {"variables", b.costs.size()}, {"binary_variables", b.model.binary_idx.size()},
    {"equalities", b.eq.size()}, {"inequalities", b.le.size()},
    {"nonzeros", 0}, {"submitted_inequalities",b.submitted_rows.size()},
    {"submitted_equalities",b.eq.size()},
    {"submitted_nonzeros",b.model.linear_part.A.nonZeros()+b.model.linear_part.Aeq.nonZeros()},
    {"row_presolve",b.prune_rows ? "enabled" : "none"},
    {"variable_families", J::object()}, {"constraint_families", J::object()},
    {"commitment_by_kind", J::object()}, {"bound_redundant_inequalities", 0}};
  for (const auto& [name, c] : variables)
    out["variable_families"][name] = {{"variables",c.columns},{"binary_variables",c.binaries},{"fixed_variables",c.fixed}};
  size_t redundant = 0;
  for (const auto& [name, c] : rules) {
    out["constraint_families"][name] = {{"equalities",c.equalities},{"inequalities",c.inequalities},
      {"nonzeros",c.nonzeros},{"bound_redundant_inequalities",c.redundant}};
    redundant += c.redundant;
    out["nonzeros"] = out["nonzeros"].get<size_t>()+c.nonzeros;
  }
  out["bound_redundant_inequalities"] = redundant;
  for (const auto& g : input.at("generators")) {
    const std::string kind = g.at("kind");
    if (!out["commitment_by_kind"].contains(kind))
      out["commitment_by_kind"][kind] = {{"units",0},{"binary_variables",0},{"fixed_variables",0}};
    auto& c = out["commitment_by_kind"][kind];
    c["units"] = c["units"].get<int>()+1;
    for (int t=0;t<static_cast<int>(input.at("periods").size());++t) {
      const int col = b.col("u",g.at("id"),t);
      c["binary_variables"] = c["binary_variables"].get<size_t>()+binaries[col];
      const auto& var = b.model.linear_part.vars[col];
      c["fixed_variables"] = c["fixed_variables"].get<size_t>()+(var.lb == var.ub);
    }
  }
  return out;
}

std::map<std::string, double> solution_map(const Build& b, const engine::SolveResult& r) {
  std::map<std::string, double> result;
  // Source columns are sorted by name; an end hint keeps the same ordering
  // and values with linear insertion cost. performance.md, output overhead.
  for (const auto& [name, col] : b.columns) result.emplace_hint(result.end(),name,r.x[col]);
  return result;
}

// Exclusive leases retain only a bounded number of small-market templates.
// Numeric values are recomputed; structural admission is exact, not hash-based.
struct BuildPool {
  std::mutex mutex;
  std::array<std::vector<std::unique_ptr<Build>>,3> stages;
};
BuildPool& build_pool() { static BuildPool pool; return pool; }
struct ReturnBuild {
  int slot{-1};
  void operator()(Build* pointer) const noexcept {
    std::unique_ptr<Build> model(pointer);
    // Bound retained storage as well as asset count; cache budget in performance.md.
    if(slot<0 || !model || !model->reusable || model->costs.size()>250000 ||
       model->le.size()+model->eq.size()>400000 ||
       model->le_pattern.slots.size()+model->eq_pattern.slots.size()>2000000)return;
    try {
      auto& pool=build_pool();std::lock_guard lock(pool.mutex);
      if(pool.stages[slot].size()<2)pool.stages[slot].push_back(std::move(model));
    } catch (...) { /* Cache allocation failure must not affect a completed solve. */ }
  }
};
using BuildLease=std::unique_ptr<Build,ReturnBuild>;
BuildLease acquire_build(const J& j,const std::string& stage,bool reference,bool independent) {
  const bool eligible=!reference && independent && !j.contains("_rt") && !j.contains("_yunnan") &&
    j.at("buses").size()<=118 && j.at("generators").size()<=128 &&
    j.at("storage").size()<=16 && j.at("reservoirs").size()<=24;
  const int slot=eligible ? (stage=="scuc" ? 0 : stage=="sced" ? 1 : 2) : -1;
  std::unique_ptr<Build> model;
  if(slot>=0) {
    auto& pool=build_pool();std::lock_guard lock(pool.mutex);
    if(!pool.stages[slot].empty()){model=std::move(pool.stages[slot].back());pool.stages[slot].pop_back();}
  }
  if(!model)model=std::make_unique<Build>();
  model->prepare(reference);
  return BuildLease(model.release(),ReturnBuild{slot});
}

BuildLease assemble_model(const J& j, const std::string& stage,
                  std::map<std::string, double> previous = {},
                  const std::vector<SecurityCut>& cuts = {},
                  bool independent_projections_allowed = true, bool reference = false) {
  AssemblyScope scope(reference);
  auto lease=acquire_build(j,stage,reference,independent_projections_allowed);
  auto& b=*lease; b.stage = stage; b.fixed = std::move(previous);
  const int T=static_cast<int>(j.at("periods").size());
  const bool realtime=j.contains("_rt");
  const int energy_points=realtime?T:96;
  const bool uc = stage == "scuc", pricing = stage == "lmp";
  const bool ancillary = !uc && j.contains("_yunnan");
  std::map<int, const J*> agc_members;
  if (ancillary) for (const auto& a : j.at("_yunnan").at("config").at("agc_units"))
    for (const auto& member : a.at("members")) agc_members[member.at("generator_id").get<int>()] = &member;
  const auto independent_award=[&](const char* mode,int id,int t)->std::pair<const J*,double> {
    if(ancillary&&j.at("_yunnan").at("config").contains("independent_units"))
      for(const auto& a:j.at("_yunnan").at("config").at("independent_units"))if(a.at("mode")==mode&&a.at("resource_id")==id)
        return {&a,yunnan::required_capacity(j.at("_yunnan"),a.at("id"),t)};
    return {nullptr,0};
  };
  const auto& exec = j.at("execution");
  b.compact = exec.value("formulation",std::string("compact")) == "compact";
  b.prune_rows = b.compact && !pricing && exec.value("row_presolve",std::string("none")) == "enabled";
  b.reservoir_scaling = b.compact && exec.value("reservoir_scaling",std::string("auto")) == "auto";
  const auto& penalties = exec.at(pricing ? "pricing_penalties" : "penalties");
  std::vector<double> weights(T),durations(T),minutes(T);
  for(int t=0;t<T;++t){const auto& period=j.at("periods")[t];
    weights[t]=num(period,"weight_hr");durations[t]=num(period,"duration_hr");minutes[t]=num(period,"start_minute");}
  const auto w = [&](int t) { return weights[t]; };
  const auto dt = [&](int t) { return durations[t]; };
  const auto minute = [&](int t) { return minutes[t]; };
  const double max_time = minute(T-1) + 1000001;
  std::map<int, int> bus_area;
  for (const auto& node : j.at("buses")) bus_area[node.at("id")] = node.at("area");
  std::map<int, const J*> generators;
  for (const auto& g : j.at("generators")) generators[g.at("id")] = &g;
  std::map<std::pair<int, int>, Expr> injection, headroom, footroom, primary;
  for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t)
    injection[{node.at("id"), t}].constant = -at(node, "load_mw", t)-node.value("gs_mw", 0.0);
  for (const auto& e : j.at("external_schedules")) for (int t = 0; t < T; ++t)
    injection[{e.at("bus"), t}].constant += at(e, "power_mw", t);
  // Research slack balance; see southern_execution_contract.md rolling rationale.
  const bool diagnostic = exec.value("balance_policy", std::string("strict")) == "diagnostic";
  if (diagnostic) for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t) {
    const int id = node.at("id");
    const double penalty = exec.value("balance_penalty_per_mwh", 100000.0)*w(t);
    injection[{id,t}].add(b.var(key("deficit", id, t), 0, at(node, "load_mw", t), penalty, "diagnostic_imbalance"));
    injection[{id,t}].add(b.var(key("surplus", id, t), 0, inf, penalty, "diagnostic_imbalance"), -1);
  }

  for (const auto& g : j.at("generators")) {
    const int id = g.at("id"), bus = g.at("bus"), area = bus_area.at(bus);
    // Exact projection and event hull: southern_execution_contract.md,
    // "2000-bus exact formulation experiment" (2.6.3.13 derivation).
    const bool compact_class = b.compact && g.at("startup_cost")[0] == g.at("startup_cost")[1]
      && g.at("startup_cost")[1] == g.at("startup_cost")[2]
      && std::all_of(g.at("startup_curves_mw").begin(),g.at("startup_curves_mw").end(),[](const J& c){return c.empty();});
    if (compact_class) b.compact_units.insert(id);
    const bool renewable = g.at("kind") == "renewable" || g.at("kind") == "wind" || g.at("kind") == "solar";
    // Dominant integer representative, retaining the original power projection:
    // docs/modules/market/performance.md, Zero-Cost Commitment Projection.
    const double maximum_power = *std::max_element(g.at("pmax_mw").begin(),g.at("pmax_mw").end());
    bool project_commitment = independent_projections_allowed && uc && b.compact && !ancillary && !realtime &&
      (g.at("kind") == "hydro" || renewable) &&
      compact_class && num(g,"minimum_cost_per_hour") == 0 && g.at("startup_cost")[0] == 0 &&
      num(g,"technical_min_mw") == 0 && g.at("shutdown_curve_mw").empty() &&
      num(g,"min_up_minutes") == 0 && num(g,"min_down_minutes") == 0 &&
      num(g,"max_starts") >= T && num(g,"max_stops") >= T &&
      num(g,"initial_power_mw") >= 0 && num(g,"initial_power_mw") <= maximum_power &&
      (num(g,"initial_on") != 0 || num(g,"initial_power_mw") == 0);
    for (int t=0;project_commitment && t<T;++t) {
      const double interval = t ? minute(t)-minute(t-1) : dt(t)*60;
      project_commitment = at(g,"pmin_mw",t) == 0 && at(g,"regulation_up_mw",t) == 0 &&
        at(g,"regulation_down_mw",t) == 0 && num(g,"ramp_up_mw_min")*interval >= maximum_power &&
        num(g,"ramp_down_mw_min")*interval >= maximum_power;
    }
    if (project_commitment) b.projected_commitment.insert(id);
    for (int t = 0; t < T; ++t) {
      const double active = at(g, "available", t)*(1-at(g, "must_off", t));
      b.var(key("u", id, t), project_commitment ? active : at(g, "must_on", t), active,
        uc && !realtime ? num(g, "minimum_cost_per_hour")*w(t) : 0, "minimum_output", true, !uc, project_commitment);
      b.var(key("start", id, t), 0, 1, uc && compact_class && !realtime ? g.at("startup_cost")[0].get<double>() : 0, "startup", true, !uc, b.compact);
      b.var(key("stop", id, t), 0, 1, 0, "startup", true, !uc, b.compact);
      b.var(key("stable", id, t), 0, 1, 0, "energy", false, !uc);
      if (!compact_class) {
      b.var(key("offline_minutes", id, t), 0, max_time, 0, "energy", false, !uc);
      for (int k = 0; k < 3; ++k)
        b.var(key(("start"+std::to_string(k)).c_str(), id, t), 0, 1,
              uc && !realtime ? g.at("startup_cost")[k].get<double>() : 0, "startup", true, !uc);
      }
      double lo = 0, hi = at(g, "pmax_mw", t)*active;
      if (pricing) {
        // Remove accepted predecessor bound roundoff before taking the pricing
        // intersection; see southern_execution_contract.md GUI numerical audit.
        const double p = std::clamp(b.fixed.at(key("p", id, t)), 0.0, hi);
        const double delta = num(exec, "price_delta");
        if (at(g, "price_setting", t) == 0) lo = hi = p;
        else { lo = std::max(0.0, (1-delta)*p); hi = std::min(hi, (1+delta)*p); }
      }
      const int p = b.var(key("p", id, t), lo, hi);
      injection[{bus, t}].add(p);
      const int pf = b.var(key("primary", id, t), 0, at(g, "pmax_mw", t), 0, "energy", false, ancillary);
      primary[{area, t}].add(pf);
      const double capacity_max = at(g, "pmax_mw", t) - (uc ? 0 : at(g, "regulation_up_mw", t));
      const double capacity_min = at(g, "pmin_mw", t) + (uc ? 0 : at(g, "regulation_down_mw", t));
      // 2.6.3.2--3 after substituting area balance: all output is subtracted,
      // while alpha gates only the capacity term, including hydro eligibility.
      headroom[{area, t}].add(p, -1);
      footroom[{area, t}].add(p);
      // RT 3.5.3.2-3: hydro alpha denotes capability, thermal alpha denotes
      // online status. Keep this distinction separate from accident reserve.
      const bool hydro_capability=realtime&&(g.at("kind")=="hydro"||g.at("kind")=="pumped_hydro");
      if (at(g, "reserve_up_eligible", t) != 0) {
        if(hydro_capability)headroom[{area,t}].constant+=at(g,"available",t)*capacity_max;
        else headroom[{area, t}].add(b.col("u", id, t), capacity_max);
      }
      if (at(g, "reserve_down_eligible", t) != 0) {
        if(hydro_capability||(realtime&&g.at("kind")=="vpp"))footroom[{area,t}].constant-=at(g,"available",t)*capacity_min;
        else footroom[{area, t}].add(b.col("u", id, t), -capacity_min);
      }
      b.upper(key("2.6.3.4/headroom", id, t), variable(pf).add(p).add(b.col("u", id, t), -capacity_max));
      b.upper(key("2.6.3.4/fraction", id, t), variable(pf).add(b.col("u", id, t), -capacity_max*at(g, "primary_fraction", t)));
      if (ancillary) {
        // Yunnan 2025 art.15: five-minute AGC capacity. Primary is fixed above;
        // see yunnan_ancillary_markets.md for the up-only primary convention.
        const double standard = agc_members.count(id) ? num(*agc_members.at(id), "standard_ramp_mw_min") : 0;
        const double stable = std::round(b.fixed.at(key("stable", id, t)));
        const double ru = t < 96 ? 5*std::min(standard,num(g,"ramp_up_mw_min"))*stable : 0;
        const double rd = t < 96 ? 5*std::min(standard,num(g,"ramp_down_mw_min"))*stable : 0;
        const int up = b.var(key("secondary_up",id,t),0,ru,0,"energy",false,pricing);
        const int down = b.var(key("secondary_down",id,t),0,rd,0,"energy",false,pricing);
        b.upper(key("yunnan/headroom",id,t),variable(p).add(pf).add(up).add(b.col("u",id,t),-capacity_max));
        // Stable-output lower bound is also applied below, including technical minimum.
        b.upper(key("yunnan/footroom",id,t),variable(p,-1).add(down).add(b.col("stable",id,t),capacity_min));
        headroom[{area,t}].add(up,-1); footroom[{area,t}].add(down,-1);
      }
      for (size_t k = 0; k < g.at("segments").size(); ++k) {
        const auto& s = g.at("segments")[k];
        b.var(key(("segment"+std::to_string(k)).c_str(), id, t), 0, num(s, "quantity_mw"),
              g.at("bid_mode") == "quantity" ? 0 : num(s, "price_per_mwh")*w(t));
      }
      if (renewable) b.var(key("renewable_deviation", id, t), 0, at(g, "max_curtailment_mw", t), penalties[1].get<double>()*w(t), "renewable_deviation");
    }
    Expr starts, stops;
    const double minimum_up=num(g,"min_up_minutes"),minimum_down=num(g,"min_down_minutes");
    for (int t = 0; t < T; ++t) {
      const auto u = b.expr("u", id, t), start = b.expr("start", id, t), stop = b.expr("stop", id, t);
      const Expr prev_u = t ? b.expr("u", id, t-1) : Expr(num(g, "initial_on"));
      Expr transition = u; transition.add(prev_u, -1).add(start, -1).add(stop);
      b.equal(key("2.6.3.13/state", id, t), transition);
      Expr events = start; events.add(stop); b.upper(key("2.6.3.13/exclusive", id, t), events, 1);
      if (b.compact) {
        Expr e = start; e.add(u,-1); b.upper(key("2.6.3.13/start_on",id,t),e);
        e = start; e.add(prev_u); b.upper(key("2.6.3.13/start_off",id,t),e,1);
        e = stop; e.add(prev_u,-1); b.upper(key("2.6.3.13/stop_on",id,t),e);
        e = stop; e.add(u); b.upper(key("2.6.3.13/stop_off",id,t),e,1);
      }
      starts.add(start); stops.add(stop);
      const double elapsed = t ? minute(t)-minute(t-1) : 0;
      if (!compact_class) {
      Expr categories; for (int k = 0; k < 3; ++k) categories.add(b.col(("start"+std::to_string(k)).c_str(), id, t));
      categories.add(start, -1); b.equal(key("2.6.3.13/start_class", id, t), categories);
      Expr downtime = t ? b.expr("offline_minutes", id, t-1) : Expr(num(g, "initial_on") ? 0 : num(g, "initial_state_minutes"));
      downtime.constant += elapsed; downtime.add(prev_u, -elapsed);
      // Integer-minute history makes hot/warm/cold thresholds disjoint, including ties.
      const double lower[3] = {0, num(g, "warm_after_minutes"), num(g, "cold_after_minutes")};
      const double upper[3] = {num(g, "warm_after_minutes")-1, num(g, "cold_after_minutes")-1, max_time};
      for (int k = 0; k < 3; ++k) {
        const int sk = b.col(("start"+std::to_string(k)).c_str(), id, t);
        Expr low; low.add(downtime, -1).add(sk, max_time);
        b.upper(key(("2.6.3.13/class_lo"+std::to_string(k)).c_str(), id, t), low, max_time-lower[k]);
        Expr high = downtime; high.add(sk, max_time);
        b.upper(key(("2.6.3.13/class_hi"+std::to_string(k)).c_str(), id, t), high, max_time+upper[k]);
      }
      const auto off = b.expr("offline_minutes", id, t);
      Expr offmax = off; offmax.add(u, max_time); b.upper(key("history/offline_zero", id, t), offmax, max_time);
      Expr offdiff = off; offdiff.add(downtime, -1);
      Expr plus = offdiff; plus.add(u, -max_time); b.upper(key("history/offline_up", id, t), plus);
      Expr minus; minus.add(offdiff, -1).add(u, -max_time); b.upper(key("history/offline_down", id, t), minus);
      }
      // 2.6.3.12: event windows include the residual obligation of the authored initial state.
      // Increasing timestamps make both zero-duration windows empty; preserve
      // the reference scan for exact matrix checks. performance.md, empty windows.
      if(b.reference || minimum_up>0 || minimum_down>0)for (int s = 0; s <= t; ++s) {
        if (minute(t)-minute(s) < minimum_up) {
          Expr e = b.expr("start", id, s); e.add(u, -1); b.upper(key(("2.6.3.12/up"+std::to_string(s)).c_str(), id, t), e);
        }
        if (minute(t)-minute(s) < minimum_down) {
          Expr e = b.expr("stop", id, s); e.add(u); b.upper(key(("2.6.3.12/down"+std::to_string(s)).c_str(), id, t), e, 1);
        }
      }
      const double minimum = num(g,"initial_on") ? minimum_up : minimum_down;
      if (minute(t) + num(g, "initial_state_minutes") < minimum)
        b.equal(key("2.6.3.12/initial", id, t), u, num(g, "initial_on"));
      Expr startup_phase, shutdown_phase, trajectory;
      for (int k = 0; k < 3; ++k) {
        const auto& curve = g.at("startup_curves_mw")[k];
        for (int d = 0; d < static_cast<int>(curve.size()) && d <= t; ++d) {
          const int sk = b.col(("start"+std::to_string(k)).c_str(), id, t-d);
          startup_phase.add(sk); trajectory.add(sk, curve[d].get<double>());
        }
      }
      const auto& down_curve = g.at("shutdown_curve_mw");
      for (int d = 0; d < static_cast<int>(down_curve.size()); ++d) {
        const int event = t + static_cast<int>(down_curve.size())-d;
        if (event < T) { const int sd = b.col("stop", id, event); shutdown_phase.add(sd); trajectory.add(sd, down_curve[d].get<double>()); }
      }
      Expr stable = b.expr("stable", id, t);
      Expr phases = stable; phases.add(startup_phase).add(shutdown_phase).add(u, -1);
      b.equal(key("2.6.3.8/phases", id, t), phases);
      Expr p = b.expr("p", id, t);
      Expr segmented = p; segmented.add(trajectory, -1).add(stable, -num(g, "technical_min_mw"));
      for (size_t k = 0; k < g.at("segments").size(); ++k) {
        const int seg = b.col(("segment"+std::to_string(k)).c_str(), id, t);
        segmented.add(seg, -1);
        Expr limit = variable(seg); limit.add(stable, -num(g.at("segments")[k], "quantity_mw"));
        b.upper(key(("2.6.3.6/segment"+std::to_string(k)).c_str(), id, t), limit);
      }
      b.equal(key("2.6.3.6/output", id, t), segmented);
      b.metrics[key("trajectory", id, t)] = trajectory;
      double lower_bound = at(g, "pmin_mw", t), upper_bound = at(g, "pmax_mw", t);
      if (!uc) {
        lower_bound += at(g, "regulation_down_mw", t);
        upper_bound -= at(g, "regulation_up_mw", t);
        if (at(g, "regulation_down_mw", t)+at(g, "regulation_up_mw", t) > 0 && b.fixed.at(key("stable", id, t)) < 0.5)
          throw std::invalid_argument(key("regulation/preclearing_on_nonstable_unit", id, t));
      }
      Expr lo; lo.add(p, -1).add(trajectory).add(stable, lower_bound);
      Expr hi = p; hi.add(trajectory, -1).add(stable, -upper_bound);
      b.upper(key("2.6.3.8/output_min", id, t), lo); b.upper(key("2.6.3.8/output_max", id, t), hi);
      if (ancillary) {
        Expr lower; lower.add(p,-1).add(trajectory).add(stable,std::max(lower_bound,num(g,"technical_min_mw"))).add(b.col("secondary_down",id,t));
        b.upper(key("yunnan/stable_lower",id,t),lower);
        if (agc_members.count(id) && !agc_members.at(id)->at("safe_intervals_mw").empty()) {
          // Art.15/40 engineering disjunction: the entire AGC operating envelope
          // lies in one connected safe band; transitions during startup are separate.
          Expr selected; selected.add(stable,-1);
          Expr low; low.add(p,-1).add(trajectory).add(b.col("secondary_down",id,t));
          Expr high=p; high.add(trajectory,-1).add(b.col("primary",id,t)).add(b.col("secondary_up",id,t));
          const auto& bands=agc_members.at(id)->at("safe_intervals_mw");
          for (size_t k=0;k<bands.size();++k) {
            const std::string family="safe_band"+std::to_string(k);
            const int z=b.var(key(family.c_str(),id,t),0,1,0,"energy",true,pricing);
            selected.add(z);low.add(z,bands[k][0].get<double>());high.add(z,-bands[k][1].get<double>());
          }
          b.equal(key("yunnan/safe_band_choice",id,t),selected);
          b.upper(key("yunnan/safe_band_lower",id,t),low);b.upper(key("yunnan/safe_band_upper",id,t),high);
        }
      }
      Expr prev_p = t ? b.expr("p", id, t-1) : Expr(num(g, "initial_power_mw"));
      const double ramp_minutes = t ? elapsed : dt(t)*60;
      const double max_p = *std::max_element(g.at("pmax_mw").begin(), g.at("pmax_mw").end());
      const double ru = std::min(max_p, num(g, "ramp_up_mw_min")*ramp_minutes);
      const double rd = std::min(max_p, num(g, "ramp_down_mw_min")*ramp_minutes);
      Expr rampup = p; rampup.add(prev_p, -1).add(u, -ru).add(startup_phase, -(max_p-ru));
      Expr rampdown = prev_p; rampdown.add(p, -1).add(prev_u, -rd).add(shutdown_phase, -(max_p-rd));
      // Zero-length trajectories still represent an instantaneous start/stop transition.
      rampup.add(start, -max_p); rampdown.add(stop, -max_p);
      b.upper(key("2.6.3.11/up", id, t), rampup); b.upper(key("2.6.3.11/down", id, t), rampdown);
      if (renewable && (!pricing || exec.at("lmp_renewable_priority") == "retain_sced")) {
        const int curtailed = b.col("renewable_deviation", id, t);
        if (g.at("bid_mode") == "quantity") {
          Expr e = p; e.add(curtailed); b.equal(key("2.6.3.20/quantity", id, t), e, at(g, "forecast_mw", t)*at(g, "available", t));
        } else {
          Expr e; e.add(p, -1).add(curtailed, -1);
          b.upper(key("2.6.3.20/minimum", id, t), e, -num(g, "renewable_alpha")*at(g, "forecast_mw", t)*at(g, "available", t));
          b.upper(key("2.6.3.20/forecast", id, t), p, at(g, "forecast_mw", t)*at(g, "available", t));
        }
      }
    }
    b.upper(key("2.6.3.13/max_starts", id, 0), starts, num(g, "max_starts"));
    b.upper(key("2.6.3.13/max_stops", id, 0), stops, num(g, "max_stops"));
  }

  if (ancillary) for (const auto& a:j.at("_yunnan").at("config").at("agc_units")) for(int t=0;t<96;++t) {
    const double award=yunnan::required_capacity(j.at("_yunnan"),a.at("id"),t);
    Expr up,down;
    for(const auto& m:a.at("members")) { const int id=m.at("generator_id");up.add(b.col("secondary_up",id,t));down.add(b.col("secondary_down",id,t)); }
    b.equal(key("yunnan/plant_up",a.at("id"),t),up,award);
    b.equal(key("yunnan/plant_down",a.at("id"),t),down,award);
  }
  for (const auto& a : j.at("areas")) for (int t = 0; t < T; ++t) {
    const int area = a.at("id");
    const int up_row = static_cast<int>(b.le_count);
    Expr up; up.add(headroom[{area, t}], -1);
    b.upper(key("2.6.3.2/reserve", area, t), up, -at(a, "reserve_up_mw", t)-at(a, "network_reserve_reduction_mw", t));
    Expr down; down.add(footroom[{area, t}], -1);
    const int down_row = static_cast<int>(b.le_count);
    b.upper(key("2.6.3.3/reserve", area, t), down, -at(a, "reserve_down_mw", t)+at(a, "load_side_down_reserve_mw", t));
    b.reserve_rows[{area, t}] = {up_row, down_row};
    Expr pf; pf.add(primary[{area, t}], -1);
    b.upper(key("2.6.3.4/province", area, t), pf, -at(a, "primary_mw", t));
  }
  for (const auto& group : j.at("primary_groups")) for (int t = 0; t < T; ++t) {
    Expr e; for (int id : group.at("generators")) e.add(b.col("primary", id, t), -1);
    b.upper(key("2.6.3.4/direct_dispatch", group.at("id"), t), e, -at(group, "requirement_mw", t));
  }
  for (const auto& group : j.at("groups")) {
    Expr energy;
    for (int t = 0; t < T; ++t) {
      Expr power, online;
      for (int id : group.at("generators")) { power.add(b.col("p", id, t)); online.add(b.col("u", id, t)); }
      b.range(key("2.6.3.9/power", group.at("id"), t), power, at(group, "min_mw", t), at(group, "max_mw", t));
      b.range(key("2.4.7/group_status", group.at("id"), t), online, at(group, "min_online", t), at(group, "max_online", t));
      if (t < energy_points) energy.add(power, dt(t));
    }
    b.range(key("2.6.3.10/energy", group.at("id"), 0), energy, num(group, "min_mwh"), num(group, "max_mwh"));
  }

  // Research extension, southern_execution_contract.md: compensated interruption,
  // not energy shifting. Forecast load already includes the interruptible demand.
  std::map<std::pair<int,int>, Expr> reductions;
  for (const auto& d : j.at("controllable_loads")) {
    const int id = d.at("id"), bus = d.at("bus"); Expr energy;double activation_energy=0;
    for (int t = 0; t < T; ++t) {
      const auto [agc,award]=independent_award("load",id,t);
      const int r = b.var(key("load_reduction", id, t), 0,
        at(d, "available", t)*at(d, "max_reduction_mw", t),
        award>0?0:at(d, "compensation_per_mwh", t)*w(t), "demand_response", false, pricing);
      if(award>0) {
        const double baseline=agc->at("baseline_reduction_mw")[t/4];
        if(baseline<award-1e-6||baseline+award>at(d,"available",t)*at(d,"max_reduction_mw",t)+1e-6)
          throw std::invalid_argument("Independent load AGC baseline has insufficient symmetric headroom");
        b.equal(key("yunnan/load_baseline",id,t),variable(r),baseline);
        activation_energy+=award*dt(t);
      }
      injection[{bus, t}].add(r); reductions[{bus, t}].add(r);
      if (t < energy_points) energy.add(r, dt(t));
    }
    b.upper(key("demand_response/day_energy", id, 0), energy, num(d, "max_day_reduction_mwh")-activation_energy);
  }
  for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t) {
    const auto found = reductions.find({node.at("id"), t});
    if (found != reductions.end()) b.upper(key("demand_response/bus_load", node.at("id"), t), found->second, at(node, "load_mw", t));
  }

  for (const auto& s : j.at("storage")) {
    const int id = s.at("id"), bus = s.at("bus");
    const double eta = std::sqrt(num(s, "roundtrip_efficiency"));
    const bool energy_model = ancillary || !pricing || exec.at("lmp_storage_policy") == "retain_energy_constraints";
    // Exact power/SOC projection of 2.6.3.16 with zero minimum powers:
    // one direction per hour, independent forecast points. Proof and idle-mode
    // pricing equivalence: docs/modules/market/performance.md, Hourly Storage Mode.
    const bool hourly_mode = independent_projections_allowed && b.compact && !ancillary && !realtime &&
      num(s,"discharge_min_mw") == 0 && num(s,"charge_min_mw") == 0;
    const auto mode_slot = [](int t) { return t < 96 ? t/4 : 24+t-96; };
    if (hourly_mode) {
      b.compact_storage.insert(id);
      for (int h=0;h<26;++h) b.var(key("storage_hour_mode",id,h),0,1,0,"storage",true,pricing);
    }
    for (int t = 0; t < T; ++t) {
      const double avail = at(s, "available", t);
      const double award=independent_award("storage",id,t).second;
      b.var(key("dis_on", id, t), 0, award>0?0:avail, 0, "storage", true, pricing, hourly_mode);
      b.var(key("ch_on", id, t), 0, award>0?0:avail, 0, "storage", true, pricing, hourly_mode);
      if (hourly_mode) {
        b.equal(key("storage_hour_dis",id,t),b.expr("dis_on",id,t).add(b.expr("storage_hour_mode",id,mode_slot(t)),-avail));
        b.equal(key("storage_hour_ch",id,t),b.expr("ch_on",id,t).add(b.expr("storage_hour_mode",id,mode_slot(t)),avail),avail);
      }
      double dlo = 0, dhi = num(s, "discharge_max_mw")*avail;
      double clo = -num(s, "charge_max_mw")*avail, chi = 0;
      if (pricing) {
        const double dis = std::clamp(b.fixed.at(key("dis", id, t)), 0.0, dhi);
        const double ch = std::clamp(b.fixed.at(key("ch", id, t)), clo, 0.0);
        const double delta = at(s, "price_setting", t) ? num(exec, "price_delta") : 0;
        dlo = std::max(dlo, (1-delta)*dis); dhi = std::min(dhi, (1+delta)*dis);
        clo = std::max(clo, (1+delta)*ch); chi = std::min(chi, (1-delta)*ch);
      }
      if(award>0) {dlo=dhi=clo=chi=0;}
      const int dis = b.var(key("dis", id, t), dlo, dhi, num(s, "discharge_price")*w(t), "storage");
      const int ch = b.var(key("ch", id, t), clo, chi, num(s, "charge_price")*w(t), "storage");
      injection[{bus, t}].add(dis).add(ch);
      const auto [reserve_up, reserve_down] = b.reserve_rows.at({bus_area.at(bus), t});
      b.le[reserve_up].expr.add(dis).add(ch);
      b.le[reserve_down].expr.add(dis, -1).add(ch, -1);
      if (energy_model) {
        b.var(key("energy", id, t), at(s, "min_mwh", t)+award/eta, at(s, "max_mwh", t)-award*eta);
        if(award>0) {
          if(t) b.range(key("yunnan/storage_start_energy",id,t),b.expr("energy",id,t-1),at(s,"min_mwh",t)+award/eta,at(s,"max_mwh",t)-award*eta);
          else if(num(s,"initial_mwh")<at(s,"min_mwh",t)+award/eta-1e-6 || num(s,"initial_mwh")>at(s,"max_mwh",t)-award*eta+1e-6)
            throw std::invalid_argument("Independent storage cannot sustain first-hour AGC from initial energy");
        }
      }
    }
    Expr cycle;
    for (int t = 0; t < T; ++t) {
      auto dis = b.expr("dis", id, t), ch = b.expr("ch", id, t);
      Expr dir = b.expr("dis_on", id, t); dir.add(b.expr("ch_on", id, t)); b.upper(key("2.6.3.16/direction", id, t), dir, 1);
      Expr dlo; dlo.add(dis, -1).add(b.col("dis_on", id, t), num(s, "discharge_min_mw")); b.upper(key("2.6.3.16/dis_min", id, t), dlo);
      Expr dhi = dis; dhi.add(b.col("dis_on", id, t), -num(s, "discharge_max_mw")); b.upper(key("2.6.3.16/dis_max", id, t), dhi);
      Expr clo; clo.add(ch, -1).add(b.col("ch_on", id, t), -num(s, "charge_max_mw")); b.upper(key("2.6.3.16/ch_min", id, t), clo);
      Expr chi = ch; chi.add(b.col("ch_on", id, t), num(s, "charge_min_mw")); b.upper(key("2.6.3.16/ch_max", id, t), chi);
      if (t < energy_points) {
        Expr hourly = b.expr("dis_on", id, t);
        if(realtime) {
          const int hour=static_cast<int>((num(j.at("_rt"),"start_minute")+minute(t))/60);
          int count=0;for(int tt=0;tt<T;++tt)if(static_cast<int>((num(j.at("_rt"),"start_minute")+minute(tt))/60)==hour)++count;
          for(int tt=0;tt<T;++tt)if(static_cast<int>((num(j.at("_rt"),"start_minute")+minute(tt))/60)==hour)hourly.add(b.col("ch_on",id,tt),1.0/count);
          for(const auto& mode:j.at("_rt").at("storage_hour_modes"))if(mode.at("id")==id&&mode.at("hour")==hour)
            b.equal(key("3.5.3.15/executed_hour_mode",id,t),b.expr(mode.at("mode")=="charge"?"dis_on":"ch_on",id,t),0);
        } else for (int tt = t/4*4; tt < t/4*4+4; ++tt) hourly.add(b.col("ch_on", id, tt), 0.25);
        b.upper(key("2.6.3.16/hour", id, t), hourly, 1);
      }
      if (energy_model) {
        Expr energy = b.expr("energy", id, t);
        if (t) energy.add(b.col("energy", id, t-1), -1); else energy.constant -= num(s, "initial_mwh");
        energy.add(ch, eta*dt(t)).add(dis, dt(t)/eta);
        b.equal(key("2.6.3.16/energy", id, t), energy);
        cycle.add(dis, dt(t)/eta).add(ch, -dt(t)*eta);
      }
    }
    if (energy_model) {
      b.equal(key("2.6.3.16/terminal", id, energy_points-1), b.expr("energy", id, energy_points-1), num(s, "terminal_mwh"));
      b.upper(key("2.6.3.16/cycles", id, 0), cycle, 2*num(s, "rated_mwh")*num(s, "max_cycles"));
    }
  }

  std::map<std::pair<int,int>, Expr> hubs;
  for (const auto& d : j.at("dc_links")) {
    const int id = d.at("id");
    for (int t = 0; t < T; ++t) {
      const double avail = at(d, "available", t);
      const int power = b.var(key("dc", id, t), at(d, "min_mw", t)*avail, at(d, "max_mw", t)*avail);
      b.var(key("dc_up", id, t), 0, 1, 0, "transmission", true, pricing);
      b.var(key("dc_down", id, t), 0, 1, 0, "transmission", true, pricing);
      if (d.at("from_bus").get<int>() >= 0) injection[{d.at("from_bus"), t}].add(power, -1);
      else hubs[{d.at("from_hub"), t}].add(power, -1);
      if (d.at("to_bus").get<int>() >= 0) injection[{d.at("to_bus"), t}].add(power, 1-num(d, "loss_fraction"));
      else hubs[{d.at("to_hub"), t}].add(power, 1-num(d, "loss_fraction"));
    }
    for (int t = 0; t < T; ++t) {
      Expr diff = b.expr("dc", id, t);
      if (t) diff.add(b.col("dc", id, t-1), -1); else diff.constant -= num(d, "initial_mw");
      Expr up = diff; up.add(b.col("dc_up", id, t), -at(d, "ramp_up_mw", t)); b.upper(key("2.6.3.17/up", id, t), up);
      Expr down; down.add(diff, -1).add(b.col("dc_down", id, t), -at(d, "ramp_down_mw", t)); b.upper(key("2.6.3.17/down", id, t), down);
      Expr dir = b.expr("dc_up", id, t); dir.add(b.col("dc_down", id, t)); b.upper(key("2.6.3.17/direction", id, t), dir, 1);
      for (const auto& [now, before, initial] : {std::tuple{"dc_up", "dc_down", -1}, std::tuple{"dc_down", "dc_up", 1}}) {
        Expr e = b.expr(now, id, t);
        if (t) e.add(b.col(before, id, t-1)); else e.constant += num(d, "initial_adjustment") == initial ? 1 : 0;
        b.upper(key((std::string("2.6.3.17/no_reversal/")+now).c_str(), id, t), e, 1);
      }
    }
  }
  for (const auto& hub : j.at("dc_hubs")) for (int t = 0; t < T; ++t)
    b.equal(key("2.6.3.17/hub", hub.at("id"), t), hubs[{hub.at("id"), t}]);

  // 2.6.3.14--15. B-theta and PTDF are equivalent on each connected component;
  // outages remove the branch equation and flow, so topology changes each period.
  for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t) {
    b.var(key("theta", node.at("id"), t), -inf, inf);
  }
  for (const auto& line : j.at("branches")) for (int t = 0; t < T; ++t) {
    const int id = line.at("id"), from = line.at("from_bus"), to = line.at("to_bus");
    const bool available = at(line, "available", t) != 0;
    const int flow = b.var(key("flow", id, t), available ? -inf : 0, available ? inf : 0);
    injection[{from, t}].add(flow, -1); injection[{to, t}].add(flow);
    if (available) {
      const double susceptance = num(j, "base_mva")/(num(line, "x_pu")*num(line, "tap"));
      Expr eq = variable(flow); eq.add(b.col("theta", from, t), -susceptance).add(b.col("theta", to, t), susceptance);
      b.equal(key("network/flow", id, t), eq, -susceptance*num(line, "shift_deg")*std::acos(-1)/180);
    }
    const int sp = b.var(key("line_slack_plus", id, t), 0, available ? inf : 0, penalties[0].get<double>()*w(t), "network_slack");
    const int sn = b.var(key("line_slack_minus", id, t), 0, available ? inf : 0, penalties[0].get<double>()*w(t), "network_slack");
    Expr e = variable(flow); e.add(sp, -1).add(sn);
    b.range(key("2.6.3.14/line", id, t), e, available ? at(line, "min_mw", t) : 0, available ? at(line, "max_mw", t) : 0);
  }
  for (int t = 0; t < T; ++t) {
    std::map<int,int> parent; for (const auto& n : j.at("buses")) parent[n.at("id")] = n.at("id");
    const auto root = [&](int n) { while (parent.at(n) != n) n = parent.at(n); return n; };
    for (const auto& l : j.at("branches")) if (at(l, "available", t) != 0) parent[root(l.at("to_bus"))] = root(l.at("from_bus"));
    std::set<int> roots;
    for (const auto& n : j.at("buses")) roots.insert(root(n.at("id")));
    for (int r : roots) b.equal(key("network/reference", r, t), b.expr("theta", r, t));
    for (const auto& node : j.at("buses")) {
      const int id = node.at("id");
      b.balance_rows[key("bus", id, t)] = b.equal(key("2.6.3.1/balance", id, t), injection[{id, t}]);
    }
  }
  for (const auto& section : j.at("sections")) for (int t = 0; t < T; ++t) {
    const int id = section.at("id"); Expr flow;
    for (const auto& member : section.at("members")) flow.add(b.col("flow", member.at("branch"), t), num(member, "coefficient"));
    b.metrics[key("section_flow", id, t)] = flow;
    flow.add(b.var(key("section_slack_plus", id, t), 0, inf, penalties[0].get<double>()*w(t), "network_slack"), -1);
    flow.add(b.var(key("section_slack_minus", id, t), 0, inf, penalties[0].get<double>()*w(t), "network_slack"));
    b.range(key("2.6.3.15/section", id, t), flow, at(section, "min_mw", t), at(section, "max_mw", t));
  }

  std::map<std::pair<std::string,int>, std::vector<const J*>> gateways;
  for (const auto& trade : j.at("trades")) {
    const int id = trade.at("id");
    gateways[{trade.at("gateway_kind"), trade.at("gateway")}].push_back(&trade);
    Expr energy;
    for (int t = 0; t < T; ++t) {
      const int p = b.var(key("trade", id, t), realtime&&exec.at("priority_policy")=="penalized_shortfall"?0:at(trade, "min_mw", t), at(trade, "max_mw", t),
          num(trade, uc ? "scuc_fee" : pricing ? "lmp_fee" : "sced_fee")*w(t), "transmission");
      if (t < energy_points) energy.add(p, dt(t));
    }
    if (realtime) {
      for(int t=0;t<T;++t){Expr power=b.expr("trade",id,t);
        if(exec.at("priority_policy")=="penalized_shortfall")power.add(b.var(key("rt_priority_slack",id,t),0,at(trade,"min_mw",t),penalties[3].get<double>()*w(t),"priority_shortfall"));
        b.upper(key("3.5.3.20/priority",id,t),Expr{}.add(power,-1),-at(trade,"min_mw",t));}
    } else if (!pricing || exec.at("lmp_renewable_priority") == "retain_sced") {
      b.upper(key("2.6.3.21/maximum", id, 0), energy, num(trade, "max_mwh"));
      if (exec.at("priority_policy") == "penalized_shortfall")
        energy.add(b.var(key("priority_shortfall_mwh", id, 0), 0, num(trade, "adjusted_min_mwh"), penalties[3], "priority_shortfall"));
      Expr e; e.add(energy, -1); b.upper(key("2.6.3.21/minimum", id, 0), e, -num(trade, "adjusted_min_mwh"));
    }
  }
  for (const auto& [gateway, trades] : gateways) for (int t = 0; t < T; ++t) {
    Expr e = b.expr(gateway.first == "ac_branch" ? "flow" : "dc", gateway.second, t);
    for (const auto* trade : trades) e.add(b.col("trade", trade->at("id"), t), trade->at("direction") == "forward" ? -1 : 1);
    b.equal(key(("2.6.3.21/gateway/"+gateway.first).c_str(), gateway.second, t), e);
  }

  std::map<int, const J*> reservoirs;
  for (const auto& h : j.at("reservoirs")) {
    const int id = h.at("id"); reservoirs[id] = &h;
    const J members = h.contains("generator") ? J::array({h.at("generator")}) : h.at("generators");
    for (int t = 0; t < T; ++t) {
      // Invertible water-energy coordinate; 2.6.3.18 and conditioning rationale
      // in southern_execution_contract.md. Public level and audits stay in m.
      if (b.reservoir_scaling) {
        const double scale = num(h,"water_m3_mwh")/num(h,"area_m2"), initial = num(h,"initial_level_m");
        const int e = b.var(key("water_energy",id,t),(at(h,"min_level_m",t)-initial)/scale,(at(h,"max_level_m",t)-initial)/scale);
        b.metrics[key("level",id,t)] = Expr(initial).add(e,scale);
      } else {
        b.var(key("level", id, t), at(h, "min_level_m", t), at(h, "max_level_m", t));
        b.metrics[key("level",id,t)] = b.expr("level",id,t);
      }
      // h is m3/MWh; spill/h * 3600 converts m3/s to equivalent spilled MW.
      b.var(key("spill", id, t), 0, at(h, "spill_max_m3_s", t), penalties[2].get<double>()*w(t)*3600/num(h, "water_m3_mwh"), "hydro_spill");
      Expr release = b.expr("spill", id, t);
      for (const auto& member : members) release.add(b.col("p", member, t), num(h, "water_m3_mwh")/3600);
      b.metrics[key("release", id, t)] = release;
    }
  }
  for (const auto& h : j.at("reservoirs")) {
    const int id = h.at("id"), parent = h.at("upstream"), lag = h.at("lag_slots");
    const J members = h.contains("generator") ? J::array({h.at("generator")}) : h.at("generators");
    Expr energy;
    for (int t = 0; t < T; ++t) {
      const Expr release = b.metrics.at(key("release", id, t));
      Expr balance = b.metrics.at(key("level", id, t));
      if (t) balance.add(b.metrics.at(key("level", id, t-1)), -1); else balance.constant -= num(h, "initial_level_m");
      const double factor = dt(t)*3600/num(h, "area_m2");
      balance.add(release, factor); balance.constant -= factor*at(h, "inflow_m3_s", t);
      if (parent >= 0) {
        if (t >= lag) balance.add(b.metrics.at(key("release", parent, t-lag)), -factor);
        else {
          const auto& history = reservoirs.at(parent)->at("release_history_m3_s");
          balance.constant -= factor*history[history.size()-static_cast<size_t>(lag-t)].get<double>();
        }
      }
      const int water_row = b.equal(key("2.6.3.18/water", id, t), balance);
      if (b.reservoir_scaling) b.eq[water_row].solver_scale = num(h,"area_m2")/num(h,"water_m3_mwh");
      b.range(key("2.4.8/release", id, t), release, at(h, "release_min_m3_s", t), at(h, "release_max_m3_s", t));
      Expr diff = release;
      if (t) diff.add(b.metrics.at(key("release", id, t-1)), -1); else diff.constant -= num(h, "initial_release_m3_s");
      b.range(key("2.4.8/release_ramp", id, t), diff, -at(h, "release_ramp_m3_s", t), at(h, "release_ramp_m3_s", t));
      if (t < energy_points) for (const auto& member : members) energy.add(b.col("p", member, t), dt(t));
    }
    b.range(key("2.6.3.19/hydro_energy", id, 0), energy, num(h, "min_mwh"), num(h, "max_mwh"));
  }
  if(realtime) {
    // Southern 3.5.3.17/3.5.4.14; formulas and LMP interpretation:
    // docs/modules/market/southern_real_time.md.
    for(const auto& area:j.at("_rt").at("accident_reserve"))for(int t=0;t<T;++t) {
      Expr total;
      for(const auto& g:j.at("generators"))if(bus_area.at(g.at("bus"))==area.at("id")&&(g.at("kind")=="thermal"||g.at("kind")=="hydro")) {
        const int id=g.at("id");const double cap=at(g,"available",t)*at(g,"pmax_mw",t);
        const int r=b.var(key("accident_reserve",id,t),0,cap);
        Expr e=variable(r).add(b.col("p",id,t));
        if(g.at("kind")=="thermal") {e.add(b.col("u",id,t),-cap);b.upper(key("3.5.3.17/headroom",id,t),e);
          b.upper(key("3.5.3.17/ten_minute",id,t),variable(r).add(b.col("u",id,t),-10*num(g,"ramp_up_mw_min")));}
        else b.upper(key("3.5.3.17/hydro",id,t),e,cap);
        total.add(r);
      }
      for(const auto& p:j.at("_rt").at("pumped_reserve"))if(p.at("area")==area.at("id")) {
        for(const auto& ext:j.at("external_schedules"))if(ext.at("id")==p.at("id")){const double power=at(ext,"power_mw",t);total.constant+=power>=0?at(p,"online_capacity_mw",t)-power:-power;}
      }
      const double penalty=num(j.at("_rt"),pricing?"pricing_reserve_penalty":"reserve_penalty");
      const int slack=b.var(key("accident_slack",area.at("id"),t),0,j.at("_rt").at("reserve_policy")=="strict"?0:inf,penalty*w(t),"accident_shortfall");
      total.add(slack);
      b.upper(key("3.5.3.17/province",area.at("id"),t),Expr{}.add(total,-1),-at(area,"requirement_mw",t)-at(area,"reduction_mw",t));
    }
    for(const auto& plant:j.at("_rt").at("hydro_plants"))for(int t=0;t<T;++t){Expr power;for(int id:plant.at("generators"))power.add(b.col("p",id,t));
      b.range(key("3.5.3.11/plant",plant.at("id"),t),power,std::max(at(plant,"min_mw",t),at(plant,"min_mwh",t)/dt(t)),std::min(at(plant,"max_mw",t),at(plant,"max_mwh",t)/dt(t)));}
  }
  for (const auto& cut : cuts) {
    Expr e; for (const auto& [name, coefficient] : cut.coefficients) e.add(b.columns.at(name), coefficient);
    b.upper(cut.name, e, cut.rhs);
  }
  b.finish(); b.size_report = model_size_report(b,j); return lease;
}

void compare_assembly(const Build& actual,const Build& expected) {
  const auto require=[](bool same,const char* field) {
    if(!same)throw std::logic_error(std::string("assembly reference mismatch: ")+field);
  };
  const auto vector_equal=[&](const auto& a,const auto& b,const char* field) {
    require(a.size()==b.size() && (a.array()==b.array()).all(),field);
  };
  const auto matrix_equal=[&](const auto& a,const auto& b,const char* field) {
    require(a.rows()==b.rows() && a.cols()==b.cols() && a.nonZeros()==b.nonZeros(),field);
    require(std::equal(a.outerIndexPtr(),a.outerIndexPtr()+a.outerSize()+1,b.outerIndexPtr()),field);
    require(std::equal(a.innerIndexPtr(),a.innerIndexPtr()+a.nonZeros(),b.innerIndexPtr()),field);
    require(std::equal(a.valuePtr(),a.valuePtr()+a.nonZeros(),b.valuePtr()),field);
  };
  const auto expr_equal=[&](const Expr& a,const Expr& b) {
    Expr::Flat av,bv;a.each([&](int i,double v){av.emplace_back(i,v);});b.each([&](int i,double v){bv.emplace_back(i,v);});
    require(a.constant==b.constant && av==bv,"original expression");
  };
  const auto& a=actual.model.linear_part;const auto& b=expected.model.linear_part;
  matrix_equal(a.A,b.A,"A");matrix_equal(a.Aeq,b.Aeq,"Aeq");
  vector_equal(a.b,b.b,"b");vector_equal(a.beq,b.beq,"beq");vector_equal(a.c,b.c,"c");
  vector_equal(a.row_lhs,b.row_lhs,"row lower sides");
  require(a.sense==b.sense && a.vars.size()==b.vars.size(),"variables");
  for(size_t i=0;i<a.vars.size();++i)require(a.vars[i].name==b.vars[i].name && a.vars[i].type==b.vars[i].type && a.vars[i].lb==b.vars[i].lb && a.vars[i].ub==b.vars[i].ub,"variable metadata");
  require(actual.model.binary_idx==expected.model.binary_idx && actual.audited_binary_columns==expected.audited_binary_columns,"integers");
  require(actual.columns==expected.columns && actual.balance_rows==expected.balance_rows && actual.reserve_rows==expected.reserve_rows && actual.fixed==expected.fixed,"index/dispatch maps");
  require(actual.constant_cost==expected.constant_cost && actual.cost_categories==expected.cost_categories,"objective categories");
  require(actual.compact_units==expected.compact_units && actual.compact_storage==expected.compact_storage && actual.projected_commitment==expected.projected_commitment,"projections");
  require(actual.submitted_rows==expected.submitted_rows && actual.redundant_rows==expected.redundant_rows,"row certificates");
  for(const auto* rows:{&actual.le,&actual.eq}) {
    const auto& other=rows==&actual.le ? expected.le : expected.eq;
    require(rows->size()==other.size(),"original rows");
    for(size_t i=0;i<rows->size();++i){const auto& x=rows->at(i);const auto& y=other[i];require(x.name==y.name && x.rhs==y.rhs && x.solver_scale==y.solver_scale,"row metadata");expr_equal(x.expr,y.expr);}
  }
  require(actual.metrics.size()==expected.metrics.size(),"metrics");
  for(const auto& [name,expr]:actual.metrics)expr_equal(expr,expected.metrics.at(name));
  require(actual.size_report==expected.size_report,"size report");
}

BuildLease build_model(const J& j,const std::string& stage,std::map<std::string,double> previous={},
                       const std::vector<SecurityCut>& cuts={},bool independent=true) {
  const auto mode=j.at("execution").value("assembly_mode",std::string("cached"));
  auto result=assemble_model(j,stage,std::move(previous),cuts,independent,mode=="reference");
  if(mode=="verify") {
    auto reference=assemble_model(j,stage,result->fixed,cuts,independent,true);
    compare_assembly(*result,*reference);
    result->assembly_template["matrix_comparison"]="exact_match";
    result->assembly_template["mode"]="verify";
  }
  return result;
}

bool derive_sced(Build& b,const J& input,const engine::SolveResult& solved,
    std::map<std::string,double>& dispatch,const std::vector<SecurityCut>& cuts) {
  // Audit every uc-dependent branch before reusing the original matrices.
  // Branch ledger, subset proof and independent reference check: performance.md.
  if(b.stage!="scuc"||b.reference||input.contains("_rt")||input.contains("_yunnan")||b.prune_rows)return false;
  for(const auto& g:input.at("generators")) {
    if(num(g,"minimum_cost_per_hour")!=0)return false;
    for(const auto& cost:g.at("startup_cost"))if(cost.get<double>()!=0)return false;
    for(int t=0;t<T;++t)if(at(g,"regulation_up_mw",t)!=0||at(g,"regulation_down_mw",t)!=0)return false;
  }
  for(const auto& trade:input.at("trades"))if(num(trade,"scuc_fee")!=num(trade,"sced_fee"))return false;
  struct Update {int col;double value;engine::VarType type;};
  std::vector<Update> updates;
  for(const auto& g:input.at("generators"))for(int t=0;t<T;++t) {
    const int id=g.at("id");
    const auto freeze=[&](const char* family,bool integer,bool implied){
      const int c=b.col(family,id,t);const auto& var=b.model.linear_part.vars[c];
      const double v=integer?std::round(solved.x[c]):solved.x[c];
      if(v<var.lb||v>var.ub)return false;
      updates.push_back({c,v,integer&&!implied?engine::VarType::Binary:engine::VarType::Continuous});return true;
    };
    if(!freeze("u",true,false)||!freeze("start",true,b.compact)||!freeze("stop",true,b.compact)||!freeze("stable",false,false))return false;
    if(!b.compact_units.contains(id)) {
      if(!freeze("offline_minutes",false,false))return false;
      for(const char* family:{"start0","start1","start2"})if(!freeze(family,true,false))return false;
    }
  }
  std::vector<bool> fixed(b.costs.size(),false);
  for(const auto& update:updates){auto& var=b.model.linear_part.vars[update.col];
    var.lb=var.ub=update.value;var.type=update.type;fixed[update.col]=true;}
  auto& integers=b.model.binary_idx;
  integers.erase(std::remove_if(integers.begin(),integers.end(),[&](int c){return fixed[c];}),integers.end());
  b.stage="sced";b.fixed=std::move(dispatch);b.projected_commitment.clear();
  for(size_t r=0;r<b.le.size();++r)b.redundant_rows[r]=b.bound_redundant(b.le[r]);
  b.size_report=model_size_report(b,input);
  b.assembly_template={{"mode",input.at("execution").value("assembly_mode",std::string("cached"))},
    {"derived_from","scuc"},{"guard","zero_commitment_costs_zero_regulation_equal_trade_fees"},
    {"reused_matrices",2},{"layout_reused",true},{"matrix_comparison","unchanged_by_construction"}};
  if(input.at("execution").value("assembly_mode",std::string("cached"))=="verify") {
    auto reference=assemble_model(input,"sced",b.fixed,cuts,true,true);
    compare_assembly(b,*reference);b.assembly_template["matrix_comparison"]="exact_match";
  }
  return true;
}

bool derive_lmp(Build& b,const J& input,std::map<std::string,double>& dispatch,
                const std::vector<SecurityCut>& cuts) {
  // Ordered row/column selection, not numerical elimination. The pricing branch
  // ledger and A_price=R*A_sced*C proof are in performance.md, SCED-to-LMP reuse.
  if(b.stage!="sced"||b.reference||!b.compact||b.prune_rows||!cuts.empty()||
     input.contains("_rt")||input.contains("_yunnan"))return false;
  const auto& exec=input.at("execution");
  const auto& penalties=exec.at("pricing_penalties");
  const bool omit_energy=exec.at("lmp_storage_policy")=="power_neighborhood_only";
  const bool omit_priority=exec.at("lmp_renewable_priority")=="omit_unlisted";
  auto& lp=b.model.linear_part;
  std::vector<int> columns(lp.vars.size(),-1);
  std::vector<bool> removed(lp.vars.size(),false);
  const auto weight=[&](int t){return num(input.at("periods")[t],"weight_hr");};
  const auto freeze=[&](const char* family,int id,int t,bool integer) {
    const int c=b.col(family,id,t);
    const double value=dispatch.at(key(family,id,t));
    lp.vars[c].lb=lp.vars[c].ub=integer?std::round(value):value;
  };
  for(const auto& g:input.at("generators"))for(int t=0;t<T;++t) {
    const int id=g.at("id");
    for(const char* family:{"u","start","stop"})freeze(family,id,t,true);
    freeze("stable",id,t,false);
    if(!b.compact_units.contains(id)) {
      freeze("offline_minutes",id,t,false);
      for(const char* family:{"start0","start1","start2"})freeze(family,id,t,true);
    }
    const double active=at(g,"available",t)*(1-at(g,"must_off",t));
    const double hi=at(g,"pmax_mw",t)*active;
    const double p=std::clamp(dispatch.at(key("p",id,t)),0.0,hi),delta=num(exec,"price_delta");
    auto& var=lp.vars[b.col("p",id,t)];
    if(at(g,"price_setting",t)==0)var.lb=var.ub=p;
    else {var.lb=std::max(0.0,(1-delta)*p);var.ub=std::min(hi,(1+delta)*p);}
    if(g.at("kind")=="renewable"||g.at("kind")=="wind"||g.at("kind")=="solar")
      b.costs[b.col("renewable_deviation",id,t)]=penalties[1].get<double>()*weight(t);
  }
  for(const auto& d:input.at("controllable_loads"))for(int t=0;t<T;++t)freeze("load_reduction",d.at("id"),t,false);
  for(const auto& s:input.at("storage")) {
    const int id=s.at("id");
    if(b.compact_storage.contains(id))for(int h=0;h<26;++h)freeze("storage_hour_mode",id,h,true);
    for(int t=0;t<T;++t) {
      freeze("dis_on",id,t,true);freeze("ch_on",id,t,true);
      const double dhi=num(s,"discharge_max_mw")*at(s,"available",t);
      const double clo=-num(s,"charge_max_mw")*at(s,"available",t);
      const double dis=std::clamp(dispatch.at(key("dis",id,t)),0.0,dhi);
      const double ch=std::clamp(dispatch.at(key("ch",id,t)),clo,0.0);
      const double delta=at(s,"price_setting",t)?num(exec,"price_delta"):0;
      auto& discharge=lp.vars[b.col("dis",id,t)];auto& charge=lp.vars[b.col("ch",id,t)];
      discharge.lb=std::max(0.0,(1-delta)*dis);discharge.ub=std::min(dhi,(1+delta)*dis);
      charge.lb=std::max(clo,(1+delta)*ch);charge.ub=std::min(0.0,(1-delta)*ch);
      if(omit_energy)removed[b.col("energy",id,t)]=true;
    }
  }
  for(const auto& d:input.at("dc_links"))for(int t=0;t<T;++t) {
    freeze("dc_up",d.at("id"),t,true);freeze("dc_down",d.at("id"),t,true);
  }
  for(const auto& line:input.at("branches"))for(int t=0;t<T;++t)
    for(const char* family:{"line_slack_plus","line_slack_minus"})b.costs[b.col(family,line.at("id"),t)]=penalties[0].get<double>()*weight(t);
  for(const auto& section:input.at("sections"))for(int t=0;t<T;++t)
    for(const char* family:{"section_slack_plus","section_slack_minus"})b.costs[b.col(family,section.at("id"),t)]=penalties[0].get<double>()*weight(t);
  for(const auto& h:input.at("reservoirs"))for(int t=0;t<T;++t)
    b.costs[b.col("spill",h.at("id"),t)]=penalties[2].get<double>()*weight(t)*3600/num(h,"water_m3_mwh");
  for(const auto& trade:input.at("trades")) {
    const int id=trade.at("id");
    for(int t=0;t<T;++t)b.costs[b.col("trade",id,t)]=num(trade,"lmp_fee")*weight(t);
    if(exec.at("priority_policy")=="penalized_shortfall") {
      const int c=b.col("priority_shortfall_mwh",id,0);
      if(omit_priority)removed[c]=true;else b.costs[c]=penalties[3].get<double>();
    }
  }
  int count=0;
  for(size_t c=0;c<columns.size();++c)if(!removed[c])columns[c]=count++;
  const auto remap_expression=[&](Expr& expr) {
    auto* terms=std::get_if<Expr::Flat>(&expr.terms);
    if(!terms)throw std::logic_error("LMP reuse expected compact expression storage");
    for(auto& [col,value]:*terms) {
      if(columns[col]<0)throw std::logic_error("Retained LMP row references a removed column");
      col=columns[col];
    }
  };
  const auto select_rows=[&](std::vector<Row>& rows) {
    std::vector<int> mapping(rows.size(),-1);size_t next=0;
    for(size_t i=0;i<rows.size();++i) {
      const std::string_view name=rows[i].name;
      const bool drop_energy=omit_energy&&(name.starts_with("2.6.3.16/energy/")||
        name.starts_with("2.6.3.16/terminal/")||name.starts_with("2.6.3.16/cycles/"));
      const bool drop_priority=omit_priority&&(name.starts_with("2.6.3.20/")||
        name.starts_with("2.6.3.21/minimum/")||name.starts_with("2.6.3.21/maximum/"));
      if(drop_energy||drop_priority)continue;
      remap_expression(rows[i].expr);mapping[i]=static_cast<int>(next);
      if(i!=next)rows[next]=std::move(rows[i]);++next;
    }
    rows.resize(next);return mapping;
  };
  const auto inequalities=select_rows(b.le),equalities=select_rows(b.eq);
  const auto select_matrix=[&](Eigen::SparseMatrix<double>& matrix,const std::vector<int>& rows,int row_count) {
    Eigen::SparseMatrix<double> next(row_count,count);next.reserve(matrix.nonZeros());
    for(int c=0;c<matrix.outerSize();++c)if(columns[c]>=0) {
      next.startVec(columns[c]);
      for(Eigen::SparseMatrix<double>::InnerIterator it(matrix,c);it;++it)
        if(rows[it.row()]>=0)next.insertBack(rows[it.row()],columns[c])=it.value();
    }
    next.finalize();matrix=std::move(next);
  };
  const auto select_vector=[](Eigen::VectorXd& values,const std::vector<int>& mapping,int size) {
    Eigen::VectorXd next(size);
    for(size_t i=0;i<mapping.size();++i)if(mapping[i]>=0)next[mapping[i]]=values[i];
    values=std::move(next);
  };
  select_matrix(lp.A,inequalities,static_cast<int>(b.le.size()));
  select_matrix(lp.Aeq,equalities,static_cast<int>(b.eq.size()));
  select_vector(lp.b,inequalities,static_cast<int>(b.le.size()));
  select_vector(lp.beq,equalities,static_cast<int>(b.eq.size()));
  if(lp.row_lhs.size())select_vector(lp.row_lhs,inequalities,static_cast<int>(b.le.size()));
  const auto select_columns=[&](auto& values) {
    for(size_t i=0;i<columns.size();++i)if(columns[i]>=0&&columns[i]!=static_cast<int>(i))values[columns[i]]=std::move(values[i]);
    values.resize(count);
  };
  select_columns(lp.vars);select_columns(b.costs);select_columns(b.cost_categories);
  lp.c=Eigen::Map<Eigen::VectorXd>(b.costs.data(),count);
  b.column_hints.clear();
  for(auto it=b.columns.begin();it!=b.columns.end();) {
    if(columns[it->second]<0)it=b.columns.erase(it);else {it->second=columns[it->second];++it;}
  }
  for(auto family=b.column_slots.begin();family!=b.column_slots.end();) {
    bool any=false;
    for(auto& [id,slots]:family->second)for(auto& col:slots)if(col>=0){col=columns[col];any|=col>=0;}
    if(any)++family;else family=b.column_slots.erase(family);
  }
  for(auto& [name,expr]:b.metrics)remap_expression(expr);
  for(auto& [name,row]:b.balance_rows)row=equalities.at(row);
  for(auto& [key,rows]:b.reserve_rows){rows.first=inequalities.at(rows.first);rows.second=inequalities.at(rows.second);}
  for(auto& col:b.audited_binary_columns)col=columns.at(col);
  b.model.binary_idx.clear();b.model.initial_solution.resize(0);
  b.fixed=std::move(dispatch);b.stage="lmp";b.projected_commitment.clear();
  b.native_diagnostics=b.solver_timing=b.price_consistency=b.gap_certificate=nullptr;
  b.primal_start={{"status","not_requested"},{"runtime_sec",0.0},{"accepted",false}};
  b.eq_count=b.eq.size();b.le_count=b.le.size();b.redundant_rows.clear();
  b.submitted_rows.resize(b.le.size());std::iota(b.submitted_rows.begin(),b.submitted_rows.end(),size_t{0});
  for(const auto& row:b.le)b.redundant_rows.push_back(b.bound_redundant(row));
  b.le_pattern={};b.eq_pattern={};b.reusable=b.layout_match=b.column_layout_match=false;
  b.size_report=model_size_report(b,input);
  b.assembly_template={{"mode",exec.value("assembly_mode",std::string("cached"))},{"derived_from","sced"},
    {"reused_matrices",0},{"layout_reused",false},{"matrix_comparison","ordered_selection_by_construction"},
    {"removed_columns",columns.size()-b.costs.size()},{"removed_rows",inequalities.size()+equalities.size()-b.le.size()-b.eq.size()}};
  if(exec.value("assembly_mode",std::string("cached"))=="verify") {
    auto reference=assemble_model(input,"lmp",b.fixed,cuts,true,true);
    compare_assembly(b,*reference);b.assembly_template["matrix_comparison"]="exact_match";
  }
  return true;
}

int gurobi_method(const Build& b, const J& input) {
  if (b.stage == "lmp") return b.costs.size()>=1000000 ? 2 : 1;
  const std::string method = input.at("execution").value("gurobi_method",std::string("auto"));
  if (method == "barrier") return 2;
  if (method == "dual_simplex") return 1;
  // Profiled million-column regime; rationale and acceptance in execution contract.
  return method == "auto" && b.compact && b.costs.size() >= 1000000 ? 2 : -1;
}

double level_bound_residual(const Build& b, const Eigen::VectorXd& x, const J& input) {
  // Invertible coordinates do not make absolute tolerances unit invariant.
  // Audit authored meter bounds too; water-coordinate proof in execution contract.
  double residual = 0;
  for (const auto& h : input.at("reservoirs")) for (int t=0;t<static_cast<int>(input.at("periods").size());++t) {
    const double level = value(b.metrics.at(key("level",h.at("id"),t)),x);
    residual = std::max({residual,at(h,"min_level_m",t)-level,level-at(h,"max_level_m",t)});
  }
  return residual;
}

double primal_residual(const Build& b, const Eigen::VectorXd& x, const J& input) {
  if (x.size() != static_cast<Eigen::Index>(b.costs.size()) || !x.allFinite()) return inf;
  double residual = 0;
  for (const auto& row : b.eq) residual = std::max(residual,std::abs(value(row.expr,x)-row.rhs));
  for (const auto& row : b.le) residual = std::max(residual,value(row.expr,x)-row.rhs);
  for (size_t i=0;i<b.costs.size();++i) residual = std::max({residual,b.model.linear_part.vars[i].lb-x[i],x[i]-b.model.linear_part.vars[i].ub});
  for (int i : b.audited_binary_columns) residual = std::max(residual,std::abs(x[i]-std::round(x[i])));
  return std::max(residual,level_bound_residual(b,x,input));
}

size_t project_storage_candidate(const Build& b,const J& input,const Eigen::VectorXd& target,
    engine::LPModel& candidate,double time_limit) {
  // L1 projection onto the original storage rows with fixed integer modes.
  // This restricts only a primal heuristic, not its lower-bound LP; performance.md.
  if(input.at("storage").empty()||input.contains("_rt")||input.contains("_yunnan"))return 0;
  for(const auto& s:input.at("storage"))if(num(s,"charge_min_mw")!=0||num(s,"discharge_min_mw")!=0)return 0;
  engine::LPModel local;
  std::vector<int> index(candidate.c.size(),-1),power;
  for(size_t c=0;c<candidate.vars.size();++c) {
    const auto& name=candidate.vars[c].name;
    const bool is_power=name.starts_with("dis/")||name.starts_with("ch/");
    if(is_power||name.starts_with("energy/")||name.starts_with("dis_on/")||
        name.starts_with("ch_on/")||name.starts_with("storage_hour_mode/")) {
      index[c]=static_cast<int>(local.vars.size());
      local.vars.push_back(candidate.vars[c]);if(is_power)power.push_back(c);
    }
  }
  std::vector<Eigen::Triplet<double>> eq,le;
  std::vector<double> rhs_eq,rhs_le;
  bool valid=true;
  for(const auto* rows:{&b.eq,&b.le})for(const auto& row:*rows) {
    if(!row.name.starts_with("2.6.3.16/")&&!row.name.starts_with("storage_hour_"))continue;
    auto& entries=rows==&b.eq?eq:le;auto& rhs=rows==&b.eq?rhs_eq:rhs_le;
    const int r=static_cast<int>(rhs.size());rhs.push_back(row.rhs-row.expr.constant);
    row.expr.each([&](int c,double a){if(a==0)return;if(index[c]<0)valid=false;else entries.emplace_back(r,index[c],a);});
  }
  if(!valid)return 0;
  for(int c:power) {
    const int dev=static_cast<int>(local.vars.size());
    local.vars.push_back({engine::VarType::Continuous,0,inf,"deviation/"+std::to_string(c)});
    const int r=static_cast<int>(rhs_le.size());rhs_le.push_back(target[c]);rhs_le.push_back(-target[c]);
    le.emplace_back(r,index[c],1);le.emplace_back(r,dev,-1);
    le.emplace_back(r+1,index[c],-1);le.emplace_back(r+1,dev,-1);
  }
  local.c=Eigen::VectorXd::Zero(local.vars.size());local.c.tail(power.size()).setOnes();
  local.A.resize(rhs_le.size(),local.vars.size());local.A.setFromTriplets(le.begin(),le.end());
  local.Aeq.resize(rhs_eq.size(),local.vars.size());local.Aeq.setFromTriplets(eq.begin(),eq.end());
  local.b=Eigen::Map<Eigen::VectorXd>(rhs_le.data(),rhs_le.size());
  local.beq=Eigen::Map<Eigen::VectorXd>(rhs_eq.data(),rhs_eq.size());
  engine::GurobiOptions options;options.method=1;options.threads=1;options.time_limit_sec=time_limit;
  const auto projected=engine::GurobiAdapter(options).solve_lp(local);
  if(!projected.stats.success||projected.x.size()!=local.c.size()||!projected.x.allFinite())return 0;
  for(int c:power) {
    auto& var=candidate.vars[c];
    var.lb=var.ub=std::clamp(projected.x[index[c]],var.lb,var.ub);
  }
  return input.at("storage").size();
}

J current_gurobi_timing() {
  const auto timing=engine::last_gurobi_solve_timing();
  const auto seconds=[](const std::optional<double>& v){return v?J(*v):J(nullptr);};
  return {{"model_import_sec",seconds(timing.model_import_sec)},
    {"optimize_sec",seconds(timing.optimize_sec)},{"result_extract_sec",seconds(timing.result_extract_sec)}};
}

std::optional<engine::SolveResult> certified_integer_repair(Build& b,const J& input,
    const engine::GurobiOptions& options) {
  // L <= z* <= U, with a separately evaluated box support bound. Failed
  // rounding is only a failed heuristic. Full derivation: performance.md.
  const auto start=std::chrono::steady_clock::now();
  const auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();};
  auto relaxation_options=options;
  relaxation_options.time_limit_sec=options.time_limit_sec*.3;
  const double relative_tolerance=std::clamp(options.mip_gap*.5,1e-8,1e-2);
  auto relaxed=engine::GurobiAdapter(relaxation_options).solve_relaxation_lp(b.model.linear_part,relative_tolerance);
  b.gap_certificate={{"policy","lagrangian-box-repair-v1"},{"accepted",false},
    {"relaxation_status",relaxed.stats.status},{"relaxation_sec",elapsed()},
    {"relaxation_barrier_tolerance",relative_tolerance},{"bound_cny",nullptr},{"gap",nullptr}};
  b.gap_certificate["relaxation_solver_timing"]=current_gurobi_timing();
  if(relaxed.x.size()!=b.model.linear_part.c.size()||!relaxed.x.allFinite()||
      relaxed.constraint_duals.size()!=b.model.linear_part.A.rows()+b.model.linear_part.Aeq.rows())return std::nullopt;
  auto repaired_lp=b.model.linear_part;
  for(int c:b.model.binary_idx) {
    auto& var=repaired_lp.vars[c];var.lb=var.ub=std::clamp(std::round(relaxed.x[c]),var.lb,var.ub);
  }
  const double projection_started=elapsed();
  const size_t projected=project_storage_candidate(b,input,relaxed.x,repaired_lp,std::min(2.0,.05*options.time_limit_sec));
  b.gap_certificate["storage_projection_sec"]=elapsed()-projection_started;
  const double load_projection_started=elapsed();
  // Monotone downscaling preserves both box and bus/energy upper bounds.
  // Only a candidate is restricted; the independent bound remains full-model.
  std::map<int,std::vector<const J*>> demand_by_bus;
  Eigen::VectorXd demand=relaxed.x;
  for(const auto& d:input.at("controllable_loads")) {
    demand_by_bus[d.at("bus")].push_back(&d);
    for(int t=0;t<T;++t){const int c=b.col("load_reduction",d.at("id"),t);
      demand[c]=std::clamp(demand[c],repaired_lp.vars[c].lb,repaired_lp.vars[c].ub);}
  }
  const auto reduce=[](double limit,double total){return total>limit?
    std::max(0.0,std::nextafter(limit/total,0.0)):1.0;};
  for(const auto& node:input.at("buses")) {
    const auto found=demand_by_bus.find(node.at("id"));if(found==demand_by_bus.end())continue;
    for(int t=0;t<T;++t) {
      double sum=0;for(const auto* d:found->second)sum+=demand[b.col("load_reduction",d->at("id"),t)];
      const double scale=reduce(at(node,"load_mw",t),sum);
      for(const auto* d:found->second)demand[b.col("load_reduction",d->at("id"),t)]*=scale;
    }
  }
  for(const auto& d:input.at("controllable_loads")) {
    double energy=0;for(int t=0;t<96;++t)energy+=demand[b.col("load_reduction",d.at("id"),t)]*num(input.at("periods")[t],"duration_hr");
    const double scale=reduce(num(d,"max_day_reduction_mwh"),energy);
    for(int t=0;t<T;++t){const int c=b.col("load_reduction",d.at("id"),t);auto& var=repaired_lp.vars[c];
      var.lb=var.ub=demand[c]*(t<96?scale:1.0);}
  }
  b.gap_certificate["load_projection_sec"]=elapsed()-load_projection_started;
  b.gap_certificate["candidate_projection_sec"]=elapsed()-projection_started;
  b.gap_certificate["storage_candidate_assets"]=projected;
  b.gap_certificate["load_candidate_assets"]=input.at("controllable_loads").size();
  b.gap_certificate["storage_candidate_policy"]="fixed-mode-l1-projection";
  // The certificate reads the original model; repair owns a separate LP copy.
  auto certification=std::async(std::launch::async,[&]{
    const auto began=std::chrono::steady_clock::now();
    const auto box=certificate_box(b,input);
    auto lower=box?detail::box_dual_lower_bound(b.model.linear_part,relaxed.constraint_duals,box->first,box->second):std::nullopt;
    return std::pair{lower,std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()};
  });
  auto repair_options=options;repair_options.method=2;repair_options.crossover=0;
  repair_options.time_limit_sec=options.time_limit_sec*.4;
  const double repair_started=elapsed();
  auto repaired=engine::GurobiAdapter(repair_options).solve_lp(repaired_lp);
  b.gap_certificate["repair_solver_timing"]=current_gurobi_timing();
  b.gap_certificate["repair_sec"]=elapsed()-repair_started;
  const auto [lower,certificate_sec]=certification.get();
  b.gap_certificate["certificate_sec"]=certificate_sec;
  b.gap_certificate["repair_status"]=repaired.stats.status;
  const double residual=primal_residual(b,repaired.x,input);
  b.gap_certificate["max_residual"]=residual<inf?J(residual):J(nullptr);
  if(!lower||residual>tolerance)return std::nullopt;
  const double upper=b.model.linear_part.c.dot(repaired.x)+b.constant_cost;
  const double bound=std::nextafter(*lower+b.constant_cost,-std::numeric_limits<double>::infinity());
  const double gap=(upper-bound)/std::max(1e-10,std::abs(upper));
  b.gap_certificate["bound_cny"]=bound;b.gap_certificate["upper_cny"]=upper;b.gap_certificate["gap"]=gap;
  if(!std::isfinite(gap)||gap<0||gap>options.mip_gap)return std::nullopt;
  b.gap_certificate["accepted"]=true;
  repaired.stats.success=true;repaired.stats.status="Optimality gap reached";
  repaired.stats.mip_gap=gap;repaired.stats.dual_objective=*lower;
  repaired.stats.objective=upper-b.constant_cost;repaired.stats.runtime_sec=elapsed();
  repaired.constraint_duals.resize(0);
  b.solver_timing={{"scope","certified-relaxation-and-integer-repair; certificate overlaps repair"},
    {"total_wall_sec",elapsed()},{"relaxation_sec",b.gap_certificate["relaxation_sec"]},
    {"repair_sec",b.gap_certificate["repair_sec"]},{"certificate_sec",certificate_sec}};
  return repaired;
}

engine::SolveResult solve_scaled(Build& b, const J& input) {
  if (input.at("execution").value("solver",std::string("highs")) == "gurobi") {
    // Same sparse model and row ordering; options and parity ledger in execution contract.
    engine::GurobiOptions options;
    options.time_limit_sec = num(input.at("execution"),"time_limit_sec");
    options.mip_gap = num(input.at("execution"),"mip_gap");
    options.threads = b.stage == "lmp" ? 1 : input.at("execution").value("threads",0);
    options.method = gurobi_method(b,input);
    const auto strategy=input.at("execution").value("large_mip_strategy",std::string("auto"));
    const bool try_repair=b.stage=="scuc" && !input.contains("_rt") && !input.contains("_yunnan") &&
      !b.model.binary_idx.empty() && b.model.integer_idx.empty() && b.compact && options.mip_gap>0 &&
      (strategy=="certified_repair" || (strategy=="auto" && b.costs.size()>=1000000));
    if(try_repair) {
      const auto began=std::chrono::steady_clock::now();
      if(auto repaired=certified_integer_repair(b,input,options))return std::move(*repaired);
      const double spent=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
      b.gap_certificate["fallback"]="original_milp";b.gap_certificate["attempt_sec"]=spent;
      options.time_limit_sec=std::max(.001,options.time_limit_sec-spent);
    }
    // Pure LP consumers need X/Pi, not a basis; Gurobi Crossover=0 rationale
    // and dual/feasibility gates are in southern_execution_contract.md.
    if (options.method == 2 && b.model.binary_idx.empty()) options.crossover = 0;
    const std::string start_policy = input.at("execution").value("mip_start",std::string("auto"));
    const bool use_start = !try_repair && !b.model.binary_idx.empty() && b.compact &&
      (start_policy == "enabled" || (start_policy == "auto" && b.costs.size() >= 1000000));
    double start_wall = 0;
    if (use_start) {
      // Primal completion is an incumbent heuristic, not a model restriction.
      // Full bounds are restored before MILP; proof/budget in execution contract.
      const auto begin = std::chrono::steady_clock::now();
      Eigen::VectorXd candidate;
      double spent_budget = 0;
      if (b.stage == "sced") {
        candidate.resize(b.costs.size());
        for (const auto& [name,col] : b.columns) candidate[col] = b.fixed.at(name);
        b.primal_start["status"] = "preceding_scuc";
      } else {
        auto options_seed = options;
        if (options_seed.method == 2) options_seed.crossover = 0;
        options_seed.time_limit_sec = std::min(60.0,0.1*options.time_limit_sec);
        const auto authored = b.model.linear_part.vars;
        for (int i : b.model.binary_idx) {
          auto& var = b.model.linear_part.vars[i];
          const double v = var.name.rfind("u/",0) == 0 ? var.ub : var.lb;
          var.lb = var.ub = v;
        }
        engine::SolveResult seed;
        try { seed = engine::GurobiAdapter(options_seed).solve_lp(b.model.linear_part); }
        catch (...) { b.model.linear_part.vars = authored; throw; }
        b.model.linear_part.vars = authored;
        candidate = std::move(seed.x); spent_budget = options_seed.time_limit_sec;
        b.primal_start["status"] = seed.stats.status;
      }
      const double residual = primal_residual(b,candidate,input);
      const bool accepted = residual <= tolerance;
      if (accepted) {
        // Gurobi GRB_UNDEFINED=1e101 requests completion of unspecified columns.
        // Submit only audited integer states: original-unit 1e-6 acceptance can
        // otherwise violate the adapter's tighter 1e-8 scaled feasibility gate.
        b.model.initial_solution = Eigen::VectorXd::Constant(b.costs.size(),1e101);
        for (int i : b.model.binary_idx) b.model.initial_solution[i] = std::round(candidate[i]);
      }
      b.primal_start["accepted"] = accepted;
      b.primal_start["submission"] = accepted ? "partial_integer_completion" : "none";
      b.primal_start["max_residual"] = residual < inf ? J(residual) : J(nullptr);
      start_wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      b.primal_start["runtime_sec"] = start_wall;
      options.time_limit_sec -= spent_budget;
    }
    const auto environment_start = std::chrono::steady_clock::now();
    engine::GurobiAdapter adapter(options);
    const double environment_sec = std::chrono::duration<double>(std::chrono::steady_clock::now()-environment_start).count();
    if (!adapter.available()) throw std::runtime_error("Gurobi unavailable: library or license initialization failed; no solver substitution performed");
    auto result = b.stage == "lmp" ? adapter.solve_pricing_lp(b.model.linear_part) :
      b.model.binary_idx.empty() ? adapter.solve_lp(b.model.linear_part) : adapter.solve_milp(b.model);
    const auto timing = engine::last_gurobi_solve_timing();
    const auto seconds = [](const std::optional<double>& v) { return v ? J(*v) : J(nullptr); };
    b.solver_timing = {{"scope","gurobi-final-solve-wall; optimize includes presolve/root/search and pending model updates"},
      {"environment_sec",environment_sec},{"model_import_sec",seconds(timing.model_import_sec)},
      {"optimize_sec",seconds(timing.optimize_sec)},{"result_extract_sec",seconds(timing.result_extract_sec)},
      {"primal_start_sec",start_wall},{"presolve_sec",nullptr},{"search_sec",nullptr}};
    result.stats.runtime_sec += start_wall;
    if(try_repair)result.stats.runtime_sec+=b.gap_certificate.value("attempt_sec",0.0);
    return result;
  }
  if (b.stage == "lmp") return engine::HighsAdapter{}.solve_pricing_lp(b.model.linear_part,num(input.at("execution"),"time_limit_sec"));
  if (b.model.binary_idx.empty()) return engine::HighsAdapter{}.solve_lp(b.model.linear_part);
  engine::BCOptions options;
  options.time_limit_sec = num(input.at("execution"), "time_limit_sec");
  options.gap_tol = num(input.at("execution"), "mip_gap");
  if (input.at("execution").value("solver",std::string("highs")) == "native") {
    // Native tree ownership with the production HiGHS LP kernel; no strict MIP
    // delegation or automatic root pipeline. Algorithm ledger: execution contract.
    options.strict_highs_mip_contract = false;
    options.auto_highs_root_pipeline = false;
    options.lp_kernel_backend = engine::LpKernelBackend::HiGHS;
    const auto profile = input.at("execution").value("native_root_cuts",std::string("default"));
    if (profile == "enhanced") {
      // Bounded separation experiment, not new inequalities. Rationale and
      // admission limits: execution contract, Native Root Cut Experiments.
      options.root_cut_rounds = 20;
      options.cuts_per_round = 100;
      options.root_cut_audit_force_separation = true;
      // IEEE118 has about 80k presolved rows; retain the very-large 3x30
      // adaptive cap and reject roots beyond 100k. See admission audit there.
      options.cut_budget_xlarge_row_threshold = 100000;
    }
    // Preserve the public B&C diagnostics discarded by the generic adapter.
    // Identical full-MIP entry point and result conversion; LP dispatch above
    // still uses the pricing-capable LP adapter.
    auto bc = engine::solve_milp_bc(b.model,options);
    const auto& s = bc.bc_stats;
    const auto bound = [&](double v) -> J {
      return std::isfinite(v) && std::abs(v) < inf ? J(v+b.constant_cost) : J(nullptr);
    };
    b.native_diagnostics = {{"profile",profile},{"collection_scope",s.collection_scope},
      {"cut_diagnostics_available",s.cut_diagnostics_available},
      {"requested_root_rounds",options.root_cut_rounds},{"requested_cuts_per_round",options.cuts_per_round},
      {"force_separation",options.root_cut_audit_force_separation},
      {"root_row_admission_limit",options.cut_budget_xlarge_row_threshold},
      {"root_presolved_rows",s.root_presolved_rows > 0 ? J(s.root_presolved_rows) : J(nullptr)},
      {"cuts_added",s.cut_diagnostics_available ? J(s.cuts_added) : J(nullptr)},
      {"root_source_cuts_added",s.cut_diagnostics_available ? J(s.root_cuts_added) : J(nullptr)},
      {"rejected_nonmoving_rows",s.cut_diagnostics_available ? J(s.root_cut_rows_rejected_nonmoving) : J(nullptr)},
      {"nodes_explored",s.nodes_explored},
      {"lp_solves",s.lp_solve_count_available ? J(s.lp_solves) : J(nullptr)},
      {"incumbent_updates",s.incumbent_timeline_available ? J(s.incumbent_updates) : J(nullptr)},
      {"best_bound_cny",bound(s.best_bound)}, {"best_incumbent_cny",bound(s.best_obj)}};
    engine::SolveResult result;
    result.x = std::move(bc.x);
    result.stats = std::move(bc.stats);
    result.stats.solver_name = "NativeBranchAndCut";
    return result;
  }
  return engine::StrictHighsBranchAndCutAdapter{options}.solve_milp(b.model);
}

void restore_duals(const Build& b,engine::SolveResult& result) {
  // Lift submitted row duals into the authored row order; omitted valid
  // inequalities admit zero multipliers. Pricing keeps all authored rows.
  if (b.submitted_rows.size() != b.le.size() &&
      result.constraint_duals.size() == static_cast<Eigen::Index>(b.submitted_rows.size()+b.eq.size())) {
    Eigen::VectorXd original = Eigen::VectorXd::Zero(b.le.size()+b.eq.size());
    for(size_t i=0;i<b.submitted_rows.size();++i) original[b.submitted_rows[i]] = result.constraint_duals[i];
    original.tail(b.eq.size()) = result.constraint_duals.tail(b.eq.size());
    result.constraint_duals = std::move(original);
  }
  // For S*A*x=S*b, the original multiplier is S*lambda_scaled.
  // See the reservoir-coordinate derivation in the execution contract.
  if (result.constraint_duals.size() == static_cast<Eigen::Index>(b.le.size()+b.eq.size())) {
    for (size_t i=0;i<b.le.size();++i) result.constraint_duals[i] *= b.le[i].solver_scale;
    for (size_t i=0;i<b.eq.size();++i) result.constraint_duals[b.le.size()+i] *= b.eq[i].solver_scale;
  }
}

engine::SolveResult solve_once(Build& b, const J& input) {
  auto result=solve_scaled(b,input);restore_duals(b,result);return result;
}

engine::SolveResult solve(Build& b, const J& input) {
  if(b.stage=="lmp" && b.costs.size()>=1000000 && input.at("execution").value("solver",std::string("highs"))=="gurobi") {
    // At most one large pair per process. Separate environments and timing
    // collectors share only an immutable ordered LP; performance.md, v2.
    static std::mutex large_pricing_mutex;
    const auto began=std::chrono::steady_clock::now();
    std::lock_guard lock(large_pricing_mutex);
    const auto independent=[&]{
      const auto start=std::chrono::steady_clock::now();
      engine::GurobiOptions options;options.time_limit_sec=num(input.at("execution"),"time_limit_sec");
      engine::GurobiAdapter adapter(options);
      auto result=adapter.solve_pricing_lp(b.model.linear_part);
      const auto timing=engine::last_gurobi_solve_timing();
      restore_duals(b,result);
      const auto seconds=[](const std::optional<double>& v){return v?J(*v):J(nullptr);};
      J report={{"scope","independent-pricing-wall"},
        {"model_import_sec",seconds(timing.model_import_sec)},{"optimize_sec",seconds(timing.optimize_sec)},
        {"result_extract_sec",seconds(timing.result_extract_sec)},
        {"wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
      return std::pair{std::move(result),std::move(report)};
    };
    auto verification=std::async(std::launch::async,independent);
    auto [result,timing]=independent();
    auto [repeat,repeat_timing]=verification.get();
    const auto objective=[&](const Eigen::VectorXd& x){return x.size()==b.model.linear_part.c.size()&&x.allFinite()?
      b.model.linear_part.c.dot(x):std::numeric_limits<double>::infinity();};
    b.price_consistency=detail::check_price_duals(result,repeat,b.le.size()+b.eq.size(),
      primal_residual(b,result.x,input),primal_residual(b,repeat.x,input),
      std::abs(objective(result.x)-objective(repeat.x)));
    b.price_consistency["policy"]="ordered-lp-barrier-8-v2";
    b.price_consistency["algorithm"]="barrier";b.price_consistency["threads"]=8;
    b.price_consistency["execution"]="concurrent_independent_solves";
    b.price_consistency["repeat_wall_sec"]=repeat_timing["wall_sec"];
    b.price_consistency["repeat_solver_timing"]=repeat_timing;
    b.solver_timing=timing;
    result.stats.runtime_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    return result;
  }
  if(b.stage=="lmp" && input.at("execution").value("solver",std::string("highs"))=="gurobi") {
    // Small (<1e6 columns) deterministic pricing pair. RATIONALE: the two fresh
    // solves of the same ordered LP share only immutable input, so running them
    // concurrently halves the stage wall while the strict zero-dual-difference
    // admission (check_price_duals) is unchanged. Mirrors ordered-lp-barrier-8-v2
    // above; solve_pricing_lp still pins single-thread dual simplex, Seed default.
    // Time-limit semantics: both solves start together and each holds the full
    // requested limit; the retired sequential variant gave the repeat only the
    // remainder of the budget. A failed primary no longer skips the repeat; the
    // consistency check fails on optimality_not_proven either way, so the
    // exported prices_valid contract is unchanged. performance.md,
    // 恢复并发扩展与定价并发双解. HiGHS/native stay on the sequential path below.
    const auto began=std::chrono::steady_clock::now();
    const auto independent=[&]{
      const auto start=std::chrono::steady_clock::now();
      engine::GurobiOptions options;
      options.time_limit_sec=num(input.at("execution"),"time_limit_sec");
      options.mip_gap=num(input.at("execution"),"mip_gap");
      const auto environment_start=std::chrono::steady_clock::now();
      engine::GurobiAdapter adapter(options);
      const double environment_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-environment_start).count();
      if(!adapter.available()) throw std::runtime_error("Gurobi unavailable: library or license initialization failed; no solver substitution performed");
      auto result=adapter.solve_pricing_lp(b.model.linear_part);
      const auto timing=engine::last_gurobi_solve_timing();
      restore_duals(b,result);
      const auto seconds=[](const std::optional<double>& v){return v?J(*v):J(nullptr);};
      J report={{"scope","concurrent-independent-pricing-wall; each solve holds the full requested time limit"},
        {"environment_sec",environment_sec},
        {"model_import_sec",seconds(timing.model_import_sec)},{"optimize_sec",seconds(timing.optimize_sec)},
        {"result_extract_sec",seconds(timing.result_extract_sec)},
        {"primal_start_sec",0},{"presolve_sec",nullptr},{"search_sec",nullptr},
        {"wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
      return std::pair{std::move(result),std::move(report)};
    };
    auto verification=std::async(std::launch::async,independent);
    auto [result,timing]=independent();
    auto [repeat,repeat_timing]=verification.get();
    const auto objective=[&](const Eigen::VectorXd& x){return x.size()==b.model.linear_part.c.size()&&x.allFinite()?
      b.model.linear_part.c.dot(x):std::numeric_limits<double>::infinity();};
    b.price_consistency=detail::check_price_duals(result,repeat,b.le.size()+b.eq.size(),
      primal_residual(b,result.x,input),primal_residual(b,repeat.x,input),
      std::abs(objective(result.x)-objective(repeat.x)));
    b.price_consistency["execution"]="concurrent_independent_solves";
    b.price_consistency["repeat_wall_sec"]=repeat_timing["wall_sec"];
    b.price_consistency["repeat_solver_timing"]=repeat_timing;
    b.solver_timing=timing;
    result.stats.runtime_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    return result;
  }
  const auto start = std::chrono::steady_clock::now();
  auto result = solve_once(b,input);
  if (b.stage != "lmp") return result;
  const J primary_timing = b.solver_timing;
  const auto repeat_start = std::chrono::steady_clock::now();
  const double remaining = num(input.at("execution"),"time_limit_sec") -
    std::chrono::duration<double>(repeat_start-start).count();
  engine::SolveResult repeat;
  b.solver_timing = nullptr;
  if (result.stats.success && remaining > 0) {
    J repeat_input = input;
    repeat_input["execution"]["time_limit_sec"] = remaining;
    repeat = solve_once(b,repeat_input);
  } else repeat.stats.status = remaining <= 0 ? "Time limit reached" : "primary_solve_failed";
  const double repeat_wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-repeat_start).count();
  const auto objective = [&](const Eigen::VectorXd& x) {
    return x.size() == b.model.linear_part.c.size() && x.allFinite() ?
      b.model.linear_part.c.dot(x) : std::numeric_limits<double>::infinity();
  };
  b.price_consistency = detail::check_price_duals(result,repeat,b.le.size()+b.eq.size(),
    primal_residual(b,result.x,input),primal_residual(b,repeat.x,input),
    std::abs(objective(result.x)-objective(repeat.x)));
  b.price_consistency["repeat_wall_sec"] = repeat_wall;
  b.price_consistency["repeat_solver_timing"] = b.solver_timing;
  b.solver_timing = primary_timing;
  result.stats.runtime_sec += repeat.stats.runtime_sec;
  return result;
}

J stage_result(const Build& b, const engine::SolveResult& solved, const J& j) {
  const int T=static_cast<int>(j.at("periods").size());
  const int energy_points=j.contains("_rt")?T:96;
  const auto failed = detail::southern_solver_status(solved.stats.status, false, solved.stats.mip_gap);
  J out = {{"stage", b.stage}, {"solver_status", solved.stats.status}, {"solver", solved.stats.solver_name},
    {"solver_success", solved.stats.success}, {"variables", b.costs.size()}, {"binary_variables", b.model.binary_idx.size()},
    {"equalities", b.eq.size()}, {"inequalities", b.le.size()}, {"runtime_sec", solved.stats.runtime_sec},
    {"nonzeros", b.size_report.at("nonzeros")},
    {"formulation",b.compact ? "compact" : "reference"}, {"compact_units",b.compact_units.size()},
    {"compact_storage",b.compact_storage.size()},
    {"projected_commitment_units",b.projected_commitment.size()},
    {"reservoir_scaling",b.reservoir_scaling ? "energy_coordinate" : "original"},
    {"primal_start",b.primal_start}, {"model_size",b.size_report}, {"native_diagnostics",b.native_diagnostics},
    {"solver_timing",b.solver_timing},
    {"price_consistency",b.price_consistency},
    {"gap_certificate",b.gap_certificate},
    {"prices_valid",false},
    {"assembly_template",b.assembly_template},
    {"lp_algorithm",b.stage == "lmp" ? (b.price_consistency.is_object()?b.price_consistency.value("algorithm",std::string("dual_simplex")):"dual_simplex") : j.at("execution").value("solver",std::string("highs")) != "gurobi" ? "solver_default" : gurobi_method(b,j) == 2 ? "barrier" : gurobi_method(b,j) == 1 ? "dual_simplex" : "solver_default"},
    {"mip_gap", solved.stats.success ? J(solved.stats.mip_gap) : J(nullptr)},
    {"requested_solver",j.at("execution").value("solver",std::string("highs"))},
    {"requested_time_limit_sec",num(j.at("execution"),"time_limit_sec")},
    {"requested_mip_gap",num(j.at("execution"),"mip_gap")},
    {"requested_threads",j.at("execution").value("threads",0)},
    {"optimality_proven", false}, {"solution_quality",failed.quality}, {"limit_reached",failed.limit_reached}, {"feasible", false}};
  if (solved.x.size() != static_cast<Eigen::Index>(b.costs.size()) || !solved.x.allFinite()) return out;
  double residual = 0;
  J families = J::object(), binding = J::array();
  struct FamilyAudit {int rows{0};double max_violation{0};};
  std::map<std::string_view,FamilyAudit> family_audit;
  size_t binding_count = 0;
  const auto audit = [&](const Row& row, bool equality) {
    const double lhs = value(row.expr, solved.x), delta = lhs-row.rhs;
    const double violation = equality ? std::abs(delta) : std::max(0.0, delta);
    residual = std::max(residual, violation);
    const auto family=std::string_view(row.name).substr(0,row.name.find('/'));
    auto& count=family_audit[family];++count.rows;count.max_violation=std::max(count.max_violation,violation);
    if (violation > tolerance || (!equality && std::abs(delta) <= tolerance)) {
      ++binding_count;
      if (binding.size() < 1000) binding.push_back({{"name", row.name}, {"lhs", lhs}, {"rhs", row.rhs}, {"violation", violation}});
    }
  };
  for (const auto& r : b.eq) audit(r, true);
  for (const auto& r : b.le) audit(r, false);
  for(const auto& [family,count]:family_audit)families[std::string(family)]={{"rows",count.rows},{"max_violation",count.max_violation}};
  for (size_t i = 0; i < b.costs.size(); ++i) {
    const auto& v = b.model.linear_part.vars[i];
    residual = std::max({residual, v.lb-solved.x[i], solved.x[i]-v.ub});
  }
  for (int i : b.audited_binary_columns)
    residual = std::max(residual,std::abs(solved.x[i]-std::round(solved.x[i])));
  const double level_residual = level_bound_residual(b,solved.x,j);
  residual = std::max(residual,level_residual);
  // Recover and audit every projected history/class row in original units,
  // including big-M inequalities. See the exact-projection proof in the contract.
  std::map<int,std::array<std::vector<double>,3>> recovered_classes;
  double reconstructed_residual = 0;
  const double max_time = num(j.at("periods")[T-1],"start_minute")+1000001;
  for (const auto& g : j.at("generators")) {
    const int id = g.at("id");
    if (!b.compact_units.count(id)) continue;
    auto& classes = recovered_classes[id];
    double offline = num(g,"initial_on") ? 0 : num(g,"initial_state_minutes");
    double previous_u = num(g,"initial_on");
    for (int t = 0; t < T; ++t) {
      const double u = solved.x[b.col("u",id,t)], start = solved.x[b.col("start",id,t)];
      const double elapsed = t ? num(j.at("periods")[t],"start_minute")-num(j.at("periods")[t-1],"start_minute") : 0;
      const double downtime = offline+elapsed*(1-previous_u);
      const int category = downtime < num(g,"warm_after_minutes") ? 0 : downtime < num(g,"cold_after_minutes") ? 1 : 2;
      const double lo[3] = {0,num(g,"warm_after_minutes"),num(g,"cold_after_minutes")};
      const double hi[3] = {lo[1]-1,lo[2]-1,max_time};
      for (int k = 0; k < 3; ++k) {
        const double sk = k == category ? start : 0;
        classes[k].push_back(sk);
        reconstructed_residual = std::max({reconstructed_residual,-downtime+max_time*sk-max_time+lo[k],
          downtime+max_time*sk-max_time-hi[k],-sk,sk-1,std::abs(sk-std::round(sk))});
      }
      offline = u > .5 ? 0 : downtime;
      reconstructed_residual = std::max({reconstructed_residual,-offline,offline-max_time,
        offline+max_time*u-max_time,offline-downtime-max_time*u,downtime-offline-max_time*u});
      previous_u = u;
    }
  }
  residual = std::max(residual,reconstructed_residual);
  out["reconstructed_max_residual"] = reconstructed_residual;
  out["max_residual"] = residual; out["constraint_families"] = families; out["binding_constraints"] = binding;
  out["binding_constraint_count"] = binding_count;
  out["binding_constraints_truncated"] = binding_count > binding.size();
  out["feasible"] = solved.stats.success && residual <= tolerance;
  const auto quality = detail::southern_solver_status(solved.stats.status, out.at("feasible").get<bool>(), solved.stats.mip_gap);
  out["solution_quality"] = quality.quality;
  out["optimality_proven"] = quality.optimality_proven;
  out["objective"] = b.model.linear_part.c.dot(solved.x);
  J costs = J::object();std::map<std::string_view,double> category_costs;
  for (size_t i = 0; i < b.costs.size(); ++i) {
    category_costs[b.cost_categories[i]]+=b.costs[i]*solved.x[i];
  }
  for(const auto& [category,cost]:category_costs)costs[std::string(category)]=cost;
  out["objective_terms"] = costs;
  const auto rows = [&](const char* table, const std::vector<std::pair<std::string,std::string>>& fields) {
    J result = J::array();
    for (const auto& source : j.at(table)) {
      J row = {{"id", source.at("id")}, {"name", source.at("name")}};
      if (source.contains("bus")) row["bus"] = source.at("bus");
      if (source.contains("kind")) row["kind"] = source.at("kind");
      for (const auto& [field, prefix] : fields) {
        row[field] = J::array();
        row[field].get_ref<J::array_t&>().reserve(T);
        const std::vector<int>* slots=nullptr;
        if(!b.reference) {
          const auto family=b.column_slots.find(prefix);
          if(family!=b.column_slots.end()) {
            const auto entity=family->second.find(source.at("id"));
            if(entity!=family->second.end())slots=&entity->second;
          }
        }
        for (int t = 0; t < T; ++t) {
          if(slots && static_cast<size_t>(t)<slots->size() && (*slots)[t]>=0) {
            row[field].push_back(solved.x[(*slots)[t]]);continue;
          }
          const auto k = key(prefix.c_str(), source.at("id"), t);
          if (b.columns.count(k)) row[field].push_back(solved.x[b.columns.at(k)]);
          else if (b.metrics.count(k)) row[field].push_back(value(b.metrics.at(k), solved.x));
          else row[field].push_back(nullptr);
        }
      }
      result.push_back(std::move(row));
    }
    return result;
  };
  out["generators"] = rows("generators", {{"power_mw", "p"}, {"online", "u"}, {"primary_reserve_mw", "primary"}, {"start", "start"}, {"stop", "stop"},
    {"hot_start", "start0"}, {"warm_start", "start1"}, {"cold_start", "start2"}, {"trajectory_mw", "trajectory"}, {"renewable_deviation_mw", "renewable_deviation"}});
  if (j.contains("_yunnan") && b.stage != "scuc") {
    const auto regulation=rows("generators",{{"secondary_up_mw","secondary_up"},{"secondary_down_mw","secondary_down"},{"stable","stable"}});
    for(size_t i=0;i<regulation.size();++i) for(const char* field:{"secondary_up_mw","secondary_down_mw","stable"})
      out["generators"][i][field]=regulation[i].at(field);
  }
  for (auto& g : out["generators"]) if (const auto found = recovered_classes.find(g.at("id")); found != recovered_classes.end()) {
    g["hot_start"] = found->second[0]; g["warm_start"] = found->second[1]; g["cold_start"] = found->second[2];
  }
  out["storage"] = rows("storage", {{"discharge_mw", "dis"}, {"charge_mw", "ch"}, {"energy_mwh", "energy"}});
  out["controllable_loads"] = rows("controllable_loads", {{"reduction_mw", "load_reduction"}});
  if(j.contains("_yunnan")&&b.stage!="scuc")for(const auto& a:j.at("_yunnan").at("config").value("independent_units",J::array()))
    for(auto& row:out[a.at("mode")=="storage"?"storage":"controllable_loads"])if(row.at("id")==a.at("resource_id")) {
      row["agc_reserved_mw"]=J::array();row["energy_market_eligible"]=J::array();
      for(int t=0;t<T;++t){const double award=yunnan::required_capacity(j.at("_yunnan"),a.at("id"),t);row["agc_reserved_mw"].push_back(award);row["energy_market_eligible"].push_back(award<=1e-6);}
    }
  double reduced_energy = 0;
  for (const auto& d : out["controllable_loads"]) for (int t = 0; t < energy_points; ++t) reduced_energy += at(d, "reduction_mw", t)*num(j.at("periods")[t],"duration_hr");
  out["day_load_reduction_mwh"] = reduced_energy;
  out["branches"] = rows("branches", {{"power_mw", "flow"}, {"slack_plus_mw", "line_slack_plus"}, {"slack_minus_mw", "line_slack_minus"}});
  for (size_t i = 0; i < out["branches"].size(); ++i) {
    auto& line = out["branches"][i]; const auto& input = j.at("branches")[i];
    line["overload_mw"] = J::array();
    // Physical signed-limit excess, independent of auxiliary penalty variables.
    for (int t = 0; t < T; ++t) {
      const double flow = at(line, "power_mw", t), available = at(input, "available", t);
      line["overload_mw"].push_back(std::max({0.0, flow-available*at(input, "max_mw", t), available*at(input, "min_mw", t)-flow}));
    }
  }
  out["sections"] = rows("sections", {{"power_mw", "section_flow"}, {"slack_plus_mw", "section_slack_plus"}, {"slack_minus_mw", "section_slack_minus"}});
  out["dc_links"] = rows("dc_links", {{"power_mw", "dc"}, {"up", "dc_up"}, {"down", "dc_down"}});
  out["reservoirs"] = rows("reservoirs", {{"level_m", "level"}, {"spill_m3_s", "spill"}, {"release_m3_s", "release"}});
  out["trades"] = rows("trades", {{"power_mw", "trade"}});
  for (auto& trade : out["trades"]) {
    const auto k = key("priority_shortfall_mwh", trade.at("id"), 0);
    trade["priority_shortfall_mwh"] = b.columns.count(k) ? J(solved.x[b.columns.at(k)]) : J(nullptr);
    if(j.contains("_rt")){trade.erase("priority_shortfall_mwh");trade["priority_shortfall_mw"]=J::array();for(int t=0;t<T;++t){const auto rtkey=key("rt_priority_slack",trade.at("id"),t);trade["priority_shortfall_mw"].push_back(b.columns.count(rtkey)?solved.x[b.columns.at(rtkey)]:0.0);}}
  }
  double day_energy_cost = 0, generation_mwh = 0;
  for (const auto& g : j.at("generators")) for (int t = 0; t < energy_points; ++t) {
    generation_mwh += solved.x[b.col("p", g.at("id"), t)]*num(j.at("periods")[t],"duration_hr");
    for (size_t k = 0; k < g.at("segments").size(); ++k) {
      const int i = b.col(("segment"+std::to_string(k)).c_str(), g.at("id"), t);
      day_energy_cost += b.costs[i]*solved.x[i];
    }
  }
  out["day_energy_bid_cost"] = day_energy_cost; out["day_generation_mwh"] = generation_mwh;
  if(j.contains("_rt")) {
    out["window_generation_mwh"]=generation_mwh;out["window_energy_bid_cost_cny"]=day_energy_cost;
    out.erase("day_generation_mwh");out.erase("day_energy_bid_cost");
    out["window_load_reduction_mwh"]=reduced_energy;out.erase("day_load_reduction_mwh");
    out["accident_reserve"]=J::array();
    for(const auto& area:j.at("_rt").at("accident_reserve")){J row={{"id",area.at("id")},{"shortage_mw",J::array()}};for(int t=0;t<T;++t)row["shortage_mw"].push_back(solved.x[b.col("accident_slack",area.at("id"),t)]);out["accident_reserve"].push_back(row);}
    for(auto& g:out["generators"]){g["accident_reserve_mw"]=J::array();for(int t=0;t<T;++t){const auto k=key("accident_reserve",g.at("id"),t);g["accident_reserve_mw"].push_back(b.columns.count(k)?J(solved.x[b.columns.at(k)]):J(nullptr));}}
  }
  out["buses"] = J::array();
  const bool duals = b.stage == "lmp" && solved.stats.success &&
    b.price_consistency.is_object() && b.price_consistency.value("passed",false) &&
    solved.constraint_duals.size() == static_cast<Eigen::Index>(b.eq.size()+b.le.size()) &&
    solved.constraint_duals.allFinite() && residual <= tolerance;
  out["prices_valid"] = duals && residual <= tolerance;
  for (const auto& node : j.at("buses")) {
    J row = {{"id", node.at("id")}, {"name", node.at("name")}, {"lmp_per_mwh", J::array()}, {"node_imbalance_mw", J::array()}};
    row["deficit_mw"] = J::array(); row["surplus_mw"] = J::array();
    for (int t = 0; t < T; ++t) {
      const auto balance = b.balance_rows.at(key("bus", node.at("id"), t));
      row["node_imbalance_mw"].push_back(value(b.eq.at(balance).expr, solved.x) - b.eq.at(balance).rhs);
      for (const auto* family : {"deficit", "surplus"}) {
        double power=0;
        if(b.reference) {
          const auto c=b.columns.find(key(family,node.at("id"),t));
          if(c!=b.columns.end())power=std::max(0.0,solved.x[c->second]);
        } else if(b.column_slots.contains(family))power=std::max(0.0,solved.x[b.col(family,node.at("id"),t)]);
        row[std::string(family)+"_mw"].push_back(power);
      }
      const int index = static_cast<int>(b.le.size()) + balance;
      // 2.6.6 uses the original balance multiplier. Reserve substitution added
      // balance to the upward row and subtracted it from the downward row;
      // undo this row operation (HiGHS upper-row shadows are -mu).
      const auto [up, down] = b.reserve_rows.at({node.at("area"), t});
      row["lmp_per_mwh"].push_back(duals ? J((solved.constraint_duals[index]+solved.constraint_duals[up]-solved.constraint_duals[down])/
        num(j.at("periods")[t], "weight_hr")) : J(nullptr));
    }
    out["buses"].push_back(std::move(row));
  }
  return out;
}

struct ACObservation {
  bool converged{false};
  std::map<std::string, double> violation_functions;
  double losses{0};
  double residual{0};
  std::map<int, double> active_adjustment_mw;
};

ACObservation observe_ac(const J& j, const std::map<std::string,double>& x, int t) {
  HybridPowerSystem sys; sys.base_mva = sys.ac.base_mva = num(j, "base_mva");
  std::map<int,size_t> positions;
  for (const auto& n : j.at("buses")) {
    ACBus bus; bus.index = n.at("id"); bus.area = n.at("area"); bus.base_kv = n.at("base_kv");
    bus.pd_mw = at(n, "load_mw", t); bus.qd_mvar = at(n, "q_load_mvar", t);
    bus.gs_mw = n.value("gs_mw", 0.0); bus.bs_mvar = n.value("bs_mvar", 0.0);
    bus.vmin_pu = n.at("vmin_pu"); bus.vmax_pu = n.at("vmax_pu");
    positions[bus.index] = sys.ac.buses.size(); sys.ac.buses.push_back(bus);
  }
  const auto injection = [&](int bus, double p) { sys.ac.buses.at(positions.at(bus)).pd_mw -= p; };
  for (const auto& e : j.at("external_schedules")) injection(e.at("bus"), at(e, "power_mw", t));
  for (const auto& d : j.at("controllable_loads")) injection(d.at("bus"), x.at(key("load_reduction", d.at("id"), t)));
  for (const auto& s : j.at("storage")) injection(s.at("bus"), x.at(key("dis", s.at("id"), t))+x.at(key("ch", s.at("id"), t)));
  for (const auto& d : j.at("dc_links")) {
    const double p = x.at(key("dc", d.at("id"), t));
    if (d.at("from_bus").get<int>() >= 0) injection(d.at("from_bus"), -p);
    if (d.at("to_bus").get<int>() >= 0) injection(d.at("to_bus"), p*(1-num(d, "loss_fraction")));
  }
  std::map<int,int> parent;
  for (const auto& n : j.at("buses")) parent[n.at("id")] = n.at("id");
  const auto root = [&](int n) { while (parent.at(n) != n) n = parent.at(n); return n; };
  std::vector<const J*> lines;
  for (const auto& l : j.at("branches")) if (at(l, "available", t) != 0) {
    ACBranch branch; branch.index = l.at("id"); branch.from_bus = l.at("from_bus"); branch.to_bus = l.at("to_bus");
    branch.r_pu = l.at("r_pu"); branch.x_pu = l.at("x_pu"); branch.b_pu = l.at("b_pu");
    branch.tap = l.at("tap"); branch.shift_deg = l.at("shift_deg"); branch.rate_a_mva = at(l, "rate_mva", t);
    sys.ac.branches.push_back(branch); lines.push_back(&l);
    parent[root(branch.to_bus)] = root(branch.from_bus);
  }
  std::set<int> slacks;
  for (const auto& g : j.at("generators")) if (x.at(key("u", g.at("id"), t)) > 0.5) {
    Generator gen; gen.index = g.at("id"); gen.bus = g.at("bus"); gen.pg_mw = x.at(key("p", g.at("id"), t));
    gen.pmin_mw = 0; gen.pmax_mw = at(g, "pmax_mw", t); gen.qmin_mvar = g.at("qmin_mvar");
    gen.qmax_mvar = g.at("qmax_mvar"); gen.vg_pu = g.at("voltage_pu");
    auto& bus = sys.ac.buses.at(positions.at(gen.bus)); bus.vm_pu = gen.vg_pu;
    if (slacks.insert(root(gen.bus)).second) bus.bus_type = BusType::SLACK;
    else if (bus.bus_type != BusType::SLACK) bus.bus_type = BusType::PV;
    sys.ac.generators.push_back(gen);
  }
  ACObservation out;
  for (const auto& n : j.at("buses")) if (!slacks.count(root(n.at("id")))) return out;
  PowerFlowOptions options; options.tol = 1e-9;
  const auto pf = solve_power_flow(sys, options);
  out.converged = pf.converged; out.residual = pf.residual;
  if (!pf.converged) return out;
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& n = sys.ac.buses[i]; const double vm = pf.vm.at(i);
    out.violation_functions[key("ac_voltage_upper", n.index, t)] = vm-n.vmax_pu;
    out.violation_functions[key("ac_voltage_lower", n.index, t)] = n.vmin_pu-vm;
  }
  std::map<int,double> branch_power;
  std::map<int,double> required_p, required_q, scheduled_p, pmin, pmax, qmin, qmax;
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& bus = sys.ac.buses[i]; const double v2 = pf.vm.at(i)*pf.vm.at(i);
    required_p[bus.index] = bus.pd_mw+bus.gs_mw*v2;
    required_q[bus.index] = bus.qd_mvar-bus.bs_mvar*v2;
  }
  for (const auto& g : sys.ac.generators) {
    scheduled_p[g.bus] += g.pg_mw; pmin[g.bus] += g.pmin_mw; pmax[g.bus] += g.pmax_mw;
    qmin[g.bus] += g.qmin_mvar; qmax[g.bus] += g.qmax_mvar;
  }
  for (size_t i = 0; i < pf.branch_flows.size(); ++i) {
    const auto& flow = pf.branch_flows[i]; const auto& line = *lines.at(i); const int id = line.at("id");
    branch_power[id] = flow.pf_mw;
    required_p[line.at("from_bus")] += flow.pf_mw; required_p[line.at("to_bus")] += flow.pt_mw;
    required_q[line.at("from_bus")] += flow.qf_mvar; required_q[line.at("to_bus")] += flow.qt_mvar;
    out.losses += flow.pf_mw+flow.pt_mw;
    out.violation_functions[key("ac_thermal_from", id, t)] = std::hypot(flow.pf_mw, flow.qf_mvar)-at(line, "rate_mva", t);
    out.violation_functions[key("ac_thermal_to", id, t)] = std::hypot(flow.pt_mw, flow.qt_mvar)-at(line, "rate_mva", t);
    out.violation_functions[key("ac_active_upper", id, t)] = flow.pf_mw-at(line, "max_mw", t);
    out.violation_functions[key("ac_active_lower", id, t)] = at(line, "min_mw", t)-flow.pf_mw;
  }
  // Reconstruct aggregate generator injections from terminal flows; do not let
  // slack balancing hide a generator capacity or reactive capability violation.
  for (const auto& [bus, scheduled] : scheduled_p) {
    out.active_adjustment_mw[bus] = required_p[bus]-scheduled;
    out.violation_functions[key("ac_generator_pmax", bus, t)] = required_p[bus]-pmax[bus];
    out.violation_functions[key("ac_generator_pmin", bus, t)] = pmin[bus]-required_p[bus];
    out.violation_functions[key("ac_generator_qmax", bus, t)] = required_q[bus]-qmax[bus];
    out.violation_functions[key("ac_generator_qmin", bus, t)] = qmin[bus]-required_q[bus];
  }
  for (const auto& s : j.at("sections")) {
    double flow = 0;
    for (const auto& member : s.at("members")) flow += branch_power[member.at("branch")]*num(member, "coefficient");
    out.violation_functions[key("ac_section_upper", s.at("id"), t)] = flow-at(s, "max_mw", t);
    out.violation_functions[key("ac_section_lower", s.at("id"), t)] = at(s, "min_mw", t)-flow;
  }
  return out;
}

J audit_security(const J& j, const std::map<std::string,double>& x,
                 std::vector<SecurityCut>& cuts, int iteration) {
  J out = {{"iteration", iteration}, {"secure", true}, {"periods", J::array()}, {"new_cuts", J::array()}};
  for (int t = 0; t < static_cast<int>(j.at("periods").size()); ++t) {
    const auto ac = observe_ac(j, x, t);
    J period = {{"period", t}, {"converged", ac.converged}, {"residual", ac.residual},
                {"losses_mw", ac.converged ? J(ac.losses) : J(nullptr)},
                {"active_adjustment_by_bus_mw", ac.active_adjustment_mw}, {"violations", J::array()}};
    if (!ac.converged) out["secure"] = false;
    std::vector<std::string> failures;
    for (const auto& [name, residual] : ac.violation_functions) if (residual > tolerance) {
      failures.push_back(name); period["violations"].push_back({{"name", name}, {"excess", residual}}); out["secure"] = false;
    }
    if (!failures.empty() && iteration + 1 < j.at("execution").at("security_iterations").get<int>()) {
      std::vector<std::string> controls;
      for (const auto& g : j.at("generators")) if (x.at(key("u", g.at("id"), t)) > 0.5) controls.push_back(key("p", g.at("id"), t));
      for (const auto& s : j.at("storage")) for (const char* c : {"ch", "dis"}) controls.push_back(key(c, s.at("id"), t));
      for (const auto& d : j.at("controllable_loads")) controls.push_back(key("load_reduction", d.at("id"), t));
      for (const auto& d : j.at("dc_links")) controls.push_back(key("dc", d.at("id"), t));
      std::map<std::string, SecurityCut> candidates;
      for (const auto& f : failures) { candidates[f].name = "2.6.2.4/"+std::to_string(iteration)+"/"+f; candidates[f].rhs = -ac.violation_functions.at(f); }
      // Sequential linear security feedback, not a global convex relaxation.
      // The 1e-3 MW forward perturbation is checked by a fresh nonlinear PF at
      // every iterate. No iterate is certified from these derivatives alone.
      constexpr double perturbation = 1e-3;
      for (const auto& control : controls) {
        auto perturbed = x; perturbed[control] += perturbation;
        const auto next = observe_ac(j, perturbed, t);
        if (!next.converged) continue;
        for (const auto& f : failures) {
          const double derivative = (next.violation_functions.at(f)-ac.violation_functions.at(f))/perturbation;
          if (std::abs(derivative) > 1e-10) { candidates[f].coefficients[control] = derivative; candidates[f].rhs += derivative*x.at(control); }
        }
      }
      for (auto& [f, cut] : candidates) if (!cut.coefficients.empty()) {
        out["new_cuts"].push_back({{"name", cut.name}, {"coefficients", cut.coefficients}, {"rhs", cut.rhs}});
        cuts.push_back(std::move(cut));
      }
    }
    out["periods"].push_back(period);
  }
  return out;
}

}  // namespace

J analyze_southern_market_result(const J& result, bool audit_ac) {
  J out = {{"ac_audit", {{"status", "not_requested"}}},
    {"settlement", {{"status", "unavailable"}, {"formal_settlement_eligible", false}}}};
  if (!result.value("schedule_feasible", false)) return out;
  const auto& b = result.at("effective_boundary"); const auto& s = result.at("sced");
  if (audit_ac) {
    std::map<std::string,double> x;
    const std::map<std::string,std::map<std::string,std::string>> fields = {
      {"generators", {{"u","online"},{"p","power_mw"}}},
      {"storage", {{"dis","discharge_mw"},{"ch","charge_mw"}}},
      {"controllable_loads", {{"load_reduction","reduction_mw"}}}, {"dc_links", {{"dc","power_mw"}}}};
    for (const auto& [table, mapping] : fields) for (const auto& row : s.at(table))
      for (const auto& [variable, field] : mapping) for (int t = 0; t < T; ++t) x[key(variable.c_str(),row.at("id"),t)] = at(row,field.c_str(),t);
    J input = b; input["execution"]["security_iterations"] = 1;
    std::vector<SecurityCut> cuts;
    auto audit = audit_security(input,x,cuts,0);
    audit["status"] = audit.at("secure").get<bool>() ? "passed" : "failed";
    audit["scope"] = "post-dispatch AC PF, 96 realized + 2 forecast points; no corrective optimization or exhaustive N-1 certificate";
    out["ac_audit"] = std::move(audit);
  }
  auto& ledger = out["settlement"];
  if (!result.value("prices_valid",false)) { ledger["reason"] = "No valid diagnostic LMP"; return out; }
  if (!b.at("dc_links").empty() || !b.at("external_schedules").empty()) {
    ledger["status"] = "unsupported"; ledger["reason"] = "Ledger currently requires a closed AC market without external schedules or DC links"; return out;
  }
  std::map<int,const J*> buses, prices;
  for (const auto& n : b.at("buses")) buses[n.at("id")] = &n;
  for (const auto& n : result.at("lmp").at("buses")) prices[n.at("id")] = &n;
  const auto price = [&](int id, int t) { return at(*prices.at(id),"lmp_per_mwh",t); };
  ledger["scope"] = "conditional SCED quantity x diagnostic LMP, realized 96 x 0.25h; no contracts, real-time deviation, auxiliary-service or cost allocation";
  ledger["accounts"] = J::array(); ledger["periods"] = J::array();
  std::vector<double> generation(96), storage(96), load(96), slack(96), rent(96), compensation(96);
  for (const auto& [table, field] : {std::pair{"generators","power_mw"},std::pair{"storage","discharge_mw"},std::pair{"controllable_loads","reduction_mw"}}) {
    std::map<int,const J*> inputs; for (const auto& row : b.at(table)) inputs[row.at("id")] = &row;
    for (const auto& row : s.at(table)) {
      const auto& input = *inputs.at(row.at("id")); const int bus = input.at("bus");
      double energy = 0, cash = 0; J payments = J::array();
      for (int t = 0; t < 96; ++t) {
        const double mw = at(row,field,t)+(std::string(table)=="storage" ? at(row,"charge_mw",t) : 0);
        const bool energy_eligible=!row.contains("energy_market_eligible")||row.at("energy_market_eligible")[t].get<bool>();
        const double payment = energy_eligible?.25*mw*(std::string(table)=="controllable_loads" ? at(input,"compensation_per_mwh",t) : price(bus,t)):0;
        energy += .25*mw; cash += payment; payments.push_back(payment);
        if (std::string(table)=="generators") generation[t] += payment;
        else if (std::string(table)=="storage") storage[t] += payment;
        else { compensation[t] += payment; load[t] -= .25*mw*price(bus,t); }
      }
      ledger["accounts"].push_back({{"table",table},{"id",row.at("id")},{"name",row.at("name")},{"bus",bus},
        {"energy_mwh",energy},{"receipt_cny",cash},{"receipt_cny_by_period",payments}});
    }
  }
  for (const auto& n : s.at("buses")) for (int t = 0; t < 96; ++t) {
    const int id = n.at("id"); load[t] += .25*at(*buses.at(id),"load_mw",t)*price(id,t);
    slack[t] += .25*(at(n,"deficit_mw",t)-at(n,"surplus_mw",t))*price(id,t);
  }
  std::map<int,const J*> branches; for (const auto& l : b.at("branches")) branches[l.at("id")] = &l;
  for (const auto& l : s.at("branches")) for (int t = 0; t < 96; ++t) {
    const auto& input = *branches.at(l.at("id"));
    rent[t] += .25*at(l,"power_mw",t)*(price(input.at("to_bus"),t)-price(input.at("from_bus"),t));
  }
  // Fault/Inflow Study in southern_execution_contract.md: sum nodal balances
  // weighted by LMP; congestion rent is computed from flows, never a plug.
  double max_residual = 0, total_residual = 0, gross = 0;
  for (int t = 0; t < 96; ++t) {
    const double residual = load[t]-generation[t]-storage[t]-slack[t]-rent[t];
    max_residual = std::max(max_residual,std::abs(residual)); total_residual += residual;
    gross += std::abs(load[t])+std::abs(generation[t])+std::abs(storage[t])+std::abs(slack[t])+std::abs(rent[t]);
    ledger["periods"].push_back({{"slot",t},{"load_payment_cny",load[t]},{"generation_receipt_cny",generation[t]},
      {"storage_receipt_cny",storage[t]},{"diagnostic_slack_receipt_cny",slack[t]},{"line_rent_cny",rent[t]},
      {"dr_compensation_cny",compensation[t]},{"residual_cny",residual}});
  }
  const double threshold = std::max(1e-3,1e-8*gross);
  ledger["status"] = max_residual <= threshold && std::abs(total_residual) <= threshold ? "conditional" : "balance_failed";
  ledger["max_residual_cny"] = max_residual; ledger["total_residual_cny"] = total_residual; ledger["tolerance_cny"] = threshold;
  return out;
}

J inspect_southern_market_model(const J& boundary) {
  const auto start = std::chrono::steady_clock::now();
  const J input = validate_southern_market(boundary);
  const auto lease = build_model(input,"scuc");const auto& model=*lease;
  return {{"status","model_inspected"},{"solved",false},{"stage","scuc"},
    {"formulation",model.compact ? "compact" : "reference"},{"model_size",model.size_report},{"assembly_template",model.assembly_template},
    {"runtime_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
}

J southern_market_ptdf(const J& boundary, int period, const std::vector<int>& branch_ids) {
  const auto started = std::chrono::steady_clock::now();
  const J j = validate_southern_market(boundary);
  if (period < 0 || period >= T || branch_ids.empty() || branch_ids.size() > 64)
    throw std::invalid_argument("PTDF requires period 0..97 and 1..64 unique branch IDs");
  std::map<int,int> position;
  std::vector<int> ids;
  for (const auto& bus : j.at("buses")) { position[bus.at("id")] = static_cast<int>(ids.size()); ids.push_back(bus.at("id")); }
  const int n = static_cast<int>(ids.size());
  std::vector<int> parent(n); std::iota(parent.begin(),parent.end(),0);
  const auto root = [&](int p) { while (parent[p] != p) p = parent[p]; return p; };
  std::map<int,const J*> lines;
  for (const auto& l : j.at("branches")) {
    lines[l.at("id")] = &l;
    if (at(l,"available",period)) {
      const int a = root(position.at(l.at("from_bus"))), b = root(position.at(l.at("to_bus")));
      parent[std::max(a,b)] = std::min(a,b);
    }
  }
  std::set<int> requested;
  for (int id : branch_ids) if (!lines.count(id) || !requested.insert(id).second)
    throw std::invalid_argument("PTDF unknown or duplicate branch ID");
  std::vector<bool> slack(n); J references = J::array(), bus_references = J::array();
  for (int i=0;i<n;++i) { slack[i] = root(i)==i; if (slack[i]) references.push_back(ids[i]); bus_references.push_back(ids[root(i)]); }
  // B = A^T D A. Pin one reference per island; selected H rows are obtained
  // from B_red^T h = D_l A_l^T, without forming an inverse or dense L*N matrix.
  // Affine phase-shift term: f0 = c - H A^T c. Derivation: execution contract.
  std::vector<Eigen::Triplet<double>> entries;
  Eigen::VectorXd shift_rhs = Eigen::VectorXd::Zero(n);
  for (const auto& l : j.at("branches")) if (at(l,"available",period)) {
    const int a=position.at(l.at("from_bus")), b=position.at(l.at("to_bus"));
    const double d=num(j,"base_mva")/(num(l,"x_pu")*num(l,"tap"));
    const double c=-d*num(l,"shift_deg")*std::acos(-1)/180;
    if (!slack[a]) { entries.emplace_back(a,a,d); shift_rhs[a]-=c; }
    if (!slack[b]) { entries.emplace_back(b,b,d); shift_rhs[b]+=c; }
    if (!slack[a] && !slack[b]) { entries.emplace_back(a,b,-d); entries.emplace_back(b,a,-d); }
  }
  for (int i=0;i<n;++i) if (slack[i]) entries.emplace_back(i,i,1);
  Eigen::SparseMatrix<double> matrix(n,n); matrix.setFromTriplets(entries.begin(),entries.end());
  Eigen::SparseLU<Eigen::SparseMatrix<double>> factor; factor.compute(matrix);
  if (factor.info()!=Eigen::Success) throw std::runtime_error("PTDF singular island susceptance matrix");
  const Eigen::VectorXd shift_angle=factor.solve(shift_rhs);
  if (factor.info()!=Eigen::Success || !shift_angle.allFinite()) throw std::runtime_error("PTDF phase-shift solve failed");
  double residual=(matrix*shift_angle-shift_rhs).lpNorm<Eigen::Infinity>();
  J out={{"period",period},{"bus_ids",ids},{"reference_bus_ids",references},{"bus_reference_ids",bus_references},
    {"rows",J::array()},{"equivalent_periods",J::array()},{"unique_topologies",0},
    {"model","lossless-dc-with-phase-shift"},{"coefficient_unit","MW/MW"},{"factorizations",1}};
  std::set<std::vector<int>> signatures;
  std::vector<int> selected_signature;
  for (const auto& l : j.at("branches")) selected_signature.push_back(at(l,"available",period)!=0);
  for (int t=0;t<T;++t) {
    std::vector<int> signature;
    for (const auto& l : j.at("branches")) signature.push_back(at(l,"available",t)!=0);
    if (signature==selected_signature) out["equivalent_periods"].push_back(t);
    signatures.insert(std::move(signature));
  }
  out["unique_topologies"]=signatures.size();
  for (int id : branch_ids) {
    const J& l=*lines.at(id); const bool available=at(l,"available",period)!=0;
    const int a=position.at(l.at("from_bus")), b=position.at(l.at("to_bus"));
    const double d=num(j,"base_mva")/(num(l,"x_pu")*num(l,"tap"));
    Eigen::VectorXd rhs=Eigen::VectorXd::Zero(n), h=Eigen::VectorXd::Zero(n);
    double offset=0;
    if (available) {
      if (!slack[a]) rhs[a]+=d;
      if (!slack[b]) rhs[b]-=d;
      h=factor.solve(rhs);
      if (factor.info()!=Eigen::Success || !h.allFinite()) throw std::runtime_error("PTDF sensitivity solve failed");
      residual=std::max(residual,(matrix*h-rhs).lpNorm<Eigen::Infinity>());
      offset=d*(shift_angle[a]-shift_angle[b]-num(l,"shift_deg")*std::acos(-1)/180);
    }
    out["rows"].push_back({{"branch_id",id},{"available",available},{"from_bus",l.at("from_bus")},{"to_bus",l.at("to_bus")},
      {"coefficients",std::vector<double>(h.data(),h.data()+h.size())},{"phase_shift_flow_mw",offset}});
  }
  if (residual>tolerance) throw std::runtime_error("PTDF linear-system residual exceeds 1e-6");
  out["max_linear_residual"]=residual;
  out["runtime_sec"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  return out;
}

J southern_market_solver_capabilities() {
  const bool gurobi = engine::GurobiAdapter{}.available();
  return J::array({{{"id","highs"},{"available",true},{"label","HiGHS"}},
    {{"id","gurobi"},{"available",gurobi},{"label","Gurobi"},
     {"reason",gurobi ? "Local environment initialized; model-specific license limits checked at solve" : "Gurobi library or license initialization unavailable"}},
    {{"id","native"},{"available",true},{"label","Native B&C / HiGHS LP"},
     {"root_cut_profiles",{"default","enhanced"}},
     {"reason","Native tree and cuts; HiGHS continuous kernel and pricing LP. Experimental for large markets; LP calls have no hard deadline."}}});
}

static J run_southern_market_impl(const J& boundary, const J& ancillary, const J& preceding = nullptr,
                                  bool pricing = true) {
  const auto begin = std::chrono::steady_clock::now();
  J effective = validate_southern_market(boundary);
  const double validation_sec = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  if (!ancillary.is_null()) yunnan::validate_full(effective,ancillary);
  const bool intraday=!preceding.is_null();
  J sealed=ancillary.is_null()?J(nullptr):yunnan::seal_bids(ancillary,effective);
  if(!ancillary.is_null()) {
    sealed["workflow"]=ancillary.value("workflow",yunnan::workflow_defaults());
    yunnan::require((sealed.at("workflow").at("stage")=="intraday")==intraday,"intraday clearing requires a preceding day-ahead result");
  }
  if(intraday) {
    yunnan::require(preceding.at("boundary_snapshot")==boundary,"preceding day-ahead boundary differs");
    yunnan::require(preceding.value("schedule_feasible",false)&&preceding.contains("ancillary")&&preceding.contains("commitment_solution"),"valid preceding commitment required");
    yunnan::require(preceding.at("ancillary").at("workflow").at("stage")=="day_ahead","intraday must reference day-ahead results");
    auto previous_config=preceding.at("ancillary").at("config"),new_config=sealed;
    previous_config.erase("workflow");new_config.erase("workflow");
    yunnan::require(previous_config==new_config,"sealed day-ahead declarations cannot change during intraday clearing");
    yunnan::require(preceding.at("ancillary").at("workflow").at("bid_submissions")==sealed.at("workflow").at("bid_submissions"),"sealed submissions cannot change intraday");
  }
  J out = {{"mode", "southern_day_ahead"}, {"schema_version", boundary.at("schema_version")},
    {"status", "scuc_failed"}, {"feasible", false}, {"schedule_feasible", false}, {"prices_valid", false},
    {"boundary_snapshot", boundary}, {"effective_boundary", effective}, {"security_iterations", J::array()},
    {"model_scope", {{"model", "southern-2025-v1.0-execution-1"}, {"regulator_certified", false},
      {"time_points", 98}, {"day_intervals", 96}, {"network", "lossless-ac-dc-flow-with-constant-hvdc-losses"},
      {"regulation", "externally-precleared-capacity-awards-applied-after-scuc"},
      {"interpretation", effective.at("execution")},
      {"limitations", {"A1-A7 use the explicitly selected execution interpretation; this is not an official corrigendum.",
        "Regulation market bids and clearing are governed by a separate rulebook; this pipeline consumes sourced preclearing awards.",
        "Representative-point state integration follows supplied durations; gaps carry no unobserved energy or water flows.",
        "Security feedback uses local AC sensitivities; exhausted or nonconverged iterations never certify security.",
        "Prices are conditional on the preceding discrete dispatch states; nonlinear AC losses are certified separately."}}}}};
  if (!effective.at("controllable_loads").empty())
    out["model_scope"]["limitations"].push_back("Compensated interruptible demand is a research extension; no rebound, load reserve, or independent demand-side bidding rule certification. LMP fixes its SCED dispatch.");
  if (!ancillary.is_null()) {
    out["model_scope"]["regulation"]="yunnan-2025-hourly-prearrangement-fixed-uc-primary-hydro-safe-bands";
    out["model_scope"]["limitations"][1]="Yunnan AGC research prearrangement with coupled energy SCED; formal intraday and AGC activation security are not certified.";
  }
  out["diagnostic"] = effective.at("execution").value("balance_policy", std::string("strict")) == "diagnostic";
  if (out["diagnostic"].get<bool>()) out["model_scope"]["limitations"].push_back("Diagnostic node deficit/surplus slacks are penalized research extensions. Prices depend on the diagnostic penalty; this is not a feasible physical supply schedule or normal-market price certification.");
  std::vector<SecurityCut> cuts;
  const bool ac_required = effective.at("execution").at("ac_security") == "required";
  const auto finish = [&]() {
    out["validation_sec"] = validation_sec;
    if(out.contains("ancillary")) {
      auto& a=out["ancillary"];
      a["coupled_schedule_feasible"]=out.at("schedule_feasible");
      a["energy_status"]=out.at("status");
      a["diagnostic"]=out.at("diagnostic");
      a["settlement_eligible"]=intraday&&out.value("schedule_feasible",false)&&!out.at("diagnostic").get<bool>()&&a.value("realtime_capacity_sufficient",true)&&(!ac_required||out.at("status")=="converged");
      if(a.at("capacity_sufficient").get<bool>()) a["status"]=out.value("schedule_feasible",false)?(intraday?"intraday_schedule_only":"prearranged_schedule_only"):"coupled_safety_failed";
      if(!a.value("realtime_capacity_sufficient",true))a["status"]="realtime_capacity_shortfall";
      if(out.at("status")=="ac_security_failed") a["status"]="ac_security_failed";
    }
    // Every caller immediately returns finish(); move the captured result,
    // which is not eligible for NRVO. See performance.md, output ownership.
    out["runtime_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(); return std::move(out);
  };
  const int iterations = ac_required ? effective.at("execution").at("security_iterations").get<int>() : 1;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    out["schedule_feasible"]=false;out["prices_valid"]=false;
    out.erase("sced");out.erase("lmp");
    // Stage lifetime is bounded: keep solutions, not three sparse MILP builders.
    // Cost model and unchanged mathematical contract: southern_execution_contract.md.
    std::map<std::string,double> dispatch;
    struct PriorBound {
      engine::LPModel lp;
      std::vector<int> integers;
      engine::SolveResult result;
      double constant;
      J certificate;
    };
    std::unique_ptr<PriorBound> prior;
    BuildLease derived;
    BuildLease sced_source;
    double derived_assembly_sec=0;
    for (const std::string stage : {"scuc", "sced"}) {
      if(stage=="scuc"&&intraday) {
        dispatch=preceding.at("commitment_solution").get<std::map<std::string,double>>();
        for(const auto& s:preceding.at("sced").at("storage"))for(int t=0;t<T;++t) {
          dispatch[key("da_dis",s.at("id"),t)]=s.at("discharge_mw")[t];dispatch[key("da_ch",s.at("id"),t)]=s.at("charge_mw")[t];
        }
        out["scuc"]=preceding.at("scuc");out["scuc"]["reused_day_ahead"]=true;
        out["commitment_solution"]=preceding.at("commitment_solution");
        out["ancillary"]=yunnan::prearrange(effective,sealed,dispatch);
        yunnan::adjust_awards(out["ancillary"],sealed.at("workflow"));
        if(!out["ancillary"].at("capacity_sufficient").get<bool>()) {out["status"]="ancillary_capacity_shortfall";return finish();}
        effective["_yunnan"]=out.at("ancillary");continue;
      }
      const auto start = std::chrono::steady_clock::now();
      BuildLease lease;
      const bool used_derived=bool(derived);
      try { lease = used_derived?std::move(derived):build_model(effective, stage, std::move(dispatch), cuts, ancillary.is_null()); }
      catch (const std::invalid_argument& e) {
        out["status"] = stage == "sced" ? "regulation_boundary_failed" : "scuc_failed";
        out["error"] = e.what(); return finish();
      }
      auto& model=*lease;
      const double assembly = used_derived?derived_assembly_sec:std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      const auto solve_start = std::chrono::steady_clock::now();
      engine::SolveResult solved;
      bool reused_solution=false;
      if(prior && stage=="sced" && ancillary.is_null() && prior->constant==model.constant_cost &&
          detail::same_lp_with_tighter_box(prior->lp,model.model.linear_part) &&
          std::all_of(model.model.binary_idx.begin(),model.model.binary_idx.end(),[&](int c){
            return std::binary_search(prior->integers.begin(),prior->integers.end(),c);}) &&
          primal_residual(model,prior->result.x,effective)<=tolerance) {
        solved=std::move(prior->result);
        reused_solution=true;
        solved.stats.runtime_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-solve_start).count();
        model.gap_certificate=prior->certificate;
        model.gap_certificate["reused_from"]="scuc";
        model.gap_certificate["matrix_cost_rhs_comparison"]="exact_match";
        model.gap_certificate["bound_subset"]=true;
        model.solver_timing={{"scope","preceding-bound-and-feasible-candidate-reuse"},
          {"verification_sec",solved.stats.runtime_sec}};
      } else solved=solve(model,effective);
      prior.reset();
      if(stage=="scuc" && ancillary.is_null() && model.gap_certificate.is_object() &&
          model.gap_certificate.value("accepted",false)) {
        prior=std::make_unique<PriorBound>(PriorBound{model.model.linear_part,model.model.binary_idx,
          solved,model.constant_cost,model.gap_certificate});
        std::sort(prior->integers.begin(),prior->integers.end());
      }
      const auto audit_start = std::chrono::steady_clock::now();
      out[stage] = stage_result(model, solved, effective);
      out[stage]["solve_wall_sec"] = std::chrono::duration<double>(audit_start-solve_start).count();
      out[stage]["audit_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-audit_start).count();
      out[stage]["assembly_sec"] = assembly;
      if (!out[stage].at("feasible").get<bool>()) { out["status"] = stage+"_failed"; return finish(); }
      const auto export_start = std::chrono::steady_clock::now();
      // A derived SCED stores precisely the prior solution map. The reuse gate
      // above preserves x; transfer that map without rebuilding it (performance.md).
      dispatch = used_derived && reused_solution ? std::move(model.fixed) : solution_map(model, solved);
      out[stage]["solution_export_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-export_start).count();
      if(stage=="scuc" && prior && ancillary.is_null()) {
        const auto derive_started=std::chrono::steady_clock::now();
        if(derive_sced(model,effective,solved,dispatch,cuts)) {
          derived_assembly_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-derive_started).count();
          derived=std::move(lease);
        }
      }
      if(stage=="scuc" && !ancillary.is_null()) {
        out["commitment_solution"]=dispatch;
        out["ancillary"]=yunnan::prearrange(effective,sealed,dispatch);
        yunnan::adjust_awards(out["ancillary"],sealed.at("workflow"));
        if(!out["ancillary"].at("capacity_sufficient").get<bool>()) {out["status"]="ancillary_capacity_shortfall";return finish();}
        effective["_yunnan"]=out.at("ancillary");
      }
      if(stage=="sced"&&pricing&&ancillary.is_null()&&!ac_required)sced_source=std::move(lease);
    }
    out["schedule_feasible"] = true;
    if (ac_required) {
      auto security = audit_security(effective, dispatch, cuts, iteration);
      out["security_iterations"].push_back(security);
      if (!security.at("secure").get<bool>()) {
        out["status"] = "ac_security_failed";
        if (security.at("new_cuts").empty()) return finish();
        continue;
      }
    }
    // Recovery metrics depend only on SCED; performance.md, recovery rationale.
    if (!pricing) {
      out["status"] = "dispatch_only";
      out["model_scope"]["pricing"] = "not_requested";
      out["model_scope"]["limitations"].push_back("Dispatch recovery only: LMP and settlement were not calculated.");
      return finish();
    }
    const auto assembly_start = std::chrono::steady_clock::now();
    BuildLease lmp_lease;
    if(sced_source&&derive_lmp(*sced_source,effective,dispatch,cuts))lmp_lease=std::move(sced_source);
    else {
      sced_source.reset();
      lmp_lease=build_model(effective,"lmp",std::move(dispatch),cuts,ancillary.is_null());
    }
    auto& lmp=*lmp_lease;
    const double assembly = std::chrono::duration<double>(std::chrono::steady_clock::now()-assembly_start).count();
    const auto solve_start = std::chrono::steady_clock::now();
    const auto priced = solve(lmp, effective);
    const auto audit_start = std::chrono::steady_clock::now();
    out["lmp"] = stage_result(lmp, priced, effective);
    out["lmp"]["solve_wall_sec"] = std::chrono::duration<double>(audit_start-solve_start).count();
    out["lmp"]["solution_export_sec"] = 0.0;
    out["lmp"]["assembly_sec"] = assembly;
    out["lmp"]["audit_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-audit_start).count();
    if (!out["lmp"].at("feasible").get<bool>() || !out["lmp"].value("prices_valid", false)) { out["status"] = "lmp_failed"; return finish(); }
    out["prices_valid"] = true; out["feasible"] = ac_required;
    out["status"] = ac_required ? "converged" : "schedule_only";
    return finish();
  }
  return finish();
}

J run_southern_day_ahead_market(const J& boundary) { return run_southern_market_impl(boundary,nullptr); }
J run_southern_dispatch_recovery(const J& boundary) { return run_southern_market_impl(boundary,nullptr,nullptr,false); }
J southern_realtime_defaults(const J& boundary) { return realtime::defaults(boundary); }
J southern_realtime_catalog() { return realtime::catalog(); }

// Chapter 3.5: independent 5min dispatch and 15min price programs. The
// endpoint projection is an explicit execution interpretation, not averaging
// dispatch duals. See southern_real_time.md for clock and water-lag semantics.
static J realtime_window(const J& input, bool price) {
  const auto begin=std::chrono::steady_clock::now();
  J out={{"boundary_snapshot",input},{"status","scuc_failed"},{"schedule_feasible",false},{"physical_schedule_feasible",false},
    {"prices_valid",false},{"security_iterations",J::array()},{"model_scope",{
      {"time_points",24},{"interval_minutes",5},{"pricing_points",8},{"pricing_interval_minutes",15},
      {"network","lossless AC angle-flow with authored HVDC losses"},
      {"limitations",{"Schedule simulation, not an official market or settlement certificate.",
        "AGC and deep-peak awards are external confirmed boundaries; their markets are not cleared here.",
        "LMP samples each quarter-hour endpoint and solves a separate LP; intra-quarter-hour events may make that projection infeasible.",
        "Startup/shutdown trajectories need observed phase history and are rejected. No emergency price repair or fair-curtailment certification.",
        "Two-hour hydro/load energy targets and storage terminal targets are authored window targets, not full-day conservation guarantees."}}}}};
  auto finish=[&](){out["runtime_sec"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();return out;};
  std::vector<SecurityCut> cuts;
  const bool security=input.at("execution").at("ac_security")=="required";
  std::map<std::string,double> dispatch;
  const int iterations=security?input.at("execution").at("security_iterations").get<int>():1;
  for(int iteration=0;iteration<iterations;++iteration) {
    dispatch.clear();out["schedule_feasible"]=false;
    for(const std::string stage:{"scuc","sced"}) {
      try {auto lease=build_model(input,stage,std::move(dispatch),cuts);auto& model=*lease;const auto solved=solve(model,input);
        out[stage]=stage_result(model,solved,input);
        if(!out[stage].at("feasible").get<bool>()){out["status"]=stage+"_failed";return finish();}
        dispatch=solution_map(model,solved);
      } catch(const std::exception& e){out["status"]=stage+"_failed";out["error"]=e.what();return finish();}
    }
    out["schedule_feasible"]=true;
    if(security){auto audit=audit_security(input,dispatch,cuts,iteration);out["security_iterations"].push_back(audit);
      if(!audit.at("secure").get<bool>()){out["status"]="ac_security_failed";if(audit.at("new_cuts").empty()||iteration+1==iterations)return finish();continue;}}
    break;
  }
  double violation=0;out["events"]=J::array();
  const auto event=[&](const char* family,const char* field,const char* reason){for(const auto& row:out.at("sced").at(family))for(int t=0;t<24;++t){const double v=at(row,field,t);violation=std::max(violation,v);if(v>tolerance)out["events"].push_back({{"family",family},{"id",row.at("id")},{"point",t},{"mw",v},{"metric",field},{"evidence",reason}});}};
  event("buses","deficit_mw","Supply-demand deficit under topology, unit, ramp and energy boundaries; binding constraints are evidence, not isolated causality.");
  event("buses","surplus_mw","Excess injection under minimum-output, fixed-plan and absorption boundaries; counterfactual required for unique attribution.");
  event("branches","overload_mw","Computed signed flow exceeds the available line rating.");
  event("sections","slack_plus_mw","Section upper transfer bound relaxed.");event("sections","slack_minus_mw","Section lower transfer bound relaxed.");
  event("accident_reserve","shortage_mw","Provincial ten-minute reserve exceeds eligible headroom and response capacity.");
  event("trades","priority_shortfall_mw","Per-period priority transfer lower bound relaxed; D-2 contracted energy metadata retained.");
  out["physical_schedule_feasible"]=violation<=tolerance;
  out["status"]=violation>tolerance?"diagnostic_schedule":security?"converged":"schedule_only";
  if(!price)return finish();
  try {
    std::vector<int> points;for(int t=0;t<8;++t)points.push_back(3*t+2);
    auto pricing=realtime::resample(input,24,points,15);
    for(auto& table:pricing["_rt"])if(table.is_array())for(auto& row:table)if(row.is_object())for(auto f=row.begin();f!=row.end();++f)if(realtime::temporal(f.key(),f.value(),24)){J values=J::array();for(int t:points)values.push_back(f.value()[t]);f.value()=values;}
    for(auto& h:pricing["reservoirs"]){
      realtime::require(h.at("lag_slots").get<int>()%3==0,"pricing requires water lag aligned to 15min; cannot round a 5min lag");
      h["lag_slots"]=h.at("lag_slots").get<int>()/3;
      const auto history=h.at("release_history_m3_s");J sampled=J::array();for(int i=static_cast<int>(history.size())%3+2;i<static_cast<int>(history.size());i+=3)sampled.push_back(history[i]);h["release_history_m3_s"]=sampled;
      for(auto& v:h["release_ramp_m3_s"])v=v.get<double>()*3;
    }
    for(auto& h:pricing["_rt"]["hydro_plants"])for(const char* f:{"min_mwh","max_mwh"})for(auto& v:h[f])v=v.get<double>()*3;
    for(auto& d:pricing["dc_links"])for(const char* f:{"ramp_up_mw","ramp_down_mw"})for(auto& v:d[f])v=v.get<double>()*3;
    std::map<std::string,double> preceding;
    for(const auto& [name,v]:dispatch){const auto split=name.rfind('/');if(split==std::string::npos)continue;const int t=std::stoi(name.substr(split+1));if(t%3==2)preceding[name.substr(0,split+1)+std::to_string(t/3)]=v;}
    for(const auto& g:pricing.at("generators")) {
      const int id=g.at("id");double old=num(g,"initial_on"),off=old?0:num(g,"initial_state_minutes");
      for(int t=0;t<8;++t){const double on=std::round(preceding.at(key("u",id,t)));const double start=std::max(0.0,on-old),stop=std::max(0.0,old-on);if(t)off+=15*(1-old);
        preceding[key("start",id,t)]=start;preceding[key("stop",id,t)]=stop;preceding[key("stable",id,t)]=on;
        const int category=off<num(g,"warm_after_minutes")?0:off<num(g,"cold_after_minutes")?1:2;
        for(int k=0;k<3;++k)preceding[key(("start"+std::to_string(k)).c_str(),id,t)]=k==category?start:0;
        off=on?0:off;preceding[key("offline_minutes",id,t)]=off;old=on;
      }
    }
    // AC sensitivity cuts refer to 5min indices and cannot be copied to the
    // independent pricing grid. The result certifies only the dispatch audit.
    auto lease=build_model(pricing,"lmp",std::move(preceding),{});auto& model=*lease;const auto solved=solve(model,pricing);out["lmp"]=stage_result(model,solved,pricing);
    out["pricing_boundary_snapshot"]=pricing;out["prices_valid"]=out["lmp"].value("prices_valid",false);
    if(!out["prices_valid"].get<bool>())out["pricing_error"]="Independent quarter-hour LP failed; five-minute duals are not substituted.";
  }catch(const std::exception& e){out["pricing_error"]=e.what();}
  return finish();
}
J make_southern_realtime(const J& boundary,const J& config) {
  auto source=validate_southern_market(boundary);auto prepared=realtime::prepare(source,config);
  return {{"schema_version","southern-realtime-job-v1"},{"config",config},{"prepared",prepared},{"forecast_resolution",prepared.at("forecast_resolution")},{"completed_steps",0},{"status","ready"},{"runs",J::array()},{"hourly_prices",J::array()}};
}
J step_southern_realtime(const J& job) {
  realtime::require(job.at("schema_version")=="southern-realtime-job-v1","invalid job");
  realtime::require(job.at("status")=="ready"||job.at("status")=="running","job is not runnable");
  auto next=job;const int step=job.at("completed_steps");const auto& prepared=job.at("prepared");
  auto boundary=realtime::window(prepared,3*step);
  if(step){const auto& prior=job.at("runs").back().at("dispatch");realtime::carry(boundary,prior.at("boundary_snapshot"),prior.at("sced"),3);}
  auto dispatch=realtime_window(boundary,true);
  J run={{"start_minute",prepared.at("start_minute").get<int>()+15*step},{"dispatch",dispatch},{"outlook",nullptr},{"executed_points",0}};
  const bool accepted=dispatch.value("schedule_feasible",false)&&dispatch.at("status")!="ac_security_failed";
  if(accepted){
    auto outlook=realtime::window(prepared,3*step+24);realtime::carry(outlook,boundary,dispatch.at("sced"),24);
    run["outlook"]=realtime_window(outlook,false);run["outlook"]["reference_only"]=true;run["executed_points"]=3;
    next["completed_steps"]=step+1;next["status"]=step+1==prepared.at("steps").get<int>()?"complete":"running";
  } else next["status"]="failed";
  next["runs"].push_back(run);next["hourly_prices"]=J::array();
  for(const auto& bus:prepared.at("boundary").at("buses"))for(int hour=prepared.at("start_minute").get<int>()/60;hour<=(prepared.at("start_minute").get<int>()+15*next.at("completed_steps").get<int>())/60;++hour){
    double sum=0;int count=0;for(const auto& r:next.at("runs"))if(r.at("start_minute").get<int>()/60==hour&&r.at("executed_points")==3&&r.at("dispatch").value("prices_valid",false))for(const auto& n:r.at("dispatch").at("lmp").at("buses"))if(n.at("id")==bus.at("id")){sum+=at(n,"lmp_per_mwh",0);++count;}
    next["hourly_prices"].push_back({{"bus_id",bus.at("id")},{"hour",hour},{"observed_quarters",count},{"price_per_mwh",count==4?J(sum/4):J(nullptr)}});
  }
  return next;
}
J yunnan_ancillary_defaults(const J& boundary) { return yunnan::complete_defaults(validate_southern_market(boundary)); }
void validate_yunnan_ancillary(const J& boundary,const J& config) { yunnan::validate_full(validate_southern_market(boundary),config); }
J run_yunnan_ancillary_market(const J& boundary,const J& config) { return run_southern_market_impl(boundary,config); }
J run_yunnan_ancillary_intraday(const J& boundary,const J& config,const J& preceding) { return run_southern_market_impl(boundary,config,preceding); }
J settle_yunnan_ancillary(const J& result,const J& request) { return yunnan::settle(result,request); }
J post_yunnan_ancillary_statement(const J& journal,const J& result,const J& entry) { return yunnan::post_statement(journal,result,entry); }
J settle_yunnan_ancillary_month(const J& journal,const J& request) { return yunnan::settle_month(journal,request); }

J compare_southern_market_results(const J& baseline, const J& scenario) {
  if (baseline.at("schema_version") != scenario.at("schema_version")) throw std::invalid_argument("incompatible Southern result versions");
  const auto& before = baseline.at("boundary_snapshot"); const auto& after = scenario.at("boundary_snapshot");
  for (const auto* table : {"areas", "buses", "generators", "branches", "storage", "reservoirs", "trades", "dc_links"}) {
    std::set<int> a, b;
    for (const auto& row : before.at(table)) a.insert(row.at("id").get<int>());
    for (const auto& row : after.at(table)) b.insert(row.at("id").get<int>());
    if (a != b) throw std::invalid_argument(std::string("comparison entity mismatch: ")+table);
  }
  J result = {{"boundary_changes", J::diff(before, after)}, {"baseline_status", baseline.at("status")}, {"scenario_status", scenario.at("status")},
    {"comparable", baseline.value("schedule_feasible", false) && scenario.value("schedule_feasible", false)}};
  if (!result.at("comparable").get<bool>()) return result;
  for (const char* metric : {"objective", "day_energy_bid_cost", "day_generation_mwh", "day_load_reduction_mwh"})
    result[std::string("delta_")+metric] = scenario.at("sced").value(metric, 0.0)-baseline.at("sced").value(metric, 0.0);
  result["generator_changes"] = J::array();
  for (const auto& g : scenario.at("sced").at("generators")) {
    for (const auto& old : baseline.at("sced").at("generators")) if (g.at("id") == old.at("id")) {
      double energy = 0; J difference = J::array();
      for (int t = 0; t < T; ++t) { double delta = at(g, "power_mw", t)-at(old, "power_mw", t); difference.push_back(delta); if (t < 96) energy += 0.25*delta; }
      result["generator_changes"].push_back({{"id", g.at("id")}, {"delta_power_mw", difference}, {"delta_day_mwh", energy}});
    }
  }
  result["price_changes"] = J::array();
  if (baseline.value("prices_valid", false) && scenario.value("prices_valid", false))
    for (const auto& node : scenario.at("lmp").at("buses")) for (const auto& old : baseline.at("lmp").at("buses")) if (node.at("id") == old.at("id")) {
      J delta = J::array(); for (int t = 0; t < T; ++t) delta.push_back(at(node, "lmp_per_mwh", t)-at(old, "lmp_per_mwh", t));
      result["price_changes"].push_back({{"id", node.at("id")}, {"delta_lmp_per_mwh", delta}});
    }
  return result;
}
}  // namespace hacdcpf::market
