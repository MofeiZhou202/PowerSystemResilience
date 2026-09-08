#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace hacdcpf::market::yunnan {
using Json = nlohmann::json;
inline void require(bool ok, const std::string& message) {
  if (!ok) throw std::invalid_argument("Yunnan ancillary: " + message);
}
inline void fields(const Json& row, std::initializer_list<const char*> keys) {
  require(row.is_object(), "expected object");
  std::set<std::string> allowed;
  for (auto k : keys) { allowed.insert(k); require(row.contains(k), std::string("missing ")+k); }
  for (auto it=row.begin(); it!=row.end(); ++it) require(allowed.count(it.key()), "unknown field "+it.key());
}
inline double number(const Json& v, double lo, double hi, const char* label) {
  require(v.is_number(), std::string(label)+" must be numeric");
  double x=v.get<double>();
  require(std::isfinite(x) && x>=lo && x<=hi, std::string(label)+" outside bounds"); return x;
}
inline void series(const Json& v, int count, double lo, double hi, double quantum, const char* label) {
  require(v.is_array() && v.size()==static_cast<size_t>(count), std::string(label)+" wrong time dimension");
  for (const auto& item:v) {
    double x=number(item,lo,hi,label);
    require(quantum==0 || std::abs(x/quantum-std::round(x/quantum))<1e-7, std::string(label)+" invalid increment");
  }
}
inline void validate(const Json& boundary, const Json& c) {
  Json base=c; base.erase("workflow"); base.erase("independent_units");
  fields(base,{"schema_version","source","research","cmin_mw","load_ratio","renewable_ratio","agc_units"});
  require(c.at("schema_version")=="yunnan-agc-2025-v1", "unsupported schema_version");
  require(c.at("source").is_string() && !c.at("source").get<std::string>().empty(),"source required");
  require(c.at("research").is_boolean(),"research must be boolean");
  const double cmin=number(c.at("cmin_mw"),0,1e6,"Cmin");
  if (!c.at("research").get<bool>()) require(cmin==450,"official Cmin is 450 MW; scaled studies require research=true");
  number(c.at("load_ratio"),0,1,"R1"); number(c.at("renewable_ratio"),0,1,"R2");
  std::map<int,const Json*> generators;
  for (const auto& g:boundary.at("generators")) {
    generators.emplace(g.at("id").get<int>(),&g);
    for (const auto* field:{"regulation_up_mw","regulation_down_mw"})
      for (const auto& v:g.at(field)) require(v.get<double>()==0,"external regulation awards must be zero for coupled clearing");
  }
  require(c.at("agc_units").is_array() && c.at("agc_units").size()<=generators.size(),"invalid AGC list");
  std::set<int> unit_ids, members;
  for (const auto& a:c.at("agc_units")) {
    fields(a,{"id","name","mode","qualified","source","k_history","price_per_mw","default_price_per_mw","capacity_mw","members"});
    require(a.at("id").is_number_integer() && number(a.at("id"),0,1000000,"AGC id")>=0 && unit_ids.insert(a.at("id").get<int>()).second,"duplicate AGC id");
    require(a.at("name").is_string() && a.at("source").is_string() && !a.at("source").get<std::string>().empty(),"AGC name/source required");
    require(a.at("mode")=="plant" || a.at("mode")=="single","generator AGC mode must be plant or single");
    require(a.at("qualified").is_boolean(),"qualified must be boolean");
    series(a.at("k_history"),8,-1e6,1e6,0,"k_history");
    double k=0; for(const auto& v:a.at("k_history")) k+=v.get<double>()/8;
    require(k>0,"average of last 8 awarded performance indices must be positive");
    series(a.at("price_per_mw"),24,-1e6,1e6,0.1,"price_per_mw");
    series(a.at("capacity_mw"),24,0,1e6,1,"capacity_mw");
    const auto& d=a.at("default_price_per_mw");
    if(!d.is_null()) { double v=number(d,3,8,"default price"); require(std::abs(v*10-std::round(v*10))<1e-7,"invalid default price increment"); }
    require(a.at("members").is_array() && !a.at("members").empty(),"AGC members required");
    if(a.at("mode")=="single") require(a.at("members").size()==1,"single AGC must have one member");
    for(const auto& m:a.at("members")) {
      fields(m,{"generator_id","standard_ramp_mw_min","safe_intervals_mw"});
      require(m.at("generator_id").is_number_integer(),"generator_id must be integer");
      int id=m.at("generator_id");
      require(generators.count(id) && members.insert(id).second,"unknown or repeated generator membership");
      const auto& g=*generators.at(id);
      require(g.at("kind")=="hydro" || g.at("kind")=="thermal","AGC bids currently support conventional hydro and thermal only");
      if(a.at("mode")=="single") require(g.at("kind")=="thermal","article 11: hydro requires plant AGC");
      number(m.at("standard_ramp_mw_min"),0,1e6,"standard ramp");
      const auto& ranges=m.at("safe_intervals_mw");
      require(ranges.is_array() && ranges.size()<=16,"safe intervals must be an array of at most 16 bands");
      require(g.at("kind")=="hydro" ? !ranges.empty() : ranges.empty(),"hydro requires safe bands; thermal uses energy output bounds");
      double last=-1;
      for(const auto& band:ranges) {
        require(band.is_array() && band.size()==2,"safe band must be [lower MW, upper MW]");
        double lo=number(band[0],0,1e6,"band lower"),hi=number(band[1],0,1e6,"band upper");
        require(lo<hi && lo>last,"safe bands must be increasing, disjoint and nonempty"); last=hi;
      }
    }
  }
  for(const auto& g:boundary.at("generators")) if(g.at("kind")=="hydro")
    require(members.count(g.at("id")),"all hydro machines need authored safe intervals, including nonwinning units");
}

inline Json defaults(const Json& b) {
  Json c={{"schema_version","yunnan-agc-2025-v1"},{"source","Synthetic AGC grouping, performance, offers and vibration bands; not plant measurements"},
    {"research",true},{"cmin_mw",450},{"load_ratio",0.005},{"renewable_ratio",0.003},{"agc_units",Json::array()}};
  std::map<int,int> reservoirs;
  for(const auto& r:b.at("reservoirs")) for(const auto& id:r.at("generators")) reservoirs[id.get<int>()]=r.at("id");
  std::map<std::string,size_t> groups;
  for(const auto& g:b.at("generators")) {
    const bool hydro=g.at("kind")=="hydro";
    if(!hydro && g.at("kind")!="thermal") continue;
    const int id=g.at("id");
    const std::string group=hydro && reservoirs.count(id) ? "reservoir/"+std::to_string(reservoirs.at(id)) : "generator/"+std::to_string(id);
    if(!groups.count(group)) {
      groups[group]=c["agc_units"].size();
      c["agc_units"].push_back({{"id",static_cast<int>(groups[group])},{"name",group},{"mode",hydro?"plant":"single"},
        {"qualified",true},{"source","Synthetic research qualification and AGC grouping; verify against actual AGC controls"},
        {"k_history",std::vector<double>(8,hydro?1.5:1)},{"price_per_mw",std::vector<double>(24,hydro?4:5)},
        {"default_price_per_mw",nullptr},{"capacity_mw",std::vector<double>(24,10)},{"members",Json::array()}});
    }
    double cap=0; for(const auto& p:g.at("pmax_mw")) cap=std::max(cap,p.get<double>());
    const double lo=g.at("technical_min_mw");
    Json bands=Json::array();
    if(hydro) {
      const double width=cap-lo;
      require(width>0,"hydro requires a positive authored operating range");
      bands={{lo,lo+0.4*width},{lo+0.5*width,cap}};
    }
    c["agc_units"][groups.at(group)]["members"].push_back({{"generator_id",id},
      {"standard_ramp_mw_min",cap*(hydro?0.2:0.015)},{"safe_intervals_mw",bands}});
  }
  return c;
}

// 2025 arts. 15, 31, 34-37; engineering capacity upper bound derived in
// docs/modules/market/yunnan_ancillary_markets.md. Full SCED validates sufficiency.
inline Json prearrange(const Json& b, const Json& c, const std::map<std::string,double>& fixed) {
  std::map<int,const Json*> generators;
  for(const auto& g:b.at("generators")) generators[g.at("id").get<int>()]=&g;
  auto x=[&](const char* v,int id,int t){return fixed.at(std::string(v)+"/"+std::to_string(id)+"/"+std::to_string(t));};
  std::vector<double> performance;
  Json units=c.at("agc_units");
  for(const auto& a:c.value("independent_units",Json::array())) units.push_back(a);
  double kmax=0;
  for(const auto& a:units) {
    double k=0; for(const auto& v:a.at("k_history")) k+=v.get<double>()/8;
    performance.push_back(k);kmax=std::max(kmax,k);
  }
  Json result={{"config",c},{"status","capacity_prearranged"},{"capacity_sufficient",true},
    {"hours",Json::array()},{"regulator_certified",false},{"settlement_cny",nullptr},
    {"model_limitations",{
      "Rule-based research clearing; formal regulatory certification and telemetry authenticity remain external.",
      "AGC qualification, last-eight performance and safe operating bands are authored inputs, not independently certified.",
      "Safety reviews are sourced inputs; solver infeasibility alone does not identify a unique unsafe AGC unit.",
      "Independent AGC resources use authored capability and sustained-response evidence; physical activation is not simulated.",
      "Network and cascade checks cover scheduled energy; AGC activation trajectories, head-dependent vibration and dynamic stability are not certified.",
      "Settlement requires separately supplied AGC command/response measurements; schedules do not imply mileage.",
      "Next-day representative points retain physical constraints but have no invented next-day hourly AGC awards."}}};
  for(int h=0;h<24;++h) {
    double load=0,renewable=0;
    for(int t=4*h;t<4*h+4;++t) {
      double l=0,d=0;
      for(const auto& a:b.at("areas")) l+=a.at("load_mw")[t].get<double>();
      for(const auto& g:b.at("generators")) if(g.at("kind")=="wind" || g.at("kind")=="solar" || g.at("kind")=="renewable") d+=g.at("forecast_mw")[t].get<double>();
      load=std::max(load,l);renewable=std::max(renewable,d);
    }
    double demand=c.at("cmin_mw").get<double>()+c.at("load_ratio").get<double>()*load+c.at("renewable_ratio").get<double>()*renewable;
    Json hour={{"hour",h},{"load_max_mw",load},{"renewable_max_mw",renewable},{"demand_mw",demand},{"bids",Json::array()}};
    size_t aidx=0;
    for(const auto& a:units) {
      double capability=1e20;
      bool eligible=a.at("qualified").get<bool>();
      for(int t=4*h;t<4*h+4;++t) {
        double sum=0;
        if(a.contains("members")) for(const auto& m:a.at("members")) {
          int id=m.at("generator_id");const auto& g=*generators.at(id);
          if(x("stable",id,t)<0.5) continue;
          double pmin=std::max(g.at("technical_min_mw").get<double>(),g.at("pmin_mw")[t].get<double>()),pmax=g.at("pmax_mw")[t];
          double width=pmax-pmin-x("primary",id,t);
          if(!m.at("safe_intervals_mw").empty()) {
            width=0;
            for(const auto& band:m.at("safe_intervals_mw")) width=std::max(width,std::min(pmax,band[1].get<double>())-std::max(pmin,band[0].get<double>())-x("primary",id,t));
          }
          const double ramp=std::min({g.at("ramp_up_mw_min").get<double>(),g.at("ramp_down_mw_min").get<double>(),m.at("standard_ramp_mw_min").get<double>()});
          sum+=std::max(0.0,std::min(width/2,5*ramp));
        }
        else {
          const bool storage=a.at("mode")=="storage";const int id=a.at("resource_id");
          const auto& family=b.at(storage?"storage":"controllable_loads");
          const Json* resource=nullptr;for(const auto& r:family)if(r.at("id")==id)resource=&r;
          require(resource!=nullptr,"missing independent AGC resource");const auto& r=*resource;
          eligible=eligible && r.at("available")[t].get<double>()>0 && a.at("sustained_hours").get<double>()>=1;
          eligible=eligible && a.at("cross_province_reserve_mw")[h].get<double>()==0;
          if(storage) {
            const double eta=std::sqrt(r.at("roundtrip_efficiency").get<double>());
            sum=std::min({r.at("charge_max_mw").get<double>(),r.at("discharge_max_mw").get<double>(),
              (r.at("max_mwh")[t].get<double>()-r.at("min_mwh")[t].get<double>())/(eta+1/eta)});
            if(c.contains("workflow")&&c.at("workflow").at("stage")=="intraday") {
              // Art.39 refers to the accepted day-ahead energy plan, supplied
              // by the caller separately from the frozen UC states.
              eligible=eligible && std::abs(x("da_dis",id,t))+std::abs(x("da_ch",id,t))<=1e-6;
            }
          } else {
            const double baseline=a.at("baseline_reduction_mw")[h];
            sum=std::min(baseline,r.at("max_reduction_mw")[t].get<double>()-baseline);
          }
          sum=std::max(0.0,std::min(sum,5*a.at("standard_ramp_mw_min").get<double>()));
        }
        capability=std::min(capability,sum);
      }
      const double declared=a.at("capacity_mw")[h];
      double bid=a.at("price_per_mw")[h];bool fallback=bid<3 || bid>8;
      if(fallback) bid=a.at("default_price_per_mw").is_null()?3:a.at("default_price_per_mw").get<double>();
      const double adjusted=eligible ? std::min({declared,std::floor(0.5*demand+1e-9),std::floor(capability+1e-9)}) : 0;
      Json reasons=Json::array();
      if(!a.at("qualified").get<bool>()) reasons.push_back("AGC资格/控制状态未满足");
      if(!eligible && a.at("qualified").get<bool>()) reasons.push_back("独立资源持续响应、可用状态或跨市场互斥条件未满足");
      if(declared>0.5*demand+1e-6) reasons.push_back("申报超过单主体50%需求上限，研究预安排降额");
      if(declared>capability+1e-6) reasons.push_back("固定开机/一次调频后的5分钟爬坡或允许运行区间容量受限");
      if(fallback) reasons.push_back("越界里程报价采用缺省报价");
      const double ranking=bid*kmax/performance[aidx];
      require(std::isfinite(ranking),"performance normalization overflows; verify k_history units");
      hour["bids"].push_back({{"id",a.at("id")},{"name",a.at("name")},{"k",performance[aidx]},{"eligible",eligible},
        {"effective_price_per_mw",bid},{"ranking_price_per_mw",ranking},
        {"declared_mw",declared},{"capability_upper_bound_mw",capability},{"adjusted_mw",adjusted},{"award_mw",0},{"reasons",reasons}});
      ++aidx;
    }
    std::sort(hour["bids"].begin(),hour["bids"].end(),[](const Json& a,const Json& d){
      if(a.at("ranking_price_per_mw")!=d.at("ranking_price_per_mw"))return a.at("ranking_price_per_mw").get<double>()<d.at("ranking_price_per_mw").get<double>();
      if(a.at("k")!=d.at("k"))return a.at("k").get<double>()>d.at("k").get<double>();
      return a.at("id").get<int>()<d.at("id").get<int>();
    });
    double awarded=0;Json price=nullptr;
    for(auto& bid:hour["bids"]) if(awarded<demand-1e-6 && bid.at("adjusted_mw").get<double>()>0) {
      bid["award_mw"]=bid.at("adjusted_mw");awarded+=bid.at("award_mw").get<double>();
      price=std::min(15.0,bid.at("ranking_price_per_mw").get<double>());
    }
    hour["awarded_mw"]=awarded;hour["shortage_mw"]=std::max(0.0,demand-awarded);
    hour["reference_price_per_mw"]=awarded>=demand-1e-6?price:Json(nullptr);
    if(awarded<demand-1e-6) result["capacity_sufficient"]=false;
    result["hours"].push_back(hour);
  }
  if(!result.at("capacity_sufficient").get<bool>()) result["status"]="capacity_shortfall";
  return result;
}
} // namespace hacdcpf::market::yunnan
