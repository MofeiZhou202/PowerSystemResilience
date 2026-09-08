#pragma once

#include "yunnan_ancillary.hpp"
#include "hacdcpf/market/southern_market.hpp"

namespace hacdcpf::market::realtime {
using Json=nlohmann::json;
inline void require(bool ok,const std::string& message) { if(!ok)throw std::invalid_argument("Southern real-time: "+message); }
inline void fields(const Json& value,const std::set<std::string>& allowed) {
  require(value.is_object(),"expected object");for(auto f=value.begin();f!=value.end();++f)require(allowed.count(f.key()),"unknown field: "+f.key());
  for(const auto& f:allowed)require(value.contains(f),"missing field: "+f);
}
inline double number(const Json& value,double lo,double hi,const std::string& name) {
  require(value.is_number(),name+" must be numeric");const double v=value.get<double>();require(std::isfinite(v)&&v>=lo&&v<=hi,name+" outside bounds");return v;
}
constexpr int forecast_points=72;
inline bool temporal(const std::string& field,const Json& value,int size) {
  static const auto names=[](){
    std::set<std::string> found={"requirement_mw","reduction_mw","online_capacity_mw"};
    const auto schema=southern_market_schema();
    for(const auto& table:schema.at("properties"))if(table.value("type",std::string())=="array"&&table.at("items").contains("properties"))
      for(auto f=table.at("items").at("properties").begin();f!=table.at("items").at("properties").end();++f)
        if(f.value().value("type",std::string())=="array"&&f.value().value("minItems",0)==98&&f.value().value("maxItems",0)==98)found.insert(f.key());
    return found;
  }();
  return names.count(field)&&value.is_array()&&value.size()==static_cast<size_t>(size);
}
inline Json resample(Json boundary,int old_size,const std::vector<int>& indices,double minutes) {
  for(auto it=boundary.begin();it!=boundary.end();++it)if(it.value().is_array()&&it.key()!="periods")
    for(auto& row:it.value())if(row.is_object())for(auto f=row.begin();f!=row.end();++f)if(temporal(f.key(),f.value(),old_size)) {
      Json values=Json::array();for(int t:indices)values.push_back(f.value().at(t));f.value()=values;
    }
  boundary["periods"]=Json::array();for(size_t t=0;t<indices.size();++t)boundary["periods"].push_back({{"kind","day"},{"start_minute",minutes*t},{"duration_hr",minutes/60},{"weight_hr",minutes/60}});
  return boundary;
}
inline Json catalog() {
  return Json::array({
    {{"id","unit_status"},{"clauses","3.2;3.3.1-7"},{"name","并网解列、限高限低、故障、调试及供热"},{"fields",{"generators.available","generators.must_on","generators.must_off","generators.pmin_mw","generators.pmax_mw","generators.initial_power_mw"}}},
    {{"id","hydro"},{"clauses","3.3.8-10;3.4.7-9;3.5.3.11"},{"name","水位、来水、水库运用及水电分时电力电量"},{"fields",{"reservoirs","hydro_plants"}}},
    {{"id","renewable"},{"clauses","3.3.11-13"},{"name","新能源6小时超短期预测及缺测递补、β"},{"fields",{"renewable_forecasts","generators.renewable_alpha"}}},
    {{"id","load"},{"clauses","3.4.1"},{"name","统调与母线超短期负荷预测"},{"fields",{"areas.load_mw","buses.load_mw"}}},
    {{"id","priority"},{"clauses","3.4.2;3.5.3.20"},{"name","跨省优先、区外及联络计划"},{"fields",{"trades.min_mw","trades.max_mw","external_schedules","dc_links"}}},
    {{"id","topology"},{"clauses","3.4.3"},{"name","发输变检修与现场拓扑"},{"fields",{"generators.available","branches.available","branches.x_pu"}}},
    {{"id","reserve"},{"clauses","3.4.4;3.5.3.17"},{"name","运行备用、一次调频及10分钟事故备用"},{"fields",{"areas","accident_reserve","pumped_reserve"}}},
    {{"id","security"},{"clauses","3.4.5;3.8.1"},{"name","网络安全、断面控制裕度与交流校核"},{"fields",{"branches","sections","section_margin_fraction","execution.ac_security"}}},
    {{"id","nonmarket"},{"clauses","3.4.6"},{"name","非市场与抽蓄计划"},{"fields",{"external_schedules","storage"}}},
    {{"id","ancillary"},{"clauses","3.5.2.2-3"},{"name","调频及深调峰已确认边界"},{"fields",{"generators.regulation_up_mw","generators.regulation_down_mw","generators.regulation_source"}}}
  });
}
inline Json defaults(const Json& source) {
  auto b=validate_southern_market(source);std::vector<int> index;for(int t=0;t<72;++t)index.push_back(t/3);
  b=resample(b,98,index,5);
  Json c={{"schema_version","southern-realtime-2025-v1"},{"source","Synthetic 6h forecast and observed-state template from sealed energy boundary; 15min inputs held for three 5min points"},
    {"start_minute",0},{"steps",1},{"boundary",b},{"section_margin_fraction",0},{"reserve_policy","strict"},
    {"reserve_penalty",10000},{"pricing_reserve_penalty",12000},{"accident_reserve",Json::array()},
    {"pumped_reserve",Json::array()},{"hydro_plants",Json::array()},{"renewable_forecasts",Json::array()},
    {"storage_hour_modes",Json::array()}};
  for(const auto& a:b.at("areas"))c["accident_reserve"].push_back({{"id",a.at("id")},{"source","Explicit synthetic zero accident reserve requirement"},{"requirement_mw",std::vector<double>(72,0)},{"reduction_mw",std::vector<double>(72,0)}});
  for(auto& g:c["boundary"]["generators"]) {
    for(auto& curve:g["startup_curves_mw"]){Json expanded=Json::array();for(const auto& v:curve)for(int k=0;k<3;++k)expanded.push_back(v);curve=expanded;}
    Json down=Json::array();for(const auto& v:g.at("shutdown_curve_mw"))for(int k=0;k<3;++k)down.push_back(v);g["shutdown_curve_mw"]=down;
    if(g.at("kind")=="wind"||g.at("kind")=="solar"||g.at("kind")=="renewable")c["renewable_forecasts"].push_back({{"id",g.at("id")},{"source","Synthetic prior-day held forecast"},{"submitted",g.at("forecast_mw")},{"previous_complete_files",Json::array()},{"dispatch_forecast",g.at("forecast_mw")}});
  }
  // This is an explicit uniform decomposition template, not an inference that
  // all daily hydro targets can be attained by independent two-hour windows.
  for(auto& g:c["boundary"]["groups"]){g["min_mwh"]=g.at("min_mwh").get<double>()/12;g["max_mwh"]=g.at("max_mwh").get<double>()/12;}
  for(auto& h:c["boundary"]["reservoirs"]){h["min_mwh"]=h.at("min_mwh").get<double>()/12;h["max_mwh"]=h.at("max_mwh").get<double>()/12;h["lag_slots"]=h.at("lag_slots").get<int>()*3;
    Json history=Json::array();for(const auto& v:h.at("release_history_m3_s"))for(int k=0;k<3;++k)history.push_back(v);h["release_history_m3_s"]=history;
    for(auto& v:h["release_ramp_m3_s"])v=v.get<double>()/3;
    Json plant={{"id",h.at("id")},{"source","Uniform day-to-window hydro target decomposition; revise from dispatch targets"},{"generators",h.at("generators")},{"min_mw",std::vector<double>(72,0)},{"max_mw",std::vector<double>(72,1e6)},{"min_mwh",std::vector<double>(72,0)},{"max_mwh",std::vector<double>(72,1e6)}};c["hydro_plants"].push_back(plant);
  }
  for(auto& d:c["boundary"]["dc_links"])for(const char* key:{"ramp_up_mw","ramp_down_mw"})for(auto& v:d[key])v=v.get<double>()/3;
  for(auto& d:c["boundary"]["controllable_loads"])d["max_day_reduction_mwh"]=d.at("max_day_reduction_mwh").get<double>()/12;
  for(auto& s:c["boundary"]["storage"])s["source"]=s.at("source").get<std::string>()+"; synthetic two-hour terminal target copied from day target; revise from dispatch allocation";
  return c;
}

inline void validate(const Json& source,const Json& c) {
  fields(c,{"schema_version","source","start_minute","steps","boundary","section_margin_fraction","reserve_policy","reserve_penalty","pricing_reserve_penalty","accident_reserve","pumped_reserve","hydro_plants","renewable_forecasts","storage_hour_modes"});
  require(c.at("schema_version")=="southern-realtime-2025-v1","unknown real-time schema");
  require(c.at("source").is_string()&&!c.at("source").get<std::string>().empty(),"real-time source required");
  require(c.at("start_minute").is_number_integer()&&c.at("steps").is_number_integer(),"integer start minute/steps required");
  require(static_cast<int>(number(c.at("start_minute"),0,1080,"start minute"))%15==0,"start must align to 15min");
  number(c.at("steps"),1,8,"steps");number(c.at("section_margin_fraction"),0,.99,"section control margin");
  require(c.at("reserve_policy")=="strict"||c.at("reserve_policy")=="penalized","reserve policy");number(c.at("reserve_penalty"),1,1e9,"reserve penalty");number(c.at("pricing_reserve_penalty"),1,1e9,"reserve pricing penalty");
  const auto sealed=validate_southern_market(source);const auto& b=c.at("boundary");
  require(b.at("periods").is_array()&&b.at("periods").size()==72,"real-time boundary requires 72 five-minute points");
  for(int t=0;t<72;++t)require(b.at("periods")[t]==Json({{"kind","day"},{"start_minute",5*t},{"duration_hr",1.0/12},{"weight_hr",1.0/12}}),"real-time periods must be 5min");
  // Reuse full typed Southern validation through a padded validation-only
  // projection. Solver never sees padding or a 98-point real-time window.
  for(auto table=b.begin();table!=b.end();++table)if(table.value().is_array()&&table.key()!="periods")for(const auto& row:table.value())if(row.is_object())
    for(auto field=row.begin();field!=row.end();++field)if(temporal(field.key(),field.value(),98))require(false,"98-point array in real-time boundary");
  std::vector<int> pad;for(int t=0;t<98;++t)pad.push_back(std::min(t,71));auto validation=resample(b,72,pad,15);validation["periods"]=sealed.at("periods");
  validate_southern_market(validation);
  for(const char* family:{"areas","buses","generators","branches","sections","storage","reservoirs","controllable_loads","dc_links","dc_hubs","trades","groups","primary_groups","external_schedules"}) {
    std::map<int,const Json*> original;for(const auto& a:sealed.at(family))original[a.at("id")]=&a;
    require(original.size()==b.at(family).size(),"real-time cannot change entity membership; replace sealed boundary first");
    for(const auto& a:b.at(family)){require(original.count(a.at("id")),"real-time unknown stable ID");const auto& old=*original.at(a.at("id"));
      for(const char* field:{"bus","area","kind","from_bus","to_bus","from_hub","to_hub","generators","generator","members","upstream","gateway_kind","gateway","direction"})if(old.contains(field))require(a.at(field)==old.at(field),"real-time identity/membership changed");
      for(const char* field:{"segments","bid_mode","startup_cost","minimum_cost_per_hour","charge_price","discharge_price","compensation_per_mwh","scuc_fee","sced_fee","lmp_fee","original_min_mwh","adjusted_min_mwh"})if(old.contains(field)&&!old.at(field).is_array())require(a.at(field)==old.at(field),"sealed commercial field changed");
      if(std::string(family)=="controllable_loads")for(int t=0;t<72;++t)require(a.at("compensation_per_mwh")[t]==old.at("compensation_per_mwh")[c.at("start_minute").get<int>()/15+t/3],"sealed load compensation changed");
      if(std::string(family)=="generators") {require(a.at("segments")==old.at("segments")&&a.at("startup_cost")==old.at("startup_cost"),"sealed generator bids changed");
        require(a.at("shutdown_curve_mw").empty()&&std::all_of(a.at("startup_curves_mw").begin(),a.at("startup_curves_mw").end(),[](const Json& x){return x.empty();}),"real-time rolling startup/shutdown trajectories require observed phase history; use confirmed fixed output boundaries");}
    }
  }
  auto records=[&](const char* name){require(c.at(name).is_array(),std::string(name)+" must be array");};
  for(const char* name:{"accident_reserve","pumped_reserve","hydro_plants","renewable_forecasts","storage_hour_modes"})records(name);
  auto sourced=[](const Json& r){require(r.at("source").is_string()&&!r.at("source").get<std::string>().empty(),"boundary record source required");};
  std::set<int> areas;for(const auto& a:c.at("accident_reserve")){fields(a,{"id","source","requirement_mw","reduction_mw"});sourced(a);require(a.at("id").is_number_integer()&&areas.insert(a.at("id").get<int>()).second,"duplicate accident reserve area");yunnan::series(a.at("requirement_mw"),72,0,1e6,0,"accident MW");yunnan::series(a.at("reduction_mw"),72,0,1e6,0,"accident reduction");}
  require(areas.size()==b.at("areas").size(),"accident reserve must cover all areas");for(const auto& a:b.at("areas"))require(areas.count(a.at("id")),"unknown accident reserve area");
  std::set<int> pumps;for(const auto& p:c.at("pumped_reserve")){fields(p,{"id","area","source","online_capacity_mw"});sourced(p);require(p.at("id").is_number_integer()&&pumps.insert(p.at("id").get<int>()).second,"duplicate pumped reserve");require(areas.count(p.at("area")),"unknown pumped reserve area");yunnan::series(p.at("online_capacity_mw"),72,0,1e6,0,"pumped reserve cap");bool found=false;for(const auto& e:b.at("external_schedules"))if(e.at("id")==p.at("id")){require(e.at("kind")=="pumped_storage","reserve source is not pumped storage");for(int t=0;t<72;++t)require(e.at("power_mw")[t].get<double>()<=p.at("online_capacity_mw")[t].get<double>(),"pumped reserve below fixed generation");found=true;}require(found,"unknown pumped plan");}
  std::set<int> plants;std::set<int> hydro_members;for(const auto& p:c.at("hydro_plants")){fields(p,{"id","source","generators","min_mw","max_mw","min_mwh","max_mwh"});sourced(p);require(p.at("id").is_number_integer()&&plants.insert(p.at("id").get<int>()).second,"duplicate hydro plant");require(p.at("generators").is_array()&&!p.at("generators").empty(),"hydro members required");for(const auto& id:p.at("generators")){require(id.is_number_integer()&&hydro_members.insert(id.get<int>()).second,"duplicate hydro plant member");bool found=false;for(const auto& g:b.at("generators"))if(g.at("id")==id&&g.at("kind")=="hydro")found=true;require(found,"unknown hydro plant member");}
    for(const char* field:{"min_mw","max_mw","min_mwh","max_mwh"})yunnan::series(p.at(field),72,0,1e6,0,field);
    for(int t=0;t<72;++t)require(p.at("min_mw")[t]<=p.at("max_mw")[t]&&p.at("min_mwh")[t]<=p.at("max_mwh")[t],"reversed hydro limits");}
  for(const auto& p:c.at("pumped_reserve"))for(const auto& e:b.at("external_schedules"))if(e.at("id")==p.at("id"))for(const auto& bus:b.at("buses"))if(bus.at("id")==e.at("bus"))require(bus.at("area")==p.at("area"),"pumped reserve province disagrees with physical bus");
  std::set<int> forecasts;for(const auto& p:c.at("renewable_forecasts")){fields(p,{"id","source","submitted","previous_complete_files","dispatch_forecast"});sourced(p);require(p.at("id").is_number_integer()&&forecasts.insert(p.at("id").get<int>()).second,"duplicate renewable forecast");
    auto nullable=[](const Json& v){require(v.is_array()&&v.size()==72,"forecast dimension must be72");for(const auto& x:v)if(!x.is_null())number(x,0,1e6,"forecast MW");};nullable(p.at("submitted"));nullable(p.at("dispatch_forecast"));require(p.at("previous_complete_files").is_array()&&p.at("previous_complete_files").size()<=32,"forecast history length");for(const auto& file:p.at("previous_complete_files"))yunnan::series(file,72,0,1e6,0,"previous complete forecast");
    bool found=false;for(const auto& g:b.at("generators"))if(g.at("id")==p.at("id")&&(g.at("kind")=="wind"||g.at("kind")=="solar"||g.at("kind")=="renewable"))found=true;require(found,"unknown renewable forecast ID");}
  for(const auto& g:b.at("generators"))if(g.at("kind")=="wind"||g.at("kind")=="solar"||g.at("kind")=="renewable")require(forecasts.count(g.at("id")),"renewable forecast missing");
  std::set<std::pair<int,int>> modes;for(const auto& p:c.at("storage_hour_modes")){fields(p,{"id","hour","mode","source"});sourced(p);require(p.at("id").is_number_integer()&&p.at("hour").is_number_integer(),"integer storage mode identity");number(p.at("hour"),0,23,"storage hour");require(modes.emplace(p.at("id"),p.at("hour")).second,"duplicate storage hour mode");require(p.at("mode")=="charge"||p.at("mode")=="discharge","storage mode");bool found=false;for(const auto& s:b.at("storage"))found=found||s.at("id")==p.at("id");require(found,"unknown storage hour mode");}
}
inline Json prepare(const Json& source,const Json& c) {
  validate(source,c);auto out=c;auto& b=out["boundary"];
  out["forecast_resolution"]=Json::array();
  for(const auto& f:c.at("renewable_forecasts"))for(auto& g:b["generators"])if(g.at("id")==f.at("id"))for(int t=0;t<72;++t){Json v=f.at("submitted")[t];std::string origin="submitted";
    if(v.is_null())for(const auto& previous:f.at("previous_complete_files")){v=previous[t];origin="previous_complete_file";break;}
    if(v.is_null()){v=f.at("dispatch_forecast")[t];origin="dispatch_forecast";}
    if(v.is_null()){const int slot=c.at("start_minute").get<int>()/15+t/3;for(const auto& old:source.at("generators"))if(old.at("id")==g.at("id"))v=old.at("forecast_mw")[slot];origin="day_ahead_fallback";}
    require(!v.is_null(),"renewable fallback missing");g["forecast_mw"][t]=v;
    out["forecast_resolution"].push_back({{"id",g.at("id")},{"point",t},{"mw",v},{"origin",origin},{"source",f.at("source")},{"assessment_eligible",origin=="submitted"}});
  }
  const double margin=c.at("section_margin_fraction");if(margin>0)for(auto& s:b["sections"])for(int t=0;t<72;++t){require(s.at("min_mw")[t].get<double>()<=0&&s.at("max_mw")[t].get<double>()>=0,"nonzero section control margin requires signed bidirectional limits");s["min_mw"][t]=s.at("min_mw")[t].get<double>()*(1-margin);s["max_mw"][t]=s.at("max_mw")[t].get<double>()*(1-margin);}
  // Preserve normalized bus-load semantics from the shared boundary resolver.
  std::vector<int> pad;for(int t=0;t<98;++t)pad.push_back(std::min(t,71));auto normalized=resample(b,72,pad,15);normalized["periods"]=source.at("periods");normalized=validate_southern_market(normalized);
  std::vector<int> first;for(int t=0;t<72;++t)first.push_back(t);out["boundary"]=resample(normalized,98,first,5);return out;
}
inline Json window(const Json& c,int offset) {
  std::vector<int> points;for(int t=0;t<24;++t)points.push_back(offset+t);auto b=resample(c.at("boundary"),72,points,5);
  Json rt={{"start_minute",c.at("start_minute").get<int>()+5*offset},{"reserve_policy",c.at("reserve_policy")},{"reserve_penalty",c.at("reserve_penalty")},{"pricing_reserve_penalty",c.at("pricing_reserve_penalty")},{"storage_hour_modes",c.at("storage_hour_modes")}};
  for(const char* family:{"accident_reserve","pumped_reserve","hydro_plants"}){rt[family]=c.at(family);for(auto& row:rt[family])for(auto f=row.begin();f!=row.end();++f)if(temporal(f.key(),f.value(),72)){Json values=Json::array();for(int i:points)values.push_back(f.value()[i]);f.value()=values;}}
  b["_rt"]=rt;return b;
}
inline void carry(Json& next,const Json& previous,const Json& stage,int count) {
  for(const auto& mode:previous.at("_rt").at("storage_hour_modes")) {
    bool found=false;for(const auto& existing:next["_rt"]["storage_hour_modes"])if(existing.at("id")==mode.at("id")&&existing.at("hour")==mode.at("hour")){require(existing.at("mode")==mode.at("mode"),"carried storage direction conflicts with authored history");found=true;}
    if(!found)next["_rt"]["storage_hour_modes"].push_back(mode);
  }
  for(auto& g:next["generators"])for(const auto& old:stage.at("generators"))if(g.at("id")==old.at("id")) {
    const Json* prior=nullptr;for(const auto& p:previous.at("generators"))if(p.at("id")==g.at("id"))prior=&p;
    bool on=prior->at("initial_on").get<int>()!=0;double minutes=prior->at("initial_state_minutes");int starts=0,stops=0;
    for(int t=0;t<count;++t){const bool current=old.at("online")[t].get<double>()>.5;if(current!=on){current?++starts:++stops;minutes=0;}minutes+=5;on=current;}
    g["initial_on"]=on?1:0;g["initial_power_mw"]=old.at("power_mw")[count-1];g["initial_state_minutes"]=minutes;
    g["max_starts"]=std::max(0,prior->at("max_starts").get<int>()-starts);g["max_stops"]=std::max(0,prior->at("max_stops").get<int>()-stops);
  }
  for(auto& s:next["storage"])for(const auto& old:stage.at("storage"))if(s.at("id")==old.at("id")) {
    s["initial_mwh"]=old.at("energy_mwh")[count-1];
    const Json* prior=nullptr;for(const auto& p:previous.at("storage"))if(p.at("id")==s.at("id"))prior=&p;
    double throughput=0;const double eta=std::sqrt(prior->at("roundtrip_efficiency").get<double>());
    for(int t=0;t<count;++t)throughput+=(old.at("discharge_mw")[t].get<double>()/eta-old.at("charge_mw")[t].get<double>()*eta)/12;
    s["max_cycles"]=std::max(0.0,prior->at("max_cycles").get<double>()-throughput/(2*prior->at("rated_mwh").get<double>()));
    for(int t=0;t<count;++t){const double dis=old.at("discharge_mw")[t],ch=old.at("charge_mw")[t];if(dis<=1e-6&&ch>=-1e-6)continue;
      const int hour=(previous.at("_rt").at("start_minute").get<int>()+5*t)/60;bool found=false;for(const auto& mode:next["_rt"]["storage_hour_modes"])if(mode.at("id")==s.at("id")&&mode.at("hour")==hour)found=true;
      if(!found)next["_rt"]["storage_hour_modes"].push_back({{"id",s.at("id")},{"hour",hour},{"mode",dis>1e-6?"discharge":"charge"},{"source","carried executed five-minute schedule"}});
    }
  }
  for(auto& h:next["reservoirs"])for(const auto& old:stage.at("reservoirs"))if(h.at("id")==old.at("id")) {
    h["initial_level_m"]=old.at("level_m")[count-1];h["initial_release_m3_s"]=old.at("release_m3_s")[count-1];
    for(const auto& prior:previous.at("reservoirs"))if(prior.at("id")==h.at("id")) {Json history=prior.at("release_history_m3_s");for(int t=0;t<count;++t)history.push_back(old.at("release_m3_s")[t]);while(history.size()>96)history.erase(0);h["release_history_m3_s"]=history;}
  }
  for(auto& d:next["dc_links"])for(const auto& old:stage.at("dc_links"))if(d.at("id")==old.at("id")){d["initial_mw"]=old.at("power_mw")[count-1];const double up=old.at("up")[count-1],down=old.at("down")[count-1];d["initial_adjustment"]=up>.5?1:down>.5?-1:0;}
}
} // namespace hacdcpf::market::realtime
