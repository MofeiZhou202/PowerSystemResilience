#pragma once

#include "yunnan_ancillary.hpp"
#include <numeric>
#include <tuple>
#include <chrono>

namespace hacdcpf::market::yunnan {
inline void text_source(const Json& value,const char* name) {
  require(value.is_string() && !value.get<std::string>().empty() && value.get_ref<const std::string&>().size()<=4096,std::string(name)+" requires a nonempty source");
}
inline int integer(const Json& value,int lo,int hi,const char* name) {
  require(value.is_number_integer(),std::string(name)+" must be integer");return static_cast<int>(number(value,lo,hi,name));
}
inline bool boolean(const Json& value,const char* name) { require(value.is_boolean(),std::string(name)+" must be boolean");return value.get<bool>(); }
inline Json workflow_defaults() {
  return {{"stage","day_ahead"},{"clearing_minutes",std::vector<double>(24,-660)},
    {"bid_submissions",Json::array()},{"safety_reviews",Json::array()},
    {"allow_uplift",true},{"suspensions",Json::array()},{"realtime_adjustments",Json::array()}};
}
inline Json complete_defaults(const Json& b) {
  auto c=defaults(b);c["workflow"]=workflow_defaults();c["independent_units"]=Json::array();
  int id=static_cast<int>(c.at("agc_units").size());
  for(const char* family:{"storage","controllable_loads"})for(const auto& r:b.at(family)) {
    const bool storage=std::string(family)=="storage";
    c["independent_units"].push_back({{"id",id++},{"resource_id",r.at("id")},{"name",r.at("name")},
      {"mode",storage?"storage":"load"},{"qualified",false},{"source","Unverified independent AGC resource; supply qualification and sustained-response evidence"},
      {"k_history",std::vector<double>(8,1)},{"price_per_mw",std::vector<double>(24,5)},
      {"default_price_per_mw",nullptr},{"capacity_mw",std::vector<double>(24,0)},
      {"standard_ramp_mw_min",0},{"sustained_hours",0},{"baseline_reduction_mw",std::vector<double>(24,0)},
      {"cross_province_reserve_mw",std::vector<double>(24,0)}});
  }
  return c;
}
inline void validate_workflow(const Json& w,const Json& c) {
  fields(w,{"stage","clearing_minutes","bid_submissions","safety_reviews","allow_uplift","suspensions","realtime_adjustments"});
  require(w.at("stage")=="day_ahead" || w.at("stage")=="intraday","stage must be day_ahead or intraday");
  series(w.at("clearing_minutes"),24,-660,1410,0,"clearing_minutes");
  for(int h=0;h<24;++h) require(w.at("clearing_minutes")[h].get<double>()<=60*h-30,"art.29: clearing must be at least 30 minutes before the hour");
  boolean(w.at("allow_uplift"),"allow_uplift");
  std::set<int> ids;for(const auto& a:c.at("agc_units"))ids.insert(a.at("id").get<int>());
  if(c.contains("independent_units"))for(const auto& a:c.at("independent_units"))require(ids.insert(a.at("id").get<int>()).second,"duplicate independent AGC id");
  auto identity=[&](const Json& item){require(ids.count(integer(item.at("unit_id"),0,1000000,"unit_id")),"unknown AGC in workflow");integer(item.at("hour"),0,23,"hour");};
  for(const char* list:{"bid_submissions","safety_reviews","suspensions","realtime_adjustments"})require(w.at(list).is_array()&&w.at(list).size()<=100000,std::string(list)+" must be a bounded array");
  std::set<std::string> submissions;
  for(const auto& s:w.at("bid_submissions")) {
    fields(s,{"submission_id","unit_id","hour","minute","price_per_mw","capacity_mw","source"});identity(s);
    text_source(s.at("submission_id"),"submission_id");require(submissions.insert(s.at("submission_id").get<std::string>()).second,"duplicate submission_id");
    text_source(s.at("source"),"submission source");number(s.at("minute"),-900,-720,"D-1 bid window");
    number(s.at("price_per_mw"),-1e6,1e6,"bid price");number(s.at("capacity_mw"),0,1e6,"bid capacity");
  }
  std::set<std::pair<int,int>> reviews;
  for(const auto& s:w.at("safety_reviews")) {
    fields(s,{"unit_id","hour","up_mw","down_mw","agc_available","reason","source"});identity(s);
    require(reviews.emplace(s.at("unit_id"),s.at("hour")).second,"duplicate hourly safety review");
    number(s.at("up_mw"),0,1e6,"review up MW");number(s.at("down_mw"),0,1e6,"review down MW");
    boolean(s.at("agc_available"),"AGC available");text_source(s.at("reason"),"safety reason");text_source(s.at("source"),"safety evidence");
  }
  std::set<int> suspended;
  for(const auto& s:w.at("suspensions")) {
    fields(s,{"hour","previous_trading_day_price_per_mw","source"});
    require(suspended.insert(integer(s.at("hour"),0,23,"hour")).second,"duplicate suspended hour");
    number(s.at("previous_trading_day_price_per_mw"),0,15,"previous trading day price");text_source(s.at("source"),"suspension source");
  }
  std::set<std::string> changes;
  for(const auto& s:w.at("realtime_adjustments")) {
    fields(s,{"event_id","unit_id","hour","second","capacity_mw","reason","source"});identity(s);
    text_source(s.at("event_id"),"event_id");require(changes.insert(s.at("event_id").get<std::string>()).second,"duplicate adjustment event");
    number(s.at("second"),0,3599.999,"adjustment second");number(s.at("capacity_mw"),0,1e6,"adjusted MW");
    text_source(s.at("reason"),"adjustment reason");text_source(s.at("source"),"adjustment source");
  }
  if(w.at("stage")=="day_ahead") require(w.at("suspensions").empty()&&w.at("realtime_adjustments").empty(),"real-time interventions require intraday results");
  std::set<std::tuple<int,int,double>> instants;
  for(const auto& e:w.at("realtime_adjustments")) require(instants.emplace(e.at("unit_id"),e.at("hour"),e.at("second")).second,"conflicting simultaneous real-time changes");
}
inline void validate_full(const Json& b,const Json& c) {
  validate(b,c);
  std::set<int> ids;for(const auto& a:c.at("agc_units"))ids.insert(a.at("id").get<int>());
  std::set<std::pair<std::string,int>> resources;
  const auto independent=c.value("independent_units",Json::array());
  require(independent.is_array()&&independent.size()<=b.at("storage").size()+b.at("controllable_loads").size(),"invalid independent AGC list");
  for(const auto& a:independent) {
    fields(a,{"id","resource_id","name","mode","qualified","source","k_history","price_per_mw","default_price_per_mw","capacity_mw","standard_ramp_mw_min","sustained_hours","baseline_reduction_mw","cross_province_reserve_mw"});
    require(ids.insert(integer(a.at("id"),0,1000000,"AGC id")).second,"duplicate AGC id");
    require(a.at("mode")=="storage"||a.at("mode")=="load","independent mode must be storage or load");
    const std::string mode=a.at("mode");const int rid=integer(a.at("resource_id"),0,1000000,"resource ID");
    require(resources.emplace(mode,rid).second,"duplicate independent resource");
    bool found=false;for(const auto& r:b.at(mode=="storage"?"storage":"controllable_loads"))found=found||r.at("id")==rid;
    require(found,"unknown independent resource");text_source(a.at("name"),"name");text_source(a.at("source"),"qualification and sustain source");
    boolean(a.at("qualified"),"qualified");series(a.at("k_history"),8,-1e6,1e6,0,"k_history");
    double k=0;for(const auto& v:a.at("k_history"))k+=v.get<double>()/8;require(k>0,"positive last-eight mean required");
    series(a.at("price_per_mw"),24,-1e6,1e6,.1,"price");series(a.at("capacity_mw"),24,0,1e6,1,"capacity");
    if(!a.at("default_price_per_mw").is_null()){double q=number(a.at("default_price_per_mw"),3,8,"default price");require(std::abs(q*10-std::round(q*10))<1e-7,"default price increment");}
    number(a.at("standard_ramp_mw_min"),0,1e6,"AGC ramp");number(a.at("sustained_hours"),0,1e6,"sustained hours");
    series(a.at("baseline_reduction_mw"),24,0,1e6,0,"load baseline");series(a.at("cross_province_reserve_mw"),24,0,1e6,0,"external reserve");
    if(mode=="storage")for(const auto& v:a.at("baseline_reduction_mw"))require(v.get<double>()==0,"storage AGC baseline is zero");
  }
  if(c.contains("workflow"))validate_workflow(c.at("workflow"),c);
}

// Arts.30-33: last valid sealed declaration, with price fallback. Capacity or
// precision-invalid declarations do not replace a prior valid declaration.
inline Json seal_bids(const Json& config, const Json& boundary) {
  Json c=config;if(!c.contains("workflow"))return c;
  std::vector<Json> submissions=c.at("workflow").at("bid_submissions").get<std::vector<Json>>();
  std::stable_sort(submissions.begin(),submissions.end(),[](const Json& a,const Json& b){return std::make_pair(a.at("minute").get<double>(),a.at("submission_id").get<std::string>())<std::make_pair(b.at("minute").get<double>(),b.at("submission_id").get<std::string>());});
  for(const auto& s:submissions) {
    double price=s.at("price_per_mw"),cap=s.at("capacity_mw");
    if(std::abs(price*10-std::round(price*10))>1e-7 || std::abs(cap-std::round(cap))>1e-7)continue;
    const int hour=s.at("hour");double load=0,renewable=0;
    for(int t=4*hour;t<4*hour+4;++t){double l=0,r=0;for(const auto& a:boundary.at("areas"))l+=a.at("load_mw")[t].get<double>();for(const auto& g:boundary.at("generators"))if(g.at("kind")=="wind"||g.at("kind")=="solar"||g.at("kind")=="renewable")r+=g.at("forecast_mw")[t].get<double>();load=std::max(load,l);renewable=std::max(renewable,r);}
    const double demand=c.at("cmin_mw").get<double>()+c.at("load_ratio").get<double>()*load+c.at("renewable_ratio").get<double>()*renewable;
    if(cap>0.5*demand+1e-6)continue;
    for(const char* family:{"agc_units","independent_units"}) if(c.contains(family))for(auto& a:c[family])if(a.at("id")==s.at("unit_id")) {
      const int h=s.at("hour");a["price_per_mw"][h]=price;a["capacity_mw"][h]=cap;
    }
  }
  return c;
}

// Arts.35-42: disclosed safety removal/derating, outside-sequence backfill,
// then upward revision. Full-block bids are kept; MW uplift may be fractional.
inline void adjust_awards(Json& report,const Json& workflow) {
  report["workflow"]=workflow;report["adjustment_log"]=Json::array();
  report["capacity_sufficient"]=true;
  for(auto& h:report["hours"]) {
    const int hour=h.at("hour");const double demand=h.at("demand_mw");
    for(auto& bid:h["bids"]) {bid["initial_award_mw"]=bid.at("award_mw");bid["safe_capacity_mw"]=bid.at("capability_upper_bound_mw");}
    for(auto it=h["bids"].rbegin();it!=h["bids"].rend();++it) {
      auto& bid=*it;double safe=bid.at("capability_upper_bound_mw");
      for(const auto& review:workflow.at("safety_reviews"))if(review.at("hour")==hour && review.at("unit_id")==bid.at("id")) {
        safe=review.at("agc_available").get<bool>()?std::min({safe,review.at("up_mw").get<double>(),review.at("down_mw").get<double>()}):0;
        bid["reasons"].push_back(review.at("reason"));bid["safety_evidence"]=review;
      }
      if(bid.contains("eligible") && !bid.at("eligible").get<bool>())safe=0;
      bid["safe_capacity_mw"]=std::min(safe,0.5*demand);
      double old=bid.at("award_mw"),next=std::min(old,bid.at("safe_capacity_mw").get<double>());
      if(next<old-1e-6) report["adjustment_log"].push_back({{"hour",hour},{"unit_id",bid.at("id")},{"action","safety_remove_or_reduce"},{"before_mw",old},{"after_mw",next},{"evidence",bid.value("safety_evidence",Json(nullptr))}});
      bid["award_mw"]=next;
    }
    auto total=[&](){double sum=0;for(const auto& b:h.at("bids"))sum+=b.at("award_mw").get<double>();return sum;};
    double sum=total();
    for(auto& bid:h["bids"])if(sum<demand-1e-6 && bid.at("initial_award_mw").get<double>()==0) {
      double next=std::min(bid.at("adjusted_mw").get<double>(),bid.at("safe_capacity_mw").get<double>());
      if(next>0) {bid["award_mw"]=next;sum+=next;report["adjustment_log"].push_back({{"hour",hour},{"unit_id",bid.at("id")},{"action","backfill"},{"before_mw",0},{"after_mw",next}});}
    }
    if(workflow.at("allow_uplift").get<bool>())for(auto& bid:h["bids"])if(sum<demand-1e-6) {
      double old=bid.at("award_mw"),delta=std::min(demand-sum,bid.at("safe_capacity_mw").get<double>()-old);
      if(delta>1e-6) {bid["award_mw"]=old+delta;sum+=delta;report["adjustment_log"].push_back({{"hour",hour},{"unit_id",bid.at("id")},{"action","safety_capacity_uplift"},{"before_mw",old},{"after_mw",old+delta}});}
    }
    Json price=nullptr;for(const auto& bid:h.at("bids"))if(bid.at("award_mw").get<double>()>1e-6)price=std::min(15.0,bid.at("ranking_price_per_mw").get<double>());
    h["awarded_mw"]=sum;h["shortage_mw"]=std::max(0.0,demand-sum);
    h["reference_price_per_mw"]=sum>=demand-1e-6?price:Json(nullptr);
    h["clearing_price_per_mw"]=workflow.at("stage")=="intraday"?h.at("reference_price_per_mw"):Json(nullptr);
    for(const auto& s:workflow.at("suspensions"))if(s.at("hour")==hour) {h["clearing_price_per_mw"]=s.at("previous_trading_day_price_per_mw");h["suspension"]=s;}
    if(sum<demand-1e-6)report["capacity_sufficient"]=false;
  }
  report["status"]=report.at("capacity_sufficient").get<bool>()?"capacity_prearranged":"capacity_shortfall";
  report["realtime_dispatch"]=Json::array();
  auto changes=workflow.at("realtime_adjustments").get<std::vector<Json>>();
  std::stable_sort(changes.begin(),changes.end(),[](const Json& a,const Json& b){return std::make_tuple(a.at("hour").get<int>(),a.at("second").get<double>(),a.at("event_id").get<std::string>())<std::make_tuple(b.at("hour").get<int>(),b.at("second").get<double>(),b.at("event_id").get<std::string>());});
  for(const auto& change:changes) {
    const auto& hour=report.at("hours")[change.at("hour").get<int>()];
    for(const auto& b:hour.at("bids"))if(b.at("id")==change.at("unit_id"))
      require(change.at("capacity_mw").get<double>()<=b.at("safe_capacity_mw").get<double>()+1e-6,"real-time adjustment exceeds reviewed safe capability");
    auto row=change;row["price_per_mw"]=hour.at("clearing_price_per_mw");report["realtime_dispatch"].push_back(row);
  }
  report["realtime_capacity_sufficient"]=true;
  report["realtime_capacity_audit"]=Json::array();
  for(const auto& hour:report.at("hours")) {
    std::map<int,double> capacities;for(const auto& bid:hour.at("bids"))capacities[bid.at("id")]=bid.at("award_mw");
    std::map<double,std::vector<Json>> batches;for(const auto& e:report.at("realtime_dispatch"))if(e.at("hour")==hour.at("hour"))batches[e.at("second")].push_back(e);
    for(const auto& [second,batch]:batches){for(const auto& e:batch)capacities.at(e.at("unit_id"))=e.at("capacity_mw");double sum=0;for(auto [id,mw]:capacities)sum+=mw;const double shortage=std::max(0.0,hour.at("demand_mw").get<double>()-sum);
      report["realtime_capacity_audit"].push_back({{"hour",hour.at("hour")},{"second",second},{"awarded_mw",sum},{"shortage_mw",shortage}});
      if(shortage>1e-6)report["realtime_capacity_sufficient"]=false;
    }
  }
}

// Conservative hour envelope: reserve every capacity requested during the hour.
// A successful SCED covers every capacity combination; no AGC trajectory claim.
inline double required_capacity(const Json& report,int id,int t) {
  if(t>=96)return 0;double value=0;
  for(const auto& b:report.at("hours")[t/4].at("bids"))if(b.at("id")==id)value=b.at("award_mw");
  for(const auto& e:report.value("realtime_dispatch",Json::array()))if(e.at("unit_id")==id&&e.at("hour")==t/4)value=std::max(value,e.at("capacity_mw").get<double>());
  return value;
}

// Appendices1/2. All rates/errors are rated-power fractions, not mixed MW/min.
inline Json performance_metrics(const Json& m) {
  fields(m,{"rate_fraction_per_min","delay_seconds","error_fraction","fleet_standard_fraction_per_min","standard_delay_seconds","allowed_error_fraction"});
  const double rate=number(m.at("rate_fraction_per_min"),0,100,"rate"),delay=number(m.at("delay_seconds"),0,3600,"delay"),error=number(m.at("error_fraction"),0,100,"error");
  const double fleet=number(m.at("fleet_standard_fraction_per_min"),1e-12,100,"fleet rate"),standard=number(m.at("standard_delay_seconds"),1e-12,3600,"standard delay"),allowed=number(m.at("allowed_error_fraction"),1e-12,100,"allowed error");
  const double k1=rate/fleet,k2=1-delay/standard,k3=1-error/allowed;
  const double m1=std::min(4.0,rate/.015),m2=1-delay/60,m3=1-error/.01;
  return {{"k1",k1},{"k2",k2},{"k3",k3},{"k",(k1+k2+k3)/3},{"m1",m1},{"m2",m2},{"m3",m3},{"m",std::min(2.0,(m1+m2+m3)/3)}};
}

inline double union_seconds(const Json& intervals) {
  require(intervals.is_array(),"own unavailability must be intervals");std::vector<std::pair<double,double>> sorted;
  for(const auto& v:intervals) {require(v.is_array()&&v.size()==2,"unavailability interval requires start/end seconds");double a=number(v[0],0,3600,"start second"),b=number(v[1],0,3600,"end second");require(a<b,"empty/reversed unavailability");sorted.emplace_back(a,b);}
  std::sort(sorted.begin(),sorted.end());double total=0,end=0;
  for(auto [a,b]:sorted){total+=std::max(0.0,b-std::max(a,end));end=std::max(end,b);}return total;
}

// Arts.14,18,45,46,52. Measurements are explicit command/response events;
// missing measurement rows are unknown, never fabricated zero deliveries.
inline Json meter(const Json& clearing,const Json& measurements) {
  require(clearing.value("coupled_schedule_feasible",false),"settlement needs a feasible coupled energy schedule");
  require(clearing.value("realtime_capacity_sufficient",true),"real-time capacity shortage prevents complete settlement");
  require(!clearing.value("diagnostic",false),"diagnostic slack schedules cannot support settlement");
  require(clearing.value("settlement_eligible",false),"clearing status does not permit settlement");
  require(clearing.contains("workflow")&&clearing.at("workflow").at("stage")=="intraday","settlement requires intraday clearing");
  require(measurements.is_array(),"measurements must be an array");
  Json out={{"rows",Json::array()},{"total_compensation_cny",0},{"measurement_scope","authored AGC events; source authenticity is external"}};
  std::set<std::pair<int,int>> seen;std::set<std::string> events;
  double total=0;
  for(const auto& row:measurements) {
    fields(row,{"unit_id","hour","source","test_period","own_unavailability_seconds","events"});
    int id=integer(row.at("unit_id"),0,1000000,"AGC id"),h=integer(row.at("hour"),0,23,"hour");
    require(seen.emplace(id,h).second,"duplicate hourly measurement");text_source(row.at("source"),"measurement source");
    bool testing=boolean(row.at("test_period"),"test_period");const double unavailable=union_seconds(row.at("own_unavailability_seconds"));
    const auto& hour=clearing.at("hours")[h];const Json* bidder=nullptr;
    for(const auto& a:hour.at("bids"))if(a.at("id")==id)bidder=&a;
    require(bidder!=nullptr,"measurement refers to unknown AGC");
    require(!hour.at("clearing_price_per_mw").is_null(),"hour has no valid settlement price");
    require(hour.at("shortage_mw").get<double>()<=1e-6,"shortage hour has no valid settlement clearance");
    require(row.at("events").is_array(),"explicit AGC measurement events required (empty means sourced zero commands)");
    double mileage=0,msum=0;int count=0;double last_end=-1;Json metrics=Json::array();
    auto commands=row.at("events").get<std::vector<Json>>();
    for(const auto& e:commands){fields(e,{"event_id","start_second","end_second","start_mw","end_mw","autor","metrics"});number(e.at("start_second"),0,3599.999,"command second");}
    std::sort(commands.begin(),commands.end(),[](const Json& a,const Json& b){return a.at("start_second").get<double>()<b.at("start_second").get<double>();});
    for(const auto& e:commands) {
      text_source(e.at("event_id"),"AGC event ID");require(events.insert(e.at("event_id").get<std::string>()).second,"duplicate AGC event ID");
      double start=number(e.at("start_second"),0,3599.999,"command second"),end=number(e.at("end_second"),0,3600,"response end");
      require(start>=last_end&&end>start,"overlapping or reversed AGC command responses");last_end=end;
      double award=bidder->at("award_mw");
      for(const auto& change:clearing.value("realtime_dispatch",Json::array()))if(change.at("hour")==h&&change.at("unit_id")==id) {
        const double second=change.at("second");require(!(second>start&&second<end),"split telemetry at real-time award changes");
        if(second<=start)award=change.at("capacity_mw");
      }
      require(award>1e-6||testing,"unawarded, uncalled AGC cannot receive compensation");
      const double from=number(e.at("start_mw"),-1e6,1e6,"start MW"),to=number(e.at("end_mw"),-1e6,1e6,"end MW");
      auto perf=performance_metrics(e.at("metrics"));metrics.push_back(perf);
      require(perf.at("m").get<double>()>=0,"negative performance coefficient requires external statistical-rule resolution");
      if(boolean(e.at("autor"),"AUTOR")) {mileage+=std::abs(to-from);msum+=perf.at("m").get<double>();++count;}
    }
    double m=count?msum/count:0,price=hour.at("clearing_price_per_mw");
    double pay=!testing&&unavailable<=300?mileage*price*m:0;total+=pay;
    out["rows"].push_back({{"unit_id",id},{"hour",h},{"source",row.at("source")},{"mileage_mw",mileage},{"performance_m",m},{"price_per_mw",price},{"own_unavailability_seconds",unavailable},{"test_period",testing},{"compensation_cny",pay},{"metrics",metrics}});
  }
  out["total_compensation_cny"]=total;
  out["complete"]=true;out["missing_measurements"]=Json::array();
  for(const auto& h:clearing.at("hours"))for(const auto& b:h.at("bids")) {
    bool called=b.at("award_mw").get<double>()>1e-6;
    for(const auto& e:clearing.value("realtime_dispatch",Json::array()))if(e.at("hour")==h.at("hour")&&e.at("unit_id")==b.at("id")&&e.at("capacity_mw").get<double>()>1e-6)called=true;
    if(called&&!seen.count({b.at("id"),h.at("hour")})) {out["complete"]=false;out["missing_measurements"].push_back({{"unit_id",b.at("id")},{"hour",h.at("hour")}});}
  }
  return out;
}

// Arts.47-50: separate compensation and assessment accounts. Point-to-grid
// export weights and receipts are halved before forming their denominators.
inline Json allocate(const Json& request) {
  fields(request,{"continuous_spot","generation_share","assessment_pool_cny","participants"});
  const bool continuous=boolean(request.at("continuous_spot"),"continuous_spot");
  const double share=number(request.at("generation_share"),0,1,"generation share"),assessment=number(request.at("assessment_pool_cny"),0,1e15,"assessment pool");
  require(request.at("participants").is_array()&&!request.at("participants").empty(),"allocation participants required");
  double pool=0,gen_weight=0,user_weight=0,return_weight=0;std::set<int> ids;
  for(const auto& p:request.at("participants")) {
    fields(p,{"id","source","point_to_grid","compensation_cny","export_mwh","nonmarket_export_mwh","import_mwh"});
    require(ids.insert(integer(p.at("id"),0,1000000,"participant id")).second,"duplicate allocation participant");text_source(p.at("source"),"allocation source");
    double factor=boolean(p.at("point_to_grid"),"point_to_grid")?.5:1;
    const double export_mwh=number(p.at("export_mwh"),0,1e15,"export MWh"),nonmarket=number(p.at("nonmarket_export_mwh"),0,1e15,"nonmarket MWh");
    require(nonmarket<=export_mwh,"nonmarket export cannot exceed total export");
    pool+=factor*number(p.at("compensation_cny"),0,1e15,"compensation");
    gen_weight+=factor*(continuous?nonmarket:export_mwh);user_weight+=number(p.at("import_mwh"),0,1e15,"import MWh");return_weight+=factor*export_mwh;
  }
  const double gen_pool=pool*(continuous?share:1),user_pool=pool-gen_pool;
  require(gen_pool==0||gen_weight>0,"generation pool has zero allocation denominator");require(user_pool==0||user_weight>0,"user pool has zero allocation denominator");require(assessment==0||return_weight>0,"assessment pool has zero return denominator");
  Json out={{"rows",Json::array()},{"compensation_pool_cny",pool},{"assessment_pool_cny",assessment}};double charges=0,returns=0;
  for(const auto& p:request.at("participants")) {
    double factor=p.at("point_to_grid").get<bool>()?.5:1;
    double gen=gen_weight?gen_pool*factor*p.at(continuous?"nonmarket_export_mwh":"export_mwh").get<double>()/gen_weight:0;
    double use=user_weight?user_pool*p.at("import_mwh").get<double>()/user_weight:0;
    double returned=return_weight?assessment*factor*p.at("export_mwh").get<double>()/return_weight:0;
    double compensation=factor*p.at("compensation_cny").get<double>();charges+=gen+use;returns+=returned;
    out["rows"].push_back({{"id",p.at("id")},{"compensation_cny",compensation},{"generation_charge_cny",gen},{"user_charge_cny",use},{"assessment_return_cny",returned},{"net_before_own_assessment_cny",compensation-gen-use+returned}});
  }
  out["compensation_balance_residual_cny"]=charges-pool;out["assessment_balance_residual_cny"]=returns-assessment;
  require(std::abs(charges-pool)<=1e-6&&std::abs(returns-assessment)<=1e-6,"allocation numerical balance failed");return out;
}

inline Json settle(const Json& result,const Json& request) {
  fields(request,{"measurements","allocation"});
  require(result.contains("ancillary"),"missing ancillary clearance");
  auto measured=meter(result.at("ancillary"),request.at("measurements"));
  Json out={{"metering",measured},{"allocation",nullptr},{"complete",false},
    {"scope","Daily accrued simulation statement; official monthly meter closure, external assessment standards and source authenticity are not certified"},
    {"regulator_certified",false},{"request",request}};
  if(!measured.at("complete").get<bool>())return out;
  Json allocation=request.at("allocation");
  fields(allocation,{"continuous_spot","generation_share","assessment_pool_cny","participants"});
  if(!result.at("ancillary").at("config").at("research").get<bool>())require(number(allocation.at("generation_share"),0,1,"generation share")==.5,"official generation share is 0.5");
  require(allocation.at("participants").is_array(),"participants must be array");
  std::map<int,double> payments;for(const auto& row:measured.at("rows"))payments[row.at("unit_id")]+=row.at("compensation_cny").get<double>();
  std::set<int> agc_ids;for(const auto& bid:result.at("ancillary").at("hours")[0].at("bids"))agc_ids.insert(bid.at("id").get<int>());
  std::set<int> assigned;
  for(auto& p:allocation["participants"]) {
    fields(p,{"id","source","point_to_grid","unit_ids","export_mwh","nonmarket_export_mwh","import_mwh"});
    require(p.at("unit_ids").is_array(),"unit_ids must be an array");double compensation=0;
    for(const auto& v:p.at("unit_ids")){int id=integer(v,0,1000000,"settlement AGC id");require(agc_ids.count(id)&&assigned.insert(id).second,"unknown or multiply assigned settlement AGC");compensation+=payments[id];}
    p.erase("unit_ids");p["compensation_cny"]=compensation;
  }
  for(const auto& [id,payment]:payments)require(assigned.count(id),"metered AGC missing from financial participants");
  out["allocation"]=allocate(allocation);out["complete"]=true;return out;
}

inline std::chrono::year_month_day calendar_date(const Json& value) {
  require(value.is_string(),"calendar date must be YYYY-MM-DD");const auto s=value.get<std::string>();
  require(s.size()==10&&s[4]=='-'&&s[7]=='-',"calendar date must be YYYY-MM-DD");
  for(size_t i=0;i<s.size();++i)if(i!=4&&i!=7)require(s[i]>='0'&&s[i]<='9',"invalid calendar digit");
  const std::chrono::year_month_day date{std::chrono::year{std::stoi(s.substr(0,4))},std::chrono::month{static_cast<unsigned>(std::stoi(s.substr(5,2)))},std::chrono::day{static_cast<unsigned>(std::stoi(s.substr(8,2)))}};
  require(date.ok()&&int(date.year())>=2000&&int(date.year())<=2200,"invalid supported calendar date");return date;
}
inline std::chrono::sys_days calendar_offset(std::chrono::year_month_day date,int months) {
  const auto target=date+std::chrono::months{months};
  return target.ok()?std::chrono::sys_days{target}:std::chrono::sys_days{target.year()/target.month()/std::chrono::last};
}
inline Json post_statement(const Json& journal,const Json& result,const Json& entry) {
  fields(entry,{"book","delivery_date","posting_date","discovered_date","supersedes_id","reason","source"});
  text_source(entry.at("book"),"book");text_source(entry.at("source"),"accounting source");text_source(entry.at("reason"),"accounting reason");
  const auto delivery=calendar_date(entry.at("delivery_date")),posted=calendar_date(entry.at("posting_date"));
  require(std::chrono::sys_days{posted}>=std::chrono::sys_days{delivery},"cannot post before delivery");
  require(result.contains("statement")&&result.at("statement").value("complete",false)&&result.at("ancillary").value("settlement_eligible",false),"only complete metered statements can enter journal");
  require(result.contains("clearing_id"),"server clearance identity required");
  const Json* previous=nullptr;
  for(const auto& row:journal) {
    require(row.at("clearing_id")!=result.at("clearing_id"),"clearance already posted; duplicate accounting rejected");
    if(row.at("book")==entry.at("book")&&row.at("delivery_date")==entry.at("delivery_date"))previous=&row;
  }
  if(previous) {
    require(entry.at("supersedes_id")==previous->at("entry_id"),"correction must name latest original statement");
    const auto discovered=calendar_date(entry.at("discovered_date"));
    require(std::chrono::sys_days{posted}>=std::chrono::sys_days{calendar_date(previous->at("posting_date"))},"correction cannot precede prior posting");
    require(std::chrono::sys_days{discovered}>=std::chrono::sys_days{delivery}&&std::chrono::sys_days{posted}>=std::chrono::sys_days{discovered},"invalid correction chronology");
    require(std::chrono::sys_days{posted}<=calendar_offset(discovered,1),"correction exceeds one calendar month after discovery");
    require(std::chrono::sys_days{posted}<=calendar_offset(delivery,6),"correction exceeds six-month lookback; external exception is unsupported");
  } else require(entry.at("supersedes_id").is_null()&&entry.at("discovered_date").is_null(),"new statement cannot supersede a missing entry");
  Json record=entry;record["entry_id"]=journal.size()+1;record["clearing_id"]=result.at("clearing_id");record["metering"]=result.at("statement").at("metering");
  record["unit_identity"]=Json::array();
  for(const char* family:{"agc_units","independent_units"})for(const auto& a:result.at("ancillary").at("config").value(family,Json::array())) {
    Json identity={{"id",a.at("id")},{"mode",a.at("mode")}};
    if(a.contains("members")){identity["generators"]=Json::array();for(const auto& m:a.at("members"))identity["generators"].push_back(m.at("generator_id"));std::sort(identity["generators"].begin(),identity["generators"].end());}
    else identity["resource_id"]=a.at("resource_id");record["unit_identity"].push_back(identity);
  }
  std::sort(record["unit_identity"].begin(),record["unit_identity"].end(),[](const Json& a,const Json& b){return a.at("id")<b.at("id");});
  for(const auto& row:journal)if(row.at("book")==entry.at("book"))require(row.at("unit_identity")==record.at("unit_identity"),"book resource identity changed; use a separate simulation book");
  record["events"]=Json::array();
  for(const auto& m:result.at("statement").at("request").at("measurements"))for(const auto& e:m.at("events"))record["events"].push_back(e.at("event_id"));
  // Cross-day replay cannot earn twice. Revisions of the same day retain their
  // event identities in history but replace the active accounting version.
  for(const auto& row:journal)if(row.at("book")==entry.at("book")&&row.at("delivery_date")!=entry.at("delivery_date"))
    for(const auto& id:record.at("events"))for(const auto& old:row.at("events"))require(id!=old,"AGC event replayed on another delivery day");
  record["raw_compensation_delta_cny"]=record.at("metering").at("total_compensation_cny").get<double>()-(previous?previous->at("metering").at("total_compensation_cny").get<double>():0);
  auto out=journal;out.push_back(record);return out;
}

// Arts.46-50: sum latest daily metering first, then apply monthly energy
// denominators once. Daily allocations are deliberately not summed.
inline Json settle_month(const Json& journal,const Json& request) {
  fields(request,{"book","month","allocation"});text_source(request.at("book"),"book");
  require(request.at("month").is_string(),"month must be YYYY-MM");const auto month=request.at("month").get<std::string>();
  require(month.size()==7,"month must be YYYY-MM");const auto first=calendar_date(month+"-01");
  const unsigned days=unsigned(std::chrono::year_month_day{first.year()/first.month()/std::chrono::last}.day());
  std::map<std::string,const Json*> latest;
  for(const auto& row:journal)if(row.at("book")==request.at("book")&&row.at("delivery_date").get<std::string>().substr(0,7)==month)latest[row.at("delivery_date")]=&row;
  Json out={{"book",request.at("book")},{"month",month},{"complete",false},{"missing_days",Json::array()},{"allocation",nullptr},{"entry_ids",Json::array()},
    {"scope","Restated simulation month using latest posted daily measurements; remittance and correction payment in the next legal settlement are external"}};
  for(unsigned d=1;d<=days;++d){const auto date=month+"-"+(d<10?"0":"")+std::to_string(d);if(!latest.count(date))out["missing_days"].push_back(date);}
  if(!out.at("missing_days").empty())return out;
  std::map<int,double> amounts;std::set<int> known;
  for(const auto& [date,row]:latest){out["entry_ids"].push_back(row->at("entry_id"));for(const auto& u:row->at("unit_identity"))known.insert(u.at("id").get<int>());for(const auto& m:row->at("metering").at("rows"))amounts[m.at("unit_id")]+=m.at("compensation_cny").get<double>();}
  Json allocation=request.at("allocation");fields(allocation,{"continuous_spot","generation_share","assessment_pool_cny","participants"});
  require(number(allocation.at("generation_share"),0,1,"generation share")==.5,"monthly rule allocation requires F=0.5");
  require(allocation.at("participants").is_array(),"monthly participants must be array");std::set<int> mapped;
  for(auto& p:allocation["participants"]){fields(p,{"id","source","point_to_grid","unit_ids","export_mwh","nonmarket_export_mwh","import_mwh"});require(p.at("unit_ids").is_array(),"unit_ids must be array");double sum=0;
    for(const auto& id:p.at("unit_ids")){int a=integer(id,0,1000000,"AGC id");require(known.count(a)&&mapped.insert(a).second,"unknown or duplicate monthly AGC mapping");sum+=amounts[a];}p.erase("unit_ids");p["compensation_cny"]=sum;}
  for(const auto& [id,amount]:amounts)require(mapped.count(id),"monthly financial owner missing");
  out["allocation"]=allocate(allocation);out["complete"]=true;return out;
}
} // namespace hacdcpf::market::yunnan
