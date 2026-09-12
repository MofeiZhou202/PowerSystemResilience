#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cstdlib>
#include <numeric>
#include <random>
#include <set>
#include <future>
#include "hacdcpf/market/southern_market.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "../src/market/southern_solver_status.hpp"
#include "../src/market/southern_price_consistency.hpp"
#include "../src/market/southern_bound_certificate.hpp"
#include "../src/market/yunnan_rules_workflow.hpp"
#include "../src/market/southern_real_time.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/engine.hpp"

using nlohmann::json;
using namespace hacdcpf::market;
using Catch::Approx;

TEST_CASE("Southern box lower bound obeys weak duality for arbitrary multipliers", "[southern_market][bound_certificate]") {
  hacdcpf::engine::LPModel lp;
  lp.c=Eigen::Vector2d(-2,3);lp.A.resize(1,2);lp.Aeq.resize(1,2);
  lp.A.insert(0,0)=1;lp.A.insert(0,1)=1;lp.Aeq.insert(0,0)=1;lp.Aeq.insert(0,1)=-1;
  lp.b=Eigen::VectorXd::Constant(1,3);lp.beq=Eigen::VectorXd::Zero(1);
  const Eigen::Vector2d lower(-2,-2),upper(2,2);
  // Feasible x=(t,t), -2<=t<=1.5; min -2x+3y=-2 at t=-2.
  for(double y:{-1e5,-2.,0.,3.,1e5})for(double lambda:{-1e5,-3.,0.,2.,1e5}) {
    const auto bound=detail::box_dual_lower_bound(lp,Eigen::Vector2d(y,lambda),lower,upper);
    REQUIRE(bound.has_value());REQUIRE(*bound<=-2.0);
  }
  auto bound=detail::box_dual_lower_bound(lp,Eigen::Vector2d(0,-2),lower,upper);
  REQUIRE(*bound==Approx(-2).margin(1e-12));
  lp.b.resize(0);REQUIRE_FALSE(detail::box_dual_lower_bound(lp,Eigen::Vector2d(0,0),lower,upper));
  lp.b=Eigen::VectorXd::Constant(1,3);lp.A.coeffRef(0,0)=std::numeric_limits<double>::quiet_NaN();
  REQUIRE_FALSE(detail::box_dual_lower_bound(lp,Eigen::Vector2d(0,0),lower,upper));
  lp.A.coeffRef(0,0)=1;
  REQUIRE_FALSE(detail::box_dual_lower_bound(lp,Eigen::Vector2d(0,0),upper,lower));
  REQUIRE_FALSE(detail::box_dual_lower_bound(lp,Eigen::Vector2d(0,0),lower,Eigen::Vector2d(2,INFINITY)));
}

TEST_CASE("Southern pricing rejects inconsistent or unproven full dual vectors", "[southern_market][price_consistency]") {
  hacdcpf::engine::SolveResult first;
  first.stats.success=true;first.stats.status="optimal";first.stats.mip_gap=0;
  first.constraint_duals=Eigen::VectorXd::Constant(3,200);
  auto repeat=first;
  const auto check=[&] { return detail::check_price_duals(first,repeat,3,0,0,0); };
  REQUIRE(check()["passed"]==true);
  repeat.constraint_duals[2]=std::nextafter(200.0,201.0);
  REQUIRE(check()["status"]=="dual_mismatch");REQUIRE(check()["passed"]==false);
  repeat=first;repeat.constraint_duals[1]=std::numeric_limits<double>::quiet_NaN();
  REQUIRE(check()["status"]=="missing_or_nonfinite_duals");
  repeat=first;repeat.constraint_duals.resize(2);REQUIRE(check()["passed"]==false);
  repeat=first;repeat.stats.status="time_limit";REQUIRE(check()["passed"]==false);
  repeat=first;repeat.stats.success=false;REQUIRE(check()["passed"]==false);
  repeat=first;
  REQUIRE(detail::check_price_duals(first,repeat,3,0,2e-6,0)["passed"]==false);
  REQUIRE(detail::check_price_duals(first,repeat,3,0,0,2e-6)["passed"]==false);
}

TEST_CASE("Southern certified repair audits original models and rejects unsupported certificates", "[southern_market][certified_repair]") {
  if(!southern_market_solver_capabilities()[1]["available"].get<bool>())SKIP("Gurobi unavailable");
  auto boundary=make_southern_market_example();
  boundary["execution"]["solver"]="gurobi";boundary["execution"]["mip_gap"]=.01;
  boundary["execution"]["large_mip_strategy"]="certified_repair";
  boundary["execution"]["time_limit_sec"]=30;
  const auto result=run_southern_day_ahead_market(boundary);
  INFO(result.at("scuc").dump().substr(0,3000));
  REQUIRE(result["schedule_feasible"]==true);REQUIRE(result["prices_valid"]==true);
  REQUIRE(result["scuc"]["gap_certificate"]["accepted"]==true);
  const auto& certificate=result["scuc"]["gap_certificate"];
  boundary["execution"]["large_mip_strategy"]="reference";boundary["execution"]["mip_gap"]=0;
  const auto oracle=run_southern_day_ahead_market(boundary);
  REQUIRE(oracle["schedule_feasible"]==true);
  REQUIRE(certificate["bound_cny"].get<double>()<=oracle["scuc"]["objective"].get<double>()+1e-6);
  REQUIRE(certificate["gap"].get<double>()<=.01);
  for(const auto* stage:{"scuc","sced","lmp"})REQUIRE(result[stage]["max_residual"].get<double>()<=1e-6);
  auto derived=make_southern_market_example();
  derived["execution"]["solver"]="gurobi";derived["execution"]["mip_gap"]=.01;
  derived["execution"]["large_mip_strategy"]="certified_repair";derived["execution"]["assembly_mode"]="verify";
  for(auto& g:derived["generators"]) {
    g["minimum_cost_per_hour"]=0;g["startup_cost"]={0,0,0};
    g["regulation_up_mw"]=std::vector<double>(98,0);g["regulation_down_mw"]=std::vector<double>(98,0);
  }
  const auto derived_result=run_southern_day_ahead_market(derived);
  REQUIRE(derived_result["schedule_feasible"]==true);REQUIRE(derived_result["prices_valid"]==true);
  REQUIRE(derived_result["sced"]["assembly_template"]["derived_from"]=="scuc");
  REQUIRE(derived_result["sced"]["assembly_template"]["matrix_comparison"]=="exact_match");
  auto unsupported=make_southern_market_demo();
  unsupported["execution"]["solver"]="gurobi";unsupported["execution"]["mip_gap"]=.01;
  unsupported["execution"]["large_mip_strategy"]="certified_repair";
  const auto storage_case=run_southern_day_ahead_market(unsupported);
  REQUIRE(storage_case["schedule_feasible"]==true);REQUIRE(storage_case["prices_valid"]==true);
  REQUIRE(storage_case["scuc"]["gap_certificate"]["storage_candidate_assets"]==unsupported["storage"].size());
  for(const auto& authored:unsupported["storage"]) {
    const auto& rows=storage_case["sced"]["storage"];
    const auto found=std::find_if(rows.begin(),rows.end(),[&](const auto& s){return s["id"]==authored["id"];});
    REQUIRE(found!=rows.end());
    double energy=authored["initial_mwh"].get<double>();
    const double eta=std::sqrt(authored["roundtrip_efficiency"].get<double>());
    for(int t=0;t<98;++t) {
      energy-=unsupported["periods"][t]["duration_hr"].get<double>()*
        ((*found)["discharge_mw"][t].get<double>()/eta+(*found)["charge_mw"][t].get<double>()*eta);
      REQUIRE((*found)["energy_mwh"][t].get<double>()==Approx(energy).margin(1e-6));
      if(t==95)REQUIRE(energy==Approx(authored["terminal_mwh"].get<double>()).margin(1e-6));
    }
  }
  unsupported["branches"][0]["shift_deg"]=10;
  const auto fallback=run_southern_day_ahead_market(unsupported);
  REQUIRE(fallback["schedule_feasible"]==true);REQUIRE(fallback["prices_valid"]==true);
  REQUIRE(fallback["scuc"]["gap_certificate"]["accepted"]==false);
  REQUIRE(fallback["scuc"]["gap_certificate"]["fallback"]=="original_milp");
  boundary["execution"]["large_mip_strategy"]="certified_repair";
  REQUIRE_THROWS(validate_southern_market(boundary));
}

static void check_lmp_derivation(const char* solver) {
  for(const std::string storage_policy:{"power_neighborhood_only","retain_energy_constraints"})
    for(const std::string priority_policy:{"omit_unlisted","retain_sced"}) {
      auto boundary=make_southern_market_demo();
      boundary["execution"]["solver"]=solver;
      boundary["execution"]["ac_security"]="schedule_only";
      boundary["execution"]["assembly_mode"]="verify";
      boundary["execution"]["lmp_storage_policy"]=storage_policy;
      boundary["execution"]["lmp_renewable_priority"]=priority_policy;
      boundary["execution"]["priority_policy"]="penalized_shortfall";
      const auto result=run_southern_day_ahead_market(boundary);
      INFO(storage_policy+" / "+priority_policy);
      REQUIRE(result.at("schedule_feasible")==true);
      REQUIRE(result.at("prices_valid")==true);
      REQUIRE(result.at("lmp").at("assembly_template").at("derived_from")=="sced");
      REQUIRE(result.at("lmp").at("assembly_template").at("matrix_comparison")=="exact_match");
      REQUIRE(result.at("lmp").at("price_consistency").at("max_dual_difference")==0);
      for(const char* stage:{"scuc","sced","lmp"})REQUIRE(result.at(stage).at("max_residual").get<double>()<=1e-6);
      const auto& storage=result.at("lmp").at("storage").at(0).at("energy_mwh");
      REQUIRE(storage.at(0).is_null()==(storage_policy=="power_neighborhood_only"));
    }
  for(const std::string mode:{"reference","verify"}) {
    auto boundary=make_southern_market_demo();
    boundary["execution"]["solver"]=solver;
    boundary["execution"]["ac_security"]="schedule_only";
    boundary["execution"]["assembly_mode"]=mode;
    boundary["execution"]["row_presolve"]="enabled";
    const auto result=run_southern_day_ahead_market(boundary);
    REQUIRE(result.at("prices_valid")==true);
    REQUIRE_FALSE(result.at("lmp").at("assembly_template").contains("derived_from"));
  }
}

TEST_CASE("Southern pricing derives the original ordered LP from SCED", "[southern_market][lmp_reuse]") {
  check_lmp_derivation("highs");
}

TEST_CASE("Southern Gurobi pricing derives the original ordered LP from SCED", "[southern_market][lmp_reuse_gurobi]") {
  if(!southern_market_solver_capabilities()[1]["available"].get<bool>())SKIP("Gurobi unavailable");
  check_lmp_derivation("gurobi");
}

TEST_CASE("Southern SCED lower-bound reuse requires exact LP containment", "[southern_market][bound_certificate]") {
  hacdcpf::engine::LPModel original;
  original.c=Eigen::Vector2d(1,2);original.b=Eigen::VectorXd::Constant(1,3);
  original.beq=Eigen::VectorXd::Constant(1,0);
  original.A.resize(1,2);original.A.insert(0,0)=1;original.A.insert(0,1)=2;original.A.makeCompressed();
  original.Aeq.resize(1,2);original.Aeq.insert(0,0)=1;original.Aeq.insert(0,1)=-1;original.Aeq.makeCompressed();
  original.vars={{hacdcpf::engine::VarType::Continuous,0,3,"x"},{hacdcpf::engine::VarType::Continuous,0,3,"y"}};
  auto next=original;
  REQUIRE(detail::same_lp_with_tighter_box(original,next));
  next.vars[0].lb=next.vars[0].ub=1;
  REQUIRE(detail::same_lp_with_tighter_box(original,next));
  next.vars[1].ub=4;REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
  next=original;next.c[0]=std::nextafter(1.,2.);REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
  next=original;next.b[0]=std::nextafter(3.,4.);REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
  next=original;next.A.coeffRef(0,0)=2;REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
  next=original;std::swap(next.vars[0].name,next.vars[1].name);REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
  next=original;next.row_lhs=Eigen::VectorXd::Zero(1);REQUIRE_FALSE(detail::same_lp_with_tighter_box(original,next));
}

TEST_CASE("Southern pricing repeats independently of requested dispatch algorithms", "[southern_market][price_consistency]") {
  auto boundary=make_southern_market_example();
  for(const std::string backend:{"highs","gurobi"}) {
    if(backend=="gurobi" && !southern_market_solver_capabilities()[1]["available"].get<bool>())continue;
    boundary["execution"]["solver"]=backend;
    json expected;
    for(const std::string method:{"auto","barrier","dual_simplex"}) {
      boundary["execution"]["gurobi_method"]=backend=="gurobi"?method:"auto";
      boundary["execution"]["threads"]=backend=="gurobi"?(method=="auto"?1:2):0;
      boundary["execution"]["assembly_mode"]=method=="auto"?"cached":method=="barrier"?"reference":"verify";
      const auto result=run_southern_day_ahead_market(boundary);
      INFO(result.at("lmp").dump().substr(0,2000));
      REQUIRE(result["prices_valid"]==true);
      const auto& lmp=result.at("lmp");
      REQUIRE(lmp["price_consistency"]["passed"]==true);
      REQUIRE(lmp["price_consistency"]["max_dual_difference"]==0);
      REQUIRE(lmp["price_consistency"]["threads"]==1);
      REQUIRE(lmp["lp_algorithm"]=="dual_simplex");
      const auto prices=lmp.at("buses")[0].at("lmp_per_mwh");
      if(expected.is_null())expected=prices;else REQUIRE(prices==expected);
    }
  }
}

TEST_CASE("Southern assembly templates exactly match original coefficients after boundary changes", "[southern_market][assembly_cache]") {
  const auto base=make_southern_market_demo();
  for(int change=0;change<14;++change) {
    auto b=base;b["execution"]["assembly_mode"]="verify";
    if(change==1)b["generators"][0]["segments"][0]["price_per_mwh"]=123.45;
    if(change==2)for(auto& h:b["reservoirs"])for(auto& v:h["inflow_m3_s"])v=v.get<double>()*1.1;
    if(change==3)for(auto& s:b["storage"]){s["initial_mwh"]=s["initial_mwh"].get<double>()*.9;s["roundtrip_efficiency"]=.81;}
    if(change==4)b["generators"][0]["available"][5]=0;
    if(change==5)b["branches"][0]["available"][8]=0;
    if(change==6){auto g=b["generators"][0];g["id"]=100001;b["generators"].push_back(g);}
    if(change==7)b["generators"][0]["segments"].push_back(b["generators"][0]["segments"][0]);
    if(change==8)for(auto& s:b["storage"]){s["discharge_min_mw"]=1;s["charge_min_mw"]=1;}
    if(change==9)b["execution"]["row_presolve"]="enabled";
    if(change==10)for(auto& h:b["reservoirs"]){h["water_m3_mwh"]=h["water_m3_mwh"].get<double>()*1.2;h["initial_level_m"]=h["initial_level_m"].get<double>()+.1;}
    if(change==11)for(auto& g:b["generators"])g["primary_fraction"]=std::vector<double>(98,0);
    if(change==12)b["storage"].clear();
    if(change==13) {auto& line=b["branches"][0];std::swap(line["from_bus"],line["to_bus"]);}
    INFO(change);
    const auto first=inspect_southern_market_model(b);
    REQUIRE(first["assembly_template"]["matrix_comparison"]=="exact_match");
    const auto repeated=inspect_southern_market_model(b);
    REQUIRE(repeated["assembly_template"]["matrix_comparison"]=="exact_match");
    REQUIRE(repeated["assembly_template"]["reused_matrices"]==2);
  }
  auto foreign=make_southern_market_example();foreign["execution"]["assembly_mode"]="verify";
  REQUIRE(inspect_southern_market_model(foreign)["assembly_template"]["matrix_comparison"]=="exact_match");
  auto original=base;original["execution"]["assembly_mode"]="verify";
  REQUIRE(inspect_southern_market_model(original)["assembly_template"]["layout_reused"]==false);
  auto a=std::async(std::launch::async,[&] {return inspect_southern_market_model(original);});
  auto b=std::async(std::launch::async,[&] {return inspect_southern_market_model(foreign);});
  REQUIRE(a.get()["assembly_template"]["matrix_comparison"]=="exact_match");
  REQUIRE(b.get()["assembly_template"]["matrix_comparison"]=="exact_match");
}

TEST_CASE("Southern assembly templates preserve rolling objectives residuals and water SOC carry", "[southern_market][assembly_cache]") {
  auto b=make_southern_market_demo();
  b["execution"]["ac_security"]="schedule_only";
  b["execution"]["mip_gap"]=0;b["execution"]["assembly_mode"]="reference";
  const json config={{"horizon","week"},{"start_date","2026-09-08"},{"explain",false},{"penalty_per_mwh",100000},{"days",json::array()}};
  auto reference=make_market_operation(b,config);
  b["execution"]["assembly_mode"]="verify";auto cached=make_market_operation(b,config);
  REQUIRE_FALSE(b["storage"].empty());REQUIRE_FALSE(b["reservoirs"].empty());
  for(int d=0;d<7;++d) {
    reference=step_market_operation(reference);cached=step_market_operation(cached);
    REQUIRE(reference["completed_days"]==d+1);REQUIRE(cached["completed_days"]==d+1);
    const auto& x=reference["days"][d];const auto& y=cached["days"][d];
    for(const auto* field:{"resources","periods","nodes","lookahead","state_start","state_end"})REQUIRE(x[field]==y[field]);
    for(const auto* stage:{"scuc","sced","lmp"}) {
      REQUIRE(y["stages"][stage]["assembly_template"]["matrix_comparison"]=="exact_match");
      REQUIRE(x["stages"][stage]["objective"]==y["stages"][stage]["objective"]);
      REQUIRE(x["stages"][stage]["max_residual"]==y["stages"][stage]["max_residual"]);
      REQUIRE(y["stages"][stage]["max_residual"].get<double>()<=1e-6);
    }
    REQUIRE(reference["carry"]==cached["carry"]);
    REQUIRE(y["stages"]["lmp"]["assembly_template"]["derived_from"]=="sced");
    REQUIRE(y["stages"]["lmp"]["price_consistency"]["passed"]==true);
    REQUIRE(y["stages"]["lmp"]["price_consistency"]["max_dual_difference"]==0);
  }
}

TEST_CASE("Southern realtime independent time grids and executed rolling state have hand oracles", "[southern_market][realtime]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  auto c=southern_realtime_defaults(b);c["steps"]=4;
  auto job=make_southern_realtime(b,c);
  for(int step=0;step<4;++step){job=step_southern_realtime(job);const auto& r=job["runs"].back()["dispatch"];
    INFO(r.value("error",std::string("")));INFO(r.value("pricing_error",std::string("")));
    REQUIRE(r["schedule_feasible"]==true);REQUIRE(r["physical_schedule_feasible"]==true);REQUIRE(r["prices_valid"]==true);
    REQUIRE(r["sced"]["generators"][0]["power_mw"].size()==24);REQUIRE(r["lmp"]["buses"][0]["lmp_per_mwh"].size()==8);
    REQUIRE(r["sced"]["window_generation_mwh"].get<double>()==Approx(200).margin(1e-6));
    REQUIRE(r["sced"]["window_energy_bid_cost_cny"].get<double>()==Approx(40000).margin(1e-5));
    REQUIRE(r["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>()==Approx(200).margin(1e-5));
    REQUIRE(job["runs"].back()["executed_points"]==3);
    REQUIRE(job["runs"].back()["outlook"]["reference_only"]==true);
    if(step<3)REQUIRE(job["hourly_prices"][0]["price_per_mwh"].is_null());
    if(step)REQUIRE(r["boundary_snapshot"]["generators"][0]["initial_state_minutes"].get<double>()==Approx(b["generators"][0]["initial_state_minutes"].get<double>()+15*step));
  }
  REQUIRE(job["status"]=="complete");REQUIRE(job["hourly_prices"][0]["price_per_mwh"].get<double>()==Approx(200).margin(1e-5));REQUIRE_THROWS(step_southern_realtime(job));
}

TEST_CASE("Southern realtime ramp and accident reserve failure cannot advance a rolling job", "[southern_market][realtime]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  auto c=southern_realtime_defaults(b);auto& g=c["boundary"]["generators"][0];g["initial_power_mw"]=90;g["ramp_up_mw_min"]=1;
  auto job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="failed");REQUIRE(job["completed_steps"]==0);
  c["boundary"]["execution"]["balance_policy"]="diagnostic";
  job=step_southern_realtime(make_southern_realtime(b,c));const auto& r=job["runs"][0]["dispatch"];
  REQUIRE(r["schedule_feasible"]==true);REQUIRE(r["physical_schedule_feasible"]==false);
  REQUIRE(r["sced"]["generators"][0]["power_mw"][0].get<double>()==Approx(95).margin(1e-5));
  REQUIRE(r["sced"]["buses"][0]["deficit_mw"][0].get<double>()==Approx(5).margin(1e-5));
  c=southern_realtime_defaults(b);c["boundary"]["generators"][0]["pmax_mw"]=std::vector<double>(72,110);c["boundary"]["generators"][0]["ramp_up_mw_min"]=1;
  c["accident_reserve"][0]["requirement_mw"]=std::vector<double>(72,11);
  job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="failed");
  c["reserve_policy"]="penalized";job=step_southern_realtime(make_southern_realtime(b,c));
  REQUIRE(job["runs"][0]["dispatch"]["sced"]["accident_reserve"][0]["shortage_mw"][0].get<double>()==Approx(1).margin(1e-5));
}

TEST_CASE("Southern realtime sourced forecasts and sealed declarations reject silent changes", "[southern_market][realtime]") {
  auto b=make_southern_market_example();auto wind=b["generators"][0];wind["id"]=2;wind["kind"]="wind";wind["forecast_mw"]=std::vector<double>(98,40);b["generators"].push_back(wind);
  auto c=southern_realtime_defaults(b);auto& f=c["renewable_forecasts"][0];f["submitted"][0]=nullptr;f["dispatch_forecast"][0]=nullptr;
  auto job=make_southern_realtime(b,c);REQUIRE(job["prepared"]["forecast_resolution"][0]["origin"]=="day_ahead_fallback");
  f["dispatch_forecast"][0]=30;job=make_southern_realtime(b,c);REQUIRE(job["prepared"]["forecast_resolution"][0]["mw"]==30);
  f["previous_complete_files"]={std::vector<double>(72,20)};job=make_southern_realtime(b,c);REQUIRE(job["prepared"]["forecast_resolution"][0]["mw"]==20);
  f["submitted"][0]=10;job=make_southern_realtime(b,c);REQUIRE(job["prepared"]["forecast_resolution"][0]["mw"]==10);
  auto bad=c;bad["steps"]=9;REQUIRE_THROWS(make_southern_realtime(b,bad));bad=c;bad["boundary"]["generators"][0]["segments"][0]["price"]=999;REQUIRE_THROWS(make_southern_realtime(b,bad));
  bad=c;bad["boundary"]["generators"][0]["available"]=std::vector<int>(98,1);REQUIRE_THROWS(make_southern_realtime(b,bad));
  bad=c;bad["boundary"]["generators"][0]["startup_curves_mw"][0]={10};REQUIRE_THROWS(make_southern_realtime(b,bad));
}

namespace {
std::pair<json,json> ancillary_three_units() {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  b["generators"][0]["must_on"]=std::vector<int>(98,1);
  for(int id:{2,3}){auto g=b["generators"][0];g["id"]=id;b["generators"].push_back(g);}
  auto c=yunnan_ancillary_defaults(b);c["cmin_mw"]=20;c["load_ratio"]=0;c["renewable_ratio"]=0;
  for(int i=0;i<3;++i)c["agc_units"][i]["price_per_mw"]=std::vector<double>(24,4+i);
  return {b,c};
}
json ancillary_telemetry(const json& a) {
  json rows=json::array();
  for(const auto& h:a.at("hours"))for(const auto& b:h.at("bids"))if(b.at("award_mw").get<double>()>0)
    rows.push_back({{"unit_id",b.at("id")},{"hour",h.at("hour")},{"source","hand oracle: explicit zero AGC commands"},
      {"test_period",false},{"own_unavailability_seconds",json::array()},{"events",json::array()}});
  return rows;
}
json ancillary_event(const char* id,double mileage) {
  return {{"event_id",id},{"start_second",0},{"end_second",60},{"start_mw",50},{"end_mw",50+mileage},{"autor",true},
    {"metrics",{{"rate_fraction_per_min",.015},{"delay_seconds",0},{"error_fraction",0},{"fleet_standard_fraction_per_min",.015},{"standard_delay_seconds",60},{"allowed_error_fraction",.01}}}};
}
}

TEST_CASE("Yunnan sealed declarations and safety merit order have independent logical oracles", "[southern_market][ancillary][rules]") {
  auto [b,c]=ancillary_three_units();std::map<std::string,double> fixed;
  for(int id:{1,2,3})for(int t=0;t<98;++t){fixed["stable/"+std::to_string(id)+"/"+std::to_string(t)]=1;fixed["primary/"+std::to_string(id)+"/"+std::to_string(t)]=0;}
  auto submission=[](const char* id,double minute,double price,double cap){return json{{"submission_id",id},{"unit_id",0},{"hour",0},{"minute",minute},{"price_per_mw",price},{"capacity_mw",cap},{"source","hand bid log"}};};
  c["workflow"]["bid_submissions"]={submission("valid",-800,3.5,9),submission("invalid",-720,3.6,11)};
  validate_yunnan_ancillary(b,c);auto sealed=yunnan::seal_bids(c,b);
  REQUIRE(sealed["agc_units"][0]["price_per_mw"][0]==3.5);REQUIRE(sealed["agc_units"][0]["capacity_mw"][0]==9);
  auto bad=c;bad["workflow"]["bid_submissions"][0]["minute"]=-719;REQUIRE_THROWS(validate_yunnan_ancillary(b,bad));
  bad=c;bad["workflow"]["bid_submissions"].push_back(bad["workflow"]["bid_submissions"][0]);REQUIRE_THROWS(validate_yunnan_ancillary(b,bad));
  c["workflow"]["bid_submissions"]=json::array();
  c["workflow"]["stage"]="intraday";
  c["workflow"]["safety_reviews"]={{{"unit_id",0},{"hour",0},{"up_mw",0},{"down_mw",0},{"agc_available",false},{"reason","authored AGC outage"},{"source","hand safety review"}}};
  auto report=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(report,c["workflow"]);
  const auto& h=report["hours"][0];REQUIRE(h["awarded_mw"]==20);REQUIRE(h["clearing_price_per_mw"]==6);
  REQUIRE(h["bids"][0]["award_mw"]==0);REQUIRE(h["bids"][1]["award_mw"]==10);REQUIRE(h["bids"][2]["award_mw"]==10);
  REQUIRE(report["adjustment_log"][0]["action"]=="safety_remove_or_reduce");REQUIRE(report["adjustment_log"][1]["action"]=="backfill");
  auto permutation=c;std::reverse(permutation["agc_units"].begin(),permutation["agc_units"].end());
  auto perm=yunnan::prearrange(b,permutation,fixed);yunnan::adjust_awards(perm,permutation["workflow"]);REQUIRE(perm["hours"]==report["hours"]);
  c["agc_units"][2]["capacity_mw"][0]=5;
  auto uplift=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(uplift,c["workflow"]);REQUIRE(uplift["hours"][0]["bids"][2]["award_mw"]==10);
  c["workflow"]["allow_uplift"]=false;
  auto shortage=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(shortage,c["workflow"]);REQUIRE(shortage["hours"][0]["shortage_mw"]==5);REQUIRE(shortage["hours"][0]["clearing_price_per_mw"].is_null());
  c["workflow"]["allow_uplift"]=true;c["agc_units"][2]["qualified"]=false;
  shortage=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(shortage,c["workflow"]);REQUIRE(shortage["hours"][0]["shortage_mw"]==10);
  c["workflow"]["clearing_minutes"][0]=-29;REQUIRE_THROWS(validate_yunnan_ancillary(b,c));
}

TEST_CASE("Yunnan intraday reuses commitment and safety reviewed dispatch gates metered money", "[southern_market][ancillary][rules]") {
  auto [b,c]=ancillary_three_units();const auto da=run_yunnan_ancillary_market(b,c);REQUIRE(da["schedule_feasible"]==true);
  c["workflow"]["stage"]="intraday";
  for(int h=0;h<24;++h)c["workflow"]["safety_reviews"].push_back({{"unit_id",0},{"hour",h},{"up_mw",0},{"down_mw",0},{"agc_available",false},{"reason","AGC outage"},{"source","hand review"}});
  const auto rt=run_yunnan_ancillary_intraday(b,c,da);INFO(rt.value("error",std::string("")));REQUIRE(rt["schedule_feasible"]==true);
  REQUIRE(rt["scuc"]["reused_day_ahead"]==true);REQUIRE(rt["commitment_solution"]==da["commitment_solution"]);
  REQUIRE(rt["ancillary"]["settlement_eligible"]==true);
  for(const auto& g:rt["sced"]["generators"])for(int t=0;t<96;++t)REQUIRE(g["secondary_up_mw"][t].get<double>()==Approx(g["id"]==1?0:10).margin(1e-6));
  auto changed=c;changed["agc_units"][0]["price_per_mw"][0]=7;REQUIRE_THROWS(run_yunnan_ancillary_intraday(b,changed,da));
  REQUIRE_THROWS(run_yunnan_ancillary_market(b,c));
  auto measurements=ancillary_telemetry(rt["ancillary"]);
  measurements[0]["events"]={ancillary_event("command-A",10)};measurements[1]["events"]={ancillary_event("command-B",20)};
  auto money=yunnan::meter(rt["ancillary"],measurements);
  REQUIRE(money["complete"]==true);REQUIRE(money["total_compensation_cny"].get<double>()==Approx(180).margin(1e-6));
  measurements[0]["own_unavailability_seconds"]={{0,200},{100,300}};
  REQUIRE(yunnan::meter(rt["ancillary"],measurements)["total_compensation_cny"]==180);
  measurements[0]["own_unavailability_seconds"]={{0,301}};
  REQUIRE(yunnan::meter(rt["ancillary"],measurements)["total_compensation_cny"]==120);
  measurements[0]["own_unavailability_seconds"]=json::array();measurements[1]["test_period"]=true;
  REQUIRE(yunnan::meter(rt["ancillary"],measurements)["total_compensation_cny"]==60);
  measurements[1]["test_period"]=false;measurements[1]["events"][0]["autor"]=false;
  REQUIRE(yunnan::meter(rt["ancillary"],measurements)["total_compensation_cny"]==60);
  measurements[1]["events"][0]["autor"]=true;
  auto invalid=measurements;invalid[1]["events"][0]["event_id"]="command-A";REQUIRE_THROWS(yunnan::meter(rt["ancillary"],invalid));
  invalid=measurements;invalid[0]["events"][0]["start_second"]=nullptr;REQUIRE_THROWS(yunnan::meter(rt["ancillary"],invalid));
  invalid=measurements;invalid.erase(2);REQUIRE(yunnan::meter(rt["ancillary"],invalid)["complete"]==false);
  REQUIRE_THROWS(yunnan::meter(da["ancillary"],measurements));
  auto disabled=rt["ancillary"];disabled["coupled_schedule_feasible"]=false;REQUIRE_THROWS(yunnan::meter(disabled,measurements));
  json request={{"measurements",measurements},{"allocation",{{"continuous_spot",true},{"generation_share",.5},{"assessment_pool_cny",0},{"participants",{
    {{"id",1},{"source","metered export"},{"point_to_grid",false},{"unit_ids",{0,1,2}},{"export_mwh",100},{"nonmarket_export_mwh",100},{"import_mwh",0}},
    {{"id",2},{"source","metered import"},{"point_to_grid",false},{"unit_ids",json::array()},{"export_mwh",0},{"nonmarket_export_mwh",0},{"import_mwh",100}}}}}}};
  const auto settled=settle_yunnan_ancillary(rt,request);REQUIRE(settled["complete"]==true);
  REQUIRE(settled["allocation"]["rows"][0]["generation_charge_cny"]==90);REQUIRE(settled["allocation"]["rows"][1]["user_charge_cny"]==90);
  request["allocation"]["participants"][0]["compensation_cny"]=100000;REQUIRE_THROWS(settle_yunnan_ancillary(rt,request));
}

TEST_CASE("Yunnan suspension price real-time capacity and account conservation", "[southern_market][ancillary][rules]") {
  auto [b,c]=ancillary_three_units();std::map<std::string,double> fixed;
  for(int id:{1,2,3})for(int t=0;t<98;++t){fixed["stable/"+std::to_string(id)+"/"+std::to_string(t)]=1;fixed["primary/"+std::to_string(id)+"/"+std::to_string(t)]=0;}
  c["workflow"]["stage"]="intraday";c["workflow"]["suspensions"]={{{"hour",0},{"previous_trading_day_price_per_mw",7.2},{"source","previous trading day hour zero"}}};
  auto event=[](int id,double cap){return json{{"event_id","adjust-"+std::to_string(id)},{"unit_id",id},{"hour",0},{"second",100},{"capacity_mw",cap},{"reason","safety dispatch"},{"source","operator instruction"}};};
  c["workflow"]["realtime_adjustments"]={event(0,0),event(2,10)};validate_yunnan_ancillary(b,c);
  auto r=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(r,c["workflow"]);
  REQUIRE(r["hours"][0]["clearing_price_per_mw"]==7.2);REQUIRE(r["hours"][1]["clearing_price_per_mw"]==5);
  REQUIRE(r["realtime_capacity_sufficient"]==true);REQUIRE(r["realtime_dispatch"][0]["price_per_mw"]==7.2);
  REQUIRE(yunnan::required_capacity(r,0,0)==10);REQUIRE(yunnan::required_capacity(r,2,0)==10);
  c["workflow"]["realtime_adjustments"].erase(1);r=yunnan::prearrange(b,c,fixed);yunnan::adjust_awards(r,c["workflow"]);REQUIRE(r["realtime_capacity_sufficient"]==false);
  c["workflow"]["realtime_adjustments"][0]["capacity_mw"]=10000;r=yunnan::prearrange(b,c,fixed);REQUIRE_THROWS(yunnan::adjust_awards(r,c["workflow"]));
  auto metrics=ancillary_event("metric",0)["metrics"];metrics["rate_fraction_per_min"]=.15;
  REQUIRE(yunnan::performance_metrics(metrics)["m"]==2);metrics["delay_seconds"]=120;REQUIRE(yunnan::performance_metrics(metrics)["m2"]==-1);
  json allocation={{"continuous_spot",true},{"generation_share",.5},{"assessment_pool_cny",90},{"participants",{
    {{"id",1},{"source","point export"},{"point_to_grid",true},{"compensation_cny",180},{"export_mwh",100},{"nonmarket_export_mwh",100},{"import_mwh",0}},
    {{"id",2},{"source","local export"},{"point_to_grid",false},{"compensation_cny",0},{"export_mwh",100},{"nonmarket_export_mwh",100},{"import_mwh",100}}}}};
  auto a=yunnan::allocate(allocation);REQUIRE(a["compensation_pool_cny"]==90);REQUIRE(a["rows"][0]["generation_charge_cny"]==15);REQUIRE(a["rows"][0]["assessment_return_cny"]==30);
  REQUIRE(a["compensation_balance_residual_cny"]==0);REQUIRE(a["assessment_balance_residual_cny"]==0);
  allocation["continuous_spot"]=false;a=yunnan::allocate(allocation);REQUIRE(a["rows"][0]["generation_charge_cny"]==30);REQUIRE(a["rows"][1]["user_charge_cny"]==0);
  allocation["continuous_spot"]=true;allocation["participants"][1]["import_mwh"]=0;REQUIRE_THROWS(yunnan::allocate(allocation));
}

TEST_CASE("Yunnan monthly journal replaces corrections and uses whole-month denominators", "[southern_market][ancillary][rules]") {
  // Accounting-layer oracle: algorithmic feasibility is tested separately.
  json result={{"clearing_id",1},{"ancillary",{{"settlement_eligible",true},{"config",{{"agc_units",{{{"id",0},{"mode","single"},{"members",{{{"generator_id",1}}}}}}},{"independent_units",json::array()}}}}},
    {"statement",{{"complete",true},{"metering",{{"rows",{{{"unit_id",0},{"compensation_cny",100}}}},{"total_compensation_cny",100}}},{"request",{{"measurements",{{{"events",{{{"event_id","day-one"}}}}}}}}}}}};
  json journal=json::array(),entry={{"book","oracle"},{"delivery_date","2026-02-01"},{"posting_date","2026-03-01"},{"discovered_date",nullptr},{"supersedes_id",nullptr},{"reason","original"},{"source","hand accounting evidence"}};
  journal=post_yunnan_ancillary_statement(journal,result,entry);
  REQUIRE_THROWS(post_yunnan_ancillary_statement(journal,result,entry));
  json monthly={{"book","oracle"},{"month","2026-02"},{"allocation",{{"continuous_spot",false},{"generation_share",.5},{"assessment_pool_cny",0},{"participants",{
    {{"id",1},{"source","monthly meter A"},{"point_to_grid",false},{"unit_ids",{0}},{"export_mwh",90},{"nonmarket_export_mwh",90},{"import_mwh",0}},
    {{"id",2},{"source","monthly meter B"},{"point_to_grid",false},{"unit_ids",json::array()},{"export_mwh",10},{"nonmarket_export_mwh",10},{"import_mwh",0}}}}}}};
  REQUIRE(settle_yunnan_ancillary_month(journal,monthly)["missing_days"].size()==27);
  for(int d=2;d<=28;++d) {
    result["clearing_id"]=d;entry["delivery_date"]="2026-02-"+std::string(d<10?"0":"")+std::to_string(d);
    result["statement"]["metering"]["rows"][0]["compensation_cny"]=d==2?300:0;
    result["statement"]["metering"]["total_compensation_cny"]=d==2?300:0;
    result["statement"]["request"]["measurements"][0]["events"][0]["event_id"]="day-"+std::to_string(d);
    journal=post_yunnan_ancillary_statement(journal,result,entry);
  }
  const auto month=settle_yunnan_ancillary_month(journal,monthly);REQUIRE(month["complete"]==true);
  REQUIRE(month["allocation"]["rows"][0]["generation_charge_cny"]==360);REQUIRE(month["allocation"]["rows"][1]["generation_charge_cny"]==40);
  result["clearing_id"]=29;entry["delivery_date"]="2026-02-01";entry["supersedes_id"]=1;entry["discovered_date"]="2026-03-01";entry["posting_date"]="2026-03-02";
  result["statement"]["request"]["measurements"][0]["events"][0]["event_id"]="day-one";
  result["statement"]["metering"]["rows"][0]["compensation_cny"]=200;result["statement"]["metering"]["total_compensation_cny"]=200;
  const auto corrected=post_yunnan_ancillary_statement(journal,result,entry);REQUIRE(corrected.size()==29);REQUIRE(corrected[28]["raw_compensation_delta_cny"]==100);
  REQUIRE(settle_yunnan_ancillary_month(corrected,monthly)["allocation"]["compensation_pool_cny"]==500);
  result["clearing_id"]=30;REQUIRE_THROWS(post_yunnan_ancillary_statement(corrected,result,entry));
  entry["supersedes_id"]=29;entry["posting_date"]="2026-04-02";REQUIRE_THROWS(post_yunnan_ancillary_statement(corrected,result,entry));
  entry["discovered_date"]="2026-08-01";entry["posting_date"]="2026-08-02";REQUIRE_THROWS(post_yunnan_ancillary_statement(corrected,result,entry));
  entry["delivery_date"]="2026-02-29";REQUIRE_THROWS(post_yunnan_ancillary_statement(corrected,result,entry));
  entry["delivery_date"]="2026-03-01";entry["discovered_date"]=nullptr;entry["supersedes_id"]=nullptr;
  REQUIRE_THROWS(post_yunnan_ancillary_statement(corrected,result,entry));
  REQUIRE(yunnan::calendar_offset(yunnan::calendar_date("2028-01-31"),1)==std::chrono::sys_days{yunnan::calendar_date("2028-02-29")});
}

TEST_CASE("Yunnan pricing fallback ties hourly minima and cap have independent oracles", "[southern_market][ancillary]") {
  auto b=make_southern_market_example();
  auto second=b["generators"][0];second["id"]=2;b["generators"].push_back(second);
  auto c=yunnan_ancillary_defaults(b);c["cmin_mw"]=20;c["load_ratio"]=0;c["renewable_ratio"]=0;
  std::map<std::string,double> fixed;
  for(int id:{1,2}) for(int t=0;t<98;++t) {fixed["stable/"+std::to_string(id)+"/"+std::to_string(t)]=1;fixed["primary/"+std::to_string(id)+"/"+std::to_string(t)]=0;}
  c["agc_units"][0]["k_history"]=std::vector<double>(8,1);
  c["agc_units"][1]["k_history"]=std::vector<double>(8,2);
  c["agc_units"][0]["price_per_mw"]=std::vector<double>(24,3);
  c["agc_units"][1]["price_per_mw"]=std::vector<double>(24,6);
  auto r=yunnan::prearrange(b,c,fixed);
  REQUIRE(r["hours"][0]["bids"][0]["id"]==1);REQUIRE(r["hours"][0]["reference_price_per_mw"]==6);
  // Last quarter unavailable: the hourly award must not use the other three quarters alone.
  fixed["stable/2/3"]=0;r=yunnan::prearrange(b,c,fixed);
  REQUIRE(r["hours"][0]["shortage_mw"]==10);REQUIRE(r["hours"][1]["shortage_mw"]==0);
  fixed["stable/2/3"]=1;
  c["agc_units"][0]["price_per_mw"][0]=9;
  r=yunnan::prearrange(b,c,fixed);
  REQUIRE(r["hours"][0]["bids"][1]["effective_price_per_mw"]==3);
  c["agc_units"][0]["default_price_per_mw"]=4;
  r=yunnan::prearrange(b,c,fixed);
  REQUIRE(r["hours"][0]["bids"][1]["effective_price_per_mw"]==4);
  c["agc_units"][0]["k_history"]=std::vector<double>(8,.1);
  r=yunnan::prearrange(b,c,fixed);REQUIRE(r["hours"][0]["reference_price_per_mw"]==15);
  c["agc_units"][0]["capacity_mw"][0]=100;
  r=yunnan::prearrange(b,c,fixed);REQUIRE(r["hours"][0]["bids"][1]["adjusted_mw"]==10);
  auto invalid=c;invalid["agc_units"][0]["unknown"]=1;REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  b["generators"][0]["regulation_up_mw"][0]=1;REQUIRE_THROWS(validate_yunnan_ancillary(b,c));
}

TEST_CASE("Yunnan independent storage and load enforce energy exclusion sustain and scarcity", "[southern_market][ancillary][rules]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  b["generators"][0]["must_on"]=std::vector<int>(98,1);
  const auto demo=make_southern_market_demo();
  auto s=demo.at("storage")[0];s["bus"]=b["buses"][0]["id"];b["storage"].push_back(s);
  auto d=demo.at("controllable_loads")[0];d["bus"]=b["buses"][0]["id"];b["controllable_loads"].push_back(d);
  auto c=yunnan_ancillary_defaults(b);c["cmin_mw"]=10;c["load_ratio"]=0;c["renewable_ratio"]=0;
  c["agc_units"][0]["qualified"]=false;
  for(auto& a:c["independent_units"]){a["qualified"]=true;a["capacity_mw"]=std::vector<double>(24,5);a["sustained_hours"]=1;a["standard_ramp_mw_min"]=5;}
  c["independent_units"][1]["baseline_reduction_mw"]=std::vector<double>(24,10);
  const auto da=run_yunnan_ancillary_market(b,c);INFO(da.value("error",std::string("")));INFO(da.at("status"));REQUIRE(da["schedule_feasible"]==true);
  const auto energy_ledger=analyze_southern_market_result(da,false);
  for(const auto& account:energy_ledger["settlement"]["accounts"])if(account["table"]=="controllable_loads")REQUIRE(account["receipt_cny"].get<double>()==Approx(0).margin(1e-6));
  for(int t=0;t<96;++t){REQUIRE(da["sced"]["storage"][0]["energy_market_eligible"][t]==false);REQUIRE(da["sced"]["controllable_loads"][0]["energy_market_eligible"][t]==false);}
  for(int t=0;t<96;++t){REQUIRE(da["sced"]["storage"][0]["discharge_mw"][t].get<double>()==Approx(0).margin(1e-6));REQUIRE(da["sced"]["storage"][0]["charge_mw"][t].get<double>()==Approx(0).margin(1e-6));REQUIRE(da["sced"]["storage"][0]["energy_mwh"][t].get<double>()==Approx(40).margin(1e-6));REQUIRE(da["sced"]["controllable_loads"][0]["reduction_mw"][t].get<double>()==Approx(10).margin(1e-6));}
  c["workflow"]["stage"]="intraday";const auto rt=run_yunnan_ancillary_intraday(b,c,da);REQUIRE(rt["schedule_feasible"]==true);
  auto bad=c;bad["independent_units"][0]["resource_id"]=999;REQUIRE_THROWS(validate_yunnan_ancillary(b,bad));
  c["workflow"]["stage"]="day_ahead";
  auto unavailable=c;unavailable["independent_units"][0]["sustained_hours"]=.99;
  auto rejected=run_yunnan_ancillary_market(b,unavailable);REQUIRE(rejected["status"]=="ancillary_capacity_shortfall");
  unavailable=c;unavailable["independent_units"][0]["cross_province_reserve_mw"][0]=1;
  rejected=run_yunnan_ancillary_market(b,unavailable);REQUIRE(rejected["status"]=="ancillary_capacity_shortfall");
  auto empty=b;empty["storage"][0]["initial_mwh"]=8;empty["storage"][0]["terminal_mwh"]=8;
  rejected=run_yunnan_ancillary_market(empty,c);REQUIRE(rejected["schedule_feasible"]==false);REQUIRE(rejected["ancillary"]["settlement_eligible"]==false);
  auto daily_cap=b;daily_cap["controllable_loads"][0]["max_day_reduction_mwh"]=300;
  // Baseline is 240 MWh but full up activation adds 120 MWh: hourly
  // capability alone cannot certify a 300 MWh daily limit.
  rejected=run_yunnan_ancillary_market(daily_cap,c);REQUIRE(rejected["ancillary"]["capacity_sufficient"]==true);REQUIRE(rejected["schedule_feasible"]==false);
  // A valid day-ahead energy award cannot be erased to admit storage intraday.
  auto fixed=da["commitment_solution"].get<std::map<std::string,double>>();
  const int sid=s["id"];for(int t=0;t<98;++t){fixed["da_dis/"+std::to_string(sid)+"/"+std::to_string(t)]=t==3?1:0;fixed["da_ch/"+std::to_string(sid)+"/"+std::to_string(t)]=0;}
  c["workflow"]["stage"]="intraday";auto candidates=yunnan::prearrange(b,c,fixed);
  for(const auto& bid:candidates["hours"][0]["bids"])if(bid["id"]==c["independent_units"][0]["id"])REQUIRE(bid["eligible"]==false);
  for(const auto& bid:candidates["hours"][1]["bids"])if(bid["id"]==c["independent_units"][0]["id"])REQUIRE(bid["eligible"]==true);
}

TEST_CASE("Yunnan hourly merit order and fixed primary have a hand oracle", "[southern_market][ancillary]") {
  auto b=make_southern_market_example();
  b["execution"]["ac_security"]="schedule_only";
  b["generators"][0]["must_on"]=std::vector<int>(98,1);
  auto g=b["generators"][0];g["id"]=7;g["name"]="second";g["segments"][0]["price_per_mwh"]=300;
  b["generators"].push_back(g);
  b["areas"][0]["primary_mw"]=std::vector<double>(98,5);
  for(auto& gen:b["generators"]) gen["primary_fraction"]=std::vector<double>(98,.1);
  auto c=yunnan_ancillary_defaults(b);c["cmin_mw"]=20;c["load_ratio"]=0;c["renewable_ratio"]=0;
  c["agc_units"][0]["k_history"]=std::vector<double>(8,2);c["agc_units"][0]["price_per_mw"]=std::vector<double>(24,4);
  c["agc_units"][1]["price_per_mw"]=std::vector<double>(24,3);
  const auto r=run_yunnan_ancillary_market(b,c);
  INFO(r.value("error",std::string("")));INFO(r.at("status"));
  REQUIRE(r["schedule_feasible"]==true);REQUIRE(r["prices_valid"]==true);
  REQUIRE(r["ancillary"]["status"]=="prearranged_schedule_only");
  REQUIRE(r["ancillary"]["settlement_cny"].is_null());
  for(const auto& h:r["ancillary"]["hours"]) {
    REQUIRE(h["reference_price_per_mw"].get<double>()==Approx(6).margin(1e-9));
    REQUIRE(h["awarded_mw"]==20);REQUIRE(h["shortage_mw"]==0);
    REQUIRE(h["bids"][0]["id"]==0);REQUIRE(h["bids"][1]["id"]==1);
  }
  REQUIRE(r["sced"]["variables"].get<int>()-r["scuc"]["variables"].get<int>()==392);
  for(size_t i=0;i<2;++i) for(int t=0;t<98;++t) {
    const auto& before=r["scuc"]["generators"][i];const auto& after=r["sced"]["generators"][i];
    REQUIRE(after["online"][t].get<double>()==Approx(before["online"][t].get<double>()).margin(1e-6));
    REQUIRE(after["primary_reserve_mw"][t].get<double>()==Approx(before["primary_reserve_mw"][t].get<double>()).margin(1e-6));
    REQUIRE(after["secondary_up_mw"][t].get<double>()==Approx(t<96?10:0).margin(1e-6));
    REQUIRE(after["secondary_down_mw"][t].get<double>()==Approx(t<96?10:0).margin(1e-6));
  }
  auto invalid=c;invalid["agc_units"][0]["members"][0]["generator_id"]=999;REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  invalid=c;invalid["agc_units"][0]["capacity_mw"][0]=.5;REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  invalid=c;invalid["agc_units"][0]["k_history"]={1};REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  invalid=c;invalid["research"]=false;REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  c["cmin_mw"]=1000;
  const auto shortage=run_yunnan_ancillary_market(b,c);
  REQUIRE(shortage["status"]=="ancillary_capacity_shortfall");REQUIRE_FALSE(shortage.contains("sced"));
  REQUIRE(shortage["ancillary"]["hours"][0]["reference_price_per_mw"].is_null());
}

TEST_CASE("Yunnan hydro safe bands constrain dispatch and preserve plant aggregation", "[southern_market][ancillary]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  auto hydro=b["generators"][0];hydro["kind"]="hydro";hydro["must_on"]=std::vector<int>(98,1);
  b["generators"][0]=hydro;auto second=hydro;second["id"]=9;b["generators"].push_back(second);
  auto c=yunnan_ancillary_defaults(b);c["cmin_mw"]=20;c["load_ratio"]=0;c["renewable_ratio"]=0;
  for(auto& a:c["agc_units"]) {a["members"][0]["safe_intervals_mw"]={{20,60},{80,140}};a["members"][0]["standard_ramp_mw_min"]=50;}
  const auto r=run_yunnan_ancillary_market(b,c);
  INFO(r.at("status"));REQUIRE(r["schedule_feasible"]==true);REQUIRE(r["prices_valid"]==true);
  for(const auto& g:r["sced"]["generators"]) for(int t=0;t<98;++t) {
    const double p=g["power_mw"][t],up=g["secondary_up_mw"][t],down=g["secondary_down_mw"][t],primary=g["primary_reserve_mw"][t];
    REQUIRE(((p-down>=20-1e-6 && p+primary+up<=60+1e-6) || (p-down>=80-1e-6 && p+primary+up<=140+1e-6)));
  }
  auto invalid=c;invalid["agc_units"][0]["mode"]="single";REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  invalid=c;invalid["agc_units"][0]["members"][0]["safe_intervals_mw"]={{20,80},{70,100}};REQUIRE_THROWS(validate_yunnan_ancillary(b,invalid));
  auto plant=c;plant["agc_units"][0]["members"].push_back(plant["agc_units"][1]["members"][0]);plant["agc_units"].erase(1);
  plant["cmin_mw"]=0;plant["agc_units"][0]["capacity_mw"]=std::vector<double>(24,0);
  REQUIRE_NOTHROW(validate_yunnan_ancillary(b,plant));
  // Aggregate supply exists, but the fixed 100 MW energy balance cannot fit
  // two [10,20] bands. Merit-order success must not masquerade as safe clearing.
  for(auto& a:c["agc_units"]) a["members"][0]["safe_intervals_mw"]={{10,30}};
  auto failed=run_yunnan_ancillary_market(b,c);
  REQUIRE(failed["ancillary"]["capacity_sufficient"]==true);
  REQUIRE(failed["ancillary"]["status"]=="coupled_safety_failed");REQUIRE(failed["schedule_feasible"]==false);
}

TEST_CASE("Southern study independently crosses faults inflows and bids", "[southern_market][study]") {
  const auto b = make_southern_market_demo();
  const json operation = {{"horizon","day"},{"start_date","2026-09-07"},{"days",json::array()},
    {"explain",false},{"penalty_per_mwh",100000},{"posthoc_ac_audit",false}};
  const json normal = {{"name","normal"},{"first_day",0},{"last_day",0},{"first_slot",40},{"last_slot",43},
    {"generator_outages",json::array()},{"branch_outages",json::array()}};
  auto fault = normal; fault["name"]="line";fault["branch_outages"]={b["branches"][0]["id"]};
  json config = {{"operation",operation},{"inflow_scales",{.5,1.5}},{"bid_scales",{1,1.2}},{"faults",{normal,fault}}};
  const auto job = make_market_study(b,config);
  const auto replay_difference=json::diff(job,make_market_study(b,job.at("config")));
  INFO(replay_difference.dump());REQUIRE(replay_difference.empty());
  auto invalid=config;invalid["mode"]="probabilistic";REQUIRE_THROWS(make_market_study(b,invalid));
  invalid=config;invalid["operation"]["reference_days"]=json::array();REQUIRE_THROWS(make_market_study(b,invalid));
  REQUIRE(job["scenarios"].size()==8);REQUIRE(job["total_days"]==8);
  for (const auto& scenario:job["scenarios"]) {
    REQUIRE(scenario["carry"]==job["scenarios"][0]["carry"]);
    REQUIRE(scenario["total_days"]==1);REQUIRE(scenario["config"]["days"].size()==2);
    const auto preview=preview_market_operation_boundary(b,scenario["config"],0)["effective"];
    for(int t=0;t<96;++t){
      REQUIRE(preview["reservoirs"][0]["inflow_m3_s"][t].get<double>()==Approx(b["reservoirs"][0]["inflow_m3_s"][t].get<double>()*scenario["inflow_scale"].get<double>()));
      REQUIRE(preview["branches"][0]["available"][t] == (scenario["fault"]["name"]=="line" && t>=40 && t<=43 ? 0 : 1));
    }
    REQUIRE(preview["generators"][0]["segments"][0]["price_per_mwh"].get<double>()==Approx(b["generators"][0]["segments"][0]["price_per_mwh"].get<double>()*scenario["bid_scale"].get<double>()));
  }
  auto stepped=step_market_forecast(job);
  REQUIRE(stepped["scenarios"][0]["status"]=="completed");
  REQUIRE(stepped["scenarios"][1]==job["scenarios"][1]);
  REQUIRE(stepped["statistics"]["periods"].size()==96);
  REQUIRE(stepped["statistics"]["week_delta_p_peak_mw"]["probability_bounds"].is_null());
  config["faults"][1]["branch_outages"]={999999};REQUIRE_THROWS(make_market_study(b,config));
  config["faults"]={normal};config["inflow_scales"]={1,1};REQUIRE_THROWS(make_market_study(b,config));
}

TEST_CASE("Southern conditional ledger has independent nodal cashflow identity", "[southern_market][study]") {
  auto b=make_southern_market_example();
  b["execution"]["balance_policy"]="diagnostic";
  b["execution"]["ac_security"]="schedule_only";
  const auto result=run_southern_day_ahead_market(b);
  REQUIRE(result["prices_valid"]==true);
  auto analysis=analyze_southern_market_result(result,true);
  const auto ledger=analysis["settlement"];
  REQUIRE(ledger["status"]=="conditional");REQUIRE(ledger["formal_settlement_eligible"]==false);
  REQUIRE(ledger["periods"].size()==96);REQUIRE(analysis["ac_audit"]["periods"].size()==98);
  for(const auto& p:ledger["periods"]){
    REQUIRE(p["residual_cny"].get<double>()==Approx(0).margin(1e-6));
    REQUIRE(p["line_rent_cny"].get<double>()==Approx(0).margin(1e-6));
    REQUIRE(p["load_payment_cny"].get<double>()==Approx(100*.25*result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>()));
  }
  auto corrupt=result;corrupt["sced"]["generators"][0]["power_mw"][0]=0;
  REQUIRE(analyze_southern_market_result(corrupt,false)["settlement"]["status"]=="balance_failed");
  corrupt=result;corrupt["prices_valid"]=false;
  REQUIRE(analyze_southern_market_result(corrupt,false)["settlement"]["status"]=="unavailable");
}

TEST_CASE("Southern mixed IEEE118 preserves source capacity and has shared cascade ownership", "[southern_market][ieee118_mixed]") {
  const auto source=hacdcpf::io::parse_matpower(std::string(HACDCPF_MATPOWER_DATA_DIR)+"/case118.m");
  const auto b=make_southern_market_ieee118_mixed(source);
  const auto rich=make_ieee118_market_system(source);
  REQUIRE(b["buses"].size()==118); REQUIRE(b["branches"].size()==186);
  REQUIRE(b["generators"].size()==66); REQUIRE(b["reservoirs"].size()==12);
  REQUIRE(b["storage"].size()==6); REQUIRE(b["controllable_loads"].size()==6);
  std::map<std::string,int> kinds;std::map<int,json> generators;
  for(const auto& g:b["generators"]){++kinds[g.at("kind")];generators[g.at("id")]=g;}
  REQUIRE(kinds==std::map<std::string,int>{{"hydro",36},{"thermal",18},{"wind",6},{"solar",6}});
  for(const auto& g:source.ac.generators) {
    REQUIRE(generators.at(g.index)["bus"]==g.bus);
    REQUIRE(generators.at(g.index)["pmax_mw"][0]==g.pmax_mw);
  }
  std::set<int> owned;int roots=0;
  for(const auto& h:b["reservoirs"]) {
    REQUIRE(h["generators"].size()==3);if(h["upstream"]==-1)++roots;
    for(const auto& id:h["generators"]){REQUIRE(owned.insert(id.get<int>()).second);REQUIRE(generators.at(id)["kind"]=="hydro");}
  }
  REQUIRE(roots==4);REQUIRE(owned.size()==36);
  double energy=0,original_load=0,rich_load=0;
  for(const auto& bus:b["buses"])for(int t=0;t<96;++t)energy+=bus["load_mw"][t].get<double>()*.25;
  for(const auto& bus:source.ac.buses)original_load+=bus.pd_mw;
  REQUIRE(energy==Approx(original_load*24).margin(1e-6));
  for(const auto& bus:rich.ac.buses)rich_load+=bus.pd_mw;
  for(const auto& load:rich.ac.flexible_loads)rich_load+=load.p_mw;
  double expected=0;for(const auto& bus:b["buses"])expected+=bus["load_mw"][0].get<double>();
  REQUIRE(rich_load==Approx(expected).margin(1e-6));
  REQUIRE(rich.ac.generators.size()==66);REQUIRE(rich.ac.storage.size()==6);
  REQUIRE(rich.ac.flexible_loads.size()==6);REQUIRE(rich.dc.buses.empty());
  auto reordered=source;std::reverse(reordered.ac.generators.begin(),reordered.ac.generators.end());
  const auto reordered_boundary=make_southern_market_ieee118_mixed(reordered);
  for(const auto& g:reordered_boundary["generators"])
    REQUIRE(g["kind"]==generators.at(g["id"])["kind"]);
}

TEST_CASE("Southern IEEE118 Native fixed-integer repair reaches a verified schedule", "[southern_market][native_repair]") {
  const auto system=hacdcpf::io::parse_matpower(std::string(HACDCPF_MATPOWER_DATA_DIR)+"/case118.m");
  auto boundary=make_southern_market_ieee118(system);
  boundary["execution"]["solver"]="native";
  // Unoptimized sanitizer builds validate memory/numerics under an explicit
  // test budget; Release performance acceptance remains 30 seconds.
  double seconds=30;
  if(const char* configured=std::getenv("HACDCPF_TEST_NATIVE_REPAIR_SECONDS")) {
    seconds=std::stod(configured);
    REQUIRE(std::isfinite(seconds));
    REQUIRE(seconds>0);
    REQUIRE(seconds<=600);
  }
  boundary["execution"]["time_limit_sec"]=seconds;
  boundary["execution"]["balance_policy"]="diagnostic";
  boundary["execution"]["balance_penalty_per_mwh"]=100000;
  for(const auto* profile:{"default","enhanced"}) {
    boundary["execution"]["native_root_cuts"]=profile;
    const auto result=run_southern_day_ahead_market(boundary);
    INFO(profile); INFO(result["scuc"].value("solver_status",std::string()));
    REQUIRE(result["schedule_feasible"]==true);
    REQUIRE(result["prices_valid"]==true);
    for(const auto* stage:{"scuc","sced","lmp"}) {
      REQUIRE(result[stage]["max_residual"].get<double>()<=1e-6);
      REQUIRE(result[stage]["reconstructed_max_residual"].get<double>()<=1e-6);
    }
    const auto& scuc=result["scuc"];
    // Verified feasible Gurobi objective, same synthetic boundary; execution
    // contract, Native Fixed-Integer Repair Debug. Compare requested MIP gap.
    REQUIRE(scuc["objective"].get<double>()<=17413231.016942*1.01);
    REQUIRE(scuc["solver"]=="NativeBranchAndCut");
    REQUIRE(scuc["solution_quality"]=="optimal_within_tolerance");
    REQUIRE(scuc["limit_reached"]==false);
    REQUIRE(scuc["native_diagnostics"]["best_incumbent_cny"].is_number());
    REQUIRE(scuc["native_diagnostics"]["incumbent_updates"].get<int>()>0);
    REQUIRE(result["sced"]["generators"][0]["power_mw"].size()==98);
  }
}

TEST_CASE("Southern native root cut experiments retain the analytic oracle", "[southern_market][root_cuts]") {
  auto b=make_southern_market_example();
  REQUIRE(validate_southern_market(b)["execution"]["native_root_cuts"]=="default");
  b["execution"]["native_root_cuts"]="enhanced";
  REQUIRE_THROWS(validate_southern_market(b));
  b["execution"]["solver"]="native";
  b["execution"]["native_root_cuts"]="unknown";
  REQUIRE_THROWS(validate_southern_market(b));
  b["execution"]["ac_security"]="schedule_only";
  b["execution"]["time_limit_sec"]=10;
  for(const auto* profile:{"default","enhanced"}) {
    b["execution"]["native_root_cuts"]=profile;
    const auto result=run_southern_day_ahead_market(b);
    INFO(result.dump().substr(0,2000));
    REQUIRE(result["prices_valid"]==true);
    REQUIRE(result["scuc"]["objective"].get<double>()==Approx(490000).margin(1e-6));
    REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>()==Approx(200).margin(1e-6));
    for(const auto* stage:{"scuc","sced","lmp"}) REQUIRE(result[stage]["max_residual"].get<double>()<=1e-6);
    const auto& d=result["scuc"]["native_diagnostics"];
    REQUIRE(d["profile"]==profile);
    REQUIRE(d["requested_root_rounds"]==(std::string(profile)=="enhanced"?20:10));
    REQUIRE(d["force_separation"]==(std::string(profile)=="enhanced"));
    REQUIRE(d["root_row_admission_limit"]==(std::string(profile)=="enhanced"?100000:35000));
    REQUIRE(d["best_bound_cny"].get<double>()==Approx(490000).margin(1e-6));
    REQUIRE(d["best_incumbent_cny"].get<double>()==Approx(490000).margin(1e-6));
    REQUIRE(result["lmp"]["native_diagnostics"].is_null());
  }
}

TEST_CASE("Southern certified row omission retains original feasibility and conditional prices", "[southern_market][row_presolve]") {
  for (bool resources : {false,true}) {
    auto boundary = resources ? make_southern_market_demo() : make_southern_market_example();
    boundary["execution"]["ac_security"] = "schedule_only";
    boundary["execution"]["mip_gap"] = 0;
    boundary["execution"]["balance_policy"] = "diagnostic";
    const auto original = run_southern_day_ahead_market(boundary);
    boundary["execution"]["row_presolve"] = "enabled";
    const auto pruned = run_southern_day_ahead_market(boundary);
    REQUIRE(original["prices_valid"] == true); REQUIRE(pruned["prices_valid"] == true);
    for (const auto* stage : {"scuc","sced","lmp"}) {
      const auto& a=original[stage]; const auto& b=pruned[stage];
      REQUIRE(b["max_residual"].get<double>() <= 1e-6);
      REQUIRE(b["reconstructed_max_residual"].get<double>() <= 1e-6);
      INFO(stage);
      // Equal-cost discrete schedules can define different conditional LMP LPs.
      if (std::string(stage)!="lmp" || !resources)
        REQUIRE(b["objective"].get<double>() == Approx(a["objective"].get<double>()).margin(1e-4));
      REQUIRE(b["variables"] == a["variables"]); REQUIRE(b["inequalities"] == a["inequalities"]);
      REQUIRE(b["nonzeros"] == a["nonzeros"]);
      const auto& size=b["model_size"];
      if (std::string(stage)=="lmp") REQUIRE(size["constraints_removed"] == 0);
      else {
        REQUIRE(size["constraints_removed"].get<int>() > 0);
        REQUIRE(size["constraints_removed"] == size["bound_redundant_inequalities"]);
        REQUIRE(size["submitted_inequalities"].get<int>()+size["constraints_removed"].get<int>() == b["inequalities"]);
        REQUIRE(size["submitted_nonzeros"].get<int>() < b["nonzeros"].get<int>());
      }
    }
    if (!resources) {
      REQUIRE(pruned["scuc"]["objective"].get<double>() == Approx(490000).margin(1e-6));
      REQUIRE(pruned["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
    }
  }
  auto bad=make_southern_market_example();bad["execution"]["row_presolve"]="invalid";
  REQUIRE_THROWS(validate_southern_market(bad));
  bad["execution"]["row_presolve"]="enabled";bad["execution"]["formulation"]="reference";
  REQUIRE_THROWS(validate_southern_market(bad));
}

TEST_CASE("Southern IEEE 118 fixture preserves electrical network and marks synthetic limits", "[southern_market][ieee118]") {
  const auto system = hacdcpf::io::parse_matpower(std::string(HACDCPF_MATPOWER_DATA_DIR)+"/case118.m");
  const auto b = make_southern_market_ieee118(system);
  REQUIRE(b["buses"].size() == 118); REQUIRE(b["branches"].size() == 186);
  REQUIRE(b["generators"].size() == 60); REQUIRE(b["reservoirs"].size() == 2);
  REQUIRE(b["storage"].size() == 2); REQUIRE(b["controllable_loads"].size() == 2);
  REQUIRE(b["reservoirs"][1]["upstream"] == b["reservoirs"][0]["id"]);
  std::set<int> members;
  for (const auto& h : b["reservoirs"]) {
    REQUIRE(h["generators"].size() == 2);
    for (const auto& id : h["generators"]) REQUIRE(members.insert(id.get<int>()).second);
  }
  for (size_t i=0;i<system.ac.branches.size();++i) {
    const auto& authored = system.ac.branches[i]; const auto& line = b["branches"][i];
    REQUIRE(line["id"] == authored.index); REQUIRE(line["from_bus"] == authored.from_bus);
    REQUIRE(line["to_bus"] == authored.to_bus); REQUIRE(line["x_pu"] == authored.x_pu);
    REQUIRE(line["r_pu"] == authored.r_pu); REQUIRE(line["shift_deg"] == authored.shift_deg);
    REQUIRE(line["max_mw"][0] == 200);
    REQUIRE(line["source"].get<std::string>().find("synthetic") != std::string::npos);
  }
  auto invalid = system; invalid.ac.branches.pop_back(); REQUIRE_THROWS(make_southern_market_ieee118(invalid));
  REQUIRE_FALSE(system.ac.transformers_2w.empty());
  invalid = system; invalid.ac.transformers_2w[0].source_branch_idx = -1;
  REQUIRE_THROWS(make_southern_market_ieee118(invalid));
  invalid = system; invalid.ac.transformers_2w[0].tap_step_percent = 1;
  REQUIRE_THROWS(make_southern_market_ieee118(invalid));
  REQUIRE(b["periods"].size() == 98);
  // Variable time steps are not supported by the Southern 96+2 contract.
  auto changed = b; changed["periods"][0]["duration_hr"] = .5;
  REQUIRE_THROWS(validate_southern_market(changed));
}

TEST_CASE("Southern model inspection counts authored families without solving or pruning", "[southern_market][model_size]") {
  auto boundary = make_southern_market_example();
  const auto before = boundary;
  const auto report = inspect_southern_market_model(boundary);
  REQUIRE(boundary == before);
  REQUIRE(report["solved"] == false);
  REQUIRE_FALSE(report.contains("prices_valid"));
  const auto& size = report["model_size"];
  size_t variables=0, binary=0, eq=0, le=0, nnz=0, redundant=0;
  for (const auto& family : size["variable_families"]) {
    variables += family["variables"].get<size_t>();
    binary += family["binary_variables"].get<size_t>();
  }
  for (const auto& family : size["constraint_families"]) {
    eq += family["equalities"].get<size_t>();
    le += family["inequalities"].get<size_t>();
    nnz += family["nonzeros"].get<size_t>();
    redundant += family["bound_redundant_inequalities"].get<size_t>();
  }
  REQUIRE(variables == size["variables"]); REQUIRE(binary == size["binary_variables"]);
  REQUIRE(eq == size["equalities"]); REQUIRE(le == size["inequalities"]); REQUIRE(nnz == size["nonzeros"]);
  REQUIRE(redundant == size["bound_redundant_inequalities"]);
  REQUIRE(size["constraints_removed"] == 0);
  REQUIRE(size["commitment_by_kind"]["thermal"]["binary_variables"] == 98);
  REQUIRE(size["constraint_families"]["2.6.3.8"]["bound_redundant_inequalities"] == 98);
  // -p <= 0 follows from p >= 0; -p+stable <= 0 does not follow from the box.
  boundary["generators"][0]["pmin_mw"] = std::vector<double>(98,1);
  const auto tightened = inspect_southern_market_model(boundary);
  REQUIRE(tightened["model_size"]["constraint_families"]["2.6.3.8"]["bound_redundant_inequalities"] == 0);
  boundary = before;
  boundary["execution"]["ac_security"] = "schedule_only";
  const auto solved = run_southern_day_ahead_market(boundary);
  REQUIRE(solved["prices_valid"] == true);
  REQUIRE(solved["scuc"]["model_size"] == size);
  REQUIRE(solved["scuc"]["objective"].get<double>() == Approx(490000).margin(1e-6));
  REQUIRE(solved["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
}

TEST_CASE("Southern local algorithms share the 1 percent gap analytic oracle", "[southern_market][local_solvers]") {
  for (const auto* solver : {"highs","native"}) {
    auto b=make_southern_market_example();
    REQUIRE(b["execution"]["mip_gap"]==.01);
    b["execution"]["solver"]=solver; b["execution"]["time_limit_sec"]=10;
    b["execution"]["ac_security"]="schedule_only";
    const auto result=run_southern_day_ahead_market(b);
    INFO(solver); INFO(result.dump().substr(0,1500));
    REQUIRE(result["schedule_feasible"]==true); REQUIRE(result["prices_valid"]==true);
    REQUIRE(result["scuc"]["requested_mip_gap"]==.01);
    REQUIRE(result["scuc"]["solver"]==(std::string(solver)=="native" ? "NativeBranchAndCut" : "StrictHiGHS"));
    REQUIRE(result["scuc"]["objective"].get<double>()==Approx(490000).margin(1e-6));
    REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>()==Approx(200).margin(1e-6));
    for(const auto* s:{"scuc","sced","lmp"}) REQUIRE(result[s]["max_residual"].get<double>()<=1e-6);
  }
  REQUIRE(detail::southern_solver_status("Time limit reached",false,0).limit_reached);
  REQUIRE(detail::southern_solver_status("Node limit reached",true,.5).quality=="feasible_limit");
  REQUIRE(detail::southern_solver_status("Optimality gap reached",true,.009).quality=="optimal_within_tolerance");
  REQUIRE_FALSE(detail::southern_solver_status("Optimality gap reached",true,.009).optimality_proven);
}

TEST_CASE("Southern reduced thermal research fleet preserves network and hydro identities", "[southern_market][reduced_fleet]") {
  const auto system=hacdcpf::io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR)+"/case_ACTIVSg2000.m");
  const auto b=southern_market_from_system(system,true,120);
  REQUIRE(b["buses"].size()==2000); REQUIRE(b["branches"].size()==3206);
  REQUIRE(b["generators"].size()==1320); REQUIRE(b["reservoirs"].size()==180);
  REQUIRE(b["storage"].size()==80); REQUIRE(b["controllable_loads"].size()==120);
  std::map<std::string,int> counts; std::set<int> hydro;
  for(const auto& g:b["generators"]) { ++counts[g.at("kind")]; if(g["kind"]=="hydro")hydro.insert(g.at("id").get<int>()); }
  REQUIRE(counts["thermal"]==120); REQUIRE(counts["hydro"]==720);
  REQUIRE(counts["wind"]==240); REQUIRE(counts["solar"]==240);
  for(const auto& h:b["reservoirs"]) for(const auto& id:h["generators"]) REQUIRE(hydro.count(id)==1);
  REQUIRE_THROWS(southern_market_from_system(system,false,120));
}

TEST_CASE("Southern PTDF reproduces phase shifted flows and separates outage islands", "[southern_market][ptdf]") {
  auto b=make_southern_market_demo();
  auto extra=b["buses"][0]; extra["id"]=31; extra["load_mw"]=std::vector<double>(98,0); b["buses"].push_back(extra);
  extra["id"]=90; b["buses"].push_back(extra);
  auto line=b["branches"][0]; line["id"]=17; line["from_bus"]=2; line["to_bus"]=31; line["tap"]=2; line["shift_deg"]=10; b["branches"].push_back(line);
  line["id"]=83; line["from_bus"]=31; line["to_bus"]=1; line["tap"]=1; line["shift_deg"]=0; b["branches"].push_back(line);
  for(auto& l:b["branches"]) l["available"][40]=0;
  const auto ptdf=southern_market_ptdf(b,0,{1,17,83});
  REQUIRE(ptdf["unique_topologies"]==2); REQUIRE(ptdf["equivalent_periods"].size()==97);
  REQUIRE(ptdf["reference_bus_ids"].size()==2); REQUIRE(ptdf["factorizations"]==1);
  const std::vector<double> theta={0,.01,-.02,0}; std::vector<double> injection(4,0), flows;
  const std::map<int,int> pos={{1,0},{2,1},{31,2},{90,3}};
  for(const auto& l:b["branches"]) {
    const int a=pos.at(l.at("from_bus")), z=pos.at(l.at("to_bus"));
    const double f=100/(l["x_pu"].get<double>()*l["tap"].get<double>())*(theta[a]-theta[z]-l["shift_deg"].get<double>()*std::acos(-1)/180);
    injection[a]+=f; injection[z]-=f; flows.push_back(f);
  }
  for(size_t k=0;k<flows.size();++k) {
    const auto& row=ptdf["rows"][k]; double flow=row["phase_shift_flow_mw"];
    for(size_t i=0;i<injection.size();++i) flow+=row["coefficients"][i].get<double>()*injection[i];
    REQUIRE(flow==Approx(flows[k]).margin(1e-6)); REQUIRE(row["coefficients"][3]==0);
  }
  const auto outage=southern_market_ptdf(b,40,{17}); REQUIRE(outage["reference_bus_ids"].size()==4);
  REQUIRE(outage["rows"][0]["coefficients"]==std::vector<double>(4,0)); REQUIRE(outage["rows"][0]["phase_shift_flow_mw"]==0);
  REQUIRE_THROWS(southern_market_ptdf(b,98,{1})); REQUIRE_THROWS(southern_market_ptdf(b,0,{1,1})); REQUIRE_THROWS(southern_market_ptdf(b,0,{999}));
}

TEST_CASE("Southern compact projection preserves original startup history and objective", "[southern_market][compact]") {
  for (const int downtime : {239,240,719,720}) {
    auto b = make_southern_market_example();
    b["execution"]["ac_security"] = "schedule_only";
    auto& g = b["generators"][0];
    g["initial_on"] = 0; g["initial_power_mw"] = 0; g["initial_state_minutes"] = downtime;
    g["startup_cost"] = {17,17,17};
    // Forced load makes t=0 a start; a middle-day outage exercises all four transitions.
    b["execution"]["balance_policy"] = "diagnostic";
    g["must_off"][30] = 1; g["must_off"][31] = 1;
    b["execution"]["formulation"] = "reference";
    const auto reference = run_southern_day_ahead_market(b);
    b["execution"]["formulation"] = "compact";
    const auto compact = run_southern_day_ahead_market(b);
    REQUIRE(reference["prices_valid"] == true); REQUIRE(compact["prices_valid"] == true);
    for (const auto* stage : {"scuc","sced","lmp"}) {
      REQUIRE(compact[stage]["objective"].get<double>() == Approx(reference[stage]["objective"].get<double>()).margin(1e-4));
      REQUIRE(compact[stage]["max_residual"].get<double>() <= 1e-6);
      REQUIRE(compact[stage]["reconstructed_max_residual"].get<double>() <= 1e-6);
      REQUIRE(compact[stage]["compact_units"] == 1);
      const auto& a = compact[stage]["generators"][0]; const auto& r = reference[stage]["generators"][0];
      for (const auto* field : {"online","start","stop","hot_start","warm_start","cold_start","power_mw"})
        for (int t=0;t<98;++t) REQUIRE(a[field][t].get<double>() == Approx(r[field][t].get<double>()).margin(1e-6));
    }
    REQUIRE(reference["scuc"]["variables"].get<int>()-compact["scuc"]["variables"].get<int>() == 4*98);
    REQUIRE(reference["scuc"]["binary_variables"].get<int>()-compact["scuc"]["binary_variables"].get<int>() == 5*98);
  }
  auto b = make_southern_market_example();
  b["generators"][0]["startup_cost"] = {10,20,30};
  const auto noneligible = run_southern_day_ahead_market(b);
  REQUIRE(noneligible["prices_valid"] == true);
  REQUIRE(noneligible["scuc"]["compact_units"] == 0);
  b["execution"]["formulation"] = "invalid"; REQUIRE_THROWS(validate_southern_market(b));
}

TEST_CASE("Southern solver options normalize old inputs and reject unavailable selection", "[southern_market][solver_options]") {
  auto b = make_southern_market_example();
  b["execution"].erase("solver"); b["execution"].erase("threads");
  const auto effective = validate_southern_market(b);
  REQUIRE(effective["execution"]["solver"] == "highs"); REQUIRE(effective["execution"]["threads"] == 0);
  auto invalid = b; invalid["execution"]["solver"] = "unknown"; REQUIRE_THROWS(validate_southern_market(invalid));
  invalid = b; invalid["execution"]["threads"] = 2; REQUIRE_THROWS(validate_southern_market(invalid));
  invalid = b; invalid["execution"]["gurobi_method"] = "barrier"; REQUIRE_THROWS(validate_southern_market(invalid));
  invalid = b; invalid["execution"]["gurobi_method"] = "typo"; REQUIRE_THROWS(validate_southern_market(invalid));
  b["execution"]["solver"] = "gurobi";
  REQUIRE_NOTHROW(validate_southern_market(b));
  if (!southern_market_solver_capabilities()[1]["available"].get<bool>()) {
    REQUIRE_THROWS(run_southern_day_ahead_market(b));
    json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
    const auto initial=make_market_operation(b,c), failed=step_market_operation(initial);
    REQUIRE(failed["status"] == "failed"); REQUIRE(failed["completed_days"] == 0);
    REQUIRE(failed["carry"] == initial["carry"]);
  }
}

TEST_CASE("Southern reservoir coordinate preserves shared cascade conservation", "[southern_market][compact]") {
  auto small = make_southern_market_demo();
  REQUIRE_FALSE(small["reservoirs"].empty());
  auto downstream = small["reservoirs"][0];
  const auto members = downstream["generators"];
  small["reservoirs"][0]["generators"] = {members[0],members[1]};
  downstream["generators"] = {members[2],members[3]};
  downstream["id"] = 2; downstream["upstream"] = 1;
  small["reservoirs"].push_back(downstream);
  for (auto& g : small["generators"]) if (g["kind"] == "hydro") {
    g["pmin_mw"] = std::vector<double>(98,5); g["pmax_mw"] = std::vector<double>(98,5);
    g["must_on"] = std::vector<int>(98,1);
  }
  small["execution"]["reservoir_scaling"] = "original";
  const auto original = run_southern_day_ahead_market(small);
  small["execution"]["reservoir_scaling"] = "auto";
  const auto scaled = run_southern_day_ahead_market(small);
  REQUIRE(original["prices_valid"] == true); REQUIRE(scaled["prices_valid"] == true);
  for (const auto* stage : {"scuc","sced","lmp"}) {
    REQUIRE(scaled[stage]["max_residual"].get<double>() <= 1e-6);
    REQUIRE(scaled[stage]["objective"].get<double>() == Approx(original[stage]["objective"].get<double>()).margin(1e-4));
    for (size_t h=0;h<small["reservoirs"].size();++h) for (int t=0;t<98;++t)
      REQUIRE(scaled[stage]["reservoirs"][h]["level_m"][t].get<double>() == Approx(original[stage]["reservoirs"][h]["level_m"][t].get<double>()).margin(1e-6));
  }
}

TEST_CASE("Southern Gurobi clears identical sparse models and preserves rolling selection", "[southern_market][gurobi]") {
  const auto capabilities = southern_market_solver_capabilities();
  if (!capabilities[1]["available"].get<bool>()) { SKIP("Gurobi library or license unavailable"); }
  auto b = make_southern_market_example(); b["execution"]["solver"] = "gurobi"; b["execution"]["threads"] = 2;
  b["execution"]["gurobi_method"] = "barrier";
  const auto result = run_southern_day_ahead_market(b);
  INFO(result.dump().substr(0,2500));
  REQUIRE(result["schedule_feasible"] == true); REQUIRE(result["prices_valid"] == true);
  for (const auto* stage : {"scuc","sced","lmp"}) {
    REQUIRE(result[stage]["solver"] == "Gurobi");
    REQUIRE(result[stage]["requested_threads"] == 2);
    REQUIRE(result[stage]["lp_algorithm"] == (std::string(stage)=="lmp" ? "dual_simplex" : "barrier"));
    REQUIRE(result[stage]["max_residual"].get<double>() <= 1e-6);
    REQUIRE(result[stage]["optimality_proven"] == true);
  }
  REQUIRE(result["sced"]["day_energy_bid_cost"].get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(result["sced"]["generators"][0]["power_mw"][95].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
  auto demo = make_southern_market_demo(); const auto highs = run_southern_day_ahead_market(demo);
  demo["execution"]["solver"] = "gurobi"; demo["execution"]["threads"] = 2;
  const auto gurobi = run_southern_day_ahead_market(demo);
  INFO(gurobi.dump().substr(0,2000)); REQUIRE(gurobi["schedule_feasible"] == true); REQUIRE(gurobi["prices_valid"] == true);
  REQUIRE(highs["schedule_feasible"] == true);
  REQUIRE(gurobi["scuc"]["objective"].get<double>() == Approx(highs["scuc"]["objective"].get<double>()).margin(1e-4));
  for (const auto* stage : {"scuc","sced","lmp"}) REQUIRE(gurobi[stage]["max_residual"].get<double>() <= 1e-6);
  json config={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()},
    {"solver_options",{{"solver","gurobi"},{"threads",2},{"time_limit_sec",30},{"mip_gap",.01}}}};
  auto job = step_market_operation(make_market_operation(make_southern_market_example(),config));
  REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["stages"]["scuc"]["solver"] == "Gurobi");
  REQUIRE(job["days"][0]["stages"]["scuc"]["requested_mip_gap"] == .01);
  REQUIRE(job["days"][0]["stages"]["scuc"]["requested_time_limit_sec"] == 30);
  REQUIRE(preview_market_operation_boundary(b,config,0)["effective"]["execution"]["solver"] == "gurobi");
  config["solver_options"]["solver"] = "typo"; REQUIRE_THROWS(make_market_operation(b,config));
  config["solver_options"]["solver"] = "highs"; REQUIRE_THROWS(make_market_operation(b,config));
}

TEST_CASE("Gurobi sparse continuous adapter retains ranged-row dual identity", "[southern_market][gurobi]") {
  using namespace hacdcpf::engine;
  GurobiAdapter adapter(GurobiOptions{30,0,1});
  if (!adapter.available()) { SKIP("Gurobi library or license unavailable"); }
  REQUIRE_THROWS(GurobiAdapter(GurobiOptions{-1,0,1}));
  REQUIRE_THROWS(GurobiAdapter(GurobiOptions{30,0,1,6}));
  REQUIRE_THROWS(GurobiAdapter(GurobiOptions{30,0,1,2,5}));
  LPModel lp; lp.vars.resize(2);
  for (auto& v : lp.vars) { v.lb=0; v.ub=10; }
  lp.c=Eigen::Vector2d(1,2); lp.A.resize(1,2); lp.A.insert(0,0)=1;
  lp.b=Eigen::VectorXd::Constant(1,3); lp.row_lhs=Eigen::VectorXd::Constant(1,2);
  lp.Aeq.resize(1,2); lp.Aeq.insert(0,1)=1; lp.beq=Eigen::VectorXd::Constant(1,4);
  const auto solved=adapter.solve_lp(lp);
  REQUIRE(solved.stats.success); REQUIRE(solved.stats.objective == Approx(10).margin(1e-8));
  REQUIRE(solved.constraint_duals.size()==2);
  REQUIRE(solved.constraint_duals[0]==Approx(1).margin(1e-8));
  REQUIRE(solved.constraint_duals[1]==Approx(2).margin(1e-8));
  for (const int method : {1,2}) {
    const auto alternative = GurobiAdapter(GurobiOptions{30,0,1,method,method == 2 ? 0 : -1}).solve_lp(lp);
    REQUIRE(alternative.stats.success); REQUIRE(alternative.stats.objective == Approx(10).margin(1e-8));
    REQUIRE(alternative.constraint_duals[0] == Approx(1).margin(1e-8));
    REQUIRE(alternative.constraint_duals[1] == Approx(2).margin(1e-8));
  }
  GurobiAdapter limited(GurobiOptions{1e-9,0,1});
  const auto timeout = limited.solve_lp(lp);
  REQUIRE(timeout.stats.status == "TimeLimit");
  REQUIRE(timeout.constraint_duals.size() == 0);
  lp.row_lhs[0] = 11; lp.b[0] = 12;
  const auto impossible = adapter.solve_lp(lp);
  REQUIRE_FALSE(impossible.stats.success);
  REQUIRE((impossible.stats.status == "Infeasible" || impossible.stats.status == "UnboundedOrInfeasible"));
}

TEST_CASE("Southern primal completion never fixes final commitment", "[southern_market][gurobi][compact]") {
  if (!southern_market_solver_capabilities()[1]["available"].get<bool>()) SKIP("Gurobi unavailable");
  auto b = make_southern_market_example();
  b["execution"]["solver"] = "gurobi"; b["execution"]["mip_start"] = "enabled";
  b["execution"]["ac_security"] = "schedule_only";
  auto result = run_southern_day_ahead_market(b);
  REQUIRE(result["prices_valid"] == true);
  REQUIRE(result["scuc"]["primal_start"]["accepted"] == true);
  REQUIRE(result["sced"]["primal_start"]["status"] == "not_requested");
  REQUIRE(result["sced"]["day_energy_bid_cost"].get<double>() == Approx(480000).margin(1e-6));
  // The all-available candidate violates a required initial down-time. Final
  // MILP must still find the feasible later start under diagnostic balance.
  auto& g = b["generators"][0]; g["initial_on"] = 0; g["initial_power_mw"] = 0;
  g["initial_state_minutes"] = 0; g["min_down_minutes"] = 30;
  b["execution"]["balance_policy"] = "diagnostic";
  result = run_southern_day_ahead_market(b);
  REQUIRE(result["prices_valid"] == true);
  REQUIRE(result["scuc"]["primal_start"]["accepted"] == false);
  REQUIRE(result["scuc"]["generators"][0]["online"][0].get<double>() == Approx(0).margin(1e-6));
  REQUIRE(result["scuc"]["generators"][0]["online"][2].get<double>() == Approx(1).margin(1e-6));
}

TEST_CASE("Southern solver quality requires audited feasibility and exact adapter status", "[southern_market][lookahead]") {
  using detail::southern_solver_status;
  REQUIRE(southern_solver_status("StrictHiGHS Optimal run=0",true,0).optimality_proven);
  REQUIRE(southern_solver_status("HiGHS optimal",true,0).quality == "optimal_within_tolerance");
  REQUIRE_FALSE(southern_solver_status("StrictHiGHS Optimal run=0",false,0).optimality_proven);
  REQUIRE_FALSE(southern_solver_status("StrictHiGHS Optimal run=0",true,.01).optimality_proven);
  REQUIRE(southern_solver_status("StrictHiGHS TimeLimit run=1",true,.2).quality == "feasible_limit");
  REQUIRE(southern_solver_status("HiGHS time_limit",false,.2).quality == "limit_without_verified_solution");
  REQUIRE(southern_solver_status("StrictHiGHS UnboundedOrInfeasible run=0",false,0).quality == "infeasible_or_unbounded");
  REQUIRE(southern_solver_status("StrictHiGHS Infeasible run=0",false,0).quality == "proven_infeasible");
  REQUIRE_FALSE(southern_solver_status("suboptimal",true,0).optimality_proven);
  REQUIRE(southern_solver_status("Optimal",true,0).optimality_proven);
  REQUIRE(southern_solver_status("NodeLimit",true,.2).quality == "feasible_limit");
  REQUIRE(southern_solver_status("TimeLimit",false,0).quality == "limit_without_verified_solution");
}

TEST_CASE("Rolling next-day extrema drive all series and realized ramp carry", "[southern_market][operation][lookahead]") {
  auto b = make_southern_market_example();
  auto& g = b["generators"][0]; g["must_on"] = std::vector<int>(98,1); g["ramp_up_mw_min"] = 2;
  json config = {{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  auto baseline = step_market_operation(make_market_operation(b,config));
  REQUIRE(baseline["completed_days"] == 1);
  config = baseline["config"];
  REQUIRE(config["days"].size() == 8);
  const auto edit = [](const char* table,const char* field,int t,double value) {
    return json{{"table",table},{"id",1},{"field",field},{"first_slot",t},{"last_slot",t},{"value",value},{"reason","analytic next-day forecast"}};
  };
  config["days"][1]["boundary_overrides"] = json::array({edit("areas","load_mw",0,180),edit("areas","load_mw",95,60),edit("generators","pmin_mw",0,180)});
  const auto p = preview_market_operation_boundary(b,config,0);
  REQUIRE(p["lookahead"]["source_day"] == 1);
  REQUIRE(p["lookahead"]["points"][0]["kind"] == "peak");
  REQUIRE(p["lookahead"]["points"][0]["source_slot"] == 0);
  REQUIRE(p["authored"]["areas"][0]["load_mw"][96] == 180);
  REQUIRE(p["authored"]["areas"][0]["load_mw"][97] == 60);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][96] == 180);
  REQUIRE(p["authored"]["generators"][0]["pmin_mw"][96] == 180);
  for (int t = 0; t < 96; ++t) REQUIRE(p["authored"]["areas"][0]["load_mw"][t] == 100);
  auto job = step_market_operation(make_market_operation(b,config));
  INFO(job.dump().substr(0,1000)); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(150).margin(1e-6));
  REQUIRE(baseline["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(job["days"][0]["surplus_mwh"].get<double>() == Approx(17.5).margin(1e-6));
  REQUIRE(job["days"][0]["periods"].size() == 96);
  job = step_market_operation(job); REQUIRE(job["completed_days"] == 2);
  REQUIRE(job["days"][1]["state_start"] == job["days"][0]["state_end"]);
  config["days"][7]["load_scale"] = 1.5;
  REQUIRE(preview_market_operation_boundary(b,config,6)["authored"]["areas"][0]["load_mw"][96] == 150);
  REQUIRE(preview_market_operation_boundary(b,config,7)["forecast_only"] == true);
  auto invalid = config;
  invalid["days"][7]["boundary_overrides"] = json::array({edit("generators","ramp_up_mw_min",0,2)});
  invalid["days"][7]["boundary_overrides"][0]["last_slot"] = 97;
  REQUIRE_THROWS(make_market_operation(b,invalid));
  config["days"][0]["boundary_overrides"] = json::array({edit("areas","load_mw",96,100)});
  REQUIRE_THROWS(make_market_operation(b,config));
}

TEST_CASE("Rule boundary catalog matches admitted schemas and daily load reconciliation", "[southern_market][operation][boundary_rules]") {
  const auto schema = southern_market_schema(), catalog = southern_market_boundary_catalog();
  REQUIRE(catalog.at("items").size() == 15);
  for (const auto& group : catalog.at("items")) {
    REQUIRE_FALSE(group.at("clauses").get<std::string>().empty());
    REQUIRE_FALSE(group.at("limitation").get<std::string>().empty());
    for (const auto& f : group.at("fields")) REQUIRE(f.at("schema") == schema.at("properties").at(f.at("table").get<std::string>()).at("items").at("properties").at(f.at("field").get<std::string>()));
  }
  auto b=make_southern_market_example();
  b["areas"][0]["load_mw"]=std::vector<double>(98,200);
  b["buses"][0]["load_mw"]=std::vector<double>(98,1);
  auto other=b["buses"][0];other["id"]=22;other["load_mw"]=std::vector<double>(98,3);b["buses"].push_back(other);
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  auto p=preview_market_operation_boundary(b,c,0);
  REQUIRE(p["authored"]["buses"][0]["load_mw"][0]==1);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(50));
  REQUIRE(p["effective"]["buses"][1]["load_mw"][0].get<double>()==Approx(150));
  c=make_market_operation(b,c)["config"];
  json e={{"table","buses"},{"id",1},{"field","load_mw"},{"first_slot",0},{"last_slot",95},{"value",3},{"reason","forecast revised"}};
  c["days"][0]["boundary_overrides"]=json::array({e});
  p=preview_market_operation_boundary(b,c,0);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(100));
  REQUIRE(p["effective"]["buses"][1]["load_mw"][0].get<double>()==Approx(100));
  REQUIRE(p["effective"]["buses"][0]["load_mw"][96].get<double>()==Approx(50));
  REQUIRE(preview_market_operation_boundary(b,c,1)["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(50));
  auto invalid=c;invalid["days"][0]["boundary_overrides"].push_back(e);REQUIRE_THROWS(make_market_operation(b,invalid));
  for (const auto* field : {"initial_power_mw","segments","unimplemented_field"}) {
    invalid=c;invalid["days"][0]["boundary_overrides"][0]["table"]="generators";invalid["days"][0]["boundary_overrides"][0]["field"]=field;REQUIRE_THROWS(make_market_operation(b,invalid));
  }
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["id"]=999;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["id"]=4294967297LL;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["reason"]="";REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["value"]=-1;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["table"]="generators";invalid["days"][0]["boundary_overrides"][0]["field"]="pmin_mw";invalid["days"][0]["boundary_overrides"][0]["value"]=300;REQUIRE_THROWS(make_market_operation(b,invalid));
}

TEST_CASE("Rolling representatives synchronize every typed time series by stable ID", "[southern_market][operation][lookahead]") {
  const auto b = make_southern_market_demo();
  json c = {{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c = make_market_operation(b,c)["config"];
  c["days"][1]["wind_scale"] = .7; c["days"][1]["solar_scale"] = .8;
  c["days"][1]["inflow_scale"] = .9; c["days"][1]["load_scale"] = 1.1;
  c["days"][1]["line_limit_scale"] = .6; c["days"][1]["branch_outages"] = {b["branches"][0]["id"]};
  c["days"][1]["load_bid_scale"] = .5;
  const auto p = preview_market_operation_boundary(b,c,0), next = preview_market_operation_boundary(b,c,1);
  const auto schema = southern_market_schema();
  size_t compared = 0;
  for (const auto& point : p["lookahead"]["points"]) {
    const int target = point["target_slot"], source = point["source_slot"];
    for (auto table = schema["properties"].begin(); table != schema["properties"].end(); ++table) {
      if (table.key() == "periods" || table.value().value("type", "") != "array" || !table.value()["items"].contains("properties")) continue;
      for (const auto& row : p["authored"][table.key()]) {
        const auto& sources = next["authored"][table.key()];
        const auto it = std::find_if(sources.begin(),sources.end(),[&](const json& r){return r["id"] == row["id"];});
        REQUIRE(it != sources.end());
        for (auto f = table.value()["items"]["properties"].begin(); f != table.value()["items"]["properties"].end(); ++f)
          if (f.value().value("type", "") == "array" && f.value().value("minItems",0) == 98 && f.value().value("maxItems",0) == 98 && row.contains(f.key())) {
            REQUIRE(row[f.key()][target] == it->at(f.key())[source]); ++compared;
          }
      }
    }
  }
  REQUIRE(compared > 200);
}

TEST_CASE("Daily nonmarket plan enters clearing and forecast boundaries", "[southern_market][operation][boundary_rules]") {
  auto b=make_southern_market_example();
  b["external_schedules"].push_back({{"id",72},{"name","nonmarket nuclear"},{"source","analytic"},{"kind","nuclear"},{"bus",1},{"power_mw",std::vector<double>(98,0)}});
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c=make_market_operation(b,c)["config"];
  json e={{"table","external_schedules"},{"id",72},{"field","power_mw"},{"first_slot",0},{"last_slot",95},{"value",40},{"reason","approved nuclear plan"}};
  c["days"][0]["boundary_overrides"]=json::array({e});
  auto job=step_market_operation(make_market_operation(b,c));
  REQUIRE(job["days"][0]["valid"]==true);
  REQUIRE(job["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>()==Approx(60).margin(1e-6));
  REQUIRE(job["days"][0]["nodes"][0]["load_mw"][0]==100);
  auto f=market_forecast_defaults();f["sample_count"]=1;f["operation"]=c;
  for(auto& m:f["marginals"]) {m["distribution"]="fixed";m["center"]=std::vector<double>(7,1);}
  auto forecast=make_market_forecast(b,f);
  REQUIRE(forecast["scenarios"][0]["config"]["days"][0]["boundary_overrides"][0]==e);
  forecast=step_market_forecast(forecast);
  REQUIRE(forecast["scenarios"][0]["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>()==Approx(60).margin(1e-6));
}

TEST_CASE("Recovery triggering and historical dispatch-only explanations preserve realized state", "[southern_market][recovery_policy]") {
  auto b = make_southern_market_example();
  auto c = market_forecast_defaults().at("operation");
  c["horizon"] = "week"; c["start_date"] = "2028-02-01";
  c = make_market_operation(b,c).at("config");
  c["days"][0]["load_scale"] = 1.1;
  c["days"][1]["load_scale"] = 3.0;
  auto job = step_market_operation(make_market_operation(b,c));
  REQUIRE(job.at("days")[0].at("cause_analysis").at("status") == "not_triggered");
  REQUIRE(job.at("days")[0].at("counterfactuals").empty());
  REQUIRE(job.at("days")[0].at("diagnostic_prices_valid") == true);
  job = step_market_operation(job);
  REQUIRE(job.at("days")[1].at("cause_analysis").at("status") == "completed");
  const auto& automatic = job.at("days")[1].at("counterfactuals")[0];
  REQUIRE(automatic.at("pricing_scope") == "dispatch_only");
  REQUIRE(automatic.at("prices_valid") == false);
  REQUIRE_FALSE(automatic.at("stages").contains("lmp"));
  REQUIRE(automatic.at("reduction_deficit_mwh").get<double>() == Approx(2400).margin(1e-6));
  const auto dispatch = explain_market_operation_day(job,0);
  const auto full = explain_market_operation_day(job,0,"full");
  for (const auto* manual : {&dispatch,&full}) {
    REQUIRE(manual->at("carry") == job.at("carry"));
    REQUIRE(manual->at("completed_days") == 2);
    REQUIRE(manual->at("status") == job.at("status"));
    REQUIRE(manual->at("days")[1] == job.at("days")[1]);
    REQUIRE_FALSE(manual->at("days")[0].contains("recovery_execution"));
    REQUIRE(manual->at("days")[0].contains("manual_recovery_execution"));
    for (const auto* field : {"resources","nodes","lines","periods","state_start","state_end","stages","analysis","execution_timing"})
      REQUIRE(manual->at("days")[0].at(field) == job.at("days")[0].at(field));
  }
  const auto& a = dispatch.at("days")[0].at("counterfactuals")[0];
  const auto& f = full.at("days")[0].at("counterfactuals")[0];
  REQUIRE(f.at("stages").contains("lmp")); REQUIRE(f.at("prices_valid") == true);
  REQUIRE(a.at("periods") == f.at("periods"));
  for (const auto* stage : {"scuc","sced"}) {
    REQUIRE(a.at("stages").at(stage).at("objective").get<double>() == Approx(f.at("stages").at(stage).at("objective").get<double>()).margin(1e-6));
    REQUIRE(a.at("stages").at(stage).at("max_residual").get<double>() <= 1e-6);
  }
  REQUIRE_THROWS(explain_market_operation_day(job,2));
  REQUIRE_THROWS(explain_market_operation_day(job,-1));
  REQUIRE_THROWS(explain_market_operation_day(job,0,"unknown"));
  c["explain_trigger"] = "manual";
  auto manual = step_market_operation(make_market_operation(b,c));
  REQUIRE(manual.at("days")[0].at("cause_analysis").at("status") == "not_requested");
  c["explain_trigger"] = "always";
  auto always = step_market_operation(make_market_operation(b,c));
  REQUIRE(always.at("days")[0].at("counterfactuals").size() == 1);
  c["explain_trigger"] = "bad"; REQUIRE_THROWS(make_market_operation(b,c));
  c["explain_trigger"] = "anomaly"; c["recovery_pricing"] = "bad"; REQUIRE_THROWS(make_market_operation(b,c));
}

TEST_CASE("Gurobi internal wall timers report executed phases", "[southern_market][recovery_policy]") {
  const auto capabilities = southern_market_solver_capabilities();
  const auto cap = std::find_if(capabilities.begin(), capabilities.end(), [](const json& row) { return row.at("id") == "gurobi"; });
  if (cap == capabilities.end() || !cap->at("available").get<bool>()) { SUCCEED("Gurobi unavailable"); return; }
  auto b = make_southern_market_example(); b["execution"]["solver"] = "gurobi";
  const auto result = run_southern_day_ahead_market(b);
  REQUIRE(result.at("prices_valid") == true);
  for (const auto* stage : {"scuc","sced","lmp"}) {
    const auto& timing = result.at(stage).at("solver_timing"); double sum = 0;
    for (const auto* key : {"environment_sec","model_import_sec","optimize_sec","result_extract_sec"}) {
      REQUIRE(timing.at(key).is_number()); REQUIRE(timing.at(key).get<double>() >= 0);
      sum += timing.at(key).get<double>();
    }
    REQUIRE(sum <= result.at(stage).at("solve_wall_sec").get<double>() + 0.001);
    REQUIRE(timing.at("presolve_sec").is_null()); REQUIRE(timing.at("search_sec").is_null());
  }
}

TEST_CASE("Rolling market operation calendar and analytic deficit sensitivity", "[southern_market][operation]") {
  auto b = make_southern_market_example();
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", true}, {"days", json::array()}};
  auto invalid = c; invalid["unexpected"] = 1;
  REQUIRE_THROWS(make_market_operation(b, invalid));
  invalid = c; invalid["start_date"] = "2027-02-29";
  REQUIRE_THROWS(make_market_operation(b, invalid));
  auto job = make_market_operation(b, c);
  REQUIRE(job["total_days"] == 7);
  c = job["config"]; c["days"][1]["load_scale"] = 3;
  job = make_market_operation(b, c);
  for (int d = 0; d < 7; ++d) {
    job = step_market_operation(job);
    INFO(job.value("error", ""));
    REQUIRE(job["completed_days"] == d+1);
    const auto& day = job["days"][d]; REQUIRE(day["valid"] == true);
    REQUIRE(day["periods"].size() == 96);
    for (const auto& p : day["periods"]) REQUIRE(p["deficit_mw"].get<double>() == Approx(d == 1 ? 100 : 0).margin(1e-6));
    REQUIRE(day["deficit_mwh"].get<double>() == Approx(d == 1 ? 2400 : 0).margin(1e-6));
    if (d) REQUIRE(day["state_start"] == job["days"][d-1]["state_end"]);
  }
  REQUIRE(job["status"] == "completed");
  const auto& proof = job["days"][1]["counterfactuals"][0];
  REQUIRE(proof["factor"] == "load_scale"); REQUIRE(proof["valid"] == true);
  REQUIRE(proof["reduction_deficit_mwh"].get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(job["days"][1]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(job["days"][1]["state_end"]["generators"][0]["initial_state_minutes"].get<int>() == 4320);
  REQUIRE_THROWS(step_market_operation(job));
  b["areas"][0]["load_mw"] = std::vector<double>(98, 300);
  b["buses"][0]["load_mw"] = std::vector<double>(98, 300);
  REQUIRE(run_southern_day_ahead_market(b)["schedule_feasible"] == false);
}

TEST_CASE("Rolling market operation realizes a complete leap month", "[southern_market][operation]") {
  const auto b = make_southern_market_example();
  json c = {{"horizon", "month"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  auto job = make_market_operation(b, c); REQUIRE(job["total_days"] == 29);
  for (int d = 0; d < 29; ++d) { job = step_market_operation(job); INFO(job.value("error", "")); REQUIRE(job["completed_days"] == d+1); }
  REQUIRE(job["status"] == "completed");
  size_t points = 0; for (const auto& d : job["days"]) points += d["periods"].size(); REQUIRE(points == 2784);
  c["start_date"] = "2028-02-02"; REQUIRE_THROWS(make_market_operation(b, c));
  c["start_date"] = "2027-02-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 28);
  c["start_date"] = "2028-04-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 30);
  c["start_date"] = "2028-01-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 31);
}

TEST_CASE("Southern GUI factories expose valid named research boundaries", "[southern_market]") {
  const auto demo = make_southern_market_demo();
  REQUIRE(demo["buses"].size() == 2);
  REQUIRE(demo["generators"].size() == 8);
  REQUIRE(demo["reservoirs"].size() == 1);
  REQUIRE(demo["reservoirs"][0]["generators"].size() == 4);
  REQUIRE(demo["storage"].size() == 2);
  const auto result = run_southern_day_ahead_market(demo);
  INFO(result.value("status", ""));
  INFO((result.contains("lmp") ? result["lmp"].dump().substr(0, 4000) : result.dump().substr(0, 1000)));
  REQUIRE(result["schedule_feasible"] == true);
  REQUIRE(result["prices_valid"] == true);
  auto system = hacdcpf::io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR)+"/case_ACTIVSg2000.m");
  const auto imported = southern_market_from_system(system);
  REQUIRE(imported["generators"].size() == system.ac.generators.size());
  REQUIRE(imported["reservoirs"].empty());
  const auto large = southern_market_from_system(system, true);
  REQUIRE(large["buses"].size() == 2000);
  REQUIRE(large["branches"].size() == 3206);
  REQUIRE(large["generators"].size() == 1744);
  REQUIRE(large["reservoirs"].size() == 180);
  REQUIRE(large["storage"].size() == 80);
  REQUIRE(large["controllable_loads"].size() == 120);
}

namespace {
json series(double x) { return std::vector<double>(98, x); }
json empty_record(const json& schema) {
  if (schema.contains("enum")) return schema.at("enum")[0];
  const std::string type = schema.at("type");
  if (type == "object") {
    json result = json::object();
    for (auto it = schema.at("properties").begin(); it != schema.at("properties").end(); ++it) result[it.key()] = empty_record(it.value());
    return result;
  }
  if (type == "array") {
    json result = json::array();
    for (int i = 0; i < schema.at("minItems").get<int>(); ++i) result.push_back(empty_record(schema.at("items")));
    return result;
  }
  if (type == "string") return "synthetic-test";
  if (type == "integer") return std::max(0, schema.at("minimum").get<int>());
  return std::max(0.0, schema.at("minimum").get<double>());
}
json record(const char* table) {
  auto row = empty_record(southern_market_schema().at("properties").at(table).at("items"));
  row["id"] = 10; return row;
}
json two_bus() {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  j["buses"][0]["load_mw"] = series(0);
  auto node = j["buses"][0]; node["id"] = 2; node["load_mw"] = series(100); j["buses"].push_back(node);
  auto g = j["generators"][0]; g["id"] = 2; g["bus"] = 2; g["segments"][0]["price_per_mwh"] = 300;
  g["initial_power_mw"] = 0; j["generators"].push_back(g);
  auto line = record("branches"); line["id"] = 10; line["from_bus"] = 1; line["to_bus"] = 2;
  line["available"] = std::vector<int>(98, 1); line["x_pu"] = 0.1; line["tap"] = 1;
  line["min_mw"] = series(-50); line["max_mw"] = series(50); line["rate_mva"] = series(60);
  j["branches"].push_back(line); return j;
}
double first_power(const json& result, int position = 0) {
  return result.at("sced").at("generators")[position].at("power_mw")[0];
}
void require_schedule(const json& result) {
  INFO(result.value("error", ""));
  INFO(result.at("status"));
  if (result.contains("scuc")) INFO(result.at("scuc").at("solver_status"));
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("prices_valid") == true);
}
}

TEST_CASE("Southern realtime hydro standby and network outage have independent physical oracles", "[southern_market][realtime]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  auto h=b["generators"][0];h["id"]=2;h["kind"]="hydro";h["initial_on"]=0;h["initial_power_mw"]=0;h["must_off"]=std::vector<int>(98,1);b["generators"].push_back(h);
  b["generators"][0]["pmax_mw"]=series(100);b["areas"][0]["reserve_up_mw"]=series(50);
  auto c=southern_realtime_defaults(b);auto job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["runs"][0]["dispatch"]["schedule_feasible"]==true);
  c["boundary"]["generators"][1]["reserve_up_eligible"]=std::vector<int>(72,0);job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="failed");
  b=two_bus();b["generators"].erase(1);b["execution"]["balance_policy"]="diagnostic";
  c=southern_realtime_defaults(b);c["boundary"]["branches"][0]["available"][0]=0;
  job=step_southern_realtime(make_southern_realtime(b,c));const auto& result=job["runs"][0]["dispatch"]["sced"];
  REQUIRE(result["feasible"]==true);REQUIRE(result["branches"][0]["power_mw"][0].get<double>()==Approx(0).margin(1e-6));
  REQUIRE(result["buses"][1]["deficit_mw"][0].get<double>()==Approx(100).margin(1e-6));
  REQUIRE(result["branches"][0]["overload_mw"][1].get<double>()==Approx(50).margin(1e-6));
}

TEST_CASE("Southern realtime shared reservoir and storage carry actual quarter-hour state", "[southern_market][realtime]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";b["generators"][0]["kind"]="hydro";
  auto h=record("reservoirs");h.erase("generator");h["generators"]={1};h["upstream"]=-1;h["water_m3_mwh"]=3600;h["area_m2"]=90000;h["initial_level_m"]=100;
  h["physical_max_m"]=200;h["max_level_m"]=series(200);h["inflow_m3_s"]=series(0);h["release_max_m3_s"]=series(1000);h["release_ramp_m3_s"]=series(1000);h["max_mwh"]=2400;b["reservoirs"].push_back(h);
  auto c=southern_realtime_defaults(b);c["steps"]=2;auto job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="running");
  const auto& first=job["runs"][0]["dispatch"]["sced"];REQUIRE(first["reservoirs"][0]["level_m"][2].get<double>()==Approx(99).margin(1e-6));REQUIRE(first["reservoirs"][0]["level_m"][23].get<double>()==Approx(92).margin(1e-6));
  job=step_southern_realtime(job);REQUIRE(job["runs"][1]["dispatch"]["boundary_snapshot"]["reservoirs"][0]["initial_level_m"].get<double>()==Approx(99).margin(1e-6));
  b=make_southern_market_demo();c=southern_realtime_defaults(b);auto prev=realtime::window(realtime::prepare(b,c),0),next=realtime::window(realtime::prepare(b,c),3);
  json stage={{"generators",json::array()},{"storage",json::array()},{"reservoirs",json::array()},{"dc_links",json::array()}};
  prev["_rt"]["storage_hour_modes"]={{{"id",prev["storage"][0]["id"]},{"hour",0},{"mode","charge"},{"source","measured charging earlier this hour"}}};
  realtime::carry(next,prev,stage,3);REQUIRE(next["_rt"]["storage_hour_modes"]==prev["_rt"]["storage_hour_modes"]);
}

TEST_CASE("Southern realtime pumped plan and hydro interval energy preserve units", "[southern_market][realtime]") {
  auto b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";
  auto p=record("external_schedules");p["kind"]="pumped_storage";p["bus"]=1;p["power_mw"]=series(-20);b["external_schedules"].push_back(p);
  auto c=southern_realtime_defaults(b);c["boundary"]["generators"][0]["pmax_mw"]=std::vector<double>(72,120);
  c["pumped_reserve"]={{{"id",10},{"area",1},{"source","hand fixed pumping plan"},{"online_capacity_mw",std::vector<double>(72,0)}}};c["accident_reserve"][0]["requirement_mw"]=std::vector<double>(72,20);
  auto job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["runs"][0]["dispatch"]["schedule_feasible"]==true);
  c["accident_reserve"][0]["requirement_mw"][0]=21;job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="failed");
  b=make_southern_market_example();b["execution"]["ac_security"]="schedule_only";b["generators"][0]["kind"]="hydro";c=southern_realtime_defaults(b);
  c["hydro_plants"]={{{"id",8},{"source","hand interval bound"},{"generators",{1}},{"min_mw",std::vector<double>(72,0)},{"max_mw",std::vector<double>(72,200)},{"min_mwh",std::vector<double>(72,0)},{"max_mwh",std::vector<double>(72,100.0/12)}}};
  job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["runs"][0]["dispatch"]["schedule_feasible"]==true);REQUIRE(job["runs"][0]["dispatch"]["prices_valid"]==true);
  c["hydro_plants"][0]["max_mwh"][0]=8;job=step_southern_realtime(make_southern_realtime(b,c));REQUIRE(job["status"]=="failed");
}

TEST_CASE("Southern explicit 98-point boundary and analytic dispatch", "[southern_market]") {
  const auto boundary = make_southern_market_example();
  const auto result = run_southern_day_ahead_market(boundary);
  INFO(result.dump().substr(0, 8000));
  REQUIRE(result.at("status") == "converged");
  REQUIRE(result.at("feasible") == true);
  REQUIRE(result.at("scuc").at("max_residual").get<double>() <= 1e-6);
  REQUIRE(result.at("scuc").at("optimality_proven") == true);
  REQUIRE(result.at("sced").at("day_energy_bid_cost").get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(result.at("sced").at("objective").get<double>() == Approx(490000).margin(1e-6));
  REQUIRE(result.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(200).margin(1e-6));
  auto changed = boundary;
  changed["areas"][0]["load_mw"] = std::vector<double>(98, 101);
  const auto scenario = run_southern_day_ahead_market(changed);
  const auto comparison = compare_southern_market_results(result, scenario);
  REQUIRE(comparison.at("delta_day_energy_bid_cost").get<double>() == Approx(4800).margin(1e-6));
  REQUIRE(scenario.at("effective_boundary").at("buses")[0].at("load_mw")[0] == 101);
}

TEST_CASE("Rolling diagnostics distinguish a line limit breach from node deficit", "[southern_market][operation]") {
  auto b = two_bus(); b["generators"].erase(1);
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", true}, {"days", json::array()}};
  auto job = make_market_operation(b, c); c = job["config"];
  c["days"][0]["line_limit_scale"] = 0.5; c["days"][0]["first_slot"] = 4; c["days"][0]["last_slot"] = 7;
  job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  const auto& d = job["days"][0]; REQUIRE(d["valid"] == true);
  for (int t = 0; t < 96; ++t) {
    REQUIRE(d["periods"][t]["deficit_mw"].get<double>() == Approx(0).margin(1e-6));
    REQUIRE(d["lines"][0]["power_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(d["lines"][0]["overload_mw"][t].get<double>() == Approx(t >= 4 && t <= 7 ? 75 : 50).margin(1e-6));
    REQUIRE(d["lines"][0]["loading_percent"][t].get<double>() == Approx(t >= 4 && t <= 7 ? 400 : 200).margin(1e-6));
  }
  REQUIRE(d["counterfactuals"][0]["reduction_overload_mwh"].get<double>() == Approx(25).margin(1e-6));
  auto invalid = c; invalid["days"][0]["branch_outages"] = {999}; REQUIRE_THROWS(make_market_operation(b, invalid));
  c["days"][0]["branch_outages"] = {10};
  job = step_market_operation(make_market_operation(b, c)); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["periods"][4]["deficit_mw"].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(job["days"][0]["lines"][0]["overload_mw"][4].get<double>() == Approx(0).margin(1e-6));
  REQUIRE(job["days"][0]["lines"][0]["loading_percent"][4].is_null());
}

TEST_CASE("Paired recovery preserves independent inputs and deterministic factor ordering", "[southern_market][operation][recovery_parallel]") {
  if (!hacdcpf::engine::GurobiAdapter{}.available()) { SKIP("Gurobi is required for parallel recovery"); }
  auto b = make_southern_market_example();
  b["execution"]["solver"] = "gurobi"; b["execution"]["threads"] = 2;
  json config = {{"horizon","day"},{"start_date","2026-09-07"},
    {"penalty_per_mwh",100000},{"explain",true},{"days",json::array()}};
  config=make_market_operation(b,config)["config"];
  config["recovery_pricing"] = "full";
  config["days"][0]["load_scale"]=2.5;
  config["days"][0]["solar_scale"]=1.1;
  config["days"][0]["generator_bid_scale"]=1.1;
  const auto before = make_market_operation(b,config), saved = before;
  const auto job = step_market_operation(before);
  REQUIRE(before == saved);
  REQUIRE(job["completed_days"] == 1);
  const auto& day = job["days"][0];
  REQUIRE(day["counterfactuals"].size() == 3);
  REQUIRE(day["counterfactuals"][0]["factor"] == "load_scale");
  REQUIRE(day["counterfactuals"][1]["factor"] == "solar_scale");
  REQUIRE(day["counterfactuals"][2]["factor"] == "generator_bid_scale");
  REQUIRE(day["recovery_execution"]["resolved_solver_threads"] == 2);
  for(const auto& proof:day["counterfactuals"]) {
    REQUIRE(proof["valid"] == true);
    auto independent = config; independent["explain"] = false;
    independent["days"][0][proof["factor"].get<std::string>()] = 1;
    const auto expected = step_market_operation(make_market_operation(b,independent));
    REQUIRE(expected["completed_days"] == 1);
    for(const auto* field:{"deficit_mwh","surplus_mwh","overload_mwh"})
      REQUIRE(proof[std::string("reduction_")+field].get<double>() == Approx(day[field].get<double>()-expected["days"][0][field].get<double>()).margin(1e-6));
    for(const auto* stage:{"scuc","sced","lmp"}) {
      REQUIRE(proof["stages"][stage]["objective"].get<double>() == Approx(expected["days"][0]["stages"][stage]["objective"].get<double>()).margin(1e-6));
      REQUIRE(proof["stages"][stage]["max_residual"].get<double>() <= 1e-6);
    }
  }
}

TEST_CASE("Weekly reserve projection preserves SCED decision and constraint units", "[southern_market][operation][weekly_plan]") {
  auto b = make_southern_market_example();
  b["areas"][0]["primary_mw"] = std::vector<double>(98,20);
  b["generators"][0]["primary_fraction"] = std::vector<double>(98,.1);
  json c = {{"horizon","week"},{"start_date","2026-09-07"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  const auto original = run_southern_day_ahead_market(b);
  REQUIRE(original["schedule_feasible"] == true);
  const auto job = step_market_operation(make_market_operation(b,c));
  INFO(job.value("error","")); REQUIRE(job["completed_days"] == 1);
  const auto& resources = job["days"][0]["resources"];
  const auto& g = resources["generators"][0]; const auto& a = resources["areas"][0];
  REQUIRE(g["primary_reserve_mw"].size() == 96);
  for (int t=0;t<96;++t) {
    REQUIRE(g["power_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(g["online"][t].get<double>() == Approx(1).margin(1e-6));
    REQUIRE(g["primary_reserve_mw"][t].get<double>() == Approx(20).margin(1e-6));
    REQUIRE(g["primary_reserve_mw"][t] == original["sced"]["generators"][0]["primary_reserve_mw"][t]);
    REQUIRE(g["reserve_up_contribution_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(g["reserve_down_contribution_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(a["reserve_up_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(a["reserve_down_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(g["utilization_percent"][t].is_null());
    REQUIRE(g["renewable_available_mw"][t].is_null());
  }
}

TEST_CASE("Rolling shared hydro and storage carry realized state across days", "[southern_market][operation]") {
  const auto b = make_southern_market_demo();
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  auto job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  const auto end = job["days"][0]["state_end"];
  REQUIRE(end["storage"].size() == 2); REQUIRE(end["reservoirs"].size() == 1);
  REQUIRE(end["reservoirs"][0]["release_history_m3_s"].size() == 96);
  REQUIRE(end["reservoirs"][0]["release_history_m3_s"][95] == end["reservoirs"][0]["initial_release_m3_s"]);
  job = step_market_operation(job); INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 2);
  REQUIRE(job["days"][1]["state_start"] == end);
  auto invalid = b; invalid["generators"][0]["shutdown_curve_mw"] = {0};
  REQUIRE_THROWS(make_market_operation(invalid, c));
}

TEST_CASE("Rolling diagnostic surplus is distinct from equality residual", "[southern_market][operation]") {
  auto b = make_southern_market_example();
  b["generators"][0]["pmin_mw"] = series(150);
  b["generators"][0]["must_on"] = std::vector<int>(98, 1);
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  const auto job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["surplus_mwh"].get<double>() == Approx(1200).margin(1e-6));
  for (const auto& p : job["days"][0]["periods"]) REQUIRE(p["surplus_mw"].get<double>() == Approx(50).margin(1e-6));
  for (const auto& v : job["days"][0]["nodes"][0]["node_imbalance_mw"]) REQUIRE(std::abs(v.get<double>()) <= 1e-6);
}

TEST_CASE("Southern hand oracle: interruptible load chooses by compensation", "[southern_market][hand_oracle]") {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  auto d = record("controllable_loads");
  d["id"] = 7; d["bus"] = 1; d["available"] = std::vector<int>(98, 1);
  d["max_reduction_mw"] = series(20); d["compensation_per_mwh"] = series(50);
  d["max_day_reduction_mwh"] = 480;
  j["controllable_loads"].push_back(d);
  j["generators"][0]["segments"][0]["price_per_mwh"] = 100;
  const auto result = run_southern_day_ahead_market(j);
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("sced").at("day_load_reduction_mwh").get<double>() == Approx(480).margin(1e-6));
  REQUIRE(result.at("sced").at("day_generation_mwh").get<double>() == Approx(1920).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("demand_response").get<double>() == Approx(24500).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("energy").get<double>() == Approx(196000).margin(1e-6));

  j["controllable_loads"][0]["compensation_per_mwh"] = series(150);
  const auto no_reduction = run_southern_day_ahead_market(j);
  REQUIRE(no_reduction.at("sced").at("day_load_reduction_mwh").get<double>() == Approx(0).margin(1e-6));
  REQUIRE(no_reduction.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
}

TEST_CASE("Southern hand oracle: two units conserve one reservoir water", "[southern_market][hand_oracle]") {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  j["generators"][0]["kind"] = "hydro";
  auto g = j["generators"][0]; g["id"] = 2; g["pmax_mw"] = series(100); g["segments"][0]["quantity_mw"] = 100;
  j["generators"].push_back(g);
  auto h = record("reservoirs"); h.erase("generator"); h["generators"] = {1, 2}; h["upstream"] = -1;
  h["water_m3_mwh"] = 3600; h["area_m2"] = 90000; h["initial_level_m"] = 100;
  h["physical_max_m"] = 200; h["max_level_m"] = series(200); h["inflow_m3_s"] = series(0);
  h["release_max_m3_s"] = series(1000); h["release_ramp_m3_s"] = series(1000);
  h["initial_release_m3_s"] = 0; h["max_mwh"] = 2400;
  j["reservoirs"].push_back(h);
  const auto result = run_southern_day_ahead_market(j);
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("release_m3_s")[0].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("release_m3_s")[95].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("level_m")[95].get<double>() == Approx(4).margin(1e-6));
}

TEST_CASE("Southern A1 hand oracle: representative points dispatch but day energy integrates only 96 slots", "[southern_market][hand_oracle][boundary_rules]") {
  // Load 100 MW on the 96 day slots, 200 MW at the authored peak (t=96) and
  // 40 MW at the valley (t=97). Day energy = 100*0.25*96 = 2400 MWh and day bid
  // cost = 2400*200 = 480000 regardless of the representatives; the optimized
  // objective still prices them: 200*(2400+0.25*200+0.25*40) = 492000.
  auto b = make_southern_market_example();
  b["execution"]["ac_security"] = "schedule_only";
  b["areas"][0]["load_mw"][96] = 200; b["areas"][0]["load_mw"][97] = 40;
  b["buses"][0]["load_mw"][96] = 200; b["buses"][0]["load_mw"][97] = 40;
  const auto result = run_southern_day_ahead_market(b); require_schedule(result);
  const auto& power = result.at("sced").at("generators")[0].at("power_mw");
  REQUIRE(power[96].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(power[97].get<double>() == Approx(40).margin(1e-6));
  REQUIRE(result.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(result.at("sced").at("day_energy_bid_cost").get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("energy").get<double>() == Approx(492000).margin(1e-6));
  // Authored representative duration/weight scales the objective (200*(2400+1*200+0.25*40)
  // = 522000) but never leaks into the D-day energy accounts.
  b["periods"][96]["duration_hr"] = 1.0; b["periods"][96]["weight_hr"] = 1.0;
  const auto reweighted = run_southern_day_ahead_market(b); require_schedule(reweighted);
  REQUIRE(reweighted.at("sced").at("generators")[0].at("power_mw")[96].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(reweighted.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(reweighted.at("sced").at("day_energy_bid_cost").get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(reweighted.at("sced").at("objective_terms").at("energy").get<double>() == Approx(522000).margin(1e-6));
}

TEST_CASE("Southern A3 hand oracle: incremental bid segments price the marginal block and the 1 percent width is inclusive", "[southern_market][hand_oracle]") {
  auto b = make_southern_market_example();
  b["execution"]["ac_security"] = "schedule_only";
  auto& g = b["generators"][0];
  g["segments"] = {{{"quantity_mw", 100}, {"price_per_mwh", 200}}, {{"quantity_mw", 100}, {"price_per_mwh", 300}}};
  // Demand 150 MW: segment 1 full (100 MW @200) plus 50 MW @300.
  // Day bid cost = 96*0.25*(100*200+50*300) = 840000; in the pricing
  // neighborhood [142.5,157.5] segment 1 stays at its 100 MW cap and segment 2
  // is marginal -> LMP 300.
  b["areas"][0]["load_mw"] = series(150); b["buses"][0]["load_mw"] = series(150);
  const auto second = run_southern_day_ahead_market(b); require_schedule(second);
  REQUIRE(second.at("sced").at("day_energy_bid_cost").get<double>() == Approx(840000).margin(1e-6));
  REQUIRE(second.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(300).margin(1e-6));
  REQUIRE(second.at("lmp").at("buses")[0].at("lmp_per_mwh")[97].get<double>() == Approx(300).margin(1e-6));
  // Demand 80 MW stays inside segment 1: cost 96*0.25*80*200 = 384000, LMP 200.
  b["areas"][0]["load_mw"] = series(80); b["buses"][0]["load_mw"] = series(80);
  const auto first = run_southern_day_ahead_market(b); require_schedule(first);
  REQUIRE(first.at("sced").at("day_energy_bid_cost").get<double>() == Approx(384000).margin(1e-6));
  REQUIRE(first.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(200).margin(1e-6));
  // 2.2.10: each segment must cover at least 1% of the 200 MW flexible range;
  // exactly 2 MW is admitted, 1.99 MW is rejected.
  g["segments"] = {{{"quantity_mw", 2}, {"price_per_mwh", 200}}, {{"quantity_mw", 198}, {"price_per_mwh", 300}}};
  b["areas"][0]["load_mw"] = series(150); b["buses"][0]["load_mw"] = series(150);
  const auto boundary = run_southern_day_ahead_market(b); require_schedule(boundary);
  // 96*0.25*(2*200+148*300) = 1075200; the second block is marginal -> LMP 300.
  REQUIRE(boundary.at("sced").at("day_energy_bid_cost").get<double>() == Approx(1075200).margin(1e-6));
  REQUIRE(boundary.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(300).margin(1e-6));
  g["segments"] = {{{"quantity_mw", 1.99}, {"price_per_mwh", 200}}, {{"quantity_mw", 198.01}, {"price_per_mwh", 300}}};
  REQUIRE_THROWS(validate_southern_market(b));
}

TEST_CASE("Southern A4 hand oracle: startup class enters the new state exactly at the downtime threshold", "[southern_market][hand_oracle]") {
  // warm_after=240, cold_after=720. must_off[0]=1 with a single unit and strict
  // balance forces the first start at t=1, where downtime = initial_state+15:
  // 224+15=239 -> hot (10); 225+15=240 -> warm (20), equality enters the new
  // state; 704+15=719 -> warm; 705+15=720 -> cold (30).
  for (const int initial : {224, 225, 704, 705}) {
    auto b = make_southern_market_example();
    b["execution"]["ac_security"] = "schedule_only";
    auto& g = b["generators"][0];
    g["initial_on"] = 0; g["initial_power_mw"] = 0; g["initial_state_minutes"] = initial;
    g["startup_cost"] = {10, 20, 30}; g["must_off"][0] = 1;
    // No load at t=0 so the forced outage is feasible; from t=1 on the single
    // unit must serve 100 MW, pinning the start to t=1.
    b["areas"][0]["load_mw"][0] = 0; b["buses"][0]["load_mw"][0] = 0;
    const auto result = run_southern_day_ahead_market(b);
    INFO(initial);
    require_schedule(result);
    const auto& row = result.at("scuc").at("generators")[0];
    const int downtime = initial+15;
    const int expected = downtime < 240 ? 10 : downtime < 720 ? 20 : 30;
    REQUIRE(row.at("start")[0].get<double>() == Approx(0).margin(1e-6));
    REQUIRE(row.at("start")[1].get<double>() == Approx(1).margin(1e-6));
    REQUIRE(row.at("online")[1].get<double>() == Approx(1).margin(1e-6));
    REQUIRE(row.at("hot_start")[1].get<double>() == Approx(expected == 10 ? 1 : 0).margin(1e-6));
    REQUIRE(row.at("warm_start")[1].get<double>() == Approx(expected == 20 ? 1 : 0).margin(1e-6));
    REQUIRE(row.at("cold_start")[1].get<double>() == Approx(expected == 30 ? 1 : 0).margin(1e-6));
    REQUIRE(result.at("scuc").at("objective_terms").at("startup").get<double>() == Approx(expected).margin(1e-6));
  }
}

TEST_CASE("Southern A5 hand oracle: penalized priority shortfall sits at its guaranteed-energy bound and is priced at M4", "[southern_market][hand_oracle]") {
  // Zero gateway capacity delivers no trade energy. hard keeps the 480 MWh
  // guarantee as a hard floor -> infeasible; penalized_shortfall covers it with
  // the slack at its upper bound (= adjusted_min_mwh) for 480*M4 = 480*10000.
  // The mid-range shortfall oracle (800 guaranteed, 720 delivered, 80 short) is
  // already anchored by "[southern_market] priority trade mapping".
  auto j = two_bus();
  auto trade = record("trades"); trade["gateway_kind"] = "ac_branch"; trade["gateway"] = 10;
  trade["max_mw"] = series(0); trade["adjusted_min_mwh"] = 480; trade["original_min_mwh"] = 480; trade["max_mwh"] = 2000;
  j["trades"].push_back(trade);
  REQUIRE(run_southern_day_ahead_market(j).at("status") == "scuc_failed");
  j["execution"]["priority_policy"] = "penalized_shortfall";
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(result.at("sced").at("trades")[0].at("power_mw")[0].get<double>() == Approx(0).margin(1e-6));
  REQUIRE(result.at("sced").at("trades")[0].at("priority_shortfall_mwh").get<double>() == Approx(480).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("priority_shortfall").get<double>() == Approx(480.0*10000).margin(1e-6));
  REQUIRE(first_power(result) == Approx(0).margin(1e-6));
  REQUIRE(first_power(result, 1) == Approx(100).margin(1e-6));
}

TEST_CASE("Southern A6 hand oracle: spill closes the SI water balance at the upper level bound", "[southern_market][hand_oracle]") {
  // h=3600 m3/MWh turns 100 MW into 100 m3/s; S=90000 m2; inflow 110 m3/s.
  // The dispatch corridor pins the level at the 200 m ceiling, so every point
  // must spill 10 m3/s: release = P*h/3600 + spill = 110 and the level stays
  // 200 m. Spill penalty = 98*0.25*1000*(3600/3600)*10 = 245000.
  // Lag-slot history carry is already anchored by "Southern hydrology uses SI
  // conservation" (cascade section: levels 100.5/101.5 from supplied history).
  auto b = make_southern_market_example();
  b["execution"]["ac_security"] = "schedule_only";
  auto& g = b["generators"][0];
  g["kind"] = "hydro"; g["must_on"] = std::vector<int>(98, 1);
  g["pmin_mw"] = series(100); g["pmax_mw"] = series(100);
  auto h = record("reservoirs"); h["generator"] = 1; h["upstream"] = -1;
  h["lag_slots"] = 0; h["release_history_m3_s"] = json::array();
  h["water_m3_mwh"] = 3600; h["area_m2"] = 90000;
  h["initial_level_m"] = 200; h["physical_min_m"] = 0; h["physical_max_m"] = 200;
  h["min_level_m"] = series(200); h["max_level_m"] = series(200);
  h["inflow_m3_s"] = series(110); h["spill_max_m3_s"] = series(1000);
  h["release_min_m3_s"] = series(0); h["release_max_m3_s"] = series(1000); h["release_ramp_m3_s"] = series(1000);
  h["initial_release_m3_s"] = 110; h["min_mwh"] = 0; h["max_mwh"] = 2400;
  b["reservoirs"].push_back(h);
  const auto result = run_southern_day_ahead_market(b); require_schedule(result);
  const auto& r = result.at("sced").at("reservoirs")[0];
  for (const int t : {0, 47, 95, 97}) {
    REQUIRE(r.at("spill_m3_s")[t].get<double>() == Approx(10).margin(1e-6));
    REQUIRE(r.at("release_m3_s")[t].get<double>() == Approx(110).margin(1e-6));
    REQUIRE(r.at("level_m")[t].get<double>() == Approx(200).margin(1e-6));
  }
  REQUIRE(result.at("sced").at("objective_terms").at("hydro_spill").get<double>() == Approx(245000).margin(1e-6));
}

TEST_CASE("Southern A7 hand oracle: the 5 percent charge neighborhood switches storage pricing eligibility", "[southern_market][hand_oracle]") {
  // The 996 MWh terminal target forces charging 40 MW at every day slot; gen1
  // (200 CNY/MWh, pmax 139.5, non-price-setting) then covers 139.5 MW and gen2
  // (300 CNY/MWh) 0.5 MW. In the pricing LP gen2 is trapped by its own 5%
  // neighborhood [0.475,0.525] while the charging storage keeps the wider
  // ordered negative-charge neighborhood [(1+d)*(-40),(1-d)*(-40)] = [-42,-38]
  // (charge_max 50 leaves both sides inside the retained physical charge row),
  // so the storage becomes marginal: charge_price 250 -> ch=-39.975, LMP 250;
  // charge_price 350 -> ch=-40.025, LMP 350. With price_setting=0 the charge is
  // fixed at -40 and the margin returns to gen2: LMP 300.
  auto base = make_southern_market_example();
  base["execution"]["ac_security"] = "schedule_only";
  base["generators"][0]["pmax_mw"] = series(139.5);
  base["generators"][0]["price_setting"] = std::vector<int>(98, 0);
  auto g2 = base["generators"][0]; g2["id"] = 2; g2["pmax_mw"] = series(200);
  g2["segments"][0]["price_per_mwh"] = 300; g2["price_setting"] = std::vector<int>(98, 1);
  g2["initial_power_mw"] = 0;
  base["generators"].push_back(g2);
  auto s = record("storage"); s["id"] = 5; s["bus"] = 1; s["available"] = std::vector<int>(98, 1);
  s["discharge_min_mw"] = 0; s["discharge_max_mw"] = 0; s["charge_min_mw"] = 0; s["charge_max_mw"] = 50;
  s["rated_mwh"] = 1000; s["roundtrip_efficiency"] = 1.0; s["initial_mwh"] = 36; s["terminal_mwh"] = 996;
  // Pinning the SOC trajectory per slot (eta=1: +10 MWh per day slot) forces
  // exactly 40 MW charging and 0.5 MW from gen2 at every slot; charge_max 50
  // leaves the +/-5% pricing neighborhood free of the physical charge row.
  std::vector<double> soc(98);
  for (int t = 0; t < 98; ++t) soc[t] = 36+10*std::min(t+1, 96);
  s["min_mwh"] = soc; s["max_mwh"] = soc; s["max_cycles"] = 1;
  s["discharge_price"] = 0; s["charge_price"] = 250; s["price_setting"] = std::vector<int>(98, 1);
  base["storage"].push_back(s);
  const auto run = [&](double charge_price, int price_setting) {
    auto b = base;
    b["storage"][0]["charge_price"] = charge_price;
    b["storage"][0]["price_setting"] = std::vector<int>(98, price_setting);
    return run_southern_day_ahead_market(b);
  };
  const auto cheaper = run(250, 1);
  const auto dearer = run(350, 1);
  const auto excluded = run(250, 0);
  for (const auto* result : {&cheaper, &dearer, &excluded}) {
    require_schedule(*result);
    REQUIRE(result->at("sced").at("generators")[0].at("power_mw")[0].get<double>() == Approx(139.5).margin(1e-6));
    REQUIRE(result->at("sced").at("generators")[1].at("power_mw")[0].get<double>() == Approx(0.5).margin(1e-6));
    REQUIRE(result->at("sced").at("storage")[0].at("charge_mw")[0].get<double>() == Approx(-40).margin(1e-6));
    REQUIRE(result->at("sced").at("storage")[0].at("charge_mw")[95].get<double>() == Approx(-40).margin(1e-6));
  }
  REQUIRE(cheaper.at("lmp").at("storage")[0].at("charge_mw")[0].get<double>() == Approx(-39.975).margin(1e-6));
  REQUIRE(cheaper.at("lmp").at("generators")[1].at("power_mw")[0].get<double>() == Approx(0.475).margin(1e-6));
  REQUIRE(cheaper.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(250).margin(1e-6));
  REQUIRE(cheaper.at("lmp").at("buses")[0].at("lmp_per_mwh")[95].get<double>() == Approx(250).margin(1e-6));
  REQUIRE(cheaper.at("lmp").at("storage")[0].at("energy_mwh")[0].is_null());
  REQUIRE(dearer.at("lmp").at("storage")[0].at("charge_mw")[0].get<double>() == Approx(-40.025).margin(1e-6));
  REQUIRE(dearer.at("lmp").at("generators")[1].at("power_mw")[0].get<double>() == Approx(0.525).margin(1e-6));
  REQUIRE(dearer.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(350).margin(1e-6));
  REQUIRE(excluded.at("lmp").at("storage")[0].at("charge_mw")[0].get<double>() == Approx(-40).margin(1e-6));
  REQUIRE(excluded.at("lmp").at("generators")[1].at("power_mw")[0].get<double>() == Approx(0.5).margin(1e-6));
  REQUIRE(excluded.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(300).margin(1e-6));
  REQUIRE(excluded.at("lmp").at("buses")[0].at("lmp_per_mwh")[95].get<double>() == Approx(300).margin(1e-6));
}

TEST_CASE("Southern internal scenario sweep preserves weighted result semantics", "[southern_market][scenario]") {
  constexpr int scenarios = 20;
  std::mt19937 rng(20260905);
  std::normal_distribution<double> noise(0.0, 0.04);
  double probability_sum = 0.0;
  std::vector<double> load_factor, generation;
  for (int s = 0; s < scenarios; ++s) {
    auto j = make_southern_market_example();
    j["execution"]["ac_security"] = "schedule_only";
    const double factor = std::clamp(1.0 + noise(rng), 0.85, 1.15);
    const double probability = 1.0 / scenarios;
    probability_sum += probability;
    load_factor.push_back(factor);
    j["areas"][0]["load_mw"] = series(100 * factor);
    j["buses"][0]["load_mw"] = series(100 * factor);
    auto result = run_southern_day_ahead_market(j);
    REQUIRE(result.at("schedule_feasible") == true);
    REQUIRE(result.at("sced").at("max_residual").get<double>() <= 1e-6);
    REQUIRE(result.at("effective_boundary").at("areas")[0].at("load_mw")[0].get<double>() == Approx(100 * factor).margin(1e-6));
    generation.push_back(result.at("sced").at("day_generation_mwh").get<double>());
  }
  REQUIRE(probability_sum == Approx(1.0).margin(1e-12));
  REQUIRE(std::inner_product(load_factor.begin(), load_factor.end(), generation.begin(), 0.0) > 0.0);
  for (size_t i = 1; i < generation.size(); ++i) {
    if (load_factor[i] > load_factor[0] + 0.02) REQUIRE(generation[i] > generation[0]);
    if (load_factor[i] < load_factor[0] - 0.02) REQUIRE(generation[i] < generation[0]);
  }
}

TEST_CASE("Southern joint input scenarios emit overload price and renewable statistics", "[southern_market][scenario][statistics]") {
  constexpr int scenarios = 12;
  std::mt19937 rng(20260905); std::normal_distribution<double> eps(0.0, 0.06);
  int valid = 0, overload_events = 0; double price_sum = 0, renewable_util_sum = 0;
  for (int s = 0; s < scenarios; ++s) {
    auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
    const double load = std::clamp(1.0 + eps(rng), 0.85, 1.15);
    const double wind = std::clamp(1.0 + eps(rng), 0.0, 1.2);
    const double solar = std::clamp(1.0 + eps(rng), 0.0, 1.2);
    j["areas"][0]["load_mw"] = series(100 * load); j["buses"][0]["load_mw"] = series(100 * load);
    auto renewable = j["generators"][0]; renewable["id"] = 2; renewable["kind"] = "wind"; renewable["bid_mode"] = "quantity";
    renewable["pmax_mw"] = series(80); renewable["segments"][0]["quantity_mw"] = 80;
    renewable["forecast_mw"] = series(40 * wind); renewable["max_curtailment_mw"] = series(80);
    renewable["price_setting"] = std::vector<int>(98, 0); renewable["initial_power_mw"] = 0; j["generators"].push_back(renewable);
    auto solar_unit = renewable; solar_unit["id"] = 3; solar_unit["kind"] = "solar"; solar_unit["forecast_mw"] = series(30 * solar); j["generators"].push_back(solar_unit);
    auto d = record("controllable_loads"); d["id"] = 8; d["bus"] = 1; d["available"] = std::vector<int>(98, 1);
    d["max_reduction_mw"] = series(10); d["compensation_per_mwh"] = series(70 + 20 * load); d["max_day_reduction_mwh"] = 240; j["controllable_loads"].push_back(d);
    const auto result = run_southern_day_ahead_market(j);
    if (!result.value("schedule_feasible", false)) continue;
    ++valid; REQUIRE(result.at("sced").at("max_residual").get<double>() <= 1e-6);
    price_sum += result.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>();
    double forecast = 0, dispatched = 0;
    for (const auto& g : result.at("effective_boundary").at("generators")) if (g.at("kind") == "wind" || g.at("kind") == "solar") forecast += g.at("forecast_mw")[0].get<double>();
    for (const auto& g : result.at("sced").at("generators")) if (g.at("kind") == "wind" || g.at("kind") == "solar") dispatched += g.at("power_mw")[0].get<double>();
    if (forecast > 1e-9) renewable_util_sum += dispatched / forecast;
    for (const auto& line : result.at("sced").at("branches")) if (line.at("slack_plus_mw")[0].get<double>() > 1e-6 || line.at("slack_minus_mw")[0].get<double>() > 1e-6) ++overload_events;
  }
  REQUIRE(valid == scenarios); REQUIRE(price_sum / valid >= 0); REQUIRE(renewable_util_sum / valid >= 0);
  REQUIRE(overload_events >= 0);
}

TEST_CASE("Southern boundaries reject silent field and identity errors", "[southern_market]") {
  auto input = make_southern_market_example();
  SECTION("unknown field") { input["ignored"] = 1; }
  SECTION("missing field") { input.erase("periods"); }
  SECTION("wrong horizon") { input["periods"].erase(97); }
  SECTION("duplicate generator") { input["generators"].push_back(input["generators"][0]); }
  SECTION("invalid bus reference") { input["generators"][0]["bus"] = 555; }
  SECTION("maintenance and must on") { input["generators"][0]["available"][0] = 0; input["generators"][0]["must_on"][0] = 1; }
  SECTION("cannot reconcile zero forecast") { input["buses"][0]["load_mw"][0] = 0; }
  SECTION("mislabelled representative point") { input["periods"][97]["kind"] = "valley"; }
  REQUIRE_THROWS_AS(validate_southern_market(input), std::invalid_argument);
}

TEST_CASE("Southern congestion prices and maintenance are boundary sensitive", "[southern_market]") {
  auto j = two_bus();
  auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(first_power(result) == Approx(50).margin(1e-6));
  REQUIRE(first_power(result, 1) == Approx(50).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][1]["lmp_per_mwh"][0].get<double>() == Approx(300).margin(1e-6));
  SECTION("line outage changes topology") {
    j["branches"][0]["available"][0] = 0;
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(0).margin(1e-6));
    REQUIRE(first_power(result, 1) == Approx(100).margin(1e-6));
  }
  SECTION("generator maintenance cannot be bypassed") {
    j["generators"][0]["available"][0] = 0;
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(0).margin(1e-6));
  }
  SECTION("section binds separately from branch rating") {
    auto section = record("sections"); section["min_mw"] = series(-30); section["max_mw"] = series(30);
    section["members"] = {{{"branch", 10}, {"coefficient", 1}}}; j["sections"].push_back(section);
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(30).margin(1e-6));
  }
  SECTION("LMP matches load perturbation at the congested node") {
    j["areas"][0]["load_mw"][0] = 100.001;
    const auto next = run_southern_day_ahead_market(j); require_schedule(next);
    const double marginal = (next["sced"]["objective"].get<double>()-result["sced"]["objective"].get<double>())/0.00025;
    REQUIRE(marginal == Approx(300).margin(1e-4));
  }
}

TEST_CASE("Southern three-state starts, initial obligation, and trajectory", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto& g = j["generators"][0]; g["initial_on"] = 0; g["initial_power_mw"] = 0;
  g["startup_cost"] = {10, 20, 30};
  SECTION("hot") { g["initial_state_minutes"] = 100; }
  SECTION("warm inclusive threshold") { g["initial_state_minutes"] = 240; }
  SECTION("cold inclusive threshold") { g["initial_state_minutes"] = 720; }
  SECTION("residual downtime prevents initial start") {
    g["initial_state_minutes"] = 10; g["min_down_minutes"] = 60;
    const auto result = run_southern_day_ahead_market(j);
    REQUIRE(result["status"] == "scuc_failed"); return;
  }
  SECTION("authored startup trajectory") {
    for (auto& curve : g["startup_curves_mw"]) curve = {20, 40};
    j["areas"][0]["load_mw"][0] = 20; j["areas"][0]["load_mw"][1] = 40;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  const int expected = g["initial_state_minutes"].get<int>() < 240 ? 10 : g["initial_state_minutes"].get<int>() < 720 ? 20 : 30;
  REQUIRE(result["scuc"]["objective_terms"]["startup"].get<double>() == Approx(expected).margin(1e-6));
}

TEST_CASE("Southern provincial reserves, group policy, regulation and nonmarket plans", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto second = j["generators"][0]; second["id"] = 2; second["segments"][0]["price_per_mwh"] = 300;
  second["initial_power_mw"] = 0; j["generators"].push_back(second);
  double expected = 100;
  SECTION("positive reserve shortage") {
    j["areas"][0]["reserve_up_mw"] = series(301);
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("downward reserve shortage") {
    j["areas"][0]["reserve_down_mw"] = series(101);
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("primary frequency sum min") {
    j["areas"][0]["primary_mw"] = series(20);
    j["generators"][0]["primary_fraction"] = series(0.05);
    j["generators"][1]["primary_fraction"] = series(0.05);
  }
  SECTION("nonmarket nuclear schedule") {
    auto e = record("external_schedules"); e["bus"] = 1; e["kind"] = "nuclear"; e["power_mw"] = series(30);
    j["external_schedules"].push_back(e); expected = 70;
  }
  SECTION("regulation preclearing reduces SCED range") {
    j["generators"][0]["regulation_up_mw"] = series(120);
    j["generators"][1]["must_on"] = std::vector<int>(98, 1); expected = 80;
  }
  SECTION("group energy maximum") {
    auto group = record("groups"); group["generators"] = {1}; group["max_online"] = series(1);
    group["max_mw"] = series(200); group["max_mwh"] = 1200; j["groups"].push_back(group);
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    double energy = 0; for (int t = 0; t < 96; ++t) energy += result["sced"]["generators"][0]["power_mw"][t].get<double>()*0.25;
    REQUIRE(energy == Approx(1200).margin(1e-6)); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(first_power(result) == Approx(expected).margin(1e-6));
}

TEST_CASE("Southern storage signs, terminal energy and efficiency weighted cycle", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto s = record("storage"); s["bus"] = 1; s["available"] = std::vector<int>(98, 1);
  s["discharge_max_mw"] = 40; s["charge_max_mw"] = 40; s["rated_mwh"] = 100;
  s["roundtrip_efficiency"] = 0.81; s["initial_mwh"] = 50; s["terminal_mwh"] = 59;
  s["max_mwh"] = series(100); s["max_cycles"] = 2; s["price_setting"] = std::vector<int>(98, 1);
  s["discharge_price"] = 10000; s["charge_price"] = 0;
  j["storage"].push_back(s);
  SECTION("efficiency and independent terminal target") {}
  SECTION("zero cycles rejects unequal terminal target") {
    j["storage"][0]["max_cycles"] = 0;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  const auto& storage = result["sced"]["storage"][0];
  double charged = 0;
  for (int t = 0; t < 96; ++t) {
    REQUIRE(storage["charge_mw"][t].get<double>() <= 1e-6);
    charged -= storage["charge_mw"][t].get<double>()*0.25;
    const double before = t ? storage["energy_mwh"][t-1].get<double>() : 50;
    const double expected = before-storage["charge_mw"][t].get<double>()*0.9*0.25-storage["discharge_mw"][t].get<double>()/0.9*0.25;
    REQUIRE(storage["energy_mwh"][t].get<double>() == Approx(expected).margin(1e-6));
  }
  REQUIRE(charged == Approx(10).margin(1e-6));
  REQUIRE(storage["energy_mwh"][95].get<double>() == Approx(59).margin(1e-6));
  for (int hour = 0; hour < 24; ++hour) {
    double dis = 0, ch = 0;
    for (int t = 4*hour; t < 4*hour+4; ++t) { dis += storage["discharge_mw"][t].get<double>(); ch -= storage["charge_mw"][t].get<double>(); }
    REQUIRE(std::min(dis, ch) <= 1e-6);
  }
}

TEST_CASE("Costless unrestricted commitment has an exact available-state power projection", "[southern_market][compact][commitment_projection]") {
  auto b = make_southern_market_example();
  b["execution"]["ac_security"] = "schedule_only";
  b["execution"]["balance_policy"] = "diagnostic";
  b["execution"]["balance_penalty_per_mwh"] = 100000;
  b["execution"]["mip_gap"] = 0;
  auto& g = b["generators"][0];
  g["kind"] = "hydro"; g["minimum_cost_per_hour"] = 0; g["startup_cost"] = {0,0,0};
  g["min_up_minutes"] = 0; g["min_down_minutes"] = 0;
  g["max_starts"] = 98; g["max_stops"] = 98;
  g["technical_min_mw"] = 0; g["pmin_mw"] = series(0);
  g["ramp_up_mw_min"] = 1000; g["ramp_down_mw_min"] = 1000;
  bool eligible = true;
  SECTION("partial outage and restoration") { g["available"][2] = 0; }
  SECTION("startup cost retains choice") { eligible=false; g["startup_cost"]={1,1,1}; }
  SECTION("positive minimum retains choice") { eligible=false; g["pmin_mw"]=series(1); }
  SECTION("slow ramp retains choice") { eligible=false; g["ramp_up_mw_min"]=1; }
  SECTION("minimum uptime retains choice") { eligible=false; g["min_up_minutes"]=30; }
  SECTION("event budget retains choice") { eligible=false; g["max_starts"]=1; }
  const auto compact=run_southern_day_ahead_market(b); require_schedule(compact);
  // This reservoir-free fixture has identical thermal/hydro energy equations.
  // Thermal retains the pre-projection compact UC oracle, including free u.
  g["kind"]="thermal";
  const auto reference=run_southern_day_ahead_market(b); require_schedule(reference);
  REQUIRE(reference["scuc"]["projected_commitment_units"] == 0);
  REQUIRE(compact["scuc"]["projected_commitment_units"] == (eligible ? 1 : 0));
  for(const auto* stage:{"scuc","sced","lmp"}) {
    REQUIRE(compact[stage]["objective"].get<double>() == Approx(reference[stage]["objective"].get<double>()).margin(1e-5));
    REQUIRE(compact[stage]["max_residual"].get<double>() <= 1e-6);
  }
  if(eligible) {
    const auto& row=compact["scuc"]["generators"][0];
    for(int t=0;t<98;++t) REQUIRE(row["online"][t].get<double>() == g["available"][t].get<double>());
    REQUIRE(row["power_mw"][2].get<double>() == Approx(0).margin(1e-6));
    REQUIRE(compact["scuc"]["binary_variables"] == 0);
  }
}

TEST_CASE("Southern hourly storage projection preserves objective and physical constraints", "[southern_market][compact][storage_projection]") {
  auto b = make_southern_market_example();
  b["execution"]["ac_security"] = "schedule_only";
  b["execution"]["mip_gap"] = 0;
  auto s = record("storage"); s["bus"] = 1;
  s["available"] = std::vector<int>(98,1); s["available"][1] = 0;
  s["discharge_max_mw"] = 40; s["charge_max_mw"] = 40;
  s["rated_mwh"] = 100; s["max_mwh"] = series(100);
  s["initial_mwh"] = 50; s["terminal_mwh"] = 59;
  s["roundtrip_efficiency"] = .81; s["max_cycles"] = 2;
  s["discharge_price"] = 10000; s["charge_price"] = 0;
  s["price_setting"] = std::vector<int>(98,1);
  bool eligible = true;
  SECTION("partial availability and idle periods") {}
  SECTION("positive minimum retains per-slot formulation") {
    eligible = false; s["discharge_min_mw"] = 1; s["charge_min_mw"] = 1;
  }
  b["storage"].push_back(s);
  const auto compact = run_southern_day_ahead_market(b); require_schedule(compact);
  b["execution"]["formulation"] = "reference";
  const auto reference = run_southern_day_ahead_market(b); require_schedule(reference);
  REQUIRE(compact["scuc"]["compact_storage"] == (eligible ? 1 : 0));
  REQUIRE(compact["sced"]["binary_variables"] == (eligible ? 26 : 196));
  REQUIRE(reference["sced"]["binary_variables"] == 196);
  for (const auto* stage : {"scuc","sced","lmp"}) {
    REQUIRE(compact[stage]["objective"].get<double>() == Approx(reference[stage]["objective"].get<double>()).margin(1e-5));
    REQUIRE(compact[stage]["max_residual"].get<double>() <= 1e-6);
  }
  for (const auto* result : {&compact,&reference}) {
    const auto& row = (*result)["sced"]["storage"][0];
    for (int t=0;t<98;++t) {
      const double dis=row["discharge_mw"][t], ch=row["charge_mw"][t];
      const double prior=t ? row["energy_mwh"][t-1].get<double>() : 50;
      REQUIRE(row["energy_mwh"][t].get<double>() == Approx(prior-b["periods"][t]["duration_hr"].get<double>()*(dis/.9+.9*ch)).margin(1e-6));
      REQUIRE(std::min(dis,-ch) <= 1e-6);
    }
    REQUIRE(row["discharge_mw"][1].get<double>() == Approx(0).margin(1e-6));
    REQUIRE(row["charge_mw"][1].get<double>() == Approx(0).margin(1e-6));
    for (int h=0;h<24;++h) {
      double dis=0,ch=0;
      for(int t=4*h;t<4*h+4;++t) { dis+=row["discharge_mw"][t].get<double>();ch-=row["charge_mw"][t].get<double>(); }
      REQUIRE(std::min(dis,ch) <= 1e-6);
    }
  }
}

TEST_CASE("Southern hydrology uses SI conservation and D-day energy policy", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  j["generators"][0]["kind"] = "hydro";
  auto h = record("reservoirs"); h["generator"] = 1; h["upstream"] = -1;
  h["water_m3_mwh"] = 3600; h["area_m2"] = 90000; h["initial_level_m"] = 100;
  h["physical_max_m"] = 200; h["max_level_m"] = series(200); h["inflow_m3_s"] = series(90);
  h["release_max_m3_s"] = series(1000); h["release_ramp_m3_s"] = series(1000);
  h["initial_release_m3_s"] = 100; h["max_mwh"] = 2500;
  j["reservoirs"].push_back(h);
  SECTION("water conservation") {}
  SECTION("hydro daily energy cap") {
    j["reservoirs"][0]["max_mwh"] = 2399;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("cascade upstream delay includes supplied history") {
    j["reservoirs"][0]["release_history_m3_s"] = {50};
    auto g = j["generators"][0]; g["id"] = 2; g["must_off"] = std::vector<int>(98, 1);
    g["initial_on"] = 0; g["initial_power_mw"] = 0; j["generators"].push_back(g);
    auto downstream = h; downstream["id"] = 11; downstream["generator"] = 2; downstream["upstream"] = 10;
    downstream["lag_slots"] = 1; downstream["inflow_m3_s"] = series(0); downstream["initial_release_m3_s"] = 0;
    downstream["max_level_m"] = series(300); downstream["physical_max_m"] = 300;
    j["reservoirs"].push_back(downstream);
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(result["sced"]["reservoirs"][1]["level_m"][0].get<double>() == Approx(100.5).margin(1e-6));
    REQUIRE(result["sced"]["reservoirs"][1]["level_m"][1].get<double>() == Approx(101.5).margin(1e-6));
    json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
    auto job = step_market_operation(make_market_operation(j, c)); REQUIRE(job["completed_days"] == 1);
    REQUIRE(job["carry"]["reservoirs"][1]["initial_level_m"].get<double>() == Approx(195.5).margin(1e-6));
    job = step_market_operation(job); REQUIRE(job["completed_days"] == 2);
    REQUIRE(job["carry"]["reservoirs"][1]["initial_level_m"].get<double>() == Approx(291.5).margin(1e-6)); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(result["sced"]["reservoirs"][0]["level_m"][0].get<double>() == Approx(99.9).margin(1e-6));
  REQUIRE(result["sced"]["reservoirs"][0]["level_m"][95].get<double>() == Approx(90.4).margin(1e-6));
}

TEST_CASE("Southern HVDC loss, hub balance and adjustment reversal", "[southern_market]") {
  auto j = two_bus(); j["branches"] = json::array();
  auto dc = record("dc_links"); dc["from_bus"] = 1; dc["to_bus"] = 2; dc["from_hub"] = -1; dc["to_hub"] = -1;
  dc["available"] = std::vector<int>(98, 1); dc["max_mw"] = series(200); dc["loss_fraction"] = 0.1;
  dc["initial_mw"] = 100; dc["ramp_up_mw"] = series(200); dc["ramp_down_mw"] = series(200);
  j["dc_links"].push_back(dc);
  SECTION("constant line losses") {
    auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(100/0.9).margin(1e-6));
    REQUIRE(first_power(result, 1) == Approx(0).margin(1e-6));
  }
  SECTION("multiterminal hub") {
    auto hub = record("dc_hubs"); j["dc_hubs"].push_back(hub);
    j["dc_links"][0]["to_bus"] = -1; j["dc_links"][0]["to_hub"] = 10;
    dc["id"] = 11; dc["from_bus"] = -1; dc["from_hub"] = 10;
    j["dc_links"].push_back(dc);
    auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(100/0.81).margin(1e-6));
  }
  SECTION("forced consecutive opposite adjustments are infeasible") {
    j["dc_links"][0]["min_mw"][0] = 120; j["dc_links"][0]["max_mw"][0] = 120;
    j["dc_links"][0]["min_mw"][1] = 100; j["dc_links"][0]["max_mw"][1] = 100;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
}

TEST_CASE("Southern renewable categories and zero start limit", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  SECTION("quantity renewable deviation") {
    auto g = j["generators"][0]; g["id"] = 2; g["kind"] = "renewable"; g["bid_mode"] = "quantity";
    g["forecast_mw"] = series(120); g["max_curtailment_mw"] = series(120);
    g["price_setting"] = std::vector<int>(98, 0); j["generators"].push_back(g);
    const auto r = run_southern_day_ahead_market(j); require_schedule(r);
    REQUIRE(first_power(r, 1) == Approx(100).margin(1e-6));
    REQUIRE(r["sced"]["generators"][1]["renewable_deviation_mw"][0].get<double>() == Approx(20).margin(1e-6));
  }
  SECTION("zero starts forbids activation") {
    j["generators"][0]["initial_on"] = 0; j["generators"][0]["initial_power_mw"] = 0;
    j["generators"][0]["max_starts"] = 0;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
}

TEST_CASE("Southern AC feedback repeats the clearing chain and never prices failed security", "[southern_market]") {
  auto j = two_bus(); j["execution"]["ac_security"] = "required";
  j["branches"][0]["rate_mva"] = series(35);
  SECTION("thermal sensitivity cuts reduce transfer") {
    const auto result = run_southern_day_ahead_market(j);
    INFO(result.at("status"));
    REQUIRE(result["security_iterations"].size() > 1);
    REQUIRE(result["feasible"] == true);
    REQUIRE(first_power(result) < 35);
    REQUIRE(result["security_iterations"].back()["secure"] == true);
  }
  SECTION("iteration cap returns failure with no prices") {
    j["execution"]["security_iterations"] = 1;
    const auto result = run_southern_day_ahead_market(j);
    REQUIRE(result["status"] == "ac_security_failed");
    REQUIRE(result["feasible"] == false);
    REQUIRE(result["prices_valid"] == false);
    REQUIRE_FALSE(result.contains("lmp"));
  }
}

TEST_CASE("Southern priority trade mapping and strict versus penalized energy", "[southern_market]") {
  auto j = two_bus();
  auto trade = record("trades"); trade["gateway_kind"] = "ac_branch"; trade["gateway"] = 10;
  trade["max_mw"] = series(30); trade["adjusted_min_mwh"] = 800; trade["original_min_mwh"] = 800;
  trade["max_mwh"] = 2000; j["trades"].push_back(trade);
  SECTION("hard priority infeasible") {
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
  SECTION("explicit slack interpretation") {
    j["execution"]["priority_policy"] = "penalized_shortfall";
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(30).margin(1e-6));
    REQUIRE(result["sced"]["trades"][0]["priority_shortfall_mwh"].get<double>() == Approx(80).margin(1e-6));
  }
}

TEST_CASE("Southern LMP derivation preserves trade fees and noncompact asset branches", "[southern_market][lmp_reuse_branches]") {
  auto boundary=make_southern_market_demo();
  if(southern_market_solver_capabilities()[1]["available"].get<bool>())boundary["execution"]["solver"]="gurobi";
  boundary["execution"]["assembly_mode"]="verify";
  boundary["execution"]["price_delta"]=.15;
  boundary["execution"]["priority_policy"]="penalized_shortfall";
  boundary["generators"][0]["startup_cost"]={10,20,30};
  boundary["generators"][0]["price_setting"][0]=0;
  boundary["storage"][0]["charge_min_mw"]=1;
  boundary["storage"][0]["discharge_min_mw"]=2;
  boundary["storage"][0]["price_setting"][0]=0;
  auto trade=record("trades");trade["gateway_kind"]="ac_branch";trade["gateway"]=1;
  trade["max_mw"]=series(80);trade["max_mwh"]=2000;
  trade["original_min_mwh"]=trade["adjusted_min_mwh"]=300;
  trade["scuc_fee"]=.2;trade["sced_fee"]=.3;trade["lmp_fee"]=.7;
  boundary["trades"].push_back(trade);
  auto section=record("sections");section["members"]=json::array({{{"branch",1},{"coefficient",.4}}});
  section["min_mw"]=series(-100);section["max_mw"]=series(100);boundary["sections"].push_back(section);
  for(const std::string policy:{"omit_unlisted","retain_sced"}) {
    boundary["execution"]["lmp_renewable_priority"]=policy;
    const auto result=run_southern_day_ahead_market(boundary);require_schedule(result);
    REQUIRE(result["lmp"]["assembly_template"]["derived_from"]=="sced");
    REQUIRE(result["lmp"]["assembly_template"]["matrix_comparison"]=="exact_match");
    REQUIRE(result["lmp"]["price_consistency"]["max_dual_difference"]==0);
  }
}

TEST_CASE("Every admitted rule field round trips through typed daily overlays", "[southern_market][operation][boundary_rules]") {
  auto b=make_southern_market_demo();
  auto primary=record("primary_groups");primary["generators"]={1};b["primary_groups"].push_back(primary);
  auto group=record("groups");group["generators"]={1};group["max_online"]=series(1);group["max_mw"]=series(10000);group["max_mwh"]=1e6;b["groups"].push_back(group);
  auto section=record("sections");section["members"]=json::array({{{"branch",1},{"coefficient",1}}});section["min_mw"]=series(-10000);section["max_mw"]=series(10000);b["sections"].push_back(section);
  auto trade=record("trades");trade["gateway_kind"]="ac_branch";trade["gateway"]=1;trade["max_mw"]=series(10000);trade["max_mwh"]=1e6;b["trades"].push_back(trade);
  auto dc=record("dc_links");dc["from_bus"]=1;dc["to_bus"]=2;dc["from_hub"]=-1;dc["to_hub"]=-1;dc["available"]=std::vector<int>(98,1);dc["max_mw"]=series(100);dc["ramp_up_mw"]=series(100);dc["ramp_down_mw"]=series(100);b["dc_links"].push_back(dc);
  auto plan=record("external_schedules");plan["bus"]=1;b["external_schedules"].push_back(plan);
  const auto effective=validate_southern_market(b);
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c=make_market_operation(b,c)["config"];
  std::set<std::pair<std::string,std::string>> seen;
  json edits=json::array();
  const auto catalog=southern_market_boundary_catalog();
  for(const auto& category:catalog.at("items")) for(const auto& f:category.at("fields")) {
    const std::string table=f.at("table"),field=f.at("field");
    if(!seen.emplace(table,field).second)continue;
    REQUIRE_FALSE(b.at(table).empty());
    const auto& row=b.at(table)[0];const bool is_series=row.at(field).is_array();
    edits.push_back({{"table",table},{"id",row.at("id")},{"field",field},{"first_slot",0},{"last_slot",is_series?0:97},{"value",is_series?row.at(field)[0]:row.at(field)},{"reason","field round trip"}});
  }
  c["days"][0]["boundary_overrides"]=edits;
  const auto preview=preview_market_operation_boundary(b,c,0);
  for(const auto& [table,field]:seen) {
    if (b[table][0][field].is_array()) {
      for (int t = 0; t < 96; ++t) {
        REQUIRE(preview["authored"][table][0][field][t]==b[table][0][field][t]);
        REQUIRE(preview["effective"][table][0][field][t]==effective[table][0][field][t]);
      }
    } else {
      REQUIRE(preview["authored"][table][0][field]==b[table][0][field]);
      REQUIRE(preview["effective"][table][0][field]==effective[table][0][field]);
    }
  }
  REQUIRE(seen.size()>60);
}
