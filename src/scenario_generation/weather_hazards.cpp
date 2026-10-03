#include "hacdcpf/analysis/weather_hazards.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <stdexcept>

namespace hacdcpf::analysis {
namespace {
using json = nlohmann::json;
template<class T> struct Field {
  const char* key; const char* label; const char* unit;
  double T::*member; double min; double max; bool advanced;
};
const std::vector<Field<RainstormOptions>> rain_fields{
  {"start_hr", "降雨开始", "h", &RainstormOptions::start_hr, 0, 47, false},
  {"duration_hr", "降雨历时", "h", &RainstormOptions::duration_hr, 1, 48, false},
  {"total_mm", "累计降雨量", "mm", &RainstormOptions::total_mm, 0, 2000, false},
  {"drainage_mm_hr", "等效排水能力", "mm/h", &RainstormOptions::drainage_mm_hr, 0, 200, false},
  {"rainwater_resistivity_ohm_m", "雨水电阻率", "Ω·m", &RainstormOptions::rainwater_resistivity_ohm_m, 1, 10000, true},
  {"altitude_m", "绝缘子海拔", "m", &RainstormOptions::altitude_m, 0, 5000, true},
  {"insulator_surface_c", "绝缘子表面电阻拟合系数", "演示标定", &RainstormOptions::insulator_surface_c, .00001, 1, true},
  {"insulator_pressure_exponent", "闪络气压指数", "无量纲", &RainstormOptions::insulator_pressure_exponent, 0, 2, true},
  {"transformer_oil_initial_ppm", "变压器初始油含水", "ppm", &RainstormOptions::transformer_oil_initial_ppm, 0, 100, true},
  {"transformer_paper_initial_pct", "变压器初始纸含水", "%干重", &RainstormOptions::transformer_paper_initial_pct, 0, 10, true},
  {"transformer_oil_a", "油受潮演示拟合系数", "ppm/(min·(mm/min)^0.949)", &RainstormOptions::transformer_oil_a, 0, 10, true},
  {"transformer_paper_a", "纸受潮演示拟合系数", "%/min", &RainstormOptions::transformer_paper_a, 0, 1, true},
  {"transformer_oil_shutdown_ppm", "油含水保护停运判据", "ppm", &RainstormOptions::transformer_oil_shutdown_ppm, 1, 1000, true},
  {"transformer_paper_shutdown_pct", "纸含水保护停运判据", "%干重", &RainstormOptions::transformer_paper_shutdown_pct, .1, 30, true},
  {"repair_hr", "退水后修复时长", "h", &RainstormOptions::repair_hr, 1, 720, false},
  {"peak_fraction", "雨峰相对位置", "比例", &RainstormOptions::peak_fraction, .05, .95, true},
  {"shape_b_hr", "雨型平滑时间", "h", &RainstormOptions::shape_b_hr, .05, 10, true},
  {"shape_n", "雨型衰减指数", "无量纲", &RainstormOptions::shape_n, .05, .95, true},
  {"runoff", "产流系数", "比例", &RainstormOptions::runoff, 0, 1, true},
  {"severity_variation", "候选雨量相对扰动", "比例", &RainstormOptions::severity_variation, 0, .9, true}
};
const std::vector<Field<LightningOptions>> lightning_fields{
  {"start_hr", "雷暴开始", "h", &LightningOptions::start_hr, 0, 47, false},
  {"duration_hr", "雷暴历时", "h", &LightningOptions::duration_hr, 1, 48, false},
  {"density_km2_hr", "地闪密度", "次/(km²·h)", &LightningOptions::density_km2_hr, 0, 1000, false},
  {"critical_current_ka", "等效闪络临界电流", "kA", &LightningOptions::critical_current_ka, 1, 500, false},
  {"permanent_fraction", "跳闸后永久损坏概率", "比例", &LightningOptions::permanent_fraction, 0, 1, false},
  {"repair_hr", "雷暴结束后修复时长", "h", &LightningOptions::repair_hr, 1, 720, false},
  {"collection_width_m", "等效雷击收集宽度", "m", &LightningOptions::collection_width_m, 0, 1000, true},
  {"fallback_length_km", "缺失线路长度时的假设", "km", &LightningOptions::fallback_length_km, .001, 100, true},
  {"median_current_ka", "雷电流中值", "kA", &LightningOptions::median_current_ka, 1, 500, true},
  {"log_current_sigma", "电流对数标准差", "无量纲", &LightningOptions::log_current_sigma, .05, 2, true},
  {"transient_duration_hr", "暂时跳闸等效停运时间", "h", &LightningOptions::transient_duration_hr, 1, 48, true},
  {"severity_variation", "候选地闪密度相对扰动", "比例", &LightningOptions::severity_variation, 0, .9, true}
};
template<class T> json schema(const std::vector<Field<T>>& fields) {
  json out = json::array(); T defaults;
  for (const auto& f : fields) out.push_back({{"key",f.key},{"label",f.label},
      {"unit",f.unit},{"default",defaults.*(f.member)},{"min",f.min},
      {"max",f.max},{"advanced",f.advanced}});
  return out;
}
template<class T> void validate(const T& v, const std::vector<Field<T>>& fields) {
  for (const auto& f : fields) {
    const double value = v.*(f.member);
    if (!std::isfinite(value) || value < f.min || value > f.max)
      throw std::invalid_argument(std::string("Invalid weather hazard parameter: ")+f.key);
  }
}
template<class T> void parse(T& v, const json& j, const std::vector<Field<T>>& fields) {
  if (!j.is_object()) throw std::invalid_argument("weather hazard parameters must be an object");
  for (const auto& [key, value] : j.items()) {
    const auto it = std::find_if(fields.begin(),fields.end(),[&](const auto& f){return key == f.key;});
    if (it == fields.end() || !value.is_number())
      throw std::invalid_argument("Unknown or nonnumeric weather hazard parameter: "+key);
    v.*(it->member) = value.get<double>();
  }
  validate(v,fields);
}
template<class T> json parameters(const T& v, const std::vector<Field<T>>& fields) {
  json j=json::object(); for (const auto& f:fields) j[f.key]=v.*(f.member); return j;
}
struct Branch {
  ResilienceBranchKind kind;
  int index;
  double length;
  std::string line_type;
  double cable_entry_m;
  double insulator_ref_kv;
  double operating_kv;
  bool assumed_type;
};
std::string classified_type(std::string type, ResilienceBranchKind kind) {
  std::transform(type.begin(), type.end(), type.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (type == "cable" || type.find("电缆") != std::string::npos ||
      type.find("underground") != std::string::npos) return "cable";
  if (type == "overhead" || type.find("架空") != std::string::npos) return "overhead";
  if (type == "transformer_equivalent") return type;
  return kind == ResilienceBranchKind::DC ? "cable" : "overhead";
}
bool type_assumed(std::string type) {
  std::transform(type.begin(), type.end(), type.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return type != "cable" && type != "overhead" &&
    type != "transformer_equivalent" &&
    type.find("电缆") == std::string::npos &&
    type.find("underground") == std::string::npos &&
    type.find("架空") == std::string::npos;
}
double cdf(double x) { return .5 * std::erfc(-x / std::sqrt(2.0)); }
} // namespace

json weather_hazard_schema() {
  return json::array({
    {{"id","typhoon"},{"label","台风"},{"fields",json::array()}},
    {{"id","rainstorm"},{"label","暴雨内涝"},{"fields",schema(rain_fields)},
     {"description","Chicago 形状雨型与零维积水；只对已标记的电缆附件、架空线绝缘子和支路关联变压器计算停运。系数为演示标定。"}},
    {{"id","lightning"},{"label","雷暴雷击"},{"fields",schema(lightning_fields)},
     {"description","地闪泊松过程与等效电流阈值；区分暂时跳闸和永久损坏。小时尺度，不解析雷电暂态或秒级重合闸。"}}
  });
}
WeatherHazardOptions weather_hazard_options_from_json(const json& r) {
  WeatherHazardOptions o;
  o.hazard_type=r.value("hazard_type",o.hazard_type);
  if (o.hazard_type!="typhoon" && o.hazard_type!="rainstorm" && o.hazard_type!="lightning")
    throw std::invalid_argument("Unsupported resilience hazard_type: "+o.hazard_type);
  if(r.contains("rainstorm")) parse(o.rainstorm,r.at("rainstorm"),rain_fields);
  if(r.contains("lightning")) parse(o.lightning,r.at("lightning"),lightning_fields);
  return o;
}
void validate_weather_hazard_options(const WeatherHazardOptions& o, int horizon) {
  if(o.hazard_type=="typhoon") return;
  double start=0,duration=0;
  if(o.hazard_type=="rainstorm") {validate(o.rainstorm,rain_fields); start=o.rainstorm.start_hr; duration=o.rainstorm.duration_hr;}
  else if(o.hazard_type=="lightning") {validate(o.lightning,lightning_fields); start=o.lightning.start_hr; duration=o.lightning.duration_hr;}
  else throw std::invalid_argument("Unsupported resilience hazard_type: "+o.hazard_type);
  if(horizon<1 || horizon>8760 || start+duration>horizon)
    throw std::invalid_argument("Weather event must fit within the scenario horizon");
}

WeatherHazardScenario generate_weather_hazard(const HybridPowerSystem& sys,
    const WeatherHazardOptions& o,int horizon,unsigned seed) {
  validate_weather_hazard_options(o,horizon);
  if(o.hazard_type=="typhoon") throw std::invalid_argument("Use the typhoon generator for typhoon events");
  WeatherHazardScenario out;
  std::vector<Branch> branches;
  std::set<std::pair<int,int>> identities;
  std::map<int,double> ac_kv;
  for (const auto& bus:sys.ac.buses) ac_kv[bus.index]=bus.base_kv;
  for(const auto& b:sys.ac.branches) {
    if(!identities.emplace(0,b.index).second)
      throw std::invalid_argument("Duplicate stable AC branch identity in weather hazard input");
    if(!b.in_service) continue;
    const auto type=classified_type(b.line_type,ResilienceBranchKind::AC);
    if(type=="transformer_equivalent") continue;
    const auto kv=ac_kv.count(b.from_bus)?ac_kv.at(b.from_bus):0.0;
    branches.push_back({ResilienceBranchKind::AC,b.index,b.length_km,type,
      b.weather_cable_entry_height_m,b.weather_insulator_wet_ref_kv,
      kv/std::sqrt(3.0),type_assumed(b.line_type)});
  }
  for(const auto& b:sys.dc.branches) {
    if(!identities.emplace(1,b.index).second)
      throw std::invalid_argument("Duplicate stable DC branch identity in weather hazard input");
    if(!b.in_service) continue;
    branches.push_back({ResilienceBranchKind::DC,b.index,b.length_km,
      classified_type(b.line_type,ResilienceBranchKind::DC),
      b.weather_cable_entry_height_m,0.0,b.base_kv,type_assumed(b.line_type)});
  }
  // Stable IDs determine random draw ordering; vector reordering does not change damage.
  std::sort(branches.begin(),branches.end(),[](const auto& a,const auto& b){
    return std::pair{a.kind,a.index}<std::pair{b.kind,b.index};
  });
  std::mt19937 rng(seed);
  const auto uniform=[&](){return (static_cast<double>(rng())+.5)/4294967296.0;};
  const bool rain=o.hazard_type=="rainstorm";
  const double variation=rain?o.rainstorm.severity_variation:o.lightning.severity_variation;
  const double scale=1+variation*(2*uniform()-1);
  json profiles=json::array(), risks=json::array(), effects=json::array();
  std::vector<double> hours(horizon),intensity(horizon,0),depth(horizon,0);
  for(int t=0;t<horizon;++t) hours[t]=t;
  double safe_start=0;
  std::vector<double> fine_depth, fine_rain;
  constexpr int subdivisions=12;
  constexpr double dt=1.0/subdivisions;
  if(rain) {
    const auto& p=o.rainstorm;
    std::vector<double> weights(horizon*subdivisions,0);
    double norm=0;
    for(std::size_t k=0;k<weights.size();++k) {
      const double t=(k+.5)*dt-p.start_hr;
      if(t<0 || t>=p.duration_hr) continue;
      const double peak=p.peak_fraction*p.duration_hr;
      const double tau=t<peak?(peak-t)/p.peak_fraction:(t-peak)/(1-p.peak_fraction);
      weights[k]=((1-p.shape_n)*tau+p.shape_b_hr)/std::pow(tau+p.shape_b_hr,1+p.shape_n);
      norm+=weights[k]*dt;
    }
    double water_mm=0;
    safe_start=p.start_hr+p.duration_hr;
    fine_depth.resize(weights.size());
    fine_rain.resize(weights.size());
    for(std::size_t k=0;k<weights.size();++k) {
      const double rate=norm>0?weights[k]*p.total_mm*scale/norm:0;
      fine_rain[k]=rate/60.0; // book formulas use mm/min
      water_mm=std::max(0.0,water_mm+(p.runoff*rate-p.drainage_mm_hr)*dt);
      fine_depth[k]=water_mm/1000;
      const auto t=k/subdivisions;
      intensity[t]+=rate*dt;
      depth[t]=std::max(depth[t],fine_depth[k]);
    }
    out.peak_intensity=*std::max_element(depth.begin(),depth.end());
    profiles.push_back({{"id","rainfall_mm_hr"},{"label","小时平均降雨强度"},{"unit","mm/h"},{"time_hr",hours},{"values",intensity}});
    profiles.push_back({{"id","water_depth_m"},{"label","小时最大积水深度"},{"unit","m"},{"time_hr",hours},{"values",depth}});
  } else {
    const auto& p=o.lightning;
    safe_start=p.start_hr+p.duration_hr;
    for(int t=0;t<horizon;++t) intensity[t]=p.density_km2_hr*scale*std::max(0.0,std::min(t+1.0,safe_start)-std::max(double(t),p.start_hr));
    out.peak_intensity=p.density_km2_hr*scale;
    profiles.push_back({{"id","lightning_density_km2_hr"},{"label","小时平均地闪密度"},{"unit","次/(km²·h)"},{"time_hr",hours},{"values",intensity}});
  }
  const auto recede_after = [&](double entry_m) {
    const std::size_t after_rain=std::min(fine_depth.size(),
      static_cast<std::size_t>(std::ceil(safe_start/dt)));
    for(std::size_t k=after_rain;k<fine_depth.size();++k)
      if(fine_depth[k]<entry_m) return (k+1)*dt;
    if(fine_depth.empty() || fine_depth.back()<entry_m) return safe_start;
    if(o.rainstorm.drainage_mm_hr<=0)
      throw std::invalid_argument("Flooded cable accessory has no finite drainage/restoration time");
    return horizon+(fine_depth.back()-entry_m)*1000/o.rainstorm.drainage_mm_hr;
  };
  int fallback_lengths=0, exposed_ac=0, exposed_dc=0, failed_ac=0, failed_dc=0;
  int exposed_overhead=0, exposed_cable=0, cable_shutdowns=0, insulator_trips=0;
  int transformer_exposed=0, transformer_shutdowns=0, assumed_types=0;
  int temporary_trips=0, permanent_failures=0, protective_shutdowns=0;
  std::set<int> faulted_ac_branch_ids;
  const auto emit=[&](ResilienceBranchKind kind,int branch_id,
      std::string equipment_type,int equipment_id,std::string cause,
      std::string mode,double physical_onset,double access,double restore,double hands_on) {
    if(physical_onset<0 || physical_onset>=horizon) return;
    const double onset=std::floor(physical_onset);
    DistributionResilienceFault f;
    f.branch_kind=kind; f.branch_index=branch_id;
    f.ac_branch_index=kind==ResilienceBranchKind::AC?branch_id:0;
    f.outage_start_hr=onset;
    f.repair_duration_hr=std::max(1.0,restore-onset);
    f.equipment_type=equipment_type; f.equipment_index=equipment_id;
    f.failure_cause=cause;
    f.name=cause+" "+std::to_string(equipment_id);
    out.faults.push_back(f);
    if(kind==ResilienceBranchKind::AC) faulted_ac_branch_ids.insert(branch_id);
    if(kind==ResilienceBranchKind::AC) ++failed_ac; else ++failed_dc;
    if(mode=="temporary_trip") ++temporary_trips;
    else if(mode=="protective_shutdown") ++protective_shutdowns;
    else ++permanent_failures;
    effects.push_back({{"branch_type",to_string(kind)},{"branch_index",branch_id},
      {"equipment_type",equipment_type},{"equipment_index",equipment_id},
      {"failure_cause",cause},{"failure_mode",mode},
      {"physical_onset_hr",physical_onset},{"start_hr",onset},
      {"safe_access_hr",access},{"safe_access_within_horizon",access<horizon},
      {"restoration_time_hr",restore},{"outage_duration_hr",f.repair_duration_hr},
      {"hands_on_repair_hr",hands_on}});
  };
  for(const auto& b:branches) {
    if(b.kind==ResilienceBranchKind::AC) ++exposed_ac; else ++exposed_dc;
    if(b.line_type=="overhead") ++exposed_overhead; else ++exposed_cable;
    if(b.assumed_type) ++assumed_types;
    double probability=0,physical_onset=-1,restore=0,access=0,hands_on=0;
    std::string mode="permanent",equipment="",cause="";
    if(rain && b.line_type=="cable") {
      equipment="cable_accessory";
      if(b.cable_entry_m>=0) {
        for(std::size_t k=0;k<fine_depth.size();++k)
          if(fine_depth[k]>=b.cable_entry_m && fine_depth[k]>0) {
            physical_onset=(k+1)*dt; break;
          }
        probability=physical_onset>=0?1:0; // deterministic screening indicator
        if(physical_onset>=0) {
          access=std::max(safe_start,recede_after(b.cable_entry_m));
          hands_on=o.rainstorm.repair_hr;
          restore=std::ceil(access+hands_on);
          cause="电缆附件受淹保护停运";
          mode="protective_shutdown";
          ++cable_shutdowns;
        }
      }
    } else if(rain && b.line_type=="overhead") {
      equipment="insulator";
      if(b.insulator_ref_kv>0 && b.operating_kv>0) {
        const auto& p=o.rainstorm;
        const double pressure_ratio=std::pow(1-p.altitude_m/44330.0,5.25);
        const double ref=std::pow(1.02,-0.44);
        for(std::size_t k=0;k<fine_rain.size();++k) {
          const double a=fine_rain[k];
          if(a<=0) continue;
          const double wet_kv=b.insulator_ref_kv*
            std::exp(p.insulator_surface_c*p.rainwater_resistivity_ohm_m*
              (std::pow(a+0.02,-0.44)-ref))*
            std::pow(pressure_ratio,p.insulator_pressure_exponent);
          if(wet_kv<=b.operating_kv) { physical_onset=(k+1)*dt; break; }
        }
        probability=physical_onset>=0?1:0;
        if(physical_onset>=0) {
          mode="temporary_trip"; access=physical_onset;
          restore=std::ceil(physical_onset+1.0);
          cause="绝缘子雨闪暂时跳闸";
          ++insulator_trips;
        }
      }
    } else if(!rain && b.line_type=="overhead") {
      const auto& p=o.lightning;
      const bool fallback=!std::isfinite(b.length)||b.length<=0;
      if(fallback) ++fallback_lengths;
      const double area=(fallback?p.fallback_length_km:b.length)*p.collection_width_m/1000;
      const double exceedance=1-cdf(std::log(p.critical_current_ka/p.median_current_ka)/p.log_current_sigma);
      const double rate=p.density_km2_hr*scale*area*exceedance;
      probability=-std::expm1(-rate*p.duration_hr);
      const double wait=rate>0?-std::log(uniform())/rate:INFINITY;
      if(wait<p.duration_hr) {
        physical_onset=p.start_hr+wait;
        mode=uniform()<p.permanent_fraction?"permanent":"temporary_trip";
        access=mode=="permanent"?safe_start:physical_onset;
        hands_on=mode=="permanent"?p.repair_hr:0;
        restore=mode=="permanent"?std::ceil(access+hands_on)
                                  :std::ceil(physical_onset+p.transient_duration_hr);
        equipment="overhead_line";
        cause=mode=="permanent"?"架空线雷击永久故障":"架空线雷击暂时跳闸";
      }
    }
    out.peak_failure_probability=std::max(out.peak_failure_probability,probability);
    risks.push_back({{"branch_type",to_string(b.kind)},{"branch_index",b.index},
      {"line_type",b.line_type},{"type_assumed",b.assumed_type},
      {"equipment_type",equipment},{"modelled",rain?
        (b.line_type=="cable"?b.cable_entry_m>=0:b.insulator_ref_kv>0):b.line_type=="overhead"},
      {"event_failure_probability",rain?json(nullptr):json(probability)},
      {"screening_triggered",rain?json(probability>0):json(nullptr)}});
    if(physical_onset>=0) emit(b.kind,b.index,equipment,b.index,cause,mode,
      physical_onset,access,restore,hands_on);
  }
  if(rain) {
    const auto& p=o.rainstorm;
    const double pressure_ratio=std::pow(1-p.altitude_m/44330.0,5.25);
    for(const auto& tr:sys.ac.transformers_2w) {
      if(!tr.in_service || !tr.weather_moisture_vulnerable) continue;
      if(tr.source_branch_idx<=0 || !identities.count({0,tr.source_branch_idx}))
        throw std::invalid_argument("Weather-vulnerable transformer needs a valid source_branch_idx");
      if(faulted_ac_branch_ids.count(tr.source_branch_idx))
        throw std::invalid_argument("Weather-vulnerable transformer shares a branch with another outage; use a dedicated transformer-equivalent branch");
      ++transformer_exposed;
      double oil=p.transformer_oil_initial_ppm;
      double paper=p.transformer_paper_initial_pct;
      double physical_onset=-1;
      for(std::size_t k=0;k<fine_rain.size();++k) {
        const double a=fine_rain[k];
        if(a<=0) continue;
        const double minutes=dt*60;
        oil+=p.transformer_oil_a*std::pow(a,0.949)*minutes*
          std::pow(pressure_ratio,p.insulator_pressure_exponent);
        paper+=p.transformer_paper_a*std::exp(1.323*a)*minutes*
          std::pow(pressure_ratio,p.insulator_pressure_exponent);
        if(!std::isfinite(paper))
          throw std::invalid_argument("Transformer moisture demonstration exceeded finite range");
        if(oil>=p.transformer_oil_shutdown_ppm ||
           paper>=p.transformer_paper_shutdown_pct) {
          physical_onset=(k+1)*dt; break;
        }
      }
      risks.push_back({{"branch_type","AC"},{"branch_index",tr.source_branch_idx},
        {"equipment_type","transformer_2w"},{"equipment_index",tr.index},
        {"oil_moisture_ppm",oil},{"paper_moisture_pct",paper},
        {"screening_triggered",physical_onset>=0},
        {"event_failure_probability",nullptr}});
      if(physical_onset>=0) {
        ++transformer_shutdowns;
        out.peak_failure_probability=1;
        emit(ResilienceBranchKind::AC,tr.source_branch_idx,"transformer_2w",tr.index,
          "变压器绝缘受潮保护停运","protective_shutdown",physical_onset,safe_start,
          std::ceil(safe_start+p.repair_hr),p.repair_hr);
      }
    }
  }
  json limits=json::array({
    "Rainstorm moisture and wet-flashover coefficients and device heights are synthetic demonstration assumptions, not field-calibrated breakdown limits.",
    "Cable-body inundation is not converted to an electrical failure probability; only authored flood-vulnerable accessories cause protective branch shutdown.",
    "Shared zero-dimensional rain and water depth is a demonstration footprint, not a geographical flood map.",
    "At most one outage per branch per scenario. Hourly outage windows conservatively round onset down and restoration up; durations include waiting for safe access. No repair-crew optimization.",
    "Demand and renewable profiles retain the base stochastic model; no additional weather-driven derating is applied."});
  limits.push_back(rain?"Book eqs. 2.47-2.54 are used as weather-screening forms; variable rainfall is integrated in five-minute steps, and threshold crossing means protective outage, not certified dielectric breakdown. The 2025 Williams cable damage percentage is context only, not sampled as a failure probability.":"Lightning applies only to classified overhead lines. Poisson effective exposure and lognormal current threshold do not resolve electromagnetic transients or second-scale reclosing.");
  out.evidence={{"hazard_type",o.hazard_type},{"model_scope",rain?"rainstorm-asset-screening-cable-insulator-transformer-v2":"lightning-overhead-poisson-current-threshold-v2"},
    {"model_limitations",limits},{"parameters",rain?parameters(o.rainstorm,rain_fields):parameters(o.lightning,lightning_fields)},
    {"seed",seed},{"severity_scale",scale},{"time_step_hr",1.0},
    {"probability_scope",rain?"deterministic-screening-indicator-not-a-probability":"conditional-on-configured-hazard"},{"profiles",profiles},
    {"asset_risks",risks},{"fault_effects",effects},{"fallback_length_count",fallback_lengths},
    {"affected_equipment",{{"unit","in-service branch components"},
       {"exposed_ac_branches",exposed_ac},{"exposed_dc_branches",exposed_dc},
       {"exposed_overhead_branches",exposed_overhead},{"exposed_cable_branches",exposed_cable},
       {"assumed_line_types",assumed_types},
       {"failed_ac_branches",failed_ac},{"failed_dc_branches",failed_dc},
       {"cable_accessory_shutdowns",cable_shutdowns},{"insulator_flashover_trips",insulator_trips},
       {"exposed_transformers",transformer_exposed},{"transformer_moisture_shutdowns",transformer_shutdowns},
       {"temporary_trip_branches",temporary_trips},{"permanent_failure_branches",permanent_failures},
       {"protective_shutdown_branches",protective_shutdowns},
       {"fault_count",static_cast<int>(out.faults.size())}}},
    {"cable_damage_reference",rain?json{{"source","Williams et al. (2025), doi:10.1111/jfr3.70045"},
      {"depth_m",3.0},{"damage_percent_range",json::array({9,40})},
      {"meaning","repair-cost damage ratio under New Zealand expert elicitation; not outage probability"}}:json(nullptr)},
    {"peak_intensity",out.peak_intensity},{"intensity_unit",rain?"m":"次/(km²·h)"},
    {"intensity_label",rain?"最大积水深度":"地闪密度"},
    {"hazard_end_hr",rain?o.rainstorm.start_hr+o.rainstorm.duration_hr:o.lightning.start_hr+o.lightning.duration_hr}};
  return out;
}
} // namespace hacdcpf::analysis
