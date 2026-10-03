#include "hacdcpf/resilience/resilience_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace hacdcpf::analysis {
namespace {
constexpr double kEps = 1e-9;

ResilienceMetricDefinition def(std::string id, std::string zh, std::string en,
                               std::string symbol, std::string phase,
                               std::string topic, std::string formula,
                               std::string unit, std::string direction,
                               std::string scope, std::string availability,
                               std::vector<std::string> inputs,
                               std::vector<std::string> notes = {},
                               std::vector<std::string> limitations = {}) {
  return {std::move(id), std::move(zh), std::move(en), std::move(symbol),
          std::move(phase), std::move(topic), std::move(formula),
          std::move(unit), std::move(direction), std::move(scope),
          std::move(availability), std::move(inputs), std::move(notes),
          std::move(limitations)};
}

const ResilienceMetricDefinition* lookup(const std::string& id) {
  const auto& all = resilience_metric_catalog();
  const auto it = std::find_if(all.begin(), all.end(),
                               [&](const auto& item) { return item.id == id; });
  return it == all.end() ? nullptr : &*it;
}

ResilienceMetricResult result_for(const ResilienceMetricDefinition& item) {
  ResilienceMetricResult result;
  result.id = item.id;
  result.unit = item.unit;
  result.formula_ref = item.formula_ref;
  result.direction = item.direction;
  result.scope = item.calculation_scope;
  result.source_fields = item.required_inputs;
  result.source_notes = item.source_notes;
  result.limitations = item.limitations;
  return result;
}

bool valid_run(const ResilienceMetricRun& run) {
  if (run.steps.empty()) return false;
  double previous = -1.0;
  for (const auto& s : run.steps) {
    if (!std::isfinite(s.hour) || !std::isfinite(s.demand_mw) ||
        !std::isfinite(s.served_mw) || !std::isfinite(s.shed_mw) ||
        !std::isfinite(s.restoration_ratio) || s.hour < 0.0 || s.hour <= previous ||
        s.demand_mw < 0.0 || s.served_mw < 0.0 || s.shed_mw < 0.0 ||
        s.served_mw > s.demand_mw + 1e-6 ||
        std::abs(s.demand_mw - s.served_mw - s.shed_mw) > 1e-6 * std::max(1.0, s.demand_mw) ||
        s.restoration_ratio < 0.0 || s.restoration_ratio > 1.0 ||
        s.active_faults < 0 || s.repaired_faults < 0) return false;
    if (s.duration_hr && (!std::isfinite(*s.duration_hr) || *s.duration_hr <= 0.0)) return false;
    previous = s.hour;
  }
  for (std::size_t i = 1; i < run.steps.size(); ++i)
    if (run.steps[i-1].duration_hr && std::abs(run.steps[i-1].hour +
        *run.steps[i-1].duration_hr - run.steps[i].hour) > 1e-6) return false;
  for (const auto t : {run.disaster_end_hr, run.recovery_start_hr})
    if (t && (!std::isfinite(*t) || *t < 0.0)) return false;
  if (run.disaster_end_hr && run.recovery_start_hr &&
      *run.recovery_start_hr < *run.disaster_end_hr) return false;
  return true;
}

std::optional<ResilienceMetricStep> at(const std::vector<ResilienceMetricStep>& v,
                                       double hour) {
  if (v.empty() || hour < v.front().hour - kEps || hour > v.back().hour + kEps)
    return std::nullopt;
  if (hour <= v.front().hour + kEps) return v.front();
  for (std::size_t i = 1; i < v.size(); ++i) {
    if (hour > v[i].hour + kEps) continue;
    const auto& a = v[i - 1];
    const auto& b = v[i];
    const double span = b.hour - a.hour;
    if (span <= kEps) return b;
    const double t = std::clamp((hour - a.hour) / span, 0.0, 1.0);
    return ResilienceMetricStep{
        hour, a.demand_mw + t * (b.demand_mw - a.demand_mw),
        a.served_mw + t * (b.served_mw - a.served_mw),
        a.shed_mw + t * (b.shed_mw - a.shed_mw),
        a.restoration_ratio + t * (b.restoration_ratio - a.restoration_ratio),
        0, 0};
  }
  return v.back();
}

std::optional<double> recovery_start(const ResilienceMetricRun& run,
                                     const std::vector<ResilienceMetricStep>& v) {
  if (run.recovery_start_hr) return run.recovery_start_hr;
  if (!run.disaster_end_hr) return std::nullopt;
  for (std::size_t i = 1; i < v.size(); ++i) {
    if (v[i].hour + kEps < *run.disaster_end_hr) continue;
    if (v[i].restoration_ratio > v[i - 1].restoration_ratio + kEps ||
        v[i].repaired_faults > v[i - 1].repaired_faults)
      return std::max(*run.disaster_end_hr, v[i - 1].hour);
  }
  return std::nullopt;
}

double ratio_area(const std::vector<ResilienceMetricStep>& v, double start,
                  double end) {
  if (end <= start + kEps) return 0.0;
  std::vector<double> points{start, end};
  for (const auto& s : v)
    if (s.hour > start + kEps && s.hour < end - kEps) points.push_back(s.hour);
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end(),
                           [](double a, double b) { return std::abs(a - b) < kEps; }),
               points.end());
  double area = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto a = at(v, points[i - 1]);
    const auto b = at(v, points[i]);
    if (a && b) area += 0.5 * (a->restoration_ratio + b->restoration_ratio) *
                         (points[i] - points[i - 1]);
  }
  return area;
}

std::optional<double> crossing(const std::vector<ResilienceMetricStep>& v,
                               double start, double target) {
  auto previous = at(v, start);
  if (!previous) return std::nullopt;
  if (previous->restoration_ratio + kEps >= target) return start;
  for (const auto& current : v) {
    if (current.hour <= start + kEps) continue;
    if (current.restoration_ratio + kEps < target) {
      previous = current;
      continue;
    }
    const double delta = current.restoration_ratio - previous->restoration_ratio;
    const double t = std::abs(delta) < kEps
                         ? 1.0
                         : std::clamp((target - previous->restoration_ratio) / delta,
                                      0.0, 1.0);
    return previous->hour + t * (current.hour - previous->hour);
  }
  return std::nullopt;
}

void unavailable(ResilienceMetricResult& r, std::string reason,
                 std::vector<std::string> missing = {}) {
  r.status = ResilienceMetricStatus::Unavailable;
  r.reason_code = std::move(reason);
  r.missing_dependencies = std::move(missing);
}

// Solver powers represent complete intervals, including the final interval.
// Point-sampled curves have no implied last interval and are integrated linearly.
template<class F>
double integrate(const std::vector<ResilienceMetricStep>& v, F value) {
  double sum = 0.0;
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (v[i].duration_hr) sum += value(v[i]) * *v[i].duration_hr;
    else if (i + 1 < v.size())
      sum += 0.5 * (value(v[i]) + value(v[i+1])) * (v[i+1].hour - v[i].hour);
  }
  return sum;
}

bool operational_metric(const ResilienceMetricRun& run,
                        const std::vector<ResilienceMetricStep>& v,
                        ResilienceMetricResult& r) {
  if (r.id.rfind("run.", 0) != 0) return false;
  const auto key = r.id.substr(4);
  r.assumptions.push_back("Solver intervals use sum(power * duration_hr), including the last interval; point samples use trapezoids without extrapolation.");
  const double duration = integrate(v, [](const auto&) { return 1.0; });
  const double demand = integrate(v, [](const auto& s) { return s.demand_mw; });
  const double shed = integrate(v, [](const auto& s) { return s.shed_mw; });
  const double served = integrate(v, [](const auto& s) { return s.served_mw; });
  double peak = 0.0, minimum = 1.0, faults = 0.0, repaired = 0.0;
  bool has_demand = false;
  for (const auto& s : v) {
    peak = std::max(peak, s.shed_mw);
    if (s.demand_mw > kEps) { minimum = std::min(minimum, s.served_mw / s.demand_mw); has_demand = true; }
    faults = std::max(faults, double(s.active_faults));
    repaired = std::max(repaired, double(s.repaired_faults));
  }
  auto divide = [&](double a, double b) {
    if (b > kEps) r.value = a / b;
    else { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "ZERO_DENOMINATOR"; }
  };
  r.status = ResilienceMetricStatus::Computed;
  if (key == "duration") r.value = duration;
  else if (key == "demand_energy") r.value = demand;
  else if (key == "served_energy") r.value = served;
  else if (key == "ens") r.value = shed;
  else if (key == "energy_supply_ratio") divide(served, demand);
  else if (key == "energy_loss_ratio") divide(shed, demand);
  else if (key == "mean_shed") divide(shed, duration);
  else if (key == "peak_shed") r.value = peak;
  else if (key == "minimum_supply_ratio") divide(minimum, has_demand ? 1.0 : 0.0);
  else if (key == "final_supply_ratio") divide(v.back().served_mw, v.back().demand_mw);
  else if (key == "equivalent_outage_hours") divide(shed * duration, demand);
  else if (key == "interrupted_hours" || key == "below_90_hours") {
    if (std::any_of(v.begin(), v.end(), [](const auto& s) { return !s.duration_hr; }))
      unavailable(r, "INTERVAL_DURATIONS_REQUIRED", {"steps.duration_hr"});
    else r.value = integrate(v, [&](const auto& s) {
      return (key == "interrupted_hours" ? s.shed_mw > kEps :
              s.demand_mw > kEps && s.served_mw < 0.9 * s.demand_mw - kEps) ? 1.0 : 0.0;
    });
  }
  else if (key == "weighted_ens") {
    if (std::any_of(v.begin(), v.end(), [](const auto& s) {
      return !s.weighted_shed_mw || !std::isfinite(*s.weighted_shed_mw) || *s.weighted_shed_mw < 0.0;
    })) unavailable(r, "IMPORTANCE_SERIES_MISSING", {"steps.weighted_shed_mw"});
    else r.value = integrate(v, [](const auto& s) { return *s.weighted_shed_mw; });
  }
  else if (key == "critical_ens" || key == "high_ens" || key == "medium_ens" || key == "low_ens") {
    const std::size_t tier = key == "critical_ens" ? 0 : key == "high_ens" ? 1 : key == "medium_ens" ? 2 : 3;
    if (std::any_of(v.begin(), v.end(), [&](const auto& s) {
      return s.shed_by_priority.size() != 4 || !std::isfinite(s.shed_by_priority[tier]) || s.shed_by_priority[tier] < 0.0;
    })) unavailable(r, "PRIORITY_SERIES_MISSING", {"steps.shed_by_priority"});
    else r.value = integrate(v, [&](const auto& s) { return s.shed_by_priority[tier]; });
  }
  else if (key == "peak_active_faults") r.value = faults;
  else if (key == "repaired_faults") r.value = repaired;
  else if (key == "peak_islands" || key == "switch_actions") {
    double value = 0.0;
    for (const auto& s : v) {
      const auto count = key == "peak_islands" ? s.island_count : s.switch_actions;
      if (!count || *count < 0) { unavailable(r, "TOPOLOGY_SERIES_MISSING", {"steps." + key}); return true; }
      value = key == "peak_islands" ? std::max(value, double(*count)) : value + *count;
    }
    r.value = value;
  }
  else if (key == "mess_energy" || key == "mess_distance") {
    const auto value = key == "mess_energy" ? run.mess_energy_delivered_mwh : run.mess_travel_distance_km;
    if (!value || !std::isfinite(*value) || *value < 0.0) unavailable(r, "MESS_SUMMARY_MISSING");
    else r.value = value;
  }
  if (duration <= kEps && (key == "demand_energy" || key == "served_energy" || key == "ens" || key.find("_ens") != std::string::npos)) {
    r.value.reset(); r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "EMPTY_OBSERVATION_WINDOW";
  }
  return true;
}
}  // namespace

const char* to_string(ResilienceMetricStatus status) {
  switch (status) {
    case ResilienceMetricStatus::Computed: return "computed";
    case ResilienceMetricStatus::Approximate: return "approximate";
    case ResilienceMetricStatus::Unavailable: return "unavailable";
    case ResilienceMetricStatus::Invalid: return "invalid";
    case ResilienceMetricStatus::NotApplicable: return "not_applicable";
    case ResilienceMetricStatus::NotRequested: return "not_requested";
  }
  return "invalid";
}

const std::vector<ResilienceMetricDefinition>& resilience_metric_catalog() {
  static const std::vector<ResilienceMetricDefinition> catalog = [] {
    std::vector<ResilienceMetricDefinition> entries{
      def("ch3.lolp", "失负荷概率", "Loss of Load Probability", "LOLP", "pre_disaster", "expected_damage", "book_ch3_eq_3_23", "1", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.loss_of_load"}, {}, {"需要多场景概率样本，单次确定性运行不可计算。"}),
      def("ch3.edns", "电力不足期望", "Expected Demand Not Served", "EDNS", "pre_disaster", "expected_damage", "book_ch3_eq_3_24", "MW", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.loss_of_load_mw"}, {"正文为 EDNS，算例图出现 EENS；按式(3.24)保留 EDNS/MW。"}),
      def("ch3.alril", "平均失负荷比例", "Average Load Inadequacy Ratio", "ALRIL", "pre_disaster", "expected_damage", "book_ch3_eq_3_25", "1", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.demand_mw", "scenario_samples.shed_mw"}),
      def("ch3.pcfd", "灾害期间设备故障概率", "Probability of Component Failure During Disaster", "PCFD", "pre_disaster", "expected_damage", "book_ch3_eq_3_26", "1", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.failed_components", "component_type_denominators"}, {"后文存在 PCDF 拼写；稳定 ID 按式(3.26)使用 PCFD。"}),
      def("ch3.psi", "系统解列风险", "Power System Islanding Risk", "PSI", "pre_disaster", "islanding_risk", "book_ch3_eq_3_27", "1", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.system_islanding"}),
      def("ch3.pin", "系统孤立节点出现概率", "Probability of Isolated Nodes", "PIN", "pre_disaster", "islanding_risk", "book_ch3_eq_3_28", "1", "lower_is_better", "ensemble", "unavailable", {"scenario_samples.domain_qualified_isolated_nodes"}),
      def("ch3.gma", "区域发电裕度", "Generation Margin Area", "GMA", "pre_disaster", "margins", "book_ch3_eq_3_29", "1", "higher_is_better", "regional", "unavailable", {"region_definition", "pre_disaster_generation", "disaster_generation"}),
      def("ch3.tmts", "断面输电裕度", "Transmission Margin of Transmission Section", "TMTS", "pre_disaster", "margins", "book_ch3_eq_3_30", "1", "higher_is_better", "transmission_section", "unavailable", {"section_definition", "branch_capacity", "domain_qualified_flows"}),
      def("ch3.cllp", "当前切负荷比例", "Current Load Loss Percentage", "CLLP", "during_disaster", "load_shedding", "book_ch3_eq_3_31", "1", "lower_is_better", "single_scenario_timeseries", "available", {"steps.demand_mw", "steps.served_mw"}),
      def("ch3.atcs", "断面可用传输能力", "Available Transmission Capacity of Section", "ATCS", "during_disaster", "transmission", "book_ch3_eq_3_32", "MW/MVA", "higher_is_better", "transmission_section", "unavailable", {"section_definition", "branch_capacity", "during_disaster_flows"}),
      def("ch3.apda", "区域内有功不足缺额", "Active Power Deficit in Area", "APDA", "during_disaster", "regional_deficit", "book_ch3_eq_3_33", "MW", "lower_is_better", "regional", "conditional", {"region_definition", "generator_actual_output"}, {}, {"系统总供需缺口只能作为明确标注的代理。"}),
      def("ch3.ledsr", "灾害结束到负荷恢复开始的间隔", "Loss Event End to Demand Service Restoration", "LEDSR", "post_disaster", "response", "book_ch3_text_definition", "h", "lower_is_better", "single_scenario", "conditional", {"scenario.disaster_end_hr", "recovery_start_hr"}),
      def("ch3.rlro", "恢复开始后一小时内负荷恢复比例", "Load Restoration Ratio in First Hour", "RLRO", "post_disaster", "response", "book_ch3_eq_3_34", "1", "higher_is_better", "single_scenario", "conditional", {"steps.hour", "steps.served_mw", "recovery_start_hr"}, {"L_pl,b 的相邻正文定义不清，首版采用版本化近似解释。"}),
      def("ch3.t_sp", "达到指定停电负荷恢复比例的时间", "Time to Specified Restoration Percentage", "t_sp", "post_disaster", "response", "book_ch3_text_definition", "h", "lower_is_better", "single_scenario", "conditional", {"steps.hour", "steps.shed_mw", "scenario.disaster_end_hr", "recovery_start_hr", "target_ratio"}),
      def("ch3.arss", "系统负荷平均恢复速度", "Average Restoration Speed of System Load", "ARSS", "post_disaster", "recovery_efficiency", "book_ch3_eq_3_35", "1", "higher_is_better", "single_scenario", "conditional", {"steps.hour", "steps.shed_mw", "recovery_start_hr"}, {}, {"结果是单场景恢复轨迹值，不是蒙特卡罗期望。"}),
      def("ch3.res", "系统负荷恢复效率", "Restoration Efficiency of System Load", "RES", "post_disaster", "recovery_efficiency", "book_ch3_eq_3_36", "1", "higher_is_better", "single_scenario", "conditional", {"steps.hour", "steps.weighted_shed_mw", "load_importance", "rich_load_mapping"}, {"书中 RES 条目附近存在 REI acronym 误植。"}, {"priority-first rich-load 回投影不是原生逐负荷优化结果。"}),
      def("ch3.rei", "重要负荷恢复效率", "Restoration Efficiency of Important Loads", "REI", "post_disaster", "recovery_efficiency", "book_ch3_eq_3_37", "1", "higher_is_better", "single_scenario", "unavailable", {"formula_review", "important_load_set"}, {"式(3.37)有可疑前导负号、分母量纲和下标，需人工核版。"}),
      def("ch3.rse", "恢复方案经济性", "Restoration Scheme Economy", "RSE", "post_disaster", "economics", "book_ch3_eq_3_38", "currency_or_ratio", "higher_is_better", "single_scenario", "unavailable", {"outage_cost_data", "repair_cost_data", "operation_cost_data"}, {"式(3.38)为 PDF 图片；外部来源不得静默替换书中定义。"})};
    entries.push_back(def("run.duration", "计算覆盖时长", "duration", "DURATION", "operational", "run_evidence", "run_v1_duration", "h", "neutral", "single_run_observation_window", "available", {"steps.duration_hr", "steps.hour"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.demand_energy", "需求电量", "demand energy", "DEMAND_ENERGY", "operational", "run_evidence", "run_v1_demand_energy", "MWh", "neutral", "single_run_observation_window", "available", {"steps.demand_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.served_energy", "实际供电量", "served energy", "SERVED_ENERGY", "operational", "run_evidence", "run_v1_served_energy", "MWh", "higher_is_better", "single_run_observation_window", "available", {"steps.served_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.ens", "未供电量 ENS", "ens", "ENS", "operational", "run_evidence", "run_v1_ens", "MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.energy_supply_ratio", "电量供给率", "energy supply ratio", "ENERGY_SUPPLY_RATIO", "operational", "run_evidence", "run_v1_energy_supply_ratio", "1", "higher_is_better", "single_run_observation_window", "available", {"steps.served_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.energy_loss_ratio", "电量损失率", "energy loss ratio", "ENERGY_LOSS_RATIO", "operational", "run_evidence", "run_v1_energy_loss_ratio", "1", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.mean_shed", "时均失供功率", "mean shed", "MEAN_SHED", "operational", "run_evidence", "run_v1_mean_shed", "MW", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.peak_shed", "最大失供功率", "peak shed", "PEAK_SHED", "operational", "run_evidence", "run_v1_peak_shed", "MW", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.minimum_supply_ratio", "最低供电比例", "minimum supply ratio", "MINIMUM_SUPPLY_RATIO", "operational", "run_evidence", "run_v1_minimum_supply_ratio", "1", "higher_is_better", "single_run_observation_window", "available", {"steps.served_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.final_supply_ratio", "期末供电比例", "final supply ratio", "FINAL_SUPPLY_RATIO", "operational", "run_evidence", "run_v1_final_supply_ratio", "1", "higher_is_better", "single_run_observation_window", "available", {"steps.served_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.equivalent_outage_hours", "等效全负荷停电时长", "equivalent outage hours", "EQUIVALENT_OUTAGE_HOURS", "operational", "run_evidence", "run_v1_equivalent_outage_hours", "h", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.interrupted_hours", "存在失供的累计时长", "interrupted hours", "INTERRUPTED_HOURS", "operational", "run_evidence", "run_v1_interrupted_hours", "h", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.below_90_hours", "供电不足90%的累计时长", "below 90 hours", "BELOW_90_HOURS", "operational", "run_evidence", "run_v1_below_90_hours", "h", "lower_is_better", "single_run_observation_window", "available", {"steps.served_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.weighted_ens", "重要度加权未供电量", "weighted ens", "WEIGHTED_ENS", "operational", "run_evidence", "run_v1_weighted_ens", "weighted_MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.weighted_shed_mw", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.critical_ens", "关键负荷未供电量", "critical ens", "CRITICAL_ENS", "operational", "run_evidence", "run_v1_critical_ens", "MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_by_priority", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.high_ens", "高优先级负荷未供电量", "high ens", "HIGH_ENS", "operational", "run_evidence", "run_v1_high_ens", "MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_by_priority", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.medium_ens", "中优先级负荷未供电量", "medium ens", "MEDIUM_ENS", "operational", "run_evidence", "run_v1_medium_ens", "MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_by_priority", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.low_ens", "低优先级负荷未供电量", "low ens", "LOW_ENS", "operational", "run_evidence", "run_v1_low_ens", "MWh", "lower_is_better", "single_run_observation_window", "available", {"steps.shed_by_priority", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.peak_active_faults", "同时故障数峰值", "peak active faults", "PEAK_ACTIVE_FAULTS", "operational", "run_evidence", "run_v1_peak_active_faults", "count", "lower_is_better", "single_run_observation_window", "available", {"steps.active_faults", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.repaired_faults", "累计修复故障数", "repaired faults", "REPAIRED_FAULTS", "operational", "run_evidence", "run_v1_repaired_faults", "count", "higher_is_better", "single_run_observation_window", "available", {"steps.repaired_faults", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.peak_islands", "电气岛数峰值", "peak islands", "PEAK_ISLANDS", "operational", "run_evidence", "run_v1_peak_islands", "count", "neutral", "single_run_observation_window", "available", {"steps.island_count", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.switch_actions", "开关动作总次数", "switch actions", "SWITCH_ACTIONS", "operational", "run_evidence", "run_v1_switch_actions", "count", "neutral", "single_run_observation_window", "available", {"steps.switch_actions", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.mess_energy", "移动储能送出电量", "mess energy", "MESS_ENERGY", "operational", "run_evidence", "run_v1_mess_energy", "MWh", "neutral", "single_run_observation_window", "available", {"mess_energy_delivered_mwh", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    entries.push_back(def("run.mess_distance", "移动储能行驶里程", "mess distance", "MESS_DISTANCE", "operational", "run_evidence", "run_v1_mess_distance", "km", "neutral", "single_run_observation_window", "available", {"mess_travel_distance_km", "steps.hour", "steps.duration_hr"}, {"工程扩展指标；关联第三章损失、抵抗、恢复维度，非书中概率或期望指标。"}, {"继承本次求解的近似与模型覆盖限制，不证明动态安全。"}));
    return entries;
  }();
  return catalog;
}

std::vector<ResilienceMetricResult> evaluate_resilience_metrics(
    const ResilienceMetricRun& run, const ResilienceMetricEvaluationOptions& options) {
  const auto v = run.steps;
  std::vector<std::string> ids = options.selected_metric_ids;
  if (ids.empty()) for (const auto& item : resilience_metric_catalog()) ids.push_back(item.id);
  std::vector<ResilienceMetricResult> output;
  output.reserve(ids.size());
  for (const auto& id : ids) {
    const auto* item = lookup(id);
    if (!item) {
      ResilienceMetricResult r;
      r.id = id; r.status = ResilienceMetricStatus::Invalid;
      r.reason_code = "UNKNOWN_METRIC_ID"; output.push_back(std::move(r)); continue;
    }
    auto r = result_for(*item);
    if (!valid_run(run)) { r.status = ResilienceMetricStatus::Invalid; r.reason_code = "INVALID_RUN_STEPS"; output.push_back(std::move(r)); continue; }
    r.limitations.insert(r.limitations.end(), run.limitations.begin(), run.limitations.end());
    if (!run.usable) { unavailable(r, "RUN_NOT_FEASIBLE_OR_COMPLETE"); output.push_back(std::move(r)); continue; }
    if (operational_metric(run, v, r)) { output.push_back(std::move(r)); continue; }
    if (id == "ch3.apda") {
      if (!options.allow_apda_system_gap_approximation) unavailable(r, "APDA_PROXY_CONSENT_REQUIRED", {"region_definition", "generator_actual_output"});
      else {
        r.status = ResilienceMetricStatus::Approximate; r.approximate = true;
        r.value = std::max_element(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.shed_mw < b.shed_mw; })->shed_mw;
        r.source_fields = {"steps.shed_mw"};
        r.assumptions.push_back("Explicit system peak unserved-power proxy; not regional generation balance in equation 3.33.");
      }
      output.push_back(std::move(r)); continue;
    }
    if (id == "ch3.cllp") {
      const double demand = v.back().demand_mw, shed = v.back().shed_mw;
      r.assumptions.push_back("Equation 3.31 evaluated at the final observed step, not summed across time.");
      if (demand <= kEps) { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "ZERO_DEMAND"; }
      else { r.status = ResilienceMetricStatus::Computed; r.value = std::clamp(shed / demand, 0.0, 1.0); }
      output.push_back(std::move(r)); continue;
    }
    if (id == "ch3.ledsr" || id == "ch3.rlro" || id == "ch3.t_sp" || id == "ch3.arss" || id == "ch3.res") {
      const auto start = recovery_start(run, v);
      if (!start) { unavailable(r, run.disaster_end_hr ? "RECOVERY_START_NOT_IDENTIFIABLE" : "DISASTER_END_MISSING", {"recovery_start_hr"}); output.push_back(std::move(r)); continue; }
      const auto initial = at(v, *start); const double end = v.back().hour;
      if (!initial || end <= *start + kEps) { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "EMPTY_RECOVERY_WINDOW"; output.push_back(std::move(r)); continue; }
      if (id == "ch3.ledsr") { if (!run.disaster_end_hr) unavailable(r, "DISASTER_END_MISSING", {"scenario.disaster_end_hr"}); else { r.status = ResilienceMetricStatus::Computed; r.value = *start - *run.disaster_end_hr; } output.push_back(std::move(r)); continue; }
      if (id == "ch3.t_sp") {
        if (!run.disaster_end_hr) unavailable(r, "DISASTER_END_MISSING", {"scenario.disaster_end_hr"});
        else if (!std::isfinite(options.target_ratio) || options.target_ratio <= 0.0 || options.target_ratio > 1.0) { r.status = ResilienceMetricStatus::Invalid; r.reason_code = "TARGET_RATIO_OUT_OF_RANGE"; }
        else if (initial->shed_mw <= kEps) { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "NO_INITIAL_UNSERVED_LOAD"; }
        else {
          auto curve = v;
          for (auto& s : curve) s.restoration_ratio = 1.0 - s.shed_mw / initial->shed_mw;
          if (const auto hit = crossing(curve, *start, options.target_ratio)) { r.status = ResilienceMetricStatus::Computed; r.value = *hit - *run.disaster_end_hr; }
          else { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "TARGET_NOT_REACHED_WITHIN_WINDOW"; r.censored = true; }
          r.assumptions.push_back("Initially interrupted load uses shed power at recovery_start_hr; elapsed time starts at disaster_end_hr (PDF page 53).");
        }
        output.push_back(std::move(r)); continue;
      }
      const double duration = end - *start;
      r.assumptions.push_back("Recovery integrals use linear interpolation between observed points; the last sample is the evaluation endpoint, not proof of completed restoration.");
      if (v.back().shed_mw > kEps) { r.censored = true; r.limitations.push_back("Recovery remains incomplete at the observation endpoint."); }
      auto recovered_fraction = [&](bool weighted) -> std::optional<double> {
        auto curve = v;
        if (weighted) {
          for (auto& s : curve) {
            if (!s.weighted_shed_mw || !std::isfinite(*s.weighted_shed_mw) || *s.weighted_shed_mw < 0.0) return std::nullopt;
            s.shed_mw = *s.weighted_shed_mw;
          }
        }
        const double initial_shed = at(curve, *start)->shed_mw;
        if (initial_shed <= kEps) return std::nullopt;
        for (auto& s : curve) s.restoration_ratio = 1.0 - s.shed_mw / initial_shed;
        return ratio_area(curve, *start, end) / duration;
      };
      if (id == "ch3.rlro") { const auto one = at(v, *start + 1.0); if (!one) { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "WINDOW_SHORTER_THAN_ONE_HOUR"; } else { const double unserved = std::max(0.0, initial->demand_mw - initial->served_mw); if (unserved <= kEps) { r.status = ResilienceMetricStatus::NotApplicable; r.reason_code = "NO_INITIAL_UNSERVED_LOAD"; } else { r.status = ResilienceMetricStatus::Approximate; r.approximate = true; r.value = std::clamp((one->served_mw - initial->served_mw) / unserved, 0.0, 1.0); r.assumptions.push_back("L_pl,b interpreted as unserved load at recovery start."); r.limitations.push_back("PDF adjacent text does not define L_pl,b unambiguously."); } } }
      else if (id == "ch3.arss") {
        r.value = recovered_fraction(false);
        r.status = r.value ? ResilienceMetricStatus::Computed : ResilienceMetricStatus::NotApplicable;
        if (!r.value) r.reason_code = "NO_INITIAL_UNSERVED_LOAD";
      }
      else if (!run.has_traceable_importance_mapping || !options.allow_res_approximation) unavailable(r, "TRACEABLE_IMPORTANCE_MAPPING_REQUIRED", {"load_importance", "rich_load_mapping"});
      else {
        r.value = recovered_fraction(true);
        if (!r.value) unavailable(r, "WEIGHTED_INITIAL_SHED_OR_SERIES_MISSING", {"steps.weighted_shed_mw"});
        else { r.status = ResilienceMetricStatus::Approximate; r.approximate = true; r.source_fields = {"steps.weighted_shed_mw", "steps.hour"}; r.limitations.push_back("Priority-weighted canonical load allocation is a proxy for book per-load importance."); }
      }
      output.push_back(std::move(r)); continue;
    }
    unavailable(r, id == "ch3.rei" ? "FORMULA_REVIEW_REQUIRED" : id == "ch3.rse" ? "COST_DATA_MISSING_AND_FORMULA_IMAGE" : "REQUIRES_UNIMPLEMENTED_DATA");
    output.push_back(std::move(r));
  }
  return output;
}
}  // namespace hacdcpf::analysis
