#include "hacdcpf/market/southern_market.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace hacdcpf::market {
namespace {
using J = nlohmann::json;
constexpr int kT = 98;
J number(const char* title, const char* unit, double lo = -1e9, double hi = 1e9) {
  return {{"type", "number"}, {"title", title}, {"unit", unit},
          {"minimum", lo}, {"maximum", hi}};
}
J integer(const char* title, int lo = 0, int hi = 1000000) {
  J s = number(title, "", lo, hi); s["type"] = "integer"; return s;
}
J string(const char* title) { return {{"type", "string"}, {"title", title}}; }
J choice(const char* title, J values) {
  J s = string(title); s["enum"] = std::move(values); return s;
}
J array(const char* title, J item, int minimum = 0, int maximum = 10000) {
  return {{"type", "array"}, {"title", title}, {"items", std::move(item)},
          {"minItems", minimum}, {"maxItems", maximum}};
}
J series(const char* title, const char* unit, double lo = -1e9, double hi = 1e9) {
  return array(title, number(title, unit, lo, hi), kT, kT);
}
J object(const char* title, J properties) {
  J required = J::array();
  for (auto it = properties.begin(); it != properties.end(); ++it) required.push_back(it.key());
  return {{"type", "object"}, {"title", title}, {"properties", properties},
          {"required", required}, {"additionalProperties", false}};
}
J record(const char* title, J properties) {
  properties["id"] = integer("稳定 ID");
  properties["name"] = string("名称");
  properties["source"] = string("边界来源 / 调整依据");
  return object(title, std::move(properties));
}
void require(bool ok, const std::string& path, const std::string& message) {
  if (!ok) throw std::invalid_argument("southern boundary " + path + ": " + message);
}
void check(const J& value, const J& schema, const std::string& path) {
  const auto type = schema.at("type").get<std::string>();
  if (type == "object") {
    require(value.is_object(), path, "expected object");
    for (const auto& key : schema.at("required"))
      require(value.contains(key.get<std::string>()), path, "missing " + key.get<std::string>());
    for (auto it = value.begin(); it != value.end(); ++it) {
      require(schema.at("properties").contains(it.key()), path, "unknown field " + it.key());
      check(it.value(), schema.at("properties").at(it.key()), path + "/" + it.key());
    }
  } else if (type == "array") {
    require(value.is_array(), path, "expected array");
    require(value.size() >= schema.at("minItems").get<size_t>() &&
            value.size() <= schema.at("maxItems").get<size_t>(), path, "invalid array length");
    for (size_t i = 0; i < value.size(); ++i) check(value[i], schema.at("items"), path + "/" + std::to_string(i));
  } else if (type == "string") {
    require(value.is_string(), path, "expected string");
    require(value.get_ref<const std::string&>().size() <= 2048, path, "text too long");
  } else {
    require(type == "integer" ? value.is_number_integer() : value.is_number(), path, "expected " + type);
    const double v = value.get<double>();
    require(std::isfinite(v) && v >= schema.at("minimum").get<double>() &&
            v <= schema.at("maximum").get<double>(), path, "value outside bounds");
  }
  if (schema.contains("enum")) {
    const auto& values = schema.at("enum");
    require(std::find(values.begin(), values.end(), value) != values.end(), path, "invalid enum value");
  }
}
J constant(double value) { return std::vector<double>(kT, value); }
}  // namespace

J southern_market_schema() {
  const auto id = integer("稳定 ID");
  const auto on = array("可用状态", integer("状态", 0, 1), kT, kT);
  const auto unit = record("发电交易单元", {
    {"bus", id}, {"kind", choice("机组类别", {"thermal", "hydro", "pumped_hydro", "renewable", "wind", "solar", "vpp"})},
    {"bid_mode", choice("申报方式", {"price", "quantity"})},
    {"available", on}, {"must_on", on}, {"must_off", on},
    {"pmin_mw", series("出力下限", "MW", 0)}, {"pmax_mw", series("出力上限", "MW", 0)},
    {"technical_min_mw", number("最小技术出力", "MW", 0)},
    {"forecast_mw", series("新能源预测", "MW", 0)},
    {"renewable_alpha", number("新能源下限系数", "p.u.", 0, 1)},
    {"max_curtailment_mw", series("新能源偏差上限", "MW", 0)},
    {"reserve_up_eligible", on}, {"reserve_down_eligible", on},
    {"primary_fraction", series("一次调频容量比例", "p.u.", 0, 1)},
    {"price_setting", on}, {"initial_on", integer("初始开机", 0, 1)},
    {"initial_state_minutes", integer("初始状态持续时间", 0, 1000000)},
    {"initial_power_mw", number("初始出力", "MW", 0)},
    {"min_up_minutes", integer("最小开机时间 / min")},
    {"min_down_minutes", integer("最小停机时间 / min")},
    {"ramp_up_mw_min", number("上爬坡速率", "MW/min", 0)},
    {"ramp_down_mw_min", number("下爬坡速率", "MW/min", 0)},
    {"max_starts", integer("最大启动次数", 0, 98)}, {"max_stops", integer("最大停机次数", 0, 98)},
    {"warm_after_minutes", integer("温态停机门槛 / min", 1)},
    {"cold_after_minutes", integer("冷态停机门槛 / min", 2)},
    {"startup_cost", array("热 / 温 / 冷启动费", number("启动费", "CNY", 0), 3, 3)},
    {"startup_curves_mw", array("热 / 温 / 冷启动曲线", array("启动轨迹", number("功率", "MW", 0), 0, 96), 3, 3)},
    {"shutdown_curve_mw", array("停机轨迹", number("功率", "MW", 0), 0, 96)},
    {"minimum_cost_per_hour", number("最小出力费用", "CNY/h")},
    {"segments", array("增量报价段", object("报价段", {
      {"quantity_mw", number("段宽", "MW", 0)}, {"price_per_mwh", number("段价", "CNY/MWh")}}), 1, 10)},
    {"regulation_up_mw", series("调频预出清上调容量", "MW", 0)},
    {"regulation_down_mw", series("调频预出清下调容量", "MW", 0)},
    {"regulation_source", string("调频预出清来源")},
    {"qmin_mvar", number("无功下限", "Mvar")}, {"qmax_mvar", number("无功上限", "Mvar")},
    {"voltage_pu", number("电压设定值", "p.u.", 0.5, 1.5)}
  });
  auto schema = object("南方区域日前边界", {
    {"schema_version", choice("数据版本", {"southern-2025-v1.0-execution-1"})},
    {"name", string("场景名称")}, {"source", string("数据来源")},
    {"base_mva", number("基准容量", "MVA", 1, 1000000)},
    {"periods", array("优化时间点", object("时间点", {
      {"kind", choice("类型", {"day", "peak", "valley"})},
      {"start_minute", integer("距运行日零点 / min", 0, 10000)},
      {"duration_hr", number("物理持续时长", "h", 0.001, 24)},
      {"weight_hr", number("目标费用权重", "h", 0.001, 24)}}), 98, 98)},
    {"areas", array("统调边界", record("平衡区", {
      {"load_mw", series("统调负荷预测", "MW", 0)},
      {"reserve_up_mw", series("正备用需求", "MW", 0)},
      {"reserve_down_mw", series("负备用需求", "MW", 0)},
      {"network_reserve_reduction_mw", series("网络备用受限量", "MW", 0)},
      {"load_side_down_reserve_mw", series("负荷侧负备用", "MW", 0)},
      {"primary_mw", series("一次调频需求", "MW", 0)}}), 1, 100)},
    {"primary_groups", array("直调一次调频系统", record("直调系统", {
      {"generators", array("机组 ID", id, 1)}, {"requirement_mw", series("一次调频需求", "MW", 0)}}))},
    {"buses", array("母线预测", record("交流母线", {
      {"area", id}, {"load_mw", series("原始母线负荷预测", "MW", 0)},
      {"q_load_mvar", series("母线无功负荷", "Mvar")},
      {"base_kv", number("额定电压", "kV", 0.1, 1500)},
      {"vmin_pu", number("电压下限", "p.u.", 0.5, 1.5)},
      {"vmax_pu", number("电压上限", "p.u.", 0.5, 1.5)}}), 1)},
    {"generators", array("机组运行 / 检修 / 申报", unit, 1)},
    {"controllable_loads", array("可中断负荷（研究扩展）", record("可中断负荷", {
      {"bus", id}, {"available", on},
      {"max_reduction_mw", series("最大可削减功率", "MW", 0)},
      {"compensation_per_mwh", series("削减补偿报价", "CNY/MWh", 0)},
      {"max_day_reduction_mwh", number("运行日削减电量上限", "MWh", 0)}}))},
    {"branches", array("输变电边界", record("交流支路 / 变压器", {
      {"from_bus", id}, {"to_bus", id}, {"available", on},
      {"r_pu", number("电阻", "p.u.", 0, 100)}, {"x_pu", number("电抗", "p.u.", 1e-6, 100)},
      {"b_pu", number("充电电纳", "p.u.", -100, 100)},
      {"tap", number("变比", "p.u.", 0.01, 100)}, {"shift_deg", number("相移", "deg", -360, 360)},
      {"min_mw", series("反向功率限值", "MW")}, {"max_mw", series("正向功率限值", "MW")},
      {"rate_mva", series("交流热限额", "MVA", 0)}}))},
    {"sections", array("安全断面", record("断面", {
      {"members", array("有向支路", object("断面成员", {{"branch", id}, {"coefficient", number("方向权重", "", -100, 100)}}), 1)},
      {"min_mw", series("断面下限", "MW")}, {"max_mw", series("断面上限", "MW")}}))},
    {"external_schedules", array("非市场主体 / 区外计划", record("计划", {
      {"bus", id}, {"kind", choice("主体类别", {"hydro", "renewable", "biomass", "nuclear", "captive", "pumped_storage", "external"})},
      {"power_mw", series("计划净注入", "MW")}}))},
    {"groups", array("机组群 / 水电优化调度", record("机组群", {
      {"generators", array("机组 ID", id, 1)},
      {"min_online", series("最少开机台数", "", 0)}, {"max_online", series("最多开机台数", "", 0)},
      {"min_mw", series("群出力下限", "MW", 0)}, {"max_mw", series("群出力上限", "MW", 0)},
      {"min_mwh", number("运行日电量下限", "MWh", 0)}, {"max_mwh", number("运行日电量上限", "MWh", 0)}}))},
    {"storage", array("储能交易单元", record("储能", {
      {"bus", id}, {"available", on},
      {"discharge_min_mw", number("最小放电功率", "MW", 0)}, {"discharge_max_mw", number("最大放电功率", "MW", 0)},
      {"charge_min_mw", number("最小充电幅值", "MW", 0)}, {"charge_max_mw", number("最大充电幅值", "MW", 0)},
      {"rated_mwh", number("额定容量", "MWh", 0.001)},
      {"roundtrip_efficiency", number("往返效率", "p.u.", 0.001, 1)},
      {"initial_mwh", number("前日末能量", "MWh", 0)}, {"terminal_mwh", number("运行日末目标", "MWh", 0)},
      {"min_mwh", series("能量下限", "MWh", 0)}, {"max_mwh", series("能量上限", "MWh", 0)},
      {"max_cycles", number("最大循环次数", "", 0)},
      {"discharge_price", number("放电报价", "CNY/MWh")}, {"charge_price", number("充电报价", "CNY/MWh")},
      {"price_setting", on}}))},
    {"dc_hubs", array("多端直流中枢", record("中枢", J::object()))},
    {"dc_links", array("直流联络线", record("直流端口", {
      {"from_bus", integer("送端 AC 母线 / -1 中枢", -1)}, {"to_bus", integer("受端 AC 母线 / -1 中枢", -1)},
      {"from_hub", integer("送端中枢 / -1 母线", -1)}, {"to_hub", integer("受端中枢 / -1 母线", -1)},
      {"available", on}, {"min_mw", series("送端功率下限", "MW", 0)}, {"max_mw", series("送端功率上限", "MW", 0)},
      {"loss_fraction", number("损耗率", "p.u.", 0, 0.99)},
      {"initial_mw", number("初始送端功率", "MW", 0)},
      {"initial_adjustment", integer("初始调节方向", -1, 1)},
      {"ramp_up_mw", series("上调限值", "MW", 0)}, {"ramp_down_mw", series("下调限值", "MW", 0)}}))},
    {"trades", array("跨省交易 / D-2 保障下限", record("交易成分", {
      {"gateway_kind", choice("物理关口类型", {"ac_branch", "dc_link"})}, {"gateway", id},
      {"direction", choice("关口方向", {"forward", "reverse"})},
      {"min_mw", series("交易功率下限", "MW", 0)}, {"max_mw", series("交易功率上限", "MW", 0)},
      {"original_min_mwh", number("调整前保障电量", "MWh", 0)},
      {"adjusted_min_mwh", number("D-2 校核后保障电量", "MWh", 0)},
      {"max_mwh", number("日电量上限", "MWh", 0)}, {"adjustment_reason", string("D-2 调整原因")},
      {"scuc_fee", number("SCUC 过网费", "CNY/MWh")}, {"sced_fee", number("SCED 过网费", "CNY/MWh")},
      {"lmp_fee", number("定价过网费", "CNY/MWh")}}))},
    {"reservoirs", array("水库运用边界", record("水库", {
      {"generator", id}, {"generators", array("同库水电机组 ID", id, 1)}, {"upstream", integer("上游水库 / -1 无", -1)}, {"lag_slots", integer("传播时滞 / 点", 0, 96)},
      {"release_history_m3_s", array("历史下泄流量", number("下泄流量", "m3/s", 0), 0, 96)},
      {"water_m3_mwh", number("耗水率", "m3/MWh", 0.001)}, {"area_m2", number("库面面积", "m2", 1)},
      {"initial_level_m", number("初始水位", "m")},
      {"physical_min_m", number("物理最低水位", "m")}, {"physical_max_m", number("物理最高水位", "m")},
      {"min_level_m", series("调度水位下限", "m")}, {"max_level_m", series("调度水位上限", "m")},
      {"inflow_m3_s", series("区间来水", "m3/s", 0)},
      {"spill_max_m3_s", series("泄洪流量上限", "m3/s", 0)},
      {"release_min_m3_s", series("生态 / 航运下泄下限", "m3/s", 0)},
      {"release_max_m3_s", series("防洪下泄上限", "m3/s", 0)},
      {"release_ramp_m3_s", series("下泄流量变幅", "m3/s", 0)},
      {"initial_release_m3_s", number("初始下泄流量", "m3/s", 0)},
      {"min_mwh", number("水电日电量下限", "MWh", 0)}, {"max_mwh", number("水电日电量上限", "MWh", 0)}}))},
    {"execution", object("执行解释 / 求解", {
      {"interpretation", choice("A1–A7 解释版本", {"explicit-time-incremental-bids-si-water-v1"})},
      {"priority_policy", choice("优先计划 A5", {"hard", "penalized_shortfall"})},
      {"lmp_storage_policy", choice("定价储能 A7", {"power_neighborhood_only", "retain_energy_constraints"})},
      {"lmp_renewable_priority", choice("定价新能源 / 优先计划 A7", {"omit_unlisted", "retain_sced"})},
      {"penalties", array("M1 / M2 / M3 / M4", number("出清罚因子", "CNY/MWh", 0.001), 4, 4)},
      {"pricing_penalties", array("M1' / M2' / M3' / M4'", number("定价罚因子", "CNY/MWh", 0.001), 4, 4)},
      {"price_delta", number("定价邻域比例", "p.u.", 0.000001, 1)},
      {"time_limit_sec", number("每次 MILP 时限", "s", 0.1, 3600)},
      {"mip_gap", number("相对最优性间隙", "", 0, 0.1)},
      {"ac_security", choice("交流安全校核", {"required", "schedule_only"})},
      {"security_iterations", integer("全链路安全迭代上限", 1, 20)}})}
  });
  // Additive execution-1 fields: older saved snapshots retain their meaning.
  auto& required = schema["required"];
  required.erase(std::remove(required.begin(), required.end(), J("controllable_loads")), required.end());
  auto& bus_properties = schema["properties"]["buses"]["items"]["properties"];
  bus_properties["gs_mw"] = number("并联电导额定损耗", "MW", 0);
  bus_properties["bs_mvar"] = number("并联电纳额定注入", "Mvar");
  auto& bus_required = schema["properties"]["buses"]["items"]["required"];
  bus_required.erase(std::remove(bus_required.begin(), bus_required.end(), J("gs_mw")), bus_required.end());
  bus_required.erase(std::remove(bus_required.begin(), bus_required.end(), J("bs_mvar")), bus_required.end());
  auto& reservoir_required = schema["properties"]["reservoirs"]["items"]["required"];
  reservoir_required.erase(std::remove(reservoir_required.begin(), reservoir_required.end(), J("generators")), reservoir_required.end());
  reservoir_required.erase(std::remove(reservoir_required.begin(), reservoir_required.end(), J("generator")), reservoir_required.end());
  return schema;
}

J validate_southern_market(const J& boundary) {
  check(boundary, southern_market_schema(), "");
  J effective = boundary;
  if (!effective.contains("controllable_loads")) effective["controllable_loads"] = J::array();
  std::map<std::string, std::set<int>> ids;
  for (const auto* name : {"areas", "buses", "generators", "branches", "sections", "external_schedules", "groups", "storage", "dc_hubs", "dc_links", "trades", "reservoirs", "primary_groups"}) {
    for (const auto& row : boundary.at(name)) {
      const int key = row.at("id");
      require(ids[name].insert(key).second, name, "duplicate id " + std::to_string(key));
      require(!row.at("source").get_ref<const std::string&>().empty(), name, "source is required");
    }
  }
  const auto ref = [&](const char* table, int value) {
    require(ids[table].count(value) != 0, table, "unknown referenced id " + std::to_string(value));
  };
  const auto bounds = [&](const J& row, const char* low, const char* high) {
    const auto& l = row.at(low); const auto& h = row.at(high);
    if (l.is_array()) {
      for (int t = 0; t < kT; ++t) require(l[t].get<double>() <= h[t].get<double>(), low, "lower exceeds " + std::string(high));
    } else require(l.get<double>() <= h.get<double>(), low, "lower exceeds " + std::string(high));
  };
  std::set<std::string> representatives;
  for (int t = 0; t < kT; ++t) {
    const auto& p = boundary.at("periods")[t];
    if (t < 96) require(p.at("kind") == "day" && p.at("start_minute") == 15*t &&
      p.at("duration_hr") == 0.25 && p.at("weight_hr") == 0.25, "periods", "D day must contain 96 quarter hours");
    else {
      require(p.at("kind") != "day" && p.at("start_minute").get<int>() >= 1440,
              "periods", "representative point must belong to D+1");
      require(p.at("start_minute").get<double>() + 60*p.at("duration_hr").get<double>() <= 2880,
              "periods", "representative interval extends beyond D+1");
      representatives.insert(p.at("kind"));
    }
    if (t > 0) require(p.at("start_minute").get<int>() >= boundary.at("periods")[t-1].at("start_minute").get<int>() +
      std::lround(60*boundary.at("periods")[t-1].at("duration_hr").get<double>()), "periods", "physical intervals overlap or are unordered");
  }
  require(representatives.size() == 2, "periods", "need one peak and one valley");
  for (const auto& b : boundary.at("buses")) { ref("areas", b.at("area")); bounds(b, "vmin_pu", "vmax_pu"); }
  // 2.4.1.2: preserve authored forecasts; return the proportional reconciliation.
  for (const auto& a : boundary.at("areas")) for (int t = 0; t < kT; ++t) {
    double total = 0;
    for (const auto& b : boundary.at("buses")) if (b.at("area") == a.at("id")) total += b.at("load_mw")[t].get<double>();
    const double demand = a.at("load_mw")[t];
    require(total > 0 || demand == 0, "areas/load_mw", "nonzero dispatch forecast has zero bus forecast sum");
    for (auto& b : effective["buses"]) if (b.at("area") == a.at("id"))
      b["load_mw"][t] = total > 0 ? b.at("load_mw")[t].get<double>() * demand/total : 0.0;
  }
  for (const auto* table : {"generators", "storage", "external_schedules"}) for (const auto& r : boundary.at(table)) ref("buses", r.at("bus"));
  std::set<int> load_ids;
  for (const auto& d : effective.at("controllable_loads")) {
    require(load_ids.insert(d.at("id").get<int>()).second, "controllable_loads", "duplicate id");
    ref("buses", d.at("bus"));
    require(!d.at("source").get_ref<const std::string&>().empty(), "controllable_loads", "source is required");
  }
  for (const auto& g : boundary.at("generators")) {
    bounds(g, "pmin_mw", "pmax_mw"); bounds(g, "qmin_mvar", "qmax_mvar");
    require(g.at("warm_after_minutes").get<int>() < g.at("cold_after_minutes").get<int>(), "generators", "startup thresholds must increase");
    require(g.at("initial_on") != 0 || g.at("initial_power_mw") == 0, "generators", "offline initial power must be zero");
    require(!g.at("regulation_source").get_ref<const std::string&>().empty(), "generators", "regulation source required (including explicit zero awards)");
    double width = g.at("technical_min_mw"); double price = -1e10;
    for (const auto& s : g.at("segments")) {
      require(s.at("price_per_mwh").get<double>() >= price, "segments", "incremental prices must be nondecreasing");
      price = s.at("price_per_mwh"); width += s.at("quantity_mw").get<double>();
    }
    // 2.2.10 provides the absolute bid interval semantics missing from the
    // free-index expression in 2.6.3.6. Widths are their equivalent differences.
    const double flexible = width - g.at("technical_min_mw").get<double>();
    for (const auto& segment : g.at("segments")) require(segment.at("quantity_mw").get<double>() >= 0.01*flexible,
      "segments", "each segment must cover at least 1% of the bid flexible range (2.2.10)");
    for (const auto& curve : g.at("startup_curves_mw")) if (g.at("initial_on") == 1)
      require(g.at("initial_state_minutes").get<double>() >= 15*curve.size(), "generators", "initial on state is inside a startup trajectory; a pre-horizon trajectory extension is required");
    for (int t = 0; t < kT; ++t) {
      require(g.at("must_on")[t].get<int>() + g.at("must_off")[t].get<int>() <= 1 &&
              g.at("must_on")[t].get<int>() <= g.at("available")[t].get<int>(), "generators", "conflicting maintenance/must-on/must-off");
      require(g.at("pmax_mw")[t].get<double>() <= width + 1e-9, "segments", "bid widths do not cover pmax");
      require(g.at("technical_min_mw").get<double>() <= g.at("pmin_mw")[t].get<double>(), "generators", "technical minimum exceeds stable lower bound");
      const bool renewable = g.at("kind") == "renewable" || g.at("kind") == "wind" || g.at("kind") == "solar";
      if (!renewable) require(g.at("forecast_mw")[t] == 0 && g.at("max_curtailment_mw")[t] == 0,
        "generators", "renewable fields require renewable kind");
      if (renewable || g.at("kind") == "vpp")
        require(g.at("primary_fraction")[t] == 0, "generators", "primary reserve in 2.6.3.4 only covers thermal, hydro and pumped hydro");
    }
    if (g.at("bid_mode") == "quantity") require(g.at("kind") == "renewable" || g.at("kind") == "wind" || g.at("kind") == "solar", "bid_mode", "quantity mode is reserved for renewable units; use external_schedules for nonmarket plans");
  }
  for (const auto& b : boundary.at("branches")) {
    ref("buses", b.at("from_bus")); ref("buses", b.at("to_bus"));
    require(b.at("from_bus") != b.at("to_bus"), "branches", "self loop"); bounds(b, "min_mw", "max_mw");
  }
  for (const auto& s : boundary.at("sections")) {
    bounds(s, "min_mw", "max_mw"); std::set<int> members;
    for (const auto& m : s.at("members")) { ref("branches", m.at("branch")); require(members.insert(m.at("branch").get<int>()).second, "sections", "duplicate branch"); }
  }
  for (const auto* table : {"groups", "primary_groups"}) for (const auto& g : boundary.at(table)) {
    std::set<int> members;
    for (const auto& i : g.at("generators")) { ref("generators", i); require(members.insert(i.get<int>()).second, table, "duplicate generator"); }
    if (std::string(table) == "groups") { bounds(g, "min_mw", "max_mw"); bounds(g, "min_mwh", "max_mwh"); bounds(g, "min_online", "max_online"); }
  }
  for (const auto& s : boundary.at("storage")) {
    bounds(s, "min_mwh", "max_mwh"); bounds(s, "charge_min_mw", "charge_max_mw"); bounds(s, "discharge_min_mw", "discharge_max_mw");
    for (const auto& e : s.at("max_mwh")) require(e.get<double>() <= s.at("rated_mwh").get<double>(), "storage", "energy upper bound exceeds rating");
    require(s.at("initial_mwh").get<double>() <= s.at("rated_mwh").get<double>() &&
      s.at("terminal_mwh").get<double>() >= s.at("min_mwh")[95].get<double>() &&
      s.at("terminal_mwh").get<double>() <= s.at("max_mwh")[95].get<double>(), "storage", "invalid initial or terminal energy");
  }
  for (const auto& d : boundary.at("dc_links")) {
    for (const auto* side : {"from", "to"}) {
      const int b = d.at(std::string(side)+"_bus"), h = d.at(std::string(side)+"_hub");
      require((b >= 0) != (h >= 0), "dc_links", "endpoint must be exactly one bus or hub");
      if (b >= 0) ref("buses", b); else ref("dc_hubs", h);
    }
    bounds(d, "min_mw", "max_mw");
  }
  for (const auto& tr : boundary.at("trades")) {
    ref(tr.at("gateway_kind") == "ac_branch" ? "branches" : "dc_links", tr.at("gateway"));
    require(tr.at("gateway_kind") != "dc_link" || tr.at("direction") == "forward", "trades", "DC ports use nonnegative sending power");
    bounds(tr, "min_mw", "max_mw"); bounds(tr, "adjusted_min_mwh", "max_mwh");
    if (tr.at("original_min_mwh") != tr.at("adjusted_min_mwh")) require(!tr.at("adjustment_reason").get_ref<const std::string&>().empty(), "trades", "D-2 adjustment reason required");
  }
  std::map<int, int> upstream;
  std::map<int, size_t> history;
  std::set<int> hydro_generators;
  for (const auto& h : boundary.at("reservoirs")) {
    J members = h.contains("generator") ? J::array({h.at("generator")}) : h.at("generators");
    require(!members.empty(), "reservoirs", "at least one generator is required");
    for (const auto& member : members) { ref("generators", member); require(hydro_generators.insert(member.get<int>()).second, "reservoirs", "generator belongs to multiple reservoirs"); }
    for (const auto& g : boundary.at("generators")) if (std::find(members.begin(), members.end(), g.at("id")) != members.end())
      require(g.at("kind") == "hydro" || g.at("kind") == "pumped_hydro", "reservoirs", "reservoir requires hydro generator");
    bounds(h, "physical_min_m", "physical_max_m"); bounds(h, "min_level_m", "max_level_m");
    bounds(h, "release_min_m3_s", "release_max_m3_s"); bounds(h, "min_mwh", "max_mwh");
    upstream[h.at("id")] = h.at("upstream"); history[h.at("id")] = h.at("release_history_m3_s").size();
    if (h.at("upstream").get<int>() >= 0) ref("reservoirs", h.at("upstream"));
    require(h.at("initial_level_m").get<double>() >= h.at("physical_min_m").get<double>() &&
      h.at("initial_level_m").get<double>() <= h.at("physical_max_m").get<double>(), "reservoirs", "initial level outside physical range");
    for (int t = 0; t < kT; ++t) require(h.at("min_level_m")[t].get<double>() >= h.at("physical_min_m").get<double>() &&
      h.at("max_level_m")[t].get<double>() <= h.at("physical_max_m").get<double>(), "reservoirs", "dispatch level outside physical range");
  }
  for (const auto& h : boundary.at("reservoirs")) {
    std::set<int> visited; int u = h.at("id");
    while (u >= 0) { require(visited.insert(u).second, "reservoirs", "cyclic upstream chain"); u = upstream.at(u); }
    const int parent = h.at("upstream");
    if (parent >= 0) require(history.at(parent) >= h.at("lag_slots").get<size_t>(), "reservoirs", "upstream release history too short");
  }
  return effective;
}

J make_southern_market_example() {
  J j = {{"schema_version", "southern-2025-v1.0-execution-1"}, {"name", "南方规则解析算例"},
         {"source", "Synthetic analytic input; not actual Southern grid data"}, {"base_mva", 100}};
  for (const auto* table : {"periods", "areas", "buses", "generators", "branches", "sections", "external_schedules", "groups", "primary_groups", "storage", "dc_hubs", "dc_links", "trades", "reservoirs"}) j[table] = J::array();
  j["controllable_loads"] = J::array();
  for (int t = 0; t < kT; ++t) j["periods"].push_back({{"kind", t < 96 ? "day" : t == 96 ? "valley" : "peak"},
    {"start_minute", t < 96 ? 15*t : t == 96 ? 1680 : 2160}, {"duration_hr", 0.25}, {"weight_hr", 0.25}});
  j["areas"].push_back({{"id", 1}, {"name", "示例平衡区"}, {"source", "synthetic"}, {"load_mw", constant(100)},
    {"reserve_up_mw", constant(0)}, {"reserve_down_mw", constant(0)}, {"network_reserve_reduction_mw", constant(0)},
    {"load_side_down_reserve_mw", constant(0)}, {"primary_mw", constant(0)}});
  j["buses"].push_back({{"id", 1}, {"name", "示例母线"}, {"source", "synthetic"}, {"area", 1},
    {"load_mw", constant(100)}, {"q_load_mvar", constant(0)}, {"base_kv", 220}, {"vmin_pu", 0.9}, {"vmax_pu", 1.1}});
  j["generators"].push_back({{"id", 1}, {"name", "示例机组"}, {"source", "synthetic"}, {"bus", 1}, {"kind", "thermal"},
    {"bid_mode", "price"}, {"available", std::vector<int>(kT, 1)}, {"must_on", std::vector<int>(kT, 0)}, {"must_off", std::vector<int>(kT, 0)},
    {"pmin_mw", constant(0)}, {"pmax_mw", constant(200)}, {"technical_min_mw", 0},
    {"forecast_mw", constant(0)}, {"renewable_alpha", 0}, {"max_curtailment_mw", constant(0)},
    {"reserve_up_eligible", std::vector<int>(kT, 1)}, {"reserve_down_eligible", std::vector<int>(kT, 1)},
    {"primary_fraction", constant(0)}, {"price_setting", std::vector<int>(kT, 1)},
    {"initial_on", 1}, {"initial_state_minutes", 1440}, {"initial_power_mw", 100},
    {"min_up_minutes", 0}, {"min_down_minutes", 0}, {"ramp_up_mw_min", 200}, {"ramp_down_mw_min", 200},
    {"max_starts", 98}, {"max_stops", 98}, {"warm_after_minutes", 240}, {"cold_after_minutes", 720},
    {"startup_cost", {0, 0, 0}}, {"startup_curves_mw", {J::array(), J::array(), J::array()}},
    {"shutdown_curve_mw", J::array()}, {"minimum_cost_per_hour", 0},
    {"segments", {{{"quantity_mw", 200}, {"price_per_mwh", 200}}}},
    {"regulation_up_mw", constant(0)}, {"regulation_down_mw", constant(0)}, {"regulation_source", "Explicit zero preclearing awards"},
    {"qmin_mvar", -200}, {"qmax_mvar", 200}, {"voltage_pu", 1}});
  j["execution"] = {{"interpretation", "explicit-time-incremental-bids-si-water-v1"}, {"priority_policy", "hard"},
    {"lmp_storage_policy", "power_neighborhood_only"}, {"lmp_renewable_priority", "omit_unlisted"},
    {"penalties", {10000, 1000, 1000, 10000}}, {"pricing_penalties", {12000, 1200, 1200, 12000}},
    {"price_delta", 0.05}, {"time_limit_sec", 120}, {"mip_gap", 0}, {"ac_security", "required"}, {"security_iterations", 4}};
  validate_southern_market(j);
  return j;
}

J southern_market_from_system(const HybridPowerSystem& system) {
  // Conversion is deliberately limited to explicit AC buses, branches, loads,
  // and generators. Missing market declarations must remain visible inputs.
  require(system.dc.buses.empty() && system.vsc_converters.empty() && system.lcc_converters.empty(), "system", "hybrid assets require explicit Southern gateway/boundary authoring");
  require(system.ac.storage.empty() && system.ac.static_generators.empty() && system.ac.renewable_gens.empty() &&
    system.ac.pv_systems.empty() && system.ac.external_grids.empty() &&
    system.ac.flexible_loads.empty() && system.ac.asymmetric_loads.empty() && system.ac.transformers_2w.empty() &&
    system.ac.transformers_3w.empty() && system.ac.switches.empty() && system.ac.circuit_breakers.empty() &&
    system.ac.charging_stations.empty() && system.ac.chargers.empty() && system.ac.motors.empty() &&
    system.energy_routers.empty() && system.mobile_storage.empty() && system.vpps.empty() && system.microgrids.empty(),
    "system", "additional engineering devices require explicit market-boundary authoring; automatic import cannot discard them");
  J j = make_southern_market_example();
  j["name"] = "工程系统南方市场边界"; j["source"] = "Imported engineering AC network with explicitly synthetic market offers/history";
  j["base_mva"] = system.base_mva;
  for (const auto* table : {"areas", "buses", "generators", "branches"}) j[table] = J::array();
  std::map<int, double> demand;
  std::map<int, int> area;
  for (const auto& b : system.ac.buses) if (b.in_service) {
    double p = b.pd_mw, q = b.qd_mvar;
    double gs = b.gs_mw, bs = b.bs_mvar;
    for (const auto& shunt : system.ac.shunts) if (shunt.in_service && shunt.bus == b.index) {
      gs += shunt.gs_mw; bs += shunt.bs_mvar;
    }
    for (const auto& l : system.ac.loads) if (l.in_service && l.bus == b.index) { p += l.p_mw*l.scaling; q += l.q_mvar*l.scaling; }
    demand[b.area] += p; area[b.index] = b.area;
    j["buses"].push_back({{"id", b.index}, {"name", b.name}, {"source", "engineering system"}, {"area", b.area},
      {"load_mw", constant(p)}, {"q_load_mvar", constant(q)}, {"gs_mw", gs}, {"bs_mvar", bs},
      {"base_kv", b.base_kv}, {"vmin_pu", b.vmin_pu}, {"vmax_pu", b.vmax_pu}});
  }
  const J seed = make_southern_market_example();
  for (const auto& [id, p] : demand) {
    J a = seed["areas"][0]; a["id"] = id; a["name"] = "Area " + std::to_string(id); a["load_mw"] = constant(p); j["areas"].push_back(a);
  }
  for (const auto& g : system.ac.generators) {
    require(area.count(g.bus) != 0, "generators", "generator references inactive bus");
    J row = seed["generators"][0]; row["id"] = g.index; row["bus"] = g.bus; row["name"] = g.name;
    row["source"] = "engineering capabilities; synthetic offer and startup history";
    row["available"] = std::vector<int>(kT, g.in_service ? 1 : 0);
    row["pmax_mw"] = constant(g.pmax_mw); row["pmin_mw"] = constant(std::max(0.0, g.pmin_mw));
    row["initial_on"] = g.in_service && g.pg_mw > 0 ? 1 : 0; row["initial_power_mw"] = g.in_service ? std::max(0.0, g.pg_mw) : 0;
    row["segments"][0]["quantity_mw"] = g.pmax_mw;
    row["qmin_mvar"] = g.qmin_mvar; row["qmax_mvar"] = g.qmax_mvar; row["voltage_pu"] = g.vg_pu;
    j["generators"].push_back(row);
  }
  // Large-system research augmentation: preserve every imported unit and add
  // explicitly synthetic renewable/hydro units to exercise the full market
  // boundary. Stable IDs are outside the source case's authored range.
  const std::vector<int> bus_ids = [&] { std::vector<int> ids; for (const auto& b : j["buses"]) ids.push_back(b.at("id")); return ids; }();
  const int next_gen = j["generators"].empty() ? 1 : (*std::max_element(j["generators"].begin(), j["generators"].end(), [](const J& a, const J& b) { return a.at("id") < b.at("id"); })).at("id").get<int>() + 1;
  const int add_wind = 240, add_solar = 240, add_hydro = 720;
  for (int k = 0; k < add_wind + add_solar + add_hydro; ++k) {
    const int id = next_gen + k, bus = bus_ids[static_cast<size_t>(k) % bus_ids.size()];
    const char* kind = k < add_wind ? "wind" : k < add_wind + add_solar ? "solar" : "hydro";
    const double cap = kind[0] == 'h' ? 180.0 : 120.0;
    J row = seed["generators"][0]; row["id"] = id; row["bus"] = bus;
    row["name"] = std::string("Synthetic ") + kind + " " + std::to_string(id);
    row["source"] = "synthetic resource augmentation for 2000-bus benchmark"; row["kind"] = kind;
    row["pmax_mw"] = constant(cap); row["pmin_mw"] = constant(0); row["technical_min_mw"] = 0;
    row["segments"][0]["quantity_mw"] = cap; row["segments"][0]["price_per_mwh"] = kind[0] == 'h' ? 35 : 5;
    row["forecast_mw"] = constant(cap * (kind[0] == 'h' ? 0.55 : 0.45)); row["max_curtailment_mw"] = constant(cap);
    row["bid_mode"] = "quantity"; row["initial_on"] = 1; row["initial_power_mw"] = cap * 0.4;
    row["ramp_up_mw_min"] = cap; row["ramp_down_mw_min"] = cap;
    j["generators"].push_back(std::move(row));
  }
  const int next_load = j["controllable_loads"].empty() ? 1 : (*std::max_element(j["controllable_loads"].begin(), j["controllable_loads"].end(), [](const J& a, const J& b) { return a.at("id") < b.at("id"); })).at("id").get<int>() + 1;
  for (int k = 0; k < 120; ++k) {
    const int bus = bus_ids[static_cast<size_t>(k * 17) % bus_ids.size()];
    j["controllable_loads"].push_back({{"id", next_load + k}, {"name", "Synthetic interruptible load " + std::to_string(next_load + k)},
      {"source", "synthetic demand-response augmentation for 2000-bus benchmark"}, {"bus", bus},
      {"available", std::vector<int>(kT, 1)}, {"max_reduction_mw", constant(20)},
      {"compensation_per_mwh", constant(90)}, {"max_day_reduction_mwh", 480}});
  }
  for (const auto& b : system.ac.branches) {
    require(area.count(b.from_bus) && area.count(b.to_bus), "branches", "branch references inactive bus");
    const double rate = b.rate_a_mva > 0 ? b.rate_a_mva : 1e6;
    j["branches"].push_back({{"id", b.index}, {"name", b.name}, {"source", b.rate_a_mva > 0 ? "engineering system" : "engineering system; absent thermal rating represented by explicit 1e6 MVA research bound"},
      {"from_bus", b.from_bus}, {"to_bus", b.to_bus}, {"available", std::vector<int>(kT, b.in_service ? 1 : 0)},
      {"r_pu", b.r_pu}, {"x_pu", b.x_pu}, {"b_pu", b.b_pu}, {"tap", b.tap == 0 ? 1 : b.tap}, {"shift_deg", b.shift_deg},
      {"min_mw", constant(-rate)}, {"max_mw", constant(rate)}, {"rate_mva", constant(rate)}});
  }
  validate_southern_market(j);
  return j;
}
}  // namespace hacdcpf::market
