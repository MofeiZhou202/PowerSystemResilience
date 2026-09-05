#include "hacdcpf/market/southern_market.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

namespace hacdcpf::market {
namespace {
using J = nlohmann::json;
const std::vector<std::string> factors = {"load_scale", "wind_scale", "solar_scale", "inflow_scale", "generator_bid_scale", "load_bid_scale", "line_limit_scale"};
constexpr double tolerance = 1e-6; // Same physical-unit residual/event threshold as the rolling contract.
void check(bool ok, const std::string& message) { if (!ok) throw std::invalid_argument("market forecast: " + message); }
void keys(const J& j, const std::set<std::string>& allowed) {
  check(j.is_object(), "expected object");
  for (auto i = j.begin(); i != j.end(); ++i) check(allowed.count(i.key()), "unknown field " + i.key());
}
double numeric(const J& j, double lo, double hi, const std::string& name) {
  check(j.is_number(), name + " must be numeric"); const double v = j.get<double>();
  check(std::isfinite(v) && v >= lo && v <= hi, name + " outside bounds"); return v;
}
double quantile(std::vector<double> values, double q) {
  std::sort(values.begin(), values.end());
  return values.at(std::max<size_t>(1, static_cast<size_t>(std::ceil(q*values.size())))-1);
}
J summary(const std::vector<double>& values, int total, bool probability, double threshold = tolerance) {
  const int n = static_cast<int>(values.size()); int events = 0;
  for (double v : values) if (v > threshold) ++events;
  J out = {{"valid_count", n}, {"total_count", total}, {"unknown_count", total-n}, {"event_count", events},
    {"mean", nullptr}, {"p05", nullptr}, {"p50", nullptr}, {"p95", nullptr}, {"max", nullptr},
    {"sample_event_fraction", nullptr}, {"event_probability_valid_only", nullptr}, {"probability_bounds", nullptr}, {"wilson95_valid_only", nullptr}};
  if (probability) out["probability_bounds"] = {double(events)/total, double(events+total-n)/total};
  if (!n) return out;
  out["mean"] = std::accumulate(values.begin(), values.end(), 0.0)/n;
  out["p05"] = quantile(values, .05); out["p50"] = quantile(values, .5); out["p95"] = quantile(values, .95);
  out["max"] = *std::max_element(values.begin(), values.end());
  const double p = double(events)/n; out["sample_event_fraction"] = p;
  if (probability) {
    out["event_probability_valid_only"] = p;
    // Wilson (1927), score interval; scenarios, not slots, are independent trials.
    constexpr double z = 1.959963984540054;
    const double denom = 1+z*z/n, center = (p+z*z/(2*n))/denom;
    const double half = z*std::sqrt(p*(1-p)/n+z*z/(4*n*n))/denom;
    out["wilson95_valid_only"] = {std::max(0.0, center-half), std::min(1.0, center+half)};
  }
  return out;
}
J pearson(const std::vector<double>& x, const std::vector<double>& y) {
  if (x.size() < 3) return nullptr;
  const double mx = std::accumulate(x.begin(), x.end(), 0.0)/x.size(), my = std::accumulate(y.begin(), y.end(), 0.0)/y.size();
  double xx = 0, yy = 0, xy = 0;
  for (size_t i = 0; i < x.size(); ++i) { xx += (x[i]-mx)*(x[i]-mx); yy += (y[i]-my)*(y[i]-my); xy += (x[i]-mx)*(y[i]-my); }
  if (xx <= 1e-20 || yy <= 1e-20) return nullptr;
  return std::clamp(xy/std::sqrt(xx*yy), -1.0, 1.0);
}
} // namespace

J market_forecast_defaults() {
  using namespace std::chrono;
  const year_month_day day{floor<days>(system_clock::now()) + days{1}};
  std::ostringstream date; date << std::setfill('0') << std::setw(4) << int(day.year()) << '-' << std::setw(2) << unsigned(day.month()) << '-' << std::setw(2) << unsigned(day.day());
  J result = {{"mode", "probabilistic"}, {"sample_count", 8}, {"seed", 20250905}, {"temporal_rho", 0.5},
    {"operation", {{"horizon", "week"}, {"start_date", date.str()}, {"penalty_per_mwh", 100000}, {"explain", true}, {"days", J::array()}}},
    {"marginals", J::array()}, {"correlation", J::array()}};
  for (size_t k = 0; k < factors.size(); ++k) {
    result["marginals"].push_back({{"factor", factors[k]}, {"distribution", k == 6 ? "fixed" : "uniform"},
      {"center", std::vector<double>(8, 1)}, {"lower", 0.9}, {"upper", 1.1}, {"sigma", 0.05}});
    std::vector<double> row(factors.size(), 0); row[k] = 1; result["correlation"].push_back(row);
  }
  return result;
}

J make_market_forecast(const J& boundary, const J& config) {
  keys(config, {"mode", "sample_count", "seed", "temporal_rho", "operation", "marginals", "correlation"});
  const bool probability = config.at("mode") == "probabilistic";
  check(probability || config.at("mode") == "interval", "mode must be probabilistic or interval");
  check(config.at("sample_count").is_number_integer(), "sample_count must be integer");
  const int count = static_cast<int>(numeric(config.at("sample_count"), 1, 512, "sample_count"));
  check(config.at("seed").is_number_integer(), "seed must be integer");
  const auto seed = static_cast<uint64_t>(numeric(config.at("seed"), 0, 4294967295.0, "seed"));
  const double rho = numeric(config.at("temporal_rho"), -0.95, 0.95, "temporal_rho");
  auto prototype = make_market_operation(boundary, config.at("operation"));
  check(prototype.at("total_days") == 7 && prototype.at("config").at("horizon") == "week", "forecast horizon must be one week");
  check(!config.at("operation").contains("reference_days"), "forecast defines reference_days from predicted centers");
  const int n = static_cast<int>(factors.size());
  J normalized = config;
  normalized["operation"]["solver_options"] = prototype.at("config").at("solver_options");
  auto& marginals = normalized.at("marginals");
  bool extended_centers = false;
  check(marginals.is_array() && marginals.size() == factors.size(), "seven ordered marginals required");
  for (int k = 0; k < n; ++k) {
    auto& m = marginals[k]; keys(m, {"factor", "distribution", "center", "lower", "upper", "sigma"});
    check(m.at("factor") == factors[k], "marginal order mismatch at " + factors[k]);
    const std::string kind = m.at("distribution");
    check(kind == "fixed" || (probability ? kind == "uniform" || kind == "triangular" || kind == "clipped_normal" : kind == "interval"), "distribution incompatible with mode");
    const double lo = numeric(m.at("lower"), 0, 10, "lower"), hi = numeric(m.at("upper"), lo, 10, "upper");
    numeric(m.at("sigma"), 0, 10, "sigma");
    check(m.at("center").is_array() && (m.at("center").size() == 7 || m.at("center").size() == 8), "center requires eight daily values including terminal forecast");
    if (m.at("center").size() == 7) { const auto last = m.at("center").back(); m["center"].push_back(last); extended_centers = true; }
    for (int d = 0; d < 8; ++d) {
      numeric(m.at("center")[d], lo, hi, "center");
      check(prototype.at("config").at("days")[d].at(factors[k]).get<double>()*hi <= 10, "generated " + factors[k] + " can exceed supported multiplier 10");
    }
  }
  Eigen::MatrixXd corr(n,n);
  check(config.at("correlation").is_array() && config.at("correlation").size() == factors.size(), "correlation must be 7x7");
  for (int i = 0; i < n; ++i) {
    check(config.at("correlation")[i].is_array() && config.at("correlation")[i].size() == factors.size(), "correlation must be 7x7");
    for (int k = 0; k < n; ++k) corr(i,k) = numeric(config.at("correlation")[i][k], -1, 1, "correlation");
  }
  check((corr-corr.transpose()).cwiseAbs().maxCoeff() <= 1e-10 && (corr.diagonal().array()-1).abs().maxCoeff() <= 1e-10, "correlation must be symmetric with unit diagonal");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(corr);
  check(eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1e-10, "correlation must be positive semidefinite");
  check(probability || (rho == 0 && (corr-Eigen::MatrixXd::Identity(n,n)).cwiseAbs().maxCoeff() <= 1e-10), "interval mode does not assign probabilistic correlations");
  // Nelsen (2006), Gaussian copula; singular PSD correlations are admitted.
  const Eigen::MatrixXd root = eig.eigenvectors()*eig.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
  std::mt19937_64 rng(seed); std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(0,1);
  std::vector<std::vector<int>> strata(8*n, std::vector<int>(count));
  for (auto& row : strata) { std::iota(row.begin(), row.end(), 0); if (!probability) std::shuffle(row.begin(), row.end(), rng); }
  J job = {{"status", "ready"}, {"config", normalized}, {"base", prototype.at("base")}, {"scenarios", J::array()},
    {"next_scenario", 0}, {"finished_scenarios", 0}, {"completed_days", 0}, {"total_days", count*7},
    {"factor_order", factors}, {"forecast_days",8}, {"statistics", nullptr}, {"sampler", "mt19937_64:normal-copula-ar1-or-interval-stratification-v2-eight-days"},
    {"limitations", {"日误差为同类型资源共同倍数，叠加已有 96 点模板；不是节点级或 15 分钟独立随机预测。", "限幅正态在上下界有概率质量；相关矩阵描述潜在高斯变量，非输出倍数的 Pearson 相关。", "概率是所设输入分布下的 Monte Carlo 估计；区间采样不提供概率或严格最坏界。", "失败/未运行场景保留为未知；有效样本统计、未知比例与概率上下界分别报告。", "逐日滚动及条件恢复，不是全周联合最优或唯一因果解释；未做交流安全认证。", "生成器和标准随机库版本相同才保证同 seed 逐值复现；导出保留已生成场景。"}}};
  job["limitations"].push_back("联合采样8日边界，只出清统计7日；第8日仅为第7日峰谷预安排。单值报价仍属于当天98点日窗。限时可行解的异常是已求得方案指标，不证明不可避免。");
  if (extended_centers) job["limitations"].push_back("旧版7日预测中心已显式按第7日中心补齐第8日；请核实末日预测。");
  for (int s = 0; s < count; ++s) {
    J scenario = prototype; scenario.erase("base"); scenario["id"] = s;
    scenario["config"]["reference_days"] = scenario["config"]["days"];
    Eigen::VectorXd previous = Eigen::VectorXd::Zero(n);
    for (int d = 0; d < 8; ++d) {
      Eigen::VectorXd eps(n); for (int k = 0; k < n; ++k) eps[k] = normal(rng);
      const Eigen::VectorXd z = d == 0 ? Eigen::VectorXd(root*eps) : Eigen::VectorXd(rho*previous+std::sqrt(1-rho*rho)*root*eps);
      previous = z;
      for (int k = 0; k < n; ++k) {
        const auto& m = marginals[k]; const std::string kind = m.at("distribution");
        const double lo = m.at("lower"), hi = m.at("upper"), center = m.at("center")[d];
        const double u = probability ? 0.5*std::erfc(-z[k]/std::sqrt(2.0)) : (strata[d*n+k][s]+uniform(rng))/count;
        double value = center;
        if (kind == "uniform" || kind == "interval") value = lo+(hi-lo)*u;
        else if (kind == "clipped_normal") value = std::clamp(center+m.at("sigma").get<double>()*z[k], lo, hi);
        else if (kind == "triangular" && hi > lo) value = u < (center-lo)/(hi-lo) ? lo+std::sqrt(u*(hi-lo)*(center-lo)) : hi-std::sqrt((1-u)*(hi-lo)*(hi-center));
        const double authored = prototype.at("config").at("days")[d].at(factors[k]);
        scenario["config"]["days"][d][factors[k]] = authored*value;
        scenario["config"]["reference_days"][d][factors[k]] = authored*center;
      }
      for (const auto* f : {"generator_outages", "branch_outages"}) scenario["config"]["reference_days"][d][f] = J::array();
    }
    job["scenarios"].push_back(std::move(scenario));
  }
  return job;
}

J market_forecast_statistics(const J& job) {
  const int total = static_cast<int>(job.at("scenarios").size());
  check(total > 0, "statistics requires scenarios");
  const bool probability = job.at("config").at("mode") == "probabilistic";
  J out = {{"mode", job.at("config").at("mode")}, {"total_scenarios", total}, {"complete_scenarios", 0},
    {"complete_scenarios_with_limit",0}, {"complete_scenarios_unproven",0},
    {"failed_scenarios", 0}, {"periods", J::array()}, {"nodes", J::array()}, {"lines", J::array()}, {"correlations", J::array()}};
  std::vector<const J*> complete;
  std::vector<double> week_delta, week_line, deficit_energy, surplus_energy, line_energy;
  for (const auto& scenario : job.at("scenarios")) {
    if (scenario.at("status") == "failed") out["failed_scenarios"] = out["failed_scenarios"].get<int>()+1;
    if (scenario.at("status") != "completed") continue;
    complete.push_back(&scenario); double dp = 0, dij = 0, de = 0, se = 0, le = 0;
    bool limited = false, unproven = false;
    for (const auto& day : scenario.at("days")) {
      for (const auto* stage : {"scuc","sced"}) {
        const auto result = day.value("stages",J::object()).value(stage,J::object());
        limited = limited || result.value("limit_reached",false);
        unproven = unproven || !result.value("optimality_proven",false);
      }
      de += day.at("deficit_mwh").get<double>(); se += day.at("surplus_mwh").get<double>(); le += day.at("overload_mwh").get<double>();
      for (const auto& p : day.at("periods")) {
        dp = std::max(dp, p.at("deficit_mw").get<double>()+p.at("surplus_mw").get<double>());
        dij = std::max(dij, p.at("max_line_overload_mw").get<double>());
      }
    }
    if (limited) out["complete_scenarios_with_limit"] = out["complete_scenarios_with_limit"].get<int>()+1;
    if (unproven) out["complete_scenarios_unproven"] = out["complete_scenarios_unproven"].get<int>()+1;
    week_delta.push_back(dp); week_line.push_back(dij); deficit_energy.push_back(de); surplus_energy.push_back(se); line_energy.push_back(le);
  }
  out["complete_scenarios"] = complete.size();
  out["week_delta_p_peak_mw"] = summary(week_delta, total, probability);
  out["week_delta_pij_peak_mw"] = summary(week_line, total, probability);
  out["week_deficit_mwh"] = summary(deficit_energy, total, false);
  out["week_surplus_mwh"] = summary(surplus_energy, total, false);
  out["week_overload_mwh"] = summary(line_energy, total, false);
  // Per-slot trials are across independent weekly scenarios, never pooled over time.
  for (int d = 0; d < 7; ++d) for (int t = 0; t < 96; ++t) {
    std::vector<double> dp, dij;
    for (const auto& scenario : job.at("scenarios")) {
      if (scenario.at("days").size() <= static_cast<size_t>(d) || !scenario.at("days")[d].at("valid").get<bool>()) continue;
      const auto& p = scenario.at("days")[d].at("periods")[t];
      dp.push_back(p.at("deficit_mw").get<double>()+p.at("surplus_mw").get<double>());
      dij.push_back(p.at("overload_sum_mw"));
    }
    out["periods"].push_back({{"day", d}, {"slot", t}, {"delta_p_abs_sum_mw", summary(dp, total, probability)}, {"delta_pij_sum_mw", summary(dij, total, probability)}});
  }
  for (const auto& [table, output] : {std::pair{"buses", "nodes"}, std::pair{"branches", "lines"}}) {
    size_t pos = 0;
    for (const auto& source : job.at("base").at(table)) {
      std::vector<double> peaks, energies;
      for (const J* scenario : complete) {
        double peak = 0, energy = 0;
        for (const auto& day : scenario->at("days")) {
          const auto& row = day.at(output).at(pos); check(row.at("id") == source.at("id"), "stable entity identity changed");
          for (int t = 0; t < 96; ++t) {
            const double value = std::string(output) == "nodes" ? row.at("deficit_mw")[t].get<double>()+row.at("surplus_mw")[t].get<double>() : row.at("overload_mw")[t].get<double>();
            peak = std::max(peak, value); energy += value*0.25;
          }
        }
        peaks.push_back(peak); energies.push_back(energy);
      }
      out[output].push_back({{"id", source.at("id")}, {"name", source.at("name")}, {"peak_mw", summary(peaks, total, probability)}, {"integral_mwh", summary(energies, total, false)}});
      ++pos;
    }
  }
  for (const auto& factor : factors) {
    std::vector<double> input;
    for (const J* scenario : complete) {
      double mean = 0; for (int d = 0; d < 7; ++d) mean += scenario->at("config").at("days")[d].at(factor).get<double>()/7;
      input.push_back(mean);
    }
    out["correlations"].push_back({{"factor", factor}, {"valid_count", input.size()}, {"deficit_energy_pearson", pearson(input, deficit_energy)}, {"overload_integral_pearson", pearson(input, line_energy)}});
  }
  return out;
}

J step_market_forecast(const J& previous) {
  check(previous.at("status") == "ready" || previous.at("status") == "running", "job cannot advance");
  J job = previous; const int index = job.at("next_scenario");
  J operation = job.at("scenarios").at(index); operation["base"] = job.at("base");
  const int old_days = operation.at("completed_days");
  operation = step_market_operation(operation); operation.erase("base");
  job["completed_days"] = job.at("completed_days").get<int>()+operation.at("completed_days").get<int>()-old_days;
  const bool finished = operation.at("status") == "completed" || operation.at("status") == "failed";
  job["scenarios"][index] = std::move(operation);
  if (finished) {
    job["next_scenario"] = index+1; job["finished_scenarios"] = index+1;
    job["statistics"] = market_forecast_statistics(job);
  }
  job["status"] = index+int(finished) == static_cast<int>(job.at("scenarios").size()) ? "completed" : "running";
  return job;
}
} // namespace hacdcpf::market
