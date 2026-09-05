#include "hacdcpf/market/southern_market.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
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
struct Expr {
  std::map<int, double> terms;
  double constant{0};
  Expr() = default;
  explicit Expr(double c) : constant(c) {}
  Expr& add(int index, double coefficient = 1) { terms[index] += coefficient; return *this; }
  Expr& add(const Expr& rhs, double scale = 1) {
    constant += scale*rhs.constant;
    for (const auto& [i, v] : rhs.terms) terms[i] += scale*v;
    return *this;
  }
};
Expr variable(int id, double coefficient = 1) { Expr e; return e.add(id, coefficient); }
double value(const Expr& e, const Eigen::VectorXd& x) {
  double v = e.constant;
  for (const auto& [i, c] : e.terms) v += c*x[i];
  return v;
}
double num(const J& row, const char* key) { return row.at(key).get<double>(); }
double at(const J& row, const char* key, int t) { return row.at(key)[t].get<double>(); }
std::string key(const char* family, int id, int t) {
  return std::string(family) + "/" + std::to_string(id) + "/" + std::to_string(t);
}
struct Row { Expr expr; double rhs; std::string name; };
struct SecurityCut {
  std::string name;
  std::map<std::string, double> coefficients;
  double rhs{0};
};
struct Build {
  engine::MIPModel model;
  std::vector<double> costs;
  std::vector<std::string> cost_categories;
  std::map<std::string, int> columns;
  std::vector<Row> eq, le;
  std::map<std::string, int> balance_rows;
  std::map<std::pair<int,int>, std::pair<int,int>> reserve_rows;
  std::map<std::string, Expr> metrics;
  double constant_cost{0};
  std::map<std::string, double> fixed;
  std::string stage;
  int var(std::string name, double lo, double hi, double cost = 0,
          const char* category = "energy", bool binary = false, bool freeze = false) {
    if (freeze) {
      const auto found = fixed.find(name);
      if (found == fixed.end()) throw std::logic_error("missing preceding-stage variable: " + name);
      lo = hi = binary ? std::round(found->second) : found->second;
    }
    if (lo > hi + tolerance) throw std::invalid_argument(name + ": incompatible effective bounds");
    const int id = static_cast<int>(costs.size());
    columns.emplace(name, id); costs.push_back(cost); cost_categories.emplace_back(category);
    model.linear_part.vars.push_back({binary ? engine::VarType::Binary : engine::VarType::Continuous, lo, hi, name});
    if (binary && !freeze) model.binary_idx.push_back(id);
    return id;
  }
  int col(const char* family, int id, int t) const { return columns.at(key(family, id, t)); }
  Expr expr(const char* family, int id, int t) const { return variable(col(family, id, t)); }
  int equal(std::string name, Expr e, double rhs = 0) {
    eq.push_back({std::move(e), rhs, std::move(name)}); return static_cast<int>(eq.size()) - 1;
  }
  void upper(std::string name, Expr e, double rhs = 0) { le.push_back({std::move(e), rhs, std::move(name)}); }
  void range(const std::string& name, const Expr& e, double lo, double hi) {
    upper(name + "/upper", e, hi); Expr negative; negative.add(e, -1); upper(name + "/lower", negative, -lo);
  }
  void finish() {
    auto& lp = model.linear_part;
    lp.c = Eigen::Map<Eigen::VectorXd>(costs.data(), static_cast<Eigen::Index>(costs.size()));
    lp.sense = engine::Sense::Minimize;
    const auto matrix = [&](const std::vector<Row>& rows, auto& A, auto& b) {
      std::vector<Eigen::Triplet<double>> entries;
      b.resize(static_cast<Eigen::Index>(rows.size()));
      for (size_t r = 0; r < rows.size(); ++r) {
        b[static_cast<Eigen::Index>(r)] = rows[r].rhs - rows[r].expr.constant;
        for (const auto& [c, v] : rows[r].expr.terms) if (v != 0) entries.emplace_back(static_cast<int>(r), c, v);
      }
      A.resize(static_cast<int>(rows.size()), static_cast<int>(costs.size()));
      A.setFromTriplets(entries.begin(), entries.end());
    };
    matrix(le, lp.A, lp.b); matrix(eq, lp.Aeq, lp.beq);
  }
};

std::map<std::string, double> solution_map(const Build& b, const engine::SolveResult& r) {
  std::map<std::string, double> result;
  for (const auto& [name, col] : b.columns) result.emplace(name, r.x[col]);
  return result;
}

Build build_model(const J& j, const std::string& stage,
                  std::map<std::string, double> previous = {},
                  const std::vector<SecurityCut>& cuts = {}) {
  Build b; b.stage = stage; b.fixed = std::move(previous);
  const bool uc = stage == "scuc", pricing = stage == "lmp";
  const auto& exec = j.at("execution");
  const auto& penalties = exec.at(pricing ? "pricing_penalties" : "penalties");
  const auto w = [&](int t) { return num(j.at("periods")[t], "weight_hr"); };
  const auto dt = [&](int t) { return num(j.at("periods")[t], "duration_hr"); };
  const auto minute = [&](int t) { return num(j.at("periods")[t], "start_minute"); };
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

  for (const auto& g : j.at("generators")) {
    const int id = g.at("id"), bus = g.at("bus"), area = bus_area.at(bus);
    const bool renewable = g.at("kind") == "renewable" || g.at("kind") == "wind" || g.at("kind") == "solar";
    for (int t = 0; t < T; ++t) {
      const double active = at(g, "available", t)*(1-at(g, "must_off", t));
      b.var(key("u", id, t), at(g, "must_on", t), active, uc ? num(g, "minimum_cost_per_hour")*w(t) : 0, "minimum_output", true, !uc);
      b.var(key("start", id, t), 0, 1, 0, "startup", true, !uc);
      b.var(key("stop", id, t), 0, 1, 0, "startup", true, !uc);
      b.var(key("stable", id, t), 0, 1, 0, "energy", false, !uc);
      b.var(key("offline_minutes", id, t), 0, max_time, 0, "energy", false, !uc);
      for (int k = 0; k < 3; ++k)
        b.var(key(("start"+std::to_string(k)).c_str(), id, t), 0, 1,
              uc ? g.at("startup_cost")[k].get<double>() : 0, "startup", true, !uc);
      double lo = 0, hi = at(g, "pmax_mw", t)*active;
      if (pricing) {
        const double p = b.fixed.at(key("p", id, t));
        const double delta = num(exec, "price_delta");
        if (at(g, "price_setting", t) == 0) lo = hi = p;
        else { lo = std::max(0.0, (1-delta)*p); hi = std::min(hi, (1+delta)*p); }
      }
      const int p = b.var(key("p", id, t), lo, hi);
      injection[{bus, t}].add(p);
      const int pf = b.var(key("primary", id, t), 0, at(g, "pmax_mw", t));
      primary[{area, t}].add(pf);
      const double capacity_max = at(g, "pmax_mw", t) - (uc ? 0 : at(g, "regulation_up_mw", t));
      const double capacity_min = at(g, "pmin_mw", t) + (uc ? 0 : at(g, "regulation_down_mw", t));
      // 2.6.3.2--3 after substituting area balance: all output is subtracted,
      // while alpha gates only the capacity term, including hydro eligibility.
      headroom[{area, t}].add(p, -1);
      footroom[{area, t}].add(p);
      if (at(g, "reserve_up_eligible", t) != 0)
        headroom[{area, t}].add(b.col("u", id, t), capacity_max);
      if (at(g, "reserve_down_eligible", t) != 0)
        footroom[{area, t}].add(b.col("u", id, t), -capacity_min);
      b.upper(key("2.6.3.4/headroom", id, t), variable(pf).add(p).add(b.col("u", id, t), -capacity_max));
      b.upper(key("2.6.3.4/fraction", id, t), variable(pf).add(b.col("u", id, t), -capacity_max*at(g, "primary_fraction", t)));
      for (size_t k = 0; k < g.at("segments").size(); ++k) {
        const auto& s = g.at("segments")[k];
        b.var(key(("segment"+std::to_string(k)).c_str(), id, t), 0, num(s, "quantity_mw"),
              g.at("bid_mode") == "quantity" ? 0 : num(s, "price_per_mwh")*w(t));
      }
      if (renewable) b.var(key("renewable_deviation", id, t), 0, at(g, "max_curtailment_mw", t), penalties[1].get<double>()*w(t), "renewable_deviation");
    }
    Expr starts, stops;
    for (int t = 0; t < T; ++t) {
      const auto u = b.expr("u", id, t), start = b.expr("start", id, t), stop = b.expr("stop", id, t);
      const Expr prev_u = t ? b.expr("u", id, t-1) : Expr(num(g, "initial_on"));
      Expr transition = u; transition.add(prev_u, -1).add(start, -1).add(stop);
      b.equal(key("2.6.3.13/state", id, t), transition);
      Expr events = start; events.add(stop); b.upper(key("2.6.3.13/exclusive", id, t), events, 1);
      starts.add(start); stops.add(stop);
      Expr categories; for (int k = 0; k < 3; ++k) categories.add(b.col(("start"+std::to_string(k)).c_str(), id, t));
      categories.add(start, -1); b.equal(key("2.6.3.13/start_class", id, t), categories);
      const double elapsed = t ? minute(t)-minute(t-1) : 0;
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
      // 2.6.3.12: event windows include the residual obligation of the authored initial state.
      for (int s = 0; s <= t; ++s) {
        if (minute(t)-minute(s) < num(g, "min_up_minutes")) {
          Expr e = b.expr("start", id, s); e.add(u, -1); b.upper(key(("2.6.3.12/up"+std::to_string(s)).c_str(), id, t), e);
        }
        if (minute(t)-minute(s) < num(g, "min_down_minutes")) {
          Expr e = b.expr("stop", id, s); e.add(u); b.upper(key(("2.6.3.12/down"+std::to_string(s)).c_str(), id, t), e, 1);
        }
      }
      const double minimum = num(g, num(g, "initial_on") ? "min_up_minutes" : "min_down_minutes");
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
      Expr prev_p = t ? b.expr("p", id, t-1) : Expr(num(g, "initial_power_mw"));
      const double ramp_minutes = t ? elapsed : 15;
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

  for (const auto& a : j.at("areas")) for (int t = 0; t < T; ++t) {
    const int area = a.at("id");
    const int up_row = static_cast<int>(b.le.size());
    Expr up; up.add(headroom[{area, t}], -1);
    b.upper(key("2.6.3.2/reserve", area, t), up, -at(a, "reserve_up_mw", t)-at(a, "network_reserve_reduction_mw", t));
    Expr down; down.add(footroom[{area, t}], -1);
    const int down_row = static_cast<int>(b.le.size());
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
      if (t < 96) energy.add(power, dt(t));
    }
    b.range(key("2.6.3.10/energy", group.at("id"), 0), energy, num(group, "min_mwh"), num(group, "max_mwh"));
  }

  // Research extension, southern_execution_contract.md: compensated interruption,
  // not energy shifting. Forecast load already includes the interruptible demand.
  std::map<std::pair<int,int>, Expr> reductions;
  for (const auto& d : j.at("controllable_loads")) {
    const int id = d.at("id"), bus = d.at("bus"); Expr energy;
    for (int t = 0; t < T; ++t) {
      const int r = b.var(key("load_reduction", id, t), 0,
        at(d, "available", t)*at(d, "max_reduction_mw", t),
        at(d, "compensation_per_mwh", t)*w(t), "demand_response", false, pricing);
      injection[{bus, t}].add(r); reductions[{bus, t}].add(r);
      if (t < 96) energy.add(r, dt(t));
    }
    b.upper(key("demand_response/day_energy", id, 0), energy, num(d, "max_day_reduction_mwh"));
  }
  for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t) {
    const auto found = reductions.find({node.at("id"), t});
    if (found != reductions.end()) b.upper(key("demand_response/bus_load", node.at("id"), t), found->second, at(node, "load_mw", t));
  }

  for (const auto& s : j.at("storage")) {
    const int id = s.at("id"), bus = s.at("bus");
    const double eta = std::sqrt(num(s, "roundtrip_efficiency"));
    const bool energy_model = !pricing || exec.at("lmp_storage_policy") == "retain_energy_constraints";
    for (int t = 0; t < T; ++t) {
      const double avail = at(s, "available", t);
      b.var(key("dis_on", id, t), 0, avail, 0, "storage", true, pricing);
      b.var(key("ch_on", id, t), 0, avail, 0, "storage", true, pricing);
      double dlo = 0, dhi = num(s, "discharge_max_mw")*avail;
      double clo = -num(s, "charge_max_mw")*avail, chi = 0;
      if (pricing) {
        const double dis = b.fixed.at(key("dis", id, t)), ch = b.fixed.at(key("ch", id, t));
        const double delta = at(s, "price_setting", t) ? num(exec, "price_delta") : 0;
        dlo = std::max(dlo, (1-delta)*dis); dhi = std::min(dhi, (1+delta)*dis);
        clo = std::max(clo, (1+delta)*ch); chi = std::min(chi, (1-delta)*ch);
      }
      const int dis = b.var(key("dis", id, t), dlo, dhi, num(s, "discharge_price")*w(t), "storage");
      const int ch = b.var(key("ch", id, t), clo, chi, num(s, "charge_price")*w(t), "storage");
      injection[{bus, t}].add(dis).add(ch);
      const auto [reserve_up, reserve_down] = b.reserve_rows.at({bus_area.at(bus), t});
      b.le[reserve_up].expr.add(dis).add(ch);
      b.le[reserve_down].expr.add(dis, -1).add(ch, -1);
      if (energy_model) b.var(key("energy", id, t), at(s, "min_mwh", t), at(s, "max_mwh", t));
    }
    Expr cycle;
    for (int t = 0; t < T; ++t) {
      auto dis = b.expr("dis", id, t), ch = b.expr("ch", id, t);
      Expr dir = b.expr("dis_on", id, t); dir.add(b.expr("ch_on", id, t)); b.upper(key("2.6.3.16/direction", id, t), dir, 1);
      Expr dlo; dlo.add(dis, -1).add(b.col("dis_on", id, t), num(s, "discharge_min_mw")); b.upper(key("2.6.3.16/dis_min", id, t), dlo);
      Expr dhi = dis; dhi.add(b.col("dis_on", id, t), -num(s, "discharge_max_mw")); b.upper(key("2.6.3.16/dis_max", id, t), dhi);
      Expr clo; clo.add(ch, -1).add(b.col("ch_on", id, t), -num(s, "charge_max_mw")); b.upper(key("2.6.3.16/ch_min", id, t), clo);
      Expr chi = ch; chi.add(b.col("ch_on", id, t), num(s, "charge_min_mw")); b.upper(key("2.6.3.16/ch_max", id, t), chi);
      if (t < 96) {
        Expr hourly = b.expr("dis_on", id, t);
        for (int tt = t/4*4; tt < t/4*4+4; ++tt) hourly.add(b.col("ch_on", id, tt), 0.25);
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
      b.equal(key("2.6.3.16/terminal", id, 95), b.expr("energy", id, 95), num(s, "terminal_mwh"));
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
  for (const auto& node : j.at("buses")) for (int t = 0; t < T; ++t)
    b.var(key("theta", node.at("id"), t), -inf, inf);
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
      const int p = b.var(key("trade", id, t), at(trade, "min_mw", t), at(trade, "max_mw", t),
          num(trade, uc ? "scuc_fee" : pricing ? "lmp_fee" : "sced_fee")*w(t), "transmission");
      if (t < 96) energy.add(p, dt(t));
    }
    if (!pricing || exec.at("lmp_renewable_priority") == "retain_sced") {
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
      b.var(key("level", id, t), at(h, "min_level_m", t), at(h, "max_level_m", t));
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
      Expr balance = b.expr("level", id, t);
      if (t) balance.add(b.col("level", id, t-1), -1); else balance.constant -= num(h, "initial_level_m");
      const double factor = dt(t)*3600/num(h, "area_m2");
      balance.add(release, factor); balance.constant -= factor*at(h, "inflow_m3_s", t);
      if (parent >= 0) {
        if (t >= lag) balance.add(b.metrics.at(key("release", parent, t-lag)), -factor);
        else {
          const auto& history = reservoirs.at(parent)->at("release_history_m3_s");
          balance.constant -= factor*history[history.size()-static_cast<size_t>(lag-t)].get<double>();
        }
      }
      b.equal(key("2.6.3.18/water", id, t), balance);
      b.range(key("2.4.8/release", id, t), release, at(h, "release_min_m3_s", t), at(h, "release_max_m3_s", t));
      Expr diff = release;
      if (t) diff.add(b.metrics.at(key("release", id, t-1)), -1); else diff.constant -= num(h, "initial_release_m3_s");
      b.range(key("2.4.8/release_ramp", id, t), diff, -at(h, "release_ramp_m3_s", t), at(h, "release_ramp_m3_s", t));
      if (t < 96) for (const auto& member : members) energy.add(b.col("p", member, t), dt(t));
    }
    b.range(key("2.6.3.19/hydro_energy", id, 0), energy, num(h, "min_mwh"), num(h, "max_mwh"));
  }
  for (const auto& cut : cuts) {
    Expr e; for (const auto& [name, coefficient] : cut.coefficients) e.add(b.columns.at(name), coefficient);
    b.upper(cut.name, e, cut.rhs);
  }
  b.finish(); return b;
}

engine::SolveResult solve(const Build& b, const J& input) {
  if (b.model.binary_idx.empty()) return engine::HighsAdapter{}.solve_lp(b.model.linear_part);
  engine::BCOptions options;
  options.time_limit_sec = num(input.at("execution"), "time_limit_sec");
  options.gap_tol = num(input.at("execution"), "mip_gap");
  return engine::StrictHighsBranchAndCutAdapter{options}.solve_milp(b.model);
}

J stage_result(const Build& b, const engine::SolveResult& solved, const J& j) {
  J out = {{"stage", b.stage}, {"solver_status", solved.stats.status}, {"solver", solved.stats.solver_name},
    {"solver_success", solved.stats.success}, {"variables", b.costs.size()}, {"binary_variables", b.model.binary_idx.size()},
    {"equalities", b.eq.size()}, {"inequalities", b.le.size()}, {"runtime_sec", solved.stats.runtime_sec},
    {"mip_gap", solved.stats.success ? J(solved.stats.mip_gap) : J(nullptr)},
    {"optimality_proven", solved.stats.success && solved.stats.status.find("optimal") != std::string::npos && solved.stats.mip_gap <= 1e-9}, {"feasible", false}};
  if (solved.x.size() != static_cast<Eigen::Index>(b.costs.size()) || !solved.x.allFinite()) return out;
  double residual = 0;
  J families = J::object(), binding = J::array();
  size_t binding_count = 0;
  const auto audit = [&](const Row& row, bool equality) {
    const double lhs = value(row.expr, solved.x), delta = lhs-row.rhs;
    const double violation = equality ? std::abs(delta) : std::max(0.0, delta);
    residual = std::max(residual, violation);
    const std::string family = row.name.substr(0, row.name.find('/'));
    if (!families.contains(family)) families[family] = {{"rows", 0}, {"max_violation", 0.0}};
    families[family]["rows"] = families[family]["rows"].get<int>()+1;
    families[family]["max_violation"] = std::max(families[family]["max_violation"].get<double>(), violation);
    if (violation > tolerance || (!equality && std::abs(delta) <= tolerance)) {
      ++binding_count;
      if (binding.size() < 1000) binding.push_back({{"name", row.name}, {"lhs", lhs}, {"rhs", row.rhs}, {"violation", violation}});
    }
  };
  for (const auto& r : b.eq) audit(r, true);
  for (const auto& r : b.le) audit(r, false);
  for (size_t i = 0; i < b.costs.size(); ++i) {
    const auto& v = b.model.linear_part.vars[i];
    residual = std::max({residual, v.lb-solved.x[i], solved.x[i]-v.ub});
    if (v.type == engine::VarType::Binary) residual = std::max(residual, std::abs(solved.x[i]-std::round(solved.x[i])));
  }
  out["max_residual"] = residual; out["constraint_families"] = families; out["binding_constraints"] = binding;
  out["binding_constraint_count"] = binding_count;
  out["binding_constraints_truncated"] = binding_count > binding.size();
  out["feasible"] = solved.stats.success && residual <= tolerance;
  out["objective"] = b.model.linear_part.c.dot(solved.x);
  J costs = J::object();
  for (size_t i = 0; i < b.costs.size(); ++i) {
    if (!costs.contains(b.cost_categories[i])) costs[b.cost_categories[i]] = 0.0;
    costs[b.cost_categories[i]] = costs[b.cost_categories[i]].get<double>()+b.costs[i]*solved.x[i];
  }
  out["objective_terms"] = costs;
  const auto rows = [&](const char* table, const std::vector<std::pair<std::string,std::string>>& fields) {
    J result = J::array();
    for (const auto& source : j.at(table)) {
      J row = {{"id", source.at("id")}, {"name", source.at("name")}};
      if (source.contains("bus")) row["bus"] = source.at("bus");
      if (source.contains("kind")) row["kind"] = source.at("kind");
      for (const auto& [field, prefix] : fields) {
        row[field] = J::array();
        for (int t = 0; t < T; ++t) {
          const auto k = key(prefix.c_str(), source.at("id"), t);
          if (b.columns.count(k)) row[field].push_back(solved.x[b.columns.at(k)]);
          else if (b.metrics.count(k)) row[field].push_back(value(b.metrics.at(k), solved.x));
          else row[field].push_back(nullptr);
        }
      }
      result.push_back(row);
    }
    return result;
  };
  out["generators"] = rows("generators", {{"power_mw", "p"}, {"online", "u"}, {"start", "start"}, {"stop", "stop"},
    {"hot_start", "start0"}, {"warm_start", "start1"}, {"cold_start", "start2"}, {"trajectory_mw", "trajectory"}, {"renewable_deviation_mw", "renewable_deviation"}});
  out["storage"] = rows("storage", {{"discharge_mw", "dis"}, {"charge_mw", "ch"}, {"energy_mwh", "energy"}});
  out["controllable_loads"] = rows("controllable_loads", {{"reduction_mw", "load_reduction"}});
  double reduced_energy = 0;
  for (const auto& d : out["controllable_loads"]) for (int t = 0; t < 96; ++t) reduced_energy += at(d, "reduction_mw", t)*0.25;
  out["day_load_reduction_mwh"] = reduced_energy;
  out["branches"] = rows("branches", {{"power_mw", "flow"}, {"slack_plus_mw", "line_slack_plus"}, {"slack_minus_mw", "line_slack_minus"}});
  out["sections"] = rows("sections", {{"power_mw", "section_flow"}, {"slack_plus_mw", "section_slack_plus"}, {"slack_minus_mw", "section_slack_minus"}});
  out["dc_links"] = rows("dc_links", {{"power_mw", "dc"}, {"up", "dc_up"}, {"down", "dc_down"}});
  out["reservoirs"] = rows("reservoirs", {{"level_m", "level"}, {"spill_m3_s", "spill"}, {"release_m3_s", "release"}});
  out["trades"] = rows("trades", {{"power_mw", "trade"}});
  for (auto& trade : out["trades"]) {
    const auto k = key("priority_shortfall_mwh", trade.at("id"), 0);
    trade["priority_shortfall_mwh"] = b.columns.count(k) ? J(solved.x[b.columns.at(k)]) : J(nullptr);
  }
  double day_energy_cost = 0, generation_mwh = 0;
  for (const auto& g : j.at("generators")) for (int t = 0; t < 96; ++t) {
    generation_mwh += solved.x[b.col("p", g.at("id"), t)]*0.25;
    for (size_t k = 0; k < g.at("segments").size(); ++k) {
      const int i = b.col(("segment"+std::to_string(k)).c_str(), g.at("id"), t);
      day_energy_cost += b.costs[i]*solved.x[i];
    }
  }
  out["day_energy_bid_cost"] = day_energy_cost; out["day_generation_mwh"] = generation_mwh;
  out["buses"] = J::array();
  const bool duals = b.stage == "lmp" && solved.stats.success && solved.constraint_duals.size() == static_cast<Eigen::Index>(b.eq.size()+b.le.size());
  out["prices_valid"] = duals && residual <= tolerance;
  for (const auto& node : j.at("buses")) {
    J row = {{"id", node.at("id")}, {"name", node.at("name")}, {"lmp_per_mwh", J::array()}};
    for (int t = 0; t < T; ++t) {
      const int index = static_cast<int>(b.le.size()) + b.balance_rows.at(key("bus", node.at("id"), t));
      // 2.6.6 uses the original balance multiplier. Reserve substitution added
      // balance to the upward row and subtracted it from the downward row;
      // undo this row operation (HiGHS upper-row shadows are -mu).
      const auto [up, down] = b.reserve_rows.at({node.at("area"), t});
      row["lmp_per_mwh"].push_back(duals ? J((solved.constraint_duals[index]+solved.constraint_duals[up]-solved.constraint_duals[down])/
        num(j.at("periods")[t], "weight_hr")) : J(nullptr));
    }
    out["buses"].push_back(row);
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
  for (int t = 0; t < T; ++t) {
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

J run_southern_day_ahead_market(const J& boundary) {
  const auto begin = std::chrono::steady_clock::now();
  const J effective = validate_southern_market(boundary);
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
  std::vector<SecurityCut> cuts;
  const auto finish = [&]() {
    out["runtime_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(); return out;
  };
  const bool ac_required = effective.at("execution").at("ac_security") == "required";
  const int iterations = ac_required ? effective.at("execution").at("security_iterations").get<int>() : 1;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    // Stage lifetime is bounded: keep solutions, not three sparse MILP builders.
    // Cost model and unchanged mathematical contract: southern_execution_contract.md.
    std::map<std::string,double> dispatch;
    for (const std::string stage : {"scuc", "sced"}) {
      const auto start = std::chrono::steady_clock::now();
      Build model;
      try { model = build_model(effective, stage, std::move(dispatch), cuts); }
      catch (const std::invalid_argument& e) {
        out["status"] = stage == "sced" ? "regulation_boundary_failed" : "scuc_failed";
        out["error"] = e.what(); return finish();
      }
      const double assembly = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      const auto solved = solve(model, effective); out[stage] = stage_result(model, solved, effective);
      out[stage]["assembly_sec"] = assembly;
      if (!out[stage].at("feasible").get<bool>()) { out["status"] = stage+"_failed"; return finish(); }
      dispatch = solution_map(model, solved);
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
    auto lmp = build_model(effective, "lmp", std::move(dispatch), cuts);
    const auto priced = solve(lmp, effective); out["lmp"] = stage_result(lmp, priced, effective);
    if (!out["lmp"].at("feasible").get<bool>() || !out["lmp"].value("prices_valid", false)) { out["status"] = "lmp_failed"; return finish(); }
    out["prices_valid"] = true; out["feasible"] = ac_required;
    out["status"] = ac_required ? "converged" : "schedule_only";
    return finish();
  }
  return finish();
}

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
