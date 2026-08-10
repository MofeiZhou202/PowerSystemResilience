// joint_optimizer_full_nlp.cpp
// ──────────────────────────────────────────────────────────────────────────
// Monolithic nonlinear Formulation-D extension.
//
// This source assembles one reduced finite-route nonlinear programme with:
//   • route/departure flows and unserved demand,
//   • endogenous BPR congestion in route costs,
//   • exact product Wardrop complementarity rows,
//   • aggregate charging/V2G/SOC constraints, and
//   • optional DC-OPF equations.
//
// The solve certificate is intentionally modest: the sparse local NLP path
// provides only a local NLP/MPEC stationarity certificate.  This is not a
// global nonlinear MIP gap certificate for the full CTM/LTM MPEC.

#include "hacdcpf/ev_power_traffic/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"

namespace hacdcpf::evpt {
namespace {

constexpr double kInf = 1.0e20;

struct FJColumn {
  enum class Kind {
    ChoiceFlow,
    UnservedDemand,
    DemandMinCost,
    ChargePower,
    DischargePower,
    BatteryEnergy,
    GeneratorPower,
    BusAngle,
    BranchFlow,
    PowerSlack,
  };

  Kind kind{Kind::ChoiceFlow};
  int demand_pos{-1};
  int choice_pos{-1};
  int group_pos{-1};
  int step{-1};
  int offset{-1};
  int gen_pos{-1};
  int bus_pos{-1};
  int branch_pos{-1};
};

struct FJChoice {
  int demand_pos{-1};
  const RouteAlternative* route{nullptr};
  int departure_step{0};
  int arrival_step{0};
  int h_col{-1};
  double vehicles_ub{0.0};
};

struct FJChargeGroup {
  int choice_pos{-1};
  int station_id{0};
  int arrival_step{0};
  int departure_step{0};
  int window{1};
  double requested_kwh_per_veh{0.0};
  double p_ch_max_per_veh{0.0};
  double p_dis_max_per_veh{0.0};
  double eta_ch{1.0};
  double eta_dis{1.0};
  double e_arrival_per_veh{0.0};
  double e_min_per_veh{0.0};
  double e_max_per_veh{0.0};
  double e_target_per_veh{0.0};
  bool v2g{false};
  std::vector<int> pch_cols;
  std::vector<int> pdis_cols;
  std::vector<int> energy_cols;
};

struct FJModel {
  const EVPowerTrafficProblem* problem{nullptr};
  JointOptimizerOptions opts;
  EVPowerTrafficOptions ev_opts;
  int T{1};
  double dt{1.0};

  std::unordered_map<int, std::size_t> link_pos;
  std::unordered_map<int, int> bus_pos;
  std::unordered_map<int, int> station_bus;
  std::unordered_map<int, double> station_cap_kw;

  std::vector<int> active_demands;
  std::unordered_map<int, int> demand_to_active;
  std::vector<FJChoice> choices;
  std::vector<FJChargeGroup> charge_groups;
  std::vector<FJColumn> cols;
  std::vector<int> unserved_col_by_active;
  std::vector<int> mu_col_by_active;

  std::vector<int> gen_base;
  std::vector<int> bus_angle_base;
  std::vector<int> branch_flow_base;
  std::vector<int> power_slack_base;
  std::vector<double> branch_b;
  int slack_bus_pos{0};

  std::vector<std::pair<int, int>> road_rows;       // (step, link_index)
  std::vector<std::pair<int, int>> station_rows;    // (step, station_id)
  std::vector<std::pair<int, int>> station_dis_rows;
};

struct DerivativeCheckStats {
  bool passed{false};
  double max_abs{0.0};
  double max_rel{0.0};
  int failures{0};
};

static double full_charge_power_kw_per_veh(const RouteChargingStop& stop,
                                           const JointOptimizerOptions& opts,
                                           double dt_hr) {
  if (stop.max_charge_kw_per_vehicle > kTol) {
    return stop.max_charge_kw_per_vehicle;
  }
  const double dwell_hr = std::max(1, stop.dwell_steps) * dt_hr;
  if (stop.requested_energy_kwh_per_vehicle > kTol && dwell_hr > kTol) {
    return stop.requested_energy_kwh_per_vehicle / dwell_hr;
  }
  return opts.default_route_stop_power_kw_per_vehicle;
}

static double route_drive_energy_kwh_per_veh(
    const RouteAlternative& route,
    const EVPowerTrafficProblem& problem,
    const std::unordered_map<int, std::size_t>& link_pos) {
  double energy = 0.0;
  for (int link_index : route.link_indices) {
    const auto it = link_pos.find(link_index);
    if (it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[it->second];
    energy += std::max(0.0, link.drive_energy_kwh_per_veh_km) *
              std::max(0.0, link.length_km);
  }
  return energy;
}

static double station_effective_cap_kw(const HybridPowerSystem& sys,
                                       const JointOptimizerOptions& opts,
                                       int station_id) {
  for (const auto& cs : sys.ac.charging_stations) {
    if (cs.index != station_id) continue;
    if (!cs.in_service) return 0.0;
    double cap = cs.max_power_kw > 0.0 ? cs.max_power_kw
               : static_cast<double>(cs.n_fast) * cs.p_fast_max_kw +
                 static_cast<double>(cs.n_slow) * cs.p_slow_max_kw;
    if (cap <= kTol) cap = opts.default_station_power_kw;
    return std::max(0.0, cap * std::clamp(cs.simultaneity_factor, 0.0, 1.0));
  }
  return opts.default_station_power_kw;
}

static std::vector<int> departure_steps_for(const EVDemand& demand) {
  if (!demand.departure_window_steps.empty()) {
    return demand.departure_window_steps;
  }
  return {demand.departure_step};
}

static double finite_or(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

static double choice_capacity_ub(const FJModel& m, const FJChoice& choice) {
  if (!m.opts.enforce_road_capacity || choice.route == nullptr) {
    return choice.vehicles_ub;
  }
  double ub = choice.vehicles_ub;
  for (int link_index : choice.route->link_indices) {
    const auto it = m.link_pos.find(link_index);
    if (it == m.link_pos.end()) return 0.0;
    const double cap =
        link_capacity_vehicles(m.problem->traffic.links[it->second],
                               choice.departure_step, m.dt);
    ub = std::min(ub, cap);
  }
  return std::max(0.0, finite_or(ub, 0.0));
}

static int add_col(FJModel& m,
                   engine::NLPModel& nlp,
                   FJColumn col,
                   double lb,
                   double ub,
                   const std::string& name) {
  const int idx = static_cast<int>(m.cols.size());
  m.cols.push_back(std::move(col));
  nlp.vars.push_back({engine::VarType::Continuous, lb, ub, name});
  return idx;
}

static double h_value(const FJModel& m, const Eigen::VectorXd& x, int choice_pos) {
  if (choice_pos < 0 || choice_pos >= static_cast<int>(m.choices.size())) return 0.0;
  const int col = m.choices[static_cast<std::size_t>(choice_pos)].h_col;
  if (col < 0 || col >= x.size()) return 0.0;
  return std::max(0.0, finite_or(x[col], 0.0));
}

static double link_flow(const FJModel& m,
                        const Eigen::VectorXd& x,
                        int step,
                        int link_index) {
  double flow = 0.0;
  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    if (choice.departure_step != step || choice.route == nullptr) continue;
    if (std::find(choice.route->link_indices.begin(),
                  choice.route->link_indices.end(),
                  link_index) != choice.route->link_indices.end()) {
      flow += h_value(m, x, ci);
    }
  }
  return flow;
}

static bool choice_uses_link(const FJChoice& choice, int link_index) {
  return choice.route != nullptr &&
         std::find(choice.route->link_indices.begin(),
                   choice.route->link_indices.end(),
                   link_index) != choice.route->link_indices.end();
}

static double link_travel_time_derivative_wrt_flow(const TrafficLink& link,
                                                   double inflow_vehicles,
                                                   double capacity_vehicles) {
  if (capacity_vehicles <= kTol) return 0.0;
  const double t0 = std::max(link.free_flow_time_hr, 0.0);
  if (t0 <= kTol || link.alpha == 0.0 || link.beta <= 0.0) return 0.0;
  const double ratio = std::max(0.0, inflow_vehicles / capacity_vehicles);
  if (ratio <= kTol && link.beta < 1.0) return 0.0;
  const double pow_term = (ratio <= kTol && std::abs(link.beta - 1.0) <= 1.0e-12)
      ? 1.0
      : std::pow(ratio, link.beta - 1.0);
  return t0 * link.alpha * link.beta * pow_term / capacity_vehicles;
}

static double route_travel_time_endogenous(const FJModel& m,
                                           const Eigen::VectorXd& x,
                                           const FJChoice& choice) {
  if (choice.route == nullptr) return kInf;
  double tt = 0.0;
  for (int link_index : choice.route->link_indices) {
    const auto it = m.link_pos.find(link_index);
    if (it == m.link_pos.end()) return kInf;
    const auto& link = m.problem->traffic.links[it->second];
    const double cap = link_capacity_vehicles(link, choice.departure_step, m.dt);
    if (cap <= kTol) return kInf;
    const double flow = link_flow(m, x, choice.departure_step, link_index);
    tt += link_travel_time_hr(link, flow, cap);
  }
  return tt;
}

static double route_travel_time_derivative_wrt_choice(const FJModel& m,
                                                      const Eigen::VectorXd& x,
                                                      const FJChoice& route_choice,
                                                      int flow_choice_pos) {
  if (route_choice.route == nullptr ||
      flow_choice_pos < 0 ||
      flow_choice_pos >= static_cast<int>(m.choices.size())) {
    return 0.0;
  }
  const auto& flow_choice = m.choices[static_cast<std::size_t>(flow_choice_pos)];
  if (flow_choice.departure_step != route_choice.departure_step) return 0.0;

  double deriv = 0.0;
  for (int link_index : route_choice.route->link_indices) {
    if (!choice_uses_link(flow_choice, link_index)) continue;
    const auto it = m.link_pos.find(link_index);
    if (it == m.link_pos.end()) continue;
    const auto& link = m.problem->traffic.links[it->second];
    const double cap = link_capacity_vehicles(link, route_choice.departure_step, m.dt);
    if (cap <= kTol) continue;
    const double flow = link_flow(m, x, route_choice.departure_step, link_index);
    deriv += link_travel_time_derivative_wrt_flow(link, flow, cap);
  }
  return deriv;
}

static double schedule_delay_derivative_wrt_arrival(double t_arr_hr,
                                                    double t_star_hr,
                                                    double gamma_early,
                                                    double gamma_late) {
  if (t_star_hr < 0.0) return 0.0;
  if (t_arr_hr < t_star_hr - 1.0e-10) return -gamma_early;
  if (t_arr_hr > t_star_hr + 1.0e-10) return gamma_late;
  return 0.0;
}

static double choice_generalized_cost(const FJModel& m,
                                      const Eigen::VectorXd& x,
                                      int choice_pos) {
  const auto& choice = m.choices[static_cast<std::size_t>(choice_pos)];
  const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
  const double tt = route_travel_time_endogenous(m, x, choice);
  if (!std::isfinite(tt)) return 1.0e9;

  const double vot = demand.value_of_time_per_hr > kTol
      ? demand.value_of_time_per_hr
      : m.opts.value_of_time_per_hr;
  double cost = vot * tt + (choice.route ? choice.route->toll_cost : 0.0);
  const double arr_hr = choice.departure_step * m.dt + tt;
  cost += schedule_delay_cost(arr_hr, demand.desired_arrival_time_hr,
                              demand.early_penalty_per_hr,
                              demand.late_penalty_per_hr);
  if (choice.route) {
    for (const auto& stop : choice.route->charging_stops) {
      const int price_step = std::clamp(choice.arrival_step, 0, m.T - 1);
      const double price =
          station_price(*m.problem, m.ev_opts, stop.station_id, price_step);
      cost += m.opts.station_energy_cost_weight * price *
              stop.requested_energy_kwh_per_vehicle;
      if (stop.v2g_capable &&
          stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
        cost -= m.opts.station_energy_cost_weight * price *
                stop.requested_discharge_energy_kwh_per_vehicle;
      }
    }
  }
  return cost;
}

static double choice_cost_derivative_wrt_choice(const FJModel& m,
                                                const Eigen::VectorXd& x,
                                                int cost_choice_pos,
                                                int flow_choice_pos) {
  if (cost_choice_pos < 0 ||
      cost_choice_pos >= static_cast<int>(m.choices.size())) {
    return 0.0;
  }
  const auto& choice = m.choices[static_cast<std::size_t>(cost_choice_pos)];
  const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
  const double dtt =
      route_travel_time_derivative_wrt_choice(m, x, choice, flow_choice_pos);
  if (dtt == 0.0) return 0.0;
  const double vot = demand.value_of_time_per_hr > kTol
      ? demand.value_of_time_per_hr
      : m.opts.value_of_time_per_hr;
  const double tt = route_travel_time_endogenous(m, x, choice);
  if (!std::isfinite(tt)) return 0.0;
  const double arr_hr = choice.departure_step * m.dt + tt;
  const double sched_deriv = schedule_delay_derivative_wrt_arrival(
      arr_hr, demand.desired_arrival_time_hr,
      demand.early_penalty_per_hr, demand.late_penalty_per_hr);
  return (vot + sched_deriv) * dtt;
}

static double demand_mu(const FJModel& m, const Eigen::VectorXd& x, int demand_pos) {
  const auto it = m.demand_to_active.find(demand_pos);
  if (it == m.demand_to_active.end()) return 0.0;
  const int mu_col = m.mu_col_by_active[static_cast<std::size_t>(it->second)];
  return (mu_col >= 0 && mu_col < x.size()) ? finite_or(x[mu_col], 0.0) : 0.0;
}

static double power_slack_penalty_term(const FJModel& m, const Eigen::VectorXd& x) {
  if (!m.opts.include_dcopf) return 0.0;
  double term = 0.0;
  for (int base : m.power_slack_base) {
    if (base < 0) continue;
    for (int k = 0; k < m.T; ++k) {
      term += std::max(0.0, finite_or(x[base + k], 0.0)) *
              m.opts.power_slack_penalty;
    }
  }
  return term;
}

static double full_objective(const FJModel& m, const Eigen::VectorXd& x) {
  const bool user_mode =
      m.opts.mode == JointOptimizerMode::FullJointUserBenefitNLP;
  double obj = 0.0;

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
    const double h = h_value(m, x, ci);
    const double tt = route_travel_time_endogenous(m, x, choice);
    const double vot = demand.value_of_time_per_hr > kTol
        ? demand.value_of_time_per_hr
        : m.opts.value_of_time_per_hr;
    obj += h * vot * finite_or(tt, 1.0e6);
    obj -= h * demand.willingness_to_pay_per_vehicle;
  }

  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int ucol = m.unserved_col_by_active[static_cast<std::size_t>(ai)];
    const double outside = user_mode ? m.opts.outside_option_cost : 0.0;
    obj += std::max(0.0, finite_or(x[ucol], 0.0)) *
           (m.opts.unserved_trip_penalty + outside);
  }

  if (user_mode) {
    for (const auto& group : m.charge_groups) {
      for (int off = 0; off < group.window; ++off) {
        const int k = group.arrival_step + off;
        const double price = station_price(*m.problem, m.ev_opts,
                                           group.station_id,
                                           std::clamp(k, 0, m.T - 1));
        obj += m.opts.station_energy_cost_weight * price * m.dt *
               (finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0) -
                finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0));
      }
    }
  } else if (m.opts.include_dcopf) {
    const auto& gens = m.problem->system.ac.generators;
    for (int gi = 0; gi < static_cast<int>(gens.size()); ++gi) {
      const int base = m.gen_base[static_cast<std::size_t>(gi)];
      if (base < 0) continue;
      for (int k = 0; k < m.T; ++k) {
        obj += gens[static_cast<std::size_t>(gi)].cost_c1 *
               finite_or(x[base + k], 0.0) * m.dt;
      }
    }
  }

  obj += power_slack_penalty_term(m, x);

  const double eq_pen = std::max(0.0, m.opts.full_joint_equilibrium_penalty);
  if (eq_pen > 0.0) {
    double ss = 0.0;
    for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
      const auto& choice = m.choices[static_cast<std::size_t>(ci)];
      const double h = h_value(m, x, ci);
      const double gap =
          choice_generalized_cost(m, x, ci) - demand_mu(m, x, choice.demand_pos);
      const double comp = h * gap;
      ss += comp * comp;
      const double neg_gap = std::max(0.0, -gap);
      ss += neg_gap * neg_gap;
    }
    obj += eq_pen * ss;
  }
  return obj;
}

static void add_triplet(std::vector<Eigen::Triplet<double>>& trips,
                        int row,
                        int col,
                        double value) {
  if (row >= 0 && col >= 0 && std::isfinite(value) && std::abs(value) > 1.0e-14) {
    trips.emplace_back(row, col, value);
  }
}

static void full_objective_gradient(const FJModel& m,
                                    const Eigen::VectorXd& x,
                                    Eigen::VectorXd& grad) {
  const int n = static_cast<int>(m.cols.size());
  grad = Eigen::VectorXd::Zero(n);
  const bool user_mode =
      m.opts.mode == JointOptimizerMode::FullJointUserBenefitNLP;

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
    const int h_col = choice.h_col;
    const double h = h_value(m, x, ci);
    const double tt = route_travel_time_endogenous(m, x, choice);
    const double tt_finite = finite_or(tt, 1.0e6);
    const double vot = demand.value_of_time_per_hr > kTol
        ? demand.value_of_time_per_hr
        : m.opts.value_of_time_per_hr;
    if (h_col >= 0 && h_col < n) {
      grad[h_col] += vot * tt_finite - demand.willingness_to_pay_per_vehicle;
    }
    for (int cj = 0; cj < static_cast<int>(m.choices.size()); ++cj) {
      const int hj_col = m.choices[static_cast<std::size_t>(cj)].h_col;
      if (hj_col < 0 || hj_col >= n) continue;
      const double dtt = route_travel_time_derivative_wrt_choice(m, x, choice, cj);
      if (dtt != 0.0) grad[hj_col] += h * vot * dtt;
    }
  }

  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int ucol = m.unserved_col_by_active[static_cast<std::size_t>(ai)];
    if (ucol < 0 || ucol >= n) continue;
    const double outside = user_mode ? m.opts.outside_option_cost : 0.0;
    grad[ucol] += m.opts.unserved_trip_penalty + outside;
  }

  if (user_mode) {
    for (const auto& group : m.charge_groups) {
      for (int off = 0; off < group.window; ++off) {
        const int k = group.arrival_step + off;
        const double price = station_price(*m.problem, m.ev_opts,
                                           group.station_id,
                                           std::clamp(k, 0, m.T - 1));
        const double coef = m.opts.station_energy_cost_weight * price * m.dt;
        const int pch = group.pch_cols[static_cast<std::size_t>(off)];
        const int pdis = group.pdis_cols[static_cast<std::size_t>(off)];
        if (pch >= 0 && pch < n) grad[pch] += coef;
        if (pdis >= 0 && pdis < n) grad[pdis] -= coef;
      }
    }
  } else if (m.opts.include_dcopf) {
    const auto& gens = m.problem->system.ac.generators;
    for (int gi = 0; gi < static_cast<int>(gens.size()); ++gi) {
      const int base = m.gen_base[static_cast<std::size_t>(gi)];
      if (base < 0) continue;
      for (int k = 0; k < m.T; ++k) {
        grad[base + k] += gens[static_cast<std::size_t>(gi)].cost_c1 * m.dt;
      }
    }
  }

  if (m.opts.include_dcopf) {
    for (int base : m.power_slack_base) {
      if (base < 0) continue;
      for (int k = 0; k < m.T; ++k) {
        grad[base + k] += m.opts.power_slack_penalty;
      }
    }
  }

  const double eq_pen = std::max(0.0, m.opts.full_joint_equilibrium_penalty);
  if (eq_pen <= 0.0) return;

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const int hi_col = choice.h_col;
    const double h = h_value(m, x, ci);
    const double gap =
        choice_generalized_cost(m, x, ci) - demand_mu(m, x, choice.demand_pos);
    const double comp = h * gap;

    for (int cj = 0; cj < static_cast<int>(m.choices.size()); ++cj) {
      const int hj_col = m.choices[static_cast<std::size_t>(cj)].h_col;
      if (hj_col < 0 || hj_col >= n) continue;
      const double dcost = choice_cost_derivative_wrt_choice(m, x, ci, cj);
      const double dcomp = (cj == ci ? gap : 0.0) + h * dcost;
      grad[hj_col] += eq_pen * 2.0 * comp * dcomp;
      if (gap < 0.0) {
        const double neg_gap = -gap;
        grad[hj_col] += eq_pen * (-2.0 * neg_gap * dcost);
      }
    }

    const auto ait = m.demand_to_active.find(choice.demand_pos);
    if (ait != m.demand_to_active.end()) {
      const int mu_col = m.mu_col_by_active[static_cast<std::size_t>(ait->second)];
      if (mu_col >= 0 && mu_col < n) {
        grad[mu_col] += eq_pen * 2.0 * comp * (-h);
        if (gap < 0.0) grad[mu_col] += eq_pen * 2.0 * (-gap);
      }
    }
    (void)hi_col;
  }
}

static void full_equalities(const FJModel& m,
                            const Eigen::VectorXd& x,
                            Eigen::VectorXd& g) {
  const int n_dem = static_cast<int>(m.active_demands.size());
  const int n_energy_eq = [&]() {
    int count = 0;
    for (const auto& group : m.charge_groups) count += 1 + group.window;
    return count;
  }();
  const int n_flow_eq = m.opts.include_dcopf
      ? static_cast<int>(m.problem->system.ac.branches.size()) * m.T
      : 0;
  const int n_pbal_eq = m.opts.include_dcopf
      ? static_cast<int>(m.problem->system.ac.buses.size()) * m.T
      : 0;
  const int n_comp = static_cast<int>(m.choices.size());
  g = Eigen::VectorXd::Zero(n_dem + n_energy_eq + n_flow_eq + n_pbal_eq + n_comp);

  int row = 0;
  for (int ai = 0; ai < n_dem; ++ai, ++row) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(dpos)];
    double sum = finite_or(x[m.unserved_col_by_active[static_cast<std::size_t>(ai)]], 0.0);
    for (const auto& choice : m.choices) {
      if (choice.demand_pos == dpos) sum += h_value(m, x, static_cast<int>(&choice - m.choices.data()));
    }
    g[row] = sum - demand.vehicles;
  }

  for (const auto& group : m.charge_groups) {
    const double h = h_value(m, x, group.choice_pos);
    g[row++] = finite_or(x[group.energy_cols.front()], 0.0) -
               group.e_arrival_per_veh * h;
    for (int off = 0; off < group.window; ++off) {
      const double e0 = finite_or(x[group.energy_cols[static_cast<std::size_t>(off)]], 0.0);
      const double e1 = finite_or(x[group.energy_cols[static_cast<std::size_t>(off + 1)]], 0.0);
      const double pch = finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0);
      const double pdis = finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0);
      g[row++] = e1 - e0 - group.eta_ch * pch * m.dt +
                 pdis * m.dt / std::max(group.eta_dis, kTol);
    }
  }

  if (m.opts.include_dcopf) {
    const auto& sys = m.problem->system;
    for (int li = 0; li < static_cast<int>(sys.ac.branches.size()); ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      for (int k = 0; k < m.T; ++k, ++row) {
        if (!br.in_service || m.branch_flow_base[static_cast<std::size_t>(li)] < 0) {
          g[row] = 0.0;
          continue;
        }
        const auto bi_it = m.bus_pos.find(br.from_bus);
        const auto bj_it = m.bus_pos.find(br.to_bus);
        if (bi_it == m.bus_pos.end() || bj_it == m.bus_pos.end()) {
          g[row] = 0.0;
          continue;
        }
        const double f = finite_or(x[m.branch_flow_base[static_cast<std::size_t>(li)] + k], 0.0);
        const double ti = finite_or(x[m.bus_angle_base[static_cast<std::size_t>(bi_it->second)] + k], 0.0);
        const double tj = finite_or(x[m.bus_angle_base[static_cast<std::size_t>(bj_it->second)] + k], 0.0);
        const double b = m.branch_b[static_cast<std::size_t>(li)];
        g[row] = f - b * ti + b * tj;
      }
    }

    for (int bi = 0; bi < static_cast<int>(sys.ac.buses.size()); ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      for (int k = 0; k < m.T; ++k, ++row) {
        double lhs = 0.0;
        if (m.power_slack_base[static_cast<std::size_t>(bi)] >= 0) {
          lhs += finite_or(x[m.power_slack_base[static_cast<std::size_t>(bi)] + k], 0.0);
        }
        for (int gi = 0; gi < static_cast<int>(sys.ac.generators.size()); ++gi) {
          const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
          if (!gen.in_service || gen.bus != bus.index ||
              m.gen_base[static_cast<std::size_t>(gi)] < 0) {
            continue;
          }
          lhs += finite_or(x[m.gen_base[static_cast<std::size_t>(gi)] + k], 0.0);
        }
        for (int li = 0; li < static_cast<int>(sys.ac.branches.size()); ++li) {
          const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
          if (!br.in_service || m.branch_flow_base[static_cast<std::size_t>(li)] < 0) {
            continue;
          }
          const double f = finite_or(x[m.branch_flow_base[static_cast<std::size_t>(li)] + k], 0.0);
          if (br.from_bus == bus.index) lhs -= f;
          if (br.to_bus == bus.index) lhs += f;
        }
        for (const auto& group : m.charge_groups) {
          const auto st_it = m.station_bus.find(group.station_id);
          if (st_it == m.station_bus.end() || st_it->second != bus.index) continue;
          for (int off = 0; off < group.window; ++off) {
            if (group.arrival_step + off != k) continue;
            lhs -= finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0) / 1000.0;
            lhs += finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0) / 1000.0;
          }
        }
        g[row] = lhs - bus.pd_mw;
      }
    }
  }

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci, ++row) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const double h = h_value(m, x, ci);
    const double gap =
        choice_generalized_cost(m, x, ci) - demand_mu(m, x, choice.demand_pos);
    g[row] = h * gap;
  }
}

static void full_inequalities(const FJModel& m,
                              const Eigen::VectorXd& x,
                              Eigen::VectorXd& h) {
  const int n_gap = static_cast<int>(m.choices.size());
  int n_rows = n_gap + static_cast<int>(m.road_rows.size()) +
               static_cast<int>(m.station_rows.size()) +
               static_cast<int>(m.station_dis_rows.size());
  for (const auto& group : m.charge_groups) {
    n_rows += 2 * group.window + 2 * static_cast<int>(group.energy_cols.size()) + 1;
  }
  h = Eigen::VectorXd::Zero(n_rows);
  int row = 0;

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci, ++row) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const double gap =
        choice_generalized_cost(m, x, ci) - demand_mu(m, x, choice.demand_pos);
    h[row] = -gap;  // C_i - mu_d >= 0
  }

  for (const auto& [step, link_index] : m.road_rows) {
    const auto it = m.link_pos.find(link_index);
    const double cap = (it == m.link_pos.end())
        ? 0.0
        : link_capacity_vehicles(m.problem->traffic.links[it->second], step, m.dt);
    h[row++] = link_flow(m, x, step, link_index) - cap;
  }

  for (const auto& [step, station_id] : m.station_rows) {
    double used = 0.0;
    for (const auto& group : m.charge_groups) {
      if (group.station_id != station_id) continue;
      for (int off = 0; off < group.window; ++off) {
        if (group.arrival_step + off == step) {
          used += finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0);
        }
      }
    }
    const double cap = m.station_cap_kw.count(station_id)
        ? m.station_cap_kw.at(station_id)
        : m.opts.default_station_power_kw;
    h[row++] = used - cap;
  }

  for (const auto& [step, station_id] : m.station_dis_rows) {
    double used = 0.0;
    for (const auto& group : m.charge_groups) {
      if (group.station_id != station_id) continue;
      for (int off = 0; off < group.window; ++off) {
        if (group.arrival_step + off == step) {
          used += finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0);
        }
      }
    }
    const double cap = m.station_cap_kw.count(station_id)
        ? m.station_cap_kw.at(station_id)
        : m.opts.default_station_power_kw;
    h[row++] = used - cap;
  }

  for (const auto& group : m.charge_groups) {
    const double flow = h_value(m, x, group.choice_pos);
    for (int off = 0; off < group.window; ++off) {
      h[row++] = finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0) -
                 group.p_ch_max_per_veh * flow;
      h[row++] = finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0) -
                 group.p_dis_max_per_veh * flow;
    }
    for (int ecol : group.energy_cols) {
      const double e = finite_or(x[ecol], 0.0);
      h[row++] = group.e_min_per_veh * flow - e;
      h[row++] = e - group.e_max_per_veh * flow;
    }
    h[row++] = group.e_target_per_veh * flow -
               finite_or(x[group.energy_cols.back()], 0.0);
  }
}

static void full_equality_jacobian(const FJModel& m,
                                   const Eigen::VectorXd& x,
                                   Eigen::SparseMatrix<double>& jac) {
  Eigen::VectorXd g_dummy;
  full_equalities(m, x, g_dummy);
  const int n_rows = static_cast<int>(g_dummy.size());
  const int n_cols = static_cast<int>(m.cols.size());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(std::max(1, n_rows) * 4));

  int row = 0;
  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai, ++row) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    add_triplet(trips, row, m.unserved_col_by_active[static_cast<std::size_t>(ai)], 1.0);
    for (const auto& choice : m.choices) {
      if (choice.demand_pos == dpos) add_triplet(trips, row, choice.h_col, 1.0);
    }
  }

  for (const auto& group : m.charge_groups) {
    const int h_col = m.choices[static_cast<std::size_t>(group.choice_pos)].h_col;
    add_triplet(trips, row, group.energy_cols.front(), 1.0);
    add_triplet(trips, row, h_col, -group.e_arrival_per_veh);
    ++row;
    for (int off = 0; off < group.window; ++off, ++row) {
      add_triplet(trips, row, group.energy_cols[static_cast<std::size_t>(off + 1)], 1.0);
      add_triplet(trips, row, group.energy_cols[static_cast<std::size_t>(off)], -1.0);
      add_triplet(trips, row, group.pch_cols[static_cast<std::size_t>(off)],
                  -group.eta_ch * m.dt);
      add_triplet(trips, row, group.pdis_cols[static_cast<std::size_t>(off)],
                  m.dt / std::max(group.eta_dis, kTol));
    }
  }

  if (m.opts.include_dcopf) {
    const auto& sys = m.problem->system;
    for (int li = 0; li < static_cast<int>(sys.ac.branches.size()); ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      for (int k = 0; k < m.T; ++k, ++row) {
        if (!br.in_service || m.branch_flow_base[static_cast<std::size_t>(li)] < 0) {
          continue;
        }
        const auto bi_it = m.bus_pos.find(br.from_bus);
        const auto bj_it = m.bus_pos.find(br.to_bus);
        if (bi_it == m.bus_pos.end() || bj_it == m.bus_pos.end()) continue;
        const double b = m.branch_b[static_cast<std::size_t>(li)];
        add_triplet(trips, row, m.branch_flow_base[static_cast<std::size_t>(li)] + k, 1.0);
        add_triplet(trips, row, m.bus_angle_base[static_cast<std::size_t>(bi_it->second)] + k, -b);
        add_triplet(trips, row, m.bus_angle_base[static_cast<std::size_t>(bj_it->second)] + k, b);
      }
    }

    for (int bi = 0; bi < static_cast<int>(sys.ac.buses.size()); ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      for (int k = 0; k < m.T; ++k, ++row) {
        if (m.power_slack_base[static_cast<std::size_t>(bi)] >= 0) {
          add_triplet(trips, row,
                      m.power_slack_base[static_cast<std::size_t>(bi)] + k,
                      1.0);
        }
        for (int gi = 0; gi < static_cast<int>(sys.ac.generators.size()); ++gi) {
          const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
          if (!gen.in_service || gen.bus != bus.index ||
              m.gen_base[static_cast<std::size_t>(gi)] < 0) {
            continue;
          }
          add_triplet(trips, row, m.gen_base[static_cast<std::size_t>(gi)] + k, 1.0);
        }
        for (int li = 0; li < static_cast<int>(sys.ac.branches.size()); ++li) {
          const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
          if (!br.in_service || m.branch_flow_base[static_cast<std::size_t>(li)] < 0) {
            continue;
          }
          if (br.from_bus == bus.index) {
            add_triplet(trips, row,
                        m.branch_flow_base[static_cast<std::size_t>(li)] + k,
                        -1.0);
          }
          if (br.to_bus == bus.index) {
            add_triplet(trips, row,
                        m.branch_flow_base[static_cast<std::size_t>(li)] + k,
                        1.0);
          }
        }
        for (const auto& group : m.charge_groups) {
          const auto st_it = m.station_bus.find(group.station_id);
          if (st_it == m.station_bus.end() || st_it->second != bus.index) continue;
          for (int off = 0; off < group.window; ++off) {
            if (group.arrival_step + off != k) continue;
            add_triplet(trips, row, group.pch_cols[static_cast<std::size_t>(off)],
                        -1.0 / 1000.0);
            add_triplet(trips, row, group.pdis_cols[static_cast<std::size_t>(off)],
                        1.0 / 1000.0);
          }
        }
      }
    }
  }

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci, ++row) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const double h = h_value(m, x, ci);
    const double gap =
        choice_generalized_cost(m, x, ci) - demand_mu(m, x, choice.demand_pos);
    for (int cj = 0; cj < static_cast<int>(m.choices.size()); ++cj) {
      const int hj_col = m.choices[static_cast<std::size_t>(cj)].h_col;
      const double dcost = choice_cost_derivative_wrt_choice(m, x, ci, cj);
      add_triplet(trips, row, hj_col, (cj == ci ? gap : 0.0) + h * dcost);
    }
    const auto ait = m.demand_to_active.find(choice.demand_pos);
    if (ait != m.demand_to_active.end()) {
      add_triplet(trips, row,
                  m.mu_col_by_active[static_cast<std::size_t>(ait->second)],
                  -h);
    }
  }

  jac.resize(n_rows, n_cols);
  jac.setFromTriplets(trips.begin(), trips.end());
  jac.makeCompressed();
}

static void full_inequality_jacobian(const FJModel& m,
                                     const Eigen::VectorXd& x,
                                     Eigen::SparseMatrix<double>& jac) {
  Eigen::VectorXd h_dummy;
  full_inequalities(m, x, h_dummy);
  const int n_rows = static_cast<int>(h_dummy.size());
  const int n_cols = static_cast<int>(m.cols.size());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(std::max(1, n_rows) * 3));

  int row = 0;
  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci, ++row) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    for (int cj = 0; cj < static_cast<int>(m.choices.size()); ++cj) {
      const int hj_col = m.choices[static_cast<std::size_t>(cj)].h_col;
      add_triplet(trips, row, hj_col,
                  -choice_cost_derivative_wrt_choice(m, x, ci, cj));
    }
    const auto ait = m.demand_to_active.find(choice.demand_pos);
    if (ait != m.demand_to_active.end()) {
      add_triplet(trips, row,
                  m.mu_col_by_active[static_cast<std::size_t>(ait->second)],
                  1.0);
    }
  }

  for (const auto& [step, link_index] : m.road_rows) {
    for (const auto& choice : m.choices) {
      if (choice.departure_step == step && choice_uses_link(choice, link_index)) {
        add_triplet(trips, row, choice.h_col, 1.0);
      }
    }
    ++row;
  }

  for (const auto& [step, station_id] : m.station_rows) {
    for (const auto& group : m.charge_groups) {
      if (group.station_id != station_id) continue;
      for (int off = 0; off < group.window; ++off) {
        if (group.arrival_step + off == step) {
          add_triplet(trips, row, group.pch_cols[static_cast<std::size_t>(off)], 1.0);
        }
      }
    }
    ++row;
  }

  for (const auto& [step, station_id] : m.station_dis_rows) {
    for (const auto& group : m.charge_groups) {
      if (group.station_id != station_id) continue;
      for (int off = 0; off < group.window; ++off) {
        if (group.arrival_step + off == step) {
          add_triplet(trips, row, group.pdis_cols[static_cast<std::size_t>(off)], 1.0);
        }
      }
    }
    ++row;
  }

  for (const auto& group : m.charge_groups) {
    const int h_col = m.choices[static_cast<std::size_t>(group.choice_pos)].h_col;
    for (int off = 0; off < group.window; ++off) {
      add_triplet(trips, row, group.pch_cols[static_cast<std::size_t>(off)], 1.0);
      add_triplet(trips, row, h_col, -group.p_ch_max_per_veh);
      ++row;
      add_triplet(trips, row, group.pdis_cols[static_cast<std::size_t>(off)], 1.0);
      add_triplet(trips, row, h_col, -group.p_dis_max_per_veh);
      ++row;
    }
    for (int ecol : group.energy_cols) {
      add_triplet(trips, row, h_col, group.e_min_per_veh);
      add_triplet(trips, row, ecol, -1.0);
      ++row;
      add_triplet(trips, row, ecol, 1.0);
      add_triplet(trips, row, h_col, -group.e_max_per_veh);
      ++row;
    }
    add_triplet(trips, row, h_col, group.e_target_per_veh);
    add_triplet(trips, row, group.energy_cols.back(), -1.0);
    ++row;
  }

  jac.resize(n_rows, n_cols);
  jac.setFromTriplets(trips.begin(), trips.end());
  jac.makeCompressed();
}

static Eigen::VectorXd finite_difference_gradient(
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<engine::VariableMeta>& vars,
    const Eigen::VectorXd& x,
    double fd_step) {
  const int n = static_cast<int>(x.size());
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);
  const double f0 = f(x);
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd xp = x;
    double step = std::max(fd_step, fd_step * std::max(1.0, std::abs(x[j])));
    const double ub = vars[static_cast<std::size_t>(j)].ub;
    const double lb = vars[static_cast<std::size_t>(j)].lb;
    if (xp[j] + step <= ub) {
      xp[j] += step;
    } else if (xp[j] - step >= lb) {
      step = -step;
      xp[j] += step;
    } else {
      continue;
    }
    grad[j] = (f(xp) - f0) / step;
  }
  return grad;
}

static void finite_difference_jacobian(
    const std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)>& fun,
    const std::vector<engine::VariableMeta>& vars,
    const Eigen::VectorXd& x,
    double fd_step,
    Eigen::SparseMatrix<double>& jac) {
  Eigen::VectorXd f0;
  fun(x, f0);
  const int m = static_cast<int>(f0.size());
  const int n = static_cast<int>(x.size());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(std::max(1, m) * std::min(n, 16)));
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd xp = x;
    double step = std::max(fd_step, fd_step * std::max(1.0, std::abs(x[j])));
    const double ub = vars[static_cast<std::size_t>(j)].ub;
    const double lb = vars[static_cast<std::size_t>(j)].lb;
    if (xp[j] + step <= ub) {
      xp[j] += step;
    } else if (xp[j] - step >= lb) {
      step = -step;
      xp[j] += step;
    } else {
      continue;
    }
    Eigen::VectorXd fp;
    fun(xp, fp);
    if (fp.size() != m) continue;
    for (int i = 0; i < m; ++i) {
      const double v = (fp[i] - f0[i]) / step;
      if (std::abs(v) > 1.0e-12) trips.emplace_back(i, j, v);
    }
  }
  jac.resize(m, n);
  jac.setFromTriplets(trips.begin(), trips.end());
  jac.makeCompressed();
}

static double sparse_coeff(const Eigen::SparseMatrix<double>& mat,
                           int row,
                           int col) {
  if (row < 0 || row >= mat.rows() || col < 0 || col >= mat.cols()) return 0.0;
  return mat.coeff(row, col);
}

static bool derivative_check_skip_col(const std::vector<engine::VariableMeta>& vars,
                                      int col) {
  if (col < 0 || col >= static_cast<int>(vars.size())) return true;
  const auto& v = vars[static_cast<std::size_t>(col)];
  return std::isfinite(v.lb) && std::isfinite(v.ub) && std::abs(v.ub - v.lb) <= 1.0e-14;
}

static void accumulate_derivative_error(double analytic,
                                        double reference,
                                        double tol,
                                        DerivativeCheckStats& stats) {
  const double abs_err = std::abs(analytic - reference);
  const double denom = std::max({1.0, std::abs(analytic), std::abs(reference)});
  const double rel_err = abs_err / denom;
  stats.max_abs = std::max(stats.max_abs, abs_err);
  stats.max_rel = std::max(stats.max_rel, rel_err);
  if (abs_err > tol && rel_err > tol) ++stats.failures;
}

static void compare_sparse_matrices(const Eigen::SparseMatrix<double>& analytic,
                                    const Eigen::SparseMatrix<double>& reference,
                                    const std::vector<engine::VariableMeta>& vars,
                                    double tol,
                                    DerivativeCheckStats& stats) {
  for (int col = 0; col < analytic.outerSize(); ++col) {
    if (derivative_check_skip_col(vars, col)) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(analytic, col); it; ++it) {
      accumulate_derivative_error(it.value(),
                                  sparse_coeff(reference, it.row(), it.col()),
                                  tol,
                                  stats);
    }
  }
  for (int col = 0; col < reference.outerSize(); ++col) {
    if (derivative_check_skip_col(vars, col)) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(reference, col); it; ++it) {
      if (std::abs(sparse_coeff(analytic, it.row(), it.col())) <= 1.0e-14) {
        accumulate_derivative_error(0.0, it.value(), tol, stats);
      }
    }
  }
}

static DerivativeCheckStats verify_sparse_derivatives(
    const FJModel& m,
    const engine::NLPModel& nlp,
    const Eigen::VectorXd& x,
    double fd_step,
    double tol) {
  DerivativeCheckStats stats;

  Eigen::VectorXd grad_an;
  full_objective_gradient(m, x, grad_an);
  const Eigen::VectorXd grad_fd =
      finite_difference_gradient(nlp.f, nlp.vars, x, fd_step);
  const int gn = std::min<int>(grad_an.size(), grad_fd.size());
  for (int j = 0; j < gn; ++j) {
    if (derivative_check_skip_col(nlp.vars, j)) continue;
    accumulate_derivative_error(grad_an[j], grad_fd[j], tol, stats);
  }
  stats.failures += std::abs(grad_an.size() - grad_fd.size());

  Eigen::SparseMatrix<double> jg_an;
  Eigen::SparseMatrix<double> jg_fd;
  full_equality_jacobian(m, x, jg_an);
  finite_difference_jacobian(nlp.g, nlp.vars, x, fd_step, jg_fd);
  compare_sparse_matrices(jg_an, jg_fd, nlp.vars, tol, stats);

  Eigen::SparseMatrix<double> jh_an;
  Eigen::SparseMatrix<double> jh_fd;
  full_inequality_jacobian(m, x, jh_an);
  finite_difference_jacobian(nlp.h, nlp.vars, x, fd_step, jh_fd);
  compare_sparse_matrices(jh_an, jh_fd, nlp.vars, tol, stats);

  stats.passed = stats.failures == 0;
  return stats;
}

static EVPowerTrafficOptions to_ev_opts(const JointOptimizerOptions& jo) {
  EVPowerTrafficOptions ev;
  ev.num_steps = jo.num_steps;
  ev.time_step_hr = jo.time_step_hr;
  ev.value_of_time_per_hr = jo.value_of_time_per_hr;
  ev.station_energy_cost_weight = jo.station_energy_cost_weight;
  ev.default_charging_efficiency = jo.default_charging_efficiency;
  ev.default_route_stop_power_kw_per_vehicle =
      jo.default_route_stop_power_kw_per_vehicle;
  ev.default_station_power_kw = jo.default_station_power_kw;
  ev.default_station_price_per_kwh = jo.default_station_price_per_kwh;
  ev.enforce_road_capacity = jo.enforce_road_capacity;
  ev.enforce_generation_capacity = jo.enforce_generation_capacity;
  ev.allow_v2g = jo.allow_v2g;
  return ev;
}

static void add_warning(JointOptimizerResult& result,
                        const std::string& warning,
                        bool verbose) {
  result.warnings.push_back(warning);
  if (verbose) {
    std::fprintf(stderr, "[FullJointNLP] WARNING: %s\n", warning.c_str());
  }
}

static bool build_full_model(const EVPowerTrafficProblem& problem,
                             const JointOptimizerOptions& opts,
                             FJModel& m,
                             engine::NLPModel& nlp,
                             JointOptimizerResult& result) {
  m.problem = &problem;
  m.opts = opts;
  m.ev_opts = to_ev_opts(opts);
  m.T = std::max(1, opts.num_steps);
  m.dt = std::max(1.0e-9, opts.time_step_hr);
  nlp.sense = engine::Sense::Minimize;

  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i) {
    m.link_pos[problem.traffic.links[i].index] = i;
  }
  for (int bi = 0; bi < static_cast<int>(problem.system.ac.buses.size()); ++bi) {
    const auto& bus = problem.system.ac.buses[static_cast<std::size_t>(bi)];
    m.bus_pos[bus.index] = bi;
    if (bus.bus_type == BusType::SLACK) m.slack_bus_pos = bi;
  }
  for (const auto& cs : problem.system.ac.charging_stations) {
    m.station_bus[cs.index] = cs.bus;
    m.station_cap_kw[cs.index] = station_effective_cap_kw(problem.system, opts, cs.index);
  }

  for (int di = 0; di < static_cast<int>(problem.demands.size()); ++di) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    if (demand.vehicles <= kTol) continue;
    m.demand_to_active[di] = static_cast<int>(m.active_demands.size());
    m.active_demands.push_back(di);
  }
  if (m.active_demands.empty()) {
    result.feasible = true;
    result.proven_optimal = true;
    result.local_optimum_certificate = true;
    result.solver_backend = "FullJointNLP-empty";
    result.solver_status = "empty";
    return false;
  }

  m.unserved_col_by_active.assign(m.active_demands.size(), -1);
  m.mu_col_by_active.assign(m.active_demands.size(), -1);

  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int di = m.active_demands[static_cast<std::size_t>(ai)];
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];

    FJColumn mc;
    mc.kind = FJColumn::Kind::DemandMinCost;
    mc.demand_pos = di;
    m.mu_col_by_active[static_cast<std::size_t>(ai)] =
        add_col(m, nlp, mc, -1.0e8, 1.0e8,
                "mu_d" + std::to_string(demand.index));

    FJColumn uc;
    uc.kind = FJColumn::Kind::UnservedDemand;
    uc.demand_pos = di;
    m.unserved_col_by_active[static_cast<std::size_t>(ai)] =
        add_col(m, nlp, uc, 0.0, demand.vehicles,
                "u_d" + std::to_string(demand.index));

    const auto routes = candidate_routes(problem, demand);
    for (int dep : departure_steps_for(demand)) {
      if (dep < 0 || dep >= m.T) continue;
      for (const auto* route : routes) {
        if (route == nullptr) continue;
        if (!route_can_deliver_requested_energy(*route, m.ev_opts, m.dt)) continue;
        if (!route_soc_feasible(*route, demand, problem, m.link_pos)) continue;
        const int arr = route_arrival_step(*route, m.link_pos, problem, dep, m.dt);
        bool fits = true;
        for (const auto& stop : route->charging_stops) {
          const int dwell = std::max(1, stop.dwell_steps);
          if (arr < 0 || arr >= m.T || arr + dwell > m.T) {
            fits = false;
            break;
          }
        }
        if (!fits) continue;

        FJChoice choice;
        choice.demand_pos = di;
        choice.route = route;
        choice.departure_step = dep;
        choice.arrival_step = arr;
        choice.vehicles_ub = demand.vehicles;
        const int ci = static_cast<int>(m.choices.size());

        FJColumn hc;
        hc.kind = FJColumn::Kind::ChoiceFlow;
        hc.demand_pos = di;
        hc.choice_pos = ci;
        hc.step = dep;
        choice.h_col = add_col(m, nlp, hc, 0.0, demand.vehicles,
                               "h_d" + std::to_string(demand.index) +
                               "_k" + std::to_string(dep) +
                               "_r" + std::to_string(route->index));
        m.choices.push_back(choice);
      }
    }
  }

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const auto& demand = problem.demands[static_cast<std::size_t>(choice.demand_pos)];
    const double drive_kwh =
        route_drive_energy_kwh_per_veh(*choice.route, problem, m.link_pos);
    double e_arrival = 0.0;
    double e_min = 0.0;
    double e_max = 0.0;
    bool has_e_max = false;
    if (demand.initial_energy_kwh >= 0.0) {
      e_arrival = std::max(0.0, demand.initial_energy_kwh - drive_kwh);
      e_min = std::max(0.0, demand.energy_min_kwh);
      if (demand.energy_max_kwh >= 0.0) {
        e_max = std::max(e_min, demand.energy_max_kwh);
        has_e_max = true;
        e_arrival = std::min(e_arrival, e_max);
      }
    }

    for (const auto& stop : choice.route->charging_stops) {
      FJChargeGroup group;
      group.choice_pos = ci;
      group.station_id = stop.station_id;
      group.arrival_step = choice.arrival_step;
      group.window = std::max(1, stop.dwell_steps);
      group.departure_step = std::min(m.T, group.arrival_step + group.window);
      group.window = std::max(1, group.departure_step - group.arrival_step);
      group.requested_kwh_per_veh =
          std::max(0.0, stop.requested_energy_kwh_per_vehicle);
      group.p_ch_max_per_veh = full_charge_power_kw_per_veh(stop, opts, m.dt);
      group.p_dis_max_per_veh =
          (opts.allow_v2g && stop.v2g_capable)
              ? std::max(0.0, stop.max_discharge_kw_per_vehicle)
              : 0.0;
      group.eta_ch = std::max(opts.default_charging_efficiency, kTol);
      group.eta_dis = std::max(opts.default_charging_efficiency, kTol);
      group.e_arrival_per_veh = e_arrival;
      group.e_min_per_veh = e_min;
      group.e_target_per_veh = group.requested_kwh_per_veh > kTol
          ? e_arrival + group.requested_kwh_per_veh
          : group.e_min_per_veh;
      group.e_max_per_veh = has_e_max
          ? e_max
          : std::max(group.e_target_per_veh +
                         group.p_ch_max_per_veh * group.window * m.dt,
                     group.e_arrival_per_veh + group.requested_kwh_per_veh);
      group.e_max_per_veh = std::max(group.e_max_per_veh, group.e_min_per_veh);
      group.v2g = group.p_dis_max_per_veh > kTol;

      const int gi = static_cast<int>(m.charge_groups.size());
      for (int off = 0; off <= group.window; ++off) {
        FJColumn ec;
        ec.kind = FJColumn::Kind::BatteryEnergy;
        ec.choice_pos = ci;
        ec.group_pos = gi;
        ec.step = group.arrival_step + off;
        ec.offset = off;
        const double ub = std::max(group.e_max_per_veh * choice.vehicles_ub, 1.0e-6);
        group.energy_cols.push_back(add_col(
            m, nlp, ec, 0.0, ub,
            "e_c" + std::to_string(ci) + "_o" + std::to_string(off)));
      }
      for (int off = 0; off < group.window; ++off) {
        const int k = group.arrival_step + off;
        FJColumn pc;
        pc.kind = FJColumn::Kind::ChargePower;
        pc.choice_pos = ci;
        pc.group_pos = gi;
        pc.step = k;
        pc.offset = off;
        group.pch_cols.push_back(add_col(
            m, nlp, pc, 0.0, group.p_ch_max_per_veh * choice.vehicles_ub,
            "pch_c" + std::to_string(ci) + "_k" + std::to_string(k)));

        FJColumn pd;
        pd.kind = FJColumn::Kind::DischargePower;
        pd.choice_pos = ci;
        pd.group_pos = gi;
        pd.step = k;
        pd.offset = off;
        group.pdis_cols.push_back(add_col(
            m, nlp, pd, 0.0, group.p_dis_max_per_veh * choice.vehicles_ub,
            "pdis_c" + std::to_string(ci) + "_k" + std::to_string(k)));
      }
      m.charge_groups.push_back(std::move(group));
    }
  }

  if (opts.include_dcopf) {
    const auto& sys = problem.system;
    const int n_gens = static_cast<int>(sys.ac.generators.size());
    const int n_buses = static_cast<int>(sys.ac.buses.size());
    const int n_branches = static_cast<int>(sys.ac.branches.size());
    m.gen_base.assign(n_gens, -1);
    m.bus_angle_base.assign(n_buses, -1);
    m.branch_flow_base.assign(n_branches, -1);
    m.power_slack_base.assign(n_buses, -1);
    m.branch_b.assign(n_branches, 0.0);

    for (int gi = 0; gi < n_gens; ++gi) {
      const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
      if (!gen.in_service) continue;
      m.gen_base[static_cast<std::size_t>(gi)] = static_cast<int>(m.cols.size());
      for (int k = 0; k < m.T; ++k) {
        FJColumn c;
        c.kind = FJColumn::Kind::GeneratorPower;
        c.gen_pos = gi;
        c.step = k;
        add_col(m, nlp, c, std::max(0.0, gen.pmin_mw), gen.pmax_mw,
                "Pg_g" + std::to_string(gen.index) + "_k" + std::to_string(k));
      }
    }
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      m.bus_angle_base[static_cast<std::size_t>(bi)] = static_cast<int>(m.cols.size());
      for (int k = 0; k < m.T; ++k) {
        FJColumn c;
        c.kind = FJColumn::Kind::BusAngle;
        c.bus_pos = bi;
        c.step = k;
        const double lb = (bi == m.slack_bus_pos) ? 0.0 : -1.0e6;
        const double ub = (bi == m.slack_bus_pos) ? 0.0 : 1.0e6;
        add_col(m, nlp, c, lb, ub,
                "theta_b" + std::to_string(bus.index) + "_k" + std::to_string(k));
      }
    }
    for (int li = 0; li < n_branches; ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      m.branch_b[static_cast<std::size_t>(li)] =
          std::abs(br.x_pu) > 1.0e-12 ? 1.0 / br.x_pu : 1.0e6;
      if (!br.in_service) continue;
      m.branch_flow_base[static_cast<std::size_t>(li)] = static_cast<int>(m.cols.size());
      const double fmax = br.rate_a_mva > kTol ? br.rate_a_mva : 1.0e6;
      for (int k = 0; k < m.T; ++k) {
        FJColumn c;
        c.kind = FJColumn::Kind::BranchFlow;
        c.branch_pos = li;
        c.step = k;
        add_col(m, nlp, c, -fmax, fmax,
                "f_l" + std::to_string(br.index) + "_k" + std::to_string(k));
      }
    }
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      m.power_slack_base[static_cast<std::size_t>(bi)] = static_cast<int>(m.cols.size());
      for (int k = 0; k < m.T; ++k) {
        FJColumn c;
        c.kind = FJColumn::Kind::PowerSlack;
        c.bus_pos = bi;
        c.step = k;
        add_col(m, nlp, c, 0.0, 1.0e6,
                "lp_b" + std::to_string(bus.index) + "_k" + std::to_string(k));
      }
    }
  }

  std::map<std::pair<int, int>, bool> road_seen;
  if (opts.enforce_road_capacity) {
    for (const auto& choice : m.choices) {
      for (int link_index : choice.route->link_indices) {
        const auto key = std::make_pair(choice.departure_step, link_index);
        if (road_seen.emplace(key, true).second) m.road_rows.push_back(key);
      }
    }
  }
  std::map<std::pair<int, int>, bool> st_seen, sd_seen;
  for (const auto& group : m.charge_groups) {
    for (int off = 0; off < group.window; ++off) {
      const auto key = std::make_pair(group.arrival_step + off, group.station_id);
      if (st_seen.emplace(key, true).second) m.station_rows.push_back(key);
      if (sd_seen.emplace(key, true).second) m.station_dis_rows.push_back(key);
    }
  }

  nlp.x0 = Eigen::VectorXd::Zero(static_cast<int>(nlp.vars.size()));
  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    const auto& demand = problem.demands[static_cast<std::size_t>(dpos)];
    std::vector<int> cands;
    double cap_sum = 0.0;
    for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
      if (m.choices[static_cast<std::size_t>(ci)].demand_pos != dpos) continue;
      const double cap = choice_capacity_ub(m, m.choices[static_cast<std::size_t>(ci)]);
      if (cap > kTol) {
        cands.push_back(ci);
        cap_sum += cap;
      }
    }
    double assigned = 0.0;
    for (int ci : cands) {
      const double share = demand.vehicles *
                           choice_capacity_ub(m, m.choices[static_cast<std::size_t>(ci)]) /
                           std::max(cap_sum, kTol);
      const double h0 = std::min(share, choice_capacity_ub(m, m.choices[static_cast<std::size_t>(ci)]));
      nlp.x0[m.choices[static_cast<std::size_t>(ci)].h_col] = h0;
      assigned += h0;
    }
    nlp.x0[m.unserved_col_by_active[static_cast<std::size_t>(ai)]] =
        std::max(0.0, demand.vehicles - assigned);
  }

  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    double min_cost = 1.0e6;
    for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
      if (m.choices[static_cast<std::size_t>(ci)].demand_pos != dpos) continue;
      min_cost = std::min(min_cost, choice_generalized_cost(m, nlp.x0, ci));
    }
    if (!std::isfinite(min_cost)) min_cost = 0.0;
    nlp.x0[m.mu_col_by_active[static_cast<std::size_t>(ai)]] = min_cost;
  }

  for (const auto& group : m.charge_groups) {
    const double flow = h_value(m, nlp.x0, group.choice_pos);
    double e = group.e_arrival_per_veh * flow;
    nlp.x0[group.energy_cols.front()] = e;
    const double target = group.e_target_per_veh * flow;
    const double need = std::max(0.0, target - e);
    const double p_each =
        group.window > 0 ? need / (group.eta_ch * m.dt * group.window) : 0.0;
    for (int off = 0; off < group.window; ++off) {
      const double pch = std::min(group.p_ch_max_per_veh * flow, p_each);
      nlp.x0[group.pch_cols[static_cast<std::size_t>(off)]] = pch;
      nlp.x0[group.pdis_cols[static_cast<std::size_t>(off)]] = 0.0;
      e += group.eta_ch * pch * m.dt;
      nlp.x0[group.energy_cols[static_cast<std::size_t>(off + 1)]] = e;
    }
  }

  if (opts.include_dcopf) {
    const auto& sys = problem.system;
    for (int k = 0; k < m.T; ++k) {
      double total_ev_mw = 0.0;
      for (const auto& group : m.charge_groups) {
        for (int off = 0; off < group.window; ++off) {
          if (group.arrival_step + off == k) {
            total_ev_mw += nlp.x0[group.pch_cols[static_cast<std::size_t>(off)]] / 1000.0;
            total_ev_mw -= nlp.x0[group.pdis_cols[static_cast<std::size_t>(off)]] / 1000.0;
          }
        }
      }
      double load = total_ev_mw;
      for (const auto& bus : sys.ac.buses) load += std::max(0.0, bus.pd_mw);
      double remaining = load;
      for (int gi = 0; gi < static_cast<int>(sys.ac.generators.size()); ++gi) {
        const int base = m.gen_base[static_cast<std::size_t>(gi)];
        if (base < 0) continue;
        const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
        const double pg = std::min(std::max(0.0, gen.pmax_mw), remaining);
        nlp.x0[base + k] = std::max(gen.pmin_mw, pg);
        remaining -= pg;
      }
      for (int bi = 0; bi < static_cast<int>(sys.ac.buses.size()); ++bi) {
        const int base = m.power_slack_base[static_cast<std::size_t>(bi)];
        if (base >= 0 && remaining > 0.0 && bi == m.slack_bus_pos) {
          nlp.x0[base + k] = remaining;
        }
      }
    }
  }

  auto data = std::make_shared<FJModel>(std::move(m));
  const double fd = std::max(1.0e-9, opts.full_joint_fd_step);
  nlp.f = [data](const Eigen::VectorXd& x) { return full_objective(*data, x); };
  nlp.grad = [data](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    full_objective_gradient(*data, x, grad);
  };
  nlp.g = [data](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    full_equalities(*data, x, g);
  };
  nlp.jac_g = [data](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& jac) {
    full_equality_jacobian(*data, x, jac);
  };
  nlp.h = [data](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    full_inequalities(*data, x, h);
  };
  nlp.jac_h = [data](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& jac) {
    full_inequality_jacobian(*data, x, jac);
  };

  m = *data;
  result.sparse_derivatives_enabled = true;
  result.sparse_derivatives_verified = false;
  Eigen::SparseMatrix<double> jg0;
  Eigen::SparseMatrix<double> jh0;
  nlp.jac_g(nlp.x0, jg0);
  nlp.jac_h(nlp.x0, jh0);
  result.sparse_jacobian_nnz =
      static_cast<int>(jg0.nonZeros() + jh0.nonZeros());
  if (opts.full_joint_verify_sparse_derivatives) {
    const auto stats = verify_sparse_derivatives(
        m, nlp, nlp.x0, fd,
        std::max(1.0e-12, opts.full_joint_derivative_check_tol));
    result.sparse_derivatives_verified = stats.passed;
    result.derivative_check_max_abs_error = stats.max_abs;
    result.derivative_check_max_rel_error = stats.max_rel;
    result.derivative_check_failures = stats.failures;
    if (!stats.passed) {
      add_warning(result,
                  "Sparse derivative check failed against finite differences: "
                  "max_abs=" + std::to_string(stats.max_abs) +
                  ", max_rel=" + std::to_string(stats.max_rel) +
                  ", failures=" + std::to_string(stats.failures) + ".",
                  opts.verbose);
    }
  }
  return true;
}

static double max_bound_violation(const engine::NLPModel& nlp,
                                  const Eigen::VectorXd& x) {
  double v = 0.0;
  for (int j = 0; j < x.size() && j < static_cast<int>(nlp.vars.size()); ++j) {
    const auto& var = nlp.vars[static_cast<std::size_t>(j)];
    v = std::max(v, finite_bound_violation(x[j], var.lb, var.ub));
  }
  return v;
}

static void decode_solution(const FJModel& m,
                            const engine::NLPModel& nlp,
                            const Eigen::VectorXd& x,
                            JointOptimizerResult& result) {
  const int T = m.T;
  const double dt = m.dt;

  for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
    const auto& choice = m.choices[static_cast<std::size_t>(ci)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
    const double flow = h_value(m, x, ci);
    if (flow <= kTol) continue;
    const double tt = route_travel_time_endogenous(m, x, choice);
    result.route_flow[demand.index][choice.route->index] += flow;
    result.assignments.push_back({demand.index,
                                  choice.route->index,
                                  choice.departure_step,
                                  flow,
                                  choice_generalized_cost(m, x, ci),
                                  finite_or(tt, 0.0),
                                  true,
                                  "full-joint-nlp"});
    result.ev_benefit += demand.willingness_to_pay_per_vehicle * flow;
    result.total_served_vehicles += flow;
    result.total_travel_time_hr += flow * finite_or(tt, 0.0);
    const double vot = demand.value_of_time_per_hr > kTol
        ? demand.value_of_time_per_hr
        : m.opts.value_of_time_per_hr;
    result.traffic_delay_cost += vot * flow * finite_or(tt, 0.0);
  }

  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(dpos)];
    const double u = std::max(0.0, finite_or(x[m.unserved_col_by_active[static_cast<std::size_t>(ai)]], 0.0));
    if (u > kTol) {
      result.total_unserved_vehicles += u;
      result.assignments.push_back({demand.index, 0, demand.departure_step,
                                    u, 0.0, 0.0, false,
                                    "full-joint unmet demand"});
    }
  }

  int session_id = 0;
  for (const auto& group : m.charge_groups) {
    const auto& choice = m.choices[static_cast<std::size_t>(group.choice_pos)];
    const auto& demand = m.problem->demands[static_cast<std::size_t>(choice.demand_pos)];
    const double flow = h_value(m, x, group.choice_pos);
    if (flow <= kTol) continue;

    EVChargingSession sess;
    sess.index = session_id++;
    sess.station_id = group.station_id;
    sess.arrival_step = group.arrival_step;
    sess.departure_step = group.departure_step;
    sess.vehicle_count = flow;
    sess.energy_initial_kwh = group.e_arrival_per_veh * flow;
    sess.energy_target_kwh = group.e_target_per_veh * flow;
    sess.energy_min_kwh = group.e_min_per_veh * flow;
    sess.energy_max_kwh = group.e_max_per_veh * flow;
    sess.max_charge_kw = group.p_ch_max_per_veh * flow;
    sess.max_discharge_kw = group.p_dis_max_per_veh * flow;
    sess.eta_charge = group.eta_ch;
    sess.eta_discharge = group.eta_dis;
    sess.v2g_capable = group.v2g;
    sess.source_demand_index = demand.index;
    sess.source_route_index = choice.route->index;

    ChargingSessionResult sr;
    sr.session_index = sess.index;
    sr.station_id = group.station_id;
    sr.p_charge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.p_discharge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.energy_kwh.assign(static_cast<std::size_t>(T + 1), 0.0);
    for (int off = 0; off < group.window; ++off) {
      const int k = group.arrival_step + off;
      if (k < 0 || k >= T) continue;
      const double pch = std::max(0.0, finite_or(x[group.pch_cols[static_cast<std::size_t>(off)]], 0.0));
      const double pdis = std::max(0.0, finite_or(x[group.pdis_cols[static_cast<std::size_t>(off)]], 0.0));
      sr.p_charge_kw[static_cast<std::size_t>(k)] = pch;
      sr.p_discharge_kw[static_cast<std::size_t>(k)] = pdis;
      result.total_delivered_energy_kwh += group.eta_ch * pch * dt;
      result.total_v2g_energy_kwh += pdis * dt / std::max(group.eta_dis, kTol);
    }
    for (int off = 0; off <= group.window; ++off) {
      const int k = group.arrival_step + off;
      if (k >= 0 && k <= T) {
        sr.energy_kwh[static_cast<std::size_t>(k)] =
            std::max(0.0, finite_or(x[group.energy_cols[static_cast<std::size_t>(off)]], 0.0));
      }
    }
    for (int k = 1; k <= T; ++k) {
      if (sr.energy_kwh[static_cast<std::size_t>(k)] <= kTol) {
        sr.energy_kwh[static_cast<std::size_t>(k)] =
            sr.energy_kwh[static_cast<std::size_t>(k - 1)];
      }
    }
    result.total_requested_energy_kwh += group.requested_kwh_per_veh * flow;
    sr.unserved_energy_kwh = std::max(
        0.0,
        sess.energy_target_kwh -
            sr.energy_kwh[static_cast<std::size_t>(std::clamp(sess.departure_step, 0, T))]);
    result.total_unserved_energy_kwh += sr.unserved_energy_kwh;
    result.sessions.push_back(sess);
    result.session_results.push_back(std::move(sr));
  }

  if (m.opts.include_dcopf) {
    const auto& sys = m.problem->system;
    result.gen_dispatch_mw.assign(static_cast<std::size_t>(T), {});
    result.lmp_available = false;
    result.lmp_unavailability_reason =
        "The full-joint NLP path does not expose power-balance equality multipliers as LMPs.";
    result.warnings.push_back(result.lmp_unavailability_reason);
    for (int gi = 0; gi < static_cast<int>(sys.ac.generators.size()); ++gi) {
      const int base = m.gen_base[static_cast<std::size_t>(gi)];
      if (base < 0) continue;
      const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
      for (int k = 0; k < T; ++k) {
        const double pg = finite_or(x[base + k], 0.0);
        result.gen_dispatch_mw[static_cast<std::size_t>(k)][gen.index] = pg;
        result.gen_cost += gen.cost_c1 * pg * dt;
      }
    }
  }

  result.social_welfare =
      result.ev_benefit - result.traffic_delay_cost - result.gen_cost;
  result.total_assignment_cost = result.traffic_delay_cost;

  Eigen::VectorXd geq, hineq;
  full_equalities(m, x, geq);
  full_inequalities(m, x, hineq);
  for (int i = 0; i < geq.size(); ++i) {
    result.nonlinear_constraint_violation =
        std::max(result.nonlinear_constraint_violation, std::abs(geq[i]));
  }
  for (int i = 0; i < hineq.size(); ++i) {
    result.nonlinear_constraint_violation =
        std::max(result.nonlinear_constraint_violation, std::max(0.0, hineq[i]));
  }
  result.nonlinear_constraint_violation =
      std::max(result.nonlinear_constraint_violation, max_bound_violation(nlp, x));
  result.primal_max_violation = result.nonlinear_constraint_violation;

  for (const auto& [step, link_index] : m.road_rows) {
    const auto it = m.link_pos.find(link_index);
    const double cap = (it == m.link_pos.end())
        ? 0.0
        : link_capacity_vehicles(m.problem->traffic.links[it->second], step, dt);
    result.road_capacity_max_violation =
        std::max(result.road_capacity_max_violation,
                 std::max(0.0, link_flow(m, x, step, link_index) - cap));
  }

  double weighted_gap = 0.0;
  double weighted_cost = 0.0;
  for (int ai = 0; ai < static_cast<int>(m.active_demands.size()); ++ai) {
    const int dpos = m.active_demands[static_cast<std::size_t>(ai)];
    double min_cost = kInf;
    for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
      if (m.choices[static_cast<std::size_t>(ci)].demand_pos != dpos) continue;
      min_cost = std::min(min_cost, choice_generalized_cost(m, x, ci));
    }
    if (!std::isfinite(min_cost)) continue;
    for (int ci = 0; ci < static_cast<int>(m.choices.size()); ++ci) {
      const auto& choice = m.choices[static_cast<std::size_t>(ci)];
      if (choice.demand_pos != dpos) continue;
      const double flow = h_value(m, x, ci);
      const double cost = choice_generalized_cost(m, x, ci);
      const double gap = cost - min_cost;
      if (flow > kTol) {
        result.wardrop_gap = std::max(result.wardrop_gap, std::max(0.0, gap));
        weighted_gap += flow * std::max(0.0, gap);
        weighted_cost += flow * std::max(0.0, cost);
      }
      const double mu_gap = cost - demand_mu(m, x, choice.demand_pos);
      result.wardrop_complementarity_residual = std::max(
          result.wardrop_complementarity_residual,
          std::max(std::abs(flow * mu_gap), std::max(0.0, -mu_gap)));
    }
  }
  if (weighted_cost > kTol) {
    result.wardrop_gap = std::max(result.wardrop_gap, weighted_gap / weighted_cost);
  }

  if (m.opts.include_dcopf) {
    Eigen::VectorXd g;
    full_equalities(m, x, g);
    const int n_dem = static_cast<int>(m.active_demands.size());
    int n_energy_eq = 0;
    for (const auto& group : m.charge_groups) n_energy_eq += 1 + group.window;
    const int pbal_start = n_dem + n_energy_eq +
        static_cast<int>(m.problem->system.ac.branches.size()) * T;
    const int pbal_rows = static_cast<int>(m.problem->system.ac.buses.size()) * T;
    for (int i = 0; i < pbal_rows && pbal_start + i < g.size(); ++i) {
      result.power_balance_max_violation =
          std::max(result.power_balance_max_violation, std::abs(g[pbal_start + i]));
    }
  }
}

}  // namespace

JointOptimizerResult solve_joint_optimizer_full_nlp(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& opts) {
  using namespace std::chrono;
  const auto t_start = steady_clock::now();

  JointOptimizerResult result;
  result.monolithic_full_joint_model = true;
  result.nonlinear_model = true;
  result.local_optimum_certificate = false;
  result.global_optimum_certificate = false;
  result.endogenous_congestion_enforced = true;
  result.travel_times_are_endogenous = true;
  result.wardrop_complementarity_enforced = true;
  result.dcopf_coupling_enforced = opts.include_dcopf;
  result.charging_v2g_enforced = true;
  result.best_bound = std::numeric_limits<double>::quiet_NaN();
  result.mip_gap = std::numeric_limits<double>::quiet_NaN();

  if (opts.require_exact_mathematical_model) {
    result.solver_backend = "FullJointNLP";
    result.solver_status =
        "unsupported: exact full mathematical-model verification requires a "
        "global nonlinear MPEC/MINLP certificate that is not available";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: FullJointNLP is a reduced finite-route BPR NLP/MPEC with "
        "local stationarity only, not a global CTM/LTM joint-model proof";
    add_warning(result, result.mathematical_model_verification_status,
                opts.verbose);
    return result;
  }

  if (opts.require_global_nonlinear_certificate) {
    result.solver_backend = "FullJointNLP";
    result.solver_status =
        "unsupported: no in-tree global nonlinear MPEC/MINLP certificate";
    add_warning(result,
                "Requested a global exact nonlinear MPEC certificate, but the "
                "available sparse NativeNLP path certifies only local NLP "
                "feasibility/stationarity for this reduced model.",
                opts.verbose);
    return result;
  }

  if (!opts.include_dcopf) {
    add_warning(result,
                "FullJoint* NLP was run without DC-OPF rows because "
                "include_dcopf=false.",
                opts.verbose);
  }
  add_warning(result,
              "FullJoint* NLP is monolithic for the reduced finite-route "
              "BPR congestion + charging/V2G + DC-OPF + Wardrop-MPEC model; "
              "the solver certificate is local, not a global nonlinear MIP gap.",
              opts.verbose);

  FJModel model;
  engine::NLPModel nlp;
  if (!build_full_model(problem, opts, model, nlp, result)) {
    return result;
  }

  result.n_variables = static_cast<int>(nlp.vars.size());
  Eigen::VectorXd g0, h0;
  nlp.g(nlp.x0, g0);
  nlp.h(nlp.x0, h0);
  result.n_constraints = static_cast<int>(g0.size() + h0.size());

  engine::SolveResult solve;
  if (opts.full_joint_prefer_ipopt) {
    add_warning(result,
        "The embedded Ipopt/MUMPS path is not used for FullJoint* NLP "
                "because this path currently uses the in-process sparse "
                "NativeNLP local penalty solve for deterministic verification.",
                opts.verbose);
  }

  engine::NativeNLPOptions nlp_opts;
  nlp_opts.max_iter = 300;
  nlp_opts.tol_grad = 1.0e-5;
  nlp_opts.tol_step = 1.0e-8;
  nlp_opts.penalty_rho = 1.0e4;
  nlp_opts.regularization0 = 1.0e-8;
  nlp_opts.step_backoff = 0.5;
  nlp_opts.max_line_search_steps = 30;
  engine::NativeNLPAdapter native_nlp(nlp_opts);
  solve = native_nlp.solve_nlp(nlp);
  result.solver_backend = "NativeNLP";
  result.solver_status = solve.stats.status;

  result.objective = solve.stats.objective;
  result.nonlinear_stationarity_residual =
      std::max({std::abs(solve.stats.dual_feas),
                std::abs(solve.stats.residual_inf),
                std::abs(solve.stats.complementarity)});

  const auto t_end = steady_clock::now();
  result.solve_time_sec =
      duration_cast<microseconds>(t_end - t_start).count() * 1e-6;

  if (!solve.stats.success || solve.x.size() != static_cast<int>(nlp.vars.size())) {
    result.solver_status = result.solver_status.empty()
        ? solve.stats.status
        : result.solver_status;
    add_warning(result, "Full-joint NLP solve failed: " + result.solver_status,
                opts.verbose);
    return result;
  }

  decode_solution(model, nlp, solve.x, result);
  result.feasible = result.nonlinear_constraint_violation <= 1.0e-4;
  result.local_optimum_certificate =
      solve.stats.success && result.feasible &&
      result.nonlinear_stationarity_residual <= 1.0e-3;
  result.proven_optimal = false;
  result.global_optimum_certificate = false;
  // FullJointNLP enforces congestion endogenously through BPR link-cost rows.
  result.travel_times_are_endogenous = true;
  return result;
}

}  // namespace hacdcpf::evpt
