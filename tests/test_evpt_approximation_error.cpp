#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Highs.h"

#include "hacdcpf/engine/engine.hpp"

using Approx = Catch::Approx;

namespace {

constexpr double kTol = 1e-9;

struct Corridor {
  int id{0};
  int free_flow_steps{1};
  double capacity_veh_per_step{1.0};
  double jam_veh_per_cell{4.0};
  double background_veh_per_step{0.0};
};

struct Fleet {
  int id{0};
  double vehicles{1.0};
  double energy_mwh{0.05};
  double charging_power_mw{0.20};
  int charging_deadline_step{6};
};

struct Plan {
  int id{0};
  int corridor_id{0};
  int departure_step{0};
  int station_id{0};
};

struct Token {
  int fleet_pos{0};
  double vehicles{0.0};
  double entry_time_hr{0.0};
};

struct PendingToken {
  int fleet_pos{0};
  double vehicles{0.0};
};

struct FleetTravelResult {
  double entered_vehicles{0.0};
  double exited_vehicles{0.0};
  double mean_trip_time_hr{0.0};
  double last_arrival_time_hr{0.0};
};

struct CorridorCountResult {
  std::vector<double> cumulative_in;
  std::vector<double> cumulative_out;
};

struct CTMReplayResult {
  std::vector<FleetTravelResult> fleet;
  std::unordered_map<int, CorridorCountResult> corridor_counts;
};

struct StudyCase {
  double dt_hr{0.25};
  int traffic_steps{16};
  int power_steps{8};
  double value_of_time_per_vehicle_hr{20.0};
  double shortfall_penalty_per_mwh{500.0};
  double line_limit_mw{0.15};
  double cheap_generator_capacity_mw{1.0};
  std::vector<double> cheap_generation_cost;
  std::vector<double> local_generation_cost;
  std::vector<Corridor> corridors;
  std::vector<Fleet> fleets;
  std::vector<std::vector<Plan>> plans;
  std::unordered_map<int, double> station_capacity_mw;
  std::unordered_map<int, int> station_bus;
};

double token_mass(const std::deque<Token>& queue) {
  double total = 0.0;
  for (const auto& token : queue) total += token.vehicles;
  return total;
}

double pending_mass(const std::deque<PendingToken>& queue) {
  double total = 0.0;
  for (const auto& token : queue) total += token.vehicles;
  return total;
}

template <class OnMove>
void move_tokens(std::deque<Token>& from,
                 std::deque<Token>* to,
                 double amount,
                 OnMove&& on_move) {
  double remaining = amount;
  while (remaining > kTol && !from.empty()) {
    Token token = from.front();
    const double moved = std::min(remaining, token.vehicles);
    Token part = token;
    part.vehicles = moved;
    on_move(part);
    if (to != nullptr) to->push_back(part);
    from.front().vehicles -= moved;
    if (from.front().vehicles <= kTol) from.pop_front();
    remaining -= moved;
  }
}

void enter_pending(std::deque<PendingToken>& origin,
                   std::deque<Token>& first_cell,
                   double amount,
                   double entry_time_hr,
                   std::vector<FleetTravelResult>& fleet_results) {
  double remaining = amount;
  while (remaining > kTol && !origin.empty()) {
    PendingToken pending = origin.front();
    const double moved = std::min(remaining, pending.vehicles);
    first_cell.push_back({pending.fleet_pos, moved, entry_time_hr});
    if (pending.fleet_pos >= 0) {
      fleet_results[static_cast<std::size_t>(pending.fleet_pos)]
          .entered_vehicles += moved;
    }
    origin.front().vehicles -= moved;
    if (origin.front().vehicles <= kTol) origin.pop_front();
    remaining -= moved;
  }
}

CTMReplayResult run_fifo_corridor_ctm(const StudyCase& data,
                                      const std::vector<int>& assignment,
                                      const std::vector<bool>& loaded) {
  struct CorridorState {
    Corridor parameters;
    std::deque<PendingToken> origin;
    std::vector<std::deque<Token>> cells;
    CorridorCountResult counts;
  };

  CTMReplayResult result;
  result.fleet.resize(data.fleets.size());
  std::unordered_map<int, CorridorState> states;
  for (const auto& corridor : data.corridors) {
    CorridorState state;
    state.parameters = corridor;
    state.cells.resize(static_cast<std::size_t>(corridor.free_flow_steps));
    state.counts.cumulative_in.assign(
        static_cast<std::size_t>(data.traffic_steps + 1), 0.0);
    state.counts.cumulative_out.assign(
        static_cast<std::size_t>(data.traffic_steps + 1), 0.0);
    states.emplace(corridor.id, std::move(state));
  }

  for (int step = 0; step < data.traffic_steps; ++step) {
    for (auto& [corridor_id, state] : states) {
      if (state.parameters.background_veh_per_step > kTol) {
        state.origin.push_back(
            {-1, state.parameters.background_veh_per_step});
      }
      (void)corridor_id;
    }
    for (std::size_t w = 0; w < data.fleets.size(); ++w) {
      if (!loaded[w]) continue;
      const Plan& plan = data.plans[w][static_cast<std::size_t>(assignment[w])];
      if (plan.departure_step == step) {
        states.at(plan.corridor_id).origin.push_back(
            {static_cast<int>(w), data.fleets[w].vehicles});
      }
    }

    for (auto& [corridor_id, state] : states) {
      const double capacity = state.parameters.capacity_veh_per_step;
      auto& first_cell = state.cells.front();
      const double receiving = std::min(
          capacity,
          std::max(0.0, state.parameters.jam_veh_per_cell - token_mass(first_cell)));
      const double entered = std::min(receiving, pending_mass(state.origin));
      enter_pending(state.origin, first_cell, entered,
                    static_cast<double>(step) * data.dt_hr, result.fleet);

      std::vector<double> occupancy(state.cells.size(), 0.0);
      for (std::size_t m = 0; m < state.cells.size(); ++m) {
        occupancy[m] = token_mass(state.cells[m]);
      }

      const double exited = std::min(capacity, occupancy.back());
      const double exit_time_hr = static_cast<double>(step + 1) * data.dt_hr;
      move_tokens(state.cells.back(), nullptr, exited, [&](const Token& token) {
        if (token.fleet_pos < 0) return;
        auto& fr = result.fleet[static_cast<std::size_t>(token.fleet_pos)];
        fr.exited_vehicles += token.vehicles;
        fr.mean_trip_time_hr += token.vehicles *
            (exit_time_hr -
             data.plans[static_cast<std::size_t>(token.fleet_pos)]
                       [static_cast<std::size_t>(assignment[static_cast<std::size_t>(token.fleet_pos)])]
                           .departure_step * data.dt_hr);
        fr.last_arrival_time_hr = exit_time_hr;
      });

      if (state.cells.size() > 1) {
        for (int m = static_cast<int>(state.cells.size()) - 2; m >= 0; --m) {
          const double sending = std::min(capacity, occupancy[static_cast<std::size_t>(m)]);
          const double receiving_down = std::min(
              capacity,
              std::max(0.0, state.parameters.jam_veh_per_cell -
                                occupancy[static_cast<std::size_t>(m + 1)]));
          const double flow = std::min(sending, receiving_down);
          move_tokens(state.cells[static_cast<std::size_t>(m)],
                      &state.cells[static_cast<std::size_t>(m + 1)],
                      flow, [](const Token&) {});
        }
      }

      state.counts.cumulative_in[static_cast<std::size_t>(step + 1)] =
          state.counts.cumulative_in[static_cast<std::size_t>(step)] + entered;
      state.counts.cumulative_out[static_cast<std::size_t>(step + 1)] =
          state.counts.cumulative_out[static_cast<std::size_t>(step)] + exited;
      (void)corridor_id;
    }
  }

  for (auto& fr : result.fleet) {
    if (fr.exited_vehicles > kTol) fr.mean_trip_time_hr /= fr.exited_vehicles;
  }
  for (auto& [corridor_id, state] : states) {
    result.corridor_counts.emplace(corridor_id, std::move(state.counts));
  }
  return result;
}

StudyCase make_study_case(int fleet_count = 4,
                          double short_corridor_capacity = 1.0) {
  StudyCase data;
  data.cheap_generation_cost = {15.0, 15.0, 60.0, 60.0,
                                20.0, 20.0, 20.0, 20.0};
  data.local_generation_cost = {95.0, 95.0, 140.0, 140.0,
                                100.0, 100.0, 100.0, 100.0};
  data.corridors = {
      {1, 1, short_corridor_capacity, 4.0},
      {2, 2, 2.0, 8.0},
  };
  data.station_capacity_mw = {{101, 0.40}, {102, 0.20}};
  data.station_bus = {{101, 1}, {102, 2}};

  for (int w = 0; w < fleet_count; ++w) {
    data.fleets.push_back({w + 1, 1.0, 0.05, 0.20, 6});
    data.plans.push_back({
        {0, 1, 0, 102},
        {1, 1, 1, 102},
        {2, 2, 0, 101},
        {3, 2, 1, 101},
    });
  }
  return data;
}

StudyCase make_heterogeneous_case(int fleet_count = 6) {
  StudyCase data;
  data.traffic_steps = 24;
  data.power_steps = 10;
  data.value_of_time_per_vehicle_hr = 24.0;
  data.shortfall_penalty_per_mwh = 800.0;
  data.line_limit_mw = 0.20;
  data.cheap_generator_capacity_mw = 1.20;
  data.cheap_generation_cost = {
      18.0, 18.0, 42.0, 68.0, 68.0, 32.0, 24.0, 24.0, 24.0, 24.0};
  data.local_generation_cost = {
      82.0, 82.0, 108.0, 145.0, 145.0, 96.0, 88.0, 88.0, 88.0, 88.0};
  data.corridors = {
      {1, 1, 0.75, 4.0, 0.25},
      {2, 2, 1.00, 8.0, 0.50},
      {3, 3, 2.00, 12.0, 0.75},
  };
  data.station_capacity_mw = {{101, 0.40}, {102, 0.20}, {103, 0.40}};
  data.station_bus = {{101, 1}, {102, 2}, {103, 1}};

  const std::vector<int> deadlines = {5, 5, 6, 6, 7, 7, 8, 8};
  for (int w = 0; w < fleet_count; ++w) {
    data.fleets.push_back(
        {w + 1, 1.0, 0.05, 0.20,
         deadlines[static_cast<std::size_t>(w) % deadlines.size()]});
    const int late_departure = 1 + (w % 2);
    data.plans.push_back({
        {0, 1, 0, 102},
        {1, 1, late_departure, 102},
        {2, 2, 0, 101},
        {3, 2, late_departure, 101},
        {4, 3, 0, 103},
        {5, 3, late_departure, 103},
    });
  }
  return data;
}

double free_flow_trip_time(const StudyCase& data, const Plan& plan) {
  for (const auto& corridor : data.corridors) {
    if (corridor.id == plan.corridor_id) {
      return corridor.free_flow_steps * data.dt_hr;
    }
  }
  return std::numeric_limits<double>::infinity();
}

struct OperatingCost {
  double total{std::numeric_limits<double>::infinity()};
  double travel{0.0};
  double generation{0.0};
  double shortfall{0.0};
  std::vector<int> charging_step;
};

double generation_cost(const StudyCase& data,
                       const std::vector<double>& bus1_load,
                       const std::vector<double>& bus2_load) {
  double cost = 0.0;
  for (int h = 0; h < data.power_steps; ++h) {
    const double local1 = bus1_load[static_cast<std::size_t>(h)];
    const double local2 = bus2_load[static_cast<std::size_t>(h)];
    const double available_export = std::max(
        0.0, data.cheap_generator_capacity_mw - local1);
    const double export_mw = std::min(
        {local2, data.line_limit_mw, available_export});
    const double cheap_mw = local1 + export_mw;
    const double local_mw = local2 - export_mw;
    cost += data.dt_hr *
        (data.cheap_generation_cost[static_cast<std::size_t>(h)] * cheap_mw +
         data.local_generation_cost[static_cast<std::size_t>(h)] * local_mw);
  }
  return cost;
}

OperatingCost reschedule_charging(
    const StudyCase& data,
    const std::vector<int>& assignment,
    const std::vector<double>& arrival_time_hr,
    const std::vector<double>& trip_time_hr) {
  OperatingCost best;
  std::vector<double> bus1_load(static_cast<std::size_t>(data.power_steps), 0.0);
  std::vector<double> bus2_load(static_cast<std::size_t>(data.power_steps), 0.0);
  std::unordered_map<int, std::vector<double>> station_load;
  for (const auto& [station_id, capacity] : data.station_capacity_mw) {
    (void)capacity;
    station_load[station_id].assign(static_cast<std::size_t>(data.power_steps), 0.0);
  }
  std::vector<int> charging_step(data.fleets.size(), -1);

  double travel_cost = 0.0;
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    travel_cost += data.value_of_time_per_vehicle_hr *
                   data.fleets[w].vehicles * trip_time_hr[w];
  }

  std::function<void(std::size_t, double)> visit =
      [&](std::size_t w, double shortfall_cost) {
    if (travel_cost + shortfall_cost >= best.total - kTol) return;
    if (w == data.fleets.size()) {
      const double grid_cost = generation_cost(data, bus1_load, bus2_load);
      const double total = travel_cost + shortfall_cost + grid_cost;
      if (total < best.total - kTol) {
        best.total = total;
        best.travel = travel_cost;
        best.generation = grid_cost;
        best.shortfall = shortfall_cost;
        best.charging_step = charging_step;
      }
      return;
    }

    const Fleet& fleet = data.fleets[w];
    const Plan& plan = data.plans[w][static_cast<std::size_t>(assignment[w])];
    const int first_step = std::max(
        0, static_cast<int>(std::ceil(arrival_time_hr[w] / data.dt_hr - 1e-8)));
    const int last_step = std::min(fleet.charging_deadline_step, data.power_steps);
    for (int h = first_step; h < last_step; ++h) {
      auto& load = station_load.at(plan.station_id);
      if (load[static_cast<std::size_t>(h)] + fleet.charging_power_mw >
          data.station_capacity_mw.at(plan.station_id) + kTol) {
        continue;
      }
      load[static_cast<std::size_t>(h)] += fleet.charging_power_mw;
      auto& bus_load = data.station_bus.at(plan.station_id) == 1
          ? bus1_load : bus2_load;
      bus_load[static_cast<std::size_t>(h)] += fleet.charging_power_mw;
      charging_step[w] = h;
      visit(w + 1, shortfall_cost);
      charging_step[w] = -1;
      bus_load[static_cast<std::size_t>(h)] -= fleet.charging_power_mw;
      load[static_cast<std::size_t>(h)] -= fleet.charging_power_mw;
    }

    charging_step[w] = -1;
    visit(w + 1,
          shortfall_cost + data.shortfall_penalty_per_mwh * fleet.energy_mwh);
  };

  visit(0, 0.0);
  return best;
}

OperatingCost free_flow_cost(const StudyCase& data,
                             const std::vector<int>& assignment) {
  std::vector<double> arrivals(data.fleets.size(), 0.0);
  std::vector<double> travel(data.fleets.size(), 0.0);
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    const Plan& plan = data.plans[w][static_cast<std::size_t>(assignment[w])];
    travel[w] = free_flow_trip_time(data, plan);
    arrivals[w] = plan.departure_step * data.dt_hr + travel[w];
  }
  return reschedule_charging(data, assignment, arrivals, travel);
}

OperatingCost full_ctm_cost(const StudyCase& data,
                            const std::vector<int>& assignment,
                            CTMReplayResult* replay_out = nullptr) {
  const std::vector<bool> loaded(data.fleets.size(), true);
  CTMReplayResult replay = run_fifo_corridor_ctm(data, assignment, loaded);
  std::vector<double> arrivals(data.fleets.size(), 0.0);
  std::vector<double> travel(data.fleets.size(), 0.0);
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    arrivals[w] = replay.fleet[w].last_arrival_time_hr;
    travel[w] = replay.fleet[w].mean_trip_time_hr;
    if (replay.fleet[w].exited_vehicles + kTol < data.fleets[w].vehicles) {
      arrivals[w] = data.traffic_steps * data.dt_hr;
      travel[w] = arrivals[w] -
          data.plans[w][static_cast<std::size_t>(assignment[w])].departure_step *
              data.dt_hr;
    }
  }
  if (replay_out != nullptr) *replay_out = replay;
  return reschedule_charging(data, assignment, arrivals, travel);
}

std::string assignment_key(const std::vector<int>& assignment) {
  std::string key;
  for (int plan : assignment) {
    if (!key.empty()) key.push_back('-');
    key += std::to_string(plan);
  }
  return key;
}

void enumerate_assignments(
    const std::vector<std::vector<int>>& allowed,
    const std::function<void(const std::vector<int>&)>& visitor) {
  std::vector<int> assignment(allowed.size(), 0);
  std::function<void(std::size_t)> visit = [&](std::size_t w) {
    if (w == allowed.size()) {
      visitor(assignment);
      return;
    }
    for (int plan : allowed[w]) {
      assignment[w] = plan;
      visit(w + 1);
    }
  };
  visit(0);
}

std::vector<std::vector<int>> all_plan_sets(const StudyCase& data) {
  std::vector<std::vector<int>> allowed(data.fleets.size());
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    allowed[w].resize(data.plans[w].size());
    std::iota(allowed[w].begin(), allowed[w].end(), 0);
  }
  return allowed;
}

struct CachedEvaluator {
  const StudyCase& data;
  std::unordered_map<std::string, OperatingCost> full_cache;
  std::unordered_map<std::string, CTMReplayResult> replay_cache;
  int full_ctm_calls{0};

  const OperatingCost& full(const std::vector<int>& assignment) {
    const std::string key = assignment_key(assignment);
    auto it = full_cache.find(key);
    if (it == full_cache.end()) {
      ++full_ctm_calls;
      CTMReplayResult replay;
      const OperatingCost value = full_ctm_cost(data, assignment, &replay);
      replay_cache.emplace(key, std::move(replay));
      it = full_cache.emplace(key, value).first;
    }
    return it->second;
  }

  const CTMReplayResult& replay(const std::vector<int>& assignment) {
    const std::string key = assignment_key(assignment);
    (void)full(assignment);
    return replay_cache.at(key);
  }
};

struct NodeBound {
  double value{std::numeric_limits<double>::infinity()};
  std::vector<int> completion;
  std::vector<double> partial_arrival;
  std::vector<double> partial_trip;
};

NodeBound calculate_node_bound_enumerated(
    const StudyCase& data,
    const std::vector<std::vector<int>>& allowed) {
  std::vector<bool> fixed(data.fleets.size(), false);
  std::vector<int> fixed_assignment(data.fleets.size(), 0);
  for (std::size_t w = 0; w < allowed.size(); ++w) {
    fixed[w] = allowed[w].size() == 1;
    fixed_assignment[w] = allowed[w].front();
  }

  CTMReplayResult partial = run_fifo_corridor_ctm(data, fixed_assignment, fixed);
  NodeBound bound;
  bound.partial_arrival.assign(data.fleets.size(), 0.0);
  bound.partial_trip.assign(data.fleets.size(), 0.0);
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    if (fixed[w]) {
      bound.partial_arrival[w] = partial.fleet[w].last_arrival_time_hr;
      bound.partial_trip[w] = partial.fleet[w].mean_trip_time_hr;
    }
  }

  enumerate_assignments(allowed, [&](const std::vector<int>& assignment) {
    std::vector<double> arrivals(data.fleets.size(), 0.0);
    std::vector<double> travel(data.fleets.size(), 0.0);
    for (std::size_t w = 0; w < data.fleets.size(); ++w) {
      const Plan& plan = data.plans[w][static_cast<std::size_t>(assignment[w])];
      if (fixed[w]) {
        arrivals[w] = bound.partial_arrival[w];
        travel[w] = bound.partial_trip[w];
      } else {
        travel[w] = free_flow_trip_time(data, plan);
        arrivals[w] = plan.departure_step * data.dt_hr + travel[w];
      }
    }
    const OperatingCost candidate =
        reschedule_charging(data, assignment, arrivals, travel);
    if (candidate.total < bound.value - kTol) {
      bound.value = candidate.total;
      bound.completion = assignment;
    }
  });
  return bound;
}

NodeBound calculate_node_bound_milp(
    const StudyCase& data,
    const std::vector<std::vector<int>>& allowed) {
  using hacdcpf::engine::HighsAdapter;
  using hacdcpf::engine::LPModel;
  using hacdcpf::engine::MIPModel;
  using hacdcpf::engine::Sense;
  using hacdcpf::engine::VariableMeta;
  using hacdcpf::engine::VarType;

  std::vector<bool> fixed(data.fleets.size(), false);
  std::vector<int> fixed_assignment(data.fleets.size(), 0);
  for (std::size_t w = 0; w < allowed.size(); ++w) {
    fixed[w] = allowed[w].size() == 1;
    fixed_assignment[w] = allowed[w].front();
  }

  const CTMReplayResult partial =
      run_fifo_corridor_ctm(data, fixed_assignment, fixed);
  NodeBound bound;
  bound.partial_arrival.assign(data.fleets.size(), 0.0);
  bound.partial_trip.assign(data.fleets.size(), 0.0);

  MIPModel mip;
  LPModel& lp = mip.linear_part;
  lp.sense = Sense::Minimize;
  std::vector<VariableMeta> vars;
  std::vector<double> objective;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  std::vector<double> beq;
  std::vector<double> bineq;

  auto add_binary = [&](double cost, const std::string& name) {
    const int index = static_cast<int>(vars.size());
    vars.push_back({VarType::Binary, 0.0, 1.0, name});
    objective.push_back(cost);
    mip.binary_idx.push_back(index);
    return index;
  };
  auto add_continuous = [&](double lower, double upper, double cost,
                            const std::string& name) {
    const int index = static_cast<int>(vars.size());
    vars.push_back({VarType::Continuous, lower, upper, name});
    objective.push_back(cost);
    return index;
  };
  auto add_eq = [&](double rhs) {
    const int row = static_cast<int>(beq.size());
    beq.push_back(rhs);
    return row;
  };
  auto add_ineq = [&](double rhs) {
    const int row = static_cast<int>(bineq.size());
    bineq.push_back(rhs);
    return row;
  };

  std::vector<std::vector<int>> z(data.fleets.size());
  struct ChargeVariable {
    int index{-1};
    std::size_t fleet{0};
    int plan{0};
    int step{0};
    int station{0};
  };
  std::vector<ChargeVariable> charge_vars;
  std::vector<int> shortfall(data.fleets.size(), -1);

  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    z[w].assign(data.plans[w].size(), -1);
    const int plan_row = add_eq(1.0);
    const int service_row = add_eq(1.0);
    const Fleet& fleet = data.fleets[w];

    if (fixed[w]) {
      bound.partial_arrival[w] = partial.fleet[w].last_arrival_time_hr;
      bound.partial_trip[w] = partial.fleet[w].mean_trip_time_hr;
    }

    for (int k : allowed[w]) {
      const Plan& plan = data.plans[w][static_cast<std::size_t>(k)];
      const double trip_time = fixed[w]
          ? bound.partial_trip[w]
          : free_flow_trip_time(data, plan);
      const double arrival = fixed[w]
          ? bound.partial_arrival[w]
          : plan.departure_step * data.dt_hr + trip_time;
      const double travel_cost = data.value_of_time_per_vehicle_hr *
                                 fleet.vehicles * trip_time;
      z[w][static_cast<std::size_t>(k)] =
          add_binary(travel_cost,
                     "z_" + std::to_string(w) + "_" + std::to_string(k));
      eq_trips.emplace_back(plan_row, z[w][static_cast<std::size_t>(k)], 1.0);

      const int first_step = std::max(
          0, static_cast<int>(std::ceil(arrival / data.dt_hr - 1e-8)));
      const int last_step =
          std::min(fleet.charging_deadline_step, data.power_steps);
      for (int h = first_step; h < last_step; ++h) {
        const int x = add_binary(
            0.0, "x_" + std::to_string(w) + "_" + std::to_string(k) +
                     "_" + std::to_string(h));
        charge_vars.push_back({x, w, k, h, plan.station_id});
        eq_trips.emplace_back(service_row, x, 1.0);
        const int linking_row = add_ineq(0.0);
        ineq_trips.emplace_back(linking_row, x, 1.0);
        ineq_trips.emplace_back(
            linking_row, z[w][static_cast<std::size_t>(k)], -1.0);
      }
    }

    shortfall[w] = add_binary(
        data.shortfall_penalty_per_mwh * fleet.energy_mwh,
        "shortfall_" + std::to_string(w));
    eq_trips.emplace_back(service_row, shortfall[w], 1.0);
  }

  std::map<std::pair<int, int>, int> station_rows;
  for (const auto& x : charge_vars) {
    const auto key = std::make_pair(x.station, x.step);
    auto [it, inserted] = station_rows.emplace(key, -1);
    if (inserted) {
      it->second = add_ineq(data.station_capacity_mw.at(x.station));
    }
    const Fleet& fleet = data.fleets[x.fleet];
    ineq_trips.emplace_back(
        it->second, x.index, fleet.vehicles * fleet.charging_power_mw);
  }

  const double max_load = std::accumulate(
      data.fleets.begin(), data.fleets.end(), 0.0,
      [](double total, const Fleet& fleet) {
        return total + fleet.vehicles * fleet.charging_power_mw;
      });
  for (int h = 0; h < data.power_steps; ++h) {
    const int bus1_row = add_eq(0.0);
    const int bus2_row = add_eq(0.0);
    const int generator1 = add_continuous(
        0.0, data.cheap_generator_capacity_mw,
        data.dt_hr * data.cheap_generation_cost[static_cast<std::size_t>(h)],
        "p1_" + std::to_string(h));
    const int generator2 = add_continuous(
        0.0, max_load,
        data.dt_hr * data.local_generation_cost[static_cast<std::size_t>(h)],
        "p2_" + std::to_string(h));
    const int line_flow = add_continuous(
        0.0, data.line_limit_mw, 0.0, "flow_" + std::to_string(h));
    eq_trips.emplace_back(bus1_row, generator1, 1.0);
    eq_trips.emplace_back(bus1_row, line_flow, -1.0);
    eq_trips.emplace_back(bus2_row, generator2, 1.0);
    eq_trips.emplace_back(bus2_row, line_flow, 1.0);

    for (const auto& x : charge_vars) {
      if (x.step != h) continue;
      const Fleet& fleet = data.fleets[x.fleet];
      const double power = fleet.vehicles * fleet.charging_power_mw;
      const int bus = data.station_bus.at(x.station);
      eq_trips.emplace_back(bus == 1 ? bus1_row : bus2_row, x.index, -power);
    }
  }

  lp.c = Eigen::VectorXd::Map(objective.data(),
                              static_cast<Eigen::Index>(objective.size()));
  lp.vars = std::move(vars);
  lp.Aeq.resize(static_cast<int>(beq.size()), lp.vars.size());
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Map(beq.data(),
                                static_cast<Eigen::Index>(beq.size()));
  lp.A.resize(static_cast<int>(bineq.size()), lp.vars.size());
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Map(bineq.data(),
                              static_cast<Eigen::Index>(bineq.size()));

  const auto solve = HighsAdapter{}.solve_milp(mip);
  if (!solve.stats.success ||
      solve.x.size() != static_cast<Eigen::Index>(lp.vars.size())) {
    return bound;
  }

  bound.value = lp.c.dot(solve.x);
  bound.completion.assign(data.fleets.size(), -1);
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    for (int k : allowed[w]) {
      const int index = z[w][static_cast<std::size_t>(k)];
      if (index >= 0 && solve.x[index] > 0.5) {
        bound.completion[w] = k;
        break;
      }
    }
  }
  return bound;
}

NodeBound calculate_node_bound(const StudyCase& data,
                               const std::vector<std::vector<int>>& allowed) {
  return calculate_node_bound_milp(data, allowed);
}

double true_node_value(CachedEvaluator& evaluator,
                       const std::vector<std::vector<int>>& allowed) {
  double best = std::numeric_limits<double>::infinity();
  enumerate_assignments(allowed, [&](const std::vector<int>& assignment) {
    best = std::min(best, evaluator.full(assignment).total);
  });
  return best;
}

struct SearchResult {
  double value{std::numeric_limits<double>::infinity()};
  std::vector<int> assignment;
  int generated_nodes{0};
  int pruned_nodes{0};
  int partial_ctm_calls{0};
  int processed_nodes{0};
  int leaf_values{0};
  int implied_pruned_combinations{0};
  double max_bound_violation{0.0};
  double min_child_strengthening{std::numeric_limits<double>::infinity()};
};

SearchResult run_partial_loading_search(const StudyCase& data,
                                        CachedEvaluator& evaluator,
                                        bool independently_validate = true) {
  struct Node {
    std::vector<std::vector<int>> allowed;
    NodeBound bound;
  };
  std::vector<Node> active;
  Node root;
  root.allowed = all_plan_sets(data);
  root.bound = calculate_node_bound(data, root.allowed);
  active.push_back(root);

  SearchResult result;
  result.generated_nodes = 1;
  result.partial_ctm_calls = 0;
  std::set<std::string> evaluated;
  std::set<std::string> pruned_combinations;
  CachedEvaluator validation_evaluator{data};

  auto mark_pruned = [&](const std::vector<std::vector<int>>& allowed) {
    enumerate_assignments(allowed, [&](const std::vector<int>& assignment) {
      const std::string key = assignment_key(assignment);
      if (!evaluated.count(key)) pruned_combinations.insert(key);
    });
  };

  while (!active.empty()) {
    auto best_node_it = std::min_element(
        active.begin(), active.end(),
        [](const Node& lhs, const Node& rhs) {
          return lhs.bound.value < rhs.bound.value;
        });
    Node node = *best_node_it;
    active.erase(best_node_it);
    ++result.processed_nodes;

    if (node.bound.value >= result.value - kTol) {
      ++result.pruned_nodes;
      mark_pruned(node.allowed);
      continue;
    }

    const std::string candidate_key = assignment_key(node.bound.completion);
    if (evaluated.insert(candidate_key).second) {
      const OperatingCost& candidate = evaluator.full(node.bound.completion);
      if (candidate.total < result.value - kTol) {
        result.value = candidate.total;
        result.assignment = node.bound.completion;
      }
    }

    if (independently_validate) {
      const double true_value =
          true_node_value(validation_evaluator, node.allowed);
      result.max_bound_violation = std::max(
          result.max_bound_violation, node.bound.value - true_value);
    }

    int branch_fleet = -1;
    double branch_score = -1.0;
    const CTMReplayResult& completion_replay = evaluator.replay(node.bound.completion);
    for (std::size_t w = 0; w < node.allowed.size(); ++w) {
      if (node.allowed[w].size() == 1) continue;
      const Plan& plan = data.plans[w]
          [static_cast<std::size_t>(node.bound.completion[w])];
      const double delay = std::max(
          0.0, completion_replay.fleet[w].mean_trip_time_hr -
                   free_flow_trip_time(data, plan));
      const double score = data.fleets[w].vehicles *
                           static_cast<double>(node.allowed[w].size() - 1) *
                           (delay + 1e-6);
      if (score > branch_score) {
        branch_score = score;
        branch_fleet = static_cast<int>(w);
      }
    }

    if (branch_fleet < 0) {
      ++result.leaf_values;
      CHECK(node.bound.value == Approx(evaluator.full(node.bound.completion).total)
                                    .margin(1e-8));
      continue;
    }

    for (int plan : node.allowed[static_cast<std::size_t>(branch_fleet)]) {
      Node child;
      child.allowed = node.allowed;
      child.allowed[static_cast<std::size_t>(branch_fleet)] = {plan};
      child.bound = calculate_node_bound(data, child.allowed);
      ++result.generated_nodes;
      ++result.partial_ctm_calls;
      result.min_child_strengthening = std::min(
          result.min_child_strengthening,
          child.bound.value - node.bound.value);

      if (independently_validate) {
        const double child_true =
            true_node_value(validation_evaluator, child.allowed);
        result.max_bound_violation = std::max(
            result.max_bound_violation, child.bound.value - child_true);
      }

      if (child.bound.value >= result.value - kTol) {
        ++result.pruned_nodes;
        mark_pruned(child.allowed);
      } else {
        active.push_back(std::move(child));
      }
    }
  }
  result.implied_pruned_combinations =
      static_cast<int>(pruned_combinations.size());
  return result;
}

struct ExhaustiveResult {
  double free_flow_optimum{std::numeric_limits<double>::infinity()};
  double ctm_optimum{std::numeric_limits<double>::infinity()};
  std::vector<int> free_flow_assignment;
  std::vector<int> ctm_assignment;
  int combinations{0};
  double max_pointwise_violation{0.0};
};

ExhaustiveResult run_exhaustive(const StudyCase& data,
                                CachedEvaluator& evaluator) {
  ExhaustiveResult result;
  const std::vector<std::vector<int>> all = all_plan_sets(data);
  enumerate_assignments(all, [&](const std::vector<int>& assignment) {
    ++result.combinations;
    const OperatingCost lower = free_flow_cost(data, assignment);
    const OperatingCost& full = evaluator.full(assignment);
    result.max_pointwise_violation = std::max(
        result.max_pointwise_violation, lower.total - full.total);
    if (lower.total < result.free_flow_optimum - kTol) {
      result.free_flow_optimum = lower.total;
      result.free_flow_assignment = assignment;
    }
    if (full.total < result.ctm_optimum - kTol) {
      result.ctm_optimum = full.total;
      result.ctm_assignment = assignment;
    }
  });
  return result;
}

void print_assignment(const std::vector<int>& assignment) {
  std::printf("[");
  for (std::size_t i = 0; i < assignment.size(); ++i) {
    std::printf("%s%d", i == 0 ? "" : ",", assignment[i]);
  }
  std::printf("]");
}

}  // namespace

TEST_CASE("Congestion approximation error: exhaustive small-case verification",
          "[evpt_approximation][small_case]") {
  const StudyCase data = make_study_case();
  CachedEvaluator evaluator{data};
  const ExhaustiveResult exhaustive = run_exhaustive(data, evaluator);

  REQUIRE(exhaustive.combinations == 256);
  CHECK(exhaustive.max_pointwise_violation <= 1e-8);

  const OperatingCost& ff_under_ctm = evaluator.full(exhaustive.free_flow_assignment);
  const double actual_loss = ff_under_ctm.total - exhaustive.ctm_optimum;
  const double error_bound = ff_under_ctm.total - exhaustive.free_flow_optimum;
  CHECK(actual_loss >= -1e-8);
  CHECK(actual_loss <= error_bound + 1e-8);
  CHECK(error_bound > 1e-6);

  CTMReplayResult ff_replay;
  const OperatingCost ff_full_detail =
      full_ctm_cost(data, exhaustive.free_flow_assignment, &ff_replay);
  CTMReplayResult optimum_replay;
  const OperatingCost optimum_detail =
      full_ctm_cost(data, exhaustive.ctm_assignment, &optimum_replay);
  const OperatingCost ff_lower_detail =
      free_flow_cost(data, exhaustive.free_flow_assignment);
  for (const auto& [corridor_id, counts] : ff_replay.corridor_counts) {
    (void)corridor_id;
    REQUIRE(counts.cumulative_in.size() == counts.cumulative_out.size());
    for (std::size_t k = 1; k < counts.cumulative_in.size(); ++k) {
      CHECK(counts.cumulative_in[k] + kTol >= counts.cumulative_in[k - 1]);
      CHECK(counts.cumulative_out[k] + kTol >= counts.cumulative_out[k - 1]);
      CHECK(counts.cumulative_out[k] <= counts.cumulative_in[k] + kTol);
    }
  }

  std::printf("\nSmall coupled case: %d plan combinations\n", exhaustive.combinations);
  std::printf("free-flow optimum L(z_ff) = %.6f, z_ff = ",
              exhaustive.free_flow_optimum);
  print_assignment(exhaustive.free_flow_assignment);
  std::printf("\nF(z_ff) = %.6f\n", ff_under_ctm.total);
  std::printf("CTM optimum F(z*) = %.6f, z* = ", exhaustive.ctm_optimum);
  print_assignment(exhaustive.ctm_assignment);
  std::printf("\nactual loss = %.6f, a posteriori bound = %.6f\n",
              actual_loss, error_bound);
  std::printf("cost components (travel, generation, shortfall)\n");
  std::printf("L(z_ff): (%.4f, %.4f, %.4f)\n",
              ff_lower_detail.travel, ff_lower_detail.generation,
              ff_lower_detail.shortfall);
  std::printf("F(z_ff): (%.4f, %.4f, %.4f)\n",
              ff_full_detail.travel, ff_full_detail.generation,
              ff_full_detail.shortfall);
  std::printf("F(z*):   (%.4f, %.4f, %.4f)\n",
              optimum_detail.travel, optimum_detail.generation,
              optimum_detail.shortfall);
  std::printf("fleet  z_ff_plan  z_ff_trip  z_ff_arrival  z*_plan  z*_trip  z*_arrival\n");
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    std::printf("%5zu %10d %10.4f %12.4f %8d %9.4f %11.4f\n",
                w + 1, exhaustive.free_flow_assignment[w],
                ff_replay.fleet[w].mean_trip_time_hr,
                ff_replay.fleet[w].last_arrival_time_hr,
                exhaustive.ctm_assignment[w],
                optimum_replay.fleet[w].mean_trip_time_hr,
                optimum_replay.fleet[w].last_arrival_time_hr);
  }
}

TEST_CASE("Congestion approximation error: partial-loading search",
          "[evpt_approximation][branch_and_bound]") {
  const StudyCase data = make_study_case();
  CachedEvaluator exhaustive_evaluator{data};
  const ExhaustiveResult exhaustive = run_exhaustive(data, exhaustive_evaluator);

  CachedEvaluator search_evaluator{data};
  const SearchResult search = run_partial_loading_search(data, search_evaluator);

  CHECK(search.value == Approx(exhaustive.ctm_optimum).margin(1e-8));
  CHECK(search_evaluator.full(search.assignment).total ==
        Approx(exhaustive.ctm_optimum).margin(1e-8));
  CHECK(search.max_bound_violation <= 1e-8);
  CHECK(search.min_child_strengthening >= -1e-8);
  CHECK(search_evaluator.full_ctm_calls < exhaustive.combinations);
  CHECK(search.pruned_nodes > 0);
  CHECK(search_evaluator.full_ctm_calls + search.implied_pruned_combinations ==
        exhaustive.combinations);

  std::printf("\nPartial-loading branch-and-bound\n");
  std::printf("objective = %.6f, assignment = ", search.value);
  print_assignment(search.assignment);
  std::printf("\ncreated nodes = %d, processed nodes = %d, pruned nodes = %d\n",
              search.generated_nodes, search.processed_nodes, search.pruned_nodes);
  std::printf("partial CTM calls = %d, full CTM calls = %d, exhaustive calls = %d\n",
              search.partial_ctm_calls, search_evaluator.full_ctm_calls,
              exhaustive.combinations);
  std::printf("implied pruned combinations = %d, max bound violation = %.3e\n",
              search.implied_pruned_combinations, search.max_bound_violation);
}

TEST_CASE("Congestion approximation error: node MILP matches enumeration",
          "[evpt_approximation][node_milp]") {
  const StudyCase data = make_study_case();
  const std::vector<std::vector<std::vector<int>>> nodes = {
      {{0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}},
      {{0}, {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}},
      {{1}, {2}, {0, 1, 2, 3}, {0, 1, 2, 3}},
      {{0}, {0}, {2}, {0, 1, 2, 3}},
      {{3}, {2}, {1}, {0}},
  };
  for (const auto& allowed : nodes) {
    const NodeBound enumerated =
        calculate_node_bound_enumerated(data, allowed);
    const NodeBound milp = calculate_node_bound_milp(data, allowed);
    CHECK(milp.value == Approx(enumerated.value).margin(1e-8));
    CHECK(std::isfinite(milp.value));
    CHECK(milp.completion.size() == data.fleets.size());
  }
}

TEST_CASE("Congestion approximation error: demand and capacity sweep",
          "[evpt_approximation][analysis]") {
  std::printf("\ncapacity  fleets  combinations  J_ff      F(z_ff)   J_ctm     bound\n");
  for (double capacity : {0.75, 1.0, 1.5, 2.0}) {
    for (int fleets : {2, 3, 4}) {
      const StudyCase data = make_study_case(fleets, capacity);
      CachedEvaluator evaluator{data};
      const ExhaustiveResult exhaustive = run_exhaustive(data, evaluator);
      const double ff_ctm = evaluator.full(exhaustive.free_flow_assignment).total;
      std::printf("%8.2f %7d %13d %9.4f %10.4f %9.4f %9.4f\n",
                  capacity, fleets, exhaustive.combinations,
                  exhaustive.free_flow_optimum, ff_ctm,
                  exhaustive.ctm_optimum,
                  ff_ctm - exhaustive.free_flow_optimum);
      CHECK(exhaustive.max_pointwise_violation <= 1e-8);
      CHECK(ff_ctm - exhaustive.ctm_optimum <=
            ff_ctm - exhaustive.free_flow_optimum + 1e-8);
    }
  }
}

TEST_CASE("Congestion approximation error: charging and grid sensitivity",
          "[evpt_approximation][analysis]") {
  auto evaluate = [](const char* factor, double level, StudyCase data) {
    CachedEvaluator evaluator{data};
    const ExhaustiveResult exhaustive = run_exhaustive(data, evaluator);
    const OperatingCost ff_ctm =
        evaluator.full(exhaustive.free_flow_assignment);
    const OperatingCost optimum =
        evaluator.full(exhaustive.ctm_assignment);
    const double actual_loss = ff_ctm.total - optimum.total;
    const double error_bound =
        ff_ctm.total - exhaustive.free_flow_optimum;
    auto short_corridor_count = [&](const std::vector<int>& assignment) {
      int count = 0;
      for (std::size_t w = 0; w < assignment.size(); ++w) {
        const Plan& plan =
            data.plans[w][static_cast<std::size_t>(assignment[w])];
        if (plan.corridor_id == 1) ++count;
      }
      return count;
    };
    std::printf(
        "%-11s %6.2f %9.4f %9.4f %9.4f %9.4f %10.4f %10.4f %8d %9d\n",
        factor, level, exhaustive.free_flow_optimum, ff_ctm.total,
        exhaustive.ctm_optimum, actual_loss, error_bound,
        ff_ctm.shortfall - optimum.shortfall,
        short_corridor_count(exhaustive.free_flow_assignment),
        short_corridor_count(exhaustive.ctm_assignment));
    CHECK(exhaustive.max_pointwise_violation <= 1e-8);
    CHECK(actual_loss <= error_bound + 1e-8);
    CHECK(exhaustive.ctm_optimum == Approx(optimum.total).margin(1e-8));
  };

  std::printf(
      "\nfactor       level      J_ff   F(z_ff)     J_ctm      loss      bound "
      "d_shortfall ff_short ctm_short\n");
  for (int deadline : {3, 4, 5, 6}) {
    StudyCase data = make_study_case();
    for (auto& fleet : data.fleets) {
      fleet.charging_deadline_step = deadline;
    }
    evaluate("deadline", static_cast<double>(deadline), std::move(data));
  }
  for (double station_capacity : {0.20, 0.40, 0.60}) {
    StudyCase data = make_study_case();
    data.station_capacity_mw.at(102) = station_capacity;
    for (auto& fleet : data.fleets) {
      fleet.charging_deadline_step = 3;
    }
    evaluate("station_MW", station_capacity, std::move(data));
  }
  for (double line_limit : {0.00, 0.15, 0.30}) {
    StudyCase data = make_study_case();
    data.line_limit_mw = line_limit;
    evaluate("line_MW", line_limit, std::move(data));
  }
}

TEST_CASE("Congestion approximation error: search scaling",
          "[evpt_approximation][scaling]") {
  using Clock = std::chrono::steady_clock;
  std::printf(
      "\nfleets combinations enum_ms search_ms full_ctm partial_ctm nodes "
      "reduction_pct\n");
  for (int fleets : {2, 3, 4, 5}) {
    const StudyCase data = make_study_case(fleets);
    std::vector<double> exhaustive_times;
    std::vector<double> search_times;
    int combinations = 0;
    int full_ctm_calls = 0;
    int partial_ctm_calls = 0;
    int generated_nodes = 0;
    for (int repetition = 0; repetition < 3; ++repetition) {
      CachedEvaluator exhaustive_evaluator{data};
      const auto exhaustive_start = Clock::now();
      const ExhaustiveResult exhaustive =
          run_exhaustive(data, exhaustive_evaluator);
      const auto exhaustive_end = Clock::now();

      CachedEvaluator search_evaluator{data};
      const auto search_start = Clock::now();
      const SearchResult search =
          run_partial_loading_search(data, search_evaluator, false);
      const auto search_end = Clock::now();

      exhaustive_times.push_back(
          std::chrono::duration<double, std::milli>(
              exhaustive_end - exhaustive_start).count());
      search_times.push_back(
          std::chrono::duration<double, std::milli>(
              search_end - search_start).count());
      combinations = exhaustive.combinations;
      full_ctm_calls = search_evaluator.full_ctm_calls;
      partial_ctm_calls = search.partial_ctm_calls;
      generated_nodes = search.generated_nodes;
      CHECK(search.value == Approx(exhaustive.ctm_optimum).margin(1e-8));
      CHECK(search_evaluator.full_ctm_calls < exhaustive.combinations);
    }
    std::sort(exhaustive_times.begin(), exhaustive_times.end());
    std::sort(search_times.begin(), search_times.end());
    const double exhaustive_ms = exhaustive_times[1];
    const double search_ms = search_times[1];
    const double reduction = 100.0 *
        (1.0 - static_cast<double>(full_ctm_calls) /
                   static_cast<double>(combinations));

    std::printf("%6d %12d %7.3f %9.3f %8d %11d %5d %13.1f\n",
                fleets, combinations, exhaustive_ms, search_ms,
                full_ctm_calls, partial_ctm_calls, generated_nodes, reduction);
  }
}

TEST_CASE("Congestion approximation error: heterogeneous three-corridor case",
          "[evpt_approximation][heterogeneous]") {
  using Clock = std::chrono::steady_clock;
  const StudyCase data = make_heterogeneous_case(6);

  CachedEvaluator exhaustive_evaluator{data};
  const auto exhaustive_start = Clock::now();
  const ExhaustiveResult exhaustive =
      run_exhaustive(data, exhaustive_evaluator);
  const auto exhaustive_end = Clock::now();

  CachedEvaluator search_evaluator{data};
  const auto search_start = Clock::now();
  const SearchResult search =
      run_partial_loading_search(data, search_evaluator, false);
  const auto search_end = Clock::now();

  const OperatingCost free_flow_under_ctm =
      exhaustive_evaluator.full(exhaustive.free_flow_assignment);
  const OperatingCost optimum =
      exhaustive_evaluator.full(exhaustive.ctm_assignment);
  const OperatingCost free_flow_detail =
      free_flow_cost(data, exhaustive.free_flow_assignment);
  CTMReplayResult free_flow_replay;
  const OperatingCost free_flow_ctm_detail =
      full_ctm_cost(data, exhaustive.free_flow_assignment, &free_flow_replay);
  CTMReplayResult optimum_replay;
  const OperatingCost optimum_detail =
      full_ctm_cost(data, exhaustive.ctm_assignment, &optimum_replay);
  const auto all_plans = all_plan_sets(data);
  const NodeBound root_milp = calculate_node_bound_milp(data, all_plans);
  const NodeBound root_enumerated =
      calculate_node_bound_enumerated(data, all_plans);
  std::vector<std::vector<std::vector<int>>> sampled_nodes;
  auto node_1 = all_plans;
  node_1[0] = {0};
  sampled_nodes.push_back(node_1);
  auto node_2 = node_1;
  node_2[1] = {1};
  node_2[2] = {2, 3, 4, 5};
  sampled_nodes.push_back(node_2);
  auto node_3 = node_2;
  node_3[2] = {2};
  node_3[3] = {0, 1};
  sampled_nodes.push_back(node_3);

  auto route_counts = [&](const std::vector<int>& assignment) {
    std::vector<int> counts(data.corridors.size(), 0);
    for (std::size_t w = 0; w < assignment.size(); ++w) {
      const Plan& plan =
          data.plans[w][static_cast<std::size_t>(assignment[w])];
      ++counts[static_cast<std::size_t>(plan.corridor_id - 1)];
    }
    return counts;
  };
  const auto ff_routes = route_counts(exhaustive.free_flow_assignment);
  const auto optimum_routes = route_counts(exhaustive.ctm_assignment);
  const double exhaustive_ms =
      std::chrono::duration<double, std::milli>(
          exhaustive_end - exhaustive_start).count();
  const double search_ms =
      std::chrono::duration<double, std::milli>(
          search_end - search_start).count();

  std::printf("\nHeterogeneous three-corridor case\n");
  std::printf("fleets = %zu, plans/fleet = 6, combinations = %d\n",
              data.fleets.size(), exhaustive.combinations);
  std::printf("J_ff = %.4f, F(z_ff) = %.4f, J_ctm = %.4f\n",
              exhaustive.free_flow_optimum, free_flow_under_ctm.total,
              exhaustive.ctm_optimum);
  std::printf("actual loss = %.4f, bound = %.4f\n",
              free_flow_under_ctm.total - optimum.total,
              free_flow_under_ctm.total - exhaustive.free_flow_optimum);
  std::printf("cost components (travel, generation, shortfall)\n");
  std::printf("L(z_ff): (%.4f, %.4f, %.4f)\n",
              free_flow_detail.travel, free_flow_detail.generation,
              free_flow_detail.shortfall);
  std::printf("F(z_ff): (%.4f, %.4f, %.4f)\n",
              free_flow_ctm_detail.travel, free_flow_ctm_detail.generation,
              free_flow_ctm_detail.shortfall);
  std::printf("F(z*):   (%.4f, %.4f, %.4f)\n",
              optimum_detail.travel, optimum_detail.generation,
              optimum_detail.shortfall);
  std::printf("z_ff = ");
  print_assignment(exhaustive.free_flow_assignment);
  std::printf(", route counts = [%d,%d,%d]\n",
              ff_routes[0], ff_routes[1], ff_routes[2]);
  std::printf("z* = ");
  print_assignment(exhaustive.ctm_assignment);
  std::printf(", route counts = [%d,%d,%d]\n",
              optimum_routes[0], optimum_routes[1], optimum_routes[2]);
  std::printf(
      "enumeration_ms = %.3f, search_ms = %.3f, full_ctm = %d, "
      "partial_ctm = %d, nodes = %d, reduction = %.1f%%\n",
      exhaustive_ms, search_ms, search_evaluator.full_ctm_calls,
      search.partial_ctm_calls, search.generated_nodes,
      100.0 * (1.0 - static_cast<double>(search_evaluator.full_ctm_calls) /
                         static_cast<double>(exhaustive.combinations)));
  std::printf(
      "fleet ff_plan ff_trip ff_arrival opt_plan opt_trip opt_arrival\n");
  for (std::size_t w = 0; w < data.fleets.size(); ++w) {
    std::printf("%5zu %7d %7.4f %10.4f %8d %8.4f %11.4f\n",
                w + 1, exhaustive.free_flow_assignment[w],
                free_flow_replay.fleet[w].mean_trip_time_hr,
                free_flow_replay.fleet[w].last_arrival_time_hr,
                exhaustive.ctm_assignment[w],
                optimum_replay.fleet[w].mean_trip_time_hr,
                optimum_replay.fleet[w].last_arrival_time_hr);
  }

  REQUIRE(exhaustive.combinations == 46656);
  CHECK(exhaustive.max_pointwise_violation <= 1e-8);
  CHECK(root_milp.value == Approx(root_enumerated.value).margin(1e-8));
  CHECK(root_milp.value <= exhaustive.ctm_optimum + 1e-8);
  for (const auto& allowed : sampled_nodes) {
    const NodeBound milp = calculate_node_bound_milp(data, allowed);
    const NodeBound enumerated = calculate_node_bound_enumerated(data, allowed);
    const double true_value = true_node_value(exhaustive_evaluator, allowed);
    CHECK(milp.value == Approx(enumerated.value).margin(1e-8));
    CHECK(milp.value <= true_value + 1e-8);
  }
  CHECK(search.value == Approx(exhaustive.ctm_optimum).margin(1e-8));
  CHECK(search_evaluator.full_ctm_calls < exhaustive.combinations);
  CHECK(search_evaluator.full_ctm_calls + search.implied_pruned_combinations ==
        exhaustive.combinations);
  CHECK(std::accumulate(optimum_routes.begin(), optimum_routes.end(), 0) ==
        static_cast<int>(data.fleets.size()));
  CHECK(std::all_of(optimum_routes.begin(), optimum_routes.end(),
                    [](int count) { return count > 0; }));
  CHECK(free_flow_under_ctm.total - optimum.total <=
        free_flow_under_ctm.total - exhaustive.free_flow_optimum + 1e-8);
}
