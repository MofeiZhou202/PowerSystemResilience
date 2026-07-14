#include "hacdcpf/market/market_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/problem_types.hpp"

namespace hacdcpf::market {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLargeBound = 1.0e9;

struct PeriodNetworkData {
  HybridPowerSystem snapshot;
  std::vector<double> gross_demand_by_bus_mw;
  std::vector<double> exogenous_injection_by_bus_mw;
  std::vector<double> net_demand_by_bus_mw;
};

struct N1SecurityCut {
  int period{0};
  int monitored_active_branch{0};
  int outage_active_branch{0};
  double lodf{0.0};
  double sense{1.0};
  double emergency_rating_mw{0.0};
};

struct LODFModel {
  bool available{false};
  Eigen::MatrixXd values;
  std::vector<int> branch_positions;
  std::vector<int> valid_outage_active_indices;
  std::vector<int> skipped_islanding_branch_positions;
  std::vector<std::string> warnings;
};

struct PricingBuild {
  engine::LPModel model;
  int G{0};
  int T{0};
  int B{0};
  int L{0};
  int K{0};
  int p_offset{0};
  int reserve_offset{0};
  int theta_offset{0};
  int flow_offset{0};
  int shed_offset{0};
  int curtail_offset{0};
  int segment_offset{0};
  int balance_row_offset{0};
  int flow_row_offset{0};
  int offer_row_offset{0};
  int reserve_row_offset{0};
  std::vector<int> generator_positions;
  std::vector<int> branch_positions;
  std::vector<double> reserve_requirements_mw;
  std::vector<PeriodNetworkData> periods;

  int p(int g, int t) const { return p_offset + g * T + t; }
  int reserve(int g, int t) const { return reserve_offset + g * T + t; }
  int theta(int b, int t) const { return theta_offset + t * B + b; }
  int flow(int l, int t) const { return flow_offset + l * T + t; }
  int shed(int b, int t) const { return shed_offset + t * B + b; }
  int curtail(int b, int t) const { return curtail_offset + t * B + b; }
  int segment(int g, int t, int k) const {
    return segment_offset + (g * T + t) * K + k;
  }
  int balance_row(int b, int t) const {
    return balance_row_offset + t * B + b;
  }
  int flow_row(int l, int t) const { return flow_row_offset + t * L + l; }
  int offer_row(int g, int t) const { return offer_row_offset + g * T + t; }
  int reserve_row(int t) const { return reserve_row_offset + t; }
};

struct MarketCommitmentBuild {
  engine::MIPModel model;
  int G{0};
  int T{0};
  int B{0};
  int L{0};
  int K{0};
  int p_offset{0};
  int commitment_offset{0};
  int startup_offset{0};
  int shutdown_offset{0};
  int reserve_offset{0};
  int segment_offset{0};
  int theta_offset{0};
  int flow_offset{0};
  int shed_offset{0};
  int curtail_offset{0};
  std::vector<int> generator_positions;
  std::vector<int> branch_positions;
  std::vector<PeriodNetworkData> periods;

  int p(int g, int t) const { return p_offset + g * T + t; }
  int commitment(int g, int t) const {
    return commitment_offset + g * T + t;
  }
  int startup(int g, int t) const { return startup_offset + g * T + t; }
  int shutdown(int g, int t) const { return shutdown_offset + g * T + t; }
  int reserve(int g, int t) const { return reserve_offset + g * T + t; }
  int segment(int g, int t, int k) const {
    return segment_offset + (g * T + t) * K + k;
  }
  int theta(int b, int t) const { return theta_offset + t * B + b; }
  int flow(int l, int t) const { return flow_offset + l * T + t; }
  int shed(int b, int t) const { return shed_offset + t * B + b; }
  int curtail(int b, int t) const { return curtail_offset + t * B + b; }
};

std::unordered_map<int, int> make_bus_position_map(
    const HybridPowerSystem& system) {
  std::unordered_map<int, int> out;
  out.reserve(system.ac.buses.size());
  for (int b = 0; b < static_cast<int>(system.ac.buses.size()); ++b) {
    const int id = system.ac.buses[static_cast<size_t>(b)].index;
    if (!out.emplace(id, b).second) {
      throw std::invalid_argument("market: duplicate AC bus id " +
                                  std::to_string(id));
    }
  }
  return out;
}

std::vector<int> active_generator_positions(const HybridPowerSystem& system) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(system.ac.generators.size()); ++i) {
    if (system.ac.generators[static_cast<size_t>(i)].in_service) out.push_back(i);
  }
  return out;
}

std::vector<int> active_branch_positions(const HybridPowerSystem& system) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(system.ac.branches.size()); ++i) {
    if (system.ac.branches[static_cast<size_t>(i)].in_service) out.push_back(i);
  }
  return out;
}

bool has_unsupported_hybrid_market_assets(const HybridPowerSystem& system) {
  const auto any_in_service = [](const auto& items) {
    return std::any_of(items.begin(), items.end(),
                       [](const auto& item) { return item.in_service; });
  };
  return any_in_service(system.dc.buses) || any_in_service(system.dc.branches) ||
         any_in_service(system.dc.loads) || any_in_service(system.dc.storage) ||
         any_in_service(system.vsc_converters) ||
         any_in_service(system.dc.dcdc_converters) ||
         any_in_service(system.ac.external_grids);
}

PeriodNetworkData make_period_data(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const UCSchedule& commitment,
    int period,
    const TimeSeriesPFOptions& uc_options,
    const std::unordered_map<int, int>& bus_positions) {
  TimeSeriesPFOptions snapshot_options = uc_options;
  snapshot_options.enable_uc_opf_tracking_band = false;
  PeriodNetworkData data;
  data.snapshot = build_time_series_system_snapshot(
      system, time_series, commitment, period, snapshot_options);
  const int B = static_cast<int>(system.ac.buses.size());
  data.gross_demand_by_bus_mw.assign(static_cast<size_t>(B), 0.0);
  data.exogenous_injection_by_bus_mw.assign(static_cast<size_t>(B), 0.0);

  for (int b = 0; b < B; ++b) {
    const auto& bus = data.snapshot.ac.buses[static_cast<size_t>(b)];
    if (bus.in_service) {
      data.gross_demand_by_bus_mw[static_cast<size_t>(b)] +=
          std::max(0.0, bus.pd_mw);
    }
  }
  const auto add_demand = [&](int bus, double mw) {
    const auto it = bus_positions.find(bus);
    if (it != bus_positions.end()) {
      data.gross_demand_by_bus_mw[static_cast<size_t>(it->second)] +=
          std::max(0.0, mw);
    }
  };
  const auto add_injection = [&](int bus, double mw) {
    const auto it = bus_positions.find(bus);
    if (it != bus_positions.end()) {
      data.exogenous_injection_by_bus_mw[static_cast<size_t>(it->second)] += mw;
    }
  };

  for (const auto& load : data.snapshot.ac.loads) {
    if (load.in_service) add_demand(load.bus, load.p_mw * load.scaling);
  }
  for (const auto& load : data.snapshot.ac.flexible_loads) {
    if (load.in_service) add_demand(load.bus, load.p_mw);
  }
  for (const auto& gen : data.snapshot.ac.static_generators) {
    if (gen.in_service) add_injection(gen.bus, gen.p_mw * gen.scaling);
  }
  for (const auto& gen : data.snapshot.ac.renewable_gens) {
    if (gen.in_service) add_injection(gen.bus, gen.p_mw);
  }
  for (const auto& pv : data.snapshot.ac.pv_systems) {
    if (pv.in_service) add_injection(pv.bus, pv.p_mw);
  }
  for (const auto& storage : data.snapshot.ac.storage) {
    if (storage.in_service) add_injection(storage.bus, storage.p_mw);
  }

  data.net_demand_by_bus_mw.resize(static_cast<size_t>(B), 0.0);
  for (int b = 0; b < B; ++b) {
    data.net_demand_by_bus_mw[static_cast<size_t>(b)] =
        data.gross_demand_by_bus_mw[static_cast<size_t>(b)] -
        data.exogenous_injection_by_bus_mw[static_cast<size_t>(b)];
  }
  return data;
}

engine::SolverAdapterPtr create_market_milp_adapter(UCSolverChoice choice) {
  using namespace engine;
  const auto native = []() -> SolverAdapterPtr {
    BCOptions options;
    options.cuts = CutType::Gomory;
    options.root_cut_rounds = 15;
    options.cuts_per_round = 20;
    options.use_simplex_lp_nodes = true;
    options.use_feasibility_pump = false;
    options.branching = BranchingStrategy::Pseudocost;
    options.node_sel = NodeSelection::Hybrid;
    options.gap_tol = 1e-4;
    options.max_lp_iter = 30000;
    options.verbose = false;
    return std::make_shared<NativeBranchAndCutAdapter>(options);
  };
  if (choice == UCSolverChoice::Native) return native();
  if (choice == UCSolverChoice::HiGHS) {
    return std::make_shared<HighsAdapter>();
  }
  if (choice == UCSolverChoice::SCIP) {
    auto adapter = std::make_shared<ScipAdapter>();
    if (adapter->available() && adapter->supports(ProblemClass::MILP)) {
      return adapter;
    }
    return native();
  }
  if (choice == UCSolverChoice::Gurobi) {
    return std::make_shared<GurobiAdapter>();
  }
  auto highs = std::make_shared<HighsAdapter>();
  if (highs->available() && highs->supports(ProblemClass::MILP)) return highs;
  return native();
}

MarketCommitmentBuild build_market_commitment_model(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options) {
  using Triplet = Eigen::Triplet<double>;
  using engine::VarType;
  MarketCommitmentBuild build;
  build.generator_positions = active_generator_positions(system);
  build.branch_positions = active_branch_positions(system);
  build.G = static_cast<int>(build.generator_positions.size());
  build.T = time_series.num_steps;
  build.B = static_cast<int>(system.ac.buses.size());
  build.L = static_cast<int>(build.branch_positions.size());
  build.K = std::max(1, options.energy_offer_segments);
  if (build.G == 0 || build.B == 0 || build.T <= 0 ||
      static_cast<int>(offers.size()) != build.G) {
    throw std::invalid_argument(
        "market: SCUC requires aligned active generators, offers, buses and periods");
  }

  const auto bus_positions = make_bus_position_map(system);
  UCSchedule no_commitment;
  for (int t = 0; t < build.T; ++t) {
    build.periods.push_back(make_period_data(
        system, time_series, no_commitment, t, options.uc_options,
        bus_positions));
  }

  const int nGT = build.G * build.T;
  build.p_offset = 0;
  build.commitment_offset = build.p_offset + nGT;
  build.startup_offset = build.commitment_offset + nGT;
  build.shutdown_offset = build.startup_offset + nGT;
  build.reserve_offset = build.shutdown_offset + nGT;
  build.segment_offset = build.reserve_offset + nGT;
  build.theta_offset = build.segment_offset + nGT * build.K;
  build.flow_offset = build.theta_offset + build.B * build.T;
  build.shed_offset = build.flow_offset + build.L * build.T;
  build.curtail_offset = build.shed_offset + build.B * build.T;
  const int nvar = build.curtail_offset + build.B * build.T;

  auto& model = build.model;
  model.linear_part.sense = engine::Sense::Minimize;
  model.linear_part.c = Eigen::VectorXd::Zero(nvar);
  model.linear_part.vars.resize(static_cast<size_t>(nvar));
  const double dt = time_series.step_duration_hr;

  for (int g = 0; g < build.G; ++g) {
    const int position = build.generator_positions[static_cast<size_t>(g)];
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    const auto& offer = offers[static_cast<size_t>(g)];
    if (offer.generator_position != position ||
        static_cast<int>(offer.energy_segments.size()) != build.K) {
      throw std::invalid_argument("market: SCUC offer order or segment count mismatch");
    }
    const double pmin = std::max(0.0, offer.minimum_output_mw);
    const double pmax = std::max(pmin, offer.offered_maximum_output_mw);
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.p(g, t))] = {
          VarType::Continuous, 0.0, pmax,
          "market_uc_p_g" + std::to_string(g) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.reserve(g, t))] = {
          VarType::Continuous, 0.0, std::max(0.0, pmax - pmin),
          "market_uc_r_g" + std::to_string(g) + "_t" + std::to_string(t)};
      double commitment_lo = 0.0;
      double commitment_hi = 1.0;
      if (options.uc_options.fixed_commitment_schedule &&
          g < static_cast<int>(
                  options.uc_options.fixed_commitment_schedule->size()) &&
          t < static_cast<int>((*options.uc_options.fixed_commitment_schedule)
                                   [static_cast<size_t>(g)]
                                       .size())) {
        const double fixed =
            (*options.uc_options.fixed_commitment_schedule)
                [static_cast<size_t>(g)][static_cast<size_t>(t)]
                ? 1.0
                : 0.0;
        commitment_lo = fixed;
        commitment_hi = fixed;
      } else if (options.uc_options.fix_commitment) {
        commitment_lo = 1.0;
      }
      model.linear_part.vars[static_cast<size_t>(build.commitment(g, t))] = {
          VarType::Binary, commitment_lo, commitment_hi,
          "market_uc_u_g" + std::to_string(g) + "_t" + std::to_string(t)};
      model.binary_idx.push_back(build.commitment(g, t));
      for (const auto variable : {build.startup(g, t), build.shutdown(g, t)}) {
        model.linear_part.vars[static_cast<size_t>(variable)] = {
            VarType::Binary, 0.0, 1.0,
            "market_uc_binary_" + std::to_string(variable)};
        model.binary_idx.push_back(variable);
      }
      model.linear_part.c[build.commitment(g, t)] =
          (std::max(0.0, offer.minimum_output_cost_per_hour) +
           std::max(0.0, offer.no_load_price_per_hour)) * dt;
      model.linear_part.c[build.startup(g, t)] =
          std::max(0.0, offer.startup_price);
      model.linear_part.c[build.shutdown(g, t)] =
          std::max(0.0, offer.shutdown_price);
      model.linear_part.c[build.reserve(g, t)] =
          std::max(0.0, offer.upward_reserve_price_per_mwh) * dt;
      for (int k = 0; k < build.K; ++k) {
        const auto& segment = offer.energy_segments[static_cast<size_t>(k)];
        const int variable = build.segment(g, t, k);
        model.linear_part.vars[static_cast<size_t>(variable)] = {
            VarType::Continuous, 0.0, std::max(0.0, segment.quantity_mw),
            "market_uc_seg_g" + std::to_string(g) + "_t" +
                std::to_string(t) + "_k" + std::to_string(k)};
        model.linear_part.c[variable] =
            std::max(0.0, segment.price_per_mwh) * dt;
      }
    }
  }

  int reference_bus = 0;
  for (int b = 0; b < build.B; ++b) {
    if (system.ac.buses[static_cast<size_t>(b)].bus_type == BusType::SLACK) {
      reference_bus = b;
      break;
    }
  }
  for (int t = 0; t < build.T; ++t) {
    for (int b = 0; b < build.B; ++b) {
      const double angle_bound = b == reference_bus ? 0.0 : kPi;
      model.linear_part.vars[static_cast<size_t>(build.theta(b, t))] = {
          VarType::Continuous, -angle_bound, angle_bound,
          "market_uc_theta_b" + std::to_string(b) + "_t" +
              std::to_string(t)};
      const auto& period = build.periods[static_cast<size_t>(t)];
      const double demand =
          std::max(0.0, period.gross_demand_by_bus_mw[static_cast<size_t>(b)]);
      const double injection = std::max(
          0.0, period.exogenous_injection_by_bus_mw[static_cast<size_t>(b)]);
      model.linear_part.vars[static_cast<size_t>(build.shed(b, t))] = {
          VarType::Continuous, 0.0, demand,
          "market_uc_shed_b" + std::to_string(b) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.curtail(b, t))] = {
          VarType::Continuous, 0.0, injection,
          "market_uc_curtail_b" + std::to_string(b) + "_t" +
              std::to_string(t)};
      model.linear_part.c[build.shed(b, t)] =
          std::max(1.0, options.value_of_lost_load_per_mwh) * dt;
      model.linear_part.c[build.curtail(b, t)] =
          std::max(0.0, options.exogenous_curtailment_penalty_per_mwh) * dt;
    }
  }
  for (int l = 0; l < build.L; ++l) {
    const auto& branch = system.ac.branches[
        static_cast<size_t>(build.branch_positions[static_cast<size_t>(l)])];
    const double limit = options.enable_network_constraints && branch.rate_a_mva > 1e-9
        ? branch.rate_a_mva
        : kLargeBound;
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.flow(l, t))] = {
          VarType::Continuous, -limit, limit,
          "market_uc_flow_l" + std::to_string(l) + "_t" +
              std::to_string(t)};
    }
  }

  std::vector<Triplet> equality;
  std::vector<double> equality_rhs;
  const auto add_eq = [&](const std::vector<std::pair<int, double>>& terms,
                          double bound) {
    const int row = static_cast<int>(equality_rhs.size());
    for (const auto& [column, value] : terms) {
      equality.emplace_back(row, column, value);
    }
    equality_rhs.push_back(bound);
  };
  std::vector<Triplet> inequality;
  std::vector<double> inequality_rhs;
  const auto add_le = [&](const std::vector<std::pair<int, double>>& terms,
                          double bound) {
    const int row = static_cast<int>(inequality_rhs.size());
    for (const auto& [column, value] : terms) {
      inequality.emplace_back(row, column, value);
    }
    inequality_rhs.push_back(bound);
  };

  for (int t = 0; t < build.T; ++t) {
    for (int b = 0; b < build.B; ++b) {
      std::vector<std::pair<int, double>> terms = {
          {build.shed(b, t), 1.0}, {build.curtail(b, t), -1.0}};
      for (int g = 0; g < build.G; ++g) {
        const auto& generator = system.ac.generators[static_cast<size_t>(
            build.generator_positions[static_cast<size_t>(g)])];
        if (bus_positions.at(generator.bus) == b) {
          terms.emplace_back(build.p(g, t), 1.0);
        }
      }
      for (int l = 0; l < build.L; ++l) {
        const auto& branch = system.ac.branches[static_cast<size_t>(
            build.branch_positions[static_cast<size_t>(l)])];
        if (bus_positions.at(branch.from_bus) == b) {
          terms.emplace_back(build.flow(l, t), -1.0);
        }
        if (bus_positions.at(branch.to_bus) == b) {
          terms.emplace_back(build.flow(l, t), 1.0);
        }
      }
      add_eq(terms, build.periods[static_cast<size_t>(t)]
                        .net_demand_by_bus_mw[static_cast<size_t>(b)]);
    }
  }

  const double base_mva = std::max({system.base_mva, system.ac.base_mva, 1.0});
  for (int l = 0; l < build.L; ++l) {
    const auto& branch = system.ac.branches[static_cast<size_t>(
        build.branch_positions[static_cast<size_t>(l)])];
    double x = branch.x_pu;
    if (std::abs(x) < 1e-12) x = (x < 0.0 ? -1.0 : 1.0) * 1e-6;
    const double tap = std::abs(branch.tap) > 1e-12 ? branch.tap : 1.0;
    const double coefficient = base_mva / (x * tap);
    const double shift = branch.shift_deg * kPi / 180.0;
    const int from = bus_positions.at(branch.from_bus);
    const int to = bus_positions.at(branch.to_bus);
    for (int t = 0; t < build.T; ++t) {
      add_eq({{build.flow(l, t), 1.0},
              {build.theta(from, t), -coefficient},
              {build.theta(to, t), coefficient}},
             -coefficient * shift);
    }
  }

  for (int g = 0; g < build.G; ++g) {
    const auto& generator = system.ac.generators[static_cast<size_t>(
        build.generator_positions[static_cast<size_t>(g)])];
    const auto& offer = offers[static_cast<size_t>(g)];
    const double pmin = std::max(0.0, offer.minimum_output_mw);
    const double pmax = std::max(pmin, offer.offered_maximum_output_mw);
    const double initial_dispatch = std::clamp(generator.pg_mw, 0.0, pmax);
    const double initial_commitment = generator.pg_mw > 1e-6 ? 1.0 : 0.0;
    const double ramp_up = generator.ramp_up_mw_min > 0.0
        ? generator.ramp_up_mw_min * 60.0 * dt
        : kLargeBound;
    const double ramp_down = generator.ramp_dn_mw_min > 0.0
        ? generator.ramp_dn_mw_min * 60.0 * dt
        : kLargeBound;
    for (int t = 0; t < build.T; ++t) {
      std::vector<std::pair<int, double>> output = {
          {build.p(g, t), 1.0}, {build.commitment(g, t), -pmin}};
      for (int k = 0; k < build.K; ++k) {
        output.emplace_back(build.segment(g, t, k), -1.0);
        add_le({{build.segment(g, t, k), 1.0},
                {build.commitment(g, t),
                 -std::max(0.0, offer.energy_segments[static_cast<size_t>(k)]
                                          .quantity_mw)}},
               0.0);
      }
      add_eq(output, 0.0);
      add_le({{build.p(g, t), 1.0}, {build.reserve(g, t), 1.0},
              {build.commitment(g, t), -pmax}},
             0.0);
      if (generator.ramp_up_mw_min > 0.0) {
        if (t == 0) {
          add_le({{build.p(g, t), 1.0}, {build.reserve(g, t), 1.0}},
                 initial_dispatch + ramp_up);
        } else {
          add_le({{build.p(g, t), 1.0}, {build.reserve(g, t), 1.0},
                  {build.p(g, t - 1), -1.0}},
                 ramp_up);
        }
      }
      if (generator.ramp_dn_mw_min > 0.0) {
        if (t == 0) {
          add_le({{build.p(g, t), -1.0}}, ramp_down - initial_dispatch);
        } else {
          add_le({{build.p(g, t - 1), 1.0}, {build.p(g, t), -1.0}},
                 ramp_down);
        }
      }
      std::vector<std::pair<int, double>> transition = {
          {build.commitment(g, t), 1.0}, {build.startup(g, t), -1.0},
          {build.shutdown(g, t), 1.0}};
      double transition_rhs = initial_commitment;
      if (t > 0) {
        transition.emplace_back(build.commitment(g, t - 1), -1.0);
        transition_rhs = 0.0;
      }
      add_eq(transition, transition_rhs);
      add_le({{build.startup(g, t), 1.0}, {build.shutdown(g, t), 1.0}}, 1.0);
    }

    const int minimum_up = static_cast<int>(
        std::ceil(std::max(0.0, generator.min_up_time_hr) / dt));
    const int minimum_down = static_cast<int>(
        std::ceil(std::max(0.0, generator.min_dn_time_hr) / dt));
    for (int t = 0; t < build.T; ++t) {
      if (minimum_up > 0) {
        std::vector<std::pair<int, double>> terms = {
            {build.commitment(g, t), -1.0}};
        for (int tau = std::max(0, t - minimum_up + 1); tau <= t; ++tau) {
          terms.emplace_back(build.startup(g, tau), 1.0);
        }
        add_le(terms, 0.0);
      }
      if (minimum_down > 0) {
        std::vector<std::pair<int, double>> terms = {
            {build.commitment(g, t), 1.0}};
        for (int tau = std::max(0, t - minimum_down + 1); tau <= t; ++tau) {
          terms.emplace_back(build.shutdown(g, tau), 1.0);
        }
        add_le(terms, 1.0);
      }
    }

    const int periods_per_day = std::max(1, static_cast<int>(std::lround(24.0 / dt)));
    for (int day_start = 0; day_start < build.T; day_start += periods_per_day) {
      const int day_end = std::min(build.T, day_start + periods_per_day);
      if (generator.max_startups_per_day > 0) {
        std::vector<std::pair<int, double>> terms;
        for (int t = day_start; t < day_end; ++t) {
          terms.emplace_back(build.startup(g, t), 1.0);
        }
        add_le(terms, static_cast<double>(generator.max_startups_per_day));
      }
      if (generator.max_shutdowns_per_day > 0) {
        std::vector<std::pair<int, double>> terms;
        for (int t = day_start; t < day_end; ++t) {
          terms.emplace_back(build.shutdown(g, t), 1.0);
        }
        add_le(terms, static_cast<double>(generator.max_shutdowns_per_day));
      }
    }
  }

  for (int t = 0; t < build.T; ++t) {
    std::vector<std::pair<int, double>> reserve_terms;
    for (int g = 0; g < build.G; ++g) {
      reserve_terms.emplace_back(build.reserve(g, t), -1.0);
    }
    const double gross_demand = std::accumulate(
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.begin(),
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.end(), 0.0);
    add_le(reserve_terms,
           -std::max(0.0, options.upward_reserve_fraction) * gross_demand);
  }

  model.linear_part.Aeq.resize(static_cast<int>(equality_rhs.size()), nvar);
  model.linear_part.Aeq.setFromTriplets(equality.begin(), equality.end());
  model.linear_part.Aeq.makeCompressed();
  model.linear_part.beq.resize(static_cast<int>(equality_rhs.size()));
  for (int row = 0; row < static_cast<int>(equality_rhs.size()); ++row) {
    model.linear_part.beq[row] = equality_rhs[static_cast<size_t>(row)];
  }
  model.linear_part.A.resize(static_cast<int>(inequality_rhs.size()), nvar);
  model.linear_part.A.setFromTriplets(inequality.begin(), inequality.end());
  model.linear_part.A.makeCompressed();
  model.linear_part.b.resize(static_cast<int>(inequality_rhs.size()));
  for (int row = 0; row < static_cast<int>(inequality_rhs.size()); ++row) {
    model.linear_part.b[row] = inequality_rhs[static_cast<size_t>(row)];
  }
  return build;
}

UCSchedule solve_market_commitment(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options) {
  const auto build = build_market_commitment_model(
      system, time_series, offers, options);
  auto adapter = create_market_milp_adapter(options.uc_options.uc_solver);
  const auto solved = adapter->solve_milp(build.model);
  UCSchedule schedule;
  schedule.solver_name = solved.stats.solver_name;
  schedule.feasible = solved.stats.success &&
      solved.x.size() >= build.model.linear_part.c.size();
  if (!schedule.feasible) return schedule;
  schedule.total_cost = solved.stats.objective;
  schedule.gen_dispatch.assign(
      static_cast<size_t>(build.G),
      std::vector<double>(static_cast<size_t>(build.T), 0.0));
  schedule.gen_commit.assign(
      static_cast<size_t>(build.G),
      std::vector<int>(static_cast<size_t>(build.T), 0));
  for (int g = 0; g < build.G; ++g) {
    for (int t = 0; t < build.T; ++t) {
      schedule.gen_dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)] =
          solved.x[build.p(g, t)];
      schedule.gen_commit[static_cast<size_t>(g)][static_cast<size_t>(t)] =
          solved.x[build.commitment(g, t)] > 0.5 ? 1 : 0;
    }
  }
  return schedule;
}

PricingBuild build_pricing_model(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const UCSchedule& commitment,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options,
    const std::vector<N1SecurityCut>& security_cuts = {}) {
  using Triplet = Eigen::Triplet<double>;
  PricingBuild build;
  build.generator_positions = active_generator_positions(system);
  build.branch_positions = active_branch_positions(system);
  build.G = static_cast<int>(build.generator_positions.size());
  build.T = time_series.num_steps;
  build.B = static_cast<int>(system.ac.buses.size());
  build.L = static_cast<int>(build.branch_positions.size());
  build.K = std::max(1, options.energy_offer_segments);

  if (build.G == 0 || build.B == 0 || build.T <= 0) {
    throw std::invalid_argument("market: SCED requires AC buses, generators and periods");
  }
  if (static_cast<int>(commitment.gen_commit.size()) != build.G) {
    throw std::invalid_argument("market: SCUC commitment rows do not align with active generators");
  }
  if (static_cast<int>(offers.size()) != build.G) {
    throw std::invalid_argument("market: cost offer rows do not align with active generators");
  }

  const auto bus_positions = make_bus_position_map(system);
  build.periods.reserve(static_cast<size_t>(build.T));
  build.reserve_requirements_mw.resize(static_cast<size_t>(build.T), 0.0);
  for (int t = 0; t < build.T; ++t) {
    build.periods.push_back(make_period_data(
        system, time_series, commitment, t, options.uc_options, bus_positions));
    const double demand = std::accumulate(
        build.periods.back().gross_demand_by_bus_mw.begin(),
        build.periods.back().gross_demand_by_bus_mw.end(), 0.0);
    build.reserve_requirements_mw[static_cast<size_t>(t)] =
        std::max(0.0, options.upward_reserve_fraction) * demand;
  }

  const int nP = build.G * build.T;
  const int nR = build.G * build.T;
  const int nTheta = build.B * build.T;
  const int nFlow = build.L * build.T;
  const int nShed = build.B * build.T;
  const int nCurtail = build.B * build.T;
  const int nSegment = build.G * build.T * build.K;
  build.p_offset = 0;
  build.reserve_offset = build.p_offset + nP;
  build.theta_offset = build.reserve_offset + nR;
  build.flow_offset = build.theta_offset + nTheta;
  build.shed_offset = build.flow_offset + nFlow;
  build.curtail_offset = build.shed_offset + nShed;
  build.segment_offset = build.curtail_offset + nCurtail;
  const int nvar = build.segment_offset + nSegment;

  build.model.sense = engine::Sense::Minimize;
  build.model.vars.resize(static_cast<size_t>(nvar));
  build.model.c = Eigen::VectorXd::Zero(nvar);
  const double dt = time_series.step_duration_hr;

  for (int g = 0; g < build.G; ++g) {
    const auto& generator =
        system.ac.generators[static_cast<size_t>(build.generator_positions[static_cast<size_t>(g)])];
    const auto& offer = offers[static_cast<size_t>(g)];
    if (static_cast<int>(offer.energy_segments.size()) != build.K) {
      throw std::invalid_argument("market: energy offer segment count mismatch");
    }
    const double pmin = std::max(0.0, offer.minimum_output_mw);
    const double offered_quantity = std::accumulate(
        offer.energy_segments.begin(), offer.energy_segments.end(), 0.0,
        [](double total, const OfferSegment& segment) {
          return total + std::max(0.0, segment.quantity_mw);
        });
    const double pmax = std::min(
        std::max(pmin, generator.pmax_mw), pmin + offered_quantity);
    for (int t = 0; t < build.T; ++t) {
      const bool online = commitment.gen_commit[static_cast<size_t>(g)][static_cast<size_t>(t)] != 0;
      double dispatch_lo = online ? pmin : 0.0;
      double dispatch_hi = online ? pmax : 0.0;
      const auto schedule_value = [&](const auto* schedule) {
        if (!schedule ||
            build.generator_positions[static_cast<size_t>(g)] >=
                static_cast<int>(schedule->size())) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        const auto& row = (*schedule)[static_cast<size_t>(
            build.generator_positions[static_cast<size_t>(g)])];
        return t < static_cast<int>(row.size())
            ? row[static_cast<size_t>(t)]
            : std::numeric_limits<double>::quiet_NaN();
      };
      const double minimum_dispatch = schedule_value(
          options.minimum_dispatch_schedule_mw);
      if (std::isfinite(minimum_dispatch)) {
        dispatch_lo = std::max(dispatch_lo, minimum_dispatch);
      }
      const double fixed_dispatch = schedule_value(
          options.fixed_dispatch_schedule_mw);
      if (std::isfinite(fixed_dispatch)) {
        dispatch_lo = fixed_dispatch;
        dispatch_hi = fixed_dispatch;
      }
      if (dispatch_lo < -1e-8 || dispatch_lo > dispatch_hi + 1e-8 ||
          dispatch_hi > pmax + 1e-8) {
        throw std::invalid_argument(
            "market: real-time dispatch instruction violates commitment or offered capacity");
      }
      build.model.vars[static_cast<size_t>(build.p(g, t))] = {
          engine::VarType::Continuous, dispatch_lo, dispatch_hi,
          "market_p_g" + std::to_string(g) + "_t" + std::to_string(t)};
      double reserve_ub = online ? std::max(0.0, pmax - pmin) : 0.0;
      if (generator.ramp_up_mw_min > 0.0) {
        reserve_ub = std::min(
            reserve_ub, generator.ramp_up_mw_min * 60.0 * dt);
      }
      build.model.vars[static_cast<size_t>(build.reserve(g, t))] = {
          engine::VarType::Continuous, 0.0, reserve_ub,
          "market_rup_g" + std::to_string(g) + "_t" + std::to_string(t)};
      build.model.c[build.reserve(g, t)] =
          std::max(0.0, offers[static_cast<size_t>(g)].upward_reserve_price_per_mwh) * dt;
      for (int k = 0; k < build.K; ++k) {
        const auto& segment = offer.energy_segments[static_cast<size_t>(k)];
        const double segment_width = std::max(0.0, segment.quantity_mw);
        const int v = build.segment(g, t, k);
        build.model.vars[static_cast<size_t>(v)] = {
            engine::VarType::Continuous, 0.0, online ? segment_width : 0.0,
            "market_seg_g" + std::to_string(g) + "_t" + std::to_string(t) +
                "_k" + std::to_string(k)};
        build.model.c[v] = segment.price_per_mwh * dt;
      }
    }
  }

  int reference_bus = 0;
  for (int b = 0; b < build.B; ++b) {
    if (system.ac.buses[static_cast<size_t>(b)].bus_type == BusType::SLACK) {
      reference_bus = b;
      break;
    }
  }
  for (int t = 0; t < build.T; ++t) {
    for (int b = 0; b < build.B; ++b) {
      const double bound = (b == reference_bus) ? 0.0 : kPi;
      build.model.vars[static_cast<size_t>(build.theta(b, t))] = {
          engine::VarType::Continuous, -bound, bound,
          "market_theta_b" + std::to_string(b) + "_t" + std::to_string(t)};
      const double gross =
          build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw[static_cast<size_t>(b)];
      build.model.vars[static_cast<size_t>(build.shed(b, t))] = {
          engine::VarType::Continuous, 0.0, std::max(0.0, gross),
          "market_shed_b" + std::to_string(b) + "_t" + std::to_string(t)};
      build.model.c[build.shed(b, t)] =
          std::max(1.0, options.value_of_lost_load_per_mwh) * dt;
      const double exogenous = build.periods[static_cast<size_t>(t)]
          .exogenous_injection_by_bus_mw[static_cast<size_t>(b)];
      build.model.vars[static_cast<size_t>(build.curtail(b, t))] = {
          engine::VarType::Continuous, 0.0, std::max(0.0, exogenous),
          "market_curtail_b" + std::to_string(b) + "_t" +
              std::to_string(t)};
      build.model.c[build.curtail(b, t)] =
          std::max(0.0, options.exogenous_curtailment_penalty_per_mwh) * dt;
    }
  }
  for (int l = 0; l < build.L; ++l) {
    const auto& branch =
        system.ac.branches[static_cast<size_t>(build.branch_positions[static_cast<size_t>(l)])];
    const double rate = options.enable_network_constraints && branch.rate_a_mva > 1e-9
                            ? branch.rate_a_mva
                            : kLargeBound;
    for (int t = 0; t < build.T; ++t) {
      build.model.vars[static_cast<size_t>(build.flow(l, t))] = {
          engine::VarType::Continuous, -rate, rate,
          "market_flow_l" + std::to_string(l) + "_t" + std::to_string(t)};
    }
  }

  build.balance_row_offset = 0;
  build.flow_row_offset = build.balance_row_offset + build.B * build.T;
  build.offer_row_offset = build.flow_row_offset + build.L * build.T;
  build.reserve_row_offset = build.offer_row_offset + build.G * build.T;
  const int neq = build.reserve_row_offset + build.T;
  std::vector<Triplet> eq;
  eq.reserve(static_cast<size_t>(neq) * 6U);
  build.model.beq = Eigen::VectorXd::Zero(neq);

  for (int t = 0; t < build.T; ++t) {
    for (int b = 0; b < build.B; ++b) {
      const int row = build.balance_row(b, t);
      build.model.beq[row] =
          build.periods[static_cast<size_t>(t)].net_demand_by_bus_mw[static_cast<size_t>(b)];
      eq.emplace_back(row, build.shed(b, t), 1.0);
      eq.emplace_back(row, build.curtail(b, t), -1.0);
    }
    for (int g = 0; g < build.G; ++g) {
      const auto& generator = system.ac.generators[
          static_cast<size_t>(build.generator_positions[static_cast<size_t>(g)])];
      const auto bus_it = bus_positions.find(generator.bus);
      if (bus_it == bus_positions.end()) {
        throw std::invalid_argument("market: generator references an unknown AC bus");
      }
      eq.emplace_back(build.balance_row(bus_it->second, t), build.p(g, t), 1.0);

      const int offer_row = build.offer_row(g, t);
      eq.emplace_back(offer_row, build.p(g, t), 1.0);
      for (int k = 0; k < build.K; ++k) {
        eq.emplace_back(offer_row, build.segment(g, t, k), -1.0);
      }
      const bool online = commitment.gen_commit[static_cast<size_t>(g)][static_cast<size_t>(t)] != 0;
      build.model.beq[offer_row] = online ? std::max(0.0, generator.pmin_mw) : 0.0;
      eq.emplace_back(build.reserve_row(t), build.reserve(g, t), 1.0);
    }
    build.model.beq[build.reserve_row(t)] =
        build.reserve_requirements_mw[static_cast<size_t>(t)];
  }

  const double base_mva = std::max({system.base_mva, system.ac.base_mva, 1.0});
  for (int l = 0; l < build.L; ++l) {
    const auto& branch = system.ac.branches[
        static_cast<size_t>(build.branch_positions[static_cast<size_t>(l)])];
    const auto from_it = bus_positions.find(branch.from_bus);
    const auto to_it = bus_positions.find(branch.to_bus);
    if (from_it == bus_positions.end() || to_it == bus_positions.end()) {
      throw std::invalid_argument("market: branch references an unknown AC bus");
    }
    double x = branch.x_pu;
    if (std::abs(x) < 1e-12) x = (x < 0.0 ? -1.0 : 1.0) * 1e-6;
    const double tap = std::abs(branch.tap) > 1e-12 ? branch.tap : 1.0;
    const double coefficient = base_mva / (x * tap);
    const double shift = branch.shift_deg * kPi / 180.0;
    for (int t = 0; t < build.T; ++t) {
      const int flow_var = build.flow(l, t);
      eq.emplace_back(build.balance_row(from_it->second, t), flow_var, -1.0);
      eq.emplace_back(build.balance_row(to_it->second, t), flow_var, 1.0);
      const int row = build.flow_row(l, t);
      eq.emplace_back(row, flow_var, 1.0);
      eq.emplace_back(row, build.theta(from_it->second, t), -coefficient);
      eq.emplace_back(row, build.theta(to_it->second, t), coefficient);
      build.model.beq[row] = -coefficient * shift;
    }
  }

  build.model.Aeq.resize(neq, nvar);
  build.model.Aeq.setFromTriplets(eq.begin(), eq.end());
  build.model.Aeq.makeCompressed();

  std::vector<Triplet> ineq;
  std::vector<double> rhs;
  const auto add_le = [&](std::initializer_list<std::pair<int, double>> terms,
                          double bound) {
    const int row = static_cast<int>(rhs.size());
    for (const auto& [column, value] : terms) ineq.emplace_back(row, column, value);
    rhs.push_back(bound);
  };

  // Energy plus upward reserve cannot exceed committed capacity.
  for (int g = 0; g < build.G; ++g) {
    const auto& generator = system.ac.generators[
        static_cast<size_t>(build.generator_positions[static_cast<size_t>(g)])];
    const auto& offer = offers[static_cast<size_t>(g)];
    const double submitted_capacity = std::min(
        generator.pmax_mw,
        std::max(offer.minimum_output_mw, offer.offered_maximum_output_mw));
    for (int t = 0; t < build.T; ++t) {
      const bool online = commitment.gen_commit[static_cast<size_t>(g)][static_cast<size_t>(t)] != 0;
      add_le({{build.p(g, t), 1.0}, {build.reserve(g, t), 1.0}},
             online ? submitted_capacity : 0.0);
    }
    const double initial_dispatch = generator.in_service
        ? std::max(0.0, generator.pg_mw)
        : 0.0;
    if (build.T > 0 && generator.ramp_up_mw_min > 0.0) {
      add_le({{build.p(g, 0), 1.0}, {build.reserve(g, 0), 1.0}},
             initial_dispatch + generator.ramp_up_mw_min * 60.0 * dt);
    }
    if (build.T > 0 && generator.ramp_dn_mw_min > 0.0) {
      add_le({{build.p(g, 0), -1.0}},
             generator.ramp_dn_mw_min * 60.0 * dt - initial_dispatch);
    }
    for (int t = 1; t < build.T; ++t) {
      if (generator.ramp_up_mw_min > 0.0) {
        add_le({{build.p(g, t), 1.0},
                {build.reserve(g, t), 1.0},
                {build.p(g, t - 1), -1.0}},
               generator.ramp_up_mw_min * 60.0 * dt);
      }
      if (generator.ramp_dn_mw_min > 0.0) {
        add_le({{build.p(g, t - 1), 1.0}, {build.p(g, t), -1.0}},
               generator.ramp_dn_mw_min * 60.0 * dt);
      }
    }
  }
  for (const auto& cut : security_cuts) {
    if (cut.period < 0 || cut.period >= build.T ||
        cut.monitored_active_branch < 0 ||
        cut.monitored_active_branch >= build.L ||
        cut.outage_active_branch < 0 ||
        cut.outage_active_branch >= build.L) {
      throw std::invalid_argument("market: invalid N-1 security cut index");
    }
    add_le({{build.flow(cut.monitored_active_branch, cut.period), cut.sense},
            {build.flow(cut.outage_active_branch, cut.period),
             cut.sense * cut.lodf}},
           cut.emergency_rating_mw);
  }

  build.model.A.resize(static_cast<int>(rhs.size()), nvar);
  build.model.A.setFromTriplets(ineq.begin(), ineq.end());
  build.model.A.makeCompressed();
  build.model.b.resize(static_cast<int>(rhs.size()));
  for (int i = 0; i < static_cast<int>(rhs.size()); ++i) build.model.b[i] = rhs[static_cast<size_t>(i)];
  return build;
}

LODFModel build_lodf_model(const HybridPowerSystem& system) {
  LODFModel out;
  out.branch_positions = active_branch_positions(system);
  const int B = static_cast<int>(system.ac.buses.size());
  const int L = static_cast<int>(out.branch_positions.size());
  out.values = Eigen::MatrixXd::Zero(L, L);
  if (B < 2 || L == 0) {
    out.warnings.push_back("N-1 LODF requires at least two AC buses and one branch.");
    return out;
  }
  const auto bus_positions = make_bus_position_map(system);
  int reference_bus = 0;
  for (int b = 0; b < B; ++b) {
    if (system.ac.buses[static_cast<size_t>(b)].bus_type == BusType::SLACK) {
      reference_bus = b;
      break;
    }
  }
  const double base_mva = std::max({system.base_mva, system.ac.base_mva, 1.0});
  Eigen::MatrixXd incidence = Eigen::MatrixXd::Zero(L, B);
  Eigen::VectorXd susceptance = Eigen::VectorXd::Zero(L);
  bool has_phase_shift = false;
  for (int l = 0; l < L; ++l) {
    const auto& branch =
        system.ac.branches[static_cast<size_t>(out.branch_positions[static_cast<size_t>(l)])];
    const auto from = bus_positions.find(branch.from_bus);
    const auto to = bus_positions.find(branch.to_bus);
    if (from == bus_positions.end() || to == bus_positions.end()) {
      out.warnings.push_back("N-1 LODF branch references an unknown bus.");
      return out;
    }
    double x = branch.x_pu;
    if (std::abs(x) < 1e-12) x = (x < 0.0 ? -1.0 : 1.0) * 1e-6;
    const double tap = std::abs(branch.tap) > 1e-12 ? branch.tap : 1.0;
    susceptance[l] = base_mva / (x * tap);
    incidence(l, from->second) = 1.0;
    incidence(l, to->second) = -1.0;
    has_phase_shift = has_phase_shift || std::abs(branch.shift_deg) > 1e-10;
  }
  if (has_phase_shift) {
    out.warnings.push_back(
        "LODF sensitivities ignore fixed phase-shift injections; base flows still retain them.");
  }
  const Eigen::MatrixXd bbus =
      incidence.transpose() * susceptance.asDiagonal() * incidence;
  Eigen::MatrixXd reduced(B - 1, B - 1);
  for (int i = 0, ri = 0; i < B; ++i) {
    if (i == reference_bus) continue;
    for (int j = 0, rj = 0; j < B; ++j) {
      if (j == reference_bus) continue;
      reduced(ri, rj++) = bbus(i, j);
    }
    ++ri;
  }
  Eigen::FullPivLU<Eigen::MatrixXd> factor(reduced);
  if (!factor.isInvertible()) {
    out.warnings.push_back("Base AC network is disconnected; LODF matrix is unavailable.");
    return out;
  }

  for (int outage = 0; outage < L; ++outage) {
    Eigen::VectorXd transfer = incidence.row(outage).transpose();
    Eigen::VectorXd reduced_transfer(B - 1);
    for (int i = 0, r = 0; i < B; ++i) {
      if (i == reference_bus) continue;
      reduced_transfer[r++] = transfer[i];
    }
    const Eigen::VectorXd reduced_theta = factor.solve(reduced_transfer);
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(B);
    for (int i = 0, r = 0; i < B; ++i) {
      if (i == reference_bus) continue;
      theta[i] = reduced_theta[r++];
    }
    const Eigen::VectorXd ptdf =
        susceptance.asDiagonal() * incidence * theta;
    const double denominator = 1.0 - ptdf[outage];
    if (std::abs(denominator) < 1e-7 || !std::isfinite(denominator)) {
      out.skipped_islanding_branch_positions.push_back(
          out.branch_positions[static_cast<size_t>(outage)]);
      continue;
    }
    out.values.col(outage) = ptdf / denominator;
    out.values(outage, outage) = -1.0;
    out.valid_outage_active_indices.push_back(outage);
  }
  out.available = !out.valid_outage_active_indices.empty();
  if (!out.available) {
    out.warnings.push_back("Every candidate branch outage islands the network.");
  }
  return out;
}

std::vector<int> select_n1_contingencies(
    const LODFModel& lodf,
    const std::vector<PricingPeriod>& pricing,
    int maximum) {
  std::vector<int> candidates = lodf.valid_outage_active_indices;
  std::stable_sort(candidates.begin(), candidates.end(), [&](int lhs, int rhs) {
    const int lhs_position = lodf.branch_positions[static_cast<size_t>(lhs)];
    const int rhs_position = lodf.branch_positions[static_cast<size_t>(rhs)];
    double lhs_flow = 0.0;
    double rhs_flow = 0.0;
    for (const auto& period : pricing) {
      if (lhs_position < static_cast<int>(period.branch_flow_mw.size())) {
        lhs_flow = std::max(
            lhs_flow, std::abs(period.branch_flow_mw[static_cast<size_t>(lhs_position)]));
      }
      if (rhs_position < static_cast<int>(period.branch_flow_mw.size())) {
        rhs_flow = std::max(
            rhs_flow, std::abs(period.branch_flow_mw[static_cast<size_t>(rhs_position)]));
      }
    }
    return lhs_flow > rhs_flow;
  });
  if (maximum > 0 && static_cast<int>(candidates.size()) > maximum) {
    candidates.resize(static_cast<size_t>(maximum));
  }
  return candidates;
}

struct N1Screen {
  std::vector<N1Violation> violations;
  double worst_overload_mw{0.0};
};

N1Screen screen_n1(
    const HybridPowerSystem& system,
    const LODFModel& lodf,
    const std::vector<int>& contingencies,
    const std::vector<PricingPeriod>& pricing,
    const MarketOptions& options) {
  N1Screen screen;
  const double rating_multiplier =
      std::max(0.0, options.n1_emergency_rating_multiplier);
  for (int t = 0; t < static_cast<int>(pricing.size()); ++t) {
    const auto& period = pricing[static_cast<size_t>(t)];
    for (int outage : contingencies) {
      const int outage_position =
          lodf.branch_positions[static_cast<size_t>(outage)];
      const double outage_flow =
          period.branch_flow_mw[static_cast<size_t>(outage_position)];
      for (int monitored = 0;
           monitored < static_cast<int>(lodf.branch_positions.size()); ++monitored) {
        if (monitored == outage) continue;
        const int monitored_position =
            lodf.branch_positions[static_cast<size_t>(monitored)];
        const auto& branch =
            system.ac.branches[static_cast<size_t>(monitored_position)];
        if (branch.rate_a_mva <= 1e-9) continue;
        const double base_flow =
            period.branch_flow_mw[static_cast<size_t>(monitored_position)];
        const double post_flow =
            base_flow + lodf.values(monitored, outage) * outage_flow;
        const double rating = rating_multiplier * branch.rate_a_mva;
        const double overload = std::abs(post_flow) - rating;
        if (overload <= options.n1_violation_tolerance_mw) continue;
        screen.worst_overload_mw = std::max(screen.worst_overload_mw, overload);
        screen.violations.push_back(N1Violation{
            t,
            outage_position,
            system.ac.branches[static_cast<size_t>(outage_position)].index,
            monitored_position,
            branch.index,
            base_flow,
            post_flow,
            rating,
            overload});
      }
    }
  }
  std::stable_sort(
      screen.violations.begin(), screen.violations.end(),
      [](const N1Violation& lhs, const N1Violation& rhs) {
        return lhs.overload_mw > rhs.overload_mw;
      });
  return screen;
}

std::vector<PricingPeriod> extract_pricing(
    const HybridPowerSystem& system,
    const PricingBuild& build,
    const engine::SolveResult& solve,
    double dt) {
  std::vector<PricingPeriod> out(static_cast<size_t>(build.T));
  const int inequality_rows = build.model.A.rows();
  const int equality_rows = build.model.Aeq.rows();
  const bool have_duals =
      solve.constraint_duals.size() >= inequality_rows + equality_rows;
  for (int t = 0; t < build.T; ++t) {
    auto& period = out[static_cast<size_t>(t)];
    period.converged = solve.stats.success;
    period.status = solve.stats.status;
    period.generator_dispatch_mw.assign(system.ac.generators.size(), 0.0);
    period.upward_reserve_mw.assign(system.ac.generators.size(), 0.0);
    period.lmp_per_mwh.assign(system.ac.buses.size(), 0.0);
    period.branch_flow_mw.assign(system.ac.branches.size(), 0.0);
    period.load_shedding_mw.assign(system.ac.buses.size(), 0.0);
    period.exogenous_curtailment_mw.assign(system.ac.buses.size(), 0.0);
    period.reserve_requirement_mw =
        build.reserve_requirements_mw[static_cast<size_t>(t)];
    period.gross_demand_mw = std::accumulate(
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.begin(),
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.end(), 0.0);
    if (!solve.stats.success || solve.x.size() < build.model.c.size()) continue;

    for (int g = 0; g < build.G; ++g) {
      const int position = build.generator_positions[static_cast<size_t>(g)];
      period.generator_dispatch_mw[static_cast<size_t>(position)] = solve.x[build.p(g, t)];
      period.upward_reserve_mw[static_cast<size_t>(position)] = solve.x[build.reserve(g, t)];
      period.objective += solve.x[build.reserve(g, t)] *
                          build.model.c[build.reserve(g, t)];
      for (int k = 0; k < build.K; ++k) {
        const int v = build.segment(g, t, k);
        period.objective += solve.x[v] * build.model.c[v];
      }
    }
    for (int l = 0; l < build.L; ++l) {
      const int position = build.branch_positions[static_cast<size_t>(l)];
      period.branch_flow_mw[static_cast<size_t>(position)] = solve.x[build.flow(l, t)];
    }
    for (int b = 0; b < build.B; ++b) {
      const double shed = std::max(0.0, solve.x[build.shed(b, t)]);
      period.load_shedding_mw[static_cast<size_t>(b)] = shed;
      period.objective += shed * build.model.c[build.shed(b, t)];
      const double curtail = std::max(0.0, solve.x[build.curtail(b, t)]);
      period.exogenous_curtailment_mw[static_cast<size_t>(b)] = curtail;
      period.objective += curtail * build.model.c[build.curtail(b, t)];
      if (have_duals) {
        period.lmp_per_mwh[static_cast<size_t>(b)] =
            solve.constraint_duals[inequality_rows + build.balance_row(b, t)] / dt;
      }
    }
    if (have_duals) {
      period.upward_reserve_price_per_mwh =
          solve.constraint_duals[inequality_rows + build.reserve_row(t)] / dt;
    }
  }
  return out;
}

void select_online_slack(HybridPowerSystem& snapshot) {
  int chosen_generator = -1;
  for (int g = 0; g < static_cast<int>(snapshot.ac.generators.size()); ++g) {
    const auto& generator = snapshot.ac.generators[static_cast<size_t>(g)];
    if (!generator.in_service) continue;
    if (generator.is_slack) {
      chosen_generator = g;
      break;
    }
    if (chosen_generator < 0) chosen_generator = g;
  }
  if (chosen_generator < 0) return;

  for (auto& bus : snapshot.ac.buses) {
    if (bus.bus_type == BusType::SLACK) bus.bus_type = BusType::PQ;
  }
  for (auto& generator : snapshot.ac.generators) generator.is_slack = false;
  auto& chosen = snapshot.ac.generators[static_cast<size_t>(chosen_generator)];
  chosen.is_slack = true;
  for (auto& bus : snapshot.ac.buses) {
    if (bus.index == chosen.bus) {
      bus.bus_type = BusType::SLACK;
      break;
    }
  }
}

void apply_pricing_state(
    HybridPowerSystem& snapshot,
    const PeriodNetworkData& period,
    const PricingPeriod& pricing) {
  for (size_t g = 0; g < snapshot.ac.generators.size() &&
                     g < pricing.generator_dispatch_mw.size(); ++g) {
    snapshot.ac.generators[g].pg_mw = pricing.generator_dispatch_mw[g];
  }

  // AC certification must replay the commercial served-load state, not the
  // original gross-load snapshot.  Allocate nodal shedding proportionally to
  // all load representations connected to that authored bus.
  const auto bus_positions = make_bus_position_map(snapshot);
  std::vector<double> served_fraction(snapshot.ac.buses.size(), 1.0);
  for (size_t b = 0; b < snapshot.ac.buses.size(); ++b) {
    const double gross = b < period.gross_demand_by_bus_mw.size()
        ? std::max(0.0, period.gross_demand_by_bus_mw[b])
        : 0.0;
    const double shed = b < pricing.load_shedding_mw.size()
        ? std::clamp(pricing.load_shedding_mw[b], 0.0, gross)
        : 0.0;
    const double fraction = gross > 1e-12 ? (gross - shed) / gross : 1.0;
    served_fraction[b] = fraction;
    snapshot.ac.buses[b].pd_mw *= fraction;
    snapshot.ac.buses[b].qd_mvar *= fraction;
  }
  for (auto& load : snapshot.ac.loads) {
    const auto bus = bus_positions.find(load.bus);
    if (load.in_service && bus != bus_positions.end()) {
      load.scaling *= served_fraction[static_cast<size_t>(bus->second)];
    }
  }
  for (auto& load : snapshot.ac.flexible_loads) {
    const auto bus = bus_positions.find(load.bus);
    if (load.in_service && bus != bus_positions.end()) {
      const double fraction = served_fraction[static_cast<size_t>(bus->second)];
      load.p_mw *= fraction;
      load.q_mvar *= fraction;
    }
  }

  std::vector<double> positive_exogenous_mw(snapshot.ac.buses.size(), 0.0);
  const auto add_positive = [&](int bus, double mw) {
    const auto found = bus_positions.find(bus);
    if (found != bus_positions.end()) {
      positive_exogenous_mw[static_cast<size_t>(found->second)] +=
          std::max(0.0, mw);
    }
  };
  for (const auto& source : snapshot.ac.static_generators)
    if (source.in_service) add_positive(source.bus, source.p_mw * source.scaling);
  for (const auto& source : snapshot.ac.renewable_gens)
    if (source.in_service) add_positive(source.bus, source.p_mw);
  for (const auto& source : snapshot.ac.pv_systems)
    if (source.in_service) add_positive(source.bus, source.p_mw);
  for (const auto& source : snapshot.ac.storage)
    if (source.in_service) add_positive(source.bus, source.p_mw);
  std::vector<double> exogenous_fraction(snapshot.ac.buses.size(), 1.0);
  for (size_t b = 0; b < snapshot.ac.buses.size(); ++b) {
    const double curtail = b < pricing.exogenous_curtailment_mw.size()
        ? std::max(0.0, pricing.exogenous_curtailment_mw[b])
        : 0.0;
    if (positive_exogenous_mw[b] > 1e-12) {
      exogenous_fraction[b] = std::clamp(
          1.0 - curtail / positive_exogenous_mw[b], 0.0, 1.0);
    }
  }
  const auto source_fraction = [&](int bus) {
    const auto found = bus_positions.find(bus);
    return found == bus_positions.end()
        ? 1.0
        : exogenous_fraction[static_cast<size_t>(found->second)];
  };
  for (auto& source : snapshot.ac.static_generators) {
    if (source.in_service && source.p_mw * source.scaling > 0.0) {
      source.scaling *= source_fraction(source.bus);
    }
  }
  for (auto& source : snapshot.ac.renewable_gens) {
    if (source.in_service && source.p_mw > 0.0) {
      const double fraction = source_fraction(source.bus);
      source.p_mw *= fraction;
      source.q_mvar *= fraction;
    }
  }
  for (auto& source : snapshot.ac.pv_systems) {
    if (source.in_service && source.p_mw > 0.0) {
      source.p_mw *= source_fraction(source.bus);
    }
  }
  for (auto& source : snapshot.ac.storage) {
    if (source.in_service && source.p_mw > 0.0) {
      source.p_mw *= source_fraction(source.bus);
    }
  }
  select_online_slack(snapshot);
}

ACValidationPeriod summarize_ac_validation(
    const HybridPowerSystem& snapshot,
    const PricingPeriod& pricing,
    const PowerFlowResult& power_flow,
    const MarketOptions& options) {
  ACValidationPeriod out;
  out.converged = power_flow.converged;
  out.residual = power_flow.residual;
  out.status = power_flow.converged
      ? "converged"
      : (power_flow.diagnostics.termination_reason.empty()
             ? "not_converged"
             : power_flow.diagnostics.termination_reason);
  if (!power_flow.converged) return out;

  for (size_t b = 0; b < power_flow.vm.size() &&
                     b < snapshot.ac.buses.size(); ++b) {
    const auto& bus = snapshot.ac.buses[b];
    const double vm = power_flow.vm[b];
    out.maximum_voltage_violation_pu = std::max(
        out.maximum_voltage_violation_pu,
        std::max({0.0, bus.vmin_pu - vm, vm - bus.vmax_pu}));
  }

  for (size_t l = 0; l < power_flow.branch_flows.size(); ++l) {
    const auto& flow = power_flow.branch_flows[l];
    out.total_branch_loss_mw += flow.pf_mw + flow.pt_mw;
    if (l < snapshot.ac.branches.size()) {
      const double rate = snapshot.ac.branches[l].rate_a_mva;
      if (rate > 1e-9) {
        const double from_mva = std::hypot(flow.pf_mw, flow.qf_mvar);
        const double to_mva = std::hypot(flow.pt_mw, flow.qt_mvar);
        const double apparent = std::max(from_mva, to_mva);
        out.maximum_branch_loading_percent = std::max(
            out.maximum_branch_loading_percent,
            100.0 * apparent / rate);
        out.maximum_branch_overload_mva = std::max(
            out.maximum_branch_overload_mva, apparent - rate);
      }
    }
  }
  const double generation = std::accumulate(
      pricing.generator_dispatch_mw.begin(), pricing.generator_dispatch_mw.end(), 0.0);
  double exogenous = 0.0;
  for (const auto& gen : snapshot.ac.static_generators)
    if (gen.in_service) exogenous += gen.p_mw * gen.scaling;
  for (const auto& gen : snapshot.ac.renewable_gens)
    if (gen.in_service) exogenous += gen.p_mw;
  for (const auto& pv : snapshot.ac.pv_systems)
    if (pv.in_service) exogenous += pv.p_mw;
  for (const auto& storage : snapshot.ac.storage)
    if (storage.in_service) exogenous += storage.p_mw;
  const double served = pricing.gross_demand_mw -
      std::accumulate(pricing.load_shedding_mw.begin(),
                      pricing.load_shedding_mw.end(), 0.0);
  out.slack_adjustment_mw = served + out.total_branch_loss_mw - generation - exogenous;
  for (size_t g = 0; g < snapshot.ac.generators.size(); ++g) {
    const auto& generator = snapshot.ac.generators[g];
    if (!generator.in_service || !generator.is_slack) continue;
    out.slack_generator_position = static_cast<int>(g);
    const double scheduled = g < pricing.generator_dispatch_mw.size()
        ? pricing.generator_dispatch_mw[g]
        : generator.pg_mw;
    const double adjusted = scheduled + out.slack_adjustment_mw;
    out.maximum_generator_active_violation_mw = std::max(
        {0.0, generator.pmin_mw - adjusted, adjusted - generator.pmax_mw});
    break;
  }
  out.maximum_branch_overload_mva =
      std::max(0.0, out.maximum_branch_overload_mva);
  out.secure =
      out.maximum_voltage_violation_pu <=
          std::max(0.0, options.ac_validation_voltage_tolerance_pu) &&
      out.maximum_branch_overload_mva <=
          std::max(0.0, options.ac_validation_thermal_tolerance_mva) &&
      out.maximum_generator_active_violation_mw <=
          std::max(0.0, options.ac_validation_generator_tolerance_mw);
  out.status = out.secure ? "secure" : "ac_security_limits_violated";
  return out;
}

void run_ac_contingency_validation(
    const HybridPowerSystem& system,
    const PricingBuild& build,
    const std::vector<int>& contingencies,
    const MarketOptions& options,
    MarketResult& result) {
  result.security.ac_contingency_validation_run = true;
  result.security.ac_contingencies_secure = true;
  for (int t = 0; t < build.T; ++t) {
    for (int outage_active : contingencies) {
      if (outage_active < 0 || outage_active >= build.L) continue;
      const int outage_position =
          build.branch_positions[static_cast<size_t>(outage_active)];
      ACContingencyCheck check;
      check.period = t;
      check.outage_branch_position = outage_position;
      check.outage_branch_index =
          system.ac.branches[static_cast<size_t>(outage_position)].index;
      HybridPowerSystem snapshot = build.periods[static_cast<size_t>(t)].snapshot;
      apply_pricing_state(
          snapshot, build.periods[static_cast<size_t>(t)],
          result.pricing[static_cast<size_t>(t)]);
      snapshot.ac.branches[static_cast<size_t>(outage_position)].in_service = false;
      try {
        const auto power_flow =
            solve_power_flow(snapshot, options.ac_validation_options);
        check.converged = power_flow.converged;
        check.status = power_flow.converged
            ? "converged"
            : (power_flow.diagnostics.termination_reason.empty()
                   ? "not_converged"
                   : power_flow.diagnostics.termination_reason);
        if (power_flow.converged) {
          for (size_t b = 0; b < power_flow.vm.size() &&
                             b < snapshot.ac.buses.size(); ++b) {
            const auto& bus = snapshot.ac.buses[b];
            const double vm = power_flow.vm[b];
            check.maximum_voltage_violation_pu = std::max(
                check.maximum_voltage_violation_pu,
                std::max({0.0, bus.vmin_pu - vm, vm - bus.vmax_pu}));
          }
          for (size_t l = 0; l < power_flow.branch_flows.size() &&
                             l < snapshot.ac.branches.size(); ++l) {
            const auto& branch = snapshot.ac.branches[l];
            if (!branch.in_service || branch.rate_a_mva <= 1e-9) continue;
            const auto& flow = power_flow.branch_flows[l];
            const double apparent = std::max(
                std::hypot(flow.pf_mw, flow.qf_mvar),
                std::hypot(flow.pt_mw, flow.qt_mvar));
            const double emergency =
                std::max(0.0, options.n1_emergency_rating_multiplier) *
                branch.rate_a_mva;
            check.maximum_branch_overload_mva = std::max(
                check.maximum_branch_overload_mva, apparent - emergency);
          }
        }
      } catch (const std::exception& error) {
        check.status = error.what();
      }
      check.maximum_branch_overload_mva =
          std::max(0.0, check.maximum_branch_overload_mva);
      check.secure = check.converged &&
          check.maximum_voltage_violation_pu <=
              options.ac_contingency_voltage_tolerance_pu &&
          check.maximum_branch_overload_mva <=
              options.ac_contingency_thermal_tolerance_mva;
      result.security.ac_contingencies_secure =
          result.security.ac_contingencies_secure && check.secure;
      result.security.ac_checks.push_back(std::move(check));
    }
  }
}

double incremental_offer_cost(const GeneratorOffer& offer, double dispatch_mw) {
  double remaining = std::max(0.0, dispatch_mw - offer.minimum_output_mw);
  double cost = offer.minimum_output_cost_per_hour;
  for (const auto& segment : offer.energy_segments) {
    const double accepted = std::min(remaining, std::max(0.0, segment.quantity_mw));
    cost += accepted * segment.price_per_mwh;
    remaining -= accepted;
    if (remaining <= 1e-9) break;
  }
  return cost;
}

void aggregate_participant_settlement(MarketResult& result) {
  std::unordered_map<std::string, size_t> participant_position;
  result.participant_settlement.reserve(result.participants.size());
  for (const auto& participant : result.participants) {
    ParticipantSettlement row;
    row.participant_id = participant.participant_id;
    row.participant_name = participant.participant_name;
    row.generator_positions = participant.generator_positions;
    participant_position.emplace(row.participant_id,
                                 result.participant_settlement.size());
    result.participant_settlement.push_back(std::move(row));
  }

  for (size_t g = 0; g < result.generator_settlement.size(); ++g) {
    const auto& generator = result.generator_settlement[g];
    const auto& offer = result.offers[g];
    const auto it = participant_position.find(offer.participant_id);
    if (it == participant_position.end()) continue;
    auto& participant = result.participant_settlement[it->second];
    participant.physical_capacity_mw += offer.physical_maximum_output_mw;
    participant.offered_capacity_mw += offer.offered_maximum_output_mw;
    participant.withheld_capacity_mw +=
        offer.physical_maximum_output_mw - offer.offered_maximum_output_mw;
    participant.energy_mwh += generator.energy_mwh;
    participant.reserve_mwh += generator.reserve_mwh;
    participant.market_revenue +=
        generator.energy_revenue + generator.reserve_revenue;
    participant.true_cost += generator.true_cost;
    participant.as_bid_cost += generator.as_bid_cost;
    participant.uplift += generator.uplift;
    participant.profit_after_uplift += generator.profit_after_uplift;
  }

  const double total_output = std::accumulate(
      result.participant_settlement.begin(), result.participant_settlement.end(), 0.0,
      [](double total, const ParticipantSettlement& row) {
        return total + std::max(0.0, row.energy_mwh);
      });
  const double total_revenue = std::accumulate(
      result.participant_settlement.begin(), result.participant_settlement.end(), 0.0,
      [](double total, const ParticipantSettlement& row) {
        return total + std::max(0.0, row.market_revenue);
      });
  std::vector<double> output_shares;
  output_shares.reserve(result.participant_settlement.size());
  bool first = true;
  for (const auto& row : result.participant_settlement) {
    const double output_share = total_output > 1e-12
        ? 100.0 * std::max(0.0, row.energy_mwh) / total_output
        : 0.0;
    const double revenue_share = total_revenue > 1e-12
        ? 100.0 * std::max(0.0, row.market_revenue) / total_revenue
        : 0.0;
    output_shares.push_back(output_share);
    result.market_power.output_hhi += output_share * output_share;
    result.market_power.revenue_hhi += revenue_share * revenue_share;
    if (output_share > result.market_power.maximum_output_share_percent) {
      result.market_power.maximum_output_share_percent = output_share;
      result.market_power.maximum_output_participant = row.participant_id;
    }
    if (first || row.profit_after_uplift > result.market_power.maximum_profit) {
      first = false;
      result.market_power.maximum_profit = row.profit_after_uplift;
      result.market_power.maximum_profit_participant = row.participant_id;
    }
    result.market_power.total_withheld_capacity_mw += row.withheld_capacity_mw;
  }
  std::sort(output_shares.begin(), output_shares.end(), std::greater<double>());
  for (size_t i = 0; i < std::min<size_t>(3, output_shares.size()); ++i) {
    result.market_power.top3_output_share_percent += output_shares[i];
  }
  if (!result.behavior_actions.empty()) {
    result.market_power.average_offer_markup_fraction = std::accumulate(
        result.behavior_actions.begin(), result.behavior_actions.end(), 0.0,
        [](double total, const BehaviorAction& action) {
          return total + action.energy_markup_fraction;
        }) / static_cast<double>(result.behavior_actions.size());
  }
}

void settle_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const PricingBuild& build,
    const UCSchedule& commitment,
    MarketResult& result) {
  const double dt = time_series.step_duration_hr;
  const auto bus_positions = make_bus_position_map(system);
  result.generator_settlement.resize(build.generator_positions.size());
  for (int g = 0; g < build.G; ++g) {
    const int position = build.generator_positions[static_cast<size_t>(g)];
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    const auto& offer = result.offers[static_cast<size_t>(g)];
    auto& settlement = result.generator_settlement[static_cast<size_t>(g)];
    settlement.generator_position = position;
    settlement.generator_index = generator.index;
    settlement.generator_name = generator.name;
    const int bus = bus_positions.at(generator.bus);
    bool previous_online = generator.pg_mw > 1e-6;
    for (int t = 0; t < build.T; ++t) {
      const auto& pricing = result.pricing[static_cast<size_t>(t)];
      const bool online = commitment.gen_commit[static_cast<size_t>(g)][static_cast<size_t>(t)] != 0;
      const double p = pricing.generator_dispatch_mw[static_cast<size_t>(position)];
      const double reserve = pricing.upward_reserve_mw[static_cast<size_t>(position)];
      const double lmp = pricing.lmp_per_mwh[static_cast<size_t>(bus)];
      settlement.energy_mwh += p * dt;
      settlement.reserve_mwh += reserve * dt;
      settlement.energy_revenue += lmp * p * dt;
      settlement.reserve_revenue +=
          pricing.upward_reserve_price_per_mwh * reserve * dt;
      if (online) {
        settlement.true_cost +=
            (generator.cost_c2 * p * p + generator.cost_c1 * p + generator.cost_c0) * dt;
        settlement.as_bid_cost +=
            (incremental_offer_cost(offer, p) + offer.no_load_price_per_hour +
             offer.upward_reserve_price_per_mwh * reserve) * dt;
      }
      if (online && !previous_online) {
        settlement.true_cost += generator.startup_cost;
        settlement.as_bid_cost += offer.startup_price;
      }
      if (!online && previous_online) {
        settlement.true_cost += generator.shutdown_cost;
        settlement.as_bid_cost += offer.shutdown_price;
      }
      previous_online = online;
    }
    settlement.offered_cost_markup =
        settlement.as_bid_cost - settlement.true_cost;
    settlement.uplift = std::max(
        0.0, settlement.as_bid_cost - settlement.energy_revenue -
                 settlement.reserve_revenue);
    settlement.profit_after_uplift =
        settlement.energy_revenue + settlement.reserve_revenue + settlement.uplift -
        settlement.true_cost;
    result.settlement.resource_energy_revenue += settlement.energy_revenue;
    result.settlement.resource_reserve_revenue += settlement.reserve_revenue;
    result.settlement.resource_uplift_revenue += settlement.uplift;
  }

  // Settle fixed exogenous injections (renewable, static generation, storage)
  // at the nodal energy price so the merchandising identity remains complete.
  for (int t = 0; t < build.T; ++t) {
    const auto& pricing = result.pricing[static_cast<size_t>(t)];
    const auto& period = build.periods[static_cast<size_t>(t)];
    for (int b = 0; b < build.B; ++b) {
      const double lmp = pricing.lmp_per_mwh[static_cast<size_t>(b)];
      const double served_load_mw = std::max(
          0.0, period.gross_demand_by_bus_mw[static_cast<size_t>(b)] -
                   pricing.load_shedding_mw[static_cast<size_t>(b)]);
      result.settlement.customer_energy_payment +=
          lmp * served_load_mw * dt;
      result.settlement.resource_energy_revenue +=
          lmp * (period.exogenous_injection_by_bus_mw[static_cast<size_t>(b)] -
                 pricing.exogenous_curtailment_mw[static_cast<size_t>(b)]) * dt;
    }
    result.settlement.customer_reserve_charge +=
        pricing.upward_reserve_price_per_mwh * pricing.reserve_requirement_mw * dt;
  }
  result.settlement.customer_uplift_charge =
      result.settlement.resource_uplift_revenue;
  result.settlement.customer_total_payment =
      result.settlement.customer_energy_payment +
      result.settlement.customer_reserve_charge +
      result.settlement.customer_uplift_charge;
  result.settlement.resource_total_revenue =
      result.settlement.resource_energy_revenue +
      result.settlement.resource_reserve_revenue +
      result.settlement.resource_uplift_revenue;
  result.settlement.congestion_rent =
      result.settlement.customer_energy_payment -
      result.settlement.resource_energy_revenue;
  result.settlement.cashflow_residual =
      result.settlement.customer_total_payment -
      result.settlement.resource_total_revenue -
      result.settlement.congestion_rent;
  aggregate_participant_settlement(result);
}

}  // namespace

MarketResult run_day_ahead_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const MarketOptions& options) {
  MarketResult result;
  if (time_series.num_steps <= 0 || time_series.step_duration_hr <= 0.0) {
    result.status = "invalid_time_series";
    result.warnings.push_back("Market time series must have positive periods and duration.");
    return result;
  }
  if (has_unsupported_hybrid_market_assets(system)) {
    result.status = "unsupported_hybrid_market_assets";
    result.warnings.push_back(
        "The first market slice prices the AC generator/branch subset only; "
        "external-grid and hybrid AC/DC co-optimisation is not yet enabled.");
    return result;
  }
  if (system.ac.generators.empty() || system.ac.buses.empty()) {
    result.status = "empty_ac_market";
    return result;
  }
  for (const auto& generator : system.ac.generators) {
    if (generator.in_service && generator.cost_c2 < -1e-12) {
      result.status = "nonconvex_generator_offer";
      result.warnings.push_back(
          "Negative quadratic generator costs cannot be represented by the "
          "convex piecewise-linear pricing LP.");
      return result;
    }
  }

  OfferSubmission submission;
  try {
    submission = submit_participant_offers(
        system, options.participants, options.energy_offer_segments);
  } catch (const std::exception& error) {
    result.status = "offer_submission_error";
    result.warnings.push_back(error.what());
    return result;
  }
  result.participants = submission.participants;
  result.behavior_actions = submission.actions;
  result.offers = submission.offers;
  result.warnings.insert(result.warnings.end(),
                         submission.warnings.begin(), submission.warnings.end());
  TimeSeriesPFOptions uc_options = options.uc_options;
  uc_options.enable_network_constraints = options.enable_network_constraints;
  uc_options.reserve_requirement_fraction =
      std::max(0.0, options.upward_reserve_fraction);
  uc_options.run_opf = false;
  uc_options.verbose = options.verbose;
  MarketOptions commitment_options = options;
  commitment_options.uc_options = uc_options;
  try {
    result.commitment = solve_market_commitment(
        system, time_series, result.offers, commitment_options);
  } catch (const std::exception& error) {
    result.status = "scuc_model_error";
    result.warnings.push_back(error.what());
    return result;
  }
  if (!result.commitment.feasible &&
      uc_options.uc_solver != UCSolverChoice::HiGHS) {
    auto retry_options = commitment_options;
    retry_options.uc_options.uc_solver = UCSolverChoice::HiGHS;
    auto retry = solve_market_commitment(
        system, time_series, result.offers, retry_options);
    if (retry.feasible) {
      result.warnings.push_back(
          "The requested SCUC backend did not return a feasible schedule; "
          "the market runner recovered with the HiGHS backend.");
      result.commitment = std::move(retry);
      uc_options = retry_options.uc_options;
    }
  }
  result.commitment_cost = result.commitment.total_cost;
  if (!result.commitment.feasible) {
    result.status = "scuc_infeasible";
    return result;
  }

  PricingBuild pricing_build;
  MarketOptions effective_options = options;
  effective_options.uc_options = uc_options;
  try {
    pricing_build = build_pricing_model(
        system, time_series, result.commitment, result.offers, effective_options);
  } catch (const std::exception& error) {
    result.status = "pricing_model_error";
    result.warnings.push_back(error.what());
    return result;
  }

  engine::SimplexOptions simplex_options;
  simplex_options.max_iter = std::max(10000, options.uc_options.opf_options.max_inner_iterations);
  simplex_options.feasibility_tol = 1e-8;
  simplex_options.optimality_tol = 1e-8;
  simplex_options.verbose = options.verbose;
  const auto baseline_solved = engine::solve_lp_with_basis(
      pricing_build.model, simplex_options, nullptr);
  if (!baseline_solved.result.stats.success) {
    result.status = "sced_infeasible:" + baseline_solved.result.stats.status;
    return result;
  }
  const auto baseline_pricing = extract_pricing(
      system, pricing_build, baseline_solved.result,
      time_series.step_duration_hr);
  const auto pricing_objective = [](const std::vector<PricingPeriod>& pricing) {
    return std::accumulate(
        pricing.begin(), pricing.end(), 0.0,
        [](double total, const PricingPeriod& period) {
          return total + period.objective;
        });
  };
  const double baseline_objective = pricing_objective(baseline_pricing);
  result.pricing = baseline_pricing;
  result.pricing_objective = baseline_objective;

  LODFModel lodf;
  bool lodf_built = false;
  std::vector<int> n1_contingencies;
  if (options.enable_n1_security || options.run_ac_contingency_validation) {
    lodf = build_lodf_model(system);
    lodf_built = true;
    result.security.lodf_available = lodf.available;
    result.security.skipped_islanding_branch_positions =
        lodf.skipped_islanding_branch_positions;
    result.security.warnings.insert(
        result.security.warnings.end(), lodf.warnings.begin(), lodf.warnings.end());
    result.warnings.insert(
        result.warnings.end(), lodf.warnings.begin(), lodf.warnings.end());
  }

  result.security.enabled = options.enable_n1_security;
  result.security.baseline_pricing_objective = baseline_objective;
  result.security.secured_pricing_objective = baseline_objective;
  if (options.enable_n1_security) {
    if (lodf.available) {
      n1_contingencies = select_n1_contingencies(
          lodf, result.pricing, options.n1_max_contingencies);
    }
    result.security.candidate_contingencies =
        static_cast<int>(n1_contingencies.size());

    N1Screen current_screen;
    if (lodf.available && !n1_contingencies.empty()) {
      current_screen = screen_n1(
          system, lodf, n1_contingencies, result.pricing, options);
    }
    result.security.initial_violations =
        static_cast<int>(current_screen.violations.size());
    result.security.initial_worst_overload_mw =
        current_screen.worst_overload_mw;
    result.security.trajectory.push_back(N1Iteration{
        0,
        static_cast<int>(current_screen.violations.size()),
        0,
        0,
        current_screen.worst_overload_mw});

    std::vector<N1SecurityCut> cuts;
    std::set<std::tuple<int, int, int, int>> cut_keys;
    std::unordered_map<int, int> active_position;
    for (int active = 0;
         active < static_cast<int>(lodf.branch_positions.size()); ++active) {
      active_position.emplace(
          lodf.branch_positions[static_cast<size_t>(active)], active);
    }

    const int max_iterations = std::max(0, options.n1_max_iterations);
    const int max_cuts = std::max(0, options.n1_max_cuts_per_iteration);
    for (int iteration = 1;
         lodf.available && !current_screen.violations.empty() &&
         iteration <= max_iterations;
         ++iteration) {
      int added = 0;
      for (const auto& violation : current_screen.violations) {
        if (added >= max_cuts) break;
        const auto monitored = active_position.find(
            violation.monitored_branch_position);
        const auto outage = active_position.find(
            violation.outage_branch_position);
        if (monitored == active_position.end() ||
            outage == active_position.end()) {
          continue;
        }
        const int direction =
            violation.post_contingency_flow_mw >= 0.0 ? 1 : -1;
        const auto key = std::make_tuple(
            violation.period, monitored->second, outage->second, direction);
        if (!cut_keys.insert(key).second) continue;
        cuts.push_back(N1SecurityCut{
            violation.period,
            monitored->second,
            outage->second,
            lodf.values(monitored->second, outage->second),
            static_cast<double>(direction),
            violation.emergency_rating_mw});
        ++added;
      }
      if (added == 0) {
        result.security.warnings.push_back(
            "N-1 screening found violations but no new unique cuts could be added.");
        break;
      }

      PricingBuild secured_build;
      try {
        secured_build = build_pricing_model(
            system, time_series, result.commitment, result.offers,
            effective_options, cuts);
      } catch (const std::exception& error) {
        result.status = "security_pricing_model_error";
        result.warnings.push_back(error.what());
        result.security.cuts_added = static_cast<int>(cuts.size());
        result.security.iterations = iteration;
        return result;
      }
      const auto secured_solved = engine::solve_lp_with_basis(
          secured_build.model, simplex_options, nullptr);
      if (!secured_solved.result.stats.success) {
        result.status =
            "security_sced_infeasible:" + secured_solved.result.stats.status;
        result.security.cuts_added = static_cast<int>(cuts.size());
        result.security.iterations = iteration;
        result.security.remaining_violations = current_screen.violations;
        result.security.final_violations =
            static_cast<int>(current_screen.violations.size());
        result.security.final_worst_overload_mw =
            current_screen.worst_overload_mw;
        return result;
      }

      auto secured_pricing = extract_pricing(
          system, secured_build, secured_solved.result,
          time_series.step_duration_hr);
      current_screen = screen_n1(
          system, lodf, n1_contingencies, secured_pricing, options);
      pricing_build = std::move(secured_build);
      result.pricing = std::move(secured_pricing);
      result.pricing_objective = pricing_objective(result.pricing);
      result.security.iterations = iteration;
      result.security.cuts_added = static_cast<int>(cuts.size());
      result.security.trajectory.push_back(N1Iteration{
          iteration,
          static_cast<int>(current_screen.violations.size()),
          added,
          static_cast<int>(cuts.size()),
          current_screen.worst_overload_mw});
    }

    result.security.dc_n1_secured =
        lodf.available && !n1_contingencies.empty() &&
        current_screen.violations.empty();
    result.security.remaining_violations = current_screen.violations;
    result.security.final_violations =
        static_cast<int>(current_screen.violations.size());
    result.security.final_worst_overload_mw =
        current_screen.worst_overload_mw;
    result.security.secured_pricing_objective = result.pricing_objective;
    result.security.preventive_redispatch_cost = std::max(
        0.0, result.security.secured_pricing_objective - baseline_objective);
    if (!lodf.available) {
      result.security.warnings.push_back(
          "N-1 security was requested but no usable LODF contingencies are available.");
    } else if (n1_contingencies.empty()) {
      result.security.warnings.push_back(
          "N-1 security was requested but the contingency set is empty.");
    } else if (!result.security.dc_n1_secured) {
      result.security.warnings.push_back(
          "The N-1 cut loop stopped with remaining DC contingency violations.");
    }
  }

  result.num_pricing_converged = 0;
  for (const auto& period : result.pricing) {
    if (period.converged) ++result.num_pricing_converged;
  }

  if (options.run_ac_validation) {
    result.ac_validation.reserve(static_cast<size_t>(time_series.num_steps));
    result.ac_power_flow_results.reserve(static_cast<size_t>(time_series.num_steps));
    for (int t = 0; t < time_series.num_steps; ++t) {
      HybridPowerSystem snapshot =
          pricing_build.periods[static_cast<size_t>(t)].snapshot;
      const auto& pricing = result.pricing[static_cast<size_t>(t)];
      apply_pricing_state(
          snapshot, pricing_build.periods[static_cast<size_t>(t)], pricing);
      auto power_flow = solve_power_flow(snapshot, options.ac_validation_options);
      const auto validation = summarize_ac_validation(
          snapshot, pricing, power_flow, options);
      result.ac_validation.push_back(validation);
      if (power_flow.converged) ++result.num_ac_converged;
      if (validation.secure) ++result.num_ac_secure;
      result.ac_power_flow_results.push_back(std::move(power_flow));
    }
  }

  if (options.run_ac_contingency_validation) {
    std::vector<int> ac_contingencies;
    if (lodf_built && lodf.available) {
      ac_contingencies = select_n1_contingencies(
          lodf, result.pricing, options.max_ac_contingencies);
    }
    if (ac_contingencies.empty()) {
      result.security.ac_contingency_validation_run = true;
      result.security.ac_contingencies_secure = false;
      result.security.warnings.push_back(
          "AC contingency validation was requested but no usable contingencies are available.");
    } else {
      run_ac_contingency_validation(
          system, pricing_build, ac_contingencies, options, result);
    }
  }

  MarketResult baseline_result;
  baseline_result.participants = result.participants;
  baseline_result.behavior_actions = result.behavior_actions;
  baseline_result.offers = result.offers;
  baseline_result.pricing = baseline_pricing;
  settle_market(system, time_series, pricing_build,
                result.commitment, baseline_result);
  settle_market(system, time_series, pricing_build,
                result.commitment, result);
  if (options.enable_n1_security) {
    result.security.incremental_security_uplift = std::max(
        0.0, result.settlement.resource_uplift_revenue -
                 baseline_result.settlement.resource_uplift_revenue);
    result.security.customer_payment_impact =
        result.settlement.customer_total_payment -
        baseline_result.settlement.customer_total_payment;
  }
  const bool pricing_ok = result.num_pricing_converged == time_series.num_steps;
  const bool ac_ok = !options.run_ac_validation ||
      result.num_ac_secure == time_series.num_steps;
  const bool n1_ok = !options.enable_n1_security ||
      result.security.dc_n1_secured;
  const bool ac_contingency_ok = !options.run_ac_contingency_validation ||
      result.security.ac_contingencies_secure;
  result.feasible = pricing_ok && ac_ok && n1_ok && ac_contingency_ok;
  if (result.feasible) {
    result.status = "converged";
  } else if (!pricing_ok) {
    result.status = "sced_infeasible";
  } else if (!n1_ok) {
    result.status = "n1_security_failed";
  } else if (!ac_contingency_ok) {
    result.status = "ac_contingency_failed";
  } else {
    result.status = "ac_validation_failed";
  }
  return result;
}

RealTimeMarketResult run_real_time_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& day_ahead_time_series,
    const TimeSeriesData& realized_time_series,
    const MarketResult& day_ahead_result,
    const RealTimeMarketOptions& options) {
  RealTimeMarketResult result;
  result.ancillary_services_enabled = options.ancillary_services.enabled;
  if (!day_ahead_result.feasible ||
      !day_ahead_result.commitment.feasible ||
      day_ahead_result.pricing.empty()) {
    result.status = "invalid_day_ahead_baseline";
    result.warnings.push_back(
        "Real-time clearing requires a feasible day-ahead commitment and pricing result.");
    return result;
  }
  if (day_ahead_time_series.num_steps <= 0 ||
      realized_time_series.num_steps != day_ahead_time_series.num_steps ||
      std::abs(realized_time_series.step_duration_hr -
               day_ahead_time_series.step_duration_hr) > 1e-12) {
    result.status = "real_time_horizon_mismatch";
    result.warnings.push_back(
        "Day-ahead and realized time series must have identical horizons and interval duration.");
    return result;
  }
  const int T = day_ahead_time_series.num_steps;
  if (static_cast<int>(day_ahead_result.pricing.size()) != T) {
    result.status = "day_ahead_pricing_horizon_mismatch";
    return result;
  }
  for (const auto& commitment : day_ahead_result.commitment.gen_commit) {
    if (static_cast<int>(commitment.size()) != T) {
      result.status = "day_ahead_commitment_horizon_mismatch";
      return result;
    }
  }

  const auto& ancillary = options.ancillary_services;
  const auto invalid_nonnegative = [](double value) {
    return !std::isfinite(value) || value < 0.0;
  };
  if (ancillary.enabled &&
      (invalid_nonnegative(
           ancillary.generator_imbalance_tolerance_fraction) ||
       invalid_nonnegative(ancillary.load_imbalance_tolerance_fraction) ||
       invalid_nonnegative(ancillary.generator_imbalance_penalty_per_mwh) ||
       invalid_nonnegative(ancillary.load_imbalance_penalty_per_mwh) ||
       invalid_nonnegative(
           ancillary.reserve_performance_payment_per_mwh) ||
       invalid_nonnegative(
           ancillary.reserve_nonperformance_penalty_per_mwh) ||
       std::any_of(
           ancillary.reserve_performance_factor_by_generator.begin(),
           ancillary.reserve_performance_factor_by_generator.end(),
           [](double factor) {
             return !std::isfinite(factor) || factor < 0.0 || factor > 1.0;
           }))) {
    result.status = "invalid_ancillary_service_options";
    result.warnings.push_back(
        "Ancillary-service tolerances and prices must be non-negative, and reserve performance factors must be in [0, 1].");
    return result;
  }

  MarketOptions real_time_options = options.market_options;
  real_time_options.participants = day_ahead_result.participants;
  // This real-time slice activates day-ahead reserve and clears replacement
  // energy; it does not procure a second, otherwise-unsettled reserve product.
  real_time_options.upward_reserve_fraction = 0.0;
  if (!day_ahead_result.offers.empty() &&
      !day_ahead_result.offers.front().energy_segments.empty()) {
    real_time_options.energy_offer_segments = static_cast<int>(
        day_ahead_result.offers.front().energy_segments.size());
  }
  real_time_options.uc_options.fixed_commitment_schedule =
      &day_ahead_result.commitment.gen_commit;
  const auto bus_positions = make_bus_position_map(system);
  std::vector<PeriodNetworkData> day_ahead_periods;
  std::vector<PeriodNetworkData> realized_periods;
  day_ahead_periods.reserve(static_cast<size_t>(T));
  realized_periods.reserve(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    day_ahead_periods.push_back(make_period_data(
        system, day_ahead_time_series, day_ahead_result.commitment, t,
        real_time_options.uc_options, bus_positions));
    realized_periods.push_back(make_period_data(
        system, realized_time_series, day_ahead_result.commitment, t,
        real_time_options.uc_options, bus_positions));
  }

  const double missing_dispatch =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<std::vector<double>> reserve_instruction_mw(
      system.ac.generators.size(),
      std::vector<double>(static_cast<size_t>(T), 0.0));
  std::vector<double> reserve_activation_requirement_mw(
      static_cast<size_t>(T), 0.0);
  std::vector<std::vector<double>> minimum_dispatch_mw(
      system.ac.generators.size(),
      std::vector<double>(static_cast<size_t>(T), missing_dispatch));
  if (ancillary.enabled) {
    for (int t = 0; t < T; ++t) {
      const auto& day_ahead_period = day_ahead_periods[static_cast<size_t>(t)];
      const auto& realized_period = realized_periods[static_cast<size_t>(t)];
      const double day_ahead_net_demand_mw = std::accumulate(
          day_ahead_period.net_demand_by_bus_mw.begin(),
          day_ahead_period.net_demand_by_bus_mw.end(), 0.0);
      const double realized_net_demand_mw = std::accumulate(
          realized_period.net_demand_by_bus_mw.begin(),
          realized_period.net_demand_by_bus_mw.end(), 0.0);
      const double activation = std::max(
          0.0, realized_net_demand_mw - day_ahead_net_demand_mw);
      reserve_activation_requirement_mw[static_cast<size_t>(t)] = activation;
      const auto& day_ahead_pricing =
          day_ahead_result.pricing[static_cast<size_t>(t)];
      const double total_award = std::accumulate(
          day_ahead_pricing.upward_reserve_mw.begin(),
          day_ahead_pricing.upward_reserve_mw.end(), 0.0);
      if (total_award <= 1e-12 || activation <= 1e-12) continue;
      for (int position : active_generator_positions(system)) {
        const double award = std::max(
            0.0, day_ahead_pricing.upward_reserve_mw[
                     static_cast<size_t>(position)]);
        const double instruction = std::min(
            award, activation * award / total_award);
        reserve_instruction_mw[static_cast<size_t>(position)]
                              [static_cast<size_t>(t)] = instruction;
        if (instruction > 1e-12) {
          minimum_dispatch_mw[static_cast<size_t>(position)]
                             [static_cast<size_t>(t)] =
              day_ahead_pricing.generator_dispatch_mw[
                  static_cast<size_t>(position)] + instruction;
        }
      }
    }
  }

  if (ancillary.enabled) {
    MarketOptions instruction_options = real_time_options;
    instruction_options.minimum_dispatch_schedule_mw = &minimum_dispatch_mw;
    instruction_options.run_ac_validation = false;
    instruction_options.run_ac_contingency_validation = false;
    result.dispatch_instruction_market = run_day_ahead_market(
        system, realized_time_series, instruction_options);
    if (!result.dispatch_instruction_market.feasible ||
        static_cast<int>(result.dispatch_instruction_market.pricing.size()) !=
            T) {
      result.status = "reserve_activation_dispatch_failed:" +
          result.dispatch_instruction_market.status;
      result.warnings = result.dispatch_instruction_market.warnings;
      return result;
    }

    std::vector<std::vector<double>> actual_fixed_dispatch_mw(
        system.ac.generators.size(),
        std::vector<double>(static_cast<size_t>(T), missing_dispatch));
    for (int t = 0; t < T; ++t) {
      const auto& instructed =
          result.dispatch_instruction_market.pricing[static_cast<size_t>(t)];
      for (int position : active_generator_positions(system)) {
        const double reserve_instruction =
            reserve_instruction_mw[static_cast<size_t>(position)]
                                  [static_cast<size_t>(t)];
        if (reserve_instruction <= 1e-12) continue;
        const double factor =
            position < static_cast<int>(
                           ancillary.reserve_performance_factor_by_generator
                               .size())
                ? ancillary.reserve_performance_factor_by_generator[
                      static_cast<size_t>(position)]
                : 1.0;
        const double shortfall = reserve_instruction * (1.0 - factor);
        actual_fixed_dispatch_mw[static_cast<size_t>(position)]
                                [static_cast<size_t>(t)] =
            instructed.generator_dispatch_mw[static_cast<size_t>(position)] -
            shortfall;
      }
    }
    MarketOptions balancing_options = real_time_options;
    balancing_options.fixed_dispatch_schedule_mw = &actual_fixed_dispatch_mw;
    result.real_time_market = run_day_ahead_market(
        system, realized_time_series, balancing_options);
    result.warnings = result.dispatch_instruction_market.warnings;
    result.warnings.insert(result.warnings.end(),
                           result.real_time_market.warnings.begin(),
                           result.real_time_market.warnings.end());
  } else {
    result.real_time_market = run_day_ahead_market(
        system, realized_time_series, real_time_options);
    result.warnings = result.real_time_market.warnings;
  }
  result.status = result.real_time_market.status;
  if (static_cast<int>(result.real_time_market.pricing.size()) != T) {
    result.status = "real_time_pricing_failed:" + result.real_time_market.status;
    return result;
  }

  const double dt = realized_time_series.step_duration_hr;

  std::unordered_map<int, const GeneratorSettlement*> day_ahead_generator;
  for (const auto& settlement : day_ahead_result.generator_settlement) {
    day_ahead_generator[settlement.generator_position] = &settlement;
  }
  std::unordered_map<int, const GeneratorSettlement*> real_time_generator;
  for (const auto& settlement : result.real_time_market.generator_settlement) {
    real_time_generator[settlement.generator_position] = &settlement;
  }
  std::unordered_map<int, std::string> participant_by_generator;
  for (const auto& offer : day_ahead_result.offers) {
    participant_by_generator[offer.generator_position] = offer.participant_id;
  }

  const auto generator_positions = active_generator_positions(system);
  result.generator_deviation_settlement.reserve(generator_positions.size());
  std::unordered_map<int, size_t> deviation_by_generator;
  for (int position : generator_positions) {
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    GeneratorDeviationSettlement settlement;
    settlement.participant_id = participant_by_generator[position];
    settlement.generator_position = position;
    settlement.generator_index = generator.index;
    settlement.generator_name = generator.name;
    const auto da_it = day_ahead_generator.find(position);
    if (da_it != day_ahead_generator.end()) {
      settlement.day_ahead_energy_revenue = da_it->second->energy_revenue;
      settlement.day_ahead_reserve_revenue = da_it->second->reserve_revenue;
      settlement.day_ahead_uplift = da_it->second->uplift;
    }
    const auto rt_it = real_time_generator.find(position);
    if (rt_it != real_time_generator.end()) {
      settlement.actual_true_cost = rt_it->second->true_cost;
    }
    deviation_by_generator[position] =
        result.generator_deviation_settlement.size();
    result.generator_deviation_settlement.push_back(std::move(settlement));
  }

  result.periods.reserve(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    const auto& day_ahead_pricing =
        day_ahead_result.pricing[static_cast<size_t>(t)];
    const auto& real_time_pricing =
        result.real_time_market.pricing[static_cast<size_t>(t)];
    const auto& dispatch_instruction_pricing = ancillary.enabled
        ? result.dispatch_instruction_market.pricing[static_cast<size_t>(t)]
        : real_time_pricing;
    RealTimePeriod period;
    period.period = t;
    period.day_ahead_demand_mw = day_ahead_pricing.gross_demand_mw;
    period.realized_demand_mw = real_time_pricing.gross_demand_mw;
    period.demand_deviation_mw =
        period.realized_demand_mw - period.day_ahead_demand_mw;
    const auto& day_ahead_period = day_ahead_periods[static_cast<size_t>(t)];
    const auto& realized_period = realized_periods[static_cast<size_t>(t)];
    period.reserve_activation_requirement_mw =
        reserve_activation_requirement_mw[static_cast<size_t>(t)];
    if (!day_ahead_pricing.lmp_per_mwh.empty()) {
      period.average_day_ahead_lmp_per_mwh = std::accumulate(
          day_ahead_pricing.lmp_per_mwh.begin(),
          day_ahead_pricing.lmp_per_mwh.end(), 0.0) /
          static_cast<double>(day_ahead_pricing.lmp_per_mwh.size());
    }
    if (!real_time_pricing.lmp_per_mwh.empty()) {
      period.average_real_time_lmp_per_mwh = std::accumulate(
          real_time_pricing.lmp_per_mwh.begin(),
          real_time_pricing.lmp_per_mwh.end(), 0.0) /
          static_cast<double>(real_time_pricing.lmp_per_mwh.size());
    }

    for (int position : generator_positions) {
      const auto& generator = system.ac.generators[static_cast<size_t>(position)];
      const auto bus_it = bus_positions.find(generator.bus);
      if (bus_it == bus_positions.end()) continue;
      const double day_ahead_mw =
          day_ahead_pricing.generator_dispatch_mw[static_cast<size_t>(position)];
      const double real_time_mw =
          real_time_pricing.generator_dispatch_mw[static_cast<size_t>(position)];
      const double dispatch_instruction_mw =
          dispatch_instruction_pricing.generator_dispatch_mw[
              static_cast<size_t>(position)];
      const double deviation_mw = real_time_mw - day_ahead_mw;
      const double deviation_mwh = deviation_mw * dt;
      const double real_time_lmp = real_time_pricing.lmp_per_mwh[
          static_cast<size_t>(bus_it->second)];
      auto& settlement = result.generator_deviation_settlement[
          deviation_by_generator.at(position)];
      settlement.day_ahead_energy_mwh += day_ahead_mw * dt;
      settlement.real_time_energy_mwh += real_time_mw * dt;
      settlement.real_time_dispatch_instruction_mwh +=
          dispatch_instruction_mw * dt;
      settlement.deviation_mwh += deviation_mwh;
      settlement.real_time_deviation_revenue +=
          deviation_mwh * real_time_lmp;
      if (ancillary.enabled) {
        const double instruction_mw =
            reserve_instruction_mw[static_cast<size_t>(position)]
                                  [static_cast<size_t>(t)];
        const double performance_factor =
            position < static_cast<int>(
                           ancillary.reserve_performance_factor_by_generator
                               .size())
                ? ancillary.reserve_performance_factor_by_generator[
                      static_cast<size_t>(position)]
                : 1.0;
        const double delivered_mw = instruction_mw * performance_factor;
        const double shortfall_mw =
            std::max(0.0, instruction_mw - delivered_mw);
        const double actual_output_deviation_mw =
            instruction_mw > 1e-12
                ? real_time_mw - dispatch_instruction_mw
                : 0.0;
        const double tolerance_mw =
            ancillary.generator_imbalance_tolerance_fraction *
            std::max(std::abs(dispatch_instruction_mw), 1.0);
        const double penalized_imbalance_mw = std::max(
            0.0, std::abs(actual_output_deviation_mw) - tolerance_mw);

        settlement.instructed_reserve_mwh += instruction_mw * dt;
        settlement.delivered_reserve_mwh += delivered_mw * dt;
        settlement.reserve_shortfall_mwh += shortfall_mw * dt;
        settlement.actual_output_deviation_mwh +=
            actual_output_deviation_mw * dt;
        settlement.reserve_performance_payment +=
            delivered_mw * dt *
            ancillary.reserve_performance_payment_per_mwh;
        settlement.reserve_nonperformance_charge +=
            shortfall_mw * dt *
            ancillary.reserve_nonperformance_penalty_per_mwh;
        settlement.penalized_imbalance_mwh += penalized_imbalance_mw * dt;
        settlement.imbalance_charge +=
            penalized_imbalance_mw * dt *
            ancillary.generator_imbalance_penalty_per_mwh;
        period.reserve_instruction_mw += instruction_mw;
        period.reserve_delivered_mw += delivered_mw;
        period.reserve_shortfall_mw += shortfall_mw;
      }
      period.absolute_generator_deviation_mw += std::abs(deviation_mw);
      period.resource_deviation_revenue += deviation_mwh * real_time_lmp;
    }

    for (int b = 0; b < static_cast<int>(system.ac.buses.size()); ++b) {
      const double real_time_lmp =
          real_time_pricing.lmp_per_mwh[static_cast<size_t>(b)];
      const double load_deviation_mw =
          realized_period.gross_demand_by_bus_mw[static_cast<size_t>(b)] -
          day_ahead_period.gross_demand_by_bus_mw[static_cast<size_t>(b)];
      const double served_load_deviation_mw =
          (realized_period.gross_demand_by_bus_mw[static_cast<size_t>(b)] -
           real_time_pricing.load_shedding_mw[static_cast<size_t>(b)]) -
          (day_ahead_period.gross_demand_by_bus_mw[static_cast<size_t>(b)] -
           day_ahead_pricing.load_shedding_mw[static_cast<size_t>(b)]);
      const double exogenous_deviation_mw =
          (realized_period.exogenous_injection_by_bus_mw[static_cast<size_t>(b)] -
           real_time_pricing.exogenous_curtailment_mw[static_cast<size_t>(b)]) -
          (day_ahead_period.exogenous_injection_by_bus_mw[static_cast<size_t>(b)] -
           day_ahead_pricing.exogenous_curtailment_mw[static_cast<size_t>(b)]);
      period.customer_deviation_payment +=
          served_load_deviation_mw * real_time_lmp * dt;
      period.resource_deviation_revenue +=
          exogenous_deviation_mw * real_time_lmp * dt;
      if (ancillary.enabled) {
        const double day_ahead_load_mw =
            day_ahead_period.gross_demand_by_bus_mw[static_cast<size_t>(b)];
        const double tolerance_mw =
            ancillary.load_imbalance_tolerance_fraction *
            std::max(std::abs(day_ahead_load_mw), 1.0);
        const double penalized_imbalance_mw = std::max(
            0.0, std::abs(load_deviation_mw) - tolerance_mw);
        period.load_penalized_imbalance_mwh +=
            penalized_imbalance_mw * dt;
        period.load_imbalance_penalty +=
            penalized_imbalance_mw * dt *
            ancillary.load_imbalance_penalty_per_mwh;
      }
    }
    period.deviation_congestion_rent =
        period.customer_deviation_payment - period.resource_deviation_revenue;
    result.total_absolute_generator_deviation_mwh +=
        period.absolute_generator_deviation_mw * dt;
    result.periods.push_back(period);
  }

  for (auto& settlement : result.generator_deviation_settlement) {
    settlement.reserve_performance_ratio =
        settlement.instructed_reserve_mwh > 1e-12
            ? settlement.delivered_reserve_mwh /
                  settlement.instructed_reserve_mwh
            : 1.0;
    settlement.net_ancillary_adjustment =
        settlement.reserve_performance_payment -
        settlement.reserve_nonperformance_charge -
        settlement.imbalance_charge;
    settlement.two_settlement_revenue =
        settlement.day_ahead_energy_revenue +
        settlement.day_ahead_reserve_revenue +
        settlement.day_ahead_uplift +
        settlement.real_time_deviation_revenue +
        settlement.net_ancillary_adjustment;
    settlement.profit_after_two_settlement =
        settlement.two_settlement_revenue - settlement.actual_true_cost;
  }

  std::unordered_map<std::string, size_t> participant_position;
  result.participant_deviation_settlement.reserve(
      day_ahead_result.participants.size());
  for (const auto& participant : day_ahead_result.participants) {
    ParticipantDeviationSettlement settlement;
    settlement.participant_id = participant.participant_id;
    settlement.participant_name = participant.participant_name;
    settlement.generator_positions = participant.generator_positions;
    participant_position[participant.participant_id] =
        result.participant_deviation_settlement.size();
    result.participant_deviation_settlement.push_back(std::move(settlement));
  }
  for (const auto& generator : result.generator_deviation_settlement) {
    const auto participant_it =
        participant_position.find(generator.participant_id);
    if (participant_it == participant_position.end()) continue;
    auto& participant =
        result.participant_deviation_settlement[participant_it->second];
    participant.day_ahead_energy_mwh += generator.day_ahead_energy_mwh;
    participant.real_time_energy_mwh += generator.real_time_energy_mwh;
    participant.real_time_dispatch_instruction_mwh +=
        generator.real_time_dispatch_instruction_mwh;
    participant.actual_output_deviation_mwh +=
        generator.actual_output_deviation_mwh;
    participant.deviation_mwh += generator.deviation_mwh;
    participant.day_ahead_market_revenue +=
        generator.day_ahead_energy_revenue +
        generator.day_ahead_reserve_revenue +
        generator.day_ahead_uplift;
    participant.real_time_deviation_revenue +=
        generator.real_time_deviation_revenue;
    participant.instructed_reserve_mwh += generator.instructed_reserve_mwh;
    participant.delivered_reserve_mwh += generator.delivered_reserve_mwh;
    participant.reserve_shortfall_mwh += generator.reserve_shortfall_mwh;
    participant.reserve_performance_payment +=
        generator.reserve_performance_payment;
    participant.reserve_nonperformance_charge +=
        generator.reserve_nonperformance_charge;
    participant.penalized_imbalance_mwh +=
        generator.penalized_imbalance_mwh;
    participant.imbalance_charge += generator.imbalance_charge;
    participant.net_ancillary_adjustment +=
        generator.net_ancillary_adjustment;
    participant.two_settlement_revenue += generator.two_settlement_revenue;
    participant.actual_true_cost += generator.actual_true_cost;
    participant.profit_after_two_settlement +=
        generator.profit_after_two_settlement;
  }
  for (auto& participant : result.participant_deviation_settlement) {
    participant.reserve_performance_ratio =
        participant.instructed_reserve_mwh > 1e-12
            ? participant.delivered_reserve_mwh /
                  participant.instructed_reserve_mwh
            : 1.0;
  }

  result.settlement.customer_day_ahead_payment =
      day_ahead_result.settlement.customer_total_payment;
  result.settlement.resource_day_ahead_revenue =
      day_ahead_result.settlement.resource_total_revenue;
  result.settlement.day_ahead_congestion_rent =
      day_ahead_result.settlement.congestion_rent;
  for (const auto& period : result.periods) {
    result.settlement.customer_real_time_deviation_payment +=
        period.customer_deviation_payment;
    result.settlement.resource_real_time_deviation_revenue +=
        period.resource_deviation_revenue;
    result.settlement.real_time_deviation_congestion_rent +=
        period.deviation_congestion_rent;
    result.settlement.customer_imbalance_penalty +=
        period.load_imbalance_penalty;
  }
  for (const auto& generator : result.generator_deviation_settlement) {
    result.settlement.resource_reserve_performance_payment +=
        generator.reserve_performance_payment;
    result.settlement.resource_generator_imbalance_charge +=
        generator.imbalance_charge;
    result.settlement.resource_reserve_nonperformance_charge +=
        generator.reserve_nonperformance_charge;
  }
  result.settlement.customer_two_settlement_payment =
      result.settlement.customer_day_ahead_payment +
      result.settlement.customer_real_time_deviation_payment +
      result.settlement.customer_imbalance_penalty;
  result.settlement.resource_two_settlement_revenue =
      result.settlement.resource_day_ahead_revenue +
      result.settlement.resource_real_time_deviation_revenue +
      result.settlement.resource_reserve_performance_payment -
      result.settlement.resource_generator_imbalance_charge -
      result.settlement.resource_reserve_nonperformance_charge;
  result.settlement.total_congestion_rent =
      result.settlement.day_ahead_congestion_rent +
      result.settlement.real_time_deviation_congestion_rent;
  result.settlement.system_operator_ancillary_balance =
      result.settlement.customer_imbalance_penalty +
      result.settlement.resource_generator_imbalance_charge +
      result.settlement.resource_reserve_nonperformance_charge -
      result.settlement.resource_reserve_performance_payment;
  result.settlement.cashflow_residual =
      result.settlement.customer_two_settlement_payment -
      result.settlement.resource_two_settlement_revenue -
      result.settlement.total_congestion_rent -
      result.settlement.system_operator_ancillary_balance;
  result.feasible = result.real_time_market.feasible;
  return result;
}

RepeatedGameResult run_repeated_market_game(
    const HybridPowerSystem& system,
    const TimeSeriesData& day_ahead_time_series,
    const TimeSeriesData& realized_time_series,
    const RepeatedGameOptions& options) {
  RepeatedGameResult result;
  if (options.max_rounds <= 0) {
    result.status = "invalid_game_rounds";
    return result;
  }

  std::vector<MarketParticipant> participants;
  try {
    participants = submit_participant_offers(
        system, options.day_ahead_options.participants,
        options.day_ahead_options.energy_offer_segments).participants;
  } catch (const std::exception& error) {
    result.status = "game_offer_submission_error";
    result.warnings.push_back(error.what());
    return result;
  }

  struct Evaluation {
    bool usable{false};
    MarketResult day_ahead;
    RealTimeMarketResult real_time;
    std::unordered_map<std::string, double> profit;
    std::unordered_map<std::string, double> ancillary_adjustment;
  };
  const auto evaluate = [&](const std::vector<MarketParticipant>& strategies) {
    Evaluation evaluation;
    MarketOptions day_ahead_options = options.day_ahead_options;
    day_ahead_options.participants = strategies;
    evaluation.day_ahead = run_day_ahead_market(
        system, day_ahead_time_series, day_ahead_options);
    if (!evaluation.day_ahead.feasible ||
        !evaluation.day_ahead.commitment.feasible ||
        evaluation.day_ahead.pricing.empty()) {
      return evaluation;
    }
    if (options.run_real_time) {
      evaluation.real_time = run_real_time_market(
          system, day_ahead_time_series, realized_time_series,
          evaluation.day_ahead, options.real_time_options);
      if (!evaluation.real_time.feasible ||
          evaluation.real_time.real_time_market.pricing.empty()) {
        return evaluation;
      }
      for (const auto& settlement :
           evaluation.real_time.participant_deviation_settlement) {
        evaluation.profit[settlement.participant_id] =
            settlement.profit_after_two_settlement;
        evaluation.ancillary_adjustment[settlement.participant_id] =
            settlement.net_ancillary_adjustment;
      }
    } else {
      for (const auto& settlement : evaluation.day_ahead.participant_settlement) {
        evaluation.profit[settlement.participant_id] =
            settlement.profit_after_uplift;
      }
    }
    evaluation.usable = true;
    return evaluation;
  };

  const auto average_lmp = [](const std::vector<PricingPeriod>& pricing) {
    double total = 0.0;
    size_t count = 0;
    for (const auto& period : pricing) {
      for (double lmp : period.lmp_per_mwh) {
        if (!std::isfinite(lmp)) continue;
        total += lmp;
        ++count;
      }
    }
    return count == 0 ? 0.0 : total / static_cast<double>(count);
  };
  const auto behavior_type = [](double markup, double withholding,
                                double commitment_markup) {
    const bool uses_markup = markup > 1e-12 || commitment_markup > 1e-12;
    const bool uses_withholding = withholding > 1e-12;
    if (uses_markup && uses_withholding) {
      return BehaviorPolicyType::MarkupAndWithholding;
    }
    if (uses_markup) return BehaviorPolicyType::FixedMarkup;
    if (uses_withholding) return BehaviorPolicyType::CapacityWithholding;
    return BehaviorPolicyType::CostBased;
  };

  Evaluation current = evaluate(participants);
  if (!current.usable) {
    result.status = "initial_game_clearing_failed";
    result.final_day_ahead = std::move(current.day_ahead);
    result.final_real_time = std::move(current.real_time);
    return result;
  }

  const double markup_step = std::max(0.0, options.markup_step_fraction);
  const double withholding_step =
      std::max(0.0, options.withholding_step_fraction);
  const double max_markup = std::max(0.0, options.maximum_markup_fraction);
  const double max_withholding = std::clamp(
      options.maximum_withholding_fraction, 0.0, 0.95);
  const double improvement_tolerance =
      std::max(0.0, options.profit_improvement_tolerance);

  for (int round_index = 0; round_index < options.max_rounds; ++round_index) {
    RepeatedGameRound round;
    round.round = round_index;
    round.day_ahead_feasible = current.day_ahead.feasible;
    round.real_time_feasible = !options.run_real_time || current.real_time.feasible;
    round.day_ahead_average_lmp_per_mwh =
        average_lmp(current.day_ahead.pricing);
    round.real_time_average_lmp_per_mwh = options.run_real_time
        ? average_lmp(current.real_time.real_time_market.pricing)
        : 0.0;
    round.total_absolute_generator_deviation_mwh = options.run_real_time
        ? current.real_time.total_absolute_generator_deviation_mwh
        : 0.0;
    round.output_hhi = current.day_ahead.market_power.output_hhi;

    std::vector<MarketParticipant> next_participants = participants;
    bool any_change = false;
    for (size_t p = 0; p < participants.size(); ++p) {
      const auto& participant = participants[p];
      GameParticipantRound participant_round;
      participant_round.participant_id = participant.participant_id;
      participant_round.behavior = participant.behavior;
      participant_round.next_behavior = participant.behavior;
      participant_round.profit = current.profit[participant.participant_id];
      participant_round.net_ancillary_adjustment =
          current.ancillary_adjustment[participant.participant_id];
      participant_round.best_response_profit = participant_round.profit;

      const bool learns = options.include_cost_based_participants ||
          participant.behavior.type != BehaviorPolicyType::CostBased;
      if (learns) {
        std::vector<ParticipantBehavior> candidates;
        const auto add_candidate = [&](double markup, double withholding) {
          ParticipantBehavior candidate = participant.behavior;
          candidate.energy_markup_fraction =
              std::clamp(markup, 0.0, max_markup);
          candidate.capacity_withholding_fraction =
              std::clamp(withholding, 0.0, max_withholding);
          candidate.type = behavior_type(
              candidate.energy_markup_fraction,
              candidate.capacity_withholding_fraction,
              candidate.commitment_markup_fraction);
          const bool duplicate = std::any_of(
              candidates.begin(), candidates.end(), [&](const auto& existing) {
                return existing.type == candidate.type &&
                    std::abs(existing.energy_markup_fraction -
                             candidate.energy_markup_fraction) < 1e-12 &&
                    std::abs(existing.capacity_withholding_fraction -
                             candidate.capacity_withholding_fraction) < 1e-12;
              });
          if (!duplicate) candidates.push_back(candidate);
        };
        add_candidate(participant.behavior.energy_markup_fraction + markup_step,
                      participant.behavior.capacity_withholding_fraction);
        add_candidate(participant.behavior.energy_markup_fraction - markup_step,
                      participant.behavior.capacity_withholding_fraction);
        add_candidate(participant.behavior.energy_markup_fraction,
                      participant.behavior.capacity_withholding_fraction +
                          withholding_step);
        add_candidate(participant.behavior.energy_markup_fraction,
                      participant.behavior.capacity_withholding_fraction -
                          withholding_step);

        for (const auto& candidate : candidates) {
          auto trial_participants = participants;
          trial_participants[p].behavior = candidate;
          Evaluation trial = evaluate(trial_participants);
          if (!trial.usable) continue;
          const double trial_profit =
              trial.profit[participant.participant_id];
          if (trial_profit > participant_round.best_response_profit +
                                 improvement_tolerance) {
            participant_round.best_response_profit = trial_profit;
            participant_round.next_behavior = candidate;
          }
        }
        participant_round.best_response_improvement =
            participant_round.best_response_profit - participant_round.profit;
        participant_round.strategy_changed =
            participant_round.best_response_improvement > improvement_tolerance;
        if (participant_round.strategy_changed) {
          next_participants[p].behavior = participant_round.next_behavior;
          any_change = true;
        }
      }
      round.participants.push_back(std::move(participant_round));
    }
    result.rounds.push_back(std::move(round));

    if (!any_change) {
      result.converged = true;
      break;
    }
    // The last permitted round still has to evaluate profitable deviations so
    // it can distinguish a local equilibrium from an exhausted round budget.
    // Do not apply an un-cleared strategy profile beyond the recorded horizon;
    // the final result remains the market state evaluated in this round.
    if (round_index + 1 >= options.max_rounds) break;
    participants = std::move(next_participants);
    current = evaluate(participants);
    if (!current.usable) {
      result.status = "game_update_clearing_failed";
      result.final_participants = participants;
      result.final_day_ahead = std::move(current.day_ahead);
      result.final_real_time = std::move(current.real_time);
      return result;
    }
  }

  result.final_participants = participants;
  result.final_day_ahead = std::move(current.day_ahead);
  result.final_real_time = std::move(current.real_time);
  result.feasible = result.final_day_ahead.feasible &&
      (!options.run_real_time || result.final_real_time.feasible);
  result.status = result.converged ? "converged" : "maximum_rounds_reached";
  return result;
}

}  // namespace hacdcpf::market
