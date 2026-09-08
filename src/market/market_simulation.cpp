#include "hacdcpf/market/market_simulation.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/Dense>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::market {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLargeBound = 1.0e9;

struct PeriodNetworkData {
  HybridPowerSystem snapshot;
  std::vector<double> gross_demand_by_bus_mw;
  std::vector<double> exogenous_injection_by_bus_mw;
  std::vector<double> net_demand_by_bus_mw;
  std::vector<double> gross_dc_demand_by_bus_mw;
  std::vector<double> exogenous_dc_injection_by_bus_mw;
  std::vector<double> net_dc_demand_by_bus_mw;
};

struct N1SecurityCut {
  int period{0};
  int monitored_active_branch{0};
  int outage_active_branch{0};
  double lodf{0.0};
  double sense{1.0};
  double emergency_rating_mw{0.0};
};

struct ComponentContingency {
  std::string component_type;
  int component_position{-1};
  int component_index{0};
  std::string component_name;
  int bus{0};
  int from_bus{0};
  int to_bus{0};
};

struct LODFModel {
  bool available{false};
  Eigen::MatrixXd values;
  std::vector<int> branch_positions;
  std::vector<int> value_column_by_outage_active_index;
  std::vector<int> valid_outage_active_indices;
  std::vector<int> skipped_islanding_branch_positions;
  std::vector<std::string> warnings;
  long long estimated_sparse_bytes{0};
};

double lodf_value(
    const LODFModel& model, int monitored_active_index,
    int outage_active_index) {
  if (outage_active_index < 0 ||
      outage_active_index >=
          static_cast<int>(model.value_column_by_outage_active_index.size())) {
    throw std::out_of_range("market: LODF outage index is out of range");
  }
  const int column = model.value_column_by_outage_active_index[
      static_cast<size_t>(outage_active_index)];
  if (column < 0 || column >= model.values.cols()) {
    throw std::out_of_range("market: requested LODF column was not computed");
  }
  return model.values(monitored_active_index, column);
}

struct MarketDCStorage {
  bool rich{false};
  int position{-1};
  int index{0};
  int bus{0};
  std::string name;
  double charge_cap_mw{0.0};
  double discharge_cap_mw{0.0};
  double energy_capacity_mwh{0.0};
  double initial_energy_mwh{0.0};
  double minimum_energy_mwh{0.0};
  double maximum_energy_mwh{0.0};
  double eta_charge{1.0};
  double eta_discharge{1.0};
  double retention{1.0};
  double charge_bid_price{0.0};
  double discharge_bid_price{0.0};
  double daily_cycle_limit{0.0};
};

struct PricingBuild {
  engine::LPModel model;
  int G{0};
  int T{0};
  int B{0};
  int L{0};
  int K{0};
  int D{0};
  int M{0};
  int C{0};
  int Q{0};
  int S{0};
  int p_offset{0};
  int reserve_offset{0};
  int theta_offset{0};
  int flow_offset{0};
  int shed_offset{0};
  int curtail_offset{0};
  int segment_offset{0};
  int vdc_offset{0};
  int dc_flow_offset{0};
  int dc_shed_offset{0};
  int dc_curtail_offset{0};
  int vsc_ac_to_dc_offset{0};
  int vsc_dc_to_ac_offset{0};
  int dcdc_forward_offset{0};
  int dcdc_reverse_offset{0};
  int storage_charge_offset{0};
  int storage_discharge_offset{0};
  int storage_energy_offset{0};
  int balance_row_offset{0};
  int flow_row_offset{0};
  int offer_row_offset{0};
  int reserve_row_offset{0};
  int dc_balance_row_offset{0};
  int dc_flow_row_offset{0};
  int vdc_reference_row_offset{0};
  int control_row_offset{0};
  int storage_row_offset{0};
  std::vector<int> generator_positions;
  std::vector<int> branch_positions;
  std::vector<int> dc_branch_positions;
  std::vector<int> vsc_positions;
  std::vector<int> dcdc_positions;
  std::vector<int> dc_reference_buses;
  std::vector<MarketDCStorage> dc_storages;
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
  int vdc(int d, int t) const { return vdc_offset + t * D + d; }
  int dc_flow(int m, int t) const { return dc_flow_offset + m * T + t; }
  int dc_shed(int d, int t) const { return dc_shed_offset + t * D + d; }
  int dc_curtail(int d, int t) const {
    return dc_curtail_offset + t * D + d;
  }
  int vsc_ac_to_dc(int c, int t) const {
    return vsc_ac_to_dc_offset + c * T + t;
  }
  int vsc_dc_to_ac(int c, int t) const {
    return vsc_dc_to_ac_offset + c * T + t;
  }
  int dcdc_forward(int q, int t) const {
    return dcdc_forward_offset + q * T + t;
  }
  int dcdc_reverse(int q, int t) const {
    return dcdc_reverse_offset + q * T + t;
  }
  int storage_charge(int s, int t) const {
    return storage_charge_offset + s * T + t;
  }
  int storage_discharge(int s, int t) const {
    return storage_discharge_offset + s * T + t;
  }
  int storage_energy(int s, int t) const {
    return storage_energy_offset + s * T + t;
  }
  int balance_row(int b, int t) const {
    return balance_row_offset + t * B + b;
  }
  int flow_row(int l, int t) const { return flow_row_offset + t * L + l; }
  int offer_row(int g, int t) const { return offer_row_offset + g * T + t; }
  int reserve_row(int t) const { return reserve_row_offset + t; }
  int dc_balance_row(int d, int t) const {
    return dc_balance_row_offset + t * D + d;
  }
  int dc_flow_row(int m, int t) const {
    return dc_flow_row_offset + t * M + m;
  }
  int vdc_reference_row(int r, int t) const {
    return vdc_reference_row_offset + t * static_cast<int>(dc_reference_buses.size()) + r;
  }
  int storage_row(int s, int t) const {
    return storage_row_offset + s * T + t;
  }
};

struct MarketCommitmentBuild {
  engine::MIPModel model;
  int G{0};
  int T{0};
  int B{0};
  int L{0};
  int K{0};
  int D{0};
  int M{0};
  int C{0};
  int Q{0};
  int S{0};
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
  int vdc_offset{0};
  int dc_flow_offset{0};
  int dc_shed_offset{0};
  int dc_curtail_offset{0};
  int vsc_ac_to_dc_offset{0};
  int vsc_dc_to_ac_offset{0};
  int vsc_direction_offset{0};
  int dcdc_forward_offset{0};
  int dcdc_reverse_offset{0};
  int dcdc_direction_offset{0};
  int storage_charge_offset{0};
  int storage_discharge_offset{0};
  int storage_energy_offset{0};
  int storage_direction_offset{0};
  int reserve_total_offset{0};
  std::vector<int> generator_positions;
  std::vector<int> branch_positions;
  std::vector<int> dc_branch_positions;
  std::vector<int> vsc_positions;
  std::vector<int> dcdc_positions;
  std::vector<int> dc_reference_buses;
  std::vector<MarketDCStorage> dc_storages;
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
  int vdc(int d, int t) const { return vdc_offset + t * D + d; }
  int dc_flow(int m, int t) const { return dc_flow_offset + m * T + t; }
  int dc_shed(int d, int t) const { return dc_shed_offset + t * D + d; }
  int dc_curtail(int d, int t) const {
    return dc_curtail_offset + t * D + d;
  }
  int vsc_ac_to_dc(int c, int t) const {
    return vsc_ac_to_dc_offset + c * T + t;
  }
  int vsc_dc_to_ac(int c, int t) const {
    return vsc_dc_to_ac_offset + c * T + t;
  }
  int vsc_direction(int c, int t) const {
    return vsc_direction_offset + c * T + t;
  }
  int dcdc_forward(int q, int t) const {
    return dcdc_forward_offset + q * T + t;
  }
  int dcdc_reverse(int q, int t) const {
    return dcdc_reverse_offset + q * T + t;
  }
  int dcdc_direction(int q, int t) const {
    return dcdc_direction_offset + q * T + t;
  }
  int storage_charge(int s, int t) const {
    return storage_charge_offset + s * T + t;
  }
  int storage_discharge(int s, int t) const {
    return storage_discharge_offset + s * T + t;
  }
  int storage_energy(int s, int t) const {
    return storage_energy_offset + s * T + t;
  }
  int storage_direction(int s, int t) const {
    return storage_direction_offset + s * T + t;
  }
  int reserve_total(int t) const { return reserve_total_offset + t; }
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

std::unordered_map<int, int> make_dc_bus_position_map(
    const HybridPowerSystem& system) {
  std::unordered_map<int, int> out;
  out.reserve(system.dc.buses.size());
  for (int d = 0; d < static_cast<int>(system.dc.buses.size()); ++d) {
    const int id = system.dc.buses[static_cast<size_t>(d)].index;
    if (!out.emplace(id, d).second) {
      throw std::invalid_argument("market: duplicate DC bus id " +
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

std::vector<int> active_dc_branch_positions(const HybridPowerSystem& system) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(system.dc.branches.size()); ++i) {
    if (system.dc.branches[static_cast<size_t>(i)].in_service) out.push_back(i);
  }
  return out;
}

std::vector<int> active_vsc_positions(const HybridPowerSystem& system) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(system.vsc_converters.size()); ++i) {
    if (system.vsc_converters[static_cast<size_t>(i)].in_service) out.push_back(i);
  }
  return out;
}

std::vector<int> active_dcdc_positions(const HybridPowerSystem& system) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(system.dc.dcdc_converters.size()); ++i) {
    if (system.dc.dcdc_converters[static_cast<size_t>(i)].in_service) out.push_back(i);
  }
  return out;
}

template <typename StorageType>
bool market_optimizes_dc_storage(const StorageType& storage, bool enabled) {
  return enabled && storage.in_service && storage.controllable &&
      storage.cap_charging_strategy != "static" &&
      storage.e_rated_mwh > 1e-9;
}

template <typename StorageType>
MarketDCStorage make_market_dc_storage(const StorageType& storage,
                                        bool rich,
                                        int position) {
  MarketDCStorage out;
  out.rich = rich;
  out.position = position;
  out.index = storage.index;
  out.bus = storage.bus;
  out.name = storage.name;
  out.charge_cap_mw = storage.pmin_mw < -1e-9
      ? -storage.pmin_mw
      : std::max(0.0, storage.p_rated_mw);
  out.discharge_cap_mw = storage.pmax_mw > 1e-9
      ? storage.pmax_mw
      : std::max(0.0, storage.p_rated_mw);
  out.energy_capacity_mwh =
      storage.e_rated_mwh * std::clamp(storage.soh, 0.0, 1.0);
  if (out.energy_capacity_mwh <= 1e-9) {
    out.energy_capacity_mwh = storage.e_rated_mwh;
  }
  out.minimum_energy_mwh = std::clamp(
      storage.soc_min, 0.0, 1.0) * out.energy_capacity_mwh;
  out.maximum_energy_mwh = std::clamp(
      storage.soc_max, 0.0, 1.0) * out.energy_capacity_mwh;
  if (out.maximum_energy_mwh < out.minimum_energy_mwh) {
    std::swap(out.minimum_energy_mwh, out.maximum_energy_mwh);
  }
  const double authored_initial = storage.e_mwh > 1e-9
      ? storage.e_mwh
      : storage.soc_init * out.energy_capacity_mwh;
  out.initial_energy_mwh = std::clamp(
      authored_initial, out.minimum_energy_mwh, out.maximum_energy_mwh);
  out.eta_charge = std::clamp(storage.eta_charge, 1e-6, 1.0);
  out.eta_discharge = std::clamp(storage.eta_discharge, 1e-6, 1.0);
  out.retention = std::clamp(
      1.0 - storage.self_discharge_pct / 100.0, 0.0, 1.0);
  out.charge_bid_price = std::max(0.0, storage.charge_bid_price);
  out.discharge_bid_price = std::max(0.0, storage.discharge_bid_price);
  out.daily_cycle_limit = std::max(0.0, storage.daily_cycle_limit);
  return out;
}

std::vector<MarketDCStorage> active_market_dc_storages(
    const HybridPowerSystem& system,
    bool enabled) {
  std::vector<MarketDCStorage> out;
  for (int i = 0; i < static_cast<int>(system.dc.storage.size()); ++i) {
    const auto& storage = system.dc.storage[static_cast<size_t>(i)];
    if (market_optimizes_dc_storage(storage, enabled)) {
      out.push_back(make_market_dc_storage(storage, false, i));
    }
  }
  for (int i = 0; i < static_cast<int>(system.dc.dc_storage.size()); ++i) {
    const auto& storage = system.dc.dc_storage[static_cast<size_t>(i)];
    if (market_optimizes_dc_storage(storage, enabled)) {
      out.push_back(make_market_dc_storage(storage, true, i));
    }
  }
  return out;
}

std::vector<UnsupportedMarketAsset> unsupported_market_assets(
    const HybridPowerSystem& system) {
  std::vector<UnsupportedMarketAsset> out;
  const auto add = [&](const std::string& type, int position, int index,
                       const std::string& name, int bus, int from_bus,
                       int to_bus, const std::string& reason) {
    out.push_back(UnsupportedMarketAsset{
        type, position, index, name, bus, from_bus, to_bus, reason});
  };
  for (size_t i = 0; i < system.ac.external_grids.size(); ++i) {
    const auto& item = system.ac.external_grids[i];
    if (item.in_service)
      add("external_grid", static_cast<int>(i), item.index, item.name,
          item.bus, 0, 0, "外部电网尚未建模为可报价的市场平衡资源");
  }
  for (size_t i = 0; i < system.energy_routers.size(); ++i) {
    const auto& item = system.energy_routers[i];
    if (item.in_service)
      add("energy_router", static_cast<int>(i), item.index, item.name,
          0, 0, 0, "能量路由器尚未建立市场报价与端口结算契约");
  }
  return out;
}

std::vector<int> dc_reference_bus_positions(
    const HybridPowerSystem& system,
    const std::unordered_map<int, int>& dc_bus_positions) {
  std::vector<int> refs;
  std::unordered_set<int> seen;
  for (int d = 0; d < static_cast<int>(system.dc.buses.size()); ++d) {
    const auto& bus = system.dc.buses[static_cast<size_t>(d)];
    if (bus.in_service && bus.bus_type == DCBusType::DC_V && seen.insert(d).second) {
      refs.push_back(d);
    }
  }
  for (const auto& converter : system.vsc_converters) {
    if (!converter.in_service ||
        !resolve_device_control_role(converter).provides_dc_v_reference) {
      continue;
    }
    const auto found = dc_bus_positions.find(converter.bus_dc);
    if (found != dc_bus_positions.end() && seen.insert(found->second).second) {
      refs.push_back(found->second);
    }
  }
  return refs;
}

std::vector<int> dc_voltage_island_components(
    const HybridPowerSystem& system,
    const std::unordered_map<int, int>& dc_bus_positions,
    int& component_count) {
  const int D = static_cast<int>(system.dc.buses.size());
  component_count = 0;
  std::vector<int> component(static_cast<size_t>(D), -1);
  std::vector<std::vector<int>> adjacency(static_cast<size_t>(D));
  const auto link = [&](int from_bus, int to_bus) {
    const auto from = dc_bus_positions.find(from_bus);
    const auto to = dc_bus_positions.find(to_bus);
    if (from == dc_bus_positions.end() || to == dc_bus_positions.end()) return;
    adjacency[static_cast<size_t>(from->second)].push_back(to->second);
    adjacency[static_cast<size_t>(to->second)].push_back(from->second);
  };
  for (const auto& branch : system.dc.branches) {
    if (branch.in_service) link(branch.from_bus, branch.to_bus);
  }

  for (int start = 0; start < D; ++start) {
    const auto& bus = system.dc.buses[static_cast<size_t>(start)];
    if (!bus.in_service || bus.bus_type == DCBusType::DC_ISOLATED) {
      component[static_cast<size_t>(start)] = -2;
      continue;
    }
    if (component[static_cast<size_t>(start)] >= 0) continue;
    std::vector<int> queue{start};
    component[static_cast<size_t>(start)] = component_count;
    for (size_t head = 0; head < queue.size(); ++head) {
      const int current = queue[head];
      for (int next : adjacency[static_cast<size_t>(current)]) {
        if (component[static_cast<size_t>(next)] >= 0) continue;
        const auto& next_bus = system.dc.buses[static_cast<size_t>(next)];
        if (!next_bus.in_service ||
            next_bus.bus_type == DCBusType::DC_ISOLATED) {
          continue;
        }
        component[static_cast<size_t>(next)] = component_count;
        queue.push_back(next);
      }
    }
    ++component_count;
  }
  return component;
}

std::string dc_reference_coverage_error(
    const HybridPowerSystem& system,
    const std::unordered_map<int, int>& dc_bus_positions,
    const std::vector<int>& explicit_reference_buses,
    const char* model_name) {
  int component_count = 0;
  const auto component = dc_voltage_island_components(
      system, dc_bus_positions, component_count);
  std::vector<bool> has_reference(static_cast<size_t>(component_count), false);
  for (int bus_position : explicit_reference_buses) {
    if (bus_position >= 0 &&
        bus_position < static_cast<int>(component.size())) {
      const int island = component[static_cast<size_t>(bus_position)];
      if (island >= 0) has_reference[static_cast<size_t>(island)] = true;
    }
  }
  // DC/DC voltage and non-zero droop controls anchor the output-side metallic
  // DC island through their own control equation; DC/DC power mode does not.
  for (const auto& converter : system.dc.dcdc_converters) {
    if (!converter.in_service) continue;
    const bool forms_voltage =
        converter.control_mode == DCDCControlMode::Voltage ||
        (converter.control_mode == DCDCControlMode::Droop &&
         std::abs(converter.k_droop) > 1e-12);
    if (!forms_voltage) continue;
    const auto found = dc_bus_positions.find(converter.bus_out);
    if (found == dc_bus_positions.end()) continue;
    const int island = component[static_cast<size_t>(found->second)];
    if (island >= 0) has_reference[static_cast<size_t>(island)] = true;
  }

  for (int island = 0; island < component_count; ++island) {
    if (has_reference[static_cast<size_t>(island)]) continue;
    std::string buses;
    for (int d = 0; d < static_cast<int>(component.size()); ++d) {
      if (component[static_cast<size_t>(d)] != island) continue;
      if (!buses.empty()) buses += ",";
      buses += std::to_string(system.dc.buses[static_cast<size_t>(d)].index);
    }
    return std::string("market: hybrid ") + model_name + " DC island " +
        std::to_string(island) + " (buses " + buses +
        ") requires a DC_V bus, a DC-voltage-forming VSC, or an output-side "
        "voltage/droop-controlled DC/DC converter";
  }
  return {};
}

double dc_reference_setpoint(const HybridPowerSystem& system,
                             int dc_bus_position) {
  const auto& bus = system.dc.buses[static_cast<size_t>(dc_bus_position)];
  double setpoint = bus.vm_pu > 0.0 ? bus.vm_pu : 1.0;
  for (const auto& converter : system.vsc_converters) {
    if (converter.in_service && converter.bus_dc == bus.index &&
        resolve_device_control_role(converter).provides_dc_v_reference) {
      setpoint = converter.v_dc_set_pu > 0.0 ? converter.v_dc_set_pu : setpoint;
      break;
    }
  }
  return setpoint;
}

PeriodNetworkData make_period_data(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const UCSchedule& commitment,
    int period,
    const TimeSeriesPFOptions& uc_options,
    const std::unordered_map<int, int>& bus_positions,
    const std::unordered_map<int, int>& dc_bus_positions,
    bool optimize_dc_storage) {
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

  const int D = static_cast<int>(system.dc.buses.size());
  data.gross_dc_demand_by_bus_mw.assign(static_cast<size_t>(D), 0.0);
  data.exogenous_dc_injection_by_bus_mw.assign(static_cast<size_t>(D), 0.0);
  for (int d = 0; d < D; ++d) {
    const auto& bus = data.snapshot.dc.buses[static_cast<size_t>(d)];
    if (bus.in_service) {
      data.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] +=
          std::max(0.0, bus.pd_mw);
    }
  }
  const auto add_dc_demand = [&](int bus, double mw) {
    const auto found = dc_bus_positions.find(bus);
    if (found != dc_bus_positions.end()) {
      data.gross_dc_demand_by_bus_mw[static_cast<size_t>(found->second)] +=
          std::max(0.0, mw);
    }
  };
  const auto add_dc_injection = [&](int bus, double mw) {
    const auto found = dc_bus_positions.find(bus);
    if (found != dc_bus_positions.end()) {
      data.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(found->second)] += mw;
    }
  };
  for (const auto& load : data.snapshot.dc.loads) {
    if (load.in_service) add_dc_demand(load.bus, load.p_mw * load.scaling);
  }
  for (const auto& source : data.snapshot.dc.static_generators) {
    if (source.in_service) add_dc_injection(source.bus, source.p_mw * source.scaling);
  }
  for (const auto& source : data.snapshot.dc.dc_static_generators) {
    if (source.in_service) add_dc_injection(source.bus, source.p_set_mw * source.scaling);
  }
  for (const auto& source : data.snapshot.dc.pv_arrays) {
    if (source.in_service) add_dc_injection(source.bus, source.p_set_mw);
  }
  for (const auto& storage : data.snapshot.dc.storage) {
    if (storage.in_service &&
        !market_optimizes_dc_storage(storage, optimize_dc_storage)) {
      add_dc_injection(storage.bus, storage.p_mw);
    }
  }
  for (const auto& storage : data.snapshot.dc.dc_storage) {
    if (storage.in_service &&
        !market_optimizes_dc_storage(storage, optimize_dc_storage)) {
      add_dc_injection(storage.bus, storage.p_mw);
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
  data.net_dc_demand_by_bus_mw.resize(static_cast<size_t>(D), 0.0);
  for (int d = 0; d < D; ++d) {
    data.net_dc_demand_by_bus_mw[static_cast<size_t>(d)] =
        data.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] -
        data.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)];
  }
  return data;
}

double effective_converter_efficiency(double eta) {
  return std::clamp(eta, 0.01, 1.0);
}

double dc_branch_rating_mw(const DCBranch& branch,
                           bool enforce_network_constraints) {
  if (!enforce_network_constraints) return kLargeBound;
  if (branch.rate_a_mva > 1e-9) return branch.rate_a_mva;
  if (branch.s_max_mva > 1e-9) return branch.s_max_mva;
  return kLargeBound;
}

std::pair<double, double> vsc_directional_caps(const VSCConverter& converter) {
  const double fallback = std::max(0.0, converter.p_rated_mw);
  const double ac_to_dc = converter.pmin_mw < -1e-9
      ? -converter.pmin_mw
      : fallback;
  const double dc_to_ac_output = converter.pmax_mw > 1e-9
      ? converter.pmax_mw
      : fallback;
  return {ac_to_dc,
          dc_to_ac_output / effective_converter_efficiency(converter.eta)};
}

std::pair<double, double> dcdc_directional_caps(
    const DCDCConverter& converter) {
  const double fallback = std::max(0.0, converter.sn_mva);
  return {converter.pmax_mw > 1e-9 ? converter.pmax_mw : fallback,
          converter.pmin_mw < -1e-9 ? -converter.pmin_mw : fallback};
}

engine::BCOptions market_scuc_bc_options(
    const MarketOptions& market_options) {
  using namespace engine;
  BCOptions options;
  options.time_limit_sec = std::max(0.0, market_options.scuc_time_limit_sec);
  options.gap_tol = std::max(0.0, market_options.scuc_mip_relative_gap);
  options.max_nodes = std::max(1, market_options.scuc_max_nodes);
  options.cuts = CutType::Gomory;
  options.root_cut_rounds = 15;
  options.cuts_per_round = 20;
  options.use_simplex_lp_nodes = true;
  options.use_feasibility_pump = false;
  options.branching = BranchingStrategy::Pseudocost;
  options.node_sel = NodeSelection::Hybrid;
  options.max_lp_iter = 30000;
  options.verbose = false;
  return options;
}

bool market_gurobi_available();

bool market_uses_structured_highs(
    UCSolverChoice choice,
    const MarketOptions& market_options,
    int binary_variables) {
  const bool use_highs = choice == UCSolverChoice::HiGHS ||
      (choice == UCSolverChoice::Auto && !market_gurobi_available());
  return use_highs &&
      market_options.structured_scuc_branching &&
      binary_variables >=
          std::max(0, market_options.structured_scuc_min_binary_variables);
}

bool market_gurobi_available() {
  static const bool available = [] {
    engine::GurobiAdapter adapter;
    return adapter.available() &&
        adapter.supports(engine::ProblemClass::MILP);
  }();
  return available;
}

engine::SolverAdapterPtr create_market_milp_adapter(
    UCSolverChoice choice,
    const MarketOptions& market_options,
    int binary_variables,
    bool* structured_branching_used = nullptr) {
  using namespace engine;
  if (structured_branching_used != nullptr) {
    *structured_branching_used = false;
  }
  const auto native = [&]() -> SolverAdapterPtr {
    return std::make_shared<NativeBranchAndCutAdapter>(
        market_scuc_bc_options(market_options));
  };
  const auto highs = [&]() -> SolverAdapterPtr {
    const bool structured = market_uses_structured_highs(
        choice, market_options, binary_variables);
    if (structured) {
      if (structured_branching_used != nullptr) {
        *structured_branching_used = true;
      }
#if defined(HACDCPF_MIPSOLVERS_HAVE_STRICT_HIGHS_ADAPTER)
      return std::make_shared<StrictHighsBranchAndCutAdapter>(
          market_scuc_bc_options(market_options));
#else
      return std::make_shared<NativeBranchAndCutAdapter>(
          market_scuc_bc_options(market_options));
#endif
    }
    return std::make_shared<StrictHighsBranchAndCutAdapter>(
        market_scuc_bc_options(market_options));
  };
  if (choice == UCSolverChoice::Native) return native();
  if (choice == UCSolverChoice::HiGHS) return highs();
  if (choice == UCSolverChoice::SCIP) {
    auto adapter = std::make_shared<ScipAdapter>();
    if (adapter->available() && adapter->supports(ProblemClass::MILP)) {
      return adapter;
    }
    return native();
  }
  if (choice == UCSolverChoice::Gurobi) {
    auto adapter = std::make_shared<GurobiAdapter>();
    if (adapter->available() && adapter->supports(ProblemClass::MILP)) {
      return adapter;
    }
    return highs();
  }
  if (market_gurobi_available()) {
    return std::make_shared<GurobiAdapter>();
  }
  auto highs_adapter = highs();
  if (highs_adapter->supports(ProblemClass::MILP)) return highs_adapter;
  return native();
}

engine::SolveResult solve_market_milp_with_fallback(
    const engine::MIPModel& model,
    UCSolverChoice choice,
    const MarketOptions& market_options,
    int binary_variables,
    bool* structured_branching_used = nullptr) {
  using namespace engine;
  if (structured_branching_used != nullptr) {
    *structured_branching_used = false;
  }

  if (choice != UCSolverChoice::Auto &&
      choice != UCSolverChoice::Gurobi) {
    auto adapter = create_market_milp_adapter(
        choice, market_options, binary_variables,
        structured_branching_used);
    return adapter->solve_milp(model);
  }

  std::string gurobi_failure = "unavailable or unlicensed";
  GurobiAdapter gurobi;
  if (gurobi.available() && gurobi.supports(ProblemClass::MILP)) {
    auto solved = gurobi.solve_milp(model);
    if (solved.stats.success) return solved;
    gurobi_failure = solved.stats.status;
  }

  bool used_structured = false;
  auto packaged = create_market_milp_adapter(
      UCSolverChoice::HiGHS, market_options, binary_variables,
      &used_structured);
  auto solved = packaged->solve_milp(model);
  if (structured_branching_used != nullptr) {
    *structured_branching_used = used_structured;
  }
  if (solved.stats.success) {
    solved.stats.status += " (fallback after Gurobi: " + gurobi_failure + ")";
    return solved;
  }

  auto native = create_market_milp_adapter(
      UCSolverChoice::Native, market_options, binary_variables);
  auto native_solved = native->solve_milp(model);
  native_solved.stats.status +=
      " (fallback after Gurobi: " + gurobi_failure +
      "; packaged MILP: " + solved.stats.status + ")";
  return native_solved;
}

engine::SolveResult solve_market_pricing_lp(
    const engine::LPModel& model,
    const engine::SimplexOptions& options,
    int native_max_variables,
    bool* fallback_used = nullptr,
    bool* direct_highs_used = nullptr) {
  if (fallback_used != nullptr) *fallback_used = false;
  if (direct_highs_used != nullptr) *direct_highs_used = false;
  engine::HighsAdapter highs;
  const bool prefer_highs = native_max_variables >= 0 &&
      model.c.size() > native_max_variables;
  if (prefer_highs && highs.available() &&
      highs.supports(engine::ProblemClass::LP)) {
    auto direct = highs.solve_lp(model);
    if (direct.stats.success) {
      if (direct_highs_used != nullptr) *direct_highs_used = true;
      direct.stats.status += " (direct large-model route; native simplex skipped)";
      return direct;
    }
  }
  auto native = engine::solve_lp_with_basis(model, options, nullptr).result;
  if (native.stats.success) return native;

  if (!highs.available() || !highs.supports(engine::ProblemClass::LP)) {
    return native;
  }
  auto fallback = highs.solve_lp(model);
  if (!fallback.stats.success) return native;
  if (fallback_used != nullptr) *fallback_used = true;
  fallback.stats.status += " (fallback after native simplex: " +
      native.stats.status + ")";
  return fallback;
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
  build.dc_branch_positions = active_dc_branch_positions(system);
  build.vsc_positions = active_vsc_positions(system);
  build.dcdc_positions = active_dcdc_positions(system);
  build.dc_storages = active_market_dc_storages(
      system, options.optimize_dc_storage);
  build.G = static_cast<int>(build.generator_positions.size());
  build.T = time_series.num_steps;
  build.B = static_cast<int>(system.ac.buses.size());
  build.L = static_cast<int>(build.branch_positions.size());
  build.K = std::max(1, options.energy_offer_segments);
  build.D = static_cast<int>(system.dc.buses.size());
  build.M = static_cast<int>(build.dc_branch_positions.size());
  build.C = static_cast<int>(build.vsc_positions.size());
  build.Q = static_cast<int>(build.dcdc_positions.size());
  build.S = static_cast<int>(build.dc_storages.size());
  if (build.G == 0 || build.B == 0 || build.T <= 0 ||
      static_cast<int>(offers.size()) != build.G) {
    throw std::invalid_argument(
        "market: SCUC requires aligned active generators, offers, buses and periods");
  }

  const auto bus_positions = make_bus_position_map(system);
  const auto dc_bus_positions = make_dc_bus_position_map(system);
  build.dc_reference_buses = dc_reference_bus_positions(system, dc_bus_positions);
  const auto reference_error = dc_reference_coverage_error(
      system, dc_bus_positions, build.dc_reference_buses, "SCUC");
  if (!reference_error.empty()) throw std::invalid_argument(reference_error);
  UCSchedule no_commitment;
  for (int t = 0; t < build.T; ++t) {
    build.periods.push_back(make_period_data(
        system, time_series, no_commitment, t, options.uc_options,
        bus_positions, dc_bus_positions, options.optimize_dc_storage));
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
  build.vdc_offset = build.curtail_offset + build.B * build.T;
  build.dc_flow_offset = build.vdc_offset + build.D * build.T;
  build.dc_shed_offset = build.dc_flow_offset + build.M * build.T;
  build.dc_curtail_offset = build.dc_shed_offset + build.D * build.T;
  build.vsc_ac_to_dc_offset = build.dc_curtail_offset + build.D * build.T;
  build.vsc_dc_to_ac_offset = build.vsc_ac_to_dc_offset + build.C * build.T;
  build.vsc_direction_offset = build.vsc_dc_to_ac_offset + build.C * build.T;
  build.dcdc_forward_offset = build.vsc_direction_offset + build.C * build.T;
  build.dcdc_reverse_offset = build.dcdc_forward_offset + build.Q * build.T;
  build.dcdc_direction_offset = build.dcdc_reverse_offset + build.Q * build.T;
  build.storage_charge_offset =
      build.dcdc_direction_offset + build.Q * build.T;
  build.storage_discharge_offset =
      build.storage_charge_offset + build.S * build.T;
  build.storage_energy_offset =
      build.storage_discharge_offset + build.S * build.T;
  build.storage_direction_offset =
      build.storage_energy_offset + build.S * build.T;
  build.reserve_total_offset =
      build.storage_direction_offset + build.S * build.T;
  const int nvar = build.reserve_total_offset +
      (options.enable_n1_security ? build.T : 0);

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
      // With binary commitment, the transition equality fixes a genuine
      // startup/shutdown to 0/1. Extra simultaneous startup and shutdown can
      // only add non-negative cost and tighten the remaining UC constraints,
      // so these auxiliaries do not need their own integrality declarations.
      model.linear_part.vars[static_cast<size_t>(build.startup(g, t))] = {
          VarType::Continuous, 0.0, 1.0,
          "market_uc_startup_g" + std::to_string(g) + "_t" +
              std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.shutdown(g, t))] = {
          VarType::Continuous, 0.0, 1.0,
          "market_uc_shutdown_g" + std::to_string(g) + "_t" +
              std::to_string(t)};
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
  if (options.enable_n1_security) {
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[
          static_cast<size_t>(build.reserve_total(t))] = {
          VarType::Continuous, 0.0, kLargeBound,
          "market_uc_reserve_total_t" + std::to_string(t)};
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
      const double angle_bound =
          (b == reference_bus ||
           system.ac.buses[static_cast<size_t>(b)].bus_type == BusType::SLACK)
          ? 0.0
          : kPi;
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

  for (int d = 0; d < build.D; ++d) {
    const auto& bus = system.dc.buses[static_cast<size_t>(d)];
    const double v0 = bus.vm_pu > 0.0 ? bus.vm_pu : 1.0;
    const double vmin = bus.in_service && bus.vmin_pu > 0.0 ? bus.vmin_pu : v0;
    const double vmax = bus.in_service && bus.vmax_pu > 0.0 ? bus.vmax_pu : v0;
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.vdc(d, t))] = {
          VarType::Continuous, std::min(vmin, vmax), std::max(vmin, vmax),
          "market_uc_vdc_d" + std::to_string(d) + "_t" + std::to_string(t)};
      const auto& period = build.periods[static_cast<size_t>(t)];
      const double demand = std::max(
          0.0, period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)]);
      const double injection = std::max(
          0.0, period.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)]);
      model.linear_part.vars[static_cast<size_t>(build.dc_shed(d, t))] = {
          VarType::Continuous, 0.0, demand,
          "market_uc_dc_shed_d" + std::to_string(d) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.dc_curtail(d, t))] = {
          VarType::Continuous, 0.0, injection,
          "market_uc_dc_curtail_d" + std::to_string(d) + "_t" + std::to_string(t)};
      model.linear_part.c[build.dc_shed(d, t)] =
          std::max(1.0, options.value_of_lost_load_per_mwh) * dt;
      model.linear_part.c[build.dc_curtail(d, t)] =
          std::max(0.0, options.exogenous_curtailment_penalty_per_mwh) * dt;
    }
  }
  for (int m = 0; m < build.M; ++m) {
    const auto& branch = system.dc.branches[static_cast<size_t>(
        build.dc_branch_positions[static_cast<size_t>(m)])];
    const double limit = dc_branch_rating_mw(
        branch, options.enable_network_constraints);
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.dc_flow(m, t))] = {
          VarType::Continuous, -limit, limit,
          "market_uc_dc_flow_m" + std::to_string(m) + "_t" + std::to_string(t)};
    }
  }
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    const auto [ac_to_dc_cap, dc_to_ac_cap] = vsc_directional_caps(converter);
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.vsc_ac_to_dc(c, t))] = {
          VarType::Continuous, 0.0, ac_to_dc_cap,
          "market_uc_vsc_ad_c" + std::to_string(c) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.vsc_dc_to_ac(c, t))] = {
          VarType::Continuous, 0.0, dc_to_ac_cap,
          "market_uc_vsc_da_c" + std::to_string(c) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.vsc_direction(c, t))] = {
          VarType::Binary, 0.0, 1.0,
          "market_uc_vsc_dir_c" + std::to_string(c) + "_t" + std::to_string(t)};
      model.binary_idx.push_back(build.vsc_direction(c, t));
    }
  }
  for (int q = 0; q < build.Q; ++q) {
    const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
        build.dcdc_positions[static_cast<size_t>(q)])];
    const auto [forward_cap, reverse_cap] = dcdc_directional_caps(converter);
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.dcdc_forward(q, t))] = {
          VarType::Continuous, 0.0, forward_cap,
          "market_uc_dcdc_f_q" + std::to_string(q) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.dcdc_reverse(q, t))] = {
          VarType::Continuous, 0.0, reverse_cap,
          "market_uc_dcdc_r_q" + std::to_string(q) + "_t" + std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.dcdc_direction(q, t))] = {
          VarType::Binary, 0.0, 1.0,
          "market_uc_dcdc_dir_q" + std::to_string(q) + "_t" + std::to_string(t)};
      model.binary_idx.push_back(build.dcdc_direction(q, t));
    }
  }
  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    for (int t = 0; t < build.T; ++t) {
      model.linear_part.vars[static_cast<size_t>(build.storage_charge(s, t))] = {
          VarType::Continuous, 0.0, storage.charge_cap_mw,
          "market_uc_dc_storage_charge_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.storage_discharge(s, t))] = {
          VarType::Continuous, 0.0, storage.discharge_cap_mw,
          "market_uc_dc_storage_discharge_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      double energy_lo = storage.minimum_energy_mwh;
      double energy_hi = storage.maximum_energy_mwh;
      if (options.enforce_terminal_dc_storage_soc && t == build.T - 1) {
        energy_lo = storage.initial_energy_mwh;
        energy_hi = storage.initial_energy_mwh;
      }
      model.linear_part.vars[static_cast<size_t>(build.storage_energy(s, t))] = {
          VarType::Continuous, energy_lo, energy_hi,
          "market_uc_dc_storage_energy_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      model.linear_part.vars[static_cast<size_t>(build.storage_direction(s, t))] = {
          VarType::Binary, 0.0, 1.0,
          "market_uc_dc_storage_charging_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      model.binary_idx.push_back(build.storage_direction(s, t));
      model.linear_part.c[build.storage_charge(s, t)] =
          storage.charge_bid_price * dt;
      model.linear_part.c[build.storage_discharge(s, t)] =
          storage.discharge_bid_price * dt;
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
      for (int c = 0; c < build.C; ++c) {
        const auto& converter = system.vsc_converters[static_cast<size_t>(
            build.vsc_positions[static_cast<size_t>(c)])];
        if (bus_positions.at(converter.bus_ac) != b) continue;
        const double eta = effective_converter_efficiency(converter.eta);
        terms.emplace_back(build.vsc_ac_to_dc(c, t), -1.0);
        terms.emplace_back(build.vsc_dc_to_ac(c, t), eta);
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

  for (int t = 0; t < build.T; ++t) {
    for (int d = 0; d < build.D; ++d) {
      std::vector<std::pair<int, double>> terms = {
          {build.dc_shed(d, t), 1.0},
          {build.dc_curtail(d, t), -1.0}};
      for (int c = 0; c < build.C; ++c) {
        const auto& converter = system.vsc_converters[static_cast<size_t>(
            build.vsc_positions[static_cast<size_t>(c)])];
        if (dc_bus_positions.at(converter.bus_dc) != d) continue;
        const double eta = effective_converter_efficiency(converter.eta);
        terms.emplace_back(build.vsc_ac_to_dc(c, t), eta);
        terms.emplace_back(build.vsc_dc_to_ac(c, t), -1.0);
      }
      for (int q = 0; q < build.Q; ++q) {
        const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
            build.dcdc_positions[static_cast<size_t>(q)])];
        const double eta = effective_converter_efficiency(converter.eta);
        if (dc_bus_positions.at(converter.bus_in) == d) {
          terms.emplace_back(build.dcdc_forward(q, t), -1.0);
          terms.emplace_back(build.dcdc_reverse(q, t), eta);
        }
        if (dc_bus_positions.at(converter.bus_out) == d) {
          terms.emplace_back(build.dcdc_forward(q, t), eta);
          terms.emplace_back(build.dcdc_reverse(q, t), -1.0);
        }
      }
      for (int s = 0; s < build.S; ++s) {
        const auto& storage = build.dc_storages[static_cast<size_t>(s)];
        if (dc_bus_positions.at(storage.bus) != d) continue;
        terms.emplace_back(build.storage_discharge(s, t), 1.0);
        terms.emplace_back(build.storage_charge(s, t), -1.0);
      }
      for (int m = 0; m < build.M; ++m) {
        const auto& branch = system.dc.branches[static_cast<size_t>(
            build.dc_branch_positions[static_cast<size_t>(m)])];
        if (dc_bus_positions.at(branch.from_bus) == d) {
          terms.emplace_back(build.dc_flow(m, t), -1.0);
        }
        if (dc_bus_positions.at(branch.to_bus) == d) {
          terms.emplace_back(build.dc_flow(m, t), 1.0);
        }
      }
      add_eq(terms, build.periods[static_cast<size_t>(t)]
                        .net_dc_demand_by_bus_mw[static_cast<size_t>(d)]);
    }
  }

  const double dc_base_mva =
      std::max({system.base_mva, system.dc.base_mva, 1.0});
  for (int m = 0; m < build.M; ++m) {
    const auto& branch = system.dc.branches[static_cast<size_t>(
        build.dc_branch_positions[static_cast<size_t>(m)])];
    double resistance = branch.r_pu;
    if (std::abs(resistance) < 1e-12) resistance = 1e-6;
    const double coefficient = dc_base_mva / resistance;
    const int from = dc_bus_positions.at(branch.from_bus);
    const int to = dc_bus_positions.at(branch.to_bus);
    for (int t = 0; t < build.T; ++t) {
      add_eq({{build.dc_flow(m, t), 1.0},
              {build.vdc(from, t), -coefficient},
              {build.vdc(to, t), coefficient}},
             0.0);
    }
  }
  for (int r = 0; r < static_cast<int>(build.dc_reference_buses.size()); ++r) {
    const int d = build.dc_reference_buses[static_cast<size_t>(r)];
    const double setpoint = dc_reference_setpoint(system, d);
    for (int t = 0; t < build.T; ++t) {
      add_eq({{build.vdc(d, t), 1.0}}, setpoint);
    }
  }
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    const auto [ac_to_dc_cap, dc_to_ac_cap] = vsc_directional_caps(converter);
    const double eta = effective_converter_efficiency(converter.eta);
    for (int t = 0; t < build.T; ++t) {
      add_le({{build.vsc_ac_to_dc(c, t), 1.0},
              {build.vsc_direction(c, t), -ac_to_dc_cap}},
             0.0);
      add_le({{build.vsc_dc_to_ac(c, t), 1.0},
              {build.vsc_direction(c, t), dc_to_ac_cap}},
             dc_to_ac_cap);
      if (!converter.controllable || converter.p_is_hard_constraint) {
        add_eq({{build.vsc_ac_to_dc(c, t), -1.0},
                {build.vsc_dc_to_ac(c, t), eta}},
               converter.p_set_mw);
      }
    }
  }
  for (int q = 0; q < build.Q; ++q) {
    const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
        build.dcdc_positions[static_cast<size_t>(q)])];
    const auto [forward_cap, reverse_cap] = dcdc_directional_caps(converter);
    const int output_bus = dc_bus_positions.at(converter.bus_out);
    for (int t = 0; t < build.T; ++t) {
      add_le({{build.dcdc_forward(q, t), 1.0},
              {build.dcdc_direction(q, t), -forward_cap}},
             0.0);
      add_le({{build.dcdc_reverse(q, t), 1.0},
              {build.dcdc_direction(q, t), reverse_cap}},
             reverse_cap);
      if (!converter.controllable ||
          converter.control_mode == DCDCControlMode::Power) {
        add_eq({{build.dcdc_forward(q, t), 1.0},
                {build.dcdc_reverse(q, t), -1.0}},
               converter.p_ref_mw);
      } else if (converter.control_mode == DCDCControlMode::Voltage) {
        add_eq({{build.vdc(output_bus, t), 1.0}}, converter.v_ref_pu);
      } else if (converter.control_mode == DCDCControlMode::Droop) {
        add_eq({{build.dcdc_forward(q, t), 1.0},
                {build.dcdc_reverse(q, t), -1.0},
                {build.vdc(output_bus, t), -converter.k_droop}},
               converter.p_ref_mw - converter.k_droop * converter.v_ref_pu);
      }
    }
  }
  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    for (int t = 0; t < build.T; ++t) {
      add_le({{build.storage_charge(s, t), 1.0},
              {build.storage_direction(s, t), -storage.charge_cap_mw}},
             0.0);
      add_le({{build.storage_discharge(s, t), 1.0},
              {build.storage_direction(s, t), storage.discharge_cap_mw}},
             storage.discharge_cap_mw);
      std::vector<std::pair<int, double>> energy = {
          {build.storage_energy(s, t), 1.0},
          {build.storage_charge(s, t), -storage.eta_charge * dt},
          {build.storage_discharge(s, t), dt / storage.eta_discharge}};
      double rhs = storage.retention * storage.initial_energy_mwh;
      if (t > 0) {
        energy.emplace_back(
            build.storage_energy(s, t - 1), -storage.retention);
        rhs = 0.0;
      }
      add_eq(energy, rhs);
    }
    if (storage.daily_cycle_limit > 1e-9) {
      const int periods_per_day = std::max(
          1, static_cast<int>(std::lround(24.0 / dt)));
      for (int day_start = 0; day_start < build.T;
           day_start += periods_per_day) {
        std::vector<std::pair<int, double>> throughput;
        const int day_end = std::min(build.T, day_start + periods_per_day);
        for (int t = day_start; t < day_end; ++t) {
          throughput.emplace_back(build.storage_charge(s, t), dt);
          throughput.emplace_back(build.storage_discharge(s, t), dt);
        }
        add_le(throughput, 2.0 * storage.energy_capacity_mwh *
                               storage.daily_cycle_limit);
      }
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
    const double gross_dc_demand = std::accumulate(
        build.periods[static_cast<size_t>(t)].gross_dc_demand_by_bus_mw.begin(),
        build.periods[static_cast<size_t>(t)].gross_dc_demand_by_bus_mw.end(), 0.0);
    add_le(reserve_terms,
           -std::max(0.0, options.upward_reserve_fraction) *
               (gross_demand + gross_dc_demand));
    if (options.enable_n1_security) {
      std::vector<std::pair<int, double>> reserve_total_definition = {
          {build.reserve_total(t), 1.0}};
      for (int g = 0; g < build.G; ++g) {
        reserve_total_definition.emplace_back(build.reserve(g, t), -1.0);
      }
      add_eq(reserve_total_definition, 0.0);
      for (int outage = 0; outage < build.G; ++outage) {
        add_le({{build.p(outage, t), 1.0},
                {build.reserve(outage, t), 1.0},
                {build.reserve_total(t), -1.0}},
               0.0);
      }
    }
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

  engine::MIPModel::UCGenHint hint;
  hint.ng = build.G;
  hint.T = build.T;
  hint.period_hours = dt;
  hint.pg_start = build.p_offset;
  hint.ig_start = build.commitment_offset;
  hint.su_start = build.startup_offset;
  hint.sd_start = build.shutdown_offset;
  const int commitment_entries = build.G * build.T;
  hint.ig_cols.assign(static_cast<size_t>(commitment_entries), -1);
  hint.su_cols.assign(static_cast<size_t>(commitment_entries), -1);
  hint.sd_cols.assign(static_cast<size_t>(commitment_entries), -1);
  hint.pg_cols.assign(static_cast<size_t>(commitment_entries), -1);
  hint.min_up.resize(static_cast<size_t>(build.G), 0);
  hint.min_down.resize(static_cast<size_t>(build.G), 0);
  hint.ig0.resize(static_cast<size_t>(build.G), 0);
  hint.pmin.resize(static_cast<size_t>(build.G), 0.0);
  hint.pmax.resize(static_cast<size_t>(build.G), 0.0);
  hint.ramp.resize(static_cast<size_t>(build.G), kLargeBound);
  hint.gen_bus.resize(static_cast<size_t>(build.G), -1);
#if defined(HACDCPF_MIPSOLVERS_HAVE_EXTENDED_SCUC_HINTS)
  hint.up_reserve_headroom_cap.resize(static_cast<size_t>(build.G), 0.0);
#endif
  for (int g = 0; g < build.G; ++g) {
    const int position = build.generator_positions[static_cast<size_t>(g)];
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    const auto& offer = offers[static_cast<size_t>(g)];
    hint.min_up[static_cast<size_t>(g)] = static_cast<int>(std::ceil(
        std::max(0.0, generator.min_up_time_hr) / dt));
    hint.min_down[static_cast<size_t>(g)] = static_cast<int>(std::ceil(
        std::max(0.0, generator.min_dn_time_hr) / dt));
    hint.ig0[static_cast<size_t>(g)] = generator.pg_mw > 1e-6 ? 1 : 0;
    hint.pmin[static_cast<size_t>(g)] =
        std::max(0.0, offer.minimum_output_mw);
    hint.pmax[static_cast<size_t>(g)] =
        std::max(hint.pmin[static_cast<size_t>(g)],
                 offer.offered_maximum_output_mw);
    const double ramp_up = generator.ramp_up_mw_min > 0.0
        ? generator.ramp_up_mw_min * 60.0 * dt
        : kLargeBound;
    const double ramp_down = generator.ramp_dn_mw_min > 0.0
        ? generator.ramp_dn_mw_min * 60.0 * dt
        : kLargeBound;
    hint.ramp[static_cast<size_t>(g)] = std::max(ramp_up, ramp_down);
    const auto bus = bus_positions.find(generator.bus);
    if (bus != bus_positions.end()) hint.gen_bus[static_cast<size_t>(g)] = bus->second;
#if defined(HACDCPF_MIPSOLVERS_HAVE_EXTENDED_SCUC_HINTS)
    hint.up_reserve_headroom_cap[static_cast<size_t>(g)] =
        std::max(0.0, hint.pmax[static_cast<size_t>(g)] -
                          hint.pmin[static_cast<size_t>(g)]);
#endif
    for (int t = 0; t < build.T; ++t) {
      const size_t slot = static_cast<size_t>(t * build.G + g);
      hint.ig_cols[slot] = build.commitment(g, t);
      hint.su_cols[slot] = build.startup(g, t);
      hint.sd_cols[slot] = build.shutdown(g, t);
      hint.pg_cols[slot] = build.p(g, t);
    }
  }
  hint.demand.resize(static_cast<size_t>(build.T), 0.0);
  hint.reserve_requirement.resize(static_cast<size_t>(build.T), 0.0);
  for (int t = 0; t < build.T; ++t) {
    const auto& period = build.periods[static_cast<size_t>(t)];
    const double ac_demand = std::accumulate(
        period.gross_demand_by_bus_mw.begin(),
        period.gross_demand_by_bus_mw.end(), 0.0);
    const double dc_demand = std::accumulate(
        period.gross_dc_demand_by_bus_mw.begin(),
        period.gross_dc_demand_by_bus_mw.end(), 0.0);
    hint.demand[static_cast<size_t>(t)] = ac_demand + dc_demand;
    hint.reserve_requirement[static_cast<size_t>(t)] =
        std::max(0.0, options.upward_reserve_fraction) *
        (ac_demand + dc_demand);
  }
  hint.n_segments = build.K;
  hint.segment_cols.assign(
      static_cast<size_t>(build.G * build.T * build.K), -1);
  hint.segment_cap.assign(static_cast<size_t>(build.G * build.K), 0.0);
  for (int g = 0; g < build.G; ++g) {
    for (int k = 0; k < build.K; ++k) {
      hint.segment_cap[static_cast<size_t>(g * build.K + k)] =
          std::max(0.0, offers[static_cast<size_t>(g)]
                            .energy_segments[static_cast<size_t>(k)]
                            .quantity_mw);
      for (int t = 0; t < build.T; ++t) {
        const size_t slot = static_cast<size_t>(
            (t * build.G + g) * build.K + k);
        hint.segment_cols[slot] = build.segment(g, t, k);
      }
    }
  }
  hint.certifies_power_balance_rows = true;
  hint.certifies_generation_capacity_rows = true;
#if defined(HACDCPF_MIPSOLVERS_HAVE_EXTENDED_SCUC_HINTS)
  hint.certifies_ramping_rows = true;
#endif
  hint.certifies_min_up_down_rows = true;
  hint.certifies_segment_bound_rows = true;
  hint.certifies_system_reserve_rows = true;
  model.uc_hint = std::move(hint);

  model.branching_priority.assign(static_cast<size_t>(nvar), 0);
  const int time_stride = build.T + 1;
  for (int g = 0; g < build.G; ++g) {
    const double pmax = offers[static_cast<size_t>(g)].offered_maximum_output_mw;
    const int capacity_class = pmax > 600.0 ? 6
        : pmax > 500.0 ? 5
        : pmax > 400.0 ? 4
        : pmax > 250.0 ? 3
        : pmax > 150.0 ? 2
        : pmax > 100.0 ? 1
        : 0;
    for (int t = 0; t < build.T; ++t) {
      model.branching_priority[
          static_cast<size_t>(build.commitment(g, t))] =
          (3 + capacity_class) * time_stride + (build.T - t);
    }
  }
  for (int c = 0; c < build.C; ++c) {
    for (int t = 0; t < build.T; ++t) {
      model.branching_priority[
          static_cast<size_t>(build.vsc_direction(c, t))] =
          time_stride + (build.T - t);
    }
  }
  for (int q = 0; q < build.Q; ++q) {
    for (int t = 0; t < build.T; ++t) {
      model.branching_priority[
          static_cast<size_t>(build.dcdc_direction(q, t))] =
          time_stride + (build.T - t);
    }
  }
  for (int s = 0; s < build.S; ++s) {
    for (int t = 0; t < build.T; ++t) {
      model.branching_priority[
          static_cast<size_t>(build.storage_direction(s, t))] =
          time_stride + (build.T - t);
    }
  }
  return build;
}

Eigen::VectorXd build_market_scuc_warm_start(
    const MarketCommitmentBuild& build,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options) {
  engine::LPModel relaxation = build.model.linear_part;
  for (auto& variable : relaxation.vars) {
    variable.type = engine::VarType::Continuous;
  }
  engine::HighsAdapter highs;
  if (!highs.available() || !highs.supports(engine::ProblemClass::LP)) {
    return {};
  }
  const auto relaxed = highs.solve_lp(relaxation);
  if (!relaxed.stats.success ||
      relaxed.x.size() != relaxation.c.size()) {
    return {};
  }
  const Eigen::VectorXd& relaxed_x = relaxed.x;

  std::vector<int> merit(static_cast<size_t>(build.G));
  std::iota(merit.begin(), merit.end(), 0);
  std::stable_sort(merit.begin(), merit.end(), [&](int lhs, int rhs) {
    const auto offer_cost = [](const GeneratorOffer& offer) {
      double quantity = 0.0;
      double cost = 0.0;
      for (const auto& segment : offer.energy_segments) {
        quantity += std::max(0.0, segment.quantity_mw);
        cost += std::max(0.0, segment.quantity_mw) * segment.price_per_mwh;
      }
      return quantity > 1e-9 ? cost / quantity : kLargeBound;
    };
    return offer_cost(offers[static_cast<size_t>(lhs)]) <
        offer_cost(offers[static_cast<size_t>(rhs)]);
  });

  const auto try_threshold = [&](double threshold) -> Eigen::VectorXd {
    Eigen::VectorXd candidate = relaxed_x;
    for (int index : build.model.binary_idx) {
      candidate[index] = candidate[index] >= 0.5 ? 1.0 : 0.0;
    }
    for (int g = 0; g < build.G; ++g) {
      for (int t = 0; t < build.T; ++t) {
        candidate[build.commitment(g, t)] =
            relaxed_x[build.commitment(g, t)] >= threshold ? 1.0 : 0.0;
      }
    }

    for (int t = 0; t < build.T; ++t) {
      const auto& period = build.periods[static_cast<size_t>(t)];
      const double gross_demand = std::accumulate(
          period.gross_demand_by_bus_mw.begin(),
          period.gross_demand_by_bus_mw.end(), 0.0) +
          std::accumulate(period.gross_dc_demand_by_bus_mw.begin(),
                          period.gross_dc_demand_by_bus_mw.end(), 0.0);
      const double reserve =
          std::max(0.0, options.upward_reserve_fraction) * gross_demand;
      auto committed_capacity = [&]() {
        double total = 0.0;
        double largest = 0.0;
        for (int g = 0; g < build.G; ++g) {
          if (candidate[build.commitment(g, t)] < 0.5) continue;
          const double pmax = std::max(
              0.0, offers[static_cast<size_t>(g)].offered_maximum_output_mw);
          total += pmax;
          largest = std::max(largest, pmax);
        }
        return std::pair<double, double>{total, largest};
      };
      auto [capacity, largest] = committed_capacity();
      std::vector<int> activation_order = merit;
      std::stable_sort(
          activation_order.begin(), activation_order.end(),
          [&](int lhs, int rhs) {
            return relaxed_x[build.commitment(lhs, t)] >
                relaxed_x[build.commitment(rhs, t)];
          });
      for (int g : activation_order) {
        const double required = gross_demand + reserve +
            (options.enable_n1_security ? largest : 0.0);
        if (capacity >= required) break;
        if (candidate[build.commitment(g, t)] >= 0.5) continue;
        candidate[build.commitment(g, t)] = 1.0;
        const double pmax = std::max(
            0.0, offers[static_cast<size_t>(g)].offered_maximum_output_mw);
        capacity += pmax;
        largest = std::max(largest, pmax);
      }
    }

    engine::LPModel fixed = build.model.linear_part;
    for (auto& variable : fixed.vars) {
      variable.type = engine::VarType::Continuous;
    }
    for (int index : build.model.binary_idx) {
      const double value = candidate[index] >= 0.5 ? 1.0 : 0.0;
      fixed.vars[static_cast<size_t>(index)].lb = value;
      fixed.vars[static_cast<size_t>(index)].ub = value;
    }
    const auto repaired = highs.solve_lp(fixed);
    if (repaired.stats.success && repaired.x.size() == fixed.c.size()) {
      return repaired.x;
    }
    return {};
  };

  for (double threshold : {0.5, 0.75, 0.9, 0.95, 0.25, 0.05}) {
    auto repaired = try_threshold(threshold);
    if (repaired.size() == relaxation.c.size()) return repaired;
  }
  return {};
}

Eigen::VectorXd repair_market_scuc_fixed_integers(
    const MarketCommitmentBuild& build,
    const Eigen::VectorXd& candidate) {
  if (candidate.size() != build.model.linear_part.c.size()) return {};
  engine::LPModel fixed = build.model.linear_part;
  for (auto& variable : fixed.vars) {
    variable.type = engine::VarType::Continuous;
  }
  for (int index : build.model.binary_idx) {
    const double value = candidate[index] >= 0.5 ? 1.0 : 0.0;
    fixed.vars[static_cast<size_t>(index)].lb = value;
    fixed.vars[static_cast<size_t>(index)].ub = value;
  }
  engine::HighsAdapter highs;
  if (!highs.available() || !highs.supports(engine::ProblemClass::LP)) {
    return {};
  }
  const auto repaired = highs.solve_lp(fixed);
  if (!repaired.stats.success || repaired.x.size() != fixed.c.size()) {
    return {};
  }
  return repaired.x;
}

struct SCUCNetworkLimitCandidate {
  int flow_variable{-1};
  double lower{-kLargeBound};
  double upper{kLargeBound};
  int from_angle_variable{-1};
  int to_angle_variable{-1};
  double angle_coefficient{0.0};
  double angle_lower{-kLargeBound};
  double angle_upper{kLargeBound};
  bool activated{false};
};

struct SCUCNetworkLimitViolation {
  int candidate_index{-1};
  double violation_mw{0.0};
};

#if defined(HACDCPF_MIPSOLVERS_HAVE_SCUC_CROSS_ROUND_STATE)
struct SCUCCrossRoundSolverState {
  std::shared_ptr<engine::BCRootCuts> root_cuts;
  std::shared_ptr<engine::BCRootBasis> root_basis;
  std::shared_ptr<engine::BCPseudocostInit> pseudocosts;
};
#endif

UCSchedule solve_market_commitment(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options) {
  auto build = build_market_commitment_model(
      system, time_series, offers, options);
  const auto solve_started = std::chrono::steady_clock::now();
  std::vector<SCUCNetworkLimitCandidate> network_candidates;
  if (options.enable_network_constraints) {
    network_candidates.reserve(static_cast<size_t>(build.L * build.T));
    const auto bus_positions = make_bus_position_map(system);
    const double base_mva =
        std::max({system.base_mva, system.ac.base_mva, 1.0});
    for (int l = 0; l < build.L; ++l) {
      const auto& branch = system.ac.branches[static_cast<size_t>(
          build.branch_positions[static_cast<size_t>(l)])];
      double x = branch.x_pu;
      if (std::abs(x) < 1e-12) {
        x = (x < 0.0 ? -1.0 : 1.0) * 1e-6;
      }
      const double tap = std::abs(branch.tap) > 1e-12 ? branch.tap : 1.0;
      const double coefficient = base_mva / (x * tap);
      const double shift = branch.shift_deg * kPi / 180.0;
      const int from = bus_positions.at(branch.from_bus);
      const int to = bus_positions.at(branch.to_bus);
      for (int t = 0; t < build.T; ++t) {
        const int variable = build.flow(l, t);
        const auto& metadata =
            build.model.linear_part.vars[static_cast<size_t>(variable)];
        if (metadata.lb <= -0.5 * kLargeBound &&
            metadata.ub >= 0.5 * kLargeBound) {
          continue;
        }
        network_candidates.push_back(SCUCNetworkLimitCandidate{
            variable,
            metadata.lb,
            metadata.ub,
            build.theta(from, t),
            build.theta(to, t),
            coefficient,
            metadata.lb + coefficient * shift,
            metadata.ub + coefficient * shift,
            false});
      }
    }
  }
  const bool generate_network_constraints =
      options.enable_scuc_network_constraint_generation &&
      static_cast<int>(network_candidates.size()) >= std::max(
          0, options.scuc_network_constraint_generation_min_candidates);
  if (generate_network_constraints) {
    for (const auto& candidate : network_candidates) {
      auto& metadata = build.model.linear_part.vars[
          static_cast<size_t>(candidate.flow_variable)];
      metadata.lb = -kLargeBound;
      metadata.ub = kLargeBound;
    }
  }

  const int max_network_iterations = std::max(
      1, options.scuc_network_constraint_generation_max_iterations);
  const auto find_network_violations = [&](
      const Eigen::VectorXd& solution,
      double* worst_violation) {
    std::vector<SCUCNetworkLimitViolation> violations;
    if (worst_violation != nullptr) *worst_violation = 0.0;
    for (int index = 0;
         index < static_cast<int>(network_candidates.size()); ++index) {
      const auto& candidate = network_candidates[static_cast<size_t>(index)];
      const double flow = solution[candidate.flow_variable];
      const double violation = std::max(
          {0.0, candidate.lower - flow, flow - candidate.upper});
      if (violation <= std::max(
              0.0, options.scuc_network_constraint_generation_tolerance_mw)) {
        continue;
      }
      if (worst_violation != nullptr) {
        *worst_violation = std::max(*worst_violation, violation);
      }
      violations.push_back(SCUCNetworkLimitViolation{index, violation});
    }
    std::stable_sort(
        violations.begin(), violations.end(),
        [](const auto& lhs, const auto& rhs) {
          return lhs.violation_mw > rhs.violation_mw;
        });
    return violations;
  };
  const auto activate_network_limits = [&](
      const std::vector<SCUCNetworkLimitViolation>& violations) {
    int added = 0;
    const int maximum_new =
        options.scuc_network_constraint_generation_max_new_per_iteration;
    for (const auto& violation : violations) {
      auto& candidate = network_candidates[
          static_cast<size_t>(violation.candidate_index)];
      if (candidate.activated) continue;
      if (maximum_new > 0 && added >= maximum_new) break;
      auto& metadata = build.model.linear_part.vars[
          static_cast<size_t>(candidate.flow_variable)];
      metadata.lb = candidate.lower;
      metadata.ub = candidate.upper;
      candidate.activated = true;
      ++added;
    }
    return added;
  };

  int network_activated = 0;

  const auto warm_start_started = std::chrono::steady_clock::now();
  if (build.T > 1) {
    build.model.initial_solution = build_market_scuc_warm_start(
        build, offers, options);
    if (generate_network_constraints &&
        build.model.initial_solution.size() ==
            build.model.linear_part.c.size()) {
      double warm_start_worst_violation = 0.0;
      const auto warm_start_violations = find_network_violations(
          build.model.initial_solution, &warm_start_worst_violation);
      const int warm_start_limits =
          activate_network_limits(warm_start_violations);
      network_activated += warm_start_limits;
      if (warm_start_limits > 0) {
        build.model.initial_solution = repair_market_scuc_fixed_integers(
            build, build.model.initial_solution);
      }
    }
  }
  const double warm_start_generation_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - warm_start_started).count();
  bool structured_branching_used = false;
  bool any_mip_start =
      build.model.initial_solution.size() == build.model.linear_part.c.size();
  bool network_converged = !generate_network_constraints;
  int network_iterations = 0;
  int network_remaining = 0;
  double network_worst_violation = 0.0;
#if defined(HACDCPF_MIPSOLVERS_HAVE_SCUC_CROSS_ROUND_STATE)
  SCUCCrossRoundSolverState reusable_state;
#endif
  bool solver_state_reuse_used = false;
  int solver_state_reuse_rounds = 0;
  bool root_cuts_reused = false;
  int root_cuts_reused_count = 0;
  bool root_basis_reused = false;
  bool pseudocosts_reused = false;
  const bool enable_in_solve_generation =
      generate_network_constraints &&
      options.enable_scuc_in_solve_network_constraint_generation;
  std::vector<bool> in_solve_constraint_submitted(
      network_candidates.size(), false);
  std::mutex in_solve_constraint_mutex;
  int in_solve_callback_calls = 0;
  int in_solve_constraints_submitted = 0;
  engine::SolveResult solved;
  for (int iteration = 1;
       iteration <= (generate_network_constraints ? max_network_iterations : 1);
       ++iteration) {
    MarketOptions iteration_options = options;
    if (options.scuc_time_limit_sec > 0.0) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - solve_started).count();
      iteration_options.scuc_time_limit_sec =
          std::max(0.0, options.scuc_time_limit_sec - elapsed);
      if (iteration_options.scuc_time_limit_sec <= 1e-6) break;
    }
    const int binary_variables =
        static_cast<int>(build.model.binary_idx.size());
    const bool iteration_structured = market_uses_structured_highs(
        options.uc_options.uc_solver, iteration_options, binary_variables);
    if (iteration_structured) {
#if defined(HACDCPF_MIPSOLVERS_HAVE_STRICT_HIGHS_ADAPTER)
      auto bc_options = engine::make_strict_highs_problem_options(
          build.model, market_scuc_bc_options(iteration_options));
#else
      auto bc_options = market_scuc_bc_options(iteration_options);
      bc_options.use_vendored_highs_lp_kernel = true;
#endif
      bool reused_this_round = false;
#if defined(HACDCPF_MIPSOLVERS_HAVE_SCUC_CROSS_ROUND_STATE)
      if (iteration > 1 &&
          options.enable_scuc_cross_round_solver_state_reuse) {
        const int num_columns =
            static_cast<int>(build.model.linear_part.vars.size());
        const int original_rows = static_cast<int>(
            build.model.linear_part.A.rows() +
            build.model.linear_part.Aeq.rows());
        if (reusable_state.root_cuts &&
            !reusable_state.root_cuts->empty()) {
          bc_options.highs_root_cut_warm_start = reusable_state.root_cuts;
          root_cuts_reused = true;
          root_cuts_reused_count = std::max(
              root_cuts_reused_count,
              reusable_state.root_cuts->numCuts());
          reused_this_round = true;
          if (reusable_state.root_basis &&
              !reusable_state.root_basis->empty() &&
              static_cast<int>(
                  reusable_state.root_basis->col_status.size()) ==
                  num_columns &&
              static_cast<int>(
                  reusable_state.root_basis->row_status.size()) ==
                  original_rows + reusable_state.root_cuts->numCuts()) {
            bc_options.highs_root_basis_warm_start =
                reusable_state.root_basis;
            root_basis_reused = true;
          }
        }
        if (reusable_state.pseudocosts &&
            !reusable_state.pseudocosts->empty() &&
            reusable_state.pseudocosts->n_orig_cols == num_columns) {
          bc_options.highs_pseudocost_warm_start =
              reusable_state.pseudocosts;
          pseudocosts_reused = true;
          reused_this_round = true;
        }
      }
#endif
      if (reused_this_round) {
        solver_state_reuse_used = true;
        ++solver_state_reuse_rounds;
      }
      engine::BCCallbacks callbacks;
      if (enable_in_solve_generation) {
        callbacks.policy_tag = "market-scuc-exact-network-limits";
        callbacks.dynamic_node_cut = [&, maximum_new =
            options.scuc_network_constraint_generation_max_new_per_iteration](
                const engine::BCDynamicNodeCutContext& context,
                std::vector<engine::BCDynamicNodeCut>& cuts) {
          std::lock_guard<std::mutex> lock(in_solve_constraint_mutex);
          ++in_solve_callback_calls;
          if (context.original_col_value.empty()) return;
          std::vector<SCUCNetworkLimitViolation> violations;
          violations.reserve(network_candidates.size());
          for (int index = 0;
               index < static_cast<int>(network_candidates.size()); ++index) {
            if (in_solve_constraint_submitted[static_cast<size_t>(index)]) {
              continue;
            }
            const auto& candidate =
                network_candidates[static_cast<size_t>(index)];
            if (candidate.activated) continue;
            if (candidate.flow_variable < 0 ||
                candidate.flow_variable >=
                    static_cast<int>(context.original_col_value.size())) {
              continue;
            }
            const double flow = context.original_col_value[
                static_cast<size_t>(candidate.flow_variable)];
            if (!std::isfinite(flow)) continue;
            const double violation = std::max(
                {0.0, candidate.lower - flow, flow - candidate.upper});
            if (violation <= std::max(
                    0.0,
                    options.scuc_network_constraint_generation_tolerance_mw)) {
              continue;
            }
            violations.push_back({index, violation});
          }
          std::stable_sort(
              violations.begin(), violations.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.violation_mw > rhs.violation_mw;
              });
          for (const auto& violation : violations) {
            if (maximum_new > 0 &&
                static_cast<int>(cuts.size()) >= maximum_new) {
              break;
            }
            const auto& candidate = network_candidates[
                static_cast<size_t>(violation.candidate_index)];
            engine::BCDynamicNodeCut cut;
            cut.indices = {candidate.from_angle_variable,
                           candidate.to_angle_variable};
            cut.values = {candidate.angle_coefficient,
                          -candidate.angle_coefficient};
            cut.lower = candidate.angle_lower;
            cut.upper = candidate.angle_upper;
            cut.column_space =
                engine::BCDynamicNodeCut::ColumnSpace::Original;
            cut.validity_scope = engine::ValidityScope::GlobalCut;
            cut.propagate = true;
            cut.audit_family = "market_ac_thermal_limit";
            cut.audit_key = "flow_" +
                std::to_string(candidate.flow_variable);
            cuts.push_back(std::move(cut));
            in_solve_constraint_submitted[
                static_cast<size_t>(violation.candidate_index)] = true;
            ++in_solve_constraints_submitted;
          }
        };
      }
      engine::BCResult bc_result;
      if (callbacks.dynamic_node_cut) {
        bc_result = engine::solve_milp_bc(
            build.model, bc_options, engine::BCWarmStart{}, callbacks);
      } else {
        bc_result = engine::solve_milp_bc(build.model, bc_options);
      }
      solved.x = std::move(bc_result.x);
      solved.stats = std::move(bc_result.stats);
      solved.stats.solver_name = "StrictHiGHS";
#if defined(HACDCPF_MIPSOLVERS_HAVE_SCUC_CROSS_ROUND_STATE)
      if (bc_result.highs_root_cuts &&
          !bc_result.highs_root_cuts->empty()) {
        reusable_state.root_cuts = std::move(bc_result.highs_root_cuts);
        reusable_state.root_basis = std::move(bc_result.highs_root_basis);
      }
      if (bc_result.highs_pseudocost_init &&
          !bc_result.highs_pseudocost_init->empty()) {
        reusable_state.pseudocosts =
            std::move(bc_result.highs_pseudocost_init);
      }
#endif
    } else {
      bool adapter_structured = false;
      solved = solve_market_milp_with_fallback(
          build.model,
          options.uc_options.uc_solver, iteration_options,
          binary_variables, &adapter_structured);
    }
    structured_branching_used =
        structured_branching_used || iteration_structured;
    ++network_iterations;
    const bool has_full_solution =
        solved.x.size() >= build.model.linear_part.c.size();
    std::vector<SCUCNetworkLimitViolation> violations;
    if (generate_network_constraints && has_full_solution) {
      violations = find_network_violations(
          solved.x, &network_worst_violation);
      network_remaining = static_cast<int>(violations.size());
      network_converged = violations.empty();
    }
    if (!solved.stats.success || !has_full_solution) break;
    if (!generate_network_constraints || network_converged) break;

    int restored_callback_limits = 0;
    for (int index = 0;
         index < static_cast<int>(network_candidates.size()); ++index) {
      if (!in_solve_constraint_submitted[static_cast<size_t>(index)]) continue;
      auto& candidate = network_candidates[static_cast<size_t>(index)];
      if (candidate.activated) continue;
      auto& metadata = build.model.linear_part.vars[
          static_cast<size_t>(candidate.flow_variable)];
      metadata.lb = candidate.lower;
      metadata.ub = candidate.upper;
      candidate.activated = true;
      ++restored_callback_limits;
    }
    network_activated += restored_callback_limits;
    const int added = activate_network_limits(violations);
    network_activated += added;
    if (added == 0 && restored_callback_limits == 0) break;
    build.model.initial_solution = repair_market_scuc_fixed_integers(
        build, solved.x);
    any_mip_start = any_mip_start ||
        build.model.initial_solution.size() ==
            build.model.linear_part.c.size();
  }
  UCSchedule schedule;
  schedule.solver_name = solved.stats.solver_name;
  schedule.solver_status = solved.stats.status;
  schedule.mip_gap = solved.stats.mip_gap;
  schedule.mip_gap_target_met =
      solved.stats.status.find("Optimal") != std::string::npos ||
      solved.stats.status.find("optimal") != std::string::npos;
  schedule.optimality_proven =
      schedule.mip_gap_target_met && solved.stats.mip_gap <= 1e-9;
  schedule.structured_branching_used = structured_branching_used;
  schedule.mip_start_provided = any_mip_start;
  schedule.uc_structure_hint_provided = build.model.uc_hint.has_value();
  schedule.branching_priorities_provided =
      build.model.branching_priority.size() ==
      build.model.linear_part.vars.size();
  schedule.warm_start_generation_sec = warm_start_generation_sec;
#if defined(HACDCPF_MIPSOLVERS_HAVE_SCUC_CROSS_ROUND_STATE)
  schedule.cross_round_solver_state_reuse_enabled =
      options.enable_scuc_cross_round_solver_state_reuse;
#else
  schedule.cross_round_solver_state_reuse_enabled = false;
#endif
  schedule.cross_round_solver_state_reuse_used = solver_state_reuse_used;
  schedule.cross_round_solver_state_reuse_rounds =
      solver_state_reuse_rounds;
  schedule.root_cuts_reused = root_cuts_reused;
  schedule.root_cuts_reused_count = root_cuts_reused_count;
  schedule.root_basis_reused = root_basis_reused;
  schedule.pseudocosts_reused = pseudocosts_reused;
  schedule.search_tree_rebuilt = generate_network_constraints &&
      network_iterations > 1;
  schedule.in_solve_network_constraint_generation_used =
      enable_in_solve_generation && structured_branching_used;
  schedule.in_solve_network_constraint_callback_calls =
      in_solve_callback_calls;
  schedule.in_solve_network_constraints_submitted =
      in_solve_constraints_submitted;
  schedule.network_constraint_generation_run = generate_network_constraints;
  schedule.network_constraint_generation_converged = network_converged;
  schedule.network_constraint_generation_iterations = network_iterations;
  schedule.network_constraint_candidates =
      static_cast<int>(network_candidates.size());
  schedule.network_constraints_activated = network_activated;
  schedule.network_constraint_remaining_violations = network_remaining;
  schedule.network_constraint_worst_violation_mw = network_worst_violation;
  schedule.feasible = solved.stats.success &&
      solved.x.size() >= build.model.linear_part.c.size() &&
      network_converged;
  if (!network_converged && generate_network_constraints) {
    schedule.solver_status +=
        " (network constraint generation incomplete)";
  }
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
  schedule.vsc_dispatch.assign(
      static_cast<size_t>(build.C),
      std::vector<double>(static_cast<size_t>(build.T), 0.0));
  schedule.vsc_direction_ac_to_dc.assign(
      static_cast<size_t>(build.C),
      std::vector<int>(static_cast<size_t>(build.T), 0));
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    const double eta = effective_converter_efficiency(converter.eta);
    for (int t = 0; t < build.T; ++t) {
      schedule.vsc_dispatch[static_cast<size_t>(c)][static_cast<size_t>(t)] =
          -solved.x[build.vsc_ac_to_dc(c, t)] +
          eta * solved.x[build.vsc_dc_to_ac(c, t)];
      schedule.vsc_direction_ac_to_dc[static_cast<size_t>(c)]
                                             [static_cast<size_t>(t)] =
          solved.x[build.vsc_direction(c, t)] > 0.5 ? 1 : 0;
    }
  }
  schedule.dcdc_dispatch.assign(
      static_cast<size_t>(build.Q),
      std::vector<double>(static_cast<size_t>(build.T), 0.0));
  schedule.dcdc_direction_forward.assign(
      static_cast<size_t>(build.Q),
      std::vector<int>(static_cast<size_t>(build.T), 0));
  for (int q = 0; q < build.Q; ++q) {
    for (int t = 0; t < build.T; ++t) {
      schedule.dcdc_dispatch[static_cast<size_t>(q)][static_cast<size_t>(t)] =
          solved.x[build.dcdc_forward(q, t)] -
          solved.x[build.dcdc_reverse(q, t)];
      schedule.dcdc_direction_forward[static_cast<size_t>(q)]
                                             [static_cast<size_t>(t)] =
          solved.x[build.dcdc_direction(q, t)] > 0.5 ? 1 : 0;
    }
  }
  schedule.market_dc_storage_dispatch_mw.assign(
      static_cast<size_t>(build.S),
      std::vector<double>(static_cast<size_t>(build.T), 0.0));
  schedule.market_dc_storage_soc_mwh.assign(
      static_cast<size_t>(build.S),
      std::vector<double>(static_cast<size_t>(build.T), 0.0));
  schedule.market_dc_storage_direction_charging.assign(
      static_cast<size_t>(build.S),
      std::vector<int>(static_cast<size_t>(build.T), 0));
  for (int s = 0; s < build.S; ++s) {
    for (int t = 0; t < build.T; ++t) {
      schedule.market_dc_storage_dispatch_mw[static_cast<size_t>(s)]
          [static_cast<size_t>(t)] =
          solved.x[build.storage_discharge(s, t)] -
          solved.x[build.storage_charge(s, t)];
      schedule.market_dc_storage_soc_mwh[static_cast<size_t>(s)]
          [static_cast<size_t>(t)] = solved.x[build.storage_energy(s, t)];
      schedule.market_dc_storage_direction_charging[static_cast<size_t>(s)]
          [static_cast<size_t>(t)] =
          solved.x[build.storage_direction(s, t)] > 0.5 ? 1 : 0;
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
  build.dc_branch_positions = active_dc_branch_positions(system);
  build.vsc_positions = active_vsc_positions(system);
  build.dcdc_positions = active_dcdc_positions(system);
  build.dc_storages = active_market_dc_storages(
      system, options.optimize_dc_storage);
  build.G = static_cast<int>(build.generator_positions.size());
  build.T = time_series.num_steps;
  build.B = static_cast<int>(system.ac.buses.size());
  build.L = static_cast<int>(build.branch_positions.size());
  build.K = std::max(1, options.energy_offer_segments);
  build.D = static_cast<int>(system.dc.buses.size());
  build.M = static_cast<int>(build.dc_branch_positions.size());
  build.C = static_cast<int>(build.vsc_positions.size());
  build.Q = static_cast<int>(build.dcdc_positions.size());
  build.S = static_cast<int>(build.dc_storages.size());

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
  const auto dc_bus_positions = make_dc_bus_position_map(system);
  build.dc_reference_buses = dc_reference_bus_positions(system, dc_bus_positions);
  const auto reference_error = dc_reference_coverage_error(
      system, dc_bus_positions, build.dc_reference_buses, "SCED");
  if (!reference_error.empty()) throw std::invalid_argument(reference_error);
  build.periods.reserve(static_cast<size_t>(build.T));
  build.reserve_requirements_mw.resize(static_cast<size_t>(build.T), 0.0);
  for (int t = 0; t < build.T; ++t) {
    build.periods.push_back(make_period_data(
        system, time_series, commitment, t, options.uc_options, bus_positions,
        dc_bus_positions, options.optimize_dc_storage));
    const double demand = std::accumulate(
        build.periods.back().gross_demand_by_bus_mw.begin(),
        build.periods.back().gross_demand_by_bus_mw.end(), 0.0);
    const double dc_demand = std::accumulate(
        build.periods.back().gross_dc_demand_by_bus_mw.begin(),
        build.periods.back().gross_dc_demand_by_bus_mw.end(), 0.0);
    build.reserve_requirements_mw[static_cast<size_t>(t)] =
        std::max(0.0, options.upward_reserve_fraction) *
        (demand + dc_demand);
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
  build.vdc_offset = build.segment_offset + nSegment;
  build.dc_flow_offset = build.vdc_offset + build.D * build.T;
  build.dc_shed_offset = build.dc_flow_offset + build.M * build.T;
  build.dc_curtail_offset = build.dc_shed_offset + build.D * build.T;
  build.vsc_ac_to_dc_offset = build.dc_curtail_offset + build.D * build.T;
  build.vsc_dc_to_ac_offset = build.vsc_ac_to_dc_offset + build.C * build.T;
  build.dcdc_forward_offset = build.vsc_dc_to_ac_offset + build.C * build.T;
  build.dcdc_reverse_offset = build.dcdc_forward_offset + build.Q * build.T;
  build.storage_charge_offset =
      build.dcdc_reverse_offset + build.Q * build.T;
  build.storage_discharge_offset =
      build.storage_charge_offset + build.S * build.T;
  build.storage_energy_offset =
      build.storage_discharge_offset + build.S * build.T;
  const int nvar = build.storage_energy_offset + build.S * build.T;

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
      const double bound =
          (b == reference_bus ||
           system.ac.buses[static_cast<size_t>(b)].bus_type == BusType::SLACK)
          ? 0.0
          : kPi;
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
  for (int d = 0; d < build.D; ++d) {
    const auto& bus = system.dc.buses[static_cast<size_t>(d)];
    const double v0 = bus.vm_pu > 0.0 ? bus.vm_pu : 1.0;
    const double vmin = bus.in_service && bus.vmin_pu > 0.0 ? bus.vmin_pu : v0;
    const double vmax = bus.in_service && bus.vmax_pu > 0.0 ? bus.vmax_pu : v0;
    for (int t = 0; t < build.T; ++t) {
      build.model.vars[static_cast<size_t>(build.vdc(d, t))] = {
          engine::VarType::Continuous, std::min(vmin, vmax), std::max(vmin, vmax),
          "market_vdc_d" + std::to_string(d) + "_t" + std::to_string(t)};
      const auto& period = build.periods[static_cast<size_t>(t)];
      const double demand = std::max(
          0.0, period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)]);
      const double injection = std::max(
          0.0, period.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)]);
      build.model.vars[static_cast<size_t>(build.dc_shed(d, t))] = {
          engine::VarType::Continuous, 0.0, demand,
          "market_dc_shed_d" + std::to_string(d) + "_t" + std::to_string(t)};
      build.model.vars[static_cast<size_t>(build.dc_curtail(d, t))] = {
          engine::VarType::Continuous, 0.0, injection,
          "market_dc_curtail_d" + std::to_string(d) + "_t" + std::to_string(t)};
      build.model.c[build.dc_shed(d, t)] =
          std::max(1.0, options.value_of_lost_load_per_mwh) * dt;
      build.model.c[build.dc_curtail(d, t)] =
          std::max(0.0, options.exogenous_curtailment_penalty_per_mwh) * dt;
    }
  }
  for (int m = 0; m < build.M; ++m) {
    const auto& branch = system.dc.branches[static_cast<size_t>(
        build.dc_branch_positions[static_cast<size_t>(m)])];
    const double limit = dc_branch_rating_mw(
        branch, options.enable_network_constraints);
    for (int t = 0; t < build.T; ++t) {
      build.model.vars[static_cast<size_t>(build.dc_flow(m, t))] = {
          engine::VarType::Continuous, -limit, limit,
          "market_dc_flow_m" + std::to_string(m) + "_t" + std::to_string(t)};
    }
  }
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    const auto [ac_to_dc_cap, dc_to_ac_cap] = vsc_directional_caps(converter);
    for (int t = 0; t < build.T; ++t) {
      const bool ac_to_dc = c < static_cast<int>(
              commitment.vsc_direction_ac_to_dc.size()) &&
          t < static_cast<int>(commitment.vsc_direction_ac_to_dc[
              static_cast<size_t>(c)].size())
          ? commitment.vsc_direction_ac_to_dc[static_cast<size_t>(c)]
                                                   [static_cast<size_t>(t)] != 0
          : true;
      build.model.vars[static_cast<size_t>(build.vsc_ac_to_dc(c, t))] = {
          engine::VarType::Continuous, 0.0, ac_to_dc ? ac_to_dc_cap : 0.0,
          "market_vsc_ad_c" + std::to_string(c) + "_t" + std::to_string(t)};
      build.model.vars[static_cast<size_t>(build.vsc_dc_to_ac(c, t))] = {
          engine::VarType::Continuous, 0.0, ac_to_dc ? 0.0 : dc_to_ac_cap,
          "market_vsc_da_c" + std::to_string(c) + "_t" + std::to_string(t)};
    }
  }
  for (int q = 0; q < build.Q; ++q) {
    const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
        build.dcdc_positions[static_cast<size_t>(q)])];
    const auto [forward_cap, reverse_cap] = dcdc_directional_caps(converter);
    for (int t = 0; t < build.T; ++t) {
      const bool forward = q < static_cast<int>(
              commitment.dcdc_direction_forward.size()) &&
          t < static_cast<int>(commitment.dcdc_direction_forward[
              static_cast<size_t>(q)].size())
          ? commitment.dcdc_direction_forward[static_cast<size_t>(q)]
                                              [static_cast<size_t>(t)] != 0
          : true;
      build.model.vars[static_cast<size_t>(build.dcdc_forward(q, t))] = {
          engine::VarType::Continuous, 0.0, forward ? forward_cap : 0.0,
          "market_dcdc_f_q" + std::to_string(q) + "_t" + std::to_string(t)};
      build.model.vars[static_cast<size_t>(build.dcdc_reverse(q, t))] = {
          engine::VarType::Continuous, 0.0, forward ? 0.0 : reverse_cap,
          "market_dcdc_r_q" + std::to_string(q) + "_t" + std::to_string(t)};
    }
  }
  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    for (int t = 0; t < build.T; ++t) {
      const bool charging =
          s < static_cast<int>(
                  commitment.market_dc_storage_direction_charging.size()) &&
          t < static_cast<int>(
                  commitment.market_dc_storage_direction_charging[
                      static_cast<size_t>(s)].size())
          ? commitment.market_dc_storage_direction_charging[
                static_cast<size_t>(s)][static_cast<size_t>(t)] != 0
          : false;
      build.model.vars[static_cast<size_t>(build.storage_charge(s, t))] = {
          engine::VarType::Continuous, 0.0,
          charging ? storage.charge_cap_mw : 0.0,
          "market_dc_storage_charge_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      build.model.vars[static_cast<size_t>(build.storage_discharge(s, t))] = {
          engine::VarType::Continuous, 0.0,
          charging ? 0.0 : storage.discharge_cap_mw,
          "market_dc_storage_discharge_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      double energy_lo = storage.minimum_energy_mwh;
      double energy_hi = storage.maximum_energy_mwh;
      if (options.enforce_terminal_dc_storage_soc && t == build.T - 1) {
        energy_lo = storage.initial_energy_mwh;
        energy_hi = storage.initial_energy_mwh;
      }
      build.model.vars[static_cast<size_t>(build.storage_energy(s, t))] = {
          engine::VarType::Continuous, energy_lo, energy_hi,
          "market_dc_storage_energy_s" + std::to_string(s) + "_t" +
              std::to_string(t)};
      build.model.c[build.storage_charge(s, t)] =
          storage.charge_bid_price * dt;
      build.model.c[build.storage_discharge(s, t)] =
          storage.discharge_bid_price * dt;
    }
  }

  build.balance_row_offset = 0;
  build.flow_row_offset = build.balance_row_offset + build.B * build.T;
  build.offer_row_offset = build.flow_row_offset + build.L * build.T;
  build.reserve_row_offset = build.offer_row_offset + build.G * build.T;
  build.dc_balance_row_offset = build.reserve_row_offset + build.T;
  build.dc_flow_row_offset = build.dc_balance_row_offset + build.D * build.T;
  build.vdc_reference_row_offset = build.dc_flow_row_offset + build.M * build.T;
  build.control_row_offset = build.vdc_reference_row_offset +
      static_cast<int>(build.dc_reference_buses.size()) * build.T;
  int controlled_vsc = 0;
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    if (!converter.controllable || converter.p_is_hard_constraint) {
      ++controlled_vsc;
    }
  }
  build.storage_row_offset = build.control_row_offset +
      (controlled_vsc + build.Q) * build.T;
  const int neq = build.storage_row_offset + build.S * build.T;
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
    for (int d = 0; d < build.D; ++d) {
      const int row = build.dc_balance_row(d, t);
      build.model.beq[row] = build.periods[static_cast<size_t>(t)]
                                  .net_dc_demand_by_bus_mw[static_cast<size_t>(d)];
      eq.emplace_back(row, build.dc_shed(d, t), 1.0);
      eq.emplace_back(row, build.dc_curtail(d, t), -1.0);
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
    for (int c = 0; c < build.C; ++c) {
      const auto& converter = system.vsc_converters[static_cast<size_t>(
          build.vsc_positions[static_cast<size_t>(c)])];
      const double eta = effective_converter_efficiency(converter.eta);
      const int ac_bus = bus_positions.at(converter.bus_ac);
      const int dc_bus = dc_bus_positions.at(converter.bus_dc);
      eq.emplace_back(build.balance_row(ac_bus, t),
                      build.vsc_ac_to_dc(c, t), -1.0);
      eq.emplace_back(build.balance_row(ac_bus, t),
                      build.vsc_dc_to_ac(c, t), eta);
      eq.emplace_back(build.dc_balance_row(dc_bus, t),
                      build.vsc_ac_to_dc(c, t), eta);
      eq.emplace_back(build.dc_balance_row(dc_bus, t),
                      build.vsc_dc_to_ac(c, t), -1.0);
    }
    for (int q = 0; q < build.Q; ++q) {
      const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
          build.dcdc_positions[static_cast<size_t>(q)])];
      const double eta = effective_converter_efficiency(converter.eta);
      const int input_bus = dc_bus_positions.at(converter.bus_in);
      const int output_bus = dc_bus_positions.at(converter.bus_out);
      eq.emplace_back(build.dc_balance_row(input_bus, t),
                      build.dcdc_forward(q, t), -1.0);
      eq.emplace_back(build.dc_balance_row(input_bus, t),
                      build.dcdc_reverse(q, t), eta);
      eq.emplace_back(build.dc_balance_row(output_bus, t),
                      build.dcdc_forward(q, t), eta);
      eq.emplace_back(build.dc_balance_row(output_bus, t),
                      build.dcdc_reverse(q, t), -1.0);
    }
    for (int s = 0; s < build.S; ++s) {
      const auto& storage = build.dc_storages[static_cast<size_t>(s)];
      const int dc_bus = dc_bus_positions.at(storage.bus);
      eq.emplace_back(build.dc_balance_row(dc_bus, t),
                      build.storage_discharge(s, t), 1.0);
      eq.emplace_back(build.dc_balance_row(dc_bus, t),
                      build.storage_charge(s, t), -1.0);
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

  const double dc_base_mva =
      std::max({system.base_mva, system.dc.base_mva, 1.0});
  for (int m = 0; m < build.M; ++m) {
    const auto& branch = system.dc.branches[static_cast<size_t>(
        build.dc_branch_positions[static_cast<size_t>(m)])];
    const int from = dc_bus_positions.at(branch.from_bus);
    const int to = dc_bus_positions.at(branch.to_bus);
    double resistance = branch.r_pu;
    if (std::abs(resistance) < 1e-12) resistance = 1e-6;
    const double coefficient = dc_base_mva / resistance;
    for (int t = 0; t < build.T; ++t) {
      const int flow_var = build.dc_flow(m, t);
      eq.emplace_back(build.dc_balance_row(from, t), flow_var, -1.0);
      eq.emplace_back(build.dc_balance_row(to, t), flow_var, 1.0);
      const int row = build.dc_flow_row(m, t);
      eq.emplace_back(row, flow_var, 1.0);
      eq.emplace_back(row, build.vdc(from, t), -coefficient);
      eq.emplace_back(row, build.vdc(to, t), coefficient);
    }
  }
  for (int r = 0; r < static_cast<int>(build.dc_reference_buses.size()); ++r) {
    const int d = build.dc_reference_buses[static_cast<size_t>(r)];
    const double setpoint = dc_reference_setpoint(system, d);
    for (int t = 0; t < build.T; ++t) {
      const int row = build.vdc_reference_row(r, t);
      eq.emplace_back(row, build.vdc(d, t), 1.0);
      build.model.beq[row] = setpoint;
    }
  }
  int control_row = build.control_row_offset;
  for (int c = 0; c < build.C; ++c) {
    const auto& converter = system.vsc_converters[static_cast<size_t>(
        build.vsc_positions[static_cast<size_t>(c)])];
    if (converter.controllable && !converter.p_is_hard_constraint) continue;
    const double eta = effective_converter_efficiency(converter.eta);
    for (int t = 0; t < build.T; ++t) {
      eq.emplace_back(control_row, build.vsc_ac_to_dc(c, t), -1.0);
      eq.emplace_back(control_row, build.vsc_dc_to_ac(c, t), eta);
      build.model.beq[control_row] = converter.p_set_mw;
      ++control_row;
    }
  }
  for (int q = 0; q < build.Q; ++q) {
    const auto& converter = system.dc.dcdc_converters[static_cast<size_t>(
        build.dcdc_positions[static_cast<size_t>(q)])];
    const int output_bus = dc_bus_positions.at(converter.bus_out);
    for (int t = 0; t < build.T; ++t) {
      if (!converter.controllable ||
          converter.control_mode == DCDCControlMode::Power) {
        eq.emplace_back(control_row, build.dcdc_forward(q, t), 1.0);
        eq.emplace_back(control_row, build.dcdc_reverse(q, t), -1.0);
        build.model.beq[control_row] = converter.p_ref_mw;
      } else if (converter.control_mode == DCDCControlMode::Voltage) {
        eq.emplace_back(control_row, build.vdc(output_bus, t), 1.0);
        build.model.beq[control_row] = converter.v_ref_pu;
      } else {
        eq.emplace_back(control_row, build.dcdc_forward(q, t), 1.0);
        eq.emplace_back(control_row, build.dcdc_reverse(q, t), -1.0);
        eq.emplace_back(control_row, build.vdc(output_bus, t),
                        -converter.k_droop);
        build.model.beq[control_row] =
            converter.p_ref_mw - converter.k_droop * converter.v_ref_pu;
      }
      ++control_row;
    }
  }
  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    for (int t = 0; t < build.T; ++t) {
      const int row = build.storage_row(s, t);
      eq.emplace_back(row, build.storage_energy(s, t), 1.0);
      eq.emplace_back(row, build.storage_charge(s, t),
                      -storage.eta_charge * dt);
      eq.emplace_back(row, build.storage_discharge(s, t),
                      dt / storage.eta_discharge);
      if (t == 0) {
        build.model.beq[row] =
            storage.retention * storage.initial_energy_mwh;
      } else {
        eq.emplace_back(row, build.storage_energy(s, t - 1),
                        -storage.retention);
      }
    }
  }

  build.model.Aeq.resize(neq, nvar);
  build.model.Aeq.setFromTriplets(eq.begin(), eq.end());
  build.model.Aeq.makeCompressed();

  std::vector<Triplet> ineq;
  std::vector<double> rhs;
  const auto append_le = [&](const auto& terms, double bound) {
    const int row = static_cast<int>(rhs.size());
    for (const auto& [column, value] : terms) ineq.emplace_back(row, column, value);
    rhs.push_back(bound);
  };
  const auto add_le = [&](std::initializer_list<std::pair<int, double>> terms,
                          double bound) { append_le(terms, bound); };

  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    if (storage.daily_cycle_limit <= 1e-9) continue;
    const int periods_per_day = std::max(
        1, static_cast<int>(std::lround(24.0 / dt)));
    for (int day_start = 0; day_start < build.T;
         day_start += periods_per_day) {
      std::vector<std::pair<int, double>> throughput;
      const int day_end = std::min(build.T, day_start + periods_per_day);
      for (int t = day_start; t < day_end; ++t) {
        throughput.emplace_back(build.storage_charge(s, t), dt);
        throughput.emplace_back(build.storage_discharge(s, t), dt);
      }
      append_le(throughput, 2.0 * storage.energy_capacity_mwh *
                                storage.daily_cycle_limit);
    }
  }

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

std::vector<int> rank_n1_contingencies(
    const std::vector<int>& branch_positions,
    const std::vector<PricingPeriod>& pricing) {
  std::vector<int> candidates(branch_positions.size());
  std::iota(candidates.begin(), candidates.end(), 0);
  std::stable_sort(candidates.begin(), candidates.end(), [&](int lhs, int rhs) {
    const int lhs_position = branch_positions[static_cast<size_t>(lhs)];
    const int rhs_position = branch_positions[static_cast<size_t>(rhs)];
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
  return candidates;
}

LODFModel build_lodf_model(
    const HybridPowerSystem& system,
    const std::vector<int>& ranked_candidate_active_indices,
    int maximum_valid_contingencies) {
  LODFModel out;
  out.branch_positions = active_branch_positions(system);
  const int B = static_cast<int>(system.ac.buses.size());
  const int L = static_cast<int>(out.branch_positions.size());
  out.value_column_by_outage_active_index.assign(static_cast<size_t>(L), -1);
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
  Eigen::VectorXd susceptance = Eigen::VectorXd::Zero(L);
  std::vector<int> from_positions(static_cast<size_t>(L), -1);
  std::vector<int> to_positions(static_cast<size_t>(L), -1);
  std::vector<Eigen::Triplet<double>> bbus_triplets;
  bbus_triplets.reserve(static_cast<size_t>(4 * L));
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
    from_positions[static_cast<size_t>(l)] = from->second;
    to_positions[static_cast<size_t>(l)] = to->second;
    const int reduced_from = from->second < reference_bus
        ? from->second
        : from->second - 1;
    const int reduced_to = to->second < reference_bus
        ? to->second
        : to->second - 1;
    if (from->second != reference_bus) {
      bbus_triplets.emplace_back(reduced_from, reduced_from, susceptance[l]);
    }
    if (to->second != reference_bus) {
      bbus_triplets.emplace_back(reduced_to, reduced_to, susceptance[l]);
    }
    if (from->second != reference_bus && to->second != reference_bus) {
      bbus_triplets.emplace_back(reduced_from, reduced_to, -susceptance[l]);
      bbus_triplets.emplace_back(reduced_to, reduced_from, -susceptance[l]);
    }
    has_phase_shift = has_phase_shift || std::abs(branch.shift_deg) > 1e-10;
  }
  if (has_phase_shift) {
    out.warnings.push_back(
        "LODF sensitivities ignore fixed phase-shift injections; base flows still retain them.");
  }
  Eigen::SparseMatrix<double> reduced(B - 1, B - 1);
  reduced.setFromTriplets(bbus_triplets.begin(), bbus_triplets.end());
  reduced.makeCompressed();
  Eigen::SparseLU<
      Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> factor;
  factor.analyzePattern(reduced);
  factor.factorize(reduced);
  if (factor.info() != Eigen::Success) {
    out.warnings.push_back("Base AC network is disconnected; LODF matrix is unavailable.");
    return out;
  }

  std::vector<Eigen::VectorXd> columns;
  columns.reserve(maximum_valid_contingencies > 0
                      ? static_cast<size_t>(std::min(L, maximum_valid_contingencies))
                      : static_cast<size_t>(L));
  for (int outage : ranked_candidate_active_indices) {
    if (outage < 0 || outage >= L) continue;
    if (maximum_valid_contingencies > 0 &&
        static_cast<int>(columns.size()) >= maximum_valid_contingencies) {
      break;
    }
    Eigen::VectorXd reduced_transfer = Eigen::VectorXd::Zero(B - 1);
    const int outage_from = from_positions[static_cast<size_t>(outage)];
    const int outage_to = to_positions[static_cast<size_t>(outage)];
    if (outage_from != reference_bus) {
      const int row = outage_from < reference_bus
          ? outage_from
          : outage_from - 1;
      reduced_transfer[row] = 1.0;
    }
    if (outage_to != reference_bus) {
      const int row = outage_to < reference_bus ? outage_to : outage_to - 1;
      reduced_transfer[row] = -1.0;
    }
    const Eigen::VectorXd reduced_theta = factor.solve(reduced_transfer);
    if (factor.info() != Eigen::Success || !reduced_theta.allFinite()) {
      out.warnings.push_back(
          "Sparse LODF backsolve failed for AC branch position " +
          std::to_string(out.branch_positions[static_cast<size_t>(outage)]) + ".");
      continue;
    }
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(B);
    for (int i = 0, r = 0; i < B; ++i) {
      if (i == reference_bus) continue;
      theta[i] = reduced_theta[r++];
    }
    Eigen::VectorXd ptdf(L);
    for (int monitored = 0; monitored < L; ++monitored) {
      ptdf[monitored] = susceptance[monitored] *
          (theta[from_positions[static_cast<size_t>(monitored)]] -
           theta[to_positions[static_cast<size_t>(monitored)]]);
    }
    const double denominator = 1.0 - ptdf[outage];
    if (std::abs(denominator) < 1e-7 || !std::isfinite(denominator)) {
      out.skipped_islanding_branch_positions.push_back(
          out.branch_positions[static_cast<size_t>(outage)]);
      continue;
    }
    Eigen::VectorXd column = ptdf / denominator;
    column[outage] = -1.0;
    out.value_column_by_outage_active_index[static_cast<size_t>(outage)] =
        static_cast<int>(columns.size());
    columns.push_back(std::move(column));
    out.valid_outage_active_indices.push_back(outage);
  }
  out.values.resize(L, static_cast<int>(columns.size()));
  for (int column = 0; column < static_cast<int>(columns.size()); ++column) {
    out.values.col(column) = columns[static_cast<size_t>(column)];
  }
  const long long sparse_matrix_bytes =
      static_cast<long long>(reduced.nonZeros()) *
          static_cast<long long>(sizeof(double) + sizeof(int)) +
      static_cast<long long>(reduced.outerSize() + 1) * sizeof(int);
  const long long topology_bytes =
      static_cast<long long>(L) *
      static_cast<long long>(sizeof(double) + 3 * sizeof(int));
  const long long column_bytes =
      static_cast<long long>(L) * static_cast<long long>(columns.size()) *
      static_cast<long long>(sizeof(double));
  // SparseLU fill depends on ordering. Four times the assembled matrix is a
  // conservative planning estimate; the exact allocator footprint is backend-specific.
  out.estimated_sparse_bytes = 4 * sparse_matrix_bytes + topology_bytes + column_bytes;
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
  const auto ranked = rank_n1_contingencies(lodf.branch_positions, pricing);
  std::vector<int> rank_by_active_index(lodf.branch_positions.size(), 0);
  for (int rank = 0; rank < static_cast<int>(ranked.size()); ++rank) {
    rank_by_active_index[static_cast<size_t>(ranked[static_cast<size_t>(rank)])] = rank;
  }
  std::stable_sort(candidates.begin(), candidates.end(), [&](int lhs, int rhs) {
    return rank_by_active_index[static_cast<size_t>(lhs)] <
        rank_by_active_index[static_cast<size_t>(rhs)];
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
            base_flow + lodf_value(lodf, monitored, outage) * outage_flow;
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
    period.dc_lmp_per_mwh.assign(system.dc.buses.size(), 0.0);
    period.dc_bus_voltage_pu.assign(system.dc.buses.size(), 0.0);
    period.dc_branch_flow_mw.assign(system.dc.branches.size(), 0.0);
    period.dc_load_shedding_mw.assign(system.dc.buses.size(), 0.0);
    period.dc_exogenous_curtailment_mw.assign(system.dc.buses.size(), 0.0);
    period.vsc_ac_injection_mw.assign(system.vsc_converters.size(), 0.0);
    period.vsc_dc_injection_mw.assign(system.vsc_converters.size(), 0.0);
    period.vsc_loss_mw.assign(system.vsc_converters.size(), 0.0);
    period.dcdc_input_withdrawal_mw.assign(
        system.dc.dcdc_converters.size(), 0.0);
    period.dcdc_output_injection_mw.assign(
        system.dc.dcdc_converters.size(), 0.0);
    period.dcdc_loss_mw.assign(system.dc.dcdc_converters.size(), 0.0);
    period.legacy_dc_storage_dispatch_mw.assign(
        system.dc.storage.size(), 0.0);
    period.legacy_dc_storage_soc_mwh.assign(
        system.dc.storage.size(), 0.0);
    period.dc_storage_dispatch_mw.assign(
        system.dc.dc_storage.size(), 0.0);
    period.dc_storage_soc_mwh.assign(
        system.dc.dc_storage.size(), 0.0);
    period.reserve_requirement_mw =
        build.reserve_requirements_mw[static_cast<size_t>(t)];
    period.gross_demand_mw = std::accumulate(
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.begin(),
        build.periods[static_cast<size_t>(t)].gross_demand_by_bus_mw.end(), 0.0);
    period.gross_dc_demand_mw = std::accumulate(
        build.periods[static_cast<size_t>(t)].gross_dc_demand_by_bus_mw.begin(),
        build.periods[static_cast<size_t>(t)].gross_dc_demand_by_bus_mw.end(), 0.0);
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
    for (int m = 0; m < build.M; ++m) {
      const int position = build.dc_branch_positions[static_cast<size_t>(m)];
      period.dc_branch_flow_mw[static_cast<size_t>(position)] =
          solve.x[build.dc_flow(m, t)];
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
    for (int d = 0; d < build.D; ++d) {
      period.dc_bus_voltage_pu[static_cast<size_t>(d)] = solve.x[build.vdc(d, t)];
      const double shed = std::max(0.0, solve.x[build.dc_shed(d, t)]);
      const double curtail = std::max(0.0, solve.x[build.dc_curtail(d, t)]);
      period.dc_load_shedding_mw[static_cast<size_t>(d)] = shed;
      period.dc_exogenous_curtailment_mw[static_cast<size_t>(d)] = curtail;
      period.objective += shed * build.model.c[build.dc_shed(d, t)] +
                          curtail * build.model.c[build.dc_curtail(d, t)];
      if (have_duals) {
        period.dc_lmp_per_mwh[static_cast<size_t>(d)] =
            solve.constraint_duals[
                inequality_rows + build.dc_balance_row(d, t)] / dt;
      }
    }
    for (int c = 0; c < build.C; ++c) {
      const int position = build.vsc_positions[static_cast<size_t>(c)];
      const auto& converter = system.vsc_converters[static_cast<size_t>(position)];
      const double eta = effective_converter_efficiency(converter.eta);
      const double ac_to_dc = solve.x[build.vsc_ac_to_dc(c, t)];
      const double dc_to_ac = solve.x[build.vsc_dc_to_ac(c, t)];
      const double ac_injection = -ac_to_dc + eta * dc_to_ac;
      const double dc_injection = eta * ac_to_dc - dc_to_ac;
      period.vsc_ac_injection_mw[static_cast<size_t>(position)] = ac_injection;
      period.vsc_dc_injection_mw[static_cast<size_t>(position)] = dc_injection;
      period.vsc_loss_mw[static_cast<size_t>(position)] =
          std::max(0.0, -ac_injection - dc_injection);
    }
    for (int q = 0; q < build.Q; ++q) {
      const int position = build.dcdc_positions[static_cast<size_t>(q)];
      const auto& converter =
          system.dc.dcdc_converters[static_cast<size_t>(position)];
      const double eta = effective_converter_efficiency(converter.eta);
      const double forward = solve.x[build.dcdc_forward(q, t)];
      const double reverse = solve.x[build.dcdc_reverse(q, t)];
      const double input_withdrawal = forward - eta * reverse;
      const double output_injection = eta * forward - reverse;
      period.dcdc_input_withdrawal_mw[static_cast<size_t>(position)] =
          input_withdrawal;
      period.dcdc_output_injection_mw[static_cast<size_t>(position)] =
          output_injection;
      period.dcdc_loss_mw[static_cast<size_t>(position)] =
          std::max(0.0, input_withdrawal - output_injection);
    }
    for (int s = 0; s < build.S; ++s) {
      const auto& storage = build.dc_storages[static_cast<size_t>(s)];
      const double charge = solve.x[build.storage_charge(s, t)];
      const double discharge = solve.x[build.storage_discharge(s, t)];
      const double dispatch = discharge - charge;
      const double energy = solve.x[build.storage_energy(s, t)];
      if (storage.rich) {
        period.dc_storage_dispatch_mw[
            static_cast<size_t>(storage.position)] = dispatch;
        period.dc_storage_soc_mwh[
            static_cast<size_t>(storage.position)] = energy;
      } else {
        period.legacy_dc_storage_dispatch_mw[
            static_cast<size_t>(storage.position)] = dispatch;
        period.legacy_dc_storage_soc_mwh[
            static_cast<size_t>(storage.position)] = energy;
      }
      period.objective +=
          charge * build.model.c[build.storage_charge(s, t)] +
          discharge * build.model.c[build.storage_discharge(s, t)];
    }
    if (have_duals) {
      period.upward_reserve_price_per_mwh =
          solve.constraint_duals[inequality_rows + build.reserve_row(t)] / dt;
    }
  }
  return out;
}

std::vector<int> ac_island_components(
    const HybridPowerSystem& system,
    const std::unordered_map<int, int>& bus_positions,
    int& component_count) {
  const int B = static_cast<int>(system.ac.buses.size());
  component_count = 0;
  std::vector<int> component(static_cast<size_t>(B), -1);
  std::vector<std::vector<int>> adjacency(static_cast<size_t>(B));
  const auto link = [&](int from_bus, int to_bus) {
    const auto from = bus_positions.find(from_bus);
    const auto to = bus_positions.find(to_bus);
    if (from == bus_positions.end() || to == bus_positions.end()) return;
    adjacency[static_cast<size_t>(from->second)].push_back(to->second);
    adjacency[static_cast<size_t>(to->second)].push_back(from->second);
  };
  for (const auto& branch : system.ac.branches)
    if (branch.in_service) link(branch.from_bus, branch.to_bus);
  for (const auto& transformer : system.ac.transformers_2w)
    if (transformer.in_service) link(transformer.hv_bus, transformer.lv_bus);
  for (const auto& transformer : system.ac.transformers_3w) {
    if (!transformer.in_service) continue;
    link(transformer.hv_bus, transformer.mv_bus);
    link(transformer.hv_bus, transformer.lv_bus);
  }
  for (const auto& item : system.ac.switches)
    if (item.in_service && item.closed) link(item.bus_from, item.bus_to);
  for (const auto& item : system.ac.circuit_breakers)
    if (item.in_service && item.closed) link(item.bus_from, item.bus_to);

  for (int start = 0; start < B; ++start) {
    const auto& bus = system.ac.buses[static_cast<size_t>(start)];
    if (!bus.in_service || bus.bus_type == BusType::ISOLATED) {
      component[static_cast<size_t>(start)] = -2;
      continue;
    }
    if (component[static_cast<size_t>(start)] >= 0) continue;
    std::vector<int> queue{start};
    component[static_cast<size_t>(start)] = component_count;
    for (size_t head = 0; head < queue.size(); ++head) {
      const int current = queue[head];
      for (int next : adjacency[static_cast<size_t>(current)]) {
        if (component[static_cast<size_t>(next)] >= 0) continue;
        const auto& next_bus = system.ac.buses[static_cast<size_t>(next)];
        if (!next_bus.in_service || next_bus.bus_type == BusType::ISOLATED) {
          continue;
        }
        component[static_cast<size_t>(next)] = component_count;
        queue.push_back(next);
      }
    }
    ++component_count;
  }
  return component;
}

void select_online_slacks_by_island(HybridPowerSystem& snapshot) {
  const auto bus_positions = make_bus_position_map(snapshot);
  int component_count = 0;
  const auto component = ac_island_components(
      snapshot, bus_positions, component_count);
  const std::vector<bool> original_generator_slack = [&] {
    std::vector<bool> values;
    values.reserve(snapshot.ac.generators.size());
    for (const auto& generator : snapshot.ac.generators)
      values.push_back(generator.is_slack);
    return values;
  }();
  const std::vector<bool> original_bus_slack = [&] {
    std::vector<bool> values;
    values.reserve(snapshot.ac.buses.size());
    for (const auto& bus : snapshot.ac.buses)
      values.push_back(bus.bus_type == BusType::SLACK);
    return values;
  }();

  for (auto& bus : snapshot.ac.buses) {
    if (bus.bus_type == BusType::SLACK) bus.bus_type = BusType::PQ;
  }
  for (auto& generator : snapshot.ac.generators) generator.is_slack = false;

  for (int island = 0; island < component_count; ++island) {
    int chosen_generator = -1;
    int chosen_score = -1;
    for (int g = 0; g < static_cast<int>(snapshot.ac.generators.size()); ++g) {
      const auto& generator = snapshot.ac.generators[static_cast<size_t>(g)];
      if (!generator.in_service) continue;
      const auto found = bus_positions.find(generator.bus);
      if (found == bus_positions.end() ||
          component[static_cast<size_t>(found->second)] != island) {
        continue;
      }
      const int score = original_generator_slack[static_cast<size_t>(g)]
          ? 2
          : (original_bus_slack[static_cast<size_t>(found->second)] ? 1 : 0);
      if (score > chosen_score) {
        chosen_generator = g;
        chosen_score = score;
      }
    }
    if (chosen_generator >= 0) {
      auto& chosen =
          snapshot.ac.generators[static_cast<size_t>(chosen_generator)];
      chosen.is_slack = true;
      snapshot.ac.buses[static_cast<size_t>(bus_positions.at(chosen.bus))]
          .bus_type = BusType::SLACK;
      continue;
    }

    // A grid-forming VSC owns the angle equation itself. If there is no such
    // device, retain one authored slack bus so the certification problem is
    // structurally anchored and can report the missing balancing source.
    bool has_converter_reference = false;
    for (const auto& converter : snapshot.vsc_converters) {
      if (!converter.in_service ||
          !resolve_device_control_role(converter).provides_ac_angle_reference) {
        continue;
      }
      const auto found = bus_positions.find(converter.bus_ac);
      if (found != bus_positions.end() &&
          component[static_cast<size_t>(found->second)] == island) {
        has_converter_reference = true;
        break;
      }
    }
    if (has_converter_reference) continue;
    for (int b = 0; b < static_cast<int>(snapshot.ac.buses.size()); ++b) {
      if (component[static_cast<size_t>(b)] == island &&
          original_bus_slack[static_cast<size_t>(b)]) {
        snapshot.ac.buses[static_cast<size_t>(b)].bus_type = BusType::SLACK;
        break;
      }
    }
  }
}

void apply_pricing_state(
    HybridPowerSystem& snapshot,
    const PeriodNetworkData& period,
    const PricingPeriod& pricing,
    bool optimize_dc_storage) {
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

  const auto dc_bus_positions = make_dc_bus_position_map(snapshot);
  std::vector<double> dc_served_fraction(snapshot.dc.buses.size(), 1.0);
  for (size_t d = 0; d < snapshot.dc.buses.size(); ++d) {
    const double gross = d < period.gross_dc_demand_by_bus_mw.size()
        ? std::max(0.0, period.gross_dc_demand_by_bus_mw[d])
        : 0.0;
    const double shed = d < pricing.dc_load_shedding_mw.size()
        ? std::clamp(pricing.dc_load_shedding_mw[d], 0.0, gross)
        : 0.0;
    dc_served_fraction[d] = gross > 1e-12 ? (gross - shed) / gross : 1.0;
    snapshot.dc.buses[d].pd_mw *= dc_served_fraction[d];
  }
  for (auto& load : snapshot.dc.loads) {
    const auto found = dc_bus_positions.find(load.bus);
    if (load.in_service && found != dc_bus_positions.end()) {
      load.scaling *= dc_served_fraction[static_cast<size_t>(found->second)];
    }
  }

  std::vector<double> positive_dc_exogenous(snapshot.dc.buses.size(), 0.0);
  const auto add_positive_dc = [&](int bus, double mw) {
    const auto found = dc_bus_positions.find(bus);
    if (found != dc_bus_positions.end()) {
      positive_dc_exogenous[static_cast<size_t>(found->second)] +=
          std::max(0.0, mw);
    }
  };
  for (const auto& source : snapshot.dc.static_generators)
    if (source.in_service) add_positive_dc(source.bus, source.p_mw * source.scaling);
  for (const auto& source : snapshot.dc.dc_static_generators)
    if (source.in_service) add_positive_dc(source.bus, source.p_set_mw * source.scaling);
  for (const auto& source : snapshot.dc.pv_arrays)
    if (source.in_service) add_positive_dc(source.bus, source.p_set_mw);
  for (const auto& source : snapshot.dc.storage)
    if (source.in_service &&
        !market_optimizes_dc_storage(source, optimize_dc_storage))
      add_positive_dc(source.bus, source.p_mw);
  for (const auto& source : snapshot.dc.dc_storage)
    if (source.in_service &&
        !market_optimizes_dc_storage(source, optimize_dc_storage))
      add_positive_dc(source.bus, source.p_mw);
  std::vector<double> dc_exogenous_fraction(snapshot.dc.buses.size(), 1.0);
  for (size_t d = 0; d < snapshot.dc.buses.size(); ++d) {
    const double curtail = d < pricing.dc_exogenous_curtailment_mw.size()
        ? std::max(0.0, pricing.dc_exogenous_curtailment_mw[d])
        : 0.0;
    if (positive_dc_exogenous[d] > 1e-12) {
      dc_exogenous_fraction[d] = std::clamp(
          1.0 - curtail / positive_dc_exogenous[d], 0.0, 1.0);
    }
  }
  const auto dc_fraction = [&](int bus) {
    const auto found = dc_bus_positions.find(bus);
    return found == dc_bus_positions.end()
        ? 1.0
        : dc_exogenous_fraction[static_cast<size_t>(found->second)];
  };
  for (auto& source : snapshot.dc.static_generators)
    if (source.in_service && source.p_mw * source.scaling > 0.0)
      source.scaling *= dc_fraction(source.bus);
  for (auto& source : snapshot.dc.dc_static_generators)
    if (source.in_service && source.p_set_mw * source.scaling > 0.0)
      source.scaling *= dc_fraction(source.bus);
  for (auto& source : snapshot.dc.pv_arrays)
    if (source.in_service && source.p_set_mw > 0.0)
      source.p_set_mw *= dc_fraction(source.bus);
  for (auto& source : snapshot.dc.storage)
    if (source.in_service &&
        !market_optimizes_dc_storage(source, optimize_dc_storage) &&
        source.p_mw > 0.0)
      source.p_mw *= dc_fraction(source.bus);
  for (auto& source : snapshot.dc.dc_storage)
    if (source.in_service &&
        !market_optimizes_dc_storage(source, optimize_dc_storage) &&
        source.p_mw > 0.0)
      source.p_mw *= dc_fraction(source.bus);

  if (optimize_dc_storage) {
    for (size_t s = 0; s < snapshot.dc.storage.size() &&
                       s < pricing.legacy_dc_storage_dispatch_mw.size(); ++s) {
      auto& storage = snapshot.dc.storage[s];
      if (market_optimizes_dc_storage(storage, true)) {
        storage.p_mw = pricing.legacy_dc_storage_dispatch_mw[s];
      }
    }
    for (size_t s = 0; s < snapshot.dc.dc_storage.size() &&
                       s < pricing.dc_storage_dispatch_mw.size(); ++s) {
      auto& storage = snapshot.dc.dc_storage[s];
      if (market_optimizes_dc_storage(storage, true)) {
        storage.p_mw = pricing.dc_storage_dispatch_mw[s];
      }
    }
  }

  for (size_t c = 0; c < snapshot.vsc_converters.size() &&
                     c < pricing.vsc_ac_injection_mw.size(); ++c) {
    auto& converter = snapshot.vsc_converters[c];
    if (!converter.in_service) continue;
    const double scheduled = pricing.vsc_ac_injection_mw[c];
    converter.p_schedule_mw = scheduled;
    converter.p_initial_mw = scheduled;
    if (!resolve_device_control_role(converter).provides_dc_v_reference &&
        !resolve_device_control_role(converter).is_ac_grid_forming) {
      converter.p_set_mw = scheduled;
      converter.p_is_hard_constraint = true;
      if (converter.control_mode != ConverterMode::AC_PV) {
        converter.control_mode = ConverterMode::PQ_MODE;
      }
    }
  }
  for (size_t q = 0; q < snapshot.dc.dcdc_converters.size() &&
                     q < pricing.dcdc_input_withdrawal_mw.size(); ++q) {
    auto& converter = snapshot.dc.dcdc_converters[q];
    if (!converter.in_service) continue;
    if (converter.control_mode == DCDCControlMode::Power ||
        !converter.controllable) {
      converter.p_ref_mw = pricing.dcdc_input_withdrawal_mw[q];
      converter.control_mode = DCDCControlMode::Power;
    }
  }
  select_online_slacks_by_island(snapshot);
}

ACValidationPeriod summarize_ac_validation(
    const HybridPowerSystem& snapshot,
    const PricingPeriod& pricing,
    const PowerFlowResult& power_flow,
    const MarketOptions& options) {
  ACValidationPeriod out;
  out.converged = power_flow.converged;
  out.residual = power_flow.residual;
  out.maximum_p_mismatch_pu = power_flow.diagnostics.max_p_mismatch_pu;
  out.maximum_q_mismatch_pu = power_flow.diagnostics.max_q_mismatch_pu;
  out.solver_warnings = power_flow.diagnostics.warnings;
  out.converter_model_scope = power_flow.converter_model_scope;
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
    const double violation =
        std::max({0.0, bus.vmin_pu - vm, vm - bus.vmax_pu});
    if (violation > std::max(0.0, options.ac_validation_voltage_tolerance_pu)) {
      out.violations.push_back(MarketConstraintViolation{
          "bus_voltage", "ac_bus", static_cast<int>(b), bus.index,
          bus.name.empty() ? "Bus " + std::to_string(bus.index) : bus.name,
          bus.index, 0, 0, vm, bus.vmin_pu, bus.vmax_pu, violation, "pu"});
    }
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
        const double overload = std::max(0.0, apparent - rate);
        if (overload >
            std::max(0.0, options.ac_validation_thermal_tolerance_mva)) {
          const auto& branch = snapshot.ac.branches[l];
          out.violations.push_back(MarketConstraintViolation{
              "branch_thermal", "ac_branch", static_cast<int>(l),
              branch.index,
              branch.name.empty()
                  ? "Branch " + std::to_string(branch.index)
                  : branch.name,
              0, branch.from_bus, branch.to_bus, apparent, 0.0, rate,
              overload, "MVA"});
        }
      }
    }
  }
  for (size_t d = 0; d < power_flow.vdc.size() &&
                     d < snapshot.dc.buses.size(); ++d) {
    const auto& bus = snapshot.dc.buses[d];
    const double voltage = power_flow.vdc[d];
    const double violation = std::max(
        {0.0, bus.vmin_pu - voltage, voltage - bus.vmax_pu});
    out.maximum_dc_voltage_violation_pu = std::max(
        out.maximum_dc_voltage_violation_pu, violation);
    if (violation > std::max(0.0, options.ac_validation_voltage_tolerance_pu)) {
      out.violations.push_back(MarketConstraintViolation{
          "dc_bus_voltage", "dc_bus", static_cast<int>(d), bus.index,
          bus.name.empty() ? "DC Bus " + std::to_string(bus.index) : bus.name,
          bus.index, 0, 0, voltage, bus.vmin_pu, bus.vmax_pu, violation, "pu"});
    }
  }
  const auto dc_bus_positions = make_dc_bus_position_map(snapshot);
  const double dc_base_mva =
      std::max({snapshot.base_mva, snapshot.dc.base_mva, 1.0});
  for (size_t l = 0; l < snapshot.dc.branches.size(); ++l) {
    const auto& branch = snapshot.dc.branches[l];
    if (!branch.in_service) continue;
    const auto from = dc_bus_positions.find(branch.from_bus);
    const auto to = dc_bus_positions.find(branch.to_bus);
    if (from == dc_bus_positions.end() || to == dc_bus_positions.end() ||
        from->second >= static_cast<int>(power_flow.vdc.size()) ||
        to->second >= static_cast<int>(power_flow.vdc.size())) {
      continue;
    }
    double resistance = branch.r_pu;
    if (std::abs(resistance) < 1e-12) resistance = 1e-6;
    const double vf = power_flow.vdc[static_cast<size_t>(from->second)];
    const double vt = power_flow.vdc[static_cast<size_t>(to->second)];
    const double current_pu = (vf - vt) / resistance;
    const double from_mw = vf * current_pu * dc_base_mva;
    const double to_mw = -vt * current_pu * dc_base_mva;
    const double actual = std::max(std::abs(from_mw), std::abs(to_mw));
    const double rating = branch.rate_a_mva > 1e-9
        ? branch.rate_a_mva
        : branch.s_max_mva;
    if (rating <= 1e-9) continue;
    const double overload = std::max(0.0, actual - rating);
    out.maximum_dc_branch_overload_mw = std::max(
        out.maximum_dc_branch_overload_mw, overload);
    if (overload > std::max(0.0, options.ac_validation_thermal_tolerance_mva)) {
      out.violations.push_back(MarketConstraintViolation{
          "dc_branch_thermal", "dc_branch", static_cast<int>(l), branch.index,
          branch.name.empty() ? "DC Branch " + std::to_string(branch.index)
                              : branch.name,
          0, branch.from_bus, branch.to_bus, actual, 0.0, rating, overload, "MW"});
    }
  }
  for (const auto& transfer : power_flow.vsc_transfers) {
    const auto found = std::find_if(
        snapshot.vsc_converters.begin(), snapshot.vsc_converters.end(),
        [&](const VSCConverter& converter) { return converter.index == transfer.index; });
    if (found == snapshot.vsc_converters.end()) continue;
    const int position = static_cast<int>(
        std::distance(snapshot.vsc_converters.begin(), found));
    if (!found->p_is_hard_constraint) continue;
    if (position >= static_cast<int>(pricing.vsc_ac_injection_mw.size())) continue;
    const double deviation = std::abs(
        transfer.p_ac_mw - pricing.vsc_ac_injection_mw[static_cast<size_t>(position)]);
    out.maximum_vsc_schedule_deviation_mw = std::max(
        out.maximum_vsc_schedule_deviation_mw, deviation);
  }
  for (const auto& transfer : power_flow.dcdc_transfers) {
    const auto found = std::find_if(
        snapshot.dc.dcdc_converters.begin(), snapshot.dc.dcdc_converters.end(),
        [&](const DCDCConverter& converter) { return converter.index == transfer.index; });
    if (found == snapshot.dc.dcdc_converters.end()) continue;
    const int position = static_cast<int>(
        std::distance(snapshot.dc.dcdc_converters.begin(), found));
    if (found->control_mode != DCDCControlMode::Power) continue;
    if (position >= static_cast<int>(pricing.dcdc_input_withdrawal_mw.size())) continue;
    const double deviation = std::abs(
        transfer.p_in_mw -
        pricing.dcdc_input_withdrawal_mw[static_cast<size_t>(position)]);
    out.maximum_dcdc_schedule_deviation_mw = std::max(
        out.maximum_dcdc_schedule_deviation_mw, deviation);
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
  const double converter_ac_injection = std::accumulate(
      power_flow.vsc_transfers.begin(), power_flow.vsc_transfers.end(), 0.0,
      [](double total, const VSCTransfer& transfer) {
        return total + transfer.p_ac_mw;
      });
  const double served = pricing.gross_demand_mw -
      std::accumulate(pricing.load_shedding_mw.begin(),
                      pricing.load_shedding_mw.end(), 0.0);
  out.slack_adjustment_mw = served + out.total_branch_loss_mw - generation -
      exogenous - converter_ac_injection;
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
    if (out.maximum_generator_active_violation_mw >
        std::max(0.0, options.ac_validation_generator_tolerance_mw)) {
      out.violations.push_back(MarketConstraintViolation{
          "generator_active", "generator", static_cast<int>(g),
          generator.index,
          generator.name.empty()
              ? "Generator " + std::to_string(generator.index)
              : generator.name,
          generator.bus, 0, 0, adjusted, generator.pmin_mw,
          generator.pmax_mw, out.maximum_generator_active_violation_mw,
          "MW"});
    }
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
          std::max(0.0, options.ac_validation_generator_tolerance_mw) &&
      out.maximum_dc_voltage_violation_pu <=
          std::max(0.0, options.ac_validation_voltage_tolerance_pu) &&
      out.maximum_dc_branch_overload_mw <=
          std::max(0.0, options.ac_validation_thermal_tolerance_mva) &&
      out.maximum_vsc_schedule_deviation_mw <=
          std::max(0.0, options.ac_validation_generator_tolerance_mw) &&
      out.maximum_dcdc_schedule_deviation_mw <=
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
      const auto& outage_branch =
          system.ac.branches[static_cast<size_t>(outage_position)];
      check.outage_branch_name = outage_branch.name.empty()
          ? "Branch " + std::to_string(outage_branch.index)
          : outage_branch.name;
      check.outage_from_bus = outage_branch.from_bus;
      check.outage_to_bus = outage_branch.to_bus;
      HybridPowerSystem snapshot = build.periods[static_cast<size_t>(t)].snapshot;
      apply_pricing_state(
          snapshot, build.periods[static_cast<size_t>(t)],
          result.pricing[static_cast<size_t>(t)],
          options.optimize_dc_storage);
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
            const double violation =
                std::max({0.0, bus.vmin_pu - vm, vm - bus.vmax_pu});
            if (violation > options.ac_contingency_voltage_tolerance_pu) {
              check.violations.push_back(MarketConstraintViolation{
                  "bus_voltage", "ac_bus", static_cast<int>(b), bus.index,
                  bus.name.empty() ? "Bus " + std::to_string(bus.index)
                                   : bus.name,
                  bus.index, 0, 0, vm, bus.vmin_pu, bus.vmax_pu,
                  violation, "pu"});
            }
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
            const double overload = std::max(0.0, apparent - emergency);
            if (overload > options.ac_contingency_thermal_tolerance_mva) {
              check.violations.push_back(MarketConstraintViolation{
                  "branch_thermal", "ac_branch", static_cast<int>(l),
                  branch.index,
                  branch.name.empty()
                      ? "Branch " + std::to_string(branch.index)
                      : branch.name,
                  0, branch.from_bus, branch.to_bus, apparent, 0.0,
                  emergency, overload, "MVA"});
            }
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

std::vector<ComponentContingency> component_n1_contingencies(
    const HybridPowerSystem& system,
    int maximum) {
  std::vector<ComponentContingency> out;
  const auto append = [&](std::string type, int position, int index,
                          const std::string& name, int bus, int from_bus,
                          int to_bus) {
    out.push_back(ComponentContingency{
        std::move(type), position, index, name, bus, from_bus, to_bus});
  };
  for (int g = 0; g < static_cast<int>(system.ac.generators.size()); ++g) {
    const auto& item = system.ac.generators[static_cast<size_t>(g)];
    if (item.in_service)
      append("ac_generator", g, item.index, item.name, item.bus, 0, 0);
  }
  for (int l = 0; l < static_cast<int>(system.ac.branches.size()); ++l) {
    const auto& item = system.ac.branches[static_cast<size_t>(l)];
    if (item.in_service)
      append("ac_branch", l, item.index, item.name, 0,
             item.from_bus, item.to_bus);
  }
  for (int m = 0; m < static_cast<int>(system.dc.branches.size()); ++m) {
    const auto& item = system.dc.branches[static_cast<size_t>(m)];
    if (item.in_service)
      append("dc_branch", m, item.index, item.name, 0,
             item.from_bus, item.to_bus);
  }
  for (int c = 0; c < static_cast<int>(system.vsc_converters.size()); ++c) {
    const auto& item = system.vsc_converters[static_cast<size_t>(c)];
    if (item.in_service)
      append("vsc_converter", c, item.index, item.name, 0,
             item.bus_ac, item.bus_dc);
  }
  for (int q = 0; q < static_cast<int>(system.dc.dcdc_converters.size()); ++q) {
    const auto& item = system.dc.dcdc_converters[static_cast<size_t>(q)];
    if (item.in_service)
      append("dcdc_converter", q, item.index, item.name, 0,
             item.bus_in, item.bus_out);
  }
  for (int s = 0; s < static_cast<int>(system.dc.storage.size()); ++s) {
    const auto& item = system.dc.storage[static_cast<size_t>(s)];
    if (item.in_service)
      append("legacy_dc_storage", s, item.index, item.name, item.bus, 0, 0);
  }
  for (int s = 0; s < static_cast<int>(system.dc.dc_storage.size()); ++s) {
    const auto& item = system.dc.dc_storage[static_cast<size_t>(s)];
    if (item.in_service)
      append("dc_storage", s, item.index, item.name, item.bus, 0, 0);
  }
  if (maximum > 0 && static_cast<int>(out.size()) > maximum) {
    out.resize(static_cast<size_t>(maximum));
  }
  return out;
}

template <typename Row>
std::vector<Row> filter_schedule_rows(
    const std::vector<Row>& rows,
    const std::vector<int>& original_positions,
    const std::vector<int>& retained_positions) {
  std::unordered_map<int, size_t> row_by_position;
  for (size_t i = 0; i < original_positions.size() && i < rows.size(); ++i) {
    row_by_position.emplace(original_positions[i], i);
  }
  std::vector<Row> out;
  out.reserve(retained_positions.size());
  for (int position : retained_positions) {
    const auto found = row_by_position.find(position);
    if (found != row_by_position.end()) out.push_back(rows[found->second]);
  }
  return out;
}

UCSchedule contingency_commitment(
    const HybridPowerSystem& original,
    const HybridPowerSystem& contingency_system,
    const UCSchedule& commitment) {
  UCSchedule out = commitment;
  const auto original_vsc = active_vsc_positions(original);
  const auto retained_vsc = active_vsc_positions(contingency_system);
  out.vsc_dispatch = filter_schedule_rows(
      commitment.vsc_dispatch, original_vsc, retained_vsc);
  out.vsc_direction_ac_to_dc = filter_schedule_rows(
      commitment.vsc_direction_ac_to_dc, original_vsc, retained_vsc);

  const auto original_dcdc = active_dcdc_positions(original);
  const auto retained_dcdc = active_dcdc_positions(contingency_system);
  out.dcdc_dispatch = filter_schedule_rows(
      commitment.dcdc_dispatch, original_dcdc, retained_dcdc);
  out.dcdc_direction_forward = filter_schedule_rows(
      commitment.dcdc_direction_forward, original_dcdc, retained_dcdc);

  const auto original_storage = active_market_dc_storages(original, true);
  const auto retained_storage =
      active_market_dc_storages(contingency_system, true);
  const auto key = [](const MarketDCStorage& item) {
    return (item.rich ? 1000000000 : 0) + item.position;
  };
  std::vector<int> original_storage_keys;
  std::vector<int> retained_storage_keys;
  for (const auto& item : original_storage)
    original_storage_keys.push_back(key(item));
  for (const auto& item : retained_storage)
    retained_storage_keys.push_back(key(item));
  out.market_dc_storage_dispatch_mw = filter_schedule_rows(
      commitment.market_dc_storage_dispatch_mw,
      original_storage_keys, retained_storage_keys);
  out.market_dc_storage_soc_mwh = filter_schedule_rows(
      commitment.market_dc_storage_soc_mwh,
      original_storage_keys, retained_storage_keys);
  out.market_dc_storage_direction_charging = filter_schedule_rows(
      commitment.market_dc_storage_direction_charging,
      original_storage_keys, retained_storage_keys);
  return out;
}

void apply_component_outage(
    const ComponentContingency& contingency,
    HybridPowerSystem& system,
    std::vector<GeneratorOffer>& offers) {
  const size_t position = static_cast<size_t>(contingency.component_position);
  if (contingency.component_type == "ac_generator") {
    auto& item = system.ac.generators[position];
    item.pg_mw = 0.0;
    item.pmin_mw = 0.0;
    item.pmax_mw = 0.0;
    const auto active = active_generator_positions(system);
    const auto found = std::find(active.begin(), active.end(),
                                 contingency.component_position);
    if (found != active.end()) {
      auto& offer = offers[static_cast<size_t>(
          std::distance(active.begin(), found))];
      offer.minimum_output_mw = 0.0;
      offer.physical_maximum_output_mw = 0.0;
      offer.offered_maximum_output_mw = 0.0;
      for (auto& segment : offer.energy_segments) segment.quantity_mw = 0.0;
    }
  } else if (contingency.component_type == "ac_branch") {
    system.ac.branches[position].in_service = false;
  } else if (contingency.component_type == "dc_branch") {
    system.dc.branches[position].in_service = false;
  } else if (contingency.component_type == "vsc_converter") {
    system.vsc_converters[position].in_service = false;
  } else if (contingency.component_type == "dcdc_converter") {
    system.dc.dcdc_converters[position].in_service = false;
  } else if (contingency.component_type == "legacy_dc_storage") {
    auto& item = system.dc.storage[position];
    item.in_service = false;
    item.p_mw = 0.0;
  } else if (contingency.component_type == "dc_storage") {
    auto& item = system.dc.dc_storage[position];
    item.in_service = false;
    item.p_mw = 0.0;
  }
}

double total_load_shedding_mwh(
    const std::vector<PricingPeriod>& pricing,
    double dt) {
  double total = 0.0;
  for (const auto& period : pricing) {
    total += dt * (std::accumulate(
        period.load_shedding_mw.begin(), period.load_shedding_mw.end(), 0.0) +
        std::accumulate(period.dc_load_shedding_mw.begin(),
                        period.dc_load_shedding_mw.end(), 0.0));
  }
  return total;
}

double total_curtailment_mwh(
    const std::vector<PricingPeriod>& pricing,
    double dt) {
  double total = 0.0;
  for (const auto& period : pricing) {
    total += dt * (std::accumulate(
        period.exogenous_curtailment_mw.begin(),
        period.exogenous_curtailment_mw.end(), 0.0) +
        std::accumulate(period.dc_exogenous_curtailment_mw.begin(),
                        period.dc_exogenous_curtailment_mw.end(), 0.0));
  }
  return total;
}

void run_full_component_n1_validation(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const std::vector<GeneratorOffer>& offers,
    const MarketOptions& options,
    const engine::SimplexOptions& simplex_options,
    MarketResult& result) {
  result.security.full_component_validation_run = true;
  result.security.full_component_contingencies_secure = true;
  const auto contingencies = component_n1_contingencies(
      system, options.n1_max_contingencies);
  result.security.full_component_candidate_contingencies =
      static_cast<int>(contingencies.size());
  result.performance.component_n1_parallel_requested =
      options.parallel_component_n1;
  const int workers = options.parallel_component_n1
      ? util::resolve_worker_count(
            options.component_n1_parallel_workers,
            static_cast<int>(contingencies.size()))
      : 1;
  result.performance.component_n1_parallel_workers = workers;
  result.performance.component_n1_parallel_effective = workers > 1;
  const double dt = time_series.step_duration_hr;
  const double baseline_shedding =
      total_load_shedding_mwh(result.pricing, dt);

  std::vector<ComponentContingencyCheck> checks(contingencies.size());
  const auto solve_contingency = [&](size_t contingency_index) {
    const auto& contingency = contingencies[contingency_index];
    ComponentContingencyCheck check;
    check.component_type = contingency.component_type;
    check.component_position = contingency.component_position;
    check.component_index = contingency.component_index;
    check.component_name = contingency.component_name;
    check.bus = contingency.bus;
    check.from_bus = contingency.from_bus;
    check.to_bus = contingency.to_bus;

    HybridPowerSystem contingency_system = system;
    auto contingency_offers = offers;
    apply_component_outage(
        contingency, contingency_system, contingency_offers);
    const auto fixed_commitment = contingency_commitment(
        system, contingency_system, result.commitment);
    try {
      const auto build = build_pricing_model(
          contingency_system, time_series, fixed_commitment,
          contingency_offers, options);
      const auto solved = solve_market_pricing_lp(
          build.model, simplex_options, options.pricing_native_max_variables);
      check.sced_converged = solved.stats.success;
      check.status = solved.stats.status;
      if (check.sced_converged) {
        const auto pricing = extract_pricing(
            contingency_system, build, solved, dt);
        check.total_load_shedding_mwh =
            total_load_shedding_mwh(pricing, dt);
        check.incremental_load_shedding_mwh = std::max(
            0.0, check.total_load_shedding_mwh - baseline_shedding);
        check.total_exogenous_curtailment_mwh =
            total_curtailment_mwh(pricing, dt);
        check.objective = std::accumulate(
            pricing.begin(), pricing.end(), 0.0,
            [](double total, const PricingPeriod& period) {
              return total + period.objective;
            });
      }
    } catch (const std::exception& error) {
      check.status = error.what();
    }
    check.secure = check.sced_converged &&
        check.incremental_load_shedding_mwh <=
            std::max(0.0, options.n1_violation_tolerance_mw) * dt;
    checks[contingency_index] = std::move(check);
  };

  if (workers > 1) {
    util::ThreadPool pool(workers);
    pool.parallel_for_dynamic(
        contingencies.size(), solve_contingency, workers);
  } else {
    for (size_t index = 0; index < contingencies.size(); ++index) {
      solve_contingency(index);
    }
  }
  for (auto& check : checks) {
    result.security.full_component_contingencies_secure =
        result.security.full_component_contingencies_secure && check.secure;
    result.security.component_checks.push_back(std::move(check));
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
  const auto dc_bus_positions = make_dc_bus_position_map(system);
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
    result.settlement.resource_ac_energy_revenue += settlement.energy_revenue;
    result.settlement.resource_reserve_revenue += settlement.reserve_revenue;
    result.settlement.resource_uplift_revenue += settlement.uplift;
  }

  result.dc_storage_settlement.resize(build.dc_storages.size());
  for (int s = 0; s < build.S; ++s) {
    const auto& storage = build.dc_storages[static_cast<size_t>(s)];
    auto& settlement = result.dc_storage_settlement[static_cast<size_t>(s)];
    settlement.component_type =
        storage.rich ? "dc_storage" : "legacy_dc_storage";
    settlement.component_position = storage.position;
    settlement.component_index = storage.index;
    settlement.component_name = storage.name;
    settlement.dc_bus = storage.bus;
    settlement.initial_soc_mwh = storage.initial_energy_mwh;
    const int bus = dc_bus_positions.at(storage.bus);
    for (int t = 0; t < build.T; ++t) {
      const auto& pricing = result.pricing[static_cast<size_t>(t)];
      const double dispatch = storage.rich
          ? pricing.dc_storage_dispatch_mw[
                static_cast<size_t>(storage.position)]
          : pricing.legacy_dc_storage_dispatch_mw[
                static_cast<size_t>(storage.position)];
      const double energy = storage.rich
          ? pricing.dc_storage_soc_mwh[static_cast<size_t>(storage.position)]
          : pricing.legacy_dc_storage_soc_mwh[
                static_cast<size_t>(storage.position)];
      const double charge = std::max(0.0, -dispatch);
      const double discharge = std::max(0.0, dispatch);
      const double lmp = pricing.dc_lmp_per_mwh[static_cast<size_t>(bus)];
      settlement.charge_mwh += charge * dt;
      settlement.discharge_mwh += discharge * dt;
      settlement.net_injection_mwh += dispatch * dt;
      settlement.energy_revenue += lmp * dispatch * dt;
      settlement.as_bid_cost +=
          (storage.charge_bid_price * charge +
           storage.discharge_bid_price * discharge) * dt;
      settlement.terminal_soc_mwh = energy;
    }
    settlement.profit = settlement.energy_revenue - settlement.as_bid_cost;
    result.settlement.resource_energy_revenue += settlement.energy_revenue;
    result.settlement.resource_dc_energy_revenue += settlement.energy_revenue;
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
      result.settlement.customer_ac_energy_payment +=
          lmp * served_load_mw * dt;
      const double exogenous_revenue =
          lmp * (period.exogenous_injection_by_bus_mw[static_cast<size_t>(b)] -
                 pricing.exogenous_curtailment_mw[static_cast<size_t>(b)]) * dt;
      result.settlement.resource_energy_revenue += exogenous_revenue;
      result.settlement.resource_ac_energy_revenue += exogenous_revenue;
    }
    for (int d = 0; d < build.D; ++d) {
      const double lmp = pricing.dc_lmp_per_mwh[static_cast<size_t>(d)];
      const double served_load_mw = std::max(
          0.0, period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] -
                   pricing.dc_load_shedding_mw[static_cast<size_t>(d)]);
      const double customer_payment = lmp * served_load_mw * dt;
      const double resource_revenue =
          lmp * (period.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)] -
                 pricing.dc_exogenous_curtailment_mw[static_cast<size_t>(d)]) * dt;
      result.settlement.customer_energy_payment += customer_payment;
      result.settlement.customer_dc_energy_payment += customer_payment;
      result.settlement.resource_energy_revenue += resource_revenue;
      result.settlement.resource_dc_energy_revenue += resource_revenue;
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

using MarketClock = std::chrono::steady_clock;

double market_elapsed_sec(MarketClock::time_point start) {
  return std::chrono::duration<double>(MarketClock::now() - start).count();
}

struct ScopedMarketTotalTimer {
  MarketPerformanceProfile& profile;
  MarketClock::time_point start{MarketClock::now()};
  ~ScopedMarketTotalTimer() { profile.total_sec = market_elapsed_sec(start); }
};

}  // namespace

MarketResult run_day_ahead_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const MarketOptions& options) {
  MarketResult result;
  ScopedMarketTotalTimer total_timer{result.performance};
  const auto generator_positions = active_generator_positions(system);
  const auto branch_positions = active_branch_positions(system);
  const auto dc_branch_positions = active_dc_branch_positions(system);
  const auto vsc_positions = active_vsc_positions(system);
  const auto dcdc_positions = active_dcdc_positions(system);
  const auto optimized_dc_storages = active_market_dc_storages(
      system, options.optimize_dc_storage);
  auto& performance = result.performance;
  performance.periods = time_series.num_steps;
  performance.active_generators = static_cast<int>(generator_positions.size());
  performance.active_ac_branches = static_cast<int>(branch_positions.size());
  performance.active_dc_branches =
      static_cast<int>(dc_branch_positions.size());
  performance.active_vsc_converters = static_cast<int>(vsc_positions.size());
  performance.active_dcdc_converters =
      static_cast<int>(dcdc_positions.size());
  performance.optimized_dc_storages =
      static_cast<int>(optimized_dc_storages.size());
  const long long T = std::max(0, time_series.num_steps);
  const long long G = performance.active_generators;
  const long long B = static_cast<long long>(system.ac.buses.size());
  const long long L = performance.active_ac_branches;
  const long long K = std::max(1, options.energy_offer_segments);
  const long long D = static_cast<long long>(system.dc.buses.size());
  const long long M = performance.active_dc_branches;
  const long long C = performance.active_vsc_converters;
  const long long Q = performance.active_dcdc_converters;
  const long long S = performance.optimized_dc_storages;
  performance.estimated_scuc_variables =
      T * ((5 + K) * G + 3 * B + L + 3 * D + M + 3 * C + 3 * Q + 4 * S +
           (options.enable_n1_security ? 1 : 0));
  performance.estimated_scuc_binary_variables = T * (G + C + Q + S);
  performance.estimated_sced_variables =
      T * ((2 + K) * G + 3 * B + L + 3 * D + M + 2 * C + 2 * Q + 3 * S);
  const long long reduced_buses = std::max(0LL, B - 1);
  performance.estimated_lodf_dense_bytes =
      static_cast<long long>(sizeof(double)) *
      (L * B + L * L + B * B + reduced_buses * reduced_buses);
  const bool hybrid = !system.dc.buses.empty() || !system.dc.branches.empty() ||
      !system.vsc_converters.empty() || !system.dc.dcdc_converters.empty();
  result.model_scope.model_scope = hybrid
      ? "ac-dc-linear-v1:dc-voltage+bidirectional-converters"
      : "ac-only-dc-power-flow";
  result.model_scope.dc_network_modelled = hybrid;
  result.model_scope.dc_voltage_linearized = hybrid;
  result.model_scope.vsc_bidirectional_efficiency_modelled =
      !system.vsc_converters.empty();
  result.model_scope.dcdc_bidirectional_efficiency_modelled =
      !system.dc.dcdc_converters.empty();
  const bool has_optimized_dc_storage = !optimized_dc_storages.empty();
  result.model_scope.dc_storage_optimized = has_optimized_dc_storage;
  result.model_scope.dc_storage_intertemporal_modelled =
      has_optimized_dc_storage;
  result.model_scope.limitations = {
      "DC branch losses are omitted from commercial SCUC/SCED and checked in nonlinear certification.",
      "VSC fixed, current-dependent, capacity-circle, current and modulation limits are not commercial constraints.",
      "Preventive price-eligible N-1 cuts cover AC branches; generator, DC branch, converter and DC-storage outages use corrective fixed-commitment SCED.",
      "Bus, load, shunt, switchgear and protection failures are outside the market N-1 element set."};
  if (!system.ac.storage.empty()) result.model_scope.limitations.push_back(
      "AC storage uses authored exogenous power in this generic market; charge/discharge bids and intertemporal SOC are not optimized. Use the Southern boundary model for AC storage scheduling.");
  if (!system.ac.flexible_loads.empty()) result.model_scope.limitations.push_back(
      "AC flexible loads enter generic market gross demand; compensated interruption is not optimized. Use the Southern boundary model for controllable-load compensation.");
  if (std::any_of(system.ac.generators.begin(),system.ac.generators.end(),
      [](const auto& g){return g.fuel_type==FuelType::Hydro;})) result.model_scope.limitations.push_back(
      "Generic hydro generator offers have no reservoir or cascade water balance; those states are modeled only in the Southern boundary workflow.");
  if (time_series.num_steps <= 0 || time_series.step_duration_hr <= 0.0) {
    result.status = "invalid_time_series";
    result.warnings.push_back("Market time series must have positive periods and duration.");
    return result;
  }
  result.unsupported_assets = unsupported_market_assets(system);
  if (!result.unsupported_assets.empty()) {
    result.status = "unsupported_hybrid_market_assets";
    result.warnings.push_back(
        "The hybrid market does not yet price external grids or energy-router ports. "
        "See unsupported_assets for the exact authored components.");
    return result;
  }
  if (system.ac.generators.empty() || system.ac.buses.empty()) {
    result.status = "empty_ac_market";
    return result;
  }
  const auto dc_bus_positions = make_dc_bus_position_map(system);
  const auto dc_references =
      dc_reference_bus_positions(system, dc_bus_positions);
  const auto dc_reference_error = dc_reference_coverage_error(
      system, dc_bus_positions, dc_references, "market");
  if (!dc_reference_error.empty()) {
    result.status = "invalid_dc_reference";
    result.warnings.push_back(dc_reference_error);
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
  const auto offer_started = MarketClock::now();
  try {
    submission = submit_participant_offers(
        system, options.participants, options.energy_offer_segments);
  } catch (const std::exception& error) {
    result.status = "offer_submission_error";
    result.warnings.push_back(error.what());
    performance.offer_submission_sec = market_elapsed_sec(offer_started);
    return result;
  }
  performance.offer_submission_sec = market_elapsed_sec(offer_started);
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
  const auto scuc_started = MarketClock::now();
  try {
    result.commitment = solve_market_commitment(
        system, time_series, result.offers, commitment_options);
  } catch (const std::exception& error) {
    result.status = "scuc_model_error";
    result.warnings.push_back(error.what());
    performance.scuc_sec = market_elapsed_sec(scuc_started);
    return result;
  }
  performance.scuc_sec = market_elapsed_sec(scuc_started);
  const auto capture_scuc_performance = [&]() {
    performance.scuc_solver_name = result.commitment.solver_name;
    performance.scuc_mip_start_provided =
        result.commitment.mip_start_provided;
    performance.scuc_structure_hint_provided =
        result.commitment.uc_structure_hint_provided;
    performance.scuc_branching_priorities_provided =
        result.commitment.branching_priorities_provided;
    performance.scuc_structured_branching_used =
        result.commitment.structured_branching_used;
    performance.scuc_mip_gap_target_met =
        result.commitment.mip_gap_target_met;
    performance.scuc_optimality_proven =
        result.commitment.optimality_proven;
    performance.scuc_mip_gap = result.commitment.mip_gap;
    performance.scuc_solver_status = result.commitment.solver_status;
    performance.scuc_warm_start_generation_sec =
        result.commitment.warm_start_generation_sec;
    performance.scuc_cross_round_solver_state_reuse_enabled =
        result.commitment.cross_round_solver_state_reuse_enabled;
    performance.scuc_cross_round_solver_state_reuse_used =
        result.commitment.cross_round_solver_state_reuse_used;
    performance.scuc_cross_round_solver_state_reuse_rounds =
        result.commitment.cross_round_solver_state_reuse_rounds;
    performance.scuc_root_cuts_reused =
        result.commitment.root_cuts_reused;
    performance.scuc_root_cuts_reused_count =
        result.commitment.root_cuts_reused_count;
    performance.scuc_root_basis_reused =
        result.commitment.root_basis_reused;
    performance.scuc_pseudocosts_reused =
        result.commitment.pseudocosts_reused;
    performance.scuc_search_tree_rebuilt =
        result.commitment.search_tree_rebuilt;
    performance.scuc_in_solve_network_constraint_generation_used =
        result.commitment.in_solve_network_constraint_generation_used;
    performance.scuc_in_solve_network_constraint_callback_calls =
        result.commitment.in_solve_network_constraint_callback_calls;
    performance.scuc_in_solve_network_constraints_submitted =
        result.commitment.in_solve_network_constraints_submitted;
    performance.scuc_network_constraint_generation_run =
        result.commitment.network_constraint_generation_run;
    performance.scuc_network_constraint_generation_converged =
        result.commitment.network_constraint_generation_converged;
    performance.scuc_network_constraint_generation_iterations =
        result.commitment.network_constraint_generation_iterations;
    performance.scuc_network_constraint_candidates =
        result.commitment.network_constraint_candidates;
    performance.scuc_network_constraints_activated =
        result.commitment.network_constraints_activated;
    performance.scuc_network_constraint_remaining_violations =
        result.commitment.network_constraint_remaining_violations;
    performance.scuc_network_constraint_worst_violation_mw =
        result.commitment.network_constraint_worst_violation_mw;
  };
  capture_scuc_performance();
  std::string actual_solver_name = result.commitment.solver_name;
  std::transform(actual_solver_name.begin(), actual_solver_name.end(),
                 actual_solver_name.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (!result.commitment.feasible &&
      uc_options.uc_solver != UCSolverChoice::HiGHS &&
      actual_solver_name.find("highs") == std::string::npos) {
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
      capture_scuc_performance();
    }
  }
  result.commitment_cost = result.commitment.total_cost;
  if (!result.commitment.feasible) {
    result.status = "scuc_infeasible";
    return result;
  }
  if (!result.commitment.optimality_proven) {
    if (result.commitment.mip_gap_target_met) {
      result.warnings.push_back(
          "SCUC met the configured MIP-gap target but is not a zero-gap optimality proof; inspect performance.scuc_mip_gap.");
    } else {
      result.warnings.push_back(
          "SCUC returned a feasible incumbent without meeting the configured MIP-gap target; inspect the solver status and actual gap.");
    }
  }

  PricingBuild pricing_build;
  MarketOptions effective_options = options;
  effective_options.uc_options = uc_options;
  const auto base_sced_started = MarketClock::now();
  try {
    pricing_build = build_pricing_model(
        system, time_series, result.commitment, result.offers, effective_options);
  } catch (const std::exception& error) {
    result.status = "pricing_model_error";
    result.warnings.push_back(error.what());
    performance.base_sced_sec = market_elapsed_sec(base_sced_started);
    return result;
  }

  engine::SimplexOptions simplex_options;
  simplex_options.max_iter = std::max(10000, options.uc_options.opf_options.max_inner_iterations);
  simplex_options.feasibility_tol = 1e-8;
  simplex_options.optimality_tol = 1e-8;
  simplex_options.verbose = options.verbose;
  bool pricing_fallback_used = false;
  bool pricing_direct_highs_used = false;
  const auto baseline_solved = solve_market_pricing_lp(
      pricing_build.model, simplex_options,
      options.pricing_native_max_variables,
      &pricing_fallback_used, &pricing_direct_highs_used);
  performance.base_sced_sec = market_elapsed_sec(base_sced_started);
  performance.pricing_solver_name = baseline_solved.stats.solver_name;
  performance.pricing_solver_fallback_used = pricing_fallback_used;
  performance.pricing_large_model_direct_highs_used =
      pricing_direct_highs_used;
  if (!baseline_solved.stats.success) {
    result.status = "sced_infeasible:" + baseline_solved.stats.status;
    return result;
  }
  if (pricing_fallback_used) {
    result.warnings.push_back(
        "Native simplex could not solve the pricing LP; embedded HiGHS recovered the SCED and dual prices.");
  }
  if (pricing_direct_highs_used) {
    result.warnings.push_back(
        "Pricing LP exceeded pricing_native_max_variables; embedded HiGHS was used directly and native simplex was skipped.");
  }
  const auto baseline_pricing = extract_pricing(
      system, pricing_build, baseline_solved,
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
  result.model_scope.energy_prices_valid = true;
  if (hybrid && options.enable_n1_security) {
    result.warnings.push_back(
        "Hybrid N-1 uses preventive LODF cuts for AC branches and corrective "
        "fixed-commitment SCED for DC branches, VSC, DC/DC, generators and DC storage.");
  }

  LODFModel lodf;
  bool lodf_built = false;
  std::vector<int> n1_contingencies;
  if (options.enable_n1_security || options.run_ac_contingency_validation) {
    const auto lodf_started = MarketClock::now();
    const auto ranked_candidates = rank_n1_contingencies(
        active_branch_positions(system), result.pricing);
    const bool all_security_contingencies =
        options.enable_n1_security && options.n1_max_contingencies <= 0;
    const bool all_validation_contingencies =
        options.run_ac_contingency_validation &&
        options.max_ac_contingencies <= 0;
    int required_valid_columns = 0;
    if (!all_security_contingencies && !all_validation_contingencies) {
      if (options.enable_n1_security) {
        required_valid_columns = std::max(
            required_valid_columns, options.n1_max_contingencies);
      }
      if (options.run_ac_contingency_validation) {
        required_valid_columns = std::max(
            required_valid_columns, options.max_ac_contingencies);
      }
    }
    lodf = build_lodf_model(
        system, ranked_candidates, required_valid_columns);
    performance.lodf_build_sec = market_elapsed_sec(lodf_started);
    performance.lodf_computed_columns = lodf.values.cols();
    performance.estimated_lodf_sparse_bytes = lodf.estimated_sparse_bytes;
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
    const auto n1_cut_started = MarketClock::now();
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
            lodf_value(lodf, monitored->second, outage->second),
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
      bool secured_fallback_used = false;
      bool secured_direct_highs_used = false;
      const auto secured_solved = solve_market_pricing_lp(
          secured_build.model, simplex_options,
          options.pricing_native_max_variables,
          &secured_fallback_used, &secured_direct_highs_used);
      if (!secured_solved.stats.success) {
        result.status =
            "security_sced_infeasible:" + secured_solved.stats.status;
        result.security.cuts_added = static_cast<int>(cuts.size());
        result.security.iterations = iteration;
        result.security.remaining_violations = current_screen.violations;
        result.security.final_violations =
            static_cast<int>(current_screen.violations.size());
        result.security.final_worst_overload_mw =
            current_screen.worst_overload_mw;
        return result;
      }
      if (secured_fallback_used) {
        result.warnings.push_back(
            "Native simplex could not solve an N-1 pricing LP; embedded HiGHS recovered the secured SCED and dual prices.");
      }
      if (secured_direct_highs_used) {
        performance.pricing_large_model_direct_highs_used = true;
      }

      auto secured_pricing = extract_pricing(
          system, secured_build, secured_solved,
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
          "The N-1 cut loop stopped with remaining AC branch contingency violations.");
    }
    performance.n1_cut_loop_sec = market_elapsed_sec(n1_cut_started);
    const auto component_n1_started = MarketClock::now();
    run_full_component_n1_validation(
        system, time_series, result.offers, effective_options,
        simplex_options, result);
    performance.component_n1_sec = market_elapsed_sec(component_n1_started);
    performance.component_contingency_solves =
        result.security.full_component_candidate_contingencies;
    result.model_scope.hybrid_n1_modelled = true;
    result.model_scope.full_component_n1_modelled = true;
    result.model_scope.n1_recourse_policy =
        "preventive-ac-branch-lodf+preventive-generator-capability+corrective-fixed-commitment-sced";
    if (!result.security.full_component_contingencies_secure) {
      result.security.warnings.push_back(
          "At least one generator, AC/DC branch, converter, or DC-storage outage "
          "requires incremental load shedding or has no feasible corrective SCED.");
    }
  }

  result.num_pricing_converged = 0;
  for (const auto& period : result.pricing) {
    if (period.converged) ++result.num_pricing_converged;
  }

  const auto nonlinear_started = MarketClock::now();
  if (options.run_ac_validation) {
    result.ac_validation.reserve(static_cast<size_t>(time_series.num_steps));
    result.ac_power_flow_results.reserve(static_cast<size_t>(time_series.num_steps));
    for (int t = 0; t < time_series.num_steps; ++t) {
      HybridPowerSystem snapshot =
          pricing_build.periods[static_cast<size_t>(t)].snapshot;
      const auto& pricing = result.pricing[static_cast<size_t>(t)];
      apply_pricing_state(
          snapshot, pricing_build.periods[static_cast<size_t>(t)], pricing,
          options.optimize_dc_storage);
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
  performance.nonlinear_validation_sec = market_elapsed_sec(nonlinear_started);

  MarketResult baseline_result;
  baseline_result.participants = result.participants;
  baseline_result.behavior_actions = result.behavior_actions;
  baseline_result.offers = result.offers;
  baseline_result.pricing = baseline_pricing;
  const auto settlement_started = MarketClock::now();
  settle_market(system, time_series, pricing_build,
                result.commitment, baseline_result);
  settle_market(system, time_series, pricing_build,
                result.commitment, result);
  performance.settlement_sec = market_elapsed_sec(settlement_started);
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
      (result.security.dc_n1_secured &&
       result.security.full_component_contingencies_secure);
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
  const bool has_eligible_dc_storage =
      !active_market_dc_storages(system, true).empty();
  if (has_eligible_dc_storage &&
      real_time_options.optimize_dc_storage !=
          day_ahead_result.model_scope.dc_storage_optimized) {
    result.status = "real_time_dc_storage_policy_mismatch";
    result.warnings.push_back(
        "Real-time DC-storage optimization must use the accepted day-ahead storage policy.");
    return result;
  }
  const auto bus_positions = make_bus_position_map(system);
  const auto dc_bus_positions = make_dc_bus_position_map(system);
  const auto dc_storages = active_market_dc_storages(
      system, real_time_options.optimize_dc_storage);
  std::vector<PeriodNetworkData> day_ahead_periods;
  std::vector<PeriodNetworkData> realized_periods;
  day_ahead_periods.reserve(static_cast<size_t>(T));
  realized_periods.reserve(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    day_ahead_periods.push_back(make_period_data(
        system, day_ahead_time_series, day_ahead_result.commitment, t,
        real_time_options.uc_options, bus_positions, dc_bus_positions,
        real_time_options.optimize_dc_storage));
    realized_periods.push_back(make_period_data(
        system, realized_time_series, day_ahead_result.commitment, t,
        real_time_options.uc_options, bus_positions, dc_bus_positions,
        real_time_options.optimize_dc_storage));
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
          day_ahead_period.net_demand_by_bus_mw.end(), 0.0) +
          std::accumulate(day_ahead_period.net_dc_demand_by_bus_mw.begin(),
                          day_ahead_period.net_dc_demand_by_bus_mw.end(), 0.0);
      const double realized_net_demand_mw = std::accumulate(
          realized_period.net_demand_by_bus_mw.begin(),
          realized_period.net_demand_by_bus_mw.end(), 0.0) +
          std::accumulate(realized_period.net_dc_demand_by_bus_mw.begin(),
                          realized_period.net_dc_demand_by_bus_mw.end(), 0.0);
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
    period.day_ahead_demand_mw = day_ahead_pricing.gross_demand_mw +
        day_ahead_pricing.gross_dc_demand_mw;
    period.realized_demand_mw = real_time_pricing.gross_demand_mw +
        real_time_pricing.gross_dc_demand_mw;
    period.demand_deviation_mw =
        period.realized_demand_mw - period.day_ahead_demand_mw;
    const auto& day_ahead_period = day_ahead_periods[static_cast<size_t>(t)];
    const auto& realized_period = realized_periods[static_cast<size_t>(t)];
    period.reserve_activation_requirement_mw =
        reserve_activation_requirement_mw[static_cast<size_t>(t)];
    if (!day_ahead_pricing.lmp_per_mwh.empty() ||
        !day_ahead_pricing.dc_lmp_per_mwh.empty()) {
      const double total = std::accumulate(
          day_ahead_pricing.lmp_per_mwh.begin(),
          day_ahead_pricing.lmp_per_mwh.end(), 0.0) +
          std::accumulate(day_ahead_pricing.dc_lmp_per_mwh.begin(),
                          day_ahead_pricing.dc_lmp_per_mwh.end(), 0.0);
      period.average_day_ahead_lmp_per_mwh = total /
          static_cast<double>(day_ahead_pricing.lmp_per_mwh.size() +
                              day_ahead_pricing.dc_lmp_per_mwh.size());
    }
    if (!real_time_pricing.lmp_per_mwh.empty() ||
        !real_time_pricing.dc_lmp_per_mwh.empty()) {
      const double total = std::accumulate(
          real_time_pricing.lmp_per_mwh.begin(),
          real_time_pricing.lmp_per_mwh.end(), 0.0) +
          std::accumulate(real_time_pricing.dc_lmp_per_mwh.begin(),
                          real_time_pricing.dc_lmp_per_mwh.end(), 0.0);
      period.average_real_time_lmp_per_mwh = total /
          static_cast<double>(real_time_pricing.lmp_per_mwh.size() +
                              real_time_pricing.dc_lmp_per_mwh.size());
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
    for (int d = 0; d < static_cast<int>(system.dc.buses.size()); ++d) {
      const double real_time_lmp =
          real_time_pricing.dc_lmp_per_mwh[static_cast<size_t>(d)];
      const double load_deviation_mw =
          realized_period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] -
          day_ahead_period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)];
      const double served_load_deviation_mw =
          (realized_period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] -
           real_time_pricing.dc_load_shedding_mw[static_cast<size_t>(d)]) -
          (day_ahead_period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)] -
           day_ahead_pricing.dc_load_shedding_mw[static_cast<size_t>(d)]);
      const double exogenous_deviation_mw =
          (realized_period.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)] -
           real_time_pricing.dc_exogenous_curtailment_mw[static_cast<size_t>(d)]) -
          (day_ahead_period.exogenous_dc_injection_by_bus_mw[static_cast<size_t>(d)] -
           day_ahead_pricing.dc_exogenous_curtailment_mw[static_cast<size_t>(d)]);
      period.customer_deviation_payment +=
          served_load_deviation_mw * real_time_lmp * dt;
      period.resource_deviation_revenue +=
          exogenous_deviation_mw * real_time_lmp * dt;
      if (ancillary.enabled) {
        const double day_ahead_load_mw =
            day_ahead_period.gross_dc_demand_by_bus_mw[static_cast<size_t>(d)];
        const double tolerance_mw =
            ancillary.load_imbalance_tolerance_fraction *
            std::max(std::abs(day_ahead_load_mw), 1.0);
        const double penalized_imbalance_mw = std::max(
            0.0, std::abs(load_deviation_mw) - tolerance_mw);
        period.load_penalized_imbalance_mwh += penalized_imbalance_mw * dt;
        period.load_imbalance_penalty += penalized_imbalance_mw * dt *
            ancillary.load_imbalance_penalty_per_mwh;
      }
    }
    for (const auto& storage : dc_storages) {
      const auto bus_it = dc_bus_positions.find(storage.bus);
      if (bus_it == dc_bus_positions.end()) continue;
      const auto& day_ahead_dispatch = storage.rich
          ? day_ahead_pricing.dc_storage_dispatch_mw
          : day_ahead_pricing.legacy_dc_storage_dispatch_mw;
      const auto& real_time_dispatch = storage.rich
          ? real_time_pricing.dc_storage_dispatch_mw
          : real_time_pricing.legacy_dc_storage_dispatch_mw;
      const size_t position = static_cast<size_t>(storage.position);
      if (position >= day_ahead_dispatch.size() ||
          position >= real_time_dispatch.size()) {
        continue;
      }
      const double deviation_mwh =
          (real_time_dispatch[position] - day_ahead_dispatch[position]) * dt;
      const double real_time_lmp = real_time_pricing.dc_lmp_per_mwh[
          static_cast<size_t>(bus_it->second)];
      const double revenue = deviation_mwh * real_time_lmp;
      period.dc_storage_deviation_revenue += revenue;
      period.resource_deviation_revenue += revenue;
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
      for (double lmp : period.dc_lmp_per_mwh) {
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
