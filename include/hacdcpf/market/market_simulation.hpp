#pragma once
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::market {

// ── Market configuration ─────────────────────────────────────────────────────

enum class MarketType { DayAhead, Intraday, RealTime, Capacity };

struct MarketConfig {
  MarketType type{MarketType::DayAhead};
  int num_periods{24};
  double period_duration_hr{1.0};
  double price_cap_usd_mwh{500.0};
  double price_floor_usd_mwh{0.0};
  bool enable_ancillary_services{false};
  bool enable_transmission_constraints{true};
  bool enable_unit_commitment{false};
};

// ── Participant bids ──────────────────────────────────────────────────────────

struct PricedQuantity {
  double quantity_mw{0.0};
  double price_usd_mwh{0.0};
};

struct GenCoBid {
  int generator_index{0};
  std::string name;
  std::vector<PricedQuantity> offer_curve;  ///< Stepped offer (MW, $/MWh)
  double startup_cost_usd{0.0};
  double no_load_cost_usd_hr{0.0};
  double min_up_time_hr{0.0};
  double min_down_time_hr{0.0};
};

// ── Optional inputs ───────────────────────────────────────────────────────────

struct MarketProfiles {
  std::vector<double> load_scale_per_period;   ///< Per-period load scaling
  std::vector<double> wind_scale_per_period;   ///< Per-period wind scaling
  std::vector<double> solar_scale_per_period;  ///< Per-period solar scaling
};

struct InitialStatus {
  std::vector<int>    unit_on_status;    ///< 1 = on, 0 = off per generator
  std::vector<double> hours_on;          ///< Hours each generator has been on
  std::vector<double> hours_off;         ///< Hours each generator has been off
};

struct ScenarioConfig {
  int num_scenarios{1};
  int random_seed{0};
  double probability_weight{1.0};
};

// ── Market output ─────────────────────────────────────────────────────────────

struct GenDispatch {
  int generator_index{0};
  std::string name;
  std::vector<double> pg_mw_per_period;
  std::vector<double> qg_mvar_per_period;
  std::vector<double> revenue_usd_per_period;
  std::vector<int>    commitment_per_period;  ///< 1=on, 0=off
  double total_revenue_usd{0.0};
  double total_cost_usd{0.0};
};

struct MarketClearingOutput {
  bool cleared{false};
  std::string status;
  std::vector<double> lmp_per_bus_per_period;   ///< Locational Marginal Prices
  std::vector<double> system_price_per_period;  ///< System-level clearing price
  std::vector<GenDispatch> dispatches;
  double total_social_welfare_usd{0.0};
  double total_generation_cost_usd{0.0};
  double runtime_sec{0.0};
};

// ── Solver function ───────────────────────────────────────────────────────────

MarketClearingOutput run_market_clearing(
    const HybridPowerSystem& sys,
    const MarketConfig& config = {},
    const std::vector<GenCoBid>& bids = {},
    const MarketProfiles* profiles = nullptr,
    const InitialStatus* init = nullptr,
    const ScenarioConfig* scen_cfg = nullptr);

}  // namespace hacdcpf::market
