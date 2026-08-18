#include "hacdcpf/integrated_energy/integrated_energy_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::integrated_energy {
namespace {

using hacdcpf::aml::Key;
using hacdcpf::aml::LinearExpr;
using hacdcpf::aml::Model;
using hacdcpf::aml::SolveOptions;
using hacdcpf::aml::SolveResult;
using hacdcpf::aml::TerminationStatus;
using hacdcpf::aml::VarArray;
using hacdcpf::aml::VarType;

constexpr double kEps = 1e-8;

std::string time_id(int t) {
  return "t" + std::to_string(t);
}

std::string state_id(int t) {
  return "s" + std::to_string(t);
}

Key tk(int t) {
  return Key::scalar(time_id(t));
}

Key sk(int t) {
  return Key::scalar(state_id(t));
}

double positive_or_zero(double v) {
  return std::isfinite(v) ? std::max(0.0, v) : 0.0;
}

double require_unit_efficiency(double value, const char* field_name) {
  // Passive conversion and storage factors obey 0 < eta <= 1. Heat-pump COP
  // is validated separately because it is a performance ratio, not eta.
  // Contract and carrier-balance derivation: docs/theory/integrated_energy_contract.md.
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
    throw std::invalid_argument(
        std::string("CampusIESData.") + field_name +
        " must be finite and in (0, 1]");
  }
  return value;
}

std::vector<double> profile_or_default(const std::vector<double>& src,
                                       int n,
                                       double fallback,
                                       const std::string& name) {
  if (n <= 0) {
    throw std::invalid_argument("CampusIESData.num_steps must be positive");
  }
  if (src.empty()) {
    return std::vector<double>(static_cast<std::size_t>(n), fallback);
  }
  if (src.size() == 1U) {
    return std::vector<double>(static_cast<std::size_t>(n), src.front());
  }
  if (src.size() != static_cast<std::size_t>(n)) {
    throw std::invalid_argument("CampusIESData." + name +
                                " length must be 0, 1, or num_steps");
  }
  return src;
}

double sum_vec(const std::vector<double>& values) {
  return std::accumulate(values.begin(), values.end(), 0.0);
}

double sum_energy(const std::vector<double>& mw, double dt) {
  return sum_vec(mw) * dt;
}

CampusIESData normalize_data(const CampusIESData& input) {
  CampusIESData d = input;
  if (d.num_steps <= 0) {
    throw std::invalid_argument("CampusIESData.num_steps must be positive");
  }
  if (!std::isfinite(d.step_duration_hr) || d.step_duration_hr <= 0.0) {
    throw std::invalid_argument("CampusIESData.step_duration_hr must be positive");
  }
  const int n = d.num_steps;
  if (!std::isfinite(d.fixed_power_factor) ||
      std::abs(d.fixed_power_factor - 1.0) > 1e-12) {
    throw std::invalid_argument(
        "CampusIESData.fixed_power_factor must be 1.0 because the campus MILP "
        "does not model reactive power or network voltage");
  }

  d.import_limit_mw = positive_or_zero(d.import_limit_mw);
  d.export_limit_mw = positive_or_zero(d.export_limit_mw);
  d.solar_rated_mw = positive_or_zero(d.solar_rated_mw);
  d.wind_rated_mw = positive_or_zero(d.wind_rated_mw);
  d.chp_power_max_mw = positive_or_zero(d.chp_power_max_mw);
  d.chp_heat_max_mw = positive_or_zero(d.chp_heat_max_mw);
  d.heat_pump_power_max_mw = positive_or_zero(d.heat_pump_power_max_mw);
  d.electrolyzer_power_max_mw = positive_or_zero(d.electrolyzer_power_max_mw);
  d.fuel_cell_power_max_mw = positive_or_zero(d.fuel_cell_power_max_mw);
  d.fuel_purchase_limit_mw = positive_or_zero(d.fuel_purchase_limit_mw);

  d.electric_storage_capacity_mwh = positive_or_zero(d.electric_storage_capacity_mwh);
  d.electric_storage_min_mwh = std::clamp(positive_or_zero(d.electric_storage_min_mwh),
                                          0.0, d.electric_storage_capacity_mwh);
  d.electric_storage_initial_mwh = std::clamp(positive_or_zero(d.electric_storage_initial_mwh),
                                              d.electric_storage_min_mwh,
                                              d.electric_storage_capacity_mwh);
  d.electric_storage_charge_max_mw = positive_or_zero(d.electric_storage_charge_max_mw);
  d.electric_storage_discharge_max_mw = positive_or_zero(d.electric_storage_discharge_max_mw);

  d.thermal_storage_capacity_mwh = positive_or_zero(d.thermal_storage_capacity_mwh);
  d.thermal_storage_min_mwh = std::clamp(positive_or_zero(d.thermal_storage_min_mwh),
                                         0.0, d.thermal_storage_capacity_mwh);
  d.thermal_storage_initial_mwh = std::clamp(positive_or_zero(d.thermal_storage_initial_mwh),
                                             d.thermal_storage_min_mwh,
                                             d.thermal_storage_capacity_mwh);
  d.thermal_storage_charge_max_mw = positive_or_zero(d.thermal_storage_charge_max_mw);
  d.thermal_storage_discharge_max_mw = positive_or_zero(d.thermal_storage_discharge_max_mw);

  d.hydrogen_storage_capacity_mwh = positive_or_zero(d.hydrogen_storage_capacity_mwh);
  d.hydrogen_storage_min_mwh = std::clamp(positive_or_zero(d.hydrogen_storage_min_mwh),
                                          0.0, d.hydrogen_storage_capacity_mwh);
  d.hydrogen_storage_initial_mwh = std::clamp(positive_or_zero(d.hydrogen_storage_initial_mwh),
                                              d.hydrogen_storage_min_mwh,
                                              d.hydrogen_storage_capacity_mwh);
  d.hydrogen_storage_charge_max_mw = positive_or_zero(d.hydrogen_storage_charge_max_mw);
  d.hydrogen_storage_discharge_max_mw = positive_or_zero(d.hydrogen_storage_discharge_max_mw);

  d.weekly_hydrogen_storage_capacity_mwh = positive_or_zero(d.weekly_hydrogen_storage_capacity_mwh);
  d.weekly_hydrogen_storage_min_mwh =
      std::clamp(positive_or_zero(d.weekly_hydrogen_storage_min_mwh),
                 0.0, d.weekly_hydrogen_storage_capacity_mwh);
  d.weekly_hydrogen_storage_initial_mwh =
      std::clamp(positive_or_zero(d.weekly_hydrogen_storage_initial_mwh),
                 d.weekly_hydrogen_storage_min_mwh,
                 d.weekly_hydrogen_storage_capacity_mwh);
  d.weekly_hydrogen_storage_charge_max_mw = positive_or_zero(d.weekly_hydrogen_storage_charge_max_mw);
  d.weekly_hydrogen_storage_discharge_max_mw = positive_or_zero(d.weekly_hydrogen_storage_discharge_max_mw);

  d.seasonal_hydrogen_storage_capacity_mwh = positive_or_zero(d.seasonal_hydrogen_storage_capacity_mwh);
  d.seasonal_hydrogen_storage_min_mwh =
      std::clamp(positive_or_zero(d.seasonal_hydrogen_storage_min_mwh),
                 0.0, d.seasonal_hydrogen_storage_capacity_mwh);
  d.seasonal_hydrogen_storage_initial_mwh =
      std::clamp(positive_or_zero(d.seasonal_hydrogen_storage_initial_mwh),
                 d.seasonal_hydrogen_storage_min_mwh,
                 d.seasonal_hydrogen_storage_capacity_mwh);
  d.seasonal_hydrogen_storage_charge_max_mw = positive_or_zero(d.seasonal_hydrogen_storage_charge_max_mw);
  d.seasonal_hydrogen_storage_discharge_max_mw = positive_or_zero(d.seasonal_hydrogen_storage_discharge_max_mw);
  d.daily_weekly_hydrogen_transfer_max_mw = positive_or_zero(d.daily_weekly_hydrogen_transfer_max_mw);
  d.weekly_seasonal_hydrogen_transfer_max_mw = positive_or_zero(d.weekly_seasonal_hydrogen_transfer_max_mw);

  d.eta_electrolysis = require_unit_efficiency(d.eta_electrolysis, "eta_electrolysis");
  d.eta_power_to_fuel = require_unit_efficiency(d.eta_power_to_fuel, "eta_power_to_fuel");
  d.eta_fuelcell = require_unit_efficiency(d.eta_fuelcell, "eta_fuelcell");
  if (!std::isfinite(d.cop_heatpump) || d.cop_heatpump <= 0.0) {
    d.cop_heatpump = 3.2;
  }
  d.eta_chp_elec = require_unit_efficiency(d.eta_chp_elec, "eta_chp_elec");
  d.eta_chp_heat = require_unit_efficiency(d.eta_chp_heat, "eta_chp_heat");
  d.eta_chp_total = require_unit_efficiency(d.eta_chp_total, "eta_chp_total");
  d.eta_storage_charge = require_unit_efficiency(d.eta_storage_charge, "eta_storage_charge");
  d.eta_storage_discharge = require_unit_efficiency(d.eta_storage_discharge, "eta_storage_discharge");
  d.electric_storage_retention = require_unit_efficiency(
      d.electric_storage_retention, "electric_storage_retention");
  d.thermal_storage_retention = require_unit_efficiency(
      d.thermal_storage_retention, "thermal_storage_retention");
  d.hydrogen_storage_retention = require_unit_efficiency(
      d.hydrogen_storage_retention, "hydrogen_storage_retention");
  d.weekly_hydrogen_storage_retention = require_unit_efficiency(
      d.weekly_hydrogen_storage_retention,
      "weekly_hydrogen_storage_retention");
  d.seasonal_hydrogen_storage_retention = require_unit_efficiency(
      d.seasonal_hydrogen_storage_retention,
      "seasonal_hydrogen_storage_retention");
  d.eta_daily_to_weekly = require_unit_efficiency(
      d.eta_daily_to_weekly, "eta_daily_to_weekly");
  d.eta_weekly_to_daily = require_unit_efficiency(
      d.eta_weekly_to_daily, "eta_weekly_to_daily");
  d.eta_weekly_to_seasonal = require_unit_efficiency(
      d.eta_weekly_to_seasonal, "eta_weekly_to_seasonal");
  d.eta_seasonal_to_weekly = require_unit_efficiency(
      d.eta_seasonal_to_weekly, "eta_seasonal_to_weekly");

  d.ev_ratio = positive_or_zero(d.ev_ratio);
  d.hv_ratio = positive_or_zero(d.hv_ratio);
  d.icv_ratio = positive_or_zero(d.icv_ratio);
  const double ratio_sum = d.ev_ratio + d.hv_ratio + d.icv_ratio;
  if (ratio_sum > kEps) {
    d.ev_ratio /= ratio_sum;
    d.hv_ratio /= ratio_sum;
    d.icv_ratio /= ratio_sum;
  } else {
    d.ev_ratio = 0.0;
    d.hv_ratio = 0.0;
    d.icv_ratio = 1.0;
  }

  d.ccus_capture_fraction = std::clamp(positive_or_zero(d.ccus_capture_fraction), 0.0, 1.0);
  d.ccus_max_tco2_per_h = positive_or_zero(d.ccus_max_tco2_per_h);
  d.ccus_power_mwh_per_tco2 = positive_or_zero(d.ccus_power_mwh_per_tco2);
  d.renewable_mandate_fraction = std::clamp(positive_or_zero(d.renewable_mandate_fraction), 0.0, 1.0);

  d.electric_load_mw = profile_or_default(d.electric_load_mw, n, 8.0, "electric_load_mw");
  d.heat_load_mw = profile_or_default(d.heat_load_mw, n, 4.0, "heat_load_mw");
  d.hydrogen_load_mw = profile_or_default(d.hydrogen_load_mw, n, 0.2, "hydrogen_load_mw");
  d.fuel_load_mw = profile_or_default(d.fuel_load_mw, n, 0.1, "fuel_load_mw");
  d.transport_demand_km = profile_or_default(d.transport_demand_km, n, 120.0, "transport_demand_km");
  d.solar_available_mw = profile_or_default(d.solar_available_mw, n, 0.0, "solar_available_mw");
  d.wind_available_mw = profile_or_default(d.wind_available_mw, n, 0.0, "wind_available_mw");
  d.grid_buy_price = profile_or_default(d.grid_buy_price, n, 90.0, "grid_buy_price");
  d.grid_sell_price = profile_or_default(d.grid_sell_price, n, 35.0, "grid_sell_price");
  d.grid_carbon_tco2_mwh = profile_or_default(d.grid_carbon_tco2_mwh, n, 0.58,
                                               "grid_carbon_tco2_mwh");

  for (int t = 0; t < n; ++t) {
    d.electric_load_mw[static_cast<std::size_t>(t)] = positive_or_zero(d.electric_load_mw[static_cast<std::size_t>(t)]);
    d.heat_load_mw[static_cast<std::size_t>(t)] = positive_or_zero(d.heat_load_mw[static_cast<std::size_t>(t)]);
    d.hydrogen_load_mw[static_cast<std::size_t>(t)] = positive_or_zero(d.hydrogen_load_mw[static_cast<std::size_t>(t)]);
    d.fuel_load_mw[static_cast<std::size_t>(t)] = positive_or_zero(d.fuel_load_mw[static_cast<std::size_t>(t)]);
    d.transport_demand_km[static_cast<std::size_t>(t)] = positive_or_zero(d.transport_demand_km[static_cast<std::size_t>(t)]);
    d.solar_available_mw[static_cast<std::size_t>(t)] =
        std::min(positive_or_zero(d.solar_available_mw[static_cast<std::size_t>(t)]), d.solar_rated_mw);
    d.wind_available_mw[static_cast<std::size_t>(t)] =
        std::min(positive_or_zero(d.wind_available_mw[static_cast<std::size_t>(t)]), d.wind_rated_mw);
    d.grid_sell_price[static_cast<std::size_t>(t)] =
        positive_or_zero(d.grid_sell_price[static_cast<std::size_t>(t)]);
    d.grid_buy_price[static_cast<std::size_t>(t)] =
        positive_or_zero(d.grid_buy_price[static_cast<std::size_t>(t)]);
    d.grid_carbon_tco2_mwh[static_cast<std::size_t>(t)] =
        positive_or_zero(d.grid_carbon_tco2_mwh[static_cast<std::size_t>(t)]);
  }

  return d;
}

std::string solver_name(CampusIESSolver solver) {
  switch (solver) {
    case CampusIESSolver::Auto:
      return "";
    case CampusIESSolver::HiGHS:
      return "highs";
    case CampusIESSolver::SCIP:
      return "scip";
    case CampusIESSolver::Gurobi:
      return "gurobi";
    case CampusIESSolver::Native:
      return "native";
  }
  return "";
}

std::string termination_status_to_string(TerminationStatus status) {
  switch (status) {
    case TerminationStatus::Optimal:
      return "optimal";
    case TerminationStatus::Infeasible:
      return "infeasible";
    case TerminationStatus::Unbounded:
      return "unbounded";
    case TerminationStatus::InfeasibleOrUnbounded:
      return "infeasible_or_unbounded";
    case TerminationStatus::TimeLimit:
      return "time_limit";
    case TerminationStatus::IterationLimit:
      return "iteration_limit";
    case TerminationStatus::NodeLimit:
      return "node_limit";
    case TerminationStatus::ObjectiveLimit:
      return "objective_limit";
    case TerminationStatus::NumericalError:
      return "numerical_error";
    case TerminationStatus::UserInterrupt:
      return "user_interrupt";
    case TerminationStatus::SolverError:
      return "solver_error";
    case TerminationStatus::Unknown:
      return "unknown";
  }
  return "unknown";
}

std::vector<double> extract_time_series(const SolveResult& sr,
                                        const VarArray& vars,
                                        int n) {
  std::vector<double> values(static_cast<std::size_t>(n), 0.0);
  if (!sr.has_primal()) {
    return values;
  }
  for (int t = 0; t < n; ++t) {
    double value = sr.var_value(vars, tk(t));
    if (std::abs(value) < 1e-7) {
      value = 0.0;
    }
    values[static_cast<std::size_t>(t)] = value;
  }
  return values;
}

std::vector<double> extract_state_series(const SolveResult& sr,
                                         const VarArray& vars,
                                         int n) {
  std::vector<double> values(static_cast<std::size_t>(n + 1), 0.0);
  if (!sr.has_primal()) {
    return values;
  }
  for (int t = 0; t <= n; ++t) {
    double value = sr.var_value(vars, sk(t));
    if (std::abs(value) < 1e-7) {
      value = 0.0;
    }
    values[static_cast<std::size_t>(t)] = value;
  }
  return values;
}

void add_sankey_flow(std::vector<CampusIESSankeyFlow>& flows,
                     const std::string& source,
                     const std::string& target,
                     double value_mwh,
                     const std::string& carrier) {
  if (value_mwh > 1e-5) {
    flows.push_back({source, target, value_mwh, carrier});
  }
}

double profile_sum_energy(const std::vector<double>& values, double dt) {
  return sum_vec(values) * dt;
}

}  // namespace

CampusIESData make_sample_campus_ies_data(int num_steps) {
  CampusIESData d;
  d.num_steps = std::max(1, num_steps);
  d.step_duration_hr = 1.0;

  d.electric_load_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.heat_load_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.hydrogen_load_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.fuel_load_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.transport_demand_km.resize(static_cast<std::size_t>(d.num_steps));
  d.solar_available_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.wind_available_mw.resize(static_cast<std::size_t>(d.num_steps));
  d.grid_buy_price.resize(static_cast<std::size_t>(d.num_steps));
  d.grid_sell_price.resize(static_cast<std::size_t>(d.num_steps));
  d.grid_carbon_tco2_mwh.resize(static_cast<std::size_t>(d.num_steps));

  constexpr double pi = 3.14159265358979323846;
  for (int t = 0; t < d.num_steps; ++t) {
    const double hour = static_cast<double>(t % 24);
    const double evening_peak = (hour >= 17.0 && hour <= 21.0) ? 1.0 : 0.0;
    const double workday = (hour >= 8.0 && hour <= 18.0) ? 1.0 : 0.0;
    const double solar_shape = std::max(0.0, std::sin((hour - 6.0) * pi / 12.0));
    const double wind_shape = 0.52 + 0.18 * std::sin((hour + 3.0) * pi / 12.0);

    d.electric_load_mw[static_cast<std::size_t>(t)] =
        7.0 + 1.2 * workday + 2.0 * evening_peak + 0.4 * std::sin(hour * pi / 12.0);
    d.heat_load_mw[static_cast<std::size_t>(t)] =
        3.0 + 0.7 * (1.0 - workday) + 0.3 * evening_peak;
    d.hydrogen_load_mw[static_cast<std::size_t>(t)] =
        0.18 + 0.06 * workday;
    d.fuel_load_mw[static_cast<std::size_t>(t)] =
        0.10 + 0.03 * workday;
    d.transport_demand_km[static_cast<std::size_t>(t)] =
        (hour >= 7.0 && hour <= 9.0) || (hour >= 17.0 && hour <= 19.0) ? 260.0 : 90.0;
    d.solar_available_mw[static_cast<std::size_t>(t)] = d.solar_rated_mw * solar_shape;
    d.wind_available_mw[static_cast<std::size_t>(t)] =
        d.wind_rated_mw * std::clamp(wind_shape, 0.15, 0.95);
    d.grid_buy_price[static_cast<std::size_t>(t)] =
        evening_peak > 0.0 ? 165.0 : ((hour >= 10.0 && hour <= 15.0) ? 55.0 : 95.0);
    d.grid_sell_price[static_cast<std::size_t>(t)] = 30.0;
    d.grid_carbon_tco2_mwh[static_cast<std::size_t>(t)] =
        evening_peak > 0.0 ? 0.68 : 0.52;
  }

  d.import_limit_mw = 16.0;
  d.export_limit_mw = 5.0;
  d.co2_budget_tco2 = 70.0;
  d.carbon_penalty_per_tco2 = 70.0;
  return d;
}

CampusIESResult solve_campus_ies(const CampusIESData& raw_data,
                                 const CampusIESOptions& options) {
  const CampusIESData data = normalize_data(raw_data);
  const int n = data.num_steps;
  const double dt = data.step_duration_hr;

  Model m("campus_integrated_energy");

  auto& times = m.add_ordered_set("time");
  auto& states = m.add_ordered_set("state");
  for (int t = 0; t < n; ++t) {
    times.add_element(time_id(t));
  }
  for (int t = 0; t <= n; ++t) {
    states.add_element(state_id(t));
  }

  auto& p_grid_import = m.add_var("P_grid_import", times, VarType::Continuous, 0.0,
                                  data.import_limit_mw);
  auto& p_grid_export = m.add_var("P_grid_export", times, VarType::Continuous, 0.0,
                                  data.export_limit_mw);
  auto& p_solar = m.add_var("P_solar", times, VarType::Continuous, 0.0,
                            std::max(0.0, data.solar_rated_mw));
  auto& p_wind = m.add_var("P_wind", times, VarType::Continuous, 0.0,
                           std::max(0.0, data.wind_rated_mw));
  auto& p_solar_curtail = m.add_var("P_solar_curtail", times, VarType::Continuous, 0.0,
                                    std::max(0.0, data.solar_rated_mw));
  auto& p_wind_curtail = m.add_var("P_wind_curtail", times, VarType::Continuous, 0.0,
                                   std::max(0.0, data.wind_rated_mw));

  auto& p_chp = m.add_var("P_CHP", times, VarType::Continuous, 0.0, data.chp_power_max_mw);
  auto& q_chp = m.add_var("Q_CHP", times, VarType::Continuous, 0.0, data.chp_heat_max_mw);
  auto& f_chp = m.add_var("F_CHP", times, VarType::Continuous, 0.0, data.fuel_purchase_limit_mw);
  auto& f_external = m.add_var("F_external", times, VarType::Continuous, 0.0,
                               data.fuel_purchase_limit_mw);
  auto& f_synthetic = m.add_var("F_synthetic", times, VarType::Continuous, 0.0, 1e20);
  auto& p_fuelcell = m.add_var("P_fuelcell", times, VarType::Continuous, 0.0,
                               data.fuel_cell_power_max_mw);
  auto& h_fuelcell = m.add_var("H_fuelcell", times, VarType::Continuous, 0.0, 1e20);

  auto& p_heatpump = m.add_var("P_heatpump", times, VarType::Continuous, 0.0,
                               data.heat_pump_power_max_mw);
  auto& q_heatpump = m.add_var("Q_heatpump", times, VarType::Continuous, 0.0, 1e20);
  auto& p_electrolysis_h2 = m.add_var("P_electrolysis_H2", times, VarType::Continuous, 0.0,
                                      data.electrolyzer_power_max_mw);
  auto& p_electrolysis_fuel = m.add_var("P_electrolysis_fuel", times, VarType::Continuous, 0.0,
                                        data.electrolyzer_power_max_mw);
  auto& h_electrolysis = m.add_var("H_electrolysis", times, VarType::Continuous, 0.0, 1e20);

  auto& p_storage_charge = m.add_var("P_storage_charge", times, VarType::Continuous, 0.0,
                                     data.electric_storage_charge_max_mw);
  auto& p_storage_discharge = m.add_var("P_storage_discharge", times, VarType::Continuous, 0.0,
                                        data.electric_storage_discharge_max_mw);
  auto& e_storage = m.add_var("E_storage", states, VarType::Continuous,
                              data.electric_storage_min_mwh,
                              data.electric_storage_capacity_mwh);

  auto& q_storage_charge = m.add_var("Q_storage_charge", times, VarType::Continuous, 0.0,
                                     data.thermal_storage_charge_max_mw);
  auto& q_storage_discharge = m.add_var("Q_storage_discharge", times, VarType::Continuous, 0.0,
                                        data.thermal_storage_discharge_max_mw);
  auto& q_storage = m.add_var("Q_storage", states, VarType::Continuous,
                              data.thermal_storage_min_mwh,
                              data.thermal_storage_capacity_mwh);

  auto& h_storage_charge = m.add_var("H_storage_charge", times, VarType::Continuous, 0.0,
                                     data.hydrogen_storage_charge_max_mw);
  auto& h_storage_discharge = m.add_var("H_storage_discharge", times, VarType::Continuous, 0.0,
                                        data.hydrogen_storage_discharge_max_mw);
  auto& h_storage = m.add_var("H_storage", states, VarType::Continuous,
                              data.hydrogen_storage_min_mwh,
                              data.hydrogen_storage_capacity_mwh);

  auto& h_weekly_charge = m.add_var("H_weekly_charge", times, VarType::Continuous, 0.0,
                                    data.weekly_hydrogen_storage_charge_max_mw);
  auto& h_weekly_discharge = m.add_var("H_weekly_discharge", times, VarType::Continuous, 0.0,
                                       data.weekly_hydrogen_storage_discharge_max_mw);
  auto& h_weekly_storage = m.add_var("H_weekly_storage", states, VarType::Continuous,
                                     data.weekly_hydrogen_storage_min_mwh,
                                     data.weekly_hydrogen_storage_capacity_mwh);
  auto& h_seasonal_charge = m.add_var("H_seasonal_charge", times, VarType::Continuous, 0.0,
                                      data.seasonal_hydrogen_storage_charge_max_mw);
  auto& h_seasonal_discharge = m.add_var("H_seasonal_discharge", times, VarType::Continuous, 0.0,
                                         data.seasonal_hydrogen_storage_discharge_max_mw);
  auto& h_seasonal_storage = m.add_var("H_seasonal_storage", states, VarType::Continuous,
                                       data.seasonal_hydrogen_storage_min_mwh,
                                       data.seasonal_hydrogen_storage_capacity_mwh);
  auto& h_daily_to_weekly = m.add_var("H_daily_to_weekly", times, VarType::Continuous, 0.0,
                                      data.daily_weekly_hydrogen_transfer_max_mw);
  auto& h_weekly_to_daily = m.add_var("H_weekly_to_daily", times, VarType::Continuous, 0.0,
                                      data.daily_weekly_hydrogen_transfer_max_mw);
  auto& h_weekly_to_seasonal = m.add_var("H_weekly_to_seasonal", times, VarType::Continuous, 0.0,
                                         data.weekly_seasonal_hydrogen_transfer_max_mw);
  auto& h_seasonal_to_weekly = m.add_var("H_seasonal_to_weekly", times, VarType::Continuous, 0.0,
                                         data.weekly_seasonal_hydrogen_transfer_max_mw);

  auto& d_ev = m.add_var("D_EV", times, VarType::Continuous, 0.0, 1e20);
  auto& d_hv = m.add_var("D_HV", times, VarType::Continuous, 0.0, 1e20);
  auto& d_icv = m.add_var("D_ICV", times, VarType::Continuous, 0.0, 1e20);
  auto& p_ev_charge = m.add_var("P_EV_charge", times, VarType::Continuous, 0.0, 1e20);
  auto& p_ev_v2g = m.add_var("P_EV_V2G", times, VarType::Continuous, 0.0, 0.0);
  auto& h_hv_refuel = m.add_var("H_HV_refuel", times, VarType::Continuous, 0.0, 1e20);
  auto& f_icv_refuel = m.add_var("F_ICV_refuel", times, VarType::Continuous, 0.0, 1e20);

  auto& emissions = m.add_var("Emissions_total", times, VarType::Continuous, 0.0, 1e20);
  auto& co2_captured = m.add_var("CO2_captured", times, VarType::Continuous, 0.0,
                                 data.ccus_max_tco2_per_h * dt);
  auto& carbon_residual = m.add_var("Carbon_residual", times, VarType::Continuous, 0.0, 1e20);
  auto& p_ccus = m.add_var("P_CCUS", times, VarType::Continuous, 0.0, 1e20);

  e_storage.fix(sk(0), data.electric_storage_initial_mwh);
  q_storage.fix(sk(0), data.thermal_storage_initial_mwh);
  h_storage.fix(sk(0), data.hydrogen_storage_initial_mwh);
  h_weekly_storage.fix(sk(0), data.weekly_hydrogen_storage_initial_mwh);
  h_seasonal_storage.fix(sk(0), data.seasonal_hydrogen_storage_initial_mwh);
  if (options.enforce_terminal_storage_cyclic) {
    e_storage.fix(sk(n), data.electric_storage_initial_mwh);
    q_storage.fix(sk(n), data.thermal_storage_initial_mwh);
    h_storage.fix(sk(n), data.hydrogen_storage_initial_mwh);
    h_weekly_storage.fix(sk(n), data.weekly_hydrogen_storage_initial_mwh);
    h_seasonal_storage.fix(sk(n), data.seasonal_hydrogen_storage_initial_mwh);
  }

  VarArray* grid_mode = nullptr;
  VarArray* e_storage_mode = nullptr;
  VarArray* q_storage_mode = nullptr;
  VarArray* h_storage_mode = nullptr;
  VarArray* h_weekly_mode = nullptr;
  VarArray* h_seasonal_mode = nullptr;
  if (options.enable_grid_exchange_exclusivity) {
    grid_mode = &m.add_var("grid_import_mode", times, VarType::Binary, 0.0, 1.0);
  }
  if (options.enable_storage_exclusivity) {
    e_storage_mode = &m.add_var("electric_storage_charge_mode", times, VarType::Binary, 0.0, 1.0);
    q_storage_mode = &m.add_var("thermal_storage_charge_mode", times, VarType::Binary, 0.0, 1.0);
    h_storage_mode = &m.add_var("hydrogen_storage_charge_mode", times, VarType::Binary, 0.0, 1.0);
    h_weekly_mode = &m.add_var("weekly_hydrogen_charge_mode", times, VarType::Binary, 0.0, 1.0);
    h_seasonal_mode = &m.add_var("seasonal_hydrogen_charge_mode", times, VarType::Binary, 0.0, 1.0);
  }

  LinearExpr total_renewable_expr;
  LinearExpr total_supply_expr;
  LinearExpr total_residual_carbon_expr;
  LinearExpr total_curtailment_expr;
  LinearExpr total_cost_expr;

  for (int t = 0; t < n; ++t) {
    const auto k = tk(t);
    const auto cur = sk(t);
    const auto nxt = sk(t + 1);
    const auto idx = static_cast<std::size_t>(t);

    m.add_constraint(p_solar(k) + p_solar_curtail(k) == data.solar_available_mw[idx],
                     "solar_availability_" + time_id(t));
    m.add_constraint(p_wind(k) + p_wind_curtail(k) == data.wind_available_mw[idx],
                     "wind_availability_" + time_id(t));
    m.add_constraint(p_electrolysis_h2(k) + p_electrolysis_fuel(k) <= data.electrolyzer_power_max_mw,
                     "electrolyzer_capacity_" + time_id(t));

    LinearExpr elec_supply = static_cast<LinearExpr>(p_grid_import(k)) - p_grid_export(k) +
                             p_solar(k) + p_wind(k) +
                             p_chp(k) + p_fuelcell(k) + p_storage_discharge(k) + p_ev_v2g(k);
    LinearExpr elec_demand = LinearExpr::const_expr(data.electric_load_mw[idx]) +
                             p_electrolysis_h2(k) + p_electrolysis_fuel(k) +
                             p_heatpump(k) + p_storage_charge(k) + p_ev_charge(k) +
                             p_ccus(k);
    m.add_constraint(elec_supply == elec_demand, "electricity_balance_" + time_id(t));

    LinearExpr heat_supply = q_chp(k) + q_heatpump(k) + q_storage_discharge(k) +
                             data.eta_wasteheat * p_heatpump(k);
    LinearExpr heat_demand = LinearExpr::const_expr(data.heat_load_mw[idx]) + q_storage_charge(k);
    m.add_constraint(heat_supply == heat_demand, "heat_balance_" + time_id(t));

    LinearExpr h_supply = h_electrolysis(k) + h_storage_discharge(k);
    if (options.enable_hydrogen_storage_layers) {
      h_supply += h_weekly_discharge(k) + h_seasonal_discharge(k);
    } else {
      m.add_constraint(h_weekly_charge(k) == 0.0, "weekly_h_charge_off_" + time_id(t));
      m.add_constraint(h_weekly_discharge(k) == 0.0, "weekly_h_discharge_off_" + time_id(t));
      m.add_constraint(h_seasonal_charge(k) == 0.0, "seasonal_h_charge_off_" + time_id(t));
      m.add_constraint(h_seasonal_discharge(k) == 0.0, "seasonal_h_discharge_off_" + time_id(t));
      m.add_constraint(h_daily_to_weekly(k) == 0.0, "h_daily_to_weekly_off_" + time_id(t));
      m.add_constraint(h_weekly_to_daily(k) == 0.0, "h_weekly_to_daily_off_" + time_id(t));
      m.add_constraint(h_weekly_to_seasonal(k) == 0.0, "h_weekly_to_seasonal_off_" + time_id(t));
      m.add_constraint(h_seasonal_to_weekly(k) == 0.0, "h_seasonal_to_weekly_off_" + time_id(t));
    }
    LinearExpr h_demand = LinearExpr::const_expr(data.hydrogen_load_mw[idx]) +
                          h_fuelcell(k) + h_hv_refuel(k) + h_storage_charge(k);
    if (options.enable_hydrogen_storage_layers) {
      h_demand += h_weekly_charge(k) + h_seasonal_charge(k);
    }
    m.add_constraint(h_supply == h_demand, "hydrogen_balance_" + time_id(t));

    m.add_constraint(f_external(k) + f_synthetic(k) ==
                         f_chp(k) + LinearExpr::const_expr(data.fuel_load_mw[idx]) + f_icv_refuel(k),
                     "fuel_balance_" + time_id(t));

    m.add_constraint(h_electrolysis(k) == data.eta_electrolysis * p_electrolysis_h2(k),
                     "electrolysis_conversion_" + time_id(t));
    m.add_constraint(f_synthetic(k) == data.eta_power_to_fuel * p_electrolysis_fuel(k),
                     "synthetic_fuel_conversion_" + time_id(t));
    m.add_constraint(p_fuelcell(k) == data.eta_fuelcell * h_fuelcell(k),
                     "fuelcell_conversion_" + time_id(t));
    m.add_constraint(q_heatpump(k) == data.cop_heatpump * p_heatpump(k),
                     "heatpump_conversion_" + time_id(t));
    m.add_constraint(p_chp(k) <= data.eta_chp_elec * f_chp(k),
                     "chp_electric_efficiency_" + time_id(t));
    m.add_constraint(q_chp(k) <= data.eta_chp_heat * f_chp(k),
                     "chp_heat_efficiency_" + time_id(t));
    m.add_constraint(p_chp(k) + q_chp(k) <= data.eta_chp_total * f_chp(k),
                     "chp_total_efficiency_" + time_id(t));

    m.add_constraint(e_storage(nxt) ==
                         data.electric_storage_retention * e_storage(cur) +
                         data.eta_storage_charge * dt * p_storage_charge(k) -
                         (dt / data.eta_storage_discharge) * p_storage_discharge(k),
                     "electric_storage_state_" + time_id(t));
    m.add_constraint(q_storage(nxt) ==
                         data.thermal_storage_retention * q_storage(cur) +
                         data.eta_storage_charge * dt * q_storage_charge(k) -
                         (dt / data.eta_storage_discharge) * q_storage_discharge(k),
                     "thermal_storage_state_" + time_id(t));
    m.add_constraint(h_storage(nxt) ==
                         data.hydrogen_storage_retention * h_storage(cur) +
                         data.eta_storage_charge * dt * h_storage_charge(k) -
                         (dt / data.eta_storage_discharge) * h_storage_discharge(k) +
                         dt * data.eta_weekly_to_daily * h_weekly_to_daily(k) -
                         dt * h_daily_to_weekly(k),
                     "hydrogen_storage_state_" + time_id(t));
    m.add_constraint(h_weekly_storage(nxt) ==
                         data.weekly_hydrogen_storage_retention * h_weekly_storage(cur) +
                         data.eta_storage_charge * dt * h_weekly_charge(k) -
                         (dt / data.eta_storage_discharge) * h_weekly_discharge(k) +
                         dt * data.eta_daily_to_weekly * h_daily_to_weekly(k) +
                         dt * data.eta_seasonal_to_weekly * h_seasonal_to_weekly(k) -
                         dt * h_weekly_to_daily(k) -
                         dt * h_weekly_to_seasonal(k),
                     "weekly_hydrogen_storage_state_" + time_id(t));
    m.add_constraint(h_seasonal_storage(nxt) ==
                         data.seasonal_hydrogen_storage_retention * h_seasonal_storage(cur) +
                         data.eta_storage_charge * dt * h_seasonal_charge(k) -
                         (dt / data.eta_storage_discharge) * h_seasonal_discharge(k) +
                         dt * data.eta_weekly_to_seasonal * h_weekly_to_seasonal(k) -
                         dt * h_seasonal_to_weekly(k),
                     "seasonal_hydrogen_storage_state_" + time_id(t));

    const double transport = options.enable_transport ? data.transport_demand_km[idx] : 0.0;
    m.add_constraint(d_ev(k) == data.ev_ratio * transport, "ev_transport_" + time_id(t));
    m.add_constraint(d_hv(k) == data.hv_ratio * transport, "hv_transport_" + time_id(t));
    m.add_constraint(d_icv(k) == data.icv_ratio * transport, "icv_transport_" + time_id(t));
    m.add_constraint(p_ev_charge(k) == (data.alpha_ev_mwh_per_km * data.ev_ratio * transport) / dt,
                     "ev_energy_" + time_id(t));
    m.add_constraint(h_hv_refuel(k) == (data.alpha_hv_mwh_per_km * data.hv_ratio * transport) / dt,
                     "hv_energy_" + time_id(t));
    m.add_constraint(f_icv_refuel(k) == (data.alpha_icv_mwh_per_km * data.icv_ratio * transport) / dt,
                     "icv_energy_" + time_id(t));

    m.add_constraint(emissions(k) ==
                         data.grid_carbon_tco2_mwh[idx] * dt * p_grid_import(k) +
                         data.fuel_carbon_tco2_mwh * dt * f_external(k),
                     "emissions_" + time_id(t));
    m.add_constraint(co2_captured(k) <= data.ccus_capture_fraction * emissions(k),
                     "ccus_capture_limit_" + time_id(t));
    m.add_constraint(carbon_residual(k) >=
                         static_cast<LinearExpr>(emissions(k)) - co2_captured(k),
                     "carbon_residual_" + time_id(t));
    m.add_constraint(p_ccus(k) >= (data.ccus_power_mwh_per_tco2 / dt) * co2_captured(k),
                     "ccus_power_" + time_id(t));
    if (data.eta_carbon_to_fuel > 0.0) {
      m.add_constraint(f_synthetic(k) <= data.eta_carbon_to_fuel * co2_captured(k),
                       "carbon_to_fuel_limit_" + time_id(t));
    } else {
      m.add_constraint(f_synthetic(k) == 0.0, "synthetic_fuel_off_" + time_id(t));
    }

    if (grid_mode != nullptr) {
      m.add_constraint(p_grid_import(k) <= data.import_limit_mw * (*grid_mode)(k),
                       "grid_import_mode_" + time_id(t));
      m.add_constraint(p_grid_export(k) <= data.export_limit_mw *
                                           (LinearExpr::const_expr(1.0) - (*grid_mode)(k)),
                       "grid_export_mode_" + time_id(t));
    }
    if (e_storage_mode != nullptr) {
      m.add_constraint(p_storage_charge(k) <= data.electric_storage_charge_max_mw *
                                               (*e_storage_mode)(k),
                       "electric_storage_charge_mode_" + time_id(t));
      m.add_constraint(p_storage_discharge(k) <= data.electric_storage_discharge_max_mw *
                                                  (LinearExpr::const_expr(1.0) - (*e_storage_mode)(k)),
                       "electric_storage_discharge_mode_" + time_id(t));
      m.add_constraint(q_storage_charge(k) <= data.thermal_storage_charge_max_mw *
                                               (*q_storage_mode)(k),
                       "thermal_storage_charge_mode_" + time_id(t));
      m.add_constraint(q_storage_discharge(k) <= data.thermal_storage_discharge_max_mw *
                                                  (LinearExpr::const_expr(1.0) - (*q_storage_mode)(k)),
                       "thermal_storage_discharge_mode_" + time_id(t));
      m.add_constraint(h_storage_charge(k) <= data.hydrogen_storage_charge_max_mw *
                                               (*h_storage_mode)(k),
                       "hydrogen_storage_charge_mode_" + time_id(t));
      m.add_constraint(h_storage_discharge(k) <= data.hydrogen_storage_discharge_max_mw *
                                                  (LinearExpr::const_expr(1.0) - (*h_storage_mode)(k)),
                       "hydrogen_storage_discharge_mode_" + time_id(t));
      m.add_constraint(h_weekly_charge(k) <= data.weekly_hydrogen_storage_charge_max_mw *
                                             (*h_weekly_mode)(k),
                       "weekly_hydrogen_charge_mode_" + time_id(t));
      m.add_constraint(h_weekly_discharge(k) <= data.weekly_hydrogen_storage_discharge_max_mw *
                                                (LinearExpr::const_expr(1.0) - (*h_weekly_mode)(k)),
                       "weekly_hydrogen_discharge_mode_" + time_id(t));
      m.add_constraint(h_seasonal_charge(k) <= data.seasonal_hydrogen_storage_charge_max_mw *
                                               (*h_seasonal_mode)(k),
                       "seasonal_hydrogen_charge_mode_" + time_id(t));
      m.add_constraint(h_seasonal_discharge(k) <= data.seasonal_hydrogen_storage_discharge_max_mw *
                                                  (LinearExpr::const_expr(1.0) - (*h_seasonal_mode)(k)),
                       "seasonal_hydrogen_discharge_mode_" + time_id(t));
    }

    total_renewable_expr += dt * (p_solar(k) + p_wind(k));
    total_supply_expr += dt * (p_grid_import(k) + p_solar(k) + p_wind(k) +
                               p_chp(k) + p_fuelcell(k));
    total_residual_carbon_expr += carbon_residual(k);
    total_curtailment_expr += dt * (p_solar_curtail(k) + p_wind_curtail(k));

    total_cost_expr += data.grid_buy_price[idx] * dt * p_grid_import(k);
    total_cost_expr += -data.grid_sell_price[idx] * dt * p_grid_export(k);
    total_cost_expr += data.fuel_cost_per_mwh * dt * f_external(k);
    total_cost_expr += data.solar_om_cost_per_mwh * dt * p_solar(k);
    total_cost_expr += data.wind_om_cost_per_mwh * dt * p_wind(k);
    total_cost_expr += data.chp_om_cost_per_mwh * dt * (p_chp(k) + q_chp(k));
    total_cost_expr += data.heat_pump_om_cost_per_mwh * dt * q_heatpump(k);
    total_cost_expr += data.electrolyzer_om_cost_per_mwh * dt *
                       (p_electrolysis_h2(k) + p_electrolysis_fuel(k));
    total_cost_expr += data.fuel_cell_om_cost_per_mwh * dt * p_fuelcell(k);
    total_cost_expr += data.storage_throughput_cost_per_mwh * dt *
                       (p_storage_charge(k) + p_storage_discharge(k) +
                        q_storage_charge(k) + q_storage_discharge(k));
    total_cost_expr += data.hydrogen_storage_throughput_cost_per_mwh * dt *
                       (h_storage_charge(k) + h_storage_discharge(k) +
                        h_weekly_charge(k) + h_weekly_discharge(k) +
                        h_seasonal_charge(k) + h_seasonal_discharge(k) +
                        h_daily_to_weekly(k) + h_weekly_to_daily(k) +
                        h_weekly_to_seasonal(k) + h_seasonal_to_weekly(k));
    total_cost_expr += data.curtailment_cost_per_mwh * dt *
                       (p_solar_curtail(k) + p_wind_curtail(k));
    total_cost_expr += data.carbon_penalty_per_tco2 * carbon_residual(k);
    total_cost_expr += data.ccus_cost_per_tco2 * co2_captured(k);
    total_cost_expr += data.ccus_power_cost_per_mwh * dt * p_ccus(k);
  }

  if (data.renewable_mandate_fraction > 0.0) {
    m.add_constraint(total_renewable_expr >= data.renewable_mandate_fraction * total_supply_expr,
                     "renewable_mandate");
  }
  if (options.enable_carbon_budget) {
    m.add_constraint(total_residual_carbon_expr <= data.co2_budget_tco2, "carbon_budget");
  }

  LinearExpr objective;
  switch (options.objective) {
    case CampusIESObjective::Cost:
      objective = total_cost_expr;
      break;
    case CampusIESObjective::Carbon:
      objective = total_residual_carbon_expr + 1e-6 * total_cost_expr;
      break;
    case CampusIESObjective::MinCurtailment:
      objective = total_curtailment_expr + 1e-6 * total_cost_expr;
      break;
    case CampusIESObjective::Weighted:
      objective = options.weight_cost * total_cost_expr +
                  options.weight_carbon * total_residual_carbon_expr +
                  options.weight_curtailment * total_curtailment_expr;
      break;
  }
  m.minimize(objective);

  SolveOptions solve_options;
  solve_options.solver_name = solver_name(options.solver);
  solve_options.time_limit_sec = options.time_limit_sec;
  solve_options.mip_gap_tol = options.mip_gap_tol;
  solve_options.verbosity = options.verbose ? 1 : 0;
  solve_options.allow_fallback = true;

  m.check_bounds();
  const auto sr = m.solve(solve_options);

  CampusIESResult result;
  result.pcc_ac_bus = data.pcc_ac_bus;
  result.model_limitations = {
      "The PCC is an aggregate active-power exchange and is not coupled to an AC network.",
      "Reactive power, bus voltages, branch flows, and electrical security limits are not modelled."};
  result.feasible = sr.has_primal();
  result.optimal = sr.is_optimal() &&
                   std::isfinite(sr.optimality_gap) &&
                   sr.optimality_gap <= options.mip_gap_tol + 1e-12;
  result.solver_name = sr.solver_used;
  result.status = termination_status_to_string(sr.termination_status);
  result.objective = sr.objective_value;
  result.solve_time_sec = sr.solve_time_sec;
  if (!sr.has_primal()) {
    return result;
  }

  result.p_grid_import_mw = extract_time_series(sr, p_grid_import, n);
  result.p_grid_export_mw = extract_time_series(sr, p_grid_export, n);
  result.p_solar_mw = extract_time_series(sr, p_solar, n);
  result.p_wind_mw = extract_time_series(sr, p_wind, n);
  result.p_solar_curtail_mw = extract_time_series(sr, p_solar_curtail, n);
  result.p_wind_curtail_mw = extract_time_series(sr, p_wind_curtail, n);
  result.p_chp_mw = extract_time_series(sr, p_chp, n);
  result.q_chp_mw = extract_time_series(sr, q_chp, n);
  result.f_chp_mw = extract_time_series(sr, f_chp, n);
  result.f_external_mw = extract_time_series(sr, f_external, n);
  result.f_synthetic_mw = extract_time_series(sr, f_synthetic, n);
  result.p_fuelcell_mw = extract_time_series(sr, p_fuelcell, n);
  result.h_fuelcell_mw = extract_time_series(sr, h_fuelcell, n);
  result.p_heatpump_mw = extract_time_series(sr, p_heatpump, n);
  result.q_heatpump_mw = extract_time_series(sr, q_heatpump, n);
  result.p_electrolysis_h2_mw = extract_time_series(sr, p_electrolysis_h2, n);
  result.p_electrolysis_fuel_mw = extract_time_series(sr, p_electrolysis_fuel, n);
  result.h_electrolysis_mw = extract_time_series(sr, h_electrolysis, n);
  result.p_storage_charge_mw = extract_time_series(sr, p_storage_charge, n);
  result.p_storage_discharge_mw = extract_time_series(sr, p_storage_discharge, n);
  result.e_storage_mwh = extract_state_series(sr, e_storage, n);
  result.q_storage_charge_mw = extract_time_series(sr, q_storage_charge, n);
  result.q_storage_discharge_mw = extract_time_series(sr, q_storage_discharge, n);
  result.q_storage_mwh = extract_state_series(sr, q_storage, n);
  result.h_storage_charge_mw = extract_time_series(sr, h_storage_charge, n);
  result.h_storage_discharge_mw = extract_time_series(sr, h_storage_discharge, n);
  result.h_storage_mwh = extract_state_series(sr, h_storage, n);
  result.h_weekly_charge_mw = extract_time_series(sr, h_weekly_charge, n);
  result.h_weekly_discharge_mw = extract_time_series(sr, h_weekly_discharge, n);
  result.h_weekly_storage_mwh = extract_state_series(sr, h_weekly_storage, n);
  result.h_seasonal_charge_mw = extract_time_series(sr, h_seasonal_charge, n);
  result.h_seasonal_discharge_mw = extract_time_series(sr, h_seasonal_discharge, n);
  result.h_seasonal_storage_mwh = extract_state_series(sr, h_seasonal_storage, n);
  result.h_daily_to_weekly_mw = extract_time_series(sr, h_daily_to_weekly, n);
  result.h_weekly_to_daily_mw = extract_time_series(sr, h_weekly_to_daily, n);
  result.h_weekly_to_seasonal_mw = extract_time_series(sr, h_weekly_to_seasonal, n);
  result.h_seasonal_to_weekly_mw = extract_time_series(sr, h_seasonal_to_weekly, n);
  result.d_ev_km = extract_time_series(sr, d_ev, n);
  result.d_hv_km = extract_time_series(sr, d_hv, n);
  result.d_icv_km = extract_time_series(sr, d_icv, n);
  result.p_ev_charge_mw = extract_time_series(sr, p_ev_charge, n);
  result.p_ev_v2g_mw = extract_time_series(sr, p_ev_v2g, n);
  result.h_hv_refuel_mw = extract_time_series(sr, h_hv_refuel, n);
  result.f_icv_refuel_mw = extract_time_series(sr, f_icv_refuel, n);
  result.emissions_tco2 = extract_time_series(sr, emissions, n);
  result.co2_captured_tco2 = extract_time_series(sr, co2_captured, n);
  result.carbon_residual_tco2 = extract_time_series(sr, carbon_residual, n);
  result.p_ccus_mw = extract_time_series(sr, p_ccus, n);

  result.p_pcc_mw.resize(static_cast<std::size_t>(n), 0.0);
  for (int t = 0; t < n; ++t) {
    const auto idx = static_cast<std::size_t>(t);
    result.p_pcc_mw[idx] = result.p_grid_export_mw[idx] - result.p_grid_import_mw[idx];
  }

  result.total_grid_import_mwh = sum_energy(result.p_grid_import_mw, dt);
  result.total_grid_export_mwh = sum_energy(result.p_grid_export_mw, dt);
  result.total_renewable_mwh = sum_energy(result.p_solar_mw, dt) + sum_energy(result.p_wind_mw, dt);
  result.total_renewable_available_mwh =
      profile_sum_energy(data.solar_available_mw, dt) + profile_sum_energy(data.wind_available_mw, dt);
  result.total_curtailment_mwh =
      sum_energy(result.p_solar_curtail_mw, dt) + sum_energy(result.p_wind_curtail_mw, dt);
  result.total_electric_load_mwh = profile_sum_energy(data.electric_load_mw, dt);
  result.total_heat_load_mwh = profile_sum_energy(data.heat_load_mw, dt);
  result.total_hydrogen_load_mwh = profile_sum_energy(data.hydrogen_load_mw, dt);
  result.total_fuel_load_mwh = profile_sum_energy(data.fuel_load_mw, dt);
  result.total_transport_km = sum_vec(result.d_ev_km) +
                              sum_vec(result.d_hv_km) +
                              sum_vec(result.d_icv_km);
  result.total_emissions_tco2 = sum_vec(result.emissions_tco2);
  result.total_co2_captured_tco2 = sum_vec(result.co2_captured_tco2);
  result.total_carbon_residual_tco2 = sum_vec(result.carbon_residual_tco2);
  result.renewable_utilization =
      result.total_renewable_available_mwh > kEps
          ? result.total_renewable_mwh / result.total_renewable_available_mwh
          : 0.0;

  double total_cost = 0.0;
  for (int t = 0; t < n; ++t) {
    const auto idx = static_cast<std::size_t>(t);
    total_cost += data.grid_buy_price[idx] * dt * result.p_grid_import_mw[idx];
    total_cost -= data.grid_sell_price[idx] * dt * result.p_grid_export_mw[idx];
    total_cost += data.fuel_cost_per_mwh * dt * result.f_external_mw[idx];
    total_cost += data.solar_om_cost_per_mwh * dt * result.p_solar_mw[idx];
    total_cost += data.wind_om_cost_per_mwh * dt * result.p_wind_mw[idx];
    total_cost += data.chp_om_cost_per_mwh * dt * (result.p_chp_mw[idx] + result.q_chp_mw[idx]);
    total_cost += data.heat_pump_om_cost_per_mwh * dt * result.q_heatpump_mw[idx];
    total_cost += data.electrolyzer_om_cost_per_mwh * dt *
                  (result.p_electrolysis_h2_mw[idx] + result.p_electrolysis_fuel_mw[idx]);
    total_cost += data.fuel_cell_om_cost_per_mwh * dt * result.p_fuelcell_mw[idx];
    total_cost += data.storage_throughput_cost_per_mwh * dt *
                  (result.p_storage_charge_mw[idx] + result.p_storage_discharge_mw[idx] +
                   result.q_storage_charge_mw[idx] + result.q_storage_discharge_mw[idx]);
    total_cost += data.hydrogen_storage_throughput_cost_per_mwh * dt *
                  (result.h_storage_charge_mw[idx] + result.h_storage_discharge_mw[idx] +
                   result.h_weekly_charge_mw[idx] + result.h_weekly_discharge_mw[idx] +
                   result.h_seasonal_charge_mw[idx] + result.h_seasonal_discharge_mw[idx] +
                   result.h_daily_to_weekly_mw[idx] + result.h_weekly_to_daily_mw[idx] +
                   result.h_weekly_to_seasonal_mw[idx] + result.h_seasonal_to_weekly_mw[idx]);
    total_cost += data.curtailment_cost_per_mwh * dt *
                  (result.p_solar_curtail_mw[idx] + result.p_wind_curtail_mw[idx]);
    total_cost += data.carbon_penalty_per_tco2 * result.carbon_residual_tco2[idx];
    total_cost += data.ccus_cost_per_tco2 * result.co2_captured_tco2[idx];
    total_cost += data.ccus_power_cost_per_mwh * dt * result.p_ccus_mw[idx];
  }
  result.total_cost = total_cost;

  add_sankey_flow(result.sankey_flows, "Grid", "Electric Bus",
                  result.total_grid_import_mwh, "electricity");
  add_sankey_flow(result.sankey_flows, "Solar", "Electric Bus",
                  sum_energy(result.p_solar_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Wind", "Electric Bus",
                  sum_energy(result.p_wind_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "CHP", "Electric Bus",
                  sum_energy(result.p_chp_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Fuel Cell", "Electric Bus",
                  sum_energy(result.p_fuelcell_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Storage", "Electric Bus",
                  sum_energy(result.p_storage_discharge_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Electric Load",
                  result.total_electric_load_mwh, "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Heat Pump",
                  sum_energy(result.p_heatpump_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Electrolyzer",
                  sum_energy(result.p_electrolysis_h2_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Power-to-Fuel",
                  sum_energy(result.p_electrolysis_fuel_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Electric Storage",
                  sum_energy(result.p_storage_charge_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "EV Charging",
                  sum_energy(result.p_ev_charge_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "CCUS",
                  sum_energy(result.p_ccus_mw, dt), "electricity");
  add_sankey_flow(result.sankey_flows, "Electric Bus", "Grid Export",
                  result.total_grid_export_mwh, "electricity");
  add_sankey_flow(result.sankey_flows, "Renewable Potential", "Curtailment",
                  result.total_curtailment_mwh, "electricity");

  add_sankey_flow(result.sankey_flows, "CHP", "Heat Bus",
                  sum_energy(result.q_chp_mw, dt), "heat");
  add_sankey_flow(result.sankey_flows, "Heat Pump", "Heat Bus",
                  sum_energy(result.q_heatpump_mw, dt), "heat");
  add_sankey_flow(result.sankey_flows, "Thermal Storage", "Heat Bus",
                  sum_energy(result.q_storage_discharge_mw, dt), "heat");
  add_sankey_flow(result.sankey_flows, "Heat Bus", "Heat Load",
                  result.total_heat_load_mwh, "heat");
  add_sankey_flow(result.sankey_flows, "Heat Bus", "Thermal Storage",
                  sum_energy(result.q_storage_charge_mw, dt), "heat");

  add_sankey_flow(result.sankey_flows, "Electrolyzer", "Hydrogen Bus",
                  sum_energy(result.h_electrolysis_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Storage", "Hydrogen Bus",
                  sum_energy(result.h_storage_discharge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Weekly H2 Storage", "Hydrogen Bus",
                  sum_energy(result.h_weekly_discharge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Seasonal H2 Storage", "Hydrogen Bus",
                  sum_energy(result.h_seasonal_discharge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "Hydrogen Load",
                  result.total_hydrogen_load_mwh, "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "Fuel Cell",
                  sum_energy(result.h_fuelcell_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "H2 Vehicles",
                  sum_energy(result.h_hv_refuel_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "Hydrogen Storage",
                  sum_energy(result.h_storage_charge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "Weekly H2 Storage",
                  sum_energy(result.h_weekly_charge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Bus", "Seasonal H2 Storage",
                  sum_energy(result.h_seasonal_charge_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Hydrogen Storage", "Weekly H2 Storage",
                  sum_energy(result.h_daily_to_weekly_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Weekly H2 Storage", "Hydrogen Storage",
                  sum_energy(result.h_weekly_to_daily_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Weekly H2 Storage", "Seasonal H2 Storage",
                  sum_energy(result.h_weekly_to_seasonal_mw, dt), "hydrogen");
  add_sankey_flow(result.sankey_flows, "Seasonal H2 Storage", "Weekly H2 Storage",
                  sum_energy(result.h_seasonal_to_weekly_mw, dt), "hydrogen");

  add_sankey_flow(result.sankey_flows, "External Fuel", "Fuel Bus",
                  sum_energy(result.f_external_mw, dt), "fuel");
  add_sankey_flow(result.sankey_flows, "Synthetic Fuel", "Fuel Bus",
                  sum_energy(result.f_synthetic_mw, dt), "fuel");
  add_sankey_flow(result.sankey_flows, "Fuel Bus", "CHP",
                  sum_energy(result.f_chp_mw, dt), "fuel");
  add_sankey_flow(result.sankey_flows, "Fuel Bus", "Fuel Load",
                  result.total_fuel_load_mwh, "fuel");
  add_sankey_flow(result.sankey_flows, "Fuel Bus", "ICV Vehicles",
                  sum_energy(result.f_icv_refuel_mw, dt), "fuel");

  return result;
}

}  // namespace hacdcpf::integrated_energy
