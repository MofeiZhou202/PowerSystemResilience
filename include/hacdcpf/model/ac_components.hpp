#pragma once

/// model/ac_components.hpp
/// =======================
/// Consolidated AC component types for the hacdcpf power system model.
/// Replaces: ac_network.hpp, generators.hpp, loads.hpp (AC loads),
///           storage.hpp, ev_charging.hpp, switching.hpp,
///           three_phase.hpp, island.hpp.

#include <array>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/enums/charging_enums.hpp"
#include "hacdcpf/model/enums/generator_enums.hpp"
#include "hacdcpf/model/enums/load_enums.hpp"
#include "hacdcpf/model/enums/renewable_enums.hpp"
#include "hacdcpf/model/enums/storage_enums.hpp"
#include "hacdcpf/model/enums/switching_enums.hpp"
#include "hacdcpf/model/dynamic_model_profile.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// AC Bus
// ═══════════════════════════════════════════════════════════════════════
struct ACBus {
  int index{0};
  BusType bus_type{BusType::PQ};
  double pd_mw{0.0};
  double qd_mvar{0.0};
  double vm_pu{1.0};
  double va_deg{0.0};
  int area{1};
  double base_kv{110.0};
  double vmax_pu{1.1};
  double vmin_pu{0.9};
  double gs_mw{0.0};
  double bs_mvar{0.0};
  int zone{1};
  bool in_service{true};
  std::string name;

  // Resilience / customer data (for planning, load priority)
  int n_customers{0};
  double importance{1.0};

  // Bearing capability assessment (DL/T 2041)
  double i_breaker_ka{0.0};

  // Geographical (for planning / visualization)
  double latitude{0.0};
  double longitude{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// AC Branch
// ═══════════════════════════════════════════════════════════════════════
struct ACBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  double r_pu{0.0};
  double x_pu{0.0};
  double b_pu{0.0};
  double tap{1.0};
  double shift_deg{0.0};
  double rate_a_mva{0.0};
  bool in_service{true};
  std::string name;

  double rate_b_mva{0.0};
  double rate_c_mva{0.0};

  double length_km{0.0};
  double r_ohm_per_km{0.0};
  double x_ohm_per_km{0.0};
  double b_us_per_km{0.0};
  double c_nf_per_km{0.0};

  // Physical conductor metadata retained by engineering-model importers.
  // These fields let the explicit Model IO completion workflow derive actual
  // units from a design handbook without overwriting authored per-unit data.
  std::string conductor_model;
  double cross_section_mm2{0.0};
  bool cross_section_inferred{false};
  std::string line_type;
  std::string parameter_source;
  bool parameters_inferred{false};

  // Reliability
  double failure_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
  int n_parallel{1};

  // Zero-sequence parameters
  double r0_pu{0.0};
  double x0_pu{0.0};
  double b0_pu{0.0};

  // Transformer nameplate
  double vn_hv_kv{0.0};
  double vn_lv_kv{0.0};
  double sn_mva{0.0};

  // Dynamics: when true, the transient builder represents the branch series
  // path as differential current states instead of a static Y-bus admittance.
  bool dynamic_rl{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Two-winding Transformer
// ═══════════════════════════════════════════════════════════════════════
struct Transformer2W {
  int index{0};
  std::string name;
  std::string std_type;
  int hv_bus{0};
  int lv_bus{0};
  bool in_service{true};

  double sn_mva{0.0};
  double vn_hv_kv{0.0};
  double vn_lv_kv{0.0};

  double vk_percent{0.0};
  double vkr_percent{0.0};
  double pk_kw{0.0};

  double pfe_kw{0.0};
  double i0_percent{0.0};

  int tap_side{0};
  int tap_pos{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  double tap_step_percent{0.0};

  double shift_deg{0.0};
  std::string vector_group;

  double z0_percent{0.0};
  double x0_r0{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  int n_parallel{1};
  int source_branch_idx{0};

  // Hosting-capacity assessment (DL/T 2041-2025, equipment-level)
  double cap_power_factor{0.95};             // cosθ used in β·S·cosθ
  double cap_max_reverse_load_rate{0.0};     // β; 0 = auto N-1 from n_parallel, >0 = manual
  double cap_dr_max_output_coeff{1.0};       // τ_max, max output coefficient of distributed resources
  double cap_registered_dr_mw{0.0};          // registered-but-not-connected DR under this transformer (S_d,reg)
  double cap_expected_new_storage_min_mw{0.0}; // ΔP_ESS lower bound (expected new storage charging)
  double cap_expected_new_storage_max_mw{0.0}; // ΔP_ESS upper bound
};

struct Transformer3W {
  int index{0};
  std::string name;
  std::string std_type;
  int hv_bus{0};
  int mv_bus{0};
  int lv_bus{0};
  bool in_service{true};

  double sn_hv_mva{0.0};
  double sn_mv_mva{0.0};
  double sn_lv_mva{0.0};
  double vn_hv_kv{0.0};
  double vn_mv_kv{0.0};
  double vn_lv_kv{0.0};

  double vk_hv_mv_percent{0.0};
  double vk_hv_lv_percent{0.0};
  double vk_mv_lv_percent{0.0};
  double vkr_hv_mv_percent{0.0};
  double vkr_hv_lv_percent{0.0};
  double vkr_mv_lv_percent{0.0};

  double pfe_kw{0.0};
  double i0_percent{0.0};

  int tap_side{0};
  int tap_pos{0};
  double tap_step_percent{0.0};

  double shift_mv_deg{0.0};
  double shift_lv_deg{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// External Grid (grid equivalent / infinite bus)
// ═══════════════════════════════════════════════════════════════════════
struct ExternalGrid {
  int index{0};
  std::string name;
  int bus{0};
  bool in_service{true};

  double vm_pu{1.0};
  double va_deg{0.0};

  double s_sc_max_mva{0.0};
  double s_sc_min_mva{0.0};
  double rx_max{0.0};
  double rx_min{0.0};

  double r_pu{0.0};
  double x_pu{0.0};
  double r0_pu{0.0};
  double x0_pu{0.0};

  double vn_kv{0.0};
  double ikq_ka{0.0};
  double x_r{0.0};
  bool controllable{true};

  double emission_factor_tco2_mwh{0.0};

  // OPF cost (0 = not participating in OPF objective)
  double cost_c2{0.0};
  double cost_c1{0.0};
  double cost_c0{0.0};

  // Time-varying electricity price: profile_id -> cost_c1 at each timestep.
  // -1 = no profile (use static cost_c1).
  int price_profile_id{-1};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Asynchronous Motor (IEC 60909 short-circuit analysis)
// ═══════════════════════════════════════════════════════════════════════
struct AsynchronousMotor {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double vn_kv{0.0};
  double sn_mva{0.0};
  double r_pu{0.0};
  double x_pu{0.0};
  double x_r{0.0};
  double lrc{0.0};
  int    poles{2};
  double cos_phi{0.85};
  double efficiency{0.95};
  double r0_pu{0.0};
  double x0_pu{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Generator
// ═══════════════════════════════════════════════════════════════════════
struct Generator {
  int index{0};
  int bus{0};
  bool in_service{true};
  double pg_mw{0.0};
  double qg_mvar{0.0};
  double vg_pu{1.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  bool is_slack{false};

  double cost_c2{0.0};
  double cost_c1{0.0};
  double cost_c0{0.0};

  std::string name;

  FuelType fuel_type{FuelType::Unknown};
  double startup_cost{0.0};
  double shutdown_cost{0.0};
  double min_up_time_hr{0.0};
  double min_dn_time_hr{0.0};
  double ramp_up_mw_min{0.0};
  double ramp_dn_mw_min{0.0};
  int max_startups_per_day{0};
  int max_shutdowns_per_day{0};
  double mbase_mva{0.0};

  double inertia_h{0.0};
  double droop_r{0.05};
  double xd_pu{0.0};
  double xdp_pu{0.0};
  double xdpp_pu{0.0};
  double td0p_s{0.0};
  double td0pp_s{0.0};

  double ra_pu{0.0};
  double vn_kv{0.0};
  double cos_phi{1.0};
  double xq_pu{0.0};
  double xd_xq{1.0};
  double x0_pu{0.0};
  double r0_pu{0.0};

  double emission_factor_tco2_mwh{0.0};
  double nox_factor_kg_mwh{0.0};
  double so2_factor_kg_mwh{0.0};

  int profile_id{-1};

  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Static Generator (distributed generation: PV inverter, wind, CHP, ...)
// ═══════════════════════════════════════════════════════════════════════
struct StaticGenerator {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  SgenType sgen_type{SgenType::Other};

  double p_mw{0.0};
  double q_mvar{0.0};
  double p_rated_mw{0.0};
  double sn_mva{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double scaling{1.0};

  bool controllable{false};
  bool grid_forming{false};
  bool anti_islanding{true};
  double v_ref_pu{1.0};
  double k_p{0.0};
  double k_q{0.0};

  double k{1.0};
  double rx{0.0};

  double co2_emission_rate{0.0};
  double cost_c1{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  double f_ref_hz{50.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Renewable Generator (wind / solar, non-dispatchable)
// ═══════════════════════════════════════════════════════════════════════
struct RenewableGen {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  RenewableType type{RenewableType::Wind};

  double p_mw{0.0};
  double q_mvar{0.0};
  double p_rated_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  bool curtailable{true};
  bool grid_forming{false};
  bool anti_islanding{true};
  double cost_c1{0.0};
  double cost_curtail_mwh{0.0};
  double capacity_factor{0.3};

  int profile_id{-1};

  double emission_offset_tco2_mwh{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// PV System (solar PV with inverter)
// ═══════════════════════════════════════════════════════════════════════
struct PVSystem {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};
  double q_mvar{0.0};
  double sn_mva{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  PVControlMode control_mode{PVControlMode::MPPT};
  bool controllable{false};
  bool grid_forming{false};
  bool anti_islanding{true};
  double v_ac_set_pu{1.0};
  double v_dc_set_pu{1.0};

  double inverter_eff{0.97};
  double loss_percent{0.0};

  int num_series{0};
  int num_parallel{0};
  double vmpp{0.0};
  double impp{0.0};
  double voc{0.0};
  double isc{0.0};
  double alpha_isc{0.0};
  double beta_voc{0.0};

  double irradiance{1000.0};
  double temperature{25.0};

  int profile_id{-1};

  double cost_c1{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  double mtbf_panel_hours{0.0};
  double mttr_panel_hours{0.0};
  double mtbf_inverter_hours{0.0};
  double mttr_inverter_hours{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Load (first-class, decoupled from bus)
// ═══════════════════════════════════════════════════════════════════════
struct Load {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};
  double q_mvar{0.0};
  double scaling{1.0};

  LoadModel model{LoadModel::ConstantPower};

  double z_percent_p{0.0};
  double i_percent_p{0.0};
  double p_percent_p{100.0};
  double z_percent_q{0.0};
  double i_percent_q{0.0};
  double p_percent_q{100.0};

  bool controllable{false};
  double p_min_mw{0.0};
  double cost_mw{0.0};

  LoadPriority priority{LoadPriority::Medium};
  int n_customers{0};

  int profile_id{-1};

  double sn_mva{0.0};
  double motor_percent{0.0};
  double x_sub_pu{0.0};
  double r_sc_pu{0.0};

  // Provenance for IEC 60909 motor contribution after rich AsynchronousMotor
  // projection.  Empty/zero values keep legacy load motor-fraction semantics.
  std::string sc_source_type;
  int sc_source_index{0};
  int motor_poles{2};
  double motor_efficiency{0.95};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Flexible Load (demand response)
// ═══════════════════════════════════════════════════════════════════════
struct FlexibleLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};
  double q_mvar{0.0};

  double flex_up_mw{0.0};
  double flex_down_mw{0.0};
  double flex_duration_h{0.0};

  double response_time_s{0.0};
  double ramp_rate_mw_min{0.0};
  double availability_pct{100.0};

  bool controllable{true};
  LoadPriority priority{LoadPriority::Medium};
  std::string control_area;
};

// ═══════════════════════════════════════════════════════════════════════
// Asymmetric Load (three-phase unbalanced)
// ═══════════════════════════════════════════════════════════════════════
struct AsymmetricLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  std::string connection;
  bool grounded{true};

  double pa_rated_mw{0.0};  double qa_rated_mvar{0.0};
  double pb_rated_mw{0.0};  double qb_rated_mvar{0.0};
  double pc_rated_mw{0.0};  double qc_rated_mvar{0.0};

  double pa_mw{0.0};  double qa_mvar{0.0};
  double pb_mw{0.0};  double qb_mvar{0.0};
  double pc_mw{0.0};  double qc_mvar{0.0};

  double scaling{1.0};

  double const_z_percent{0.0};
  double const_i_percent{0.0};
  double const_p_percent{100.0};

  bool controllable{false};
  LoadPriority priority{LoadPriority::Medium};
  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Energy Storage (battery, pumped hydro, etc.)
// ═══════════════════════════════════════════════════════════════════════
struct Storage {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};
  double q_mvar{0.0};
  double p_rated_mw{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  double e_rated_mwh{0.0};
  double soc_init{0.5};
  double soc_min{0.1};
  double soc_max{0.9};
  double soc_carbon_intensity_tco2_mwh{0.0};

  double eta_charge{0.95};
  double eta_discharge{0.95};
  double self_discharge_pct{0.0};

  int max_cycles{5000};
  int current_cycles{0};
  double soh{1.0};
  double soh_cycle{1.0};
  double soh_calendar{1.0};
  double l_calendar_yr{15.0};
  double eol_percent{0.8};
  double replacement_cost{0.0};

  double e_mwh{0.0};

  int profile_id{-1};

  bool controllable{true};
  bool grid_forming{false};       // true = can black-start / anchor an island in restoration studies
  bool anti_islanding{true};      // disconnects if no external or grid-forming voltage anchor remains
  std::string control_mode;
  std::string type;

  double charge_bid_price{0.0};
  double discharge_bid_price{0.0};
  double daily_cycle_limit{0.0};

  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};

  double mtbf_battery_hr{0.0};
  double mttr_battery_hr{0.0};
  double mtbf_pcs_hr{0.0};
  double mttr_pcs_hr{0.0};
  double mtbf_bms_hr{0.0};
  double mttr_bms_hr{0.0};

  // Hosting-capacity assessment (DL/T 2041-2025): charging strategy.
  // "opf"    = optimized in OPF/UC (default, as today);
  // "static" = fixed injection, excluded from optimization, held at cap_static_charging_mw.
  std::string cap_charging_strategy{"opf"};
  double cap_static_charging_mw{0.0};        // charging power (positive) used when strategy=="static" (P_ESS)

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Mobile Storage (EV battery trailer / transportable ESS)
// ═══════════════════════════════════════════════════════════════════════
struct MobileStorage {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};
  double q_mvar{0.0};
  double p_rated_mw{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double e_rated_mwh{0.0};
  double soc_init{0.5};
  double soc_min{0.1};
  double soc_max{0.9};
  double eta_charge{0.95};
  double eta_discharge{0.95};

  int max_cycles{5000};
  int current_cycles{0};
  double soh{1.0};
  double soh_cycle{1.0};
  double soh_calendar{1.0};
  double l_calendar_yr{10.0};
  double eol_percent{0.8};

  double e_mwh{0.0};

  bool controllable{true};
  std::string type;

  bool is_mobile{true};
  MobileStorageStatus status{MobileStorageStatus::Stationary};
  std::string current_location;
  int target_bus{0};
  double e_consumption_mwh_km{0.0};
  double max_travel_distance_km{0.0};
  double arrival_time{0.0};
  double departure_time{0.0};
  double t_stay_min_hr{0.0};
  double t_stay_max_hr{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  double mtbf_battery_hr{0.0};
  double mttr_battery_hr{0.0};
  double mtbf_pcs_hr{0.0};
  double mttr_pcs_hr{0.0};
  double mtbf_bms_hr{0.0};
  double mttr_bms_hr{0.0};
  double mtbf_vehicle_hr{0.0};
  double mttr_vehicle_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Charger (individual EV charging unit)
// ═══════════════════════════════════════════════════════════════════════
struct Charger {
  int index{0};
  std::string name;
  int station_id{0};
  ChargerType charger_type{ChargerType::AC_L2};
  bool in_service{true};

  double p_rated_kw{0.0};
  double p_ch_max_kw{0.0};
  double p_ch_min_kw{0.0};
  double eta{0.95};

  bool v2g_capable{false};
  double p_dis_max_kw{0.0};

  std::string connector_type;
  std::string status;
  double p_ev_kw{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Charging Station (EV charging site)
// ═══════════════════════════════════════════════════════════════════════
struct ChargingStation {
  int index{0};
  std::string name;
  int bus{0};
  std::string location;
  bool in_service{true};

  int n_fast{0};
  int n_slow{0};
  int num_chargers{0};

  double p_fast_max_kw{0.0};
  double p_slow_max_kw{0.0};
  double max_power_kw{0.0};

  double simultaneity_factor{1.0};
  double power_factor{0.95};
  double utilization_rate{0.0};

  double p_total_kw{0.0};
  double q_total_kvar{0.0};

  int n_cars{0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Switch
// ═══════════════════════════════════════════════════════════════════════
struct SwitchCapabilities {
  bool can_interrupt_fault_current{false};
  bool can_interrupt_load_current{false};
  bool can_close_for_restoration{false};
  bool requires_deenergized_operation{false};
  bool allows_source_parallel{false};
};

struct FuseProtection {
  std::string curve_type;
  double rated_current_a{0.0};
  double minimum_melting_current_a{0.0};
  double total_clearing_time_s{0.0};
  bool replace_after_operation{true};
  double replacement_time_hr{0.0};
};

struct RecloserProtection {
  std::string curve_type;
  int max_reclose_attempts{0};
  std::vector<double> reclose_intervals_s;
  double lockout_time_s{0.0};
  double successful_reclose_probability{0.0};
};

struct SectionalizerProtection {
  int fault_count_to_open{0};
  int upstream_switch_index{-1};
  bool opens_during_dead_time{true};
};

struct Switch {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  SwitchType switch_type{SwitchType::CircuitBreaker};
  bool closed{true};

  // Normal planning state is distinct from the current handle state.  Legacy
  // JSON without normal_closed is read with normal_closed == closed.
  bool normal_closed{true};
  bool normal_state_explicit{false};
  bool locked_open{false};
  bool locked_closed{false};
  SwitchRole role{SwitchRole::Unspecified};
  SwitchOperatingMode operating_mode{SwitchOperatingMode::Manual};
  int protection_zone_id{0};
  // Generic protection dependency used by disconnectors, sectionalizers and
  // other boundary devices that may only operate after an upstream trip.
  int upstream_protective_switch_index{-1};
  int controlled_branch_index{-1};
  std::string controlled_element_type;
  int controlled_element_index{-1};
  int interlock_group_id{0};
  bool synchronization_required{false};
  bool binding_inferred{false};
  std::string binding_source;

  // When false, algorithms derive conservative backward-compatible defaults
  // from switch_type.  CIM and edited models should set this true.
  bool capabilities_explicit{false};
  SwitchCapabilities capabilities{};

  FuseProtection fuse_protection{};
  RecloserProtection recloser_protection{};
  SectionalizerProtection sectionalizer_protection{};

  double r_contact_ohm{0.0};
  double z_ohm{0.0};
  double i_rated_ka{0.0};
  double i_breaking_ka{0.0};

  int element_type{0};
  int element_id{0};

  bool is_remote{false};
  bool is_automated{false};
  double t_operation_s{0.0};
  double t_open_s{0.0};
  double t_close_s{0.0};

  double p_sw_fail{0.0};
  double p_fail_to_open{0.0};
  double p_fail_to_close{0.0};
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
  double t_tp_hr{0.0};
};

inline SwitchCapabilities effective_switch_capabilities(const Switch& sw) {
  if (sw.capabilities_explicit) return sw.capabilities;
  SwitchCapabilities caps;
  switch (sw.switch_type) {
    case SwitchType::CircuitBreaker:
    case SwitchType::Recloser:
      caps.can_interrupt_fault_current = true;
      caps.can_interrupt_load_current = true;
      caps.can_close_for_restoration = true;
      break;
    case SwitchType::LoadBreakSwitch:
      caps.can_interrupt_load_current = true;
      caps.can_close_for_restoration = true;
      break;
    case SwitchType::Sectionalizer:
      caps.requires_deenergized_operation = true;
      break;
    case SwitchType::Disconnector:
      caps.requires_deenergized_operation = true;
      break;
    case SwitchType::Fuse:
      caps.can_interrupt_fault_current = true;
      break;
    case SwitchType::Unknown:
      break;
  }
  return caps;
}

inline bool effective_switch_normal_closed(const Switch& sw) {
  return sw.normal_state_explicit ? sw.normal_closed : sw.closed;
}

// ═══════════════════════════════════════════════════════════════════════
// Circuit Breaker (HVCB)
// ═══════════════════════════════════════════════════════════════════════
struct CircuitBreaker {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  BreakerType breaker_type{BreakerType::CB};
  bool closed{true};

  double z_ohm{0.0};
  double rated_voltage_kv{0.0};
  double i_rated_ka{0.0};
  double i_breaking_ka{0.0};

  std::string element_type;
  int element_id{0};
};

// ═══════════════════════════════════════════════════════════════════════
// Shunt (fixed or switchable)
// ═══════════════════════════════════════════════════════════════════════
struct Shunt {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double gs_mw{0.0};
  double bs_mvar{0.0};

  bool switchable{false};
  int n_steps{1};
  int current_step{1};
  double bs_per_step{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase AC Types
// ═══════════════════════════════════════════════════════════════════════

enum class Phase : std::uint8_t {
  A = 0,
  B = 1,
  C = 2,
};

constexpr int phase_to_index(Phase phase) {
  return static_cast<int>(phase);
}

using PhaseValueMatrix3 = std::array<double, 9>;

constexpr int phase_matrix_offset(int row, int col) {
  return row * 3 + col;
}

inline double phase_matrix_get(const PhaseValueMatrix3& matrix, int row, int col) {
  return matrix[static_cast<std::size_t>(phase_matrix_offset(row, col))];
}

inline void phase_matrix_set(PhaseValueMatrix3& matrix, int row, int col, double value) {
  matrix[static_cast<std::size_t>(phase_matrix_offset(row, col))] = value;
}

struct PhaseMask {
  std::uint8_t bits{0x7};

  constexpr PhaseMask() = default;
  constexpr explicit PhaseMask(std::uint8_t raw_bits)
      : bits(static_cast<std::uint8_t>(raw_bits & 0x7)) {}

  static constexpr PhaseMask none() { return PhaseMask(0x0); }
  static constexpr PhaseMask a()   { return PhaseMask(0x1); }
  static constexpr PhaseMask b()   { return PhaseMask(0x2); }
  static constexpr PhaseMask c()   { return PhaseMask(0x4); }
  static constexpr PhaseMask ab()  { return PhaseMask(0x3); }
  static constexpr PhaseMask ac()  { return PhaseMask(0x5); }
  static constexpr PhaseMask bc()  { return PhaseMask(0x6); }
  static constexpr PhaseMask abc() { return PhaseMask(0x7); }

  constexpr bool empty() const { return bits == 0; }
  constexpr bool has(int phase_index) const {
    return phase_index >= 0 && phase_index < 3 &&
           (bits & static_cast<std::uint8_t>(1u << phase_index)) != 0;
  }
  constexpr bool has(Phase phase) const { return has(phase_to_index(phase)); }
  constexpr bool contains(PhaseMask other) const {
    return (bits & other.bits) == other.bits;
  }
  constexpr int count() const {
    return (has(0) ? 1 : 0) + (has(1) ? 1 : 0) + (has(2) ? 1 : 0);
  }
};

inline std::string phase_mask_to_string(PhaseMask mask) {
  std::string text;
  if (mask.has(Phase::A)) text.push_back('A');
  if (mask.has(Phase::B)) text.push_back('B');
  if (mask.has(Phase::C)) text.push_back('C');
  return text;
}

inline PhaseMask phase_mask_from_string(const std::string& text,
                                         PhaseMask fallback = PhaseMask::abc()) {
  if (text.empty()) return fallback;
  std::uint8_t b = 0;
  for (unsigned char raw_ch : text) {
    const char ch = static_cast<char>(std::toupper(raw_ch));
    if (ch == 'A' || ch == '1') b |= 0x1;
    if (ch == 'B' || ch == '2') b |= 0x2;
    if (ch == 'C' || ch == '3') b |= 0x4;
  }
  return b == 0 ? fallback : PhaseMask(b);
}

struct ThreePhaseACBus {
  int index{0};
  BusType bus_type{BusType::PQ};
  std::string name;
  double base_kv{0.0};
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  double vm_a_pu{1.0};  double va_a_deg{0.0};
  double vm_b_pu{1.0};  double va_b_deg{-120.0};
  double vm_c_pu{1.0};  double va_c_deg{120.0};

  double pd_a_mw{0.0};   double qd_a_mvar{0.0};
  double pd_b_mw{0.0};   double qd_b_mvar{0.0};
  double pd_c_mw{0.0};   double qd_c_mvar{0.0};

  double gs_a_mw{0.0};   double bs_a_mvar{0.0};
  double gs_b_mw{0.0};   double bs_b_mvar{0.0};
  double gs_c_mw{0.0};   double bs_c_mvar{0.0};

  double vmin_pu{0.9};
  double vmax_pu{1.1};

  int area{1};
  int zone{1};
};

struct ThreePhaseACLine {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  double length_km{0.0};
  int parallel{1};

  double r1_ohm_per_km{0.0};
  double x1_ohm_per_km{0.0};
  double c1_nf_per_km{0.0};

  double r0_ohm_per_km{0.0};
  double x0_ohm_per_km{0.0};
  double c0_nf_per_km{0.0};

  double r1_pu{0.0};  double x1_pu{0.0};  double b1_pu{0.0};
  double r0_pu{0.0};  double x0_pu{0.0};  double b0_pu{0.0};

  bool use_phase_matrix{false};
  PhaseValueMatrix3 r_matrix_pu{};
  PhaseValueMatrix3 x_matrix_pu{};
  PhaseValueMatrix3 b_matrix_pu{};

  double max_i_ka{0.0};
  double rate_a_mva{0.0};

  double failure_rate{0.0};
  double mttr_hr{0.0};
};

struct ThreePhaseTransformer {
  int index{0};
  std::string name;
  int hv_bus{0};
  int lv_bus{0};
  bool in_service{true};
  PhaseMask hv_phase_mask{PhaseMask::abc()};
  PhaseMask lv_phase_mask{PhaseMask::abc()};

  double sn_mva{0.0};
  double vn_hv_kv{0.0};
  double vn_lv_kv{0.0};

  double vk_percent{0.0};
  double vkr_percent{0.0};
  double pfe_kw{0.0};
  double i0_percent{0.0};

  std::string vector_group;
  std::string hv_winding_topology;
  std::string lv_winding_topology;

  double vk0_percent{0.0};
  double vkr0_percent{0.0};
  double mag0_percent{0.0};
  double mag0_rx{0.0};
  double si0_hv_partial{0.5};

  int tap_side{0};
  int tap_pos{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  double tap_step_percent{0.0};

  double shift_deg{0.0};

  double mtbf_hr{0.0};
  double mttr_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Regulator control (DSS-aligned control object)
// ═══════════════════════════════════════════════════════════════════════
struct RegulatorControl {
  int index{0};
  std::string name;

  int transformer_index{0};
  std::string transformer_name;

  int winding{0};
  int tap_winding{0};
  int monitored_bus{0};
  int monitored_node{1};
  double vreg_volts{0.0};
  double band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int max_tap_change{1};
  bool reversible{false};
  bool enabled{true};
};

struct ThreePhaseRegulatorControl {
  int index{0};
  std::string name;

  int transformer_index{0};
  std::string transformer_name;

  int winding{0};
  int tap_winding{0};
  int monitored_bus{0};
  int monitored_node{1};
  double vreg_volts{0.0};
  double band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int max_tap_change{1};
  bool reversible{false};
  bool enabled{true};
};

struct ThreePhaseLoad {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  std::string connection{"wye"};
  bool grounded{true};
  double r_neut_ohm{0.0};
  double x_neut_ohm{0.0};

  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};

  double vmin_pu{0.95};
  double vmax_pu{1.05};
  double zipv_cutoff_pu{0.0};

  double const_z_percent{0.0};
  double const_i_percent{0.0};
  double const_p_percent{100.0};

  double p_const_z_percent{-1.0};
  double p_const_i_percent{-1.0};
  double p_const_p_percent{-1.0};
  double q_const_z_percent{-1.0};
  double q_const_i_percent{-1.0};
  double q_const_p_percent{-1.0};

  double motor_percent{0.0};
  double lrc_pu{0.0};
  double x_r_ratio{0.0};

  DynamicModelProfile dynamic_model;
};

struct ThreePhaseGenerator {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  bool is_slack{false};
  PhaseMask phase_mask{PhaseMask::abc()};

  double p_mw{0.0};
  double q_mvar{0.0};

  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};

  double vm_pu{1.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double mbase_mva{0.0};

  double xd_pu{0.0};
  double xdpp_pu{0.0};
  double x2_pu{0.0};
  double x0_pu{0.0};
  double r0_pu{0.0};

  DynamicModelProfile dynamic_model;
};

struct ThreePhaseExternalGrid {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  double vm_pu{1.0};
  double va_deg{0.0};

  bool use_phase_voltage_setpoint{false};
  double vm_a_pu{0.0};  double va_a_deg{0.0};
  double vm_b_pu{0.0};  double va_b_deg{0.0};
  double vm_c_pu{0.0};  double va_c_deg{0.0};

  std::string source_topology;

  double s_sc_max_mva{0.0};
  double s_sc_min_mva{0.0};
  double rx_max{0.0};
  double rx_min{0.0};

  double r1_pu{0.0};
  double x1_pu{0.0};
  double r2_pu{0.0};
  double x2_pu{0.0};
  double r0_pu{0.0};
  double x0_pu{0.0};

  bool use_phase_impedance{false};
  double r_a_pu{0.0};  double x_a_pu{0.0};
  double r_b_pu{0.0};  double x_b_pu{0.0};
  double r_c_pu{0.0};  double x_c_pu{0.0};

  bool use_phase_impedance_matrix{false};
  PhaseValueMatrix3 r_matrix_pu{};
  PhaseValueMatrix3 x_matrix_pu{};

  DynamicModelProfile dynamic_model;
};

struct ThreePhaseACSystem {
  std::vector<ThreePhaseACBus> buses;
  std::vector<ThreePhaseACLine> lines;
  std::vector<ThreePhaseTransformer> transformers;
  std::vector<ThreePhaseLoad> loads;
  std::vector<ThreePhaseGenerator> generators;
  std::vector<ThreePhaseExternalGrid> external_grids;
  std::vector<ThreePhaseRegulatorControl> regulator_controls;

  double base_mva{100.0};
  double base_freq_hz{50.0};
  std::string name{"Three-Phase AC System"};
};

struct ThreePhaseBusResult {
  int bus_id{0};

  double vm_a_pu{0.0};  double va_a_deg{0.0};
  double vm_b_pu{0.0};  double va_b_deg{0.0};
  double vm_c_pu{0.0};  double va_c_deg{0.0};

  double p_a_mw{0.0};   double q_a_mvar{0.0};
  double p_b_mw{0.0};   double q_b_mvar{0.0};
  double p_c_mw{0.0};   double q_c_mvar{0.0};

  bool is_unbalanced{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Island tracking (solver internal)
// ═══════════════════════════════════════════════════════════════════════
struct IslandInfo {
  int id{0};
  bool has_ac_slack{false};
  bool has_dc_slack{false};
  int ac_slack_bus{0};
  int dc_slack_bus{0};
  bool has_generators{false};

  std::vector<int> ac_buses;
  std::vector<int> dc_buses;
  std::vector<int> converters;
};

}  // namespace hacdcpf
