#pragma once

#include <string>
#include <vector>

namespace hacdcpf::dynamics {

// IEEE 1547-2018 abnormal-conditions performance categories for voltage and
// frequency ride-through (design doc §11.7). Category I is the least stringent
// (small distributed energy resources), Category III the most stringent
// (bulk-system support); the categories differ in how long a DER must ride
// through an abnormal voltage before it is permitted to trip.
enum class IEEE1547Category { CategoryI, CategoryII, CategoryIII };

// One must-trip entry of a ride-through band table. If the measured quantity
// stays beyond `threshold` (below it for the under-voltage / under-frequency
// tables, above it for the over-voltage / over-frequency tables) continuously
// for longer than `clearing_s`, the DER trips. `clearing_s <= 0` trips on the
// first violating step (instantaneous).
struct RideThroughTrip {
  double threshold{0.0};
  double clearing_s{0.0};
};

// IEEE 1547 ride-through + trip/reconnect settings for a single DER (design doc
// §11.7). The default-constructed value is disabled; make_default_ieee1547()
// fills the per-category band tables and the caller opts in by setting
// `enabled`. All quantities are per unit (voltage), Hz (frequency), or seconds.
struct IEEE1547Settings {
  bool enabled{false};
  IEEE1547Category category{IEEE1547Category::CategoryII};
  double nominal_frequency_hz{60.0};

  // Continuous-operation window. Used as the healthy band that qualifies a
  // reconnect (IEEE 1547 "enter service") after a trip.
  double v_continuous_min_pu{0.88};
  double v_continuous_max_pu{1.10};
  double f_continuous_min_hz{59.5};
  double f_continuous_max_hz{60.5};

  // Must-trip tables (the fastest applicable band governs).
  std::vector<RideThroughTrip> undervoltage_trip;    // trip if |V| < threshold
  std::vector<RideThroughTrip> overvoltage_trip;      // trip if |V| > threshold
  std::vector<RideThroughTrip> underfrequency_trip;   // trip if f  < threshold
  std::vector<RideThroughTrip> overfrequency_trip;    // trip if f  > threshold

  // Measurement low-pass filter time constants (§7 role 4: protection acts on
  // measured, filtered quantities, never on the raw algebraic voltage).
  double v_filter_t_s{0.01};
  double f_filter_t_s{0.05};

  // Reconnect: the DER must observe the continuous window uninterrupted for
  // `reconnect_delay_s`, then restores power over `power_ramp_s` (soft start).
  bool allow_reconnect{true};
  double reconnect_delay_s{5.0};
  double power_ramp_s{2.0};
};

// Mutable per-DER protection runtime state: the measurement filters, the
// per-threshold violation-time accumulators, the trip status, and the reconnect
// soft-start ramp scale. One instance lives on each protected device.
struct IEEE1547RuntimeState {
  bool initialized{false};
  bool tripped{false};
  double trip_time_s{-1.0};

  double v_meas_pu{1.0};
  double f_meas_hz{60.0};
  double prev_angle_rad{0.0};
  bool have_prev_angle{false};

  double continuous_ok_timer_s{0.0};  // time held inside the continuous window
  double restore_scale{1.0};          // injection soft-start ramp in [0, 1]

  std::vector<double> uv_timer_s;
  std::vector<double> ov_timer_s;
  std::vector<double> uf_timer_s;
  std::vector<double> of_timer_s;

  std::string last_reason;
};

// Status transition produced by one protection step.
enum class IEEE1547Action { None, Tripped, Reconnected };

// Result of advancing one protection interval. `action_offset_s` is measured
// from the beginning of the interval and is finite only when `action` is not
// None. The interval update is deterministic, so the same function can be run
// on a copied runtime state to preview an event without mutating the device.
struct IEEE1547StepResult {
  IEEE1547Action action{IEEE1547Action::None};
  double action_offset_s{0.0};
};

// Builds the default ride-through band tables for `category` at `nominal_hz`
// (IEEE 1547-2018 default trip settings; the 60 Hz frequency bands are scaled to
// the system nominal). Leaves `enabled` false so the caller opts in.
IEEE1547Settings make_default_ieee1547(IEEE1547Category category,
                                       double nominal_hz);

// Advances the protection state machine over one accepted interval. The raw
// terminal measurement is held constant over the interval and the first-order
// measurement filters are integrated analytically. Threshold-crossing times and
// continuous-violation durations are therefore resolved inside the interval
// instead of being rounded to its right endpoint. See Zhao--Hu (2008),
// DOI:10.1109/DRPT.2008.4523550, and docs/modules/dynamics/chapters/
// theory_machine_dae.tex, event-restart remark.
IEEE1547StepResult advance_ieee1547_interval(
    const IEEE1547Settings& settings,
    IEEE1547RuntimeState& state,
    double v_mag_pu,
    double angle_rad,
    double dt);

// Backward-compatible wrapper that discards the sub-step event time.
IEEE1547Action step_ieee1547(const IEEE1547Settings& settings,
                             IEEE1547RuntimeState& state,
                             double v_mag_pu,
                             double angle_rad,
                             double dt);

// IEEE 1547 volt-var smart-inverter function (design doc §11.7): a piecewise-
// linear reactive-power characteristic Q(V) with a deadband. Below the deadband
// the DER injects vars (voltage support); above it, it absorbs. Q is in per unit
// of the device base. Defaults approximate IEEE 1547-2018 Category B.
struct VoltVarSettings {
  bool enabled{false};
  double v1_pu{0.92};   // full injection at/below V1
  double q1_pu{0.44};
  double v2_pu{0.98};   // deadband lower edge (Q = 0)
  double v3_pu{1.02};   // deadband upper edge (Q = 0)
  double v4_pu{1.08};   // full absorption at/above V4
  double q4_pu{-0.44};
  double filter_t_s{1.0};          // open-loop response low-pass T_qf
  double ramp_rate_pu_per_s{0.0};  // slew limit (0 = unlimited)
};

// IEEE 1547 frequency-watt smart-inverter function (design doc §11.7): an active-
// power droop with a deadband about nominal frequency. Over-frequency curtails P;
// under-frequency raises it (subject to headroom). Delta is in per unit of base.
struct FreqWattSettings {
  bool enabled{false};
  double nominal_frequency_hz{60.0};
  double db_over_hz{0.036};
  double db_under_hz{0.036};
  double droop_over{0.05};   // per-unit frequency deviation per per-unit power
  double droop_under{0.05};
  double p_min_pu{0.0};      // curtailment floor (pu of base)
  double p_max_pu{1.20};     // headroom ceiling (pu of base)
  double filter_t_s{0.20};         // open-loop response low-pass T_pf
  double ramp_rate_pu_per_s{0.0};  // slew limit (0 = unlimited)
};

// Mutable per-DER smart-inverter runtime state: the filtered + rate-limited
// volt-var and frequency-watt references and the measured-frequency tracker. One
// instance lives on each device that implements the smart-inverter functions.
struct SmartInverterState {
  bool initialized{false};
  double q_pu{0.0};        // filtered + slew-limited volt-var reactive cmd (pu base)
  double p_delta_pu{0.0};  // filtered + slew-limited freq-watt active delta (pu base)
  double f_meas_hz{60.0};
  double prev_angle_rad{0.0};
  bool have_prev_angle{false};
};

// Volt-var reactive-power command (pu of base) at terminal voltage `v_pu`.
// Returns 0 when disabled or inside the deadband.
double volt_var_q_pu(const VoltVarSettings& settings, double v_pu);

// Frequency-watt active-power delta (pu of base) at measured frequency `f_hz`.
// Negative curtails (over-frequency), positive raises (under-frequency); 0 when
// disabled or inside the deadband.
double freq_watt_delta_pu(const FreqWattSettings& settings, double f_hz);

// Advances the smart-inverter references by one accepted step of length `dt`,
// given the terminal positive-sequence voltage magnitude `v_mag_pu` and angle
// `angle_rad` (rad, for the measured-frequency estimate). Applies the volt-var /
// frequency-watt curves, then the T_qf/T_pf low-pass and the slew-rate limit, and
// stores the filtered references in `state`. Pure: the caller reads state.q_pu /
// state.p_delta_pu into its control (design doc §11.7).
void step_smart_inverter(const VoltVarSettings& volt_var,
                         const FreqWattSettings& freq_watt,
                         SmartInverterState& state,
                         double v_mag_pu,
                         double angle_rad,
                         double dt);

}  // namespace hacdcpf::dynamics
