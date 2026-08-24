#include "hacdcpf/dynamics/devices/IEEE1547Protection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hacdcpf::dynamics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Comparison-only guard, six orders below the public 1 us localization default;
// derivation and numerical boundary: dynamics manual Eq. event-filter/event-cost.
constexpr double kFilterEpsilon = 1e-12;

// Exact solution of dm/dt=(target-m)/tau for a piecewise-constant raw
// measurement. This is the protection measurement model used by IEEE 1547
// interval event localization; it removes the step-size-dependent forward-Euler
// filter error from the old post-step implementation. Song et al. (2016),
// DOI:10.1109/TPWRS.2015.2439237, Sec. II-B; derivation in the dynamics manual.
struct FirstOrderSignal {
  double initial{0.0};
  double target{0.0};
  double tau_s{0.0};

  [[nodiscard]] double value(double t) const {
    if (tau_s <= kFilterEpsilon) return target;
    return target + (initial - target) * std::exp(-std::max(0.0, t) / tau_s);
  }

  [[nodiscard]] double crossing(double threshold, double dt) const {
    if (tau_s <= kFilterEpsilon) return 0.0;
    const double denominator = initial - target;
    if (std::abs(denominator) <= kFilterEpsilon) return dt;
    const double ratio = (threshold - target) / denominator;
    if (!(ratio > 0.0) || !std::isfinite(ratio)) return dt;
    return std::clamp(-tau_s * std::log(ratio), 0.0, dt);
  }
};

struct TimeWindow {
  bool valid{false};
  double begin_s{0.0};
  double end_s{0.0};
};

TimeWindow comparison_window(const FirstOrderSignal& signal,
                             double threshold,
                             double dt,
                             bool below) {
  const auto satisfies = [&](double value) {
    return below ? value < threshold : value > threshold;
  };
  const bool at_start = satisfies(signal.value(0.0));
  const bool at_end = satisfies(signal.value(dt));
  if (at_start && at_end) return {true, 0.0, dt};
  if (!at_start && !at_end) return {};
  const double crossing_s = signal.crossing(threshold, dt);
  return at_start ? TimeWindow{true, 0.0, crossing_s}
                  : TimeWindow{true, crossing_s, dt};
}

TimeWindow inclusive_window(const FirstOrderSignal& signal,
                            double threshold,
                            double dt,
                            bool at_least) {
  const auto satisfies = [&](double value) {
    return at_least ? value >= threshold : value <= threshold;
  };
  const bool at_start = satisfies(signal.value(0.0));
  const bool at_end = satisfies(signal.value(dt));
  if (at_start && at_end) return {true, 0.0, dt};
  if (!at_start && !at_end) return {};
  const double crossing_s = signal.crossing(threshold, dt);
  return at_start ? TimeWindow{true, 0.0, crossing_s}
                  : TimeWindow{true, crossing_s, dt};
}

TimeWindow intersect_windows(const TimeWindow& lhs, const TimeWindow& rhs) {
  if (!lhs.valid || !rhs.valid) return {};
  const double begin_s = std::max(lhs.begin_s, rhs.begin_s);
  const double end_s = std::min(lhs.end_s, rhs.end_s);
  if (end_s + kFilterEpsilon < begin_s) return {};
  return {true, begin_s, end_s};
}

double band_event_time(const RideThroughTrip& band,
                       double timer_s,
                       const TimeWindow& violation) {
  if (!violation.valid) return std::numeric_limits<double>::infinity();
  const double carried_timer = violation.begin_s <= kFilterEpsilon ? timer_s : 0.0;
  const double remaining_s = std::max(0.0, band.clearing_s - carried_timer);
  const double event_s = violation.begin_s + remaining_s;
  return event_s <= violation.end_s + kFilterEpsilon
             ? event_s
             : std::numeric_limits<double>::infinity();
}

void advance_band_timers(const std::vector<RideThroughTrip>& table,
                         std::vector<double>& timers,
                         const FirstOrderSignal& signal,
                         bool below,
                         double full_dt,
                         double advance_dt) {
  if (timers.size() != table.size()) timers.assign(table.size(), 0.0);
  for (std::size_t i = 0; i < table.size(); ++i) {
    const TimeWindow violation =
        comparison_window(signal, table[i].threshold, full_dt, below);
    if (!violation.valid || advance_dt + kFilterEpsilon < violation.begin_s) {
      timers[i] = 0.0;
      continue;
    }
    const double active_end = std::min(advance_dt, violation.end_s);
    if (active_end + kFilterEpsilon < violation.begin_s) {
      timers[i] = 0.0;
      continue;
    }
    const double carried = violation.begin_s <= kFilterEpsilon ? timers[i] : 0.0;
    timers[i] = carried + std::max(0.0, active_end - violation.begin_s);
    if (violation.end_s + kFilterEpsilon < advance_dt) timers[i] = 0.0;
  }
}

void consider_band_events(const std::vector<RideThroughTrip>& table,
                          const std::vector<double>& timers,
                          const FirstOrderSignal& signal,
                          bool below,
                          double dt,
                          const char* tag,
                          double& earliest_s,
                          std::string& reason) {
  for (std::size_t i = 0; i < table.size(); ++i) {
    const double timer_s = i < timers.size() ? timers[i] : 0.0;
    const TimeWindow violation =
        comparison_window(signal, table[i].threshold, dt, below);
    const double candidate_s = band_event_time(table[i], timer_s, violation);
    if (candidate_s + kFilterEpsilon < earliest_s) {
      earliest_s = candidate_s;
      reason = tag;
    }
  }
}

void reset_timers(IEEE1547RuntimeState& st) {
  std::fill(st.uv_timer_s.begin(), st.uv_timer_s.end(), 0.0);
  std::fill(st.ov_timer_s.begin(), st.ov_timer_s.end(), 0.0);
  std::fill(st.uf_timer_s.begin(), st.uf_timer_s.end(), 0.0);
  std::fill(st.of_timer_s.begin(), st.of_timer_s.end(), 0.0);
}

}  // namespace

IEEE1547Settings make_default_ieee1547(IEEE1547Category category,
                                       double nominal_hz) {
  const double fn = nominal_hz > 0.0 ? nominal_hz : 60.0;
  const double scale = fn / 60.0;  // scale the 60 Hz standard bands to nominal
  IEEE1547Settings cfg;
  cfg.category = category;
  cfg.nominal_frequency_hz = fn;
  cfg.v_continuous_min_pu = 0.88;
  cfg.v_continuous_max_pu = 1.10;
  cfg.f_continuous_min_hz = 59.5 * scale;
  cfg.f_continuous_max_hz = 60.5 * scale;

  // Frequency trip is category-independent in IEEE 1547-2018 (Table 22
  // defaults): a shallow excursion may persist for minutes, a deep one clears
  // in ~10 cycles.
  cfg.underfrequency_trip = {{58.5 * scale, 300.0}, {56.5 * scale, 0.16}};
  cfg.overfrequency_trip = {{61.2 * scale, 300.0}, {62.0 * scale, 0.16}};

  // Voltage must-trip (approximate IEEE 1547-2018 defaults; deeper excursions
  // clear faster, and the higher category rides through longer). Thresholds are
  // fractions of nominal voltage.
  switch (category) {
    case IEEE1547Category::CategoryI:
      cfg.undervoltage_trip = {{0.88, 2.0}, {0.50, 0.16}};
      cfg.overvoltage_trip = {{1.10, 1.0}, {1.20, 0.16}};
      break;
    case IEEE1547Category::CategoryIII:
      cfg.undervoltage_trip = {{0.88, 20.0}, {0.50, 2.0}};
      cfg.overvoltage_trip = {{1.10, 13.0}, {1.20, 0.16}};
      break;
    case IEEE1547Category::CategoryII:
    default:
      cfg.undervoltage_trip = {{0.88, 10.0}, {0.50, 0.16}};
      cfg.overvoltage_trip = {{1.10, 13.0}, {1.20, 0.16}};
      break;
  }
  return cfg;
}

IEEE1547StepResult advance_ieee1547_interval(const IEEE1547Settings& s,
                                             IEEE1547RuntimeState& st,
                                             double v_mag_pu,
                                             double angle_rad,
                                             double dt) {
  IEEE1547StepResult result;
  if (!s.enabled) return result;
  dt = std::max(0.0, dt);

  double angle_delta = 0.0;
  double f_inst = s.nominal_frequency_hz;
  if (st.have_prev_angle && dt > 0.0) {
    angle_delta = angle_rad - st.prev_angle_rad;
    while (angle_delta > kPi) angle_delta -= kTwoPi;
    while (angle_delta <= -kPi) angle_delta += kTwoPi;
    f_inst = s.nominal_frequency_hz + (angle_delta / dt) / kTwoPi;
  }
  if (!st.initialized) {
    st.v_meas_pu = v_mag_pu;
    st.f_meas_hz = s.nominal_frequency_hz;
    st.initialized = true;
  }

  // IEEE 1547 acts on measured quantities. Under the interval's zero-order-hold
  // input, the filter solution below is exact; event localization then operates
  // on this continuous curve rather than a right-endpoint sample.
  const FirstOrderSignal voltage{st.v_meas_pu, v_mag_pu, s.v_filter_t_s};
  const FirstOrderSignal frequency{st.f_meas_hz, f_inst, s.f_filter_t_s};
  double action_s = std::numeric_limits<double>::infinity();
  std::string reason;

  if (!st.tripped) {
    consider_band_events(s.undervoltage_trip, st.uv_timer_s, voltage, true, dt,
                         "undervoltage", action_s, reason);
    consider_band_events(s.overvoltage_trip, st.ov_timer_s, voltage, false, dt,
                         "overvoltage", action_s, reason);
    consider_band_events(s.underfrequency_trip, st.uf_timer_s, frequency, true, dt,
                         "underfrequency", action_s, reason);
    consider_band_events(s.overfrequency_trip, st.of_timer_s, frequency, false, dt,
                         "overfrequency", action_s, reason);
  } else if (s.allow_reconnect) {
    TimeWindow healthy = inclusive_window(
        voltage, s.v_continuous_min_pu, dt, true);
    healthy = intersect_windows(
        healthy, inclusive_window(voltage, s.v_continuous_max_pu, dt, false));
    healthy = intersect_windows(
        healthy, inclusive_window(frequency, s.f_continuous_min_hz, dt, true));
    healthy = intersect_windows(
        healthy, inclusive_window(frequency, s.f_continuous_max_hz, dt, false));
    if (healthy.valid) {
      const double carried =
          healthy.begin_s <= kFilterEpsilon ? st.continuous_ok_timer_s : 0.0;
      const double remaining_s = std::max(0.0, s.reconnect_delay_s - carried);
      const double candidate_s = healthy.begin_s + remaining_s;
      if (candidate_s <= healthy.end_s + kFilterEpsilon) action_s = candidate_s;
    }
  }

  const bool has_action = std::isfinite(action_s);
  const double advance_dt = has_action ? std::clamp(action_s, 0.0, dt) : dt;
  advance_band_timers(s.undervoltage_trip, st.uv_timer_s, voltage, true, dt,
                      advance_dt);
  advance_band_timers(s.overvoltage_trip, st.ov_timer_s, voltage, false, dt,
                      advance_dt);
  advance_band_timers(s.underfrequency_trip, st.uf_timer_s, frequency, true, dt,
                      advance_dt);
  advance_band_timers(s.overfrequency_trip, st.of_timer_s, frequency, false, dt,
                      advance_dt);
  st.v_meas_pu = voltage.value(advance_dt);
  st.f_meas_hz = frequency.value(advance_dt);
  if (st.have_prev_angle && dt > 0.0) {
    st.prev_angle_rad += angle_delta * (advance_dt / dt);
  } else {
    st.prev_angle_rad = angle_rad;
  }
  st.have_prev_angle = true;

  if (!has_action) {
    if (st.tripped && s.allow_reconnect) {
      TimeWindow healthy = inclusive_window(
          voltage, s.v_continuous_min_pu, dt, true);
      healthy = intersect_windows(
          healthy, inclusive_window(voltage, s.v_continuous_max_pu, dt, false));
      healthy = intersect_windows(
          healthy, inclusive_window(frequency, s.f_continuous_min_hz, dt, true));
      healthy = intersect_windows(
          healthy, inclusive_window(frequency, s.f_continuous_max_hz, dt, false));
      if (healthy.valid && healthy.end_s + kFilterEpsilon >= dt) {
        const double carried =
            healthy.begin_s <= kFilterEpsilon ? st.continuous_ok_timer_s : 0.0;
        st.continuous_ok_timer_s = carried + dt - healthy.begin_s;
      } else {
        st.continuous_ok_timer_s = 0.0;
      }
    }
    if (!st.tripped && st.restore_scale < 1.0 && s.power_ramp_s > 0.0) {
      st.restore_scale =
          std::min(1.0, st.restore_scale + advance_dt / s.power_ramp_s);
    }
    return result;
  }

  result.action_offset_s = advance_dt;
  if (!st.tripped) {
    st.tripped = true;
    st.restore_scale = 0.0;
    st.continuous_ok_timer_s = 0.0;
    st.last_reason = reason;
    reset_timers(st);
    result.action = IEEE1547Action::Tripped;
  } else {
    st.tripped = false;
    st.continuous_ok_timer_s = 0.0;
    st.restore_scale = s.power_ramp_s > 0.0 ? 0.0 : 1.0;
    reset_timers(st);
    result.action = IEEE1547Action::Reconnected;
  }
  return result;
}

IEEE1547Action step_ieee1547(const IEEE1547Settings& s,
                             IEEE1547RuntimeState& st,
                             double v_mag_pu,
                             double angle_rad,
                             double dt) {
  return advance_ieee1547_interval(s, st, v_mag_pu, angle_rad, dt).action;
}

double volt_var_q_pu(const VoltVarSettings& s, double v_pu) {
  if (!s.enabled) return 0.0;
  if (v_pu <= s.v1_pu) return s.q1_pu;
  if (v_pu < s.v2_pu) {
    const double span = std::max(1e-6, s.v2_pu - s.v1_pu);
    return s.q1_pu * (s.v2_pu - v_pu) / span;  // V1..V2: q1 -> 0
  }
  if (v_pu <= s.v3_pu) return 0.0;  // deadband
  if (v_pu < s.v4_pu) {
    const double span = std::max(1e-6, s.v4_pu - s.v3_pu);
    return s.q4_pu * (v_pu - s.v3_pu) / span;  // V3..V4: 0 -> q4
  }
  return s.q4_pu;
}

double freq_watt_delta_pu(const FreqWattSettings& s, double f_hz) {
  if (!s.enabled) return 0.0;
  const double fn = s.nominal_frequency_hz > 0.0 ? s.nominal_frequency_hz : 60.0;
  if (f_hz > fn + s.db_over_hz) {
    const double dev = (f_hz - fn - s.db_over_hz) / fn;
    return -dev / std::max(1e-3, s.droop_over);  // over-frequency: curtail
  }
  if (f_hz < fn - s.db_under_hz) {
    const double dev = (fn - s.db_under_hz - f_hz) / fn;
    return dev / std::max(1e-3, s.droop_under);  // under-frequency: raise
  }
  return 0.0;
}

namespace {

// Moves `state` toward `target` by one step: a first-order low-pass with time
// constant `tau_s` followed by a slew-rate limit `ramp` (pu/s; 0 = unlimited).
void low_pass_and_slew(double& state, double target, double tau_s, double ramp,
                       double dt) {
  double next = target;
  if (tau_s > 1e-9 && dt > 0.0) {
    const double a = std::clamp(dt / tau_s, 0.0, 1.0);
    next = state + a * (target - state);
  }
  double delta = next - state;
  if (ramp > 0.0 && dt > 0.0) {
    const double cap = ramp * dt;
    delta = std::clamp(delta, -cap, cap);
  }
  state += delta;
}

}  // namespace

void step_smart_inverter(const VoltVarSettings& vv, const FreqWattSettings& fw,
                         SmartInverterState& st, double v_mag_pu,
                         double angle_rad, double dt) {
  if (!vv.enabled && !fw.enabled) return;

  // Measured frequency from the terminal voltage-angle derivative (§7 role 4),
  // lightly filtered.
  double f_inst = fw.nominal_frequency_hz > 0.0 ? fw.nominal_frequency_hz : 60.0;
  if (st.have_prev_angle && dt > 0.0) {
    double dtheta = angle_rad - st.prev_angle_rad;
    while (dtheta > kPi) dtheta -= kTwoPi;
    while (dtheta <= -kPi) dtheta += kTwoPi;
    f_inst += (dtheta / dt) / kTwoPi;  // f = f_nominal + dtheta/dt / 2pi
  }
  st.prev_angle_rad = angle_rad;
  st.have_prev_angle = true;

  const double q_target = volt_var_q_pu(vv, v_mag_pu);
  if (!st.initialized) {
    st.f_meas_hz = fw.nominal_frequency_hz > 0.0 ? fw.nominal_frequency_hz : 60.0;
    st.q_pu = q_target;                                   // seed at steady state
    st.p_delta_pu = freq_watt_delta_pu(fw, st.f_meas_hz);  // (no startup transient)
    st.initialized = true;
    return;
  }
  if (dt > 0.0) {
    const double af = std::clamp(dt / 0.05, 0.0, 1.0);  // 50 ms freq measurement
    st.f_meas_hz += af * (f_inst - st.f_meas_hz);
  }

  low_pass_and_slew(st.q_pu, q_target, vv.filter_t_s, vv.ramp_rate_pu_per_s, dt);
  const double p_target = freq_watt_delta_pu(fw, st.f_meas_hz);
  low_pass_and_slew(st.p_delta_pu, p_target, fw.filter_t_s, fw.ramp_rate_pu_per_s,
                    dt);
}

}  // namespace hacdcpf::dynamics
