#include "hacdcpf/dynamics/devices/IEEE1547Protection.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::dynamics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Accumulates the violation timer for one band table on the measured value
// `meas`; `below` selects the under- (true) or over- (false) comparison. Sets
// `tripped` and `reason` when any entry stays in violation past its clearing
// time. A brief recovery resets that entry's timer (continuous-violation
// semantics).
void scan_band(const std::vector<RideThroughTrip>& table,
               std::vector<double>& timers,
               double meas,
               bool below,
               double dt,
               const char* tag,
               bool& tripped,
               std::string& reason) {
  if (timers.size() != table.size()) timers.assign(table.size(), 0.0);
  for (std::size_t i = 0; i < table.size(); ++i) {
    const bool violating =
        below ? (meas < table[i].threshold) : (meas > table[i].threshold);
    if (violating) {
      timers[i] += dt;
      if (timers[i] >= table[i].clearing_s) {
        tripped = true;
        if (reason.empty()) reason = tag;
      }
    } else {
      timers[i] = 0.0;
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

IEEE1547Action step_ieee1547(const IEEE1547Settings& s,
                             IEEE1547RuntimeState& st,
                             double v_mag_pu,
                             double angle_rad,
                             double dt) {
  if (!s.enabled) return IEEE1547Action::None;

  // Measured frequency from the terminal voltage-angle derivative (§7 role 4).
  // The first sample seeds the frame at nominal (no derivative available yet).
  double f_inst = s.nominal_frequency_hz;
  if (st.have_prev_angle && dt > 0.0) {
    double dtheta = angle_rad - st.prev_angle_rad;
    while (dtheta > kPi) dtheta -= kTwoPi;
    while (dtheta <= -kPi) dtheta += kTwoPi;
    f_inst = s.nominal_frequency_hz + (dtheta / dt) / kTwoPi;
  }
  st.prev_angle_rad = angle_rad;
  st.have_prev_angle = true;

  // Measurement low-pass filters.
  if (!st.initialized) {
    st.v_meas_pu = v_mag_pu;
    st.f_meas_hz = s.nominal_frequency_hz;
    st.initialized = true;
  } else if (dt > 0.0) {
    const double av = std::clamp(dt / std::max(1e-9, s.v_filter_t_s), 0.0, 1.0);
    const double af = std::clamp(dt / std::max(1e-9, s.f_filter_t_s), 0.0, 1.0);
    st.v_meas_pu += av * (v_mag_pu - st.v_meas_pu);
    st.f_meas_hz += af * (f_inst - st.f_meas_hz);
  }

  const double v = st.v_meas_pu;
  const double f = st.f_meas_hz;

  if (!st.tripped) {
    bool trip = false;
    std::string reason;
    scan_band(s.undervoltage_trip, st.uv_timer_s, v, true, dt, "undervoltage",
              trip, reason);
    scan_band(s.overvoltage_trip, st.ov_timer_s, v, false, dt, "overvoltage",
              trip, reason);
    scan_band(s.underfrequency_trip, st.uf_timer_s, f, true, dt,
              "underfrequency", trip, reason);
    scan_band(s.overfrequency_trip, st.of_timer_s, f, false, dt,
              "overfrequency", trip, reason);
    if (trip) {
      st.tripped = true;
      st.restore_scale = 0.0;
      st.continuous_ok_timer_s = 0.0;
      st.last_reason = reason;
      reset_timers(st);
      return IEEE1547Action::Tripped;
    }
    // Operating normally: advance the soft-start ramp if a reconnect is in
    // progress.
    if (st.restore_scale < 1.0 && s.power_ramp_s > 0.0 && dt > 0.0) {
      st.restore_scale = std::min(1.0, st.restore_scale + dt / s.power_ramp_s);
    }
    return IEEE1547Action::None;
  }

  // Tripped: qualify a reconnect once the terminal has held the continuous
  // window for the reconnect delay.
  if (!s.allow_reconnect) return IEEE1547Action::None;
  const bool healthy = v >= s.v_continuous_min_pu && v <= s.v_continuous_max_pu &&
                       f >= s.f_continuous_min_hz && f <= s.f_continuous_max_hz;
  if (healthy) {
    st.continuous_ok_timer_s += dt;
    if (st.continuous_ok_timer_s >= s.reconnect_delay_s) {
      st.tripped = false;
      st.continuous_ok_timer_s = 0.0;
      st.restore_scale = s.power_ramp_s > 0.0 ? 0.0 : 1.0;
      reset_timers(st);
      return IEEE1547Action::Reconnected;
    }
  } else {
    st.continuous_ok_timer_s = 0.0;
  }
  return IEEE1547Action::None;
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
