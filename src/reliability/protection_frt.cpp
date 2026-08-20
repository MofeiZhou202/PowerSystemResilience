#include "hacdcpf/reliability/protection_frt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_set>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"

namespace hacdcpf::analysis {
namespace {

void validate_probability(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0 || value > 1.0)
    throw std::invalid_argument(std::string(name) + " must be in [0,1]");
}

void validate_time_series(const std::vector<RelayCurrentPoint>& points) {
  for (size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].time_s) || points[i].time_s < 0.0 ||
        !std::isfinite(points[i].current))
      throw std::invalid_argument("relay trajectory contains an invalid point");
    if (i > 0 && !(points[i].time_s > points[i - 1].time_s))
      throw std::invalid_argument("relay trajectory times must be strictly increasing");
  }
}

double chain_delay_s(const BreakerClearingChain& chain) {
  const double values[] = {chain.channel_delay_s, chain.trip_coil_delay_s,
                           chain.mechanical_delay_s, chain.arc_delay_s};
  for (double value : values)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("breaker clearing-chain delays must be non-negative");
  return std::accumulate(std::begin(values), std::end(values), 0.0);
}

void validate_measurement_series(
    const std::vector<ProtectionMeasurementPoint>& points) {
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];
    const double values[] = {point.time_s, point.current,
                             point.directional_current,
                             point.apparent_impedance_ohm,
                             point.differential_current,
                             point.restraint_current};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument(
            "protection trajectory contains a non-finite value");
    if (point.time_s < 0.0 || point.apparent_impedance_ohm < 0.0 ||
        point.restraint_current < 0.0)
      throw std::invalid_argument(
          "protection trajectory contains a negative magnitude or time");
    if (i > 0 && !(point.time_s > points[i - 1].time_s))
      throw std::invalid_argument(
          "protection trajectory times must be strictly increasing");
    if (point.phasor_measurement_valid &&
        (!std::isfinite(point.voltage_phasor_v.real()) ||
         !std::isfinite(point.voltage_phasor_v.imag()) ||
         !std::isfinite(point.current_phasor_a.real()) ||
         !std::isfinite(point.current_phasor_a.imag())))
      throw std::invalid_argument(
          "protection trajectory contains a non-finite phasor");
  }
}

void validate_relay_model(const ProtectionRelayModel& relay) {
  if (!std::isfinite(relay.pickup_current) || relay.pickup_current <= 0.0 ||
      !std::isfinite(relay.definite_time_delay_s) ||
      relay.definite_time_delay_s < 0.0 ||
      !std::isfinite(relay.differential_pickup) ||
      relay.differential_pickup < 0.0 ||
      !std::isfinite(relay.differential_slope) ||
      relay.differential_slope < 0.0 ||
      !std::isfinite(relay.differential_high_set) ||
      relay.differential_high_set < 0.0 ||
      !std::isfinite(relay.reset_time_s) || relay.reset_time_s <= 0.0)
    throw std::invalid_argument("invalid protection relay settings");
  for (const auto& zone : relay.distance_zones) {
    if (!std::isfinite(zone.reach_ohm) || zone.reach_ohm <= 0.0 ||
        !std::isfinite(zone.delay_s) || zone.delay_s < 0.0 ||
        !std::isfinite(zone.line_angle_rad) ||
        !std::isfinite(zone.forward_resistance_ohm) ||
        !std::isfinite(zone.reverse_resistance_ohm) ||
        !std::isfinite(zone.forward_reactance_ohm) ||
        !std::isfinite(zone.reverse_reactance_ohm) ||
        zone.forward_resistance_ohm < 0.0 ||
        zone.reverse_resistance_ohm < 0.0 ||
        zone.forward_reactance_ohm < 0.0 ||
        zone.reverse_reactance_ohm < 0.0)
      throw std::invalid_argument("invalid distance-protection zone");
    if (zone.shape == DistanceZoneShape::Quadrilateral &&
        (zone.forward_resistance_ohm <= 0.0 ||
         zone.forward_reactance_ohm <= 0.0))
      throw std::invalid_argument(
          "quadrilateral distance zone requires positive forward reaches");
  }
  if (relay.characteristic == ProtectionRelayCharacteristic::Distance &&
      relay.distance_zones.empty())
    throw std::invalid_argument("distance relay requires at least one zone");
}

bool directional_permitted(const ProtectionRelayModel& relay,
                           const ProtectionMeasurementPoint& point) {
  return !relay.directional || point.directional_current > 0.0;
}

std::complex<double> limit_phasor_magnitude(std::complex<double> value,
                                            double limit) {
  if (limit <= 0.0 || std::abs(value) <= limit) return value;
  return value * (limit / std::abs(value));
}

bool distance_zone_pickup(const DistanceProtectionZone& zone,
                          const ProtectionMeasurementPoint& point) {
  if (zone.shape == DistanceZoneShape::MagnitudeCircle)
    return point.phasor_measurement_valid &&
               std::abs(point.current_phasor_a) > 1e-15
        ? std::abs(point.voltage_phasor_v / point.current_phasor_a) <=
              zone.reach_ohm
        : point.apparent_impedance_ohm <= zone.reach_ohm;
  if (!point.phasor_measurement_valid ||
      std::abs(point.current_phasor_a) <= 1e-15)
    return false;
  const std::complex<double> impedance =
      point.voltage_phasor_v / point.current_phasor_a;
  const std::complex<double> rotation =
      std::polar(1.0, -zone.line_angle_rad);
  const std::complex<double> rotated = impedance * rotation;
  if (zone.shape == DistanceZoneShape::Mho) {
    // IEEE C37.113-2015, mho characteristic in the impedance plane:
    // |Z - Zr/2| <= |Zr|/2, with Zr aligned to the line angle.
    return std::abs(rotated - std::complex<double>(zone.reach_ohm / 2.0, 0.0))
        <= zone.reach_ohm / 2.0 + 1e-12;
  }
  // IEEE C37.113 quadrilateral characteristic after rotation into line axes.
  return rotated.real() >= -zone.reverse_resistance_ohm - 1e-12 &&
      rotated.real() <= zone.forward_resistance_ohm + 1e-12 &&
      rotated.imag() >= -zone.reverse_reactance_ohm - 1e-12 &&
      rotated.imag() <= zone.forward_reactance_ohm + 1e-12;
}

bool contains_string(const std::vector<std::string>& values,
                     const std::string& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

bool binding_available(int binding, const ProtectionCyberStateClass& state,
                       size_t function_count, const char* name) {
  if (binding < -1 ||
      (binding >= 0 && static_cast<size_t>(binding) >= function_count))
    throw std::invalid_argument(std::string(name) +
                                " function binding is out of range");
  return binding < 0 || state.function_available[static_cast<size_t>(binding)];
}

struct ReliabilityMetrics {
  double eens{0.0};
  double lole{0.0};
  double lolf{0.0};
};

void add_stage(double duration_hr, double shed_mw, double threshold_mw,
               double weighted_frequency, ReliabilityMetrics& metrics,
               bool& interruption) {
  if (duration_hr <= 0.0) return;
  metrics.eens += weighted_frequency * duration_hr * shed_mw;
  if (shed_mw > threshold_mw) {
    metrics.lole += weighted_frequency * duration_hr;
    interruption = true;
  }
}

double der_curtailment_mw(const ProtectionCyberGeneratedClass& generated,
                          const ProtectionCyberEventInput& event) {
  if (generated.protection.der_results.size() != event.ders.size())
    throw std::runtime_error("DER FRT result dimension mismatch");
  double shed = 0.0;
  for (size_t i = 0; i < event.ders.size(); ++i) {
    const double capacity = event.ders[i].loss_of_generation_shed_mw;
    if (!std::isfinite(capacity) || capacity < 0.0)
      throw std::invalid_argument(
          "DER loss-of-generation shed contribution must be non-negative");
    shed += capacity *
        std::clamp(1.0 - generated.protection.der_results[i].terminal_restore_scale,
                   0.0, 1.0);
  }
  return shed;
}

ReliabilityMetrics aggregate_generated_classes(
    const ProtectionCyberClassGenerationResult& generated,
    const ProtectionCyberEventInput& event,
    const ProtectionCyberConsequenceModel& consequence,
    double frequency, double threshold_mw) {
  const double values[] = {
      consequence.primary_clearing_shed_mw,
      consequence.backup_clearing_shed_mw,
      consequence.uncleared_shed_mw,
      consequence.isolated_shed_mw,
      consequence.isolation_failed_shed_mw,
      consequence.restored_shed_mw,
      consequence.restoration_failed_shed_mw,
      consequence.automatic_restoration_hr,
      consequence.manual_restoration_hr,
      consequence.repair_hr};
  for (double value : values)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(
          "protection/cyber consequence values must be non-negative");

  ReliabilityMetrics metrics;
  for (const auto& item : generated.classes) {
    const double weighted_frequency =
        frequency * item.protection.conditional_probability;
    const double clearing_hr = item.protection.terminal_time_s / 3600.0;
    const double switching_hr = item.restoration_success
        ? consequence.automatic_restoration_hr
        : consequence.manual_restoration_hr;
    const double residual_hr =
        std::max(0.0, consequence.repair_hr - clearing_hr - switching_hr);
    double clearing_shed = consequence.uncleared_shed_mw;
    if (item.protection.protection_outcome ==
        ProtectionClearingOutcome::PrimaryCleared)
      clearing_shed = consequence.primary_clearing_shed_mw;
    else if (item.protection.protection_outcome ==
             ProtectionClearingOutcome::BackupCleared)
      clearing_shed = consequence.backup_clearing_shed_mw;
    const double der_shed = der_curtailment_mw(item, event);
    const double switching_shed =
        (item.isolation_success ? consequence.isolated_shed_mw
                                : consequence.isolation_failed_shed_mw) +
        der_shed;
    const double residual_shed =
        (item.restoration_success ? consequence.restored_shed_mw
                                  : consequence.restoration_failed_shed_mw) +
        der_shed;
    bool interruption = false;
    add_stage(clearing_hr, clearing_shed, threshold_mw, weighted_frequency,
              metrics, interruption);
    add_stage(switching_hr, switching_shed, threshold_mw, weighted_frequency,
              metrics, interruption);
    add_stage(residual_hr, residual_shed, threshold_mw, weighted_frequency,
              metrics, interruption);
    if (interruption) metrics.lolf += weighted_frequency;
  }
  return metrics;
}

void append_class(ProtectionFRTClassGenerationResult& result,
                  const ProtectionFRTEventInput& input,
                  ProtectionClearingOutcome outcome,
                  ProtectionFailureCause cause,
                  double probability,
                  double terminal_time_s,
                  bool coordination_satisfied) {
  if (probability <= 0.0) return;
  ProtectionFRTGeneratedClass generated;
  generated.protection_outcome = outcome;
  generated.failure_cause = cause;
  generated.conditional_probability = probability;
  generated.terminal_time_s = terminal_time_s;
  generated.coordination_satisfied = coordination_satisfied;
  generated.der_stable_ids.reserve(input.ders.size());
  generated.der_results.reserve(input.ders.size());
  for (const auto& der : input.ders) {
    generated.der_stable_ids.push_back(der.stable_id);
    generated.der_results.push_back(classify_der_frt_trajectory(
        der.settings, der.trajectory, terminal_time_s));
  }
  result.classes.push_back(std::move(generated));
}

}  // namespace

InverseTimeRelayResult evaluate_inverse_time_relay(
    const std::vector<RelayCurrentPoint>& trajectory,
    const InverseTimeRelaySettings& settings) {
  validate_time_series(trajectory);
  if (!std::isfinite(settings.pickup_current) || settings.pickup_current <= 0.0 ||
      !std::isfinite(settings.time_multiplier) || settings.time_multiplier <= 0.0 ||
      !std::isfinite(settings.curve_a) || settings.curve_a <= 0.0 ||
      !std::isfinite(settings.curve_p) || settings.curve_p <= 0.0 ||
      !std::isfinite(settings.curve_b) || settings.curve_b < 0.0 ||
      !std::isfinite(settings.additional_delay_s) ||
      settings.additional_delay_s < 0.0 ||
      !std::isfinite(settings.reset_time_s) || settings.reset_time_s <= 0.0)
    throw std::invalid_argument("invalid inverse-time relay settings");

  InverseTimeRelayResult result;
  if (trajectory.size() < 2) return result;
  double action = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const double t0 = trajectory[i - 1].time_s;
    const double dt = trajectory[i].time_s - t0;
    const double multiple =
        std::abs(trajectory[i - 1].current) / settings.pickup_current;
    result.peak_multiple_of_pickup =
        std::max(result.peak_multiple_of_pickup, multiple);
    if (multiple > 1.0) {
      // IEC 60255-151 inverse-time family, integrated as the action quantity
      // dOmega/dt=1/T(M) so changing fault current is represented explicitly.
      const double denominator = std::pow(multiple, settings.curve_p) - 1.0;
      const double operation_time = settings.time_multiplier *
          (settings.curve_a / denominator + settings.curve_b) +
          settings.additional_delay_s;
      const double increment = dt / operation_time;
      if (action + increment >= 1.0) {
        result.operated = true;
        result.command_time_s = t0 + (1.0 - action) * operation_time;
        result.terminal_action_integral = 1.0;
        return result;
      }
      action += increment;
    } else {
      // Exact solution of dOmega/dt=-Omega/T_reset over the interval.
      action *= std::exp(-dt / settings.reset_time_s);
    }
  }
  result.terminal_action_integral = action;
  return result;
}

double clearing_time_s(const InverseTimeRelayResult& relay,
                       const BreakerClearingChain& chain) {
  if (!relay.operated)
    throw std::invalid_argument("clearing time is undefined for a relay that did not operate");
  return relay.command_time_s + chain_delay_s(chain);
}

std::vector<ProtectionMeasurementPoint> simulate_instrument_transformers(
    const std::vector<PrimaryProtectionPhasorPoint>& trajectory,
    const InstrumentTransformerSettings& settings) {
  const double values[] = {
      settings.ct_ratio, settings.pt_ratio, settings.ct_time_constant_s,
      settings.pt_time_constant_s, settings.ct_saturation_secondary_a,
      settings.pt_saturation_secondary_v};
  for (double value : values)
    if (!std::isfinite(value))
      throw std::invalid_argument(
          "instrument-transformer settings must be finite");
  if (settings.ct_ratio <= 0.0 || settings.pt_ratio <= 0.0 ||
      settings.ct_time_constant_s < 0.0 ||
      settings.pt_time_constant_s < 0.0 ||
      settings.ct_saturation_secondary_a < 0.0 ||
      settings.pt_saturation_secondary_v < 0.0)
    throw std::invalid_argument("invalid instrument-transformer settings");
  if (!std::isfinite(settings.initial_ct_secondary_a.real()) ||
      !std::isfinite(settings.initial_ct_secondary_a.imag()) ||
      !std::isfinite(settings.initial_pt_secondary_v.real()) ||
      !std::isfinite(settings.initial_pt_secondary_v.imag()))
    throw std::invalid_argument(
        "instrument-transformer initial phasors must be finite");

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i];
    if (!std::isfinite(point.time_s) || point.time_s < 0.0 ||
        !std::isfinite(point.voltage_phasor_v.real()) ||
        !std::isfinite(point.voltage_phasor_v.imag()) ||
        !std::isfinite(point.current_phasor_a.real()) ||
        !std::isfinite(point.current_phasor_a.imag()) ||
        (i > 0 && !(point.time_s > trajectory[i - 1].time_s)))
      throw std::invalid_argument(
          "instrument-transformer input trajectory is invalid");
  }
  if (trajectory.empty()) return {};

  std::complex<double> current = settings.initial_ct_secondary_a;
  std::complex<double> voltage = settings.initial_pt_secondary_v;
  std::vector<ProtectionMeasurementPoint> result;
  result.reserve(trajectory.size());
  const auto append = [&](double time_s, std::complex<double> measured_voltage,
                          std::complex<double> measured_current) {
    ProtectionMeasurementPoint point;
    point.time_s = time_s;
    point.phasor_measurement_valid = true;
    point.voltage_phasor_v = limit_phasor_magnitude(
        measured_voltage, settings.pt_saturation_secondary_v);
    point.current_phasor_a = limit_phasor_magnitude(
        measured_current, settings.ct_saturation_secondary_a);
    point.current = std::abs(point.current_phasor_a);
    if (std::abs(point.current_phasor_a) > 1e-15)
      point.apparent_impedance_ohm =
          std::abs(point.voltage_phasor_v / point.current_phasor_a);
    return point;
  };
  result.push_back(append(trajectory.front().time_s, voltage, current));
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const double dt = trajectory[i].time_s - trajectory[i - 1].time_s;
    const std::complex<double> current_target =
        trajectory[i - 1].current_phasor_a / settings.ct_ratio;
    const std::complex<double> voltage_target =
        trajectory[i - 1].voltage_phasor_v / settings.pt_ratio;
    // IEC 61869 transient channel represented by the exact zero-order-hold
    // solution y(t+dt)=u+(y(t)-u)exp(-dt/tau).
    current = settings.ct_time_constant_s > 0.0
        ? current_target + (current - current_target) *
              std::exp(-dt / settings.ct_time_constant_s)
        : current_target;
    voltage = settings.pt_time_constant_s > 0.0
        ? voltage_target + (voltage - voltage_target) *
              std::exp(-dt / settings.pt_time_constant_s)
        : voltage_target;
    result.push_back(append(trajectory[i].time_s, voltage, current));
  }
  return result;
}

ProtectionRelayModel adapt_protection_settings(
    const ProtectionRelayModel& base,
    const AdaptiveProtectionContext& context) {
  validate_relay_model(base);
  const double values[] = {context.pickup_scale,
                           context.distance_reach_scale,
                           context.time_delay_scale};
  for (double value : values)
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument(
          "adaptive protection scales must be finite and positive");
  if (!context.communication_available) return base;

  ProtectionRelayModel adapted = base;
  adapted.pickup_current *= context.pickup_scale;
  adapted.inverse_time.pickup_current = adapted.pickup_current;
  adapted.definite_time_delay_s *= context.time_delay_scale;
  adapted.inverse_time.time_multiplier *= context.time_delay_scale;
  adapted.inverse_time.additional_delay_s *= context.time_delay_scale;
  adapted.differential_pickup *= context.pickup_scale;
  adapted.differential_high_set *= context.pickup_scale;
  for (auto& zone : adapted.distance_zones) {
    zone.reach_ohm *= context.distance_reach_scale;
    zone.forward_resistance_ohm *= context.distance_reach_scale;
    zone.reverse_resistance_ohm *= context.distance_reach_scale;
    zone.forward_reactance_ohm *= context.distance_reach_scale;
    zone.reverse_reactance_ohm *= context.distance_reach_scale;
    zone.delay_s *= context.time_delay_scale;
  }
  validate_relay_model(adapted);
  return adapted;
}

RecloserFuseSectionalizerResult simulate_recloser_fuse_sectionalizer(
    const RecloserFuseSectionalizerInput& input) {
  if (input.maximum_shots <= 0 ||
      input.shot_trip_times_s.size() !=
          static_cast<size_t>(input.maximum_shots) ||
      input.reclose_intervals_s.size() !=
          static_cast<size_t>(std::max(0, input.maximum_shots - 1)) ||
      input.fault_clears_after_shot < -1 ||
      input.fault_clears_after_shot > input.maximum_shots ||
      input.sectionalizer_count_to_open < 0 ||
      !std::isfinite(input.fuse_total_clearing_time_s) ||
      input.fuse_total_clearing_time_s < 0.0)
    throw std::invalid_argument(
        "invalid recloser/fuse/sectionalizer sequence settings");
  for (double time_s : input.shot_trip_times_s)
    if (!std::isfinite(time_s) || time_s < 0.0)
      throw std::invalid_argument("recloser shot times must be non-negative");
  for (double time_s : input.reclose_intervals_s)
    if (!std::isfinite(time_s) || time_s < 0.0)
      throw std::invalid_argument("recloser dead times must be non-negative");

  RecloserFuseSectionalizerResult result;
  double time_s = 0.0;
  for (int shot = 1; shot <= input.maximum_shots; ++shot) {
    const double energized_duration =
        input.shot_trip_times_s[static_cast<size_t>(shot - 1)];
    if (input.fuse_total_clearing_time_s > 0.0) {
      const double remaining_fraction = 1.0 - result.fuse_melting_fraction;
      const double time_to_fuse =
          remaining_fraction * input.fuse_total_clearing_time_s;
      if (time_to_fuse <= energized_duration + 1e-12) {
        time_s += time_to_fuse;
        result.fuse_melting_fraction = 1.0;
        result.fuse_open = true;
        result.fault_cleared = true;
        result.events.push_back(
            {time_s, ProtectionSequenceAction::FuseOpen, shot});
        result.terminal_time_s = time_s;
        return result;
      }
      result.fuse_melting_fraction +=
          energized_duration / input.fuse_total_clearing_time_s;
    }

    time_s += energized_duration;
    result.shots_completed = shot;
    result.recloser_closed = false;
    result.events.push_back(
        {time_s, ProtectionSequenceAction::TripOpen, shot});
    if (input.instantaneous_lockout) {
      result.recloser_locked_out = true;
      result.events.push_back(
          {time_s, ProtectionSequenceAction::Lockout, shot});
      result.terminal_time_s = time_s;
      return result;
    }

    if (input.sectionalizer_count_to_open > 0 &&
        shot >= input.sectionalizer_count_to_open) {
      result.sectionalizer_open = true;
      result.fault_cleared = true;
      result.events.push_back(
          {time_s, ProtectionSequenceAction::SectionalizerOpen, shot});
    }
    if (input.fault_clears_after_shot == shot)
      result.fault_cleared = true;

    if (shot == input.maximum_shots && !result.fault_cleared) {
      result.recloser_locked_out = true;
      result.events.push_back(
          {time_s, ProtectionSequenceAction::Lockout, shot});
      result.terminal_time_s = time_s;
      return result;
    }

    if (shot < input.maximum_shots) {
      time_s += input.reclose_intervals_s[static_cast<size_t>(shot - 1)];
      result.recloser_closed = true;
      result.events.push_back(
          {time_s, ProtectionSequenceAction::Reclose, shot});
      if (result.fault_cleared) {
        result.terminal_time_s = time_s;
        return result;
      }
    }
  }
  result.terminal_time_s = time_s;
  return result;
}

AutomaticProtectionTopologyResult evaluate_automatic_protection_topology(
    const HybridPowerSystem& system,
    const AutomaticProtectionTopologyInput& input) {
  HybridPowerSystem post_action = system;
  const auto open_stable_id = [](auto& components,
                                 const std::vector<int>& stable_ids,
                                 auto&& open, const char* kind) {
    for (int stable_id : stable_ids) {
      const auto it = std::find_if(
          components.begin(), components.end(),
          [&](const auto& component) { return component.index == stable_id; });
      if (it == components.end())
        throw std::invalid_argument(
            std::string("unknown ") + kind + " stable index " +
            std::to_string(stable_id));
      open(*it);
    }
  };
  open_stable_id(post_action.ac.switches, input.opened_switch_indices,
                 [](auto& item) { item.closed = false; }, "switch");
  open_stable_id(post_action.ac.branches, input.opened_ac_branch_indices,
                 [](auto& item) { item.in_service = false; }, "AC branch");
  open_stable_id(post_action.dc.branches, input.opened_dc_branch_indices,
                 [](auto& item) { item.in_service = false; }, "DC branch");

  const auto power_graph = graph::build_power_system_graph(post_action);
  const auto topology = graph::analyze_topology(power_graph);
  std::unordered_set<int> deenergized_ac;
  std::unordered_set<int> deenergized_dc;
  AutomaticProtectionTopologyResult result;
  for (const auto& island : topology.islands) {
    if (island.status == graph::IslandStatus::Valid ||
        island.status == graph::IslandStatus::Empty)
      continue;
    ++result.islands_without_source;
    deenergized_ac.insert(island.ac_bus_ids.begin(), island.ac_bus_ids.end());
    deenergized_dc.insert(island.dc_bus_ids.begin(), island.dc_bus_ids.end());
  }
  result.deenergized_ac_bus_ids.assign(deenergized_ac.begin(),
                                       deenergized_ac.end());
  result.deenergized_dc_bus_ids.assign(deenergized_dc.begin(),
                                       deenergized_dc.end());
  std::sort(result.deenergized_ac_bus_ids.begin(),
            result.deenergized_ac_bus_ids.end());
  std::sort(result.deenergized_dc_bus_ids.begin(),
            result.deenergized_dc_bus_ids.end());

  for (const auto& bus : post_action.ac.buses) {
    if (!bus.in_service || !deenergized_ac.contains(bus.index)) continue;
    result.shed_mw += std::max(0.0, bus.pd_mw);
    result.interrupted_customers += std::max(0, bus.n_customers);
  }
  for (const auto& load : post_action.ac.loads) {
    if (!load.in_service || !deenergized_ac.contains(load.bus)) continue;
    result.shed_mw += std::max(0.0, load.p_mw * load.scaling);
    result.interrupted_customers += std::max(0, load.n_customers);
  }
  for (const auto& bus : post_action.dc.buses) {
    if (!bus.in_service || !deenergized_dc.contains(bus.index)) continue;
    result.shed_mw += std::max(0.0, bus.pd_mw);
    result.interrupted_customers += std::max(0, bus.n_customers);
  }
  for (const auto& load : post_action.dc.loads) {
    if (!load.in_service || !deenergized_dc.contains(load.bus)) continue;
    result.shed_mw += std::max(0.0, load.p_mw * load.scaling);
    result.interrupted_customers += std::max(0, load.n_customers);
  }
  result.stable_ids_resolved = true;
  result.domain_qualified_topology_used = true;
  return result;
}

ProtectionMisoperationResult evaluate_protection_misoperation(
    const ProtectionMisoperationInput& input,
    double curtailment_threshold_mw) {
  const double nonnegative[] = {
      input.no_fault_decision_windows_per_year,
      input.disconnected_load_mw,
      input.restoration_duration_hr,
      curtailment_threshold_mw};
  for (double value : nonnegative)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(
          "misoperation exposure and consequence values must be non-negative");
  validate_probability(input.false_trip_probability_per_window,
                       "false trip probability");
  validate_probability(input.trip_channel_success_probability,
                       "trip channel success probability");
  validate_probability(input.breaker_success_probability,
                       "breaker success probability");

  ProtectionMisoperationResult result;
  result.false_trip_frequency_per_year =
      input.no_fault_decision_windows_per_year *
      input.false_trip_probability_per_window *
      input.trip_channel_success_probability *
      input.breaker_success_probability;
  result.eens_mwh_yr = result.false_trip_frequency_per_year *
      input.disconnected_load_mw * input.restoration_duration_hr;
  if (input.disconnected_load_mw > curtailment_threshold_mw) {
    result.lole_hr_yr = result.false_trip_frequency_per_year *
        input.restoration_duration_hr;
    result.lolf_occ_yr = result.false_trip_frequency_per_year;
  }
  return result;
}

ProtectionRelayEvaluation evaluate_protection_relay(
    const std::vector<ProtectionMeasurementPoint>& trajectory,
    const ProtectionRelayModel& relay) {
  validate_measurement_series(trajectory);
  validate_relay_model(relay);
  ProtectionRelayEvaluation result;
  result.relay_id = relay.relay_id;
  if (trajectory.empty()) return result;

  if (relay.characteristic ==
      ProtectionRelayCharacteristic::InverseTimeOvercurrent) {
    std::vector<RelayCurrentPoint> current;
    current.reserve(trajectory.size());
    for (const auto& point : trajectory)
      current.push_back({point.time_s,
                         directional_permitted(relay, point)
                             ? point.current : 0.0});
    auto settings = relay.inverse_time;
    settings.pickup_current = relay.pickup_current;
    const auto inverse = evaluate_inverse_time_relay(current, settings);
    result.operated = inverse.operated;
    result.command_time_s = inverse.command_time_s;
    result.terminal_action_integral = inverse.terminal_action_integral;
    result.criterion = "IEC 60255 inverse-time action integral";
    return result;
  }

  std::vector<double> action(
      relay.characteristic == ProtectionRelayCharacteristic::Distance
          ? relay.distance_zones.size() : 1,
      0.0);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i - 1];
    const double dt = trajectory[i].time_s - point.time_s;
    for (size_t element = 0; element < action.size(); ++element) {
      bool picked_up = directional_permitted(relay, point);
      double delay_s = relay.definite_time_delay_s;
      if (relay.characteristic ==
          ProtectionRelayCharacteristic::DefiniteTimeOvercurrent) {
        picked_up = picked_up && std::abs(point.current) >= relay.pickup_current;
        result.criterion = "definite-time overcurrent";
      } else if (relay.characteristic ==
                 ProtectionRelayCharacteristic::Distance) {
        const auto& zone = relay.distance_zones[element];
        picked_up = picked_up && distance_zone_pickup(zone, point);
        delay_s = zone.delay_s;
        result.criterion = zone.shape == DistanceZoneShape::Mho
            ? "complex-plane mho distance zone"
            : (zone.shape == DistanceZoneShape::Quadrilateral
                   ? "complex-plane quadrilateral distance zone"
                   : "distance magnitude reach and zone timer");
      } else {
        // Percentage-biased differential element: Iop >= Imin + k*Irest,
        // with an optional instantaneous high-set element.
        const double threshold = relay.differential_pickup +
            relay.differential_slope * point.restraint_current;
        picked_up = picked_up &&
            (std::abs(point.differential_current) >= threshold ||
             (relay.differential_high_set > 0.0 &&
              std::abs(point.differential_current) >=
                  relay.differential_high_set));
        result.criterion = "biased differential or high-set";
      }

      if (!picked_up) {
        action[element] *= std::exp(-dt / relay.reset_time_s);
        continue;
      }
      if (delay_s == 0.0) {
        result.operated = true;
        result.command_time_s = point.time_s;
      } else if (action[element] + dt / delay_s >= 1.0) {
        result.operated = true;
        result.command_time_s =
            point.time_s + (1.0 - action[element]) * delay_s;
      } else {
        action[element] += dt / delay_s;
      }
      if (result.operated) {
        result.terminal_action_integral = 1.0;
        if (relay.characteristic == ProtectionRelayCharacteristic::Distance) {
          result.operated_zone = static_cast<int>(element);
          result.operated_zone_name = relay.distance_zones[element].name;
        }
        return result;
      }
    }
  }
  result.terminal_action_integral =
      action.empty() ? 0.0 : *std::max_element(action.begin(), action.end());
  return result;
}

ProtectionCoordinationReport evaluate_protection_coordination(
    const std::vector<ProtectionRelayModel>& relays,
    const std::vector<std::vector<ProtectionMeasurementPoint>>& trajectories,
    const std::vector<BreakerClearingChain>& breaker_chains,
    const std::vector<ProtectionCoordinationPair>& pairs) {
  if (trajectories.size() != relays.size() ||
      breaker_chains.size() != relays.size())
    throw std::invalid_argument(
        "coordination relay, trajectory and breaker dimensions must match");
  ProtectionCoordinationReport report;
  report.all_selective = true;
  report.relay_evaluations.reserve(relays.size());
  report.clearing_times_s.assign(relays.size(), 0.0);
  for (size_t i = 0; i < relays.size(); ++i) {
    report.relay_evaluations.push_back(
        evaluate_protection_relay(trajectories[i], relays[i]));
    if (report.relay_evaluations.back().operated)
      report.clearing_times_s[i] =
          report.relay_evaluations.back().command_time_s +
          chain_delay_s(breaker_chains[i]);
  }
  for (const auto& pair : pairs) {
    if (pair.primary_relay >= relays.size() ||
        pair.backup_relay >= relays.size() ||
        !std::isfinite(pair.required_margin_s) ||
        pair.required_margin_s < 0.0)
      throw std::invalid_argument("invalid protection coordination pair");
    ProtectionCoordinationCheck check;
    check.primary_relay_id = relays[pair.primary_relay].relay_id;
    check.backup_relay_id = relays[pair.backup_relay].relay_id;
    check.primary_operated =
        report.relay_evaluations[pair.primary_relay].operated;
    check.backup_operated =
        report.relay_evaluations[pair.backup_relay].operated;
    check.primary_clear_time_s = report.clearing_times_s[pair.primary_relay];
    check.backup_clear_time_s = report.clearing_times_s[pair.backup_relay];
    check.required_margin_s = pair.required_margin_s;
    check.actual_margin_s = check.primary_operated && check.backup_operated
        ? check.backup_clear_time_s - check.primary_clear_time_s
        : -std::numeric_limits<double>::infinity();
    check.selective = check.primary_operated && check.backup_operated &&
        check.actual_margin_s + 1e-12 >= check.required_margin_s;
    check.message = check.selective
        ? "selective: backup clearing margin is sufficient"
        : "non-selective: missing operation or insufficient clearing margin";
    report.all_selective = report.all_selective && check.selective;
    report.checks.push_back(std::move(check));
  }
  return report;
}

DERFRTTrajectoryResult classify_der_frt_trajectory(
    const dynamics::IEEE1547Settings& settings,
    const std::vector<DERTrajectoryPoint>& trajectory,
    double terminal_time_s) {
  return classify_der_frt_trajectory(
      settings, DERMomentaryCessationSettings{}, trajectory, terminal_time_s);
}

DERFRTTrajectoryResult classify_der_frt_trajectory(
    const dynamics::IEEE1547Settings& settings,
    const DERMomentaryCessationSettings& momentary_cessation,
    const std::vector<DERTrajectoryPoint>& trajectory,
    double terminal_time_s) {
  if (!std::isfinite(terminal_time_s) || terminal_time_s < 0.0)
    throw std::invalid_argument("DER FRT terminal time must be non-negative");
  if (momentary_cessation.enabled &&
      (!std::isfinite(momentary_cessation.enter_below_voltage_pu) ||
       !std::isfinite(momentary_cessation.exit_above_voltage_pu) ||
       !std::isfinite(momentary_cessation.exit_dwell_s) ||
       !std::isfinite(momentary_cessation.maximum_duration_s) ||
       momentary_cessation.enter_below_voltage_pu < 0.0 ||
       momentary_cessation.exit_above_voltage_pu <
           momentary_cessation.enter_below_voltage_pu ||
       momentary_cessation.exit_dwell_s < 0.0 ||
       momentary_cessation.maximum_duration_s < 0.0))
    throw std::invalid_argument(
        "DER momentary-cessation thresholds and durations are invalid");
  DERFRTTrajectoryResult result;
  if (!settings.enabled || trajectory.empty()) return result;

  dynamics::IEEE1547RuntimeState state;
  bool momentary_active = false;
  bool momentary_escalated = false;
  double momentary_start_s = -1.0;
  double exit_dwell_s = 0.0;
  bool exit_qualifying = false;
  auto step_momentary = [&](double time_s, double voltage_pu,
                            double dt_s) {
    if (!momentary_cessation.enabled || momentary_escalated) return;
    if (!momentary_active &&
        voltage_pu <= momentary_cessation.enter_below_voltage_pu) {
      momentary_active = true;
      momentary_start_s = time_s;
      exit_dwell_s = 0.0;
      result.ever_momentary_ceased = true;
      if (result.first_momentary_cessation_time_s < 0.0)
        result.first_momentary_cessation_time_s = time_s;
    }
    if (!momentary_active) return;
    if (voltage_pu >= momentary_cessation.exit_above_voltage_pu) {
      if (exit_qualifying)
        exit_dwell_s += dt_s;
      else
        exit_qualifying = true;
      if (exit_dwell_s + 1e-12 >= momentary_cessation.exit_dwell_s) {
        momentary_active = false;
        if (result.first_momentary_recovery_time_s < 0.0)
          result.first_momentary_recovery_time_s = time_s;
      }
    } else {
      exit_dwell_s = 0.0;
      exit_qualifying = false;
    }
    if (momentary_active && momentary_cessation.maximum_duration_s > 0.0 &&
        time_s - momentary_start_s + 1e-12 >=
            momentary_cessation.maximum_duration_s) {
      momentary_escalated = true;
      momentary_active = false;
      result.ever_tripped = true;
      if (result.first_trip_time_s < 0.0) result.first_trip_time_s = time_s;
      result.trip_reason = "momentary cessation maximum duration exceeded";
    }
  };
  double previous_time = trajectory.front().time_s;
  if (!std::isfinite(previous_time) || previous_time < 0.0 ||
      previous_time > terminal_time_s)
    throw std::invalid_argument("DER FRT trajectory has an invalid initial time");
  dynamics::step_ieee1547(settings, state, trajectory.front().voltage_pu,
                          trajectory.front().angle_rad, 0.0);
  step_momentary(previous_time, trajectory.front().voltage_pu, 0.0);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i];
    if (!std::isfinite(point.time_s) || !std::isfinite(point.voltage_pu) ||
        !std::isfinite(point.angle_rad) || point.time_s <= previous_time)
      throw std::invalid_argument("DER FRT trajectory must be finite and strictly increasing");
    if (point.time_s > terminal_time_s) break;
    const auto action = dynamics::step_ieee1547(
        settings, state, point.voltage_pu, point.angle_rad,
        point.time_s - previous_time);
    step_momentary(point.time_s, point.voltage_pu,
                   point.time_s - previous_time);
    if (action == dynamics::IEEE1547Action::Tripped) {
      result.ever_tripped = true;
      if (result.first_trip_time_s < 0.0) result.first_trip_time_s = point.time_s;
    } else if (action == dynamics::IEEE1547Action::Reconnected) {
      result.ever_reconnected = true;
      if (result.first_reconnect_time_s < 0.0)
        result.first_reconnect_time_s = point.time_s;
    }
    previous_time = point.time_s;
  }
  result.terminal_connected = !state.tripped && !momentary_escalated;
  result.terminal_momentary_ceased = momentary_active;
  result.terminal_restore_scale =
      (momentary_active || momentary_escalated) ? 0.0 : state.restore_scale;
  if (!momentary_escalated) result.trip_reason = state.last_reason;
  if (state.tripped || momentary_escalated)
    result.terminal_class = DERFRTTerminalClass::Tripped;
  else if (momentary_active)
    result.terminal_class = DERFRTTerminalClass::MomentaryCessation;
  else if (result.ever_tripped)
    result.terminal_class = DERFRTTerminalClass::Reconnected;
  else
    result.terminal_class = DERFRTTerminalClass::RideThrough;
  return result;
}

MicrogridSynchronizationResult evaluate_microgrid_synchronization(
    const MicrogridSynchronizationInput& input) {
  const double values[] = {
      input.voltage_difference_pu, input.frequency_difference_hz,
      input.angle_difference_rad, input.maximum_voltage_difference_pu,
      input.maximum_frequency_difference_hz,
      input.maximum_angle_difference_rad};
  for (double value : values)
    if (!std::isfinite(value))
      throw std::invalid_argument(
          "microgrid synchronization values must be finite");
  if (input.maximum_voltage_difference_pu < 0.0 ||
      input.maximum_frequency_difference_hz < 0.0 ||
      input.maximum_angle_difference_rad < 0.0)
    throw std::invalid_argument(
        "microgrid synchronization windows must be non-negative");

  MicrogridSynchronizationResult result;
  const double wrapped_angle = std::abs(std::atan2(
      std::sin(input.angle_difference_rad),
      std::cos(input.angle_difference_rad)));
  result.voltage_within_window =
      std::abs(input.voltage_difference_pu) <=
      input.maximum_voltage_difference_pu + 1e-12;
  result.frequency_within_window =
      std::abs(input.frequency_difference_hz) <=
      input.maximum_frequency_difference_hz + 1e-12;
  result.angle_within_window = wrapped_angle <=
      input.maximum_angle_difference_rad + 1e-12;
  result.close_permitted = input.synchronization_measurement_available &&
      input.close_command_channel_available &&
      result.voltage_within_window && result.frequency_within_window &&
      result.angle_within_window;
  if (!input.synchronization_measurement_available)
    result.reason = "synchronization measurement unavailable";
  else if (!input.close_command_channel_available)
    result.reason = "close-command channel unavailable";
  else if (!result.voltage_within_window)
    result.reason = "voltage difference exceeds synchronization window";
  else if (!result.frequency_within_window)
    result.reason = "frequency difference exceeds synchronization window";
  else if (!result.angle_within_window)
    result.reason = "angle difference exceeds synchronization window";
  else
    result.reason = "synchronization window and command gates satisfied";
  return result;
}

ProtectionFRTClassGenerationResult generate_protection_frt_classes(
    const ProtectionFRTEventInput& input) {
  validate_probability(input.primary.relay_success_probability,
                       "primary relay success probability");
  validate_probability(input.primary.breaker_success_probability,
                       "primary breaker success probability");
  validate_probability(input.backup.relay_success_probability,
                       "backup relay success probability");
  validate_probability(input.backup.breaker_success_probability,
                       "backup breaker success probability");
  if (!std::isfinite(input.coordination_margin_s) ||
      input.coordination_margin_s < 0.0 ||
      !std::isfinite(input.uncleared_terminal_time_s) ||
      input.uncleared_terminal_time_s < 0.0)
    throw std::invalid_argument("invalid protection/FRT event timing");

  ProtectionFRTClassGenerationResult result;
  result.primary_relay = evaluate_inverse_time_relay(
      input.primary.current_trajectory, input.primary.relay);
  result.backup_relay = evaluate_inverse_time_relay(
      input.backup.current_trajectory, input.backup.relay);
  result.primary_clear_time_s = result.primary_relay.operated
      ? clearing_time_s(result.primary_relay, input.primary.breaker) : 0.0;
  result.backup_clear_time_s = result.backup_relay.operated
      ? clearing_time_s(result.backup_relay, input.backup.breaker) : 0.0;

  const double primary_relay_success = result.primary_relay.operated
      ? input.primary.relay_success_probability : 0.0;
  const double primary_clear_probability = primary_relay_success *
      input.primary.breaker_success_probability;
  const double primary_relay_failure = 1.0 - primary_relay_success;
  const double primary_breaker_failure = primary_relay_success *
      (1.0 - input.primary.breaker_success_probability);
  const double backup_success = result.backup_relay.operated
      ? input.backup.relay_success_probability *
            input.backup.breaker_success_probability
      : 0.0;
  const bool coordinated = result.primary_relay.operated &&
      result.backup_relay.operated &&
      result.backup_clear_time_s - result.primary_clear_time_s >=
          input.coordination_margin_s;

  append_class(result, input, ProtectionClearingOutcome::PrimaryCleared,
               ProtectionFailureCause::None, primary_clear_probability,
               result.primary_clear_time_s, coordinated);
  append_class(result, input, ProtectionClearingOutcome::BackupCleared,
               ProtectionFailureCause::PrimaryRelayFailed,
               primary_relay_failure * backup_success,
               result.backup_clear_time_s, coordinated);
  append_class(result, input, ProtectionClearingOutcome::BackupCleared,
               ProtectionFailureCause::PrimaryBreakerFailed,
               primary_breaker_failure * backup_success,
               result.backup_clear_time_s, coordinated);
  append_class(result, input, ProtectionClearingOutcome::Uncleared,
               ProtectionFailureCause::PrimaryRelayAndBackupFailed,
               primary_relay_failure * (1.0 - backup_success),
               input.uncleared_terminal_time_s, false);
  append_class(result, input, ProtectionClearingOutcome::Uncleared,
               ProtectionFailureCause::PrimaryBreakerAndBackupFailed,
               primary_breaker_failure * (1.0 - backup_success),
               input.uncleared_terminal_time_s, false);

  const double probability_sum = std::accumulate(
      result.classes.begin(), result.classes.end(), 0.0,
      [](double sum, const ProtectionFRTGeneratedClass& generated) {
        return sum + generated.conditional_probability;
      });
  result.class_probabilities_normalized =
      std::abs(probability_sum - 1.0) <= 1e-12;
  if (!result.class_probabilities_normalized)
    throw std::runtime_error("protection/FRT event tree failed probability normalization");
  return result;
}

ProtectionCyberStateResult enumerate_protection_cyber_states(
    const std::vector<ProtectionCyberComponent>& components,
    const std::vector<ProtectionCyberFunction>& functions,
    const std::vector<ProtectionCyberEnvironment>& environments) {
  constexpr size_t kMaxExactComponents = 20;
  if (components.size() > kMaxExactComponents)
    throw std::invalid_argument(
        "exact protection/cyber enumeration supports at most 20 components");

  std::set<std::string> component_ids;
  for (const auto& component : components) {
    if (component.id.empty() || !component_ids.insert(component.id).second)
      throw std::invalid_argument(
          "protection/cyber component ids must be non-empty and unique");
    validate_probability(component.intrinsic_availability,
                         "cyber component intrinsic availability");
    validate_probability(component.packet_delivery_probability,
                         "cyber component packet delivery probability");
    const double values[] = {component.latency_ms, component.jitter_ms,
                             component.backup_energy_wh,
                             component.power_draw_w};
    for (double value : values)
      if (!std::isfinite(value) || value < 0.0)
        throw std::invalid_argument(
            "cyber component QoS, energy and power values must be non-negative");
    if (!component.supplied_by_bus_id.empty() &&
        component.power_draw_w <= 0.0)
      throw std::invalid_argument(
          "a bus-supplied cyber component requires positive power_draw_w");
  }
  for (const auto& function : functions) {
    if (!std::isfinite(function.max_latency_ms) ||
        function.max_latency_ms < 0.0 ||
        !std::isfinite(function.max_jitter_ms) ||
        function.max_jitter_ms < 0.0)
      throw std::invalid_argument("cyber function QoS limits are invalid");
    validate_probability(function.min_packet_delivery_probability,
                         "cyber function minimum packet delivery probability");
    for (const auto& path : function.alternative_paths)
      for (size_t index : path.component_indices)
        if (index >= components.size())
          throw std::invalid_argument(
              "cyber function path component index is out of range");
  }

  std::vector<ProtectionCyberEnvironment> effective_environments = environments;
  if (effective_environments.empty())
    effective_environments.push_back({"nominal", 1.0, {}, {}, 0.0});
  double environment_probability_sum = 0.0;
  for (const auto& environment : effective_environments) {
    validate_probability(environment.probability,
                         "cyber environment probability");
    if (!std::isfinite(environment.information_outage_duration_hr) ||
        environment.information_outage_duration_hr < 0.0)
      throw std::invalid_argument(
          "cyber environment outage duration must be non-negative");
    environment_probability_sum += environment.probability;
  }
  if (std::abs(environment_probability_sum - 1.0) > 1e-12)
    throw std::invalid_argument(
        "protection/cyber environment probabilities must sum to one");

  ProtectionCyberStateResult result;
  result.function_availability.assign(functions.size(), 0.0);
  std::vector<size_t> component_path_uses(components.size(), 0);
  for (const auto& function : functions)
    for (const auto& path : function.alternative_paths)
      for (size_t index : path.component_indices)
        ++component_path_uses[index];
  result.shared_dependencies_modelled = std::any_of(
      component_path_uses.begin(), component_path_uses.end(),
      [](size_t uses) { return uses > 1; });

  const size_t state_count = size_t{1} << components.size();
  for (const auto& environment : effective_environments) {
    result.common_cause_conditioned =
        result.common_cause_conditioned ||
        !environment.failed_common_cause_groups.empty();
    std::vector<double> conditional_success(components.size(), 0.0);
    for (size_t i = 0; i < components.size(); ++i) {
      const auto& component = components[i];
      bool forced_down = !component.common_cause_group.empty() &&
          contains_string(environment.failed_common_cause_groups,
                          component.common_cause_group);
      if (!component.supplied_by_bus_id.empty() &&
          contains_string(environment.deenergized_bus_ids,
                          component.supplied_by_bus_id)) {
        result.power_dependency_modelled = true;
        if (component.power_draw_w > 0.0) {
          const double autonomy_hr =
              component.backup_energy_wh / component.power_draw_w;
          forced_down = forced_down ||
              autonomy_hr + 1e-12 < environment.information_outage_duration_hr;
        }
      }
      // Conditional independence is applied only inside one explicit outer
      // environment. Common-cause and physical-supply dependence are carried
      // by the shared environment variable, not multiplied as path marginals.
      conditional_success[i] = forced_down ? 0.0
          : component.intrinsic_availability *
                component.packet_delivery_probability;
    }

    for (size_t mask = 0; mask < state_count; ++mask) {
      ProtectionCyberStateClass state;
      state.environment_name = environment.name;
      state.probability = environment.probability;
      state.component_available.resize(components.size(), false);
      for (size_t i = 0; i < components.size(); ++i) {
        const bool available = (mask & (size_t{1} << i)) != 0;
        state.component_available[i] = available;
        state.probability *= available ? conditional_success[i]
                                       : 1.0 - conditional_success[i];
      }
      if (state.probability <= 0.0) continue;

      state.function_available.assign(functions.size(), false);
      for (size_t f = 0; f < functions.size(); ++f) {
        for (const auto& path : functions[f].alternative_paths) {
          bool path_available = true;
          double latency_ms = 0.0;
          double jitter_ms = 0.0;
          double packet_delivery = 1.0;
          for (size_t index : path.component_indices) {
            path_available = path_available && state.component_available[index];
            latency_ms += components[index].latency_ms;
            jitter_ms += components[index].jitter_ms;
            packet_delivery *= components[index].packet_delivery_probability;
          }
          const auto& function = functions[f];
          path_available = path_available &&
              (function.max_latency_ms == 0.0 ||
               latency_ms <= function.max_latency_ms + 1e-12) &&
              (function.max_jitter_ms == 0.0 ||
               jitter_ms <= function.max_jitter_ms + 1e-12) &&
              packet_delivery + 1e-12 >=
                  function.min_packet_delivery_probability;
          if (path_available) {
            state.function_available[f] = true;
            break;
          }
        }
        if (state.function_available[f])
          result.function_availability[f] += state.probability;
      }
      result.classes.push_back(std::move(state));
    }
  }

  const double probability_sum = std::accumulate(
      result.classes.begin(), result.classes.end(), 0.0,
      [](double sum, const ProtectionCyberStateClass& state) {
        return sum + state.probability;
      });
  result.probabilities_normalized =
      std::abs(probability_sum - 1.0) <= 1e-12;
  if (!result.probabilities_normalized)
    throw std::runtime_error(
        "protection/cyber state enumeration failed probability normalization");
  return result;
}

ProtectionCyberClassGenerationResult generate_protection_cyber_classes(
    const ProtectionCyberEventInput& input) {
  validate_probability(input.primary.relay_success_probability,
                       "primary relay success probability");
  validate_probability(input.primary.breaker_success_probability,
                       "primary breaker success probability");
  validate_probability(input.backup.relay_success_probability,
                       "backup relay success probability");
  validate_probability(input.backup.breaker_success_probability,
                       "backup breaker success probability");
  if (!std::isfinite(input.coordination_margin_s) ||
      input.coordination_margin_s < 0.0 ||
      !std::isfinite(input.uncleared_terminal_time_s) ||
      input.uncleared_terminal_time_s < 0.0)
    throw std::invalid_argument("invalid protection/cyber event timing");

  ProtectionCyberClassGenerationResult result;
  result.coordination = evaluate_protection_coordination(
      {input.primary.relay, input.backup.relay},
      {input.primary.trajectory, input.backup.trajectory},
      {input.primary.breaker, input.backup.breaker},
      {{0, 1, input.coordination_margin_s}});
  result.information = enumerate_protection_cyber_states(
      input.information_components, input.information_functions,
      input.information_environments);

  const auto append = [&](const ProtectionCyberStateClass& state,
                          bool detection, bool primary_channel,
                          bool backup_channel, bool isolation,
                          bool restoration, ProtectionClearingOutcome outcome,
                          ProtectionFailureCause cause, double probability,
                          double terminal_time_s) {
    if (probability <= 0.0) return;
    ProtectionCyberGeneratedClass generated;
    generated.information_environment = state.environment_name;
    generated.information_component_available = state.component_available;
    generated.detection_success = detection;
    generated.primary_trip_channel_available = primary_channel;
    generated.backup_trip_channel_available = backup_channel;
    generated.isolation_success = isolation;
    generated.restoration_success = restoration;
    generated.protection.protection_outcome = outcome;
    generated.protection.failure_cause = cause;
    generated.protection.conditional_probability = probability;
    generated.protection.terminal_time_s = terminal_time_s;
    generated.protection.coordination_satisfied =
        result.coordination.all_selective && detection && primary_channel &&
        backup_channel;
    const std::vector<ProtectionFRTDERInput>* outcome_ders = &input.ders;
    if (outcome == ProtectionClearingOutcome::BackupCleared &&
        !input.backup_ders.empty())
      outcome_ders = &input.backup_ders;
    else if (outcome == ProtectionClearingOutcome::Uncleared &&
             !input.uncleared_ders.empty())
      outcome_ders = &input.uncleared_ders;
    if (outcome_ders->size() != input.ders.size())
      throw std::invalid_argument(
          "outcome-specific DER trajectory dimensions must match primary DER inputs");
    for (size_t der_index = 0; der_index < outcome_ders->size(); ++der_index) {
      const auto& der = (*outcome_ders)[der_index];
      if (der.stable_id != input.ders[der_index].stable_id ||
          std::abs(der.loss_of_generation_shed_mw -
                   input.ders[der_index].loss_of_generation_shed_mw) > 1e-12)
        throw std::invalid_argument(
            "outcome-specific DER identities and shed capacities must match");
      generated.protection.der_stable_ids.push_back(der.stable_id);
      generated.protection.der_results.push_back(classify_der_frt_trajectory(
          der.settings, der.momentary_cessation, der.trajectory,
          terminal_time_s));
    }
    result.classes.push_back(std::move(generated));
  };

  const auto& primary_eval = result.coordination.relay_evaluations[0];
  const auto& backup_eval = result.coordination.relay_evaluations[1];
  const double primary_clear_time = result.coordination.clearing_times_s[0];
  const double backup_clear_time = result.coordination.clearing_times_s[1];
  for (const auto& state : result.information.classes) {
    const size_t function_count = input.information_functions.size();
    const bool detection = binding_available(
        input.function_bindings.detection_function, state, function_count,
        "detection");
    const bool primary_channel = binding_available(
        input.function_bindings.primary_trip_function, state, function_count,
        "primary trip");
    const bool backup_channel = binding_available(
        input.function_bindings.backup_trip_function, state, function_count,
        "backup trip");
    const bool isolation_function = binding_available(
        input.function_bindings.isolation_function, state, function_count,
        "isolation");
    const bool restoration_function = binding_available(
        input.function_bindings.restoration_function, state, function_count,
        "restoration");
    // Automatic isolation requires a detected fault; automatic restoration is
    // credited only after successful isolation. Manual fallback is represented
    // later by the consequence model's manual restoration window.
    const bool isolation = detection && isolation_function;
    const bool restoration = isolation && restoration_function;

    const double primary_relay_success =
        primary_eval.operated && detection && primary_channel
        ? input.primary.relay_success_probability : 0.0;
    const double primary_clear_probability = primary_relay_success *
        input.primary.breaker_success_probability;
    const double primary_relay_failure = 1.0 - primary_relay_success;
    const double primary_breaker_failure = primary_relay_success *
        (1.0 - input.primary.breaker_success_probability);
    const double backup_success =
        backup_eval.operated && detection && backup_channel
        ? input.backup.relay_success_probability *
              input.backup.breaker_success_probability
        : 0.0;
    ProtectionFailureCause primary_failure_cause =
        ProtectionFailureCause::PrimaryRelayFailed;
    if (!detection)
      primary_failure_cause = ProtectionFailureCause::DetectionUnavailable;
    else if (!primary_channel)
      primary_failure_cause =
          ProtectionFailureCause::PrimaryTripChannelUnavailable;
    ProtectionFailureCause uncleared_relay_cause =
        ProtectionFailureCause::PrimaryRelayAndBackupFailed;
    if (!detection)
      uncleared_relay_cause = ProtectionFailureCause::DetectionUnavailable;
    else if (!backup_channel)
      uncleared_relay_cause =
          ProtectionFailureCause::BackupTripChannelUnavailable;

    const double p = state.probability;
    append(state, detection, primary_channel, backup_channel, isolation,
           restoration, ProtectionClearingOutcome::PrimaryCleared,
           ProtectionFailureCause::None,
           p * primary_clear_probability, primary_clear_time);
    append(state, detection, primary_channel, backup_channel, isolation,
           restoration, ProtectionClearingOutcome::BackupCleared,
           primary_failure_cause,
           p * primary_relay_failure * backup_success, backup_clear_time);
    append(state, detection, primary_channel, backup_channel, isolation,
           restoration, ProtectionClearingOutcome::BackupCleared,
           ProtectionFailureCause::PrimaryBreakerFailed,
           p * primary_breaker_failure * backup_success, backup_clear_time);
    append(state, detection, primary_channel, backup_channel, isolation,
           restoration, ProtectionClearingOutcome::Uncleared,
           uncleared_relay_cause,
           p * primary_relay_failure * (1.0 - backup_success),
           input.uncleared_terminal_time_s);
    append(state, detection, primary_channel, backup_channel, isolation,
           restoration, ProtectionClearingOutcome::Uncleared,
           backup_channel
               ? ProtectionFailureCause::PrimaryBreakerAndBackupFailed
               : ProtectionFailureCause::BackupTripChannelUnavailable,
           p * primary_breaker_failure * (1.0 - backup_success),
           input.uncleared_terminal_time_s);
  }

  const double probability_sum = std::accumulate(
      result.classes.begin(), result.classes.end(), 0.0,
      [](double sum, const ProtectionCyberGeneratedClass& generated) {
        return sum + generated.protection.conditional_probability;
      });
  result.class_probabilities_normalized =
      std::abs(probability_sum - 1.0) <= 1e-12;
  if (!result.class_probabilities_normalized)
    throw std::runtime_error(
        "protection/cyber event tree failed probability normalization");
  return result;
}

ProtectionCyberReliabilityComparison compare_protection_cyber_reliability(
    const std::vector<ProtectionCyberReliabilityScenario>& scenarios,
    double curtailment_threshold_mw) {
  if (!std::isfinite(curtailment_threshold_mw) ||
      curtailment_threshold_mw < 0.0)
    throw std::invalid_argument("curtailment threshold must be non-negative");
  ProtectionCyberReliabilityComparison result;
  for (const auto& scenario : scenarios) {
    if (!std::isfinite(scenario.initiating_frequency_per_year) ||
        scenario.initiating_frequency_per_year < 0.0)
      throw std::invalid_argument(
          "initiating fault frequency must be non-negative");
    const double frequency = scenario.initiating_frequency_per_year;
    const auto joint = generate_protection_cyber_classes(scenario.event);
    result.validity.information_topology_modelled =
        result.validity.information_topology_modelled ||
        (!scenario.event.information_components.empty() &&
         !scenario.event.information_functions.empty());
    result.validity.information_qos_modelled =
        result.validity.information_qos_modelled || std::any_of(
            scenario.event.information_functions.begin(),
            scenario.event.information_functions.end(), [](const auto& f) {
              return f.max_latency_ms > 0.0 || f.max_jitter_ms > 0.0 ||
                  f.min_packet_delivery_probability > 0.0;
            });
    result.validity.common_cause_conditioned =
        result.validity.common_cause_conditioned ||
        joint.information.common_cause_conditioned;
    result.validity.cyber_power_dependency_modelled =
        result.validity.cyber_power_dependency_modelled ||
        joint.information.power_dependency_modelled;
    ProtectionCyberEventInput protection_only_event = scenario.event;
    protection_only_event.information_components.clear();
    protection_only_event.information_functions.clear();
    protection_only_event.information_environments.clear();
    protection_only_event.function_bindings = {};
    const auto protection_only =
        generate_protection_cyber_classes(protection_only_event);

    const double static_eens = frequency *
        scenario.consequence.uncleared_shed_mw *
        scenario.consequence.repair_hr;
    result.static_fmea_eens_mwh_yr += static_eens;
    if (scenario.consequence.uncleared_shed_mw >
        curtailment_threshold_mw) {
      result.static_fmea_lole_hr_yr +=
          frequency * scenario.consequence.repair_hr;
      result.static_fmea_lolf_occ_yr += frequency;
    }
    const auto protection_metrics = aggregate_generated_classes(
        protection_only, protection_only_event, scenario.consequence,
        frequency, curtailment_threshold_mw);
    const auto joint_metrics = aggregate_generated_classes(
        joint, scenario.event, scenario.consequence, frequency,
        curtailment_threshold_mw);
    result.protection_only_eens_mwh_yr += protection_metrics.eens;
    result.protection_only_lole_hr_yr += protection_metrics.lole;
    result.protection_only_lolf_occ_yr += protection_metrics.lolf;
    result.cyber_conditioned_eens_mwh_yr += joint_metrics.eens;
    result.cyber_conditioned_lole_hr_yr += joint_metrics.lole;
    result.cyber_conditioned_lolf_occ_yr += joint_metrics.lolf;
    result.joint_classes_evaluated += static_cast<int>(joint.classes.size());
    ++result.scenarios_evaluated;
  }
  result.cyber_increment_mwh_yr = result.cyber_conditioned_eens_mwh_yr -
      result.protection_only_eens_mwh_yr;
  result.protection_benefit_mwh_yr = result.static_fmea_eens_mwh_yr -
      result.protection_only_eens_mwh_yr;
  result.validity.relay_logic_modelled = true;
  result.validity.protection_coordination_modelled = true;
  result.validity.breaker_failure_modelled = true;
  result.validity.der_ride_through_modelled = std::any_of(
      scenarios.begin(), scenarios.end(), [](const auto& scenario) {
        return !scenario.event.ders.empty();
      });
  result.validity.online_network_dae_coupled = false;
  return result;
}

ProtectionFRTReliabilityResult aggregate_protection_frt_reliability(
    const std::vector<ProtectionFRTReliabilityScenario>& scenarios,
    double curtailment_threshold_mw) {
  if (!std::isfinite(curtailment_threshold_mw) ||
      curtailment_threshold_mw < 0.0)
    throw std::invalid_argument("curtailment threshold must be non-negative");
  ProtectionFRTReliabilityResult result;
  for (const auto& scenario : scenarios) {
    if (!std::isfinite(scenario.initiating_frequency_per_year) ||
        scenario.initiating_frequency_per_year < 0.0)
      throw std::invalid_argument("initiating fault frequency must be non-negative");
    if (!scenario.trace_classes_validated)
      throw std::invalid_argument(
          "protection/FRT reliability requires trace-validated generated classes");
    double class_probability_sum = 0.0;
    for (const auto& item : scenario.classes) {
      const double class_probability =
          item.generated_class.conditional_probability;
      validate_probability(class_probability, "protection/FRT class probability");
      class_probability_sum += class_probability;
      double event_ens = 0.0;
      double event_lole = 0.0;
      for (const auto& stage : item.shed_stages) {
        if (!std::isfinite(stage.duration_hr) || stage.duration_hr < 0.0 ||
            !std::isfinite(stage.shed_mw) || stage.shed_mw < 0.0)
          throw std::invalid_argument("invalid protection/FRT shed stage");
        event_ens += stage.duration_hr * stage.shed_mw;
        if (stage.shed_mw > curtailment_threshold_mw)
          event_lole += stage.duration_hr;
      }
      const double weighted_frequency =
          scenario.initiating_frequency_per_year * class_probability;
      result.eens_mwh_yr += weighted_frequency * event_ens;
      result.lole_hr_yr += weighted_frequency * event_lole;
      if (event_lole > 0.0) result.lolf_occ_yr += weighted_frequency;
      ++result.classes_evaluated;
    }
    if (std::abs(class_probability_sum - 1.0) > 1e-9)
      throw std::invalid_argument(
          "protection/FRT scenario class probabilities must sum to one");
    ++result.scenarios_evaluated;
  }
  result.validity.protection_frt_reliability_coupled = true;
  result.validity.der_ride_through_modelled = true;
  result.validity.protection_coordination_modelled = true;
  result.validity.breaker_failure_modelled = true;
  result.validity.online_dae_coupled = false;
  return result;
}

}  // namespace hacdcpf::analysis
