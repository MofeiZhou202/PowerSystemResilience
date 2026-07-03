#pragma once

/// model/telemetry.hpp
/// ===================
/// Temporal + telemetry data model for the digital-twin integration axis
/// (docs/digital_twin_data_io_architecture.md §10).  These types let a static
/// network model bind to live or historical measurements and carry timestamped
/// state seeds for state reconciliation (see io/state_estimation.hpp).
///
/// Lives in the model layer because it is optional state attached to
/// HybridPowerSystem; io/json_io serializes it and io/state_estimation consumes
/// it.

#include <string>
#include <vector>

namespace hacdcpf {

/// Physical quantity a measurement observes.
enum class MeasurementType {
  Voltage,
  Angle,
  ActivePower,
  ReactivePower,
  Current,
  StateOfCharge,
  TapPosition,
  Status,
};

/// Sign convention of a power measurement.
enum class SignConvention { Load, Generator };

/// Reference frame for angle / sequence quantities.
enum class ReferenceFrame { PhaseGround, PmuGlobal, Sequence };

/// Measurement quality flag (bad/stale are excluded before influencing state).
enum class QualityFlag { Good, Suspect, Bad, Stale };

/// Multi-rate stream alignment policy.
enum class ResamplePolicy { Hold, Linear, Nearest };

/// Time base for a telemetry stream (§10.1): single UTC epoch, explicit
/// resolution, explicit multi-rate alignment policy.
struct TimeBase {
  std::string epoch_utc;        ///< ISO-8601 stream start, e.g. "2026-07-03T00:00:00Z".
  std::string timezone{"UTC"};
  double resolution_s{1.0};     ///< Nominal sample spacing in seconds.
  ResamplePolicy resample{ResamplePolicy::Hold};
};

/// Binds one model quantity to one measurement source (§10.2).
struct TelemetryBinding {
  std::string component_ref;    ///< e.g. "ac.buses[3]" or a component name.
  MeasurementType measurement_type{MeasurementType::Voltage};
  std::string phase;            ///< "", "A"/"B"/"C", or "pos"/"neg"/"zero".
  std::string unit;
  SignConvention sign_convention{SignConvention::Load};
  ReferenceFrame reference_frame{ReferenceFrame::PhaseGround};
  std::string source_system;    ///< "SCADA","PMU","AMI","DERMS","Historian".
  std::string tag;              ///< point/tag name in the source system.
  double sampling_interval_s{1.0};
  double deadband{0.0};
  double scale{1.0};
  std::string cardinality{"1:1"};   ///< e.g. "1:1", "1:N", derived.
  std::string constrains_state;     ///< state var, e.g. "vm_pu@bus3".
  QualityFlag quality{QualityFlag::Good};
  std::string timestamp;            ///< last-update timestamp (ISO-8601).
};

/// One time-stamped sample value.
struct TelemetrySample {
  std::string timestamp;
  double value{0.0};
  QualityFlag quality{QualityFlag::Good};
};

/// A homogeneous stream of measurements sharing one time base.
struct TelemetryStream {
  std::string stream_id;
  TimeBase time_base;
  std::vector<TelemetryBinding> bindings;
  std::vector<TelemetrySample> samples;
};

/// A timestamped seed for one model state variable, reconciled against a
/// snapshot solve (§10.1, §10.3).
struct StateSeed {
  std::string timestamp;
  std::string component_ref;
  std::string quantity;        ///< "vm_pu","va_rad","soc","p_mw","q_mvar","status".
  double value{0.0};
  std::string provenance;
};

/// Optional telemetry section attached to a HybridPowerSystem.
struct TelemetrySection {
  std::vector<TelemetryStream> streams;
  std::vector<StateSeed> state_seeds;

  [[nodiscard]] bool empty() const {
    return streams.empty() && state_seeds.empty();
  }
  /// Total number of measurement bindings across all streams.
  [[nodiscard]] std::size_t binding_count() const {
    std::size_t n = 0;
    for (const auto& s : streams) n += s.bindings.size();
    return n;
  }
};

// ── String conversions (inline; header-only) ─────────────────────────────────

inline std::string to_string(MeasurementType t) {
  switch (t) {
    case MeasurementType::Voltage:       return "voltage";
    case MeasurementType::Angle:         return "angle";
    case MeasurementType::ActivePower:   return "active_power";
    case MeasurementType::ReactivePower: return "reactive_power";
    case MeasurementType::Current:       return "current";
    case MeasurementType::StateOfCharge: return "soc";
    case MeasurementType::TapPosition:   return "tap_position";
    case MeasurementType::Status:        return "status";
  }
  return "unknown";
}

inline std::string to_string(SignConvention c) {
  return c == SignConvention::Generator ? "generator" : "load";
}

inline std::string to_string(ReferenceFrame f) {
  switch (f) {
    case ReferenceFrame::PhaseGround: return "phase_ground";
    case ReferenceFrame::PmuGlobal:   return "pmu_global";
    case ReferenceFrame::Sequence:    return "sequence";
  }
  return "unknown";
}

inline std::string to_string(QualityFlag q) {
  switch (q) {
    case QualityFlag::Good:    return "good";
    case QualityFlag::Suspect: return "suspect";
    case QualityFlag::Bad:     return "bad";
    case QualityFlag::Stale:   return "stale";
  }
  return "unknown";
}

inline std::string to_string(ResamplePolicy p) {
  switch (p) {
    case ResamplePolicy::Hold:    return "hold";
    case ResamplePolicy::Linear:  return "linear";
    case ResamplePolicy::Nearest: return "nearest";
  }
  return "unknown";
}

inline MeasurementType measurement_type_from_string(const std::string& s) {
  if (s == "angle") return MeasurementType::Angle;
  if (s == "active_power") return MeasurementType::ActivePower;
  if (s == "reactive_power") return MeasurementType::ReactivePower;
  if (s == "current") return MeasurementType::Current;
  if (s == "soc") return MeasurementType::StateOfCharge;
  if (s == "tap_position") return MeasurementType::TapPosition;
  if (s == "status") return MeasurementType::Status;
  return MeasurementType::Voltage;
}

inline SignConvention sign_convention_from_string(const std::string& s) {
  return s == "generator" ? SignConvention::Generator : SignConvention::Load;
}

inline ReferenceFrame reference_frame_from_string(const std::string& s) {
  if (s == "pmu_global") return ReferenceFrame::PmuGlobal;
  if (s == "sequence") return ReferenceFrame::Sequence;
  return ReferenceFrame::PhaseGround;
}

inline QualityFlag quality_flag_from_string(const std::string& s) {
  if (s == "suspect") return QualityFlag::Suspect;
  if (s == "bad") return QualityFlag::Bad;
  if (s == "stale") return QualityFlag::Stale;
  return QualityFlag::Good;
}

inline ResamplePolicy resample_policy_from_string(const std::string& s) {
  if (s == "linear") return ResamplePolicy::Linear;
  if (s == "nearest") return ResamplePolicy::Nearest;
  return ResamplePolicy::Hold;
}

}  // namespace hacdcpf
