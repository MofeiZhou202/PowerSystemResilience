#pragma once

/// model/typical_parameters.hpp
/// ============================
/// Opt-in engineering defaults for sparse hybrid AC/DC distribution models.
///
/// This is intentionally separate from the low-level struct initializers and
/// JSON parser defaults.  Case import and round-trip fidelity stay unchanged;
/// callers apply this helper when they want a computation-ready model from a
/// sparse study case.

#include <algorithm>
#include <cmath>
#include <string>

#include "hacdcpf/model/defaults.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf {

struct TypicalParameters {
  // System bases
  static constexpr double kBaseMva = Defaults::kBaseMva;
  static constexpr double kFreqHz = Defaults::kFreqHz;
  static constexpr double kACDistributionKv = 10.0;
  static constexpr double kLowVoltageACDistributionKv = 0.4;
  static constexpr double kDCDistributionKv = 0.75;

  // Voltage and branch data
  static constexpr double kVoltagePu = Defaults::kBusVoltage;
  static constexpr double kVoltageMinPu = Defaults::kVoltageMinPu;
  static constexpr double kVoltageMaxPu = Defaults::kVoltageMaxPu;
  static constexpr double kACBranchRPu = 0.01;
  static constexpr double kACBranchXPu = 0.04;
  static constexpr double kACBranchRToX = 0.25;
  static constexpr double kDCBranchRPu = 0.01;
  static constexpr double kZeroSeqMultiplier = 3.0;

  // Equipment ratings and limits
  static constexpr double kGeneratorCapacityMw = 10.0;
  static constexpr double kDERCapacityMw = 1.0;
  static constexpr double kStoragePowerMw = 1.0;
  static constexpr double kStorageDurationH = 2.0;
  static constexpr double kReactiveLimitRatio = 0.5;
  static constexpr double kConverterRatedMw = 1.0;
  static constexpr double kConverterEfficiency = 0.99;
  static constexpr double kDCDCConverterEfficiency = 0.98;
  static constexpr double kConverterShortCircuitXPu = 0.15;
  static constexpr double kConverterShortCircuitRPu = 0.01;
  static constexpr double kConverterCurrentLimitPu = 1.2;
  static constexpr double kConverterACCurrentLimitPu = 1.1;
  static constexpr double kConverterDCCurrentLimitPu = 1.1;
  static constexpr double kConverterModulationGain = 1.0;
  static constexpr double kConverterModulationMin = 0.05;
  static constexpr double kConverterModulationMax = 1.0;
  static constexpr double kConverterACLossRPu = 0.005;

  // Transformer nameplate defaults
  static constexpr double kTransformerSnMva = 10.0;
  static constexpr double kTransformerVkPercent = 6.0;
  static constexpr double kTransformerVkrPercent = 0.6;
  static constexpr double kTransformerPfeKw = 10.0;
  static constexpr double kTransformerI0Percent = 0.1;
  inline static constexpr const char* kTransformerVectorGroup = "Dyn11";

  // Short-circuit and dynamic placeholders
  static constexpr double kExternalGridSscMaxMva = 1000.0;
  static constexpr double kExternalGridSscMinMva = 500.0;
  static constexpr double kExternalGridRX = 0.1;
  static constexpr double kGeneratorInertiaH = 3.5;
  static constexpr double kGeneratorDroop = 0.05;
  static constexpr double kGeneratorXdPu = 1.8;
  static constexpr double kGeneratorXdpPu = 0.3;
  static constexpr double kGeneratorXdppPu = 0.2;
  static constexpr double kGeneratorRaPu = 0.002;
  static constexpr double kMotorXRPu = 5.0;
  static constexpr double kMotorLockedRotorCurrentPu = 6.0;

  // PV and storage
  static constexpr double kPVInverterEfficiency = 0.97;
  static constexpr double kPVIrradianceWm2 = 1000.0;
  static constexpr double kPVTemperatureC = 25.0;
  static constexpr double kPVVmpp = 30.0;
  static constexpr double kPVImpp = 8.5;
  static constexpr double kPVVoc = 37.0;
  static constexpr double kPVIsc = 9.0;
  static constexpr double kSOCInit = 0.5;
  static constexpr double kSOCMin = 0.1;
  static constexpr double kSOCMax = 0.9;
  static constexpr double kStorageEfficiency = 0.95;
  static constexpr int kStorageMaxCycles = 5000;

  // Reliability planning placeholders
  static constexpr double kBranchFailureRatePerYear = 0.05;
  static constexpr double kBranchMttrHr = 4.0;
  static constexpr double kEquipmentMtbfHr = 87600.0;
  static constexpr double kEquipmentMttrHr = 8.0;
  static constexpr double kSwitchFailureProbability = 0.01;
  static constexpr double kSwitchOperationTimeS = 1.0;
  static constexpr double kBreakerRatedCurrentKa = 1.25;
  static constexpr double kBreakerInterruptingCurrentKa = 25.0;

  static constexpr double kMissingTol = 1e-12;
};

struct TypicalParameterOptions {
  bool fill_electrical{true};
  bool fill_operational_limits{true};
  bool fill_short_circuit{true};
  bool fill_reliability{true};
};

struct TypicalParameterFillReport {
  int electrical_fields{0};
  int operational_limit_fields{0};
  int short_circuit_fields{0};
  int reliability_fields{0};

  int total() const {
    return electrical_fields + operational_limit_fields +
           short_circuit_fields + reliability_fields;
  }
};

namespace typical_parameter_detail {

inline bool missing(double value) {
  return !std::isfinite(value) ||
         std::abs(value) <= TypicalParameters::kMissingTol;
}

inline bool missing_positive(double value) {
  return !std::isfinite(value) ||
         value <= TypicalParameters::kMissingTol;
}

inline int fill_positive(double& value, double fallback) {
  if (missing_positive(value) && std::isfinite(fallback) &&
      fallback > TypicalParameters::kMissingTol) {
    value = fallback;
    return 1;
  }
  return 0;
}

inline int fill_missing(double& value, double fallback) {
  if (missing(value) && std::isfinite(fallback)) {
    const bool changed =
        !std::isfinite(value) ||
        std::abs(value - fallback) > TypicalParameters::kMissingTol;
    value = fallback;
    return changed ? 1 : 0;
  }
  return 0;
}

inline int fill_positive_int(int& value, int fallback) {
  if (value <= 0 && fallback > 0) {
    value = fallback;
    return 1;
  }
  return 0;
}

inline int fill_efficiency(double& value, double fallback) {
  if (!std::isfinite(value) || value <= TypicalParameters::kMissingTol ||
      value > 1.0) {
    value = fallback;
    return 1;
  }
  return 0;
}

inline int fill_nonfinite(double& value, double fallback) {
  if (!std::isfinite(value)) {
    value = fallback;
    return 1;
  }
  return 0;
}

inline int fill_string(std::string& value, const char* fallback) {
  if (value.empty() && fallback != nullptr) {
    value = fallback;
    return 1;
  }
  return 0;
}

inline double max_positive(double a, double b) {
  const double aa = (std::isfinite(a) && a > 0.0) ? a : 0.0;
  const double bb = (std::isfinite(b) && b > 0.0) ? b : 0.0;
  return std::max(aa, bb);
}

inline double max_positive(double a, double b, double c) {
  return max_positive(max_positive(a, b), c);
}

inline double max_positive(double a, double b, double c, double d) {
  return max_positive(max_positive(a, b), max_positive(c, d));
}

inline double apparent_power_mva(double p_mw, double q_mvar) {
  const double p = std::isfinite(p_mw) ? p_mw : 0.0;
  const double q = std::isfinite(q_mvar) ? q_mvar : 0.0;
  return std::sqrt(p * p + q * q);
}

inline double ac_base_mva(const HybridPowerSystem& sys) {
  if (sys.ac.base_mva > TypicalParameters::kMissingTol) return sys.ac.base_mva;
  if (sys.base_mva > TypicalParameters::kMissingTol) return sys.base_mva;
  return TypicalParameters::kBaseMva;
}

inline double dc_base_mva(const HybridPowerSystem& sys) {
  if (sys.dc.base_mva > TypicalParameters::kMissingTol) return sys.dc.base_mva;
  if (sys.base_mva > TypicalParameters::kMissingTol) return sys.base_mva;
  return TypicalParameters::kBaseMva;
}

inline double ac_bus_kv(const HybridPowerSystem& sys, int bus, double fallback) {
  for (const auto& b : sys.ac.buses) {
    if (b.index == bus && b.base_kv > TypicalParameters::kMissingTol) {
      return b.base_kv;
    }
  }
  return fallback;
}

inline double dc_bus_kv(const HybridPowerSystem& sys, int bus, double fallback) {
  for (const auto& b : sys.dc.buses) {
    if (b.index == bus && b.base_kv > TypicalParameters::kMissingTol) {
      return b.base_kv;
    }
  }
  return fallback;
}

inline double total_ac_load_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& b : sys.ac.buses) total += std::max(0.0, b.pd_mw);
  for (const auto& l : sys.ac.loads) total += std::max(0.0, l.p_mw) * std::max(0.0, l.scaling);
  for (const auto& l : sys.ac.flexible_loads) total += std::max(0.0, l.p_mw);
  for (const auto& l : sys.ac.asymmetric_loads) {
    total += std::max(0.0, l.pa_mw + l.pb_mw + l.pc_mw) * std::max(0.0, l.scaling);
  }
  for (const auto& c : sys.ac.charging_stations) total += std::max(0.0, c.max_power_kw) / 1000.0;
  return total;
}

inline double total_dc_load_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& b : sys.dc.buses) total += std::max(0.0, b.pd_mw);
  for (const auto& l : sys.dc.loads) total += std::max(0.0, l.p_mw) * std::max(0.0, l.scaling);
  return total;
}

}  // namespace typical_parameter_detail

inline TypicalParameterFillReport apply_typical_parameters(
    HybridPowerSystem& sys,
    const TypicalParameterOptions& options = {}) {
  namespace d = typical_parameter_detail;
  TypicalParameterFillReport report;

  if (options.fill_electrical) {
    report.electrical_fields += d::fill_positive(sys.base_mva, TypicalParameters::kBaseMva);
    report.electrical_fields += d::fill_positive(sys.ac.base_mva, sys.base_mva);
    report.electrical_fields += d::fill_positive(sys.dc.base_mva, sys.base_mva);
    report.electrical_fields += d::fill_positive(sys.ac.freq_hz, TypicalParameters::kFreqHz);
  }

  const double ac_base = d::ac_base_mva(sys);
  const double dc_base = d::dc_base_mva(sys);
  const double ac_load_hint =
      d::max_positive(d::total_ac_load_mw(sys), TypicalParameters::kGeneratorCapacityMw);
  const double dc_load_hint =
      d::max_positive(d::total_dc_load_mw(sys), TypicalParameters::kConverterRatedMw);

  if (options.fill_electrical) {
    for (auto& b : sys.ac.buses) {
      report.electrical_fields += d::fill_positive(b.vm_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(b.base_kv, TypicalParameters::kACDistributionKv);
      report.electrical_fields += d::fill_positive(b.vmin_pu, TypicalParameters::kVoltageMinPu);
      report.electrical_fields += d::fill_positive(b.vmax_pu, TypicalParameters::kVoltageMaxPu);
      if (b.vmax_pu <= b.vmin_pu) {
        b.vmin_pu = TypicalParameters::kVoltageMinPu;
        b.vmax_pu = TypicalParameters::kVoltageMaxPu;
        report.electrical_fields += 2;
      }
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(
            b.i_breaker_ka, TypicalParameters::kBreakerInterruptingCurrentKa);
      }
    }

    for (auto& b : sys.dc.buses) {
      report.electrical_fields += d::fill_positive(b.vm_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(b.base_kv, TypicalParameters::kDCDistributionKv);
      report.electrical_fields += d::fill_positive(b.vmin_pu, Defaults::kDCVoltageMinPu);
      report.electrical_fields += d::fill_positive(b.vmax_pu, Defaults::kDCVoltageMaxPu);
      if (b.vmax_pu <= b.vmin_pu) {
        b.vmin_pu = Defaults::kDCVoltageMinPu;
        b.vmax_pu = Defaults::kDCVoltageMaxPu;
        report.electrical_fields += 2;
      }
    }

    for (auto& br : sys.ac.branches) {
      report.electrical_fields += d::fill_positive(br.tap, Defaults::kTapRatio);
      if (d::missing(br.r_pu) && d::missing(br.x_pu)) {
        br.r_pu = TypicalParameters::kACBranchRPu;
        br.x_pu = TypicalParameters::kACBranchXPu;
        report.electrical_fields += 2;
      } else if (d::missing(br.r_pu) && !d::missing(br.x_pu)) {
        br.r_pu = std::abs(br.x_pu) * TypicalParameters::kACBranchRToX;
        report.electrical_fields += 1;
      } else if (!d::missing(br.r_pu) && d::missing(br.x_pu)) {
        br.x_pu = std::abs(br.r_pu) / TypicalParameters::kACBranchRToX;
        report.electrical_fields += 1;
      }
      report.electrical_fields += d::fill_positive_int(br.n_parallel, 1);
      report.electrical_fields += d::fill_positive(br.rate_a_mva, ac_base);
      report.electrical_fields += d::fill_positive(br.rate_b_mva, br.rate_a_mva);
      report.electrical_fields += d::fill_positive(br.rate_c_mva, br.rate_a_mva);
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(
            br.r0_pu, std::abs(br.r_pu) * TypicalParameters::kZeroSeqMultiplier);
        report.short_circuit_fields += d::fill_positive(
            br.x0_pu, std::abs(br.x_pu) * TypicalParameters::kZeroSeqMultiplier);
      }
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(
            br.failure_rate, TypicalParameters::kBranchFailureRatePerYear);
        report.reliability_fields += d::fill_positive(
            br.mttr_hr, TypicalParameters::kBranchMttrHr);
      }
    }

    for (auto& br : sys.dc.branches) {
      report.electrical_fields += d::fill_positive(br.r_pu, TypicalParameters::kDCBranchRPu);
      report.electrical_fields += d::fill_positive(br.base_kv,
          d::dc_bus_kv(sys, br.from_bus, TypicalParameters::kDCDistributionKv));
      report.electrical_fields += d::fill_positive(br.rate_a_mva, dc_base);
      report.electrical_fields += d::fill_positive(br.s_max_mva, br.rate_a_mva);
      report.electrical_fields += d::fill_positive_int(br.n_parallel, 1);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(
            br.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(
            br.mttr_hours, TypicalParameters::kBranchMttrHr);
      }
    }
  }

  if (options.fill_operational_limits) {
    for (auto& g : sys.ac.generators) {
      const double rating = d::max_positive(g.mbase_mva, std::abs(g.pg_mw),
                                            ac_load_hint, TypicalParameters::kGeneratorCapacityMw);
      report.operational_limit_fields += d::fill_positive(g.mbase_mva, rating);
      report.operational_limit_fields += d::fill_positive(g.pmax_mw, rating);
      report.operational_limit_fields += d::fill_missing(g.pmin_mw, 0.0);
      report.operational_limit_fields += d::fill_positive(
          g.qmax_mvar, rating * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(g.qmin_mvar)) {
        g.qmin_mvar = -g.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      report.electrical_fields += d::fill_positive(g.vg_pu, TypicalParameters::kVoltagePu);
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(g.inertia_h, TypicalParameters::kGeneratorInertiaH);
        report.short_circuit_fields += d::fill_positive(g.droop_r, TypicalParameters::kGeneratorDroop);
        report.short_circuit_fields += d::fill_positive(g.xd_pu, TypicalParameters::kGeneratorXdPu);
        report.short_circuit_fields += d::fill_positive(g.xdp_pu, TypicalParameters::kGeneratorXdpPu);
        report.short_circuit_fields += d::fill_positive(g.xdpp_pu, TypicalParameters::kGeneratorXdppPu);
        report.short_circuit_fields += d::fill_positive(g.ra_pu, TypicalParameters::kGeneratorRaPu);
        report.short_circuit_fields += d::fill_positive(g.vn_kv,
            d::ac_bus_kv(sys, g.bus, TypicalParameters::kACDistributionKv));
        report.short_circuit_fields += d::fill_positive(g.x0_pu, TypicalParameters::kGeneratorXdppPu);
        report.short_circuit_fields += d::fill_positive(g.r0_pu, TypicalParameters::kGeneratorRaPu);
      }
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(g.forced_outage_rate, 0.05);
        report.reliability_fields += d::fill_positive(g.mttr_hr, TypicalParameters::kEquipmentMttrHr);
      }
    }

    auto fill_static_generator = [&](StaticGenerator& g, bool ac_domain) {
      const double hint = ac_domain ? ac_load_hint : dc_load_hint;
      const double rating = d::max_positive(g.sn_mva, g.p_rated_mw,
                                            std::abs(g.p_mw), TypicalParameters::kDERCapacityMw);
      const double final_rating = d::max_positive(rating, hint * 0.25);
      report.operational_limit_fields += d::fill_positive(g.p_rated_mw, final_rating);
      report.operational_limit_fields += d::fill_positive(g.sn_mva, g.p_rated_mw);
      report.operational_limit_fields += d::fill_positive(g.pmax_mw, g.p_rated_mw);
      report.operational_limit_fields += d::fill_missing(g.pmin_mw, 0.0);
      report.operational_limit_fields += d::fill_positive(
          g.qmax_mvar, g.sn_mva * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(g.qmin_mvar)) {
        g.qmin_mvar = -g.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      report.electrical_fields += d::fill_positive(g.v_ref_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(g.f_ref_hz, TypicalParameters::kFreqHz);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(g.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(g.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    };
    for (auto& g : sys.ac.static_generators) fill_static_generator(g, true);
    for (auto& g : sys.dc.static_generators) fill_static_generator(g, false);

    for (auto& g : sys.dc.dc_static_generators) {
      const double rating = d::max_positive(g.pmax_mw, g.p_set_mw, dc_load_hint,
                                            TypicalParameters::kDERCapacityMw);
      report.operational_limit_fields += d::fill_positive(g.pmax_mw, rating);
      report.operational_limit_fields += d::fill_missing(g.pmin_mw, 0.0);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(g.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(g.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }

    for (auto& r : sys.ac.renewable_gens) {
      const double rating = d::max_positive(r.p_rated_mw, std::abs(r.p_mw),
                                            TypicalParameters::kDERCapacityMw);
      report.operational_limit_fields += d::fill_positive(r.p_rated_mw, rating);
      report.operational_limit_fields += d::fill_positive(
          r.qmax_mvar, rating * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(r.qmin_mvar)) {
        r.qmin_mvar = -r.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(r.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(r.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }

    for (auto& p : sys.ac.pv_systems) {
      const double rating = d::max_positive(p.sn_mva, p.pmax_mw, std::abs(p.p_mw),
                                            TypicalParameters::kDERCapacityMw);
      report.operational_limit_fields += d::fill_positive(p.sn_mva, rating);
      report.operational_limit_fields += d::fill_positive(p.pmax_mw, p.sn_mva);
      report.operational_limit_fields += d::fill_missing(p.pmin_mw, 0.0);
      report.operational_limit_fields += d::fill_positive(
          p.qmax_mvar, p.sn_mva * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(p.qmin_mvar)) {
        p.qmin_mvar = -p.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      report.electrical_fields += d::fill_efficiency(p.inverter_eff, TypicalParameters::kPVInverterEfficiency);
      report.electrical_fields += d::fill_positive(p.v_ac_set_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(p.v_dc_set_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive_int(p.num_series, 1);
      report.electrical_fields += d::fill_positive_int(p.num_parallel, 1);
      report.electrical_fields += d::fill_positive(p.vmpp, TypicalParameters::kPVVmpp);
      report.electrical_fields += d::fill_positive(p.impp, TypicalParameters::kPVImpp);
      report.electrical_fields += d::fill_positive(p.voc, TypicalParameters::kPVVoc);
      report.electrical_fields += d::fill_positive(p.isc, TypicalParameters::kPVIsc);
      report.electrical_fields += d::fill_nonfinite(p.irradiance, TypicalParameters::kPVIrradianceWm2);
      report.electrical_fields += d::fill_nonfinite(p.temperature, TypicalParameters::kPVTemperatureC);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(p.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(p.mttr_hours, TypicalParameters::kEquipmentMttrHr);
        report.reliability_fields += d::fill_positive(p.mtbf_panel_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(p.mttr_panel_hours, TypicalParameters::kEquipmentMttrHr);
        report.reliability_fields += d::fill_positive(p.mtbf_inverter_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(p.mttr_inverter_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }

    for (auto& p : sys.dc.pv_arrays) {
      report.electrical_fields += d::fill_positive_int(p.num_series, 1);
      report.electrical_fields += d::fill_positive_int(p.num_parallel, 1);
      report.electrical_fields += d::fill_positive(p.vmpp, TypicalParameters::kPVVmpp);
      report.electrical_fields += d::fill_positive(p.impp, TypicalParameters::kPVImpp);
      report.electrical_fields += d::fill_positive(p.voc, TypicalParameters::kPVVoc);
      report.electrical_fields += d::fill_positive(p.isc, TypicalParameters::kPVIsc);
      report.electrical_fields += d::fill_nonfinite(p.irradiance, TypicalParameters::kPVIrradianceWm2);
      report.electrical_fields += d::fill_nonfinite(p.temperature, TypicalParameters::kPVTemperatureC);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(p.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(p.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }

    auto fill_storage = [&](Storage& st, bool dc_domain) {
      const double power = d::max_positive(st.p_rated_mw, st.pmax_mw,
                                           std::abs(st.p_mw), TypicalParameters::kStoragePowerMw);
      report.operational_limit_fields += d::fill_positive(st.p_rated_mw, power);
      report.operational_limit_fields += d::fill_positive(st.pmax_mw, st.p_rated_mw);
      if (d::missing(st.pmin_mw)) {
        st.pmin_mw = -st.pmax_mw;
        report.operational_limit_fields += 1;
      }
      if (!dc_domain) {
        report.operational_limit_fields += d::fill_positive(
            st.qmax_mvar, st.p_rated_mw * TypicalParameters::kReactiveLimitRatio);
        if (d::missing(st.qmin_mvar)) {
          st.qmin_mvar = -st.qmax_mvar;
          report.operational_limit_fields += 1;
        }
      }
      report.operational_limit_fields += d::fill_positive(
          st.e_rated_mwh, st.p_rated_mw * TypicalParameters::kStorageDurationH);
      report.operational_limit_fields += d::fill_positive(st.soc_min, TypicalParameters::kSOCMin);
      report.operational_limit_fields += d::fill_positive(st.soc_max, TypicalParameters::kSOCMax);
      if (st.soc_max <= st.soc_min) {
        st.soc_min = TypicalParameters::kSOCMin;
        st.soc_max = TypicalParameters::kSOCMax;
        report.operational_limit_fields += 2;
      }
      if (!std::isfinite(st.soc_init) || st.soc_init < st.soc_min || st.soc_init > st.soc_max) {
        st.soc_init = TypicalParameters::kSOCInit;
        report.operational_limit_fields += 1;
      }
      report.operational_limit_fields += d::fill_efficiency(st.eta_charge, TypicalParameters::kStorageEfficiency);
      report.operational_limit_fields += d::fill_efficiency(st.eta_discharge, TypicalParameters::kStorageEfficiency);
      report.operational_limit_fields += d::fill_positive_int(st.max_cycles, TypicalParameters::kStorageMaxCycles);
      report.operational_limit_fields += d::fill_positive(st.soh, 1.0);
      report.operational_limit_fields += d::fill_positive(st.e_mwh, st.e_rated_mwh * st.soc_init);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(st.forced_outage_rate, 0.02);
        report.reliability_fields += d::fill_positive(st.mttr_hr, TypicalParameters::kEquipmentMttrHr);
        report.reliability_fields += d::fill_positive(st.mtbf_battery_hr, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(st.mttr_battery_hr, TypicalParameters::kEquipmentMttrHr);
        report.reliability_fields += d::fill_positive(st.mtbf_pcs_hr, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(st.mttr_pcs_hr, TypicalParameters::kEquipmentMttrHr);
        report.reliability_fields += d::fill_positive(st.mtbf_bms_hr, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(st.mttr_bms_hr, TypicalParameters::kEquipmentMttrHr);
      }
    };
    for (auto& st : sys.ac.storage) fill_storage(st, false);
    for (auto& st : sys.dc.storage) fill_storage(st, true);

    for (auto& st : sys.dc.dc_storage) {
      const double power = d::max_positive(st.p_rated_mw, st.pmax_mw,
                                           std::abs(st.p_mw), TypicalParameters::kStoragePowerMw);
      report.operational_limit_fields += d::fill_positive(st.p_rated_mw, power);
      report.operational_limit_fields += d::fill_positive(st.pmax_mw, st.p_rated_mw);
      if (d::missing(st.pmin_mw)) {
        st.pmin_mw = -st.pmax_mw;
        report.operational_limit_fields += 1;
      }
      report.operational_limit_fields += d::fill_positive(
          st.e_rated_mwh, st.p_rated_mw * TypicalParameters::kStorageDurationH);
      report.operational_limit_fields += d::fill_positive(st.soc_min, TypicalParameters::kSOCMin);
      report.operational_limit_fields += d::fill_positive(st.soc_max, TypicalParameters::kSOCMax);
      if (!std::isfinite(st.soc_init) || st.soc_init < st.soc_min || st.soc_init > st.soc_max) {
        st.soc_init = TypicalParameters::kSOCInit;
        report.operational_limit_fields += 1;
      }
      report.operational_limit_fields += d::fill_efficiency(st.eta_charge, TypicalParameters::kStorageEfficiency);
      report.operational_limit_fields += d::fill_efficiency(st.eta_discharge, TypicalParameters::kStorageEfficiency);
      report.operational_limit_fields += d::fill_positive(st.e_mwh, st.e_rated_mwh * st.soc_init);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(st.forced_outage_rate, 0.02);
        report.reliability_fields += d::fill_positive(st.mttr_hr, TypicalParameters::kEquipmentMttrHr);
      }
    }
  }

  if (options.fill_electrical || options.fill_operational_limits) {
    for (auto& e : sys.ac.external_grids) {
      report.electrical_fields += d::fill_positive(e.vm_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(
          e.vn_kv, d::ac_bus_kv(sys, e.bus, TypicalParameters::kACDistributionKv));
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(e.s_sc_max_mva, TypicalParameters::kExternalGridSscMaxMva);
        report.short_circuit_fields += d::fill_positive(e.s_sc_min_mva, TypicalParameters::kExternalGridSscMinMva);
        report.short_circuit_fields += d::fill_positive(e.rx_max, TypicalParameters::kExternalGridRX);
        report.short_circuit_fields += d::fill_positive(e.rx_min, TypicalParameters::kExternalGridRX);
        const double x = ac_base / std::max(e.s_sc_max_mva, TypicalParameters::kMissingTol);
        report.short_circuit_fields += d::fill_positive(e.x_pu, x);
        report.short_circuit_fields += d::fill_positive(e.r_pu, x * e.rx_max);
        report.short_circuit_fields += d::fill_positive(e.x0_pu, e.x_pu * TypicalParameters::kZeroSeqMultiplier);
        report.short_circuit_fields += d::fill_positive(e.r0_pu, e.r_pu * TypicalParameters::kZeroSeqMultiplier);
      }
    }

    for (auto& tr : sys.ac.transformers_2w) {
      report.operational_limit_fields += d::fill_positive(tr.sn_mva, TypicalParameters::kTransformerSnMva);
      report.electrical_fields += d::fill_positive(
          tr.vn_hv_kv, d::ac_bus_kv(sys, tr.hv_bus, TypicalParameters::kACDistributionKv));
      report.electrical_fields += d::fill_positive(
          tr.vn_lv_kv, d::ac_bus_kv(sys, tr.lv_bus, TypicalParameters::kLowVoltageACDistributionKv));
      report.electrical_fields += d::fill_positive(tr.vk_percent, TypicalParameters::kTransformerVkPercent);
      report.electrical_fields += d::fill_positive(tr.vkr_percent, TypicalParameters::kTransformerVkrPercent);
      report.electrical_fields += d::fill_positive(tr.pfe_kw, TypicalParameters::kTransformerPfeKw);
      report.electrical_fields += d::fill_positive(tr.i0_percent, TypicalParameters::kTransformerI0Percent);
      report.electrical_fields += d::fill_string(tr.vector_group, TypicalParameters::kTransformerVectorGroup);
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(tr.z0_percent, tr.vk_percent);
        report.short_circuit_fields += d::fill_positive(tr.x0_r0, TypicalParameters::kZeroSeqMultiplier);
      }
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(tr.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(tr.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }

    for (auto& tr : sys.ac.transformers_3w) {
      report.operational_limit_fields += d::fill_positive(tr.sn_hv_mva, TypicalParameters::kTransformerSnMva);
      report.operational_limit_fields += d::fill_positive(tr.sn_mv_mva, tr.sn_hv_mva);
      report.operational_limit_fields += d::fill_positive(tr.sn_lv_mva, tr.sn_hv_mva);
      report.electrical_fields += d::fill_positive(
          tr.vn_hv_kv, d::ac_bus_kv(sys, tr.hv_bus, TypicalParameters::kACDistributionKv));
      report.electrical_fields += d::fill_positive(
          tr.vn_mv_kv, d::ac_bus_kv(sys, tr.mv_bus, TypicalParameters::kACDistributionKv));
      report.electrical_fields += d::fill_positive(
          tr.vn_lv_kv, d::ac_bus_kv(sys, tr.lv_bus, TypicalParameters::kLowVoltageACDistributionKv));
      report.electrical_fields += d::fill_positive(tr.vk_hv_mv_percent, TypicalParameters::kTransformerVkPercent);
      report.electrical_fields += d::fill_positive(tr.vk_hv_lv_percent, TypicalParameters::kTransformerVkPercent);
      report.electrical_fields += d::fill_positive(tr.vk_mv_lv_percent, TypicalParameters::kTransformerVkPercent);
      report.electrical_fields += d::fill_positive(tr.vkr_hv_mv_percent, TypicalParameters::kTransformerVkrPercent);
      report.electrical_fields += d::fill_positive(tr.vkr_hv_lv_percent, TypicalParameters::kTransformerVkrPercent);
      report.electrical_fields += d::fill_positive(tr.vkr_mv_lv_percent, TypicalParameters::kTransformerVkrPercent);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(tr.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(tr.mttr_hours, TypicalParameters::kEquipmentMttrHr);
      }
    }
  }

  for (auto& l : sys.ac.loads) {
    report.electrical_fields += d::fill_positive(l.scaling, 1.0);
    if (d::missing(l.p_percent_p) && d::missing(l.i_percent_p) && d::missing(l.z_percent_p)) {
      l.p_percent_p = 100.0;
      report.electrical_fields += 1;
    }
    if (d::missing(l.p_percent_q) && d::missing(l.i_percent_q) && d::missing(l.z_percent_q)) {
      l.p_percent_q = 100.0;
      report.electrical_fields += 1;
    }
    const double s = d::apparent_power_mva(l.p_mw, l.q_mvar);
    report.operational_limit_fields += d::fill_positive(l.sn_mva, s);
    if (options.fill_short_circuit && l.motor_percent > TypicalParameters::kMissingTol) {
      report.short_circuit_fields += d::fill_positive(l.x_sub_pu, TypicalParameters::kGeneratorXdppPu);
      report.short_circuit_fields += d::fill_positive(l.r_sc_pu, TypicalParameters::kGeneratorRaPu);
      report.short_circuit_fields += d::fill_positive_int(l.motor_poles, 2);
      report.short_circuit_fields += d::fill_efficiency(l.motor_efficiency, 0.95);
    }
  }
  for (auto& l : sys.dc.loads) {
    report.electrical_fields += d::fill_positive(l.scaling, 1.0);
    if (d::missing(l.p_percent) && d::missing(l.i_percent) && d::missing(l.z_percent)) {
      l.p_percent = 100.0;
      report.electrical_fields += 1;
    }
    report.operational_limit_fields += d::fill_positive(l.p_rated_mw, std::abs(l.p_mw));
  }

  for (auto& c : sys.vsc_converters) {
    const double rating = d::max_positive(c.p_rated_mw, c.pmax_mw,
                                          std::abs(c.p_set_mw), dc_load_hint);
    report.operational_limit_fields += d::fill_positive(c.p_rated_mw, rating);
    report.operational_limit_fields += d::fill_positive(c.pmax_mw, c.p_rated_mw);
    if (d::missing(c.pmin_mw)) {
      c.pmin_mw = -c.pmax_mw;
      report.operational_limit_fields += 1;
    }
    report.operational_limit_fields += d::fill_positive(
        c.qmax_mvar, c.p_rated_mw * TypicalParameters::kReactiveLimitRatio);
    if (d::missing(c.qmin_mvar)) {
      c.qmin_mvar = -c.qmax_mvar;
      report.operational_limit_fields += 1;
    }
    report.electrical_fields += d::fill_positive(c.v_dc_set_pu, TypicalParameters::kVoltagePu);
    report.electrical_fields += d::fill_positive(c.v_ac_set_pu, TypicalParameters::kVoltagePu);
    report.electrical_fields += d::fill_efficiency(c.eta, TypicalParameters::kConverterEfficiency);
    report.electrical_fields += d::fill_positive(c.k_vdc, 0.1);
    report.electrical_fields += d::fill_positive(
        c.vn_ac_kv, d::ac_bus_kv(sys, c.bus_ac, TypicalParameters::kACDistributionKv));
    report.electrical_fields += d::fill_positive(
        c.vn_dc_kv, d::dc_bus_kv(sys, c.bus_dc, TypicalParameters::kDCDistributionKv));
    if (options.fill_short_circuit) {
      report.short_circuit_fields += d::fill_positive(c.r_conv_ac_pu, TypicalParameters::kConverterACLossRPu);
      report.short_circuit_fields += d::fill_positive(c.r_sc_pu, TypicalParameters::kConverterShortCircuitRPu);
      report.short_circuit_fields += d::fill_positive(c.x_sc_pu, TypicalParameters::kConverterShortCircuitXPu);
      report.short_circuit_fields += d::fill_positive(c.i_max_pu, TypicalParameters::kConverterCurrentLimitPu);
      report.short_circuit_fields += d::fill_positive(c.i_ac_max_pu, TypicalParameters::kConverterACCurrentLimitPu);
      report.short_circuit_fields += d::fill_positive(c.i_dc_max_pu, TypicalParameters::kConverterDCCurrentLimitPu);
      report.short_circuit_fields += d::fill_positive(c.k_m_modulation, TypicalParameters::kConverterModulationGain);
      report.short_circuit_fields += d::fill_positive(c.m_min, TypicalParameters::kConverterModulationMin);
      report.short_circuit_fields += d::fill_positive(c.m_max, TypicalParameters::kConverterModulationMax);
    }
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(c.forced_outage_rate, 0.02);
      report.reliability_fields += d::fill_positive(c.mttr_hr, TypicalParameters::kEquipmentMttrHr);
      report.reliability_fields += d::fill_positive(c.mtbf_hr, TypicalParameters::kEquipmentMtbfHr);
    }
  }

  for (auto& c : sys.dc.dcdc_converters) {
    const double rating = d::max_positive(c.sn_mva, c.pmax_mw,
                                          std::abs(c.p_ref_mw), dc_load_hint);
    report.operational_limit_fields += d::fill_positive(c.sn_mva, rating);
    report.operational_limit_fields += d::fill_positive(c.pmax_mw, c.sn_mva);
    if (d::missing(c.pmin_mw)) {
      c.pmin_mw = -c.pmax_mw;
      report.operational_limit_fields += 1;
    }
    report.electrical_fields += d::fill_positive(c.v_ref_pu, TypicalParameters::kVoltagePu);
    report.electrical_fields += d::fill_efficiency(c.eta, TypicalParameters::kDCDCConverterEfficiency);
    report.electrical_fields += d::fill_positive(c.vn_in_kv,
        d::dc_bus_kv(sys, c.bus_in, TypicalParameters::kDCDistributionKv));
    report.electrical_fields += d::fill_positive(c.vn_out_kv,
        d::dc_bus_kv(sys, c.bus_out, TypicalParameters::kDCDistributionKv));
    report.electrical_fields += d::fill_positive(c.r_eq_pu, TypicalParameters::kDCBranchRPu);
    report.electrical_fields += d::fill_positive(c.d_min, 0.05);
    report.electrical_fields += d::fill_positive(c.d_max, 0.95);
    report.electrical_fields += d::fill_positive(c.n_ratio, 1.0);
    report.electrical_fields += d::fill_positive(c.f_switching_hz, 10000.0);
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(c.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
      report.reliability_fields += d::fill_positive(c.mttr_hours, TypicalParameters::kEquipmentMttrHr);
    }
  }

  for (auto& router : sys.energy_routers) {
    const double rating = d::max_positive(router.p_rated_mw, router.pmax_mw,
                                          TypicalParameters::kConverterRatedMw);
    report.operational_limit_fields += d::fill_positive(router.p_rated_mw, rating);
    report.operational_limit_fields += d::fill_positive(router.pmax_mw, router.p_rated_mw);
    if (d::missing(router.pmin_mw)) {
      router.pmin_mw = -router.pmax_mw;
      report.operational_limit_fields += 1;
    }
    report.operational_limit_fields += d::fill_positive(
        router.qmax_mvar, router.p_rated_mw * TypicalParameters::kReactiveLimitRatio);
    if (d::missing(router.qmin_mvar)) {
      router.qmin_mvar = -router.qmax_mvar;
      report.operational_limit_fields += 1;
    }
    report.electrical_fields += d::fill_positive(router.vn_ac_kv, TypicalParameters::kACDistributionKv);
    report.electrical_fields += d::fill_positive(router.vn_dc_kv, TypicalParameters::kDCDistributionKv);
    for (auto& port : router.ports) {
      const double port_rating = d::max_positive(port.pmax_mw, std::abs(port.p_mw), router.p_rated_mw);
      report.operational_limit_fields += d::fill_positive(port.pmax_mw, port_rating);
      if (d::missing(port.pmin_mw)) {
        port.pmin_mw = -port.pmax_mw;
        report.operational_limit_fields += 1;
      }
      report.operational_limit_fields += d::fill_positive(
          port.qmax_mvar, port.pmax_mw * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(port.qmin_mvar)) {
        port.qmin_mvar = -port.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      report.electrical_fields += d::fill_positive(port.voltage_level_kv,
          port.port_type == ERPortType::DC ? TypicalParameters::kDCDistributionKv
                                           : TypicalParameters::kACDistributionKv);
      report.electrical_fields += d::fill_positive(port.v_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(port.v_set_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_efficiency(port.eta, TypicalParameters::kDCDCConverterEfficiency);
    }
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(router.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
      report.reliability_fields += d::fill_positive(router.mttr_hours, TypicalParameters::kEquipmentMttrHr);
    }
  }

  for (auto& sw : sys.ac.switches) {
    if (options.fill_short_circuit) {
      report.short_circuit_fields += d::fill_positive(sw.i_rated_ka, TypicalParameters::kBreakerRatedCurrentKa);
      report.short_circuit_fields += d::fill_positive(sw.i_breaking_ka, TypicalParameters::kBreakerInterruptingCurrentKa);
      report.short_circuit_fields += d::fill_positive(sw.t_operation_s, TypicalParameters::kSwitchOperationTimeS);
    }
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(sw.p_sw_fail, TypicalParameters::kSwitchFailureProbability);
      report.reliability_fields += d::fill_positive(sw.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
      report.reliability_fields += d::fill_positive(sw.mttr_hours, TypicalParameters::kEquipmentMttrHr);
    }
  }
  for (auto& cb : sys.ac.circuit_breakers) {
    report.electrical_fields += d::fill_positive(cb.rated_voltage_kv,
        d::ac_bus_kv(sys, cb.bus_from, TypicalParameters::kACDistributionKv));
    if (options.fill_short_circuit) {
      report.short_circuit_fields += d::fill_positive(cb.i_rated_ka, TypicalParameters::kBreakerRatedCurrentKa);
      report.short_circuit_fields += d::fill_positive(cb.i_breaking_ka, TypicalParameters::kBreakerInterruptingCurrentKa);
    }
  }
  for (auto& cb : sys.dc.dc_circuit_breakers) {
    report.electrical_fields += d::fill_positive(cb.rated_voltage_kv,
        d::dc_bus_kv(sys, cb.bus_from, TypicalParameters::kDCDistributionKv));
    if (options.fill_short_circuit) {
      report.short_circuit_fields += d::fill_positive(cb.i_rated_ka, TypicalParameters::kBreakerRatedCurrentKa);
      report.short_circuit_fields += d::fill_positive(cb.i_breaking_ka, TypicalParameters::kBreakerInterruptingCurrentKa);
    }
  }

  for (auto& cs : sys.ac.charging_stations) {
    report.operational_limit_fields += d::fill_positive(cs.p_fast_max_kw, 60.0);
    report.operational_limit_fields += d::fill_positive(cs.p_slow_max_kw, 7.0);
    const double charger_count =
        static_cast<double>(std::max(1, cs.n_fast + cs.n_slow + cs.num_chargers));
    report.operational_limit_fields += d::fill_positive(
        cs.max_power_kw, charger_count * cs.p_slow_max_kw);
    report.operational_limit_fields += d::fill_efficiency(cs.power_factor, 0.95);
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(cs.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
      report.reliability_fields += d::fill_positive(cs.mttr_hours, TypicalParameters::kEquipmentMttrHr);
    }
  }
  for (auto& ch : sys.ac.chargers) {
    report.operational_limit_fields += d::fill_positive(ch.p_rated_kw, 7.0);
    report.operational_limit_fields += d::fill_positive(ch.p_ch_max_kw, ch.p_rated_kw);
    report.operational_limit_fields += d::fill_efficiency(ch.eta, 0.95);
    if (ch.v2g_capable) {
      report.operational_limit_fields += d::fill_positive(ch.p_dis_max_kw, ch.p_rated_kw);
    }
    if (options.fill_reliability) {
      report.reliability_fields += d::fill_positive(ch.mtbf_hours, TypicalParameters::kEquipmentMtbfHr);
      report.reliability_fields += d::fill_positive(ch.mttr_hours, TypicalParameters::kEquipmentMttrHr);
    }
  }

  for (auto& motor : sys.ac.motors) {
    report.electrical_fields += d::fill_positive(motor.vn_kv,
        d::ac_bus_kv(sys, motor.bus, TypicalParameters::kLowVoltageACDistributionKv));
    report.operational_limit_fields += d::fill_positive(motor.sn_mva, 0.1);
    if (options.fill_short_circuit) {
      report.short_circuit_fields += d::fill_positive(motor.x_r, TypicalParameters::kMotorXRPu);
      report.short_circuit_fields += d::fill_positive(motor.lrc, TypicalParameters::kMotorLockedRotorCurrentPu);
      report.short_circuit_fields += d::fill_positive(motor.x_pu, TypicalParameters::kGeneratorXdppPu);
      report.short_circuit_fields += d::fill_positive(motor.r_pu, TypicalParameters::kGeneratorRaPu);
      report.short_circuit_fields += d::fill_positive(motor.x0_pu, TypicalParameters::kGeneratorXdppPu);
      report.short_circuit_fields += d::fill_positive(motor.r0_pu, TypicalParameters::kGeneratorRaPu);
    }
    report.electrical_fields += d::fill_efficiency(motor.cos_phi, 0.85);
    report.electrical_fields += d::fill_efficiency(motor.efficiency, 0.95);
  }

  for (auto& s : sys.ac.shunts) {
    report.electrical_fields += d::fill_positive_int(s.n_steps, 1);
    report.electrical_fields += d::fill_positive_int(s.current_step, 1);
  }

  if (sys.three_phase_ac.has_value()) {
    auto& tp = *sys.three_phase_ac;
    report.electrical_fields += d::fill_positive(tp.base_mva, ac_base);
    report.electrical_fields += d::fill_positive(tp.base_freq_hz, TypicalParameters::kFreqHz);
    for (auto& b : tp.buses) {
      report.electrical_fields += d::fill_positive(b.base_kv, TypicalParameters::kACDistributionKv);
      report.electrical_fields += d::fill_positive(b.vm_a_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(b.vm_b_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(b.vm_c_pu, TypicalParameters::kVoltagePu);
      report.electrical_fields += d::fill_positive(b.vmin_pu, TypicalParameters::kVoltageMinPu);
      report.electrical_fields += d::fill_positive(b.vmax_pu, TypicalParameters::kVoltageMaxPu);
    }
    for (auto& line : tp.lines) {
      if (d::missing(line.r1_pu) && d::missing(line.x1_pu)) {
        line.r1_pu = TypicalParameters::kACBranchRPu;
        line.x1_pu = TypicalParameters::kACBranchXPu;
        report.electrical_fields += 2;
      }
      report.electrical_fields += d::fill_positive_int(line.parallel, 1);
      report.operational_limit_fields += d::fill_positive(line.rate_a_mva, ac_base);
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(
            line.r0_pu, line.r1_pu * TypicalParameters::kZeroSeqMultiplier);
        report.short_circuit_fields += d::fill_positive(
            line.x0_pu, line.x1_pu * TypicalParameters::kZeroSeqMultiplier);
      }
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(line.failure_rate, TypicalParameters::kBranchFailureRatePerYear);
        report.reliability_fields += d::fill_positive(line.mttr_hr, TypicalParameters::kBranchMttrHr);
      }
    }
    for (auto& tr : tp.transformers) {
      report.operational_limit_fields += d::fill_positive(tr.sn_mva, TypicalParameters::kTransformerSnMva);
      report.electrical_fields += d::fill_positive(tr.vn_hv_kv, TypicalParameters::kACDistributionKv);
      report.electrical_fields += d::fill_positive(tr.vn_lv_kv, TypicalParameters::kLowVoltageACDistributionKv);
      report.electrical_fields += d::fill_positive(tr.vk_percent, TypicalParameters::kTransformerVkPercent);
      report.electrical_fields += d::fill_positive(tr.vkr_percent, TypicalParameters::kTransformerVkrPercent);
      report.electrical_fields += d::fill_string(tr.vector_group, TypicalParameters::kTransformerVectorGroup);
      if (options.fill_reliability) {
        report.reliability_fields += d::fill_positive(tr.mtbf_hr, TypicalParameters::kEquipmentMtbfHr);
        report.reliability_fields += d::fill_positive(tr.mttr_hr, TypicalParameters::kEquipmentMttrHr);
      }
    }
    for (auto& g : tp.generators) {
      const double rating = d::max_positive(g.mbase_mva, std::abs(g.p_mw),
                                            TypicalParameters::kGeneratorCapacityMw);
      report.operational_limit_fields += d::fill_positive(g.mbase_mva, rating);
      report.operational_limit_fields += d::fill_positive(g.pmax_mw, rating);
      report.operational_limit_fields += d::fill_positive(
          g.qmax_mvar, rating * TypicalParameters::kReactiveLimitRatio);
      if (d::missing(g.qmin_mvar)) {
        g.qmin_mvar = -g.qmax_mvar;
        report.operational_limit_fields += 1;
      }
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(g.xd_pu, TypicalParameters::kGeneratorXdPu);
        report.short_circuit_fields += d::fill_positive(g.xdpp_pu, TypicalParameters::kGeneratorXdppPu);
        report.short_circuit_fields += d::fill_positive(g.x2_pu, TypicalParameters::kGeneratorXdppPu);
        report.short_circuit_fields += d::fill_positive(g.x0_pu, TypicalParameters::kGeneratorXdppPu);
        report.short_circuit_fields += d::fill_positive(g.r0_pu, TypicalParameters::kGeneratorRaPu);
      }
    }
    for (auto& eg : tp.external_grids) {
      report.electrical_fields += d::fill_positive(eg.vm_pu, TypicalParameters::kVoltagePu);
      if (options.fill_short_circuit) {
        report.short_circuit_fields += d::fill_positive(eg.s_sc_max_mva, TypicalParameters::kExternalGridSscMaxMva);
        report.short_circuit_fields += d::fill_positive(eg.s_sc_min_mva, TypicalParameters::kExternalGridSscMinMva);
        report.short_circuit_fields += d::fill_positive(eg.rx_max, TypicalParameters::kExternalGridRX);
        report.short_circuit_fields += d::fill_positive(eg.rx_min, TypicalParameters::kExternalGridRX);
        report.short_circuit_fields += d::fill_positive(eg.x1_pu, ac_base / eg.s_sc_max_mva);
        report.short_circuit_fields += d::fill_positive(eg.r1_pu, eg.x1_pu * eg.rx_max);
        report.short_circuit_fields += d::fill_positive(eg.x2_pu, eg.x1_pu);
        report.short_circuit_fields += d::fill_positive(eg.r2_pu, eg.r1_pu);
        report.short_circuit_fields += d::fill_positive(eg.x0_pu, eg.x1_pu * TypicalParameters::kZeroSeqMultiplier);
        report.short_circuit_fields += d::fill_positive(eg.r0_pu, eg.r1_pu * TypicalParameters::kZeroSeqMultiplier);
      }
    }
  }

  return report;
}

inline HybridPowerSystem with_typical_parameters(
    HybridPowerSystem sys,
    const TypicalParameterOptions& options = {}) {
  apply_typical_parameters(sys, options);
  return sys;
}

}  // namespace hacdcpf
