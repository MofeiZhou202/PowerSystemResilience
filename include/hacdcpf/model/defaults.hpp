#pragma once

#include <limits>

/// Centralized constants and default values
/// ==========================================
/// Single source of truth for all numerical defaults used across:
///   - JSON / Excel import
///   - MATPOWER parser
///   - Model constructors
///   - Solver assembly
///   - Tests
///
/// Usage:
///   using D = hacdcpf::Defaults;
///   double base = D::kBaseMva;           // 100.0
///   double vpu  = D::kBusVoltage;        // 1.0
///   double freq = D::kFreqHz;            // 50.0

namespace hacdcpf {

namespace detail {

constexpr double constexpr_sqrt(double value) {
    if (!(value > 0.0)) return 0.0;
    double estimate = value >= 1.0 ? value : 1.0;
    for (int iteration = 0; iteration < 64; ++iteration) {
        estimate = 0.5 * (estimate + value / estimate);
    }
    return estimate;
}

}  // namespace detail

/// Machine-representation scales shared by every numerical module.
///
/// These are not solver tolerances. Algorithms derive named thresholds from
/// them in Defaults so representation scale and model accuracy remain distinct.
struct NumericalConstants {
    static constexpr double kMachineEpsilon =
        std::numeric_limits<double>::epsilon();
    static constexpr double kSqrtMachineEpsilon =
        detail::constexpr_sqrt(kMachineEpsilon);
};

struct Defaults {
    // ── System ────────────────────────────────────────────────────────────────
    static constexpr double kBaseMva   = 100.0;  ///< System base MVA
    static constexpr double kFreqHz    =  50.0;  ///< Nominal frequency (Hz)

    // ── Voltage ───────────────────────────────────────────────────────────────
    static constexpr double kBusVoltage     = 1.0;   ///< Bus voltage magnitude (pu)
    static constexpr double kBusAngleDeg    = 0.0;   ///< Bus voltage angle (deg)
    static constexpr double kVoltageMinPu   = 0.9;   ///< AC bus lower bound (pu)
    static constexpr double kVoltageMaxPu   = 1.1;   ///< AC bus upper bound (pu)
    static constexpr double kDCVoltageMinPu = 0.9;   ///< DC bus lower bound (pu)
    static constexpr double kDCVoltageMaxPu = 1.1;   ///< DC bus upper bound (pu)

    // ── Branch π-model ────────────────────────────────────────────────────────
    static constexpr double kTapRatio       = 1.0;   ///< Transformer tap ratio (pu)
    static constexpr double kShiftDeg       = 0.0;   ///< Phase shift (deg)
    static constexpr double kRateAMva       = 0.0;   ///< Thermal rating (0 = unlimited)

    // ── Generator ─────────────────────────────────────────────────────────────
    static constexpr double kGenPmax        =  1e6;  ///< Default Pmax (MW; effectively unlimited)
    static constexpr double kGenPmin        =  0.0;  ///< Default Pmin (MW)
    static constexpr double kGenQmax        =  1e6;  ///< Default Qmax (MVAr)
    static constexpr double kGenQmin        = -1e6;  ///< Default Qmin (MVAr)
    static constexpr double kGenVgPu        =  1.0;  ///< Generator terminal voltage setpoint (pu)

    // ── Solver convergence ────────────────────────────────────────────────────
    static constexpr double kPFTol          = 1e-8;  ///< Newton PF convergence tolerance
    static constexpr int    kPFMaxIter      = 50;    ///< Newton PF max iterations
    static constexpr double kOPFTol         = 1e-8;  ///< OPF convergence tolerance
    static constexpr int    kOPFMaxIter     = 300;   ///< OPF max iterations
    static constexpr double kVSCSchurLocalRcondTol =
        NumericalConstants::kSqrtMachineEpsilon;
    static constexpr double kVSCSchurBackwardErrorTol = 1e-12;
    /// Production crossover guard from the fixed case300/ACTIVSg2000
    /// benchmark protocol. Smaller systems retain full sparse LU because the
    /// O(m) local certificate overhead dominates their factorization latency.
    static constexpr int kVSCSchurMinNetworkDimension = 1000;

    // ── Scaling ───────────────────────────────────────────────────────────────
    static constexpr double kCostScaleFactor = 1.0;  ///< Objective cost scale ($/h)

    // ── JSON schema ───────────────────────────────────────────────────────────
    static constexpr const char* kSchemaVersion  = "1.1";  // 1.1: telemetry (§10)
    static constexpr const char* kPackageVersion = "0.5.0";
};

}  // namespace hacdcpf
