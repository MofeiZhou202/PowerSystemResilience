#pragma once

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

    // ── Scaling ───────────────────────────────────────────────────────────────
    static constexpr double kCostScaleFactor = 1.0;  ///< Objective cost scale ($/h)

    // ── JSON schema ───────────────────────────────────────────────────────────
    static constexpr const char* kSchemaVersion  = "1.0";
    static constexpr const char* kPackageVersion = "0.5.0";
};

}  // namespace hacdcpf
