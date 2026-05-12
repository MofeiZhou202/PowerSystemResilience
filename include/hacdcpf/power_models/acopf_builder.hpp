#pragma once

/// AC Optimal Power Flow Builder
/// ==============================
/// Builds and solves a full AC OPF problem via the AML NonlinearExpr DAG.
///
/// Mathematical form (per-unit throughout, cost in $/h):
///
///   min   Σ_g [c2_g·(Pg_g·Sb)² + c1_g·(Pg_g·Sb) + c0_g]
///   s.t.  Σ_{g∈G(i)} Pg_g − Pd_i − Gs_i·Vi²
///              − Σ_{ℓ:f(ℓ)=i} Pf_ℓ − Σ_{ℓ:t(ℓ)=i} Pt_ℓ = 0   ∀ i  (P-balance)
///         Σ_{g∈G(i)} Qg_g − Qd_i + Bs_i·Vi²
///              − Σ_{ℓ:f(ℓ)=i} Qf_ℓ − Σ_{ℓ:t(ℓ)=i} Qt_ℓ = 0   ∀ i  (Q-balance)
///         Pf_ℓ²+Qf_ℓ² ≤ (Srate_ℓ/Sb)²                           ∀ ℓ (from-side limit)
///         Pt_ℓ²+Qt_ℓ² ≤ (Srate_ℓ/Sb)²                           ∀ ℓ (to-side   limit)
///         Pg_g^min ≤ Pg_g ≤ Pg_g^max   ∀ g
///         Qg_g^min ≤ Qg_g ≤ Qg_g^max   ∀ g
///         Vi^min  ≤ Vi   ≤ Vi^max       ∀ i
///         θ_ref = 0
///
/// Branch model (π-equivalent with off-nominal tap τ and phase shift φ):
///   y_s = g_s+jb_s = 1/(r+jx)
///   Yff = y_s/τ²   + j·bc/2     Ytt = y_s + j·bc/2
///   Yft = −y_s/(τ·e^{−jφ})      Ytf = −y_s/(τ·e^{+jφ})
///
/// Reference:
///   Zimmermann et al., "MATPOWER: Steady-State Operations, Planning and
///   Analysis Tools for Power Systems Research", IEEE TPWRS, 2011.

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::power_models {

// ════════════════════════════════════════════════════════════════════════════
// Input data structures
// ════════════════════════════════════════════════════════════════════════════

struct ACOPFBusData {
  std::string id;        ///< Bus identifier (unique string)
  double pd_pu{0.0};     ///< Active load (pu)
  double qd_pu{0.0};     ///< Reactive load (pu)
  double gs_pu{0.0};     ///< Shunt conductance (pu; P = Gs·V²)
  double bs_pu{0.0};     ///< Shunt susceptance (pu; Q = Bs·V², +ve = capacitive)
  double vm_min{0.9};    ///< Voltage magnitude lower bound (pu)
  double vm_max{1.1};    ///< Voltage magnitude upper bound (pu)
  double vm0{1.0};       ///< Warm-start voltage magnitude (pu)
  double va0_rad{0.0};   ///< Warm-start voltage angle (rad)
  bool   is_ref{false};  ///< True for the reference (slack) bus → θ = 0
};

struct ACOPFGenData {
  std::string id;           ///< Generator identifier (unique string)
  std::string bus_id;       ///< Connected bus
  double pg_min_pu{0.0};    ///< Active power lower bound (pu)
  double pg_max_pu{0.0};    ///< Active power upper bound (pu)
  double qg_min_pu{-1e6};   ///< Reactive power lower bound (pu)
  double qg_max_pu{ 1e6};   ///< Reactive power upper bound (pu)
  /// Generation cost: f(Pg_MW) = c2·Pg² + c1·Pg + c0  ($/h, Pg in MW)
  double cost_c2{0.0};      ///< $/MW²h
  double cost_c1{1.0};      ///< $/MWh
  double cost_c0{0.0};      ///< $/h
  double pg0_pu{0.0};       ///< Warm-start active power (pu)
  double qg0_pu{0.0};       ///< Warm-start reactive power (pu)
};

struct ACOPFBranchData {
  std::string id;            ///< Branch identifier (unique string)
  std::string from_bus;      ///< From-bus id
  std::string to_bus;        ///< To-bus id
  double r_pu{0.0};          ///< Series resistance (pu)
  double x_pu{1e-4};         ///< Series reactance (pu)
  double bc_pu{0.0};         ///< Total line-charging susceptance (pu)
  double tap{1.0};           ///< Off-nominal tap ratio (1 = plain line)
  double shift_deg{0.0};     ///< Phase shift (degrees)
  double rate_a_pu{0.0};     ///< Thermal limit in apparent power (pu); 0 = unconstrained
};

struct ACOPFData {
  double base_mva{100.0};
  std::vector<ACOPFBusData>    buses;
  std::vector<ACOPFGenData>    generators;
  std::vector<ACOPFBranchData> branches;
};

// ════════════════════════════════════════════════════════════════════════════
// Output
// ════════════════════════════════════════════════════════════════════════════

struct ACOPFBuilderResult {
  aml::SolveResult solve_result;

  double obj_per_h{0.0};          ///< Total generation cost ($/h)
  double max_p_viol_pu{0.0};      ///< Max |P-balance violation| (pu)
  double max_q_viol_pu{0.0};      ///< Max |Q-balance violation| (pu)
  double max_thermal_viol_pu{0.0};///< Max apparent-power thermal violation (pu)

  /// Voltage magnitudes by bus id (pu)
  std::map<std::string, double> vm_pu;
  /// Voltage angles by bus id (rad)
  std::map<std::string, double> va_rad;

  /// Generator active dispatch by gen id (MW)
  std::map<std::string, double> pg_mw;
  /// Generator reactive dispatch by gen id (MVAr)
  std::map<std::string, double> qg_mvar;

  /// From-side branch active flow by branch id (MW)
  std::map<std::string, double> pf_mw;
  /// From-side branch reactive flow by branch id (MVAr)
  std::map<std::string, double> qf_mw;
};

// ════════════════════════════════════════════════════════════════════════════
// Conversion helper: HybridPowerSystem → ACOPFData
// ════════════════════════════════════════════════════════════════════════════

/// Convert an AC-only (or AC-dominant) HybridPowerSystem to a flat ACOPFData
/// suitable for solve_acopf().  Only in-service AC buses, generators and
/// branches are included.  DC components are silently ignored.
ACOPFData to_acopf_data(const HybridPowerSystem& sys);

// ════════════════════════════════════════════════════════════════════════════
// Solver
// ════════════════════════════════════════════════════════════════════════════

/// Solve an AC OPF problem described by `data`.
/// The default solver is "NativeNLP" (penalty-Newton AD-based).
/// Pass opts.solver_name to override (e.g. "ipopt").
ACOPFBuilderResult solve_acopf(const ACOPFData& data,
                                const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
