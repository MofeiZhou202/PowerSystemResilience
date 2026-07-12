#pragma once

/// analysis/hosting_capacity.hpp
/// =============================
/// Distributed-resource hosting-capacity assessment based on DL/T 2041-2025
/// (equipment-level path + optional engineering verification).
///
/// This replaces the earlier inline DL/T 2041-2019 "bearing capacity" endpoint.
/// The equipment-level hosting capacity of each transformer is
///
///     S_d = ( P - P_G + beta * S * cos(theta) + P_ESS + dP_ESS ) / tau_max
///
/// evaluated over the transformer's downstream (LV-side) supply area, where the
/// supply-area quantities P (load), P_G (non-DR generation), existing DR and
/// P_ESS (storage charging) are aggregated from the loaded HybridPowerSystem.
/// beta / cos(theta) / tau_max / registered-DR / dP_ESS come from the
/// transformer's cap_* fields (falling back to the global option defaults).
///
/// The optional engineering-verification stage reuses the existing power-flow,
/// short-circuit and harmonic engines.

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

// ───────────────────────────────────────────────────────────────────────────
// Options
// ───────────────────────────────────────────────────────────────────────────
struct HostingCapacityOptions {
  // Global defaults, applied where a transformer's cap_* field is left at its
  // "unset" sentinel (see assess_hosting_capacity for the exact fallback rules).
  double default_power_factor{0.95};          ///< cos(theta) fallback
  double default_dr_max_output_coeff{1.0};    ///< tau_max fallback (must be > 0)
  double single_transformer_beta{0.80};       ///< beta for a lone transformer (DL/T 2041 §7.6.1)
  double n1_loading_limit{1.0};               ///< loading limit used in beta_N-1 = (n-1)/n · limit

  // Engineering verification (DL/T 2041 §12). Off by default — planning screen only.
  bool   enable_verification{false};
  double kr{0.8};                             ///< equipment margin factor k_r (headroom P_m)
  double delta_UH_pct{7.0};                   ///< max positive voltage deviation % (GB/T 12325)
  double delta_UL_pct{7.0};                   ///< max negative voltage deviation %
  double thd_limit_pct{5.0};                  ///< THD limit % (GB/T 14549)
  bool   enable_harmonic{false};              ///< run harmonic PF in verification (best-effort)
};

HostingCapacityOptions hosting_capacity_options_from_json(const nlohmann::json& j);

// ───────────────────────────────────────────────────────────────────────────
// Result model
// ───────────────────────────────────────────────────────────────────────────
struct TransformerHostingResult {
  int         index{0};
  std::string name;
  std::string canvas_type{"transformer_2w"};
  int         canvas_index{0};
  int         hv_bus{0};
  int         lv_bus{0};
  std::string voltage_level;          ///< bucketed HV level string (e.g. "110kV")
  int         area{1};                ///< administrative area (bus.area of HV terminal)

  double sn_mva{0.0};
  double power_factor{0.0};
  double beta{0.0};                   ///< reverse load rate actually used
  bool   beta_auto{false};            ///< true if beta came from the N-1 rule
  double tau_max{0.0};

  // Supply-area aggregates (typical time)
  double supply_load_mw{0.0};         ///< P
  double supply_nondr_gen_mw{0.0};    ///< P_G
  double supply_existing_dr_mw{0.0};  ///< S_d,con
  double supply_ess_charging_mw{0.0}; ///< P_ESS
  int    supply_bus_count{0};

  // Equipment-level hosting capacity interval S_d
  double hosting_min_mw{0.0};
  double hosting_max_mw{0.0};

  // Accessible capacity
  double registered_dr_mw{0.0};       ///< S_d,reg
  double accessible_grid_min_mw{0.0}; ///< C_d1,min
  double accessible_grid_max_mw{0.0}; ///< C_d1,max
  double accessible_reg_min_mw{0.0};  ///< C_d2,min
  double accessible_reg_max_mw{0.0};  ///< C_d2,max

  std::string self_grade;             ///< grade from own accessible capacity
  std::string grade;                  ///< final grade (subordinate obeys superior area)
};

struct AreaHostingResult {
  int         area{1};
  int         transformer_count{0};

  double hosting_min_mw{0.0};         ///< Σ S_d,min
  double hosting_max_mw{0.0};         ///< Σ S_d,max
  double existing_dr_mw{0.0};
  double registered_dr_mw{0.0};

  double accessible_grid_min_mw{0.0}; ///< C_s1,min
  double accessible_grid_max_mw{0.0}; ///< C_s1,max
  double accessible_reg_min_mw{0.0};  ///< C_s2,min
  double accessible_reg_max_mw{0.0};  ///< C_s2,max

  std::string grade;
};

struct HostingViolation {
  std::string type;            ///< voltage_limit / transformer_overload / short_circuit_exceedance / harmonic_exceedance
  int         bus_id{0};
  std::string equipment;
  double      value{0.0};
  double      limit{0.0};
  std::string unit;
  std::string severity;        ///< low / medium / high / critical
  std::string message;
};

struct HostingVerification {
  bool run{false};
  bool power_flow_converged{false};
  int  pf_iterations{0};
  double pf_residual{0.0};

  bool power_flow_passed{true};
  bool short_circuit_passed{true};
  bool voltage_deviation_passed{true};
  bool harmonic_evaluated{false};
  bool harmonic_passed{true};
  double max_thd_pct{0.0};

  std::vector<HostingViolation> violations;
  std::string recommendation; ///< allow_connection / allow_with_mitigation / suspend_connection / require_further_study
};

struct HostingCapacityResult {
  std::string standard{"DL/T 2041-2025"};
  std::vector<TransformerHostingResult> transformers;
  std::vector<AreaHostingResult>        areas;
  std::vector<std::string>              warnings;
  HostingVerification                   verification;
};

// ───────────────────────────────────────────────────────────────────────────
// Entry points
// ───────────────────────────────────────────────────────────────────────────
HostingCapacityResult assess_hosting_capacity(const HybridPowerSystem& sys,
                                              const HostingCapacityOptions& opt = {});

nlohmann::json hosting_capacity_result_to_json(const HostingCapacityResult& r);

}  // namespace hacdcpf::analysis
