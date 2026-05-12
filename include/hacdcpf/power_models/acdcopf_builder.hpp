#pragma once

/// Hybrid AC/DC Optimal Power Flow Builder
/// =========================================
/// Extends the pure-AC AML OPF builder with DC buses, DC branches, and
/// VSC converter coupling.  All formulation details are in §4 of the AML
/// design document.
///
/// Mathematical form (per-unit throughout, cost in $/h):
///
///   min   Σ_g [c2_g·(Pg_g·Sb)² + c1_g·(Pg_g·Sb) + c0_g]
///
/// AC side (same as ACOPFData):
///   P-balance[i]:  Σ_{g∈i} Pg − Pd_i − Gs_i·Vm_i²
///                    − Σ_{ℓ} Pf/Pt − Σ_k Pac_k = 0   ∀ i ∈ B_AC
///   Q-balance[i]:  Σ_{g∈i} Qg − Qd_i + Bs_i·Vm_i²
///                    − Σ_{ℓ} Qf/Qt − Σ_k Qac_k = 0   ∀ i ∈ B_AC
///   θ_ref = 0
///
/// DC side:
///   Vdc_k^min ≤ Vdc_k ≤ Vdc_k^max           ∀ k ∈ B_DC
///   P-balance[k]:  Σ_{m∈δ+(k)} (Vdc_k−Vdc_m)/r_km
///                    − Σ_{m∈δ−(k)} (Vdc_m−Vdc_k)/r_mk
///                    + Pdc_k(conv) = 0        ∀ k ∈ B_DC
///   VDC_Q converters: Vdc[dc_bus_c] = Vdc_set_c   (equality)
///
/// VSC converter (per converter c):
///   Pac_c ∈ [pmin_c, pmax_c] / Sb   (optimization variable)
///   Qac_c ∈ [qmin_c, qmax_c] / Sb
///   Power balance (quadratic loss model):
///     Pac_c + Pdc_c + a_c + b_c·Iac_c + c_c·Iac_c² = 0
///     where Iac_c = sqrt(Pac_c²+Qac_c²+ε) / Vm[ac_bus_c]
///           a_c = loss_mw/Sb,  b_c = loss_percent/100,  c_c = 1−η_c

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/power_models/acopf_builder.hpp"   // ACOPFData, ACOPFBuilderResult

namespace hacdcpf::power_models {

// ════════════════════════════════════════════════════════════════════════════
// Additional DC/converter data (AC data reuses ACOPFData structs)
// ════════════════════════════════════════════════════════════════════════════

struct ACDCOPFDCBusData {
  std::string id;
  double vdc_min{0.9};
  double vdc_max{1.1};
  double vdc0{1.0};         ///< Warm-start voltage (pu)
  double pd_pu{0.0};        ///< DC load (pu)
  bool   is_vdc_slack{false}; ///< VDC_Q mode: Vdc fixed to vdc_set
  double vdc_set{1.0};      ///< Set-point (only used when is_vdc_slack=true)
};

struct ACDCOPFDCBranchData {
  std::string id;
  std::string from_bus;     ///< DC from-bus id
  std::string to_bus;       ///< DC to-bus id
  double r_pu{0.01};        ///< DC line resistance (pu)
};

struct ACDCOPFConverterData {
  std::string id;
  std::string ac_bus_id;    ///< Connected AC bus id
  std::string dc_bus_id;    ///< Connected DC bus id
  double pac_min_pu{-1e6};  ///< AC active power lower bound (pu, + = into AC bus)
  double pac_max_pu{ 1e6};
  double qac_min_pu{-1e6};
  double qac_max_pu{ 1e6};
  double pac0_pu{0.0};      ///< Warm-start
  double qac0_pu{0.0};
  /// Loss model: ploss = a + b·Iac + c·Iac²
  ///   a = loss_mw/Sb,  b = loss_percent/100,  c = 1−η
  double a_pu{0.0};
  double b_loss{0.0};
  double c_loss{0.0};
  bool   is_vdc_slack{false}; ///< VDC_Q mode: this converter sets Vdc on its DC bus
};

struct ACDCOPFData {
  ACOPFData                         ac;          ///< AC buses, generators, branches
  std::vector<ACDCOPFDCBusData>     dc_buses;
  std::vector<ACDCOPFDCBranchData>  dc_branches;
  std::vector<ACDCOPFConverterData> converters;
};

// ════════════════════════════════════════════════════════════════════════════
// Extended result (AC fields are in the embedded ACOPFBuilderResult)
// ════════════════════════════════════════════════════════════════════════════

struct ACDCOPFBuilderResult {
  ACOPFBuilderResult ac_result;         ///< AC bus/gen/branch results + violations

  /// DC bus voltages (pu)
  std::map<std::string, double> vdc_pu;
  /// Converter AC-side dispatch (MW / MVAr)
  std::map<std::string, double> pac_mw;
  std::map<std::string, double> qac_mvar;
  /// Converter DC-side injection extracted from balance (MW, negative = into DC grid)
  std::map<std::string, double> pdc_mw;

  double max_dc_p_viol_pu{0.0};         ///< Max DC bus power-balance violation (pu)
};

// ════════════════════════════════════════════════════════════════════════════
// Builder API
// ════════════════════════════════════════════════════════════════════════════

/// Convert a HybridPowerSystem to ACDCOPFData.
/// VSC converters in PQ_MODE and VDC_Q mode are both included;
/// the optimizer is free to dispatch PQ converters within their bounds.
ACDCOPFData to_acdcopf_data(const HybridPowerSystem& sys);

/// Build and solve the hybrid AC/DC OPF via the AML NonlinearExpr DAG.
ACDCOPFBuilderResult solve_acdcopf(const ACDCOPFData& data,
                                    const aml::SolveOptions& opts);

}  // namespace hacdcpf::power_models
