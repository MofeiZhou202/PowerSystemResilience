#pragma once

/// power_models/ac_pf_model_builder.hpp
/// ========================================
/// AML-based AC OPF model builder (solver-agnostic formulation layer).
/// Replaces: power_models/acopf_builder.hpp.

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_models/aml_build_result.hpp"

namespace hacdcpf::power_models {

struct ACOPFBusData {
  std::string id;
  double pd_pu{0.0};
  double qd_pu{0.0};
  double gs_pu{0.0};
  double bs_pu{0.0};
  double vm_min{0.9};
  double vm_max{1.1};
  double vm0{1.0};
  double va0_rad{0.0};
  bool is_ref{false};
};

struct ACOPFGenData {
  std::string id;
  std::string bus_id;
  double pg_min_pu{0.0};
  double pg_max_pu{0.0};
  double qg_min_pu{-1.0};
  double qg_max_pu{1.0};
  double cost_c2{0.0};
  double cost_c1{1.0};
  double cost_c0{0.0};
  double pg0_pu{0.0};
  double qg0_pu{0.0};
};

struct ACOPFBranchData {
  std::string id;
  std::string from_bus;
  std::string to_bus;
  double r_pu{0.0};
  double x_pu{0.05};
  double bc_pu{0.0};
  double tap{1.0};
  double shift_deg{0.0};
  double rate_a_pu{0.0};
};

struct ACOPFData {
  double base_mva{100.0};
  std::vector<ACOPFBusData> buses;
  std::vector<ACOPFGenData> generators;
  std::vector<ACOPFBranchData> branches;
};

/// Backward-compatible alias.
using ACOPFBuilderResult = AMLBuildResult;

/// Build an ACOPFData from a HybridPowerSystem.
ACOPFData to_acopf_data(const HybridPowerSystem& sys);

/// Solve the AC OPF using the AML backend.
AMLBuildResult solve_acopf(const ACOPFData& data, const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
