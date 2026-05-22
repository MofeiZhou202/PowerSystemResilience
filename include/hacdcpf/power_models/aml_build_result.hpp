#pragma once

/// power_models/aml_build_result.hpp
/// =====================================
/// Aggregated result types for the AML-based model builders.

#include <map>
#include <string>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

/// Result from an AML power model solve (AC/DC/hybrid OPF).
struct AMLBuildResult {
  aml::SolveResult solve_result;
  double obj_per_h{0.0};

  double max_p_viol_pu{0.0};
  double max_q_viol_pu{0.0};
  double max_thermal_viol_pu{0.0};
  double max_dc_p_viol_pu{0.0};

  std::map<std::string, double> vm_pu;
  std::map<std::string, double> va_rad;
  std::map<std::string, double> pg_mw;
  std::map<std::string, double> qg_mvar;
  std::map<std::string, double> pf_mw;
  std::map<std::string, double> qf_mw;
  std::map<std::string, double> vdc_pu;
  std::map<std::string, double> pac_mw;
  std::map<std::string, double> qac_mvar;
  std::map<std::string, double> pdc_mw;
};

}  // namespace hacdcpf::power_models
