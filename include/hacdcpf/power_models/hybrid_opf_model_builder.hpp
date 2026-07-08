#pragma once

/// power_models/hybrid_opf_model_builder.hpp
/// ============================================
/// AML-based hybrid AC/DC OPF model builder (extends AC OPF with DC buses,
/// DC branches, and VSC converters).
/// Replaces: power_models/acdcopf_builder.hpp.

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_models/ac_pf_model_builder.hpp"
#include "hacdcpf/power_models/aml_build_result.hpp"

namespace hacdcpf::power_models {

struct ACDCOPFDCBusData {
  std::string id;
  double vdc_min{0.9};
  double vdc_max{1.1};
  double vdc0{1.0};
  double pd_pu{0.0};
  bool is_vdc_slack{false};
  double vdc_set{1.0};
};

struct ACDCOPFDCBranchData {
  std::string id;
  std::string from_bus;
  std::string to_bus;
  double r_pu{0.01};
};

struct ACDCOPFConverterData {
  std::string id;
  std::string ac_bus_id;
  std::string dc_bus_id;
  double pac_min_pu{-1.0};
  double pac_max_pu{1.0};
  double qac_min_pu{-0.5};
  double qac_max_pu{0.5};
  double pac0_pu{0.0};
  double qac0_pu{0.0};
  double a_pu{0.0};    ///< Constant loss term (p.u.)
  double b_loss{0.0};  ///< Linear loss coefficient
  double c_loss{0.0};  ///< Quadratic loss coefficient
  bool is_vdc_slack{false};
};

/// DC/DC converter coupling two DC buses (multi-converter model §3.2). Modeled in
/// the OPF as a single output-power variable Pout drawn from the input bus at
/// Pout/eta; the control mode fixes Pout (Power), the output-bus voltage
/// (Voltage), or a Vdc droop (Droop).
struct ACDCOPFDCDCData {
  std::string id;
  std::string in_bus_id;
  std::string out_bus_id;
  DCDCControlMode control_mode{DCDCControlMode::Voltage};
  double eta{0.98};
  double p_ref_pu{0.0};
  double v_ref_pu{1.0};
  double k_droop{0.0};
  double pout_min_pu{-1.0};
  double pout_max_pu{1.0};
  double pout0_pu{0.0};
  bool forms_out_voltage{false};  ///< Voltage mode (or Droop w/ gain) pins Vdc_out
};

struct ACDCOPFData {
  ACOPFData ac;
  std::vector<ACDCOPFDCBusData>       dc_buses;
  std::vector<ACDCOPFDCBranchData>    dc_branches;
  std::vector<ACDCOPFConverterData>   converters;
  std::vector<ACDCOPFDCDCData>        dcdc_converters;
};

/// Hybrid AC/DC OPF result (AML model builder output).
struct ACDCOPFBuilderResult {
  AMLBuildResult ac_result;
  std::map<std::string, double> vdc_pu;
  std::map<std::string, double> pac_mw;
  std::map<std::string, double> qac_mvar;
  std::map<std::string, double> pdc_mw;
  std::map<std::string, double> pdcdc_mw;  ///< DC/DC output power (into out bus)
  double max_dc_p_viol_pu{0.0};
};

ACDCOPFData to_acdcopf_data(const HybridPowerSystem& sys);

ACDCOPFBuilderResult solve_acdcopf(const ACDCOPFData& data,
                                   const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
