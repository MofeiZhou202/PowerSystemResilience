#pragma once

// Short Circuit Analysis — Z-bus method (IEC 60909 / simplified sequence network)
// Equivalent to DistributionPowerFlow.jl ShortCircuit module.
// Uses Eigen complex SparseLU for the admittance matrix factorisation.
// Cross-validated against analytical Thevenin impedances on known networks.

#include <complex>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Fault type
// ---------------------------------------------------------------------------

enum class FaultType {
  ThreePhase,           ///< Balanced three-phase fault (positive sequence)
  SinglePhaseGround,    ///< Single-line-to-ground fault (uses sequence networks)
  TwoPhase,             ///< Line-to-line fault
  TwoPhaseGround,       ///< Double-line-to-ground fault
};

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

struct SCOptions {
  FaultType fault_type{FaultType::ThreePhase}; ///< Fault type to analyse
  double c_factor{1.1};           ///< IEC 60909 voltage correction factor (c_max)
  double default_xdpp{0.2};       ///< Sub-transient reactance [p.u.] used when
                                  ///  Generator::xdpp_pu == 0 or is absent
  bool   compute_all_buses{true}; ///< If true compute fault at every bus;
                                  ///  set to false and use compute_fault_at_bus()
                                  ///  for single-bus queries.
  size_t inverse_rhs_batch_size{32}; ///< Bounded RHS columns for inverse diagonal extraction.
  std::function<bool()> cancellation_requested; ///< Cooperative cancellation between RHS batches.
};

// ---------------------------------------------------------------------------
// Per-bus fault result
// ---------------------------------------------------------------------------

struct BusFaultResult {
  int bus_id;                          ///< Original bus index (ACBus::index)
  std::complex<double> z_thevenin;     ///< Thevenin impedance seen at fault bus [p.u.]
  std::complex<double> i_fault_pu;     ///< Fault current (positive sequence) [p.u.]
  double sk_mva;                       ///< Three-phase short-circuit power [MVA]
  double ikpp_ka;                      ///< Initial symmetrical short-circuit current [kA]
};

// ---------------------------------------------------------------------------
// Full result set
// ---------------------------------------------------------------------------

struct SCResult {
  std::vector<BusFaultResult> bus_results; ///< One entry per bus (if compute_all_buses)
  double base_mva{100.0};
  std::string summary() const;
};

// ---------------------------------------------------------------------------
// Extended Migration Scaffolding (single-module path, no separate v2 module)
// ---------------------------------------------------------------------------

enum class SCCalcType {
  Max,
  Min,
};

enum class SCKappaMethod {
  A,
  B,
  C,
};

enum class SCTopology {
  Auto,
  Radial,
  Meshed,
};

/// Converter SC model for IEC 60909
enum class SCConverterModel {
  GridFollowing,   ///< Current-source model: I_sc = k · I_rated (IEC 60909 §6.7)
  GridForming,     ///< Voltage-source behind impedance: V_int / (Z_filter + Z_virtual)
};

struct SCDetailedOptions {
  FaultType fault_type{FaultType::ThreePhase};
  SCCalcType calc_type{SCCalcType::Max};
  SCKappaMethod kappa_method{SCKappaMethod::C};
  SCTopology topology{SCTopology::Auto};
  double c_factor{0.0};                 ///< Optional explicit IEC voltage factor.
                                        ///  <=0 uses calc_type + nominal voltage.
  double fault_impedance_pu{0.0};
  double breaking_time_s{0.10};
  double base_frequency_hz{50.0};
  double default_xdpp{0.2};
  bool   apply_iec_transformer_correction{true}; ///< Apply IEC 60909 K_T to transformer impedance.
  bool   compute_branch_flows{true};   ///< Compute branch fault currents/power flows
  bool   compute_voltage_drops{true};  ///< Compute remaining voltage at non-fault buses
  bool   compute_nonfault_currents{true}; ///< Compute current metrics at every non-fault bus.
  bool   compute_ith{true};            ///< Compute thermal equivalent SC current
  double ith_duration_s{1.0};          ///< Duration for I_th computation (default 1 s)
  size_t inverse_rhs_batch_size{32};   ///< Bounded dense RHS columns for selected inverse data.
  std::function<bool()> cancellation_requested; ///< Cooperative cancellation between RHS batches.
};

struct SCDetailedBusResult {
  int bus_id{0};
  double ikss_ka{0.0};
  double ikss_1_ka{0.0};
  double ikss_2_ka{0.0};
  double ikss_gen_contrib_ka{0.0};
  double ikss_motor_contrib_ka{0.0};
  double ikss_load_contrib_ka{0.0};
  double ikss_sgen_contrib_ka{0.0};
  double ikss_extgrid_contrib_ka{0.0};
  double ikss_converter_contrib_ka{0.0};
  double ikss_no_motor_ka{0.0};
  double ip_ka{0.0};
  double ib_ka{0.0};
  double ik_ka{0.0};
  double ith_ka{0.0};                 ///< Thermal equivalent SC current (IEC 60909 §8)
  double thermal_m{0.0};              ///< IEC 60909 Annex A DC heat factor m
  double thermal_n{0.0};              ///< IEC 60909 Annex A AC heat factor n
  double v_remaining_pu{1.0};         ///< Remaining voltage during fault [p.u.]
  double i_phase_a_ka{0.0};           ///< Fault-current phase-A magnitude [kA]
  double i_phase_b_ka{0.0};           ///< Fault-current phase-B magnitude [kA]
  double i_phase_c_ka{0.0};           ///< Fault-current phase-C magnitude [kA]
  double i_ground_ka{0.0};            ///< Ground-return current magnitude [kA]
};

/// Per-branch fault current result
struct SCDetailedBranchResult {
  int from_bus{0};
  int to_bus{0};
  int branch_index{0};
  std::string domain{"AC"};           ///< Domain-qualified public identity.
  std::string component_kind{"ACBranch"}; ///< Authored component type.
  int component_index{0};              ///< Authored component stable .index.
  int pair_number{-1};                 ///< Transformer3W pair, otherwise -1.
  bool attribution_complete{true};     ///< False only when authored recovery is ambiguous.
  bool electrical_value_available{true}; ///< False for contracted ideal devices.
  std::string message;
  double i_branch_ka{0.0};            ///< Branch fault current magnitude [kA]
  double i_from_ka{0.0};              ///< Current at from-end [kA]
  double i_to_ka{0.0};                ///< Current at to-end [kA]
  double s_branch_mva{0.0};           ///< Apparent power flow through branch during fault [MVA]
};

struct SCNumericalQuality {
  bool factorization_valid{false};
  bool all_finite{false};
  double max_linear_residual{0.0};
};

struct SCConverterContributionResult {
  int converter_index{0};
  int bus_id{0};
  std::string name;
  std::string model;                 ///< grid_following_current_source, ac_grid_forming_voltage_source, ...
  bool ac_grid_forming{false};
  bool dc_grid_forming{false};
  double p_rated_mw{0.0};
  double i_limit_pu{0.0};
  double contribution_ka{0.0};
};

struct SCDetailedResult {
  int fault_bus_id{0};
  bool solved{false};
  std::string status{"not_run"};
  std::string message;
  std::string model_scope{"iec60909-quasi-static-sequence-v2"};
  std::vector<std::string> model_limitations;
  double effective_c_factor{0.0};
  SCNumericalQuality numerical_quality;
  std::vector<SCDetailedBusResult> bus_results;
  std::vector<SCDetailedBranchResult> branch_results;
  std::vector<SCConverterContributionResult> converter_contributions;
};

struct SCGeneratorParams {
  double xd_sub_pu{0.2};
  double x0_pu{0.2};
  double cos_phi{1.0};
};

struct SCTransformerParams {
  double x_pu{0.1};
  double x0_pu{0.1};
  double sn_mva{100.0};
};

struct SCMotorParams {
  double r_ohm{0.0};
  double x_ohm{0.0};
};

struct SCThermalFactors {
  double m{0.0};
  double n{0.0};
};

double get_voltage_factor_sc(double vn_kv, SCCalcType calc_type);
double calculate_kappa_basic_sc(double rx_ratio);
double calculate_generator_correction_factor_sc(const SCGeneratorParams& p,
                                                double vn_kv,
                                                SCCalcType calc_type);
double calculate_zero_sequence_generator_correction_factor_sc(const SCGeneratorParams& p,
                                                              double vn_kv,
                                                              SCCalcType calc_type);
double calculate_transformer_correction_factor_sc(const SCTransformerParams& p,
                                                  double vn_kv,
                                                  SCCalcType calc_type,
                                                  double base_mva);
double calculate_transformer_zero_sequence_correction_factor_sc(const SCTransformerParams& p,
                                                                double vn_kv,
                                                                SCCalcType calc_type,
                                                                double base_mva);
std::complex<double> calculate_motor_impedance_sc(const SCMotorParams& p);
SCThermalFactors calculate_thermal_factors_sc(double kappa,
                                              double ikss_ka,
                                              double ik_ka,
                                              double frequency_hz,
                                              double duration_s);

SCDetailedResult run_short_circuit_detailed(const HybridPowerSystem& sys,
                                            int fault_bus_id,
                                            const SCDetailedOptions& opt = {});

std::vector<SCDetailedResult> run_short_circuit_detailed_batch(const HybridPowerSystem& sys,
                                                               const std::vector<int>& fault_bus_ids,
                                                               const SCDetailedOptions& opt = {});

// ---------------------------------------------------------------------------
// Primary interfaces
// ---------------------------------------------------------------------------

/// Compute short-circuit quantities at every bus in the system.
SCResult compute_short_circuit(const HybridPowerSystem& sys,
                               const SCOptions& opt = {});

/// Compute short-circuit quantities at a single bus.
BusFaultResult compute_fault_at_bus(const HybridPowerSystem& sys,
                                    int bus_id,
                                    const SCOptions& opt = {});

}  // namespace hacdcpf::analysis
