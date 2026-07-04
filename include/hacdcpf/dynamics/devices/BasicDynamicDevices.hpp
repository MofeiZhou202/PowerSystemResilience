#pragma once

#include <array>
#include <complex>
#include <string>
#include <vector>

#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"
#include "hacdcpf/dynamics/devices/MachineControlLink.hpp"

namespace hacdcpf::dynamics {

enum class DCLinkMode {
  ConstantDCVoltage,
  DynamicDCVoltage
};

enum class FrequencyEstimatorKind {
  ReducedOrderPLL,
  KauraPLL,
  FixedFrequency
};

enum class DynamicLoadModelKind {
  ConstantPower,
  ConstantCurrent,
  ConstantImpedance,
  ZIP
};

enum class SynchronousMachineModelKind {
  Classical,
  OneDOneQ,
  GENROU
};

struct ACLoadDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"load"};
  std::string component_domain{"AC"};
  std::string source_type{"ac_load"};
  std::vector<DynamicModelProfile> model_profiles;
  double p_mw{0.0};
  double q_mvar{0.0};
  double nominal_voltage_pu{1.0};
  double z_weight_p{0.0};
  double i_weight_p{0.0};
  double p_weight_p{1.0};
  double z_weight_q{0.0};
  double i_weight_q{0.0};
  double p_weight_q{1.0};
  double base_mva{100.0};
  double scale{1.0};
  DynamicLoadModelKind model_kind{DynamicLoadModelKind::ConstantImpedance};
  bool in_service{true};
};

class DynamicLoad : public DynamicDevice {
 public:
  explicit DynamicLoad(ACLoadDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "ACLoad"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  ACLoadDynamicParams params_;
};

struct ThreePhaseLoadDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"asymLoad"};
  std::string component_domain{"AC"};
  std::string source_type{"three_phase_load"};
  std::vector<DynamicModelProfile> model_profiles;
  std::array<double, 3> p_mw{0.0, 0.0, 0.0};
  std::array<double, 3> q_mvar{0.0, 0.0, 0.0};
  std::array<bool, 3> phase_active{true, true, true};
  double base_mva{100.0};
  double scale{1.0};
  bool in_service{true};
};

class ThreePhaseDynamicLoad : public DynamicDevice {
 public:
  explicit ThreePhaseDynamicLoad(ThreePhaseLoadDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "ThreePhaseLoad"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  ThreePhaseLoadDynamicParams params_;
};

struct DCLoadDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"dcLoad"};
  std::string component_domain{"DC"};
  std::string source_type{"dc_load"};
  std::vector<DynamicModelProfile> model_profiles;
  double p_mw{0.0};
  double base_mva{100.0};
  double scale{1.0};
  bool in_service{true};
};

class DCDynamicLoad : public DynamicDevice {
 public:
  explicit DCDynamicLoad(DCLoadDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "DCLoad"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  DCLoadDynamicParams params_;
};

struct DCVoltageSourceDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"dc"};
  std::string component_domain{"DC"};
  std::string source_type{"dc_voltage_source"};
  double v_ref_pu{1.0};
  double conductance_pu{1e4};
  bool trip_on_vsc_event{false};
  bool in_service{true};
};

class DCVoltageSourceDynamic : public DynamicDevice {
 public:
  explicit DCVoltageSourceDynamic(DCVoltageSourceDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "DCVoltageSource"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  DCVoltageSourceDynamicParams params_;
};

struct VoltageSourceDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string device_type{"VoltageSource"};
  std::string canvas_type{"gen"};
  std::string component_domain{"AC"};
  std::string source_type{"voltage_source"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double vm_set_pu{1.0};
  double angle_set_rad{0.0};
  double frequency_hz{50.0};
  double r_pu{0.0};
  double x_pu{0.10};
  double p_mech_mw{0.0};
  double q_elec_mvar{0.0};
  double inertia_h{0.0};
  double damping_d{1.0};
  double droop_r{0.05};
  double xd_pu{0.0};
  double xq_pu{0.0};
  double xdp_pu{0.0};
  double xqp_pu{0.0};
  double xdpp_pu{0.0};
  double xl_pu{0.0};
  double td0p_s{0.0};
  double td0pp_s{0.0};
  double tq0p_s{0.0};
  double tq0pp_s{0.0};
  double saturation_a{0.0};
  double saturation_b{0.0};
  SynchronousMachineModelKind machine_model{SynchronousMachineModelKind::Classical};
  std::string machine_model_name{"ClassicalMachine"};
  bool psd_genrou_model{false};
  bool dynamic_angle{false};
  bool in_service{true};
};

class SynchronousMachine : public DynamicDevice {
 public:
  explicit SynchronousMachine(VoltageSourceDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "IEEE"; }
  [[nodiscard]] std::string modelName() const override;
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

  // Control coupling: a governor/exciter attaches to this machine and drives the
  // derivative of the machine-owned mechanical-power / field state. The link's
  // range pointer resolves to the final state offset after assignStateIndices().
  [[nodiscard]] const MachineControlLink* controlLink() const { return &link_; }
  void markGovernorAttached() { governor_attached_ = true; }
  void markExciterAttached() { exciter_attached_ = true; }

 private:
  VoltageSourceDynamicParams params_;
  StateIndexRange range_;
  MachineControlLink link_;
  bool governor_attached_{false};
  bool exciter_attached_{false};
};

enum class GovernorModel {
  TGOV1,
  IEEEG1
};

struct GovernorDynamicParams {
  int component_index{0};
  int machine_index{0};  // component index of the parent synchronous machine
  std::string label;
  std::string device_type{"Governor"};
  std::string canvas_type{"governor"};
  std::string component_domain{"AC"};
  std::string source_type{"governor"};
  std::string model_standard{"IEEE"};
  std::string model_name{"TGOV1"};
  std::string parameter_set;
  GovernorModel model{GovernorModel::TGOV1};
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double droop_r{0.05};
  double t_s{0.50};       // T1: governor/valve time constant (s)
  double turbine_t_s{0.50};  // T3: turbine time constant (s)
  double reheat_t_s{6.0};    // IEEEG1 reheat time constant (s)
  double reheat_k{0.30};     // IEEEG1 HP fraction (0..1)
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  bool in_service{true};
};

class Governor : public DynamicDevice {
 public:
  explicit Governor(GovernorDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

  // Attach to the parent machine so this governor drives the machine's mechanical
  // power state. When unattached the governor keeps its legacy standalone
  // behaviour (a single p_ref tracking state) for backward compatibility.
  void attachMachine(const MachineControlLink* link) { machine_ = link; }

 private:
  GovernorDynamicParams params_;
  StateIndexRange range_;
  const MachineControlLink* machine_{nullptr};
};

enum class ExciterModel {
  SEXS,
  IEEET1
};

struct ExciterDynamicParams {
  int component_index{0};
  int machine_index{0};  // component index of the parent synchronous machine
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string device_type{"Exciter"};
  std::string canvas_type{"exciter"};
  std::string component_domain{"AC"};
  std::string source_type{"exciter"};
  std::string model_standard{"IEEE4215"};
  std::string model_name{"SEXS"};
  std::string parameter_set;
  ExciterModel model{ExciterModel::SEXS};
  double v_ref_pu{1.0};
  double ka{20.0};
  double ta_s{0.05};
  double te_s{0.40};       // IEEET1 exciter time constant (s)
  double efd_min_pu{0.0};
  double efd_max_pu{5.0};
  bool in_service{true};
};

class Exciter : public DynamicDevice {
 public:
  explicit Exciter(ExciterDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

  // Attach to the parent machine so this exciter drives the machine's field
  // state (GENROU vf / classical internal EMF). setPSS supplies an optional
  // stabilizing signal that is summed into the voltage error.
  void attachMachine(const MachineControlLink* link) { machine_ = link; }
  void setPSS(const PSSOutputLink* pss) { pss_ = pss; }

 private:
  [[nodiscard]] double terminalVoltage(const NetworkState& y) const;
  void captureReference(const DynamicState& x, const NetworkState& y);

  ExciterDynamicParams params_;
  StateIndexRange range_;
  const MachineControlLink* machine_{nullptr};
  const PSSOutputLink* pss_{nullptr};
  double v_ref_captured_{1.0};
  double field0_{1.0};
  bool captured_{false};
};

// ── Power System Stabilizer (single-input speed PSS, IEEE PSS1A subset) ──
struct PSSDynamicParams {
  int component_index{0};
  int machine_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string device_type{"PSS"};
  std::string canvas_type{"pss"};
  std::string component_domain{"AC"};
  std::string source_type{"pss"};
  std::string model_standard{"IEEE"};
  std::string model_name{"PSS1A"};
  std::string parameter_set;
  double ks{5.0};        // stabilizer gain
  double tw_s{10.0};     // washout time constant (s)
  double t1_s{0.15};     // lead-lag 1 numerator (s)
  double t2_s{0.03};     // lead-lag 1 denominator (s)
  double t3_s{0.15};     // lead-lag 2 numerator (s)
  double t4_s{0.03};     // lead-lag 2 denominator (s)
  double vs_max_pu{0.10};
  double vs_min_pu{-0.10};
  bool in_service{true};
};

class PowerSystemStabilizer : public DynamicDevice {
 public:
  explicit PowerSystemStabilizer(PSSDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

  void attachMachine(const MachineControlLink* link);
  [[nodiscard]] const PSSOutputLink* pssLink() const { return &link_; }

 private:
  PSSDynamicParams params_;
  StateIndexRange range_;
  const MachineControlLink* machine_{nullptr};
  PSSOutputLink link_;
};

struct GridFormingInverterParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  int dc_bus_pos{-1};
  std::string label;
  std::string device_type{"GridFormingInverter"};
  std::string canvas_type{"vsc"};
  std::string component_domain{"AC"};
  std::string source_type{"grid_forming_inverter"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double v_ref_pu{1.0};
  double angle_ref_rad{0.0};
  double frequency_hz{50.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.01};
  double q_droop_pu{0.05};
  double power_filter_t_s{0.05};
  double voltage_control_t_s{0.02};
  double voltage_kp{0.1};
  double voltage_ki{10.0};
  double overload_kp{0.1};
  double overload_ki{10.0};
  double current_limit_pu{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double vmax_internal_pu{1.30};
  double vmin_internal_pu{0.20};
  double eta{0.98};
  double dc_link_capacitance_s{0.10};
  double dc_link_conductance_pu{1e4};
  double vdc_ref_pu{1.0};
  double vdc_min_pu{0.20};
  double vdc_max_pu{2.00};
  DCLinkMode dc_link_mode{DCLinkMode::ConstantDCVoltage};
  bool in_service{true};
};

class GridFormingInverter : public DynamicDevice {
 public:
  explicit GridFormingInverter(GridFormingInverterParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override { return "GridFormingNortonDroop"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  GridFormingInverterParams params_;
  StateIndexRange range_;
};

struct GridFollowingInverterParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  int dc_bus_pos{-1};
  std::string label;
  std::string device_type{"GridFollowingInverter"};
  std::string canvas_type{"vsc"};
  std::string component_domain{"AC"};
  std::string source_type{"grid_following_inverter"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double response_t_s{0.02};
  FrequencyEstimatorKind frequency_estimator{FrequencyEstimatorKind::ReducedOrderPLL};
  std::string frequency_estimator_name{"ReducedOrderPLL"};
  double pll_kp{0.01};
  double pll_ki{1.0};
  double pll_lpf_t_s{0.005};
  double power_filter_t_s{0.02};
  double v_min_current_pu{0.20};
  double current_limit_pu{0.0};
  double stabilizing_admittance_pu{0.0};
  double frequency_watt_droop_pu{0.0};
  double volt_var_droop_pu{0.0};
  double v_ref_pu{1.0};
  double f_ref_hz{50.0};
  double eta{0.98};
  double dc_link_capacitance_s{0.10};
  double dc_link_conductance_pu{1e4};
  double vdc_ref_pu{1.0};
  double vdc_min_pu{0.20};
  double vdc_max_pu{2.00};
  DCLinkMode dc_link_mode{DCLinkMode::ConstantDCVoltage};
  bool stamp_dc_power{false};
  bool reactive_current_priority{false};
  bool in_service{true};
};

class GridFollowingInverter : public DynamicDevice {
 public:
  explicit GridFollowingInverter(GridFollowingInverterParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override { return "REGC_REEC_GFL_Subset"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  GridFollowingInverterParams params_;
  StateIndexRange range_;
};

struct VSCConverterDynamicParams : public GridFollowingInverterParams {
  bool grid_forming{false};
  double angle_ref_rad{0.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.01};
  double q_droop_pu{0.05};
  double voltage_control_t_s{0.02};
  double voltage_kp{0.1};
  double voltage_ki{10.0};
  double overload_kp{0.1};
  double overload_ki{10.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double vmax_internal_pu{1.30};
  double vmin_internal_pu{0.20};
};

class VSCConverterDynamic : public DynamicDevice {
 public:
  explicit VSCConverterDynamic(VSCConverterDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override {
    return params_.grid_forming ? "VSCGridForming" : "VSCGridFollowing";
  }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override {
    return params_.grid_forming ? "GridFormingNortonDroop" : "REGC_REEC_GFL_Subset";
  }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override {
    return params_.grid_forming ? gfm_.modelProfiles() : gfl_.modelProfiles();
  }

 private:
  VSCConverterDynamicParams params_;
  GridFormingInverter gfm_;
  GridFollowingInverter gfl_;
};

struct DCDCConverterDynamicParams {
  int component_index{0};
  int bus_in{0};
  int bus_out{0};
  int bus_in_pos{-1};
  int bus_out_pos{-1};
  std::string label;
  std::string canvas_type{"dcdcConverter"};
  std::string component_domain{"DC"};
  std::string source_type{"dcdc_converter"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double eta{0.98};
  double response_t_s{0.02};
  bool in_service{true};
};

class DCDCConverterDynamic : public DynamicDevice {
 public:
  explicit DCDCConverterDynamic(DCDCConverterDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "DCDCConverter"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "HACDCPF"; }
  [[nodiscard]] std::string modelName() const override { return "FirstOrderDCDCConverter"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  DCDCConverterDynamicParams params_;
  StateIndexRange range_;
};

struct BatteryDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  bool is_ac{true};
  std::string label;
  std::string canvas_type{"storage"};
  std::string component_domain{"AC"};
  std::string source_type{"storage"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double e_rated_mwh{0.0};
  double soc_init{0.5};
  double soc_min{0.0};
  double soc_max{1.0};
  double eta_charge{0.95};
  double eta_discharge{0.95};
  double self_discharge_pct_per_h{0.0};
  double response_t_s{0.05};
  bool stamp_power{true};
  bool in_service{true};
};

class BatteryDynamic : public DynamicDevice {
 public:
  explicit BatteryDynamic(BatteryDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.is_ac ? "ACStorage" : "DCStorage"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "IEEE1547"; }
  [[nodiscard]] std::string modelName() const override { return "BatterySOCFirstOrder"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  BatteryDynamicParams params_;
  StateIndexRange range_;
};

struct PVDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"pv"};
  std::string component_domain{"AC"};
  std::string source_type{"pv_system"};
  std::string model_standard{"IEEE1547"};
  std::string model_name{"PVCurrentSourceFirstOrder"};
  std::string parameter_set;
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double irradiance_pu{1.0};
  double response_t_s{0.05};
  double current_limit_pu{0.0};
  bool in_service{true};
};

class PVDynamic : public DynamicDevice {
 public:
  explicit PVDynamic(PVDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "PVDynamic"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  PVDynamicParams params_;
  StateIndexRange range_;
};

struct ProtectionRelayParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"relay"};
  std::string component_domain{"AC"};
  std::string source_type{"protection_relay"};
  std::string model_standard{"IEEE"};
  std::string model_name{"VoltageFrequencyRelay"};
  std::string parameter_set;
  double undervoltage_pickup_pu{0.70};
  double overvoltage_pickup_pu{1.20};
  double underfrequency_hz{47.0};
  double overfrequency_hz{53.0};
  double trip_delay_s{0.16};
  bool tripped{false};
  bool in_service{true};
};

class ProtectionRelay : public DynamicDevice {
 public:
  explicit ProtectionRelay(ProtectionRelayParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x,
                                NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "ProtectionRelay"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  ProtectionRelayParams params_;
  StateIndexRange range_;
};

}  // namespace hacdcpf::dynamics
