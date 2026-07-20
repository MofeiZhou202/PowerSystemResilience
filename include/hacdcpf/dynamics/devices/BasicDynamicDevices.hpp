#pragma once

#include <array>
#include <complex>
#include <string>
#include <vector>

#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"
#include "hacdcpf/dynamics/devices/IEEE1547Protection.hpp"
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

enum class GridFormingControlKind {
  Droop,
  VirtualInertia,
  VirtualOscillator
};

enum class InverterFilterKind {
  RL,
  LCL
};

enum class CurrentLimiterKind {
  Magnitude,
  ActivePriority,
  ReactivePriority,
  Instantaneous,
  Saturation,
  Hybrid
};

enum class SynchronousMachineModelKind {
  Classical,
  OneDOneQ,
  SimpleAF,
  AndersonFouad,
  SimpleMarconato,
  Marconato,
  SauerPai,
  GENROU,
  GENROE,
  GENSAL,
  GENSAE
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
  double phase_power_scale{1.0 / 3.0};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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

struct DynamicRLLineParams {
  int component_index{0};
  int from_bus{0};
  int to_bus{0};
  int from_pos{-1};
  int to_pos{-1};
  std::string label;
  std::string canvas_type{"branch"};
  std::string component_domain{"AC"};
  std::string source_type{"dynamic_rl_line"};
  std::string model_standard{"PowerSimulationsDynamics"};
  std::string model_name{"DynamicRLLine"};
  std::string parameter_set;
  double base_mva{100.0};
  double frequency_hz{50.0};
  double r_pu{0.0};
  double x_pu{0.0};
  bool in_service{true};
};

class DynamicRLLine : public DynamicDevice {
 public:
  explicit DynamicRLLine(DynamicRLLineParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "DynamicRLLine"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return params_.model_standard; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::string parameterSet() const override { return params_.parameter_set; }
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  [[nodiscard]] std::complex<double> branchCurrent(const DynamicState& x) const;
  [[nodiscard]] std::complex<double> voltageDrop(const NetworkState& y) const;
  [[nodiscard]] std::complex<double> equilibriumCurrent(const NetworkState& y) const;

  DynamicRLLineParams params_;
  StateIndexRange range_;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  double phase_power_scale{1.0 / 3.0};
  double inertia_h{0.0};
  double damping_d{1.0};
  double droop_r{0.05};
  double xd_pu{0.0};
  double xq_pu{0.0};
  double xdp_pu{0.0};
  double xqp_pu{0.0};
  double xdpp_pu{0.0};
  double xqpp_pu{0.0};
  double xl_pu{0.0};
  // Sequence-network interface (design doc §8.8). When x2_pu/x0_pu are set the
  // machine presents a sequence-coupled Norton admittance to the unbalanced
  // network (negative/zero-sequence impedances) and, if r2_pu is set, feels the
  // negative-sequence braking torque. All default to 0, which reproduces the
  // balanced positive-sequence-only behavior.
  double x2_pu{0.0};
  double r2_pu{0.0};
  double x0_pu{0.0};
  double r0_pu{0.0};
  double td0p_s{0.0};
  double td0pp_s{0.0};
  double tq0p_s{0.0};
  double tq0pp_s{0.0};
  double t_aa_s{0.0};
  double saturation_a{0.0};
  double saturation_b{0.0};
  SynchronousMachineModelKind machine_model{SynchronousMachineModelKind::Classical};
  std::string machine_model_name{"ClassicalMachine"};
  bool psd_genrou_model{false};
  bool dynamic_angle{false};
  // IEEE 1547 ride-through / trip / reconnect protection (design doc §11.7) for a
  // synchronous DER. Disabled by default; opt-in per device.
  IEEE1547Settings protection{};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  bool updateProtection(double t,
                        double dt,
                        DynamicState& x,
                        NetworkState& y,
                        std::vector<DynamicEvent>& events) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "IEEE"; }
  [[nodiscard]] std::string modelName() const override;
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;
  [[nodiscard]] FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const override;

  // Control coupling: a governor/exciter attaches to this machine and drives the
  // derivative of the machine-owned mechanical-power / field state. The link's
  // range pointer resolves to the final state offset after assignStateIndices().
  [[nodiscard]] const MachineControlLink* controlLink() const { return &link_; }
  void markGovernorAttached() {
    governor_attached_ = true;
    link_.inner_vars.has_turbine_governor = true;
  }
  void markExciterAttached() {
    exciter_attached_ = true;
    link_.inner_vars.has_avr = true;
  }
  void markPssAttached() { link_.inner_vars.has_pss = true; }

 private:
  VoltageSourceDynamicParams params_;
  StateIndexRange range_;
  MachineControlLink link_;
  bool governor_attached_{false};
  bool exciter_attached_{false};
  IEEE1547RuntimeState protection_state_;
};

struct FiveMassShaftParams {
  int component_index{0};
  int machine_index{0};
  std::string label;
  std::string device_type{"Shaft"};
  std::string canvas_type{"shaft"};
  std::string component_domain{"AC"};
  std::string source_type{"shaft"};
  std::string model_standard{"PowerSimulationsDynamics"};
  std::string model_name{"FiveMassShaft"};
  std::string parameter_set;
  double base_mva{100.0};
  double frequency_hz{50.0};
  std::array<double, 5> inertia_h{{0.30, 0.30, 0.30, 0.30, 1.80}};
  std::array<double, 4> stiffness_pu{{25.0, 25.0, 25.0, 25.0}};
  std::array<double, 4> damping_pu{{0.02, 0.02, 0.02, 0.02}};
  bool in_service{true};
};

class FiveMassShaft : public DynamicDevice {
 public:
  explicit FiveMassShaft(FiveMassShaftParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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

  void attachMachine(const MachineControlLink* link) { machine_ = link; }

 private:
  void setEquilibrium(DynamicState& x) const;
  [[nodiscard]] double mechanicalTorque(const DynamicState& x) const;
  [[nodiscard]] double recoveredElectricalTorque(const DynamicState& x,
                                                 Eigen::Ref<const Eigen::VectorXd> dxdt) const;

  FiveMassShaftParams params_;
  StateIndexRange range_;
  const MachineControlLink* machine_{nullptr};
};

enum class GovernorModel {
  TGOV1,
  IEEEG1,
  TGTypeI,
  TGTypeII,
  GAST,
  HYGOV,
  DEGOV,
  DEGOV1,
  PIDGOV,
  WPIDHY,
  TGSimple
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
  double t2_s{0.0};       // TGOV1 lead-lag numerator T2 (s)
  double turbine_t_s{0.50};  // T3: turbine time constant (s)
  double damping_d_t{0.0};    // TGOV1 turbine damping D_T
  double reheat_t_s{6.0};    // IEEEG1 reheat time constant (s)
  double reheat_k{0.30};     // IEEEG1 HP fraction (0..1)
  double tc_s{0.50};         // TGTypeI servo time constant Tc (s)
  double t3_s{0.10};         // TGTypeI transient gain time constant T3 (s)
  double t4_s{0.30};         // TGTypeI power fraction time constant T4 (s)
  double t5_s{5.00};         // TGTypeI reheat time constant T5 (s)
  double ta_s{0.10};         // Diesel/hydro/PID actuator derivative time constant
  double tb_s{0.30};         // Diesel/hydro/PID actuator denominator time constant
  double ki{1.0};            // PID/integral governor gain
  double kd{0.0};            // PID/derivative governor gain
  double fuel_t_s{0.40};     // GAST/DEGOV fuel or actuator time constant
  double temperature_t_s{3.0};  // GAST exhaust/load-limit temperature lag
  double water_t_s{1.0};     // HYGOV/PID hydro water inertia time constant
  double gate_t_s{0.25};     // Hydro gate servo time constant
  double gate_max_pu{1.0};
  double gate_min_pu{0.0};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  IEEET1,
  AVRSimple,
  AVRTypeI,
  AVRTypeII,
  ESAC1A,
  EXAC1,
  EXST1,
  SCRX,
  ESST1A,
  ST6B,
  ST8C
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
  double ta_over_tb{1.0};   // SEXS lead-lag ratio Ta/Tb
  double tb_s{0.05};        // SEXS lead-lag denominator Tb (s)
  double te_s{0.40};       // IEEET1 exciter time constant (s)
  double kv{20.0};         // AVRSimple integrator gain
  double ke{1.0};          // AVRTypeI exciter gain denominator
  double kf{0.0};          // AVRTypeI stabilizing feedback gain
  double tf_s{1.0};        // AVRTypeI feedback time constant (s)
  double tr_s{0.01};       // AVRTypeI/II measurement time constant (s)
  double ae{0.0};          // AVR saturation coefficient
  double be{0.0};          // AVR saturation exponent coefficient
  double k0{20.0};         // AVRTypeII regulator gain
  double t1_s{0.05};       // AVRTypeII lead-lag numerator T1
  double t2_s{0.01};       // AVRTypeII lead-lag denominator T2
  double t3_s{0.05};       // AVRTypeII lead-lag numerator T3
  double t4_s{0.01};       // AVRTypeII lead-lag denominator T4
  double tc_s{0.05};       // AC/ST lead-lag numerator
  double tb1_s{0.05};      // ST8C second lead-lag denominator
  double tc1_s{0.05};      // ST8C second lead-lag numerator
  double kg{1.0};          // Exciter/current-compounding gain
  double kc{0.0};          // Rectifier/current-compounding coefficient
  double kd{0.0};          // Demagnetizing coefficient
  double kp{1.0};          // PI proportional gain
  double ki{0.0};          // PI integral gain
  double va_min_pu{-5.0};
  double va_max_pu{5.0};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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

// ── Power System Stabilizer (single-input speed PSS variants) ──
enum class PSSModel {
  PSS1A,
  IEEEST,
  STAB1,
  PSS2A,
  PSS2B,
  PSS2C
};

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
  PSSModel model{PSSModel::PSS1A};
  double ks{5.0};        // stabilizer gain
  double tw_s{10.0};     // washout time constant (s)
  double t1_s{0.15};     // lead-lag 1 numerator (s)
  double t2_s{0.03};     // lead-lag 1 denominator (s)
  double t3_s{0.15};     // lead-lag 2 numerator (s)
  double t4_s{0.03};     // lead-lag 2 denominator (s)
  double vs_max_pu{0.10};
  double vs_min_pu{-0.10};
  double a1{0.0};
  double a2{1.0};
  double a3{1.0};
  double a4{1.0};
  double a5{1.0};
  double a6{0.0};
  double t5_s{0.10};
  double t6_s{0.05};
  double vcu{0.0};
  double vcl{0.0};
  int input_code{1};
  double kt{5.0};
  double stab_t_s{10.0};
  double t1_over_t3{1.0};
  double t2_over_t4{1.0};
  double h_lim{0.10};
  double ks1{10.0};
  double ks2{1.0};
  double ks3{1.0};
  double m_rtf{5.0};
  double n_rtf{1.0};
  double tw1_s{2.0};
  double tw2_s{2.0};
  double tw3_s{2.0};
  double tw4_s{0.0};
  double t7_s{2.0};
  double t8_s{0.2};
  double t9_s{0.1};
  double t10_s{0.0};
  double t11_s{0.0};
  double t12_s{0.0};
  double t13_s{0.0};
  double vs1_max_pu{0.10};
  double vs1_min_pu{-0.10};
  double vs2_max_pu{0.10};
  double vs2_min_pu{-0.10};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  std::string model_name{"GridFormingNortonDroop"};
  std::vector<DynamicModelProfile> model_profiles;
  GridFormingControlKind control_kind{GridFormingControlKind::Droop};
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
  double reactive_power_filter_t_s{0.05};
  double voltage_control_t_s{0.02};
  double voltage_kp{0.1};
  double voltage_ki{10.0};
  double overload_kp{0.1};
  double overload_ki{10.0};
  double current_limit_pu{0.0};
  CurrentLimiterKind limiter_kind{CurrentLimiterKind::Magnitude};
  InverterFilterKind filter_kind{InverterFilterKind::RL};
  double filter_c_pu{0.0};
  double filter_grid_r_pu{0.0};
  double filter_grid_x_pu{0.05};
  bool reactive_current_priority{false};
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
  double vsm_ta_s{2.0};
  double vsm_damping_kd{0.0};
  double vsm_frequency_droop_kw{20.0};
  double voc_k1{0.0033};
  double voc_psi_rad{0.7853981633974483};
  double voc_k2{0.0796};
  bool reference_frame_locked{false};
  // Phase-domain terminal model for explicitly unbalanced networks. The GFM
  // retains one common frequency-forming oscillator, while its Norton port,
  // power feedback, current limit, and telemetry are evaluated per phase.
  bool phase_domain_control{false};
  bool allow_zero_sequence_current{false};
  DCLinkMode dc_link_mode{DCLinkMode::ConstantDCVoltage};
  // IEEE 1547 ride-through / trip / reconnect protection (design doc §11.7).
  // Disabled by default; opt-in per device. A grid-forming DER re-energizes on
  // reconnect (no soft-start current ramp — that is a grid-following behavior).
  IEEE1547Settings protection{};
  // IEEE 1547 volt-var / frequency-watt smart-inverter functions (design doc
  // §11.7): the filtered references adjust the reactive/active setpoints.
  VoltVarSettings volt_var{};
  FreqWattSettings freq_watt{};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  bool updateProtection(double t,
                        double dt,
                        DynamicState& x,
                        NetworkState& y,
                        std::vector<DynamicEvent>& events) override;
  void updateSmartControls(double dt, const NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override { return params_.model_name; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;
  [[nodiscard]] FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const override;

 private:
  GridFormingInverterParams params_;
  StateIndexRange range_;
  InverterInnerVariableBus inner_vars_;
  IEEE1547RuntimeState protection_state_;
  SmartInverterState smart_state_;
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
  CurrentLimiterKind limiter_kind{CurrentLimiterKind::Magnitude};
  InverterFilterKind filter_kind{InverterFilterKind::RL};
  double filter_c_pu{0.0};
  double filter_grid_r_pu{0.0};
  double filter_grid_x_pu{0.05};
  double stabilizing_admittance_pu{0.0};
  double frequency_watt_droop_pu{0.0};
  double volt_var_droop_pu{0.0};
  // IEEE 1547 smart-inverter functions (design doc §11.7): piecewise-linear
  // volt-var and frequency-watt with deadbands. When enabled they supersede the
  // simple linear droop above. Disabled by default (opt-in).
  VoltVarSettings volt_var{};
  FreqWattSettings freq_watt{};
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
  // Phase-domain current control for explicitly unbalanced networks. Six
  // differential states represent the real/imaginary current of phases a/b/c;
  // each phase follows its own terminal voltage and current limit while the
  // positive-sequence PLL supplies the common synchronization angle.
  bool phase_domain_control{false};
  bool allow_zero_sequence_current{false};
  // IEEE 1547 ride-through / trip / reconnect protection (design doc §11.7).
  // Disabled by default; opt-in per device.
  IEEE1547Settings protection{};
  // Full-fidelity GFL chain (PowerSimulationsDynamics.jl-compatible):
  // OuterControl ActivePowerPI/ReactivePowerPI + CurrentModeControl inner PI
  // + differential LCL filter.  Opt-in; when false the compact first-order
  // REGC/REEC response is used.  References follow the PSD wiring: the P-loop
  // drives Iq_ref, the Q-loop drives Id_ref.
  bool full_fidelity{false};
  double outer_kp_p{2.0};        // Kp_p
  double outer_ki_p{30.0};       // Ki_p
  double outer_omega_z{41.468};  // ωz  = 0.132·2π·50 (p measurement low-pass)
  double outer_kp_q{2.0};        // Kp_q
  double outer_ki_q{30.0};       // Ki_q
  double outer_omega_f{41.468};  // ωf  = 0.132·2π·50 (q measurement low-pass)
  double inner_kpc{0.37};        // kpc current-loop proportional gain
  double inner_kic{0.7};         // kic current-loop integral gain
  double inner_kffv{1.0};        // kffv voltage feedforward
  double lcl_lf_pu{0.009};       // converter-side inductance
  double lcl_rf_pu{0.016};       // converter-side resistance
  double lcl_cf_pu{2.5};         // shunt capacitance
  double lcl_lg_pu{0.002};       // grid-side inductance
  double lcl_rg_pu{0.003};       // grid-side resistance
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  bool updateProtection(double t,
                        double dt,
                        DynamicState& x,
                        NetworkState& y,
                        std::vector<DynamicEvent>& events) override;
  void updateSmartControls(double dt, const NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override { return "REGC_REEC_GFL_Subset"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;
  [[nodiscard]] FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const override;

 private:
  void computeDerivativesFullFidelity(const DynamicState& x,
                                      const NetworkState& y,
                                      Eigen::Ref<Eigen::VectorXd> dxdt) const;
  void seedFullFidelityEquilibrium(DynamicState& x,
                                   const NetworkState& y,
                                   bool set_reference);

  GridFollowingInverterParams params_;
  StateIndexRange range_;
  InverterInnerVariableBus inner_vars_;
  IEEE1547RuntimeState protection_state_;
  SmartInverterState smart_state_;
};

struct VSCConverterDynamicParams : public GridFollowingInverterParams {
  bool grid_forming{false};
  std::string model_name{"GridFormingNortonDroop"};
  GridFormingControlKind control_kind{GridFormingControlKind::Droop};
  double angle_ref_rad{0.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.01};
  double q_droop_pu{0.05};
  double reactive_power_filter_t_s{0.05};
  double voltage_control_t_s{0.02};
  double voltage_kp{0.1};
  double voltage_ki{10.0};
  double overload_kp{0.1};
  double overload_ki{10.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double vmax_internal_pu{1.30};
  double vmin_internal_pu{0.20};
  double vsm_ta_s{2.0};
  double vsm_damping_kd{0.0};
  double vsm_frequency_droop_kw{20.0};
  double voc_k1{0.0033};
  double voc_psi_rad{0.7853981633974483};
  double voc_k2{0.0796};
  bool reference_frame_locked{false};
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  bool updateProtection(double t,
                        double dt,
                        DynamicState& x,
                        NetworkState& y,
                        std::vector<DynamicEvent>& events) override;
  void updateSmartControls(double dt, const NetworkState& y) override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;
  [[nodiscard]] FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override {
    return params_.grid_forming ? "VSCGridForming" : "VSCGridFollowing";
  }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "NERC"; }
  [[nodiscard]] std::string modelName() const override {
    return params_.grid_forming ? gfm_.modelName() : gfl_.modelName();
  }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override {
    return params_.grid_forming ? gfm_.modelProfiles() : gfl_.modelProfiles();
  }

 private:
  VSCConverterDynamicParams params_;
  GridFormingInverter gfm_;
  GridFollowingInverter gfl_;
};

struct PeriodicVariableSourceDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"extGrid"};
  std::string component_domain{"AC"};
  std::string source_type{"periodic_variable_source"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double r_th_pu{0.0};
  double x_th_pu{0.05};
  double voltage_bias_pu{1.0};
  double voltage_frequency_rad_s{0.0};
  double voltage_sin_coeff_pu{0.0};
  double voltage_cos_coeff_pu{0.0};
  double angle_bias_rad{0.0};
  double angle_frequency_rad_s{0.0};
  double angle_sin_coeff_rad{0.0};
  double angle_cos_coeff_rad{0.0};
  double initial_voltage_pu{1.0};
  double initial_angle_rad{0.0};
  bool in_service{true};
};

class PeriodicVariableSourceDynamic : public DynamicDevice {
 public:
  explicit PeriodicVariableSourceDynamic(PeriodicVariableSourceDynamicParams params);

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
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "PeriodicVariableSource"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PowerSimulationsDynamics"; }
  [[nodiscard]] std::string modelName() const override { return "PeriodicVariableSource"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  PeriodicVariableSourceDynamicParams params_;
  StateIndexRange range_;
};

struct CSVGN1DynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"sgen"};
  std::string component_domain{"AC"};
  std::string source_type{"csvgn1"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double q_ref_mvar{0.0};
  double K{20.0};
  double T1{0.0};
  double T2{1.0};
  double T3{0.154833};
  double T4{1.0};
  double T5{0.005167};
  double Rmin{0.0};
  double Vmax{1.0};
  double Vmin{0.0};
  double CBase{60.0};
  double model_base_mva{500.0};
  double v_ref_pu{0.0};
  bool in_service{true};
};

class CSVGN1Dynamic : public DynamicDevice {
 public:
  explicit CSVGN1Dynamic(CSVGN1DynamicParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "CSVGN1"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PSS/E"; }
  [[nodiscard]] std::string modelName() const override { return "CSVGN1"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  [[nodiscard]] double susceptancePu(const DynamicState& x) const;

  CSVGN1DynamicParams params_;
  StateIndexRange range_;
};

struct DERAADynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"sgen"};
  std::string component_domain{"AC"};
  std::string source_type{"dera_a"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double model_base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double v_ref_pu{1.0};
  double pf_angle_ref_rad{0.0};
  int pf_flag{1};
  int freq_flag{0};
  double T_rv{0.02};
  double Trf{0.02};
  double dbd1{-99.0};
  double dbd2{99.0};
  double K_qv{5.0};
  double Tp{0.02};
  double T_iq{0.02};
  double Tg{0.02};
  double Tv{0.02};
  double Tpord{0.02};
  double Kpg{0.1};
  double Kig{10.0};
  double I_max{1.2};
  double Iq_min{-1.0};
  double Iq_max{1.0};
  double Ip_min{0.0};
  double Ip_max{1.1};
  double rr_pwr{99.0};
  double v_trip_low_pu{0.0};
  double v_trip_high_pu{0.0};
  double f_trip_low_pu{0.0};
  double f_trip_high_pu{0.0};
  double trip_delay_s{0.0};
  bool in_service{true};
};

class DERAADynamic : public DynamicDevice {
 public:
  explicit DERAADynamic(DERAADynamicParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "AggregateDistributedGenerationA"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PSS/E"; }
  [[nodiscard]] std::string modelName() const override { return "AggregateDistributedGenerationA"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  [[nodiscard]] std::pair<double, double> currentDq(const DynamicState& x) const;

  DERAADynamicParams params_;
  StateIndexRange range_;
};

// WECC generic renewable-energy converter model REGC_A (design doc §11.6). The
// network-interface block of the utility-scale PV / wind / BESS plant: current-
// command lags (T_g) on the active and reactive currents, a terminal-voltage
// filter (T_fltr), low-voltage active-current management (LVACM, ramps I_p down
// under sags) and high-voltage reactive-current management (HVRCM, absorbs vars
// under swells). Active/reactive current commands come from a simplified
// electrical control on the P/Q references so the model is usable standalone.
//
// REEC_A electrical control (design doc §11.6): optional inner control that,
// when enabled, replaces the trivial P/Q command with a ramp-limited active-
// power order and a reactive command combining a Q-V droop with a deadband (fast
// dynamic voltage support during faults) and the reactive-power schedule, with
// the REEC current limits. It couples to the REGC_A current lags in the same
// device (this codebase uses monolithic devices rather than a composed bus).
struct REECASettings {
  bool enabled{false};
  double vref0_pu{0.0};     // Q-V droop reference; <= 0 => use the initial Vt
  double dbd1{-0.05};       // Q-V droop deadband on (Vref - Vt), lower edge
  double dbd2{0.05};        // upper edge
  double kqv{2.0};          // reactive-current gain per pu voltage error
  double iqh1{1.0};         // dynamic reactive-current injection max
  double iql1{-1.0};        // min
  double tpord_s{0.05};     // active-power-order lag T_pord
  double p_rate_pu_per_s{99.0};  // active-power-order slew limit
  double ip_min{0.0};
  double ip_max{1.1};
  double iq_min{-1.0};
  double iq_max{1.0};
};

// WECC generic renewable plant controller REPC_A (design doc §11.6): the outer
// plant-level loop. This single-unit-plant realization regulates the device's
// own (filtered) terminal voltage with a deadband PI, producing the reactive-
// power command Qext that feeds the REEC_A reactive reference. Requires REEC_A.
// (Plant-level active-power / frequency control is a documented follow-up.)
struct REPCASettings {
  bool enabled{false};
  double vref_pu{0.0};   // plant voltage setpoint; <= 0 => use the initial Vt
  double dbd1{-0.01};    // voltage-error deadband lower edge
  double dbd2{0.01};     // upper edge
  double kp{1.0};        // plant reactive PI proportional gain
  double ki{5.0};        // plant reactive PI integral gain
  double q_min{-0.44};   // plant reactive command limits (pu of base)
  double q_max{0.44};
  // Plant active-power / frequency droop (design doc §11.6, REPC_A second axis).
  bool freq_control{false};
  double f_nominal_hz{60.0};
  double f_dbd_hz{0.017};   // frequency deadband (Hz)
  double f_droop{0.05};     // droop: per-unit frequency deviation per pu power
  double p_min_pu{0.0};     // plant active-power command limits (pu of base)
  double p_max_pu{1.20};
};

struct REGCADynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"sgen"};
  std::string component_domain{"AC"};
  std::string source_type{"regc_a"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double model_base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double v_ref_pu{1.0};
  double t_g_s{0.02};      // current-command lag T_g
  double t_fltr_s{0.02};   // terminal-voltage filter T_fltr
  double v_lvacm0_pu{0.4};  // LVACM: I_p ramped from 0 here...
  double v_lvacm1_pu{0.8};  //        ...to full at this voltage
  double v_hvrcm_pu{1.2};   // HVRCM: absorb vars above this voltage
  double k_hvrcm{10.0};     // HVRCM gain
  double i_max_pu{1.2};
  bool reactive_priority{false};
  REECASettings reec{};  // optional REEC_A electrical control (design doc §11.6)
  REPCASettings repc{};  // optional REPC_A plant controller (design doc §11.6)
  bool in_service{true};
};

class REGCADynamic : public DynamicDevice {
 public:
  explicit REGCADynamic(REGCADynamicParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void updateSmartControls(double dt, const NetworkState& y) override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "RenewableEnergyGeneratorA"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "WECC"; }
  [[nodiscard]] std::string modelName() const override { return "REGC_A"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;
  [[nodiscard]] FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const override;

 private:
  [[nodiscard]] std::pair<double, double> currentDq(const DynamicState& x) const;

  REGCADynamicParams params_;
  StateIndexRange range_;
  SmartInverterState freq_state_;  // plant-frequency measurement (REPC_A P-f axis)
};

struct InductionMachineDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"motors"};
  std::string component_domain{"AC"};
  std::string source_type{"asynchronous_motor"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double model_base_mva{1.0};
  double p_mech_mw{0.0};
  double q_nom_mvar{0.0};
  double r_s_pu{0.02};
  double x_s_pu{0.10};
  double r_r_pu{0.02};
  double x_r_pu{0.08};
  double x_m_pu{3.0};
  double inertia_h{0.5};
  double damping_d{0.0};
  double torque_exponent{2.0};
  bool fifth_order{false};
  // Use the full flux-based (transient-EMF) single-cage model rather than the
  // simplified current-lag model. Routed to FluxInductionMachineDynamic.
  bool flux_model{false};
  bool in_service{true};
};

class InductionMachineDynamic : public DynamicDevice {
 public:
  explicit InductionMachineDynamic(InductionMachineDynamicParams params);

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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "InductionMachine"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PowerSystems"; }
  [[nodiscard]] std::string modelName() const override {
    return params_.fifth_order ? "SingleCageInductionMachine"
                               : "SimplifiedSingleCageInductionMachine";
  }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  [[nodiscard]] std::complex<double> current(const DynamicState& x,
                                             const NetworkState& y) const;
  [[nodiscard]] double electricalTorque(const DynamicState& x,
                                        const NetworkState& y) const;
  [[nodiscard]] double mechanicalTorque(double omega_r) const;

  InductionMachineDynamicParams params_;
  StateIndexRange range_;
};

// Flux-based single-cage induction machine (design doc §13): the transient-EMF
// (voltage-behind-transient-reactance) form with rotor transient EMFs e'_d, e'_q
// and slip s. It captures the torque-slip characteristic and the stall /
// fault-induced delayed voltage recovery (FIDVR) behavior that the simplified
// current-lag model omits. Motor (load) convention: draws current from the bus.
// Three states [e'_d, e'_q, s]; reuses InductionMachineDynamicParams.
class FluxInductionMachineDynamic : public DynamicDevice {
 public:
  explicit FluxInductionMachineDynamic(InductionMachineDynamicParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "SingleCageInductionMachine"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PowerSystems"; }
  [[nodiscard]] std::string modelName() const override { return "SingleCageInductionMachine"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  struct Reactances {
    double x0;   // stator open-circuit reactance
    double xp;   // stator transient reactance
    double tp0;  // rotor open-circuit transient time constant
    double wb;   // base angular speed
  };
  [[nodiscard]] Reactances reactances() const;
  [[nodiscard]] std::complex<double> statorCurrent(const std::complex<double>& v,
                                                   double ed, double eq) const;
  [[nodiscard]] double loadTorque(double omega_r) const;

  InductionMachineDynamicParams params_;
  StateIndexRange range_;
};

// Active constant-power load (design doc §13): a rectifier-behind-controls CPL
// for electronics-rich feeders. An input filter (series R+jX to a shunt
// capacitor) feeds an idealized constant-power stage that draws
// I = conj(S_ref / V_c) from the filter capacitor. The constant-power stage has
// negative incremental resistance (as the capacitor voltage falls the drawn
// current rises), which interacts with the filter to reproduce the classic CPL
// instability dynamically rather than as a static injection. Two states: the
// filter-capacitor voltage phasor (real, imag). A low-voltage floor turns the
// stage into constant current below v_min_pu to bound the runaway.
struct ActiveConstantPowerLoadParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string canvas_type{"loads"};
  std::string component_domain{"AC"};
  std::string source_type{"active_cpl"};
  std::vector<DynamicModelProfile> model_profiles;
  double base_mva{100.0};
  double p_mw{0.0};          // constant-power setpoint (load convention, > 0 = load)
  double q_mvar{0.0};
  double filter_r_pu{0.01};  // input filter series resistance
  double filter_x_pu{0.05};  // input filter series reactance
  double filter_c_s{0.02};   // filter-capacitor time constant (seconds)
  double v_min_pu{0.20};     // capacitor-voltage floor for the power stage
  bool in_service{true};
};

class ActiveConstantPowerLoadDynamic : public DynamicDevice {
 public:
  explicit ActiveConstantPowerLoadDynamic(ActiveConstantPowerLoadParams params);

  void assignStateIndices(int& offset) override;
  void initializeFromPowerFlow(const PowerFlowResult& pf,
                               DynamicState& x,
                               NetworkState& y) override;
  bool trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) override;
  void computeDerivatives(double t,
                          const DynamicState& x,
                          const NetworkState& y,
                          Eigen::Ref<Eigen::VectorXd> dxdt) const override;
  void stamp(double t,
             const DynamicState& x,
             const NetworkState& y,
             DynamicStamp& stamp) const override;
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
  void handleEvent(const DynamicEvent& event,
                   DynamicState& x,
                   NetworkState& y) override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::string type() const override { return "ActiveConstantPowerLoad"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }
  [[nodiscard]] std::string modelStandard() const override { return "PowerSystems"; }
  [[nodiscard]] std::string modelName() const override { return "ActiveConstantPowerLoad"; }
  [[nodiscard]] std::vector<DynamicModelProfile> modelProfiles() const override;
  [[nodiscard]] DynamicDeviceOutput output(const DynamicState& x,
                                           const NetworkState& y) const override;

 private:
  [[nodiscard]] std::complex<double> loadCurrent(const std::complex<double>& vc) const;

  ActiveConstantPowerLoadParams params_;
  StateIndexRange range_;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
  void addJacobian(double t,
                   const DynamicState& x,
                   const NetworkState& y,
                   const DynamicJacobianContext& context,
                   std::vector<Eigen::Triplet<double>>& triplets) const override;
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
