#pragma once

#include <complex>
#include <string>

#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"

namespace hacdcpf::dynamics {

struct ACLoadDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  double p_mw{0.0};
  double q_mvar{0.0};
  double base_mva{100.0};
  double scale{1.0};
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

 private:
  ACLoadDynamicParams params_;
};

struct DCLoadDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
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

 private:
  DCLoadDynamicParams params_;
};

struct VoltageSourceDynamicParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  std::string label;
  std::string device_type{"VoltageSource"};
  double base_mva{100.0};
  double vm_set_pu{1.0};
  double angle_set_rad{0.0};
  double frequency_hz{50.0};
  double r_pu{0.0};
  double x_pu{0.10};
  double p_mech_mw{0.0};
  double inertia_h{0.0};
  double damping_d{1.0};
  double droop_r{0.05};
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

 private:
  VoltageSourceDynamicParams params_;
  StateIndexRange range_;
};

struct GridFormingInverterParams {
  int component_index{0};
  int bus{0};
  int bus_pos{-1};
  int dc_bus_pos{-1};
  std::string label;
  std::string device_type{"GridFormingInverter"};
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double v_ref_pu{1.0};
  double angle_ref_rad{0.0};
  double frequency_hz{50.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.02};
  double q_droop_pu{0.04};
  double power_filter_t_s{0.05};
  double eta{0.98};
  bool in_service{true};
};

class GridFormingInverter : public DynamicDevice {
 public:
  explicit GridFormingInverter(GridFormingInverterParams params);

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
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }

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
  double base_mva{100.0};
  double p_ref_mw{0.0};
  double q_ref_mvar{0.0};
  double response_t_s{0.02};
  double current_limit_pu{0.0};
  double eta{0.98};
  bool stamp_dc_power{false};
  bool in_service{true};
};

class GridFollowingInverter : public DynamicDevice {
 public:
  explicit GridFollowingInverter(GridFollowingInverterParams params);

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
  [[nodiscard]] std::string type() const override { return params_.device_type; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }

 private:
  GridFollowingInverterParams params_;
  StateIndexRange range_;
};

struct DCDCConverterDynamicParams {
  int component_index{0};
  int bus_in{0};
  int bus_out{0};
  int bus_in_pos{-1};
  int bus_out_pos{-1};
  std::string label;
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
  bool in_service{true};
};

class BatteryDynamic : public DynamicDevice {
 public:
  explicit BatteryDynamic(BatteryDynamicParams params);

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
  [[nodiscard]] std::string type() const override { return params_.is_ac ? "ACStorage" : "DCStorage"; }
  [[nodiscard]] int componentIndex() const override { return params_.component_index; }

 private:
  BatteryDynamicParams params_;
  StateIndexRange range_;
};

}  // namespace hacdcpf::dynamics
