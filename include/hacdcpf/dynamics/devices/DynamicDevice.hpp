#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicResults.hpp"
#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/DynamicState.hpp"
#include "hacdcpf/dynamics/NetworkState.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::dynamics {

struct DynamicJacobianContext {
  int n_x{0};
  int n_ac{0};
  int n_dc{0};
  int ac_real_offset{0};
  int ac_imag_offset{0};
  int dc_offset{0};
  int total_size{0};
  double dt{0.0};
  double theta{1.0};
  bool include_differential_derivatives{false};

  [[nodiscard]] bool validStateIndex(int idx) const noexcept {
    return idx >= 0 && idx < n_x;
  }

  [[nodiscard]] bool validAcNode(int node) const noexcept {
    return node >= 0 && node < n_ac;
  }

  [[nodiscard]] bool validDcNode(int node) const noexcept {
    return node >= 0 && node < n_dc;
  }

  [[nodiscard]] int acRealRow(int node) const noexcept {
    return ac_real_offset + node;
  }

  [[nodiscard]] int acImagRow(int node) const noexcept {
    return ac_imag_offset + node;
  }

  [[nodiscard]] int acRealCol(int node) const noexcept {
    return ac_real_offset + node;
  }

  [[nodiscard]] int acImagCol(int node) const noexcept {
    return ac_imag_offset + node;
  }

  [[nodiscard]] int dcRow(int node) const noexcept {
    return dc_offset + node;
  }

  [[nodiscard]] int dcCol(int node) const noexcept {
    return dc_offset + node;
  }
};

// Frequency observability contract (design doc §7, §23.3). Each device reports
// how it participates in island frequency so the solver can compute a
// center-of-inertia (COI) frequency, detect islands that have lost their
// frequency anchor, and expose measured/aggregate frequency as an output rather
// than copying the constant nominal value. Pure loads and passive elements leave
// this at its default (no participation).
struct FrequencyParticipation {
  bool is_source{false};        // generation-capable device (not a pure load)
  bool is_anchor{false};        // can set an island's frequency (machine/GFM/slack)
  bool contributes_coi{false};  // carries a physical speed state for the COI average
  int ac_bus_pos{-1};           // AC bus position for island assignment (-1 => none)
  double inertia_h{0.0};        // inertia constant H on the device base [s]
  double base_mva{0.0};         // device base S [MVA]
  double speed_pu{1.0};         // rotor / virtual speed (pu of nominal frequency)
};

class DynamicDevice {
 public:
  virtual ~DynamicDevice() = default;

  virtual void assignStateIndices(int& offset) = 0;

  virtual void initializeFromPowerFlow(const PowerFlowResult& pf,
                                       DynamicState& x,
                                       NetworkState& y) = 0;

  virtual bool trimToNetworkEquilibrium(DynamicState& x,
                                        NetworkState& y) {
    (void)x;
    (void)y;
    return false;
  }

  virtual void computeDerivatives(double t,
                                  const DynamicState& x,
                                  const NetworkState& y,
                                  Eigen::Ref<Eigen::VectorXd> dxdt) const = 0;

  virtual void maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const {
    (void)dxdt;
  }

  virtual void stamp(double t,
                     const DynamicState& x,
                     const NetworkState& y,
                     DynamicStamp& stamp) const = 0;

  virtual void updateAlgebraicOutputs(const DynamicState& x,
                                      const NetworkState& y) {
    (void)x;
    (void)y;
  }

  // Reports this device's contribution to island frequency (see
  // FrequencyParticipation). Default: no participation (pure loads, passive
  // network elements). Synchronous machines and grid-forming inverters override
  // this to anchor and weight the center-of-inertia average.
  [[nodiscard]] virtual FrequencyParticipation frequencyParticipation(
      const DynamicState& x, const NetworkState& y) const {
    (void)x;
    (void)y;
    return {};
  }

  [[nodiscard]] virtual DynamicDeviceOutput output(const DynamicState& x,
                                                   const NetworkState& y) const {
    (void)x;
    (void)y;
    DynamicDeviceOutput out;
    out.name = name();
    out.type = type();
    out.component_index = componentIndex();
    out.model_standard = modelStandard();
    out.model_name = modelName();
    out.parameter_set = parameterSet();
    out.model_profiles = modelProfiles();
    return out;
  }

  virtual void handleEvent(const DynamicEvent& event,
                           DynamicState& x,
                           NetworkState& y) {
    (void)event;
    (void)x;
    (void)y;
  }

  // Device-level protection hook (IEEE 1547 ride-through, design doc §11.7).
  // Called once per accepted step of length `dt` ending at time `t`, after the
  // network is consistent. A device with protection evaluates its ride-through
  // state machine on measured (filtered) terminal quantities (§7 role 4) and may
  // change its own in-service status: a trip zeroes/freezes its states, a
  // reconnect re-seeds them and ramps power back. Returns true when the status
  // change requires the network matrices to be rebuilt; any trip/reconnect is
  // appended to `events` for the results log (§17). Default: no protection.
  virtual bool updateProtection(double t,
                                double dt,
                                DynamicState& x,
                                NetworkState& y,
                                std::vector<DynamicEvent>& events) {
    (void)t;
    (void)dt;
    (void)x;
    (void)y;
    (void)events;
    return false;
  }

  // Smart-inverter control hook (IEEE 1547 volt-var / frequency-watt, design doc
  // §11.7). Called once per accepted step of length `dt` after the network is
  // consistent, always (not gated). A device with these functions advances its
  // filtered + slew-limited volt-var / frequency-watt references on the measured
  // terminal, which its residual then reads. Default: no smart-inverter control.
  virtual void updateSmartControls(double dt, const NetworkState& y) {
    (void)dt;
    (void)y;
  }

  virtual void addJacobian(double t,
                           const DynamicState& x,
                           const NetworkState& y,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)t;
    (void)x;
    (void)y;
    (void)triplets;
  }

  virtual void addJacobian(double t,
                           const DynamicState& x,
                           const NetworkState& y,
                           const DynamicJacobianContext& context,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)context;
    addJacobian(t, x, y, triplets);
  }

  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual std::string type() const = 0;
  [[nodiscard]] virtual int componentIndex() const = 0;
  [[nodiscard]] virtual std::string modelStandard() const { return "HACDCPF"; }
  [[nodiscard]] virtual std::string modelName() const { return type(); }
  [[nodiscard]] virtual std::string parameterSet() const { return {}; }
  [[nodiscard]] virtual std::vector<DynamicModelProfile> modelProfiles() const {
    DynamicModelProfile profile;
    profile.standard = modelStandard();
    profile.model_name = modelName();
    profile.parameter_set = parameterSet();
    return {profile};
  }
};

}  // namespace hacdcpf::dynamics
