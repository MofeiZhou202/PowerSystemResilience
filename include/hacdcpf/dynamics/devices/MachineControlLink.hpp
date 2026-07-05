#pragma once

#include "hacdcpf/dynamics/DynamicState.hpp"
#include "hacdcpf/dynamics/devices/InnerVariableBus.hpp"

namespace hacdcpf::dynamics {

// Published by a SynchronousMachine so attached controllers (governor, exciter,
// PSS) can read the machine's rotor speed and write the derivative of a
// machine-owned actuator state (mechanical power / field). The machine keeps
// owning the state value (initialization, equilibrium trim, and residual masking
// stay in the machine); a controller only overrides that state's time
// derivative. `range` points at the machine's own StateIndexRange, whose address
// is stable for the lifetime of the machine device, so the final state offset is
// visible to controllers after assignStateIndices() has run for all devices.
struct MachineControlLink {
  const StateIndexRange* range{nullptr};
  bool valid{false};
  bool genrou{false};
  int bus_pos{-1};
  double base_mva{100.0};
  double frequency_hz{50.0};
  double inertia_h{1.0};
  int omega_local{1};  // machine speed at local index 1
  int pm_local{3};     // classical pm=3; OneDOneQ tau_m=4; GENROU tau_m=6
  int efd_local{2};    // classical field=2; OneDOneQ vf=5; GENROU vf=7
  GeneratorInnerVariableBus inner_vars;

  [[nodiscard]] int omegaIndex() const {
    return inner_vars.valid ? inner_vars.omegaIndex()
                            : (range ? range->offset + omega_local : -1);
  }
  [[nodiscard]] int pmIndex() const {
    return inner_vars.valid ? inner_vars.mechanicalTorqueIndex()
                            : (range ? range->offset + pm_local : -1);
  }
  [[nodiscard]] int fieldIndex() const {
    return inner_vars.valid ? inner_vars.fieldVoltageIndex()
                            : (range ? range->offset + efd_local : -1);
  }
  [[nodiscard]] const GeneratorInnerVariableBus& generatorBus() const {
    return inner_vars;
  }
};

// Published by a PowerSystemStabilizer so its parent exciter can add the
// stabilizing signal Vs into the AVR voltage summing junction. Vs is a pure
// algebraic function of the PSS states and the machine speed deviation, so the
// exciter recomputes it on demand (no mutable cache needed in const paths).
struct PSSOutputLink {
  const StateIndexRange* range{nullptr};   // the PSS device's own state slice
  const MachineControlLink* machine{nullptr};
  bool valid{false};
  int model{0};  // 0=PSS1A, 1=IEEEST, 2=STAB1
  double ks{0.0};
  double tw_s{10.0};
  double t1_s{0.0};
  double t2_s{0.0};
  double t3_s{0.0};
  double t4_s{0.0};
  double vs_max_pu{0.1};
  double vs_min_pu{-0.1};
  double a1{0.0};
  double a2{1.0};
  double a3{1.0};
  double a4{1.0};
  double a5{1.0};
  double a6{0.0};
  double t5_s{0.1};
  double t6_s{0.05};
  double vcu{0.0};
  double vcl{0.0};
  int input_code{1};
  double kt{5.0};
  double stab_t_s{10.0};
  double t1_over_t3{1.0};
  double t2_over_t4{1.0};
  double h_lim{0.1};
};

}  // namespace hacdcpf::dynamics
