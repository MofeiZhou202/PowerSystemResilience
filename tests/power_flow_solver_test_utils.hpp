#pragma once

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::test {

inline HybridPowerSystem make_two_bus_voltage_stability_case() {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.vmin_pu = 0.1;
  slack.vmax_pu = 1.2;

  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.vm_pu = 0.8;
  load.pd_mw = 50.0;
  load.qd_mvar = 20.0;
  load.vmin_pu = 0.1;
  load.vmax_pu = 1.2;
  sys.ac.buses = {slack, load};

  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.0;
  line.x_pu = 0.5;
  line.tap = 1.0;
  line.in_service = true;
  sys.ac.branches = {line};

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  generator.vg_pu = 1.0;
  generator.qmin_mvar = -1000.0;
  generator.qmax_mvar = 1000.0;
  generator.pmin_mw = -1000.0;
  generator.pmax_mw = 1000.0;
  sys.ac.generators = {generator};
  return sys;
}

}  // namespace hacdcpf::test
