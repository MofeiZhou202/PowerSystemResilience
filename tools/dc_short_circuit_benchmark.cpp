#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>

#include "hacdcpf/analysis/dc_short_circuit.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

int main() {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;
  constexpr int bus_count = 1000;
  constexpr int fault_count = 64;
  HybridPowerSystem sys;
  sys.base_mva = sys.dc.base_mva = 100.0;
  sys.dc.buses.reserve(bus_count);
  sys.dc.branches.reserve(bus_count - 1);
  for (int i = 0; i < bus_count; ++i) {
    DCBus bus;
    bus.index = i + 1;
    bus.bus_type = i == 0 ? DCBusType::DC_V : DCBusType::DC_P;
    bus.base_kv = 320.0;
    bus.vm_pu = 1.0;
    sys.dc.buses.push_back(bus);
    if (i == 0) continue;
    DCBranch branch;
    branch.index = i;
    branch.from_bus = i;
    branch.to_bus = i + 1;
    branch.r_pu = 1e-3;
    sys.dc.branches.push_back(branch);
  }
  std::vector<int> fault_buses;
  fault_buses.reserve(fault_count);
  for (int i = 0; i < fault_count; ++i)
    fault_buses.push_back(2 + i * (bus_count - 2) / (fault_count - 1));

  std::vector<double> batch_samples;
  batch_samples.reserve(200);
  std::vector<DCFaultResult> batch;
  for (int sample = 0; sample < 200; ++sample) {
    const auto start = std::chrono::steady_clock::now();
    batch = dc_bus_fault_levels(sys, fault_buses);
    const auto end = std::chrono::steady_clock::now();
    batch_samples.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::vector<double> repeated_samples;
  repeated_samples.reserve(5);
  std::vector<DCFaultResult> repeated;
  for (int sample = 0; sample < 5; ++sample) {
    repeated.clear();
    repeated.reserve(fault_buses.size());
    const auto start = std::chrono::steady_clock::now();
    for (int bus : fault_buses) repeated.push_back(dc_bus_fault_level(sys, bus));
    const auto end = std::chrono::steady_clock::now();
    repeated_samples.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }

  double max_difference = 0.0;
  for (size_t i = 0; i < batch.size(); ++i) {
    if (!batch[i].solved || !repeated[i].solved) return 2;
    max_difference = std::max(max_difference,
                              std::abs(batch[i].i_fault_pu - repeated[i].i_fault_pu));
  }
  std::sort(batch_samples.begin(), batch_samples.end());
  std::sort(repeated_samples.begin(), repeated_samples.end());
  const double batch_ms = batch_samples[batch_samples.size() / 2];
  const double repeated_ms = repeated_samples[repeated_samples.size() / 2];
  std::cout << "buses=" << bus_count << " faults=" << fault_count
            << " batch_ms=" << batch_ms
            << " repeated_single_ms=" << repeated_ms
            << " ratio=" << batch_ms / repeated_ms
            << " max_current_difference_pu=" << max_difference << '\n';
  return max_difference <= 1e-9 ? 0 : 3;
}
