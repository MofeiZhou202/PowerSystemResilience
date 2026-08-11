// =============================================================================
// three_stage_reliability.cpp
//
// Native C++ three-stage fault-recovery reliability evaluator.  This version
// keeps the public JSON schema and evaluates staged load restoration with the
// embedded MIPSolvers C++ MILP engine.
// =============================================================================

#include "hacdcpf/analysis/three_stage_reliability.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace fs = std::filesystem;
namespace hacdcpf::analysis {
namespace {

constexpr double kTauSwitchHr = 1.0 / 60.0;
constexpr double kTauTrippingHr = 1.0 / 30.0;
constexpr double kTauRepairHr = 1.0;
constexpr double kReliabilityVoll = 10.0;
constexpr double kDefaultFailureRate = 0.1;
// Darwin pthreads default to a 512 KiB stack. ASan expands the embedded
// solver's largest observed frame to about 340 KiB, so use an 8x stack margin
// for solver-bearing contingency workers. POSIX platforms with a larger
// default retain their existing stack size.
constexpr size_t kReliabilityWorkerStackBytes = 4U * 1024U * 1024U;

// HiGHS scheduler reset and native B&C process state are not safe across
// concurrent solves. Fault preprocessing remains parallel outside this lock.
std::mutex stage_milp_solve_mutex;

// DC bus IDs are shifted by kDCBusOffset throughout the NativeCase model so
// that DC bus k is represented as (k + kDCBusOffset) in the c.buses / c.loads /
// c.sources vectors.  This prevents ID collisions when AC bus i and DC bus i
// both exist in the hybrid system (e.g. AC bus 1 vs DC bus 1).  The offset is
// purely internal; it is never exposed to the caller.
constexpr int kDCBusOffset = 1'000'000;
struct LoadPoint {
  int bus{0};
  double p_kw{0.0};
  double customers{1.0};
  double q_kvar{0.0};  // F14: measured reactive demand (0 -> reconstruct from 0.9 PF)
  ReliabilityComponentKind component_kind{ReliabilityComponentKind::Unknown};
  int component_position{-1};
};

struct SourcePoint {
  int bus{0};
  double p_kw{0.0};
  ReliabilityComponentKind component_kind{ReliabilityComponentKind::Unknown};
  int component_position{-1};
};

enum class FaultKind { ACBranch, DCBranch, Generator, Transformer2W,
                       VSCConverter, DCDCConverter, ACSwitch, ACCircuitBreaker,
                       DCCircuitBreaker, StaticGenerator, RenewableGenerator,
                       PVSystem, Storage, DCStaticGenerator,
                       DCStaticGeneratorAC, DCPVArray, DCStorage,
                       MobileStorage, VirtualPowerPlant, Microgrid };

std::string fault_kind_type(FaultKind kind) {
  switch (kind) {
    case FaultKind::ACBranch: return "ac_branch";
    case FaultKind::DCBranch: return "dc_branch";
    case FaultKind::Generator: return "generator";
    case FaultKind::Transformer2W: return "transformer_2w";
    case FaultKind::VSCConverter: return "vsc_converter";
    case FaultKind::DCDCConverter: return "dcdc_converter";
    case FaultKind::ACSwitch: return "ac_switch";
    case FaultKind::ACCircuitBreaker: return "ac_circuit_breaker";
    case FaultKind::DCCircuitBreaker: return "dc_circuit_breaker";
    case FaultKind::StaticGenerator: return "static_generator";
    case FaultKind::RenewableGenerator: return "renewable_gen";
    case FaultKind::PVSystem: return "ac_pv_system";
    case FaultKind::Storage: return "storage";
    case FaultKind::DCStaticGenerator: return "dc_static_generator";
    case FaultKind::DCStaticGeneratorAC: return "dc_static_generator_ac";
    case FaultKind::DCPVArray: return "dc_pv_array";
    case FaultKind::DCStorage: return "dc_storage";
    case FaultKind::MobileStorage: return "mobile_storage";
    case FaultKind::VirtualPowerPlant: return "virtual_power_plant";
    case FaultKind::Microgrid: return "microgrid";
  }
  return "unknown";
}

struct FaultLine {
  int id{0};
  bool ac{true};
  int index{0};
  int from_bus{0};
  int to_bus{0};
  bool normally_in_service{true};
  double failure_rate{0.0};
  // Per-component stage durations (hr).  Isolation/switching default to the
  // global boundary times (no per-branch field exists); the repair stage
  // absorbs the faulted component's MTTR (r_m - tau_iso - tau_sw).
  double tau_iso_hr{kTauSwitchHr};
  double tau_sw_hr{kTauTrippingHr - kTauSwitchHr};
  double tau_rep_hr{kTauRepairHr - kTauTrippingHr};
  FaultKind kind{FaultKind::ACBranch};
  double initiating_failure_rate{0.0};
  double scenario_probability{1.0};
  std::string protection_scenario{"unconfigured"};
  std::string protection_id;
  std::string primary_device_id;
  std::string backup_device_id;
  double reclose_success_probability{0.0};
  double primary_failure_probability{0.0};
  double backup_failure_probability{0.0};
  double clearing_time_s{0.0};
  std::vector<ComponentRef> additional_outages;
  std::vector<std::string> protection_zone_component_ids;
  bool restoration_forbidden{false};
};

struct NativeCase {
  HybridPowerSystem sys;
  std::vector<int> buses;
  std::vector<LoadPoint> loads;
  std::vector<SourcePoint> sources;
  std::vector<FaultLine> faults;
  bool include_generator_faults{false};
  bool include_transformer_faults{false};
  bool include_converter_faults{false};
  bool include_switch_faults{false};
  bool include_dc_power_flow{true};
  bool protection_configuration_applied{false};
  int protection_rows_applied{0};
  int protection_scenarios_generated{0};
  double initiating_fault_frequency_per_year{0.0};
  double sustained_fault_frequency_per_year{0.0};
  double transient_reclose_frequency_per_year{0.0};
  std::vector<std::string> protection_configuration_limitations;
};

ReliabilityComponentKind primary_component_kind(FaultKind kind) {
  using K = ReliabilityComponentKind;
  switch (kind) {
    case FaultKind::ACBranch: return K::ACBranch;
    case FaultKind::DCBranch: return K::DCBranch;
    case FaultKind::Generator: return K::ACGenerator;
    case FaultKind::Transformer2W: return K::ACTransformer2W;
    case FaultKind::VSCConverter: return K::VSCConverter;
    case FaultKind::DCDCConverter: return K::DCDCConverter;
    case FaultKind::ACSwitch: return K::ACSwitch;
    case FaultKind::ACCircuitBreaker: return K::ACCircuitBreaker;
    case FaultKind::DCCircuitBreaker: return K::DCCircuitBreaker;
    case FaultKind::StaticGenerator: return K::ACStaticGenerator;
    case FaultKind::RenewableGenerator: return K::ACRenewableGenerator;
    case FaultKind::PVSystem: return K::ACPVSystem;
    case FaultKind::Storage: return K::ACStorage;
    case FaultKind::DCStaticGenerator: return K::DCStaticGenerator;
    case FaultKind::DCStaticGeneratorAC: return K::DCStaticGeneratorAC;
    case FaultKind::DCPVArray: return K::DCPVArray;
    case FaultKind::DCStorage: return K::DCStorage;
    case FaultKind::MobileStorage: return K::MobileStorage;
    case FaultKind::VirtualPowerPlant: return K::VirtualPowerPlant;
    case FaultKind::Microgrid: return K::Microgrid;
  }
  return K::Unknown;
}

bool component_outaged(const FaultLine& fault, ReliabilityComponentKind kind,
                       int position) {
  if (primary_component_kind(fault.kind) == kind && fault.index == position)
    return true;
  return std::any_of(fault.additional_outages.begin(),
                     fault.additional_outages.end(),
                     [&](const ComponentRef& ref) {
                       return ref.kind == kind && ref.element_index == position;
                     });
}

bool load_outaged(const FaultLine& fault, const LoadPoint& load) {
  return component_outaged(fault, load.component_kind,
                            load.component_position);
}

template <typename T>
std::optional<ComponentRef> fault_component_ref(
    const FaultLine& fault, ReliabilityComponentKind kind,
    const std::vector<T>& values) {
  if (fault.index < 0 || fault.index >= static_cast<int>(values.size()))
    return std::nullopt;
  ComponentRef ref;
  ref.kind = kind;
  ref.element_index = fault.index;
  ref.component_index = values[static_cast<size_t>(fault.index)].index;
  ref.element_name = values[static_cast<size_t>(fault.index)].name;
  ref.stable_id = to_string(kind) + ":" + std::to_string(ref.component_index);
  return ref;
}

std::optional<ComponentRef> fault_component_ref(const HybridPowerSystem& sys,
                                                const FaultLine& fault) {
  using K = ReliabilityComponentKind;
  switch (fault.kind) {
    case FaultKind::ACBranch:
      return fault_component_ref(fault, K::ACBranch, sys.ac.branches);
    case FaultKind::DCBranch:
      return fault_component_ref(fault, K::DCBranch, sys.dc.branches);
    case FaultKind::Generator:
      return fault_component_ref(fault, K::ACGenerator, sys.ac.generators);
    case FaultKind::Transformer2W:
      return fault_component_ref(fault, K::ACTransformer2W,
                                 sys.ac.transformers_2w);
    case FaultKind::VSCConverter:
      return fault_component_ref(fault, K::VSCConverter, sys.vsc_converters);
    case FaultKind::DCDCConverter:
      return fault_component_ref(fault, K::DCDCConverter,
                                 sys.dc.dcdc_converters);
    case FaultKind::ACSwitch:
      return fault_component_ref(fault, K::ACSwitch, sys.ac.switches);
    case FaultKind::ACCircuitBreaker:
      return fault_component_ref(fault, K::ACCircuitBreaker,
                                 sys.ac.circuit_breakers);
    case FaultKind::DCCircuitBreaker:
      return fault_component_ref(fault, K::DCCircuitBreaker,
                                 sys.dc.dc_circuit_breakers);
    case FaultKind::StaticGenerator:
      return fault_component_ref(fault, K::ACStaticGenerator,
                                 sys.ac.static_generators);
    case FaultKind::RenewableGenerator:
      return fault_component_ref(fault, K::ACRenewableGenerator,
                                 sys.ac.renewable_gens);
    case FaultKind::PVSystem:
      return fault_component_ref(fault, K::ACPVSystem, sys.ac.pv_systems);
    case FaultKind::Storage:
      return fault_component_ref(fault, K::ACStorage, sys.ac.storage);
    case FaultKind::DCStaticGenerator:
      return fault_component_ref(fault, K::DCStaticGenerator,
                                 sys.dc.dc_static_generators);
    case FaultKind::DCStaticGeneratorAC:
      return fault_component_ref(fault, K::DCStaticGeneratorAC,
                                 sys.dc.static_generators);
    case FaultKind::DCPVArray:
      return fault_component_ref(fault, K::DCPVArray, sys.dc.pv_arrays);
    case FaultKind::DCStorage:
      return fault_component_ref(fault, K::DCStorage, sys.dc.storage);
    case FaultKind::MobileStorage:
      return fault_component_ref(fault, K::MobileStorage, sys.mobile_storage);
    case FaultKind::VirtualPowerPlant:
      return fault_component_ref(fault, K::VirtualPowerPlant, sys.vpps);
    case FaultKind::Microgrid:
      return fault_component_ref(fault, K::Microgrid, sys.microgrids);
  }
  return std::nullopt;
}

double combined_primary_failure_probability(
    const ProtectionConfiguration& protection) {
  const double trip = std::clamp(protection.fail_to_trip_probability, 0.0, 1.0);
  const double open = std::clamp(protection.fail_to_open_probability, 0.0, 1.0);
  // Series-success model for command/trip and physical contact opening.
  // Derivation: docs/reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md
  // Sections 17-18; P(success)=(1-p_trip)(1-p_open).
  return 1.0 - (1.0 - trip) * (1.0 - open);
}

void append_unique_outage(std::vector<ComponentRef>& refs,
                          const ComponentRef& candidate) {
  if (std::none_of(refs.begin(), refs.end(), [&](const ComponentRef& ref) {
        return ref.kind == candidate.kind &&
               ref.element_index == candidate.element_index;
      }))
    refs.push_back(candidate);
}

bool three_stage_zone_kind_supported(ReliabilityComponentKind kind) {
  using K = ReliabilityComponentKind;
  switch (kind) {
    case K::ACGenerator:
    case K::ACBranch:
    case K::ACLoad:
    case K::ACBusLoad:
    case K::ACStaticGenerator:
    case K::ACRenewableGenerator:
    case K::ACStorage:
    case K::ACPVSystem:
    case K::ACTransformer2W:
    case K::ACSwitch:
    case K::ACCircuitBreaker:
    case K::ExternalGrid:
    case K::DCBusLoad:
    case K::DCBranch:
    case K::DCLoad:
    case K::DCStaticGenerator:
    case K::DCStaticGeneratorAC:
    case K::DCDCConverter:
    case K::DCCircuitBreaker:
    case K::DCStorage:
    case K::DCPVArray:
    case K::VSCConverter:
    case K::MobileStorage:
    case K::VirtualPowerPlant:
    case K::Microgrid:
      return true;
    default:
      return false;
  }
}

void apply_protection_configuration(
    NativeCase& c, const ReliabilityConfiguration& configuration) {
  for (const auto& fault : c.faults)
    c.initiating_fault_frequency_per_year += fault.failure_rate;
  if (configuration.protection.empty()) {
    c.sustained_fault_frequency_per_year =
        c.initiating_fault_frequency_per_year;
    return;
  }

  std::unordered_map<std::string, const ProtectionConfiguration*> by_component;
  std::unordered_map<std::string, const ProtectionConfiguration*> by_device;
  for (const auto& row : configuration.protection) {
    if (!row.enabled) continue;
    by_component[row.protected_component_id] = &row;
    by_device[row.protective_device_id] = &row;
  }

  std::unordered_set<std::string> applied_rows;
  std::unordered_set<std::string> limitations;
  std::vector<FaultLine> scenarios;
  scenarios.reserve(c.faults.size() * 2U);
  int next_id = 1;
  for (const auto& base : c.faults) {
    const auto ref = fault_component_ref(c.sys, base);
    const auto configured = ref ? by_component.find(ref->stable_id)
                                : by_component.end();
    if (configured == by_component.end()) {
      auto unchanged = base;
      unchanged.id = next_id++;
      unchanged.initiating_failure_rate = base.failure_rate;
      unchanged.clearing_time_s = base.tau_iso_hr * 3600.0;
      scenarios.push_back(std::move(unchanged));
      c.sustained_fault_frequency_per_year += base.failure_rate;
      continue;
    }

    const auto& protection = *configured->second;
    applied_rows.insert(protection.protection_id);
    const double reclose = protection.automatic_reclose
        ? std::clamp(protection.successful_reclose_probability, 0.0, 1.0)
        : 0.0;
    const double primary_failure =
        combined_primary_failure_probability(protection);
    double backup_failure = 0.0;
    if (!protection.backup_device_id.empty()) {
      const auto backup_row = by_device.find(protection.backup_device_id);
      if (backup_row != by_device.end()) {
        backup_failure =
            combined_primary_failure_probability(*backup_row->second);
      } else {
        limitations.insert("Backup device '" + protection.backup_device_id +
                           "' has no protection row; its clearing availability is assumed to be 1.0.");
      }
    } else if (primary_failure > 0.0) {
      limitations.insert("Protection row '" + protection.protection_id +
                         "' has no backup_device_id; configured zone clearance is treated as an abstract upstream backup action.");
    }

    const double total_duration =
        base.tau_iso_hr + base.tau_sw_hr + base.tau_rep_hr;
    const double permanent_probability = 1.0 - reclose;
    c.transient_reclose_frequency_per_year += base.failure_rate * reclose;

    const auto add_scenario = [&](double conditional_probability,
                                  const std::string& outcome,
                                  double clearing_time_s,
                                  bool use_backup_zone,
                                  bool restoration_forbidden) {
      if (conditional_probability <= 1e-15) return;
      FaultLine scenario = base;
      scenario.id = next_id++;
      scenario.initiating_failure_rate = base.failure_rate;
      scenario.scenario_probability = conditional_probability;
      scenario.failure_rate = base.failure_rate * conditional_probability;
      scenario.protection_scenario = outcome;
      scenario.protection_id = protection.protection_id;
      scenario.primary_device_id = protection.protective_device_id;
      scenario.backup_device_id = protection.backup_device_id;
      scenario.reclose_success_probability = reclose;
      scenario.primary_failure_probability = primary_failure;
      scenario.backup_failure_probability = backup_failure;
      scenario.clearing_time_s = clearing_time_s > 0.0
          ? clearing_time_s : base.tau_iso_hr * 3600.0;
      scenario.tau_iso_hr = scenario.clearing_time_s / 3600.0;
      scenario.tau_rep_hr = std::max(
          0.0, total_duration - scenario.tau_iso_hr - scenario.tau_sw_hr);
      scenario.protection_zone_component_ids = protection.zone_component_ids;
      scenario.restoration_forbidden = restoration_forbidden;
      if (!use_backup_zone && protection.resolved_protective_device)
        append_unique_outage(scenario.additional_outages,
                             *protection.resolved_protective_device);
      if (use_backup_zone) {
        for (const auto& zone_ref : protection.resolved_zone_components) {
          if (three_stage_zone_kind_supported(zone_ref.kind)) {
            append_unique_outage(scenario.additional_outages, zone_ref);
          } else {
            limitations.insert("Protection zone component '" +
                               zone_ref.stable_id +
                               "' is outside the three-stage consequence model and is not removed from the recovery topology.");
          }
        }
        if (protection.resolved_backup_device)
          append_unique_outage(scenario.additional_outages,
                               *protection.resolved_backup_device);
      }
      scenarios.push_back(std::move(scenario));
      c.sustained_fault_frequency_per_year +=
          base.failure_rate * conditional_probability;
      ++c.protection_scenarios_generated;
    };

    add_scenario(permanent_probability * (1.0 - primary_failure),
                 "primary_cleared", protection.primary_clearing_time_s,
                 false, false);
    add_scenario(permanent_probability * primary_failure *
                     (1.0 - backup_failure),
                 "backup_cleared", protection.backup_clearing_time_s,
                 true, false);
    add_scenario(permanent_probability * primary_failure * backup_failure,
                 "unresolved_after_backup_failure",
                 protection.backup_clearing_time_s, true, true);
  }

  for (const auto& row : configuration.protection) {
    if (row.enabled && !applied_rows.count(row.protection_id))
      limitations.insert("Protection row '" + row.protection_id +
                         "' does not match an enabled three-stage contingency; enable the corresponding component fault family.");
  }
  c.faults = std::move(scenarios);
  c.protection_rows_applied = static_cast<int>(applied_rows.size());
  c.protection_configuration_applied = !applied_rows.empty();
  c.protection_configuration_limitations.assign(limitations.begin(),
                                                 limitations.end());
}

std::string read_file_text(const fs::path& path) {
  std::ifstream in(path);
  if (!in) return {};
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void add_bus(std::vector<int>& buses, int bus) {
  if (bus <= 0) return;
  if (std::find(buses.begin(), buses.end(), bus) == buses.end()) buses.push_back(bus);
}

std::unordered_map<int, int> bus_position_map(const std::vector<int>& buses) {
  std::unordered_map<int, int> out;
  out.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) out[buses[i]] = i;
  return out;
}

double load_customers(const HybridPowerSystem& sys, const Load& ld) {
  if (ld.n_customers > 0) return static_cast<double>(ld.n_customers);
  for (const auto& b : sys.ac.buses) {
    if (b.index == ld.bus && b.n_customers > 0) return static_cast<double>(b.n_customers);
  }
  return std::max(1.0, ld.p_mw * 10.0);
}

double bus_customers(const ACBus& b) {
  if (b.n_customers > 0) return static_cast<double>(b.n_customers);
  return std::max(1.0, b.pd_mw * 10.0);
}

void add_source(std::vector<SourcePoint>& sources, int bus, double p_mw,
                ReliabilityComponentKind kind, int position) {
  if (bus <= 0 || p_mw <= 1e-9) return;
  sources.push_back({bus, p_mw * 1000.0, kind, position});
}

bool microgrid_contains_bus(const Microgrid& microgrid, int bus) {
  return microgrid.pcc_bus == bus ||
      std::find(microgrid.internal_buses.begin(), microgrid.internal_buses.end(),
                bus) != microgrid.internal_buses.end();
}

bool microgrid_available_for_islanding(const HybridPowerSystem& sys, int bus,
                                       const FaultLine* fault = nullptr) {
  for (int i = 0; i < static_cast<int>(sys.microgrids.size()); ++i) {
    const auto& microgrid = sys.microgrids[i];
    if (!microgrid_contains_bus(microgrid, bus)) continue;
    const bool faulted = fault && component_outaged(
        *fault, ReliabilityComponentKind::Microgrid, i);
    return microgrid.in_service && microgrid.islanding_capability && !faulted;
  }
  return true;
}

double microgrid_declared_capacity_mw(const Microgrid& microgrid) {
  return std::max({0.0, microgrid.capacity_mw,
                   microgrid.total_dg_capacity_mw,
                   microgrid.total_generation_mw});
}

double explicit_microgrid_source_capacity_mw(const HybridPowerSystem& sys,
                                             const Microgrid& microgrid) {
  const auto inside = [&](int bus) { return microgrid_contains_bus(microgrid, bus); };
  double capacity = 0.0;
  for (const auto& generator : sys.ac.generators)
    if (generator.in_service && inside(generator.bus))
      capacity += hacdcpf::model::effective_capacity_mw(generator);
  for (const auto& source : sys.ac.static_generators)
    if (source.in_service && inside(source.bus))
      capacity += hacdcpf::model::effective_capacity_mw(source);
  for (const auto& source : sys.ac.renewable_gens)
    if (source.in_service && inside(source.bus))
      capacity += source.p_rated_mw > 0.0
          ? source.p_rated_mw * source.capacity_factor : source.p_mw;
  for (const auto& source : sys.ac.pv_systems)
    if (source.in_service && inside(source.bus))
      capacity += source.pmax_mw > 0.0 ? source.pmax_mw : source.p_mw;
  for (const auto& storage : sys.ac.storage)
    if (storage.in_service && inside(storage.bus))
      capacity += hacdcpf::model::effective_capacity_mw(storage);
  return std::max(0.0, capacity);
}

double residual_microgrid_capacity_mw(const HybridPowerSystem& sys,
                                      const Microgrid& microgrid) {
  return std::max(0.0, microgrid_declared_capacity_mw(microgrid) -
                           explicit_microgrid_source_capacity_mw(sys, microgrid));
}

double initial_storage_deliverable_energy_mwh(const Storage& storage) {
  if (!storage.in_service || !std::isfinite(storage.e_rated_mwh) ||
      storage.e_rated_mwh <= 0.0 || !std::isfinite(storage.soc_init) ||
      !std::isfinite(storage.soc_min))
    return 0.0;
  const double eta = (std::isfinite(storage.eta_discharge) &&
                      storage.eta_discharge > 0.0 &&
                      storage.eta_discharge <= 1.0)
      ? storage.eta_discharge : 0.0;
  return std::max(0.0, storage.e_rated_mwh *
                           (std::clamp(storage.soc_init, 0.0, 1.0) -
                            std::clamp(storage.soc_min, 0.0, 1.0))) * eta;
}

double initial_storage_deliverable_energy_mwh(const DCStorage& storage) {
  if (!storage.in_service || !std::isfinite(storage.e_rated_mwh) ||
      storage.e_rated_mwh <= 0.0 || !std::isfinite(storage.soc_init) ||
      !std::isfinite(storage.soc_min))
    return 0.0;
  const double eta = (std::isfinite(storage.eta_discharge) &&
                      storage.eta_discharge > 0.0 &&
                      storage.eta_discharge <= 1.0)
      ? storage.eta_discharge : 0.0;
  // Taylor (2015), Sec. 4.3: deliverable terminal energy is the usable SOC
  // window multiplied by the constant discharge efficiency.
  return std::max(0.0, storage.e_rated_mwh *
                           (std::clamp(storage.soc_init, 0.0, 1.0) -
                            std::clamp(storage.soc_min, 0.0, 1.0))) * eta;
}

double initial_storage_deliverable_energy_mwh(const MobileStorage& storage) {
  if (!storage.in_service || storage.status == MobileStorageStatus::InTransit ||
      !std::isfinite(storage.e_rated_mwh) || storage.e_rated_mwh <= 0.0 ||
      !std::isfinite(storage.soc_init) || !std::isfinite(storage.soc_min))
    return 0.0;
  const double eta = (std::isfinite(storage.eta_discharge) &&
                      storage.eta_discharge > 0.0 &&
                      storage.eta_discharge <= 1.0)
      ? storage.eta_discharge : 0.0;
  return std::max(0.0, storage.e_rated_mwh *
                           (std::clamp(storage.soc_init, 0.0, 1.0) -
                            std::clamp(storage.soc_min, 0.0, 1.0))) * eta;
}

double mobile_storage_discharge_capacity_mw(const MobileStorage& storage) {
  if (!storage.in_service || storage.status == MobileStorageStatus::InTransit)
    return 0.0;
  const double capacity = storage.pmax_mw > 0.0 ? storage.pmax_mw
      : (storage.p_rated_mw > 0.0 ? storage.p_rated_mw
                                  : std::max(0.0, storage.p_mw));
  return std::isfinite(capacity) ? capacity : 0.0;
}

double vpp_available_generation_mw(const VirtualPowerPlant& vpp) {
  if (!vpp.in_service) return 0.0;
  const double capacity = vpp.pmax_mw > 0.0 ? vpp.pmax_mw
      : (vpp.p_generation_sum_mw > 0.0 ? vpp.p_generation_sum_mw
                                       : std::max(0.0, vpp.p_output_mw));
  return std::isfinite(capacity) ? capacity : 0.0;
}

NativeCase build_native_case(const HybridPowerSystem& input,
                             const ThreeStageReliabilityOptions& options = {}) {
  NativeCase c;
  c.sys = input;
  // The persistent JSON model may carry dedicated DCStorage rows while solver
  // code consumes Storage-typed DC rows. Materialize once in the private case
  // copy so stable public input remains unchanged and every DC battery follows
  // the same dispatch/energy chronology.
  materialize_dc_storage(c.sys);
  const HybridPowerSystem& sys = c.sys;
  c.include_generator_faults = options.include_generator_faults;
  c.include_transformer_faults = options.include_transformer_faults;
  c.include_converter_faults = options.include_converter_faults;
  c.include_switch_faults = options.include_switch_faults;
  c.include_dc_power_flow = options.include_dc_power_flow;

  for (const auto& b : sys.ac.buses) {
    if (b.in_service) add_bus(c.buses, b.index);
  }
  for (const auto& b : sys.dc.buses) {
    if (b.in_service) add_bus(c.buses, b.index + kDCBusOffset);
  }

  for (int i = 0; i < static_cast<int>(sys.ac.loads.size()); ++i) {
    const auto& ld = sys.ac.loads[i];
    if (!ld.in_service) continue;
    add_bus(c.buses, ld.bus);
    c.loads.push_back({ld.bus, std::max(0.0, ld.p_mw * ld.scaling * 1000.0),
                       load_customers(sys, ld), ld.q_mvar * ld.scaling * 1000.0,
                       ReliabilityComponentKind::ACLoad, i});
  }
  for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
    const auto& b = sys.ac.buses[i];
    if (!b.in_service || b.pd_mw <= 1e-9) continue;
    c.loads.push_back({b.index, b.pd_mw * 1000.0, bus_customers(b),
                       b.qd_mvar * 1000.0,
                       ReliabilityComponentKind::ACBusLoad, i});
  }
  for (int i = 0; i < static_cast<int>(sys.dc.loads.size()); ++i) {
    const auto& ld = sys.dc.loads[i];
    if (!ld.in_service) continue;
    add_bus(c.buses, ld.bus + kDCBusOffset);
    c.loads.push_back({ld.bus + kDCBusOffset, std::max(0.0, ld.p_mw * ld.scaling * 1000.0),
                       std::max(1.0, ld.p_mw * 10.0), 0.0,
                       ReliabilityComponentKind::DCLoad, i});
  }
  for (int i = 0; i < static_cast<int>(sys.dc.buses.size()); ++i) {
    const auto& b = sys.dc.buses[i];
    if (!b.in_service || b.pd_mw <= 1e-9) continue;
    c.loads.push_back({b.index + kDCBusOffset, b.pd_mw * 1000.0,
                       b.n_customers > 0 ? static_cast<double>(b.n_customers) : std::max(1.0, b.pd_mw * 10.0),
                       0.0, ReliabilityComponentKind::DCBusLoad, i});
  }

  for (int i = 0; i < static_cast<int>(sys.ac.external_grids.size()); ++i) {
    const auto& eg = sys.ac.external_grids[i];
    if (!eg.in_service) continue;
    add_source(c.sources, eg.bus,
               eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : 1.0e4,
               ReliabilityComponentKind::ExternalGrid, i);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    const auto& g = sys.ac.generators[i];
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus, cap,
                              ReliabilityComponentKind::ACGenerator, i);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.static_generators.size()); ++i) {
    const auto& sg = sys.ac.static_generators[i];
    // Effective fixed-injection capacity must honour `scaling`; without it,
    // a unit scheduled out (scaling=0) would still appear at full nameplate
    // power in the connectivity model and over-state restoration headroom.
    if (!microgrid_available_for_islanding(sys, sg.bus)) continue;
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    if (cap > 0.0) add_source(c.sources, sg.bus, cap,
                              ReliabilityComponentKind::ACStaticGenerator, i);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.renewable_gens.size()); ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    if (!rg.in_service || !microgrid_available_for_islanding(sys, rg.bus)) continue;
    add_source(c.sources, rg.bus,
               rg.p_rated_mw > 0.0 ? rg.p_rated_mw * rg.capacity_factor : rg.p_mw,
               ReliabilityComponentKind::ACRenewableGenerator, i);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.pv_systems.size()); ++i) {
    const auto& pv = sys.ac.pv_systems[i];
    if (!pv.in_service || !microgrid_available_for_islanding(sys, pv.bus)) continue;
    add_source(c.sources, pv.bus, pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw,
               ReliabilityComponentKind::ACPVSystem, i);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!microgrid_available_for_islanding(sys, st.bus)) continue;
    const double cap = hacdcpf::model::effective_capacity_mw(st);
    if (cap > 0.0) add_source(c.sources, st.bus, cap,
                              ReliabilityComponentKind::ACStorage, i);
  }
  for (int i = 0; i < static_cast<int>(sys.mobile_storage.size()); ++i) {
    const auto& storage = sys.mobile_storage[i];
    const double cap = mobile_storage_discharge_capacity_mw(storage);
    if (cap > 0.0 && microgrid_available_for_islanding(sys, storage.bus))
      add_source(c.sources, storage.bus, cap,
                 ReliabilityComponentKind::MobileStorage, i);
  }
  // VPPs follow the repository-wide power-flow contract: each row is an
  // independent PCC boundary injection. Member devices are not expanded here
  // because the data model carries counts/capacity totals, not stable member
  // references suitable for outage attribution.
  for (int i = 0; i < static_cast<int>(sys.vpps.size()); ++i) {
    const auto& vpp = sys.vpps[i];
    const double cap = vpp_available_generation_mw(vpp);
    if (cap > 0.0)
      add_source(c.sources, vpp.pcc_bus, cap,
                 ReliabilityComponentKind::VirtualPowerPlant, i);
  }
  for (int i = 0; i < static_cast<int>(sys.dc.dc_static_generators.size()); ++i) {
    const auto& g = sys.dc.dc_static_generators[i];
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus + kDCBusOffset, cap,
                              ReliabilityComponentKind::DCStaticGenerator, i);
  }
  for (int i = 0; i < static_cast<int>(sys.dc.static_generators.size()); ++i) {
    const auto& g = sys.dc.static_generators[i];
    // Same scaling-aware treatment as the AC counterpart above.
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus + kDCBusOffset, cap,
                              ReliabilityComponentKind::DCStaticGeneratorAC, i);
  }
  for (int i = 0; i < static_cast<int>(sys.dc.pv_arrays.size()); ++i) {
    const auto& pv = sys.dc.pv_arrays[i];
    if (!pv.in_service) continue;
    add_source(c.sources, pv.bus + kDCBusOffset, pv.p_set_mw,
               ReliabilityComponentKind::DCPVArray, i);
  }
  for (int i = 0; i < static_cast<int>(sys.dc.storage.size()); ++i) {
    const auto& storage = sys.dc.storage[i];
    const double cap = hacdcpf::model::effective_capacity_mw(storage);
    if (cap > 0.0)
      add_source(c.sources, storage.bus + kDCBusOffset, cap,
                 ReliabilityComponentKind::DCStorage, i);
  }
  // A DC_V bus is a voltage reference, not an unbounded energy source. Its
  // active-power support must come from an explicit DC DER/storage device or
  // through a converter coupled to the AC balance below.
  for (int i = 0; i < static_cast<int>(sys.microgrids.size()); ++i) {
    const auto& microgrid = sys.microgrids[i];
    if (!microgrid.in_service || !microgrid.islanding_capability) continue;
    add_source(c.sources, microgrid.pcc_bus,
               residual_microgrid_capacity_mw(sys, microgrid),
               ReliabilityComponentKind::Microgrid, i);
  }

  int id = 1;
  for (int i = 0; i < static_cast<int>(sys.ac.branches.size()); ++i) {
    const auto& br = sys.ac.branches[i];
    add_bus(c.buses, br.from_bus);
    add_bus(c.buses, br.to_bus);
    if (!br.in_service) continue;  // out-of-service branches cannot fail
    // Stage-3 repair duration from this branch's MTTR (per-component); isolation
    // and switching keep the global defaults (no per-branch field exists).
    const double r_ac = br.mttr_hr > 1e-9 ? br.mttr_hr : kTauRepairHr;
    const double d3_ac = std::max(0.0, r_ac - kTauTrippingHr);
    c.faults.push_back({id++, true, i, br.from_bus, br.to_bus, br.in_service,
                        br.failure_rate > 0.0 ? br.failure_rate : kDefaultFailureRate,
                        kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_ac});
  }
  for (int i = 0; i < static_cast<int>(sys.dc.branches.size()); ++i) {
    const auto& br = sys.dc.branches[i];
    add_bus(c.buses, br.from_bus + kDCBusOffset);
    add_bus(c.buses, br.to_bus + kDCBusOffset);
    if (!br.in_service) continue;  // out-of-service branches cannot fail
    double lambda = br.mtbf_hours > 0.0 ? 8760.0 / br.mtbf_hours : kDefaultFailureRate;
    const double r_dc = br.mttr_hours > 1e-9 ? br.mttr_hours : kTauRepairHr;
    const double d3_dc = std::max(0.0, r_dc - kTauTrippingHr);
    // from_bus/to_bus stored without offset — informational only, not used in stage_components
    c.faults.push_back({id++, false, i, br.from_bus, br.to_bus, br.in_service, lambda,
                        kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_dc,
                        FaultKind::DCBranch});
  }
  // Generator forced-outage contingencies (opt-in).  lambda from FOR + MTTR.
  if (c.include_generator_faults) {
    const auto add_source_fault = [&](bool ac, int index, int bus, double lambda,
                                      double repair_hr, FaultKind kind) {
      const double repair = repair_hr > 1e-9 ? repair_hr : kTauRepairHr;
      c.faults.push_back({id++, ac, index, bus, bus, true,
                          lambda > 0.0 ? lambda : kDefaultFailureRate,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr,
                          std::max(0.0, repair - kTauTrippingHr), kind});
    };
    for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
      const auto& g = sys.ac.generators[i];
      if (!g.in_service) continue;
      if (hacdcpf::model::effective_capacity_mw(g) <= 1e-9) continue;
      add_bus(c.buses, g.bus);
      double lam = kDefaultFailureRate;
      const double f = g.forced_outage_rate;
      const double m = g.mttr_hr;
      if (f > 0.0 && f < 1.0 && m > 1e-9) lam = f / ((1.0 - f) * m) * 8760.0;
      const double r_g = m > 1e-9 ? m : kTauRepairHr;
      const double d3_g = std::max(0.0, r_g - kTauTrippingHr);
      c.faults.push_back({id++, true, i, g.bus, g.bus, true, lam,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_g,
                          FaultKind::Generator});
    }
    for (int i = 0; i < static_cast<int>(sys.ac.static_generators.size()); ++i) {
      const auto& source = sys.ac.static_generators[i];
      if (!source.in_service ||
          hacdcpf::model::effective_capacity_mw(source) <= 1e-9)
        continue;
      add_source_fault(true, i, source.bus,
                          source.mtbf_hours > 0.0
                              ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                          source.mttr_hours, FaultKind::StaticGenerator);
    }
    for (int i = 0; i < static_cast<int>(sys.ac.renewable_gens.size()); ++i) {
      const auto& source = sys.ac.renewable_gens[i];
      const double capacity = source.p_rated_mw > 0.0
          ? source.p_rated_mw * source.capacity_factor : source.p_mw;
      if (!source.in_service || capacity <= 1e-9) continue;
      add_source_fault(true, i, source.bus,
                          source.mtbf_hours > 0.0
                              ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                          source.mttr_hours, FaultKind::RenewableGenerator);
    }
    for (int i = 0; i < static_cast<int>(sys.ac.pv_systems.size()); ++i) {
      const auto& source = sys.ac.pv_systems[i];
      const double capacity = source.pmax_mw > 0.0 ? source.pmax_mw : source.p_mw;
      if (!source.in_service || capacity <= 1e-9) continue;
      add_source_fault(true, i, source.bus,
                          source.mtbf_hours > 0.0
                              ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                          source.mttr_hours, FaultKind::PVSystem);
    }
    for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
      const auto& source = sys.ac.storage[i];
      if (!source.in_service ||
          hacdcpf::model::effective_capacity_mw(source) <= 1e-9)
        continue;
      double lambda = kDefaultFailureRate;
      if (source.forced_outage_rate > 0.0 &&
          source.forced_outage_rate < 1.0 && source.mttr_hr > 1e-9) {
        lambda = source.forced_outage_rate /
                 ((1.0 - source.forced_outage_rate) * source.mttr_hr) * 8760.0;
      }
      add_source_fault(true, i, source.bus, lambda, source.mttr_hr,
                          FaultKind::Storage);
    }
    for (int i = 0; i < static_cast<int>(sys.mobile_storage.size()); ++i) {
      const auto& source = sys.mobile_storage[i];
      if (mobile_storage_discharge_capacity_mw(source) <= 1e-9) continue;
      add_source_fault(true, i, source.bus,
                       source.mtbf_hours > 0.0
                           ? 8760.0 / source.mtbf_hours
                           : kDefaultFailureRate,
                       source.mttr_hours, FaultKind::MobileStorage);
    }
    for (int i = 0; i < static_cast<int>(sys.vpps.size()); ++i) {
      const auto& source = sys.vpps[i];
      if (vpp_available_generation_mw(source) <= 1e-9) continue;
      add_source_fault(true, i, source.pcc_bus,
                       source.mtbf_hours > 0.0
                           ? 8760.0 / source.mtbf_hours
                           : kDefaultFailureRate,
                       source.mttr_hours, FaultKind::VirtualPowerPlant);
    }
    for (int i = 0; i < static_cast<int>(sys.microgrids.size()); ++i) {
      const auto& microgrid = sys.microgrids[i];
      if (!microgrid.in_service || !microgrid.islanding_capability ||
          microgrid_declared_capacity_mw(microgrid) <= 1e-9)
        continue;
      add_source_fault(true, i, microgrid.pcc_bus,
                          microgrid.mtbf_hours > 0.0
                              ? 8760.0 / microgrid.mtbf_hours
                              : kDefaultFailureRate,
                          microgrid.mttr_hours, FaultKind::Microgrid);
    }
    for (int i = 0; i < static_cast<int>(sys.dc.dc_static_generators.size()); ++i) {
      const auto& source = sys.dc.dc_static_generators[i];
      if (!source.in_service ||
          hacdcpf::model::effective_capacity_mw(source) <= 1e-9)
        continue;
      add_source_fault(false, i, source.bus,
                       source.mtbf_hours > 0.0
                           ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                       source.mttr_hours, FaultKind::DCStaticGenerator);
    }
    for (int i = 0; i < static_cast<int>(sys.dc.static_generators.size()); ++i) {
      const auto& source = sys.dc.static_generators[i];
      if (!source.in_service ||
          hacdcpf::model::effective_capacity_mw(source) <= 1e-9)
        continue;
      add_source_fault(false, i, source.bus,
                       source.mtbf_hours > 0.0
                           ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                       source.mttr_hours, FaultKind::DCStaticGeneratorAC);
    }
    for (int i = 0; i < static_cast<int>(sys.dc.pv_arrays.size()); ++i) {
      const auto& source = sys.dc.pv_arrays[i];
      if (!source.in_service || source.p_set_mw <= 1e-9) continue;
      add_source_fault(false, i, source.bus,
                       source.mtbf_hours > 0.0
                           ? 8760.0 / source.mtbf_hours : kDefaultFailureRate,
                       source.mttr_hours, FaultKind::DCPVArray);
    }
    for (int i = 0; i < static_cast<int>(sys.dc.storage.size()); ++i) {
      const auto& storage = sys.dc.storage[i];
      if (!storage.in_service ||
          hacdcpf::model::effective_capacity_mw(storage) <= 1e-9)
        continue;
      double lambda = kDefaultFailureRate;
      if (storage.forced_outage_rate > 0.0 &&
          storage.forced_outage_rate < 1.0 && storage.mttr_hr > 1e-9) {
        lambda = storage.forced_outage_rate /
                 ((1.0 - storage.forced_outage_rate) * storage.mttr_hr) *
                 8760.0;
      }
      add_source_fault(false, i, storage.bus, lambda, storage.mttr_hr,
                       FaultKind::DCStorage);
    }
  }
  // 2-winding transformer outage contingencies (opt-in; modelled as edges).
  if (c.include_transformer_faults) {
    for (int i = 0; i < static_cast<int>(sys.ac.transformers_2w.size()); ++i) {
      const auto& t = sys.ac.transformers_2w[i];
      add_bus(c.buses, t.hv_bus);
      add_bus(c.buses, t.lv_bus);
      if (!t.in_service) continue;
      double lam = t.mtbf_hours > 0.0 ? 8760.0 / t.mtbf_hours : kDefaultFailureRate;
      const double r_t = t.mttr_hours > 1e-9 ? t.mttr_hours : kTauRepairHr;
      const double d3_t = std::max(0.0, r_t - kTauTrippingHr);
      c.faults.push_back({id++, true, i, t.hv_bus, t.lv_bus, true, lam,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_t,
                          FaultKind::Transformer2W});
    }
  }
  // VSC / DC-DC converter outage contingencies (opt-in). A faulted converter
  // removes its coupled transfer variables from the joint AC/DC stage MILP.
  if (c.include_converter_faults) {
    for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()); ++i) {
      const auto& v = sys.vsc_converters[i];
      if (!v.in_service) continue;
      double lam = kDefaultFailureRate;
      if (v.forced_outage_rate > 0.0 && v.forced_outage_rate < 1.0 && v.mttr_hr > 1e-9)
        lam = v.forced_outage_rate / ((1.0 - v.forced_outage_rate) * v.mttr_hr) * 8760.0;
      const double r_v = v.mttr_hr > 1e-9 ? v.mttr_hr : kTauRepairHr;
      const double d3_v = std::max(0.0, r_v - kTauTrippingHr);
      c.faults.push_back({id++, true, i, v.bus_ac, v.bus_dc, true, lam,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_v,
                          FaultKind::VSCConverter});
    }
    for (int i = 0; i < static_cast<int>(sys.dc.dcdc_converters.size()); ++i) {
      const auto& d = sys.dc.dcdc_converters[i];
      if (!d.in_service) continue;
      double lam = d.mtbf_hours > 0.0 ? 8760.0 / d.mtbf_hours : kDefaultFailureRate;
      const double r_d = d.mttr_hours > 1e-9 ? d.mttr_hours : kTauRepairHr;
      const double d3_d = std::max(0.0, r_d - kTauTrippingHr);
      c.faults.push_back({id++, false, i, d.bus_in, d.bus_out, true, lam,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_d,
                          FaultKind::DCDCConverter});
    }
  }
  // AC switch / AC & DC circuit-breaker outage contingencies (opt-in).
  if (c.include_switch_faults) {
    for (int i = 0; i < static_cast<int>(sys.ac.switches.size()); ++i) {
      const auto& sw = sys.ac.switches[i];
      if (!sw.in_service) continue;
      double lam = sw.mtbf_hours > 0.0 ? 8760.0 / sw.mtbf_hours : kDefaultFailureRate;
      const double r_s = sw.mttr_hours > 1e-9 ? sw.mttr_hours : kTauRepairHr;
      const double d3_s = std::max(0.0, r_s - kTauTrippingHr);
      c.faults.push_back({id++, true, i, sw.bus_from, sw.bus_to, true, lam,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_s,
                          FaultKind::ACSwitch});
    }
    for (int i = 0; i < static_cast<int>(sys.ac.circuit_breakers.size()); ++i) {
      const auto& cb = sys.ac.circuit_breakers[i];
      if (!cb.in_service) continue;
      const double d3_c = std::max(0.0, kTauRepairHr - kTauTrippingHr);
      c.faults.push_back({id++, true, i, cb.bus_from, cb.bus_to, true, kDefaultFailureRate,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_c,
                          FaultKind::ACCircuitBreaker});
    }
    for (int i = 0; i < static_cast<int>(sys.dc.dc_circuit_breakers.size()); ++i) {
      const auto& cb = sys.dc.dc_circuit_breakers[i];
      if (!cb.in_service) continue;
      const double d3_c = std::max(0.0, kTauRepairHr - kTauTrippingHr);
      c.faults.push_back({id++, false, i, cb.bus_from, cb.bus_to, true, kDefaultFailureRate,
                          kTauSwitchHr, kTauTrippingHr - kTauSwitchHr, d3_c,
                          FaultKind::DCCircuitBreaker});
    }
  }

  std::sort(c.buses.begin(), c.buses.end());
  c.buses.erase(std::unique(c.buses.begin(), c.buses.end()), c.buses.end());
  apply_protection_configuration(c, options.reliability_configuration);
  return c;
}

struct DSU {
  std::vector<int> p;
  explicit DSU(int n) : p(n) { std::iota(p.begin(), p.end(), 0); }
  int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
  void unite(int a, int b) {
    a = find(a); b = find(b);
    if (a != b) p[b] = a;
  }
};

// ─── LinDistFlow MILP — stages 1, 2, 3 ───────────────────────────────────────
//
// Implements the exact mathematical model:
//
//   Objective (eq. 3):   min Σ_i w_i · p^sh_i
//
//   C0  (eq. 3a):  Energisation/load-pickup coupling:
//                    P^d_i - p^sh_i ≤ P^d_i y_i, y_s = 1 for source buses
//
//   C1  (eq. 4'):  LinDistFlow active power balance at every AC bus i:
//                    Σ_{j:(j,i)∈L} P_ji - Σ_{j:(i,j)∈L} P_ij + p_g,i + p^sh_i = P^d_i
//                    0 ≤ p_g,i ≤ P^g,max_i
//
//   C2  (eq. 5):   Reactive power balance (q^sh_i = q_d,i · p^sh_i / p_d,i):
//                    Σ_{j:(j,i)∈L} Q_ji - Σ_{j:(i,j)∈L} Q_ij + q_g,i + q^sh_i = Q^d_i
//                    0 ≤ q_g,i ≤ Q^g,max_i
//
//   C3  (eq. 6'):  LinDistFlow voltage drop (linearised, losses dropped):
//                    v_j = v_i - 2(r_ij P_ij + x_ij Q_ij)
//                  Enforced via Big-M on z_ij:
//                    v_j - v_i + 2(r P + x Q) ≤  M(1 - z_ij)
//                    v_j - v_i + 2(r P + x Q) ≥ -M(1 - z_ij)
//
//   C4  (eq. 7–8): Branch active/reactive flow limits with Big-M:
//                    -z_ij · S̄_ij ≤ P_ij ≤ z_ij · S̄_ij
//                    -z_ij · S̄_ij ≤ Q_ij ≤ z_ij · S̄_ij
//
//   C5  (eq. 9):   Voltage bounds:   V̲² ≤ v_i ≤ V̄²
//
//   C6  (eq. 10–11): Strict energized radial forest:
//                    Σ f_in - Σ f_out = y_i, ∀i∉S
//                    Σ f_in - Σ f_out ≤ 0, ∀i∈S
//                    |f_ij| ≤ (|B|-1) z_ij, z_ij ≤ y_i, z_ij ≤ y_j
//                    Σ z_ij ≤ Σ y_i - |S|
//
//   C7  (eq. 12):  Forced-open failed branch:   z_k = 0
//
//   C8  (eq. 13):  Normally-closed non-switch branches stay closed:
//                    z_ij = 1  ∀(i,j) ∉ (L^NO ∪ {k})
//                  Stage 1: all switches remain at nominal position (no switching allowed).
//                  Stage 2: normally-open switches are free (0 ≤ z_ij ≤ 1, binary) subject to C9.
//                  Stage 3: repair window — the fault branch stays FORCED-OPEN (z_k = 0,
//                           it is repaired only at τ_RP) and the normally-open ties remain
//                           free (the Stage-2 reconfiguration is held), subject to C9.  This
//                           is topologically identical to Stage 2; only the duration differs
//                           (τ_rep ≈ MTTR).  (F7 fix — Stage 3 previously restored the fault
//                           and re-opened the ties, so unrestorable load was charged only the
//                           brief switching window instead of the whole repair window.)
//
//   C9  (eq. 14):  Switch count:   Σ_{(i,j)∈L^NO} z_ij ≤ K^sw
//
//   C10 (eq. 15):  Load shed bounds:   0 ≤ p^sh_i ≤ p_d,i
//
// MODEL SCOPE: coupled AC/DC radial restoration. AC and DC networks have
// domain-qualified bus maps and separate voltage/flow equations; VSC and DC-DC
// converters couple their nodal balances with one-way efficiency terms.
// ─────────────────────────────────────────────────────────────────────────────

// Big-M constant for voltage-drop linearisation (in pu²).
// The voltage span is at most [V̲², V̄²] ≈ [0.81, 1.21]; 2r·P + 2x·Q is
// bounded in magnitude by 2·(r+x)·S̄.  A value of 2.0 pu² is conservative
// enough for any realistic distribution system.
constexpr double kBigMVoltage = 2.0;

struct StageSolve {
  double shed_kw{0.0};
  std::vector<double> shed_by_load;
  std::string status{"unknown"};
  double objective{0.0};
  double mip_gap{0.0};
  bool proven_optimal{true};
  std::vector<int> closed_tie_switch_indices;
  std::vector<int> closed_legacy_branch_indices;
  bool switch_sequence_valid{true};
  std::string switch_sequence_message{"no switching required"};
  std::vector<double> storage_discharge_mw;
  std::vector<double> storage_energy_used_mwh;
  std::vector<double> vsc_dispatch_kw;
  std::vector<double> dcdc_dispatch_kw;
};

bool eligible_restoration_tie(const Switch& sw) {
  if (!sw.in_service || sw.closed || sw.locked_open) return false;
  const auto caps = effective_switch_capabilities(sw);
  if (!caps.can_close_for_restoration || sw.switch_type == SwitchType::Fuse)
    return false;
  return !sw.capabilities_explicit || sw.role == SwitchRole::Tie;
}

struct ProtectionInterlockPrecheck {
  bool valid{true};
  bool explicit_plan{false};
  std::string message{"fault isolation represented by forced-open failed component"};
  std::unordered_set<int> forced_open_switch_indices;
};

ProtectionInterlockPrecheck precheck_protection_interlocks(
    const HybridPowerSystem& sys, const FaultLine& fault) {
  ProtectionInterlockPrecheck out;
  std::string controlled_type;
  int controlled_index = -1;
  if (fault.kind == FaultKind::ACBranch && fault.index >= 0 &&
      fault.index < static_cast<int>(sys.ac.branches.size())) {
    controlled_type = "ac_branch";
    controlled_index = sys.ac.branches[fault.index].index;
  } else if (fault.kind == FaultKind::Transformer2W && fault.index >= 0 &&
             fault.index < static_cast<int>(sys.ac.transformers_2w.size())) {
    controlled_type = "transformer_2w";
    controlled_index = sys.ac.transformers_2w[fault.index].index;
  } else {
    return out;
  }

  std::unordered_map<int, const Switch*> switch_by_index;
  std::vector<const Switch*> protection;
  std::vector<const Switch*> boundaries;
  for (const auto& sw : sys.ac.switches) {
    switch_by_index[sw.index] = &sw;
    const bool generic_match =
        sw.controlled_element_type == controlled_type &&
        sw.controlled_element_index == controlled_index;
    const bool legacy_match = controlled_type == "ac_branch" &&
        sw.controlled_branch_index == controlled_index;
    if (!sw.in_service || !sw.closed || (!generic_match && !legacy_match))
      continue;
    if (sw.role == SwitchRole::Protection) protection.push_back(&sw);
    else if (sw.role == SwitchRole::Sectionalizing ||
             sw.role == SwitchRole::Isolation)
      boundaries.push_back(&sw);
  }
  out.explicit_plan = !protection.empty() || !boundaries.empty();
  const auto add_protection = [&](const Switch* sw) {
    if (std::none_of(protection.begin(), protection.end(),
                     [&](const Switch* existing) {
                       return existing->index == sw->index;
                     }))
      protection.push_back(sw);
  };
  std::unordered_map<int, int> boundary_upstream;
  for (const Switch* boundary : boundaries) {
    if (!effective_switch_capabilities(*boundary)
             .requires_deenergized_operation)
      continue;
    int upstream_index = boundary->upstream_protective_switch_index;
    if (upstream_index < 0)
      upstream_index = boundary->sectionalizer_protection.upstream_switch_index;
    if (upstream_index < 0) continue;
    boundary_upstream[boundary->index] = upstream_index;
    const auto upstream = switch_by_index.find(upstream_index);
    if (upstream != switch_by_index.end() && upstream->second->in_service &&
        upstream->second->closed)
      add_protection(upstream->second);
  }

  std::unordered_set<int> valid_protection;
  for (const Switch* sw : protection) {
    const auto caps = effective_switch_capabilities(*sw);
    const bool valid = !sw->locked_closed &&
        sw->role == SwitchRole::Protection &&
        caps.can_interrupt_fault_current;
    out.valid = out.valid && valid;
    if (valid) {
      valid_protection.insert(sw->index);
      out.forced_open_switch_indices.insert(sw->index);
    }
  }
  for (const Switch* boundary : boundaries) {
    const auto caps = effective_switch_capabilities(*boundary);
    const auto dependency = boundary_upstream.find(boundary->index);
    const bool upstream_cleared = dependency == boundary_upstream.end()
        ? !valid_protection.empty()
        : valid_protection.count(dependency->second) > 0;
    const bool valid = !boundary->locked_closed &&
        (caps.can_interrupt_load_current ||
         (caps.requires_deenergized_operation && upstream_cleared));
    out.valid = out.valid && valid;
    if (valid) out.forced_open_switch_indices.insert(boundary->index);
  }
  if (out.explicit_plan) {
    out.message = out.valid
        ? "protection and interlock precheck passed; forced-open states applied to restoration MILP"
        : "protection or interlock precheck failed; restoration switching is blocked";
  }
  return out;
}

double vsc_transfer_capacity_kw(const VSCConverter& vsc) {
  const double capacity_mw = std::max({std::abs(vsc.pmax_mw),
                                       std::abs(vsc.pmin_mw),
                                       std::abs(vsc.p_set_mw),
                                       vsc.p_rated_mw});
  if (!std::isfinite(capacity_mw) || capacity_mw <= 0.0) return 0.0;
  // IEC 62747:2014: nameplate P bounds the converter input/terminal transfer;
  // efficiency is applied once in the opposite-port balance, not to the bound.
  return capacity_mw * 1000.0;
}

// solve_stage_milp() — LinDistFlow MILP for stages 1, 2, and 3.
//
// The MILP jointly represents the AC and DC sub-networks. The two domains keep
// independent bus index maps even when their stable bus IDs are numerically
// equal. VSC and DC-DC transfer variables couple the domain-specific balances.
StageSolve solve_stage_milp(const NativeCase& c, const FaultLine& fault, int stage,
                            int max_sw_ops = INT_MAX,
                            const std::unordered_set<int>* unavailable_ties = nullptr,
                            const std::unordered_set<int>* forced_open_switches = nullptr,
                            const StageSolve* held_stage2_plan = nullptr,
                            bool fixed_topology = false,
                            const std::vector<double>* storage_energy_available_mwh = nullptr,
                            double stage_duration_hr = 0.0) {
  StageSolve out;
  const int nd_total = static_cast<int>(c.loads.size());
  out.shed_by_load.assign(c.loads.size(), 0.0);
  // Public result/chronology ordering is stable within a solve:
  // [AC Storage | MobileStorage | DC Storage].
  const size_t storage_count = c.sys.ac.storage.size() +
      c.sys.mobile_storage.size() + c.sys.dc.storage.size();
  out.storage_discharge_mw.assign(storage_count, 0.0);
  out.storage_energy_used_mwh.assign(storage_count, 0.0);
  out.vsc_dispatch_kw.assign(c.sys.vsc_converters.size(), 0.0);
  out.dcdc_dispatch_kw.assign(c.sys.dc.dcdc_converters.size(), 0.0);
  if (nd_total == 0) {
    out.status = "success";
    return out;
  }

  // ── Build index maps for the AC sub-network ──────────────────────────────
  const HybridPowerSystem& sys = c.sys;

  // AC buses: map bus_id → position in ac_buses vector (0-based)
  const int n_bus = static_cast<int>(sys.ac.buses.size());
  std::unordered_map<int, int> ac_bus_pos;
  ac_bus_pos.reserve(static_cast<size_t>(n_bus));
  for (int i = 0; i < n_bus; ++i) ac_bus_pos[sys.ac.buses[i].index] = i;

  // AC candidate edges: all physical branches plus standalone switch elements.
  // Closed/in-service edges are nominally closed.  Out-of-service branches and
  // open switches are normally-open candidates that may close only in Stage 2.
  struct BrInfo {
    int idx_global;   // index in sys.ac.branches
    int from_pos;     // position in ac_buses
    int to_pos;
    double r_pu;
    double x_pu;
    double s_max_mw;  // thermal limit in MW (= rate_a_mva, or default)
    bool normally_open;  // true if this is a normally-open switch
    bool failed;         // true if this is the faulted branch
    int switch_index;    // Switch::index of the controlling tie (-1 if none)
    int transformer_index{-1};  // sys.ac.transformers_2w index (-1 if not a transformer)
    bool restoration_eligible{true};
  };
  // Default branch rating: 2× total system demand (ensures feasibility when
  // no explicit rating is given, without making Big-M constraints too loose).
  double total_demand_mw = 0.0;
  for (const auto& ld : c.loads) total_demand_mw += std::max(0.0, ld.p_kw) / 1000.0;
  const double default_rate_mw = std::max(10.0, 2.0 * total_demand_mw);

  std::vector<BrInfo> ac_branches;
  ac_branches.reserve(sys.ac.branches.size() + sys.ac.switches.size());
  auto undirected_key = [](int a, int b) -> long long {
    if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) ^ static_cast<unsigned int>(b);
  };
  std::unordered_set<long long> forced_open_pairs;
  if (forced_open_switches) {
    for (const auto& sw : sys.ac.switches) {
      if (forced_open_switches->count(sw.index) > 0)
        forced_open_pairs.insert(undirected_key(sw.bus_from, sw.bus_to));
    }
  }
  std::unordered_map<long long, bool> branch_pair_seen;
  for (int b = 0; b < static_cast<int>(sys.ac.branches.size()); ++b) {
    const auto& br = sys.ac.branches[b];
    auto it_f = ac_bus_pos.find(br.from_bus);
    auto it_t = ac_bus_pos.find(br.to_bus);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
    branch_pair_seen[undirected_key(br.from_bus, br.to_bus)] = true;

    bool has_open_switch = false;
    int open_switch_index = -1;
    bool open_switch_eligible = true;
    for (const auto& sw : sys.ac.switches) {
      if (!sw.in_service) continue;
      if (!sw.closed &&
          ((sw.bus_from == br.from_bus && sw.bus_to == br.to_bus) ||
           (sw.bus_from == br.to_bus   && sw.bus_to == br.from_bus))) {
        has_open_switch = true;
        open_switch_index = sw.index;
        open_switch_eligible = eligible_restoration_tie(sw);
        break;
      }
    }
    const bool is_no_switch = !br.in_service || has_open_switch;
    const bool is_failed =
        component_outaged(fault, ReliabilityComponentKind::ACBranch, b) ||
        forced_open_pairs.count(undirected_key(br.from_bus, br.to_bus)) > 0;
    const double s_max = br.rate_a_mva > 1e-9 ? br.rate_a_mva : default_rate_mw;
    ac_branches.push_back({b, it_f->second, it_t->second,
                           std::max(1e-6, br.r_pu),
                           std::max(1e-6, br.x_pu),
                           s_max,
                           is_no_switch,
                           is_failed,
                           open_switch_index,
                           -1,
                           !has_open_switch || open_switch_eligible});
  }
  for (int si = 0; si < static_cast<int>(sys.ac.switches.size()); ++si) {
    const auto& sw = sys.ac.switches[si];
    if (!sw.in_service) continue;
    if (branch_pair_seen.count(undirected_key(sw.bus_from, sw.bus_to))) continue;
    auto it_f = ac_bus_pos.find(sw.bus_from);
    auto it_t = ac_bus_pos.find(sw.bus_to);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
    bool controlled_branch_fault = false;
    for (int branch_pos = 0;
         branch_pos < static_cast<int>(sys.ac.branches.size()); ++branch_pos) {
      const auto& controlled = sys.ac.branches[branch_pos];
      if (component_outaged(fault, ReliabilityComponentKind::ACBranch,
                            branch_pos) &&
          ((sw.controlled_element_type == "ac_branch" &&
            sw.controlled_element_index == controlled.index) ||
           sw.controlled_branch_index == controlled.index)) {
        controlled_branch_fault = true;
        break;
      }
    }
    bool controlled_transformer_fault = false;
    for (int transformer_pos = 0;
         transformer_pos < static_cast<int>(sys.ac.transformers_2w.size());
         ++transformer_pos) {
      const auto& controlled = sys.ac.transformers_2w[transformer_pos];
      if (component_outaged(fault,
                            ReliabilityComponentKind::ACTransformer2W,
                            transformer_pos) &&
          sw.controlled_element_type == "transformer_2w" &&
          sw.controlled_element_index == controlled.index) {
        controlled_transformer_fault = true;
        break;
      }
    }
    const bool sw_failed =
        component_outaged(fault, ReliabilityComponentKind::ACSwitch, si) ||
        controlled_branch_fault || controlled_transformer_fault ||
        (forced_open_switches &&
         forced_open_switches->count(sw.index) > 0);
    ac_branches.push_back({-1, it_f->second, it_t->second,
                           1e-6, 1e-6, default_rate_mw,
                           !sw.closed,
                           sw_failed,
                           sw.index,
                           -1,
                           sw.closed || eligible_restoration_tie(sw)});
    branch_pair_seen[undirected_key(sw.bus_from, sw.bus_to)] = true;
  }
  // AC circuit breakers are always topology edges; include_switch_faults only
  // controls whether their own outages enter the initiating contingency set.
  // deduped against branch/switch pairs and forced open when faulted.
  {
    for (int ci = 0; ci < static_cast<int>(sys.ac.circuit_breakers.size()); ++ci) {
      const auto& cb = sys.ac.circuit_breakers[ci];
      if (!cb.in_service) continue;
      if (branch_pair_seen.count(undirected_key(cb.bus_from, cb.bus_to))) continue;
      auto it_f = ac_bus_pos.find(cb.bus_from);
      auto it_t = ac_bus_pos.find(cb.bus_to);
      if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
      const bool cb_failed = component_outaged(
          fault, ReliabilityComponentKind::ACCircuitBreaker, ci);
      ac_branches.push_back({-1, it_f->second, it_t->second,
                             1e-6, 1e-6, default_rate_mw,
                             !cb.closed, cb_failed, -1});
      branch_pair_seen[undirected_key(cb.bus_from, cb.bus_to)] = true;
    }
  }
  // Transformers are always part of the healthy network topology. The option
  // include_transformer_faults controls contingency enumeration only; omitting
  // the physical edge when that option is false disconnects every LV load and
  // makes all upstream branch contingencies appear to shed the same total load.
  for (int t = 0; t < static_cast<int>(sys.ac.transformers_2w.size()); ++t) {
    const auto& tr = sys.ac.transformers_2w[t];
    if (!tr.in_service) continue;
    auto it_f = ac_bus_pos.find(tr.hv_bus);
    auto it_t = ac_bus_pos.find(tr.lv_bus);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
    const bool tf_failed = component_outaged(
        fault, ReliabilityComponentKind::ACTransformer2W, t);
    const double s_max = tr.sn_mva > 1e-9 ? tr.sn_mva : default_rate_mw;
    ac_branches.push_back({-1, it_f->second, it_t->second,
                           1e-6, 1e-6, s_max,
                           false, tf_failed, -1, t});
  }
  const int n_br = static_cast<int>(ac_branches.size());

  // ── Identify voltage anchors (S set in radiality constraint C6) ─────────
  // Only an upstream grid, a synchronous source, a grid-forming inverter, or
  // an explicitly island-capable microgrid aggregate may root an energized
  // component. Grid-following DER can inject only after such a root exists.
  std::vector<bool> is_source(static_cast<size_t>(n_bus), false);
  // Injection capability and voltage-reference duty are distinct. Grid-
  // following/local DER can supply an island without forcing its terminal to
  // exactly 1.0 pu when it remains connected to the upstream slack.
  std::vector<bool> is_voltage_anchor(static_cast<size_t>(n_bus), false);
  // p_gen[i]: maximum active injection in MW at AC bus i (sum over all generators)
  std::vector<double> p_gen_max(static_cast<size_t>(n_bus), 0.0);
  // q_gen[i]: maximum reactive injection in Mvar (used in reactive balance C2)
  std::vector<double> q_gen_min(static_cast<size_t>(n_bus), 0.0);
  std::vector<double> q_gen_max(static_cast<size_t>(n_bus), 0.0);
  const int n_ac_storage = static_cast<int>(sys.ac.storage.size());
  const int n_mobile_storage = static_cast<int>(sys.mobile_storage.size());
  const int n_storage = n_ac_storage + n_mobile_storage;
  std::vector<int> storage_bus_pos(static_cast<size_t>(n_storage), -1);
  std::vector<double> storage_pmax_mw(static_cast<size_t>(n_storage), 0.0);

  auto add_dispatch = [&](int bus, double p_mw) {
    auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) return;
    p_gen_max[it->second] += std::max(0.0, p_mw);
  };
  auto mark_anchor = [&](int bus, bool fixes_voltage = true) {
    auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) return;
    is_source[it->second] = true;
    if (fixes_voltage) is_voltage_anchor[it->second] = true;
  };
  const auto add_reactive = [&](int bus, double qmin_mvar,
                                double qmax_mvar) {
    const auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) return;
    q_gen_min[static_cast<size_t>(it->second)] += std::min(0.0, qmin_mvar);
    q_gen_max[static_cast<size_t>(it->second)] += std::max(0.0, qmax_mvar);
  };
  for (int i = 0; i < static_cast<int>(sys.ac.external_grids.size()); ++i) {
    const auto& eg = sys.ac.external_grids[i];
    if (!eg.in_service) continue;
    if (component_outaged(fault, ReliabilityComponentKind::ExternalGrid, i))
      continue;
    const double cap = eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : 1.0e4;
    add_dispatch(eg.bus, cap);
    mark_anchor(eg.bus);
    auto it = ac_bus_pos.find(eg.bus);
    if (it != ac_bus_pos.end()) {
      add_reactive(eg.bus, -cap, cap);
    }
  }
  for (int gi = 0; gi < static_cast<int>(sys.ac.generators.size()); ++gi) {
    const auto& g = sys.ac.generators[gi];
    if (!g.in_service) continue;
    // Generator forced outage: the unit is out for the whole event and is only
    // repaired at the end of the repair window (tau_RP).  Stage 3 [tau_TP, tau_RP]
    // IS that repair window, so the faulted unit stays out in all three stages
    // (F7: Stage 3 is "during repair", not "after repair").
    if (component_outaged(fault, ReliabilityComponentKind::ACGenerator, gi))
      continue;
    add_dispatch(g.bus, g.pmax_mw > 0.0 ? g.pmax_mw : g.pg_mw);
    // The Generator model represents a synchronous/voltage-controlled source.
    // Inverter-based DER use StaticGenerator/RenewableGen/PVSystem instead.
    mark_anchor(g.bus, g.is_slack);
    auto it = ac_bus_pos.find(g.bus);
    if (it != ac_bus_pos.end()) {
      const double qcap = g.qmax_mvar > 0.0
          ? g.qmax_mvar : std::abs(g.qg_mvar);
      add_reactive(g.bus, g.qmin_mvar < 0.0 ? g.qmin_mvar : -qcap, qcap);
    }
  }
  for (int i = 0; i < static_cast<int>(sys.ac.static_generators.size()); ++i) {
    const auto& sg = sys.ac.static_generators[i];
    if (!sg.in_service) continue;
    if (component_outaged(fault, ReliabilityComponentKind::ACStaticGenerator,
                          i))
      continue;
    if (!microgrid_available_for_islanding(sys, sg.bus, &fault)) continue;
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    add_dispatch(sg.bus, cap);
    if (sg.grid_forming && cap > 1e-9) mark_anchor(sg.bus, false);
    add_reactive(sg.bus, sg.qmin_mvar, sg.qmax_mvar);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.renewable_gens.size()); ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    if (!rg.in_service) continue;
    if (component_outaged(fault,
                          ReliabilityComponentKind::ACRenewableGenerator, i))
      continue;
    if (!microgrid_available_for_islanding(sys, rg.bus, &fault)) continue;
    const double cap = rg.p_rated_mw > 0.0 ? rg.p_rated_mw * rg.capacity_factor : rg.p_mw;
    add_dispatch(rg.bus, cap);
    add_reactive(rg.bus, rg.qmin_mvar, rg.qmax_mvar);
    if (rg.grid_forming && cap > 1e-9) mark_anchor(rg.bus, false);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.pv_systems.size()); ++i) {
    const auto& pv = sys.ac.pv_systems[i];
    if (!pv.in_service) continue;
    if (component_outaged(fault, ReliabilityComponentKind::ACPVSystem, i))
      continue;
    if (!microgrid_available_for_islanding(sys, pv.bus, &fault)) continue;
    const double cap = pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw;
    add_dispatch(pv.bus, cap);
    add_reactive(pv.bus, pv.qmin_mvar, pv.qmax_mvar);
    if (pv.grid_forming && cap > 1e-9) mark_anchor(pv.bus, false);
  }
  for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!st.in_service) continue;
    if (component_outaged(fault, ReliabilityComponentKind::ACStorage, i))
      continue;
    if (!microgrid_available_for_islanding(sys, st.bus, &fault)) continue;
    const auto bus = ac_bus_pos.find(st.bus);
    if (bus == ac_bus_pos.end()) continue;
    const double available = storage_energy_available_mwh &&
                                     i < static_cast<int>(storage_energy_available_mwh->size())
        ? std::max(0.0, (*storage_energy_available_mwh)[static_cast<size_t>(i)])
        : initial_storage_deliverable_energy_mwh(st);
    double cap = std::max(0.0, hacdcpf::model::effective_capacity_mw(st));
    if (stage_duration_hr > 1e-12)
      cap = std::min(cap, available / stage_duration_hr);
    if (available <= 1e-12) cap = 0.0;
    storage_bus_pos[static_cast<size_t>(i)] = bus->second;
    storage_pmax_mw[static_cast<size_t>(i)] = cap;
    add_reactive(st.bus, std::max(-cap, st.qmin_mvar),
                 std::min(cap, std::max(0.0, st.qmax_mvar)));
    if (st.grid_forming && cap > 1e-9) mark_anchor(st.bus, false);
  }
  for (int i = 0; i < n_mobile_storage; ++i) {
    const auto& st = sys.mobile_storage[static_cast<size_t>(i)];
    if (!st.in_service || st.status == MobileStorageStatus::InTransit ||
        component_outaged(fault, ReliabilityComponentKind::MobileStorage, i) ||
        !microgrid_available_for_islanding(sys, st.bus, &fault))
      continue;
    const auto bus = ac_bus_pos.find(st.bus);
    if (bus == ac_bus_pos.end()) continue;
    const size_t energy_pos = static_cast<size_t>(n_ac_storage + i);
    const double available = storage_energy_available_mwh &&
                                     energy_pos < storage_energy_available_mwh->size()
        ? std::max(0.0, (*storage_energy_available_mwh)[energy_pos])
        : initial_storage_deliverable_energy_mwh(st);
    double cap = mobile_storage_discharge_capacity_mw(st);
    if (stage_duration_hr > 1e-12)
      cap = std::min(cap, available / stage_duration_hr);
    if (available <= 1e-12) cap = 0.0;
    const size_t storage_pos = static_cast<size_t>(n_ac_storage + i);
    storage_bus_pos[storage_pos] = bus->second;
    storage_pmax_mw[storage_pos] = cap;
    add_reactive(st.bus, std::max(-cap, st.qmin_mvar),
                 std::min(cap, std::max(0.0, st.qmax_mvar)));
    if (st.grid_forming && cap > 1e-9) mark_anchor(st.bus, false);
  }
  for (int i = 0; i < static_cast<int>(sys.vpps.size()); ++i) {
    const auto& vpp = sys.vpps[static_cast<size_t>(i)];
    if (component_outaged(fault, ReliabilityComponentKind::VirtualPowerPlant,
                          i))
      continue;
    const double cap = vpp_available_generation_mw(vpp);
    if (cap <= 1e-9) continue;
    add_dispatch(vpp.pcc_bus, cap);
    // VirtualPowerPlant exposes an operating Q point but no Q capability
    // curve. During restoration it may be curtailed toward zero, never beyond
    // the submitted operating point.
    add_reactive(vpp.pcc_bus, std::min(0.0, vpp.q_output_mvar),
                 std::max(0.0, vpp.q_output_mvar));
  }
  for (int i = 0; i < static_cast<int>(sys.microgrids.size()); ++i) {
    const auto& microgrid = sys.microgrids[i];
    if (!microgrid.in_service || !microgrid.islanding_capability ||
        component_outaged(fault, ReliabilityComponentKind::Microgrid, i))
      continue;
    const double residual = residual_microgrid_capacity_mw(sys, microgrid);
    if (residual <= 1e-9) continue;
    add_dispatch(microgrid.pcc_bus, residual);
    mark_anchor(microgrid.pcc_bus, false);
    add_reactive(microgrid.pcc_bus, -0.5 * residual, 0.5 * residual);
  }
  for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()); ++i) {
    const auto& converter = sys.vsc_converters[i];
    if (!converter.in_service ||
        component_outaged(fault, ReliabilityComponentKind::VSCConverter, i))
      continue;
    const auto role = resolve_device_control_role(converter);
    if (role.provides_ac_angle_reference)
      mark_anchor(converter.bus_ac, true);
  }

  // ── Build per-bus demand vectors (MW and Mvar) for AC loads ────────────
  // p_d[i]: net active demand at AC bus i (MW)
  // q_d[i]: net reactive demand (Mvar); approximated as p_d * tan(arccos(0.9))
  // Demand is sourced only from c.loads. build_native_case() already expands
  // both ACLoad entries and ACBus::pd_mw into explicit LoadPoint records, so
  // adding sys.ac.buses[i].pd_mw here would double-count bus-level demand.
  std::vector<double> p_d(static_cast<size_t>(n_bus), 0.0);
  std::vector<double> q_d(static_cast<size_t>(n_bus), 0.0);
  // Map from load-point index (in c.loads) → AC bus position (-1 if DC-only)
  std::vector<int> load_ac_bus(static_cast<size_t>(nd_total), -1);

  for (int li = 0; li < nd_total; ++li) {
    const int bus = c.loads[li].bus;
    auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) continue;  // DC-only load — handled below
    load_ac_bus[li] = it->second;
    if (load_outaged(fault, c.loads[li])) continue;
    const double p_mw = std::max(0.0, c.loads[li].p_kw) / 1000.0;
    p_d[it->second] += p_mw;
    // F14: use the load's measured reactive demand when provided; otherwise fall
    // back to a uniform 0.9-PF reconstruction (tan(arccos(0.9)) approx 0.4843).
    const double q_mvar = c.loads[li].q_kvar / 1000.0;
    q_d[it->second] += (std::abs(q_mvar) > 1e-9) ? q_mvar : p_mw * 0.4843;
  }
  // DC load points retain -1 here and enter the separate coupled DC balance.

  // ── Coupled DC network and converter data ───────────────────────────────
  const int n_dc_bus = c.include_dc_power_flow
      ? static_cast<int>(sys.dc.buses.size()) : 0;
  std::unordered_map<int, int> dc_bus_pos;
  dc_bus_pos.reserve(static_cast<size_t>(n_dc_bus));
  for (int i = 0; i < n_dc_bus; ++i)
    dc_bus_pos[sys.dc.buses[static_cast<size_t>(i)].index] = i;

  struct DCEdgeInfo {
    int from_pos{-1};
    int to_pos{-1};
    double r_pu{0.0};
    double pmax_mw{0.0};
    bool failed{false};
    bool radial_representative{true};
    std::string name;
  };
  std::vector<DCEdgeInfo> dc_edges;
  dc_edges.reserve(sys.dc.branches.size() + sys.dc.dc_circuit_breakers.size());
  std::unordered_set<long long> dc_corridor_seen;
  for (int i = 0; i < static_cast<int>(sys.dc.branches.size()) &&
                  c.include_dc_power_flow; ++i) {
    const auto& branch = sys.dc.branches[static_cast<size_t>(i)];
    if (!branch.in_service) continue;
    const auto from = dc_bus_pos.find(branch.from_bus);
    const auto to = dc_bus_pos.find(branch.to_bus);
    if (from == dc_bus_pos.end() || to == dc_bus_pos.end()) continue;
    const double rate = branch.rate_a_mva > 1e-9 ? branch.rate_a_mva
        : (branch.s_max_mva > 1e-9 ? branch.s_max_mva : default_rate_mw);
    const bool radial_representative =
        dc_corridor_seen.insert(undirected_key(from->second, to->second)).second;
    dc_edges.push_back({from->second, to->second,
                        std::max(1e-6, branch.r_pu), rate,
                        component_outaged(fault,
                            ReliabilityComponentKind::DCBranch, i),
                        radial_representative,
                        "dc_branch_" + std::to_string(branch.index)});
  }
  for (int i = 0; i < static_cast<int>(sys.dc.dc_circuit_breakers.size()) &&
                  c.include_dc_power_flow; ++i) {
    const auto& breaker = sys.dc.dc_circuit_breakers[static_cast<size_t>(i)];
    if (!breaker.in_service || !breaker.closed) continue;
    const auto from = dc_bus_pos.find(breaker.bus_from);
    const auto to = dc_bus_pos.find(breaker.bus_to);
    if (from == dc_bus_pos.end() || to == dc_bus_pos.end()) continue;
    const bool radial_representative =
        dc_corridor_seen.insert(undirected_key(from->second, to->second)).second;
    dc_edges.push_back({from->second, to->second, 1e-6, default_rate_mw,
                        component_outaged(fault,
                            ReliabilityComponentKind::DCCircuitBreaker, i),
                        radial_representative,
                        "dc_breaker_" + std::to_string(breaker.index)});
  }
  // Parallel circuits form one topological corridor but retain independent
  // electrical flow/rating constraints. Choose a healthy member as the
  // commodity/radial representative for the current contingency.
  std::unordered_map<long long, int> dc_corridor_representative;
  for (int e = 0; e < static_cast<int>(dc_edges.size()); ++e) {
    auto& edge = dc_edges[static_cast<size_t>(e)];
    edge.radial_representative = false;
    const auto key = undirected_key(edge.from_pos, edge.to_pos);
    const auto current = dc_corridor_representative.find(key);
    if (current == dc_corridor_representative.end() ||
        (dc_edges[static_cast<size_t>(current->second)].failed && !edge.failed))
      dc_corridor_representative[key] = e;
  }
  for (const auto& [key, representative] : dc_corridor_representative) {
    (void)key;
    dc_edges[static_cast<size_t>(representative)].radial_representative = true;
  }
  const int n_dc_edge = static_cast<int>(dc_edges.size());

  std::vector<double> dc_demand(static_cast<size_t>(n_dc_bus), 0.0);
  std::vector<int> load_dc_bus(static_cast<size_t>(nd_total), -1);
  for (int li = 0; li < nd_total; ++li) {
    if (c.loads[static_cast<size_t>(li)].bus < kDCBusOffset) continue;
    const auto bus = dc_bus_pos.find(
        c.loads[static_cast<size_t>(li)].bus - kDCBusOffset);
    if (bus == dc_bus_pos.end()) continue;
    load_dc_bus[static_cast<size_t>(li)] = bus->second;
    if (!load_outaged(fault, c.loads[static_cast<size_t>(li)]))
      dc_demand[static_cast<size_t>(bus->second)] +=
          std::max(0.0, c.loads[static_cast<size_t>(li)].p_kw) / 1000.0;
  }

  std::vector<double> dc_gen_max(static_cast<size_t>(n_dc_bus), 0.0);
  for (const auto& source : c.sources) {
    if (source.bus < kDCBusOffset ||
        source.component_kind == ReliabilityComponentKind::DCStorage ||
        component_outaged(fault, source.component_kind,
                           source.component_position))
      continue;
    const auto bus = dc_bus_pos.find(source.bus - kDCBusOffset);
    if (bus != dc_bus_pos.end())
      dc_gen_max[static_cast<size_t>(bus->second)] +=
          std::max(0.0, source.p_kw) / 1000.0;
  }

  const int n_dc_storage = c.include_dc_power_flow
      ? static_cast<int>(sys.dc.storage.size()) : 0;
  std::vector<int> dc_storage_bus_pos(static_cast<size_t>(n_dc_storage), -1);
  std::vector<double> dc_storage_pmax_mw(static_cast<size_t>(n_dc_storage), 0.0);
  for (int i = 0; i < n_dc_storage; ++i) {
    const auto& storage = sys.dc.storage[static_cast<size_t>(i)];
    const auto bus = dc_bus_pos.find(storage.bus);
    if (!storage.in_service || bus == dc_bus_pos.end() ||
        component_outaged(fault, ReliabilityComponentKind::DCStorage, i))
      continue;
    const size_t energy_pos = static_cast<size_t>(n_storage + i);
    const double available = storage_energy_available_mwh &&
                                     energy_pos < storage_energy_available_mwh->size()
        ? std::max(0.0, (*storage_energy_available_mwh)[energy_pos])
        : initial_storage_deliverable_energy_mwh(storage);
    double cap = hacdcpf::model::effective_capacity_mw(storage);
    if (stage_duration_hr > 1e-12) cap = std::min(cap, available / stage_duration_hr);
    if (available <= 1e-12) cap = 0.0;
    dc_storage_bus_pos[static_cast<size_t>(i)] = bus->second;
    dc_storage_pmax_mw[static_cast<size_t>(i)] = std::max(0.0, cap);
  }

  std::vector<bool> dc_is_source(static_cast<size_t>(n_dc_bus), false);
  std::vector<bool> dc_voltage_anchor(static_cast<size_t>(n_dc_bus), false);
  for (int i = 0; i < n_dc_bus; ++i) {
    const auto& bus = sys.dc.buses[static_cast<size_t>(i)];
    if (bus.in_service && bus.bus_type == DCBusType::DC_V) {
      dc_is_source[static_cast<size_t>(i)] = true;
      dc_voltage_anchor[static_cast<size_t>(i)] = true;
    }
  }
  for (int i = 0; i < n_dc_storage; ++i) {
    if (dc_storage_bus_pos[static_cast<size_t>(i)] >= 0 &&
        sys.dc.storage[static_cast<size_t>(i)].grid_forming &&
        dc_storage_pmax_mw[static_cast<size_t>(i)] > 1e-9)
      dc_is_source[static_cast<size_t>(
          dc_storage_bus_pos[static_cast<size_t>(i)])] = true;
  }
  for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()) &&
                  c.include_dc_power_flow; ++i) {
    const auto& converter = sys.vsc_converters[static_cast<size_t>(i)];
    if (!converter.in_service || component_outaged(
          fault, ReliabilityComponentKind::VSCConverter, i))
      continue;
    const auto bus = dc_bus_pos.find(converter.bus_dc);
    const auto role = resolve_device_control_role(converter);
    if (bus != dc_bus_pos.end()) {
      // In the restoration steady state an energized VSC terminal is a valid
      // DC network root; its active power remains limited by the coupled AC
      // balance. A declared DC-forming role additionally fixes voltage at 1 pu.
      dc_is_source[static_cast<size_t>(bus->second)] = true;
      if (role.provides_dc_v_reference)
        dc_voltage_anchor[static_cast<size_t>(bus->second)] = true;
    }
  }

  struct VSCInfo {
    int ac_pos{-1};
    int dc_pos{-1};
    double cap_mw{0.0};
    double eta{1.0};
    double qmin_mvar{0.0};
    double qmax_mvar{0.0};
    bool available{false};
  };
  std::vector<VSCInfo> vsc_info(sys.vsc_converters.size());
  for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()) &&
                  c.include_dc_power_flow; ++i) {
    const auto& converter = sys.vsc_converters[static_cast<size_t>(i)];
    const auto ac = ac_bus_pos.find(converter.bus_ac);
    const auto dc = dc_bus_pos.find(converter.bus_dc);
    if (!converter.in_service || ac == ac_bus_pos.end() || dc == dc_bus_pos.end() ||
        component_outaged(fault, ReliabilityComponentKind::VSCConverter, i))
      continue;
    double eta = (std::isfinite(converter.eta) && converter.eta > 0.0 &&
                  converter.eta <= 1.0) ? converter.eta : 1.0;
    if (std::isfinite(converter.loss_percent) && converter.loss_percent > 0.0 &&
        converter.loss_percent < 100.0)
      eta *= 1.0 - converter.loss_percent / 100.0;
    vsc_info[static_cast<size_t>(i)] = {
        ac->second, dc->second,
        vsc_transfer_capacity_kw(converter) / 1000.0,
        std::clamp(eta, 1e-6, 1.0), converter.qmin_mvar,
        converter.qmax_mvar, true};
  }

  struct DCDCInfo {
    int from_pos{-1};
    int to_pos{-1};
    double cap_mw{0.0};
    double eta{1.0};
    bool available{false};
  };
  std::vector<DCDCInfo> dcdc_info(sys.dc.dcdc_converters.size());
  for (int i = 0; i < static_cast<int>(sys.dc.dcdc_converters.size()) &&
                  c.include_dc_power_flow; ++i) {
    const auto& converter = sys.dc.dcdc_converters[static_cast<size_t>(i)];
    const auto from = dc_bus_pos.find(converter.bus_in);
    const auto to = dc_bus_pos.find(converter.bus_out);
    if (!converter.in_service || from == dc_bus_pos.end() || to == dc_bus_pos.end() ||
        component_outaged(fault, ReliabilityComponentKind::DCDCConverter, i))
      continue;
    const double cap = std::max({std::abs(converter.pmax_mw),
                                 std::abs(converter.pmin_mw),
                                 std::abs(converter.p_ref_mw),
                                 std::max(0.0, converter.sn_mva)});
    const double eta = (std::isfinite(converter.eta) && converter.eta > 0.0 &&
                        converter.eta <= 1.0) ? converter.eta : 1.0;
    dcdc_info[static_cast<size_t>(i)] = {
        from->second, to->second, cap, eta, cap > 1e-9};
    if (cap > 1e-9) {
      // A controlled DC-DC power stage establishes the downstream DC voltage
      // reference while its active-power equations still require real supply.
      dc_is_source[static_cast<size_t>(from->second)] = true;
      dc_is_source[static_cast<size_t>(to->second)] = true;
    }
  }

  // ── Variable index helpers ────────────────────────────────────────────────
  // AC layout is followed by the coupled DC network, VSC, and DC-DC blocks.
  const int off_shed  = 0;
  const int off_y      = off_shed + n_bus;
  const int off_z      = off_y     + n_bus;
  const int off_P     = off_z    + n_br;
  const int off_Q     = off_P   + n_br;
  const int off_v     = off_Q   + n_br;
  const int off_f     = off_v   + n_bus;
  const int off_pg    = off_f   + n_br;
  const int off_qg    = off_pg  + n_bus;
  const int off_pst   = off_qg  + n_bus;
  const int off_dc_shed = off_pst + n_storage;
  const int off_dc_y = off_dc_shed + n_dc_bus;
  const int off_dc_z = off_dc_y + n_dc_bus;
  const int off_dc_P = off_dc_z + n_dc_edge;
  const int off_dc_v = off_dc_P + n_dc_edge;
  const int off_dc_f = off_dc_v + n_dc_bus;
  const int off_dc_pg = off_dc_f + n_dc_edge;
  const int off_dc_storage = off_dc_pg + n_dc_bus;
  const int off_vsc_forward = off_dc_storage + n_dc_storage;
  const int off_vsc_reverse = off_vsc_forward + static_cast<int>(vsc_info.size());
  const int off_vsc_q = off_vsc_reverse + static_cast<int>(vsc_info.size());
  const int off_dcdc_forward = off_vsc_q + static_cast<int>(vsc_info.size());
  const int off_dcdc_reverse = off_dcdc_forward + static_cast<int>(dcdc_info.size());
  const int n_vars = off_dcdc_reverse + static_cast<int>(dcdc_info.size());

  engine::MIPModel mip;
  auto& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;
  lp.vars.resize(static_cast<size_t>(n_vars));
  lp.c = Eigen::VectorXd::Zero(n_vars);

  // p^sh_i — continuous load shed (MW) at AC bus i
  // Objective: min Σ_i p^sh_i  (weight w_i = 1 per MW shed, eq. 3)
  for (int i = 0; i < n_bus; ++i) {
    lp.vars[off_shed + i] = {engine::VarType::Continuous,
                              0.0, p_d[i],
                              "psh_" + std::to_string(sys.ac.buses[i].index)};
    lp.c[off_shed + i] = 1.0;  // minimise total MW shed (eq. 3)
  }
  for (int s = 0; s < n_storage; ++s) {
    const std::string storage_name = s < n_ac_storage
        ? std::to_string(sys.ac.storage[static_cast<size_t>(s)].index)
        : "mobile_" + std::to_string(
              sys.mobile_storage[static_cast<size_t>(s - n_ac_storage)].index);
    lp.vars[off_pst + s] = {engine::VarType::Continuous, 0.0,
                             storage_pmax_mw[static_cast<size_t>(s)],
                             "pst_" + storage_name};
    // Break dispatch degeneracy: upstream/non-storage supply is preferred when
    // it can serve the same load, preserving battery energy for later stages.
    lp.c[off_pst + s] = 1e-8;
  }

  const int n_sources = static_cast<int>(std::count(is_source.begin(), is_source.end(), true));
  int n_source_components = 0;
  if (n_sources > 0) {
    DSU source_dsu(n_bus);
    for (int b = 0; b < n_br; ++b) {
      const auto& br = ac_branches[b];
      if (br.normally_open) continue;
      if (br.failed) continue;  // F7: faulted branch is out in every stage
      source_dsu.unite(br.from_pos, br.to_pos);
    }
    std::unordered_set<int> source_components;
    for (int i = 0; i < n_bus; ++i) {
      if (is_source[i]) source_components.insert(source_dsu.find(i));
    }
    n_source_components = std::max(1, static_cast<int>(source_components.size()));
  }

  std::vector<bool> fixed_bus_energized(static_cast<size_t>(n_bus), false);
  std::vector<bool> fixed_branch_energized(static_cast<size_t>(n_br), false);
  if (fixed_topology) {
    const auto held_closed = [&](const BrInfo& br) {
      if (!br.normally_open || br.failed) return !br.failed;
      if (!held_stage2_plan) return false;
      if (br.switch_index >= 0) {
        return std::find(held_stage2_plan->closed_tie_switch_indices.begin(),
                         held_stage2_plan->closed_tie_switch_indices.end(),
                         br.switch_index) !=
               held_stage2_plan->closed_tie_switch_indices.end();
      }
      return br.idx_global >= 0 &&
             std::find(held_stage2_plan->closed_legacy_branch_indices.begin(),
                       held_stage2_plan->closed_legacy_branch_indices.end(),
                       br.idx_global) !=
                 held_stage2_plan->closed_legacy_branch_indices.end();
    };
    DSU fixed_dsu(n_bus);
    std::vector<bool> active(static_cast<size_t>(n_br), false);
    for (int b = 0; b < n_br; ++b) {
      active[static_cast<size_t>(b)] = held_closed(ac_branches[b]);
      if (active[static_cast<size_t>(b)])
        fixed_dsu.unite(ac_branches[b].from_pos, ac_branches[b].to_pos);
    }
    std::unordered_set<int> source_roots;
    for (int i = 0; i < n_bus; ++i)
      if (is_source[i]) source_roots.insert(fixed_dsu.find(i));
    n_source_components = static_cast<int>(source_roots.size());
    for (int i = 0; i < n_bus; ++i)
      fixed_bus_energized[static_cast<size_t>(i)] =
          source_roots.count(fixed_dsu.find(i)) > 0;
    for (int b = 0; b < n_br; ++b) {
      const auto& br = ac_branches[b];
      fixed_branch_energized[static_cast<size_t>(b)] =
          active[static_cast<size_t>(b)] &&
          fixed_bus_energized[static_cast<size_t>(br.from_pos)] &&
          fixed_bus_energized[static_cast<size_t>(br.to_pos)];
    }
  }

  // y_i — energized/served bus indicator.  Source buses are energized roots.
  // If no source exists, all y_i are fixed to zero and the model sheds all load.
  for (int i = 0; i < n_bus; ++i) {
    double y_lb = 0.0, y_ub = 1.0;
    engine::VarType y_type = engine::VarType::Binary;
    if (fixed_topology) {
      y_lb = fixed_bus_energized[static_cast<size_t>(i)] ? 1.0 : 0.0;
      y_ub = y_lb;
      y_type = engine::VarType::Continuous;
    } else if (is_source[i]) {
      y_lb = 1.0; y_ub = 1.0; y_type = engine::VarType::Continuous;
    } else if (n_sources == 0) {
      y_lb = 0.0; y_ub = 0.0; y_type = engine::VarType::Continuous;
    }
    lp.vars[off_y + i] = {y_type, y_lb, y_ub,
                          "y_" + std::to_string(sys.ac.buses[i].index)};
    if (y_type == engine::VarType::Binary) mip.binary_idx.push_back(off_y + i);
  }

  // z_ij — binary branch status (1 = closed, 0 = open)
  const int n_no_switch = [&]() {
    int cnt = 0;
    for (const auto& br : ac_branches)
      if (br.normally_open && br.restoration_eligible) ++cnt;
    return cnt;
  }();
  // Big-M commodity capacity = |B| − 1 (number of buses minus 1) so that the
  // single-commodity flow can route one unit to every non-source bus (C6).
  const double commodity_cap = std::max(1.0, static_cast<double>(n_bus - 1));

  for (int b = 0; b < n_br; ++b) {
    const auto& br = ac_branches[b];
    double z_lb = 0.0, z_ub = 1.0;

    // C7: Forced-open failed branch (eq. 12).  F7: the faulted element is out for
    // the whole event and is only repaired at tau_RP, so it stays forced-open in
    // ALL stages, including the Stage-3 repair window [tau_TP, tau_RP].
    if (fixed_topology) {
      z_lb = fixed_branch_energized[static_cast<size_t>(b)] ? 1.0 : 0.0;
      z_ub = z_lb;
    } else if (br.failed) {
      z_lb = 0.0; z_ub = 0.0;
    }
    // C8: z denotes energized branch use, not the mechanical switch handle.
    // Healthy normally-closed edges may de-energize when their endpoint bus is
    // shed; the radial/commodity constraints choose the energized forest.
    // Normally-open ties stay open only in Stage 1 (no switching yet); in Stage 2
    // AND Stage 3 they are candidate switches, so the post-fault reconfiguration
    // is HELD through the repair window (F7).
    else if (!br.normally_open) {
      z_lb = 0.0; z_ub = 1.0;
    } else if (stage == 1) {
      // Stage 1: no switching — normally-open switches stay open.
      z_lb = 0.0; z_ub = 0.0;
    }
    // Stage 2 & 3: normally-open switch — free binary variable, subject to C9.

    // Deterministic fail-to-close: a tie listed as unavailable cannot close in
    // the reconfiguration stages (2 and 3), so its back-feed restoration path is
    // lost for the entire repair window (F7).
    if ((stage == 2 || stage == 3) && br.normally_open && !br.failed &&
        unavailable_ties && br.switch_index >= 0 &&
        unavailable_ties->count(br.switch_index) > 0) {
      z_lb = 0.0;
      z_ub = 0.0;
    }
    if (br.normally_open && !br.restoration_eligible) {
      z_lb = 0.0;
      z_ub = 0.0;
    }
    if (stage == 3 && held_stage2_plan && br.normally_open && !br.failed) {
      const bool held_closed = br.switch_index >= 0
          ? std::find(held_stage2_plan->closed_tie_switch_indices.begin(),
                      held_stage2_plan->closed_tie_switch_indices.end(),
                      br.switch_index) !=
                held_stage2_plan->closed_tie_switch_indices.end()
          : br.idx_global >= 0 &&
                std::find(held_stage2_plan->closed_legacy_branch_indices.begin(),
                          held_stage2_plan->closed_legacy_branch_indices.end(),
                          br.idx_global) !=
                    held_stage2_plan->closed_legacy_branch_indices.end();
      z_lb = held_closed ? 1.0 : 0.0;
      z_ub = z_lb;
    }

    // Fixed z (lb==ub) → Continuous: pure-LP path, avoids B&C postsolve issues.
    // Free z (lb < ub) → Binary: only for truly free NO switches in stage 2.
    const engine::VarType z_type = (z_lb < z_ub) ? engine::VarType::Binary
                                                   : engine::VarType::Continuous;
    lp.vars[off_z + b] = {z_type, z_lb, z_ub, "z_" + std::to_string(b)};
    if (z_lb < z_ub) mip.binary_idx.push_back(off_z + b);
  }

  // p_g_i / q_g_i — finite source dispatch at every AC bus.
  for (int i = 0; i < n_bus; ++i) {
    lp.vars[off_pg + i] = {engine::VarType::Continuous, 0.0, p_gen_max[i],
                           "pg_" + std::to_string(sys.ac.buses[i].index)};
    lp.vars[off_qg + i] = {engine::VarType::Continuous, q_gen_min[i], q_gen_max[i],
                           "qg_" + std::to_string(sys.ac.buses[i].index)};
  }

  // P_ij — active power flow (MW, signed: positive = from_bus → to_bus)
  for (int b = 0; b < n_br; ++b) {
    const double s_max = ac_branches[b].s_max_mw;
    lp.vars[off_P + b] = {engine::VarType::Continuous, -s_max, s_max,
                           "P_" + std::to_string(b)};
  }
  // Q_ij — reactive power flow (Mvar, signed)
  for (int b = 0; b < n_br; ++b) {
    const double s_max = ac_branches[b].s_max_mw;
    lp.vars[off_Q + b] = {engine::VarType::Continuous, -s_max, s_max,
                           "Q_" + std::to_string(b)};
  }
  // v_i — squared voltage magnitude (pu²) at AC bus i
  // Source (slack) buses are fixed at 1.0 pu² (nominal substation voltage).
  // Load buses are bounded by [vmin², vmax²] per their bus data.
  for (int i = 0; i < n_bus; ++i) {
    double vmin2, vmax2;
    if (is_voltage_anchor[i]) {
      vmin2 = 1.0; vmax2 = 1.0;  // source bus: fixed at nominal 1.0 pu²
    } else {
      const double vmin = sys.ac.buses[i].vmin_pu;
      const double vmax = sys.ac.buses[i].vmax_pu;
      vmin2 = vmin * vmin;
      vmax2 = vmax * vmax;
    }
    lp.vars[off_v + i] = {engine::VarType::Continuous, vmin2, vmax2,
                           "v_" + std::to_string(sys.ac.buses[i].index)};
  }
  // f_ij — single-commodity flow auxiliary for radiality (C6)
  // Signed: f_ij < 0 means commodity flows in reverse (to_bus → from_bus),
  // which is needed when the source bus is at the to_bus end of a branch.
  for (int b = 0; b < n_br; ++b) {
    lp.vars[off_f + b] = {engine::VarType::Continuous, -commodity_cap, commodity_cap,
                           "f_" + std::to_string(b)};
  }

  const int n_dc_sources = static_cast<int>(
      std::count(dc_is_source.begin(), dc_is_source.end(), true));
  const double dc_commodity_cap =
      std::max(1.0, static_cast<double>(n_dc_bus - 1));
  for (int i = 0; i < n_dc_bus; ++i) {
    const auto& bus = sys.dc.buses[static_cast<size_t>(i)];
    lp.vars[off_dc_shed + i] = {engine::VarType::Continuous, 0.0,
                                 dc_demand[static_cast<size_t>(i)],
                                 "dc_shed_" + std::to_string(bus.index)};
    lp.c[off_dc_shed + i] = 1.0;
    const bool fixed_off = !bus.in_service || n_dc_sources == 0;
    const bool fixed_on = bus.in_service && dc_is_source[static_cast<size_t>(i)];
    const double ylb = fixed_on ? 1.0 : 0.0;
    const double yub = (fixed_off && !fixed_on) ? 0.0 : 1.0;
    const auto ytype = ylb < yub ? engine::VarType::Binary
                                 : engine::VarType::Continuous;
    lp.vars[off_dc_y + i] = {ytype, ylb, yub,
                             "dc_y_" + std::to_string(bus.index)};
    if (ytype == engine::VarType::Binary) mip.binary_idx.push_back(off_dc_y + i);
    const double vmin2 = dc_voltage_anchor[static_cast<size_t>(i)]
        ? 1.0 : bus.vmin_pu * bus.vmin_pu;
    const double vmax2 = dc_voltage_anchor[static_cast<size_t>(i)]
        ? 1.0 : bus.vmax_pu * bus.vmax_pu;
    lp.vars[off_dc_v + i] = {engine::VarType::Continuous, vmin2, vmax2,
                             "dc_v_" + std::to_string(bus.index)};
    lp.vars[off_dc_pg + i] = {engine::VarType::Continuous, 0.0,
                              dc_gen_max[static_cast<size_t>(i)],
                              "dc_pg_" + std::to_string(bus.index)};
  }
  for (int e = 0; e < n_dc_edge; ++e) {
    const auto& edge = dc_edges[static_cast<size_t>(e)];
    const double zub = edge.failed ? 0.0 : 1.0;
    const auto ztype = zub > 0.5 ? engine::VarType::Binary
                                 : engine::VarType::Continuous;
    lp.vars[off_dc_z + e] = {ztype, 0.0, zub,
                             "dc_z_" + std::to_string(e)};
    if (ztype == engine::VarType::Binary) mip.binary_idx.push_back(off_dc_z + e);
    lp.vars[off_dc_P + e] = {engine::VarType::Continuous,
                             -edge.pmax_mw, edge.pmax_mw,
                             "dc_P_" + std::to_string(e)};
    const double fcap = edge.radial_representative ? dc_commodity_cap : 0.0;
    lp.vars[off_dc_f + e] = {engine::VarType::Continuous,
                             -fcap, fcap,
                             "dc_f_" + std::to_string(e)};
  }
  for (int i = 0; i < n_dc_storage; ++i) {
    lp.vars[off_dc_storage + i] = {
        engine::VarType::Continuous, 0.0,
        dc_storage_pmax_mw[static_cast<size_t>(i)],
        "dc_storage_" + std::to_string(sys.dc.storage[static_cast<size_t>(i)].index)};
    lp.c[off_dc_storage + i] = 1e-8;
  }
  for (int i = 0; i < static_cast<int>(vsc_info.size()); ++i) {
    const auto& info = vsc_info[static_cast<size_t>(i)];
    const double cap = info.available ? info.cap_mw : 0.0;
    lp.vars[off_vsc_forward + i] = {engine::VarType::Continuous, 0.0, cap,
                                    "vsc_ac_to_dc_" + std::to_string(i)};
    lp.vars[off_vsc_reverse + i] = {engine::VarType::Continuous, 0.0, cap,
                                    "vsc_dc_to_ac_" + std::to_string(i)};
    lp.vars[off_vsc_q + i] = {engine::VarType::Continuous,
                              info.available ? info.qmin_mvar : 0.0,
                              info.available ? info.qmax_mvar : 0.0,
                              "vsc_q_" + std::to_string(i)};
    lp.c[off_vsc_forward + i] = 1e-8;
    lp.c[off_vsc_reverse + i] = 1e-8;
  }
  for (int i = 0; i < static_cast<int>(dcdc_info.size()); ++i) {
    const auto& info = dcdc_info[static_cast<size_t>(i)];
    const double cap = info.available ? info.cap_mw : 0.0;
    lp.vars[off_dcdc_forward + i] = {engine::VarType::Continuous, 0.0, cap,
                                     "dcdc_forward_" + std::to_string(i)};
    lp.vars[off_dcdc_reverse + i] = {engine::VarType::Continuous, 0.0, cap,
                                     "dcdc_reverse_" + std::to_string(i)};
    lp.c[off_dcdc_forward + i] = 1e-8;
    lp.c[off_dcdc_reverse + i] = 1e-8;
  }

  // ── Build constraint triplets (equality and inequality) ───────────────────
  std::vector<Eigen::Triplet<double>> eq_trips, ineq_trips;
  std::vector<double> beq_vals, b_vals;

  auto add_eq = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int row = static_cast<int>(beq_vals.size());
    for (const auto& [col, val] : terms)
      if (std::abs(val) > 1e-12) eq_trips.emplace_back(row, col, val);
    beq_vals.push_back(rhs);
  };
  auto add_le = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int row = static_cast<int>(b_vals.size());
    for (const auto& [col, val] : terms)
      if (std::abs(val) > 1e-12) ineq_trips.emplace_back(row, col, val);
    b_vals.push_back(rhs);
  };

  // ── C0 (eq. 3a): energized/load-pickup coupling ─────────────────────────
  // P^d_i - p^sh_i ≤ P^d_i y_i.  If y_i=0 the bus must shed all active load.
  for (int i = 0; i < n_bus; ++i) {
    if (p_d[i] <= 1e-9) continue;
    add_le({{off_shed + i, -1.0}, {off_y + i, -p_d[i]}}, -p_d[i]);
  }
  // Grid-following generation and storage cannot energize a dead component.
  // Their dispatch is available only when y_i is established by an anchor.
  for (int i = 0; i < n_bus; ++i) {
    if (p_gen_max[i] > 1e-12)
      add_le({{off_pg + i, 1.0}, {off_y + i, -p_gen_max[i]}}, 0.0);
    if (q_gen_max[i] > 1e-12)
      add_le({{off_qg + i, 1.0}, {off_y + i, -q_gen_max[i]}}, 0.0);
    if (q_gen_min[i] < -1e-12)
      add_le({{off_qg + i, -1.0}, {off_y + i, q_gen_min[i]}}, 0.0);
  }
  for (int s = 0; s < n_storage; ++s) {
    const int bus = storage_bus_pos[static_cast<size_t>(s)];
    const double cap = storage_pmax_mw[static_cast<size_t>(s)];
    if (bus >= 0 && cap > 1e-12)
      add_le({{off_pst + s, 1.0}, {off_y + bus, -cap}}, 0.0);
  }
  for (int i = 0; i < n_dc_bus; ++i) {
    const double demand = dc_demand[static_cast<size_t>(i)];
    if (demand > 1e-12)
      add_le({{off_dc_shed + i, -1.0},
              {off_dc_y + i, -demand}}, -demand);
    const double generation = dc_gen_max[static_cast<size_t>(i)];
    if (generation > 1e-12)
      add_le({{off_dc_pg + i, 1.0},
              {off_dc_y + i, -generation}}, 0.0);
  }
  for (int i = 0; i < n_dc_storage; ++i) {
    const int bus = dc_storage_bus_pos[static_cast<size_t>(i)];
    const double cap = dc_storage_pmax_mw[static_cast<size_t>(i)];
    if (bus >= 0 && cap > 1e-12)
      add_le({{off_dc_storage + i, 1.0},
              {off_dc_y + bus, -cap}}, 0.0);
  }
  for (int i = 0; i < static_cast<int>(vsc_info.size()); ++i) {
    const auto& info = vsc_info[static_cast<size_t>(i)];
    if (!info.available || info.cap_mw <= 1e-12) continue;
    // IEC 62747:2014 power-transfer boundary, represented in both directions.
    // Both terminal buses must be energized; the strictly positive objective
    // coefficient removes counter-flow degeneracy without another binary.
    for (const int var : {off_vsc_forward + i, off_vsc_reverse + i}) {
      add_le({{var, 1.0}, {off_y + info.ac_pos, -info.cap_mw}}, 0.0);
      add_le({{var, 1.0}, {off_dc_y + info.dc_pos, -info.cap_mw}}, 0.0);
    }
    const double qcap = std::max(std::abs(info.qmin_mvar),
                                 std::abs(info.qmax_mvar));
    if (qcap > 1e-12) {
      add_le({{off_vsc_q + i, 1.0},
              {off_y + info.ac_pos, -qcap}}, 0.0);
      add_le({{off_vsc_q + i, -1.0},
              {off_y + info.ac_pos, -qcap}}, 0.0);
    }
  }
  for (int i = 0; i < static_cast<int>(dcdc_info.size()); ++i) {
    const auto& info = dcdc_info[static_cast<size_t>(i)];
    if (!info.available || info.cap_mw <= 1e-12) continue;
    for (const int var : {off_dcdc_forward + i, off_dcdc_reverse + i}) {
      add_le({{var, 1.0}, {off_dc_y + info.from_pos, -info.cap_mw}}, 0.0);
      add_le({{var, 1.0}, {off_dc_y + info.to_pos, -info.cap_mw}}, 0.0);
    }
  }

  // ── C1 (eq. 4'): LinDistFlow active power balance at every AC bus i ──────
  // Σ_{j:(j,i)} P_ji - Σ_{j:(i,j)} P_ij + p_g,i + p^sh_i = P^d_i
  // with 0 ≤ p_g,i ≤ P^g,max_i.  Source buses are no longer skipped: their
  // injection is finite and appears explicitly through p_g,i.
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms = {{off_shed + i, 1.0},
                                                {off_pg + i, 1.0}};
    for (int s = 0; s < n_storage; ++s) {
      if (storage_bus_pos[static_cast<size_t>(s)] == i)
        terms.push_back({off_pst + s, 1.0});
    }
    // Flow: +P_ji for branches where to_pos = i; -P_ij for branches where from_pos = i
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_P + b,  1.0});
      if (ac_branches[b].from_pos == i) terms.push_back({off_P + b, -1.0});
    }
    for (int v = 0; v < static_cast<int>(vsc_info.size()); ++v) {
      const auto& info = vsc_info[static_cast<size_t>(v)];
      if (!info.available || info.ac_pos != i) continue;
      terms.push_back({off_vsc_forward + v, -1.0});
      terms.push_back({off_vsc_reverse + v, info.eta});
    }
    add_eq(terms, p_d[i]);
  }

  // ── C2 (eq. 5): Reactive power balance at every AC bus i ────────────────
  // Σ Q_ji - Σ Q_ij + q_g,i + (q_d,i/p_d,i) · p^sh_i = Q^d_i
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms = {{off_qg + i, 1.0}};
    // Reactive shed proportional to active shed
    const double ratio = (p_d[i] > 1e-9) ? (q_d[i] / p_d[i]) : 0.0;
    if (std::abs(ratio) > 1e-12) terms.push_back({off_shed + i, ratio});
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_Q + b,  1.0});
      if (ac_branches[b].from_pos == i) terms.push_back({off_Q + b, -1.0});
    }
    for (int v = 0; v < static_cast<int>(vsc_info.size()); ++v) {
      const auto& info = vsc_info[static_cast<size_t>(v)];
      if (info.available && info.ac_pos == i)
        terms.push_back({off_vsc_q + v, 1.0});
    }
    add_eq(terms, q_d[i]);
  }

  // Coupled DC nodal balance. Baran & Wu (1989), eqs. (1)-(3), specialized
  // to a resistive DC feeder. Converter input and efficiency-weighted output
  // appear in the same stage model as the AC balance above.
  for (int i = 0; i < n_dc_bus; ++i) {
    std::vector<std::pair<int, double>> terms = {
        {off_dc_shed + i, 1.0}, {off_dc_pg + i, 1.0}};
    for (int s = 0; s < n_dc_storage; ++s)
      if (dc_storage_bus_pos[static_cast<size_t>(s)] == i)
        terms.push_back({off_dc_storage + s, 1.0});
    for (int e = 0; e < n_dc_edge; ++e) {
      if (dc_edges[static_cast<size_t>(e)].to_pos == i)
        terms.push_back({off_dc_P + e, 1.0});
      if (dc_edges[static_cast<size_t>(e)].from_pos == i)
        terms.push_back({off_dc_P + e, -1.0});
    }
    for (int v = 0; v < static_cast<int>(vsc_info.size()); ++v) {
      const auto& info = vsc_info[static_cast<size_t>(v)];
      if (!info.available || info.dc_pos != i) continue;
      terms.push_back({off_vsc_forward + v, info.eta});
      terms.push_back({off_vsc_reverse + v, -1.0});
    }
    for (int d = 0; d < static_cast<int>(dcdc_info.size()); ++d) {
      const auto& info = dcdc_info[static_cast<size_t>(d)];
      if (!info.available) continue;
      if (info.from_pos == i) {
        terms.push_back({off_dcdc_forward + d, -1.0});
        terms.push_back({off_dcdc_reverse + d, info.eta});
      }
      if (info.to_pos == i) {
        terms.push_back({off_dcdc_forward + d, info.eta});
        terms.push_back({off_dcdc_reverse + d, -1.0});
      }
    }
    add_eq(terms, dc_demand[static_cast<size_t>(i)]);
  }

  // ── C3 (eq. 6'): LinDistFlow voltage drop with Big-M on z_ij ────────────
  // When z_ij = 1 (closed):  v_j = v_i - 2(r_pu/S_base · P_MW + x_pu/S_base · Q_MW)
  // When z_ij = 0 (open):    v_j and v_i are decoupled (Big-M relaxation).
  //
  // Dimensional note: v is in pu², r/x are in pu, P/Q are in MW.
  //   Per-unit equation: v_j = v_i - 2(r_pu · P_pu + x_pu · Q_pu)
  //   Substituting P_pu = P_MW / S_base:  coefficient = 2 · r_pu / S_base
  //
  // Big-M formulation (with correct +M·z coefficient):
  //   |v_j - v_i + 2(r/S·P + x/S·Q)| ≤ M·(1 - z_ij)
  // As two ≤ constraints:
  //   v_j - v_i + 2r/S·P + 2x/S·Q + M·z ≤ M
  //  -v_j + v_i - 2r/S·P - 2x/S·Q + M·z ≤ M
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  for (int b = 0; b < n_br; ++b) {
    const auto& br = ac_branches[b];
    // Scale r/x by 2/S_base for dimensional consistency (MW → pu)
    const double rc = 2.0 * br.r_pu / base_mva;
    const double xc = 2.0 * br.x_pu / base_mva;
    // Constraint 1: v_j - v_i + rc·P + xc·Q + M·z ≤ M
    add_le({{ off_v + br.to_pos,    1.0},
            { off_v + br.from_pos, -1.0},
            { off_P + b,  rc},
            { off_Q + b,  xc},
            { off_z + b,  kBigMVoltage}},
           kBigMVoltage);
    // Constraint 2: -v_j + v_i - rc·P - xc·Q + M·z ≤ M
    add_le({{ off_v + br.to_pos,   -1.0},
            { off_v + br.from_pos,  1.0},
            { off_P + b, -rc},
            { off_Q + b, -xc},
            { off_z + b,  kBigMVoltage}},
           kBigMVoltage);
  }

  // ── C4 (eq. 7–8): Branch flow limits with Big-M ──────────────────────────
  // -z_ij · S̄ ≤ P_ij ≤ z_ij · S̄
  // P_ij - z_ij · S̄ ≤ 0    →   P_ij - S̄ · z_ij ≤ 0
  // -P_ij - z_ij · S̄ ≤ 0   →  -P_ij - S̄ · z_ij ≤ 0
  for (int b = 0; b < n_br; ++b) {
    const double s = ac_branches[b].s_max_mw;
    add_le({{ off_P + b,  1.0}, { off_z + b, -s}}, 0.0);  // P ≤ z·S̄
    add_le({{ off_P + b, -1.0}, { off_z + b, -s}}, 0.0);  // -P ≤ z·S̄
    add_le({{ off_Q + b,  1.0}, { off_z + b, -s}}, 0.0);  // Q ≤ z·S̄
    add_le({{ off_Q + b, -1.0}, { off_z + b, -s}}, 0.0);  // -Q ≤ z·S̄
  }

  // DC voltage drop and branch transfer limits. Taylor (2015), Sec. 4.3;
  // docs/reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md
  // Sec. 10.5. P is MW, r is pu, and v is squared pu voltage.
  const double dc_base_mva = std::max(sys.dc.base_mva, 1.0);
  for (int e = 0; e < n_dc_edge; ++e) {
    const auto& edge = dc_edges[static_cast<size_t>(e)];
    const double rc = 2.0 * edge.r_pu / dc_base_mva;
    add_le({{off_dc_v + edge.to_pos, 1.0},
            {off_dc_v + edge.from_pos, -1.0},
            {off_dc_P + e, rc},
            {off_dc_z + e, kBigMVoltage}}, kBigMVoltage);
    add_le({{off_dc_v + edge.to_pos, -1.0},
            {off_dc_v + edge.from_pos, 1.0},
            {off_dc_P + e, -rc},
            {off_dc_z + e, kBigMVoltage}}, kBigMVoltage);
    add_le({{off_dc_P + e, 1.0},
            {off_dc_z + e, -edge.pmax_mw}}, 0.0);
    add_le({{off_dc_P + e, -1.0},
            {off_dc_z + e, -edge.pmax_mw}}, 0.0);
  }

  // ── C6 (eq. 10–11): strict energized radial forest ──────────────────────
  // Energized non-source buses consume one commodity unit; source buses inject.
  // z≤y endpoint constraints forbid closed branches touching de-energized buses.
  // The edge-count inequality eliminates redundant energized cycles.
  for (int b = 0; b < n_br; ++b) {
    // |f_ij| ≤ (|B|-1) · z_ij  (bidirectional flow bound)
    add_le({{ off_f + b,  1.0}, { off_z + b, -commodity_cap}}, 0.0);  // f ≤ cap·z
    add_le({{ off_f + b, -1.0}, { off_z + b, -commodity_cap}}, 0.0);  // -f ≤ cap·z
    add_le({{ off_z + b,  1.0}, { off_y + ac_branches[b].from_pos, -1.0}}, 0.0);
    add_le({{ off_z + b,  1.0}, { off_y + ac_branches[b].to_pos,   -1.0}}, 0.0);
    if (!ac_branches[b].normally_open && !ac_branches[b].failed) {
      add_le({{ off_y + ac_branches[b].from_pos, 1.0},
              { off_y + ac_branches[b].to_pos,   1.0},
              { off_z + b,                      -1.0}},
             1.0);
    }
  }
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms;
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_f + b,  1.0});  // f_ji (inflow)
      if (ac_branches[b].from_pos == i) terms.push_back({off_f + b, -1.0});  // f_ij (outflow)
    }
    if (is_source[i]) {
      // Source bus: net outflow ≥ 0  →  -(net outflow) ≤ 0  →  Σ inflow - Σ outflow ≤ 0
      add_le(terms, 0.0);
    } else {
      terms.push_back({off_y + i, -1.0});
      add_eq(terms, 0.0);
    }
  }
  if (n_sources > 0) {
    std::vector<std::pair<int,double>> forest_terms;
    for (int b = 0; b < n_br; ++b) forest_terms.push_back({off_z + b, 1.0});
    for (int i = 0; i < n_bus; ++i) forest_terms.push_back({off_y + i, -1.0});
    add_le(forest_terms, -static_cast<double>(n_source_components));
  }

  if (n_dc_bus > 0) {
    DSU dc_source_dsu(n_dc_bus);
    for (const auto& edge : dc_edges)
      if (!edge.failed) dc_source_dsu.unite(edge.from_pos, edge.to_pos);
    std::unordered_set<int> dc_source_roots;
    for (int i = 0; i < n_dc_bus; ++i)
      if (dc_is_source[static_cast<size_t>(i)])
        dc_source_roots.insert(dc_source_dsu.find(i));
    const int dc_source_components = static_cast<int>(dc_source_roots.size());

    for (int e = 0; e < n_dc_edge; ++e) {
      const auto& edge = dc_edges[static_cast<size_t>(e)];
      if (edge.radial_representative) {
        add_le({{off_dc_f + e, 1.0},
                {off_dc_z + e, -dc_commodity_cap}}, 0.0);
        add_le({{off_dc_f + e, -1.0},
                {off_dc_z + e, -dc_commodity_cap}}, 0.0);
      }
      add_le({{off_dc_z + e, 1.0},
              {off_dc_y + edge.from_pos, -1.0}}, 0.0);
      add_le({{off_dc_z + e, 1.0},
              {off_dc_y + edge.to_pos, -1.0}}, 0.0);
      if (!edge.failed) {
        add_le({{off_dc_y + edge.from_pos, 1.0},
                {off_dc_y + edge.to_pos, 1.0},
                {off_dc_z + e, -1.0}}, 1.0);
      }
    }
    for (int i = 0; i < n_dc_bus; ++i) {
      std::vector<std::pair<int, double>> terms;
      for (int e = 0; e < n_dc_edge; ++e) {
        if (!dc_edges[static_cast<size_t>(e)].radial_representative) continue;
        if (dc_edges[static_cast<size_t>(e)].to_pos == i)
          terms.push_back({off_dc_f + e, 1.0});
        if (dc_edges[static_cast<size_t>(e)].from_pos == i)
          terms.push_back({off_dc_f + e, -1.0});
      }
      if (dc_is_source[static_cast<size_t>(i)]) {
        add_le(terms, 0.0);
      } else {
        terms.push_back({off_dc_y + i, -1.0});
        add_eq(terms, 0.0);
      }
    }
    if (dc_source_components > 0) {
      std::vector<std::pair<int, double>> forest_terms;
      for (int e = 0; e < n_dc_edge; ++e)
        if (dc_edges[static_cast<size_t>(e)].radial_representative)
          forest_terms.push_back({off_dc_z + e, 1.0});
      for (int i = 0; i < n_dc_bus; ++i)
        forest_terms.push_back({off_dc_y + i, -1.0});
      add_le(forest_terms, -static_cast<double>(dc_source_components));
    }
  }

  // ── C9 (eq. 14): Switch count ≤ K^sw ────────────────────────────────────
  // Σ_{(i,j)∈L^NO} z_ij ≤ K^sw   (Stages 2 and 3; K^sw = max_sw_ops).
  // The reconfiguration chosen at switching (Stage 2) is HELD through the repair
  // window (Stage 3, F7), so the same switch-count budget applies in both.
  if ((stage == 2 || stage == 3) && n_no_switch > 0) {
    std::vector<std::pair<int,double>> sw_terms;
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].normally_open && ac_branches[b].restoration_eligible)
        sw_terms.push_back({off_z + b, 1.0});
    }
    if (!sw_terms.empty()) {
      const double ksw = (max_sw_ops == INT_MAX)
                           ? static_cast<double>(n_no_switch)  // no limit
                           : static_cast<double>(max_sw_ops);
      add_le(sw_terms, ksw);
    }
  }

  // C5 (eq. 9): Voltage bounds enforced via variable bounds [vmin², vmax²]
  // (already set in lp.vars above — no additional constraint row needed)

  // ── Finalise constraint matrices ─────────────────────────────────────────
  const int n_eq   = static_cast<int>(beq_vals.size());
  const int n_ineq = static_cast<int>(b_vals.size());

  lp.Aeq.resize(n_eq, n_vars);
  lp.beq.resize(n_eq);
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();
  for (int r = 0; r < n_eq; ++r) lp.beq[r] = beq_vals[r];

  lp.A.resize(n_ineq, n_vars);
  lp.b.resize(n_ineq);
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();
  for (int r = 0; r < n_ineq; ++r) lp.b[r] = b_vals[r];

  // ── Solve ─────────────────────────────────────────────────────────────────
  // When all z variables are fixed (no free binary/integer decisions), bypass
  // the B&C to avoid presolve/postsolve size-mismatch bugs on LP-only problems.
  // Only use B&C when there are genuinely free binary/integer variables (stage 2
  // with normally-open switches).
  Eigen::VectorXd res_x;
  bool res_success = false;
  std::string res_status;
  double res_objective = 0.0;
  double res_mip_gap = 0.0;
  constexpr double kGapTol = 1e-6;

  if (mip.binary_idx.empty() && mip.integer_idx.empty()) {
    // Fixed-topology LPs are large and highly sparse. Prefer HiGHS here: the
    // native dual simplex can return a nominally successful basis with material
    // residual on these heavily fixed models, which then triggers a costly MILP
    // fallback for every contingency.
    engine::HighsAdapter highs;
    if (highs.available()) {
      auto hr = highs.solve_lp(lp);
      res_x = hr.x;
      res_success = hr.stats.success;
      res_status = hr.stats.status;
      res_objective = hr.stats.objective;
    } else {
      engine::SimplexOptions simp_opt;
      simp_opt.max_iter        = 10000;
      simp_opt.feasibility_tol = 1e-8;
      simp_opt.optimality_tol  = 1e-8;
      simp_opt.verbose         = false;
      auto sr = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
      res_x        = sr.result.x;
      res_success  = sr.result.stats.success;
      res_status   = sr.result.stats.status;
      res_objective= sr.result.stats.objective;
    }
    res_mip_gap  = 0.0;  // pure LP has no integrality gap
  } else {
    auto solve_with_native_bc = [&](const std::string& previous_failure) {
      engine::BCOptions opt;
      opt.max_nodes        = 2048;
      opt.time_limit_sec   = 30.0;
      opt.gap_tol          = kGapTol;
      opt.verbose          = false;
      opt.use_simplex_lp_nodes = true;
      // Contingencies are already parallelized by the caller. Keep fallback
      // B&C serial to avoid nested worker pools and oversubscription.
      opt.num_threads      = 1;
      auto bc_res  = engine::solve_milp_bc(mip, opt);
      res_x        = bc_res.x;
      res_success  = bc_res.stats.success;
      res_status   = previous_failure.empty()
          ? bc_res.stats.status
          : "HiGHS failed: " + previous_failure + "; NativeB&C: " + bc_res.stats.status;
      res_objective= bc_res.stats.objective;
      res_mip_gap  = std::isfinite(bc_res.bc_stats.gap) ? bc_res.bc_stats.gap
                                                         : bc_res.stats.mip_gap;
    };

    engine::BCOptions highs_options;
    highs_options.max_nodes = 2048;
    highs_options.time_limit_sec = 30.0;
    highs_options.gap_tol = kGapTol;
    engine::StrictHighsBranchAndCutAdapter highs(highs_options);
    {
      std::lock_guard<std::mutex> solve_lock(stage_milp_solve_mutex);
      auto highs_res = highs.solve_milp(mip);
      res_x        = highs_res.x;
      res_success  = highs_res.stats.success;
      res_status   = highs_res.stats.status;
      res_objective= highs_res.stats.objective;
      res_mip_gap  = highs_res.stats.mip_gap;
      if (!res_success || res_x.size() != static_cast<size_t>(n_vars)) {
        std::string reason = res_status.empty() ? std::string("unsuccessful solve") : res_status;
        if (res_success && res_x.size() != static_cast<size_t>(n_vars)) {
          reason += " (solution vector has wrong size)";
        }
        solve_with_native_bc(reason);
      }
    }
  }

  if (!res_success || res_x.size() != static_cast<size_t>(n_vars)) {
    out.status = res_status.empty() ? "failed" : res_status;
    // Conservative failure result: shed every represented AC and DC load.
    for (int li = 0; li < nd_total; ++li)
      out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
    out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
    return out;
  }

  const bool proven_optimal = (res_mip_gap <= kGapTol + 1e-9);
  out.status = proven_optimal ? "success" : "success (approximate)";
  out.mip_gap = res_mip_gap;
  out.proven_optimal = proven_optimal;
  out.objective = res_objective;

  const double bound_tol = 1e-6;
  const double integer_tol = 1e-4;
  auto fail_postsolve = [&]() {
    out.status = "failed (constraint violation)";
    out.proven_optimal = false;
    for (int li = 0; li < nd_total; ++li)
      out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
    out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
  };
  for (int var = 0; var < n_vars; ++var) {
    const double value = res_x[var];
    const auto& bounds = lp.vars[static_cast<size_t>(var)];
    if (value < bounds.lb - bound_tol || value > bounds.ub + bound_tol) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP bound violation: var[{}]={:.8f} bounds=[{:.8f},{:.8f}]",
                   var, value, bounds.lb, bounds.ub);
      fail_postsolve();
      return out;
    }
  }
  auto check_integer = [&](int var) -> bool {
    if (var < 0 || var >= n_vars) return true;
    const double value = res_x[var];
    if (std::abs(value - std::round(value)) <= integer_tol) return true;
    spdlog::warn("[三阶段可靠性] LinDistFlow MILP integrality violation: var[{}]={:.8f}",
                 var, value);
    return false;
  };
  for (int var : mip.binary_idx) {
    if (!check_integer(var)) { fail_postsolve(); return out; }
  }
  for (int var : mip.integer_idx) {
    if (!check_integer(var)) { fail_postsolve(); return out; }
  }
  const double recomputed_objective = lp.c.dot(res_x);
  const double objective_tol = 1e-5 * std::max(1.0, std::abs(recomputed_objective));
  if (std::isfinite(res_objective) &&
      std::abs(res_objective - recomputed_objective) > objective_tol) {
    spdlog::warn("[三阶段可靠性] LinDistFlow MILP objective mismatch: solver={:.8f} recomputed={:.8f}",
                 res_objective, recomputed_objective);
    fail_postsolve();
    return out;
  }
  out.objective = recomputed_objective;

  for (int s = 0; s < n_storage; ++s) {
    const double discharge = std::max(0.0, res_x[off_pst + s]);
    out.storage_discharge_mw[static_cast<size_t>(s)] = discharge;
    out.storage_energy_used_mwh[static_cast<size_t>(s)] =
        discharge * std::max(0.0, stage_duration_hr);
  }
  for (int s = 0; s < n_dc_storage; ++s) {
    const double discharge = std::max(0.0, res_x[off_dc_storage + s]);
    const size_t out_pos = static_cast<size_t>(n_storage + s);
    out.storage_discharge_mw[out_pos] = discharge;
    out.storage_energy_used_mwh[out_pos] =
        discharge * std::max(0.0, stage_duration_hr);
  }
  for (int i = 0; i < static_cast<int>(vsc_info.size()); ++i) {
    out.vsc_dispatch_kw[static_cast<size_t>(i)] =
        (res_x[off_vsc_forward + i] - res_x[off_vsc_reverse + i]) * 1000.0;
  }
  for (int i = 0; i < static_cast<int>(dcdc_info.size()); ++i) {
    out.dcdc_dispatch_kw[static_cast<size_t>(i)] =
        (res_x[off_dcdc_forward + i] - res_x[off_dcdc_reverse + i]) * 1000.0;
  }

  // Post-solve acceptance threshold fixed by the coupled-model validation
  // protocol in docs/reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md
  // Sec. 10.5. This is 100x the LP feasibility tolerance and remains below the
  // 1 W error scale on a 1 MW transfer.
  constexpr double kCoupledResidualTolerance = 1e-6;
  if (n_eq > 0) {
    const Eigen::VectorXd eq_res = (lp.Aeq * res_x - lp.beq).cwiseAbs();
    if (eq_res.maxCoeff() > kCoupledResidualTolerance) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP equality violation={:.2e} — fallback",
                   eq_res.maxCoeff());
      out.status = "failed (constraint violation)";
      for (int li = 0; li < nd_total; ++li)
        out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
      out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
      return out;
    }
  }
  if (n_ineq > 0) {
    const double ineq_viol = (lp.A * res_x - lp.b).cwiseMax(0.0).maxCoeff();
    if (ineq_viol > kCoupledResidualTolerance) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP inequality violation={:.2e} — fallback",
                   ineq_viol);
      out.status = "failed (constraint violation)";
      for (int li = 0; li < nd_total; ++li)
        out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
      out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
      return out;
    }
  }

  // Extract an auditable ordered switching plan from the accepted final
  // topology.  Every prefix of these closures remains acyclic because the MILP
  // final state is a strict radial forest and no normally-closed energized edge
  // is silently opened to make room for a tie.
  if (stage == 2 || stage == 3) {
    for (int b = 0; b < n_br; ++b) {
      const auto& br = ac_branches[b];
      if (!br.normally_open || br.switch_index < 0) continue;
      if (res_x[off_z + b] > 0.5)
        out.closed_tie_switch_indices.push_back(br.switch_index);
    }
    for (int b = 0; b < n_br; ++b) {
      const auto& br = ac_branches[b];
      if (br.normally_open && br.switch_index < 0 && br.idx_global >= 0 &&
          res_x[off_z + b] > 0.5)
        out.closed_legacy_branch_indices.push_back(br.idx_global);
    }
    out.switch_sequence_valid = out.closed_legacy_branch_indices.empty();
    out.switch_sequence_message = !out.switch_sequence_valid
        ? "selected legacy out-of-service branch has no controlling switch metadata"
        : (out.closed_tie_switch_indices.empty()
               ? "no switching required"
               : "ordered tie closures validated against eligibility, radiality, voltage, and branch limits");
  }

  // ── Extract p^sh_i (per AC bus) → map back to load points ───────────────
  // p^sh_i is defined per AC bus.  Each load point (c.loads[li]) that maps
  // to AC bus i gets a shed proportional to its share of p_d[i].
  for (int i = 0; i < n_bus; ++i) {
    const double psh_bus_mw = std::max(0.0, std::min(res_x[off_shed + i], p_d[i]));
    if (p_d[i] < 1e-9) continue;
    // Distribute shed proportionally among load points at this bus
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] != i) continue;
      if (load_outaged(fault, c.loads[li])) continue;
      const double ld_mw = std::max(0.0, c.loads[li].p_kw) / 1000.0;
      const double share = ld_mw / p_d[i];
      out.shed_by_load[li] = psh_bus_mw * share * 1000.0;  // back to kW
    }
  }

  // ── DC-only loads: explicit opt-out connectivity/capacity fallback ───────
  // Loads whose bus is not in ac_bus_pos (i.e., DC buses not connected to any
  // AC bus in this model) are handled by the original capacity-balance approach.
  if (!c.include_dc_power_flow) {
    // Build DSU over DC buses for the current stage
    auto dc_pos = bus_position_map(c.buses);
    DSU dsu_dc(static_cast<int>(c.buses.size()));
    auto connect_dc = [&](int a, int b_bus) {
      auto ia = dc_pos.find(a), ib = dc_pos.find(b_bus);
      if (ia != dc_pos.end() && ib != dc_pos.end()) dsu_dc.unite(ia->second, ib->second);
    };
    for (int b = 0; b < static_cast<int>(sys.ac.branches.size()); ++b) {
      const auto& br = sys.ac.branches[b];
      bool on = br.in_service;
      // F7: the faulted branch is out in every stage (repaired only at tau_RP);
      // do not re-energize it in Stage 3.
      if (component_outaged(fault, ReliabilityComponentKind::ACBranch, b))
        on = false;
      if (on) connect_dc(br.from_bus, br.to_bus);
    }
    for (int b = 0; b < static_cast<int>(sys.dc.branches.size()); ++b) {
      const auto& br = sys.dc.branches[b];
      bool on = br.in_service;
      if (component_outaged(fault, ReliabilityComponentKind::DCBranch, b))
        on = false;  // F7
      if (on) connect_dc(br.from_bus + kDCBusOffset, br.to_bus + kDCBusOffset);
    }
    for (int vi = 0; vi < static_cast<int>(sys.vsc_converters.size()); ++vi) {
      const auto& vsc = sys.vsc_converters[vi];
      if (!vsc.in_service) continue;
      if (component_outaged(fault, ReliabilityComponentKind::VSCConverter,
                            vi))
        continue;
      connect_dc(vsc.bus_ac, vsc.bus_dc + kDCBusOffset);
    }
    for (int di = 0; di < static_cast<int>(sys.dc.dcdc_converters.size()); ++di) {
      const auto& dc = sys.dc.dcdc_converters[di];
      if (!dc.in_service) continue;
      if (component_outaged(fault, ReliabilityComponentKind::DCDCConverter,
                            di))
        continue;
      connect_dc(dc.bus_in + kDCBusOffset, dc.bus_out + kDCBusOffset);
    }
    // DC circuit breakers are always connectivity edges; the option controls
    // initiating-breaker enumeration, while protection zones may still open one.
    {
      for (int ci = 0; ci < static_cast<int>(sys.dc.dc_circuit_breakers.size()); ++ci) {
        const auto& cb = sys.dc.dc_circuit_breakers[ci];
        if (!cb.in_service || !cb.closed) continue;
        if (component_outaged(fault,
                              ReliabilityComponentKind::DCCircuitBreaker, ci))
          continue;
        connect_dc(cb.bus_from + kDCBusOffset, cb.bus_to + kDCBusOffset);
      }
    }

    std::unordered_map<int, int> dc_comp;
    for (int i = 0; i < static_cast<int>(c.buses.size()); ++i)
      dc_comp[c.buses[i]] = dsu_dc.find(i);

    std::unordered_map<int, double> comp_ac_source_kw;
    std::unordered_map<int, double> comp_dc_source_kw;
    for (const auto& s : c.sources) {
      if (component_outaged(fault, s.component_kind,
                            s.component_position))
        continue;
      auto it = dc_comp.find(s.bus);
      if (it == dc_comp.end()) continue;
      if (s.bus >= kDCBusOffset) comp_dc_source_kw[it->second] += s.p_kw;
      else comp_ac_source_kw[it->second] += s.p_kw;
    }
    std::unordered_map<int, double> comp_ac_served_kw;
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] < 0) continue;
      if (load_outaged(fault, c.loads[li])) continue;
      auto it = dc_comp.find(c.loads[li].bus);
      if (it == dc_comp.end()) continue;
      comp_ac_served_kw[it->second] +=
          std::max(0.0, c.loads[li].p_kw - out.shed_by_load[li]);
    }
    std::unordered_map<int, double> comp_vsc_transfer_kw;
    for (int vi = 0; vi < static_cast<int>(sys.vsc_converters.size()); ++vi) {
      const auto& vsc = sys.vsc_converters[vi];
      if (!vsc.in_service) continue;
      if (component_outaged(fault, ReliabilityComponentKind::VSCConverter,
                            vi))
        continue;
      auto ac_it = dc_comp.find(vsc.bus_ac);
      auto dc_it = dc_comp.find(vsc.bus_dc + kDCBusOffset);
      if (ac_it == dc_comp.end() || dc_it == dc_comp.end()) continue;
      if (ac_it->second == dc_it->second) {
        comp_vsc_transfer_kw[ac_it->second] += vsc_transfer_capacity_kw(vsc);
      }
    }
    // ── DC LinDistFlow power flow (opt-in) ────────────────────────────────
    // When enabled, replace the aggregate capacity check with a per-bus DC power
    // flow: v in [vmin^2, vmax^2], resistive drop v_j = v_i - 2 r P, per-branch
    // thermal limits, DC sources, and VSC transfers budgeted by the AC surplus of
    // the merged component.  Falls back to the capacity check if it cannot solve.
    bool dc_pf_done = false;
    if (c.include_dc_power_flow) {
      dc_pf_done = [&]() -> bool {
        std::vector<int> dcbus_ids;
        std::unordered_map<int, int> dcpos;
        for (const auto& b : sys.dc.buses) {
          if (!b.in_service || dcpos.count(b.index)) continue;
          dcpos[b.index] = static_cast<int>(dcbus_ids.size());
          dcbus_ids.push_back(b.index);
        }
        const int ndcb = static_cast<int>(dcbus_ids.size());
        if (ndcb == 0) return true;  // no DC buses -> nothing to shed

        std::vector<double> vlo(ndcb, 0.81), vhi(ndcb, 1.21);
        std::vector<char> vref(ndcb, 0);
        std::vector<double> dcsrc_mw(ndcb, 0.0);
        for (const auto& b : sys.dc.buses) {
          auto it = dcpos.find(b.index);
          if (it == dcpos.end()) continue;
          vlo[it->second] = b.vmin_pu * b.vmin_pu;
          vhi[it->second] = b.vmax_pu * b.vmax_pu;
          if (b.bus_type == DCBusType::DC_V) vref[it->second] = 1;
        }
        for (const auto& s : c.sources) {
          if (component_outaged(fault, s.component_kind,
                                s.component_position))
            continue;
          if (s.bus < kDCBusOffset) continue;
          auto it = dcpos.find(s.bus - kDCBusOffset);
          if (it != dcpos.end()) dcsrc_mw[it->second] += s.p_kw / 1000.0;
        }
        struct Edge { int f, t; double r, smax; };
        std::vector<Edge> dcbrs, dcdcs;
        for (int b = 0; b < static_cast<int>(sys.dc.branches.size()); ++b) {
          const auto& br = sys.dc.branches[b];
          if (!br.in_service) continue;
          if (component_outaged(fault, ReliabilityComponentKind::DCBranch, b))
            continue;
          auto itf = dcpos.find(br.from_bus), itt = dcpos.find(br.to_bus);
          if (itf == dcpos.end() || itt == dcpos.end()) continue;
          const double smax = br.rate_a_mva > 1e-9 ? br.rate_a_mva
                            : (br.s_max_mva > 1e-9 ? br.s_max_mva : default_rate_mw);
          dcbrs.push_back({itf->second, itt->second, std::max(1e-6, br.r_pu), smax});
        }
        for (int d = 0; d < static_cast<int>(sys.dc.dcdc_converters.size()); ++d) {
          const auto& dd = sys.dc.dcdc_converters[d];
          if (!dd.in_service) continue;
          if (component_outaged(fault,
                                ReliabilityComponentKind::DCDCConverter, d))
            continue;
          auto itf = dcpos.find(dd.bus_in), itt = dcpos.find(dd.bus_out);
          if (itf == dcpos.end() || itt == dcpos.end()) continue;
          const double cap = std::max(std::abs(dd.pmax_mw), std::abs(dd.pmin_mw));
          dcdcs.push_back({itf->second, itt->second, 0.0, cap > 1e-9 ? cap : default_rate_mw});
        }
        struct VscT { int dcb; double cap_mw; int accomp; };
        std::vector<VscT> vscs;
        for (int vi = 0; vi < static_cast<int>(sys.vsc_converters.size()); ++vi) {
          const auto& v = sys.vsc_converters[vi];
          if (!v.in_service) continue;
          if (component_outaged(fault, ReliabilityComponentKind::VSCConverter,
                                vi))
            continue;
          auto itd = dcpos.find(v.bus_dc);
          if (itd == dcpos.end()) continue;
          vref[itd->second] = 1;  // VSC forms the DC voltage reference
          auto acc = dc_comp.find(v.bus_ac);
          vscs.push_back({itd->second, vsc_transfer_capacity_kw(v) / 1000.0,
                          acc != dc_comp.end() ? acc->second : -1});
        }
        std::vector<int> dcload_li, dcload_pos;
        std::vector<double> dcload_mw;
        for (int li = 0; li < nd_total; ++li) {
          if (load_ac_bus[li] >= 0) continue;
          if (load_outaged(fault, c.loads[li])) {
            out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
            continue;
          }
          auto it = dcpos.find(c.loads[li].bus - kDCBusOffset);
          if (it == dcpos.end()) { out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw); continue; }
          dcload_li.push_back(li);
          dcload_pos.push_back(it->second);
          dcload_mw.push_back(std::max(0.0, c.loads[li].p_kw) / 1000.0);
        }
        const int nL = static_cast<int>(dcload_li.size());
        const int nBr = static_cast<int>(dcbrs.size());
        const int nDd = static_cast<int>(dcdcs.size());
        const int nV = static_cast<int>(vscs.size());
        // Layout: [shed_L][v_bus][P_br][psrc_bus][pvsc_V][pdcdc_Dd]
        const int o_shed = 0, o_v = o_shed + nL, o_P = o_v + ndcb;
        const int o_src = o_P + nBr, o_vsc = o_src + ndcb, o_dd = o_vsc + nV;
        const int nvar = o_dd + nDd;

        engine::MIPModel dcmip;
        auto& dclp = dcmip.linear_part;
        dclp.sense = engine::Sense::Minimize;
        dclp.vars.resize(static_cast<size_t>(nvar));
        dclp.c = Eigen::VectorXd::Zero(nvar);
        for (int i = 0; i < nL; ++i) {
          dclp.vars[o_shed + i] = {engine::VarType::Continuous, 0.0, dcload_mw[i], "dcshed"};
          dclp.c[o_shed + i] = 1.0;
        }
        for (int i = 0; i < ndcb; ++i)
          dclp.vars[o_v + i] = {engine::VarType::Continuous,
                                vref[i] ? 1.0 : vlo[i], vref[i] ? 1.0 : vhi[i], "dcv"};
        for (int i = 0; i < nBr; ++i)
          dclp.vars[o_P + i] = {engine::VarType::Continuous, -dcbrs[i].smax, dcbrs[i].smax, "dcP"};
        for (int i = 0; i < ndcb; ++i)
          dclp.vars[o_src + i] = {engine::VarType::Continuous, 0.0, dcsrc_mw[i], "dcsrc"};
        for (int i = 0; i < nV; ++i)
          dclp.vars[o_vsc + i] = {engine::VarType::Continuous, 0.0, std::max(0.0, vscs[i].cap_mw), "dcvsc"};
        for (int i = 0; i < nDd; ++i)
          dclp.vars[o_dd + i] = {engine::VarType::Continuous, -dcdcs[i].smax, dcdcs[i].smax, "dcdc"};

        std::vector<Eigen::Triplet<double>> eqT, inT;
        std::vector<double> beqV, binV;
        auto d_eq = [&](const std::vector<std::pair<int, double>>& t, double rhs) {
          const int r = static_cast<int>(beqV.size());
          for (const auto& [col, val] : t) if (std::abs(val) > 1e-12) eqT.emplace_back(r, col, val);
          beqV.push_back(rhs);
        };
        auto d_le = [&](const std::vector<std::pair<int, double>>& t, double rhs) {
          const int r = static_cast<int>(binV.size());
          for (const auto& [col, val] : t) if (std::abs(val) > 1e-12) inT.emplace_back(r, col, val);
          binV.push_back(rhs);
        };
        // Bus balance: (inflow - outflow) + src + vsc_in + dcdc_net + shed = demand
        std::vector<std::vector<std::pair<int, double>>> busTerms(ndcb);
        std::vector<double> busDemand(ndcb, 0.0);
        for (int i = 0; i < nBr; ++i) {
          busTerms[dcbrs[i].t].push_back({o_P + i, +1.0});
          busTerms[dcbrs[i].f].push_back({o_P + i, -1.0});
        }
        for (int i = 0; i < ndcb; ++i) busTerms[i].push_back({o_src + i, +1.0});
        for (int i = 0; i < nV; ++i) busTerms[vscs[i].dcb].push_back({o_vsc + i, +1.0});
        for (int i = 0; i < nDd; ++i) {
          busTerms[dcdcs[i].t].push_back({o_dd + i, +1.0});
          busTerms[dcdcs[i].f].push_back({o_dd + i, -1.0});
        }
        for (int i = 0; i < nL; ++i) {
          busTerms[dcload_pos[i]].push_back({o_shed + i, +1.0});
          busDemand[dcload_pos[i]] += dcload_mw[i];
        }
        for (int b = 0; b < ndcb; ++b) d_eq(busTerms[b], busDemand[b]);
        // Resistive voltage drop for each closed DC branch: v_f - v_t - 2 r P = 0
        for (int i = 0; i < nBr; ++i)
          d_eq({{o_v + dcbrs[i].f, 1.0}, {o_v + dcbrs[i].t, -1.0}, {o_P + i, -2.0 * dcbrs[i].r}}, 0.0);
        // VSC transfer budget: total AC->DC transfer <= AC surplus of its component
        std::unordered_map<int, std::vector<int>> vsc_by_comp;
        for (int i = 0; i < nV; ++i)
          if (vscs[i].accomp >= 0) vsc_by_comp[vscs[i].accomp].push_back(i);
        for (const auto& [comp, vlist] : vsc_by_comp) {
          auto sit = comp_ac_source_kw.find(comp);
          auto vit = comp_ac_served_kw.find(comp);
          const double surplus_mw = std::max(0.0,
              (sit != comp_ac_source_kw.end() ? sit->second : 0.0) -
              (vit != comp_ac_served_kw.end() ? vit->second : 0.0)) / 1000.0;
          std::vector<std::pair<int, double>> t;
          for (int i : vlist) t.push_back({o_vsc + i, 1.0});
          d_le(t, surplus_mw);
        }

        const int neq = static_cast<int>(beqV.size()), nin = static_cast<int>(binV.size());
        dclp.Aeq.resize(neq, nvar); dclp.beq.resize(neq);
        dclp.Aeq.setFromTriplets(eqT.begin(), eqT.end()); dclp.Aeq.makeCompressed();
        for (int r = 0; r < neq; ++r) dclp.beq[r] = beqV[r];
        dclp.A.resize(nin, nvar); dclp.b.resize(nin);
        dclp.A.setFromTriplets(inT.begin(), inT.end()); dclp.A.makeCompressed();
        for (int r = 0; r < nin; ++r) dclp.b[r] = binV[r];

        engine::SimplexOptions so;
        so.max_iter = 20000; so.feasibility_tol = 1e-8; so.optimality_tol = 1e-8; so.verbose = false;
        auto sr = engine::solve_lp_with_basis(dclp, so, nullptr);
        if (!sr.result.stats.success || sr.result.x.size() != nvar) return false;
        for (int i = 0; i < nL; ++i)
          out.shed_by_load[dcload_li[i]] =
              std::max(0.0, std::min(dcload_mw[i], sr.result.x[o_shed + i])) * 1000.0;
        return true;
      }();
    }

    if (!dc_pf_done) {
    // Proper per-component capacity check for DC loads
    std::unordered_map<int, double> dc_comp_demand;
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) continue;
      if (load_outaged(fault, c.loads[li])) continue;
      auto it = dc_comp.find(c.loads[li].bus);
      if (it != dc_comp.end()) dc_comp_demand[it->second] += std::max(0.0, c.loads[li].p_kw);
    }
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) continue;
      const double li_kw = std::max(0.0, c.loads[li].p_kw);
      if (load_outaged(fault, c.loads[li])) {
        out.shed_by_load[li] = li_kw;
        continue;
      }
      auto it = dc_comp.find(c.loads[li].bus);
      if (it == dc_comp.end()) { out.shed_by_load[li] = li_kw; continue; }
      const int cid = it->second;
      const double dc_cap = comp_dc_source_kw.count(cid) ? comp_dc_source_kw.at(cid) : 0.0;
      const double ac_cap = comp_ac_source_kw.count(cid) ? comp_ac_source_kw.at(cid) : 0.0;
      const double ac_served = comp_ac_served_kw.count(cid) ? comp_ac_served_kw.at(cid) : 0.0;
      const double ac_surplus = std::max(0.0, ac_cap - ac_served);
      const double transfer_cap = comp_vsc_transfer_kw.count(cid) ? comp_vsc_transfer_kw.at(cid) : 0.0;
      const double cap = dc_cap + std::min(ac_surplus, transfer_cap);
      const double demand = dc_comp_demand.count(cid) ? dc_comp_demand.at(cid) : 0.0;
      if (cap >= demand - 1e-6) {
        out.shed_by_load[li] = 0.0;
      } else {
        // Proportional shed: each DC load gets shed proportional to its share
        const double ratio = demand > 1e-9 ? (demand - cap) / demand : 1.0;
        out.shed_by_load[li] = li_kw * std::min(1.0, std::max(0.0, ratio));
      }
    }
    }  // if (!dc_pf_done)
  } else {
    for (int i = 0; i < n_dc_bus; ++i) {
      const double shed_mw = std::clamp(res_x[off_dc_shed + i], 0.0,
                                        dc_demand[static_cast<size_t>(i)]);
      if (dc_demand[static_cast<size_t>(i)] <= 1e-12) continue;
      for (int li = 0; li < nd_total; ++li) {
        if (load_dc_bus[static_cast<size_t>(li)] != i ||
            load_outaged(fault, c.loads[static_cast<size_t>(li)]))
          continue;
        const double load_mw =
            std::max(0.0, c.loads[static_cast<size_t>(li)].p_kw) / 1000.0;
        out.shed_by_load[static_cast<size_t>(li)] =
            shed_mw * load_mw / dc_demand[static_cast<size_t>(i)] * 1000.0;
      }
    }
  }

  for (int li = 0; li < nd_total; ++li) {
    if (load_outaged(fault, c.loads[li]))
      out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
  }

  out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
  return out;
}

StageSolve solve_stage2_by_topology_enumeration(
    const NativeCase& c, const FaultLine& fault, int max_sw_ops,
    const std::unordered_set<int>& unavailable_ties,
    const std::unordered_set<int>& forced_open_switches,
    const std::vector<double>* storage_energy_available_mwh,
    double stage_duration_hr) {
  // Legacy normally-open branches have no controlling switch identifier and
  // cannot be represented by the auditable subset enumeration.
  if (std::any_of(c.sys.ac.branches.begin(), c.sys.ac.branches.end(),
                  [](const ACBranch& branch) { return !branch.in_service; })) {
    return solve_stage_milp(c, fault, 2, max_sw_ops, &unavailable_ties,
                            &forced_open_switches, nullptr, false,
                            storage_energy_available_mwh, stage_duration_hr);
  }

  std::vector<int> candidates;
  for (const auto& sw : c.sys.ac.switches) {
    if (!eligible_restoration_tie(sw) || unavailable_ties.count(sw.index) > 0 ||
        forced_open_switches.count(sw.index) > 0)
      continue;
    candidates.push_back(sw.index);
  }
  const int limit = std::min<int>(
      max_sw_ops == INT_MAX ? static_cast<int>(candidates.size())
                            : std::max(0, max_sw_ops),
      static_cast<int>(candidates.size()));
  // Bound the exact subset count. Larger restoration spaces retain the generic
  // MILP path; distribution cases with a handful of genuine ties use LPs.
  if (candidates.size() > 8) {
    return solve_stage_milp(c, fault, 2, max_sw_ops, &unavailable_ties,
                            &forced_open_switches, nullptr, false,
                            storage_energy_available_mwh, stage_duration_hr);
  }

  std::vector<std::vector<int>> plans(1);
  for (int switch_index : candidates) {
    const size_t existing = plans.size();
    for (size_t i = 0; i < existing; ++i) {
      if (static_cast<int>(plans[i].size()) >= limit) continue;
      auto plan = plans[i];
      plan.push_back(switch_index);
      plans.push_back(std::move(plan));
    }
  }
  if (plans.size() > 256) {
    return solve_stage_milp(c, fault, 2, max_sw_ops, &unavailable_ties,
                            &forced_open_switches, nullptr, false,
                            storage_energy_available_mwh, stage_duration_hr);
  }

  StageSolve best;
  bool found = false;
  for (const auto& closed_ties : plans) {
    StageSolve held;
    held.closed_tie_switch_indices = closed_ties;
    auto candidate = solve_stage_milp(
        c, fault, 3, max_sw_ops, &unavailable_ties, &forced_open_switches,
        &held, true, storage_energy_available_mwh, stage_duration_hr);
    if (candidate.status.rfind("success", 0) != 0) continue;
    if (!found || candidate.shed_kw < best.shed_kw - 1e-6 ||
        (std::abs(candidate.shed_kw - best.shed_kw) <= 1e-6 &&
         candidate.closed_tie_switch_indices.size() <
             best.closed_tie_switch_indices.size())) {
      best = std::move(candidate);
      found = true;
    }
  }
  if (found) return best;
  return solve_stage_milp(c, fault, 2, max_sw_ops, &unavailable_ties,
                          &forced_open_switches, nullptr, false,
                          storage_energy_available_mwh, stage_duration_hr);
}

void fill_summary(const NativeCase& c, ThreeStageReliabilityResult& r) {
  r.nb_ac = static_cast<int>(c.sys.ac.buses.size());
  r.nb_dc = static_cast<int>(c.sys.dc.buses.size());
  r.nb = r.nb_ac + r.nb_dc;
  r.nl_ac = static_cast<int>(c.sys.ac.branches.size());
  r.nl_dc = static_cast<int>(c.sys.dc.branches.size());
  r.nl_vsc = static_cast<int>(c.sys.vsc_converters.size());
  r.nl_sop = r.nl_vsc;  // each VSC converter is treated as one SOP port
  r.sop_config.clear();
  for (const auto& vsc : c.sys.vsc_converters) {
    ThreeStageSopConfig sc;
    sc.id        = vsc.index;
    sc.node_a    = vsc.bus_ac;
    sc.node_b    = vsc.bus_dc;
    sc.pmax_kw   = vsc.pmax_mw * 1000.0;
    sc.qmax_kw   = vsc.qmax_mvar * 1000.0;
    sc.efficiency = (vsc.eta > 0.0 && vsc.eta <= 1.0) ? vsc.eta : 0.98;
    r.sop_config.push_back(sc);
  }
  r.nl = r.nl_ac + r.nl_dc + r.nl_vsc;
  r.nd = static_cast<int>(c.loads.size());
  r.ng = static_cast<int>(c.sources.size());
  r.nmg = static_cast<int>(c.sys.microgrids.size());
}

void run_native_case(const NativeCase& c, ThreeStageReliabilityResult& r,
                     const ThreeStageReliabilityOptions& options,
                     const std::unordered_set<int>& unavailable_ties = {}) {
  int max_sw_ops = options.max_switch_operations;
  // Clamp negative values: negative has no sensible meaning; treat as 0
  // (no switching) rather than propagating a misleading negative count string.
  if (max_sw_ops < 0) max_sw_ops = 0;
  fill_summary(c, r);
  r.nodal_eens_kwh_yr.assign(c.loads.size(), 0.0);
  r.nodal_cif.assign(c.loads.size(), 0.0);
  r.nodal_cid_min.assign(c.loads.size(), 0.0);
  r.faults.clear();
  r.saifi = 0.0;
  r.saidi_min = 0.0;
  r.eens_kwh_yr = 0.0;
  r.eens_cost = 0.0;
  r.worst_line = 0;
  r.protection_configuration_applied =
      c.protection_configuration_applied;
  r.protection_rows_applied = c.protection_rows_applied;
  r.protection_scenarios_generated = c.protection_scenarios_generated;
  r.initiating_fault_frequency_per_year =
      c.initiating_fault_frequency_per_year;
  r.sustained_fault_frequency_per_year =
      c.sustained_fault_frequency_per_year;
  r.transient_reclose_frequency_per_year =
      c.transient_reclose_frequency_per_year;
  r.protection_configuration_limitations =
      c.protection_configuration_limitations;

  double total_customers = 0.0;
  for (const auto& ld : c.loads) total_customers += std::max(1.0, ld.customers);
  total_customers = std::max(1.0, total_customers);

  double weighted_interruptions = 0.0;
  double weighted_duration_min = 0.0;
  double max_pls = -1.0;
  int worst = 0;

  struct FaultStageEvaluation {
    ProtectionInterlockPrecheck interlock;
    StageSolve s1;
    StageSolve s2;
    StageSolve s3;
  };
  struct BaselineStageEvaluation {
    StageSolve s1;
    StageSolve s2;
    StageSolve s3;
  };
  std::vector<BaselineStageEvaluation> n0_evaluated(c.faults.size());
  std::unordered_map<std::string, BaselineStageEvaluation> n0_cache;
  const auto n0_key = [&](const FaultLine& fault) {
    if (c.sys.ac.storage.empty() && c.sys.dc.storage.empty())
      return std::string("no-storage");
    std::ostringstream os;
    os.precision(12);
    os << std::fixed << fault.tau_iso_hr << ':' << fault.tau_sw_hr << ':'
       << fault.tau_rep_hr;
    return os.str();
  };
  const auto evaluate_n0 = [&](const FaultLine& durations) {
    BaselineStageEvaluation baseline;
    FaultLine healthy;
    healthy.kind = FaultKind::ACBranch;
    healthy.index = -1;
    std::vector<double> energy(
        c.sys.ac.storage.size() + c.sys.mobile_storage.size() +
            c.sys.dc.storage.size(),
        0.0);
    for (size_t i = 0; i < c.sys.ac.storage.size(); ++i)
      energy[i] = initial_storage_deliverable_energy_mwh(c.sys.ac.storage[i]);
    for (size_t i = 0; i < c.sys.mobile_storage.size(); ++i)
      energy[c.sys.ac.storage.size() + i] =
          initial_storage_deliverable_energy_mwh(c.sys.mobile_storage[i]);
    const size_t dc_offset = c.sys.ac.storage.size() +
                             c.sys.mobile_storage.size();
    for (size_t i = 0; i < c.sys.dc.storage.size(); ++i)
      energy[dc_offset + i] =
          initial_storage_deliverable_energy_mwh(c.sys.dc.storage[i]);
    const auto consume = [&](const StageSolve& solve) {
      if (solve.status.rfind("success", 0) != 0) return;
      for (size_t i = 0; i < energy.size() &&
                         i < solve.storage_energy_used_mwh.size(); ++i)
        energy[i] = std::max(0.0, energy[i] - solve.storage_energy_used_mwh[i]);
    };
    const auto solve_healthy_stage = [&](double duration_hr) {
      auto solve = solve_stage_milp(c, healthy, 1, INT_MAX, nullptr, nullptr,
                                    nullptr, true, &energy, duration_hr);
      if (solve.status.rfind("success", 0) != 0)
        solve = solve_stage_milp(c, healthy, 1, INT_MAX, nullptr, nullptr,
                                 nullptr, false, &energy, duration_hr);
      consume(solve);
      return solve;
    };
    baseline.s1 = solve_healthy_stage(durations.tau_iso_hr);
    baseline.s2 = solve_healthy_stage(durations.tau_sw_hr);
    baseline.s3 = solve_healthy_stage(durations.tau_rep_hr);
    return baseline;
  };
  for (size_t i = 0; i < c.faults.size(); ++i) {
    const std::string key = n0_key(c.faults[i]);
    auto [it, inserted] = n0_cache.try_emplace(key);
    if (inserted) it->second = evaluate_n0(c.faults[i]);
    n0_evaluated[i] = it->second;
  }
  std::vector<FaultStageEvaluation> evaluated(c.faults.size());
  const int automatic_cap = 4;
  const int requested_workers = options.parallel_threads > 0
      ? options.parallel_threads
      : std::min(automatic_cap, hacdcpf::util::detect_hardware_threads());
  r.parallel_execution = hacdcpf::util::make_parallel_execution_info(
      options.enable_parallel, requested_workers,
      static_cast<int>(c.faults.size()),
      "parallel-three-stage-contingencies", "native-thread-pool");
  r.parallel_execution.requested_threads = options.parallel_threads;
  r.parallel_execution.batch_size = static_cast<int>(c.faults.size());

  const auto evaluate_fault = [&](size_t fault_pos) {
    const auto& fault = c.faults[fault_pos];
    auto& item = evaluated[fault_pos];
    std::vector<double> storage_energy(
        c.sys.ac.storage.size() + c.sys.mobile_storage.size() +
            c.sys.dc.storage.size(),
        0.0);
    for (size_t i = 0; i < c.sys.ac.storage.size(); ++i)
      storage_energy[i] = initial_storage_deliverable_energy_mwh(
          c.sys.ac.storage[i]);
    for (size_t i = 0; i < c.sys.mobile_storage.size(); ++i)
      storage_energy[c.sys.ac.storage.size() + i] =
          initial_storage_deliverable_energy_mwh(c.sys.mobile_storage[i]);
    const size_t dc_offset = c.sys.ac.storage.size() +
                             c.sys.mobile_storage.size();
    for (size_t i = 0; i < c.sys.dc.storage.size(); ++i)
      storage_energy[dc_offset + i] =
          initial_storage_deliverable_energy_mwh(c.sys.dc.storage[i]);
    const auto consume_storage_energy = [&](const StageSolve& stage_solve) {
      if (stage_solve.status.rfind("success", 0) != 0) return;
      for (size_t i = 0; i < storage_energy.size() &&
                         i < stage_solve.storage_energy_used_mwh.size(); ++i) {
        storage_energy[i] = std::max(
            0.0, storage_energy[i] - stage_solve.storage_energy_used_mwh[i]);
      }
    };
    item.interlock = precheck_protection_interlocks(c.sys, fault);
    if (fault.restoration_forbidden) {
      item.interlock.valid = false;
      item.interlock.message =
          "configured primary and backup protection both failed; restoration is blocked until fault clearance";
    }
    item.s1 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                               &item.interlock.forced_open_switch_indices,
                               nullptr, true, &storage_energy,
                               fault.tau_iso_hr);
    if (item.s1.status.rfind("success", 0) != 0) {
      // Meshed or otherwise non-fixed topology: preserve the generic MILP.
      item.s1 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                                 &item.interlock.forced_open_switch_indices,
                                 nullptr, false, &storage_energy,
                                 fault.tau_iso_hr);
    }
    consume_storage_energy(item.s1);
    if (item.interlock.valid) {
      item.s2 = solve_stage2_by_topology_enumeration(
          c, fault, max_sw_ops, unavailable_ties,
          item.interlock.forced_open_switch_indices, &storage_energy,
          fault.tau_sw_hr);
      consume_storage_energy(item.s2);
      if (options.revalidate_stage3_plan || !c.sys.ac.storage.empty() ||
          !c.sys.mobile_storage.empty() || !c.sys.dc.storage.empty()) {
        // Diagnostic replay: Stage 3 is the repair window with the exact
        // Stage-2 switching plan held as fixed binary bounds. Storage always
        // requires this replay because its remaining MWh and duration-limited
        // discharge capability differ from Stage 2.
        item.s3 = solve_stage_milp(c, fault, 3, max_sw_ops, &unavailable_ties,
                                   &item.interlock.forced_open_switch_indices,
                                   &item.s2, true, &storage_energy,
                                   fault.tau_rep_hr);
      } else {
        // The fault stays out throughout Stage 3 and demand/physical limits do
        // not change. The Stage-2 optimum therefore remains the exact optimum
        // for the same topology in the repair window; copying it removes one
        // redundant model build and solve per contingency.
        item.s3 = item.s2;
      }
    } else {
      // Restoration switching is blocked, but storage energy still advances
      // through the remaining durations on the unchanged Stage-1 topology.
      item.s2 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                                 &item.interlock.forced_open_switch_indices,
                                 nullptr, true, &storage_energy,
                                 fault.tau_sw_hr);
      if (item.s2.status.rfind("success", 0) != 0)
        item.s2 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                                   &item.interlock.forced_open_switch_indices,
                                   nullptr, false, &storage_energy,
                                   fault.tau_sw_hr);
      consume_storage_energy(item.s2);
      item.s3 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                                 &item.interlock.forced_open_switch_indices,
                                 nullptr, true, &storage_energy,
                                 fault.tau_rep_hr);
      if (item.s3.status.rfind("success", 0) != 0)
        item.s3 = solve_stage_milp(c, fault, 1, INT_MAX, nullptr,
                                   &item.interlock.forced_open_switch_indices,
                                   nullptr, false, &storage_energy,
                                   fault.tau_rep_hr);
      item.s2.status = "failed (protection interlock)";
      item.s3.status = "failed (protection interlock)";
      item.s2.switch_sequence_valid = false;
      item.s3.switch_sequence_valid = false;
      item.s2.switch_sequence_message = item.interlock.message;
      item.s3.switch_sequence_message = item.interlock.message;
    }
  };
  if (r.parallel_execution.effective) {
    hacdcpf::util::ThreadPool pool(r.parallel_execution.resolved_workers,
                                   kReliabilityWorkerStackBytes);
    pool.parallel_for_dynamic(c.faults.size(), evaluate_fault,
                              r.parallel_execution.resolved_workers);
    r.parallel_execution.actual_parallel_evaluations =
        static_cast<long long>(c.faults.size());
  } else {
    for (size_t i = 0; i < c.faults.size(); ++i) evaluate_fault(i);
    r.parallel_execution.serial_evaluations =
        static_cast<long long>(c.faults.size());
    if (options.enable_parallel)
      r.parallel_execution.guard_reason =
          hacdcpf::util::insufficient_work_reason(r.parallel_execution);
  }

  for (size_t fault_pos = 0; fault_pos < c.faults.size(); ++fault_pos) {
    const auto& fault = c.faults[fault_pos];
    const auto& interlock = evaluated[fault_pos].interlock;
    const auto& s1 = evaluated[fault_pos].s1;
    const auto& s2 = evaluated[fault_pos].s2;
    const auto& s3 = evaluated[fault_pos].s3;
    const auto& n0 = n0_evaluated[fault_pos];
    const auto incremental_shed = [&](const StageSolve& fault_stage,
                                      const StageSolve& healthy_stage) {
      std::vector<double> incremental(c.loads.size(), 0.0);
      double positive_sum = 0.0;
      for (size_t i = 0; i < incremental.size(); ++i) {
        const double raw = i < fault_stage.shed_by_load.size()
            ? fault_stage.shed_by_load[i] : 0.0;
        const double healthy = i < healthy_stage.shed_by_load.size()
            ? healthy_stage.shed_by_load[i] : 0.0;
        incremental[i] = std::max(0.0, raw - healthy);
        positive_sum += incremental[i];
      }
      const double target = std::max(0.0, fault_stage.shed_kw -
                                             healthy_stage.shed_kw);
      const double scale = positive_sum > 1e-12 ? target / positive_sum : 0.0;
      for (double& shed : incremental) shed *= scale;
      return incremental;
    };
    const auto incremental1 = incremental_shed(s1, n0.s1);
    const auto incremental2 = incremental_shed(s2, n0.s2);
    const auto incremental3 = incremental_shed(s3, n0.s3);
    const auto shed_sum = [](const std::vector<double>& values) {
      return std::accumulate(values.begin(), values.end(), 0.0);
    };

    ThreeStageFaultDetail d;
    d.line_id = fault.id;
    d.component_type = fault_kind_type(fault.kind);
    d.component_index = fault.index;
    d.ac = fault.ac;
    d.from_bus = fault.from_bus;
    d.to_bus = fault.to_bus;
    d.failure_rate = fault.failure_rate;
    d.initiating_failure_rate = fault.initiating_failure_rate > 0.0
        ? fault.initiating_failure_rate : fault.failure_rate;
    d.scenario_probability = fault.scenario_probability;
    d.protection_scenario = fault.protection_scenario;
    d.protection_id = fault.protection_id;
    d.primary_device_id = fault.primary_device_id;
    d.backup_device_id = fault.backup_device_id;
    d.reclose_success_probability = fault.reclose_success_probability;
    d.primary_failure_probability = fault.primary_failure_probability;
    d.backup_failure_probability = fault.backup_failure_probability;
    d.clearing_time_s = fault.clearing_time_s;
    d.protection_zone_component_ids =
        fault.protection_zone_component_ids;
    d.stage1_status = s1.status;
    d.stage2_status = s2.status;
    d.stage3_status = s3.status;
    d.stage1_mip_gap = s1.mip_gap;
    d.stage2_mip_gap = s2.mip_gap;
    d.stage3_mip_gap = s3.mip_gap;
    auto is_success_like = [](const std::string& s) { return s.rfind("success", 0) == 0; };
    const bool all_success = is_success_like(n0.s1.status) &&
                 is_success_like(n0.s2.status) &&
                 is_success_like(n0.s3.status) &&
                 is_success_like(s1.status) &&
                 is_success_like(s2.status) &&
                 is_success_like(s3.status);
    const bool any_approx = n0.s1.status == "success (approximate)" ||
                n0.s2.status == "success (approximate)" ||
                n0.s3.status == "success (approximate)" ||
                s1.status == "success (approximate)" ||
                s2.status == "success (approximate)" ||
                s3.status == "success (approximate)";
    d.status = !all_success ? "failed" : (any_approx ? "success (approximate)" : "success");
    d.raw_pls_stage1 = s1.shed_kw;
    d.raw_pls_stage2 = s2.shed_kw;
    d.raw_pls_stage3 = s3.shed_kw;
    d.n0_pls_stage1 = n0.s1.shed_kw;
    d.n0_pls_stage2 = n0.s2.shed_kw;
    d.n0_pls_stage3 = n0.s3.shed_kw;
    d.pls_stage1 = shed_sum(incremental1);
    d.pls_stage2 = shed_sum(incremental2);
    d.pls_stage3 = shed_sum(incremental3);
    d.pls_total = d.pls_stage1 + d.pls_stage2 + d.pls_stage3;
    d.tau_iso_hr = fault.tau_iso_hr;
    d.tau_sw_hr = fault.tau_sw_hr;
    d.tau_rep_hr = fault.tau_rep_hr;
    const auto storage_energy_sum = [](const std::vector<double>& values) {
      return std::accumulate(values.begin(), values.end(), 0.0);
    };
    for (const auto& storage : c.sys.ac.storage)
      d.storage_energy_initial_mwh +=
          initial_storage_deliverable_energy_mwh(storage);
    for (const auto& storage : c.sys.mobile_storage)
      d.storage_energy_initial_mwh +=
          initial_storage_deliverable_energy_mwh(storage);
    for (const auto& storage : c.sys.dc.storage)
      d.storage_energy_initial_mwh +=
          initial_storage_deliverable_energy_mwh(storage);
    d.storage_energy_used_stage1_mwh =
        storage_energy_sum(s1.storage_energy_used_mwh);
    d.storage_energy_used_stage2_mwh =
        storage_energy_sum(s2.storage_energy_used_mwh);
    d.storage_energy_used_stage3_mwh =
        storage_energy_sum(s3.storage_energy_used_mwh);
    d.storage_energy_remaining_mwh = std::max(
        0.0, d.storage_energy_initial_mwh -
                 d.storage_energy_used_stage1_mwh -
                 d.storage_energy_used_stage2_mwh -
                 d.storage_energy_used_stage3_mwh);
    // P1c: per-component stage durations from the faulted component's MTTR.
    // Stage 1 = tau_iso, Stage 2 = tau_sw, Stage 3 = tau_rep (repair).
    d.objective = d.pls_stage1 * fault.tau_iso_hr
                + d.pls_stage2 * fault.tau_sw_hr
                + d.pls_stage3 * fault.tau_rep_hr;
    d.duration_hr = (d.pls_stage1 > 1e-6 ? fault.tau_iso_hr : 0.0)
                  + (d.pls_stage2 > 1e-6 ? fault.tau_sw_hr : 0.0)
                  + (d.pls_stage3 > 1e-6 ? fault.tau_rep_hr : 0.0);
    d.ens_kwh = d.objective;
    d.eens_contribution_mwh_yr = fault.failure_rate * d.ens_kwh / 1000.0;
    d.lole_contribution_hr_yr = fault.failure_rate * d.duration_hr;
    d.lolf_contribution_occ_yr = d.duration_hr > 0.0 ? fault.failure_rate : 0.0;
    d.protection_interlock_valid = interlock.valid;
    d.restoration_milp_admitted = interlock.valid;
    d.stage3_switch_plan_held = interlock.valid &&
        s2.closed_tie_switch_indices == s3.closed_tie_switch_indices &&
        s2.closed_legacy_branch_indices == s3.closed_legacy_branch_indices;
    d.switching_sequence_valid = interlock.valid &&
                                 s2.switch_sequence_valid &&
                                 s3.switch_sequence_valid &&
                                 s2.closed_tie_switch_indices ==
                                     s3.closed_tie_switch_indices &&
                                 s2.closed_legacy_branch_indices ==
                                     s3.closed_legacy_branch_indices;
    d.switching_sequence_message = !interlock.valid
        ? interlock.message
        : (d.switching_sequence_valid
               ? s2.switch_sequence_message
               : "Stage-2 and repair-window switching plans are inconsistent");
    int action_order = 0;
    bool isolation_valid = true;
    if (!fault.protection_id.empty()) {
      const bool backup_action =
          fault.protection_scenario != "primary_cleared";
      const std::string device_id = backup_action
          ? fault.backup_device_id : fault.primary_device_id;
      ThreeStageSwitchAction action;
      action.sequence_order = ++action_order;
      action.action = "trip";
      action.purpose = backup_action
          ? "configured_backup_fault_clearance"
          : "configured_primary_fault_clearance";
      action.operation_time_s = fault.clearing_time_s;
      if (device_id.empty()) {
        action.switch_name = "abstract upstream backup protection";
        action.switch_type = "unresolved_backup_device";
      }
      const auto bind_switch = [&](const Switch& sw) {
        action.switch_index = sw.index;
        action.switch_name = sw.name;
        action.switch_type = switch_type_str(sw.switch_type);
        action.bus_from = sw.bus_from;
        action.bus_to = sw.bus_to;
        const auto caps = effective_switch_capabilities(sw);
        action.validated = sw.in_service && sw.closed && !sw.locked_closed &&
                           caps.can_interrupt_fault_current;
      };
      if (device_id.rfind("ac_switch:", 0) == 0) {
        const int stable_index = std::stoi(device_id.substr(10));
        const auto it = std::find_if(
            c.sys.ac.switches.begin(), c.sys.ac.switches.end(),
            [&](const Switch& sw) { return sw.index == stable_index; });
        if (it != c.sys.ac.switches.end()) bind_switch(*it);
      } else if (device_id.rfind("ac_circuit_breaker:", 0) == 0) {
        const int stable_index = std::stoi(device_id.substr(19));
        const auto it = std::find_if(
            c.sys.ac.circuit_breakers.begin(),
            c.sys.ac.circuit_breakers.end(),
            [&](const CircuitBreaker& breaker) {
              return breaker.index == stable_index;
            });
        if (it != c.sys.ac.circuit_breakers.end()) {
          action.switch_index = it->index;
          action.switch_name = it->name;
          action.switch_type = "CircuitBreaker";
          action.bus_from = it->bus_from;
          action.bus_to = it->bus_to;
          action.validated = it->in_service && it->closed;
        }
      } else if (device_id.rfind("dc_circuit_breaker:", 0) == 0) {
        const int stable_index = std::stoi(device_id.substr(19));
        const auto it = std::find_if(
            c.sys.dc.dc_circuit_breakers.begin(),
            c.sys.dc.dc_circuit_breakers.end(),
            [&](const DCCircuitBreaker& breaker) {
              return breaker.index == stable_index;
            });
        if (it != c.sys.dc.dc_circuit_breakers.end()) {
          action.switch_index = it->index;
          action.switch_name = it->name;
          action.switch_type = "DCCircuitBreaker";
          action.bus_from = it->bus_from;
          action.bus_to = it->bus_to;
          action.validated = it->in_service && it->closed;
        }
      }
      action.validation_message = action.validated
          ? "configured protection device resolved by stable component identity"
          : "configured protection device is absent, open, locked, or lacks fault-interruption capability";
      if (!action.validated) {
        isolation_valid = false;
        d.switching_sequence_valid = false;
      }
      d.switching_sequence.push_back(std::move(action));
    }
    std::vector<const Switch*> protection_actions;
    std::vector<const Switch*> boundary_actions;
    std::unordered_map<int, const Switch*> switch_by_index;
    std::unordered_map<int, int> boundary_upstream_index;
    for (const auto& sw : c.sys.ac.switches)
      switch_by_index[sw.index] = &sw;
    const auto add_protection_action = [&](const Switch* sw) {
      if (std::none_of(protection_actions.begin(), protection_actions.end(),
                       [&](const Switch* existing) {
                         return existing->index == sw->index;
                       }))
        protection_actions.push_back(sw);
    };
    std::string controlled_type;
    int controlled_index = -1;
    if (fault.kind == FaultKind::ACBranch && fault.index >= 0 &&
        fault.index < static_cast<int>(c.sys.ac.branches.size())) {
      controlled_type = "ac_branch";
      controlled_index = c.sys.ac.branches[fault.index].index;
    } else if (fault.kind == FaultKind::Transformer2W && fault.index >= 0 &&
               fault.index < static_cast<int>(c.sys.ac.transformers_2w.size())) {
      controlled_type = "transformer_2w";
      controlled_index = c.sys.ac.transformers_2w[fault.index].index;
    }
    if (controlled_index >= 0) {
      for (const auto& sw : c.sys.ac.switches) {
        const bool generic_match =
            sw.controlled_element_type == controlled_type &&
            sw.controlled_element_index == controlled_index;
        const bool legacy_branch_match = controlled_type == "ac_branch" &&
            sw.controlled_branch_index == controlled_index;
        if (!sw.in_service || !sw.closed ||
            (!generic_match && !legacy_branch_match))
          continue;
        if (sw.role == SwitchRole::Protection) add_protection_action(&sw);
        else if (sw.role == SwitchRole::Sectionalizing ||
                 sw.role == SwitchRole::Isolation)
          boundary_actions.push_back(&sw);
      }
    }
    for (const Switch* boundary : boundary_actions) {
      if (!effective_switch_capabilities(*boundary)
               .requires_deenergized_operation)
        continue;
      int upstream_index = boundary->upstream_protective_switch_index;
      if (upstream_index < 0)
        upstream_index =
            boundary->sectionalizer_protection.upstream_switch_index;
      if (upstream_index < 0) continue;
      boundary_upstream_index[boundary->index] = upstream_index;
      const auto upstream = switch_by_index.find(upstream_index);
      if (upstream != switch_by_index.end() && upstream->second->in_service &&
          upstream->second->closed)
        add_protection_action(upstream->second);
    }
    std::unordered_set<int> cleared_by_protection;
    const auto append_isolation_action = [&](const Switch& sw,
                                             bool protection_action,
                                             bool upstream_cleared) {
      const auto caps = effective_switch_capabilities(sw);
      ThreeStageSwitchAction action;
      action.sequence_order = ++action_order;
      action.switch_index = sw.index;
      action.switch_name = sw.name;
      action.switch_type = switch_type_str(sw.switch_type);
      action.action = protection_action ? "trip" : "open";
      action.purpose = protection_action ? "fault_clearance"
                                         : "fault_section_isolation";
      action.bus_from = sw.bus_from;
      action.bus_to = sw.bus_to;
      action.operation_time_s = sw.t_open_s > 0.0
          ? sw.t_open_s : sw.t_operation_s;
      action.validated = !sw.locked_closed &&
          (protection_action ? (sw.role == SwitchRole::Protection &&
                                caps.can_interrupt_fault_current)
                             : (caps.can_interrupt_load_current ||
                                (caps.requires_deenergized_operation &&
                                 upstream_cleared)));
      action.validation_message = action.validated
          ? (protection_action
                 ? "protection device is rated to interrupt fault current"
                 : "boundary device opens after upstream fault clearance")
          : (caps.requires_deenergized_operation && !upstream_cleared
                 ? "de-energized operation requires a validated upstream protective trip"
                 : "device capability or lock state does not permit this isolation action");
      if (!action.validated) {
        isolation_valid = false;
        d.switching_sequence_valid = false;
      } else if (protection_action) {
        cleared_by_protection.insert(sw.index);
      }
      d.switching_sequence.push_back(std::move(action));
    };
    for (const Switch* sw : protection_actions)
      append_isolation_action(*sw, true, false);
    for (const Switch* sw : boundary_actions) {
      const auto dependency = boundary_upstream_index.find(sw->index);
      const bool upstream_cleared = dependency == boundary_upstream_index.end()
          ? !cleared_by_protection.empty()
          : cleared_by_protection.count(dependency->second) > 0;
      append_isolation_action(*sw, false, upstream_cleared);
    }
    d.fault_isolation_explicit = !fault.protection_id.empty() ||
                                 !protection_actions.empty() ||
                                 !boundary_actions.empty();
    d.fault_isolation_message = d.fault_isolation_explicit
        ? (isolation_valid
               ? "controlled-equipment protection/isolation actions are explicit and validated"
               : "controlled-equipment isolation contains an invalid action")
        : "fault isolation represented by forced-open failed component; no controlled-equipment switch binding";
    for (int switch_index : s2.closed_tie_switch_indices) {
      ThreeStageSwitchAction action;
      action.sequence_order = ++action_order;
      action.switch_index = switch_index;
      action.action = "close";
      action.purpose = "service_restoration_tie_close";
      const auto it = std::find_if(
          c.sys.ac.switches.begin(), c.sys.ac.switches.end(),
          [switch_index](const Switch& sw) { return sw.index == switch_index; });
      if (it == c.sys.ac.switches.end()) {
        action.validated = false;
        action.validation_message = "selected topology edge has no switch metadata";
        d.switching_sequence_valid = false;
        d.switching_sequence_message = action.validation_message;
      } else {
        action.switch_name = it->name;
        action.switch_type = switch_type_str(it->switch_type);
        action.bus_from = it->bus_from;
        action.bus_to = it->bus_to;
        action.operation_time_s = it->t_close_s > 0.0
            ? it->t_close_s : it->t_operation_s;
        action.validated = eligible_restoration_tie(*it);
        action.validation_message = action.validated
            ? "eligible tie; ordered closure preserves the accepted radial forest"
            : "switch capability or role forbids restoration closing";
        if (!action.validated) {
          d.switching_sequence_valid = false;
          d.switching_sequence_message = action.validation_message;
        }
      }
      d.switching_sequence.push_back(std::move(action));
    }
    for (int branch_pos : s2.closed_legacy_branch_indices) {
      ThreeStageSwitchAction action;
      action.sequence_order = ++action_order;
      action.switch_index = -1;
      action.switch_type = "normally_open_branch";
      action.action = "close";
      action.purpose = "legacy_normally_open_branch_close";
      if (branch_pos >= 0 &&
          branch_pos < static_cast<int>(c.sys.ac.branches.size())) {
        const auto& branch = c.sys.ac.branches[branch_pos];
        action.switch_name = branch.name;
        action.bus_from = branch.from_bus;
        action.bus_to = branch.to_bus;
      }
      action.validated = false;
      action.validation_message =
          "branch restoration edge has no controlling switch capability metadata";
      d.switching_sequence.push_back(std::move(action));
      d.switching_sequence_valid = false;
      d.switching_sequence_message =
          "sequence contains a legacy branch action without switch capability metadata";
    }
    d.psop1 = s1.vsc_dispatch_kw;
    d.psop2 = s2.vsc_dispatch_kw;
    d.psop3 = s3.vsc_dispatch_kw;
    r.faults.push_back(d);

    if (d.pls_total > max_pls) {
      max_pls = d.pls_total;
      worst = fault.id;
    }

    for (size_t i = 0; i < c.loads.size(); ++i) {
      const double ens = fault.failure_rate *
          (incremental1[i] * fault.tau_iso_hr +
           incremental2[i] * fault.tau_sw_hr +
           incremental3[i] * fault.tau_rep_hr);
      r.nodal_eens_kwh_yr[i] += ens;
      r.eens_kwh_yr += ens;
      const bool interrupted = incremental1[i] > 1e-6 ||
                               incremental2[i] > 1e-6 ||
                               incremental3[i] > 1e-6;
      if (interrupted) {
        const double cust = std::max(1.0, c.loads[i].customers);
        const double duration_min = ((incremental1[i] > 1e-6 ? fault.tau_iso_hr : 0.0) +
                                     (incremental2[i] > 1e-6 ? fault.tau_sw_hr  : 0.0) +
                                     (incremental3[i] > 1e-6 ? fault.tau_rep_hr : 0.0)) * 60.0;
        r.nodal_cif[i] += fault.failure_rate;
        r.nodal_cid_min[i] += fault.failure_rate * duration_min;
        weighted_interruptions += fault.failure_rate * cust;
        weighted_duration_min += fault.failure_rate * duration_min * cust;
      }
    }
  }

  r.worst_line = worst > 0 ? worst : (r.nl_ac + r.nl_dc > 0 ? 1 : 0);
  r.saifi = weighted_interruptions / total_customers;
  r.saidi_min = weighted_duration_min / total_customers;
  r.eens_cost = r.eens_kwh_yr * kReliabilityVoll;

  // Populate model limitations so callers can surface the linearization
  // boundary separately from actual component-coverage gaps.
  const bool has_dc_or_vsc = !c.sys.dc.buses.empty() || !c.sys.dc.branches.empty() ||
                             !c.sys.dc.loads.empty() || !c.sys.vsc_converters.empty() ||
                             !c.sys.dc.dcdc_converters.empty();
  const bool has_unmodelled_hybrid_devices =
      !c.sys.lcc_converters.empty() || !c.sys.energy_routers.empty();
  r.model_limitations =
      "Coupled AC/DC restoration MILP: the AC network uses finite-source LinDistFlow with energized-bus variables, "
      "explicit p_g/q_g source capacity bounds, strict commodity-flow radial forest, "
      "branch active/reactive flow limits, voltage bounds [vmin², vmax²], "
      "continuous load shed variables p^sh_i ∈ [0, p_d,i], "
      "and switch-count constraint"
      + (max_sw_ops == INT_MAX
           ? std::string(" (no limit)")
           : " (≤ " + std::to_string(max_sw_ops) + " operations per fault)")
      + ". "
        "Selected restoration switch closures are returned in execution order and "
        "checked against device role/capability plus the MILP radiality, voltage, and "
        "branch limits. Legacy out-of-service branch closures are reported as "
        "unvalidated. Controlled-equipment protection and interlock bindings are "
        "prechecked before restoration; validated trip/isolation devices are forced "
        "open in the MILP, invalid interlocks block restoration switching, and Stage-3 "
        "binary switch states are fixed to the accepted Stage-2 plan. "
        "Grid-following DER injects only in a component containing an upstream or "
        "grid-forming voltage anchor; it cannot root an outage island. AC and DC storage "
        "use explicit per-device discharge variables and sequentially carry "
        "deliverable MWh from isolation through switching and the repair window. "
        "Reported PLS and reliability indices are incremental to a matching N-0 "
        "healthy-state solve; raw fault-state and N-0 shed remain available for audit. "
        "N-1 contingency enumeration covers ACBranch and DCBranch outages by default; "
        "generator, static/renewable/PV generation, storage, microgrid, transformer, "
        "DC static/PV/storage DER, VSC/DC-DC converter, AC switch, and AC/DC circuit-breaker "
        "mobile storage, VPP, DC static/PV/storage DER, VSC/DC-DC converter, AC switch, "
        "and AC/DC circuit-breaker outages are enumerated only when their opt-in flags "
        "are set (converter faults remove coupled dispatch; switch/breaker faults force "
        "open their topology edge; load outage is not enumerated as a standalone fault). "
       "The DC network uses nodal active-power balance, squared-voltage drop, branch limits, "
       "energized-bus and radial-forest constraints. VSC and DC-DC transfers are bidirectional "
       "with constant efficiency and are co-optimized with AC/DC load pickup; psop reports the "
       "signed AC-to-DC VSC transfer. Converter fixed standby loss, quadratic conduction loss, "
       "DC-DC duty-ratio voltage conversion, and nonlinear AC apparent-power circles remain outside "
       "this linear reliability model. Missing reactive demand is reconstructed at 0.9 power factor; "
       "quadratic branch losses are dropped by LinDistFlow.";
  if (!c.include_dc_power_flow && has_dc_or_vsc)
    r.model_limitations +=
        " DC power flow was explicitly disabled, so DC consequences use the legacy connectivity/capacity fallback.";
  if (has_unmodelled_hybrid_devices)
    r.model_limitations +=
        " LCC converters and multi-port energy routers require their dedicated commutation/port-balance model and are not represented by the VSC equations.";
  if (c.protection_configuration_applied) {
    r.model_limitations +=
        " Session protection configuration is applied by mutually exclusive "
        "sustained-event scenarios: successful automatic reclose is classified "
        "as momentary and excluded from IEEE-1366 sustained SAIFI/SAIDI/EENS; "
        "primary command/trip and contact-opening failures are independent; "
        "configured backup zones are topology/load consequence sets, not relay "
        "pickup, time-current, directional, DER-FRT, or waveform simulations.";
  }
  for (const auto& limitation : c.protection_configuration_limitations)
    r.model_limitations += " " + limitation;

  const bool coupled_scope_valid =
      (!has_dc_or_vsc || c.include_dc_power_flow) &&
      !has_unmodelled_hybrid_devices;
  r.model_scope = has_dc_or_vsc
      ? (c.include_dc_power_flow
             ? "coupled-acdc-lindistflow-restoration-milp"
             : "ac-lindistflow-milp+dc-connectivity-fallback")
      : "ac-lindistflow-milp";
  r.validity = ThreeStageReliabilityResult::ValidityFlags{
      .branch_flow_enforced = coupled_scope_valid,
      .voltage_constraints_enforced = coupled_scope_valid,
      .radial_topology_enforced = coupled_scope_valid,
      .sop_dispatch_optimised = c.include_dc_power_flow &&
                                !c.sys.vsc_converters.empty(),
      .dc_power_flow_enforced = c.include_dc_power_flow && has_dc_or_vsc,
      .restoration_milp_solved = coupled_scope_valid,
  };

  // r.ok is true only when every fault stage solved to a verified optimum and
  // the submitted system is inside the evaluator's full physical scope.  Hybrid
  // DC/VSC/SOP cases return metrics, but they use the documented connectivity
  // fallback and therefore are not exact full-system MILP results.
  // Any stage that returned "failed*" used conservative full-shed estimates;
  // those values are still accumulated into EENS but the result is flagged
  // so that callers know the metrics are upper-bound estimates, not exact.
  // M3: "success (approximate)" stages also clear r.ok — the gap was not
  // proven, so the shed values are feasible but potentially non-optimal.
  bool any_failed = false;
  for (const auto& fd : r.faults) {
    if (fd.status.rfind("failed", 0) == 0 ||
        fd.status == "success (approximate)") {
      any_failed = true;
      break;
    }
  }
  r.ok = !any_failed && coupled_scope_valid;
  if (!r.ok && r.error.empty()) {
    if (has_unmodelled_hybrid_devices) {
      r.error = "three-stage reliability does not map LCC or multi-port energy-router equations into the coupled restoration MILP";
    } else if (has_dc_or_vsc && !c.include_dc_power_flow) {
      r.error = "DC power flow was disabled; hybrid consequences use the connectivity/capacity fallback";
    } else if (any_failed) {
      r.error = "one or more fault stages failed or returned an approximate solution";
    }
  }
}

}  // anonymous namespace

ThreeStageReliabilityResult run_three_stage_reliability(
    const fs::path& case_json,
    const ThreeStageReliabilityOptions& options) {
  ThreeStageReliabilityResult result;
  if (!fs::exists(case_json)) {
    result.error = "case JSON does not exist: " + case_json.string();
    return result;
  }
  const std::string text = read_file_text(case_json);
  if (text.empty()) {
    result.error = "case JSON is empty or unreadable: " + case_json.string();
    return result;
  }
  try {
    HybridPowerSystem sys = io::from_json(text);
    ThreeStageReliabilityOptions effective_options = options;
    if (!effective_options.reliability_configuration.protection.empty() ||
        !effective_options.reliability_configuration.mode_overrides.empty()) {
      const auto validation = validate_reliability_configuration(
          sys, effective_options.reliability_configuration);
      if (!validation.ok()) {
        std::string message = "invalid reliability configuration";
        for (const auto& error : validation.errors) message += "; " + error;
        throw std::runtime_error(message);
      }
      effective_options.reliability_configuration =
          resolve_reliability_configuration(
              sys, std::move(effective_options.reliability_configuration));
    }
    NativeCase c = build_native_case(sys, effective_options);
    if (c.loads.empty()) {
      result.error = "case JSON contains no load points";
      return result;
    }
    if (c.faults.empty() && c.initiating_fault_frequency_per_year <= 0.0) {
      result.error = "case JSON contains no AC/DC branch contingencies";
      return result;
    }
    std::unordered_set<int> unavailable_ties(
        effective_options.unavailable_tie_switch_ids.begin(),
        effective_options.unavailable_tie_switch_ids.end());
    run_native_case(c, result, effective_options, unavailable_ties);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

ThreeStageReliabilityResult run_three_stage_reliability_from_string(
    const std::string& case_json_text,
    const ThreeStageReliabilityOptions& options) {
  ThreeStageReliabilityResult result;
  if (case_json_text.empty()) {
    result.error = "case JSON text is empty";
    return result;
  }
  try {
    HybridPowerSystem sys = io::from_json(case_json_text);
    ThreeStageReliabilityOptions effective_options = options;
    if (!effective_options.reliability_configuration.protection.empty() ||
        !effective_options.reliability_configuration.mode_overrides.empty()) {
      const auto validation = validate_reliability_configuration(
          sys, effective_options.reliability_configuration);
      if (!validation.ok()) {
        std::string message = "invalid reliability configuration";
        for (const auto& error : validation.errors) message += "; " + error;
        throw std::runtime_error(message);
      }
      effective_options.reliability_configuration =
          resolve_reliability_configuration(
              sys, std::move(effective_options.reliability_configuration));
    }
    NativeCase c = build_native_case(sys, effective_options);
    if (c.loads.empty()) {
      result.error = "case JSON contains no load points";
      return result;
    }
    if (c.faults.empty() && c.initiating_fault_frequency_per_year <= 0.0) {
      result.error = "case JSON contains no AC/DC branch contingencies";
      return result;
    }
    std::unordered_set<int> unavailable_ties(
        effective_options.unavailable_tie_switch_ids.begin(),
        effective_options.unavailable_tie_switch_ids.end());
    run_native_case(c, result, effective_options, unavailable_ties);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

}  // namespace hacdcpf::analysis
