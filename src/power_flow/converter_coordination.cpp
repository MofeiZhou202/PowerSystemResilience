#include "hacdcpf/power_flow/converter_coordination.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/enum_strings.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr double kTol = 1e-9;
constexpr double kLargeFlexMw = 1e9;
constexpr double kVSetTolPu = 1e-4;

std::string join_ints(const std::vector<int>& values) {
  std::ostringstream os;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) os << ",";
    os << values[i];
  }
  return os.str();
}

void add_issue(ConverterCoordinationReport& report,
               CoordinationSeverity severity,
               std::string rule_id,
               std::string component_type,
               int component_index,
               int island_index,
               std::string message) {
  report.issues.push_back(CoordinationIssue{severity,
                                            std::move(rule_id),
                                            std::move(component_type),
                                            component_index,
                                            island_index,
                                            std::move(message)});
  if (severity == CoordinationSeverity::Fatal || severity == CoordinationSeverity::Error) {
    report.feasible = false;
  }
}

int bus_position(const std::unordered_map<int, int>& pos_by_index, int bus_index) {
  const auto it = pos_by_index.find(bus_index);
  return it == pos_by_index.end() ? -1 : it->second;
}

std::vector<int> compute_dc_voltage_islands(const HybridPowerSystem& sys,
                                            const std::unordered_map<int, int>& pos_by_index,
                                            int& component_count) {
  const int n = static_cast<int>(sys.dc.buses.size());
  component_count = 0;
  std::vector<int> component(static_cast<size_t>(n), -1);
  if (n == 0) return component;

  std::vector<std::vector<int>> adj(static_cast<size_t>(n));
  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    const int f = bus_position(pos_by_index, br.from_bus);
    const int t = bus_position(pos_by_index, br.to_bus);
    if (f >= 0 && t >= 0) {
      adj[static_cast<size_t>(f)].push_back(t);
      adj[static_cast<size_t>(t)].push_back(f);
    }
  }
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    if (!cb.in_service || !cb.closed) continue;
    const int f = bus_position(pos_by_index, cb.bus_from);
    const int t = bus_position(pos_by_index, cb.bus_to);
    if (f >= 0 && t >= 0) {
      adj[static_cast<size_t>(f)].push_back(t);
      adj[static_cast<size_t>(t)].push_back(f);
    }
  }

  for (int start = 0; start < n; ++start) {
    if (component[static_cast<size_t>(start)] >= 0) continue;
    if (!sys.dc.buses[static_cast<size_t>(start)].in_service ||
        sys.dc.buses[static_cast<size_t>(start)].bus_type == DCBusType::DC_ISOLATED) {
      component[static_cast<size_t>(start)] = -2;
      continue;
    }
    std::vector<int> queue{start};
    component[static_cast<size_t>(start)] = component_count;
    size_t head = 0;
    while (head < queue.size()) {
      const int u = queue[head++];
      for (int v : adj[static_cast<size_t>(u)]) {
        if (component[static_cast<size_t>(v)] >= 0) continue;
        if (!sys.dc.buses[static_cast<size_t>(v)].in_service ||
            sys.dc.buses[static_cast<size_t>(v)].bus_type == DCBusType::DC_ISOLATED) {
          continue;
        }
        component[static_cast<size_t>(v)] = component_count;
        queue.push_back(v);
      }
    }
    ++component_count;
  }
  return component;
}

void add_voltage_source(DCIslandCoordinationSummary& summary,
                        std::string component_type,
                        int component_index,
                        int bus,
                        double v_set_pu,
                        bool has_v_set,
                        bool droop) {
  summary.voltage_sources.push_back(DCVoltageControlSource{std::move(component_type),
                                                           component_index,
                                                           bus,
                                                           v_set_pu,
                                                           has_v_set,
                                                           droop});
  if (droop) {
    summary.droop_sources += 1;
  } else {
    summary.hard_vdc_sources += 1;
  }
}

double finite_limit(double primary, double fallback) {
  if (std::isfinite(primary) && std::abs(primary) > kTol) return primary;
  return fallback;
}

void add_flex_range(double p0,
                    double pmin,
                    double pmax,
                    double& up,
                    double& down) {
  if (std::isfinite(pmax) && pmax > p0) up += pmax - p0;
  if (std::isfinite(pmin) && pmin < p0) down += p0 - pmin;
}

bool storage_can_adjust(const Storage& st) {
  if (!st.in_service || !st.controllable) return false;
  return st.soc_init > st.soc_min + 1e-6 && st.soc_init < st.soc_max - 1e-6;
}

double vsc_dc_injection_from_ac_setpoint_mw(const VSCConverter& conv) {
  const double eta = std::clamp(conv.eta, 0.01, 1.0);
  return (conv.p_set_mw >= 0.0) ? (-conv.p_set_mw / eta) : (-conv.p_set_mw * eta);
}

void add_vsc_vdc_flexibility(const VSCConverter& conv, DCIslandCoordinationSummary& summary) {
  const double eta = std::clamp(conv.eta, 0.01, 1.0);
  double p_ac_min = finite_limit(conv.pmin_mw, -conv.p_rated_mw);
  double p_ac_max = finite_limit(conv.pmax_mw, conv.p_rated_mw);
  if (!std::isfinite(p_ac_min)) p_ac_min = 0.0;
  if (!std::isfinite(p_ac_max)) p_ac_max = 0.0;

  if (std::abs(p_ac_min) <= kTol && std::abs(p_ac_max) <= kTol) {
    summary.flexible_up_mw += kLargeFlexMw;
    summary.flexible_down_mw += kLargeFlexMw;
    return;
  }

  const auto dc_from_ac = [eta](double pac) {
    return (pac >= 0.0) ? (-pac / eta) : (-pac * eta);
  };
  const double pdc_a = dc_from_ac(p_ac_min);
  const double pdc_b = dc_from_ac(p_ac_max);
  const double pdc_min = std::min(pdc_a, pdc_b);
  const double pdc_max = std::max(pdc_a, pdc_b);
  add_flex_range(0.0, pdc_min, pdc_max, summary.flexible_up_mw, summary.flexible_down_mw);
}

struct DCDCTransferMw {
  double p_injection_in_mw{0.0};
  double p_injection_out_mw{0.0};
};

DCDCTransferMw dcdc_scheduled_transfer_mw(const DCDCConverter& dcdc) {
  const double eta = std::clamp(dcdc.eta, 0.01, 1.0);
  const double p_out = dcdc.p_ref_mw;
  const double p_in = (p_out >= 0.0) ? (p_out / eta) : (p_out * eta);
  return DCDCTransferMw{.p_injection_in_mw = -p_in, .p_injection_out_mw = p_out};
}

std::string source_label(const DCVoltageControlSource& source) {
  return source.component_type + "#" + std::to_string(source.component_index) +
         "@bus" + std::to_string(source.bus);
}

void evaluate_vsc_device_rules(const HybridPowerSystem& sys,
                               const std::unordered_map<int, int>& dc_pos_by_index,
                               const std::vector<int>& component,
                               ConverterCoordinationReport& report) {
  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) continue;
    const int dc_pos = bus_position(dc_pos_by_index, conv.bus_dc);
    const int island = (dc_pos >= 0 && dc_pos < static_cast<int>(component.size()))
                           ? component[static_cast<size_t>(dc_pos)]
                           : -1;

    if (dc_pos < 0) {
      add_issue(report,
                CoordinationSeverity::Fatal,
                "ACDC-REF-01",
                "vsc_converter",
                conv.index,
                -1,
                "VSC converter " + std::to_string(conv.index) +
                    " references missing DC bus " + std::to_string(conv.bus_dc) + ".");
      continue;
    }

    if ((conv.control_mode == ConverterMode::VDC_Q ||
         conv.control_mode == ConverterMode::VDC_VAC) &&
        std::abs(conv.p_set_mw) > 1e-6) {
      add_issue(report,
                CoordinationSeverity::Warning,
                "ACDC-03",
                "vsc_converter",
                conv.index,
                island,
                "VSC converter " + std::to_string(conv.index) + " is in " +
                    converter_mode_str(conv.control_mode) +
                    " mode; p_set_mw=" + std::to_string(conv.p_set_mw) +
                    " MW is treated only as a schedule/initial value because DC voltage "
                    "control must release AC active power.");
    }

    if ((conv.control_mode == ConverterMode::VDC_Q ||
         conv.control_mode == ConverterMode::VDC_VAC) &&
        std::abs(conv.k_vdc) < 1e-9) {
      add_issue(report,
                CoordinationSeverity::Error,
                "ACDC-DROOP-01",
                "vsc_converter",
                conv.index,
                island,
                "VSC converter " + std::to_string(conv.index) +
                    " is declared as " + converter_mode_str(conv.control_mode) +
                    ", but the current PF model realizes VDC control through k_vdc "
                    "droop stiffness. k_vdc=0 releases no active-power freedom and "
                    "cannot form a DC voltage reference.");
    }
  }
}

void evaluate_dcdc_device_rules(const HybridPowerSystem& sys,
                                const std::unordered_map<int, int>& dc_pos_by_index,
                                const std::vector<int>& component,
                                ConverterCoordinationReport& report) {
  for (const auto& dcdc : sys.dc.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int in_pos = bus_position(dc_pos_by_index, dcdc.bus_in);
    const int out_pos = bus_position(dc_pos_by_index, dcdc.bus_out);
    const int island =
        (in_pos >= 0 && in_pos < static_cast<int>(component.size()))
            ? component[static_cast<size_t>(in_pos)]
            : ((out_pos >= 0 && out_pos < static_cast<int>(component.size()))
                   ? component[static_cast<size_t>(out_pos)]
                   : -1);

    if (in_pos < 0 || out_pos < 0) {
      add_issue(report,
                CoordinationSeverity::Fatal,
                "DCDC-REF-01",
                "dcdc_converter",
                dcdc.index,
                island,
                "DC/DC converter " + std::to_string(dcdc.index) +
                    " references a missing DC bus.");
      continue;
    }

    if (dcdc.control_mode == DCDCControlMode::Droop && std::abs(dcdc.k_droop) < 1e-9) {
      add_issue(report,
                CoordinationSeverity::Error,
                "DCDC-DROOP-01",
                "dcdc_converter",
                dcdc.index,
                island,
                "DC/DC converter " + std::to_string(dcdc.index) +
                    " is in Droop mode but k_droop=0, so it cannot share DC power "
                    "imbalance.");
    }

    if (dcdc.control_mode == DCDCControlMode::Voltage) {
      add_issue(report,
                CoordinationSeverity::Error,
                "DCDC-CTRL-01",
                "dcdc_converter",
                dcdc.index,
                island,
                "DC/DC converter " + std::to_string(dcdc.index) +
                    " is declared as Voltage mode, but the current PF injection model "
                    "does not enforce v_ref_pu as an output-voltage equation. It is "
                    "therefore assessed as a fixed p_ref_mw transfer, not as a DC "
                    "voltage-forming source.");
    }
  }
}

}  // namespace

const char* coordination_severity_str(CoordinationSeverity severity) noexcept {
  switch (severity) {
    case CoordinationSeverity::Info: return "info";
    case CoordinationSeverity::Warning: return "warning";
    case CoordinationSeverity::Error: return "error";
    case CoordinationSeverity::Fatal: return "fatal";
  }
  return "info";
}

bool ConverterCoordinationReport::has_blocking_issue() const noexcept {
  return std::any_of(issues.begin(), issues.end(), [](const CoordinationIssue& issue) {
    return issue.severity == CoordinationSeverity::Fatal ||
           issue.severity == CoordinationSeverity::Error;
  });
}

bool ConverterCoordinationReport::has_fatal() const noexcept {
  return std::any_of(issues.begin(), issues.end(), [](const CoordinationIssue& issue) {
    return issue.severity == CoordinationSeverity::Fatal;
  });
}

int ConverterCoordinationReport::blocking_count() const noexcept {
  return static_cast<int>(std::count_if(issues.begin(), issues.end(),
                                        [](const CoordinationIssue& issue) {
                                          return issue.severity == CoordinationSeverity::Fatal ||
                                                 issue.severity == CoordinationSeverity::Error;
                                        }));
}

int ConverterCoordinationReport::fatal_count() const noexcept {
  return static_cast<int>(std::count_if(issues.begin(), issues.end(),
                                        [](const CoordinationIssue& issue) {
                                          return issue.severity == CoordinationSeverity::Fatal;
                                        }));
}

int ConverterCoordinationReport::error_count() const noexcept {
  return static_cast<int>(std::count_if(issues.begin(), issues.end(),
                                        [](const CoordinationIssue& issue) {
                                          return issue.severity == CoordinationSeverity::Error;
                                        }));
}

int ConverterCoordinationReport::warning_count() const noexcept {
  return static_cast<int>(std::count_if(issues.begin(), issues.end(),
                                        [](const CoordinationIssue& issue) {
                                          return issue.severity == CoordinationSeverity::Warning;
                                        }));
}

ConverterCoordinationReport evaluate_converter_coordination(const HybridPowerSystem& sys,
                                                            bool enabled) {
  ConverterCoordinationReport report;
  report.enabled = enabled;
  report.feasible = true;
  if (!enabled) return report;

  std::unordered_map<int, int> dc_pos_by_index;
  for (int i = 0; i < static_cast<int>(sys.dc.buses.size()); ++i) {
    dc_pos_by_index[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }

  int component_count = 0;
  const std::vector<int> component =
      compute_dc_voltage_islands(sys, dc_pos_by_index, component_count);
  report.dc_islands.resize(static_cast<size_t>(component_count));
  for (int c = 0; c < component_count; ++c) {
    report.dc_islands[static_cast<size_t>(c)].island_index = c;
  }

  for (int i = 0; i < static_cast<int>(sys.dc.buses.size()); ++i) {
    const int c = component.empty() ? -1 : component[static_cast<size_t>(i)];
    if (c < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(c)];
    const auto& bus = sys.dc.buses[static_cast<size_t>(i)];
    summary.dc_buses.push_back(bus.index);
    if (bus.bus_type == DCBusType::DC_V) {
      summary.declared_v_buses.push_back(bus.index);
      summary.has_declared_v_bus = true;
      add_issue(report,
                CoordinationSeverity::Warning,
                "DCBUS-01",
                "dc_bus",
                bus.index,
                c,
                "DC bus " + std::to_string(bus.index) +
                    " is typed as DC_V. This is treated only as a declared bus "
                    "type, not as a physical voltage-forming device; a real "
                    "VSC/ESS/ER/DC source must provide the Vdc control freedom, "
                    "otherwise the island is reported as having no reference.");
    }
    if (std::abs(bus.pd_mw) > kTol) {
      summary.fixed_power_devices += 1;
      summary.fixed_power_mw -= bus.pd_mw;
    }
  }

  evaluate_vsc_device_rules(sys, dc_pos_by_index, component, report);
  evaluate_dcdc_device_rules(sys, dc_pos_by_index, component, report);

  auto island_for_bus = [&](int bus_index) {
    const int pos = bus_position(dc_pos_by_index, bus_index);
    if (pos < 0 || pos >= static_cast<int>(component.size())) return -1;
    return component[static_cast<size_t>(pos)];
  };

  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) continue;
    const int island = island_for_bus(conv.bus_dc);
    if (island < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    if (conv.control_mode == ConverterMode::PQ_MODE) {
      // A PQ VSC currently transfers its scheduled power, but the unified solver
      // auto-promotes the largest in-service PQ VSC of an otherwise
      // reference-less island to VDC_Q so it forms the DC voltage (see
      // plan_dc_island_references).  Record it both as the present fixed-power
      // injection and as a promotable voltage-forming candidate; the island
      // check decides which role applies and adds the regulating flexibility.
      const double pdc = vsc_dc_injection_from_ac_setpoint_mw(conv);
      summary.fixed_power_devices += 1;
      summary.fixed_power_mw += pdc;
      summary.promotable_vsc_sources += 1;
      add_vsc_vdc_flexibility(conv, summary);
    } else if (conv.control_mode == ConverterMode::VDC_Q ||
               conv.control_mode == ConverterMode::VDC_VAC) {
      if (std::abs(conv.k_vdc) > 1e-9) {
        add_voltage_source(summary,
                           "vsc_converter",
                           conv.index,
                           conv.bus_dc,
                           conv.v_dc_set_pu,
                           true,
                           true);
        add_vsc_vdc_flexibility(conv, summary);
      }
    }
  }

  for (const auto& dcdc : sys.dc.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int island_in = island_for_bus(dcdc.bus_in);
    const int island_out = island_for_bus(dcdc.bus_out);
    const DCDCTransferMw transfer = dcdc_scheduled_transfer_mw(dcdc);
    if (island_in >= 0 && island_out >= 0 && island_in == island_out) {
      add_issue(report,
                CoordinationSeverity::Warning,
                "DCDC-TOPO-01",
                "dcdc_converter",
                dcdc.index,
                island_in,
                "DC/DC converter " + std::to_string(dcdc.index) +
                    " connects two buses inside the same metallic DC voltage island. "
                    "Check whether this should be modeled as a branch/switch or as "
                    "a converter between distinct voltage levels.");
    }
    if (dcdc.control_mode == DCDCControlMode::Droop) {
      if (island_out >= 0 && std::abs(dcdc.k_droop) > 1e-9) {
        add_voltage_source(report.dc_islands[static_cast<size_t>(island_out)],
                           "dcdc_converter",
                           dcdc.index,
                           dcdc.bus_out,
                           dcdc.v_ref_pu,
                           true,
                           true);
      }
      if (island_in >= 0) {
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_devices += 1;
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_mw +=
            transfer.p_injection_in_mw;
      }
      if (island_out >= 0) {
        report.dc_islands[static_cast<size_t>(island_out)].fixed_power_devices += 1;
        report.dc_islands[static_cast<size_t>(island_out)].fixed_power_mw +=
            transfer.p_injection_out_mw;
      }
    } else {
      if (island_in >= 0) {
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_devices += 1;
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_mw +=
            transfer.p_injection_in_mw;
      }
      if (island_out >= 0) {
        report.dc_islands[static_cast<size_t>(island_out)].fixed_power_devices += 1;
        report.dc_islands[static_cast<size_t>(island_out)].fixed_power_mw +=
            transfer.p_injection_out_mw;
      }
    }
  }

  auto add_fixed_dc_injection = [&](int bus, double p_mw, const char* type, int index) {
    const int island = island_for_bus(bus);
    if (island < 0 || std::abs(p_mw) <= kTol) return;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    summary.fixed_power_devices += 1;
    summary.fixed_power_mw += p_mw;
    (void)type;
    (void)index;
  };

  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    add_fixed_dc_injection(ld.bus, -ld.p_mw * ld.scaling, "dc_load", ld.index);
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const int island = island_for_bus(sg.bus);
    if (island < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    const double p0 = sg.p_mw * sg.scaling;
    summary.fixed_power_devices += 1;
    summary.fixed_power_mw += p0;
    if (sg.controllable) {
      add_flex_range(p0, sg.pmin_mw, sg.pmax_mw, summary.flexible_up_mw, summary.flexible_down_mw);
    }
  }
  for (const auto& pv : sys.dc.dc_static_generators) {
    if (!pv.in_service) continue;
    const int island = island_for_bus(pv.bus);
    if (island < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    const double p0 = pv.p_set_mw * pv.scaling;
    summary.fixed_power_devices += 1;
    summary.fixed_power_mw += p0;
    if (pv.controllable) {
      add_flex_range(p0, pv.pmin_mw, pv.pmax_mw, summary.flexible_up_mw, summary.flexible_down_mw);
    }
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    add_fixed_dc_injection(pv.bus, pv.p_set_mw, "dc_pv_array", pv.index);
  }
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    const int island = island_for_bus(st.bus);
    if (island < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    summary.fixed_power_devices += 1;
    summary.fixed_power_mw += st.p_mw;
    if (storage_can_adjust(st)) {
      const double pmax = finite_limit(st.pmax_mw, st.p_rated_mw);
      const double pmin = finite_limit(st.pmin_mw, -st.p_rated_mw);
      add_flex_range(st.p_mw, pmin, pmax, summary.flexible_up_mw, summary.flexible_down_mw);
    }
    const std::string mode = st.control_mode;
    if (mode == "DC_V" || mode == "VDC" || mode == "Voltage") {
      add_voltage_source(summary, "dc_storage", st.index, st.bus, 0.0, false, false);
    } else if (mode == "Droop" || mode == "DC_DROOP") {
      add_voltage_source(summary, "dc_storage", st.index, st.bus, 0.0, false, true);
    }
  }
  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service || p.port_type != ERPortType::DC) continue;
      const int island = island_for_bus(p.bus);
      if (island < 0) continue;
      auto& summary = report.dc_islands[static_cast<size_t>(island)];
      if (p.control_mode == ERControlMode::VF) {
        add_voltage_source(summary, "energy_router_port", p.index, p.bus, p.v_set_pu, true, false);
      } else if (p.control_mode == ERControlMode::Droop) {
        add_voltage_source(summary, "energy_router_port", p.index, p.bus, p.v_set_pu, true, true);
      } else {
        summary.fixed_power_devices += 1;
        summary.fixed_power_mw += p.p_mw;
      }
    }
  }

  for (const auto& summary : report.dc_islands) {
    // An island is solvable when it has an *explicit* voltage-forming source
    // (rigid/droop VSC, ESS, ER, DC source) OR an in-service PQ VSC the solver
    // can auto-promote to VDC_Q.  A DC_V-typed bus is deliberately NOT counted:
    // per the multi-converter model a bus type is only a declaration, not a
    // physical voltage-forming device, so a DC_V bus with no real source/
    // converter behind it is rejected (the solver could pin it, but that is a
    // non-physical reference and is treated as a modeling error here).
    const bool explicit_reference =
        summary.hard_vdc_sources > 0 || summary.droop_sources > 0;
    const bool has_reference =
        explicit_reference || summary.promotable_vsc_sources > 0;

    if (!has_reference) {
      const std::string dcv_hint =
          summary.has_declared_v_bus
              ? " The DC_V bus type alone is only a declaration, not a physical "
                "voltage-forming device; attach a VSC/ESS/ER/DC source."
              : "";
      add_issue(report,
                CoordinationSeverity::Fatal,
                "DCISLAND-01",
                "dc_island",
                -1,
                summary.island_index,
                "DC island " + std::to_string(summary.island_index) + " (buses " +
                    join_ints(summary.dc_buses) +
                    ") has no DC voltage-forming device and no PQ VSC that can be "
                    "auto-promoted to regulate Vdc. Fixed-P converters, loads, PV, "
                    "and power-mode DC/DC converters cannot provide the voltage "
                    "reference." + dcv_hint);
    } else if (!explicit_reference && summary.promotable_vsc_sources > 0) {
      add_issue(report,
                CoordinationSeverity::Info,
                "DCISLAND-PROMOTE-01",
                "dc_island",
                -1,
                summary.island_index,
                "DC island " + std::to_string(summary.island_index) + " (buses " +
                    join_ints(summary.dc_buses) +
                    ") has no explicitly declared DC voltage source; the solver "
                    "will auto-promote an in-service PQ VSC to VDC_Q to form the "
                    "Vdc reference and balance the island.");
    }

    std::vector<DCVoltageControlSource> setpoint_sources;
    std::copy_if(summary.voltage_sources.begin(),
                 summary.voltage_sources.end(),
                 std::back_inserter(setpoint_sources),
                 [](const DCVoltageControlSource& source) {
                   return source.has_v_set && std::isfinite(source.v_set_pu);
                 });
    if (setpoint_sources.size() > 1) {
      const auto minmax = std::minmax_element(
          setpoint_sources.begin(),
          setpoint_sources.end(),
          [](const DCVoltageControlSource& a, const DCVoltageControlSource& b) {
            return a.v_set_pu < b.v_set_pu;
          });
      if (std::abs(minmax.second->v_set_pu - minmax.first->v_set_pu) > kVSetTolPu) {
        add_issue(report,
                  CoordinationSeverity::Fatal,
                  "DCISLAND-05",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "DC island " + std::to_string(summary.island_index) + " (buses " +
                      join_ints(summary.dc_buses) +
                      ") has multiple DC voltage-forming sources with conflicting setpoints: " +
                      source_label(*minmax.first) + "=" +
                      std::to_string(minmax.first->v_set_pu) + " pu, " +
                      source_label(*minmax.second) + "=" +
                      std::to_string(minmax.second->v_set_pu) + " pu.");
      }
    }

    std::vector<DCVoltageControlSource> hard_sources;
    std::copy_if(summary.voltage_sources.begin(),
                 summary.voltage_sources.end(),
                 std::back_inserter(hard_sources),
                 [](const DCVoltageControlSource& source) { return !source.droop; });

    if (hard_sources.size() > 1) {
      if (summary.droop_sources == 0) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "DCISLAND-04",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "DC island " + std::to_string(summary.island_index) + " (buses " +
                      join_ints(summary.dc_buses) + ") has " +
                      std::to_string(summary.hard_vdc_sources) +
                      " rigid DC voltage sources and no droop/participation rule; active-power "
                      "sharing is over-constrained or non-unique.");
      }
    }

    if (!has_reference && summary.fixed_power_devices > 0 &&
        summary.flexible_up_mw <= 1e-6 && summary.flexible_down_mw <= 1e-6) {
      add_issue(report,
                CoordinationSeverity::Fatal,
                "DCISLAND-02",
                "dc_island",
                -1,
                summary.island_index,
                "DC island " + std::to_string(summary.island_index) + " (buses " +
                    join_ints(summary.dc_buses) +
                    ") contains only fixed-power injections/loads and has no declared "
                    "power-balancing voltage-forming device.");
    }

    if (has_reference && summary.fixed_power_devices > 0 &&
        summary.flexible_up_mw < kLargeFlexMw / 2.0 &&
        summary.flexible_down_mw < kLargeFlexMw / 2.0) {
      const double required_absorb = summary.fixed_power_mw;
      if (required_absorb > summary.flexible_down_mw + 1e-6) {
        add_issue(report,
                  CoordinationSeverity::Warning,
                  "DCISLAND-03",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "DC island " + std::to_string(summary.island_index) +
                      " has fixed net injection " + std::to_string(summary.fixed_power_mw) +
                      " MW, but only " + std::to_string(summary.flexible_down_mw) +
                      " MW downward/absorbing flexibility is declared. Check VSC/ESS limits.");
      } else if (-required_absorb > summary.flexible_up_mw + 1e-6) {
        add_issue(report,
                  CoordinationSeverity::Warning,
                  "DCISLAND-03",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "DC island " + std::to_string(summary.island_index) +
                      " has fixed net load " + std::to_string(-summary.fixed_power_mw) +
                      " MW, but only " + std::to_string(summary.flexible_up_mw) +
                      " MW upward/supplying flexibility is declared. Check VSC/ESS limits.");
      }
    }
  }

  return report;
}

}  // namespace hacdcpf::powerflow
