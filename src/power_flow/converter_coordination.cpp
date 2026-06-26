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
                        bool droop,
                        const std::string& group_id = "",
                        bool is_master = false,
                        double participation_factor = 0.0,
                        double droop_gain = 0.0) {
  DCVoltageControlSource src{std::move(component_type), component_index, bus,
                             v_set_pu, has_v_set, droop};
  src.group_id = group_id;
  src.is_master = is_master;
  src.participation_factor = participation_factor;
  src.droop_gain = droop_gain;
  summary.voltage_sources.push_back(std::move(src));
  if (droop) {
    summary.droop_sources += 1;
    summary.total_droop_gain += std::abs(droop_gain);
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

bool dc_storage_can_adjust(const DCStorage& st) {
  if (!st.in_service || !st.controllable) return false;
  return st.soc_init > st.soc_min + 1e-6 && st.soc_init < st.soc_max - 1e-6;
}

bool dc_v_bus_has_storage_source(const HybridPowerSystem& sys, int bus) {
  for (const auto& st : sys.dc.storage) {
    if (st.bus == bus && storage_can_adjust(st)) return true;
  }
  for (const auto& st : sys.dc.dc_storage) {
    if (st.bus == bus && dc_storage_can_adjust(st)) return true;
  }
  return false;
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

// Forward declaration: AC connectivity islands (defined below) are needed by the
// device-rule evaluator for the AC-side support check (ACDC-GFM-06).
std::vector<int> compute_ac_islands(const HybridPowerSystem& sys,
                                    const std::unordered_map<int, int>& ac_pos_by_index,
                                    int& n_islands);

// Does the DC voltage island `dc_island` provide an energy source or a
// voltage/power-balancing mechanism (multi-converter model ACDC-GFM-05)?  True
// when a DC bus on the island is a rigid DC_V reference, another in-service VSC
// on the island forms Vdc, or the island hosts a DC energy source (storage /
// static generator / PV array).  Used to validate that an AC-side grid-forming
// converter has the DC-side support it needs to balance its active power.
bool dc_island_has_support(const HybridPowerSystem& sys,
                           const std::unordered_map<int, int>& dc_pos_by_index,
                           const std::vector<int>& component,
                           int dc_island,
                           int exclude_conv_index) {
  if (dc_island < 0) return false;
  const auto island_of = [&](int bus_index) -> int {
    const int pos = bus_position(dc_pos_by_index, bus_index);
    return (pos >= 0 && pos < static_cast<int>(component.size()))
               ? component[static_cast<size_t>(pos)]
               : -1;
  };
  for (size_t i = 0; i < sys.dc.buses.size() && i < component.size(); ++i) {
    if (component[i] == dc_island && sys.dc.buses[i].in_service &&
        sys.dc.buses[i].bus_type == DCBusType::DC_V) {
      return true;
    }
  }
  for (const auto& c : sys.vsc_converters) {
    if (!c.in_service || c.index == exclude_conv_index) continue;
    if (island_of(c.bus_dc) != dc_island) continue;
    const bool forms = ((c.control_mode == ConverterMode::VDC_Q ||
                         c.control_mode == ConverterMode::VDC_VAC ||
                         c.control_mode == ConverterMode::DC_V_DROOP_AC_V) &&
                        std::abs(c.k_vdc) > 1e-9) ||
                       c.grid_forming;
    if (forms) return true;
  }
  for (const auto& s : sys.dc.storage)
    if (s.in_service && island_of(s.bus) == dc_island) return true;
  for (const auto& s : sys.dc.dc_storage)
    if (s.in_service && island_of(s.bus) == dc_island) return true;
  for (const auto& g : sys.dc.static_generators)
    if (g.in_service && island_of(g.bus) == dc_island) return true;
  for (const auto& g : sys.dc.dc_static_generators)
    if (g.in_service && island_of(g.bus) == dc_island) return true;
  for (const auto& pv : sys.dc.pv_arrays)
    if (pv.in_service && island_of(pv.bus) == dc_island) return true;
  return false;
}

// Does the AC connectivity island containing `ac_bus_index` provide an
// active-power balancing source (multi-converter model ACDC-GFM-06): a slack
// generator, an external grid, an in-service generator, or another converter
// that injects AC active power (AC_PQ / AC_PV / DC-forming) on the island?
bool ac_island_has_active_support(const HybridPowerSystem& sys,
                                  const std::unordered_map<int, int>& ac_pos_by_index,
                                  const std::vector<int>& ac_component,
                                  int ac_bus_index,
                                  int exclude_conv_index) {
  const auto island_of = [&](int bus_index) -> int {
    const int pos = bus_position(ac_pos_by_index, bus_index);
    return (pos >= 0 && pos < static_cast<int>(ac_component.size()))
               ? ac_component[static_cast<size_t>(pos)]
               : -1;
  };
  const int target = island_of(ac_bus_index);
  // If the converter's AC bus is not part of any modeled, in-service AC island
  // (e.g. a DC-focused model that omits the AC network), the AC-side support
  // cannot be assessed — do not flag it.
  if (target < 0) return true;
  for (const auto& eg : sys.ac.external_grids)
    if (eg.in_service && island_of(eg.bus) == target) return true;
  for (const auto& g : sys.ac.generators)
    if (g.in_service && island_of(g.bus) == target) return true;
  for (const auto& sg : sys.ac.static_generators)
    if (sg.in_service && island_of(sg.bus) == target) return true;
  for (const auto& c : sys.vsc_converters) {
    if (!c.in_service || c.index == exclude_conv_index) continue;
    if (island_of(c.bus_ac) != target) continue;
    // A converter that does not itself DC-grid-form injects/withdraws AC active
    // power and can balance the AC island (AC_PQ, AC_PV, or another AC former).
    if (c.control_mode != ConverterMode::AC_GRID_FORMING) return true;
  }
  return false;
}

void evaluate_vsc_device_rules(const HybridPowerSystem& sys,
                               const std::unordered_map<int, int>& dc_pos_by_index,
                               const std::vector<int>& component,
                               ConverterCoordinationReport& report) {
  // AC connectivity islands, built once for the AC-side support check (GFM-06).
  std::unordered_map<int, int> ac_pos_by_index;
  for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
    ac_pos_by_index[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }
  int n_ac_islands = 0;
  const std::vector<int> ac_component =
      compute_ac_islands(sys, ac_pos_by_index, n_ac_islands);

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

    // A converter "forms the DC voltage" when it actively regulates Vdc
    // (VDC_Q / VDC_VAC / DC_V_DROOP_AC_V with a non-zero droop) or is declared
    // grid-forming.  Such a converter must release its AC active power to balance
    // the DC island, so an explicit hard AC-P constraint is contradictory
    // (multi-converter model §4.1.3 / §16.3).
    const bool forms_dc_voltage =
        ((conv.control_mode == ConverterMode::VDC_Q ||
          conv.control_mode == ConverterMode::VDC_VAC ||
          conv.control_mode == ConverterMode::DC_V_DROOP_AC_V) &&
         std::abs(conv.k_vdc) > 1e-9) ||
        conv.grid_forming;

    // A converter forms the AC reference when it opts into AC grid-forming via
    // the explicit flag or selects the AC_GRID_FORMING (Mode 1) control mode.
    const bool ac_is_grid_forming =
        conv.ac_grid_forming ||
        conv.control_mode == ConverterMode::AC_GRID_FORMING;

    if (forms_dc_voltage && conv.p_is_hard_constraint) {
      add_issue(report,
                CoordinationSeverity::Error,
                "ACDC-GFM-01",
                "vsc_converter",
                conv.index,
                island,
                "VSC converter " + std::to_string(conv.index) +
                    " forms the DC voltage (" + converter_mode_str(conv.control_mode) +
                    ") but also declares p_is_hard_constraint=true. A DC grid-forming "
                    "converter must release its AC active power to balance the DC island "
                    "and cannot hold AC P as a hard constraint at the same time. Set "
                    "p_is_hard_constraint=false and use p_schedule_mw / p_initial_mw for "
                    "the dispatch reference and initial guess.");
    } else if ((conv.control_mode == ConverterMode::VDC_Q ||
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
                    "control must release AC active power. Set p_schedule_mw / p_initial_mw "
                    "to declare this explicitly.");
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

    // AC-side PV control rules (multi-converter model r1 §1.5).  AC_PV holds Pac
    // and Vac and releases Qac; it never forms the AC angle reference (handled by
    // ACISLAND-REF-01).
    if (conv.control_mode == ConverterMode::AC_PV) {
      if (!(conv.v_ac_set_pu > 0.0)) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "ACDC-CTRL-03",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " is in AC_PV mode but has no valid AC voltage setpoint "
                      "(v_ac_set_pu must be > 0). AC_PV holds the AC active power and "
                      "AC voltage magnitude, so a positive voltage setpoint is required.");
      }
      if (std::abs(conv.q_set_mvar) > 1e-6) {
        add_issue(report,
                  CoordinationSeverity::Warning,
                  "ACDC-CTRL-04",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " is in AC_PV mode; q_set_mvar=" + std::to_string(conv.q_set_mvar) +
                      " is ignored because AC reactive power is released as the free "
                      "balancing injection that holds the AC voltage.");
      }
    }

    // AC-side grid-forming rules (multi-converter model r1 §2/§11.2).  These only
    // fire when the converter opts into AC grid-forming (explicit flag or the
    // AC_GRID_FORMING control mode), so ordinary converters are unaffected.
    if (ac_is_grid_forming) {
      // ACDC-GFM-03: an ordinary two-port converter cannot grid-form on both the
      // AC and DC sides at once; the dual-side case needs an explicit energy
      // buffer to decouple the two power balances.
      if (forms_dc_voltage &&
          !(conv.allow_dual_side_grid_forming && conv.has_energy_buffer)) {
        add_issue(report,
                  CoordinationSeverity::Fatal,
                  "ACDC-GFM-03",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " is declared grid-forming on both the AC side (ac_grid_forming) "
                      "and the DC side (" + converter_mode_str(conv.control_mode) +
                      "/grid_forming). An ordinary AC/DC converter cannot form both "
                      "voltage references simultaneously without an explicit energy "
                      "buffer (set allow_dual_side_grid_forming and has_energy_buffer "
                      "only for a buffered dual-side device).");
      }
      // ACDC-GFM-04: an AC grid-forming converter releases active power to hold
      // its AC reference and cannot also pin AC/DC active power as a hard
      // constraint.
      if (conv.p_is_hard_constraint) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "ACDC-GFM-04",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " is AC grid-forming but also declares p_is_hard_constraint=true. "
                      "An AC grid-forming converter must release its active power to "
                      "balance the AC island and cannot hold AC/DC active power as a "
                      "hard constraint. Use p_schedule_mw / p_initial_mw instead.");
      }
      // ACDC-GFM-05: an AC-side grid-forming converter sources its (free) AC slack
      // power from the DC side, so its DC island must provide an energy source or
      // a voltage/power-balancing mechanism — another Vdc former, a rigid DC_V
      // bus, a DC energy source, or the converter's own energy buffer.
      const bool dc_support =
          conv.has_energy_buffer ||
          dc_island_has_support(sys, dc_pos_by_index, component, island, conv.index);
      if (!dc_support) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "ACDC-GFM-05",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " is AC grid-forming but its DC island " + std::to_string(island) +
                      " has no energy source or voltage/power-balancing mechanism (no "
                      "other Vdc-forming converter, no rigid DC_V bus, no DC source, and "
                      "the converter declares no energy buffer). The AC slack power it "
                      "injects cannot be sourced. Add a DC-side reference/source or set "
                      "has_energy_buffer=true for a buffered device.");
      }
    }

    // ACDC-GFM-06: a DC-side grid-forming converter regulates Vdc by exchanging
    // active power with its AC terminal, so its AC island must provide an
    // active-power balancing source (slack, external grid, generator, or another
    // AC-injecting converter).  Skip when the converter also forms the AC side
    // (covered by the AC-island reference rules instead).
    if (forms_dc_voltage && !ac_is_grid_forming) {
      const bool ac_support = ac_island_has_active_support(
          sys, ac_pos_by_index, ac_component, conv.bus_ac, conv.index);
      if (!ac_support) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "ACDC-GFM-06",
                  "vsc_converter",
                  conv.index,
                  island,
                  "VSC converter " + std::to_string(conv.index) +
                      " forms the DC voltage (" + converter_mode_str(conv.control_mode) +
                      ") but its AC island has no active-power balancing source (no slack "
                      "bus, external grid, generator, or AC-injecting converter). The "
                      "active power it must exchange to hold Vdc cannot be sourced on the "
                      "AC side. Add an AC slack/source on the converter's AC island.");
      }
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

    // Voltage mode is now a true voltage-forming control: the solver enforces
    // vdc_out = v_ref_pu via a stiff negative-feedback law (see
    // dcdc_power_transfer), and the output island registers a hard Vdc reference.
    // It therefore no longer raises DCDC-CTRL-01; its input-side reference
    // requirement is checked alongside Droop in the §4.2 pass below.
  }
}

// AC connectivity islands: in-service AC buses joined by in-service branches,
// transformers and closed switches/breakers.  Mirrors compute_dc_voltage_islands.
std::vector<int> compute_ac_islands(const HybridPowerSystem& sys,
                                    const std::unordered_map<int, int>& pos_by_index,
                                    int& component_count) {
  const int n = static_cast<int>(sys.ac.buses.size());
  component_count = 0;
  std::vector<int> component(static_cast<size_t>(n), -1);
  if (n == 0) return component;

  std::vector<std::vector<int>> adj(static_cast<size_t>(n));
  const auto link = [&](int a_index, int b_index) {
    const int a = bus_position(pos_by_index, a_index);
    const int b = bus_position(pos_by_index, b_index);
    if (a >= 0 && b >= 0) {
      adj[static_cast<size_t>(a)].push_back(b);
      adj[static_cast<size_t>(b)].push_back(a);
    }
  };
  for (const auto& br : sys.ac.branches)
    if (br.in_service) link(br.from_bus, br.to_bus);
  for (const auto& t : sys.ac.transformers_2w)
    if (t.in_service) link(t.hv_bus, t.lv_bus);
  for (const auto& t : sys.ac.transformers_3w)
    if (t.in_service) { link(t.hv_bus, t.mv_bus); link(t.hv_bus, t.lv_bus); }
  for (const auto& sw : sys.ac.switches)
    if (sw.in_service && sw.closed) link(sw.bus_from, sw.bus_to);
  for (const auto& cb : sys.ac.circuit_breakers)
    if (cb.in_service && cb.closed) link(cb.bus_from, cb.bus_to);

  for (int start = 0; start < n; ++start) {
    if (component[static_cast<size_t>(start)] >= 0) continue;
    if (!sys.ac.buses[static_cast<size_t>(start)].in_service) {
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
        if (!sys.ac.buses[static_cast<size_t>(v)].in_service) continue;
        component[static_cast<size_t>(v)] = component_count;
        queue.push_back(v);
      }
    }
    ++component_count;
  }
  return component;
}

// AC island reference rules (multi-converter model r1 §10.1/§10.2).  An energized
// AC island needs exactly one angle reference; zero references is unsolvable
// (ACISLAND-REF-01) and more than one rigid reference over-determines the angle
// (ACISLAND-REF-02).  Both are reported as warnings so they stay diagnostic and
// never block a solve.  External grids count as a reference for REF-01 but are
// not added to the rigid count for REF-02 (they are commonly co-located with the
// slack bus, which would otherwise double-count).
void evaluate_ac_island_rules(const HybridPowerSystem& sys,
                              const std::unordered_map<int, int>& ac_pos_by_index,
                              ConverterCoordinationReport& report) {
  int n_islands = 0;
  const std::vector<int> comp = compute_ac_islands(sys, ac_pos_by_index, n_islands);
  if (n_islands == 0) return;

  struct ACIslandRef {
    int slack_count{0};
    int ac_gfm_count{0};
    bool has_external_grid{false};
    bool has_active_source{false};
    bool energized{false};
  };
  std::vector<ACIslandRef> islands(static_cast<size_t>(n_islands));

  const auto island_of = [&](int bus_index) -> int {
    const int p = bus_position(ac_pos_by_index, bus_index);
    if (p < 0 || p >= static_cast<int>(comp.size())) return -1;
    return comp[static_cast<size_t>(p)];
  };

  for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
    const int c = comp[static_cast<size_t>(i)];
    if (c < 0) continue;
    const auto& b = sys.ac.buses[static_cast<size_t>(i)];
    if (b.bus_type == BusType::SLACK) {
      islands[static_cast<size_t>(c)].slack_count += 1;
      islands[static_cast<size_t>(c)].has_active_source = true;
    }
    if (std::abs(b.pd_mw) > kTol || std::abs(b.qd_mvar) > kTol)
      islands[static_cast<size_t>(c)].energized = true;
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const int c = island_of(eg.bus);
    if (c >= 0) {
      islands[static_cast<size_t>(c)].has_external_grid = true;
      islands[static_cast<size_t>(c)].has_active_source = true;
      islands[static_cast<size_t>(c)].energized = true;
    }
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const int c = island_of(g.bus);
    if (c >= 0) {
      islands[static_cast<size_t>(c)].energized = true;
      islands[static_cast<size_t>(c)].has_active_source = true;
    }
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const int c = island_of(sg.bus);
    if (c >= 0) islands[static_cast<size_t>(c)].has_active_source = true;
  }
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    const int c = island_of(ld.bus);
    if (c >= 0) islands[static_cast<size_t>(c)].energized = true;
  }
  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) continue;
    const int c = island_of(conv.bus_ac);
    if (c < 0) continue;
    islands[static_cast<size_t>(c)].energized = true;
    // A converter that does not DC-grid-form injects AC active power and can help
    // balance the island; an AC grid-forming converter additionally forms the
    // reference.
    const bool ac_gfm = conv.ac_grid_forming ||
                        conv.control_mode == ConverterMode::AC_GRID_FORMING;
    if (ac_gfm) {
      islands[static_cast<size_t>(c)].ac_gfm_count += 1;
      islands[static_cast<size_t>(c)].has_active_source = true;
    }
  }

  for (int c = 0; c < n_islands; ++c) {
    const auto& s = islands[static_cast<size_t>(c)];
    if (!s.energized) continue;
    const int rigid = s.slack_count + s.ac_gfm_count;
    const bool has_reference = rigid > 0 || s.has_external_grid;
    if (!has_reference) {
      add_issue(report,
                CoordinationSeverity::Warning,
                "ACISLAND-REF-01",
                "ac_island",
                c,
                c,
                "AC island " + std::to_string(c) +
                    " is energized but has no AC angle reference (no slack bus, "
                    "external grid, or AC grid-forming converter). A device must "
                    "provide the AC angle reference, otherwise the island is "
                    "unsolvable.");
    } else if (rigid > 1) {
      add_issue(report,
                CoordinationSeverity::Warning,
                "ACISLAND-REF-02",
                "ac_island",
                c,
                c,
                "AC island " + std::to_string(c) + " has " + std::to_string(rigid) +
                    " rigid AC angle references (slack buses and/or AC grid-forming "
                    "converters). Without an explicit coordination (P-f droop or "
                    "virtual synchronous control) they over-determine the angle "
                    "reference.");
    }

    // ACISLAND-BALANCE-01 (multi-converter model r1 §10, step 19): an energized
    // AC island must have at least one active-power balancing source — a slack
    // bus, external grid, generator, static generator, or AC grid-forming
    // converter.  This is distinct from the angle-reference check: a generator-
    // only island has a power source (silent here) yet still lacks an angle
    // reference (ACISLAND-REF-01); a pure load island fails both.
    if (!s.has_active_source) {
      add_issue(report,
                CoordinationSeverity::Warning,
                "ACISLAND-BALANCE-01",
                "ac_island",
                c,
                c,
                "AC island " + std::to_string(c) +
                    " is energized but has no active-power balancing source (no slack "
                    "bus, external grid, generator, or AC-injecting converter). Its net "
                    "active power cannot be balanced; add a generator, external grid, or "
                    "grid-forming converter on the island.");
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
      if (dc_v_bus_has_storage_source(sys, bus.index)) {
        add_voltage_source(summary,
                           "dc_bus_storage_source",
                           bus.index,
                           bus.index,
                           bus.vm_pu,
                           true,
                           false);
      } else {
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
    }
    if (std::abs(bus.pd_mw) > kTol) {
      summary.fixed_power_devices += 1;
      summary.fixed_power_mw -= bus.pd_mw;
    }
  }

  evaluate_vsc_device_rules(sys, dc_pos_by_index, component, report);
  evaluate_dcdc_device_rules(sys, dc_pos_by_index, component, report);

  // AC-side island reference rules (multi-converter model r1 §10).  Built from a
  // fresh AC-bus-index map so AC connectivity is independent of the DC islands.
  {
    std::unordered_map<int, int> ac_pos_by_index;
    for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
      ac_pos_by_index[sys.ac.buses[static_cast<size_t>(i)].index] = i;
    }
    evaluate_ac_island_rules(sys, ac_pos_by_index, report);
  }

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
                           true,
                           conv.coordination_group_id,
                           conv.is_master,
                           conv.participation_factor,
                           conv.k_vdc);
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
                           true,
                           /*group_id=*/"",
                           /*is_master=*/false,
                           /*participation_factor=*/0.0,
                           /*droop_gain=*/dcdc.k_droop);
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
    } else if (dcdc.control_mode == DCDCControlMode::Voltage) {
      // Voltage mode forms the OUTPUT bus voltage: register a hard Vdc reference
      // on the output island (the solver holds vdc_out = v_ref_pu). The regulated
      // transfer is set by the power balance, so it is drawn from — and must be
      // supported by — the input island, accounted here as a fixed-power proxy.
      if (island_out >= 0) {
        add_voltage_source(report.dc_islands[static_cast<size_t>(island_out)],
                           "dcdc_converter",
                           dcdc.index,
                           dcdc.bus_out,
                           dcdc.v_ref_pu,
                           /*has_v_set=*/true,
                           /*droop=*/false);
      }
      if (island_in >= 0) {
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_devices += 1;
        report.dc_islands[static_cast<size_t>(island_in)].fixed_power_mw +=
            transfer.p_injection_in_mw;
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
      add_voltage_source(summary, "dc_storage", st.index, st.bus, 0.0, false, true,
                         /*group_id=*/"", /*is_master=*/false,
                         /*participation_factor=*/0.0, /*droop_gain=*/1.0);
    }
  }
  for (const auto& st : sys.dc.dc_storage) {
    if (!st.in_service) continue;
    const int island = island_for_bus(st.bus);
    if (island < 0) continue;
    auto& summary = report.dc_islands[static_cast<size_t>(island)];
    summary.fixed_power_devices += 1;
    summary.fixed_power_mw += st.p_mw;
    if (dc_storage_can_adjust(st)) {
      const double pmax = finite_limit(st.pmax_mw, st.p_rated_mw);
      const double pmin = finite_limit(st.pmin_mw, -st.p_rated_mw);
      add_flex_range(st.p_mw, pmin, pmax, summary.flexible_up_mw, summary.flexible_down_mw);
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
        add_voltage_source(summary, "energy_router_port", p.index, p.bus, p.v_set_pu, true, true,
                           /*group_id=*/"", /*is_master=*/false,
                           /*participation_factor=*/0.0, /*droop_gain=*/1.0);
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

    // ── Multi-source DC voltage coordination (§6.5–6.7) ──────────────────────
    // DCISLAND-DROOP-01: droop sources exist but their total gain is zero, so
    // they cannot regulate Vdc or share the island imbalance.
    if (summary.droop_sources > 0 && summary.total_droop_gain < 1e-9) {
      add_issue(report,
                CoordinationSeverity::Error,
                "DCISLAND-DROOP-01",
                "dc_island",
                -1,
                summary.island_index,
                "DC island " + std::to_string(summary.island_index) + " (buses " +
                    join_ints(summary.dc_buses) +
                    ") has droop DC voltage sources but their total droop gain is zero; "
                    "they cannot regulate Vdc or share the island power imbalance.");
    }

    // Group the island's voltage sources by coordination_group_id to validate
    // master-slave and participation-factor declarations.
    std::unordered_map<std::string, int> group_master_count;
    std::unordered_map<std::string, double> group_participation_sum;
    bool island_has_participation = false;
    for (const auto& s : summary.voltage_sources) {
      if (!s.group_id.empty() && s.is_master) {
        group_master_count[s.group_id] += 1;
      } else if (!s.group_id.empty()) {
        group_master_count.try_emplace(s.group_id, 0);
      }
      if (std::abs(s.participation_factor) > 1e-9) {
        island_has_participation = true;
        if (!s.group_id.empty()) {
          group_participation_sum[s.group_id] += s.participation_factor;
        }
      }
    }
    // DCISLAND-MS-01: a declared master-slave group needs exactly one master.
    for (const auto& [gid, masters] : group_master_count) {
      if (masters >= 2) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "DCISLAND-MS-01",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "Master-slave DC voltage group '" + gid + "' in island " +
                      std::to_string(summary.island_index) + " declares " +
                      std::to_string(masters) +
                      " masters; exactly one master is required (the other sources "
                      "must follow via power/droop/participation, not as masters).");
      }
    }
    // DCISLAND-PARTICIPATION-01: participation factors per group must sum to 1.
    for (const auto& [gid, sum] : group_participation_sum) {
      if (std::abs(sum - 1.0) > 1e-3) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "DCISLAND-PARTICIPATION-01",
                  "dc_island",
                  -1,
                  summary.island_index,
                  "Participation factors in DC voltage group '" + gid + "' (island " +
                      std::to_string(summary.island_index) + ") sum to " +
                      std::to_string(sum) + "; they must sum to 1.");
      }
    }
    // DCISLAND-PARTICIPATION-02: rigid Vdc control plus participation-factor
    // sharing in the same island over-constrains the active-power balance.
    if (island_has_participation && summary.hard_vdc_sources > 0) {
      add_issue(report,
                CoordinationSeverity::Error,
                "DCISLAND-PARTICIPATION-02",
                "dc_island",
                -1,
                summary.island_index,
                "DC island " + std::to_string(summary.island_index) +
                    " combines a rigid Vdc source with participation-factor sharing; "
                    "the rigid source already fixes Vdc and absorbs the imbalance, so "
                    "participation factors over-constrain the balance. Use rigid control "
                    "or participation, not both.");
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

  // ── DC/DC control-direction reference requirements (multi-converter §4.2) ──
  // A DC/DC is a power-electronic interface, not a source: it can only hold a
  // port voltage or push a scheduled power if the partner DC island has a real
  // voltage-forming source to draw from / absorb into.
  auto island_has_ref = [&](int island_idx) -> bool {
    if (island_idx < 0 || island_idx >= static_cast<int>(report.dc_islands.size())) {
      return false;
    }
    const auto& s = report.dc_islands[static_cast<size_t>(island_idx)];
    return s.hard_vdc_sources > 0 || s.droop_sources > 0 || s.promotable_vsc_sources > 0;
  };
  for (const auto& dcdc : sys.dc.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int in_isl = island_for_bus(dcdc.bus_in);
    const int out_isl = island_for_bus(dcdc.bus_out);
    const bool in_ref = island_has_ref(in_isl);
    const bool out_ref = island_has_ref(out_isl);
    if (dcdc.control_mode == DCDCControlMode::Droop ||
        dcdc.control_mode == DCDCControlMode::Voltage) {
      // Output voltage forming (Droop or Voltage mode): the regulated output
      // power must be drawn from the input side, which therefore needs its own
      // voltage-forming source.
      if (!in_ref) {
        add_issue(report,
                  CoordinationSeverity::Error,
                  "DCDC-CTRL-03",
                  "dcdc_converter",
                  dcdc.index,
                  in_isl,
                  "DC/DC converter " + std::to_string(dcdc.index) + " forms its output "
                  "voltage, but its input-side DC island has no voltage-forming "
                  "source to supply the regulated output. Add a VSC/ESS/DC source on the "
                  "input side.");
      }
    } else if (dcdc.control_mode == DCDCControlMode::Power) {
      // Power-controlled DC/DC: the scheduled transfer must be both sourced and
      // absorbed, so both DC islands need a voltage reference.
      if (!in_ref || !out_ref) {
        const std::string missing =
            (!in_ref && !out_ref) ? "input and output" : (!in_ref ? "input" : "output");
        add_issue(report,
                  CoordinationSeverity::Error,
                  "DCDC-CTRL-05",
                  "dcdc_converter",
                  dcdc.index,
                  (!in_ref ? in_isl : out_isl),
                  "Power-controlled DC/DC converter " + std::to_string(dcdc.index) +
                      " requires a voltage reference on both DC sides so the scheduled "
                      "transfer can be sourced and absorbed; missing on the " + missing +
                      " side.");
      }
    }
  }

  return report;
}

}  // namespace hacdcpf::powerflow
