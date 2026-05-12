#include "hacdcpf/io/excel_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hacdcpf/core/string_utils.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf.hpp"

// OpenXLSX master branch has defaulted copy/move on non-copyable members;
// suppress the resulting -Wdefaulted-function-deleted diagnostic.
#ifdef __clang__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdefaulted-function-deleted"
#endif
#include <OpenXLSX.hpp>
#ifdef __clang__
#  pragma clang diagnostic pop
#endif

namespace hacdcpf::io {
namespace {

using OpenXLSX::XLCellReference;
using OpenXLSX::XLDocument;
using OpenXLSX::XLWorksheet;
using OpenXLSX::XLWorkbook;

std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return s;
}

std::string bool_str(bool v) { return v ? "TRUE" : "FALSE"; }

bool bool_from_str(const std::string& s, bool default_value = false) {
  const std::string u = to_upper(trim(s));
  if (u == "TRUE" || u == "T" || u == "YES" || u == "Y" || u == "1") {
    return true;
  }
  if (u == "FALSE" || u == "F" || u == "NO" || u == "N" || u == "0") {
    return false;
  }
  return default_value;
}

int int_from_str(const std::string& s, int default_value = 0) {
  try {
    if (trim(s).empty()) return default_value;
    return std::stoi(s);
  } catch (const std::exception&) {
    return default_value;
  }
}

double dbl_from_str(const std::string& s, double default_value = 0.0) {
  try {
    if (trim(s).empty()) return default_value;
    return std::stod(s);
  } catch (const std::exception&) {
    return default_value;
  }
}

std::string cell_to_string(const XLWorksheet& ws, uint32_t row, uint16_t col) {
  auto cell = ws.findCell(row, col);
  if (cell.empty()) return "";
  return trim(cell.getString());
}

std::string cell_by_name(const XLWorksheet& ws,
                         uint32_t row,
                         const std::unordered_map<std::string, uint16_t>& colmap,
                         const std::string& name) {
  auto it = colmap.find(name);
  if (it == colmap.end()) return "";
  return cell_to_string(ws, row, it->second);
}

std::unordered_map<std::string, uint16_t> make_colmap(const XLWorksheet& ws) {
  std::unordered_map<std::string, uint16_t> map;
  const uint16_t cols = ws.columnCount();
  for (uint16_t c = 1; c <= cols; ++c) {
    const std::string key = cell_to_string(ws, 1, c);
    if (!key.empty()) map.emplace(key, c);
  }
  return map;
}

void write_headers(XLWorksheet ws, const std::vector<std::string>& headers) {
  for (uint16_t c = 0; c < headers.size(); ++c) {
    ws.cell(1, static_cast<uint16_t>(c + 1)).value() = headers[c];
  }
}

uint16_t ensure_header_column(XLWorksheet ws, const std::string& header) {
  const auto colmap = make_colmap(ws);
  auto it = colmap.find(header);
  if (it != colmap.end()) return it->second;
  const uint16_t col = static_cast<uint16_t>(ws.columnCount() + 1);
  ws.cell(1, col).value() = header;
  return col;
}

XLWorksheet ensure_sheet(XLWorkbook& wb, const std::string& name) {
  if (!wb.worksheetExists(name)) {
    wb.addWorksheet(name);
  }
  return wb.worksheet(name);
}

std::vector<ACBus> buses_with_aggregated_ac_loads(const HybridPowerSystem& sys) {
  std::vector<ACBus> out = sys.ac.buses;
  std::unordered_map<int, std::pair<double, double>> by_bus;

  // First-class AC load table.
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    auto& acc = by_bus[ld.bus];
    acc.first += ld.p_mw;
    acc.second += ld.q_mvar;
  }

  // Charging stations are modeled as constant-power demand.
  for (const auto& cs : sys.ac.charging_stations) {
    if (!cs.in_service) continue;
    auto& acc = by_bus[cs.bus];
    acc.first += cs.p_total_kw / 1000.0;
    acc.second += cs.q_total_kvar / 1000.0;
  }

  // Charging mode for storage behaves as load (p_mw < 0, q_mvar < 0).
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    auto& acc = by_bus[st.bus];
    if (st.p_mw < 0.0) acc.first += -st.p_mw;
    if (st.q_mvar < 0.0) acc.second += -st.q_mvar;
  }

  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service) continue;
    if (ms.status == MobileStorageStatus::InTransit) continue;
    auto& acc = by_bus[ms.bus];
    if (ms.p_mw < 0.0) acc.first += -ms.p_mw;
    if (ms.q_mvar < 0.0) acc.second += -ms.q_mvar;
  }

  // Aggregation-level flexible demand terms.
  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service) continue;
    auto& acc = by_bus[vpp.pcc_bus];
    if (vpp.p_load_controllable_mw > 0.0) acc.first += vpp.p_load_controllable_mw;
    if (vpp.p_output_mw < 0.0) acc.first += -vpp.p_output_mw;
    if (vpp.q_output_mvar < 0.0) acc.second += -vpp.q_output_mvar;
  }

  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service) continue;
    if (mg.operating_mode != MicrogridMode::GridConnected) continue;
    if (mg.p_exchange_mw < 0.0) {
      by_bus[mg.pcc_bus].first += -mg.p_exchange_mw;
    }
  }

  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      if (p.port_type != ERPortType::AC) continue;
      auto& acc = by_bus[p.bus];
      if (p.p_mw < 0.0) acc.first += -p.p_mw;
      if (p.q_mvar < 0.0) acc.second += -p.q_mvar;
    }
  }

  if (!by_bus.empty()) {
    for (auto& b : out) {
      b.pd_mw = 0.0;
      b.qd_mvar = 0.0;
    }
  }

  for (auto& b : out) {
    auto it = by_bus.find(b.index);
    if (it != by_bus.end()) {
      b.pd_mw = it->second.first;
      b.qd_mvar = it->second.second;
    }
  }
  return out;
}

std::vector<Load> make_aggregated_ac_load_table(const HybridPowerSystem& sys) {
  struct Acc {
    double p_mw{0.0};
    double q_mvar{0.0};
  };
  std::unordered_map<std::string, Acc> by_bus_source;

  auto key_for = [](int bus, const std::string& source) {
    return std::to_string(bus) + "|" + source;
  };

  auto add = [&](int bus, const std::string& source, double p_mw, double q_mvar) {
    if (bus <= 0) return;
    if (std::abs(p_mw) <= 1e-12 && std::abs(q_mvar) <= 1e-12) return;
    auto& acc = by_bus_source[key_for(bus, source)];
    acc.p_mw += p_mw;
    acc.q_mvar += q_mvar;
  };

  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    add(ld.bus, "Load", ld.p_mw, ld.q_mvar);
  }

  for (const auto& cs : sys.ac.charging_stations) {
    if (!cs.in_service) continue;
    add(cs.bus, "ChargingStation", cs.p_total_kw / 1000.0, cs.q_total_kvar / 1000.0);
  }

  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    add(st.bus, "StorageCharge", (st.p_mw < 0.0) ? -st.p_mw : 0.0, (st.q_mvar < 0.0) ? -st.q_mvar : 0.0);
  }

  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service) continue;
    if (ms.status == MobileStorageStatus::InTransit) continue;
    add(ms.bus, "MobileStorageCharge", (ms.p_mw < 0.0) ? -ms.p_mw : 0.0, (ms.q_mvar < 0.0) ? -ms.q_mvar : 0.0);
  }

  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service) continue;
    add(vpp.pcc_bus, "VPPControllableLoad", std::max(0.0, vpp.p_load_controllable_mw), 0.0);
    add(vpp.pcc_bus, "VPPNetImport", (vpp.p_output_mw < 0.0) ? -vpp.p_output_mw : 0.0,
        (vpp.q_output_mvar < 0.0) ? -vpp.q_output_mvar : 0.0);
  }

  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service) continue;
    if (mg.operating_mode != MicrogridMode::GridConnected) continue;
    add(mg.pcc_bus, "MicrogridImport", (mg.p_exchange_mw < 0.0) ? -mg.p_exchange_mw : 0.0, 0.0);
  }

  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      if (p.port_type != ERPortType::AC) continue;
      add(p.bus, "EnergyRouterPort", (p.p_mw < 0.0) ? -p.p_mw : 0.0, (p.q_mvar < 0.0) ? -p.q_mvar : 0.0);
    }
  }

  std::vector<std::pair<std::string, Acc>> rows;
  rows.reserve(by_bus_source.size());
  for (const auto& kv : by_bus_source) rows.push_back(kv);

  std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
    const auto pa = a.first.find('|');
    const auto pb = b.first.find('|');
    const int ba = (pa == std::string::npos) ? 0 : int_from_str(a.first.substr(0, pa), 0);
    const int bb = (pb == std::string::npos) ? 0 : int_from_str(b.first.substr(0, pb), 0);
    if (ba != bb) return ba < bb;
    const std::string sa = (pa == std::string::npos) ? a.first : a.first.substr(pa + 1);
    const std::string sb = (pb == std::string::npos) ? b.first : b.first.substr(pb + 1);
    return sa < sb;
  });

  std::vector<Load> out;
  out.reserve(rows.size());
  int next_index = 1;
  for (const auto& kv : rows) {
    const auto sep = kv.first.find('|');
    const int bus = (sep == std::string::npos) ? 0 : int_from_str(kv.first.substr(0, sep), 0);
    const std::string source = (sep == std::string::npos) ? "Aggregated" : kv.first.substr(sep + 1);
    Load ld;
    ld.index = next_index++;
    ld.name = "Agg_" + source + "_Bus" + std::to_string(bus);
    ld.bus = bus;
    ld.p_mw = kv.second.p_mw;
    ld.q_mvar = kv.second.q_mvar;
    ld.model = LoadModel::ConstantPower;
    ld.p_percent_p = 100.0;
    ld.p_percent_q = 100.0;
    ld.controllable = false;
    ld.priority = LoadPriority::Medium;
    ld.in_service = true;
    out.push_back(std::move(ld));
  }
  return out;
}

std::vector<DCBus> buses_with_aggregated_dc_loads(const HybridPowerSystem& sys) {
  std::vector<DCBus> out = sys.dc.buses;
  std::unordered_map<int, double> by_bus;

  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    by_bus[ld.bus] += ld.p_mw;
  }

  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    if (st.p_mw < 0.0) by_bus[st.bus] += -st.p_mw;
  }

  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      if (p.port_type != ERPortType::DC) continue;
      if (p.p_mw < 0.0) by_bus[p.bus] += -p.p_mw;
    }
  }

  if (!by_bus.empty()) {
    for (auto& b : out) {
      b.pd_mw = 0.0;
    }
  }

  for (auto& b : out) {
    auto it = by_bus.find(b.index);
    if (it != by_bus.end()) {
      b.pd_mw = it->second;
    }
  }
  return out;
}

std::vector<DCLoad> make_aggregated_dc_load_table(const HybridPowerSystem& sys) {
  std::unordered_map<std::string, double> by_bus_source;

  auto key_for = [](int bus, const std::string& source) {
    return std::to_string(bus) + "|" + source;
  };

  auto add = [&](int bus, const std::string& source, double p_mw) {
    if (bus <= 0) return;
    if (std::abs(p_mw) <= 1e-12) return;
    by_bus_source[key_for(bus, source)] += p_mw;
  };

  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    add(ld.bus, "DCLoad", ld.p_mw);
  }

  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    add(st.bus, "DCStorageCharge", (st.p_mw < 0.0) ? -st.p_mw : 0.0);
  }

  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      if (p.port_type != ERPortType::DC) continue;
      add(p.bus, "EnergyRouterPort", (p.p_mw < 0.0) ? -p.p_mw : 0.0);
    }
  }

  std::vector<std::pair<std::string, double>> rows;
  rows.reserve(by_bus_source.size());
  for (const auto& kv : by_bus_source) rows.push_back(kv);

  std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
    const auto pa = a.first.find('|');
    const auto pb = b.first.find('|');
    const int ba = (pa == std::string::npos) ? 0 : int_from_str(a.first.substr(0, pa), 0);
    const int bb = (pb == std::string::npos) ? 0 : int_from_str(b.first.substr(0, pb), 0);
    if (ba != bb) return ba < bb;
    const std::string sa = (pa == std::string::npos) ? a.first : a.first.substr(pa + 1);
    const std::string sb = (pb == std::string::npos) ? b.first : b.first.substr(pb + 1);
    return sa < sb;
  });

  std::vector<DCLoad> out;
  out.reserve(rows.size());
  int next_index = 1;
  for (const auto& kv : rows) {
    const auto sep = kv.first.find('|');
    const int bus = (sep == std::string::npos) ? 0 : int_from_str(kv.first.substr(0, sep), 0);
    const std::string source = (sep == std::string::npos) ? "Aggregated" : kv.first.substr(sep + 1);
    DCLoad ld;
    ld.index = next_index++;
    ld.name = "Agg_" + source + "_Bus" + std::to_string(bus);
    ld.bus = bus;
    ld.p_mw = kv.second;
    ld.controllable = false;
    ld.in_service = true;
    out.push_back(std::move(ld));
  }
  return out;
}

void sync_bus_demands_from_load_tables(HybridPowerSystem& sys) {
  const HybridPowerSystem view = sys;
  const auto ac_buses = buses_with_aggregated_ac_loads(view);
  const auto dc_buses = buses_with_aggregated_dc_loads(view);

  if (ac_buses.size() == sys.ac.buses.size()) {
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      sys.ac.buses[i].pd_mw = ac_buses[i].pd_mw;
      sys.ac.buses[i].qd_mvar = ac_buses[i].qd_mvar;
    }
  }

  if (dc_buses.size() == sys.dc.buses.size()) {
    for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
      sys.dc.buses[i].pd_mw = dc_buses[i].pd_mw;
    }
  }
}

ThreePhaseACSystem& ensure_three_phase_system(HybridPowerSystem& sys) {
  if (!sys.three_phase_ac.has_value()) {
    sys.three_phase_ac.emplace();
  }
  return *sys.three_phase_ac;
}

// enum helpers (consistent with json_io.cpp patterns)
std::string bus_type_str(BusType t) {
  switch (t) {
    case BusType::PQ: return "PQ";
    case BusType::PV: return "PV";
    case BusType::SLACK: return "SLACK";
    case BusType::ISOLATED: return "ISOLATED";
  }
  return "PQ";
}
BusType bus_type_from_str(const std::string& s) {
  if (s == "PV") return BusType::PV;
  if (s == "SLACK") return BusType::SLACK;
  if (s == "ISOLATED") return BusType::ISOLATED;
  return BusType::PQ;
}

std::string dc_bus_type_str(DCBusType t) {
  return (t == DCBusType::DC_V) ? "DC_V" : "DC_P";
}
DCBusType dc_bus_type_from_str(const std::string& s) {
  return (s == "DC_V") ? DCBusType::DC_V : DCBusType::DC_P;
}

std::string fuel_type_str(FuelType f) {
  switch (f) {
    case FuelType::Coal: return "Coal";
    case FuelType::Gas: return "Gas";
    case FuelType::Oil: return "Oil";
    case FuelType::Nuclear: return "Nuclear";
    case FuelType::Hydro: return "Hydro";
    case FuelType::Wind: return "Wind";
    case FuelType::Solar: return "Solar";
    case FuelType::Biomass: return "Biomass";
    case FuelType::Geothermal: return "Geothermal";
    case FuelType::Storage: return "Storage";
    default: return "Unknown";
  }
}
FuelType fuel_type_from_str(const std::string& s) {
  if (s == "Coal") return FuelType::Coal;
  if (s == "Gas") return FuelType::Gas;
  if (s == "Oil") return FuelType::Oil;
  if (s == "Nuclear") return FuelType::Nuclear;
  if (s == "Hydro") return FuelType::Hydro;
  if (s == "Wind") return FuelType::Wind;
  if (s == "Solar") return FuelType::Solar;
  if (s == "Biomass") return FuelType::Biomass;
  if (s == "Geothermal") return FuelType::Geothermal;
  if (s == "Storage") return FuelType::Storage;
  return FuelType::Unknown;
}

std::string load_model_str(LoadModel m) {
  switch (m) {
    case LoadModel::ZIP: return "ZIP";
    case LoadModel::Exponential: return "Exponential";
    default: return "ConstantPower";
  }
}
LoadModel load_model_from_str(const std::string& s) {
  if (s == "ZIP") return LoadModel::ZIP;
  if (s == "Exponential") return LoadModel::Exponential;
  return LoadModel::ConstantPower;
}

std::string load_priority_str(LoadPriority p) {
  switch (p) {
    case LoadPriority::Low: return "Low";
    case LoadPriority::High: return "High";
    case LoadPriority::Critical: return "Critical";
    default: return "Medium";
  }
}
LoadPriority load_priority_from_str(const std::string& s) {
  if (s == "Low") return LoadPriority::Low;
  if (s == "High") return LoadPriority::High;
  if (s == "Critical") return LoadPriority::Critical;
  return LoadPriority::Medium;
}

std::string renewable_type_str(RenewableType t) {
  switch (t) {
    case RenewableType::SolarPV: return "SolarPV";
    case RenewableType::SolarCSP: return "SolarCSP";
    default: return "Wind";
  }
}
RenewableType renewable_type_from_str(const std::string& s) {
  if (s == "SolarPV") return RenewableType::SolarPV;
  if (s == "SolarCSP") return RenewableType::SolarCSP;
  return RenewableType::Wind;
}

std::string sgen_type_str(SgenType t) {
  switch (t) {
    case SgenType::PV: return "PV";
    case SgenType::Wind: return "Wind";
    case SgenType::CHP: return "CHP";
    case SgenType::Diesel: return "Diesel";
    case SgenType::FuelCell: return "FuelCell";
    default: return "Other";
  }
}
SgenType sgen_type_from_str(const std::string& s) {
  if (s == "PV") return SgenType::PV;
  if (s == "Wind") return SgenType::Wind;
  if (s == "CHP") return SgenType::CHP;
  if (s == "Diesel") return SgenType::Diesel;
  if (s == "FuelCell") return SgenType::FuelCell;
  return SgenType::Other;
}

std::string pv_control_mode_str(PVControlMode m) {
  switch (m) {
    case PVControlMode::PQ: return "PQ";
    case PVControlMode::VQ: return "VQ";
    case PVControlMode::Curtailed: return "Curtailed";
    default: return "MPPT";
  }
}
PVControlMode pv_control_mode_from_str(const std::string& s) {
  if (s == "PQ") return PVControlMode::PQ;
  if (s == "VQ") return PVControlMode::VQ;
  if (s == "Curtailed") return PVControlMode::Curtailed;
  return PVControlMode::MPPT;
}

std::string converter_mode_str(ConverterMode m) {
  switch (m) {
    case ConverterMode::VDC_Q: return "VDC_Q";
    case ConverterMode::VDC_VAC: return "VDC_VAC";
    default: return "PQ";
  }
}
ConverterMode converter_mode_from_str(const std::string& s) {
  if (s == "VDC_Q") return ConverterMode::VDC_Q;
  if (s == "VDC_VAC") return ConverterMode::VDC_VAC;
  return ConverterMode::PQ_MODE;
}

std::string dcdc_control_mode_str(DCDCControlMode m) {
  switch (m) {
    case DCDCControlMode::Power: return "Power";
    case DCDCControlMode::Droop: return "Droop";
    default: return "Voltage";
  }
}

std::string er_port_type_str(ERPortType t) {
  return (t == ERPortType::AC) ? "AC" : "DC";
}
ERPortType er_port_type_from_str(const std::string& s) {
  return (s == "DC") ? ERPortType::DC : ERPortType::AC;
}

std::string er_control_mode_str(ERControlMode m) {
  switch (m) {
    case ERControlMode::PQ: return "PQ";
    case ERControlMode::VF: return "VF";
    default: return "Droop";
  }
}
ERControlMode er_control_mode_from_str(const std::string& s) {
  if (s == "VF") return ERControlMode::VF;
  if (s == "Droop") return ERControlMode::Droop;
  return ERControlMode::PQ;
}

DCDCControlMode dcdc_control_mode_from_str(const std::string& s) {
  if (s == "Power") return DCDCControlMode::Power;
  if (s == "Droop") return DCDCControlMode::Droop;
  return DCDCControlMode::Voltage;
}

std::string switch_type_str(SwitchType t) {
  switch (t) {
    case SwitchType::Disconnector: return "Disconnector";
    case SwitchType::LoadBreakSwitch: return "LoadBreakSwitch";
    case SwitchType::Fuse: return "Fuse";
    case SwitchType::Recloser: return "Recloser";
    case SwitchType::Sectionalizer: return "Sectionalizer";
    default: return "CircuitBreaker";
  }
}
SwitchType switch_type_from_str(const std::string& s) {
  if (s == "Disconnector") return SwitchType::Disconnector;
  if (s == "LoadBreakSwitch") return SwitchType::LoadBreakSwitch;
  if (s == "Fuse") return SwitchType::Fuse;
  if (s == "Recloser") return SwitchType::Recloser;
  if (s == "Sectionalizer") return SwitchType::Sectionalizer;
  return SwitchType::CircuitBreaker;
}

std::string breaker_type_str(BreakerType t) {
  switch (t) {
    case BreakerType::LS: return "LS";
    case BreakerType::DS: return "DS";
    default: return "CB";
  }
}
BreakerType breaker_type_from_str(const std::string& s) {
  if (s == "LS") return BreakerType::LS;
  if (s == "DS") return BreakerType::DS;
  return BreakerType::CB;
}

std::string charger_type_str(ChargerType t) {
  switch (t) {
    case ChargerType::AC_L1: return "AC_L1";
    case ChargerType::DC_Fast: return "DC_Fast";
    default: return "AC_L2";
  }
}
ChargerType charger_type_from_str(const std::string& s) {
  if (s == "AC_L1") return ChargerType::AC_L1;
  if (s == "DC_Fast") return ChargerType::DC_Fast;
  return ChargerType::AC_L2;
}

std::string microgrid_mode_str(MicrogridMode m) {
  switch (m) {
    case MicrogridMode::Islanded: return "Islanded";
    case MicrogridMode::Transition: return "Transition";
    default: return "GridConnected";
  }
}
MicrogridMode microgrid_mode_from_str(const std::string& s) {
  if (s == "Islanded") return MicrogridMode::Islanded;
  if (s == "Transition") return MicrogridMode::Transition;
  return MicrogridMode::GridConnected;
}

std::string mobile_storage_status_str(MobileStorageStatus s) {
  switch (s) {
    case MobileStorageStatus::InTransit: return "InTransit";
    case MobileStorageStatus::Deployed: return "Deployed";
    default: return "Stationary";
  }
}
MobileStorageStatus mobile_storage_status_from_str(const std::string& s) {
  if (s == "InTransit") return MobileStorageStatus::InTransit;
  if (s == "Deployed") return MobileStorageStatus::Deployed;
  return MobileStorageStatus::Stationary;
}

void write_info(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Key", "Value"});
  ws.cell(2, 1).value() = "name";
  ws.cell(2, 2).value() = sys.name;
  ws.cell(3, 1).value() = "base_mva";
  ws.cell(3, 2).value() = sys.base_mva;
  ws.cell(4, 1).value() = "ac_freq_hz";
  ws.cell(4, 2).value() = sys.ac.freq_hz;
  if (sys.three_phase_ac.has_value()) {
    ws.cell(5, 1).value() = "three_phase_name";
    ws.cell(5, 2).value() = sys.three_phase_ac->name;
    ws.cell(6, 1).value() = "three_phase_base_mva";
    ws.cell(6, 2).value() = sys.three_phase_ac->base_mva;
    ws.cell(7, 1).value() = "three_phase_base_freq_hz";
    ws.cell(7, 2).value() = sys.three_phase_ac->base_freq_hz;
  }
}

void load_info(const XLWorksheet& ws, HybridPowerSystem& sys) {
  const uint32_t rows = ws.rowCount();
  for (uint32_t r = 2; r <= rows; ++r) {
    const std::string key = cell_to_string(ws, r, 1);
    const std::string value = cell_to_string(ws, r, 2);
    if (key == "name") {
      sys.name = value;
    } else if (key == "base_mva") {
      sys.base_mva = dbl_from_str(value, sys.base_mva);
      sys.ac.base_mva = sys.base_mva;
      sys.dc.base_mva = sys.base_mva;
    } else if (key == "ac_freq_hz") {
      sys.ac.freq_hz = dbl_from_str(value, sys.ac.freq_hz);
    } else if (key == "three_phase_name") {
      ensure_three_phase_system(sys).name = value;
    } else if (key == "three_phase_base_mva") {
      ensure_three_phase_system(sys).base_mva =
          dbl_from_str(value, ensure_three_phase_system(sys).base_mva);
    } else if (key == "three_phase_base_freq_hz") {
      ensure_three_phase_system(sys).base_freq_hz =
          dbl_from_str(value, ensure_three_phase_system(sys).base_freq_hz);
    }
  }
}

void write_ac_buses(XLWorksheet ws, const std::vector<ACBus>& buses) {
  write_headers(ws, {"index", "name", "bus_type", "pd_mw", "qd_mvar", "vm_pu", "va_deg", "base_kv", "vmin_pu",
                     "vmax_pu", "gs_mw", "bs_mvar", "area", "zone", "in_service", "n_customers", "importance", "latitude", "longitude"});
  for (size_t i = 0; i < buses.size(); ++i) {
    const auto& b = buses[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = b.index;
    ws.cell(r, 2).value() = b.name;
    ws.cell(r, 3).value() = bus_type_str(b.bus_type);
    ws.cell(r, 4).value() = b.pd_mw;
    ws.cell(r, 5).value() = b.qd_mvar;
    ws.cell(r, 6).value() = b.vm_pu;
    ws.cell(r, 7).value() = b.va_deg;
    ws.cell(r, 8).value() = b.base_kv;
    ws.cell(r, 9).value() = b.vmin_pu;
    ws.cell(r, 10).value() = b.vmax_pu;
    ws.cell(r, 11).value() = b.gs_mw;
    ws.cell(r, 12).value() = b.bs_mvar;
    ws.cell(r, 13).value() = b.area;
    ws.cell(r, 14).value() = b.zone;
    ws.cell(r, 15).value() = bool_str(b.in_service);
    ws.cell(r, 16).value() = b.n_customers;
    ws.cell(r, 17).value() = b.importance;
    ws.cell(r, 18).value() = b.latitude;
    ws.cell(r, 19).value() = b.longitude;
  }
}

std::vector<ACBus> read_ac_buses(const XLWorksheet& ws) {
  std::vector<ACBus> out;
  const auto col = make_colmap(ws);
  const uint32_t rows = ws.rowCount();
  for (uint32_t r = 2; r <= rows; ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ACBus b;
    b.index = int_from_str(idx, b.index);
    b.name = cell_by_name(ws, r, col, "name");
    b.bus_type = bus_type_from_str(cell_by_name(ws, r, col, "bus_type"));
    b.pd_mw = dbl_from_str(cell_by_name(ws, r, col, "pd_mw"), b.pd_mw);
    b.qd_mvar = dbl_from_str(cell_by_name(ws, r, col, "qd_mvar"), b.qd_mvar);
    b.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_pu"), b.vm_pu);
    b.va_deg = dbl_from_str(cell_by_name(ws, r, col, "va_deg"), b.va_deg);
    b.base_kv = dbl_from_str(cell_by_name(ws, r, col, "base_kv"), b.base_kv);
    b.vmin_pu = dbl_from_str(cell_by_name(ws, r, col, "vmin_pu"), b.vmin_pu);
    b.vmax_pu = dbl_from_str(cell_by_name(ws, r, col, "vmax_pu"), b.vmax_pu);
    b.gs_mw = dbl_from_str(cell_by_name(ws, r, col, "gs_mw"), b.gs_mw);
    b.bs_mvar = dbl_from_str(cell_by_name(ws, r, col, "bs_mvar"), b.bs_mvar);
    b.area = int_from_str(cell_by_name(ws, r, col, "area"), b.area);
    b.zone = int_from_str(cell_by_name(ws, r, col, "zone"), b.zone);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), b.in_service);
    b.n_customers = int_from_str(cell_by_name(ws, r, col, "n_customers"), b.n_customers);
    b.importance = dbl_from_str(cell_by_name(ws, r, col, "importance"), b.importance);
    b.latitude = dbl_from_str(cell_by_name(ws, r, col, "latitude"), b.latitude);
    b.longitude = dbl_from_str(cell_by_name(ws, r, col, "longitude"), b.longitude);
    out.push_back(std::move(b));
  }
  return out;
}

void write_ac_branches(XLWorksheet ws, const std::vector<ACBranch>& branches) {
  write_headers(ws, {"index", "name", "from_bus", "to_bus", "r_pu", "x_pu", "b_pu", "tap", "shift_deg", "rate_a_mva",
                     "rate_b_mva", "rate_c_mva", "length_km", "in_service", "failure_rate", "mttr_hr"});
  for (size_t i = 0; i < branches.size(); ++i) {
    const auto& b = branches[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = b.index;
    ws.cell(r, 2).value() = b.name;
    ws.cell(r, 3).value() = b.from_bus;
    ws.cell(r, 4).value() = b.to_bus;
    ws.cell(r, 5).value() = b.r_pu;
    ws.cell(r, 6).value() = b.x_pu;
    ws.cell(r, 7).value() = b.b_pu;
    ws.cell(r, 8).value() = b.tap;
    ws.cell(r, 9).value() = b.shift_deg;
    ws.cell(r, 10).value() = b.rate_a_mva;
    ws.cell(r, 11).value() = b.rate_b_mva;
    ws.cell(r, 12).value() = b.rate_c_mva;
    ws.cell(r, 13).value() = b.length_km;
    ws.cell(r, 14).value() = bool_str(b.in_service);
    ws.cell(r, 15).value() = b.failure_rate;
    ws.cell(r, 16).value() = b.mttr_hr;
  }
}

std::vector<ACBranch> read_ac_branches(const XLWorksheet& ws) {
  std::vector<ACBranch> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ACBranch b;
    b.index = int_from_str(idx, b.index);
    b.name = cell_by_name(ws, r, col, "name");
    b.from_bus = int_from_str(cell_by_name(ws, r, col, "from_bus"), b.from_bus);
    b.to_bus = int_from_str(cell_by_name(ws, r, col, "to_bus"), b.to_bus);
    b.r_pu = dbl_from_str(cell_by_name(ws, r, col, "r_pu"), b.r_pu);
    b.x_pu = dbl_from_str(cell_by_name(ws, r, col, "x_pu"), b.x_pu);
    b.b_pu = dbl_from_str(cell_by_name(ws, r, col, "b_pu"), b.b_pu);
    b.tap = dbl_from_str(cell_by_name(ws, r, col, "tap"), b.tap);
    b.shift_deg = dbl_from_str(cell_by_name(ws, r, col, "shift_deg"), b.shift_deg);
    b.rate_a_mva = dbl_from_str(cell_by_name(ws, r, col, "rate_a_mva"), b.rate_a_mva);
    b.rate_b_mva = dbl_from_str(cell_by_name(ws, r, col, "rate_b_mva"), b.rate_b_mva);
    b.rate_c_mva = dbl_from_str(cell_by_name(ws, r, col, "rate_c_mva"), b.rate_c_mva);
    b.length_km = dbl_from_str(cell_by_name(ws, r, col, "length_km"), b.length_km);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), b.in_service);
    b.failure_rate = dbl_from_str(cell_by_name(ws, r, col, "failure_rate"), b.failure_rate);
    b.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), b.mttr_hr);
    out.push_back(std::move(b));
  }
  return out;
}

void write_generators(XLWorksheet ws, const std::vector<Generator>& gens) {
  write_headers(ws, {"index", "name", "bus", "pg_mw", "qg_mvar", "vg_pu", "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar",
                     "is_slack", "fuel_type", "mbase_mva", "cost_c2", "cost_c1", "cost_c0", "in_service", "inertia_h", "droop_r",
                     "ramp_up_mw_min", "ramp_dn_mw_min"});
  for (size_t i = 0; i < gens.size(); ++i) {
    const auto& g = gens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = g.pg_mw;
    ws.cell(r, 5).value() = g.qg_mvar;
    ws.cell(r, 6).value() = g.vg_pu;
    ws.cell(r, 7).value() = g.pmax_mw;
    ws.cell(r, 8).value() = g.pmin_mw;
    ws.cell(r, 9).value() = g.qmax_mvar;
    ws.cell(r, 10).value() = g.qmin_mvar;
    ws.cell(r, 11).value() = bool_str(g.is_slack);
    ws.cell(r, 12).value() = fuel_type_str(g.fuel_type);
    ws.cell(r, 13).value() = g.mbase_mva;
    ws.cell(r, 14).value() = g.cost_c2;
    ws.cell(r, 15).value() = g.cost_c1;
    ws.cell(r, 16).value() = g.cost_c0;
    ws.cell(r, 17).value() = bool_str(g.in_service);
    ws.cell(r, 18).value() = g.inertia_h;
    ws.cell(r, 19).value() = g.droop_r;
    ws.cell(r, 20).value() = g.ramp_up_mw_min;
    ws.cell(r, 21).value() = g.ramp_dn_mw_min;
  }
}

std::vector<Generator> read_generators(const XLWorksheet& ws) {
  std::vector<Generator> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Generator g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.pg_mw = dbl_from_str(cell_by_name(ws, r, col, "pg_mw"), g.pg_mw);
    g.qg_mvar = dbl_from_str(cell_by_name(ws, r, col, "qg_mvar"), g.qg_mvar);
    g.vg_pu = dbl_from_str(cell_by_name(ws, r, col, "vg_pu"), g.vg_pu);
    g.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), g.pmax_mw);
    g.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), g.pmin_mw);
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), g.qmax_mvar);
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), g.qmin_mvar);
    g.is_slack = bool_from_str(cell_by_name(ws, r, col, "is_slack"), g.is_slack);
    g.fuel_type = fuel_type_from_str(cell_by_name(ws, r, col, "fuel_type"));
    g.mbase_mva = dbl_from_str(cell_by_name(ws, r, col, "mbase_mva"), g.mbase_mva);
    g.cost_c2 = dbl_from_str(cell_by_name(ws, r, col, "cost_c2"), g.cost_c2);
    g.cost_c1 = dbl_from_str(cell_by_name(ws, r, col, "cost_c1"), g.cost_c1);
    g.cost_c0 = dbl_from_str(cell_by_name(ws, r, col, "cost_c0"), g.cost_c0);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    g.inertia_h = dbl_from_str(cell_by_name(ws, r, col, "inertia_h"), g.inertia_h);
    g.droop_r = dbl_from_str(cell_by_name(ws, r, col, "droop_r"), g.droop_r);
    g.ramp_up_mw_min = dbl_from_str(cell_by_name(ws, r, col, "ramp_up_mw_min"), g.ramp_up_mw_min);
    g.ramp_dn_mw_min = dbl_from_str(cell_by_name(ws, r, col, "ramp_dn_mw_min"), g.ramp_dn_mw_min);
    out.push_back(std::move(g));
  }
  return out;
}

void write_loads(XLWorksheet ws, const std::vector<Load>& loads) {
  write_headers(ws, {"index", "name", "bus", "p_mw", "q_mvar", "model", "z_percent_p", "i_percent_p", "p_percent_p", "z_percent_q",
                     "i_percent_q", "p_percent_q", "controllable", "priority", "cost_mw", "in_service"});
  for (size_t i = 0; i < loads.size(); ++i) {
    const auto& l = loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.bus;
    ws.cell(r, 4).value() = l.p_mw;
    ws.cell(r, 5).value() = l.q_mvar;
    ws.cell(r, 6).value() = load_model_str(l.model);
    ws.cell(r, 7).value() = l.z_percent_p;
    ws.cell(r, 8).value() = l.i_percent_p;
    ws.cell(r, 9).value() = l.p_percent_p;
    ws.cell(r, 10).value() = l.z_percent_q;
    ws.cell(r, 11).value() = l.i_percent_q;
    ws.cell(r, 12).value() = l.p_percent_q;
    ws.cell(r, 13).value() = bool_str(l.controllable);
    ws.cell(r, 14).value() = load_priority_str(l.priority);
    ws.cell(r, 15).value() = l.cost_mw;
    ws.cell(r, 16).value() = bool_str(l.in_service);
  }
}

std::vector<Load> read_loads(const XLWorksheet& ws) {
  std::vector<Load> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Load l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.bus = int_from_str(cell_by_name(ws, r, col, "bus"), l.bus);
    l.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), l.p_mw);
    l.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), l.q_mvar);
    l.model = load_model_from_str(cell_by_name(ws, r, col, "model"));
    l.z_percent_p = dbl_from_str(cell_by_name(ws, r, col, "z_percent_p"), l.z_percent_p);
    l.i_percent_p = dbl_from_str(cell_by_name(ws, r, col, "i_percent_p"), l.i_percent_p);
    l.p_percent_p = dbl_from_str(cell_by_name(ws, r, col, "p_percent_p"), l.p_percent_p);
    l.z_percent_q = dbl_from_str(cell_by_name(ws, r, col, "z_percent_q"), l.z_percent_q);
    l.i_percent_q = dbl_from_str(cell_by_name(ws, r, col, "i_percent_q"), l.i_percent_q);
    l.p_percent_q = dbl_from_str(cell_by_name(ws, r, col, "p_percent_q"), l.p_percent_q);
    l.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), l.controllable);
    l.priority = load_priority_from_str(cell_by_name(ws, r, col, "priority"));
    l.cost_mw = dbl_from_str(cell_by_name(ws, r, col, "cost_mw"), l.cost_mw);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_flexible_loads(XLWorksheet ws, const std::vector<FlexibleLoad>& loads) {
  write_headers(ws, {"index", "name", "bus", "p_mw", "q_mvar", "flex_up_mw", "flex_down_mw", "flex_duration_h",
                     "response_time_s", "ramp_rate_mw_min", "availability_pct", "controllable", "priority",
                     "control_area", "in_service"});
  for (size_t i = 0; i < loads.size(); ++i) {
    const auto& l = loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.bus;
    ws.cell(r, 4).value() = l.p_mw;
    ws.cell(r, 5).value() = l.q_mvar;
    ws.cell(r, 6).value() = l.flex_up_mw;
    ws.cell(r, 7).value() = l.flex_down_mw;
    ws.cell(r, 8).value() = l.flex_duration_h;
    ws.cell(r, 9).value() = l.response_time_s;
    ws.cell(r, 10).value() = l.ramp_rate_mw_min;
    ws.cell(r, 11).value() = l.availability_pct;
    ws.cell(r, 12).value() = bool_str(l.controllable);
    ws.cell(r, 13).value() = load_priority_str(l.priority);
    ws.cell(r, 14).value() = l.control_area;
    ws.cell(r, 15).value() = bool_str(l.in_service);
  }
}

std::vector<FlexibleLoad> read_flexible_loads(const XLWorksheet& ws) {
  std::vector<FlexibleLoad> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    FlexibleLoad l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.bus = int_from_str(cell_by_name(ws, r, col, "bus"), l.bus);
    l.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), l.p_mw);
    l.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), l.q_mvar);
    l.flex_up_mw = dbl_from_str(cell_by_name(ws, r, col, "flex_up_mw"), l.flex_up_mw);
    l.flex_down_mw = dbl_from_str(cell_by_name(ws, r, col, "flex_down_mw"), l.flex_down_mw);
    l.flex_duration_h = dbl_from_str(cell_by_name(ws, r, col, "flex_duration_h"), l.flex_duration_h);
    l.response_time_s = dbl_from_str(cell_by_name(ws, r, col, "response_time_s"), l.response_time_s);
    l.ramp_rate_mw_min = dbl_from_str(cell_by_name(ws, r, col, "ramp_rate_mw_min"), l.ramp_rate_mw_min);
    l.availability_pct = dbl_from_str(cell_by_name(ws, r, col, "availability_pct"), l.availability_pct);
    l.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), l.controllable);
    l.priority = load_priority_from_str(cell_by_name(ws, r, col, "priority"));
    l.control_area = cell_by_name(ws, r, col, "control_area");
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_asymmetric_loads(XLWorksheet ws, const std::vector<AsymmetricLoad>& loads) {
  write_headers(ws, {"index", "name", "bus", "connection", "grounded",
                     "pa_rated_mw", "qa_rated_mvar", "pb_rated_mw", "qb_rated_mvar", "pc_rated_mw", "qc_rated_mvar",
                     "pa_mw", "qa_mvar", "pb_mw", "qb_mvar", "pc_mw", "qc_mvar",
                     "scaling", "const_z_percent", "const_i_percent", "const_p_percent",
                     "controllable", "priority", "in_service"});
  for (size_t i = 0; i < loads.size(); ++i) {
    const auto& l = loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.bus;
    ws.cell(r, 4).value() = l.connection;
    ws.cell(r, 5).value() = bool_str(l.grounded);
    ws.cell(r, 6).value() = l.pa_rated_mw;
    ws.cell(r, 7).value() = l.qa_rated_mvar;
    ws.cell(r, 8).value() = l.pb_rated_mw;
    ws.cell(r, 9).value() = l.qb_rated_mvar;
    ws.cell(r, 10).value() = l.pc_rated_mw;
    ws.cell(r, 11).value() = l.qc_rated_mvar;
    ws.cell(r, 12).value() = l.pa_mw;
    ws.cell(r, 13).value() = l.qa_mvar;
    ws.cell(r, 14).value() = l.pb_mw;
    ws.cell(r, 15).value() = l.qb_mvar;
    ws.cell(r, 16).value() = l.pc_mw;
    ws.cell(r, 17).value() = l.qc_mvar;
    ws.cell(r, 18).value() = l.scaling;
    ws.cell(r, 19).value() = l.const_z_percent;
    ws.cell(r, 20).value() = l.const_i_percent;
    ws.cell(r, 21).value() = l.const_p_percent;
    ws.cell(r, 22).value() = bool_str(l.controllable);
    ws.cell(r, 23).value() = load_priority_str(l.priority);
    ws.cell(r, 24).value() = bool_str(l.in_service);
  }
}

std::vector<AsymmetricLoad> read_asymmetric_loads(const XLWorksheet& ws) {
  std::vector<AsymmetricLoad> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    AsymmetricLoad l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.bus = int_from_str(cell_by_name(ws, r, col, "bus"), l.bus);
    l.connection = cell_by_name(ws, r, col, "connection");
    l.grounded = bool_from_str(cell_by_name(ws, r, col, "grounded"), l.grounded);
    l.pa_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "pa_rated_mw"), l.pa_rated_mw);
    l.qa_rated_mvar = dbl_from_str(cell_by_name(ws, r, col, "qa_rated_mvar"), l.qa_rated_mvar);
    l.pb_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "pb_rated_mw"), l.pb_rated_mw);
    l.qb_rated_mvar = dbl_from_str(cell_by_name(ws, r, col, "qb_rated_mvar"), l.qb_rated_mvar);
    l.pc_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "pc_rated_mw"), l.pc_rated_mw);
    l.qc_rated_mvar = dbl_from_str(cell_by_name(ws, r, col, "qc_rated_mvar"), l.qc_rated_mvar);
    l.pa_mw = dbl_from_str(cell_by_name(ws, r, col, "pa_mw"), l.pa_mw);
    l.qa_mvar = dbl_from_str(cell_by_name(ws, r, col, "qa_mvar"), l.qa_mvar);
    l.pb_mw = dbl_from_str(cell_by_name(ws, r, col, "pb_mw"), l.pb_mw);
    l.qb_mvar = dbl_from_str(cell_by_name(ws, r, col, "qb_mvar"), l.qb_mvar);
    l.pc_mw = dbl_from_str(cell_by_name(ws, r, col, "pc_mw"), l.pc_mw);
    l.qc_mvar = dbl_from_str(cell_by_name(ws, r, col, "qc_mvar"), l.qc_mvar);
    l.scaling = dbl_from_str(cell_by_name(ws, r, col, "scaling"), l.scaling);
    l.const_z_percent = dbl_from_str(cell_by_name(ws, r, col, "const_z_percent"), l.const_z_percent);
    l.const_i_percent = dbl_from_str(cell_by_name(ws, r, col, "const_i_percent"), l.const_i_percent);
    l.const_p_percent = dbl_from_str(cell_by_name(ws, r, col, "const_p_percent"), l.const_p_percent);
    l.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), l.controllable);
    l.priority = load_priority_from_str(cell_by_name(ws, r, col, "priority"));
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_dc_loads(XLWorksheet ws, const std::vector<DCLoad>& loads) {
  write_headers(ws, {"index", "name", "bus", "p_mw", "controllable", "p_min_mw", "cost_mw", "in_service"});
  for (size_t i = 0; i < loads.size(); ++i) {
    const auto& l = loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.bus;
    ws.cell(r, 4).value() = l.p_mw;
    ws.cell(r, 5).value() = bool_str(l.controllable);
    ws.cell(r, 6).value() = l.p_min_mw;
    ws.cell(r, 7).value() = l.cost_mw;
    ws.cell(r, 8).value() = bool_str(l.in_service);
  }
}

std::vector<DCLoad> read_dc_loads(const XLWorksheet& ws) {
  std::vector<DCLoad> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    DCLoad l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.bus = int_from_str(cell_by_name(ws, r, col, "bus"), l.bus);
    l.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), l.p_mw);
    l.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), l.controllable);
    l.p_min_mw = dbl_from_str(cell_by_name(ws, r, col, "p_min_mw"), l.p_min_mw);
    l.cost_mw = dbl_from_str(cell_by_name(ws, r, col, "cost_mw"), l.cost_mw);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_dc_static_generators(XLWorksheet ws, const std::vector<StaticGeneratorDC>& gens) {
  write_headers(ws, {"index", "name", "bus", "type", "p_set_mw", "scaling", "profile_id", "pmax_mw", "pmin_mw",
                     "controllable", "mtbf_hr", "mttr_hr", "t_scheduled_hr", "in_service"});
  for (size_t i = 0; i < gens.size(); ++i) {
    const auto& g = gens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = g.type;
    ws.cell(r, 5).value() = g.p_set_mw;
    ws.cell(r, 6).value() = g.scaling;
    ws.cell(r, 7).value() = g.profile_id;
    ws.cell(r, 8).value() = g.pmax_mw;
    ws.cell(r, 9).value() = g.pmin_mw;
    ws.cell(r, 10).value() = bool_str(g.controllable);
    ws.cell(r, 11).value() = g.mtbf_hr;
    ws.cell(r, 12).value() = g.mttr_hr;
    ws.cell(r, 13).value() = g.t_scheduled_hr;
    ws.cell(r, 14).value() = bool_str(g.in_service);
  }
}

std::vector<StaticGeneratorDC> read_dc_static_generators(const XLWorksheet& ws) {
  std::vector<StaticGeneratorDC> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    StaticGeneratorDC g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.type = cell_by_name(ws, r, col, "type");
    g.p_set_mw = dbl_from_str(cell_by_name(ws, r, col, "p_set_mw"), g.p_set_mw);
    g.scaling = dbl_from_str(cell_by_name(ws, r, col, "scaling"), g.scaling);
    g.profile_id = int_from_str(cell_by_name(ws, r, col, "profile_id"), g.profile_id);
    g.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), g.pmax_mw);
    g.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), g.pmin_mw);
    g.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), g.controllable);
    g.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), g.mtbf_hr);
    g.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), g.mttr_hr);
    g.t_scheduled_hr = dbl_from_str(cell_by_name(ws, r, col, "t_scheduled_hr"), g.t_scheduled_hr);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_dc_pv_arrays(XLWorksheet ws, const std::vector<PVArrayDC>& arrays) {
  write_headers(ws, {"index", "name", "bus", "p_set_mw", "profile_id", "num_series", "num_parallel", "vmpp", "impp",
                     "voc", "isc", "alpha_isc", "beta_voc", "temperature", "irradiance", "mtbf_hr", "mttr_hr",
                     "t_scheduled_hr", "in_service"});
  for (size_t i = 0; i < arrays.size(); ++i) {
    const auto& p = arrays[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = p.index;
    ws.cell(r, 2).value() = p.name;
    ws.cell(r, 3).value() = p.bus;
    ws.cell(r, 4).value() = p.p_set_mw;
    ws.cell(r, 5).value() = p.profile_id;
    ws.cell(r, 6).value() = p.num_series;
    ws.cell(r, 7).value() = p.num_parallel;
    ws.cell(r, 8).value() = p.vmpp;
    ws.cell(r, 9).value() = p.impp;
    ws.cell(r, 10).value() = p.voc;
    ws.cell(r, 11).value() = p.isc;
    ws.cell(r, 12).value() = p.alpha_isc;
    ws.cell(r, 13).value() = p.beta_voc;
    ws.cell(r, 14).value() = p.temperature;
    ws.cell(r, 15).value() = p.irradiance;
    ws.cell(r, 16).value() = p.mtbf_hr;
    ws.cell(r, 17).value() = p.mttr_hr;
    ws.cell(r, 18).value() = p.t_scheduled_hr;
    ws.cell(r, 19).value() = bool_str(p.in_service);
  }
}

std::vector<PVArrayDC> read_dc_pv_arrays(const XLWorksheet& ws) {
  std::vector<PVArrayDC> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    PVArrayDC p;
    p.index = int_from_str(idx, p.index);
    p.name = cell_by_name(ws, r, col, "name");
    p.bus = int_from_str(cell_by_name(ws, r, col, "bus"), p.bus);
    p.p_set_mw = dbl_from_str(cell_by_name(ws, r, col, "p_set_mw"), p.p_set_mw);
    p.profile_id = int_from_str(cell_by_name(ws, r, col, "profile_id"), p.profile_id);
    p.num_series = int_from_str(cell_by_name(ws, r, col, "num_series"), p.num_series);
    p.num_parallel = int_from_str(cell_by_name(ws, r, col, "num_parallel"), p.num_parallel);
    p.vmpp = dbl_from_str(cell_by_name(ws, r, col, "vmpp"), p.vmpp);
    p.impp = dbl_from_str(cell_by_name(ws, r, col, "impp"), p.impp);
    p.voc = dbl_from_str(cell_by_name(ws, r, col, "voc"), p.voc);
    p.isc = dbl_from_str(cell_by_name(ws, r, col, "isc"), p.isc);
    p.alpha_isc = dbl_from_str(cell_by_name(ws, r, col, "alpha_isc"), p.alpha_isc);
    p.beta_voc = dbl_from_str(cell_by_name(ws, r, col, "beta_voc"), p.beta_voc);
    p.temperature = dbl_from_str(cell_by_name(ws, r, col, "temperature"), p.temperature);
    p.irradiance = dbl_from_str(cell_by_name(ws, r, col, "irradiance"), p.irradiance);
    p.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), p.mtbf_hr);
    p.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), p.mttr_hr);
    p.t_scheduled_hr = dbl_from_str(cell_by_name(ws, r, col, "t_scheduled_hr"), p.t_scheduled_hr);
    p.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), p.in_service);
    out.push_back(std::move(p));
  }
  return out;
}

void write_static_gens(XLWorksheet ws, const std::vector<StaticGenerator>& sgens) {
  write_headers(ws, {"index", "name", "bus", "sgen_type", "p_mw", "q_mvar", "p_rated_mw", "sn_mva", "pmax_mw", "pmin_mw", "qmax_mvar",
                     "qmin_mvar", "scaling", "controllable", "in_service"});
  for (size_t i = 0; i < sgens.size(); ++i) {
    const auto& g = sgens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = sgen_type_str(g.sgen_type);
    ws.cell(r, 5).value() = g.p_mw;
    ws.cell(r, 6).value() = g.q_mvar;
    ws.cell(r, 7).value() = g.p_rated_mw;
    ws.cell(r, 8).value() = g.sn_mva;
    ws.cell(r, 9).value() = g.pmax_mw;
    ws.cell(r, 10).value() = g.pmin_mw;
    ws.cell(r, 11).value() = g.qmax_mvar;
    ws.cell(r, 12).value() = g.qmin_mvar;
    ws.cell(r, 13).value() = g.scaling;
    ws.cell(r, 14).value() = bool_str(g.controllable);
    ws.cell(r, 15).value() = bool_str(g.in_service);
  }
}

std::vector<StaticGenerator> read_static_gens(const XLWorksheet& ws) {
  std::vector<StaticGenerator> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    StaticGenerator g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.sgen_type = sgen_type_from_str(cell_by_name(ws, r, col, "sgen_type"));
    g.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), g.p_mw);
    g.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), g.q_mvar);
    g.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), g.p_rated_mw);
    g.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), g.sn_mva);
    g.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), g.pmax_mw);
    g.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), g.pmin_mw);
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), g.qmax_mvar);
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), g.qmin_mvar);
    g.scaling = dbl_from_str(cell_by_name(ws, r, col, "scaling"), g.scaling);
    g.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), g.controllable);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_storage(XLWorksheet ws, const std::vector<Storage>& storage) {
  write_headers(ws, {"index", "name", "bus", "p_mw", "q_mvar", "p_rated_mw", "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar",
                     "e_rated_mwh", "soc_init", "soc_min", "soc_max", "soc_carbon_intensity_tco2_mwh",
                     "eta_charge", "eta_discharge", "in_service"});
  for (size_t i = 0; i < storage.size(); ++i) {
    const auto& s = storage[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = s.index;
    ws.cell(r, 2).value() = s.name;
    ws.cell(r, 3).value() = s.bus;
    ws.cell(r, 4).value() = s.p_mw;
    ws.cell(r, 5).value() = s.q_mvar;
    ws.cell(r, 6).value() = s.p_rated_mw;
    ws.cell(r, 7).value() = s.pmax_mw;
    ws.cell(r, 8).value() = s.pmin_mw;
    ws.cell(r, 9).value() = s.qmax_mvar;
    ws.cell(r, 10).value() = s.qmin_mvar;
    ws.cell(r, 11).value() = s.e_rated_mwh;
    ws.cell(r, 12).value() = s.soc_init;
    ws.cell(r, 13).value() = s.soc_min;
    ws.cell(r, 14).value() = s.soc_max;
    ws.cell(r, 15).value() = s.soc_carbon_intensity_tco2_mwh;
    ws.cell(r, 16).value() = s.eta_charge;
    ws.cell(r, 17).value() = s.eta_discharge;
    ws.cell(r, 18).value() = bool_str(s.in_service);
  }
}

std::vector<Storage> read_storage(const XLWorksheet& ws) {
  std::vector<Storage> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Storage s;
    s.index = int_from_str(idx, s.index);
    s.name = cell_by_name(ws, r, col, "name");
    s.bus = int_from_str(cell_by_name(ws, r, col, "bus"), s.bus);
    s.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), s.p_mw);
    s.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), s.q_mvar);
    s.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), s.p_rated_mw);
    s.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), s.pmax_mw);
    s.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), s.pmin_mw);
    s.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), s.qmax_mvar);
    s.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), s.qmin_mvar);
    s.e_rated_mwh = dbl_from_str(cell_by_name(ws, r, col, "e_rated_mwh"), s.e_rated_mwh);
    s.soc_init = dbl_from_str(cell_by_name(ws, r, col, "soc_init"), s.soc_init);
    s.soc_min = dbl_from_str(cell_by_name(ws, r, col, "soc_min"), s.soc_min);
    s.soc_max = dbl_from_str(cell_by_name(ws, r, col, "soc_max"), s.soc_max);
    s.soc_carbon_intensity_tco2_mwh =
        dbl_from_str(cell_by_name(ws, r, col, "soc_carbon_intensity_tco2_mwh"),
                     s.soc_carbon_intensity_tco2_mwh);
    s.eta_charge = dbl_from_str(cell_by_name(ws, r, col, "eta_charge"), s.eta_charge);
    s.eta_discharge = dbl_from_str(cell_by_name(ws, r, col, "eta_discharge"), s.eta_discharge);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), s.in_service);
    out.push_back(std::move(s));
  }
  return out;
}

void write_renewable(XLWorksheet ws, const std::vector<RenewableGen>& gens) {
  write_headers(ws, {"index", "name", "bus", "type", "p_mw", "q_mvar", "p_rated_mw", "qmax_mvar", "qmin_mvar", "curtailable",
                     "capacity_factor", "in_service"});
  for (size_t i = 0; i < gens.size(); ++i) {
    const auto& g = gens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = renewable_type_str(g.type);
    ws.cell(r, 5).value() = g.p_mw;
    ws.cell(r, 6).value() = g.q_mvar;
    ws.cell(r, 7).value() = g.p_rated_mw;
    ws.cell(r, 8).value() = g.qmax_mvar;
    ws.cell(r, 9).value() = g.qmin_mvar;
    ws.cell(r, 10).value() = bool_str(g.curtailable);
    ws.cell(r, 11).value() = g.capacity_factor;
    ws.cell(r, 12).value() = bool_str(g.in_service);
  }
}

std::vector<RenewableGen> read_renewable(const XLWorksheet& ws) {
  std::vector<RenewableGen> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    RenewableGen g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.type = renewable_type_from_str(cell_by_name(ws, r, col, "type"));
    g.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), g.p_mw);
    g.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), g.q_mvar);
    g.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), g.p_rated_mw);
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), g.qmax_mvar);
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), g.qmin_mvar);
    g.curtailable = bool_from_str(cell_by_name(ws, r, col, "curtailable"), g.curtailable);
    g.capacity_factor = dbl_from_str(cell_by_name(ws, r, col, "capacity_factor"), g.capacity_factor);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_pv(XLWorksheet ws, const std::vector<PVSystem>& pvs) {
  write_headers(ws, {"index", "name", "bus", "p_mw", "q_mvar", "sn_mva", "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar", "control_mode",
                     "inverter_eff", "in_service"});
  for (size_t i = 0; i < pvs.size(); ++i) {
    const auto& p = pvs[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = p.index;
    ws.cell(r, 2).value() = p.name;
    ws.cell(r, 3).value() = p.bus;
    ws.cell(r, 4).value() = p.p_mw;
    ws.cell(r, 5).value() = p.q_mvar;
    ws.cell(r, 6).value() = p.sn_mva;
    ws.cell(r, 7).value() = p.pmax_mw;
    ws.cell(r, 8).value() = p.pmin_mw;
    ws.cell(r, 9).value() = p.qmax_mvar;
    ws.cell(r, 10).value() = p.qmin_mvar;
    ws.cell(r, 11).value() = pv_control_mode_str(p.control_mode);
    ws.cell(r, 12).value() = p.inverter_eff;
    ws.cell(r, 13).value() = bool_str(p.in_service);
  }
}

std::vector<PVSystem> read_pv(const XLWorksheet& ws) {
  std::vector<PVSystem> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    PVSystem p;
    p.index = int_from_str(idx, p.index);
    p.name = cell_by_name(ws, r, col, "name");
    p.bus = int_from_str(cell_by_name(ws, r, col, "bus"), p.bus);
    p.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), p.p_mw);
    p.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), p.q_mvar);
    p.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), p.sn_mva);
    p.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), p.pmax_mw);
    p.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), p.pmin_mw);
    p.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), p.qmax_mvar);
    p.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), p.qmin_mvar);
    p.control_mode = pv_control_mode_from_str(cell_by_name(ws, r, col, "control_mode"));
    p.inverter_eff = dbl_from_str(cell_by_name(ws, r, col, "inverter_eff"), p.inverter_eff);
    p.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), p.in_service);
    out.push_back(std::move(p));
  }
  return out;
}

void write_shunts(XLWorksheet ws, const std::vector<Shunt>& shunts) {
  write_headers(ws, {"index", "name", "bus", "gs_mw", "bs_mvar", "switchable", "n_steps", "current_step", "bs_per_step", "in_service"});
  for (size_t i = 0; i < shunts.size(); ++i) {
    const auto& s = shunts[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = s.index;
    ws.cell(r, 2).value() = s.name;
    ws.cell(r, 3).value() = s.bus;
    ws.cell(r, 4).value() = s.gs_mw;
    ws.cell(r, 5).value() = s.bs_mvar;
    ws.cell(r, 6).value() = bool_str(s.switchable);
    ws.cell(r, 7).value() = s.n_steps;
    ws.cell(r, 8).value() = s.current_step;
    ws.cell(r, 9).value() = s.bs_per_step;
    ws.cell(r, 10).value() = bool_str(s.in_service);
  }
}

std::vector<Shunt> read_shunts(const XLWorksheet& ws) {
  std::vector<Shunt> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Shunt s;
    s.index = int_from_str(idx, s.index);
    s.name = cell_by_name(ws, r, col, "name");
    s.bus = int_from_str(cell_by_name(ws, r, col, "bus"), s.bus);
    s.gs_mw = dbl_from_str(cell_by_name(ws, r, col, "gs_mw"), s.gs_mw);
    s.bs_mvar = dbl_from_str(cell_by_name(ws, r, col, "bs_mvar"), s.bs_mvar);
    s.switchable = bool_from_str(cell_by_name(ws, r, col, "switchable"), s.switchable);
    s.n_steps = int_from_str(cell_by_name(ws, r, col, "n_steps"), s.n_steps);
    s.current_step = int_from_str(cell_by_name(ws, r, col, "current_step"), s.current_step);
    s.bs_per_step = dbl_from_str(cell_by_name(ws, r, col, "bs_per_step"), s.bs_per_step);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), s.in_service);
    out.push_back(std::move(s));
  }
  return out;
}

void write_external_grids(XLWorksheet ws, const std::vector<ExternalGrid>& grids) {
  write_headers(ws, {"index", "name", "bus", "vm_pu", "va_deg", "s_sc_max_mva", "s_sc_min_mva", "rx_max", "rx_min", "controllable", "in_service"});
  for (size_t i = 0; i < grids.size(); ++i) {
    const auto& g = grids[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = g.vm_pu;
    ws.cell(r, 5).value() = g.va_deg;
    ws.cell(r, 6).value() = g.s_sc_max_mva;
    ws.cell(r, 7).value() = g.s_sc_min_mva;
    ws.cell(r, 8).value() = g.rx_max;
    ws.cell(r, 9).value() = g.rx_min;
    ws.cell(r, 10).value() = bool_str(g.controllable);
    ws.cell(r, 11).value() = bool_str(g.in_service);
  }
}

std::vector<ExternalGrid> read_external_grids(const XLWorksheet& ws) {
  std::vector<ExternalGrid> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ExternalGrid g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_pu"), g.vm_pu);
    g.va_deg = dbl_from_str(cell_by_name(ws, r, col, "va_deg"), g.va_deg);
    g.s_sc_max_mva = dbl_from_str(cell_by_name(ws, r, col, "s_sc_max_mva"), g.s_sc_max_mva);
    g.s_sc_min_mva = dbl_from_str(cell_by_name(ws, r, col, "s_sc_min_mva"), g.s_sc_min_mva);
    g.rx_max = dbl_from_str(cell_by_name(ws, r, col, "rx_max"), g.rx_max);
    g.rx_min = dbl_from_str(cell_by_name(ws, r, col, "rx_min"), g.rx_min);
    g.controllable = bool_from_str(cell_by_name(ws, r, col, "controllable"), g.controllable);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_transformers_2w(XLWorksheet ws, const std::vector<Transformer2W>& tx) {
  write_headers(ws, {"index", "name", "hv_bus", "lv_bus", "sn_mva", "vn_hv_kv", "vn_lv_kv", "vk_percent", "vkr_percent", "pfe_kw", "i0_percent",
                     "tap_side", "tap_pos", "tap_min", "tap_max", "tap_neutral", "tap_step_percent", "shift_deg", "vector_group", "in_service"});
  for (size_t i = 0; i < tx.size(); ++i) {
    const auto& t = tx[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = t.index;
    ws.cell(r, 2).value() = t.name;
    ws.cell(r, 3).value() = t.hv_bus;
    ws.cell(r, 4).value() = t.lv_bus;
    ws.cell(r, 5).value() = t.sn_mva;
    ws.cell(r, 6).value() = t.vn_hv_kv;
    ws.cell(r, 7).value() = t.vn_lv_kv;
    ws.cell(r, 8).value() = t.vk_percent;
    ws.cell(r, 9).value() = t.vkr_percent;
    ws.cell(r, 10).value() = t.pfe_kw;
    ws.cell(r, 11).value() = t.i0_percent;
    ws.cell(r, 12).value() = t.tap_side;
    ws.cell(r, 13).value() = t.tap_pos;
    ws.cell(r, 14).value() = t.tap_min;
    ws.cell(r, 15).value() = t.tap_max;
    ws.cell(r, 16).value() = t.tap_neutral;
    ws.cell(r, 17).value() = t.tap_step_percent;
    ws.cell(r, 18).value() = t.shift_deg;
    ws.cell(r, 19).value() = t.vector_group;
    ws.cell(r, 20).value() = bool_str(t.in_service);
  }
}

std::vector<Transformer2W> read_transformers_2w(const XLWorksheet& ws) {
  std::vector<Transformer2W> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Transformer2W t;
    t.index = int_from_str(idx, t.index);
    t.name = cell_by_name(ws, r, col, "name");
    t.hv_bus = int_from_str(cell_by_name(ws, r, col, "hv_bus"), t.hv_bus);
    t.lv_bus = int_from_str(cell_by_name(ws, r, col, "lv_bus"), t.lv_bus);
    t.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), t.sn_mva);
    t.vn_hv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_hv_kv"), t.vn_hv_kv);
    t.vn_lv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_lv_kv"), t.vn_lv_kv);
    t.vk_percent = dbl_from_str(cell_by_name(ws, r, col, "vk_percent"), t.vk_percent);
    t.vkr_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr_percent"), t.vkr_percent);
    t.pfe_kw = dbl_from_str(cell_by_name(ws, r, col, "pfe_kw"), t.pfe_kw);
    t.i0_percent = dbl_from_str(cell_by_name(ws, r, col, "i0_percent"), t.i0_percent);
    t.tap_side = int_from_str(cell_by_name(ws, r, col, "tap_side"), t.tap_side);
    t.tap_pos = int_from_str(cell_by_name(ws, r, col, "tap_pos"), t.tap_pos);
    t.tap_min = int_from_str(cell_by_name(ws, r, col, "tap_min"), t.tap_min);
    t.tap_max = int_from_str(cell_by_name(ws, r, col, "tap_max"), t.tap_max);
    t.tap_neutral = int_from_str(cell_by_name(ws, r, col, "tap_neutral"), t.tap_neutral);
    t.tap_step_percent = dbl_from_str(cell_by_name(ws, r, col, "tap_step_percent"), t.tap_step_percent);
    t.shift_deg = dbl_from_str(cell_by_name(ws, r, col, "shift_deg"), t.shift_deg);
    t.vector_group = cell_by_name(ws, r, col, "vector_group");
    t.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), t.in_service);
    out.push_back(std::move(t));
  }
  return out;
}

void write_transformers_3w(XLWorksheet ws, const std::vector<Transformer3W>& tx) {
  write_headers(ws, {"index", "name", "std_type", "hv_bus", "mv_bus", "lv_bus", "sn_hv_mva", "sn_mv_mva", "sn_lv_mva",
                     "vn_hv_kv", "vn_mv_kv", "vn_lv_kv",
                     "vk_hv_mv_percent", "vk_hv_lv_percent", "vk_mv_lv_percent",
                     "vkr_hv_mv_percent", "vkr_hv_lv_percent", "vkr_mv_lv_percent",
                     "pfe_kw", "i0_percent", "tap_side", "tap_pos", "tap_step_percent",
                     "shift_mv_deg", "shift_lv_deg", "mtbf_hr", "mttr_hr", "in_service"});
  for (size_t i = 0; i < tx.size(); ++i) {
    const auto& t = tx[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = t.index;
    ws.cell(r, 2).value() = t.name;
    ws.cell(r, 3).value() = t.std_type;
    ws.cell(r, 4).value() = t.hv_bus;
    ws.cell(r, 5).value() = t.mv_bus;
    ws.cell(r, 6).value() = t.lv_bus;
    ws.cell(r, 7).value() = t.sn_hv_mva;
    ws.cell(r, 8).value() = t.sn_mv_mva;
    ws.cell(r, 9).value() = t.sn_lv_mva;
    ws.cell(r, 10).value() = t.vn_hv_kv;
    ws.cell(r, 11).value() = t.vn_mv_kv;
    ws.cell(r, 12).value() = t.vn_lv_kv;
    ws.cell(r, 13).value() = t.vk_hv_mv_percent;
    ws.cell(r, 14).value() = t.vk_hv_lv_percent;
    ws.cell(r, 15).value() = t.vk_mv_lv_percent;
    ws.cell(r, 16).value() = t.vkr_hv_mv_percent;
    ws.cell(r, 17).value() = t.vkr_hv_lv_percent;
    ws.cell(r, 18).value() = t.vkr_mv_lv_percent;
    ws.cell(r, 19).value() = t.pfe_kw;
    ws.cell(r, 20).value() = t.i0_percent;
    ws.cell(r, 21).value() = t.tap_side;
    ws.cell(r, 22).value() = t.tap_pos;
    ws.cell(r, 23).value() = t.tap_step_percent;
    ws.cell(r, 24).value() = t.shift_mv_deg;
    ws.cell(r, 25).value() = t.shift_lv_deg;
    ws.cell(r, 26).value() = t.mtbf_hr;
    ws.cell(r, 27).value() = t.mttr_hr;
    ws.cell(r, 28).value() = bool_str(t.in_service);
  }
}

std::vector<Transformer3W> read_transformers_3w(const XLWorksheet& ws) {
  std::vector<Transformer3W> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Transformer3W t;
    t.index = int_from_str(idx, t.index);
    t.name = cell_by_name(ws, r, col, "name");
    t.std_type = cell_by_name(ws, r, col, "std_type");
    t.hv_bus = int_from_str(cell_by_name(ws, r, col, "hv_bus"), t.hv_bus);
    t.mv_bus = int_from_str(cell_by_name(ws, r, col, "mv_bus"), t.mv_bus);
    t.lv_bus = int_from_str(cell_by_name(ws, r, col, "lv_bus"), t.lv_bus);
    t.sn_hv_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_hv_mva"), t.sn_hv_mva);
    t.sn_mv_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mv_mva"), t.sn_mv_mva);
    t.sn_lv_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_lv_mva"), t.sn_lv_mva);
    t.vn_hv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_hv_kv"), t.vn_hv_kv);
    t.vn_mv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_mv_kv"), t.vn_mv_kv);
    t.vn_lv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_lv_kv"), t.vn_lv_kv);
    t.vk_hv_mv_percent = dbl_from_str(cell_by_name(ws, r, col, "vk_hv_mv_percent"), t.vk_hv_mv_percent);
    t.vk_hv_lv_percent = dbl_from_str(cell_by_name(ws, r, col, "vk_hv_lv_percent"), t.vk_hv_lv_percent);
    t.vk_mv_lv_percent = dbl_from_str(cell_by_name(ws, r, col, "vk_mv_lv_percent"), t.vk_mv_lv_percent);
    t.vkr_hv_mv_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr_hv_mv_percent"), t.vkr_hv_mv_percent);
    t.vkr_hv_lv_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr_hv_lv_percent"), t.vkr_hv_lv_percent);
    t.vkr_mv_lv_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr_mv_lv_percent"), t.vkr_mv_lv_percent);
    t.pfe_kw = dbl_from_str(cell_by_name(ws, r, col, "pfe_kw"), t.pfe_kw);
    t.i0_percent = dbl_from_str(cell_by_name(ws, r, col, "i0_percent"), t.i0_percent);
    t.tap_side = int_from_str(cell_by_name(ws, r, col, "tap_side"), t.tap_side);
    t.tap_pos = int_from_str(cell_by_name(ws, r, col, "tap_pos"), t.tap_pos);
    t.tap_step_percent = dbl_from_str(cell_by_name(ws, r, col, "tap_step_percent"), t.tap_step_percent);
    t.shift_mv_deg = dbl_from_str(cell_by_name(ws, r, col, "shift_mv_deg"), t.shift_mv_deg);
    t.shift_lv_deg = dbl_from_str(cell_by_name(ws, r, col, "shift_lv_deg"), t.shift_lv_deg);
    t.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), t.mtbf_hr);
    t.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), t.mttr_hr);
    t.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), t.in_service);
    out.push_back(std::move(t));
  }
  return out;
}

void write_charging_stations(XLWorksheet ws, const std::vector<ChargingStation>& stations) {
  write_headers(ws, {"index", "name", "bus", "n_fast", "n_slow", "num_chargers", "p_fast_max_kw", "p_slow_max_kw", "max_power_kw",
                     "simultaneity_factor", "power_factor", "p_total_kw", "q_total_kvar", "in_service"});
  for (size_t i = 0; i < stations.size(); ++i) {
    const auto& s = stations[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = s.index;
    ws.cell(r, 2).value() = s.name;
    ws.cell(r, 3).value() = s.bus;
    ws.cell(r, 4).value() = s.n_fast;
    ws.cell(r, 5).value() = s.n_slow;
    ws.cell(r, 6).value() = s.num_chargers;
    ws.cell(r, 7).value() = s.p_fast_max_kw;
    ws.cell(r, 8).value() = s.p_slow_max_kw;
    ws.cell(r, 9).value() = s.max_power_kw;
    ws.cell(r, 10).value() = s.simultaneity_factor;
    ws.cell(r, 11).value() = s.power_factor;
    ws.cell(r, 12).value() = s.p_total_kw;
    ws.cell(r, 13).value() = s.q_total_kvar;
    ws.cell(r, 14).value() = bool_str(s.in_service);
  }
}

std::vector<ChargingStation> read_charging_stations(const XLWorksheet& ws) {
  std::vector<ChargingStation> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ChargingStation s;
    s.index = int_from_str(idx, s.index);
    s.name = cell_by_name(ws, r, col, "name");
    s.bus = int_from_str(cell_by_name(ws, r, col, "bus"), s.bus);
    s.n_fast = int_from_str(cell_by_name(ws, r, col, "n_fast"), s.n_fast);
    s.n_slow = int_from_str(cell_by_name(ws, r, col, "n_slow"), s.n_slow);
    s.num_chargers = int_from_str(cell_by_name(ws, r, col, "num_chargers"), s.num_chargers);
    s.p_fast_max_kw = dbl_from_str(cell_by_name(ws, r, col, "p_fast_max_kw"), s.p_fast_max_kw);
    s.p_slow_max_kw = dbl_from_str(cell_by_name(ws, r, col, "p_slow_max_kw"), s.p_slow_max_kw);
    s.max_power_kw = dbl_from_str(cell_by_name(ws, r, col, "max_power_kw"), s.max_power_kw);
    s.simultaneity_factor = dbl_from_str(cell_by_name(ws, r, col, "simultaneity_factor"), s.simultaneity_factor);
    s.power_factor = dbl_from_str(cell_by_name(ws, r, col, "power_factor"), s.power_factor);
    s.p_total_kw = dbl_from_str(cell_by_name(ws, r, col, "p_total_kw"), s.p_total_kw);
    s.q_total_kvar = dbl_from_str(cell_by_name(ws, r, col, "q_total_kvar"), s.q_total_kvar);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), s.in_service);
    out.push_back(std::move(s));
  }
  return out;
}

void write_chargers(XLWorksheet ws, const std::vector<Charger>& chargers) {
  write_headers(ws, {"index", "name", "station_id", "charger_type", "in_service", "p_rated_kw", "p_ch_max_kw",
                     "p_ch_min_kw", "eta", "v2g_capable", "p_dis_max_kw", "mtbf_hr", "mttr_hr"});
  for (size_t i = 0; i < chargers.size(); ++i) {
    const auto& c = chargers[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = c.index;
    ws.cell(r, 2).value() = c.name;
    ws.cell(r, 3).value() = c.station_id;
    ws.cell(r, 4).value() = charger_type_str(c.charger_type);
    ws.cell(r, 5).value() = bool_str(c.in_service);
    ws.cell(r, 6).value() = c.p_rated_kw;
    ws.cell(r, 7).value() = c.p_ch_max_kw;
    ws.cell(r, 8).value() = c.p_ch_min_kw;
    ws.cell(r, 9).value() = c.eta;
    ws.cell(r, 10).value() = bool_str(c.v2g_capable);
    ws.cell(r, 11).value() = c.p_dis_max_kw;
    ws.cell(r, 12).value() = c.mtbf_hr;
    ws.cell(r, 13).value() = c.mttr_hr;
  }
}

std::vector<Charger> read_chargers(const XLWorksheet& ws) {
  std::vector<Charger> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Charger c;
    c.index = int_from_str(idx, c.index);
    c.name = cell_by_name(ws, r, col, "name");
    c.station_id = int_from_str(cell_by_name(ws, r, col, "station_id"), c.station_id);
    c.charger_type = charger_type_from_str(cell_by_name(ws, r, col, "charger_type"));
    c.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), c.in_service);
    c.p_rated_kw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_kw"), c.p_rated_kw);
    c.p_ch_max_kw = dbl_from_str(cell_by_name(ws, r, col, "p_ch_max_kw"), c.p_ch_max_kw);
    c.p_ch_min_kw = dbl_from_str(cell_by_name(ws, r, col, "p_ch_min_kw"), c.p_ch_min_kw);
    c.eta = dbl_from_str(cell_by_name(ws, r, col, "eta"), c.eta);
    c.v2g_capable = bool_from_str(cell_by_name(ws, r, col, "v2g_capable"), c.v2g_capable);
    c.p_dis_max_kw = dbl_from_str(cell_by_name(ws, r, col, "p_dis_max_kw"), c.p_dis_max_kw);
    c.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), c.mtbf_hr);
    c.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), c.mttr_hr);
    out.push_back(std::move(c));
  }
  return out;
}

void write_asynchronous_motors(XLWorksheet ws, const std::vector<AsynchronousMotor>& motors) {
  write_headers(ws, {"index", "name", "bus", "vn_kv", "sn_mva", "r_pu", "x_pu", "x_r", "lrc", "poles",
                     "cos_phi", "efficiency", "r0_pu", "x0_pu", "in_service"});
  for (size_t i = 0; i < motors.size(); ++i) {
    const auto& m = motors[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = m.index;
    ws.cell(r, 2).value() = m.name;
    ws.cell(r, 3).value() = m.bus;
    ws.cell(r, 4).value() = m.vn_kv;
    ws.cell(r, 5).value() = m.sn_mva;
    ws.cell(r, 6).value() = m.r_pu;
    ws.cell(r, 7).value() = m.x_pu;
    ws.cell(r, 8).value() = m.x_r;
    ws.cell(r, 9).value() = m.lrc;
    ws.cell(r, 10).value() = m.poles;
    ws.cell(r, 11).value() = m.cos_phi;
    ws.cell(r, 12).value() = m.efficiency;
    ws.cell(r, 13).value() = m.r0_pu;
    ws.cell(r, 14).value() = m.x0_pu;
    ws.cell(r, 15).value() = bool_str(m.in_service);
  }
}

std::vector<AsynchronousMotor> read_asynchronous_motors(const XLWorksheet& ws) {
  std::vector<AsynchronousMotor> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    AsynchronousMotor m;
    m.index = int_from_str(idx, m.index);
    m.name = cell_by_name(ws, r, col, "name");
    m.bus = int_from_str(cell_by_name(ws, r, col, "bus"), m.bus);
    m.vn_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_kv"), m.vn_kv);
    m.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), m.sn_mva);
    m.r_pu = dbl_from_str(cell_by_name(ws, r, col, "r_pu"), m.r_pu);
    m.x_pu = dbl_from_str(cell_by_name(ws, r, col, "x_pu"), m.x_pu);
    m.x_r = dbl_from_str(cell_by_name(ws, r, col, "x_r"), m.x_r);
    m.lrc = dbl_from_str(cell_by_name(ws, r, col, "lrc"), m.lrc);
    m.poles = int_from_str(cell_by_name(ws, r, col, "poles"), m.poles);
    m.cos_phi = dbl_from_str(cell_by_name(ws, r, col, "cos_phi"), m.cos_phi);
    m.efficiency = dbl_from_str(cell_by_name(ws, r, col, "efficiency"), m.efficiency);
    m.r0_pu = dbl_from_str(cell_by_name(ws, r, col, "r0_pu"), m.r0_pu);
    m.x0_pu = dbl_from_str(cell_by_name(ws, r, col, "x0_pu"), m.x0_pu);
    m.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), m.in_service);
    out.push_back(std::move(m));
  }
  return out;
}

void write_three_phase_buses(XLWorksheet ws, const std::vector<ThreePhaseACBus>& buses) {
  write_headers(ws, {"index", "name", "bus_type", "base_kv", "vm_a_pu", "va_a_deg", "vm_b_pu", "va_b_deg", "vm_c_pu", "va_c_deg",
                     "pd_a_mw", "qd_a_mvar", "pd_b_mw", "qd_b_mvar", "pd_c_mw", "qd_c_mvar",
                     "gs_a_mw", "bs_a_mvar", "gs_b_mw", "bs_b_mvar", "gs_c_mw", "bs_c_mvar",
                     "vmin_pu", "vmax_pu", "area", "zone", "in_service"});
  for (size_t i = 0; i < buses.size(); ++i) {
    const auto& b = buses[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = b.index;
    ws.cell(r, 2).value() = b.name;
    ws.cell(r, 3).value() = bus_type_str(b.bus_type);
    ws.cell(r, 4).value() = b.base_kv;
    ws.cell(r, 5).value() = b.vm_a_pu;
    ws.cell(r, 6).value() = b.va_a_deg;
    ws.cell(r, 7).value() = b.vm_b_pu;
    ws.cell(r, 8).value() = b.va_b_deg;
    ws.cell(r, 9).value() = b.vm_c_pu;
    ws.cell(r, 10).value() = b.va_c_deg;
    ws.cell(r, 11).value() = b.pd_a_mw;
    ws.cell(r, 12).value() = b.qd_a_mvar;
    ws.cell(r, 13).value() = b.pd_b_mw;
    ws.cell(r, 14).value() = b.qd_b_mvar;
    ws.cell(r, 15).value() = b.pd_c_mw;
    ws.cell(r, 16).value() = b.qd_c_mvar;
    ws.cell(r, 17).value() = b.gs_a_mw;
    ws.cell(r, 18).value() = b.bs_a_mvar;
    ws.cell(r, 19).value() = b.gs_b_mw;
    ws.cell(r, 20).value() = b.bs_b_mvar;
    ws.cell(r, 21).value() = b.gs_c_mw;
    ws.cell(r, 22).value() = b.bs_c_mvar;
    ws.cell(r, 23).value() = b.vmin_pu;
    ws.cell(r, 24).value() = b.vmax_pu;
    ws.cell(r, 25).value() = b.area;
    ws.cell(r, 26).value() = b.zone;
    ws.cell(r, 27).value() = bool_str(b.in_service);
  }
}

std::vector<ThreePhaseACBus> read_three_phase_buses(const XLWorksheet& ws) {
  std::vector<ThreePhaseACBus> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseACBus b;
    b.index = int_from_str(idx, b.index);
    b.name = cell_by_name(ws, r, col, "name");
    b.bus_type = bus_type_from_str(cell_by_name(ws, r, col, "bus_type"));
    b.base_kv = dbl_from_str(cell_by_name(ws, r, col, "base_kv"), b.base_kv);
    b.vm_a_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_a_pu"), b.vm_a_pu);
    b.va_a_deg = dbl_from_str(cell_by_name(ws, r, col, "va_a_deg"), b.va_a_deg);
    b.vm_b_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_b_pu"), b.vm_b_pu);
    b.va_b_deg = dbl_from_str(cell_by_name(ws, r, col, "va_b_deg"), b.va_b_deg);
    b.vm_c_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_c_pu"), b.vm_c_pu);
    b.va_c_deg = dbl_from_str(cell_by_name(ws, r, col, "va_c_deg"), b.va_c_deg);
    b.pd_a_mw = dbl_from_str(cell_by_name(ws, r, col, "pd_a_mw"), b.pd_a_mw);
    b.qd_a_mvar = dbl_from_str(cell_by_name(ws, r, col, "qd_a_mvar"), b.qd_a_mvar);
    b.pd_b_mw = dbl_from_str(cell_by_name(ws, r, col, "pd_b_mw"), b.pd_b_mw);
    b.qd_b_mvar = dbl_from_str(cell_by_name(ws, r, col, "qd_b_mvar"), b.qd_b_mvar);
    b.pd_c_mw = dbl_from_str(cell_by_name(ws, r, col, "pd_c_mw"), b.pd_c_mw);
    b.qd_c_mvar = dbl_from_str(cell_by_name(ws, r, col, "qd_c_mvar"), b.qd_c_mvar);
    b.gs_a_mw = dbl_from_str(cell_by_name(ws, r, col, "gs_a_mw"), b.gs_a_mw);
    b.bs_a_mvar = dbl_from_str(cell_by_name(ws, r, col, "bs_a_mvar"), b.bs_a_mvar);
    b.gs_b_mw = dbl_from_str(cell_by_name(ws, r, col, "gs_b_mw"), b.gs_b_mw);
    b.bs_b_mvar = dbl_from_str(cell_by_name(ws, r, col, "bs_b_mvar"), b.bs_b_mvar);
    b.gs_c_mw = dbl_from_str(cell_by_name(ws, r, col, "gs_c_mw"), b.gs_c_mw);
    b.bs_c_mvar = dbl_from_str(cell_by_name(ws, r, col, "bs_c_mvar"), b.bs_c_mvar);
    b.vmin_pu = dbl_from_str(cell_by_name(ws, r, col, "vmin_pu"), b.vmin_pu);
    b.vmax_pu = dbl_from_str(cell_by_name(ws, r, col, "vmax_pu"), b.vmax_pu);
    b.area = int_from_str(cell_by_name(ws, r, col, "area"), b.area);
    b.zone = int_from_str(cell_by_name(ws, r, col, "zone"), b.zone);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), b.in_service);
    out.push_back(std::move(b));
  }
  return out;
}

void write_three_phase_lines(XLWorksheet ws, const std::vector<ThreePhaseACLine>& lines) {
  write_headers(ws, {"index", "name", "from_bus", "to_bus", "length_km", "parallel",
                     "r1_ohm_per_km", "x1_ohm_per_km", "c1_nf_per_km",
                     "r0_ohm_per_km", "x0_ohm_per_km", "c0_nf_per_km",
                     "r1_pu", "x1_pu", "b1_pu", "r0_pu", "x0_pu", "b0_pu",
                     "max_i_ka", "rate_a_mva", "failure_rate", "mttr_hr", "in_service"});
  for (size_t i = 0; i < lines.size(); ++i) {
    const auto& l = lines[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.from_bus;
    ws.cell(r, 4).value() = l.to_bus;
    ws.cell(r, 5).value() = l.length_km;
    ws.cell(r, 6).value() = l.parallel;
    ws.cell(r, 7).value() = l.r1_ohm_per_km;
    ws.cell(r, 8).value() = l.x1_ohm_per_km;
    ws.cell(r, 9).value() = l.c1_nf_per_km;
    ws.cell(r, 10).value() = l.r0_ohm_per_km;
    ws.cell(r, 11).value() = l.x0_ohm_per_km;
    ws.cell(r, 12).value() = l.c0_nf_per_km;
    ws.cell(r, 13).value() = l.r1_pu;
    ws.cell(r, 14).value() = l.x1_pu;
    ws.cell(r, 15).value() = l.b1_pu;
    ws.cell(r, 16).value() = l.r0_pu;
    ws.cell(r, 17).value() = l.x0_pu;
    ws.cell(r, 18).value() = l.b0_pu;
    ws.cell(r, 19).value() = l.max_i_ka;
    ws.cell(r, 20).value() = l.rate_a_mva;
    ws.cell(r, 21).value() = l.failure_rate;
    ws.cell(r, 22).value() = l.mttr_hr;
    ws.cell(r, 23).value() = bool_str(l.in_service);
  }
}

std::vector<ThreePhaseACLine> read_three_phase_lines(const XLWorksheet& ws) {
  std::vector<ThreePhaseACLine> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseACLine l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.from_bus = int_from_str(cell_by_name(ws, r, col, "from_bus"), l.from_bus);
    l.to_bus = int_from_str(cell_by_name(ws, r, col, "to_bus"), l.to_bus);
    l.length_km = dbl_from_str(cell_by_name(ws, r, col, "length_km"), l.length_km);
    l.parallel = int_from_str(cell_by_name(ws, r, col, "parallel"), l.parallel);
    l.r1_ohm_per_km = dbl_from_str(cell_by_name(ws, r, col, "r1_ohm_per_km"), l.r1_ohm_per_km);
    l.x1_ohm_per_km = dbl_from_str(cell_by_name(ws, r, col, "x1_ohm_per_km"), l.x1_ohm_per_km);
    l.c1_nf_per_km = dbl_from_str(cell_by_name(ws, r, col, "c1_nf_per_km"), l.c1_nf_per_km);
    l.r0_ohm_per_km = dbl_from_str(cell_by_name(ws, r, col, "r0_ohm_per_km"), l.r0_ohm_per_km);
    l.x0_ohm_per_km = dbl_from_str(cell_by_name(ws, r, col, "x0_ohm_per_km"), l.x0_ohm_per_km);
    l.c0_nf_per_km = dbl_from_str(cell_by_name(ws, r, col, "c0_nf_per_km"), l.c0_nf_per_km);
    l.r1_pu = dbl_from_str(cell_by_name(ws, r, col, "r1_pu"), l.r1_pu);
    l.x1_pu = dbl_from_str(cell_by_name(ws, r, col, "x1_pu"), l.x1_pu);
    l.b1_pu = dbl_from_str(cell_by_name(ws, r, col, "b1_pu"), l.b1_pu);
    l.r0_pu = dbl_from_str(cell_by_name(ws, r, col, "r0_pu"), l.r0_pu);
    l.x0_pu = dbl_from_str(cell_by_name(ws, r, col, "x0_pu"), l.x0_pu);
    l.b0_pu = dbl_from_str(cell_by_name(ws, r, col, "b0_pu"), l.b0_pu);
    l.max_i_ka = dbl_from_str(cell_by_name(ws, r, col, "max_i_ka"), l.max_i_ka);
    l.rate_a_mva = dbl_from_str(cell_by_name(ws, r, col, "rate_a_mva"), l.rate_a_mva);
    l.failure_rate = dbl_from_str(cell_by_name(ws, r, col, "failure_rate"), l.failure_rate);
    l.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), l.mttr_hr);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_three_phase_transformers(XLWorksheet ws, const std::vector<ThreePhaseTransformer>& tx) {
  write_headers(ws, {"index", "name", "hv_bus", "lv_bus", "sn_mva", "vn_hv_kv", "vn_lv_kv",
                     "vk_percent", "vkr_percent", "pfe_kw", "i0_percent", "vector_group",
                     "vk0_percent", "vkr0_percent", "mag0_percent", "mag0_rx", "si0_hv_partial",
                     "tap_side", "tap_pos", "tap_min", "tap_max", "tap_neutral", "tap_step_percent",
                     "shift_deg", "mtbf_hr", "mttr_hr", "in_service"});
  for (size_t i = 0; i < tx.size(); ++i) {
    const auto& t = tx[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = t.index;
    ws.cell(r, 2).value() = t.name;
    ws.cell(r, 3).value() = t.hv_bus;
    ws.cell(r, 4).value() = t.lv_bus;
    ws.cell(r, 5).value() = t.sn_mva;
    ws.cell(r, 6).value() = t.vn_hv_kv;
    ws.cell(r, 7).value() = t.vn_lv_kv;
    ws.cell(r, 8).value() = t.vk_percent;
    ws.cell(r, 9).value() = t.vkr_percent;
    ws.cell(r, 10).value() = t.pfe_kw;
    ws.cell(r, 11).value() = t.i0_percent;
    ws.cell(r, 12).value() = t.vector_group;
    ws.cell(r, 13).value() = t.vk0_percent;
    ws.cell(r, 14).value() = t.vkr0_percent;
    ws.cell(r, 15).value() = t.mag0_percent;
    ws.cell(r, 16).value() = t.mag0_rx;
    ws.cell(r, 17).value() = t.si0_hv_partial;
    ws.cell(r, 18).value() = t.tap_side;
    ws.cell(r, 19).value() = t.tap_pos;
    ws.cell(r, 20).value() = t.tap_min;
    ws.cell(r, 21).value() = t.tap_max;
    ws.cell(r, 22).value() = t.tap_neutral;
    ws.cell(r, 23).value() = t.tap_step_percent;
    ws.cell(r, 24).value() = t.shift_deg;
    ws.cell(r, 25).value() = t.mtbf_hr;
    ws.cell(r, 26).value() = t.mttr_hr;
    ws.cell(r, 27).value() = bool_str(t.in_service);
  }
}

std::vector<ThreePhaseTransformer> read_three_phase_transformers(const XLWorksheet& ws) {
  std::vector<ThreePhaseTransformer> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseTransformer t;
    t.index = int_from_str(idx, t.index);
    t.name = cell_by_name(ws, r, col, "name");
    t.hv_bus = int_from_str(cell_by_name(ws, r, col, "hv_bus"), t.hv_bus);
    t.lv_bus = int_from_str(cell_by_name(ws, r, col, "lv_bus"), t.lv_bus);
    t.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), t.sn_mva);
    t.vn_hv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_hv_kv"), t.vn_hv_kv);
    t.vn_lv_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_lv_kv"), t.vn_lv_kv);
    t.vk_percent = dbl_from_str(cell_by_name(ws, r, col, "vk_percent"), t.vk_percent);
    t.vkr_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr_percent"), t.vkr_percent);
    t.pfe_kw = dbl_from_str(cell_by_name(ws, r, col, "pfe_kw"), t.pfe_kw);
    t.i0_percent = dbl_from_str(cell_by_name(ws, r, col, "i0_percent"), t.i0_percent);
    t.vector_group = cell_by_name(ws, r, col, "vector_group");
    t.vk0_percent = dbl_from_str(cell_by_name(ws, r, col, "vk0_percent"), t.vk0_percent);
    t.vkr0_percent = dbl_from_str(cell_by_name(ws, r, col, "vkr0_percent"), t.vkr0_percent);
    t.mag0_percent = dbl_from_str(cell_by_name(ws, r, col, "mag0_percent"), t.mag0_percent);
    t.mag0_rx = dbl_from_str(cell_by_name(ws, r, col, "mag0_rx"), t.mag0_rx);
    t.si0_hv_partial = dbl_from_str(cell_by_name(ws, r, col, "si0_hv_partial"), t.si0_hv_partial);
    t.tap_side = int_from_str(cell_by_name(ws, r, col, "tap_side"), t.tap_side);
    t.tap_pos = int_from_str(cell_by_name(ws, r, col, "tap_pos"), t.tap_pos);
    t.tap_min = int_from_str(cell_by_name(ws, r, col, "tap_min"), t.tap_min);
    t.tap_max = int_from_str(cell_by_name(ws, r, col, "tap_max"), t.tap_max);
    t.tap_neutral = int_from_str(cell_by_name(ws, r, col, "tap_neutral"), t.tap_neutral);
    t.tap_step_percent = dbl_from_str(cell_by_name(ws, r, col, "tap_step_percent"), t.tap_step_percent);
    t.shift_deg = dbl_from_str(cell_by_name(ws, r, col, "shift_deg"), t.shift_deg);
    t.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), t.mtbf_hr);
    t.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), t.mttr_hr);
    t.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), t.in_service);
    out.push_back(std::move(t));
  }
  return out;
}

void write_three_phase_loads(XLWorksheet ws, const std::vector<ThreePhaseLoad>& loads) {
  write_headers(ws, {"index", "name", "bus", "connection", "grounded",
                     "p_a_mw", "q_a_mvar", "p_b_mw", "q_b_mvar", "p_c_mw", "q_c_mvar",
                     "const_z_percent", "const_i_percent", "const_p_percent",
                     "motor_percent", "lrc_pu", "x_r_ratio", "in_service"});
  for (size_t i = 0; i < loads.size(); ++i) {
    const auto& l = loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = l.index;
    ws.cell(r, 2).value() = l.name;
    ws.cell(r, 3).value() = l.bus;
    ws.cell(r, 4).value() = l.connection;
    ws.cell(r, 5).value() = bool_str(l.grounded);
    ws.cell(r, 6).value() = l.p_a_mw;
    ws.cell(r, 7).value() = l.q_a_mvar;
    ws.cell(r, 8).value() = l.p_b_mw;
    ws.cell(r, 9).value() = l.q_b_mvar;
    ws.cell(r, 10).value() = l.p_c_mw;
    ws.cell(r, 11).value() = l.q_c_mvar;
    ws.cell(r, 12).value() = l.const_z_percent;
    ws.cell(r, 13).value() = l.const_i_percent;
    ws.cell(r, 14).value() = l.const_p_percent;
    ws.cell(r, 15).value() = l.motor_percent;
    ws.cell(r, 16).value() = l.lrc_pu;
    ws.cell(r, 17).value() = l.x_r_ratio;
    ws.cell(r, 18).value() = bool_str(l.in_service);
  }
}

std::vector<ThreePhaseLoad> read_three_phase_loads(const XLWorksheet& ws) {
  std::vector<ThreePhaseLoad> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseLoad l;
    l.index = int_from_str(idx, l.index);
    l.name = cell_by_name(ws, r, col, "name");
    l.bus = int_from_str(cell_by_name(ws, r, col, "bus"), l.bus);
    l.connection = cell_by_name(ws, r, col, "connection");
    l.grounded = bool_from_str(cell_by_name(ws, r, col, "grounded"), l.grounded);
    l.p_a_mw = dbl_from_str(cell_by_name(ws, r, col, "p_a_mw"), l.p_a_mw);
    l.q_a_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_a_mvar"), l.q_a_mvar);
    l.p_b_mw = dbl_from_str(cell_by_name(ws, r, col, "p_b_mw"), l.p_b_mw);
    l.q_b_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_b_mvar"), l.q_b_mvar);
    l.p_c_mw = dbl_from_str(cell_by_name(ws, r, col, "p_c_mw"), l.p_c_mw);
    l.q_c_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_c_mvar"), l.q_c_mvar);
    l.const_z_percent = dbl_from_str(cell_by_name(ws, r, col, "const_z_percent"), l.const_z_percent);
    l.const_i_percent = dbl_from_str(cell_by_name(ws, r, col, "const_i_percent"), l.const_i_percent);
    l.const_p_percent = dbl_from_str(cell_by_name(ws, r, col, "const_p_percent"), l.const_p_percent);
    l.motor_percent = dbl_from_str(cell_by_name(ws, r, col, "motor_percent"), l.motor_percent);
    l.lrc_pu = dbl_from_str(cell_by_name(ws, r, col, "lrc_pu"), l.lrc_pu);
    l.x_r_ratio = dbl_from_str(cell_by_name(ws, r, col, "x_r_ratio"), l.x_r_ratio);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), l.in_service);
    out.push_back(std::move(l));
  }
  return out;
}

void write_three_phase_generators(XLWorksheet ws, const std::vector<ThreePhaseGenerator>& gens) {
  write_headers(ws, {"index", "name", "bus", "is_slack", "p_mw", "q_mvar", "vm_pu",
                     "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar", "mbase_mva",
                     "xd_pu", "xdpp_pu", "x2_pu", "x0_pu", "r0_pu", "in_service"});
  for (size_t i = 0; i < gens.size(); ++i) {
    const auto& g = gens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = bool_str(g.is_slack);
    ws.cell(r, 5).value() = g.p_mw;
    ws.cell(r, 6).value() = g.q_mvar;
    ws.cell(r, 7).value() = g.vm_pu;
    ws.cell(r, 8).value() = g.pmax_mw;
    ws.cell(r, 9).value() = g.pmin_mw;
    ws.cell(r, 10).value() = g.qmax_mvar;
    ws.cell(r, 11).value() = g.qmin_mvar;
    ws.cell(r, 12).value() = g.mbase_mva;
    ws.cell(r, 13).value() = g.xd_pu;
    ws.cell(r, 14).value() = g.xdpp_pu;
    ws.cell(r, 15).value() = g.x2_pu;
    ws.cell(r, 16).value() = g.x0_pu;
    ws.cell(r, 17).value() = g.r0_pu;
    ws.cell(r, 18).value() = bool_str(g.in_service);
  }
}

std::vector<ThreePhaseGenerator> read_three_phase_generators(const XLWorksheet& ws) {
  std::vector<ThreePhaseGenerator> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseGenerator g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.is_slack = bool_from_str(cell_by_name(ws, r, col, "is_slack"), g.is_slack);
    g.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), g.p_mw);
    g.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), g.q_mvar);
    g.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_pu"), g.vm_pu);
    g.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), g.pmax_mw);
    g.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), g.pmin_mw);
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), g.qmax_mvar);
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), g.qmin_mvar);
    g.mbase_mva = dbl_from_str(cell_by_name(ws, r, col, "mbase_mva"), g.mbase_mva);
    g.xd_pu = dbl_from_str(cell_by_name(ws, r, col, "xd_pu"), g.xd_pu);
    g.xdpp_pu = dbl_from_str(cell_by_name(ws, r, col, "xdpp_pu"), g.xdpp_pu);
    g.x2_pu = dbl_from_str(cell_by_name(ws, r, col, "x2_pu"), g.x2_pu);
    g.x0_pu = dbl_from_str(cell_by_name(ws, r, col, "x0_pu"), g.x0_pu);
    g.r0_pu = dbl_from_str(cell_by_name(ws, r, col, "r0_pu"), g.r0_pu);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_three_phase_external_grids(XLWorksheet ws, const std::vector<ThreePhaseExternalGrid>& grids) {
  write_headers(ws, {"index", "name", "bus", "vm_pu", "va_deg", "s_sc_max_mva", "s_sc_min_mva",
                     "rx_max", "rx_min", "r1_pu", "x1_pu", "r2_pu", "x2_pu", "r0_pu", "x0_pu", "in_service"});
  for (size_t i = 0; i < grids.size(); ++i) {
    const auto& g = grids[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = g.index;
    ws.cell(r, 2).value() = g.name;
    ws.cell(r, 3).value() = g.bus;
    ws.cell(r, 4).value() = g.vm_pu;
    ws.cell(r, 5).value() = g.va_deg;
    ws.cell(r, 6).value() = g.s_sc_max_mva;
    ws.cell(r, 7).value() = g.s_sc_min_mva;
    ws.cell(r, 8).value() = g.rx_max;
    ws.cell(r, 9).value() = g.rx_min;
    ws.cell(r, 10).value() = g.r1_pu;
    ws.cell(r, 11).value() = g.x1_pu;
    ws.cell(r, 12).value() = g.r2_pu;
    ws.cell(r, 13).value() = g.x2_pu;
    ws.cell(r, 14).value() = g.r0_pu;
    ws.cell(r, 15).value() = g.x0_pu;
    ws.cell(r, 16).value() = bool_str(g.in_service);
  }
}

std::vector<ThreePhaseExternalGrid> read_three_phase_external_grids(const XLWorksheet& ws) {
  std::vector<ThreePhaseExternalGrid> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    ThreePhaseExternalGrid g;
    g.index = int_from_str(idx, g.index);
    g.name = cell_by_name(ws, r, col, "name");
    g.bus = int_from_str(cell_by_name(ws, r, col, "bus"), g.bus);
    g.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_pu"), g.vm_pu);
    g.va_deg = dbl_from_str(cell_by_name(ws, r, col, "va_deg"), g.va_deg);
    g.s_sc_max_mva = dbl_from_str(cell_by_name(ws, r, col, "s_sc_max_mva"), g.s_sc_max_mva);
    g.s_sc_min_mva = dbl_from_str(cell_by_name(ws, r, col, "s_sc_min_mva"), g.s_sc_min_mva);
    g.rx_max = dbl_from_str(cell_by_name(ws, r, col, "rx_max"), g.rx_max);
    g.rx_min = dbl_from_str(cell_by_name(ws, r, col, "rx_min"), g.rx_min);
    g.r1_pu = dbl_from_str(cell_by_name(ws, r, col, "r1_pu"), g.r1_pu);
    g.x1_pu = dbl_from_str(cell_by_name(ws, r, col, "x1_pu"), g.x1_pu);
    g.r2_pu = dbl_from_str(cell_by_name(ws, r, col, "r2_pu"), g.r2_pu);
    g.x2_pu = dbl_from_str(cell_by_name(ws, r, col, "x2_pu"), g.x2_pu);
    g.r0_pu = dbl_from_str(cell_by_name(ws, r, col, "r0_pu"), g.r0_pu);
    g.x0_pu = dbl_from_str(cell_by_name(ws, r, col, "x0_pu"), g.x0_pu);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), g.in_service);
    out.push_back(std::move(g));
  }
  return out;
}

void write_switches(XLWorksheet ws, const std::vector<Switch>& switches) {
  write_headers(ws, {"index", "name", "bus_from", "bus_to", "switch_type", "closed", "in_service"});
  for (size_t i = 0; i < switches.size(); ++i) {
    const auto& s = switches[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = s.index;
    ws.cell(r, 2).value() = s.name;
    ws.cell(r, 3).value() = s.bus_from;
    ws.cell(r, 4).value() = s.bus_to;
    ws.cell(r, 5).value() = switch_type_str(s.switch_type);
    ws.cell(r, 6).value() = bool_str(s.closed);
    ws.cell(r, 7).value() = bool_str(s.in_service);
  }
}

std::vector<Switch> read_switches(const XLWorksheet& ws) {
  std::vector<Switch> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Switch s;
    s.index = int_from_str(idx, s.index);
    s.name = cell_by_name(ws, r, col, "name");
    s.bus_from = int_from_str(cell_by_name(ws, r, col, "bus_from"), s.bus_from);
    s.bus_to = int_from_str(cell_by_name(ws, r, col, "bus_to"), s.bus_to);
    s.switch_type = switch_type_from_str(cell_by_name(ws, r, col, "switch_type"));
    s.closed = bool_from_str(cell_by_name(ws, r, col, "closed"), s.closed);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), s.in_service);
    out.push_back(std::move(s));
  }
  return out;
}

void write_circuit_breakers(XLWorksheet ws, const std::vector<CircuitBreaker>& cbs) {
  write_headers(ws, {"index", "name", "bus_from", "bus_to", "breaker_type", "closed",
                      "z_ohm", "rated_voltage_kv", "i_rated_ka", "i_breaking_ka",
                      "element_type", "element_id", "in_service"});
  for (size_t i = 0; i < cbs.size(); ++i) {
    const auto& cb = cbs[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = cb.index;
    ws.cell(r, 2).value() = cb.name;
    ws.cell(r, 3).value() = cb.bus_from;
    ws.cell(r, 4).value() = cb.bus_to;
    ws.cell(r, 5).value() = breaker_type_str(cb.breaker_type);
    ws.cell(r, 6).value() = bool_str(cb.closed);
    ws.cell(r, 7).value() = cb.z_ohm;
    ws.cell(r, 8).value() = cb.rated_voltage_kv;
    ws.cell(r, 9).value() = cb.i_rated_ka;
    ws.cell(r, 10).value() = cb.i_breaking_ka;
    ws.cell(r, 11).value() = cb.element_type;
    ws.cell(r, 12).value() = cb.element_id;
    ws.cell(r, 13).value() = bool_str(cb.in_service);
  }
}

std::vector<CircuitBreaker> read_circuit_breakers(const XLWorksheet& ws) {
  std::vector<CircuitBreaker> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    CircuitBreaker cb;
    cb.index = int_from_str(idx, cb.index);
    cb.name = cell_by_name(ws, r, col, "name");
    cb.bus_from = int_from_str(cell_by_name(ws, r, col, "bus_from"), cb.bus_from);
    cb.bus_to = int_from_str(cell_by_name(ws, r, col, "bus_to"), cb.bus_to);
    cb.breaker_type = breaker_type_from_str(cell_by_name(ws, r, col, "breaker_type"));
    cb.closed = bool_from_str(cell_by_name(ws, r, col, "closed"), cb.closed);
    cb.z_ohm = dbl_from_str(cell_by_name(ws, r, col, "z_ohm"), cb.z_ohm);
    cb.rated_voltage_kv = dbl_from_str(cell_by_name(ws, r, col, "rated_voltage_kv"), cb.rated_voltage_kv);
    cb.i_rated_ka = dbl_from_str(cell_by_name(ws, r, col, "i_rated_ka"), cb.i_rated_ka);
    cb.i_breaking_ka = dbl_from_str(cell_by_name(ws, r, col, "i_breaking_ka"), cb.i_breaking_ka);
    cb.element_type = cell_by_name(ws, r, col, "element_type");
    cb.element_id = int_from_str(cell_by_name(ws, r, col, "element_id"), cb.element_id);
    cb.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), cb.in_service);
    out.push_back(std::move(cb));
  }
  return out;
}

void write_dc_circuit_breakers(XLWorksheet ws, const std::vector<DCCircuitBreaker>& cbs) {
  write_headers(ws, {"index", "name", "bus_from", "bus_to", "breaker_type", "closed",
                      "r_ohm", "rated_voltage_kv", "i_rated_ka", "i_breaking_ka",
                      "element_type", "element_id", "in_service"});
  for (size_t i = 0; i < cbs.size(); ++i) {
    const auto& cb = cbs[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = cb.index;
    ws.cell(r, 2).value() = cb.name;
    ws.cell(r, 3).value() = cb.bus_from;
    ws.cell(r, 4).value() = cb.bus_to;
    ws.cell(r, 5).value() = breaker_type_str(cb.breaker_type);
    ws.cell(r, 6).value() = bool_str(cb.closed);
    ws.cell(r, 7).value() = cb.r_ohm;
    ws.cell(r, 8).value() = cb.rated_voltage_kv;
    ws.cell(r, 9).value() = cb.i_rated_ka;
    ws.cell(r, 10).value() = cb.i_breaking_ka;
    ws.cell(r, 11).value() = cb.element_type;
    ws.cell(r, 12).value() = cb.element_id;
    ws.cell(r, 13).value() = bool_str(cb.in_service);
  }
}

std::vector<DCCircuitBreaker> read_dc_circuit_breakers(const XLWorksheet& ws) {
  std::vector<DCCircuitBreaker> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    DCCircuitBreaker cb;
    cb.index = int_from_str(idx, cb.index);
    cb.name = cell_by_name(ws, r, col, "name");
    cb.bus_from = int_from_str(cell_by_name(ws, r, col, "bus_from"), cb.bus_from);
    cb.bus_to = int_from_str(cell_by_name(ws, r, col, "bus_to"), cb.bus_to);
    cb.breaker_type = breaker_type_from_str(cell_by_name(ws, r, col, "breaker_type"));
    cb.closed = bool_from_str(cell_by_name(ws, r, col, "closed"), cb.closed);
    cb.r_ohm = dbl_from_str(cell_by_name(ws, r, col, "r_ohm"), cb.r_ohm);
    cb.rated_voltage_kv = dbl_from_str(cell_by_name(ws, r, col, "rated_voltage_kv"), cb.rated_voltage_kv);
    cb.i_rated_ka = dbl_from_str(cell_by_name(ws, r, col, "i_rated_ka"), cb.i_rated_ka);
    cb.i_breaking_ka = dbl_from_str(cell_by_name(ws, r, col, "i_breaking_ka"), cb.i_breaking_ka);
    cb.element_type = cell_by_name(ws, r, col, "element_type");
    cb.element_id = int_from_str(cell_by_name(ws, r, col, "element_id"), cb.element_id);
    cb.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), cb.in_service);
    out.push_back(std::move(cb));
  }
  return out;
}

void write_dc_buses(XLWorksheet ws, const std::vector<DCBus>& buses) {
  write_headers(ws, {"index", "name", "bus_type", "vm_pu", "vmax_pu", "vmin_pu", "pd_mw", "in_service"});
  for (size_t i = 0; i < buses.size(); ++i) {
    const auto& b = buses[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = b.index;
    ws.cell(r, 2).value() = b.name;
    ws.cell(r, 3).value() = dc_bus_type_str(b.bus_type);
    ws.cell(r, 4).value() = b.vm_pu;
    ws.cell(r, 5).value() = b.vmax_pu;
    ws.cell(r, 6).value() = b.vmin_pu;
    ws.cell(r, 7).value() = b.pd_mw;
    ws.cell(r, 8).value() = bool_str(b.in_service);
  }
}

std::vector<DCBus> read_dc_buses(const XLWorksheet& ws) {
  std::vector<DCBus> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    DCBus b;
    b.index = int_from_str(idx, b.index);
    b.name = cell_by_name(ws, r, col, "name");
    b.bus_type = dc_bus_type_from_str(cell_by_name(ws, r, col, "bus_type"));
    b.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "vm_pu"), b.vm_pu);
    b.vmax_pu = dbl_from_str(cell_by_name(ws, r, col, "vmax_pu"), b.vmax_pu);
    b.vmin_pu = dbl_from_str(cell_by_name(ws, r, col, "vmin_pu"), b.vmin_pu);
    b.pd_mw = dbl_from_str(cell_by_name(ws, r, col, "pd_mw"), b.pd_mw);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), b.in_service);
    out.push_back(std::move(b));
  }
  return out;
}

void write_dc_branches(XLWorksheet ws, const std::vector<DCBranch>& branches) {
  write_headers(ws, {"index", "name", "from_bus", "to_bus", "r_pu", "rate_a_mva", "length_km", "in_service"});
  for (size_t i = 0; i < branches.size(); ++i) {
    const auto& b = branches[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = b.index;
    ws.cell(r, 2).value() = b.name;
    ws.cell(r, 3).value() = b.from_bus;
    ws.cell(r, 4).value() = b.to_bus;
    ws.cell(r, 5).value() = b.r_pu;
    ws.cell(r, 6).value() = b.rate_a_mva;
    ws.cell(r, 7).value() = b.length_km;
    ws.cell(r, 8).value() = bool_str(b.in_service);
  }
}

std::vector<DCBranch> read_dc_branches(const XLWorksheet& ws) {
  std::vector<DCBranch> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    DCBranch b;
    b.index = int_from_str(idx, b.index);
    b.name = cell_by_name(ws, r, col, "name");
    b.from_bus = int_from_str(cell_by_name(ws, r, col, "from_bus"), b.from_bus);
    b.to_bus = int_from_str(cell_by_name(ws, r, col, "to_bus"), b.to_bus);
    b.r_pu = dbl_from_str(cell_by_name(ws, r, col, "r_pu"), b.r_pu);
    b.rate_a_mva = dbl_from_str(cell_by_name(ws, r, col, "rate_a_mva"), b.rate_a_mva);
    b.length_km = dbl_from_str(cell_by_name(ws, r, col, "length_km"), b.length_km);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), b.in_service);
    out.push_back(std::move(b));
  }
  return out;
}

void write_vsc(XLWorksheet ws, const std::vector<VSCConverter>& vsc) {
  write_headers(ws, {"index", "name", "bus_ac", "bus_dc", "control_mode", "p_set_mw", "q_set_mvar", "v_dc_set_pu", "v_ac_set_pu", "eta",
                     "loss_percent", "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar", "p_rated_mw", "in_service"});
  for (size_t i = 0; i < vsc.size(); ++i) {
    const auto& c = vsc[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = c.index;
    ws.cell(r, 2).value() = c.name;
    ws.cell(r, 3).value() = c.bus_ac;
    ws.cell(r, 4).value() = c.bus_dc;
    ws.cell(r, 5).value() = converter_mode_str(c.control_mode);
    ws.cell(r, 6).value() = c.p_set_mw;
    ws.cell(r, 7).value() = c.q_set_mvar;
    ws.cell(r, 8).value() = c.v_dc_set_pu;
    ws.cell(r, 9).value() = c.v_ac_set_pu;
    ws.cell(r, 10).value() = c.eta;
    ws.cell(r, 11).value() = c.loss_percent;
    ws.cell(r, 12).value() = c.pmax_mw;
    ws.cell(r, 13).value() = c.pmin_mw;
    ws.cell(r, 14).value() = c.qmax_mvar;
    ws.cell(r, 15).value() = c.qmin_mvar;
    ws.cell(r, 16).value() = c.p_rated_mw;
    ws.cell(r, 17).value() = bool_str(c.in_service);
  }
}

std::vector<VSCConverter> read_vsc(const XLWorksheet& ws) {
  std::vector<VSCConverter> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    VSCConverter c;
    c.index = int_from_str(idx, c.index);
    c.name = cell_by_name(ws, r, col, "name");
    c.bus_ac = int_from_str(cell_by_name(ws, r, col, "bus_ac"), c.bus_ac);
    c.bus_dc = int_from_str(cell_by_name(ws, r, col, "bus_dc"), c.bus_dc);
    c.control_mode = converter_mode_from_str(cell_by_name(ws, r, col, "control_mode"));
    c.p_set_mw = dbl_from_str(cell_by_name(ws, r, col, "p_set_mw"), c.p_set_mw);
    c.q_set_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_set_mvar"), c.q_set_mvar);
    c.v_dc_set_pu = dbl_from_str(cell_by_name(ws, r, col, "v_dc_set_pu"), c.v_dc_set_pu);
    c.v_ac_set_pu = dbl_from_str(cell_by_name(ws, r, col, "v_ac_set_pu"), c.v_ac_set_pu);
    c.eta = dbl_from_str(cell_by_name(ws, r, col, "eta"), c.eta);
    c.loss_percent = dbl_from_str(cell_by_name(ws, r, col, "loss_percent"), c.loss_percent);
    c.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), c.pmax_mw);
    c.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), c.pmin_mw);
    c.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), c.qmax_mvar);
    c.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), c.qmin_mvar);
    c.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), c.p_rated_mw);
    c.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), c.in_service);
    out.push_back(std::move(c));
  }
  return out;
}

void write_dcdc(XLWorksheet ws, const std::vector<DCDCConverter>& dcdc) {
  write_headers(ws, {"index", "name", "bus_in", "bus_out", "control_mode", "p_ref_mw", "v_ref_pu", "sn_mva", "eta", "pmax_mw", "pmin_mw", "in_service"});
  for (size_t i = 0; i < dcdc.size(); ++i) {
    const auto& c = dcdc[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = c.index;
    ws.cell(r, 2).value() = c.name;
    ws.cell(r, 3).value() = c.bus_in;
    ws.cell(r, 4).value() = c.bus_out;
    ws.cell(r, 5).value() = dcdc_control_mode_str(c.control_mode);
    ws.cell(r, 6).value() = c.p_ref_mw;
    ws.cell(r, 7).value() = c.v_ref_pu;
    ws.cell(r, 8).value() = c.sn_mva;
    ws.cell(r, 9).value() = c.eta;
    ws.cell(r, 10).value() = c.pmax_mw;
    ws.cell(r, 11).value() = c.pmin_mw;
    ws.cell(r, 12).value() = bool_str(c.in_service);
  }
}

std::vector<DCDCConverter> read_dcdc(const XLWorksheet& ws) {
  std::vector<DCDCConverter> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    DCDCConverter c;
    c.index = int_from_str(idx, c.index);
    c.name = cell_by_name(ws, r, col, "name");
    c.bus_in = int_from_str(cell_by_name(ws, r, col, "bus_in"), c.bus_in);
    c.bus_out = int_from_str(cell_by_name(ws, r, col, "bus_out"), c.bus_out);
    c.control_mode = dcdc_control_mode_from_str(cell_by_name(ws, r, col, "control_mode"));
    c.p_ref_mw = dbl_from_str(cell_by_name(ws, r, col, "p_ref_mw"), c.p_ref_mw);
    c.v_ref_pu = dbl_from_str(cell_by_name(ws, r, col, "v_ref_pu"), c.v_ref_pu);
    c.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "sn_mva"), c.sn_mva);
    c.eta = dbl_from_str(cell_by_name(ws, r, col, "eta"), c.eta);
    c.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), c.pmax_mw);
    c.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), c.pmin_mw);
    c.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), c.in_service);
    out.push_back(std::move(c));
  }
  return out;
}

void write_energy_routers(XLWorksheet ws, const std::vector<EnergyRouter>& routers) {
  write_headers(ws, {"index", "name", "router_type", "num_ports", "p_rated_mw", "vn_ac_kv", "vn_dc_kv",
                     "loss_percent", "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar",
                     "mtbf_hr", "mttr_hr", "in_service"});
  for (size_t i = 0; i < routers.size(); ++i) {
    const auto& rtr = routers[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = rtr.index;
    ws.cell(r, 2).value() = rtr.name;
    ws.cell(r, 3).value() = rtr.router_type;
    ws.cell(r, 4).value() = rtr.num_ports;
    ws.cell(r, 5).value() = rtr.p_rated_mw;
    ws.cell(r, 6).value() = rtr.vn_ac_kv;
    ws.cell(r, 7).value() = rtr.vn_dc_kv;
    ws.cell(r, 8).value() = rtr.loss_percent;
    ws.cell(r, 9).value() = rtr.pmax_mw;
    ws.cell(r, 10).value() = rtr.pmin_mw;
    ws.cell(r, 11).value() = rtr.qmax_mvar;
    ws.cell(r, 12).value() = rtr.qmin_mvar;
    ws.cell(r, 13).value() = rtr.mtbf_hr;
    ws.cell(r, 14).value() = rtr.mttr_hr;
    ws.cell(r, 15).value() = bool_str(rtr.in_service);
  }
}

std::vector<EnergyRouter> read_energy_routers(const XLWorksheet& ws) {
  std::vector<EnergyRouter> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    EnergyRouter er;
    er.index = int_from_str(idx, er.index);
    er.name = cell_by_name(ws, r, col, "name");
    er.router_type = cell_by_name(ws, r, col, "router_type");
    er.num_ports = int_from_str(cell_by_name(ws, r, col, "num_ports"), er.num_ports);
    er.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), er.p_rated_mw);
    er.vn_ac_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_ac_kv"), er.vn_ac_kv);
    er.vn_dc_kv = dbl_from_str(cell_by_name(ws, r, col, "vn_dc_kv"), er.vn_dc_kv);
    er.loss_percent = dbl_from_str(cell_by_name(ws, r, col, "loss_percent"), er.loss_percent);
    er.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), er.pmax_mw);
    er.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), er.pmin_mw);
    er.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), er.qmax_mvar);
    er.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), er.qmin_mvar);
    er.mtbf_hr = dbl_from_str(cell_by_name(ws, r, col, "mtbf_hr"), er.mtbf_hr);
    er.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "mttr_hr"), er.mttr_hr);
    er.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), er.in_service);
    out.push_back(std::move(er));
  }
  return out;
}

void write_energy_router_ports(XLWorksheet ws, const std::vector<EnergyRouter>& routers) {
  write_headers(ws, {"router_index", "index", "name", "bus", "port_type", "voltage_level_kv", "p_mw", "q_mvar", "v_pu",
                     "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar",
                     "control_mode", "p_set_mw", "q_set_mvar", "v_set_pu", "in_service"});
  uint32_t row = 2;
  for (const auto& er : routers) {
    for (const auto& p : er.ports) {
      ws.cell(row, 1).value() = er.index;
      ws.cell(row, 2).value() = p.index;
      ws.cell(row, 3).value() = p.name;
      ws.cell(row, 4).value() = p.bus;
      ws.cell(row, 5).value() = er_port_type_str(p.port_type);
      ws.cell(row, 6).value() = p.voltage_level_kv;
      ws.cell(row, 7).value() = p.p_mw;
      ws.cell(row, 8).value() = p.q_mvar;
      ws.cell(row, 9).value() = p.v_pu;
      ws.cell(row, 10).value() = p.pmax_mw;
      ws.cell(row, 11).value() = p.pmin_mw;
      ws.cell(row, 12).value() = p.qmax_mvar;
      ws.cell(row, 13).value() = p.qmin_mvar;
      ws.cell(row, 14).value() = er_control_mode_str(p.control_mode);
      ws.cell(row, 15).value() = p.p_set_mw;
      ws.cell(row, 16).value() = p.q_set_mvar;
      ws.cell(row, 17).value() = p.v_set_pu;
      ws.cell(row, 18).value() = bool_str(p.in_service);
      ++row;
    }
  }
}

void read_energy_router_ports(const XLWorksheet& ws, std::vector<EnergyRouter>& routers) {
  const auto col = make_colmap(ws);
  std::unordered_map<int, size_t> router_pos;
  router_pos.reserve(routers.size());
  for (size_t i = 0; i < routers.size(); ++i) {
    router_pos.emplace(routers[i].index, i);
  }

  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string router_idx = cell_by_name(ws, r, col, "router_index");
    const std::string port_idx = cell_by_name(ws, r, col, "index");
    if (router_idx.empty() && port_idx.empty()) break;
    if (router_idx.empty() || port_idx.empty()) continue;

    const int router_index = int_from_str(router_idx, 0);
    auto it = router_pos.find(router_index);
    if (it == router_pos.end()) {
      EnergyRouter er;
      er.index = router_index;
      routers.push_back(std::move(er));
      it = router_pos.emplace(router_index, routers.size() - 1).first;
    }

    EnergyRouterPort p;
    p.index = int_from_str(port_idx, p.index);
    p.name = cell_by_name(ws, r, col, "name");
    p.bus = int_from_str(cell_by_name(ws, r, col, "bus"), p.bus);
    p.port_type = er_port_type_from_str(cell_by_name(ws, r, col, "port_type"));
    p.voltage_level_kv = dbl_from_str(cell_by_name(ws, r, col, "voltage_level_kv"), p.voltage_level_kv);
    p.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), p.p_mw);
    p.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), p.q_mvar);
    p.v_pu = dbl_from_str(cell_by_name(ws, r, col, "v_pu"), p.v_pu);
    p.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), p.pmax_mw);
    p.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), p.pmin_mw);
    p.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmax_mvar"), p.qmax_mvar);
    p.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "qmin_mvar"), p.qmin_mvar);
    p.control_mode = er_control_mode_from_str(cell_by_name(ws, r, col, "control_mode"));
    p.p_set_mw = dbl_from_str(cell_by_name(ws, r, col, "p_set_mw"), p.p_set_mw);
    p.q_set_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_set_mvar"), p.q_set_mvar);
    p.v_set_pu = dbl_from_str(cell_by_name(ws, r, col, "v_set_pu"), p.v_set_pu);
    p.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), p.in_service);

    routers[it->second].ports.push_back(std::move(p));
  }

  for (auto& er : routers) {
    er.num_ports = static_cast<int>(er.ports.size());
  }
}

void write_vpp(XLWorksheet ws, const std::vector<VirtualPowerPlant>& vpps) {
  write_headers(ws, {"index", "name", "pcc_bus", "p_output_mw", "q_output_mvar", "pmax_mw", "pmin_mw", "n_pv_systems", "n_wind_turbines",
                     "n_battery_systems", "in_service"});
  for (size_t i = 0; i < vpps.size(); ++i) {
    const auto& v = vpps[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = v.index;
    ws.cell(r, 2).value() = v.name;
    ws.cell(r, 3).value() = v.pcc_bus;
    ws.cell(r, 4).value() = v.p_output_mw;
    ws.cell(r, 5).value() = v.q_output_mvar;
    ws.cell(r, 6).value() = v.pmax_mw;
    ws.cell(r, 7).value() = v.pmin_mw;
    ws.cell(r, 8).value() = v.n_pv_systems;
    ws.cell(r, 9).value() = v.n_wind_turbines;
    ws.cell(r, 10).value() = v.n_battery_systems;
    ws.cell(r, 11).value() = bool_str(v.in_service);
  }
}

std::vector<VirtualPowerPlant> read_vpp(const XLWorksheet& ws) {
  std::vector<VirtualPowerPlant> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    VirtualPowerPlant v;
    v.index = int_from_str(idx, v.index);
    v.name = cell_by_name(ws, r, col, "name");
    v.pcc_bus = int_from_str(cell_by_name(ws, r, col, "pcc_bus"), v.pcc_bus);
    v.p_output_mw = dbl_from_str(cell_by_name(ws, r, col, "p_output_mw"), v.p_output_mw);
    v.q_output_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_output_mvar"), v.q_output_mvar);
    v.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "pmax_mw"), v.pmax_mw);
    v.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "pmin_mw"), v.pmin_mw);
    v.n_pv_systems = int_from_str(cell_by_name(ws, r, col, "n_pv_systems"), v.n_pv_systems);
    v.n_wind_turbines = int_from_str(cell_by_name(ws, r, col, "n_wind_turbines"), v.n_wind_turbines);
    v.n_battery_systems = int_from_str(cell_by_name(ws, r, col, "n_battery_systems"), v.n_battery_systems);
    v.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), v.in_service);
    out.push_back(std::move(v));
  }
  return out;
}

void write_microgrids(XLWorksheet ws, const std::vector<Microgrid>& mgs) {
  write_headers(ws, {"index", "name", "pcc_bus", "operating_mode", "islanding_capability", "p_exchange_mw", "p_exchange_max_mw", "total_generation_mw",
                     "total_load_mw", "in_service"});
  for (size_t i = 0; i < mgs.size(); ++i) {
    const auto& m = mgs[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = m.index;
    ws.cell(r, 2).value() = m.name;
    ws.cell(r, 3).value() = m.pcc_bus;
    ws.cell(r, 4).value() = microgrid_mode_str(m.operating_mode);
    ws.cell(r, 5).value() = bool_str(m.islanding_capability);
    ws.cell(r, 6).value() = m.p_exchange_mw;
    ws.cell(r, 7).value() = m.p_exchange_max_mw;
    ws.cell(r, 8).value() = m.total_generation_mw;
    ws.cell(r, 9).value() = m.total_load_mw;
    ws.cell(r, 10).value() = bool_str(m.in_service);
  }
}

std::vector<Microgrid> read_microgrids(const XLWorksheet& ws) {
  std::vector<Microgrid> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    Microgrid m;
    m.index = int_from_str(idx, m.index);
    m.name = cell_by_name(ws, r, col, "name");
    m.pcc_bus = int_from_str(cell_by_name(ws, r, col, "pcc_bus"), m.pcc_bus);
    m.operating_mode = microgrid_mode_from_str(cell_by_name(ws, r, col, "operating_mode"));
    m.islanding_capability = bool_from_str(cell_by_name(ws, r, col, "islanding_capability"), m.islanding_capability);
    m.p_exchange_mw = dbl_from_str(cell_by_name(ws, r, col, "p_exchange_mw"), m.p_exchange_mw);
    m.p_exchange_max_mw = dbl_from_str(cell_by_name(ws, r, col, "p_exchange_max_mw"), m.p_exchange_max_mw);
    m.total_generation_mw = dbl_from_str(cell_by_name(ws, r, col, "total_generation_mw"), m.total_generation_mw);
    m.total_load_mw = dbl_from_str(cell_by_name(ws, r, col, "total_load_mw"), m.total_load_mw);
    m.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), m.in_service);
    out.push_back(std::move(m));
  }
  return out;
}

void write_mobile_storage(XLWorksheet ws, const std::vector<MobileStorage>& units) {
  write_headers(ws, {"index", "name", "bus", "status", "p_mw", "q_mvar", "p_rated_mw", "e_rated_mwh", "soc_init", "in_service"});
  for (size_t i = 0; i < units.size(); ++i) {
    const auto& s = units[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = s.index;
    ws.cell(r, 2).value() = s.name;
    ws.cell(r, 3).value() = s.bus;
    ws.cell(r, 4).value() = mobile_storage_status_str(s.status);
    ws.cell(r, 5).value() = s.p_mw;
    ws.cell(r, 6).value() = s.q_mvar;
    ws.cell(r, 7).value() = s.p_rated_mw;
    ws.cell(r, 8).value() = s.e_rated_mwh;
    ws.cell(r, 9).value() = s.soc_init;
    ws.cell(r, 10).value() = bool_str(s.in_service);
  }
}

std::vector<MobileStorage> read_mobile_storage(const XLWorksheet& ws) {
  std::vector<MobileStorage> out;
  const auto col = make_colmap(ws);
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string idx = cell_by_name(ws, r, col, "index");
    if (idx.empty()) break;
    MobileStorage s;
    s.index = int_from_str(idx, s.index);
    s.name = cell_by_name(ws, r, col, "name");
    s.bus = int_from_str(cell_by_name(ws, r, col, "bus"), s.bus);
    s.status = mobile_storage_status_from_str(cell_by_name(ws, r, col, "status"));
    s.p_mw = dbl_from_str(cell_by_name(ws, r, col, "p_mw"), s.p_mw);
    s.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "q_mvar"), s.q_mvar);
    s.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "p_rated_mw"), s.p_rated_mw);
    s.e_rated_mwh = dbl_from_str(cell_by_name(ws, r, col, "e_rated_mwh"), s.e_rated_mwh);
    s.soc_init = dbl_from_str(cell_by_name(ws, r, col, "soc_init"), s.soc_init);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "in_service"), s.in_service);
    out.push_back(std::move(s));
  }
  return out;
}

void write_results(const HybridPowerSystem& sys,
                   const PowerFlowResult& result,
                   XLWorkbook& wb) {
  std::unordered_map<int, size_t> ac_bus_pos;
  ac_bus_pos.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    ac_bus_pos[sys.ac.buses[i].index] = i;
  }

  std::unordered_map<int, size_t> dc_bus_pos;
  dc_bus_pos.reserve(sys.dc.buses.size());
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    dc_bus_pos[sys.dc.buses[i].index] = i;
  }

  auto ac_vm_by_index = [&](int bus_index, double fallback) {
    auto it = ac_bus_pos.find(bus_index);
    if (it == ac_bus_pos.end()) return fallback;
    const size_t pos = it->second;
    if (pos >= result.vm.size()) return fallback;
    return result.vm[pos];
  };

  auto ac_va_by_index = [&](int bus_index, double fallback) {
    auto it = ac_bus_pos.find(bus_index);
    if (it == ac_bus_pos.end()) return fallback;
    const size_t pos = it->second;
    if (pos >= result.va.size()) return fallback;
    return result.va[pos];
  };

  auto dc_v_by_index = [&](int bus_index, double fallback) {
    auto it = dc_bus_pos.find(bus_index);
    if (it == dc_bus_pos.end()) return fallback;
    const size_t pos = it->second;
    if (pos >= result.vdc.size()) return fallback;
    return result.vdc[pos];
  };

  std::unordered_map<int, VSCTransfer> vsc_by_index;
  vsc_by_index.reserve(result.vsc_transfers.size());
  for (const auto& tr : result.vsc_transfers) {
    vsc_by_index.emplace(tr.index, tr);
  }

  std::unordered_map<int, DCDCTransfer> dcdc_by_index;
  dcdc_by_index.reserve(result.dcdc_transfers.size());
  for (const auto& tr : result.dcdc_transfers) {
    dcdc_by_index.emplace(tr.index, tr);
  }

  std::unordered_map<long long, ERPortTransfer> er_port_by_key;
  er_port_by_key.reserve(result.er_port_transfers.size());
  for (const auto& tr : result.er_port_transfers) {
    const long long key = (static_cast<long long>(tr.router_index) << 32) |
                          static_cast<unsigned int>(tr.port_index);
    er_port_by_key.emplace(key, tr);
  }

  std::unordered_map<int, int> station_bus_by_id;
  station_bus_by_id.reserve(sys.ac.charging_stations.size());
  for (const auto& st : sys.ac.charging_stations) {
    station_bus_by_id.emplace(st.index, st.bus);
  }

  {
    auto ws = ensure_sheet(wb, "Results_ACBus");
    write_headers(ws, {"index", "name", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& b = sys.ac.buses[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double vm = (i < result.vm.size()) ? result.vm[i] : 0.0;
      const double va = (i < result.va.size()) ? result.va[i] : 0.0;
      ws.cell(r, 1).value() = b.index;
      ws.cell(r, 2).value() = b.name;
      ws.cell(r, 3).value() = vm;
      ws.cell(r, 4).value() = va;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_DCBus");
    write_headers(ws, {"index", "name", "vm_pu"});
    for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const auto& b = sys.dc.buses[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double v = (i < result.vdc.size()) ? result.vdc[i] : 0.0;
      ws.cell(r, 1).value() = b.index;
      ws.cell(r, 2).value() = b.name;
      ws.cell(r, 3).value() = v;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_BranchFlow");
    write_headers(ws, {"index", "from_bus", "to_bus", "pf_mw", "qf_mvar", "pt_mw", "qt_mvar", "loss_mw", "loading_pct"});
    for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
      const auto& br = sys.ac.branches[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      double pf_mw = 0.0;
      double qf_mvar = 0.0;
      double pt_mw = 0.0;
      double qt_mvar = 0.0;
      if (i < result.branch_flows.size()) {
        pf_mw = result.branch_flows[i].pf_mw;
        qf_mvar = result.branch_flows[i].qf_mvar;
        pt_mw = result.branch_flows[i].pt_mw;
        qt_mvar = result.branch_flows[i].qt_mvar;
      }
      const double sf = std::sqrt(pf_mw * pf_mw + qf_mvar * qf_mvar);
      const double st = std::sqrt(pt_mw * pt_mw + qt_mvar * qt_mvar);
      const double rating_mva = (br.rate_a_mva > 1e-12)
                                    ? br.rate_a_mva
                                    : ((sys.base_mva > 1e-12)
                                           ? sys.base_mva
                                           : ((sys.ac.base_mva > 1e-12)
                                                  ? sys.ac.base_mva
                                                  : 100.0));
      const double loading_pct =
          (rating_mva > 1e-12) ? (100.0 * std::max(sf, st) / rating_mva) : 0.0;
      ws.cell(r, 1).value() = br.index;
      ws.cell(r, 2).value() = br.from_bus;
      ws.cell(r, 3).value() = br.to_bus;
      ws.cell(r, 4).value() = pf_mw;
      ws.cell(r, 5).value() = qf_mvar;
      ws.cell(r, 6).value() = pt_mw;
      ws.cell(r, 7).value() = qt_mvar;
      ws.cell(r, 8).value() = std::max(pf_mw + pt_mw, 0.0);
      ws.cell(r, 9).value() = loading_pct;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_DCBranch");
    write_headers(ws, {"index", "name", "from_bus", "to_bus", "in_service", "v_from_pu", "v_to_pu", "i_pu", "p_from_mw", "p_to_mw", "loss_mw", "loading_pct"});
    const double base_mva = (sys.dc.base_mva > 0.0)
                                ? sys.dc.base_mva
                                : ((sys.base_mva > 0.0) ? sys.base_mva : 100.0);
    for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
      const auto& br = sys.dc.branches[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double v_from = dc_v_by_index(br.from_bus, 1.0);
      const double v_to = dc_v_by_index(br.to_bus, 1.0);

      double i_pu = 0.0;
      double p_from_mw = 0.0;
      double p_to_mw = 0.0;
      if (br.in_service && br.r_pu > 0.0) {
        i_pu = (v_from - v_to) / br.r_pu;
        p_from_mw = base_mva * v_from * i_pu;
        p_to_mw = -base_mva * v_to * i_pu;
      }
      const double loss_mw = p_from_mw + p_to_mw;
      const double rating_mva = (br.s_max_mva > 1e-12)
                                    ? br.s_max_mva
                                    : ((br.rate_a_mva > 1e-12) ? br.rate_a_mva : base_mva);
      const double loading_pct =
          (rating_mva > 1e-12)
              ? 100.0 * std::max(std::abs(p_from_mw), std::abs(p_to_mw)) / rating_mva
              : 0.0;

      ws.cell(r, 1).value() = br.index;
      ws.cell(r, 2).value() = br.name;
      ws.cell(r, 3).value() = br.from_bus;
      ws.cell(r, 4).value() = br.to_bus;
      ws.cell(r, 5).value() = bool_str(br.in_service);
      ws.cell(r, 6).value() = v_from;
      ws.cell(r, 7).value() = v_to;
      ws.cell(r, 8).value() = i_pu;
      ws.cell(r, 9).value() = p_from_mw;
      ws.cell(r, 10).value() = p_to_mw;
      ws.cell(r, 11).value() = loss_mw;
      ws.cell(r, 12).value() = loading_pct;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_VSCConverter");
    write_headers(ws, {"index", "name", "bus_ac", "bus_dc", "in_service", "control_mode", "p_ac_mw", "q_ac_mvar", "p_dc_mw", "loss_mw", "p_set_mw", "q_set_mvar", "v_ac_pu", "v_dc_pu", "eta", "loss_percent"});
    for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
      const auto& c = sys.vsc_converters[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double v_ac = ac_vm_by_index(c.bus_ac, 1.0);
      const double v_dc = dc_v_by_index(c.bus_dc, 1.0);
      const auto tr_it = vsc_by_index.find(c.index);
      const double p_ac_mw = (tr_it != vsc_by_index.end()) ? tr_it->second.p_ac_mw : c.p_set_mw;
      const double q_ac_mvar = (tr_it != vsc_by_index.end()) ? tr_it->second.q_ac_mvar : c.q_set_mvar;
      const double p_dc_mw = (tr_it != vsc_by_index.end()) ? tr_it->second.p_dc_mw : -(p_ac_mw + c.loss_mw);
      const double loss_mw = (tr_it != vsc_by_index.end()) ? tr_it->second.loss_mw : c.loss_mw;

      ws.cell(r, 1).value() = c.index;
      ws.cell(r, 2).value() = c.name;
      ws.cell(r, 3).value() = c.bus_ac;
      ws.cell(r, 4).value() = c.bus_dc;
      ws.cell(r, 5).value() = bool_str(c.in_service);
      ws.cell(r, 6).value() = converter_mode_str(c.control_mode);
      ws.cell(r, 7).value() = p_ac_mw;
      ws.cell(r, 8).value() = q_ac_mvar;
      ws.cell(r, 9).value() = p_dc_mw;
      ws.cell(r, 10).value() = loss_mw;
      ws.cell(r, 11).value() = c.p_set_mw;
      ws.cell(r, 12).value() = c.q_set_mvar;
      ws.cell(r, 13).value() = v_ac;
      ws.cell(r, 14).value() = v_dc;
      ws.cell(r, 15).value() = c.eta;
      ws.cell(r, 16).value() = c.loss_percent;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_DCDCConverter");
    write_headers(ws, {"index", "name", "bus_in", "bus_out", "in_service", "control_mode", "p_ref_mw", "v_ref_pu", "v_in_pu", "v_out_pu", "eta", "p_in_mw", "p_out_mw", "loss_mw"});
    for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
      const auto& c = sys.dc.dcdc_converters[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double v_in = dc_v_by_index(c.bus_in, 1.0);
      const double v_out = dc_v_by_index(c.bus_out, 1.0);
      const auto tr_it = dcdc_by_index.find(c.index);
      const double p_in = (tr_it != dcdc_by_index.end()) ? tr_it->second.p_in_mw : c.p_ref_mw;
      const double p_out = (tr_it != dcdc_by_index.end()) ? tr_it->second.p_out_mw : c.p_ref_mw;
      const double loss = (tr_it != dcdc_by_index.end()) ? tr_it->second.loss_mw : (p_in - p_out);

      ws.cell(r, 1).value() = c.index;
      ws.cell(r, 2).value() = c.name;
      ws.cell(r, 3).value() = c.bus_in;
      ws.cell(r, 4).value() = c.bus_out;
      ws.cell(r, 5).value() = bool_str(c.in_service);
      ws.cell(r, 6).value() = dcdc_control_mode_str(c.control_mode);
      ws.cell(r, 7).value() = c.p_ref_mw;
      ws.cell(r, 8).value() = c.v_ref_pu;
      ws.cell(r, 9).value() = v_in;
      ws.cell(r, 10).value() = v_out;
      ws.cell(r, 11).value() = c.eta;
      ws.cell(r, 12).value() = p_in;
      ws.cell(r, 13).value() = p_out;
      ws.cell(r, 14).value() = loss;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Storage_AC");
    write_headers(ws, {"index", "name", "bus", "in_service", "p_mw", "q_mvar", "soc", "soh", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
      const auto& s = sys.ac.storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = s.index;
      ws.cell(r, 2).value() = s.name;
      ws.cell(r, 3).value() = s.bus;
      ws.cell(r, 4).value() = bool_str(s.in_service);
      ws.cell(r, 5).value() = s.p_mw;
      ws.cell(r, 6).value() = s.q_mvar;
      ws.cell(r, 7).value() = s.soc_init;
      ws.cell(r, 8).value() = s.soh;
      ws.cell(r, 9).value() = ac_vm_by_index(s.bus, 1.0);
      ws.cell(r, 10).value() = ac_va_by_index(s.bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Storage_DC");
    write_headers(ws, {"index", "name", "bus", "in_service", "p_mw", "q_mvar", "soc", "soh", "vdc_pu"});
    for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
      const auto& s = sys.dc.storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = s.index;
      ws.cell(r, 2).value() = s.name;
      ws.cell(r, 3).value() = s.bus;
      ws.cell(r, 4).value() = bool_str(s.in_service);
      ws.cell(r, 5).value() = s.p_mw;
      ws.cell(r, 6).value() = s.q_mvar;
      ws.cell(r, 7).value() = s.soc_init;
      ws.cell(r, 8).value() = s.soh;
      ws.cell(r, 9).value() = dc_v_by_index(s.bus, 1.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_ChargingStation");
    write_headers(ws, {"index", "name", "bus", "in_service", "num_chargers", "p_total_kw", "q_total_kvar", "max_power_kw", "utilization_rate", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.ac.charging_stations.size(); ++i) {
      const auto& st = sys.ac.charging_stations[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = st.index;
      ws.cell(r, 2).value() = st.name;
      ws.cell(r, 3).value() = st.bus;
      ws.cell(r, 4).value() = bool_str(st.in_service);
      ws.cell(r, 5).value() = st.num_chargers;
      ws.cell(r, 6).value() = st.p_total_kw;
      ws.cell(r, 7).value() = st.q_total_kvar;
      ws.cell(r, 8).value() = st.max_power_kw;
      ws.cell(r, 9).value() = st.utilization_rate;
      ws.cell(r, 10).value() = ac_vm_by_index(st.bus, 1.0);
      ws.cell(r, 11).value() = ac_va_by_index(st.bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Charger");
    write_headers(ws, {"index", "name", "station_id", "station_bus", "in_service", "charger_type", "p_rated_kw", "p_ch_max_kw", "p_dis_max_kw", "eta", "v2g_capable", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.ac.chargers.size(); ++i) {
      const auto& ch = sys.ac.chargers[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const auto st_it = station_bus_by_id.find(ch.station_id);
      const int station_bus = (st_it != station_bus_by_id.end()) ? st_it->second : 0;
      ws.cell(r, 1).value() = ch.index;
      ws.cell(r, 2).value() = ch.name;
      ws.cell(r, 3).value() = ch.station_id;
      ws.cell(r, 4).value() = station_bus;
      ws.cell(r, 5).value() = bool_str(ch.in_service);
      ws.cell(r, 6).value() = charger_type_str(ch.charger_type);
      ws.cell(r, 7).value() = ch.p_rated_kw;
      ws.cell(r, 8).value() = ch.p_ch_max_kw;
      ws.cell(r, 9).value() = ch.p_dis_max_kw;
      ws.cell(r, 10).value() = ch.eta;
      ws.cell(r, 11).value() = bool_str(ch.v2g_capable);
      ws.cell(r, 12).value() = ac_vm_by_index(station_bus, 1.0);
      ws.cell(r, 13).value() = ac_va_by_index(station_bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_EnergyRouter");
    write_headers(ws, {"index", "name", "in_service", "router_type", "num_ports", "p_rated_mw", "loss_percent", "p_port_sum_mw", "q_port_sum_mvar"});
    for (size_t i = 0; i < sys.energy_routers.size(); ++i) {
      const auto& er = sys.energy_routers[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      double p_sum = 0.0;
      double q_sum = 0.0;
      for (const auto& p : er.ports) {
        p_sum += p.p_mw;
        q_sum += p.q_mvar;
      }
      ws.cell(r, 1).value() = er.index;
      ws.cell(r, 2).value() = er.name;
      ws.cell(r, 3).value() = bool_str(er.in_service);
      ws.cell(r, 4).value() = er.router_type;
      ws.cell(r, 5).value() = er.num_ports;
      ws.cell(r, 6).value() = er.p_rated_mw;
      ws.cell(r, 7).value() = er.loss_percent;
      ws.cell(r, 8).value() = p_sum;
      ws.cell(r, 9).value() = q_sum;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_EnergyRouterPort");
    write_headers(ws, {"router_index", "port_index", "port_name", "bus", "port_type", "control_mode", "in_service", "p_mw", "q_mvar", "v_solved_pu"});
    uint32_t row = 2;
    for (const auto& er : sys.energy_routers) {
      for (const auto& p : er.ports) {
        const long long key = (static_cast<long long>(er.index) << 32) |
                              static_cast<unsigned int>(p.index);
        auto tr_it = er_port_by_key.find(key);
        const double v = (tr_it != er_port_by_key.end())
                             ? tr_it->second.v_pu
                             : ((p.port_type == ERPortType::AC)
                                    ? ac_vm_by_index(p.bus, 1.0)
                                    : dc_v_by_index(p.bus, 1.0));
        const double p_mw = (tr_it != er_port_by_key.end()) ? tr_it->second.p_mw : p.p_mw;
        const double q_mvar = (tr_it != er_port_by_key.end()) ? tr_it->second.q_mvar : p.q_mvar;
        ws.cell(row, 1).value() = er.index;
        ws.cell(row, 2).value() = p.index;
        ws.cell(row, 3).value() = p.name;
        ws.cell(row, 4).value() = p.bus;
        ws.cell(row, 5).value() = er_port_type_str(p.port_type);
        ws.cell(row, 6).value() = er_control_mode_str(p.control_mode);
        ws.cell(row, 7).value() = bool_str(p.in_service);
        ws.cell(row, 8).value() = p_mw;
        ws.cell(row, 9).value() = q_mvar;
        ws.cell(row, 10).value() = v;
        ++row;
      }
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_VPP");
    write_headers(ws, {"index", "name", "pcc_bus", "in_service", "p_output_mw", "q_output_mvar", "pmax_mw", "pmin_mw", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.vpps.size(); ++i) {
      const auto& vpp = sys.vpps[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = vpp.index;
      ws.cell(r, 2).value() = vpp.name;
      ws.cell(r, 3).value() = vpp.pcc_bus;
      ws.cell(r, 4).value() = bool_str(vpp.in_service);
      ws.cell(r, 5).value() = vpp.p_output_mw;
      ws.cell(r, 6).value() = vpp.q_output_mvar;
      ws.cell(r, 7).value() = vpp.pmax_mw;
      ws.cell(r, 8).value() = vpp.pmin_mw;
      ws.cell(r, 9).value() = ac_vm_by_index(vpp.pcc_bus, 1.0);
      ws.cell(r, 10).value() = ac_va_by_index(vpp.pcc_bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Microgrid");
    write_headers(ws, {"index", "name", "pcc_bus", "operating_mode", "in_service", "p_exchange_mw", "total_generation_mw", "total_load_mw", "vm_pu", "va_deg", "internal_buses"});
    for (size_t i = 0; i < sys.microgrids.size(); ++i) {
      const auto& mg = sys.microgrids[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      std::string buses;
      for (size_t k = 0; k < mg.internal_buses.size(); ++k) {
        if (k > 0) buses += ",";
        buses += std::to_string(mg.internal_buses[k]);
      }
      ws.cell(r, 1).value() = mg.index;
      ws.cell(r, 2).value() = mg.name;
      ws.cell(r, 3).value() = mg.pcc_bus;
      ws.cell(r, 4).value() = microgrid_mode_str(mg.operating_mode);
      ws.cell(r, 5).value() = bool_str(mg.in_service);
      ws.cell(r, 6).value() = mg.p_exchange_mw;
      ws.cell(r, 7).value() = mg.total_generation_mw;
      ws.cell(r, 8).value() = mg.total_load_mw;
      ws.cell(r, 9).value() = ac_vm_by_index(mg.pcc_bus, 1.0);
      ws.cell(r, 10).value() = ac_va_by_index(mg.pcc_bus, 0.0);
      ws.cell(r, 11).value() = buses;
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_MobileStorage");
    write_headers(ws, {"index", "name", "bus", "status", "in_service", "p_mw", "q_mvar", "soc", "target_bus", "vm_pu", "va_deg"});
    for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
      const auto& ms = sys.mobile_storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = ms.index;
      ws.cell(r, 2).value() = ms.name;
      ws.cell(r, 3).value() = ms.bus;
      ws.cell(r, 4).value() = mobile_storage_status_str(ms.status);
      ws.cell(r, 5).value() = bool_str(ms.in_service);
      ws.cell(r, 6).value() = ms.p_mw;
      ws.cell(r, 7).value() = ms.q_mvar;
      ws.cell(r, 8).value() = ms.soc_init;
      ws.cell(r, 9).value() = ms.target_bus;
      ws.cell(r, 10).value() = ac_vm_by_index(ms.bus, 1.0);
      ws.cell(r, 11).value() = ac_va_by_index(ms.bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Summary");
    write_headers(ws, {"Key", "Value"});
    ws.cell(2, 1).value() = "converged";
    ws.cell(2, 2).value() = bool_str(result.converged);
    ws.cell(3, 1).value() = "iterations";
    ws.cell(3, 2).value() = result.iterations;
    ws.cell(4, 1).value() = "residual";
    ws.cell(4, 2).value() = result.residual;
  }
}

void write_results_compact(const HybridPowerSystem& sys,
                          const PowerFlowResult& result,
                          XLWorkbook& wb) {
  std::unordered_map<int, size_t> ac_bus_pos;
  ac_bus_pos.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) ac_bus_pos[sys.ac.buses[i].index] = i;

  std::unordered_map<int, size_t> dc_bus_pos;
  dc_bus_pos.reserve(sys.dc.buses.size());
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) dc_bus_pos[sys.dc.buses[i].index] = i;

  auto ac_vm_by_index = [&](int bus_index, double fallback) {
    auto it = ac_bus_pos.find(bus_index);
    if (it == ac_bus_pos.end() || it->second >= result.vm.size()) return fallback;
    return result.vm[it->second];
  };

  auto ac_va_by_index = [&](int bus_index, double fallback) {
    auto it = ac_bus_pos.find(bus_index);
    if (it == ac_bus_pos.end() || it->second >= result.va.size()) return fallback;
    return result.va[it->second];
  };

  auto dc_v_by_index = [&](int bus_index, double fallback) {
    auto it = dc_bus_pos.find(bus_index);
    if (it == dc_bus_pos.end() || it->second >= result.vdc.size()) return fallback;
    return result.vdc[it->second];
  };

  std::unordered_map<int, VSCTransfer> vsc_by_index;
  for (const auto& tr : result.vsc_transfers) vsc_by_index.emplace(tr.index, tr);

  std::unordered_map<int, DCDCTransfer> dcdc_by_index;
  for (const auto& tr : result.dcdc_transfers) dcdc_by_index.emplace(tr.index, tr);

  if (wb.worksheetExists("ACBus")) {
    auto ws = wb.worksheet("ACBus");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = (i < result.vm.size()) ? result.vm[i] : 0.0;
      ws.cell(r, c_va).value() = (i < result.va.size()) ? result.va[i] : 0.0;
    }
  }

  if (wb.worksheetExists("DCBus")) {
    auto ws = wb.worksheet("DCBus");
    const uint16_t c_v = ensure_header_column(ws, "result_vdc_pu");
    for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_v).value() = (i < result.vdc.size()) ? result.vdc[i] : 0.0;
    }
  }

  if (wb.worksheetExists("ACBranch")) {
    auto ws = wb.worksheet("ACBranch");
    const uint16_t c_pf = ensure_header_column(ws, "result_pf_mw");
    const uint16_t c_qf = ensure_header_column(ws, "result_qf_mvar");
    const uint16_t c_pt = ensure_header_column(ws, "result_pt_mw");
    const uint16_t c_qt = ensure_header_column(ws, "result_qt_mvar");
    for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const BranchFlow bf = (i < result.branch_flows.size()) ? result.branch_flows[i] : BranchFlow{};
      ws.cell(r, c_pf).value() = bf.pf_mw;
      ws.cell(r, c_qf).value() = bf.qf_mvar;
      ws.cell(r, c_pt).value() = bf.pt_mw;
      ws.cell(r, c_qt).value() = bf.qt_mvar;
    }
  }

  if (wb.worksheetExists("DCBranch")) {
    auto ws = wb.worksheet("DCBranch");
    const uint16_t c_vf = ensure_header_column(ws, "result_v_from_pu");
    const uint16_t c_vt = ensure_header_column(ws, "result_v_to_pu");
    const uint16_t c_i = ensure_header_column(ws, "result_i_pu");
    const uint16_t c_pf = ensure_header_column(ws, "result_p_from_mw");
    const uint16_t c_pt = ensure_header_column(ws, "result_p_to_mw");
    const uint16_t c_loss = ensure_header_column(ws, "result_loss_mw");
    const double base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;
    for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
      const auto& br = sys.dc.branches[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      const double v_from = dc_v_by_index(br.from_bus, 1.0);
      const double v_to = dc_v_by_index(br.to_bus, 1.0);
      double i_pu = 0.0;
      double p_from_mw = 0.0;
      double p_to_mw = 0.0;
      if (br.in_service && br.r_pu > 0.0) {
        i_pu = (v_from - v_to) / br.r_pu;
        p_from_mw = base_mva * v_from * i_pu;
        p_to_mw = -base_mva * v_to * i_pu;
      }
      ws.cell(r, c_vf).value() = v_from;
      ws.cell(r, c_vt).value() = v_to;
      ws.cell(r, c_i).value() = i_pu;
      ws.cell(r, c_pf).value() = p_from_mw;
      ws.cell(r, c_pt).value() = p_to_mw;
      ws.cell(r, c_loss).value() = p_from_mw + p_to_mw;
    }
  }

  if (wb.worksheetExists("Storage")) {
    auto ws = wb.worksheet("Storage");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
      const auto& s = sys.ac.storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = ac_vm_by_index(s.bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(s.bus, 0.0);
    }
  }

  if (wb.worksheetExists("DCStorage")) {
    auto ws = wb.worksheet("DCStorage");
    const uint16_t c_v = ensure_header_column(ws, "result_vdc_pu");
    for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
      const auto& s = sys.dc.storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_v).value() = dc_v_by_index(s.bus, 1.0);
    }
  }

  if (wb.worksheetExists("DCLoad")) {
    auto ws = wb.worksheet("DCLoad");
    const uint16_t c_v = ensure_header_column(ws, "result_vdc_pu");
    const auto col = make_colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string idx = cell_by_name(ws, r, col, "index");
      if (idx.empty()) break;
      const int bus = int_from_str(cell_by_name(ws, r, col, "bus"), 0);
      ws.cell(r, c_v).value() = dc_v_by_index(bus, 1.0);
    }
  }

  if (wb.worksheetExists("Load")) {
    auto ws = wb.worksheet("Load");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    const auto col = make_colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string idx = cell_by_name(ws, r, col, "index");
      if (idx.empty()) break;
      const int bus = int_from_str(cell_by_name(ws, r, col, "bus"), 0);
      ws.cell(r, c_vm).value() = ac_vm_by_index(bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(bus, 0.0);
    }
  }

  if (wb.worksheetExists("ACLoad")) {
    auto ws = wb.worksheet("ACLoad");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    const auto col = make_colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string idx = cell_by_name(ws, r, col, "index");
      if (idx.empty()) break;
      const int bus = int_from_str(cell_by_name(ws, r, col, "bus"), 0);
      ws.cell(r, c_vm).value() = ac_vm_by_index(bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(bus, 0.0);
    }
  }

  if (wb.worksheetExists("ChargingStation")) {
    auto ws = wb.worksheet("ChargingStation");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.ac.charging_stations.size(); ++i) {
      const auto& st = sys.ac.charging_stations[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = ac_vm_by_index(st.bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(st.bus, 0.0);
    }
  }

  if (wb.worksheetExists("VSCConverter")) {
    auto ws = wb.worksheet("VSCConverter");
    const uint16_t c_pac = ensure_header_column(ws, "result_p_ac_mw");
    const uint16_t c_qac = ensure_header_column(ws, "result_q_ac_mvar");
    const uint16_t c_pdc = ensure_header_column(ws, "result_p_dc_mw");
    const uint16_t c_loss = ensure_header_column(ws, "result_loss_mw");
    const uint16_t c_vac = ensure_header_column(ws, "result_v_ac_pu");
    const uint16_t c_vdc = ensure_header_column(ws, "result_v_dc_pu");
    for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
      const auto& c = sys.vsc_converters[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      auto it = vsc_by_index.find(c.index);
      const VSCTransfer tr = (it != vsc_by_index.end()) ? it->second : VSCTransfer{};
      ws.cell(r, c_pac).value() = tr.p_ac_mw;
      ws.cell(r, c_qac).value() = tr.q_ac_mvar;
      ws.cell(r, c_pdc).value() = tr.p_dc_mw;
      ws.cell(r, c_loss).value() = tr.loss_mw;
      ws.cell(r, c_vac).value() = ac_vm_by_index(c.bus_ac, 1.0);
      ws.cell(r, c_vdc).value() = dc_v_by_index(c.bus_dc, 1.0);
    }
  }

  if (wb.worksheetExists("DCDCConverter")) {
    auto ws = wb.worksheet("DCDCConverter");
    const uint16_t c_pin = ensure_header_column(ws, "result_p_in_mw");
    const uint16_t c_pout = ensure_header_column(ws, "result_p_out_mw");
    const uint16_t c_loss = ensure_header_column(ws, "result_loss_mw");
    const uint16_t c_vin = ensure_header_column(ws, "result_v_in_pu");
    const uint16_t c_vout = ensure_header_column(ws, "result_v_out_pu");
    for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
      const auto& c = sys.dc.dcdc_converters[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      auto it = dcdc_by_index.find(c.index);
      const DCDCTransfer tr = (it != dcdc_by_index.end()) ? it->second : DCDCTransfer{};
      ws.cell(r, c_pin).value() = tr.p_in_mw;
      ws.cell(r, c_pout).value() = tr.p_out_mw;
      ws.cell(r, c_loss).value() = tr.loss_mw;
      ws.cell(r, c_vin).value() = dc_v_by_index(c.bus_in, 1.0);
      ws.cell(r, c_vout).value() = dc_v_by_index(c.bus_out, 1.0);
    }
  }

  if (wb.worksheetExists("VPP")) {
    auto ws = wb.worksheet("VPP");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.vpps.size(); ++i) {
      const auto& vpp = sys.vpps[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = ac_vm_by_index(vpp.pcc_bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(vpp.pcc_bus, 0.0);
    }
  }

  if (wb.worksheetExists("Microgrid")) {
    auto ws = wb.worksheet("Microgrid");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.microgrids.size(); ++i) {
      const auto& mg = sys.microgrids[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = ac_vm_by_index(mg.pcc_bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(mg.pcc_bus, 0.0);
    }
  }

  if (wb.worksheetExists("MobileStorage")) {
    auto ws = wb.worksheet("MobileStorage");
    const uint16_t c_vm = ensure_header_column(ws, "result_vm_pu");
    const uint16_t c_va = ensure_header_column(ws, "result_va_deg");
    for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
      const auto& ms = sys.mobile_storage[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, c_vm).value() = ac_vm_by_index(ms.bus, 1.0);
      ws.cell(r, c_va).value() = ac_va_by_index(ms.bus, 0.0);
    }
  }

  {
    auto ws = ensure_sheet(wb, "Results_Summary");
    write_headers(ws, {"Key", "Value"});
    ws.cell(2, 1).value() = "converged";
    ws.cell(2, 2).value() = bool_str(result.converged);
    ws.cell(3, 1).value() = "iterations";
    ws.cell(3, 2).value() = result.iterations;
    ws.cell(4, 1).value() = "residual";
    ws.cell(4, 2).value() = result.residual;
  }
}

}  // namespace

void save_xlsx(const HybridPowerSystem& sys, const std::string& path) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("Info");
  } else {
    ensure_sheet(wb, "Info");
  }

  const std::vector<std::string> needed = {
      "ACBus",          "ACBranch",         "Generator",      "Load",         "ACLoad",          "FlexibleLoad",   "AsymmetricLoad",
      "StaticGenerator", "Storage",         "RenewableGen",   "PVSystem",     "Shunt",           "ExternalGrid",   "Transformer2W",
      "Transformer3W",  "ChargingStation",  "Charger",        "AsynchronousMotor", "Switch",  "CircuitBreaker",  "DCBus",
      "DCLoad",         "DCLoadRaw",        "DCBranch",       "DCStorage",    "DCStaticGen",  "DCStaticGenerator", "DCPVArray", "DCCircuitBreaker",  "VSCConverter",  "DCDCConverter",
      "EnergyRouter",   "EnergyRouterPort", "VPP",            "Microgrid",
      "ThreePhaseBus",  "ThreePhaseLine",   "ThreePhaseTransformer", "ThreePhaseLoad", "ThreePhaseGenerator", "ThreePhaseExternalGrid",
      "MobileStorage",  "Results_ACBus",    "Results_DCBus",  "Results_BranchFlow", "Results_DCBranch", "Results_VSCConverter",
      "Results_DCDCConverter", "Results_Storage_AC", "Results_Storage_DC", "Results_ChargingStation", "Results_Charger",
      "Results_EnergyRouter", "Results_EnergyRouterPort", "Results_VPP", "Results_Microgrid", "Results_MobileStorage",
      "Results_Summary"};

  for (const auto& name : needed) ensure_sheet(wb, name);

  const auto ac_buses_view = buses_with_aggregated_ac_loads(sys);
  const auto dc_buses_view = buses_with_aggregated_dc_loads(sys);
  const auto ac_loads_view = make_aggregated_ac_load_table(sys);
  const auto dc_loads_view = make_aggregated_dc_load_table(sys);

  write_info(wb.worksheet("Info"), sys);
  write_ac_buses(wb.worksheet("ACBus"), ac_buses_view);
  write_ac_branches(wb.worksheet("ACBranch"), sys.ac.branches);
  write_generators(wb.worksheet("Generator"), sys.ac.generators);
  write_loads(wb.worksheet("Load"), sys.ac.loads);
  write_loads(wb.worksheet("ACLoad"), ac_loads_view);
  write_flexible_loads(wb.worksheet("FlexibleLoad"), sys.ac.flexible_loads);
  write_asymmetric_loads(wb.worksheet("AsymmetricLoad"), sys.ac.asymmetric_loads);
  write_static_gens(wb.worksheet("StaticGenerator"), sys.ac.static_generators);
  write_storage(wb.worksheet("Storage"), sys.ac.storage);
  write_renewable(wb.worksheet("RenewableGen"), sys.ac.renewable_gens);
  write_pv(wb.worksheet("PVSystem"), sys.ac.pv_systems);
  write_shunts(wb.worksheet("Shunt"), sys.ac.shunts);
  write_external_grids(wb.worksheet("ExternalGrid"), sys.ac.external_grids);
  write_transformers_2w(wb.worksheet("Transformer2W"), sys.ac.transformers_2w);
  write_transformers_3w(wb.worksheet("Transformer3W"), sys.ac.transformers_3w);
  write_charging_stations(wb.worksheet("ChargingStation"), sys.ac.charging_stations);
  write_chargers(wb.worksheet("Charger"), sys.ac.chargers);
  write_asynchronous_motors(wb.worksheet("AsynchronousMotor"), sys.ac.motors);
  write_switches(wb.worksheet("Switch"), sys.ac.switches);
  write_circuit_breakers(wb.worksheet("CircuitBreaker"), sys.ac.circuit_breakers);

  write_dc_buses(wb.worksheet("DCBus"), dc_buses_view);
  write_dc_loads(wb.worksheet("DCLoad"), dc_loads_view);
  write_dc_loads(wb.worksheet("DCLoadRaw"), sys.dc.loads);
  write_dc_branches(wb.worksheet("DCBranch"), sys.dc.branches);
  write_storage(wb.worksheet("DCStorage"), sys.dc.storage);
  write_static_gens(wb.worksheet("DCStaticGen"), sys.dc.static_generators);
  write_dc_static_generators(wb.worksheet("DCStaticGenerator"), sys.dc.dc_static_generators);
  write_dc_pv_arrays(wb.worksheet("DCPVArray"), sys.dc.pv_arrays);
  write_dc_circuit_breakers(wb.worksheet("DCCircuitBreaker"), sys.dc.dc_circuit_breakers);

  write_vsc(wb.worksheet("VSCConverter"), sys.vsc_converters);
  write_dcdc(wb.worksheet("DCDCConverter"), sys.dc.dcdc_converters);
  write_energy_routers(wb.worksheet("EnergyRouter"), sys.energy_routers);
  write_energy_router_ports(wb.worksheet("EnergyRouterPort"), sys.energy_routers);
  write_vpp(wb.worksheet("VPP"), sys.vpps);
  write_microgrids(wb.worksheet("Microgrid"), sys.microgrids);
  write_mobile_storage(wb.worksheet("MobileStorage"), sys.mobile_storage);
  if (sys.three_phase_ac.has_value()) {
    write_three_phase_buses(wb.worksheet("ThreePhaseBus"), sys.three_phase_ac->buses);
    write_three_phase_lines(wb.worksheet("ThreePhaseLine"), sys.three_phase_ac->lines);
    write_three_phase_transformers(wb.worksheet("ThreePhaseTransformer"), sys.three_phase_ac->transformers);
    write_three_phase_loads(wb.worksheet("ThreePhaseLoad"), sys.three_phase_ac->loads);
    write_three_phase_generators(wb.worksheet("ThreePhaseGenerator"), sys.three_phase_ac->generators);
    write_three_phase_external_grids(wb.worksheet("ThreePhaseExternalGrid"), sys.three_phase_ac->external_grids);
  }

  doc.save();
  doc.close();
}

HybridPowerSystem load_xlsx(const std::string& path) {
  XLDocument doc;
  doc.open(path);
  XLWorkbook wb = doc.workbook();

  HybridPowerSystem sys;

  if (wb.worksheetExists("Info")) load_info(wb.worksheet("Info"), sys);

  if (wb.worksheetExists("ACBus")) sys.ac.buses = read_ac_buses(wb.worksheet("ACBus"));
  if (wb.worksheetExists("ACBranch")) sys.ac.branches = read_ac_branches(wb.worksheet("ACBranch"));
  if (wb.worksheetExists("Generator")) sys.ac.generators = read_generators(wb.worksheet("Generator"));
  if (wb.worksheetExists("Load")) {
    sys.ac.loads = read_loads(wb.worksheet("Load"));
  } else if (wb.worksheetExists("ACLoad")) {
    sys.ac.loads = read_loads(wb.worksheet("ACLoad"));
  }
  if (wb.worksheetExists("FlexibleLoad")) sys.ac.flexible_loads = read_flexible_loads(wb.worksheet("FlexibleLoad"));
  if (wb.worksheetExists("AsymmetricLoad")) sys.ac.asymmetric_loads = read_asymmetric_loads(wb.worksheet("AsymmetricLoad"));
  if (wb.worksheetExists("StaticGenerator")) sys.ac.static_generators = read_static_gens(wb.worksheet("StaticGenerator"));
  if (wb.worksheetExists("Storage")) sys.ac.storage = read_storage(wb.worksheet("Storage"));
  if (wb.worksheetExists("RenewableGen")) sys.ac.renewable_gens = read_renewable(wb.worksheet("RenewableGen"));
  if (wb.worksheetExists("PVSystem")) sys.ac.pv_systems = read_pv(wb.worksheet("PVSystem"));
  if (wb.worksheetExists("Shunt")) sys.ac.shunts = read_shunts(wb.worksheet("Shunt"));
  if (wb.worksheetExists("ExternalGrid")) sys.ac.external_grids = read_external_grids(wb.worksheet("ExternalGrid"));
  if (wb.worksheetExists("Transformer2W")) sys.ac.transformers_2w = read_transformers_2w(wb.worksheet("Transformer2W"));
  if (wb.worksheetExists("Transformer3W")) sys.ac.transformers_3w = read_transformers_3w(wb.worksheet("Transformer3W"));
  if (wb.worksheetExists("ChargingStation")) sys.ac.charging_stations = read_charging_stations(wb.worksheet("ChargingStation"));
  if (wb.worksheetExists("Charger")) sys.ac.chargers = read_chargers(wb.worksheet("Charger"));
  if (wb.worksheetExists("AsynchronousMotor")) sys.ac.motors = read_asynchronous_motors(wb.worksheet("AsynchronousMotor"));
  if (wb.worksheetExists("Switch")) sys.ac.switches = read_switches(wb.worksheet("Switch"));
  if (wb.worksheetExists("CircuitBreaker")) sys.ac.circuit_breakers = read_circuit_breakers(wb.worksheet("CircuitBreaker"));

  if (wb.worksheetExists("DCBus")) sys.dc.buses = read_dc_buses(wb.worksheet("DCBus"));
  if (wb.worksheetExists("DCLoadRaw")) {
    sys.dc.loads = read_dc_loads(wb.worksheet("DCLoadRaw"));
  } else if (wb.worksheetExists("DCLoad")) {
    sys.dc.loads = read_dc_loads(wb.worksheet("DCLoad"));
  }
  if (wb.worksheetExists("DCBranch")) sys.dc.branches = read_dc_branches(wb.worksheet("DCBranch"));
  if (wb.worksheetExists("DCStorage")) sys.dc.storage = read_storage(wb.worksheet("DCStorage"));
  if (wb.worksheetExists("DCStaticGen")) sys.dc.static_generators = read_static_gens(wb.worksheet("DCStaticGen"));
  if (wb.worksheetExists("DCStaticGenerator")) {
    sys.dc.dc_static_generators = read_dc_static_generators(wb.worksheet("DCStaticGenerator"));
  } else if (wb.worksheetExists("DCStaticGeneratorDC")) {
    sys.dc.dc_static_generators = read_dc_static_generators(wb.worksheet("DCStaticGeneratorDC"));
  }
  if (wb.worksheetExists("DCPVArray")) sys.dc.pv_arrays = read_dc_pv_arrays(wb.worksheet("DCPVArray"));
  if (wb.worksheetExists("DCCircuitBreaker")) sys.dc.dc_circuit_breakers = read_dc_circuit_breakers(wb.worksheet("DCCircuitBreaker"));

  if (wb.worksheetExists("VSCConverter")) sys.vsc_converters = read_vsc(wb.worksheet("VSCConverter"));
  if (wb.worksheetExists("DCDCConverter")) sys.dc.dcdc_converters = read_dcdc(wb.worksheet("DCDCConverter"));
  if (wb.worksheetExists("EnergyRouter")) sys.energy_routers = read_energy_routers(wb.worksheet("EnergyRouter"));
  if (wb.worksheetExists("EnergyRouterPort")) read_energy_router_ports(wb.worksheet("EnergyRouterPort"), sys.energy_routers);
  if (wb.worksheetExists("VPP")) sys.vpps = read_vpp(wb.worksheet("VPP"));
  if (wb.worksheetExists("Microgrid")) sys.microgrids = read_microgrids(wb.worksheet("Microgrid"));
  if (wb.worksheetExists("MobileStorage")) sys.mobile_storage = read_mobile_storage(wb.worksheet("MobileStorage"));
  if (wb.worksheetExists("ThreePhaseBus") || wb.worksheetExists("ThreePhaseLine") ||
      wb.worksheetExists("ThreePhaseTransformer") || wb.worksheetExists("ThreePhaseLoad") ||
      wb.worksheetExists("ThreePhaseGenerator") || wb.worksheetExists("ThreePhaseExternalGrid")) {
    auto& tp = ensure_three_phase_system(sys);
    if (wb.worksheetExists("ThreePhaseBus")) tp.buses = read_three_phase_buses(wb.worksheet("ThreePhaseBus"));
    if (wb.worksheetExists("ThreePhaseLine")) tp.lines = read_three_phase_lines(wb.worksheet("ThreePhaseLine"));
    if (wb.worksheetExists("ThreePhaseTransformer")) tp.transformers = read_three_phase_transformers(wb.worksheet("ThreePhaseTransformer"));
    if (wb.worksheetExists("ThreePhaseLoad")) tp.loads = read_three_phase_loads(wb.worksheet("ThreePhaseLoad"));
    if (wb.worksheetExists("ThreePhaseGenerator")) tp.generators = read_three_phase_generators(wb.worksheet("ThreePhaseGenerator"));
    if (wb.worksheetExists("ThreePhaseExternalGrid")) tp.external_grids = read_three_phase_external_grids(wb.worksheet("ThreePhaseExternalGrid"));
  }

  sync_bus_demands_from_load_tables(sys);

  doc.close();
  return sys;
}

void save_results_xlsx(const HybridPowerSystem& sys,
                       const PowerFlowResult& result,
                       const std::string& path) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("Results_Summary");
  }

  ensure_sheet(wb, "Results_ACBus");
  ensure_sheet(wb, "Results_DCBus");
  ensure_sheet(wb, "Results_BranchFlow");
  ensure_sheet(wb, "Results_DCBranch");
  ensure_sheet(wb, "Results_VSCConverter");
  ensure_sheet(wb, "Results_DCDCConverter");
  ensure_sheet(wb, "Results_Storage_AC");
  ensure_sheet(wb, "Results_Storage_DC");
  ensure_sheet(wb, "Results_ChargingStation");
  ensure_sheet(wb, "Results_Charger");
  ensure_sheet(wb, "Results_EnergyRouter");
  ensure_sheet(wb, "Results_EnergyRouterPort");
  ensure_sheet(wb, "Results_VPP");
  ensure_sheet(wb, "Results_Microgrid");
  ensure_sheet(wb, "Results_MobileStorage");
  ensure_sheet(wb, "Results_Summary");

  write_results(sys, result, wb);

  doc.save();
  doc.close();
}

void save_results_compact_xlsx(const HybridPowerSystem& sys,
                               const PowerFlowResult& result,
                               const std::string& path) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("Info");
  } else {
    ensure_sheet(wb, "Info");
  }

  const std::vector<std::string> needed = {
      "ACBus",    "ACBranch",   "Generator",  "Load",      "ACLoad",          "FlexibleLoad",    "AsymmetricLoad",
      "StaticGenerator", "Storage",   "RenewableGen", "PVSystem", "Shunt",      "ExternalGrid", "Transformer2W",
      "Transformer3W", "ChargingStation", "Charger", "AsynchronousMotor", "Switch", "CircuitBreaker", "DCBus",
      "DCLoad",   "DCLoadRaw",  "DCBranch",   "DCStorage",     "DCStaticGen", "DCStaticGenerator", "DCPVArray", "DCCircuitBreaker",  "VSCConverter",  "DCDCConverter",
      "EnergyRouter", "EnergyRouterPort", "VPP", "Microgrid",
      "ThreePhaseBus", "ThreePhaseLine", "ThreePhaseTransformer", "ThreePhaseLoad", "ThreePhaseGenerator", "ThreePhaseExternalGrid",
      "MobileStorage", "Results_Summary"};

  for (const auto& name : needed) ensure_sheet(wb, name);

  const auto ac_buses_view = buses_with_aggregated_ac_loads(sys);
  const auto dc_buses_view = buses_with_aggregated_dc_loads(sys);
  const auto ac_loads_view = make_aggregated_ac_load_table(sys);
  const auto dc_loads_view = make_aggregated_dc_load_table(sys);

  write_info(wb.worksheet("Info"), sys);
  write_ac_buses(wb.worksheet("ACBus"), ac_buses_view);
  write_ac_branches(wb.worksheet("ACBranch"), sys.ac.branches);
  write_generators(wb.worksheet("Generator"), sys.ac.generators);
  write_loads(wb.worksheet("Load"), sys.ac.loads);
  write_loads(wb.worksheet("ACLoad"), ac_loads_view);
  write_flexible_loads(wb.worksheet("FlexibleLoad"), sys.ac.flexible_loads);
  write_asymmetric_loads(wb.worksheet("AsymmetricLoad"), sys.ac.asymmetric_loads);
  write_static_gens(wb.worksheet("StaticGenerator"), sys.ac.static_generators);
  write_storage(wb.worksheet("Storage"), sys.ac.storage);
  write_renewable(wb.worksheet("RenewableGen"), sys.ac.renewable_gens);
  write_pv(wb.worksheet("PVSystem"), sys.ac.pv_systems);
  write_shunts(wb.worksheet("Shunt"), sys.ac.shunts);
  write_external_grids(wb.worksheet("ExternalGrid"), sys.ac.external_grids);
  write_transformers_2w(wb.worksheet("Transformer2W"), sys.ac.transformers_2w);
  write_transformers_3w(wb.worksheet("Transformer3W"), sys.ac.transformers_3w);
  write_charging_stations(wb.worksheet("ChargingStation"), sys.ac.charging_stations);
  write_chargers(wb.worksheet("Charger"), sys.ac.chargers);
  write_asynchronous_motors(wb.worksheet("AsynchronousMotor"), sys.ac.motors);
  write_switches(wb.worksheet("Switch"), sys.ac.switches);
  write_circuit_breakers(wb.worksheet("CircuitBreaker"), sys.ac.circuit_breakers);
  write_dc_buses(wb.worksheet("DCBus"), dc_buses_view);
  write_dc_loads(wb.worksheet("DCLoad"), dc_loads_view);
  write_dc_loads(wb.worksheet("DCLoadRaw"), sys.dc.loads);
  write_dc_branches(wb.worksheet("DCBranch"), sys.dc.branches);
  write_storage(wb.worksheet("DCStorage"), sys.dc.storage);
  write_static_gens(wb.worksheet("DCStaticGen"), sys.dc.static_generators);
  write_dc_static_generators(wb.worksheet("DCStaticGenerator"), sys.dc.dc_static_generators);
  write_dc_pv_arrays(wb.worksheet("DCPVArray"), sys.dc.pv_arrays);
  write_dc_circuit_breakers(wb.worksheet("DCCircuitBreaker"), sys.dc.dc_circuit_breakers);
  write_vsc(wb.worksheet("VSCConverter"), sys.vsc_converters);
  write_dcdc(wb.worksheet("DCDCConverter"), sys.dc.dcdc_converters);
  write_energy_routers(wb.worksheet("EnergyRouter"), sys.energy_routers);
  write_energy_router_ports(wb.worksheet("EnergyRouterPort"), sys.energy_routers);
  write_vpp(wb.worksheet("VPP"), sys.vpps);
  write_microgrids(wb.worksheet("Microgrid"), sys.microgrids);
  write_mobile_storage(wb.worksheet("MobileStorage"), sys.mobile_storage);
  if (sys.three_phase_ac.has_value()) {
    write_three_phase_buses(wb.worksheet("ThreePhaseBus"), sys.three_phase_ac->buses);
    write_three_phase_lines(wb.worksheet("ThreePhaseLine"), sys.three_phase_ac->lines);
    write_three_phase_transformers(wb.worksheet("ThreePhaseTransformer"), sys.three_phase_ac->transformers);
    write_three_phase_loads(wb.worksheet("ThreePhaseLoad"), sys.three_phase_ac->loads);
    write_three_phase_generators(wb.worksheet("ThreePhaseGenerator"), sys.three_phase_ac->generators);
    write_three_phase_external_grids(wb.worksheet("ThreePhaseExternalGrid"), sys.three_phase_ac->external_grids);
  }

  write_results_compact(sys, result, wb);

  doc.save();
  doc.close();
}

// ═══════════════════════════════════════════════════════════════════════
// OPF result export
// ═══════════════════════════════════════════════════════════════════════

namespace {

void write_opf_results(const HybridPowerSystem& sys,
                       const opf::ACOPFResult& result,
                       XLWorkbook& wb) {
  // OPF_Bus: per-bus voltage results + load shedding
  {
    auto ws = ensure_sheet(wb, "OPF_Bus");
    write_headers(ws, {"index", "name", "base_kv", "vm_pu", "va_deg",
                       "dpd_mw", "dqd_mvar"});
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& b = sys.ac.buses[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = b.index;
      ws.cell(r, 2).value() = b.name;
      ws.cell(r, 3).value() = b.base_kv;
      ws.cell(r, 4).value() = (i < result.vm.size()) ? result.vm[i] : 0.0;
      ws.cell(r, 5).value() = (i < result.va.size()) ? result.va[i] : 0.0;
      ws.cell(r, 6).value() = (i < result.dpd_mw.size()) ? result.dpd_mw[i] : 0.0;
      ws.cell(r, 7).value() = (i < result.dqd_mvar.size()) ? result.dqd_mvar[i] : 0.0;
    }
  }

  // OPF_Generator: optimal dispatch
  {
    auto ws = ensure_sheet(wb, "OPF_Generator");
    write_headers(ws, {"index", "name", "bus", "pg_mw", "qg_mvar",
                       "pmax_mw", "pmin_mw", "qmax_mvar", "qmin_mvar",
                       "cost_c2", "cost_c1", "cost_c0", "fuel_type"});
    for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
      const auto& g = sys.ac.generators[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = g.index;
      ws.cell(r, 2).value() = g.name;
      ws.cell(r, 3).value() = g.bus;
      ws.cell(r, 4).value() = (i < result.pg_mw.size()) ? result.pg_mw[i] : g.pg_mw;
      ws.cell(r, 5).value() = (i < result.qg_mvar.size()) ? result.qg_mvar[i] : g.qg_mvar;
      ws.cell(r, 6).value() = g.pmax_mw;
      ws.cell(r, 7).value() = g.pmin_mw;
      ws.cell(r, 8).value() = g.qmax_mvar;
      ws.cell(r, 9).value() = g.qmin_mvar;
      ws.cell(r, 10).value() = g.cost_c2;
      ws.cell(r, 11).value() = g.cost_c1;
      ws.cell(r, 12).value() = g.cost_c0;
      ws.cell(r, 13).value() = fuel_type_str(g.fuel_type);
    }
  }

  // OPF_Converter: converter operating points (if any)
  if (!sys.vsc_converters.empty()) {
    auto ws = ensure_sheet(wb, "OPF_Converter");
    write_headers(ws, {"index", "name", "bus_ac", "bus_dc",
                       "pac_mw", "qac_mvar"});
    for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
      const auto& c = sys.vsc_converters[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = c.index;
      ws.cell(r, 2).value() = c.name;
      ws.cell(r, 3).value() = c.bus_ac;
      ws.cell(r, 4).value() = c.bus_dc;
      ws.cell(r, 5).value() = (i < result.pac_mw.size()) ? result.pac_mw[i] : 0.0;
      ws.cell(r, 6).value() = (i < result.qac_mvar.size()) ? result.qac_mvar[i] : 0.0;
    }
  }

  // OPF_DCBus: DC bus voltage results (if any)
  if (!sys.dc.buses.empty()) {
    auto ws = ensure_sheet(wb, "OPF_DCBus");
    write_headers(ws, {"index", "name", "vm_pu"});
    for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const auto& b = sys.dc.buses[i];
      const uint32_t r = static_cast<uint32_t>(i + 2);
      ws.cell(r, 1).value() = b.index;
      ws.cell(r, 2).value() = b.name;
      ws.cell(r, 3).value() = (i < result.vdc.size()) ? result.vdc[i] : b.vm_pu;
    }
  }

  // OPF_Renewable: curtailable renewable dispatch (packed variable results)
  if (!result.pren_mw.empty()) {
    auto ws = ensure_sheet(wb, "OPF_Renewable");
    write_headers(ws, {"var_idx", "source_type", "component_index", "name", "bus",
                       "type", "pren_mw", "qren_mvar", "p_rated_mw"});
    for (size_t k = 0; k < result.pren_mw.size(); ++k) {
      const uint32_t r = static_cast<uint32_t>(k + 2);
      ws.cell(r, 1).value() = static_cast<int>(k);

      // Resolve identity via ren_map
      int src = 0;
      int oi  = static_cast<int>(k);
      if (k < result.ren_map.size()) {
        src = result.ren_map[k].source_type;
        oi  = result.ren_map[k].original_index;
      }
      ws.cell(r, 2).value() = (src == 0) ? "RenewableGen" : "PVSystem";
      ws.cell(r, 3).value() = oi;

      if (src == 0 && static_cast<size_t>(oi) < sys.ac.renewable_gens.size()) {
        const auto& rg = sys.ac.renewable_gens[static_cast<size_t>(oi)];
        ws.cell(r, 4).value() = rg.name;
        ws.cell(r, 5).value() = rg.bus;
        ws.cell(r, 6).value() = (rg.type == RenewableType::Wind) ? "Wind" : "Solar";
        ws.cell(r, 9).value() = rg.p_rated_mw;
      } else if (src == 1 && static_cast<size_t>(oi) < sys.ac.pv_systems.size()) {
        const auto& pv = sys.ac.pv_systems[static_cast<size_t>(oi)];
        ws.cell(r, 4).value() = pv.name;
        ws.cell(r, 5).value() = pv.bus;
        ws.cell(r, 6).value() = "PV";
        ws.cell(r, 9).value() = pv.pmax_mw;
      }
      ws.cell(r, 7).value() = result.pren_mw[k];
      ws.cell(r, 8).value() = (k < result.qren_mvar.size()) ? result.qren_mvar[k] : 0.0;
    }
  }

  // OPF_Storage: storage dispatch (packed variable results)
  if (!result.pstor_mw.empty()) {
    auto ws = ensure_sheet(wb, "OPF_Storage");
    write_headers(ws, {"var_idx", "source_type", "component_index", "name", "bus",
                       "pstor_mw", "qstor_mvar", "pmax_mw", "pmin_mw"});
    for (size_t k = 0; k < result.pstor_mw.size(); ++k) {
      const uint32_t r = static_cast<uint32_t>(k + 2);
      ws.cell(r, 1).value() = static_cast<int>(k);

      int src = 0;
      int oi = static_cast<int>(k);
      if (k < result.stor_map.size()) {
        src = result.stor_map[k].source_type;
        oi = result.stor_map[k].original_index;
      }
      ws.cell(r, 2).value() = (src == 0) ? "ACStorage" : "DCStorage";
      ws.cell(r, 3).value() = oi;

      if (src == 0 && static_cast<size_t>(oi) < sys.ac.storage.size()) {
        const auto& s = sys.ac.storage[static_cast<size_t>(oi)];
        ws.cell(r, 4).value() = s.name;
        ws.cell(r, 5).value() = s.bus;
        ws.cell(r, 8).value() = s.pmax_mw;
        ws.cell(r, 9).value() = s.pmin_mw;
      } else if (src == 1 && static_cast<size_t>(oi) < sys.dc.storage.size()) {
        const auto& s = sys.dc.storage[static_cast<size_t>(oi)];
        ws.cell(r, 4).value() = s.name;
        ws.cell(r, 5).value() = s.bus;
        ws.cell(r, 8).value() = s.pmax_mw;
        ws.cell(r, 9).value() = s.pmin_mw;
      }
      ws.cell(r, 6).value() = result.pstor_mw[k];
      ws.cell(r, 7).value() = (k < result.qstor_mvar.size()) ? result.qstor_mvar[k] : 0.0;
    }
  }

  // OPF_DCDC: DC-DC converter transfer (packed variable results)
  if (!result.pdcdc_mw.empty()) {
    auto ws = ensure_sheet(wb, "OPF_DCDC");
    write_headers(ws, {"var_idx", "component_index", "name", "bus_in", "bus_out",
                       "pdcdc_mw", "pmax_mw", "pmin_mw"});
    for (size_t k = 0; k < result.pdcdc_mw.size(); ++k) {
      const uint32_t r = static_cast<uint32_t>(k + 2);
      ws.cell(r, 1).value() = static_cast<int>(k);

      int oi = static_cast<int>(k);
      if (k < result.dcdc_map.size())
        oi = result.dcdc_map[k].original_index;
      ws.cell(r, 2).value() = oi;

      if (static_cast<size_t>(oi) < sys.dc.dcdc_converters.size()) {
        const auto& d = sys.dc.dcdc_converters[static_cast<size_t>(oi)];
        ws.cell(r, 3).value() = d.name;
        ws.cell(r, 4).value() = d.bus_in;
        ws.cell(r, 5).value() = d.bus_out;
        ws.cell(r, 7).value() = d.pmax_mw;
        ws.cell(r, 8).value() = d.pmin_mw;
      }
      ws.cell(r, 6).value() = result.pdcdc_mw[k];
    }
  }

  // OPF_FlexLoad: flexible load actual demand (packed variable results)
  if (!result.pflex_mw.empty()) {
    auto ws = ensure_sheet(wb, "OPF_FlexLoad");
    write_headers(ws, {"var_idx", "component_index", "name", "bus",
                       "pflex_mw", "p_baseline_mw", "flex_up_mw", "flex_down_mw"});
    for (size_t k = 0; k < result.pflex_mw.size(); ++k) {
      const uint32_t r = static_cast<uint32_t>(k + 2);
      ws.cell(r, 1).value() = static_cast<int>(k);

      int oi = static_cast<int>(k);
      if (k < result.flex_map.size())
        oi = result.flex_map[k].original_index;
      ws.cell(r, 2).value() = oi;

      if (static_cast<size_t>(oi) < sys.ac.flexible_loads.size()) {
        const auto& fl = sys.ac.flexible_loads[static_cast<size_t>(oi)];
        ws.cell(r, 3).value() = fl.name;
        ws.cell(r, 4).value() = fl.bus;
        ws.cell(r, 6).value() = fl.p_mw;
        ws.cell(r, 7).value() = fl.flex_up_mw;
        ws.cell(r, 8).value() = fl.flex_down_mw;
      }
      ws.cell(r, 5).value() = result.pflex_mw[k];
    }
  }

  // OPF_Summary
  {
    auto ws = ensure_sheet(wb, "OPF_Summary");
    write_headers(ws, {"Key", "Value"});
    uint32_t r = 2;
    ws.cell(r, 1).value() = "converged";
    ws.cell(r, 2).value() = bool_str(result.converged);
    ++r;
    ws.cell(r, 1).value() = "iterations";
    ws.cell(r, 2).value() = result.iterations;
    ++r;
    ws.cell(r, 1).value() = "outer_iterations";
    ws.cell(r, 2).value() = result.outer_iterations;
    ++r;
    ws.cell(r, 1).value() = "objective";
    ws.cell(r, 2).value() = result.objective;
    ++r;
    ws.cell(r, 1).value() = "max_constraint_violation";
    ws.cell(r, 2).value() = result.max_constraint_violation;
    ++r;
    ws.cell(r, 1).value() = "max_stationarity";
    ws.cell(r, 2).value() = result.max_stationarity;
    ++r;
    ws.cell(r, 1).value() = "status";
    ws.cell(r, 2).value() = result.status;
    ++r;
    ws.cell(r, 1).value() = "solver_path";
    switch (result.solver_path) {
      case opf::OPFSolverPath::NativeAC:  ws.cell(r, 2).value() = "NativeAC"; break;
      case opf::OPFSolverPath::ParityIPM: ws.cell(r, 2).value() = "ParityIPM"; break;
      default:                            ws.cell(r, 2).value() = "Unknown"; break;
    }
  }
}

}  // anonymous namespace

void save_opf_results_xlsx(const HybridPowerSystem& sys,
                           const opf::ACOPFResult& result,
                           const std::string& path) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("OPF_Summary");
  }

  ensure_sheet(wb, "OPF_Bus");
  ensure_sheet(wb, "OPF_Generator");
  ensure_sheet(wb, "OPF_Converter");
  ensure_sheet(wb, "OPF_Summary");

  write_opf_results(sys, result, wb);

  doc.save();
  doc.close();
}

void save_opf_results_compact_xlsx(const HybridPowerSystem& sys,
                                   const opf::ACOPFResult& result,
                                   const std::string& path) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("Info");
  } else {
    ensure_sheet(wb, "Info");
  }

  const std::vector<std::string> needed = {
      "ACBus",    "ACBranch",   "Generator",  "Load",      "ACLoad", "FlexibleLoad", "AsymmetricLoad",
      "StaticGenerator", "Storage",    "RenewableGen",
      "PVSystem", "Shunt",      "ExternalGrid", "Transformer2W", "Transformer3W",
      "ChargingStation", "Charger", "AsynchronousMotor", "Switch", "CircuitBreaker",
      "DCBus",    "DCLoad",     "DCLoadRaw",  "DCBranch",
      "DCStorage", "DCStaticGen", "DCStaticGenerator", "DCPVArray", "DCCircuitBreaker",
      "VSCConverter",  "DCDCConverter", "EnergyRouter", "EnergyRouterPort", "VPP", "Microgrid",
      "ThreePhaseBus", "ThreePhaseLine", "ThreePhaseTransformer", "ThreePhaseLoad", "ThreePhaseGenerator", "ThreePhaseExternalGrid",
      "MobileStorage",
      "OPF_Bus",  "OPF_Generator", "OPF_Converter", "OPF_Summary"};

  for (const auto& name : needed) ensure_sheet(wb, name);

  const auto ac_buses_view = buses_with_aggregated_ac_loads(sys);
  const auto dc_buses_view = buses_with_aggregated_dc_loads(sys);
  const auto ac_loads_view = make_aggregated_ac_load_table(sys);
  const auto dc_loads_view = make_aggregated_dc_load_table(sys);

  write_info(wb.worksheet("Info"), sys);
  write_ac_buses(wb.worksheet("ACBus"), ac_buses_view);
  write_ac_branches(wb.worksheet("ACBranch"), sys.ac.branches);
  write_generators(wb.worksheet("Generator"), sys.ac.generators);
  write_loads(wb.worksheet("Load"), sys.ac.loads);
  write_loads(wb.worksheet("ACLoad"), ac_loads_view);
  write_flexible_loads(wb.worksheet("FlexibleLoad"), sys.ac.flexible_loads);
  write_asymmetric_loads(wb.worksheet("AsymmetricLoad"), sys.ac.asymmetric_loads);
  write_static_gens(wb.worksheet("StaticGenerator"), sys.ac.static_generators);
  write_storage(wb.worksheet("Storage"), sys.ac.storage);
  write_renewable(wb.worksheet("RenewableGen"), sys.ac.renewable_gens);
  write_pv(wb.worksheet("PVSystem"), sys.ac.pv_systems);
  write_shunts(wb.worksheet("Shunt"), sys.ac.shunts);
  write_external_grids(wb.worksheet("ExternalGrid"), sys.ac.external_grids);
  write_transformers_2w(wb.worksheet("Transformer2W"), sys.ac.transformers_2w);
  write_transformers_3w(wb.worksheet("Transformer3W"), sys.ac.transformers_3w);
  write_charging_stations(wb.worksheet("ChargingStation"), sys.ac.charging_stations);
  write_chargers(wb.worksheet("Charger"), sys.ac.chargers);
  write_asynchronous_motors(wb.worksheet("AsynchronousMotor"), sys.ac.motors);
  write_switches(wb.worksheet("Switch"), sys.ac.switches);
  write_circuit_breakers(wb.worksheet("CircuitBreaker"), sys.ac.circuit_breakers);
  write_dc_buses(wb.worksheet("DCBus"), dc_buses_view);
  write_dc_loads(wb.worksheet("DCLoad"), dc_loads_view);
  write_dc_loads(wb.worksheet("DCLoadRaw"), sys.dc.loads);
  write_dc_branches(wb.worksheet("DCBranch"), sys.dc.branches);
  write_storage(wb.worksheet("DCStorage"), sys.dc.storage);
  write_static_gens(wb.worksheet("DCStaticGen"), sys.dc.static_generators);
  write_dc_static_generators(wb.worksheet("DCStaticGenerator"), sys.dc.dc_static_generators);
  write_dc_pv_arrays(wb.worksheet("DCPVArray"), sys.dc.pv_arrays);
  write_dc_circuit_breakers(wb.worksheet("DCCircuitBreaker"), sys.dc.dc_circuit_breakers);
  write_vsc(wb.worksheet("VSCConverter"), sys.vsc_converters);
  write_dcdc(wb.worksheet("DCDCConverter"), sys.dc.dcdc_converters);
  write_energy_routers(wb.worksheet("EnergyRouter"), sys.energy_routers);
  write_energy_router_ports(wb.worksheet("EnergyRouterPort"), sys.energy_routers);
  write_vpp(wb.worksheet("VPP"), sys.vpps);
  write_microgrids(wb.worksheet("Microgrid"), sys.microgrids);
  write_mobile_storage(wb.worksheet("MobileStorage"), sys.mobile_storage);
  if (sys.three_phase_ac.has_value()) {
    write_three_phase_buses(wb.worksheet("ThreePhaseBus"), sys.three_phase_ac->buses);
    write_three_phase_lines(wb.worksheet("ThreePhaseLine"), sys.three_phase_ac->lines);
    write_three_phase_transformers(wb.worksheet("ThreePhaseTransformer"), sys.three_phase_ac->transformers);
    write_three_phase_loads(wb.worksheet("ThreePhaseLoad"), sys.three_phase_ac->loads);
    write_three_phase_generators(wb.worksheet("ThreePhaseGenerator"), sys.three_phase_ac->generators);
    write_three_phase_external_grids(wb.worksheet("ThreePhaseExternalGrid"), sys.three_phase_ac->external_grids);
  }

  write_opf_results(sys, result, wb);

  doc.save();
  doc.close();
}

// ═══════════════════════════════════════════════════════════════════════
// Carbon Analysis Results Excel Export
// ═══════════════════════════════════════════════════════════════════════

void save_carbon_results_xlsx(const analysis::CarbonAnalysisResult& result,
                              const std::string& path) {
  using namespace OpenXLSX;
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  // --- Sheet 1: Load Carbon Intensity ---
  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("LoadCarbon");
  } else {
    wb.addWorksheet("LoadCarbon");
  }
  auto write_load_sheet = [](XLWorksheet ws,
                             const std::vector<analysis::LoadCarbonResult>& items) {
    ws.cell(1, 1).value() = "load_index";
    ws.cell(1, 2).value() = "bus";
    ws.cell(1, 3).value() = "demand_mw";
    ws.cell(1, 4).value() = "carbon_intensity_tco2_mwh";
    ws.cell(1, 5).value() = "total_emissions_tco2";
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
      const int r = i + 2;
      const auto& lc = items[i];
      ws.cell(r, 1).value() = lc.load_index;
      ws.cell(r, 2).value() = lc.bus;
      ws.cell(r, 3).value() = lc.demand_mw;
      ws.cell(r, 4).value() = lc.carbon_intensity_tco2_mwh;
      ws.cell(r, 5).value() = lc.total_emissions_tco2;
    }
  };
  write_load_sheet(wb.worksheet("LoadCarbon"), result.load_carbon);

  wb.addWorksheet("DCLoadCarbon");
  write_load_sheet(wb.worksheet("DCLoadCarbon"), result.dc_load_carbon);

  // --- Sheet 2: Bus Carbon Intensity ---
  auto write_bus_sheet = [](XLWorksheet ws,
                            const std::vector<analysis::BusCarbonResult>& items) {
    ws.cell(1, 1).value() = "bus_index";
    ws.cell(1, 2).value() = "carbon_intensity_tco2_mwh";
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
      const int r = i + 2;
      ws.cell(r, 1).value() = items[i].bus_index;
      ws.cell(r, 2).value() = items[i].carbon_intensity_tco2_mwh;
    }
  };
  wb.addWorksheet("BusCarbon");
  write_bus_sheet(wb.worksheet("BusCarbon"), result.bus_carbon);

  wb.addWorksheet("DCBusCarbon");
  write_bus_sheet(wb.worksheet("DCBusCarbon"), result.dc_bus_carbon);

  // --- Sheet 3: Branch Loss Emissions ---
  auto write_branch_sheet = [](XLWorksheet ws,
                               const std::vector<analysis::BranchCarbonResult>& items) {
    ws.cell(1, 1).value() = "branch_index";
    ws.cell(1, 2).value() = "from_bus";
    ws.cell(1, 3).value() = "to_bus";
    ws.cell(1, 4).value() = "loss_mw";
    ws.cell(1, 5).value() = "carbon_intensity_tco2_mwh";
    ws.cell(1, 6).value() = "total_emissions_tco2";
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
      const int r = i + 2;
      const auto& bc = items[i];
      ws.cell(r, 1).value() = bc.branch_index;
      ws.cell(r, 2).value() = bc.from_bus;
      ws.cell(r, 3).value() = bc.to_bus;
      ws.cell(r, 4).value() = bc.loss_mw;
      ws.cell(r, 5).value() = bc.carbon_intensity_tco2_mwh;
      ws.cell(r, 6).value() = bc.total_emissions_tco2;
    }
  };
  wb.addWorksheet("BranchCarbon");
  write_branch_sheet(wb.worksheet("BranchCarbon"), result.branch_carbon);

  wb.addWorksheet("DCBranchCarbon");
  write_branch_sheet(wb.worksheet("DCBranchCarbon"), result.dc_branch_carbon);

  wb.addWorksheet("VSCCarbon");
  auto ws_vsc = wb.worksheet("VSCCarbon");
  ws_vsc.cell(1, 1).value() = "converter_index";
  ws_vsc.cell(1, 2).value() = "bus_ac";
  ws_vsc.cell(1, 3).value() = "bus_dc";
  ws_vsc.cell(1, 4).value() = "ac_to_dc";
  ws_vsc.cell(1, 5).value() = "input_power_mw";
  ws_vsc.cell(1, 6).value() = "output_power_mw";
  ws_vsc.cell(1, 7).value() = "loss_mw";
  ws_vsc.cell(1, 8).value() = "carbon_intensity_tco2_mwh";
  ws_vsc.cell(1, 9).value() = "total_emissions_tco2";
  for (int i = 0; i < static_cast<int>(result.vsc_carbon.size()); ++i) {
    const int r = i + 2;
    const auto& vc = result.vsc_carbon[i];
    ws_vsc.cell(r, 1).value() = vc.converter_index;
    ws_vsc.cell(r, 2).value() = vc.bus_ac;
    ws_vsc.cell(r, 3).value() = vc.bus_dc;
    ws_vsc.cell(r, 4).value() = vc.ac_to_dc ? "true" : "false";
    ws_vsc.cell(r, 5).value() = vc.input_power_mw;
    ws_vsc.cell(r, 6).value() = vc.output_power_mw;
    ws_vsc.cell(r, 7).value() = vc.loss_mw;
    ws_vsc.cell(r, 8).value() = vc.carbon_intensity_tco2_mwh;
    ws_vsc.cell(r, 9).value() = vc.total_emissions_tco2;
  }

  wb.addWorksheet("DCDCCarbon");
  auto ws_dcdc = wb.worksheet("DCDCCarbon");
  ws_dcdc.cell(1, 1).value() = "converter_index";
  ws_dcdc.cell(1, 2).value() = "bus_in";
  ws_dcdc.cell(1, 3).value() = "bus_out";
  ws_dcdc.cell(1, 4).value() = "input_to_output";
  ws_dcdc.cell(1, 5).value() = "input_power_mw";
  ws_dcdc.cell(1, 6).value() = "output_power_mw";
  ws_dcdc.cell(1, 7).value() = "loss_mw";
  ws_dcdc.cell(1, 8).value() = "carbon_intensity_tco2_mwh";
  ws_dcdc.cell(1, 9).value() = "total_emissions_tco2";
  for (int i = 0; i < static_cast<int>(result.dcdc_carbon.size()); ++i) {
    const int r = i + 2;
    const auto& dc = result.dcdc_carbon[i];
    ws_dcdc.cell(r, 1).value() = dc.converter_index;
    ws_dcdc.cell(r, 2).value() = dc.bus_in;
    ws_dcdc.cell(r, 3).value() = dc.bus_out;
    ws_dcdc.cell(r, 4).value() = dc.input_to_output ? "true" : "false";
    ws_dcdc.cell(r, 5).value() = dc.input_power_mw;
    ws_dcdc.cell(r, 6).value() = dc.output_power_mw;
    ws_dcdc.cell(r, 7).value() = dc.loss_mw;
    ws_dcdc.cell(r, 8).value() = dc.carbon_intensity_tco2_mwh;
    ws_dcdc.cell(r, 9).value() = dc.total_emissions_tco2;
  }

  wb.addWorksheet("StorageCarbon");
  auto ws_storage = wb.worksheet("StorageCarbon");
  ws_storage.cell(1, 1).value() = "storage_index";
  ws_storage.cell(1, 2).value() = "bus";
  ws_storage.cell(1, 3).value() = "is_dc";
  ws_storage.cell(1, 4).value() = "p_mw";
  ws_storage.cell(1, 5).value() = "soc";
  ws_storage.cell(1, 6).value() = "stored_energy_mwh";
  ws_storage.cell(1, 7).value() = "soc_carbon_intensity_tco2_mwh";
  ws_storage.cell(1, 8).value() = "carbon_intensity_tco2_mwh";
  ws_storage.cell(1, 9).value() = "total_emissions_tco2";
  for (int i = 0; i < static_cast<int>(result.storage_carbon.size()); ++i) {
    const int r = i + 2;
    const auto& sc = result.storage_carbon[i];
    ws_storage.cell(r, 1).value() = sc.storage_index;
    ws_storage.cell(r, 2).value() = sc.bus;
    ws_storage.cell(r, 3).value() = sc.is_dc ? "true" : "false";
    ws_storage.cell(r, 4).value() = sc.p_mw;
    ws_storage.cell(r, 5).value() = sc.soc;
    ws_storage.cell(r, 6).value() = sc.stored_energy_mwh;
    ws_storage.cell(r, 7).value() = sc.soc_carbon_intensity_tco2_mwh;
    ws_storage.cell(r, 8).value() = sc.carbon_intensity_tco2_mwh;
    ws_storage.cell(r, 9).value() = sc.total_emissions_tco2;
  }

  wb.addWorksheet("EnergyRouterCarbon");
  auto ws_router = wb.worksheet("EnergyRouterCarbon");
  ws_router.cell(1, 1).value() = "router_index";
  ws_router.cell(1, 2).value() = "input_power_mw";
  ws_router.cell(1, 3).value() = "output_power_mw";
  ws_router.cell(1, 4).value() = "loss_mw";
  ws_router.cell(1, 5).value() = "active_input_ports";
  ws_router.cell(1, 6).value() = "active_output_ports";
  ws_router.cell(1, 7).value() = "carbon_intensity_tco2_mwh";
  ws_router.cell(1, 8).value() = "total_emissions_tco2";
  for (int i = 0; i < static_cast<int>(result.energy_router_carbon.size()); ++i) {
    const int r = i + 2;
    const auto& rc = result.energy_router_carbon[i];
    ws_router.cell(r, 1).value() = rc.router_index;
    ws_router.cell(r, 2).value() = rc.input_power_mw;
    ws_router.cell(r, 3).value() = rc.output_power_mw;
    ws_router.cell(r, 4).value() = rc.loss_mw;
    ws_router.cell(r, 5).value() = rc.active_input_ports;
    ws_router.cell(r, 6).value() = rc.active_output_ports;
    ws_router.cell(r, 7).value() = rc.carbon_intensity_tco2_mwh;
    ws_router.cell(r, 8).value() = rc.total_emissions_tco2;
  }

  // --- Sheet 4: Summary ---
  wb.addWorksheet("Summary");
  auto ws_sum = wb.worksheet("Summary");

  auto write_summary = [&](int row, const std::string& label,
                            const analysis::EmissionsSummary& s) {
    ws_sum.cell(row, 1).value() = label;
    ws_sum.cell(row + 1, 1).value() = "total_generation_emissions_tco2";
    ws_sum.cell(row + 1, 2).value() = s.total_generation_emissions_tco2;
    ws_sum.cell(row + 2, 1).value() = "total_load_emissions_tco2";
    ws_sum.cell(row + 2, 2).value() = s.total_load_emissions_tco2;
    ws_sum.cell(row + 3, 1).value() = "total_loss_emissions_tco2";
    ws_sum.cell(row + 3, 2).value() = s.total_loss_emissions_tco2;
    ws_sum.cell(row + 4, 1).value() = "balance_error_tco2";
    ws_sum.cell(row + 4, 2).value() = s.balance_error_tco2;
    ws_sum.cell(row + 5, 1).value() = "balance_error_pct";
    ws_sum.cell(row + 5, 2).value() = s.balance_error_pct;
  };

  write_summary(1, "Tracing Method", result.tracing_summary);
  ws_sum.cell(7, 1).value() = "tracing_verified";
  ws_sum.cell(7, 2).value() = result.tracing_verified ? "true" : "false";

  write_summary(9, "Matrix Method", result.matrix_summary);
  ws_sum.cell(15, 1).value() = "matrix_solved";
  ws_sum.cell(15, 2).value() = result.matrix_solved ? "true" : "false";
  ws_sum.cell(16, 1).value() = "matrix_residual";
  ws_sum.cell(16, 2).value() = result.matrix_residual;

  doc.save();
  doc.close();
}

}  // namespace hacdcpf::io
