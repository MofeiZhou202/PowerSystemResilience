#pragma once

// Shared enum ↔ string conversions for all hacdcpf enum types.
// Used by json_io, etap_io, html_visualizer, and any future serializers.

#include <algorithm>
#include <cctype>
#include <string>

#include "hacdcpf/model/enums.hpp"

namespace hacdcpf {

inline std::string enum_parse_key(std::string s) {
  std::string out;
  out.reserve(s.size());
  bool last_sep = false;
  for (unsigned char ch : s) {
    if (std::isalnum(ch)) {
      out.push_back(static_cast<char>(std::toupper(ch)));
      last_sep = false;
    } else if (!last_sep) {
      out.push_back('_');
      last_sep = true;
    }
  }
  while (!out.empty() && out.front() == '_') out.erase(out.begin());
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out;
}

// ── BusType ─────────────────────────────────────────────────────────────
inline std::string bus_type_str(BusType t) {
  switch (t) {
    case BusType::PQ: return "PQ";
    case BusType::PV: return "PV";
    case BusType::SLACK: return "SLACK";
    case BusType::ISOLATED: return "ISOLATED";
  }
  return "PQ";
}
inline BusType bus_type_from_str(const std::string& s) {
  if (s == "PV") return BusType::PV;
  if (s == "SLACK") return BusType::SLACK;
  if (s == "ISOLATED") return BusType::ISOLATED;
  return BusType::PQ;
}

// ── DCBusType ───────────────────────────────────────────────────────────
inline std::string dc_bus_type_str(DCBusType t) {
  switch (t) {
    case DCBusType::DC_V: return "DC_V";
    case DCBusType::DC_ISOLATED: return "DC_ISOLATED";
    case DCBusType::DC_P: return "DC_P";
  }
  return "DC_P";
}
inline DCBusType dc_bus_type_from_str(const std::string& s) {
  if (s == "DC_V") return DCBusType::DC_V;
  if (s == "DC_ISOLATED") return DCBusType::DC_ISOLATED;
  // Legacy/GUI-authored files may store AC-style "SLACK" for a DC bus.  DC buses
  // have no SLACK type — a DC bus is only a true voltage reference (DC_V) when a
  // real element holds its voltage.  Map the legacy alias to DC_P so DC-side
  // voltage regulation is decided by converter/island analysis, not the label.
  if (s == "SLACK" || s == "DC_SLACK") return DCBusType::DC_P;
  return DCBusType::DC_P;
}

// ── ConverterMode ───────────────────────────────────────────────────────
inline std::string converter_mode_str(ConverterMode m) {
  switch (m) {
    case ConverterMode::PQ_MODE: return "PQ";
    case ConverterMode::VDC_Q: return "VDC_Q";
    case ConverterMode::VDC_VAC: return "VDC_VAC";
    case ConverterMode::AC_PV: return "AC_PV";
    case ConverterMode::AC_GRID_FORMING: return "AC_GRID_FORMING";
    case ConverterMode::DC_V_DROOP_AC_V: return "DC_V_DROOP_AC_V";
  }
  return "PQ";
}
inline ConverterMode converter_mode_from_str(const std::string& s) {
  const std::string k = enum_parse_key(s);
  if (k == "PQ" || k == "P_Q" || k == "PQ_MODE" || k == "AC_PQ" || k == "AC_P_Q") return ConverterMode::PQ_MODE;
  if (k == "VDC_Q" || k == "DC_V_AC_Q" || k == "DC_V_DROOP_AC_Q") return ConverterMode::VDC_Q;
  if (k == "VDC_VAC" || k == "VDC_V_AC" || k == "DC_V_AC_V") return ConverterMode::VDC_VAC;
  if (k == "AC_PV" || k == "AC_P_V") return ConverterMode::AC_PV;
  if (k == "AC_GRID_FORMING" || k == "AC_GFM" || k == "GFM" || k == "GRID_FORMING") return ConverterMode::AC_GRID_FORMING;
  if (k == "DC_V_DROOP_AC_V") return ConverterMode::DC_V_DROOP_AC_V;
  return ConverterMode::PQ_MODE;
}

// Seven-mode VSC control taxonomy (multi-converter model r1 §6.1).
inline std::string acdc_control_mode_str(ACDCControlMode m) {
  switch (m) {
    case ACDCControlMode::AC_PQ: return "AC_PQ";
    case ACDCControlMode::AC_PV: return "AC_PV";
    case ACDCControlMode::DC_V_AC_Q: return "DC_V_AC_Q";
    case ACDCControlMode::DC_V_AC_V: return "DC_V_AC_V";
    case ACDCControlMode::DC_V_DROOP_AC_Q: return "DC_V_DROOP_AC_Q";
    case ACDCControlMode::DC_V_DROOP_AC_V: return "DC_V_DROOP_AC_V";
    case ACDCControlMode::AC_GRID_FORMING: return "AC_GRID_FORMING";
  }
  return "AC_PQ";
}

// Map an internal ConverterMode to the human-facing seven-mode taxonomy.  The
// rigid-vs-droop Vdc distinction (Mode 4/5 vs 6/7) is resolved at solve time by
// the DC-island reference logic, so VDC_Q/VDC_VAC report as the droop variants.
inline ACDCControlMode to_acdc_control_mode(ConverterMode m) {
  switch (m) {
    case ConverterMode::PQ_MODE: return ACDCControlMode::AC_PQ;
    case ConverterMode::AC_PV: return ACDCControlMode::AC_PV;
    case ConverterMode::VDC_Q: return ACDCControlMode::DC_V_DROOP_AC_Q;
    case ConverterMode::VDC_VAC: return ACDCControlMode::DC_V_AC_V;
    case ConverterMode::DC_V_DROOP_AC_V: return ACDCControlMode::DC_V_DROOP_AC_V;
    case ConverterMode::AC_GRID_FORMING: return ACDCControlMode::AC_GRID_FORMING;
  }
  return ACDCControlMode::AC_PQ;
}

// ── FuelType ────────────────────────────────────────────────────────────
// ── LCCStationRole / LCCControlMode ─────────────────────────────────────
inline std::string lcc_station_role_str(LCCStationRole r) {
  switch (r) {
    case LCCStationRole::Rectifier: return "RECTIFIER";
    case LCCStationRole::Inverter: return "INVERTER";
  }
  return "RECTIFIER";
}
inline LCCStationRole lcc_station_role_from_str(const std::string& s) {
  const std::string k = enum_parse_key(s);
  if (k == "INVERTER" || k == "INV" || k == "I") return LCCStationRole::Inverter;
  return LCCStationRole::Rectifier;
}

inline std::string lcc_control_mode_str(LCCControlMode m) {
  switch (m) {
    case LCCControlMode::ConstantPower: return "CONSTANT_POWER";
    case LCCControlMode::ConstantCurrent: return "CONSTANT_CURRENT";
    case LCCControlMode::ConstantAlpha: return "CONSTANT_ALPHA";
    case LCCControlMode::ConstantGamma: return "CONSTANT_GAMMA";
  }
  return "CONSTANT_POWER";
}
inline LCCControlMode lcc_control_mode_from_str(const std::string& s) {
  const std::string k = enum_parse_key(s);
  if (k == "CONSTANT_CURRENT" || k == "CC" || k == "CURRENT") {
    return LCCControlMode::ConstantCurrent;
  }
  if (k == "CONSTANT_ALPHA" || k == "ALPHA" || k == "CIA") {
    return LCCControlMode::ConstantAlpha;
  }
  if (k == "CONSTANT_GAMMA" || k == "GAMMA" || k == "CEA") {
    return LCCControlMode::ConstantGamma;
  }
  return LCCControlMode::ConstantPower;
}

inline std::string fuel_type_str(FuelType f) {
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
inline FuelType fuel_type_from_str(const std::string& s) {
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

// ── LoadModel ───────────────────────────────────────────────────────────
inline std::string load_model_str(LoadModel m) {
  switch (m) {
    case LoadModel::ZIP: return "ZIP";
    case LoadModel::Exponential: return "Exponential";
    default: return "ConstantPower";
  }
}
inline LoadModel load_model_from_str(const std::string& s) {
  if (s == "ZIP") return LoadModel::ZIP;
  if (s == "Exponential") return LoadModel::Exponential;
  return LoadModel::ConstantPower;
}

// ── LoadPriority ────────────────────────────────────────────────────────
inline std::string load_priority_str(LoadPriority p) {
  switch (p) {
    case LoadPriority::Low: return "Low";
    case LoadPriority::High: return "High";
    case LoadPriority::Critical: return "Critical";
    default: return "Medium";
  }
}
inline LoadPriority load_priority_from_str(const std::string& s) {
  if (s == "Low") return LoadPriority::Low;
  if (s == "High") return LoadPriority::High;
  if (s == "Critical") return LoadPriority::Critical;
  return LoadPriority::Medium;
}

// ── RenewableType ───────────────────────────────────────────────────────
inline std::string renewable_type_str(RenewableType t) {
  switch (t) {
    case RenewableType::Wind: return "Wind";
    case RenewableType::SolarPV: return "SolarPV";
    case RenewableType::SolarCSP: return "SolarCSP";
    case RenewableType::Hydro: return "Hydro";
  }
  return "Wind";
}
inline RenewableType renewable_type_from_str(const std::string& s) {
  if (s == "SolarPV") return RenewableType::SolarPV;
  if (s == "SolarCSP") return RenewableType::SolarCSP;
  if (s == "Hydro") return RenewableType::Hydro;
  return RenewableType::Wind;
}

// ── SgenType ────────────────────────────────────────────────────────────
inline std::string sgen_type_str(SgenType t) {
  switch (t) {
    case SgenType::PV: return "PV";
    case SgenType::Wind: return "Wind";
    case SgenType::CHP: return "CHP";
    case SgenType::Diesel: return "Diesel";
    case SgenType::FuelCell: return "FuelCell";
    default: return "Other";
  }
}
inline SgenType sgen_type_from_str(const std::string& s) {
  if (s == "PV") return SgenType::PV;
  if (s == "Wind") return SgenType::Wind;
  if (s == "CHP") return SgenType::CHP;
  if (s == "Diesel") return SgenType::Diesel;
  if (s == "FuelCell") return SgenType::FuelCell;
  return SgenType::Other;
}

// ── DCDCControlMode ─────────────────────────────────────────────────────
inline std::string dcdc_control_str(DCDCControlMode m) {
  switch (m) {
    case DCDCControlMode::Power: return "Power";
    case DCDCControlMode::Droop: return "Droop";
    default: return "Voltage";
  }
}
inline DCDCControlMode dcdc_control_from_str(const std::string& s) {
  const std::string k = enum_parse_key(s);
  if (k == "POWER" || k == "P" || k == "P_MODE" || k == "P_REF" || k == "CONSTANT_POWER") return DCDCControlMode::Power;
  if (k == "DROOP" || k == "D" || k == "DROOP_MODE") return DCDCControlMode::Droop;
  if (k == "VOLTAGE" || k == "V" || k == "V_MODE" || k == "DC_V" || k == "CONSTANT_VOLTAGE") return DCDCControlMode::Voltage;
  return DCDCControlMode::Voltage;
}

// ── DCDCTopology ────────────────────────────────────────────────────────
inline std::string dcdc_topology_str(DCDCTopology t) {
  switch (t) {
    case DCDCTopology::Buck: return "Buck";
    case DCDCTopology::Boost: return "Boost";
    case DCDCTopology::BuckBoost: return "BuckBoost";
    case DCDCTopology::Isolated: return "Isolated";
    default: return "Generic";
  }
}
inline DCDCTopology dcdc_topology_from_str(const std::string& s) {
  const std::string k = enum_parse_key(s);
  if (k == "BUCK") return DCDCTopology::Buck;
  if (k == "BOOST") return DCDCTopology::Boost;
  if (k == "BUCKBOOST" || k == "BUCK_BOOST") return DCDCTopology::BuckBoost;
  if (k == "ISOLATED" || k == "DAB") return DCDCTopology::Isolated;
  return DCDCTopology::Generic;
}

// ── ERPortType ──────────────────────────────────────────────────────────
inline std::string er_port_type_str(ERPortType t) {
  return (t == ERPortType::DC) ? "DC" : "AC";
}
inline ERPortType er_port_type_from_str(const std::string& s) {
  return (s == "DC") ? ERPortType::DC : ERPortType::AC;
}

// ── ERControlMode ───────────────────────────────────────────────────────
inline std::string er_control_str(ERControlMode m) {
  switch (m) {
    case ERControlMode::VF: return "VF";
    case ERControlMode::Droop: return "Droop";
    default: return "PQ";
  }
}
inline ERControlMode er_control_from_str(const std::string& s) {
  if (s == "VF") return ERControlMode::VF;
  if (s == "Droop") return ERControlMode::Droop;
  return ERControlMode::PQ;
}

// ── SwitchType ──────────────────────────────────────────────────────────
inline std::string switch_type_str(SwitchType t) {
  switch (t) {
    case SwitchType::CircuitBreaker: return "CircuitBreaker";
    case SwitchType::Disconnector: return "Disconnector";
    case SwitchType::LoadBreakSwitch: return "LoadBreakSwitch";
    case SwitchType::Fuse: return "Fuse";
    case SwitchType::Recloser: return "Recloser";
    case SwitchType::Sectionalizer: return "Sectionalizer";
    case SwitchType::Unknown: return "Unknown";
    default: return "Unknown";
  }
}
inline SwitchType switch_type_from_str(const std::string& s) {
  if (s == "CircuitBreaker") return SwitchType::CircuitBreaker;
  if (s == "Disconnector") return SwitchType::Disconnector;
  if (s == "LoadBreakSwitch") return SwitchType::LoadBreakSwitch;
  if (s == "Fuse") return SwitchType::Fuse;
  if (s == "Recloser") return SwitchType::Recloser;
  if (s == "Sectionalizer") return SwitchType::Sectionalizer;
  if (s == "Unknown") return SwitchType::Unknown;
  return SwitchType::CircuitBreaker;
}

inline std::string switch_role_str(SwitchRole role) {
  switch (role) {
    case SwitchRole::Protection: return "Protection";
    case SwitchRole::Sectionalizing: return "Sectionalizing";
    case SwitchRole::Tie: return "Tie";
    case SwitchRole::Isolation: return "Isolation";
    case SwitchRole::Grounding: return "Grounding";
    default: return "Unspecified";
  }
}
inline SwitchRole switch_role_from_str(const std::string& s) {
  if (s == "Protection") return SwitchRole::Protection;
  if (s == "Sectionalizing") return SwitchRole::Sectionalizing;
  if (s == "Tie") return SwitchRole::Tie;
  if (s == "Isolation") return SwitchRole::Isolation;
  if (s == "Grounding") return SwitchRole::Grounding;
  return SwitchRole::Unspecified;
}

inline std::string switch_operating_mode_str(SwitchOperatingMode mode) {
  switch (mode) {
    case SwitchOperatingMode::Remote: return "Remote";
    case SwitchOperatingMode::Automatic: return "Automatic";
    default: return "Manual";
  }
}
inline SwitchOperatingMode switch_operating_mode_from_str(const std::string& s) {
  if (s == "Remote") return SwitchOperatingMode::Remote;
  if (s == "Automatic") return SwitchOperatingMode::Automatic;
  return SwitchOperatingMode::Manual;
}

// ── BreakerType ─────────────────────────────────────────────────────────
inline std::string breaker_type_str(BreakerType t) {
  switch (t) {
    case BreakerType::LS: return "LS";
    case BreakerType::DS: return "DS";
    default: return "CB";
  }
}
inline BreakerType breaker_type_from_str(const std::string& s) {
  if (s == "LS") return BreakerType::LS;
  if (s == "DS") return BreakerType::DS;
  return BreakerType::CB;
}

// ── PVControlMode ───────────────────────────────────────────────────────
inline std::string pv_control_str(PVControlMode m) {
  switch (m) {
    case PVControlMode::PQ: return "PQ";
    case PVControlMode::VQ: return "VQ";
    case PVControlMode::Curtailed: return "Curtailed";
    default: return "MPPT";
  }
}
inline PVControlMode pv_control_from_str(const std::string& s) {
  if (s == "PQ") return PVControlMode::PQ;
  if (s == "VQ") return PVControlMode::VQ;
  if (s == "Curtailed") return PVControlMode::Curtailed;
  return PVControlMode::MPPT;
}

// ── MicrogridMode ───────────────────────────────────────────────────────
inline std::string microgrid_mode_str(MicrogridMode m) {
  switch (m) {
    case MicrogridMode::Islanded: return "Islanded";
    case MicrogridMode::Transition: return "Transition";
    default: return "GridConnected";
  }
}
inline MicrogridMode microgrid_mode_from_str(const std::string& s) {
  if (s == "Islanded") return MicrogridMode::Islanded;
  if (s == "Transition") return MicrogridMode::Transition;
  return MicrogridMode::GridConnected;
}

// ── ChargerType ─────────────────────────────────────────────────────────
inline std::string charger_type_str(ChargerType t) {
  switch (t) {
    case ChargerType::AC_L1: return "AC_L1";
    case ChargerType::DC_Fast: return "DC_Fast";
    default: return "AC_L2";
  }
}
inline ChargerType charger_type_from_str(const std::string& s) {
  if (s == "AC_L1") return ChargerType::AC_L1;
  if (s == "DC_Fast") return ChargerType::DC_Fast;
  return ChargerType::AC_L2;
}

// ── MobileStorageStatus ─────────────────────────────────────────────────
inline std::string mobile_storage_status_str(MobileStorageStatus st) {
  switch (st) {
    case MobileStorageStatus::InTransit: return "InTransit";
    case MobileStorageStatus::Deployed: return "Deployed";
    default: return "Stationary";
  }
}
inline MobileStorageStatus mobile_storage_status_from_str(const std::string& s) {
  if (s == "InTransit") return MobileStorageStatus::InTransit;
  if (s == "Deployed") return MobileStorageStatus::Deployed;
  return MobileStorageStatus::Stationary;
}

}  // namespace hacdcpf
