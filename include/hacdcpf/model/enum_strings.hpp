#pragma once

// Shared enum ↔ string conversions for all hacdcpf enum types.
// Used by json_io, etap_io, html_visualizer, and any future serializers.

#include <string>

#include "hacdcpf/model/enums.hpp"

namespace hacdcpf {

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
  }
  return "PQ";
}
inline ConverterMode converter_mode_from_str(const std::string& s) {
  if (s == "PQ" || s == "PQ_MODE" || s == "AC_PQ") return ConverterMode::PQ_MODE;
  if (s == "VDC_Q") return ConverterMode::VDC_Q;
  if (s == "VDC_VAC") return ConverterMode::VDC_VAC;
  return ConverterMode::PQ_MODE;
}

// ── FuelType ────────────────────────────────────────────────────────────
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
    case RenewableType::SolarPV: return "SolarPV";
    case RenewableType::SolarCSP: return "SolarCSP";
    default: return "Wind";
  }
}
inline RenewableType renewable_type_from_str(const std::string& s) {
  if (s == "SolarPV") return RenewableType::SolarPV;
  if (s == "SolarCSP") return RenewableType::SolarCSP;
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
  if (s == "Power") return DCDCControlMode::Power;
  if (s == "Droop") return DCDCControlMode::Droop;
  return DCDCControlMode::Voltage;
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
    case SwitchType::Disconnector: return "Disconnector";
    case SwitchType::LoadBreakSwitch: return "LoadBreakSwitch";
    case SwitchType::Fuse: return "Fuse";
    case SwitchType::Recloser: return "Recloser";
    case SwitchType::Sectionalizer: return "Sectionalizer";
    default: return "CircuitBreaker";
  }
}
inline SwitchType switch_type_from_str(const std::string& s) {
  if (s == "Disconnector") return SwitchType::Disconnector;
  if (s == "LoadBreakSwitch") return SwitchType::LoadBreakSwitch;
  if (s == "Fuse") return SwitchType::Fuse;
  if (s == "Recloser") return SwitchType::Recloser;
  if (s == "Sectionalizer") return SwitchType::Sectionalizer;
  return SwitchType::CircuitBreaker;
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
