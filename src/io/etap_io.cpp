#include "hacdcpf/io/etap_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <queue>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hacdcpf/detail/string_utils.hpp"
#include "hacdcpf/model/enum_strings.hpp"

// OpenXLSX master has defaulted copy/move on non-copyable members; suppress the
// resulting -Wdefaulted-function-deleted diagnostic.
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

using OpenXLSX::XLDocument;
using OpenXLSX::XLWorkbook;
using OpenXLSX::XLWorksheet;

// ── small scalar / cell helpers ─────────────────────────────────────────
std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return s;
}

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return s;
}

std::string bool_str(bool v) { return v ? "TRUE" : "FALSE"; }

bool bool_from_str(const std::string& s, bool default_value = false) {
  const std::string u = to_upper(trim(s));
  if (u == "TRUE" || u == "T" || u == "YES" || u == "Y" || u == "1") return true;
  if (u == "FALSE" || u == "F" || u == "NO" || u == "N" || u == "0") return false;
  return default_value;
}

int int_from_str(const std::string& s, int default_value = 0) {
  try {
    if (trim(s).empty()) return default_value;
    return std::stoi(trim(s));
  } catch (const std::exception&) {
    return default_value;
  }
}

double dbl_from_str(const std::string& s, double default_value = 0.0) {
  try {
    if (trim(s).empty()) return default_value;
    return std::stod(trim(s));
  } catch (const std::exception&) {
    return default_value;
  }
}

std::string cell_to_string(const XLWorksheet& ws, uint32_t row, uint16_t col) {
  auto cell = ws.findCell(row, col);
  if (cell.empty()) return "";
  // OpenXLSX::getString() formats Float cells with std::to_string (6 decimals),
  // which silently truncates small impedance / admittance magnitudes.  Read the
  // stored double directly at full round-trip precision instead; pugixml keeps
  // the value losslessly in the XML on write.
  OpenXLSX::XLCellValue v = cell.value();
  if (v.type() == OpenXLSX::XLValueType::Float) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v.get<double>());
    return std::string(buf);
  }
  return trim(v.getString());
}

using ColMap = std::unordered_map<std::string, uint16_t>;

// Header map keyed by upper-cased header text so look-ups are case-insensitive.
ColMap make_colmap(const XLWorksheet& ws) {
  ColMap map;
  const uint16_t cols = ws.columnCount();
  for (uint16_t c = 1; c <= cols; ++c) {
    const std::string key = cell_to_string(ws, 1, c);
    if (!key.empty()) map.emplace(to_upper(key), c);
  }
  return map;
}

std::string cell_by_name(const XLWorksheet& ws, uint32_t row, const ColMap& col,
                         const std::string& name) {
  auto it = col.find(to_upper(name));
  if (it == col.end()) return "";
  return cell_to_string(ws, row, it->second);
}

void write_headers(XLWorksheet ws, const std::vector<std::string>& headers) {
  for (uint16_t c = 0; c < headers.size(); ++c) {
    ws.cell(1, static_cast<uint16_t>(c + 1)).value() = headers[c];
  }
}

XLWorksheet ensure_sheet(XLWorkbook& wb, const std::string& name) {
  if (!wb.worksheetExists(name)) wb.addWorksheet(name);
  return wb.worksheet(name);
}

// ETAP exports use lower-case sheet names ("bus"); we write upper-case ("BUS").
// Accept any case on import.
std::string find_sheet_name(const XLWorkbook& wb, const std::string& want) {
  XLWorkbook& mwb = const_cast<XLWorkbook&>(wb);
  for (const std::string& cand : {want, to_upper(want), to_lower(want)}) {
    if (mwb.worksheetExists(cand)) return cand;
  }
  return "";
}

// ── per-unit helpers ────────────────────────────────────────────────────────
double z_base(double base_kv, double base_mva) {
  if (base_kv > 0.0 && base_mva > 0.0) return (base_kv * base_kv) / base_mva;
  return 1.0;  // fall back to identity so ohmic columns round-trip per-unit
}

// ── bus identity maps ───────────────────────────────────────────────────────
std::string ac_bus_name(const HybridPowerSystem& sys, int index) {
  for (const auto& b : sys.ac.buses) {
    if (b.index == index) return b.name.empty() ? ("BUS" + std::to_string(index)) : b.name;
  }
  return index == 0 ? "" : ("BUS" + std::to_string(index));
}

std::string dc_bus_name(const HybridPowerSystem& sys, int index) {
  for (const auto& b : sys.dc.buses) {
    if (b.index == index) return b.name.empty() ? ("DCBUS" + std::to_string(index)) : b.name;
  }
  return index == 0 ? "" : ("DCBUS" + std::to_string(index));
}

double ac_bus_base_kv(const HybridPowerSystem& sys, int index) {
  for (const auto& b : sys.ac.buses) {
    if (b.index == index) return b.base_kv;
  }
  return 0.0;
}

double dc_bus_base_kv(const HybridPowerSystem& sys, int index) {
  for (const auto& b : sys.dc.buses) {
    if (b.index == index) return b.base_kv;
  }
  return 0.0;
}

// Resolution map name -> index, populated while reading BUS / DCBUS sheets.
using NameIndex = std::unordered_map<std::string, int>;
using NameKv = std::unordered_map<std::string, double>;

int resolve(const NameIndex& m, const std::string& name) {
  auto it = m.find(to_upper(trim(name)));
  return it == m.end() ? 0 : it->second;
}

double resolve_kv(const NameKv& m, const std::string& name) {
  auto it = m.find(to_upper(trim(name)));
  return it == m.end() ? 0.0 : it->second;
}

void add_count(EtapIoReport& rep, const std::string& sheet, int n) {
  if (n > 0) rep.sheet_counts.emplace_back(sheet, n);
}

// Return the first non-empty cell among a list of candidate column names.  This
// lets a single reader accept both the round-trip schema written by save_etap
// and the raw attribute names emitted by the ETAP toolkit (etap_output.py).
std::string getv(const XLWorksheet& ws, uint32_t r, const ColMap& col,
                 std::initializer_list<const char*> names) {
  for (const char* nm : names) {
    std::string s = cell_by_name(ws, r, col, nm);
    if (!trim(s).empty()) return s;
  }
  return "";
}

// A voltage that may be expressed in pu (~1.0) or ETAP percent (~100).  Values
// above 2 are interpreted as percent and scaled down.
double volt_pu(const std::string& s, double dflt) {
  if (trim(s).empty()) return dflt;
  const double v = dbl_from_str(s, dflt);
  return (v > 2.0) ? v / 100.0 : v;
}

// A power factor that may be a fraction (0.85) or ETAP percent (85).
double pf_fraction(double v) { return (v > 1.0) ? v / 100.0 : v; }

// ETAP exports carry no explicit bus type; derive it from connected sources:
// a utility/external grid pins its bus to SLACK, a generator to PV (unless the
// bus is already SLACK).  Applied after all sheets are read.
void derive_bus_types(HybridPowerSystem& sys) {
  std::unordered_map<int, size_t> pos;
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) pos[sys.ac.buses[i].index] = i;
  for (const auto& g : sys.ac.external_grids) {
    auto it = pos.find(g.bus);
    if (g.in_service && it != pos.end())
      sys.ac.buses[it->second].bus_type = BusType::SLACK;
  }
  for (const auto& g : sys.ac.generators) {
    auto it = pos.find(g.bus);
    if (g.in_service && it != pos.end() &&
        sys.ac.buses[it->second].bus_type != BusType::SLACK)
      sys.ac.buses[it->second].bus_type = BusType::PV;
  }
}

// Validate that every element with a required bus reference resolved (index!=0).
// Circuit breakers are excluded because ETAP connects them by element name, not
// by bus.  In Strict mode a dangling reference throws; in Permissive it warns.
void validate_refs(const HybridPowerSystem& sys, EtapImportMode mode,
                   EtapIoReport& rep) {
  auto check = [&](bool ok, const std::string& what) {
    if (ok) return;
    if (mode == EtapImportMode::Strict)
      throw std::runtime_error("ETAP import: " + what);
    rep.warnings.push_back(what);
  };
  for (const auto& b : sys.ac.branches)
    check(b.from_bus && b.to_bus, "XLINE '" + b.name + "': unresolved bus reference");
  for (const auto& t : sys.ac.transformers_2w)
    check(t.hv_bus && t.lv_bus, "XFORM2W '" + t.name + "': unresolved bus reference");
  for (const auto& t : sys.ac.transformers_3w)
    check(t.hv_bus && t.mv_bus && t.lv_bus, "XFORM3W '" + t.name + "': unresolved bus reference");
  for (const auto& g : sys.ac.external_grids)
    check(g.bus != 0, "UTIL '" + g.name + "': unresolved bus reference");
  for (const auto& g : sys.ac.generators)
    check(g.bus != 0, "SYNGEN '" + g.name + "': unresolved bus reference");
  for (const auto& l : sys.ac.loads)
    check(l.bus != 0, "LUMPEDLOAD '" + l.name + "': unresolved bus reference");
  for (const auto& p : sys.ac.pv_systems)
    check(p.bus != 0, "PVARRAY '" + p.name + "': unresolved bus reference");
  for (const auto& g : sys.ac.renewable_gens)
    check(g.bus != 0, "WIND '" + g.name + "': unresolved bus reference");
  for (const auto& s : sys.ac.shunts)
    check(s.bus != 0, "CAPACITOR '" + s.name + "': unresolved bus reference");
  for (const auto& m : sys.ac.motors)
    check(m.bus != 0, "INDMOTOR '" + m.name + "': unresolved bus reference");
  for (const auto& br : sys.dc.branches)
    check(br.from_bus && br.to_bus, "DCIMPEDANCE '" + br.name + "': unresolved bus reference");
  for (const auto& l : sys.dc.loads)
    check(l.bus != 0, "DCLUMPLOAD '" + l.name + "': unresolved bus reference");
  for (const auto& c : sys.dc.dcdc_converters)
    check(c.bus_in && c.bus_out, "DCCONVERTER '" + c.name + "': unresolved bus reference");
  for (const auto& s : sys.dc.storage)
    check(s.bus != 0, "BATTERY '" + s.name + "': unresolved bus reference");
  for (const auto& v : sys.vsc_converters)
    check(v.bus_ac && v.bus_dc,
          std::string(v.type == "CHARGER" ? "CHARGER '" : "INVERTER '") + v.name +
              "': unresolved bus reference");
}

// ═════════════════════════════════════════════════════════════════════════
// EXPORT  (internal -> ETAP workbook)
// ═════════════════════════════════════════════════════════════════════════
void write_project(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"base_mva", "freq_hz", "name"});
  ws.cell(2, 1).value() = sys.base_mva;
  ws.cell(2, 2).value() = sys.ac.freq_hz;
  ws.cell(2, 3).value() = sys.name;
}

void write_bus(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Type", "NominalkV", "Vm_pu", "Va_deg",
                     "VMaxLimit", "VMinLimit", "Area", "Zone", "InService",
                     "Pd_MW", "Qd_Mvar", "Gs_MW", "Bs_Mvar"});
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& b = sys.ac.buses[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = ac_bus_name(sys, b.index);
    ws.cell(r, 3).value() = bus_type_str(b.bus_type);
    ws.cell(r, 4).value() = b.base_kv;
    ws.cell(r, 5).value() = b.vm_pu;
    ws.cell(r, 6).value() = b.va_deg;
    ws.cell(r, 7).value() = b.vmax_pu;
    ws.cell(r, 8).value() = b.vmin_pu;
    ws.cell(r, 9).value() = b.area;
    ws.cell(r, 10).value() = b.zone;
    ws.cell(r, 11).value() = bool_str(b.in_service);
    ws.cell(r, 12).value() = b.pd_mw;
    ws.cell(r, 13).value() = b.qd_mvar;
    ws.cell(r, 14).value() = b.gs_mw;
    ws.cell(r, 15).value() = b.bs_mvar;
  }
}

void write_xline(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "FromBus", "ToBus", "R_ohm", "X_ohm",
                     "B_S", "Tap", "ShiftDeg", "Rate_MVA", "Length_km",
                     "InService", "FailureRate", "MTTR_hr", "N_parallel"});
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    const double zb = z_base(ac_bus_base_kv(sys, br.from_bus), sys.base_mva);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = br.name.empty() ? ("XLINE" + std::to_string(br.index)) : br.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, br.from_bus);
    ws.cell(r, 4).value() = ac_bus_name(sys, br.to_bus);
    ws.cell(r, 5).value() = br.r_pu * zb;
    ws.cell(r, 6).value() = br.x_pu * zb;
    ws.cell(r, 7).value() = (zb != 0.0) ? (br.b_pu / zb) : 0.0;
    ws.cell(r, 8).value() = br.tap;
    ws.cell(r, 9).value() = br.shift_deg;
    ws.cell(r, 10).value() = br.rate_a_mva;
    ws.cell(r, 11).value() = br.length_km;
    ws.cell(r, 12).value() = bool_str(br.in_service);
    ws.cell(r, 13).value() = br.failure_rate;
    ws.cell(r, 14).value() = br.mttr_hr;
    ws.cell(r, 15).value() = br.n_parallel;
  }
}

void write_xform2w(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "FromBus", "ToBus", "PrimkV", "SeckV",
                     "Sn_MVA", "Z_percent", "ZR_percent", "XR_ratio",
                     "ShiftDeg", "Pk_kW", "InService", "MTBF_hr", "MTTR_hr",
                     "TapSide", "TapPos", "TapMin", "TapMax", "TapNeutral",
                     "TapStepPct", "Z0_percent"});
  for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    double xr = 0.0;
    if (t.vkr_percent > 1e-9) {
      const double x2 = t.vk_percent * t.vk_percent - t.vkr_percent * t.vkr_percent;
      xr = (x2 > 0.0) ? std::sqrt(x2) / t.vkr_percent : 0.0;
    }
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = t.name.empty() ? ("XFORM2W" + std::to_string(t.index)) : t.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, t.hv_bus);
    ws.cell(r, 4).value() = ac_bus_name(sys, t.lv_bus);
    ws.cell(r, 5).value() = t.vn_hv_kv;
    ws.cell(r, 6).value() = t.vn_lv_kv;
    ws.cell(r, 7).value() = t.sn_mva;
    ws.cell(r, 8).value() = t.vk_percent;
    ws.cell(r, 9).value() = t.vkr_percent;
    ws.cell(r, 10).value() = xr;
    ws.cell(r, 11).value() = t.shift_deg;
    ws.cell(r, 12).value() = t.pk_kw;
    ws.cell(r, 13).value() = bool_str(t.in_service);
    ws.cell(r, 14).value() = t.mtbf_hours;
    ws.cell(r, 15).value() = t.mttr_hours;
    ws.cell(r, 16).value() = t.tap_side;
    ws.cell(r, 17).value() = t.tap_pos;
    ws.cell(r, 18).value() = t.tap_min;
    ws.cell(r, 19).value() = t.tap_max;
    ws.cell(r, 20).value() = t.tap_neutral;
    ws.cell(r, 21).value() = t.tap_step_percent;
    ws.cell(r, 22).value() = t.z0_percent;
  }
}

void write_xform3w(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "HVBus", "MVBus", "LVBus", "PrimkV", "SeckV",
                     "TerkV", "Sn_HV_MVA", "Sn_MV_MVA", "Sn_LV_MVA",
                     "Z_HV_MV_pct", "Z_HV_LV_pct", "Z_MV_LV_pct", "InService",
                     "TapSide", "TapPos", "TapStepPct", "ShiftMV_deg",
                     "ShiftLV_deg", "ZR_HV_MV_pct", "ZR_HV_LV_pct",
                     "ZR_MV_LV_pct"});
  for (size_t i = 0; i < sys.ac.transformers_3w.size(); ++i) {
    const auto& t = sys.ac.transformers_3w[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = t.name.empty() ? ("XFORM3W" + std::to_string(t.index)) : t.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, t.hv_bus);
    ws.cell(r, 4).value() = ac_bus_name(sys, t.mv_bus);
    ws.cell(r, 5).value() = ac_bus_name(sys, t.lv_bus);
    ws.cell(r, 6).value() = t.vn_hv_kv;
    ws.cell(r, 7).value() = t.vn_mv_kv;
    ws.cell(r, 8).value() = t.vn_lv_kv;
    ws.cell(r, 9).value() = t.sn_hv_mva;
    ws.cell(r, 10).value() = t.sn_mv_mva;
    ws.cell(r, 11).value() = t.sn_lv_mva;
    ws.cell(r, 12).value() = t.vk_hv_mv_percent;
    ws.cell(r, 13).value() = t.vk_hv_lv_percent;
    ws.cell(r, 14).value() = t.vk_mv_lv_percent;
    ws.cell(r, 15).value() = bool_str(t.in_service);
    ws.cell(r, 16).value() = t.tap_side;
    ws.cell(r, 17).value() = t.tap_pos;
    ws.cell(r, 18).value() = t.tap_step_percent;
    ws.cell(r, 19).value() = t.shift_mv_deg;
    ws.cell(r, 20).value() = t.shift_lv_deg;
    ws.cell(r, 21).value() = t.vkr_hv_mv_percent;
    ws.cell(r, 22).value() = t.vkr_hv_lv_percent;
    ws.cell(r, 23).value() = t.vkr_mv_lv_percent;
  }
}

void write_util(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "KV", "Vm_pu", "Va_deg", "R_pu",
                     "X_pu", "InService", "S_sc_max_MVA", "S_sc_min_MVA",
                     "RX_max", "RX_min", "R0_pu", "X0_pu"});
  for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
    const auto& g = sys.ac.external_grids[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = g.name.empty() ? ("UTIL" + std::to_string(g.index)) : g.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, g.bus);
    ws.cell(r, 4).value() = g.vn_kv;
    ws.cell(r, 5).value() = g.vm_pu;
    ws.cell(r, 6).value() = g.va_deg;
    ws.cell(r, 7).value() = g.r_pu;
    ws.cell(r, 8).value() = g.x_pu;
    ws.cell(r, 9).value() = bool_str(g.in_service);
    ws.cell(r, 10).value() = g.s_sc_max_mva;
    ws.cell(r, 11).value() = g.s_sc_min_mva;
    ws.cell(r, 12).value() = g.rx_max;
    ws.cell(r, 13).value() = g.rx_min;
    ws.cell(r, 14).value() = g.r0_pu;
    ws.cell(r, 15).value() = g.x0_pu;
  }
}

void write_syngen(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "KV", "PG_MW", "QG_Mvar", "Pmax_MW",
                     "Pmin_MW", "Qmax_Mvar", "Qmin_Mvar", "Vg_pu", "MVA",
                     "CosPhi", "IsSlack", "InService", "Cost_c2", "Cost_c1",
                     "Cost_c0", "FailureRate", "MTTR_hr", "Xdpp_pu", "Xdp_pu",
                     "Xd_pu", "Ra_pu", "R0_pu", "X0_pu"});
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    const auto& g = sys.ac.generators[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = g.name.empty() ? ("SYNGEN" + std::to_string(g.index)) : g.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, g.bus);
    ws.cell(r, 4).value() = g.vn_kv;
    ws.cell(r, 5).value() = g.pg_mw;
    ws.cell(r, 6).value() = g.qg_mvar;
    ws.cell(r, 7).value() = g.pmax_mw;
    ws.cell(r, 8).value() = g.pmin_mw;
    ws.cell(r, 9).value() = g.qmax_mvar;
    ws.cell(r, 10).value() = g.qmin_mvar;
    ws.cell(r, 11).value() = g.vg_pu;
    ws.cell(r, 12).value() = g.mbase_mva;
    ws.cell(r, 13).value() = g.cos_phi;
    ws.cell(r, 14).value() = bool_str(g.is_slack);
    ws.cell(r, 15).value() = bool_str(g.in_service);
    ws.cell(r, 16).value() = g.cost_c2;
    ws.cell(r, 17).value() = g.cost_c1;
    ws.cell(r, 18).value() = g.cost_c0;
    ws.cell(r, 19).value() = g.forced_outage_rate;
    ws.cell(r, 20).value() = g.mttr_hr;
    ws.cell(r, 21).value() = g.xdpp_pu;
    ws.cell(r, 22).value() = g.xdp_pu;
    ws.cell(r, 23).value() = g.xd_pu;
    ws.cell(r, 24).value() = g.ra_pu;
    ws.cell(r, 25).value() = g.r0_pu;
    ws.cell(r, 26).value() = g.x0_pu;
  }
}

void write_pvarray(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "P_MW", "Q_Mvar", "Sn_MVA",
                     "Pmax_MW", "Pmin_MW", "Qmax_Mvar", "Qmin_Mvar",
                     "InService"});
  for (size_t i = 0; i < sys.ac.pv_systems.size(); ++i) {
    const auto& p = sys.ac.pv_systems[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = p.name.empty() ? ("PVARRAY" + std::to_string(p.index)) : p.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, p.bus);
    ws.cell(r, 4).value() = p.p_mw;
    ws.cell(r, 5).value() = p.q_mvar;
    ws.cell(r, 6).value() = p.sn_mva;
    ws.cell(r, 7).value() = p.pmax_mw;
    ws.cell(r, 8).value() = p.pmin_mw;
    ws.cell(r, 9).value() = p.qmax_mvar;
    ws.cell(r, 10).value() = p.qmin_mvar;
    ws.cell(r, 11).value() = bool_str(p.in_service);
  }
}

void write_wind(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "Type", "P_MW", "Q_Mvar",
                     "P_rated_MW", "Qmax_Mvar", "Qmin_Mvar", "Curtailable",
                     "CapacityFactor", "CostCurtail_MWh", "InService"});
  for (size_t i = 0; i < sys.ac.renewable_gens.size(); ++i) {
    const auto& g = sys.ac.renewable_gens[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = g.name.empty() ? ("WIND" + std::to_string(g.index)) : g.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, g.bus);
    ws.cell(r, 4).value() = renewable_type_str(g.type);
    ws.cell(r, 5).value() = g.p_mw;
    ws.cell(r, 6).value() = g.q_mvar;
    ws.cell(r, 7).value() = g.p_rated_mw;
    ws.cell(r, 8).value() = g.qmax_mvar;
    ws.cell(r, 9).value() = g.qmin_mvar;
    ws.cell(r, 10).value() = bool_str(g.curtailable);
    ws.cell(r, 11).value() = g.capacity_factor;
    ws.cell(r, 12).value() = g.cost_curtail_mwh;
    ws.cell(r, 13).value() = bool_str(g.in_service);
  }
}

void write_lumpedload(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "KV", "P_MW", "Q_Mvar", "MVA", "PF",
                     "Scaling", "InService", "Model", "Z_pct_P", "I_pct_P",
                     "P_pct_P", "Z_pct_Q", "I_pct_Q", "P_pct_Q", "Priority"});
  for (size_t i = 0; i < sys.ac.loads.size(); ++i) {
    const auto& l = sys.ac.loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    const double s = std::sqrt(l.p_mw * l.p_mw + l.q_mvar * l.q_mvar);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = l.name.empty() ? ("LOAD" + std::to_string(l.index)) : l.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, l.bus);
    ws.cell(r, 4).value() = ac_bus_base_kv(sys, l.bus);
    ws.cell(r, 5).value() = l.p_mw;
    ws.cell(r, 6).value() = l.q_mvar;
    ws.cell(r, 7).value() = s;
    ws.cell(r, 8).value() = (s > 0.0) ? (l.p_mw / s) : 1.0;
    ws.cell(r, 9).value() = l.scaling;
    ws.cell(r, 10).value() = bool_str(l.in_service);
    ws.cell(r, 11).value() = load_model_str(l.model);
    ws.cell(r, 12).value() = l.z_percent_p;
    ws.cell(r, 13).value() = l.i_percent_p;
    ws.cell(r, 14).value() = l.p_percent_p;
    ws.cell(r, 15).value() = l.z_percent_q;
    ws.cell(r, 16).value() = l.i_percent_q;
    ws.cell(r, 17).value() = l.p_percent_q;
    ws.cell(r, 18).value() = load_priority_str(l.priority);
  }
}

void write_capacitor(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "Gs_MW", "Bs_Mvar", "InService"});
  for (size_t i = 0; i < sys.ac.shunts.size(); ++i) {
    const auto& sh = sys.ac.shunts[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = sh.name.empty() ? ("CAP" + std::to_string(sh.index)) : sh.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, sh.bus);
    ws.cell(r, 4).value() = sh.gs_mw;
    ws.cell(r, 5).value() = sh.bs_mvar;
    ws.cell(r, 6).value() = bool_str(sh.in_service);
  }
}

void write_hvcb(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "FromBus", "ToBus", "Closed", "RatedKV",
                     "InService", "I_rated_kA", "I_breaking_kA"});
  for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
    const auto& cb = sys.ac.circuit_breakers[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = cb.name.empty() ? ("HVCB" + std::to_string(cb.index)) : cb.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, cb.bus_from);
    ws.cell(r, 4).value() = ac_bus_name(sys, cb.bus_to);
    ws.cell(r, 5).value() = bool_str(cb.closed);
    ws.cell(r, 6).value() = cb.rated_voltage_kv;
    ws.cell(r, 7).value() = bool_str(cb.in_service);
    ws.cell(r, 8).value() = cb.i_rated_ka;
    ws.cell(r, 9).value() = cb.i_breaking_ka;
  }
}

void write_indmotor(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "KV", "MVA", "R_pu", "X_pu",
                     "InService"});
  for (size_t i = 0; i < sys.ac.motors.size(); ++i) {
    const auto& m = sys.ac.motors[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = m.name.empty() ? ("INDMOTOR" + std::to_string(m.index)) : m.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, m.bus);
    ws.cell(r, 4).value() = m.vn_kv;
    ws.cell(r, 5).value() = m.sn_mva;
    ws.cell(r, 6).value() = m.r_pu;
    ws.cell(r, 7).value() = m.x_pu;
    ws.cell(r, 8).value() = bool_str(m.in_service);
  }
}

void write_dcbus(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Type", "NominalV", "Vm_pu", "VMaxLimit",
                     "VMinLimit", "Area", "Zone", "InService", "Pd_MW"});
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    const auto& b = sys.dc.buses[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = dc_bus_name(sys, b.index);
    ws.cell(r, 3).value() = dc_bus_type_str(b.bus_type);
    ws.cell(r, 4).value() = b.base_kv * 1000.0;  // volts
    ws.cell(r, 5).value() = b.vm_pu;
    ws.cell(r, 6).value() = b.vmax_pu;
    ws.cell(r, 7).value() = b.vmin_pu;
    ws.cell(r, 8).value() = b.area;
    ws.cell(r, 9).value() = b.zone;
    ws.cell(r, 10).value() = bool_str(b.in_service);
    ws.cell(r, 11).value() = b.pd_mw;
  }
}

void write_dcimpedance(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "FromBus", "ToBus", "R_ohm", "Rate_MVA",
                     "Length_km", "InService"});
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    const double zb = z_base(dc_bus_base_kv(sys, br.from_bus), sys.base_mva);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = br.name.empty() ? ("DCIMP" + std::to_string(br.index)) : br.name;
    ws.cell(r, 3).value() = dc_bus_name(sys, br.from_bus);
    ws.cell(r, 4).value() = dc_bus_name(sys, br.to_bus);
    ws.cell(r, 5).value() = br.r_pu * zb;
    ws.cell(r, 6).value() = br.rate_a_mva;
    ws.cell(r, 7).value() = br.length_km;
    ws.cell(r, 8).value() = bool_str(br.in_service);
  }
}

void write_dclumpload(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "DCV", "KW", "Scaling", "InService"});
  for (size_t i = 0; i < sys.dc.loads.size(); ++i) {
    const auto& l = sys.dc.loads[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = l.name.empty() ? ("DCLOAD" + std::to_string(l.index)) : l.name;
    ws.cell(r, 3).value() = dc_bus_name(sys, l.bus);
    ws.cell(r, 4).value() = dc_bus_base_kv(sys, l.bus) * 1000.0;  // volts
    ws.cell(r, 5).value() = l.p_mw * 1000.0;                      // kW
    ws.cell(r, 6).value() = l.scaling;
    ws.cell(r, 7).value() = bool_str(l.in_service);
  }
}

void write_dcconverter(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "InputBus", "OutputBus", "KW", "PercentEFF",
                     "Vin_kV", "Vout_kV", "Pmax_MW", "Pmin_MW", "InService"});
  for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
    const auto& c = sys.dc.dcdc_converters[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = c.name.empty() ? ("DCDC" + std::to_string(c.index)) : c.name;
    ws.cell(r, 3).value() = dc_bus_name(sys, c.bus_in);
    ws.cell(r, 4).value() = dc_bus_name(sys, c.bus_out);
    ws.cell(r, 5).value() = c.p_ref_mw * 1000.0;  // kW
    ws.cell(r, 6).value() = c.eta * 100.0;
    ws.cell(r, 7).value() = c.vn_in_kv;
    ws.cell(r, 8).value() = c.vn_out_kv;
    ws.cell(r, 9).value() = c.pmax_mw;
    ws.cell(r, 10).value() = c.pmin_mw;
    ws.cell(r, 11).value() = bool_str(c.in_service);
  }
}

void write_dccb(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "FromBus", "ToBus", "Closed", "RatedKV",
                     "InService", "I_breaking_kA", "R_ohm"});
  for (size_t i = 0; i < sys.dc.dc_circuit_breakers.size(); ++i) {
    const auto& cb = sys.dc.dc_circuit_breakers[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = cb.name.empty() ? ("DCCB" + std::to_string(cb.index)) : cb.name;
    ws.cell(r, 3).value() = dc_bus_name(sys, cb.bus_from);
    ws.cell(r, 4).value() = dc_bus_name(sys, cb.bus_to);
    ws.cell(r, 5).value() = bool_str(cb.closed);
    ws.cell(r, 6).value() = cb.rated_voltage_kv;
    ws.cell(r, 7).value() = bool_str(cb.in_service);
    ws.cell(r, 8).value() = cb.i_breaking_ka;
    ws.cell(r, 9).value() = cb.r_ohm;
  }
}

void write_converter_sheet(XLWorksheet ws, const HybridPowerSystem& sys,
                           bool want_charger) {
  write_headers(ws, {"Index", "ID", "ACBus", "DCBus", "Mode", "P_MW", "Q_Mvar",
                     "Vdc_set_pu", "Vac_set_pu", "Pmax_MW", "Pmin_MW",
                     "Qmax_Mvar", "Qmin_Mvar", "PercentEFF", "Vac_kV", "Vdc_kV",
                     "InService"});
  uint32_t r = 2;
  int n = 1;
  for (const auto& v : sys.vsc_converters) {
    const bool is_charger = (to_upper(v.type) == "CHARGER");
    if (is_charger != want_charger) continue;
    const std::string prefix = want_charger ? "CHARGER" : "INVERTER";
    ws.cell(r, 1).value() = n++;
    ws.cell(r, 2).value() = v.name.empty() ? (prefix + std::to_string(v.index)) : v.name;
    ws.cell(r, 3).value() = ac_bus_name(sys, v.bus_ac);
    ws.cell(r, 4).value() = dc_bus_name(sys, v.bus_dc);
    ws.cell(r, 5).value() = converter_mode_str(v.control_mode);
    ws.cell(r, 6).value() = v.p_set_mw;
    ws.cell(r, 7).value() = v.q_set_mvar;
    ws.cell(r, 8).value() = v.v_dc_set_pu;
    ws.cell(r, 9).value() = v.v_ac_set_pu;
    ws.cell(r, 10).value() = v.pmax_mw;
    ws.cell(r, 11).value() = v.pmin_mw;
    ws.cell(r, 12).value() = v.qmax_mvar;
    ws.cell(r, 13).value() = v.qmin_mvar;
    ws.cell(r, 14).value() = v.eta * 100.0;
    ws.cell(r, 15).value() = v.vn_ac_kv;
    ws.cell(r, 16).value() = v.vn_dc_kv;
    ws.cell(r, 17).value() = bool_str(v.in_service);
    ++r;
  }
}

void write_battery(XLWorksheet ws, const HybridPowerSystem& sys) {
  write_headers(ws, {"Index", "ID", "Bus", "P_MW", "Pmax_MW", "Pmin_MW",
                     "E_MWh", "SoC", "SoCMin", "SoCMax", "EtaCh", "EtaDis",
                     "InService"});
  for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
    const auto& s = sys.dc.storage[i];
    const uint32_t r = static_cast<uint32_t>(i + 2);
    ws.cell(r, 1).value() = static_cast<int>(i + 1);
    ws.cell(r, 2).value() = s.name.empty() ? ("BATTERY" + std::to_string(s.index)) : s.name;
    ws.cell(r, 3).value() = dc_bus_name(sys, s.bus);
    ws.cell(r, 4).value() = s.p_mw;
    ws.cell(r, 5).value() = s.pmax_mw;
    ws.cell(r, 6).value() = s.pmin_mw;
    ws.cell(r, 7).value() = s.e_rated_mwh;
    ws.cell(r, 8).value() = s.soc_init;
    ws.cell(r, 9).value() = s.soc_min;
    ws.cell(r, 10).value() = s.soc_max;
    ws.cell(r, 11).value() = s.eta_charge;
    ws.cell(r, 12).value() = s.eta_discharge;
    ws.cell(r, 13).value() = bool_str(s.in_service);
  }
}

// ═════════════════════════════════════════════════════════════════════════
// IMPORT  (ETAP workbook -> internal)
// ═════════════════════════════════════════════════════════════════════════
void read_project(const XLWorksheet& ws, HybridPowerSystem& sys) {
  const auto col = make_colmap(ws);
  const std::string bm = cell_by_name(ws, 2, col, "base_mva");
  if (!bm.empty()) {
    sys.base_mva = dbl_from_str(bm, sys.base_mva);
    sys.ac.base_mva = sys.base_mva;
    sys.dc.base_mva = sys.base_mva;
  }
  const std::string fr = cell_by_name(ws, 2, col, "freq_hz");
  if (!fr.empty()) sys.ac.freq_hz = dbl_from_str(fr, sys.ac.freq_hz);
  const std::string nm = cell_by_name(ws, 2, col, "name");
  if (!nm.empty()) sys.name = nm;
}

void read_bus(const XLWorksheet& ws, HybridPowerSystem& sys, NameIndex& name2idx,
              NameKv& name2kv, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    ACBus b;
    b.index = ++idx;
    b.name = id;
    b.bus_type = bus_type_from_str(cell_by_name(ws, r, col, "Type"));
    const double nk = dbl_from_str(getv(ws, r, col, {"NominalkV", "NominalKV"}));
    const double bk = dbl_from_str(getv(ws, r, col, {"BasekV", "BaseKV", "base_kv"}));
    // Use 0 (not the ACBus default of 110 kV) when no base voltage is given so
    // that per-unit <-> ohm impedance conversion uses the same Z_base on import
    // and export (both fall back to the 1.0 identity base).
    b.base_kv = nk > 0.0 ? nk : bk;
    b.vm_pu = volt_pu(getv(ws, r, col, {"Vm_pu", "OpVMag", "VMag"}), b.vm_pu);
    b.va_deg = dbl_from_str(getv(ws, r, col, {"Va_deg", "OpVAng"}), b.va_deg);
    b.vmax_pu = volt_pu(getv(ws, r, col, {"VMaxLimit", "vmax_pu"}), b.vmax_pu);
    b.vmin_pu = volt_pu(getv(ws, r, col, {"VMinLimit", "vmin_pu"}), b.vmin_pu);
    b.area = int_from_str(cell_by_name(ws, r, col, "Area"), b.area);
    b.zone = int_from_str(cell_by_name(ws, r, col, "Zone"), b.zone);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), b.in_service);
    b.pd_mw = dbl_from_str(cell_by_name(ws, r, col, "Pd_MW"), b.pd_mw);
    b.qd_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qd_Mvar"), b.qd_mvar);
    b.gs_mw = dbl_from_str(cell_by_name(ws, r, col, "Gs_MW"), b.gs_mw);
    b.bs_mvar = dbl_from_str(cell_by_name(ws, r, col, "Bs_Mvar"), b.bs_mvar);
    name2idx[to_upper(trim(id))] = b.index;
    name2kv[to_upper(trim(id))] = b.base_kv;
    sys.ac.buses.push_back(std::move(b));
    ++n;
  }
  add_count(rep, "BUS", n);
}

void read_branch_sheet(const XLWorksheet& ws, HybridPowerSystem& sys,
                       const NameIndex& name2idx, const NameKv& name2kv,
                       double base_mva, EtapIoReport& rep,
                       const std::string& sheet) {
  const auto col = make_colmap(ws);
  int idx = static_cast<int>(sys.ac.branches.size());
  int n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    ACBranch br;
    br.index = ++idx;
    br.name = id;
    const std::string fb = cell_by_name(ws, r, col, "FromBus");
    const std::string tb = cell_by_name(ws, r, col, "ToBus");
    br.from_bus = resolve(name2idx, fb);
    br.to_bus = resolve(name2idx, tb);
    const double zb = z_base(resolve_kv(name2kv, fb), base_mva);
    br.r_pu = dbl_from_str(getv(ws, r, col, {"R_ohm", "RPos", "RPosValue"})) / zb;
    br.x_pu = dbl_from_str(getv(ws, r, col, {"X_ohm", "XPos", "XPosValue"})) / zb;
    br.b_pu = dbl_from_str(getv(ws, r, col, {"B_S", "YPos", "YPosValue"})) * zb;
    br.tap = dbl_from_str(cell_by_name(ws, r, col, "Tap"), 1.0);
    br.shift_deg = dbl_from_str(cell_by_name(ws, r, col, "ShiftDeg"));
    br.rate_a_mva = dbl_from_str(getv(ws, r, col, {"Rate_MVA", "RatedA"}));
    br.length_km = dbl_from_str(getv(ws, r, col, {"Length_km", "Length", "LengthValue"}));
    br.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    br.failure_rate = dbl_from_str(cell_by_name(ws, r, col, "FailureRate"), br.failure_rate);
    br.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "MTTR_hr"), br.mttr_hr);
    br.n_parallel = int_from_str(cell_by_name(ws, r, col, "N_parallel"), br.n_parallel);
    sys.ac.branches.push_back(std::move(br));
    ++n;
  }
  add_count(rep, sheet, n);
}

void read_xform2w(const XLWorksheet& ws, HybridPowerSystem& sys,
                  const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Transformer2W t;
    t.index = ++idx;
    t.name = id;
    t.hv_bus = resolve(name2idx, getv(ws, r, col, {"FromBus", "PrimaryBus", "HVBus"}));
    t.lv_bus = resolve(name2idx, getv(ws, r, col, {"ToBus", "SecondaryBus", "LVBus"}));
    t.vn_hv_kv = dbl_from_str(getv(ws, r, col, {"PrimkV", "PrimKV"}));
    t.vn_lv_kv = dbl_from_str(getv(ws, r, col, {"SeckV", "SecKV"}));
    const std::string sn = getv(ws, r, col, {"Sn_MVA"});
    if (!sn.empty())
      t.sn_mva = dbl_from_str(sn);
    else
      t.sn_mva = dbl_from_str(getv(ws, r, col, {"ZBaseMVA", "AnsiMVA"})) / 1000.0;  // kVA -> MVA
    t.vk_percent = dbl_from_str(getv(ws, r, col, {"Z_percent", "AnsiPosZ"}));
    t.vkr_percent = dbl_from_str(getv(ws, r, col, {"ZR_percent", "PosR"}));
    t.shift_deg = dbl_from_str(cell_by_name(ws, r, col, "ShiftDeg"));
    t.pk_kw = dbl_from_str(cell_by_name(ws, r, col, "Pk_kW"));
    t.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    t.mtbf_hours = dbl_from_str(cell_by_name(ws, r, col, "MTBF_hr"), t.mtbf_hours);
    t.mttr_hours = dbl_from_str(cell_by_name(ws, r, col, "MTTR_hr"), t.mttr_hours);
    t.tap_side = int_from_str(cell_by_name(ws, r, col, "TapSide"), t.tap_side);
    t.tap_pos = int_from_str(cell_by_name(ws, r, col, "TapPos"), t.tap_pos);
    t.tap_min = int_from_str(cell_by_name(ws, r, col, "TapMin"), t.tap_min);
    t.tap_max = int_from_str(cell_by_name(ws, r, col, "TapMax"), t.tap_max);
    t.tap_neutral = int_from_str(cell_by_name(ws, r, col, "TapNeutral"), t.tap_neutral);
    t.tap_step_percent = dbl_from_str(cell_by_name(ws, r, col, "TapStepPct"), t.tap_step_percent);
    t.z0_percent = dbl_from_str(getv(ws, r, col, {"Z0_percent", "AnsiZeroZ"}), t.z0_percent);
    sys.ac.transformers_2w.push_back(std::move(t));
    ++n;
  }
  add_count(rep, "XFORM2W", n);
}

void read_xform3w(const XLWorksheet& ws, HybridPowerSystem& sys,
                  const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Transformer3W t;
    t.index = ++idx;
    t.name = id;
    t.hv_bus = resolve(name2idx, getv(ws, r, col, {"HVBus", "PrimaryBus", "FromBus"}));
    t.mv_bus = resolve(name2idx, getv(ws, r, col, {"MVBus", "SecondaryBus"}));
    t.lv_bus = resolve(name2idx, getv(ws, r, col, {"LVBus", "TeritiaryBus", "TertiaryBus"}));
    t.vn_hv_kv = dbl_from_str(getv(ws, r, col, {"PrimkV", "PrimKV"}));
    t.vn_mv_kv = dbl_from_str(getv(ws, r, col, {"SeckV", "SecKV"}));
    t.vn_lv_kv = dbl_from_str(getv(ws, r, col, {"TerkV", "TerKV"}));
    // ETAP ratings are in kVA; the round-trip schema uses MVA.
    auto mva = [&](std::initializer_list<const char*> mva_names,
                   std::initializer_list<const char*> kva_names) {
      const std::string m = getv(ws, r, col, mva_names);
      if (!m.empty()) return dbl_from_str(m);
      return dbl_from_str(getv(ws, r, col, kva_names)) / 1000.0;
    };
    t.sn_hv_mva = mva({"Sn_HV_MVA"}, {"PrimkVA", "PrimKVA"});
    t.sn_mv_mva = mva({"Sn_MV_MVA"}, {"SeckVA", "SecKVA"});
    t.sn_lv_mva = mva({"Sn_LV_MVA"}, {"TerkVA", "TerKVA"});
    t.vk_hv_mv_percent = dbl_from_str(getv(ws, r, col, {"Z_HV_MV_pct", "PSPosZ"}));
    t.vk_hv_lv_percent = dbl_from_str(getv(ws, r, col, {"Z_HV_LV_pct", "PTPosZ"}));
    t.vk_mv_lv_percent = dbl_from_str(getv(ws, r, col, {"Z_MV_LV_pct", "STPosZ"}));
    t.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    t.tap_side = int_from_str(cell_by_name(ws, r, col, "TapSide"), t.tap_side);
    t.tap_pos = int_from_str(cell_by_name(ws, r, col, "TapPos"), t.tap_pos);
    t.tap_step_percent = dbl_from_str(cell_by_name(ws, r, col, "TapStepPct"), t.tap_step_percent);
    t.shift_mv_deg = dbl_from_str(cell_by_name(ws, r, col, "ShiftMV_deg"), t.shift_mv_deg);
    t.shift_lv_deg = dbl_from_str(cell_by_name(ws, r, col, "ShiftLV_deg"), t.shift_lv_deg);
    t.vkr_hv_mv_percent = dbl_from_str(getv(ws, r, col, {"ZR_HV_MV_pct", "PSPosR"}), t.vkr_hv_mv_percent);
    t.vkr_hv_lv_percent = dbl_from_str(getv(ws, r, col, {"ZR_HV_LV_pct", "PTPosR"}), t.vkr_hv_lv_percent);
    t.vkr_mv_lv_percent = dbl_from_str(getv(ws, r, col, {"ZR_MV_LV_pct", "STPosR"}), t.vkr_mv_lv_percent);
    sys.ac.transformers_3w.push_back(std::move(t));
    ++n;
  }
  add_count(rep, "XFORM3W", n);
}

// ETAP motor-generator set: ingested best-effort as a synchronous generator.
// The ETAP export carries no bus attribute, so rows without a resolvable bus
// are skipped (a generator with no bus is meaningless).
void read_mgset(const XLWorksheet& ws, HybridPowerSystem& sys,
                const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int n = 0;
  int idx = static_cast<int>(sys.ac.generators.size());
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    const int bus = resolve(name2idx, getv(ws, r, col, {"Bus"}));
    if (bus == 0) {
      rep.warnings.push_back("MGSET '" + id + "': no resolvable bus, skipped");
      continue;
    }
    Generator g;
    g.index = ++idx;
    g.name = id;
    g.bus = bus;
    g.vn_kv = dbl_from_str(getv(ws, r, col, {"KV"}));
    g.mbase_mva = dbl_from_str(getv(ws, r, col, {"MVA"}));
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.generators.push_back(std::move(g));
    ++n;
  }
  add_count(rep, "MGSET", n);
}

void read_util(const XLWorksheet& ws, HybridPowerSystem& sys,
               const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    ExternalGrid g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    g.vn_kv = dbl_from_str(cell_by_name(ws, r, col, "KV"));
    g.vm_pu = volt_pu(getv(ws, r, col, {"Vm_pu", "VoltageMagnitude", "OpVMag"}), g.vm_pu);
    g.va_deg = dbl_from_str(getv(ws, r, col, {"Va_deg", "VoltageAngle", "OpVAng"}));
    g.r_pu = dbl_from_str(getv(ws, r, col, {"R_pu", "PosR"}));
    g.x_pu = dbl_from_str(getv(ws, r, col, {"X_pu", "PosX"}));
    g.s_sc_max_mva = dbl_from_str(getv(ws, r, col, {"S_sc_max_MVA"}), g.s_sc_max_mva);
    g.s_sc_min_mva = dbl_from_str(getv(ws, r, col, {"S_sc_min_MVA"}), g.s_sc_min_mva);
    g.rx_max = dbl_from_str(getv(ws, r, col, {"RX_max"}), g.rx_max);
    g.rx_min = dbl_from_str(getv(ws, r, col, {"RX_min"}), g.rx_min);
    g.r0_pu = dbl_from_str(getv(ws, r, col, {"R0_pu", "ZeroR"}), g.r0_pu);
    g.x0_pu = dbl_from_str(getv(ws, r, col, {"X0_pu", "ZeroX"}), g.x0_pu);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.external_grids.push_back(std::move(g));
    ++n;
  }
  add_count(rep, "UTIL", n);
}

void read_syngen(const XLWorksheet& ws, HybridPowerSystem& sys,
                 const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Generator g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    g.vn_kv = dbl_from_str(cell_by_name(ws, r, col, "KV"));
    g.pg_mw = dbl_from_str(cell_by_name(ws, r, col, "PG_MW"));
    g.qg_mvar = dbl_from_str(cell_by_name(ws, r, col, "QG_Mvar"));
    g.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmax_MW"));
    g.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmin_MW"));
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmax_Mvar"));
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmin_Mvar"));
    g.vg_pu = dbl_from_str(cell_by_name(ws, r, col, "Vg_pu"), g.vg_pu);
    g.mbase_mva = dbl_from_str(cell_by_name(ws, r, col, "MVA"));
    g.cos_phi = dbl_from_str(cell_by_name(ws, r, col, "CosPhi"), g.cos_phi);
    g.is_slack = bool_from_str(cell_by_name(ws, r, col, "IsSlack"), false);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    g.cost_c2 = dbl_from_str(cell_by_name(ws, r, col, "Cost_c2"), g.cost_c2);
    g.cost_c1 = dbl_from_str(cell_by_name(ws, r, col, "Cost_c1"), g.cost_c1);
    g.cost_c0 = dbl_from_str(cell_by_name(ws, r, col, "Cost_c0"), g.cost_c0);
    g.forced_outage_rate = dbl_from_str(cell_by_name(ws, r, col, "FailureRate"), g.forced_outage_rate);
    g.mttr_hr = dbl_from_str(cell_by_name(ws, r, col, "MTTR_hr"), g.mttr_hr);
    g.xdpp_pu = dbl_from_str(getv(ws, r, col, {"Xdpp_pu", "Xdsat"}), g.xdpp_pu);
    g.xdp_pu = dbl_from_str(cell_by_name(ws, r, col, "Xdp_pu"), g.xdp_pu);
    g.xd_pu = dbl_from_str(cell_by_name(ws, r, col, "Xd_pu"), g.xd_pu);
    g.ra_pu = dbl_from_str(cell_by_name(ws, r, col, "Ra_pu"), g.ra_pu);
    g.r0_pu = dbl_from_str(cell_by_name(ws, r, col, "R0_pu"), g.r0_pu);
    g.x0_pu = dbl_from_str(cell_by_name(ws, r, col, "X0_pu"), g.x0_pu);
    sys.ac.generators.push_back(std::move(g));
    ++n;
  }
  add_count(rep, "SYNGEN", n);
}

void read_pvarray(const XLWorksheet& ws, HybridPowerSystem& sys,
                  const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    PVSystem p;
    p.index = ++idx;
    p.name = id;
    p.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    p.p_mw = dbl_from_str(cell_by_name(ws, r, col, "P_MW"));
    p.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "Q_Mvar"));
    p.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "Sn_MVA"));
    p.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmax_MW"));
    p.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmin_MW"));
    p.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmax_Mvar"));
    p.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmin_Mvar"));
    p.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.pv_systems.push_back(std::move(p));
    ++n;
  }
  add_count(rep, "PVARRAY", n);
}

void read_wind(const XLWorksheet& ws, HybridPowerSystem& sys,
               const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    RenewableGen g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    g.type = renewable_type_from_str(cell_by_name(ws, r, col, "Type"));
    g.p_mw = dbl_from_str(cell_by_name(ws, r, col, "P_MW"));
    g.q_mvar = dbl_from_str(cell_by_name(ws, r, col, "Q_Mvar"));
    g.p_rated_mw = dbl_from_str(cell_by_name(ws, r, col, "P_rated_MW"));
    g.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmax_Mvar"));
    g.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmin_Mvar"));
    g.curtailable = bool_from_str(cell_by_name(ws, r, col, "Curtailable"), g.curtailable);
    g.capacity_factor = dbl_from_str(cell_by_name(ws, r, col, "CapacityFactor"), g.capacity_factor);
    g.cost_curtail_mwh = dbl_from_str(cell_by_name(ws, r, col, "CostCurtail_MWh"), g.cost_curtail_mwh);
    g.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.renewable_gens.push_back(std::move(g));
    ++n;
  }
  add_count(rep, "WIND", n);
}

void read_lumpedload(const XLWorksheet& ws, HybridPowerSystem& sys,
                     const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Load l;
    l.index = ++idx;
    l.name = id;
    l.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    l.p_mw = dbl_from_str(getv(ws, r, col, {"P_MW", "OpMW"}));
    l.q_mvar = dbl_from_str(getv(ws, r, col, {"Q_Mvar", "OpMvar"}));
    // ETAP exports rated load as MVA + PF; fall back to it when the operating
    // P/Q snapshot is absent (no load-flow study was run).
    if (std::abs(l.p_mw) < 1e-12 && std::abs(l.q_mvar) < 1e-12) {
      const double mva = dbl_from_str(getv(ws, r, col, {"MVA"}));
      if (mva > 0.0) {
        const double pf = pf_fraction(dbl_from_str(getv(ws, r, col, {"PF"}), 100.0));
        l.p_mw = mva * pf;
        l.q_mvar = mva * std::sqrt(std::max(0.0, 1.0 - pf * pf));
      }
    }
    l.scaling = dbl_from_str(cell_by_name(ws, r, col, "Scaling"), 1.0);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    l.model = load_model_from_str(cell_by_name(ws, r, col, "Model"));
    l.z_percent_p = dbl_from_str(cell_by_name(ws, r, col, "Z_pct_P"), l.z_percent_p);
    l.i_percent_p = dbl_from_str(cell_by_name(ws, r, col, "I_pct_P"), l.i_percent_p);
    l.p_percent_p = dbl_from_str(cell_by_name(ws, r, col, "P_pct_P"), l.p_percent_p);
    l.z_percent_q = dbl_from_str(cell_by_name(ws, r, col, "Z_pct_Q"), l.z_percent_q);
    l.i_percent_q = dbl_from_str(cell_by_name(ws, r, col, "I_pct_Q"), l.i_percent_q);
    l.p_percent_q = dbl_from_str(cell_by_name(ws, r, col, "P_pct_Q"), l.p_percent_q);
    l.priority = load_priority_from_str(cell_by_name(ws, r, col, "Priority"));
    sys.ac.loads.push_back(std::move(l));
    ++n;
  }
  add_count(rep, "LUMPEDLOAD", n);
}

void read_capacitor(const XLWorksheet& ws, HybridPowerSystem& sys,
                    const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Shunt sh;
    sh.index = ++idx;
    sh.name = id;
    sh.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    sh.gs_mw = dbl_from_str(cell_by_name(ws, r, col, "Gs_MW"));
    sh.bs_mvar = dbl_from_str(cell_by_name(ws, r, col, "Bs_Mvar"));
    sh.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.shunts.push_back(std::move(sh));
    ++n;
  }
  add_count(rep, "CAPACITOR", n);
}

void read_hvcb(const XLWorksheet& ws, HybridPowerSystem& sys,
               const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    CircuitBreaker cb;
    cb.index = ++idx;
    cb.name = id;
    cb.bus_from = resolve(name2idx, cell_by_name(ws, r, col, "FromBus"));
    cb.bus_to = resolve(name2idx, cell_by_name(ws, r, col, "ToBus"));
    cb.closed = bool_from_str(cell_by_name(ws, r, col, "Closed"), true);
    cb.rated_voltage_kv = dbl_from_str(cell_by_name(ws, r, col, "RatedKV"));
    cb.i_rated_ka = dbl_from_str(getv(ws, r, col, {"I_rated_kA", "Rated"}), cb.i_rated_ka);
    cb.i_breaking_ka = dbl_from_str(getv(ws, r, col, {"I_breaking_kA", "Interrupting"}), cb.i_breaking_ka);
    cb.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.circuit_breakers.push_back(std::move(cb));
    ++n;
  }
  add_count(rep, "HVCB", n);
}

void read_indmotor(const XLWorksheet& ws, HybridPowerSystem& sys,
                   const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    AsynchronousMotor m;
    m.index = ++idx;
    m.name = id;
    m.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    m.vn_kv = dbl_from_str(cell_by_name(ws, r, col, "KV"));
    m.sn_mva = dbl_from_str(cell_by_name(ws, r, col, "MVA"));
    m.r_pu = dbl_from_str(cell_by_name(ws, r, col, "R_pu"));
    m.x_pu = dbl_from_str(cell_by_name(ws, r, col, "X_pu"));
    m.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.ac.motors.push_back(std::move(m));
    ++n;
  }
  add_count(rep, "INDMOTOR", n);
}

void read_dcbus(const XLWorksheet& ws, HybridPowerSystem& sys,
                NameIndex& name2idx, NameKv& name2kv, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    DCBus b;
    b.index = ++idx;
    b.name = id;
    b.bus_type = dc_bus_type_from_str(cell_by_name(ws, r, col, "Type"));
    b.base_kv = dbl_from_str(cell_by_name(ws, r, col, "NominalV")) / 1000.0;  // V -> kV
    b.vm_pu = dbl_from_str(cell_by_name(ws, r, col, "Vm_pu"), b.vm_pu);
    b.vmax_pu = dbl_from_str(cell_by_name(ws, r, col, "VMaxLimit"), b.vmax_pu);
    b.vmin_pu = dbl_from_str(cell_by_name(ws, r, col, "VMinLimit"), b.vmin_pu);
    b.area = int_from_str(cell_by_name(ws, r, col, "Area"), b.area);
    b.zone = int_from_str(cell_by_name(ws, r, col, "Zone"), b.zone);
    b.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), b.in_service);
    b.pd_mw = dbl_from_str(cell_by_name(ws, r, col, "Pd_MW"), b.pd_mw);
    name2idx[to_upper(trim(id))] = b.index;
    name2kv[to_upper(trim(id))] = b.base_kv;
    sys.dc.buses.push_back(std::move(b));
    ++n;
  }
  add_count(rep, "DCBUS", n);
}

void read_dcimpedance(const XLWorksheet& ws, HybridPowerSystem& sys,
                      const NameIndex& name2idx, const NameKv& name2kv,
                      double base_mva, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    DCBranch br;
    br.index = ++idx;
    br.name = id;
    const std::string fb = cell_by_name(ws, r, col, "FromBus");
    br.from_bus = resolve(name2idx, fb);
    br.to_bus = resolve(name2idx, cell_by_name(ws, r, col, "ToBus"));
    const double zb = z_base(resolve_kv(name2kv, fb), base_mva);
    br.r_pu = dbl_from_str(getv(ws, r, col, {"R_ohm", "RValue"})) / zb;
    br.rate_a_mva = dbl_from_str(cell_by_name(ws, r, col, "Rate_MVA"));
    br.length_km = dbl_from_str(cell_by_name(ws, r, col, "Length_km"));
    br.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.dc.branches.push_back(std::move(br));
    ++n;
  }
  add_count(rep, "DCIMPEDANCE", n);
}

void read_dclumpload(const XLWorksheet& ws, HybridPowerSystem& sys,
                     const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    DCLoad l;
    l.index = ++idx;
    l.name = id;
    l.bus = resolve(name2idx, cell_by_name(ws, r, col, "Bus"));
    l.p_mw = dbl_from_str(cell_by_name(ws, r, col, "KW")) / 1000.0;  // kW -> MW
    l.scaling = dbl_from_str(cell_by_name(ws, r, col, "Scaling"), 1.0);
    l.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.dc.loads.push_back(std::move(l));
    ++n;
  }
  add_count(rep, "DCLUMPLOAD", n);
}

void read_dcconverter(const XLWorksheet& ws, HybridPowerSystem& sys,
                      const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    DCDCConverter c;
    c.index = ++idx;
    c.name = id;
    c.bus_in = resolve(name2idx, cell_by_name(ws, r, col, "InputBus"));
    c.bus_out = resolve(name2idx, cell_by_name(ws, r, col, "OutputBus"));
    c.p_ref_mw = dbl_from_str(cell_by_name(ws, r, col, "KW")) / 1000.0;  // kW -> MW
    c.eta = dbl_from_str(getv(ws, r, col, {"PercentEFF", "DcPercentEFF"}), 98.0) / 100.0;
    // ETAP exports input/output voltages in volts (InputV/OutputV).
    {
      const std::string vin = getv(ws, r, col, {"Vin_kV"});
      c.vn_in_kv = !vin.empty() ? dbl_from_str(vin)
                                : dbl_from_str(getv(ws, r, col, {"InputV"})) / 1000.0;
      const std::string vout = getv(ws, r, col, {"Vout_kV"});
      c.vn_out_kv = !vout.empty() ? dbl_from_str(vout)
                                  : dbl_from_str(getv(ws, r, col, {"OutputV"})) / 1000.0;
    }
    c.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmax_MW"));
    c.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmin_MW"));
    c.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.dc.dcdc_converters.push_back(std::move(c));
    ++n;
  }
  add_count(rep, "DCCONVERTER", n);
}

void read_dccb(const XLWorksheet& ws, HybridPowerSystem& sys,
               const NameIndex& name2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    DCCircuitBreaker cb;
    cb.index = ++idx;
    cb.name = id;
    cb.bus_from = resolve(name2idx, cell_by_name(ws, r, col, "FromBus"));
    cb.bus_to = resolve(name2idx, cell_by_name(ws, r, col, "ToBus"));
    cb.closed = bool_from_str(cell_by_name(ws, r, col, "Closed"), true);
    cb.rated_voltage_kv = dbl_from_str(cell_by_name(ws, r, col, "RatedKV"));
    cb.i_breaking_ka = dbl_from_str(getv(ws, r, col, {"I_breaking_kA", "Rated"}), cb.i_breaking_ka);
    cb.r_ohm = dbl_from_str(cell_by_name(ws, r, col, "R_ohm"), cb.r_ohm);
    cb.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.dc.dc_circuit_breakers.push_back(std::move(cb));
    ++n;
  }
  add_count(rep, "DCCB", n);
}

void read_converter_sheet(const XLWorksheet& ws, HybridPowerSystem& sys,
                          const NameIndex& ac2idx, const NameIndex& dc2idx,
                          const std::string& type_tag, EtapIoReport& rep,
                          const std::string& sheet) {
  const auto col = make_colmap(ws);
  int idx = static_cast<int>(sys.vsc_converters.size());
  int n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    VSCConverter v;
    v.index = ++idx;
    v.name = id;
    v.type = type_tag;
    v.bus_ac = resolve(ac2idx, getv(ws, r, col, {"ACBus", "BusID", "Bus"}));
    v.bus_dc = resolve(dc2idx, getv(ws, r, col, {"DCBus", "OutputBus"}));
    v.control_mode = converter_mode_from_str(cell_by_name(ws, r, col, "Mode"));
    {
      // P_MW is the round-trip schema (MW); ETAP exports OpAcKw in kW.
      const std::string pac = getv(ws, r, col, {"P_MW"});
      v.p_set_mw = !pac.empty()
                       ? dbl_from_str(pac)
                       : dbl_from_str(getv(ws, r, col, {"OpAcKw"})) / 1000.0;
    }
    v.q_set_mvar = dbl_from_str(cell_by_name(ws, r, col, "Q_Mvar"));
    v.v_dc_set_pu = dbl_from_str(cell_by_name(ws, r, col, "Vdc_set_pu"), v.v_dc_set_pu);
    v.v_ac_set_pu = dbl_from_str(cell_by_name(ws, r, col, "Vac_set_pu"), v.v_ac_set_pu);
    v.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmax_MW"));
    v.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmin_MW"));
    v.qmax_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmax_Mvar"));
    v.qmin_mvar = dbl_from_str(cell_by_name(ws, r, col, "Qmin_Mvar"));
    v.eta = dbl_from_str(getv(ws, r, col, {"PercentEFF", "DcPercentEFF"}), 99.0) / 100.0;
    v.vn_ac_kv = dbl_from_str(cell_by_name(ws, r, col, "Vac_kV"));
    v.vn_dc_kv = dbl_from_str(cell_by_name(ws, r, col, "Vdc_kV"));
    v.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.vsc_converters.push_back(std::move(v));
    ++n;
  }
  add_count(rep, sheet, n);
}

void read_battery(const XLWorksheet& ws, HybridPowerSystem& sys,
                  const NameIndex& dc2idx, EtapIoReport& rep) {
  const auto col = make_colmap(ws);
  int idx = 0, n = 0;
  for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
    const std::string id = cell_by_name(ws, r, col, "ID");
    if (trim(id).empty()) break;
    Storage s;
    s.index = ++idx;
    s.name = id;
    s.bus = resolve(dc2idx, cell_by_name(ws, r, col, "Bus"));
    s.p_mw = dbl_from_str(cell_by_name(ws, r, col, "P_MW"));
    s.pmax_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmax_MW"));
    s.pmin_mw = dbl_from_str(cell_by_name(ws, r, col, "Pmin_MW"));
    s.e_rated_mwh = dbl_from_str(cell_by_name(ws, r, col, "E_MWh"));
    s.soc_init = dbl_from_str(cell_by_name(ws, r, col, "SoC"), s.soc_init);
    s.soc_min = dbl_from_str(cell_by_name(ws, r, col, "SoCMin"), s.soc_min);
    s.soc_max = dbl_from_str(cell_by_name(ws, r, col, "SoCMax"), s.soc_max);
    s.eta_charge = dbl_from_str(getv(ws, r, col, {"EtaCh", "Eff"}), s.eta_charge);
    s.eta_discharge = dbl_from_str(cell_by_name(ws, r, col, "EtaDis"), s.eta_discharge);
    s.in_service = bool_from_str(cell_by_name(ws, r, col, "InService"), true);
    sys.dc.storage.push_back(std::move(s));
    ++n;
  }
  add_count(rep, "BATTERY", n);
}

// ── native ETAP XML: safe tokenizer + connectivity helpers ─────────────────
//
// A single forward pass over the document, O(n), with no regular-expression
// backtracking (the previous std::regex scan hung on real 400 KB exports).
// Every open/self-closing element is captured by its UPPER-CASED tag name with
// its attributes (keys upper-cased, values copied raw so UTF-8 / CJK bytes pass
// through untouched).  Closing tags, the XML prolog, comments, CDATA and
// DOCTYPE are skipped.  ETAP attribute values never contain a raw '<' or '>',
// but the scanner tolerates them inside a quoted value anyway.
struct XmlElement {
  std::string tag;                                       // UPPER-CASED
  std::unordered_map<std::string, std::string> attrs;    // UPPER-CASED key
};

std::vector<XmlElement> tokenize_xml(const std::string& xml) {
  std::vector<XmlElement> out;
  const size_t n = xml.size();
  size_t i = 0;
  auto is_name = [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' ||
           c == ':' || c == '.';
  };
  while (i < n) {
    // Advance to the next '<'.
    if (xml[i] != '<') { ++i; continue; }
    if (i + 1 >= n) break;
    const char c1 = xml[i + 1];
    if (c1 == '?') {                                   // <? ... ?>
      const size_t e = xml.find("?>", i + 2);
      i = (e == std::string::npos) ? n : e + 2;
      continue;
    }
    if (c1 == '!') {                                   // <!-- --> , <![CDATA[ ]]>, <!DOCTYPE>
      if (xml.compare(i, 4, "<!--") == 0) {
        const size_t e = xml.find("-->", i + 4);
        i = (e == std::string::npos) ? n : e + 3;
      } else if (xml.compare(i, 9, "<![CDATA[") == 0) {
        const size_t e = xml.find("]]>", i + 9);
        i = (e == std::string::npos) ? n : e + 3;
      } else {
        const size_t e = xml.find('>', i + 2);
        i = (e == std::string::npos) ? n : e + 1;
      }
      continue;
    }
    if (c1 == '/') {                                   // closing tag </tag>
      const size_t e = xml.find('>', i + 2);
      i = (e == std::string::npos) ? n : e + 1;
      continue;
    }
    // Opening / self-closing element: read the tag name.
    size_t j = i + 1;
    const size_t name_beg = j;
    while (j < n && is_name(xml[j])) ++j;
    if (j == name_beg) { ++i; continue; }              // not a real tag
    XmlElement el;
    el.tag = to_upper(xml.substr(name_beg, j - name_beg));
    // Parse attributes until '>' or '/>'.
    while (j < n) {
      while (j < n && std::isspace(static_cast<unsigned char>(xml[j]))) ++j;
      if (j >= n) break;
      if (xml[j] == '>') { ++j; break; }
      if (xml[j] == '/') { ++j; continue; }            // self-close slash
      const size_t kbeg = j;
      while (j < n && is_name(xml[j])) ++j;
      if (j == kbeg) { ++j; continue; }                // stray char
      std::string key = to_upper(xml.substr(kbeg, j - kbeg));
      while (j < n && std::isspace(static_cast<unsigned char>(xml[j]))) ++j;
      std::string val;
      if (j < n && xml[j] == '=') {
        ++j;
        while (j < n && std::isspace(static_cast<unsigned char>(xml[j]))) ++j;
        if (j < n && (xml[j] == '"' || xml[j] == '\'')) {
          const char q = xml[j++];
          const size_t vbeg = j;
          while (j < n && xml[j] != q) ++j;
          val = xml.substr(vbeg, j - vbeg);
          if (j < n) ++j;                              // skip closing quote
        } else {                                        // unquoted value
          const size_t vbeg = j;
          while (j < n && !std::isspace(static_cast<unsigned char>(xml[j])) &&
                 xml[j] != '>' && xml[j] != '/')
            ++j;
          val = xml.substr(vbeg, j - vbeg);
        }
      }
      el.attrs.emplace(std::move(key), std::move(val));  // first occurrence wins
    }
    out.push_back(std::move(el));
    i = j;  // advance past this element (otherwise the outer loop re-parses it)
  }
  return out;
}

// One resolved device endpoint discovered from a <CONNECT> record: the bus the
// device pin attaches to, the device-side pin (0 = from/primary/input,
// 1 = to/secondary/output, 2 = tertiary), and whether the bus is on the DC side.
struct EtapEndpoint {
  std::string bus_id;
  int pin{0};
  bool is_dc{false};
};

}  // namespace

// ═════════════════════════════════════════════════════════════════════════
// Public API
// ═════════════════════════════════════════════════════════════════════════
void save_etap(const HybridPowerSystem& sys, const std::string& path,
               EtapIoReport& report) {
  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();

  if (wb.worksheetExists("Sheet1")) {
    wb.worksheet("Sheet1").setName("PROJECT");
  } else {
    ensure_sheet(wb, "PROJECT");
  }

  const std::vector<std::string> sheets = {
      "BUS",        "XLINE",      "CABLE",       "XFORM2W",   "XFORM3W",
      "UTIL",       "SYNGEN",     "PVARRAY",     "WIND",       "LUMPEDLOAD",
      "CAPACITOR",  "HVCB",       "INDMOTOR",    "DCBUS",      "DCIMPEDANCE",
      "DCLUMPLOAD", "DCCONVERTER", "DCCB",       "INVERTER",   "CHARGER",
      "BATTERY"};
  for (const auto& s : sheets) ensure_sheet(wb, s);

  write_project(wb.worksheet("PROJECT"), sys);
  write_bus(wb.worksheet("BUS"), sys);
  write_xline(wb.worksheet("XLINE"), sys);
  write_headers(wb.worksheet("CABLE"),
                {"Index", "ID", "FromBus", "ToBus", "R_ohm", "X_ohm", "B_S",
                 "Tap", "ShiftDeg", "Rate_MVA", "Length_km", "InService",
                 "FailureRate", "MTTR_hr", "N_parallel"});
  write_xform2w(wb.worksheet("XFORM2W"), sys);
  write_xform3w(wb.worksheet("XFORM3W"), sys);
  write_util(wb.worksheet("UTIL"), sys);
  write_syngen(wb.worksheet("SYNGEN"), sys);
  write_pvarray(wb.worksheet("PVARRAY"), sys);
  write_wind(wb.worksheet("WIND"), sys);
  write_lumpedload(wb.worksheet("LUMPEDLOAD"), sys);
  write_capacitor(wb.worksheet("CAPACITOR"), sys);
  write_hvcb(wb.worksheet("HVCB"), sys);
  write_indmotor(wb.worksheet("INDMOTOR"), sys);
  write_dcbus(wb.worksheet("DCBUS"), sys);
  write_dcimpedance(wb.worksheet("DCIMPEDANCE"), sys);
  write_dclumpload(wb.worksheet("DCLUMPLOAD"), sys);
  write_dcconverter(wb.worksheet("DCCONVERTER"), sys);
  write_dccb(wb.worksheet("DCCB"), sys);
  write_converter_sheet(wb.worksheet("INVERTER"), sys, /*want_charger=*/false);
  write_converter_sheet(wb.worksheet("CHARGER"), sys, /*want_charger=*/true);
  write_battery(wb.worksheet("BATTERY"), sys);

  add_count(report, "BUS", static_cast<int>(sys.ac.buses.size()));
  add_count(report, "XLINE", static_cast<int>(sys.ac.branches.size()));
  add_count(report, "XFORM2W", static_cast<int>(sys.ac.transformers_2w.size()));
  add_count(report, "XFORM3W", static_cast<int>(sys.ac.transformers_3w.size()));
  add_count(report, "UTIL", static_cast<int>(sys.ac.external_grids.size()));
  add_count(report, "SYNGEN", static_cast<int>(sys.ac.generators.size()));
  add_count(report, "PVARRAY", static_cast<int>(sys.ac.pv_systems.size()));
  add_count(report, "WIND", static_cast<int>(sys.ac.renewable_gens.size()));
  add_count(report, "LUMPEDLOAD", static_cast<int>(sys.ac.loads.size()));
  add_count(report, "CAPACITOR", static_cast<int>(sys.ac.shunts.size()));
  add_count(report, "HVCB", static_cast<int>(sys.ac.circuit_breakers.size()));
  add_count(report, "INDMOTOR", static_cast<int>(sys.ac.motors.size()));
  add_count(report, "DCBUS", static_cast<int>(sys.dc.buses.size()));
  add_count(report, "DCIMPEDANCE", static_cast<int>(sys.dc.branches.size()));
  add_count(report, "DCLUMPLOAD", static_cast<int>(sys.dc.loads.size()));
  add_count(report, "DCCONVERTER", static_cast<int>(sys.dc.dcdc_converters.size()));
  add_count(report, "DCCB", static_cast<int>(sys.dc.dc_circuit_breakers.size()));
  add_count(report, "BATTERY", static_cast<int>(sys.dc.storage.size()));

  doc.save();
  doc.close();
}

void save_etap(const HybridPowerSystem& sys, const std::string& path) {
  EtapIoReport report;
  save_etap(sys, path, report);
}

HybridPowerSystem load_etap(const std::string& path, EtapImportMode mode,
                            EtapIoReport& report) {
  XLDocument doc;
  doc.open(path);
  XLWorkbook wb = doc.workbook();

  HybridPowerSystem sys;
  NameIndex ac2idx, dc2idx;
  NameKv ac2kv, dc2kv;

  std::string s;
  if (!(s = find_sheet_name(wb, "PROJECT")).empty()) read_project(wb.worksheet(s), sys);

  // Buses first so element sheets can resolve references and impedance bases.
  if (!(s = find_sheet_name(wb, "BUS")).empty()) read_bus(wb.worksheet(s), sys, ac2idx, ac2kv, report);
  if (!(s = find_sheet_name(wb, "DCBUS")).empty()) read_dcbus(wb.worksheet(s), sys, dc2idx, dc2kv, report);

  if (!(s = find_sheet_name(wb, "XLINE")).empty()) read_branch_sheet(wb.worksheet(s), sys, ac2idx, ac2kv, sys.base_mva, report, "XLINE");
  if (!(s = find_sheet_name(wb, "CABLE")).empty()) read_branch_sheet(wb.worksheet(s), sys, ac2idx, ac2kv, sys.base_mva, report, "CABLE");
  if (!(s = find_sheet_name(wb, "XFORM2W")).empty()) read_xform2w(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "XFORM3W")).empty()) read_xform3w(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "UTIL")).empty()) read_util(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "SYNGEN")).empty()) read_syngen(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "MGSET")).empty()) read_mgset(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "PVARRAY")).empty()) read_pvarray(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "WIND")).empty()) read_wind(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "LUMPEDLOAD")).empty()) read_lumpedload(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "CAPACITOR")).empty()) read_capacitor(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "HVCB")).empty()) read_hvcb(wb.worksheet(s), sys, ac2idx, report);
  if (!(s = find_sheet_name(wb, "INDMOTOR")).empty()) read_indmotor(wb.worksheet(s), sys, ac2idx, report);

  if (!(s = find_sheet_name(wb, "DCIMPEDANCE")).empty()) read_dcimpedance(wb.worksheet(s), sys, dc2idx, dc2kv, sys.base_mva, report);
  if (!(s = find_sheet_name(wb, "DCLUMPLOAD")).empty()) read_dclumpload(wb.worksheet(s), sys, dc2idx, report);
  if (!(s = find_sheet_name(wb, "DCCONVERTER")).empty()) read_dcconverter(wb.worksheet(s), sys, dc2idx, report);
  if (!(s = find_sheet_name(wb, "DCCB")).empty()) read_dccb(wb.worksheet(s), sys, dc2idx, report);

  if (!(s = find_sheet_name(wb, "INVERTER")).empty()) read_converter_sheet(wb.worksheet(s), sys, ac2idx, dc2idx, "INVERTER", report, "INVERTER");
  if (!(s = find_sheet_name(wb, "CHARGER")).empty()) read_converter_sheet(wb.worksheet(s), sys, ac2idx, dc2idx, "CHARGER", report, "CHARGER");
  if (!(s = find_sheet_name(wb, "BATTERY")).empty()) read_battery(wb.worksheet(s), sys, dc2idx, report);

  doc.close();

  derive_bus_types(sys);
  validate_refs(sys, mode, report);
  return sys;
}

HybridPowerSystem load_etap(const std::string& path, EtapIoReport& report) {
  return load_etap(path, EtapImportMode::Permissive, report);
}

HybridPowerSystem load_etap(const std::string& path) {
  EtapIoReport report;
  return load_etap(path, EtapImportMode::Permissive, report);
}

EtapFidelityReport etap_fidelity_check(const HybridPowerSystem& sys, double tol) {
  EtapFidelityReport rep;
  namespace fs = std::filesystem;
  const std::string tmp =
      (fs::temp_directory_path() / "hacdcpf_etap_fidelity.xlsx").string();
  EtapIoReport io;
  save_etap(sys, tmp, io);
  const HybridPowerSystem rt = load_etap(tmp);
  std::error_code ec;
  fs::remove(tmp, ec);

  auto fd = [&](const std::string& w, double a, double b) {
    ++rep.fields_checked;
    if (std::abs(a - b) > tol) {
      ++rep.fields_mismatched;
      rep.lossless = false;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s: %.6g -> %.6g", w.c_str(), a, b);
      rep.mismatches.emplace_back(buf);
    }
  };
  auto fi = [&](const std::string& w, long long a, long long b) {
    ++rep.fields_checked;
    if (a != b) {
      ++rep.fields_mismatched;
      rep.lossless = false;
      rep.mismatches.push_back(w + ": " + std::to_string(a) + " -> " + std::to_string(b));
    }
  };
  auto fstr = [&](const std::string& w, const std::string& a, const std::string& b) {
    ++rep.fields_checked;
    if (a != b) {
      ++rep.fields_mismatched;
      rep.lossless = false;
      rep.mismatches.push_back(w + ": '" + a + "' -> '" + b + "'");
    }
  };

  fi("count ac.buses", sys.ac.buses.size(), rt.ac.buses.size());
  fi("count ac.branches", sys.ac.branches.size(), rt.ac.branches.size());
  fi("count ac.transformers_2w", sys.ac.transformers_2w.size(), rt.ac.transformers_2w.size());
  fi("count ac.transformers_3w", sys.ac.transformers_3w.size(), rt.ac.transformers_3w.size());
  fi("count ac.external_grids", sys.ac.external_grids.size(), rt.ac.external_grids.size());
  fi("count ac.generators", sys.ac.generators.size(), rt.ac.generators.size());
  fi("count ac.pv_systems", sys.ac.pv_systems.size(), rt.ac.pv_systems.size());
  fi("count ac.renewable_gens", sys.ac.renewable_gens.size(), rt.ac.renewable_gens.size());
  fi("count ac.loads", sys.ac.loads.size(), rt.ac.loads.size());
  fi("count ac.shunts", sys.ac.shunts.size(), rt.ac.shunts.size());
  fi("count ac.motors", sys.ac.motors.size(), rt.ac.motors.size());
  fi("count ac.circuit_breakers", sys.ac.circuit_breakers.size(), rt.ac.circuit_breakers.size());
  fi("count dc.buses", sys.dc.buses.size(), rt.dc.buses.size());
  fi("count dc.branches", sys.dc.branches.size(), rt.dc.branches.size());
  fi("count dc.loads", sys.dc.loads.size(), rt.dc.loads.size());
  fi("count dc.dcdc_converters", sys.dc.dcdc_converters.size(), rt.dc.dcdc_converters.size());
  fi("count dc.storage", sys.dc.storage.size(), rt.dc.storage.size());
  fi("count vsc_converters", sys.vsc_converters.size(), rt.vsc_converters.size());

  if (sys.ac.buses.size() == rt.ac.buses.size()) {
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& a = sys.ac.buses[i];
      const auto& b = rt.ac.buses[i];
      const std::string p = "ACBus[" + std::to_string(i) + "].";
      fstr(p + "name", a.name, b.name);
      fi(p + "bus_type", static_cast<int>(a.bus_type), static_cast<int>(b.bus_type));
      fd(p + "base_kv", a.base_kv, b.base_kv);
      fd(p + "pd_mw", a.pd_mw, b.pd_mw);
      fd(p + "qd_mvar", a.qd_mvar, b.qd_mvar);
      fd(p + "bs_mvar", a.bs_mvar, b.bs_mvar);
    }
  }
  if (sys.ac.branches.size() == rt.ac.branches.size()) {
    for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
      const auto& a = sys.ac.branches[i];
      const auto& b = rt.ac.branches[i];
      const std::string p = "ACBranch[" + std::to_string(i) + "].";
      fi(p + "from_bus", a.from_bus, b.from_bus);
      fi(p + "to_bus", a.to_bus, b.to_bus);
      fd(p + "r_pu", a.r_pu, b.r_pu);
      fd(p + "x_pu", a.x_pu, b.x_pu);
      fd(p + "b_pu", a.b_pu, b.b_pu);
      fd(p + "tap", a.tap, b.tap);
    }
  }
  if (sys.ac.transformers_2w.size() == rt.ac.transformers_2w.size()) {
    for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
      const auto& a = sys.ac.transformers_2w[i];
      const auto& b = rt.ac.transformers_2w[i];
      const std::string p = "Transformer2W[" + std::to_string(i) + "].";
      fd(p + "vk_percent", a.vk_percent, b.vk_percent);
      fd(p + "sn_mva", a.sn_mva, b.sn_mva);
      fi(p + "tap_pos", a.tap_pos, b.tap_pos);
      fd(p + "tap_step_percent", a.tap_step_percent, b.tap_step_percent);
      fd(p + "z0_percent", a.z0_percent, b.z0_percent);
    }
  }
  if (sys.ac.generators.size() == rt.ac.generators.size()) {
    for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
      const auto& a = sys.ac.generators[i];
      const auto& b = rt.ac.generators[i];
      const std::string p = "Generator[" + std::to_string(i) + "].";
      fi(p + "bus", a.bus, b.bus);
      fd(p + "pg_mw", a.pg_mw, b.pg_mw);
      fd(p + "vg_pu", a.vg_pu, b.vg_pu);
      fd(p + "qmax_mvar", a.qmax_mvar, b.qmax_mvar);
      fd(p + "xdpp_pu", a.xdpp_pu, b.xdpp_pu);
      fd(p + "xdp_pu", a.xdp_pu, b.xdp_pu);
      fd(p + "xd_pu", a.xd_pu, b.xd_pu);
      fd(p + "x0_pu", a.x0_pu, b.x0_pu);
      fd(p + "r0_pu", a.r0_pu, b.r0_pu);
    }
  }
  if (sys.ac.external_grids.size() == rt.ac.external_grids.size()) {
    for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
      const auto& a = sys.ac.external_grids[i];
      const auto& b = rt.ac.external_grids[i];
      const std::string p = "ExternalGrid[" + std::to_string(i) + "].";
      fi(p + "bus", a.bus, b.bus);
      fd(p + "vm_pu", a.vm_pu, b.vm_pu);
      fd(p + "r_pu", a.r_pu, b.r_pu);
      fd(p + "x_pu", a.x_pu, b.x_pu);
      fd(p + "s_sc_max_mva", a.s_sc_max_mva, b.s_sc_max_mva);
      fd(p + "s_sc_min_mva", a.s_sc_min_mva, b.s_sc_min_mva);
      fd(p + "rx_max", a.rx_max, b.rx_max);
      fd(p + "r0_pu", a.r0_pu, b.r0_pu);
      fd(p + "x0_pu", a.x0_pu, b.x0_pu);
    }
  }
  if (sys.ac.transformers_3w.size() == rt.ac.transformers_3w.size()) {
    for (size_t i = 0; i < sys.ac.transformers_3w.size(); ++i) {
      const auto& a = sys.ac.transformers_3w[i];
      const auto& b = rt.ac.transformers_3w[i];
      const std::string p = "Transformer3W[" + std::to_string(i) + "].";
      fi(p + "hv_bus", a.hv_bus, b.hv_bus);
      fi(p + "mv_bus", a.mv_bus, b.mv_bus);
      fi(p + "lv_bus", a.lv_bus, b.lv_bus);
      fd(p + "vn_hv_kv", a.vn_hv_kv, b.vn_hv_kv);
      fd(p + "vn_mv_kv", a.vn_mv_kv, b.vn_mv_kv);
      fd(p + "vn_lv_kv", a.vn_lv_kv, b.vn_lv_kv);
      fd(p + "sn_hv_mva", a.sn_hv_mva, b.sn_hv_mva);
      fd(p + "sn_mv_mva", a.sn_mv_mva, b.sn_mv_mva);
      fd(p + "sn_lv_mva", a.sn_lv_mva, b.sn_lv_mva);
      fd(p + "vk_hv_mv_percent", a.vk_hv_mv_percent, b.vk_hv_mv_percent);
      fd(p + "vk_hv_lv_percent", a.vk_hv_lv_percent, b.vk_hv_lv_percent);
      fd(p + "vk_mv_lv_percent", a.vk_mv_lv_percent, b.vk_mv_lv_percent);
      fd(p + "vkr_hv_mv_percent", a.vkr_hv_mv_percent, b.vkr_hv_mv_percent);
      fd(p + "vkr_hv_lv_percent", a.vkr_hv_lv_percent, b.vkr_hv_lv_percent);
      fd(p + "vkr_mv_lv_percent", a.vkr_mv_lv_percent, b.vkr_mv_lv_percent);
      fi(p + "tap_pos", a.tap_pos, b.tap_pos);
      fd(p + "tap_step_percent", a.tap_step_percent, b.tap_step_percent);
    }
  }
  if (sys.ac.pv_systems.size() == rt.ac.pv_systems.size()) {
    for (size_t i = 0; i < sys.ac.pv_systems.size(); ++i) {
      const auto& a = sys.ac.pv_systems[i];
      const auto& b = rt.ac.pv_systems[i];
      const std::string p = "PVSystem[" + std::to_string(i) + "].";
      fd(p + "p_mw", a.p_mw, b.p_mw);
      fd(p + "q_mvar", a.q_mvar, b.q_mvar);
      fd(p + "sn_mva", a.sn_mva, b.sn_mva);
      fd(p + "pmax_mw", a.pmax_mw, b.pmax_mw);
      fd(p + "qmax_mvar", a.qmax_mvar, b.qmax_mvar);
    }
  }
  if (sys.ac.renewable_gens.size() == rt.ac.renewable_gens.size()) {
    for (size_t i = 0; i < sys.ac.renewable_gens.size(); ++i) {
      const auto& a = sys.ac.renewable_gens[i];
      const auto& b = rt.ac.renewable_gens[i];
      const std::string p = "RenewableGen[" + std::to_string(i) + "].";
      fi(p + "type", static_cast<int>(a.type), static_cast<int>(b.type));
      fd(p + "p_mw", a.p_mw, b.p_mw);
      fd(p + "p_rated_mw", a.p_rated_mw, b.p_rated_mw);
      fd(p + "capacity_factor", a.capacity_factor, b.capacity_factor);
      fi(p + "curtailable", a.curtailable ? 1 : 0, b.curtailable ? 1 : 0);
    }
  }
  if (sys.ac.loads.size() == rt.ac.loads.size()) {
    for (size_t i = 0; i < sys.ac.loads.size(); ++i) {
      const auto& a = sys.ac.loads[i];
      const auto& b = rt.ac.loads[i];
      const std::string p = "Load[" + std::to_string(i) + "].";
      fd(p + "p_mw", a.p_mw, b.p_mw);
      fd(p + "q_mvar", a.q_mvar, b.q_mvar);
      fd(p + "scaling", a.scaling, b.scaling);
      fi(p + "model", static_cast<int>(a.model), static_cast<int>(b.model));
      fd(p + "z_percent_p", a.z_percent_p, b.z_percent_p);
      fd(p + "i_percent_p", a.i_percent_p, b.i_percent_p);
      fd(p + "p_percent_p", a.p_percent_p, b.p_percent_p);
      fi(p + "priority", static_cast<int>(a.priority), static_cast<int>(b.priority));
    }
  }
  if (sys.ac.shunts.size() == rt.ac.shunts.size()) {
    for (size_t i = 0; i < sys.ac.shunts.size(); ++i) {
      const auto& a = sys.ac.shunts[i];
      const auto& b = rt.ac.shunts[i];
      const std::string p = "Shunt[" + std::to_string(i) + "].";
      fd(p + "gs_mw", a.gs_mw, b.gs_mw);
      fd(p + "bs_mvar", a.bs_mvar, b.bs_mvar);
    }
  }
  if (sys.ac.circuit_breakers.size() == rt.ac.circuit_breakers.size()) {
    for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
      const auto& a = sys.ac.circuit_breakers[i];
      const auto& b = rt.ac.circuit_breakers[i];
      const std::string p = "CircuitBreaker[" + std::to_string(i) + "].";
      fi(p + "bus_from", a.bus_from, b.bus_from);
      fi(p + "bus_to", a.bus_to, b.bus_to);
      fi(p + "closed", a.closed ? 1 : 0, b.closed ? 1 : 0);
      fd(p + "rated_voltage_kv", a.rated_voltage_kv, b.rated_voltage_kv);
      fd(p + "i_rated_ka", a.i_rated_ka, b.i_rated_ka);
      fd(p + "i_breaking_ka", a.i_breaking_ka, b.i_breaking_ka);
    }
  }
  if (sys.ac.motors.size() == rt.ac.motors.size()) {
    for (size_t i = 0; i < sys.ac.motors.size(); ++i) {
      const auto& a = sys.ac.motors[i];
      const auto& b = rt.ac.motors[i];
      const std::string p = "Motor[" + std::to_string(i) + "].";
      fd(p + "vn_kv", a.vn_kv, b.vn_kv);
      fd(p + "sn_mva", a.sn_mva, b.sn_mva);
      fd(p + "r_pu", a.r_pu, b.r_pu);
      fd(p + "x_pu", a.x_pu, b.x_pu);
    }
  }
  if (sys.dc.branches.size() == rt.dc.branches.size()) {
    for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
      const auto& a = sys.dc.branches[i];
      const auto& b = rt.dc.branches[i];
      const std::string p = "DCBranch[" + std::to_string(i) + "].";
      fd(p + "r_pu", a.r_pu, b.r_pu);
      fd(p + "rate_a_mva", a.rate_a_mva, b.rate_a_mva);
      fd(p + "length_km", a.length_km, b.length_km);
    }
  }
  if (sys.dc.loads.size() == rt.dc.loads.size()) {
    for (size_t i = 0; i < sys.dc.loads.size(); ++i) {
      const std::string p = "DCLoad[" + std::to_string(i) + "].";
      fd(p + "p_mw", sys.dc.loads[i].p_mw, rt.dc.loads[i].p_mw);
      fd(p + "scaling", sys.dc.loads[i].scaling, rt.dc.loads[i].scaling);
    }
  }
  if (sys.dc.dcdc_converters.size() == rt.dc.dcdc_converters.size()) {
    for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
      const auto& a = sys.dc.dcdc_converters[i];
      const auto& b = rt.dc.dcdc_converters[i];
      const std::string p = "DCDCConverter[" + std::to_string(i) + "].";
      fd(p + "p_ref_mw", a.p_ref_mw, b.p_ref_mw);
      fd(p + "eta", a.eta, b.eta);
      fd(p + "vn_in_kv", a.vn_in_kv, b.vn_in_kv);
      fd(p + "vn_out_kv", a.vn_out_kv, b.vn_out_kv);
      fd(p + "pmax_mw", a.pmax_mw, b.pmax_mw);
    }
  }
  if (sys.dc.storage.size() == rt.dc.storage.size()) {
    for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
      const auto& a = sys.dc.storage[i];
      const auto& b = rt.dc.storage[i];
      const std::string p = "Storage[" + std::to_string(i) + "].";
      fd(p + "pmax_mw", a.pmax_mw, b.pmax_mw);
      fd(p + "e_rated_mwh", a.e_rated_mwh, b.e_rated_mwh);
      fd(p + "soc_init", a.soc_init, b.soc_init);
      fd(p + "soc_min", a.soc_min, b.soc_min);
      fd(p + "eta_charge", a.eta_charge, b.eta_charge);
      fd(p + "eta_discharge", a.eta_discharge, b.eta_discharge);
    }
  }
  if (sys.vsc_converters.size() == rt.vsc_converters.size()) {
    for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
      const auto& a = sys.vsc_converters[i];
      const auto& b = rt.vsc_converters[i];
      const std::string p = "VSCConverter[" + std::to_string(i) + "].";
      fi(p + "bus_ac", a.bus_ac, b.bus_ac);
      fi(p + "bus_dc", a.bus_dc, b.bus_dc);
      fi(p + "control_mode", static_cast<int>(a.control_mode), static_cast<int>(b.control_mode));
      fd(p + "p_set_mw", a.p_set_mw, b.p_set_mw);
      fd(p + "q_set_mvar", a.q_set_mvar, b.q_set_mvar);
      fd(p + "pmax_mw", a.pmax_mw, b.pmax_mw);
      fd(p + "qmax_mvar", a.qmax_mvar, b.qmax_mvar);
      fd(p + "eta", a.eta, b.eta);
      fd(p + "vn_ac_kv", a.vn_ac_kv, b.vn_ac_kv);
      fd(p + "vn_dc_kv", a.vn_dc_kv, b.vn_dc_kv);
    }
  }
  return rep;
}

// ═════════════════════════════════════════════════════════════════════════
// Native ETAP project XML (e.g. Feeder.xml) — best-effort direct importer.
// ═════════════════════════════════════════════════════════════════════════
HybridPowerSystem load_etap_xml(const std::string& path, EtapImportMode mode,
                                EtapIoReport& report) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("ETAP XML: cannot open " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string xml = ss.str();

  using XmlAttrs = std::unordered_map<std::string, std::string>;
  // Safe, non-backtracking tokenizer (the old std::regex scan hung on real
  // 400 KB exports). Collect every element by UPPER-CASED tag; dedupe by IID so
  // an element that appears in both <LAYOUT> and <DUMPSTERS> is imported once.
  std::unordered_map<std::string, std::vector<XmlAttrs>> elems;
  {
    std::vector<XmlElement> tokens = tokenize_xml(xml);
    std::unordered_map<std::string, std::unordered_set<std::string>> seen_iid;
    int dumpster_note = 0;
    for (auto& el : tokens) {
      if (el.attrs.empty()) continue;
      auto iit = el.attrs.find("IID");
      if (iit != el.attrs.end() && !trim(iit->second).empty()) {
        if (!seen_iid[el.tag].insert(iit->second).second) { ++dumpster_note; continue; }
      }
      elems[el.tag].push_back(std::move(el.attrs));
    }
    if (dumpster_note > 0)
      report.warnings.push_back("ETAP XML: skipped " + std::to_string(dumpster_note) +
                                " duplicate element(s) (same IID in LAYOUT/DUMPSTERS)");
  }

  auto xget = [](const XmlAttrs& a,
                 std::initializer_list<const char*> keys) -> std::string {
    for (const char* k : keys) {
      std::string up;
      up.reserve(std::char_traits<char>::length(k));
      for (const char* c = k; *c; ++c) up.push_back(static_cast<char>(std::toupper(*c)));
      auto it = a.find(up);
      if (it != a.end() && !trim(it->second).empty()) return it->second;
    }
    return "";
  };

  HybridPowerSystem sys;
  NameIndex ac2idx, dc2idx;
  NameKv ac2kv, dc2kv;

  // AC buses first.
  int idx = 0;
  for (const auto& a : elems["BUS"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    ACBus b;
    b.index = ++idx;
    b.name = id;
    const double nk = dbl_from_str(xget(a, {"NominalkV"}));
    const double bk = dbl_from_str(xget(a, {"BasekV"}));
    b.base_kv = nk > 0.0 ? nk : bk;
    b.vm_pu = volt_pu(xget(a, {"OpVMag", "VMag"}), 1.0);
    b.va_deg = dbl_from_str(xget(a, {"OpVAng"}));
    b.vmax_pu = volt_pu(xget(a, {"VMaxLimit"}), 1.1);
    b.vmin_pu = volt_pu(xget(a, {"VMinLimit"}), 0.9);
    b.area = int_from_str(xget(a, {"Area"}), 1);
    b.zone = int_from_str(xget(a, {"Zone"}), 1);
    b.in_service = bool_from_str(xget(a, {"InService"}), true);
    ac2idx[to_upper(id)] = b.index;
    ac2kv[to_upper(id)] = b.base_kv;
    sys.ac.buses.push_back(std::move(b));
  }
  // DC buses.
  idx = 0;
  for (const auto& a : elems["DCBUS"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    DCBus b;
    b.index = ++idx;
    b.name = id;
    b.base_kv = dbl_from_str(xget(a, {"NominalV"})) / 1000.0;
    b.in_service = bool_from_str(xget(a, {"InService"}), true);
    dc2idx[to_upper(id)] = b.index;
    dc2kv[to_upper(id)] = b.base_kv;
    sys.dc.buses.push_back(std::move(b));
  }

  // ---- Connectivity from <CONNECT> records ----
  // ETAP wires devices to buses through separate CONNECT records (the device's
  // own FromBus/ToBus/Bus attributes are often empty — always so for breakers).
  // device IID -> [{bus id, device-side pin, dc?}].
  std::unordered_map<std::string, std::vector<EtapEndpoint>> conn;
  // IID -> element tag, used to skip transformers during base-kV propagation.
  std::unordered_map<std::string, std::string> iid2tag;
  for (const auto& kv : elems)
    for (const auto& a : kv.second) {
      auto it = a.find("IID");
      if (it != a.end() && !trim(it->second).empty()) iid2tag[it->second] = kv.first;
    }
  {
    auto cget = [](const XmlAttrs& a, const char* k) -> std::string {
      auto it = a.find(k);
      return it == a.end() ? std::string() : it->second;
    };
    auto is_bus = [](const std::string& e) { return e == "BUS" || e == "DCBUS"; };
    for (const auto& c : elems["CONNECT"]) {
      const std::string fe = to_upper(cget(c, "FROMELEMENT"));
      const std::string te = to_upper(cget(c, "TOELEMENT"));
      if (is_bus(fe) && !is_bus(te))
        conn[cget(c, "TOIID")].push_back(
            {cget(c, "FROMID"), int_from_str(cget(c, "TOPIN")), fe == "DCBUS"});
      else if (is_bus(te) && !is_bus(fe))
        conn[cget(c, "FROMIID")].push_back(
            {cget(c, "TOID"), int_from_str(cget(c, "FROMPIN")), te == "DCBUS"});
    }
  }

  // Resolve a device endpoint: prefer an explicit attribute bus id, else fall
  // back to the device's CONNECT records at the requested pin (-1 = any).
  auto resolve_endpoint = [&](const NameIndex& nidx, const XmlAttrs& a,
                              std::initializer_list<const char*> keys,
                              int wanted_pin) -> int {
    const std::string ex = xget(a, keys);
    if (!ex.empty()) {
      const int r = resolve(nidx, ex);
      if (r) return r;
    }
    auto iit = a.find("IID");
    if (iit != a.end()) {
      auto cit = conn.find(iit->second);
      if (cit != conn.end()) {
        for (const auto& ep : cit->second)
          if (wanted_pin < 0 || ep.pin == wanted_pin) {
            const int r = resolve(nidx, ep.bus_id);
            if (r) return r;
          }
        for (const auto& ep : cit->second) {  // pin not matched -> first resolvable
          const int r = resolve(nidx, ep.bus_id);
          if (r) return r;
        }
      }
    }
    return 0;
  };

  // Resolve BOTH terminals of a two-terminal device (line, 2W transformer,
  // breaker, switch, DC branch, DC converter) at once.  Resolving each end with
  // resolve_endpoint() independently is wrong: real ETAP exports leave the
  // device's own FromBus/ToBus empty and carry connectivity only in CONNECT
  // records whose device-side pin numbers often do NOT follow the expected 0/1
  // convention.  When the pins fail to match, both ends fall back to "first
  // resolvable" and collapse onto the SAME bus.  This resolver assigns the two
  // CONNECT endpoints positionally so the ends always land on different buses.
  auto resolve_pair = [&](const NameIndex& nidx, const XmlAttrs& a,
                          std::initializer_list<const char*> from_keys,
                          std::initializer_list<const char*> to_keys,
                          int from_pin, int to_pin) -> std::pair<int, int> {
    int from = 0, to = 0;
    // 1) Explicit attribute bus ids win when present.
    { const std::string ex = xget(a, from_keys); if (!ex.empty()) from = resolve(nidx, ex); }
    { const std::string ex = xget(a, to_keys);   if (!ex.empty()) to   = resolve(nidx, ex); }

    // Gather this device's resolvable CONNECT endpoints (pin, bus index), in
    // document order.
    std::vector<std::pair<int, int>> eps;
    auto iit = a.find("IID");
    if (iit != a.end()) {
      auto cit = conn.find(iit->second);
      if (cit != conn.end())
        for (const auto& ep : cit->second) {
          const int r = resolve(nidx, ep.bus_id);
          if (r) eps.push_back({ep.pin, r});
        }
    }
    // 2) Pin-matched assignment for ends still unresolved (the to end must not
    //    reuse the bus already taken by the from end).
    if (!from)
      for (const auto& e : eps) if (e.first == from_pin) { from = e.second; break; }
    if (!to)
      for (const auto& e : eps) if (e.first == to_pin && e.second != from) { to = e.second; break; }
    // 3) Pins didn't distinguish the ends: assign distinct endpoints by order so
    //    the two terminals never collapse onto the same bus.
    if (!from)
      for (const auto& e : eps) if (e.second != to) { from = e.second; break; }
    if (!to)
      for (const auto& e : eps) if (e.second != from) { to = e.second; break; }
    return {from, to};
  };

  // ---- Infer base_kV for junction nodes (BUS_CNODE_JCT_*, NominalkV=0) ----
  // BFS from sized buses through NON-transformer series elements only.
  {
    const size_t nb = sys.ac.buses.size();
    std::vector<std::vector<int>> adj(nb + 1);
    auto add_edge = [&](int u, int v) {
      if (u > 0 && v > 0 && u != v) { adj[u].push_back(v); adj[v].push_back(u); }
    };
    for (const auto& kv : conn) {
      const auto tit = iid2tag.find(kv.first);
      const std::string tg = tit == iid2tag.end() ? std::string() : tit->second;
      if (tg == "XFORM2W" || tg == "XFORM3W") continue;  // voltage transforms
      std::vector<int> bs;
      for (const auto& ep : kv.second)
        if (!ep.is_dc) { const int r = resolve(ac2idx, ep.bus_id); if (r) bs.push_back(r); }
      for (size_t k = 1; k < bs.size(); ++k) add_edge(bs[0], bs[k]);
    }
    for (const char* tag : {"CABLE", "XLINE"})
      for (const auto& a : elems[to_upper(tag)])
        add_edge(resolve(ac2idx, xget(a, {"FromBus"})),
                 resolve(ac2idx, xget(a, {"ToBus"})));
    std::vector<double> kvv(nb + 1, 0.0);
    std::queue<int> q;
    for (const auto& b : sys.ac.buses)
      if (b.base_kv > 0.0) { kvv[b.index] = b.base_kv; q.push(b.index); }
    while (!q.empty()) {
      const int u = q.front(); q.pop();
      for (const int v : adj[u])
        if (kvv[v] == 0.0) { kvv[v] = kvv[u]; q.push(v); }
    }
    int inferred = 0;
    for (auto& b : sys.ac.buses)
      if (b.base_kv == 0.0 && kvv[b.index] > 0.0) {
        b.base_kv = kvv[b.index];
        ac2kv[to_upper(b.name)] = b.base_kv;
        ++inferred;
      }
    if (inferred > 0)
      report.warnings.push_back("ETAP XML: inferred base kV for " +
                                std::to_string(inferred) + " junction bus(es)");
  }

  // ETAP length-unit code -> kilometres. 0=feet, 1=miles, 2=metres, 3=km (default).
  auto etap_len_to_km = [](const std::string& unit, double value) -> double {
    if (unit == "0") return value * 0.0003048;  // feet   -> km
    if (unit == "1") return value * 1.60934;    // miles  -> km
    if (unit == "2") return value * 0.001;      // metres -> km
    return value;                               // "3"/blank: already km
  };

  auto add_lines = [&](const char* tag) {
    int li = static_cast<int>(sys.ac.branches.size());
    for (const auto& a : elems[to_upper(tag)]) {
      const std::string id = xget(a, {"ID"});
      if (id.empty()) continue;
      ACBranch br;
      br.index = ++li;
      br.name = id;
      std::tie(br.from_bus, br.to_bus) =
          resolve_pair(ac2idx, a, {"FromBus"}, {"ToBus"}, 0, 1);
      const double zb = z_base(ac_bus_base_kv(sys, br.from_bus), sys.base_mva);
      // ETAP stores positive/zero-sequence impedance as ohms over a *base length*
      // (CABLE: OhmsPerLengthValue/OhmsPerLengthUnit, XLINE: PerLength/PerLengthUnit)
      // and a separate total run length (CABLE: LengthValue/CableLengthUnit,
      // XLINE: Length/LengthUnit). Each length carries its OWN unit code, so convert
      // both to km independently, then total_ohm = (Rvalue / base_km) * length_km.
      const double length_km = etap_len_to_km(xget(a, {"CableLengthUnit", "LengthUnit"}),
                                              dbl_from_str(xget(a, {"LengthValue", "Length"})));
      const double base_km = etap_len_to_km(xget(a, {"OhmsPerLengthUnit", "PerLengthUnit"}),
                                            dbl_from_str(xget(a, {"OhmsPerLengthValue", "PerLength"})));
      // When both lengths are known, scale per-base ohms by length/base; otherwise the
      // raw R/X values are treated as total ohms (synthetic exports use base=len=1).
      const double scale = (base_km > 0.0 && length_km > 0.0) ? (length_km / base_km) : 1.0;
      const double r_ohm = dbl_from_str(xget(a, {"R_ohm", "RPosValue", "RPos"})) * scale;
      const double x_ohm = dbl_from_str(xget(a, {"X_ohm", "XPosValue", "XPos"})) * scale;
      const double b_s = dbl_from_str(xget(a, {"B_S", "YPosValue", "YPos"})) * scale;
      br.r_pu = r_ohm / zb;
      br.x_pu = x_ohm / zb;
      br.b_pu = b_s * zb;
      br.r0_pu = dbl_from_str(xget(a, {"R0_ohm", "RZeroValue", "RZero"})) * scale / zb;
      br.x0_pu = dbl_from_str(xget(a, {"X0_ohm", "XZeroValue", "XZero"})) * scale / zb;
      br.tap = dbl_from_str(xget(a, {"Tap"}), 1.0);
      br.shift_deg = dbl_from_str(xget(a, {"ShiftDeg"}));
      br.rate_a_mva = dbl_from_str(xget(a, {"Rate_MVA", "RatedA"}));
      {
        const std::string lkm = xget(a, {"Length_km"});
        br.length_km = !lkm.empty() ? dbl_from_str(lkm) : length_km;
      }
      br.failure_rate = dbl_from_str(xget(a, {"FailureRate", "OutageRate"}), br.failure_rate);
      br.mttr_hr = dbl_from_str(xget(a, {"MTTR_hr"}), br.mttr_hr);
      br.n_parallel = int_from_str(xget(a, {"N_parallel"}), br.n_parallel);
      br.in_service = bool_from_str(xget(a, {"InService"}), true);
      sys.ac.branches.push_back(std::move(br));
    }
  };
  add_lines("XLINE");
  add_lines("CABLE");

  // ---- 2-winding transformers (XFORM2W) ----
  idx = 0;
  for (const auto& a : elems["XFORM2W"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Transformer2W t;
    t.index = ++idx;
    t.name = id;
    std::tie(t.hv_bus, t.lv_bus) = resolve_pair(
        ac2idx, a, {"FromBus", "PrimaryBus", "HVBus"}, {"ToBus", "SecondaryBus", "LVBus"}, 0, 1);
    t.vn_hv_kv = dbl_from_str(xget(a, {"PrimkV", "PrimKV"}));
    t.vn_lv_kv = dbl_from_str(xget(a, {"SeckV", "SecKV"}));
    {
      const std::string sn = xget(a, {"Sn_MVA"});
      const std::string am = xget(a, {"AnsiMVA"});
      if (!sn.empty()) t.sn_mva = dbl_from_str(sn);
      else if (!am.empty()) t.sn_mva = dbl_from_str(am);            // ETAP XML: MVA
      else t.sn_mva = dbl_from_str(xget(a, {"ZBaseMVA"})) / 1000.0;  // Excel schema: kVA
    }
    t.vk_percent = dbl_from_str(xget(a, {"Z_percent", "AnsiPosZ"}));
    // %R: use an explicit value if present, else derive from %Z and the X/R ratio.
    {
      const std::string zr = xget(a, {"ZR_percent", "PosR"});
      if (!zr.empty()) {
        t.vkr_percent = dbl_from_str(zr);
      } else {
        const double xr = dbl_from_str(xget(a, {"AnsiPosXR"}));
        if (xr > 0.0 && t.vk_percent > 0.0)
          t.vkr_percent = t.vk_percent / std::sqrt(1.0 + xr * xr);
      }
    }
    t.shift_deg = dbl_from_str(xget(a, {"ShiftDeg"}));
    t.pk_kw = dbl_from_str(xget(a, {"Pk_kW"}));
    t.mtbf_hours = dbl_from_str(xget(a, {"MTBF_hr"}), t.mtbf_hours);
    t.mttr_hours = dbl_from_str(xget(a, {"MTTR_hr"}), t.mttr_hours);
    t.tap_side = int_from_str(xget(a, {"TapSide"}), t.tap_side);
    t.tap_pos = int_from_str(xget(a, {"TapPos"}), t.tap_pos);
    t.tap_min = int_from_str(xget(a, {"TapMin", "NegTapSetting"}), t.tap_min);
    t.tap_max = int_from_str(xget(a, {"TapMax", "PosTapSetting"}), t.tap_max);
    t.tap_neutral = int_from_str(xget(a, {"TapNeutral"}), t.tap_neutral);
    t.tap_step_percent =
        dbl_from_str(xget(a, {"TapStepPct", "PrimaryStepPercentTap"}), t.tap_step_percent);
    t.z0_percent = dbl_from_str(xget(a, {"Z0_percent", "AnsiZeroZ"}), t.z0_percent);
    t.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.transformers_2w.push_back(std::move(t));
  }

  // ---- 3-winding transformers (XFORM3W) ----
  idx = 0;
  for (const auto& a : elems["XFORM3W"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Transformer3W t;
    t.index = ++idx;
    t.name = id;
    t.hv_bus = resolve_endpoint(ac2idx, a, {"HVBus", "PrimaryBus", "FromBus", "PrimBus"}, 0);
    t.mv_bus = resolve_endpoint(ac2idx, a, {"MVBus", "SecondaryBus", "SecBus"}, 1);
    t.lv_bus =
        resolve_endpoint(ac2idx, a, {"LVBus", "TertiaryBus", "TeritiaryBus", "TerBus"}, 2);
    t.vn_hv_kv = dbl_from_str(xget(a, {"PrimkV", "PrimKV"}));
    t.vn_mv_kv = dbl_from_str(xget(a, {"SeckV", "SecKV"}));
    t.vn_lv_kv = dbl_from_str(xget(a, {"TerkV", "TerKV"}));
    auto mva3 = [&](std::initializer_list<const char*> mn,
                    std::initializer_list<const char*> kn) {
      const std::string m = xget(a, mn);
      return !m.empty() ? dbl_from_str(m) : dbl_from_str(xget(a, kn)) / 1000.0;
    };
    t.sn_hv_mva = mva3({"Sn_HV_MVA"}, {"PrimkVA", "PrimKVA"});
    t.sn_mv_mva = mva3({"Sn_MV_MVA"}, {"SeckVA", "SecKVA"});
    t.sn_lv_mva = mva3({"Sn_LV_MVA"}, {"TerkVA", "TerKVA"});
    t.vk_hv_mv_percent = dbl_from_str(xget(a, {"Z_HV_MV_pct", "PSPosZ"}));
    t.vk_hv_lv_percent = dbl_from_str(xget(a, {"Z_HV_LV_pct", "PTPosZ"}));
    t.vk_mv_lv_percent = dbl_from_str(xget(a, {"Z_MV_LV_pct", "STPosZ"}));
    t.vkr_hv_mv_percent = dbl_from_str(xget(a, {"ZR_HV_MV_pct", "PSPosR"}), t.vkr_hv_mv_percent);
    t.vkr_hv_lv_percent = dbl_from_str(xget(a, {"ZR_HV_LV_pct", "PTPosR"}), t.vkr_hv_lv_percent);
    t.vkr_mv_lv_percent = dbl_from_str(xget(a, {"ZR_MV_LV_pct", "STPosR"}), t.vkr_mv_lv_percent);
    t.tap_side = int_from_str(xget(a, {"TapSide"}), t.tap_side);
    t.tap_pos = int_from_str(xget(a, {"TapPos"}), t.tap_pos);
    t.tap_step_percent = dbl_from_str(xget(a, {"TapStepPct", "PrimaryStepPercentTap"}), t.tap_step_percent);
    t.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.transformers_3w.push_back(std::move(t));
  }

  // ---- External grids (UTIL) ----
  idx = 0;
  for (const auto& a : elems["UTIL"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    ExternalGrid g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve_endpoint(ac2idx, a, {"Bus", "BusID"}, -1);
    g.vn_kv = dbl_from_str(xget(a, {"KV"}));
    g.vm_pu = volt_pu(xget(a, {"Vm_pu", "VoltageMagnitude", "OpVMag"}), 1.0);
    g.va_deg = dbl_from_str(xget(a, {"Va_deg", "VoltageAngle", "OpVAng"}));
    g.r_pu = dbl_from_str(xget(a, {"R_pu", "PosR"}));
    g.x_pu = dbl_from_str(xget(a, {"X_pu", "PosX"}));
    g.r0_pu = dbl_from_str(xget(a, {"R0_pu", "ZeroR"}), g.r0_pu);
    g.x0_pu = dbl_from_str(xget(a, {"X0_pu", "ZeroX"}), g.x0_pu);
    g.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.external_grids.push_back(std::move(g));
  }

  // ---- Synchronous generators (SYNGEN) ----
  idx = 0;
  for (const auto& a : elems["SYNGEN"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Generator g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    g.vn_kv = dbl_from_str(xget(a, {"KV"}));
    g.pg_mw = dbl_from_str(xget(a, {"PG_MW", "PG", "MW"}));
    g.qg_mvar = dbl_from_str(xget(a, {"QG_Mvar"}));
    g.pmax_mw = dbl_from_str(xget(a, {"Pmax_MW", "MW"}));
    g.pmin_mw = dbl_from_str(xget(a, {"Pmin_MW"}));
    g.qmax_mvar = dbl_from_str(xget(a, {"Qmax_Mvar"}));
    g.qmin_mvar = dbl_from_str(xget(a, {"Qmin_Mvar"}));
    g.vg_pu = volt_pu(xget(a, {"Vg_pu", "VoltageMagnitude"}), g.vg_pu);
    g.mbase_mva = dbl_from_str(xget(a, {"MVA"}));
    g.cos_phi = dbl_from_str(xget(a, {"CosPhi", "PowerFactor"}), g.cos_phi);
    g.is_slack = bool_from_str(xget(a, {"IsSlack"}), false);
    g.cost_c2 = dbl_from_str(xget(a, {"Cost_c2"}), g.cost_c2);
    g.cost_c1 = dbl_from_str(xget(a, {"Cost_c1"}), g.cost_c1);
    g.cost_c0 = dbl_from_str(xget(a, {"Cost_c0"}), g.cost_c0);
    g.forced_outage_rate =
        dbl_from_str(xget(a, {"FailureRate", "OutageRate"}), g.forced_outage_rate);
    g.mttr_hr = dbl_from_str(xget(a, {"MTTR_hr"}), g.mttr_hr);
    g.xdpp_pu = dbl_from_str(xget(a, {"Xdpp_pu", "Xdsat", "SubTransX"}), g.xdpp_pu);
    g.xdp_pu = dbl_from_str(xget(a, {"Xdp_pu", "TransX"}), g.xdp_pu);
    g.xd_pu = dbl_from_str(xget(a, {"Xd_pu", "SyncX"}), g.xd_pu);
    g.r0_pu = dbl_from_str(xget(a, {"R0_pu", "ZeroR"}), g.r0_pu);
    g.x0_pu = dbl_from_str(xget(a, {"X0_pu", "ZeroX"}), g.x0_pu);
    g.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.generators.push_back(std::move(g));
  }

  // ---- PV arrays (PVARRAY) ----
  idx = 0;
  for (const auto& a : elems["PVARRAY"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    PVSystem p;
    p.index = ++idx;
    p.name = id;
    p.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    {
      const std::string pmw = xget(a, {"P_MW"});
      p.p_mw = !pmw.empty() ? dbl_from_str(pmw)
                            : dbl_from_str(xget(a, {"PVAPower"})) / 1000.0;  // kW -> MW
    }
    p.q_mvar = dbl_from_str(xget(a, {"Q_Mvar"}));
    p.sn_mva = dbl_from_str(xget(a, {"Sn_MVA"}));
    p.pmax_mw = dbl_from_str(xget(a, {"Pmax_MW"}));
    p.pmin_mw = dbl_from_str(xget(a, {"Pmin_MW"}));
    p.qmax_mvar = dbl_from_str(xget(a, {"Qmax_Mvar"}));
    p.qmin_mvar = dbl_from_str(xget(a, {"Qmin_Mvar"}));
    p.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.pv_systems.push_back(std::move(p));
  }

  // ---- Wind / renewable generators (WIND) ----
  idx = 0;
  for (const auto& a : elems["WIND"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    RenewableGen g;
    g.index = ++idx;
    g.name = id;
    g.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    g.type = renewable_type_from_str(xget(a, {"Type"}));
    g.p_mw = dbl_from_str(xget(a, {"P_MW"}));
    g.q_mvar = dbl_from_str(xget(a, {"Q_Mvar"}));
    g.p_rated_mw = dbl_from_str(xget(a, {"P_rated_MW"}));
    g.qmax_mvar = dbl_from_str(xget(a, {"Qmax_Mvar"}));
    g.qmin_mvar = dbl_from_str(xget(a, {"Qmin_Mvar"}));
    g.curtailable = bool_from_str(xget(a, {"Curtailable"}), g.curtailable);
    g.capacity_factor = dbl_from_str(xget(a, {"CapacityFactor"}), g.capacity_factor);
    g.cost_curtail_mwh = dbl_from_str(xget(a, {"CostCurtail_MWh"}), g.cost_curtail_mwh);
    g.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.renewable_gens.push_back(std::move(g));
  }

  // ---- Lumped loads (LUMPEDLOAD) ----
  idx = 0;
  for (const auto& a : elems["LUMPEDLOAD"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Load l;
    l.index = ++idx;
    l.name = id;
    l.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    l.p_mw = dbl_from_str(xget(a, {"P_MW", "OpMW"}));
    l.q_mvar = dbl_from_str(xget(a, {"Q_Mvar", "OpMvar"}));
    const double mva = dbl_from_str(xget(a, {"MVA"}));
    if (std::abs(l.p_mw) < 1e-12 && std::abs(l.q_mvar) < 1e-12) {
      if (mva > 0.0) {
        const double pf = pf_fraction(dbl_from_str(xget(a, {"PF", "PowerFactor"}), 100.0));
        l.p_mw = mva * pf;
        l.q_mvar = mva * std::sqrt(std::max(0.0, 1.0 - pf * pf));
      }
    }
    l.sn_mva = mva;  // nameplate
    l.motor_percent = dbl_from_str(xget(a, {"MotorLoadPercent"}), l.motor_percent);
    l.scaling = dbl_from_str(xget(a, {"Scaling"}), 1.0);
    l.model = load_model_from_str(xget(a, {"Model", "ModelType"}));
    l.priority = load_priority_from_str(xget(a, {"Priority"}));
    l.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.loads.push_back(std::move(l));
  }

  // ---- Shunt capacitors (CAPACITOR) ----
  idx = 0;
  for (const auto& a : elems["CAPACITOR"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Shunt sh;
    sh.index = ++idx;
    sh.name = id;
    sh.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    sh.gs_mw = dbl_from_str(xget(a, {"Gs_MW"}));
    sh.bs_mvar = dbl_from_str(xget(a, {"Bs_Mvar", "Mvar", "Kvar"}));
    sh.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.shunts.push_back(std::move(sh));
  }

  // ---- AC circuit breakers (HVCB, LVCB) ----
  // ETAP wires breakers only through CONNECT records (no FromBus/ToBus attrs);
  // pin 0 = from side, pin 1 = to side. Junction nodes between are preserved.
  auto add_breakers = [&](const char* tag, BreakerType bt) {
    for (const auto& a : elems[to_upper(tag)]) {
      const std::string id = xget(a, {"ID"});
      if (id.empty()) continue;
      CircuitBreaker cb;
      cb.index = static_cast<int>(sys.ac.circuit_breakers.size()) + 1;
      cb.name = id;
      cb.breaker_type = bt;
      std::tie(cb.bus_from, cb.bus_to) =
          resolve_pair(ac2idx, a, {"FromBus"}, {"ToBus"}, 0, 1);
      cb.closed = bool_from_str(xget(a, {"Closed"}), true);
      cb.rated_voltage_kv = dbl_from_str(xget(a, {"RatedKV", "MaxkV"}));
      cb.i_rated_ka = dbl_from_str(xget(a, {"I_rated_kA", "RatedAmp"}), cb.i_rated_ka);
      cb.i_breaking_ka =
          dbl_from_str(xget(a, {"I_breaking_kA", "Rated", "Interrupting"}), cb.i_breaking_ka);
      cb.in_service = bool_from_str(xget(a, {"InService"}), true);
      sys.ac.circuit_breakers.push_back(std::move(cb));
    }
  };
  add_breakers("HVCB", BreakerType::CB);
  add_breakers("LVCB", BreakerType::LS);

  // ---- AC switches (SINGLESWITCH / DOUBLESWITCH / GroundSwitch) ----
  // Mapped best-effort to two-terminal switches; endpoints come from CONNECT.
  auto add_switches = [&](const char* tag) {
    int n = 0;
    for (const auto& a : elems[to_upper(tag)]) {
      const std::string id = xget(a, {"ID"});
      if (id.empty()) continue;
      Switch sw;
      sw.index = static_cast<int>(sys.ac.switches.size()) + 1;
      sw.name = id;
      std::tie(sw.bus_from, sw.bus_to) =
          resolve_pair(ac2idx, a, {"FromBus"}, {"ToBus"}, 0, 1);
      sw.closed = bool_from_str(xget(a, {"Closed"}), true);
      sw.in_service = bool_from_str(xget(a, {"InService"}), true);
      sys.ac.switches.push_back(std::move(sw));
      ++n;
    }
    if (n > 0)
      report.warnings.push_back(std::string("ETAP XML: imported ") + std::to_string(n) +
                                " " + tag + " as two-terminal switch(es) (best-effort)");
  };
  add_switches("SINGLESWITCH");
  add_switches("DOUBLESWITCH");
  add_switches("GROUNDSWITCH");

  // ---- Induction motors (INDMOTOR) ----
  idx = 0;
  for (const auto& a : elems["INDMOTOR"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    AsynchronousMotor m;
    m.index = ++idx;
    m.name = id;
    m.bus = resolve_endpoint(ac2idx, a, {"Bus"}, -1);
    m.vn_kv = dbl_from_str(xget(a, {"KV"}));
    m.sn_mva = dbl_from_str(xget(a, {"MVA"}));
    m.r_pu = dbl_from_str(xget(a, {"R_pu"}));
    m.x_pu = dbl_from_str(xget(a, {"X_pu"}));
    m.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.ac.motors.push_back(std::move(m));
  }

  // ---- DC branches (DCIMPEDANCE) ----
  idx = 0;
  for (const auto& a : elems["DCIMPEDANCE"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    DCBranch br;
    br.index = ++idx;
    br.name = id;
    std::tie(br.from_bus, br.to_bus) =
        resolve_pair(dc2idx, a, {"FromBus"}, {"ToBus"}, 0, 1);
    const double zb = z_base(dc_bus_base_kv(sys, br.from_bus), sys.base_mva);
    br.r_pu = dbl_from_str(xget(a, {"R_ohm", "RValue", "RPosValue"})) / zb;
    br.rate_a_mva = dbl_from_str(xget(a, {"Rate_MVA"}));
    br.length_km = dbl_from_str(xget(a, {"Length_km", "LengthValue"}));
    br.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.dc.branches.push_back(std::move(br));
  }

  // ---- DC loads (DCLUMPLOAD) ----
  idx = 0;
  for (const auto& a : elems["DCLUMPLOAD"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    DCLoad l;
    l.index = ++idx;
    l.name = id;
    l.bus = resolve_endpoint(dc2idx, a, {"Bus"}, -1);
    l.p_mw = dbl_from_str(xget(a, {"KW"})) / 1000.0;
    l.scaling = dbl_from_str(xget(a, {"Scaling"}), 1.0);
    l.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.dc.loads.push_back(std::move(l));
  }

  // ---- DC/DC converters (DCCONVERTER) ----
  idx = 0;
  for (const auto& a : elems["DCCONVERTER"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    DCDCConverter c;
    c.index = ++idx;
    c.name = id;
    std::tie(c.bus_in, c.bus_out) =
        resolve_pair(dc2idx, a, {"InputBus"}, {"OutputBus"}, 0, 1);
    c.p_ref_mw = dbl_from_str(xget(a, {"KW"})) / 1000.0;  // kW -> MW
    c.eta = dbl_from_str(xget(a, {"PercentEFF", "DcPercentEFF"}), 98.0) / 100.0;
    {
      const std::string vin = xget(a, {"Vin_kV"});
      c.vn_in_kv = !vin.empty() ? dbl_from_str(vin)
                                : dbl_from_str(xget(a, {"InputV"})) / 1000.0;  // V -> kV
      const std::string vout = xget(a, {"Vout_kV"});
      c.vn_out_kv = !vout.empty() ? dbl_from_str(vout)
                                  : dbl_from_str(xget(a, {"OutputV"})) / 1000.0;
    }
    c.pmax_mw = dbl_from_str(xget(a, {"Pmax_MW"}));
    c.pmin_mw = dbl_from_str(xget(a, {"Pmin_MW"}));
    c.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.dc.dcdc_converters.push_back(std::move(c));
  }

  // ---- DC circuit breakers (DCCB) ----
  idx = 0;
  for (const auto& a : elems["DCCB"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    DCCircuitBreaker cb;
    cb.index = ++idx;
    cb.name = id;
    std::tie(cb.bus_from, cb.bus_to) =
        resolve_pair(dc2idx, a, {"FromBus"}, {"ToBus"}, 0, 1);
    cb.closed = bool_from_str(xget(a, {"Closed"}), true);
    cb.rated_voltage_kv = dbl_from_str(xget(a, {"RatedKV"}));
    cb.i_breaking_ka = dbl_from_str(xget(a, {"I_breaking_kA", "Rated"}), cb.i_breaking_ka);
    cb.r_ohm = dbl_from_str(xget(a, {"R_ohm", "RValue"}), cb.r_ohm);
    cb.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.dc.dc_circuit_breakers.push_back(std::move(cb));
  }

  // ---- VSC converters (AC<->DC).  ETAP XML exposes the AC bus; the DC terminal
  // is resolved best-effort and may be left unset. ----
  auto add_vsc = [&](const char* tag, const char* type_tag) {
    int vi = static_cast<int>(sys.vsc_converters.size());
    for (const auto& a : elems[to_upper(tag)]) {
      const std::string id = xget(a, {"ID"});
      if (id.empty()) continue;
      VSCConverter v;
      v.index = ++vi;
      v.name = id;
      v.type = type_tag;
      v.bus_ac = resolve_endpoint(ac2idx, a, {"ACBus", "BusID", "Bus"}, -1);
      v.bus_dc = resolve_endpoint(dc2idx, a, {"DCBus", "DcBusID", "OutputBus"}, -1);
      v.control_mode = converter_mode_from_str(xget(a, {"Mode"}));
      {
        const std::string pac = xget(a, {"P_MW"});
        v.p_set_mw = !pac.empty() ? dbl_from_str(pac)
                                  : dbl_from_str(xget(a, {"OpAcKw", "DckW"})) / 1000.0;
      }
      v.q_set_mvar = dbl_from_str(xget(a, {"Q_Mvar"}));
      v.v_dc_set_pu = dbl_from_str(xget(a, {"Vdc_set_pu"}), v.v_dc_set_pu);
      v.v_ac_set_pu = dbl_from_str(xget(a, {"Vac_set_pu"}), v.v_ac_set_pu);
      v.pmax_mw = dbl_from_str(xget(a, {"Pmax_MW"}));
      v.pmin_mw = dbl_from_str(xget(a, {"Pmin_MW"}));
      v.qmax_mvar = dbl_from_str(xget(a, {"Qmax_Mvar"}));
      v.qmin_mvar = dbl_from_str(xget(a, {"Qmin_Mvar"}));
      v.eta = dbl_from_str(xget(a, {"PercentEFF", "DcPercentEFF"}), 99.0) / 100.0;
      v.vn_ac_kv = dbl_from_str(xget(a, {"Vac_kV", "KV"}));
      v.vn_dc_kv = dbl_from_str(xget(a, {"Vdc_kV"}));
      v.in_service = bool_from_str(xget(a, {"InService"}), true);
      sys.vsc_converters.push_back(std::move(v));
    }
  };
  add_vsc("INVERTER", "INVERTER");
  add_vsc("CHARGER", "CHARGER");

  // ---- Batteries (BATTERY) ----
  idx = 0;
  for (const auto& a : elems["BATTERY"]) {
    const std::string id = xget(a, {"ID"});
    if (id.empty()) continue;
    Storage s;
    s.index = ++idx;
    s.name = id;
    s.bus = resolve_endpoint(dc2idx, a, {"Bus"}, -1);
    s.p_mw = dbl_from_str(xget(a, {"P_MW", "OpGenDisChargeMW"}));
    s.pmax_mw = dbl_from_str(xget(a, {"Pmax_MW", "Rated"}));
    s.pmin_mw = dbl_from_str(xget(a, {"Pmin_MW"}));
    s.e_rated_mwh = dbl_from_str(xget(a, {"E_MWh"}));
    s.soc_init = dbl_from_str(xget(a, {"SoC"}), s.soc_init);
    s.soc_min = dbl_from_str(xget(a, {"SoCMin"}), s.soc_min);
    s.soc_max = dbl_from_str(xget(a, {"SoCMax"}), s.soc_max);
    s.eta_charge = dbl_from_str(xget(a, {"EtaCh", "Eff"}), s.eta_charge);
    s.eta_discharge = dbl_from_str(xget(a, {"EtaDis"}), s.eta_discharge);
    s.in_service = bool_from_str(xget(a, {"InService"}), true);
    sys.dc.storage.push_back(std::move(s));
  }

  add_count(report, "BUS", static_cast<int>(sys.ac.buses.size()));
  add_count(report, "XLINE", static_cast<int>(sys.ac.branches.size()));
  add_count(report, "XFORM2W", static_cast<int>(sys.ac.transformers_2w.size()));
  add_count(report, "XFORM3W", static_cast<int>(sys.ac.transformers_3w.size()));
  add_count(report, "UTIL", static_cast<int>(sys.ac.external_grids.size()));
  add_count(report, "SYNGEN", static_cast<int>(sys.ac.generators.size()));
  add_count(report, "PVARRAY", static_cast<int>(sys.ac.pv_systems.size()));
  add_count(report, "WIND", static_cast<int>(sys.ac.renewable_gens.size()));
  add_count(report, "LUMPEDLOAD", static_cast<int>(sys.ac.loads.size()));
  add_count(report, "CAPACITOR", static_cast<int>(sys.ac.shunts.size()));
  add_count(report, "HVCB", static_cast<int>(sys.ac.circuit_breakers.size()));
  add_count(report, "INDMOTOR", static_cast<int>(sys.ac.motors.size()));
  add_count(report, "DCBUS", static_cast<int>(sys.dc.buses.size()));
  add_count(report, "DCIMPEDANCE", static_cast<int>(sys.dc.branches.size()));
  add_count(report, "DCLUMPLOAD", static_cast<int>(sys.dc.loads.size()));
  add_count(report, "DCCONVERTER", static_cast<int>(sys.dc.dcdc_converters.size()));
  add_count(report, "DCCB", static_cast<int>(sys.dc.dc_circuit_breakers.size()));
  add_count(report, "VSC", static_cast<int>(sys.vsc_converters.size()));
  add_count(report, "BATTERY", static_cast<int>(sys.dc.storage.size()));
  add_count(report, "SWITCH", static_cast<int>(sys.ac.switches.size()));

  derive_bus_types(sys);
  validate_refs(sys, mode, report);
  return sys;
}

HybridPowerSystem load_etap_xml(const std::string& path) {
  EtapIoReport report;
  return load_etap_xml(path, EtapImportMode::Permissive, report);
}

// ═════════════════════════════════════════════════════════════════════════
// Native ETAP project XML exporter (PDE document).
// ═════════════════════════════════════════════════════════════════════════
void save_etap_xml(const HybridPowerSystem& sys, const std::string& path,
                   EtapIoReport& report) {
  std::ofstream os(path, std::ios::binary);
  if (!os) throw std::runtime_error("ETAP XML: cannot open for write " + path);

  auto esc = [](const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (char c : s) {
      switch (c) {
        case '&': o += "&amp;"; break;
        case '<': o += "&lt;"; break;
        case '>': o += "&gt;"; break;
        case '"': o += "&quot;"; break;
        case '\'': o += "&apos;"; break;
        default: o += c;
      }
    }
    return o;
  };
  auto num = [](double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.10g", v);
    return std::string(buf);
  };
  auto yn = [](bool b) { return b ? "true" : "false"; };

  int iid = 1000;
  std::unordered_map<int, std::pair<std::string, std::string>> ac_bus, dc_bus;  // idx -> (ID,IID)
  std::ostringstream conns;
  int unresolved = 0;

  auto bus_ac = [&](int i) {
    auto it = ac_bus.find(i);
    return it == ac_bus.end() ? std::pair<std::string, std::string>{"", ""} : it->second;
  };
  auto bus_dc = [&](int i) {
    auto it = dc_bus.find(i);
    return it == dc_bus.end() ? std::pair<std::string, std::string>{"", ""} : it->second;
  };
  // Emit a CONNECT record wiring device pin `pin` to bus `bus_idx`.
  auto connect = [&](const char* dev_tag, const std::string& dev_id,
                     const std::string& dev_iid, int pin, bool dc, int bus_idx) {
    const auto bp = dc ? bus_dc(bus_idx) : bus_ac(bus_idx);
    if (bp.second.empty()) { ++unresolved; return; }
    conns << "    <CONNECT FromElement=\"" << dev_tag << "\" FromID=\"" << esc(dev_id)
          << "\" FromIID=\"" << dev_iid << "\" FromPin=\"" << pin
          << "\" ToElement=\"" << (dc ? "DCBUS" : "BUS") << "\" ToID=\"" << esc(bp.first)
          << "\" ToIID=\"" << bp.second << "\" ToPin=\"0\"/>\n";
  };

  os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  os << "<PDE ProjectName=\"" << esc(sys.name.empty() ? "export" : sys.name)
     << "\" AppVersion=\"hacdcpf\">\n";
  os << "  <PROJECTINFO Frequency=\"" << num(sys.ac.freq_hz)
     << "\" UnitSystem=\"1\" Standard=\"0\" Config=\"Normal\"/>\n";
  os << "  <COMPONENTS/>\n";
  os << "  <LAYOUT>\n";

  // ---- Buses ----
  for (const auto& b : sys.ac.buses) {
    const std::string id = b.name.empty() ? ("BUS" + std::to_string(b.index)) : b.name;
    const std::string ii = std::to_string(iid++);
    ac_bus[b.index] = {id, ii};
    os << "    <BUS ID=\"" << esc(id) << "\" IID=\"" << ii << "\" NominalkV=\""
       << num(b.base_kv) << "\" VMag=\"" << num(b.vm_pu * 100.0) << "\" VAng=\""
       << num(b.va_deg) << "\" VMaxLimit=\"" << num(b.vmax_pu * 100.0)
       << "\" VMinLimit=\"" << num(b.vmin_pu * 100.0) << "\" Area=\"" << b.area
       << "\" Zone=\"" << b.zone << "\" InService=\"" << yn(b.in_service) << "\"/>\n";
  }
  for (const auto& b : sys.dc.buses) {
    const std::string id = b.name.empty() ? ("DCBUS" + std::to_string(b.index)) : b.name;
    const std::string ii = std::to_string(iid++);
    dc_bus[b.index] = {id, ii};
    os << "    <DCBUS ID=\"" << esc(id) << "\" IID=\"" << ii << "\" NominalV=\""
       << num(b.base_kv * 1000.0) << "\" InService=\"" << yn(b.in_service) << "\"/>\n";
  }

  // ---- AC lines (XLINE) — total ohms (OhmsPerLength=Length=1 -> importer scale 1) ----
  for (const auto& br : sys.ac.branches) {
    const std::string id = br.name.empty() ? ("XLINE" + std::to_string(br.index)) : br.name;
    const std::string ii = std::to_string(iid++);
    const double zb = z_base(ac_bus_base_kv(sys, br.from_bus), sys.base_mva);
    os << "    <XLINE ID=\"" << esc(id) << "\" IID=\"" << ii
       << "\" OhmsPerLengthValue=\"1\" LengthValue=\"1\" RPosValue=\"" << num(br.r_pu * zb)
       << "\" XPosValue=\"" << num(br.x_pu * zb) << "\" YPosValue=\"" << num(br.b_pu / zb)
       << "\" RZeroValue=\"" << num(br.r0_pu * zb) << "\" XZeroValue=\"" << num(br.x0_pu * zb)
       << "\" Rate_MVA=\"" << num(br.rate_a_mva) << "\" FromBus=\"" << esc(bus_ac(br.from_bus).first)
       << "\" ToBus=\"" << esc(bus_ac(br.to_bus).first) << "\" InService=\"" << yn(br.in_service)
       << "\"/>\n";
    connect("XLINE", id, ii, 0, false, br.from_bus);
    connect("XLINE", id, ii, 1, false, br.to_bus);
  }

  // ---- 2-winding transformers ----
  for (const auto& t : sys.ac.transformers_2w) {
    const std::string id = t.name.empty() ? ("XFORM2W" + std::to_string(t.index)) : t.name;
    const std::string ii = std::to_string(iid++);
    os << "    <XFORM2W ID=\"" << esc(id) << "\" IID=\"" << ii << "\" PrimkV=\""
       << num(t.vn_hv_kv) << "\" SeckV=\"" << num(t.vn_lv_kv) << "\" AnsiMVA=\""
       << num(t.sn_mva) << "\" ZBaseMVA=\"" << num(t.sn_mva) << "\" AnsiPosZ=\""
       << num(t.vk_percent) << "\" PosR=\"" << num(t.vkr_percent) << "\" AnsiZeroZ=\""
       << num(t.z0_percent) << "\" PrimaryStepPercentTap=\"" << num(t.tap_step_percent)
       << "\" FromBus=\"" << esc(bus_ac(t.hv_bus).first) << "\" ToBus=\""
       << esc(bus_ac(t.lv_bus).first) << "\" InService=\"" << yn(t.in_service) << "\"/>\n";
    connect("XFORM2W", id, ii, 0, false, t.hv_bus);
    connect("XFORM2W", id, ii, 1, false, t.lv_bus);
  }

  // ---- 3-winding transformers ----
  for (const auto& t : sys.ac.transformers_3w) {
    const std::string id = t.name.empty() ? ("XFORM3W" + std::to_string(t.index)) : t.name;
    const std::string ii = std::to_string(iid++);
    os << "    <XFORM3W ID=\"" << esc(id) << "\" IID=\"" << ii << "\" PrimkV=\""
       << num(t.vn_hv_kv) << "\" SeckV=\"" << num(t.vn_mv_kv) << "\" TerkV=\""
       << num(t.vn_lv_kv) << "\" AnsiMVA=\"" << num(t.sn_hv_mva) << "\" PSPosZ=\""
       << num(t.vk_hv_mv_percent) << "\" PTPosZ=\"" << num(t.vk_hv_lv_percent)
       << "\" STPosZ=\"" << num(t.vk_mv_lv_percent) << "\" PrimaryBus=\""
       << esc(bus_ac(t.hv_bus).first) << "\" SecondaryBus=\"" << esc(bus_ac(t.mv_bus).first)
       << "\" TeritiaryBus=\"" << esc(bus_ac(t.lv_bus).first) << "\" InService=\""
       << yn(t.in_service) << "\"/>\n";
    connect("XFORM3W", id, ii, 0, false, t.hv_bus);
    connect("XFORM3W", id, ii, 1, false, t.mv_bus);
    connect("XFORM3W", id, ii, 2, false, t.lv_bus);
  }

  // ---- External grids (UTIL) ----
  for (const auto& g : sys.ac.external_grids) {
    const std::string id = g.name.empty() ? ("UTIL" + std::to_string(g.index)) : g.name;
    const std::string ii = std::to_string(iid++);
    os << "    <UTIL ID=\"" << esc(id) << "\" IID=\"" << ii << "\" KV=\"" << num(g.vn_kv)
       << "\" OpVMag=\"" << num(g.vm_pu * 100.0) << "\" OpVAng=\"" << num(g.va_deg)
       << "\" PosR=\"" << num(g.r_pu) << "\" PosX=\"" << num(g.x_pu) << "\" MVAsc=\""
       << num(g.s_sc_max_mva) << "\" Bus=\"" << esc(bus_ac(g.bus).first) << "\" InService=\""
       << yn(g.in_service) << "\"/>\n";
    connect("UTIL", id, ii, 0, false, g.bus);
  }

  // ---- Synchronous generators ----
  for (const auto& g : sys.ac.generators) {
    const std::string id = g.name.empty() ? ("SYNGEN" + std::to_string(g.index)) : g.name;
    const std::string ii = std::to_string(iid++);
    os << "    <SYNGEN ID=\"" << esc(id) << "\" IID=\"" << ii << "\" KV=\"" << num(g.vn_kv)
       << "\" MW=\"" << num(g.pg_mw) << "\" Mvar=\"" << num(g.qg_mvar) << "\" MVA=\""
       << num(g.mbase_mva) << "\" Pmax_MW=\"" << num(g.pmax_mw) << "\" Pmin_MW=\""
       << num(g.pmin_mw) << "\" Bus=\"" << esc(bus_ac(g.bus).first) << "\" InService=\""
       << yn(g.in_service) << "\"/>\n";
    connect("SYNGEN", id, ii, 0, false, g.bus);
  }

  // ---- PV arrays ----
  for (const auto& p : sys.ac.pv_systems) {
    const std::string id = p.name.empty() ? ("PVA" + std::to_string(p.index)) : p.name;
    const std::string ii = std::to_string(iid++);
    os << "    <PVArray ID=\"" << esc(id) << "\" IID=\"" << ii << "\" PVAPower=\""
       << num(p.p_mw * 1000.0) << "\" Q_Mvar=\"" << num(p.q_mvar) << "\" Bus=\""
       << esc(bus_ac(p.bus).first) << "\" InService=\"" << yn(p.in_service) << "\"/>\n";
    connect("PVArray", id, ii, 0, false, p.bus);
  }

  // ---- Lumped loads ----
  for (const auto& l : sys.ac.loads) {
    const std::string id = l.name.empty() ? ("LOAD" + std::to_string(l.index)) : l.name;
    const std::string ii = std::to_string(iid++);
    const double mva = (l.sn_mva > 0.0) ? l.sn_mva : std::hypot(l.p_mw, l.q_mvar);
    const double pf = (mva > 0.0) ? (l.p_mw / mva) * 100.0 : 100.0;
    os << "    <LUMPEDLOAD ID=\"" << esc(id) << "\" IID=\"" << ii << "\" MVA=\"" << num(mva)
       << "\" PF=\"" << num(pf) << "\" MotorLoadPercent=\"" << num(l.motor_percent)
       << "\" Bus=\"" << esc(bus_ac(l.bus).first) << "\" InService=\"" << yn(l.in_service)
       << "\"/>\n";
    connect("LUMPEDLOAD", id, ii, 0, false, l.bus);
  }

  // ---- Shunt capacitors ----
  for (const auto& sh : sys.ac.shunts) {
    const std::string id = sh.name.empty() ? ("CAP" + std::to_string(sh.index)) : sh.name;
    const std::string ii = std::to_string(iid++);
    os << "    <CAPACITOR ID=\"" << esc(id) << "\" IID=\"" << ii << "\" Mvar=\""
       << num(sh.bs_mvar) << "\" Bus=\"" << esc(bus_ac(sh.bus).first) << "\" InService=\""
       << yn(sh.in_service) << "\"/>\n";
    connect("CAPACITOR", id, ii, 0, false, sh.bus);
  }

  // ---- Induction motors ----
  for (const auto& m : sys.ac.motors) {
    const std::string id = m.name.empty() ? ("MTR" + std::to_string(m.index)) : m.name;
    const std::string ii = std::to_string(iid++);
    os << "    <INDMOTOR ID=\"" << esc(id) << "\" IID=\"" << ii << "\" KV=\"" << num(m.vn_kv)
       << "\" MVA=\"" << num(m.sn_mva) << "\" R_pu=\"" << num(m.r_pu) << "\" X_pu=\""
       << num(m.x_pu) << "\" Bus=\"" << esc(bus_ac(m.bus).first) << "\" InService=\""
       << yn(m.in_service) << "\"/>\n";
    connect("INDMOTOR", id, ii, 0, false, m.bus);
  }

  // ---- AC breakers / switches ----
  for (const auto& cb : sys.ac.circuit_breakers) {
    const char* tag = (cb.breaker_type == BreakerType::CB) ? "HVCB" : "LVCB";
    const std::string id = cb.name.empty() ? (std::string(tag) + std::to_string(cb.index)) : cb.name;
    const std::string ii = std::to_string(iid++);
    os << "    <" << tag << " ID=\"" << esc(id) << "\" IID=\"" << ii << "\" Closed=\""
       << yn(cb.closed) << "\" RatedKV=\"" << num(cb.rated_voltage_kv) << "\" InService=\""
       << yn(cb.in_service) << "\"/>\n";
    connect(tag, id, ii, 0, false, cb.bus_from);
    connect(tag, id, ii, 1, false, cb.bus_to);
  }
  for (const auto& sw : sys.ac.switches) {
    const std::string id = sw.name.empty() ? ("SW" + std::to_string(sw.index)) : sw.name;
    const std::string ii = std::to_string(iid++);
    os << "    <SINGLESWITCH ID=\"" << esc(id) << "\" IID=\"" << ii << "\" Closed=\""
       << yn(sw.closed) << "\" InService=\"" << yn(sw.in_service) << "\"/>\n";
    connect("SINGLESWITCH", id, ii, 0, false, sw.bus_from);
    connect("SINGLESWITCH", id, ii, 1, false, sw.bus_to);
  }

  // ---- VSC converters (INVERTER / CHARGER) ----
  for (const auto& v : sys.vsc_converters) {
    const std::string tag = v.type.empty() ? "INVERTER" : to_upper(v.type);
    const std::string id = v.name.empty() ? (tag + std::to_string(v.index)) : v.name;
    const std::string ii = std::to_string(iid++);
    os << "    <" << tag << " ID=\"" << esc(id) << "\" IID=\"" << ii << "\" P_MW=\""
       << num(v.p_set_mw) << "\" Q_Mvar=\"" << num(v.q_set_mvar) << "\" PercentEFF=\""
       << num(v.eta * 100.0) << "\" BusID=\"" << esc(bus_ac(v.bus_ac).first)
       << "\" InService=\"" << yn(v.in_service) << "\"/>\n";
    connect(tag.c_str(), id, ii, 0, false, v.bus_ac);
    connect(tag.c_str(), id, ii, 1, true, v.bus_dc);
  }

  // ---- DC loads ----
  for (const auto& l : sys.dc.loads) {
    const std::string id = l.name.empty() ? ("DCLOAD" + std::to_string(l.index)) : l.name;
    const std::string ii = std::to_string(iid++);
    os << "    <DCLUMPLOAD ID=\"" << esc(id) << "\" IID=\"" << ii << "\" KW=\""
       << num(l.p_mw * 1000.0) << "\" Bus=\"" << esc(bus_dc(l.bus).first) << "\" InService=\""
       << yn(l.in_service) << "\"/>\n";
    connect("DCLUMPLOAD", id, ii, 0, true, l.bus);
  }

  // ---- DC/DC converters ----
  for (const auto& c : sys.dc.dcdc_converters) {
    const std::string id = c.name.empty() ? ("DCCONV" + std::to_string(c.index)) : c.name;
    const std::string ii = std::to_string(iid++);
    os << "    <DCCONVERTER ID=\"" << esc(id) << "\" IID=\"" << ii << "\" KW=\""
       << num(c.p_ref_mw * 1000.0) << "\" PercentEFF=\"" << num(c.eta * 100.0)
       << "\" InputV=\"" << num(c.vn_in_kv * 1000.0) << "\" OutputV=\""
       << num(c.vn_out_kv * 1000.0) << "\" InputBus=\"" << esc(bus_dc(c.bus_in).first)
       << "\" OutputBus=\"" << esc(bus_dc(c.bus_out).first) << "\" InService=\""
       << yn(c.in_service) << "\"/>\n";
    connect("DCCONVERTER", id, ii, 0, true, c.bus_in);
    connect("DCCONVERTER", id, ii, 1, true, c.bus_out);
  }

  // ---- Batteries ----
  for (const auto& s : sys.dc.storage) {
    const std::string id = s.name.empty() ? ("BATT" + std::to_string(s.index)) : s.name;
    const std::string ii = std::to_string(iid++);
    os << "    <BATTERY ID=\"" << esc(id) << "\" IID=\"" << ii << "\" Rated=\""
       << num(s.pmax_mw) << "\" SoC=\"" << num(s.soc_init) << "\" Bus=\""
       << esc(bus_dc(s.bus).first) << "\" InService=\"" << yn(s.in_service) << "\"/>\n";
    connect("BATTERY", id, ii, 0, true, s.bus);
  }

  // ---- DC breakers ----
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    const std::string id = cb.name.empty() ? ("DCCB" + std::to_string(cb.index)) : cb.name;
    const std::string ii = std::to_string(iid++);
    os << "    <DCCB ID=\"" << esc(id) << "\" IID=\"" << ii << "\" Closed=\""
       << yn(cb.closed) << "\" RatedKV=\"" << num(cb.rated_voltage_kv) << "\" InService=\""
       << yn(cb.in_service) << "\"/>\n";
    connect("DCCB", id, ii, 0, true, cb.bus_from);
    connect("DCCB", id, ii, 1, true, cb.bus_to);
  }

  os << "  </LAYOUT>\n";
  os << "  <CONNECTIONS>\n" << conns.str() << "  </CONNECTIONS>\n";
  os << "</PDE>\n";
  os.flush();
  if (!os) throw std::runtime_error("ETAP XML: write failed for " + path);

  add_count(report, "BUS", static_cast<int>(sys.ac.buses.size()));
  add_count(report, "DCBUS", static_cast<int>(sys.dc.buses.size()));
  add_count(report, "XLINE", static_cast<int>(sys.ac.branches.size()));
  add_count(report, "XFORM2W", static_cast<int>(sys.ac.transformers_2w.size()));
  add_count(report, "XFORM3W", static_cast<int>(sys.ac.transformers_3w.size()));
  add_count(report, "UTIL", static_cast<int>(sys.ac.external_grids.size()));
  add_count(report, "SYNGEN", static_cast<int>(sys.ac.generators.size()));
  add_count(report, "PVARRAY", static_cast<int>(sys.ac.pv_systems.size()));
  add_count(report, "LUMPEDLOAD", static_cast<int>(sys.ac.loads.size()));
  add_count(report, "CAPACITOR", static_cast<int>(sys.ac.shunts.size()));
  add_count(report, "INDMOTOR", static_cast<int>(sys.ac.motors.size()));
  add_count(report, "HVCB", static_cast<int>(sys.ac.circuit_breakers.size()));
  add_count(report, "SWITCH", static_cast<int>(sys.ac.switches.size()));
  add_count(report, "VSC", static_cast<int>(sys.vsc_converters.size()));
  add_count(report, "DCLUMPLOAD", static_cast<int>(sys.dc.loads.size()));
  add_count(report, "DCCONVERTER", static_cast<int>(sys.dc.dcdc_converters.size()));
  add_count(report, "BATTERY", static_cast<int>(sys.dc.storage.size()));
  add_count(report, "DCCB", static_cast<int>(sys.dc.dc_circuit_breakers.size()));
  if (unresolved > 0)
    report.warnings.push_back("ETAP XML export: " + std::to_string(unresolved) +
                              " device endpoint(s) had no bus and were left unconnected");
}

void save_etap_xml(const HybridPowerSystem& sys, const std::string& path) {
  EtapIoReport report;
  save_etap_xml(sys, path, report);
}

}  // namespace hacdcpf::io
