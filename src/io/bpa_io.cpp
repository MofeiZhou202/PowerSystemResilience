/// io/bpa_io.cpp
/// ===============
/// PSD-BPA / DSP card-file (.dat) importer — see bpa_io.hpp.
///
/// Card columns are 1-based and inclusive, fixed-width.  Numeric fields may
/// carry an explicit decimal point anywhere in the field; when the point is
/// omitted some fields use an implied-decimal convention (Vsch: F4.3,
/// transformer taps: F6.2).  Blank numeric fields default to zero / absent.
/// The BZ/BZ+/LZ (VSC-HVDC) column layout is not covered by the dat card
/// manual; it was established empirically against DSP Pwrflow (see the
/// BzStation struct comment and docs/bpa_dsp_component_mapping.md §5).
///
/// GBK-encoded files (Chinese bus names, e.g. IEEE90.dat) are parsed in the
/// raw byte domain so fixed columns stay aligned (GBK characters are 2 bytes);
/// only the extracted name tokens are converted to UTF-8 for the model.

#include "hacdcpf/io/bpa_io.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <complex>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace hacdcpf::io {
namespace {

constexpr double kBpaOpenActivePowerBoundMw = 9999.0;
constexpr double kBpaMinLineReactancePu = 1e-4;

// ── Small parsing helpers ────────────────────────────────────────────────────

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

/// Raw fixed field (1-based inclusive columns), clipped to the line length.
std::string raw_field(const std::string& line, size_t first, size_t last) {
  if (first == 0 || first > last || first > line.size()) return {};
  const size_t begin = first - 1;
  const size_t len = std::min(last, line.size()) - begin;
  return line.substr(begin, len);
}

std::string field(const std::string& line, size_t first, size_t last) {
  return trim(raw_field(line, first, last));
}

/// Parse a plain numeric field.  Returns 0 and clears @p present when blank.
double num(const std::string& f, bool& present) {
  present = false;
  if (f.empty()) return 0.0;
  try {
    size_t used = 0;
    const double v = std::stod(f, &used);
    present = true;
    return v;
  } catch (...) {
    return 0.0;
  }
}

/// Numeric field with implied decimals when no '.' is written (e.g. Vsch
/// "1047" -> 1.047 with scale 1000; tap "50000" -> 500.0 with scale 100).
double num_scaled(const std::string& f, double scale_if_no_dot, bool& present) {
  const double v = num(f, present);
  if (!present) return 0.0;
  if (f.find('.') == std::string::npos && f.find('e') == std::string::npos &&
      f.find('E') == std::string::npos) {
    return v / scale_if_no_dot;
  }
  return v;
}

bool is_valid_utf8(const std::string& s) {
  size_t i = 0;
  while (i < s.size()) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) { ++i; continue; }
    size_t extra = 0;
    if ((c & 0xE0) == 0xC0) extra = 1;
    else if ((c & 0xF0) == 0xE0) extra = 2;
    else if ((c & 0xF8) == 0xF0) extra = 3;
    else return false;
    if (i + extra >= s.size()) return false;
    for (size_t k = 1; k <= extra; ++k) {
      if ((static_cast<unsigned char>(s[i + k]) & 0xC0) != 0x80) return false;
    }
    i += extra + 1;
  }
  return true;
}

/// Convert a GBK (CP936) byte string to UTF-8 while preserving the raw-byte
/// fixed-column parsing performed before this function is called.
std::string gbk_to_utf8(const std::string& s) {
#ifdef _WIN32
  if (s.empty()) return s;
  const int wlen =
      MultiByteToWideChar(936 /*CP_GBK*/, 0, s.c_str(), -1, nullptr, 0);
  if (wlen <= 0) return s;
  std::wstring w(static_cast<size_t>(wlen), L'\0');
  MultiByteToWideChar(936, 0, s.c_str(), -1, w.data(), wlen);
  const int ulen =
      WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
  if (ulen <= 0) return s;
  std::string u(static_cast<size_t>(ulen), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, u.data(), ulen, nullptr, nullptr);
  if (!u.empty() && u.back() == '\0') u.pop_back();
  return u;
#else
  if (s.empty()) return s;
  iconv_t converter = iconv_open("UTF-8", "GBK");
  if (converter == reinterpret_cast<iconv_t>(-1)) return s;

  char* input = const_cast<char*>(s.data());
  size_t input_left = s.size();
  std::string converted(std::max<size_t>(32, s.size() * 2), '\0');
  size_t output_used = 0;

  while (input_left > 0) {
    char* output = converted.data() + output_used;
    size_t output_left = converted.size() - output_used;
    const size_t rc =
        iconv(converter, &input, &input_left, &output, &output_left);
    output_used = converted.size() - output_left;
    if (rc != static_cast<size_t>(-1)) break;
    if (errno != E2BIG) {
      iconv_close(converter);
      return s;
    }
    converted.resize(converted.size() * 2);
  }

  iconv_close(converter);
  converted.resize(output_used);
  return converted;
#endif
}

// ── Importer internals ───────────────────────────────────────────────────────

// Impedance/admittance fields on L and T cards use implied decimals when the
// decimal point is omitted: R/X and G/B are all F6.5 (e.g. "3000" -> 0.03 pu,
// "400000" -> 4.0 pu).  Fields with an explicit '.' are taken verbatim.
// (Confirmed against the DSP 2DC tutorial case: only "400000" = B/2 4.0 pu
// closes DSP's own reactive balance — its generators absorb the excess line
// charging, exactly as DSP's solved labels show; 0.4 pu would leave the
// network ~2900 Mvar short and drive every generator over its Q limit.)
double num_rx(const std::string& f, bool& present) {
  return num_scaled(f, 1e5, present);
}
double num_gb(const std::string& f, bool& present) {
  return num_scaled(f, 1e5, present);
}

/// Pending two-terminal HVDC converter-station data (BD card), linked to the
/// DC line (LD card) processed afterwards.
struct BdStation {
  std::string name;       // valve-side node name (also the AC bus name)
  int ac_bus{0};          // AC bus index of the valve-side node
  int dc_bus{0};          // DC bus index created for this station
  double ac_kv{0.0};      // valve-side AC base voltage (kV), cols 15-18
  double dc_kv{0.0};      // rated DC voltage from the BD card (may be 0)
  double bridges{0.0};
  double sr_mh{0.0};      // smoothing reactor (mH), cols 26-30 (dynamic only)
  double alpha_min_deg{0.0};  // min firing angle (deg), cols 31-35
  double alpha_stop_deg{0.0}; // max firing angle (deg), cols 36-40
  double vdrop_v{0.0};    // valve voltage drop per bridge (V), cols 41-45
  double bridge_in_a{0.0};// rated bridge current (A), cols 46-50
  std::string primary_name; // converter-transformer primary AC bus, cols 51-58
  double primary_kv{0.0}; // primary-side AC base voltage (kV), cols 59-62
};

/// Pending LCC link (LccQuasiSteady path): the LD card has been consumed and
/// the DCBranch created, but the LCCConverter pair is materialized only after
/// the whole file is parsed, because the converter-transformer T card may
/// follow the LD card (e.g. 2DC.dat).
struct PendingLccLink {
  std::string locator;
  std::string rect_name;
  std::string inv_name;
  double p_sch{0.0};
  double vdc_kv{0.0};
  double inv_vdc_set_kv{0.0};
  double alpha_n_deg{0.0};
  double gamma_n_deg{0.0};
};

/// R card attached to one T-card transformer. R is a control record, not a
/// second electrical branch. For two-terminal LCC links it supplies the
/// adjustable winding and tap bounds consumed by the power-flow control loop.
struct RControl {
  std::string locator;
  std::string name1;
  std::string name2;
  double kv1{0.0};
  double kv2{0.0};
  int adjustable_terminal{0};
  std::string controlled_name;
  double controlled_kv{0.0};
  double tap_max_kv{0.0};
  double tap_min_kv{0.0};
  // Source columns 56-57. DSP perturbation tests on the CIGRE LCC case show
  // that this value does not discretize converter-transformer tap control:
  // blank, 02, 25 and 50 produce identical taps and LCC operating points.
  // Retain it as source metadata, but do not map it to LCC discrete positions.
  int tap_count{0};
};

struct TransformerCardData {
  int branch_index{-1};
  std::string name1;
  std::string name2;
  double kv1{0.0};
  double kv2{0.0};
  double tap1_kv{0.0};
  double tap2_kv{0.0};
};

/// Pending VSC-HVDC station (BZ card, plus the BZ+ continuation card when
/// present).  The VSCConverter element is materialized only after the whole
/// file is parsed (see run()), mirroring the pending-LCC pattern, so the BZ+
/// card and the DC base voltage are known.  Column layout was established
/// empirically against DSP 2.1.47 Pwrflow (single-field perturbation runs on
/// a synthetic two-terminal VSC case, cross-checked against a production
/// grid dat file; see docs/bpa_dsp_component_mapping.md §5):
///   BZ : 7-14 station (= AC bus) name, 15-18 AC base kV, 19-20 zone,
///        21-25 rated capacity Sn (MVA, informational), 34-37 a pu reactance
///        (steady-state inert in DSP; dynamic-suspect, not imported),
///        51-55 Rc converter resistance (pu on the converter base — DSP books
///        |P|*Rc as the station active loss), 64 number of poles (the DC
///        network sees the LZ resistance divided by this), 67-70 rated DC kV
///        (informational; superseded by the BZ+ value).
///   BZ+: 7-14 station name, 20-24 P setpoint (MW; >0 ABSORBS from the AC
///        grid = rectifier, <0 injects = inverter — the BPA load sign
///        convention, verified against DSP branch-flow signs), 26-29 Q
///        setpoint (Mvar; >0 ABSORBS reactive power), 34 control-mode flag
///        (1 = constant DC
///        voltage station; blank/other = constant P/Q), 36-39 rated DC
///        voltage UdcN (kV; DSP requires it > 0 and equal for every station
///        of one DC network), 41-44 DC voltage setpoint (kV; mode-1
///        stations), 53-58 Xc converter reactance (pu; internal reactive
///        loss only — invisible at the AC terminal, not imported),
///        60-63 / 66-69 AC/DC voltage references (informational).
struct BzStation {
  std::string name;         // station name (also the AC bus name)
  std::string locator;      // locator of the BZ card
  int ac_bus{0};            // AC bus index of the station bus
  int dc_bus{0};            // DC bus index created for this station
  double ac_kv{0.0};        // AC base voltage (kV), cols 15-18
  double sn_mva{0.0};       // rated capacity (MVA), cols 21-25 (informational)
  double rc_pu{0.0};        // converter resistance (pu), cols 51-55
  int n_poles{1};           // number of poles, col 64 (blank -> 1)
  double udcn_bz_kv{0.0};   // rated DC voltage from the BZ card, cols 67-70
  // BZ+ continuation card fields (has_plus = false until one is seen):
  bool has_plus{false};
  int mode_flag{0};         // col 34: 1 = constant DC voltage, else const P/Q
  double p_set_mw{0.0};     // cols 20-24 (sign: >0 absorbs = rectifier)
  double q_card_mvar{0.0};  // cols 26-29 (sign: >0 absorbs reactive power)
  double udcn_kv{0.0};      // cols 36-39 (required > 0 by DSP)
  double udc_set_kv{0.0};   // cols 41-44 (mode-1 stations)
  double xc_pu{0.0};        // cols 53-58 (internal reactive loss; not imported)
};

/// Pending VSC-HVDC line (LZ card).  Steady state uses only the resistance;
/// L / C / smoothing reactors / length are dynamic-only and ignored (same
/// treatment as the LD card).  Columns: 7-14 / 20-27 terminal BZ names,
/// 15-18 / 28-31 terminal nominal kV (informational), 34-37 rated current
/// (A), 38-43 resistance per pole (ohm; the DC network sees R/n_poles).
struct PendingLzLink {
  std::string locator;
  std::string name1;
  std::string name2;
  double i_rated_a{0.0};
  double r_ohm{0.0};
  double inductance_mh{0.0};
};

/// BA/BA1/BA2 form one layered LCC station record.  BA names the external
/// AC/DC terminal, BA1 supplies the built-in converter transformer, and BA2
/// supplies the steady-state converter limits and setpoints.
struct BaStation {
  std::string locator;
  std::string name;
  double primary_kv{0.0};
  std::string role;
  std::string layer;
  int bridges{1};
  double smoothing_reactor_mh{0.0};
  double power_percent{100.0};
  double q_compensation_mvar{0.0};

  bool has_ba1{false};
  double valve_kv{0.0};
  double transformer_sn_mva{0.0};
  double transformer_x_pu{0.0};
  double tap_max_kv{0.0};
  double tap_min_kv{0.0};

  bool has_ba2{false};
  double v_drop_v{0.0};
  double rated_current_a{0.0};
  double alpha_min_deg{0.0};
  double alpha_stop_deg{0.0};
  double normal_angle_deg{0.0};
  double rated_dc_kv{0.0};
  int dc_bus{0};
};

/// DC is the system-level control record for a layered/hybrid LCC scheme.
/// The four terminals are high/low rectifier and high/low inverter positions;
/// a blank name denotes an unused position.
struct LayeredDcControl {
  std::string locator;
  std::string rectifier_high;
  double rectifier_high_kv{0.0};
  std::string rectifier_low;
  double rectifier_low_kv{0.0};
  std::string inverter_high;
  double inverter_high_kv{0.0};
  std::string inverter_low;
  double inverter_low_kv{0.0};
  int system_type{0};
  int control_mode{0};
  int inverter_layer_mode{0};
  double p_sch_mw{0.0};
  double rectifier_v_sch_kv{0.0};
  double inverter_v_sch_kv{0.0};
  int converter_calculation_mode{1};
};

struct BmStation {
  BdStation station;
  std::string locator;
  std::string role;
  double normal_angle_deg{0.0};
  double gamma_min_deg{0.0};
  double p_sch_mw{0.0};
  double v_sch_kv{0.0};
};

struct PendingNativeDcLine {
  std::string locator;
  std::string source_card;
  std::string name1;
  double kv1{0.0};
  std::string name2;
  double kv2{0.0};
  double i_rated_a{0.0};
  double r_ohm{0.0};
  double inductance_mh{0.0};
  double capacitance_uf{0.0};
  double length_km{0.0};
};

/// L+ continuation data for line shunt reactors. The two Mvar values belong
/// to the terminal buses of an existing L card; L+ never creates a branch.
struct PendingLineShunt {
  std::string locator;
  std::string name1;
  std::string name2;
  double kv1{0.0};
  double kv2{0.0};
  std::string circuit;
  double q1_mvar{0.0};
  double q2_mvar{0.0};
};

struct Importer {
  explicit Importer(const BpaImportOptions& opt,
                    BpaSmallReactanceMode reactance_mode)
      : options(opt), small_reactance_mode(reactance_mode) {}

  BpaImportOptions options;
  BpaSmallReactanceMode small_reactance_mode{
      BpaSmallReactanceMode::DspCompatible};
  BpaImportResult result;
  bool names_need_gbk{false};
  double mva_base{100.0};
  std::string case_id;
  std::string project;

  std::unordered_map<std::string, int> ac_bus_by_key;
  std::unordered_set<int> ac_bus_stubs;
  std::unordered_map<std::string, int> dc_bus_by_name;
  std::unordered_map<std::string, BdStation> bd_by_name;
  std::vector<PendingLccLink> pending_lcc_links;
  std::vector<RControl> r_controls;
  std::unordered_map<int, TransformerCardData> transformer_card_data;
  std::unordered_map<std::string, BzStation> bz_by_name;
  std::vector<std::string> bz_order;  // BZ card order -> stable VSC .index
  std::vector<PendingLzLink> pending_lz_links;
  std::unordered_map<std::string, BaStation> ba_by_name;
  std::vector<std::string> ba_order;
  std::vector<LayeredDcControl> layered_dc_controls;
  std::unordered_map<std::string, BmStation> bm_by_name;
  std::vector<std::string> bm_order;
  std::vector<PendingNativeDcLine> pending_native_dc_lines;
  std::vector<PendingLineShunt> pending_line_shunts;
  bool g_half_note_emitted{false};
  bool magnetizing_note_emitted{false};
  int current_source_order{-1};

  std::string name_of(const std::string& raw) const {
    return names_need_gbk ? gbk_to_utf8(trim(raw)) : trim(raw);
  }

  void warn(ImportDisposition disp, ImportReasonCode code,
            const std::string& locator, const std::string& msg) {
    result.report.add(disp, code, ImportSeverity::Warning, locator, msg);
  }

  static double normalized_ac_kv(double base_kv) {
    return base_kv > 0.0 ? base_kv : 110.0;
  }

  static std::string ac_bus_key(const std::string& name, double base_kv) {
    // BPA identifies a terminal by fixed-width name and nominal voltage.
    const auto kv_millivolts = static_cast<long long>(
        std::llround(normalized_ac_kv(base_kv) * 1000.0));
    return name + '\x1f' + std::to_string(kv_millivolts);
  }

  int find_ac_bus(const std::string& name, double base_kv) const {
    const auto it = ac_bus_by_key.find(ac_bus_key(name, base_kv));
    return it == ac_bus_by_key.end() ? 0 : it->second;
  }

  /// Fetch (or create as PQ stub) an AC bus by UTF-8 name.  When
  /// @p quiet is true the creation is not reported (used by cards that are
  /// themselves a bus declaration, e.g. BD).
  int ensure_ac_bus(const std::string& name, double base_kv,
                    const std::string& locator, bool quiet = false) {
    const std::string key = ac_bus_key(name, base_kv);
    auto it = ac_bus_by_key.find(key);
    if (it != ac_bus_by_key.end()) return it->second;
    ACBus bus;
    bus.index = static_cast<int>(result.system.ac.buses.size()) + 1;
    bus.bus_type = BusType::PQ;
    bus.base_kv = normalized_ac_kv(base_kv);
    bus.name = name;
    if (!quiet) {
      warn(ImportDisposition::Coerced, ImportReasonCode::UnresolvedBusRef,
           locator, "Bus '" + name + "' referenced before declaration; created "
                    "as PQ stub (base_kv from referencing card).");
    }
    result.system.ac.buses.push_back(bus);
    ac_bus_by_key.emplace(key, bus.index);
    ac_bus_stubs.insert(bus.index);
    return bus.index;
  }

  int ensure_dc_bus(const std::string& name, double base_kv) {
    auto it = dc_bus_by_name.find(name);
    if (it != dc_bus_by_name.end()) {
      DCBus& existing =
          result.system.dc.buses[static_cast<size_t>(it->second) - 1];
      if (existing.base_kv <= 0.0 && base_kv > 0.0) existing.base_kv = base_kv;
      return it->second;
    }
    DCBus bus;
    bus.index = static_cast<int>(result.system.dc.buses.size()) + 1;
    bus.bus_type = DCBusType::DC_P;
    bus.base_kv = base_kv;
    bus.name = name;
    result.system.dc.buses.push_back(bus);
    dc_bus_by_name.emplace(name, bus.index);
    return bus.index;
  }

  // ── AC bus cards: B / BS / BE / BQ ─────────────────────────────────────
  void parse_bus_card(const std::string& line, const std::string& locator) {
    const std::string type = trim(raw_field(line, 1, 2));
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, type + " card without a bus name; skipped.");
      return;
    }
    const double kv = num(field(line, 15, 18), present);
    const double pload = num(field(line, 21, 25), present);
    const double qload = num(field(line, 26, 30), present);
    const double pshunt = num(field(line, 31, 34), present);
    const double qshunt = num(field(line, 35, 38), present);
    const double pgenmax = num(field(line, 39, 42), present);
    const double pgen = num(field(line, 43, 47), present);
    const double qgenmax = num(field(line, 48, 52), present);
    const double qgenmin = num(field(line, 53, 57), present);
    const double vsch = num_scaled(field(line, 58, 61), 1000.0, present);
    const double tail = num_scaled(field(line, 62, 65), 1000.0, present);

    ACBus bus;
    bus.base_kv = normalized_ac_kv(kv);
    bus.name = name;
    const std::string zone = field(line, 19, 20);
    if (!zone.empty() && std::isdigit(static_cast<unsigned char>(zone[0]))) {
      bus.zone = std::atoi(zone.c_str());
    }
    if (type == "BS") {
      bus.bus_type = BusType::SLACK;
      bus.vm_pu = vsch > 0.0 ? vsch : 1.0;
      bus.va_deg = tail;  // BS trailing field is the scheduled angle (deg).
    } else if (type == "BE" || type == "BQ") {
      bus.bus_type = BusType::PV;
      bus.vm_pu = vsch > 0.0 ? vsch : 1.0;
      bus.vmin_pu = tail > 0.0 ? tail : bus.vmin_pu;
    } else {
      bus.bus_type = BusType::PQ;
      bus.vmax_pu = vsch > 0.0 ? vsch : bus.vmax_pu;
      bus.vmin_pu = tail > 0.0 ? tail : bus.vmin_pu;
    }
    const std::string key = ac_bus_key(name, kv);
    const auto existing = ac_bus_by_key.find(key);
    if (existing == ac_bus_by_key.end()) {
      bus.index = static_cast<int>(result.system.ac.buses.size()) + 1;
      result.system.ac.buses.push_back(bus);
      ac_bus_by_key.emplace(key, bus.index);
    } else if (ac_bus_stubs.erase(existing->second) > 0) {
      // Preserve the stable ID already used by preceding L/T/BD/BZ cards.
      bus.index = existing->second;
      result.system.ac.buses[static_cast<size_t>(bus.index) - 1] = bus;
    } else {
      warn(ImportDisposition::Rejected, ImportReasonCode::DuplicateId,
           locator, "Duplicate " + type + " bus declaration for '" + name +
                        "' at " + std::to_string(bus.base_kv) +
                        " kV; skipped.");
      return;
    }

    if (std::abs(pload) > 0.0 || std::abs(qload) > 0.0) {
      Load ld;
      ld.index = static_cast<int>(result.system.ac.loads.size());
      ld.bus = bus.index;
      ld.p_mw = pload;
      ld.q_mvar = qload;
      ld.name = name;
      result.system.ac.loads.push_back(std::move(ld));
    }
    if (std::abs(pshunt) > 0.0 || std::abs(qshunt) > 0.0) {
      Shunt sh;
      sh.index = static_cast<int>(result.system.ac.shunts.size());
      sh.bus = bus.index;
      sh.gs_mw = pshunt;
      sh.bs_mvar = qshunt;  // BPA: positive Qshunt is capacitive, matches bs>0.
      sh.name = name;
      result.system.ac.shunts.push_back(std::move(sh));
    }
    const bool has_generation_data =
        std::abs(pgenmax) > 0.0 || std::abs(pgen) > 0.0 ||
        std::abs(qgenmax) > 0.0 || std::abs(qgenmin) > 0.0;
    if (type != "B" || has_generation_data) {
      Generator gen;
      gen.index = static_cast<int>(result.system.ac.generators.size());
      gen.bus = bus.index;
      gen.pg_mw = pgen;
      // On a plain B (PQ) card, columns 48-52 are the scheduled fixed
      // reactive injection.  The same columns are Qmax on BE/BQ/BS cards.
      if (type == "B") gen.qg_mvar = qgenmax;
      gen.vg_pu = vsch > 0.0 ? vsch : 1.0;
      if (type == "BS") {
        gen.pmin_mw = -kBpaOpenActivePowerBoundMw;
        gen.pmax_mw = pgenmax > 0.0 ? pgenmax
                                    : kBpaOpenActivePowerBoundMw;
      } else {
        gen.pmin_mw = 0.0;
        gen.pmax_mw = pgenmax;
      }
      if (type == "B") {
        gen.qmax_mvar = qgenmax;
        gen.qmin_mvar = qgenmax;
      } else {
        gen.qmax_mvar = qgenmax;
        gen.qmin_mvar = qgenmin;
      }
      gen.is_slack = (type == "BS");
      gen.bpa_is_bq = (type == "BQ");
      gen.bpa_source_order = gen.bpa_is_bq ? current_source_order : -1;
      gen.name = name;
      result.system.ac.generators.push_back(std::move(gen));
    }
  }

  // ── AC line card: L ────────────────────────────────────────────────────
  void parse_line_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string n1 = name_of(raw_field(line, 7, 14));
    const std::string n2 = name_of(raw_field(line, 20, 27));
    const double kv1 = num(field(line, 15, 18), present);
    const double kv2 = num(field(line, 28, 31), present);
    if (n1.empty() || n2.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "L card with missing terminal name; skipped.");
      return;
    }
    ACBranch br;
    br.index = static_cast<int>(result.system.ac.branches.size());
    br.from_bus = ensure_ac_bus(n1, kv1, locator);
    br.to_bus = ensure_ac_bus(n2, kv2, locator);
    br.r_pu = num_rx(field(line, 39, 44), present);
    br.x_pu = num_rx(field(line, 45, 50), present);
    const double source_r_pu = br.r_pu;
    const double source_x_pu = br.x_pu;
    const double g_half = num_gb(field(line, 51, 56), present);
    const double b_half = num_gb(field(line, 57, 62), present);
    br.b_pu = 2.0 * b_half;  // card stores B/2 per side; model stores total.
    if (std::abs(g_half) > 0.0 && !g_half_note_emitted) {
      g_half_note_emitted = true;
      warn(ImportDisposition::Coerced, ImportReasonCode::StructuralLoss, locator,
           "Line shunt conductance G/2 is not modeled (ACBranch carries no G); "
           "ignored. Further occurrences are not reported individually.");
    }
    br.length_km = num(field(line, 63, 68), present);
    const double i_rated = num(field(line, 34, 37), present);
    if (i_rated > 0.0 && kv1 > 0.0) {
      br.rate_a_mva = std::sqrt(3.0) * kv1 * i_rated / 1000.0;
    }
    const double n_par = num(field(line, 38, 38), present);
    if (n_par > 0.0) br.n_parallel = static_cast<int>(std::lround(n_par));
    const std::string ckt = field(line, 32, 32);
    br.name = "L_" + n1 + "_" + n2 + (ckt.empty() ? "" : "_" + ckt);
    const bool ideal_source =
        source_r_pu == 0.0 && source_x_pu == 0.0 && g_half == 0.0 &&
        br.b_pu == 0.0 && br.length_km == 0.0 && br.rate_a_mva == 0.0;
    if (small_reactance_mode == BpaSmallReactanceMode::DspCompatible &&
        std::abs(br.x_pu) < kBpaMinLineReactancePu && !ideal_source) {
      const double source_x = br.x_pu;
      br.x_pu = source_x == 0.0
                    ? kBpaMinLineReactancePu
                    : std::copysign(kBpaMinLineReactancePu, source_x);
      std::ostringstream msg;
      msg << "L-card reactance " << source_x
          << " pu is below the DSP numerical floor; coerced to "
          << br.x_pu << " pu.";
      warn(ImportDisposition::Coerced, ImportReasonCode::RangeCoerced,
           locator, msg.str());
    }
    // A BPA L card with no series/shunt/rating/length data is an ideal
    // connectivity relation. Preserve that source semantics explicitly so
    // canonical projection contracts it instead of injecting an arbitrary
    // 1/x numerical coupling into Ybus. Non-ideal values coerced to the DSP
    // 1e-4 pu floor remain physical branches.
    if (ideal_source) {
      br.ideal_connectivity = true;
      br.parameter_source = "bpa_dsp_ideal_connectivity";
    }
    result.system.ac.branches.push_back(std::move(br));
  }

  // AC line continuation card: L+ (terminal shunt reactors).
  void parse_line_plus_card(const std::string& line,
                            const std::string& locator) {
    bool present = false;
    PendingLineShunt extension;
    extension.locator = locator;
    extension.name1 = name_of(raw_field(line, 7, 14));
    extension.kv1 = num(field(line, 15, 18), present);
    extension.name2 = name_of(raw_field(line, 20, 27));
    extension.kv2 = num(field(line, 28, 31), present);
    extension.circuit = field(line, 32, 32);
    extension.q1_mvar = num(field(line, 34, 38), present);
    extension.q2_mvar = num(field(line, 44, 48), present);
    if (extension.name1.empty() || extension.name2.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "L+ card with a missing terminal name; skipped.");
      return;
    }
    pending_line_shunts.push_back(std::move(extension));
  }

  void make_line_shunts(const PendingLineShunt& extension) {
    const int bus1 = find_ac_bus(extension.name1, extension.kv1);
    const int bus2 = find_ac_bus(extension.name2, extension.kv2);
    if (bus1 == 0 || bus2 == 0) {
      warn(ImportDisposition::Rejected, ImportReasonCode::UnresolvedBusRef,
           extension.locator,
           "L+ card references an undeclared terminal bus; skipped.");
      return;
    }

    const std::string line_name =
        "L_" + extension.name1 + "_" + extension.name2 +
        (extension.circuit.empty() ? "" : "_" + extension.circuit);
    const auto line_it = std::find_if(
        result.system.ac.branches.begin(), result.system.ac.branches.end(),
        [&](const ACBranch& branch) {
          return branch.name == line_name && branch.from_bus == bus1 &&
                 branch.to_bus == bus2;
        });
    if (line_it == result.system.ac.branches.end()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::UnresolvedBusRef,
           extension.locator,
           "L+ card has no corresponding L branch '" + line_name +
               "'; skipped.");
      return;
    }

    const auto add_reactor = [&](int bus, double q_mvar,
                                 const std::string& terminal) {
      if (std::abs(q_mvar) <= 0.0) return;
      Shunt shunt;
      shunt.index = static_cast<int>(result.system.ac.shunts.size());
      shunt.bus = bus;
      // Positive L+ Mvar absorbs Q; positive model susceptance injects Q.
      shunt.bs_mvar = -q_mvar;
      shunt.name = "L+_" + extension.name1 + "_" + extension.name2 +
                   (extension.circuit.empty()
                        ? ""
                        : "_" + extension.circuit) +
                   "_" + terminal;
      result.system.ac.shunts.push_back(std::move(shunt));
    };
    add_reactor(bus1, extension.q1_mvar, "from");
    add_reactor(bus2, extension.q2_mvar, "to");
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, extension.locator,
                      "L+ line-reactor continuation bound to '" + line_name +
                          "' as terminal shunt element(s).");
  }

  // ── AC transformer card: T ─────────────────────────────────────────────
  void parse_transformer_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string n1 = name_of(raw_field(line, 7, 14));
    const std::string n2 = name_of(raw_field(line, 20, 27));
    const double kv1 = num(field(line, 15, 18), present);
    const double kv2 = num(field(line, 28, 31), present);
    if (n1.empty() || n2.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "T card with missing terminal name; skipped.");
      return;
    }
    ACBranch br;
    br.index = static_cast<int>(result.system.ac.branches.size());
    br.from_bus = ensure_ac_bus(n1, kv1, locator);
    br.to_bus = ensure_ac_bus(n2, kv2, locator);
    br.r_pu = num_rx(field(line, 39, 44), present);
    br.x_pu = num_rx(field(line, 45, 50), present);
    if (small_reactance_mode == BpaSmallReactanceMode::DspCompatible &&
        std::abs(br.x_pu) < kBpaMinLineReactancePu) {
      const double source_x = br.x_pu;
      br.x_pu = source_x == 0.0
                    ? kBpaMinLineReactancePu
                    : std::copysign(kBpaMinLineReactancePu, source_x);
      std::ostringstream msg;
      msg << "T-card reactance " << source_x
          << " pu is below the DSP numerical floor; coerced to "
          << br.x_pu << " pu.";
      warn(ImportDisposition::Coerced, ImportReasonCode::RangeCoerced,
           locator, msg.str());
    }
    const double g_exc = num_gb(field(line, 51, 56), present);
    const double b_exc = num_gb(field(line, 57, 62), present);
    if ((std::abs(g_exc) > 0.0 || std::abs(b_exc) > 0.0) &&
        !magnetizing_note_emitted) {
      magnetizing_note_emitted = true;
      warn(ImportDisposition::Coerced, ImportReasonCode::StructuralLoss, locator,
           "Transformer magnetizing G/B is not modeled on AC branches; "
           "ignored. Further occurrences are not reported individually.");
    }
    const double tap1 = num_scaled(field(line, 62, 67), 100.0, present);
    const double tap2 = num_scaled(field(line, 68, 73), 100.0, present);
    if (tap1 > 0.0 && tap2 > 0.0 && kv1 > 0.0 && kv2 > 0.0) {
      // BPA T-card R/X is referred to the second winding's stated voltage.
      // DSP converts it to the terminal-2 bus base before building Ybus.
      const double ratio2 = tap2 / kv2;
      const double impedance_scale = ratio2 * ratio2;
      br.r_pu *= impedance_scale;
      br.x_pu *= impedance_scale;
      // Per-unit turns ratio referred to the from-bus (side 1).
      br.tap = (tap1 / kv1) / (tap2 / kv2);
    }
    br.sn_mva = num(field(line, 34, 37), present);
    br.vn_hv_kv = std::max(kv1, kv2);
    br.vn_lv_kv = std::min(kv1, kv2);
    const double n_par = num(field(line, 38, 38), present);
    if (n_par > 0.0) br.n_parallel = static_cast<int>(std::lround(n_par));
    const std::string ckt = field(line, 32, 32);
    br.name = "T_" + n1 + "_" + n2 + (ckt.empty() ? "" : "_" + ckt);
    TransformerCardData card;
    card.branch_index = br.index;
    card.name1 = n1;
    card.name2 = n2;
    card.kv1 = kv1;
    card.kv2 = kv2;
    card.tap1_kv = tap1;
    card.tap2_kv = tap2;
    transformer_card_data.emplace(br.index, std::move(card));
    result.system.ac.branches.push_back(std::move(br));
  }

  // ── Two-terminal HVDC node card: BD ────────────────────────────────────
  // R is a control record bound to an existing T card; it never creates a
  // second electrical branch. For an LCC transformer it supplies the tap
  // range used by the unified power-flow alpha/gamma control outer loop.
  void parse_r_card(const std::string& line, const std::string& locator) {
    bool present = false;
    RControl control;
    control.locator = locator;
    control.name1 = name_of(raw_field(line, 7, 14));
    control.kv1 = num(field(line, 15, 18), present);
    control.adjustable_terminal =
        static_cast<int>(std::lround(num(field(line, 19, 19), present)));
    if (control.adjustable_terminal != 2) {
      // DSP treats a blank winding flag as side 1 (e.g. Samples/2DC).
      control.adjustable_terminal = 1;
    }
    control.name2 = name_of(raw_field(line, 20, 27));
    control.kv2 = num(field(line, 28, 31), present);
    control.controlled_name = name_of(raw_field(line, 34, 41));
    control.controlled_kv = num(field(line, 42, 45), present);
    control.tap_max_kv = num_scaled(field(line, 46, 50), 100.0, present);
    control.tap_min_kv = num_scaled(field(line, 51, 55), 100.0, present);
    control.tap_count =
        static_cast<int>(std::lround(num(field(line, 56, 57), present)));
    if (control.name1.empty() || control.name2.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "R card with a missing transformer terminal; skipped.");
      return;
    }
    r_controls.push_back(std::move(control));
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, locator,
                      "R tap-control card retained and bound to its T-card "
                      "transformer; no duplicate electrical branch created.");
  }

  void parse_bd_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "BD card without a node name; skipped.");
      return;
    }
    const double kv = num(field(line, 15, 18), present);
    BdStation st;
    st.name = name;
    st.ac_bus = ensure_ac_bus(name, kv, locator, /*quiet=*/true);
    st.ac_kv = kv;
    st.bridges = num(field(line, 21, 25), present);
    st.sr_mh = num(field(line, 26, 30), present);
    st.alpha_min_deg = num(field(line, 31, 35), present);
    st.alpha_stop_deg = num(field(line, 36, 40), present);
    st.vdrop_v = num(field(line, 41, 45), present);
    st.bridge_in_a = num(field(line, 46, 50), present);
    st.primary_name = name_of(raw_field(line, 51, 58));
    st.primary_kv = num(field(line, 59, 62), present);
    st.dc_kv = num(field(line, 63, 66), present);
    st.dc_bus = ensure_dc_bus(name, st.dc_kv);
    result.system.dc.buses[static_cast<size_t>(st.dc_bus) - 1].source_card =
        "BD";
    bd_by_name.emplace(name, st);
  }

  // ── LCC quasi-steady station construction (LccQuasiSteady path) ─────────
  // Builds one LCCConverter from the pending BD data of one LD terminal.
  // The commutation reactance is taken from the converter-transformer T card
  // connecting the BD primary-side bus (cols 51-58) to the valve-side bus.
  // DSP consumes T-card R/X directly on the case MVA base. The explicit
  // transformer branch keeps that AC-parallel equivalent leakage. The LCC
  // characteristic needs the leakage of one bridge transformer; n_bridges
  // bridge transformers are parallel on the AC side and series on the DC side,
  // so one bridge's X_c is n_bridges times the branch-equivalent value.
  const RControl* find_lcc_r_control(const BdStation& st) const {
    for (const auto& control : r_controls) {
      const bool direct = control.name1 == st.primary_name &&
                          control.name2 == st.name;
      const bool reverse = control.name2 == st.primary_name &&
                           control.name1 == st.name;
      if ((direct || reverse) &&
          (control.controlled_name.empty() ||
           control.controlled_name == st.name)) {
        return &control;
      }
    }
    return nullptr;
  }

  std::pair<double, double> r_card_tap_bounds(const RControl& control,
                                               int branch_index) const {
    if (!(control.tap_min_kv > 0.0) || !(control.tap_max_kv > 0.0)) {
      return {0.0, 0.0};
    }
    const auto card_it = transformer_card_data.find(branch_index);
    if (card_it == transformer_card_data.end()) return {0.0, 0.0};
    const auto& card = card_it->second;
    const double ratio1 = card.tap1_kv > 0.0 && card.kv1 > 0.0
                              ? card.tap1_kv / card.kv1
                              : 1.0;
    const double ratio2 = card.tap2_kv > 0.0 && card.kv2 > 0.0
                              ? card.tap2_kv / card.kv2
                              : 1.0;
    const std::string adjustable = control.adjustable_terminal == 2
                                       ? control.name2
                                       : control.name1;
    auto branch_tap = [&](double winding_kv) {
      if (adjustable == card.name1 && card.kv1 > 0.0) {
        return (winding_kv / card.kv1) / ratio2;
      }
      if (adjustable == card.name2 && card.kv2 > 0.0) {
        return ratio1 / (winding_kv / card.kv2);
      }
      return 0.0;
    };
    const double a = branch_tap(control.tap_min_kv);
    const double b = branch_tap(control.tap_max_kv);
    return {std::min(a, b), std::max(a, b)};
  }

  void make_lcc_station(const BdStation& st, LCCStationRole role,
                        const std::string& locator, double p_sch,
                        double vdc_kv, double local_vdc_set_kv,
                        double alpha_n_deg,
                        double gamma_n_deg) {
    LCCConverter c;
    c.index = static_cast<int>(result.system.lcc_converters.size());
    c.name = "LCC_" + st.name;
    c.ac_bus = st.ac_bus;
    c.dc_bus = st.dc_bus;
    c.station_role = role;
    c.n_bridges = std::max(1, static_cast<int>(std::lround(st.bridges)));
    if (st.alpha_min_deg > 0.0) c.alpha_min_deg = st.alpha_min_deg;
    if (st.alpha_stop_deg > 0.0) c.alpha_stop_deg = st.alpha_stop_deg;
    c.v_drop_v = st.vdrop_v;
    c.rated_current_a = st.bridge_in_a;
    c.rated_dc_kv = vdc_kv;
    c.vn_ac_kv = st.ac_kv;
    c.smoothing_reactor_mh = st.sr_mh;

    // Converter transformer: the AC branch between the primary-side bus and
    // the valve-side bus named on the BD card.
    const ACBranch* xfmr = nullptr;
    int primary_bus = 0;
    if (!st.primary_name.empty()) {
      primary_bus = find_ac_bus(st.primary_name, st.primary_kv);
    }
    for (const auto& br : result.system.ac.branches) {
      const bool touches_valve =
          br.from_bus == st.ac_bus || br.to_bus == st.ac_bus;
      if (!touches_valve) continue;
      if (primary_bus != 0 && br.from_bus != primary_bus &&
          br.to_bus != primary_bus) {
        continue;
      }
      xfmr = &br;
      break;
    }
    if (xfmr != nullptr) {
      c.converter_transformer_branch = xfmr->index;
      const double n_bridges = static_cast<double>(c.n_bridges);
      c.x_comm_pu = xfmr->x_pu * n_bridges;
      c.x_comm_base_mva = mva_base;
      if (st.ac_kv > 0.0 && c.x_comm_base_mva > 0.0) {
        c.x_comm_ohm =
            c.x_comm_pu * st.ac_kv * st.ac_kv / c.x_comm_base_mva;
      }
    } else {
      warn(ImportDisposition::Coerced, ImportReasonCode::StructuralLoss,
           locator, "LCC station '" + st.name +
                    "': converter transformer T card not found; commutation "
                    "reactance left 0.");
    }

    // Control mode by station role (BPA two-terminal convention): the
    // rectifier regulates the scheduled DC power at the rectifier-side
    // voltage setpoint (alpha free, AlphaN is the expected operating point);
    // the inverter runs constant extinction angle (CEA) at GamaN.
    c.p_set_mw = p_sch;
    if (role == LCCStationRole::Rectifier) {
      c.control_mode = LCCControlMode::ConstantPower;
      c.external_control_code = "PAAL";
      c.v_dc_set_kv = local_vdc_set_kv;
      c.alpha_set_deg = alpha_n_deg;
    } else {
      c.control_mode = LCCControlMode::ConstantGamma;
      c.external_control_code = "VDGA";
      c.v_dc_set_kv = local_vdc_set_kv;
      c.gamma_set_deg = gamma_n_deg;
    }

    if (xfmr != nullptr) {
      if (const RControl* control = find_lcc_r_control(st);
          control != nullptr) {
        const auto [tap_min, tap_max] =
            r_card_tap_bounds(*control, xfmr->index);
        const double target_angle =
            role == LCCStationRole::Rectifier ? alpha_n_deg : gamma_n_deg;
        if (tap_min > 0.0 && tap_max >= tap_min && target_angle > 0.0) {
          c.tap_control_modelled = true;
          c.transformer_tap_min_pu = tap_min;
          c.transformer_tap_max_pu = tap_max;
          // DSP solves the LCC converter-transformer tap continuously inside
          // the R-card range. Columns 56-57 do not quantize this control (the
          // CIGRE black-box matrix blank/02/25/50 is invariant), so importing
          // them as discrete positions creates artificial angle and Udc
          // residuals on production cases.
          c.transformer_tap_steps = 0;
          c.transformer_tap_winding =
              control->adjustable_terminal == 2 ? 2 : 1;
        } else {
          warn(ImportDisposition::Coerced,
               ImportReasonCode::UnsupportedControl, control->locator,
               "R card could not be activated for LCC tap control because "
               "its tap range or LD angle target is missing.");
        }
      }
    }

    c.model_scope = c.tap_control_modelled
                        ? "lcc-quasi-steady+transformer-tap-control"
                        : "lcc-quasi-steady";
    c.model_limitations =
        "Quasi-steady LCC station model (BD/LD card import): no commutation "
        "overlap-angle iteration; smoothing reactor ignored (dynamic-only). "
        "Consumed by the unified Newton power flow (Q = P*tan(phi), "
        "cos(phi) ~= U_d/U_d0).";
    if (!c.tap_control_modelled) {
      c.model_limitations +=
          " No applicable R card was found, so the converter-transformer tap "
          "is fixed and the reported alpha/gamma may depart from its target.";
    }
    result.system.lcc_converters.push_back(std::move(c));
  }

  // ── Two-terminal HVDC line card: LD ────────────────────────────────────
  void parse_ld_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string n_rect = name_of(raw_field(line, 7, 14));
    const std::string n_inv = name_of(raw_field(line, 20, 27));
    if (n_rect.empty() || n_inv.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "LD card with missing terminal name; skipped.");
      return;
    }
    auto it_r = bd_by_name.find(n_rect);
    auto it_i = bd_by_name.find(n_inv);
    if (it_r == bd_by_name.end() || it_i == bd_by_name.end()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::UnresolvedBusRef,
           locator, "LD card references a DC node without a preceding BD card "
                    "('" + n_rect + "' / '" + n_inv + "'); skipped.");
      return;
    }
    BdStation& rect = it_r->second;
    BdStation& inv = it_i->second;

    const double i_rated = num(field(line, 34, 37), present);
    const double r_ohm = num(field(line, 38, 41), present);
    const double inductance_mh = num(field(line, 42, 46), present);
    const double capacitance_uf = num(field(line, 47, 51), present);
    const std::string ctrl_point = field(line, 56, 56);
    const double p_sch = num(field(line, 57, 61), present);
    const double vdc_rect = num(field(line, 62, 66), present);
    const double alpha_n_deg = num(field(line, 67, 70), present);
    const double gamma_n_deg = num(field(line, 71, 74), present);
    const double length = num(field(line, 75, 78), present);
    const double vdc_kv = vdc_rect > 0.0 ? vdc_rect
                          : (rect.dc_kv > 0.0 ? rect.dc_kv : inv.dc_kv);

    // Back-fill DC bus base voltages when the BD cards left them blank.
    for (BdStation* st : {&rect, &inv}) {
      DCBus& b = result.system.dc.buses[static_cast<size_t>(st->dc_bus) - 1];
      if (b.base_kv <= 0.0) b.base_kv = vdc_kv > 0.0 ? vdc_kv : 1.0;
    }

    DCBranch br;
    br.index = static_cast<int>(result.system.dc.branches.size());
    br.from_bus = rect.dc_bus;
    br.to_bus = inv.dc_bus;
    if (vdc_kv > 0.0) {
      br.r_pu = r_ohm * mva_base / (vdc_kv * vdc_kv);
      br.base_kv = vdc_kv;
    } else {
      warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
           locator, "LD card: no DC voltage known; DC line resistance left 0.");
    }
    if (i_rated > 0.0 && vdc_kv > 0.0) {
      br.rate_a_mva = vdc_kv * i_rated / 1000.0;
    }
    br.length_km = length;
    br.inductance_mh = inductance_mh;
    br.capacitance_uf = capacitance_uf;
    br.source_card = "LD";
    if (length > 0.0) {
      // LD columns 38-41 contain the resistance of the complete DC line.
      // Preserve the equivalent engineering value used by the GUI instead of
      // leaving the per-kilometre field at its default zero.
      br.r_ohm_per_km = r_ohm / length;
    }
    br.name = "LD_" + n_rect + "_" + n_inv;
    result.system.dc.branches.push_back(std::move(br));

    // Native quasi-steady LCC import (opt-in via BpaImportOptions::lcc_model):
    // the LD pairing produces the DCBranch above plus two LCCConverter
    // elements — no VSC stand-ins.  Station roles follow the LD card terminal
    // order (first terminal = rectifier), same convention as the VSC path.
    // The station elements are materialized after the whole file is parsed
    // (see run()) because the converter-transformer T card may follow the LD
    // card (e.g. 2DC.dat).
    if (options.lcc_model == BpaLccModel::LccQuasiSteady) {
      if (!ctrl_point.empty() && ctrl_point != "R") {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnsupportedControl,
             locator, "LD card: power control point '" + ctrl_point +
                      "' is not rectifier-side ('R'); only rectifier-side "
                      "constant-power control is supported — control modes "
                      "assigned as if the flag were 'R'.");
      }
      PendingLccLink link;
      link.locator = locator;
      link.rect_name = n_rect;
      link.inv_name = n_inv;
      link.p_sch = p_sch;
      link.vdc_kv = vdc_kv;
      const double scheduled_id_ka =
          p_sch > 0.0 && vdc_kv > 0.0 ? p_sch / vdc_kv : 0.0;
      link.inv_vdc_set_kv =
          vdc_kv > 0.0
              ? std::max(0.0, vdc_kv - scheduled_id_ka * r_ohm)
              : 0.0;
      link.alpha_n_deg = alpha_n_deg;
      link.gamma_n_deg = gamma_n_deg;
      pending_lcc_links.push_back(std::move(link));
      return;
    }

    // Legacy opt-in path: approximate the LCC link with VSC converters when
    // BpaImportOptions::lcc_model is explicitly VscApprox.  Native
    // LCCConverter import is the default path above.  Station roles mirror
    // the physical LCC link:
    //   * Rectifier (sending end): PQ_MODE, draws the scheduled power from
    //     its AC system (p_set = -Psch) — matches the DSP solution exactly.
    //   * Inverter (receiving end): VDC_Q, forms the inverter-side DC voltage
    //     (constant-gamma role).  Its delivered AC power emerges from the DC
    //     island balance (= Psch - line losses), as in DSP.  A hard VDC
    //     converter is also what the engine requires: a bare DC_V bus is only
    //     a declaration and fixed-P converters get auto-promoted otherwise.
    // Converter efficiency is 1.0 unless the BD card specifies a valve drop
    // (mapped onto eta / loss_mw below), so the DC line R carries essentially
    // all of the link loss — matching the DSP solution (1410 = 1500 - I^2R).
    // Both stations absorb Q estimated as lcc_q_ratio * P (typical LCC
    // reactive consumption).
    const double i_est = (p_sch > 0.0 && vdc_kv > 0.0) ? p_sch / vdc_kv : 0.0;
    const double line_loss = i_est * i_est * r_ohm;  // MW (kA^2 * ohm)
    const double p_recv = std::max(0.0, p_sch - line_loss);
    // Station losses from the BD card valve drop: P_loss = Vdrop * Id * bridges
    // (0.6 MW per station for the CIGRE card's 100 V / 3 kA / 2 bridges; the
    // 2DC card leaves Vdrop blank -> 0).  The loss is mapped onto eta because
    // the engine's default Linear loss model reads eta only (loss = (1-eta)*P,
    // which for constant-voltage valve drop has exactly the right ∝I shape);
    // loss_mw is also set for users of the CurrentBased loss model.
    const double id_a = i_est * 1000.0;
    const double rect_loss_mw = rect.vdrop_v * id_a * std::max(1.0, rect.bridges) / 1e6;
    const double inv_loss_mw = inv.vdrop_v * id_a * std::max(1.0, inv.bridges) / 1e6;
    const double rect_eta = p_sch > 0.0 ? std::max(0.9, 1.0 - rect_loss_mw / p_sch) : 1.0;
    const double inv_eta = p_recv > 0.0 ? std::max(0.9, 1.0 - inv_loss_mw / p_recv) : 1.0;

    VSCConverter crect;
    crect.index = static_cast<int>(result.system.vsc_converters.size());
    crect.bus_ac = rect.ac_bus;
    crect.bus_dc = rect.dc_bus;
    crect.control_mode = ConverterMode::PQ_MODE;
    crect.p_set_mw = -p_sch;
    crect.q_set_mvar = -options.lcc_q_ratio * p_sch;
    crect.eta = rect_eta;
    crect.loss_mw = rect_loss_mw;
    crect.vn_ac_kv = result.system.ac.buses[static_cast<size_t>(rect.ac_bus) - 1].base_kv;
    crect.vn_dc_kv = vdc_kv;
    crect.name = "LCC_" + n_rect;
    crect.type = "LCC(approx)";
    result.system.vsc_converters.push_back(std::move(crect));

    VSCConverter cinv;
    cinv.index = static_cast<int>(result.system.vsc_converters.size());
    cinv.bus_ac = inv.ac_bus;
    cinv.bus_dc = inv.dc_bus;
    cinv.control_mode = ConverterMode::VDC_Q;
    // Inverter-side DC voltage setpoint: rectifier rated voltage minus the
    // estimated line drop (V_inv = V_rect - I*R), in pu of the DC base.
    cinv.v_dc_set_pu = (vdc_kv > 0.0)
                           ? std::max(0.5, (vdc_kv - i_est * r_ohm) / vdc_kv)
                           : 1.0;
    cinv.k_vdc = 1000.0;  // stiff Vdc hold (hard constant-voltage role)
    cinv.q_set_mvar = -options.lcc_q_ratio * p_recv;
    // Scheduled / warm-start values only (P is free in VDC_Q).
    cinv.p_set_mw = p_recv;
    cinv.p_schedule_mw = p_recv;
    cinv.p_initial_mw = p_recv;
    cinv.eta = inv_eta;
    cinv.loss_mw = inv_loss_mw;
    cinv.vn_ac_kv = result.system.ac.buses[static_cast<size_t>(inv.ac_bus) - 1].base_kv;
    cinv.vn_dc_kv = vdc_kv;
    cinv.name = "LCC_" + n_inv;
    cinv.type = "LCC(approx)";
    result.system.vsc_converters.push_back(std::move(cinv));

    std::ostringstream msg;
    msg << "LCC HVDC link '" << n_rect << "' -> '" << n_inv << "' (P=" << p_sch
        << " MW, V=" << vdc_kv << " kV) approximated by VSC converters: "
           "rectifier PQ (draws scheduled P), inverter VDC_Q (forms Vdc; "
           "delivers ~" << p_recv << " MW = Psch - I^2R). Station Q estimated "
           "as " << options.lcc_q_ratio << " * P; valve-drop station loss "
           << rect_loss_mw << " / " << inv_loss_mw << " MW (rect/inv).";
    warn(ImportDisposition::Coerced, ImportReasonCode::StructuralLoss, locator,
         msg.str());
  }

  // ── VSC-HVDC station node card: BZ ───────────────────────────────────
  // Layered/hybrid LCC cards: BA, BA1, BA2, DC, BB, LY.
  void parse_ba_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "BA/BA1/BA2 card without a station name; skipped.");
      return;
    }

    const char continuation = line.size() > 2 ? line[2] : ' ';
    auto it = ba_by_name.find(name);
    if (it == ba_by_name.end()) {
      BaStation seed;
      seed.name = name;
      seed.locator = locator;
      it = ba_by_name.emplace(name, std::move(seed)).first;
      ba_order.push_back(name);
    }
    BaStation& st = it->second;

    if (continuation == '1') {
      st.has_ba1 = true;
      st.valve_kv = num(field(line, 20, 23), present);
      st.transformer_sn_mva = num(field(line, 25, 28), present);
      st.transformer_x_pu = num_rx(field(line, 37, 40), present);
      st.tap_max_kv = num(field(line, 44, 49), present);
      st.tap_min_kv = num(field(line, 51, 56), present);
      return;
    }
    if (continuation == '2') {
      st.has_ba2 = true;
      st.v_drop_v = num(field(line, 20, 23), present);
      st.rated_current_a = num(field(line, 26, 30), present);
      st.alpha_min_deg = num(field(line, 31, 35), present);
      st.alpha_stop_deg = num(field(line, 36, 41), present);
      st.normal_angle_deg = num(field(line, 43, 47), present);
      st.rated_dc_kv = num(field(line, 68, 71), present);
      st.dc_bus = ensure_dc_bus(name, st.rated_dc_kv);
      return;
    }

    st.locator = locator;
    st.primary_kv = num(field(line, 15, 18), present);
    st.role = field(line, 22, 22);
    st.layer = field(line, 24, 24);
    const double bridges = num(field(line, 26, 26), present);
    st.bridges = bridges > 0.0 ? static_cast<int>(std::lround(bridges)) : 1;
    st.smoothing_reactor_mh = num(field(line, 41, 45), present);
    const double percent = num(field(line, 46, 50), present);
    if (percent > 0.0) st.power_percent = percent;
    st.q_compensation_mvar = num(field(line, 51, 55), present);
    st.dc_bus = ensure_dc_bus(name, st.rated_dc_kv);
  }

  void parse_layered_dc_control(const std::string& line,
                                const std::string& locator) {
    bool present = false;
    LayeredDcControl control;
    control.locator = locator;
    control.rectifier_high = name_of(raw_field(line, 4, 11));
    control.rectifier_high_kv = num(field(line, 12, 15), present);
    control.rectifier_low = name_of(raw_field(line, 16, 23));
    control.rectifier_low_kv = num(field(line, 24, 27), present);
    control.inverter_high = name_of(raw_field(line, 28, 35));
    control.inverter_high_kv = num(field(line, 36, 39), present);
    control.inverter_low = name_of(raw_field(line, 40, 47));
    control.inverter_low_kv = num(field(line, 48, 51), present);
    control.system_type =
        static_cast<int>(std::lround(num(field(line, 53, 53), present)));
    control.control_mode =
        static_cast<int>(std::lround(num(field(line, 55, 55), present)));
    control.inverter_layer_mode =
        static_cast<int>(std::lround(num(field(line, 57, 57), present)));
    control.p_sch_mw = num(field(line, 61, 65), present);
    control.rectifier_v_sch_kv = num(field(line, 67, 71), present);
    control.inverter_v_sch_kv = num(field(line, 73, 77), present);
    const double calculation_mode = num(field(line, 87, 87), present);
    if (present) {
      control.converter_calculation_mode =
          static_cast<int>(std::lround(calculation_mode));
    }
    layered_dc_controls.push_back(std::move(control));
  }

  void parse_bb_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    const double kv = num(field(line, 15, 18), present);
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "BB card without a DC bus name; skipped.");
      return;
    }
    const int index = ensure_dc_bus(name, kv);
    DCBus& bus = result.system.dc.buses[static_cast<size_t>(index) - 1];
    bus.source_card = "BB";
    bus.converter_role = field(line, 22, 22);
    bus.converter_layer = field(line, 24, 24);
    bus.bus_type = DCBusType::DC_P;
    bus.pd_mw = 0.0;
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, locator,
                      "BB passive junction imported as a zero-injection "
                      "DC_P bus whose voltage is solved from DC KCL.");
  }

  // Multi-terminal LCC cards: BM and LM.
  void parse_bm_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "BM card without a station name; skipped.");
      return;
    }
    if (bm_by_name.count(name) != 0) {
      warn(ImportDisposition::Rejected, ImportReasonCode::DuplicateId, locator,
           "Duplicate BM station '" + name + "'; skipped.");
      return;
    }

    BmStation bm;
    bm.locator = locator;
    BdStation& st = bm.station;
    st.name = name;
    st.ac_kv = num(field(line, 15, 18), present);
    st.ac_bus = ensure_ac_bus(name, st.ac_kv, locator, /*quiet=*/true);
    st.bridges = num(field(line, 21, 25), present);
    st.sr_mh = num(field(line, 26, 30), present);
    st.alpha_min_deg = num(field(line, 31, 35), present);
    st.alpha_stop_deg = num(field(line, 36, 40), present);
    st.vdrop_v = num(field(line, 41, 45), present);
    st.bridge_in_a = num(field(line, 46, 50), present);
    st.primary_name = name_of(raw_field(line, 51, 58));
    st.primary_kv = num(field(line, 59, 62), present);
    bm.role = field(line, 63, 63);
    bm.normal_angle_deg = num_scaled(field(line, 64, 66), 10.0, present);
    bm.gamma_min_deg = num(field(line, 67, 69), present);
    bm.p_sch_mw = num(field(line, 70, 74), present);
    bm.v_sch_kv = num(field(line, 75, 79), present);
    st.dc_kv = num(field(line, 80, 85), present);
    st.dc_bus = ensure_dc_bus(name, st.dc_kv);
    DCBus& dc_bus =
        result.system.dc.buses[static_cast<size_t>(st.dc_bus) - 1];
    dc_bus.source_card = "BM";
    dc_bus.converter_role = bm.role;
    bm_order.push_back(name);
    bm_by_name.emplace(name, std::move(bm));
  }

  void parse_native_dc_line(const std::string& line,
                            const std::string& locator,
                            const std::string& source_card) {
    bool present = false;
    PendingNativeDcLine branch;
    branch.locator = locator;
    branch.source_card = source_card;
    branch.name1 = name_of(raw_field(line, 7, 14));
    branch.kv1 = num(field(line, 15, 18), present);
    branch.name2 = name_of(raw_field(line, 20, 27));
    branch.kv2 = num(field(line, 28, 31), present);
    branch.i_rated_a = num(field(line, 34, 37), present);
    branch.r_ohm = num(field(line, 38, 42), present);
    branch.inductance_mh = num(field(line, 43, 49), present);
    branch.capacitance_uf = num(field(line, 50, 56), present);
    branch.length_km = num(field(line, 77, 81), present);
    if (branch.name1.empty() || branch.name2.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, source_card + " card with a missing terminal; skipped.");
      return;
    }
    pending_native_dc_lines.push_back(std::move(branch));
  }

  const LayeredDcControl* find_layered_control(
      const BaStation& station) const {
    for (const auto& control : layered_dc_controls) {
      if (control.rectifier_high == station.name ||
          control.rectifier_low == station.name ||
          control.inverter_high == station.name ||
          control.inverter_low == station.name) {
        return &control;
      }
    }
    return nullptr;
  }

  void make_ba_station(BaStation& st) {
    if (!st.has_ba1 || !st.has_ba2 || st.primary_kv <= 0.0 ||
        st.valve_kv <= 0.0 || st.rated_dc_kv <= 0.0) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           st.locator, "BA station '" + st.name +
                           "' is missing BA1/BA2 or a required voltage; "
                           "station skipped.");
      return;
    }

    st.dc_bus = ensure_dc_bus(st.name, st.rated_dc_kv);
    DCBus& dc_bus =
        result.system.dc.buses[static_cast<size_t>(st.dc_bus) - 1];
    dc_bus.source_card = "BA";
    dc_bus.converter_role = st.role;
    dc_bus.converter_layer = st.layer;

    const int primary_bus =
        ensure_ac_bus(st.name, st.primary_kv, st.locator, /*quiet=*/true);
    const int valve_bus =
        ensure_ac_bus(st.name, st.valve_kv, st.locator, /*quiet=*/true);

    ACBranch transformer;
    transformer.index = static_cast<int>(result.system.ac.branches.size());
    transformer.from_bus = primary_bus;
    transformer.to_bus = valve_bus;
    transformer.x_pu = st.transformer_x_pu;
    transformer.tap = 1.0;
    transformer.sn_mva = st.transformer_sn_mva;
    transformer.vn_hv_kv = std::max(st.primary_kv, st.valve_kv);
    transformer.vn_lv_kv = std::min(st.primary_kv, st.valve_kv);
    transformer.name = "T_BA_" + st.name + "_" + st.layer;
    const int transformer_index = transformer.index;
    result.system.ac.branches.push_back(std::move(transformer));

    TransformerCardData transformer_data;
    transformer_data.branch_index = transformer_index;
    transformer_data.name1 = st.name;
    transformer_data.name2 = st.name;
    transformer_data.kv1 = st.primary_kv;
    transformer_data.kv2 = st.valve_kv;
    transformer_data.tap1_kv = st.primary_kv;
    transformer_data.tap2_kv = st.valve_kv;
    transformer_card_data.emplace(transformer_index,
                                  std::move(transformer_data));

    const LayeredDcControl* control = find_layered_control(st);
    const bool rectifier = st.role != "I";
    double p_sch = 0.0;
    double local_v_sch = 0.0;
    if (control != nullptr) {
      p_sch = control->p_sch_mw * st.power_percent / 100.0;
      local_v_sch = rectifier ? control->rectifier_v_sch_kv
                              : control->inverter_v_sch_kv;
    } else {
      warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
           st.locator, "BA station '" + st.name +
                           "' has no matching DC control card; zero scheduled "
                           "power is used.");
    }

    BdStation native;
    native.name = st.name;
    native.ac_bus = valve_bus;
    native.dc_bus = st.dc_bus;
    native.ac_kv = st.valve_kv;
    native.dc_kv = st.rated_dc_kv;
    native.bridges = st.bridges;
    native.sr_mh = st.smoothing_reactor_mh;
    native.alpha_min_deg = st.alpha_min_deg;
    native.alpha_stop_deg = st.alpha_stop_deg;
    native.vdrop_v = st.v_drop_v;
    native.bridge_in_a = st.rated_current_a;
    native.primary_name = st.name;
    native.primary_kv = st.primary_kv;

    const size_t before = result.system.lcc_converters.size();
    make_lcc_station(native,
                     rectifier ? LCCStationRole::Rectifier
                               : LCCStationRole::Inverter,
                     st.locator, p_sch, st.rated_dc_kv, local_v_sch,
                     st.normal_angle_deg, st.normal_angle_deg);
    if (result.system.lcc_converters.size() == before) return;
    LCCConverter& converter = result.system.lcc_converters.back();
    converter.source_card = "BA";
    converter.layer_code = st.layer;
    converter.power_percent = st.power_percent;
    converter.q_compensation_mvar = st.q_compensation_mvar;
    converter.model_scope =
        "bpa-layered-lcc-quasi-steady+embedded-transformer-tap-control";
    converter.model_limitations =
        "BA/BA1/BA2 layered-LCC steady-state projection. BA1 creates the "
        "converter transformer; LY and the common DC network provide KCL. "
        "Smoothing reactor and line inductance are dynamic-only. Series "
        "high/low layer sharing remains explicitly uncalibrated until a DSP "
        "component case containing both layers is available.";

    if (st.tap_min_kv > 0.0 && st.tap_max_kv >= st.tap_min_kv &&
        st.primary_kv > 0.0 && st.normal_angle_deg > 0.0) {
      converter.tap_control_modelled = true;
      converter.transformer_tap_min_pu = st.tap_min_kv / st.primary_kv;
      converter.transformer_tap_max_pu = st.tap_max_kv / st.primary_kv;
      converter.transformer_tap_steps = 0;
      converter.transformer_tap_winding = 1;
    }

    if (std::abs(st.q_compensation_mvar) > 0.0) {
      Shunt compensation;
      compensation.index =
          static_cast<int>(result.system.ac.shunts.size());
      compensation.bus = primary_bus;
      // BPA BA Qshunt is positive for capacitive compensation. The rich
      // model uses positive bs_mvar for reactive injection at 1 pu.
      compensation.bs_mvar = st.q_compensation_mvar;
      compensation.name = "BA_QSH_" + st.name + "_" + st.layer;
      result.system.ac.shunts.push_back(std::move(compensation));
    }

    result.report.add(
        ImportDisposition::Accepted, ImportReasonCode::Ok,
        ImportSeverity::Info, st.locator,
        "BA/BA1/BA2 station '" + st.name +
            "' imported as a native LCC converter plus its embedded BA1 "
            "converter transformer.");
  }

  void make_bm_station(BmStation& bm) {
    const bool rectifier = bm.role != "I";
    const size_t before = result.system.lcc_converters.size();
    make_lcc_station(bm.station,
                     rectifier ? LCCStationRole::Rectifier
                               : LCCStationRole::Inverter,
                     bm.locator, bm.p_sch_mw, bm.station.dc_kv,
                     bm.v_sch_kv, bm.normal_angle_deg,
                     bm.normal_angle_deg);
    if (result.system.lcc_converters.size() == before) return;
    LCCConverter& converter = result.system.lcc_converters.back();
    converter.source_card = "BM";
    converter.gamma_min_deg = bm.gamma_min_deg;
    converter.model_scope = converter.tap_control_modelled
                                ? "bpa-mtdc-lcc-quasi-steady+tap-control"
                                : "bpa-mtdc-lcc-quasi-steady";
    converter.model_limitations =
        "BM multi-terminal LCC station on the native DC nodal network. LM "
        "resistance is in the unified Newton equations; smoothing reactors "
        "and line inductance are dynamic-only.";
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, bm.locator,
                      "BM station '" + bm.station.name +
                          "' imported as a native multi-terminal LCC "
                          "converter.");
  }

  void make_native_dc_line(const PendingNativeDcLine& pending) {
    const auto from_it = dc_bus_by_name.find(pending.name1);
    const auto to_it = dc_bus_by_name.find(pending.name2);
    if (from_it == dc_bus_by_name.end() || to_it == dc_bus_by_name.end()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::UnresolvedBusRef,
           pending.locator, pending.source_card + " card references an "
           "undeclared DC node ('" + pending.name1 + "' / '" +
           pending.name2 + "'); skipped.");
      return;
    }
    const DCBus& from_bus =
        result.system.dc.buses[static_cast<size_t>(from_it->second) - 1];
    const DCBus& to_bus =
        result.system.dc.buses[static_cast<size_t>(to_it->second) - 1];
    const double base_kv = from_bus.base_kv > 0.0 ? from_bus.base_kv
                           : to_bus.base_kv > 0.0 ? to_bus.base_kv
                                                  : 0.0;
    if (base_kv <= 0.0) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           pending.locator, pending.source_card +
                                " line has no rated DC voltage; skipped.");
      return;
    }
    if (from_bus.base_kv > 0.0 && to_bus.base_kv > 0.0 &&
        std::abs(from_bus.base_kv - to_bus.base_kv) > 1e-6) {
      warn(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
           pending.locator, pending.source_card +
                                " terminal DC bases differ; the from-terminal "
                                "base is used for per-unit conversion.");
    }

    DCBranch branch;
    branch.index = static_cast<int>(result.system.dc.branches.size());
    branch.from_bus = from_it->second;
    branch.to_bus = to_it->second;
    branch.r_pu = pending.r_ohm * mva_base / (base_kv * base_kv);
    branch.base_kv = base_kv;
    branch.rate_a_mva = pending.i_rated_a > 0.0
                            ? base_kv * pending.i_rated_a / 1000.0
                            : 0.0;
    branch.inductance_mh = pending.inductance_mh;
    branch.capacitance_uf = pending.capacitance_uf;
    branch.length_km = pending.length_km;
    if (pending.length_km > 0.0) {
      branch.r_ohm_per_km = pending.r_ohm / pending.length_km;
    }
    branch.source_card = pending.source_card;
    branch.name = pending.source_card + "_" + pending.name1 + "_" +
                  pending.name2;
    result.system.dc.branches.push_back(std::move(branch));
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, pending.locator,
                      pending.source_card +
                          " line imported into the native resistive DC "
                          "network; L/C retained as dynamic metadata.");
  }

  void parse_bz_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    if (name.empty()) {
      warn(ImportDisposition::Rejected, ImportReasonCode::MissingRequired,
           locator, "BZ card without a station name; skipped.");
      return;
    }
    if (bz_by_name.count(name) != 0) {
      warn(ImportDisposition::Rejected, ImportReasonCode::DuplicateId, locator,
           "Duplicate BZ station '" + name + "'; skipped.");
      return;
    }
    const double kv = num(field(line, 15, 18), present);
    BzStation st;
    st.name = name;
    st.locator = locator;
    st.ac_bus = ensure_ac_bus(name, kv, locator, /*quiet=*/true);
    st.ac_kv = kv;
    st.sn_mva = num(field(line, 21, 25), present);
    st.rc_pu = num(field(line, 51, 55), present);
    const double poles = num(field(line, 64, 64), present);
    if (poles >= 1.0) st.n_poles = static_cast<int>(std::lround(poles));
    st.udcn_bz_kv = num(field(line, 67, 70), present);
    st.dc_bus = ensure_dc_bus(name, st.udcn_bz_kv);
    result.system.dc.buses[static_cast<size_t>(st.dc_bus) - 1].source_card =
        "BZ";
    bz_order.push_back(name);
    bz_by_name.emplace(name, std::move(st));
  }

  // ── VSC-HVDC station continuation card: BZ+ ──────────────────────────
  void parse_bz_plus_card(const std::string& line, const std::string& locator) {
    bool present = false;
    const std::string name = name_of(raw_field(line, 7, 14));
    const auto it = bz_by_name.find(name);
    if (name.empty() || it == bz_by_name.end()) {
      // DSP aborts with "THE FOLLOWING BZ+ CARD DO NOT HAVE CORRESPONDING BZ
      // CARD"; mirror that as an import error.
      result.report.add(ImportDisposition::Rejected,
                        ImportReasonCode::UnresolvedBusRef,
                        ImportSeverity::Error, locator,
                        "BZ+ card for station '" + name +
                            "' has no corresponding BZ card; skipped.");
      return;
    }
    BzStation& st = it->second;
    if (st.has_plus) {
      warn(ImportDisposition::Coerced, ImportReasonCode::DuplicateId, locator,
           "Duplicate BZ+ card for station '" + name + "'; later card wins.");
    }
    st.has_plus = true;
    st.p_set_mw = num(field(line, 20, 24), present);
    st.q_card_mvar = num(field(line, 26, 29), present);
    const std::string flag = field(line, 34, 34);
    st.mode_flag = flag.empty() ? 0 : std::atoi(flag.c_str());
    st.udcn_kv = num(field(line, 36, 39), present);
    st.udc_set_kv = num(field(line, 41, 44), present);
    st.xc_pu = num(field(line, 53, 58), present);
    if (st.udcn_kv <= 0.0) {
      // DSP aborts with "直流额定电压<=0kV" here; fall back to the BZ-card
      // value (which may itself be 0 -> handled at materialization).
      warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
           locator, "BZ+ card for station '" + name +
                        "' has no rated DC voltage (cols 36-39); falling back "
                        "to the BZ-card value.");
    }
  }

  // ── VSC-HVDC line card: LZ ───────────────────────────────────────────
  void parse_lz_card(const std::string& line, const std::string& locator) {
    bool present = false;
    PendingLzLink lz;
    lz.locator = locator;
    lz.name1 = name_of(raw_field(line, 7, 14));
    lz.name2 = name_of(raw_field(line, 20, 27));
    lz.i_rated_a = num(field(line, 34, 37), present);
    lz.r_ohm = num(field(line, 38, 43), present);
    lz.inductance_mh = num(field(line, 44, 50), present);
    pending_lz_links.push_back(std::move(lz));
  }

  /// Rated DC voltage of a BZ station: the BZ+ value governs; the BZ-card
  /// value (cols 67-70) is the fallback.
  static double bz_udcn_kv(const BzStation& st) {
    return st.udcn_kv > 0.0 ? st.udcn_kv : st.udcn_bz_kv;
  }

  // ── VSC station construction (materialized after the whole file) ──────
  void make_vsc_station(BzStation& st) {
    const double udcn = bz_udcn_kv(st);
    if (udcn > 0.0) {
      DCBus& b = result.system.dc.buses[static_cast<size_t>(st.dc_bus) - 1];
      if (b.base_kv <= 0.0) b.base_kv = udcn;
    }
    if (!st.has_plus) {
      warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
           st.locator, "BZ station '" + st.name +
                           "' has no BZ+ continuation card; imported as a "
                           "constant-P/Q station with zero setpoints.");
    }

    VSCConverter c;
    c.index = static_cast<int>(result.system.vsc_converters.size());
    c.bus_ac = st.ac_bus;
    c.bus_dc = st.dc_bus;
    c.vn_ac_kv = st.ac_kv;
    c.vn_dc_kv = udcn;
    // Station active loss: DSP books |P|*Rc (Rc in pu on the converter
    // base), i.e. an efficiency eta = 1 - Rc under the engine's Linear loss
    // model.  (DSP applies the factor in the power-flow direction on both
    // stations, so its reported DC-side power exceeds the physical
    // conservation value by ~Rc*|P| at each end; see the compare test.)
    c.eta = st.rc_pu > 0.0 ? std::max(0.9, 1.0 - st.rc_pu) : 1.0;
    if (st.mode_flag == 1) {
      // Constant DC voltage station (定直流电压站): forms the DC voltage,
      // AC active power is free; P from the card is schedule/warm-start only.
      c.control_mode = ConverterMode::VDC_Q;
      if (st.udc_set_kv > 0.0 && udcn > 0.0) {
        c.v_dc_set_pu = st.udc_set_kv / udcn;
      } else {
        c.v_dc_set_pu = 1.0;
        warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
             st.locator, "Constant-DC-voltage station '" + st.name +
                             "' without a valid DC voltage setpoint "
                             "(BZ+ cols 41-44); 1.0 pu used.");
      }
      // Very stiff Vdc hold: DSP mode-1 stations hold Udc exactly, and the
      // sole-former lock (plan_dc_island_references) keeps the adaptive
      // mode switch from demoting the station, so a much stiffer droop than
      // the LCC-approx precedent (1e3) is safe and lands Udc within ~0.01%
      // of the setpoint for typical transfers.
      c.k_vdc = 1e5;
      c.p_set_mw = -st.p_set_mw;  // card: >0 absorbs; model: >0 injects
      c.p_schedule_mw = -st.p_set_mw;
      c.p_initial_mw = -st.p_set_mw;
    } else {
      // Constant power station (定功率站): P and Q held at the AC terminal.
      if (st.mode_flag != 0 && st.mode_flag != 2) {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnsupportedControl,
             st.locator, "BZ+ control-mode flag " +
                             std::to_string(st.mode_flag) + " on station '" +
                             st.name +
                             "' is not modeled; constant-P/Q control assumed.");
      }
      c.control_mode = ConverterMode::PQ_MODE;
      c.p_set_mw = -st.p_set_mw;  // card: >0 absorbs; model: >0 injects
    }
    // BPA card P/Q signs follow the load convention (positive absorbs —
    // verified against DSP branch-flow signs and the bus-voltage response);
    // the model uses the injection convention.
    c.q_set_mvar = -st.q_card_mvar;
    c.name = "VSC_" + st.name;
    c.type = "VSC(BZ)";
    result.system.vsc_converters.push_back(std::move(c));

    std::ostringstream msg;
    msg << "VSC station '" << st.name << "' (BZ/BZ+): "
        << (st.mode_flag == 1 ? "constant DC voltage, Udc=" : "constant P/Q, P=")
        << (st.mode_flag == 1 ? st.udc_set_kv : -st.p_set_mw)
        << (st.mode_flag == 1 ? " kV" : " MW") << ", Q=" << -st.q_card_mvar
        << " Mvar (injection), UdcN=" << udcn << " kV, eta=" << 1.0 - st.rc_pu
        << " (from Rc=" << st.rc_pu
        << " pu). The Xc internal reactive loss (BZ+ cols 53-58) is inside "
           "DSP's converter model and does not reach the AC terminal; it is "
           "not part of the imported power-flow model.";
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, st.locator, msg.str());
  }

  // ── VSC-HVDC line construction (materialized after the whole file) ────
  void make_lz_branch(const PendingLzLink& lz) {
    const auto dc1 = dc_bus_by_name.find(lz.name1);
    const auto dc2 = dc_bus_by_name.find(lz.name2);
    if (lz.name1.empty() || lz.name2.empty() || dc1 == dc_bus_by_name.end() ||
        dc2 == dc_bus_by_name.end()) {
      // DSP aborts with "LZ卡没有对应BZ卡"; here the line is dropped.
      warn(ImportDisposition::Rejected, ImportReasonCode::UnresolvedBusRef,
           lz.locator, "LZ card references an undeclared DC node "
                        "('" + lz.name1 + "' / '" + lz.name2 + "'); skipped.");
      return;
    }
    const DCBus& bus1 =
        result.system.dc.buses[static_cast<size_t>(dc1->second) - 1];
    const DCBus& bus2 =
        result.system.dc.buses[static_cast<size_t>(dc2->second) - 1];
    const double udcn1 = bus1.base_kv;
    const double udcn2 = bus2.base_kv;
    const double udcn = udcn1 > 0.0 ? udcn1 : udcn2;
    if (udcn1 > 0.0 && udcn2 > 0.0 && std::abs(udcn1 - udcn2) > 1e-6) {
      // The card manual requires equal rated DC voltages at both ends.
      warn(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
           lz.locator, "LZ card '" + lz.name1 + "'-'" + lz.name2 +
                           "': terminal rated DC voltages differ (" +
                           std::to_string(udcn1) + " / " +
                           std::to_string(udcn2) + " kV); the first is used.");
    }
    int n_poles = 1;
    const auto bz1 = bz_by_name.find(lz.name1);
    const auto bz2 = bz_by_name.find(lz.name2);
    if (bz1 != bz_by_name.end()) n_poles = std::max(1, bz1->second.n_poles);
    if (bz2 != bz_by_name.end()) {
      if (bz1 != bz_by_name.end() &&
          bz1->second.n_poles != bz2->second.n_poles) {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
             lz.locator, "LZ card '" + lz.name1 + "'-'" + lz.name2 +
                             "': terminal pole counts differ; the first "
                             "VSC station's value is used.");
      } else if (bz1 == bz_by_name.end()) {
        n_poles = std::max(1, bz2->second.n_poles);
      }
    }
    if (bz1 == bz_by_name.end() && bz2 == bz_by_name.end()) {
      warn(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
           lz.locator, "LZ line has no BZ terminal; one pole is assumed.");
    }

    DCBranch br;
    br.index = static_cast<int>(result.system.dc.branches.size());
    br.from_bus = dc1->second;
    br.to_bus = dc2->second;
    // R on the card is per pole; n poles in parallel give R/n (verified
    // against DSP: poles 2 -> 1 doubles the line loss).
    const double r_eff = lz.r_ohm / n_poles;
    if (udcn > 0.0) {
      br.r_pu = r_eff * mva_base / (udcn * udcn);
      br.base_kv = udcn;
    } else {
      warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
           lz.locator, "LZ card: no DC voltage known; DC line resistance "
                       "left 0.");
    }
    if (lz.i_rated_a > 0.0 && udcn > 0.0) {
      br.rate_a_mva = udcn * lz.i_rated_a / 1000.0;
    }
    br.inductance_mh = lz.inductance_mh;
    br.source_card = "LZ";
    br.name = "LZ_" + lz.name1 + "_" + lz.name2;
    result.system.dc.branches.push_back(std::move(br));
  }

  static bool solve_dense_linear(std::vector<std::vector<double>>& a,
                                 std::vector<double>& b) {
    const size_t n = b.size();
    if (a.size() != n) return false;
    for (size_t col = 0; col < n; ++col) {
      size_t pivot = col;
      for (size_t row = col + 1; row < n; ++row) {
        if (std::abs(a[row][col]) > std::abs(a[pivot][col])) pivot = row;
      }
      if (std::abs(a[pivot][col]) < 1e-12) return false;
      if (pivot != col) {
        std::swap(a[pivot], a[col]);
        std::swap(b[pivot], b[col]);
      }
      for (size_t row = col + 1; row < n; ++row) {
        const double factor = a[row][col] / a[col][col];
        if (factor == 0.0) continue;
        a[row][col] = 0.0;
        for (size_t k = col + 1; k < n; ++k) {
          a[row][k] -= factor * a[col][k];
        }
        b[row] -= factor * b[col];
      }
    }
    for (size_t rev = 0; rev < n; ++rev) {
      const size_t row = n - 1 - rev;
      double rhs = b[row];
      for (size_t col = row + 1; col < n; ++col) {
        rhs -= a[row][col] * b[col];
      }
      b[row] = rhs / a[row][row];
    }
    return true;
  }

  /// DSP layered control mode 1 fixes the BA rectifier terminal at the DC-card
  /// P/Udc boundary.  A mode-1 BZ in the same hybrid DC grid balances power,
  /// but its own card voltage is not the controlled physical terminal.  The
  /// generic solver realizes a BZ voltage former through a stiff droop, so a
  /// small resistive-network pre-solve translates the DSP boundary into the
  /// equivalent internal BZ droop target without changing Newton equations.
  void calibrate_layered_dc_voltage_targets() {
    if (layered_dc_controls.empty() || result.system.dc.buses.empty()) return;

    const size_t ndc = result.system.dc.buses.size();
    std::unordered_map<int, size_t> bus_pos;
    bus_pos.reserve(ndc);
    for (size_t i = 0; i < ndc; ++i) {
      bus_pos[result.system.dc.buses[i].index] = i;
    }

    std::vector<std::vector<std::pair<size_t, double>>> graph(ndc);
    for (const auto& branch : result.system.dc.branches) {
      if (!branch.in_service || !(branch.r_pu > 0.0)) continue;
      const auto from = bus_pos.find(branch.from_bus);
      const auto to = bus_pos.find(branch.to_bus);
      if (from == bus_pos.end() || to == bus_pos.end()) continue;
      const double g = 1.0 / branch.r_pu;
      graph[from->second].emplace_back(to->second, g);
      graph[to->second].emplace_back(from->second, g);
    }

    const double base_mva = mva_base > 0.0 ? mva_base : 100.0;
    for (const auto& control : layered_dc_controls) {
      if (control.control_mode != 1) {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnsupportedControl,
             control.locator,
             "DC layered-control mode " +
                 std::to_string(control.control_mode) +
                 " is retained as metadata but is not calibrated to a BZ "
                 "voltage-former target.");
        continue;
      }

      std::vector<const BaStation*> active_rectifiers;
      for (const std::string* name :
           {&control.rectifier_high, &control.rectifier_low}) {
        if (name->empty()) continue;
        const auto it = ba_by_name.find(*name);
        if (it != ba_by_name.end() && it->second.dc_bus > 0) {
          active_rectifiers.push_back(&it->second);
        }
      }
      if (active_rectifiers.size() != 1) {
        warn(ImportDisposition::Coerced,
             ImportReasonCode::UnsupportedControl, control.locator,
             "DC layered-control calibration requires exactly one active BA "
             "rectifier layer; high/low series sharing remains uncalibrated.");
        continue;
      }

      const BaStation& ba = *active_rectifiers.front();
      const auto ba_pos_it = bus_pos.find(ba.dc_bus);
      if (ba_pos_it == bus_pos.end()) continue;
      const size_t ba_pos = ba_pos_it->second;

      std::vector<size_t> component;
      std::vector<char> seen(ndc, 0);
      std::vector<size_t> stack{ba_pos};
      seen[ba_pos] = 1;
      while (!stack.empty()) {
        const size_t current = stack.back();
        stack.pop_back();
        component.push_back(current);
        for (const auto& [next, conductance] : graph[current]) {
          (void)conductance;
          if (!seen[next]) {
            seen[next] = 1;
            stack.push_back(next);
          }
        }
      }

      std::vector<size_t> balancing_vsc;
      for (size_t i = 0; i < result.system.vsc_converters.size(); ++i) {
        const auto& converter = result.system.vsc_converters[i];
        const auto pos = bus_pos.find(converter.bus_dc);
        if (converter.in_service && pos != bus_pos.end() && seen[pos->second] &&
            converter.control_mode == ConverterMode::VDC_Q) {
          balancing_vsc.push_back(i);
        }
      }
      if (balancing_vsc.size() != 1) {
        warn(ImportDisposition::Coerced,
             ImportReasonCode::UnsupportedControl, control.locator,
             "DC layered-control calibration requires exactly one mode-1 BZ "
             "balancing station in the BA-connected DC component.");
        continue;
      }

      VSCConverter& balance =
          result.system.vsc_converters[balancing_vsc.front()];
      const size_t balance_pos = bus_pos.at(balance.bus_dc);
      if (balance_pos == ba_pos || component.size() < 2 ||
          !(balance.k_vdc > 0.0)) {
        continue;
      }

      const double ba_base_kv =
          result.system.dc.buses[ba_pos].base_kv > 0.0
              ? result.system.dc.buses[ba_pos].base_kv
              : ba.rated_dc_kv;
      if (!(ba_base_kv > 0.0) || !(control.rectifier_v_sch_kv > 0.0)) {
        warn(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
             control.locator,
             "DC layered-control calibration skipped because the BA rated or "
             "scheduled rectifier voltage is missing.");
        continue;
      }
      const double ba_voltage = control.rectifier_v_sch_kv / ba_base_kv;

      std::vector<double> p_spec(ndc, 0.0);
      for (size_t pos : component) {
        p_spec[pos] -= result.system.dc.buses[pos].pd_mw / base_mva;
      }
      for (const auto& converter : result.system.vsc_converters) {
        if (!converter.in_service || &converter == &balance) continue;
        const auto pos = bus_pos.find(converter.bus_dc);
        if (pos == bus_pos.end() || !seen[pos->second]) continue;
        if (converter.control_mode != ConverterMode::PQ_MODE &&
            converter.control_mode != ConverterMode::AC_PV &&
            converter.control_mode != ConverterMode::AC_GRID_FORMING) {
          continue;
        }
        const double p = converter.p_set_mw / base_mva;
        const double loss = (1.0 - converter.eta) * std::abs(p);
        p_spec[pos->second] -= p + loss;
      }
      for (const auto& converter : result.system.lcc_converters) {
        if (!converter.in_service ||
            converter.control_mode != LCCControlMode::ConstantPower) {
          continue;
        }
        const auto pos = bus_pos.find(converter.dc_bus);
        if (pos == bus_pos.end() || !seen[pos->second]) continue;
        const double sign = converter.station_role == LCCStationRole::Rectifier
                                ? 1.0
                                : -1.0;
        p_spec[pos->second] += sign * converter.p_set_mw / base_mva;
      }

      std::vector<size_t> variable_buses;
      std::vector<size_t> equation_buses;
      for (size_t pos : component) {
        if (pos != ba_pos) variable_buses.push_back(pos);
        if (pos != balance_pos) equation_buses.push_back(pos);
      }
      if (variable_buses.size() != equation_buses.size()) continue;

      std::unordered_map<size_t, size_t> variable_col;
      for (size_t i = 0; i < variable_buses.size(); ++i) {
        variable_col[variable_buses[i]] = i;
      }
      std::vector<double> voltage(ndc, 1.0);
      voltage[ba_pos] = ba_voltage;
      bool converged = false;
      for (int iteration = 0; iteration < 40; ++iteration) {
        const size_t n = variable_buses.size();
        std::vector<double> mismatch(n, 0.0);
        std::vector<std::vector<double>> jacobian(
            n, std::vector<double>(n, 0.0));
        double max_mismatch = 0.0;
        for (size_t row = 0; row < n; ++row) {
          const size_t i = equation_buses[row];
          double current = 0.0;
          double diagonal_g = 0.0;
          for (const auto& [j, g] : graph[i]) {
            if (!seen[j]) continue;
            current += g * (voltage[i] - voltage[j]);
            diagonal_g += g;
            const auto col = variable_col.find(j);
            if (col != variable_col.end()) {
              jacobian[row][col->second] -= voltage[i] * g;
            }
          }
          mismatch[row] = voltage[i] * current - p_spec[i];
          max_mismatch = std::max(max_mismatch, std::abs(mismatch[row]));
          const auto diagonal = variable_col.find(i);
          if (diagonal != variable_col.end()) {
            jacobian[row][diagonal->second] += current +
                                               voltage[i] * diagonal_g;
          }
        }
        // Near-zero LZ links are common between a passive BB junction and a
        // small auxiliary BZ terminal.  Their large conductance causes benign
        // cancellation at roughly 1e-9--1e-8 pu in double precision; 1e-8 pu
        // is already sub-watt on the 100 MVA BPA base.
        if (max_mismatch < 1e-8) {
          converged = true;
          break;
        }
        for (double& value : mismatch) value = -value;
        if (!solve_dense_linear(jacobian, mismatch)) break;

        double step = 1.0;
        for (size_t col = 0; col < variable_buses.size(); ++col) {
          const double dv = mismatch[col];
          if (dv < 0.0) {
            step = std::min(step,
                            0.8 * voltage[variable_buses[col]] / -dv);
          }
        }
        step = std::clamp(step, 1e-3, 1.0);
        for (size_t col = 0; col < variable_buses.size(); ++col) {
          voltage[variable_buses[col]] += step * mismatch[col];
        }
      }
      if (!converged) {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnsupportedControl,
             control.locator,
             "DC layered-control resistive pre-solve did not converge; the "
             "original BZ voltage target is retained.");
        continue;
      }

      double balance_p = 0.0;
      for (const auto& [other, conductance] : graph[balance_pos]) {
        if (seen[other]) {
          balance_p += voltage[balance_pos] * conductance *
                       (voltage[balance_pos] - voltage[other]);
        }
      }
      const double target_sq = voltage[balance_pos] * voltage[balance_pos] +
                               balance_p / balance.k_vdc;
      if (!(target_sq > 0.0) || !std::isfinite(target_sq)) {
        warn(ImportDisposition::Coerced, ImportReasonCode::UnsupportedControl,
             control.locator,
             "DC layered-control calibration produced an invalid BZ droop "
             "target; the original target is retained.");
        continue;
      }

      balance.v_dc_set_pu = std::sqrt(target_sq);
      for (size_t pos : component) {
        result.system.dc.buses[pos].vm_pu = voltage[pos];
      }
      std::ostringstream message;
      message << "DC mode-1 layered control calibrated: BA '" << ba.name
              << "' holds " << control.rectifier_v_sch_kv
              << " kV at " << control.p_sch_mw
              << " MW; balancing BZ '"
              << result.system.dc.buses[balance_pos].name
              << "' uses internal droop target " << balance.v_dc_set_pu
              << " pu for a predicted physical terminal voltage "
              << voltage[balance_pos] << " pu.";
      result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                        ImportSeverity::Info, control.locator, message.str());
    }
  }

  // ── Line dispatch ──────────────────────────────────────────────────────
  void parse_control_card(const std::string& line) {
    const std::string up = line;
    auto extract = [&](const std::string& key, bool allow_space) -> std::string {
      const size_t p = up.find(key);
      if (p == std::string::npos) return {};
      size_t b = p + key.size();
      size_t e = b;
      while (e < up.size() && up[e] != ',' && up[e] != ')' && up[e] != '\\' &&
             (allow_space || up[e] != ' ')) {
        ++e;
      }
      return trim(up.substr(b, e - b));
    };
    if (const std::string v = extract("MVA_BASE=", false); !v.empty()) {
      bool present = false;
      const double b = num(v, present);
      if (present && b > 0.0) mva_base = b;
    }
    if (const std::string v = extract("CASEID=", false); !v.empty()) case_id = v;
    if (const std::string v = extract("PROJECT=", true); !v.empty()) project = v;
  }

  void run(const std::string& content) {
    names_need_gbk = !is_valid_utf8(content);

    std::istringstream in(content);
    std::string line;
    size_t line_no = 0;
    bool end_seen = false;
    while (!end_seen && std::getline(in, line)) {
      ++line_no;
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (trim(line).empty()) continue;
      // Network/control card types occupy column 1. Indented nonblank lines
      // in production decks are free-form equipment descriptions.
      if (line[0] == ' ' || line[0] == '\t') continue;
      const char c0 = line[0];
      const std::string locator =
          "line " + std::to_string(line_no);

      if (c0 == '.') continue;  // .DSP header / comment banners
      if (c0 == '(') {
        if (line.find("END") != std::string::npos) {
          end_seen = true;
        } else {
          parse_control_card(line);
        }
        continue;
      }
      if (c0 == '/' || c0 == '>') {
        parse_control_card(line);
        continue;
      }

      current_source_order = static_cast<int>(line_no);

      const std::string type = raw_field(line, 1, 2);
      if (type == "B " || type == "BS" || type == "BE" || type == "BQ") {
        parse_bus_card(line, locator);
      } else if (type == "L ") {
        parse_line_card(line, locator);
      } else if (type == "L+") {
        parse_line_plus_card(line, locator);
      } else if (type == "T ") {
        parse_transformer_card(line, locator);
      } else if (type == "BD") {
        parse_bd_card(line, locator);
      } else if (type == "LD") {
        parse_ld_card(line, locator);
      } else if (type == "BA") {
        parse_ba_card(line, locator);
      } else if (type == "DC") {
        parse_layered_dc_control(line, locator);
      } else if (type == "BB") {
        parse_bb_card(line, locator);
      } else if (type == "LY") {
        parse_native_dc_line(line, locator, "LY");
      } else if (type == "BM") {
        parse_bm_card(line, locator);
      } else if (type == "LM") {
        parse_native_dc_line(line, locator, "LM");
      } else if (type == "BZ") {
        // BZ+ shares the leading "BZ"; column 3 disambiguates.
        if (line.size() > 2 && line[2] == '+') {
          parse_bz_plus_card(line, locator);
        } else {
          parse_bz_card(line, locator);
        }
      } else if (type == "LZ") {
        parse_lz_card(line, locator);
      } else if (type == "R ") {
        parse_r_card(line, locator);
      } else {
        warn(ImportDisposition::Skipped, ImportReasonCode::UnknownField, locator,
             "Unrecognized card type '" + name_of(raw_field(line, 1, 2)) +
                 "'; line skipped.");
      }
    }

    // L+ cards repeat the parent L-card identity, so they can be bound after
    // the full deck has declared every line and formal bus.
    for (const auto& extension : pending_line_shunts) {
      make_line_shunts(extension);
    }

    // Materialize pending LCC links now that every T card has been seen (the
    // converter transformer may be declared after the LD card).
    for (const auto& link : pending_lcc_links) {
      const auto it_r = bd_by_name.find(link.rect_name);
      const auto it_i = bd_by_name.find(link.inv_name);
      if (it_r == bd_by_name.end() || it_i == bd_by_name.end()) continue;
      make_lcc_station(it_r->second, LCCStationRole::Rectifier, link.locator,
                       link.p_sch, link.vdc_kv, link.vdc_kv,
                       link.alpha_n_deg, link.gamma_n_deg);
      make_lcc_station(it_i->second, LCCStationRole::Inverter, link.locator,
                       link.p_sch, link.vdc_kv, link.inv_vdc_set_kv,
                       link.alpha_n_deg, link.gamma_n_deg);
      std::ostringstream lcc_msg;
      lcc_msg << "LCC HVDC link '" << link.rect_name << "' -> '"
              << link.inv_name << "' (P=" << link.p_sch
              << " MW, V=" << link.vdc_kv
              << " kV) imported as two quasi-steady LCCConverter elements "
                 "(consumed by the unified Newton power flow; applicable R "
                 "cards activate the converter-transformer tap outer loop).";
      result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                        ImportSeverity::Info, link.locator, lcc_msg.str());
    }

    // Materialize layered and multi-terminal LCC stations only after every
    // BA continuation, DC control card, T card, and R card has been read.
    for (const auto& name : ba_order) {
      make_ba_station(ba_by_name.at(name));
    }
    for (const auto& name : bm_order) {
      make_bm_station(bm_by_name.at(name));
    }
    for (const auto& branch : pending_native_dc_lines) {
      make_native_dc_line(branch);
    }

    // Materialize VSC-HVDC stations (BZ/BZ+) and lines (LZ) now that every
    // card has been seen: the BZ+ continuation and the LZ terminals may in
    // principle appear anywhere after their BZ cards.
    for (const auto& name : bz_order) {
      make_vsc_station(bz_by_name.at(name));
    }
    for (const auto& lz : pending_lz_links) {
      make_lz_branch(lz);
    }
    calibrate_layered_dc_voltage_targets();

    HybridPowerSystem& sys = result.system;
    sys.base_mva = mva_base;
    sys.ac.base_mva = mva_base;
    sys.dc.base_mva = mva_base;
    sys.name = !project.empty() ? project
               : (!case_id.empty() ? case_id : "BPA case");
    // Control-card PROJECT/CASEID values may carry GBK bytes; the model name
    // must be valid UTF-8 for JSON serialisation.
    if (!is_valid_utf8(sys.name)) sys.name = gbk_to_utf8(sys.name);
    sys.ac.name = sys.name + " AC";
    sys.dc.name = sys.name + " DC";
    if (names_need_gbk) {
      result.report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
                        ImportSeverity::Info, "file",
                        "File is not valid UTF-8; bus names converted from GBK "
                        "(CP936) to UTF-8.");
    }
    std::ostringstream ok;
    ok << "Parsed BPA case '" << sys.name << "': " << sys.ac.buses.size()
       << " AC buses, " << sys.ac.branches.size() << " AC branches, "
       << sys.ac.generators.size() << " generators, " << sys.ac.loads.size()
       << " loads, " << sys.dc.buses.size() << " DC buses, "
       << sys.dc.branches.size() << " DC branches, "
       << sys.vsc_converters.size() << " VSC + "
       << sys.lcc_converters.size() << " LCC converters (base " << mva_base
       << " MVA).";
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, "file", ok.str());
  }
};

}  // namespace

BpaImportResult parse_bpa_dat_string(const std::string& content,
                                     const BpaImportOptions& options) {
  return parse_bpa_dat_string(content, options,
                              BpaSmallReactanceMode::DspCompatible);
}

BpaImportResult parse_bpa_dat_string(
    const std::string& content, const BpaImportOptions& options,
    BpaSmallReactanceMode reactance_mode) {
  Importer imp(options, reactance_mode);
  imp.run(content);
  return std::move(imp.result);
}

BpaImportResult parse_bpa_dat(const std::string& filepath,
                              const BpaImportOptions& options) {
  return parse_bpa_dat(filepath, options,
                       BpaSmallReactanceMode::DspCompatible);
}

BpaImportResult parse_bpa_dat(const std::string& filepath,
                              const BpaImportOptions& options,
                              BpaSmallReactanceMode reactance_mode) {
  std::ifstream in(filepath, std::ios::binary);
  if (!in) {
    BpaImportResult res;
    res.report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
                   ImportSeverity::Error, filepath, "Cannot open file.");
    return res;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  BpaImportResult res =
      parse_bpa_dat_string(ss.str(), options, reactance_mode);
  if (res.system.name == "BPA case") {
    // Fall back to the file stem; on a GBK-locale Windows host argv arrives
    // in the ANSI codepage, so re-encode to UTF-8 for the JSON output.
    const size_t slash = filepath.find_last_of("/\\");
    const size_t dot = filepath.find_last_of('.');
    std::string stem = filepath.substr(
        slash == std::string::npos ? 0 : slash + 1,
        (dot == std::string::npos ? filepath.size() : dot) -
            (slash == std::string::npos ? 0 : slash + 1));
    res.system.name = stem;
  }
  // Control-card PROJECT/CASEID values may carry GBK bytes too; the model
  // name must be valid UTF-8 for JSON serialisation.
  if (!is_valid_utf8(res.system.name)) {
    res.system.name = gbk_to_utf8(res.system.name);
  }
  return res;
}

std::string to_bpa_dat(const HybridPowerSystem& system) {
  using Values = std::unordered_map<int, double>;
  struct GenValues {
    double p{0.0};
    double pmax{0.0};
    double qmax{0.0};
    double qmin{0.0};
    double vg{1.0};
    bool present{false};
    bool slack{false};
  };

  auto number = [](double value, std::size_t width) {
    if (!std::isfinite(value)) value = 0.0;
    for (int precision = 6; precision >= 0; --precision) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(precision) << value;
      std::string text = out.str();
      if (precision > 0) {
        while (!text.empty() && text.back() == '0') text.pop_back();
        if (!text.empty() && text.back() == '.') text.pop_back();
      }
      if (text == "-0") text = "0";
      if (text.size() <= width) return text;
    }
    for (int precision = 3; precision >= 0; --precision) {
      std::ostringstream out;
      out << std::uppercase << std::scientific << std::setprecision(precision)
          << value;
      const std::string text = out.str();
      if (text.size() <= width) return text;
    }
    throw std::runtime_error("BPA export: numeric value does not fit a fixed-width field");
  };
  auto put = [](std::string& card, std::size_t first, std::size_t last,
                const std::string& value, bool right = true) {
    if (first == 0 || first > last || last > card.size()) {
      throw std::runtime_error("BPA export: invalid card field bounds");
    }
    const std::size_t width = last - first + 1;
    if (value.size() > width) {
      throw std::runtime_error("BPA export: card value exceeds field width");
    }
    const std::size_t offset = first - 1 + (right ? width - value.size() : 0);
    card.replace(offset, value.size(), value);
  };
  auto bus_card_name = [](std::size_t ordinal) {
    std::ostringstream out;
    out << 'A' << std::setw(7) << std::setfill('0') << ordinal;
    return out.str();
  };

  std::unordered_map<int, const ACBus*> ac_bus;
  std::unordered_map<int, std::string> ac_name;
  for (std::size_t i = 0; i < system.ac.buses.size(); ++i) {
    const auto& bus = system.ac.buses[i];
    ac_bus[bus.index] = &bus;
    ac_name[bus.index] = bus_card_name(i + 1);
  }

  Values load_p, load_q, shunt_g, shunt_b;
  std::unordered_map<int, GenValues> generation;
  for (const auto& bus : system.ac.buses) {
    load_p[bus.index] += bus.pd_mw;
    load_q[bus.index] += bus.qd_mvar;
    shunt_g[bus.index] += bus.gs_mw;
    shunt_b[bus.index] += bus.bs_mvar;
  }
  for (const auto& load : system.ac.loads) {
    if (!load.in_service) continue;
    load_p[load.bus] += load.p_mw * load.scaling;
    load_q[load.bus] += load.q_mvar * load.scaling;
  }
  for (const auto& shunt : system.ac.shunts) {
    if (!shunt.in_service) continue;
    shunt_g[shunt.bus] += shunt.gs_mw;
    shunt_b[shunt.bus] += shunt.bs_mvar;
  }
  for (const auto& gen : system.ac.generators) {
    if (!gen.in_service) continue;
    auto& values = generation[gen.bus];
    values.p += gen.pg_mw;
    values.pmax += gen.pmax_mw;
    values.qmax += gen.qmax_mvar;
    values.qmin += gen.qmin_mvar;
    values.vg = gen.vg_pu;
    values.present = true;
    values.slack = values.slack || gen.is_slack;
  }

  std::ostringstream output;
  output << ".DSP 01000\n"
         << "(POWERFLOW,CASEID=HACDCPF,PROJECT=HACDCPF_EXPORT)\n"
         << "/MVA_BASE=" << (system.base_mva > 0.0 ? system.base_mva : 100.0)
         << "\\\n/NETWORK_DATA\\\n";

  for (const auto& bus : system.ac.buses) {
    std::string card(81, ' ');
    const auto gen_it = generation.find(bus.index);
    const GenValues gen = gen_it == generation.end() ? GenValues{} : gen_it->second;
    const bool slack = bus.bus_type == BusType::SLACK || gen.slack;
    const bool pv = bus.bus_type == BusType::PV || gen.present;
    put(card, 1, 2, slack ? "BS" : (pv ? "BE" : "B "), false);
    put(card, 7, 14, ac_name.at(bus.index), false);
    put(card, 15, 18, number(bus.base_kv, 4));
    put(card, 19, 20, number(bus.zone, 2));
    put(card, 21, 25, number(load_p[bus.index], 5));
    put(card, 26, 30, number(load_q[bus.index], 5));
    put(card, 31, 34, number(shunt_g[bus.index], 4));
    put(card, 35, 38, number(shunt_b[bus.index], 4));
    if (pv) {
      put(card, 39, 42, number(gen.pmax, 4));
      put(card, 43, 47, number(gen.p, 5));
      put(card, 48, 52, number(gen.qmax, 5));
      put(card, 53, 57, number(gen.qmin, 5));
      put(card, 58, 61, number(gen.present ? gen.vg : bus.vm_pu, 4));
      put(card, 62, 65, number(slack ? bus.va_deg : bus.vmin_pu, 4));
    } else {
      put(card, 58, 61, number(bus.vmax_pu, 4));
      put(card, 62, 65, number(bus.vmin_pu, 4));
    }
    output << card << '\n';
  }

  std::unordered_map<int, const VSCConverter*> dc_converter;
  for (const auto& converter : system.vsc_converters) {
    if (converter.in_service && ac_name.count(converter.bus_ac)) {
      dc_converter.emplace(converter.bus_dc, &converter);
    }
  }
  for (const auto& dc_bus : system.dc.buses) {
    const auto converter_it = dc_converter.find(dc_bus.index);
    if (converter_it == dc_converter.end()) continue;
    const auto* converter = converter_it->second;
    std::string card(81, ' ');
    put(card, 1, 2, "BD", false);
    put(card, 7, 14, ac_name.at(converter->bus_ac), false);
    const auto ac_it = ac_bus.find(converter->bus_ac);
    put(card, 15, 18, number(ac_it->second->base_kv, 4));
    const double dc_kv = dc_bus.base_kv > 0.0 ? dc_bus.base_kv : converter->vn_dc_kv;
    put(card, 63, 66, number(dc_kv, 4));
    output << card << '\n';
  }

  // Native LCC stations (quasi-steady model): one BD card per station.  The
  // primary-side bus is recovered through the converter-transformer branch
  // (the end that is not the valve-side bus), mirroring the import logic.
  std::unordered_map<int, const LCCConverter*> lcc_by_dc_bus;
  for (const auto& lcc : system.lcc_converters) {
    if (lcc.in_service && ac_name.count(lcc.ac_bus)) {
      lcc_by_dc_bus.emplace(lcc.dc_bus, &lcc);
    }
  }
  for (const auto& dc_bus : system.dc.buses) {
    const auto lcc_it = lcc_by_dc_bus.find(dc_bus.index);
    if (lcc_it == lcc_by_dc_bus.end()) continue;
    const auto* lcc = lcc_it->second;
    std::string card(81, ' ');
    put(card, 1, 2, "BD", false);
    put(card, 7, 14, ac_name.at(lcc->ac_bus), false);
    put(card, 15, 18, number(lcc->vn_ac_kv, 4));
    if (lcc->n_bridges > 0) put(card, 21, 25, number(lcc->n_bridges, 4));
    if (lcc->smoothing_reactor_mh > 0.0) {
      put(card, 26, 30, number(lcc->smoothing_reactor_mh, 4));
    }
    if (lcc->alpha_min_deg > 0.0) put(card, 31, 35, number(lcc->alpha_min_deg, 4));
    if (lcc->alpha_stop_deg > 0.0) put(card, 36, 40, number(lcc->alpha_stop_deg, 4));
    if (lcc->v_drop_v > 0.0) put(card, 41, 45, number(lcc->v_drop_v, 4));
    if (lcc->rated_current_a > 0.0) put(card, 46, 50, number(lcc->rated_current_a, 4));
    if (lcc->converter_transformer_branch >= 0) {
      for (const auto& br : system.ac.branches) {
        if (br.index != lcc->converter_transformer_branch) continue;
        const int primary = (br.from_bus == lcc->ac_bus) ? br.to_bus : br.from_bus;
        if (ac_name.count(primary)) {
          put(card, 51, 58, ac_name.at(primary), false);
          const auto pb = ac_bus.find(primary);
          if (pb != ac_bus.end()) put(card, 59, 62, number(pb->second->base_kv, 4));
        }
        break;
      }
    }
    const double dc_kv = dc_bus.base_kv > 0.0 ? dc_bus.base_kv : lcc->rated_dc_kv;
    put(card, 63, 66, number(dc_kv, 4));
    output << card << '\n';
  }

  for (const auto& branch : system.ac.branches) {
    if (!branch.in_service || !ac_name.count(branch.from_bus) ||
        !ac_name.count(branch.to_bus)) continue;
    const auto* from = ac_bus.at(branch.from_bus);
    const auto* to = ac_bus.at(branch.to_bus);
    const bool transformer = std::abs(branch.tap - 1.0) > 1e-9 ||
                             branch.sn_mva > 0.0 || branch.vn_hv_kv > 0.0 ||
                             branch.vn_lv_kv > 0.0 ||
                             branch.name.rfind("T_", 0) == 0;
    std::string card(81, ' ');
    put(card, 1, 2, transformer ? "T " : "L ", false);
    put(card, 7, 14, ac_name.at(branch.from_bus), false);
    put(card, 15, 18, number(from->base_kv, 4));
    put(card, 20, 27, ac_name.at(branch.to_bus), false);
    put(card, 28, 31, number(to->base_kv, 4));
    put(card, 32, 32, "1", false);
    const double rating = transformer && branch.sn_mva > 0.0
                              ? branch.sn_mva
                              : (branch.rate_a_mva > 0.0 && from->base_kv > 0.0
                                     ? branch.rate_a_mva * 1000.0 /
                                           (std::sqrt(3.0) * from->base_kv)
                                     : 0.0);
    put(card, 34, 37, number(rating, 4));
    put(card, 38, 38, number(std::max(1, branch.n_parallel), 1));
    put(card, 39, 44, number(branch.r_pu, 6));
    put(card, 45, 50, number(branch.x_pu, 6));
    if (transformer) {
      put(card, 62, 67, number(branch.tap * from->base_kv, 6));
      put(card, 68, 73, number(to->base_kv, 6));
    } else {
      put(card, 57, 62, number(branch.b_pu / 2.0, 6));
      put(card, 63, 68, number(branch.length_km, 6));
    }
    output << card << '\n';
  }

  const double mva_base = system.base_mva > 0.0 ? system.base_mva : 100.0;
  for (const auto& branch : system.dc.branches) {
    if (!branch.in_service) continue;
    const double dc_kv_fallback = [&] {
      double kv = 0.0;
      for (const auto& dc_bus : system.dc.buses) {
        if ((dc_bus.index == branch.from_bus || dc_bus.index == branch.to_bus) &&
            dc_bus.base_kv > kv) {
          kv = dc_bus.base_kv;
        }
      }
      return kv;
    }();
    const double dc_kv = branch.base_kv > 0.0 ? branch.base_kv : dc_kv_fallback;
    const double resistance =
        dc_kv > 0.0 ? branch.r_pu * dc_kv * dc_kv / mva_base : 0.0;
    double scheduled = 0.0;
    double alpha_n = 0.0, gamma_n = 0.0, rated_a = 0.0;
    std::string rect_name, inv_name;

    const auto from_vsc = dc_converter.find(branch.from_bus);
    const auto to_vsc = dc_converter.find(branch.to_bus);
    const auto from_lcc = lcc_by_dc_bus.find(branch.from_bus);
    const auto to_lcc = lcc_by_dc_bus.find(branch.to_bus);
    if (from_vsc != dc_converter.end() && to_vsc != dc_converter.end()) {
      const auto* rectifier = from_vsc->second;
      const auto* inverter = to_vsc->second;
      scheduled = rectifier->p_set_mw < 0.0
                      ? -rectifier->p_set_mw
                      : std::abs(rectifier->p_schedule_mw);
      if (scheduled == 0.0) scheduled = std::abs(inverter->p_schedule_mw);
      rect_name = ac_name.at(rectifier->bus_ac);
      inv_name = ac_name.at(inverter->bus_ac);
    } else if (from_lcc != lcc_by_dc_bus.end() &&
               to_lcc != lcc_by_dc_bus.end()) {
      // Native LCC link: from-bus station is the rectifier (LD convention).
      const auto* rectifier = from_lcc->second;
      const auto* inverter = to_lcc->second;
      scheduled = std::abs(rectifier->p_set_mw);
      alpha_n = rectifier->alpha_set_deg;
      gamma_n = inverter->gamma_set_deg;
      rated_a = std::max(rectifier->rated_current_a, inverter->rated_current_a);
      rect_name = ac_name.at(rectifier->ac_bus);
      inv_name = ac_name.at(inverter->ac_bus);
    } else {
      continue;  // DC branch without a converter pair on both ends
    }
    if (rated_a <= 0.0 && branch.rate_a_mva > 0.0 && dc_kv > 0.0) {
      rated_a = branch.rate_a_mva * 1000.0 / dc_kv;
    }
    std::string card(81, ' ');
    put(card, 1, 2, "LD", false);
    put(card, 7, 14, rect_name, false);
    put(card, 20, 27, inv_name, false);
    if (rated_a > 0.0) put(card, 34, 37, number(rated_a, 4));
    put(card, 38, 41, number(resistance, 4));
    if (scheduled > 0.0) {
      put(card, 56, 56, "R", false);
      put(card, 57, 61, number(scheduled, 5));
    }
    if (dc_kv > 0.0) put(card, 62, 66, number(dc_kv, 5));
    if (alpha_n > 0.0) put(card, 67, 70, number(alpha_n, 4));
    if (gamma_n > 0.0) put(card, 71, 74, number(gamma_n, 4));
    put(card, 77, 81, number(branch.length_km, 5));
    output << card << '\n';
  }

  output << "(END)\n";
  return output.str();
}

void save_bpa_dat(const HybridPowerSystem& system,
                  const std::string& filepath) {
  std::ofstream output(filepath, std::ios::binary);
  if (!output) {
    throw std::runtime_error("BPA export: cannot open for write " + filepath);
  }
  output << to_bpa_dat(system);
}

}  // namespace hacdcpf::io
