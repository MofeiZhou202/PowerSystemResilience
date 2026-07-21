/// io/bpa_io.cpp
/// ===============
/// PSD-BPA / DSP card-file (.dat) importer — see bpa_io.hpp.
///
/// Card columns are 1-based and inclusive, fixed-width.  Numeric fields may
/// carry an explicit decimal point anywhere in the field; when the point is
/// omitted some fields use an implied-decimal convention (Vsch: F4.3,
/// transformer taps: F6.2).  Blank numeric fields default to zero / absent.
///
/// GBK-encoded files (Chinese bus names, e.g. IEEE90.dat) are parsed in the
/// raw byte domain so fixed columns stay aligned (GBK characters are 2 bytes);
/// only the extracted name tokens are converted to UTF-8 for the model.

#include "hacdcpf/io/bpa_io.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace hacdcpf::io {
namespace {

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
  double dc_kv{0.0};      // rated DC voltage from the BD card (may be 0)
  double bridges{0.0};
  double vdrop_v{0.0};    // valve voltage drop per bridge (V), cols 41-45
  double bridge_in_a{0.0};// rated bridge current (A), cols 46-50
};

struct Importer {
  explicit Importer(const BpaImportOptions& opt) : options(opt) {}

  BpaImportOptions options;
  BpaImportResult result;
  bool names_need_gbk{false};
  double mva_base{100.0};
  std::string case_id;
  std::string project;

  std::unordered_map<std::string, int> ac_bus_by_name;
  std::unordered_map<std::string, int> dc_bus_by_name;
  std::unordered_map<std::string, BdStation> bd_by_name;
  bool g_half_note_emitted{false};
  bool magnetizing_note_emitted{false};

  std::string name_of(const std::string& raw) const {
    return names_need_gbk ? gbk_to_utf8(trim(raw)) : trim(raw);
  }

  void warn(ImportDisposition disp, ImportReasonCode code,
            const std::string& locator, const std::string& msg) {
    result.report.add(disp, code, ImportSeverity::Warning, locator, msg);
  }

  /// Fetch (or create as PQ stub) an AC bus by UTF-8 name.  When
  /// @p quiet is true the creation is not reported (used by cards that are
  /// themselves a bus declaration, e.g. BD).
  int ensure_ac_bus(const std::string& name, double base_kv,
                    const std::string& locator, bool quiet = false) {
    auto it = ac_bus_by_name.find(name);
    if (it != ac_bus_by_name.end()) return it->second;
    ACBus bus;
    bus.index = static_cast<int>(result.system.ac.buses.size()) + 1;
    bus.bus_type = BusType::PQ;
    bus.base_kv = base_kv > 0.0 ? base_kv : 110.0;
    bus.name = name;
    if (!quiet) {
      warn(ImportDisposition::Coerced, ImportReasonCode::UnresolvedBusRef,
           locator, "Bus '" + name + "' referenced before declaration; created "
                    "as PQ stub (base_kv from referencing card).");
    }
    result.system.ac.buses.push_back(bus);
    ac_bus_by_name.emplace(name, bus.index);
    return bus.index;
  }

  int ensure_dc_bus(const std::string& name, double base_kv) {
    auto it = dc_bus_by_name.find(name);
    if (it != dc_bus_by_name.end()) return it->second;
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
    bus.index = static_cast<int>(result.system.ac.buses.size()) + 1;
    bus.base_kv = kv > 0.0 ? kv : 110.0;
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
    result.system.ac.buses.push_back(bus);
    ac_bus_by_name.emplace(name, bus.index);

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
    if (type != "B") {
      Generator gen;
      gen.index = static_cast<int>(result.system.ac.generators.size());
      gen.bus = bus.index;
      gen.pg_mw = pgen;
      gen.vg_pu = vsch > 0.0 ? vsch : 1.0;
      gen.pmax_mw = pgenmax;
      gen.qmax_mvar = qgenmax;
      gen.qmin_mvar = qgenmin;
      gen.is_slack = (type == "BS");
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
    result.system.ac.branches.push_back(std::move(br));
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
    result.system.ac.branches.push_back(std::move(br));
  }

  // ── Two-terminal HVDC node card: BD ────────────────────────────────────
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
    st.bridges = num(field(line, 21, 25), present);
    st.vdrop_v = num(field(line, 41, 45), present);
    st.bridge_in_a = num(field(line, 46, 50), present);
    st.dc_kv = num(field(line, 63, 66), present);
    st.dc_bus = ensure_dc_bus(name, st.dc_kv);
    bd_by_name.emplace(name, st);
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
    const double p_sch = num(field(line, 57, 61), present);
    const double vdc_rect = num(field(line, 62, 66), present);
    const double length = num(field(line, 77, 81), present);
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
    br.name = "LD_" + n_rect + "_" + n_inv;
    result.system.dc.branches.push_back(std::move(br));

    // LCC link approximated by VSC converters (the model library has no LCC
    // element).  Station roles mirror the physical LCC link:
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
      const char c0 = line.size() > 0 ? line[0] : ' ';
      const char c1 = line.size() > 1 ? line[1] : ' ';
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

      const std::string type = raw_field(line, 1, 2);
      if (type == "B " || type == "BS" || type == "BE" || type == "BQ") {
        parse_bus_card(line, locator);
      } else if (type == "L ") {
        parse_line_card(line, locator);
      } else if (type == "T ") {
        parse_transformer_card(line, locator);
      } else if (type == "BD") {
        parse_bd_card(line, locator);
      } else if (type == "LD") {
        parse_ld_card(line, locator);
      } else if (type == "R ") {
        warn(ImportDisposition::Skipped, ImportReasonCode::UnsupportedControl,
             locator, "R (LTC tap-control) card not modeled; transformer tap "
                      "held at the fixed T-card value.");
      } else if (type == "BA" || type == "LY" || type == "DC" || type == "BB" ||
                 type == "LZ" || type == "BM" || type == "LM") {
        warn(ImportDisposition::Skipped, ImportReasonCode::UnsupportedControl,
             locator, type + " card (multi-terminal / LCCDC DC data) is not "
                      "supported by this importer; skipped.");
      } else {
        warn(ImportDisposition::Skipped, ImportReasonCode::UnknownField, locator,
             "Unrecognized card type '" + name_of(raw_field(line, 1, 2)) +
                 "'; line skipped.");
      }
    }

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
       << sys.vsc_converters.size() << " converters (base " << mva_base
       << " MVA).";
    result.report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
                      ImportSeverity::Info, "file", ok.str());
  }
};

}  // namespace

BpaImportResult parse_bpa_dat_string(const std::string& content,
                                     const BpaImportOptions& options) {
  Importer imp(options);
  imp.run(content);
  return std::move(imp.result);
}

BpaImportResult parse_bpa_dat(const std::string& filepath,
                              const BpaImportOptions& options) {
  std::ifstream in(filepath, std::ios::binary);
  if (!in) {
    BpaImportResult res;
    res.report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
                   ImportSeverity::Error, filepath, "Cannot open file.");
    return res;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  BpaImportResult res = parse_bpa_dat_string(ss.str(), options);
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
    const auto from_converter = dc_converter.find(branch.from_bus);
    const auto to_converter = dc_converter.find(branch.to_bus);
    if (from_converter == dc_converter.end() || to_converter == dc_converter.end()) continue;
    const auto* rectifier = from_converter->second;
    const auto* inverter = to_converter->second;
    const double dc_kv = branch.base_kv > 0.0
                             ? branch.base_kv
                             : std::max(rectifier->vn_dc_kv, inverter->vn_dc_kv);
    const double resistance = dc_kv > 0.0
                                  ? branch.r_pu * dc_kv * dc_kv / mva_base
                                  : 0.0;
    double scheduled = rectifier->p_set_mw < 0.0
                           ? -rectifier->p_set_mw
                           : std::abs(rectifier->p_schedule_mw);
    if (scheduled == 0.0) scheduled = std::abs(inverter->p_schedule_mw);
    std::string card(81, ' ');
    put(card, 1, 2, "LD", false);
    put(card, 7, 14, ac_name.at(rectifier->bus_ac), false);
    put(card, 20, 27, ac_name.at(inverter->bus_ac), false);
    if (branch.rate_a_mva > 0.0 && dc_kv > 0.0) {
      put(card, 34, 37, number(branch.rate_a_mva * 1000.0 / dc_kv, 4));
    }
    put(card, 38, 41, number(resistance, 4));
    put(card, 57, 61, number(scheduled, 5));
    put(card, 62, 66, number(dc_kv, 5));
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
