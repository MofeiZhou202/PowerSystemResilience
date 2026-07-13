#include "hacdcpf/io/cim_dist_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/// Import / export for the Chinese distribution-grid CIM/RDF dialect (config
/// 台区). See cim_dist_io.hpp for the class/attribute crosswalk and the honest
/// ceiling (loads and line/transformer impedances are ESTIMATED). The RDF/XML
/// reader is the same self-contained, XXE-hardened parser used by cim_io.cpp.

namespace hacdcpf::io {
namespace {

constexpr std::size_t knpos = std::string::npos;
// The distribution dialect namespaces (declared here for the exporter).
const std::string kCimNs = "http://iec.ch/TC57/2003/CIM-schema-cim10#";
const std::string kRdfNs = "http://www.w3.org/1999/02/22-rdf-syntax-ns#";
const std::string kCimsNs =
    "http://iec.ch/TC57/1999/rdf-schema-extensions-19990926#";
const std::string kPfNs = "http://www.chinapower.cn/Rfs/2006/Rdf-Cim#";
const std::string kRdfsNs = "http://www.w3.org/2000/01/rdf-schema#";

std::string trim(const std::string& s) {
  std::size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool contains(const std::string& hay, const std::string& needle) {
  return hay.find(needle) != knpos;
}

std::string xml_escape(const std::string& s) {
  std::string o;
  o.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      default: o += c;
    }
  }
  return o;
}

// ── Minimal, XXE-hardened RDF/XML tree (mirrors cim_io.cpp) ──────────────────

struct XmlNode {
  std::string tag;
  std::map<std::string, std::string> attrs;
  std::string text;
  std::vector<XmlNode> children;

  [[nodiscard]] std::string attr(const std::string& a) const {
    auto it = attrs.find(a);
    return it == attrs.end() ? std::string() : it->second;
  }
};

std::map<std::string, std::string> parse_attrs(const std::string& s) {
  std::map<std::string, std::string> out;
  std::size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    const std::size_t name_start = i;
    while (i < s.size() && s[i] != '=' &&
           !std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    if (i >= s.size() || name_start == i) break;
    const std::string name = s.substr(name_start, i - name_start);
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    if (i >= s.size() || s[i] != '=') break;
    ++i;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    if (i >= s.size()) break;
    const char q = s[i];
    if (q != '"' && q != '\'') break;
    ++i;
    const std::size_t val_start = i;
    while (i < s.size() && s[i] != q) ++i;
    out[name] = s.substr(val_start, i - val_start);
    if (i < s.size()) ++i;
  }
  return out;
}

class XmlParser {
 public:
  explicit XmlParser(const std::string& s) : s_(s) {}

  XmlNode parse() {
    XmlNode root;
    root.tag = "#document";
    while (pos_ < s_.size()) {
      skip_trivia();
      if (pos_ >= s_.size()) break;
      if (s_[pos_] == '<')
        root.children.push_back(parse_element());
      else
        ++pos_;
    }
    return root;
  }

 private:
  const std::string& s_;
  std::size_t pos_{0};

  [[nodiscard]] bool starts_with(const char* p) const {
    return s_.compare(pos_, std::strlen(p), p) == 0;
  }

  void skip_trivia() {
    for (;;) {
      while (pos_ < s_.size() &&
             std::isspace(static_cast<unsigned char>(s_[pos_])))
        ++pos_;
      if (starts_with("<?")) {
        pos_ = s_.find("?>", pos_);
        pos_ = (pos_ == knpos) ? s_.size() : pos_ + 2;
        continue;
      }
      if (starts_with("<!--")) {
        pos_ = s_.find("-->", pos_);
        pos_ = (pos_ == knpos) ? s_.size() : pos_ + 3;
        continue;
      }
      if (starts_with("<!DOCTYPE") || starts_with("<!ENTITY")) {
        throw std::runtime_error(
            "CIM: DOCTYPE/ENTITY declarations are rejected (XXE hardening)");
      }
      break;
    }
  }

  XmlNode parse_element() {
    ++pos_;  // skip '<'
    const std::size_t tag_start = pos_;
    while (pos_ < s_.size() && s_[pos_] != ' ' && s_[pos_] != '\t' &&
           s_[pos_] != '\n' && s_[pos_] != '\r' && s_[pos_] != '>' &&
           s_[pos_] != '/')
      ++pos_;
    XmlNode node;
    node.tag = s_.substr(tag_start, pos_ - tag_start);
    const std::size_t attr_start = pos_;
    while (pos_ < s_.size() && s_[pos_] != '>') ++pos_;
    std::string inside = s_.substr(attr_start, pos_ - attr_start);
    bool self_close = false;
    if (!inside.empty() && inside.back() == '/') {
      self_close = true;
      inside.pop_back();
    }
    node.attrs = parse_attrs(inside);
    if (pos_ < s_.size()) ++pos_;  // skip '>'
    if (self_close) return node;

    std::string text;
    while (pos_ < s_.size()) {
      if (s_[pos_] == '<') {
        if (starts_with("</")) {
          pos_ = s_.find('>', pos_);
          pos_ = (pos_ == knpos) ? s_.size() : pos_ + 1;
          break;
        }
        if (starts_with("<!--")) {
          pos_ = s_.find("-->", pos_);
          pos_ = (pos_ == knpos) ? s_.size() : pos_ + 3;
          continue;
        }
        if (starts_with("<!DOCTYPE") || starts_with("<!ENTITY"))
          throw std::runtime_error("CIM: DOCTYPE/ENTITY rejected (XXE)");
        node.children.push_back(parse_element());
      } else {
        text.push_back(s_[pos_]);
        ++pos_;
      }
    }
    node.text = trim(text);
    return node;
  }
};

// ── Accessors over the parsed tree (mirror cim_io.cpp) ───────────────────────

std::string local_name(const std::string& tag) {
  const auto p = tag.find(':');
  return p == knpos ? tag : tag.substr(p + 1);
}

const XmlNode* child_by_suffix(const XmlNode& n, const std::string& suffix) {
  for (const auto& c : n.children)
    if (ends_with(c.tag, suffix)) return &c;
  return nullptr;
}

std::string prop_text(const XmlNode& n, const std::string& suffix) {
  const auto* c = child_by_suffix(n, suffix);
  return c ? c->text : std::string();
}

double prop_double(const XmlNode& n, const std::string& suffix,
                   double def = 0.0) {
  const auto* c = child_by_suffix(n, suffix);
  if (!c || c->text.empty()) return def;
  try {
    return std::stod(c->text);
  } catch (...) {
    return def;
  }
}

std::string strip_hash(std::string r) {
  if (!r.empty() && r[0] == '#') r = r.substr(1);
  return r;
}

std::string prop_ref(const XmlNode& n, const std::string& suffix) {
  const auto* c = child_by_suffix(n, suffix);
  return c ? strip_hash(c->attr("rdf:resource")) : std::string();
}

std::string object_id(const XmlNode& n) {
  std::string id = n.attr("rdf:ID");
  if (id.empty()) id = strip_hash(n.attr("rdf:about"));
  return id;
}

// ── Cable / line impedance estimate (0.4 kV / 10 kV distribution) ────────────

struct LineImpedance {
  double r_ohm_per_km{0.0};
  double x_ohm_per_km{0.0};
};

// Aluminium if the conductor model code names an Al conductor; otherwise copper.
// Overhead reactance is higher than buried-cable reactance.
LineImpedance estimate_line_impedance(const std::string& model,
                                      double cross_section_mm2) {
  const bool aluminium =
      contains(model, "LGJ") || contains(model, "JKLYJ") ||
      contains(model, "YJLV") || contains(model, "YJLW") ||
      contains(model, "NLYJ") || contains(model, "LJ");
  const bool overhead = contains(model, "LGJ") || contains(model, "JKLYJ");
  // ρ at operating temperature (Ω·mm²/km): Cu ≈ 18.5, Al ≈ 29.4.
  const double rho = aluminium ? 29.4 : 18.5;
  const double area = cross_section_mm2 > 0.0 ? cross_section_mm2 : 16.0;
  LineImpedance z;
  z.r_ohm_per_km = rho / area;
  z.x_ohm_per_km = overhead ? 0.35 : 0.08;
  return z;
}

}  // namespace

// ── Import ───────────────────────────────────────────────────────────────────

CimDistImportResult from_cim_dist(const std::string& xml, ImportMode mode,
                                  const CimDistImportOptions& opts) {
  CimDistImportResult result;
  ImportReport& report = result.report;
  // We reconstruct topology faithfully but ESTIMATE loads and impedances, so the
  // binding level is Canonical and units are best-effort.
  report.binding_level = ImportBindingLevel::Canonical;
  report.unit_assertion = UnitAssertion::BestEffort;

  XmlNode doc;
  try {
    doc = XmlParser(xml).parse();
  } catch (const std::exception& e) {
    report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
               ImportSeverity::Error, "<document>", e.what());
    result.warnings.push_back(std::string("解析失败：") + e.what());
    return result;
  }

  const XmlNode* rdf = nullptr;
  for (const auto& c : doc.children)
    if (local_name(c.tag) == "RDF") rdf = &c;
  if (!rdf) {
    report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
               ImportSeverity::Error, "<document>", "No rdf:RDF root element.");
    result.warnings.push_back("未找到 rdf:RDF 根节点。");
    return result;
  }

  HybridPowerSystem& sys = result.system;
  sys.base_mva = opts.base_mva;
  sys.ac.base_mva = opts.base_mva;
  sys.ac.freq_hz = 50.0;

  // Pass 1: base voltages (nominalVoltage is in VOLTS in this dialect).
  std::map<std::string, double> bv_kv;  // bv id -> kV
  for (const auto& e : rdf->children)
    if (local_name(e.tag) == "BaseVoltage")
      bv_kv[object_id(e)] = prop_double(e, ".nominalVoltage", 0.0) / 1000.0;

  // Pass 2: class + base-voltage index for every object, keyed by rdf:ID.
  std::map<std::string, std::string> equip_class;  // id -> local class name
  std::map<std::string, std::string> equip_bv;     // id -> base-voltage id
  for (const auto& e : rdf->children) {
    const std::string id = object_id(e);
    if (id.empty()) continue;
    equip_class[id] = local_name(e.tag);
    const std::string bv = prop_ref(e, ".BaseVoltage");
    if (!bv.empty()) equip_bv[id] = bv;
  }

  // Pass 3: terminals → equipment-to-node adjacency (ordered by sequenceNumber).
  std::map<std::string, std::vector<std::pair<int, std::string>>> equip_terms;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Terminal") continue;
    const std::string equip = prop_ref(e, ".ConductingEquipment");
    const std::string node = prop_ref(e, ".ConnectivityNode");
    if (equip.empty() || node.empty()) continue;
    const int seq = static_cast<int>(prop_double(e, ".sequenceNumber", 0.0));
    equip_terms[equip].push_back({seq, node});
  }
  for (auto& [equip, terms] : equip_terms)
    std::sort(terms.begin(), terms.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

  // Which classes count as electrically meaningful topology endpoints.  Nodes
  // only touched by structural objects (cable heads, pole sites) are dropped.
  const auto is_topology_equip = [](const std::string& cls) {
    return cls == "ACLineSegment" || cls == "Breaker" ||
           cls == "Disconnector" || cls == "Junction" ||
           cls == "TransformerWinding" || cls == "BusbarSection" ||
           cls == "LVBuilding";
  };

  // Infer a node's nominal voltage from the base voltage of the equipment that
  // terminates there (the transformer primary winding is what makes its node HV).
  std::map<std::string, double> node_kv;
  std::set<std::string> nodes_used;
  for (const auto& [equip, terms] : equip_terms) {
    const auto cit = equip_class.find(equip);
    if (cit == equip_class.end() || !is_topology_equip(cit->second)) continue;
    double kv = 0.0;
    const auto bvit = equip_bv.find(equip);
    if (bvit != equip_bv.end()) {
      const auto kvit = bv_kv.find(bvit->second);
      if (kvit != bv_kv.end()) kv = kvit->second;
    }
    for (const auto& [seq, node] : terms) {
      nodes_used.insert(node);
      if (kv > 0.0) node_kv[node] = std::max(node_kv[node], kv);
    }
  }

  // Pass 4: one AC bus per used ConnectivityNode (deterministic declaration
  // order; sequential indices — the mRIDs overflow int).
  std::map<std::string, int> node_to_bus;
  int next_bus = 0;
  const auto ensure_bus = [&](const std::string& node,
                              const std::string& name) -> int {
    auto it = node_to_bus.find(node);
    if (it != node_to_bus.end()) return it->second;
    ACBus bus;
    bus.index = ++next_bus;
    const auto kvit = node_kv.find(node);
    bus.base_kv = (kvit != node_kv.end() && kvit->second > 0.0) ? kvit->second
                                                                : 0.4;
    bus.name = name.empty() ? ("节点" + std::to_string(bus.index)) : name;
    bus.bus_type = BusType::PQ;
    sys.ac.buses.push_back(bus);
    node_to_bus[node] = bus.index;
    return bus.index;
  };
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "ConnectivityNode") continue;
    const std::string id = object_id(e);
    if (nodes_used.count(id)) ensure_bus(id, prop_text(e, ".name"));
  }
  // Any used-but-undeclared node still gets a bus.
  for (const auto& node : nodes_used) ensure_bus(node, "");

  const auto bus_of = [&](const std::string& node) -> int {
    const auto it = node_to_bus.find(node);
    return it == node_to_bus.end() ? 0 : it->second;
  };
  const auto bus_ptr = [&](int idx) -> ACBus* {
    for (auto& b : sys.ac.buses)
      if (b.index == idx) return &b;
    return nullptr;
  };

  std::map<std::string, std::size_t> accepted;
  std::size_t dropped_lines = 0, dropped_switches = 0, dropped_buildings = 0;

  // Pass 5: transformers (PowerTransformer + primary/secondary windings).
  int ext_idx = 0;
  double sn_total_mva = 0.0;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "PowerTransformer") continue;
    const std::string tid = object_id(e);
    // Find this transformer's windings.
    std::string prim_node, sec_node, prim_bv, sec_bv;
    for (const auto& w : rdf->children) {
      if (local_name(w.tag) != "TransformerWinding") continue;
      if (prop_ref(w, ".MemberOf_PowerTransformer") != tid) continue;
      const std::string wid = object_id(w);
      const std::string wtype = prop_text(w, ".windingType");
      const auto& wt = equip_terms[wid];
      const std::string wnode = wt.empty() ? std::string() : wt.front().second;
      const std::string wbv = equip_bv.count(wid) ? equip_bv[wid] : std::string();
      if (contains(wtype, "primary")) {
        prim_node = wnode;
        prim_bv = wbv;
      } else if (contains(wtype, "secondary")) {
        sec_node = wnode;
        sec_bv = wbv;
      }
    }
    if (prim_node.empty() || sec_node.empty()) {
      report.add(ImportDisposition::Skipped, ImportReasonCode::StructuralLoss,
                 ImportSeverity::Warning, tid,
                 "PowerTransformer missing a primary/secondary winding node.");
      result.warnings.push_back("变压器缺少一次/二次绕组节点，已跳过：" + tid);
      continue;
    }
    Transformer2W t;
    t.index = 1001 + static_cast<int>(sys.ac.transformers_2w.size());
    t.name = prop_text(e, ".name");
    if (t.name.empty()) t.name = prop_text(e, ".model");
    if (t.name.empty()) t.name = "变压器#" + std::to_string(t.index);
    t.hv_bus = bus_of(prim_node);
    t.lv_bus = bus_of(sec_node);
    t.sn_mva = prop_double(e, ".ratedCapacity", 0.0);
    if (t.sn_mva <= 0.0) t.sn_mva = 0.4;  // conservative default
    sn_total_mva += t.sn_mva;
    t.vn_hv_kv = bv_kv.count(prim_bv) ? bv_kv[prim_bv] : 10.0;
    t.vn_lv_kv = bv_kv.count(sec_bv) ? bv_kv[sec_bv] : 0.4;
    t.vk_percent = opts.default_vk_percent;
    t.vkr_percent = opts.default_vkr_percent;
    t.i0_percent = opts.default_i0_percent;
    t.vector_group = "Dyn11";
    t.std_type = prop_text(e, ".model");
    sys.ac.transformers_2w.push_back(t);
    ++accepted["PowerTransformer"];

    // The HV winding node is the incoming 10 kV feed: make it the slack bus and
    // attach a synthetic external grid there.
    if (auto* hv = bus_ptr(t.hv_bus)) {
      hv->bus_type = BusType::SLACK;
      hv->base_kv = t.vn_hv_kv;
      hv->vm_pu = 1.0;
    }
    ExternalGrid grid;
    grid.index = ++ext_idx;
    grid.name = "10kV进线 ExternalGrid";
    grid.bus = t.hv_bus;
    grid.vm_pu = 1.0;
    grid.vn_kv = t.vn_hv_kv;
    grid.s_sc_max_mva = opts.grid_s_sc_mva;
    grid.s_sc_min_mva = opts.grid_s_sc_mva * 0.6;
    grid.rx_max = 0.1;
    grid.rx_min = 0.1;
    grid.r_pu = 0.001;
    grid.x_pu = 0.01;
    grid.controllable = true;
    sys.ac.external_grids.push_back(grid);
  }

  // Pass 6: AC line segments → branches (impedance estimated from geometry).
  // Sub-metre segments (cable heads / joints) become closed switches so the
  // switch-contraction pass merges them and the Y-bus stays non-singular.
  int br_idx = 0;
  int sw_idx = 0;
  std::size_t connectors = 0;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "ACLineSegment") continue;
    const std::string id = object_id(e);
    const auto& terms = equip_terms[id];
    if (terms.size() < 2) {
      ++dropped_lines;
      continue;
    }
    const int from_bus = bus_of(terms[0].second);
    const int to_bus = bus_of(terms[1].second);
    if (from_bus == to_bus) {
      ++dropped_lines;  // degenerate self-loop
      continue;
    }
    const double length_m = prop_double(e, ".length", 0.0);
    if (length_m < opts.min_segment_length_m) {
      Switch sw;
      sw.index = ++sw_idx;
      sw.name = prop_text(e, ".name");
      sw.bus_from = from_bus;
      sw.bus_to = to_bus;
      sw.switch_type = SwitchType::CircuitBreaker;
      sw.closed = true;
      sys.ac.switches.push_back(sw);
      ++connectors;
      continue;
    }
    ACBranch br;
    br.index = ++br_idx;
    br.name = prop_text(e, ".name");
    br.from_bus = from_bus;
    br.to_bus = to_bus;
    const double area = prop_double(e, ".crossSectionArea", 0.0);
    const std::string model = prop_text(e, ".model");
    br.length_km = length_m / 1000.0;
    const LineImpedance z = estimate_line_impedance(model, area);
    br.r_ohm_per_km = z.r_ohm_per_km;
    br.x_ohm_per_km = z.x_ohm_per_km;
    // Per-unit on the system base at this branch's nominal voltage.
    double base_kv = 0.4;
    if (auto* fb = bus_ptr(br.from_bus)) base_kv = fb->base_kv;
    const double zbase = (base_kv * base_kv) / opts.base_mva;  // Ω
    const double r_ohm = z.r_ohm_per_km * br.length_km;
    const double x_ohm = z.x_ohm_per_km * br.length_km;
    br.r_pu = zbase > 0.0 ? r_ohm / zbase : 0.0;
    br.x_pu = zbase > 0.0 ? x_ohm / zbase : 0.0;
    // Coarse thermal rating from cross-section (generous; ~2 A/mm²).
    const double i_amp = area > 0.0 ? 2.0 * area : 100.0;
    br.rate_a_mva = std::sqrt(3.0) * base_kv * i_amp / 1000.0;
    sys.ac.branches.push_back(br);
    ++accepted["ACLineSegment"];
  }

  // Pass 7: breakers / disconnectors → switches (closed = !normalOpen).
  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    if (cls != "Breaker" && cls != "Disconnector" && cls != "Junction")
      continue;
    const std::string id = object_id(e);
    const auto& terms = equip_terms[id];
    if (terms.size() < 2) {
      ++dropped_switches;
      continue;
    }
    Switch sw;
    sw.index = ++sw_idx;
    sw.name = prop_text(e, ".name");
    sw.bus_from = bus_of(terms[0].second);
    sw.bus_to = bus_of(terms[1].second);
    sw.switch_type =
        cls == "Disconnector" ? SwitchType::Disconnector : SwitchType::CircuitBreaker;
    const std::string no = prop_text(e, ".normalOpen");
    sw.closed = !(no == "true" || no == "1");
    sys.ac.switches.push_back(sw);
    ++accepted[cls];
  }

  // Pass 8: meters aggregated per LVBuilding.
  std::map<std::string, int> building_meters;
  int total_meters = 0;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Meter") continue;
    const std::string b = prop_ref(e, ".MemberOf_LVBuilding");
    if (b.empty()) continue;
    ++building_meters[b];
    ++total_meters;
  }

  // Pass 9: LVBuildings → estimated loads.
  struct BuildingRec {
    std::string id;
    std::string name;
    int bus{0};
    int meters{0};
  };
  std::vector<BuildingRec> buildings;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "LVBuilding") continue;
    const std::string id = object_id(e);
    const auto& terms = equip_terms[id];
    if (terms.empty()) {
      ++dropped_buildings;
      continue;
    }
    BuildingRec rec;
    rec.id = id;
    rec.name = prop_text(e, ".name");
    rec.bus = bus_of(terms.front().second);
    rec.meters = building_meters.count(id) ? building_meters[id] : 0;
    buildings.push_back(rec);
  }

  const double pf = std::clamp(opts.power_factor, 0.1, 1.0);
  const double tan_phi = std::tan(std::acos(pf));
  const double total_p_mw = sn_total_mva * opts.load_factor * pf;
  int meter_sum = 0;
  for (const auto& b : buildings) meter_sum += b.meters;
  int load_idx = 0;
  for (const auto& b : buildings) {
    double share = 0.0;
    if (meter_sum > 0)
      share = static_cast<double>(b.meters) / static_cast<double>(meter_sum);
    else if (!buildings.empty())
      share = 1.0 / static_cast<double>(buildings.size());
    Load l;
    l.index = ++load_idx;
    l.bus = b.bus;
    l.name = b.name.empty() ? ("楼栋#" + std::to_string(l.index)) : b.name;
    l.p_mw = total_p_mw * share;
    l.q_mvar = l.p_mw * tan_phi;
    l.n_customers = b.meters;
    l.priority = LoadPriority::Medium;
    sys.ac.loads.push_back(l);
    ++accepted["LVBuilding"];
  }

  // System naming from the Substation.
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Substation") continue;
    const std::string nm = prop_text(e, ".name");
    if (!nm.empty()) {
      sys.name = nm;
      sys.ac.name = nm;
      break;
    }
  }
  if (sys.name.empty() || sys.name == "Hybrid AC/DC System")
    sys.name = "配电台区 CIM 导入";

  // ── Diagnostics ────────────────────────────────────────────────────────────
  for (const auto& [cls, n] : accepted)
    report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
               ImportSeverity::Info, cls,
               "Imported " + std::to_string(n) + " " + cls + " object(s).");
  if (connectors)
    result.warnings.push_back(
        std::to_string(connectors) +
        " 条短线段(<" + std::to_string(opts.min_segment_length_m) +
        "m，电缆头/接头)作为闭合开关处理并被拓扑合并。");
  if (dropped_lines)
    result.warnings.push_back(std::to_string(dropped_lines) +
                              " 条线路缺少两个端子或自环，已跳过。");
  if (dropped_switches)
    result.warnings.push_back(std::to_string(dropped_switches) +
                              " 个开关缺少两个端子，已跳过。");
  if (dropped_buildings)
    result.warnings.push_back(std::to_string(dropped_buildings) +
                              " 个低压楼栋无接入节点，已跳过。");
  report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
             ImportSeverity::Warning, "Load",
             "Loads are ESTIMATED from transformer capacity split by meter count.");
  report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
             ImportSeverity::Warning, "ACLineSegment",
             "Line impedances are ESTIMATED from length + cross-section.");
  result.warnings.push_back(
      "负荷为估算值：变压器容量 × 负载率(" +
      std::to_string(opts.load_factor) + ") × 功率因数(" +
      std::to_string(opts.power_factor) + ")，按各楼栋电表数分摊。");
  result.warnings.push_back(
      "线路阻抗为估算值：由导线截面与长度按典型电缆参数计算。");
  result.warnings.push_back(
      "变压器短路阻抗采用默认值 vk%=" + std::to_string(opts.default_vk_percent) +
      ", vkr%=" + std::to_string(opts.default_vkr_percent) + "。");

  (void)mode;  // acceptance gated by passes_mode() at the call site.
  return result;
}

CimDistImportResult load_cim_dist(const std::filesystem::path& path,
                                  ImportMode mode,
                                  const CimDistImportOptions& opts) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    CimDistImportResult r;
    r.report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
                 ImportSeverity::Error, path.string(), "Cannot open CIM file.");
    r.warnings.push_back("无法打开文件：" + path.string());
    return r;
  }
  const std::string xml((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
  return from_cim_dist(xml, mode, opts);
}

// ── Export ───────────────────────────────────────────────────────────────────

std::string to_cim_dist(const HybridPowerSystem& sys,
                        const CimDistExportOptions& opts) {
  std::ostringstream o;
  o << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  o << "<rdf:RDF xml:base=\"defaultDomain#\""
    << " xmlns:cims=\"" << kCimsNs << "\""
    << " xmlns:cim=\"" << kCimNs << "\""
    << " xmlns:rdf=\"" << kRdfNs << "\""
    << " xmlns:pf=\"" << kPfNs << "\""
    << " xmlns:rdfs=\"" << kRdfsNs << "\">\n";

  const std::string subst_id = "SUBST_1";
  o << " <cim:Substation rdf:ID=\"" << subst_id << "\">\n"
    << "  <cim:Naming.name>" << xml_escape(sys.name) << "</cim:Naming.name>\n"
    << "  <cim:PowerSystemResource.status>1</cim:PowerSystemResource.status>\n"
    << " </cim:Substation>\n";

  // Base voltages (one per distinct bus base_kv), nominalVoltage in VOLTS.
  std::map<long, std::string> kv_to_bv;
  const auto bv_id = [&](double kv) -> std::string {
    const long key = std::lround(kv * 1000.0);
    auto it = kv_to_bv.find(key);
    if (it != kv_to_bv.end()) return it->second;
    const std::string id = "BASEVOL_" + std::to_string(key);
    kv_to_bv[key] = id;
    return id;
  };
  for (const auto& b : sys.ac.buses) (void)bv_id(b.base_kv);
  for (const auto& t : sys.ac.transformers_2w) {
    (void)bv_id(t.vn_hv_kv);
    (void)bv_id(t.vn_lv_kv);
  }
  for (const auto& [key, id] : kv_to_bv) {
    o << " <cim:BaseVoltage rdf:ID=\"" << id << "\">\n"
      << "  <cim:BaseVoltage.nominalVoltage>" << key
      << "</cim:BaseVoltage.nominalVoltage>\n"
      << " </cim:BaseVoltage>\n";
  }

  const auto node_id = [](int bus) { return "NODE_" + std::to_string(bus); };
  const auto base_kv_of = [&](int bus) {
    for (const auto& b : sys.ac.buses)
      if (b.index == bus) return b.base_kv;
    return 0.4;
  };

  // ConnectivityNodes (one per bus).
  for (const auto& b : sys.ac.buses) {
    o << " <cim:ConnectivityNode rdf:ID=\"" << node_id(b.index) << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(b.name) << "</cim:Naming.name>\n"
      << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(b.base_kv) << "\"/>\n"
      << " </cim:ConnectivityNode>\n";
  }

  const auto emit_terminal = [&](const std::string& tid,
                                 const std::string& equip, int seq, int bus) {
    o << " <cim:Terminal rdf:ID=\"" << tid << "\">\n"
      << "  <cim:Terminal.sequenceNumber>" << seq
      << "</cim:Terminal.sequenceNumber>\n"
      << "  <cim:Terminal.ConductingEquipment rdf:resource=\"#" << equip
      << "\"/>\n"
      << "  <cim:Terminal.ConnectivityNode rdf:resource=\"#" << node_id(bus)
      << "\"/>\n"
      << " </cim:Terminal>\n";
  };

  // PowerTransformers + windings.
  for (const auto& t : sys.ac.transformers_2w) {
    const std::string tid = "TRANS_" + std::to_string(t.index);
    o << " <cim:PowerTransformer rdf:ID=\"" << tid << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(t.name) << "</cim:Naming.name>\n";
    if (!t.std_type.empty())
      o << "  <cim:Equipment.model>" << xml_escape(t.std_type)
        << "</cim:Equipment.model>\n";
    o << "  <cim:PowerTransformer.ratedCapacity>" << t.sn_mva
      << "</cim:PowerTransformer.ratedCapacity>\n"
      << "  <cim:Equipment.MemberOf_EquipmentContainer rdf:resource=\"#"
      << subst_id << "\"/>\n"
      << " </cim:PowerTransformer>\n";
    const std::string wp = "WINDING_" + std::to_string(t.index) + "_P";
    const std::string ws = "WINDING_" + std::to_string(t.index) + "_S";
    o << " <cim:TransformerWinding rdf:ID=\"" << wp << "\">\n"
      << "  <cim:TransformerWinding.ratedMVA>" << t.sn_mva
      << "</cim:TransformerWinding.ratedMVA>\n"
      << "  <cim:TransformerWinding.windingType>primary"
      << "</cim:TransformerWinding.windingType>\n"
      << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(t.vn_hv_kv) << "\"/>\n"
      << "  <cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource=\"#"
      << tid << "\"/>\n"
      << " </cim:TransformerWinding>\n";
    emit_terminal(wp + "_T1", wp, 1, t.hv_bus);
    o << " <cim:TransformerWinding rdf:ID=\"" << ws << "\">\n"
      << "  <cim:TransformerWinding.ratedMVA>" << t.sn_mva
      << "</cim:TransformerWinding.ratedMVA>\n"
      << "  <cim:TransformerWinding.windingType>secondary"
      << "</cim:TransformerWinding.windingType>\n"
      << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(t.vn_lv_kv) << "\"/>\n"
      << "  <cim:TransformerWinding.MemberOf_PowerTransformer rdf:resource=\"#"
      << tid << "\"/>\n"
      << " </cim:TransformerWinding>\n";
    emit_terminal(ws + "_T1", ws, 1, t.lv_bus);
  }

  // ACLineSegments (recover cross-section from the copper resistance estimate).
  for (const auto& br : sys.ac.branches) {
    const std::string id = "SEG_" + std::to_string(br.index);
    const double base_kv = base_kv_of(br.from_bus);
    double length_km = br.length_km;
    double r_ohm_per_km = br.r_ohm_per_km;
    if (length_km <= 0.0 && br.r_pu > 0.0) {
      // Fall back to a nominal 100 m if geometry was never populated.
      length_km = 0.1;
      const double zbase = (base_kv * base_kv) / sys.ac.base_mva;
      r_ohm_per_km = zbase > 0.0 ? (br.r_pu * zbase) / length_km : 0.0;
    }
    const double area = r_ohm_per_km > 0.0 ? 18.5 / r_ohm_per_km : 0.0;
    o << " <cim:ACLineSegment rdf:ID=\"" << id << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(br.name) << "</cim:Naming.name>\n"
      << "  <cim:Conductor.length>" << (length_km * 1000.0)
      << "</cim:Conductor.length>\n";
    if (area > 0.0)
      o << "  <cim:Conductor.crossSectionArea>" << std::lround(area)
        << "</cim:Conductor.crossSectionArea>\n";
    o << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(base_kv) << "\"/>\n"
      << " </cim:ACLineSegment>\n";
    emit_terminal(id + "_T1", id, 1, br.from_bus);
    emit_terminal(id + "_T2", id, 2, br.to_bus);
  }

  // Breakers / disconnectors from switches.
  for (const auto& sw : sys.ac.switches) {
    const std::string id = "SWITCH_" + std::to_string(sw.index);
    const char* cls =
        sw.switch_type == SwitchType::Disconnector ? "Disconnector" : "Breaker";
    o << " <cim:" << cls << " rdf:ID=\"" << id << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(sw.name) << "</cim:Naming.name>\n"
      << "  <cim:Switch.normalOpen>" << (sw.closed ? "false" : "true")
      << "</cim:Switch.normalOpen>\n"
      << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(base_kv_of(sw.bus_from)) << "\"/>\n"
      << " </cim:" << cls << ">\n";
    emit_terminal(id + "_T1", id, 1, sw.bus_from);
    emit_terminal(id + "_T2", id, 2, sw.bus_to);
  }
  // Circuit breakers likewise (modelled as CIM Breaker).
  int cb_seq = static_cast<int>(sys.ac.switches.size());
  for (const auto& cb : sys.ac.circuit_breakers) {
    const std::string id = "SWITCH_CB_" + std::to_string(cb.index);
    o << " <cim:Breaker rdf:ID=\"" << id << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(cb.name) << "</cim:Naming.name>\n"
      << "  <cim:Switch.normalOpen>" << (cb.closed ? "false" : "true")
      << "</cim:Switch.normalOpen>\n"
      << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(base_kv_of(cb.bus_from)) << "\"/>\n"
      << " </cim:Breaker>\n";
    emit_terminal(id + "_T1", id, 1, cb.bus_from);
    emit_terminal(id + "_T2", id, 2, cb.bus_to);
    (void)cb_seq;
  }

  // LVBuildings (one per load) + generic meters per customer.
  for (std::size_t i = 0; i < sys.ac.loads.size(); ++i) {
    const auto& l = sys.ac.loads[i];
    const std::string bid = "LVBUILDING_" + std::to_string(l.index);
    o << " <cim:LVBuilding rdf:ID=\"" << bid << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(l.name) << "</cim:Naming.name>\n"
      << "  <cim:Equipment.MemberOf_EquipmentContainer rdf:resource=\"#"
      << subst_id << "\"/>\n"
      << " </cim:LVBuilding>\n";
    emit_terminal(bid + "_T1", bid, 1, l.bus);
    if (opts.emit_meters) {
      for (int m = 0; m < l.n_customers; ++m) {
        const std::string mid =
            "METER_" + std::to_string(l.index) + "_" + std::to_string(m + 1);
        o << " <cim:Meter rdf:ID=\"" << mid << "\">\n"
          << "  <cim:Meter.MemberOf_LVBuilding rdf:resource=\"#" << bid
          << "\"/>\n"
          << " </cim:Meter>\n";
      }
    }
  }

  o << "</rdf:RDF>\n";
  (void)opts.model_name;
  return o.str();
}

void save_cim_dist(const HybridPowerSystem& sys,
                   const std::filesystem::path& path,
                   const CimDistExportOptions& opts) {
  std::ofstream os(path, std::ios::binary);
  if (!os)
    throw std::runtime_error("CIM(dist): cannot open for write " +
                             path.string());
  os << to_cim_dist(sys, opts);
}

}  // namespace hacdcpf::io
