#include "hacdcpf/io/cim_dist_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <numeric>
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

PhaseMask cim_phase_mask(const std::string& text,
                         PhaseMask fallback = PhaseMask::abc()) {
  if (text.empty()) return fallback;
  std::uint8_t bits = 0;
  if (contains(text, "A") || contains(text, "a") || contains(text, "A相"))
    bits |= PhaseMask::a().bits;
  if (contains(text, "B") || contains(text, "b") || contains(text, "B相"))
    bits |= PhaseMask::b().bits;
  if (contains(text, "C") || contains(text, "c") || contains(text, "C相"))
    bits |= PhaseMask::c().bits;
  return bits == 0 ? fallback : PhaseMask(bits);
}

bool is_load_class(const std::string& cls) {
  return cls == "EnergyConsumer" || cls == "ConformLoad" ||
         cls == "NonConformLoad" || cls == "StationSupply";
}

bool is_generator_class(const std::string& cls) {
  return cls == "SynchronousMachine" || cls == "GeneratingUnit" ||
         cls == "EnergySource" || cls == "PowerElectronicsConnection" ||
         cls == "PhotovoltaicUnit" || cls == "SolarGeneratingUnit" ||
         cls == "WindGeneratingUnit";
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

double prop_double_any(const XmlNode& n,
                       std::initializer_list<const char*> suffixes,
                       double def = 0.0) {
  for (const char* suffix : suffixes) {
    const auto* child = child_by_suffix(n, suffix);
    if (!child || child->text.empty()) continue;
    try {
      return std::stod(child->text);
    } catch (...) {
    }
  }
  return def;
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

void serialize_xml_node(std::ostringstream& out, const XmlNode& node) {
  out << '<' << node.tag;
  for (const auto& [name, value] : node.attrs)
    out << ' ' << name << "=\"" << value << "\"";
  if (node.children.empty() && node.text.empty()) {
    out << "/>\n";
    return;
  }
  out << '>';
  if (!node.text.empty()) out << node.text;
  if (!node.children.empty()) out << '\n';
  for (const auto& child : node.children) serialize_xml_node(out, child);
  out << "</" << node.tag << ">\n";
}

struct StitchedXml {
  std::string xml;
  std::size_t duplicate_objects{0};
  std::vector<std::string> warnings;
};

StitchedXml stitch_xml_documents(const std::vector<std::string>& documents) {
  StitchedXml stitched;
  XmlNode merged;
  merged.tag = "rdf:RDF";
  std::map<std::string, std::size_t> object_positions;

  for (std::size_t document_index = 0; document_index < documents.size();
       ++document_index) {
    XmlNode doc = XmlParser(documents[document_index]).parse();
    const XmlNode* rdf = nullptr;
    for (const auto& child : doc.children)
      if (local_name(child.tag) == "RDF") rdf = &child;
    if (!rdf) {
      throw std::runtime_error("CIM document " +
                               std::to_string(document_index + 1) +
                               " has no rdf:RDF root element");
    }

    for (const auto& object : rdf->children) {
      const std::string id = object_id(object);
      if (id.empty()) {
        merged.children.push_back(object);
        continue;
      }
      auto [position_it, inserted] =
          object_positions.emplace(id, merged.children.size());
      if (inserted) {
        merged.children.push_back(object);
        continue;
      }

      ++stitched.duplicate_objects;
      XmlNode& existing = merged.children[position_it->second];
      if (local_name(existing.tag) != local_name(object.tag)) {
        stitched.warnings.push_back(
            "重复 rdf:ID 的类型不一致，保留首次声明：" + id);
        continue;
      }

      for (const auto& child : object.children) {
        const std::string resource = child.attr("rdf:resource");
        auto same = std::find_if(
            existing.children.begin(), existing.children.end(),
            [&](const XmlNode& current) {
              if (current.tag != child.tag) return false;
              const std::string current_resource = current.attr("rdf:resource");
              return resource.empty() ? current_resource.empty()
                                      : current_resource == resource;
            });
        if (same == existing.children.end()) {
          existing.children.push_back(child);
        } else if (same->text.empty() && !child.text.empty()) {
          *same = child;
        } else if (!child.text.empty() && same->text != child.text) {
          stitched.warnings.push_back(
              "重复对象属性冲突，保留首次声明：" + id + "/" + child.tag);
        }
      }
    }
  }

  std::ostringstream out;
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<rdf:RDF>\n";
  for (const auto& child : merged.children) serialize_xml_node(out, child);
  out << "</rdf:RDF>\n";
  stitched.xml = out.str();
  return stitched;
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
  const double area = cross_section_mm2 > 0.0 ? cross_section_mm2 : 50.0;
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
  std::map<std::string, std::string> psr_type_name;
  for (const auto& e : rdf->children)
    if (local_name(e.tag) == "BaseVoltage")
      bv_kv[object_id(e)] = prop_double(e, ".nominalVoltage", 0.0) / 1000.0;
    else if (local_name(e.tag) == "PSRType")
      psr_type_name[object_id(e)] = prop_text(e, ".name");

  // Pass 2: class + base-voltage index for every object, keyed by rdf:ID.
  std::map<std::string, std::string> equip_class;  // id -> local class name
  std::map<std::string, std::string> equip_bv;     // id -> base-voltage id
  std::map<std::string, std::string> equip_psr_type;
  std::map<std::string, PhaseMask> equip_phase;
  for (const auto& e : rdf->children) {
    const std::string id = object_id(e);
    if (id.empty()) continue;
    const std::string cls = local_name(e.tag);
    equip_class[id] = cls;
    const std::string bv = prop_ref(e, ".BaseVoltage");
    if (!bv.empty()) equip_bv[id] = bv;
    const std::string psr_type = prop_ref(e, ".PSRType");
    if (!psr_type.empty()) equip_psr_type[id] = psr_type;
    const std::string phases = prop_text(e, ".phases");
    if (!phases.empty()) {
      equip_phase[id] = cim_phase_mask(phases);
      result.has_explicit_phase_data = true;
    }
    if (is_load_class(cls)) ++result.source_load_objects;
    if (is_generator_class(cls)) ++result.source_generator_objects;
  }

  // Pass 3: terminals → equipment-to-node adjacency (ordered by sequenceNumber).
  std::map<std::string, std::vector<std::pair<int, std::string>>> equip_terms;
  std::size_t synthetic_winding_nodes = 0;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Terminal") continue;
    const std::string equip = prop_ref(e, ".ConductingEquipment");
    std::string node = prop_ref(e, ".ConnectivityNode");
    if (equip.empty()) continue;
    if (node.empty()) {
      const auto class_it = equip_class.find(equip);
      if (class_it == equip_class.end() ||
          class_it->second != "TransformerWinding")
        continue;
      // Feeder exports often omit the unloaded LV-side ConnectivityNode. Keep
      // the transformer by giving that dangling winding its own synthetic bus.
      node = "__CIM_SYNTHETIC_NODE_" + equip;
      ++synthetic_winding_nodes;
    }
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
           cls == "Disconnector" || cls == "LoadBreakSwitch" ||
           cls == "Fuse" || cls == "Switch" || cls == "Junction" ||
           cls == "TransformerWinding" || cls == "BusbarSection" ||
           cls == "LVBuilding" || is_load_class(cls) ||
           is_generator_class(cls);
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
  std::map<std::string, std::size_t> transformer_position;
  std::map<int, PhaseMask> branch_phase;
  std::map<int, PhaseMask> switch_phase;
  struct PhaseWeights {
    double a{0.0};
    double b{0.0};
    double c{0.0};
  };
  std::map<int, PhaseWeights> load_phase_weights;
  std::map<int, PhaseMask> generator_phase;

  // Pass 5: transformers (PowerTransformer + primary/secondary windings).
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
    transformer_position[tid] = sys.ac.transformers_2w.size() - 1;
    ++accepted["PowerTransformer"];

  }

  // Pass 6: AC line segments → branches (impedance estimated from geometry).
  // Sub-metre segments (cable heads / joints) become closed switches so the
  // switch-contraction pass merges them and the Y-bus stays non-singular.
  int br_idx = 0;
  int sw_idx = 0;
  std::size_t connectors = 0;
  std::size_t inferred_overhead_conductors = 0;
  std::size_t inferred_cable_conductors = 0;
  std::size_t inferred_other_conductors = 0;
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
      switch_phase[sw.index] =
          equip_phase.count(id) ? equip_phase[id] : PhaseMask::abc();
      ++connectors;
      continue;
    }
    ACBranch br;
    br.index = ++br_idx;
    br.name = prop_text(e, ".name");
    br.from_bus = from_bus;
    br.to_bus = to_bus;
    const double source_area = prop_double(e, ".crossSectionArea", 0.0);
    const std::string model = prop_text(e, ".model");
    const std::string psr_id = equip_psr_type.count(id)
                                   ? equip_psr_type[id]
                                   : std::string();
    const std::string psr_name = psr_type_name.count(psr_id)
                                     ? psr_type_name[psr_id]
                                     : std::string();
    br.length_km = length_m / 1000.0;
    double base_kv = 0.4;
    if (auto* fb = bus_ptr(br.from_bus)) base_kv = fb->base_kv;
    double effective_area = source_area;
    std::string impedance_model = model;
    if (effective_area <= 0.0) {
      if (contains(psr_name, "架空")) {
        effective_area = opts.default_lv_overhead_cross_section_mm2;
        impedance_model += " JKLYJ";  // aluminium overhead fallback
        ++inferred_overhead_conductors;
      } else if (contains(psr_name, "电缆")) {
        effective_area = opts.default_lv_cable_cross_section_mm2;
        ++inferred_cable_conductors;
      } else {
        effective_area = base_kv > 1.0
                             ? opts.default_mv_cross_section_mm2
                             : opts.default_lv_overhead_cross_section_mm2;
        ++inferred_other_conductors;
      }
    }
    br.conductor_model = model;
    br.cross_section_mm2 = effective_area;
    br.cross_section_inferred = source_area <= 0.0;
    br.line_type = psr_name;
    br.parameters_inferred = source_area <= 0.0 || model.empty();
    br.parameter_source = br.parameters_inferred
                              ? "cim_geometry_estimate"
                              : "cim_model_cross_section_estimate";
    const LineImpedance z =
        estimate_line_impedance(impedance_model, effective_area);
    br.r_ohm_per_km = z.r_ohm_per_km;
    br.x_ohm_per_km = z.x_ohm_per_km;
    // Per-unit on the system base at this branch's nominal voltage.
    const double zbase = (base_kv * base_kv) / opts.base_mva;  // Ω
    const double r_ohm = z.r_ohm_per_km * br.length_km;
    const double x_ohm = z.x_ohm_per_km * br.length_km;
    br.r_pu = zbase > 0.0 ? r_ohm / zbase : 0.0;
    br.x_pu = zbase > 0.0 ? x_ohm / zbase : 0.0;
    // Coarse thermal rating from cross-section (generous; ~2 A/mm²).
    const double i_amp = 2.0 * effective_area;
    br.rate_a_mva = std::sqrt(3.0) * base_kv * i_amp / 1000.0;
    sys.ac.branches.push_back(br);
    branch_phase[br.index] =
        equip_phase.count(id) ? equip_phase[id] : PhaseMask::abc();
    ++accepted["ACLineSegment"];
  }

  // Pass 7: switching/protection equipment.  Preserve the CIM device class;
  // reliability restoration must not treat a fuse or disconnector as a tie CB.
  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    if (cls != "Breaker" && cls != "Disconnector" &&
        cls != "LoadBreakSwitch" && cls != "Fuse" && cls != "Recloser" &&
        cls != "Sectionalizer" && cls != "Switch" && cls != "Junction")
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
    if (cls == "Breaker") sw.switch_type = SwitchType::CircuitBreaker;
    else if (cls == "Disconnector" || cls == "Junction")
      sw.switch_type = SwitchType::Disconnector;
    else if (cls == "LoadBreakSwitch") sw.switch_type = SwitchType::LoadBreakSwitch;
    else if (cls == "Fuse") sw.switch_type = SwitchType::Fuse;
    else if (cls == "Recloser") sw.switch_type = SwitchType::Recloser;
    else if (cls == "Sectionalizer") sw.switch_type = SwitchType::Sectionalizer;
    else sw.switch_type = SwitchType::Unknown;
    const std::string no = prop_text(e, ".normalOpen");
    sw.closed = !(no == "true" || no == "1");
    sw.normal_closed = sw.closed;
    sw.normal_state_explicit = true;

    const bool is_ring = [&]() {
      const std::string value = prop_text(e, ".isRing");
      return value == "true" || value == "1";
    }();
    switch (sw.switch_type) {
      case SwitchType::CircuitBreaker:
      case SwitchType::Fuse:
      case SwitchType::Recloser:
        sw.role = SwitchRole::Protection;
        break;
      case SwitchType::LoadBreakSwitch:
        // normalOpen is a state, not proof of a transfer function.  Only an
        // explicit CIM ring indication authorizes restoration tie closing.
        sw.role = is_ring ? SwitchRole::Tie : SwitchRole::Sectionalizing;
        break;
      case SwitchType::Disconnector:
        sw.role = SwitchRole::Isolation;
        break;
      case SwitchType::Sectionalizer:
        sw.role = SwitchRole::Sectionalizing;
        break;
      case SwitchType::Unknown:
        sw.role = SwitchRole::Unspecified;
        break;
    }
    sw.capabilities = effective_switch_capabilities(sw);
    sw.capabilities_explicit = true;
    sys.ac.switches.push_back(sw);
    switch_phase[sw.index] =
        equip_phase.count(id) ? equip_phase[id] : PhaseMask::abc();
    ++accepted[cls];
  }

  // Pass 7b: add one synthetic source at each CIM-declared feeder origin. A
  // transformer-area export commonly omits the upstream MV feeder entirely;
  // each transformer whose HV bus has no MV connection then needs its own
  // source. Do not de-duplicate those sources through an LV tie line.
  if (opts.auto_add_external_grid && !sys.ac.buses.empty()) {
    std::map<int, std::size_t> bus_position;
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i)
      bus_position[sys.ac.buses[i].index] = i;
    std::vector<std::size_t> parent(sys.ac.buses.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto find_root = [&](std::size_t start) {
      std::size_t root = start;
      while (parent[root] != root) root = parent[root];
      while (parent[start] != start) {
        const std::size_t next = parent[start];
        parent[start] = root;
        start = next;
      }
      return root;
    };
    const auto join_buses = [&](int a, int b) {
      const auto ai = bus_position.find(a), bi = bus_position.find(b);
      if (ai == bus_position.end() || bi == bus_position.end()) return;
      const std::size_t ar = find_root(ai->second), br = find_root(bi->second);
      if (ar != br) parent[br] = ar;
    };
    for (const auto& branch : sys.ac.branches)
      if (branch.in_service) join_buses(branch.from_bus, branch.to_bus);
    for (const auto& transformer : sys.ac.transformers_2w)
      if (transformer.in_service)
        join_buses(transformer.hv_bus, transformer.lv_bus);
    // Source placement follows the physical feeder boundary. Open switches
    // must not make a de-energized section look like an independent grid that
    // needs its own synthetic source.
    for (const auto& sw : sys.ac.switches)
      join_buses(sw.bus_from, sw.bus_to);

    std::set<int> source_buses;
    std::set<std::size_t> explicitly_sourced_components;
    for (const auto& object : rdf->children) {
      if (local_name(object.tag) != "Circuit") continue;
      const std::string breaker = prop_ref(object, ".SourceBreaker");
      const auto terms_it = equip_terms.find(breaker);
      if (breaker.empty() || terms_it == equip_terms.end() ||
          terms_it->second.empty())
        continue;
      const int source_bus = bus_of(terms_it->second.front().second);
      if (source_bus != 0) source_buses.insert(source_bus);
      // Both sides belong to this declared feeder origin even if the source
      // breaker is open; do not synthesize another source downstream.
      for (const auto& [sequence, node] : terms_it->second) {
        const auto position = bus_position.find(bus_of(node));
        if (position != bus_position.end())
          explicitly_sourced_components.insert(find_root(position->second));
      }
    }

    const auto has_upstream_mv_connection = [&](int hv_bus) {
      const ACBus* hv = bus_ptr(hv_bus);
      if (!hv) return false;
      const auto is_mv_peer = [&](int bus) {
        const ACBus* peer = bus_ptr(bus);
        return peer && peer->base_kv >= 0.9 * hv->base_kv;
      };
      for (const auto& branch : sys.ac.branches) {
        if (!branch.in_service) continue;
        if (branch.from_bus == hv_bus && is_mv_peer(branch.to_bus)) return true;
        if (branch.to_bus == hv_bus && is_mv_peer(branch.from_bus)) return true;
      }
      for (const auto& sw : sys.ac.switches) {
        if (!sw.closed) continue;
        if (sw.bus_from == hv_bus && is_mv_peer(sw.bus_to)) return true;
        if (sw.bus_to == hv_bus && is_mv_peer(sw.bus_from)) return true;
      }
      return false;
    };

    std::set<std::size_t> fallback_components;
    std::set<std::size_t> components_with_transformer_source;
    for (const auto& transformer : sys.ac.transformers_2w) {
      const auto position = bus_position.find(transformer.hv_bus);
      if (position == bus_position.end()) continue;
      const std::size_t root = find_root(position->second);
      if (explicitly_sourced_components.count(root) ||
          has_upstream_mv_connection(transformer.hv_bus))
        continue;
      source_buses.insert(transformer.hv_bus);
      components_with_transformer_source.insert(root);
    }

    // If the source-less model does contain an MV feeder, one transformer HV
    // bus remains the fallback origin for that connected MV network.
    for (const auto& transformer : sys.ac.transformers_2w) {
      const auto position = bus_position.find(transformer.hv_bus);
      if (position == bus_position.end()) continue;
      const std::size_t root = find_root(position->second);
      if (!explicitly_sourced_components.count(root) &&
          !components_with_transformer_source.count(root) &&
          fallback_components.insert(root).second)
        source_buses.insert(transformer.hv_bus);
    }

    int ext_idx = 0;
    for (const int source_bus : source_buses) {
      ACBus* source = bus_ptr(source_bus);
      if (!source) continue;
      source->bus_type = BusType::SLACK;
      source->vm_pu = 1.0;
      ExternalGrid grid;
      grid.index = ++ext_idx;
      grid.name = "CIM自动外部电网#" + std::to_string(ext_idx);
      grid.bus = source_bus;
      grid.vm_pu = 1.0;
      grid.vn_kv = source->base_kv;
      grid.s_sc_max_mva = opts.grid_s_sc_mva;
      grid.s_sc_min_mva = opts.grid_s_sc_mva * 0.6;
      grid.rx_max = 0.1;
      grid.rx_min = 0.1;
      grid.r_pu = 0.001;
      grid.x_pu = 0.01;
      grid.controllable = true;
      sys.ac.external_grids.push_back(grid);
    }
    if (!source_buses.empty()) {
      report.add(ImportDisposition::Coerced, ImportReasonCode::MissingRequired,
                 ImportSeverity::Warning, "ExternalGrid",
                 "Added " + std::to_string(source_buses.size()) +
                     " synthetic external grid source(s).");
      result.warnings.push_back(
          "源模型缺失，已自动添加 " + std::to_string(source_buses.size()) +
          " 个外部电网（优先馈线首端；无上游中压连接的配变在高压侧独立补源）。");
    }
  }

  const auto equal_phase_weights = [](PhaseMask mask) {
    PhaseWeights weights;
    const int count = std::max(1, mask.count());
    if (mask.has(0)) weights.a = 1.0 / count;
    if (mask.has(1)) weights.b = 1.0 / count;
    if (mask.has(2)) weights.c = 1.0 / count;
    return weights;
  };

  // Pass 8: meters aggregated per LVBuilding, including their actual phase.
  std::map<std::string, int> building_meters;
  std::map<std::string, PhaseWeights> building_meter_phases;
  int total_meters = 0;
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Meter") continue;
    const std::string b = prop_ref(e, ".MemberOf_LVBuilding");
    if (b.empty()) continue;
    ++building_meters[b];
    ++total_meters;
    const std::string phase_text = prop_text(e, ".phases");
    if (phase_text.empty()) continue;
    const PhaseMask phase = cim_phase_mask(phase_text);
    auto& weights = building_meter_phases[b];
    if (phase.has(0)) weights.a += 1.0;
    if (phase.has(1)) weights.b += 1.0;
    if (phase.has(2)) weights.c += 1.0;
  }

  // Pass 9a: explicit CIM load and generator objects. These take precedence
  // over inferred transformer demand when present.
  int load_idx = 0;
  int generator_idx = 0;
  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    if (!is_load_class(cls) && !is_generator_class(cls)) continue;
    const std::string id = object_id(e);
    const auto terms_it = equip_terms.find(id);
    if (terms_it == equip_terms.end() || terms_it->second.empty()) {
      report.add(ImportDisposition::Skipped, ImportReasonCode::StructuralLoss,
                 ImportSeverity::Warning, id,
                 cls + " has no connected Terminal.");
      continue;
    }
    const int bus = bus_of(terms_it->second.front().second);
    const PhaseMask phase = equip_phase.count(id) ? equip_phase[id]
                                                  : PhaseMask::abc();
    if (is_load_class(cls)) {
      Load load;
      load.index = ++load_idx;
      load.bus = bus;
      load.name = prop_text(e, ".name");
      if (load.name.empty()) load.name = cls + "#" + std::to_string(load.index);
      load.p_mw = prop_double_any(
          e, {".p", ".pfixed", ".pFixed", ".activePower"}, 0.0);
      load.q_mvar = prop_double_any(
          e, {".q", ".qfixed", ".qFixed", ".reactivePower"}, 0.0);
      sys.ac.loads.push_back(load);
      load_phase_weights[load.index] = equal_phase_weights(phase);
      ++accepted[cls];
      continue;
    }

    Generator generator;
    generator.index = ++generator_idx;
    generator.bus = bus;
    generator.name = prop_text(e, ".name");
    if (generator.name.empty())
      generator.name = cls + "#" + std::to_string(generator.index);
    generator.pg_mw = prop_double_any(
        e, {".p", ".activePower", ".ratedNetMaxP"}, 0.0);
    generator.qg_mvar =
        prop_double_any(e, {".q", ".reactivePower"}, 0.0);
    generator.pmax_mw = prop_double_any(
        e, {".maxP", ".maxOperatingP", ".ratedNetMaxP"}, generator.pg_mw);
    generator.pmin_mw =
        prop_double_any(e, {".minP", ".minOperatingP"}, 0.0);
    generator.qmax_mvar =
        prop_double_any(e, {".maxQ", ".maxOperatingQ"}, generator.qg_mvar);
    generator.qmin_mvar =
        prop_double_any(e, {".minQ", ".minOperatingQ"}, generator.qg_mvar);
    generator.mbase_mva =
        prop_double_any(e, {".ratedS", ".ratedApparentPower"}, 0.0);
    sys.ac.generators.push_back(generator);
    generator_phase[generator.index] = phase;
    ++accepted[cls];
  }

  // Pass 9b: LVBuildings -> estimated loads. Demand is allocated within each
  // transformer district, not globally across unrelated folder documents.
  struct BuildingRec {
    std::string id;
    std::string name;
    std::string transformer_id;
    int bus{0};
    int meters{0};
    PhaseWeights meter_phases;
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
    rec.transformer_id = prop_ref(e, ".MemberOf_PowerTransformer");
    rec.bus = bus_of(terms.front().second);
    rec.meters = building_meters.count(id) ? building_meters[id] : 0;
    if (building_meter_phases.count(id))
      rec.meter_phases = building_meter_phases[id];
    buildings.push_back(rec);
  }

  const double pf = std::clamp(opts.power_factor, 0.1, 1.0);
  const double tan_phi = std::tan(std::acos(pf));
  if (result.source_load_objects == 0) {
    std::map<std::string, std::vector<std::size_t>> transformer_buildings;
    for (std::size_t i = 0; i < buildings.size(); ++i)
      if (!buildings[i].transformer_id.empty())
        transformer_buildings[buildings[i].transformer_id].push_back(i);

    std::vector<std::size_t> orphan_buildings;
    for (std::size_t i = 0; i < buildings.size(); ++i)
      if (buildings[i].transformer_id.empty() ||
          !transformer_position.count(buildings[i].transformer_id))
        orphan_buildings.push_back(i);

    std::set<std::size_t> consumed_buildings;
    double unallocated_transformer_mva = 0.0;
    for (const auto& [transformer_id, transformer_pos] : transformer_position) {
      const auto grouped = transformer_buildings.find(transformer_id);
      if (grouped == transformer_buildings.end() || grouped->second.empty()) {
        const auto& transformer = sys.ac.transformers_2w[transformer_pos];
        if (!orphan_buildings.empty()) {
          unallocated_transformer_mva += transformer.sn_mva;
          continue;
        }
        Load load;
        load.index = ++load_idx;
        load.bus = transformer.lv_bus;
        load.name = transformer.name + " 估算负荷";
        load.p_mw = transformer.sn_mva * opts.load_factor * pf;
        load.q_mvar = load.p_mw * tan_phi;
        load.priority = LoadPriority::Medium;
        sys.ac.loads.push_back(load);
        load_phase_weights[load.index] = equal_phase_weights(PhaseMask::abc());
        ++result.inferred_load_objects;
        continue;
      }

      int group_meter_sum = 0;
      for (const std::size_t building_pos : grouped->second)
        group_meter_sum += buildings[building_pos].meters;
      const double transformer_p =
          sys.ac.transformers_2w[transformer_pos].sn_mva * opts.load_factor * pf;
      for (const std::size_t building_pos : grouped->second) {
        const auto& building = buildings[building_pos];
        consumed_buildings.insert(building_pos);
        const double share = group_meter_sum > 0
                                 ? static_cast<double>(building.meters) /
                                       static_cast<double>(group_meter_sum)
                                 : 1.0 / static_cast<double>(grouped->second.size());
        Load load;
        load.index = ++load_idx;
        load.bus = building.bus;
        load.name = building.name.empty()
                        ? ("楼栋#" + std::to_string(load.index))
                        : building.name;
        load.p_mw = transformer_p * share;
        load.q_mvar = load.p_mw * tan_phi;
        load.n_customers = building.meters;
        load.priority = LoadPriority::Medium;
        sys.ac.loads.push_back(load);
        const double phase_total = building.meter_phases.a +
                                   building.meter_phases.b +
                                   building.meter_phases.c;
        load_phase_weights[load.index] =
            phase_total > 0.0
                ? PhaseWeights{building.meter_phases.a / phase_total,
                               building.meter_phases.b / phase_total,
                               building.meter_phases.c / phase_total}
                : equal_phase_weights(PhaseMask::abc());
        ++accepted["LVBuilding"];
        ++result.inferred_load_objects;
      }
    }

    // Preserve orphan buildings even when their transformer reference is
    // absent or unresolved. They share the otherwise unallocated capacity.
    if (!orphan_buildings.empty()) {
      int orphan_meter_sum = 0;
      for (const auto pos : orphan_buildings)
        orphan_meter_sum += buildings[pos].meters;
      const double orphan_total_p =
          unallocated_transformer_mva * opts.load_factor * pf;
      for (const auto pos : orphan_buildings) {
        const auto& building = buildings[pos];
        const double share = orphan_meter_sum > 0
                                 ? static_cast<double>(building.meters) /
                                       static_cast<double>(orphan_meter_sum)
                                 : 1.0 / static_cast<double>(orphan_buildings.size());
        Load load;
        load.index = ++load_idx;
        load.bus = building.bus;
        load.name = building.name.empty()
                        ? ("楼栋#" + std::to_string(load.index))
                        : building.name;
        load.p_mw = orphan_total_p * share;
        load.q_mvar = load.p_mw * tan_phi;
        load.n_customers = building.meters;
        sys.ac.loads.push_back(load);
        const double phase_total = building.meter_phases.a +
                                   building.meter_phases.b +
                                   building.meter_phases.c;
        load_phase_weights[load.index] =
            phase_total > 0.0
                ? PhaseWeights{building.meter_phases.a / phase_total,
                               building.meter_phases.b / phase_total,
                               building.meter_phases.c / phase_total}
                : equal_phase_weights(PhaseMask::abc());
        ++accepted["LVBuilding"];
        ++result.inferred_load_objects;
      }
    }
  }

  // Pass 10: preserve the phase-domain facts as an executable three-phase
  // model. The positive-sequence AC model remains available for existing
  // workflows, while single-phase meters/lines are no longer flattened away.
  ThreePhaseACSystem three_phase;
  three_phase.base_mva = sys.ac.base_mva;
  three_phase.base_freq_hz = sys.ac.freq_hz;
  three_phase.name = sys.ac.name;
  for (const auto& bus : sys.ac.buses) {
    ThreePhaseACBus phase_bus;
    phase_bus.index = bus.index;
    phase_bus.bus_type = bus.bus_type;
    phase_bus.name = bus.name;
    phase_bus.base_kv = bus.base_kv;
    phase_bus.in_service = bus.in_service;
    phase_bus.phase_mask = PhaseMask::abc();
    phase_bus.vm_a_pu = bus.vm_pu;
    phase_bus.vm_b_pu = bus.vm_pu;
    phase_bus.vm_c_pu = bus.vm_pu;
    phase_bus.vmin_pu = bus.vmin_pu;
    phase_bus.vmax_pu = bus.vmax_pu;
    three_phase.buses.push_back(phase_bus);
  }
  for (const auto& branch : sys.ac.branches) {
    ThreePhaseACLine line;
    line.index = branch.index;
    line.from_bus = branch.from_bus;
    line.to_bus = branch.to_bus;
    line.name = branch.name;
    line.in_service = branch.in_service;
    line.phase_mask = branch_phase.count(branch.index)
                          ? branch_phase[branch.index]
                          : PhaseMask::abc();
    line.length_km = branch.length_km;
    line.r1_ohm_per_km = branch.r_ohm_per_km;
    line.x1_ohm_per_km = branch.x_ohm_per_km;
    line.r0_ohm_per_km = 3.0 * branch.r_ohm_per_km;
    line.x0_ohm_per_km = 3.0 * branch.x_ohm_per_km;
    line.r1_pu = branch.r_pu;
    line.x1_pu = branch.x_pu;
    line.b1_pu = branch.b_pu;
    line.r0_pu = 3.0 * branch.r_pu;
    line.x0_pu = 3.0 * branch.x_pu;
    line.rate_a_mva = branch.rate_a_mva;
    three_phase.lines.push_back(line);
    if (line.phase_mask.bits != PhaseMask::abc().bits)
      result.is_unbalanced = true;
  }
  int phase_line_index = static_cast<int>(three_phase.lines.size());
  for (const auto& sw : sys.ac.switches) {
    if (!sw.closed) continue;
    ThreePhaseACLine line;
    line.index = ++phase_line_index;
    line.from_bus = sw.bus_from;
    line.to_bus = sw.bus_to;
    line.name = sw.name.empty() ? "CIM闭合开关" : sw.name;
    line.phase_mask = switch_phase.count(sw.index)
                          ? switch_phase[sw.index]
                          : PhaseMask::abc();
    line.r1_pu = 1e-6;
    line.x1_pu = 1e-6;
    line.r0_pu = 3e-6;
    line.x0_pu = 3e-6;
    three_phase.lines.push_back(line);
    if (line.phase_mask.bits != PhaseMask::abc().bits)
      result.is_unbalanced = true;
  }
  for (const auto& transformer : sys.ac.transformers_2w) {
    ThreePhaseTransformer phase_transformer;
    phase_transformer.index = transformer.index;
    phase_transformer.name = transformer.name;
    phase_transformer.hv_bus = transformer.hv_bus;
    phase_transformer.lv_bus = transformer.lv_bus;
    phase_transformer.in_service = transformer.in_service;
    phase_transformer.sn_mva = transformer.sn_mva;
    phase_transformer.vn_hv_kv = transformer.vn_hv_kv;
    phase_transformer.vn_lv_kv = transformer.vn_lv_kv;
    phase_transformer.vk_percent = transformer.vk_percent;
    phase_transformer.vkr_percent = transformer.vkr_percent;
    phase_transformer.i0_percent = transformer.i0_percent;
    phase_transformer.vector_group = transformer.vector_group;
    phase_transformer.shift_deg = transformer.shift_deg;
    three_phase.transformers.push_back(phase_transformer);
  }
  double phase_p_a = 0.0, phase_p_b = 0.0, phase_p_c = 0.0;
  for (const auto& load : sys.ac.loads) {
    const PhaseWeights weights = load_phase_weights.count(load.index)
                                     ? load_phase_weights[load.index]
                                     : equal_phase_weights(PhaseMask::abc());
    ThreePhaseLoad phase_load;
    phase_load.index = load.index;
    phase_load.bus = load.bus;
    phase_load.name = load.name;
    phase_load.in_service = load.in_service;
    std::uint8_t bits = 0;
    if (weights.a > 0.0) bits |= PhaseMask::a().bits;
    if (weights.b > 0.0) bits |= PhaseMask::b().bits;
    if (weights.c > 0.0) bits |= PhaseMask::c().bits;
    phase_load.phase_mask = bits == 0 ? PhaseMask::abc() : PhaseMask(bits);
    phase_load.p_a_mw = load.p_mw * weights.a;
    phase_load.q_a_mvar = load.q_mvar * weights.a;
    phase_load.p_b_mw = load.p_mw * weights.b;
    phase_load.q_b_mvar = load.q_mvar * weights.b;
    phase_load.p_c_mw = load.p_mw * weights.c;
    phase_load.q_c_mvar = load.q_mvar * weights.c;
    phase_p_a += phase_load.p_a_mw;
    phase_p_b += phase_load.p_b_mw;
    phase_p_c += phase_load.p_c_mw;
    if (phase_load.phase_mask.bits != PhaseMask::abc().bits)
      result.is_unbalanced = true;
    three_phase.loads.push_back(phase_load);
  }
  for (const auto& generator : sys.ac.generators) {
    ThreePhaseGenerator phase_generator;
    phase_generator.index = generator.index;
    phase_generator.bus = generator.bus;
    phase_generator.name = generator.name;
    phase_generator.in_service = generator.in_service;
    phase_generator.is_slack = generator.is_slack;
    phase_generator.phase_mask = generator_phase.count(generator.index)
                                     ? generator_phase[generator.index]
                                     : PhaseMask::abc();
    const int active_phases = std::max(1, phase_generator.phase_mask.count());
    const double p_per_phase = generator.pg_mw / active_phases;
    const double q_per_phase = generator.qg_mvar / active_phases;
    if (phase_generator.phase_mask.has(0)) {
      phase_generator.p_a_mw = p_per_phase;
      phase_generator.q_a_mvar = q_per_phase;
    }
    if (phase_generator.phase_mask.has(1)) {
      phase_generator.p_b_mw = p_per_phase;
      phase_generator.q_b_mvar = q_per_phase;
    }
    if (phase_generator.phase_mask.has(2)) {
      phase_generator.p_c_mw = p_per_phase;
      phase_generator.q_c_mvar = q_per_phase;
    }
    phase_generator.p_mw = generator.pg_mw;
    phase_generator.q_mvar = generator.qg_mvar;
    phase_generator.vm_pu = generator.vg_pu;
    phase_generator.pmax_mw = generator.pmax_mw;
    phase_generator.pmin_mw = generator.pmin_mw;
    phase_generator.qmax_mvar = generator.qmax_mvar;
    phase_generator.qmin_mvar = generator.qmin_mvar;
    phase_generator.mbase_mva = generator.mbase_mva;
    three_phase.generators.push_back(phase_generator);
    if (phase_generator.phase_mask.bits != PhaseMask::abc().bits)
      result.is_unbalanced = true;
  }
  for (const auto& grid : sys.ac.external_grids) {
    ThreePhaseExternalGrid phase_grid;
    phase_grid.index = grid.index;
    phase_grid.bus = grid.bus;
    phase_grid.name = grid.name;
    phase_grid.in_service = grid.in_service;
    phase_grid.vm_pu = grid.vm_pu;
    phase_grid.va_deg = grid.va_deg;
    phase_grid.s_sc_max_mva = grid.s_sc_max_mva;
    phase_grid.s_sc_min_mva = grid.s_sc_min_mva;
    phase_grid.rx_max = grid.rx_max;
    phase_grid.rx_min = grid.rx_min;
    phase_grid.r1_pu = grid.r_pu;
    phase_grid.x1_pu = grid.x_pu;
    phase_grid.r2_pu = grid.r_pu;
    phase_grid.x2_pu = grid.x_pu;
    phase_grid.r0_pu = grid.r0_pu > 0.0 ? grid.r0_pu : 3.0 * grid.r_pu;
    phase_grid.x0_pu = grid.x0_pu > 0.0 ? grid.x0_pu : 3.0 * grid.x_pu;
    three_phase.external_grids.push_back(phase_grid);
  }
  const double phase_scale = std::max({1e-9, std::abs(phase_p_a),
                                       std::abs(phase_p_b), std::abs(phase_p_c)});
  if (std::max({phase_p_a, phase_p_b, phase_p_c}) -
          std::min({phase_p_a, phase_p_b, phase_p_c}) >
      1e-6 * phase_scale)
    result.is_unbalanced = true;
  sys.three_phase_ac = std::move(three_phase);

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
  if (sys.ac.name.empty()) sys.ac.name = sys.name;
  if (sys.three_phase_ac) sys.three_phase_ac->name = sys.name + " 三相模型";

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
  const std::size_t inferred_conductors = inferred_overhead_conductors +
                                          inferred_cable_conductors +
                                          inferred_other_conductors;
  if (inferred_conductors) {
    report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
               ImportSeverity::Warning, "ACLineSegment",
               "Inferred conductor cross-section for " +
                   std::to_string(inferred_conductors) + " line segment(s).");
    result.warnings.push_back(
        std::to_string(inferred_conductors) +
        " 条线路缺少导线截面，已按 PSRType 使用典型截面：低压架空线 " +
        std::to_string(opts.default_lv_overhead_cross_section_mm2) +
        " mm²、低压站内电缆 " +
        std::to_string(opts.default_lv_cable_cross_section_mm2) +
        " mm²、中压线路 " +
        std::to_string(opts.default_mv_cross_section_mm2) + " mm²。分类计数：架空 " +
        std::to_string(inferred_overhead_conductors) + "、电缆 " +
        std::to_string(inferred_cable_conductors) + "、其他 " +
        std::to_string(inferred_other_conductors) + "。");
  }
  if (synthetic_winding_nodes) {
    report.add(ImportDisposition::Coerced,
               ImportReasonCode::MissingRequired, ImportSeverity::Warning,
               "TransformerWinding",
               "Created " + std::to_string(synthetic_winding_nodes) +
                   " synthetic buses for winding terminals with no "
                   "ConnectivityNode.");
    result.warnings.push_back(
        std::to_string(synthetic_winding_nodes) +
        " 个变压器绕组端子缺少 ConnectivityNode，已生成独立低压母线以保留变压器。");
  }
  if (result.inferred_load_objects > 0) {
    report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
               ImportSeverity::Warning, "Load",
               "Created " + std::to_string(result.inferred_load_objects) +
                   " estimated load(s) from transformer capacity and meter "
                   "counts.");
    result.warnings.push_back(
        "源文件没有负荷功率值，已生成 " +
        std::to_string(result.inferred_load_objects) +
        " 个估算负荷：变压器容量 × 负载率(" +
        std::to_string(opts.load_factor) + ") × 功率因数(" +
        std::to_string(opts.power_factor) +
        ")；有楼栋时按本台配变电表数分摊，否则挂到配变低压侧。");
  }
  if (result.source_generator_objects == 0) {
    report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
               ImportSeverity::Info, "Generator",
               "The source CIM document contains no generator object.");
    result.warnings.push_back(
        "源 CIM 中未声明发电设备；外部电网是为潮流可解而生成的电源等值，不伪装成发电机。");
  }
  if (result.has_explicit_phase_data) {
    result.warnings.push_back(
        std::string("已读取 ConductingEquipment.phases 并生成三相 abc 模型；判定：") +
        (result.is_unbalanced ? "存在三相不平衡。"
                              : "已提供的相别字段中未发现不平衡。"));
  } else {
    result.warnings.push_back(
        "源 CIM 未提供相别字段；已按 ABC 平衡假设生成三相等值模型，不能据此认定原系统平衡。");
  }
  report.add(ImportDisposition::Coerced, ImportReasonCode::UnitInferred,
             ImportSeverity::Warning, "ACLineSegment",
             "Line impedances are ESTIMATED from length + cross-section.");
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

CimDistImportResult from_cim_dist(
    const std::vector<std::string>& xml_documents, ImportMode mode,
    const CimDistImportOptions& opts) {
  if (xml_documents.empty()) {
    CimDistImportResult result;
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::MissingRequired,
                      ImportSeverity::Error, "<documents>",
                      "No CIM XML documents were provided.");
    result.warnings.push_back("未提供 CIM XML 文件。");
    return result;
  }
  try {
    StitchedXml stitched = stitch_xml_documents(xml_documents);
    CimDistImportResult result = from_cim_dist(stitched.xml, mode, opts);
    result.warnings.insert(result.warnings.begin(), stitched.warnings.begin(),
                           stitched.warnings.end());
    result.report.add(
        ImportDisposition::Accepted, ImportReasonCode::Ok,
        ImportSeverity::Info, "<documents>",
        "Stitched " + std::to_string(xml_documents.size()) +
            " CIM document(s); merged " +
            std::to_string(stitched.duplicate_objects) +
            " repeated rdf:ID declaration(s).");
    if (xml_documents.size() > 1)
      result.warnings.insert(
          result.warnings.begin(),
          "已拼接 " + std::to_string(xml_documents.size()) +
              " 个 CIM XML 文件，按 rdf:ID 合并 " +
              std::to_string(stitched.duplicate_objects) +
              " 条重复对象声明及跨文件引用。");
    return result;
  } catch (const std::exception& e) {
    CimDistImportResult result;
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::ParseError, ImportSeverity::Error,
                      "<documents>", e.what());
    result.warnings.push_back(std::string("CIM 多文件拼接失败：") + e.what());
    return result;
  }
}

CimDistImportResult load_cim_dist(
    const std::vector<std::filesystem::path>& paths, ImportMode mode,
    const CimDistImportOptions& opts) {
  std::vector<std::string> documents;
  documents.reserve(paths.size());
  for (const auto& path : paths) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
      CimDistImportResult result;
      result.report.add(ImportDisposition::Rejected,
                        ImportReasonCode::ParseError, ImportSeverity::Error,
                        path.string(), "Cannot open CIM file.");
      result.warnings.push_back("无法打开文件：" + path.string());
      return result;
    }
    documents.emplace_back(std::istreambuf_iterator<char>(ifs),
                           std::istreambuf_iterator<char>());
  }
  return from_cim_dist(documents, mode, opts);
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
    const double area = br.cross_section_mm2 > 0.0
                            ? br.cross_section_mm2
                            : (r_ohm_per_km > 0.0 ? 18.5 / r_ohm_per_km : 0.0);
    o << " <cim:ACLineSegment rdf:ID=\"" << id << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(br.name) << "</cim:Naming.name>\n"
      << "  <cim:Conductor.length>" << (length_km * 1000.0)
      << "</cim:Conductor.length>\n";
    if (!br.conductor_model.empty())
      o << "  <cim:Equipment.model>" << xml_escape(br.conductor_model)
        << "</cim:Equipment.model>\n";
    if (area > 0.0)
      o << "  <cim:Conductor.crossSectionArea>" << std::lround(area)
        << "</cim:Conductor.crossSectionArea>\n";
    o << "  <cim:PowerSystemResource.BaseVoltage rdf:resource=\"#"
      << bv_id(base_kv) << "\"/>\n"
      << " </cim:ACLineSegment>\n";
    emit_terminal(id + "_T1", id, 1, br.from_bus);
    emit_terminal(id + "_T2", id, 2, br.to_bus);
  }

  // Switching and protection devices retain their CIM class on round-trip.
  for (const auto& sw : sys.ac.switches) {
    const std::string id = "SWITCH_" + std::to_string(sw.index);
    const char* cls = "Switch";
    switch (sw.switch_type) {
      case SwitchType::CircuitBreaker: cls = "Breaker"; break;
      case SwitchType::Disconnector: cls = "Disconnector"; break;
      case SwitchType::LoadBreakSwitch: cls = "LoadBreakSwitch"; break;
      case SwitchType::Fuse: cls = "Fuse"; break;
      case SwitchType::Recloser: cls = "Recloser"; break;
      case SwitchType::Sectionalizer: cls = "Sectionalizer"; break;
      case SwitchType::Unknown: cls = "Switch"; break;
    }
    o << " <cim:" << cls << " rdf:ID=\"" << id << "\">\n"
      << "  <cim:Naming.name>" << xml_escape(sw.name) << "</cim:Naming.name>\n"
      << "  <cim:Switch.normalOpen>"
      << (effective_switch_normal_closed(sw) ? "false" : "true")
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
