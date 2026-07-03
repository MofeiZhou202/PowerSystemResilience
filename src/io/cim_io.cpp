#include "hacdcpf/io/cim_io.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/// Bounded IEC 61970 CGMES 3.0 (EQ + SSH) import/export.
/// See docs/cim_cgmes3_crosswalk.md for the class/attribute crosswalk and the
/// documented out-of-scope ceiling (transformers, DC, converters, TP/SV,
/// dynamics).  The RDF/XML reader is self-contained and XXE-hardened.

namespace hacdcpf::io {
namespace {

constexpr std::size_t knpos = std::string::npos;
const std::string kCimNs = "http://iec.ch/TC57/CIM100#";
const std::string kRdfNs = "http://www.w3.org/1999/02/22-rdf-syntax-ns#";

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

// ── Minimal, XXE-hardened RDF/XML tree ───────────────────────────────────────

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

// ── Accessors over the parsed tree ───────────────────────────────────────────

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

// If an id looks like "_prefixNNN", return NNN, else -1.
int index_suffix(const std::string& id) {
  std::size_t i = id.size();
  while (i > 0 && std::isdigit(static_cast<unsigned char>(id[i - 1]))) --i;
  if (i == id.size()) return -1;
  try {
    return std::stoi(id.substr(i));
  } catch (...) {
    return -1;
  }
}

}  // namespace

// ── Export ───────────────────────────────────────────────────────────────────

std::string to_cim(const HybridPowerSystem& sys, const CimExportOptions& opts) {
  std::ostringstream o;
  o << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  o << "<rdf:RDF xmlns:rdf=\"" << kRdfNs << "\" xmlns:cim=\"" << kCimNs
    << "\">\n";
  o << "  <!-- HACDCPF CGMES 3.0 EQ" << (opts.include_ssh ? "+SSH" : "")
    << " bounded export: " << xml_escape(opts.model_name) << " -->\n";

  // Base voltages (one per distinct base_kv).
  std::map<long, int> kv_to_bv;
  auto bv_id = [&](double kv) {
    const long key = std::lround(kv * 1000.0);
    auto it = kv_to_bv.find(key);
    if (it != kv_to_bv.end()) return it->second;
    const int id = static_cast<int>(kv_to_bv.size()) + 1;
    kv_to_bv[key] = id;
    return id;
  };
  for (const auto& b : sys.ac.buses) (void)bv_id(b.base_kv);
  for (const auto& [key, id] : kv_to_bv) {
    o << "  <cim:BaseVoltage rdf:ID=\"_bv" << id << "\">\n"
      << "    <cim:BaseVoltage.nominalVoltage>"
      << (static_cast<double>(key) / 1000.0)
      << "</cim:BaseVoltage.nominalVoltage>\n"
      << "  </cim:BaseVoltage>\n";
  }

  // TopologicalNodes (one per AC bus).
  for (const auto& b : sys.ac.buses) {
    o << "  <cim:TopologicalNode rdf:ID=\"_bus" << b.index << "\">\n"
      << "    <cim:IdentifiedObject.name>" << xml_escape(b.name)
      << "</cim:IdentifiedObject.name>\n"
      << "    <cim:TopologicalNode.BaseVoltage rdf:resource=\"#_bv"
      << bv_id(b.base_kv) << "\"/>\n"
      << "  </cim:TopologicalNode>\n";
  }

  auto emit_terminal = [&](const std::string& tid, const std::string& equip,
                           int node) {
    o << "  <cim:Terminal rdf:ID=\"" << tid << "\">\n"
      << "    <cim:Terminal.ConductingEquipment rdf:resource=\"#" << equip
      << "\"/>\n"
      << "    <cim:Terminal.TopologicalNode rdf:resource=\"#_bus" << node
      << "\"/>\n"
      << "  </cim:Terminal>\n";
  };

  // ACLineSegments (one per AC branch).
  for (const auto& br : sys.ac.branches) {
    const std::string id = "_line" + std::to_string(br.index);
    o << "  <cim:ACLineSegment rdf:ID=\"" << id << "\">\n"
      << "    <cim:IdentifiedObject.name>" << xml_escape(br.name)
      << "</cim:IdentifiedObject.name>\n"
      << "    <cim:ACLineSegment.r>" << br.r_pu << "</cim:ACLineSegment.r>\n"
      << "    <cim:ACLineSegment.x>" << br.x_pu << "</cim:ACLineSegment.x>\n"
      << "    <cim:ACLineSegment.bch>" << br.b_pu << "</cim:ACLineSegment.bch>\n"
      << "  </cim:ACLineSegment>\n";
    emit_terminal(id + "_T1", id, br.from_bus);
    emit_terminal(id + "_T2", id, br.to_bus);
  }

  // SynchronousMachines (one per generator).
  for (const auto& g : sys.ac.generators) {
    const std::string id = "_gen" + std::to_string(g.index);
    o << "  <cim:SynchronousMachine rdf:ID=\"" << id << "\">\n"
      << "    <cim:IdentifiedObject.name>" << xml_escape(g.name)
      << "</cim:IdentifiedObject.name>\n"
      << "    <cim:SynchronousMachine.minP>" << g.pmin_mw
      << "</cim:SynchronousMachine.minP>\n"
      << "    <cim:SynchronousMachine.maxP>" << g.pmax_mw
      << "</cim:SynchronousMachine.maxP>\n"
      << "    <cim:SynchronousMachine.minQ>" << g.qmin_mvar
      << "</cim:SynchronousMachine.minQ>\n"
      << "    <cim:SynchronousMachine.maxQ>" << g.qmax_mvar
      << "</cim:SynchronousMachine.maxQ>\n"
      << "    <cim:RotatingMachine.ratedS>" << g.mbase_mva
      << "</cim:RotatingMachine.ratedS>\n";
    if (opts.include_ssh) {
      o << "    <cim:RotatingMachine.p>" << g.pg_mw
        << "</cim:RotatingMachine.p>\n";
    }
    o << "  </cim:SynchronousMachine>\n";
    emit_terminal(id + "_T1", id, g.bus);
  }

  // EnergyConsumers (one per load).
  for (const auto& l : sys.ac.loads) {
    const std::string id = "_load" + std::to_string(l.index);
    o << "  <cim:EnergyConsumer rdf:ID=\"" << id << "\">\n"
      << "    <cim:IdentifiedObject.name>" << xml_escape(l.name)
      << "</cim:IdentifiedObject.name>\n";
    if (opts.include_ssh) {
      o << "    <cim:EnergyConsumer.p>" << l.p_mw << "</cim:EnergyConsumer.p>\n"
        << "    <cim:EnergyConsumer.q>" << l.q_mvar
        << "</cim:EnergyConsumer.q>\n";
    }
    o << "  </cim:EnergyConsumer>\n";
    emit_terminal(id + "_T1", id, l.bus);
  }

  o << "</rdf:RDF>\n";
  return o.str();
}

void save_cim(const HybridPowerSystem& sys, const std::filesystem::path& path,
              const CimExportOptions& opts) {
  std::ofstream os(path);
  if (!os) throw std::runtime_error("CIM: cannot open for write " +
                                    path.string());
  os << to_cim(sys, opts);
}

// ── Import ───────────────────────────────────────────────────────────────────

CimImportResult from_cim(const std::string& xml, ImportMode mode) {
  CimImportResult result;
  ImportReport& report = result.report;
  report.binding_level = ImportBindingLevel::Rich;
  report.unit_assertion = UnitAssertion::Asserted;  // CGMES is SI/per-unit typed

  XmlNode doc;
  try {
    doc = XmlParser(xml).parse();
  } catch (const std::exception& e) {
    report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
               ImportSeverity::Error, "<document>", e.what());
    return result;
  }

  // Locate the rdf:RDF root.
  const XmlNode* rdf = nullptr;
  for (const auto& c : doc.children)
    if (local_name(c.tag) == "RDF") rdf = &c;
  if (!rdf) {
    report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
               ImportSeverity::Error, "<document>", "No rdf:RDF root element.");
    return result;
  }

  HybridPowerSystem& sys = result.system;

  // Pass 1: base voltages and topological nodes → AC buses.
  std::map<std::string, double> bv_kv;         // bv id -> nominal kV
  std::map<std::string, int> node_to_bus;      // node id -> bus index
  std::map<std::string, std::string> node_bv;  // node id -> bv id
  int next_bus = 0;
  std::map<std::string, std::size_t> class_counts;
  std::map<std::string, std::size_t> unsupported_counts;

  const auto note_class = [&](const std::string& cls) { ++class_counts[cls]; };

  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    if (cls == "BaseVoltage") {
      bv_kv[object_id(e)] = prop_double(e, ".nominalVoltage", 0.0);
      note_class(cls);
    }
  }
  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    if (cls == "TopologicalNode" || cls == "ConnectivityNode") {
      const std::string id = object_id(e);
      ACBus bus;
      const int suf = index_suffix(id);
      bus.index = suf >= 0 ? suf : (++next_bus);
      if (suf > next_bus) next_bus = suf;
      bus.name = prop_text(e, ".name");
      node_bv[id] = prop_ref(e, ".BaseVoltage");
      node_to_bus[id] = bus.index;
      sys.ac.buses.push_back(bus);
      note_class(cls);
    }
  }
  // Fill bus base_kv from its base voltage reference.
  for (auto& bus : sys.ac.buses) {
    for (const auto& [nid, idx] : node_to_bus) {
      if (idx != bus.index) continue;
      const auto bvit = bv_kv.find(node_bv[nid]);
      if (bvit != bv_kv.end() && bvit->second > 0.0) bus.base_kv = bvit->second;
    }
  }

  // Pass 2: terminals → equipment-to-node adjacency.
  std::map<std::string, std::vector<std::string>> equip_nodes;  // equip -> node ids
  for (const auto& e : rdf->children) {
    if (local_name(e.tag) != "Terminal") continue;
    const std::string equip = prop_ref(e, ".ConductingEquipment");
    const std::string node = prop_ref(e, ".TopologicalNode");
    if (!equip.empty() && !node.empty()) equip_nodes[equip].push_back(node);
  }

  const auto bus_of = [&](const std::string& node) -> int {
    const auto it = node_to_bus.find(node);
    return it == node_to_bus.end() ? 0 : it->second;
  };

  // Pass 3: equipment.
  for (const auto& e : rdf->children) {
    const std::string cls = local_name(e.tag);
    const std::string id = object_id(e);
    if (cls == "ACLineSegment") {
      ACBranch br;
      const int suf = index_suffix(id);
      br.index = suf >= 0 ? suf : static_cast<int>(sys.ac.branches.size()) + 1;
      br.name = prop_text(e, ".name");
      br.r_pu = prop_double(e, ".r");
      br.x_pu = prop_double(e, ".x");
      br.b_pu = prop_double(e, ".bch");
      const auto& nodes = equip_nodes[id];
      if (nodes.size() >= 2) {
        br.from_bus = bus_of(nodes[0]);
        br.to_bus = bus_of(nodes[1]);
      } else {
        report.add(ImportDisposition::Coerced, ImportReasonCode::UnresolvedBusRef,
                   ImportSeverity::Warning, id,
                   "ACLineSegment has fewer than two terminals; endpoints "
                   "defaulted.");
      }
      sys.ac.branches.push_back(br);
      note_class(cls);
    } else if (cls == "SynchronousMachine") {
      Generator g;
      const int suf = index_suffix(id);
      g.index = suf >= 0 ? suf : static_cast<int>(sys.ac.generators.size()) + 1;
      g.name = prop_text(e, ".name");
      g.pmin_mw = prop_double(e, ".minP");
      g.pmax_mw = prop_double(e, ".maxP");
      g.qmin_mvar = prop_double(e, ".minQ");
      g.qmax_mvar = prop_double(e, ".maxQ");
      g.mbase_mva = prop_double(e, ".ratedS");
      g.pg_mw = prop_double(e, ".p", 0.0);
      const auto& nodes = equip_nodes[id];
      if (!nodes.empty()) g.bus = bus_of(nodes[0]);
      sys.ac.generators.push_back(g);
      note_class(cls);
    } else if (cls == "EnergyConsumer") {
      Load l;
      const int suf = index_suffix(id);
      l.index = suf >= 0 ? suf : static_cast<int>(sys.ac.loads.size()) + 1;
      l.name = prop_text(e, ".name");
      l.p_mw = prop_double(e, ".p");
      l.q_mvar = prop_double(e, ".q");
      const auto& nodes = equip_nodes[id];
      if (!nodes.empty()) l.bus = bus_of(nodes[0]);
      sys.ac.loads.push_back(l);
      note_class(cls);
    } else if (cls == "BaseVoltage" || cls == "TopologicalNode" ||
               cls == "ConnectivityNode" || cls == "Terminal") {
      // handled above
    } else {
      // Out-of-scope class (transformer, converter, DC, dynamics, ...).
      ++unsupported_counts[cls];
    }
  }

  // Summarize what was imported and what was structurally dropped.
  for (const auto& [cls, n] : class_counts) {
    report.add(ImportDisposition::Accepted, ImportReasonCode::Ok,
               ImportSeverity::Info, cls,
               "Imported " + std::to_string(n) + " " + cls + " object(s).");
  }
  for (const auto& [cls, n] : unsupported_counts) {
    report.add(ImportDisposition::Skipped, ImportReasonCode::StructuralLoss,
               ImportSeverity::Warning, cls,
               "Unsupported CGMES class dropped (out of bounded EQ+SSH scope): " +
                   std::to_string(n) + " " + cls + " object(s).");
  }

  sys.name = "CIM import";
  (void)mode;  // mode gates acceptance via passes_mode() at the call site.
  return result;
}

CimImportResult load_cim(const std::filesystem::path& path, ImportMode mode) {
  std::ifstream ifs(path);
  if (!ifs) {
    CimImportResult r;
    r.report.add(ImportDisposition::Rejected, ImportReasonCode::ParseError,
                 ImportSeverity::Error, path.string(), "Cannot open CIM file.");
    return r;
  }
  const std::string xml((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
  return from_cim(xml, mode);
}

}  // namespace hacdcpf::io
