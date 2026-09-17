#include "hacdcpf/io/svg_distribution_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hacdcpf/model/enum_strings.hpp"

namespace hacdcpf::io {
namespace {

constexpr std::size_t knpos = std::string::npos;
constexpr double kPi = 3.14159265358979323846;

std::string path_utf8(const std::filesystem::path& path) {
  const auto utf8 = path.u8string();
  return std::string(utf8.begin(), utf8.end());
}

std::string trim(const std::string& value) {
  std::size_t first = 0;
  std::size_t last = value.size();
  while (first < last &&
         std::isspace(static_cast<unsigned char>(value[first])))
    ++first;
  while (last > first &&
         std::isspace(static_cast<unsigned char>(value[last - 1])))
    --last;
  return value.substr(first, last - first);
}

std::string xml_decode(std::string value) {
  const std::pair<const char*, const char*> entities[] = {
      {"&amp;", "&"}, {"&lt;", "<"}, {"&gt;", ">"},
      {"&quot;", "\""}, {"&apos;", "'"}};
  for (const auto& [encoded, decoded] : entities) {
    std::size_t pos = 0;
    while ((pos = value.find(encoded, pos)) != knpos) {
      value.replace(pos, std::strlen(encoded), decoded);
      pos += std::strlen(decoded);
    }
  }
  return value;
}

struct XmlNode {
  std::string tag;
  std::map<std::string, std::string> attrs;
  std::string text;
  std::vector<XmlNode> children;

  [[nodiscard]] std::string attr(const std::string& name) const {
    const auto it = attrs.find(name);
    return it == attrs.end() ? std::string{} : it->second;
  }
};

std::map<std::string, std::string> parse_attrs(const std::string& source) {
  std::map<std::string, std::string> attrs;
  std::size_t pos = 0;
  while (pos < source.size()) {
    while (pos < source.size() &&
           std::isspace(static_cast<unsigned char>(source[pos])))
      ++pos;
    const std::size_t name_begin = pos;
    while (pos < source.size() && source[pos] != '=' &&
           !std::isspace(static_cast<unsigned char>(source[pos])))
      ++pos;
    if (name_begin == pos) break;
    const std::string name = source.substr(name_begin, pos - name_begin);
    while (pos < source.size() &&
           std::isspace(static_cast<unsigned char>(source[pos])))
      ++pos;
    if (pos >= source.size() || source[pos] != '=') break;
    ++pos;
    while (pos < source.size() &&
           std::isspace(static_cast<unsigned char>(source[pos])))
      ++pos;
    if (pos >= source.size() || (source[pos] != '\"' && source[pos] != '\''))
      break;
    const char quote = source[pos++];
    const std::size_t value_begin = pos;
    while (pos < source.size() && source[pos] != quote) ++pos;
    attrs[name] = xml_decode(source.substr(value_begin, pos - value_begin));
    if (pos < source.size()) ++pos;
  }
  return attrs;
}

class XmlParser {
 public:
  explicit XmlParser(const std::string& source) : source_(source) {}

  XmlNode parse() {
    if (source_.find("<!DOCTYPE") != knpos ||
        source_.find("<!ENTITY") != knpos) {
      throw std::runtime_error(
          "SVG distribution: DOCTYPE/ENTITY declarations are rejected");
    }
    XmlNode document;
    document.tag = "#document";
    while (pos_ < source_.size()) {
      skip_trivia();
      if (pos_ >= source_.size()) break;
      if (source_[pos_] == '<' && !starts_with("</"))
        document.children.push_back(parse_element());
      else
        ++pos_;
    }
    return document;
  }

 private:
  const std::string& source_;
  std::size_t pos_{0};

  [[nodiscard]] bool starts_with(const char* value) const {
    return source_.compare(pos_, std::strlen(value), value) == 0;
  }

  void skip_trivia() {
    for (;;) {
      while (pos_ < source_.size() &&
             std::isspace(static_cast<unsigned char>(source_[pos_])))
        ++pos_;
      if (starts_with("<?")) {
        const auto end = source_.find("?>", pos_ + 2);
        pos_ = end == knpos ? source_.size() : end + 2;
      } else if (starts_with("<!--")) {
        const auto end = source_.find("-->", pos_ + 4);
        pos_ = end == knpos ? source_.size() : end + 3;
      } else {
        break;
      }
    }
  }

  XmlNode parse_element() {
    if (source_[pos_] != '<')
      throw std::runtime_error("SVG distribution: malformed XML element");
    ++pos_;
    const std::size_t tag_begin = pos_;
    while (pos_ < source_.size() &&
           !std::isspace(static_cast<unsigned char>(source_[pos_])) &&
           source_[pos_] != '>' && source_[pos_] != '/')
      ++pos_;
    XmlNode node;
    node.tag = source_.substr(tag_begin, pos_ - tag_begin);
    const std::size_t attrs_begin = pos_;
    bool quoted = false;
    char quote = 0;
    while (pos_ < source_.size()) {
      const char ch = source_[pos_];
      if (quoted) {
        if (ch == quote) quoted = false;
      } else if (ch == '\"' || ch == '\'') {
        quoted = true;
        quote = ch;
      } else if (ch == '>') {
        break;
      }
      ++pos_;
    }
    if (pos_ >= source_.size())
      throw std::runtime_error("SVG distribution: unterminated start tag");
    std::string inside = source_.substr(attrs_begin, pos_ - attrs_begin);
    bool self_closing = false;
    const auto last = inside.find_last_not_of(" \t\r\n");
    if (last != knpos && inside[last] == '/') {
      self_closing = true;
      inside.erase(last, 1);
    }
    node.attrs = parse_attrs(inside);
    ++pos_;
    if (self_closing) return node;

    std::string text;
    while (pos_ < source_.size()) {
      if (starts_with("</")) {
        const auto end = source_.find('>', pos_ + 2);
        pos_ = end == knpos ? source_.size() : end + 1;
        break;
      }
      if (starts_with("<!--")) {
        const auto end = source_.find("-->", pos_ + 4);
        pos_ = end == knpos ? source_.size() : end + 3;
        continue;
      }
      if (starts_with("<![CDATA[")) {
        const auto end = source_.find("]]>", pos_ + 9);
        pos_ = end == knpos ? source_.size() : end + 3;
        continue;
      }
      if (source_[pos_] == '<') {
        node.children.push_back(parse_element());
      } else {
        text.push_back(source_[pos_++]);
      }
    }
    node.text = xml_decode(trim(text));
    return node;
  }
};

std::string local_name(const std::string& tag) {
  const auto colon = tag.find(':');
  return colon == knpos ? tag : tag.substr(colon + 1);
}

const XmlNode* find_first(const XmlNode& node, const std::string& name) {
  if (local_name(node.tag) == name) return &node;
  for (const auto& child : node.children) {
    if (const auto* found = find_first(child, name)) return found;
  }
  return nullptr;
}

void collect_text(const XmlNode& node, std::vector<std::string>& out) {
  if (local_name(node.tag) == "text" && !node.text.empty()) out.push_back(node.text);
  for (const auto& child : node.children) collect_text(child, out);
}

std::string normalized_id(const std::string& value) {
  std::string result;
  for (unsigned char ch : value) {
    if (std::isalnum(ch) || ch >= 0x80)
      result.push_back(static_cast<char>(std::tolower(ch)));
  }
  return result;
}

std::string uppercase_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::toupper(ch));
                 });
  return value;
}

struct Point {
  double x{0.0};
  double y{0.0};
};

double distance(Point a, Point b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<Point> parse_points(std::string source) {
  std::replace(source.begin(), source.end(), ',', ' ');
  std::istringstream input(source);
  std::vector<Point> result;
  double x = 0.0, y = 0.0;
  while (input >> x >> y) result.push_back({x, y});
  return result;
}

std::optional<std::vector<double>> transform_args(const std::string& transform,
                                                  const std::string& name) {
  const auto begin = transform.find(name + "(");
  if (begin == knpos) return std::nullopt;
  const auto args_begin = begin + name.size() + 1;
  const auto end = transform.find(')', args_begin);
  if (end == knpos) return std::nullopt;
  std::string args = transform.substr(args_begin, end - args_begin);
  std::replace(args.begin(), args.end(), ',', ' ');
  std::istringstream input(args);
  std::vector<double> values;
  double value = 0.0;
  while (input >> value) values.push_back(value);
  return values.empty() ? std::nullopt
                        : std::optional<std::vector<double>>(std::move(values));
}

struct GraphicObject {
  std::string object_id;
  std::string class_name;
  std::string psr_type;
  std::string href;
  std::string label;
  std::vector<Point> points;
  Point center{};
  Point terminal_a{};
  Point terminal_b{};
  bool has_terminals{false};
  std::map<std::string, std::string> model_attrs;
};

std::optional<GraphicObject> parse_graphic_object(
    const XmlNode& group,
    const std::unordered_map<std::string, std::string>& labels) {
  const auto* ref = find_first(group, "psr_ref");
  if (!ref) return std::nullopt;
  GraphicObject object;
  object.object_id = ref->attr("objectid");
  object.class_name = ref->attr("classname");
  object.psr_type = ref->attr("psrtype");
  if (object.object_id.empty() || object.class_name.empty()) return std::nullopt;
  const auto label = labels.find(normalized_id(object.object_id));
  if (label != labels.end()) object.label = label->second;
  if (const auto* model = find_first(group, "model"))
    object.model_attrs = model->attrs;

  if (const auto* polyline = find_first(group, "polyline"))
    object.points = parse_points(polyline->attr("points"));

  const auto* use = find_first(group, "use");
  if (!use) return object;
  object.href = use->attr("xlink:href");
  const auto scale = transform_args(use->attr("transform"), "scale");
  const auto translate = transform_args(use->attr("transform"), "translate");
  const auto rotate = transform_args(use->attr("transform"), "rotate");
  if (!scale || !translate || translate->size() < 2) return object;
  const double sx = (*scale)[0];
  const double sy = scale->size() > 1 ? (*scale)[1] : sx;
  object.center = {sx * (*translate)[0], sy * (*translate)[1]};
  double height = 0.0;
  try {
    height = std::stod(use->attr("height"));
  } catch (...) {
    return object;
  }
  const double half_length = std::abs(sy * height * 0.5);
  const double angle = (rotate && !rotate->empty() ? (*rotate)[0] : 0.0) *
                       kPi / 180.0;
  const Point delta{-std::sin(angle) * half_length,
                    std::cos(angle) * half_length};
  object.terminal_a = {object.center.x - delta.x, object.center.y - delta.y};
  object.terminal_b = {object.center.x + delta.x, object.center.y + delta.y};
  object.has_terminals = half_length > 0.0;
  return object;
}

std::vector<GraphicObject> extract_objects(const XmlNode& document) {
  std::unordered_map<std::string, std::string> labels;
  const XmlNode* svg = find_first(document, "svg");
  if (!svg) throw std::runtime_error("SVG distribution: missing <svg> root");

  for (const auto& layer : svg->children) {
    if (local_name(layer.tag) != "g" || layer.attr("id") != "Text_Layer")
      continue;
    for (const auto& group : layer.children) {
      const std::string id = group.attr("id");
      const std::string prefix = "TXT-PD_";
      if (id.rfind(prefix, 0) != 0 || id.size() == prefix.size()) continue;
      std::vector<std::string> parts;
      collect_text(group, parts);
      std::string value;
      for (const auto& part : parts) value += part;
      if (!value.empty()) labels[normalized_id(id.substr(prefix.size()))] = value;
    }
  }

  std::vector<GraphicObject> objects;
  for (const auto& layer : svg->children) {
    if (local_name(layer.tag) != "g") continue;
    const std::string layer_id = layer.attr("id");
    if (layer_id.size() < 6 ||
        layer_id.compare(layer_id.size() - 6, 6, "_Layer") != 0 ||
        layer_id == "Text_Layer" || layer_id == "BackGround_Layer")
      continue;
    for (const auto& group : layer.children) {
      if (local_name(group.tag) != "g") continue;
      if (auto object = parse_graphic_object(group, labels))
        objects.push_back(std::move(*object));
    }
  }
  return objects;
}

class DisjointSet {
 public:
  int add() {
    const int index = static_cast<int>(parent_.size());
    parent_.push_back(index);
    rank_.push_back(0);
    return index;
  }
  int find(int value) {
    if (parent_[value] != value) parent_[value] = find(parent_[value]);
    return parent_[value];
  }
  void unite(int lhs, int rhs) {
    lhs = find(lhs);
    rhs = find(rhs);
    if (lhs == rhs) return;
    if (rank_[lhs] < rank_[rhs]) std::swap(lhs, rhs);
    parent_[rhs] = lhs;
    if (rank_[lhs] == rank_[rhs]) ++rank_[lhs];
  }

 private:
  std::vector<int> parent_;
  std::vector<std::uint8_t> rank_;
};

class PointPool {
 public:
  explicit PointPool(double tolerance) : tolerance_(tolerance) {}

  int add(Point point) {
    for (std::size_t i = 0; i < points_.size(); ++i) {
      if (distance(points_[i], point) <= tolerance_) return static_cast<int>(i);
    }
    points_.push_back(point);
    return dsu_.add();
  }
  int root(int index) { return dsu_.find(index); }
  void unite(int lhs, int rhs) { dsu_.unite(lhs, rhs); }
  const std::vector<Point>& points() const { return points_; }

 private:
  double tolerance_;
  std::vector<Point> points_;
  DisjointSet dsu_;
};

bool is_line(const GraphicObject& object) {
  return object.class_name == "PWConductorSecPSR" ||
         object.class_name == "PWCableSecPSR";
}

bool is_ideal_conductor(const GraphicObject& object) {
  return object.class_name == "PWConnectLine" ||
         object.class_name == "PWInnerLinkLine" ||
         object.class_name == "PWBusbarPSR";
}

bool is_switch(const GraphicObject& object) {
  return object.class_name == "PWLoadSwitchPSR" ||
         object.class_name == "PWOPFusePSR" ||
         object.class_name == "PWOPBreakerPSR" ||
         object.class_name == "PWBreakerPSR" ||
         object.class_name == "PWDisconnectorPSR" ||
         object.class_name == "PWOPDisconnectorPSR" ||
         object.class_name == "PWBusCouplePSR";
}

bool is_transformer(const GraphicObject& object) {
  return object.class_name == "PWOPTransformerPSR";
}

bool point_on_segment(Point point, Point a, Point b, double tolerance) {
  const double length = distance(a, b);
  if (length <= tolerance) return distance(point, a) <= tolerance;
  const double cross = std::abs((point.x - a.x) * (b.y - a.y) -
                                (point.y - a.y) * (b.x - a.x));
  if (cross / length > tolerance) return false;
  const double dot = (point.x - a.x) * (point.x - b.x) +
                     (point.y - a.y) * (point.y - b.y);
  return dot <= tolerance * tolerance;
}

std::string object_name(const GraphicObject& object) {
  return object.label.empty() ? object.object_id : object.label;
}

bool whole_number(const std::string& value, double& result) {
  const std::string clean = trim(value);
  if (clean.empty()) return false;
  char* end = nullptr;
  result = std::strtod(clean.c_str(), &end);
  return end && *end == '\0' && std::isfinite(result);
}

std::optional<double> model_number(const GraphicObject& object,
                                   const std::string& name) {
  const auto it = object.model_attrs.find(name);
  if (it == object.model_attrs.end()) return std::nullopt;
  double value = 0.0;
  return whole_number(it->second, value) ? std::optional<double>(value)
                                         : std::nullopt;
}

std::string model_string(const GraphicObject& object,
                         const std::string& name,
                         const std::string& fallback = {}) {
  const auto it = object.model_attrs.find(name);
  return it == object.model_attrs.end() ? fallback : it->second;
}

bool model_bool(const GraphicObject& object,
                const std::string& name,
                bool fallback) {
  const std::string value = model_string(object, name);
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  return fallback;
}

int model_index(const GraphicObject& object,
                const std::string& name,
                std::set<int>& claimed,
                int& next_index) {
  const auto embedded = model_number(object, name);
  int candidate = embedded.has_value()
                      ? static_cast<int>(std::llround(*embedded))
                      : -1;
  if (candidate < 0 || claimed.count(candidate) != 0) {
    while (claimed.count(next_index) != 0) ++next_index;
    candidate = next_index++;
  }
  claimed.insert(candidate);
  return candidate;
}

double transformer_capacity_mva(const GraphicObject& object,
                                double fallback) {
  // Utility SVG labels conventionally end with transformer capacity in kVA.
  // Scan all numeric runs so labels such as "...公变(杭湾)400" are supported.
  double candidate = 0.0;
  std::string token;
  for (std::size_t i = 0; i <= object.label.size(); ++i) {
    const unsigned char ch = i < object.label.size()
                                 ? static_cast<unsigned char>(object.label[i])
                                 : 0;
    if ((ch >= '0' && ch <= '9') || ch == '.') {
      token.push_back(static_cast<char>(ch));
    } else if (!token.empty()) {
      double value = 0.0;
      if (whole_number(token, value) && value >= 10.0 && value <= 10000.0)
        candidate = value;
      token.clear();
    }
  }
  return candidate > 0.0 ? candidate / 1000.0 : fallback;
}

SwitchType switch_type(const GraphicObject& object) {
  if (object.class_name.find("Fuse") != knpos) return SwitchType::Fuse;
  if (object.class_name.find("Disconnector") != knpos)
    return SwitchType::Disconnector;
  if (object.class_name.find("LoadSwitch") != knpos)
    return SwitchType::LoadBreakSwitch;
  return SwitchType::CircuitBreaker;
}

SwitchRole switch_role(SwitchType type, bool closed) {
  if (!closed) return SwitchRole::Tie;
  if (type == SwitchType::Fuse || type == SwitchType::CircuitBreaker)
    return SwitchRole::Protection;
  if (type == SwitchType::Disconnector) return SwitchRole::Isolation;
  return SwitchRole::Sectionalizing;
}

void add_warning(SvgDistributionImportResult& result,
                 ImportReasonCode reason,
                 std::string locator,
                 std::string message) {
  result.warnings.push_back(message);
  result.report.add(ImportDisposition::Coerced, reason,
                    ImportSeverity::Warning, std::move(locator), message);
}

std::string xml_escape(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (char ch : value) {
    switch (ch) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      case '\'': result += "&apos;"; break;
      default: result.push_back(ch); break;
    }
  }
  return result;
}

std::string svg_number(double value) {
  std::ostringstream output;
  output << std::setprecision(12) << value;
  return output.str();
}

std::map<int, Point> distribution_export_layout(
    const HybridPowerSystem& system,
    const SvgDistributionExportOptions& opts) {
  std::set<int> bus_ids;
  for (const auto& bus : system.ac.buses)
    bus_ids.insert(bus.index);

  std::unordered_map<int, std::vector<int>> adjacency;
  const auto add_edge = [&](int from, int to) {
    if (from == to || bus_ids.count(from) == 0 || bus_ids.count(to) == 0)
      return;
    adjacency[from].push_back(to);
    adjacency[to].push_back(from);
  };
  for (const auto& branch : system.ac.branches)
    add_edge(branch.from_bus, branch.to_bus);
  for (const auto& sw : system.ac.switches)
    add_edge(sw.bus_from, sw.bus_to);
  for (const auto& transformer : system.ac.transformers_2w)
    add_edge(transformer.hv_bus, transformer.lv_bus);
  for (auto& [bus, neighbours] : adjacency) {
    std::sort(neighbours.begin(), neighbours.end());
    neighbours.erase(std::unique(neighbours.begin(), neighbours.end()),
                     neighbours.end());
  }

  std::vector<int> starts;
  for (const auto& grid : system.ac.external_grids)
    if (bus_ids.count(grid.bus) != 0)
      starts.push_back(grid.bus);
  for (const auto& bus : system.ac.buses)
    if (bus.bus_type == BusType::SLACK)
      starts.push_back(bus.index);
  starts.insert(starts.end(), bus_ids.begin(), bus_ids.end());

  std::unordered_set<int> visited;
  std::map<int, Point> positions;
  double component_top = opts.margin;
  for (int start : starts) {
    if (visited.count(start) != 0) continue;
    std::queue<int> queue;
    std::map<int, int> depth;
    queue.push(start);
    visited.insert(start);
    depth[start] = 0;
    while (!queue.empty()) {
      const int at = queue.front();
      queue.pop();
      for (int next : adjacency[at]) {
        if (!visited.insert(next).second) continue;
        depth[next] = depth[at] + 1;
        queue.push(next);
      }
    }
    std::map<int, std::vector<int>> levels;
    for (const auto& [bus, level] : depth) levels[level].push_back(bus);
    std::size_t max_rows = 1;
    for (auto& [level, buses] : levels) {
      std::sort(buses.begin(), buses.end());
      max_rows = std::max(max_rows, buses.size());
      for (std::size_t row = 0; row < buses.size(); ++row) {
        positions[buses[row]] = {
            opts.margin + level * opts.horizontal_spacing,
            component_top + row * opts.vertical_spacing};
      }
    }
    component_top += max_rows * opts.vertical_spacing + opts.vertical_spacing;
  }
  return positions;
}

std::pair<std::string, std::string> svg_switch_class(const Switch& sw) {
  switch (sw.switch_type) {
    case SwitchType::Fuse: return {"PWOPFusePSR", "0401009"};
    case SwitchType::Disconnector: return {"PWDisconnectorPSR", "0401006"};
    case SwitchType::LoadBreakSwitch: return {"PWLoadSwitchPSR", "0401004"};
    case SwitchType::CircuitBreaker: return {"PWBreakerPSR", "0401005"};
    case SwitchType::Recloser:
    case SwitchType::Sectionalizer:
      return {"PWOPBreakerPSR", "0401005"};
    case SwitchType::Unknown: return {"PWLoadSwitchPSR", "0401004"};
  }
  return {"PWLoadSwitchPSR", "0401004"};
}

}  // namespace

SvgDistributionImportResult from_svg_distribution(
    const std::string& svg, ImportMode mode,
    const SvgDistributionImportOptions& opts) {
  SvgDistributionImportResult result;
  result.report.binding_level = ImportBindingLevel::Rich;
  result.report.unit_assertion = UnitAssertion::BestEffort;

  if (svg.empty()) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::MissingRequired,
                      ImportSeverity::Error, "svg", "Empty SVG document");
    return result;
  }
  if (!(opts.base_mva > 0.0) || !(opts.nominal_mv_kv > 0.0) ||
      !(opts.nominal_lv_kv > 0.0) || !(opts.connection_tolerance > 0.0) ||
      !(opts.km_per_svg_unit > 0.0) ||
      !(opts.overhead_r_ohm_per_km >= 0.0) ||
      !(opts.overhead_x_ohm_per_km >= 0.0) ||
      !(opts.cable_r_ohm_per_km >= 0.0) ||
      !(opts.cable_x_ohm_per_km >= 0.0) ||
      !(opts.default_line_rate_mva > 0.0) ||
      !(opts.default_mv_cross_section_mm2 > 0.0) ||
      !(opts.default_transformer_sn_mva > 0.0) ||
      !(opts.default_transformer_vk_percent > 0.0) ||
      !(opts.default_transformer_vkr_percent >= 0.0) ||
      opts.default_transformer_vkr_percent >
          opts.default_transformer_vk_percent ||
      !(opts.transformer_load_factor >= 0.0) ||
      !(opts.power_factor > 0.0 && opts.power_factor <= 1.0) ||
      !(opts.synthetic_source_s_sc_max_mva > 0.0) ||
      !(opts.synthetic_source_s_sc_min_mva > 0.0) ||
      opts.synthetic_source_s_sc_min_mva >
          opts.synthetic_source_s_sc_max_mva ||
      !(opts.synthetic_source_rx > 0.0) ||
      !(opts.synthetic_source_zero_sequence_multiplier > 0.0)) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::RangeCoerced,
                      ImportSeverity::Error, "options",
                      "Invalid SVG distribution import options");
    return result;
  }

  std::vector<GraphicObject> objects;
  try {
    objects = extract_objects(XmlParser(svg).parse());
  } catch (const std::exception& error) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::ParseError,
                      ImportSeverity::Error, "svg", error.what());
    return result;
  }
  if (objects.empty()) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::MissingRequired,
                      ImportSeverity::Error, "svg/cge:psr_ref",
                      "No IEC-CGE cge:psr_ref equipment was found");
    return result;
  }

  result.system.name = opts.model_name;
  result.system.base_mva = opts.base_mva;
  result.system.ac.base_mva = opts.base_mva;
  result.system.dc.base_mva = opts.base_mva;

  PointPool pool(opts.connection_tolerance);
  struct PolylineRef {
    const GraphicObject* object{};
    std::vector<int> point_ids;
  };
  std::vector<PolylineRef> physical_lines;
  std::vector<PolylineRef> ideal_lines;
  struct DeviceRef {
    const GraphicObject* object{};
    int terminal_a{-1};
    int terminal_b{-1};
  };
  std::vector<DeviceRef> switches;
  std::vector<DeviceRef> transformers;
  std::vector<const GraphicObject*> substations;

  for (const auto& object : objects) {
    if (!object.model_attrs.empty()) ++result.embedded_parameter_objects;
    if (is_line(object) || is_ideal_conductor(object)) {
      if (object.points.size() < 2) {
        ++result.unresolved_objects;
        continue;
      }
      PolylineRef ref;
      ref.object = &object;
      for (Point point : object.points) ref.point_ids.push_back(pool.add(point));
      if (is_line(object)) {
        ++result.source_line_objects;
        physical_lines.push_back(std::move(ref));
      } else {
        ideal_lines.push_back(std::move(ref));
      }
    } else if (is_switch(object) || is_transformer(object)) {
      DeviceRef ref;
      ref.object = &object;
      if (object.has_terminals) {
        ref.terminal_a = pool.add(object.terminal_a);
        ref.terminal_b = pool.add(object.terminal_b);
      } else if (is_switch(object) && object.points.size() >= 2) {
        // Ring-main-unit bus couplers are emitted as a single polyline instead
        // of a reusable <use> symbol. Its two endpoints are still terminals.
        ref.terminal_a = pool.add(object.points.front());
        ref.terminal_b = pool.add(object.points.back());
      } else {
        ++result.unresolved_objects;
        continue;
      }
      if (is_switch(object)) {
        ++result.source_switch_objects;
        switches.push_back(ref);
      } else {
        ++result.source_transformer_objects;
        transformers.push_back(ref);
      }
    } else if (object.class_name == "Substation") {
      substations.push_back(&object);
    }
  }

  // Ideal connectors and busbars collapse into a single connectivity node.
  // Also attach any line/device vertex that lies on a busbar interior.
  for (const auto& line : ideal_lines) {
    for (std::size_t i = 1; i < line.point_ids.size(); ++i) {
      const int a = line.point_ids[i - 1];
      const int b = line.point_ids[i];
      pool.unite(a, b);
      const Point pa = pool.points()[a];
      const Point pb = pool.points()[b];
      for (std::size_t p = 0; p < pool.points().size(); ++p) {
        if (point_on_segment(pool.points()[p], pa, pb,
                             opts.connection_tolerance))
          pool.unite(a, static_cast<int>(p));
      }
    }
  }

  std::set<int> used_roots;
  auto use_root = [&](int point_id) { used_roots.insert(pool.root(point_id)); };
  for (const auto& line : ideal_lines) {
    if (!model_bool(*line.object, "preserve_node", false)) continue;
    for (int point : line.point_ids) use_root(point);
  }
  for (const auto& line : physical_lines)
    for (int point : line.point_ids) use_root(point);
  for (const auto& device : switches) {
    use_root(device.terminal_a);
    use_root(device.terminal_b);
  }
  for (const auto& device : transformers) {
    use_root(device.terminal_a);
    use_root(device.terminal_b);
  }
  if (used_roots.empty()) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::MissingRequired,
                      ImportSeverity::Error, "svg/geometry",
                      "No calculable line or device terminals were recovered");
    return result;
  }

  std::unordered_map<int, int> root_to_bus;
  std::unordered_map<int, const GraphicObject*> bus_models;
  for (const auto& line : ideal_lines) {
    if (!model_bool(*line.object, "preserve_node", false)) continue;
    for (int point : line.point_ids)
      bus_models[pool.root(point)] = line.object;
  }
  std::set<int> claimed_bus_ids;
  int next_bus_index = 1;
  for (int root : used_roots) {
    ACBus bus;
    const auto model = bus_models.find(root);
    const GraphicObject* source =
        model == bus_models.end() ? nullptr : model->second;
    const auto embedded_index = source == nullptr
                                    ? std::optional<double>{}
                                    : model_number(*source, "source_index");
    int requested_index = embedded_index.has_value()
                              ? static_cast<int>(std::llround(*embedded_index))
                              : -1;
    if (requested_index < 0 || claimed_bus_ids.count(requested_index) != 0) {
      while (claimed_bus_ids.count(next_bus_index) != 0) ++next_bus_index;
      requested_index = next_bus_index++;
    }
    claimed_bus_ids.insert(requested_index);
    bus.index = requested_index;
    bus.base_kv = source == nullptr
                      ? opts.nominal_mv_kv
                      : model_number(*source, "base_kv")
                            .value_or(opts.nominal_mv_kv);
    bus.name = source == nullptr
                   ? "SVG node " + std::to_string(bus.index)
                   : model_string(*source, "name",
                                  "SVG node " + std::to_string(bus.index));
    if (source != nullptr) {
      bus.in_service = model_bool(*source, "in_service", true);
      bus.vm_pu = model_number(*source, "vm_pu").value_or(bus.vm_pu);
      bus.va_deg = model_number(*source, "va_deg").value_or(bus.va_deg);
      bus.vmin_pu = model_number(*source, "vmin_pu").value_or(bus.vmin_pu);
      bus.vmax_pu = model_number(*source, "vmax_pu").value_or(bus.vmax_pu);
      bus.bus_type = bus_type_from_str(
          model_string(*source, "bus_type", bus_type_str(bus.bus_type)));
    }
    result.system.ac.buses.push_back(bus);
    root_to_bus[root] = bus.index;
  }
  result.recovered_nodes = result.system.ac.buses.size();
  auto bus_of = [&](int point_id) { return root_to_bus.at(pool.root(point_id)); };

  int branch_index = 1;
  std::set<int> claimed_branch_ids;
  for (const auto& line : physical_lines) {
    for (std::size_t i = 1; i < line.point_ids.size(); ++i) {
      const int from = bus_of(line.point_ids[i - 1]);
      const int to = bus_of(line.point_ids[i]);
      if (from == to) continue;
      const double svg_length =
          distance(pool.points()[line.point_ids[i - 1]],
                   pool.points()[line.point_ids[i]]);
      const bool cable = line.object->class_name == "PWCableSecPSR";
      ACBranch branch;
      branch.index = model_index(*line.object, "source_index",
                                 claimed_branch_ids, branch_index);
      branch.from_bus = from;
      branch.to_bus = to;
      branch.in_service = model_bool(*line.object, "in_service", true);
      branch.name = object_name(*line.object);
      if (line.point_ids.size() > 2)
        branch.name += " [" + std::to_string(i) + "]";
      branch.length_km = model_number(*line.object, "length_km")
                             .value_or(std::max(
                                 1e-4, svg_length * opts.km_per_svg_unit));
      branch.r_ohm_per_km = model_number(*line.object, "r_ohm_per_km")
                                .value_or(cable ? opts.cable_r_ohm_per_km
                                                : opts.overhead_r_ohm_per_km);
      branch.x_ohm_per_km = model_number(*line.object, "x_ohm_per_km")
                                .value_or(cable ? opts.cable_x_ohm_per_km
                                                : opts.overhead_x_ohm_per_km);
      const double z_base = opts.nominal_mv_kv * opts.nominal_mv_kv /
                            opts.base_mva;
      branch.r_pu = model_number(*line.object, "r_pu")
                        .value_or(branch.r_ohm_per_km * branch.length_km /
                                  z_base);
      branch.x_pu = model_number(*line.object, "x_pu")
                        .value_or(branch.x_ohm_per_km * branch.length_km /
                                  z_base);
      branch.rate_a_mva = model_number(*line.object, "rate_a_mva")
                              .value_or(opts.default_line_rate_mva);
      branch.rate_b_mva = model_number(*line.object, "rate_b_mva")
                              .value_or(branch.rate_a_mva);
      branch.rate_c_mva = model_number(*line.object, "rate_c_mva")
                              .value_or(branch.rate_a_mva);
      branch.line_type = model_string(
          *line.object, "line_type", cable ? "cable" : "overhead");
      branch.conductor_model = model_string(*line.object, "conductor_model");
      branch.cross_section_mm2 =
          model_number(*line.object, "cross_section_mm2")
              .value_or(opts.default_mv_cross_section_mm2);
      branch.cross_section_inferred = model_bool(
          *line.object, "cross_section_inferred",
          model_number(*line.object, "cross_section_mm2") == std::nullopt);
      branch.parameter_source = model_string(
          *line.object, "parameter_source", "svg_geometry_estimate");
      branch.parameters_inferred = model_bool(
          *line.object, "parameters_inferred",
          line.object->model_attrs.empty());
      result.system.ac.branches.push_back(std::move(branch));
    }
  }

  int switch_index = 1;
  std::set<int> claimed_switch_ids;
  for (const auto& ref : switches) {
    const int from = bus_of(ref.terminal_a);
    const int to = bus_of(ref.terminal_b);
    if (from == to) continue;
    Switch sw;
    sw.index = model_index(*ref.object, "source_index",
                           claimed_switch_ids, switch_index);
    sw.name = object_name(*ref.object);
    sw.bus_from = from;
    sw.bus_to = to;
    sw.in_service = model_bool(*ref.object, "in_service", true);
    sw.switch_type = switch_type(*ref.object);
    sw.closed = model_bool(*ref.object, "closed",
                           ref.object->href.find("@1") == knpos);
    sw.normal_closed = model_bool(*ref.object, "normal_closed", sw.closed);
    sw.normal_state_explicit = true;
    sw.role = switch_role_from_str(model_string(
        *ref.object, "role", switch_role_str(switch_role(sw.switch_type,
                                                         sw.closed))));
    sw.controlled_element_type =
        model_string(*ref.object, "controlled_element_type");
    sw.controlled_element_index = static_cast<int>(std::llround(
        model_number(*ref.object, "controlled_element_index").value_or(-1.0)));
    sw.controlled_branch_index = static_cast<int>(std::llround(
        model_number(*ref.object, "controlled_branch_index").value_or(-1.0)));
    sw.protection_zone_id = static_cast<int>(std::llround(
        model_number(*ref.object, "protection_zone_id").value_or(0.0)));
    sw.upstream_protective_switch_index = static_cast<int>(std::llround(
        model_number(*ref.object, "upstream_protective_switch_index")
            .value_or(-1.0)));
    sw.binding_inferred = model_bool(*ref.object, "binding_inferred",
                                     ref.object->model_attrs.empty());
    sw.binding_source = model_string(
        *ref.object, "binding_source", "svg-symbol-state-and-geometry");
    result.system.ac.switches.push_back(std::move(sw));
  }

  // Degree before transformer insertion identifies the terminal attached to
  // the MV drawing. The opposite terminal becomes the 0.4 kV load-side bus.
  std::unordered_map<int, int> mv_degree;
  for (const auto& branch : result.system.ac.branches) {
    ++mv_degree[branch.from_bus];
    ++mv_degree[branch.to_bus];
  }
  for (const auto& sw : result.system.ac.switches) {
    ++mv_degree[sw.bus_from];
    ++mv_degree[sw.bus_to];
  }

  int transformer_index = 1;
  int load_index = 1;
  std::set<int> claimed_transformer_ids;
  std::set<int> claimed_load_ids;
  const double q_ratio =
      std::tan(std::acos(std::clamp(opts.power_factor, 1e-6, 1.0)));
  for (const auto& ref : transformers) {
    int hv_bus = bus_of(ref.terminal_a);
    int lv_bus = bus_of(ref.terminal_b);
    if (!model_bool(*ref.object, "terminal_a_is_hv", false) &&
        mv_degree[lv_bus] > mv_degree[hv_bus])
      std::swap(hv_bus, lv_bus);
    if (hv_bus == lv_bus) continue;
    for (auto& bus : result.system.ac.buses)
      if (bus.index == lv_bus) bus.base_kv = opts.nominal_lv_kv;

    const double sn_mva = model_number(*ref.object, "sn_mva")
                              .value_or(transformer_capacity_mva(
                                  *ref.object,
                                  opts.default_transformer_sn_mva));
    Transformer2W transformer;
    transformer.index = model_index(*ref.object, "source_index",
                                    claimed_transformer_ids,
                                    transformer_index);
    transformer.name = object_name(*ref.object);
    transformer.hv_bus = hv_bus;
    transformer.lv_bus = lv_bus;
    transformer.in_service = model_bool(*ref.object, "in_service", true);
    transformer.sn_mva = sn_mva;
    transformer.std_type = model_string(*ref.object, "std_type");
    transformer.vn_hv_kv = model_number(*ref.object, "vn_hv_kv")
                               .value_or(opts.nominal_mv_kv);
    transformer.vn_lv_kv = model_number(*ref.object, "vn_lv_kv")
                               .value_or(opts.nominal_lv_kv);
    transformer.vk_percent = model_number(*ref.object, "vk_percent")
                                 .value_or(opts.default_transformer_vk_percent);
    transformer.vkr_percent = model_number(*ref.object, "vkr_percent")
                                  .value_or(opts.default_transformer_vkr_percent);
    transformer.pk_kw = model_number(*ref.object, "pk_kw")
                            .value_or(transformer.sn_mva * 1000.0 *
                                      transformer.vkr_percent / 100.0);
    transformer.pfe_kw = model_number(*ref.object, "pfe_kw").value_or(0.0);
    transformer.i0_percent =
        model_number(*ref.object, "i0_percent").value_or(0.5);
    transformer.vector_group = model_string(*ref.object, "vector_group");
    transformer.z0_percent =
        model_number(*ref.object, "z0_percent").value_or(0.0);
    transformer.x0_r0 = model_number(*ref.object, "x0_r0").value_or(0.0);
    transformer.tap_side = static_cast<int>(std::llround(
        model_number(*ref.object, "tap_side").value_or(0.0)));
    transformer.tap_pos = static_cast<int>(std::llround(
        model_number(*ref.object, "tap_pos").value_or(0.0)));
    transformer.tap_min = static_cast<int>(std::llround(
        model_number(*ref.object, "tap_min").value_or(0.0)));
    transformer.tap_max = static_cast<int>(std::llround(
        model_number(*ref.object, "tap_max").value_or(0.0)));
    transformer.tap_neutral = static_cast<int>(std::llround(
        model_number(*ref.object, "tap_neutral").value_or(0.0)));
    transformer.tap_step_percent =
        model_number(*ref.object, "tap_step_percent").value_or(0.0);
    transformer.shift_deg =
        model_number(*ref.object, "shift_deg").value_or(0.0);
    transformer.n_parallel = static_cast<int>(std::llround(
        model_number(*ref.object, "n_parallel").value_or(1.0)));
    transformer.source_branch_idx = static_cast<int>(std::llround(
        model_number(*ref.object, "source_branch_idx").value_or(0.0)));
    result.system.ac.transformers_2w.push_back(std::move(transformer));

    Load load;
    load.index = model_index(*ref.object, "load_source_index",
                             claimed_load_ids, load_index);
    load.bus = lv_bus;
    load.in_service = model_bool(*ref.object, "load_in_service", true);
    load.name = model_string(
        *ref.object, "load_name", object_name(*ref.object) + " estimated load");
    load.sn_mva = model_number(*ref.object, "load_sn_mva")
                      .value_or(sn_mva * opts.transformer_load_factor);
    load.p_mw = model_number(*ref.object, "load_p_mw")
                    .value_or(load.sn_mva * opts.power_factor);
    load.q_mvar = model_number(*ref.object, "load_q_mvar")
                      .value_or(load.p_mw * q_ratio);
    result.system.ac.loads.push_back(std::move(load));
  }

  // Find connected components over in-service lines, closed switches and
  // transformers. Only substation-bearing components receive a source; if no
  // substation symbol exists, the largest component is the explicit fallback.
  DisjointSet electrical;
  for (std::size_t i = 0; i < result.system.ac.buses.size(); ++i) electrical.add();
  std::unordered_map<int, int> electrical_position;
  for (int position = 0;
       position < static_cast<int>(result.system.ac.buses.size()); ++position)
    electrical_position[result.system.ac.buses[position].index] = position;
  const auto bus_pos = [&](int bus) { return electrical_position.at(bus); };
  for (const auto& branch : result.system.ac.branches)
    if (branch.in_service)
      electrical.unite(bus_pos(branch.from_bus), bus_pos(branch.to_bus));
  for (const auto& sw : result.system.ac.switches)
    if (sw.in_service && sw.closed)
      electrical.unite(bus_pos(sw.bus_from), bus_pos(sw.bus_to));
  for (const auto& transformer : result.system.ac.transformers_2w)
    if (transformer.in_service)
      electrical.unite(bus_pos(transformer.hv_bus), bus_pos(transformer.lv_bus));

  std::map<int, std::vector<int>> components;
  for (const auto& bus : result.system.ac.buses)
    components[electrical.find(bus_pos(bus.index))].push_back(bus.index);
  std::set<int> preferred_source_buses;
  std::unordered_map<int, const GraphicObject*> source_models_by_bus;
  for (const auto* substation : substations) {
    double best = std::numeric_limits<double>::infinity();
    int best_bus = 0;
    for (const auto& [root, bus] : root_to_bus) {
      const double candidate = distance(substation->center, pool.points()[root]);
      if (candidate < best) {
        best = candidate;
        best_bus = bus;
      }
    }
    if (best_bus != 0) {
      preferred_source_buses.insert(best_bus);
      source_models_by_bus[best_bus] = substation;
    }
  }

  std::set<int> source_components;
  for (int bus : preferred_source_buses)
    source_components.insert(electrical.find(bus_pos(bus)));
  if (source_components.empty() && !components.empty()) {
    const auto largest = std::max_element(
        components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.second.size() < rhs.second.size();
        });
    source_components.insert(largest->first);
    preferred_source_buses.insert(largest->second.front());
  }
  if (!opts.auto_add_external_grids) {
    source_components.clear();
    preferred_source_buses.clear();
  }

  int grid_index = 1;
  std::set<int> claimed_grid_ids;
  if (opts.auto_add_external_grids) {
    for (int component : source_components) {
      int source_bus = components.at(component).front();
      const auto preferred = std::find_if(
          preferred_source_buses.begin(), preferred_source_buses.end(),
          [&](int bus) {
            return electrical.find(bus_pos(bus)) == component;
          });
      if (preferred != preferred_source_buses.end()) source_bus = *preferred;
      ExternalGrid grid;
      grid.bus = source_bus;
      const auto source_model = source_models_by_bus.find(source_bus);
      const GraphicObject* source = source_model == source_models_by_bus.end()
                                        ? nullptr
                                        : source_model->second;
      if (source == nullptr) {
        while (claimed_grid_ids.count(grid_index) != 0) ++grid_index;
        grid.index = grid_index++;
        claimed_grid_ids.insert(grid.index);
      } else {
        grid.index = model_index(*source, "source_index", claimed_grid_ids,
                                 grid_index);
      }
      grid.in_service = source == nullptr
                            ? true
                            : model_bool(*source, "in_service", true);
      grid.name = source == nullptr
                      ? "SVG synthetic source " + std::to_string(grid.index)
                      : model_string(*source, "name", object_name(*source));
      grid.vn_kv = source == nullptr
                       ? opts.nominal_mv_kv
                       : model_number(*source, "vn_kv")
                             .value_or(opts.nominal_mv_kv);
      grid.s_sc_max_mva = source == nullptr
                              ? opts.synthetic_source_s_sc_max_mva
                              : model_number(*source, "s_sc_max_mva")
                                    .value_or(opts.synthetic_source_s_sc_max_mva);
      grid.s_sc_min_mva = source == nullptr
                              ? opts.synthetic_source_s_sc_min_mva
                              : model_number(*source, "s_sc_min_mva")
                                    .value_or(opts.synthetic_source_s_sc_min_mva);
      grid.rx_max = source == nullptr
                        ? opts.synthetic_source_rx
                        : model_number(*source, "rx_max")
                              .value_or(opts.synthetic_source_rx);
      grid.rx_min = source == nullptr
                        ? opts.synthetic_source_rx
                        : model_number(*source, "rx_min")
                              .value_or(opts.synthetic_source_rx);
      const double z_pu = opts.base_mva / grid.s_sc_max_mva;
      grid.x_pu = source == nullptr
                      ? z_pu / std::sqrt(1.0 + grid.rx_max * grid.rx_max)
                      : model_number(*source, "x_pu")
                            .value_or(z_pu / std::sqrt(
                                1.0 + grid.rx_max * grid.rx_max));
      grid.r_pu = source == nullptr
                      ? grid.rx_max * grid.x_pu
                      : model_number(*source, "r_pu")
                            .value_or(grid.rx_max * grid.x_pu);
      grid.x0_pu = source == nullptr
                       ? grid.x_pu *
                             opts.synthetic_source_zero_sequence_multiplier
                       : model_number(*source, "x0_pu")
                             .value_or(grid.x_pu *
                                       opts.synthetic_source_zero_sequence_multiplier);
      grid.r0_pu = source == nullptr
                       ? grid.r_pu *
                             opts.synthetic_source_zero_sequence_multiplier
                       : model_number(*source, "r0_pu")
                             .value_or(grid.r_pu *
                                       opts.synthetic_source_zero_sequence_multiplier);
      grid.x_r = grid.x_pu / grid.r_pu;
      if (source != nullptr && !source->model_attrs.empty())
        ++result.restored_external_grids;
      else
        ++result.synthetic_external_grids;
      result.system.ac.external_grids.push_back(std::move(grid));
      for (auto& bus : result.system.ac.buses)
        if (bus.index == source_bus) bus.bus_type = BusType::SLACK;
    }
  }
  for (const auto& [component, buses] : components) {
    if (source_components.count(component) != 0) continue;
    ++result.isolated_components;
    result.isolated_buses += buses.size();
    for (auto& bus : result.system.ac.buses) {
      if (std::find(buses.begin(), buses.end(), bus.index) != buses.end())
        bus.bus_type = BusType::ISOLATED;
    }
  }

  if (opts.auto_complete_parameters) {
    DesignHandbookCompletionOptions completion_options;
    completion_options.apply = true;
    completion_options.overwrite_import_estimates = true;
    completion_options.infer_switch_bindings = true;
    completion_options.overwrite_inferred_switch_bindings = true;
    completion_options.default_mv_cross_section_mm2 =
        opts.default_mv_cross_section_mm2;
    completion_options.cable_reactance_ohm_per_km =
        opts.cable_x_ohm_per_km;
    completion_options.overhead_reactance_ohm_per_km =
        opts.overhead_x_ohm_per_km;
    result.parameter_completion = complete_design_handbook_parameters(
        result.system, completion_options);
    result.parameter_completion_applied = true;
  }

  result.report.add(
      ImportDisposition::Accepted, ImportReasonCode::Ok,
      ImportSeverity::Info, "svg/cge:psr_ref",
      "Recovered " + std::to_string(result.source_line_objects) +
          " line, " + std::to_string(result.source_switch_objects) +
          " switch, and " +
          std::to_string(result.source_transformer_objects) +
          " transformer objects from IEC-CGE metadata");
  if (result.embedded_parameter_objects > 0)
    result.report.add(
        ImportDisposition::Accepted, ImportReasonCode::Ok,
        ImportSeverity::Info, "svg/hacdcpf:model",
        "Restored HACDCPF parameter extensions from " +
            std::to_string(result.embedded_parameter_objects) +
            " SVG object(s).");
  add_warning(result, ImportReasonCode::UnitInferred, "svg/geometry",
              "Electrical connectivity was recovered from SVG coordinates; "
              "the source does not provide CIM Terminal/ConnectivityNode links.");
  add_warning(result, ImportReasonCode::UnitInferred, "svg/line-parameters",
              "Line lengths were estimated from drawing geometry; electrical "
              "parameters use standards-aware conductor tables plus explicit "
              "engineering screening assumptions.");
  if (result.parameter_completion_applied)
    add_warning(result, ImportReasonCode::UnitInferred,
                "svg/parameter-completion",
                "Applied standards-aware completion to " +
                    std::to_string(result.parameter_completion.candidates) +
                    " line candidate(s), changing " +
                    std::to_string(result.parameter_completion.fields_changed) +
                    " field(s); manufacturer nameplates remain authoritative.");
  if (!result.system.ac.transformers_2w.empty())
    add_warning(result, ImportReasonCode::MissingRequired,
                "svg/transformer-loads",
                "Transformer impedance and terminal demand were estimated from "
                "label capacity (when present), load factor, and power factor.");
  if (result.synthetic_external_grids > 0)
    add_warning(result, ImportReasonCode::MissingRequired, "svg/source",
                "Added " + std::to_string(result.synthetic_external_grids) +
                    " synthetic external-grid equivalent(s) at substation "
                    "drawing component(s).");
  if (result.restored_external_grids > 0)
    result.report.add(
        ImportDisposition::Accepted, ImportReasonCode::Ok,
        ImportSeverity::Info, "svg/source",
        "Restored " + std::to_string(result.restored_external_grids) +
            " external-grid equivalent(s) from embedded HACDCPF parameters.");
  if (!opts.auto_add_external_grids)
    add_warning(result, ImportReasonCode::MissingRequired, "svg/source",
                "Synthetic external-grid creation was disabled; all recovered "
                "components remain explicitly isolated until a source is "
                "provided by the caller.");
  if (result.isolated_components > 0)
    add_warning(result, ImportReasonCode::StructuralLoss, "svg/islands",
                "Kept " + std::to_string(result.isolated_components) +
                    " source-disconnected drawing component(s), containing " +
                    std::to_string(result.isolated_buses) +
                    " bus(es), explicitly isolated from power-flow supply.");
  if (result.unresolved_objects > 0)
    add_warning(result, ImportReasonCode::StructuralLoss, "svg/unresolved",
                "Skipped " + std::to_string(result.unresolved_objects) +
                    " recognized object(s) without usable geometry.");

  if (mode == ImportMode::Strict && !passes_mode(result.report, mode)) {
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::UnitInferred,
                      ImportSeverity::Error, "svg",
                      "Strict import rejects geometry-derived topology and "
                      "best-effort electrical parameters");
  }
  return result;
}

SvgDistributionImportResult load_svg_distribution(
    const std::filesystem::path& path, ImportMode mode,
    const SvgDistributionImportOptions& opts) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    SvgDistributionImportResult result;
    result.report.binding_level = ImportBindingLevel::Rich;
    result.report.unit_assertion = UnitAssertion::BestEffort;
    result.report.add(ImportDisposition::Rejected,
                      ImportReasonCode::ParseError,
                      ImportSeverity::Error, path_utf8(path),
                      "Cannot open SVG distribution file");
    return result;
  }
  std::string svg((std::istreambuf_iterator<char>(input)),
                  std::istreambuf_iterator<char>());
  auto options = opts;
  if (options.model_name == "SVG distribution feeder")
    options.model_name = path_utf8(path.stem());
  return from_svg_distribution(svg, mode, options);
}

SvgDistributionExportResult to_svg_distribution(
    const HybridPowerSystem& system,
    const SvgDistributionExportOptions& opts) {
  if (system.ac.buses.empty())
    throw std::invalid_argument(
        "SVG distribution export requires at least one AC bus");
  if (!(opts.horizontal_spacing > 0.0) ||
      !(opts.vertical_spacing > 0.0) || !(opts.margin >= 0.0))
    throw std::invalid_argument("Invalid SVG distribution export layout options");

  SvgDistributionExportResult result;
  const auto positions = distribution_export_layout(system, opts);
  result.exported_buses = positions.size();
  double max_x = opts.margin;
  double max_y = opts.margin;
  for (const auto& [bus, point] : positions) {
    (void)bus;
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  const double width = max_x + opts.margin;
  const double height = max_y + opts.margin;
  const std::string title = opts.title.empty()
                                ? (system.name.empty()
                                       ? "HACDCPF distribution model"
                                       : system.name)
                                : opts.title;

  using Attributes = std::vector<std::pair<std::string, std::string>>;
  struct Label {
    std::string object_id;
    Point point;
    std::string text;
  };
  std::vector<Label> labels;
  std::ostringstream output;
  output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
         << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
         << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
         << "xmlns:cge=\"http://iec.ch/TC57/2005/SVG-schema#\" "
         << "xmlns:hacdcpf=\"https://xjtu.edu.cn/hacdcpf/svg-model/1\" "
         << "width=\"" << svg_number(width) << "\" height=\""
         << svg_number(height) << "\" viewBox=\"0 0 "
         << svg_number(width) << ' ' << svg_number(height) << "\">\n"
         << "  <title>" << xml_escape(title) << "</title>\n"
         << "  <defs>\n"
         << "    <symbol id=\"TerminalSpan\" viewBox=\"0 0 1 1\"><path d=\"M0 0 L0 1\"/></symbol>\n"
         << "    <symbol id=\"Switch\" viewBox=\"0 0 1 1\"><path d=\"M0 0 L0 1\"/></symbol>\n"
         << "    <symbol id=\"Switch@1\" viewBox=\"0 0 1 1\"><path d=\"M0 0 L0 1\"/></symbol>\n"
         << "    <symbol id=\"Transformer\" viewBox=\"0 0 1 1\"><path d=\"M0 0 L0 1\"/></symbol>\n"
         << "    <symbol id=\"Substation\" viewBox=\"0 0 1 1\"><path d=\"M0 0 L0 1\"/></symbol>\n"
         << "  </defs>\n"
         << "  <g id=\"BackGround_Layer\"><rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/></g>\n";

  const auto write_metadata = [&](const std::string& object_id,
                                  const std::string& class_name,
                                  const std::string& psr_type,
                                  const Attributes& attributes) {
    output << "      <metadata><cge:psr_ref objectid=\""
           << xml_escape(object_id) << "\" globeid=\""
           << xml_escape(object_id) << "\" psrtype=\""
           << xml_escape(psr_type) << "\" classname=\""
           << xml_escape(class_name) << "\"/>";
    if (opts.include_hacdcpf_parameters) {
      output << "<hacdcpf:model";
      for (const auto& [name, value] : attributes)
        output << ' ' << name << "=\"" << xml_escape(value) << "\"";
      output << "/>";
    }
    output << "</metadata>\n";
  };
  const auto point_of = [&](int bus) -> std::optional<Point> {
    const auto it = positions.find(bus);
    return it == positions.end() ? std::nullopt
                                 : std::optional<Point>(it->second);
  };
  const auto bool_text = [](bool value) { return value ? "true" : "false"; };

  output << "  <g id=\"BusbarSection_Layer\" stroke=\"#1f2937\" stroke-width=\"3\" fill=\"none\">\n";
  for (const auto& bus : system.ac.buses) {
    const auto point = point_of(bus.index);
    if (!point) continue;
    const std::string object_id =
        "hacdcpf-ac-bus-" + std::to_string(bus.index);
    output << "    <g id=\"PD_" << xml_escape(object_id) << "\">\n";
    write_metadata(object_id, "PWBusbarPSR", "0401001",
                   {{"source_index", std::to_string(bus.index)},
                    {"name", bus.name},
                    {"base_kv", svg_number(bus.base_kv)},
                    {"in_service", bool_text(bus.in_service)},
                    {"bus_type", bus_type_str(bus.bus_type)},
                    {"vm_pu", svg_number(bus.vm_pu)},
                    {"va_deg", svg_number(bus.va_deg)},
                    {"vmin_pu", svg_number(bus.vmin_pu)},
                    {"vmax_pu", svg_number(bus.vmax_pu)},
                    {"preserve_node", "true"}});
    output << "      <polyline points=\"" << svg_number(point->x - 10.0)
           << ',' << svg_number(point->y) << ' '
           << svg_number(point->x + 10.0) << ',' << svg_number(point->y)
           << "\"/>\n    </g>\n";
    labels.push_back({object_id, {point->x + 12.0, point->y - 8.0},
                      bus.name.empty() ? "AC bus " + std::to_string(bus.index)
                                       : bus.name});
  }
  output << "  </g>\n";

  output << "  <g id=\"ACLineSegment_Layer\" stroke=\"#2563eb\" stroke-width=\"2\" fill=\"none\">\n";
  for (const auto& branch : system.ac.branches) {
    const auto from = point_of(branch.from_bus);
    const auto to = point_of(branch.to_bus);
    if (!from || !to) {
      result.warnings.push_back(
          "Skipped AC branch " + std::to_string(branch.index) +
          " because an endpoint bus is missing from the AC index space.");
      continue;
    }
    const bool cable = uppercase_ascii(branch.line_type).find("CABLE") != knpos ||
                       branch.line_type.find("电缆") != knpos;
    const std::string object_id =
        "hacdcpf-ac-branch-" + std::to_string(branch.index);
    const std::string source = branch.parameter_source.empty()
                                   ? (branch.parameters_inferred
                                          ? "svg_estimate_embedded"
                                          : "svg_embedded_authored")
                                   : branch.parameter_source;
    output << "    <g id=\"PD_" << xml_escape(object_id) << "\">\n";
    write_metadata(
        object_id, cable ? "PWCableSecPSR" : "PWConductorSecPSR",
        cable ? "0402002" : "0401002",
        {{"source_index", std::to_string(branch.index)},
         {"name", branch.name},
         {"in_service", bool_text(branch.in_service)},
         {"length_km", svg_number(branch.length_km)},
         {"r_ohm_per_km", svg_number(branch.r_ohm_per_km)},
         {"x_ohm_per_km", svg_number(branch.x_ohm_per_km)},
         {"r_pu", svg_number(branch.r_pu)},
         {"x_pu", svg_number(branch.x_pu)},
         {"rate_a_mva", svg_number(branch.rate_a_mva)},
         {"rate_b_mva", svg_number(branch.rate_b_mva)},
         {"rate_c_mva", svg_number(branch.rate_c_mva)},
         {"line_type", branch.line_type},
         {"conductor_model", branch.conductor_model},
         {"cross_section_mm2", svg_number(branch.cross_section_mm2)},
         {"cross_section_inferred", bool_text(branch.cross_section_inferred)},
         {"parameter_source", source},
         {"parameters_inferred", bool_text(branch.parameters_inferred)}});
    output << "      <polyline points=\"" << svg_number(from->x) << ','
           << svg_number(from->y) << ' ' << svg_number(to->x) << ','
           << svg_number(to->y) << "\""
           << (branch.in_service ? "" : " stroke-dasharray=\"5 4\"")
           << "/>\n    </g>\n";
    labels.push_back({object_id,
                      {(from->x + to->x) * 0.5 + 6.0,
                       (from->y + to->y) * 0.5 - 6.0},
                      branch.name.empty()
                          ? "AC branch " + std::to_string(branch.index)
                          : branch.name});
    ++result.exported_branches;
  }
  output << "  </g>\n";

  const auto write_device_span = [&](const std::string& href, Point from,
                                     Point to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::max(1e-6, std::hypot(dx, dy));
    const double angle = std::atan2(-dx, dy) * 180.0 / kPi;
    const Point center{(from.x + to.x) * 0.5, (from.y + to.y) * 0.5};
    output << "      <use xlink:href=\"" << href
           << "\" width=\"1\" height=\"" << svg_number(length)
           << "\" transform=\"scale(1) translate(" << svg_number(center.x)
           << ' ' << svg_number(center.y) << ") rotate(" << svg_number(angle)
           << ")\" opacity=\"0\"/>\n";
    return center;
  };

  output << "  <g id=\"Breaker_Layer\" stroke=\"#111827\" stroke-width=\"2\" fill=\"#ffffff\">\n";
  for (const auto& sw : system.ac.switches) {
    const auto from = point_of(sw.bus_from);
    const auto to = point_of(sw.bus_to);
    if (!from || !to) {
      result.warnings.push_back(
          "Skipped switch " + std::to_string(sw.index) +
          " because an endpoint bus is missing from the AC index space.");
      continue;
    }
    const auto [class_name, psr_type] = svg_switch_class(sw);
    const std::string object_id =
        "hacdcpf-switch-" + std::to_string(sw.index);
    output << "    <g id=\"PD_" << xml_escape(object_id) << "\">\n";
    write_metadata(object_id, class_name, psr_type,
                   {{"source_index", std::to_string(sw.index)},
                    {"name", sw.name},
                    {"in_service", bool_text(sw.in_service)},
                    {"closed", bool_text(sw.closed)},
                    {"normal_closed", bool_text(sw.normal_closed)},
                    {"role", switch_role_str(sw.role)},
                    {"controlled_element_type", sw.controlled_element_type},
                    {"controlled_element_index",
                     std::to_string(sw.controlled_element_index)},
                    {"controlled_branch_index",
                     std::to_string(sw.controlled_branch_index)},
                    {"protection_zone_id",
                     std::to_string(sw.protection_zone_id)},
                    {"upstream_protective_switch_index",
                     std::to_string(sw.upstream_protective_switch_index)},
                    {"binding_inferred", bool_text(sw.binding_inferred)},
                    {"binding_source", sw.binding_source}});
    const Point center = write_device_span(
        sw.closed ? "#Switch" : "#Switch@1", *from, *to);
    output << "      <line x1=\"" << svg_number(from->x) << "\" y1=\""
           << svg_number(from->y) << "\" x2=\"" << svg_number(to->x)
           << "\" y2=\"" << svg_number(to->y) << "\""
           << (sw.closed ? "" : " stroke-dasharray=\"7 5\"") << "/>\n"
           << "      <rect x=\"" << svg_number(center.x - 5.0)
           << "\" y=\"" << svg_number(center.y - 5.0)
           << "\" width=\"10\" height=\"10\"/>\n    </g>\n";
    labels.push_back({object_id, {center.x + 8.0, center.y - 8.0},
                      sw.name.empty() ? "Switch " + std::to_string(sw.index)
                                      : sw.name});
    ++result.exported_switches;
  }
  output << "  </g>\n";

  std::unordered_map<int, std::vector<std::size_t>> loads_by_bus;
  for (std::size_t i = 0; i < system.ac.loads.size(); ++i)
    loads_by_bus[system.ac.loads[i].bus].push_back(i);
  std::vector<bool> embedded_load(system.ac.loads.size(), false);

  output << "  <g id=\"PowerTransformer_Layer\" stroke=\"#7c3aed\" stroke-width=\"2\" fill=\"none\">\n";
  for (const auto& transformer : system.ac.transformers_2w) {
    const auto hv = point_of(transformer.hv_bus);
    const auto lv = point_of(transformer.lv_bus);
    if (!hv || !lv) {
      result.warnings.push_back(
          "Skipped transformer " + std::to_string(transformer.index) +
          " because a terminal bus is missing from the AC index space.");
      continue;
    }
    double load_p = 0.0;
    double load_q = 0.0;
    double load_sn = 0.0;
    int load_source_index = -1;
    bool load_in_service = true;
    std::string load_name;
    const auto at_lv = loads_by_bus.find(transformer.lv_bus);
    if (at_lv != loads_by_bus.end()) {
      for (std::size_t position : at_lv->second) {
        if (embedded_load[position]) continue;
        const auto& load = system.ac.loads[position];
        embedded_load[position] = true;
        ++result.embedded_loads;
        load_p += load.p_mw;
        load_q += load.q_mvar;
        load_sn += load.sn_mva;
        load_in_service = load_in_service && load.in_service;
        if (load_source_index < 0) load_source_index = load.index;
        if (!load.name.empty()) {
          if (!load_name.empty()) load_name += " + ";
          load_name += load.name;
        }
      }
    }
    const std::string object_id =
        "hacdcpf-transformer-" + std::to_string(transformer.index);
    Attributes attributes{
        {"source_index", std::to_string(transformer.index)},
        {"name", transformer.name},
        {"in_service", bool_text(transformer.in_service)},
        {"terminal_a_is_hv", "true"},
        {"std_type", transformer.std_type},
        {"sn_mva", svg_number(transformer.sn_mva)},
        {"vn_hv_kv", svg_number(transformer.vn_hv_kv)},
        {"vn_lv_kv", svg_number(transformer.vn_lv_kv)},
        {"vk_percent", svg_number(transformer.vk_percent)},
        {"vkr_percent", svg_number(transformer.vkr_percent)},
        {"pk_kw", svg_number(transformer.pk_kw)},
        {"pfe_kw", svg_number(transformer.pfe_kw)},
        {"i0_percent", svg_number(transformer.i0_percent)},
        {"vector_group", transformer.vector_group},
        {"z0_percent", svg_number(transformer.z0_percent)},
        {"x0_r0", svg_number(transformer.x0_r0)},
        {"tap_side", std::to_string(transformer.tap_side)},
        {"tap_pos", std::to_string(transformer.tap_pos)},
        {"tap_min", std::to_string(transformer.tap_min)},
        {"tap_max", std::to_string(transformer.tap_max)},
        {"tap_neutral", std::to_string(transformer.tap_neutral)},
        {"tap_step_percent", svg_number(transformer.tap_step_percent)},
        {"shift_deg", svg_number(transformer.shift_deg)},
        {"n_parallel", std::to_string(transformer.n_parallel)},
        {"source_branch_idx", std::to_string(transformer.source_branch_idx)}};
    if (load_source_index >= 0) {
      attributes.insert(attributes.end(),
                        {{"load_source_index", std::to_string(load_source_index)},
                         {"load_name", load_name},
                         {"load_in_service", bool_text(load_in_service)},
                         {"load_p_mw", svg_number(load_p)},
                         {"load_q_mvar", svg_number(load_q)},
                         {"load_sn_mva", svg_number(load_sn)}});
    }
    output << "    <g id=\"PD_" << xml_escape(object_id) << "\">\n";
    write_metadata(object_id, "PWOPTransformerPSR", "0301001", attributes);
    const Point center = write_device_span("#Transformer", *hv, *lv);
    output << "      <line x1=\"" << svg_number(hv->x) << "\" y1=\""
           << svg_number(hv->y) << "\" x2=\"" << svg_number(lv->x)
           << "\" y2=\"" << svg_number(lv->y) << "\"/>\n"
           << "      <circle cx=\"" << svg_number(center.x - 4.0)
           << "\" cy=\"" << svg_number(center.y)
           << "\" r=\"6\"/><circle cx=\"" << svg_number(center.x + 4.0)
           << "\" cy=\"" << svg_number(center.y)
           << "\" r=\"6\"/>\n    </g>\n";
    labels.push_back({object_id, {center.x + 10.0, center.y - 10.0},
                      (transformer.name.empty()
                           ? "Transformer " + std::to_string(transformer.index)
                           : transformer.name) +
                          " " + svg_number(transformer.sn_mva * 1000.0)});
    ++result.exported_transformers;
  }
  output << "  </g>\n";

  result.omitted_loads = static_cast<std::size_t>(std::count(
      embedded_load.begin(), embedded_load.end(), false));
  if (result.omitted_loads > 0)
    result.warnings.push_back(
        std::to_string(result.omitted_loads) +
        " AC load(s) are not on an exported transformer LV bus and cannot be "
        "represented by this IEC-CGE SVG profile.");

  output << "  <g id=\"Substation_Layer\" stroke=\"#dc2626\" stroke-width=\"2\" fill=\"#fee2e2\">\n";
  for (const auto& grid : system.ac.external_grids) {
    const auto point = point_of(grid.bus);
    if (!point) {
      result.warnings.push_back(
          "Skipped external grid " + std::to_string(grid.index) +
          " because its AC bus is missing.");
      continue;
    }
    const std::string object_id =
        "hacdcpf-external-grid-" + std::to_string(grid.index);
    output << "    <g id=\"PD_" << xml_escape(object_id) << "\">\n";
    write_metadata(object_id, "Substation", "0201001",
                   {{"source_index", std::to_string(grid.index)},
                    {"name", grid.name},
                    {"in_service", bool_text(grid.in_service)},
                    {"vn_kv", svg_number(grid.vn_kv)},
                    {"s_sc_max_mva", svg_number(grid.s_sc_max_mva)},
                    {"s_sc_min_mva", svg_number(grid.s_sc_min_mva)},
                    {"rx_max", svg_number(grid.rx_max)},
                    {"rx_min", svg_number(grid.rx_min)},
                    {"r_pu", svg_number(grid.r_pu)},
                    {"x_pu", svg_number(grid.x_pu)},
                    {"r0_pu", svg_number(grid.r0_pu)},
                    {"x0_pu", svg_number(grid.x0_pu)}});
    output << "      <use xlink:href=\"#Substation\" width=\"1\" height=\"1\" transform=\"scale(1) translate("
           << svg_number(point->x) << ' ' << svg_number(point->y)
           << ")\" opacity=\"0\"/>\n"
           << "      <polygon points=\"" << svg_number(point->x) << ','
           << svg_number(point->y - 12.0) << ' '
           << svg_number(point->x - 12.0) << ',' << svg_number(point->y + 10.0)
           << ' ' << svg_number(point->x + 12.0) << ','
           << svg_number(point->y + 10.0) << "\"/>\n    </g>\n";
    labels.push_back({object_id, {point->x + 14.0, point->y + 14.0},
                      grid.name.empty()
                          ? "External grid " + std::to_string(grid.index)
                          : grid.name});
    ++result.exported_external_grids;
  }
  output << "  </g>\n";

  if (result.exported_external_grids > 1)
    result.warnings.push_back(
        "The bounded SVG importer restores at most one external-grid equivalent "
        "per connected component; parallel sources require JSON/CIM for a "
        "lossless round trip.");

  output << "  <g id=\"Text_Layer\" fill=\"#111827\" font-family=\"sans-serif\" font-size=\"11\">\n";
  for (const auto& label : labels) {
    output << "    <g id=\"TXT-PD_" << xml_escape(label.object_id)
           << "\"><text x=\"" << svg_number(label.point.x) << "\" y=\""
           << svg_number(label.point.y) << "\">" << xml_escape(label.text)
           << "</text></g>\n";
  }
  output << "  </g>\n</svg>\n";

  result.omitted_non_ac_components =
      system.dc.buses.size() + system.dc.branches.size() +
      system.dc.loads.size() + system.vsc_converters.size() +
      system.dc.dcdc_converters.size();
  if (result.omitted_non_ac_components > 0)
    result.warnings.push_back(
        std::to_string(result.omitted_non_ac_components) +
        " DC/converter component(s) are outside the AC IEC-CGE distribution "
        "SVG export profile; use rich JSON or CIM for those assets.");
  result.svg = output.str();
  return result;
}

SvgDistributionExportResult save_svg_distribution(
    const HybridPowerSystem& system,
    const std::filesystem::path& path,
    const SvgDistributionExportOptions& opts) {
  auto result = to_svg_distribution(system, opts);
  std::ofstream output(path, std::ios::binary);
  if (!output)
    throw std::runtime_error("Cannot open SVG distribution export path: " +
                             path_utf8(path));
  output.write(result.svg.data(), static_cast<std::streamsize>(result.svg.size()));
  if (!output)
    throw std::runtime_error("Failed to write SVG distribution export: " +
                             path_utf8(path));
  return result;
}

}  // namespace hacdcpf::io
