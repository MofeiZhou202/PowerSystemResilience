#include "hacdcpf/io/svg_distribution_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hacdcpf::io {
namespace {

constexpr std::size_t knpos = std::string::npos;
constexpr double kPi = 3.14159265358979323846;

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
      !(opts.default_transformer_sn_mva > 0.0) ||
      !(opts.default_transformer_vk_percent > 0.0) ||
      !(opts.default_transformer_vkr_percent >= 0.0) ||
      opts.default_transformer_vkr_percent >
          opts.default_transformer_vk_percent ||
      !(opts.transformer_load_factor >= 0.0) ||
      !(opts.power_factor > 0.0 && opts.power_factor <= 1.0)) {
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
  std::vector<Point> substations;

  for (const auto& object : objects) {
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
      substations.push_back(object.center);
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
  int bus_index = 0;
  for (int root : used_roots) {
    ACBus bus;
    bus.index = ++bus_index;
    bus.base_kv = opts.nominal_mv_kv;
    bus.name = "SVG node " + std::to_string(bus.index);
    result.system.ac.buses.push_back(bus);
    root_to_bus[root] = bus.index;
  }
  result.recovered_nodes = result.system.ac.buses.size();
  auto bus_of = [&](int point_id) { return root_to_bus.at(pool.root(point_id)); };

  int branch_index = 0;
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
      branch.index = ++branch_index;
      branch.from_bus = from;
      branch.to_bus = to;
      branch.name = object_name(*line.object);
      if (line.point_ids.size() > 2)
        branch.name += " [" + std::to_string(i) + "]";
      branch.length_km = std::max(1e-4, svg_length * opts.km_per_svg_unit);
      branch.r_ohm_per_km = cable ? opts.cable_r_ohm_per_km
                                  : opts.overhead_r_ohm_per_km;
      branch.x_ohm_per_km = cable ? opts.cable_x_ohm_per_km
                                  : opts.overhead_x_ohm_per_km;
      const double z_base = opts.nominal_mv_kv * opts.nominal_mv_kv /
                            opts.base_mva;
      branch.r_pu = branch.r_ohm_per_km * branch.length_km / z_base;
      branch.x_pu = branch.x_ohm_per_km * branch.length_km / z_base;
      branch.rate_a_mva = opts.default_line_rate_mva;
      branch.rate_b_mva = branch.rate_a_mva;
      branch.rate_c_mva = branch.rate_a_mva;
      branch.line_type = cable ? "cable" : "overhead";
      branch.parameter_source = "svg-geometry-default";
      branch.parameters_inferred = true;
      result.system.ac.branches.push_back(std::move(branch));
    }
  }

  int switch_index = 0;
  for (const auto& ref : switches) {
    const int from = bus_of(ref.terminal_a);
    const int to = bus_of(ref.terminal_b);
    if (from == to) continue;
    Switch sw;
    sw.index = ++switch_index;
    sw.name = object_name(*ref.object);
    sw.bus_from = from;
    sw.bus_to = to;
    sw.switch_type = switch_type(*ref.object);
    sw.closed = ref.object->href.find("@1") == knpos;
    sw.normal_closed = sw.closed;
    sw.normal_state_explicit = true;
    sw.role = switch_role(sw.switch_type, sw.closed);
    sw.binding_inferred = true;
    sw.binding_source = "svg-symbol-state-and-geometry";
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

  int transformer_index = 0;
  int load_index = 0;
  const double q_ratio =
      std::tan(std::acos(std::clamp(opts.power_factor, 1e-6, 1.0)));
  for (const auto& ref : transformers) {
    int hv_bus = bus_of(ref.terminal_a);
    int lv_bus = bus_of(ref.terminal_b);
    if (mv_degree[lv_bus] > mv_degree[hv_bus]) std::swap(hv_bus, lv_bus);
    if (hv_bus == lv_bus) continue;
    for (auto& bus : result.system.ac.buses)
      if (bus.index == lv_bus) bus.base_kv = opts.nominal_lv_kv;

    const double sn_mva = transformer_capacity_mva(
        *ref.object, opts.default_transformer_sn_mva);
    Transformer2W transformer;
    transformer.index = ++transformer_index;
    transformer.name = object_name(*ref.object);
    transformer.hv_bus = hv_bus;
    transformer.lv_bus = lv_bus;
    transformer.sn_mva = sn_mva;
    transformer.vn_hv_kv = opts.nominal_mv_kv;
    transformer.vn_lv_kv = opts.nominal_lv_kv;
    transformer.vk_percent = opts.default_transformer_vk_percent;
    transformer.vkr_percent = opts.default_transformer_vkr_percent;
    transformer.i0_percent = 0.5;
    result.system.ac.transformers_2w.push_back(std::move(transformer));

    Load load;
    load.index = ++load_index;
    load.bus = lv_bus;
    load.name = object_name(*ref.object) + " estimated load";
    load.sn_mva = sn_mva * opts.transformer_load_factor;
    load.p_mw = load.sn_mva * opts.power_factor;
    load.q_mvar = load.p_mw * q_ratio;
    result.system.ac.loads.push_back(std::move(load));
  }

  // Find connected components over in-service lines, closed switches and
  // transformers. Only substation-bearing components receive a source; if no
  // substation symbol exists, the largest component is the explicit fallback.
  DisjointSet electrical;
  for (std::size_t i = 0; i < result.system.ac.buses.size(); ++i) electrical.add();
  auto bus_pos = [](int bus) { return bus - 1; };
  for (const auto& branch : result.system.ac.branches)
    electrical.unite(bus_pos(branch.from_bus), bus_pos(branch.to_bus));
  for (const auto& sw : result.system.ac.switches)
    if (sw.closed) electrical.unite(bus_pos(sw.bus_from), bus_pos(sw.bus_to));
  for (const auto& transformer : result.system.ac.transformers_2w)
    electrical.unite(bus_pos(transformer.hv_bus), bus_pos(transformer.lv_bus));

  std::map<int, std::vector<int>> components;
  for (const auto& bus : result.system.ac.buses)
    components[electrical.find(bus_pos(bus.index))].push_back(bus.index);
  std::set<int> preferred_source_buses;
  for (Point substation : substations) {
    double best = std::numeric_limits<double>::infinity();
    int best_bus = 0;
    for (const auto& [root, bus] : root_to_bus) {
      const double candidate = distance(substation, pool.points()[root]);
      if (candidate < best) {
        best = candidate;
        best_bus = bus;
      }
    }
    if (best_bus != 0) preferred_source_buses.insert(best_bus);
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

  int grid_index = 0;
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
      grid.index = ++grid_index;
      grid.bus = source_bus;
      grid.name = "SVG synthetic source " + std::to_string(grid.index);
      grid.vn_kv = opts.nominal_mv_kv;
      grid.s_sc_max_mva = 100.0;
      grid.s_sc_min_mva = 50.0;
      result.system.ac.external_grids.push_back(std::move(grid));
      for (auto& bus : result.system.ac.buses)
        if (bus.index == source_bus) bus.bus_type = BusType::SLACK;
    }
  }
  result.synthetic_external_grids = result.system.ac.external_grids.size();
  for (const auto& [component, buses] : components) {
    if (source_components.count(component) != 0) continue;
    ++result.isolated_components;
    result.isolated_buses += buses.size();
    for (auto& bus : result.system.ac.buses) {
      if (std::find(buses.begin(), buses.end(), bus.index) != buses.end())
        bus.bus_type = BusType::ISOLATED;
    }
  }

  result.report.add(
      ImportDisposition::Accepted, ImportReasonCode::Ok,
      ImportSeverity::Info, "svg/cge:psr_ref",
      "Recovered " + std::to_string(result.source_line_objects) +
          " line, " + std::to_string(result.source_switch_objects) +
          " switch, and " +
          std::to_string(result.source_transformer_objects) +
          " transformer objects from IEC-CGE metadata");
  add_warning(result, ImportReasonCode::UnitInferred, "svg/geometry",
              "Electrical connectivity was recovered from SVG coordinates; "
              "the source does not provide CIM Terminal/ConnectivityNode links.");
  add_warning(result, ImportReasonCode::UnitInferred, "svg/line-parameters",
              "Line lengths and impedances were estimated from drawing geometry "
              "and configurable overhead/cable defaults.");
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
                      ImportSeverity::Error, path.string(),
                      "Cannot open SVG distribution file");
    return result;
  }
  std::string svg((std::istreambuf_iterator<char>(input)),
                  std::istreambuf_iterator<char>());
  auto options = opts;
  if (options.model_name == "SVG distribution feeder")
    options.model_name = path.stem().string();
  return from_svg_distribution(svg, mode, options);
}

}  // namespace hacdcpf::io
