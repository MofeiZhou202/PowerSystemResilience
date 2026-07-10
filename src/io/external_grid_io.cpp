#include "hacdcpf/io/external_grid_io.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hacdcpf/model/unit_conversion.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::io {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt3 = 1.7320508075688772935;
constexpr double kMileToKm = 1.609344;
constexpr double kFootToKm = 0.0003048;
constexpr double kTiny = 1e-12;

std::string trim(std::string value) {
  auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

bool starts_with_icase(const std::string& value, const std::string& prefix) {
  if (value.size() < prefix.size()) return false;
  return ascii_lower(value.substr(0, prefix.size())) == ascii_lower(prefix);
}

std::string strip_quotes(std::string value) {
  value = trim(std::move(value));
  if (value.size() >= 2U) {
    const char a = value.front();
    const char b = value.back();
    if ((a == '"' && b == '"') || (a == '\'' && b == '\'')) {
      value = value.substr(1, value.size() - 2U);
    }
  }
  return value;
}

std::string strip_brackets(std::string value) {
  value = strip_quotes(std::move(value));
  if (value.size() >= 2U && value.front() == '[' && value.back() == ']') {
    value = value.substr(1, value.size() - 2U);
  }
  return trim(std::move(value));
}

std::string sanitize_identifier(std::string raw, const std::string& fallback) {
  raw = strip_quotes(trim(std::move(raw)));
  if (raw.empty()) raw = fallback;
  std::string out;
  out.reserve(raw.size() + 2U);
  for (unsigned char ch : raw) {
    if (std::isalnum(ch) || ch == '_') {
      out.push_back(static_cast<char>(ch));
    } else {
      out.push_back('_');
    }
  }
  while (!out.empty() && out.front() == '_') out.erase(out.begin());
  if (out.empty()) out = fallback;
  if (std::isdigit(static_cast<unsigned char>(out.front()))) {
    out.insert(out.begin(), 'n');
  }
  return out;
}

std::string unique_identifier(std::string raw,
                              const std::string& fallback,
                              std::unordered_set<std::string>& used) {
  std::string base = sanitize_identifier(std::move(raw), fallback);
  std::string candidate = base;
  int suffix = 2;
  while (used.count(ascii_lower(candidate)) != 0U) {
    candidate = base + "_" + std::to_string(suffix++);
  }
  used.insert(ascii_lower(candidate));
  return candidate;
}

std::string format_double(double value) {
  std::ostringstream os;
  os << std::setprecision(15) << value;
  return os.str();
}

double positive_or(double value, double fallback) {
  return value > kTiny ? value : fallback;
}

double bus_base_kv(const ACBus& bus, double minimum_base_kv) {
  return std::max(minimum_base_kv, bus.base_kv > 0.0 ? bus.base_kv : 12.47);
}

double system_base_mva(const HybridPowerSystem& sys) {
  if (sys.ac.base_mva > kTiny) return sys.ac.base_mva;
  if (sys.base_mva > kTiny) return sys.base_mva;
  return 100.0;
}

std::string read_text_file_or_throw(const std::filesystem::path& path,
                                    const std::string& label) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot open " + label + ": " + path.string());
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void write_text_file_or_throw(const std::filesystem::path& path,
                              const std::string& text,
                              const std::string& label) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("Cannot write " + label + ": " + path.string());
  out << text;
}

std::filesystem::path temp_export_dir(const std::string& prefix) {
  static std::atomic<unsigned long long> counter{1};
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("hacdcpf_" + prefix + "_" + std::to_string(now) + "_" +
          std::to_string(counter.fetch_add(1)));
}

double unit_to_km(std::string unit, double default_value = 1.0) {
  unit = ascii_lower(trim(std::move(unit)));
  if (unit.empty()) return default_value;
  if (unit == "km" || unit == "kilometer" || unit == "kilometers") return 1.0;
  if (unit == "m" || unit == "meter" || unit == "meters") return 0.001;
  if (unit == "mi" || unit == "mile" || unit == "miles") return kMileToKm;
  if (unit == "ft" || unit == "foot" || unit == "feet") return kFootToKm;
  if (unit == "kft" || unit == "kfeet") return 1000.0 * kFootToKm;
  return default_value;
}

std::string unit_word_from_value(const std::string& value) {
  std::string text = ascii_lower(value);
  for (char& ch : text) {
    if (ch == ';' || ch == '"' || ch == '\'') ch = ' ';
  }
  std::stringstream ss(text);
  std::string token;
  while (ss >> token) {
    if (token == "ohm/mile" || token == "ohm/mi") return "mile";
    if (token == "ohm/km") return "km";
    if (token == "ohm/ft") return "ft";
    if (token == "ohm/kft") return "kft";
    if (token == "mile" || token == "miles" || token == "mi" ||
        token == "km" || token == "m" || token == "ft" || token == "kft") {
      return token;
    }
  }
  return {};
}

double parse_number(const std::string& raw, double fallback = 0.0) {
  std::string value = strip_quotes(trim(raw));
  if (!value.empty() && value.front() == '[') value.erase(value.begin());
  if (!value.empty() && value.back() == ']') value.pop_back();
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    (void)consumed;
    return parsed;
  } catch (const std::exception&) {
    return fallback;
  }
}

std::complex<double> parse_complex_rect(std::string raw) {
  raw = strip_quotes(trim(std::move(raw)));
  const auto unit_pos = raw.find(' ');
  if (unit_pos != std::string::npos) raw = raw.substr(0, unit_pos);
  raw.erase(std::remove_if(raw.begin(), raw.end(),
                           [](unsigned char ch) {
                             return std::isspace(ch) != 0 || ch == '(' ||
                                    ch == ')';
                           }),
            raw.end());
  if (!raw.empty() && raw.back() == ';') raw.pop_back();
  const auto j_pos = raw.find_first_of("jJ");
  if (j_pos == std::string::npos) {
    return {parse_number(raw), 0.0};
  }
  raw = raw.substr(0, j_pos);
  std::size_t sign_pos = std::string::npos;
  for (std::size_t i = 1; i < raw.size(); ++i) {
    if ((raw[i] == '+' || raw[i] == '-') && raw[i - 1] != 'e' &&
        raw[i - 1] != 'E') {
      sign_pos = i;
    }
  }
  if (sign_pos == std::string::npos) {
    return {0.0, parse_number(raw)};
  }
  const double real = parse_number(raw.substr(0, sign_pos));
  const double imag = parse_number(raw.substr(sign_pos));
  return {real, imag};
}

std::vector<std::string> split_list(std::string raw) {
  raw = strip_brackets(std::move(raw));
  std::vector<std::string> out;
  std::string token;
  bool quoted = false;
  for (char ch : raw) {
    if (ch == '"' || ch == '\'') {
      quoted = !quoted;
      token.push_back(ch);
      continue;
    }
    if (!quoted && (ch == ',' || std::isspace(static_cast<unsigned char>(ch)))) {
      if (!trim(token).empty()) {
        out.push_back(strip_quotes(trim(token)));
        token.clear();
      }
      continue;
    }
    token.push_back(ch);
  }
  if (!trim(token).empty()) out.push_back(strip_quotes(trim(token)));
  return out;
}

std::string strip_bus_phases(std::string bus) {
  bus = strip_quotes(trim(std::move(bus)));
  if (bus.empty()) return bus;
  const auto dot = bus.find('.');
  if (dot == std::string::npos) return bus;
  const std::string suffix = bus.substr(dot + 1U);
  if (!suffix.empty() &&
      std::all_of(suffix.begin(), suffix.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0 || ch == '.';
      })) {
    return bus.substr(0, dot);
  }
  return bus;
}

std::string dss_bus_ref(const std::string& bus_name) {
  return sanitize_identifier(bus_name, "bus") + ".1.2.3";
}

std::vector<std::string> tokenize_statement(const std::string& statement) {
  std::vector<std::string> tokens;
  std::string token;
  bool quoted = false;
  int bracket_depth = 0;
  for (char ch : statement) {
    if (ch == '"' || ch == '\'') {
      quoted = !quoted;
      token.push_back(ch);
      continue;
    }
    if (!quoted) {
      if (ch == '[' || ch == '(') ++bracket_depth;
      if ((ch == ']' || ch == ')') && bracket_depth > 0) --bracket_depth;
      if (std::isspace(static_cast<unsigned char>(ch)) && bracket_depth == 0) {
        if (!token.empty()) {
          tokens.push_back(token);
          token.clear();
        }
        continue;
      }
    }
    token.push_back(ch);
  }
  if (!token.empty()) tokens.push_back(token);
  return tokens;
}

std::unordered_map<std::string, std::string> parse_key_values(
    const std::vector<std::string>& tokens,
    std::size_t first_key_token) {
  std::unordered_map<std::string, std::string> out;
  for (std::size_t i = first_key_token; i < tokens.size(); ++i) {
    std::string token = tokens[i];
    auto eq = token.find('=');
    if (eq == std::string::npos) {
      if (i + 1U < tokens.size() && tokens[i + 1U] == "=" &&
          i + 2U < tokens.size()) {
        out[ascii_lower(token)] = strip_quotes(tokens[i + 2U]);
        i += 2U;
      }
      continue;
    }
    std::string key = ascii_lower(trim(token.substr(0, eq)));
    std::string value = token.substr(eq + 1U);
    if (value.empty() && i + 1U < tokens.size()) {
      value = tokens[++i];
    }
    out[key] = strip_quotes(trim(value));
  }
  return out;
}

std::string get_value(const std::unordered_map<std::string, std::string>& kv,
                      const std::string& key,
                      const std::string& fallback = {}) {
  const auto it = kv.find(ascii_lower(key));
  return it == kv.end() ? fallback : it->second;
}

double get_number(const std::unordered_map<std::string, std::string>& kv,
                  const std::string& key,
                  double fallback = 0.0) {
  const auto it = kv.find(ascii_lower(key));
  return it == kv.end() ? fallback : parse_number(it->second, fallback);
}

void add_warning_or_throw(ExternalGridImportReport& report,
                          const ExternalGridImportOptions& options,
                          const std::string& message) {
  if (options.mode == ImportMode::Strict) {
    throw std::runtime_error(message);
  }
  report.warnings.push_back(message);
}

class BusBuilder {
 public:
  BusBuilder(HybridPowerSystem& sys, double default_base_kv)
      : sys_(sys), default_base_kv_(default_base_kv) {}

  int ensure(std::string name,
             double base_kv = 0.0,
             BusType type = BusType::PQ,
             double vm_pu = 1.0,
             double va_deg = 0.0) {
    name = strip_bus_phases(strip_quotes(trim(std::move(name))));
    if (name.empty()) name = "bus_" + std::to_string(sys_.ac.buses.size() + 1U);
    const std::string key = ascii_lower(name);
    auto it = by_name_.find(key);
    if (it != by_name_.end()) {
      auto& bus = sys_.ac.buses[static_cast<std::size_t>(it->second - 1)];
      if (base_kv > kTiny) bus.base_kv = base_kv;
      if (type == BusType::SLACK) bus.bus_type = BusType::SLACK;
      if (vm_pu > kTiny) bus.vm_pu = vm_pu;
      bus.va_deg = va_deg;
      return bus.index;
    }

    ACBus bus;
    bus.index = static_cast<int>(sys_.ac.buses.size()) + 1;
    bus.name = name;
    bus.bus_type = type;
    bus.base_kv = positive_or(base_kv, default_base_kv_);
    bus.vm_pu = positive_or(vm_pu, 1.0);
    bus.va_deg = va_deg;
    bus.in_service = true;
    sys_.ac.buses.push_back(bus);
    by_name_[key] = bus.index;
    return bus.index;
  }

  std::optional<int> find(const std::string& name) const {
    const auto it = by_name_.find(ascii_lower(strip_bus_phases(name)));
    if (it == by_name_.end()) return std::nullopt;
    return it->second;
  }

  double base_kv(int bus_index) const {
    for (const auto& bus : sys_.ac.buses) {
      if (bus.index == bus_index) return positive_or(bus.base_kv, default_base_kv_);
    }
    return default_base_kv_;
  }

 private:
  HybridPowerSystem& sys_;
  double default_base_kv_{12.47};
  std::unordered_map<std::string, int> by_name_;
};

void ensure_single_slack_generator(HybridPowerSystem& sys,
                                   int bus,
                                   double vm_pu,
                                   double va_deg,
                                   bool create) {
  if (bus == 0) return;
  for (auto& b : sys.ac.buses) {
    if (b.index == bus) {
      b.bus_type = BusType::SLACK;
      b.vm_pu = positive_or(vm_pu, 1.0);
      b.va_deg = va_deg;
      break;
    }
  }
  if (!create) return;
  for (auto& gen : sys.ac.generators) {
    if (gen.bus == bus && gen.is_slack) {
      gen.vg_pu = positive_or(vm_pu, 1.0);
      return;
    }
  }
  Generator gen;
  gen.index = static_cast<int>(sys.ac.generators.size()) + 1;
  gen.bus = bus;
  gen.name = "Source";
  gen.is_slack = true;
  gen.in_service = true;
  gen.vg_pu = positive_or(vm_pu, 1.0);
  gen.pmax_mw = 1.0e6;
  gen.pmin_mw = -1.0e6;
  gen.qmax_mvar = 1.0e6;
  gen.qmin_mvar = -1.0e6;
  (void)va_deg;
  sys.ac.generators.push_back(gen);
}

std::unordered_map<int, const ACBus*> bus_by_index(const ACSystem& ac) {
  std::unordered_map<int, const ACBus*> out;
  for (const auto& bus : ac.buses) out[bus.index] = &bus;
  return out;
}

int find_slack_bus(const ACSystem& ac) {
  for (const auto& bus : ac.buses) {
    if (bus.in_service && bus.bus_type == BusType::SLACK) return bus.index;
  }
  for (const auto& gen : ac.generators) {
    if (gen.in_service && gen.is_slack) return gen.bus;
  }
  for (const auto& eg : ac.external_grids) {
    if (eg.in_service) return eg.bus;
  }
  return ac.buses.empty() ? 0 : ac.buses.front().index;
}

std::unordered_map<int, std::pair<double, double>> slack_voltage_by_bus(
    const ACSystem& ac) {
  std::unordered_map<int, std::pair<double, double>> out;
  for (const auto& bus : ac.buses) {
    if (bus.bus_type == BusType::SLACK) out[bus.index] = {bus.vm_pu, bus.va_deg};
  }
  for (const auto& eg : ac.external_grids) {
    if (eg.in_service) out[eg.bus] = {eg.vm_pu, eg.va_deg};
  }
  for (const auto& gen : ac.generators) {
    if (gen.in_service && gen.is_slack) {
      out[gen.bus] = {positive_or(gen.vg_pu, 1.0), 0.0};
    }
  }
  return out;
}

struct BusLoad {
  double p_mw{0.0};
  double q_mvar{0.0};
};

std::map<int, BusLoad> aggregate_bus_demand(const HybridPowerSystem& sys) {
  std::map<int, BusLoad> out;
  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service) continue;
    out[bus.index].p_mw += bus.pd_mw;
    out[bus.index].q_mvar += bus.qd_mvar;
  }
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    const double scale = load.scaling > 0.0 ? load.scaling : 1.0;
    out[load.bus].p_mw += scale * load.p_mw;
    out[load.bus].q_mvar += scale * load.q_mvar;
  }
  return out;
}

std::string strip_dss_comment(const std::string& line) {
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"' || ch == '\'') quoted = !quoted;
    if (!quoted && ch == '!') return line.substr(0, i);
    if (!quoted && ch == '/' && i + 1U < line.size() && line[i + 1U] == '/') {
      return line.substr(0, i);
    }
  }
  return line;
}

std::vector<std::string> preprocess_dss_statements(const std::string& text) {
  std::vector<std::string> statements;
  std::string current;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    line = trim(strip_dss_comment(line));
    if (line.empty()) continue;
    if (!line.empty() && line.front() == '~') {
      current += " " + trim(line.substr(1U));
      continue;
    }
    if (!current.empty()) statements.push_back(current);
    current = line;
  }
  if (!current.empty()) statements.push_back(current);
  return statements;
}

struct DssObjectId {
  std::string cls;
  std::string name;
};

std::optional<DssObjectId> parse_dss_object_id(const std::string& token) {
  const auto dot = token.find('.');
  if (dot == std::string::npos) {
    return DssObjectId{ascii_lower(token), token};
  }
  return DssObjectId{ascii_lower(token.substr(0, dot)), token.substr(dot + 1U)};
}

void initialise_import_system(HybridPowerSystem& sys,
                              const ExternalGridImportOptions& options,
                              const std::string& name) {
  sys.name = name;
  sys.base_mva = positive_or(options.default_base_mva, 100.0);
  sys.ac.name = "Imported AC System";
  sys.ac.base_mva = sys.base_mva;
  sys.ac.freq_hz = positive_or(options.default_frequency_hz, 50.0);
  sys.dc.base_mva = sys.base_mva;
}

ExternalGridImportReport finalise_import(ExternalGridImportReport report,
                                         const ExternalGridImportOptions& options) {
  if (options.convert_actual_to_per_unit) {
    convert_actual_to_per_unit(report.system);
  }
  if (!report.system.ac.buses.empty()) {
    const bool has_slack = std::any_of(
        report.system.ac.buses.begin(), report.system.ac.buses.end(),
        [](const ACBus& bus) { return bus.bus_type == BusType::SLACK; });
    if (!has_slack) {
      report.system.ac.buses.front().bus_type = BusType::SLACK;
      report.warnings.push_back(
          "No explicit source/slack bus was found; the first imported AC bus "
          "was marked as SLACK.");
      if (options.create_slack_generator) {
        ensure_single_slack_generator(report.system, report.system.ac.buses.front().index,
                                      report.system.ac.buses.front().vm_pu,
                                      report.system.ac.buses.front().va_deg, true);
      }
    }
  }
  return report;
}

void import_opendss_line(ExternalGridImportReport& report,
                         BusBuilder& buses,
                         const DssObjectId& id,
                         const std::unordered_map<std::string, std::string>& kv,
                         const OpenDSSImportOptions& options) {
  const std::string bus1_name = strip_bus_phases(get_value(kv, "bus1"));
  const std::string bus2_name = strip_bus_phases(get_value(kv, "bus2"));
  if (bus1_name.empty() || bus2_name.empty()) {
    add_warning_or_throw(report, options,
                         "OpenDSS Line." + id.name + " is missing bus1 or bus2.");
    return;
  }
  const int from = buses.ensure(bus1_name);
  const int to = buses.ensure(bus2_name, buses.base_kv(from));
  const double length = positive_or(get_number(kv, "length", 1.0), 1.0);
  const std::string units = get_value(kv, "units", "km");
  const double unit_km = unit_to_km(units, 1.0);
  const double length_km = std::max(kTiny, length * unit_km);
  const double r1_per_unit = get_number(kv, "r1", get_number(kv, "rmatrix", 0.0));
  const double x1_per_unit = get_number(kv, "x1", get_number(kv, "xmatrix", 0.0));
  const double r0_per_unit = get_number(kv, "r0", r1_per_unit);
  const double x0_per_unit = get_number(kv, "x0", x1_per_unit);
  const double base_kv = positive_or(buses.base_kv(from), options.default_base_kv_ll);
  const double zbase = (base_kv * base_kv) / positive_or(report.system.ac.base_mva, 100.0);

  ACBranch br;
  br.index = static_cast<int>(report.system.ac.branches.size()) + 1;
  br.name = id.name;
  br.from_bus = from;
  br.to_bus = to;
  br.length_km = length_km;
  br.r_ohm_per_km = r1_per_unit / unit_km;
  br.x_ohm_per_km = x1_per_unit / unit_km;
  br.r_pu = (r1_per_unit * length) / zbase;
  br.x_pu = (x1_per_unit * length) / zbase;
  br.r0_pu = (r0_per_unit * length) / zbase;
  br.x0_pu = (x0_per_unit * length) / zbase;
  br.tap = 1.0;
  br.in_service = true;
  report.system.ac.branches.push_back(br);
}

void import_opendss_transformer(
    ExternalGridImportReport& report,
    BusBuilder& buses,
    const DssObjectId& id,
    const std::unordered_map<std::string, std::string>& kv,
    const OpenDSSImportOptions& options) {
  const auto bus_list = split_list(get_value(kv, "buses"));
  if (bus_list.size() < 2U) {
    add_warning_or_throw(report, options,
                         "OpenDSS Transformer." + id.name +
                             " is missing a two-winding buses=[...] list.");
    return;
  }
  const auto kvs = split_list(get_value(kv, "kvs"));
  const auto kvas = split_list(get_value(kv, "kvas"));
  const double hv_kv = kvs.empty() ? options.default_base_kv_ll
                                   : positive_or(parse_number(kvs[0]), options.default_base_kv_ll);
  const double lv_kv = kvs.size() < 2U ? hv_kv : positive_or(parse_number(kvs[1]), hv_kv);
  const double rating_mva =
      kvas.empty() ? report.system.ac.base_mva
                   : positive_or(parse_number(kvas[0]) / 1000.0, report.system.ac.base_mva);
  const int hv_bus = buses.ensure(bus_list[0], hv_kv);
  const int lv_bus = buses.ensure(bus_list[1], lv_kv);
  const double z_scale = positive_or(report.system.ac.base_mva, 100.0) /
                         positive_or(rating_mva, report.system.ac.base_mva);

  ACBranch br;
  br.index = static_cast<int>(report.system.ac.branches.size()) + 1;
  br.name = id.name;
  br.from_bus = hv_bus;
  br.to_bus = lv_bus;
  br.r_pu = (get_number(kv, "%r", get_number(kv, "r", 0.0)) / 100.0) * z_scale;
  br.x_pu = (get_number(kv, "xhl", get_number(kv, "x", 0.0)) / 100.0) * z_scale;
  br.tap = 1.0;
  br.in_service = true;
  br.rate_a_mva = rating_mva;
  br.vn_hv_kv = hv_kv;
  br.vn_lv_kv = lv_kv;
  br.sn_mva = rating_mva;
  report.system.ac.branches.push_back(br);
}

ExternalGridImportReport import_opendss_text(
    const std::string& dss_text,
    const OpenDSSImportOptions& options) {
  ExternalGridImportReport report;
  initialise_import_system(report.system, options, "Imported OpenDSS System");
  BusBuilder buses(report.system, positive_or(options.default_base_kv_ll, 12.47));

  for (const auto& statement : preprocess_dss_statements(dss_text)) {
    const auto tokens = tokenize_statement(statement);
    if (tokens.empty()) continue;
    const std::string verb = ascii_lower(tokens.front());
    if (verb == "set") {
      const auto kv = parse_key_values(tokens, 1U);
      const double base_mva = get_number(kv, "basemva", 0.0);
      if (base_mva > kTiny) {
        report.system.base_mva = base_mva;
        report.system.ac.base_mva = base_mva;
        report.system.dc.base_mva = base_mva;
      }
      const double freq = get_number(kv, "defaultbasefrequency", 0.0);
      if (freq > kTiny) report.system.ac.freq_hz = freq;
      continue;
    }
    if (verb == "clear" || verb == "solve" || verb == "calcv") {
      continue;
    }
    if (verb == "redirect" || verb == "compile") {
      report.skipped.push_back("External OpenDSS include skipped: " + statement);
      continue;
    }
    if (verb != "new" && verb != "edit") {
      report.skipped.push_back("Unsupported OpenDSS statement skipped: " + statement);
      continue;
    }
    if (tokens.size() < 2U) continue;
    const auto id = parse_dss_object_id(tokens[1]);
    if (!id.has_value()) continue;
    const auto kv = parse_key_values(tokens, 2U);

    if (id->cls == "circuit") {
      report.system.name = "OpenDSS " + id->name;
      const double base_kv =
          positive_or(get_number(kv, "basekv", options.default_base_kv_ll),
                      options.default_base_kv_ll);
      const double vm = positive_or(get_number(kv, "pu", 1.0), 1.0);
      const double va = get_number(kv, "angle", 0.0);
      const std::string bus_name =
          strip_bus_phases(get_value(kv, "bus1", "sourcebus"));
      const int bus = buses.ensure(bus_name, base_kv, BusType::SLACK, vm, va);
      ensure_single_slack_generator(report.system, bus, vm, va,
                                    options.create_slack_generator);
    } else if (id->cls == "vsource") {
      const double base_kv =
          positive_or(get_number(kv, "basekv", options.default_base_kv_ll),
                      options.default_base_kv_ll);
      const double vm = positive_or(get_number(kv, "pu", 1.0), 1.0);
      const double va = get_number(kv, "angle", 0.0);
      const std::string bus_name =
          strip_bus_phases(get_value(kv, "bus1", "sourcebus"));
      const int bus = buses.ensure(bus_name, base_kv, BusType::SLACK, vm, va);
      ensure_single_slack_generator(report.system, bus, vm, va,
                                    options.create_slack_generator);
    } else if (id->cls == "line") {
      import_opendss_line(report, buses, *id, kv, options);
    } else if (id->cls == "transformer") {
      import_opendss_transformer(report, buses, *id, kv, options);
    } else if (id->cls == "load") {
      const double kv_ll =
          positive_or(get_number(kv, "kv", options.default_base_kv_ll),
                      options.default_base_kv_ll);
      const int bus = buses.ensure(get_value(kv, "bus1"), kv_ll);
      Load load;
      load.index = static_cast<int>(report.system.ac.loads.size()) + 1;
      load.name = id->name;
      load.bus = bus;
      load.in_service = true;
      load.p_mw = get_number(kv, "kw", 0.0) / 1000.0;
      load.q_mvar = get_number(kv, "kvar", 0.0) / 1000.0;
      load.scaling = 1.0;
      report.system.ac.loads.push_back(load);
    } else if (id->cls == "generator" || id->cls == "pvsystem") {
      const double kv_ll =
          positive_or(get_number(kv, "kv", options.default_base_kv_ll),
                      options.default_base_kv_ll);
      const int bus = buses.ensure(get_value(kv, "bus1"), kv_ll);
      StaticGenerator gen;
      gen.index = static_cast<int>(report.system.ac.static_generators.size()) + 1;
      gen.name = id->name;
      gen.bus = bus;
      gen.in_service = true;
      gen.p_mw = get_number(kv, "kw", get_number(kv, "pmpp", 0.0)) / 1000.0;
      gen.q_mvar = get_number(kv, "kvar", 0.0) / 1000.0;
      gen.p_rated_mw = positive_or(gen.p_mw, 0.0);
      gen.sn_mva = positive_or(get_number(kv, "kva", 0.0) / 1000.0,
                               std::hypot(gen.p_mw, gen.q_mvar));
      report.system.ac.static_generators.push_back(gen);
    } else if (id->cls == "linecode" || id->cls == "loadshape") {
      report.skipped.push_back("OpenDSS " + id->cls + "." + id->name +
                               " was parsed as metadata only.");
    } else {
      report.skipped.push_back("Unsupported OpenDSS object skipped: " +
                               id->cls + "." + id->name);
    }
  }

  return finalise_import(std::move(report), options);
}

struct GlmObject {
  std::string cls;
  std::unordered_map<std::string, std::string> props;
};

std::string strip_glm_line_comment(const std::string& line) {
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"' || ch == '\'') quoted = !quoted;
    if (!quoted && ch == '/' && i + 1U < line.size() && line[i + 1U] == '/') {
      return line.substr(0, i);
    }
  }
  return line;
}

std::string remove_glm_comments(const std::string& text) {
  std::stringstream in(text);
  std::ostringstream out;
  std::string line;
  while (std::getline(in, line)) out << strip_glm_line_comment(line) << "\n";
  return out.str();
}

std::vector<GlmObject> parse_glm_objects(const std::string& glm_text) {
  const std::string text = remove_glm_comments(glm_text);
  std::vector<GlmObject> objects;
  std::size_t pos = 0;
  while (true) {
    const auto obj_pos = text.find("object", pos);
    if (obj_pos == std::string::npos) break;
    const bool left_ok =
        obj_pos == 0U ||
        !std::isalnum(static_cast<unsigned char>(text[obj_pos - 1U]));
    const bool right_ok =
        obj_pos + 6U >= text.size() ||
        !std::isalnum(static_cast<unsigned char>(text[obj_pos + 6U]));
    if (!left_ok || !right_ok) {
      pos = obj_pos + 6U;
      continue;
    }
    std::size_t cursor = obj_pos + 6U;
    while (cursor < text.size() &&
           std::isspace(static_cast<unsigned char>(text[cursor]))) {
      ++cursor;
    }
    std::size_t cls_end = cursor;
    while (cls_end < text.size() &&
           !std::isspace(static_cast<unsigned char>(text[cls_end])) &&
           text[cls_end] != '{' && text[cls_end] != ':') {
      ++cls_end;
    }
    const std::string cls = ascii_lower(text.substr(cursor, cls_end - cursor));
    std::string object_id;
    std::size_t after_cls = cls_end;
    if (after_cls < text.size() && text[after_cls] == ':') {
      ++after_cls;
      while (after_cls < text.size() &&
             std::isspace(static_cast<unsigned char>(text[after_cls]))) {
        ++after_cls;
      }
      std::size_t id_end = after_cls;
      while (id_end < text.size() &&
             !std::isspace(static_cast<unsigned char>(text[id_end])) &&
             text[id_end] != '{') {
        ++id_end;
      }
      object_id = trim(text.substr(after_cls, id_end - after_cls));
    }
    const auto open = text.find('{', cls_end);
    if (open == std::string::npos) break;
    int depth = 1;
    std::size_t close = open + 1U;
    for (; close < text.size(); ++close) {
      if (text[close] == '{') ++depth;
      if (text[close] == '}') --depth;
      if (depth == 0) break;
    }
    if (close >= text.size()) break;
    const std::string body = text.substr(open + 1U, close - open - 1U);

    GlmObject object;
    object.cls = cls;
    std::string statement;
    bool quoted = false;
    int nested_depth = 0;
    for (char ch : body) {
      if (ch == '"' || ch == '\'') quoted = !quoted;
      if (!quoted && ch == '{') ++nested_depth;
      if (!quoted && ch == '}' && nested_depth > 0) --nested_depth;
      if (!quoted && nested_depth == 0 && ch == ';') {
        const std::string item = trim(statement);
        statement.clear();
        if (item.empty() || starts_with_icase(item, "object")) continue;
        std::stringstream ss(item);
        std::string key;
        ss >> key;
        std::string value;
        std::getline(ss, value);
        if (!key.empty()) {
          object.props[ascii_lower(key)] = strip_quotes(trim(value));
        }
        continue;
      }
      statement.push_back(ch);
    }
    if (!object_id.empty()) {
      object.props["_object_id"] = object_id;
      if (object.props.find("name") == object.props.end()) {
        object.props["name"] = cls + ":" + object_id;
      }
    }
    objects.push_back(std::move(object));
    pos = close + 1U;
  }
  return objects;
}

std::string glm_prop(const GlmObject& obj,
                     const std::string& key,
                     const std::string& fallback = {}) {
  const auto it = obj.props.find(ascii_lower(key));
  return it == obj.props.end() ? fallback : it->second;
}

std::string glm_name_or_id(const GlmObject& obj,
                           const std::string& fallback_prefix = "obj") {
  std::string name = glm_prop(obj, "name");
  if (!name.empty()) return name;
  const std::string id = glm_prop(obj, "_object_id");
  if (!id.empty()) return obj.cls + ":" + id;
  return fallback_prefix;
}

std::string glm_electrical_bus_name(const GlmObject& obj) {
  const std::string parent = glm_prop(obj, "parent");
  if (!parent.empty()) return parent;
  return glm_name_or_id(obj);
}

bool glm_has_power_injection(const GlmObject& obj) {
  for (const auto& phase : {"A", "B", "C"}) {
    if (!glm_prop(obj, "constant_power_" + std::string(phase)).empty()) {
      return true;
    }
  }
  for (const auto& key : {"power_1", "power_2", "power_12"}) {
    if (!glm_prop(obj, key).empty()) return true;
  }
  return false;
}

double glm_number(const GlmObject& obj,
                  const std::string& key,
                  double fallback = 0.0) {
  const auto it = obj.props.find(ascii_lower(key));
  return it == obj.props.end() ? fallback : parse_number(it->second, fallback);
}

struct GlmLineConfiguration {
  std::complex<double> z_self_ohm_per_unit{0.0, 0.0};
  std::complex<double> z_mutual_ohm_per_unit{0.0, 0.0};
  double config_unit_km{kMileToKm};
  bool approximated{false};
};

struct GlmTransformerConfiguration {
  double rating_mva{100.0};
  double primary_kv{0.0};
  double secondary_kv{0.0};
  double r_pu_on_rating{0.0};
  double x_pu_on_rating{0.0};
};

struct GlmConductor {
  double r_ohm_per_unit{0.0};
  double x_ohm_per_unit{0.0};
  double unit_km{kMileToKm};
};

std::optional<GlmConductor> conductor_from_object(const GlmObject& obj) {
  if (obj.cls == "overhead_line_conductor") {
    const std::string raw_r = glm_prop(obj, "resistance");
    const double r = glm_number(obj, "resistance", 0.0);
    if (r <= kTiny) return std::nullopt;
    GlmConductor c;
    c.r_ohm_per_unit = r;
    c.x_ohm_per_unit = 0.40;
    c.unit_km = unit_to_km(unit_word_from_value(raw_r), kMileToKm);
    return c;
  }
  if (obj.cls == "underground_line_conductor") {
    const std::string raw_r = glm_prop(obj, "conductor_resistance",
                                       glm_prop(obj, "resistance"));
    const double r = parse_number(raw_r, 0.0);
    if (r <= kTiny) return std::nullopt;
    GlmConductor c;
    c.r_ohm_per_unit = r;
    c.x_ohm_per_unit = 0.12;
    c.unit_km = unit_to_km(unit_word_from_value(raw_r), kMileToKm);
    return c;
  }
  if (obj.cls == "triplex_line_conductor") {
    const std::string raw_r = glm_prop(obj, "resistance");
    const double r = glm_number(obj, "resistance", 0.0);
    if (r <= kTiny) return std::nullopt;
    GlmConductor c;
    c.r_ohm_per_unit = r;
    c.x_ohm_per_unit = 0.08;
    c.unit_km = unit_to_km(unit_word_from_value(raw_r), kMileToKm);
    return c;
  }
  return std::nullopt;
}

bool is_closed_gridlabd_link(const GlmObject& obj) {
  const std::string status = ascii_lower(glm_prop(obj, "status", "closed"));
  if (status == "open" || status == "blown" || status == "tripped" ||
      status == "false") {
    return false;
  }
  for (const auto& key : {"phase_A_state", "phase_B_state", "phase_C_state"}) {
    const std::string state = ascii_lower(glm_prop(obj, key));
    if (!state.empty() && state != "closed") return false;
  }
  return status.empty() || status == "closed" || status == "good" ||
         status == "true";
}

SwitchType gridlabd_switch_type(const std::string& cls) {
  if (cls == "fuse") return SwitchType::Fuse;
  if (cls == "recloser") return SwitchType::Recloser;
  if (cls == "sectionalizer") return SwitchType::Sectionalizer;
  if (cls == "switch") return SwitchType::LoadBreakSwitch;
  return SwitchType::Disconnector;
}

void add_gridlabd_switch_link(ExternalGridImportReport& report,
                              BusBuilder& buses,
                              const GridLABDImportOptions& options,
                              const GlmObject& obj,
                              SwitchType type,
                              bool closed) {
  const std::string from_name = glm_prop(obj, "from");
  const std::string to_name = glm_prop(obj, "to");
  if (from_name.empty() || to_name.empty()) {
    add_warning_or_throw(report, options,
                         "GridLAB-D link object is missing from/to.");
    return;
  }
  const int from = buses.ensure(from_name);
  const int to = buses.ensure(to_name, buses.base_kv(from));
  Switch sw;
  sw.index = static_cast<int>(report.system.ac.switches.size()) + 1;
  sw.name = glm_prop(obj, "name", obj.cls + "_" + std::to_string(sw.index));
  sw.bus_from = from;
  sw.bus_to = to;
  sw.in_service = true;
  sw.switch_type = type;
  sw.closed = closed;
  report.system.ac.switches.push_back(sw);
}

void add_parent_child_link(ExternalGridImportReport& report,
                           BusBuilder& buses,
                           const GridLABDImportOptions& options,
                           const GlmObject& obj) {
  const std::string parent = glm_prop(obj, "parent");
  const std::string child = glm_name_or_id(obj);
  if (parent.empty() || child.empty() || parent == child) return;
  const int parent_bus = buses.ensure(parent);
  const int child_bus = buses.ensure(child, buses.base_kv(parent_bus));
  Switch sw;
  sw.index = static_cast<int>(report.system.ac.switches.size()) + 1;
  sw.name = "parent_link_" + child;
  sw.bus_from = parent_bus;
  sw.bus_to = child_bus;
  sw.in_service = true;
  sw.switch_type = SwitchType::Disconnector;
  sw.closed = true;
  report.system.ac.switches.push_back(sw);
  (void)options;
}

ExternalGridImportReport import_gridlabd_text(
    const std::string& glm_text,
    const GridLABDImportOptions& options) {
  ExternalGridImportReport report;
  initialise_import_system(report.system, options, "Imported GridLAB-D System");
  BusBuilder buses(report.system, positive_or(options.default_base_kv_ll, 12.47));

  const auto objects = parse_glm_objects(glm_text);
  std::unordered_map<std::string, GlmLineConfiguration> line_configs;
  std::unordered_map<std::string, GlmTransformerConfiguration> transformer_configs;
  std::unordered_map<std::string, GlmConductor> conductors;

  for (const auto& obj : objects) {
    const std::string name = glm_prop(obj, "name");
    if (auto conductor = conductor_from_object(obj)) {
      if (!name.empty()) conductors[ascii_lower(name)] = *conductor;
    }
  }

  for (const auto& obj : objects) {
    const std::string name = glm_prop(obj, "name");
    if (obj.cls == "line_configuration" || obj.cls == "triplex_line_configuration") {
      if (name.empty()) continue;
      GlmLineConfiguration cfg;
      const std::string z11 = glm_prop(obj, "z11");
      if (!z11.empty()) {
        cfg.z_self_ohm_per_unit = parse_complex_rect(z11);
        cfg.z_mutual_ohm_per_unit = parse_complex_rect(glm_prop(obj, "z12", "0+0j"));
        cfg.config_unit_km = unit_to_km(unit_word_from_value(z11), kMileToKm);
      } else {
        double r_sum = 0.0;
        double x_sum = 0.0;
        double unit_sum = 0.0;
        int count = 0;
        for (const auto& key : {"conductor_A", "conductor_B", "conductor_C",
                                "conductor_1", "conductor_2"}) {
          const std::string ref = ascii_lower(glm_prop(obj, key));
          const auto it = conductors.find(ref);
          if (it == conductors.end()) continue;
          r_sum += it->second.r_ohm_per_unit;
          x_sum += it->second.x_ohm_per_unit;
          unit_sum += it->second.unit_km;
          ++count;
        }
        if (count > 0) {
          cfg.z_self_ohm_per_unit = {r_sum / count, x_sum / count};
          cfg.config_unit_km = positive_or(unit_sum / count, kMileToKm);
          cfg.approximated = true;
          report.warnings.push_back(
              "GridLAB-D line configuration " + name +
              " projected from conductor resistance with approximate reactance.");
        } else {
          cfg.z_self_ohm_per_unit = {0.5, 0.4};
          cfg.config_unit_km = kMileToKm;
          cfg.approximated = true;
          report.warnings.push_back(
              "GridLAB-D line configuration " + name +
              " lacks z-matrix/conductor data; using a conservative generic impedance.");
        }
      }
      line_configs[ascii_lower(name)] = cfg;
    } else if (obj.cls == "transformer_configuration") {
      if (name.empty()) continue;
      GlmTransformerConfiguration cfg;
      cfg.rating_mva = positive_or(glm_number(obj, "power_rating", 100000.0) / 1000.0,
                                   positive_or(options.default_base_mva, 100.0));
      cfg.primary_kv = glm_number(obj, "primary_voltage", 0.0) / 1000.0;
      cfg.secondary_kv = glm_number(obj, "secondary_voltage", 0.0) / 1000.0;
      cfg.r_pu_on_rating = glm_number(obj, "resistance", 0.0);
      cfg.x_pu_on_rating = glm_number(obj, "reactance", 0.0);
      transformer_configs[ascii_lower(name)] = cfg;
    }
  }

  for (const auto& obj : objects) {
    if (obj.cls != "meter" && obj.cls != "load" && obj.cls != "node" &&
        obj.cls != "triplex_meter" && obj.cls != "triplex_node") {
      continue;
    }
    const bool is_bus_object =
        obj.cls == "meter" || obj.cls == "node" ||
        obj.cls == "triplex_meter" || obj.cls == "triplex_node";
    std::string name =
        is_bus_object ? glm_name_or_id(obj) : glm_electrical_bus_name(obj);
    if (name.empty()) name = "bus_" + std::to_string(report.system.ac.buses.size() + 1U);
    const double nominal_ln_v = glm_number(obj, "nominal_voltage",
                                           options.default_base_kv_ll * 1000.0 / kSqrt3);
    const bool is_triplex =
        obj.cls == "triplex_meter" || obj.cls == "triplex_node";
    const double base_kv = positive_or(
        nominal_ln_v * (is_triplex ? 2.0 : kSqrt3) / 1000.0,
        options.default_base_kv_ll);
    const bool is_swing = ascii_lower(glm_prop(obj, "bustype")) == "swing";
    const int bus = buses.ensure(name, base_kv,
                                 is_swing ? BusType::SLACK : BusType::PQ);
    if (is_swing) {
      ensure_single_slack_generator(report.system, bus, 1.0, 0.0,
                                    options.create_slack_generator);
    }

    std::complex<double> power_va{0.0, 0.0};
    bool has_power = false;
    for (const auto& phase : {"A", "B", "C"}) {
      const std::string value = glm_prop(obj, "constant_power_" + std::string(phase));
      if (!value.empty()) {
        power_va += parse_complex_rect(value);
        has_power = true;
      }
    }
    for (const auto& key : {"power_1", "power_2", "power_12"}) {
      const std::string value = glm_prop(obj, key);
      if (!value.empty()) {
        power_va += parse_complex_rect(value);
        has_power = true;
      }
    }
    if (has_power && std::abs(power_va) > 1e-6) {
      int injection_bus = bus;
      std::string injection_name = name;
      const std::string parent = glm_prop(obj, "parent");
      if (!parent.empty() && (obj.cls == "load" ||
                              (is_triplex && glm_has_power_injection(obj)))) {
        injection_bus = buses.ensure(parent, base_kv);
        injection_name = parent;
      }
      const double p_mw = power_va.real() / 1.0e6;
      const double q_mvar = power_va.imag() / 1.0e6;
      if (p_mw < -kTiny || q_mvar < -kTiny) {
        StaticGenerator gen;
        gen.index = static_cast<int>(report.system.ac.static_generators.size()) + 1;
        gen.name = "negative_load_at_" + injection_name;
        gen.bus = injection_bus;
        gen.in_service = true;
        gen.p_mw = -p_mw;
        gen.q_mvar = -q_mvar;
        gen.p_rated_mw = std::max(0.0, gen.p_mw);
        gen.sn_mva = std::hypot(gen.p_mw, gen.q_mvar);
        report.system.ac.static_generators.push_back(gen);
      } else {
        Load load;
        load.index = static_cast<int>(report.system.ac.loads.size()) + 1;
        load.name = "load_at_" + injection_name;
        load.bus = injection_bus;
        load.in_service = true;
        load.p_mw = p_mw;
        load.q_mvar = q_mvar;
        load.scaling = 1.0;
        report.system.ac.loads.push_back(load);
      }
    }
  }

  for (const auto& obj : objects) {
    if ((obj.cls == "meter" || obj.cls == "node" ||
         obj.cls == "triplex_meter" || obj.cls == "triplex_node") &&
        !glm_prop(obj, "parent").empty()) {
      add_parent_child_link(report, buses, options, obj);
    } else if (obj.cls == "overhead_line" || obj.cls == "underground_line" ||
        obj.cls == "line" || obj.cls == "triplex_line") {
      const std::string from_name = glm_prop(obj, "from");
      const std::string to_name = glm_prop(obj, "to");
      const std::string cfg_name = glm_prop(obj, "configuration");
      if (from_name.empty() || to_name.empty() || cfg_name.empty()) {
        add_warning_or_throw(report, options,
                             "GridLAB-D line object is missing from/to/configuration.");
        continue;
      }
      const int from = buses.ensure(from_name);
      const int to = buses.ensure(to_name, buses.base_kv(from));
      const auto cfg_it = line_configs.find(ascii_lower(cfg_name));
      if (cfg_it == line_configs.end()) {
        add_warning_or_throw(report, options,
                             "GridLAB-D line " + glm_prop(obj, "name") +
                                 " references unknown configuration " + cfg_name + ".");
        continue;
      }
      const auto& cfg = cfg_it->second;
      const double length = positive_or(glm_number(obj, "length", 1.0), 1.0);
      const double length_unit_km = unit_to_km(unit_word_from_value(glm_prop(obj, "length")),
                                              kFootToKm);
      const double length_km = length * length_unit_km;
      const double length_in_cfg_units = length_km / positive_or(cfg.config_unit_km, kMileToKm);
      const std::complex<double> z1_per_unit =
          cfg.z_self_ohm_per_unit - cfg.z_mutual_ohm_per_unit;
      const std::complex<double> z0_per_unit =
          cfg.z_self_ohm_per_unit + 2.0 * cfg.z_mutual_ohm_per_unit;
      const std::complex<double> z1_total = z1_per_unit * length_in_cfg_units;
      const std::complex<double> z0_total = z0_per_unit * length_in_cfg_units;
      const double zbase =
          (buses.base_kv(from) * buses.base_kv(from)) /
          positive_or(report.system.ac.base_mva, 100.0);

      ACBranch br;
      br.index = static_cast<int>(report.system.ac.branches.size()) + 1;
      br.name = glm_prop(obj, "name", "line_" + std::to_string(br.index));
      br.from_bus = from;
      br.to_bus = to;
      br.length_km = length_km;
      br.r_ohm_per_km = z1_per_unit.real() / positive_or(cfg.config_unit_km, kMileToKm);
      br.x_ohm_per_km = z1_per_unit.imag() / positive_or(cfg.config_unit_km, kMileToKm);
      br.r_pu = z1_total.real() / zbase;
      br.x_pu = z1_total.imag() / zbase;
      br.r0_pu = z0_total.real() / zbase;
      br.x0_pu = z0_total.imag() / zbase;
      br.tap = 1.0;
      br.in_service = true;
      report.system.ac.branches.push_back(br);
    } else if (obj.cls == "switch" || obj.cls == "fuse" ||
               obj.cls == "recloser" || obj.cls == "sectionalizer") {
      add_gridlabd_switch_link(report, buses, options, obj,
                               gridlabd_switch_type(obj.cls),
                               is_closed_gridlabd_link(obj));
    } else if (obj.cls == "regulator") {
      add_gridlabd_switch_link(report, buses, options, obj,
                               SwitchType::Disconnector, true);
    } else if (obj.cls == "transformer") {
      const std::string from_name = glm_prop(obj, "from");
      const std::string to_name = glm_prop(obj, "to");
      const std::string cfg_name = glm_prop(obj, "configuration");
      if (from_name.empty() || to_name.empty() || cfg_name.empty()) {
        add_warning_or_throw(report, options,
                             "GridLAB-D transformer object is missing from/to/configuration.");
        continue;
      }
      const auto cfg_it = transformer_configs.find(ascii_lower(cfg_name));
      if (cfg_it == transformer_configs.end()) {
        add_warning_or_throw(report, options,
                             "GridLAB-D transformer " + glm_prop(obj, "name") +
                                 " references unknown configuration " + cfg_name + ".");
        continue;
      }
      const auto& cfg = cfg_it->second;
      const int from = buses.ensure(from_name, positive_or(cfg.primary_kv, options.default_base_kv_ll));
      const int to = buses.ensure(to_name, positive_or(cfg.secondary_kv, buses.base_kv(from)));
      const double z_scale = positive_or(report.system.ac.base_mva, 100.0) /
                             positive_or(cfg.rating_mva, report.system.ac.base_mva);

      ACBranch br;
      br.index = static_cast<int>(report.system.ac.branches.size()) + 1;
      br.name = glm_prop(obj, "name", "transformer_" + std::to_string(br.index));
      br.from_bus = from;
      br.to_bus = to;
      br.r_pu = cfg.r_pu_on_rating * z_scale;
      br.x_pu = cfg.x_pu_on_rating * z_scale;
      br.tap = 1.0;
      br.in_service = true;
      br.rate_a_mva = cfg.rating_mva;
      br.vn_hv_kv = cfg.primary_kv;
      br.vn_lv_kv = cfg.secondary_kv;
      br.sn_mva = cfg.rating_mva;
      report.system.ac.branches.push_back(br);
    } else if (obj.cls == "recorder" || obj.cls == "collector") {
      continue;
    } else if (obj.cls != "meter" && obj.cls != "load" && obj.cls != "node" &&
               obj.cls != "triplex_meter" && obj.cls != "triplex_node" &&
               obj.cls != "line_configuration" &&
               obj.cls != "triplex_line_configuration" &&
               obj.cls != "overhead_line_conductor" &&
               obj.cls != "underground_line_conductor" &&
               obj.cls != "triplex_line_conductor" &&
               obj.cls != "line_spacing" &&
               obj.cls != "transformer_configuration" &&
               obj.cls != "regulator_configuration") {
      report.skipped.push_back("Unsupported GridLAB-D object skipped: " + obj.cls);
    }
  }

  return finalise_import(std::move(report), options);
}

std::string bus_name_for_export(const ACBus& bus,
                                std::unordered_set<std::string>& used) {
  const std::string raw =
      bus.name.empty() ? ("bus_" + std::to_string(bus.index))
                       : ("bus_" + std::to_string(bus.index) + "_" + bus.name);
  return unique_identifier(raw, "bus_" + std::to_string(bus.index), used);
}

bool is_transformer_like_branch(const ACBranch& branch,
                                const ACBus& from_bus,
                                const ACBus& to_bus,
                                double minimum_base_kv) {
  const double from_kv = bus_base_kv(from_bus, minimum_base_kv);
  const double to_kv = bus_base_kv(to_bus, minimum_base_kv);
  const double scale = std::max({1.0, std::abs(from_kv), std::abs(to_kv)});
  return std::abs(branch.tap - 1.0) > 1e-9 ||
         std::abs(from_kv - to_kv) > 1e-4 * scale ||
         branch.vn_hv_kv > 0.0 || branch.vn_lv_kv > 0.0;
}

std::string export_opendss_text(const HybridPowerSystem& input,
                                const OpenDSSExportOptions& options) {
  HybridPowerSystem sys =
      options.project_to_canonical ? project_to_canonical_models(input) : input;
  convert_actual_to_per_unit(sys);
  if (sys.ac.buses.empty()) {
    throw std::runtime_error("to_opendss: AC bus table is empty.");
  }

  const auto bus_lookup = bus_by_index(sys.ac);
  const double base_mva = system_base_mva(sys);
  const int slack_bus = find_slack_bus(sys.ac);
  const auto slack_v = slack_voltage_by_bus(sys.ac);
  const auto slack_it = bus_lookup.find(slack_bus);
  if (slack_it == bus_lookup.end()) {
    throw std::runtime_error("to_opendss: unable to locate slack bus.");
  }

  std::unordered_set<std::string> used_names;
  std::unordered_map<int, std::string> dss_bus_name;
  for (const auto& bus : sys.ac.buses) {
    dss_bus_name[bus.index] = bus_name_for_export(bus, used_names);
  }

  const auto [slack_vm, slack_va] =
      slack_v.count(slack_bus) != 0U ? slack_v.at(slack_bus)
                                     : std::make_pair(slack_it->second->vm_pu,
                                                      slack_it->second->va_deg);
  const double slack_base_kv =
      bus_base_kv(*slack_it->second, options.minimum_base_kv);

  std::ostringstream out;
  if (options.include_metadata_comments) {
    out << "! Generated by hacdcpf OpenDSS text exporter.\n";
    out << "! Balanced three-phase AC snapshot; dynamic controls are exported "
           "only as steady PQ/source equivalents.\n";
  }
  out << "Clear\n";
  out << "Set DefaultBaseFrequency=" << format_double(options.nominal_frequency_hz)
      << "\n";
  out << "New Circuit." << sanitize_identifier(options.circuit_name, "hacdcpf_export")
      << " bus1=" << dss_bus_ref(dss_bus_name.at(slack_bus))
      << " basekv=" << format_double(slack_base_kv)
      << " pu=" << format_double(positive_or(slack_vm, 1.0))
      << " angle=" << format_double(slack_va)
      << " phases=3\n";
  out << "Set baseMVA=" << format_double(base_mva) << "\n\n";

  for (const auto& branch : sys.ac.branches) {
    if (!branch.in_service) continue;
    const auto from_it = bus_lookup.find(branch.from_bus);
    const auto to_it = bus_lookup.find(branch.to_bus);
    if (from_it == bus_lookup.end() || to_it == bus_lookup.end()) continue;
    if (std::hypot(branch.r_pu, branch.x_pu) <= kTiny) continue;
    const auto& from_bus = *from_it->second;
    const auto& to_bus = *to_it->second;
    const std::string name = sanitize_identifier(
        branch.name.empty() ? ("branch_" + std::to_string(branch.index))
                            : branch.name,
        "branch_" + std::to_string(branch.index));
    if (is_transformer_like_branch(branch, from_bus, to_bus,
                                   options.minimum_base_kv)) {
      const double rating_mva = positive_or(branch.rate_a_mva,
                                            positive_or(branch.sn_mva, base_mva));
      const double z_tr_scale = rating_mva / positive_or(base_mva, 100.0);
      const double r_percent = std::max(0.0, branch.r_pu * z_tr_scale * 100.0);
      const double x_percent = std::max(0.0, branch.x_pu * z_tr_scale * 100.0);
      const double hv_kv =
          positive_or(branch.vn_hv_kv, bus_base_kv(from_bus, options.minimum_base_kv));
      const double lv_kv =
          positive_or(branch.vn_lv_kv, bus_base_kv(to_bus, options.minimum_base_kv));
      out << "New Transformer." << name
          << " phases=3 windings=2"
          << " buses=[" << dss_bus_ref(dss_bus_name.at(branch.from_bus)) << ","
          << dss_bus_ref(dss_bus_name.at(branch.to_bus)) << "]"
          << " kvs=[" << format_double(hv_kv) << "," << format_double(lv_kv) << "]"
          << " kvas=[" << format_double(rating_mva * 1000.0) << ","
          << format_double(rating_mva * 1000.0) << "]"
          << " %r=" << format_double(r_percent)
          << " xhl=" << format_double(x_percent)
          << " conns=[wye,wye]\n";
      continue;
    }

    const double base_kv = bus_base_kv(from_bus, options.minimum_base_kv);
    const double zbase = (base_kv * base_kv) / positive_or(base_mva, 100.0);
    const double length_km = positive_or(branch.length_km, 1.0);
    const double r_total = branch.r_pu * zbase;
    const double x_total = branch.x_pu * zbase;
    const double r0_total =
        (std::hypot(branch.r0_pu, branch.x0_pu) > kTiny ? branch.r0_pu
                                                        : branch.r_pu) *
        zbase;
    const double x0_total =
        (std::hypot(branch.r0_pu, branch.x0_pu) > kTiny ? branch.x0_pu
                                                        : branch.x_pu) *
        zbase;
    out << "New Line." << name
        << " bus1=" << dss_bus_ref(dss_bus_name.at(branch.from_bus))
        << " bus2=" << dss_bus_ref(dss_bus_name.at(branch.to_bus))
        << " phases=3"
        << " r1=" << format_double(r_total / length_km)
        << " x1=" << format_double(x_total / length_km)
        << " r0=" << format_double(r0_total / length_km)
        << " x0=" << format_double(x0_total / length_km)
        << " length=" << format_double(length_km)
        << " units=km\n";
  }

  int load_id = 1;
  for (const auto& [bus, demand] : aggregate_bus_demand(sys)) {
    if (std::abs(demand.p_mw) <= kTiny && std::abs(demand.q_mvar) <= kTiny) {
      continue;
    }
    const auto b_it = bus_lookup.find(bus);
    if (b_it == bus_lookup.end() || dss_bus_name.count(bus) == 0U) continue;
    out << "New Load.BusDemand" << load_id++
        << " bus1=" << dss_bus_ref(dss_bus_name.at(bus))
        << " phases=3 kv="
        << format_double(bus_base_kv(*b_it->second, options.minimum_base_kv))
        << " kw=" << format_double(demand.p_mw * 1000.0)
        << " kvar=" << format_double(demand.q_mvar * 1000.0)
        << " model=1\n";
  }

  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service || gen.is_slack) continue;
    const auto b_it = bus_lookup.find(gen.bus);
    if (b_it == bus_lookup.end() || dss_bus_name.count(gen.bus) == 0U) continue;
    out << "New Generator."
        << sanitize_identifier(gen.name.empty() ? ("Gen" + std::to_string(gen.index))
                                                : gen.name,
                               "Gen" + std::to_string(gen.index))
        << " bus1=" << dss_bus_ref(dss_bus_name.at(gen.bus))
        << " phases=3 kv="
        << format_double(bus_base_kv(*b_it->second, options.minimum_base_kv))
        << " kw=" << format_double(gen.pg_mw * 1000.0)
        << " kvar=" << format_double(gen.qg_mvar * 1000.0)
        << " model=1\n";
  }
  for (const auto& gen : sys.ac.static_generators) {
    if (!gen.in_service) continue;
    const auto b_it = bus_lookup.find(gen.bus);
    if (b_it == bus_lookup.end() || dss_bus_name.count(gen.bus) == 0U) continue;
    const double scale = gen.scaling > 0.0 ? gen.scaling : 1.0;
    out << "New Generator."
        << sanitize_identifier(gen.name.empty() ? ("SGen" + std::to_string(gen.index))
                                                : gen.name,
                               "SGen" + std::to_string(gen.index))
        << " bus1=" << dss_bus_ref(dss_bus_name.at(gen.bus))
        << " phases=3 kv="
        << format_double(bus_base_kv(*b_it->second, options.minimum_base_kv))
        << " kw=" << format_double(scale * gen.p_mw * 1000.0)
        << " kvar=" << format_double(scale * gen.q_mvar * 1000.0)
        << " model=1\n";
  }

  if (options.include_solve_command) out << "\nSolve\n";
  return out.str();
}

}  // namespace

std::string to_gridlabd(const HybridPowerSystem& sys,
                        const GridLABDExportOptions& options) {
  const auto dir = temp_export_dir("gridlabd_text_export");
  const auto snapshot = export_gridlabd_snapshot(sys, dir, options);
  const std::string text = read_text_file_or_throw(snapshot.glm_path, "GridLAB-D GLM");
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  return text;
}

void save_gridlabd(const HybridPowerSystem& sys,
                   const std::filesystem::path& path,
                   const GridLABDExportOptions& options) {
  write_text_file_or_throw(path, to_gridlabd(sys, options), "GridLAB-D GLM");
}

ExternalGridImportReport from_gridlabd_with_report(
    const std::string& glm_text,
    const GridLABDImportOptions& options) {
  return import_gridlabd_text(glm_text, options);
}

HybridPowerSystem from_gridlabd(const std::string& glm_text,
                                const GridLABDImportOptions& options) {
  return from_gridlabd_with_report(glm_text, options).system;
}

ExternalGridImportReport load_gridlabd_with_report(
    const std::filesystem::path& path,
    const GridLABDImportOptions& options) {
  return from_gridlabd_with_report(read_text_file_or_throw(path, "GridLAB-D GLM"),
                                   options);
}

HybridPowerSystem load_gridlabd(const std::filesystem::path& path,
                                const GridLABDImportOptions& options) {
  return load_gridlabd_with_report(path, options).system;
}

Result<HybridPowerSystem> try_from_gridlabd(
    const std::string& glm_text,
    const GridLABDImportOptions& options) {
  try {
    return from_gridlabd(glm_text, options);
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to parse GridLAB-D GLM: ") + e.what(), {}};
  }
}

Result<HybridPowerSystem> try_load_gridlabd(
    const std::filesystem::path& path,
    const GridLABDImportOptions& options) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return Error{ErrorCode::FileNotFound,
                 "File not found: " + path.string(), {}};
  }
  try {
    std::ostringstream ss;
    ss << in.rdbuf();
    return try_from_gridlabd(ss.str(), options);
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to read GridLAB-D GLM: ") + e.what(), {}};
  }
}

std::string to_opendss(const HybridPowerSystem& sys,
                       const OpenDSSExportOptions& options) {
  return export_opendss_text(sys, options);
}

void save_opendss(const HybridPowerSystem& sys,
                  const std::filesystem::path& path,
                  const OpenDSSExportOptions& options) {
  write_text_file_or_throw(path, to_opendss(sys, options), "OpenDSS DSS");
}

ExternalGridImportReport from_opendss_with_report(
    const std::string& dss_text,
    const OpenDSSImportOptions& options) {
  return import_opendss_text(dss_text, options);
}

HybridPowerSystem from_opendss(const std::string& dss_text,
                               const OpenDSSImportOptions& options) {
  return from_opendss_with_report(dss_text, options).system;
}

ExternalGridImportReport load_opendss_with_report(
    const std::filesystem::path& path,
    const OpenDSSImportOptions& options) {
  return from_opendss_with_report(read_text_file_or_throw(path, "OpenDSS DSS"),
                                  options);
}

HybridPowerSystem load_opendss(const std::filesystem::path& path,
                               const OpenDSSImportOptions& options) {
  return load_opendss_with_report(path, options).system;
}

Result<HybridPowerSystem> try_from_opendss(
    const std::string& dss_text,
    const OpenDSSImportOptions& options) {
  try {
    return from_opendss(dss_text, options);
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to parse OpenDSS DSS: ") + e.what(), {}};
  }
}

Result<HybridPowerSystem> try_load_opendss(
    const std::filesystem::path& path,
    const OpenDSSImportOptions& options) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return Error{ErrorCode::FileNotFound,
                 "File not found: " + path.string(), {}};
  }
  try {
    std::ostringstream ss;
    ss << in.rdbuf();
    return try_from_opendss(ss.str(), options);
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to read OpenDSS DSS: ") + e.what(), {}};
  }
}

}  // namespace hacdcpf::io
