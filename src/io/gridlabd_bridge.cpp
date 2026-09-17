#include "hacdcpf/io/gridlabd_bridge.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/unit_conversion.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::io {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt3 = 1.7320508075688772935;
constexpr double kTinyPowerMw = 1e-9;
constexpr double kTinyImpedance = 1e-12;

std::string getenv_string(const char* name) {
  const char* value = std::getenv(name);
  return value != nullptr ? std::string(value) : std::string();
}

std::string path_list_separator() {
#ifdef _WIN32
  return ";";
#else
  return ":";
#endif
}

std::vector<std::filesystem::path> split_path_list(const std::string& raw) {
  std::vector<std::filesystem::path> out;
  std::stringstream ss(raw);
  std::string item;
#ifdef _WIN32
  constexpr char sep = ';';
#else
  constexpr char sep = ':';
#endif
  while (std::getline(ss, item, sep)) {
    if (!item.empty()) out.emplace_back(item);
  }
  return out;
}

bool is_existing_directory(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::exists(path, ec) && !ec &&
         std::filesystem::is_directory(path, ec) && !ec;
}

bool is_executable_file(const std::filesystem::path& path) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) return false;
  if (std::filesystem::is_directory(path, ec)) return false;
#ifdef _WIN32
  return true;
#else
  return ::access(path.c_str(), X_OK) == 0;
#endif
}

std::filesystem::path project_root_path() {
#ifdef HACDCPF_PROJECT_ROOT
  return std::filesystem::path(HACDCPF_PROJECT_ROOT);
#else
  return std::filesystem::current_path();
#endif
}

std::string join_unique_paths(const std::vector<std::filesystem::path>& paths,
                              const std::string& existing = {}) {
  std::vector<std::string> entries;
  std::unordered_set<std::string> seen;
  std::error_code ec;

  auto add = [&](const std::filesystem::path& path) {
    if (path.empty()) return;
    const auto absolute = std::filesystem::absolute(path, ec);
    const auto text = (ec ? path : absolute).string();
    if (!text.empty() && seen.insert(text).second) entries.push_back(text);
  };

  for (const auto& path : paths) add(path);
  for (const auto& path : split_path_list(existing)) add(path);

  std::ostringstream os;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i != 0U) os << path_list_separator();
    os << entries[i];
  }
  return os.str();
}

std::vector<std::filesystem::path> gridlabd_runtime_paths(
    const std::filesystem::path& executable) {
  std::vector<std::filesystem::path> roots;
  std::vector<std::filesystem::path> candidates;
  std::vector<std::filesystem::path> existing;
  std::unordered_set<std::string> seen;
  std::error_code ec;

  auto add_root = [&](const std::filesystem::path& path) {
    if (path.empty()) return;
    const auto absolute = std::filesystem::absolute(path, ec);
    roots.push_back(ec ? path : absolute);
  };

  add_root(executable.parent_path());
  add_root(executable.parent_path().parent_path());
  add_root(project_root_path().parent_path() / "gridlab-d");
  add_root(project_root_path().parent_path() / "gridlab-d" / "cmake-build");

  for (const auto& root : roots) {
    candidates.push_back(root);
    candidates.push_back(root / "lib");
    candidates.push_back(root / "share");
    candidates.push_back(root / "cmake-build");
    candidates.push_back(root / "cmake-build" / "lib");
    candidates.push_back(root / "cmake-build" / "share");
    candidates.push_back(root / "build");
    candidates.push_back(root / "build" / "lib");
    candidates.push_back(root / "build" / "share");
    candidates.push_back(root / "build-macos");
    candidates.push_back(root / "build-macos" / "lib");
    candidates.push_back(root / "build-macos" / "share");
  }

  for (const auto& candidate : candidates) {
    if (!is_existing_directory(candidate)) continue;
    const auto absolute = std::filesystem::absolute(candidate, ec);
    const auto text = (ec ? candidate : absolute).string();
    if (seen.insert(text).second) existing.emplace_back(text);
  }
  return existing;
}

std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

std::string sanitize_identifier(std::string raw, const std::string& fallback) {
  if (raw.empty()) raw = fallback;
  std::string out;
  out.reserve(raw.size() + 2);
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

std::string unique_identifier(std::string base,
                              std::unordered_set<std::string>& used) {
  base = sanitize_identifier(std::move(base), "obj");
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

std::string format_complex_rect(double real, double imag) {
  std::ostringstream os;
  os << std::setprecision(15) << real;
  if (imag >= 0.0) os << "+";
  os << std::setprecision(15) << imag << "j";
  return os.str();
}

std::string format_complex_polar(double magnitude, double angle_deg) {
  std::ostringstream os;
  os << std::setprecision(15) << magnitude;
  if (angle_deg >= 0.0) os << "+";
  os << std::setprecision(15) << angle_deg << "d";
  return os.str();
}

double deg_to_rad(double deg) {
  return deg * kPi / 180.0;
}

double rad_to_deg(double rad) {
  return rad * 180.0 / kPi;
}

double angle_diff_deg(double a, double b) {
  double d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

double bus_base_kv(const ACBus& bus,
                   const GridLABDExportOptions& options) {
  return std::max(options.minimum_base_kv, bus.base_kv > 0.0 ? bus.base_kv : 12.47);
}

bool nominal_voltage_mismatch(const ACBus& from_bus,
                              const ACBus& to_bus,
                              const GridLABDExportOptions& options) {
  const double from_kv = bus_base_kv(from_bus, options);
  const double to_kv = bus_base_kv(to_bus, options);
  const double scale = std::max({1.0, std::abs(from_kv), std::abs(to_kv)});
  return std::abs(from_kv - to_kv) > 1e-4 * scale;
}

bool is_gridlabd_transformer_branch(const ACBranch& branch,
                                    const ACBus& from_bus,
                                    const ACBus& to_bus,
                                    const GridLABDExportOptions& options) {
  return std::abs(branch.tap - 1.0) > 1e-9 ||
         nominal_voltage_mismatch(from_bus, to_bus, options);
}

std::complex<double> branch_series_admittance(const ACBranch& branch) {
  const std::complex<double> z(branch.r_pu, branch.x_pu);
  if (std::abs(z) <= kTinyImpedance) return {0.0, 0.0};
  return 1.0 / z;
}

std::complex<double> branch_zero_sequence_admittance(const ACBranch& branch) {
  const bool has_zero_sequence =
      std::hypot(branch.r0_pu, branch.x0_pu) > kTinyImpedance;
  const std::complex<double> z(has_zero_sequence ? branch.r0_pu : branch.r_pu,
                               has_zero_sequence ? branch.x0_pu : branch.x_pu);
  if (std::abs(z) <= kTinyImpedance) return {0.0, 0.0};
  return 1.0 / z;
}

ACBranch make_parallel_equivalent_branch(
    const ACSystem& ac,
    const std::vector<int>& branch_positions,
    const std::string& equivalent_name) {
  ACBranch equivalent =
      ac.branches[static_cast<std::size_t>(branch_positions.front())];

  std::complex<double> y_sum{0.0, 0.0};
  std::complex<double> y0_sum{0.0, 0.0};
  double b_sum = 0.0;
  double b0_sum = 0.0;
  double rate_a_sum = 0.0;
  double rate_b_sum = 0.0;
  double rate_c_sum = 0.0;
  bool all_rate_a_positive = true;
  bool all_rate_b_positive = true;
  bool all_rate_c_positive = true;
  int n_parallel = 0;

  for (const int pos : branch_positions) {
    const auto& branch =
        ac.branches[static_cast<std::size_t>(pos)];
    y_sum += branch_series_admittance(branch);
    y0_sum += branch_zero_sequence_admittance(branch);
    b_sum += branch.b_pu;
    b0_sum += branch.b0_pu;
    if (branch.rate_a_mva > 0.0) {
      rate_a_sum += branch.rate_a_mva;
    } else {
      all_rate_a_positive = false;
    }
    if (branch.rate_b_mva > 0.0) {
      rate_b_sum += branch.rate_b_mva;
    } else {
      all_rate_b_positive = false;
    }
    if (branch.rate_c_mva > 0.0) {
      rate_c_sum += branch.rate_c_mva;
    } else {
      all_rate_c_positive = false;
    }
    n_parallel += std::max(1, branch.n_parallel);
  }

  if (std::abs(y_sum) > kTinyImpedance) {
    const std::complex<double> z_eq = 1.0 / y_sum;
    equivalent.r_pu = z_eq.real();
    equivalent.x_pu = z_eq.imag();
  }
  if (std::abs(y0_sum) > kTinyImpedance) {
    const std::complex<double> z0_eq = 1.0 / y0_sum;
    equivalent.r0_pu = z0_eq.real();
    equivalent.x0_pu = z0_eq.imag();
  }
  equivalent.b_pu = b_sum;
  equivalent.b0_pu = b0_sum;
  equivalent.rate_a_mva = all_rate_a_positive ? rate_a_sum : 0.0;
  equivalent.rate_b_mva = all_rate_b_positive ? rate_b_sum : 0.0;
  equivalent.rate_c_mva = all_rate_c_positive ? rate_c_sum : 0.0;
  equivalent.n_parallel = std::max(1, n_parallel);
  equivalent.name = equivalent_name;
  return equivalent;
}

std::vector<int> merged_branch_positions(
    const std::vector<GridLABDBranchMapping>& mappings,
    const GridLABDBranchMapping& mapping) {
  std::vector<int> out;
  if (!mapping.exported_as_parallel_equivalent) {
    out.push_back(mapping.canonical_branch_position);
    return out;
  }
  for (const int branch_index : mapping.merged_canonical_branch_indices) {
    const auto it = std::find_if(
        mappings.begin(), mappings.end(),
        [&](const GridLABDBranchMapping& candidate) {
          return candidate.canonical_branch_index == branch_index;
        });
    if (it != mappings.end()) out.push_back(it->canonical_branch_position);
  }
  if (out.empty()) out.push_back(mapping.canonical_branch_position);
  return out;
}

std::unordered_map<int, int> bus_pos_by_index(const ACSystem& ac) {
  std::unordered_map<int, int> out;
  out.reserve(ac.buses.size());
  for (int i = 0; i < static_cast<int>(ac.buses.size()); ++i) {
    out[ac.buses[static_cast<std::size_t>(i)].index] = i;
  }
  return out;
}

std::unordered_map<int, double> slack_vm_by_bus(const ACSystem& ac) {
  std::unordered_map<int, double> out;
  for (const auto& eg : ac.external_grids) {
    if (eg.in_service) out[eg.bus] = eg.vm_pu;
  }
  for (const auto& gen : ac.generators) {
    if (gen.in_service && gen.is_slack) out[gen.bus] = gen.vg_pu;
  }
  return out;
}

std::unordered_map<int, double> slack_va_by_bus(const ACSystem& ac) {
  std::unordered_map<int, double> out;
  for (const auto& eg : ac.external_grids) {
    if (eg.in_service) out[eg.bus] = eg.va_deg;
  }
  for (const auto& gen : ac.generators) {
    if (gen.in_service && gen.is_slack) out[gen.bus] = 0.0;
  }
  return out;
}

struct BusInjection {
  double p_mw{0.0};
  double q_mvar{0.0};
};

struct BusShuntAdmittance {
  double gs_mw{0.0};
  double bs_mvar{0.0};
};

void add_injection(std::unordered_map<int, BusInjection>& by_bus,
                   int bus,
                   double p_mw,
                   double q_mvar) {
  if (bus == 0) return;
  by_bus[bus].p_mw += p_mw;
  by_bus[bus].q_mvar += q_mvar;
}

void add_shunt_admittance(std::unordered_map<int, BusShuntAdmittance>& by_bus,
                          int bus,
                          double gs_mw,
                          double bs_mvar) {
  if (bus == 0) return;
  by_bus[bus].gs_mw += gs_mw;
  by_bus[bus].bs_mvar += bs_mvar;
}

std::unordered_map<int, BusInjection> aggregate_constant_power_demand(
    const HybridPowerSystem& sys,
    std::vector<std::string>& warnings,
    const GridLABDExportOptions& options) {
  const auto& ac = sys.ac;
  std::unordered_map<int, BusInjection> by_bus;

  for (const auto& bus : ac.buses) {
    if (!bus.in_service) continue;
    add_injection(by_bus, bus.index, bus.pd_mw, bus.qd_mvar);
    if (!options.include_shunt_admittance_as_impedance) {
      add_injection(by_bus, bus.index, bus.gs_mw, -bus.bs_mvar);
    }
  }
  for (const auto& load : ac.loads) {
    if (!load.in_service) continue;
    const double scale = load.scaling > 0.0 ? load.scaling : 1.0;
    add_injection(by_bus, load.bus, scale * load.p_mw, scale * load.q_mvar);
  }
  for (const auto& load : ac.flexible_loads) {
    if (load.in_service) add_injection(by_bus, load.bus, load.p_mw, load.q_mvar);
  }
  for (const auto& load : ac.asymmetric_loads) {
    if (!load.in_service) continue;
    const double scale = load.scaling > 0.0 ? load.scaling : 1.0;
    add_injection(by_bus, load.bus,
                  scale * (load.pa_mw + load.pb_mw + load.pc_mw),
                  scale * (load.qa_mvar + load.qb_mvar + load.qc_mvar));
  }
  for (const auto& cs : ac.charging_stations) {
    if (!cs.in_service) continue;
    add_injection(by_bus, cs.bus, cs.p_total_kw / 1000.0,
                  cs.q_total_kvar / 1000.0);
  }
  for (const auto& sh : ac.shunts) {
    if (!sh.in_service) continue;
    double bs = sh.bs_mvar;
    if (sh.switchable && sh.n_steps > 0) {
      bs = sh.bs_per_step * sh.current_step;
    }
    if (!options.include_shunt_admittance_as_impedance) {
      add_injection(by_bus, sh.bus, sh.gs_mw, -bs);
    }
  }

  for (const auto& gen : ac.generators) {
    if (!gen.in_service) continue;
    if (gen.is_slack) continue;
    add_injection(by_bus, gen.bus, -gen.pg_mw, -gen.qg_mvar);
    warnings.push_back(
        "Generator " + std::to_string(gen.index) +
        " is exported as a negative constant-power load; PV voltage regulation "
        "is not represented in this GridLAB-D snapshot.");
  }
  for (const auto& gen : ac.static_generators) {
    if (!gen.in_service) continue;
    const double scale = gen.scaling > 0.0 ? gen.scaling : 1.0;
    add_injection(by_bus, gen.bus, -scale * gen.p_mw, -scale * gen.q_mvar);
  }
  for (const auto& gen : ac.renewable_gens) {
    if (gen.in_service) add_injection(by_bus, gen.bus, -gen.p_mw, -gen.q_mvar);
  }
  for (const auto& gen : ac.pv_systems) {
    if (gen.in_service) add_injection(by_bus, gen.bus, -gen.p_mw, -gen.q_mvar);
  }
  for (const auto& storage : ac.storage) {
    if (storage.in_service) {
      add_injection(by_bus, storage.bus, -storage.p_mw, -storage.q_mvar);
    }
  }

  if (!sys.vsc_converters.empty() || !sys.dc.buses.empty() ||
      !sys.dc.branches.empty() || !sys.energy_routers.empty()) {
    if (options.include_converter_boundary_injections) {
      for (const auto& conv : sys.vsc_converters) {
        if (!conv.in_service) continue;
        add_injection(by_bus, conv.bus_ac, -conv.p_set_mw, -conv.q_set_mvar);
      }
      warnings.push_back(
          "VSC converters were exported as scheduled AC boundary injections; "
          "DC-network and converter-control equations are not represented.");
    } else if (options.ac_scope_only) {
      warnings.push_back(
          "DC networks, energy routers, and VSC internals are outside the "
          "current GridLAB-D AC-scope harness and were not exported.");
    }
  }

  return by_bus;
}

std::unordered_map<int, BusShuntAdmittance> aggregate_shunt_admittance(
    const HybridPowerSystem& sys,
    const GridLABDExportOptions& options) {
  std::unordered_map<int, BusShuntAdmittance> by_bus;
  if (!options.include_shunt_admittance_as_impedance) return by_bus;

  const auto& ac = sys.ac;
  for (const auto& bus : ac.buses) {
    if (!bus.in_service) continue;
    add_shunt_admittance(by_bus, bus.index, bus.gs_mw, bus.bs_mvar);
  }
  for (const auto& sh : ac.shunts) {
    if (!sh.in_service) continue;
    double bs = sh.bs_mvar;
    if (sh.switchable && sh.n_steps > 0) {
      bs = sh.bs_per_step * sh.current_step;
    }
    add_shunt_admittance(by_bus, sh.bus, sh.gs_mw, bs);
  }
  return by_bus;
}

std::string branch_origin_type_to_string(BranchOriginType type) {
  switch (type) {
    case BranchOriginType::Transformer2W:
      return "Transformer2W";
    case BranchOriginType::Transformer3W:
      return "Transformer3W";
    case BranchOriginType::Switch:
      return "Switch";
    case BranchOriginType::CircuitBreaker:
      return "CircuitBreaker";
  }
  return "ACBranch";
}

std::map<int, BranchExpandEntry> branch_origin_by_index(
    const HybridPowerSystem& sys) {
  std::map<int, BranchExpandEntry> out;
  if (!sys.branch_expand_map.has_value()) return out;
  for (const auto& entry : sys.branch_expand_map->entries) {
    out[entry.branch_index] = entry;
  }
  return out;
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  bool quoted = false;
  for (char ch : line) {
    if (ch == '"') {
      quoted = !quoted;
      continue;
    }
    if (ch == ',' && !quoted) {
      out.push_back(field);
      field.clear();
    } else {
      field.push_back(ch);
    }
  }
  out.push_back(field);
  return out;
}

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

std::string normalize_header(std::string value) {
  value = trim(std::move(value));
  if (!value.empty() && value.front() == '#') {
    value.erase(value.begin());
    value = trim(std::move(value));
  }
  const auto unit_pos = value.find('[');
  if (unit_pos != std::string::npos) value = value.substr(0, unit_pos);
  return ascii_lower(trim(std::move(value)));
}

std::optional<double> parse_double_field(const std::vector<std::string>& row,
                                         const std::vector<std::string>& header,
                                         std::initializer_list<std::string> names,
                                         std::size_t fallback_pos) {
  for (const auto& name : names) {
    const std::string wanted = ascii_lower(name);
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (normalize_header(header[i]) == wanted && i < row.size()) {
        try {
          return std::stod(trim(row[i]));
        } catch (const std::exception&) {
          return std::nullopt;
        }
      }
    }
  }
  if (fallback_pos < row.size()) {
    try {
      return std::stod(trim(row[fallback_pos]));
    } catch (const std::exception&) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

struct ParsedCsv {
  std::vector<std::string> header;
  std::vector<std::string> last_row;
};

std::optional<ParsedCsv> parse_recorder_csv(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) return std::nullopt;

  std::vector<std::string> header;
  std::vector<std::string> last_row;
  std::string line;
  while (std::getline(in, line)) {
    line = trim(std::move(line));
    if (line.empty()) continue;
    if (line.front() == '#') {
      if (line.find(',') != std::string::npos) {
        header = split_csv_line(line);
      }
      continue;
    }
    const auto row = split_csv_line(line);
    if (header.empty()) {
      bool has_alpha = false;
      for (const auto& field : row) {
        has_alpha = has_alpha ||
                    std::any_of(field.begin(), field.end(), [](unsigned char ch) {
                      return std::isalpha(ch) != 0;
                    });
      }
      if (has_alpha) {
        header = row;
        continue;
      }
    }
    last_row = row;
  }
  if (last_row.empty()) return std::nullopt;
  return ParsedCsv{std::move(header), std::move(last_row)};
}

std::string read_text_file(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::filesystem::path temp_snapshot_dir() {
  static std::atomic<unsigned long long> counter{1};
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  std::filesystem::path base = std::filesystem::temp_directory_path();
  return base / ("hacdcpf_gridlabd_" + std::to_string(now) + "_" +
                 std::to_string(counter.fetch_add(1)));
}

int run_process_posix(const std::filesystem::path& executable,
                      const std::filesystem::path& glm_path,
                      const std::filesystem::path& working_dir,
                      const std::filesystem::path& stdout_path,
                      const std::filesystem::path& stderr_path,
                      const std::vector<std::pair<std::string, std::string>>& env,
                      int timeout_seconds) {
#ifdef _WIN32
  (void)timeout_seconds;
  // cmd.exe strips the first quote from a /C command that starts with a
  // quoted executable. Wrap the complete command and keep every path quoted.
  std::string command = "\"";
  for (const auto& [name, value] : env) {
    command += "set \"" + name + "=" + value + "\" && ";
  }
  command += "\"" + executable.string() + "\" \"" +
             glm_path.filename().string() + "\" > \"" +
             stdout_path.string() + "\" 2> \"" +
             stderr_path.string() + "\"\"";
  const auto old = std::filesystem::current_path();
  std::filesystem::current_path(working_dir);
  const int code = std::system(command.c_str());
  std::filesystem::current_path(old);
  return code;
#else
  const pid_t pid = ::fork();
  if (pid < 0) {
    throw std::runtime_error("fork failed while launching GridLAB-D.");
  }
  if (pid == 0) {
    ::chdir(working_dir.c_str());
    const int out_fd =
        ::open(stdout_path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0644);
    const int err_fd =
        ::open(stderr_path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (out_fd >= 0) {
      ::dup2(out_fd, STDOUT_FILENO);
      ::close(out_fd);
    }
    if (err_fd >= 0) {
      ::dup2(err_fd, STDERR_FILENO);
      ::close(err_fd);
    }
    for (const auto& [name, value] : env) {
      ::setenv(name.c_str(), value.c_str(), 1);
    }
    ::execl(executable.c_str(), executable.c_str(), glm_path.filename().c_str(),
            static_cast<char*>(nullptr));
    ::_exit(127);
  }

  int status = 0;
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    const pid_t done = ::waitpid(pid, &status, WNOHANG);
    if (done == pid) break;
    if (done < 0) return -1;
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start);
    if (timeout_seconds > 0 && elapsed.count() > timeout_seconds) {
      ::kill(pid, SIGTERM);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if (::waitpid(pid, &status, WNOHANG) == 0) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
      }
      return 124;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (WIFEXITED(status)) return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
  return status;
#endif
}

const GridLABDBusMapping* find_bus_mapping(
    const std::vector<GridLABDBusMapping>& mappings,
    int bus_index) {
  for (const auto& mapping : mappings) {
    if (mapping.canonical_bus_index == bus_index) return &mapping;
  }
  return nullptr;
}

std::optional<int> position_for_bus_index(const std::vector<ACBus>& buses,
                                          int bus_index) {
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) {
    if (buses[static_cast<std::size_t>(i)].index == bus_index) return i;
  }
  return std::nullopt;
}

void add_comparison_item(GridLABDComparisonReport& report,
                         std::string kind,
                         std::string key,
                         double hacdcpf_value,
                         double gridlabd_value,
                         double tolerance,
                         std::string detail = {}) {
  const double diff = hacdcpf_value - gridlabd_value;
  GridLABDComparisonItem item;
  item.kind = std::move(kind);
  item.key = std::move(key);
  item.hacdcpf_value = hacdcpf_value;
  item.gridlabd_value = gridlabd_value;
  item.difference = diff;
  item.tolerance = tolerance;
  item.passed = std::abs(diff) <= tolerance;
  item.detail = std::move(detail);
  report.items.push_back(std::move(item));
}

struct GridLABDEquivalenceAssessment {
  std::string scope;
  std::string claim;
  std::vector<std::string> unsupported_features;
  std::vector<std::string> diagnostic_only_reasons;
};

void add_unique_reason(std::vector<std::string>& reasons, std::string reason) {
  if (reason.empty()) return;
  if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end()) {
    reasons.push_back(std::move(reason));
  }
}

template <typename Range>
int count_in_service(const Range& items) {
  return static_cast<int>(std::count_if(
      items.begin(), items.end(), [](const auto& item) {
        return item.in_service;
      }));
}

template <typename Range>
bool any_in_service(const Range& items) {
  return count_in_service(items) > 0;
}

bool has_dc_or_hybrid_scope(const HybridPowerSystem& sys) {
  return any_in_service(sys.dc.buses) || any_in_service(sys.dc.branches) ||
         any_in_service(sys.dc.loads) || any_in_service(sys.dc.storage) ||
         any_in_service(sys.dc.dc_storage) ||
         any_in_service(sys.dc.static_generators) ||
         any_in_service(sys.dc.dc_static_generators) ||
         any_in_service(sys.dc.pv_arrays) ||
         any_in_service(sys.dc.dcdc_converters) ||
         any_in_service(sys.dc.dc_circuit_breakers) ||
         any_in_service(sys.vsc_converters) ||
         any_in_service(sys.energy_routers) ||
         any_in_service(sys.mobile_storage) || any_in_service(sys.vpps) ||
         any_in_service(sys.microgrids);
}

bool has_three_phase_scope(const HybridPowerSystem& sys) {
  if (!sys.three_phase_ac.has_value()) return false;
  const auto& tp = *sys.three_phase_ac;
  return any_in_service(tp.buses) || any_in_service(tp.lines) ||
         any_in_service(tp.transformers) || any_in_service(tp.loads) ||
         any_in_service(tp.generators) || any_in_service(tp.external_grids) ||
         !tp.regulator_controls.empty();
}

GridLABDEquivalenceAssessment classify_gridlabd_equivalence_scope(
    const HybridPowerSystem& original,
    const GridLABDExportedSnapshot& snapshot,
    const GridLABDComparisonOptions& options) {
  GridLABDEquivalenceAssessment out;
  out.scope =
      "Balanced AC algebraic snapshot with one swing/source, PQ buses, "
      "constant-power P/Q injections, passive AC lines, and simple "
      "two-winding transformer equivalents.";

  if (!options.compare_bus_voltages) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Bus-voltage comparison is disabled, so no GridLAB-D voltage "
        "equivalence claim can be made for this run.");
  }
  if (!options.compare_branch_flows) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Branch-flow comparison is disabled, so this run is a solve/export "
        "diagnostic rather than a component-by-component equivalence check.");
  }

  const auto& ac = original.ac;
  int slack_buses = 0;
  int pv_buses = 0;
  int isolated_buses = 0;
  for (const auto& bus : ac.buses) {
    if (!bus.in_service) continue;
    if (bus.bus_type == BusType::SLACK) ++slack_buses;
    if (bus.bus_type == BusType::PV) ++pv_buses;
    if (bus.bus_type == BusType::ISOLATED) ++isolated_buses;
    if (std::abs(bus.gs_mw) > kTinyPowerMw ||
        std::abs(bus.bs_mvar) > kTinyPowerMw) {
      if (!options.export_options.include_shunt_admittance_as_impedance) {
        add_unique_reason(
            out.diagnostic_only_reasons,
            "Bus shunt conductance/susceptance is exported as an equivalent "
            "constant-power injection; voltage-dependent shunt admittance is "
            "not a one-to-one GridLAB-D object in this run.");
      }
    }
  }
  if (slack_buses == 0) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "No in-service SLACK bus was provided; the exporter selected a swing "
        "bus, so the source reference is not an exact user-specified match.");
  } else if (slack_buses > 1) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Multiple in-service SLACK buses are reduced to one GridLAB-D swing "
        "bus in the current harness.");
  }
  if (pv_buses > 0) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "PV bus voltage regulation is outside the current exact GridLAB-D "
        "snapshot scope; non-slack voltage controls are collapsed to fixed "
        "P/Q injections.");
  }
  if (isolated_buses > 0) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Isolated AC buses are present; the balanced GridLAB-D snapshot "
        "does not currently preserve island/de-energized bus semantics.");
  }

  for (const auto& branch : ac.branches) {
    if (!branch.in_service) continue;
    if (std::abs(branch.shift_deg) > 1e-9) {
      add_unique_reason(
          out.unsupported_features,
          "Phase-shifting AC branches are not exported to GridLAB-D by the "
          "current bridge.");
    }
    if (std::hypot(branch.r_pu, branch.x_pu) < kTinyImpedance) {
      add_unique_reason(
          out.unsupported_features,
          "Zero-impedance AC branches are skipped by the current GridLAB-D "
          "bridge and require bus-merging or explicit switch mapping first.");
    }
  }

  for (const auto& load : ac.loads) {
    if (!load.in_service) continue;
    if (load.model != LoadModel::ConstantPower) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Voltage-dependent ZIP/exponential load models are collapsed to "
          "constant P/Q in the GridLAB-D comparison snapshot.");
    }
    if (load.motor_percent > 0.0) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Motor fractions on static loads are not exported as dynamic or "
          "voltage-dependent GridLAB-D motor models in this harness.");
    }
  }

  if (any_in_service(ac.flexible_loads)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Flexible-load controllability is projected to fixed P/Q demand for "
        "the GridLAB-D snapshot.");
  }
  if (any_in_service(ac.asymmetric_loads)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Asymmetric/per-phase AC loads are aggregated to a balanced "
        "three-phase load for the current GridLAB-D comparison.");
  }
  if (any_in_service(ac.motors)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Asynchronous motors are represented as static equivalent demand; "
        "motor dynamic and short-circuit behavior is native-only here.");
  }
  for (const auto& shunt : ac.shunts) {
    if (!shunt.in_service) continue;
    if (shunt.switchable) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          options.export_options.include_shunt_admittance_as_impedance
              ? "Switchable shunts are exported at their current fixed step; "
                "step-control behavior is not replayed in GridLAB-D."
              : "Switchable shunts are exported as fixed equivalent P/Q "
                "injections, so tap/step control is not exact.");
    } else if (!options.export_options.include_shunt_admittance_as_impedance) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Fixed shunts are exported as equivalent P/Q injections rather "
          "than explicit voltage-dependent shunt admittances in this run.");
    }
  }

  for (const auto& gen : ac.generators) {
    if (!gen.in_service || gen.is_slack) continue;
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Non-slack synchronous generators are exported as negative "
        "constant-power loads; generator voltage regulation, Q limits, and "
        "dynamic source behavior are not exact in the GridLAB-D snapshot.");
  }
  for (const auto& gen : ac.static_generators) {
    if (!gen.in_service) continue;
    if (gen.controllable || gen.k_p != 0.0 || gen.k_q != 0.0) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Controllable static-generator behavior is reduced to a fixed P/Q "
          "boundary injection for the GridLAB-D snapshot.");
    }
  }
  for (const auto& gen : ac.renewable_gens) {
    if (!gen.in_service) continue;
    if (gen.curtailable || gen.qmax_mvar != 0.0 || gen.qmin_mvar != 0.0) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Renewable generator controls and capability limits are reduced to "
          "fixed P/Q injections in the GridLAB-D snapshot.");
    }
  }
  for (const auto& pv : ac.pv_systems) {
    if (!pv.in_service) continue;
    if (pv.controllable || pv.control_mode != PVControlMode::MPPT ||
        pv.qmax_mvar != 0.0 || pv.qmin_mvar != 0.0) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "PV inverter controls are reduced to fixed P/Q injections in the "
          "GridLAB-D AC snapshot.");
    }
  }
  for (const auto& storage : ac.storage) {
    if (!storage.in_service) continue;
    if (storage.controllable || storage.grid_forming ||
        !storage.control_mode.empty()) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Storage converter controls, grid-forming behavior, and SOC "
          "dynamics are reduced to fixed P/Q injections in the GridLAB-D "
          "snapshot.");
    }
  }

  if (any_in_service(ac.transformers_3w)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Three-winding transformers are projected to equivalent branches "
        "before GridLAB-D export; this is not a one-to-one component match.");
  }
  for (const auto& transformer : ac.transformers_2w) {
    if (!transformer.in_service) continue;
    if (std::abs(transformer.shift_deg) > 1e-9) {
      add_unique_reason(
          out.unsupported_features,
          "Phase-shifting two-winding transformers are not exactly exported "
          "to GridLAB-D by the current bridge.");
    }
    if (transformer.tap_step_percent != 0.0 ||
        transformer.tap_min != transformer.tap_max) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Transformer tap metadata is projected to a static transformer "
          "equivalent; tap-controller behavior is not replayed in GridLAB-D.");
    }
    if (!transformer.vector_group.empty()) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Transformer vector-group semantics are not yet matched "
          "component-by-component in the balanced GridLAB-D export.");
    }
  }
  if (std::any_of(ac.regulator_controls.begin(), ac.regulator_controls.end(),
                  [](const RegulatorControl& control) {
                    return control.enabled;
                  })) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Regulator controls are not exported as GridLAB-D tap-control "
        "objects yet; only the static projected network is compared.");
  }
  if (any_in_service(ac.charging_stations) || any_in_service(ac.chargers)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "EV charging stations/chargers are aggregated to fixed P/Q demand in "
        "the GridLAB-D snapshot.");
  }

  if (has_dc_or_hybrid_scope(original)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        options.export_options.include_converter_boundary_injections
            ? "Hybrid AC/DC and converter internals are represented only by "
              "scheduled AC boundary injections in this GridLAB-D run."
            : "Hybrid AC/DC networks, converters, energy routers, and mobile "
              "storage are outside the current GridLAB-D AC-scope equivalence "
              "claim.");
  }
  if (has_three_phase_scope(original)) {
    add_unique_reason(
        out.diagnostic_only_reasons,
        "Native phase-domain AC components are reduced to a balanced "
        "positive-sequence snapshot before GridLAB-D comparison.");
  }

  for (const auto& mapping : snapshot.branch_mappings) {
    if (mapping.exported) continue;
    if (mapping.skip_reason == "branch is out of service") {
      continue;
    }
    if (mapping.exported_as_parallel_equivalent &&
        mapping.skip_reason.find("merged into") != std::string::npos) {
      add_unique_reason(
          out.diagnostic_only_reasons,
          "Parallel AC branches are merged into an equivalent GridLAB-D line; "
          "network voltages are comparable, but individual parallel-branch "
          "flows are not one-to-one recorder quantities.");
      continue;
    }
    add_unique_reason(
        out.unsupported_features,
        "AC branch " + std::to_string(mapping.canonical_branch_index) +
            " was not exported to GridLAB-D (" + mapping.skip_reason + ").");
  }

  const bool has_transformer_export = std::any_of(
      snapshot.branch_mappings.begin(), snapshot.branch_mappings.end(),
      [](const GridLABDBranchMapping& mapping) {
        return mapping.exported && mapping.exported_as_transformer;
      });

  if (!out.unsupported_features.empty()) {
    out.claim =
        "No GridLAB-D equivalence claim: at least one AC network feature is "
        "not represented by the generated GLM.";
  } else if (!out.diagnostic_only_reasons.empty()) {
    out.claim =
        "Diagnostic-only GridLAB-D run: the exported GLM is useful for "
        "spot-checking the solved AC snapshot, but it is outside the exact "
        "balanced AC PQ-feeder equivalence scope.";
  } else {
    out.claim =
        "Eligible GridLAB-D equivalence scope: this balanced AC algebraic "
        "snapshot can support an equivalence claim after a successful "
        "external GridLAB-D solve and numerical comparison. This does not "
        "claim transient, unbalanced phase-domain, protection, DER-control, "
        "regulator-control, or full GridLAB-D object-library equivalence.";
    if (has_transformer_export && !options.compare_transformer_branch_flows) {
      out.claim +=
          " Transformer terminal voltages are included; transformer branch "
          "flows are excluded from the default pass/fail unless "
          "compare_transformer_branch_flows is enabled.";
    }
  }
  return out;
}

}  // namespace

GridLABDExecutable discover_gridlabd() {
  GridLABDExecutable result;

  auto probe_executable = [&](const std::filesystem::path& path) {
    if (path.empty()) return;
    result.search_paths.push_back(path.string());
    if (!result.available && is_executable_file(path)) {
      result.available = true;
      result.path = std::filesystem::absolute(path);
    }
  };

  auto add_candidate = [&](const std::filesystem::path& path) {
    if (path.empty()) return;
    probe_executable(path);
    std::error_code ec;
    if (!std::filesystem::is_directory(path, ec) || ec) return;
    probe_executable(path / "gridlabd");
    probe_executable(path / "bin" / "gridlabd");
    probe_executable(path / "cmake-build" / "gridlabd");
    probe_executable(path / "cmake-build" / "bin" / "gridlabd");
    probe_executable(path / "cmake-build" / "source" / "gridlabd");
    probe_executable(path / "build" / "gridlabd");
    probe_executable(path / "build" / "bin" / "gridlabd");
    probe_executable(path / "build" / "source" / "gridlabd");
    probe_executable(path / "build-macos" / "gridlabd");
    probe_executable(path / "build-macos" / "bin" / "gridlabd");
  };

  add_candidate(getenv_string("HACDCPF_GRIDLABD_BIN"));
  add_candidate(getenv_string("GRIDLABD_BIN"));

  for (const auto& dir : split_path_list(getenv_string("PATH"))) {
    add_candidate(dir / "gridlabd");
#ifdef _WIN32
    add_candidate(dir / "gridlabd.exe");
#endif
  }

  const auto root = project_root_path();
  const auto sibling = root.parent_path() / "gridlab-d";
  add_candidate(sibling);
  add_candidate(sibling / "gridlabd");
  add_candidate(sibling / "bin" / "gridlabd");
  add_candidate(sibling / "cmake-build");
  add_candidate(sibling / "cmake-build" / "gridlabd");
  add_candidate(sibling / "cmake-build" / "bin" / "gridlabd");
  add_candidate(sibling / "cmake-build" / "source" / "gridlabd");
  add_candidate(sibling / "build" / "gridlabd");
  add_candidate(sibling / "build" / "bin" / "gridlabd");
  add_candidate(sibling / "build" / "source" / "gridlabd");
  add_candidate(sibling / "build-macos" / "gridlabd");
  add_candidate(sibling / "build-macos" / "bin" / "gridlabd");

  if (!result.available) {
    result.diagnostics.push_back(
        "GridLAB-D executable was not found. Set HACDCPF_GRIDLABD_BIN or "
        "GRIDLABD_BIN, add gridlabd to PATH, or build the sibling gridlab-d "
        "checkout before running external comparisons.");
  }
  return result;
}

GridLABDExportedSnapshot export_gridlabd_snapshot(
    const HybridPowerSystem& sys,
    const std::filesystem::path& working_directory,
    const GridLABDExportOptions& options) {
  GridLABDExportedSnapshot snapshot;
  snapshot.working_directory = std::filesystem::absolute(working_directory);
  std::filesystem::create_directories(snapshot.working_directory);
  snapshot.glm_path =
      snapshot.working_directory /
      (sanitize_identifier(options.model_name, "hacdcpf_gridlabd_snapshot") +
       ".glm");

  snapshot.canonical_system =
      options.project_to_canonical ? project_to_canonical_models(sys) : sys;
  auto& canonical = snapshot.canonical_system;
  convert_actual_to_per_unit(canonical);

  if (canonical.ac.buses.empty()) {
    throw std::runtime_error("export_gridlabd_snapshot: AC bus table is empty.");
  }

  const auto bus_pos = bus_pos_by_index(canonical.ac);
  auto demand_by_bus =
      aggregate_constant_power_demand(canonical, snapshot.warnings, options);
  auto shunt_by_bus = aggregate_shunt_admittance(canonical, options);
  const auto slack_vm = slack_vm_by_bus(canonical.ac);
  const auto slack_va = slack_va_by_bus(canonical.ac);

  int swing_bus = 0;
  for (const auto& bus : canonical.ac.buses) {
    if (bus.in_service && bus.bus_type == BusType::SLACK) {
      swing_bus = bus.index;
      break;
    }
  }
  if (swing_bus == 0) {
    swing_bus = canonical.ac.buses.front().index;
    snapshot.warnings.push_back(
        "No SLACK AC bus was found; the first in-service bus was exported as "
        "the GridLAB-D swing bus.");
  }

  std::unordered_set<std::string> used_names;
  std::unordered_map<int, std::string> bus_name_by_index;
  for (int i = 0; i < static_cast<int>(canonical.ac.buses.size()); ++i) {
    const auto& bus = canonical.ac.buses[static_cast<std::size_t>(i)];
    const std::string raw =
        bus.name.empty() ? ("bus_" + std::to_string(bus.index))
                         : ("bus_" + std::to_string(bus.index) + "_" + bus.name);
    const std::string name = unique_identifier(raw, used_names);
    bus_name_by_index[bus.index] = name;

    const double base_kv = bus_base_kv(bus, options);
    GridLABDBusMapping mapping;
    mapping.canonical_bus_index = bus.index;
    mapping.canonical_bus_position = i;
    mapping.source_name = bus.name;
    mapping.gridlabd_name = name;
    mapping.base_kv_ll = base_kv;
    mapping.nominal_voltage_ln_volts = base_kv * 1000.0 / kSqrt3;
    mapping.is_swing = (bus.index == swing_bus);
    snapshot.bus_mappings.push_back(std::move(mapping));
  }

  const auto origin_by_branch = branch_origin_by_index(canonical);
  for (int i = 0; i < static_cast<int>(canonical.ac.branches.size()); ++i) {
    const auto& branch = canonical.ac.branches[static_cast<std::size_t>(i)];
    GridLABDBranchMapping mapping;
    mapping.canonical_branch_index = branch.index;
    mapping.canonical_branch_position = i;
    mapping.canonical_name = branch.name;
    mapping.from_bus = branch.from_bus;
    mapping.to_bus = branch.to_bus;
    mapping.gridlabd_name =
        unique_identifier(branch.name.empty()
                              ? ("branch_" + std::to_string(branch.index))
                              : ("branch_" + std::to_string(branch.index) + "_" +
                                 branch.name),
                          used_names);
    mapping.gridlabd_configuration_name =
        unique_identifier("cfg_" + mapping.gridlabd_name, used_names);
    const auto origin_it = origin_by_branch.find(branch.index);
    if (origin_it != origin_by_branch.end()) {
      mapping.origin_type = branch_origin_type_to_string(origin_it->second.origin_type);
      mapping.origin_index = origin_it->second.origin_index;
    }
    const auto from_it = bus_pos.find(branch.from_bus);
    const auto to_it = bus_pos.find(branch.to_bus);
    if (!branch.in_service) {
      mapping.skip_reason = "branch is out of service";
    } else if (bus_name_by_index.count(branch.from_bus) == 0U ||
               bus_name_by_index.count(branch.to_bus) == 0U) {
      mapping.skip_reason = "branch endpoint is missing from exported AC buses";
    } else if (std::hypot(branch.r_pu, branch.x_pu) < kTinyImpedance) {
      mapping.skip_reason = "zero-impedance branch is not exported";
    } else if (std::abs(branch.shift_deg) > 1e-9) {
      mapping.skip_reason = "phase-shift transformer is not yet represented";
    } else if (from_it == bus_pos.end() || to_it == bus_pos.end()) {
      mapping.skip_reason = "branch endpoint position is missing";
    } else {
      mapping.exported = true;
      const auto& from_bus = canonical.ac.buses[static_cast<std::size_t>(from_it->second)];
      const auto& to_bus = canonical.ac.buses[static_cast<std::size_t>(to_it->second)];
      mapping.exported_as_transformer =
          is_gridlabd_transformer_branch(branch, from_bus, to_bus, options);
    }
    if (!mapping.exported) {
      snapshot.warnings.push_back(
          "AC branch " + std::to_string(branch.index) + " (" +
          (branch.name.empty() ? mapping.gridlabd_name : branch.name) +
          ") skipped in GridLAB-D export: " + mapping.skip_reason + ".");
    }
    snapshot.branch_mappings.push_back(std::move(mapping));
  }

  if (options.merge_parallel_branches) {
    std::map<std::pair<int, int>, std::vector<int>> plain_line_groups;
    for (int i = 0; i < static_cast<int>(snapshot.branch_mappings.size()); ++i) {
      const auto& mapping = snapshot.branch_mappings[static_cast<std::size_t>(i)];
      if (!mapping.exported || mapping.exported_as_transformer) continue;
      const auto a = std::min(mapping.from_bus, mapping.to_bus);
      const auto b = std::max(mapping.from_bus, mapping.to_bus);
      plain_line_groups[{a, b}].push_back(i);
    }

    for (const auto& [bus_pair, positions] : plain_line_groups) {
      if (positions.size() < 2U) continue;
      const int primary_pos = positions.front();
      auto& primary =
          snapshot.branch_mappings[static_cast<std::size_t>(primary_pos)];
      primary.exported_as_parallel_equivalent = true;
      primary.merged_canonical_branch_indices.clear();
      for (const int pos : positions) {
        const auto& member =
            snapshot.branch_mappings[static_cast<std::size_t>(pos)];
        primary.merged_canonical_branch_indices.push_back(
            member.canonical_branch_index);
      }

      for (std::size_t k = 1; k < positions.size(); ++k) {
        auto& member =
            snapshot.branch_mappings[static_cast<std::size_t>(positions[k])];
        member.exported = false;
        member.exported_as_parallel_equivalent = true;
        member.merged_into_canonical_branch_index =
            primary.canonical_branch_index;
        member.merged_canonical_branch_indices =
            primary.merged_canonical_branch_indices;
        member.gridlabd_name = primary.gridlabd_name;
        member.gridlabd_configuration_name = primary.gridlabd_configuration_name;
        member.skip_reason =
            "merged into GridLAB-D parallel equivalent branch " +
            std::to_string(primary.canonical_branch_index);
      }

      std::ostringstream msg;
      msg << "Parallel AC branches between bus " << bus_pair.first << " and "
          << bus_pair.second << " were exported as one GridLAB-D equivalent "
          << primary.gridlabd_name << " (canonical branches";
      for (const int branch_index : primary.merged_canonical_branch_indices) {
        msg << " " << branch_index;
      }
      msg << "). Branch-flow comparison is skipped for this equivalent because "
             "GridLAB-D reports only the aggregated component flow.";
      snapshot.warnings.push_back(msg.str());
    }
  }

  std::ofstream glm(snapshot.glm_path);
  if (!glm) {
    throw std::runtime_error("Failed to create GridLAB-D GLM at " +
                             snapshot.glm_path.string());
  }

  glm << "// Generated by hacdcpf GridLAB-D bridge.\n";
  if (options.gridlabd_iteration_limit > 0) {
    glm << "#set iteration_limit=" << options.gridlabd_iteration_limit << ";\n";
  }
  if (options.include_metadata_comments) {
    glm << "// Scope: balanced three-phase AC snapshot of the canonical network.\n";
    glm << "// DC networks and converter dynamics are validation extensions, not "
           "part of this GLM unless explicitly exported as boundary injections.\n";
  }
  glm << "\nclock {\n"
      << "  timezone GMT0;\n"
      << "  starttime '2000-01-01 00:00:00';\n"
      << "  stoptime '2000-01-01 00:01:00';\n"
      << "};\n\n";
  glm << "module powerflow {\n"
      << "  solver_method NR;\n"
      << "  nominal_frequency " << format_double(options.nominal_frequency_hz)
      << ";\n";
  if (options.gridlabd_nr_iteration_limit > 0) {
    glm << "  NR_iteration_limit " << options.gridlabd_nr_iteration_limit
        << ";\n";
  }
  if (options.include_line_capacitance) {
    glm << "  line_capacitance TRUE;\n";
  }
  glm << "};\n\nmodule tape;\n\n";

  for (const auto& mapping : snapshot.bus_mappings) {
    const auto& bus =
        canonical.ac.buses[static_cast<std::size_t>(mapping.canonical_bus_position)];
    const auto demand_it = demand_by_bus.find(bus.index);
    const bool has_load =
        demand_it != demand_by_bus.end() &&
        (std::abs(demand_it->second.p_mw) > kTinyPowerMw ||
         std::abs(demand_it->second.q_mvar) > kTinyPowerMw);
    const auto shunt_it = shunt_by_bus.find(bus.index);
    const bool has_shunt =
        shunt_it != shunt_by_bus.end() &&
        (std::abs(shunt_it->second.gs_mw) > kTinyPowerMw ||
         std::abs(shunt_it->second.bs_mvar) > kTinyPowerMw);
    const std::string object_class = (has_load || has_shunt) ? "load" : "meter";
    glm << "object " << object_class << " {\n";
    glm << "  name " << mapping.gridlabd_name << ";\n";
    glm << "  phases ABCN;\n";
    glm << "  nominal_voltage "
        << format_double(mapping.nominal_voltage_ln_volts) << ";\n";
    const double vm =
        slack_vm.count(bus.index) != 0U ? slack_vm.at(bus.index)
                                        : (bus.vm_pu > 0.0 ? bus.vm_pu : 1.0);
    const double va =
        slack_va.count(bus.index) != 0U ? slack_va.at(bus.index) : bus.va_deg;
    const double vln = mapping.nominal_voltage_ln_volts * vm;
    glm << "  voltage_A " << format_complex_polar(vln, va) << ";\n";
    glm << "  voltage_B " << format_complex_polar(vln, va - 120.0) << ";\n";
    glm << "  voltage_C " << format_complex_polar(vln, va + 120.0) << ";\n";
    if (mapping.is_swing) {
      glm << "  bustype SWING;\n";
    }
    if (has_load) {
      const auto demand = demand_it->second;
      const double p_w = demand.p_mw * 1.0e6 / 3.0;
      const double q_var = demand.q_mvar * 1.0e6 / 3.0;
      glm << "  constant_power_A " << format_complex_rect(p_w, q_var) << ";\n";
      glm << "  constant_power_B " << format_complex_rect(p_w, q_var) << ";\n";
      glm << "  constant_power_C " << format_complex_rect(p_w, q_var) << ";\n";
      snapshot.load_mappings.push_back(GridLABDLoadMapping{
          .canonical_bus_index = bus.index,
          .gridlabd_name = "load_at_" + mapping.gridlabd_name,
          .p_mw = demand.p_mw,
          .q_mvar = demand.q_mvar,
      });
    }
    if (has_shunt) {
      const double base_mva =
          canonical.base_mva > 0.0 ? canonical.base_mva : canonical.ac.base_mva;
      const double zbase_ohm =
          (mapping.base_kv_ll * mapping.base_kv_ll) /
          std::max(1e-9, base_mva);
      const std::complex<double> y_pu(shunt_it->second.gs_mw / base_mva,
                                      shunt_it->second.bs_mvar / base_mva);
      if (std::abs(y_pu) > kTinyImpedance) {
        const std::complex<double> z_ohm = zbase_ohm / y_pu;
        glm << "  constant_impedance_A "
            << format_complex_rect(z_ohm.real(), z_ohm.imag()) << ";\n";
        glm << "  constant_impedance_B "
            << format_complex_rect(z_ohm.real(), z_ohm.imag()) << ";\n";
        glm << "  constant_impedance_C "
            << format_complex_rect(z_ohm.real(), z_ohm.imag()) << ";\n";
      }
    }
    glm << "};\n\n";

    if (options.include_load_recorders) {
      glm << "object recorder {\n";
      glm << "  parent " << mapping.gridlabd_name << ";\n";
      glm << "  file \"" << "bus_" << mapping.gridlabd_name << ".csv\";\n";
      glm << "  interval 60;\n";
      glm << "  property \"voltage_A.real,voltage_A.imag\";\n";
      glm << "};\n\n";
    }
  }

  for (const auto& mapping : snapshot.branch_mappings) {
    if (!mapping.exported) continue;
    const auto branch_positions =
        merged_branch_positions(snapshot.branch_mappings, mapping);
    const ACBranch branch =
        mapping.exported_as_parallel_equivalent
            ? make_parallel_equivalent_branch(canonical.ac,
                                              branch_positions,
                                              mapping.gridlabd_name +
                                                  "_parallel_eq")
            : canonical.ac.branches[static_cast<std::size_t>(
                  mapping.canonical_branch_position)];
    const auto from_it = bus_pos.find(branch.from_bus);
    if (from_it == bus_pos.end()) continue;
    const auto& from_bus = canonical.ac.buses[static_cast<std::size_t>(from_it->second)];
    const auto to_it = bus_pos.find(branch.to_bus);
    if (to_it == bus_pos.end()) continue;
    const auto& to_bus = canonical.ac.buses[static_cast<std::size_t>(to_it->second)];
    const auto* from_mapping = find_bus_mapping(snapshot.bus_mappings, branch.from_bus);
    const auto* to_mapping = find_bus_mapping(snapshot.bus_mappings, branch.to_bus);
    if (from_mapping == nullptr || to_mapping == nullptr) continue;
    const double base_kv = bus_base_kv(from_bus, options);
    const double base_mva =
        canonical.base_mva > 0.0 ? canonical.base_mva : canonical.ac.base_mva;
    if (mapping.exported_as_transformer) {
      const double rating_mva =
          branch.rate_a_mva > 0.0 ? branch.rate_a_mva : base_mva;
      const double z_scale = rating_mva / std::max(1e-9, base_mva);
      const double resistance_pu =
          std::max(1e-9, branch.r_pu * z_scale);
      const double reactance_pu =
          std::max(1e-9, branch.x_pu * z_scale);
      const double tap = std::max(1e-6, branch.tap);

      glm << "object transformer_configuration {\n";
      glm << "  name " << mapping.gridlabd_configuration_name << ";\n";
      glm << "  connect_type WYE_WYE;\n";
      glm << "  install_type PADMOUNT;\n";
      glm << "  power_rating " << format_double(rating_mva * 1000.0) << ";\n";
      glm << "  primary_voltage "
          << format_double(bus_base_kv(from_bus, options) * 1000.0 * tap)
          << ";\n";
      glm << "  secondary_voltage "
          << format_double(bus_base_kv(to_bus, options) * 1000.0) << ";\n";
      glm << "  resistance " << format_double(resistance_pu) << ";\n";
      glm << "  reactance " << format_double(reactance_pu) << ";\n";
      glm << "};\n\n";

      glm << "object transformer {\n";
      glm << "  name " << mapping.gridlabd_name << ";\n";
      glm << "  phases ABCN;\n";
      glm << "  from " << from_mapping->gridlabd_name << ";\n";
      glm << "  to " << to_mapping->gridlabd_name << ";\n";
      glm << "  configuration " << mapping.gridlabd_configuration_name << ";\n";
      glm << "};\n\n";

      if (options.include_branch_recorders) {
        glm << "object recorder {\n";
        glm << "  parent " << mapping.gridlabd_name << ";\n";
        glm << "  file \"" << "branch_" << mapping.gridlabd_name << ".csv\";\n";
        glm << "  interval 60;\n";
        glm << "  property "
               "\"power_in.real,power_in.imag,power_out.real,power_out.imag,"
               "power_losses.real,power_losses.imag\";\n";
        glm << "};\n\n";
      }
      continue;
    }

    const double zbase_ohm = (base_kv * base_kv) / std::max(1e-9, base_mva);
    const std::complex<double> z1(branch.r_pu * zbase_ohm,
                                  branch.x_pu * zbase_ohm);
    const std::complex<double> z0(
        (std::hypot(branch.r0_pu, branch.x0_pu) > kTinyImpedance
             ? branch.r0_pu
             : branch.r_pu) *
            zbase_ohm,
        (std::hypot(branch.r0_pu, branch.x0_pu) > kTinyImpedance
             ? branch.x0_pu
             : branch.x_pu) *
            zbase_ohm);
    const std::complex<double> z_self = (z0 + 2.0 * z1) / 3.0;
    const std::complex<double> z_mutual = (z0 - z1) / 3.0;

    glm << "object line_configuration {\n";
    glm << "  name " << mapping.gridlabd_configuration_name << ";\n";
    glm << "  z11 " << format_complex_rect(z_self.real(), z_self.imag())
        << " Ohm/mile;\n";
    glm << "  z12 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z13 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z21 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z22 " << format_complex_rect(z_self.real(), z_self.imag())
        << " Ohm/mile;\n";
    glm << "  z23 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z31 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z32 " << format_complex_rect(z_mutual.real(), z_mutual.imag())
        << " Ohm/mile;\n";
    glm << "  z33 " << format_complex_rect(z_self.real(), z_self.imag())
        << " Ohm/mile;\n";
    if (options.include_line_capacitance &&
        std::abs(branch.b_pu) > kTinyImpedance) {
      const double b_total_siemens = branch.b_pu / std::max(1e-12, zbase_ohm);
      const double c_f = b_total_siemens /
                         (2.0 * kPi *
                          std::max(1e-6, options.nominal_frequency_hz));
      const double c_nf = c_f * 1.0e9;
      glm << "  c11 " << format_double(c_nf) << " nF/mile;\n";
      glm << "  c22 " << format_double(c_nf) << " nF/mile;\n";
      glm << "  c33 " << format_double(c_nf) << " nF/mile;\n";
    }
    glm << "};\n\n";

    glm << "object overhead_line {\n";
    glm << "  name " << mapping.gridlabd_name << ";\n";
    glm << "  phases ABC;\n";
    glm << "  from " << from_mapping->gridlabd_name << ";\n";
    glm << "  to " << to_mapping->gridlabd_name << ";\n";
    glm << "  length 1 mile;\n";
    glm << "  configuration " << mapping.gridlabd_configuration_name << ";\n";
    glm << "};\n\n";

    if (options.include_branch_recorders) {
      glm << "object recorder {\n";
      glm << "  parent " << mapping.gridlabd_name << ";\n";
      glm << "  file \"" << "branch_" << mapping.gridlabd_name << ".csv\";\n";
      glm << "  interval 60;\n";
      glm << "  property "
             "\"power_in.real,power_in.imag,power_out.real,power_out.imag,"
             "power_losses.real,power_losses.imag\";\n";
      glm << "};\n\n";
    }
  }

  return snapshot;
}

GridLABDRunResult run_gridlabd_snapshot(
    const GridLABDExportedSnapshot& snapshot,
    const GridLABDRunOptions& options) {
  GridLABDRunResult result;
  GridLABDExecutable discovered;
  if (options.executable.has_value()) {
    discovered.available = is_executable_file(*options.executable);
    discovered.path = *options.executable;
    if (!discovered.available) {
      discovered.diagnostics.push_back("Configured GridLAB-D executable is not runnable: " +
                                       options.executable->string());
    }
  } else {
    discovered = discover_gridlabd();
  }
  result.executable = discovered.path;
  for (const auto& diag : discovered.diagnostics) result.warnings.push_back(diag);
  if (!discovered.available) {
    return result;
  }

  result.attempted = true;
  const auto stdout_path = snapshot.working_directory / "gridlabd.stdout.txt";
  const auto stderr_path = snapshot.working_directory / "gridlabd.stderr.txt";
  result.command = "\"" + discovered.path.string() + "\" \"" +
                   snapshot.glm_path.filename().string() + "\"";
  const auto runtime_paths = gridlabd_runtime_paths(discovered.path);
  std::vector<std::pair<std::string, std::string>> env;
  if (!runtime_paths.empty()) {
    env.emplace_back("GLPATH",
                     join_unique_paths(runtime_paths, getenv_string("GLPATH")));
  }
  result.exit_code = run_process_posix(discovered.path, snapshot.glm_path,
                                       snapshot.working_directory, stdout_path,
                                       stderr_path, env, options.timeout_seconds);
  result.stdout_text = read_text_file(stdout_path);
  result.stderr_text = read_text_file(stderr_path);
  result.success = (result.exit_code == 0);
  if (!result.success) {
    result.warnings.push_back("GridLAB-D exited with code " +
                              std::to_string(result.exit_code) + ".");
    return result;
  }

  for (const auto& mapping : snapshot.bus_mappings) {
    const auto path =
        snapshot.working_directory / ("bus_" + mapping.gridlabd_name + ".csv");
    const auto parsed = parse_recorder_csv(path);
    if (!parsed.has_value()) {
      result.warnings.push_back("Missing or empty GridLAB-D bus recorder: " +
                                path.string());
      continue;
    }
    const auto real = parse_double_field(parsed->last_row, parsed->header,
                                         {"voltage_a.real"}, 1);
    const auto imag = parse_double_field(parsed->last_row, parsed->header,
                                         {"voltage_a.imag"}, 2);
    if (!real.has_value() || !imag.has_value()) {
      result.warnings.push_back("Unable to parse voltage recorder: " +
                                path.string());
      continue;
    }
    GridLABDBusVoltage voltage;
    voltage.canonical_bus_index = mapping.canonical_bus_index;
    voltage.gridlabd_name = mapping.gridlabd_name;
    voltage.voltage_a_v = {*real, *imag};
    voltage.vm_a_pu =
        std::abs(voltage.voltage_a_v) /
        std::max(1e-9, mapping.nominal_voltage_ln_volts);
    voltage.va_a_deg = rad_to_deg(std::arg(voltage.voltage_a_v));
    result.bus_voltages.push_back(std::move(voltage));
  }

  for (const auto& mapping : snapshot.branch_mappings) {
    if (!mapping.exported) continue;
    const auto path = snapshot.working_directory /
                      ("branch_" + mapping.gridlabd_name + ".csv");
    const auto parsed = parse_recorder_csv(path);
    if (!parsed.has_value()) {
      result.warnings.push_back("Missing or empty GridLAB-D branch recorder: " +
                                path.string());
      continue;
    }
    const auto pin_r = parse_double_field(parsed->last_row, parsed->header,
                                          {"power_in.real"}, 1);
    const auto pin_i = parse_double_field(parsed->last_row, parsed->header,
                                          {"power_in.imag"}, 2);
    const auto pout_r = parse_double_field(parsed->last_row, parsed->header,
                                           {"power_out.real"}, 3);
    const auto pout_i = parse_double_field(parsed->last_row, parsed->header,
                                           {"power_out.imag"}, 4);
    const auto ploss_r = parse_double_field(parsed->last_row, parsed->header,
                                            {"power_losses.real"}, 5);
    const auto ploss_i = parse_double_field(parsed->last_row, parsed->header,
                                            {"power_losses.imag"}, 6);
    if (!pin_r || !pin_i || !pout_r || !pout_i || !ploss_r || !ploss_i) {
      result.warnings.push_back("Unable to parse branch recorder: " +
                                path.string());
      continue;
    }
    GridLABDBranchPower power;
    power.canonical_branch_index = mapping.canonical_branch_index;
    power.gridlabd_name = mapping.gridlabd_name;
    power.power_in_va = {*pin_r, *pin_i};
    power.power_out_va = {*pout_r, *pout_i};
    power.power_loss_va = {*ploss_r, *ploss_i};
    result.branch_powers.push_back(std::move(power));
  }

  return result;
}

GridLABDComparisonReport compare_gridlabd_snapshot(
    const HybridPowerSystem& sys,
    const GridLABDComparisonOptions& options) {
  GridLABDComparisonReport report;
  auto apply_equivalence_assessment = [&]() {
    const auto assessment = classify_gridlabd_equivalence_scope(
        sys, report.exported_snapshot, options);
    report.equivalence_scope = assessment.scope;
    report.equivalence_claim = assessment.claim;
    report.unsupported_features = assessment.unsupported_features;
    report.diagnostic_only_reasons = assessment.diagnostic_only_reasons;
  };
  const GridLABDExecutable discovered =
      options.run_options.executable.has_value()
          ? GridLABDExecutable{.available = is_executable_file(*options.run_options.executable),
                               .path = *options.run_options.executable}
          : discover_gridlabd();
  report.gridlabd_available = discovered.available;
  if (!discovered.available) {
    report.skipped.push_back(
        "GridLAB-D executable is unavailable; external comparison was skipped.");
    for (const auto& diag : discovered.diagnostics) report.warnings.push_back(diag);
    if (options.require_gridlabd) {
      report.passed = false;
    }
    report.exported_snapshot =
        export_gridlabd_snapshot(sys, temp_snapshot_dir(), options.export_options);
    apply_equivalence_assessment();
    return report;
  }

  report.exported_snapshot =
      export_gridlabd_snapshot(sys, temp_snapshot_dir(), options.export_options);
  apply_equivalence_assessment();

  try {
    report.hacdcpf_power_flow = solve_power_flow(sys);
    report.hacdcpf_power_flow_converged = report.hacdcpf_power_flow.converged;
  } catch (const std::exception& e) {
    report.warnings.push_back(std::string("HACDCPF power flow failed: ") + e.what());
    report.passed = false;
    return report;
  }
  if (!report.hacdcpf_power_flow.converged) {
    report.warnings.push_back(
        "HACDCPF power flow did not converge; GridLAB-D comparison is not meaningful.");
    report.passed = false;
    return report;
  }

  GridLABDRunOptions run_options = options.run_options;
  run_options.executable = discovered.path;
  report.gridlabd_result =
      run_gridlabd_snapshot(report.exported_snapshot, run_options);
  report.gridlabd_run_attempted = report.gridlabd_result.attempted;
  report.gridlabd_run_success = report.gridlabd_result.success;
  for (const auto& warning : report.exported_snapshot.warnings) {
    report.warnings.push_back(warning);
  }
  for (const auto& warning : report.gridlabd_result.warnings) {
    report.warnings.push_back(warning);
  }
  if (!report.gridlabd_result.success) {
    report.passed = false;
    return report;
  }

  std::unordered_map<int, GridLABDBusVoltage> gld_bus_by_index;
  for (const auto& voltage : report.gridlabd_result.bus_voltages) {
    gld_bus_by_index[voltage.canonical_bus_index] = voltage;
  }
  if (options.compare_bus_voltages) {
    for (const auto& mapping : report.exported_snapshot.bus_mappings) {
      const auto gld_it = gld_bus_by_index.find(mapping.canonical_bus_index);
      if (gld_it == gld_bus_by_index.end()) continue;
      const auto pf_pos = position_for_bus_index(sys.ac.buses,
                                                 mapping.canonical_bus_index);
      const int pos = pf_pos.value_or(mapping.canonical_bus_position);
      if (pos < 0 || pos >= static_cast<int>(report.hacdcpf_power_flow.vm.size()) ||
          pos >= static_cast<int>(report.hacdcpf_power_flow.va.size())) {
        report.warnings.push_back("Unable to map HACDCPF PF bus vector for bus " +
                                  std::to_string(mapping.canonical_bus_index));
        continue;
      }
      add_comparison_item(report, "bus_vm_pu", mapping.gridlabd_name,
                          report.hacdcpf_power_flow.vm[static_cast<std::size_t>(pos)],
                          gld_it->second.vm_a_pu, options.vm_tolerance_pu);
      add_comparison_item(
          report, "bus_va_deg", mapping.gridlabd_name,
          rad_to_deg(report.hacdcpf_power_flow.va[static_cast<std::size_t>(pos)]),
          gld_it->second.va_a_deg, options.va_tolerance_deg);
    }
  } else {
    report.skipped.push_back(
        "Bus-voltage numerical comparison disabled for this GridLAB-D run; "
        "the exported AC snapshot was still solved and recorder data parsed.");
  }

  if (options.compare_branch_flows) {
    std::unordered_map<int, GridLABDBranchPower> gld_branch_by_index;
    for (const auto& power : report.gridlabd_result.branch_powers) {
      gld_branch_by_index[power.canonical_branch_index] = power;
    }
    for (const auto& mapping : report.exported_snapshot.branch_mappings) {
      if (!mapping.exported) continue;
      if (mapping.exported_as_parallel_equivalent) {
        report.skipped.push_back(
            "Branch-flow comparison skipped for GridLAB-D parallel equivalent " +
            mapping.gridlabd_name + "; merged canonical branches are reported "
            "as one aggregated GridLAB-D component.");
        continue;
      }
      if (mapping.exported_as_transformer &&
          !options.compare_transformer_branch_flows) {
        report.skipped.push_back(
            "Branch-flow comparison skipped for GridLAB-D transformer " +
            mapping.gridlabd_name +
            "; transformer tap/rating semantics are validated separately.");
        continue;
      }
      const auto gld_it = gld_branch_by_index.find(mapping.canonical_branch_index);
      if (gld_it == gld_branch_by_index.end()) continue;
      const int pos = mapping.canonical_branch_position;
      if (pos < 0 ||
          pos >= static_cast<int>(report.hacdcpf_power_flow.branch_flows.size())) {
        report.warnings.push_back(
            "Unable to map HACDCPF branch flow for branch " +
            std::to_string(mapping.canonical_branch_index));
        continue;
      }
      const auto& flow =
          report.hacdcpf_power_flow.branch_flows[static_cast<std::size_t>(pos)];
      add_comparison_item(report, "branch_pf_mw", mapping.gridlabd_name,
                          flow.pf_mw, gld_it->second.power_in_va.real() / 1.0e6,
                          options.branch_p_tolerance_mw);
      add_comparison_item(report, "branch_qf_mvar", mapping.gridlabd_name,
                          flow.qf_mvar,
                          gld_it->second.power_in_va.imag() / 1.0e6,
                          options.branch_q_tolerance_mvar);
      add_comparison_item(report, "branch_loss_p_mw", mapping.gridlabd_name,
                          flow.pf_mw + flow.pt_mw,
                          gld_it->second.power_loss_va.real() / 1.0e6,
                          options.branch_p_tolerance_mw);
      add_comparison_item(report, "branch_loss_q_mvar", mapping.gridlabd_name,
                          flow.qf_mvar + flow.qt_mvar,
                          gld_it->second.power_loss_va.imag() / 1.0e6,
                          options.branch_q_tolerance_mvar);
    }
  }

  const bool all_items_passed =
      report.hacdcpf_power_flow_converged && report.gridlabd_run_success &&
      std::all_of(report.items.begin(), report.items.end(),
                  [](const GridLABDComparisonItem& item) { return item.passed; });
  report.numerical_comparison_passed = all_items_passed && !report.items.empty();
  report.equivalence_passed =
      report.numerical_comparison_passed &&
      report.unsupported_features.empty() &&
      report.diagnostic_only_reasons.empty();
  report.passed = report.equivalence_passed;
  if (report.equivalence_passed) {
    report.equivalence_claim =
        "HACDCPF is numerically equivalent to GridLAB-D for this declared "
        "balanced AC algebraic snapshot within the configured tolerances. "
        "This does not claim transient, unbalanced phase-domain, protection, "
        "DER-control, regulator-control, or full GridLAB-D object-library "
        "equivalence.";
    const bool has_transformer_export = std::any_of(
        report.exported_snapshot.branch_mappings.begin(),
        report.exported_snapshot.branch_mappings.end(),
        [](const GridLABDBranchMapping& mapping) {
          return mapping.exported && mapping.exported_as_transformer;
        });
    if (has_transformer_export && !options.compare_transformer_branch_flows) {
      report.equivalence_claim +=
          " Transformer terminal voltages are included; transformer branch "
          "flows are excluded from the default pass/fail unless "
          "compare_transformer_branch_flows is enabled.";
    }
  } else if (!report.gridlabd_run_success || !report.hacdcpf_power_flow_converged ||
             !report.numerical_comparison_passed) {
    if (report.unsupported_features.empty() &&
        report.diagnostic_only_reasons.empty()) {
      report.equivalence_claim =
          "Eligible GridLAB-D equivalence scope, but the external solve or "
          "configured numerical comparison did not pass in this run.";
    }
  }
  return report;
}

}  // namespace hacdcpf::io
