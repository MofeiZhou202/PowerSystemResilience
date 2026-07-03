#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hacdcpf/analysis/distribution_power_flow.hpp"
#include "hacdcpf/io/opendss_bridge.hpp"

namespace phase_domain_test {

constexpr double kPi = 3.14159265358979323846;

using hacdcpf::analysis::PhaseDomainBusRow;
using hacdcpf::analysis::PhaseDomainSolverAlgorithm;
using hacdcpf::analysis::RunPFPhaseOptions;
using hacdcpf::analysis::ThreePhaseJPCPhase;

inline std::string trim(std::string text) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

inline std::string uppercase(std::string text) {
  std::transform(
      text.begin(),
      text.end(),
      text.begin(),
      [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return text;
}

inline std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string current;
  bool in_quotes = false;
  for (char ch : line) {
    if (ch == '"') {
      in_quotes = !in_quotes;
      continue;
    }
    if (ch == ',' && !in_quotes) {
      fields.push_back(trim(current));
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  fields.push_back(trim(current));
  return fields;
}

inline int find_header_index(
    const std::vector<std::string>& header,
    const std::string& target) {
  const std::string wanted = uppercase(trim(target));
  for (std::size_t idx = 0; idx < header.size(); ++idx) {
    if (uppercase(trim(header[idx])) == wanted) {
      return static_cast<int>(idx);
    }
  }
  return -1;
}

inline double parse_double_or(
    const std::vector<std::string>& row,
    int index,
    double fallback = 0.0) {
  if (index < 0 || index >= static_cast<int>(row.size()) || row[static_cast<std::size_t>(index)].empty()) {
    return fallback;
  }
  return std::stod(row[static_cast<std::size_t>(index)]);
}

inline int parse_int_or(
    const std::vector<std::string>& row,
    int index,
    int fallback = 0) {
  if (index < 0 || index >= static_cast<int>(row.size()) || row[static_cast<std::size_t>(index)].empty()) {
    return fallback;
  }
  return std::stoi(row[static_cast<std::size_t>(index)]);
}

using TruthMap = std::unordered_map<std::string, std::array<std::optional<double>, 3>>;

inline TruthMap load_truth_csv(const std::filesystem::path& csv_path) {
  std::ifstream input(csv_path);
  if (!input) {
    throw std::runtime_error("Failed to open CSV: " + csv_path.string());
  }

  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("Empty CSV: " + csv_path.string());
  }
  const std::vector<std::string> header = split_csv_line(line);

  TruthMap truth;
  const int node_col = find_header_index(header, "Node");
  const int vpu_col = find_header_index(header, "V_pu");
  const int bus_col = find_header_index(header, "Bus");
  if (node_col >= 0 && vpu_col >= 0) {
    while (std::getline(input, line)) {
      if (trim(line).empty()) continue;
      const std::vector<std::string> row = split_csv_line(line);
      if (node_col >= static_cast<int>(row.size()) || vpu_col >= static_cast<int>(row.size())) {
        continue;
      }
      const std::string node = row[static_cast<std::size_t>(node_col)];
      const std::size_t dot = node.find('.');
      const std::string bus = uppercase(dot == std::string::npos ? node : node.substr(0, dot));
      const int phase =
          (dot == std::string::npos) ? 1 : std::stoi(node.substr(dot + 1));
      if (phase >= 1 && phase <= 3) {
        truth[bus][static_cast<std::size_t>(phase - 1)] =
            std::stod(row[static_cast<std::size_t>(vpu_col)]);
      }
    }
    return truth;
  }

  if (bus_col < 0) {
    throw std::runtime_error("Unsupported CSV format: " + csv_path.string());
  }

  const int node1_col = std::max(find_header_index(header, "Node1"), find_header_index(header, " Node1"));
  const int pu1_col = std::max(find_header_index(header, "pu1"), find_header_index(header, " pu1"));
  const int node2_col = std::max(find_header_index(header, "Node2"), find_header_index(header, " Node2"));
  const int pu2_col = std::max(find_header_index(header, "pu2"), find_header_index(header, " pu2"));
  const int node3_col = std::max(find_header_index(header, "Node3"), find_header_index(header, " Node3"));
  const int pu3_col = std::max(find_header_index(header, "pu3"), find_header_index(header, " pu3"));

  while (std::getline(input, line)) {
    if (trim(line).empty()) continue;
    const std::vector<std::string> row = split_csv_line(line);
    const std::string bus = uppercase(row[static_cast<std::size_t>(bus_col)]);
    const std::array<std::pair<int, int>, 3> cols = {{
        {node1_col, pu1_col},
        {node2_col, pu2_col},
        {node3_col, pu3_col},
    }};
    for (const auto& [node_col_idx, pu_col_idx] : cols) {
      const int node = parse_int_or(row, node_col_idx, 0);
      const double pu = parse_double_or(row, pu_col_idx, 0.0);
      if (node >= 1 && node <= 3 && pu > 0.0) {
        truth[bus][static_cast<std::size_t>(node - 1)] = pu;
      }
    }
  }
  return truth;
}

inline const PhaseDomainBusRow* find_bus_row(
    const ThreePhaseJPCPhase& jpc,
    const std::string& bus_name) {
  const auto it = jpc.bus_name_to_id.find(bus_name);
  if (it == jpc.bus_name_to_id.end()) {
    return nullptr;
  }
  for (const auto& row : jpc.bus_abc) {
    if (row.bus_id == it->second) {
      return &row;
    }
  }
  return nullptr;
}

inline const PhaseDomainBusRow* find_bus_row_case_insensitive(
    const ThreePhaseJPCPhase& jpc,
    const std::string& bus_name) {
  if (const PhaseDomainBusRow* row = find_bus_row(jpc, bus_name)) {
    return row;
  }
  const std::string wanted = uppercase(bus_name);
  for (const auto& [name, id] : jpc.bus_name_to_id) {
    if (uppercase(name) != wanted) continue;
    for (const auto& row : jpc.bus_abc) {
      if (row.bus_id == id) {
        return &row;
      }
    }
  }
  return nullptr;
}

inline bool row_has_phase(const PhaseDomainBusRow& row, int phase) {
  return row.has_phase[static_cast<std::size_t>(phase)];
}

inline double row_vm(const PhaseDomainBusRow& row, int phase) {
  return row.vm_pu[static_cast<std::size_t>(phase)];
}

inline double row_va(const PhaseDomainBusRow& row, int phase) {
  return row.va_deg[static_cast<std::size_t>(phase)];
}

inline std::complex<double> row_voltage(const PhaseDomainBusRow& row, int phase) {
  return std::polar(row_vm(row, phase), row_va(row, phase) * kPi / 180.0);
}

struct ErrorStats {
  double max_error{0.0};
  double avg_error{0.0};
  int count{0};
};

inline ErrorStats compare_to_truth(
    const ThreePhaseJPCPhase& jpc,
    const TruthMap& truth) {
  double max_error = 0.0;
  double sum_error = 0.0;
  int count = 0;

  for (const auto& [bus_name, _] : jpc.bus_name_to_id) {
    const PhaseDomainBusRow* row = find_bus_row(jpc, bus_name);
    if (row == nullptr) continue;
    const auto truth_it = truth.find(uppercase(bus_name));
    if (truth_it == truth.end()) continue;
    for (int phase = 0; phase < 3; ++phase) {
      if (!row_has_phase(*row, phase)) continue;
      const auto& truth_value = truth_it->second[static_cast<std::size_t>(phase)];
      if (!truth_value.has_value()) continue;
      const double error = std::abs(row_vm(*row, phase) - *truth_value);
      max_error = std::max(max_error, error);
      sum_error += error;
      ++count;
    }
  }

  return {
      .max_error = max_error,
      .avg_error = count > 0 ? (sum_error / static_cast<double>(count)) : 0.0,
      .count = count,
  };
}

inline double wrap_angle_diff_deg(double lhs, double rhs) {
  double diff = lhs - rhs;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

struct VoltageComparison {
  double max_vm_error{0.0};
  double max_angle_error_deg{0.0};
  int count{0};
};

struct DetailedVoltageComparison {
  double max_vm_error{0.0};
  double avg_vm_error{0.0};
  double max_angle_error_deg{0.0};
  double avg_angle_error_deg{0.0};
  std::string worst_bus;
  int worst_phase{0};
  double worst_vm_lhs{0.0};
  double worst_vm_rhs{0.0};
  double worst_angle_lhs_deg{0.0};
  double worst_angle_rhs_deg{0.0};
  int count{0};
};

struct VoltageMismatchPoint {
  std::string bus;
  int phase{0};
  double vm_lhs{0.0};
  double vm_rhs{0.0};
  double va_lhs_deg{0.0};
  double va_rhs_deg{0.0};
  double vm_error{0.0};
  double angle_error_deg{0.0};
};

inline DetailedVoltageComparison compare_jpc_results_detailed(
    const ThreePhaseJPCPhase& lhs,
    const ThreePhaseJPCPhase& rhs);

inline std::vector<VoltageMismatchPoint> compare_jpc_points(
    const ThreePhaseJPCPhase& lhs,
    const ThreePhaseJPCPhase& rhs);

inline VoltageComparison compare_jpc_results(
    const ThreePhaseJPCPhase& lhs,
    const ThreePhaseJPCPhase& rhs) {
  const DetailedVoltageComparison detailed = compare_jpc_results_detailed(lhs, rhs);

  return {
      .max_vm_error = detailed.max_vm_error,
      .max_angle_error_deg = detailed.max_angle_error_deg,
      .count = detailed.count,
  };
}

inline DetailedVoltageComparison compare_jpc_results_detailed(
    const ThreePhaseJPCPhase& lhs,
    const ThreePhaseJPCPhase& rhs) {
  DetailedVoltageComparison stats;
  double sum_vm_error = 0.0;
  double sum_angle_error_deg = 0.0;
  for (const auto& [bus_name, _] : lhs.bus_name_to_id) {
    const PhaseDomainBusRow* lhs_row = find_bus_row(lhs, bus_name);
    const PhaseDomainBusRow* rhs_row = find_bus_row_case_insensitive(rhs, bus_name);
    if (lhs_row == nullptr || rhs_row == nullptr) continue;
    for (int phase = 0; phase < 3; ++phase) {
      if (!row_has_phase(*lhs_row, phase) || !row_has_phase(*rhs_row, phase)) continue;
      const double vm_error =
          std::abs(row_vm(*lhs_row, phase) - row_vm(*rhs_row, phase));
      const double angle_error_deg =
          std::abs(wrap_angle_diff_deg(row_va(*lhs_row, phase), row_va(*rhs_row, phase)));
      if (vm_error > stats.max_vm_error ||
          angle_error_deg > stats.max_angle_error_deg) {
        stats.worst_bus = bus_name;
        stats.worst_phase = phase + 1;
        stats.worst_vm_lhs = row_vm(*lhs_row, phase);
        stats.worst_vm_rhs = row_vm(*rhs_row, phase);
        stats.worst_angle_lhs_deg = row_va(*lhs_row, phase);
        stats.worst_angle_rhs_deg = row_va(*rhs_row, phase);
      }
      stats.max_vm_error = std::max(stats.max_vm_error, vm_error);
      stats.max_angle_error_deg = std::max(stats.max_angle_error_deg, angle_error_deg);
      sum_vm_error += vm_error;
      sum_angle_error_deg += angle_error_deg;
      ++stats.count;
    }
  }
  if (stats.count > 0) {
    stats.avg_vm_error = sum_vm_error / static_cast<double>(stats.count);
    stats.avg_angle_error_deg = sum_angle_error_deg / static_cast<double>(stats.count);
  }
  return stats;
}

inline std::vector<VoltageMismatchPoint> compare_jpc_points(
    const ThreePhaseJPCPhase& lhs,
    const ThreePhaseJPCPhase& rhs) {
  std::vector<VoltageMismatchPoint> points;
  for (const auto& [bus_name, _] : lhs.bus_name_to_id) {
    const PhaseDomainBusRow* lhs_row = find_bus_row(lhs, bus_name);
    const PhaseDomainBusRow* rhs_row = find_bus_row_case_insensitive(rhs, bus_name);
    if (lhs_row == nullptr || rhs_row == nullptr) continue;
    for (int phase = 0; phase < 3; ++phase) {
      if (!row_has_phase(*lhs_row, phase) || !row_has_phase(*rhs_row, phase)) continue;
      const double vm_error =
          std::abs(row_vm(*lhs_row, phase) - row_vm(*rhs_row, phase));
      const double angle_error_deg =
          std::abs(wrap_angle_diff_deg(row_va(*lhs_row, phase), row_va(*rhs_row, phase)));
      points.push_back(VoltageMismatchPoint{
          .bus = bus_name,
          .phase = phase + 1,
          .vm_lhs = row_vm(*lhs_row, phase),
          .vm_rhs = row_vm(*rhs_row, phase),
          .va_lhs_deg = row_va(*lhs_row, phase),
          .va_rhs_deg = row_va(*rhs_row, phase),
          .vm_error = vm_error,
          .angle_error_deg = angle_error_deg,
      });
    }
  }
  std::sort(
      points.begin(),
      points.end(),
      [](const VoltageMismatchPoint& lhs, const VoltageMismatchPoint& rhs) {
        if (lhs.vm_error != rhs.vm_error) return lhs.vm_error > rhs.vm_error;
        return lhs.angle_error_deg > rhs.angle_error_deg;
      });
  return points;
}

inline double sequence_angle_diff_deg(
    const ThreePhaseJPCPhase& jpc,
    const std::string& lhs_bus,
    const std::string& rhs_bus) {
  const PhaseDomainBusRow* lhs = find_bus_row_case_insensitive(jpc, lhs_bus);
  const PhaseDomainBusRow* rhs = find_bus_row_case_insensitive(jpc, rhs_bus);
  if (lhs == nullptr || rhs == nullptr) {
    throw std::runtime_error("Missing bus for sequence angle diff");
  }
  const std::complex<double> a = std::polar(1.0, 2.0 * kPi / 3.0);
  const std::complex<double> a2 = std::polar(1.0, 4.0 * kPi / 3.0);
  const auto positive_sequence = [&](const PhaseDomainBusRow& row) {
    return (row_voltage(row, 0) + a * row_voltage(row, 1) + a2 * row_voltage(row, 2)) / 3.0;
  };
  const double lhs_angle = std::arg(positive_sequence(*lhs)) * 180.0 / kPi;
  const double rhs_angle = std::arg(positive_sequence(*rhs)) * 180.0 / kPi;
  return wrap_angle_diff_deg(lhs_angle, rhs_angle);
}

inline void require_or_throw(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

inline std::filesystem::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return std::filesystem::path(HACDCPF_PROJECT_ROOT);
#else
  return std::filesystem::current_path();
#endif
}

inline std::filesystem::path phase_domain_data_dir() {
  return project_root() / "tests" / "data" / "phase_domain_julia";
}

inline ThreePhaseJPCPhase run_from_dss(
    const std::filesystem::path& dss_path,
    PhaseDomainSolverAlgorithm algorithm,
    int max_iter = 200,
    double tol = 1e-6,
    bool verbose = false,
    double vmin_pu = 0.95) {
  const auto sys = hacdcpf::analysis::load_three_phase_system_from_opendss(dss_path);
  RunPFPhaseOptions options;
  options.algorithm = algorithm;
  options.max_iter = max_iter;
  options.tol = tol;
  options.verbose = verbose;
  options.vmin_pu = vmin_pu;
  if (algorithm == PhaseDomainSolverAlgorithm::OpenDSS) {
    options.dss_file_path = dss_path;
  }
  return hacdcpf::analysis::runpf_phase(sys, options);
}

inline ThreePhaseJPCPhase run_snapshot_from_dss(
    const std::filesystem::path& dss_path,
    bool include_shunts = true) {
  const auto sys = hacdcpf::analysis::load_three_phase_system_from_opendss(dss_path);
  ThreePhaseJPCPhase jpc = hacdcpf::analysis::case2jpc_phase(sys, {});
  hacdcpf::analysis::makeYbus_phase(jpc, sys, include_shunts);

  const hacdcpf::io::OpenDSSSnapshotResult snapshot =
      hacdcpf::io::solve_opendss_snapshot(dss_path);

  std::unordered_map<std::string, std::array<std::optional<std::pair<double, double>>, 3>>
      voltage_map;
  for (const auto& node_voltage : snapshot.node_voltages) {
    if (node_voltage.node < 1 || node_voltage.node > 3) continue;
    voltage_map[uppercase(node_voltage.bus_name)]
               [static_cast<std::size_t>(node_voltage.node - 1)] =
                   std::make_pair(node_voltage.vm_pu, node_voltage.va_deg);
  }

  jpc.success = snapshot.converged;
  jpc.iterations = snapshot.control_oracle.control_iterations;
  jpc.residual = 0.0;
  for (std::size_t row_idx = 0; row_idx < jpc.bus_abc.size(); ++row_idx) {
    PhaseDomainBusRow& row = jpc.bus_abc[row_idx];
    const std::array<bool, 3> original_has_phase = row.has_phase;
    row.has_phase = {false, false, false};
    row.vm_pu = {0.0, 0.0, 0.0};
    row.va_deg = {0.0, 0.0, 0.0};

    const auto bus_name_it = jpc.bus_id_to_name.find(row.bus_id);
    if (bus_name_it == jpc.bus_id_to_name.end()) continue;
    const auto voltages_it = voltage_map.find(uppercase(bus_name_it->second));
    if (voltages_it == voltage_map.end()) continue;

    for (int phase = 0; phase < 3; ++phase) {
      if (!original_has_phase[static_cast<std::size_t>(phase)]) continue;
      const auto& maybe_voltage =
          voltages_it->second[static_cast<std::size_t>(phase)];
      if (!maybe_voltage.has_value()) continue;
      const auto& [vm_pu, va_deg] = *maybe_voltage;
      row.vm_pu[static_cast<std::size_t>(phase)] = vm_pu;
      row.va_deg[static_cast<std::size_t>(phase)] = va_deg;
      row.has_phase[static_cast<std::size_t>(phase)] = true;
      jpc.v_abc[row_idx * 3 + static_cast<std::size_t>(phase)] =
          std::polar(vm_pu, va_deg * kPi / 180.0);
    }
  }

  return jpc;
}

inline std::string format_percent(double value_pu) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(4);
  out << value_pu * 100.0 << "%";
  return out.str();
}

}  // namespace phase_domain_test
