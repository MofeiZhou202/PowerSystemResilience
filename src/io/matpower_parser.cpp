#include "hacdcpf/io/matpower_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hacdcpf/detail/string_utils.hpp"

namespace hacdcpf::io {

namespace {

std::string strip_comment(const std::string& s) {
  const size_t p = s.find('%');
  if (p == std::string::npos) {
    return s;
  }
  return s.substr(0, p);
}

std::string read_text_file(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Failed to open MATPOWER file: " + path);
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

double parse_scalar_assignment(const std::string& text,
                               const std::string& key,
                               double default_value) {
  const size_t p = text.find(key);
  if (p == std::string::npos) {
    return default_value;
  }
  const size_t eq = text.find('=', p);
  if (eq == std::string::npos) {
    return default_value;
  }
  const size_t semi = text.find(';', eq);
  if (semi == std::string::npos) {
    return default_value;
  }
  const std::string token = trim(text.substr(eq + 1, semi - eq - 1));
  if (token.empty()) {
    return default_value;
  }
  try {
    return std::stod(token);
  } catch (const std::exception&) {
    return default_value;
  }
}

std::vector<double> parse_number_row(const std::string& row_text) {
  std::string s = row_text;
  for (char& c : s) {
    if (c == ',' || c == '\t') {
      c = ' ';
    }
  }

  auto parse_token = [](const std::string& token, double& out) -> bool {
    if (token.empty()) {
      return false;
    }

    std::string lower = token;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    if (lower == "inf" || lower == "+inf" || lower == "infinity" || lower == "+infinity") {
      out = std::numeric_limits<double>::infinity();
      return true;
    }
    if (lower == "-inf" || lower == "-infinity") {
      out = -std::numeric_limits<double>::infinity();
      return true;
    }
    if (lower == "nan" || lower == "+nan" || lower == "-nan") {
      out = std::numeric_limits<double>::quiet_NaN();
      return true;
    }

    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
      return false;
    }
    out = value;
    return true;
  };

  std::stringstream ss(s);
  std::vector<double> row;
  std::string token;
  while (ss >> token) {
    double value = 0.0;
    if (!parse_token(token, value)) {
      throw std::runtime_error("MATPOWER parser: failed numeric token: '" + token + "'");
    }
    row.push_back(value);
  }
  return row;
}

std::vector<std::vector<double>> parse_matrix_block(const std::string& text,
                                                    const std::string& key) {
  const size_t p = text.find(key);
  if (p == std::string::npos) {
    return {};
  }
  const size_t eq = text.find('=', p);
  if (eq == std::string::npos) {
    return {};
  }
  const size_t lb = text.find('[', eq);
  if (lb == std::string::npos) {
    return {};
  }
  const size_t rb = text.find("];", lb);
  if (rb == std::string::npos) {
    return {};
  }

  const std::string body = text.substr(lb + 1, rb - lb - 1);
  std::stringstream ls(body);
  std::string line;
  std::string acc;
  std::vector<std::vector<double>> rows;

  while (std::getline(ls, line)) {
    std::string clean = trim(strip_comment(line));
    if (clean.empty()) {
      continue;
    }
    if (!acc.empty()) {
      acc.push_back(' ');
    }
    acc += clean;

    size_t semi = std::string::npos;
    while ((semi = acc.find(';')) != std::string::npos) {
      const std::string row_text = trim(acc.substr(0, semi));
      if (!row_text.empty()) {
        auto row = parse_number_row(row_text);
        if (!row.empty()) {
          rows.push_back(std::move(row));
        }
      }
      acc = trim(acc.substr(semi + 1));
    }
  }

  return rows;
}

BusType matpower_bus_type_to_internal(int t) {
  if (t == 3) {
    return BusType::SLACK;
  }
  if (t == 2) {
    return BusType::PV;
  }
  if (t == 1) {
    return BusType::PQ;
  }
  return BusType::ISOLATED;
}

bool has_load_kw_conversion(const std::string& text) {
  return text.find("mpc.bus(:, [PD, QD])") != std::string::npos &&
         text.find("/ 1e3") != std::string::npos;
}

bool has_branch_ohm_conversion(const std::string& text) {
  return text.find("mpc.branch(:, [BR_R BR_X])") != std::string::npos;
}

void parse_gencost_row(const std::vector<double>& row, double& c2, double& c1, double& c0) {
  c2 = 0.0;
  c1 = 1.0;  // keep objective well-posed when gencost is absent/invalid
  c0 = 0.0;
  if (row.size() < 4) {
    return;
  }

  const int model = static_cast<int>(std::lround(row[0]));
  const int n = std::max(0, static_cast<int>(std::lround(row[3])));
  if (n <= 0 || row.size() < static_cast<size_t>(4 + n)) {
    return;
  }

  if (model == 2) {
    // Polynomial model, coefficients in descending order:
    // f(P) = c(n-1) P^(n-1) + ... + c1 P + c0
    const size_t start = 4;
    if (n >= 3) {
      c2 = row[start + static_cast<size_t>(n - 3)];
      c1 = row[start + static_cast<size_t>(n - 2)];
      c0 = row[start + static_cast<size_t>(n - 1)];
    } else if (n == 2) {
      c2 = 0.0;
      c1 = row[start];
      c0 = row[start + 1];
    } else if (n == 1) {
      c2 = 0.0;
      c1 = 0.0;
      c0 = row[start];
    }
    return;
  }

  if (model == 1 && n >= 2 && row.size() >= static_cast<size_t>(4 + 2 * n)) {
    // Piecewise-linear model, approximate with end-to-end affine fit.
    const size_t start = 4;
    const double x1 = row[start];
    const double y1 = row[start + 1];
    const size_t last = start + static_cast<size_t>(2 * (n - 1));
    const double x2 = row[last];
    const double y2 = row[last + 1];
    const double dx = x2 - x1;
    const double slope = (std::abs(dx) > 1e-12) ? ((y2 - y1) / dx) : 0.0;
    c2 = 0.0;
    c1 = slope;
    c0 = y1 - slope * x1;
  }
}

}  // namespace

HybridPowerSystem parse_matpower(const std::string& filepath) {
  const std::string text = read_text_file(filepath);

  const double base_mva = parse_scalar_assignment(text, "mpc.baseMVA", 100.0);
  auto bus_rows = parse_matrix_block(text, "mpc.bus");
  auto gen_rows = parse_matrix_block(text, "mpc.gen");
  auto branch_rows = parse_matrix_block(text, "mpc.branch");
  auto gencost_rows = parse_matrix_block(text, "mpc.gencost");

  if (bus_rows.empty()) {
    throw std::runtime_error("MATPOWER parser: empty/invalid bus matrix: " + filepath);
  }

  if (has_load_kw_conversion(text)) {
    for (auto& row : bus_rows) {
      if (row.size() >= 4) {
        row[2] /= 1e3;
        row[3] /= 1e3;
      }
    }
  }

  if (has_branch_ohm_conversion(text)) {
    double base_kv = 0.0;
    if (!bus_rows.empty() && bus_rows[0].size() >= 10) {
      base_kv = bus_rows[0][9];
    }
    if (base_kv > 0.0 && base_mva > 0.0) {
      const double zbase = (base_kv * base_kv) / base_mva;
      for (auto& row : branch_rows) {
        if (row.size() >= 4) {
          row[2] /= zbase;
          row[3] /= zbase;
        }
      }
    }
  }

  HybridPowerSystem sys;
  sys.base_mva = base_mva;
  sys.name = std::filesystem::path(filepath).stem().string();
  sys.ac.base_mva = base_mva;
  sys.ac.name = sys.name + " AC";

  std::unordered_map<int, int> bus_id_to_idx;
  bus_id_to_idx.reserve(bus_rows.size());

  sys.ac.buses.reserve(bus_rows.size());
  for (size_t i = 0; i < bus_rows.size(); ++i) {
    const auto& r = bus_rows[i];
    if (r.size() < 13) {
      continue;
    }

    const int bus_id = static_cast<int>(std::lround(r[0]));
    const int idx = static_cast<int>(sys.ac.buses.size()) + 1;
    bus_id_to_idx.emplace(bus_id, idx);

    ACBus b;
    b.index = idx;
    b.bus_type = matpower_bus_type_to_internal(static_cast<int>(std::lround(r[1])));
    b.pd_mw = r[2];
    b.qd_mvar = r[3];
    b.gs_mw = r[4];
    b.bs_mvar = r[5];
    b.area = static_cast<int>(std::lround(r[6]));
    b.vm_pu = r[7];
    b.va_deg = r[8];
    b.base_kv = r[9];
    b.zone = static_cast<int>(std::lround(r[10]));
    b.vmax_pu = r[11];
    b.vmin_pu = r[12];
    b.in_service = true;
    b.name = "Bus" + std::to_string(bus_id);
    sys.ac.buses.push_back(std::move(b));
  }

  sys.ac.generators.reserve(gen_rows.size());
  for (size_t i = 0; i < gen_rows.size(); ++i) {
    const auto& r = gen_rows[i];
    if (r.size() < 10) {
      continue;
    }
    const int bus_id = static_cast<int>(std::lround(r[0]));
    const auto it = bus_id_to_idx.find(bus_id);
    if (it == bus_id_to_idx.end()) {
      continue;
    }

    Generator g;
    g.index = static_cast<int>(sys.ac.generators.size()) + 1;
    g.bus = it->second;
    g.pg_mw = r[1];
    g.qg_mvar = r[2];
    g.qmax_mvar = r[3];
    g.qmin_mvar = r[4];
    g.vg_pu = r[5];
    // col 6 = mbase
    g.mbase_mva = (r.size() >= 7) ? r[6] : 0.0;
    g.in_service = (r.size() >= 8) ? (static_cast<int>(std::lround(r[7])) == 1) : true;
    g.pmax_mw = r[8];
    g.pmin_mw = r[9];

    // MATPOWER extended gen columns (if present):
    // col 16 = ramp_agc, col 17 = ramp_10, col 18 = ramp_30
    if (r.size() >= 18) {
      // Use ramp_10 (10-min ramp) as ramp rate in MW/min
      g.ramp_up_mw_min = r[17] / 10.0;
      g.ramp_dn_mw_min = r[17] / 10.0;
    }

    g.cost_c2 = 0.0;
    g.cost_c1 = 1.0;
    g.cost_c0 = 0.0;
    if (i < gencost_rows.size()) {
      parse_gencost_row(gencost_rows[i], g.cost_c2, g.cost_c1, g.cost_c0);
      // Extract startup/shutdown cost from gencost row cols 1,2
      if (gencost_rows[i].size() >= 3) {
        g.startup_cost = gencost_rows[i][1];
        g.shutdown_cost = gencost_rows[i][2];
      }
    }

    BusType bt = sys.ac.buses[static_cast<size_t>(g.bus - 1)].bus_type;
    g.is_slack = (bt == BusType::SLACK);
    g.name = "Gen" + std::to_string(g.index);
    sys.ac.generators.push_back(std::move(g));
  }

  // MATPOWER uses the generator VG column as the regulated voltage setpoint for
  // in-service PV and reference buses. The bus VM column is an initial value.
  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service || gen.vg_pu <= 0.0) {
      continue;
    }
    const int idx = gen.bus - 1;
    if (idx < 0 || idx >= static_cast<int>(sys.ac.buses.size())) {
      continue;
    }
    auto& bus = sys.ac.buses[static_cast<size_t>(idx)];
    if (bus.bus_type == BusType::PV || bus.bus_type == BusType::SLACK) {
      bus.vm_pu = gen.vg_pu;
    }
  }

  sys.ac.branches.reserve(branch_rows.size());
  for (size_t i = 0; i < branch_rows.size(); ++i) {
    const auto& r = branch_rows[i];
    if (r.size() < 10) {
      continue;
    }
    const int fbus_id = static_cast<int>(std::lround(r[0]));
    const int tbus_id = static_cast<int>(std::lround(r[1]));
    const auto itf = bus_id_to_idx.find(fbus_id);
    const auto itt = bus_id_to_idx.find(tbus_id);
    if (itf == bus_id_to_idx.end() || itt == bus_id_to_idx.end()) {
      continue;
    }

    ACBranch br;
    br.index = static_cast<int>(sys.ac.branches.size()) + 1;
    br.from_bus = itf->second;
    br.to_bus = itt->second;
    br.r_pu = r[2];
    br.x_pu = r[3];
    br.b_pu = r[4];
    br.rate_a_mva = (r.size() >= 6) ? r[5] : 0.0;
    br.rate_b_mva = (r.size() >= 7) ? r[6] : 0.0;
    br.rate_c_mva = (r.size() >= 8) ? r[7] : 0.0;
    const double ratio = (r.size() >= 9) ? r[8] : 0.0;
    br.tap = (std::abs(ratio) < 1e-12) ? 1.0 : ratio;
    br.shift_deg = (r.size() >= 10) ? r[9] : 0.0;
    br.in_service = (r.size() >= 11) ? (static_cast<int>(std::lround(r[10])) == 1) : true;
    br.name = "Line" + std::to_string(br.index);
    sys.ac.branches.push_back(std::move(br));
  }

  // ── Detect transformers among branches ──────────────────────────────
  // MATPOWER encodes transformers as branches with tap ratio ≠ 1.0 or
  // phase shift ≠ 0.  Create Transformer2W metadata entries so that
  // reactive power optimization (tap control) and the GUI can discover
  // them.  The original branches are kept in the branch list so that
  // power-flow operates on the exact pi-model data without round-trip
  // impedance loss.  source_branch_idx links each Transformer2W back
  // to its original branch so project_to_canonical_models can update
  // rather than duplicate.
  {
    int tr_idx = 1;
    for (const auto& br : sys.ac.branches) {
      const bool is_transformer =
          (std::abs(br.tap - 1.0) > 1e-8) || (std::abs(br.shift_deg) > 1e-8);
      if (!is_transformer) {
        continue;
      }
      // Derive Transformer2W parameters from the pi-model branch data.
      Transformer2W tr;
      tr.index = tr_idx++;
      tr.hv_bus = br.from_bus;
      tr.lv_bus = br.to_bus;
      tr.in_service = br.in_service;
      tr.sn_mva = (br.rate_a_mva > 0.0) ? br.rate_a_mva : base_mva;
      // Recover rated voltages from the bus base_kv.
      const auto& buses = sys.ac.buses;
      const int fi = br.from_bus - 1;
      const int ti = br.to_bus - 1;
      tr.vn_hv_kv = (fi >= 0 && fi < static_cast<int>(buses.size())) ? buses[static_cast<size_t>(fi)].base_kv : 110.0;
      tr.vn_lv_kv = (ti >= 0 && ti < static_cast<int>(buses.size())) ? buses[static_cast<size_t>(ti)].base_kv : 10.0;
      // Convert r_pu, x_pu back to vk_percent, vkr_percent
      // vkr% = r_pu * sn / base_mva * 100
      // vk%  = z_pu * sn / base_mva * 100   where z_pu = sqrt(r² + x²)
      const double sn_over_base = tr.sn_mva / std::max(1e-9, base_mva);
      tr.vkr_percent = br.r_pu * sn_over_base * 100.0;
      const double z_pu = std::sqrt(br.r_pu * br.r_pu + br.x_pu * br.x_pu);
      tr.vk_percent = z_pu * sn_over_base * 100.0;
      tr.pk_kw = br.r_pu * tr.sn_mva * tr.sn_mva / std::max(1e-9, base_mva) * 1e3;
      // Tap position: encode the off-nominal tap ratio as a step offset.
      // Use tap_step_percent = 0.01% so that tap_pos encodes the
      // deviation from nominal with 4 decimal places of precision.
      tr.tap_step_percent = 0.01;
      tr.tap_neutral = 0;
      tr.tap_min = -2000;
      tr.tap_max = 2000;
      tr.tap_pos = static_cast<int>(std::round((br.tap - 1.0) * 10000.0));
      tr.tap_side = 0;  // HV side
      tr.shift_deg = br.shift_deg;
      tr.name = "Trafo" + std::to_string(tr.index);
      tr.source_branch_idx = br.index;  // link back to original branch
      sys.ac.transformers_2w.push_back(std::move(tr));
    }
    // Branches are NOT removed — solver uses exact pi-model data.
  }

  // ── Create Shunt objects from bus-level Gs / Bs ─────────────────────
  // MATPOWER stores shunt admittance on bus columns 4–5.  Move non-zero
  // values into explicit Shunt entries so that reactive power optimization
  // (switchable shunt control) and the GUI can discover them.
  {
    int sh_idx = 1;
    for (auto& bus : sys.ac.buses) {
      if (std::abs(bus.gs_mw) < 1e-15 && std::abs(bus.bs_mvar) < 1e-15) continue;
      Shunt sh;
      sh.index = sh_idx++;
      sh.bus = bus.index;
      sh.in_service = true;
      sh.gs_mw = bus.gs_mw;
      sh.bs_mvar = bus.bs_mvar;
      sh.name = "Shunt_Bus" + std::to_string(bus.index);
      sys.ac.shunts.push_back(std::move(sh));
      bus.gs_mw = 0.0;
      bus.bs_mvar = 0.0;
    }
  }

  return sys;
}

}  // namespace hacdcpf::io
