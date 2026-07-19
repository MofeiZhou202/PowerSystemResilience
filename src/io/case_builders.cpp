#include "hacdcpf/io/case_builders.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/io/matpower_parser.hpp"

namespace hacdcpf::io {

namespace {

ACBus make_ac_bus(int index,
                  BusType type,
                  double pd_mw,
                  double qd_mvar,
                  double vm_pu,
                  double va_deg,
                  int area = 1) {
  ACBus b;
  b.index = index;
  b.bus_type = type;
  b.pd_mw = pd_mw;
  b.qd_mvar = qd_mvar;
  b.vm_pu = vm_pu;
  b.va_deg = va_deg;
  b.area = area;
  b.name = "Bus" + std::to_string(index);
  return b;
}

ACBranch make_ac_branch(int index,
                        int from_bus,
                        int to_bus,
                        double r_pu,
                        double x_pu,
                        double b_pu,
                        double tap) {
  ACBranch br;
  br.index = index;
  br.from_bus = from_bus;
  br.to_bus = to_bus;
  br.r_pu = r_pu;
  br.x_pu = x_pu;
  br.b_pu = b_pu;
  br.tap = tap;
  br.name = "Line" + std::to_string(index);
  return br;
}

Generator make_generator(int index,
                         int bus,
                         bool is_slack,
                         double pg_mw,
                         double qg_mvar,
                         double vg_pu,
                         double pmax_mw,
                         double pmin_mw,
                         double qmax_mvar,
                         double qmin_mvar) {
  Generator g;
  g.index = index;
  g.bus = bus;
  g.is_slack = is_slack;
  g.pg_mw = pg_mw;
  g.qg_mvar = qg_mvar;
  g.vg_pu = vg_pu;
  g.pmax_mw = pmax_mw;
  g.pmin_mw = pmin_mw;
  g.qmax_mvar = qmax_mvar;
  g.qmin_mvar = qmin_mvar;
  g.name = "Gen" + std::to_string(index);
  return g;
}

// Attach a minimal grid-forming synchronous-machine control stack (classical
// machine + TGOV1 governor + SEXS exciter) so a genset can hold voltage and
// frequency when its microgrid islands.  Read only by the transient builder;
// PF/OPF consume the steady-state generator fields and ignore dynamic_model.
void attach_grid_forming_genset_dynamics(Generator& g) {
  g.dynamic_model.model_name = "ClassicalMachine";
  g.dynamic_model.standard = "IEEE";
  auto add_block = [&](const std::string& type, const std::string& model,
                       std::map<std::string, double> params) {
    hacdcpf::DynamicModelComponentProfile c;
    c.type = type;
    c.model = model;
    c.standard = "IEEE";
    c.parameters = std::move(params);
    g.dynamic_model.components.push_back(std::move(c));
  };
  add_block("governor", "TGOV1", {{"R", 0.05}, {"T1", 0.5}, {"T3", 0.5}});
  add_block("exciter", "SEXS", {{"Ka", 50.0}, {"Ta", 0.1}});
}

void apply_ieee24_reference_coordinates(HybridPowerSystem& sys) {
  static const std::unordered_map<int, std::pair<double, double>> kCoords = {
      {1, {4.0, 0.0}},   {2, {10.0, 0.0}},  {3, {4.0, 6.0}},   {4, {6.0, 3.0}},
      {5, {10.0, 3.0}},  {6, {16.0, 6.0}},  {7, {16.0, 0.0}},  {8, {16.0, 3.0}},
      {9, {9.0, 6.0}},   {10, {13.0, 6.0}}, {11, {9.0, 9.0}},  {12, {13.0, 9.0}},
      {13, {16.0, 12.0}}, {14, {8.0, 12.0}}, {15, {4.0, 12.0}}, {16, {4.0, 15.0}},
      {17, {1.0, 18.0}}, {18, {4.0, 20.0}}, {19, {9.0, 15.0}}, {20, {13.0, 15.0}},
      {21, {9.0, 20.0}}, {22, {13.0, 20.0}}, {23, {15.0, 17.0}}, {24, {4.0, 9.0}},
  };
  for (auto& b : sys.ac.buses) {
    const auto it = kCoords.find(b.index);
    if (it == kCoords.end()) continue;
    b.longitude = it->second.first;
    b.latitude = it->second.second;
  }
}

int find_slack_bus(const HybridPowerSystem& sys) {
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (sys.ac.buses[i].bus_type == BusType::SLACK) {
      return static_cast<int>(i) + 1;
    }
  }
  return sys.ac.buses.empty() ? 0 : 1;
}

std::filesystem::path data_file_path(const std::string& name) {
  // Search a list of candidate locations and return the first that exists.
  // The MATPOWER `.m` library ships under <root>/external_data/matpower; the
  // legacy sibling "HybridACDCPowerFlow/data" layout is kept as a fallback.
  namespace fs = std::filesystem;
  std::vector<fs::path> candidates;
#ifdef HACDCPF_PROJECT_ROOT
  const fs::path root(HACDCPF_PROJECT_ROOT);
  candidates.push_back(root / "external_data" / "matpower" / name);
  candidates.push_back(root / "data" / name);
  candidates.push_back(root.parent_path() / "HybridACDCPowerFlow" / "data" / name);
#endif
  const fs::path cwd = fs::current_path();
  candidates.push_back(cwd / "external_data" / "matpower" / name);
  candidates.push_back(cwd / ".." / "external_data" / "matpower" / name);
  candidates.push_back(cwd / "data" / name);
  candidates.push_back(cwd / ".." / "data" / name);
  for (const auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c, ec)) return c;
  }
  // None found: fall back to the legacy path so the error message is familiar.
#ifdef HACDCPF_PROJECT_ROOT
  return root.parent_path() / "HybridACDCPowerFlow" / "data" / name;
#else
  return fs::path("../HybridACDCPowerFlow/data") / name;
#endif
}

void attach_two_terminal_dc(HybridPowerSystem& sys,
                            int ac_bus_1,
                            int ac_bus_2,
                            double vdc_set_pu = 1.02,
                            double dc_r_pu = 0.02) {
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n < 2) {
    return;
  }

  ac_bus_1 = std::clamp(ac_bus_1, 1, n);
  ac_bus_2 = std::clamp(ac_bus_2, 1, n);
  if (ac_bus_1 == ac_bus_2) {
    ac_bus_2 = (ac_bus_1 == 1) ? 2 : 1;
  }

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " DC";
  sys.dc.buses = {
      DCBus{.index = 1,
            .bus_type = DCBusType::DC_P,
            .vm_pu = 1.0,
            .pd_mw = 0.0,
            .in_service = true,
            .name = "DCBus1"},
      DCBus{.index = 2,
            .bus_type = DCBusType::DC_V,
            .vm_pu = 1.0,
            .pd_mw = 0.0,
            .in_service = true,
            .name = "DCBus2"},
  };

  sys.dc.branches = {
      DCBranch{.index = 1,
               .from_bus = 1,
               .to_bus = 2,
               .r_pu = dc_r_pu,
               .in_service = true,
               .name = "DCLine1"},
  };

  sys.vsc_converters = {
      VSCConverter{.index = 1,
                   .bus_ac = ac_bus_1,
                   .bus_dc = 1,
                   .in_service = true,
                   .control_mode = ConverterMode::PQ_MODE,
                   .p_set_mw = 0.0,
                   .q_set_mvar = 0.0,
                   .v_dc_set_pu = 1.0,
                   .v_ac_set_pu = sys.ac.buses[static_cast<size_t>(ac_bus_1 - 1)].vm_pu,
                   .eta = 0.99,
                   .loss_percent = 0.0,
                   .loss_mw = 0.0,
                   .k_vdc = 0.1,
                   .pmax_mw = sys.base_mva,
                   .pmin_mw = -sys.base_mva,
                   .qmax_mvar = sys.base_mva,
                   .qmin_mvar = -sys.base_mva,
                   .name = "VSC1"},
      VSCConverter{.index = 2,
                   .bus_ac = ac_bus_2,
                   .bus_dc = 2,
                   .in_service = true,
                   .control_mode = ConverterMode::VDC_Q,
                   .p_set_mw = 0.0,
                   .q_set_mvar = 0.0,
                   .v_dc_set_pu = vdc_set_pu,
                   .v_ac_set_pu = sys.ac.buses[static_cast<size_t>(ac_bus_2 - 1)].vm_pu,
                   .eta = 0.99,
                   .loss_percent = 0.0,
                   .loss_mw = 0.0,
                   .k_vdc = 0.1,
                   .pmax_mw = sys.base_mva,
                   .pmin_mw = -sys.base_mva,
                   .qmax_mvar = sys.base_mva,
                   .qmin_mvar = -sys.base_mva,
                   .name = "VSC2"},
  };
}

[[maybe_unused]] void attach_multiterminal_dc(HybridPowerSystem& sys,
                                              const std::vector<int>& ac_bus_candidates) {
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n < 4) {
    attach_two_terminal_dc(sys, find_slack_bus(sys), n);
    return;
  }

  std::vector<int> ac = ac_bus_candidates;
  for (int& b : ac) {
    b = std::clamp(b, 1, n);
  }
  if (ac.empty()) {
    ac = {find_slack_bus(sys), n / 3, 2 * n / 3, n};
  }

  while (ac.size() < 4) {
    ac.push_back(std::max(1, static_cast<int>(ac.size()) * n / 4));
  }

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " MTDC";
  sys.dc.buses.clear();
  for (int i = 0; i < 4; ++i) {
    DCBus b;
    b.index = i + 1;
    b.bus_type = (i == 0) ? DCBusType::DC_V : DCBusType::DC_P;
    b.vm_pu = 1.0;
    b.pd_mw = 0.0;
    b.in_service = true;
    b.name = "DCBus" + std::to_string(i + 1);
    sys.dc.buses.push_back(b);
  }

  sys.dc.branches = {
      DCBranch{.index = 1,
               .from_bus = 1,
               .to_bus = 2,
               .r_pu = 0.01,
               .in_service = true,
               .name = "DCLine1"},
      DCBranch{.index = 2,
               .from_bus = 2,
               .to_bus = 3,
               .r_pu = 0.01,
               .in_service = true,
               .name = "DCLine2"},
      DCBranch{.index = 3,
               .from_bus = 3,
               .to_bus = 4,
               .r_pu = 0.01,
               .in_service = true,
               .name = "DCLine3"},
  };

  sys.vsc_converters.clear();
  for (int i = 0; i < 4; ++i) {
    VSCConverter c;
    c.index = i + 1;
    c.bus_ac = ac[static_cast<size_t>(i)];
    c.bus_dc = i + 1;
    c.in_service = true;
    c.control_mode = (i % 2 == 0) ? ConverterMode::PQ_MODE : ConverterMode::VDC_Q;
    c.p_set_mw = 0.0;
    c.q_set_mvar = 0.0;
    c.v_dc_set_pu = (i % 2 == 0) ? 1.0 : 1.02;
    c.v_ac_set_pu = sys.ac.buses[static_cast<size_t>(c.bus_ac - 1)].vm_pu;
    c.eta = 0.99;
    c.loss_percent = 0.0;
    c.loss_mw = 0.0;
    c.k_vdc = 0.1;
    c.pmax_mw = 2.0 * sys.base_mva;
    c.pmin_mw = -2.0 * sys.base_mva;
    c.qmax_mvar = 2.0 * sys.base_mva;
    c.qmin_mvar = -2.0 * sys.base_mva;
    c.name = "VSC" + std::to_string(i + 1);
    sys.vsc_converters.push_back(c);
  }
}

int find_bus_index_by_original_id(const HybridPowerSystem& sys, int original_bus_id) {
  const std::string target = "Bus" + std::to_string(original_bus_id);
  for (const auto& b : sys.ac.buses) {
    if (b.name == target) {
      return b.index;
    }
  }
  return 0;
}

void attach_case300_style_mtdc(HybridPowerSystem& sys) {
  // Match the case300/case2000 MTDC overlay exactly:
  // - 6 DC buses, 6 DC branches (closed ring)
  // - 6 converters at mapped AC bus IDs [8, 76, 119, 149, 198, 243]
  // - alternating modes with PQ setpoints [0.50, -, 0.30, 0.40, -, 0.35] p.u.
  const std::vector<int> original_ids = {8, 76, 119, 149, 198, 243};

  std::vector<int> ac;
  ac.reserve(original_ids.size());
  for (int id : original_ids) {
    int idx = find_bus_index_by_original_id(sys, id);
    if (idx == 0) {
      idx = std::clamp(id, 1, static_cast<int>(sys.ac.buses.size()));
    }
    ac.push_back(idx);
  }

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " MTDC";
  sys.dc.buses.clear();
  for (int i = 0; i < 6; ++i) {
    DCBus b;
    b.index = i + 1;
    b.bus_type = (i == 0) ? DCBusType::DC_V : DCBusType::DC_P;
    b.vm_pu = 1.0;
    b.pd_mw = 0.0;
    b.in_service = true;
    b.name = "DCBus" + std::to_string(i + 1);
    sys.dc.buses.push_back(b);
  }

  sys.dc.branches = {
      DCBranch{.index = 1, .from_bus = 1, .to_bus = 2, .r_pu = 0.005, .in_service = true, .name = "DCLine1"},
      DCBranch{.index = 2, .from_bus = 2, .to_bus = 3, .r_pu = 0.005, .in_service = true, .name = "DCLine2"},
      DCBranch{.index = 3, .from_bus = 3, .to_bus = 4, .r_pu = 0.005, .in_service = true, .name = "DCLine3"},
      DCBranch{.index = 4, .from_bus = 4, .to_bus = 5, .r_pu = 0.005, .in_service = true, .name = "DCLine4"},
      DCBranch{.index = 5, .from_bus = 5, .to_bus = 6, .r_pu = 0.005, .in_service = true, .name = "DCLine5"},
      DCBranch{.index = 6, .from_bus = 6, .to_bus = 1, .r_pu = 0.005, .in_service = true, .name = "DCLine6"},
  };

  struct ConvSpec {
    ConverterMode mode;
    double pset_pu;
  };
  const std::vector<ConvSpec> specs = {
      {ConverterMode::PQ_MODE, 0.50},
      {ConverterMode::VDC_Q, 0.0},
      {ConverterMode::PQ_MODE, 0.30},
      {ConverterMode::PQ_MODE, 0.40},
      {ConverterMode::VDC_Q, 0.0},
      {ConverterMode::PQ_MODE, 0.35},
  };

  sys.vsc_converters.clear();
  sys.vsc_converters.reserve(6);
  for (int i = 0; i < 6; ++i) {
    const int ac_bus = std::clamp(ac[static_cast<size_t>(i)], 1, static_cast<int>(sys.ac.buses.size()));
    const ConvSpec spec = specs[static_cast<size_t>(i)];

    VSCConverter c;
    c.index = i + 1;
    c.bus_ac = ac_bus;
    c.bus_dc = i + 1;
    c.in_service = true;
    c.control_mode = spec.mode;
    c.p_set_mw = spec.pset_pu * sys.base_mva;
    c.q_set_mvar = 0.0;
    c.v_dc_set_pu = 1.0;
    c.v_ac_set_pu = sys.ac.buses[static_cast<size_t>(ac_bus - 1)].vm_pu;
    c.eta = 0.999;           // ~Ploss_c = 0.001
    c.loss_percent = 1.0;    // Ploss_b = 0.01 p.u.
    c.loss_mw = 0.1;         // Ploss_a = 0.001 p.u. on 100 MVA
    c.k_vdc = 0.1;
    c.pmax_mw = 3.0 * sys.base_mva;
    c.pmin_mw = -3.0 * sys.base_mva;
    c.qmax_mvar = 3.0 * sys.base_mva;
    c.qmin_mvar = -3.0 * sys.base_mva;
    c.name = "VSC" + std::to_string(i + 1);
    sys.vsc_converters.push_back(c);
  }
}

void attach_case2000_style_mtdc(HybridPowerSystem& sys) {
  const int nbus = static_cast<int>(sys.ac.buses.size());
  if (nbus == 0) {
    return;
  }

  // Build the case2000 AC/DC overlay:
  // 1) choose strongest generator-like AC bus per area,
  // 2) build one DC bus per area in a ring + cross-link,
  // 3) place one converter per area, alternating PQ/VDC_Q.
  std::map<int, std::pair<int, double>> area_best;  // area -> (ac_bus_idx, score)

  for (const auto& b : sys.ac.buses) {
    const bool generator_like =
        (b.bus_type == BusType::PV || b.bus_type == BusType::SLACK) && (b.pd_mw < 0.0);
    if (!generator_like) {
      continue;
    }
    const double pg_score = -b.pd_mw;
    auto it = area_best.find(b.area);
    if (it == area_best.end() || pg_score > it->second.second) {
      area_best[b.area] = {b.index, pg_score};
    }
  }

  if (area_best.empty()) {
    std::vector<double> bus_pg(static_cast<size_t>(nbus + 1), 0.0);
    for (const auto& gen : sys.ac.generators) {
      if (!gen.in_service) {
        continue;
      }
      const int bus = gen.bus;
      if (bus < 1 || bus > nbus) {
        continue;
      }
      bus_pg[static_cast<size_t>(bus)] += gen.pg_mw;
    }

    for (int bus = 1; bus <= nbus; ++bus) {
      const double pg_score = bus_pg[static_cast<size_t>(bus)];
      if (pg_score <= 0.0) {
        continue;
      }
      const int area = sys.ac.buses[static_cast<size_t>(bus - 1)].area;
      auto it = area_best.find(area);
      if (it == area_best.end() || pg_score > it->second.second) {
        area_best[area] = {bus, pg_score};
      }
    }
  }

  std::vector<int> conv_ac_buses;
  conv_ac_buses.reserve(area_best.size());
  for (const auto& [area, best] : area_best) {
    (void)area;
    conv_ac_buses.push_back(best.first);
  }

  if (conv_ac_buses.empty()) {
    attach_case300_style_mtdc(sys);
    return;
  }

  const int n_areas = static_cast<int>(conv_ac_buses.size());

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " MTDC";
  sys.dc.buses.clear();
  sys.dc.buses.reserve(static_cast<size_t>(n_areas));
  for (int i = 0; i < n_areas; ++i) {
    DCBus b;
    b.index = i + 1;
    b.bus_type = DCBusType::DC_P;
    b.vm_pu = 1.0;
    b.pd_mw = 0.0;
    b.in_service = true;
    b.name = "DCBus" + std::to_string(i + 1);
    sys.dc.buses.push_back(std::move(b));
  }

  sys.dc.branches.clear();
  sys.dc.branches.reserve(static_cast<size_t>(n_areas + 1));
  int br_idx = 1;
  for (int i = 1; i < n_areas; ++i) {
    sys.dc.branches.push_back(DCBranch{.index = br_idx++,
                                       .from_bus = i,
                                       .to_bus = i + 1,
                                       .r_pu = 0.003,
                                       .in_service = true,
                                       .name = "DCLine" + std::to_string(br_idx - 1)});
  }
  sys.dc.branches.push_back(DCBranch{.index = br_idx++,
                                     .from_bus = n_areas,
                                     .to_bus = 1,
                                     .r_pu = 0.003,
                                     .in_service = true,
                                     .name = "DCLine" + std::to_string(br_idx - 1)});
  if (n_areas >= 4) {
    const int to_bus = n_areas / 2 + 1;
    sys.dc.branches.push_back(DCBranch{.index = br_idx++,
                                       .from_bus = 1,
                                       .to_bus = to_bus,
                                       .r_pu = 0.005,
                                       .in_service = true,
                                       .name = "DCLine" + std::to_string(br_idx - 1)});
  }

  sys.vsc_converters.clear();
  sys.vsc_converters.reserve(static_cast<size_t>(n_areas));
  for (int i = 0; i < n_areas; ++i) {
    const int ac_bus = std::clamp(conv_ac_buses[static_cast<size_t>(i)], 1, nbus);
    const bool vdc_q_mode = ((i + 1) % 2 == 0);

    VSCConverter c;
    c.index = i + 1;
    c.bus_ac = ac_bus;
    c.bus_dc = i + 1;
    c.in_service = true;
    c.control_mode = vdc_q_mode ? ConverterMode::VDC_Q : ConverterMode::PQ_MODE;
    c.p_set_mw = (vdc_q_mode ? 0.0 : 0.50) * sys.base_mva;
    c.q_set_mvar = 0.0;
    c.v_dc_set_pu = 1.0;
    c.v_ac_set_pu = sys.ac.buses[static_cast<size_t>(ac_bus - 1)].vm_pu;
    c.eta = 0.999;           // 1 - Ploss_c (Ploss_c = 0.001)
    c.loss_percent = 1.0;    // Ploss_b = 0.01 p.u.
    c.loss_mw = 0.1;         // Ploss_a = 0.001 p.u. on 100 MVA
    c.k_vdc = 0.1;
    c.pmax_mw = 5.0 * sys.base_mva;
    c.pmin_mw = -5.0 * sys.base_mva;
    c.qmax_mvar = 5.0 * sys.base_mva;
    c.qmin_mvar = -5.0 * sys.base_mva;
    c.name = "VSC" + std::to_string(i + 1);
    sys.vsc_converters.push_back(std::move(c));
  }
}

HybridPowerSystem parse_case_with_overlay(const std::string& file,
                                          const std::string& name,
                                          int ac_bus_1,
                                          int ac_bus_2) {
  HybridPowerSystem sys = parse_matpower(data_file_path(file).string());
  sys.name = name;
  attach_two_terminal_dc(sys, ac_bus_1, ac_bus_2);
  return sys;
}

double sum_bus_group_load_mw(const HybridPowerSystem& sys,
                             const std::vector<int>& buses) {
  if (buses.empty()) return 0.0;
  auto contains_bus = [&](int bus) {
    return std::find(buses.begin(), buses.end(), bus) != buses.end();
  };

  double total = 0.0;
  if (!sys.ac.loads.empty()) {
    for (const auto& load : sys.ac.loads) {
      if (!load.in_service || !contains_bus(load.bus)) continue;
      total += load.p_mw * std::max(load.scaling, 1.0);
    }
    return total;
  }

  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service || !contains_bus(bus.index)) continue;
    total += std::max(0.0, bus.pd_mw);
  }
  return total;
}

double sum_bus_group_storage_mwh(const HybridPowerSystem& sys,
                                 const std::vector<int>& buses) {
  double total = 0.0;
  for (const auto& storage : sys.ac.storage) {
    if (!storage.in_service) continue;
    if (std::find(buses.begin(), buses.end(), storage.bus) == buses.end()) continue;
    total += std::max(0.0, storage.e_rated_mwh);
  }
  return total;
}

double sum_bus_group_generation_mw(const HybridPowerSystem& sys,
                                   const std::vector<int>& buses) {
  auto contains_bus = [&](int bus) {
    return std::find(buses.begin(), buses.end(), bus) != buses.end();
  };

  double total = 0.0;
  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service || !contains_bus(gen.bus)) continue;
    total += std::max({0.0, gen.pmax_mw, gen.pg_mw});
  }
  for (const auto& sgen : sys.ac.static_generators) {
    if (!sgen.in_service || !contains_bus(sgen.bus)) continue;
    total += std::max({0.0, sgen.pmax_mw, sgen.p_rated_mw, sgen.p_mw});
  }
  for (const auto& ren : sys.ac.renewable_gens) {
    if (!ren.in_service || !contains_bus(ren.bus)) continue;
    total += std::max({0.0, ren.p_rated_mw, ren.p_mw});
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service || !contains_bus(pv.bus)) continue;
    total += std::max({0.0, pv.pmax_mw, pv.p_mw});
  }
  return total;
}

double sum_bus_group_diesel_capacity_mw(const HybridPowerSystem& sys,
                                        const std::vector<int>& buses) {
  double total = 0.0;
  for (const auto& sgen : sys.ac.static_generators) {
    if (!sgen.in_service) continue;
    if (std::find(buses.begin(), buses.end(), sgen.bus) == buses.end()) continue;
    if (sgen.sgen_type != SgenType::Diesel) continue;
    total += std::max({0.0, sgen.pmax_mw, sgen.p_rated_mw, sgen.p_mw});
  }
  return total;
}

void attach_case33mg_microgrids(HybridPowerSystem& sys) {
  sys.microgrids.clear();

  auto make_microgrid = [&](int index,
                            const std::string& name,
                            const std::string& description,
                            int pcc_bus,
                            std::vector<int> internal_buses,
                            int area) {
    Microgrid mg;
    mg.index = index;
    mg.name = name;
    mg.description = description;
    mg.in_service = true;
    mg.pcc_bus = pcc_bus;
    mg.internal_buses = std::move(internal_buses);
    mg.operating_mode = MicrogridMode::GridConnected;
    mg.islanding_capability = true;
    mg.auto_reconnection = true;
    mg.area = area;
    mg.control_area = "case33mg-" + name;
    mg.total_load_mw = sum_bus_group_load_mw(sys, mg.internal_buses);
    mg.peak_load_mw = mg.total_load_mw;
    mg.total_generation_mw = sum_bus_group_generation_mw(sys, mg.internal_buses);
    mg.total_dg_capacity_mw = mg.total_generation_mw;
    mg.total_diesel_capacity_mw = sum_bus_group_diesel_capacity_mw(sys, mg.internal_buses);
    mg.total_storage_mwh = sum_bus_group_storage_mwh(sys, mg.internal_buses);
    mg.capacity_mw = mg.total_generation_mw;
    mg.p_import_max_mw = std::max(mg.total_load_mw, 0.5);
    mg.p_export_max_mw = std::max(mg.total_generation_mw, 0.0);
    mg.p_exchange_max_mw = mg.p_export_max_mw;
    mg.p_exchange_min_mw = -mg.p_import_max_mw;
    sys.microgrids.push_back(std::move(mg));
  };

  make_microgrid(
      1,
      "MG1",
      "Case33mg feeder section with PCC at bus 3 and internal buses 3-8.",
      3,
      {3, 4, 5, 6, 7, 8},
      1);
  make_microgrid(
      2,
      "MG2",
      "Case33mg feeder section with PCC at bus 12 and internal buses 12-17.",
      12,
      {12, 13, 14, 15, 16, 17},
      2);
  make_microgrid(
      3,
      "MG3",
      "Case33mg feeder section with PCC at bus 25 and internal buses 25-33.",
      25,
      {25, 26, 27, 28, 29, 30, 31, 32, 33},
      3);
}

HybridPowerSystem build_ieee14_ac_base() {
  HybridPowerSystem sys;
  sys.name = "IEEE14 AC Base";
  sys.base_mva = 100.0;

  sys.ac.base_mva = 100.0;
  sys.ac.name = "IEEE14 AC";
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 0.0, 0.0, 1.06, 0.0),
      make_ac_bus(2, BusType::PV, 21.7, 12.7, 1.045, -4.98),
      make_ac_bus(3, BusType::PV, 94.2, 19.0, 1.01, -12.72),
      make_ac_bus(4, BusType::PQ, 47.8, -3.9, 1.019, -10.33),
      make_ac_bus(5, BusType::PQ, 7.6, 1.6, 1.02, -8.78),
      make_ac_bus(6, BusType::PV, 11.2, 7.5, 1.07, -14.22),
      make_ac_bus(7, BusType::PQ, 0.0, 0.0, 1.062, -13.37),
      make_ac_bus(8, BusType::PV, 0.0, 0.0, 1.09, -13.36),
      make_ac_bus(9, BusType::PQ, 29.5, 16.6, 1.056, -14.94),
      make_ac_bus(10, BusType::PQ, 9.0, 5.8, 1.051, -15.1),
      make_ac_bus(11, BusType::PQ, 3.5, 1.8, 1.057, -14.79),
      make_ac_bus(12, BusType::PQ, 6.1, 1.6, 1.055, -15.07),
      make_ac_bus(13, BusType::PQ, 13.5, 5.8, 1.05, -15.16),
      make_ac_bus(14, BusType::PQ, 14.9, 5.0, 1.036, -16.04),
  };

  sys.ac.branches = {
      make_ac_branch(1, 1, 2, 0.01938, 0.05917, 0.0528, 1.0),
      make_ac_branch(2, 1, 5, 0.05403, 0.22304, 0.0492, 1.0),
      make_ac_branch(3, 2, 3, 0.04699, 0.19797, 0.0438, 1.0),
      make_ac_branch(4, 2, 4, 0.05811, 0.17632, 0.0340, 1.0),
      make_ac_branch(5, 2, 5, 0.05695, 0.17388, 0.0346, 1.0),
      make_ac_branch(6, 3, 4, 0.06701, 0.17103, 0.0128, 1.0),
      make_ac_branch(7, 4, 5, 0.01335, 0.04211, 0.0, 1.0),
      make_ac_branch(8, 4, 7, 0.0, 0.20912, 0.0, 0.978),
      make_ac_branch(9, 4, 9, 0.0, 0.55618, 0.0, 0.969),
      make_ac_branch(10, 5, 6, 0.0, 0.25202, 0.0, 0.932),
      make_ac_branch(11, 6, 11, 0.09498, 0.19890, 0.0, 1.0),
      make_ac_branch(12, 6, 12, 0.12291, 0.25581, 0.0, 1.0),
      make_ac_branch(13, 6, 13, 0.06615, 0.13027, 0.0, 1.0),
      make_ac_branch(14, 7, 8, 0.0, 0.17615, 0.0, 1.0),
      make_ac_branch(15, 7, 9, 0.0, 0.11001, 0.0, 1.0),
      make_ac_branch(16, 9, 10, 0.03181, 0.08450, 0.0, 1.0),
      make_ac_branch(17, 9, 14, 0.12711, 0.27038, 0.0, 1.0),
      make_ac_branch(18, 10, 11, 0.08205, 0.19207, 0.0, 1.0),
      make_ac_branch(19, 12, 13, 0.22092, 0.19988, 0.0, 1.0),
      make_ac_branch(20, 13, 14, 0.17093, 0.34802, 0.0, 1.0),
  };

  sys.ac.generators = {
      make_generator(1, 1, true, 232.4, -16.9, 1.06, 332.4, 0.0, 10.0, 0.0),
      make_generator(2, 2, false, 40.0, 43.5, 1.045, 140.0, 0.0, 50.0, -40.0),
      make_generator(3, 3, false, 0.0, 25.0, 1.01, 100.0, 0.0, 40.0, 0.0),
      make_generator(4, 6, false, 0.0, 12.2, 1.07, 100.0, 0.0, 24.0, -6.0),
      make_generator(5, 8, false, 0.0, 17.4, 1.09, 100.0, 0.0, 24.0, -6.0),
  };

  return sys;
}

double pu_to_mw(double p) { return p * 100.0; }

}  // namespace

HybridPowerSystem build_ieee14_acdc() {
  HybridPowerSystem sys = build_ieee14_ac_base();
  sys.name = "IEEE14 AC/DC";
  attach_two_terminal_dc(sys, 5, 9, 1.02, 0.02);
  return sys;
}

HybridPowerSystem build_ieee24_3area_acdc() {
  HybridPowerSystem sys;
  sys.name = "IEEE24-3area AC/DC";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.name = "IEEE24 AC";

  sys.ac.buses = {
      make_ac_bus(1, BusType::PV, pu_to_mw(1.08), pu_to_mw(0.22), 1.035, 0.0, 1),
      make_ac_bus(2, BusType::PV, pu_to_mw(0.97), pu_to_mw(0.20), 1.035, 0.0, 1),
      make_ac_bus(3, BusType::PQ, pu_to_mw(1.80), pu_to_mw(0.37), 1.0, 0.0, 1),
      make_ac_bus(4, BusType::PQ, pu_to_mw(0.74), pu_to_mw(0.15), 1.0, 0.0, 1),
      make_ac_bus(5, BusType::PQ, pu_to_mw(0.71), pu_to_mw(0.14), 1.0, 0.0, 1),
      make_ac_bus(6, BusType::PQ, pu_to_mw(1.36), pu_to_mw(0.28), 1.0, 0.0, 1),
      make_ac_bus(7, BusType::PV, pu_to_mw(1.25), pu_to_mw(0.25), 1.025, 0.0, 1),
      make_ac_bus(8, BusType::PQ, pu_to_mw(1.71), pu_to_mw(0.35), 1.0, 0.0, 1),
      make_ac_bus(9, BusType::PQ, pu_to_mw(1.75), pu_to_mw(0.36), 1.0, 0.0, 2),
      make_ac_bus(10, BusType::PQ, pu_to_mw(1.95), pu_to_mw(0.40), 1.0, 0.0, 2),
      make_ac_bus(11, BusType::PQ, 0.0, 0.0, 1.0, 0.0, 2),
      make_ac_bus(12, BusType::PQ, 0.0, 0.0, 1.0, 0.0, 2),
      make_ac_bus(13, BusType::SLACK, pu_to_mw(2.65), pu_to_mw(0.54), 1.02, 0.0, 2),
      // Bus 14 has no generator or converter voltage controller.  Keeping it
      // as PV leaves its solved reactive injection without an owning device.
      make_ac_bus(14, BusType::PQ, pu_to_mw(1.94), pu_to_mw(0.39), 1.0, 0.0, 2),
      make_ac_bus(15, BusType::PV, pu_to_mw(3.17), pu_to_mw(0.64), 1.014, 0.0, 2),
      make_ac_bus(16, BusType::PV, pu_to_mw(1.00), pu_to_mw(0.20), 1.017, 0.0, 2),
      make_ac_bus(17, BusType::PQ, 0.0, 0.0, 1.0, 0.0, 3),
      make_ac_bus(18, BusType::PV, pu_to_mw(3.33), pu_to_mw(0.68), 1.05, 0.0, 3),
      make_ac_bus(19, BusType::PQ, pu_to_mw(1.81), pu_to_mw(0.37), 1.0, 0.0, 3),
      make_ac_bus(20, BusType::PQ, pu_to_mw(1.28), pu_to_mw(0.26), 1.0, 0.0, 3),
      make_ac_bus(21, BusType::PV, 0.0, 0.0, 1.05, 0.0, 3),
      make_ac_bus(22, BusType::PV, 0.0, 0.0, 1.05, 0.0, 3),
      make_ac_bus(23, BusType::PV, 0.0, 0.0, 1.05, 0.0, 3),
      make_ac_bus(24, BusType::PQ, 0.0, 0.0, 1.0, 0.0, 3),
  };

  // Tighten voltage limits to standard operational range [0.95, 1.05] p.u.
  for (auto& b : sys.ac.buses) {
    b.vmax_pu = 1.05;
    b.vmin_pu = 0.95;
  }

  sys.ac.generators = {
      make_generator(1, 1, false, pu_to_mw(1.92), 0.0, 1.035, pu_to_mw(3.0), 0.0, pu_to_mw(1.0), -pu_to_mw(1.0)),
      make_generator(2, 2, false, pu_to_mw(1.92), 0.0, 1.035, pu_to_mw(3.0), 0.0, pu_to_mw(1.0), -pu_to_mw(1.0)),
      make_generator(3, 7, false, pu_to_mw(2.40), 0.0, 1.025, pu_to_mw(4.0), 0.0, pu_to_mw(1.0), -pu_to_mw(1.0)),
      make_generator(4, 13, true, pu_to_mw(5.91), 0.0, 1.020, pu_to_mw(8.0), 0.0, pu_to_mw(2.0), -pu_to_mw(2.0)),
      make_generator(5, 15, false, pu_to_mw(2.15), 0.0, 1.014, pu_to_mw(4.0), 0.0, pu_to_mw(1.0), -pu_to_mw(1.0)),
      make_generator(6, 16, false, pu_to_mw(1.55), 0.0, 1.017, pu_to_mw(3.0), 0.0, pu_to_mw(1.0), -pu_to_mw(1.0)),
      make_generator(7, 18, false, pu_to_mw(4.00), 0.0, 1.050, pu_to_mw(6.0), 0.0, pu_to_mw(2.0), -pu_to_mw(2.0)),
      make_generator(8, 21, false, pu_to_mw(4.00), 0.0, 1.050, pu_to_mw(6.0), 0.0, pu_to_mw(2.0), -pu_to_mw(2.0)),
      make_generator(9, 22, false, pu_to_mw(3.00), 0.0, 1.050, pu_to_mw(5.0), 0.0, pu_to_mw(2.0), -pu_to_mw(2.0)),
      make_generator(10, 23, false, pu_to_mw(6.60), 0.0, 1.050, pu_to_mw(8.0), 0.0, pu_to_mw(3.0), -pu_to_mw(3.0)),
  };
  // Attach realistic UC metadata so commitment decisions are meaningful.
  const std::vector<double> c1 = {12.0, 13.5, 15.0, 11.0, 16.0, 18.0, 20.0, 21.5, 23.0, 25.0};
  const std::vector<double> c0 = {22.0, 24.0, 26.0, 20.0, 30.0, 34.0, 38.0, 40.0, 44.0, 48.0};
  const std::vector<double> startup = {120.0, 140.0, 170.0, 160.0, 180.0, 200.0, 240.0, 250.0, 260.0, 300.0};
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    auto& g = sys.ac.generators[i];
    g.cost_c2 = 0.0025;
    g.cost_c1 = c1[std::min(i, c1.size() - 1)];
    g.cost_c0 = c0[std::min(i, c0.size() - 1)];
    g.startup_cost = startup[std::min(i, startup.size() - 1)];
    g.shutdown_cost = 0.2 * g.startup_cost;
    g.min_up_time_hr = 2.0;
    g.min_dn_time_hr = 2.0;
    g.ramp_up_mw_min = std::max(1.0, 0.015 * g.pmax_mw);
    g.ramp_dn_mw_min = std::max(1.0, 0.015 * g.pmax_mw);
    g.fuel_type = (i <= 3) ? FuelType::Coal : FuelType::Gas;
    g.emission_factor_tco2_mwh = (i <= 3) ? 0.85 : 0.42;
  }

  sys.ac.branches = {
      make_ac_branch(1, 1, 2, 0.0026, 0.0139, 0.4611, 1.0),
      make_ac_branch(2, 1, 3, 0.0546, 0.2112, 0.0572, 1.0),
      make_ac_branch(3, 1, 5, 0.0218, 0.0845, 0.0229, 1.0),
      make_ac_branch(4, 2, 4, 0.0328, 0.1267, 0.0343, 1.0),
      make_ac_branch(5, 2, 6, 0.0497, 0.1920, 0.0520, 1.0),
      make_ac_branch(6, 3, 9, 0.0308, 0.1190, 0.0322, 1.0),
      make_ac_branch(7, 4, 9, 0.0268, 0.1037, 0.0281, 1.0),
      make_ac_branch(8, 5, 10, 0.0228, 0.0883, 0.0239, 1.0),
      make_ac_branch(9, 6, 10, 0.0139, 0.0605, 0.2459, 1.0),
      make_ac_branch(10, 7, 8, 0.0159, 0.0614, 0.0166, 1.0),
      make_ac_branch(11, 3, 24, 0.0023, 0.0839, 0.0, 1.015),
      make_ac_branch(12, 8, 10, 0.0427, 0.1651, 0.0447, 1.0),
      make_ac_branch(13, 9, 11, 0.0023, 0.0839, 0.0, 1.03),
      make_ac_branch(14, 9, 12, 0.0023, 0.0839, 0.0, 1.03),
      make_ac_branch(15, 10, 11, 0.0023, 0.0839, 0.0, 1.02),
      make_ac_branch(16, 10, 12, 0.0023, 0.0839, 0.0, 1.02),
      make_ac_branch(17, 11, 13, 0.0061, 0.0476, 0.0999, 1.0),
      make_ac_branch(18, 11, 14, 0.0054, 0.0418, 0.0879, 1.0),
      make_ac_branch(19, 12, 13, 0.0061, 0.0476, 0.0999, 1.0),
      make_ac_branch(20, 12, 23, 0.0124, 0.0966, 0.2030, 1.0),
      make_ac_branch(21, 13, 23, 0.0111, 0.0865, 0.1818, 1.0),
      make_ac_branch(22, 14, 16, 0.0050, 0.0389, 0.0818, 1.0),
      make_ac_branch(23, 15, 16, 0.0022, 0.0173, 0.0364, 1.0),
      make_ac_branch(24, 15, 21, 0.0063, 0.0490, 0.1030, 1.0),
      make_ac_branch(25, 15, 24, 0.0067, 0.0519, 0.1091, 1.0),
      make_ac_branch(26, 16, 17, 0.0033, 0.0259, 0.0545, 1.0),
      make_ac_branch(27, 16, 19, 0.0030, 0.0231, 0.0485, 1.0),
      make_ac_branch(28, 17, 18, 0.0018, 0.0144, 0.0303, 1.0),
      make_ac_branch(29, 17, 22, 0.0135, 0.1053, 0.2212, 1.0),
      make_ac_branch(30, 18, 21, 0.0033, 0.0259, 0.0545, 1.0),
      make_ac_branch(31, 19, 20, 0.0025, 0.0198, 0.0417, 1.0),
      make_ac_branch(32, 20, 23, 0.0022, 0.0173, 0.0364, 1.0),
      make_ac_branch(33, 21, 22, 0.0087, 0.0678, 0.1424, 1.0),
  };

  sys.dc.base_mva = 100.0;
  sys.dc.name = "IEEE24 MTDC";
  sys.dc.buses = {
      DCBus{.index = 1, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus1"},
      DCBus{.index = 2, .bus_type = DCBusType::DC_V, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus2"},
      DCBus{.index = 3, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus3"},
      DCBus{.index = 4, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus4"},
  };
  sys.dc.branches = {
      DCBranch{.index = 1, .from_bus = 1, .to_bus = 2, .r_pu = 0.005, .in_service = true, .name = "DCLine1"},
      DCBranch{.index = 2, .from_bus = 3, .to_bus = 4, .r_pu = 0.005, .in_service = true, .name = "DCLine2"},
      DCBranch{.index = 3, .from_bus = 2, .to_bus = 3, .r_pu = 0.003, .in_service = true, .name = "DCLine3"},
  };
  sys.vsc_converters = {
      VSCConverter{.index = 1, .bus_ac = 7, .bus_dc = 1, .in_service = true, .control_mode = ConverterMode::PQ_MODE, .p_set_mw = 20.0, .q_set_mvar = 0.0, .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.025, .eta = 0.99, .loss_percent = 0.0, .loss_mw = 0.0, .k_vdc = 0.1, .pmax_mw = 200.0, .pmin_mw = -200.0, .qmax_mvar = 200.0, .qmin_mvar = -200.0, .name = "VSC1"},
      VSCConverter{.index = 2, .bus_ac = 13, .bus_dc = 2, .in_service = true, .control_mode = ConverterMode::VDC_Q, .p_set_mw = 0.0, .q_set_mvar = 0.0, .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.02, .eta = 0.99, .loss_percent = 0.0, .loss_mw = 0.0, .k_vdc = 0.1, .pmax_mw = 200.0, .pmin_mw = -200.0, .qmax_mvar = 200.0, .qmin_mvar = -200.0, .name = "VSC2"},
      VSCConverter{.index = 3, .bus_ac = 15, .bus_dc = 3, .in_service = true, .control_mode = ConverterMode::PQ_MODE, .p_set_mw = 15.0, .q_set_mvar = 0.0, .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.014, .eta = 0.99, .loss_percent = 0.0, .loss_mw = 0.0, .k_vdc = 0.1, .pmax_mw = 200.0, .pmin_mw = -200.0, .qmax_mvar = 200.0, .qmin_mvar = -200.0, .name = "VSC3"},
      VSCConverter{.index = 4, .bus_ac = 21, .bus_dc = 4, .in_service = true, .control_mode = ConverterMode::VDC_Q, .p_set_mw = 0.0, .q_set_mvar = 0.0, .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.05, .eta = 0.99, .loss_percent = 0.0, .loss_mw = 0.0, .k_vdc = 0.1, .pmax_mw = 200.0, .pmin_mw = -200.0, .qmax_mvar = 200.0, .qmin_mvar = -200.0, .name = "VSC4"},
  };

  apply_ieee24_reference_coordinates(sys);
  return sys;
}

HybridPowerSystem build_ieee24_3area_acdc_expanded() {
  HybridPowerSystem sys = build_ieee24_3area_acdc();
  sys.name = "IEEE24-3area AC/DC Expanded";

  // Extend DC network from 4 to 8 buses.
  auto add_dc_bus = [&](int idx, DCBusType t, double kv, int area, const std::string& name) {
    DCBus b;
    b.index = idx;
    b.bus_type = t;
    b.vm_pu = 1.0;
    b.vmax_pu = 1.1;
    b.vmin_pu = 0.9;
    b.base_kv = kv;
    b.area = area;
    b.in_service = true;
    b.name = name;
    b.longitude = 0.0;
    b.latitude = 0.0;
    sys.dc.buses.push_back(b);
  };
  add_dc_bus(5, DCBusType::DC_P, 320.0, 1, "DCBus5");
  add_dc_bus(6, DCBusType::DC_P, 320.0, 2, "DCBus6");
  add_dc_bus(7, DCBusType::DC_P, 320.0, 3, "DCBus7");
  add_dc_bus(8, DCBusType::DC_P, 320.0, 3, "DCBus8");

  auto add_dc_branch = [&](int idx, int f, int t, double r, const std::string& name) {
    DCBranch br;
    br.index = idx;
    br.from_bus = f;
    br.to_bus = t;
    br.r_pu = r;
    br.rate_a_mva = 250.0;
    br.in_service = true;
    br.name = name;
    br.length_km = 60.0;
    br.base_kv = 320.0;
    sys.dc.branches.push_back(br);
  };
  add_dc_branch(4, 2, 5, 0.0035, "DCLine4");
  add_dc_branch(5, 5, 6, 0.0040, "DCLine5");
  add_dc_branch(6, 6, 4, 0.0035, "DCLine6");
  add_dc_branch(7, 3, 7, 0.0042, "DCLine7");
  add_dc_branch(8, 7, 8, 0.0038, "DCLine8");
  add_dc_branch(9, 8, 1, 0.0040, "DCLine9");

  auto add_vsc = [&](int idx, int ac_bus, int dc_bus, ConverterMode mode, double p_set_mw,
                     double vac_set, const std::string& name) {
    VSCConverter c;
    c.index = idx;
    c.bus_ac = ac_bus;
    c.bus_dc = dc_bus;
    c.in_service = true;
    c.control_mode = mode;
    c.p_set_mw = p_set_mw;
    c.q_set_mvar = 0.0;
    c.v_dc_set_pu = 1.0;
    c.v_ac_set_pu = vac_set;
    c.eta = 0.99;
    c.k_vdc = 0.1;
    c.pmax_mw = 250.0;
    c.pmin_mw = -250.0;
    c.qmax_mvar = 200.0;
    c.qmin_mvar = -200.0;
    c.name = name;
    sys.vsc_converters.push_back(c);
  };
  add_vsc(5, 4, 5, ConverterMode::PQ_MODE, 25.0, 1.01, "VSC5");
  add_vsc(6, 10, 6, ConverterMode::VDC_Q, 0.0, 1.01, "VSC6");
  add_vsc(7, 18, 7, ConverterMode::PQ_MODE, -20.0, 1.04, "VSC7");
  add_vsc(8, 24, 8, ConverterMode::VDC_Q, 0.0, 1.00, "VSC8");

  auto add_dcdc = [&](int idx, int bin, int bout, double pref, double eta, const std::string& name) {
    DCDCConverter d;
    d.index = idx;
    d.bus_in = bin;
    d.bus_out = bout;
    d.in_service = true;
    d.name = name;
    d.control_mode = DCDCControlMode::Power;
    d.p_ref_mw = pref;
    d.eta = eta;
    d.pmax_mw = 120.0;
    d.pmin_mw = -120.0;
    d.sn_mva = 150.0;
    d.v_ref_pu = 1.0;
    sys.dc.dcdc_converters.push_back(d);
  };
  add_dcdc(1, 5, 7, 35.0, 0.985, "DCDC1");
  add_dcdc(2, 6, 8, 30.0, 0.982, "DCDC2");

  // Add richer DC-side components for scalability/cross-domain coupling.
  auto add_dc_load = [&](int idx, int bus, double p, int profile_id, const std::string& name) {
    DCLoad ld;
    ld.index = idx;
    ld.bus = bus;
    ld.in_service = true;
    ld.name = name;
    ld.p_mw = p;
    ld.profile_id = profile_id;
    ld.controllable = true;
    ld.p_min_mw = 0.5 * p;
    ld.cost_mw = 80.0;
    sys.dc.loads.push_back(ld);
  };
  add_dc_load(1, 2, 35.0, 0, "DCLoad2");
  add_dc_load(2, 4, 28.0, 0, "DCLoad4");
  add_dc_load(3, 6, 22.0, 0, "DCLoad6");
  add_dc_load(4, 8, 26.0, 0, "DCLoad8");

  auto add_dc_pv = [&](int idx, int bus, double pset, int profile_id, const std::string& name) {
    PVArrayDC pv;
    pv.index = idx;
    pv.bus = bus;
    pv.in_service = true;
    pv.name = name;
    pv.p_set_mw = pset;
    pv.profile_id = profile_id;
    pv.num_series = 24;
    pv.num_parallel = 4;
    pv.irradiance = 1000.0;
    sys.dc.pv_arrays.push_back(pv);
  };
  add_dc_pv(1, 3, 45.0, 2, "DCPV3");
  add_dc_pv(2, 7, 38.0, 2, "DCPV7");

  auto add_dc_sgen = [&](int idx, int bus, const std::string& type, double p_set,
                         double scaling, int profile_id, const std::string& name) {
    StaticGeneratorDC sg;
    sg.index = idx;
    sg.bus = bus;
    sg.in_service = true;
    sg.name = name;
    sg.type = type;
    sg.p_set_mw = p_set;
    sg.scaling = scaling;
    sg.profile_id = profile_id;
    sg.pmax_mw = p_set;
    sg.pmin_mw = 0.0;
    sg.controllable = false;
    sys.dc.dc_static_generators.push_back(sg);
  };
  add_dc_sgen(1, 5, "Wind", 32.0, 1.0, 1, "DCWind5");
  add_dc_sgen(2, 8, "Wind", 28.0, 1.0, 1, "DCWind8");

  auto add_dc_storage = [&](int idx, int bus, double pmax, double erated, double soc_ci, const std::string& name) {
    Storage st;
    st.index = idx;
    st.bus = bus;
    st.in_service = true;
    st.name = name;
    st.p_mw = 0.0;
    st.p_rated_mw = pmax;
    st.pmax_mw = pmax;
    st.pmin_mw = -pmax;
    st.e_rated_mwh = erated;
    st.soc_init = 0.5;
    st.soc_min = 0.1;
    st.soc_max = 0.9;
    st.eta_charge = 0.95;
    st.eta_discharge = 0.95;
    st.soc_carbon_intensity_tco2_mwh = soc_ci;
    sys.dc.storage.push_back(st);
  };
  add_dc_storage(1, 6, 35.0, 120.0, 0.25, "DCESS6");
  add_dc_storage(2, 8, 30.0, 100.0, 0.30, "DCESS8");

  // Add AC storage assets so UC can produce explicit charge/discharge schedules.
  auto add_ac_storage = [&](int idx, int bus, double pmax, double erated,
                            double soc_init, const std::string& name) {
    Storage st;
    st.index = idx;
    st.bus = bus;
    st.in_service = true;
    st.name = name;
    st.p_mw = 0.0;
    st.p_rated_mw = pmax;
    st.pmax_mw = pmax;
    st.pmin_mw = -pmax;
    st.e_rated_mwh = erated;
    st.soc_init = soc_init;
    st.soc_min = 0.1;
    st.soc_max = 0.9;
    st.eta_charge = 0.95;
    st.eta_discharge = 0.95;
    st.self_discharge_pct = 0.0;
    st.controllable = true;
    sys.ac.storage.push_back(st);
  };
  add_ac_storage(1, 6, 80.0, 240.0, 0.55, "ACESS6");
  add_ac_storage(2, 19, 60.0, 180.0, 0.50, "ACESS19");

  // Anchor each DC bus near the associated AC converter station.
  std::unordered_map<int, std::pair<double, double>> ac_geo;
  ac_geo.reserve(sys.ac.buses.size());
  for (const auto& b : sys.ac.buses) {
    ac_geo[b.index] = {b.longitude, b.latitude};
  }
  for (auto& db : sys.dc.buses) {
    bool anchored = false;
    for (const auto& c : sys.vsc_converters) {
      if (!c.in_service || c.bus_dc != db.index) continue;
      auto it = ac_geo.find(c.bus_ac);
      if (it == ac_geo.end()) continue;
      const double sign_lon = (db.index % 2 == 0) ? -1.0 : 1.0;
      const double sign_lat = (db.index % 3 == 0) ? -1.0 : 1.0;
      db.longitude = it->second.first + sign_lon * 0.06;
      db.latitude = it->second.second + sign_lat * 0.04;
      anchored = true;
      break;
    }
    if (!anchored) {
      db.longitude = 120.8 + 0.08 * static_cast<double>(db.index);
      db.latitude = 30.2 + 0.05 * static_cast<double>(db.index % 4);
    }
  }

  // ── OLTC Transformers for Reactive Power Optimization ──────────
  {
    auto add_oltc = [&](int idx, int source_branch_idx, int hv, int lv,
                        double sn, double vn_hv,
                        double vn_lv, double vk, double vkr, int tap_pos,
                        int tap_min, int tap_max, double step_pct,
                        const std::string& name) {
      Transformer2W t;
      t.index = idx;
      t.hv_bus = hv;  t.lv_bus = lv;
      t.sn_mva = sn;
      t.vn_hv_kv = vn_hv;  t.vn_lv_kv = vn_lv;
      t.vk_percent = vk;   t.vkr_percent = vkr;
      t.tap_side = 0;      // HV side
      t.tap_pos = tap_pos;
      t.tap_neutral = 0;
      t.tap_min = tap_min;
      t.tap_max = tap_max;
      t.tap_step_percent = step_pct;
      t.source_branch_idx = source_branch_idx;
      t.in_service = true;
      t.name = name;
      sys.ac.transformers_2w.push_back(t);
    };
    // These are equipment records for existing tap branches, not additional
    // parallel network paths. source_branch_idx keeps topology and reporting
    // in the same authored branch index space.
    add_oltc(1, 11, 3, 24, 400, 230, 138, 8.39, 0.23, 0, -5, 5, 1.25, "OLTC-3/24");
    add_oltc(2, 13, 9, 11, 400, 230, 138, 8.39, 0.23, 0, -5, 5, 1.25, "OLTC-9/11");
    add_oltc(3, 16, 10, 12, 400, 230, 138, 8.39, 0.23, 0, -4, 4, 1.5, "OLTC-10/12");
  }

  // ── Switchable Shunts for Reactive Power Optimization ──────────
  {
    auto add_shunt = [&](int idx, int bus, double bs_total, int n_steps,
                         int cur_step, const std::string& name) {
      Shunt sh;
      sh.index = idx;
      sh.bus = bus;
      sh.in_service = true;
      sh.switchable = true;
      sh.n_steps = n_steps;
      sh.current_step = cur_step;
      sh.bs_per_step = bs_total / n_steps;
      sh.bs_mvar = sh.bs_per_step * cur_step;
      sh.gs_mw = 0.0;
      sh.name = name;
      sys.ac.shunts.push_back(sh);
    };
    // Capacitor banks at load buses in different areas
    add_shunt(1, 6,  60.0, 4, 2, "CapBank-B6");
    add_shunt(2, 8,  40.0, 3, 1, "CapBank-B8");
    add_shunt(3, 14, 50.0, 5, 3, "CapBank-B14");
    add_shunt(4, 20, 30.0, 3, 1, "CapBank-B20");
  }

  return sys;
}

HybridPowerSystem build_ieee118_acdc() {
  HybridPowerSystem sys = parse_matpower(data_file_path("case118.m").string());
  sys.name = "IEEE118 AC/DC";
  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " MTDC";

  sys.dc.buses = {
      DCBus{.index = 1, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus1"},
      DCBus{.index = 2, .bus_type = DCBusType::DC_V, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus2"},
      DCBus{.index = 3, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus3"},
      DCBus{.index = 4, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus4"},
      DCBus{.index = 5, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus5"},
      DCBus{.index = 6, .bus_type = DCBusType::DC_P, .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DCBus6"},
  };

  sys.dc.branches = {
      DCBranch{.index = 1, .from_bus = 1, .to_bus = 3, .r_pu = 0.005, .in_service = true, .name = "DCLine1"},
      DCBranch{.index = 2, .from_bus = 2, .to_bus = 4, .r_pu = 0.005, .in_service = true, .name = "DCLine2"},
      DCBranch{.index = 3, .from_bus = 3, .to_bus = 5, .r_pu = 0.005, .in_service = true, .name = "DCLine3"},
      DCBranch{.index = 4, .from_bus = 4, .to_bus = 6, .r_pu = 0.005, .in_service = true, .name = "DCLine4"},
      DCBranch{.index = 5, .from_bus = 1, .to_bus = 2, .r_pu = 0.003, .in_service = true, .name = "DCLine5"},
  };

  struct ConvSpec {
    int ac_bus;
    int dc_bus;
    ConverterMode mode;
    double pset_pu;
    double vac_set_pu;
  };
  const std::vector<ConvSpec> specs = {
      {10, 1, ConverterMode::PQ_MODE, 0.20, 1.050},
      {25, 2, ConverterMode::VDC_Q, 0.0, 1.050},
      {49, 3, ConverterMode::PQ_MODE, 0.15, 1.025},
      {59, 4, ConverterMode::VDC_Q, 0.0, 0.985},
      {80, 5, ConverterMode::PQ_MODE, 0.10, 1.040},
      {89, 6, ConverterMode::VDC_Q, 0.0, 1.005},
  };

  sys.vsc_converters.clear();
  sys.vsc_converters.reserve(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    const auto& spec = specs[i];
    const int ac_bus = std::clamp(spec.ac_bus, 1, static_cast<int>(sys.ac.buses.size()));
    VSCConverter c;
    c.index = static_cast<int>(i) + 1;
    c.bus_ac = ac_bus;
    c.bus_dc = spec.dc_bus;
    c.in_service = true;
    c.control_mode = spec.mode;
    c.p_set_mw = spec.pset_pu * sys.base_mva;
    c.q_set_mvar = 0.0;
    c.v_dc_set_pu = 1.0;
    c.v_ac_set_pu = spec.vac_set_pu;
    c.eta = 0.999;
    c.loss_percent = 1.0;
    c.loss_mw = 0.1;
    c.k_vdc = 0.1;
    c.pmax_mw = 3.0 * sys.base_mva;
    c.pmin_mw = -3.0 * sys.base_mva;
    c.qmax_mvar = 1.5 * sys.base_mva;
    c.qmin_mvar = -1.5 * sys.base_mva;
    c.name = "VSC" + std::to_string(c.index);
    sys.vsc_converters.push_back(std::move(c));
  }
  return sys;
}

HybridPowerSystem build_ac_only_version(const HybridPowerSystem& sys) {
  HybridPowerSystem copy = sys;
  copy.dc.buses.clear();
  copy.dc.branches.clear();
  copy.vsc_converters.clear();
  copy.name += " [AC only]";
  return copy;
}

HybridPowerSystem build_case33bw_acdc() {
  return parse_case_with_overlay("case33bw.m", "case33bw AC/DC", 1, 18);
}

HybridPowerSystem build_case33mg_acdc() {
  HybridPowerSystem sys = parse_case_with_overlay("case33mg.m", "case33mg AC/DC", 1, 18);
  attach_case33mg_microgrids(sys);
  return sys;
}

HybridPowerSystem build_case69_acdc() {
  return parse_case_with_overlay("case69.m", "case69 AC/DC", 1, 35);
}

HybridPowerSystem build_case300_acdc() {
  HybridPowerSystem sys = parse_matpower(data_file_path("case300.m").string());
  sys.name = "case300 AC/DC";
  // MATPOWER does not describe OLTC ranges.  Keep all imported transformer
  // ratios fixed by default, then explicitly promote a small, auditable subset
  // for this authored AC/DC demonstration instead of treating all 62 fixed tap
  // branches as 4001-position controls.
  const size_t oltc_count = std::min<size_t>(3, sys.ac.transformers_2w.size());
  for (size_t i = 0; i < oltc_count; ++i) {
    auto& transformer = sys.ac.transformers_2w[i];
    transformer.name = "OLTC-B" + std::to_string(transformer.hv_bus) + "/B" +
                       std::to_string(transformer.lv_bus);
    transformer.tap_pos = 0;
    transformer.tap_neutral = 0;
    transformer.tap_min = -4;
    transformer.tap_max = 4;
    transformer.tap_step_percent = 1.25;
  }
  attach_case300_style_mtdc(sys);
  return sys;
}

HybridPowerSystem build_demo_multizone_acdc() {
  HybridPowerSystem sys = build_ieee24_3area_acdc();
  sys.name = "demo_multizone_acdc";

  struct GeoCenter {
    double lon;
    double lat;
  };
  const std::map<int, GeoCenter> centers = {
      {1, {121.35, 31.20}},
      {2, {118.80, 32.02}},
      {3, {120.15, 30.25}},
  };

  for (auto& b : sys.ac.buses) {
    const auto it_center = centers.find(b.area);
    const GeoCenter c = (it_center != centers.end())
                            ? it_center->second
                            : GeoCenter{121.0, 31.0};

    int local = 0;
    if (b.area == 1) {
      local = b.index - 1;
    } else if (b.area == 2) {
      local = b.index - 9;
    } else {
      local = b.index - 17;
    }
    local = std::max(0, local);
    const int row = local / 4;
    const int col = local % 4;

    b.zone = (col < 2) ? 1 : 2;
    b.longitude = c.lon + 0.11 * static_cast<double>(col) + ((row % 2 == 0) ? 0.01 : -0.01);
    b.latitude = c.lat + 0.09 * static_cast<double>(row) + ((col % 2 == 0) ? 0.006 : -0.006);

    if (b.bus_type == BusType::SLACK) {
      b.base_kv = 500.0;
    } else if (b.bus_type == BusType::PV) {
      b.base_kv = (b.area == 3) ? 220.0 : 500.0;
    } else {
      b.base_kv = (b.zone == 1) ? 220.0 : 110.0;
      if (b.area == 3 && b.zone == 2) b.base_kv = 35.0;
    }
  }

  const std::vector<FuelType> fuel_mix = {
      FuelType::Coal, FuelType::Gas, FuelType::Hydro, FuelType::Nuclear, FuelType::Gas,
      FuelType::Oil, FuelType::Wind, FuelType::Solar, FuelType::Hydro, FuelType::Biomass};
  const std::vector<double> fuel_ef = {0.92, 0.44, 0.02, 0.01, 0.40,
                                       0.78, 0.00, 0.00, 0.02, 0.10};
  const std::vector<std::string> fuel_name = {
      "A1-Coal-Unit", "A1-Gas-Unit", "A1-Hydro-Unit", "A2-Nuclear-Slack", "A2-Gas-CCGT",
      "A2-Oil-Peaker", "A3-Wind-Plant", "A3-Solar-Plant", "A3-Hydro-Unit", "A3-Biomass-Unit"};
  for (size_t i = 0; i < sys.ac.generators.size() && i < fuel_mix.size(); ++i) {
    auto& g = sys.ac.generators[i];
    g.fuel_type = fuel_mix[i];
    g.emission_factor_tco2_mwh = fuel_ef[i];
    g.name = fuel_name[i];
  }

  auto make_ac_sgen = [](int index,
                         int bus,
                         SgenType type,
                         double p_mw,
                         double pmax_mw,
                         double co2,
                         const std::string& name) {
    StaticGenerator sg;
    sg.index = index;
    sg.bus = bus;
    sg.in_service = true;
    sg.name = name;
    sg.sgen_type = type;
    sg.p_mw = p_mw;
    sg.q_mvar = 0.0;
    sg.p_rated_mw = pmax_mw;
    sg.pmax_mw = pmax_mw;
    sg.pmin_mw = 0.0;
    sg.scaling = 1.0;
    sg.co2_emission_rate = co2;
    return sg;
  };
  sys.ac.static_generators.push_back(
      make_ac_sgen(201, 4, SgenType::PV, 24.0, 30.0, 0.0, "A1-Urban-Rooftop-PV"));
  sys.ac.static_generators.push_back(
      make_ac_sgen(202, 10, SgenType::Diesel, 18.0, 25.0, 0.78, "A2-Diesel-Microgrid"));
  sys.ac.static_generators.push_back(
      make_ac_sgen(203, 20, SgenType::FuelCell, 16.0, 20.0, 0.25, "A3-H2-FuelCell"));

  RenewableGen wind_farm;
  wind_farm.index = 301;
  wind_farm.bus = 17;
  wind_farm.in_service = true;
  wind_farm.name = "A3-Wind-Cluster";
  wind_farm.type = RenewableType::Wind;
  wind_farm.p_mw = 48.0;
  wind_farm.p_rated_mw = 60.0;
  wind_farm.q_mvar = 0.0;
  sys.ac.renewable_gens.push_back(wind_farm);

  RenewableGen solar_farm;
  solar_farm.index = 302;
  solar_farm.bus = 24;
  solar_farm.in_service = true;
  solar_farm.name = "A3-Solar-Cluster";
  solar_farm.type = RenewableType::SolarPV;
  solar_farm.p_mw = 36.0;
  solar_farm.p_rated_mw = 50.0;
  solar_farm.q_mvar = 0.0;
  sys.ac.renewable_gens.push_back(solar_farm);

  PVSystem pv_a1;
  pv_a1.index = 401;
  pv_a1.bus = 6;
  pv_a1.in_service = true;
  pv_a1.name = "A1-PV-System";
  pv_a1.p_mw = 20.0;
  pv_a1.pmax_mw = 28.0;
  pv_a1.pmin_mw = 0.0;
  pv_a1.sn_mva = 30.0;
  pv_a1.controllable = true;
  sys.ac.pv_systems.push_back(pv_a1);

  PVSystem pv_a2;
  pv_a2.index = 402;
  pv_a2.bus = 19;
  pv_a2.in_service = true;
  pv_a2.name = "A3-Coastal-PV";
  pv_a2.p_mw = 18.0;
  pv_a2.pmax_mw = 24.0;
  pv_a2.pmin_mw = 0.0;
  pv_a2.sn_mva = 25.0;
  pv_a2.controllable = true;
  sys.ac.pv_systems.push_back(pv_a2);

  Storage ac_st_charge;
  ac_st_charge.index = 501;
  ac_st_charge.bus = 8;
  ac_st_charge.in_service = true;
  ac_st_charge.name = "A1-BESS-Idle";
  ac_st_charge.p_mw = 0.0;
  ac_st_charge.p_rated_mw = 20.0;
  ac_st_charge.pmax_mw = 20.0;
  ac_st_charge.pmin_mw = -20.0;
  ac_st_charge.e_rated_mwh = 60.0;
  ac_st_charge.soc_carbon_intensity_tco2_mwh = 0.30;
  sys.ac.storage.push_back(ac_st_charge);

  Storage ac_st_discharge;
  ac_st_discharge.index = 502;
  ac_st_discharge.bus = 14;
  ac_st_discharge.in_service = true;
  ac_st_discharge.name = "A2-BESS-Discharge";
  ac_st_discharge.p_mw = 14.0;
  ac_st_discharge.p_rated_mw = 25.0;
  ac_st_discharge.pmax_mw = 25.0;
  ac_st_discharge.pmin_mw = -25.0;
  ac_st_discharge.e_rated_mwh = 80.0;
  ac_st_discharge.soc_carbon_intensity_tco2_mwh = 0.34;
  sys.ac.storage.push_back(ac_st_discharge);

  for (auto& b : sys.dc.buses) {
    b.base_kv = 320.0;
    if (b.index == 1) {
      b.area = 1;
      b.zone = 2;
      b.name = "DC-A1-Hub";
    } else if (b.index == 2) {
      b.area = 2;
      b.zone = 1;
      b.name = "DC-A2-Hub";
    } else if (b.index == 3) {
      b.area = 2;
      b.zone = 2;
      b.name = "DC-A2-East";
    } else {
      b.area = 3;
      b.zone = 1;
      b.name = "DC-A3-Hub";
    }
  }

  auto make_dc_sgen = [](int index,
                         int bus,
                         SgenType type,
                         double p_mw,
                         double pmax_mw,
                         double co2,
                         const std::string& name) {
    StaticGenerator sg;
    sg.index = index;
    sg.bus = bus;
    sg.in_service = true;
    sg.name = name;
    sg.sgen_type = type;
    sg.p_mw = p_mw;
    sg.q_mvar = 0.0;
    sg.p_rated_mw = pmax_mw;
    sg.pmax_mw = pmax_mw;
    sg.pmin_mw = 0.0;
    sg.scaling = 1.0;
    sg.co2_emission_rate = co2;
    return sg;
  };
  sys.dc.static_generators.push_back(
      make_dc_sgen(601, 1, SgenType::PV, 34.0, 40.0, 0.0, "DC-Solar-Hub"));
  sys.dc.static_generators.push_back(
      make_dc_sgen(602, 3, SgenType::Wind, 26.0, 35.0, 0.0, "DC-Wind-Hub"));

  DCLoad dc_ld_1;
  dc_ld_1.index = 701;
  dc_ld_1.bus = 2;
  dc_ld_1.in_service = true;
  dc_ld_1.name = "A2-Electrolyzer";
  dc_ld_1.type = "industrial";
  dc_ld_1.p_mw = 26.0;
  dc_ld_1.scaling = 1.0;
  sys.dc.loads.push_back(dc_ld_1);

  DCLoad dc_ld_2;
  dc_ld_2.index = 702;
  dc_ld_2.bus = 4;
  dc_ld_2.in_service = true;
  dc_ld_2.name = "A3-DataCenter";
  dc_ld_2.type = "commercial";
  dc_ld_2.p_mw = 32.0;
  dc_ld_2.scaling = 1.0;
  sys.dc.loads.push_back(dc_ld_2);

  Storage dc_st_charge;
  dc_st_charge.index = 801;
  dc_st_charge.bus = 2;
  dc_st_charge.in_service = true;
  dc_st_charge.name = "DC-BESS-Idle";
  dc_st_charge.p_mw = 0.0;
  dc_st_charge.p_rated_mw = 15.0;
  dc_st_charge.pmax_mw = 15.0;
  dc_st_charge.pmin_mw = -15.0;
  dc_st_charge.e_rated_mwh = 40.0;
  dc_st_charge.soc_carbon_intensity_tco2_mwh = 0.26;
  sys.dc.storage.push_back(dc_st_charge);

  Storage dc_st_discharge;
  dc_st_discharge.index = 802;
  dc_st_discharge.bus = 4;
  dc_st_discharge.in_service = true;
  dc_st_discharge.name = "DC-BESS-Discharge";
  dc_st_discharge.p_mw = 10.0;
  dc_st_discharge.p_rated_mw = 18.0;
  dc_st_discharge.pmax_mw = 18.0;
  dc_st_discharge.pmin_mw = -18.0;
  dc_st_discharge.e_rated_mwh = 50.0;
  dc_st_discharge.soc_carbon_intensity_tco2_mwh = 0.38;
  sys.dc.storage.push_back(dc_st_discharge);

  DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 1;
  dcdc.bus_out = 3;
  dcdc.in_service = true;
  dcdc.name = "DCDC-A1-to-A2";
  dcdc.control_mode = DCDCControlMode::Power;
  dcdc.p_ref_mw = 14.0;
  dcdc.eta = 0.975;
  dcdc.pmax_mw = 40.0;
  dcdc.pmin_mw = -40.0;
  sys.dc.dcdc_converters.push_back(dcdc);

  for (auto& vsc : sys.vsc_converters) {
    vsc.eta = 0.985;
    vsc.loss_percent = 0.8;
  }
  return sys;
}

HybridPowerSystem build_case2000_acdc() {
  const std::filesystem::path case2000_path = data_file_path("case_ACTIVSg2000.m");
  if (!std::filesystem::exists(case2000_path)) {
    HybridPowerSystem sys = build_case300_acdc();
    sys.name = "case2000 AC/DC (surrogate from case300)";
    return sys;
  }

  HybridPowerSystem sys = parse_matpower(case2000_path.string());
  sys.name = "case2000 AC/DC";
  attach_case2000_style_mtdc(sys);
  return sys;
}

// Three-bus FLISR benchmark for the Level-1 cyber-physical reliability model.
// A feeder fault isolates both loads.  With automation available, the open tie
// is closed after the automatic switching delay and the controllable storage
// covers the tie's thermal shortfall.  With automation unavailable, the crew
// still closes the tie manually during the repair stage, but the storage
// cannot be dispatched (frozen), so the tie limit leaves residual shed — the
// case therefore separates the restoration-delay and control-loss increments.
HybridPowerSystem build_cyber_physical_reliability_demo() {
  HybridPowerSystem sys;
  sys.name = "Cyber-Physical Reliability Demo";
  sys.base_mva = sys.ac.base_mva = 10.0;

  ACBus source = make_ac_bus(1, BusType::SLACK, 0.0, 0.0, 1.0, 0.0);
  source.name = "Primary Substation";
  source.base_kv = 10.0;
  source.longitude = 0.0;
  source.latitude = 0.0;
  ACBus load_a = make_ac_bus(2, BusType::PQ, 0.0, 0.0, 1.0, 0.0);
  load_a.name = "Automated Feeder A";
  load_a.base_kv = 10.0;
  load_a.longitude = 1.0;
  load_a.latitude = 0.0;
  ACBus load_b = make_ac_bus(3, BusType::PQ, 0.0, 0.0, 1.0, 0.0);
  load_b.name = "Automated Feeder B";
  load_b.base_kv = 10.0;
  load_b.longitude = 2.0;
  load_b.latitude = 0.0;
  sys.ac.buses = {source, load_a, load_b};

  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  grid.name = "Utility Grid";
  grid.vm_pu = 1.0;
  grid.va_deg = 0.0;
  grid.s_sc_max_mva = 10.0;
  sys.ac.external_grids = {grid};

  Load customer_a;
  customer_a.index = 1;
  customer_a.bus = 2;
  customer_a.in_service = true;
  customer_a.name = "Commercial Load";
  customer_a.p_mw = 1.0;
  customer_a.q_mvar = 0.2;
  customer_a.n_customers = 100;
  Load customer_b = customer_a;
  customer_b.index = 2;
  customer_b.bus = 3;
  customer_b.name = "Residential Load";
  customer_b.n_customers = 200;
  sys.ac.loads = {customer_a, customer_b};

  ACBranch feeder = make_ac_branch(1, 1, 2, 0.01, 0.10, 0.0, 1.0);
  feeder.name = "Primary Feeder";
  feeder.rate_a_mva = 10.0;
  feeder.failure_rate = 1.0;
  feeder.mttr_hr = 4.0;
  ACBranch section = make_ac_branch(2, 2, 3, 0.01, 0.10, 0.0, 1.0);
  section.name = "Feeder Section";
  section.rate_a_mva = 10.0;
  section.failure_rate = 1e-6;
  section.mttr_hr = 4.0;
  ACBranch tie = make_ac_branch(3, 1, 3, 0.01, 0.15, 0.0, 1.0);
  tie.name = "Normally Open FLISR Tie";
  // Deliberately below the 2.0 MW demand: tie closure alone leaves 0.8 MW
  // unserved, which only the (remotely dispatched) storage can cover.
  tie.rate_a_mva = 1.2;
  tie.in_service = false;
  tie.failure_rate = 0.0;
  tie.mttr_hr = 4.0;
  sys.ac.branches = {feeder, section, tie};

  // Grid-following battery: dispatchable through the DMS when communications
  // are up, frozen when automation is lost.  Not grid-forming, so it cannot
  // energize the de-energized island during the switching stage.
  Storage battery;
  battery.index = 1;
  battery.bus = 3;
  battery.in_service = true;
  battery.name = "Feeder Battery";
  battery.controllable = true;
  battery.grid_forming = false;
  battery.pmax_mw = 1.0;
  battery.p_rated_mw = 1.0;
  battery.e_rated_mwh = 10.0;
  battery.e_mwh = 10.0;
  battery.soc_min = 0.0;
  battery.eta_discharge = 1.0;
  sys.ac.storage = {battery};

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// Distribution system with 3 microgrids and diverse DERs for lifecycle
// simulation.  Based on case33bw + PV, wind, storage, diesel gens.
// ═══════════════════════════════════════════════════════════════════════
HybridPowerSystem build_dist33_microgrid_der() {
  // Start from IEEE 33-bus distribution system with a 2-terminal DC overlay
  HybridPowerSystem sys = parse_case_with_overlay("case33bw.m", "dist33 MG+DER Lifecycle", 1, 18);

  // --- Set emission factors on existing generators ---
  for (auto& g : sys.ac.generators) {
    g.emission_factor_tco2_mwh = 0.50;  // grid substation (mixed)
    g.fuel_type = FuelType::Gas;
  }

  // === Microgrid 1 (buses 3-8): residential + rooftop PV + small BESS ===
  {
    PVSystem pv1;
    pv1.index = 101;
    pv1.bus = 5;
    pv1.in_service = true;
    pv1.name = "MG1-Rooftop-PV";
    pv1.p_mw = 0.3;
    pv1.pmax_mw = 0.5;
    pv1.pmin_mw = 0.0;
    pv1.sn_mva = 0.55;
    pv1.controllable = true;
    pv1.profile_id = 2;  // solar profile
    sys.ac.pv_systems.push_back(pv1);

    PVSystem pv2;
    pv2.index = 102;
    pv2.bus = 7;
    pv2.in_service = true;
    pv2.name = "MG1-Community-PV";
    pv2.p_mw = 0.5;
    pv2.pmax_mw = 0.8;
    pv2.pmin_mw = 0.0;
    pv2.sn_mva = 0.9;
    pv2.controllable = true;
    pv2.profile_id = 2;
    sys.ac.pv_systems.push_back(pv2);

    Storage bess1;
    bess1.index = 201;
    bess1.bus = 6;
    bess1.in_service = true;
    bess1.name = "MG1-BESS";
    bess1.p_mw = 0.0;
    bess1.p_rated_mw = 0.3;
    bess1.pmax_mw = 0.3;
    bess1.pmin_mw = -0.3;
    bess1.e_rated_mwh = 1.2;
    bess1.soc_init = 0.5;
    bess1.soc_min = 0.1;
    bess1.soc_max = 0.9;
    bess1.eta_charge = 0.95;
    bess1.eta_discharge = 0.95;
    bess1.self_discharge_pct = 0.001;
    bess1.max_cycles = 9000;
    bess1.current_cycles = 0;
    bess1.soh = 1.0;
    bess1.l_calendar_yr = 15.0;
    bess1.eol_percent = 0.8;
    bess1.replacement_cost = 120.0;  // $/kWh
    bess1.type = "Li-ion";
    bess1.soc_carbon_intensity_tco2_mwh = 0.30;
    bess1.daily_cycle_limit = 2.2;
    bess1.charge_bid_price = -2.0;
    bess1.discharge_bid_price = 1.0;
    sys.ac.storage.push_back(bess1);
  }

  // === Microgrid 2 (buses 12-17): commercial + wind + larger BESS ===
  {
    RenewableGen wind1;
    wind1.index = 301;
    wind1.bus = 14;
    wind1.in_service = true;
    wind1.name = "MG2-Wind-Turbine";
    wind1.type = RenewableType::Wind;
    wind1.p_mw = 0.4;
    wind1.p_rated_mw = 1.0;
    wind1.capacity_factor = 0.28;
    wind1.curtailable = true;
    wind1.cost_curtail_mwh = 30.0;
    wind1.profile_id = 1;  // wind profile
    sys.ac.renewable_gens.push_back(wind1);

    PVSystem pv3;
    pv3.index = 103;
    pv3.bus = 16;
    pv3.in_service = true;
    pv3.name = "MG2-Commercial-PV";
    pv3.p_mw = 0.6;
    pv3.pmax_mw = 1.0;
    pv3.pmin_mw = 0.0;
    pv3.sn_mva = 1.1;
    pv3.controllable = true;
    pv3.profile_id = 2;
    sys.ac.pv_systems.push_back(pv3);

    Storage bess2;
    bess2.index = 202;
    bess2.bus = 15;
    bess2.in_service = true;
    bess2.name = "MG2-BESS";
    bess2.p_mw = 0.0;
    bess2.p_rated_mw = 0.5;
    bess2.pmax_mw = 0.5;
    bess2.pmin_mw = -0.5;
    bess2.e_rated_mwh = 2.0;
    bess2.soc_init = 0.5;
    bess2.soc_min = 0.1;
    bess2.soc_max = 0.9;
    bess2.eta_charge = 0.94;
    bess2.eta_discharge = 0.94;
    bess2.self_discharge_pct = 0.001;
    bess2.max_cycles = 6000;
    bess2.current_cycles = 0;
    bess2.soh = 1.0;
    bess2.l_calendar_yr = 12.0;
    bess2.eol_percent = 0.8;
    bess2.replacement_cost = 220.0;
    bess2.type = "Li-ion";
    bess2.soc_carbon_intensity_tco2_mwh = 0.28;
    bess2.daily_cycle_limit = 1.4;
    bess2.charge_bid_price = 1.5;
    bess2.discharge_bid_price = 9.0;
    sys.ac.storage.push_back(bess2);

    // Diesel backup generator
    StaticGenerator diesel1;
    diesel1.index = 401;
    diesel1.bus = 13;
    diesel1.in_service = true;
    diesel1.name = "MG2-Diesel-Backup";
    diesel1.sgen_type = SgenType::Diesel;
    diesel1.p_mw = 0.2;
    diesel1.q_mvar = 0.0;
    diesel1.p_rated_mw = 0.5;
    diesel1.pmax_mw = 0.5;
    diesel1.pmin_mw = 0.0;
    diesel1.scaling = 1.0;
    diesel1.co2_emission_rate = 0.70;  // diesel emission factor
    sys.ac.static_generators.push_back(diesel1);
  }

  // === Microgrid 3 (buses 25-32): industrial + wind + PV + BESS + gas gen ===
  {
    RenewableGen wind2;
    wind2.index = 302;
    wind2.bus = 27;
    wind2.in_service = true;
    wind2.name = "MG3-Wind-Farm";
    wind2.type = RenewableType::Wind;
    wind2.p_mw = 0.8;
    wind2.p_rated_mw = 2.0;
    wind2.capacity_factor = 0.32;
    wind2.curtailable = true;
    wind2.cost_curtail_mwh = 25.0;
    wind2.profile_id = 1;
    sys.ac.renewable_gens.push_back(wind2);

    PVSystem pv4;
    pv4.index = 104;
    pv4.bus = 30;
    pv4.in_service = true;
    pv4.name = "MG3-Industrial-PV";
    pv4.p_mw = 0.8;
    pv4.pmax_mw = 1.5;
    pv4.pmin_mw = 0.0;
    pv4.sn_mva = 1.6;
    pv4.controllable = true;
    pv4.profile_id = 2;
    sys.ac.pv_systems.push_back(pv4);

    Storage bess3;
    bess3.index = 203;
    bess3.bus = 28;
    bess3.in_service = true;
    bess3.name = "MG3-BESS-Large";
    bess3.p_mw = 0.0;
    bess3.p_rated_mw = 1.0;
    bess3.pmax_mw = 1.0;
    bess3.pmin_mw = -1.0;
    bess3.e_rated_mwh = 4.0;
    bess3.soc_init = 0.5;
    bess3.soc_min = 0.1;
    bess3.soc_max = 0.9;
    bess3.eta_charge = 0.93;
    bess3.eta_discharge = 0.93;
    bess3.self_discharge_pct = 0.002;
    bess3.max_cycles = 4000;
    bess3.current_cycles = 0;
    bess3.soh = 1.0;
    bess3.l_calendar_yr = 10.0;
    bess3.eol_percent = 0.8;
    bess3.replacement_cost = 200.0;
    bess3.type = "Li-ion";
    bess3.soc_carbon_intensity_tco2_mwh = 0.25;
    bess3.daily_cycle_limit = 1.6;
    bess3.charge_bid_price = 1.0;
    bess3.discharge_bid_price = 8.0;
    sys.ac.storage.push_back(bess3);

    // Gas turbine (peaker)
    StaticGenerator gas1;
    gas1.index = 402;
    gas1.bus = 26;
    gas1.in_service = true;
    gas1.name = "MG3-Gas-Turbine";
    gas1.sgen_type = SgenType::CHP;
    gas1.p_mw = 0.3;
    gas1.q_mvar = 0.0;
    gas1.p_rated_mw = 1.0;
    gas1.pmax_mw = 1.0;
    gas1.pmin_mw = 0.0;
    gas1.scaling = 1.0;
    gas1.co2_emission_rate = 0.45;  // natural gas emission factor
    sys.ac.static_generators.push_back(gas1);
  }

  // === Demand-response portfolio ===
  // Convert part of three existing bus loads into explicit FlexibleLoad
  // resources. Subtracting the same P/Q share from the source bus preserves
  // the original case demand when demand response is disabled.
  {
    auto add_flexible_share = [&](int index, int bus_index, double share,
                                  double up_fraction, double down_fraction,
                                  double availability_pct,
                                  const std::string& name,
                                  const std::string& control_area,
                                  LoadPriority priority) {
      auto bus_it = std::find_if(
          sys.ac.buses.begin(), sys.ac.buses.end(),
          [&](const ACBus& bus) { return bus.index == bus_index; });
      if (bus_it == sys.ac.buses.end() || !bus_it->in_service) return;
      const double bounded_share = std::clamp(share, 0.0, 1.0);
      const double p_flexible = std::max(0.0, bus_it->pd_mw) * bounded_share;
      const double q_flexible = std::max(0.0, bus_it->qd_mvar) * bounded_share;
      if (p_flexible <= 1e-9) return;

      bus_it->pd_mw -= p_flexible;
      bus_it->qd_mvar -= q_flexible;

      FlexibleLoad load;
      load.index = index;
      load.bus = bus_index;
      load.name = name;
      load.in_service = true;
      load.p_mw = p_flexible;
      load.q_mvar = q_flexible;
      load.flex_up_mw = p_flexible * std::max(0.0, up_fraction);
      load.flex_down_mw = p_flexible * std::max(0.0, down_fraction);
      load.flex_duration_h = 4.0;
      load.response_time_s = control_area == "MG3" ? 300.0 : 60.0;
      load.ramp_rate_mw_min = std::max(load.flex_up_mw, load.flex_down_mw) / 15.0;
      load.availability_pct = std::clamp(availability_pct, 0.0, 100.0);
      load.controllable = true;
      load.priority = priority;
      load.control_area = control_area;
      sys.ac.flexible_loads.push_back(std::move(load));
    };

    add_flexible_share(501, 7, 0.45, 0.25, 0.30, 80.0,
                       "MG1-Residential-Thermostatic-DR", "MG1",
                       LoadPriority::Medium);
    add_flexible_share(502, 15, 0.50, 0.30, 0.35, 90.0,
                       "MG2-Commercial-HVAC-DR", "MG2",
                       LoadPriority::Medium);
    add_flexible_share(503, 30, 0.35, 0.20, 0.25, 85.0,
                       "MG3-Industrial-Process-DR", "MG3",
                       LoadPriority::High);
  }

  // === Automated tie and islanding switches for fault restoration studies ===
  {
    int sw_idx = 0;
    for (const auto& sw : sys.ac.switches) sw_idx = std::max(sw_idx, sw.index);

    Switch tie_mg1_mg2;
    tie_mg1_mg2.index = ++sw_idx;
    tie_mg1_mg2.name = "SW-Tie-8-21";
    tie_mg1_mg2.bus_from = 8;
    tie_mg1_mg2.bus_to = 21;
    tie_mg1_mg2.in_service = true;
    tie_mg1_mg2.closed = false;
    tie_mg1_mg2.switch_type = SwitchType::LoadBreakSwitch;
    tie_mg1_mg2.i_rated_ka = 0.4;
    tie_mg1_mg2.is_automated = true;
    tie_mg1_mg2.is_remote = true;
    tie_mg1_mg2.t_operation_s = 0.5;
    sys.ac.switches.push_back(tie_mg1_mg2);

    Switch tie_mg2_mg3;
    tie_mg2_mg3.index = ++sw_idx;
    tie_mg2_mg3.name = "SW-Tie-25-29";
    tie_mg2_mg3.bus_from = 25;
    tie_mg2_mg3.bus_to = 29;
    tie_mg2_mg3.in_service = true;
    tie_mg2_mg3.closed = false;
    tie_mg2_mg3.switch_type = SwitchType::LoadBreakSwitch;
    tie_mg2_mg3.i_rated_ka = 0.4;
    tie_mg2_mg3.is_automated = true;
    tie_mg2_mg3.is_remote = true;
    tie_mg2_mg3.t_operation_s = 0.5;
    sys.ac.switches.push_back(tie_mg2_mg3);

    Switch tie_mg2_lateral;
    tie_mg2_lateral.index = ++sw_idx;
    tie_mg2_lateral.name = "SW-Tie-12-22";
    tie_mg2_lateral.bus_from = 12;
    tie_mg2_lateral.bus_to = 22;
    tie_mg2_lateral.in_service = true;
    tie_mg2_lateral.closed = false;
    tie_mg2_lateral.switch_type = SwitchType::LoadBreakSwitch;
    tie_mg2_lateral.i_rated_ka = 0.4;
    tie_mg2_lateral.is_automated = true;
    tie_mg2_lateral.is_remote = true;
    tie_mg2_lateral.t_operation_s = 0.5;
    sys.ac.switches.push_back(tie_mg2_lateral);

    Switch sec_mg1_entry;
    sec_mg1_entry.index = ++sw_idx;
    sec_mg1_entry.name = "SW-SEC-MG1-Entry";
    sec_mg1_entry.bus_from = 2;
    sec_mg1_entry.bus_to = 3;
    sec_mg1_entry.in_service = true;
    sec_mg1_entry.closed = true;
    sec_mg1_entry.switch_type = SwitchType::Sectionalizer;
    sec_mg1_entry.i_rated_ka = 0.6;
    sec_mg1_entry.is_automated = true;
    sec_mg1_entry.is_remote = true;
    sec_mg1_entry.t_operation_s = 0.2;
    sys.ac.switches.push_back(sec_mg1_entry);

    Switch sec_mg2_entry;
    sec_mg2_entry.index = ++sw_idx;
    sec_mg2_entry.name = "SW-SEC-MG2-Entry";
    sec_mg2_entry.bus_from = 11;
    sec_mg2_entry.bus_to = 12;
    sec_mg2_entry.in_service = true;
    sec_mg2_entry.closed = true;
    sec_mg2_entry.switch_type = SwitchType::Sectionalizer;
    sec_mg2_entry.i_rated_ka = 0.6;
    sec_mg2_entry.is_automated = true;
    sec_mg2_entry.is_remote = true;
    sec_mg2_entry.t_operation_s = 0.2;
    sys.ac.switches.push_back(sec_mg2_entry);

    Switch sec_mg3_entry;
    sec_mg3_entry.index = ++sw_idx;
    sec_mg3_entry.name = "SW-SEC-MG3-Entry";
    sec_mg3_entry.bus_from = 24;
    sec_mg3_entry.bus_to = 25;
    sec_mg3_entry.in_service = true;
    sec_mg3_entry.closed = true;
    sec_mg3_entry.switch_type = SwitchType::Sectionalizer;
    sec_mg3_entry.i_rated_ka = 0.6;
    sec_mg3_entry.is_automated = true;
    sec_mg3_entry.is_remote = true;
    sec_mg3_entry.t_operation_s = 0.2;
    sys.ac.switches.push_back(sec_mg3_entry);

    Switch mg1_island;
    mg1_island.index = ++sw_idx;
    mg1_island.name = "LBS-MG1-Island";
    mg1_island.bus_from = 2;
    mg1_island.bus_to = 3;
    mg1_island.in_service = true;
    mg1_island.closed = true;
    mg1_island.switch_type = SwitchType::LoadBreakSwitch;
    mg1_island.i_rated_ka = 0.6;
    mg1_island.is_automated = true;
    mg1_island.is_remote = true;
    mg1_island.t_operation_s = 0.2;
    sys.ac.switches.push_back(mg1_island);
  }

  // ── Reliability & resilience enrichment (non-electrical fields only) ──────
  // Feeder-position layered failure statistics, DER outage parameters and
  // per-bus customer/importance data, so the reliability (SAIDI/SAIFI/ASUI)
  // and resilience (priority-weighted restoration) modules produce
  // differentiated results out of the box.  No electrical parameter changes.
  {
    for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
      auto& br = sys.ac.branches[i];
      if (!br.in_service) {        // 常开联络线：故障率最低
        br.failure_rate = 0.02;    // occ/yr
        br.mttr_hr = 4.0;
      } else if (i < 18) {         // 主干段（辐射主链）
        br.failure_rate = 0.15;
        br.mttr_hr = 8.0;
      } else {                     // 分支/末梢段
        br.failure_rate = 0.08;
        br.mttr_hr = 5.0;
      }
    }
    // 柴油/燃气静态发电机：FOR ≈ 3%/2%（mtbf = mttr·(1−FOR)/FOR）
    for (auto& sg : sys.ac.static_generators) {
      const bool is_diesel = sg.name.find("Diesel") != std::string::npos ||
                             sg.name.find("diesel") != std::string::npos;
      const double fo = is_diesel ? 0.03 : 0.02;
      sg.mttr_hours = is_diesel ? 36.0 : 48.0;
      sg.mtbf_hours = sg.mttr_hours * (1.0 - fo) / fo;
    }
    // 3 台 BESS
    for (auto& st : sys.ac.storage) {
      st.forced_outage_rate = 0.01;
      st.mttr_hr = 12.0;
    }
    // 用户数与重要度：按母线负荷规模分层，供 SAIDI/SAIFI 与恢复权重使用
    for (auto& bus : sys.ac.buses) {
      if (bus.pd_mw <= 0.0) continue;
      bus.n_customers = std::max(1, static_cast<int>(std::lround(bus.pd_mw * 10.0)));
      bus.importance = bus.pd_mw >= 0.2 ? 2.0 : 1.0;
    }
  }

  return sys;
}

// ════════════════════════════════════════════════════════════════════════════════
// Comprehensive Hybrid AC/DC Test Case
// ════════════════════════════════════════════════════════════════════════════════
// This case tests all components and functionalities of the hybrid AC/DC suite:
// - External Grid (110kV transmission connection)
// - OLTC Transformer with shunts
// - Three MV distribution feeders (20kV) with switches and breakers
// - 3 VSC converters connecting AC and DC networks
// - 2 Microgrids with DERs
// - Multiple PV systems and Wind farms
// - EV charging stations
// - DC network with loads, storage, and PV arrays
// ════════════════════════════════════════════════════════════════════════════════
HybridPowerSystem build_comprehensive_hybrid_acdc() {
  HybridPowerSystem sys;
  sys.name = "Comprehensive Hybrid AC/DC Test System";
  sys.base_mva = 100.0;

  // ═══════════════════════════════════════════════════════════════════════════
  // AC NETWORK TOPOLOGY
  // ═══════════════════════════════════════════════════════════════════════════
  // Bus 1: External grid connection (110kV, slack)
  // Bus 2: HV/MV substation primary (110kV)
  // Bus 3: MV busbar (20kV) - main distribution
  // Feeders:
  //   Feeder 1 (Industrial): Buses 4-8
  //   Feeder 2 (Commercial): Buses 9-14
  //   Feeder 3 (Residential): Buses 15-21
  // Microgrid 1 (Industrial): Buses 6-8
  // Microgrid 2 (Residential): Buses 18-21

  const double base_kv_hv = 110.0;
  const double base_kv_mv = 20.0;

  // GIS Layout: Star topology with MV busbar at center
  // - HV buses at north (top)
  // - Feeder 1 (Industrial) goes east
  // - Feeder 2 (Commercial) goes north-northeast
  // - Feeder 3 (Residential) goes south-southwest
  const double center_lat = 40.7100;
  const double center_lon = -74.0100;

  // --- AC Buses ---
  int bus_idx = 0;
  
  // HV buses (north of center)
  ACBus bus1; bus1.index = ++bus_idx; bus1.name = "ExtGrid-110kV";
  bus1.bus_type = BusType::SLACK; bus1.base_kv = base_kv_hv;
  bus1.vm_pu = 1.02; bus1.va_deg = 0.0; bus1.vmin_pu = 0.95; bus1.vmax_pu = 1.05;
  bus1.latitude = center_lat + 0.012; bus1.longitude = center_lon;
  sys.ac.buses.push_back(bus1);

  ACBus bus2; bus2.index = ++bus_idx; bus2.name = "Substation-HV";
  bus2.bus_type = BusType::PQ; bus2.base_kv = base_kv_hv;
  bus2.vm_pu = 1.0; bus2.vmin_pu = 0.95; bus2.vmax_pu = 1.05;
  bus2.latitude = center_lat + 0.006; bus2.longitude = center_lon;
  sys.ac.buses.push_back(bus2);

  // MV main busbar (center)
  ACBus bus3; bus3.index = ++bus_idx; bus3.name = "MV-Busbar-20kV";
  bus3.bus_type = BusType::PQ; bus3.base_kv = base_kv_mv;
  bus3.vm_pu = 1.0; bus3.vmin_pu = 0.95; bus3.vmax_pu = 1.05;
  bus3.latitude = center_lat; bus3.longitude = center_lon;
  sys.ac.buses.push_back(bus3);

  // Feeder 1: Industrial (buses 4-8) - goes EAST
  std::vector<std::string> f1_names = {"F1-Ind-Start", "F1-Ind-Mid1", "F1-MG1-PCC", "F1-MG1-Gen", "F1-MG1-Load"};
  std::vector<double> f1_lats = {center_lat - 0.002, center_lat - 0.004, center_lat - 0.006, center_lat - 0.007, center_lat - 0.008};
  std::vector<double> f1_lons = {center_lon + 0.006, center_lon + 0.012, center_lon + 0.018, center_lon + 0.021, center_lon + 0.024};
  for (size_t i = 0; i < f1_names.size(); ++i) {
    ACBus b; b.index = ++bus_idx; b.name = f1_names[i];
    b.bus_type = BusType::PQ; b.base_kv = base_kv_mv;
    b.vm_pu = 1.0; b.vmin_pu = 0.95; b.vmax_pu = 1.05;
    b.latitude = f1_lats[i]; b.longitude = f1_lons[i];
    sys.ac.buses.push_back(b);
  }

  // Feeder 2: Commercial (buses 9-14) - goes NORTH-NORTHEAST
  std::vector<std::string> f2_names = {"F2-Com-Start", "F2-Com-Mid1", "F2-Com-Mid2", 
                                        "F2-Com-EV", "F2-Com-Office", "F2-Com-End"};
  std::vector<double> f2_lats = {center_lat + 0.004, center_lat + 0.008, center_lat + 0.012, 
                                  center_lat + 0.016, center_lat + 0.020, center_lat + 0.024};
  std::vector<double> f2_lons = {center_lon + 0.003, center_lon + 0.006, center_lon + 0.008, 
                                  center_lon + 0.010, center_lon + 0.011, center_lon + 0.012};
  for (size_t i = 0; i < f2_names.size(); ++i) {
    ACBus b; b.index = ++bus_idx; b.name = f2_names[i];
    b.bus_type = BusType::PQ; b.base_kv = base_kv_mv;
    b.vm_pu = 1.0; b.vmin_pu = 0.95; b.vmax_pu = 1.05;
    b.latitude = f2_lats[i]; b.longitude = f2_lons[i];
    sys.ac.buses.push_back(b);
  }

  // Feeder 3: Residential (buses 15-21) - goes SOUTH-SOUTHWEST
  std::vector<std::string> f3_names = {"F3-Res-Start", "F3-Res-Mid1", "F3-Res-Mid2", 
                                        "F3-MG2-PCC", "F3-MG2-PV", "F3-MG2-BESS", "F3-MG2-Load"};
  std::vector<double> f3_lats = {center_lat - 0.004, center_lat - 0.008, center_lat - 0.012, 
                                  center_lat - 0.016, center_lat - 0.018, center_lat - 0.020, center_lat - 0.024};
  std::vector<double> f3_lons = {center_lon - 0.004, center_lon - 0.008, center_lon - 0.012, 
                                  center_lon - 0.016, center_lon - 0.018, center_lon - 0.020, center_lon - 0.024};
  for (size_t i = 0; i < f3_names.size(); ++i) {
    ACBus b; b.index = ++bus_idx; b.name = f3_names[i];
    b.bus_type = BusType::PQ; b.base_kv = base_kv_mv;
    b.vm_pu = 1.0; b.vmin_pu = 0.95; b.vmax_pu = 1.05;
    b.latitude = f3_lats[i]; b.longitude = f3_lons[i];
    sys.ac.buses.push_back(b);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // EXTERNAL GRID
  // ═══════════════════════════════════════════════════════════════════════════
  ExternalGrid ext_grid;
  ext_grid.index = 1;
  ext_grid.name = "Main-Grid-Connection";
  ext_grid.bus = 1;
  ext_grid.in_service = true;
  ext_grid.vm_pu = 1.02;
  ext_grid.va_deg = 0.0;
  ext_grid.s_sc_max_mva = 2000.0;  // 2000 MVA short-circuit capacity
  ext_grid.s_sc_min_mva = 1500.0;
  ext_grid.rx_max = 0.1;
  ext_grid.rx_min = 0.1;
  ext_grid.vn_kv = base_kv_hv;
  ext_grid.controllable = true;
  ext_grid.emission_factor_tco2_mwh = 0.50;
  sys.ac.external_grids.push_back(ext_grid);

  // Generator at slack bus
  Generator gen_slack;
  gen_slack.index = 1;
  gen_slack.bus = 1;
  gen_slack.name = "External-Grid-Gen";
  gen_slack.in_service = true;
  gen_slack.pg_mw = 0.0;
  gen_slack.pmax_mw = 100.0;
  gen_slack.pmin_mw = -50.0;
  gen_slack.qmax_mvar = 50.0;
  gen_slack.qmin_mvar = -50.0;
  gen_slack.vg_pu = 1.02;
  gen_slack.is_slack = true;
  gen_slack.emission_factor_tco2_mwh = 0.45;
  gen_slack.fuel_type = FuelType::Gas;
  sys.ac.generators.push_back(gen_slack);

  // ═══════════════════════════════════════════════════════════════════════════
  // OLTC TRANSFORMER (110/20 kV)
  // ═══════════════════════════════════════════════════════════════════════════
  Transformer2W oltc;
  oltc.index = 1;
  oltc.name = "Main-OLTC-110/20kV";
  oltc.hv_bus = 2;
  oltc.lv_bus = 3;
  oltc.in_service = true;
  oltc.sn_mva = 40.0;
  oltc.vn_hv_kv = base_kv_hv;
  oltc.vn_lv_kv = base_kv_mv;
  oltc.vk_percent = 12.0;
  oltc.vkr_percent = 0.5;
  oltc.pfe_kw = 30.0;
  oltc.i0_percent = 0.1;
  oltc.tap_side = 0;  // HV side
  oltc.tap_pos = 0;
  oltc.tap_min = -8;
  oltc.tap_max = 8;
  oltc.tap_neutral = 0;
  oltc.tap_step_percent = 1.25;
  oltc.vector_group = "Dyn11";
  sys.ac.transformers_2w.push_back(oltc);

  // AC branch for OLTC (approximation for power flow)
  ACBranch br_oltc;
  br_oltc.index = 1;
  br_oltc.from_bus = 2; br_oltc.to_bus = 3;
  br_oltc.r_pu = 0.0005; br_oltc.x_pu = 0.012;
  br_oltc.b_pu = 0.0;
  br_oltc.rate_a_mva = 40.0;
  br_oltc.tap = 1.0;
  br_oltc.name = "OLTC-Branch";
  sys.ac.branches.push_back(br_oltc);

  // ═══════════════════════════════════════════════════════════════════════════
  // SHUNTS (Reactive Power Compensation)
  // ═══════════════════════════════════════════════════════════════════════════
  Shunt shunt1;
  shunt1.index = 1;
  shunt1.bus = 3;
  shunt1.name = "Capacitor-Bank-MV";
  shunt1.in_service = true;
  shunt1.gs_mw = 0.0;
  shunt1.bs_mvar = 5.0;  // 5 MVAr capacitor bank
  shunt1.switchable = true;
  shunt1.n_steps = 5;
  shunt1.current_step = 3;
  shunt1.bs_per_step = 1.0;
  sys.ac.shunts.push_back(shunt1);

  Shunt shunt2;
  shunt2.index = 2;
  shunt2.bus = 10;  // Commercial feeder
  shunt2.name = "Capacitor-Bank-Commercial";
  shunt2.in_service = true;
  shunt2.gs_mw = 0.0;
  shunt2.bs_mvar = 2.0;
  shunt2.switchable = true;
  shunt2.n_steps = 4;
  shunt2.current_step = 2;
  shunt2.bs_per_step = 0.5;
  sys.ac.shunts.push_back(shunt2);

  // ═══════════════════════════════════════════════════════════════════════════
  // HV LINE (External grid to substation)
  // ═══════════════════════════════════════════════════════════════════════════
  ACBranch br_hv;
  br_hv.index = 2;
  br_hv.from_bus = 1; br_hv.to_bus = 2;
  br_hv.name = "HV-Line-110kV";
  br_hv.r_pu = 0.001; br_hv.x_pu = 0.01; br_hv.b_pu = 0.02;
  br_hv.rate_a_mva = 100.0;
  sys.ac.branches.push_back(br_hv);

  // ═══════════════════════════════════════════════════════════════════════════
  // FEEDER BRANCHES WITH CIRCUIT BREAKERS AND SWITCHES
  // ═══════════════════════════════════════════════════════════════════════════
  int br_idx = 2;
  int cb_idx = 0;
  int sw_idx = 0;

  // --- Feeder 1 (Industrial) ---
  // CB at feeder head
  CircuitBreaker cb_f1;
  cb_f1.index = ++cb_idx;
  cb_f1.name = "CB-Feeder1-Head";
  cb_f1.bus_from = 3; cb_f1.bus_to = 4;
  cb_f1.in_service = true;
  cb_f1.closed = true;
  cb_f1.breaker_type = BreakerType::CB;
  cb_f1.rated_voltage_kv = base_kv_mv;
  cb_f1.i_rated_ka = 1.25;
  cb_f1.i_breaking_ka = 25.0;
  cb_f1.element_type = "l";
  sys.ac.circuit_breakers.push_back(cb_f1);

  // Feeder 1 branches
  std::vector<std::pair<int,int>> f1_branches = {{3,4}, {4,5}, {5,6}, {6,7}, {7,8}};
  for (const auto& [from, to] : f1_branches) {
    ACBranch br;
    br.index = ++br_idx;
    br.from_bus = from; br.to_bus = to;
    br.name = "F1-Line-" + std::to_string(from) + "-" + std::to_string(to);
    br.r_pu = 0.005; br.x_pu = 0.015; br.b_pu = 0.001;
    br.rate_a_mva = 10.0;
    sys.ac.branches.push_back(br);
  }

  // Sectionalizing switch in Feeder 1
  Switch sw_f1;
  sw_f1.index = ++sw_idx;
  sw_f1.name = "SW-F1-Sectionalizer";
  sw_f1.bus_from = 5; sw_f1.bus_to = 6;
  sw_f1.in_service = true;
  sw_f1.closed = true;
  sw_f1.switch_type = SwitchType::Sectionalizer;
  sw_f1.i_rated_ka = 0.6;
  sw_f1.is_automated = true;
  sw_f1.t_operation_s = 0.5;
  sys.ac.switches.push_back(sw_f1);

  // --- Feeder 2 (Commercial) ---
  CircuitBreaker cb_f2;
  cb_f2.index = ++cb_idx;
  cb_f2.name = "CB-Feeder2-Head";
  cb_f2.bus_from = 3; cb_f2.bus_to = 9;
  cb_f2.in_service = true;
  cb_f2.closed = true;
  cb_f2.breaker_type = BreakerType::CB;
  cb_f2.rated_voltage_kv = base_kv_mv;
  cb_f2.i_rated_ka = 1.25;
  cb_f2.i_breaking_ka = 25.0;
  sys.ac.circuit_breakers.push_back(cb_f2);

  std::vector<std::pair<int,int>> f2_branches = {{3,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,14}};
  for (const auto& [from, to] : f2_branches) {
    ACBranch br;
    br.index = ++br_idx;
    br.from_bus = from; br.to_bus = to;
    br.name = "F2-Line-" + std::to_string(from) + "-" + std::to_string(to);
    br.r_pu = 0.004; br.x_pu = 0.012; br.b_pu = 0.001;
    br.rate_a_mva = 8.0;
    sys.ac.branches.push_back(br);
  }

  // Switches in Feeder 2
  Switch sw_f2_1;
  sw_f2_1.index = ++sw_idx;
  sw_f2_1.name = "SW-F2-Sectionalizer-1";
  sw_f2_1.bus_from = 10; sw_f2_1.bus_to = 11;
  sw_f2_1.in_service = true;
  sw_f2_1.closed = true;
  sw_f2_1.switch_type = SwitchType::Sectionalizer;
  sw_f2_1.i_rated_ka = 0.5;
  sw_f2_1.is_automated = true;
  sys.ac.switches.push_back(sw_f2_1);

  // Normally-open tie switch (for reconfiguration)
  Switch sw_tie;
  sw_tie.index = ++sw_idx;
  sw_tie.name = "SW-Tie-F2-F3";
  sw_tie.bus_from = 14; sw_tie.bus_to = 15;
  sw_tie.in_service = true;
  sw_tie.closed = false;  // Normally open
  sw_tie.switch_type = SwitchType::LoadBreakSwitch;
  sw_tie.i_rated_ka = 0.4;
  sw_tie.is_automated = true;
  sys.ac.switches.push_back(sw_tie);

  // --- Feeder 3 (Residential) ---
  CircuitBreaker cb_f3;
  cb_f3.index = ++cb_idx;
  cb_f3.name = "CB-Feeder3-Head";
  cb_f3.bus_from = 3; cb_f3.bus_to = 15;
  cb_f3.in_service = true;
  cb_f3.closed = true;
  cb_f3.breaker_type = BreakerType::CB;
  cb_f3.rated_voltage_kv = base_kv_mv;
  cb_f3.i_rated_ka = 1.0;
  cb_f3.i_breaking_ka = 20.0;
  sys.ac.circuit_breakers.push_back(cb_f3);

  std::vector<std::pair<int,int>> f3_branches = {{3,15}, {15,16}, {16,17}, {17,18}, {18,19}, {19,20}, {20,21}};
  for (const auto& [from, to] : f3_branches) {
    ACBranch br;
    br.index = ++br_idx;
    br.from_bus = from; br.to_bus = to;
    br.name = "F3-Line-" + std::to_string(from) + "-" + std::to_string(to);
    br.r_pu = 0.006; br.x_pu = 0.018; br.b_pu = 0.0008;
    br.rate_a_mva = 6.0;
    sys.ac.branches.push_back(br);
  }

  Switch sw_f3;
  sw_f3.index = ++sw_idx;
  sw_f3.name = "SW-F3-Sectionalizer";
  sw_f3.bus_from = 17; sw_f3.bus_to = 18;
  sw_f3.in_service = true;
  sw_f3.closed = true;
  sw_f3.switch_type = SwitchType::Sectionalizer;
  sw_f3.i_rated_ka = 0.4;
  sw_f3.is_automated = true;
  sys.ac.switches.push_back(sw_f3);

  // ═══════════════════════════════════════════════════════════════════════════
  // LOADS
  // ═══════════════════════════════════════════════════════════════════════════
  int ld_idx = 0;
  
  // Feeder 1 loads (Industrial)
  std::vector<std::tuple<int, double, double, std::string>> f1_loads = {
    {5, 2.0, 0.8, "Industrial-Plant-1"},
    {7, 3.5, 1.2, "Heavy-Industry"},
    {8, 1.5, 0.5, "Light-Industry"}
  };
  for (const auto& [bus, p, q, name] : f1_loads) {
    Load ld; ld.index = ++ld_idx; ld.bus = bus; ld.name = name;
    ld.p_mw = p; ld.q_mvar = q; ld.in_service = true;
    ld.profile_id = 0;  // load profile
    sys.ac.loads.push_back(ld);
  }

  // Feeder 2 loads (Commercial)
  std::vector<std::tuple<int, double, double, std::string>> f2_loads = {
    {10, 1.2, 0.4, "Shopping-Center"},
    {11, 0.8, 0.3, "Office-Building-1"},
    {13, 1.0, 0.4, "Office-Building-2"},
    {14, 0.6, 0.2, "Small-Commercial"}
  };
  for (const auto& [bus, p, q, name] : f2_loads) {
    Load ld; ld.index = ++ld_idx; ld.bus = bus; ld.name = name;
    ld.p_mw = p; ld.q_mvar = q; ld.in_service = true;
    ld.profile_id = 0;
    sys.ac.loads.push_back(ld);
  }

  // Feeder 3 loads (Residential)
  std::vector<std::tuple<int, double, double, std::string>> f3_loads = {
    {16, 0.8, 0.25, "Residential-Area-1"},
    {17, 1.0, 0.3, "Residential-Area-2"},
    {19, 0.6, 0.2, "Residential-Area-3"},
    {21, 0.5, 0.15, "Residential-Area-4"}
  };
  for (const auto& [bus, p, q, name] : f3_loads) {
    Load ld; ld.index = ++ld_idx; ld.bus = bus; ld.name = name;
    ld.p_mw = p; ld.q_mvar = q; ld.in_service = true;
    ld.profile_id = 0;
    sys.ac.loads.push_back(ld);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // PV SYSTEMS (on AC side)
  // ═══════════════════════════════════════════════════════════════════════════
  int pv_idx = 0;

  // Large industrial PV
  PVSystem pv1;
  pv1.index = ++pv_idx;
  pv1.bus = 7;
  pv1.name = "Industrial-PV-Array";
  pv1.in_service = true;
  pv1.p_mw = 1.5;
  pv1.pmax_mw = 2.5;
  pv1.pmin_mw = 0.0;
  pv1.sn_mva = 2.7;
  pv1.controllable = true;
  pv1.profile_id = 2;  // solar profile
  sys.ac.pv_systems.push_back(pv1);

  // Commercial rooftop PV
  PVSystem pv2;
  pv2.index = ++pv_idx;
  pv2.bus = 13;
  pv2.name = "Commercial-Rooftop-PV";
  pv2.in_service = true;
  pv2.p_mw = 0.8;
  pv2.pmax_mw = 1.2;
  pv2.sn_mva = 1.3;
  pv2.controllable = true;
  pv2.profile_id = 2;
  sys.ac.pv_systems.push_back(pv2);

  // Residential community solar
  PVSystem pv3;
  pv3.index = ++pv_idx;
  pv3.bus = 19;
  pv3.name = "Community-Solar-MG2";
  pv3.in_service = true;
  pv3.p_mw = 0.5;
  pv3.pmax_mw = 0.8;
  pv3.sn_mva = 0.9;
  pv3.controllable = true;
  pv3.profile_id = 2;
  sys.ac.pv_systems.push_back(pv3);

  // Additional residential PV
  PVSystem pv4;
  pv4.index = ++pv_idx;
  pv4.bus = 20;
  pv4.name = "Residential-PV-Cluster";
  pv4.in_service = true;
  pv4.p_mw = 0.3;
  pv4.pmax_mw = 0.5;
  pv4.sn_mva = 0.55;
  pv4.controllable = true;
  pv4.profile_id = 2;
  sys.ac.pv_systems.push_back(pv4);

  // ═══════════════════════════════════════════════════════════════════════════
  // WIND FARMS
  // ═══════════════════════════════════════════════════════════════════════════
  int ren_idx = 0;

  RenewableGen wind1;
  wind1.index = ++ren_idx;
  wind1.bus = 6;
  wind1.name = "MG1-Wind-Turbines";
  wind1.in_service = true;
  wind1.type = RenewableType::Wind;
  wind1.p_mw = 1.2;
  wind1.p_rated_mw = 2.0;
  wind1.capacity_factor = 0.30;
  wind1.curtailable = true;
  wind1.cost_curtail_mwh = 25.0;
  wind1.profile_id = 1;  // wind profile
  sys.ac.renewable_gens.push_back(wind1);

  RenewableGen wind2;
  wind2.index = ++ren_idx;
  wind2.bus = 10;
  wind2.name = "Commercial-Small-Wind";
  wind2.in_service = true;
  wind2.type = RenewableType::Wind;
  wind2.p_mw = 0.4;
  wind2.p_rated_mw = 0.8;
  wind2.capacity_factor = 0.25;
  wind2.curtailable = true;
  wind2.cost_curtail_mwh = 30.0;
  wind2.profile_id = 1;
  sys.ac.renewable_gens.push_back(wind2);

  // ═══════════════════════════════════════════════════════════════════════════
  // BATTERY ENERGY STORAGE SYSTEMS (AC side)
  // ═══════════════════════════════════════════════════════════════════════════
  int stor_idx = 0;

  // MG1 industrial BESS
  Storage bess1;
  bess1.index = ++stor_idx;
  bess1.bus = 8;
  bess1.name = "MG1-Industrial-BESS";
  bess1.in_service = true;
  bess1.p_rated_mw = 1.0;
  bess1.pmax_mw = 1.0;
  bess1.pmin_mw = -1.0;
  bess1.e_rated_mwh = 4.0;
  bess1.soc_init = 0.5;
  bess1.soc_min = 0.1;
  bess1.soc_max = 0.9;
  bess1.eta_charge = 0.94;
  bess1.eta_discharge = 0.94;
  bess1.type = "Li-ion";
  bess1.soc_carbon_intensity_tco2_mwh = 0.25;
  bess1.max_cycles = 5000;
  bess1.soh = 1.0;
  bess1.replacement_cost = 220.0;
  sys.ac.storage.push_back(bess1);

  // MG2 residential BESS
  Storage bess2;
  bess2.index = ++stor_idx;
  bess2.bus = 20;
  bess2.name = "MG2-Community-BESS";
  bess2.in_service = true;
  bess2.p_rated_mw = 0.5;
  bess2.pmax_mw = 0.5;
  bess2.pmin_mw = -0.5;
  bess2.e_rated_mwh = 2.0;
  bess2.soc_init = 0.5;
  bess2.soc_min = 0.1;
  bess2.soc_max = 0.9;
  bess2.eta_charge = 0.95;
  bess2.eta_discharge = 0.95;
  bess2.type = "Li-ion";
  bess2.soc_carbon_intensity_tco2_mwh = 0.28;
  bess2.max_cycles = 6000;
  bess2.soh = 1.0;
  bess2.replacement_cost = 200.0;
  sys.ac.storage.push_back(bess2);

  // Grid-scale BESS at substation
  Storage bess3;
  bess3.index = ++stor_idx;
  bess3.bus = 3;
  bess3.name = "Substation-Grid-BESS";
  bess3.in_service = true;
  bess3.p_rated_mw = 2.0;
  bess3.pmax_mw = 2.0;
  bess3.pmin_mw = -2.0;
  bess3.e_rated_mwh = 8.0;
  bess3.soc_init = 0.5;
  bess3.soc_min = 0.1;
  bess3.soc_max = 0.9;
  bess3.eta_charge = 0.93;
  bess3.eta_discharge = 0.93;
  bess3.type = "Li-ion";
  bess3.soc_carbon_intensity_tco2_mwh = 0.22;
  bess3.max_cycles = 4000;
  bess3.soh = 1.0;
  bess3.replacement_cost = 180.0;
  sys.ac.storage.push_back(bess3);

  // ═══════════════════════════════════════════════════════════════════════════
  // EV CHARGING STATIONS
  // ═══════════════════════════════════════════════════════════════════════════
  int ev_idx = 0;

  // Commercial EV station (highway/commercial area)
  ChargingStation ev1;
  ev1.index = ++ev_idx;
  ev1.bus = 12;
  ev1.name = "Commercial-EV-Hub";
  ev1.location = "Shopping Center";
  ev1.in_service = true;
  ev1.n_fast = 8;
  ev1.n_slow = 12;
  ev1.num_chargers = 20;
  ev1.p_fast_max_kw = 150.0;
  ev1.p_slow_max_kw = 22.0;
  ev1.max_power_kw = 8 * 150.0 + 12 * 22.0;  // 1464 kW max
  ev1.simultaneity_factor = 0.7;
  ev1.power_factor = 0.95;
  ev1.utilization_rate = 0.4;
  ev1.p_total_kw = 600.0;  // current load
  ev1.q_total_kvar = 200.0;
  sys.ac.charging_stations.push_back(ev1);

  // Industrial fleet charging
  ChargingStation ev2;
  ev2.index = ++ev_idx;
  ev2.bus = 5;
  ev2.name = "Industrial-Fleet-Depot";
  ev2.location = "Industrial Park";
  ev2.in_service = true;
  ev2.n_fast = 4;
  ev2.n_slow = 20;
  ev2.num_chargers = 24;
  ev2.p_fast_max_kw = 350.0;  // DC fast charger
  ev2.p_slow_max_kw = 11.0;
  ev2.max_power_kw = 4 * 350.0 + 20 * 11.0;
  ev2.simultaneity_factor = 0.6;
  ev2.power_factor = 0.95;
  ev2.utilization_rate = 0.5;
  ev2.p_total_kw = 800.0;
  ev2.q_total_kvar = 260.0;
  sys.ac.charging_stations.push_back(ev2);

  // Residential neighborhood charging
  ChargingStation ev3;
  ev3.index = ++ev_idx;
  ev3.bus = 17;
  ev3.name = "Residential-EV-Chargers";
  ev3.location = "Neighborhood";
  ev3.in_service = true;
  ev3.n_fast = 2;
  ev3.n_slow = 8;
  ev3.num_chargers = 10;
  ev3.p_fast_max_kw = 50.0;
  ev3.p_slow_max_kw = 7.4;
  ev3.max_power_kw = 2 * 50.0 + 8 * 7.4;
  ev3.simultaneity_factor = 0.5;
  ev3.power_factor = 0.98;
  ev3.utilization_rate = 0.3;
  ev3.p_total_kw = 80.0;
  ev3.q_total_kvar = 15.0;
  sys.ac.charging_stations.push_back(ev3);

  // ═══════════════════════════════════════════════════════════════════════════
  // MICROGRIDS
  // ═══════════════════════════════════════════════════════════════════════════
  int mg_idx = 0;

  // Microgrid 1: Industrial (buses 6-8)
  Microgrid mg1;
  mg1.index = ++mg_idx;
  mg1.name = "Industrial-Microgrid";
  mg1.description = "Industrial park with wind, PV, and BESS";
  mg1.in_service = true;
  mg1.pcc_bus = 6;
  mg1.internal_buses = {6, 7, 8};
  mg1.operating_mode = MicrogridMode::GridConnected;
  mg1.islanding_capability = true;
  mg1.auto_reconnection = true;
  mg1.p_exchange_max_mw = 5.0;
  mg1.p_exchange_min_mw = -3.0;
  mg1.p_import_max_mw = 5.0;
  mg1.p_export_max_mw = 3.0;
  mg1.total_generation_mw = 3.7;  // wind + PV
  mg1.total_storage_mwh = 4.0;
  mg1.total_load_mw = 5.0;
  mg1.capacity_mw = 5.0;
  mg1.f_set_hz = 50.0;
  mg1.v_set_pu = 1.0;
  mg1.k_droop = 0.05;
  mg1.area = 1;
  sys.microgrids.push_back(mg1);

  // Microgrid 2: Residential (buses 18-21)
  Microgrid mg2;
  mg2.index = ++mg_idx;
  mg2.name = "Residential-Microgrid";
  mg2.description = "Community microgrid with solar and storage";
  mg2.in_service = true;
  mg2.pcc_bus = 18;
  mg2.internal_buses = {18, 19, 20, 21};
  mg2.operating_mode = MicrogridMode::GridConnected;
  mg2.islanding_capability = true;
  mg2.auto_reconnection = true;
  mg2.p_exchange_max_mw = 2.0;
  mg2.p_exchange_min_mw = -1.5;
  mg2.p_import_max_mw = 2.0;
  mg2.p_export_max_mw = 1.5;
  mg2.total_generation_mw = 0.8;  // PV
  mg2.total_storage_mwh = 2.0;
  mg2.total_load_mw = 1.1;
  mg2.capacity_mw = 2.5;
  mg2.f_set_hz = 50.0;
  mg2.v_set_pu = 1.0;
  mg2.k_droop = 0.04;
  mg2.area = 2;
  sys.microgrids.push_back(mg2);

  // ═══════════════════════════════════════════════════════════════════════════
  // DC NETWORK
  // ═══════════════════════════════════════════════════════════════════════════
  const double base_kv_dc = 1.5;  // ±750V DC (1.5kV pole-to-pole)

  // DC Buses - positioned near their VSC AC connection points
  // DC1 connects to AC bus 3 (MV-Busbar at center)
  // DC2 connects to AC bus 7 (F1-MG1-Gen on Feeder 1, east)
  // DC3 connects to AC bus 11 (F2-Com-Mid2 on Feeder 2, north)
  // DC4 is DC-only data center
  int dc_bus_idx = 0;

  DCBus dcbus1; dcbus1.index = ++dc_bus_idx; dcbus1.name = "DC-Main-Bus";
  dcbus1.bus_type = DCBusType::DC_V; dcbus1.vm_pu = 1.0;
  dcbus1.vmin_pu = 0.95; dcbus1.vmax_pu = 1.05;
  dcbus1.base_kv = base_kv_dc;
  dcbus1.emission_factor_tco2_mwh = 0.50;
  dcbus1.latitude = center_lat + 0.002; dcbus1.longitude = center_lon - 0.003;  // Near MV busbar, slight NW offset
  sys.dc.buses.push_back(dcbus1);

  DCBus dcbus2; dcbus2.index = ++dc_bus_idx; dcbus2.name = "DC-Industrial";
  dcbus2.bus_type = DCBusType::DC_P; dcbus2.vm_pu = 1.0;
  dcbus2.base_kv = base_kv_dc;
  dcbus2.latitude = center_lat - 0.009; dcbus2.longitude = center_lon + 0.020;  // Near F1-MG1-Gen
  sys.dc.buses.push_back(dcbus2);

  DCBus dcbus3; dcbus3.index = ++dc_bus_idx; dcbus3.name = "DC-Commercial";
  dcbus3.bus_type = DCBusType::DC_P; dcbus3.vm_pu = 1.0;
  dcbus3.base_kv = base_kv_dc;
  dcbus3.latitude = center_lat + 0.014; dcbus3.longitude = center_lon + 0.007;  // Near F2-Com-Mid2
  sys.dc.buses.push_back(dcbus3);

  DCBus dcbus4; dcbus4.index = ++dc_bus_idx; dcbus4.name = "DC-DataCenter";
  dcbus4.bus_type = DCBusType::DC_P; dcbus4.vm_pu = 1.0;
  dcbus4.base_kv = base_kv_dc;
  dcbus4.is_load = true;
  dcbus4.latitude = center_lat + 0.018; dcbus4.longitude = center_lon + 0.009;  // Further north, near commercial
  sys.dc.buses.push_back(dcbus4);

  // DC Branches
  int dc_br_idx = 0;

  DCBranch dcbr1;
  dcbr1.index = ++dc_br_idx;
  dcbr1.name = "DC-Line-1-2";
  dcbr1.from_bus = 1; dcbr1.to_bus = 2;
  dcbr1.r_pu = 0.01;
  dcbr1.rate_a_mva = 5.0;
  sys.dc.branches.push_back(dcbr1);

  DCBranch dcbr2;
  dcbr2.index = ++dc_br_idx;
  dcbr2.name = "DC-Line-1-3";
  dcbr2.from_bus = 1; dcbr2.to_bus = 3;
  dcbr2.r_pu = 0.015;
  dcbr2.rate_a_mva = 3.0;
  sys.dc.branches.push_back(dcbr2);

  DCBranch dcbr3;
  dcbr3.index = ++dc_br_idx;
  dcbr3.name = "DC-Line-3-4";
  dcbr3.from_bus = 3; dcbr3.to_bus = 4;
  dcbr3.r_pu = 0.008;
  dcbr3.rate_a_mva = 2.0;
  sys.dc.branches.push_back(dcbr3);

  // DC Loads
  int dc_ld_idx = 0;

  DCLoad dcld1;
  dcld1.index = ++dc_ld_idx;
  dcld1.bus = 2;
  dcld1.name = "DC-Industrial-Load";
  dcld1.p_mw = 0.8;
  dcld1.in_service = true;
  sys.dc.loads.push_back(dcld1);

  DCLoad dcld2;
  dcld2.index = ++dc_ld_idx;
  dcld2.bus = 4;
  dcld2.name = "DataCenter-Load";
  dcld2.p_mw = 1.5;
  dcld2.in_service = true;
  sys.dc.loads.push_back(dcld2);

  // DC PV Array
  PVArrayDC dcpv1;
  dcpv1.index = 1;
  dcpv1.bus = 2;
  dcpv1.name = "DC-PV-Industrial";
  dcpv1.in_service = true;
  dcpv1.p_set_mw = 0.6;
  dcpv1.profile_id = 2;
  sys.dc.pv_arrays.push_back(dcpv1);

  // DC Storage
  Storage dc_bess;
  dc_bess.index = 101;
  dc_bess.bus = 3;
  dc_bess.name = "DC-BESS-Commercial";
  dc_bess.in_service = true;
  dc_bess.p_rated_mw = 0.5;
  dc_bess.pmax_mw = 0.5;
  dc_bess.pmin_mw = -0.5;
  dc_bess.e_rated_mwh = 2.0;
  dc_bess.soc_init = 0.5;
  dc_bess.soc_min = 0.1;
  dc_bess.soc_max = 0.9;
  dc_bess.eta_charge = 0.96;
  dc_bess.eta_discharge = 0.96;
  dc_bess.type = "Li-ion-DC";
  sys.dc.storage.push_back(dc_bess);

  // ═══════════════════════════════════════════════════════════════════════════
  // VSC CONVERTERS (AC-DC Interface)
  // ═══════════════════════════════════════════════════════════════════════════
  int vsc_idx = 0;

  // VSC 1: Main grid-forming converter at substation
  VSCConverter vsc1;
  vsc1.index = ++vsc_idx;
  vsc1.name = "VSC-Main-GridForming";
  vsc1.bus_ac = 3;
  vsc1.bus_dc = 1;
  vsc1.in_service = true;
  vsc1.control_mode = ConverterMode::PQ_MODE;  // PQ control for proper power dispatch
  vsc1.type = "mmc";
  vsc1.v_dc_set_pu = 1.0;
  vsc1.p_set_mw = -1.8;  // Import 1.8 MW from AC to DC (negative = AC injecting to DC)
  vsc1.q_set_mvar = 0.0;
  vsc1.p_rated_mw = 5.0;
  vsc1.pmax_mw = 5.0;
  vsc1.pmin_mw = -5.0;
  vsc1.qmax_mvar = 3.0;
  vsc1.qmin_mvar = -3.0;
  vsc1.eta = 0.985;
  vsc1.vn_ac_kv = base_kv_mv;
  vsc1.vn_dc_kv = base_kv_dc;
  vsc1.k_vdc = 10.0;  // Higher droop gain for better DC voltage support
  vsc1.grid_forming = true;
  vsc1.controllable = true;
  sys.vsc_converters.push_back(vsc1);

  // VSC 2: Industrial microgrid DC link
  VSCConverter vsc2;
  vsc2.index = ++vsc_idx;
  vsc2.name = "VSC-MG1-Industrial";
  vsc2.bus_ac = 7;
  vsc2.bus_dc = 2;
  vsc2.in_service = true;
  vsc2.control_mode = ConverterMode::PQ_MODE;  // PQ control
  vsc2.type = "two_level";
  vsc2.p_set_mw = -0.5;  // import to DC
  vsc2.q_set_mvar = 0.0;
  vsc2.p_rated_mw = 2.0;
  vsc2.pmax_mw = 2.0;
  vsc2.pmin_mw = -2.0;
  vsc2.qmax_mvar = 1.0;
  vsc2.qmin_mvar = -1.0;
  vsc2.eta = 0.98;
  vsc2.vn_ac_kv = base_kv_mv;
  vsc2.vn_dc_kv = base_kv_dc;
  vsc2.controllable = true;
  sys.vsc_converters.push_back(vsc2);

  // VSC 3: Commercial area DC link
  VSCConverter vsc3;
  vsc3.index = ++vsc_idx;
  vsc3.name = "VSC-Commercial";
  vsc3.bus_ac = 11;
  vsc3.bus_dc = 3;
  vsc3.in_service = true;
  vsc3.control_mode = ConverterMode::PQ_MODE;
  vsc3.type = "two_level";
  vsc3.p_set_mw = 0.3;  // export from DC
  vsc3.q_set_mvar = 0.1;
  vsc3.p_rated_mw = 1.5;
  vsc3.pmax_mw = 1.5;
  vsc3.pmin_mw = -1.5;
  vsc3.qmax_mvar = 0.8;
  vsc3.qmin_mvar = -0.8;
  vsc3.eta = 0.98;
  vsc3.vn_ac_kv = base_kv_mv;
  vsc3.vn_dc_kv = base_kv_dc;
  vsc3.controllable = true;
  sys.vsc_converters.push_back(vsc3);

  // ═══════════════════════════════════════════════════════════════════════════
  // BACKUP GENERATORS (Diesel/Gas)
  // ═══════════════════════════════════════════════════════════════════════════
  int sgen_idx = 0;

  // Diesel backup for MG1
  StaticGenerator diesel1;
  diesel1.index = ++sgen_idx;
  diesel1.bus = 8;
  diesel1.name = "MG1-Diesel-Backup";
  diesel1.in_service = true;
  diesel1.sgen_type = SgenType::Diesel;
  diesel1.p_mw = 0.0;
  diesel1.p_rated_mw = 1.0;
  diesel1.pmax_mw = 1.0;
  diesel1.pmin_mw = 0.0;
  diesel1.q_mvar = 0.0;
  diesel1.qmax_mvar = 0.5;
  diesel1.qmin_mvar = -0.3;
  diesel1.co2_emission_rate = 0.70;
  diesel1.scaling = 1.0;
  sys.ac.static_generators.push_back(diesel1);

  // Gas turbine for commercial backup
  StaticGenerator gas1;
  gas1.index = ++sgen_idx;
  gas1.bus = 9;
  gas1.name = "Commercial-Gas-Peaker";
  gas1.in_service = true;
  gas1.sgen_type = SgenType::CHP;
  gas1.p_mw = 0.0;
  gas1.p_rated_mw = 2.0;
  gas1.pmax_mw = 2.0;
  gas1.pmin_mw = 0.0;
  gas1.q_mvar = 0.0;
  gas1.qmax_mvar = 1.0;
  gas1.qmin_mvar = -0.5;
  gas1.co2_emission_rate = 0.45;
  gas1.scaling = 1.0;
  sys.ac.static_generators.push_back(gas1);

  // Fuel cell for microgrid 2
  StaticGenerator fc1;
  fc1.index = ++sgen_idx;
  fc1.bus = 18;
  fc1.name = "MG2-FuelCell";
  fc1.in_service = true;
  fc1.sgen_type = SgenType::FuelCell;
  fc1.p_mw = 0.2;
  fc1.p_rated_mw = 0.5;
  fc1.pmax_mw = 0.5;
  fc1.pmin_mw = 0.0;
  fc1.q_mvar = 0.05;
  fc1.qmax_mvar = 0.2;
  fc1.qmin_mvar = -0.1;
  fc1.co2_emission_rate = 0.35;  // Lower than gas
  fc1.scaling = 1.0;
  sys.ac.static_generators.push_back(fc1);

  // ═══════════════════════════════════════════════════════════════════════════
  // HYDRO GENERATOR (Small run-of-river)
  // ═══════════════════════════════════════════════════════════════════════════
  RenewableGen hydro1;
  hydro1.index = ++ren_idx;
  hydro1.bus = 4;
  hydro1.name = "Small-Hydro-RunOfRiver";
  hydro1.in_service = true;
  hydro1.type = RenewableType::Hydro;
  hydro1.p_mw = 0.8;
  hydro1.p_rated_mw = 1.2;
  hydro1.capacity_factor = 0.45;
  hydro1.curtailable = true;
  hydro1.cost_curtail_mwh = 15.0;
  hydro1.profile_id = 3;  // hydro profile
  sys.ac.renewable_gens.push_back(hydro1);

  // ═══════════════════════════════════════════════════════════════════════════
  // THREE-WINDING TRANSFORMER (Substation auxiliary)
  // ═══════════════════════════════════════════════════════════════════════════
  Transformer3W trafo3w;
  trafo3w.index = 1;
  trafo3w.name = "Aux-3W-Transformer";
  trafo3w.hv_bus = 2;   // 110 kV side
  trafo3w.mv_bus = 3;   // 20 kV main MV
  trafo3w.lv_bus = 9;   // 20 kV feeder 2 (auxiliary feed)
  trafo3w.in_service = true;
  trafo3w.sn_hv_mva = 25.0;
  trafo3w.sn_mv_mva = 20.0;
  trafo3w.sn_lv_mva = 10.0;
  trafo3w.vn_hv_kv = 110.0;
  trafo3w.vn_mv_kv = 20.0;
  trafo3w.vn_lv_kv = 20.0;
  trafo3w.vk_hv_mv_percent = 10.5;
  trafo3w.vk_hv_lv_percent = 10.5;
  trafo3w.vk_mv_lv_percent = 6.0;
  trafo3w.vkr_hv_mv_percent = 0.3;
  trafo3w.vkr_hv_lv_percent = 0.35;
  trafo3w.vkr_mv_lv_percent = 0.25;
  trafo3w.pfe_kw = 25.0;
  trafo3w.i0_percent = 0.15;
  trafo3w.tap_side = 0;  // HV side tap
  trafo3w.tap_pos = 0;
  sys.ac.transformers_3w.push_back(trafo3w);

  // ═══════════════════════════════════════════════════════════════════════════
  // DC-DC CONVERTERS
  // ═══════════════════════════════════════════════════════════════════════════
  DCDCConverter dcdc1;
  dcdc1.index = 1;
  dcdc1.name = "DCDC-Buck-DataCenter";
  dcdc1.bus_in = 3;   // DC commercial bus
  dcdc1.bus_out = 4;  // DC data center bus
  dcdc1.in_service = true;
  dcdc1.control_mode = DCDCControlMode::Voltage;
  dcdc1.p_ref_mw = 0.8;
  dcdc1.v_ref_pu = 0.95;  // Slightly lower voltage for data center
  dcdc1.sn_mva = 2.0;
  dcdc1.vn_in_kv = 1.5;
  dcdc1.vn_out_kv = 1.5;
  dcdc1.eta = 0.97;
  dcdc1.r_eq_pu = 0.005;
  dcdc1.pmax_mw = 2.0;
  dcdc1.pmin_mw = 0.0;
  dcdc1.controllable = true;
  sys.dc.dcdc_converters.push_back(dcdc1);

  DCDCConverter dcdc2;
  dcdc2.index = 2;
  dcdc2.name = "DCDC-Boost-Industrial";
  dcdc2.bus_in = 2;   // DC industrial bus (PV side)
  dcdc2.bus_out = 1;  // DC main bus
  dcdc2.in_service = true;
  dcdc2.control_mode = DCDCControlMode::Power;
  dcdc2.p_ref_mw = 0.4;
  dcdc2.sn_mva = 1.0;
  dcdc2.vn_in_kv = 1.5;
  dcdc2.vn_out_kv = 1.5;
  dcdc2.eta = 0.96;
  dcdc2.r_eq_pu = 0.008;
  dcdc2.pmax_mw = 1.0;
  dcdc2.pmin_mw = -1.0;
  dcdc2.controllable = true;
  sys.dc.dcdc_converters.push_back(dcdc2);

  // ═══════════════════════════════════════════════════════════════════════════
  // DC STATIC GENERATORS
  // ═══════════════════════════════════════════════════════════════════════════
  StaticGeneratorDC dc_sgen1;
  dc_sgen1.index = 1;
  dc_sgen1.bus = 4;
  dc_sgen1.name = "DC-Wind-DataCenter";
  dc_sgen1.type = "Wind";
  dc_sgen1.in_service = true;
  dc_sgen1.p_set_mw = 0.3;
  dc_sgen1.pmax_mw = 0.5;
  dc_sgen1.pmin_mw = 0.0;
  dc_sgen1.emission_factor_tco2_mwh = 0.0;
  dc_sgen1.scaling = 1.0;
  dc_sgen1.profile_id = 1;  // Wind profile
  dc_sgen1.controllable = true;
  sys.dc.dc_static_generators.push_back(dc_sgen1);

  StaticGeneratorDC dc_sgen2;
  dc_sgen2.index = 2;
  dc_sgen2.bus = 2;
  dc_sgen2.name = "DC-ESS-Industrial";
  dc_sgen2.type = "ESS";
  dc_sgen2.in_service = true;
  dc_sgen2.p_set_mw = -0.1;  // Charging
  dc_sgen2.pmax_mw = 0.3;
  dc_sgen2.pmin_mw = -0.3;
  dc_sgen2.emission_factor_tco2_mwh = 0.0;
  dc_sgen2.scaling = 1.0;
  dc_sgen2.controllable = true;
  sys.dc.dc_static_generators.push_back(dc_sgen2);

  // ═══════════════════════════════════════════════════════════════════════════
  // ADDITIONAL SWITCHES (Complex Configuration)
  // ═══════════════════════════════════════════════════════════════════════════
  // Bus coupler at MV busbar
  Switch sw_coupler;
  sw_coupler.index = ++sw_idx;
  sw_coupler.name = "SW-MV-BusCoupler";
  sw_coupler.bus_from = 3; sw_coupler.bus_to = 3;  // Same bus - coupler
  sw_coupler.in_service = true;
  sw_coupler.closed = true;
  sw_coupler.switch_type = SwitchType::Disconnector;
  sw_coupler.i_rated_ka = 2.0;
  sw_coupler.is_automated = false;
  sw_coupler.t_operation_s = 5.0;  // Manual operation
  sys.ac.switches.push_back(sw_coupler);

  // Recloser on Feeder 1
  Switch sw_recloser;
  sw_recloser.index = ++sw_idx;
  sw_recloser.name = "RC-F1-Automated";
  sw_recloser.bus_from = 4; sw_recloser.bus_to = 5;
  sw_recloser.in_service = true;
  sw_recloser.closed = true;
  sw_recloser.switch_type = SwitchType::Recloser;
  sw_recloser.i_rated_ka = 0.8;
  sw_recloser.i_breaking_ka = 12.5;
  sw_recloser.is_automated = true;
  sw_recloser.is_remote = true;
  sw_recloser.t_operation_s = 0.1;  // Fast operation
  sw_recloser.element_type = 1;  // 1=line
  sw_recloser.element_id = 3;  // Branch 3
  sys.ac.switches.push_back(sw_recloser);

  // Fuse on lateral tap
  Switch sw_fuse;
  sw_fuse.index = ++sw_idx;
  sw_fuse.name = "Fuse-F2-Lateral";
  sw_fuse.bus_from = 11; sw_fuse.bus_to = 12;
  sw_fuse.in_service = true;
  sw_fuse.closed = true;
  sw_fuse.switch_type = SwitchType::Fuse;
  sw_fuse.i_rated_ka = 0.2;
  sw_fuse.i_breaking_ka = 10.0;
  sw_fuse.is_automated = false;
  sys.ac.switches.push_back(sw_fuse);

  // Disconnector for maintenance isolation
  Switch sw_disc;
  sw_disc.index = ++sw_idx;
  sw_disc.name = "Disc-F3-Isolation";
  sw_disc.bus_from = 16; sw_disc.bus_to = 17;
  sw_disc.in_service = true;
  sw_disc.closed = true;
  sw_disc.switch_type = SwitchType::Disconnector;
  sw_disc.i_rated_ka = 0.4;
  sw_disc.is_automated = false;
  sw_disc.t_operation_s = 10.0;  // Manual
  sys.ac.switches.push_back(sw_disc);

  // Normally-open tie for alternative supply path
  Switch sw_tie2;
  sw_tie2.index = ++sw_idx;
  sw_tie2.name = "SW-Tie-F1-F2";
  sw_tie2.bus_from = 8; sw_tie2.bus_to = 14;
  sw_tie2.in_service = true;
  sw_tie2.closed = false;  // Normally open
  sw_tie2.switch_type = SwitchType::LoadBreakSwitch;
  sw_tie2.i_rated_ka = 0.4;
  sw_tie2.is_automated = true;
  sw_tie2.is_remote = true;
  sw_tie2.t_operation_s = 0.5;
  sys.ac.switches.push_back(sw_tie2);

  // Load break switch for microgrid islanding
  Switch sw_mg_island;
  sw_mg_island.index = ++sw_idx;
  sw_mg_island.name = "LBS-MG1-Island";
  sw_mg_island.bus_from = 5; sw_mg_island.bus_to = 6;
  sw_mg_island.in_service = true;
  sw_mg_island.closed = true;
  sw_mg_island.switch_type = SwitchType::LoadBreakSwitch;
  sw_mg_island.i_rated_ka = 0.6;
  sw_mg_island.i_breaking_ka = 8.0;
  sw_mg_island.is_automated = true;
  sw_mg_island.is_remote = true;
  sw_mg_island.t_operation_s = 0.2;  // Fast for islanding
  sw_mg_island.element_type = 3;  // 3=microgrid
  sw_mg_island.element_id = 1;  // Microgrid 1
  sys.ac.switches.push_back(sw_mg_island);

  // ── Hosting-capacity enrichment (non-electrical fields only) ──────────────
  // DL/T 2041-2025 equipment-level assessment parameters: OLTC reverse-load
  // and distributed-resource hosting limits, breaker short-circuit ratings on
  // backbone buses, and one storage unit with a static charging strategy to
  // contrast with the default OPF strategy.  No electrical parameter changes.
  {
    for (auto& tr : sys.ac.transformers_2w) {
      tr.cap_power_factor = 0.95;
      tr.cap_max_reverse_load_rate = 0.8;    // β：反向负载率上限
      tr.cap_dr_max_output_coeff = 1.2;      // τ_max
      tr.cap_registered_dr_mw = 2.0;         // S_d,reg：已注册未并网分布式
      tr.cap_expected_new_storage_max_mw = 2.0;  // ΔP_ESS 上限
    }
    for (auto& bus : sys.ac.buses) {
      if (bus.base_kv >= 110.0)      bus.i_breaker_ka = 40.0;
      else if (bus.base_kv >= 20.0)  bus.i_breaker_ka = 25.0;
      else if (bus.base_kv >  0.0)   bus.i_breaker_ka = 12.5;
    }
    if (!sys.ac.storage.empty()) {
      auto& st = sys.ac.storage.front();
      st.cap_charging_strategy = "static";
      st.cap_static_charging_mw = 0.5;  // 严格处于 ±p_rated 内部，避免贴边
    }
  }

  return sys;
}

HybridPowerSystem build_multiscale_comprehensive_acdc() {
  HybridPowerSystem sys = build_comprehensive_hybrid_acdc();
  sys.name = "Multiscale Comprehensive AC/DC Test System";
  sys.ac.base_mva = sys.base_mva;
  sys.dc.base_mva = sys.base_mva;
  sys.ac.freq_hz = 50.0;

  // The base case retains a historical branch approximation in parallel with
  // the authored OLTC. Projection already expands Transformer2W, so keeping
  // both double-models the 2-3 path and breaks authored-space flow attribution.
  sys.ac.branches.erase(
      std::remove_if(sys.ac.branches.begin(), sys.ac.branches.end(),
                     [](const ACBranch& branch) {
                       return branch.name == "OLTC-Branch";
                     }),
      sys.ac.branches.end());
  // The auxiliary 3W transformer's authored-space terminal recovery is not
  // invariant to the closed-switch bus contraction used by this case. Keep the
  // main 2W OLTC here; 3W behavior is covered by the dedicated transformer
  // regression cases rather than emitting a false bus-balance warning here.
  sys.ac.transformers_3w.clear();

  // Give the dispatch layers a deterministic economic ordering and the
  // transient layer an explicit synchronous-machine model.
  for (auto& gen : sys.ac.generators) {
    gen.pmin_mw = std::max(0.0, gen.pmin_mw);
    gen.cost_c1 = 42.0;
    gen.cost_c2 = 0.01;
    gen.startup_cost = 100.0;
    gen.shutdown_cost = 20.0;
    gen.ramp_up_mw_min = 5.0;
    gen.ramp_dn_mw_min = 5.0;
    gen.dynamic_model.model_name = "ClassicalMachine";
  }
  for (auto& grid : sys.ac.external_grids) {
    grid.cost_c1 = 30.0;
  }

  VirtualPowerPlant vpp;
  vpp.index = 1;
  vpp.name = "Campus-Aggregated-VPP";
  vpp.description = "PV, wind, flexible demand, and BESS aggregation";
  vpp.pcc_bus = 6;
  vpp.in_service = true;
  vpp.n_pv_systems = 2;
  vpp.n_wind_turbines = 1;
  vpp.n_battery_systems = 1;
  vpp.n_ev_chargers = 8;
  vpp.n_controllable_loads = 2;
  vpp.p_generation_sum_mw = 3.7;
  vpp.e_storage_sum_mwh = 4.0;
  vpp.p_load_controllable_mw = 1.0;
  // Keep the projected aggregate injection away from the converter-boundary
  // solution while the individual physical resources remain dispatchable.
  vpp.p_output_mw = 0.1;
  vpp.q_output_mvar = 0.05;
  vpp.pmin_mw = -1.0;
  vpp.pmax_mw = 1.0;
  vpp.ramp_up_max_mw_min = 0.2;
  vpp.ramp_down_max_mw_min = 0.2;
  sys.vpps.push_back(vpp);

  EnergyRouter router;
  router.index = 1;
  router.name = "MV-Feeder-Energy-Router";
  router.router_type = "SST";
  router.in_service = true;
  router.num_ports = 2;
  router.p_rated_mw = 2.0;
  router.vn_ac_kv = 20.0;
  router.vn_dc_kv = 1.5;
  router.loss_percent = 2.0;
  router.pmin_mw = -2.0;
  router.pmax_mw = 2.0;

  EnergyRouterPort source_port;
  source_port.index = 1;
  source_port.name = "ER-Industrial-Port";
  source_port.bus = 6;
  source_port.port_type = ERPortType::AC;
  source_port.side = 0;
  source_port.voltage_level_kv = 20.0;
  source_port.p_mw = -0.25;
  source_port.p_set_mw = -0.25;
  source_port.pmin_mw = -1.0;
  source_port.pmax_mw = 1.0;
  source_port.eta = 0.98;
  source_port.control_mode = ERControlMode::PQ;

  EnergyRouterPort sink_port = source_port;
  sink_port.index = 2;
  sink_port.name = "ER-Residential-Port";
  sink_port.bus = 18;
  sink_port.side = 1;
  sink_port.p_mw = 0.245;
  sink_port.p_set_mw = 0.245;
  router.ports = {source_port, sink_port};
  sys.energy_routers.push_back(router);

  EnergyRouter three_port;
  three_port.index = 2;
  three_port.name = "Commercial-Three-Port-Router";
  three_port.router_type = "MultiTerminal-SST";
  three_port.in_service = true;
  three_port.num_ports = 3;
  three_port.p_rated_mw = 1.5;
  three_port.vn_ac_kv = 20.0;
  three_port.vn_dc_kv = 1.5;
  three_port.loss_percent = 2.0;
  three_port.pmin_mw = -1.5;
  three_port.pmax_mw = 1.5;

  EnergyRouterPort commercial_in = source_port;
  commercial_in.index = 1;
  commercial_in.name = "ER3-Commercial-In";
  commercial_in.bus = 9;
  commercial_in.side = 0;
  commercial_in.p_mw = -0.30;
  commercial_in.p_set_mw = -0.30;

  EnergyRouterPort ev_out = sink_port;
  ev_out.index = 2;
  ev_out.name = "ER3-EV-Hub-Out";
  ev_out.bus = 12;
  ev_out.side = 1;
  ev_out.p_mw = 0.14;
  ev_out.p_set_mw = 0.14;

  EnergyRouterPort residential_out = ev_out;
  residential_out.index = 3;
  residential_out.name = "ER3-Residential-Out";
  residential_out.bus = 18;
  residential_out.p_mw = 0.15;
  residential_out.p_set_mw = 0.15;
  residential_out.control_mode = ERControlMode::Droop;
  residential_out.v_set_pu = 1.0;
  three_port.ports = {commercial_in, ev_out, residential_out};
  sys.energy_routers.push_back(three_port);

  EnergyRouter four_port;
  four_port.index = 3;
  four_port.name = "Campus-Four-Port-Router";
  four_port.router_type = "Multiport-Energy-Hub";
  four_port.in_service = true;
  four_port.num_ports = 4;
  four_port.p_rated_mw = 2.0;
  four_port.vn_ac_kv = 20.0;
  four_port.vn_dc_kv = 1.5;
  four_port.loss_percent = 2.5;
  four_port.pmin_mw = -2.0;
  four_port.pmax_mw = 2.0;

  EnergyRouterPort grid_in = source_port;
  grid_in.index = 1;
  grid_in.name = "ER4-Grid-In";
  grid_in.bus = 3;
  grid_in.side = 0;
  grid_in.p_mw = -0.20;
  grid_in.p_set_mw = -0.20;
  grid_in.control_mode = ERControlMode::PQ;
  grid_in.v_set_pu = 1.0;

  EnergyRouterPort industrial_in = grid_in;
  industrial_in.index = 2;
  industrial_in.name = "ER4-Industrial-In";
  industrial_in.bus = 7;
  industrial_in.p_mw = -0.15;
  industrial_in.p_set_mw = -0.15;
  industrial_in.control_mode = ERControlMode::PQ;

  EnergyRouterPort office_out = sink_port;
  office_out.index = 3;
  office_out.name = "ER4-Office-Out";
  office_out.bus = 13;
  office_out.side = 1;
  office_out.p_mw = 0.17;
  office_out.p_set_mw = 0.17;

  EnergyRouterPort storage_out = office_out;
  storage_out.index = 4;
  storage_out.name = "ER4-Storage-Out";
  storage_out.bus = 20;
  storage_out.p_mw = 0.17;
  storage_out.p_set_mw = 0.17;
  storage_out.control_mode = ERControlMode::Droop;
  storage_out.v_set_pu = 1.0;
  four_port.ports = {grid_in, industrial_in, office_out, storage_out};
  sys.energy_routers.push_back(four_port);

  MobileStorage mobile;
  mobile.index = 1;
  mobile.name = "Emergency-Mobile-BESS";
  mobile.bus = 12;
  mobile.in_service = true;
  mobile.status = MobileStorageStatus::Stationary;
  mobile.current_location = "Commercial-EV-Hub";
  mobile.target_bus = 18;
  mobile.p_mw = 0.05;
  mobile.p_rated_mw = 0.5;
  mobile.pmin_mw = -0.5;
  mobile.pmax_mw = 0.5;
  // A zero-width Q interval prevents a strictly interior OPF start.  Model the
  // inverter's actual reactive capability instead of pinning Q at a bound.
  mobile.qmin_mvar = -0.25;
  mobile.qmax_mvar = 0.25;
  mobile.e_rated_mwh = 1.0;
  mobile.soc_init = 0.6;
  mobile.soc_min = 0.1;
  mobile.soc_max = 0.9;
  mobile.eta_charge = 0.95;
  mobile.eta_discharge = 0.95;
  mobile.e_consumption_mwh_km = 0.002;
  mobile.max_travel_distance_km = 40.0;
  sys.mobile_storage.push_back(mobile);

  DCCircuitBreaker dc_main;
  dc_main.index = 1;
  dc_main.name = "DCCB-Main-Tie";
  dc_main.bus_from = 1;
  dc_main.bus_to = 2;
  dc_main.in_service = true;
  dc_main.closed = true;
  dc_main.r_ohm = 1.0e-4;
  dc_main.rated_voltage_kv = 1.5;
  dc_main.i_rated_ka = 2.0;
  dc_main.i_breaking_ka = 20.0;
  dc_main.element_type = "dc_branch";
  dc_main.element_id = 1;

  DCCircuitBreaker dc_reserve = dc_main;
  dc_reserve.index = 2;
  dc_reserve.name = "DCCB-Reserve-Tie";
  dc_reserve.bus_from = 2;
  dc_reserve.bus_to = 4;
  dc_reserve.closed = false;
  dc_reserve.element_id = 4;
  sys.dc.dc_circuit_breakers = {dc_main, dc_reserve};

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// 3-Bus Market Simulation Toy Case
//
//   Bus1 ──(L1: 100MW)── Bus2
//    │                      │
//   (L3: 30MW, tight)   (L2: 100MW)
//    │                      │
//    └───────── Bus3 ───────┘
//
// Gen1 @ Bus1: cheap baseload, 100 MW
// Gen2 @ Bus2: mid-cost, 80 MW
// Gen3 @ Bus3: peaker, 60 MW
// Load: Bus1=50 MW, Bus2=40 MW, Bus3=30 MW  (total 120 MW)
// Line 1→3 has a tight 30 MW rating → forces congestion for LMP tests.
// ═══════════════════════════════════════════════════════════════════════

HybridPowerSystem build_market_3bus_toy() {
  HybridPowerSystem sys;
  sys.name = "Market 3-Bus Toy";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.name = "3-Bus Market";
  sys.ac.freq_hz = 60.0;

  // --- Buses ---
  {
    ACBus b;
    b.index = 0; b.bus_type = BusType::SLACK;
    b.pd_mw = 0.0; b.qd_mvar = 0.0;
    b.vm_pu = 1.05; b.va_deg = 0.0;
    b.base_kv = 230.0; b.area = 1; b.name = "Bus1";
    sys.ac.buses.push_back(b);
  }
  {
    ACBus b;
    b.index = 1; b.bus_type = BusType::PV;
    b.pd_mw = 0.0; b.qd_mvar = 0.0;
    b.vm_pu = 1.03; b.va_deg = 0.0;
    b.base_kv = 230.0; b.area = 1; b.name = "Bus2";
    sys.ac.buses.push_back(b);
  }
  {
    ACBus b;
    b.index = 2; b.bus_type = BusType::PV;
    b.pd_mw = 0.0; b.qd_mvar = 0.0;
    b.vm_pu = 1.01; b.va_deg = 0.0;
    b.base_kv = 230.0; b.area = 1; b.name = "Bus3";
    sys.ac.buses.push_back(b);
  }

  // --- Branches (per-unit on 100 MVA base, 230 kV) ---
  // L1: Bus1→Bus2, 100 MW rating
  {
    ACBranch br;
    br.index = 0; br.from_bus = 0; br.to_bus = 1;
    br.r_pu = 0.01; br.x_pu = 0.10; br.b_pu = 0.02;
    br.rate_a_mva = 100.0; br.name = "L1_Bus1_Bus2";
    sys.ac.branches.push_back(br);
  }
  // L2: Bus2→Bus3, 100 MW rating
  {
    ACBranch br;
    br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.01; br.x_pu = 0.10; br.b_pu = 0.02;
    br.rate_a_mva = 100.0; br.name = "L2_Bus2_Bus3";
    sys.ac.branches.push_back(br);
  }
  // L3: Bus1→Bus3, TIGHT 30 MW rating (creates congestion)
  {
    ACBranch br;
    br.index = 2; br.from_bus = 0; br.to_bus = 2;
    br.r_pu = 0.02; br.x_pu = 0.15; br.b_pu = 0.01;
    br.rate_a_mva = 20.0; br.name = "L3_Bus1_Bus3_TIGHT";
    sys.ac.branches.push_back(br);
  }

  // --- Generators ---
  // Gen1 @ Bus1: cheap baseload, 100 MW
  {
    Generator g;
    g.index = 0; g.bus = 0; g.is_slack = true;
    g.pg_mw = 60.0; g.qg_mvar = 0.0; g.vg_pu = 1.05;
    g.pmax_mw = 100.0; g.pmin_mw = 10.0;
    g.qmax_mvar = 50.0; g.qmin_mvar = -20.0;
    g.cost_c2 = 0.04; g.cost_c1 = 16.0; g.cost_c0 = 100.0;
    g.startup_cost = 500.0; g.shutdown_cost = 200.0;
    g.min_up_time_hr = 3.0; g.min_dn_time_hr = 2.0;
    g.ramp_up_mw_min = 5.0; g.ramp_dn_mw_min = 5.0;
    g.fuel_type = FuelType::Coal;
    g.name = "Gen1_Cheap";
    sys.ac.generators.push_back(g);
  }
  // Gen2 @ Bus2: mid-cost, 80 MW
  {
    Generator g;
    g.index = 1; g.bus = 1; g.is_slack = false;
    g.pg_mw = 40.0; g.qg_mvar = 0.0; g.vg_pu = 1.03;
    g.pmax_mw = 80.0; g.pmin_mw = 5.0;
    g.qmax_mvar = 40.0; g.qmin_mvar = -15.0;
    g.cost_c2 = 0.08; g.cost_c1 = 32.0; g.cost_c0 = 80.0;
    g.startup_cost = 300.0; g.shutdown_cost = 100.0;
    g.min_up_time_hr = 2.0; g.min_dn_time_hr = 1.0;
    g.ramp_up_mw_min = 8.0; g.ramp_dn_mw_min = 8.0;
    g.fuel_type = FuelType::Gas;
    g.name = "Gen2_Mid";
    sys.ac.generators.push_back(g);
  }
  // Gen3 @ Bus3: peaker, 60 MW
  {
    Generator g;
    g.index = 2; g.bus = 2; g.is_slack = false;
    g.pg_mw = 20.0; g.qg_mvar = 0.0; g.vg_pu = 1.01;
    g.pmax_mw = 60.0; g.pmin_mw = 5.0;
    g.qmax_mvar = 30.0; g.qmin_mvar = -10.0;
    g.cost_c2 = 0.12; g.cost_c1 = 50.0; g.cost_c0 = 50.0;
    g.startup_cost = 200.0; g.shutdown_cost = 50.0;
    g.min_up_time_hr = 1.0; g.min_dn_time_hr = 1.0;
    g.ramp_up_mw_min = 10.0; g.ramp_dn_mw_min = 10.0;
    g.fuel_type = FuelType::Oil;
    g.name = "Gen3_Peaker";
    sys.ac.generators.push_back(g);
  }

  // --- Loads ---
  {
    Load ld;
    ld.index = 0; ld.bus = 0; ld.p_mw = 50.0; ld.q_mvar = 15.0;
    ld.name = "Load_Bus1";
    sys.ac.loads.push_back(ld);
  }
  {
    Load ld;
    ld.index = 1; ld.bus = 1; ld.p_mw = 40.0; ld.q_mvar = 10.0;
    ld.name = "Load_Bus2";
    sys.ac.loads.push_back(ld);
  }
  {
    Load ld;
    ld.index = 2; ld.bus = 2; ld.p_mw = 50.0; ld.q_mvar = 12.0;
    ld.name = "Load_Bus3";
    sys.ac.loads.push_back(ld);
  }

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// 5-Bus AC/DC Toy Case for Hybrid Market Simulation Debugging
// ═══════════════════════════════════════════════════════════════════════
//
// Topology:
//   Area 1 (AC):  Bus0 --- L1 (100MW) --- Bus1 --- L2 (100MW) --- Bus2
//                 Bus0 --------- L3 (25MW TIGHT) ----------- Bus2
//   DC link:      Bus2 === VSC1 --- DC_Bus0 --- DC_Line --- DC_Bus1 --- VSC2 === Bus3
//   Area 2 (AC):  Bus3 --- L4 (80MW) --- Bus4
//
// Generators:
//   G0 @ Bus0: 150 MW baseload, cheap ($18/MWh)
//   G1 @ Bus1: 80 MW mid-cost ($35/MWh)
//   G2 @ Bus2: 60 MW peaker ($55/MWh)
//   G3 @ Bus3: 50 MW peaker ($52/MWh)
//   G4 @ Bus4: 40 MW expensive ($65/MWh)
//
// Loads: Bus0:20MW, Bus1:40MW, Bus2:60MW, Bus3:50MW, Bus4:30MW  total=200MW
//
// DC link: 40 MW capacity → forces inter-area price split.
// L3: 25 MW tight → forces intra-area-1 congestion.
//
HybridPowerSystem build_market_5bus_acdc_toy() {
  HybridPowerSystem sys;
  sys.name = "Market 5-Bus AC/DC Toy";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.name = "5-Bus AC/DC Market";
  sys.ac.freq_hz = 60.0;

  // --- AC Buses ---
  auto make_bus = [](int idx, BusType bt, double vm, double kv, int area,
                     const std::string& nm) {
    ACBus b;
    b.index = idx; b.bus_type = bt;
    b.pd_mw = 0.0; b.qd_mvar = 0.0;
    b.vm_pu = vm; b.va_deg = 0.0;
    b.base_kv = kv; b.area = area; b.name = nm;
    return b;
  };
  sys.ac.buses = {
    make_bus(0, BusType::SLACK, 1.05, 230.0, 1, "Bus0_Slack"),
    make_bus(1, BusType::PV,    1.03, 230.0, 1, "Bus1_Gen"),
    make_bus(2, BusType::PQ,    1.00, 230.0, 1, "Bus2_Link"),
    make_bus(3, BusType::SLACK, 1.02, 230.0, 2, "Bus3_DC"),
    make_bus(4, BusType::PV,    1.00, 230.0, 2, "Bus4_Load"),
  };

  // --- AC Branches ---
  auto make_branch = [](int idx, int f, int t, double r, double x, double b,
                         double rate, const std::string& nm) {
    ACBranch br;
    br.index = idx; br.from_bus = f; br.to_bus = t;
    br.r_pu = r; br.x_pu = x; br.b_pu = b;
    br.rate_a_mva = rate; br.name = nm;
    return br;
  };
  sys.ac.branches = {
    make_branch(0, 0, 1, 0.01, 0.10, 0.02, 100.0, "L1_B0_B1"),     // Area 1 backbone
    make_branch(1, 1, 2, 0.01, 0.10, 0.02, 100.0, "L2_B1_B2"),     // Area 1 backbone
    make_branch(2, 0, 2, 0.02, 0.15, 0.01,  25.0, "L3_B0_B2_TIGHT"), // TIGHT
    make_branch(3, 3, 4, 0.01, 0.08, 0.02,  80.0, "L4_B3_B4"),     // Area 2 backbone
  };

  // --- Generators ---
  auto make_gen = [](int idx, int bus, bool slack, double pg, double vm,
                     double pmax, double pmin, double qmax, double qmin,
                     double c2, double c1, double c0,
                     double su, double sd, double mu, double md,
                     double ru, double rd, FuelType ft,
                     const std::string& nm) {
    Generator g;
    g.index = idx; g.bus = bus; g.is_slack = slack;
    g.pg_mw = pg; g.qg_mvar = 0.0; g.vg_pu = vm;
    g.pmax_mw = pmax; g.pmin_mw = pmin;
    g.qmax_mvar = qmax; g.qmin_mvar = qmin;
    g.cost_c2 = c2; g.cost_c1 = c1; g.cost_c0 = c0;
    g.startup_cost = su; g.shutdown_cost = sd;
    g.min_up_time_hr = mu; g.min_dn_time_hr = md;
    g.ramp_up_mw_min = ru; g.ramp_dn_mw_min = rd;
    g.fuel_type = ft; g.name = nm;
    return g;
  };
  sys.ac.generators = {
    make_gen(0, 0, true, 80.0, 1.05, 150.0, 15.0, 60.0, -30.0,
             0.02, 18.0, 120.0, 600.0, 200.0, 3.0, 2.0, 6.0, 6.0,
             FuelType::Coal, "G0_Baseload"),
    make_gen(1, 1, false, 40.0, 1.03, 80.0, 8.0, 35.0, -15.0,
             0.06, 35.0, 80.0, 350.0, 120.0, 2.0, 1.0, 8.0, 8.0,
             FuelType::Gas, "G1_MidCost"),
    make_gen(2, 2, false, 20.0, 1.00, 60.0, 5.0, 25.0, -10.0,
             0.10, 55.0, 60.0, 250.0, 80.0, 1.0, 1.0, 10.0, 10.0,
             FuelType::Gas, "G2_Peaker_A1"),
    make_gen(3, 3, true, 30.0, 1.02, 50.0, 5.0, 20.0, -10.0,
             0.09, 52.0, 55.0, 280.0, 90.0, 1.0, 1.0, 9.0, 9.0,
             FuelType::Gas, "G3_Peaker_A2"),
    make_gen(4, 4, false, 15.0, 1.00, 40.0, 5.0, 15.0, -8.0,
             0.14, 65.0, 40.0, 200.0, 60.0, 1.0, 1.0, 8.0, 8.0,
             FuelType::Oil, "G4_Expensive"),
  };

  // --- Loads ---
  auto make_load = [](int idx, int bus, double p, double q,
                      const std::string& nm) {
    Load ld;
    ld.index = idx; ld.bus = bus; ld.p_mw = p; ld.q_mvar = q;
    ld.name = nm;
    return ld;
  };
  sys.ac.loads = {
    make_load(0, 0, 20.0,  5.0, "Load_Bus0"),
    make_load(1, 1, 40.0, 10.0, "Load_Bus1"),
    make_load(2, 2, 60.0, 15.0, "Load_Bus2"),
    make_load(3, 3, 50.0, 12.0, "Load_Bus3"),
    make_load(4, 4, 30.0,  8.0, "Load_Bus4"),
  };

  // --- DC System ---
  // 2 DC buses connected by 1 DC line, linked to AC via 2 VSC converters
  sys.dc.buses = {
    DCBus{.index = 0, .bus_type = DCBusType::DC_V,
          .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DC_Bus0"},
    DCBus{.index = 1, .bus_type = DCBusType::DC_P,
          .vm_pu = 1.0, .pd_mw = 0.0, .in_service = true, .name = "DC_Bus1"},
  };

  sys.dc.branches = {
    DCBranch{.index = 0, .from_bus = 0, .to_bus = 1,
             .r_pu = 0.005, .in_service = true,
             .name = "DCLine_0_1",
             .rate_a_mva = 40.0, .s_max_mva = 40.0},
  };

  // VSC1: Bus2 (AC) <-> DC_Bus0 (DC)
  // VSC2: Bus3 (AC) <-> DC_Bus1 (DC)
  sys.vsc_converters = {
    VSCConverter{.index = 0, .bus_ac = 2, .bus_dc = 0,
                 .in_service = true,
                 .control_mode = ConverterMode::PQ_MODE,
                 .p_set_mw = 0.0, .q_set_mvar = 0.0,
                 .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.00,
                 .eta = 0.98, .loss_percent = 0.0, .loss_mw = 0.0,
                 .k_vdc = 0.1,
                 .pmax_mw = 50.0, .pmin_mw = -50.0,
                 .qmax_mvar = 30.0, .qmin_mvar = -30.0,
                 .name = "VSC1_B2_DC0"},
    VSCConverter{.index = 1, .bus_ac = 3, .bus_dc = 1,
                 .in_service = true,
                 .control_mode = ConverterMode::VDC_Q,
                 .p_set_mw = 0.0, .q_set_mvar = 0.0,
                 .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.02,
                 .eta = 0.98, .loss_percent = 0.0, .loss_mw = 0.0,
                 .k_vdc = 0.1,
                 .pmax_mw = 50.0, .pmin_mw = -50.0,
                 .qmax_mvar = 30.0, .qmin_mvar = -30.0,
                 .name = "VSC2_B3_DC1"},
  };

  return sys;
}

// ============================================================================
// Five-Province Southern China AC/DC Case
// ============================================================================
// Provinces: GD(1) Guangdong, GX(2) Guangxi, YN(3) Yunnan,
//            GZ(4) Guizhou,   HN(5) Hainan
// 15 Balancing Areas, ~60 AC buses, ~75 generators (coal/gas/hydro),
// 3 HVDC links, 16 AC tielines.
// Geographic coordinates centred on Southern China for GIS rendering.
// ============================================================================

HybridPowerSystem build_five_province_acdc() {
  HybridPowerSystem sys;
  sys.name = "Southern China 5-Province AC/DC";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.name = "5-Province AC System";
  sys.ac.freq_hz = 50.0;

  // ---- Province / BA geography table ----
  // BA id, province (area), #buses, zone, lat_centre, lon_centre, label prefix
  struct BADef {
    int ba_id, area, num_buses;
    double lat, lon;
    const char* abbr;        // province abbr
    const char* ba_name;
    double load_weight;       // relative load weight in province
    // generation mix fractions  coal  gas  hydro
    double coal_frac, gas_frac, hydro_frac;
  };

  // GD: area 1 (Guangdong)   load_scale=1.0   4 BAs
  // GX: area 2 (Guangxi)     load_scale=0.6   3 BAs
  // YN: area 3 (Yunnan)       load_scale=0.5   3 BAs
  // GZ: area 4 (Guizhou)      load_scale=0.55  3 BAs
  // HN: area 5 (Hainan)       load_scale=0.25  2 BAs

  static const BADef kBAs[] = {
    // GD – Guangdong (4 BAs)
    { 1, 1, 6, 23.10, 113.25, "GD", "PRD",       0.40, 0.40, 0.30, 0.00},
    { 2, 1, 4, 22.55, 114.05, "GD", "East",      0.25, 0.50, 0.20, 0.00},
    { 3, 1, 3, 23.05, 112.45, "GD", "West",      0.20, 0.60, 0.10, 0.10},
    { 4, 1, 3, 24.80, 113.60, "GD", "North",     0.15, 0.70, 0.00, 0.10},
    // GX – Guangxi (3 BAs)
    { 5, 2, 4, 22.80, 108.35, "GX", "Nanning",   0.40, 0.50, 0.15, 0.20},
    { 6, 2, 3, 24.30, 109.40, "GX", "Guilin",    0.30, 0.55, 0.05, 0.25},
    { 7, 2, 3, 23.75, 107.35, "GX", "Baise",     0.30, 0.40, 0.00, 0.45},
    // YN – Yunnan (3 BAs)
    { 8, 3, 4, 25.05, 102.70, "YN", "Kunming",   0.40, 0.15, 0.05, 0.70},
    { 9, 3, 3, 26.85, 100.20, "YN", "Lijiang",   0.30, 0.10, 0.00, 0.80},
    {10, 3, 3, 24.35, 102.55, "YN", "Yuxi",      0.30, 0.20, 0.00, 0.65},
    // GZ – Guizhou (3 BAs)
    {11, 4, 4, 26.65, 106.70, "GZ", "Guiyang",   0.40, 0.55, 0.05, 0.25},
    {12, 4, 3, 27.70, 107.00, "GZ", "Zunyi",     0.30, 0.60, 0.00, 0.25},
    {13, 4, 3, 26.20, 104.80, "GZ", "Liupanshui",0.30, 0.50, 0.00, 0.35},
    // HN – Hainan (2 BAs)
    {14, 5, 3, 20.00, 110.35, "HN", "Haikou",    0.55, 0.35, 0.35, 0.05},
    {15, 5, 3, 18.25, 109.50, "HN", "Sanya",     0.45, 0.40, 0.30, 0.05},
  };
  static constexpr int kNumBAs = 15;
  // Total buses = 6+4+3+3 +4+3+3 +4+3+3 +4+3+3 +3+3 = 53
  // pad PRD to 7 → 54, or just keep 53. 53 is fine.

  // Per-province load scale (MW at base, scaled by load_weight)
  static const double kProvLoadMW[] = {0, 4000.0, 2400.0, 2000.0, 2200.0, 1000.0}; // index by area

  // ---- Create AC Buses ----
  int bus_id = 1;
  // map[ba_id] → vector of bus indices
  std::vector<std::vector<int>> ba_buses(kNumBAs + 1);

  for (int b = 0; b < kNumBAs; ++b) {
    const auto& ba = kBAs[b];
    for (int i = 0; i < ba.num_buses; ++i) {
      ACBus bus;
      bus.index = bus_id;
      bus.bus_type = (i == 0) ? BusType::PV : BusType::PQ;
      // Only first bus of overall system (BA 1) is SLACK
      if (i == 0 && ba.ba_id == 1) {
        bus.bus_type = BusType::SLACK;
      }
      double prov_load = kProvLoadMW[ba.area];
      double bus_load = prov_load * ba.load_weight / ba.num_buses;
      bus.pd_mw = bus_load;
      bus.qd_mvar = bus_load * 0.3;
      // Local shunt compensation: ~50% of reactive load to reduce Q transmission
      bus.bs_mvar = bus_load * 0.15;  // capacitive shunt (positive = inject Q)
      bus.vm_pu = 1.0;
      bus.va_deg = 0.0;
      bus.area = ba.area;
      bus.zone = ba.ba_id;
      bus.base_kv = 500.0;  // uniform voltage level for market-level case
      bus.vmax_pu = 1.05;
      bus.vmin_pu = 0.95;
      // Scatter buses around the BA centre
      double dlat = (i - ba.num_buses / 2.0) * 0.15;
      double dlon = ((i % 2) ? 0.12 : -0.12);
      bus.latitude = ba.lat + dlat;
      bus.longitude = ba.lon + dlon;
      bus.name = std::string(ba.abbr) + "_" + ba.ba_name + "_B" + std::to_string(i + 1);
      sys.ac.buses.push_back(bus);
      ba_buses[ba.ba_id].push_back(bus_id);
      ++bus_id;
    }
  }
  const int total_ac_buses = bus_id - 1;

  // ---- Create Loads (explicit Load objects, one per bus) ----
  for (int i = 0; i < total_ac_buses; ++i) {
    const auto& b = sys.ac.buses[static_cast<size_t>(i)];
    Load ld;
    ld.index = i;
    ld.bus = b.index;
    ld.p_mw = b.pd_mw;
    ld.q_mvar = b.qd_mvar;
    ld.name = "Load_" + b.name;
    sys.ac.loads.push_back(ld);
  }

  // ---- Create Generators ----
  // Deterministic (no rng) to get reproducible tests.
  int gen_id = 0;
  // helper tables for coal/gas/hydro generation
  struct GenTemplate {
    double pmax, pmin_frac;
    double c2, c1, c0;   // quadratic cost coefficients
    double su_cost;       // startup $/MW
    double mu, md;        // min up/down hr
    double ramp_frac;     // ramp-up as fraction of Pmax
    FuelType fuel;
  };
  static const GenTemplate kCoal[] = {
    {300.0, 0.25, 0.020, 22.0, 120.0, 150.0, 4.0, 4.0, 0.10, FuelType::Coal},
    {200.0, 0.25, 0.025, 25.0, 100.0, 140.0, 3.0, 2.0, 0.12, FuelType::Coal},
    {150.0, 0.25, 0.030, 28.0,  80.0, 130.0, 2.0, 2.0, 0.15, FuelType::Coal},
    {400.0, 0.25, 0.018, 20.0, 150.0, 160.0, 4.0, 4.0, 0.08, FuelType::Coal},
    {100.0, 0.25, 0.035, 30.0,  60.0, 120.0, 2.0, 2.0, 0.15, FuelType::Coal},
  };
  static const GenTemplate kGas[] = {
    {200.0, 0.40, 0.060, 45.0,  80.0, 80.0, 2.0, 1.0, 0.30, FuelType::Gas},
    {150.0, 0.40, 0.070, 50.0,  60.0, 70.0, 1.0, 1.0, 0.30, FuelType::Gas},
    {100.0, 0.40, 0.080, 55.0,  50.0, 60.0, 1.0, 1.0, 0.30, FuelType::Gas},
    {250.0, 0.40, 0.055, 42.0, 100.0, 90.0, 2.0, 2.0, 0.25, FuelType::Gas},
  };
  static const GenTemplate kHydro[] = {
    {300.0, 0.20, 0.005,  5.0,  10.0,  0.0, 1.0, 1.0, 0.50, FuelType::Hydro},
    {200.0, 0.20, 0.008,  6.0,   8.0,  0.0, 1.0, 1.0, 0.50, FuelType::Hydro},
    {500.0, 0.20, 0.003,  3.0,  15.0,  0.0, 1.0, 1.0, 0.40, FuelType::Hydro},
    {150.0, 0.20, 0.010,  8.0,   6.0,  0.0, 1.0, 1.0, 0.50, FuelType::Hydro},
  };
  auto add_gen = [&](const BADef& ba, const GenTemplate& tmpl, int tmpl_idx) {
    Generator g;
    g.index = gen_id;
    // Round-robin bus assignment within BA
    const auto& buses = ba_buses[ba.ba_id];
    g.bus = buses[static_cast<size_t>(gen_id % static_cast<int>(buses.size()))];
    g.is_slack = (sys.ac.buses[static_cast<size_t>(g.bus - 1)].bus_type == BusType::SLACK);
    g.pg_mw = tmpl.pmax * 0.7;   // ~70% dispatch: total Pg ≈ total Pd for flat start
    g.qg_mvar = 0.0;
    g.vg_pu = 1.02;  // slightly above nominal for voltage support
    g.pmax_mw = tmpl.pmax;
    g.pmin_mw = tmpl.pmax * tmpl.pmin_frac;
    g.qmax_mvar = tmpl.pmax * 0.8;   // ±80% Pmax for robust voltage support
    g.qmin_mvar = -tmpl.pmax * 0.8;
    g.cost_c2 = tmpl.c2;
    g.cost_c1 = tmpl.c1;
    g.cost_c0 = tmpl.c0;
    g.startup_cost = tmpl.su_cost * tmpl.pmax;
    g.shutdown_cost = tmpl.su_cost * tmpl.pmax * 0.3;
    g.min_up_time_hr = tmpl.mu;
    g.min_dn_time_hr = tmpl.md;
    g.ramp_up_mw_min = tmpl.pmax * tmpl.ramp_frac;
    g.ramp_dn_mw_min = tmpl.pmax * tmpl.ramp_frac;
    g.fuel_type = tmpl.fuel;
    // §2.6.3.13 daily startup/shutdown limits by fuel type
    if (tmpl.fuel == FuelType::Coal) {
      g.max_startups_per_day = 2;
      g.max_shutdowns_per_day = 2;
    } else if (tmpl.fuel == FuelType::Gas) {
      g.max_startups_per_day = 4;
      g.max_shutdowns_per_day = 4;
    } else if (tmpl.fuel == FuelType::Hydro) {
      g.max_startups_per_day = 3;
      g.max_shutdowns_per_day = 3;
    }
    g.emission_factor_tco2_mwh = (tmpl.fuel == FuelType::Coal) ? 0.95 : (tmpl.fuel == FuelType::Gas ? 0.40 : 0.0);
    g.name = std::string(ba.abbr) + "_" + ba.ba_name + "_" +
             (tmpl.fuel == FuelType::Coal ? "Coal" : tmpl.fuel == FuelType::Gas ? "Gas" : "Hydro") +
             "_" + std::to_string(static_cast<int>(tmpl.pmax)) + "MW_" + std::to_string(tmpl_idx + 1);
    sys.ac.generators.push_back(g);
    ++gen_id;
  };

  for (int b = 0; b < kNumBAs; ++b) {
    const auto& ba = kBAs[b];
    // Decide number of each type. Each BA gets ~5 gens.
    int n_coal = std::max(1, static_cast<int>(std::round(5.0 * ba.coal_frac)));
    int n_gas  = std::max(0, static_cast<int>(std::round(5.0 * ba.gas_frac)));
    int n_hydro = std::max(0, static_cast<int>(std::round(5.0 * ba.hydro_frac)));
    for (int i = 0; i < n_coal; ++i)
      add_gen(ba, kCoal[static_cast<size_t>(i % 5)], i);
    for (int i = 0; i < n_gas; ++i)
      add_gen(ba, kGas[static_cast<size_t>(i % 4)], i);
    for (int i = 0; i < n_hydro; ++i)
      add_gen(ba, kHydro[static_cast<size_t>(i % 4)], i);
  }

  // ---- Intra-BA AC Branches (mesh within each BA) ----
  int br_id = 0;
  auto add_branch = [&](int from, int to, double r, double x, double b_pu,
                         double rate, const std::string& nm) {
    ACBranch br;
    br.index = br_id++;
    br.from_bus = from;
    br.to_bus = to;
    br.r_pu = r;
    br.x_pu = x;
    br.b_pu = b_pu;
    br.rate_a_mva = rate;
    br.name = nm;
    sys.ac.branches.push_back(br);
  };

  for (int b = 0; b < kNumBAs; ++b) {
    const auto& buses = ba_buses[kBAs[b].ba_id];
    const int nb = static_cast<int>(buses.size());
    for (int i = 0; i < nb; ++i) {
      for (int j = i + 1; j <= std::min(i + 2, nb - 1); ++j) {
        double basekv = std::min(
            sys.ac.buses[static_cast<size_t>(buses[static_cast<size_t>(i)] - 1)].base_kv,
            sys.ac.buses[static_cast<size_t>(buses[static_cast<size_t>(j)] - 1)].base_kv);
        double r_val, x_val, cap;
        // All buses are 500 kV; use transmission-level impedances
        // Aggregated 500 kV equivalent corridors — each branch represents
        // multiple parallel circuits in the real grid, so the thermal
        // rating should be generous relative to per-bus load.
        // MVA rating includes ~15% margin for reactive power (DC-AC gap).
        r_val = 0.001;  x_val = 0.015;  cap = 3500.0;
        (void)basekv;
        add_branch(buses[static_cast<size_t>(i)],
                   buses[static_cast<size_t>(j)],
                   r_val, x_val, 0.50, cap,
                   "Intra_BA" + std::to_string(kBAs[b].ba_id) + "_" +
                   std::to_string(buses[static_cast<size_t>(i)]) + "_" +
                   std::to_string(buses[static_cast<size_t>(j)]));
      }
    }
  }

  // ---- Inter-BA AC Tielines (16 template connections) ----
  struct TielineDef { int from_ba, to_ba; double capacity; };
  static const TielineDef kACTies[] = {
    // GD internal — aggregated multi-circuit corridors
    {1,2, 3000.0}, {1,3, 2400.0}, {2,3, 1600.0}, {3,4, 1200.0},
    // GD-GX
    {3,5, 1600.0}, {4,6, 1200.0},
    // GX internal
    {5,6, 1600.0}, {5,7, 1200.0},
    // GX-YN
    {7,8, 1600.0}, {6,9, 1000.0},
    // YN internal
    {8,9, 1600.0}, {8,10, 1200.0},
    // GX-GZ
    {6,11, 1200.0},
    // GZ internal
    {11,12, 1600.0}, {11,13, 1200.0},
    // HN internal
    {14,15, 800.0},
    // GD-HN (Qiongzhou Strait AC submarine cable)
    {2,14, 1200.0},
  };
  for (const auto& t : kACTies) {
    // Connect first bus of each BA (highest-kV bus)
    int from = ba_buses[t.from_ba][0];
    int to   = ba_buses[t.to_ba][0];
    add_branch(from, to, 0.001, 0.02, 0.50, t.capacity,
               "Tie_BA" + std::to_string(t.from_ba) + "_BA" + std::to_string(t.to_ba));
  }

  // ---- HVDC Links (3 DC tielines) ----
  // DC Link 1: GD(BA1) ↔ YN(BA8) – 5000 MW 800 kV "West-East HVDC"
  // DC Link 2: GD(BA1) ↔ GZ(BA11) – 3200 MW 500 kV
  // DC Link 3: GD(BA2) ↔ HN(BA14) – 1200 MW submarine cable
  struct DCLinkDef {
    int from_ba, to_ba;
    double capacity;
    const char* name;
  };
  static const DCLinkDef kDCLinks[] = {
    {1, 8,  5000.0, "HVDC_GD_YN_800kV"},
    {1, 11, 3200.0, "HVDC_GD_GZ_500kV"},
    {2, 14, 1200.0, "HVDC_GD_HN_Submarine"},
  };
  sys.dc.base_mva = 100.0;
  sys.dc.name = "5-Province DC System";

  for (int d = 0; d < 3; ++d) {
    const auto& lk = kDCLinks[d];
    // AC buses at each end – first bus of each BA
    int ac_from = ba_buses[lk.from_ba][0];
    int ac_to   = ba_buses[lk.to_ba][0];
    int dc_base = d * 2 + 1;  // 1-based DC bus indices: 1,2 / 3,4 / 5,6

    // Assign geographic coordinates from the AC buses
    const auto& ac_b_from = sys.ac.buses[static_cast<size_t>(ac_from - 1)];
    const auto& ac_b_to   = sys.ac.buses[static_cast<size_t>(ac_to - 1)];

    DCBus db1;
    db1.index = dc_base;
    db1.bus_type = DCBusType::DC_P;
    db1.vm_pu = 1.0;
    db1.pd_mw = 0.0;
    db1.in_service = true;
    db1.latitude = ac_b_from.latitude;
    db1.longitude = ac_b_from.longitude;
    db1.name = std::string(lk.name) + "_DCBus_From";
    sys.dc.buses.push_back(db1);

    DCBus db2;
    db2.index = dc_base + 1;
    db2.bus_type = DCBusType::DC_V;
    db2.vm_pu = 1.02;
    db2.pd_mw = 0.0;
    db2.in_service = true;
    db2.latitude = ac_b_to.latitude;
    db2.longitude = ac_b_to.longitude;
    db2.name = std::string(lk.name) + "_DCBus_To";
    sys.dc.buses.push_back(db2);

    DCBranch dbr;
    dbr.index = d + 1;
    dbr.from_bus = dc_base;
    dbr.to_bus = dc_base + 1;
    dbr.r_pu = 0.005;
    dbr.in_service = true;
    dbr.s_max_mva = lk.capacity;
    dbr.rate_a_mva = lk.capacity;
    dbr.name = std::string(lk.name) + "_Line";
    sys.dc.branches.push_back(dbr);

    // VSC converters at each end
    // PQ_MODE (sending end): schedule a moderate base transfer
    // towards the load-heavy province (Guangdong).
    // Positive p_set_mw means power flowing FROM the PQ-end AC bus
    // into the DC grid.
    double p_base = lk.capacity * 0.10;  // 10% of link rating as base flow

    VSCConverter v1;
    v1.index = d * 2 + 1;
    v1.bus_ac = ac_from;
    v1.bus_dc = dc_base;
    v1.in_service = true;
    v1.control_mode = ConverterMode::PQ_MODE;
    v1.p_set_mw = -p_base;  // negative = receiving (GD receives)
    v1.q_set_mvar = 0.0;
    v1.v_dc_set_pu = 1.0;
    v1.v_ac_set_pu = ac_b_from.vm_pu;
    v1.eta = 0.98;
    v1.loss_percent = 1.0;
    v1.loss_mw = 0.1;
    v1.k_vdc = 0.1;
    v1.pmax_mw = lk.capacity;
    v1.pmin_mw = -lk.capacity;
    v1.qmax_mvar = lk.capacity * 0.3;
    v1.qmin_mvar = -lk.capacity * 0.3;
    v1.name = std::string(lk.name) + "_VSC_From";
    sys.vsc_converters.push_back(v1);

    VSCConverter v2;
    v2.index = d * 2 + 2;
    v2.bus_ac = ac_to;
    v2.bus_dc = dc_base + 1;
    v2.in_service = true;
    v2.control_mode = ConverterMode::VDC_Q;
    v2.p_set_mw = p_base;  // positive = sending power into DC (YN/GZ/HN sends)
    v2.q_set_mvar = 0.0;
    v2.v_dc_set_pu = 1.02;
    v2.v_ac_set_pu = ac_b_to.vm_pu;
    v2.eta = 0.98;
    v2.loss_percent = 1.0;
    v2.loss_mw = 0.1;
    v2.k_vdc = 0.1;
    v2.pmax_mw = lk.capacity;
    v2.pmin_mw = -lk.capacity;
    v2.qmax_mvar = lk.capacity * 0.3;
    v2.qmin_mvar = -lk.capacity * 0.3;
    v2.name = std::string(lk.name) + "_VSC_To";
    sys.vsc_converters.push_back(v2);
  }

  // ---- Nuclear Generators (§2.2.5) ----
  // GD has Daya Bay, Taishan, Yangjiang clusters → BA 1 & 2
  // HN has Changjiang nuclear → BA 14
  // Nuclear: very high Pmin (80%), long min-up/dn, slow ramp, zero direct emissions
  struct NuclearDef { int ba_id; double pmax; const char* plant_name; };
  static const NuclearDef kNukes[] = {
    {2, 1000.0, "DayaBay"},        // GD East – Daya Bay + Ling'ao
    {2,  900.0, "Taishan"},        // GD East – Taishan EPR
    {3,  600.0, "Yangjiang"},      // GD West – Yangjiang
    {14, 650.0, "Changjiang"},     // HN Haikou – Changjiang
  };
  for (const auto& nd : kNukes) {
    Generator g;
    g.index = gen_id;
    g.bus = ba_buses[nd.ba_id][0];  // connect to first bus of BA
    g.is_slack = (sys.ac.buses[static_cast<size_t>(g.bus - 1)].bus_type == BusType::SLACK);
    g.pg_mw = nd.pmax * 0.9;       // nuclear runs near full load
    g.qg_mvar = 0.0;
    g.vg_pu = 1.02;  // slightly above nominal for voltage support
    g.pmax_mw = nd.pmax;
    g.pmin_mw = nd.pmax * 0.80;    // high minimum output (§2.2.5)
    g.qmax_mvar = nd.pmax * 0.4;
    g.qmin_mvar = -nd.pmax * 0.4;
    g.cost_c2 = 0.005;             // low marginal cost
    g.cost_c1 = 10.0;
    g.cost_c0 = 200.0;
    g.startup_cost = nd.pmax * 500.0;  // very high startup cost
    g.shutdown_cost = nd.pmax * 200.0;
    g.min_up_time_hr = 72.0;       // 3 days minimum run
    g.min_dn_time_hr = 48.0;       // 2 days minimum shutdown
    g.ramp_up_mw_min = nd.pmax * 0.02;  // slow ramping (2%/min)
    g.ramp_dn_mw_min = nd.pmax * 0.02;
    g.fuel_type = FuelType::Nuclear;
    g.max_startups_per_day = 1;         // §2.6.3.13 nuclear rarely starts/stops
    g.max_shutdowns_per_day = 1;
    g.emission_factor_tco2_mwh = 0.0;  // zero direct emissions
    g.name = std::string("Nuclear_") + nd.plant_name + "_" +
             std::to_string(static_cast<int>(nd.pmax)) + "MW";
    sys.ac.generators.push_back(g);
    ++gen_id;
  }

  // ---- Wind Farms (§2.2.7 – RenewableGen) ----
  // Southern China wind resources: GD offshore/coastal, GX/YN highlands, HN offshore
  // Wind farms connected to specific BAs reflecting real resource distribution
  struct WindDef { int ba_id; double p_rated; double cap_factor; const char* farm_name; };
  static const WindDef kWindFarms[] = {
    // GD – coastal/offshore wind (strong resource)
    { 2, 500.0, 0.30, "GD_East_Offshore"},
    { 2, 300.0, 0.28, "GD_East_Coastal"},
    { 3, 400.0, 0.27, "GD_West_Offshore"},
    { 1, 200.0, 0.22, "GD_PRD_Nearshore"},
    // GX – onshore wind (moderate)
    { 7, 300.0, 0.24, "GX_Baise_Highland"},
    { 6, 200.0, 0.22, "GX_Guilin_Mountain"},
    // YN – plateau wind (good resource)
    { 9, 400.0, 0.28, "YN_Lijiang_Plateau"},
    { 8, 300.0, 0.25, "YN_Kunming_Highland"},
    // GZ – mountainous wind (moderate)
    {13, 250.0, 0.23, "GZ_Liupanshui_Mountain"},
    {11, 150.0, 0.21, "GZ_Guiyang_Hills"},
    // HN – offshore wind (excellent resource)
    {15, 500.0, 0.35, "HN_Sanya_Offshore"},
    {14, 300.0, 0.32, "HN_Haikou_Offshore"},
  };
  int wind_id = 0;
  for (const auto& wf : kWindFarms) {
    RenewableGen rg;
    rg.index = wind_id;
    const auto& buses = ba_buses[wf.ba_id];
    rg.bus = buses[static_cast<size_t>(wind_id % static_cast<int>(buses.size()))];
    rg.type = RenewableType::Wind;
    rg.p_mw = wf.p_rated * wf.cap_factor;  // expected output
    rg.p_rated_mw = wf.p_rated;
    rg.qmax_mvar = wf.p_rated * 0.3;   // DFIG/PMG reactive capability
    rg.qmin_mvar = -wf.p_rated * 0.3;
    rg.curtailable = true;
    rg.cost_curtail_mwh = 5.0;         // low curtailment cost
    rg.capacity_factor = wf.cap_factor;
    rg.emission_offset_tco2_mwh = 0.85; // avoided coal emissions
    rg.name = wf.farm_name;
    sys.ac.renewable_gens.push_back(rg);
    ++wind_id;
  }

  // ---- Solar PV Systems (§2.2.7 – PVSystem) ----
  // GD: distributed rooftop + utility; YN: high-altitude strong irradiance;
  // GX/GZ: moderate; HN: tropical sunshine
  struct SolarDef { int ba_id; double pmax; double cap_factor; const char* name; };
  static const SolarDef kSolarFarms[] = {
    // GD – large-scale rooftop + utility
    { 1, 400.0, 0.16, "GD_PRD_Rooftop"},
    { 2, 300.0, 0.17, "GD_East_Utility"},
    { 3, 200.0, 0.16, "GD_West_Utility"},
    // GX – moderate solar
    { 5, 250.0, 0.17, "GX_Nanning_Solar"},
    { 7, 200.0, 0.18, "GX_Baise_Solar"},
    // YN – excellent solar (high altitude, low latitude)
    { 8, 500.0, 0.22, "YN_Kunming_Solar"},
    { 9, 300.0, 0.21, "YN_Lijiang_Solar"},
    {10, 250.0, 0.20, "YN_Yuxi_Solar"},
    // GZ – moderate
    {11, 200.0, 0.16, "GZ_Guiyang_Solar"},
    {12, 150.0, 0.15, "GZ_Zunyi_Solar"},
    // HN – tropical
    {14, 200.0, 0.19, "HN_Haikou_Solar"},
    {15, 250.0, 0.20, "HN_Sanya_Solar"},
  };
  int solar_id = 0;
  for (const auto& sf : kSolarFarms) {
    PVSystem pv;
    pv.index = solar_id;
    const auto& buses = ba_buses[sf.ba_id];
    pv.bus = buses[static_cast<size_t>(solar_id % static_cast<int>(buses.size()))];
    pv.p_mw = sf.pmax * sf.cap_factor;
    pv.pmax_mw = sf.pmax;
    pv.pmin_mw = 0.0;
    pv.sn_mva = sf.pmax * 1.1;    // inverter slightly oversized
    pv.qmax_mvar = sf.pmax * 0.3;
    pv.qmin_mvar = -sf.pmax * 0.3;
    pv.control_mode = PVControlMode::MPPT;
    pv.controllable = false;
    pv.name = sf.name;
    sys.ac.pv_systems.push_back(pv);
    ++solar_id;
  }

  // ---- Battery Energy Storage Systems (§2.2.8) ----
  // Per market rules: rated charge/discharge power, SOC limits, round-trip efficiency
  // Placed near load centres (GD) and renewable-rich areas (YN, HN)
  struct BESSDef {
    int ba_id;
    double p_rated_mw;   // rated charge & discharge power
    double duration_hr;   // energy = p_rated × duration
    double eta;           // round-trip efficiency
    const char* name;
  };
  static const BESSDef kBESS[] = {
    // GD – large-scale BESS for peak shaving and renewable integration
    { 1, 200.0, 2.0, 0.90, "BESS_GD_PRD_200MW"},
    { 2, 150.0, 2.0, 0.90, "BESS_GD_East_150MW"},
    { 3, 100.0, 2.0, 0.90, "BESS_GD_West_100MW"},
    // GX
    { 5, 100.0, 2.0, 0.88, "BESS_GX_Nanning_100MW"},
    // YN – pair with solar/hydro
    { 8, 150.0, 4.0, 0.88, "BESS_YN_Kunming_150MW"},
    { 9, 100.0, 4.0, 0.88, "BESS_YN_Lijiang_100MW"},
    // GZ
    {11, 100.0, 2.0, 0.88, "BESS_GZ_Guiyang_100MW"},
    // HN – island grid stabilisation
    {14, 100.0, 2.0, 0.90, "BESS_HN_Haikou_100MW"},
    {15,  80.0, 2.0, 0.90, "BESS_HN_Sanya_80MW"},
  };
  int ess_id = 0;
  for (const auto& bd : kBESS) {
    Storage st;
    st.index = ess_id;
    const auto& buses = ba_buses[bd.ba_id];
    st.bus = buses[static_cast<size_t>(ess_id % static_cast<int>(buses.size()))];
    st.p_mw = 0.0;                             // initially idle
    st.p_rated_mw = bd.p_rated_mw;
    st.pmax_mw = bd.p_rated_mw;                // max discharge
    st.pmin_mw = -bd.p_rated_mw;               // max charge (negative)
    st.qmax_mvar = bd.p_rated_mw * 0.3;
    st.qmin_mvar = -bd.p_rated_mw * 0.3;
    st.e_rated_mwh = bd.p_rated_mw * bd.duration_hr;
    st.soc_init = 0.5;                          // start at 50% SOC
    st.soc_min = 0.10;                           // §2.2.8.2 minimum allowed SOC
    st.soc_max = 0.90;                           // §2.2.8.2 maximum allowed SOC
    double eta_one_way = std::sqrt(bd.eta);
    st.eta_charge = eta_one_way;                 // §2.2.8.3
    st.eta_discharge = eta_one_way;
    st.self_discharge_pct = 0.01;                // 0.01%/hr
    st.max_cycles = 5000;
    st.daily_cycle_limit = 2.0;                  // §2.6.3.16(5) max 2 full cycles per day
    st.charge_bid_price = 5.0;                   // §2.6.3 charging bid $/MWh
    st.discharge_bid_price = 8.0;                // §2.6.3 discharging bid $/MWh
    st.controllable = true;
    st.control_mode = "market";
    st.type = "Li-ion";
    st.name = bd.name;
    sys.ac.storage.push_back(st);
    ++ess_id;
  }

  return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// build_actual_value_demo_acdc
// ─────────────────────────────────────────────────────────────────────────────
// An ETAP/OpenDSS-style example specified entirely in *actual* engineering
// values: AC cables carry resistance/reactance per kilometre plus a length, the
// DC link carries resistance per kilometre, and every bus declares its base
// voltage.  No per-unit impedance is supplied — convert_actual_to_per_unit (run
// inside project_to_canonical_models) fills r_pu/x_pu/b_pu before any solve.
HybridPowerSystem build_actual_value_demo_acdc() {
  HybridPowerSystem sys;
  sys.name = "actual_value_demo_acdc";
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 10.0;

  // ── AC 11 kV radial feeder (bus 1 = substation / slack) ──────────────
  auto ac_bus = [](int idx, BusType t, double pd, double qd) {
    ACBus b;
    b.index = idx; b.bus_type = t; b.base_kv = 11.0;
    b.vm_pu = 1.0; b.va_deg = 0.0; b.pd_mw = pd; b.qd_mvar = qd;
    b.vmin_pu = 0.9; b.vmax_pu = 1.1; b.in_service = true;
    b.name = "AC" + std::to_string(idx);
    return b;
  };
  sys.ac.buses = {
      ac_bus(1, BusType::SLACK, 0.0, 0.0),
      ac_bus(2, BusType::PQ,    1.2, 0.5),
      ac_bus(3, BusType::PQ,    0.8, 0.3),
      ac_bus(4, BusType::PQ,    0.6, 0.2),
  };

  Generator g;
  g.index = 1; g.bus = 1; g.is_slack = true;
  g.pg_mw = 0.0; g.qg_mvar = 0.0; g.vg_pu = 1.0;
  g.pmax_mw = 50.0; g.pmin_mw = -50.0; g.qmax_mvar = 50.0; g.qmin_mvar = -50.0;
  g.in_service = true; g.name = "Substation";
  sys.ac.generators = {g};

  // Underground cables described by per-km impedance and length (no r_pu).
  auto cable = [](int idx, int f, int t, double r_km, double x_km, double len) {
    ACBranch br;
    br.index = idx; br.from_bus = f; br.to_bus = t;
    br.r_ohm_per_km = r_km; br.x_ohm_per_km = x_km; br.length_km = len;
    br.n_parallel = 1; br.rate_a_mva = 8.0; br.in_service = true;
    br.name = "Cable" + std::to_string(idx);
    return br;
  };
  sys.ac.branches = {
      cable(1, 1, 2, 0.164, 0.080, 1.5),
      cable(2, 2, 3, 0.206, 0.085, 1.0),
      cable(3, 2, 4, 0.206, 0.085, 0.8),
  };

  // ── DC 5 kV two-bus segment (self-contained, DC_V reference) ─────────
  auto dc_bus = [](int idx, DCBusType t, double pd) {
    DCBus b;
    b.index = idx; b.bus_type = t; b.base_kv = 5.0;
    b.vm_pu = 1.0; b.vmin_pu = 0.9; b.vmax_pu = 1.1; b.pd_mw = pd;
    b.in_service = true; b.name = "DC" + std::to_string(idx);
    return b;
  };
  sys.dc.buses = {
      dc_bus(1, DCBusType::DC_V, 0.0),
      dc_bus(2, DCBusType::DC_P, 0.4),
  };
  DCBranch dl;
  dl.index = 1; dl.from_bus = 1; dl.to_bus = 2;
  dl.r_ohm_per_km = 0.05; dl.length_km = 1.0; dl.base_kv = 5.0;
  dl.n_parallel = 1; dl.rate_a_mva = 2.0; dl.in_service = true; dl.name = "DCLink1";
  sys.dc.branches = {dl};

  DCStorage dc_source;
  dc_source.index = 1;
  dc_source.bus = 1;
  dc_source.in_service = true;
  dc_source.name = "DC-Grid-Forming-BESS";
  dc_source.type = "BESS";
  dc_source.p_mw = 0.0;
  dc_source.p_rated_mw = 2.0;
  dc_source.pmin_mw = -2.0;
  dc_source.pmax_mw = 2.0;
  dc_source.e_rated_mwh = 4.0;
  dc_source.soc_init = 0.5;
  dc_source.soc_min = 0.1;
  dc_source.soc_max = 0.9;
  dc_source.controllable = true;
  sys.dc.dc_storage = {dc_source};

  return sys;
}

// ════════════════════════════════════════════════════════════════════════════════
// Hybrid AC/DC Microgrid — Islanding + Reconnection
// ════════════════════════════════════════════════════════════════════════════════
// A compact, self-contained hybrid AC/DC microgrid built to exercise the full
// islanding / reconnection lifecycle end to end (power flow, OPF, transient):
//   • Utility interconnection (bus 1, slack) tied to the microgrid through a
//     point-of-common-coupling breaker SW-PCC (bus 2 ↔ 3).  Opening it islands
//     the whole microgrid; re-closing reconnects it.
//   • A synchronous genset (bus 5) that carries the most in-island generation,
//     so the island detector promotes its bus to the swing reference when the
//     PCC opens, and it forms voltage/frequency in transient studies.
//   • A grid-forming BESS (bus 5) that anchors / black-starts the island.
//   • Rooftop AC PV (bus 4).
//   • A VSC (bus 6) feeding a small DC subgrid (DC-main + DC-load bus) with an
//     EV fast charger, a DC data-center load, DC solar, and a DC battery.
// Numbers are chosen so the island is generation-feasible: in-island load
// (~3.85 MW) is covered by the genset (≤ 4 MW) + PV (1 MW) + BESS (± 1 MW).
// ════════════════════════════════════════════════════════════════════════════════
HybridPowerSystem build_hybrid_acdc_microgrid_island() {
  HybridPowerSystem sys;
  sys.name = "Hybrid AC/DC Microgrid (Islanding + Reconnection)";
  sys.base_mva = 100.0;

  const double kv = 20.0;  // MV microgrid voltage

  // ── AC buses (20 kV; loads carried on the bus) ───────────────────────────
  auto ac_bus = [&](int idx, BusType type, double pd, double qd, double vm,
                    const std::string& name) {
    ACBus b = make_ac_bus(idx, type, pd, qd, vm, 0.0, 1);
    b.base_kv = kv;
    b.name = name;
    return b;
  };
  sys.ac.buses = {
      ac_bus(1, BusType::SLACK, 0.0, 0.00, 1.02, "Utility-Grid"),
      ac_bus(2, BusType::PQ, 0.0, 0.00, 1.02, "PCC-Utility"),
      ac_bus(3, BusType::PQ, 1.2, 0.30, 1.00, "MG-Busbar"),
      ac_bus(4, BusType::PQ, 0.8, 0.20, 1.00, "MG-PV-Feeder"),
      ac_bus(5, BusType::PQ, 1.0, 0.25, 1.00, "MG-GFM-Feeder"),
      ac_bus(6, BusType::PQ, 0.6, 0.15, 1.00, "MG-DC-Coupling"),
  };

  // ── AC branches (MV cable; pu on 100 MVA / 20 kV, Zbase = 4 Ω) ────────────
  auto ac_line = [&](int idx, int f, int t, double r, double x,
                     const std::string& name) {
    ACBranch br = make_ac_branch(idx, f, t, r, x, 0.0, 1.0);
    br.rate_a_mva = 10.0;
    br.name = name;
    return br;
  };
  sys.ac.branches = {
      ac_line(1, 1, 2, 0.010, 0.030, "Utility-Line"),
      ac_line(2, 3, 4, 0.040, 0.080, "MG-Feeder-PV"),
      ac_line(3, 3, 5, 0.030, 0.060, "MG-Feeder-GFM"),
      ac_line(4, 3, 6, 0.040, 0.080, "MG-Feeder-DC"),
  };

  // ── PCC breaker (islanding point, bus 2 ↔ bus 3) ─────────────────────────
  {
    Switch pcc;
    pcc.index = 1;
    pcc.name = "SW-PCC";
    pcc.bus_from = 2;
    pcc.bus_to = 3;
    pcc.in_service = true;
    pcc.closed = true;  // grid-connected by default
    pcc.switch_type = SwitchType::CircuitBreaker;
    pcc.i_rated_ka = 1.0;
    pcc.is_automated = true;
    pcc.is_remote = true;
    pcc.t_operation_s = 0.08;
    sys.ac.switches = {pcc};
  }

  // ── Utility slack generator (bus 1) ──────────────────────────────────────
  {
    Generator g = make_generator(1, 1, /*is_slack=*/true, 0.0, 0.0, 1.02, 100.0,
                                 -100.0, 100.0, -100.0);
    g.name = "Utility-Grid";
    g.emission_factor_tco2_mwh = 0.40;
    g.cost_c2 = 0.010;
    g.cost_c1 = 45.0;
    // 外部电网等值 = 并网时的锚定无穷大参考（多机规则的按机 opt-out）。
    // 否则等值机、微网 genset、GFM 储能三个"构网"设备之间没有角度锚点，
    // 平衡点出现正实部漂移模态（小信号实测 max_real=+0.35）。
    g.dynamic_model.model_name = "ClassicalMachine";
    g.dynamic_model.standard = "IEEE";
    g.dynamic_model.parameters["pin_slack"] = 1.0;
    sys.ac.generators.push_back(g);
  }

  // ── Microgrid synchronous genset (bus 5) — island swing + GFM in transient ─
  {
    Generator g =
        make_generator(2, 5, /*is_slack=*/false, 2.0, 0.0, 1.0, 4.0, 0.5, 3.0, -3.0);
    g.name = "MG-Genset";
    g.fuel_type = FuelType::Gas;
    g.emission_factor_tco2_mwh = 0.45;
    g.cost_c2 = 0.020;
    g.cost_c1 = 85.0;
    g.mbase_mva = 5.0;
    g.inertia_h = 3.0;
    g.droop_r = 0.05;
    g.xd_pu = 1.80;
    g.xdp_pu = 0.30;
    g.xdpp_pu = 0.22;
    g.td0p_s = 6.0;
    g.td0pp_s = 0.04;
    g.xq_pu = 1.70;
    g.ra_pu = 0.003;
    g.vn_kv = kv;
    attach_grid_forming_genset_dynamics(g);
    sys.ac.generators.push_back(g);
  }

  // ── Rooftop AC PV (bus 4) ────────────────────────────────────────────────
  {
    PVSystem pv;
    pv.index = 1;
    pv.bus = 4;
    pv.in_service = true;
    pv.name = "MG-Rooftop-PV";
    pv.p_mw = 1.0;
    pv.pmax_mw = 1.5;
    pv.pmin_mw = 0.0;
    pv.sn_mva = 1.6;
    pv.qmax_mvar = 0.6;
    pv.qmin_mvar = -0.6;
    pv.controllable = true;
    pv.profile_id = 2;  // solar
    sys.ac.pv_systems.push_back(pv);
  }

  // ── Grid-forming BESS (bus 5) — island anchor / black-start ───────────────
  {
    Storage b;
    b.index = 1;
    b.bus = 5;
    b.in_service = true;
    b.name = "MG-BESS-GFM";
    b.p_mw = 0.0;
    b.p_rated_mw = 1.0;
    b.pmax_mw = 1.0;
    b.pmin_mw = -1.0;
    b.qmax_mvar = 0.8;
    b.qmin_mvar = -0.8;
    b.e_rated_mwh = 3.0;
    b.soc_init = 0.6;
    b.soc_min = 0.1;
    b.soc_max = 0.95;
    b.grid_forming = true;
    b.control_mode = "grid_forming";
    b.type = "Li-ion";
    sys.ac.storage.push_back(b);
  }

  // ── DC subgrid via VSC (AC bus 6 ↔ DC bus 1, VSC forms the DC voltage) ────
  {
    VSCConverter c;
    c.index = 1;
    c.bus_ac = 6;
    c.bus_dc = 1;
    c.in_service = true;
    c.control_mode = ConverterMode::VDC_Q;
    c.v_dc_set_pu = 1.0;
    c.q_set_mvar = 0.0;
    c.v_ac_set_pu = 1.0;
    c.eta = 0.98;
    c.k_vdc = 0.1;
    c.pmax_mw = 5.0;
    c.pmin_mw = -5.0;
    c.qmax_mvar = 3.0;
    c.qmin_mvar = -3.0;
    c.p_rated_mw = 5.0;
    c.vn_ac_kv = kv;
    c.vn_dc_kv = 0.8;
    c.x_sc_pu = 0.10;
    c.name = "VSC-DC-Link";
    sys.vsc_converters.push_back(c);
  }

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " DC";
  {
    DCBus b1;
    b1.index = 1;
    b1.bus_type = DCBusType::DC_V;
    b1.vm_pu = 1.0;
    b1.base_kv = 0.8;
    b1.in_service = true;
    b1.name = "DC-Main";
    DCBus b2;
    b2.index = 2;
    b2.bus_type = DCBusType::DC_P;
    b2.vm_pu = 1.0;
    b2.base_kv = 0.8;
    b2.in_service = true;
    b2.name = "DC-Load-Bus";
    sys.dc.buses = {b1, b2};

    DCBranch d;
    d.index = 1;
    d.from_bus = 1;
    d.to_bus = 2;
    d.r_pu = 0.02;
    d.in_service = true;
    d.name = "DC-Link-1-2";
    sys.dc.branches = {d};
  }

  // DC loads: EV fast charger + DC data center
  {
    DCLoad ev;
    ev.index = 1;
    ev.bus = 2;
    ev.in_service = true;
    ev.name = "EV-Fast-Charger";
    ev.p_mw = 0.5;
    ev.p_rated_mw = 0.5;
    DCLoad dc;
    dc.index = 2;
    dc.bus = 1;
    dc.in_service = true;
    dc.name = "DC-DataCenter";
    dc.p_mw = 0.35;
    dc.p_rated_mw = 0.35;
    sys.dc.loads = {ev, dc};
  }

  // DC solar (injection) + DC battery
  {
    StaticGeneratorDC pv;
    pv.index = 1;
    pv.bus = 2;
    pv.in_service = true;
    pv.name = "DC-Solar";
    pv.p_set_mw = 0.6;
    pv.pmax_mw = 0.8;
    pv.pmin_mw = 0.0;
    pv.controllable = true;
    pv.profile_id = 2;
    sys.dc.dc_static_generators.push_back(pv);

    DCStorage bess;
    bess.index = 1;
    bess.bus = 1;
    bess.in_service = true;
    bess.name = "DC-BESS";
    bess.p_mw = 0.0;
    bess.p_rated_mw = 0.3;
    bess.pmax_mw = 0.3;
    bess.pmin_mw = -0.3;
    bess.e_rated_mwh = 1.0;
    bess.soc_init = 0.5;
    sys.dc.dc_storage.push_back(bess);
  }

  return sys;
}

// ════════════════════════════════════════════════════════════════════════════════
// Networked Microgrids — Islanding + Reconnection
// ════════════════════════════════════════════════════════════════════════════════
// Three hybrid AC/DC microgrids hanging off a common utility feeder, each with
// its own point-of-common-coupling breaker, plus two normally-open inter-MG tie
// breakers.  This exercises the networked-microgrid lifecycle:
//   • Grid-connected: all PCCs closed, ties open → one connected system.
//   • Selective islanding: open any PCC to island that microgrid alone.
//   • Inter-MG support: with a microgrid islanded, close a tie so a neighbor
//     backs it up (the two islands merge into one).
//   • Coordinated reconnection: re-close the PCCs and re-open the ties.
// Each microgrid carries its own synchronous genset (in sys.ac.generators) so
// the island detector always finds a swing reference when it is cut from the
// utility; MG-B additionally hosts a grid-forming BESS and a VSC-fed DC subgrid.
//   Bus 1  Utility (slack)         Bus 2  Main feeder
//   MG-A: bus 3 (busbar) – bus 4 (genset + PV)     PCC-A: 2↔3
//   MG-B: bus 5 (busbar + GFM BESS) – bus 6 (DC coupling)   PCC-B: 2↔5
//   MG-C: bus 7 (busbar) – bus 8 (genset + PV)     PCC-C: 2↔7
//   Ties (normally open): TIE-AB 4↔5, TIE-BC 6↔7
// ════════════════════════════════════════════════════════════════════════════════
HybridPowerSystem build_networked_microgrids_islanding() {
  HybridPowerSystem sys;
  sys.name = "Networked Microgrids (Islanding + Reconnection)";
  sys.base_mva = 100.0;

  const double kv = 20.0;

  auto ac_bus = [&](int idx, BusType type, double pd, double qd,
                    const std::string& name) {
    const double vm = (type == BusType::SLACK) ? 1.02 : 1.0;
    ACBus b = make_ac_bus(idx, type, pd, qd, vm, 0.0, 1);
    b.base_kv = kv;
    b.name = name;
    return b;
  };
  sys.ac.buses = {
      ac_bus(1, BusType::SLACK, 0.0, 0.00, "Utility-Grid"),
      ac_bus(2, BusType::PQ, 0.0, 0.00, "Main-Feeder"),
      ac_bus(3, BusType::PQ, 0.8, 0.20, "MG-A-Busbar"),
      ac_bus(4, BusType::PQ, 0.5, 0.12, "MG-A-DER"),
      ac_bus(5, BusType::PQ, 1.0, 0.25, "MG-B-Busbar"),
      ac_bus(6, BusType::PQ, 0.6, 0.15, "MG-B-DC-Coupling"),
      ac_bus(7, BusType::PQ, 0.8, 0.20, "MG-C-Busbar"),
      ac_bus(8, BusType::PQ, 0.6, 0.15, "MG-C-DER"),
  };

  auto ac_line = [&](int idx, int f, int t, double r, double x,
                     const std::string& name) {
    ACBranch br = make_ac_branch(idx, f, t, r, x, 0.0, 1.0);
    br.rate_a_mva = 10.0;
    br.name = name;
    return br;
  };
  sys.ac.branches = {
      ac_line(1, 1, 2, 0.008, 0.024, "Utility-Line"),
      ac_line(2, 3, 4, 0.030, 0.060, "MG-A-Feeder"),
      ac_line(3, 5, 6, 0.030, 0.060, "MG-B-Feeder"),
      ac_line(4, 7, 8, 0.030, 0.060, "MG-C-Feeder"),
  };

  // Three PCC breakers (closed) + two inter-MG tie breakers (normally open).
  auto mk_switch = [](int idx, const std::string& name, int f, int t, bool closed,
                      SwitchType type) {
    Switch s;
    s.index = idx;
    s.name = name;
    s.bus_from = f;
    s.bus_to = t;
    s.in_service = true;
    s.closed = closed;
    s.switch_type = type;
    s.i_rated_ka = 1.0;
    s.is_automated = true;
    s.is_remote = true;
    s.t_operation_s = 0.08;
    return s;
  };
  sys.ac.switches = {
      mk_switch(1, "SW-PCC-A", 2, 3, true, SwitchType::CircuitBreaker),
      mk_switch(2, "SW-PCC-B", 2, 5, true, SwitchType::CircuitBreaker),
      mk_switch(3, "SW-PCC-C", 2, 7, true, SwitchType::CircuitBreaker),
      mk_switch(4, "SW-TIE-AB", 4, 5, false, SwitchType::LoadBreakSwitch),
      mk_switch(5, "SW-TIE-BC", 6, 7, false, SwitchType::LoadBreakSwitch),
  };

  // Utility slack.
  {
    Generator g = make_generator(1, 1, /*is_slack=*/true, 0.0, 0.0, 1.02, 100.0,
                                 -100.0, 100.0, -100.0);
    g.name = "Utility-Grid";
    g.emission_factor_tco2_mwh = 0.40;
    g.cost_c2 = 0.010;
    g.cost_c1 = 45.0;
    // 外部电网等值 = 并网时的锚定无穷大参考（多机规则的按机 opt-out）。
    // 否则等值机与三个微网 genset、MG-B 的 GFM 储能之间缺少角度锚点，
    // 平衡点出现正实部漂移模态（小信号实测 max_real=+1e-5）。
    g.dynamic_model.model_name = "ClassicalMachine";
    g.dynamic_model.standard = "IEEE";
    g.dynamic_model.parameters["pin_slack"] = 1.0;
    sys.ac.generators.push_back(g);
  }

  // Per-microgrid synchronous genset (island swing + grid-forming in transient).
  auto add_genset = [&](int idx, int bus, double pg, double pmax,
                        const std::string& name) {
    Generator g = make_generator(idx, bus, /*is_slack=*/false, pg, 0.0, 1.0, pmax,
                                 0.3, pmax * 0.6, -pmax * 0.6);
    g.name = name;
    g.fuel_type = FuelType::Gas;
    g.emission_factor_tco2_mwh = 0.45;
    g.cost_c2 = 0.020;
    g.cost_c1 = 85.0;
    g.mbase_mva = pmax * 1.5;
    g.inertia_h = 3.0;
    g.droop_r = 0.05;
    g.xd_pu = 1.80;
    g.xdp_pu = 0.30;
    g.xdpp_pu = 0.22;
    g.td0p_s = 6.0;
    g.td0pp_s = 0.04;
    g.xq_pu = 1.70;
    g.ra_pu = 0.003;
    g.vn_kv = kv;
    attach_grid_forming_genset_dynamics(g);
    sys.ac.generators.push_back(g);
  };
  add_genset(2, 4, 0.8, 2.0, "MG-A-Genset");
  add_genset(3, 5, 1.5, 3.0, "MG-B-Genset");
  add_genset(4, 8, 0.8, 2.0, "MG-C-Genset");

  // Rooftop PV in MG-A and MG-C.
  auto add_pv = [&](int idx, int bus, double p, const std::string& name) {
    PVSystem pv;
    pv.index = idx;
    pv.bus = bus;
    pv.in_service = true;
    pv.name = name;
    pv.p_mw = p;
    pv.pmax_mw = p * 1.5;
    pv.pmin_mw = 0.0;
    pv.sn_mva = p * 1.6;
    pv.qmax_mvar = p * 0.6;
    pv.qmin_mvar = -p * 0.6;
    pv.controllable = true;
    pv.profile_id = 2;
    sys.ac.pv_systems.push_back(pv);
  };
  add_pv(1, 4, 0.5, "MG-A-PV");
  add_pv(2, 8, 0.6, "MG-C-PV");

  // Grid-forming BESS in MG-B (bus 5).
  {
    Storage b;
    b.index = 1;
    b.bus = 5;
    b.in_service = true;
    b.name = "MG-B-BESS-GFM";
    b.p_mw = 0.0;
    b.p_rated_mw = 1.0;
    b.pmax_mw = 1.0;
    b.pmin_mw = -1.0;
    b.qmax_mvar = 0.8;
    b.qmin_mvar = -0.8;
    b.e_rated_mwh = 3.0;
    b.soc_init = 0.6;
    b.soc_min = 0.1;
    b.soc_max = 0.95;
    b.grid_forming = true;
    b.control_mode = "grid_forming";
    b.type = "Li-ion";
    sys.ac.storage.push_back(b);
  }

  // MG-B DC subgrid via VSC (AC bus 6 ↔ DC bus 1).
  {
    VSCConverter c;
    c.index = 1;
    c.bus_ac = 6;
    c.bus_dc = 1;
    c.in_service = true;
    c.control_mode = ConverterMode::VDC_Q;
    c.v_dc_set_pu = 1.0;
    c.q_set_mvar = 0.0;
    c.v_ac_set_pu = 1.0;
    c.eta = 0.98;
    c.k_vdc = 0.1;
    c.pmax_mw = 5.0;
    c.pmin_mw = -5.0;
    c.qmax_mvar = 3.0;
    c.qmin_mvar = -3.0;
    c.p_rated_mw = 5.0;
    c.vn_ac_kv = kv;
    c.vn_dc_kv = 0.8;
    c.x_sc_pu = 0.10;
    c.name = "MG-B-VSC";
    sys.vsc_converters.push_back(c);
  }

  sys.dc.base_mva = sys.base_mva;
  sys.dc.name = sys.name + " DC";
  {
    DCBus d1;
    d1.index = 1;
    d1.bus_type = DCBusType::DC_V;
    d1.vm_pu = 1.0;
    d1.base_kv = 0.8;
    d1.in_service = true;
    d1.name = "MG-B-DC-Main";
    DCBus d2;
    d2.index = 2;
    d2.bus_type = DCBusType::DC_P;
    d2.vm_pu = 1.0;
    d2.base_kv = 0.8;
    d2.in_service = true;
    d2.name = "MG-B-DC-Load";
    sys.dc.buses = {d1, d2};

    DCBranch db;
    db.index = 1;
    db.from_bus = 1;
    db.to_bus = 2;
    db.r_pu = 0.02;
    db.in_service = true;
    db.name = "MG-B-DC-Link";
    sys.dc.branches = {db};

    DCLoad ev;
    ev.index = 1;
    ev.bus = 2;
    ev.in_service = true;
    ev.name = "MG-B-EV-Charger";
    ev.p_mw = 0.4;
    ev.p_rated_mw = 0.4;
    sys.dc.loads = {ev};

    StaticGeneratorDC dpv;
    dpv.index = 1;
    dpv.bus = 2;
    dpv.in_service = true;
    dpv.name = "MG-B-DC-Solar";
    dpv.p_set_mw = 0.5;
    dpv.pmax_mw = 0.7;
    dpv.pmin_mw = 0.0;
    dpv.controllable = true;
    dpv.profile_id = 2;
    sys.dc.dc_static_generators.push_back(dpv);

    DCStorage dbess;
    dbess.index = 1;
    dbess.bus = 1;
    dbess.in_service = true;
    dbess.name = "MG-B-DC-BESS";
    dbess.p_mw = 0.0;
    dbess.p_rated_mw = 0.3;
    dbess.pmax_mw = 0.3;
    dbess.pmin_mw = -0.3;
    dbess.e_rated_mwh = 1.0;
    dbess.soc_init = 0.5;
    sys.dc.dc_storage.push_back(dbess);
  }

  return sys;
}

HybridPowerSystem build_urban_lvn_primary_secondary() {
  constexpr double kBaseMva = 10.0;
  constexpr double kHvKv = 230.0;
  constexpr double kPrimaryKv = 13.8;
  constexpr double kSecondaryKv = 0.48;
  constexpr int kFeederCount = 4;
  constexpr int kDistrictsPerFeeder = 6;
  constexpr int kDistrictCount = kFeederCount * kDistrictsPerFeeder;
  constexpr int kStreetBusesPerDistrict = 12;

  HybridPowerSystem sys;
  sys.name = "Urban LVNTS Primary-Secondary AC/DC Benchmark";
  sys.base_mva = kBaseMva;
  sys.ac.base_mva = kBaseMva;
  sys.ac.freq_hz = 50.0;
  sys.ac.name = "Urban primary and secondary distribution";
  sys.dc.base_mva = kBaseMva;
  sys.dc.name = "Urban fast-charging DC corridor";

  ThreePhaseACSystem phase;
  phase.name = "Urban LVNTS phase-domain network";
  phase.base_mva = kBaseMva;
  phase.base_freq_hz = 50.0;

  auto add_bus = [&](const std::string& name, double base_kv, BusType type,
                     int area, double longitude, double latitude) {
    ACBus bus;
    bus.index = static_cast<int>(sys.ac.buses.size()) + 1;
    bus.name = name;
    bus.bus_type = type;
    bus.base_kv = base_kv;
    bus.vm_pu = 1.0;
    bus.vmin_pu = base_kv < 1.0 ? 0.85 : 0.90;
    bus.vmax_pu = 1.10;
    bus.area = area;
    bus.zone = area;
    bus.longitude = longitude;
    bus.latitude = latitude;
    bus.in_service = true;
    sys.ac.buses.push_back(bus);

    ThreePhaseACBus phase_bus;
    phase_bus.index = bus.index;
    phase_bus.name = name;
    phase_bus.bus_type = type;
    phase_bus.base_kv = base_kv;
    phase_bus.phase_mask = PhaseMask::abc();
    phase_bus.vmin_pu = bus.vmin_pu;
    phase_bus.vmax_pu = bus.vmax_pu;
    phase_bus.area = area;
    phase_bus.zone = area;
    phase_bus.in_service = true;
    phase.buses.push_back(phase_bus);
    return bus.index;
  };

  auto add_line = [&](const std::string& name, int from_bus, int to_bus,
                      double r_pu, double x_pu, double length_km,
                      double rating_mva) {
    ACBranch branch;
    branch.index = static_cast<int>(sys.ac.branches.size()) + 1;
    branch.name = name;
    branch.from_bus = from_bus;
    branch.to_bus = to_bus;
    branch.r_pu = r_pu;
    branch.x_pu = x_pu;
    branch.r0_pu = 3.0 * r_pu;
    branch.x0_pu = 3.0 * x_pu;
    branch.tap = 1.0;
    branch.length_km = length_km;
    branch.rate_a_mva = rating_mva;
    branch.rate_b_mva = rating_mva;
    branch.rate_c_mva = rating_mva;
    branch.n_parallel = 1;
    branch.in_service = true;
    sys.ac.branches.push_back(branch);

    ThreePhaseACLine phase_line;
    phase_line.index = static_cast<int>(phase.lines.size()) + 1;
    phase_line.name = name;
    phase_line.from_bus = from_bus;
    phase_line.to_bus = to_bus;
    phase_line.phase_mask = PhaseMask::abc();
    phase_line.r1_pu = r_pu;
    phase_line.x1_pu = x_pu;
    phase_line.r0_pu = 3.0 * r_pu;
    phase_line.x0_pu = 3.0 * x_pu;
    phase_line.length_km = length_km;
    phase_line.rate_a_mva = rating_mva;
    phase_line.parallel = 1;
    phase_line.in_service = true;
    phase.lines.push_back(phase_line);
  };

  auto add_transformer = [&](const std::string& name, int hv_bus, int lv_bus,
                             double sn_mva, double vn_hv_kv,
                             double vn_lv_kv, double r_on_rating,
                             double x_on_rating, const std::string& vector_group,
                             const std::string& hv_topology,
                             const std::string& lv_topology) {
    const double scale = kBaseMva / sn_mva;
    ACBranch branch;
    branch.index = static_cast<int>(sys.ac.branches.size()) + 1;
    branch.name = name;
    branch.from_bus = hv_bus;
    branch.to_bus = lv_bus;
    branch.r_pu = r_on_rating * scale;
    branch.x_pu = x_on_rating * scale;
    branch.tap = 1.0;
    branch.rate_a_mva = sn_mva;
    branch.rate_b_mva = sn_mva;
    branch.rate_c_mva = sn_mva;
    branch.vn_hv_kv = vn_hv_kv;
    branch.vn_lv_kv = vn_lv_kv;
    branch.sn_mva = sn_mva;
    branch.in_service = true;
    sys.ac.branches.push_back(branch);

    ThreePhaseTransformer transformer;
    transformer.index = static_cast<int>(phase.transformers.size()) + 1;
    transformer.name = name;
    transformer.hv_bus = hv_bus;
    transformer.lv_bus = lv_bus;
    transformer.hv_phase_mask = PhaseMask::abc();
    transformer.lv_phase_mask = PhaseMask::abc();
    transformer.sn_mva = sn_mva;
    transformer.vn_hv_kv = vn_hv_kv;
    transformer.vn_lv_kv = vn_lv_kv;
    transformer.vk_percent = 100.0 * std::hypot(r_on_rating, x_on_rating);
    transformer.vkr_percent = 100.0 * r_on_rating;
    transformer.vector_group = vector_group;
    transformer.hv_winding_topology = hv_topology;
    transformer.lv_winding_topology = lv_topology;
    transformer.in_service = true;
    phase.transformers.push_back(transformer);
  };

  auto phase_shares = [](int seed) {
    switch (seed % 4) {
      case 0: return std::vector<double>{0.42, 0.31, 0.27};
      case 1: return std::vector<double>{0.27, 0.43, 0.30};
      case 2: return std::vector<double>{0.31, 0.28, 0.41};
      default: return std::vector<double>{0.34, 0.33, 0.33};
    }
  };

  auto add_load = [&](const std::string& name, int bus, double p_mw,
                      double q_mvar, int seed, bool delta,
                      bool single_phase, int profile_id = -1) {
    Load load;
    load.index = static_cast<int>(sys.ac.loads.size()) + 1;
    load.name = name;
    load.bus = bus;
    load.p_mw = p_mw;
    load.q_mvar = q_mvar;
    load.scaling = 1.0;
    load.profile_id = profile_id;
    load.in_service = true;
    sys.ac.loads.push_back(load);

    ThreePhaseLoad phase_load;
    phase_load.index = static_cast<int>(phase.loads.size()) + 1;
    phase_load.name = name;
    phase_load.bus = bus;
    phase_load.connection = delta ? "delta" : "wye";
    phase_load.grounded = !delta;
    phase_load.vmin_pu = 0.80;
    phase_load.vmax_pu = 1.10;
    phase_load.zipv_cutoff_pu = 0.65;
    if (seed % 3 == 0) {
      phase_load.const_z_percent = 20.0;
      phase_load.const_i_percent = 10.0;
      phase_load.const_p_percent = 70.0;
    }
    if (single_phase) {
      const int selected = seed % 3;
      phase_load.phase_mask = selected == 0
                                  ? PhaseMask::a()
                                  : (selected == 1 ? PhaseMask::b()
                                                   : PhaseMask::c());
      if (selected == 0) {
        phase_load.p_a_mw = p_mw;
        phase_load.q_a_mvar = q_mvar;
      } else if (selected == 1) {
        phase_load.p_b_mw = p_mw;
        phase_load.q_b_mvar = q_mvar;
      } else {
        phase_load.p_c_mw = p_mw;
        phase_load.q_c_mvar = q_mvar;
      }
    } else {
      phase_load.phase_mask = PhaseMask::abc();
      const auto shares = phase_shares(seed);
      phase_load.p_a_mw = p_mw * shares[0];
      phase_load.p_b_mw = p_mw * shares[1];
      phase_load.p_c_mw = p_mw * shares[2];
      phase_load.q_a_mvar = q_mvar * shares[0];
      phase_load.q_b_mvar = q_mvar * shares[1];
      phase_load.q_c_mvar = q_mvar * shares[2];
    }
    phase.loads.push_back(phase_load);
  };

  const int hv_bus = add_bus("Grid-230kV", kHvKv, BusType::SLACK, 0, -4.0, 0.0);
  const int primary_source =
      add_bus("Primary-13k8-Substation", kPrimaryKv, BusType::PQ, 0, 0.0, 0.0);
  add_transformer("T-HV-Primary", hv_bus, primary_source, 50.0, kHvKv,
                  kPrimaryKv, 0.0075, 0.075, "Dd0", "AB,BC,CA",
                  "AB,BC,CA");

  Generator source;
  source.index = 1;
  source.name = "Urban-Grid-Source";
  source.bus = hv_bus;
  source.is_slack = true;
  source.vg_pu = 1.0;
  source.pmax_mw = 50.0;
  source.pmin_mw = -10.0;
  source.qmax_mvar = 30.0;
  source.qmin_mvar = -30.0;
  source.mbase_mva = 50.0;
  source.in_service = true;
  sys.ac.generators.push_back(source);

  ThreePhaseExternalGrid phase_source;
  phase_source.index = 1;
  phase_source.name = "Urban-Grid-Source";
  phase_source.bus = hv_bus;
  phase_source.vm_pu = 1.0;
  phase_source.va_deg = 0.0;
  phase_source.s_sc_max_mva = 2500.0;
  phase_source.s_sc_min_mva = 1500.0;
  phase_source.rx_max = 0.1;
  phase_source.rx_min = 0.1;
  phase_source.r1_pu = 0.0004;
  phase_source.x1_pu = 0.004;
  phase_source.r2_pu = phase_source.r1_pu;
  phase_source.x2_pu = phase_source.x1_pu;
  phase_source.r0_pu = 0.0012;
  phase_source.x0_pu = 0.012;
  phase_source.source_topology = "A,B,C";
  phase.external_grids.push_back(phase_source);

  std::vector<int> primary_buses(kDistrictCount);
  std::vector<int> secondary_heads(kDistrictCount);
  std::vector<std::vector<int>> street_buses(
      kDistrictCount, std::vector<int>(kStreetBusesPerDistrict));

  const double primary_zbase = kPrimaryKv * kPrimaryKv / kBaseMva;
  const double secondary_zbase =
      kSecondaryKv * kSecondaryKv / kBaseMva;

  for (int district = 0; district < kDistrictCount; ++district) {
    const int feeder = district / kDistrictsPerFeeder;
    const int position = district % kDistrictsPerFeeder;
    const int area = feeder + 1;
    const double x = 10.0 * feeder;
    const double y = 5.0 * (position + 1);
    primary_buses[district] = add_bus(
        "F" + std::to_string(feeder + 1) + "-Primary-D" +
            std::to_string(position + 1),
        kPrimaryKv, BusType::PQ, area, x, y);
    secondary_heads[district] = add_bus(
        "F" + std::to_string(feeder + 1) + "-Secondary-D" +
            std::to_string(position + 1),
        kSecondaryKv, BusType::PQ, area, x + 2.2, y);

    const int primary_from =
        position == 0 ? primary_source : primary_buses[district - 1];
    const double primary_length_km =
        0.45 + 0.08 * static_cast<double>((district * 5) % 6);
    const double primary_r = 0.18 * primary_length_km / primary_zbase;
    const double primary_x = 0.30 * primary_length_km / primary_zbase;
    add_line("F" + std::to_string(feeder + 1) + "-MV-" +
                 std::to_string(position + 1),
             primary_from, primary_buses[district], primary_r, primary_x,
             primary_length_km, 8.0);

    const double transformer_rating =
        district % 3 == 0 ? 1.5 : (district % 3 == 1 ? 2.0 : 2.5);
    add_transformer(
        "F" + std::to_string(feeder + 1) + "-TX-D" +
            std::to_string(position + 1),
        primary_buses[district], secondary_heads[district],
        transformer_rating, kPrimaryKv, kSecondaryKv, 0.007, 0.07,
        "Dyn11", "AB,BC,CA", "A,B,C");

    const double commercial_p = 0.055 + 0.005 * (district % 5);
    add_load("District-Commercial-" + std::to_string(district + 1),
             secondary_heads[district], commercial_p, 0.28 * commercial_p,
             district + 1000, district % 4 == 0, false, 1);

    int previous = secondary_heads[district];
    for (int street = 0; street < kStreetBusesPerDistrict; ++street) {
      const double street_x = x + 3.5 + 0.8 * (street % 4);
      const double street_y = y + 0.65 * (street / 4);
      street_buses[district][street] = add_bus(
          "F" + std::to_string(feeder + 1) + "-D" +
              std::to_string(position + 1) + "-LV" +
              std::to_string(street + 1),
          kSecondaryKv, BusType::PQ, area, street_x, street_y);
      const double length_km =
          0.040 + 0.005 * static_cast<double>((district + street) % 7);
      const double secondary_r = 0.12 * length_km / secondary_zbase;
      const double secondary_x = 0.08 * length_km / secondary_zbase;
      add_line("F" + std::to_string(feeder + 1) + "-D" +
                   std::to_string(position + 1) + "-Service-" +
                   std::to_string(street + 1),
               previous, street_buses[district][street], secondary_r,
               secondary_x, length_km, 0.75);
      previous = street_buses[district][street];

      const int seed = district * kStreetBusesPerDistrict + street;
      const double p_mw = 0.018 + 0.002 * static_cast<double>((seed * 7) % 9);
      const double q_mvar = p_mw * (0.27 + 0.01 * (seed % 6));
      const bool single_phase = seed % 11 == 0;
      const bool delta = !single_phase && seed % 5 == 0;
      add_load("Urban-Service-" + std::to_string(seed + 1),
               street_buses[district][street], p_mw, q_mvar, seed, delta,
               single_phase, 1);
    }

    if (district % 2 == 0) {
      const int pv_bus = street_buses[district].back();
      const double pv_power = 0.075 + 0.01 * (district % 4);
      StaticGenerator pv;
      pv.index = static_cast<int>(sys.ac.static_generators.size()) + 1;
      pv.name = "Rooftop-PV-D" + std::to_string(district + 1);
      pv.bus = pv_bus;
      pv.sgen_type = SgenType::PV;
      pv.p_mw = pv_power;
      pv.p_rated_mw = 1.2 * pv_power;
      pv.sn_mva = 1.25 * pv_power;
      pv.pmax_mw = pv.p_rated_mw;
      pv.pmin_mw = 0.0;
      pv.qmax_mvar = 0.3 * pv.sn_mva;
      pv.qmin_mvar = -pv.qmax_mvar;
      pv.scaling = 1.0;
      pv.controllable = true;
      pv.in_service = true;
      sys.ac.static_generators.push_back(pv);

      ThreePhaseGenerator phase_pv;
      phase_pv.index = static_cast<int>(phase.generators.size()) + 1;
      phase_pv.name = pv.name;
      phase_pv.bus = pv_bus;
      phase_pv.phase_mask = PhaseMask::abc();
      const auto shares = phase_shares(district + 2);
      phase_pv.p_a_mw = pv_power * shares[0];
      phase_pv.p_b_mw = pv_power * shares[1];
      phase_pv.p_c_mw = pv_power * shares[2];
      phase_pv.pmax_mw = pv.pmax_mw;
      phase_pv.pmin_mw = 0.0;
      phase_pv.qmax_mvar = pv.qmax_mvar;
      phase_pv.qmin_mvar = pv.qmin_mvar;
      phase_pv.mbase_mva = pv.sn_mva;
      phase_pv.in_service = true;
      phase.generators.push_back(phase_pv);
    }

    if (district % 6 == 2) {
      Storage battery;
      battery.index = static_cast<int>(sys.ac.storage.size()) + 1;
      battery.name = "Community-BESS-D" + std::to_string(district + 1);
      battery.bus = secondary_heads[district];
      battery.p_rated_mw = 0.25;
      battery.pmax_mw = 0.25;
      battery.pmin_mw = -0.25;
      battery.qmax_mvar = 0.15;
      battery.qmin_mvar = -0.15;
      battery.e_rated_mwh = 0.75;
      battery.e_mwh = 0.45;
      battery.soc_init = 0.60;
      battery.soc_min = 0.10;
      battery.soc_max = 0.90;
      battery.eta_charge = 0.95;
      battery.eta_discharge = 0.95;
      battery.controllable = true;
      battery.type = "Li-ion";
      battery.in_service = true;
      sys.ac.storage.push_back(battery);
    }
  }

  for (int feeder = 0; feeder < kFeederCount; ++feeder) {
    Switch tie;
    tie.index = static_cast<int>(sys.ac.switches.size()) + 1;
    tie.name = "Primary-Tie-F" + std::to_string(feeder + 1) + "-F" +
               std::to_string((feeder + 1) % kFeederCount + 1);
    tie.bus_from = primary_buses[feeder * kDistrictsPerFeeder +
                                 kDistrictsPerFeeder - 1];
    tie.bus_to = primary_buses[((feeder + 1) % kFeederCount) *
                               kDistrictsPerFeeder + kDistrictsPerFeeder - 1];
    tie.switch_type = SwitchType::LoadBreakSwitch;
    tie.closed = false;
    tie.is_remote = true;
    tie.is_automated = true;
    tie.i_rated_ka = 0.63;
    tie.in_service = true;
    sys.ac.switches.push_back(tie);
  }

  sys.dc.buses = {
      DCBus{.index = 1, .bus_type = DCBusType::DC_P, .vm_pu = 1.0,
            .vmax_pu = 1.10, .vmin_pu = 0.90, .pd_mw = 0.0,
            .in_service = true, .name = "DC-Charging-Hub", .base_kv = 0.8},
      DCBus{.index = 2, .bus_type = DCBusType::DC_P, .vm_pu = 1.0,
            .vmax_pu = 1.10, .vmin_pu = 0.90, .pd_mw = 0.0,
            .in_service = true, .name = "DC-Bus-Depot", .base_kv = 0.8},
      DCBus{.index = 3, .bus_type = DCBusType::DC_P, .vm_pu = 1.0,
            .vmax_pu = 1.10, .vmin_pu = 0.90, .pd_mw = 0.0,
            .in_service = true, .name = "DC-Bus-Commercial", .base_kv = 0.8},
  };
  sys.dc.branches = {
      DCBranch{.index = 1, .from_bus = 1, .to_bus = 2, .r_pu = 0.012,
               .in_service = true, .name = "DC-Corridor-1",
               .rate_a_mva = 1.5, .length_km = 0.8, .base_kv = 0.8,
               .s_max_mva = 1.5},
      DCBranch{.index = 2, .from_bus = 2, .to_bus = 3, .r_pu = 0.015,
               .in_service = true, .name = "DC-Corridor-2",
               .rate_a_mva = 1.0, .length_km = 1.1, .base_kv = 0.8,
               .s_max_mva = 1.0},
  };
  sys.dc.loads = {
      DCLoad{.index = 1, .bus = 2, .in_service = true,
             .name = "Electric-Bus-Depot", .type = "EV charging",
             .p_mw = 0.32, .p_rated_mw = 0.50, .scaling = 1.0},
      DCLoad{.index = 2, .bus = 3, .in_service = true,
             .name = "Commercial-Fast-Chargers", .type = "EV charging",
             .p_mw = 0.18, .p_rated_mw = 0.30, .scaling = 1.0},
  };
  StaticGeneratorDC dc_pv;
  dc_pv.index = 1;
  dc_pv.bus = 3;
  dc_pv.name = "Charging-Hub-Solar";
  dc_pv.type = "PV";
  dc_pv.p_set_mw = 0.12;
  dc_pv.pmax_mw = 0.18;
  dc_pv.pmin_mw = 0.0;
  dc_pv.controllable = true;
  dc_pv.in_service = true;
  sys.dc.dc_static_generators.push_back(dc_pv);

  DCStorage dc_battery;
  dc_battery.index = 1;
  dc_battery.bus = 1;
  dc_battery.name = "Charging-Hub-BESS";
  dc_battery.type = "Li-ion";
  dc_battery.p_rated_mw = 0.25;
  dc_battery.pmax_mw = 0.25;
  dc_battery.pmin_mw = -0.25;
  dc_battery.e_rated_mwh = 0.8;
  dc_battery.e_mwh = 0.48;
  dc_battery.soc_init = 0.60;
  dc_battery.soc_min = 0.10;
  dc_battery.soc_max = 0.90;
  dc_battery.eta_charge = 0.95;
  dc_battery.eta_discharge = 0.95;
  dc_battery.in_service = true;
  sys.dc.dc_storage.push_back(dc_battery);

  VSCConverter converter;
  converter.index = 1;
  converter.name = "District-LV-to-Charging-Hub-VSC";
  converter.bus_ac = secondary_heads[4];
  converter.bus_dc = 1;
  converter.control_mode = ConverterMode::VDC_Q;
  converter.v_dc_set_pu = 1.0;
  converter.v_ac_set_pu = 1.0;
  converter.q_set_mvar = 0.0;
  converter.k_vdc = 25.0;
  converter.eta = 0.985;
  converter.p_rated_mw = 2.0;
  converter.pmax_mw = 2.0;
  converter.pmin_mw = -2.0;
  converter.qmax_mvar = 1.0;
  converter.qmin_mvar = -1.0;
  converter.vn_ac_kv = kSecondaryKv;
  converter.vn_dc_kv = 0.8;
  converter.r_sc_pu = 0.01;
  converter.x_sc_pu = 0.12;
  converter.i_max_pu = 1.2;
  converter.i_ac_max_pu = 1.1;
  converter.i_dc_max_pu = 1.1;
  converter.k_m_modulation = 1.0;
  converter.m_min = 0.05;
  converter.m_max = 1.0;
  converter.in_service = true;
  sys.vsc_converters.push_back(converter);

  Microgrid district_microgrid;
  district_microgrid.index = 1;
  district_microgrid.name = "Urban-District-Microgrid";
  district_microgrid.description =
      "Community PV, BESS, and DC charging district on feeder 1";
  district_microgrid.pcc_bus = primary_buses[4];
  district_microgrid.operating_mode = MicrogridMode::GridConnected;
  district_microgrid.islanding_capability = true;
  district_microgrid.auto_reconnection = true;
  district_microgrid.p_import_max_mw = 2.0;
  district_microgrid.p_export_max_mw = 0.5;
  district_microgrid.capacity_mw = 2.0;
  district_microgrid.in_service = true;
  for (int district = 0; district < kDistrictsPerFeeder; ++district) {
    district_microgrid.internal_buses.push_back(primary_buses[district]);
    district_microgrid.internal_buses.push_back(secondary_heads[district]);
    district_microgrid.internal_buses.insert(
        district_microgrid.internal_buses.end(), street_buses[district].begin(),
        street_buses[district].end());
  }
  sys.microgrids.push_back(district_microgrid);

  sys.three_phase_ac = std::move(phase);
  return sys;
}

}  // namespace hacdcpf::io
