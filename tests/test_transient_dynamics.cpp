#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

HybridPowerSystem make_transient_2bus() {
  HybridPowerSystem sys;
  sys.name = "transient_2bus";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.02;
  b1.va_deg = 0.0;
  b1.in_service = true;

  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.in_service = true;

  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.02;
  g.pg_mw = 40.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  g.xdpp_pu = 0.20;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 30.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};

  return sys;
}

HybridPowerSystem make_hybrid_dc_case() {
  auto sys = make_transient_2bus();

  DCBus d1;
  d1.index = 1;
  d1.bus_type = DCBusType::DC_V;
  d1.vm_pu = 1.0;
  d1.in_service = true;
  DCBus d2;
  d2.index = 2;
  d2.bus_type = DCBusType::DC_P;
  d2.vm_pu = 1.0;
  d2.in_service = true;
  sys.dc.buses = {d1, d2};

  DCBranch dcbr;
  dcbr.index = 1;
  dcbr.from_bus = 1;
  dcbr.to_bus = 2;
  dcbr.r_pu = 0.02;
  dcbr.in_service = true;
  sys.dc.branches = {dcbr};

  DCLoad dcl;
  dcl.index = 1;
  dcl.bus = 2;
  dcl.p_mw = 5.0;
  dcl.in_service = true;
  sys.dc.loads = {dcl};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.bus_dc = 1;
  vsc.in_service = true;
  vsc.control_mode = ConverterMode::PQ_MODE;
  vsc.p_set_mw = 8.0;
  vsc.p_schedule_mw = 8.0;
  vsc.q_set_mvar = 1.0;
  vsc.eta = 0.98;
  sys.vsc_converters = {vsc};

  DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 1;
  dcdc.bus_out = 2;
  dcdc.in_service = true;
  dcdc.p_ref_mw = 4.0;
  dcdc.eta = 0.97;
  sys.dc.dcdc_converters = {dcdc};

  return sys;
}

HybridPowerSystem make_asymmetric_ac_case() {
  auto sys = make_transient_2bus();
  sys.name = "transient_asymmetric_ac";
  sys.ac.loads.clear();

  AsymmetricLoad load;
  load.index = 1;
  load.bus = 2;
  load.name = "LV unbalance";
  load.pa_mw = 15.0;
  load.qa_mvar = 4.0;
  load.pb_mw = 6.0;
  load.qb_mvar = 2.0;
  load.pc_mw = 3.0;
  load.qc_mvar = 1.0;
  load.scaling = 1.0;
  load.in_service = true;
  sys.ac.asymmetric_loads = {load};

  return sys;
}

HybridPowerSystem make_psd_genrou_three_bus_subset_case() {
  HybridPowerSystem sys;
  sys.name = "psd_genrou_three_bus_subset";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 60.0;

  ACBus b101;
  b101.index = 101;
  b101.name = "BUS 1";
  b101.bus_type = BusType::SLACK;
  b101.base_kv = 138.0;
  b101.vm_pu = 1.05;
  b101.va_deg = 0.0;
  b101.in_service = true;

  ACBus b102;
  b102.index = 102;
  b102.name = "BUS 2";
  b102.bus_type = BusType::PV;
  b102.base_kv = 138.0;
  b102.vm_pu = 1.02;
  b102.va_deg = -0.9440;
  b102.in_service = true;

  ACBus b103;
  b103.index = 103;
  b103.name = "BUS 3";
  b103.bus_type = BusType::PQ;
  b103.base_kv = 138.0;
  b103.vm_pu = 0.99341;
  b103.va_deg = -8.7697;
  b103.in_service = true;

  sys.ac.buses = {b101, b102, b103};

  auto branch = [](int index, int from, int to) {
    ACBranch br;
    br.index = index;
    br.from_bus = from;
    br.to_bus = to;
    br.r_pu = 0.01;
    br.x_pu = 0.12;
    br.tap = 1.0;
    br.in_service = true;
    br.name = "BUS " + std::to_string(from - 100) + "-BUS " +
              std::to_string(to - 100) + "-i_1";
    return br;
  };
  sys.ac.branches = {
      branch(1, 101, 102),
      branch(2, 101, 103),
      branch(3, 102, 103),
  };

  Generator slack;
  slack.index = 1;
  slack.bus = 101;
  slack.name = "generator-101-1";
  slack.is_slack = true;
  slack.in_service = true;
  slack.pg_mw = 153.335;
  slack.qg_mvar = 73.271;
  slack.vg_pu = 1.05;
  slack.pmax_mw = 318.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  slack.xdpp_pu = 1.0e-5;
  slack.dynamic_model.standard = "PSS/E";
  slack.dynamic_model.model_name = "GENCLS";
  slack.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";

  Generator genrou;
  genrou.index = 2;
  genrou.bus = 102;
  genrou.name = "generator-102-1";
  genrou.is_slack = false;
  genrou.in_service = true;
  genrou.pg_mw = 100.0;
  genrou.qg_mvar = -3.247;
  genrou.vg_pu = 1.02;
  genrou.pmax_mw = 318.0;
  genrou.pmin_mw = 0.0;
  genrou.qmax_mvar = 100.0;
  genrou.qmin_mvar = -100.0;
  genrou.ra_pu = 0.0;
  genrou.xd_pu = 1.8;
  genrou.xq_pu = 1.7;
  genrou.xdp_pu = 0.30;
  genrou.xdpp_pu = 0.25;
  genrou.td0p_s = 8.0;
  genrou.td0pp_s = 0.03;
  genrou.inertia_h = 6.175;
  genrou.droop_r = 0.05;
  genrou.dynamic_model.standard = "PSS/E";
  genrou.dynamic_model.model_name = "GENROU";
  genrou.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";
  genrou.dynamic_model.parameters = {
      {"H", 6.175},
      {"D", 0.05},
      {"Xd", 1.8},
      {"Xq", 1.7},
      {"Xd_p", 0.30},
      {"Xq_p", 0.55},
      {"Xd_pp", 0.25},
      {"Xl", 0.20},
      {"Td0_p", 8.0},
      {"Td0_pp", 0.03},
      {"Tq0_p", 0.4},
      {"Tq0_pp", 0.05},
  };

  sys.ac.generators = {slack, genrou};

  Load load;
  load.index = 1;
  load.bus = 103;
  load.name = "load1031";
  load.p_mw = 250.0;
  load.q_mvar = 30.0;
  load.in_service = true;
  load.dynamic_model.standard = "PSS/E";
  load.dynamic_model.model_name = "ConstantImpedanceLoad";
  load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";
  sys.ac.loads = {load};

  return sys;
}

HybridPowerSystem make_psd_zip_load_three_bus_subset_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_zip_load_three_bus_subset";
  sys.ac.loads.clear();

  Load l102;
  l102.index = 1;
  l102.bus = 102;
  l102.name = "load1021";
  l102.p_mw = 50.0;
  l102.q_mvar = 30.0;
  l102.in_service = true;
  l102.dynamic_model.standard = "PSS/E";
  l102.dynamic_model.model_name = "ConstantPowerLoad";
  l102.dynamic_model.source_id = "PowerSimulationsDynamics:test_case33_zip_load";

  Load l103a;
  l103a.index = 2;
  l103a.bus = 103;
  l103a.name = "load1031";
  l103a.p_mw = 150.0;
  l103a.q_mvar = 30.0;
  l103a.in_service = true;
  l103a.dynamic_model = l102.dynamic_model;

  Load l103b;
  l103b.index = 3;
  l103b.bus = 103;
  l103b.name = "load1032";
  l103b.p_mw = 50.0;
  l103b.q_mvar = 30.0;
  l103b.in_service = true;
  l103b.dynamic_model = l102.dynamic_model;

  sys.ac.loads = {l102, l103a, l103b};
  return sys;
}

HybridPowerSystem make_unbalanced_three_phase_case() {
  HybridPowerSystem sys;
  sys.name = "transient_three_phase";
  sys.base_mva = 10.0;

  ThreePhaseACSystem tp;
  tp.base_mva = 10.0;
  tp.base_freq_hz = 50.0;

  ThreePhaseACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.phase_mask = PhaseMask::abc();
  b1.in_service = true;

  ThreePhaseACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.phase_mask = PhaseMask::abc();
  b2.in_service = true;

  tp.buses = {b1, b2};

  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.phase_mask = PhaseMask::abc();
  line.r1_pu = 0.03;
  line.x1_pu = 0.08;
  line.r0_pu = 0.05;
  line.x0_pu = 0.12;
  line.in_service = true;
  tp.lines = {line};

  ThreePhaseExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.vm_pu = 1.0;
  grid.va_deg = 0.0;
  grid.x1_pu = 0.02;
  grid.in_service = true;
  tp.external_grids = {grid};

  ThreePhaseLoad load;
  load.index = 1;
  load.bus = 2;
  load.phase_mask = PhaseMask::abc();
  load.p_a_mw = 0.50;
  load.q_a_mvar = 0.10;
  load.p_b_mw = 0.20;
  load.q_b_mvar = 0.05;
  load.p_c_mw = 0.10;
  load.q_c_mvar = 0.02;
  load.in_service = true;
  tp.loads = {load};

  sys.three_phase_ac = tp;
  return sys;
}

DynamicSolverOptions fast_options() {
  DynamicSolverOptions opt;
  opt.t_start_s = 0.0;
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.run_power_flow_initialization = false;
  opt.singular_regularization_pu = 1e-7;
  return opt;
}

double vsc_p_mw(const DynamicSnapshot& snapshot) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count("p_mw") == 1);
  return it->values.at("p_mw");
}

double vsc_value(const DynamicSnapshot& snapshot, const std::string& key) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count(key) == 1);
  return it->values.at(key);
}

double final_vsc_p_mw(const DynamicResults& result) {
  REQUIRE(result.final_snapshot() != nullptr);
  return vsc_p_mw(*result.final_snapshot());
}

struct CsvSeries {
  std::vector<double> t;
  std::vector<double> y;
};

std::string shell_quote(const std::string& value) {
  std::string out = "'";
  for (char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out += ch;
    }
  }
  out += "'";
  return out;
}

std::filesystem::path psd_repo_path() {
  if (const char* env = std::getenv("HACDCPF_PSD_REPO")) {
    return env;
  }
  return "/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl";
}

std::string julia_bin() {
  if (const char* env = std::getenv("HACDCPF_JULIA_BIN")) {
    return env;
  }
  return "julia";
}

CsvSeries read_csv_series(const std::filesystem::path& path) {
  CsvSeries series;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    double t = 0.0;
    double y = 0.0;
    if (iss >> t >> y) {
      series.t.push_back(t);
      series.y.push_back(y);
    }
  }
  return series;
}

void write_csv_series(const std::filesystem::path& path,
                      const CsvSeries& series) {
  REQUIRE(series.t.size() == series.y.size());
  std::ofstream out(path);
  REQUIRE(out.good());
  for (std::size_t i = 0; i < series.t.size(); ++i) {
    out << series.t[i] << "," << series.y[i] << "\n";
  }
}

double interpolate_series(const CsvSeries& series, double t) {
  REQUIRE(series.t.size() >= 2);
  if (t <= series.t.front()) return series.y.front();
  if (t >= series.t.back()) return series.y.back();
  const auto hi = std::lower_bound(series.t.begin(), series.t.end(), t);
  const std::size_t i = static_cast<std::size_t>(std::distance(series.t.begin(), hi));
  REQUIRE(i > 0);
  const double t0 = series.t[i - 1];
  const double t1 = series.t[i];
  const double y0 = series.y[i - 1];
  const double y1 = series.y[i];
  const double a = (t - t0) / std::max(1e-12, t1 - t0);
  return y0 + a * (y1 - y0);
}

double rms_common_error(const CsvSeries& lhs,
                        const CsvSeries& rhs,
                        double t_start,
                        double t_end) {
  double sum_sq = 0.0;
  std::size_t n = 0;
  for (double t : lhs.t) {
    if (t < t_start - 1e-12 || t > t_end + 1e-12) continue;
    const double e = interpolate_series(lhs, t) - interpolate_series(rhs, t);
    sum_sq += e * e;
    ++n;
  }
  REQUIRE(n > 0);
  return std::sqrt(sum_sq / static_cast<double>(n));
}

double max_common_abs_error(const CsvSeries& lhs,
                            const CsvSeries& rhs,
                            double t_start,
                            double t_end) {
  double max_abs = 0.0;
  std::size_t n = 0;
  for (double t : lhs.t) {
    if (t < t_start - 1e-12 || t > t_end + 1e-12) continue;
    const double e = interpolate_series(lhs, t) - interpolate_series(rhs, t);
    max_abs = std::max(max_abs, std::abs(e));
    ++n;
  }
  REQUIRE(n > 0);
  return max_abs;
}

const DynamicDeviceOutput& require_device_output(const DynamicSnapshot& snapshot,
                                                 const std::string& type,
                                                 int component_index = 0) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [&](const DynamicDeviceOutput& out) {
    return out.type == type &&
           (component_index == 0 || out.component_index == component_index);
  });
  REQUIRE(it != outputs.end());
  return *it;
}

CsvSeries device_output_series(const DynamicResults& result,
                               const std::string& type,
                               int component_index,
                               const std::string& key,
                               double scale = 1.0) {
  CsvSeries series;
  for (const auto& snapshot : result.snapshots) {
    const auto& out = require_device_output(snapshot, type, component_index);
    REQUIRE(out.values.count(key) == 1);
    series.t.push_back(snapshot.time_s);
    series.y.push_back(out.values.at(key) * scale);
  }
  return series;
}

CsvSeries bus_voltage_mag_series(const DynamicResults& result, int bus_position) {
  CsvSeries series;
  for (const auto& snapshot : result.snapshots) {
    const int node = 3 * bus_position;
    REQUIRE(node >= 0);
    REQUIRE(node < snapshot.vac_abc.size());
    series.t.push_back(snapshot.time_s);
    series.y.push_back(std::abs(snapshot.vac_abc[node]));
  }
  return series;
}

double series_range(const CsvSeries& series) {
  REQUIRE_FALSE(series.y.empty());
  const auto [min_it, max_it] = std::minmax_element(series.y.begin(), series.y.end());
  return *max_it - *min_it;
}

struct ValidationMetric {
  std::string level;
  std::string case_name;
  std::string reference;
  std::string signal;
  double rms_error{0.0};
  double max_abs_error{0.0};
  double tolerance{0.0};
  bool passed{false};
};

void write_validation_summary(const std::filesystem::path& path,
                              const std::vector<ValidationMetric>& metrics) {
  std::ofstream out(path);
  REQUIRE(out.good());
  out << "level,case,reference,signal,rms_error,max_abs_error,tolerance,passed\n";
  for (const auto& metric : metrics) {
    out << metric.level << ","
        << metric.case_name << ","
        << metric.reference << ","
        << metric.signal << ","
        << metric.rms_error << ","
        << metric.max_abs_error << ","
        << metric.tolerance << ","
        << (metric.passed ? "true" : "false") << "\n";
  }
}

struct PsdGflCase {
  std::string name;
  std::string psd_case;
  std::string psd_test_file;
  std::string pll_model;
  double pll_kp{0.0};
  double pll_ki{0.0};
  double pll_lpf_t_s{0.0};
};

DynamicResults run_hacdcpf_psd_gfl_case(const PsdGflCase& spec,
                                        double base_mva) {
  auto sys = make_hybrid_dc_case();
  sys.base_mva = base_mva;
  sys.ac.base_mva = base_mva;
  sys.vsc_converters[0].p_set_mw = 50.0;
  sys.vsc_converters[0].p_schedule_mw = 50.0;
  sys.vsc_converters[0].q_set_mvar = 0.0;
  sys.vsc_converters[0].dynamic_model.standard = "NERC";
  sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  sys.vsc_converters[0].dynamic_model.components.push_back(
      {"pll", spec.pll_model, "PowerSimulationsDynamics", spec.psd_case,
       {{"kp_pll", spec.pll_kp},
        {"ki_pll", spec.pll_ki},
        {"pll_lpf_t_s", spec.pll_lpf_t_s}}});

  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent pref_step;
  pref_step.time_s = 1.0;
  pref_step.type = DynamicEventType::Custom;
  pref_step.component_type = "VSC";
  pref_step.component_index = 1;
  pref_step.label = "PSD active-power reference step";
  pref_step.params["p_ref_mw"] = 70.0;
  dyn.events.push_back(pref_step);

  DynamicSolver solver;
  return solver.solve(dyn);
}

CsvSeries hacdcpf_vsc_filtered_power_series(const DynamicResults& result,
                                            double base_mva) {
  CsvSeries local;
  for (const auto& snapshot : result.snapshots) {
    local.t.push_back(snapshot.time_s);
    local.y.push_back(vsc_value(snapshot, "p_filtered_mw") / base_mva);
  }
  return local;
}

bool export_psd_trace(const std::string& psd_case,
                      const std::string& signal,
                      const std::filesystem::path& out_csv,
                      const std::filesystem::path& log_file) {
  const std::filesystem::path repo = psd_repo_path();
  const std::filesystem::path script =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "tools" / "psd_validation" / "export_trace.jl";

  INFO("PSD repo: " << repo);
  INFO("PSD generic exporter: " << script);
  INFO("PSD case: " << psd_case << " signal: " << signal);
  INFO("PSD output CSV: " << out_csv);
  INFO("PSD log: " << log_file);
  REQUIRE(std::filesystem::exists(repo / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / "Project.toml"));
  REQUIRE(std::filesystem::exists(script));

  const std::string command =
      "cd " + shell_quote(repo.string()) + " && " +
      shell_quote(julia_bin()) + " --project=" +
      shell_quote((repo / "test").string()) + " " +
      shell_quote(script.string()) + " " +
      shell_quote(repo.string()) + " " +
      shell_quote(psd_case) + " " +
      shell_quote(out_csv.string()) + " " +
      shell_quote(signal) + " > " +
      shell_quote(log_file.string()) + " 2>&1";
  return std::system(command.c_str()) == 0;
}

bool export_psd_gridfollowing_trace(const PsdGflCase& spec,
                                    const std::filesystem::path& out_csv,
                                    const std::filesystem::path& log_file) {
  const std::filesystem::path repo = psd_repo_path();
  const std::filesystem::path script =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "tools" / "psd_validation" / "export_gridfollowing_trace.jl";

  INFO("PSD repo: " << repo);
  INFO("PSD exporter: " << script);
  INFO("PSD output CSV: " << out_csv);
  INFO("PSD log: " << log_file);
  REQUIRE(std::filesystem::exists(repo / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / spec.psd_test_file));
  REQUIRE(std::filesystem::exists(script));

  const std::string command =
      "cd " + shell_quote(repo.string()) + " && " +
      shell_quote(julia_bin()) + " --project=" +
      shell_quote((repo / "test").string()) + " " +
      shell_quote(script.string()) + " " +
      shell_quote(repo.string()) + " " +
      shell_quote(spec.psd_case) + " " +
      shell_quote(out_csv.string()) + " p_oc > " +
      shell_quote(log_file.string()) + " 2>&1";
  return std::system(command.c_str()) == 0;
}

}  // namespace

TEST_CASE("DynamicModelBuilder creates canonical transient system", "[dynamics][transient]") {
  const auto sys = make_transient_2bus();
  const auto opt = fast_options();

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  CHECK(dyn.network.ac_bus_ids.size() == 2);
  CHECK(dyn.network.acPhaseNodeCount() == 6);
  CHECK(dyn.network.ac_branches.size() == 1);
  CHECK(dyn.devices.size() >= 2);
  CHECK(dyn.stateCount() > 0);
  CHECK(dyn.y.Vac_abc.size() == 6);
}

TEST_CASE("Transient solver runs partitioned phasor dynamics", "[dynamics][transient]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.solver_type = DynamicSolverType::PartitionedHeun;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.steps == 5);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->max_ac_voltage_pu > 0.1);
  CHECK(result.final_snapshot()->min_ac_voltage_pu > 0.1);
}

TEST_CASE("Transient events preserve branch type and apply topology changes", "[dynamics][events]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.t_end_s = 0.03;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent trip;
  trip.time_s = 0.01;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 1;
  trip.component_type = "AC";
  trip.label = "trip AC branch 1";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE_FALSE(result.applied_events.empty());
  CHECK(result.applied_events.front() == "trip AC branch 1");
  REQUIRE_FALSE(dyn.network.ac_branches.empty());
  CHECK_FALSE(dyn.network.ac_branches.front().in_service);
}

TEST_CASE("Transient solver lands on off-grid event times", "[dynamics][events]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.dt_s = 0.01;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent event;
  event.time_s = 0.015;
  event.type = DynamicEventType::ACLoadScale;
  event.component_index = 1;
  event.value = 0.5;
  event.label = "mid-step load scale";
  dyn.events.push_back(event);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE_FALSE(result.applied_events.empty());
  CHECK(result.applied_events.front() == "mid-step load scale");
  bool saw_event_time = false;
  for (const auto& snapshot : result.snapshots) {
    saw_event_time = saw_event_time || std::abs(snapshot.time_s - 0.015) < 1e-12;
  }
  CHECK(saw_event_time);
}

TEST_CASE("Canonical transient builder keeps asymmetric loads per phase once", "[dynamics][three_phase]") {
  const auto sys = make_asymmetric_ac_case();
  auto opt = fast_options();
  opt.project_to_canonical = true;
  opt.t_end_s = 0.02;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  int three_phase_loads = 0;
  int balanced_loads = 0;
  for (const auto& device : dyn.devices) {
    if (device->type() == "ThreePhaseLoad") ++three_phase_loads;
    if (device->type() == "ACLoad") ++balanced_loads;
  }
  CHECK(three_phase_loads == 1);
  CHECK(balanced_loads == 0);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& v = result.final_snapshot()->vac_abc;
  REQUIRE(v.size() >= 6);
  const double va = std::abs(v[3]);
  const double vb = std::abs(v[4]);
  const double vc = std::abs(v[5]);
  CHECK(std::max({va, vb, vc}) - std::min({va, vb, vc}) > 1e-4);
}

TEST_CASE("Hybrid AC/DC transient includes VSC and DC/DC coupling", "[dynamics][hybrid_acdc]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->vdc.size() == 2);
  CHECK(result.final_snapshot()->max_dc_voltage_pu > 0.1);
  CHECK(result.final_snapshot()->vdc[0] == Catch::Approx(1.0).margin(5e-3));
}

TEST_CASE("Transient VSC roles separate AC and DC grid forming", "[dynamics][hybrid_acdc]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].grid_forming = true;
  sys.vsc_converters[0].ac_grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::PQ_MODE;
  sys.vsc_converters[0].v_dc_set_pu = 1.03;
  auto opt = fast_options();

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_dc_source = false;
  bool saw_ac_gfm = false;
  for (const auto& device : dyn.devices) {
    saw_dc_source = saw_dc_source || device->type() == "DCVoltageSource";
    saw_ac_gfm = saw_ac_gfm || device->type() == "VSCGridForming";
  }
  CHECK(saw_dc_source);
  CHECK_FALSE(saw_ac_gfm);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->vdc[0] == Catch::Approx(1.03).margin(5e-3));
}

TEST_CASE("Explicit three-phase transient model keeps unbalanced phase loads", "[dynamics][three_phase]") {
  const auto sys = make_unbalanced_three_phase_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_three_phase_load = false;
  for (const auto& device : dyn.devices) {
    saw_three_phase_load = saw_three_phase_load || device->type() == "ThreePhaseLoad";
  }
  CHECK(saw_three_phase_load);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& v = result.final_snapshot()->vac_abc;
  REQUIRE(v.size() >= 6);
  const double va = std::abs(v[3]);
  const double vb = std::abs(v[4]);
  const double vc = std::abs(v[5]);
  const double vmax = std::max({va, vb, vc});
  const double vmin = std::min({va, vb, vc});
  CHECK(vmax - vmin > 1e-4);
}

TEST_CASE("Updated GFL inverter exposes PLL current-limited positive-sequence telemetry",
          "[dynamics][gfl]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].i_ac_max_pu = 0.04;
  sys.vsc_converters[0].q_set_mvar = 6.0;
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;
  opt.solver_type = DynamicSolverType::PartitionedRK4;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("pll_frequency_hz") > 49.0);
  CHECK(it->values.at("pll_frequency_hz") < 51.0);
  CHECK(it->values.at("i_mag_pu") <= 0.045);
  CHECK(it->values.at("current_limit_active") == Catch::Approx(1.0));
  CHECK(std::abs(it->values.at("p_mw")) > 0.1);
}

TEST_CASE("GFL inverter supports KauraPLL profile and exposes dynamic model metadata",
          "[dynamics][gfl][pll][profile]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].dynamic_model.standard = "NERC";
  sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  sys.vsc_converters[0].dynamic_model.parameter_set = "kaura_demo";
  sys.vsc_converters[0].dynamic_model.components.push_back(
      {"pll", "KauraPLL", "PSD", "default", {{"kp_pll", 0.015},
                                               {"ki_pll", 1.1},
                                               {"pll_lpf_t_s", 0.004}}});
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count("pll_model") == 1);
  CHECK(it->values.at("pll_model") == Catch::Approx(1.0));
  CHECK(it->values.count("pll_vq_raw_pu") == 1);
  CHECK(it->values.count("pll_vd_pu") == 1);
  REQUIRE(it->model_profiles.size() >= 2);
  CHECK(it->model_profiles.front().model_name == "REGC_REEC_GFL_Subset");
  CHECK(std::any_of(it->model_profiles.begin(),
                    it->model_profiles.end(),
                    [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                      return profile.profile == "pll" &&
                             profile.model_name == "KauraPLL";
                    }));
}

TEST_CASE("Transient initialization uses solved power flow and exposes canvas metadata",
          "[dynamics][initialization][gui]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.01;
  opt.dt_s = 0.01;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_requested);
  CHECK(result.initialization.power_flow_converged);
  CHECK_FALSE(result.initialization.fallback_voltage_setpoints);
  CHECK(result.initialization.max_ac_voltage_pu > 0.9);
  CHECK(result.initialization.dynamic_trim_iterations > 0);
  CHECK(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto vsc = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(vsc != outputs.end());
  CHECK(vsc->canvas_type == "vsc");
  CHECK(vsc->canvas_index == 1);
  CHECK(vsc->component_domain == "AC");
  CHECK(vsc->source_type == "vsc_grid_following");
}

TEST_CASE("Transient initialization trims GFL fast states to avoid artificial PLL settling",
          "[dynamics][initialization][trim]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.record_every_step = true;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() >= 2);
  CHECK(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);

  auto pll_freq = [](const DynamicSnapshot& snapshot) {
    const auto it = std::find_if(snapshot.device_outputs.begin(),
                                 snapshot.device_outputs.end(),
                                 [](const DynamicDeviceOutput& out) {
                                   return out.type == "VSCGridFollowing";
                                 });
    REQUIRE(it != snapshot.device_outputs.end());
    REQUIRE(it->values.count("pll_frequency_hz") == 1);
    return it->values.at("pll_frequency_hz");
  };
  CHECK(pll_freq(result.snapshots.front()) == Catch::Approx(pll_freq(result.snapshots.back())).margin(1e-4));
}

TEST_CASE("No-event dynamic equilibrium residual is a hard benchmark gate",
          "[dynamics][benchmark][equilibrium]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
  REQUIRE(result.snapshots.size() >= 2);
  const double p0 = vsc_p_mw(result.snapshots.front());
  const double p1 = final_vsc_p_mw(result);
  CHECK(p1 == Catch::Approx(p0).margin(5e-4));
}

TEST_CASE("PSD validation ladder covers component, load, and system-level HACDCPF anchors",
          "[dynamics][benchmark][psd][validation]") {
  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_validation";
  std::filesystem::create_directories(out_dir);
  std::vector<ValidationMetric> metrics;

  {
    auto sys = make_psd_genrou_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = "PSD GENROU fixture branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    CHECK(result.initialization.dynamic_trim_converged);
    CHECK(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
    REQUIRE_FALSE(result.applied_event_records.empty());
    REQUIRE(result.snapshots.size() > 100);

    const auto rotor = device_output_series(result,
                                            "SynchronousMachine",
                                            2,
                                            "angle_rad");
    const auto omega = device_output_series(result,
                                            "SynchronousMachine",
                                            2,
                                            "omega_pu");
    write_csv_series(out_dir / "hacdcpf_component_genrou_angle_rad.csv", rotor);
    write_csv_series(out_dir / "hacdcpf_component_genrou_omega_pu.csv", omega);

    CHECK(series_range(rotor) > 1e-4);
    CHECK(series_range(omega) > 1e-7);

    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
    CHECK(gen_out.model_standard == "IEEE");
    REQUIRE(gen_out.model_profiles.size() >= 1);
    CHECK(std::any_of(gen_out.model_profiles.begin(),
                      gen_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.standard == "PSS/E" &&
                               profile.model_name == "GENROU";
                      }));

    metrics.push_back({"component",
                       "genrou_three_bus_subset",
                       "HACDCPF",
                       "no_event_initial_dxdt_inf",
                       result.initialization.dynamic_fast_dxdt_inf_norm,
                       result.initialization.dynamic_fast_dxdt_inf_norm,
                       opt.dynamic_trim_tol,
                       result.initialization.dynamic_fast_dxdt_inf_norm <=
                           opt.dynamic_trim_tol});
  }

  {
    auto sys = make_psd_zip_load_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = "PSD ZIP fixture branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    REQUIRE(result.snapshots.size() > 100);
    const auto v102 = bus_voltage_mag_series(result, 1);
    const auto v103 = bus_voltage_mag_series(result, 2);
    write_csv_series(out_dir / "hacdcpf_component_zip_v102.csv", v102);
    write_csv_series(out_dir / "hacdcpf_component_zip_v103.csv", v103);

    CHECK(series_range(v102) > 1e-5);
    CHECK(series_range(v103) > 1e-5);

    const auto& load_out = require_device_output(*result.final_snapshot(), "ACLoad", 1);
    REQUIRE(load_out.model_profiles.size() >= 1);
    CHECK(std::any_of(load_out.model_profiles.begin(),
                      load_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.standard == "PSS/E" &&
                               profile.model_name == "ConstantPowerLoad";
                      }));

    metrics.push_back({"component",
                       "zip_load_three_bus_subset",
                       "HACDCPF",
                       "voltage_response_range_bus103",
                       series_range(v103),
                       series_range(v103),
                       1e-5,
                       series_range(v103) > 1e-5});
  }

  {
    auto sys = make_hybrid_dc_case();
    sys.ac.generators[0].dynamic_model.standard = "IEEE";
    sys.ac.generators[0].dynamic_model.model_name = "ClassicalMachine";
    sys.vsc_converters[0].dynamic_model.standard = "NERC";
    sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
    sys.vsc_converters[0].dynamic_model.components.push_back(
        {"pll", "FixedFrequency", "PowerSimulationsDynamics", "system_anchor", {}});

    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent pref_step;
    pref_step.time_s = 1.0;
    pref_step.type = DynamicEventType::Custom;
    pref_step.component_type = "VSC";
    pref_step.component_index = 1;
    pref_step.label = "system-level VSC reference step";
    pref_step.params["p_ref_mw"] = 12.0;
    dyn.events.push_back(pref_step);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
    REQUIRE(result.snapshots.size() > 100);
    const auto p_vsc = device_output_series(result,
                                            "VSCGridFollowing",
                                            1,
                                            "p_filtered_mw");
    const auto f_vsc = device_output_series(result,
                                            "VSCGridFollowing",
                                            1,
                                            "pll_frequency_hz");
    write_csv_series(out_dir / "hacdcpf_system_hybrid_vsc_p_filtered_mw.csv", p_vsc);
    write_csv_series(out_dir / "hacdcpf_system_hybrid_vsc_pll_frequency_hz.csv", f_vsc);

    CHECK(series_range(p_vsc) > 0.5);
    CHECK(*std::min_element(f_vsc.y.begin(), f_vsc.y.end()) > 45.0);
    CHECK(*std::max_element(f_vsc.y.begin(), f_vsc.y.end()) < 55.0);

    metrics.push_back({"system",
                       "hybrid_acdc_vsc_step",
                       "HACDCPF",
                       "p_filtered_response_range_mw",
                       series_range(p_vsc),
                       series_range(p_vsc),
                       0.5,
                       series_range(p_vsc) > 0.5});
  }

  write_validation_summary(out_dir / "hacdcpf_psd_validation_summary.csv", metrics);
  for (const auto& metric : metrics) {
    INFO("Validation artifact directory: " << out_dir);
    INFO(metric.level << " " << metric.case_name << " " << metric.signal);
    CHECK(metric.passed);
  }
}

TEST_CASE("Opt-in PSD external comparisons cover generator, load, and system traces",
          "[dynamics][benchmark][psd][external]") {
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  if (!run_external_psd) {
    SUCCEED("Set HACDCPF_RUN_PSD_COMPARE=1 to run Julia-backed PSD trace comparisons");
    return;
  }

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_validation";
  std::filesystem::create_directories(out_dir);

  struct ExternalSpec {
    std::string case_name;
    std::string signal;
    CsvSeries local;
    double t_start{0.0};
    double t_end{2.0};
    double rms_tolerance{1.0};
    std::string level;
  };

  std::vector<ExternalSpec> specs;

  {
    auto sys = make_psd_genrou_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    REQUIRE(result.success);
    auto local = device_output_series(result,
                                      "SynchronousMachine",
                                      2,
                                      "angle_rad",
                                      180.0 / 3.14159265358979323846);
    write_csv_series(out_dir / "hacdcpf_psd_genrou_delta_deg.csv", local);
    specs.push_back({"genrou",
                     "generator-102-1:delta_deg",
                     std::move(local),
                     0.0,
                     2.0,
                     360.0,
                     "component"});
  }

  {
    auto sys = make_psd_zip_load_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    REQUIRE(result.success);
    auto local = bus_voltage_mag_series(result, 2);
    write_csv_series(out_dir / "hacdcpf_psd_zip_constant_power_v103.csv", local);
    specs.push_back({"zip_constant_power",
                     "bus103:voltage_mag",
                     std::move(local),
                     0.0,
                     2.0,
                     0.25,
                     "component"});
  }

  {
    const PsdGflCase psd_system_case{
        "reduced_pll_test24",
        "test24",
        "test_case_gridfollowing.jl",
        "ReducedOrderPLL",
        2.0,
        20.0,
        1.0 / (1.32 * 2.0 * 3.14159265358979323846 * 50.0)};
    const double base_mva = 100.0;
    const DynamicResults result =
        run_hacdcpf_psd_gfl_case(psd_system_case, base_mva);
    REQUIRE(result.success);
    auto local = hacdcpf_vsc_filtered_power_series(result, base_mva);
    write_csv_series(out_dir / "hacdcpf_psd_gridfollowing_reduced_p_oc.csv", local);
    specs.push_back({"test24",
                     "generator-102-1:p_oc",
                     std::move(local),
                     0.0,
                     2.0,
                     0.35,
                     "system"});
  }

  std::vector<ValidationMetric> metrics;
  for (const auto& spec : specs) {
    const std::filesystem::path psd_csv =
        out_dir / ("psd_" + spec.case_name + "_" +
                   spec.signal.substr(spec.signal.find(':') + 1) + ".csv");
    const std::filesystem::path psd_log =
        out_dir / ("psd_" + spec.case_name + ".log");
    const bool exported =
        export_psd_trace(spec.case_name, spec.signal, psd_csv, psd_log);
    INFO("PSD export failed. Inspect " << psd_log
         << ". Run `cd " << psd_repo_path()
         << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
         << "to install missing PSD test dependencies.");
    REQUIRE(exported);

    const CsvSeries psd = read_csv_series(psd_csv);
    REQUIRE(psd.t.size() > 100);
    const double rms = rms_common_error(spec.local, psd, spec.t_start, spec.t_end);
    const double max_abs =
        max_common_abs_error(spec.local, psd, spec.t_start, spec.t_end);
    metrics.push_back({spec.level,
                       spec.case_name,
                       "PowerSimulationsDynamics.jl",
                       spec.signal,
                       rms,
                       max_abs,
                       spec.rms_tolerance,
                       rms < spec.rms_tolerance});
  }
  write_validation_summary(out_dir / "psd_external_validation_summary.csv", metrics);
  for (const auto& metric : metrics) {
    INFO("PSD validation artifact directory: " << out_dir);
    INFO(metric.level << " " << metric.case_name << " " << metric.signal
                      << " rms=" << metric.rms_error
                      << " max=" << metric.max_abs_error);
    CHECK(metric.passed);
  }
}

TEST_CASE("PSD grid-following comparison harness is available",
          "[dynamics][benchmark][psd][gridfollowing]") {
  const std::vector<PsdGflCase> cases = {
      {"reduced_pll_test24",
       "test24",
       "test_case_gridfollowing.jl",
       "ReducedOrderPLL",
       2.0,
       20.0,
       1.0 / (1.32 * 2.0 * 3.14159265358979323846 * 50.0)},
      {"kaura_pll_test51",
       "test51",
       "test_case51_gridfollowing_kaura.jl",
       "KauraPLL",
       0.084,
       4.69,
       1.0 / 500.0},
  };

  const double base_mva = 100.0;
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_gridfollowing";
  std::filesystem::create_directories(out_dir);

  for (const auto& spec : cases) {
    INFO("PSD validation case: " << spec.name);
    const DynamicResults result = run_hacdcpf_psd_gfl_case(spec, base_mva);
    REQUIRE(result.success);
    REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);
    REQUIRE(result.snapshots.size() > 100);
    REQUIRE(result.final_snapshot() != nullptr);
    const double p_final = final_vsc_p_mw(result);
    CHECK(std::isfinite(p_final));
    const double p_filtered_final =
        vsc_value(*result.final_snapshot(), "p_filtered_mw");
    CHECK(p_filtered_final > 40.0);
    CHECK(p_filtered_final < 80.0);

    const CsvSeries local =
        hacdcpf_vsc_filtered_power_series(result, base_mva);
    const std::filesystem::path hacdcpf_csv =
        out_dir / ("hacdcpf_" + spec.name + "_p_oc.csv");
    write_csv_series(hacdcpf_csv, local);

    if (!run_external_psd) continue;

    const std::filesystem::path psd_csv =
        out_dir / ("psd_" + spec.name + "_p_oc.csv");
    const std::filesystem::path psd_log =
        out_dir / ("psd_" + spec.name + ".log");
    const bool exported =
        export_psd_gridfollowing_trace(spec, psd_csv, psd_log);
    INFO("PSD export failed. Inspect " << psd_log
         << ". Run `cd " << psd_repo_path()
         << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
         << "to install missing PSD test dependencies.");
    REQUIRE(exported);
    const CsvSeries psd = read_csv_series(psd_csv);
    REQUIRE(psd.t.size() > 100);
    const double rms = rms_common_error(local, psd, 0.0, 2.0);
    CHECK(rms < 0.35);
  }
}

TEST_CASE("Updated GFL inverter can use dynamic DC-link voltage state",
          "[dynamics][gfl][dc_link]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].v_dc_set_pu = 1.02;
  auto opt = fast_options();
  opt.dynamic_dc_link = true;
  opt.dc_link_capacitance_s = 0.20;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_dynamic_vsc = false;
  for (const auto& device : dyn.devices) {
    if (device->type() == "VSCGridFollowing") {
      const auto out = device->output(dyn.x, dyn.y);
      saw_dynamic_vsc = out.values.count("vdc_link_pu") == 1 &&
                        out.values.at("dc_link_dynamic") == Catch::Approx(1.0);
    }
  }
  CHECK(saw_dynamic_vsc);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("vdc_link_pu") > 0.2);
  CHECK(it->values.at("vdc_link_pu") < 2.0);
  CHECK(it->values.at("dc_link_dynamic") == Catch::Approx(1.0));
}

TEST_CASE("Updated GFM inverter stamps Norton voltage source and droop telemetry",
          "[dynamics][gfm]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].ac_grid_forming = true;
  sys.vsc_converters[0].grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::AC_GRID_FORMING;
  sys.vsc_converters[0].pmax_mw = 12.0;
  sys.vsc_converters[0].i_ac_max_pu = 0.25;
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridForming";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("e_internal_pu") > 0.2);
  CHECK(it->values.at("frequency_hz") > 45.0);
  CHECK(it->values.at("frequency_hz") < 55.0);
  CHECK(it->values.at("i_rms_pu") >= 0.0);
  CHECK(it->values.count("p_filtered_mw") == 1);
}

TEST_CASE("Updated GFM inverter exposes dynamic DC-link telemetry",
          "[dynamics][gfm][dc_link]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].ac_grid_forming = true;
  sys.vsc_converters[0].grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::AC_GRID_FORMING;
  sys.vsc_converters[0].v_dc_set_pu = 1.01;
  auto opt = fast_options();
  opt.dynamic_dc_link = true;
  opt.dc_link_capacitance_s = 0.20;
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridForming";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("vdc_link_pu") > 0.2);
  CHECK(it->values.at("vdc_link_pu") < 2.0);
  CHECK(it->values.at("dc_link_dynamic") == Catch::Approx(1.0));
  CHECK(it->values.count("p_dc_mw") == 1);
}

TEST_CASE("Implicit transient Newton solvers run without Heun fallback", "[dynamics][solver]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  opt.solver_type = DynamicSolverType::BackwardEulerNewton;
  opt.newton_tol = 1e-7;
  opt.max_newton_iters = 12;

  const DynamicResults be = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(be.success);
  CHECK(be.steps == 2);
  CHECK(be.newton_iterations > 0);
  CHECK(std::none_of(be.warnings.begin(), be.warnings.end(), [](const std::string& warning) {
    return warning.find("PartitionedHeun was used") != std::string::npos;
  }));

  opt.solver_type = DynamicSolverType::TrapezoidalNewton;
  const DynamicResults trap = hacdcpf::run_transient_simulation(sys, opt);
  REQUIRE(trap.success);
  CHECK(trap.newton_iterations > 0);
}

TEST_CASE("Documented transient device headers expose executable standard profiles",
          "[dynamics][devices][standards]") {
  DynamicState x;
  NetworkState y;
  PowerFlowResult pf;
  pf.va = {0.0};
  y.resize(3, 0);
  y.Vac_abc[0] = std::polar(1.0, 0.0);
  y.Vac_abc[1] = std::polar(1.0, -2.0 * 3.14159265358979323846 / 3.0);
  y.Vac_abc[2] = std::polar(1.0, 2.0 * 3.14159265358979323846 / 3.0);

  GovernorDynamicParams governor_params;
  governor_params.component_index = 7;
  governor_params.base_mva = 100.0;
  governor_params.p_ref_mw = 40.0;
  Governor governor(governor_params);
  ExciterDynamicParams exciter_params;
  exciter_params.component_index = 8;
  exciter_params.bus = 1;
  exciter_params.bus_pos = 0;
  Exciter exciter(exciter_params);
  PVDynamicParams pv_params;
  pv_params.component_index = 9;
  pv_params.bus = 1;
  pv_params.bus_pos = 0;
  pv_params.p_ref_mw = 1.5;
  pv_params.q_ref_mvar = 0.2;
  PVDynamic pv(pv_params);
  ProtectionRelayParams relay_params;
  relay_params.component_index = 10;
  relay_params.bus = 1;
  relay_params.bus_pos = 0;
  ProtectionRelay relay(relay_params);

  std::vector<DynamicDevice*> devices{&governor, &exciter, &pv, &relay};
  int offset = 0;
  for (auto* device : devices) device->assignStateIndices(offset);
  x.resize(static_cast<std::size_t>(offset));
  for (auto* device : devices) device->initializeFromPowerFlow(pf, x, y);

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(x.size());
  for (auto* device : devices) device->computeDerivatives(0.0, x, y, dxdt);
  CHECK(dxdt.allFinite());

  DynamicStamp stamp(3, 0);
  pv.stamp(0.0, x, y, stamp);
  CHECK(stamp.Iac.norm() > 0.0);

  CHECK(governor.output(x, y).model_standard == "IEEE");
  CHECK(exciter.output(x, y).model_standard == "IEEE4215");
  CHECK(pv.output(x, y).model_standard == "IEEE1547");
  CHECK(relay.output(x, y).model_name == "VoltageFrequencyRelay");
}

TEST_CASE("Documented integrators and algebraic solvers advance dynamic systems",
          "[dynamics][integration][solver]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.01;
  opt.dt_s = 0.01;
  opt.solver_type = DynamicSolverType::PartitionedRK4;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  AlgebraicNetworkSolver algebraic;
  const auto algebraic_result = algebraic.solve(dyn, opt.t_start_s);
  REQUIRE(algebraic_result.success);
  CHECK(dyn.y.Vac_abc.size() > 0);

  RK4 rk4;
  const auto rk4_step = rk4.step(dyn, opt.t_start_s, opt.dt_s);
  REQUIRE(rk4_step.success);
  CHECK(rk4_step.derivative_evaluations == 4);
  CHECK(dyn.x.dxdt.size() == dyn.x.x.size());

  DynamicSystem dyn_be = builder.build(sys, opt);
  BackwardEuler be;
  DaeSolver dae(be);
  const auto dae_step = dae.step(dyn_be, opt.t_start_s, opt.dt_s);
  REQUIRE(dae_step.success);
  CHECK(dae_step.nonlinear_iterations > 0);
  CHECK(dyn_be.x.time_s == Catch::Approx(opt.dt_s));

  Eigen::SparseMatrix<double> a(2, 2);
  a.insert(0, 0) = 4.0;
  a.insert(1, 1) = 2.0;
  a.makeCompressed();
  Eigen::VectorXd b(2);
  b << 8.0, 6.0;
  Eigen::VectorXd solved;
  SparseLinearSolver sparse;
  const auto solve_result = sparse.solve(a, b, solved);
  REQUIRE(solve_result.success);
  CHECK(solved[0] == Catch::Approx(2.0));
  CHECK(solved[1] == Catch::Approx(3.0));
}

TEST_CASE("Transient results support sampled recording and CSV export",
          "[dynamics][results][sampling]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.output_every_steps = 2;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() == 4);
  CHECK(result.snapshots.front().time_s == Catch::Approx(0.0));
  CHECK(result.snapshots.back().time_s == Catch::Approx(0.05));

  const std::string csv = to_csv(result);
  CHECK(csv.find("time_s") != std::string::npos);
  CHECK(csv.find("vac_0_mag_pu") != std::string::npos);
  CHECK(csv.find("VSCGridFollowing_1_p_mw") != std::string::npos);
}

TEST_CASE("Transient contingencies carry named parameters and structured records",
          "[dynamics][events][params]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.dt_s = 0.01;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicEvent fault;
  fault.time_s = 0.01;
  fault.type = DynamicEventType::FaultShunt;
  fault.bus = 2;
  fault.component_type = "AC";
  fault.label = "parameterized AC fault";
  fault.params["r_pu"] = 0.001;
  fault.params["x_pu"] = 0.002;
  fault.params["duration_s"] = 0.01;
  dyn.events.push_back(fault);

  DynamicEvent storage;
  storage.time_s = 0.02;
  storage.type = DynamicEventType::StoragePowerStep;
  storage.component_index = 1;
  storage.label = "storage dispatch";
  storage.params["p_ref_mw"] = -2.0;
  dyn.events.push_back(storage);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE(result.applied_event_records.size() == 2);
  CHECK(result.applied_event_records.front().label == "parameterized AC fault");
  CHECK(result.applied_event_records.front().params.at("r_pu") == Catch::Approx(0.001));
  CHECK(result.applied_event_records.back().params.at("p_ref_mw") == Catch::Approx(-2.0));
}

TEST_CASE("Dynamic sparse linear solver accepts optional backend requests",
          "[dynamics][solver][linear]") {
  Eigen::SparseMatrix<double> a(2, 2);
  a.insert(0, 0) = 3.0;
  a.insert(1, 1) = 5.0;
  a.makeCompressed();
  Eigen::VectorXd b(2);
  b << 6.0, 20.0;

  for (const auto type : {DynamicLinearSolverType::EigenSparseLU,
                          DynamicLinearSolverType::EigenBiCGSTAB,
                          DynamicLinearSolverType::KLU,
                          DynamicLinearSolverType::UMFPACK}) {
    Eigen::VectorXd x;
    SparseLinearSolver solver(type);
    const auto result = solver.solve(a, b, x);
    REQUIRE(result.success);
    CHECK(x[0] == Catch::Approx(2.0));
    CHECK(x[1] == Catch::Approx(4.0));
  }
}
