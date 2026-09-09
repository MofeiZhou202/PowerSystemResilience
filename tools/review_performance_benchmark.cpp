// Built by review_performance_benchmark.py; timing hooks stay outside production.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>

#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

using Clock = std::chrono::steady_clock;
double padding_seconds = 0.0;
struct PaddingTimer {
  Clock::time_point start = Clock::now();
  ~PaddingTimer() {
    padding_seconds += std::chrono::duration<double>(Clock::now() - start).count();
  }
};
hacdcpf::analysis::CarbonAnalysisResult benchmark_snapshot;
Eigen::VectorXd captured_pcalc;
#include BENCH_SOURCE

using namespace hacdcpf;

double peak_rss_mb() {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
#ifdef __APPLE__
  return usage.ru_maxrss / 1e6;
#else
  return usage.ru_maxrss / 1e3;
#endif
}

int main(int argc, char** argv) {
  if (argc != 5) return 2;
  const int n = std::stoi(argv[1]);
  const int count = std::stoi(argv[2]);
  const int steps = std::stoi(argv[3]);
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  PowerFlowResult pf;
  pf.converged = true;
  for (int i = 1; i <= n; ++i) {
    ACBus bus;
    bus.index = i;
    bus.vm_pu = 1.0;
#ifdef BENCH_CARBON
    bus.bus_type = BusType::SLACK;
#else
    bus.bus_type = i == 1 ? BusType::SLACK : BusType::PQ;
    bus.gs_mw = 0.01;
    bus.bs_mvar = 0.02;
    bus.pd_mw = 0.1;
    bus.qd_mvar = 0.02;
    if (i > 1) {
      ACBranch branch;
      branch.index = i - 1;
      branch.from_bus = i - 1;
      branch.to_bus = i;
      branch.r_pu = 0.01;
      branch.x_pu = 0.05;
      branch.b_pu = 0.001;
      branch.tap = i % 7 == 0 ? 1.03 : 1.0;
      branch.shift_deg = i % 11 == 0 ? 2.0 : 0.0;
      sys.ac.branches.push_back(branch);
    }
#endif
    sys.ac.buses.push_back(bus);
    pf.vm.push_back(1.0);
    pf.va.push_back(0.0);
  }
  std::ofstream evidence(argv[4]);
  evidence << std::setprecision(17);
#ifdef BENCH_CARBON
  for (int i = 1; i <= count; ++i) {
    Load load;
    load.index = i;
    load.bus = (i - 1) % n + 1;
    load.p_mw = 1.0;
    sys.ac.loads.push_back(load);
    Generator gen;
    gen.index = i;
    gen.bus = load.bus;
    gen.pg_mw = 1.0;
    gen.pmax_mw = 10.0;
    gen.is_slack = true;
    gen.emission_factor_tco2_mwh = 0.8;
    sys.ac.generators.push_back(gen);
  }
  // Freeze an actually solved, verified state to isolate annual materialization.
  benchmark_snapshot = analysis::compute_carbon_analysis(sys, pf);
  if (!benchmark_snapshot.tracing_verified) return 3;
  analysis::AnnualCarbonAnalysisOptions options;
  options.keep_hourly_bus_intensity = true;
  options.keep_hourly_load_emissions = true;
  options.keep_hourly_load_energy = true;
  const std::vector<PowerFlowResult> samples(steps, pf);
  const auto start = Clock::now();
  const auto result = analysis::compute_annual_carbon_analysis(sys, samples, 1.0, options);
  const double wall = std::chrono::duration<double>(Clock::now() - start).count();
  const double rss = peak_rss_mb();
  evidence << analysis::annual_bus_carbon_hourly_to_json(result, -1)
           << analysis::annual_bus_carbon_summary_to_json(result, -1)
           << analysis::annual_load_carbon_hourly_to_json(result, -1)
           << analysis::annual_load_carbon_summary_to_json(result, -1);
  // Include raw energy and NaN masks, even if the public export omits them.
  for (const auto& row : result.hourly_load_energy_mwh) {
    for (double value : row) evidence << value << ',';
    evidence << '\n';
  }
  evidence << result.num_pf_converged << ',' << result.num_carbon_verified << ','
           << result.total_generation_emissions_tco2 << ',' << result.total_load_emissions_tco2
           << ',' << result.total_loss_emissions_tco2;
  std::cout << std::setprecision(17) << "{\"wall_s\":" << wall
            << ",\"padding_s\":" << padding_seconds << ",\"peak_rss_mb\":" << rss << "}\n";
#else
  if (count < 0) {
    sys.ac.buses[1].bus_type = BusType::PV;
    for (int i : {1, 2}) {
      Generator generator;
      generator.index = generator.bus = i;
      generator.is_slack = i == 1;
      generator.pg_mw = 0.15;
      generator.pmax_mw = 100.0;
      generator.pmin_mw = 0.0;
      sys.ac.generators.push_back(generator);
    }
    auto config = powerflow::create_participation_factors(sys);
    if (count == -2) config.max_participation_p[1] = 1e-12;
    PowerFlowOptions options;
    options.tol = 1e-8;
    options.max_iter = 100;
    const auto start = Clock::now();
    const auto solved = powerflow::DistributedSlackSolver().solve_full_jacobian(sys, config, options);
    const double wall = std::chrono::duration<double>(Clock::now() - start).count();
    evidence << solved.converged << ',' << solved.iterations << ',' << solved.residual << '\n';
    for (double v : solved.vm) evidence << v << ',';
    for (double v : solved.va) evidence << v << ',';
    for (int bus : solved.hit_limits) evidence << bus << ',';
    for (int bus : config.participating_buses) evidence << solved.distributed_slack_p.at(bus) << ',';
    std::cout << std::setprecision(17) << "{\"wall_s\":" << wall
              << ",\"peak_rss_mb\":" << peak_rss_mb() << "}\n";
    if (!solved.converged || (count == -2 &&
        std::find(solved.hit_limits.begin(), solved.hit_limits.end(), 1) == solved.hit_limits.end())) return 6;
    return evidence ? 0 : 5;
  }
  if (count) {
    DCBus dc;
    dc.index = 1;
    sys.dc.buses.push_back(dc);
    VSCConverter converter;
    converter.index = converter.bus_ac = converter.bus_dc = 1;
    converter.control_mode = ConverterMode::PQ_MODE;
    converter.p_set_mw = 0.2;
    converter.q_set_mvar = 0.1;
    sys.vsc_converters.push_back(converter);
    pf.vdc = {0.97};
  }
  auto data = powerflow::make_solver_data(sys);
  data.zip_pw[0] = 0.5; data.zip_pw[1] = 0.2; data.zip_pw[2] = 0.3;
  data.zip_qw[0] = 0.4; data.zip_qw[1] = 0.3; data.zip_qw[2] = 0.3;
  Eigen::VectorXcd voltage(n);
  for (int i = 0; i < n; ++i) {
    pf.vm[i] = 0.98 + 0.04 * std::sin(i * 0.017);
    pf.va[i] = 0.01 * std::cos(i * 0.03);
    voltage[i] = std::polar(pf.vm[i], pf.va[i]);
  }
  const auto start = Clock::now();
  const double mismatch = powerflow::compute_total_participating_mismatch_pu(data, pf);
  const double wall = std::chrono::duration<double>(Clock::now() - start).count();
  const double rss = peak_rss_mb();
  const Eigen::VectorXcd current = data.ybus * voltage;
  double max_error = 0.0;
  for (int i = 0; i < n; ++i) {
    const double expected = (voltage[i] * std::conj(current[i])).real();
    max_error = std::max(max_error, std::abs(captured_pcalc[i] - expected) /
                                   std::max(1.0, std::abs(expected)));
    evidence << captured_pcalc[i] << '\n';
  }
  evidence << mismatch;
  std::cout << std::setprecision(17) << "{\"wall_s\":" << wall
            << ",\"peak_rss_mb\":" << rss << ",\"oracle_error\":" << max_error
            << ",\"nnz\":" << data.ybus.nonZeros() << "}\n";
  if (max_error > 1e-10) return 4;
#endif
  return evidence ? 0 : 5;
}
