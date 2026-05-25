// =============================================================================
// three_stage_reliability.cpp
//
// C++ bridge that invokes the Julia three-stage MILP fault-recovery reliability
// engine (julia/reliability/) out-of-process and parses its JSON output.
//
// The implementation deliberately contains no optimisation logic — it is a
// thin process-launcher / JSON parser whose sole responsibility is:
//   1. Resolve paths (julia executable, project dir, cli script, workdir).
//   2. Build and invoke the shell command.
//   3. Parse the result JSON into ThreeStageReliabilityResult.
//   4. Handle all errors gracefully (never throw).
// =============================================================================

#include "hacdcpf/analysis/three_stage_reliability.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
using nlohmann::json;

namespace hacdcpf::analysis {
namespace {

// ---------------------------------------------------------------------------
// Project-root resolution.
// HACDCPF_PROJECT_ROOT is injected by the top-level CMakeLists.txt as a
// compile definition pointing to the source directory at configure time.
// ---------------------------------------------------------------------------
const char* project_root_default() {
#ifdef HACDCPF_PROJECT_ROOT
  return HACDCPF_PROJECT_ROOT;
#else
  return "";
#endif
}

std::string shell_quote(const fs::path& p) {
  std::string s = p.string();
#ifdef _WIN32
  std::replace(s.begin(), s.end(), '\\', '/');
#endif
  return std::string("\"") + s + "\"";
}

int run_shell_cmd(const std::string& cmd) {
#ifdef _WIN32
  std::string wrapped = "\"" + cmd + "\"";
  return std::system(wrapped.c_str());
#else
  return std::system(cmd.c_str());
#endif
}

fs::path make_unique_workdir() {
  auto base = fs::temp_directory_path();
  std::random_device rd;
  std::mt19937_64 rng(rd());
  for (int attempt = 0; attempt < 8; ++attempt) {
    std::ostringstream oss;
    oss << "hacdcpf_3stage_" << std::hex << rng();
    auto candidate = base / oss.str();
    std::error_code ec;
    if (fs::create_directories(candidate, ec) && !ec) {
      return candidate;
    }
  }
  auto fallback = base / "hacdcpf_3stage_fallback";
  std::error_code ec;
  fs::create_directories(fallback, ec);
  return fallback;
}

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------
std::vector<double> as_double_vector(const json& v) {
  std::vector<double> out;
  if (!v.is_array()) return out;
  out.reserve(v.size());
  for (const auto& x : v) {
    out.push_back(x.is_number() ? x.get<double>() : 0.0);
  }
  return out;
}

void parse_result_json(const json& doc, ThreeStageReliabilityResult& r) {
  // ── system-level metrics ────────────────────────────────────────────────
  if (doc.contains("metrics") && doc["metrics"].is_object()) {
    const auto& m = doc["metrics"];
    r.saifi        = m.value("saifi",        0.0);
    r.saidi_min    = m.value("saidi_min",    0.0);
    r.eens_kwh_yr  = m.value("eens_kwh_yr",  0.0);
    r.eens_cost    = m.value("eens_cost",    0.0);
    r.worst_line   = m.value("worst_line",   0);
  }

  // ── nodal indices ────────────────────────────────────────────────────────
  if (doc.contains("nodal") && doc["nodal"].is_object()) {
    const auto& n = doc["nodal"];
    if (n.contains("eens_kwh_yr")) r.nodal_eens_kwh_yr = as_double_vector(n["eens_kwh_yr"]);
    if (n.contains("cif"))         r.nodal_cif         = as_double_vector(n["cif"]);
    if (n.contains("cid_min"))     r.nodal_cid_min     = as_double_vector(n["cid_min"]);
  }

  // ── per-fault details ────────────────────────────────────────────────────
  if (doc.contains("fault_details") && doc["fault_details"].is_array()) {
    r.faults.reserve(doc["fault_details"].size());
    for (const auto& f : doc["fault_details"]) {
      ThreeStageFaultDetail d;
      d.line_id    = f.value("line",      0);
      d.status     = f.value("status",    std::string{"unknown"});
      d.objective  = f.value("objective", 0.0);
      d.pls_stage1 = f.value("pls_stage1", 0.0);
      d.pls_stage2 = f.value("pls_stage2", 0.0);
      d.pls_stage3 = f.value("pls_stage3", 0.0);
      d.pls_total  = f.value("pls_total",  0.0);
      if (f.contains("psop1")) d.psop1 = as_double_vector(f["psop1"]);
      if (f.contains("psop2")) d.psop2 = as_double_vector(f["psop2"]);
      if (f.contains("psop3")) d.psop3 = as_double_vector(f["psop3"]);
      r.faults.push_back(std::move(d));
    }
  }

  // ── SOP configuration ────────────────────────────────────────────────────
  if (doc.contains("sop_config") && doc["sop_config"].is_array()) {
    r.sop_config.reserve(doc["sop_config"].size());
    for (const auto& s : doc["sop_config"]) {
      ThreeStageSopConfig c;
      c.id         = s.value("id",         0);
      c.node_a     = s.value("node_a",     0);
      c.node_b     = s.value("node_b",     0);
      c.pmax_kw    = s.value("pmax_kw",    0.0);
      c.qmax_kw    = s.value("qmax_kw",    0.0);
      c.efficiency = s.value("efficiency", 0.0);
      r.sop_config.push_back(std::move(c));
    }
  }

  // ── network summary ───────────────────────────────────────────────────────
  if (doc.contains("summary") && doc["summary"].is_object()) {
    const auto& s = doc["summary"];
    r.nb     = s.value("nb",     0);
    r.nb_ac  = s.value("nb_ac",  0);
    r.nb_dc  = s.value("nb_dc",  0);
    r.nl     = s.value("nl",     0);
    r.nl_ac  = s.value("nl_ac",  0);
    r.nl_dc  = s.value("nl_dc",  0);
    r.nl_vsc = s.value("nl_vsc", 0);
    r.nl_sop = s.value("nl_sop", 0);
    r.nd     = s.value("nd",     0);
    r.ng     = s.value("ng",     0);
    r.nmg    = s.value("nmg",    0);
  }
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Public entry point: file-based
// ---------------------------------------------------------------------------
ThreeStageReliabilityResult run_three_stage_reliability(
    const fs::path& case_json,
    const ThreeStageReliabilityOptions& options) {
  ThreeStageReliabilityResult result;

  if (!fs::exists(case_json)) {
    result.error = "case JSON does not exist: " + case_json.string();
    return result;
  }

  // ── resolve julia executable ─────────────────────────────────────────────
  fs::path julia_exe = options.julia_executable;
  if (julia_exe.empty()) {
    julia_exe = "julia";
  }

  // ── resolve project dir + cli script ────────────────────────────────────
  fs::path julia_proj = options.julia_project_dir;
  if (julia_proj.empty()) {
    fs::path root = project_root_default();
    if (root.empty()) {
      result.error = "HACDCPF_PROJECT_ROOT not defined and "
                     "julia_project_dir not provided";
      return result;
    }
    julia_proj = root / "julia" / "reliability";
  }

  fs::path cli_script = options.cli_script;
  if (cli_script.empty()) {
    cli_script = julia_proj / "cli.jl";
  }

  if (!fs::exists(cli_script)) {
    result.error = "Julia CLI script not found: " + cli_script.string();
    return result;
  }

  // ── resolve work directory ───────────────────────────────────────────────
  bool auto_workdir = options.workdir.empty();
  fs::path workdir = auto_workdir ? make_unique_workdir() : options.workdir;
  std::error_code ec;
  fs::create_directories(workdir, ec);

  fs::path result_json = workdir / "three_stage_result.json";

  // ── build command ─────────────────────────────────────────────────────────
  std::ostringstream cmd;
  cmd << shell_quote(julia_exe)
      << " --project=" << shell_quote(julia_proj)
      << ' ' << shell_quote(cli_script)
      << ' ' << shell_quote(case_json)
      << ' ' << shell_quote(result_json);

  if (!options.inherit_stdio) {
#ifdef _WIN32
    cmd << " > NUL 2>&1";
#else
    cmd << " >/dev/null 2>&1";
#endif
  }

  spdlog::info("[three_stage_reliability] running: {}", cmd.str());

  int rc = run_shell_cmd(cmd.str());

  if (rc != 0) {
    result.error = "Julia CLI exited with code " + std::to_string(rc);
    if (!auto_workdir || options.keep_workdir) {
      result.result_json_path = result_json;
    } else {
      fs::remove_all(workdir, ec);
    }
    return result;
  }

  if (!fs::exists(result_json)) {
    result.error = "Julia CLI succeeded but result JSON not produced: "
                   + result_json.string();
    return result;
  }

  // ── parse output JSON ─────────────────────────────────────────────────────
  try {
    std::ifstream in(result_json);
    if (!in) {
      result.error = "failed to open result JSON: " + result_json.string();
      return result;
    }
    json doc;
    in >> doc;
    parse_result_json(doc, result);
    result.ok = true;
  } catch (const std::exception& e) {
    result.error = std::string("failed to parse result JSON: ") + e.what();
    return result;
  }

  // ── retain or remove workdir ─────────────────────────────────────────────
  if (options.keep_workdir || !auto_workdir) {
    result.result_json_path = result_json;
  } else {
    fs::remove_all(workdir, ec);
  }
  return result;
}

// ---------------------------------------------------------------------------
// Public entry point: string-based (convenience wrapper)
// ---------------------------------------------------------------------------
ThreeStageReliabilityResult run_three_stage_reliability_from_string(
    const std::string& case_json_text,
    const ThreeStageReliabilityOptions& options) {
  ThreeStageReliabilityResult result;
  bool auto_workdir = options.workdir.empty();
  fs::path workdir = auto_workdir ? make_unique_workdir() : options.workdir;
  std::error_code ec;
  fs::create_directories(workdir, ec);

  fs::path case_json = workdir / "three_stage_case.json";
  {
    std::ofstream out(case_json);
    if (!out) {
      result.error = "failed to write case JSON: " + case_json.string();
      if (auto_workdir && !options.keep_workdir) {
        fs::remove_all(workdir, ec);
      }
      return result;
    }
    out << case_json_text;
  }

  ThreeStageReliabilityOptions opts = options;
  if (opts.workdir.empty()) {
    opts.workdir = workdir;
  }
  result = run_three_stage_reliability(case_json, opts);
  if (auto_workdir && !options.keep_workdir) {
    fs::remove_all(workdir, ec);
  }
  return result;
}

}  // namespace hacdcpf::analysis
