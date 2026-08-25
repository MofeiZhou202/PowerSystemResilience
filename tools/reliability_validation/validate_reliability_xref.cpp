// Reliability parameter-resolver independent cross-validation: evidence emitter.
//
// Runs the production reliability parameter resolver (resolve_reliability_params)
// on a battery of deterministic failure-mode inputs that exercise every
// conversion branch (lambda+MTTR on an operating or calendar basis, legacy MTBF
// with either convention, explicit MTTF, forced-outage-rate with and without
// repair time, and active-on-demand), and dumps the raw inputs, the data policy
// and the resolved canonical parameters as JSON.  The companion oracle
// tools/reliability_validation/run_cross_validation.py does NOT link hacdcpf: it
// re-derives the Billinton & Allan alternating-renewal closed forms from the raw
// inputs and checks the resolved lambda, repair time, unavailability, MTTF, and
// calendar / active-equivalent frequencies.
//
// Usage: validate_reliability_xref <output.json>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/reliability/reliability_assessment.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf::analysis;

namespace {

std::string default_policy_name(ReliabilityDefaultPolicy p) {
  switch (p) {
    case ReliabilityDefaultPolicy::StrictCaseDataOnly: return "strict";
    case ReliabilityDefaultPolicy::UseNamedTemplateForMissingOnly: return "template_missing";
    case ReliabilityDefaultPolicy::OverwriteWithNamedTemplate: return "overwrite";
  }
  return "unknown";
}

std::string mtbf_convention_name(MtbfConvention c) {
  switch (c) {
    case MtbfConvention::MtbfAsMttf: return "mttf";
    case MtbfConvention::MtbfAsCycleTime: return "cycle";
    case MtbfConvention::Unspecified: return "unspecified";
  }
  return "unknown";
}

std::string failure_rate_basis_name(FailureRateBasis b) {
  return b == FailureRateBasis::CalendarTime ? "calendar" : "operating";
}

json raw_to_json(const ReliabilityRawFields& r) {
  return json{{"failure_rate_per_year", r.failure_rate_per_year},
              {"mttr_hr", r.mttr_hr},
              {"mtbf_hours", r.mtbf_hours},
              {"mttr_hours", r.mttr_hours},
              {"mttf_hours", r.mttf_hours},
              {"forced_outage_rate", r.forced_outage_rate},
              {"is_active", r.is_active},
              {"probability_per_demand", r.probability_per_demand},
              {"demand_frequency_per_year", r.demand_frequency_per_year},
              {"active_params_are_template", r.active_params_are_template},
              {"cyber_recovery_hr", r.cyber_recovery_hr}};
}

json policy_to_json(const ReliabilityDataPolicy& p) {
  return json{{"default_policy", default_policy_name(p.default_policy)},
              {"mtbf_convention", mtbf_convention_name(p.mtbf_convention)},
              {"failure_rate_basis", failure_rate_basis_name(p.failure_rate_basis)},
              {"hours_per_year", p.hours_per_year}};
}

json params_to_json(const ReliabilityParams& p) {
  return json{{"has_data", p.has_data},
              {"used_default", p.used_default},
              {"data_source", p.data_source},
              {"lambda_per_year", p.lambda_per_year},
              {"calendar_frequency_per_year", p.calendar_frequency_per_year},
              {"repair_hr", p.repair_hr},
              {"unavailability", p.unavailability},
              {"mttf_hr", p.mttf_hr},
              {"lambda_active_per_year", p.lambda_active_per_year}};
}

json emit_case(const std::string& name, const ReliabilityRawFields& raw,
               const ReliabilityDataPolicy& policy) {
  // default_lambda/default_repair = 0 so the template-defaulting path is never
  // taken; every case resolves purely from its own inputs (or reports missing).
  const ReliabilityParams resolved =
      resolve_reliability_params(raw, policy, 0.0, 0.0);
  return json{{"name", name},
              {"raw", raw_to_json(raw)},
              {"policy", policy_to_json(policy)},
              {"resolved", params_to_json(resolved)}};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: validate_reliability_xref <output.json>\n";
    return 2;
  }

  ReliabilityDataPolicy operating;  // defaults: template_missing, OperatingTime
  operating.failure_rate_basis = FailureRateBasis::OperatingTime;

  ReliabilityDataPolicy calendar;
  calendar.failure_rate_basis = FailureRateBasis::CalendarTime;

  ReliabilityDataPolicy mtbf_mttf;
  mtbf_mttf.mtbf_convention = MtbfConvention::MtbfAsMttf;

  ReliabilityDataPolicy mtbf_cycle;
  mtbf_cycle.mtbf_convention = MtbfConvention::MtbfAsCycleTime;

  ReliabilityDataPolicy strict;
  strict.default_policy = ReliabilityDefaultPolicy::StrictCaseDataOnly;

  json cases = json::array();

  // 1) lambda + MTTR, operating-time basis (ACBranch style).
  {
    ReliabilityRawFields r;
    r.failure_rate_per_year = 0.5;
    r.mttr_hr = 8.0;
    cases.push_back(emit_case("lambda_mttr_operating", r, operating));
  }
  // 2) lambda + MTTR, calendar basis (undoes downtime exposure).
  {
    ReliabilityRawFields r;
    r.failure_rate_per_year = 0.5;
    r.mttr_hr = 8.0;
    cases.push_back(emit_case("lambda_mttr_calendar", r, calendar));
  }
  // 3) legacy MTBF + MTTR, MTBF-as-MTTF convention.
  {
    ReliabilityRawFields r;
    r.mtbf_hours = 17520.0;
    r.mttr_hours = 10.0;
    cases.push_back(emit_case("mtbf_mttr_asmttf", r, mtbf_mttf));
  }
  // 4) legacy MTBF + MTTR, cycle-time convention (MTTF = MTBF - MTTR).
  {
    ReliabilityRawFields r;
    r.mtbf_hours = 17530.0;
    r.mttr_hours = 10.0;
    cases.push_back(emit_case("mtbf_mttr_cycletime", r, mtbf_cycle));
  }
  // 5) explicit MTTF + MTTR.
  {
    ReliabilityRawFields r;
    r.mttf_hours = 17520.0;
    r.mttr_hours = 6.0;
    cases.push_back(emit_case("mttf_mttr", r, operating));
  }
  // 6) forced-outage-rate + MTTR (Generator/VSC/Storage style).
  {
    ReliabilityRawFields r;
    r.forced_outage_rate = 0.02;
    r.mttr_hr = 48.0;
    cases.push_back(emit_case("for_mttr", r, operating));
  }
  // 7) active-on-demand with cyber recovery time overriding physical repair.
  {
    ReliabilityRawFields r;
    r.is_active = true;
    r.probability_per_demand = 0.001;
    r.demand_frequency_per_year = 200.0;
    r.cyber_recovery_hr = 2.0;
    cases.push_back(emit_case("active_on_demand", r, operating));
  }
  // 8) forced-outage-rate only under strict policy (unavailability known,
  //    frequency/duration undetermined, no invented default).
  {
    ReliabilityRawFields r;
    r.forced_outage_rate = 0.03;
    cases.push_back(emit_case("for_only_strict", r, strict));
  }

  json out;
  out["schema"] = "hysim-reliability-resolver-evidence-v1";
  out["cases"] = cases;

  const fs::path path(argv[1]);
  if (path.has_parent_path()) fs::create_directories(path.parent_path());
  std::ofstream os(path);
  if (!os) {
    std::cerr << "cannot open output: " << argv[1] << "\n";
    return 1;
  }
  os << out.dump(2) << "\n";
  return 0;
}
