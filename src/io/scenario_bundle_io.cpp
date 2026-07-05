#include "hacdcpf/io/scenario_bundle_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#ifdef HACDCPF_ENABLE_ETAP
#  ifdef __clang__
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wdefaulted-function-deleted"
#  endif
#  include <OpenXLSX.hpp>
#  ifdef __clang__
#    pragma clang diagnostic pop
#  endif
#endif

namespace hacdcpf::io {
namespace {

std::string upper_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return s;
}

void report_issue(ScenarioBundleIoReport& report,
                  ScenarioBundleImportMode mode,
                  const std::string& message) {
  if (mode == ScenarioBundleImportMode::Strict) report.errors.push_back(message);
  else report.warnings.push_back(message);
}

nlohmann::json as_array_or_empty(const nlohmann::json& value) {
  return value.is_array() ? value : nlohmann::json::array();
}

nlohmann::json normalize_case_shape(const nlohmann::json& item, int index) {
  if (item.contains("system") || item.contains("generated_scenario") || item.contains("standard_time_series")) {
    nlohmann::json out = item;
    if (!out.contains("case_id")) {
      const auto meta = out.value("generated_scenario", nlohmann::json::object());
      out["case_id"] = meta.value("representative_id", meta.value("scenario_id", "case_" + std::to_string(index + 1)));
    }
    return out;
  }

  nlohmann::json system = item;
  nlohmann::json meta = system.value("_generated_scenario", nlohmann::json::object());
  nlohmann::json ts = system.value("_time_series", nlohmann::json::object());
  system.erase("_generated_scenario");
  system.erase("_time_series");
  return {{"case_id", meta.value("representative_id", meta.value("scenario_id", "case_" + std::to_string(index + 1)))},
          {"system", system},
          {"generated_scenario", meta},
          {"standard_time_series", ts},
          {"_generated_scenario", meta},
          {"_time_series", ts}};
}

bool finite_number(const nlohmann::json& v) {
  return v.is_number() && std::isfinite(v.get<double>());
}

#ifdef HACDCPF_ENABLE_ETAP
using OpenXLSX::XLDocument;
using OpenXLSX::XLWorkbook;
using OpenXLSX::XLWorksheet;

std::string cell_to_string(const XLWorksheet& ws, uint32_t row, uint16_t col) {
  auto cell = ws.findCell(row, col);
  if (cell.empty()) return "";
  OpenXLSX::XLCellValue v = cell.value();
  if (v.type() == OpenXLSX::XLValueType::Float) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v.get<double>());
    return std::string(buf);
  }
  return v.getString();
}

using ColMap = std::unordered_map<std::string, uint16_t>;

ColMap colmap(const XLWorksheet& ws) {
  ColMap out;
  for (uint16_t c = 1; c <= ws.columnCount(); ++c) {
    const auto key = upper_copy(cell_to_string(ws, 1, c));
    if (!key.empty()) out.emplace(key, c);
  }
  return out;
}

std::string by_name(const XLWorksheet& ws, uint32_t row, const ColMap& cols, const std::string& name) {
  const auto it = cols.find(upper_copy(name));
  return it == cols.end() ? std::string() : cell_to_string(ws, row, it->second);
}

void headers(XLWorksheet ws, const std::vector<std::string>& values) {
  for (uint16_t c = 0; c < values.size(); ++c) ws.cell(1, static_cast<uint16_t>(c + 1)).value() = values[c];
}

XLWorksheet ensure_sheet(XLWorkbook& wb, const std::string& name) {
  if (!wb.worksheetExists(name)) wb.addWorksheet(name);
  return wb.worksheet(name);
}

void write_metadata(XLWorksheet ws, const nlohmann::json& bundle) {
  headers(ws, {"key", "value"});
  const std::vector<std::pair<std::string, std::string>> rows = {
      {"workbook_schema", "HACDCPF_SCENARIO_WORKBOOK_V1"},
      {"json_format", bundle.value("format", "generated_scenario_case_bundle_v3")},
      {"schema_version", std::to_string(bundle.value("schema_version", 3))},
      {"unit_space", bundle.value("unit_space", "dimensionless_multiplier")},
      {"family", bundle.value("family", "mixed")},
      {"case_count", std::to_string(static_cast<int>(bundle.value("cases", nlohmann::json::array()).size()))},
      {"exported_at", bundle.value("exported_at", "")}};
  for (std::size_t i = 0; i < rows.size(); ++i) {
    ws.cell(static_cast<uint32_t>(i + 2), 1).value() = rows[i].first;
    ws.cell(static_cast<uint32_t>(i + 2), 2).value() = rows[i].second;
  }
}

std::vector<std::string> profile_columns(const nlohmann::json& profiles) {
  std::vector<std::string> out;
  for (const auto& p : as_array_or_empty(profiles)) {
    out.push_back("p" + std::to_string(p.value("id", static_cast<int>(out.size()))) + "_" + p.value("name", "profile"));
  }
  return out;
}

bool service_on(const nlohmann::json& item) {
  return !item.contains("in_service") || item.value("in_service", true);
}

double first_positive(const nlohmann::json& item, std::initializer_list<const char*> keys) {
  for (const char* key : keys) {
    const double value = item.value(key, 0.0);
    if (value > 1e-9) return value;
  }
  return 0.0;
}

std::string capacity_field(const nlohmann::json& item, std::initializer_list<const char*> keys) {
  for (const char* key : keys) {
    if (item.contains(key) && item.value(key, 0.0) > 1e-9) return key;
  }
  return *keys.begin();
}

bool text_contains(const nlohmann::json& item, const std::string& token) {
  const auto hay = upper_copy(item.value("type", std::string()) + " " +
                             item.value("sgen_type", std::string()) + " " +
                             item.value("name", std::string()));
  return hay.find(upper_copy(token)) != std::string::npos;
}

void write_device_row(XLWorksheet ws,
                      uint32_t& row,
                      const std::string& case_id,
                      const std::string& kind,
                      const nlohmann::json& item,
                      std::size_t position,
                      double base_mw,
                      const std::string& field,
                      double base_mwh = 0.0) {
  if (base_mw <= 1e-9 && base_mwh <= 1e-9) return;
  ws.cell(row, 1).value() = case_id;
  ws.cell(row, 2).value() = kind;
  ws.cell(row, 3).value() = item.value("index", static_cast<int>(position + 1));
  ws.cell(row, 4).value() = static_cast<int>(position);
  ws.cell(row, 5).value() = item.value("bus", 0);
  ws.cell(row, 6).value() = item.value("name", "");
  ws.cell(row, 7).value() = base_mw;
  ws.cell(row, 8).value() = item.value("q_mvar", 0.0);
  ws.cell(row, 9).value() = base_mwh;
  ws.cell(row, 10).value() = field;
  ws.cell(row, 11).value() = service_on(item) ? "TRUE" : "FALSE";
  ws.cell(row, 12).value() = item.value("profile_id", -1);
  ws.cell(row, 13).value() = "TRUE";
  ++row;
}

void write_device_capacities(XLWorksheet ws, const std::string& case_id, const nlohmann::json& system, uint32_t& row) {
  const auto ac = system.value("ac", nlohmann::json::object());
  const auto dc = system.value("dc", nlohmann::json::object());
  auto write_array = [&](const nlohmann::json& arr, const std::string& kind, std::initializer_list<const char*> keys) {
    int pos = 0;
    for (const auto& item : as_array_or_empty(arr)) {
      if (service_on(item)) write_device_row(ws, row, case_id, kind, item, static_cast<std::size_t>(pos), first_positive(item, keys), capacity_field(item, keys));
      ++pos;
    }
  };
  write_array(ac.value("loads", nlohmann::json::array()), "AC_LOAD", {"p_mw"});
  write_array(dc.value("loads", nlohmann::json::array()), "DC_LOAD", {"p_mw", "p_rated_mw"});
  write_array(ac.value("pv_systems", nlohmann::json::array()), "AC_PV_SYSTEM", {"pmax_mw", "p_mw", "sn_mva"});
  write_array(dc.value("pv_arrays", nlohmann::json::array()), "DC_PV_ARRAY", {"p_set_mw"});
  int pos = 0;
  for (const auto& item : as_array_or_empty(ac.value("renewable_gens", nlohmann::json::array()))) {
    const std::string kind = text_contains(item, "wind") ? "AC_WIND" : "AC_RENEWABLE";
    if (service_on(item)) write_device_row(ws, row, case_id, kind, item, static_cast<std::size_t>(pos), first_positive(item, {"p_rated_mw", "p_mw"}), capacity_field(item, {"p_rated_mw", "p_mw"}));
    ++pos;
  }
  pos = 0;
  for (const auto& item : as_array_or_empty(ac.value("static_generators", nlohmann::json::array()))) {
    if (text_contains(item, "pv") || text_contains(item, "solar") || text_contains(item, "wind")) {
      const std::string kind = text_contains(item, "wind") ? "AC_STATIC_WIND" : "AC_STATIC_PV";
      if (service_on(item)) write_device_row(ws, row, case_id, kind, item, static_cast<std::size_t>(pos), first_positive(item, {"p_rated_mw", "pmax_mw", "p_mw"}), capacity_field(item, {"p_rated_mw", "pmax_mw", "p_mw"}));
    }
    ++pos;
  }
  pos = 0;
  for (const auto& item : as_array_or_empty(dc.value("dc_static_generators", nlohmann::json::array()))) {
    if (text_contains(item, "pv") || text_contains(item, "solar") || text_contains(item, "wind")) {
      const std::string kind = text_contains(item, "wind") ? "DC_STATIC_WIND" : "DC_STATIC_PV";
      if (service_on(item)) write_device_row(ws, row, case_id, kind, item, static_cast<std::size_t>(pos), first_positive(item, {"p_set_mw"}), capacity_field(item, {"p_set_mw"}));
    }
    ++pos;
  }
  pos = 0;
  for (const auto& item : as_array_or_empty(ac.value("storage", nlohmann::json::array()))) {
    if (service_on(item)) write_device_row(ws, row, case_id, "AC_STORAGE", item, static_cast<std::size_t>(pos), first_positive(item, {"p_rated_mw", "pmax_mw", "p_mw"}), "p_rated_mw", item.value("e_rated_mwh", 0.0));
    ++pos;
  }
}

nlohmann::json* find_device(nlohmann::json& system, const std::string& kind, int index, int position) {
  auto& ac = system["ac"];
  auto& dc = system["dc"];
  nlohmann::json* arr = nullptr;
  if (kind == "AC_LOAD") arr = &ac["loads"];
  else if (kind == "DC_LOAD") arr = &dc["loads"];
  else if (kind == "AC_PV_SYSTEM") arr = &ac["pv_systems"];
  else if (kind == "DC_PV_ARRAY") arr = &dc["pv_arrays"];
  else if (kind == "AC_WIND" || kind == "AC_RENEWABLE") arr = &ac["renewable_gens"];
  else if (kind == "AC_STATIC_WIND" || kind == "AC_STATIC_PV") arr = &ac["static_generators"];
  else if (kind == "DC_STATIC_WIND" || kind == "DC_STATIC_PV") arr = &dc["dc_static_generators"];
  else if (kind == "AC_STORAGE") arr = &ac["storage"];
  if (!arr || !arr->is_array()) return nullptr;
  for (auto& item : *arr) if (item.value("index", -999999) == index) return &item;
  if (position >= 0 && position < static_cast<int>(arr->size())) return &(*arr)[static_cast<std::size_t>(position)];
  return nullptr;
}

void apply_device_capacity_row(nlohmann::json& system, const std::string& kind, int index, int position, const std::string& field, double base_mw, double base_mwh) {
  if (auto* item = find_device(system, kind, index, position)) {
    if (!field.empty() && base_mw >= 0.0) (*item)[field] = base_mw;
    if (base_mwh >= 0.0 && item->contains("e_rated_mwh")) (*item)["e_rated_mwh"] = base_mwh;
  }
}
#endif

}  // namespace

nlohmann::json normalize_generated_scenario_bundle(const nlohmann::json& input,
                                                   ScenarioBundleIoReport& report) {
  if (!input.is_object()) throw std::runtime_error("Scenario bundle must be a JSON object");
  if (input.value("format", "") == "generated_scenario_case_bundle_v3") return input;

  if (!input.contains("cases") || !input["cases"].is_array()) {
    if (input.contains("ac") || input.contains("dc")) {
      nlohmann::json single = input;
      nlohmann::json meta = single.value("_generated_scenario", nlohmann::json::object());
      nlohmann::json ts = single.value("_time_series", nlohmann::json::object());
      single.erase("_generated_scenario");
      single.erase("_time_series");
      return {{"format", "generated_scenario_case_bundle_v3"},
              {"schema_version", 3},
              {"unit_space", "dimensionless_multiplier"},
              {"family", meta.value("family", "unknown")},
              {"case_count", 1},
              {"cases", nlohmann::json::array({{{"case_id", meta.value("representative_id", "case_1")},
                                                 {"system", single},
                                                 {"generated_scenario", meta},
                                                 {"standard_time_series", ts},
                                                 {"_generated_scenario", meta},
                                                 {"_time_series", ts}}})}};
    }
    throw std::runtime_error("Scenario bundle does not contain cases[]");
  }

  nlohmann::json cases = nlohmann::json::array();
  int idx = 0;
  for (const auto& item : input["cases"]) cases.push_back(normalize_case_shape(item, idx++));
  nlohmann::json out = input;
  out["format"] = "generated_scenario_case_bundle_v3";
  out["schema_version"] = 3;
  out["unit_space"] = "dimensionless_multiplier";
  out["case_format"] = "hacdcpf_system_json_with_generated_scenario_metadata_v2";
  out["case_count"] = static_cast<int>(cases.size());
  out["cases"] = std::move(cases);
  report.warnings.push_back("Upgraded generated scenario bundle to v3 dimensionless multiplier schema.");
  return out;
}

nlohmann::json validate_scenario_bundle(const nlohmann::json& bundle,
                                         ScenarioBundleImportMode mode,
                                         ScenarioBundleIoReport& report) {
  if (!bundle.is_object()) throw std::runtime_error("Scenario bundle must be an object");
  if (bundle.value("unit_space", "") != "dimensionless_multiplier") {
    report_issue(report, mode, "unit_space must be dimensionless_multiplier");
  }
  const auto cases = as_array_or_empty(bundle.value("cases", nlohmann::json::array()));
  if (cases.empty()) report_issue(report, mode, "cases[] is empty");

  for (const auto& c : cases) {
    const std::string case_id = c.value("case_id", "<unknown>");
    const auto ts = c.value("standard_time_series", c.value("_time_series", nlohmann::json::object()));
    if (!ts.is_object()) continue;
    const int num_steps = ts.value("num_steps", 0);
    std::set<int> profile_ids;
    for (const auto& p : as_array_or_empty(ts.value("profiles", nlohmann::json::array()))) {
      const int id = p.value("id", -1);
      if (!profile_ids.insert(id).second) report_issue(report, mode, case_id + ": duplicate profile id " + std::to_string(id));
      const auto values = as_array_or_empty(p.value("values", nlohmann::json::array()));
      if (num_steps > 0 && static_cast<int>(values.size()) != num_steps) {
        report_issue(report, mode, case_id + ": profile " + std::to_string(id) + " length does not match num_steps");
      }
      const std::string unit = p.value("unit", "multiplier");
      for (const auto& v : values) {
        if (!finite_number(v)) {
          report_issue(report, mode, case_id + ": profile " + std::to_string(id) + " contains non-finite value");
          break;
        }
        const double x = v.get<double>();
        if (unit == "soc_fraction" && (x < -1e-9 || x > 1.0 + 1e-9)) {
          report_issue(report, mode, case_id + ": SOC profile outside [0,1]");
          break;
        }
        if (unit != "soc_fraction" && x < -1e-9) {
          report_issue(report, mode, case_id + ": multiplier profile contains negative value");
          break;
        }
      }
    }
    const auto binding = ts.value("binding", ts.value("bindings", nlohmann::json::object()));
    for (const auto& row : as_array_or_empty(binding.value("load_profile_map", nlohmann::json::array()))) {
      const int pid = row.value("profile_id", -999999);
      if (!profile_ids.count(pid)) report_issue(report, mode, case_id + ": binding references missing profile " + std::to_string(pid));
    }
  }

  if (!report.errors.empty()) throw std::runtime_error(report.errors.front());
  return bundle;
}

#ifdef HACDCPF_ENABLE_ETAP

void save_scenario_workbook(const nlohmann::json& scenario_bundle,
                            const std::string& path,
                            ScenarioBundleIoReport& report) {
  auto bundle = normalize_generated_scenario_bundle(scenario_bundle, report);
  validate_scenario_bundle(bundle, ScenarioBundleImportMode::Permissive, report);

  XLDocument doc;
  doc.create(path, OpenXLSX::XLForceOverwrite);
  XLWorkbook wb = doc.workbook();
  if (wb.worksheetExists("Sheet1")) wb.worksheet("Sheet1").setName("METADATA");
  write_metadata(wb.worksheet("METADATA"), bundle);

  auto wsScen = ensure_sheet(wb, "SCENARIOS");
  headers(wsScen, {"case_id", "scenario_id", "family", "cluster_id", "probability", "member_count", "representative_id", "intensity", "notes"});
  auto wsCat = ensure_sheet(wb, "PROFILE_CATALOG");
  headers(wsCat, {"case_id", "profile_id", "profile_name", "domain", "scope", "unit", "description"});
  auto wsProf = ensure_sheet(wb, "PROFILES");
  auto wsBind = ensure_sheet(wb, "BINDINGS");
  headers(wsBind, {"case_id", "binding_type", "kind", "component_index", "position", "bus", "profile_id", "target_field", "base_mw", "note"});
  auto wsNorm = ensure_sheet(wb, "NORMALIZATION");
  headers(wsNorm, {"case_id", "base_load_mw", "base_pv_mw", "base_wind_mw", "base_other_renewable_mw", "base_renewable_mw", "base_storage_mwh", "num_steps", "step_duration_hr", "profile_semantics"});
  auto wsCap = ensure_sheet(wb, "DEVICE_CAPACITY");
  headers(wsCap, {"case_id", "kind", "component_index", "position", "bus", "name", "base_mw", "base_mvar", "base_mwh", "capacity_field", "in_service", "profile_id", "editable"});
  auto wsSystem = ensure_sheet(wb, "SYSTEM_JSON");
  headers(wsSystem, {"case_id", "chunk_index", "json_chunk"});
  auto wsWarn = ensure_sheet(wb, "WARNINGS");
  headers(wsWarn, {"case_id", "severity", "message"});

  uint32_t scenRow = 2, catRow = 2, bindRow = 2, normRow = 2, capRow = 2, sysRow = 2, warnRow = 2;
  uint32_t profRow = 1;
  bool wroteProfileHeader = false;

  for (const auto& c : as_array_or_empty(bundle["cases"])) {
    const std::string case_id = c.value("case_id", "case");
    const auto meta = c.value("generated_scenario", c.value("_generated_scenario", nlohmann::json::object()));
    const auto ts = c.value("standard_time_series", c.value("_time_series", nlohmann::json::object()));
    const auto profiles = as_array_or_empty(ts.value("profiles", nlohmann::json::array()));
    wsScen.cell(scenRow, 1).value() = case_id;
    wsScen.cell(scenRow, 2).value() = meta.value("scenario_id", "");
    wsScen.cell(scenRow, 3).value() = meta.value("family", bundle.value("family", ""));
    wsScen.cell(scenRow, 4).value() = meta.value("cluster_id", 0);
    wsScen.cell(scenRow, 5).value() = meta.value("probability", 0.0);
    wsScen.cell(scenRow, 6).value() = meta.value("member_count", 0);
    wsScen.cell(scenRow, 7).value() = meta.value("representative_id", "");
    wsScen.cell(scenRow, 8).value() = meta.value("intensity", "");
    ++scenRow;

    for (const auto& p : profiles) {
      wsCat.cell(catRow, 1).value() = case_id;
      wsCat.cell(catRow, 2).value() = p.value("id", 0);
      wsCat.cell(catRow, 3).value() = p.value("name", "");
      wsCat.cell(catRow, 4).value() = p.value("domain", "");
      wsCat.cell(catRow, 5).value() = p.value("scope", "");
      wsCat.cell(catRow, 6).value() = p.value("unit", "multiplier");
      wsCat.cell(catRow, 7).value() = p.value("description", "");
      ++catRow;
    }

    if (!wroteProfileHeader) {
      std::vector<std::string> h = {"case_id", "time_index", "hour"};
      auto cols = profile_columns(profiles);
      h.insert(h.end(), cols.begin(), cols.end());
      headers(wsProf, h);
      wroteProfileHeader = true;
      profRow = 2;
    }
    const int num_steps = ts.value("num_steps", profiles.empty() ? 0 : static_cast<int>(profiles.front().value("values", nlohmann::json::array()).size()));
    const double step_hr = ts.value("step_duration_hr", 1.0);
    for (int t = 0; t < num_steps; ++t) {
      wsProf.cell(profRow, 1).value() = case_id;
      wsProf.cell(profRow, 2).value() = t;
      wsProf.cell(profRow, 3).value() = t * step_hr;
      uint16_t col = 4;
      for (const auto& p : profiles) {
        const auto values = as_array_or_empty(p.value("values", nlohmann::json::array()));
        if (t < static_cast<int>(values.size()) && values[static_cast<std::size_t>(t)].is_number()) {
          wsProf.cell(profRow, col).value() = values[static_cast<std::size_t>(t)].get<double>();
        }
        ++col;
      }
      ++profRow;
    }

    const auto binding = ts.value("binding", ts.value("bindings", nlohmann::json::object()));
    auto write_binding = [&](const std::string& type, const nlohmann::json& row) {
      wsBind.cell(bindRow, 1).value() = case_id;
      wsBind.cell(bindRow, 2).value() = type;
      wsBind.cell(bindRow, 3).value() = row.value("kind", "");
      wsBind.cell(bindRow, 4).value() = row.value("component_index", row.value("load_index", 0));
      wsBind.cell(bindRow, 5).value() = row.value("load_position", 0);
      wsBind.cell(bindRow, 6).value() = row.value("bus", 0);
      wsBind.cell(bindRow, 7).value() = row.value("profile_id", -1);
      wsBind.cell(bindRow, 8).value() = row.value("target_field", "");
      wsBind.cell(bindRow, 9).value() = row.value("base_mw", 0.0);
      ++bindRow;
    };
    write_binding("assign_all_loads", {{"profile_id", binding.value("assign_all_loads_to", -1)}});
    write_binding("assign_all_pv", {{"profile_id", binding.value("assign_all_pv_to", -1)}});
    for (const auto& row : as_array_or_empty(binding.value("load_profile_map", nlohmann::json::array()))) write_binding("load_profile_map", row);

    const auto norm = ts.value("normalization", nlohmann::json::object());
    wsNorm.cell(normRow, 1).value() = case_id;
    wsNorm.cell(normRow, 2).value() = norm.value("base_load_mw", 0.0);
    wsNorm.cell(normRow, 3).value() = norm.value("base_pv_mw", 0.0);
    wsNorm.cell(normRow, 4).value() = norm.value("base_wind_mw", 0.0);
    wsNorm.cell(normRow, 5).value() = norm.value("base_other_renewable_mw", 0.0);
    wsNorm.cell(normRow, 6).value() = norm.value("base_renewable_mw", 0.0);
    wsNorm.cell(normRow, 7).value() = norm.value("base_storage_mwh", 0.0);
    wsNorm.cell(normRow, 8).value() = num_steps;
    wsNorm.cell(normRow, 9).value() = step_hr;
    wsNorm.cell(normRow, 10).value() = norm.value("profile_semantics", "dimensionless_multiplier");
    ++normRow;

    write_device_capacities(wsCap, case_id, c.value("system", nlohmann::json::object()), capRow);

    const std::string sys_json = c.value("system", nlohmann::json::object()).dump();
    wsSystem.cell(sysRow, 1).value() = case_id;
    wsSystem.cell(sysRow, 2).value() = 0;
    wsSystem.cell(sysRow, 3).value() = sys_json;
    ++sysRow;

    for (const auto& w : as_array_or_empty(ts.value("warnings", nlohmann::json::array()))) {
      wsWarn.cell(warnRow, 1).value() = case_id;
      wsWarn.cell(warnRow, 2).value() = "warning";
      wsWarn.cell(warnRow, 3).value() = w.is_string() ? w.get<std::string>() : w.dump();
      ++warnRow;
    }
  }

  report.sheet_counts = {{"SCENARIOS", static_cast<int>(scenRow - 2)}, {"PROFILE_CATALOG", static_cast<int>(catRow - 2)}, {"PROFILES", static_cast<int>(profRow - 2)}, {"DEVICE_CAPACITY", static_cast<int>(capRow - 2)}};
  doc.save();
  doc.close();
}

nlohmann::json load_scenario_workbook(const std::string& path,
                                      ScenarioBundleImportMode mode,
                                      ScenarioBundleIoReport& report) {
  XLDocument doc;
  doc.open(path);
  auto wb = doc.workbook();
  if (!wb.worksheetExists("METADATA")) throw std::runtime_error("Missing METADATA sheet");
  if (!wb.worksheetExists("SYSTEM_JSON")) throw std::runtime_error("Missing SYSTEM_JSON sheet");

  nlohmann::json bundle = {{"format", "generated_scenario_case_bundle_v3"}, {"schema_version", 3}, {"unit_space", "dimensionless_multiplier"}, {"cases", nlohmann::json::array()}};
  std::unordered_map<std::string, nlohmann::json> by_case;

  auto sysWs = wb.worksheet("SYSTEM_JSON");
  auto sysCols = colmap(sysWs);
  for (uint32_t r = 2; r <= sysWs.rowCount(); ++r) {
    const std::string case_id = by_name(sysWs, r, sysCols, "case_id");
    const std::string chunk = by_name(sysWs, r, sysCols, "json_chunk");
    if (case_id.empty() || chunk.empty()) continue;
    by_case[case_id] = {{"case_id", case_id}, {"system", nlohmann::json::parse(chunk)}, {"generated_scenario", nlohmann::json::object()}, {"standard_time_series", {{"profiles", nlohmann::json::array()}}}};
  }

  if (wb.worksheetExists("SCENARIOS")) {
    auto ws = wb.worksheet("SCENARIOS");
    auto cols = colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string case_id = by_name(ws, r, cols, "case_id");
      if (case_id.empty() || !by_case.count(case_id)) continue;
      auto& meta = by_case[case_id]["generated_scenario"];
      meta["scenario_id"] = by_name(ws, r, cols, "scenario_id");
      meta["family"] = by_name(ws, r, cols, "family");
      meta["representative_id"] = by_name(ws, r, cols, "representative_id");
      meta["cluster_id"] = std::atoi(by_name(ws, r, cols, "cluster_id").c_str());
      meta["probability"] = std::atof(by_name(ws, r, cols, "probability").c_str());
      meta["member_count"] = std::atoi(by_name(ws, r, cols, "member_count").c_str());
      const auto intensity = by_name(ws, r, cols, "intensity");
      if (!intensity.empty()) meta["intensity"] = intensity;
    }
  }

  std::unordered_map<std::string, std::vector<nlohmann::json>> profile_catalog;
  if (wb.worksheetExists("PROFILE_CATALOG")) {
    auto ws = wb.worksheet("PROFILE_CATALOG");
    auto cols = colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string case_id = by_name(ws, r, cols, "case_id");
      if (case_id.empty()) continue;
      profile_catalog[case_id].push_back({{"id", std::atoi(by_name(ws, r, cols, "profile_id").c_str())},
                                          {"name", by_name(ws, r, cols, "profile_name")},
                                          {"domain", by_name(ws, r, cols, "domain")},
                                          {"scope", by_name(ws, r, cols, "scope")},
                                          {"unit", by_name(ws, r, cols, "unit")}});
    }
  }

  if (wb.worksheetExists("PROFILES")) {
    auto ws = wb.worksheet("PROFILES");
    auto cols = colmap(ws);
    std::unordered_map<std::string, std::unordered_map<int, std::vector<double>>> values;
    for (uint16_t c = 1; c <= ws.columnCount(); ++c) {
      const auto h = cell_to_string(ws, 1, c);
      if (h.size() > 1 && h[0] == 'p') {
        const int id = std::atoi(h.c_str() + 1);
        for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
          const std::string case_id = by_name(ws, r, cols, "case_id");
          if (case_id.empty()) continue;
          const auto s = cell_to_string(ws, r, c);
          values[case_id][id].push_back(s.empty() ? 0.0 : std::atof(s.c_str()));
        }
      }
    }
    for (auto& [case_id, case_values] : values) {
      auto profiles = nlohmann::json::array();
      for (auto p : profile_catalog[case_id]) {
        const int id = p.value("id", -1);
        p["values"] = case_values[id];
        profiles.push_back(std::move(p));
      }
      by_case[case_id]["standard_time_series"]["profiles"] = std::move(profiles);
      if (!profile_catalog[case_id].empty()) by_case[case_id]["standard_time_series"]["num_steps"] = static_cast<int>(case_values[profile_catalog[case_id].front().value("id", -1)].size());
    }
  }

  if (wb.worksheetExists("BINDINGS")) {
    auto ws = wb.worksheet("BINDINGS");
    auto cols = colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string case_id = by_name(ws, r, cols, "case_id");
      if (case_id.empty() || !by_case.count(case_id)) continue;
      auto& binding = by_case[case_id]["standard_time_series"]["binding"];
      if (!binding.is_object()) binding = nlohmann::json::object();
      const std::string type = by_name(ws, r, cols, "binding_type");
      const int profile_id = std::atoi(by_name(ws, r, cols, "profile_id").c_str());
      if (type == "assign_all_loads") {
        binding["assign_all_loads_to"] = profile_id;
      } else if (type == "assign_all_pv") {
        binding["assign_all_pv_to"] = profile_id;
      } else if (type == "load_profile_map") {
        if (!binding.contains("load_profile_map") || !binding["load_profile_map"].is_array()) binding["load_profile_map"] = nlohmann::json::array();
        nlohmann::json row = {{"kind", by_name(ws, r, cols, "kind")},
                              {"profile_id", profile_id},
                              {"bus", std::atoi(by_name(ws, r, cols, "bus").c_str())}};
        const int component_index = std::atoi(by_name(ws, r, cols, "component_index").c_str());
        const int position = std::atoi(by_name(ws, r, cols, "position").c_str());
        if (component_index > 0) row["load_index"] = component_index;
        if (position >= 0) row["load_position"] = position;
        binding["load_profile_map"].push_back(std::move(row));
      }
    }
  }

  if (wb.worksheetExists("NORMALIZATION")) {
    auto ws = wb.worksheet("NORMALIZATION");
    auto cols = colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string case_id = by_name(ws, r, cols, "case_id");
      if (case_id.empty() || !by_case.count(case_id)) continue;
      auto& ts = by_case[case_id]["standard_time_series"];
      ts["unit_space"] = "dimensionless_multiplier";
      ts["step_duration_hr"] = std::atof(by_name(ws, r, cols, "step_duration_hr").c_str());
      ts["normalization"] = {{"base_load_mw", std::atof(by_name(ws, r, cols, "base_load_mw").c_str())},
                              {"base_pv_mw", std::atof(by_name(ws, r, cols, "base_pv_mw").c_str())},
                              {"base_wind_mw", std::atof(by_name(ws, r, cols, "base_wind_mw").c_str())},
                              {"base_other_renewable_mw", std::atof(by_name(ws, r, cols, "base_other_renewable_mw").c_str())},
                              {"base_renewable_mw", std::atof(by_name(ws, r, cols, "base_renewable_mw").c_str())},
                              {"profile_semantics", by_name(ws, r, cols, "profile_semantics")}};
    }
  }

  if (wb.worksheetExists("DEVICE_CAPACITY")) {
    auto ws = wb.worksheet("DEVICE_CAPACITY");
    auto cols = colmap(ws);
    for (uint32_t r = 2; r <= ws.rowCount(); ++r) {
      const std::string case_id = by_name(ws, r, cols, "case_id");
      if (case_id.empty() || !by_case.count(case_id)) continue;
      const std::string kind = by_name(ws, r, cols, "kind");
      const int index = std::atoi(by_name(ws, r, cols, "component_index").c_str());
      const int position = std::atoi(by_name(ws, r, cols, "position").c_str());
      const std::string field = by_name(ws, r, cols, "capacity_field");
      const double base_mw = std::atof(by_name(ws, r, cols, "base_mw").c_str());
      const double base_mwh = std::atof(by_name(ws, r, cols, "base_mwh").c_str());
      apply_device_capacity_row(by_case[case_id]["system"], kind, index, position, field, base_mw, base_mwh);
    }
  }

  for (auto& [case_id, c] : by_case) {
    c["_generated_scenario"] = c["generated_scenario"];
    c["_time_series"] = c["standard_time_series"];
    bundle["cases"].push_back(std::move(c));
  }
  bundle["case_count"] = static_cast<int>(bundle["cases"].size());
  doc.close();
  report.sheet_counts.push_back({"SYSTEM_JSON", static_cast<int>(bundle["cases"].size())});
  validate_scenario_bundle(bundle, mode, report);
  return bundle;
}

#endif

}  // namespace hacdcpf::io
