#include "hacdcpf/io/powersimulationsdynamics_io.hpp"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/enum_strings.hpp"

namespace hacdcpf::io {
namespace {

using json = nlohmann::json;

double system_base_mva(const HybridPowerSystem& sys) {
  if (sys.base_mva > 0.0) return sys.base_mva;
  if (sys.ac.base_mva > 0.0) return sys.ac.base_mva;
  if (sys.dc.base_mva > 0.0) return sys.dc.base_mva;
  return 100.0;
}

std::string utc_now_string() {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &now);
#else
  gmtime_r(&now, &tm);
#endif
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
}

std::string fallback_name(const std::string& name,
                          const std::string& prefix,
                          int index) {
  if (!name.empty()) return name;
  return prefix + "-" + std::to_string(index);
}

json dynamic_component_to_json(const DynamicModelComponentProfile& component) {
  json out{{"slot", component.type},
           {"type", component.model},
           {"fields",
            {{"type", component.type},
             {"model", component.model},
             {"standard", component.standard},
             {"parameter_set", component.parameter_set},
             {"parameters", component.parameters}}}};
  if (!component.standard.empty()) out["standard"] = component.standard;
  return out;
}

json dynamic_model_to_json(const DynamicModelProfile& profile) {
  json out = json::object();
  if (!profile.standard.empty()) out["standard"] = profile.standard;
  if (!profile.model_name.empty()) out["model_name"] = profile.model_name;
  if (!profile.parameter_set.empty()) {
    out["parameter_set"] = profile.parameter_set;
  }
  if (!profile.source_id.empty()) out["source_id"] = profile.source_id;
  if (!profile.notes.empty()) out["notes"] = profile.notes;
  if (!profile.parameters.empty()) out["parameters"] = profile.parameters;
  if (!profile.components.empty()) {
    out["components"] = json::array();
    for (const auto& component : profile.components) {
      if (!component.type.empty() || !component.model.empty() ||
          !component.standard.empty() || !component.parameter_set.empty() ||
          !component.parameters.empty()) {
        out["components"].push_back(dynamic_component_to_json(component));
      }
    }
  }
  return out;
}

json profile_candidate(const DynamicModelProfile& profile) {
  std::string confidence = "metadata-only";
  if (!profile.model_name.empty() && !profile.parameters.empty()) {
    confidence = "profile";
  } else if (!profile.model_name.empty()) {
    confidence = "profile-only";
  }
  json out{{"model_name", profile.model_name}, {"confidence", confidence}};
  if (!profile.standard.empty()) out["standard"] = profile.standard;
  if (!profile.components.empty()) {
    out["components"] = json::array();
    for (const auto& component : profile.components) {
      out["components"].push_back(
          json{{"type", component.type}, {"model", component.model}});
    }
  }
  return out;
}

json base_record(const std::string& type,
                 const std::string& collection_path,
                 int index,
                 const std::string& name) {
  json out{{"type", type},
           {"name", fallback_name(name, type, index)},
           {"number", index},
           {"fields",
            {{"hacdcpf_component_type", type},
             {"collection_path", collection_path},
             {"index", index}}}};
  return out;
}

void attach_dynamic_profile(json& record,
                            const DynamicModelProfile& profile,
                            const PowerSimulationsDynamicsExportOptions& opts) {
  if (!opts.include_dynamic_profiles || profile.empty()) return;
  record["fields"]["dynamic_model"] = dynamic_model_to_json(profile);
  record["slots"] = json::array();
  for (const auto& component : profile.components) {
    record["slots"].push_back(dynamic_component_to_json(component));
  }
  record["hacdcpf_dynamic_profile_candidate"] = profile_candidate(profile);
}

void add_if_dynamic(json& dynamic_records,
                    json record,
                    const DynamicModelProfile& profile,
                    const PowerSimulationsDynamicsExportOptions& opts) {
  if (!opts.include_dynamic_profiles || profile.empty()) return;
  attach_dynamic_profile(record, profile, opts);
  dynamic_records.push_back(std::move(record));
}

std::string source_id(const std::string& collection_path, int index) {
  return collection_path + ":" + std::to_string(index);
}

json ac_bus_record(const ACBus& bus) {
  auto rec = base_record("ACBus", "ac.buses", bus.index, bus.name);
  rec["base_power_mva"] = nullptr;
  auto& f = rec["fields"];
  f["bus_type"] = bus_type_str(bus.bus_type);
  f["base_kv"] = bus.base_kv;
  f["vm_pu"] = bus.vm_pu;
  f["va_deg"] = bus.va_deg;
  f["pd_mw"] = bus.pd_mw;
  f["qd_mvar"] = bus.qd_mvar;
  f["in_service"] = bus.in_service;
  return rec;
}

json branch_record(const ACBranch& branch) {
  auto rec = base_record("ACBranch", "ac.branches", branch.index, branch.name);
  auto& f = rec["fields"];
  f["from_bus"] = branch.from_bus;
  f["to_bus"] = branch.to_bus;
  f["r_pu"] = branch.r_pu;
  f["x_pu"] = branch.x_pu;
  f["b_pu"] = branch.b_pu;
  f["rate_a_mva"] = branch.rate_a_mva;
  f["in_service"] = branch.in_service;
  return rec;
}

json transformer2w_record(const Transformer2W& transformer) {
  auto rec = base_record("Transformer2W", "ac.transformers_2w",
                         transformer.index, transformer.name);
  auto& f = rec["fields"];
  f["hv_bus"] = transformer.hv_bus;
  f["lv_bus"] = transformer.lv_bus;
  f["sn_mva"] = transformer.sn_mva;
  f["vn_hv_kv"] = transformer.vn_hv_kv;
  f["vn_lv_kv"] = transformer.vn_lv_kv;
  f["vk_percent"] = transformer.vk_percent;
  f["vkr_percent"] = transformer.vkr_percent;
  f["in_service"] = transformer.in_service;
  return rec;
}

json transformer3w_record(const Transformer3W& transformer) {
  auto rec = base_record("Transformer3W", "ac.transformers_3w",
                         transformer.index, transformer.name);
  auto& f = rec["fields"];
  f["hv_bus"] = transformer.hv_bus;
  f["mv_bus"] = transformer.mv_bus;
  f["lv_bus"] = transformer.lv_bus;
  f["sn_hv_mva"] = transformer.sn_hv_mva;
  f["sn_mv_mva"] = transformer.sn_mv_mva;
  f["sn_lv_mva"] = transformer.sn_lv_mva;
  f["in_service"] = transformer.in_service;
  return rec;
}

json generator_record(const Generator& generator) {
  auto rec = base_record("Generator", "ac.generators", generator.index,
                         generator.name);
  auto& f = rec["fields"];
  f["bus"] = generator.bus;
  f["pg_mw"] = generator.pg_mw;
  f["qg_mvar"] = generator.qg_mvar;
  f["vg_pu"] = generator.vg_pu;
  f["mbase_mva"] = generator.mbase_mva;
  f["inertia_h"] = generator.inertia_h;
  f["droop_r"] = generator.droop_r;
  f["is_slack"] = generator.is_slack;
  f["source_id"] = source_id("ac.generators", generator.index);
  f["in_service"] = generator.in_service;
  return rec;
}

json static_generator_record(const StaticGenerator& generator,
                             const std::string& collection_path) {
  auto rec =
      base_record("StaticGenerator", collection_path, generator.index,
                  generator.name);
  auto& f = rec["fields"];
  f["bus"] = generator.bus;
  f["p_mw"] = generator.p_mw;
  f["q_mvar"] = generator.q_mvar;
  f["p_rated_mw"] = generator.p_rated_mw;
  f["sn_mva"] = generator.sn_mva;
  f["sgen_type"] = sgen_type_str(generator.sgen_type);
  f["source_id"] = source_id(collection_path, generator.index);
  f["in_service"] = generator.in_service;
  return rec;
}

json renewable_record(const RenewableGen& generator) {
  auto rec = base_record("RenewableGen", "ac.renewable_gens",
                         generator.index, generator.name);
  auto& f = rec["fields"];
  f["bus"] = generator.bus;
  f["p_mw"] = generator.p_mw;
  f["q_mvar"] = generator.q_mvar;
  f["p_rated_mw"] = generator.p_rated_mw;
  f["renewable_type"] = renewable_type_str(generator.type);
  f["source_id"] = source_id("ac.renewable_gens", generator.index);
  f["in_service"] = generator.in_service;
  return rec;
}

json pv_record(const PVSystem& pv) {
  auto rec = base_record("PVSystem", "ac.pv_systems", pv.index, pv.name);
  auto& f = rec["fields"];
  f["bus"] = pv.bus;
  f["p_mw"] = pv.p_mw;
  f["q_mvar"] = pv.q_mvar;
  f["sn_mva"] = pv.sn_mva;
  f["v_ac_set_pu"] = pv.v_ac_set_pu;
  f["source_id"] = source_id("ac.pv_systems", pv.index);
  f["in_service"] = pv.in_service;
  return rec;
}

json load_record(const Load& load) {
  auto rec = base_record("Load", "ac.loads", load.index, load.name);
  auto& f = rec["fields"];
  f["bus"] = load.bus;
  f["p_mw"] = load.p_mw;
  f["q_mvar"] = load.q_mvar;
  f["model"] = load_model_str(load.model);
  f["z_percent_p"] = load.z_percent_p;
  f["i_percent_p"] = load.i_percent_p;
  f["p_percent_p"] = load.p_percent_p;
  f["source_id"] = source_id("ac.loads", load.index);
  f["in_service"] = load.in_service;
  return rec;
}

json asymmetric_load_record(const AsymmetricLoad& load) {
  auto rec = base_record("AsymmetricLoad", "ac.asymmetric_loads",
                         load.index, load.name);
  auto& f = rec["fields"];
  f["bus"] = load.bus;
  f["pa_mw"] = load.pa_mw;
  f["pb_mw"] = load.pb_mw;
  f["pc_mw"] = load.pc_mw;
  f["qa_mvar"] = load.qa_mvar;
  f["qb_mvar"] = load.qb_mvar;
  f["qc_mvar"] = load.qc_mvar;
  f["connection"] = load.connection;
  f["source_id"] = source_id("ac.asymmetric_loads", load.index);
  f["in_service"] = load.in_service;
  return rec;
}

json storage_record(const Storage& storage, const std::string& collection_path) {
  auto rec = base_record("Storage", collection_path, storage.index,
                         storage.name);
  auto& f = rec["fields"];
  f["bus"] = storage.bus;
  f["p_mw"] = storage.p_mw;
  f["q_mvar"] = storage.q_mvar;
  f["p_rated_mw"] = storage.p_rated_mw;
  f["e_rated_mwh"] = storage.e_rated_mwh;
  f["soc_init"] = storage.soc_init;
  f["grid_forming"] = storage.grid_forming;
  f["source_id"] = source_id(collection_path, storage.index);
  f["in_service"] = storage.in_service;
  return rec;
}

json external_grid_record(const ExternalGrid& grid) {
  auto rec = base_record("ExternalGrid", "ac.external_grids", grid.index,
                         grid.name);
  auto& f = rec["fields"];
  f["bus"] = grid.bus;
  f["vm_pu"] = grid.vm_pu;
  f["va_deg"] = grid.va_deg;
  f["s_sc_max_mva"] = grid.s_sc_max_mva;
  f["source_id"] = source_id("ac.external_grids", grid.index);
  f["in_service"] = grid.in_service;
  return rec;
}

json motor_record(const AsynchronousMotor& motor) {
  auto rec =
      base_record("AsynchronousMotor", "ac.motors", motor.index, motor.name);
  auto& f = rec["fields"];
  f["bus"] = motor.bus;
  f["sn_mva"] = motor.sn_mva;
  f["r_pu"] = motor.r_pu;
  f["x_pu"] = motor.x_pu;
  f["source_id"] = source_id("ac.motors", motor.index);
  f["in_service"] = motor.in_service;
  return rec;
}

json vsc_record(const VSCConverter& converter) {
  auto rec =
      base_record("VSCConverter", "vsc_converters", converter.index,
                  converter.name);
  auto& f = rec["fields"];
  f["bus_ac"] = converter.bus_ac;
  f["bus_dc"] = converter.bus_dc;
  f["control_mode"] = converter_mode_str(converter.control_mode);
  f["p_set_mw"] = converter.p_set_mw;
  f["q_set_mvar"] = converter.q_set_mvar;
  f["v_ac_set_pu"] = converter.v_ac_set_pu;
  f["v_dc_set_pu"] = converter.v_dc_set_pu;
  f["grid_forming"] = converter.grid_forming || converter.ac_grid_forming;
  f["source_id"] = source_id("vsc_converters", converter.index);
  f["in_service"] = converter.in_service;
  return rec;
}

json dcdc_record(const DCDCConverter& converter) {
  auto rec = base_record("DCDCConverter", "dc.dcdc_converters",
                         converter.index, converter.name);
  auto& f = rec["fields"];
  f["bus_in"] = converter.bus_in;
  f["bus_out"] = converter.bus_out;
  f["control_mode"] = dcdc_control_str(converter.control_mode);
  f["p_ref_mw"] = converter.p_ref_mw;
  f["v_ref_pu"] = converter.v_ref_pu;
  f["source_id"] = source_id("dc.dcdc_converters", converter.index);
  f["in_service"] = converter.in_service;
  return rec;
}

json dc_bus_record(const DCBus& bus) {
  auto rec = base_record("DCBus", "dc.buses", bus.index, bus.name);
  auto& f = rec["fields"];
  f["bus_type"] = dc_bus_type_str(bus.bus_type);
  f["base_kv"] = bus.base_kv;
  f["vm_pu"] = bus.vm_pu;
  f["pd_mw"] = bus.pd_mw;
  f["in_service"] = bus.in_service;
  return rec;
}

json dc_branch_record(const DCBranch& branch) {
  auto rec = base_record("DCBranch", "dc.branches", branch.index, branch.name);
  auto& f = rec["fields"];
  f["from_bus"] = branch.from_bus;
  f["to_bus"] = branch.to_bus;
  f["r_pu"] = branch.r_pu;
  f["rate_a_mva"] = branch.rate_a_mva;
  f["in_service"] = branch.in_service;
  return rec;
}

json dc_load_record(const DCLoad& load) {
  auto rec = base_record("DCLoad", "dc.loads", load.index, load.name);
  auto& f = rec["fields"];
  f["bus"] = load.bus;
  f["p_mw"] = load.p_mw;
  f["p_rated_mw"] = load.p_rated_mw;
  f["type"] = load.type;
  f["source_id"] = source_id("dc.loads", load.index);
  f["in_service"] = load.in_service;
  return rec;
}

json dc_storage_record(const DCStorage& storage) {
  auto rec = base_record("DCStorage", "dc.dc_storage", storage.index,
                         storage.name);
  auto& f = rec["fields"];
  f["bus"] = storage.bus;
  f["p_mw"] = storage.p_mw;
  f["p_rated_mw"] = storage.p_rated_mw;
  f["e_rated_mwh"] = storage.e_rated_mwh;
  f["soc_init"] = storage.soc_init;
  f["source_id"] = source_id("dc.dc_storage", storage.index);
  f["in_service"] = storage.in_service;
  return rec;
}

json dc_static_generator_record(const StaticGeneratorDC& generator) {
  auto rec = base_record("StaticGeneratorDC", "dc.dc_static_generators",
                         generator.index, generator.name);
  auto& f = rec["fields"];
  f["bus"] = generator.bus;
  f["p_set_mw"] = generator.p_set_mw;
  f["type"] = generator.type;
  f["source_id"] = source_id("dc.dc_static_generators", generator.index);
  f["in_service"] = generator.in_service;
  return rec;
}

json pv_array_dc_record(const PVArrayDC& pv) {
  auto rec = base_record("PVArrayDC", "dc.pv_arrays", pv.index, pv.name);
  auto& f = rec["fields"];
  f["bus"] = pv.bus;
  f["p_set_mw"] = pv.p_set_mw;
  f["irradiance"] = pv.irradiance;
  f["temperature"] = pv.temperature;
  f["source_id"] = source_id("dc.pv_arrays", pv.index);
  f["in_service"] = pv.in_service;
  return rec;
}

json energy_router_record(const EnergyRouter& router) {
  auto rec =
      base_record("EnergyRouter", "energy_routers", router.index,
                  router.name);
  auto& f = rec["fields"];
  f["num_ports"] = router.num_ports;
  f["p_rated_mw"] = router.p_rated_mw;
  f["control_mode"] = router.control_mode;
  f["source_id"] = source_id("energy_routers", router.index);
  f["in_service"] = router.in_service;
  f["ports"] = json::array();
  for (const auto& port : router.ports) {
    json port_record{{"index", port.index},
                     {"name", fallback_name(port.name, "EnergyRouterPort",
                                            port.index)},
                     {"bus", port.bus},
                     {"port_type", er_port_type_str(port.port_type)},
                     {"control_mode", er_control_str(port.control_mode)},
                     {"p_set_mw", port.p_set_mw},
                     {"q_set_mvar", port.q_set_mvar},
                     {"v_set_pu", port.v_set_pu}};
    if (!port.dynamic_model.empty()) {
      port_record["dynamic_model"] = dynamic_model_to_json(port.dynamic_model);
    }
    f["ports"].push_back(std::move(port_record));
  }
  return rec;
}

json mobile_storage_record(const MobileStorage& storage) {
  auto rec = base_record("MobileStorage", "mobile_storage", storage.index,
                         storage.name);
  auto& f = rec["fields"];
  f["bus"] = storage.bus;
  f["target_bus"] = storage.target_bus;
  f["p_mw"] = storage.p_mw;
  f["q_mvar"] = storage.q_mvar;
  f["p_rated_mw"] = storage.p_rated_mw;
  f["e_rated_mwh"] = storage.e_rated_mwh;
  f["soc_init"] = storage.soc_init;
  f["source_id"] = source_id("mobile_storage", storage.index);
  f["in_service"] = storage.in_service;
  return rec;
}

json vpp_record(const VirtualPowerPlant& vpp) {
  auto rec = base_record("VirtualPowerPlant", "vpps", vpp.index, vpp.name);
  auto& f = rec["fields"];
  f["pcc_bus"] = vpp.pcc_bus;
  f["p_output_mw"] = vpp.p_output_mw;
  f["q_output_mvar"] = vpp.q_output_mvar;
  f["pmax_mw"] = vpp.pmax_mw;
  f["pmin_mw"] = vpp.pmin_mw;
  f["source_id"] = source_id("vpps", vpp.index);
  f["in_service"] = vpp.in_service;
  return rec;
}

json microgrid_record(const Microgrid& microgrid) {
  auto rec = base_record("Microgrid", "microgrids", microgrid.index,
                         microgrid.name);
  auto& f = rec["fields"];
  f["pcc_bus"] = microgrid.pcc_bus;
  f["p_exchange_mw"] = microgrid.p_exchange_mw;
  f["total_generation_mw"] = microgrid.total_generation_mw;
  f["total_load_mw"] = microgrid.total_load_mw;
  f["v_set_pu"] = microgrid.v_set_pu;
  f["f_set_hz"] = microgrid.f_set_hz;
  f["source_id"] = source_id("microgrids", microgrid.index);
  f["in_service"] = microgrid.in_service;
  return rec;
}

json three_phase_bus_record(const ThreePhaseACBus& bus) {
  auto rec = base_record("ThreePhaseACBus", "three_phase_ac.buses",
                         bus.index, bus.name);
  auto& f = rec["fields"];
  f["bus_type"] = bus_type_str(bus.bus_type);
  f["base_kv"] = bus.base_kv;
  f["vm_a_pu"] = bus.vm_a_pu;
  f["vm_b_pu"] = bus.vm_b_pu;
  f["vm_c_pu"] = bus.vm_c_pu;
  f["in_service"] = bus.in_service;
  return rec;
}

json three_phase_line_record(const ThreePhaseACLine& line) {
  auto rec = base_record("ThreePhaseACLine", "three_phase_ac.lines",
                         line.index, line.name);
  auto& f = rec["fields"];
  f["from_bus"] = line.from_bus;
  f["to_bus"] = line.to_bus;
  f["length_km"] = line.length_km;
  f["phase_mask"] = phase_mask_to_string(line.phase_mask);
  f["r1_pu"] = line.r1_pu;
  f["x1_pu"] = line.x1_pu;
  f["r0_pu"] = line.r0_pu;
  f["x0_pu"] = line.x0_pu;
  f["in_service"] = line.in_service;
  return rec;
}

json three_phase_transformer_record(const ThreePhaseTransformer& transformer) {
  auto rec = base_record("ThreePhaseTransformer",
                         "three_phase_ac.transformers", transformer.index,
                         transformer.name);
  auto& f = rec["fields"];
  f["hv_bus"] = transformer.hv_bus;
  f["lv_bus"] = transformer.lv_bus;
  f["sn_mva"] = transformer.sn_mva;
  f["vn_hv_kv"] = transformer.vn_hv_kv;
  f["vn_lv_kv"] = transformer.vn_lv_kv;
  f["vector_group"] = transformer.vector_group;
  f["in_service"] = transformer.in_service;
  return rec;
}

json three_phase_load_record(const ThreePhaseLoad& load) {
  auto rec = base_record("ThreePhaseLoad", "three_phase_ac.loads",
                         load.index, load.name);
  auto& f = rec["fields"];
  f["bus"] = load.bus;
  f["p_a_mw"] = load.p_a_mw;
  f["p_b_mw"] = load.p_b_mw;
  f["p_c_mw"] = load.p_c_mw;
  f["source_id"] = source_id("three_phase_ac.loads", load.index);
  f["in_service"] = load.in_service;
  return rec;
}

json three_phase_generator_record(const ThreePhaseGenerator& generator) {
  auto rec = base_record("ThreePhaseGenerator", "three_phase_ac.generators",
                         generator.index, generator.name);
  auto& f = rec["fields"];
  f["bus"] = generator.bus;
  f["p_mw"] = generator.p_mw;
  f["q_mvar"] = generator.q_mvar;
  f["is_slack"] = generator.is_slack;
  f["source_id"] = source_id("three_phase_ac.generators", generator.index);
  f["in_service"] = generator.in_service;
  return rec;
}

json three_phase_external_grid_record(const ThreePhaseExternalGrid& grid) {
  auto rec = base_record("ThreePhaseExternalGrid",
                         "three_phase_ac.external_grids", grid.index,
                         grid.name);
  auto& f = rec["fields"];
  f["bus"] = grid.bus;
  f["vm_pu"] = grid.vm_pu;
  f["va_deg"] = grid.va_deg;
  f["s_sc_max_mva"] = grid.s_sc_max_mva;
  f["source_id"] = source_id("three_phase_ac.external_grids", grid.index);
  f["in_service"] = grid.in_service;
  return rec;
}

void add_conversion_notes(const HybridPowerSystem& sys, json& snapshot) {
  auto& notes = snapshot["conversion_notes"];
  notes.push_back(
      "This snapshot preserves HACDCPF model/controller identity for "
      "PowerSimulationsDynamics.jl comparison.");
  notes.push_back(
      "PowerSimulationsDynamics.jl is a positive-sequence dynamic simulator; "
      "hybrid DC, protection, reliability, and three-phase detail are retained "
      "as metadata unless a PSD equivalent is declared by psd_policy.");
  notes.push_back(
      "Use tools/psd_validation/export_trace.jl for PSD output trace "
      "conversion to the common time_s,value CSV format.");
  if (!sys.dc.buses.empty() || !sys.dc.branches.empty() ||
      !sys.dc.loads.empty()) {
    notes.push_back(
        "DC network objects are exported as diagnostic metadata; they are not "
        "direct PowerSystems.jl AC network elements.");
  }
  if (sys.three_phase_ac.has_value()) {
    notes.push_back(
        "Three-phase objects require aggregation/projection before PSD dynamic "
        "trace equivalence is claimed.");
  }
}

void update_counts(json& snapshot) {
  json counts = json::object();
  for (const auto& [key, value] : snapshot["components"].items()) {
    counts[key] = value.is_array() ? value.size() : 0U;
  }
  snapshot["system"]["component_counts"] = counts;
}

std::string julia_string_literal(const std::string& value) {
  std::ostringstream os;
  os << '"';
  for (unsigned char ch : value) {
    switch (ch) {
      case '\\':
        os << "\\\\";
        break;
      case '"':
        os << "\\\"";
        break;
      case '\n':
        os << "\\n";
        break;
      case '\r':
        os << "\\r";
        break;
      case '\t':
        os << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          os << "\\x" << std::hex << std::setw(2) << std::setfill('0')
             << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          os << static_cast<char>(ch);
        }
        break;
    }
  }
  os << '"';
  return os.str();
}

void skip_julia_ws(const std::string& text, std::size_t& pos) {
  while (pos < text.size()) {
    const char ch = text[pos];
    if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
      ++pos;
    } else {
      break;
    }
  }
}

std::string parse_julia_string_literal(const std::string& text,
                                       std::size_t pos) {
  if (pos >= text.size() || text[pos] != '"') {
    throw std::runtime_error("expected a Julia string literal");
  }
  ++pos;
  std::string out;
  while (pos < text.size()) {
    const char ch = text[pos++];
    if (ch == '"') return out;
    if (ch != '\\') {
      out.push_back(ch);
      continue;
    }
    if (pos >= text.size()) {
      throw std::runtime_error("unterminated Julia escape sequence");
    }
    const char esc = text[pos++];
    switch (esc) {
      case 'n':
        out.push_back('\n');
        break;
      case 'r':
        out.push_back('\r');
        break;
      case 't':
        out.push_back('\t');
        break;
      case '"':
        out.push_back('"');
        break;
      case '\\':
        out.push_back('\\');
        break;
      default:
        out.push_back(esc);
        break;
    }
  }
  throw std::runtime_error("unterminated Julia string literal");
}

std::string extract_julia_const_string(const std::string& text,
                                       const std::string& const_name) {
  const std::string needle = "const " + const_name;
  const auto found = text.find(needle);
  if (found == std::string::npos) {
    throw std::runtime_error("Julia file does not contain " + const_name);
  }
  auto pos = found + needle.size();
  skip_julia_ws(text, pos);
  if (pos >= text.size() || text[pos] != '=') {
    throw std::runtime_error("Julia const " + const_name + " has no assignment");
  }
  ++pos;
  skip_julia_ws(text, pos);
  return parse_julia_string_literal(text, pos);
}

}  // namespace

std::string to_powersimulationsdynamics_json(
    const HybridPowerSystem& sys,
    const PowerSimulationsDynamicsExportOptions& opts,
    int indent) {
  json snapshot;
  snapshot["format"] = "hacdcpf_psd_snapshot.v1";
  snapshot["generated_at"] = utc_now_string();
  snapshot["source"] = json{{"kind", "hacdcpf_rich_model"},
                            {"model_name", opts.model_name}};
  snapshot["system"] = json{{"name", sys.name},
                            {"base_power_mva", system_base_mva(sys)},
                            {"frequency_hz", sys.ac.freq_hz}};
  snapshot["components"] =
      json{{"buses", json::array()},
           {"branches", json::array()},
           {"static_injections", json::array()},
           {"dynamic_injections", json::array()},
           {"loads", json::array()},
           {"diagnostic_components", json::array()}};
  snapshot["conversion_notes"] = json::array();

  auto& components = snapshot["components"];
  auto& dynamic_records = components["dynamic_injections"];
  auto& diagnostic_records = components["diagnostic_components"];

  if (opts.include_static_components) {
    for (const auto& bus : sys.ac.buses) {
      components["buses"].push_back(ac_bus_record(bus));
    }
    for (const auto& branch : sys.ac.branches) {
      components["branches"].push_back(branch_record(branch));
    }
    for (const auto& transformer : sys.ac.transformers_2w) {
      components["branches"].push_back(transformer2w_record(transformer));
    }
    for (const auto& transformer : sys.ac.transformers_3w) {
      components["branches"].push_back(transformer3w_record(transformer));
    }
    for (const auto& load : sys.ac.loads) {
      auto rec = load_record(load);
      attach_dynamic_profile(rec, load.dynamic_model, opts);
      components["loads"].push_back(rec);
    }
    for (const auto& load : sys.ac.asymmetric_loads) {
      auto rec = asymmetric_load_record(load);
      attach_dynamic_profile(rec, load.dynamic_model, opts);
      components["loads"].push_back(rec);
    }
    for (const auto& generator : sys.ac.generators) {
      auto rec = generator_record(generator);
      attach_dynamic_profile(rec, generator.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& generator : sys.ac.static_generators) {
      auto rec = static_generator_record(generator, "ac.static_generators");
      attach_dynamic_profile(rec, generator.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& generator : sys.ac.renewable_gens) {
      auto rec = renewable_record(generator);
      attach_dynamic_profile(rec, generator.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& pv : sys.ac.pv_systems) {
      auto rec = pv_record(pv);
      attach_dynamic_profile(rec, pv.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& storage : sys.ac.storage) {
      auto rec = storage_record(storage, "ac.storage");
      attach_dynamic_profile(rec, storage.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& grid : sys.ac.external_grids) {
      auto rec = external_grid_record(grid);
      attach_dynamic_profile(rec, grid.dynamic_model, opts);
      components["static_injections"].push_back(rec);
    }
    for (const auto& motor : sys.ac.motors) {
      auto rec = motor_record(motor);
      attach_dynamic_profile(rec, motor.dynamic_model, opts);
      components["loads"].push_back(rec);
    }
  }

  for (const auto& generator : sys.ac.generators) {
    add_if_dynamic(dynamic_records, generator_record(generator),
                   generator.dynamic_model, opts);
  }
  for (const auto& generator : sys.ac.static_generators) {
    add_if_dynamic(dynamic_records,
                   static_generator_record(generator, "ac.static_generators"),
                   generator.dynamic_model, opts);
  }
  for (const auto& generator : sys.ac.renewable_gens) {
    add_if_dynamic(dynamic_records, renewable_record(generator),
                   generator.dynamic_model, opts);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    add_if_dynamic(dynamic_records, pv_record(pv), pv.dynamic_model, opts);
  }
  for (const auto& load : sys.ac.loads) {
    add_if_dynamic(dynamic_records, load_record(load), load.dynamic_model,
                   opts);
  }
  for (const auto& load : sys.ac.asymmetric_loads) {
    add_if_dynamic(dynamic_records, asymmetric_load_record(load),
                   load.dynamic_model, opts);
  }
  for (const auto& storage : sys.ac.storage) {
    add_if_dynamic(dynamic_records, storage_record(storage, "ac.storage"),
                   storage.dynamic_model, opts);
  }
  for (const auto& grid : sys.ac.external_grids) {
    add_if_dynamic(dynamic_records, external_grid_record(grid),
                   grid.dynamic_model, opts);
  }
  for (const auto& motor : sys.ac.motors) {
    add_if_dynamic(dynamic_records, motor_record(motor), motor.dynamic_model,
                   opts);
  }
  for (const auto& converter : sys.vsc_converters) {
    add_if_dynamic(dynamic_records, vsc_record(converter),
                   converter.dynamic_model, opts);
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    add_if_dynamic(dynamic_records, dcdc_record(converter),
                   converter.dynamic_model, opts);
  }
  for (const auto& load : sys.dc.loads) {
    add_if_dynamic(dynamic_records, dc_load_record(load), load.dynamic_model,
                   opts);
  }
  for (const auto& storage : sys.dc.storage) {
    add_if_dynamic(dynamic_records, storage_record(storage, "dc.storage"),
                   storage.dynamic_model, opts);
  }
  for (const auto& storage : sys.dc.dc_storage) {
    add_if_dynamic(dynamic_records, dc_storage_record(storage),
                   storage.dynamic_model, opts);
  }
  for (const auto& generator : sys.dc.static_generators) {
    add_if_dynamic(dynamic_records,
                   static_generator_record(generator, "dc.static_generators"),
                   generator.dynamic_model, opts);
  }
  for (const auto& generator : sys.dc.dc_static_generators) {
    add_if_dynamic(dynamic_records, dc_static_generator_record(generator),
                   generator.dynamic_model, opts);
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    add_if_dynamic(dynamic_records, pv_array_dc_record(pv), pv.dynamic_model,
                   opts);
  }
  for (const auto& router : sys.energy_routers) {
    add_if_dynamic(dynamic_records, energy_router_record(router),
                   router.dynamic_model, opts);
    for (const auto& port : router.ports) {
      auto rec = base_record("EnergyRouterPort", "energy_routers.ports",
                             port.index, port.name);
      rec["fields"]["router_index"] = router.index;
      rec["fields"]["bus"] = port.bus;
      rec["fields"]["port_type"] = er_port_type_str(port.port_type);
      rec["fields"]["control_mode"] = er_control_str(port.control_mode);
      add_if_dynamic(dynamic_records, std::move(rec), port.dynamic_model,
                     opts);
    }
  }
  for (const auto& storage : sys.mobile_storage) {
    add_if_dynamic(dynamic_records, mobile_storage_record(storage),
                   storage.dynamic_model, opts);
  }
  for (const auto& vpp : sys.vpps) {
    add_if_dynamic(dynamic_records, vpp_record(vpp), vpp.dynamic_model, opts);
  }
  for (const auto& microgrid : sys.microgrids) {
    add_if_dynamic(dynamic_records, microgrid_record(microgrid),
                   microgrid.dynamic_model, opts);
  }
  if (sys.three_phase_ac.has_value()) {
    const auto& tp = *sys.three_phase_ac;
    for (const auto& load : tp.loads) {
      add_if_dynamic(dynamic_records, three_phase_load_record(load),
                     load.dynamic_model, opts);
    }
    for (const auto& generator : tp.generators) {
      add_if_dynamic(dynamic_records, three_phase_generator_record(generator),
                     generator.dynamic_model, opts);
    }
    for (const auto& grid : tp.external_grids) {
      add_if_dynamic(dynamic_records, three_phase_external_grid_record(grid),
                     grid.dynamic_model, opts);
    }
  }

  for (const auto& bus : sys.dc.buses) {
    diagnostic_records.push_back(dc_bus_record(bus));
  }
  for (const auto& branch : sys.dc.branches) {
    diagnostic_records.push_back(dc_branch_record(branch));
  }
  for (const auto& load : sys.dc.loads) {
    auto rec = dc_load_record(load);
    attach_dynamic_profile(rec, load.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& storage : sys.dc.storage) {
    auto rec = storage_record(storage, "dc.storage");
    attach_dynamic_profile(rec, storage.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& storage : sys.dc.dc_storage) {
    auto rec = dc_storage_record(storage);
    attach_dynamic_profile(rec, storage.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& generator : sys.dc.static_generators) {
    auto rec = static_generator_record(generator, "dc.static_generators");
    attach_dynamic_profile(rec, generator.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& generator : sys.dc.dc_static_generators) {
    auto rec = dc_static_generator_record(generator);
    attach_dynamic_profile(rec, generator.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    auto rec = pv_array_dc_record(pv);
    attach_dynamic_profile(rec, pv.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& converter : sys.vsc_converters) {
    auto rec = vsc_record(converter);
    attach_dynamic_profile(rec, converter.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    auto rec = dcdc_record(converter);
    attach_dynamic_profile(rec, converter.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& router : sys.energy_routers) {
    auto rec = energy_router_record(router);
    attach_dynamic_profile(rec, router.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& storage : sys.mobile_storage) {
    auto rec = mobile_storage_record(storage);
    attach_dynamic_profile(rec, storage.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& vpp : sys.vpps) {
    auto rec = vpp_record(vpp);
    attach_dynamic_profile(rec, vpp.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  for (const auto& microgrid : sys.microgrids) {
    auto rec = microgrid_record(microgrid);
    attach_dynamic_profile(rec, microgrid.dynamic_model, opts);
    diagnostic_records.push_back(rec);
  }
  if (sys.three_phase_ac.has_value()) {
    const auto& tp = *sys.three_phase_ac;
    for (const auto& bus : tp.buses) {
      diagnostic_records.push_back(three_phase_bus_record(bus));
    }
    for (const auto& line : tp.lines) {
      diagnostic_records.push_back(three_phase_line_record(line));
    }
    for (const auto& transformer : tp.transformers) {
      diagnostic_records.push_back(three_phase_transformer_record(transformer));
    }
    for (const auto& load : tp.loads) {
      auto rec = three_phase_load_record(load);
      attach_dynamic_profile(rec, load.dynamic_model, opts);
      diagnostic_records.push_back(rec);
    }
    for (const auto& generator : tp.generators) {
      auto rec = three_phase_generator_record(generator);
      attach_dynamic_profile(rec, generator.dynamic_model, opts);
      diagnostic_records.push_back(rec);
    }
    for (const auto& grid : tp.external_grids) {
      auto rec = three_phase_external_grid_record(grid);
      attach_dynamic_profile(rec, grid.dynamic_model, opts);
      diagnostic_records.push_back(rec);
    }
  }

  if (opts.include_conversion_notes) add_conversion_notes(sys, snapshot);
  update_counts(snapshot);
  return snapshot.dump(indent);
}

void save_powersimulationsdynamics_json(
    const HybridPowerSystem& sys,
    const std::filesystem::path& path,
    const PowerSimulationsDynamicsExportOptions& opts,
    int indent) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot open PowerSimulationsDynamics snapshot: " +
                             path.string());
  }
  out << to_powersimulationsdynamics_json(sys, opts, indent) << '\n';
}

std::string to_powersimulationsdynamics_julia(
    const HybridPowerSystem& sys,
    const PowerSimulationsDynamicsExportOptions& opts,
    int indent) {
  const std::string snapshot =
      to_powersimulationsdynamics_json(sys, opts, indent);
  const std::string rich_json = to_json(sys, indent);

  std::ostringstream out;
  out << "# Auto-generated by HACDCPF.\n";
  out << "# PowerSimulationsDynamics.jl interchange wrapper.\n";
  out << "# The PSD snapshot is a dynamic-profile manifest; the embedded\n";
  out << "# HACDCPF rich JSON payload supports exact GUI round-trip import.\n\n";
  out << "module HACDCPFPowerSimulationsDynamicsSnapshot\n\n";
  out << "export HACDCPF_PSD_SNAPSHOT_JSON, HACDCPF_RICH_MODEL_JSON,\n";
  out << "       hacdcpf_psd_snapshot_json, hacdcpf_rich_model_json,\n";
  out << "       hacdcpf_psd_snapshot\n\n";
  out << "const HACDCPF_PSD_FORMAT = \"hacdcpf_psd_snapshot.v1\"\n";
  out << "const HACDCPF_PSD_WRAPPER_FORMAT = \"hacdcpf_psd_julia.v1\"\n";
  out << "const HACDCPF_PSD_SNAPSHOT_JSON = "
      << julia_string_literal(snapshot) << "\n";
  out << "const HACDCPF_RICH_MODEL_JSON = "
      << julia_string_literal(rich_json) << "\n\n";
  out << "hacdcpf_psd_snapshot_json() = HACDCPF_PSD_SNAPSHOT_JSON\n";
  out << "hacdcpf_rich_model_json() = HACDCPF_RICH_MODEL_JSON\n\n";
  out << "function hacdcpf_psd_snapshot()\n";
  out << "    try\n";
  out << "        @eval import JSON3\n";
  out << "        return JSON3.read(HACDCPF_PSD_SNAPSHOT_JSON)\n";
  out << "    catch\n";
  out << "        try\n";
  out << "            @eval import JSON\n";
  out << "            return JSON.parse(HACDCPF_PSD_SNAPSHOT_JSON)\n";
  out << "        catch err\n";
  out << "            error(\"Install JSON3.jl or JSON.jl to parse HACDCPF_PSD_SNAPSHOT_JSON\")\n";
  out << "        end\n";
  out << "    end\n";
  out << "end\n\n";
  out << "end # module HACDCPFPowerSimulationsDynamicsSnapshot\n";
  return out.str();
}

void save_powersimulationsdynamics_julia(
    const HybridPowerSystem& sys,
    const std::filesystem::path& path,
    const PowerSimulationsDynamicsExportOptions& opts,
    int indent) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot open PowerSimulationsDynamics Julia file: " +
                             path.string());
  }
  out << to_powersimulationsdynamics_julia(sys, opts, indent);
}

HybridPowerSystem from_powersimulationsdynamics_julia(
    const std::string& julia_source) {
  const std::string rich_json =
      extract_julia_const_string(julia_source, "HACDCPF_RICH_MODEL_JSON");
  return from_json(rich_json);
}

HybridPowerSystem load_powersimulationsdynamics_julia(
    const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open PowerSimulationsDynamics Julia file: " +
                             path.string());
  }
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  return from_powersimulationsdynamics_julia(text);
}

}  // namespace hacdcpf::io
