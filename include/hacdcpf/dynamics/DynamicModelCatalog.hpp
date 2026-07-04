#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace hacdcpf::dynamics {

// Single source of truth describing the tunable parameters of every transient
// control/device model and how they compose onto canvas component types. The
// GUI renders structured editors from this catalog, and a sync test asserts the
// catalog stays aligned with the parameter keys the DynamicModelBuilder reads.
//
// This is a data-like registry (returned by const&) mirroring the shipped
// hacdcpf::io::component_io_mapping pattern.

enum class DynamicParamKind {
  Real,
  Integer,
  Boolean,
  Enum
};

struct DynamicParamEnumOption {
  std::string value;
  std::string label;
};

// One tunable parameter of a model. `key` is the canonical name written into the
// dynamic_model parameter map; `aliases` are additional names the builder also
// accepts (kept so the sync test treats any alias as "consumed").
struct DynamicParamDescriptor {
  std::string key;
  std::vector<std::string> aliases;
  std::string label;
  std::string unit;
  DynamicParamKind kind{DynamicParamKind::Real};
  double default_value{0.0};
  std::optional<double> min_value;
  std::optional<double> max_value;
  std::string group;
  bool advanced{false};
  std::vector<DynamicParamEnumOption> enum_options;
  std::string notes;
};

// A named dynamic model (e.g. "GENROU", "TGOV1", "PSS1A") and its parameters.
struct DynamicModelDescriptor {
  std::string model_name;
  std::string standard;
  std::string display_name;
  std::string block_role;          // machine|governor|exciter|pss|gfm|gfl|pll|storage|dcdc|load
  std::string parameters_location;  // "root" | "component:pll"
  std::vector<DynamicParamDescriptor> parameters;
  std::string notes;
};

// One control-block slot on a component type (e.g. a generator's "governor").
struct DynamicControlBlockSlot {
  std::string slot;                    // machine|governor|exciter|pss|converter|pll|storage
  std::string label;
  bool optional{false};                // include a "None" choice
  std::vector<std::string> model_names;
  std::string default_model;
  std::string visible_when_model;      // only show when the parent slot has this model
};

// The control-block composition for one canvas component type.
struct DynamicComponentComposition {
  std::string canvas_type;             // gen|vsc|sgen|pv|renGen|storage|dcdcConverter|load
  std::string display_name;
  std::string component_domain;        // AC|DC
  std::vector<DynamicControlBlockSlot> slots;
};

// Immutable registries.
const std::vector<DynamicModelDescriptor>& dynamic_model_catalog();
const std::vector<DynamicComponentComposition>& dynamic_block_composition();

std::optional<DynamicModelDescriptor> find_dynamic_model(std::string_view model_name);
std::optional<DynamicComponentComposition> find_block_composition(
    std::string_view canvas_type);

std::string to_string(DynamicParamKind kind);

}  // namespace hacdcpf::dynamics
