#pragma once

/// io/etap_io.hpp
/// ==============
/// ETAP <-> internal-model Excel import/export.
///
/// Reads and writes an Excel workbook laid out in the *ETAP element schema* —
/// one worksheet per ETAP component type, with ETAP-style column headers.  The
/// schema mirrors the workbook produced by the companion `etap-main` Python
/// toolkit (see `etap_output.py::ETAPExporter`), which exports an ETAP project
/// to Excel with one sheet per element class.
///
/// Supported ETAP element sheets and their internal-model targets:
///
///   ETAP sheet     internal target                 notes
///   ----------     ---------------                 -----
///   PROJECT        HybridPowerSystem (base_mva)    optional metadata sheet
///   BUS            ACSystem::buses                 AC bus
///   XLINE / CABLE  ACSystem::branches             AC line (ohmic -> pu)
///   XFORM2W        ACSystem::transformers_2w       2-winding transformer
///   XFORM3W        ACSystem::transformers_3w       3-winding transformer
///   UTIL           ACSystem::external_grids        utility / swing source
///   SYNGEN         ACSystem::generators            synchronous generator
///   MGSET          ACSystem::generators            motor-generator (import only)
///   PVARRAY        ACSystem::pv_systems            PV array (AC inverter)
///   WIND           ACSystem::renewable_gens        wind / renewable generator
///   LUMPEDLOAD     ACSystem::loads                 lumped AC load (ZIP model)
///   CAPACITOR      ACSystem::shunts                shunt capacitor
///   HVCB           ACSystem::circuit_breakers      AC breaker
///   INDMOTOR       ACSystem::motors                induction motor
///   DCBUS          DCSystem::buses                 DC bus (NominalV in volts)
///   DCIMPEDANCE    DCSystem::branches              DC line (ohmic -> pu)
///   DCLUMPLOAD     DCSystem::loads                 DC load (kW -> MW)
///   DCCONVERTER    DCSystem::dcdc_converters       DC/DC converter
///   DCCB           DCSystem::dc_circuit_breakers   DC breaker
///   INVERTER       HybridPowerSystem::vsc_converters (type="INVERTER")
///   CHARGER        HybridPowerSystem::vsc_converters (type="CHARGER")
///   BATTERY        DCSystem::storage               battery storage
///
/// Conventions (chosen so that `load_etap(save_etap(sys))` is loss-free for the
/// element subset above):
///   * Bus identity is the ETAP `ID` string (== internal `name`).  Internal
///     integer indices are re-assigned 1..N in sheet order on import.
///   * Per-unit impedances use Z_base = base_kV^2 / base_MVA, taken from the
///     branch *from*-bus base voltage and the system base MVA.  Ohmic columns
///     therefore round-trip exactly given the same base.
///   * DC bus `NominalV` is stored in volts; `base_kv = NominalV / 1000`.
///   * DC powers are stored in kW (ETAP convention); AC powers in MW.
///   * An explicit `Type` column carries the bus type (PQ/PV/SLACK/...) so it
///     round-trips independent of source-placement heuristics.
///
/// The importer also accepts the *raw* attribute names emitted by the ETAP
/// toolkit (`etap_output.py`) — e.g. `OpVMag`/`VMag` (percent), `NominalkV`,
/// `RPos`/`XPos` (ohmic), `AnsiPosZ`/`PosR` (transformer %Z / %R),
/// `ZBaseMVA` (kVA), and lumped-load `MVA`+`PF` — applying the matching unit
/// conversions, so real ETAP exports can be ingested directly.  When no `Type`
/// column is present the bus type is derived from connected utilities
/// (-> SLACK) and generators (-> PV).  Import strictness is controlled by
/// `EtapImportMode`.
///
/// Requires OpenXLSX (enable with `-DHACDCPF_ENABLE_ETAP=ON`).  When ETAP Excel
/// support is not compiled in, the functions throw `std::runtime_error`.

#include <stdexcept>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// Strictness of an ETAP import.
enum class EtapImportMode {
  /// Unresolved bus references and malformed rows raise an exception.
  Strict,
  /// Such issues are recorded as warnings in the EtapIoReport and the offending
  /// row is best-effort imported (unresolved references left as 0).
  Permissive,
};

/// Diagnostics collected while importing or exporting an ETAP workbook.
struct EtapIoReport {
  /// Non-fatal issues (unknown bus references, skipped rows, ...).
  std::vector<std::string> warnings;
  /// Number of rows imported / exported per ETAP sheet (sheet name -> count).
  std::vector<std::pair<std::string, int>> sheet_counts;

  int count(const std::string& sheet) const {
    for (const auto& kv : sheet_counts) {
      if (kv.first == sheet) return kv.second;
    }
    return 0;
  }
};

/// Result of an ETAP export/import round-trip fidelity check: which fields (if
/// any) are not preserved by `save_etap` followed by `load_etap`.
struct EtapFidelityReport {
  bool lossless{true};
  int fields_checked{0};
  int fields_mismatched{0};
  /// Human-readable "Type[i].field: before -> after" entries for each mismatch.
  std::vector<std::string> mismatches;
};

#ifdef HACDCPF_ENABLE_ETAP

/// Import an ETAP-schema Excel workbook into a HybridPowerSystem.
HybridPowerSystem load_etap(const std::string& path);

/// Import an ETAP-schema Excel workbook, also returning per-sheet diagnostics.
HybridPowerSystem load_etap(const std::string& path, EtapIoReport& report);

/// Import with explicit strictness; warnings/errors are recorded in `report`.
HybridPowerSystem load_etap(const std::string& path, EtapImportMode mode,
                            EtapIoReport& report);

/// Round-trip `sys` through `save_etap`/`load_etap` (via a temporary workbook)
/// and report any element counts or mapped fields that are not preserved.
EtapFidelityReport etap_fidelity_check(const HybridPowerSystem& sys,
                                       double tol = 1e-6);

/// Import a native ETAP project XML export (e.g. `Feeder.xml`) directly, without
/// going through the Python toolkit.  Best-effort: recognised `<COMPONENTS>`
/// element tags are mapped with the same ETAP attribute names and units as the
/// Excel importer.  Attribute values containing a raw `>` are not supported.
HybridPowerSystem load_etap_xml(const std::string& path, EtapImportMode mode,
                                EtapIoReport& report);
HybridPowerSystem load_etap_xml(const std::string& path);

/// Export a HybridPowerSystem to an ETAP-schema Excel workbook.
void save_etap(const HybridPowerSystem& sys, const std::string& path);

/// Export a HybridPowerSystem, also returning per-sheet diagnostics.
void save_etap(const HybridPowerSystem& sys, const std::string& path,
               EtapIoReport& report);

#else

inline HybridPowerSystem load_etap(const std::string&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline HybridPowerSystem load_etap(const std::string&, EtapIoReport&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline HybridPowerSystem load_etap(const std::string&, EtapImportMode,
                                   EtapIoReport&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline EtapFidelityReport etap_fidelity_check(const HybridPowerSystem&,
                                              double = 1e-6) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline HybridPowerSystem load_etap_xml(const std::string&, EtapImportMode,
                                       EtapIoReport&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline HybridPowerSystem load_etap_xml(const std::string&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline void save_etap(const HybridPowerSystem&, const std::string&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline void save_etap(const HybridPowerSystem&, const std::string&,
                      EtapIoReport&) {
  throw std::runtime_error(
      "ETAP Excel I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

#endif  // HACDCPF_ENABLE_ETAP

}  // namespace hacdcpf::io
