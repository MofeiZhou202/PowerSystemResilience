/// tools/etap_convert.cpp
/// =======================
/// Command-line converter between the ETAP-schema Excel workbook and the native
/// hacdcpf JSON model.  Built only when ETAP support is enabled
/// (`-DHACDCPF_ENABLE_ETAP=ON`).
///
/// Usage:
///   etap_convert etap2json  <in.xlsx>  <out.json>  [--strict]
///   etap_convert json2etap  <in.json>  <out.xlsx>
///   etap_convert etap2etap  <in.xlsx>  <out.xlsx>  [--strict]   (normalise)
///
/// `--strict` rejects unresolved bus references during ETAP import (default is
/// permissive: such rows are imported best-effort and reported as warnings).

#include <iostream>
#include <string>

#include "hacdcpf/io/etap_io.hpp"
#include "hacdcpf/io/json_io.hpp"

namespace {

int usage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " <mode> <input> [output] [--strict]\n"
      << "  modes:\n"
      << "    etap2json   ETAP .xlsx  -> hacdcpf .json\n"
      << "    xml2json    ETAP project .xml (Feeder.xml) -> hacdcpf .json\n"
      << "    json2etap   hacdcpf .json -> ETAP .xlsx\n"
      << "    json2xml    hacdcpf .json -> ETAP project .xml\n"
      << "    xml2xml     ETAP .xml   -> ETAP .xml   (normalise)\n"
      << "    etap2etap   ETAP .xlsx  -> ETAP .xlsx   (normalise)\n"
      << "    fidelity    ETAP .xlsx  (report fields lost on re-export)\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace hacdcpf;
  using namespace hacdcpf::io;

  if (argc < 3) return usage(argv[0]);

  const std::string mode = argv[1];
  const std::string in = argv[2];
  bool strict = false;
  for (int i = 3; i < argc; ++i) {
    if (std::string(argv[i]) == "--strict") strict = true;
  }
  const EtapImportMode imode =
      strict ? EtapImportMode::Strict : EtapImportMode::Permissive;

  try {
    EtapIoReport rep;
    if (mode == "fidelity") {
      const HybridPowerSystem sys = load_etap(in, imode, rep);
      const EtapFidelityReport fr = etap_fidelity_check(sys);
      std::cout << "fidelity: " << (fr.lossless ? "LOSSLESS" : "LOSSY") << "  ("
                << fr.fields_mismatched << "/" << fr.fields_checked
                << " fields differ)\n";
      for (const auto& m : fr.mismatches) std::cout << "  " << m << "\n";
      return fr.lossless ? 0 : 3;
    }

    if (argc < 4) return usage(argv[0]);
    const std::string out = argv[3];
    if (mode == "etap2json") {
      const HybridPowerSystem sys = load_etap(in, imode, rep);
      save_json(sys, out);
    } else if (mode == "xml2json") {
      const HybridPowerSystem sys = load_etap_xml(in, imode, rep);
      save_json(sys, out);
    } else if (mode == "json2etap") {
      const HybridPowerSystem sys = load_json(in);
      save_etap(sys, out, rep);
    } else if (mode == "json2xml") {
      const HybridPowerSystem sys = load_json(in);
      save_etap_xml(sys, out, rep);
    } else if (mode == "xml2xml") {
      const HybridPowerSystem sys = load_etap_xml(in, imode, rep);
      save_etap_xml(sys, out, rep);
    } else if (mode == "etap2etap") {
      const HybridPowerSystem sys = load_etap(in, imode, rep);
      save_etap(sys, out, rep);
    } else {
      return usage(argv[0]);
    }

    for (const auto& w : rep.warnings) std::cerr << "warning: " << w << "\n";
    std::cout << "OK: " << mode << "  " << in << " -> " << out << "\n";
    for (const auto& kv : rep.sheet_counts) {
      std::cout << "  " << kv.first << ": " << kv.second << "\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
