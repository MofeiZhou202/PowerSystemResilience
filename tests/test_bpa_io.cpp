/// tests/test_bpa_io.cpp
/// =======================
/// BPA/DSP .dat importer contract tests (io/bpa_io).
///
///   * Structure: bus / branch / generator / DC-element counts and spot
///     values for the four sample cases under data/dsp.
///   * Round-trip: to_json -> from_json preserves element counts.
///   * Accuracy: Newton power flow on the converted pure-AC cases
///     (39.dat, IEEE90.dat) is compared against the DSP reference solutions
///     (Samples/39bus/39NEW.SOL, Samples/IEEE90/IEEE90NEW.SOL).
///   * HVDC cases (2DC.dat, cigre.dat): conversion must converge and the
///     LCC link must transfer approximately its scheduled 1500 MW.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/bpa_io.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/validation/validate_system.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::string dat_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/dsp/" + name;
}

const hacdcpf::ACBus* find_bus(const hacdcpf::HybridPowerSystem& sys,
                               const std::string& name) {
  for (const auto& b : sys.ac.buses)
    if (b.name == name) return &b;
  return nullptr;
}

const hacdcpf::ACBranch* find_branch(const hacdcpf::HybridPowerSystem& sys,
                                     const std::string& name) {
  for (const auto& b : sys.ac.branches)
    if (b.name == name) return &b;
  return nullptr;
}

const hacdcpf::Generator* find_gen(const hacdcpf::HybridPowerSystem& sys,
                                   const std::string& name) {
  for (const auto& g : sys.ac.generators)
    if (g.name == name) return &g;
  return nullptr;
}

}  // namespace

TEST_CASE("BPA import: 39-bus structure", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 39);
  REQUIRE(sys.ac.branches.size() == 46);  // 34 L + 12 T cards
  REQUIRE(sys.ac.generators.size() == 10);
  REQUIRE(sys.ac.loads.size() == 20);
  REQUIRE(sys.dc.buses.empty());

  const auto* b30 = find_bus(sys, "new30");
  REQUIRE(b30 != nullptr);
  REQUIRE(b30->bus_type == hacdcpf::BusType::PV);
  REQUIRE_THAT(b30->vm_pu, WithinAbs(1.047, 1e-9));
  REQUIRE_THAT(b30->base_kv, WithinAbs(20.0, 1e-9));

  const auto* g30 = find_gen(sys, "new30");
  REQUIRE(g30 != nullptr);
  REQUIRE_THAT(g30->pg_mw, WithinAbs(250.0, 1e-9));
  REQUIRE_THAT(g30->vg_pu, WithinAbs(1.047, 1e-9));

  const auto* bs = find_bus(sys, "new39");
  REQUIRE(bs != nullptr);
  REQUIRE(bs->bus_type == hacdcpf::BusType::SLACK);
  REQUIRE_THAT(bs->vm_pu, WithinAbs(1.03, 1e-9));

  const auto* l12 = find_branch(sys, "L_new1_new2");
  REQUIRE(l12 != nullptr);
  REQUIRE_THAT(l12->r_pu, WithinAbs(0.0035, 1e-9));
  REQUIRE_THAT(l12->x_pu, WithinAbs(0.0411, 1e-9));
  REQUIRE_THAT(l12->b_pu, WithinAbs(0.6936, 1e-9));  // 2 * B/2

  const auto* t631 = find_branch(sys, "T_new6_new31");
  REQUIRE(t631 != nullptr);
  REQUIRE_THAT(t631->x_pu, WithinAbs(0.025, 1e-9));
  // tap = (369/345) / (20/20)
  REQUIRE_THAT(t631->tap, WithinAbs(369.0 / 345.0, 1e-9));
}

TEST_CASE("BPA import: IEEE90 structure and GBK names", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("IEEE90.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 9);
  REQUIRE(sys.ac.branches.size() == 9);  // 6 L + 3 T cards
  REQUIRE(sys.ac.generators.size() == 3);
  REQUIRE(sys.ac.loads.size() == 4);
  REQUIRE(sys.ac.shunts.size() == 3);

  // Chinese bus names must survive as valid UTF-8.
  const auto* g1 = find_bus(sys, "发电机1");
  REQUIRE(g1 != nullptr);
  REQUIRE(g1->bus_type == hacdcpf::BusType::SLACK);
  REQUIRE_THAT(g1->vm_pu, WithinAbs(1.01, 1e-9));
  REQUIRE_THAT(g1->base_kv, WithinAbs(16.5, 1e-9));

  const auto* g2 = find_gen(sys, "发电机2");
  REQUIRE(g2 != nullptr);
  REQUIRE_THAT(g2->pg_mw, WithinAbs(163.0, 1e-9));

  const auto* tr = find_branch(sys, "T_发电机1_母线1");
  REQUIRE(tr != nullptr);
  REQUIRE_THAT(tr->x_pu,
               WithinAbs(0.0567 * std::pow(242.0 / 230.0, 2), 1e-9));
  // tap = (16.5/16.5) / (242/230)
  REQUIRE_THAT(tr->tap, WithinAbs(230.0 / 242.0, 1e-9));
}

TEST_CASE("BPA import: 2DC HVDC structure", "[bpa][structure]") {
  hacdcpf::io::BpaImportOptions options;
  options.lcc_model = hacdcpf::io::BpaLccModel::VscApprox;  // legacy VSC path
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"), options);
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 11);
  REQUIRE(sys.ac.branches.size() == 13);  // 6 L + 7 T cards
  REQUIRE(sys.ac.generators.size() == 5);
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE(sys.vsc_converters.size() == 2);

  const auto& dcb = sys.dc.branches.front();
  // r_pu = 10 ohm * 100 MVA / (500 kV)^2
  REQUIRE_THAT(dcb.r_pu, WithinAbs(0.004, 1e-9));
  REQUIRE_THAT(dcb.rate_a_mva, WithinAbs(1500.0, 1e-9));

  // Rectifier draws scheduled power; inverter forms the DC voltage (VDC_Q)
  // and delivers the received power emergently.
  const auto& rect = sys.vsc_converters[0];
  const auto& inv = sys.vsc_converters[1];
  REQUIRE(rect.control_mode == hacdcpf::ConverterMode::PQ_MODE);
  REQUIRE_THAT(rect.p_set_mw, WithinAbs(-1500.0, 1e-9));
  REQUIRE(inv.control_mode == hacdcpf::ConverterMode::VDC_Q);
  // v_dc_set = (500 kV - 3 kA * 10 ohm) / 500 kV = 0.94 pu
  REQUIRE_THAT(inv.v_dc_set_pu, WithinAbs(0.94, 1e-9));
  REQUIRE(inv.k_vdc > 100.0);
  // Station reactive consumption estimate: 0.5 * P, absorbed from AC.
  REQUIRE(rect.q_set_mvar < 0.0);
  REQUIRE(inv.q_set_mvar < 0.0);
}

TEST_CASE("BPA import: cigre HVDC structure", "[bpa][structure]") {
  hacdcpf::io::BpaImportOptions options;
  options.lcc_model = hacdcpf::io::BpaLccModel::VscApprox;  // legacy VSC path
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"), options);
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 6);
  REQUIRE(sys.ac.branches.size() == 4);  // 2 L + 2 T cards
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE(sys.vsc_converters.size() == 2);
  REQUIRE_THAT(sys.dc.branches.front().r_pu, WithinAbs(0.004, 1e-9));
  // Valve-drop station loss from the BD card: 100 V * 3000 A * 2 bridges,
  // mapped onto eta for the default Linear loss model (0.6 / 1500 = 4e-4).
  REQUIRE_THAT(sys.vsc_converters[0].loss_mw, WithinAbs(0.6, 1e-9));
  REQUIRE_THAT(sys.vsc_converters[1].loss_mw, WithinAbs(0.6, 1e-9));
  REQUIRE_THAT(sys.vsc_converters[0].eta, WithinAbs(0.9996, 1e-9));

  const auto* recgen = find_gen(sys, "RECGEN");
  const auto* invgen = find_gen(sys, "INVGEN");
  REQUIRE(recgen != nullptr);
  REQUIRE(invgen != nullptr);
  REQUIRE_THAT(recgen->pmin_mw, WithinAbs(-9999.0, 1e-9));
  REQUIRE_THAT(recgen->pmax_mw, WithinAbs(3100.0, 1e-9));
  REQUIRE_THAT(invgen->pmin_mw, WithinAbs(-9999.0, 1e-9));
  REQUIRE_THAT(invgen->pmax_mw, WithinAbs(9999.0, 1e-9));
}

TEST_CASE("BPA import: BS has an open active-power range while BE/BQ do not",
          "[bpa][structure][bounds]") {
  const auto bus_card = [](const std::string& type, const std::string& name,
                           const std::string& pmax) {
    std::string card(65, ' ');
    const auto put = [&](size_t first, size_t last,
                         const std::string& value) {
      REQUIRE(first >= 1);
      REQUIRE(first <= last);
      REQUIRE(value.size() <= last - first + 1);
      card.replace(first - 1, value.size(), value);
    };
    put(1, 2, type);
    put(7, 14, name);
    put(15, 18, "525.");
    if (!pmax.empty()) put(39, 42, pmax);
    put(43, 47, "100.");
    put(58, 61, "1000");
    return card + "\n";
  };

  const std::string content =
      bus_card("BS", "SLACK", "") +
      bus_card("BE", "PVGEN", "250.") +
      bus_card("BQ", "PVQGEN", "300.") +
      "(END)\n";
  const auto res = hacdcpf::io::parse_bpa_dat_string(content);
  REQUIRE_FALSE(res.report.has_errors());

  const auto* slack = find_gen(res.system, "SLACK");
  const auto* be = find_gen(res.system, "PVGEN");
  const auto* bq = find_gen(res.system, "PVQGEN");
  REQUIRE(slack != nullptr);
  REQUIRE(be != nullptr);
  REQUIRE(bq != nullptr);
  REQUIRE_THAT(slack->pmin_mw, WithinAbs(-9999.0, 1e-9));
  REQUIRE_THAT(slack->pmax_mw, WithinAbs(9999.0, 1e-9));
  REQUIRE_THAT(be->pmin_mw, WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(be->pmax_mw, WithinAbs(250.0, 1e-9));
  REQUIRE_THAT(bq->pmin_mw, WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(bq->pmax_mw, WithinAbs(300.0, 1e-9));
}

TEST_CASE("BPA import: ordinary B-card generation remains a fixed PQ injection",
          "[bpa][generator]") {
  std::string card(80, ' ');
  card.replace(0, 2, "B ");
  card.replace(6, 6, "PQGEN1");
  card.replace(14, 4, "1.95");
  card.replace(38, 4, "250.");
  card.replace(42, 4, "183.");
  card.replace(47, 5, "-120.");

  const auto imported =
      hacdcpf::io::parse_bpa_dat_string(card + "\n(END)\n");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.ac.buses.size() == 1);
  REQUIRE(imported.system.ac.buses.front().bus_type == hacdcpf::BusType::PQ);
  REQUIRE(imported.system.ac.generators.size() == 1);
  REQUIRE_THAT(imported.system.ac.generators.front().pg_mw,
               WithinAbs(183.0, 1e-12));
  REQUIRE_THAT(imported.system.ac.generators.front().pmax_mw,
               WithinAbs(250.0, 1e-12));
  REQUIRE_THAT(imported.system.ac.generators.front().qg_mvar,
               WithinAbs(-120.0, 1e-12));
  REQUIRE_THAT(imported.system.ac.generators.front().qmin_mvar,
               WithinAbs(-120.0, 1e-12));
  REQUIRE_THAT(imported.system.ac.generators.front().qmax_mvar,
               WithinAbs(-120.0, 1e-12));
}

TEST_CASE("BPA import: line continuations and order-independent bus binding",
          "[bpa][structure][ordering][line-plus]") {
  const auto card = [](const std::string& type, const std::string& name1,
                       const std::string& kv1, const std::string& name2 = "",
                       const std::string& kv2 = "",
                       const std::string& circuit = "") {
    std::string value(80, ' ');
    const auto put = [&](size_t first, size_t last,
                         const std::string& field_value) {
      REQUIRE(first >= 1);
      REQUIRE(first <= last);
      REQUIRE(field_value.size() <= last - first + 1);
      value.replace(first - 1, field_value.size(), field_value);
    };
    put(1, 2, type);
    put(7, 14, name1);
    put(15, 18, kv1);
    if (!name2.empty()) {
      put(20, 27, name2);
      put(28, 31, kv2);
      if (!circuit.empty()) put(32, 32, circuit);
    }
    return value;
  };
  const auto skipped_count = [](const auto& report) {
    return std::count_if(
        report.records.begin(), report.records.end(), [](const auto& record) {
          return record.disposition ==
                 hacdcpf::io::ImportDisposition::Skipped;
        });
  };

  SECTION("an L card before its B cards hydrates the existing bus IDs") {
    std::string line = card("L ", "BUS500A", "525.", "BUS500B", "525.");
    line.replace(38, 4, "1000");
    line.replace(44, 5, "10000");
    const std::string content =
        ".comment\n"
        " .DSP header with leading whitespace\n"
        " equipment description\n" +
        line + "\n" + card("BS", "BUS500A", "525.") + "\n" +
        card("B ", "BUS500B", "525.") + "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.buses.size() == 2);
    REQUIRE(imported.system.ac.branches.size() == 1);
    REQUIRE(imported.system.ac.branches[0].from_bus !=
            imported.system.ac.branches[0].to_bus);
    REQUIRE(imported.system.ac.buses[0].bus_type ==
            hacdcpf::BusType::SLACK);
    REQUIRE(skipped_count(imported.report) == 0);
  }

  SECTION("equal names at different nominal voltages remain distinct") {
    const std::string content =
        card("BS", "SUBSTATN", "500.") + "\n" +
        card("B ", "SUBSTATN", "220.") + "\n" +
        card("T ", "SUBSTATN", "500.", "SUBSTATN", "220.") +
        "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.buses.size() == 2);
    REQUIRE(imported.system.ac.branches.size() == 1);
    REQUIRE(imported.system.ac.branches[0].from_bus !=
            imported.system.ac.branches[0].to_bus);
  }

  SECTION("zero-data L cards retain ideal-connectivity provenance") {
    const std::string content =
        card("BS", "BUS500A", "525.") + "\n" +
        card("B ", "BUS500B", "525.") + "\n" +
        card("L ", "BUS500A", "525.", "BUS500B", "525.") +
        "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.branches.size() == 1);
    CHECK(imported.system.ac.branches.front().ideal_connectivity);
    CHECK(imported.system.ac.branches.front().parameter_source ==
          "bpa_dsp_ideal_connectivity");
    const auto projected =
        hacdcpf::project_to_canonical_models(imported.system, false);
    CHECK(projected.ac.buses.size() == 1);
    CHECK(projected.ac.branches.empty());
  }

  SECTION("L+ terminal reactor values become fixed inductive shunts") {
    std::string line = card("L ", "BUS500A", "525.", "BUS500B", "525.", "1");
    line.replace(38, 4, "1000");
    line.replace(44, 5, "10000");
    std::string extension =
        card("L+", "BUS500A", "525.", "BUS500B", "525.", "1");
    extension.replace(33, 4, "109.");
    extension.replace(43, 4, "136.");
    const std::string content =
        card("BS", "BUS500A", "525.") + "\n" +
        card("B ", "BUS500B", "525.") + "\n" + line + "\n" +
        extension + "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.branches.size() == 1);
    REQUIRE(imported.system.ac.shunts.size() == 2);
    REQUIRE_THAT(imported.system.ac.shunts[0].bs_mvar,
                 WithinAbs(-109.0, 1e-12));
    REQUIRE_THAT(imported.system.ac.shunts[1].bs_mvar,
                 WithinAbs(-136.0, 1e-12));
    REQUIRE(imported.system.ac.shunts[0].bus ==
            imported.system.ac.branches[0].from_bus);
    REQUIRE(imported.system.ac.shunts[1].bus ==
            imported.system.ac.branches[0].to_bus);
    REQUIRE(skipped_count(imported.report) == 0);
  }

  SECTION("nonzero L-card reactance below the DSP floor is coerced") {
    std::string line = card("L ", "BUS500A", "525.", "BUS500B", "525.");
    line.replace(44, 6, ".00001");
    const std::string content =
        card("BS", "BUS500A", "525.") + "\n" +
        card("B ", "BUS500B", "525.") + "\n" + line + "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.branches.size() == 1);
    REQUIRE_THAT(imported.system.ac.branches.front().x_pu,
                 WithinAbs(0.0001, 1e-12));
    CHECK_FALSE(imported.system.ac.branches.front().ideal_connectivity);
    REQUIRE(std::any_of(
        imported.report.records.begin(), imported.report.records.end(),
        [](const auto& record) {
          return record.disposition ==
                     hacdcpf::io::ImportDisposition::Coerced &&
                 record.reason_code ==
                     hacdcpf::io::ImportReasonCode::RangeCoerced;
        }));
  }

  SECTION("T-card impedance is referred to the second winding bus base") {
    std::string transformer =
        card("T ", "BUS230A", "230.", "BUS035B", "35.");
    transformer.replace(33, 4, "240.");
    transformer.replace(38, 6, ".00093");
    transformer.replace(44, 6, ".06621");
    transformer.replace(61, 4, "230.");
    transformer.replace(67, 4, "37.5");
    const std::string content =
        card("BS", "BUS230A", "230.") + "\n" +
        card("B ", "BUS035B", "35.") + "\n" + transformer + "\n(END)\n";

    const auto imported = hacdcpf::io::parse_bpa_dat_string(content);
    REQUIRE_FALSE(imported.report.has_errors());
    REQUIRE(imported.system.ac.branches.size() == 1);
    const double ratio2 = 37.5 / 35.0;
    REQUIRE_THAT(imported.system.ac.branches.front().r_pu,
                 WithinAbs(0.00093 * ratio2 * ratio2, 1e-12));
    REQUIRE_THAT(imported.system.ac.branches.front().x_pu,
                 WithinAbs(0.06621 * ratio2 * ratio2, 1e-12));
    REQUIRE_THAT(imported.system.ac.branches.front().tap,
                 WithinAbs(1.0 / ratio2, 1e-12));
  }
}

TEST_CASE("BPA import: JSON round-trip", "[bpa][roundtrip]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  const auto json = hacdcpf::io::to_json(res.system, 2);
  const auto json_doc = nlohmann::json::parse(json);
  const auto& json_branches = json_doc.at("ac").at("branches");
  std::size_t line_count = 0;
  std::size_t transformer_count = 0;
  for (const auto& branch : json_branches) {
    const auto kind = branch.at("branch_kind").get<std::string>();
    if (kind == "transformer") {
      ++transformer_count;
    } else {
      REQUIRE(kind == "line");
      ++line_count;
    }
  }
  REQUIRE(line_count == 34);
  REQUIRE(transformer_count == 12);

  auto back = hacdcpf::io::from_json(json);
  REQUIRE(back.ac.buses.size() == res.system.ac.buses.size());
  REQUIRE(back.ac.branches.size() == res.system.ac.branches.size());
  REQUIRE(back.ac.generators.size() == res.system.ac.generators.size());
  REQUIRE(back.ac.loads.size() == res.system.ac.loads.size());
  const auto roundtrip_doc =
      nlohmann::json::parse(hacdcpf::io::to_json(back, 2));
  const auto& roundtrip_branches = roundtrip_doc.at("ac").at("branches");
  REQUIRE(roundtrip_branches.size() == json_branches.size());
  for (size_t i = 0; i < json_branches.size(); ++i) {
    REQUIRE(roundtrip_branches[i].at("branch_kind") ==
            json_branches[i].at("branch_kind"));
  }
}

TEST_CASE("BPA import: 2DC LCC quasi-steady structure", "[bpa][lcc]") {
  hacdcpf::io::BpaImportOptions options;
  options.lcc_model = hacdcpf::io::BpaLccModel::LccQuasiSteady;
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"), options);
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  // Native LCC elements replace the VSC stand-ins; the DC line is unchanged.
  REQUIRE(sys.vsc_converters.empty());
  REQUIRE(sys.lcc_converters.size() == 2);
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE_THAT(sys.dc.branches.front().r_pu, WithinAbs(0.004, 1e-9));

  // Roles follow the LD card terminal order: RECTFIER first, INVERTER second.
  const auto& rect = sys.lcc_converters[0];
  const auto& inv = sys.lcc_converters[1];
  REQUIRE(rect.station_role == hacdcpf::LCCStationRole::Rectifier);
  REQUIRE(inv.station_role == hacdcpf::LCCStationRole::Inverter);

  // BD card physical parameters (both stations share the same card values).
  for (const auto* st : {&rect, &inv}) {
    REQUIRE(st->n_bridges == 2);
    REQUIRE_THAT(st->alpha_min_deg, WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(st->alpha_stop_deg, WithinAbs(140.0, 1e-9));
    REQUIRE_THAT(st->rated_current_a, WithinAbs(3000.0, 1e-9));
    REQUIRE_THAT(st->smoothing_reactor_mh, WithinAbs(200.0, 1e-9));
    REQUIRE_THAT(st->vn_ac_kv, WithinAbs(217.0, 1e-9));
    REQUIRE_THAT(st->rated_dc_kv, WithinAbs(500.0, 1e-9));
    REQUIRE_FALSE(st->model_scope.empty());
    REQUIRE_FALSE(st->model_limitations.empty());
  }
  // 2DC leaves the valve drop blank.
  REQUIRE_THAT(rect.v_drop_v, WithinAbs(0.0, 1e-9));

  // LD card control data: Psch = 1500 MW, rectifier-side Vdc = 500 kV,
  // AlphaN = 15 deg, GamaN = 17 deg.
  REQUIRE(rect.control_mode == hacdcpf::LCCControlMode::ConstantPower);
  REQUIRE(rect.external_control_code == "PAAL");
  REQUIRE(rect.tap_control_modelled);
  REQUIRE_THAT(rect.p_set_mw, WithinAbs(1500.0, 1e-9));
  REQUIRE_THAT(rect.v_dc_set_kv, WithinAbs(500.0, 1e-9));
  REQUIRE_THAT(rect.alpha_set_deg, WithinAbs(15.0, 1e-9));
  REQUIRE(inv.control_mode == hacdcpf::LCCControlMode::ConstantGamma);
  REQUIRE(inv.external_control_code == "VDGA");
  REQUIRE(inv.tap_control_modelled);
  REQUIRE_THAT(inv.gamma_set_deg, WithinAbs(17.0, 1e-9));

  // Each 2DC R card adjusts winding 1 continuously from 400 to 650 kV on a
  // 500 kV primary. R supplies control metadata for the existing T branch; it
  // does not create another electrical element.
  for (const auto* st : {&rect, &inv}) {
    REQUIRE_THAT(st->transformer_tap_min_pu, WithinAbs(400.0 / 500.0, 1e-12));
    REQUIRE_THAT(st->transformer_tap_max_pu, WithinAbs(650.0 / 500.0, 1e-12));
    REQUIRE(st->transformer_tap_steps == 0);
    REQUIRE(st->transformer_tap_winding == 1);

    // T carries the 0.00833 pu AC-parallel equivalent. With two bridge
    // transformers parallel on AC and series on DC, one bridge has twice the
    // branch-equivalent leakage for the LCC characteristic.
    REQUIRE(st->converter_transformer_branch >= 0);
    REQUIRE_THAT(st->x_comm_pu, WithinAbs(0.01666, 1e-9));
    REQUIRE_THAT(st->x_comm_base_mva, WithinAbs(100.0, 1e-9));
    REQUIRE_THAT(st->x_comm_ohm, WithinAbs(7.8450274, 1e-8));
  }

  // Static validation accepts the reconstructed model.
  const auto vrep = hacdcpf::validation::validate(sys);
  REQUIRE(vrep.ok());
}

TEST_CASE("BPA import: cigre LCC quasi-steady structure", "[bpa][lcc]") {
  hacdcpf::io::BpaImportOptions options;
  options.lcc_model = hacdcpf::io::BpaLccModel::LccQuasiSteady;
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"), options);
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.vsc_converters.empty());
  REQUIRE(sys.lcc_converters.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  const auto& dc_line = sys.dc.branches.front();
  REQUIRE_THAT(dc_line.r_pu, WithinAbs(0.004, 1e-9));
  REQUIRE_THAT(dc_line.length_km, WithinAbs(1067.0, 1e-9));
  REQUIRE_THAT(dc_line.r_ohm_per_km,
               WithinAbs(10.0 / 1067.0, 1e-12));
  REQUIRE_THAT(dc_line.r_ohm_per_km * dc_line.length_km,
               WithinAbs(10.0, 1e-9));

  const auto& rect = sys.lcc_converters[0];
  const auto& inv = sys.lcc_converters[1];
  REQUIRE(rect.station_role == hacdcpf::LCCStationRole::Rectifier);
  REQUIRE(inv.station_role == hacdcpf::LCCStationRole::Inverter);
  REQUIRE(rect.converter_transformer_branch !=
          inv.converter_transformer_branch);
  // cigre BD cards carry the 100 V valve drop and 525 kV primary base.
  for (const auto* st : {&rect, &inv}) {
    REQUIRE_THAT(st->v_drop_v, WithinAbs(100.0, 1e-9));
    REQUIRE_THAT(st->rated_current_a, WithinAbs(3000.0, 1e-9));
    REQUIRE_THAT(st->x_comm_pu, WithinAbs(0.01666, 1e-9));
    REQUIRE_THAT(st->x_comm_base_mva, WithinAbs(100.0, 1e-9));
    REQUIRE_THAT(st->x_comm_ohm, WithinAbs(7.8450274, 1e-8));
    REQUIRE(st->converter_transformer_branch >= 0);
    const auto branch = std::find_if(
        sys.ac.branches.begin(), sys.ac.branches.end(), [&](const auto& item) {
          return item.index == st->converter_transformer_branch;
        });
    REQUIRE(branch != sys.ac.branches.end());
    REQUIRE(branch->name.rfind("T_", 0) == 0);
    REQUIRE_THAT(branch->sn_mva, WithinAbs(1800.0, 1e-9));
    REQUIRE_THAT(branch->vn_hv_kv, WithinAbs(525.0, 1e-9));
    REQUIRE_THAT(branch->vn_lv_kv, WithinAbs(217.0, 1e-9));
  }
  REQUIRE_THAT(rect.p_set_mw, WithinAbs(1500.0, 1e-9));
  REQUIRE_THAT(rect.v_dc_set_kv, WithinAbs(500.0, 1e-9));
  REQUIRE_THAT(rect.alpha_set_deg, WithinAbs(15.0, 1e-9));
  REQUIRE_THAT(inv.gamma_set_deg, WithinAbs(17.0, 1e-9));
  REQUIRE(rect.external_control_code == "PAAL");
  REQUIRE(inv.external_control_code == "VDGA");
  REQUIRE(rect.tap_control_modelled);
  REQUIRE(inv.tap_control_modelled);
  REQUIRE_THAT(rect.transformer_tap_min_pu,
               WithinAbs(475.0 / 525.0, 1e-12));
  REQUIRE_THAT(rect.transformer_tap_max_pu,
               WithinAbs(610.0 / 525.0, 1e-12));
  REQUIRE_THAT(inv.transformer_tap_min_pu,
               WithinAbs(498.75 / 525.0, 1e-12));
  REQUIRE_THAT(inv.transformer_tap_max_pu,
               WithinAbs(603.75 / 525.0, 1e-12));
  REQUIRE(rect.transformer_tap_steps == 0);
  REQUIRE(inv.transformer_tap_steps == 0);
  REQUIRE(rect.transformer_tap_winding == 1);
  REQUIRE(inv.transformer_tap_winding == 1);

  const auto vrep = hacdcpf::validation::validate(sys);
  REQUIRE(vrep.ok());
}

TEST_CASE("BPA import: LCC JSON round-trip", "[bpa][lcc][roundtrip]") {
  hacdcpf::io::BpaImportOptions options;
  options.lcc_model = hacdcpf::io::BpaLccModel::LccQuasiSteady;
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"), options);
  REQUIRE(res.system.lcc_converters.size() == 2);

  const auto json = hacdcpf::io::to_json(res.system, 2);
  auto back = hacdcpf::io::from_json(json);
  REQUIRE(back.lcc_converters.size() == 2);
  REQUIRE(back.vsc_converters.empty());
  for (size_t i = 0; i < 2; ++i) {
    const auto& a = res.system.lcc_converters[i];
    const auto& b = back.lcc_converters[i];
    REQUIRE(b.index == a.index);
    REQUIRE(b.name == a.name);
    REQUIRE(b.ac_bus == a.ac_bus);
    REQUIRE(b.dc_bus == a.dc_bus);
    REQUIRE(b.station_role == a.station_role);
    REQUIRE(b.control_mode == a.control_mode);
    REQUIRE(b.n_bridges == a.n_bridges);
    REQUIRE_THAT(b.alpha_min_deg, WithinAbs(a.alpha_min_deg, 1e-12));
    REQUIRE_THAT(b.alpha_stop_deg, WithinAbs(a.alpha_stop_deg, 1e-12));
    REQUIRE_THAT(b.gamma_min_deg, WithinAbs(a.gamma_min_deg, 1e-12));
    REQUIRE_THAT(b.v_drop_v, WithinAbs(a.v_drop_v, 1e-12));
    REQUIRE_THAT(b.rated_current_a, WithinAbs(a.rated_current_a, 1e-12));
    REQUIRE_THAT(b.x_comm_pu, WithinAbs(a.x_comm_pu, 1e-12));
    REQUIRE_THAT(b.x_comm_base_mva, WithinAbs(a.x_comm_base_mva, 1e-12));
    REQUIRE_THAT(b.x_comm_ohm, WithinAbs(a.x_comm_ohm, 1e-12));
    REQUIRE_THAT(b.rated_dc_kv, WithinAbs(a.rated_dc_kv, 1e-12));
    REQUIRE_THAT(b.vn_ac_kv, WithinAbs(a.vn_ac_kv, 1e-12));
    REQUIRE_THAT(b.smoothing_reactor_mh,
                 WithinAbs(a.smoothing_reactor_mh, 1e-12));
    REQUIRE_THAT(b.p_set_mw, WithinAbs(a.p_set_mw, 1e-12));
    REQUIRE_THAT(b.i_set_ka, WithinAbs(a.i_set_ka, 1e-12));
    REQUIRE_THAT(b.alpha_set_deg, WithinAbs(a.alpha_set_deg, 1e-12));
    REQUIRE_THAT(b.gamma_set_deg, WithinAbs(a.gamma_set_deg, 1e-12));
    REQUIRE_THAT(b.v_dc_set_kv, WithinAbs(a.v_dc_set_kv, 1e-12));
    REQUIRE(b.external_control_code == a.external_control_code);
    REQUIRE(b.tap_control_modelled == a.tap_control_modelled);
    REQUIRE(b.converter_transformer_branch == a.converter_transformer_branch);
    REQUIRE_THAT(b.transformer_tap_min_pu,
                 WithinAbs(a.transformer_tap_min_pu, 1e-12));
    REQUIRE_THAT(b.transformer_tap_max_pu,
                 WithinAbs(a.transformer_tap_max_pu, 1e-12));
    REQUIRE(b.transformer_tap_steps == a.transformer_tap_steps);
    REQUIRE(b.transformer_tap_winding == a.transformer_tap_winding);
    REQUIRE(b.model_scope == a.model_scope);
    REQUIRE(b.model_limitations == a.model_limitations);
  }

  std::size_t transformer_branch_count = 0;
  for (const auto& original : res.system.ac.branches) {
    if (original.name.rfind("T_", 0) != 0) continue;
    ++transformer_branch_count;
    const auto* restored = find_branch(back, original.name);
    REQUIRE(restored != nullptr);
    REQUIRE(restored->n_parallel == original.n_parallel);
    REQUIRE_THAT(restored->sn_mva, WithinAbs(original.sn_mva, 1e-12));
    REQUIRE_THAT(restored->vn_hv_kv, WithinAbs(original.vn_hv_kv, 1e-12));
    REQUIRE_THAT(restored->vn_lv_kv, WithinAbs(original.vn_lv_kv, 1e-12));
  }
  REQUIRE(transformer_branch_count == 2);
}

TEST_CASE("LCC tap-control metadata requires a valid transformer range",
          "[bpa][lcc][validation]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  REQUIRE(res.system.lcc_converters.size() == 2);
  REQUIRE(hacdcpf::validation::validate(res.system).ok());
  res.system.lcc_converters.front().transformer_tap_min_pu = 0.0;

  const auto report = hacdcpf::validation::validate(res.system);
  REQUIRE_FALSE(report.ok());
  const auto issue = std::find_if(
      report.issues.begin(), report.issues.end(), [](const auto& item) {
        return item.component_type == "LCCConverter" &&
               item.field == "transformer_tap_min_pu" &&
               item.severity == hacdcpf::validation::Severity::Error;
      });
  REQUIRE(issue != report.issues.end());
}

TEST_CASE("BPA import: default is LccQuasiSteady", "[bpa][lcc]") {
  // The default options import the native quasi-steady LCC model; the legacy
  // VSC approximation remains available as an explicit option.
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"));
  REQUIRE(res.system.vsc_converters.empty());
  REQUIRE(res.system.lcc_converters.size() == 2);

  hacdcpf::io::BpaImportOptions legacy;
  legacy.lcc_model = hacdcpf::io::BpaLccModel::VscApprox;
  auto res_vsc = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"), legacy);
  REQUIRE(res_vsc.system.vsc_converters.size() == 2);
  REQUIRE(res_vsc.system.lcc_converters.empty());
}

TEST_CASE("BPA export: fixed-column round-trip", "[bpa][roundtrip][export]") {
  for (const auto* filename : {"39.dat", "2DC.dat"}) {
    const auto imported = hacdcpf::io::parse_bpa_dat(dat_path(filename));
    REQUIRE_FALSE(imported.report.has_errors());

    const std::string dat = hacdcpf::io::to_bpa_dat(imported.system);
    REQUIRE(dat.find("/MVA_BASE=100") != std::string::npos);
    REQUIRE(dat.find("A0000001") != std::string::npos);
    REQUIRE(dat.find("(END)") != std::string::npos);

    const auto round_trip = hacdcpf::io::parse_bpa_dat_string(dat);
    REQUIRE_FALSE(round_trip.report.has_errors());
    REQUIRE(round_trip.system.ac.buses.size() == imported.system.ac.buses.size());
    REQUIRE(round_trip.system.ac.branches.size() == imported.system.ac.branches.size());
    REQUIRE(round_trip.system.ac.generators.size() == imported.system.ac.generators.size());
    REQUIRE(round_trip.system.ac.loads.size() == imported.system.ac.loads.size());
    REQUIRE(round_trip.system.dc.buses.size() == imported.system.dc.buses.size());
    REQUIRE(round_trip.system.dc.branches.size() == imported.system.dc.branches.size());
    REQUIRE(round_trip.system.vsc_converters.size() == imported.system.vsc_converters.size());
  }
}

// DSP reference solutions (Samples/39bus/39NEW.SOL), name -> (vm, va_deg).
static const std::unordered_map<std::string, std::pair<double, double>>
    kSol39 = {
        {"new1", {1.047575, 1.487873}},   {"new2", {1.048977, 3.966515}},
        {"new3", {1.031298, 1.094970}},   {"new4", {1.006304, 0.267400}},
        {"new5", {1.008868, 1.424903}},   {"new6", {1.011043, 2.115523}},
        {"new7", {1.000283, -0.056716}},  {"new8", {0.999240, -0.551302}},
        {"new9", {1.029554, -0.263977}},  {"new10", {1.019182, 4.509627}},
        {"new11", {1.015197, 3.693142}},  {"new12", {1.002276, 3.695372}},
        {"new13", {1.016338, 3.802513}},  {"new14", {1.013760, 2.148957}},
        {"new15", {1.016729, 1.753573}},  {"new16", {1.032714, 3.164391}},
        {"new17", {1.034451, 2.173402}},  {"new18", {1.031940, 1.334891}},
        {"new19", {1.049769, 7.790752}},  {"new20", {0.990567, 6.378937}},
        {"new21", {1.031873, 5.568243}},  {"new22", {1.049711, 10.017804}},
        {"new23", {1.044762, 9.819463}},  {"new24", {1.038110, 3.284089}},
        {"new25", {1.057297, 5.332769}},  {"new26", {1.052123, 4.059755}},
        {"new27", {1.038195, 2.040314}},  {"new28", {1.049837, 7.573736}},
        {"new29", {1.049558, 10.335159}}, {"new30", {1.047000, 6.386712}},
        {"new31", {0.982000, 10.693116}}, {"new32", {0.983000, 12.487224}},
        {"new33", {0.997000, 13.009079}}, {"new34", {1.012000, 11.571776}},
        {"new35", {1.049000, 14.981018}}, {"new36", {1.063000, 17.676756}},
        {"new37", {1.027000, 12.124703}}, {"new38", {1.026000, 17.404641}},
        {"new39", {1.030000, 0.000000}},
};

TEST_CASE("BPA import: 39-bus PF matches DSP solution", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);

  double max_dv = 0.0, max_da = 0.0;
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  for (const auto& bus : res.system.ac.buses) {
    const auto it = kSol39.find(bus.name);
    REQUIRE(it != kSol39.end());
    const size_t i = static_cast<size_t>(bus.index) - 1;
    const double va_deg = pf.va[i] * kDeg;  // solver reports radians
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - it->second.first));
    max_da = std::max(max_da, std::abs(va_deg - it->second.second));
  }
  REQUIRE(max_dv < 2e-3);
  REQUIRE(max_da < 0.5);
}

// DSP reference solution (Samples/IEEE90/IEEE90NEW.SOL).
static const std::unordered_map<std::string, std::pair<double, double>>
    kSolIeee90 = {
        {"发电机1", {1.010000, 0.000000}}, {"母线1", {1.038771, -3.436394}},
        {"母线A", {1.006125, -6.161531}},  {"母线B", {1.022197, -5.463977}},
        {"母线C", {1.031936, -3.145088}},  {"母线2", {1.042976, -0.746674}},
        {"发电机2", {1.010000, 5.093616}}, {"母线3", {1.053444, -1.307074}},
        {"发电机3", {1.010000, 1.516309}},
};

TEST_CASE("BPA import: IEEE90 PF matches DSP solution", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("IEEE90.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);

  // DSP applies its own shunt/Q-limit corrections that the importer does not
  // replicate; the residual deviation is ~2.6e-3 pu / 0.4 deg (the 39-bus
  // case, which carries no shunts, matches to <2e-3 / 0.5 deg).
  double max_dv = 0.0, max_da = 0.0;
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  for (const auto& bus : res.system.ac.buses) {
    const auto it = kSolIeee90.find(bus.name);
    REQUIRE(it != kSolIeee90.end());
    const size_t i = static_cast<size_t>(bus.index) - 1;
    const double va_deg = pf.va[i] * kDeg;  // solver reports radians
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - it->second.first));
    max_da = std::max(max_da, std::abs(va_deg - it->second.second));
  }
  REQUIRE(max_dv < 5e-3);
  REQUIRE(max_da < 0.5);
}

TEST_CASE("BPA import: 2DC HVDC link transfers scheduled power", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"));  // default: LccQuasiSteady
  hacdcpf::PowerFlowOptions opt;  // default solver options (as used by the GUI)
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);
  // Native LCC quasi-steady stations replace the legacy VSC stand-ins.
  REQUIRE(pf.vsc_transfers.empty());
  REQUIRE(pf.lcc_transfers.size() == 2);
  // Rectifier (station 0) draws the scheduled 1500 MW from its AC system,
  // exactly as in the DSP solution (constant-power control).
  REQUIRE(pf.lcc_transfers[0].station_role ==
          static_cast<int>(hacdcpf::LCCStationRole::Rectifier));
  REQUIRE_THAT(pf.lcc_transfers[0].p_ac_mw, WithinAbs(-1500.0, 1.0));
  // R-card tap control holds the LD normal angles while preserving the
  // 3 kA, 500/470 kV DC operating point reported by DSP.
  REQUIRE(pf.lcc_transfers[1].station_role ==
          static_cast<int>(hacdcpf::LCCStationRole::Inverter));
  REQUIRE(pf.lcc_transfers[1].id_at_limit);
  REQUIRE_THAT(pf.lcc_transfers[1].p_ac_mw, WithinAbs(1410.0, 1.0));
  REQUIRE_THAT(pf.lcc_transfers[1].ud_kv, WithinAbs(470.0, 1.0));
  REQUIRE_THAT(pf.lcc_transfers[0].alpha_deg, WithinAbs(15.0, 0.1));
  REQUIRE_THAT(pf.lcc_transfers[1].gamma_deg, WithinAbs(17.0, 0.1));
  // The transformer-aware quasi-steady model stays within 0.25% of DSP Q.
  REQUIRE_THAT(pf.lcc_transfers[0].q_ac_mvar, WithinAbs(-526.54, 1.5));
  REQUIRE_THAT(pf.lcc_transfers[1].q_ac_mvar, WithinAbs(-530.91, 1.5));
  REQUIRE_THAT(pf.lcc_transfers[0].transformer_tap,
               WithinAbs(549.25 / 500.0, 2e-3));
  REQUIRE_THAT(pf.lcc_transfers[1].transformer_tap,
               WithinAbs(574.48 / 500.0, 2e-3));
  REQUIRE(pf.lcc_transfers[0].tap_control_active);
  REQUIRE(pf.lcc_transfers[1].tap_control_active);
  REQUIRE(pf.lcc_transfers[0].tap_control_converged);
  REQUIRE(pf.lcc_transfers[1].tap_control_converged);
}

TEST_CASE("BPA import: cigre HVDC link transfers scheduled power", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);
  REQUIRE(pf.vsc_transfers.empty());
  REQUIRE(pf.lcc_transfers.size() == 2);
  REQUIRE_THAT(pf.lcc_transfers[0].p_ac_mw, WithinAbs(-1500.0, 1.0));
  REQUIRE_THAT(pf.lcc_transfers[1].p_ac_mw, WithinAbs(1410.0, 1.0));
  REQUIRE(pf.lcc_transfers[1].id_at_limit);
  REQUIRE_THAT(pf.lcc_transfers[0].alpha_deg, WithinAbs(15.0, 0.1));
  REQUIRE_THAT(pf.lcc_transfers[1].gamma_deg, WithinAbs(17.0, 0.1));
  REQUIRE_THAT(pf.lcc_transfers[0].q_ac_mvar, WithinAbs(-526.54, 1.5));
  REQUIRE_THAT(pf.lcc_transfers[1].q_ac_mvar, WithinAbs(-530.90, 1.5));
  REQUIRE_THAT(pf.lcc_transfers[0].transformer_tap,
               WithinAbs(547.97 / 525.0, 2e-3));
  REQUIRE_THAT(pf.lcc_transfers[1].transformer_tap,
               WithinAbs(571.25 / 525.0, 2e-3));
  REQUIRE(pf.lcc_transfers[0].tap_control_active);
  REQUIRE(pf.lcc_transfers[1].tap_control_active);
  REQUIRE(pf.lcc_transfers[0].tap_control_converged);
  REQUIRE(pf.lcc_transfers[1].tap_control_converged);
}

TEST_CASE("BPA import: solver handle applies deterministic LCC tap control",
          "[bpa][pf][handle]") {
  auto imported = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  const auto ordinary_first = hacdcpf::solve_power_flow(imported.system);
  const auto ordinary_second = hacdcpf::solve_power_flow(imported.system);
  auto* handle = hacdcpf::create_solver_handle(imported.system);
  REQUIRE(handle != nullptr);

  const auto first = hacdcpf::solve_handle(handle);
  const auto second = hacdcpf::solve_handle(handle);
  hacdcpf::destroy_solver_handle(handle);

  REQUIRE(first.converged);
  REQUIRE(second.converged);
  REQUIRE(ordinary_first.converged);
  REQUIRE(ordinary_second.converged);
  REQUIRE(first.lcc_transfers.size() == 2);
  REQUIRE(second.lcc_transfers.size() == first.lcc_transfers.size());
  REQUIRE(ordinary_first.lcc_transfers.size() == first.lcc_transfers.size());
  REQUIRE(ordinary_second.lcc_transfers.size() == first.lcc_transfers.size());
  for (size_t i = 0; i < first.lcc_transfers.size(); ++i) {
    const auto& a = first.lcc_transfers[i];
    const auto& b = second.lcc_transfers[i];
    const auto& c = ordinary_first.lcc_transfers[i];
    const auto& d = ordinary_second.lcc_transfers[i];
    REQUIRE(a.tap_control_active);
    REQUIRE(a.tap_control_converged);
    REQUIRE(a.tap_control_iterations > 0);
    REQUIRE(b.tap_control_active);
    REQUIRE(b.tap_control_converged);
    REQUIRE(b.tap_control_iterations == a.tap_control_iterations);
    REQUIRE_THAT(b.transformer_tap,
                 WithinAbs(a.transformer_tap, 1e-12));
    REQUIRE_THAT(b.alpha_deg, WithinAbs(a.alpha_deg, 1e-10));
    REQUIRE_THAT(b.gamma_deg, WithinAbs(a.gamma_deg, 1e-10));
    REQUIRE_THAT(c.transformer_tap,
                 WithinAbs(a.transformer_tap, 1e-12));
    REQUIRE_THAT(d.transformer_tap,
                 WithinAbs(a.transformer_tap, 1e-12));
    REQUIRE_THAT(c.q_ac_mvar, WithinAbs(a.q_ac_mvar, 1e-10));
    REQUIRE_THAT(d.q_ac_mvar, WithinAbs(a.q_ac_mvar, 1e-10));
  }
}

TEST_CASE("BPA import: R-card tap control reports transformer edge cases",
          "[bpa][pf][tap-control]") {
  auto imported = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  REQUIRE(imported.system.lcc_converters.size() == 2);
  const int station_index = imported.system.lcc_converters[0].index;
  const int transformer_index =
      imported.system.lcc_converters[0].converter_transformer_branch;
  const auto transfer_for = [station_index](const auto& result) {
    return std::find_if(
        result.lcc_transfers.begin(), result.lcc_transfers.end(),
        [station_index](const auto& transfer) {
          return transfer.index == station_index;
        });
  };
  const auto has_warning = [](const auto& result, const std::string& code) {
    return std::any_of(
        result.diagnostics.warnings.begin(),
        result.diagnostics.warnings.end(),
        [&](const std::string& warning) {
          return warning.find(code) != std::string::npos;
        });
  };

  auto transformer_for = [transformer_index](auto& system) {
    return std::find_if(
        system.ac.branches.begin(), system.ac.branches.end(),
        [transformer_index](const auto& branch) {
          return branch.index == transformer_index;
        });
  };

  SECTION("an out-of-service T branch disables its R-card controller") {
    auto transformer = transformer_for(imported.system);
    REQUIRE(transformer != imported.system.ac.branches.end());
    transformer->in_service = false;
    const auto result = hacdcpf::solve_power_flow(imported.system);
    CHECK(has_warning(result, "[LCC-TAP-01]"));
  }

  SECTION("an out-of-range authored tap is normalized on a private snapshot") {
    auto transformer = transformer_for(imported.system);
    REQUIRE(transformer != imported.system.ac.branches.end());
    auto& lcc = imported.system.lcc_converters[0];
    transformer->tap = lcc.transformer_tap_max_pu + 1.0;
    const double authored_tap = transformer->tap;
    const auto result = hacdcpf::solve_power_flow(imported.system);
    REQUIRE(result.converged);
    const auto transfer = transfer_for(result);
    REQUIRE(transfer != result.lcc_transfers.end());
    CHECK(transfer->tap_control_active);
    CHECK(transfer->tap_control_converged);
    CHECK(transfer->tap_control_iterations > 0);
    CHECK(transfer->transformer_tap >= lcc.transformer_tap_min_pu);
    CHECK(transfer->transformer_tap <= lcc.transformer_tap_max_pu);
    CHECK(transformer->tap == authored_tap);
  }

  SECTION("a fixed admissible tap can converge PF without meeting the angle") {
    auto transformer = transformer_for(imported.system);
    REQUIRE(transformer != imported.system.ac.branches.end());
    auto& lcc = imported.system.lcc_converters[0];
    transformer->tap = 1.0;
    lcc.transformer_tap_min_pu = 1.0;
    lcc.transformer_tap_max_pu = 1.0;
    lcc.transformer_tap_steps = 0;
    const auto result = hacdcpf::solve_power_flow(imported.system);
    REQUIRE(result.converged);
    const auto transfer = transfer_for(result);
    REQUIRE(transfer != result.lcc_transfers.end());
    CHECK(transfer->tap_control_active);
    CHECK_FALSE(transfer->tap_control_converged);
    CHECK(transfer->tap_at_limit);
    CHECK(transfer->transformer_tap == 1.0);
    CHECK(has_warning(result, "[LCC-TAP-02]"));
  }
}

TEST_CASE("BPA import: vsc2 VSC-HVDC structure (BZ/BZ+/LZ)", "[bpa][structure][vsc]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("vsc2.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 3);   // SLACK + the two BZ station buses
  REQUIRE(sys.ac.branches.size() == 2);
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE(sys.vsc_converters.size() == 2);
  REQUIRE(sys.lcc_converters.empty());

  // DC network base from the BZ+ rated DC voltage (cols 36-39).
  REQUIRE_THAT(sys.dc.buses[0].base_kv, WithinAbs(300.0, 1e-9));
  REQUIRE_THAT(sys.dc.buses[1].base_kv, WithinAbs(300.0, 1e-9));

  // LZ: R = 0.01 ohm per pole, 2 poles -> 0.005 ohm;
  // r_pu = 0.005 * 100 MVA / (300 kV)^2; rating = 300 kV * 1429 A.
  const auto& dcb = sys.dc.branches.front();
  REQUIRE_THAT(dcb.r_pu, WithinAbs(0.005 * 100.0 / 90000.0, 1e-12));
  REQUIRE_THAT(dcb.base_kv, WithinAbs(300.0, 1e-9));
  REQUIRE_THAT(dcb.rate_a_mva, WithinAbs(300.0 * 1429.0 / 1000.0, 1e-9));

  // VSCA: no flag on BZ+ -> constant P/Q station.  Card P/Q use the BPA
  // load sign convention (positive absorbs); the model uses injection
  // signs: card P -1470 -> +1470 injection (inverter), card Q +50
  // absorbed -> q_set = -50.
  const auto& va = sys.vsc_converters[0];
  REQUIRE(va.control_mode == hacdcpf::ConverterMode::PQ_MODE);
  REQUIRE_THAT(va.p_set_mw, WithinAbs(1470.0, 1e-9));
  REQUIRE_THAT(va.q_set_mvar, WithinAbs(-50.0, 1e-9));
  REQUIRE_THAT(va.vn_dc_kv, WithinAbs(300.0, 1e-9));
  // Rc = 0.007 pu -> eta = 1 - Rc (DSP books |P|*Rc as the station loss).
  REQUIRE_THAT(va.eta, WithinAbs(0.993, 1e-9));

  // VSCB: flag 1 -> constant DC voltage station; Udc set 300 kV = 1.0 pu.
  const auto& vb = sys.vsc_converters[1];
  REQUIRE(vb.control_mode == hacdcpf::ConverterMode::VDC_Q);
  REQUIRE_THAT(vb.v_dc_set_pu, WithinAbs(1.0, 1e-9));
  REQUIRE(vb.k_vdc > 100.0);
  REQUIRE_THAT(vb.q_set_mvar, WithinAbs(50.0, 1e-9));
  // Card P is a schedule/warm-start only (P is free in VDC_Q).
  REQUIRE_THAT(vb.p_schedule_mw, WithinAbs(-1470.0, 1e-9));
}

TEST_CASE("BPA import: BZ+ without a BZ card is an error", "[bpa][vsc]") {
  const std::string content =
      "( POWERFLOW,CASEID=T)\n"
      "BS    SLACK   525. 1                       10001500. -5001050\n"
      "BZ+   VSCB    525. 1470. -50.    1 300. 300.        .01043 525.  300.\n"
      "(END)\n";
  auto res = hacdcpf::io::parse_bpa_dat_string(content);
  REQUIRE(res.report.has_errors());
  REQUIRE(res.system.vsc_converters.empty());
}

TEST_CASE("BPA import: LZ referencing a non-BZ station warns and skips",
          "[bpa][vsc]") {
  const std::string content =
      "( POWERFLOW,CASEID=T)\n"
      "BS    SLACK   525. 1                       10001500. -5001050\n"
      "BZ    VSCA    525.51 1500        .06    286 15000 .0070   1.   2  300.\n"
      "BZ+   VSCA    525. -1470 50.       300. 300.        .01043 525.  300.\n"
      "LZ    VSCA    525. GHOST   525.  1429.01   .001  10.\n"
      "(END)\n";
  auto res = hacdcpf::io::parse_bpa_dat_string(content);
  // Warning path (not an error): the line is dropped, the station remains.
  REQUIRE_FALSE(res.report.has_errors());
  REQUIRE(res.system.dc.branches.empty());
  REQUIRE(res.system.vsc_converters.size() == 1);
  bool rejected_lz = false;
  for (const auto& rec : res.report.records) {
    if (rec.disposition == hacdcpf::io::ImportDisposition::Rejected &&
        rec.message.find("LZ card") != std::string::npos) {
      rejected_lz = true;
    }
  }
  REQUIRE(rejected_lz);
}

TEST_CASE("BPA import: BZ station without BZ+ uses zero-setpoint defaults",
          "[bpa][vsc]") {
  const std::string content =
      "( POWERFLOW,CASEID=T)\n"
      "BS    SLACK   525. 1                       10001500. -5001050\n"
      "BZ    VSCA    525.51 1500        .06    286 15000 .0070   1.   2  300.\n"
      "(END)\n";
  auto res = hacdcpf::io::parse_bpa_dat_string(content);
  REQUIRE_FALSE(res.report.has_errors());
  REQUIRE(res.system.vsc_converters.size() == 1);
  const auto& c = res.system.vsc_converters[0];
  REQUIRE(c.control_mode == hacdcpf::ConverterMode::PQ_MODE);
  REQUIRE_THAT(c.p_set_mw, WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(c.q_set_mvar, WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(c.vn_dc_kv, WithinAbs(300.0, 1e-9));  // BZ cols 67-70 fallback
  REQUIRE_THAT(c.eta, WithinAbs(0.993, 1e-9));
}

TEST_CASE("BPA import: vsc2 VSC link solves and transfers scheduled power",
          "[bpa][pf][vsc]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("vsc2.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  INFO("termination: " + pf.diagnostics.termination_reason);
  for (const auto& w : pf.diagnostics.warnings) INFO("warning: " + w);
  REQUIRE(pf.converged);
  REQUIRE(pf.vsc_transfers.size() == 2);
  REQUIRE(pf.lcc_transfers.empty());
  // Constant-P/Q station (inverter): exactly the card setpoints at the AC
  // terminal (+1470 MW injected; 50 Mvar absorbed).
  REQUIRE_THAT(pf.vsc_transfers[0].p_ac_mw, WithinAbs(1470.0, 1.0));
  REQUIRE_THAT(pf.vsc_transfers[0].q_ac_mvar, WithinAbs(-50.0, 1.0));
  // It draws P*(1+Rc) = 1470*1.007 from the DC network; the constant-Udc
  // station (rectifier) covers that plus the line loss and absorbs
  // ~(1480.2 + 0.1) * 1.007 = ~1491.4 MW from its AC grid.
  REQUIRE_THAT(pf.vsc_transfers[0].p_dc_mw, WithinAbs(-1480.3, 1.0));
  REQUIRE_THAT(pf.vsc_transfers[1].p_ac_mw, WithinAbs(-1491.5, 2.0));
  // DC voltage: held at 300 kV at station B, I*R drop at station A.
  const double udc_b = pf.vdc[static_cast<size_t>(
                                res.system.vsc_converters[1].bus_dc) - 1] *
                       300.0;
  const double udc_a = pf.vdc[static_cast<size_t>(
                                res.system.vsc_converters[0].bus_dc) - 1] *
                       300.0;
  REQUIRE_THAT(udc_b, WithinAbs(300.0, 0.1));
  REQUIRE_THAT(udc_a, WithinAbs(299.97, 0.1));
}
