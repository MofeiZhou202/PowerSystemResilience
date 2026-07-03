// Bounded CIM / CGMES 3.0 (EQ+SSH) import/export contract (§5).
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

#include "hacdcpf/io/cim_io.hpp"

namespace {
bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
}  // namespace

TEST_CASE("CIM CGMES bounded round-trip preserves the EQ+SSH subset",
          "[io][cim]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus b1; b1.index = 1; b1.base_kv = 110.0; b1.name = "B1";
  ACBus b2; b2.index = 2; b2.base_kv = 110.0; b2.name = "B2";
  ACBus b3; b3.index = 3; b3.base_kv = 20.0;  b3.name = "B3";
  sys.ac.buses = {b1, b2, b3};

  ACBranch l1;
  l1.index = 1; l1.from_bus = 1; l1.to_bus = 2;
  l1.r_pu = 0.01; l1.x_pu = 0.10; l1.b_pu = 0.02; l1.name = "L1";
  ACBranch l2;
  l2.index = 2; l2.from_bus = 2; l2.to_bus = 3;
  l2.r_pu = 0.02; l2.x_pu = 0.20; l2.b_pu = 0.0; l2.name = "L2";
  sys.ac.branches = {l1, l2};

  Generator g;
  g.index = 1; g.bus = 1; g.name = "G1";
  g.pmin_mw = 0.0; g.pmax_mw = 100.0;
  g.qmin_mvar = -50.0; g.qmax_mvar = 50.0;
  g.mbase_mva = 120.0; g.pg_mw = 40.0;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1; ld.bus = 3; ld.name = "LD1"; ld.p_mw = 30.0; ld.q_mvar = 10.0;
  sys.ac.loads = {ld};

  const std::string xml = io::to_cim(sys);
  INFO(xml);
  const auto res = io::from_cim(xml);
  const auto& r = res.system;

  REQUIRE(r.ac.buses.size() == 3);
  REQUIRE(r.ac.branches.size() == 2);
  REQUIRE(r.ac.generators.size() == 1);
  REQUIRE(r.ac.loads.size() == 1);

  auto kv = [&](int idx) {
    for (const auto& b : r.ac.buses)
      if (b.index == idx) return b.base_kv;
    return -1.0;
  };
  CHECK(near(kv(1), 110.0));
  CHECK(near(kv(3), 20.0));

  const auto& rl1 = r.ac.branches.front();
  CHECK(near(rl1.r_pu, 0.01));
  CHECK(near(rl1.x_pu, 0.10));
  CHECK(near(rl1.b_pu, 0.02));
  CHECK(rl1.from_bus == 1);
  CHECK(rl1.to_bus == 2);

  CHECK(near(r.ac.generators.front().pmax_mw, 100.0));
  CHECK(near(r.ac.generators.front().mbase_mva, 120.0));
  CHECK(r.ac.generators.front().bus == 1);

  CHECK(near(r.ac.loads.front().p_mw, 30.0));
  CHECK(near(r.ac.loads.front().q_mvar, 10.0));
  CHECK(r.ac.loads.front().bus == 3);

  CHECK(res.report.binding_level == io::ImportBindingLevel::Rich);
  CHECK_FALSE(res.report.has_errors());
}

TEST_CASE("CIM import drops out-of-scope classes with a structural-loss record",
          "[io][cim]") {
  const std::string xml =
      "<?xml version=\"1.0\"?>\n"
      "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" "
      "xmlns:cim=\"http://iec.ch/TC57/CIM100#\">\n"
      "  <cim:PowerTransformer rdf:ID=\"_t1\">\n"
      "    <cim:IdentifiedObject.name>T1</cim:IdentifiedObject.name>\n"
      "  </cim:PowerTransformer>\n"
      "</rdf:RDF>\n";
  const auto res = hacdcpf::io::from_cim(xml);
  bool structural_loss = false;
  for (const auto& rec : res.report.records)
    if (rec.reason_code == hacdcpf::io::ImportReasonCode::StructuralLoss)
      structural_loss = true;
  CHECK(structural_loss);
}

TEST_CASE("CIM parser rejects DOCTYPE/ENTITY (XXE hardening, §12)",
          "[io][cim][security]") {
  const std::string evil =
      "<?xml version=\"1.0\"?>\n"
      "<!DOCTYPE foo [ <!ENTITY xxe SYSTEM \"file:///etc/passwd\"> ]>\n"
      "<rdf:RDF></rdf:RDF>\n";
  const auto res = hacdcpf::io::from_cim(evil);
  CHECK(res.report.has_errors());
}
