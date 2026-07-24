#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/tntp_parser.hpp"

using Catch::Approx;

namespace {

std::string sioux_falls_file(const std::string& name) {
  return std::string(HACDCPF_PROJECT_ROOT) +
         "/external_data/transportation/SiouxFalls/" + name;
}

}  // namespace

TEST_CASE("TNTP parser imports Sioux Falls network and OD table",
          "[io][tntp][sioux_falls]") {
  const auto network = hacdcpf::io::parse_tntp_network(
      sioux_falls_file("SiouxFalls_net.tntp"));
  const auto nodes = hacdcpf::io::parse_tntp_nodes(
      sioux_falls_file("SiouxFalls_node.tntp"));
  const auto trips = hacdcpf::io::parse_tntp_trips(
      sioux_falls_file("SiouxFalls_trips.tntp"));

  REQUIRE(network.number_of_zones == 24);
  REQUIRE(network.number_of_nodes == 24);
  REQUIRE(network.first_thru_node == 1);
  REQUIRE(network.links.size() == 76);
  REQUIRE(nodes.size() == 24);
  REQUIRE(trips.number_of_zones == 24);
  REQUIRE(trips.demand.size() == 24);
  CHECK(trips.total_flow() == Approx(360600.0));
  CHECK(trips.flow(1, 2) == Approx(100.0));
  CHECK(trips.flow(24, 22) == Approx(1100.0));
  CHECK(trips.flow(24, 23) == Approx(700.0));

  hacdcpf::io::TNTPImportOptions options;
  options.time_unit = hacdcpf::io::TNTPTimeUnit::Minutes;
  options.length_unit = hacdcpf::io::TNTPLengthUnit::Miles;
  const auto graph = hacdcpf::io::make_traffic_graph(network, nodes, options);

  REQUIRE(graph.nodes.size() == 24);
  REQUIRE(graph.links.size() == 76);
  CHECK(graph.nodes.front().x == Approx(-96.77041974));
  CHECK(graph.nodes.front().y == Approx(43.61282792));
  CHECK(graph.links.front().from_node == 1);
  CHECK(graph.links.front().to_node == 2);
  CHECK(graph.links.front().length_km == Approx(6.0 * 1.609344));
  CHECK(graph.links.front().free_flow_time_hr == Approx(0.1));
  CHECK(graph.links.front().capacity_veh_per_hr == Approx(25900.20064));
  CHECK(graph.links.front().jam_vehicles ==
        Approx(2.0 * 25900.20064 * 0.1));
}
