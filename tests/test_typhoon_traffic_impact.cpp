#include <chrono>
#include <filesystem>
#include <future>
#include <limits>
#include <numeric>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/analysis/typhoon_traffic_impact.hpp"
#include "hacdcpf/ev_power_traffic/simulation.hpp"

using Catch::Approx;

namespace {

hacdcpf::analysis::TyphoonTrackPoint storm_point(double hour) {
  hacdcpf::analysis::TyphoonTrackPoint point;
  point.hour = hour;
  point.latitude = 22.75;
  point.longitude = 113.55;
  point.delta_p_hpa = 70.0;
  point.holland_b = 1.2;
  point.rmw_km = 40.0;
  point.heading_deg = 0.0;
  point.translation_speed_kmph = 18.0;
  point.vmax_ms = 45.0;
  return point;
}

}  // namespace

TEST_CASE("Typhoon road impact creates common speed, capacity, and closure profiles",
          "[scenario_generation][typhoon][traffic]") {
  hacdcpf::evpt::TrafficGraph traffic;
  traffic.nodes = {{1, "west", 113.93, 22.75},
                   {2, "east", 113.95, 22.75}};
  hacdcpf::evpt::TrafficLink link;
  link.index = 10;
  link.from_node = 1;
  link.to_node = 2;
  link.length_km = 2.0;
  link.free_flow_time_hr = 0.04;
  link.capacity_veh_per_hr = 1000.0;
  link.jam_vehicles = 100.0;
  traffic.links.push_back(link);

  std::vector<hacdcpf::analysis::TyphoonTrackPoint> track;
  for (int hour = 0; hour <= 4; ++hour) {
    track.push_back(storm_point(static_cast<double>(hour)));
  }

  hacdcpf::analysis::TyphoonTrafficImpactOptions options;
  options.num_steps = 4;
  options.time_step_hr = 1.0;
  options.coordinate_mode =
      hacdcpf::analysis::TyphoonTrafficCoordinateMode::Geographic;
  options.drainage_rate_mm_hr = 0.0;
  options.flood_closure_depth_mm = 5.0;

  const auto result = hacdcpf::analysis::apply_typhoon_traffic_impact(
      traffic, track, options);
  REQUIRE(result.impacted_traffic.links.size() == 1);
  const auto& impacted = result.impacted_traffic.links.front();
  REQUIRE(impacted.free_flow_time_profile_hr.size() == 4);
  REQUIRE(impacted.capacity_profile_veh_per_hr.size() == 4);
  REQUIRE(impacted.availability_profile.size() == 4);
  CHECK_FALSE(result.used_affine_georeferencing);
  CHECK(result.peak_wind_ms > 0.0);
  CHECK(result.peak_rainfall_mm_hr > 0.0);
  CHECK(result.peak_surface_water_mm >= options.flood_closure_depth_mm);
  CHECK(result.minimum_speed_factor < 1.0);
  CHECK(result.minimum_capacity_factor < 1.0);
  CHECK(result.closed_link_steps > 0);
  CHECK(result.links_closed_at_least_once == std::vector<int>{10});
  CHECK(impacted.free_flow_time_profile_hr.back() > link.free_flow_time_hr);
  CHECK(impacted.capacity_profile_veh_per_hr.back() <
        link.capacity_veh_per_hr);
  CHECK_FALSE(impacted.availability_profile.back());
}

TEST_CASE("Typhoon road impact marks affine georeferencing of benchmark topology",
          "[scenario_generation][typhoon][traffic][georeference]") {
  hacdcpf::evpt::TrafficGraph traffic;
  traffic.nodes = {{1, "a", -96.77, 43.61},
                   {2, "b", -96.71, 43.56}};
  hacdcpf::evpt::TrafficLink link;
  link.index = 1;
  link.from_node = 1;
  link.to_node = 2;
  link.length_km = 5.0;
  link.free_flow_time_hr = 0.1;
  link.capacity_veh_per_hr = 1000.0;
  traffic.links.push_back(link);

  const std::vector<hacdcpf::analysis::TyphoonTrackPoint> track = {
      storm_point(0.0), storm_point(1.0)};
  hacdcpf::analysis::TyphoonTrafficImpactOptions options;
  options.num_steps = 2;
  options.time_step_hr = 0.5;
  options.coordinate_mode =
      hacdcpf::analysis::TyphoonTrafficCoordinateMode::Auto;
  const auto result = hacdcpf::analysis::apply_typhoon_traffic_impact(
      traffic, track, options);
  CHECK(result.used_affine_georeferencing);
  CHECK(result.model_scope.find("affinely georeferenced") !=
        std::string::npos);
}

TEST_CASE("CTM reads road profiles by parent simulation step",
          "[ev_power_traffic][ctm][dynamic_road]") {
  hacdcpf::evpt::EVPowerTrafficProblem problem;
  problem.traffic.nodes = {{1, "origin"}, {2, "destination"}};
  hacdcpf::evpt::TrafficLink link;
  link.index = 1;
  link.from_node = 1;
  link.to_node = 2;
  link.length_km = 1.0;
  link.free_flow_time_hr = 0.25;
  link.capacity_veh_per_hr = 100.0;
  link.jam_vehicles = 100.0;
  link.capacity_profile_veh_per_hr = {0.0, 100.0};
  link.availability_profile = {false, true};
  problem.traffic.links.push_back(link);

  hacdcpf::evpt::RouteAlternative route;
  route.index = 1;
  route.origin_node = 1;
  route.destination_node = 2;
  route.link_indices = {1};
  problem.routes.push_back(route);

  hacdcpf::evpt::EVDemand first;
  first.index = 1;
  first.origin_node = 1;
  first.destination_node = 2;
  first.departure_step = 0;
  first.vehicles = 10.0;
  first.candidate_route_indices = {1};
  problem.demands.push_back(first);
  auto second = first;
  second.index = 2;
  second.departure_step = 1;
  problem.demands.push_back(second);

  hacdcpf::evpt::CTMForwardAssignment assignment;
  assignment.ev_route_flow[1][1] = 10.0;
  assignment.ev_route_flow[2][1] = 10.0;
  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = 2;
  ev_options.time_step_hr = 1.0;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.dt_ctm_hr = 0.25;
  ctm_options.route_specific_cells = true;

  const auto result = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
      problem, assignment, ev_options, ctm_options);
  REQUIRE(result.valid);
  REQUIRE(result.simulation.steps_per_sim_step == 4);
  CHECK(result.requested_vehicles == Approx(20.0));
  CHECK(result.admitted_vehicles == Approx(10.0).margin(1e-8));
  const auto& admitted = result.simulation.route_admitted_departures.at(1);
  CHECK(std::accumulate(admitted.begin(), admitted.begin() + 4, 0.0) ==
        Approx(0.0).margin(1e-10));
  CHECK(std::accumulate(admitted.begin() + 4, admitted.end(), 0.0) ==
        Approx(10.0).margin(1e-8));
}

TEST_CASE("Typhoon catalog snapshots survive concurrent mixed-option refreshes",
          "[scenario_generation][typhoon][catalog][concurrency]") {
  namespace analysis = hacdcpf::analysis;
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temp = std::filesystem::temp_directory_path();

  analysis::TyphoonCatalogOptions first_options;
  first_options.samples_per_month = 2;
  first_options.first_month = 7;
  first_options.last_month = 7;
  first_options.horizon_hours = 2;
  first_options.use_sst_resource = false;
  first_options.catalog_path =
      (temp / ("hysim_typhoon_catalog_a_" + std::to_string(nonce) + ".json")).string();

  analysis::TyphoonCatalogOptions second_options = first_options;
  second_options.base_seed += 1000U;
  second_options.first_month = 8;
  second_options.last_month = 8;
  second_options.catalog_path =
      (temp / ("hysim_typhoon_catalog_b_" + std::to_string(nonce) + ".json")).string();

  const auto first = analysis::get_or_build_typhoon_catalog(first_options);
  REQUIRE(first);
  REQUIRE_FALSE(first->samples.empty());
  const auto category = first->samples.front().category;
  const auto selected = analysis::sample_typhoon_catalog(first, category, 17U);
  REQUIRE(selected);
  const std::string selected_id = selected->sample_id;

  auto refresh = std::async(std::launch::async, [&] {
    return analysis::get_or_build_typhoon_catalog(second_options);
  });
  const auto second = refresh.get();
  REQUIRE(second);
  REQUIRE_FALSE(second->samples.empty());
  CHECK(first != second);
  CHECK(selected->sample_id == selected_id);
  CHECK(selected->month == 7);

  std::error_code ignored;
  std::filesystem::remove(first_options.catalog_path, ignored);
  std::filesystem::remove(second_options.catalog_path, ignored);
}

TEST_CASE("Typhoon road impact rejects invalid clamp and hydrology bounds",
          "[scenario_generation][typhoon][traffic][validation]") {
  hacdcpf::evpt::TrafficGraph traffic;
  traffic.nodes = {{1, "a", 113.5, 22.7}, {2, "b", 113.6, 22.7}};
  hacdcpf::evpt::TrafficLink link;
  link.index = 7;
  link.from_node = 1;
  link.to_node = 2;
  link.free_flow_time_hr = 0.1;
  link.capacity_veh_per_hr = 100.0;
  traffic.links.push_back(link);
  const std::vector<hacdcpf::analysis::TyphoonTrackPoint> track = {
      storm_point(0.0), storm_point(1.0)};

  auto options = hacdcpf::analysis::TyphoonTrafficImpactOptions{};
  options.maximum_surface_water_mm = -1.0;
  CHECK_THROWS_AS(hacdcpf::analysis::apply_typhoon_traffic_impact(
                      traffic, track, options),
                  std::invalid_argument);

  options = {};
  options.minimum_open_speed_factor = 1.1;
  CHECK_THROWS_AS(hacdcpf::analysis::apply_typhoon_traffic_impact(
                      traffic, track, options),
                  std::invalid_argument);

  options = {};
  options.minimum_open_capacity_factor =
      std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(hacdcpf::analysis::apply_typhoon_traffic_impact(
                      traffic, track, options),
                  std::invalid_argument);

  options = {};
  options.runoff_multiplier_by_link[7] = -0.5;
  CHECK_THROWS_AS(hacdcpf::analysis::apply_typhoon_traffic_impact(
                      traffic, track, options),
                  std::invalid_argument);
}
