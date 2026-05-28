// ctm_propagation.cpp
// ──────────────────────────────────────────────────────────────────────────
// Daganzo Cell Transmission Model (CTM) propagation and Dynamic User
// Equilibrium (DUE) via Method of Successive Averages (MSA).
//
// Reference:
//   Daganzo, C.F. (1994). "The cell transmission model: A dynamic representation
//   of highway traffic consistent with the hydrodynamic theory."
//   Transportation Research Part B, 28(4), 269–287.
//
// Architecture:
//   - Each directed link a is divided into M_a cells of equal length δ_a.
//   - CFL condition: Δt_ctm ≤ δ_a / v^f_a for all a.
//   - Triangular fundamental diagram: free-flow slope v^f_a, backward wave w_a.
//   - Route-specific (commodity) cell occupancy n^r_{a,m,k} tracks per-route
//     vehicles, enabling correct station-arrival attribution at diverges.
//   - Node model: supply–demand merge/diverge with proportional turning.
//   - MSA DUE: x^{(i+1)} = x^{(i)} + step_i * (x^{aux}_i - x^{(i)}),
//     where x^{aux} = shortest-path (or logit) assignment under current
//     experienced travel times from the forward CTM pass.

#include "evpt_internal.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace hacdcpf::evpt {
namespace {

// ────────────────────────────────────────────────────────────────────────────
// 1.  CTM link initialisation
// ────────────────────────────────────────────────────────────────────────────

// Derive free-flow speed [km/h] from free_flow_time_hr and length_km.
// Falls back to a large sentinel if length_km == 0.
inline double link_free_flow_speed(const TrafficLink& link) {
  if (link.length_km > kTol && link.free_flow_time_hr > kTol) {
    return link.length_km / link.free_flow_time_hr;
  }
  // Fallback: treat the link as a single cell with instantaneous travel.
  return 1.0e6;
}

// Derive backward wave speed from jam_vehicles, max_flow, and free-flow speed.
// Uses triangular FD: k_jam = jam_veh / delta,  q_max from capacity_veh_per_hr,
// w = q_max / (k_jam - q_max/v_f).
inline double link_backward_wave_speed(const TrafficLink& link,
                                       double delta_km,
                                       double v_f_km_hr,
                                       double q_max_veh_hr,
                                       double fallback_w) {
  if (delta_km <= kTol || v_f_km_hr <= kTol) return fallback_w;
  const double k_jam = link.jam_vehicles > kTol
                           ? link.jam_vehicles / delta_km
                           : q_max_veh_hr / v_f_km_hr * 2.0;  // default: k_crit = k_jam/2
  const double k_crit = q_max_veh_hr / v_f_km_hr;
  if (k_jam <= k_crit + kTol) return fallback_w;
  return q_max_veh_hr / (k_jam - k_crit);
}

// Build CTMLinkState for a single link, choosing cell count from options.
CTMLinkState build_link_state(const TrafficLink& link,
                              int n_routes,
                              const CTMOptions& ctm_opts,
                              double dt_ctm) {
  CTMLinkState ls;
  ls.link_index = link.index;

  const double v_f = link_free_flow_speed(link);
  ls.free_flow_speed_km_hr = v_f;

  const double q_max = link.capacity_veh_per_hr > kTol
                           ? link.capacity_veh_per_hr
                           : 1.0e4;

  // Cell count: M_a = max(1, floor(L_a / (v_f * dt_ctm)))
  if (link.length_km > kTol && dt_ctm > kTol) {
    const double ideal_delta = v_f * dt_ctm;
    ls.n_cells = ctm_opts.n_cells_per_link > 0
                     ? ctm_opts.n_cells_per_link
                     : std::max(1, static_cast<int>(
                                       std::floor(link.length_km / ideal_delta)));
    ls.cell_length_km = link.length_km / static_cast<double>(ls.n_cells);
  } else {
    ls.n_cells = 1;
    ls.cell_length_km = std::max(link.length_km, kTol);
  }

  const double delta = ls.cell_length_km;
  const double w = link_backward_wave_speed(
      link, delta, v_f, q_max,
      ctm_opts.backward_wave_speed_fallback_km_hr);
  ls.backward_wave_speed_km_hr = w;

  // Jam occupancy per cell: N^jam_a = k_jam * delta
  // For triangular FD: k_jam = q_max*(v_f+w)/(v_f*w)
  const double k_jam_per_km = q_max * (v_f + w) / (v_f * w + kTol);
  ls.jam_occupancy_per_cell = k_jam_per_km * delta;
  ls.max_flow_veh_per_hr = q_max;

  // Initialise cells with zero occupancy.
  ls.cells.resize(static_cast<std::size_t>(ls.n_cells));
  for (auto& cell : ls.cells) {
    cell.n = 0.0;
    cell.n_route.assign(static_cast<std::size_t>(n_routes), 0.0);
  }
  return ls;
}

// ────────────────────────────────────────────────────────────────────────────
// 2.  Sending / Receiving functions (triangular FD)
// ────────────────────────────────────────────────────────────────────────────

// Sending function: S_{a,m,k} = min(n_{a,m,k} * v_f / delta, q_max) * dt [veh]
inline double sending(double n, double v_f, double delta, double q_max, double dt) {
  if (delta <= kTol || dt <= kTol) return 0.0;
  return dt * std::min(n * v_f / delta, q_max);
}

// Receiving function: R_{a,m,k} = min(q_max, w*(N^jam - n)/delta) * dt [veh]
inline double receiving(double n, double w, double delta,
                        double q_max, double n_jam, double dt) {
  if (delta <= kTol || dt <= kTol) return 0.0;
  return dt * std::min(q_max, w * std::max(0.0, n_jam - n) / delta);
}

// ────────────────────────────────────────────────────────────────────────────
// 3.  Node turning fractions
// ────────────────────────────────────────────────────────────────────────────

// Pre-compute turning fractions from route topology.
// For each node v, incoming link a, outgoing link b, route r:
//   theta[v][a][b][r] = 1 if route r passes through (a -> v -> b), else 0.
// Then normalise so Σ_b theta[v][a][b][r] = 1 for each (v,a,r) with nonzero flow.
using TurningFractionMap =
    std::unordered_map<int,                          // node
      std::unordered_map<int,                        // in_link
        std::unordered_map<int, std::vector<double>>>>;  // out_link -> [n_routes]

TurningFractionMap build_turning_fractions(
    const EVPowerTrafficProblem& problem,
    const std::unordered_map<int, std::size_t>& link_pos,
    int n_routes) {
  TurningFractionMap tfm;

  // Index routes
  std::unordered_map<int, int> route_pos;
  for (int ri = 0; ri < static_cast<int>(problem.routes.size()); ++ri) {
    route_pos[problem.routes[static_cast<std::size_t>(ri)].index] = ri;
  }

  // Link-to-node mapping
  std::unordered_map<int, int> link_to_node;  // link_index -> to_node
  std::unordered_map<int, int> link_from_node;
  for (const auto& lnk : problem.traffic.links) {
    link_to_node[lnk.index] = lnk.to_node;
    link_from_node[lnk.index] = lnk.from_node;
  }

  // Accumulate turning indicators per route
  for (const auto& route : problem.routes) {
    const int rpos = route_pos.count(route.index) ? route_pos.at(route.index) : -1;
    if (rpos < 0 || rpos >= n_routes) continue;
    const auto& links = route.link_indices;
    for (std::size_t li = 0; li + 1 < links.size(); ++li) {
      const int a = links[li];
      const int b = links[li + 1];
      if (!link_pos.count(a) || !link_pos.count(b)) continue;
      const int node = link_to_node.count(a) ? link_to_node.at(a) : -1;
      if (node < 0) continue;
      auto& row = tfm[node][a][b];
      if (static_cast<int>(row.size()) < n_routes) {
        row.assign(static_cast<std::size_t>(n_routes), 0.0);
      }
      row[static_cast<std::size_t>(rpos)] = 1.0;
    }
  }
  return tfm;
}

// ────────────────────────────────────────────────────────────────────────────
// 4.  CTM forward pass (single iteration)
// ────────────────────────────────────────────────────────────────────────────
// Propagates vehicles through the CTM network over T_ctm steps given
// per-departure-step OD route flows x[departure_step][route_index] [vehicles].
//
// Returns: CTMSimulationResult with station arrivals and experienced
//          travel times.

CTMSimulationResult ctm_forward_pass(
    const EVPowerTrafficProblem& problem,
    const std::vector<std::vector<double>>& route_flow,  // [k_dep][route_pos]
    const std::unordered_map<int, std::size_t>& link_pos,
    const std::unordered_map<int, int>& route_pos,
    const CTMOptions& ctm_opts,
    double dt_ctm,
    int T_ctm) {

  const int n_links = static_cast<int>(problem.traffic.links.size());
  const int n_routes = static_cast<int>(problem.routes.size());
  const bool route_specific = ctm_opts.route_specific_cells;

  // ── Build link states ─────────────────────────────────────────────────
  std::vector<CTMLinkState> link_states;
  link_states.reserve(static_cast<std::size_t>(n_links));
  for (const auto& lnk : problem.traffic.links) {
    link_states.push_back(build_link_state(lnk, route_specific ? n_routes : 0,
                                           ctm_opts, dt_ctm));
  }

  // ── Build turning fractions ───────────────────────────────────────────
  TurningFractionMap tfm;
  if (route_specific) {
    tfm = build_turning_fractions(problem, link_pos, n_routes);
  }

  // Link-to-node maps
  std::unordered_map<int, int> link_to_node, link_from_node;
  for (const auto& lnk : problem.traffic.links) {
    link_to_node[lnk.index] = lnk.to_node;
    link_from_node[lnk.index] = lnk.from_node;
  }

  // Route entry link (first link of each route)
  std::vector<int> route_entry_link(static_cast<std::size_t>(n_routes), -1);
  for (int ri = 0; ri < n_routes; ++ri) {
    const auto& links = problem.routes[static_cast<std::size_t>(ri)].link_indices;
    if (!links.empty()) {
      route_entry_link[static_cast<std::size_t>(ri)] = links.front();
    }
  }

  // ── Station-arrival attribution ───────────────────────────────────────
  // Identify for each (route, link) which station (if any) is located
  // at the end of that link (i.e., the link whose exit = station approach).
  // We use ChargingStop.station_id + route.link_indices ordering.
  // station_link[route_pos][stop_pos] = link_index of the approach link.
  std::unordered_map<int, std::vector<std::pair<int, int>>> route_stop_link;
  // route_stop_link[r] = vector of (link_index, station_id)
  for (int ri = 0; ri < n_routes; ++ri) {
    const auto& route = problem.routes[static_cast<std::size_t>(ri)];
    const auto& links = route.link_indices;
    const auto& stops = route.charging_stops;
    // Simple heuristic: if there is one stop per route, attribute it to the
    // last link exit on that route.  If multiple stops, distribute evenly.
    if (!stops.empty() && !links.empty()) {
      // Map stop_i -> link_index at index (n_links-1)*(i+1)/n_stops
      for (int si = 0; si < static_cast<int>(stops.size()); ++si) {
        const int ratio = static_cast<int>(links.size()) *
                          (si + 1) / static_cast<int>(stops.size());
        const int link_idx = links[static_cast<std::size_t>(
            std::min(ratio, static_cast<int>(links.size()) - 1))];
        route_stop_link[ri].emplace_back(link_idx, stops[static_cast<std::size_t>(si)].station_id);
      }
    }
  }

  // ── Initialise result ─────────────────────────────────────────────────
  CTMSimulationResult res;
  for (const auto& lnk : problem.traffic.links) {
    (void)lnk;
    res.step_link_results.emplace_back(
        static_cast<std::size_t>(n_links), CTMLinkStepResult{});
  }
  res.step_link_results.assign(
      static_cast<std::size_t>(T_ctm),
      std::vector<CTMLinkStepResult>(static_cast<std::size_t>(n_links)));
  for (int k = 0; k < T_ctm; ++k) {
    for (std::size_t li = 0; li < static_cast<std::size_t>(n_links); ++li) {
      res.step_link_results[static_cast<std::size_t>(k)][li].link_index =
          problem.traffic.links[li].index;
    }
  }

  // Station arrivals
  for (const auto& stn : problem.system.ac.charging_stations) {
    res.station_arrivals[stn.index] =
        std::vector<double>(static_cast<std::size_t>(T_ctm), 0.0);
  }

  // Experienced travel time accumulators: sum_vehicles * time for LSDT average
  // [n_routes][T_ctm departure steps]
  std::vector<std::vector<double>> tt_sum_veh_hr(
      static_cast<std::size_t>(n_routes),
      std::vector<double>(static_cast<std::size_t>(T_ctm), 0.0));
  std::vector<std::vector<double>> tt_veh(
      static_cast<std::size_t>(n_routes),
      std::vector<double>(static_cast<std::size_t>(T_ctm), 0.0));

  // ── Main time loop ────────────────────────────────────────────────────
  for (int k = 0; k < T_ctm; ++k) {
    // 4a. Inject entering vehicles at entry cells from OD flows
    if (k < static_cast<int>(route_flow.size())) {
      for (int ri = 0; ri < n_routes; ++ri) {
        const double x_rk =
            static_cast<std::size_t>(ri) < route_flow[static_cast<std::size_t>(k)].size()
                ? route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)]
                : 0.0;
        if (x_rk <= kTol) continue;
        const int entry_link = route_entry_link[static_cast<std::size_t>(ri)];
        if (!link_pos.count(entry_link)) continue;
        const std::size_t li = link_pos.at(entry_link);
        CTMLinkState& ls = link_states[li];
        const double recv = receiving(ls.cells[0].n,
                                      ls.backward_wave_speed_km_hr,
                                      ls.cell_length_km,
                                      ls.max_flow_veh_per_hr,
                                      ls.jam_occupancy_per_cell,
                                      dt_ctm);
        const double enter = std::min(x_rk, recv);
        ls.cells[0].n += enter;
        if (route_specific && !ls.cells[0].n_route.empty()) {
          ls.cells[0].n_route[static_cast<std::size_t>(ri)] += enter;
        }
        res.step_link_results[static_cast<std::size_t>(k)][li].total_inflow_veh += enter;
        // Contribute to experienced travel time (free-flow cost of entry, updated later)
        tt_veh[static_cast<std::size_t>(ri)][static_cast<std::size_t>(k)] += enter;
      }
    }

    // 4b. Intra-link cell propagation (cell 0 -> M-1 for each link)
    for (std::size_t li = 0; li < static_cast<std::size_t>(n_links); ++li) {
      CTMLinkState& ls = link_states[li];
      const int M = ls.n_cells;
      const double vf = ls.free_flow_speed_km_hr;
      const double w  = ls.backward_wave_speed_km_hr;
      const double delta = ls.cell_length_km;
      const double q_max = ls.max_flow_veh_per_hr;
      const double n_jam = ls.jam_occupancy_per_cell;

      // Compute inter-cell flows y_{a,m->m+1} for m = 0..M-2
      // y = min(S_{a,m,k}, R_{a,m+1,k})
      std::vector<double> y(static_cast<std::size_t>(M - 1), 0.0);
      for (int m = 0; m < M - 1; ++m) {
        const double S = sending(ls.cells[static_cast<std::size_t>(m)].n,
                                  vf, delta, q_max, dt_ctm);
        const double R = receiving(ls.cells[static_cast<std::size_t>(m + 1)].n,
                                    w, delta, q_max, n_jam, dt_ctm);
        y[static_cast<std::size_t>(m)] = std::min(S, R);
      }

      // Update cell occupancies: n_{a,m,k+1} = n_{a,m,k} + inflow - outflow
      for (int m = 0; m < M; ++m) {
        const double in_flow  = (m == 0)     ? 0.0 : y[static_cast<std::size_t>(m - 1)];
        const double out_flow = (m == M - 1) ? 0.0 : y[static_cast<std::size_t>(m)];
        const double delta_n = in_flow - out_flow;
        ls.cells[static_cast<std::size_t>(m)].n =
            std::max(0.0, ls.cells[static_cast<std::size_t>(m)].n + delta_n);

        // Propagate per-route shares proportionally
        if (route_specific) {
          auto& nr = ls.cells[static_cast<std::size_t>(m)].n_route;
          if (!nr.empty()) {
            const double n_tot = ls.cells[static_cast<std::size_t>(m)].n;
            // Scale per-route to sum to aggregate (renormalise)
            double sum_r = 0.0;
            for (double v : nr) sum_r += v;
            if (sum_r > kTol) {
              const double scale = n_tot / sum_r;
              for (double& v : nr) v = std::max(0.0, v * scale);
            } else {
              std::fill(nr.begin(), nr.end(), 0.0);
            }
          }
        }
      }
    }

    // 4c. Inter-link flows at nodes (diverge / merge)
    // For each node: sum sending from all incoming links, distribute to
    // outgoing links via turning fractions, subject to receiving capacity.
    for (const auto& node : problem.traffic.nodes) {
      const int nid = node.index;

      // Collect incoming and outgoing links for this node
      std::vector<std::size_t> in_links, out_links;
      for (std::size_t li = 0; li < static_cast<std::size_t>(n_links); ++li) {
        const auto& lnk = problem.traffic.links[li];
        if (lnk.to_node == nid)   in_links.push_back(li);
        if (lnk.from_node == nid) out_links.push_back(li);
      }
      if (in_links.empty() || out_links.empty()) continue;

      // Compute exit flow from each incoming link's last cell
      for (std::size_t a_li : in_links) {
        CTMLinkState& a_ls = link_states[a_li];
        const int M_a = a_ls.n_cells;
        CTMLinkCell& last_cell = a_ls.cells[static_cast<std::size_t>(M_a - 1)];

        const double S_a = sending(last_cell.n,
                                    a_ls.free_flow_speed_km_hr,
                                    a_ls.cell_length_km,
                                    a_ls.max_flow_veh_per_hr,
                                    dt_ctm);

        if (S_a <= kTol) continue;

        // Distribute to downstream links via turning fractions
        const int a_idx = problem.traffic.links[a_li].index;

        // Compute turning fraction for this incoming link -> each outgoing link
        // across all routes.  If route_specific, use per-route fractions.
        std::vector<double> demand_per_out(out_links.size(), 0.0);

        if (route_specific && tfm.count(nid) && tfm.at(nid).count(a_idx)) {
          const auto& a_turns = tfm.at(nid).at(a_idx);
          for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
            const int b_idx = problem.traffic.links[out_links[oi]].index;
            if (!a_turns.count(b_idx)) continue;
            const auto& theta = a_turns.at(b_idx);
            double demand = 0.0;
            for (int ri = 0; ri < n_routes; ++ri) {
              const double n_ri =
                  (ri < static_cast<int>(last_cell.n_route.size()))
                      ? last_cell.n_route[static_cast<std::size_t>(ri)]
                      : 0.0;
              const double th =
                  (ri < static_cast<int>(theta.size())) ? theta[static_cast<std::size_t>(ri)] : 0.0;
              demand += n_ri * th;
            }
            demand_per_out[oi] = demand;
          }
        } else {
          // Uniform split (fallback when turning fractions not available)
          const double share = S_a / static_cast<double>(out_links.size());
          std::fill(demand_per_out.begin(), demand_per_out.end(), share);
        }

        const double total_demand = std::accumulate(
            demand_per_out.begin(), demand_per_out.end(), 0.0);
        if (total_demand <= kTol) continue;

        // Check receiving capacity at each downstream link's first cell
        std::vector<double> recv_out(out_links.size(), 0.0);
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          CTMLinkState& b_ls = link_states[out_links[oi]];
          recv_out[oi] = receiving(b_ls.cells[0].n,
                                    b_ls.backward_wave_speed_km_hr,
                                    b_ls.cell_length_km,
                                    b_ls.max_flow_veh_per_hr,
                                    b_ls.jam_occupancy_per_cell,
                                    dt_ctm);
        }

        // Proportional flow: flow_to_b = min(demand_to_b, recv_b)
        // Restricted total: scale down uniformly if overall supply exceeds total recv.
        double restricted_total = 0.0;
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          restricted_total +=
              std::min(demand_per_out[oi],
                       recv_out[oi] * (demand_per_out[oi] / (total_demand + kTol)));
        }
        const double scale_factor =
            total_demand > kTol
                ? std::min(1.0, restricted_total / total_demand)
                : 0.0;

        // Total actual exit from link a's last cell
        const double actual_exit = S_a * scale_factor;

        // Remove from last cell
        last_cell.n = std::max(0.0, last_cell.n - actual_exit);

        // Add to downstream first cells proportionally
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          const double frac = total_demand > kTol
                                  ? demand_per_out[oi] / total_demand
                                  : 1.0 / static_cast<double>(out_links.size());
          const double add = actual_exit * frac;
          CTMLinkState& b_ls = link_states[out_links[oi]];
          b_ls.cells[0].n += add;

          // Attribute to routes
          if (route_specific && !b_ls.cells[0].n_route.empty()) {
            if (!last_cell.n_route.empty()) {
              double sum_r = std::accumulate(
                  last_cell.n_route.begin(), last_cell.n_route.end(), 0.0);
              for (int ri = 0; ri < n_routes; ++ri) {
                const double frac_r =
                    sum_r > kTol
                        ? last_cell.n_route[static_cast<std::size_t>(ri)] / sum_r
                        : (n_routes > 0 ? 1.0 / n_routes : 0.0);
                b_ls.cells[0].n_route[static_cast<std::size_t>(ri)] += add * frac_r;
              }
            }
          }

          // Record inter-link flow
          const std::size_t b_li = out_links[oi];
          res.step_link_results[static_cast<std::size_t>(k)][b_li].total_inflow_veh += add;
        }
      }
    }

    // 4d. Station arrival collection: last-cell exit flows attributed to stations
    for (int ri = 0; ri < n_routes; ++ri) {
      if (!route_stop_link.count(ri)) continue;
      for (const auto& [approach_link, station_id] : route_stop_link.at(ri)) {
        if (!link_pos.count(approach_link)) continue;
        const std::size_t li = link_pos.at(approach_link);
        const CTMLinkState& ls = link_states[li];
        if (!res.station_arrivals.count(station_id)) continue;
        // Estimate arrivals as sending function from last cell attributed to route r
        const int M = ls.n_cells;
        const double n_last = route_specific && !ls.cells[static_cast<std::size_t>(M - 1)].n_route.empty()
                                  ? ls.cells[static_cast<std::size_t>(M - 1)].n_route[static_cast<std::size_t>(ri)]
                                  : ls.cells[static_cast<std::size_t>(M - 1)].n;
        const double arr = sending(n_last,
                                    ls.free_flow_speed_km_hr,
                                    ls.cell_length_km,
                                    ls.max_flow_veh_per_hr,
                                    dt_ctm);
        if (k < static_cast<int>(res.station_arrivals.at(station_id).size())) {
          res.station_arrivals.at(station_id)[static_cast<std::size_t>(k)] += arr;
        }
      }
    }

    // 4e. Collect per-link step summaries
    for (std::size_t li = 0; li < static_cast<std::size_t>(n_links); ++li) {
      const CTMLinkState& ls = link_states[li];
      CTMLinkStepResult& lr = res.step_link_results[static_cast<std::size_t>(k)][li];
      double tot_occ = 0.0;
      for (const auto& cell : ls.cells) tot_occ += cell.n;
      lr.total_occupancy_veh = tot_occ;
      // Experienced travel time: Little's law  T = N / q_out
      const double last_exit = sending(ls.cells[static_cast<std::size_t>(ls.n_cells - 1)].n,
                                        ls.free_flow_speed_km_hr,
                                        ls.cell_length_km,
                                        ls.max_flow_veh_per_hr, dt_ctm);
      lr.total_outflow_veh = last_exit;
      lr.mean_travel_time_hr =
          (last_exit > kTol)
              ? (tot_occ * dt_ctm / last_exit)
              : (ls.free_flow_speed_km_hr > kTol
                     ? ls.cell_length_km * ls.n_cells / ls.free_flow_speed_km_hr
                     : 0.0);
    }
  }

  // ── Experienced route travel times (sum over links on route) ──────────
  for (int ri = 0; ri < n_routes; ++ri) {
    const auto& route = problem.routes[static_cast<std::size_t>(ri)];
    std::vector<double> tt_dep(static_cast<std::size_t>(T_ctm), 0.0);
    for (int k_dep = 0; k_dep < T_ctm; ++k_dep) {
      double tt = 0.0;
      for (int link_idx : route.link_indices) {
        if (!link_pos.count(link_idx)) continue;
        const std::size_t li = link_pos.at(link_idx);
        const int k_use = std::min(k_dep, T_ctm - 1);
        tt += res.step_link_results[static_cast<std::size_t>(k_use)][li].mean_travel_time_hr;
      }
      tt_dep[static_cast<std::size_t>(k_dep)] = tt;
    }
    res.route_travel_time[route.index] = tt_dep;
  }

  // Optionally save cell history
  if (ctm_opts.record_cell_history) {
    res.cell_history.push_back(link_states);
  }

  return res;
}

// ────────────────────────────────────────────────────────────────────────────
// 5.  Wardrop relative gap
// ────────────────────────────────────────────────────────────────────────────
// Computes the relative gap used as the DUE stopping criterion:
//   gap = (TSTT - MSTT) / MSTT
// where TSTT = total system travel time under current flows,
//       MSTT = minimum system travel time (lower bound, assign all to shortest path).

double wardrop_relative_gap(
    const EVPowerTrafficProblem& problem,
    const std::vector<std::vector<double>>& route_flow,   // [k_dep_ctm][route_pos]
    const CTMSimulationResult& ctm_res,
    const std::unordered_map<int, int>& route_pos_map,
    int T_ctm,
    int R) {

  double tstt = 0.0;  // Total System Travel Time under current flows
  double mstt = 0.0;  // Min route cost for each demand unit

  for (const auto& demand : problem.demands) {
    // Map simulation departure step -> CTM departure step
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;

    // Find minimum experienced travel time across all candidate routes
    double min_tt = std::numeric_limits<double>::infinity();
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!ctm_res.route_travel_time.count(route.index)) continue;
      const auto& tt_vec = ctm_res.route_travel_time.at(route.index);
      if (k_dep_ctm < static_cast<int>(tt_vec.size())) {
        min_tt = std::min(min_tt, tt_vec[static_cast<std::size_t>(k_dep_ctm)]);
      }
    }
    if (!std::isfinite(min_tt)) continue;
    mstt += demand.vehicles * min_tt;

    // TSTT: sum over all routes of x_{d,r} * T^CTM_{r,k_dep_ctm}
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!route_pos_map.count(route.index)) continue;
      const int rpos = route_pos_map.at(route.index);
      if (k_dep_ctm >= static_cast<int>(route_flow.size())) continue;
      if (rpos >= static_cast<int>(route_flow[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
      const double x = route_flow[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)];
      if (!ctm_res.route_travel_time.count(route.index)) continue;
      const auto& tt_vec = ctm_res.route_travel_time.at(route.index);
      if (k_dep_ctm < static_cast<int>(tt_vec.size())) {
        tstt += x * tt_vec[static_cast<std::size_t>(k_dep_ctm)];
      }
    }
  }

  if (mstt < kTol) return 0.0;
  return std::max(0.0, (tstt - mstt) / mstt);
}

// ────────────────────────────────────────────────────────────────────────────
// 6.  Auxiliary shortest-path assignment given experienced travel times
// ────────────────────────────────────────────────────────────────────────────
// Assigns all demand to the route with minimum experienced travel time
// (all-or-nothing shortest-path, or logit if theta > 0).
// Returns route_flow[k_dep][route_pos].

std::vector<std::vector<double>> auxiliary_assignment(
    const EVPowerTrafficProblem& problem,
    const CTMSimulationResult& ctm_res,
    const std::unordered_map<int, int>& route_pos_map,
    const std::unordered_map<int, std::size_t>& link_pos,
    const EVPowerTrafficOptions& ev_opts,
    const DUEOptions& due_opts,
    int T_ctm,
    int R) {

  const int n_routes = static_cast<int>(problem.routes.size());
  std::vector<std::vector<double>> x_aux(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  const double vot = std::max(ev_opts.value_of_time_per_hr, kTol);

  for (const auto& demand : problem.demands) {
    // Map simulation departure step -> CTM departure step
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
    // Simulation step for station price lookup
    const int k_sim = demand.departure_step;

    // Collect candidate routes and their experienced travel times
    std::vector<const RouteAlternative*> cands;
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        cands.push_back(&route);
      }
    }
    if (cands.empty()) continue;

    // Helper: compute generalized cost for a route.
    // gen_cost = tt + (ω_s / VOT) * Σ_{s∈stops} π_{s,k} * E_{r,s}
    // Units: hours (price term divided by VOT converts $/veh to hours-equivalent).
    auto route_gen_cost = [&](const RouteAlternative* route) -> double {
      double tt = std::numeric_limits<double>::infinity();
      if (ctm_res.route_travel_time.count(route->index)) {
        const auto& v = ctm_res.route_travel_time.at(route->index);
        if (k_dep_ctm < static_cast<int>(v.size()))
          tt = v[static_cast<std::size_t>(k_dep_ctm)];
      }
      if (!std::isfinite(tt)) return tt;
      // Add station price term
      for (const auto& stop : route->charging_stops) {
        const double pi = station_price(problem, ev_opts, stop.station_id, k_sim);
        tt += ev_opts.station_energy_cost_weight * pi *
              stop.requested_energy_kwh_per_vehicle / vot;
      }
      return tt;
    };

    if (due_opts.logit_theta > kTol) {
      // Logit (stochastic DUE)
      double weight_sum = 0.0;
      std::vector<double> weights(cands.size(), 0.0);
      for (std::size_t ci = 0; ci < cands.size(); ++ci) {
        // SOC feasibility filter (eq. evpt-route-soc-bounds)
        if (!route_soc_feasible(*cands[ci], demand, problem, link_pos)) continue;
        const double gc = route_gen_cost(cands[ci]);
        weights[ci] = std::isfinite(gc) ? std::exp(-due_opts.logit_theta * gc) : 0.0;
        weight_sum += weights[ci];
      }
      for (std::size_t ci = 0; ci < cands.size(); ++ci) {
        if (weight_sum <= kTol) continue;
        const int rpos = route_pos_map.count(cands[ci]->index)
                             ? route_pos_map.at(cands[ci]->index) : -1;
        if (rpos < 0) continue;
        x_aux[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)] +=
            demand.vehicles * weights[ci] / weight_sum;
      }
    } else {
      // All-or-nothing shortest path (minimum generalized cost)
      const RouteAlternative* best = nullptr;
      double best_gc = std::numeric_limits<double>::infinity();
      for (const auto* route : cands) {
        // SOC feasibility filter (eq. evpt-route-soc-bounds)
        if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;
        const double gc = route_gen_cost(route);
        if (gc < best_gc) { best_gc = gc; best = route; }
      }
      if (best) {
        const int rpos = route_pos_map.count(best->index)
                             ? route_pos_map.at(best->index) : -1;
        if (rpos >= 0) {
          x_aux[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)] +=
              demand.vehicles;
        }
      }
    }
  }
  return x_aux;
}

// ────────────────────────────────────────────────────────────────────────────
// 7.  Select CTM time step (auto CFL check)
// ────────────────────────────────────────────────────────────────────────────

double choose_ctm_dt(const EVPowerTrafficProblem& problem,
                     const CTMOptions& ctm_opts,
                     double dt_sim) {
  if (ctm_opts.dt_ctm_hr > kTol) return ctm_opts.dt_ctm_hr;
  // Auto: CFL condition  Δt_ctm ≤ min_a(δ_a / v^f_a)
  double min_cfl = dt_sim;  // never coarser than sim step
  for (const auto& lnk : problem.traffic.links) {
    const double vf = link_free_flow_speed(lnk);
    if (vf < kTol) continue;
    const double delta =
        lnk.length_km > kTol
            ? lnk.length_km / static_cast<double>(ctm_opts.n_cells_per_link > 0
                                                       ? ctm_opts.n_cells_per_link
                                                       : std::max(1, static_cast<int>(
                                                                         std::floor(lnk.length_km / (vf * dt_sim)))))
            : vf * dt_sim;
    const double cfl = delta / vf;
    min_cfl = std::min(min_cfl, cfl);
  }
  // Use fraction of min CFL for safety margin (0.9)
  return std::max(kTol, 0.9 * min_cfl);
}

}  // anonymous namespace (private CTM helpers)

// ────────────────────────────────────────────────────────────────────────────
// 9.  Session synthesis from converged DUE route flows
// ────────────────────────────────────────────────────────────────────────────
// Uses CTM-derived travel times (not free-flow) to compute station arrival
// steps.  Each (demand, route, stop) triple with DUE flow > kTol becomes one
// EVChargingSession, mirroring Formulation A's session table but with
// congestion-adjusted arrival timing.

namespace {

std::vector<EVChargingSession> synthesize_ctm_sessions(
    const EVPowerTrafficProblem& problem,
    const std::unordered_map<int, std::unordered_map<int, double>>& flow_map,
    const CTMSimulationResult& final_ctm,
    const EVPowerTrafficOptions& ev_options,
    double dt_sim,
    int T_sim,
    int R) {
  std::vector<EVChargingSession> sessions;
  int next_idx = 1;

  for (const auto& demand : problem.demands) {
    const int k_dep_ctm = demand.departure_step * R;

    for (const auto& route : problem.routes) {
      if (route.charging_stops.empty()) continue;
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;

      // DUE-converged flow for this (demand, route) pair
      double vehicles = 0.0;
      if (flow_map.count(demand.index)) {
        const auto& rm = flow_map.at(demand.index);
        if (rm.count(route.index)) vehicles = rm.at(route.index);
      }
      if (vehicles <= kTol) continue;

      // CTM-derived experienced travel time → arrival step
      double route_tt_hr = 0.0;
      if (final_ctm.route_travel_time.count(route.index)) {
        const auto& tv = final_ctm.route_travel_time.at(route.index);
        const int idx = std::min(k_dep_ctm, static_cast<int>(tv.size()) - 1);
        if (idx >= 0) route_tt_hr = tv[static_cast<std::size_t>(idx)];
      }
      if (!std::isfinite(route_tt_hr) || route_tt_hr < 0.0) route_tt_hr = 0.0;

      const int steps_off =
          dt_sim > kTol
              ? std::max(0, static_cast<int>(std::ceil(route_tt_hr / dt_sim)))
              : 0;
      const int k_arr_sim = demand.departure_step + steps_off;
      if (k_arr_sim >= T_sim) continue;

      for (const auto& stop : route.charging_stops) {
        EVChargingSession sess;
        sess.index           = next_idx++;
        sess.station_id      = stop.station_id;
        sess.arrival_step    = k_arr_sim;
        sess.departure_step  =
            std::min(T_sim, k_arr_sim + std::max(1, stop.dwell_steps));
        sess.vehicle_count   = vehicles;
        const double e_req   =
            std::max(0.0, stop.requested_energy_kwh_per_vehicle * vehicles);
        sess.energy_initial_kwh = 0.0;
        sess.energy_target_kwh  = e_req;
        sess.energy_min_kwh     = 0.0;
        sess.energy_max_kwh     = e_req;
        sess.max_charge_kw      =
            stop.max_charge_kw_per_vehicle > kTol
                ? stop.max_charge_kw_per_vehicle * vehicles
                : ev_options.default_route_stop_power_kw_per_vehicle * vehicles;
        sess.max_discharge_kw   =
            stop.v2g_capable
                ? stop.max_discharge_kw_per_vehicle * vehicles
                : 0.0;
        const double eta      = ev_options.default_charging_efficiency;
        sess.eta_charge       = eta;
        sess.eta_discharge    = eta;
        sess.v2g_capable      = stop.v2g_capable;
        sess.source_demand_index = demand.index;
        sess.source_route_index  = route.index;
        sessions.push_back(sess);
      }
    }
  }
  return sessions;
}

// ────────────────────────────────────────────────────────────────────────────
// 10.  Smart-charging dispatch for CTM-synthesized sessions
// ────────────────────────────────────────────────────────────────────────────
// Greedy heuristic for the per-session LP (Eqs. evpt-smart-obj through
// evpt-smart-slack): at each step in the dwell window, charge at the maximum
// feasible power until E^req is met; discharge only above the high-price
// threshold.

void dispatch_ctm_sessions(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options,
    const std::vector<EVChargingSession>& sessions,
    int T_sim,
    double dt_sim,
    CTMDUEResult& result) {
  if (sessions.empty()) return;

  const auto charge_cap  = station_capacity_kw(problem.system, ev_options);
  const auto dis_cap_map = station_discharge_capacity_kw(problem.system);
  const auto stn_ids     = active_station_ids(problem);

  for (int sid : stn_ids) {
    if (!result.station_ev_load_kw.count(sid)) {
      result.station_ev_load_kw[sid] =
          std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
    }
  }

  std::unordered_map<int, std::vector<double>> ch_used;
  std::unordered_map<int, std::vector<double>> dis_used;
  for (int sid : stn_ids) {
    ch_used[sid]  = std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
    dis_used[sid] = std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
  }

  result.session_results.reserve(sessions.size());

  for (const auto& sess : sessions) {
    ChargingSessionResult sr;
    sr.session_index = sess.index;
    sr.station_id    = sess.station_id;
    sr.p_charge_kw.assign(static_cast<std::size_t>(T_sim), 0.0);
    sr.p_discharge_kw.assign(static_cast<std::size_t>(T_sim), 0.0);
    sr.energy_kwh.assign(static_cast<std::size_t>(T_sim + 1), 0.0);

    const int sid = sess.station_id;
    double energy = sess.energy_initial_kwh;

    for (int k = 0; k < T_sim; ++k) {
      sr.energy_kwh[static_cast<std::size_t>(k)] = energy;
      if (k < sess.arrival_step || k >= sess.departure_step) continue;

      const double price =
          station_price(problem, ev_options, sid, k);
      const double cap_ch =
          charge_cap.count(sid) ? charge_cap.at(sid)
                                : ev_options.default_station_power_kw;
      const double ch_rem =
          std::max(0.0, cap_ch - ch_used[sid][static_cast<std::size_t>(k)]);
      const double needed = std::max(0.0, sess.energy_target_kwh - energy);

      double p_ch = 0.0;
      if (needed > kTol) {
        p_ch = std::min({sess.max_charge_kw, ch_rem,
                         needed / (std::max(sess.eta_charge, kTol) *
                                   std::max(dt_sim, kTol))});
        p_ch = std::max(0.0, p_ch);
      }

      double p_dis = 0.0;
      const bool v2g_ok =
          ev_options.allow_v2g && sess.v2g_capable && sess.max_discharge_kw > kTol;
      if (v2g_ok && price >= ev_options.high_price_threshold_per_kwh) {
        const double cap_dis =
            dis_cap_map.count(sid) ? dis_cap_map.at(sid)
                                   : std::numeric_limits<double>::infinity();
        const double dis_rem =
            std::max(0.0, cap_dis - dis_used[sid][static_cast<std::size_t>(k)]);
        const double surplus =
            std::max(0.0, energy - std::max(sess.energy_min_kwh,
                                            sess.energy_target_kwh));
        p_dis = std::min({sess.max_discharge_kw, dis_rem,
                          surplus * std::max(sess.eta_discharge, kTol) /
                              std::max(dt_sim, kTol)});
        p_dis = std::max(0.0, p_dis);
      }
      if (price <= ev_options.low_price_threshold_per_kwh && needed > kTol) {
        p_dis = 0.0;
      }

      energy += dt_sim * (sess.eta_charge * p_ch -
                           p_dis / std::max(sess.eta_discharge, kTol));
      energy = std::clamp(energy, sess.energy_min_kwh, sess.energy_max_kwh);

      sr.p_charge_kw[static_cast<std::size_t>(k)]    = p_ch;
      sr.p_discharge_kw[static_cast<std::size_t>(k)] = p_dis;
      ch_used[sid][static_cast<std::size_t>(k)]  += p_ch;
      dis_used[sid][static_cast<std::size_t>(k)] += p_dis;

      if (result.station_ev_load_kw.count(sid)) {
        result.station_ev_load_kw[sid][static_cast<std::size_t>(k)] +=
            p_ch - p_dis;
      }
      result.total_delivered_energy_kwh +=
          dt_sim * sess.eta_charge * p_ch;
    }

    sr.energy_kwh[static_cast<std::size_t>(T_sim)] = energy;
    sr.unserved_energy_kwh = std::max(0.0, sess.energy_target_kwh - energy);
    result.total_unserved_energy_kwh  += sr.unserved_energy_kwh;
    result.total_requested_energy_kwh += std::max(0.0, sess.energy_target_kwh);
    result.session_results.push_back(std::move(sr));
  }
}

// ────────────────────────────────────────────────────────────────────────────
// 11.  Station queue model for CTM-derived sessions
// ────────────────────────────────────────────────────────────────────────────
// Implements Eqs. evpt-queue-serve, evpt-queue-update, evpt-plug-occ, and
// evpt-wait using the synthesized sessions as the arrival schedule.

void compute_ctm_station_queue(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options,
    const std::vector<EVChargingSession>& sessions,
    int T_sim,
    CTMDUEResult& result) {
  const auto charge_cap = station_capacity_kw(problem.system, ev_options);
  const auto stn_ids    = active_station_ids(problem);

  for (int sid : stn_ids) {
    result.station_queue_veh[sid] =
        std::vector<double>(static_cast<std::size_t>(T_sim + 1), 0.0);
    result.station_plug_occupancy_veh[sid] =
        std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
    result.station_waiting_time_hr[sid] =
        std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
  }

  // λ_{s,k}: arrivals per station per sim step
  std::unordered_map<int, std::vector<double>> lam;
  for (int sid : stn_ids)
    lam[sid] = std::vector<double>(static_cast<std::size_t>(T_sim), 0.0);
  for (const auto& sess : sessions) {
    auto it = lam.find(sess.station_id);
    if (it == lam.end()) continue;
    const int k = sess.arrival_step;
    if (k >= 0 && k < T_sim)
      it->second[static_cast<std::size_t>(k)] +=
          std::max(0.0, sess.vehicle_count);
  }

  // m_{s,k}: plug occupancy from session dwell windows
  for (const auto& sess : sessions) {
    auto m_it = result.station_plug_occupancy_veh.find(sess.station_id);
    if (m_it == result.station_plug_occupancy_veh.end()) continue;
    for (int k = std::max(0, sess.arrival_step);
         k < std::min(sess.departure_step, T_sim); ++k) {
      m_it->second[static_cast<std::size_t>(k)] +=
          std::max(0.0, sess.vehicle_count);
    }
  }

  // Per-station average dwell Δ_s [steps] (eq. evpt-wait: μ_s = N^plug_s / Δ_s)
  std::unordered_map<int, double> station_avg_dwell;
  std::unordered_map<int, int>    station_dwell_count;
  for (int sid : stn_ids) {
    station_avg_dwell[sid]   = 1.0;  // default Δ_s = 1
    station_dwell_count[sid] = 0;
  }
  for (const auto& sess : sessions) {
    const int dwell = sess.departure_step - sess.arrival_step;
    if (dwell > 0 && station_avg_dwell.count(sess.station_id)) {
      station_avg_dwell[sess.station_id]   += static_cast<double>(dwell);
      station_dwell_count[sess.station_id]++;
    }
  }
  for (int sid : stn_ids) {
    if (station_dwell_count[sid] > 0) {
      station_avg_dwell[sid] /= static_cast<double>(station_dwell_count[sid]);
    }
  }

  // Queue dynamics (Eqs. evpt-queue-serve, evpt-queue-update) and W_{s,k}
  for (int k = 0; k < T_sim; ++k) {
    for (int sid : stn_ids) {
      const double lambda =
          lam[sid][static_cast<std::size_t>(k)];
      const double q_sk =
          result.station_queue_veh[sid][static_cast<std::size_t>(k)];
      const double m_sk =
          result.station_plug_occupancy_veh[sid][static_cast<std::size_t>(k)];
      const double n_plug = n_plugs_for_station(problem.system, sid);

      // u_{s,k} = min(q_{s,k} + λ_{s,k}, max(0, N^plug_s − m_{s,k}))
      const double avail = std::isinf(n_plug)
                               ? q_sk + lambda
                               : std::max(0.0, n_plug - m_sk);
      const double u_sk  = std::min(q_sk + lambda, avail);

      result.station_queue_veh[sid][static_cast<std::size_t>(k + 1)] =
          std::max(0.0, q_sk + lambda - u_sk);

      // Service throughput μ_s [veh/step] = N^plug_s / Δ_s ≈ P^max_s / (p_veh · Δ_s)
      // (eq. evpt-wait)
      const double cap_kw =
          charge_cap.count(sid) ? charge_cap.at(sid)
                                : ev_options.default_station_power_kw;
      const double pv      =
          std::max(kTol, ev_options.default_route_stop_power_kw_per_vehicle);
      const double delta_s = std::max(1.0, station_avg_dwell[sid]);
      const double mu_s    = cap_kw / (pv * delta_s);

      // W_{s,k} ≈ (q_{s,k} / μ_s) · Δt  [hours]  (eq. evpt-wait)
      result.station_waiting_time_hr[sid][static_cast<std::size_t>(k)] =
          q_sk / std::max(mu_s, 1e-9) * ev_options.time_step_hr;
    }
  }
}

}  // anonymous namespace (post-DUE synthesis helpers)

CTMDUEResult simulate_ev_power_traffic_ctm_due(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options,
    const CTMOptions& ctm_opts,
    const DUEOptions& due_opts) {

  CTMDUEResult result;

  const double dt_sim = ev_options.time_step_hr;
  const int T_sim     = ev_options.num_steps;
  if (T_sim <= 0 || dt_sim <= kTol) {
    result.converged = false;
    return result;
  }

  // ── Build index maps ───────────────────────────────────────────────────
  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i) {
    link_pos[problem.traffic.links[i].index] = i;
  }
  std::unordered_map<int, int> route_pos_map;
  const int n_routes = static_cast<int>(problem.routes.size());
  for (int ri = 0; ri < n_routes; ++ri) {
    route_pos_map[problem.routes[static_cast<std::size_t>(ri)].index] = ri;
  }

  // ── CTM time step and simulation steps ────────────────────────────────
  const double dt_ctm = choose_ctm_dt(problem, ctm_opts, dt_sim);
  // Map from CTM steps to simulation steps: T_ctm = T_sim * (dt_sim/dt_ctm)
  const int R = std::max(1, static_cast<int>(std::round(dt_sim / dt_ctm)));
  const int T_ctm = T_sim * R;

  // ── Initialise route flows: equal split (warm start) ─────────────────
  // route_flow[k_dep_ctm][route_pos] [vehicles]
  // We initialise by assigning equally among routes for each demand.
  std::vector<std::vector<double>> route_flow(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  for (const auto& demand : problem.demands) {
    // Map sim departure step -> CTM departure step
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm || demand.vehicles <= kTol) continue;
    // Collect candidate routes
    std::vector<int> cand_rpos;
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        cand_rpos.push_back(route_pos_map.count(route.index)
                                ? route_pos_map.at(route.index) : -1);
      }
    }
    cand_rpos.erase(std::remove(cand_rpos.begin(), cand_rpos.end(), -1), cand_rpos.end());
    if (cand_rpos.empty()) continue;
    const double share = demand.vehicles / static_cast<double>(cand_rpos.size());
    for (int rpos : cand_rpos) {
      route_flow[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)] += share;
    }
  }

  // ── MSA DUE iteration ─────────────────────────────────────────────────
  CTMSimulationResult ctm_res;

  for (int iter = 1; iter <= due_opts.max_iterations; ++iter) {
    // Step 1: Forward CTM pass with current route_flow
    ctm_res = ctm_forward_pass(problem, route_flow, link_pos, route_pos_map,
                               ctm_opts, dt_ctm, T_ctm);

    // Step 2: Wardrop relative gap
    const double rg = wardrop_relative_gap(problem, route_flow, ctm_res,
                                           route_pos_map, T_ctm, R);

    // Step 3: Auxiliary all-or-nothing (or logit) assignment
    auto x_aux = auxiliary_assignment(problem, ctm_res, route_pos_map,
                                      link_pos, ev_options, due_opts, T_ctm, R);

    // Step 4: Max shift (absolute gap)
    double max_shift = 0.0;
    for (int k = 0; k < T_ctm; ++k) {
      for (int ri = 0; ri < n_routes; ++ri) {
        max_shift = std::max(max_shift,
                             std::abs(x_aux[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] -
                                      route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)]));
      }
    }

    // Record history
    const double step_i =
        due_opts.msa_fixed_step > kTol
            ? due_opts.msa_fixed_step
            : 1.0 / static_cast<double>(iter);

    result.history.push_back({iter, max_shift, rg, max_shift});

    // Step 5: MSA update  x^{(i+1)} = (1 - 1/i) x^{(i)} + (1/i) x^{aux}
    for (int k = 0; k < T_ctm; ++k) {
      for (int ri = 0; ri < n_routes; ++ri) {
        route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
            (1.0 - step_i) * route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +
            step_i * x_aux[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
      }
    }

    result.iterations = iter;
    result.gap = max_shift;
    result.relative_gap = rg;

    if (rg < due_opts.convergence_tol) {
      result.converged = true;
      break;
    }
  }

  // ── Final forward pass with converged flows ───────────────────────────
  result.final_ctm = ctm_forward_pass(problem, route_flow, link_pos, route_pos_map,
                                       ctm_opts, dt_ctm, T_ctm);

  // ── Pack route flow result ────────────────────────────────────────────
  // Aggregate CTM steps back to simulation steps.
  for (const auto& demand : problem.demands) {
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!route_pos_map.count(route.index)) continue;
      const int rpos = route_pos_map.at(route.index);
      if (rpos >= static_cast<int>(route_flow[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
      const double x = route_flow[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)];
      result.flow_by_demand_route[demand.index][route.index] = x;
    }
  }

  // ── Session synthesis, dispatch, and queue model ──────────────────────
  // (Formulation B+ charging dispatch — no DC-OPF coupling)
  result.sessions = synthesize_ctm_sessions(
      problem, result.flow_by_demand_route, result.final_ctm,
      ev_options, dt_sim, T_sim, R);

  dispatch_ctm_sessions(problem, ev_options, result.sessions,
                         T_sim, dt_sim, result);

  compute_ctm_station_queue(problem, ev_options, result.sessions,
                             T_sim, result);

  // ── Price-of-anarchy benchmark ─────────────────────────────────────────
  if (due_opts.compute_system_optimal_benchmark) {
    // DUE TSTT: Σ_{d,r} x_{d,r} * T^CTM_{r, k_dep_ctm}
    double due_tstt = 0.0;
    for (const auto& demand : problem.demands) {
      const int k_dep_ctm = demand.departure_step * R;
      for (const auto& route : problem.routes) {
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        double vehicles = 0.0;
        if (result.flow_by_demand_route.count(demand.index)) {
          const auto& rm = result.flow_by_demand_route.at(demand.index);
          if (rm.count(route.index)) vehicles = rm.at(route.index);
        }
        if (vehicles <= kTol) continue;
        double tt = 0.0;
        if (result.final_ctm.route_travel_time.count(route.index)) {
          const auto& tv = result.final_ctm.route_travel_time.at(route.index);
          const int idx = std::min(k_dep_ctm, static_cast<int>(tv.size()) - 1);
          if (idx >= 0) tt = tv[static_cast<std::size_t>(idx)];
        }
        due_tstt += vehicles * tt;
      }
    }
    result.due_tstt_hr = due_tstt;

    // SO TSTT: run Formulation A system-optimal LP on the same network
    EVPowerTrafficOptions so_opts = ev_options;
    so_opts.assignment_model = AssignmentModel::SystemOptimalLP;
    const auto so_result = simulate_ev_power_traffic(problem, so_opts);
    result.so_tstt_hr      = so_result.total_travel_time_hr;
    result.has_so_benchmark = so_result.optimization_solved;
    if (result.so_tstt_hr > kTol) {
      result.poa_tstt_ratio = result.due_tstt_hr / result.so_tstt_hr;
    }
  }

  return result;
}

}  // namespace hacdcpf::evpt
