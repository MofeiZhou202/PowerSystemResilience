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

// link_free_flow_speed() is defined in evpt_internal.hpp.

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

inline double ctm_effective_capacity_veh_hr(const TrafficLink& link,
                                            int step,
                                            double dt) {
  if (dt <= kTol) return 0.0;
  return link_capacity_vehicles(link, step, dt) / dt;
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

}  // anonymous namespace (sections 1–3: private CTM helpers)

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
        const double q_eff = ctm_effective_capacity_veh_hr(
          problem.traffic.links[li], k, dt_ctm);
        const double recv = receiving(ls.cells[0].n,
                                      ls.backward_wave_speed_km_hr,
                                      ls.cell_length_km,
                        q_eff,
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
        const double q_max = ctm_effective_capacity_veh_hr(
          problem.traffic.links[li], k, dt_ctm);
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

    // 4c. Inter-link flows at nodes — SMCA diverge/merge general node model
    //
    // Implements the simultaneous multi-link capacity allocation (SMCA) node
    // model from Daganzo (1995).  For each network node:
    //
    //   Pass 1 (diverge constraint): for each in-link a compute the maximum
    //     exit y_a constrained by every downstream link b:
    //       y_a = min(S_a, min_{b: β_b>0}( R_b / β_{a,b} ))
    //     where β_{a,b} = demand_per_out[b] / total_demand_from_a.
    //
    //   Pass 2 (merge constraint): for each out-link b, if Σ_a y_a β_{a,b} > R_b,
    //     proportionally scale down all in-links contributing to that out-link:
    //       y_a ← y_a × min_{b}( R_b / Σ_a' y_a' β_{a'b} )
    //     (iterative; typically converges in 2 passes for road networks).
    //
    // Station spillback (eq:spillback-receiving): before computing R_b for
    // access-link out-links, reduce R_b by the spillback factor from the
    // associated station queue state.

    // Build map: link_index → spillback factor [0,1] for this CTM step.
    // Uses problem.station_spillback configs + cumulative arrivals up to
    // this step as a proxy for queue depth during the forward pass.
    // (exact queue dynamics run after the forward pass in compute_ctm_station_queue)
    std::unordered_map<int, double> access_link_spillback;  // link_index → factor
    for (const auto& sc : problem.station_spillback) {
      if (sc.access_link_id < 0) continue;
      if (!std::isfinite(sc.queue_capacity) || sc.queue_capacity <= kTol) continue;
      // Proxy queue: sum arrivals up to current CTM step k
      double q_proxy = 0.0;
      if (res.station_arrivals.count(sc.station_id)) {
        const auto& arr = res.station_arrivals.at(sc.station_id);
        // arr is indexed by sim step; map k (CTM) to sim step via T_ctm/arr.size()
        const int k_sim_approx = arr.empty() ? 0
            : std::min(static_cast<int>(arr.size()) - 1,
                       k * static_cast<int>(arr.size()) / std::max(1, T_ctm));
        for (int s = 0; s <= k_sim_approx; ++s)
          q_proxy += arr[static_cast<std::size_t>(s)];
      }
      const double factor = std::max(0.0, 1.0 - q_proxy / sc.queue_capacity);
      access_link_spillback[sc.access_link_id] = factor;
    }

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

      // Compute R_b for each out-link (first-cell receiving capacity).
      // Apply spillback reduction for access links (eq:spillback-receiving).
      std::vector<double> R_b(out_links.size(), 0.0);
      for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
        const CTMLinkState& b_ls = link_states[out_links[oi]];
        R_b[oi] = receiving(b_ls.cells[0].n,
                            b_ls.backward_wave_speed_km_hr,
                            b_ls.cell_length_km,
                            ctm_effective_capacity_veh_hr(
                                problem.traffic.links[out_links[oi]], k, dt_ctm),
                            b_ls.jam_occupancy_per_cell,
                            dt_ctm);
        // Spillback: reduce receiving capacity of access links
        const int b_link_idx = problem.traffic.links[out_links[oi]].index;
        if (access_link_spillback.count(b_link_idx))
          R_b[oi] *= access_link_spillback.at(b_link_idx);
      }

      // ── Per in-link: compute desired exit y_a (pass 1: diverge constraint) ──
      // y_a[ai] will hold the SMCA-constrained exit from in-link a_li.
      // turning_frac[ai][oi] = β_{a,b} = fraction of in-link ai heading to out-link oi.
      struct InLinkData {
        std::size_t li;
        double S_a{0.0};
        double y_a{0.0};
        std::vector<double> beta;     // β_{a,b} for each out-link
        std::vector<double> demand;   // demand_per_out (unnormalized)
        std::vector<double> n_route_last;  // per-route occupancy in last cell
        double n_last{0.0};                // total last-cell occupancy
      };
      std::vector<InLinkData> in_data;
      in_data.reserve(in_links.size());

      for (std::size_t ai = 0; ai < in_links.size(); ++ai) {
        const std::size_t a_li = in_links[ai];
        CTMLinkState& a_ls = link_states[a_li];
        const int M_a = a_ls.n_cells;
        const CTMLinkCell& last_cell = a_ls.cells[static_cast<std::size_t>(M_a - 1)];

        InLinkData d;
        d.li = a_li;
        d.S_a = sending(last_cell.n,
                        a_ls.free_flow_speed_km_hr,
                        a_ls.cell_length_km,
                        ctm_effective_capacity_veh_hr(
                            problem.traffic.links[a_li], k, dt_ctm),
                        dt_ctm);
        d.n_last = last_cell.n;
        d.n_route_last = last_cell.n_route;
        d.demand.resize(out_links.size(), 0.0);
        d.beta.resize(out_links.size(), 0.0);

        if (d.S_a <= kTol) {
          d.y_a = 0.0;
          in_data.push_back(std::move(d));
          continue;
        }

        const int a_idx = problem.traffic.links[a_li].index;

        // Compute demand_per_out via turning fractions or uniform fallback
        if (route_specific && tfm.count(nid) && tfm.at(nid).count(a_idx)) {
          const auto& a_turns = tfm.at(nid).at(a_idx);
          for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
            const int b_idx = problem.traffic.links[out_links[oi]].index;
            if (!a_turns.count(b_idx)) continue;
            const auto& theta = a_turns.at(b_idx);
            double dem = 0.0;
            for (int ri = 0; ri < n_routes; ++ri) {
              const double n_ri =
                  (ri < static_cast<int>(last_cell.n_route.size()))
                      ? last_cell.n_route[static_cast<std::size_t>(ri)] : 0.0;
              const double th =
                  (ri < static_cast<int>(theta.size()))
                      ? theta[static_cast<std::size_t>(ri)] : 0.0;
              dem += n_ri * th;
            }
            d.demand[oi] = dem;
          }
        } else {
          const double share = d.S_a / static_cast<double>(out_links.size());
          std::fill(d.demand.begin(), d.demand.end(), share);
        }

        const double total_dem =
            std::accumulate(d.demand.begin(), d.demand.end(), 0.0);
        if (total_dem <= kTol) {
          d.y_a = 0.0;
          in_data.push_back(std::move(d));
          continue;
        }

        for (std::size_t oi = 0; oi < out_links.size(); ++oi)
          d.beta[oi] = d.demand[oi] / total_dem;

        // Pass 1: diverge — y_a ≤ R_b / β_{a,b} for each b with β_{a,b} > 0
        double y_div = d.S_a;
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          if (d.beta[oi] > kTol)
            y_div = std::min(y_div, R_b[oi] / d.beta[oi]);
        }
        d.y_a = y_div;
        in_data.push_back(std::move(d));
      }

      // ── Pass 2: merge — proportionally scale y_a if Σ_a y_a β_{a,b} > R_b ──
      // Iterate up to 5 times (converges quickly for typical road topologies)
      for (int iter = 0; iter < 5; ++iter) {
        bool changed = false;
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          double total_into_b = 0.0;
          for (const auto& d : in_data) total_into_b += d.y_a * d.beta[oi];
          if (total_into_b > R_b[oi] + kTol) {
            const double merge_scale = R_b[oi] / (total_into_b + kTol);
            for (auto& d : in_data)
              if (d.beta[oi] > kTol) { d.y_a *= merge_scale; changed = true; }
          }
        }
        if (!changed) break;
      }

      // ── Apply flows: update last cell of each in-link and first cell of each out-link ──
      for (std::size_t ai = 0; ai < in_data.size(); ++ai) {
        auto& d = in_data[ai];
        if (d.y_a <= kTol) continue;

        CTMLinkState& a_ls   = link_states[d.li];
        const int M_a        = a_ls.n_cells;
        CTMLinkCell& last_cell = a_ls.cells[static_cast<std::size_t>(M_a - 1)];

        // Remove from last cell
        last_cell.n = std::max(0.0, last_cell.n - d.y_a);

        // Distribute to downstream first cells
        for (std::size_t oi = 0; oi < out_links.size(); ++oi) {
          const double add = d.y_a * d.beta[oi];
          if (add <= kTol) continue;
          CTMLinkState& b_ls = link_states[out_links[oi]];
          b_ls.cells[0].n += add;

          // Attribute to routes
          if (route_specific && !b_ls.cells[0].n_route.empty() &&
              !d.n_route_last.empty()) {
            const double sum_r = d.n_last;  // total last-cell occ before exit
            for (int ri = 0; ri < n_routes; ++ri) {
              const double frac_r =
                  (sum_r > kTol && ri < static_cast<int>(d.n_route_last.size()))
                      ? d.n_route_last[static_cast<std::size_t>(ri)] / sum_r
                      : (n_routes > 0 ? 1.0 / n_routes : 0.0);
              b_ls.cells[0].n_route[static_cast<std::size_t>(ri)] += add * frac_r;
            }
          }

          res.step_link_results[static_cast<std::size_t>(k)][out_links[oi]]
              .total_inflow_veh += add;
        }

        // Update per-route last-cell occupancy (scale by exit fraction)
        if (!last_cell.n_route.empty() && d.n_last > kTol) {
          const double scale = last_cell.n / (d.n_last + kTol);
          for (double& v : last_cell.n_route) v = std::max(0.0, v * scale);
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
                      ctm_effective_capacity_veh_hr(
                        problem.traffic.links[li], k, dt_ctm),
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
      const double q_eff_li = ctm_effective_capacity_veh_hr(
          problem.traffic.links[li], k, dt_ctm);
      const double last_exit = sending(ls.cells[static_cast<std::size_t>(ls.n_cells - 1)].n,
                                        ls.free_flow_speed_km_hr,
                                        ls.cell_length_km,
                        q_eff_li,
                        dt_ctm);
      lr.total_outflow_veh = last_exit;
      if (q_eff_li <= kTol) {
        // Link is unavailable (capacity = 0): report infinite travel time so
        // the auxiliary assignment avoids routes through this link.
        lr.mean_travel_time_hr = std::numeric_limits<double>::infinity();
      } else if (last_exit > kTol) {
        lr.mean_travel_time_hr = tot_occ * dt_ctm / last_exit;
      } else {
        lr.mean_travel_time_hr =
            ls.free_flow_speed_km_hr > kTol
                ? ls.cell_length_km * ls.n_cells / ls.free_flow_speed_km_hr
                : 0.0;
      }
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
    const EVPowerTrafficOptions& ev_opts,
    const DUEOptions& due_opts,
    int T_ctm,
    int R) {

  // ε^E = Σ_{w,r,k} h^E_{w,r,k} · (Ψ^E_{w,r,k} − μ^E_w)
  //       ────────────────────────────────────────────────────  (eq:ev-gap)
  //       Σ_w D^E_w · |μ^E_w| + ε
  //
  // Ψ^E_{w,r,k} = T^CTM_{r,k} + Σ_s π_{s,k} · E_{r,s}/VOT − V2G/VOT
  //               + schedule_delay_cost(...) / VOT     (eq:psi-ev, eq:icv-schedule-delay)
  // All terms in time-equivalent hours.

  double numerator   = 0.0;  // Σ h^E · (Ψ^E − μ^E_w)
  double denominator = 0.0;  // Σ D^E_w · |μ^E_w|

  const double vot = std::max(ev_opts.value_of_time_per_hr, kTol);
  const double ew  = ev_opts.station_energy_cost_weight;
  const bool full_due = (due_opts.due_mode == DUEMode::FullEndogenous);

  auto dep_steps_for = [&](const EVDemand& demand) -> std::vector<int> {
    if (full_due && !demand.departure_window_steps.empty()) {
      return demand.departure_window_steps;
    }
    return {demand.departure_step};
  };

  for (const auto& demand : problem.demands) {
    const auto dep_steps = dep_steps_for(demand);

    // ── Compute μ^E_w = min_r Ψ^E_{w,r,k} ────────────────────────────
    double mu_w = std::numeric_limits<double>::infinity();
    for (int k_sim : dep_steps) {
      const int k_dep_ctm = k_sim * R;
      if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
      const double t_dep_hr = k_sim * ev_opts.time_step_hr;

      for (const auto& route : problem.routes) {
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        if (!ctm_res.route_travel_time.count(route.index)) continue;
        const auto& tt_vec = ctm_res.route_travel_time.at(route.index);
        if (k_dep_ctm >= static_cast<int>(tt_vec.size())) continue;
        const double tt = tt_vec[static_cast<std::size_t>(k_dep_ctm)];
        if (!std::isfinite(tt)) continue;

        double gc = tt;
        // Energy cost: Σ_s π_{s,k} · E_{r,s} / VOT  (charging cost)
        // V2G revenue: − π_{s,k} · E^dis_{r,s} / VOT
        for (const auto& stop : route.charging_stops) {
          const double pi = station_price(problem, ev_opts, stop.station_id, k_sim);
          gc += ew * pi * stop.requested_energy_kwh_per_vehicle / vot;
          if (stop.v2g_capable &&
              stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
            gc -= ew * pi * stop.requested_discharge_energy_kwh_per_vehicle / vot;
          }
        }
        // Schedule delay: C^{schedule} / VOT  (eq:icv-schedule-delay)
        if (demand.desired_arrival_time_hr >= 0.0 &&
            (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
          const double t_arr = t_dep_hr + tt;
          gc += schedule_delay_cost(t_arr, demand.desired_arrival_time_hr,
                                    demand.early_penalty_per_hr,
                                    demand.late_penalty_per_hr) / vot;
        }
        mu_w = std::min(mu_w, gc);
      }
    }
    if (!std::isfinite(mu_w)) continue;

    // ── Numerator: Σ_r x_{d,r} · (Ψ^E_{r} − μ^E_w) ──────────────────
    for (int k_sim : dep_steps) {
      const int k_dep_ctm = k_sim * R;
      if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
      const double t_dep_hr = k_sim * ev_opts.time_step_hr;
      for (const auto& route : problem.routes) {
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        if (!route_pos_map.count(route.index)) continue;
        const int rpos = route_pos_map.at(route.index);
        if (k_dep_ctm >= static_cast<int>(route_flow.size())) continue;
        if (rpos >= static_cast<int>(
                route_flow[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
        const double x = route_flow[static_cast<std::size_t>(k_dep_ctm)]
                                    [static_cast<std::size_t>(rpos)];
        if (x <= kTol) continue;

        if (!ctm_res.route_travel_time.count(route.index)) continue;
        const auto& tt_vec = ctm_res.route_travel_time.at(route.index);
        if (k_dep_ctm >= static_cast<int>(tt_vec.size())) continue;
        const double tt = tt_vec[static_cast<std::size_t>(k_dep_ctm)];
        if (!std::isfinite(tt)) continue;

        double gc = tt;
        for (const auto& stop : route.charging_stops) {
          const double pi = station_price(problem, ev_opts, stop.station_id, k_sim);
          gc += ew * pi * stop.requested_energy_kwh_per_vehicle / vot;
          if (stop.v2g_capable &&
              stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
            gc -= ew * pi * stop.requested_discharge_energy_kwh_per_vehicle / vot;
          }
        }
        if (demand.desired_arrival_time_hr >= 0.0 &&
            (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
          const double t_arr = t_dep_hr + tt;
          gc += schedule_delay_cost(t_arr, demand.desired_arrival_time_hr,
                                    demand.early_penalty_per_hr,
                                    demand.late_penalty_per_hr) / vot;
        }
        numerator += x * (gc - mu_w);
      }
    }

    // ── Denominator: D^E_w · |μ^E_w| ─────────────────────────────────
    denominator += demand.vehicles * std::abs(mu_w);
  }

  if (denominator < kTol) return 0.0;
  return std::max(0.0, numerator / (denominator + kTol));
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
    int R,
    // Optional: station waiting times W_{s,k} [hr] from queue model;
    // used in Ψ^E when ev_opts.queueing_weight > 0.  Pass nullptr to disable.
    const std::unordered_map<int, std::vector<double>>* station_wait_hr = nullptr) {

  const int n_routes = static_cast<int>(problem.routes.size());
  std::vector<std::vector<double>> x_aux(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  const double vot = std::max(ev_opts.value_of_time_per_hr, kTol);
  const double ew  = ev_opts.station_energy_cost_weight;

  for (const auto& demand : problem.demands) {
    // Map simulation departure step -> CTM departure step
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
    // Simulation step for station price lookup
    const int k_sim = demand.departure_step;
    // Departure time [hr] for schedule delay (eq:icv-schedule-delay)
    const double t_dep_hr = k_sim * ev_opts.time_step_hr;

    // Collect candidate routes
    std::vector<const RouteAlternative*> cands;
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        cands.push_back(&route);
      }
    }
    if (cands.empty()) continue;

    // Compute full EV generalized cost Ψ^E_{w,r,k} for a route (eq:psi-ev).
    // Units: time-equivalent hours = gc_$ / VOT.
    auto route_gen_cost = [&](const RouteAlternative* route) -> double {
      double tt = std::numeric_limits<double>::infinity();
      if (ctm_res.route_travel_time.count(route->index)) {
        const auto& v = ctm_res.route_travel_time.at(route->index);
        if (k_dep_ctm < static_cast<int>(v.size()))
          tt = v[static_cast<std::size_t>(k_dep_ctm)];
      }
      if (!std::isfinite(tt)) return tt;

      double gc = tt;

      // Energy cost: Σ_s π_{s,k} · E_{r,s} / VOT  (eq:ev-energy-cost)
      // V2G revenue: − π_{s,k} · E^dis_{r,s} / VOT
      for (const auto& stop : route->charging_stops) {
        const double pi = station_price(problem, ev_opts, stop.station_id, k_sim);
        gc += ew * pi * stop.requested_energy_kwh_per_vehicle / vot;
        if (stop.v2g_capable &&
            stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
          gc -= ew * pi * stop.requested_discharge_energy_kwh_per_vehicle / vot;
        }
      }

      // Schedule delay: C^{schedule} / VOT  (eq:icv-schedule-delay, eq:psi-ev)
      if (demand.desired_arrival_time_hr >= 0.0 &&
          (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
        const double t_arr = t_dep_hr + tt;
        gc += schedule_delay_cost(t_arr, demand.desired_arrival_time_hr,
                                  demand.early_penalty_per_hr,
                                  demand.late_penalty_per_hr) / vot;
      }

      // Station waiting time: α_w · W_{s,k^arr} / VOT  (eq:ev-wait-cost)
      if (ev_opts.queueing_weight > kTol && station_wait_hr != nullptr) {
        // Estimate arrival step from departure step + travel time
        const int k_arr_sim =
            ev_opts.time_step_hr > kTol
                ? k_sim + std::max(0, static_cast<int>(std::ceil(tt / ev_opts.time_step_hr)))
                : k_sim;
        for (const auto& stop : route->charging_stops) {
          auto it = station_wait_hr->find(stop.station_id);
          if (it == station_wait_hr->end()) continue;
          const auto& wvec = it->second;
          if (k_arr_sim >= 0 && k_arr_sim < static_cast<int>(wvec.size())) {
            gc += ev_opts.queueing_weight * wvec[static_cast<std::size_t>(k_arr_sim)] / vot;
          }
        }
      }

      return gc;
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
      // All-or-nothing shortest path (minimum generalized cost, eq:msa-aon-icv)
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

// sections 7a–7b: multi-class DUE helpers (external linkage — used by ctm_due_vi.cpp)

// ────────────────────────────────────────────────────────────────────────────
// 7a. Phase I: ICV auxiliary assignment for multi-class DUE
// ────────────────────────────────────────────────────────────────────────────
// All-or-nothing assignment for ICV demands.  Generalized cost:
//   gc^ICV_{r,k} = VOT_icv * T^CTM_{r,k} + fuel_cost/km * d_r
// ICVs share the route network with EVs; no SOC or charging cost is applied.

std::vector<std::vector<double>> auxiliary_assignment_icv(
    const EVPowerTrafficProblem& problem,
    const CTMSimulationResult& ctm_res,
    const std::unordered_map<int, int>& route_pos_map,
    const std::unordered_map<int, std::size_t>& link_pos,
    int T_ctm,
    int R) {

  const int n_routes = static_cast<int>(problem.routes.size());
  std::vector<std::vector<double>> x_aux(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  for (const auto& demand : problem.icv_demands) {
    const int k_dep_ctm = demand.departure_step * R;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
    const double vot = std::max(demand.value_of_time_per_hr, kTol);
    // Departure time [hr] — used only if schedule delay is set
    // (departure_step × dt_sim; dt_sim not passed, so we leave departure = 0
    //  and use t_arr = tt directly, consistent with wardrop_relative_gap_icv)
    const double t_dep_hr = 0.0;

    // Candidate routes for this ICV OD pair
    std::vector<const RouteAlternative*> cands;
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        cands.push_back(&route);
      }
    }
    if (cands.empty()) continue;

    // ICV generalized cost Ψ^F_{w,r,k} = T^CTM + fuel·dist/VOT + toll/VOT
    //                                   + schedule_delay/VOT  (eq:psi-icv)
    auto icv_gc = [&](const RouteAlternative* route) -> double {
      double tt = std::numeric_limits<double>::infinity();
      if (ctm_res.route_travel_time.count(route->index)) {
        const auto& v = ctm_res.route_travel_time.at(route->index);
        if (k_dep_ctm < static_cast<int>(v.size()))
          tt = v[static_cast<std::size_t>(k_dep_ctm)];
      }
      if (!std::isfinite(tt)) return tt;

      double gc = tt;
      if (demand.fuel_cost_per_km > kTol) {
        double dist = 0.0;
        for (int li : route->link_indices) {
          if (link_pos.count(li))
            dist += problem.traffic.links[link_pos.at(li)].length_km;
        }
        gc += demand.fuel_cost_per_km * dist / vot;
      }
      gc += route->toll_cost / vot;

      // Schedule delay (eq:icv-schedule-delay)
      if (demand.desired_arrival_time_hr >= 0.0 &&
          (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
        gc += schedule_delay_cost(t_dep_hr + tt, demand.desired_arrival_time_hr,
                                  demand.early_penalty_per_hr,
                                  demand.late_penalty_per_hr) / vot;
      }
      return gc;
    };

    // All-or-nothing: assign to minimum-cost route
    const RouteAlternative* best = nullptr;
    double best_gc = std::numeric_limits<double>::infinity();
    for (const auto* route : cands) {
      const double gc = icv_gc(route);
      if (gc < best_gc) { best_gc = gc; best = route; }
    }
    if (best) {
      const int rpos = route_pos_map.count(best->index)
                           ? route_pos_map.at(best->index) : -1;
      if (rpos >= 0)
        x_aux[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)] +=
            demand.vehicles;
    }
  }
  return x_aux;
}

// Wardrop relative gap for ICV demands.
double wardrop_relative_gap_icv(
    const EVPowerTrafficProblem& problem,
    const std::vector<std::vector<double>>& x_icv,
    const CTMSimulationResult& ctm_res,
    const std::unordered_map<int, int>& route_pos_map,
    const std::unordered_map<int, std::size_t>& link_pos,
    int T_ctm,
    int R) {

  // ε^F = Σ_{w,r,k} h^F_{w,r,k} · (Ψ^F_{w,r,k} − μ^F_w)
  //       ─────────────────────────────────────────────────  (eq:icv-gap)
  //       Σ_w D^F_w · |μ^F_w| + ε
  //
  // Ψ^F_{w,r,k} = T^CTM_{r,k} + fuel_cost·dist/VOT + toll/VOT
  //               + schedule_delay_cost(...) / VOT     (eq:psi-icv)
  // All terms in time-equivalent hours.

  double numerator   = 0.0;
  double denominator = 0.0;

  auto route_dist = [&](const RouteAlternative& r) {
    double d = 0.0;
    for (int li : r.link_indices)
      if (link_pos.count(li))
        d += problem.traffic.links[link_pos.at(li)].length_km;
    return d;
  };

  for (const auto& demand : problem.icv_demands) {
    const int k_dep_ctm = demand.departure_step * R;
    const int k_sim     = demand.departure_step;
    if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
    const double vot = std::max(demand.value_of_time_per_hr, kTol);
    const double t_dep_hr = k_sim * 0.0;  // departure time: use step index if dt known;
    // Note: exact departure_time_hr = departure_step × dt_sim, but dt_sim is not
    // passed here.  We use t^arr = tt (travel time from dep) for schedule delay.
    // This is consistent with auxiliary_assignment_icv which also uses tt directly.

    // ── Compute μ^F_w = min_r Ψ^F_{w,r,k} ────────────────────────────
    double mu_w = std::numeric_limits<double>::infinity();
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!ctm_res.route_travel_time.count(route.index)) continue;
      const auto& tv = ctm_res.route_travel_time.at(route.index);
      if (k_dep_ctm >= static_cast<int>(tv.size())) continue;
      const double tt = tv[static_cast<std::size_t>(k_dep_ctm)];
      if (!std::isfinite(tt)) continue;

      double gc = tt;
      if (demand.fuel_cost_per_km > kTol)
        gc += demand.fuel_cost_per_km * route_dist(route) / vot;
      gc += route.toll_cost / vot;
      // Schedule delay (eq:icv-schedule-delay)
      if (demand.desired_arrival_time_hr >= 0.0 &&
          (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
        gc += schedule_delay_cost(t_dep_hr + tt, demand.desired_arrival_time_hr,
                                  demand.early_penalty_per_hr,
                                  demand.late_penalty_per_hr) / vot;
      }
      mu_w = std::min(mu_w, gc);
    }
    if (!std::isfinite(mu_w)) continue;

    // ── Numerator: Σ_r x^{ICV}_{d,r} · (Ψ^F_{r} − μ^F_w) ────────────
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!route_pos_map.count(route.index)) continue;
      const int rpos = route_pos_map.at(route.index);
      if (k_dep_ctm >= static_cast<int>(x_icv.size())) continue;
      if (rpos >= static_cast<int>(
              x_icv[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
      const double x = x_icv[static_cast<std::size_t>(k_dep_ctm)]
                              [static_cast<std::size_t>(rpos)];
      if (x <= kTol) continue;

      if (!ctm_res.route_travel_time.count(route.index)) continue;
      const auto& tv = ctm_res.route_travel_time.at(route.index);
      if (k_dep_ctm >= static_cast<int>(tv.size())) continue;
      const double tt = tv[static_cast<std::size_t>(k_dep_ctm)];
      if (!std::isfinite(tt)) continue;

      double gc = tt;
      if (demand.fuel_cost_per_km > kTol)
        gc += demand.fuel_cost_per_km * route_dist(route) / vot;
      gc += route.toll_cost / vot;
      if (demand.desired_arrival_time_hr >= 0.0 &&
          (demand.early_penalty_per_hr > kTol || demand.late_penalty_per_hr > kTol)) {
        gc += schedule_delay_cost(t_dep_hr + tt, demand.desired_arrival_time_hr,
                                  demand.early_penalty_per_hr,
                                  demand.late_penalty_per_hr) / vot;
      }
      numerator += x * (gc - mu_w);
    }

    // ── Denominator: D^F_w · |μ^F_w| ─────────────────────────────────
    denominator += demand.vehicles * std::abs(mu_w);
  }

  if (denominator < kTol) return 0.0;
  return std::max(0.0, numerator / (denominator + kTol));
}

// ────────────────────────────────────────────────────────────────────────────
// 7.  Select CTM time step (auto CFL check)
// ────────────────────────────────────────────────────────────────────────────
// choose_ctm_dt() is defined in evpt_internal.hpp.

// end sections 7a–7b

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

std::vector<EVChargingSession> synthesize_ctm_sessions_full_due(
    const EVPowerTrafficProblem& problem,
    const std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>>& flow_map,
    const CTMSimulationResult& final_ctm,
    const EVPowerTrafficOptions& ev_options,
    double dt_sim,
    int T_sim,
    int R) {
  std::vector<EVChargingSession> sessions;
  int next_idx = 1;

  for (const auto& demand : problem.demands) {
    auto dit = flow_map.find(demand.index);
    if (dit == flow_map.end()) continue;
    for (const auto& [dep_step, route_map] : dit->second) {
      const int k_dep_ctm = dep_step * R;
      for (const auto& route : problem.routes) {
        if (route.charging_stops.empty()) continue;
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        double vehicles = 0.0;
        auto rit = route_map.find(route.index);
        if (rit != route_map.end()) vehicles = rit->second;
        if (vehicles <= kTol) continue;

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
        const int k_arr_sim = dep_step + steps_off;
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
// 10b. Station waiting time from CTM station arrivals (used inside MSA loop)
// ────────────────────────────────────────────────────────────────────────────
// Computes W_{s,k} [hr] using a simple queue model fed directly by
// CTMSimulationResult::station_arrivals.  This provides an approximation of
// station congestion during each MSA iteration for use in Ψ^E (eq:ev-wait-cost).
// Only produces non-zero values when ev_options.queueing_weight > 0.
//
// Queue dynamics (eq:evpt-queue-serve, eq:evpt-queue-update):
//   u_{s,k}   = min(Q_{s,k} + λ_{s,k}, service_cap_{s,k})
//   Q_{s,k+1} = max(0, Q_{s,k} + λ_{s,k} − u_{s,k})
//   W_{s,k}   = (Q_{s,k} / μ_s) · Δt_sim  [hr]  (Little's law, eq:evpt-wait)

std::unordered_map<int, std::vector<double>>
compute_station_wait_from_arrivals(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options,
    const CTMSimulationResult& ctm_res,
    int T_sim,
    int T_ctm,
    int R) {

  const auto charge_cap = station_capacity_kw(problem.system, ev_options);
  const auto stn_ids    = active_station_ids(problem);
  const double dt_sim   = ev_options.time_step_hr;
  const double p_veh    =
      std::max(kTol, ev_options.default_route_stop_power_kw_per_vehicle);

  std::unordered_map<int, std::vector<double>> wait;
  for (int sid : stn_ids) {
    wait[sid].assign(static_cast<std::size_t>(T_sim), 0.0);

    const double cap_kw =
        charge_cap.count(sid) ? charge_cap.at(sid)
                               : ev_options.default_station_power_kw;
    // Service rate μ_s [veh/step]: vehicles that can be served per sim step
    // μ_s = P^max_s / (p_veh · 1 step) — simplified (dwell = 1 step)
    const double mu_s = cap_kw / p_veh;
    // n_plug bound for throughput (possibly ∞ when no plug count specified)
    const double n_plug = n_plugs_for_station(problem.system, sid);

    double q = 0.0;
    for (int ks = 0; ks < T_sim; ++ks) {
      // Aggregate CTM arrivals over R CTM steps for this sim step
      double lambda = 0.0;
      if (ctm_res.station_arrivals.count(sid)) {
        const auto& arr = ctm_res.station_arrivals.at(sid);
        for (int r = 0; r < R; ++r) {
          const int kc = ks * R + r;
          if (kc < T_ctm && kc < static_cast<int>(arr.size()))
            lambda += arr[static_cast<std::size_t>(kc)];
        }
      }

      // W_{s,k} = q / μ_s · Δt  [hr]  (Little's law, eq:evpt-wait)
      wait[sid][static_cast<std::size_t>(ks)] =
          mu_s > kTol ? q / mu_s * dt_sim : 0.0;

      // Service throughput: min(q + λ, μ_s) and optionally bounded by n_plug
      const double service_cap = std::isinf(n_plug) ? mu_s : std::min(mu_s, n_plug);
      const double u = std::min(q + lambda, service_cap);
      q = std::max(0.0, q + lambda - u);
    }
  }
  return wait;
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

// ─────────────────────────────────────────────────────────────────────────
// Full DUE: endogenous departure-time auxiliary assignment (eq:full-due)
// ─────────────────────────────────────────────────────────────────────────
// For each EV demand with non-empty departure_window_steps, assigns ALL
// vehicles to the (departure_step, route) pair with minimum generalised cost:
//   C^full_{w,k,r} = VOT · T^CTM_{r,k} + C^charge_{r,k}
//                   + γ_e · max(0, t*_w − t^arr) + γ_l · max(0, t^arr − t*_w)
// This is the "all-or-nothing" direction for the outer departure-time MSA.
// Returns the full route_flow-shaped auxiliary matrix.
namespace {
std::vector<std::vector<double>> departure_time_aon_assignment(
    const EVPowerTrafficProblem& problem,
    const CTMSimulationResult& ctm_res,
    const std::unordered_map<int, int>& route_pos_map,
    const std::unordered_map<int, std::size_t>& link_pos,
    const EVPowerTrafficOptions& ev_options,
    int T_ctm,
    int R)
{
  const int n_routes = static_cast<int>(problem.routes.size());
  std::vector<std::vector<double>> x_aux(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  for (const auto& demand : problem.demands) {
    // Determine candidate departure steps
    const std::vector<int>& dep_steps = demand.departure_window_steps.empty()
        ? std::vector<int>{demand.departure_step}
        : demand.departure_window_steps;

    // Collect OD-compatible routes
    const auto routes = candidate_routes(problem, demand);
    if (routes.empty()) continue;

    double min_cost = std::numeric_limits<double>::infinity();
    int best_kdep_ctm = -1;
    int best_rpos     = -1;

    for (int dep_step : dep_steps) {
      const int k_ctm = dep_step * R;
      if (k_ctm < 0 || k_ctm >= T_ctm) continue;

      for (const auto* route : routes) {
        // CTM travel time at this departure step
        double tt_hr = 0.0;
        if (ctm_res.route_travel_time.count(route->index)) {
          const auto& tv = ctm_res.route_travel_time.at(route->index);
          const int ku = std::min(k_ctm, static_cast<int>(tv.size()) - 1);
          if (ku >= 0) tt_hr = tv[static_cast<std::size_t>(ku)];
        } else {
          for (int li : route->link_indices) {
            auto it = link_pos.find(li);
            if (it == link_pos.end()) { tt_hr = std::numeric_limits<double>::infinity(); break; }
            tt_hr += problem.traffic.links[it->second].free_flow_time_hr;
          }
        }
        if (!std::isfinite(tt_hr)) continue;

        // Charging cost at departure step price
        double charge_cost = 0.0;
        for (const auto& stop : route->charging_stops) {
          for (const auto& pp : problem.station_prices) {
            if (pp.station_id != stop.station_id) continue;
            const double p = dep_step < static_cast<int>(pp.price_per_kwh.size())
                ? pp.price_per_kwh[static_cast<std::size_t>(dep_step)]
                : ev_options.default_station_price_per_kwh;
            charge_cost += p * stop.requested_energy_kwh_per_vehicle;
            if (stop.v2g_capable && stop.requested_discharge_energy_kwh_per_vehicle > kTol)
              charge_cost -= p * stop.requested_discharge_energy_kwh_per_vehicle;
          }
        }

        // Schedule delay cost
        const double t_arr_hr = dep_step * ev_options.time_step_hr + tt_hr;
        const double sd = schedule_delay_cost(
            t_arr_hr, demand.desired_arrival_time_hr,
            demand.early_penalty_per_hr, demand.late_penalty_per_hr);

        const double cost = ev_options.value_of_time_per_hr * tt_hr
            + ev_options.station_energy_cost_weight * charge_cost
            + sd + route->toll_cost;

        if (cost < min_cost - kTol && route_pos_map.count(route->index)) {
          min_cost = cost;
          best_kdep_ctm = k_ctm;
          best_rpos = route_pos_map.at(route->index);
        }
      }
    }

    if (best_kdep_ctm >= 0 && best_rpos >= 0)
      x_aux[static_cast<std::size_t>(best_kdep_ctm)][static_cast<std::size_t>(best_rpos)]
          += demand.vehicles;
  }
  return x_aux;
}
}  // anonymous namespace (full-DUE helpers)

CTMDUEResult simulate_ev_power_traffic_ctm_due(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options,
    const CTMOptions& ctm_opts,
    const DUEOptions& due_opts) {

  if (ev_options.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_options;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_options);
    return simulate_ev_power_traffic_ctm_due(generated,
                                             local_opts,
                                             ctm_opts,
                                             due_opts);
  }

  CTMDUEResult result;

  if (ev_options.require_exact_mathematical_model ||
      due_opts.require_exact_mathematical_model) {
    result.converged = false;
    result.vi_certificate_available = false;
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: CTM-DUE simulation is an iterative residual solver with "
        "post-DUE greedy charging dispatch, not a globally certified full "
        "mathematical programme";
    return result;
  }

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
  // Full DUE: spread evenly across all departure steps in departure_window_steps.
  // Fixed-departure DUE: use demand.departure_step only.
  const bool full_due = (due_opts.due_mode == DUEMode::FullEndogenous);
  std::vector<std::vector<double>> route_flow(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    // Collect departure steps
    std::vector<int> dep_steps;
    if (full_due && !demand.departure_window_steps.empty()) {
      dep_steps = demand.departure_window_steps;
    } else {
      dep_steps = {demand.departure_step};
    }
    // Collect candidate routes
    std::vector<int> cand_rpos;
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        const int rp = route_pos_map.count(route.index) ? route_pos_map.at(route.index) : -1;
        if (rp >= 0) cand_rpos.push_back(rp);
      }
    }
    if (cand_rpos.empty()) continue;
    const double share =
        demand.vehicles / (static_cast<double>(dep_steps.size()) *
                           static_cast<double>(cand_rpos.size()));
    for (int dep : dep_steps) {
      const int kd = dep * R;
      if (kd < 0 || kd >= T_ctm) continue;
      for (int rpos : cand_rpos)
        route_flow[static_cast<std::size_t>(kd)][static_cast<std::size_t>(rpos)] += share;
    }
  }

  // ── Phase I: multi-class (heterogeneous ICV+EV) DUE ──────────────────
  // When enable_multiclass = true and icv_demands is non-empty, maintain
  // separate EV and ICV route-flow matrices and combine them with PCE
  // weighting for the CTM forward pass.
  //
  // PCE-weighted combined flow:
  //   combined[k][r] = pce_ev * x_ev[k][r] + pce_icv * x_icv[k][r]
  //
  // Sending/receiving are computed on combined so all classes compete for
  // the same physical capacity.  Per-class route costs differ:
  //   ICV: VOT_icv * T^CTM + fuel_cost/km * dist
  //   EV:  VOT_ev  * T^CTM + electricity_cost  (unchanged)

  const bool do_multiclass =
      ctm_opts.enable_multiclass && !problem.icv_demands.empty();

  std::vector<std::vector<double>> x_ev;    // EV-only flows (PCE=1 units)
  std::vector<std::vector<double>> x_icv;   // ICV-only flows (PCE=1 units)

  if (do_multiclass) {
    // x_ev initialised from the equal-split above (all current route_flow = EV)
    x_ev = route_flow;
    // x_icv: equal split across candidate routes per ICV demand
    x_icv.assign(
        static_cast<std::size_t>(T_ctm),
        std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));
    for (const auto& demand : problem.icv_demands) {
      const int k_dep_ctm = demand.departure_step * R;
      if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm || demand.vehicles <= kTol) continue;
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
      for (int rpos : cand_rpos)
        x_icv[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)] += share;
    }

    // Rebuild combined PCE-weighted route_flow for the first CTM pass
    const double pce_ev  = std::max(kTol, ctm_opts.ev_pce);
    const double pce_icv = std::max(kTol, ctm_opts.icv_pce);
    for (int k = 0; k < T_ctm; ++k)
      for (int ri = 0; ri < n_routes; ++ri)
        route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
            pce_ev  * x_ev [static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +
            pce_icv * x_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
  }

  // ── MSA DUE iteration ─────────────────────────────────────────────────
  CTMSimulationResult ctm_res;

  for (int iter = 1; iter <= due_opts.max_iterations; ++iter) {
    // Step 1: Forward CTM pass with current route_flow
    ctm_res = ctm_forward_pass(problem, route_flow, link_pos, route_pos_map,
                               ctm_opts, dt_ctm, T_ctm);

    // Step 1b: Station waiting times W_{s,k} from current arrivals (eq:ev-wait-cost).
    // Used in Ψ^E when queueing_weight > 0; otherwise an empty map is passed.
    std::unordered_map<int, std::vector<double>> station_wait;
    if (ev_options.queueing_weight > kTol) {
      station_wait = compute_station_wait_from_arrivals(
          problem, ev_options, ctm_res, T_sim, T_ctm, R);
    }

    // Step 2: Wardrop relative gap (EV gap; include ICV gap for multi-class)
    // In multi-class mode, pass x_ev (EV-only flow) so gap is computed on EV
    // vehicles only — not the PCE-combined route_flow which includes ICV flow.
    const double rg_ev = wardrop_relative_gap(problem,
                                               do_multiclass ? x_ev : route_flow,
                                               ctm_res,
                                               route_pos_map, ev_options,
                                               due_opts, T_ctm, R);
    const double rg_icv = do_multiclass
        ? wardrop_relative_gap_icv(problem, x_icv, ctm_res,
                                   route_pos_map, link_pos, T_ctm, R)
        : 0.0;
    const double rg = std::max(rg_ev, rg_icv);

    // Step 3: Auxiliary all-or-nothing (or logit) assignment per class.
    // Full DUE (FullEndogenous): use departure_time_aon_assignment which
    // jointly minimises travel time + schedule delay over all (dep_step, route)
    // pairs.  FixedDeparture: use the existing route-choice-only assignment.
    const auto* wait_ptr = ev_options.queueing_weight > kTol ? &station_wait : nullptr;
    auto x_ev_aux = full_due
        ? departure_time_aon_assignment(problem, ctm_res, route_pos_map,
                                        link_pos, ev_options, T_ctm, R)
        : auxiliary_assignment(problem, ctm_res, route_pos_map,
                               link_pos, ev_options, due_opts, T_ctm, R,
                               wait_ptr);
    std::vector<std::vector<double>> x_icv_aux;
    if (do_multiclass)
      x_icv_aux = auxiliary_assignment_icv(problem, ctm_res, route_pos_map,
                                           link_pos, T_ctm, R);

    // Step 4: Max shift (absolute gap across both classes)
    double max_shift = 0.0;
    const auto& base_flow = do_multiclass ? x_ev : route_flow;
    for (int k = 0; k < T_ctm; ++k) {
      for (int ri = 0; ri < n_routes; ++ri) {
        max_shift = std::max(max_shift,
            std::abs(x_ev_aux[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] -
                     base_flow [static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)]));
      }
    }
    if (do_multiclass) {
      for (int k = 0; k < T_ctm; ++k)
        for (int ri = 0; ri < n_routes; ++ri)
          max_shift = std::max(max_shift,
              std::abs(x_icv_aux[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] -
                       x_icv    [static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)]));
    }

    // Record history
    const double step_i =
        due_opts.msa_fixed_step > kTol
            ? due_opts.msa_fixed_step
            : 1.0 / static_cast<double>(iter);

    result.history.push_back({iter, max_shift, rg, max_shift});

    // Step 5: MSA update
    if (do_multiclass) {
      const double pce_ev  = std::max(kTol, ctm_opts.ev_pce);
      const double pce_icv = std::max(kTol, ctm_opts.icv_pce);
      for (int k = 0; k < T_ctm; ++k) {
        for (int ri = 0; ri < n_routes; ++ri) {
          const std::size_t ki = static_cast<std::size_t>(k);
          const std::size_t ri2 = static_cast<std::size_t>(ri);
          x_ev [ki][ri2] = (1.0 - step_i) * x_ev [ki][ri2] + step_i * x_ev_aux [ki][ri2];
          x_icv[ki][ri2] = (1.0 - step_i) * x_icv[ki][ri2] + step_i * x_icv_aux[ki][ri2];
          route_flow[ki][ri2] = pce_ev * x_ev[ki][ri2] + pce_icv * x_icv[ki][ri2];
        }
      }
    } else {
      for (int k = 0; k < T_ctm; ++k) {
        for (int ri = 0; ri < n_routes; ++ri) {
          route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
              (1.0 - step_i) * route_flow[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +
              step_i * x_ev_aux[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
        }
      }
    }

    result.iterations = iter;
      result.gap = max_shift;
      result.relative_gap = rg;
      result.ev_relative_gap  = rg_ev;   // ε^E (eq:ev-gap)
      result.icv_relative_gap = rg_icv;  // ε^F (eq:icv-gap)
      result.departure_time_max_shift_veh = full_due ? max_shift : 0.0;

    if (rg < due_opts.convergence_tol) {
      result.converged = true;
      break;
    }
  }

  // ── Final forward pass with converged flows ───────────────────────────
  result.final_ctm = ctm_forward_pass(problem, route_flow, link_pos, route_pos_map,
                                       ctm_opts, dt_ctm, T_ctm);
  result.full_due_enabled = full_due;

  // ── Pack route flow result ────────────────────────────────────────────
  // EV route flows (from x_ev when multi-class, from route_flow otherwise)
  for (const auto& demand : problem.demands) {
    std::vector<int> dep_steps;
    if (full_due && !demand.departure_window_steps.empty()) {
      dep_steps = demand.departure_window_steps;
    } else {
      dep_steps = {demand.departure_step};
    }
    const auto& ev_flow = do_multiclass ? x_ev : route_flow;
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!route_pos_map.count(route.index)) continue;
      const int rpos = route_pos_map.at(route.index);
      double total = 0.0;
      for (int dep : dep_steps) {
        const int k_dep_ctm = dep * R;
        if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
        if (rpos >= static_cast<int>(
                ev_flow[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
        const double x = ev_flow[static_cast<std::size_t>(k_dep_ctm)]
                                [static_cast<std::size_t>(rpos)];
        result.flow_by_demand_departure_route[demand.index][dep][route.index] = x;
        total += x;
      }
      result.flow_by_demand_route[demand.index][route.index] = total;
    }
  }

  // ICV route flows (Phase I multi-class)
  if (do_multiclass) {
    for (const auto& demand : problem.icv_demands) {
      const int k_dep_ctm = demand.departure_step * R;
      if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
      for (const auto& route : problem.routes) {
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        if (!route_pos_map.count(route.index)) continue;
        const int rpos = route_pos_map.at(route.index);
        if (rpos >= static_cast<int>(x_icv[static_cast<std::size_t>(k_dep_ctm)].size())) continue;
        const double x = x_icv[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rpos)];
        result.flow_by_icv_route[demand.index][route.index] = x;
      }
    }

    // Per-class stats (Phase I)
    {
      CTMDUEResult::PerClassStats ev_stats;
      ev_stats.vehicle_class = VehicleClass::EV;
      ev_stats.converged     = result.converged;
      ev_stats.relative_gap  = wardrop_relative_gap(problem, x_ev, result.final_ctm,
                                                     route_pos_map, ev_options,
                                                     due_opts, T_ctm, R);
      for (const auto& d : problem.demands) ev_stats.total_demand += d.vehicles;
      for (const auto& d : problem.demands) {
        const int k = d.departure_step * R;
        for (const auto& route : problem.routes) {
          if (route.origin_node != d.origin_node ||
              route.destination_node != d.destination_node) continue;
          if (!route_pos_map.count(route.index)) continue;
          const int rpos = route_pos_map.at(route.index);
          if (k >= static_cast<int>(x_ev.size())) continue;
          const double x = x_ev[static_cast<std::size_t>(k)][static_cast<std::size_t>(rpos)];
          double tt = 0.0;
          if (result.final_ctm.route_travel_time.count(route.index)) {
            const auto& tv = result.final_ctm.route_travel_time.at(route.index);
            if (k < static_cast<int>(tv.size()))
              tt = tv[static_cast<std::size_t>(k)];
          }
          ev_stats.tstt_hr += x * (std::isfinite(tt) ? tt : 0.0);
        }
      }
      result.class_stats.push_back(ev_stats);

      CTMDUEResult::PerClassStats icv_stats;
      icv_stats.vehicle_class = VehicleClass::ICV;
      icv_stats.converged     = result.converged;
      icv_stats.relative_gap  = wardrop_relative_gap_icv(problem, x_icv, result.final_ctm,
                                                          route_pos_map, link_pos, T_ctm, R);
      for (const auto& d : problem.icv_demands) icv_stats.total_demand += d.vehicles;
      for (const auto& d : problem.icv_demands) {
        const int k = d.departure_step * R;
        for (const auto& route : problem.routes) {
          if (route.origin_node != d.origin_node ||
              route.destination_node != d.destination_node) continue;
          if (!route_pos_map.count(route.index)) continue;
          const int rpos = route_pos_map.at(route.index);
          if (k >= static_cast<int>(x_icv.size())) continue;
          const double x = x_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(rpos)];
          double tt = 0.0;
          if (result.final_ctm.route_travel_time.count(route.index)) {
            const auto& tv = result.final_ctm.route_travel_time.at(route.index);
            if (k < static_cast<int>(tv.size()))
              tt = tv[static_cast<std::size_t>(k)];
          }
          icv_stats.tstt_hr += x * (std::isfinite(tt) ? tt : 0.0);
        }
      }
      result.class_stats.push_back(icv_stats);
    }

    // Per-link per-class occupancy estimates in step results (Phase I)
    // Distributed proportionally to PCE-weighted class flow totals.
    const double pce_ev  = std::max(kTol, ctm_opts.ev_pce);
    const double pce_icv = std::max(kTol, ctm_opts.icv_pce);
    for (int k = 0; k < T_ctm && k < static_cast<int>(result.final_ctm.step_link_results.size()); ++k) {
      // Aggregate EV and ICV demand at this CTM step
      double total_ev_dep = 0.0, total_icv_dep = 0.0;
      for (int ri = 0; ri < n_routes; ++ri) {
        total_ev_dep  += x_ev [static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
        total_icv_dep += x_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
      }
      const double total_pce = pce_ev * total_ev_dep + pce_icv * total_icv_dep;
      const double frac_ev  = total_pce > kTol ? pce_ev  * total_ev_dep  / total_pce : 0.5;
      const double frac_icv = total_pce > kTol ? pce_icv * total_icv_dep / total_pce : 0.5;
      for (auto& lr : result.final_ctm.step_link_results[static_cast<std::size_t>(k)]) {
        lr.class_occupancy_veh[static_cast<int>(VehicleClass::EV)]  = lr.total_occupancy_veh * frac_ev;
        lr.class_occupancy_veh[static_cast<int>(VehicleClass::ICV)] = lr.total_occupancy_veh * frac_icv;
      }
    }
  }

  // ── Session synthesis, dispatch, and queue model ──────────────────────
  // (Formulation B+ charging dispatch — no DC-OPF coupling)
  result.sessions = full_due
      ? synthesize_ctm_sessions_full_due(
            problem, result.flow_by_demand_departure_route,
            result.final_ctm, ev_options, dt_sim, T_sim, R)
      : synthesize_ctm_sessions(
            problem, result.flow_by_demand_route, result.final_ctm,
            ev_options, dt_sim, T_sim, R);

  dispatch_ctm_sessions(problem, ev_options, result.sessions,
                         T_sim, dt_sim, result);
  // dispatch_ctm_sessions uses a greedy per-session heuristic, not a
  // monolithic optimal charging/V2G programme.
  result.charging_dispatch_is_greedy = true;

  compute_ctm_station_queue(problem, ev_options, result.sessions,
                             T_sim, result);

  // ── Price-of-anarchy benchmark ─────────────────────────────────────────
  if (due_opts.compute_system_optimal_benchmark) {
    // DUE TSTT: Σ_{d,r} x_{d,r} * T^CTM_{r, k_dep_ctm}
    double due_tstt = 0.0;
    for (const auto& demand : problem.demands) {
      std::vector<int> dep_steps;
      if (full_due && !demand.departure_window_steps.empty()) {
        dep_steps = demand.departure_window_steps;
      } else {
        dep_steps = {demand.departure_step};
      }
      for (const auto& route : problem.routes) {
        if (route.origin_node != demand.origin_node ||
            route.destination_node != demand.destination_node) continue;
        for (int dep : dep_steps) {
          const int k_dep_ctm = dep * R;
          double vehicles = 0.0;
          auto dit = result.flow_by_demand_departure_route.find(demand.index);
          if (dit != result.flow_by_demand_departure_route.end()) {
            auto kit = dit->second.find(dep);
            if (kit != dit->second.end()) {
              auto rit = kit->second.find(route.index);
              if (rit != kit->second.end()) vehicles = rit->second;
            }
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
