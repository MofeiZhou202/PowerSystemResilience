#include "hacdcpf/engine/solver/native/nle/newton_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/power_flow/nonlinear_scaling.hpp"
#include "hacdcpf/power_flow/nonmonotone_linesearch.hpp"
#include "hacdcpf/power_flow/lm_trust_region.hpp"
#include "hacdcpf/power_flow/newton_krylov.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"
#include "hacdcpf/engine/kernel/globalization/globalization.hpp"

namespace hacdcpf::engine {

namespace {

constexpr double kMinVm = 0.05;
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

enum class BusControlState {
  FixedPQ,
  PVActive,
  PQLimited,
};

void reset_or_resize(Eigen::VectorXd& v, int n) {
  if (v.size() != n) {
    v = Eigen::VectorXd::Zero(n);
  } else {
    v.setZero();
  }
}

int first_slack_or_default(const std::vector<ACBus>& ac_buses) {
  for (int i = 0; i < static_cast<int>(ac_buses.size()); ++i) {
    if (ac_buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      return i;
    }
  }
  return ac_buses.empty() ? -1 : 0;
}

/**
 * @brief Detect DC islands (connected components) of the DC network.
 *
 * Adjacency comes from in-service DC branches and DC circuit breakers only.
 * Neither VSC nor DC/DC converters are treated as DC-bus links: a VSC touches
 * exactly one DC bus, and a DC/DC is a power-electronic interface that couples
 * power between two DC sides without galvanically merging them (multi-converter
 * model §5.2).  This matches the coordination checker's
 * `compute_dc_voltage_islands`, so the solver and the pre-solve feasibility
 * checks now agree on what constitutes a DC voltage island.
 *
 * @param data            Solver data containing DC buses/branches.
 * @param num_components  [out] number of connected components found.
 * @return component id (0-based) for each DC bus index (empty if no DC buses).
 */
std::vector<int> compute_dc_components(const SolverData& data, int& num_components) {
  const int ndc = static_cast<int>(data.dc_buses.size());
  num_components = 0;
  if (ndc == 0) return {};

  std::vector<std::vector<int>> adj(static_cast<size_t>(ndc));
  for (const auto& br : data.dc_branches) {
    if (!br.in_service) continue;
    const int f = br.from_bus - 1;
    const int t = br.to_bus - 1;
    if (f >= 0 && f < ndc && t >= 0 && t < ndc) {
      adj[static_cast<size_t>(f)].push_back(t);
      adj[static_cast<size_t>(t)].push_back(f);
    }
  }
  // NOTE: DC/DC converters are deliberately NOT added as adjacency edges — they
  // are power-coupling devices, not conductive links (multi-converter model
  // §5.2).  Each DC side keeps its own voltage island and its own reference.
  // (Closed DC circuit breakers are already folded into dc_branches by the
  // canonical projection, so they need no separate handling here.)

  std::vector<int> component(static_cast<size_t>(ndc), -1);
  for (int start = 0; start < ndc; ++start) {
    if (component[static_cast<size_t>(start)] >= 0) continue;
    std::vector<int> queue;
    queue.push_back(start);
    component[static_cast<size_t>(start)] = num_components;
    size_t head = 0;
    while (head < queue.size()) {
      const int u = queue[head++];
      for (int v : adj[static_cast<size_t>(u)]) {
        if (component[static_cast<size_t>(v)] < 0) {
          component[static_cast<size_t>(v)] = num_components;
          queue.push_back(v);
        }
      }
    }
    ++num_components;
  }
  return component;
}

// Structural-rank screen for the assembled Jacobian pattern (multi-converter
// model §7.4).  An all-zero column means a solved variable that no equation
// constrains; an all-zero row means an equation that depends on no solved
// variable.  Either implies the Jacobian is structurally singular.  Operates on
// the compressed column-major sparsity pattern.
struct StructuralScan {
  int empty_rows{0};
  int empty_cols{0};
};

StructuralScan scan_empty_rows_cols(const Eigen::SparseMatrix<double>& m) {
  StructuralScan s;
  const int ncol = static_cast<int>(m.cols());
  const int nrow = static_cast<int>(m.rows());
  if (ncol == 0 || nrow == 0) return s;
  const int* outer = m.outerIndexPtr();
  for (int j = 0; j < ncol; ++j) {
    if (outer[j + 1] == outer[j]) ++s.empty_cols;
  }
  std::vector<char> row_seen(static_cast<size_t>(nrow), 0);
  const int* inner = m.innerIndexPtr();
  const int nnz = static_cast<int>(m.nonZeros());
  for (int k = 0; k < nnz; ++k) {
    const int r = inner[k];
    if (r >= 0 && r < nrow) row_seen[static_cast<size_t>(r)] = 1;
  }
  for (int i = 0; i < nrow; ++i) {
    if (!row_seen[static_cast<size_t>(i)]) ++s.empty_rows;
  }
  return s;
}

// Realize participation-factor power sharing among co-operating Vdc converters
// (multi-converter model §6.7) at the power-flow level.  Within a coordination
// group of in-service Vdc-mode converters, the total droop stiffness is
// distributed in proportion to each source's participation_factor, so the island
// imbalance is shared per the declared factors (sharing ∝ k_vdc ∝ α).  Applied
// only when the group has ≥2 members and the factors are a valid sharing (all
// ≥0, sum ≈ 1); otherwise the converters keep their configured k_vdc and share
// naturally by their individual droop gains.
void apply_participation_factor_sharing(std::vector<VSCConverter>& converters) {
  std::unordered_map<std::string, std::vector<int>> groups;
  for (int i = 0; i < static_cast<int>(converters.size()); ++i) {
    const auto& c = converters[static_cast<size_t>(i)];
    if (!c.in_service || c.coordination_group_id.empty()) continue;
    if (c.control_mode != ConverterMode::VDC_Q &&
        c.control_mode != ConverterMode::VDC_VAC &&
        c.control_mode != ConverterMode::DC_V_DROOP_AC_V) {
      continue;
    }
    groups[c.coordination_group_id].push_back(i);
  }
  for (auto& [gid, members] : groups) {
    (void)gid;
    if (members.size() < 2) continue;
    double sum_alpha = 0.0;
    bool all_nonneg = true;
    for (int i : members) {
      const double a = converters[static_cast<size_t>(i)].participation_factor;
      if (a < 0.0) all_nonneg = false;
      sum_alpha += a;
    }
    if (!all_nonneg || std::abs(sum_alpha - 1.0) > 1e-6) continue;
    double k_total = 0.0;
    for (int i : members) k_total += std::abs(converters[static_cast<size_t>(i)].k_vdc);
    if (k_total < 1e-12) k_total = static_cast<double>(members.size()) * 0.1;
    for (int i : members) {
      const double a = converters[static_cast<size_t>(i)].participation_factor;
      converters[static_cast<size_t>(i)].k_vdc = k_total * a;
    }
  }
}

/// Result of DC island voltage-reference planning.
struct DcSlackPlan {
  std::vector<int> dc_slacks;               ///< DC bus indices to pin as fixed-voltage slack.
  std::vector<int> promoted_converter_idx;  ///< indices into `converters` promoted PQ->VDC_Q.
  std::vector<int> rigid_converter_idx;     ///< indices into `converters` acting as rigid DC slacks.
  std::vector<int> sole_former_idx;         ///< authored Vdc-mode converters that are
                                            ///< their island's ONLY voltage reference.
  std::vector<std::pair<int, double>> rigid_pins;  ///< (bus index, v_set) for rigid Vdc formers.
  std::vector<std::string> warnings;        ///< human-readable diagnostics.
};

// Net non-converter fixed DC power injection (MW, generation positive) at a DC
// bus, mirroring assemble_dc_injections.  Used to size a rigid sole-former
// converter so it exactly absorbs/supplies the island imbalance.
double net_fixed_dc_injection_mw(const SolverData& data, int bus_idx) {
  const auto& bus = data.dc_buses[static_cast<size_t>(bus_idx)];
  double net = 0.0;
  if (data.dc_loads.empty()) {
    net -= bus.pd_mw;
  } else {
    for (const auto& ld : data.dc_loads) {
      if (ld.bus - 1 == bus_idx) net -= model::effective_load_p_mw(ld);
    }
  }
  for (const auto& st : data.dc_storage) {
    if (st.in_service && st.bus - 1 == bus_idx) net += st.p_mw;
  }
  for (const auto& sg : data.dc_static_generators) {
    if (sg.in_service && sg.bus - 1 == bus_idx) net += sg.p_mw * sg.scaling;
  }
  for (const auto& pv : data.dc_pv_arrays) {
    if (pv.in_service && pv.bus - 1 == bus_idx) net += pv.p_set_mw;
  }
  return net;
}

/**
 * @brief Assign a voltage reference to every DC island, promoting a PQ converter
 *        to Vdc-regulating mode when an island would otherwise have none.
 *
 * Per island the reference is chosen in this order:
 *   1. a DC_V bus                            -> pin that bus (fixed Vdc), as before;
 *   2. a converter already in VDC_Q/VDC_VAC  -> pin no bus, the droop holds Vdc;
 *   3. no reference, an in-service PQ VSC     -> with enable_rigid AND a single-bus
 *      sole-former island, pin the bus at v_set and size the converter so it
 *      exactly balances the island (rigid DC-slack former); otherwise promote the
 *      largest converter to VDC_Q so it regulates Vdc via a stiff droop;
 *   4. no reference and no VSC                -> pin the first non-isolated bus
 *      (legacy fallback) and warn that the reference is non-physical.
 *
 * @param data         Solver data (const).
 * @param converters   The solver's mutable converter copy; promotions mutate it.
 * @param enable_rigid Opt-in: use rigid DC-slack forming for single-bus islands.
 */
DcSlackPlan plan_dc_island_references(const SolverData& data,
                                      std::vector<VSCConverter>& converters,
                                      bool enable_rigid = false) {
  DcSlackPlan plan;
  int num_components = 0;
  const std::vector<int> component = compute_dc_components(data, num_components);
  const int ndc = static_cast<int>(data.dc_buses.size());
  if (ndc == 0) return plan;

  for (int c = 0; c < num_components; ++c) {
    int dc_v_bus = -1;
    int first_non_isolated = -1;
    int first_bus_in_component = -1;
    int non_isolated_count = 0;
    for (int i = 0; i < ndc; ++i) {
      if (component[static_cast<size_t>(i)] != c) continue;
      if (first_bus_in_component < 0) first_bus_in_component = i;
      const auto bt = data.dc_buses[static_cast<size_t>(i)].bus_type;
      if (bt == DCBusType::DC_V && dc_v_bus < 0) dc_v_bus = i;
      if (bt != DCBusType::DC_ISOLATED) {
        if (first_non_isolated < 0) first_non_isolated = i;
        ++non_isolated_count;
      }
    }

    // Does a converter already regulate Vdc on a bus in this island?
    bool converter_regulates = false;
    int regulating_count = 0;
    int regulating_idx = -1;
    for (int gi = 0; gi < static_cast<int>(converters.size()); ++gi) {
      const auto& conv = converters[static_cast<size_t>(gi)];
      if (!conv.in_service) continue;
      const int db = conv.bus_dc - 1;
      if (db < 0 || db >= ndc || component[static_cast<size_t>(db)] != c) continue;
      if (conv.control_mode == ConverterMode::VDC_Q ||
          conv.control_mode == ConverterMode::VDC_VAC ||
          conv.control_mode == ConverterMode::DC_V_DROOP_AC_V) {
        converter_regulates = true;
        ++regulating_count;
        regulating_idx = gi;
      }
    }
    // An authored Vdc-mode converter that is the island's ONLY voltage
    // reference must never be demoted back to PQ by the adaptive mode switch:
    // without it the island has no reference and the Newton system goes
    // singular (the same rationale as the promotion lock).  Record it so the
    // caller can lock it alongside the promoted/rigid formers.
    if (converter_regulates && regulating_count == 1 && regulating_idx >= 0) {
      plan.sole_former_idx.push_back(regulating_idx);
    }

    // A droop DC/DC anchors its OUTPUT bus voltage: in droop mode the output-side
    // injection depends on vdc_out (p_out_ref += k_droop*(vdc_out - v_ref)), so
    // that bus's DC equation is non-degenerate and forms a local reference.
    // Since DC/DC converters no longer merge islands (multi-converter model
    // §5.2), an island fed by a droop DC/DC output must be recognised here, or
    // the planner would needlessly pin it as a non-physical slack.
    bool dcdc_regulates = false;
    for (const auto& dcdc : data.dcdc_converters) {
      if (!dcdc.in_service) continue;
      // A DC/DC forms its OUTPUT bus voltage in Voltage mode, or in Droop mode
      // with a non-trivial gain. Either way its output island has a reference.
      const bool forms =
          dcdc.control_mode == DCDCControlMode::Voltage ||
          (dcdc.control_mode == DCDCControlMode::Droop &&
           std::abs(dcdc.k_droop) >= 1e-12);
      if (!forms) continue;
      const int bo = dcdc.bus_out - 1;
      if (bo >= 0 && bo < ndc && component[static_cast<size_t>(bo)] == c) {
        dcdc_regulates = true;
        break;
      }
    }

    // An LCC station in a characteristic control mode (CEA inverter /
    // constant-alpha rectifier) anchors its DC island's voltage through its
    // external characteristic U_d = U_d0*cos(gamma) - R_c*I_d, playing the
    // same role as a droop VSC — no bus pin or promotion is needed.
    bool lcc_regulates = false;
    for (const auto& lcc : data.lcc_converters) {
      if (!powerflow::lcc_forms_dc_voltage(lcc)) continue;
      const int db = lcc.dc_bus - 1;
      if (db >= 0 && db < ndc && component[static_cast<size_t>(db)] == c) {
        lcc_regulates = true;
        break;
      }
    }

    // 1. DC_V bus present -> pin it.
    if (dc_v_bus >= 0) {
      plan.dc_slacks.push_back(dc_v_bus);
      continue;
    }
    // 2. A VSC holds Vdc via droop, a droop DC/DC forms this island's
    //    output bus, or an LCC station's characteristic anchors Vdc ->
    //    no bus pin needed.
    if (converter_regulates || dcdc_regulates || lcc_regulates) continue;

    // 3. No reference: try to promote the largest in-service PQ converter.
    int best_idx = -1;
    double best_prated = -1.0;
    int master_idx = -1;
    for (int gi = 0; gi < static_cast<int>(converters.size()); ++gi) {
      const auto& conv = converters[static_cast<size_t>(gi)];
      if (!conv.in_service || conv.control_mode != ConverterMode::PQ_MODE) continue;
      const int db = conv.bus_dc - 1;
      if (db < 0 || db >= ndc || component[static_cast<size_t>(db)] != c) continue;
      if (conv.is_master && master_idx < 0) master_idx = gi;
      if (conv.p_rated_mw > best_prated) {
        best_prated = conv.p_rated_mw;
        best_idx = gi;
      }
    }
    // Respect a declared master (multi-converter model §6.6): if the island has a
    // converter flagged is_master, it forms the Vdc reference instead of the
    // largest-rated one, so the user's master-slave designation drives the solve.
    if (master_idx >= 0) best_idx = master_idx;

    const int anchor = (first_non_isolated >= 0) ? first_non_isolated : first_bus_in_component;
    const int anchor_bus_index =
        (anchor >= 0) ? data.dc_buses[static_cast<size_t>(anchor)].index : -1;

    if (best_idx >= 0) {
      auto& conv = converters[static_cast<size_t>(best_idx)];

      // ── Rigid DC-slack former (opt-in, multi-converter model §6.3) ─────────
      // For a single-bus island whose only voltage-forming candidate is this
      // converter (no other VSC, no DC/DC or ER DC port coupling the bus), pin
      // the bus at the setpoint and size the converter so its PQ DC injection
      // exactly cancels the island's fixed DC power.  Vdc is then held rigidly
      // at v_set instead of drifting on a droop, and the converter behaves as
      // the island's DC slack.
      bool rigid_eligible = enable_rigid && non_isolated_count == 1;
      if (rigid_eligible) {
        int vsc_on_island = 0;
        for (const auto& cc : converters) {
          const int db = cc.bus_dc - 1;
          if (cc.in_service && db >= 0 && db < ndc &&
              component[static_cast<size_t>(db)] == c) {
            ++vsc_on_island;
          }
        }
        int coupling_on_island = 0;
        for (const auto& dd : data.dcdc_converters) {
          if (!dd.in_service) continue;
          const int bi = dd.bus_in - 1;
          const int bo = dd.bus_out - 1;
          if ((bi >= 0 && bi < ndc && component[static_cast<size_t>(bi)] == c) ||
              (bo >= 0 && bo < ndc && component[static_cast<size_t>(bo)] == c)) {
            ++coupling_on_island;
          }
        }
        for (const auto& er : data.energy_routers) {
          if (!er.in_service) continue;
          for (const auto& p : er.ports) {
            const int pb = p.bus - 1;
            if (p.in_service && p.port_type == ERPortType::DC && pb >= 0 && pb < ndc &&
                component[static_cast<size_t>(pb)] == c) {
              ++coupling_on_island;
            }
          }
        }
        rigid_eligible = (vsc_on_island == 1 && coupling_on_island == 0);
      }

      if (rigid_eligible) {
        const int bus_pos = conv.bus_dc - 1;
        if (!(conv.v_dc_set_pu > 0.1)) conv.v_dc_set_pu = 1.0;
        const double v_set = conv.v_dc_set_pu;
        const double eta = std::clamp(conv.eta, 0.01, 1.0);
        const double net_mw = net_fixed_dc_injection_mw(data, bus_pos);
        // Size p_set so the PQ DC injection pdc = -(pset + (1-eta)|pset|) exactly
        // cancels the island's fixed injection net_mw (linear loss convention).
        const double pset_mw = (net_mw >= 0.0) ? net_mw / (2.0 - eta) : net_mw / eta;
        const double p_orig = conv.p_set_mw;
        conv.control_mode = ConverterMode::PQ_MODE;
        conv.p_set_mw = pset_mw;
        plan.dc_slacks.push_back(bus_pos);
        plan.rigid_pins.emplace_back(bus_pos, v_set);
        plan.rigid_converter_idx.push_back(best_idx);
        plan.warnings.push_back(
            "DC island has no voltage reference; converter " + std::to_string(conv.index) +
            " (bus_dc=" + std::to_string(conv.bus_dc) + ") rigidly forms Vdc=" +
            std::to_string(v_set) + " pu as a DC-slack former (p_set " +
            std::to_string(p_orig) + " -> " + std::to_string(pset_mw) +
            " MW to balance the island).");
        continue;
      }

      // ── Stiff-droop promotion (default) ────────────────────────────────────
      // A solely auto-promoted converter is the island's de-facto Vdc slack.  A
      // single voltage former should hold Vdc ~rigidly (multi-converter model
      // §6.3), not float on whatever droop gain happened to be configured: a
      // weak k_vdc lets Vdc drift far outside limits to pass the PV/load surplus
      // (e.g. Vdc=1.22 pu for k_vdc=0.1).  Override it with a stiff gain (the
      // same stiffness the energy-router expansion uses) so Vdc stays close to
      // v_dc_set.  This lets a default-drawn VSC feeding a DC island solve
      // cleanly, and removes the old non-physical "pin a bus" fallback for flat
      // droop.
      conv.control_mode = ConverterMode::VDC_Q;
      if (!(conv.v_dc_set_pu > 0.1)) conv.v_dc_set_pu = 1.0;
      const double base = (data.base_mva > 0.0) ? data.base_mva : 100.0;
      const double p_rated_pu =
          ((conv.p_rated_mw > 0.0) ? conv.p_rated_mw : std::abs(conv.p_set_mw)) / base;
      const double k_vdc_stiff = std::max(p_rated_pu * 100.0, 25.0);
      const double k_vdc_orig = conv.k_vdc;
      conv.k_vdc = std::max(std::abs(conv.k_vdc), k_vdc_stiff);
      plan.promoted_converter_idx.push_back(best_idx);
      plan.warnings.push_back(
          "DC island has no voltage reference; auto-promoted PQ converter " +
          std::to_string(conv.index) + " (bus_dc=" + std::to_string(conv.bus_dc) +
          ", p_rated=" + std::to_string(conv.p_rated_mw) +
          " MW) to VDC_Q to form the Vdc reference (k_vdc " +
          std::to_string(k_vdc_orig) + " -> " + std::to_string(conv.k_vdc) +
          " for ~rigid voltage forming, v_dc_set=" + std::to_string(conv.v_dc_set_pu) +
          " pu).");
      // No bus pinned: Vdc is a solved variable held by the converter.
      continue;
    }

    // 4. No reference and no VSC -> legacy fallback pin (warn unless de-energized).
    if (anchor >= 0) {
      plan.dc_slacks.push_back(anchor);
      if (data.dc_buses[static_cast<size_t>(anchor)].bus_type != DCBusType::DC_ISOLATED) {
        plan.warnings.push_back(
            "DC island has no voltage reference and no VSC converter; pinning DC bus " +
            std::to_string(anchor_bus_index) +
            " as a fixed-voltage slack (non-physical reference).");
      }
    }
  }

  return plan;
}

void enforce_vac_setpoints(const std::vector<VSCConverter>& converters, Eigen::VectorXd& vm) {
  const int n = static_cast<int>(vm.size());
  for (const auto& conv : converters) {
    if (!conv.in_service || conv.control_mode != ConverterMode::VDC_VAC) {
      continue;
    }
    const int ac_bus = conv.bus_ac - 1;
    if (ac_bus >= 0 && ac_bus < n) {
      vm[ac_bus] = conv.v_ac_set_pu;
    }
  }
}

int effective_ac_eval_threads(const PowerFlowOptions& opt) {
  if (opt.ac_eval_threads > 0) {
    return opt.ac_eval_threads;
  }
  const unsigned hw = std::thread::hardware_concurrency();
  return (hw == 0) ? 1 : static_cast<int>(hw);
}

void clip_step(Eigen::VectorXd& dx, const JacobianContext& ctx, const PowerFlowOptions& opt) {
  if (dx.size() != ctx.nvar) {
    return;
  }
  const double dtheta = std::abs(opt.max_delta_va_rad);
  const double dvm = std::abs(opt.max_delta_vm_pu);
  const double dvdc = std::abs(opt.max_delta_vdc_pu);

  for (int k = 0; k < ctx.np; ++k) {
    if (dtheta > 0.0) {
      dx[k] = std::clamp(dx[k], -dtheta, dtheta);
    }
  }
  for (int k = 0; k < ctx.nq; ++k) {
    const int idx = ctx.np + k;
    if (dvm > 0.0) {
      dx[idx] = std::clamp(dx[idx], -dvm, dvm);
    }
  }
  for (int k = 0; k < ctx.ndc_eq; ++k) {
    const int idx = ctx.np + ctx.nq + k;
    if (dvdc > 0.0) {
      dx[idx] = std::clamp(dx[idx], -dvdc, dvdc);
    }
  }
}

double estimate_condition_proxy(const Eigen::SparseMatrix<double>& j) {
  if (j.rows() == 0 || j.cols() == 0) {
    return 0.0;
  }

  Eigen::VectorXd row_norm = Eigen::VectorXd::Zero(j.rows());
  Eigen::VectorXd col_norm = Eigen::VectorXd::Zero(j.cols());
  for (int col = 0; col < j.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(j, col); it; ++it) {
      const double a = std::abs(it.value());
      row_norm[it.row()] += a;
      col_norm[col] += a;
    }
  }

  const double tiny = 1e-16;
  double row_max = tiny;
  double row_min = kInf;
  for (int i = 0; i < row_norm.size(); ++i) {
    row_max = std::max(row_max, row_norm[i]);
    if (row_norm[i] > tiny) {
      row_min = std::min(row_min, row_norm[i]);
    }
  }
  if (!std::isfinite(row_min)) {
    row_min = tiny;
  }

  double col_max = tiny;
  double col_min = kInf;
  for (int i = 0; i < col_norm.size(); ++i) {
    col_max = std::max(col_max, col_norm[i]);
    if (col_norm[i] > tiny) {
      col_min = std::min(col_min, col_norm[i]);
    }
  }
  if (!std::isfinite(col_min)) {
    col_min = tiny;
  }

  return (row_max / row_min) * (col_max / col_min);
}

JacobianContext build_jacobian_context(const std::vector<ACBus>& ac_buses,
                                       int ndc,
                                       int slack,
                                       const std::vector<int>& dc_slacks,
                                       const SolverData* data = nullptr,
                                       const std::vector<VSCConverter>* converters_ptr = nullptr) {
  JacobianContext jac_ctx;
  const int n = static_cast<int>(ac_buses.size());

  std::vector<int> pq;
  std::vector<int> pv;
  std::vector<int> non_slack;
  std::vector<int> dc_non_slack;

  pq.reserve(static_cast<size_t>(n));
  pv.reserve(static_cast<size_t>(n));
  non_slack.reserve(static_cast<size_t>(n));
  dc_non_slack.reserve(static_cast<size_t>(ndc));

  for (int i = 0; i < n; ++i) {
    if (i == slack) {
      continue;  // Reference bus: both P/Q equations and θ/Vm eliminated.
    }
    const BusType bt = ac_buses[static_cast<size_t>(i)].bus_type;
    // Additional SLACK buses serve as angle+voltage references for their connected island.
    // They are excluded from the Jacobian (both θ and Vm fixed), giving each disconnected
    // AC area its own reference bus.  This is required for multi-area cases such as
    // case_SyntheticUSA where several AC areas are separated by DC links.
    if (bt == BusType::SLACK) {
      continue;
    }
    if (bt == BusType::PV) {
      pv.push_back(i);
    } else {
      pq.push_back(i);
    }
  }
  // Augmented equations (Direction 2): VDC_VAC converter AC buses must be PQ
  // so that Vm is a free variable and a Q-row exists for the setpoint equation.
  if (data != nullptr && data->enable_augmented_equations && converters_ptr != nullptr) {
    std::unordered_set<int> vdc_vac_buses;
    for (const auto& conv : *converters_ptr) {
      if (!conv.in_service || conv.control_mode != ConverterMode::VDC_VAC) continue;
      const int ac_bus = conv.bus_ac - 1;
      if (ac_bus >= 0 && ac_bus < n && ac_bus != slack) {
        vdc_vac_buses.insert(ac_bus);
      }
    }
    if (!vdc_vac_buses.empty()) {
      // Move VDC_VAC buses from PV to PQ.
      std::vector<int> new_pv;
      for (int bus : pv) {
        if (vdc_vac_buses.count(bus)) {
          pq.push_back(bus);
        } else {
          new_pv.push_back(bus);
        }
      }
      pv = std::move(new_pv);
    }
  }

  // Semi-smooth Newton (Direction 3): PV buses become PQ with NCP equations.
  std::vector<int> ncp_pv_buses;
  if (data != nullptr && data->enable_semi_smooth_newton) {
    ncp_pv_buses = pv;
    pq.insert(pq.end(), pv.begin(), pv.end());
    pv.clear();
  }

  non_slack = pv;
  non_slack.insert(non_slack.end(), pq.begin(), pq.end());

  // Build set of DC slack buses for O(1) lookup
  std::unordered_set<int> dc_slack_set(dc_slacks.begin(), dc_slacks.end());
  for (int i = 0; i < ndc; ++i) {
    if (dc_slack_set.find(i) != dc_slack_set.end()) continue;
    // Isolated DC buses are de-energized and held at fixed voltage, so they are
    // excluded from the solved equation set (mirrors DCSolver::solve).
    if (data != nullptr && i < static_cast<int>(data->dc_buses.size()) &&
        data->dc_buses[static_cast<size_t>(i)].bus_type == DCBusType::DC_ISOLATED) {
      continue;
    }
    dc_non_slack.push_back(i);
  }

  const int np = static_cast<int>(non_slack.size());
  const int nq = static_cast<int>(pq.size());
  const int ndc_eq = static_cast<int>(dc_non_slack.size());
  const int nvar = np + nq + ndc_eq;

  std::vector<int> va_col(static_cast<size_t>(n), -1);
  std::vector<int> vm_col(static_cast<size_t>(n), -1);
  std::vector<int> vdc_col(static_cast<size_t>(ndc), -1);
  std::vector<int> p_row(static_cast<size_t>(n), -1);
  std::vector<int> q_row(static_cast<size_t>(n), -1);
  std::vector<int> dc_row(static_cast<size_t>(ndc), -1);

  for (int k = 0; k < np; ++k) {
    const int bus = non_slack[static_cast<size_t>(k)];
    va_col[static_cast<size_t>(bus)] = k;
    p_row[static_cast<size_t>(bus)] = k;
  }
  for (int k = 0; k < nq; ++k) {
    const int bus = pq[static_cast<size_t>(k)];
    const int idx = np + k;
    vm_col[static_cast<size_t>(bus)] = idx;
    q_row[static_cast<size_t>(bus)] = idx;
  }
  for (int k = 0; k < ndc_eq; ++k) {
    const int bus = dc_non_slack[static_cast<size_t>(k)];
    const int idx = np + nq + k;
    vdc_col[static_cast<size_t>(bus)] = idx;
    dc_row[static_cast<size_t>(bus)] = idx;
  }

  jac_ctx.n = n;
  jac_ctx.ndc = ndc;
  jac_ctx.np = np;
  jac_ctx.nq = nq;
  jac_ctx.ndc_eq = ndc_eq;
  jac_ctx.nvar = nvar;
  jac_ctx.non_slack = std::move(non_slack);
  jac_ctx.pq = std::move(pq);
  jac_ctx.dc_non_slack = std::move(dc_non_slack);
  jac_ctx.p_row = std::move(p_row);
  jac_ctx.q_row = std::move(q_row);
  jac_ctx.dc_row = std::move(dc_row);
  jac_ctx.va_col = std::move(va_col);
  jac_ctx.vm_col = std::move(vm_col);
  jac_ctx.vdc_col = std::move(vdc_col);

  // NCP bus data (Direction 3): record limits and setpoints for NCP-handled PV buses.
  if (data != nullptr && data->enable_semi_smooth_newton && !ncp_pv_buses.empty()) {
    // Build per-bus generator limits.
    for (int bus : ncp_pv_buses) {
      JacobianContext::NCPBusData nd;
      nd.bus = bus;
      nd.vm_set = ac_buses[static_cast<size_t>(bus)].vm_pu;
      nd.qmin = 0.0;
      nd.qmax = 0.0;
      bool has_gen = false;
      for (const auto& gen : data->generators) {
        if (!gen.in_service) continue;
        if (gen.bus - 1 != bus) continue;
        if (!has_gen) nd.vm_set = gen.vg_pu;
        has_gen = true;
        if (std::isfinite(gen.qmax_mvar)) {
          nd.qmax += gen.qmax_mvar / data->base_mva;
        } else {
          nd.qmax += 1e6;
        }
        if (std::isfinite(gen.qmin_mvar)) {
          nd.qmin += gen.qmin_mvar / data->base_mva;
        } else {
          nd.qmin -= 1e6;
        }
      }
      if (has_gen) {
        jac_ctx.ncp_buses.push_back(nd);
      }
    }
  }

  // Augmented equations (Direction 2): replace selected Q/DC rows with setpoint equations.
  if (data != nullptr && data->enable_augmented_equations && converters_ptr != nullptr) {
    for (const auto& conv : *converters_ptr) {
      if (!conv.in_service) continue;
      const int ac_bus = conv.bus_ac - 1;
      if (conv.control_mode == ConverterMode::VDC_VAC) {
        // Replace Q-row for ac_bus with: Vm[ac_bus] = v_ac_set_pu
        if (ac_bus >= 0 && ac_bus < n) {
          const int q_row_idx = jac_ctx.q_row[static_cast<size_t>(ac_bus)];
          if (q_row_idx >= 0) {
            jac_ctx.augmented_q_buses.push_back(ac_bus);
            jac_ctx.augmented_q_targets.push_back(conv.v_ac_set_pu);
          }
        }
      }
    }
    // Note: VDC_Q and DCDC voltage-mode DC augmented equations are intentionally
    // omitted. Their k_vdc / droop-based soft constraint converges well and
    // replacing the DC power balance with a hard Vdc setpoint can leave the
    // DC network's power balance underdetermined in some topologies.
  }

  return jac_ctx;
}

bool same_context_layout(const JacobianContext& a, const JacobianContext& b) {
  return a.n == b.n && a.ndc == b.ndc && a.np == b.np && a.nq == b.nq && a.ndc_eq == b.ndc_eq &&
         a.nvar == b.nvar && a.non_slack == b.non_slack && a.pq == b.pq &&
         a.dc_non_slack == b.dc_non_slack && a.p_row == b.p_row && a.q_row == b.q_row &&
         a.dc_row == b.dc_row && a.va_col == b.va_col && a.vm_col == b.vm_col &&
         a.vdc_col == b.vdc_col;
}

struct GeneratorLimitData {
  std::vector<double> qmax_pu;
  std::vector<double> qmin_pu;
  std::vector<double> vm_set_pu;
  std::vector<bool> has_generator;
  std::vector<bool> has_finite_q_limits;
};

GeneratorLimitData build_generator_limit_data(const SolverData& data,
                                              const std::vector<ACBus>& ac_buses) {
  const int n = static_cast<int>(ac_buses.size());
  GeneratorLimitData out;
  out.qmax_pu.assign(static_cast<size_t>(n), 0.0);
  out.qmin_pu.assign(static_cast<size_t>(n), 0.0);
  out.vm_set_pu.assign(static_cast<size_t>(n), 1.0);
  out.has_generator.assign(static_cast<size_t>(n), false);
  out.has_finite_q_limits.assign(static_cast<size_t>(n), true);

  std::vector<bool> vm_set_init(static_cast<size_t>(n), false);
  for (int i = 0; i < n; ++i) {
    out.vm_set_pu[static_cast<size_t>(i)] = ac_buses[static_cast<size_t>(i)].vm_pu;
  }

  for (const auto& gen : data.generators) {
    if (!gen.in_service) {
      continue;
    }
    const int bus = gen.bus - 1;
    if (bus < 0 || bus >= n) {
      continue;
    }
    out.has_generator[static_cast<size_t>(bus)] = true;
    if (!vm_set_init[static_cast<size_t>(bus)]) {
      out.vm_set_pu[static_cast<size_t>(bus)] = gen.vg_pu;
      vm_set_init[static_cast<size_t>(bus)] = true;
    }

    const bool qmax_finite = std::isfinite(gen.qmax_mvar);
    const bool qmin_finite = std::isfinite(gen.qmin_mvar);
    if (!qmax_finite || !qmin_finite) {
      out.has_finite_q_limits[static_cast<size_t>(bus)] = false;
      continue;
    }
    out.qmax_pu[static_cast<size_t>(bus)] += gen.qmax_mvar / data.base_mva;
    out.qmin_pu[static_cast<size_t>(bus)] += gen.qmin_mvar / data.base_mva;
  }

  for (int i = 0; i < n; ++i) {
    if (!out.has_generator[static_cast<size_t>(i)]) {
      out.has_finite_q_limits[static_cast<size_t>(i)] = false;
      out.qmax_pu[static_cast<size_t>(i)] = kInf;
      out.qmin_pu[static_cast<size_t>(i)] = -kInf;
      continue;
    }
    if (!out.has_finite_q_limits[static_cast<size_t>(i)]) {
      out.qmax_pu[static_cast<size_t>(i)] = kInf;
      out.qmin_pu[static_cast<size_t>(i)] = -kInf;
    }
  }

  return out;
}

bool apply_converter_mode_switching(std::vector<VSCConverter>& converters,
                                    const Eigen::VectorXd& vdc,
                                    const PowerFlowOptions& opt,
                                    std::vector<int>& high_count,
                                    std::vector<int>& low_count,
                                    SolverProfiling& profiling,
                                    const std::vector<char>& promotion_lock) {
  bool changed = false;
  if (static_cast<int>(high_count.size()) != static_cast<int>(converters.size())) {
    high_count.assign(converters.size(), 0);
    low_count.assign(converters.size(), 0);
  }

  const int hyster = std::max(1, opt.mode_hysteresis_iters);
  const double high = std::max(0.0, opt.converter_vdc_switch_high_pu);
  const double low = std::max(0.0, opt.converter_vdc_switch_low_pu);
  for (size_t idx = 0; idx < converters.size(); ++idx) {
    auto& conv = converters[idx];
    if (!conv.in_service) {
      high_count[idx] = 0;
      low_count[idx] = 0;
      continue;
    }

    const int dc_bus = conv.bus_dc - 1;
    if (dc_bus < 0 || dc_bus >= vdc.size()) {
      continue;
    }
    const double err = std::abs(vdc[dc_bus] - conv.v_dc_set_pu);
    if (conv.control_mode == ConverterMode::PQ_MODE) {
      if (err > high) {
        high_count[idx] += 1;
      } else {
        high_count[idx] = 0;
      }
      low_count[idx] = 0;
      if (high_count[idx] >= hyster) {
        conv.control_mode = ConverterMode::VDC_Q;
        high_count[idx] = 0;
        changed = true;
        profiling.converter_mode_switches += 1;
      }
    } else if (conv.control_mode == ConverterMode::VDC_Q) {
      // A converter promoted to hold an otherwise-reference-less DC island must
      // not be demoted back to PQ — doing so would leave the island singular.
      const bool locked = idx < promotion_lock.size() && promotion_lock[idx] != 0;
      if (err < low && !locked) {
        low_count[idx] += 1;
      } else {
        low_count[idx] = 0;
      }
      high_count[idx] = 0;
      if (low_count[idx] >= hyster) {
        conv.control_mode = ConverterMode::PQ_MODE;
        low_count[idx] = 0;
        changed = true;
        profiling.converter_mode_switches += 1;
      }
    } else {
      high_count[idx] = 0;
      low_count[idx] = 0;
    }
  }
  return changed;
}


}  // namespace

PowerFlowResult NewtonSolver::solve(const SolverData& data,
                                    const PowerFlowOptions& opt,
                                    const InitialState* init) const {
  using Clock = std::chrono::steady_clock;
  PowerFlowResult out;

  // The public facade keeps one NewtonSolver per thread. Reuse its state
  // workspace across ordinary solves, but use a local workspace for recursive
  // homotopy calls so an inner solve cannot overwrite the outer state.
  powerflow::SolverWorkspace nested_workspace;
  const bool owns_cached_workspace = !workspace_in_use_;
  powerflow::SolverWorkspace& workspace =
      owns_cached_workspace ? workspace_ : nested_workspace;
  if (owns_cached_workspace) workspace_in_use_ = true;
  struct WorkspaceGuard {
    bool& in_use;
    bool owns;
    ~WorkspaceGuard() { if (owns) in_use = false; }
  } workspace_guard{workspace_in_use_, owns_cached_workspace};

  std::vector<ACBus> ac_buses = data.ac_buses;
  std::vector<VSCConverter> converters = data.converters;
  const int n = static_cast<int>(ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());

  out.vm.assign(static_cast<size_t>(n), 1.0);
  out.va.assign(static_cast<size_t>(n), 0.0);
  out.vdc.assign(static_cast<size_t>(ndc), 1.0);
  out.converged = false;
  out.iterations = 0;
  out.residual = 0.0;
  out.profiling.ac_eval_threads = effective_ac_eval_threads(opt);

  if (n == 0 && ndc == 0) {
    out.converged = true;
    return out;
  }

  workspace.prepare_state(n, ndc);
  Eigen::VectorXd& vm = workspace.vm;
  Eigen::VectorXd& va = workspace.va;
  Eigen::VectorXd& vdc = workspace.vdc;
  Eigen::VectorXd pg_state = data.pg;
  Eigen::VectorXd qg_state = data.qg;

  if (init != nullptr) {
    if ((n > 0) &&
        (static_cast<int>(init->vm.size()) != n || static_cast<int>(init->va.size()) != n)) {
      throw std::invalid_argument("InitialState Vm/Va size does not match AC bus count.");
    }
    for (int i = 0; i < n; ++i) {
      if (!std::isfinite(init->vm[static_cast<size_t>(i)]) ||
          !std::isfinite(init->va[static_cast<size_t>(i)])) {
        throw std::invalid_argument(
            "InitialState Vm/Va contains NaN or Inf at bus index " + std::to_string(i));
      }
      vm[i] = init->vm[static_cast<size_t>(i)];
      va[i] = init->va[static_cast<size_t>(i)];
    }
  } else {
    for (int i = 0; i < n; ++i) {
      const ACBus& bus = ac_buses[static_cast<size_t>(i)];
      vm[i] = bus.vm_pu;
      va[i] = bus.va_deg * kPi / 180.0;
    }
  }

  powerflow::load_dc_initial_state(init, data.dc_buses, vdc);

  const int slack = first_slack_or_default(ac_buses);
  // Translate multi-source DC coordination metadata into the solve (multi-converter
  // model §6.6–6.7): participation factors reshape the group's droop sharing, and
  // a declared master is preferred when forming a reference (handled inside
  // plan_dc_island_references).
  apply_participation_factor_sharing(converters);
  // Plan DC island voltage references.  This may promote a PQ converter to VDC_Q
  // (mutating the local `converters` copy) so a reference-less island can balance;
  // such converters are locked against the adaptive switch demoting them back.
  DcSlackPlan dc_plan =
      plan_dc_island_references(data, converters, opt.enable_rigid_vdc_former);
  const std::vector<int>& dc_slacks = dc_plan.dc_slacks;
  // Rigid DC-slack formers hold their bus at the setpoint exactly: seed the
  // pinned bus voltage so the fixed-voltage slack carries v_set, not the bus's
  // nominal initial value.
  for (const auto& [bus_pos, v_set] : dc_plan.rigid_pins) {
    if (bus_pos >= 0 && bus_pos < ndc) vdc[bus_pos] = v_set;
  }
  std::vector<char> promotion_lock(converters.size(), 0);
  for (int idx : dc_plan.promoted_converter_idx) {
    if (idx >= 0 && idx < static_cast<int>(promotion_lock.size())) promotion_lock[idx] = 1;
  }
  // Rigid DC-slack formers are PQ converters whose injection was sized to balance
  // a pinned island; lock them too so the adaptive mode switch leaves them alone.
  for (int idx : dc_plan.rigid_converter_idx) {
    if (idx >= 0 && idx < static_cast<int>(promotion_lock.size())) promotion_lock[idx] = 1;
  }
  // Authored sole Vdc formers (e.g. a constant-Udc VSC station from a BZ card)
  // are locked for the same reason: demoting the island's only reference to PQ
  // leaves the Newton system singular and the solve blows up.
  for (int idx : dc_plan.sole_former_idx) {
    if (idx >= 0 && idx < static_cast<int>(promotion_lock.size())) promotion_lock[idx] = 1;
  }
  out.diagnostics.promoted_vsc_indices = dc_plan.promoted_converter_idx;
  for (auto& w : dc_plan.warnings) out.diagnostics.warnings.push_back(std::move(w));
  JacobianContext jac_ctx = build_jacobian_context(ac_buses, ndc, slack, dc_slacks, &data, &converters);
  jac_ctx.min_vm_pu = opt.robust_nonlinear.min_vm_pu;
  if (jac_ctx.nvar == 0) {
    out.converged = true;
    for (int i = 0; i < n; ++i) {
      out.vm[static_cast<size_t>(i)] = vm[i];
      out.va[static_cast<size_t>(i)] = va[i];
    }
    for (int i = 0; i < ndc; ++i) {
      out.vdc[static_cast<size_t>(i)] = vdc[i];
    }
    out.diagnostics.effective_converters = converters;
    return out;
  }

  std::vector<BusControlState> bus_control(static_cast<size_t>(n), BusControlState::FixedPQ);
  for (int i = 0; i < n; ++i) {
    if (i == slack) {
      continue;
    }
    const BusType bt = ac_buses[static_cast<size_t>(i)].bus_type;
    // SLACK buses (primary and secondary) are excluded from the Jacobian and
    // must not participate in PV→PQ switching.  Only genuine PV buses get PVActive.
    if (bt == BusType::PV) {
      bus_control[static_cast<size_t>(i)] = BusControlState::PVActive;
    }
  }
  const GeneratorLimitData gen_limits = build_generator_limit_data(data, ac_buses);

  // Note: We intentionally do NOT override bus.vm_pu with gen.vg_pu here.
  // For MATPOWER cases, the bus Vm column contains the actual converged
  // solution, which should be used as the initial guess.  The gen Vg column
  // specifies the voltage setpoint, which may differ slightly from the actual
  // voltage at the solution (due to reactive limits or other factors).
  // Overriding with Vg would destroy the solution-quality initial guess.

  std::vector<int> converter_high_count(converters.size(), 0);
  std::vector<int> converter_low_count(converters.size(), 0);

  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd p_spec = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd q_spec = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd pdc_linear = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_calc = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_spec = Eigen::VectorXd::Zero(ndc);
  workspace.prepare_equations(jac_ctx.np, jac_ctx.nq, jac_ctx.ndc_eq);
  Eigen::VectorXd& mismatch = workspace.residual;
  Eigen::VectorXd mismatch_scaled = Eigen::VectorXd::Zero(jac_ctx.nvar);

  Eigen::VectorXd& dx = workspace.dx;
  Eigen::VectorXd dx_scaled = Eigen::VectorXd::Zero(jac_ctx.nvar);
  Eigen::VectorXd vm_best = vm;
  Eigen::VectorXd va_best = va;
  Eigen::VectorXd vdc_best = vdc;
  Eigen::VectorXd vm_trial = vm;
  Eigen::VectorXd va_trial = va;
  Eigen::VectorXd vdc_trial = vdc;
  Eigen::VectorXd pcalc_trial = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qcalc_trial = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd p_spec_trial = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd q_spec_trial = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd pdc_linear_trial = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_calc_trial = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd pdc_spec_trial = Eigen::VectorXd::Zero(ndc);
  Eigen::VectorXd mismatch_trial = Eigen::VectorXd::Zero(jac_ctx.nvar);

  bool skip_vac_enforcement = data.enable_augmented_equations;

  // Nonmonotone line search state (Phase 2).
  const auto& ropts = opt.robust_nonlinear;
  powerflow::NonmonotoneLineSearch nm_linesearch(
      ropts.nonmonotone_window,
      ropts.armijo_c,
      ropts.line_search_beta);

  auto resize_for_context = [&]() {
    reset_or_resize(mismatch, jac_ctx.nvar);
    reset_or_resize(mismatch_scaled, jac_ctx.nvar);
    reset_or_resize(dx, jac_ctx.nvar);
    reset_or_resize(dx_scaled, jac_ctx.nvar);
    reset_or_resize(mismatch_trial, jac_ctx.nvar);
  };

  auto ensure_pattern = [&]() {
    const bool same_matrix_storage =
        cache_.ybus_value_ptr == data.ybus.valuePtr() &&
        cache_.ybus_outer_ptr == data.ybus.outerIndexPtr() &&
        cache_.ybus_inner_ptr == data.ybus.innerIndexPtr() &&
        cache_.gdc_value_ptr == data.gdc.valuePtr() &&
        cache_.gdc_outer_ptr == data.gdc.outerIndexPtr() &&
        cache_.gdc_inner_ptr == data.gdc.innerIndexPtr() &&
        cache_.ybus_rows == data.ybus.rows() && cache_.ybus_cols == data.ybus.cols() &&
        cache_.ybus_nnz == data.ybus.nonZeros() && cache_.gdc_rows == data.gdc.rows() &&
        cache_.gdc_cols == data.gdc.cols() && cache_.gdc_nnz == data.gdc.nonZeros();

    const bool reuse_cached_pattern =
        cache_.valid && cache_.solver && cache_.data_ptr == &data &&
        cache_.data_build_id == data.build_id &&
        same_matrix_storage && same_context_layout(cache_.ctx, jac_ctx);
    if (reuse_cached_pattern) {
      return;
    }
    cache_.valid = true;
    cache_.data_ptr = &data;
    cache_.data_build_id = data.build_id;
    cache_.ybus_value_ptr = data.ybus.valuePtr();
    cache_.ybus_outer_ptr = data.ybus.outerIndexPtr();
    cache_.ybus_inner_ptr = data.ybus.innerIndexPtr();
    cache_.gdc_value_ptr = data.gdc.valuePtr();
    cache_.gdc_outer_ptr = data.gdc.outerIndexPtr();
    cache_.gdc_inner_ptr = data.gdc.innerIndexPtr();
    cache_.ybus_rows = data.ybus.rows();
    cache_.ybus_cols = data.ybus.cols();
    cache_.ybus_nnz = data.ybus.nonZeros();
    cache_.gdc_rows = data.gdc.rows();
    cache_.gdc_cols = data.gdc.cols();
    cache_.gdc_nnz = data.gdc.nonZeros();
    cache_.ctx = jac_ctx;
    cache_.pattern = build_jacobian_pattern(data, cache_.ctx);
    cache_.solver = make_default_sparse_solver();
    cache_.solver->analyze_pattern(cache_.pattern.matrix);
    cache_.pattern.analyzed = true;
    out.profiling.jacobian_pattern_rebuilds += 1;
    out.profiling.jacobian_analyze_calls += 1;
  };

  auto evaluate_trial_state = [&](const Eigen::VectorXd& vm_state,
                                  const Eigen::VectorXd& va_state,
                                  const Eigen::VectorXd& vdc_state) -> double {
    return evaluate_residual_only(data,
                                  jac_ctx,
                                  ac_buses,
                                  converters,
                                  pg_state,
                                  qg_state,
                                  vm_state,
                                  va_state,
                                  vdc_state,
                                  pcalc_trial,
                                  qcalc_trial,
                                  p_spec_trial,
                                  q_spec_trial,
                                  pdc_linear_trial,
                                  pdc_calc_trial,
                                  pdc_spec_trial,
                                  mismatch_trial,
                                  cache_.pattern,
                                  out.profiling.ac_eval_threads);
  };

  auto run_line_search = [&](const Eigen::VectorXd& direction,
                             double base_resid,
                             int& eval_count,
                             double& elapsed_ms) -> bool {
    eval_count = 0;
    auto t0 = Clock::now();
    vm_best = vm;
    va_best = va;
    vdc_best = vdc;
    double best_resid = kInf;
    bool improved = false;
    double alpha = 1.0;
    const int ls_steps = std::max(1, opt.max_line_search_steps);

    // Nonmonotone reference: φ_k^max = max over window of ½‖F‖² values.
    // When nonmonotone LS is disabled, nm_ref == base_resid (monotone).
    const double nm_ref = ropts.enable_nonmonotone_linesearch
                              ? nm_linesearch.reference_merit()
                              : base_resid;
    // Use the actual nm_ref if it's a valid finite value, else fall back.
    const double reference = (std::isfinite(nm_ref) && nm_ref >= base_resid)
                                 ? nm_ref
                                 : base_resid;

    for (int ls = 0; ls < ls_steps; ++ls) {
      vm_trial = vm;
      va_trial = va;
      vdc_trial = vdc;

      for (int k = 0; k < jac_ctx.np; ++k) {
        const int bus = jac_ctx.non_slack[static_cast<size_t>(k)];
        va_trial[bus] += alpha * direction[k];
      }
      for (int k = 0; k < jac_ctx.nq; ++k) {
        const int bus = jac_ctx.pq[static_cast<size_t>(k)];
        vm_trial[bus] = std::max(vm_trial[bus] + alpha * direction[jac_ctx.np + k], kMinVm);
      }
      for (int k = 0; k < jac_ctx.ndc_eq; ++k) {
        const int bus = jac_ctx.dc_non_slack[static_cast<size_t>(k)];
        vdc_trial[bus] =
            std::max(vdc_trial[bus] + alpha * direction[jac_ctx.np + jac_ctx.nq + k], kMinVm);
      }

      if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm_trial);
      const double trial_resid = evaluate_trial_state(vm_trial, va_trial, vdc_trial);
      eval_count += 1;

      if (trial_resid < best_resid) {
        best_resid = trial_resid;
        vm_best = vm_trial;
        va_best = va_trial;
        vdc_best = vdc_trial;
        improved = (trial_resid < base_resid);
      }
      if (trial_resid < base_resid) {
        break;
      }
      alpha *= 0.5;
    }

    auto t1 = Clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0)
                     .count();
    double max_ratio = 1.01;
    if (base_resid > 10.0) {
      max_ratio = 2.0;
    } else if (base_resid > 1.0) {
      max_ratio = 1.5;
    } else if (base_resid > 0.1) {
      max_ratio = 1.2;
    }
    if (!improved && std::isfinite(best_resid) && best_resid <= base_resid * max_ratio) {
      improved = true;
    }
    // Nonmonotone acceptance: if still not improved but best_resid is within the
    // nonmonotone reference window, accept the step (GLL condition post-hoc).
    if (!improved && ropts.enable_nonmonotone_linesearch &&
        std::isfinite(best_resid) && std::isfinite(reference) &&
        best_resid <= reference * (1.0 + ropts.armijo_c)) {
      improved = true;
    }
    return improved;
  };

  auto solve_linear = [&](const Eigen::SparseMatrix<double>& jac,
                          const Eigen::VectorXd& rhs,
                          Eigen::VectorXd& direction,
                          double& elapsed_ms) -> bool {
    auto t0 = Clock::now();
    SparseLinearSolver* sparse_solver = cache_.solver.get();
    if (sparse_solver == nullptr) {
      return false;
    }
    out.profiling.factorization_calls += 1;
    if (!sparse_solver->factorize(jac)) {
      out.profiling.linear_solver_status = "factorization_failed";
      auto t1 = Clock::now();
      elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0)
                       .count();
      return false;
    }
    out.profiling.linear_solve_calls += 1;
    const bool ok = sparse_solver->solve(rhs, direction) && direction.allFinite();
    out.profiling.linear_solver_status = ok ? "ok" : "solve_failed";
    auto t1 = Clock::now();
    elapsed_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
    return ok;
  };

  // ── Phase 5: NK-GMRES fallback ────────────────────────────────────
  // Mirrors solve_linear but uses GMRES instead of sparse LU.  Called from
  // the inner Newton loop when the condition proxy exceeds nk_condition_trigger
  // AND ropts.enable_newton_krylov_fallback is true.
  auto solve_linear_nk = [&](const Eigen::SparseMatrix<double>& jac,
                              const Eigen::VectorXd& rhs,
                              Eigen::VectorXd& direction,
                              double& elapsed_ms) -> bool {
    auto t0 = Clock::now();
    const int restart = std::max(1, ropts.gmres_restart);
    const int max_outer = std::max(1, ropts.gmres_max_outer);
    const double tol = std::max(1e-15, ropts.gmres_tol);

    direction.setZero();
    powerflow::NKLinearResult nk = powerflow::newton_krylov_step(
        jac, rhs, jac_ctx,
        ropts.enable_schur_preconditioner,
        restart, max_outer, tol);

    auto t1 = Clock::now();
    elapsed_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

    if (nk.success && nk.step.allFinite()) {
      direction = nk.step;
      out.profiling.linear_solver_status =
          nk.used_schur ? "nk_gmres_schur" : "nk_gmres";
      out.profiling.linear_solve_calls += 1;
      return true;
    }
    out.profiling.linear_solver_status = "nk_gmres_failed";
    return false;
  };

  // Outer loop (standard MATPOWER approach):
  //   1. Run Newton to convergence.
  //   2. Check Q limits on PV buses — switch violated ones to PQ.
  //   3. Re-run Newton from converged state.
  //   4. After all PV buses are within limits, try PQ→PV restoration.
  //   5. Repeat until no further switching is needed.
  constexpr int kMaxOuterLoops = 30;
  int total_iters = 0;

  // Helper lambda: enforce Q limits on PV buses.
  // allow_restore=false: only PV→PQ switching (safe during iterations).
  // allow_restore=true: also PQ→PV restoration (post-convergence only).
  auto check_q_limits_and_switch = [&](bool allow_restore) -> bool {
    bool any_switched = false;
    const double q_hys = std::max(0.0, opt.pv_q_hysteresis_pu);
    const double vm_tol = std::max(0.0, opt.pv_recover_vm_tol_pu);
    for (int i = 0; i < n; ++i) {
      if (i == slack) continue;
      // Additional SLACK buses are excluded from the Jacobian and must not be switched.
      if (ac_buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) continue;
      if (!gen_limits.has_generator[static_cast<size_t>(i)] ||
          !gen_limits.has_finite_q_limits[static_cast<size_t>(i)]) continue;

      const double qmax = gen_limits.qmax_pu[static_cast<size_t>(i)];
      const double qmin = gen_limits.qmin_pu[static_cast<size_t>(i)];
      const double qload_pu = ac_buses[static_cast<size_t>(i)].qd_mvar / data.base_mva;
      const double qconv = q_spec[i] - (qg_state[i] - qload_pu);
      const double qg_implied = qcalc[i] + qload_pu - qconv;

      BusControlState& mode = bus_control[static_cast<size_t>(i)];
      if (mode == BusControlState::PVActive) {
        if (qg_implied > qmax + q_hys) {
          ac_buses[static_cast<size_t>(i)].bus_type = BusType::PQ;
          qg_state[i] = qmax;
          mode = BusControlState::PQLimited;
          any_switched = true;
          out.profiling.pv_to_pq_switches += 1;
        } else if (qg_implied < qmin - q_hys) {
          ac_buses[static_cast<size_t>(i)].bus_type = BusType::PQ;
          qg_state[i] = qmin;
          mode = BusControlState::PQLimited;
          any_switched = true;
          out.profiling.pv_to_pq_switches += 1;
        }
      } else if (allow_restore && mode == BusControlState::PQLimited) {
        const double vm_set = gen_limits.vm_set_pu[static_cast<size_t>(i)];
        if (qg_implied > qmin + q_hys && qg_implied < qmax - q_hys &&
            std::abs(vm[i] - vm_set) < vm_tol) {
          ac_buses[static_cast<size_t>(i)].bus_type = BusType::PV;
          vm[i] = vm_set;
          qg_state[i] = qg_implied;
          mode = BusControlState::PVActive;
          any_switched = true;
          out.profiling.pq_to_pv_switches += 1;
        }
      }
    }
    return any_switched;
  };

  // Globalization state (Direction 4).
  using GS = PowerFlowOptions::GlobalizationStrategy;
  double tr_delta = opt.trust_region_radius0;
  double ptc_delta_t = opt.ptc_delta0;
  double prev_resid = kInf;
  double resid_ncp_initial = -1.0;   // first residual seen with NCP active (Gap 2)

  for (int outer = 0; outer < kMaxOuterLoops; ++outer) {
    // Rebuild Jacobian context for current bus types.
    jac_ctx = build_jacobian_context(ac_buses, ndc, slack, dc_slacks, &data, &converters);
    resize_for_context();
    ensure_pattern();

    // Pre-solve structural closure screen (multi-converter model §7.4): run once
    // on the initial assembled pattern.  n_equations == n_variables holds by
    // construction; the scan additionally rejects all-zero rows/columns.
    if (outer == 0 && !out.diagnostics.equation_closure_checked) {
      const StructuralScan sc = scan_empty_rows_cols(cache_.pattern.matrix);
      out.diagnostics.equation_closure_checked = true;
      out.diagnostics.n_variables = jac_ctx.nvar;
      out.diagnostics.n_equations = jac_ctx.nvar;
      out.diagnostics.empty_jacobian_rows = sc.empty_rows;
      out.diagnostics.empty_jacobian_cols = sc.empty_cols;
      out.diagnostics.equation_closure_ok = (sc.empty_rows == 0 && sc.empty_cols == 0);
      if (!out.diagnostics.equation_closure_ok) {
        out.diagnostics.warnings.push_back(
            "[PF-JAC-STRUCT-01] Jacobian pattern has " +
            std::to_string(sc.empty_rows) + " empty row(s) and " +
            std::to_string(sc.empty_cols) +
            " empty column(s); the system is structurally rank-deficient "
            "(a DC or AC variable may lack a constraining equation or reference).");
      }
    }

    // Inner Newton loop — solve with fixed bus types.
    bool inner_converged = false;
    bool inner_failed = false;
    int inner_iters = 0;
    for (; inner_iters < opt.max_iter; ++inner_iters) {
      if (!skip_vac_enforcement) {
        enforce_vac_setpoints(converters, vm);
      }

      SparseLinearSolver* sparse_solver = cache_.solver.get();
      if (sparse_solver == nullptr) {
        inner_failed = true;
        break;
      }
      out.profiling.linear_solver_backend = sparse_solver->backend_name();

      auto eval_t0 = Clock::now();
      const double resid = evaluate_residual_and_jacobian(data,
                                                          jac_ctx,
                                                          ac_buses,
                                                          converters,
                                                          pg_state,
                                                          qg_state,
                                                          vm,
                                                          va,
                                                          vdc,
                                                          pcalc,
                                                          qcalc,
                                                          p_spec,
                                                          q_spec,
                                                          pdc_linear,
                                                          pdc_calc,
                                                          pdc_spec,
                                                          mismatch,
                                                          cache_.pattern,
                                                          out.profiling.ac_eval_threads);
      auto eval_t1 = Clock::now();
      const double eval_ms =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(eval_t1 - eval_t0)
              .count();
      out.profiling.eval_jacobian_ms_total += eval_ms;

      out.iterations = total_iters + inner_iters + 1;
      out.residual = resid;

      const auto scaling = powerflow::build_nonlinear_scaling(jac_ctx,
                                                               data,
                                                               p_spec,
                                                               q_spec,
                                                               pdc_spec,
                                                               vm,
                                                               vdc,
                                                               opt.robust_nonlinear);
      mismatch_scaled = scaling.apply_residual_scaling(mismatch);
      const double scaled_resid = (mismatch_scaled.size() == 0)
                                      ? 0.0
                                      : mismatch_scaled.cwiseAbs().maxCoeff();

      out.profiling.raw_residual_norm = resid;
      out.profiling.scaled_residual_norm = scaled_resid;

      const bool use_scaled_convergence = opt.robust_nonlinear.enable_residual_scaling;
      const double convergence_resid = use_scaled_convergence ? scaled_resid : resid;

      const bool use_scaled_linear_system =
          opt.robust_nonlinear.enable_jacobian_row_col_equilibration &&
          (opt.robust_nonlinear.enable_residual_scaling ||
           opt.robust_nonlinear.enable_variable_scaling);
      const Eigen::SparseMatrix<double> jac_for_linear =
          use_scaled_linear_system ? scaling.apply_jacobian_scaling(cache_.pattern.matrix)
                                   : cache_.pattern.matrix;
      const Eigen::VectorXd& rhs_for_linear =
          use_scaled_linear_system ? mismatch_scaled : mismatch;

      if (opt.robust_nonlinear.enable_condition_monitor ||
          ropts.enable_newton_krylov_fallback) {
        out.profiling.condition_estimate = estimate_condition_proxy(jac_for_linear);
      }

      if (opt.enable_solver_profiling) {
        out.profiling.residual_by_iter.push_back(resid);
        out.profiling.eval_jacobian_ms_by_iter.push_back(eval_ms);
      }

      // NaN/Inf guard: if residual is non-finite, the solve has diverged
      // and continuing would corrupt state.  Bail out immediately.
      if (!std::isfinite(resid)) {
        inner_failed = true;
        break;
      }

      if (opt.enable_converter_mode_switching && resid < 1e-6 &&
          apply_converter_mode_switching(converters,
                                         vdc,
                                         opt,
                                         converter_high_count,
                                         converter_low_count,
                                         out.profiling,
                                         promotion_lock)) {
        if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm);
        continue;
      }

      if (convergence_resid < opt.tol) {
        inner_converged = true;
        break;
      }

      // ── Phase 2: Push merit into nonmonotone LS window ──────────────
      nm_linesearch.push_merit(resid);

      // ── Phase 2: Anneal smooth-NCP μ when semi-smooth Newton is active ──
      if (data.enable_semi_smooth_newton && ropts.enable_smooth_ncp) {
        auto& mut_data = const_cast<powerflow::SolverData&>(data);
        if (mut_data.ncp_mu <= 0.0) {
          mut_data.ncp_mu = ropts.ncp_mu0;
          resid_ncp_initial = resid;  // record baseline for two-phase schedule
        } else if (resid < prev_resid * 0.9) {
          // Two-phase annealing (Gap 2): use a slower factor during the coarse
          // phase (active-set not yet determined) to prevent μ from dropping
          // too fast near strongly active Q limits, which causes oscillation.
          const double mu_factor =
              (resid_ncp_initial > 0.0 &&
               resid > resid_ncp_initial * ropts.ncp_mu_phase_transition)
                  ? ropts.ncp_mu_factor_coarse   // coarse phase: slow annealing
                  : ropts.ncp_mu_factor;          // fine phase:  fast annealing
          mut_data.ncp_mu =
              std::max(mut_data.ncp_mu * mu_factor, ropts.ncp_mu_min);
        }
      }

      bool accepted_step = false;
      int ls_evals_this_iter = 0;
      double ls_ms_this_iter = 0.0;
      double linear_ms_this_iter = 0.0;

      // ── Phase 5: decide whether to use NK-GMRES this iteration ──────
      // NK is triggered whenever the condition proxy exceeds the configured
      // threshold AND enable_newton_krylov_fallback is true.  All three
      // globalization branches can now benefit from NK: the default branch
      // substitutes NK for the Newton direction; the TR branch uses NK to
      // compute the Newton step in the dogleg computation; the PTC branch
      // applies NK to the pseudo-transient shifted Jacobian.
      const bool use_nk_this_iter =
          ropts.enable_newton_krylov_fallback &&
          out.profiling.condition_estimate > ropts.nk_condition_trigger;

      // ── Globalization strategy dispatch (Direction 4) ───────────────
      if (opt.globalization == GS::TrustRegion) {
        // Trust-region Newton with dogleg step.
        // Phase 5: use NK-GMRES to compute the Newton direction when the
        // condition proxy is high.  The dogleg then blends this direction
        // with the Cauchy step as usual.
        const bool tr_step_ok =
            use_nk_this_iter
                ? solve_linear_nk(jac_for_linear, rhs_for_linear, dx_scaled, linear_ms_this_iter)
                : solve_linear(jac_for_linear, rhs_for_linear, dx_scaled, linear_ms_this_iter);
        if (tr_step_ok) {
          dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
          clip_step(dx, jac_ctx, opt);
          // Compute gradient g = Jᵀ·F  and J·g for Cauchy step.
          Eigen::VectorXd neg_mismatch = -rhs_for_linear;
          Eigen::VectorXd gradient = jac_for_linear.transpose() * neg_mismatch;
          Eigen::VectorXd jg = jac_for_linear * gradient;
          Eigen::VectorXd step = dogleg_step(dx, gradient, jg, tr_delta);

          // Apply trial step.
          vm_trial = vm; va_trial = va; vdc_trial = vdc;
          for (int k = 0; k < jac_ctx.np; ++k) {
            va_trial[jac_ctx.non_slack[static_cast<size_t>(k)]] += step[k];
          }
          for (int k = 0; k < jac_ctx.nq; ++k) {
            const int bus = jac_ctx.pq[static_cast<size_t>(k)];
            vm_trial[bus] = std::max(vm_trial[bus] + step[jac_ctx.np + k], kMinVm);
          }
          for (int k = 0; k < jac_ctx.ndc_eq; ++k) {
            const int bus = jac_ctx.dc_non_slack[static_cast<size_t>(k)];
            vdc_trial[bus] = std::max(vdc_trial[bus] + step[jac_ctx.np + jac_ctx.nq + k], kMinVm);
          }
          if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm_trial);
          const double trial_resid = evaluate_trial_state(vm_trial, va_trial, vdc_trial);
          ls_evals_this_iter += 1;

          const double f_sq = resid * resid;
          const double f_new_sq = trial_resid * trial_resid;
          Eigen::VectorXd model_step = neg_mismatch + cache_.pattern.matrix * step;
          const double pred_red = f_sq - model_step.squaredNorm();

          accepted_step = trust_region_update(f_sq, f_new_sq, pred_red,
                                               tr_delta, step.norm(), opt.trust_region_max);
          if (accepted_step) {
            vm_best = vm_trial; va_best = va_trial; vdc_best = vdc_trial;
          }
        }
      } else if (opt.globalization == GS::PseudoTransient) {
        // Pseudo-transient continuation.
        // Phase 5: use NK-GMRES on the shifted Jacobian (J + I/δt) when the
        // condition proxy is high.  The Schur-complement preconditioner for
        // J + (1/δt)·I has the same block structure as for J alone, so the
        // existing SchurBlockPreconditioner works directly.
        Eigen::SparseMatrix<double> ptc_jac = jac_for_linear;
        ptc_jac.diagonal().array() += 1.0 / ptc_delta_t;
        const bool ptc_step_ok =
            use_nk_this_iter
                ? solve_linear_nk(ptc_jac, rhs_for_linear, dx_scaled, linear_ms_this_iter)
                : solve_linear(ptc_jac, rhs_for_linear, dx_scaled, linear_ms_this_iter);
        if (ptc_step_ok) {
          dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
          clip_step(dx, jac_ctx, opt);
          // PTC: accept step unconditionally.
          vm_best = vm; va_best = va; vdc_best = vdc;
          for (int k = 0; k < jac_ctx.np; ++k) {
            va_best[jac_ctx.non_slack[static_cast<size_t>(k)]] += dx[k];
          }
          for (int k = 0; k < jac_ctx.nq; ++k) {
            const int bus = jac_ctx.pq[static_cast<size_t>(k)];
            vm_best[bus] = std::max(vm_best[bus] + dx[jac_ctx.np + k], kMinVm);
          }
          for (int k = 0; k < jac_ctx.ndc_eq; ++k) {
            const int bus = jac_ctx.dc_non_slack[static_cast<size_t>(k)];
            vdc_best[bus] = std::max(vdc_best[bus] + dx[jac_ctx.np + jac_ctx.nq + k], kMinVm);
          }
          if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm_best);
          accepted_step = true;
        }
        // Update pseudo-timestep using SER formula.
        ptc_update_timestep(prev_resid, resid, ptc_delta_t, opt.ptc_growth);
      } else {
        // Default: line search (existing code).
        // Phase 5: substitute NK-GMRES when condition is flagged bad.
        if (use_nk_this_iter) {
          if (solve_linear_nk(jac_for_linear, rhs_for_linear, dx_scaled, linear_ms_this_iter)) {
            dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
            clip_step(dx, jac_ctx, opt);
            accepted_step = run_line_search(dx, resid, ls_evals_this_iter, ls_ms_this_iter);
          }
          // If NK step failed or didn't improve, fall through to standard LU below.
          if (!accepted_step) {
            if (solve_linear(jac_for_linear, rhs_for_linear, dx_scaled, linear_ms_this_iter)) {
              dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
              accepted_step = run_line_search(dx, resid, ls_evals_this_iter, ls_ms_this_iter);
            }
          }
        } else {
          if (solve_linear(jac_for_linear, rhs_for_linear, dx_scaled, linear_ms_this_iter)) {
            dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
            accepted_step = run_line_search(dx, resid, ls_evals_this_iter, ls_ms_this_iter);
          }
        }

        if (!accepted_step) {
          const int reg_steps = std::max(0, opt.max_regularization_steps);
          double lambda = std::max(1e-16, opt.regularization_lambda0);
          const double growth = std::max(1.0, opt.regularization_growth);
          for (int r = 0; r < reg_steps; ++r) {
            Eigen::SparseMatrix<double> reg_jac = jac_for_linear;
            reg_jac.diagonal().array() += lambda;
            out.profiling.regularization_attempts += 1;
            out.profiling.regularization_count += 1;

            if (solve_linear(reg_jac, rhs_for_linear, dx_scaled, linear_ms_this_iter)) {
              dx = use_scaled_linear_system ? scaling.unscale_step(dx_scaled) : dx_scaled;
              clip_step(dx, jac_ctx, opt);
              if (run_line_search(dx, resid, ls_evals_this_iter, ls_ms_this_iter)) {
                accepted_step = true;
                break;
              }
            }
            lambda *= growth;
          }
        }

        if (!accepted_step && dx.allFinite() && dx.cwiseAbs().maxCoeff() > 0.0 && resid > 0.1) {
          vm_trial = vm; va_trial = va; vdc_trial = vdc;
          constexpr double alpha_escape = 0.5;
          for (int k = 0; k < jac_ctx.np; ++k) {
            va_trial[jac_ctx.non_slack[static_cast<size_t>(k)]] += alpha_escape * dx[k];
          }
          for (int k = 0; k < jac_ctx.nq; ++k) {
            const int bus = jac_ctx.pq[static_cast<size_t>(k)];
            vm_trial[bus] = std::max(vm_trial[bus] + alpha_escape * dx[jac_ctx.np + k], kMinVm);
          }
          for (int k = 0; k < jac_ctx.ndc_eq; ++k) {
            const int bus = jac_ctx.dc_non_slack[static_cast<size_t>(k)];
            vdc_trial[bus] =
                std::max(vdc_trial[bus] + alpha_escape * dx[jac_ctx.np + jac_ctx.nq + k], kMinVm);
          }
          if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm_trial);
          const double escape_resid = evaluate_trial_state(vm_trial, va_trial, vdc_trial);
          ls_evals_this_iter += 1;
          if (std::isfinite(escape_resid) && escape_resid < resid * 2.0) {
            vm_best = vm_trial; va_best = va_trial; vdc_best = vdc_trial;
            accepted_step = true;
            out.profiling.rejected_steps += 1;
          }
        }
      }  // end globalization dispatch

      // ── Gap 3: Auto-schedule Phase 3/4 in-loop recovery ───────────────
      // When the default line-search branch exhausted all options (regulari-
      // sation + escape move) without accepting a step, automatically try a
      // Levenberg-Marquardt recovery step and then a pseudo-transient step
      // before declaring inner_failed.  This integrates Phase 3 fallbacks
      // into the main loop without requiring opt.globalization to be set
      // explicitly to TrustRegion or PseudoTransient.
      if (!accepted_step && opt.globalization == GS::LineSearch &&
          ropts.enable_auto_fallback_scheduling) {
        // ── LM recovery step ────────────────────────────────────────────
        if (!accepted_step && ropts.enable_lm_trust_region_fallback) {
          double lm_lambda = ropts.lm_lambda0;
          for (int lm_attempt = 0; lm_attempt < 4 && !accepted_step; ++lm_attempt) {
            // Form JᵀJ + λI (normal equations with Tikhonov regularisation).
            Eigen::SparseMatrix<double> JtJ =
                (jac_for_linear.transpose() * jac_for_linear).pruned(0.0);
            JtJ.diagonal().array() += lm_lambda;
            // RHS = Jᵀ·F (the negative gradient of ½‖F‖²).
            const Eigen::VectorXd lm_rhs =
                -(jac_for_linear.transpose() * rhs_for_linear);
            Eigen::VectorXd lm_dx;
            if (cache_.solver->factorize(JtJ) &&
                cache_.solver->solve(lm_rhs, lm_dx) && lm_dx.allFinite()) {
              if (use_scaled_linear_system) {
                lm_dx = scaling.unscale_step(lm_dx);
              }
              clip_step(lm_dx, jac_ctx, opt);
              if (run_line_search(lm_dx, resid, ls_evals_this_iter, ls_ms_this_iter)) {
                dx = lm_dx;
                accepted_step = true;
                out.profiling.linear_solver_status = "lm_recovery";
              }
            }
            lm_lambda = std::min(lm_lambda * 10.0, ropts.lm_lambda_max);
          }
        }
        // ── PTC recovery step ────────────────────────────────────────────
        if (!accepted_step && ropts.enable_ptc_ser) {
          Eigen::SparseMatrix<double> ptc_jac_rec = jac_for_linear;
          ptc_jac_rec.diagonal().array() += 1.0 / std::max(ptc_delta_t, 1e-8);
          Eigen::VectorXd ptc_dx;
          if (solve_linear(ptc_jac_rec, rhs_for_linear, ptc_dx, linear_ms_this_iter) &&
              ptc_dx.allFinite()) {
            if (use_scaled_linear_system) {
              ptc_dx = scaling.unscale_step(ptc_dx);
            }
            clip_step(ptc_dx, jac_ctx, opt);
            // PTC step: accept unconditionally (same as the PTC globalization branch).
            dx = ptc_dx;
            vm_best = vm; va_best = va; vdc_best = vdc;
            for (int k = 0; k < jac_ctx.np; ++k) {
              va_best[jac_ctx.non_slack[static_cast<size_t>(k)]] += dx[k];
            }
            for (int k = 0; k < jac_ctx.nq; ++k) {
              const int bus = jac_ctx.pq[static_cast<size_t>(k)];
              vm_best[bus] = std::max(vm_best[bus] + dx[jac_ctx.np + k], kMinVm);
            }
            for (int k = 0; k < jac_ctx.ndc_eq; ++k) {
              const int bus = jac_ctx.dc_non_slack[static_cast<size_t>(k)];
              vdc_best[bus] = std::max(vdc_best[bus] + dx[jac_ctx.np + jac_ctx.nq + k], kMinVm);
            }
            if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm_best);
            accepted_step = true;
            out.profiling.linear_solver_status = "ptc_recovery";
          }
        }
      }

      prev_resid = resid;

      out.profiling.linear_solve_ms_total += linear_ms_this_iter;
      out.profiling.line_search_ms_total += ls_ms_this_iter;
      out.profiling.line_search_evaluations += ls_evals_this_iter;
      if (opt.enable_solver_profiling) {
        out.profiling.linear_solve_ms_by_iter.push_back(linear_ms_this_iter);
        out.profiling.line_search_ms_by_iter.push_back(ls_ms_this_iter);
        out.profiling.line_search_evals_by_iter.push_back(ls_evals_this_iter);
      }

      if (!accepted_step) {
        out.profiling.rejected_steps += 1;
        inner_failed = true;
        break;
      }

      vm = vm_best;
      va = va_best;
      vdc = vdc_best;

      // Per-iteration PV→PQ enforcement: only when residual is small
      // enough that Q estimates are reliable.  At large residuals the
      // intermediate Q values are inaccurate and premature switching
      // destroys the convergence basin (observed on case118 basin tests).
      // The outer loop handles post-convergence Q-limit enforcement
      // unconditionally, so skipping here is safe.
      if (opt.enable_pv_pq_conversion && !data.enable_semi_smooth_newton && resid < 1e-3) {
        if (check_q_limits_and_switch(/*allow_restore=*/false)) {
          jac_ctx = build_jacobian_context(ac_buses, ndc, slack, dc_slacks, &data, &converters);
          resize_for_context();
          ensure_pattern();
        }
      }
    }  // end inner Newton loop

    total_iters += inner_iters;
    out.converged = inner_converged;
    if (inner_failed || !opt.enable_pv_pq_conversion || data.enable_semi_smooth_newton) {
      break;
    }

    if (!inner_converged) {
      // Newton failed to converge — try PV→PQ switching on violated buses
      // and re-run from current state, which may help convergence.
      // Guard: only switch when the residual is small enough that Q
      // estimates are meaningful.  At large residuals, switching based on
      // inaccurate Q values creates PV↔PQ oscillation across outer loops
      // without ever converging (observed as 48–264 spurious switches on
      // case118 basin tests).
      if (out.residual > 0.1 ||
          !check_q_limits_and_switch(/*allow_restore=*/false)) {
        break;  // No reliable switches possible — give up.
      }
      continue;  // Re-run Newton with updated bus types.
    }

    // Post-convergence: enforce Q limits on PV buses.
    // Switch violated PV buses to PQ and re-run Newton.
    if (check_q_limits_and_switch(/*allow_restore=*/false)) {
      continue;  // Re-run Newton with PV→PQ switches applied.
    }

    // All PV buses within limits. Try restoring PQ-limited buses to PV.
    if (!check_q_limits_and_switch(/*allow_restore=*/true)) {
      break;  // No more switching needed — final solution found.
    }
    // Some buses restored PQ→PV — re-run Newton to verify.
  }  // end outer PV/PQ loop

  if (!skip_vac_enforcement) enforce_vac_setpoints(converters, vm);
  out.residual = evaluate_residual_only(data,
                                        jac_ctx,
                                        ac_buses,
                                        converters,
                                        pg_state,
                                        qg_state,
                                        vm,
                                        va,
                                        vdc,
                                        pcalc,
                                        qcalc,
                                        p_spec,
                                        q_spec,
                                        pdc_linear,
                                        pdc_calc,
                                        pdc_spec,
                                        mismatch,
                                        cache_.pattern,
                                        out.profiling.ac_eval_threads);
  out.profiling.raw_residual_norm = out.residual;
  {
    const auto scaling = powerflow::build_nonlinear_scaling(jac_ctx,
                                                             data,
                                                             p_spec,
                                                             q_spec,
                                                             pdc_spec,
                                                             vm,
                                                             vdc,
                                                             opt.robust_nonlinear);
    const Eigen::VectorXd final_scaled = scaling.apply_residual_scaling(mismatch);
    out.profiling.scaled_residual_norm =
        (final_scaled.size() == 0) ? 0.0 : final_scaled.cwiseAbs().maxCoeff();
  }
  // Always set converged based on final residual. This ensures that if bus-type
  // switching (PV→PQ or PQ→PV) resulted in a higher final residual than what
  // was achieved during an earlier inner Newton loop, convergence is correctly
  // reported as false.
  out.converged = ((opt.robust_nonlinear.enable_residual_scaling
                        ? out.profiling.scaled_residual_norm
                        : out.residual) < opt.tol);

  for (int i = 0; i < n; ++i) {
    out.vm[static_cast<size_t>(i)] = vm[i];
    out.va[static_cast<size_t>(i)] = va[i];
  }
  for (int i = 0; i < ndc; ++i) {
    out.vdc[static_cast<size_t>(i)] = vdc[i];
  }

  // Post-solve DC voltage-limit check.  A weak converter droop (small k_vdc) can
  // force Vdc far from setpoint to balance an island, exceeding the bus limits;
  // warn so the user can stiffen the droop or add a true DC_V reference.
  for (int i = 0; i < ndc; ++i) {
    const auto& b = data.dc_buses[static_cast<size_t>(i)];
    if (b.bus_type == DCBusType::DC_ISOLATED) continue;
    const double v = out.vdc[static_cast<size_t>(i)];
    if (v > b.vmax_pu + 1e-6) {
      out.diagnostics.warnings.push_back(
          "DC bus " + std::to_string(b.index) + " Vdc=" + std::to_string(v) +
          " pu exceeds vmax=" + std::to_string(b.vmax_pu) +
          " pu (weak converter droop; consider a larger k_vdc or a DC_V reference).");
    } else if (v < b.vmin_pu - 1e-6) {
      out.diagnostics.warnings.push_back(
          "DC bus " + std::to_string(b.index) + " Vdc=" + std::to_string(v) +
          " pu is below vmin=" + std::to_string(b.vmin_pu) +
          " pu (weak converter droop; consider a larger k_vdc or a DC_V reference).");
    }
  }

  // Surface the final converter list (post auto-promotion / stiff Vdc forming /
  // in-iteration mode switching) so result reconstruction reports converter
  // powers consistent with the solved network state.
  out.diagnostics.effective_converters = converters;

  return out;
}

}  // namespace hacdcpf::engine
