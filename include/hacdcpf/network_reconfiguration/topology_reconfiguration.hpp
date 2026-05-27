#pragma once
// =====================================================================
// topology_reconfiguration.hpp — 拓扑重构（Network Reconfiguration after Fault）
//
// 当配电网发生线路故障时，通过 MILP 优化确定最优开关操作方案：
//   1. 断开故障线路（可能导致部分节点失电）
//   2. 在满足辐射状约束的前提下闭合联络开关，形成新的供电路径
//
// MILP 模型变量（对应 Julia TopologyAnalysis 实现）：
//   虚拟潮流: Fij (nl), Fij_vsc (nl_vsc), Fg (ng)  — 拓扑连通性
//   开关状态: β (nl+nl_vsc) ∈{0,1}                  — 故障后线路通断
//   根节点:   γ (ng) ∈{0,1}                          — 发电机根节点指示
//   LinDistFlow: Pij, Qij, Pg, Qg, v, sP, sQ        — 可选潮流约束
//
// 目标函数:
//   min λ_sw·开关操作次数 + λ_loss·Σ(R_i·β_i)
//     + λ_shed·Σ(sP+sQ) + λ_island·Σ(γ[2:end])
// =====================================================================

#include <string>
#include <vector>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"

namespace hacdcpf::analysis {

/// Unambiguous branch reference for hybrid AC/DC systems.
/// Holds both the edge category and the component-model index (.index field),
/// replacing bare int IDs which conflate AC, DC, and VSC indices in the same
/// numeric space.
struct BranchRef {
  graph::EdgeCategory category{graph::EdgeCategory::AC_Line}; ///< AC_Line, DC_Line, or VSC_Coupling
  int                 index{-1};                              ///< Component .index field
};

// ── 拓扑重构选项 ───────────────────────────────────────────────────────
struct TopoReconfOptions {
  /// 故障线路编号列表（每个元素为 ACBranch::index 字段值，非数组下标）
  /// 这些线路将被强制断开，然后通过闭合联络开关恢复供电
  std::vector<int> line_failures;

  /// Structured faulted branch list. Use this for hybrid systems where AC,
  /// DC, and VSC components may share the same numeric .index value.
  std::vector<BranchRef> faulted_branches;

  /// Structured switchable branch list. Use this for hybrid systems where
  /// AC, DC, and VSC components may share the same numeric .index value.
  /// Empty = automatic tie-switch detection.
  std::vector<BranchRef> switchable_branches;

  /// Legacy AC-only switchable branch IDs (ACBranch::index values).
  /// Empty = automatic tie-switch detection when switchable_branches is also empty.
  std::vector<int> switchable_branch_ids;

  /// 是否启用 LinDistFlow 潮流约束
  bool enable_pf{true};

  /// 电压下限 (p.u.)
  double v_min_pu{0.95};
  /// 电压上限 (p.u.)
  double v_max_pu{1.05};

  /// 支路额定容量默认值 (MVA)，当 rate_a_mva==0 时使用
  double default_rate_mva{10.0};

  /// Big-M for LinDistFlow voltage drop constraints (p.u.²)
  double big_m_v{2.0};

  /// 最大求解时间 (秒)
  int max_time_s{300};
  /// MIP 间隙
  double mip_gap{0.01};
  /// 是否输出详细日志
  bool verbose{false};
  /// 是否跳过图启发式（强制 MILP 求解）
  bool skip_heuristic{false};
};

// ── 拓扑重构结果 ───────────────────────────────────────────────────────

struct TopoReconfResult {
  bool feasible{false};
  bool optimal{false};

  // ── Structured branch references (unambiguous in hybrid systems) ────
  /// Branches that are open (de-energised) in the optimal topology.
  std::vector<BranchRef> open_branches;
  /// Branches that are closed (energised) in the optimal topology.
  std::vector<BranchRef> closed_branches;
  /// Branches newly closed relative to the initial topology.
  std::vector<BranchRef> switched_on;
  /// Branches newly opened relative to the initial topology.
  std::vector<BranchRef> switched_off;

  // ── Legacy integer ID vectors (backward compatibility) ─────────────
  /// @deprecated  Use open_branches instead.  Value is edge_orig_idx:
  ///   [0, nl_ac) → ACBranch::index, [nl_ac, nl) → DCBranch::index,
  ///   [nl, nl+nl_vsc) → VSCConverter::index.  In hybrid systems these
  ///   ranges may overlap, making bare ints ambiguous.
  std::vector<int> open_branch_ids;
  /// @deprecated  Use closed_branches instead.  Same caveat as open_branch_ids.
  std::vector<int> closed_branch_ids;
  /// @deprecated  Use switched_on instead.
  std::vector<int> switched_on_ids;
  /// @deprecated  Use switched_off instead.
  std::vector<int> switched_off_ids;

  int n_switch_on{0};
  int n_switch_off{0};

  double base_loss_mw{0.0};
  /// Approximate post-reconfiguration resistive loss [MW].
  /// Computed as Σ r_pu × base_mva for all closed branches, assuming nominal
  /// current (1 pu) on every branch.  This is a topology-comparison proxy
  /// only — it is NOT the actual solved power loss.  For accurate loss values
  /// run a full power flow on the reconfigured topology.
  double reconf_loss_mw{0.0};
  double loss_reduction_mw{0.0};
  double loss_reduction_pct{0.0};
  double milp_objective{0.0};
  double solve_time_s{0.0};

  /// Native B&C solver diagnostics.
  solver::BCStats bc_stats;

  /// Unified solver certificate for heuristic, HiGHS, and native B&C paths.
  std::string solver_backend;
  std::string solver_status;
  double solver_mip_gap{0.0};
  bool proven_optimal{false};

  /// 摘要信息
  std::string summary() const;
};

// ── 主接口 ─────────────────────────────────────────────────────────────
/// 对给定的混合 AC/DC 电力系统执行故障后拓扑重构
TopoReconfResult run_topology_reconfiguration(
    const HybridPowerSystem& sys,
    const TopoReconfOptions& opt = {});

}  // namespace hacdcpf::analysis
