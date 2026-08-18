> 本文档为 [network_reconfiguration_models.md](network_reconfiguration_models.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

> 文档同步（2026-07-12）
> 范围：已对照当前仓库结构、CMake presets/options 与已注册的测试目标完成审阅。
> 状态：有实现支撑的参考文档。
> 事实来源：当文字与实现不一致时，以 src/、include/、tests/ 与 CMake 文件为准。

# 最优网络重构 — 数学模型（规范空间）

本文档是交直流混合最优网络重构（optimal network reconfiguration, ONR）的工程/数学参考。它取代了归档技术笔记章节
`docs/archive/reference/technical_notebook/sections/07_network_reconfiguration.tex`
中的叙述性文字。该材料在此被重述为可配置的约束组、目标函数和一个可选潮流层，以便从 GUI 开关切换。断路器（CB）、
开关、AC/DC 线路与 DC/DC
换流器在**规范（canonical）**模型空间上被优化，然后投影回设备操作。

> **配套 / 增强：** 可靠性评估数学参考与严谨性审计——
> [`reliability_assessment_models.md`](reliability_assessment_models.md)——
> 记录了本 ONR LinDistFlow 模型如何被复用为 FMEA 修复搜索与三阶段可靠性 MILP 的逐阶段恢复内核，
> 并将每条可靠性公式分类为严谨或启发式。

实现：`src/network_reconfiguration/topology_reconfiguration.cpp`
（`run_topology_reconfiguration`）与 `topology_analysis.cpp`
（`solve_optimal_reconfiguration`）。

## 1. 规范投影与混合边集

在 $\widehat{\mathcal S}=\Pi_{\text{canon}}(\mathcal S)$ 上求解，使用
`project_to_canonical_models(sys, strip_dead=false)`（保留死区段，使打开的联络开关仍可作为候选）。边集：

$$\mathcal E_{\text{hyb}}=\mathcal E_{ac}^{\text{canon}}\cup\mathcal E_{dc}^{\text{canon}}\cup\mathcal E_{vsc}^{\text{canon}}.$$

设备通过 `BranchExpandMap` 映射到规范边：每个 `Switch`/`CircuitBreaker` → 一条带有端点母线与闭合状态的 `ACBranch`。线路状态决策 $z_\ell$ 由设备命令 $\Phi_\ell$ 实现：

$$z_\ell=u_{sw}\ \text{(switch/tie)},\quad z_\ell=u_{cb}\ \text{(single-side)},\quad z_\ell=u_{\text{from}}\wedge u_{\text{to}}\ \text{(double-side)}.$$

## 2. 变量

每条边 $e$ 与每条母线 $i$：$\beta_e\in\{0,1\}$ 状态；$P_e,Q_e$ 潮流；$v_i=|V_i|^2$；$f_e$ 商品流（commodity flow）；$\gamma_g\in\{0,1\}$ 根；$s_i^P,s_i^Q\ge0$ 切负荷；$t_e\ge0$ 用于 $|P_e|$。

## 3. 约束组（可在 GUI 中开关）

- **G1 树基数约束：** $\sum\beta_e+\sum\gamma_g=n_b$。
- **G2 连通性（商品流）：** 根节点 $\sum f-\sum f=-(n{-}1)$，其余 $=1$，$|f_e|\le n_b\,\beta_e$。
- **G3 功率平衡：** $\sum_{\text{in}}P-\sum_{\text{out}}P-P_g=-P_i^{net}-s_i^P$（Q 类似，AC）。
- **G4 电压降（LinDistFlow，大 M 法）：** $|v_j-v_i+2r_eP_e+2x_eQ_e|\le M(1-\beta_e)$。
- **G5 热稳定：** $|P_e|\le P_e^{\max}\beta_e$，$|Q_e|\le Q_e^{\max}\beta_e$。
- **G6 VSC 传输：** $|P_e^{vsc}|\le S^{\max}$，无功近似处理。
- **G7 开关操作预算：** $\sum|\beta_e-\beta_e^0|\le N_{sw}$。

G3–G6 仅在**可选潮流（PF）**开启时生效；G1–G2 始终生效（纯连通性）。G4/G5 可独立开关（`enable_voltage`、`enable_thermal`）。G7 在 `max_switch_ops>0` 时生效。

## 4. 目标函数（权重可在 GUI 中配置）

$$\min\ \lambda_{sw}\!\sum_{\text{ties}}\!\beta_e-\lambda_{sw}\!\sum_{\text{in-svc}}\!\beta_e+\lambda_{loss}\!\sum r_e\beta_e+\lambda_{shed}\!\sum(s^P+s^Q)+\lambda_{isl}\!\sum_{g>0}\gamma_g.$$

开关项：最小网损（$\lambda_{loss}$）、最小开关操作（$\lambda_{sw}$）、最大恢复（$\lambda_{shed}$）、最少孤岛（$\lambda_{isl}$）。

## 5. 可选潮流

PF 关闭 → 仅 G1/G2（连通性，快速）。PF 开启 → LinDistFlow G3–G6（电压/热稳定感知）。

## 6. 优化后潮流交叉校验

应用 $\beta^\*$，运行完整 Newton 潮流；报告辐射状/连通/孤岛情况、真实网损，以及经 `compute_device_terminal_flows` 得到的断路器潮流。标志位 `post_power_flow_validated`/`full_hybrid_opf_validated`。

## 7. 理论：AC/DC 换流器需要树约束吗？

**不需要——VSC/DC-DC 必须从辐射状约束中排除。** 辐射状约束按电气域分别施加。VSC 桥接一个 AC 节点与一个 DC 节点；闭合它永远不会形成 AC 环网，因此将其计入 $\sum\beta=n_b{-}1$ 会过度约束。正确做法：分域树 + 换流器作为可控功率传输，$|P^{vsc}|\le S^{\max}$，健康换流器 $z=1$。即使用 $\sum_{AC}\beta=n_{ac}{-}1$、$\sum_{DC}\beta=n_{dc}{-}1$，换流器自由。通过 `split_domain_trees` 启用：VSC 的 β 从基数约束中剔除，使网状 MTDC 链路保持闭合。
