> 文档同步：2026-08-21
> 状态：有实现支撑的模型速查。
> 完整契约：[`docs/modules/network_reconfiguration/network_reconfiguration_manual.tex`](../modules/network_reconfiguration/network_reconfiguration_manual.tex)。

# 规范空间网络重构模型

当前维护入口是 `analysis::run_topology_reconfiguration(const
HybridPowerSystem&, const TopoReconfOptions&)`，在规范 AC/DC/VSC 边空间求解一个单时刻快照。
历史 `solve_optimal_reconfiguration` 现为该入口的 AC 兼容包装；其无条件返回之后保留的旧 AC B&C
函数体不可达。

## 1. 投影与标识

核心固定以 `strip_dead_islands=false`、`preserve_switch_branches=true` 投影。AC/DC 母线使用独立
ID map，只在内部组合为本地位置。混合边输入输出使用
`BranchRef { EdgeCategory, component.index }`；AC、DC、VSC 同号时，旧裸整数向量有歧义。

规范边集为

$$E=E_{ac}\cup E_{dc}\cup E_{vsc}.$$

`BranchExpandMap` 把变化的规范 AC 边归因回富模型开关或断路器。设备能力、锁定、熔断器限制和上游
保护绑定可以否决显式候选。DC/VSC 候选仍是分支级决策。

## 2. 变量与拓扑

始终存在的拓扑块包含虚拟流 $F_e$、根注入 $F_g$、边状态 $\beta_e$ 和根指示 $\gamma_g$。
`enable_pf=true` 时增加线路/VSC 的 $P,Q$、源 $P_g,Q_g$、平方电压 $v_i$ 与非负切负荷
$s_i^P,s_i^Q$；同时开启 `loss_aware` 时增加 $t_e\ge|P_e|$。

统一模式使用源定根虚拟流与

$$\sum_{e\in E_{participating}}\beta_e+\sum_g\gamma_g=n_b.$$

分域模式把 VSC 虚拟流固定为零，对每个预计算 AC 分量、以及未开启 `allow_dc_mesh` 时的 DC 分量
分别施加树边数。VSC 不计入 AC/DC 树边数；`allow_dc_mesh=true` 不证明 DC 径向。

故障边固定断开，非候选保持原状态。动作预算展开 $|\beta-\beta^0|$ 并使用设备动作成本；要求失电
操作的隔离序列保守按三次动作计。

## 3. 可选电气层

PF 开启时，有功平衡覆盖 AC/DC 母线，无功平衡只覆盖 AC。线路电压近似为

$$|v_j-v_i+2r_eP_e+2x_eQ_e|\le M_e(1-\beta_e),$$

DC 线没有 $Q$ 项。热限是 P/Q 独立箱约束，不是圆形 MVA 约束。VSC 有功在两端等量进入且不含
效率损耗；VSC 无功是 AC 端口近似。模型没有电流平方变量，节点平衡也不含支路损耗。

AC 母线需求与 `Load` 行相加，DC 母线需求与 `DCLoad` 行相加。源容量聚合发电机、DER、储能、
外部电网、DC 源和合格 DC 电压母线。零容量伪根允许孤立区段保留图表示；PF 平衡会把其需求计为
切负荷。

## 4. 目标与损耗口径

目标是开关、损耗代理、切负荷和额外根的加权和。初始闭合边的动作项省略常数，因此
`milp_objective` 不等于正向 `obj_terms` 之和，也没有 MW 单位。

- `obj_terms.loss`：加权目标贡献；
- 核心 `reconf_loss_mw`：$base\_mva\sum_{closed\ AC/DC}r_e$，是假设 1 pu 电流的拓扑代理；
- HTTP `reconfig_loss_mw`：后验证 PF 支路损耗，仅在 `reconfig_pf_converged=true` 时有意义。

当前 HTTP 的 `estimated_loss_mw = milp_objective * base_mva` 量纲错误，已登记为 AUD-018；客户端
必须忽略该字段。

## 5. 求解与证书边界

图启发式只能返回可行 incumbent，不能证明最优。实际后端分派为：`native` 只走原生 B&C；`scip`
只走 SCIP；`highs` 和 `auto` 为 HiGHS 失败后 SCIP；未知字符串也按 auto。

所有解都检查线性等式、不等式、整数性与变量界。`feasible` 只代表该线性模型通过。核心不运行完整
混合 PF/OPF；生产 HTTP 路由把动作写回富模型、重新投影、运行 PF 和 AC OPF 后才形成
`executable`。

兼容 `ONRResult` 在核心候选后运行 AC PF，但 PF 不收敛不会清除优化 `feasible`。它还存在一个
回退：核心失败但原始 connected 拓扑 PF 收敛时，返回原拓扑并设 `feasible=true`，当前没有字段标明
该回退。

## 6. 明确限制

本模型是单时刻、平衡稳态近似。核心入口不实现精确 DistFlow/SOCP/AC 重构、三相开关、多时段储能
和动作计划、N-1/随机 ONR、动态保护、通信失败或完整混合非线性 PF/OPF 证书。
