# 大型富混合 AC/DC IPM 内部诊断与求解器调用链

## 1. 调用链

大型 AC/DC OPF 的原生求解路径为：

1. `solve_ac_opf()`：校验拓扑、提升外部电网为可调边界电源并选择后端；
2. `solve_with_parity_ipm()`：构造统一 AC/DC 全空间问题；
3. `parity::build_problem()`：生成变量、等式/不等式、上下界和元件映射；
4. `parity::solve_primal_dual_ipm()`：执行 Mehrotra 预测—校正内点法；
5. `factor_kkt_sparse()`：组装并进行 Ruiz 对称均衡的稀疏 KKT；
6. SuiteSparse UMFPACK、KLU 或 Eigen SparseLU：数值分解与回代；
7. `solve_with_parity_ipm()`：还原 Vm、Va、Vdc、Pg/Qg、换流器和 DER 结果；
8. `verify_opf_result()` / PF：进行独立物理审计和潮流回放。

可以通过环境变量
`HACDCPF_OPF_LINEAR_SOLVER=umfpack|klu|eigen|dense`
强制底层线性代数后端，用于交叉诊断。

## 2. KKT 数学结构

不等式采用松弛变量，凝聚后的预测—校正方程为

\[
\begin{bmatrix}
W+\delta I & J_g^T\\
J_g & -\delta I
\end{bmatrix}
\begin{bmatrix}\Delta x\\\Delta\lambda\end{bmatrix}
=
\begin{bmatrix}r_d\\r_p\end{bmatrix},
\]

其中

\[
W=\nabla^2_{xx}\mathcal L+J_h^T
\operatorname{diag}(\mu/z)J_h.
\]

KKT 在分解前执行对称 Ruiz 均衡
\(\widehat K=DKD\)，并采用逐级正则化和迭代改进。UMFPACK、KLU、
Eigen SparseLU 与 Dense LU 使用同一模型、同一 KKT 和同一停止判据。

## 3. case300_acdc 的根因

故障不是模型不可行，也不是某个 LU 库单独失效：

- Ipopt 可将最大原始约束残差降到约 \(2.5\times10^{-9}\)，证明模型可行；
- 修复前 KLU、Eigen SparseLU 和 Dense LU 均在 1600 次迭代后停留在约 3%
  的原始残差；
- UMFPACK 轨迹在第 723 次迭代附近进入病态区域，随后所有稀疏分解均失败。

原生 IPM 在得到预测—校正方向后直接执行全步更新，只用
fraction-to-boundary 保护松弛变量和乘子为正，没有检查更新后真实的非线性
AC/DC 残差。局部 Newton 方向因此可能连续恶化全局残差，最终使
\(J_g\) 和凝聚 KKT 数值退化。底层分解失败是该轨迹失控的结果，而非首因。

## 4. 修复

原生 IPM 对维数不小于 512 的大型 KKT 增加残差 filter 回溯。每个候选步重新计算完整 AC/DC 等式、
不等式、Jacobian、目标梯度和拉格朗日梯度。候选点至少要在以下一条轴上
取得充分进展，同时受另外两条轴的保护：

1. 原始可行性；
2. 对偶平稳性；
3. 互补性。

不满足 filter 的步长按 0.5 回溯，最多 14 次。小型 KKT 保留原有快速全步路径，
避免对已经稳定的小系统引入不必要的严格全局化。该机制位于
`src/optimal_power_flow/parity_ipm.cpp`，在线性求解完成后、状态更新前执行，
因此覆盖 UMFPACK、KLU、Eigen SparseLU 和 Dense LU 全部底层后端。

同时把 KKT 分解次数、回代次数、接受步和拒绝步写入 `ACOPFProfiling`，测试和
GUI/API 调试时可以直接区分“线性代数失败”和“全局化拒绝”。

## 5. 修复后的内部证据

`case300_acdc` 使用原生 parity IPM + UMFPACK 时：

- 42 次迭代收敛；
- 最大约束残差约 \(1.74\times10^{-7}\)；
- 最大平稳性残差约 \(2.77\times10^{-7}\)；
- 41 次接受步、17 次回溯拒绝；
- 41 次 KKT 分解、158 次预测/校正/Gondzio 回代。

强制 UMFPACK、KLU、Eigen SparseLU 和 Dense LU 四种后端均通过同一大型
富混合算例，说明收敛不再依赖单一线性求解库。

## 6. 时序热启动

`ACOPFOptions::warm_start` 可接收上一时段的兼容 `ACOPFResult`。结果中除
Vm/Va、Pg/Qg、Vdc、VSC、DER、储能、DC/DC、柔性负荷和能量路由器等物理
变量外，还保留原生 IPM 的等式乘子、不等式乘子和松弛变量。维数和正性检查
全部通过时，下一次求解复用完整原始—对偶中心路径状态；任何状态块不兼容时，
自动回退到物理初值和冷启动乘子。

在相同 `case300_acdc` 上，冷启动为 42 次迭代，完整原始—对偶热启动为 2 次，
初始原始残差从约 (4.46\times10^{-2}) 降至 (1.23\times10^{-6})。时序 OPF
已自动把上一时段收敛结果传给下一时段。

## 7. Ipopt 参数透传

`MIPSolvers::NLPModel` 现在携带 `NLPSolverOptions`，嵌入式
`IpoptAdapter` 使用其中的 `max_iterations`、`tolerance` 和
`acceptable_tolerance`，不再固定为 500、$10^{-8}$ 和 $10^{-6}$。
`solve_parity_with_ipopt()` 将 `ACOPFOptions` 形成的总 IPM 迭代上限直接传给
`max_iterations`，将原始、对偶和互补容差中的最小值作为严格 `tol`、最大值
作为 `acceptable_tol`。这样原生 IPM 与 Ipopt 接收同一组调用方预算和停止精度。

macOS arm64 的默认组合为“仓库内嵌 Ipopt + Homebrew MUMPS 动态库”。通过
`MIPSOLVERS_FORCE_BUILD_MUMPS=ON` 可以从已缓存的 MUMPS 5.7.3 源码构建本地
静态库；该配置已能完成编译和链接，但运行适配器测试时在第 1 次迭代进入
`Ipopt restoration failed`，而相同内嵌 Ipopt 配合 Homebrew MUMPS 可在 11 次
迭代收敛。因此在修复 arm64 静态 MUMPS 的运行时数值或 ABI 问题前，不把本地
MUMPS 设为默认后端。

## 8. 2000 节点富混合系统实测

使用仓库内真实 `case_ACTIVSg2000`，并通过 `build_case2000_acdc()` 附加多端
直流网络和 8 个 VSC。Release 构建下的单次冷启动结果为：

| 后端 | 预算/迭代 | 墙钟时间 | 最大约束违反 | 平稳性残差 | 结果 |
|---|---:|---:|---:|---:|---|
| 原生 parity IPM + sparse UMFPACK | 90 次 | 7.07 s | $3.49\times10^{-9}$ | $9.03\times10^{-7}$ | 收敛 |
| 内嵌 Ipopt + Homebrew MUMPS | 100 次预算 | 48.15 s | $4.96\times10^{-4}$ | 1.49 | 未收敛 |

原生 IPM 共执行 89 次 KKT 分解和 346 次预测、校正及 Gondzio 回代。Ipopt 的
100 次有界基准保持数值有限，但在耗时约为原生路径 6.8 倍后仍未达到原始或
对偶停止条件。因此当前 2000 节点富混合系统应优先使用利用网络稀疏结构的
原生 IPM；Ipopt 保留为小中型问题后端和独立交叉验证路径。

## 9. 边界

该修复解决的是普通经济 AC/DC OPF 的原生 IPM 全局化。RPO 的电压偏差/网损
目标目前仍优先使用嵌入式 Ipopt filter-line-search 作为连续 NLP 后端；其离散
层仍是局部搜索，不构成全局 MINLP 证明。三相不平衡单体 RPO 和富混合设备的
完整 authored-space PF 写回仍按覆盖审计标记为部分支持。

## 10. MIPSolvers/Ipopt 参数稳定性矩阵

### 10.1 参数契约与残差口径

专项审计发现，单独设置 Ipopt 的 `tol` 和 `acceptable_tol` 不等于约束了所有
原始量纲下的 KKT 分量。Ipopt 的总 NLP error 会进行目标和乘子缩放，而
`dual_inf_tol`、`constr_viol_tol`、`compl_inf_tol` 是独立的绝对门槛。对于
成本梯度、功率约束和电压变量量纲不同的 OPF，不能把同一个数值无条件复制给
所有未缩放分量。

`NLPSolverOptions` 因此保留原有三个总控参数，并新增六个分量参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `max_iterations` | 500 | Ipopt `max_iter`，小于 1 时归一化为 1 |
| `tolerance` | $10^{-8}$ | 缩放后总 NLP error 的严格容差 |
| `acceptable_tolerance` | $10^{-6}$ | 缩放后可接受收敛容差 |
| `dual_infeasibility_tolerance` | 1 | 未缩放对偶不可行度严格门槛 |
| `constraint_violation_tolerance` | $10^{-4}$ | 未缩放约束违反严格门槛 |
| `complementarity_tolerance` | $10^{-4}$ | 未缩放互补残差严格门槛 |
| `acceptable_dual_infeasibility_tolerance` | $10^{10}$ | 可接受对偶门槛 |
| `acceptable_constraint_violation_tolerance` | $10^{-2}$ | 可接受约束门槛 |
| `acceptable_complementarity_tolerance` | $10^{-2}$ | 可接受互补门槛 |

分量默认值与 Ipopt 保持一致，避免改变有量纲 OPF 的历史收敛口径。调用方只有在
模型已经无量纲化时才应统一收紧六个分量门槛。`SolveStats` 同时提供原有的
scaled 残差和 `unscaled_primal_feas`、`unscaled_dual_feas`、
`unscaled_complementarity`，防止把缩放后的微小数值误称为物理 KKT 残差。

零、负数、NaN 和 Inf 容差现在稳定回退到历史默认值；
`acceptable_tolerance < tolerance` 及相同的分量关系会归一化为“不小于严格
容差”。无约束 NLP 不再虚构 Jacobian 非零项。所有 Ipopt 退出码也映射为明确
状态，而不是统一的 `Ipopt failed`。

### 10.2 专项 NLP 结果

Homebrew Ipopt/MUMPS 组合通过 439 项断言，覆盖：

- $10^{-4}$、$10^{-6}$、$10^{-8}$、$10^{-10}$ 四档严格容差与
  1、10、1000 倍 acceptable 容差的 12 组组合；
- 1、5、20、100、500 次迭代预算；
- 非正、NaN、Inf 参数和 acceptable/strict 倒置；
- 活动变量边界、Rosenbrock 等式约束、近线性相关等式；
- 变量和 Jacobian 的尺度失衡。

Jacobian 系数跨度到 $10^6$ 时仍得到正确最优点。跨度达到 $10^{12}$ 时，默认
gradient scaling 会把 scaled 对偶误差显著压小；启用严格的未缩放分量门槛后，
适配器不再报告假收敛，而是明确返回 `Ipopt search direction too small`。这类
模型必须在建模层进行变量/约束无量纲化，不能仅靠降低 `tol` 修复。

强制源码构建的顺序 MUMPS 5.7.3 在相同六类测试中全部于第 1 次迭代进入
`Ipopt restoration failed`，首个良态二次 NLP 的诊断为
`primal=3, dual=999`。故障与容差、预算和模型类型无关，继续归类为 arm64
本地静态 MUMPS 的运行时/ABI 问题；当前默认仍应使用 Homebrew MUMPS 动态库。

### 10.3 真实 OPF 参数敏感性

以下为 Release、内嵌 Ipopt + Homebrew MUMPS 的冷启动实测。时间是单机墙钟
时间，只用于同机相对比较。

| 算例 | 预算 | 可行/平稳/互补容差 | 迭代 | 时间/s | 原始残差 | scaled 对偶残差 | 结果 |
|---|---:|---|---:|---:|---:|---:|---|
| case30 | 1 | $10^{-8}/10^{-8}/10^{-10}$ | 2 | 0.004 | $3.10\times10^{-2}$ | 84.1 | 截断 |
| case30 | 20 | $10^{-4}/10^{-4}/10^{-8}$ | 21 | 0.020 | $5.06\times10^{-7}$ | $2.27\times10^{-2}$ | 截断 |
| case30 | 100 | $10^{-6}/10^{-6}/10^{-8}$ | 70 | 0.064 | $1.05\times10^{-8}$ | $5.66\times10^{-9}$ | 收敛 |
| case30 | 200 | $10^{-8}/10^{-8}/10^{-10}$ | 85 | 0.078 | $1.05\times10^{-8}$ | $3.32\times10^{-11}$ | 收敛 |
| case300 AC/DC | 100 | $10^{-7}/10^{-3}/10^{-9}$ | 101 | 0.587 | $2.83\times10^{-5}$ | 0.150 | 截断 |
| case300 AC/DC | 500 | $10^{-7}/10^{-3}/10^{-9}$ | 501 | 2.98 | $2.47\times10^{-9}$ | $4.68\times10^{-4}$ | 未满足连续 acceptable 判据 |
| case300 AC/DC | 800 | $10^{-7}/10^{-2}/10^{-9}$ | 497 | 2.96 | $1.22\times10^{-8}$ | $1.69\times10^{-3}$ | 收敛 |
| case300 AC/DC | 800 | $10^{-7}/10^{-3}/10^{-9}$ | 722 | 4.34 | $4.19\times10^{-8}$ | $1.39\times10^{-4}$ | 收敛 |
| case300 AC/DC | 800 | $10^{-7}/10^{-4}/10^{-9}$ | 801 | 4.83 | $3.31\times10^{-8}$ | $9.61\times10^{-4}$ | 截断且后期退化 |
| case2000 AC/DC | 20 | $10^{-6}/10^{-3}/10^{-8}$ | 21 | 10.10 | $1.07\times10^3$ | $3.24\times10^5$ | 截断 |
| case2000 AC/DC | 50 | $10^{-6}/10^{-3}/10^{-8}$ | 51 | 23.69 | 0.110 | 5.99 | 截断 |
| case2000 AC/DC | 100 | $10^{-6}/10^{-3}/10^{-8}$ | 101 | 46.82 | $4.96\times10^{-4}$ | 1.49 | 截断 |

据此，当前推荐配置为：case30 级别至少 100 次；case300 AC/DC 使用 800 次、
可行容差 $10^{-7}$、平稳性容差 $10^{-3}$、互补容差 $10^{-9}$。在当前
limited-memory Hessian + MUMPS 组合上，把 case300 平稳性收紧到 $10^{-4}$
不会单调改善结果，800 次末反而偏离约第 500--720 次附近的较好迭代。对
case2000 AC/DC，100 次内虽然原始残差持续下降，但对偶平稳性远未合格；生产
路径继续推荐原生 parity IPM + UMFPACK，Ipopt 仅作为有界交叉诊断后端。

---

## 更新（2026-07-19）：MUMPS 后端、增广 KKT 与 (θ,φ) filter

本文 §1–§2 描述的调用链仍然成立，但以下默认值已更新（均可用环境变量回退到旧行为）：

1. **线性代数后端**：MIPSolvers 提供 MUMPS（对称不定 LDLᵀ，含惯性信息）时为默认后端，顺序为 MUMPS → UMFPACK → KLU → Eigen SparseLU；`HACDCPF_OPF_LINEAR_SOLVER=mumps|umfpack|klu|eigen|dense`。
2. **Newton 系统形式**：默认改为**增广 KKT**（保留 −ZM⁻¹ 对角块，不再凝聚成 W = Lxx + JhᵀΣJh；`HACDCPF_OPF_KKT_FORM=condensed` 可切回）。动机：凝聚形式把 σ=μ/z 的动态范围（~1e10）平方进条件数，导致 PEGASE/rte 大算例上所有后端得到同一条"残差小但方向错"的失败轨迹；增广形式 + Wächter–Biegler δ_W 惯性校正（negevals 必须等于 meq+niq）从理论上消除了该病态。
3. **全局化**：残差 filter 升级为 Wächter–Biegler (θ,φ) filter（含支配历史、切换条件与 Armijo、二阶校正 SOC、θ_max 上限）+ 有界 restoration（filter 全拒时用 (1,1):=ρI 的可行化步）。原三轴判据保留为附加接受路径。
4. **暖启动稀疏化**：`dc_warm_start` 的稠密 (nb−1)² B′ 装配 + 稠密 LU 已改为稀疏装配 + SparseLU（case13659 仅此一项 48.3 s → 5.5 s）。

数值推导与实测瓶颈层级见 MIPSolvers `docs/numerical_methods.md` §9–§10。

## 更新（2026-07-19，其二）：AC 潮流暖启动选项

`ACOPFOptions::ac_pf_warm_start=true`（默认 false）会先从算例当前运行点求解普通 AC 潮流（Newton–Raphson），把 AC 可行的 (vm, va) 通过既有 warm-start 映射注入 parity IPM —— IPM 第 0 次迭代即 θ ≈ 0，始终停留在可行域盆地内。

实测：case13659pegase 以 189 次迭代、约 41 s 收敛于参考最优 obj=386107（maxviol 2.2e-6），此前所有路径（含 Ipopt 同机 ~55 min 未返回）均失败。注意选项是**选择性启用**而非默认：在良态算例上精确可行起点反而更慢（case1354：88→330 迭代），个别 rte 算例落入较差盆地（6468）或不收敛（6495）；建议在默认起点失败的大型/受压电网上启用。理论分析见 MIPSolvers `docs/numerical_methods.md` §11。
