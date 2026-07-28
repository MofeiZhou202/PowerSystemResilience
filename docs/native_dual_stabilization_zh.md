# Native Dual Simplex 稳定化架构

> 仅限开发：生产模块使用 `LpKernelBackend::HiGHS`；本文只约束显式
> `ExperimentalNative` 的研发工作。

> 状态：算法契约。本文替代“只要 Forrest-Tomlin 因子正确，dual simplex
> 就能投产”的假设。因子只是组件；major/minor rebuild 状态机才是数值稳定化
> 的主体。

## 结论与证据

ACTIVSg2000 FMEA 已经执行到数千次合法换基。失败时，单次 HFactor 求解通常仍
满足未放宽的 backward-error 界；真正失效的是长期增量状态：primal、reduced
cost、objective 与 DSE weight 的误差在 update chain 中传播，而裸循环在
`possibly optimal/infeasible` 时没有先对当前基重建全部状态。

因此，继续更换 LU 或调 FT 阈值不能完成求解器。native 必须实现 HiGHS
`HEkkDual` 包裹在线性代数外面的稳定化闭环。

## 参考实现映射

| HiGHS 位置 | native 必须实现的机制 |
|---|---|
| `HEkkDual::solvePhase2` | major rebuild + minor iteration 双层循环 |
| `HEkkDual::rebuild` | 重算 dual、primal、infeasibility set、objective |
| `HEkkDual::cleanup` | 终止前移除 cost perturbation/shift |
| `HEkkDual::iterate` | 原子化 minor pivot transaction |
| `HEkkDual::chooseRow` | 接受 leaving row 前验证精确 DSE weight |
| `HEkkDual::updateVerify` | 行列证据异常只产生 rebuild reason |
| `HEkkDual::correctDualInfeasibilities` | rebuild 后修正工作目标的对偶不可行 |
| `HEkkDualRHS` | 维护 primal infeasibility 数组与候选集合 |
| `HEkk::rebuildRefactor/updateFactor` | INVERT、update limit、synthetic work |
| `HEkk::initialiseCost` | 只扰动工作目标，不改变发布目标 |

首版不移植 parallel slices、PAMI、基回滚/backtracking 与 HiGHS runtime
fallback；但实现串行 cycle detection 和有作用域的 taboo basis change，已经提交
的 pivot 不回滚。

## Major/Minor 状态机

```text
MAJOR REBUILD
  按 rebuild reason 对当前基执行 INVERT
  从同一 B 和 c_work 重建 x_B、y、r、objective
  重建 primal infeasibility 集合与精确/受检 DSE weight
  用确定性 cost shift 修正工作目标的 dual infeasibility
  测量并记录增量态漂移
  fresh_rebuild = true
       |
       v
MINOR ITERATIONS
  CHUZR -> BTRAN -> PRICE/BFRT -> FTRAN -> verify -> 原子提交
  增量更新 x_B、r、objective、候选集合、weight、factor
  fresh_rebuild = false
       |
       +-- update limit / synthetic work / numerical evidence --> MAJOR
       +-- possibly optimal / infeasible ----------------------> MAJOR
```

`RebuildReason` 不是 LP status。首版包括：`Initial`、`UpdateLimit`、
`SyntheticWork`、`NumericalTrouble`、`PrimalDrift`、`DualDrift`、
`ObjectiveDrift`、`PossiblyOptimal`、`PossiblyPrimalInfeasible`、`Cleanup`。

禁止 pivot blacklist 和备选 entering pivot 搜索。数值事件只在同一当前基上
rebuild；若 fresh 状态仍出现同一代数失败，则携带 residual 证据返回 numerical
status。

## Freshness 契约

只有下面所有量从同一个当前基和同一个工作目标重算后，状态才是 fresh：

```text
x_B = B^-1 (b-Nx_N)
y   = B^-T c_B
r   = c_work-A^T y
z   = c_work^T x
I_p = 当前违反 active bounds 的基本行集合
```

任意换基或 bound flip 都清除 freshness。单独一次 FTRAN/BTRAN 不能恢复
freshness。major rebuild 会在覆盖增量态前记录：

```text
delta_x = ||x_B_incremental-x_B_rebuilt||_inf
delta_r = ||r_incremental-r_rebuilt||_inf
delta_z = |z_incremental-z_rebuilt|
```

这些是漂移证据，不是放宽后的容差。发布仍使用原
`feasibility_tol/optimality_tol`。

## 工作目标、扰动与清理

求解器同时保存：

```text
c_original  不可变 canonical objective
c_work      稳定化 minor iteration 使用的 objective
sigma       c_work-c_original
```

确定性 cost perturbation 用于拆开大规模退化 breakpoint 组。它按列索引生成
可复现扰动，只改变 `c_work`，不改变 bounds、容差或迭代预算。lower-only 变量
向负方向扰动，upper-only 向正方向扰动，boxed 按原 cost 符号远离零，fixed 不
扰动。

rebuild 后若非基本变量违反 `d_j r_j <= tau_d`，deterministic cost shift 只修改
该列 `c_work[j]`，使重建 reduced cost 精确为零。它是辅助工作目标，不是容差
放宽；free variable 不能用此法修复。

工作目标上出现 possibly optimal 后必须 cleanup：

1. 恢复 `c_work=c_original`，清空全部 perturbation/shift；
2. 在不换基的情况下 fresh rebuild；
3. 原目标下 primal/dual audit 均通过才发布 `Optimal`；
4. 否则在无扰动问题上继续，或进入显式 primal cleanup phase。

cleanup 后本次 solve 禁止再次 perturb，避免 perturb-cleanup 循环。

## 因子与 DSE 稳定化

FT update 仍是 minor iteration 的常规基表示，但 INVERT 决策归 driver 所有。
HFactor hint、update limit、synthetic work、backward-error failure、行列 residual
identity failure，以及非 fresh 的终止候选都会触发 major rebuild。

每次求解返回结构化证据：`accepted`、`needs_rebuild`、residual、error limit 与
是否 refinement。factor wrapper 禁止调用 `factorize`。普通、批量和 pivotal
solve 都允许在同一因子上做一次 iterative refinement。若 pivotal vector 被精化，
HFactor 捕获的原 FT pack 已不再对应最终向量；此时原子提交已经验证的基交换，但
不使用该 pack，driver 在下一次 minor iteration 前立即对新基执行 INVERT 和完整
reconstruction。

当 `||B|| ||x_B||` 很大时，`Bx_B=rhs` 的 backward stability 可能弱于 canonical
feasibility contract。reconstruction 因此用扩展精度累加 `r=b-Ax`；若未达到原固定
门限，只做一次 defect correction `B Delta x_B=r`，再按同一门限检查。失败即返回
numerical status，绝不增大门限。

精确 DSE 初始化是 factor-level batch：所有单位 BTRAN 复用现有的一次 INVERT，
每个 RHS 按未放宽 backward-error 界检查，并可在同一因子上 refinement 一次。
任何一行都无权触发同基 INVERT；批次失败只产生一个 driver-visible rebuild 请求。

edge-weight framework 由明确状态选择：逻辑/对角基使用精确单位（或缩放对角）
DSE weight；warm basis 若携带匹配的 DSE cache 则继续 DSE；没有 cache 的非逻辑
warm basis 建立 Devex reference framework，避免启动阶段 `m` 次 BTRAN。

DSE 选择行后重新计算 `pi=B^-T e_p` 与精确 weight `||pi||^2`。若缓存 weight
低估导致该行虚假地显得有吸引力，则把精确 weight 写回 CHUZR 并重新选择 leaving
row；这不是尝试另一个 entering pivot，而是恢复 DSE merit 的定义。Goldfarb-Reid
相消仍按定义式重建。

Devex 的 reference set `R` 是 framework 建立时的基本变量。pivotal row 的精确
reference weight 与换基更新为：

```text
w_p^R = max(1, sum_{j in R} (e_p^T B^-1 a_j)^2)
w'_p  = max(1, w_p^R / alpha^2)
w'_i  = max(w_i, w'_p u_i^2),  u=B^-1 a_q.
```

advisory weight 与精确 pivotal reference weight 的比值超过平方因子 `9` 时，
在下一个基上确定性重建 reference framework。该机制只改变 pricing，不改变任何
可行性或最优性容差。

## Harris BFRT 与防循环

对剩余 nonbasic column 定义 signed pivot `alpha_j>0`、signed reduced cost
`sigma_j=d_j r_j` 和精确 breakpoint
`theta_j=max(0,-sigma_j)/alpha_j`。BFRT 先消费完整的、更早的有限 bound range
group；Harris 第一遍由全部正 signed tableau coefficient 约束，包括小到不能作为
entering pivot 的系数，并计算：

```text
theta_H = min_j (tau_d - sigma_j) / alpha_j
        = min_j (tau_d - d_j r_j) / alpha_j.
```

第二遍在 `theta_j<=theta_H` 的候选中选择最大 `alpha_j`，最后才按 column index
确定性打破平局。最终发布 audit 仍使用原 `tau_d`，Harris 没有放宽容差。

cycle signature 包含有序 basis 与全部 nonbasic bound side。状态重复时，该状态
上次离开的 `(leaving, entering)` exchange 在相同状态中 taboo `2m+1` 个已提交
pivot。taboo edge 仍可参与 bound flip，只不能作为 basis change。若某 leaving row
的全部 Harris-eligible exchange 都是 taboo，则 CHUZR 临时 taboo 该 row，而不是
释放一个 entering edge；若全部 infeasible row 都是 taboo，CHUZR 才确定性释放最早
到期的 row。因此 taboo 不能制造虚假的 infeasible 结论，也不会在同一 row 上尝试
另一个 pivot。全过程没有 pivot trial、rollback 或容差修改。

## 原子 Minor Transaction

一次 minor iteration 必须按以下顺序完成：

1. 从维护的 primal infeasibility set 选择 leaving row；
2. 精确验证该行 DSE weight；
3. BTRAN 与 PRICE；
4. 执行一次确定性 BFRT，选择唯一 entering column；
5. 对 BFRT RHS、pivotal column、DSE vector 执行 FTRAN；
6. 验证 backward error 与 primal/adjoint residual identity；
7. 物化完整候选 primal、reduced cost、objective；
8. 更新 basis factor；若 pivotal solve 被精化或增量 primal residual 超过固定门限，
   则跳过旧 FT pack，并要求新基 INVERT；
9. 一次性提交 basis、bound flips、primal/dual/objective、候选集合和 weights；
10. 清除 freshness；若第 8 步要求，则在继续 pricing 前立即 INVERT 并完整重构已提交
    的新基。

第 6 步之后的任何失败都不能留下半提交状态。

## 终止协议与测试门

`PossiblyOptimal`、`PossiblyPrimalInfeasible` 必须先回 major loop。只有满足以下
全部条件才能发布：fresh state、`c_work==c_original`、无 perturbation/shift、
canonical audit 通过原容差、original-space audit 通过、不可行结论带受检 Farkas
certificate。

统计必须报告 major rebuild、INVERT、FT updates、各 rebuild reason、iterative
refinement、canonical primal correction、新基 pivot reinversion、
perturbation/shift、cleanup、DSE reselection、BFRT flips，以及最大
primal/dual/objective drift。

验收顺序：状态机单测、扰动清理恢复原 optimum、terminal-after-update 强制
rebuild、HFactor/fresh/dense 差分、LP/numerical/Netlib/MILP 回归、3655
个 ACTIVSg2000 FMEA 无 fallback，最后同机证明 100x。
