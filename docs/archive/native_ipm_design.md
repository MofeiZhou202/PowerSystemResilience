# Native LP 内点法理论设计与验证路线

本文是 `NativeIPMLPAdapter` 的长期设计依据，服务于算法实现、代码审核和
NETLIB 回归。研究资料核对日期为 2026-08-05。本文只把满足下列证据链的改动
纳入默认路径：

```text
论文或成熟实现的结论 -> 当前代码的可定位缺口 -> 单一设计改动
                         -> 可复现 A/B -> 原空间正确性审计
```

参数搜索只能用于验证理论中已经出现的量级，不能代替算法设计。NETLIB 上更快
但原问题目标或可行性错误的结果一律按失败处理。

## 1. 当前结论

`NativeIPMLPAdapter` 已经是一个有完整稀疏数值骨架的 Mehrotra 预测-校正法，
并非从零开始：它直接处理变量上下界和行松弛，含 Ruiz 缩放、Gondzio 多重中心
校正、normal equations / augmented KKT 分流、稀疏 LDLT、动态对角扰动及迭代
精化。但已有功能名称不等于理论条件已经满足。

当前优先级如下：

| 阶段 | 理论目标 | 状态 | 默认启用条件 |
|---|---|---|---|
| P0 | 求解预算、迭代统计、原空间审计契约 | 已实现 | 数值回归通过 |
| P1 | 标准 Mehrotra 中心参数和互补度驱动的动态步长 | 已实现并通过 A/B | 90 例准确率不降且失败集改善 |
| P2a | 原模型 primal/dual/gap 审计与相对停止候选 | 已实现并通过全量回归 | 只有原模型 KKT 审计通过才发布 Optimal |
| P2 | 齐次自对偶嵌入 HSD 和可审计证书 | 待实现 | 不可行/无界专用集证书全部通过 |
| P3 | IP-PMM / proximal regularization | 数值扰动误差预算已实现；完整 proximal center 待原型 | 退化集比 P2 有显著稳定性收益 |
| P4 | KKT 后端选择、混合精度和预条件迭代解 | 完整轨迹后端恢复已实现；质量驱动主路径待设计 | 分解失败率和总时间同时下降 |
| P5 | crossover 与重复节点热启动 | 待设计 | MIP 根和节点基准通过 |

P1 只解决“沿中心路径走得更稳”；它不提供不可行或无界证明。P2 不能被 P1
替代。P3 也不是把 KKT 对角线加大：只有把 proximal 项写进问题映射、残差和
收敛判据，才是有算法意义的 IP-PMM。

## 2. 现有数学模型和数据布局

对一般 LP

\[
\min c^T x,\quad A x\le b,\quad A_e x=b_e,\quad l\le x\le u,
\]

fresh 路径在 `NativeIPMLPAdapter::solve_lp_impl` 中增加不等式松弛
\(s=b-Ax\)，组成

\[
A_a\bar x=\bar b,\qquad
A_a=\begin{bmatrix}A&I\\A_e&0\end{bmatrix},\quad
\bar x=(x,s).
\]

每个有穷下界维护
\(g_l=\bar x-l>0,z_l>0\)，每个有穷上界维护
\(g_u=u-\bar x>0,z_u>0\)。固定变量退出障碍系统。互补度为

\[
\mu={\sum_j g_{l,j}z_{l,j}+\sum_j g_{u,j}z_{u,j}\over n_c}.
\]

主要数组均是长度 \(n+m_i\) 的连续 `std::vector<double>`；`flb/fub` 是
0/1 掩码。矩阵同时保留 CSC 乘法结构、normal-equation scatter map 和按结构
选择的 augmented KKT。cached 路径在 `ipm_lp_solver_cached.cpp` 复用同一
边界变量布局和共享步长辅助函数。

## 3. 文献结论与代码差距

### 3.1 Mehrotra 预测-校正

仿射方向令中心目标为零。以仿射最大正步长 \(\alpha_p^{aff},
\alpha_d^{aff}\) 计算

\[
\mu_{aff}={1\over n_c}\sum_j
(g_j+\alpha_p^{aff}\Delta g_j)
(z_j+\alpha_d^{aff}\Delta z_j),\qquad
\sigma=\left({\mu_{aff}\over\mu}\right)^3.
\]

校正方向同时加入 \(\sigma\mu\) 中心项和
\(-\Delta g^{aff}\Delta z^{aff}\) 二阶项，并复用同一 KKT 分解。

原实现把 \(\sigma\) 无条件截断到 0.5。这不是 Mehrotra 公式；当仿射方向几乎
不能降低 \(\mu\) 时，它会恰好在最需要回中心的迭代中欠中心化。P1 改为
`clamp(sigma, 0, 1)`，legacy A/B 分支保留旧上限。

### 3.2 动态互补缓冲步长

旧实现无论中心性如何都取

\[
\alpha_p=0.9995\alpha_p^{max},\qquad
\alpha_d=0.9995\alpha_d^{max}.
\]

这只保证严格正性，不保证阻塞互补乘积仍处在有用的中心邻域。HiGHS/IPX
实际实现采用与试探互补度相关的缓冲。设在 primal/dual 最大正步长处的平均
互补度为 \(\mu_{full}\)，\(\gamma=0.9\)。若下界间隙 \(g_k\) 阻塞 primal
步，则取

\[
g_k+\alpha_p\Delta g_k =
{(1-\gamma)\mu_{full}\over z_k+\alpha_d^{max}\Delta z_k},
\]

并限制
\(\alpha_p\in[\gamma\alpha_p^{max},1]\)。上界、dual 阻塞完全对称。
因此至少保留 90% 最大步，同时避免阻塞乘积被固定常数推到机器边界附近。

P1 在 `ipm_lp_solver_internal.hpp` 的 `ipm_centrality_step_lengths` 中实现该
方程，fresh/cached 共用。公开选项 `centrality_step_control=false` 精确恢复旧
步长和旧 \(\sigma\) 上限，用于同一二进制 A/B，不作为生产建议。

### 3.3 Gondzio 多重中心校正

Gondzio 校正不重新分解 KKT，而把试探点中过小/过大的
\(g_jz_j\) 投回宽中心邻域，再解零 primal RHS 的校正方向。当前 fresh 路径使用
\([0.1\mu,10\mu]\)，只有总步长得到足够提升才接受；cached 路径尚没有多重
校正。这一不对称必须在节点基准中单独记录。

后续改进不能只比较迭代次数：一次校正增加一次 backsolve。接受标准应同时看
分解次数、backsolve 次数和墙钟时间。

### 3.4 正则化：数值扰动不等于 IP-PMM

当前代码先以

\[
\rho_\mu=\operator{clamp}(10^{-6}\mu,\epsilon_{mach},10^{-2})
\]

形成扰动候选。固定正则下限会产生
\(O(\rho(1+\|(x,y)\|_\infty))\) 的未扰动 KKT 偏差；当对偶变量很大时，
即使线性残差相对 Newton RHS 很小，绝对偏差仍可能阻止对偶收敛。当前实现按
inexact-Newton forcing 条件进一步限制

\[
\rho_k(1+\|(x_k,y_k)\|_\infty)
\le \eta\|F(x_k,y_k,z_k)\|_\infty,\qquad \eta=0.1,
\]

并要求方向的未扰动线性残差同时满足

\[
\|K_k\Delta_k-r_k\|_\infty
\le\min(10^{-9}\|r_k\|_\infty,
         \eta\|F(x_k,y_k,z_k)\|_\infty).
\]

分解失败时仍由动态重试增大正则。该规则只依赖当前 KKT 状态，不依赖案例名、
矩阵维数、带宽或 NETLIB 目标值。它控制数值扰动误差，但仍不构成有完整收敛
理论的 proximal 算法：中心路径、残差和 stopping test 仍对应原 LP，而 Newton
方程对应被扰动系统。

Friedlander--Orban 的原始-对偶正则化及 Pougkakiotis--Gondzio 的 IP-PMM
把 proximal center 显式纳入问题，例如在第 \(k\) 个外层中心处引入

\[
\rho_k(x-\zeta_k),\qquad \delta_k(y-\eta_k),
\]

并规定 center 更新、正则化衰减和内层误差。因此 P3 必须新增持久的
`proximal_center` 状态以及 regularized/unregularized 两套残差，不能继续复用
一个无语义的 `reg` 标量冒充 IP-PMM。

### 3.5 HSD、不可行和无界证书

当前 Native IPM 只在找到最优 KKT 点时可靠；普通路径不能证明 infeasible 或
unbounded。HiGHS/IPX 源码中的发散判定也是目标量级启发式，不是 HSD 证书。

对标准型 primal/dual，引入 \(\tau,\kappa\ge0\) 后，齐次系统可写成

\[
Ax-b\tau=0,\qquad
-A^Ty+c\tau-s=0,\qquad
b^Ty-c^Tx-\kappa=0,
\]

并增加 \(x^Ts+\tau\kappa\) 的互补方程。若 \(\tau>0\)，除以 \(\tau\)
恢复原问题解；若 \(\tau\to0,\kappa>0\)，则从 \((x,y,s)\) 提取 primal 或
dual 不可行证书。

Tulip 的默认 HSD 实现直接维护 \(\tau,\kappa\)，按原数据范数归一化 primal、
dual、relative-gap，并在原模型上验证两类证书。P2 应采用同样的职责边界：

1. bounded-variable HSD 点新增 `tau/kappa`，不能用大 M 松弛代替；
2. KKT 求解器只返回方向，不决定模型状态；
3. 状态判定在原始未缩放 LP 上重算；
4. `SolveResult` 必须区分 `Optimal`、`PrimalInfeasible`、`DualInfeasible` 和
   `NoProgress`，证书向量必须有独立审计函数；
5. presolve/postsolve 必须能恢复证书，否则只能返回 unknown。

### 3.6 KKT 系统和线性代数

normal equations

\[
(A_a\Theta A_a^T+\delta I)\Delta y=r
\]

维数较小且可用 Cholesky，但条件数近似平方。augmented system

\[
\begin{bmatrix}D+\rho I&-A_a^T\\-A_a&-\delta I\end{bmatrix}
\begin{bmatrix}\Delta x\\\Delta y\end{bmatrix}=r
\]

保留原矩阵条件结构，但需要稳定的对称不定/准定 LDLT。当前代码按 normal
matrix 的带宽、预测内存和稠密度选择路径，这个方向合理；缺口在于：

- 路由只看结构，没有看数值条件估计和分解质量；
- cached 路径的正则化、迭代精化和 fresh 路径不完全一致；
- 分解成功只表示后端返回成功，不表示 Newton 相对残差合格；
- normal-equation 精化基于已经加正则的矩阵，augmented 路径另有未扰动精化，
  两条路径的质量门槛需要统一。

P4 已记录每次分解的路径、正则化、Newton 初始/最终相对残差、精化次数和失败
原因。Windows 路径不再按 KKT 维数选择 CHOLMOD/PARDISO：主轨迹使用结构保持
的准定 LDLT；只有完整求解或原模型 postsolve/KKT 审计失败，才从同一初始点
重启带主元的完整 barrier 轨迹。禁止在已分叉的 barrier 轨迹中途更换后端。
主路径的条件估计和跨平台统一仍属于后续 P4 工作。

### 3.7 缩放和停止准则

缩放只改变坐标，不得改变“求解成功”的定义。P1 仍在缩放空间用绝对
`pfeas/dfeas/mu` 停止，最终只审计原空间 primal 可行性；这比成熟实现缺少
两层保护：

\[
\rho_p={\|r_p\|_\infty\over\max(1,\|(b,l,u)\|_\infty)},\quad
\rho_d={\|r_d\|_\infty\over\max(1,\|c\|_\infty)},\quad
\rho_g={|pobj-dobj|\over1+|pobj|+|dobj|}.
\]

HiGHS/IPX 以 bounds/cost measures 缩放 feasibility；Tulip HSD 对
\(b,c,l,u,\tau\) 做显式相对归一化。P2a 已实现 `IPMLPOptimalityAudit`：在
未缩放 `LPModel` 上重算 primal residual、stationarity、原/对偶目标和 relative
gap。缩放空间相对量只负责触发候选检查，只有原模型审计通过才退出并发布
`Optimal`；绝对条件也不再绕过审计。P2 中仍需把该判据扩展到 HSD 的
\(\tau,\kappa\) 和证书状态。

## 4. 成熟实现对照

| 实现 | 可复核机制 | 对 Native 的直接结论 |
|---|---|---|
| HiGHS/IPX | bounded variables、Mehrotra、动态互补缓冲步长、basis preconditioning、crossover | P1 可直接 A/B；IPX 的 infeasibility 判断不能代替证书 |
| HiGHS/HiPO | Schork--Gondzio basis preconditioning，迭代解 normal equations | P4/P5 候选，须先有 Newton 质量遥测 |
| Tulip | 默认 HSD、\(\tau/\kappa\)、多重校正、primal/dual/gap 正则化、normal/augmented 可插拔 KKT | P2 架构参考；证书必须原空间验证 |
| MOSEK | 工业级 homogeneous algorithm、不可行/无界处理 | 支持 P2 的优先级，但闭源细节不能作为代码依据 |
| 本项目 Native | fresh/cached 两路径、Ruiz、Gondzio、normal/augmented、CHOLMOD/Eigen/Accelerate | 数值骨架可复用，状态机和算法正则化仍需重构 |

## 5. 分阶段可证伪实验

### P1：中心路径控制

实验变量只有 `centrality_step_control`。benchmark 键：

```powershell
tests/Release/netlib_solver_benchmark.exe `
  --data-dir tests/data `
  --solvers native-ipm-legacy-step,native-ipm `
  --time-limit 15 --max-iterations 2000
```

接受门槛：

- 90 例 `accurate` 不低于 legacy；
- 当前 13 个失败/不准确案例至少净增 1 个 accurate；
- 共同 accurate 案例的几何平均时间不恶化超过 5%；
- `test_numerical_stability`、`test_netlib_regression` 全通过；
- cached-node 回归通过，不能只验证 fresh 路径。

任一准确案例回退都必须列出并解释。只降低迭代数但准确率下降则否决默认启用。

2026-08-05 的同一 Release 二进制、15 秒总预算实测已满足门槛：modern 为
78/90 accurate，legacy 为 76/90；新增 `israel` 和 `maros-r7`，无 accurate
回退。76 个共同准确案例上，`legacy time / modern time` 的配对几何平均为
1.199；总求解时间 118.79 s 降至 94.56 s。因此
`centrality_step_control=true` 保持默认。原始结果见
`reports/netlib_native_ipm_centrality_ab_full90_2026-08-05.*`。

### P2a：原模型最优性审计与相对停止候选

P2a 不改变 Newton 方程，也不放宽容差。缩放空间的绝对/相对残差只产生候选；
候选点必须在原模型上同时满足 primal、dual stationarity 和 relative gap，
fixed column 的非唯一界乘子由 stationarity 重构。fresh/cached 路径共用同一
审计器，API 的 `relative_primal_residual`、`relative_dual_residual`、
`relative_gap` 和 `dual_objective` 提供审核数据。`native-ipm-direct` 基准键用于
把 Native 本体与 `native-ipm` 的 HiGHS presolve 组合路径分开。

2026-08-05 的阶段结果为 83/90 success、81/90 accurate；该报告只保留为缺口
定位基线。2026-08-06 在严格原模型审计、完整轨迹后端恢复、inexact-Newton
精化和残差驱动正则化合入后，以库默认 `max_iter=200`、单例求解器时限 40 秒、
进程硬时限 50 秒隔离运行 90 例，结果为 90/90 success、90/90 accurate；总
求解时间 47.497 s，中位数 21.717 ms，无超时和崩溃。历史 9 个不准确案例
`boeing2`、`dfl001`、`e226`、`greenbea`、`greenbeb`、`lotfi`、`pilot`、
`vtpbase`、`wood1p` 全部准确。

验收报告为
`reports/netlib_native_ipm_residual_regularization_full90_2026-08-06.*`。此前使用
`max_iter=1000` 的 90/90 结果只证明轨迹最终可收敛，不作为默认配置验收依据。
`greenbea` 在默认上限下连续重复 3 次均通过，原模型 KKT 指标为 primal
`5.051e-12`、dual `3.492e-12`、gap `9.921e-13`。

### P2：HSD

先构造小型解析集：可行有界、primal infeasible、primal unbounded、零目标、free
variable、ranged row、重复/矛盾等式。对每个证书在 long double 审计器中验证
符号、范数和严格负内积。再跑 NETLIB；NETLIB 主要验证最优性能，不能替代
不可行/无界专用集。

接受门槛：解析集状态和证书 100% 正确；90 例 accurate 不低于 P1；共同准确
案例时间几何均值恶化不超过 10%。若 HSD 只提高状态完备性，可接受小幅成本，
但必须记录。

### P3：IP-PMM

固定 HSD 作为控制组，按文献定义实现 center 和正则化更新。核心实验集是退化、
近秩亏和 primal/dual 残差停滞案例，如 `shell`、`lotfi`、`agg*`、`degen*`。
验收指标为失败数、正则化重试数、Newton 残差和总时间，禁止仅以某一案例的
迭代数选参数。

### P4/P5：线性代数与 crossover

按矩阵结构分层：窄带、稀疏 normal、normal 高填充、近秩亏、大型 block
structured。比较 normal、augmented、basis-preconditioned iterative 三条路径；
每条都必须报告 symbolic/numeric factor、backsolve、refinement 和 crossover
时间。最终以 90 例 LP 加 SCUC/MIP 根节点共同决定路由。

## 6. 审核锚点

| 审核内容 | 源码位置 |
|---|---|
| 公开选项和 legacy A/B | `IPMLPOptions::centrality_step_control` |
| 最大正步长和动态互补缓冲 | `ipm_lp_solver_internal.hpp`：`ipm_maximum_step_lengths`、`ipm_centrality_step_lengths` |
| fresh 残差、预测校正、Gondzio | `ipm_lp_solver.cpp`：`NativeIPMLPAdapter::solve_lp_impl` |
| cached 节点路径 | `ipm_lp_solver_cached.cpp`：`solve_cached_node_lp` |
| normal / augmented KKT | `solve_step`、`raw_kkt_solve`、`solve_normal` |
| inexact-Newton 精化与正则误差预算 | `solve_step`、主循环中的 `regularization_budget` |
| 原模型 primal/dual/gap 审计 | `audit_ipm_lp_optimality`、`IPMLPOptimalityAudit` |
| 相对候选与审计后停止 | fresh/cached 主循环中的 `relative_candidate` |
| NETLIB A/B | `netlib_solver_benchmark.cpp`：`native-ipm-legacy-step`、`native-ipm-direct` |

## 7. 主要参考资料

1. S. Mehrotra, *On the Implementation of a Primal-Dual Interior Point
   Method*, SIAM Journal on Optimization 2, 1992,
   DOI [10.1137/0802028](https://doi.org/10.1137/0802028).
2. J. Gondzio, *Multiple Centrality Corrections in a Primal-Dual Method for
   Linear Programming*, Computational Optimization and Applications 6, 1996,
   DOI [10.1007/BF00249643](https://doi.org/10.1007/BF00249643).
3. M. Colombo, J. Gondzio, *Further Development of Multiple Centrality
   Correctors for Interior Point Methods*, 2007,
   DOI [10.1007/s10589-007-9106-0](https://doi.org/10.1007/s10589-007-9106-0).
4. E. D. Andersen, K. D. Andersen, *The MOSEK Interior Point Optimizer for
   Linear Programming: An Implementation of the Homogeneous Algorithm*, 2000,
   DOI [10.1007/978-1-4757-3216-0_8](https://doi.org/10.1007/978-1-4757-3216-0_8).
5. M. P. Friedlander, D. Orban, *A Primal-Dual Regularized Interior-Point
   Method for Convex Quadratic Programs*, 2012,
   DOI [10.1007/s12532-012-0035-2](https://doi.org/10.1007/s12532-012-0035-2).
6. S. Pougkakiotis, J. Gondzio, *Dynamic Non-diagonal Regularization in
   Interior Point Methods for Linear and Convex Quadratic Programming*, 2019,
   DOI [10.1007/s10957-019-01491-1](https://doi.org/10.1007/s10957-019-01491-1).
7. S. Pougkakiotis, J. Gondzio, *An Interior Point-Proximal Method of
   Multipliers for Convex Quadratic Programming*, 2020,
   DOI [10.1007/s10589-020-00240-9](https://doi.org/10.1007/s10589-020-00240-9).
8. L. Schork, J. Gondzio, *Implementation of an Interior Point Method with
   Basis Preconditioning*, Mathematical Programming Computation 12, 2020,
   DOI [10.1007/s12532-020-00181-8](https://doi.org/10.1007/s12532-020-00181-8).
9. M. Tanneau, M. F. Anjos, A. Lodi, *Design and Implementation of a Modular
   Interior-Point Solver for Linear Optimization*, 2021,
   DOI [10.1007/s12532-020-00200-8](https://doi.org/10.1007/s12532-020-00200-8).
10. J. Gondzio, S. Pougkakiotis, J. W. Pearson, *General-Purpose
    Preconditioning for Regularized Interior Point Methods*, 2022,
    DOI [10.1007/s10589-022-00424-5](https://doi.org/10.1007/s10589-022-00424-5).
11. S. Cipolla, J. Gondzio, *Proximal Stabilized Interior Point Methods and
    Low-Frequency-Update Preconditioning Techniques*, 2023,
    DOI [10.1007/s10957-023-02194-4](https://doi.org/10.1007/s10957-023-02194-4).
12. J. Gondzio, *Interior Point Methods in the Year 2025*, EURO Journal on
    Computational Optimization, 2025,
    DOI [10.1016/j.ejco.2025.100105](https://doi.org/10.1016/j.ejco.2025.100105).

实现核对还使用了仓库当前嵌入的 `highs/ipm/ipx`、`highs/ipm/hipo` 源码和
[Tulip.jl](https://github.com/ds4dm/Tulip.jl) 的 HSD/KKT 实现。论文决定算法
语义，源码只用于核对工程映射；二者冲突时必须回到方程和可证伪测试。
