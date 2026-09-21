# 求解器实现与算法审查手册

## Inexact Relaxation Experiments

`GurobiAdapter::solve_relaxation_lp` selects Method2/Crossover0 and an explicit
BarConvTol in [1e-8,1e-2], on a fresh adapter. This result is only a candidate
multiplier/primal vector: callers must independently certify a lower bound and
audit a repaired integer solution. `Optimal` refers to that requested barrier
tolerance; it must never certify pricing or original-unit feasibility.
LP `ObjBound` is not extracted: successful attribute access can return the
unavailable sentinel -1e100 after no-crossover barrier. DualVio/ConstrVio are
reported for inspection, not substituted for original-unit residuals.

Rationale and fixed validation protocol: HySim `docs/modules/market/performance.md`,
2000-node goal. Weak duality L<=z*<=U permits closure at the existing requested
MIP gap after an independently audited repair. Sparse factorization dominates;
early barrier termination predicted20 s versus45.8 s and measured19.979 s at
8 threads, with original repaired residual1.06582e-10 and certified gap0.064%.
No solver guarantee is inferred from a finite primal objective alone.

Ordering/dualization experiments were rejected: explicit BarOrder1 duplicated
the automatic choice, and PreDual1 increased factor work to8.190e12 and timed
out after54.29s/five iterations. Neither override remains in the adapter.
Fixed-integer repair uses the ordinary strict solve_lp entry. See HySim's
performance.md for exact inputs and failed predictions.

For million-column Gurobi pricing, the versioned policy selects fixed8-thread
barrier without crossover; smaller LPs retain single-thread dual simplex.
Both use two fresh solves with an exact full-dual equality gate in the market
caller. Prediction: two concurrent pricing solves <=16 s versus21.8 s sequential;
this is a deterministic representative policy change, not proof of mathematical
dual uniqueness. Exact model/dual repeats and full market audits remain required.

## Deterministic Pricing

Dedicated nonvirtual `solve_pricing_lp` methods retain adapter object layout and
leave generic solve_lp/solve_milp semantics unchanged. HiGHS pricing requires the
embedded backend, resets the global scheduler, uses fresh model state, threads=1,
solver=simplex, simplex_strategy=1, parallel=off, presolve=on, random_seed=0 and
the supplied positive finite time limit. Gurobi pricing resets environment
parameters (including gurobi.env overrides), uses Method=1/Threads=1 below one
million columns and Method=2/Threads=8/Crossover=0 above that threshold, default
Seed=0/presolve, OptimalityTol=1e-8 and the instance time limit. Configured sparse
LP import retains FeasibilityTol=1e-8. There is no solver substitution or objective
perturbation. Parameter failures reject the solve.

Rationale: Method=-1 concurrent LP can return different optimal row duals for a
degenerate ordered LP depending on the winning algorithm. Single-thread dual
simplex selects a repeatable representative for the same backend/version/platform,
not a mathematically unique dual across solvers. References: Gurobi parameter
reference Method/Threads/Seed; HiGHS options simplex_strategy/parallel/random_seed.
HySim's market caller independently resolves the same LP and enforces exact full
row-dual identity plus original-unit feasibility/objective gates; benchmark and
regressions are in its docs/modules/market/performance.md. Generic adapter users
do not automatically receive that caller-level verification.

> 对应工作树：2026-08-05。本文描述实际编译的 `src/engine` 实现，不把历史
> 计划视为已实现能力。源码中的 `AUDIT-NAV` 注释是本文的反向入口。

## 1. 范围与结论

### Gurobi timing

`last_gurobi_solve_timing()` 返回调用线程最近 `solve_milp` 的阶段 wall 秒数，
包括显式 options 的 `solve_lp`（转调相同稀疏入口）。模型导入从入口到 GRBoptimize
之前，optimize 包含延迟更新、预处理、根松弛与搜索，结果提取包含属性/对偶读取和
模型释放。尚未到达的阶段为 null；默认 LP/QP 不提供该计时。每次 MILP 入口重置，
两个并发市场恢复 worker 不共享记录。环境构造不在此范围，由调用方另行计时。
仅增加线程局部记录与自由 getter，不改变 Adapter、Options 或 SolveResult 布局。
计时 O(1)，不修改模型、选项或回调，预期可测开销 <0.1%；不承诺因此提速。
验证由下游 `[recovery_policy]` 检查三阶段非负、未测阶段 null、阶段和不超外层 wall；
完整市场回归与七日证据见下游 performance.md。独立 MIPSolvers 全量回归未在本轮执行。
下游本轮 Release 66 用例/29262 断言和 GUI/API 通过；七日内部优化合计30.2689 s、
导入1.0813 s、环境0.0132 s。ASan构建未链接Gurobi，不作为本适配器的sanitizer证据。
价格复现另有失败门槛：同一LP fingerprint 0x0819db94，由auto并发障碍法和
dual_simplex得到不同最优对偶（最大6.64876 CNY/MWh），目标差4.47e-8、原残差均合格。
同一Build单独重算精确复现两组价格，见下游 recovery-lmp-repeat；计时不改变默认Method，
也不提供唯一/规范化对偶价格选择，不应把不同条件价格误报成调度或守恒不一致。

### Gurobi 请求级参数与稀疏连续模型

`GurobiAdapter(GurobiOptions{time_limit_sec,mip_gap,threads})` 为可选显式构造方式，
参数作用于该实例，不修改进程环境。时限>0，gap在[0,1]，线程0为自动、最大1024；
参数优先于旧MILP环境默认值。无参构造保留既有策略。环境以empty/start初始化，
启动前关闭控制台输出。库未链接或许可证不可用时available=false，不替换后端。

RATIONALE: 对同一LP/MILP矩阵作恒等传递，显式参数实例的`solve_lp`复用现有
`solve_milp`稀疏装配，整数索引为空，Gurobi仍求解连续LP。转换及复制O(nnz+m+n)，
不采用原LP入口的m*n系数探测；不承诺优化算法速度改善。区间行展开的上/下界Pi
按原行求和，等式Pi按原等式位置恢复。仅最优连续解导出Pi，限时解不作价格证据。
显式MILP参数使用MIPGapAbs=0、FeasibilityTol/IntFeasTol=1e-8，为下游原单位1e-6
审计留余量；并不放宽模型。依据Gurobi C API `TimeLimit/MIPGap/Threads`、
`Status/SolCount/Pi`语义及下游Southern执行契约。

限额状态不以SolCount替代状态：有incumbent则返回候选供调用方审计，无候选则失败。
优化API错误单独返回错误码；不可行、无界、不可行或无界区分。动态网络行分离路径
不导出不完整连续对偶。QP仅继承实例环境时限/线程等参数，未改其装配及证据范围。

预注册验证：下游`HybridACDCDistributionSystemsSimulation`的
`test_southern_market '[gurobi]'`，解析出力100MW、日费用480000、节点价200，
原模型残差<=1e-6；多资源目标与HiGHS差<=1e-4元（相对比较1e-5）；
`min x+2y, 2<=x<=3, y=4`应得目标10、区间行/等式对偶1/2，误差<=1e-8。
极小LP时限应返回TimeLimit且不导出Pi。构造无效参数显式拒绝。
本地Release实测及GUI证据集中在下游市场执行契约；未宣称大规模时延或库全量回归。

本次下游macOS arm64 Release、Gurobi13.0实测：专项2用例41断言通过（含真实
TimeLimit与对偶为空），解析数值及区间行Pi符合预期；Southern/forecast共38用例
23766断言通过，四项浏览器E2E通过。无Gurobi的ASan/UBSan配置校验1用例9断言
通过，验证available=false及无状态推进，不算Gurobi库的sanitizer覆盖。

MIPSolvers 使用统一模型变体和适配器注册表承载八类问题：线性方程（LE）、
非线性方程（NLE）、线性规划（LP）、二次规划（QP）、非线性规划（NLP）、
混合整数线性规划（MILP）、混合整数非线性规划（MINLP）和锥规划（CONIC）。

实现分成三层：

```text
api::ProblemVariant + api::SolveOptions
               |
               v
SolverEngine -> StrategyDispatcher -> PresolveManager
                                      |
                                      v
                              SolverAdapter 注册表
                         / 原生内核       \ 外部适配器
                        v                  v
        LE/NLE/LP/QP/NLP/MILP/CONIC   HiGHS/Ipopt/SCIP/Gurobi/CPLEX
                        |
                        v
              PostsolveManager -> api::Result
```

`SolverEngine` 负责注册和公共 API，`StrategyDispatcher` 负责候选排序、显式
求解器选择、预处理及后处理。真正的迭代算法位于 `kernel/` 或
`solver/native/`，外部适配器只完成模型转换、调用和结果映射。

### 1.1 能力矩阵

| 名称 | 问题类 | 算法/角色 | 关键实现 |
|---|---|---|---|
| `NativeLinear` | LE | 稀疏直接法 | `native_adapters.cpp::solve_le` |
| `NativeNewton` | NLE | 正则化 Newton + 回溯 | `native_adapters.cpp::solve_nle` |
| `NLESolver` | NLE | Dogleg 信赖域 Newton | `nle/nle_solver.cpp` |
| `NewtonSolver` | 电力潮流方程 | AC/DC Newton、PV/PQ 切换 | `nle/newton_solver.cpp` |
| LP kernel | LP | HiGHS 或原生对偶单纯形 | `lp_kernel/dual_simplex.cpp` |
| `NativePDLP` | LP | 一阶原始-对偶混合梯度 | `lp/pdlp_solver.cpp` |
| `NativeIPMLP` | LP | 原始-对偶障碍法 | `ipm/ipm_lp_solver*.cpp` |
| `NativeLCQP` | LP/QP | 凸 QP 原始-对偶内点法 | `ipm/lcqp_solver.cpp` |
| `NativeNLP` | NLP | 二次罚函数 Newton | `native_adapters.cpp::solve_nlp` |
| `NativeIPM` | NLP | 滤子/优值函数原始-对偶 IPM | `ipm/ipm_solver.cpp` |
| `NativeConicIPM` | CONIC | Mehrotra + NT 缩放 | `ipm/conic_ipm_solver.cpp` |
| `NativeBranchAndCut` | MILP/MINLP | 分支定界、割、启发式 | `milp/bc/` |
| `StrictHiGHS` | MILP | HiGHS MIP 状态机策略 | `native_adapters.cpp` |
| `HiGHS` | LP/MILP | 进程内或命令行外部后端 | `external/adapters.cpp` |
| `Ipopt` | NLP | Ipopt TNLP 适配 | `external/adapters.cpp` |
| `SCIP` | MILP/MINLP | MPS/PIP 适配 | `external/adapters.cpp` |
| `Gurobi` | LP/QP/MILP | 原生 C API 适配 | `external/adapters.cpp` |
| `CPLEX` | MILP | 原生 Callable Library 适配 | `external/adapters.cpp` |

注意：`StrictHiGHS` 不是第二套自研 B&C。它通过
`make_strict_highs_production_options` 强制 HiGHS MIP 合约；
`NativeBranchAndCut` 才进入项目的树搜索框架，但其默认节点 LP 仍可选择 HiGHS。

## 2. 公共模型与调度

### 2.1 模型数据结构

模型定义集中在 `include/mipsolvers/engine/problem_types.hpp`：

- `SparseLinSys`：稀疏方阵 `A` 和右端 `b`。
- `NonlinearSystem`：维数、初值，以及原地写入的残差/Jacobian 回调。
- `LPModel`：不等式 `A x <= b`、等式 `Aeq x = beq`、线性目标 `c`、变量元数据
  和目标方向。
- `QPModel`：在 LP 约束上增加稀疏对称 Hessian `Q`，目标为
  `0.5 x'Qx + c'x`。
- `NLPModel`：目标、梯度、Hessian、等式 `g(x)=0`、不等式 `h(x)<=0` 及稀疏
  Jacobian 回调，并携带 NLP 选项和初值。
- `MIPModel`：`LPModel linear_part` 加二进制/整数列索引、优先级、初始解和
  可选 SCUC 元数据。
- `MINLPModel`：`NLPModel nonlinear_part` 加整数列索引。
- `ConicModel`：CVXOPT 标准型 `min c'x, Gx+s=h, Ax=b, s in K`，`ConeDims`
  描述非负、二阶和半正定锥块。

稀疏矩阵以 Eigen CSC 为公共表示。LP 单纯形内部的 `StandardColumnMatrix`
根据维数/非零元数量在 32 位与 64 位索引间自适应；`StandardRowMatrix` 保存
CSR 行访问索引和按行连续的值副本，供 PRICE、行活动与割分离热循环使用。

### 2.2 结果契约

`SolveResult` 保存原始变量 `x`、变量/约束对偶、约化成本和 `SolveStats`。
`SolveStats` 的 `success` 表示通过该适配器成功终止，不等价于所有状态都为
“最优”；审核时还必须检查 `status`、原始/对偶可行度、互补度、目标界和 gap。
公共 API 最后映射为 `api::Result`。

### 2.3 调度流程

入口为 `src/engine/api/solver.cpp::SolverEngine::solve`：

1. 规范化公共模型并执行结构/有限值校验。
2. `StrategyDispatcher::candidate_adapters` 按问题类型和 `solver_name` 构造候选。
3. 对允许的模型执行预处理；保留恢复原变量所需映射。
4. 调用适配器的类型专用虚函数。
5. 根据策略决定是否尝试下一个后端；显式指定后端时不应静默冒充另一个后端。
6. 后处理恢复变量、目标和状态，返回统一结果。

默认注册顺序可直接在 `SolverEngine::register_default_adapters` 审核。可选外部库
只有在 `available()` 为真时才进入注册表。Gurobi 和 CPLEX 不在各问题类的默认
优先级列表中，但启用且 `allow_fallback=true` 时，仍会在所有具名默认候选失败后
作为 registry 余项参与末位回退；也可通过 `preferred_solver` 或
`set_solver_preference` 显式选择。

### 2.4 CPLEX MILP adapter

`CplexAdapter` uses `CPXcopylp` to import the original column-compressed
linear model, expands each finite side of a ranged row independently, applies
integer and binary types with `CPXcopyctype`, and calls `CPXmipopt`. Objective
sense and variable order are preserved. `CplexSolveInfo` exposes per-thread
import/optimize/extraction timings plus status, incumbent availability, best
bound, relative gap and node count.

The adapter is opt-in at build time and is not in the named automatic MILP
priority list. In an enabled build it remains eligible as a final registry
fallback after those candidates fail. Its public scope is linear MILP; LP, QP,
semi-continuous and semi-integer interfaces are outside the current adapter
contract. One adapter instance owns one CPLEX environment and is not safe for
concurrent calls. The mathematical mapping, platform measurements and fixed
validation protocol are recorded in
[`cplex_callable_library.md`](cplex_callable_library.md).

## 3. 公共线性代数与 KKT

### 3.1 稀疏线性求解器

`SparseLinearSolver` 统一三阶段接口：`analyze_pattern` 只分析结构，
`factorize` 更新数值分解，`solve` 处理一个右端。实现包括 Eigen SparseLU、
UMFPACK、KLU、SuperLU、MKL Pardiso 和 MUMPS；
`make_default_sparse_solver` 根据编译能力选择后端。

同一稀疏模式的迭代算法必须复用 symbolic analysis。空的 `0x0` 系统有显式
成功路径；失败、非有限解和残差不下降不得被改写为成功。

### 3.2 KKT 系统

`kkt_system.cpp` 组装 NLP 的增广系统，缓存原始矩阵、正则化矩阵、稀疏模式和
分解器。流程为：

1. 比较精确 outer/inner 索引判断模式是否可复用。
2. 组装 Hessian、等式 Jacobian 与对角正则化。
3. 先作符号分析，再作数值分解；需要时增加正则化。
4. 求 Newton 方向并在原始、未正则化系统上算残差。
5. 迭代改进只提交严格降低无穷范数残差的候选方向。
6. 将解拆成原变量方向与乘子方向，并报告惯性供非凸修正判断。

## 4. 线性方程与非线性方程

### 4.1 `NativeLinear`

理论问题是 `A x = b`。实现先校验维数，再分析稀疏模式、数值分解、求解，最后
计算 `||Ax-b||_inf`。这是直接法封装，没有迭代停止准则；`iterations=1` 仅表示
一次直接求解。

审核重点：默认后端选择是否符合部署配置、非方阵/奇异矩阵状态是否真实、结果
残差是否使用原问题计算。测试见 `test_engine_api.cpp` 的各线性后端用例。

### 4.2 `NativeNewton`

目标是求 `F(x)=0`。第 `k` 步近似解

```text
(J(x_k) + lambda I) d_k = -F(x_k),    x_{k+1}=x_k+alpha d_k.
```

实际流程：

1. 计算残差无穷范数并检查有限性/收敛。
2. 取得稀疏 Jacobian，验证为 `n x n`。
3. 最多四次增大对角正则化并重新分解。
4. 对 Newton 方向回溯，要求试点残差不增。
5. 无可接受方向、线性求解失败或达到迭代上限时返回明确状态。

`NLESolver` 是另一条领域无关入口，使用共享 globalization 组件中的 Dogleg
信赖域；`NewtonSolver` 则是电力潮流专用实现，额外维护 Jacobian 模式缓存、
AC/DC 状态、PV/PQ 控制切换、无功限值和 Levenberg-Marquardt 恢复。三者不可
混为同一算法。

## 5. LP 求解器

### 5.1 LP 内核分派

`solve_lp_with_basis` 把 `LPModel` 转为带变量上下界的等式标准型，并按
`LpKernelBackend` 进入 HiGHS 或原生内核。`BasisState` 保存基列、非基变量所在
边界、稀疏因子状态和标准型映射，用于节点 LP、改界重优化和添加割后的热启动。

标准型转换必须同步维护目标方向、变量平移、行符号、松弛列和原始空间恢复；
证书和对偶也必须经过相同映射。`dual_simplex_api.cpp` 是公共入口，
`dual_simplex.cpp` 负责转换/后端分派，`native_dual/` 才是原生算法。

### 5.2 原生对偶单纯形

对标准型最大化问题，基矩阵为 `B`，基本值 `x_B=B^{-1}b`，对偶乘子
`pi=B^{-T}c_B`，约化成本 `d=c-A'pi`。对偶单纯形保持约化成本满足边界符号
条件，逐步修复基本变量的原始不可行性。

核心状态：

- `SolverState`：基/非基状态、基本值、约化成本、边界、权重、容差和统计。
- `FactorState`：HFactor/Eigen 稀疏基分解、FTRAN/BTRAN、更新链和重分解计数。
- `StandardFormLP`：CSC/CSR 双视图、上下界、目标和原始空间映射。
- `BasisHint`/`BasisState`：跨节点或重复求解的事务性热启动。
- `DualInfeasibilitySummary`、cost journal、cycle history：稳定化、稀疏回滚和
  反循环状态。

主流程位于 `native_dual/solver.cpp::solve`：

1. 构造或导入基；修复无效、越界或秩亏基。
2. 冷启动通过 cost shift/dual Phase I 获得对偶可行状态；必要时进入原始
   Phase I。
3. `pricing.cpp` 选择原始不可行的离基行，DSE/Devex 权重控制选择质量。
4. BTRAN 得到枢轴行；PRICE 计算候选列与方向。
5. BFRT 比率检验可先翻转一段非基变量边界，再选入基列。
6. FTRAN 得到枢轴列，更新基本值、约化成本、边界状态和权重。
7. 通过 Forrest-Tomlin 更新维护基因子；到达更新阈值或数值异常时 INVERT。
8. 周期检测通过 tabu/重建/扰动处理；残差审计失败会重建而非直接宣告最优。
9. 原始与对偶可行时清除 cost shift，重算原问题残差并发布最优结果。
10. 无候选列时区分不可行证书；原始方向无界时发布原始射线。

`primal.cpp` 提供原始 Phase I/原始定价补充路径；`certificate.cpp` 对不可行证书
和无界射线做原问题审计。所有增量改界/加行操作使用提交或回滚语义，失败后
不得污染持久基状态。

### 5.3 `NativePDLP`

PDLP 是不分解 KKT/基矩阵的一阶法。实现将行上下界统一为区间约束，构造 CSR，
用 Ruiz 风格缩放改善尺度，迭代原始/对偶变量并用加权平均点检查终止。

抽象更新为：

```text
x^{k+1} = projection_[l,u](x^k - tau(c + A' y^k))
y^{k+1} = projection_row_dual(y^k + sigma A(2x^{k+1}-x^k))
```

实现还包含预条件步长、primal weight 自适应、重启和原始/对偶目标差检查。
流程是：校验并建 CSR -> 缩放 -> 初始化步长/平均量 -> 稀疏矩阵向量乘更新 ->
周期性计算可行度与 gap -> 重启或终止 -> 反缩放并审计结果。

它适合超大稀疏、较低精度 LP，不产生单纯形基，因而不能直接替代依赖基热启动
和精确约化成本的 B&C 节点内核。

### 5.4 `NativeIPMLP`

专用 LP 内点法在变量界和线性约束上维护原始变量、松弛与对偶，求解 Mehrotra
预测-校正 Newton 系统。关键数据包括缩放后的矩阵、各类残差、界索引、Schur/
增广 KKT 缓冲区，以及 `CachedState` 中用于批量节点改界的模式、分解和工作区。

主流程：预处理/变量变换 -> Ruiz 缩放 -> 严格内部初值 -> 计算原始、对偶和
互补残差 -> 组装 normal equation 或增广系统 -> 仿射方向 -> 估计
`mu_aff` 与标准 Mehrotra 中心参数 -> 校正方向 -> 由试探互补度决定的动态
primal/dual 缓冲步长 -> Gondzio 校正 -> 更新 -> 相对候选检查 -> 反缩放并在
原模型上审计 primal、dual stationarity 和 relative gap。缩放条件只产生候选，
不能绕过原模型 KKT 审计发布 `Optimal`。
`IPMLPOptions::centrality_step_control=false` 只用于复现旧的固定 `0.9995`
步长和 `sigma<=0.5` A/B，不建议生产使用。

线性代数路径按结构选择带状 Cholesky、CHOLMOD、Eigen LDLT 或增广系统后端，
并在固定模式上复用符号分析。`ipm_lp_solver_cached.cpp` 专门处理 B&C 重复节点：
只有上下界/目标改变时复用结构，事务签名不匹配则退回完整求解。

该求解器当前仍没有 HSD 状态机，不能从普通迭代失败推出不可行或无界。现代
IPM 文献、当前差距、P1 全量 A/B 和后续 HSD/IP-PMM 验收见
[Native LP 内点法理论设计](native_ipm_design.md)。

### 5.5 `NativeLCQP`

求解凸 QP：

```text
min 0.5 x'Qx + c'x
s.t. Aeq x = beq,  l <= x <= u
```

不等式 LP 会先转换成带松弛/边界的形式。每次迭代形成
`Q + diag(z_l/s_l + z_u/s_u)` 与等式约束组成的 KKT，保持模式不变，仅更新
对角和数值分解。算法计算可行度、驻点残差和平均互补度，选择自适应中心参数，
恢复松弛/对偶方向并取正性步长。`Q` 非凸时本实现没有全局最优保证。

## 6. NLP 求解器

### 6.1 `NativeNLP`

这是轻量二次罚函数 Newton 法，而非完整 SQP/IPM。优值函数为

```text
phi(x)=f(x)+rho/2 ||g(x)||^2 + rho/2 ||max(h(x),0)||^2.
```

实现用 `H + rho Jg'Jg + rho Jh_active'Jh_active + delta I` 近似 Hessian，求
Newton 步并投影到变量界，回溯要求 `phi` 不增。固定 `rho` 意味着约束很难时
可能停在罚函数驻点；“步长很小”状态必须结合可行度审核。

### 6.2 `NativeIPM`

该实现面向一般约束 NLP，维护 `x`、等式乘子、正松弛和不等式乘子。默认使用
Wachter-Biegler 型滤子全局化，也可选 Mehrotra 风格优值函数路径。

关键结构：

- `NLPState`：原始/对偶/松弛迭代点。
- `DiagonalQNState`：没有精确 Hessian 时的分块对角拟 Newton 曲率。
- `AugmentedNewtonCache`：Hessian/Jacobian 模式、KKT 缓冲、分解器和惯性。
- `IterateSnapshot`、`TrialPoint`：回溯和二阶修正的事务性试点。
- `Filter`：记录不可行度 `theta` 与目标/障碍值 `phi` 的支配关系。

滤子路径：

1. 规范化初值和界，必要时缩放问题；构造正松弛/乘子。
2. 计算 KKT 残差；满足严格或 acceptable 的完整 KKT 条件才成功。
3. 取得精确 Hessian，或更新拟 Newton 曲率。
4. 组装凝聚或增广 Newton 系统，利用惯性检查并增大 Hessian 正则化。
5. 解原始/对偶方向；按 fraction-to-boundary 限制正变量。
6. 滤子接受降低约束违反或取得足够目标下降的试点。
7. 需要时做二阶修正；线搜索失败则进入可行性恢复阶段。
8. 接近中心路径后降低障碍参数，直到原始、对偶和互补条件同时满足。

外部回退默认关闭；显式启用时失败可调用 Ipopt，最终 `solver_name` 必须揭示
实际路径。

## 7. 锥规划 `NativeConicIPM`

锥模型为

```text
(P) min c'x,  Gx+s=h, Ax=b, s in K
(D) max -h'z-b'y, G'z+A'y+c=0, z in K
K = R_+^l x product Q_i x product S_+^j.
```

`ConeDims` 给出块布局，`ConeLayout` 计算每块偏移与锥度；SDP 使用保持 Frobenius
内积的 `svec/smat`。`ConeNtScaling` 分块保存 Nesterov-Todd 缩放，
`KktBackend` 缓存 Schur/准定系统的稀疏结构。

算法流程：

1. 校验锥维数；对足够大的稀疏 PSD 块尝试精确弦分解。
2. 取 `x=0,y=0,z=e`，把 `s=h` 沿单位元平移到锥内部。
3. 计算原始残差、对偶残差、gap 和齐次证书条件。
4. 计算 NT 缩放并分解约化 KKT。
5. 解仿射预测方向，求可行最大步和 `mu_aff`，令
   `sigma=(mu_aff/mu)^3`。
6. 加入中心化及二阶交叉项，复用同一分解解校正方向。
7. 对 SOC/SDP 方向可作 3x3 原系统迭代改进，取阻尼锥内步长并更新。
8. 同时满足可行度和绝对/相对 gap 才返回 `optimal`；否则区分原始不可行、
   对偶不可行或 `unknown`。
9. 弦分解路径恢复原 SDP 变量后重新计算原问题残差，不合格结果降级为
   `unknown`。

更完整的锥代数推导见 [锥规划专题](conic_sdp.md)。

## 8. MILP/MINLP 分支割

### 8.1 数据结构

- `BCOptions`：时间/节点/LP 预算、节点选择、分支、割、启发式、并行和证明开关。
- `BCSolveState`：一次求解的模型、预处理映射、根 LP、树、incumbent、池和统计。
- `BCNode`/紧凑域：节点界变化、深度、父界、局部割签名；静止时保持稀疏。
- `Domain`/trail：增量界、行活动、传播原因和可精确回滚的事务栈。
- `CutPool`：稀疏割、有效范围、年龄、活性与去重信息。
- `PseudoCosts`、reliability observations：上下分支收益及样本可信度。
- `CliqueTable`/implication graph：二进制互斥、蕴含和冲突传播。
- `BCWarmStart`、`BCCallbacks`：incumbent、伪成本、ML 分支先验和运行后回调。

### 8.2 主流程

`BCSolveState::run` 被拆成 14 个按执行次序编号的 `.inc` 文件；编号本身就是
主算法的审查顺序：

1. 校验整数索引，安装环境快照和截止时间，执行 PaPILO/HiGHS/native presolve，
   建立恢复映射。
2. 构造根 LP，选择 HiGHS、原生对偶单纯形或 IPM 根路径。
3. 审核根解、基和界；失败时按预算走无预处理/数值升级回退。
4. 运行根传播、探测和割分离，多轮重解直到停滞或预算耗尽。
5. 尝试 warm start、rounding、feasibility pump、diving、RENS/LNS 等原始启发式；
   每个候选都在原模型上验证后才发布 incumbent。
6. 初始化节点队列、伪成本、冲突池、共享 incumbent 和并行状态。
7. 弹出节点并应用域事务；传播空行、活动界、clique/implication 和冲突。
8. 解节点 LP；不可靠的扰动/回退结果不能形成原问题证明。
9. 用不可行证书、节点界、incumbent cutoff 和 gap 进行证明安全的剪枝。
10. 在需要时分离 GMI、MIR、cover、clique、implied-bound、CGLP 及项目特定割；
    记录全局/根局部/节点局部作用域。
11. 对分数整数变量执行 reliability branching：强分支样本优先，伪成本预测补充，
    再叠加静态优先级和动态 ML prior。
12. 创建上下子节点，精确记录界变化、局部割签名和父界，更新伪成本。
13. 串行路径维护 DFS/最佳界队列、重启、incumbent 启发式和证明前沿。
14. 并行路径通过共享 incumbent、工作窃取和可选确定性令牌协调；收尾统一目标界、
    gap、后处理解与统计。

### 8.3 割与证明边界

通用割包括 GMI/MIR、cover、clique、蕴含界和可选 lift-and-project；另有 SCUC
专用割。审核每个割时必须确认：变量空间（原始/预处理）、有效范围、数值清理、
当前点违反量、去重键以及是否错误进入全局池。仅对 incumbent cutoff 有效的割
不能作为无条件全局约束。

节点 LP 的“不可行”只有在原模型尺度上通过可信后端/证书审计后才能剪枝；扰动
回退只用于寻找候选，不能单独证明。并行线程必须以同一快照读取 incumbent 的
解和目标，避免撕裂读导致不同剪枝决定。

### 8.4 MINLP 边界

MINLP 入口复用 B&C 外框，并在连续松弛处调用 NLP 路径。一般非凸 MINLP 不具备
全局下界保证，因此除非连续松弛可证明为凸且返回可信界，不能把局部 NLP 最优
直接当作全局 B&B 证书。调用者应把该路径视为实验性局部求解能力。

## 9. 外部适配器

- HiGHS：优先使用进程内库；部分配置保留 MPS/solution 临时文件路径。映射
  行上下界、变量界、目标方向、basis/dual 和模型状态。
- Ipopt：`CallbackTNLP` 把 `NLPModel` 回调映射到 Ipopt TNLP，统一等式/不等式
  和界乘子符号，并在返回后计算项目的 KKT 诊断。
- SCIP：MILP 写 MPS、MINLP 写 PIP，启动可执行文件并解析 solution；临时文件
  在所有退出路径清理。
- Gurobi：编译可用时通过 C API 直接建模，覆盖 LP/QP/MILP；SCUC 可安装 lazy
  network cuts 和诊断回调。许可证不可用时不得注册为自动候选。

外部适配器的审核重点不是算法内部，而是模型等价性、无限界约定、目标符号、
状态翻译、时限传递、临时文件安全和最终残差复核。

## 10. 预处理、后处理与公共策略

`PresolveManager` 是公共轻量入口；MILP 主路径还可用 PaPILO、HiGHS side state
和原生 `MILPPresolve`。原生规则包含空行/不可行检测、固定列、单/双元替换、
行活动界传播、探测、整数可除性和并行行处理。

每一次删除/替换都要写入 postsolve 映射。恢复后必须在原模型检查：变量界、
等式/不等式、整数性和目标。若根求解在预处理模型失败，回退到原模型时不能
复用不相容的基、割或列索引。

`warmstart.cpp` 负责通用初值；`highs_presolve_side_state.cpp` 管理 HiGHS
presolve/postsolve 与缓存生命周期；`papilo_presolve.cpp` 负责 PaPILO 模型映射。

## 11. 测试覆盖与审核顺序

当前源码含 328 个 Catch2 `TEST_CASE`，分布在 API、LP、对偶单纯形、IPM、锥、
MILP、预处理、数值稳定性、NETLIB、SCUC、AML 和 L2O。实际构建是否包含这些
目标取决于 CMake 依赖和选项，不能用源码用例数代替通过数。

建议代码审核顺序：

1. `problem_types.hpp`、`solver_adapter.hpp`、`api/solver.cpp`、`dispatcher.cpp`。
2. `linear_solver.hpp/.cpp`、`kkt_system.hpp/.cpp`。
3. LP：`dual_simplex_api.cpp` -> `dual_simplex.cpp` -> `native_dual/solver.cpp` ->
   factor/pricing/primal/certificate。
4. IPM：相应公开头 -> 主 `solve` -> KKT/缩放/恢复模块。
5. MILP：`bc/api.cpp` -> `branch_and_cut.cpp::BCSolveState::run` -> 14 个 `.inc` ->
   relaxation/branching/cuts/domain/parallel/validation。
6. 外部适配器最后审核，因为它们依赖前述模型与结果契约。

可复现的当前测试结果见 [测试与基准结果](testing.md)。

## 12. 已知限制

Gurobi 的逐实例 `GurobiOptions.method` 对应原生 `Method`（-1..5，默认 -1），
在构造时校验并应用于模型环境。显式选 2 使用专用障碍法，避免大 LP 的并发算法
分摊线程；选 1 使用对偶单纯形。不改变稀疏矩阵、目标、对偶行映射或可行性容差。
本次性能假设与验收记录在相邻 HySim 的
`docs/modules/market/southern_execution_contract.md`：2000 节点 / 98 点 / 4线程，
默认并发根节点的障碍法仅1线程，预测专用障碍法根 LP 耗时减少30%，待实测闭环。
适配器手算 LP 的 Method 1/2 均得到目标10、区间行对偶1、等式行对偶2（50项
Gurobi专项断言通过）；这不构成大规模性能或其他求解器回归的证明。

`GurobiOptions.crossover` 对应原生 Crossover（-1..4，默认-1）；纯障碍法 LP
可选0跳过基恢复，保留最优X/Pi用于原/对偶审计，不用于需要单纯形基的调用方。
模型及行映射不变；固定验收为无 crossover 工作且手算区间/等式对偶仍为1/2。
HySim的整数根节点继续保留默认基恢复，只有纯LP定价与初解启用0。

- 原生对偶单纯形已有广泛单元测试，但冷启动、极端退化和大型 NETLIB 的覆盖
  仍需在每个平台独立运行；默认 LP kernel 可能是 HiGHS。
- PDLP 是中等精度一阶法，不提供 basis；不能用于要求严格节点证明的场景。
- `NativeNLP` 的固定罚参数不构成一般约束 NLP 的强收敛保证。
- `NativeIPM` 和锥 IPM 的 `unknown` 必须被上层保留，不能改标为 infeasible。
- 原生 B&C 依赖 HiGHS 库编译其主要实现；MINLP 非凸全局性没有保证。
- 性能结论强依赖编译器、BLAS、稀疏后端、线程和数据集。本文不引用已删除的
  历史快照；只有 `testing.md` 中带复现环境的数据可作为当前证据。

### Fixed-Integer Root Repair Kernel and Budget

Rationale before implementation: for integer preferences z, domain propagation
produces local bounds l(z),u(z). Repair solves the unchanged LP
`min c'x : A*x<=b, Aeq*x=beq, l(z)<=x<=u(z)`; switching the LP kernel does not
alter that subproblem. Acceptance still audits every bound/row and integrality
before adoption, and local domain bounds are restored on every exit. A failed
repair never proves infeasibility of the original MILP.

HySim IEEE118 profiling found an automatic `use_ipm_root` size flag dispatching
fixed-integer repair into NativeIPM even though the selected node kernel was
HiGHS. Its cached IPM had no remaining wall budget;1368/1465 main-thread samples
were in NativeIPMLPAdapter, with about2.2GB footprint. The repair dispatcher must
honor the same vendored-HiGHS condition as solve_lp_relaxation, passing the
remaining global budget less the existing finalization reserve. This uses a
local sparse LP copy (O(nnz+n) memory/traffic), then the existing HiGHS LP path
with its internal remaining-time checks. Other LP kernels retain their current
dispatch. Preference attempts and flip loops check the optional-root deadline.
Checks cannot preempt an in-progress sparse factorization or domain traversal.

Prediction: on this118 fixture fixed-integer LP repair below3s and full market
pipeline below15s (at least50% reduction relative to the prior roughly37s
failure), with a verified incumbent within the30s request. This assumes fixed
integer preferences have a feasible continuous completion; failure is evidence
to revisit that assumption, not justification to relax model constraints.
References: the LP above; solve_lp_relaxation in bc_relaxation.cpp;
try_fixed_integer_repair in bc_run/06_root_heuristics_a.inc; existing root
deadline/finalization reserve in bc_run/01_setup_presolve_root_build.inc.
Validation: HySim [native_repair] for the full11898-point chain at gap0.01,
original residual<=1e-6 and objective within1% of the verified17413231.016942CNY
incumbent; root-cut/market regression, focused ASan/UBSan, native B&C regression,
and sequential default/enhanced30s runs with120s external watchdog. Exact
commands, build provenance and measured results are recorded in HySim's
southern_execution_contract.md under Native Fixed-Integer Repair Debug.

First measurement exposed a second implementation mismatch: repair took0.397s
and produced an audited incumbent at2.435s, but the full chain took45.103s.
The root polish flag was initialized incomplete for every HiGHS LP kernel,
even with auto_highs_root_pipeline=false; it blocked root_gap_closed until late
root processing. Incumbent closure also skipped its expensive fixed-point hook
only for near-exact equality, ignoring an already-satisfied requested MIP gap.
Thus repair prediction held, but the pipeline prediction failed due to control
flow, not an infeasible integer preference. Correction: mark polish complete
when that pipeline is disabled, and bypass optional incumbent closure in this
mode when a validated incumbent U and rigorous root bound L satisfy
`max(0,U-L)/max(1,abs(U)) <= gap_tol`, unless explicit tree exhaustion is required.
The full HiGHS root pipeline and strict tree-exhaustion mode retain their rules.
The original below15s/full-feasibility acceptance target is unchanged.

After both corrections HySim's first Release full chain was6.401s (SCUC3.532s),
with objective17413261.53763228, audited gap1.75273e-6 and valid pricing. This
measurement overlapped compilation; controlled alternating profiles are stored
separately under HySim `output/market-operation/native-repair-fixed/` and
reported in its execution contract. Current-source native B&C regression
passes35 cases/490 assertions; HySim Southern passes41/22597, and explicit
Native IEEE118 GUI/API E2E passes seven stress days plus forecast-resolution
experiments. The unoptimized ASan/UBSan cache times out at the30s Release budget;
an explicit180s test-only budget is used for instrumentation validation. No
2000-node Native performance conclusion follows from this118-node repair.

Final isolated HySim alternating default/enhanced/enhanced/default full-chain
times6.2894/6.3867/6.2885/6.2757s (SCUC3.485..3.585s) satisfy the original below15s
prediction; all original/reconstructed residuals<=1e-6, gap1.7527268e-6,
1 incumbent,3 SCUC LP solves and0 cuts/nodes. This is repair/termination speedup,
not a cut-strengthening result. Explicit180s-per-solve ASan/UBSan validation
passes2 cases/62 assertions with leak detection disabled and no sanitizer
report; the failed30s and successful180s logs are both retained in HySim's
`native-repair-fixed/` artifacts. No full CTest or pinned-dependency build claim.
