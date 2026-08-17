# 求解器实现与算法审查手册

> 对应工作树：2026-08-05。本文描述实际编译的 `src/engine` 实现，不把历史
> 计划视为已实现能力。源码中的 `AUDIT-NAV` 注释是本文的反向入口。

## 1. 范围与结论

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
        LE/NLE/LP/QP/NLP/MILP/CONIC   HiGHS/Ipopt/SCIP/Gurobi
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
只有在 `available()` 为真时才进入注册表；Gurobi 不参与普通自动回退。

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

- 原生对偶单纯形已有广泛单元测试，但冷启动、极端退化和大型 NETLIB 的覆盖
  仍需在每个平台独立运行；默认 LP kernel 可能是 HiGHS。
- PDLP 是中等精度一阶法，不提供 basis；不能用于要求严格节点证明的场景。
- `NativeNLP` 的固定罚参数不构成一般约束 NLP 的强收敛保证。
- `NativeIPM` 和锥 IPM 的 `unknown` 必须被上层保留，不能改标为 infeasible。
- 原生 B&C 依赖 HiGHS 库编译其主要实现；MINLP 非凸全局性没有保证。
- 性能结论强依赖编译器、BLAS、稀疏后端、线程和数据集。本文不引用已删除的
  历史快照；只有 `testing.md` 中带复现环境的数据可作为当前证据。
