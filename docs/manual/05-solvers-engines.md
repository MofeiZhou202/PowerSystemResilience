# 第 5 章 求解器与引擎架构

> 本章整合自: docs/archive/solvers.md, docs/archive/engine.md, docs/archive/native_ipm_design.md, docs/archive/lp_kernel_selector_2026-08-11.md

本章回答一个工程问题：**我的问题该用哪个求解器、哪些参数必须动**。
先给选择结论（§1–§3），再讲统一调用接口（§4），随后是各求解器的
数学原理要点（§5）、关键选项与终止判据（§6）和数值保护设计（§7）。

## 1. 求解器能力总览

MIPSolvers 用统一模型变体和适配器注册表承载八类问题：LE（线性方程）、
NLE（非线性方程）、LP、QP、NLP、MILP、MINLP、CONIC。实现分三层：

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

完整能力矩阵（名称即 `SolveOptions::preferred_solver` 可填的字符串，
大小写敏感）：

| 适配器名 | 问题类 | 算法/角色 | 返回对偶 |
|---|---|---|---|
| `NativeLinear` | LE | 稀疏直接法 | N/A |
| `NativeNewton` | NLE | 正则化 Newton + 回溯 | 否 |
| `NativeIPMLPAdapter` | LP | Mehrotra 预测-校正 IPM | 是 |
| `NativePDLPAdapter` | LP | 一阶原始-对偶混合梯度（PDLP） | 否 |
| 原生 dual simplex | LP | 对偶单纯形（经 `NativeAutoLPAdapter`/B&C 节点调用） | 是 |
| `NativeLCQPAdapter` | QP | 凸 QP 原始-对偶内点法 | 是 |
| `NativeIPMAdapter` | NLP | 滤子/优值函数原始-对偶 IPM | 否 |
| `NativeNLPAdapter` | NLP | 二次罚函数 Newton（轻量） | 否 |
| `NativeConicIPM` | CONIC | Mehrotra + Nesterov-Todd 缩放 | 是（`[z | y]`） |
| `NativeBranchAndCut` | MILP/MINLP | 分支定界、割、启发式 | 是（LP 路径） |
| `StrictHiGHS` | MILP | 嵌入式 HiGHS MIP 状态机（生产合约） | — |
| `HiGHS` | LP/MILP | 进程内或命令行外部后端 | 否（文件路径模式） |
| `Ipopt` | NLP | Ipopt TNLP/NL 文件适配 | 否 |
| `SCIP` | MILP/MINLP | MPS/PIP 文件适配 | 否 |
| `Gurobi` | LP/QP/MILP | 原生 C API（需运行时许可证） | 是 |

两点容易误解：

- `StrictHiGHS` 不是第二套自研 B&C，而是通过
  `make_strict_highs_production_options` 强制执行 HiGHS MIP 合约的适配器；
  `NativeBranchAndCut` 才进入项目自研的树搜索框架。
- 文件型 HiGHS 适配器只解析原始解，**不返回对偶**。算 LMP 等需要对偶时，
  选 `Gurobi`、`NativeBranchAndCut`（LP 路径）或 `NativeIPMLPAdapter`。

## 2. 按问题类型选求解器

### 2.1 LP

| 场景 | 推荐 | 理由 |
|---|---|---|
| 一般生产 LP，拿不准 | 默认路径（`NativeAutoLPAdapter` 组合，见 §3） | 原生 dual simplex 与 IPM 并发竞速，取先到者 |
| 依赖外部成熟实现兜底 | `HiGHS` | 进程内库；部分配置保留 MPS/solution 临时文件路径 |
| 超大稀疏、可接受中等精度 | `NativePDLPAdapter` | 一阶法，不分解 KKT/基矩阵，内存友好 |
| B&C 节点 LP / 改界重优化 | dual simplex（经 `BasisState` 热启动） | 基热启动 + 精确约化成本，PDLP 不产生基 |
| 需要 LP 对偶/盒约束乘子 | `NativeIPMLPAdapter` 或 `Gurobi` | 填充 `constraint_duals`、`box_dual_lb/ub` |

注意：

- `NativePDLPAdapter` 是中等精度一阶法，**不提供 basis**，不能用于要求
  严格节点证明的 B&C 场景。
- 原生 IPM LP 当前**没有 HSD 状态机**，普通迭代失败不能推出不可行或无界
  （HSD 路线见 §5.4）。若必须拿到 infeasible/unbounded 证书，用 HiGHS
  或 Gurobi。
- `IPMLPOptions::centrality_step_control=false` 只为复现旧的固定步长
  A/B 保留，不建议生产使用。
- `IPMLPOptions::presolve`（默认 `true`）自 2026-08 P1 阶段起是活的：
  原生 LP presolve（Andersen & Andersen 1995 §2.1-2.3 零填充规则：空行/
  空列/固定列/冗余行/singleton 行）在冷启动求解前运行，reduced 模型
  求解后经 postsolve + 原模型残差审计发布，任何一步失败都回退原模型
  direct 求解，因此不会发布错误或更松的解。环境变量
  `MIPSOLVERS_NATIVE_PRESOLVE=0` 强制关闭（其他任何值强制开启），
  `MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE` 打印 `[NATIVE-PRESOLVE]` 汇总，
  `MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX` 改内部时间盒（默认 2s，超盒
  返回未缩减模型）。Auto 选择器（§3）的 IPM 臂本阶段不走 presolve；
  设计与分阶段验收见
  `docs/archive/native_presolve_lp_2026-08-18.md`。

### 2.2 MILP

| 场景 | 推荐 | 理由 |
|---|---|---|
| 默认生产 | `StrictHiGHS` 或默认 Auto 策略 | 强制 HiGHS MIP 合约；大根节点自动用 IPM + crossover |
| 需要自研树控制（回调、ML 分支先验、SCUC 启发式、自定义割） | `NativeBranchAndCut` | 暴露 `BCOptions`/`BCCallbacks` 全套钩子，见 §6.2 |
| 有 Gurobi 许可证 | `Gurobi` | 直接 C API，支持 LP/QP/MILP；许可证不可用时不会注册为候选 |
| 仅需快速可用解、无进程内库 | `HiGHS` | 外部二进制兜底 |
| MINLP | `SCIP` 或 `NativeBranchAndCut`（实验性） | 见下面的警告 |

`NativeBranchAndCut` 的默认节点 LP 仍走 HiGHS
（`lp_kernel_backend` 默认 `HiGHS`，`ExperimentalNative` 仅开发用）。
MINLP 复用 B&C 外框、在连续松弛处调 NLP 路径；一般非凸 MINLP **没有全局
下界保证**，除非连续松弛可证明为凸，否则应视为实验性局部求解能力。

### 2.3 NLP

| 场景 | 推荐 | 理由 |
|---|---|---|
| 一般约束 NLP 默认 | `NativeIPMAdapter` | Wachter–Biegler 型滤子全局化，精确 Hessian 或拟 Newton |
| 需要成熟外部实现 / 导出 NL 文件 | `Ipopt` | `CallbackTNLP` 映射，返回后计算项目 KKT 诊断 |
| 轻量、近似即可 | `NativeNLPAdapter` | 二次罚函数 Newton；固定罚参数不构成强收敛保证 |

### 2.4 锥规划与 QP

- CONIC（LP/SOCP/SDP，CVXOPT 标准型）只有一条原生路径
  `NativeConicIPM`：Mehrotra 预测-校正 + NT 缩放，返回对偶。
- 凸 QP 用 `NativeLCQPAdapter`；`Q` 非凸时没有全局最优保证。
  Gurobi 可用时 `Gurobi` 也覆盖 QP。

### 2.5 选择策略选项

不显式指定求解器时，`StrategyPolicy` 控制候选排序：

```cpp
enum class StrategyPolicy {
  Auto,          // 按问题特征的引擎启发式
  NativeFirst,   // 原生适配器优先
  ExternalFirst, // 外部适配器（Gurobi、HiGHS…）优先
};

struct SolveOptions {
  std::string   preferred_solver;        // 精确适配器名，"" 为自动
  bool          allow_fallback{true};    // 首选失败时尝试下一个候选
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  // 按问题类的覆盖（优先于 strategy_policy）
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
};
```

Auto 模式下的选择顺序：

1. 若用 `set_solver_preference` 设过按类偏好，该适配器最先尝试；
2. 否则按策略排序——`Auto` 下 LP/QP/MILP 优先已安装/已授权的
   `Gurobi`，MILP 随后回退 `StrictHiGHS`、`HiGHS`、`NativeBranchAndCut`；
3. 选中适配器失败且 `allow_fallback = true` 时按序尝试下一候选。
   **显式指定后端时不会静默冒充另一个后端**；最终 `stats.solver_name`
   揭示实际路径。

示例——LP 与 MILP 都强制外部优先，并锁定 Gurobi：

```cpp
SolveOptions opts;
opts.class_strategy_policy[ProblemClass::LP]   = StrategyPolicy::ExternalFirst;
opts.class_strategy_policy[ProblemClass::MILP] = StrategyPolicy::ExternalFirst;
opts.preferred_solver = "Gurobi";
auto result = eng.solve_milp(mip, opts);
```

## 3. LP 内核选择器：为什么默认路径是并发组合

> 本节依据 `docs/lp_kernel_selector_2026-08-11.md`（2026-08-11），
> 其结论比 `docs/engine.md` 的注册表描述更新：LP 默认顶层路径已切换为
> `NativeAutoLPAdapter` 并发组合。

### 3.1 没有单一赢家（实测动机）

24 个 NETLIB 实例上（macOS M4，Release，2026-08-11）没有任何单一原生
LP 内核能压倒 HiGHS simplex：

- 原生 dual simplex（ExactDSE 全精确对偶最陡边）赢下中小实例与几何平均，
  但输掉大型/稠密实例；
- 原生 IPM（centrality-step，direct）赢下大型/稠密/宽矩阵实例
  （`scsd8` 8.6x、`25fv47` 4.2x、`d2q06c` 3.8x），但输掉小实例；
- 逐实例 oracle（两者取快）可达对 HiGHS-simplex 的几何平均 **1.60x**。

### 3.2 成本模型：结构决定倾向

设 LP 有 `m` 行、`n` 列、`nnz` 非零元，normal equations
`M = A_a Θ A_aᵀ` 的符号分解给出因子 flops `F` 与填充 `nnz(L)`：

- IPM：迭代数近似常数 `K₀`，每次一次稀疏 `LDLᵀ`：`T_ipm ≈ K₀ · F`；
- dual simplex：`P ≈ κ·m` 次主元（NETLIB 上经验 `κ ≈ 1–3`），每次一次
  FTRAN + BTRAN（`≈ 2·nnz(L)`）加一遍 PRICE 扫描（`≈ nnz`）：
  `T_splx ≈ κ·m · (2·nnz(L) + nnz)`。

PRICE 项使模型感知矩阵长宽比：宽 LP（`n ≫ m`）推高单纯形成本、偏向
IPM；高瘦 LP 偏向单纯形。单纯形预测更快的条件为

```text
g := F / ( m · (2·nnz(L) + nnz) )  >  κ/K₀ =: c
```

按 NETLIB 区间 `κ ≈ 2`、`K₀ ≈ 50` 得 `c ≈ 0.04`。

### 3.3 为什么结构估计器没有上线

该规则实现后实测（2026-08-11，repeat 3）：离线标定几何平均约 1.10x，
但进程内实际选择器只得 0.92x——个别实例（`agg` 0.09x、`sc205` 0.08x、
`ship04s`、`stocfor1`）的运行时间由实际主元/迭代数决定，稀疏模式无法
预测，每次误路由代价 3–16x。**廉价先验结构信号存在根本性上限**，结构
估计器被移除。

### 3.4 上线设计：并发组合（portfolio）

`NativeAutoLPAdapter`（`src/engine/solver/native/native_lp_selector.cpp`）
在两个线程上同时跑 dual-simplex-DSE 与 IPM（`ipm-direct`），返回第一个
成功结果——直接实现逐实例 `min(T_DSE, T_IPM)` 而不是预测它：

- 两个内核都经过同一容差审计，谁先赢结果都正确；
- 一方胜出后，共享 abort 标志（`SimplexOptions::cancel_flag` /
  `IPMLPOptions::cancel_flag`，在各内核墙钟检查点轮询）及时停掉输家；
- dual simplex 路径上重新计算用户目标 `c·x`（该内核报告内部
  minimize-sense 值）。

实测（24 例 NETLIB，repeat 3，对 HiGHS-simplex 的中位数几何平均）：

| 默认 LP 路径 | 几何平均 |
|---|---|
| 旧默认（仅 IPM） | 0.88x |
| 仅 ExactDSE | 0.92x |
| 结构选择器（已移除） | 0.92x |
| **并发组合（已上线）** | **1.26x**（另一口径实测 1.30x） |

协作式取消是关键：A/B 显示它把 72 次求解的用户 CPU 降约 35%
（4.60s → 2.98s）、墙钟降约 72%（2.89s → 0.81s）——不取消的输家会
一直占核拖慢后续求解。取消标志默认未设置，非组合路径的调用者保持
逐位一致（bit-identical）路径。

对集成方的影响：

- 赢家的不确定性只体现在时序上，不体现在答案上；
- MILP/SCUC/B&C 不受影响——它们直接调 `solve_lp_with_basis` /
  `solve_lp_from_sf`，不经过默认 LP 优先级链。

## 4. Engine 统一求解接口与适配器注册

### 4.1 最小调用

```cpp
#include "mipsolvers/engine/engine.hpp"

using namespace mipsolvers::engine;

// Build an LP
LPModel lp;
lp.sense = Sense::Minimize;
lp.c = Eigen::VectorXd::Ones(2);          // min x0 + x1
lp.Aeq.resize(1, 2);
lp.Aeq.insert(0, 0) = 1.0;
lp.Aeq.insert(0, 1) = 1.0;
lp.Aeq.makeCompressed();
lp.beq = Eigen::VectorXd::Constant(1, 1.0);
lp.vars = {{VarType::Continuous, 0.0, 1.0},
           {VarType::Continuous, 0.0, 1.0}};

// Solve
SolverEngine eng;
eng.register_default_adapters();
auto result = eng.solve_lp(lp);

if (result.stats.success) {
    std::cout << "objective = " << result.stats.objective << "\n";
    std::cout << "x = " << result.x.transpose() << "\n";
}
```

MILP 只需把变量标成 `Binary`/`Integer` 并改用 `MIPModel`：

```cpp
MIPModel mip;
mip.linear_part = lp;                    // same LPModel fields
mip.binary_idx  = {0};                   // variable 0 is binary
auto result = eng.solve_milp(mip);
```

模型结构（`LPModel`、`QPModel`、`NLPModel`、`MIPModel`、`ConicModel`
等）的完整字段见 [API 参考](07-api-reference.md)；用 AML 从高层描述
生成这些模型见 [建模 API（AML）](04-modeling-aml.md)。

### 4.2 `SolverEngine` API

```cpp
class SolverEngine {
public:
  explicit SolverEngine(bool register_defaults = true);
  void register_adapter(const SolverAdapterPtr& adapter);
  std::size_t register_default_adapters();
  void set_solver_preference(ProblemClass cls, const std::string& adapter_name);
  std::vector<std::string> list_solvers(ProblemClass cls) const;

  api::Result solve(const api::ProblemVariant& problem,
                    const SolveOptions& options = {}) const;
  api::Result solve_lp   (const LPModel&  p, const SolveOptions& o = {}) const;
  api::Result solve_milp (const MIPModel& p, const SolveOptions& o = {}) const;
  api::Result solve_nlp  (const NLPModel& p, const SolveOptions& o = {}) const;
  api::Result solve_conic(const ConicModel& p, const SolveOptions& o = {}) const;
  // …solve_le / solve_nle / solve_qp / solve_minlp 同理
};
```

`register_default_adapters()` 的注册顺序（即候选偏好序）：`NativeLinear`、
`NativeNewton`、`NativeIPMLPAdapter`、`NativePDLPAdapter`、
`NativeLCQPAdapter`、`NativeIPMAdapter`、`NativeNLPAdapter`、
`NativeConicIPM`、`StrictHiGHS`、`NativeBranchAndCut`、`Gurobi`（若可用）、
`HiGHS`、`Ipopt`、`SCIP`。可选外部库只有 `available()` 为真时才进入注册表；
**Gurobi 不参与普通自动回退**。

调度流程（`src/engine/api/solver.cpp::SolverEngine::solve`）：

1. 规范化公共模型并做结构/有限值校验；
2. `StrategyDispatcher::candidate_adapters` 按问题类型和 `solver_name`
   构造候选；
3. 对允许的模型执行预处理，保留恢复原变量的映射；
4. 调适配器的类型专用虚函数；
5. 按策略决定是否尝试下一个后端；
6. 后处理恢复变量、目标和状态，返回统一结果。

排查求解器选择时可用 `list_solvers` 自省：

```cpp
for (const auto& name : eng.list_solvers(ProblemClass::MILP))
    std::cout << name << "\n";
```

### 4.3 结果契约：不要只看 `success`

`api::Stats` 关键字段：

```cpp
bool        success{false};        // 通过该适配器成功终止
double      objective{0.0};
double      residual_inf{0.0};     // 最大约束违反
double      primal_feas{0.0};
double      dual_feas{0.0};
double      complementarity{0.0};
double      mip_gap{0.0};          // 仅 MILP
std::string status;                // 人类可读状态
std::string solver_name;           // 实际产出结果的适配器
Eigen::VectorXd farkas_ray, farkas_ray_eq;  // Infeasible 时的 Farkas 证书
bool            has_farkas_certificate{false};
```

`success` 只表示“通过该适配器成功终止”，**不等价于“最优”**。验收时必须
同时检查 `status`、原始/对偶可行度、互补度、目标界和 gap。LP 求解的对偶
布局统一为 `[不等式行对偶 | 等式行对偶]`，跨适配器一致。

## 5. 各求解器数学原理要点

本节只讲影响使用判断的要点；完整推导见 [数值方法](06-numerical-methods.md)
与 [理论参考](11-theory-references.md)。

### 5.1 原生 dual simplex

对标准型最大化问题，基矩阵 `B`，基本值 `x_B = B⁻¹b`，对偶乘子
`pi = B⁻ᵀc_B`，约化成本 `d = c − A'pi`。对偶单纯形保持约化成本满足边界
符号条件，逐步修复基本变量的原始不可行性。主循环
（`native_dual/solver.cpp::solve`）：

1. 构造或导入基，修复无效/越界/秩亏基；冷启动经 cost shift / dual
   Phase I 获得对偶可行状态，必要时进入原始 Phase I；
2. pricing 选择原始不可行的离基行（DSE/Devex 权重控制质量）；BTRAN 得
   枢轴行，PRICE 计算候选列与方向；
3. BFRT 比率检验可先翻转一段非基变量边界再选入基列；FTRAN 得枢轴列并
   更新基本值、约化成本与权重；
4. Forrest–Tomlin 更新维护基因子，达到更新阈值或数值异常时 INVERT；
5. 周期检测通过 tabu/重建/扰动处理；残差审计失败会重建而非直接宣告最优；
6. 原始与对偶可行时清除 cost shift，**在原问题上重算残差**后发布最优；
   无候选列时给出不可行证书，原始方向无界时发布原始射线。

`BasisState`（基列、非基变量边界、稀疏因子状态、标准型映射）支持节点
LP、改界重优化和加割后的热启动——这是 B&C 节点内核选单纯形而非 PDLP
的根本原因。所有增量改界/加行操作使用提交或回滚语义，失败不污染持久
基状态。

### 5.2 Branch-and-cut（`NativeBranchAndCut`）

主流程按 14 个编号阶段执行（`BCSolveState::run` 拆成 14 个 `.inc`
文件），可概括为：

1. 校验整数索引，装截止时间，跑 PaPILO/HiGHS/native presolve 并建恢复
   映射；
2. 构造根 LP（HiGHS、原生 dual simplex 或 IPM 根路径），审核根解；
3. 根传播、探测、割分离，多轮重解直到停滞或预算耗尽；
4. warm start、rounding、feasibility pump、diving、RENS/LNS 等原始启发
   式——每个候选都**在原模型上验证后**才发布 incumbent；
5. 树搜索循环：弹出节点 → 域事务传播（clique/implication/冲突）→ 解
   节点 LP → 证明安全的剪枝 → 割分离（GMI/MIR/cover/clique/
   implied-bound/CGLP 及 SCUC 专用割）→ reliability branching（强分支
   样本 + 伪成本 + 静态优先级 + 动态 ML prior）→ 创建子节点；
6. 并行路径经共享 incumbent、工作窃取和可选确定性令牌协调。

证明安全边界（集成自定义割时必须遵守）：

- 每条割需确认变量空间（原始/预处理）、有效范围
  （`ValidityScope::GlobalCut / LocalNodeCut / LazyConstraint`）、数值清理
  与去重键；仅对 incumbent cutoff 有效的割不能作为无条件全局约束；
- 节点 LP 的“不可行”只有在原模型尺度上通过可信后端/证书审计后才能剪枝；
  扰动回退只用于寻找候选，不能单独证明；
- 并行线程必须以同一快照读取 incumbent，避免撕裂读导致不同剪枝决定。

### 5.3 原始-对偶 IPM / Mehrotra（`NativeIPMLPAdapter`）

对一般 LP `min cᵀx, Ax ≤ b, A_ex = b_e, l ≤ x ≤ u`，fresh 路径增加不等式
松弛 `s = b − Ax` 组成等式系统；每个有穷界维护正间隙与正对偶
（`g_l = x̄ − l > 0, z_l > 0`，上界对称），固定变量退出障碍系统。互补度

```text
μ = ( Σ_j g_l,j z_l,j + Σ_j g_u,j z_u,j ) / n_c
```

每次迭代求解 Mehrotra 预测-校正 Newton 系统：

1. 仿射方向令中心目标为零；用仿射最大正步长算 `μ_aff`，中心参数
   `σ = (μ_aff/μ)³`，`clamp(σ, 0, 1)`（旧实现无条件截断到 0.5，会在最
   需要回中心时欠中心化；P1 已修正）；
2. 校正方向同时加入 `σμ` 中心项和 `−Δg^aff Δz^aff` 二阶项，**复用同一
   KKT 分解**；
3. 步长不是固定 0.9995 回缩，而是由试探互补度决定的动态互补缓冲步长：
   至少保留 90% 最大步，同时避免阻塞互补乘积被推到机器边界附近
   （`ipm_centrality_step_lengths`，fresh/cached 共用）；
4. Gondzio 多重中心校正把试探点中过小/过大的 `g_j z_j` 投回宽中心邻域
   `[0.1μ, 10μ]`，不再分解 KKT，只在总步长得到足够提升时接受
   （cached 路径尚无多重校正）。

主流程其余环节：预处理/变量变换 → Ruiz 缩放 → 严格内部初值 → 残差计算
→ normal equation 或增广系统组装（按 normal matrix 带宽、预测内存和稠
密度路由）→ 动态对角扰动与 inexact-Newton 精化 → 相对候选检查 → 反缩
放后**在原模型上审计** primal、dual stationarity 与 relative gap，通过才
发布 `Optimal`。缩放空间条件只产生候选，不能绕过原模型 KKT 审计。

B&C 重复节点走 `ipm_lp_solver_cached.cpp`：只有上下界/目标改变时复用
结构与分解，事务签名不匹配则退回完整求解。

### 5.4 HSD 路线（不可行/无界证书，当前状态）

原生 IPM 目前只在找到最优 KKT 点时可靠；普通路径不能证明 infeasible
或 unbounded。设计路线（P2，待实现）是齐次自对偶嵌入：引入
`τ, κ ≥ 0`，

```text
Ax − bτ = 0,   −Aᵀy + cτ − s = 0,   bᵀy − cᵀx − κ = 0,
```

加 `xᵀs + τκ` 互补方程。`τ > 0` 时除以 `τ` 恢复原问题解；
`τ → 0, κ > 0` 时从 `(x,y,s)` 提取 primal 或 dual 不可行证书。
职责边界已确定：KKT 求解器只返回方向、不决定模型状态；状态判定在原始
未缩放 LP 上重算；`SolveResult` 区分 `Optimal / PrimalInfeasible /
DualInfeasible / NoProgress`；presolve/postsolve 必须能恢复证书，否则
只能返回 `unknown`。

对现场的含义：**在 HSD 落地前，若 LP 可能不可行且需要证书，把该 LP 路由
到 HiGHS 或 Gurobi，或检查 `farkas_ray` 是否由其他路径填充**；原生 IPM
返回的 `unknown` 必须被上层保留，不能改标为 infeasible。

### 5.5 锥 IPM（`NativeConicIPM`）

锥模型为 CVXOPT 标准型：

```text
(P) min c'x,  Gx + s = h, Ax = b, s ∈ K
(D) max −h'z − b'y,  G'z + A'y + c = 0, z ∈ K
K = R_+^l × ∏ Q_i × ∏ S_+^j
```

算法要点：`x=0, y=0, z=e` 起步并把 `s=h` 沿单位元平移到锥内部；分块
Nesterov–Todd 缩放；分解约化 KKT 后解仿射预测方向，`σ=(μ_aff/μ)³`；
加中心化及二阶交叉项并复用同一分解解校正方向；SOC/SDP 方向可作 3×3
原系统迭代改进，取阻尼锥内步长。**同时满足可行度和绝对/相对 gap 才返回
`optimal`**，否则区分原始不可行、对偶不可行或 `unknown`；弦分解路径恢复
原 SDP 变量后重新计算原问题残差，不合格结果降级为 `unknown`。SDP 块使用
保持 Frobenius 内积的 `svec/smat` 打包。完整推导见 `docs/conic_sdp.md`。

## 6. 关键选项与终止判据

### 6.1 `SolveOptions`（所有问题类）

见 §2.5。三个最常用的动作：

- `preferred_solver = "<适配器名>"`：锁定后端（名表见 §1）；
- `allow_fallback = false`：禁止静默回退，适合验收/审计场景；
- `class_strategy_policy[cls]`：按问题类覆盖策略。

### 6.2 `BCOptions`（MILP）

原生 B&C 可绕过 `SolverEngine` 直接使用：

```cpp
#include "mipsolvers/engine/bc/api.hpp"

BCOptions opt;
opt.max_nodes     = 100'000;
opt.gap_tol       = 1e-3;
opt.num_threads   = 4;
opt.use_feasibility_pump = true;

BCResult result = solve_milp_bc(mip, opt);
```

核心限额与容差（默认值即生产默认，改动需有依据）：

| 字段 | 默认 | 说明 |
|---|---|---|
| `max_nodes` | 50 000 | B&B 节点上限 |
| `max_lp_iter` | 500 | 每节点 LP/NLP 迭代上限 |
| `time_limit_sec` | 120.0 | 墙钟时限（秒） |
| `int_tol` | 1e-5 | 整数容差 |
| `gap_tol` | 1e-4 | 相对原始-对偶 gap 容差 |
| `require_tree_exhaustion_certificate` | `false` | 忽略 `gap_tol`，要求树完全枚举 |
| `lp_tol` | 1e-6 | LP 松弛收敛容差 |

节点 LP 内核选择：

| 字段 | 默认 | 说明 |
|---|---|---|
| `use_simplex_lp_nodes` | `true` | 树节点 LP 用 dual simplex |
| `use_ipm_root` / `use_ipm_nodes` | `false` | 根/全部节点用 IPM |
| `lp_kernel_backend` | `HiGHS` | LP 数值内核；`ExperimentalNative` 仅开发用 |
| `strict_highs_mip_contract` | `false` | 显式 full-MIP StrictHiGHS 策略 |
| `highs_mip_lp_solver` | `"choose"` | StrictHiGHS 根 LP 求解器；大根自动改 `"ipm"` |
| `highs_mip_root_crossover` | `"on"` | 根 IPM 后 crossover；后续节点需要单纯形基时保持 `"on"` |
| `highs_strict_auto_ipm_root_min_cols/rows` | 10 000 | 大根自动 IPM 的行/列阈值 |
| `highs_strict_auto_ipm_root_min_time_sec` | 30.0 | 自动根 IPM 的最小 MIP 时限 |
| `simplex_factor_backend` | 0 | 0=UmfpackNative, 1=HiGHSSafe, 2=ForceFT, 3=ShortChain |
| `ipm_auto_threshold` | 8 000 | 根 `m` 超过阈值切 IPM |
| `hybrid_ipm_threshold` | 15 000 | 节点 LP 混合 IPM/单纯形跨界 |
| `xlarge_ipm_only_threshold` | 50 000 | 超过此规模只用 IPM |

启发式、并行、割的常用开关：

| 字段 | 默认 | 说明 |
|---|---|---|
| `use_feasibility_pump` / `use_progressive_rounding` | `true` | 原始启发式 |
| `use_papilo_presolve` | `true` | PaPILO 预处理（`papilo_aggressive` 默认 `false`） |
| `enable_domain_heuristics` | `false` | UC/SCUC 领域启发式（配合 `MIPModel::uc_hint`） |
| `enable_lns` | `true` | LNS 改进启发式（`lns_fix_ratio` 0.80、`lns_node_limit` 500、`lns_time_limit` 5.0、`lns_max_iters` 3） |
| `num_threads` | -1 | 工作线程（-1 = 硬件并发） |
| `deterministic_parallel` | `false` | 轮转令牌实现可复现并行搜索 |
| `parallel_delay_until_incumbent` | `true` | 有有限 incumbent 后才启动并行树 |
| `enable_work_stealing` | `true` | 并行线程间工作窃取 |
| `cut_pool_max_size` / `cut_pool_max_age` | 2 000 / 50 | 全局割池容量与逐出年龄 |
| `root_cut_rounds` / `cuts_per_round` | 10 / 20 | 根割轮数与每轮上限 |
| `max_cut_depth` | 0 | 树节点割深度（0 = 仅根） |
| `root_cut_stall_tol` / `root_cut_max_stalls` | 1e-4 / 2 | 界提升停滞即停割 |
| `gmi_min_efficacy` / `gmi_max_density` / `gmi_max_parallelism` | 1e-3 / 0.35 / 0.90 | GMI 割准入门限 |
| `enable_lp_fallback` | `true` | 多阶段 LP 失败恢复（`fallback_l1_retries` 3、`fallback_l3_retries` 2） |
| `enable_reduced_cost_fixing` | `true` | LP 后的 reduced-cost 域固定与重解 |

分支与节点选择：

```cpp
enum class BranchingStrategy {
  MostInfeasible,  // 分数最接近 0.5 的变量
  Pseudocost,      // Benichou 伪成本估计（默认）
  FirstFractional, // 第一个分数变量（基线/调试）
};
enum class NodeSelection {
  BestFirst,   // 估计引导的最佳优先
  DepthFirst,  // LIFO（快出 incumbent、内存小）
  Hybrid,      // 首个 incumbent 前 DFS，之后 best-first（默认）
};
```

`MIPModel::branching_priority`（值大先分支）可对选定变量覆盖分支策略，
但不关闭伪成本学习。割族由 `BCOptions::cuts` 选择（`None / IntRounding /
MIR / Gomory / Cover / All`，默认 `All`）。

Warm-start 与回调（ML 先验、动态节点割）：

```cpp
BCWarmStart ws;
ws.primal_hints.push_back({initial_x, initial_obj, false});

BCCallbacks cbs;
cbs.hyper_tuner = [](const BCInstanceFeatures& feat, const BCOptions& base,
                     const BCStats* prev) -> BCOptions {
  BCOptions tuned = base;
  if (feat.n_bin > 500) tuned.branching = BranchingStrategy::Pseudocost;
  return tuned;
};

BCResult result = solve_milp_bc(mip, opt, ws, cbs);
```

每个 primal hint 都经过正常可行性检查后才被接受为 incumbent。动态节点割
钩子 `cbs.dynamic_node_cut` 可在整数可行节点注入 lazy-constraint 风格的
用户割，每条割标注 `ValidityScope`（语义见 §5.2）；回调返回 0 表示无违反。

### 6.3 IPM 终止判据

LP IPM 的发布门槛（P2a，已实现）：缩放空间的绝对/相对残差只产生候选；
候选点必须在**原模型**上同时满足

```text
ρ_p = ||r_p||_∞ / max(1, ||(b,l,u)||_∞)
ρ_d = ||r_d||_∞ / max(1, ||c||_∞)
ρ_g = |pobj − dobj| / (1 + |pobj| + |dobj|)
```

primal residual、dual stationarity 与 relative gap 审计（
`audit_ipm_lp_optimality`，fresh/cached 共用同一审计器）。fixed column
的非唯一界乘子由 stationarity 重构。API 暴露
`relative_primal_residual`、`relative_dual_residual`、`relative_gap`、
`dual_objective` 供上层审核。锥 IPM 同理：可行度与绝对/相对 gap 同时
满足才返回 `optimal`。

MILP 的终止由 §6.2 的 `gap_tol` / `max_nodes` / `time_limit_sec` 控制；
设 `require_tree_exhaustion_certificate = true` 可忽略 `gap_tol` 换取
精确树枚举证书。

## 7. 数值保护设计

以下保护贯穿各内核，理解它们有助于正确解读结果状态：

- **原模型审计优先。** LP IPM、锥 IPM、dual simplex 在发布 `Optimal` 前都
  在未缩放原模型上重算残差；缩放只改变坐标，不改变“成功”的定义。弦分解
  路径恢复原变量后复核，不合格降级为 `unknown`。
- **迭代改进只提交严格改进。** KKT 求解在原始未正则化系统上算残差，迭代
  改进只提交严格降低无穷范数残差的候选方向。
- **正则化有误差预算。** IPM 的对角扰动候选
  `ρ_μ = clamp(1e-6·μ, ε_mach, 1e-2)`，并按 inexact-Newton forcing 条件
  `ρ_k(1+‖(x_k,y_k)‖_∞) ≤ 0.1·‖F(x_k,y_k,z_k)‖_∞` 进一步限制；方向的
  未扰动线性残差须同时满足
  `‖KΔ−r‖_∞ ≤ min(1e-9‖r‖_∞, 0.1‖F‖_∞)`。分解失败由动态重试增大正则。
  注意这只控制数值扰动误差，**不是**有完整收敛理论的 IP-PMM。
- **事务语义。** 单纯形的增量改界/加行、B&C 的域 trail、IPM 的
  `IterateSnapshot`/`TrialPoint` 都是提交或回滚；失败不污染持久状态。
- **失败 loudly。** 空的 0×0 线性系统有显式成功路径；分解失败、非有限解、
  残差不下降不得被改写为成功。`unknown` 状态必须透传，不能改标。
- **回退不冒充。** 显式指定后端时失败不会静默换成另一个后端；外部回退
  （如 NLP 失败调 Ipopt）默认关闭，启用后最终 `solver_name` 必须揭示实际
  路径。根求解在预处理模型失败、回退原模型时，不能复用不相容的基、割或
  列索引。
- **Windows KKT 路径固定。** 不再按 KKT 维数切换 CHOLMOD/PARDISO：主轨迹
  使用结构保持的准定 LDLT；只有完整求解或原模型审计失败，才从同一初始
  点重启带主元的完整 barrier 轨迹，禁止中途更换后端。

## 8. 已知限制速查

- 默认 LP kernel 在部分平台可能是 HiGHS；原生 dual simplex 的冷启动、
  极端退化与大型 NETLIB 覆盖需逐平台验证。
- PDLP 不提供 basis，不能替代依赖基热启动和精确约化成本的 B&C 节点内核。
- `NativeIPMLPAdapter` 无 HSD 状态机，不能从普通迭代失败推出不可行/无界。
- `NativeNLPAdapter` 的固定罚参数不构成一般 NLP 的强收敛保证；“步长很小”
  状态必须结合可行度审核。
- 原生 B&C 的主要实现依赖 HiGHS 库编译；非凸 MINLP 没有全局性保证。
- 性能结论强依赖编译器、BLAS、稀疏后端、线程与数据集；可复现数据见
  [测试与基准](09-testing-benchmarks.md)（源码用例数不等于通过数）。

## 相关章节

- [安装与部署](02-installation-deploy.md)——可选后端（HiGHS/Ipopt/SCIP/
  Gurobi/MUMPS/CHOLMOD）的编译开关
- [建模 API（AML）](04-modeling-aml.md)——从高层模型生成 `LPModel`/`MIPModel`
- [数值方法](06-numerical-methods.md)——稀疏线性代数与 KKT 后端
- [API 参考](07-api-reference.md)——模型结构与 `api::Result` 完整字段
- [工业应用](08-industrial-applications.md)——SCUC/OPF 中的求解器配置实例
- [理论参考](11-theory-references.md)——Mehrotra、Gondzio、HSD 等文献清单
