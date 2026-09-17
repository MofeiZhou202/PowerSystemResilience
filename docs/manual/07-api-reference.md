# 第 7 章 API 参考

> 本章整合自: docs/archive/engine.md, docs/archive/python_api.md, docs/archive/aml_python_api.md

本章给出 MIPSolvers 两层 API 的对照参考：

- **C++ Engine API**（`mipsolvers::engine` 命名空间）：统一求解器调度层，支持 LE / NLE / LP / QP / NLP / MILP / MINLP / CONIC，通过单一 `SolverEngine` 类屏蔽各求解器后端差异；
- **Python API**（pybind11 绑定）：`mipsolvers.engine`（矩阵式 LP/MILP 求解）与 `mipsolvers.aml`（集合索引的高级建模接口）。

关于 SCUC 领域级 Python 接口（`mipsolvers.scuc`），见 [工业应用](08-industrial-applications.md)；安装与构建见 [安装与部署](02-installation-deploy.md)。

---

## 7.1 C++ Engine API

所有头文件位于 `include/mipsolvers/engine/`。umbrella 头文件 `engine.hpp` 包含全部 `api/` 头；常用头文件布局：

| 头文件 | 内容 |
|---|---|
| `engine.hpp` | 总头文件（含所有 api/ 头） |
| `problem_types.hpp` | 全部模型结构体 |
| `api/solver.hpp` | `SolverEngine` 类 |
| `api/session.hpp` | `LPModelSession` persistent/incremental LP API |
| `api/result.hpp` | `api::Result`, `api::Stats` |
| `api/options.hpp` | `SolveOptions`, `StrategyPolicy` |
| `solve_context.hpp` | 一次调用共享的 deadline、取消和资源契约 |
| `solver/solver_adapter.hpp` | `SolverAdapter` 基类 |
| `solver/external/adapters.hpp` | `HighsAdapter`, `IpoptAdapter`, `ScipAdapter`, `GurobiAdapter` |
| `solver/native/native_adapters.hpp` | 原生适配器 |
| `bc/api.hpp` | `solve_milp_bc` / `solve_minlp_bc`（原生 B&C，详见 [求解器与引擎](05-solvers-engines.md)） |

### 7.1.1 枚举与基础类型

| 类型 | 取值 | 说明 |
|---|---|---|
| `ProblemClass` | `LE` / `NLE` / `LP` / `QP` / `NLP` / `MILP` / `MINLP` / `CONIC` | 问题类别 |
| `VarType` | `Continuous` / `Integer` / `Binary` | 变量类型 |
| `Sense` | `Minimize`（默认）/ `Maximize` | 优化方向 |

每个变量通过模型的 `vars` 向量附带元数据：

```cpp
struct VariableMeta {
  VarType     type{VarType::Continuous};
  double      lb{-1e20};
  double      ub{ 1e20};
  std::string name;          // optional, for debugging
};
```

### 7.1.2 模型结构体

| 结构体 | 对应问题 | 关键字段 |
|---|---|---|
| `SparseLinSys` | LE | 稀疏 `A`、`b`，求解 `Ax = b` |
| `LPModel` | LP | `c`、`A`/`b`（不等式）、`Aeq`/`beq`（等式）、`row_lhs`（可选行下界）、`vars` |
| `QPModel` | QP（凸） | 上述 LP 字段 + 对称半正定 Hessian `Q`，目标为 ½xᵀQx + cᵀx |
| `NLPModel` | NLP | 回调 `f`、`grad`、`hess`、`g`/`h`、`jac_g`/`jac_h`、初始点 `x0`、可选符号表达式 `symbolic_objective` |
| `MIPModel` | MILP | `linear_part`（LPModel）、`integer_idx`、`binary_idx`、`initial_solution`（warm-start）、`branching_priority`、可选 `uc_hint` |
| `ConicModel` | CONIC | `c`、`G`/`h`、`A`/`b`、`dims`（锥维度 `{l, q, s}`），按 cvxopt 标准形 `Gx + s = h, s ∈ K` |
| `MINLPModel` | MINLP | `nonlinear_part`（NLPModel）+ `integer_idx`/`binary_idx` |

`LPModel::row_lhs` 用于 **ranged rows**：当 `row_lhs[i]` 有限时，第 `i` 行变为
`row_lhs[i] <= A[i,:]*x <= b[i]`；否则该行下界为 −∞。配套辅助函数：
`lp_has_row_lhs()`、`lp_row_lhs_or_neg_inf()`、`lp_row_has_finite_lhs()`。

MILP 的整数/二进制变量通过 `integer_idx` / `binary_idx` 以索引方式指向
`linear_part.vars` 的扁平变量列表。`MIPModel::UCGenHint` 为原生 B&C 引擎提供
UC 领域元数据（机组数 `ng`、时段数 `T`、各变量块起始索引、最小开停机时间、
功率/爬坡限值等），在 `BCOptions::enable_domain_heuristics = true` 时生效；
详见 [求解器与引擎](05-solvers-engines.md)。

### 7.1.3 SolverEngine 类

```cpp
#include "mipsolvers/engine/engine.hpp"
namespace mipsolvers::engine {

class SolverEngine {
public:
  explicit SolverEngine(bool register_defaults = true);  // false = 不自动注册适配器

  // 适配器管理
  void register_adapter(const SolverAdapterPtr& adapter);
  std::size_t register_default_adapters();               // 返回注册数量

  // 偏好覆盖
  void set_solver_preference(ProblemClass cls, const std::string& adapter_name);
  std::vector<std::string> list_solvers(ProblemClass cls) const;

  // 通用派发（类型安全 variant）
  api::Result solve(const api::ProblemVariant& problem,
                    const SolveOptions& options = {}) const;

  // 按类型入口
  api::Result solve_le   (const SparseLinSys&  p, const SolveOptions& o = {}) const;
  api::Result solve_nle  (const NonlinearSystem& p, const SolveOptions& o = {}) const;
  api::Result solve_lp   (const LPModel&       p, const SolveOptions& o = {}) const;
  api::Result solve_qp   (const QPModel&       p, const SolveOptions& o = {}) const;
  api::Result solve_nlp  (const NLPModel&      p, const SolveOptions& o = {}) const;
  api::Result solve_milp (const MIPModel&      p, const SolveOptions& o = {}) const;
  api::Result solve_minlp(const MINLPModel&    p, const SolveOptions& o = {}) const;
  api::Result solve_conic(const ConicModel&    p, const SolveOptions& o = {}) const;
};
} // namespace mipsolvers::engine
```

`register_default_adapters()` 按偏好顺序注册的适配器：

| 适配器名 | 问题类别 |
|---|---|
| `NativeLinear` | LE |
| `NativeNewton` | NLE |
| `NativeAutoLP` | LP |
| `NativeDualSimplex` | LP |
| `NativeIPMLP` | LP |
| `NativePDLP` | LP |
| `NativeLCQP` | QP |
| `NativeIPM` | NLP |
| `NativeNLP` | NLP |
| `NativeConicIPM` | CONIC |
| `StrictHiGHS` | MILP |
| `NativeBranchAndCut` | MILP, MINLP |
| `Gurobi`（若可用） | LP, QP, MILP |
| `CPLEX`（若可用） | MILP |
| `HiGHS` | LP, MILP |
| `Ipopt` | NLP |
| `SCIP` | MINLP |

### 7.1.4 SolveOptions

```cpp
enum class StrategyPolicy {
  Auto,          // engine heuristic based on problem characteristics
  NativeFirst,   // prefer native adapters before external ones
  ExternalFirst, // prefer external (Gurobi, HiGHS, …) first
};

enum class PortfolioMode {
  Latency,    // race independent native LP kernels when threads >= 2
  Throughput, // run one direct IPM worker
};

struct SolveOptions {
  std::string   preferred_solver;        // exact adapter name, or "" for auto
  bool          allow_fallback{true};    // try next adapter if primary fails
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  // Per-class overrides (takes precedence over strategy_policy)
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
  double        time_limit_sec{0.0};     // call-wide soft deadline; 0 = unlimited
  int           threads{0};              // global worker budget; 0 = hardware concurrency
  std::uint32_t random_seed{0};
  std::size_t   memory_limit_bytes{0};    // advisory until reported enforced
  PortfolioMode portfolio_mode{PortfolioMode::Latency};
  std::stop_token stop_token{};           // cooperative caller cancellation
};
```

`time_limit_sec` 属于整个调用，而不是每个 adapter：所有 fallback 共享一个
`steady_clock` 绝对截止点，截止后不会再启动下一个 adapter。进程内 API 只能保证
cooperative soft deadline；需要不可逾越的 hard deadline 时，必须把求解放在独立
进程中并由 supervisor 终止进程树。`threads` 是全局预算，latency portfolio 在预算
不少于 2 时运行两个 worker，并把每个 worker 的嵌套线程限制为
`floor(threads/2)`；throughput 模式只运行 direct IPM。`memory_limit_bytes` 当前是
请求与遥测字段，必须检查结果中的 `memory_limit_enforced`，不得假设它已执行。

第三方 `SolverAdapter` 若只实现旧的无 `SolveContext` 虚函数，兼容性默认实现会回调
旧接口；这类 adapter 不会自动获得 soft deadline、线程预算或随机种子传播。通用求解器
的生产注册门禁必须要求其覆盖对应的 context overload，或把它放入带进程级 hard
deadline 的隔离 worker。内置适配器均已覆盖本模块实际使用的 context 路径。

`preferred_solver` 接受大小写敏感的适配器名字符串：`"Gurobi"`、`"HiGHS"`、
`"StrictHiGHS"`、`"SCIP"`、`"Ipopt"`、`"CPLEX"`、`"NativeBranchAndCut"`、`"NativeAutoLP"`、
`"NativeDualSimplex"`、`"NativeIPMLP"`、`"NativePDLP"`、`"NativeLCQP"`、
`"NativeLinear"`、`"NativeNewton"`、
`"NativeIPM"`、`"NativeNLP"`、`"NativeConicIPM"`。

`preferred_solver` 为空（Auto）时的候选构造顺序（`dispatcher.cpp:candidate_adapters`）：
`preferred_solver` → `set_solver_preference` 按类偏好 → 按问题类的内建偏好表
`default_priority_for`（LP：`NativeAutoLP`、`NativeIPMLP`、`NativePDLP`、
`NativeLCQP`、`HiGHS`；MILP：`StrictHiGHS`、`HiGHS`、`NativeBranchAndCut`；
NLP：`Ipopt`、`NativeIPM`、`NativeNLP`）→ 注册表中其余支持该类的适配器
（兜底级，已注册的 Gurobi/CPLEX 在此参与回退）。`NativeFirst`/`ExternalFirst`
对内建偏好表做稳定排序。若首选失败且 `allow_fallback = true`，按序尝试下一个候选。
详见 [求解器与引擎 §5.2.5](05-solvers-engines.md)。

示例 —— MILP 强制使用 Gurobi：

```cpp
SolveOptions opts;
opts.class_strategy_policy[ProblemClass::MILP] = StrategyPolicy::ExternalFirst;
opts.preferred_solver = "Gurobi";
auto result = eng.solve_milp(mip, opts);
```

### 7.1.5 结果结构

```cpp
namespace mipsolvers::engine::api {

struct Stats {
  bool        success{false};
  int         iterations{0};
  double      objective{0.0};
  double      residual_inf{0.0};    // max constraint violation
  double      primal_feas{0.0};     // primal feasibility
  double      dual_feas{0.0};       // dual feasibility
  double      complementarity{0.0};
  double      mip_gap{0.0};         // relative gap (MILP only)
  double      runtime_sec{0.0};
  int         thread_budget{0};
  int         portfolio_workers{0};
  int         worker_thread_limit{0};
  std::size_t memory_limit_bytes{0};
  bool        memory_limit_enforced{false};
  bool        hard_deadline_enforced{false};
  double      deadline_overrun_sec{0.0};
  bool        persistent_backend_reused{false};
  std::size_t incremental_update_count{0};
  std::string status;               // human-readable status string
  std::string solver_name;          // adapter that produced the result
  int         cglp_cuts_added{0};   // native B&C only
  // Farkas infeasibility certificate (populated when status = "Infeasible")
  Eigen::VectorXd farkas_ray;
  Eigen::VectorXd farkas_ray_eq;
  bool            has_farkas_certificate{false};
};

struct Result {
  Eigen::VectorXd x;                // primal solution (n)
  Stats           stats;
  // Shadow prices (dual variables). Layout: [ineq duals | eq duals]
  Eigen::VectorXd constraint_duals;
  // Variable bound multipliers (LP certificates only)
  Eigen::VectorXd box_dual_lb;      // z_l[j] for x[j] >= lb[j]
  Eigen::VectorXd box_dual_ub;      // z_u[j] for x[j] <= ub[j]
};
} // namespace mipsolvers::engine::api
```

资源遥测描述的是本次实际契约。当前进程内入口总是报告
`hard_deadline_enforced=false`；`deadline_overrun_sec` 记录 cooperative backend
在非中断操作中的超时量。使用 `LPModelSession` 时，
`persistent_backend_reused` 表示本次是否保留了已有 HiGHS 求解状态，
`incremental_update_count` 是 session 创建后的成功更新次数。
`portfolio_workers` 与 `worker_thread_limit` 分别给出 NativeAutoLP 本次实际启动的
worker 数和每个 worker 的嵌套线程上限；其他 adapter 保持为 0。

**对偶变量布局**：`constraint_duals` 前 `m_ineq` 个分量对应 `A*x <= b` 行，后
`m_eq` 个分量对应 `Aeq*x = beq` 行，所有返回对偶的适配器排序一致。

**对偶可用性**：`constraint_duals` 由 Gurobi、NativeBranchAndCut（LP 路径）、
NativeIPMLP 填充；MILP 结果为空。HiGHS 文件式适配器只解析原始解文件，
**不返回约束对偶** —— 若需要对偶（如 LMP 计算），请选用 `"Gurobi"`、
`"NativeBranchAndCut"` 或 `"NativeIPMLP"`。

### 7.1.6 Persistent/incremental LP

固定稀疏结构、反复修改目标/界/RHS 的滚动优化应使用 `LPModelSession`：

```cpp
LPModelSession session(std::move(lp));

SolveOptions options;
options.preferred_solver = "HiGHS";
options.allow_fallback = false;
options.threads = 2;

auto first = session.solve(options);
session.update_objective(new_cost);
session.update_variable_bounds(new_lb, new_ub);
session.update_inequality_rhs(new_row_lb, new_row_ub);
session.update_equality_rhs(new_eq_rhs);
auto next = session.solve(options);
```

在 `preferred_solver="HiGHS"` 且关闭 fallback 时，模型只装载一次，更新通过
HiGHS 的列成本、列界和行界增量接口执行，并保留兼容 basis。其他策略仍复用
session 内公共模型，但通过标准 dispatcher 求解。矩阵结构和维度不可变；尺寸错误、
无效上下界或后端拒绝会抛出异常，公共模型保持在更新前状态。一个 session 不支持
并发 mutation/solve；需要并发时为每个执行流创建独立 session。

### 7.1.7 SolverAdapter 接口

自定义或扩展求解器需实现 `SolverAdapter` 基类，并通过
`SolverEngine::register_adapter()` 注册：

```cpp
class SolverAdapter {
public:
  virtual std::string name() const = 0;
  virtual bool supports(ProblemClass cls) const = 0;

  virtual SolveResult solve_le   (const SparseLinSys& prob) const;
  virtual SolveResult solve_nle  (const NonlinearSystem& prob) const;
  virtual SolveResult solve_lp   (const LPModel& prob) const;
  virtual SolveResult solve_qp   (const QPModel& prob) const;
  virtual SolveResult solve_nlp  (const NLPModel& prob) const;
  virtual SolveResult solve_milp (const MIPModel& prob) const;
  virtual SolveResult solve_minlp(const MINLPModel& prob) const;
  virtual SolveResult solve_conic(const ConicModel& prob) const;

  // Override context overloads to consume call-wide resources cooperatively.
  virtual SolveResult solve_lp(const LPModel& prob,
                               const SolveContext& context) const;
  virtual SolveResult solve_milp(const MIPModel& prob,
                                 const SolveContext& context) const;
};
```

旧的无 context virtual 保留源兼容性；默认 context overload 会转调旧接口。通用
adapter 应覆盖其支持的问题类型，把剩余时限、线程预算和随机种子映射到后端，并在
开始工作前检查 `stop_requested()`。

内置适配器一览：

| 适配器类 | `name()` | 支持 | 返回对偶 |
|---|---|---|---|
| `HighsAdapter`（外部，文件 I/O） | `"HiGHS"` | LP, MILP | 否 |
| `IpoptAdapter`（外部，NL 文件） | `"Ipopt"` | NLP | 否 |
| `ScipAdapter`（外部，文件 I/O） | `"SCIP"` | MINLP | 否 |
| `GurobiAdapter`（外部，进程内 C API，需许可证） | `"Gurobi"` | LP, QP, MILP | 是（LP） |
| `CplexAdapter`（外部，Callable Library，需许可证） | `"CPLEX"` | MILP | 否 |
| `NativeLinearAdapter` | `"NativeLinear"` | LE | N/A |
| `NativeNewtonAdapter` | `"NativeNewton"` | NLE | 否 |
| `NativeAutoLPAdapter` | `"NativeAutoLP"` | LP | 是（组合内核取先到者） |
| `NativeDualSimplexLPAdapter` | `"NativeDualSimplex"` | LP | 是 |
| `NativeIPMLPAdapter` | `"NativeIPMLP"` | LP | 是（Mehrotra 预测-校正 IPM） |
| `NativePDLPAdapter` | `"NativePDLP"` | LP | —（一阶方法，面向大规模稀疏 LP） |
| `NativeLCQPAdapter` | `"NativeLCQP"` | QP | —（线性约束凸 QP） |
| `NativeIPMAdapter` | `"NativeIPM"` | NLP | 否 |
| `NativeNLPAdapter` | `"NativeNLP"` | NLP | 否 |
| `NativeBranchAndCutAdapter` | `"NativeBranchAndCut"` | MILP, MINLP | 是（LP 路径） |
| `NativeConicIPMAdapter` | `"NativeConicIPM"` | CONIC | 是（`constraint_duals = [z | y]`） |

外部适配器的可用性探测：`HighsAdapter::available()` 在 PATH 上查找 HiGHS 可执行
文件；`GurobiAdapter::available()` 检查许可证（`GRB_LICENSE_FILE` 或标准搜索路径）。

`SparseLinearSolver` 的 `supports_numeric_refactor()` 和 `refactorize()` 是固定稀疏
模式的可选能力。默认实现保持完整 `factorize()` 行为；KLU 实现仅在已有成功数值
分解且压缩列结构完全一致时执行固定主元数值重分解。调用方必须把 `false` 视为
重新 `analyze_pattern()` 后完整分解的信号，不能把它解释为最终求解失败。

### 7.1.7 C++ 最小完整示例

Windows/main 集成后的 NLP 接口同时保留以下控制：

- `NLPSolverOptions::acceptable_iterations=0` 默认禁用 Ipopt 的连续 acceptable
  退出；`adaptive_barrier=false` 默认使用 monotone barrier，可显式开启 adaptive。
- `primal_dual_warm_start=true` 要求 `NLPModel::x0` 与三个完整对偶向量：
  `constraint_dual_start=[非线性不等式 | 等式]`，以及按原变量顺序排列的
  `box_dual_lb_start`、`box_dual_ub_start`。缺失、维度错误或无效的对偶数据必须拒绝。
  当前适配器仍会将缺失或维度错误的 `x0` 补为边界感知初值，因此调用方应显式
  传入原始 KKT 点，不能用 `warm_start_used` 推断原始 primal 向量通过了完整校验。
  `warm_start_push` 默认 `1e-8`；`SolveStats::warm_start_used` 记录该路径。
  此契约适用于编入 Ipopt 库的进程内路径。
- Native `IPMOptions::primary_max_iter_before_restoration=0` 默认不单独限制
  首次尝试；正值限制首次 filter 尝试，restoration 使用自身预算。诊断字段
  `IPMDetail::initial_attempt_iterations` 和 `initial_attempt_factorizations`
  分别记录首次迭代和数值分解数。它不是整个求解链的总预算。

对应实现为 `problem_types.hpp`、`external/adapters.cpp` 和
`kernel/ipm/ipm_solver.cpp`；回归入口为 `test_engine_api` 与 `test_ipm_solver`。

```cpp
#include "mipsolvers/engine/engine.hpp"

using namespace mipsolvers::engine;

// Build an LP
LPModel lp;
lp.sense = Sense::Minimize;
lp.c = Eigen::VectorXd::Ones(2);          // min x0 + x1
// Aeq * x = beq  (x0 + x1 = 1)
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

MILP 只需复用同一 `LPModel` 字段并标注整数变量：

```cpp
MIPModel mip;
mip.linear_part = lp;                    // same LPModel fields
mip.binary_idx  = {0};                   // variable 0 is binary
auto result = eng.solve_milp(mip);
```

---

## 7.2 Python API

模块由 pybind11 绑定生成，需 Python 3.8+ 与 numpy ≥ 1.19。主要子模块：

| 子模块 | 功能 |
|---|---|
| `mipsolvers.engine` | 矩阵式 LP / MILP 求解接口 |
| `mipsolvers.aml` | 集合索引的高级建模接口（AML） |
| `mipsolvers.scuc` | SCUC/SCED/LMP 市场出清（见 [工业应用](08-industrial-applications.md)） |

### 7.2.1 engine 求解接口

```python
mipsolvers.engine.solve_lp(c, A, b, Aeq, beq, lb, ub, solver, maximize) -> dict
mipsolvers.engine.solve_milp(c, A, b, Aeq, beq, lb, ub, vartypes,
                             solver, mip_gap, time_limit_sec, maximize) -> dict
mipsolvers.engine.list_solvers(problem_class) -> list[str]
```

求解标准形式 LP / MILP：`min cᵀx`，s.t. `Ax ≤ b`，`Aeq·x = beq`，`lb ≤ x ≤ ub`。

`solve_milp` 参数表（`solve_lp` 参数为其子集）：

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `c` | np.ndarray (n,) | 必填 | 目标系数 |
| `A` | np.ndarray (m,n) | None | 不等式约束矩阵 |
| `b` | np.ndarray (m,) | None | 不等式 RHS |
| `Aeq` | np.ndarray (p,n) | None | 等式约束矩阵 |
| `beq` | np.ndarray (p,) | None | 等式 RHS |
| `lb` | np.ndarray (n,) | None（−∞） | 变量下界 |
| `ub` | np.ndarray (n,) | None（+∞） | 变量上界 |
| `vartypes` | list[str] | []（全连续） | 变量类型：`"C"` 连续 / `"I"` 整数 / `"B"` 二进制 |
| `solver` | str | `"Auto"` | 求解器选择（见下表） |
| `mip_gap` | float | 1e-4 | 相对 MIP 间隙容差 |
| `time_limit_sec` | float | 300.0 | 时间限制（秒） |
| `maximize` | bool | False | True = 最大化 |

Python 层求解器名字符串与适用问题：

| 求解器名 | 适用问题 | 说明 |
|---|---|---|
| `Auto` | 任意 | 按内置优先级自动选择；MILP 顺序为 Gurobi → StrictHiGHS → HiGHS → NativeBranchAndCut |
| `Gurobi` | LP / MILP | 商业求解器（需许可证） |
| `StrictHiGHS` | MILP | 生产默认：嵌入式 HiGHS 状态机 + MIPSolvers 合约 |
| `HiGHS` | LP / MILP | 直接 HiGHS 适配器 |
| `NativeIPMLP` | LP | 内置内点法（小型 LP） |
| `NativePDLP` | LP | 内置一阶方法（大规模 LP） |
| `NativeLCQP` | LP / QP | 内置线性化 CQP |
| `NativeBranchAndCut` | MILP | 内置分支定界 |
| `Ipopt` | NLP / MINLP | 内置非线性求解器 |

> Python 层把 `solver` 参数直接写入 `SolveOptions::preferred_solver`（见
> `src/python/mipsolvers_py.cpp`），因此与 C++ 层注册名完全一致；跨语言迁移配置时
> 如有疑问，以 `list_solvers()` 返回的名字为准。

### 7.2.2 结果判读（返回字典）

`solve_lp` 返回键：

| 键 | 类型 | 说明 |
|---|---|---|
| `success` | bool | 是否找到最优解 |
| `objective` | float | 最优目标值 |
| `x` | np.ndarray (n,) | 最优解向量 |
| `duals` | np.ndarray | 约束对偶（不等式 + 等式；HiGHS 不支持） |
| `status` | str | 求解器状态字符串 |
| `solver` | str | 实际调用的求解器名 |
| `iterations` | int | 迭代次数 |
| `runtime_sec` | float | 求解用时（秒） |

`solve_milp` 返回键在 LP 基础上以 `mip_gap`（最终 MIP 间隙）替代 `duals` 与
`iterations`：`success` / `objective` / `x` / `mip_gap` / `status` / `solver` /
`runtime_sec`。

与 C++ 结果结构的对照：

| Python 键 | C++ 字段 |
|---|---|
| `success` | `stats.success` |
| `objective` | `stats.objective` |
| `x` | `x` |
| `duals` | `constraint_duals`（布局同为 [不等式对偶 \| 等式对偶]） |
| `status` | `stats.status` |
| `solver` | `stats.solver_name` |
| `iterations` | `stats.iterations` |
| `mip_gap` | `stats.mip_gap` |
| `runtime_sec` | `stats.runtime_sec` |

> **注意**：HiGHS 作为 MILP 求解器时不返回约束对偶，`duals` 键可能缺失或为空
> 数组。需要对偶变量时请使用 `solver="Gurobi"` 或原生 IPM 求解器。
> 求解失败（无解、超时）**不抛异常**，而是返回 `success=False`，调用方必须
> 检查该标志。

### 7.2.3 异常处理

所有错误以标准 Python 异常抛出：

| 异常类型 | 触发场景 |
|---|---|
| `RuntimeError` | JSON 格式错误、字段类型错误；求解器初始化失败（如许可证无效） |
| `ValueError` / `invalid_argument` | numpy 数组维度不符 |
| 返回 `success=False` | 问题无解或超时（不抛异常） |

```python
import numpy as np
import mipsolvers

try:
    # c 有 3 个变量，但 A 有 4 列 → 维度错误
    mipsolvers.engine.solve_lp(
        c = np.ones(3),
        A = np.ones((2, 4)),
        b = np.ones(2)
    )
except Exception as e:
    print(f"维度错误: {e}")
```

### 7.2.4 Python 最小完整示例

```python
import numpy as np
import mipsolvers

# min -x0 - 2*x1
# s.t. x0 + x1 <= 4
#      2x0 + x1 <= 6
#      x0, x1 >= 0
c  = np.array([-1.0, -2.0])
A  = np.array([[1.0, 1.0],
               [2.0, 1.0]])
b  = np.array([4.0, 6.0])
lb = np.zeros(2)

res = mipsolvers.engine.solve_lp(c, A=A, b=b, lb=lb, solver="Auto")

print(res["success"])     # True
print(res["objective"])   # -8.0
print(res["x"])           # [0. 4.]
print(res["duals"])       # 对偶变量（若求解器支持）
print(res["solver"])      # 实际使用的求解器名
```

### 7.2.5 AML 建模接口（`mipsolvers.aml`）

AML 提供免手工拼矩阵的集合索引建模方式，编译后走同一求解引擎。核心对象：

| 对象 | 说明 | 关键成员 |
|---|---|---|
| `aml.Model(name="")` | 模型容器 | `num_vars`、`num_constraints`、`solve()`、`write_lp()` / `write_mps()` |
| `aml.Key(atoms)` | 多维索引 | `Key.scalar("P1")`、`Key.pair("P1","C1")`、`Key.make([...])`；可用 `str` / `tuple` 简写 |
| `ExplicitSet` / `OrderedSet` | 集合（`add_set()` / `add_ordered_set()`） | `elements`、`contains()`、`add_element()`；OrderedSet 另有 `at()`、`prev()`、`next()` |
| `Parameter` | 参数表（`add_param(name, dim, unit)`） | `set()` / `load(dict)` / `get()` / `get_or()` / `to_dict()` |
| `VarType` | 枚举 | `Continuous` / `Integer` / `Binary` |
| `VarArray` | 变量族（`add_var(name, domain, type, lb, ub)`、`add_var2(...)`） | `a[key]` 取 `VarRef`、`set_lb()` / `set_ub()` / `fix()` |
| `LinearExpr` / `QuadExpr` / `NonlinearExpr` | 表达式 | 支持 `+ - *` 与比较运算（产出 `TempConstr`） |
| `TempConstr` → `ConstraintRef` / `ConstraintArray` | 约束 | `add_constraint(tc, name)`、`add_constraints(family, domain, fn)` |

目标函数：`minimize()` / `maximize()`（线性），`minimize_quad()` /
`maximize_quad()`（二次），`minimize_nl()` / `maximize_nl()`（非线性，配合
`m.nl_*` 系列表达式构建器与 `set_nlp_x0()` 初始点）。非线性约束用
`add_nl_constraint(name, expr, sense, rhs)`，`sense` 取 `"<="` / `">="` / `"=="`
（或 `"le"` / `"ge"` / `"eq"`）。

`aml.SolveOptions`（对应 C++ 侧求解配置，字段名不同）：

```python
opts = aml.SolveOptions()
opts.solver_name    = ""       # "" = auto, "highs", "gurobi", "native", …
opts.time_limit_sec = 1e30
opts.mip_gap_tol    = 1e-4
opts.verbosity      = 0        # 0=silent, 1=summary, 2=verbose
```

`Model.solve()` 返回 `SolveResult`，结果判读字段：

| 属性 / 方法 | 类型 | 说明 |
|---|---|---|
| `termination_status` | 枚举 | `Optimal` / `Infeasible` / `Unbounded` / `TimeLimit` / `NodeLimit` / `NumericalError` 等（完整枚举见下） |
| `primal_status` / `dual_status` | 枚举 | `Optimal` / `Feasible` / `Infeasible` / `NoSolution` / `Unknown` |
| `is_optimal` / `has_primal` / `has_duals` | bool | 快速判定标志 |
| `objective_value` / `objective_bound` / `optimality_gap` | float | 目标值 / MIP 松弛界 / 间隙 |
| `solve_time_sec` / `simplex_iterations` / `branch_and_cut_nodes` | — | 性能统计 |
| `solver_used` | str | 实际求解器 |
| `var_value(ref)` / `array_values(var_array)` | — | 原始解取值（`array_values` 返回 `{key: value}` 字典） |
| `dual(constraint_ref)` / `reduced_cost(var_ref)` | float \| None | 对偶与简约费用（不可用时为 None） |

`TerminationStatus` 完整枚举：`Optimal`, `Infeasible`, `Unbounded`,
`InfeasibleOrUnbounded`, `TimeLimit`, `IterationLimit`, `NodeLimit`,
`ObjectiveLimit`, `NumericalError`, `UserInterrupt`, `SolverError`, `Unknown`。

辅助函数 `aml.sum_over(set, fn)` / `aml.sum_over_quad(set, fn)` 对集合求和生成
表达式。诊断与导出：`print_summary()`、`check_bounds()`（lb > ub 时报错）、
`check_missing_params()`（标量未赋值、索引表为空或非有限值时报错）、
`write_lp()` / `write_mps()` / `write_json()`（Beta）。AML 完整用法与示例见
[AML 建模](04-modeling-aml.md)。

### 7.2.6 求解器可用性查询与性能要点

```python
import mipsolvers
print(mipsolvers.engine.list_solvers("LP"))
print(mipsolvers.engine.list_solvers("MILP"))
```

性能要点（源文档建议）：

- `solve_lp` / `solve_milp` / AML `solve()` 在求解阶段自动释放 GIL，支持多线程并发调用；
- 矩阵式接口接受稠密 numpy 数组并内部转稀疏，大型稀疏问题请尽量减少非零元；
- 求解器选择经验：LP 优先 `"HiGHS"`（无需许可证）、大规模可试 `"NativePDLP"`；
  MILP 有许可证优先 `"Gurobi"`，否则 `"HiGHS"`；SCUC 超 100 台机组建议
  `"Gurobi"` 并放宽 `mip_gap=0.005`。


## 7.3 两阶段随机与鲁棒优化

C++ 头文件为 `mipsolvers/engine/decomposition/two_stage.hpp`，命名空间为
`mipsolvers::engine::decomposition`。Python 扩展启用后，对应入口在
`mipsolvers.decomposition`：`solve_stochastic`、`solve_robust` 和
`solve_robust_polyhedral`，参数使用字典与 NumPy 数组。下表列出 C++ 函数。
本模块直接使用求解引擎，不依赖 SCUC 数据结构。

`TwoStageModel` 包含 `FirstStage` 与各场景的 `Recourse`；场景约束统一为
`W y >= h - T x`。随机模型按场景概率加权，鲁棒模型取最坏场景成本。

| 入口 | 用途与限制 |
|---|---|
| `solve_extensive_form_stochastic` / `solve_extensive_form_robust` | 确定性等价整体模型，可作为分解结果的对照 |
| `solve_benders_stochastic` | 随机规划；`MultiCut` / `SingleCut` 使用连续追索；`IntegerLShaped` / `Lagrangian` 支持整数追索，要求纯二进制第一阶段 |
| `solve_ccg_robust` | 有限场景集合的鲁棒优化，追索变量可连续或整数 |
| `solve_ccg_polyhedral_robust` | `PolyhedralRobustModel` 的多面体不确定集；要求连续、下界为零且无有限上界的追索变量，以及相对完全追索 |

`BendersOptions` 和 `CCGOptions` 的 `threads` 控制场景并行度，默认 1；
`time_limit_sec=0` 表示无调用时限，`gap_tolerance` 默认 `1e-6`。
Benders 可配置 `stabilization_alpha`、追索下界和 Lagrangian 内层迭代预算。
多面体 CCG 的 `big_m` 必须覆盖有效 KKT 解；触及大 M 安全阈值会明确拒绝结果。

`DecompositionResult` 返回 `success`、`status`、第一阶段解 `x`、场景解 `y`、
目标值、上下界、相对间隙及迭代统计。必须检查状态；可行 incumbent 不等于
最优性证明，未证明最优的子问题不能用于生成已认证的割或更新界。
详细模型假设与推导见[两阶段分解设计](../archive/two_stage_decomposition_design.md)。

组合 LP 求解新增 `portfolio_first_result_sec` 与 `portfolio_cancel_wait_sec`
统计，分别记录首个可返回结果的等待时间和取消后等待工作线程退出的时间。
调用返回前会等待所有工作线程结束；这是协作式取消，不是进程级硬截止。
本分支继续保留内存预算、deadline overrun 和持久后端复用统计。
