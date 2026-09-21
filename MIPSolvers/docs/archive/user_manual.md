# MIPSolvers 用户手册

> 适用版本：当前工作树（2026-08-05）。本手册面向建模、调用和结果判读；算法
> 原理与源码审核请阅读[求解器实现与算法审查手册](solvers.md)。

## 1. 选择使用方式

MIPSolvers 提供四种入口。按使用场景选择，不需要直接调用 `src/` 中的内核。

| 入口 | 适合场景 | 主要头文件/模块 |
|---|---|---|
| C++ Engine | 已有稀疏矩阵，要求低开销和完整控制 | `mipsolvers/engine/engine.hpp` |
| C++/Python AML | 希望用集合、变量和表达式建模 | `mipsolvers/aml/aml.hpp` / `mipsolvers.aml` |
| Python 数组接口 | NumPy 中已有 LP/MILP 矩阵 | `mipsolvers.engine` |
| SCUC 专用接口 | 机组组合、SCED、LMP 和标准算例 | `mipsolvers/scuc/scuc.hpp` / `mipsolvers.scuc` |

推荐原则：

- 新业务模型优先使用 AML；它负责变量编号、矩阵编译和结果映射。
- 大规模矩阵由其他程序生成时使用 C++ Engine，避免重复建模开销。
- 数据分析和快速实验使用 Python 数组接口。
- 电力市场出清直接使用 SCUC API，不要手工复制其约束。

## 2. 构建与安装

### 2.1 基本要求

- CMake 3.20 或更高版本；
- 支持 C++20 的编译器；
- 64 位构建环境；
- Python 入口需要 Python、NumPy 和 pybind11；
- Gurobi 等商业后端需要单独的运行许可证。

完整依赖、离线包和平台说明见[构建与部署](build_and_deploy.md)。

### 2.2 从源码构建 C++ 库

Windows（在 Visual Studio x64 Developer PowerShell/Command Prompt 中）：

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release -L unit
```

已有 vcpkg 环境时可使用：

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset windows-vcpkg-release
ctest --preset windows-vcpkg-release -L unit
```

Linux/macOS 的通用方式：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build -L unit --output-on-failure
```

安装到独立前缀：

```bash
cmake --install build --prefix /path/to/mipsolvers-install
```

多配置生成器（Visual Studio、Ninja Multi-Config）安装时增加
`--config Release`。

### 2.3 在其他 CMake 工程中使用

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_optimizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
find_package(mipsolvers REQUIRED)

add_executable(my_optimizer main.cpp)
target_link_libraries(my_optimizer PRIVATE mipsolvers::mipsolvers)
```

配置消费工程：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/mipsolvers-install
cmake --build build
```

也可以在同一源码树中使用
`add_subdirectory(/path/to/MIPSolvers EXCLUDE_FROM_ALL)`。

### 2.4 构建 Python 模块

```bash
python -m pip install numpy pybind11
cmake -S . -B build-py \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_BUILD_PYTHON=ON \
  -DPYTHON_EXECUTABLE=/path/to/python
cmake --build build-py --target mipsolvers_py
```

把生成扩展所在目录加入 `PYTHONPATH`，然后验证：

```bash
python -c "import mipsolvers; print(mipsolvers.engine.list_solvers('LP'))"
```

必须用与扩展编译时 ABI 兼容的 Python 解释器；不能把另一个 Python 版本生成的
`.pyd`/`.so` 直接复制过来。

## 3. C++ 十分钟上手

下面求解线性规划：

```text
min  2 x + 3 y
s.t. x + y >= 4
     x >= 0, y >= 0
```

Engine 的不等式默认写成 `A x <= b`，所以约束需要乘以 `-1`。

```cpp
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "mipsolvers/engine/engine.hpp"

int main() {
  using namespace mipsolvers::engine;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::Vector2d(2.0, 3.0);

  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = -1.0;
  lp.A.insert(0, 1) = -1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, -4.0);

  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {
      VariableMeta{VarType::Continuous, 0.0, 1e20, "x"},
      VariableMeta{VarType::Continuous, 0.0, 1e20, "y"},
  };

  SolverEngine engine;
  SolveOptions options;
  options.preferred_solver = "HiGHS";
  options.allow_fallback = true;

  const auto result = engine.solve_lp(lp, options);
  if (!result.stats.success) {
    std::cerr << result.stats.solver_name << ": "
              << result.stats.status << '\n';
    return 1;
  }

  std::cout << "objective=" << result.stats.objective << '\n'
            << "x=" << result.x.transpose() << '\n';
}
```

不要只检查 `x` 是否非空。生产代码至少检查 `success`、`status`、
`solver_name` 和相应的可行度/gap。

## 4. Engine 数据约定

### 4.1 公共类型

```cpp
using namespace mipsolvers::engine;
```

| 类型 | 说明 |
|---|---|
| `Sense::Minimize/Maximize` | 目标方向 |
| `VarType::Continuous/Integer/Binary` | 变量类型 |
| `VariableMeta` | 类型、下界、上界和名称 |
| `Eigen::SparseMatrix<double>` | 公共稀疏矩阵，默认列主序 |
| `Eigen::VectorXd` | 目标、右端、初值和结果向量 |

下标均从 0 开始。变量数组、`c`、矩阵列数必须一致。公共模型把约
`-1e20/+1e20` 视为无界哨兵；新代码也可使用 IEEE 无穷，但不能传入 NaN。

### 4.2 约束方向

- LP/MILP/QP：`A x <= b`，`Aeq x = beq`。
- LP 的可选 `row_lhs` 可表达 `row_lhs <= A x <= b`；为空表示全部下侧无界。
- NLP：`g(x)=0`，`h(x)<=0`。
- 锥规划：`Gx+s=h, Ax=b, s in K`。

没有某类约束时仍要让矩阵列数等于变量数，例如 `Aeq.resize(0, n)`，右端向量
长度为 0。

### 4.3 求解器选择

```cpp
SolverEngine engine;
for (const auto& name : engine.list_solvers(ProblemClass::LP)) {
  std::cout << name << '\n';
}

SolveOptions options;
options.preferred_solver = "NativeIPMLP"; // 空字符串表示自动选择
options.allow_fallback = false;
options.strategy_policy = StrategyPolicy::Auto;
```

名称区分大小写，并以当前构建的 `list_solvers` 输出为准。

| 名称 | 类型 | 推荐用途 |
|---|---|---|
| `HiGHS` | LP/MILP | 通用开源生产后端 |
| `StrictHiGHS` | MILP | 使用项目规定的 HiGHS MIP 合约 |
| `NativeBranchAndCut` | MILP/MINLP | 自研树搜索、研究和定制回调 |
| `NativeIPMLP` | LP | 原生 LP 内点法；研究/影子路径，必须保留解后审计和 fallback |
| `NativePDLP` | LP | 一阶法研究路径；当前 NETLIB 严格门槛下不用于生产 |
| `NativeLCQP` | LP/QP | 凸 QP 实验路径；LP 自动路由当前不应选择 |
| `NativeIPM` | NLP | 原始-对偶滤子内点法 |
| `NativeNLP` | NLP | 轻量罚函数 Newton |
| `NativeConicIPM` | CONIC | LP/SOCP/SDP 锥内点法 |
| `NativeLinear` | LE | 稀疏线性方程 |
| `NativeNewton` | NLE | 通用非线性方程 |
| `Ipopt` | NLP | 外部成熟 NLP 后端 |
| `SCIP` | MILP/MINLP | 外部整数/非线性整数后端 |
| `Gurobi` | LP/QP/MILP | 商业后端，需编译支持和许可证 |

`allow_fallback=true` 表示首选后端失败后可尝试其他适配器。需要可重复审计时，
显式选择求解器并关闭 fallback，否则 `solver_name` 可能与首选名称不同。

### 4.4 结果判读

`api::Result` 的主要字段：

| 字段 | 含义 |
|---|---|
| `x` | 原始变量解 |
| `stats.success` | 当前适配器是否按其成功条件终止 |
| `stats.status` | 人类可读的终止原因 |
| `stats.solver_name` | 实际产生结果的适配器 |
| `stats.objective` | 原目标方向下的目标值 |
| `stats.primal_feas` | 原始可行性指标 |
| `stats.dual_feas` | 对偶/驻点可行性指标 |
| `stats.complementarity` | 互补残差 |
| `stats.relative_primal_residual` | 原模型归一化 primal residual；不支持时为 NaN |
| `stats.relative_dual_residual` | 原模型归一化 stationarity/dual residual；不支持时为 NaN |
| `stats.relative_gap` | 原模型 primal/dual objective relative gap；不支持时为 NaN |
| `stats.dual_objective` | 原目标方向下的对偶目标；无可审计对偶点时为 NaN |
| `stats.mip_gap` | MILP 相对 gap |
| `stats.runtime_sec` | 求解耗时 |
| `constraint_duals` | `[不等式行对偶, 等式行对偶]`，可用时填充 |
| `box_dual_lb/ub` | 变量下界/上界乘子，可用时填充 |

建议的检查模式：

```cpp
if (!result.stats.success) {
  // 根据 status 区分 infeasible、unbounded、limit、numerical failure。
}
if (result.x.size() != expected_n || !result.x.allFinite()) {
  // 不使用尺寸错误或非有限的候选解。
}
if (is_milp && result.stats.mip_gap > required_gap) {
  // 可能有 incumbent，但未达到业务证明要求。
}
```

MILP 的变量对偶通常没有定义；不要假设 `constraint_duals` 总是存在。需要 LMP
时应在固定整数决策后的 LP/SCED 上读取对偶。

## 5. 按问题类型建模

### 5.1 线性方程 LE

```cpp
SparseLinSys system;
system.A.resize(2, 2);
system.A.insert(0, 0) = 3.0;
system.A.insert(0, 1) = 1.0;
system.A.insert(1, 0) = 1.0;
system.A.insert(1, 1) = 2.0;
system.A.makeCompressed();
system.b = Eigen::Vector2d(9.0, 8.0);

SolverEngine engine;
auto result = engine.solve_le(system);
```

`A` 必须为方阵。成功后仍可用 `||Ax-b||_inf` 对业务尺度进行复核。

### 5.2 非线性方程 NLE

```cpp
NonlinearSystem problem;
problem.n = 2;
problem.x0 = Eigen::Vector2d(1.0, 1.0);
problem.residual = [](const Eigen::VectorXd& x, Eigen::VectorXd& f) {
  f.resize(2);
  f[0] = x[0] * x[0] + x[1] - 3.0;
  f[1] = x[0] + x[1] * x[1] - 3.0;
};
problem.jacobian = [](const Eigen::VectorXd& x,
                      Eigen::SparseMatrix<double>& j) {
  j.resize(2, 2);
  j.setZero();
  j.insert(0, 0) = 2.0 * x[0];
  j.insert(0, 1) = 1.0;
  j.insert(1, 0) = 1.0;
  j.insert(1, 1) = 2.0 * x[1];
  j.makeCompressed();
};

SolveOptions options;
options.preferred_solver = "NativeNewton";
auto result = SolverEngine{}.solve_nle(problem, options);
```

回调必须在每次调用时完整写入输出，不能返回悬空引用。物理变量需要投影时设置
`problem.project`。

### 5.3 LP 与双侧行

LP 字段为 `sense, c, A, row_lhs, b, Aeq, beq, vars`。例如
`1 <= x+y <= 5`：

```cpp
lp.A.resize(1, 2);
lp.A.insert(0, 0) = 1.0;
lp.A.insert(0, 1) = 1.0;
lp.A.makeCompressed();
lp.row_lhs = Eigen::VectorXd::Constant(1, 1.0);
lp.b = Eigen::VectorXd::Constant(1, 5.0);
```

不要把变量界重复拼成稠密约束行；直接写入 `VariableMeta.lb/ub`。

### 5.4 凸 QP

目标定义为 `0.5 x' Q x + c' x`。`Q` 应对称、半正定：

```cpp
QPModel qp;
qp.sense = Sense::Minimize;
qp.Q.resize(2, 2);
qp.Q.insert(0, 0) = 2.0;
qp.Q.insert(1, 1) = 2.0;
qp.Q.makeCompressed();
qp.c = Eigen::Vector2d(-2.0, -4.0);
qp.A.resize(0, 2);
qp.b.resize(0);
qp.Aeq.resize(0, 2);
qp.beq.resize(0);
qp.vars = {
    VariableMeta{VarType::Continuous, 0.0, 10.0, "x"},
    VariableMeta{VarType::Continuous, 0.0, 10.0, "y"},
};

SolveOptions options;
options.preferred_solver = "NativeLCQP";
auto result = SolverEngine{}.solve_qp(qp, options);
```

非凸 `Q` 不具备全局最优保证。带一般线性不等式的 QP 应先确认所选后端能力；
当前原生 LCQP 路径主要覆盖等式和箱界，通用 QP 可选择 Gurobi。

### 5.5 NLP

`NLPModel` 使用回调提供 `f`、`grad`、可选 Hessian、等式 `g/jac_g` 和不等式
`h/jac_h`。最少还要提供 `vars` 与 `x0`。

```cpp
NLPModel nlp;
nlp.sense = Sense::Minimize;
nlp.vars = {
    VariableMeta{VarType::Continuous, -2.0, 2.0, "x"},
    VariableMeta{VarType::Continuous, -1.0, 3.0, "y"},
};
nlp.x0 = Eigen::Vector2d(0.0, 0.0);
nlp.f = [](const Eigen::VectorXd& x) {
  return std::pow(x[0] - 1.0, 2) + std::pow(x[1] - 2.0, 2);
};
nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
  g = Eigen::Vector2d(2.0 * (x[0] - 1.0), 2.0 * (x[1] - 2.0));
};
nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& h) {
  h.resize(2, 2);
  h.setZero();
  h.insert(0, 0) = 2.0;
  h.insert(1, 1) = 2.0;
  h.makeCompressed();
};
nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
  g = Eigen::VectorXd::Constant(1, x[0] + x[1] - 2.0);
};
nlp.jac_g = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& j) {
  j.resize(1, 2);
  j.setZero();
  j.insert(0, 0) = 1.0;
  j.insert(0, 1) = 1.0;
  j.makeCompressed();
};

SolveOptions options;
options.preferred_solver = "NativeIPM";
auto result = SolverEngine{}.solve_nlp(nlp, options);
```

没有不等式时 `h/jac_h` 可留空。生产模型建议提供解析导数并用有限差分独立检查；
NLP 成功必须同时检查原始可行度、对偶可行度和互补度。

### 5.6 MILP

`MIPModel` 在 `LPModel` 上增加整数列索引：

```cpp
MIPModel mip;
mip.linear_part = lp;
mip.binary_idx = {0, 1};
mip.integer_idx = {};  // 不要把同一列同时放进两个数组
mip.linear_part.vars[0].type = VarType::Binary;
mip.linear_part.vars[0].lb = 0.0;
mip.linear_part.vars[0].ub = 1.0;
mip.linear_part.vars[1].type = VarType::Binary;
mip.linear_part.vars[1].lb = 0.0;
mip.linear_part.vars[1].ub = 1.0;

SolveOptions options;
options.preferred_solver = "StrictHiGHS";
options.allow_fallback = false;
auto result = SolverEngine{}.solve_milp(mip, options);
```

可用 `initial_solution` 提供长度为 `n` 的候选 incumbent。它只是提示，求解器会
验证可行性；不应依赖未验证初值作为最终结果。

统一 `SolveOptions` 只负责后端选择。需要给原生 B&C 设置时限、gap、线程等参数
时，注册带选项的适配器：

```cpp
#include <memory>
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"

BCOptions bc;
bc.time_limit_sec = 120.0;
bc.gap_tol = 1e-3;
bc.num_threads = 4;
bc.random_seed = 12345;

SolverEngine engine(false);
engine.register_adapter(std::make_shared<NativeBranchAndCutAdapter>(bc));

SolveOptions options;
options.preferred_solver = "NativeBranchAndCut";
options.allow_fallback = false;
auto result = engine.solve_milp(mip, options);
```

要求可重复树搜索时使用单线程，或同时设置确定性并行与固定
`random_seed`。达到 time limit 时可能已有可行 incumbent；是否可用于业务由
`status`、`x`、gap 和业务容差共同决定。

### 5.7 MINLP

`MINLPModel` 由 `NLPModel nonlinear_part` 和整数列索引组成，可选择
`NativeBranchAndCut` 或 `SCIP`。当前原生 MINLP 对一般非凸问题不提供全局最优
保证，适合实验或已知凸连续松弛；严格生产证明优先选择具备相应能力的外部后端。

### 5.8 锥规划

直接 Engine 接口采用 CVXOPT 标准型：

```text
min c'x
s.t. Gx+s=h, Ax=b, s in K
K = R_+^l x Q(q_1) x ... x S_+(p_1) x ...
```

`ConeDims.l/q/s` 描述各块。PSD 块使用 `svec`，非对角元素乘 `sqrt(2)`。手工
拼装前请阅读[锥规划专题](conic_sdp.md)；通常更推荐 AML 的
`add_soc_constraint`、`add_rotated_soc_constraint` 和 `add_psd_constraint`。

## 6. Python 数组接口

### 6.1 LP

```python
import numpy as np
import mipsolvers

c = np.array([2.0, 3.0])
A = np.array([[-1.0, -1.0]])
b = np.array([-4.0])
lb = np.array([0.0, 0.0])
ub = np.array([np.inf, np.inf])

result = mipsolvers.engine.solve_lp(
    c, A=A, b=b, lb=lb, ub=ub, solver="HiGHS"
)
if not result["success"]:
    raise RuntimeError(f'{result["solver"]}: {result["status"]}')
print(result["objective"], result["x"])
```

可选参数为 `A, b, Aeq, beq, lb, ub, solver, maximize`。返回字典包含
`success, objective, x, status, solver, iterations, runtime_sec`，后端提供对偶时
还包含 `duals`。

### 6.2 MILP

```python
result = mipsolvers.engine.solve_milp(
    c=np.array([-6.0, -5.0, -8.0]),
    A=np.array([[2.0, 3.0, 4.0]]),
    b=np.array([6.0]),
    lb=np.zeros(3),
    ub=np.ones(3),
    vartypes=["B", "B", "B"],
    solver="StrictHiGHS",
    mip_gap=1e-4,
    time_limit_sec=60.0,
    maximize=False,
)
print(result["status"], result["objective"], result["mip_gap"])
```

`vartypes` 使用 `C/I/B`，提供时长度必须等于变量数。`mip_gap` 和
`time_limit_sec` 当前只保证由 `Auto`、`StrictHiGHS` 和
`NativeBranchAndCut` 路径执行；其他外部后端返回字典会带提示。

高级研究接口 `solve_milp_bc(..., options=BCOptions())` 可设置节点、割、分支和
确定性选项。普通业务调用优先使用 `solve_milp`。

### 6.3 查询当前后端

```python
print(mipsolvers.engine.list_solvers("LP"))
print(mipsolvers.engine.list_solvers("MILP"))
```

可查询 `LE/NLE/LP/QP/NLP/MILP/MINLP`。返回顺序是当前构建的候选优先级，不能
假设所有机器完全相同。

## 7. AML 代数建模

### 7.1 Python 示例

```python
import mipsolvers

aml = mipsolvers.aml
model = aml.Model("transport")

plants = model.add_set("plants", ["P1", "P2"])
customers = model.add_set("customers", ["C1", "C2"])

supply = model.add_param("supply", dim=1, unit="unit")
demand = model.add_param("demand", dim=1, unit="unit")
cost = model.add_param("cost", dim=2, unit="currency/unit")
supply.load({"P1": 80.0, "P2": 70.0})
demand.load({"C1": 60.0, "C2": 75.0})
cost.load({("P1", "C1"): 2.0, ("P1", "C2"): 4.0,
           ("P2", "C1"): 3.0, ("P2", "C2"): 1.0})

x = model.add_var2(
    "x", plants, customers, aml.VarType.Continuous, lb=0.0
)
model.minimize(aml.sum_over(
    plants,
    lambda i: aml.sum_over(customers, lambda j: cost[i, j] * x[i, j])
))
model.add_constraints(
    "supply", plants,
    lambda i: aml.sum_over(customers, lambda j: x[i, j]) <= supply[i]
)
model.add_constraints(
    "demand", customers,
    lambda j: aml.sum_over(plants, lambda i: x[i, j]) >= demand[j]
)

options = aml.SolveOptions()
options.solver_name = "HiGHS"
options.time_limit_sec = 60.0
result = model.solve(options)

if not result.has_primal:
    raise RuntimeError(str(result.termination_status))
print(result.objective_value)
print(result.array_values(x))
```

### 7.2 AML 结果

- `termination_status`：为什么停止；
- `primal_status`：是否有原始解以及是否最优；
- `dual_status`：是否有 LP/锥对偶；
- `has_primal/is_optimal/has_duals`：常用判定；
- `objective_value/objective_bound/optimality_gap`：目标与证明界；
- `var_value`、`array_values`：读取变量；
- `dual`、`dual_array`、`reduced_cost`：读取可用的对偶信息。

AML 可导出线性模型：

```python
model.write_lp("model.lp")
model.write_mps("model.mps")
model.write_json("model.json")
```

LP/MPS 导出只支持线性 LP/MILP；QP、NLP、MINLP 和锥模型会明确拒绝，不会静默
丢弃非线性结构。完整 AML API 见 [AML Python API](aml_python_api.md)。

## 8. SCUC/SCED/LMP

### 8.1 命令行快速运行

先生成标准算例：

```powershell
scuc_case_builder --case 6bus --T 24 --wind --storage --output case.json
```

可选算例为 `3bus`、`6bus`、`ieee39`、`ieee118`；`--dt` 设置时段小时数，
`--solar` 和 `--wind` 加入新能源。

求解：

```powershell
scuc_solve case.json result.json --solver StrictHiGHS --indent 2
```

可用参数：

```text
--solver Auto|StrictHiGHS|HiGHS|SCIP|NativeBranchAndCut
--no-sced
--no-lmp
--indent <n>
```

未指定输出文件时，JSON 写到标准输出，进度与摘要写到标准错误。SCUC 未收敛时
进程返回非零退出码。

### 8.2 C++ 接口

```cpp
#include "mipsolvers/scuc/case_builder.hpp"
#include "mipsolvers/scuc/scuc.hpp"

using namespace mipsolvers::scuc;

SCUCInput input = build_6bus_case(24, 1.0, true, true);
input.config.solver = "StrictHiGHS";
input.config.time_limit_sec = 300.0;
input.config.mip_gap = 1e-3;
input.config.solve_sced = true;
input.config.solve_lmp = true;

SCUCOutput output = scuc_solve(input);
if (!output.scuc.converged) {
  // 检查 solver_name、mip_gap、目标以及求解日志。
}
```

三阶段含义：

1. `output.scuc`：MILP 机组组合和日前计划；
2. `output.sced`：固定组合后的连续经济调度；
3. `output.lmp`：节点电价、能量和阻塞分量。

### 8.3 Python 接口

对象方式：

```python
import mipsolvers

case = mipsolvers.scuc.build_6bus_case(
    T=24, dt=1.0, with_wind=True, with_storage=True
)
case.config.solver = "StrictHiGHS"
case.config.time_limit_sec = 300.0
case.config.mip_gap = 1e-3

output = mipsolvers.scuc.solve(case)
print(output.scuc.converged, output.scuc.total_cost)
print(output.lmp.avg_lmp)
```

JSON 一次调用：

```python
import json
import mipsolvers

with open("case.json", "r", encoding="utf-8") as f:
    result = json.loads(mipsolvers.scuc.solve_json(f.read(), indent=2))
print(result["scuc"]["converged"])
```

输入的完整字段、维度和 JSON 示例见 [SCUC 数据格式](data_format_spec.md)，模型
含义见 [SCUC 模块](scuc_module.md)，内置算例见[算例构造器](case_builder.md)。

## 9. 精度、性能与可重复性

- 使用 Release 构建进行性能测试；Debug 时间不能作为基准。
- 变量、约束和目标相差很多数量级时先做物理单位归一化。
- 稀疏矩阵构造完成后调用 `makeCompressed()`，重复结构尽量复用。
- LP 需要 basis 热启动时选择单纯形路径；PDLP 不产生 basis。
- MILP 首先设置合理 time limit 和 gap；不要只比较 wall time 而忽略证明质量。
- 对比后端时固定输入、线程、随机种子、预处理与停止条件。
- 需要确定性时优先单线程；并行 B&C 还应设置固定 seed 和
  `deterministic_parallel=true`。

当前平台的实测覆盖和已知失败见[测试与基准结果](testing.md)。
90 个 NETLIB LP 的本机结果表明：通用 LP 默认应选择 `HiGHS` 的 simplex
路径；大型稀疏 LP 可尝试 HiGHS IPM，但仍需解后审计并保留 simplex fallback。
Native IPM 经理论步长和原模型 KKT 审计停止增强后达到 81/90 accurate，但仍无
HSD 不可行/无界证书；
Native dual、PDLP、LCQP 和 SCIP LP adapter 也仍适合开发验证，不应仅凭
`Optimal`/`Solved` 状态进入生产。完整数据、例外案例和性能分层见
[NETLIB 线性规划求解器全面基准](netlib_benchmark.md)。

## 10. 常见问题

### 找不到求解器

先调用 `list_solvers`。外部后端只有在编译支持、可执行文件/动态库和许可证都
可用时才注册。名称必须严格匹配返回字符串。

### `success=false` 但 `x` 非空

一些迭代法会返回最佳已知点，MILP 达到时限也可能有 incumbent。它不是自动
可接受结果；检查 `status`、可行度、gap，并按业务规则决定是否使用。

### 没有对偶变量

MILP 本身通常不返回有意义的行对偶；外部文件型适配器也可能只解析原始解。
固定整数变量后重解 LP，或选择明确支持对偶返回的 LP 路径。

### NLP 返回 `unknown` 或数值失败

检查初值、函数定义域、导数、约束方向和尺度。用有限差分核对梯度/Jacobian，
避免初值正好位于 `log/sqrt/division` 的非法点。`unknown` 不能解释为不可行。

### Python 无法导入模块

确认 `PYTHONPATH` 指向扩展文件所在目录，并确认 Python 版本、体系结构和 MSVC/
libstdc++ ABI 与构建一致。Windows 还需要确保依赖 DLL 在扩展同目录或 `PATH`。

### Windows 编译器找不到 `stdio.h`

应从 Visual Studio x64 Developer 环境启动构建，或先调用 `VsDevCmd.bat`；仅把
`cl.exe` 加入 `PATH` 不会设置 Windows SDK 和标准库 include 路径。

### NETLIB 测试读文件失败

回归数据集不随源码部署。可运行
`python tools\fetch_netlib.py --output-dir tests\data` 下载、校验并生成 manifest，
再执行回归；读取错误与数值不收敛是两类问题。

## 11. 发布前检查清单

1. 用目标部署工具链完成 Release 构建。
2. 运行 unit、integration 和业务数据回归。
3. 调用 `list_solvers` 保存实际可用后端清单。
4. 用已知小模型核对目标、原始解、对偶和状态映射。
5. 对 MILP 记录 incumbent、best bound、gap、时限、线程和 seed。
6. 对 NLP/锥规划记录原始、对偶和互补残差。
7. 验证动态库、许可证和离线依赖在部署机可用。
8. 保存模型输入、软件提交、构建配置和结果日志以便复现。

## 12. 延伸文档

- [文档总览](README.md)
- [求解器实现与算法审查手册](solvers.md)
- [数值方法](numerical_methods.md)
- [Native LP 内点法理论设计](native_ipm_design.md)
- [测试与基准结果](testing.md)
- [NETLIB 线性规划求解器全面基准](netlib_benchmark.md)
- [Engine API](engine.md)
- [Python API](python_api.md)
- [AML Python API](aml_python_api.md)
- [SCUC 数据格式](data_format_spec.md)
- [构建与部署](build_and_deploy.md)
