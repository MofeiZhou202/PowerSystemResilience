# 第 1 章 概述

> 本章整合自: docs/archive/user_manual.md, docs/archive/solvers.md, docs/archive/README.md

本章回答三个问题：MIPSolvers 是什么、能解什么问题、适不适合你的现场。
后续的[安装与部署](02-installation-deploy.md)和[快速上手](03-quickstart.md)
会给出具体命令与代码。

## 1.1 MIPSolvers 是什么

MIPSolvers 是一个 C++20 数学规划求解器套件，统一承载八类问题：线性方程
（LE）、非线性方程（NLE）、线性规划（LP）、二次规划（QP）、非线性规划
（NLP）、混合整数线性规划（MILP）、混合整数非线性规划（MINLP）和锥规划
（CONIC，覆盖 LP/SOCP/SDP）。

套件同时提供自研求解内核和第三方后端适配器，上层暴露统一的 Engine API、
代数建模层（AML）以及面向电力系统的 SCUC/SCED/LMP 专用接口。四种使用入口：

| 入口 | 适合场景 | 主要头文件/模块 |
|---|---|---|
| C++ Engine | 已有稀疏矩阵，要求低开销和完整控制 | `mipsolvers/engine/engine.hpp` |
| C++/Python AML | 希望用集合、变量和表达式建模 | `mipsolvers/aml/aml.hpp` / `mipsolvers.aml` |
| Python 数组接口 | NumPy 中已有 LP/MILP 矩阵 | `mipsolvers.engine` |
| SCUC 专用接口 | 机组组合、SCED、LMP 和标准算例 | `mipsolvers/scuc/scuc.hpp` / `mipsolvers.scuc` |

推荐原则：新业务模型优先使用 AML；大规模矩阵由其他程序生成时使用 C++
Engine；数据分析和快速实验使用 Python 数组接口；电力市场出清直接使用
SCUC API，不要手工复制其约束。

## 1.2 能力矩阵

### 1.2.1 按问题类型

| 问题类型 | 模型 | 可用后端（名称区分大小写） |
|---|---|---|
| LE | 稀疏方阵 `A x = b` | `NativeLinear` |
| NLE | `F(x)=0` | `NativeNewton`（正则化 Newton）；另有 Dogleg 信赖域实现和电力潮流专用 Newton |
| LP | `min c'x, Ax<=b, Aeq x=beq` | `HiGHS`（默认生产选择）、`NativeIPMLP`、`NativePDLP`、原生对偶单纯形（LP kernel） |
| QP | `min 0.5 x'Qx + c'x`（凸） | `NativeLCQP`、`Gurobi` |
| MILP | LP + 整数/二进制列 | `StrictHiGHS`（项目规定的 HiGHS MIP 合约）、`HiGHS`、`NativeBranchAndCut`、`SCIP`、`Gurobi` |
| NLP | `g(x)=0, h(x)<=0` | `NativeIPM`（滤子原始-对偶内点法）、`NativeNLP`（轻量罚函数 Newton）、`Ipopt` |
| MINLP | NLP + 整数列 | `NativeBranchAndCut`、`SCIP`（原生路径为实验性局部求解，见 1.5） |
| CONIC | CVXOPT 标准型 `Gx+s=h, Ax=b, s in K` | `NativeConicIPM`（Mehrotra + NT 缩放，支持非负锥、二阶锥、半正定锥） |

可用后端以当前构建的 `list_solvers` 输出为准：

```cpp
SolverEngine engine;
for (const auto& name : engine.list_solvers(ProblemClass::LP)) {
  std::cout << name << '\n';
}
```

```python
import mipsolvers
print(mipsolvers.engine.list_solvers("LP"))
print(mipsolvers.engine.list_solvers("MILP"))
```

### 1.2.2 内建内核 vs 第三方后端

自研内核（位于 `src/engine` 的 `kernel/` 与 `solver/native/`）：

| 名称 | 问题类 | 算法/角色 |
|---|---|---|
| `NativeLinear` | LE | 稀疏直接法 |
| `NativeNewton` | NLE | 正则化 Newton + 回溯 |
| LP kernel | LP | 原生对偶单纯形（含 BFRT 比率检验、Forrest-Tomlin 基更新、热启动） |
| `NativePDLP` | LP | 一阶原始-对偶混合梯度；不产生 basis |
| `NativeIPMLP` | LP | Mehrotra 预测-校正原始-对偶障碍法 |
| `NativeLCQP` | LP/QP | 凸 QP 原始-对偶内点法 |
| `NativeNLP` | NLP | 二次罚函数 Newton |
| `NativeIPM` | NLP | Wachter-Biegler 型滤子原始-对偶 IPM |
| `NativeConicIPM` | CONIC | Mehrotra + Nesterov-Todd 缩放 |
| `NativeBranchAndCut` | MILP/MINLP | 自研分支割框架：割、启发式、reliability branching、并行 |

第三方后端适配器（编译支持、库和许可证齐备时才注册）：

| 名称 | 问题类 | 接入方式 |
|---|---|---|
| `HiGHS` | LP/MILP | 优先进程内库；部分配置保留 MPS/solution 临时文件路径 |
| `StrictHiGHS` | MILP | 通过 `make_strict_highs_production_options` 强制的 HiGHS MIP 合约，不是第二套自研 B&C |
| `Ipopt` | NLP | `CallbackTNLP` 回调映射 |
| `SCIP` | MILP/MINLP | 写 MPS/PIP 后调用可执行文件并解析 solution |
| `Gurobi` | LP/QP/MILP | 原生 C API；许可证不可用时不得注册为自动候选，不参与普通自动回退 |

说明：PaPILO 在本套件中用作 MILP 预处理（presolve）组件，与 HiGHS side
state、原生 `MILPPresolve` 并列，不作为独立求解器后端出现。

生产选型的当前结论（来自 90 个 NETLIB LP 本机基准，详见[测试与基准](09-testing-benchmarks.md)）：

- 通用 LP 默认选择 `HiGHS` 的 simplex 路径；大型稀疏 LP 可尝试 HiGHS IPM，
  但仍需解后审计并保留 simplex fallback。
- `NativeIPMLP` 经增强后达到 81/90 accurate，但仍无 HSD 不可行/无界证书，
  属于研究/影子路径，必须保留解后审计和 fallback。
- `NativePDLP`、原生 dual simplex、`NativeLCQP` 和 SCIP LP adapter 当前适合
  开发验证，不应仅凭 `Optimal`/`Solved` 状态进入生产。

## 1.3 典型工业应用场景

套件内置面向电力系统的三阶段出清流程，开箱即用：

1. **SCUC（安全约束机组组合）**：MILP，输出机组组合和日前计划；
2. **SCED（安全约束经济调度）**：固定整数组合后的连续 LP；
3. **LMP（节点边际电价）**：从 SCED 对偶读取能量和阻塞分量。

命令行快速运行：

```powershell
scuc_case_builder --case 6bus --T 24 --wind --storage --output case.json
scuc_solve case.json result.json --solver StrictHiGHS --indent 2
```

可选算例为 `3bus`、`6bus`、`ieee39`、`ieee118`；`--dt` 设置时段小时数，
`--solar` 和 `--wind` 加入新能源。C++/Python 接口见[工业应用](08-industrial-applications.md)。

除市场出清外，NLE 层包含电力潮流专用 Newton 实现（AC/DC 状态、PV/PQ 控制
切换、无功限值和 Levenberg-Marquardt 恢复），可支撑潮流/OPF 类模型求解。
一般工业优化（运输、排产、组合优化）通过 AML 或 Engine 直接建模。

## 1.4 系统架构

实现分三层：统一模型与选项 → 调度与预处理 → 求解器内核/适配器。

```text
api::ProblemVariant + api::SolveOptions        <- 八类问题的统一模型变体
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

- `SolverEngine` 负责注册和公共 API；`StrategyDispatcher` 负责候选排序、显式
  求解器选择、预处理及后处理。真正的迭代算法位于 `kernel/` 或
  `solver/native/`，外部适配器只完成模型转换、调用和结果映射。
- 公共模型定义集中在 `include/mipsolvers/engine/problem_types.hpp`，稀疏矩阵
  以 Eigen CSC 为公共表示；公共模型把约 `-1e20/+1e20` 视为无界哨兵，不能传
  入 NaN。
- 求解结果统一为 `api::Result`，包含原始解、可行度/gap 指标、对偶（可用时）
  与实际产生结果的 `solver_name`。`stats.success` 只表示该适配器按其成功条
  件终止，审核时还必须检查 `status`、原始/对偶可行度、互补度和 gap。
- 求解流程：规范化模型并校验 → 按问题类型和 `solver_name` 构造候选 → 预处理
  → 调用适配器 → 按策略决定是否回退到下一个后端（显式指定后端时不应静默
  冒充另一个后端）→ 后处理恢复原变量并返回统一结果。

AML 建模层位于 Engine 之上，负责变量编号、矩阵编译和结果映射；SCUC 模块再
位于 AML/Engine 之上，输出结构化 JSON。完整接口见[求解器与引擎](05-solvers-engines.md)
和[API 参考](07-api-reference.md)。

## 1.5 选择本套件的理由与边界

适用：

- 需要 LP/MILP/QP/NLP/锥规划统一接口和统一结果契约的 C++/Python 工程；
- 电力市场 SCUC/SCED/LMP 出清，以及潮流类方程求解；
- 需要自研内核做研究、定制回调（如原生 B&C 的节点、割、分支控制）或审计
  算法内部行为的场景；
- 需要在 HiGHS/SCIP/Ipopt/Gurobi 等后端之间按构建条件切换和对比的场景。

边界与限制（引用自源文档的"已知限制"，集成前必读）：

- **生产 LP/MILP 首选 `HiGHS`/`StrictHiGHS`**；多数原生 LP 路径（IPM、PDLP、
  dual simplex、LCQP）当前定位为研究或开发验证路径，进入生产必须保留解后
  审计和 fallback。
- `NativePDLP` 是中等精度一阶法，不提供 basis；不能用于要求严格节点证明
  或依赖基热启动的场景。
- `NativeIPMLP` 尚无 HSD 状态机，不能从普通迭代失败推出不可行或无界证书。
- `NativeNLP` 的固定罚参数不构成一般约束 NLP 的强收敛保证；严格 NLP 生产
  证明优先选择 `Ipopt` 等成熟外部后端。
- 原生 MINLP 对一般非凸问题不提供全局最优保证，应视为实验性局部求解能力；
  非凸 QP（`Q` 非半正定）同样没有全局最优保证。
- `NativeIPM` 和锥 IPM 返回的 `unknown` 必须被上层保留，不能改标为
  infeasible。
- 原生对偶单纯形虽有广泛单元测试，但冷启动、极端退化和大型 NETLIB 的覆盖
  仍需在每个平台独立验证。
- 商业后端（Gurobi）需要单独的编译支持和运行许可证；外部后端只有在编译
  支持、库/可执行文件和许可证都可用时才注册。
- 性能结论强依赖编译器、BLAS、稀疏后端、线程和数据集；以[测试与基准](09-testing-benchmarks.md)
  中带复现环境的数据为准。
