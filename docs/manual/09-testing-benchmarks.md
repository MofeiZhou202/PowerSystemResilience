# 第 9 章 测试与性能基准

> 本章整合自: docs/archive/testing.md, docs/archive/netlib_benchmark.md, docs/archive/windows_ci_validation.md

本章面向需要在现场验证构建正确性、复现性能数字的工程师，回答四个问题：测试怎么跑、NETLIB 基准结论是什么、Windows 平台当前验证状态如何、基准如何复现。求解算法本身的原理见 [求解器与引擎](05-solvers-engines.md) 和 [数值方法](06-numerical-methods.md)，构建选项见 [安装与部署](02-installation-deploy.md)。

2026-09-11 的离线复评见 [9.6 节](#96-windows-离线复评2026-09-11)。以下 2026-08 的机器、测试数量与线程化结果是历史快照，不能直接代替当前提交的验证。

## 9.1 测试体系

### 9.1.1 构成与统计口径

- 源码测试定义：`tests/*.cpp` 中的 Catch2 `TEST_CASE`。
- 构建测试目标：以 `ctest --test-dir <build-dir> -N` 实际列出的目标为准。
- 通过率：只统计本次真正执行并返回的 CTest 目标。
- 性能：必须使用 Release 构建、固定数据、固定线程和重复次数；普通单元测试的 wall time 不能当作求解器性能基准。

CTest 目标按三个标签分组：

| 标签 | 目标数 | 内容 |
|---|---:|---|
| `unit` | 13 | 引擎 API、LP/IPM/锥/MILP 求解器、预处理、模型校验等单元测试 |
| `integration` | 3 | 跨模块集成测试（含 `test_branch_and_cut`、`test_netlib_regression`） |
| `benchmark` | 2 | `milp_benchmark`、`native_kernel_comparison` 冒烟基准 |
| 合计 | 18 | |

### 9.1.2 运行测试

完整流程（以 Windows MSVC preset 为例，其他 preset 见 [安装与部署](02-installation-deploy.md)）：

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release -C Release --output-on-failure
```

按标签分跑（提交前快速检查可只跑 `unit`；发布审核应跑全部三个标签）：

```powershell
ctest --test-dir build/windows-msvc-release -C Release -L unit --output-on-failure -j 2
ctest --test-dir build/windows-msvc-release -C Release -L integration --output-on-failure -j 2
ctest --test-dir build/windows-msvc-release -C Release -L benchmark --output-on-failure -j 1
```

> 注意：该构建为多配置（Ninja Multi-Config），`-N -L` 列举标签时也需要带 `-C Release`，否则列出 0 个目标。

子系统与主要测试文件的对应关系：

| 子系统 | 主要测试文件 |
|---|---|
| 公共 API/线性代数 | `test_engine_api.cpp`, `test_adapter_registry.cpp` |
| 模型校验/预处理 | `test_problem_validation.cpp`, `test_presolve.cpp` |
| LP/对偶单纯形 | `test_lp_solver.cpp`, `test_dual_simplex.cpp` |
| NLP/LP/QP 内点法 | `test_ipm_solver.cpp`, `test_numerical_stability.cpp` |
| 锥规划 | `test_conic_ipm.cpp`, `test_aml_conic.cpp` |
| MILP/B&C | `test_milp_solver.cpp`, `test_branch_and_cut.cpp` |
| 回归数据集 | `test_netlib_regression.cpp` |
| SCUC | `test_scuc_module.cpp`, `test_market_simulation.cpp` |
| AML/L2O | `test_aml_model.cpp`, `test_l2o_trace.cpp` |

SCUC 相关测试只在 `-DMIPSOLVERS_BUILD_SCUC=ON` 的构建中注册，发布前需在启用 SCUC 目标的构建中补跑（见 9.3.2）。

## 9.2 Windows 平台验证结果

### 9.2.1 当前状态（2026-08-18 实测）

在 `build/windows-msvc-release`（Release，Ninja Multi-Config / MSVC）上执行全量 CTest：

```powershell
ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 2
```

结果：**18/18 全部通过**（unit 13 + integration 3 + benchmark 2），标签耗时 benchmark 0.39 s、integration 1.13 s、unit 1.37 s（processor time）。

同日 NETLIB 90 性能工作（推导记录见
`docs/archive/lp_tail_elimination_2026-08-18.md`）：修复了 Windows 构建中
CHOLMOD 因缺少 BLAS 降级为 simplicial（`NSUPERNODAL`）的问题——CMake 现在
在无系统 BLAS 时回落到 `third_party/oneapi-mkl` 的静态 MKL（H1），
supernodal 内核恢复。

随后完成线程化升级（H4，用户批准的非对称硬件利用）：MKL 包重 stage 为
`mkl_intel_thread` + `libiomp5md`（`third_party/stage_onemkl.ps1 -Threading
both`），`MIPSOLVERS_MKL_THREADING=INTEL` 重 configure，BLAS 回落块跟随
线程层，OpenMP 运行时 DLL 在 configure 时自动复制到 `tests/<config>/`。
实测要点：dfl001 分解段 6.18s→1.64s（16 线程 3.8x），总时间 10.25s→2.66s；
`MKL_CBWR=AUTO/AVX2` 会使 PARDISO 求解段劣化 4-5 倍，**不得启用**——
线程化下的复现性依赖固定线程数（迭代数逐位一致，目标值 run-to-run 相对差
~4e-13）。同日还修复了增广路径的鲁棒性缺陷（H5-H7）："分解成功但方向
溢出"不再直接中止，而是按 IP-PMM 契约升级正则化预算并重分解
（×100 升级 / ÷10 衰减滞后环），收敛候选门放宽至 100x 并由原模型审计
权威裁决。

Native IPM direct 由 89/90 success（greenbea 超时）恢复
为 **90/90 success + 90/90 accurate**（h7 轮最终口径，ctest 18/18）。
性能演进：H1 后全量总时间 31.44s → 24.48s；H4 线程化后（h7 轮、机器
凉态）Native-IPM 15.63s vs 同轮 HiGHS-ipm 8.08s；配对几何均值 vs
HiGHS-ipm 为 Native-Auto 1.354x / Native IPM 1.275x。
残余长尾（dfl001 1.5x、maros-r7 3.8x、greenbea ~20x，均对同轮
HiGHS-ipm）的根因与后续方向记录在该推导文档中；greenbea 与 maros-r7
的 1.2x 口径差距来自 HiGHS 的 presolve 级模型缩减，direct 契约下
不可达，达成前提是原生 presolve 项目（用户决策项）。

历史对照：2026-08-06 的记录为 16/18，当时两个失败项为：

1. `test_milp_solver`：求解器结果与用例内部穷举 oracle 一致（均为 `-10`），但断言硬编码为 `-6`，属测试期望与测试数据不一致。
2. `test_ipopt_parameter_stability`：近相关等式用例的解分量未满足逐变量 `2e-5` 的固定 margin。

这两项在当前 18/18 结果中已不复现；遇到类似历史报告时以最新实测为准。

### 9.2.2 Windows 特有回归面

以下三类 Windows 特有缺陷曾导致失败，修复后构成必须保持绿色的回归面：

1. 外部求解器子进程输出重定向硬编码 `/dev/null` → 修复为 Windows 下重定向到 `NUL`。
2. `test_market_simulation.cpp` 调试 JSON 导出硬编码 `/tmp` → 修复为 `std::filesystem::temp_directory_path()`。
3. 空稀疏系统（`0x0`）进入 Eigen SparseLU 导致除零 → 修复为在调用 Eigen/SuiteSparse 后端前短路。

验收标准：配置与构建不依赖 Gurobi；`test_engine_api`、`test_milp_solver`、`test_ipm_solver`、`test_scuc_module`、`test_market_simulation` 全部通过；失败输出中不得再出现 `/dev/null` 或 `/tmp` 路径假设；`test_engine_api` 确认 `EigenSparseLUSolver` 接受空 `0x0` 系统不崩溃。

### 9.2.3 定向回归切片与 CI 映射

在 `x64 Native Tools Command Prompt for VS 2022`（或已加载 MSVC 工具链的 PowerShell）中：

```powershell
cmake -S . -B build-win -A x64 `
  -DMIPSOLVERS_BUILD_TESTS=ON `
  -DMIPSOLVERS_BUILD_PYTHON=OFF `
  -DMIPSOLVERS_BUILD_SCUC=ON `
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF `
  -DMIPSOLVERS_ENABLE_WERROR=OFF

cmake --build build-win --config Release `
  --target test_engine_api test_milp_solver test_ipm_solver test_scuc_module test_market_simulation `
  --parallel 4

ctest --test-dir build-win --build-config Release `
  -R "test_(engine_api|milp_solver|ipm_solver|scuc_module|market_simulation)" `
  --output-on-failure --parallel 2
```

对应的 CI job（GitHub Actions）应在 `windows-latest` 上：启用 tests、关闭 Python、启用 SCUC，只构建上述五个测试目标，并用同一条 `ctest -R` 命令跑定向切片。仓库内置 HiGHS/SCIP 源码树足以覆盖 CI，无需外部求解器可执行文件。

## 9.3 NETLIB 90 案例基准

### 9.3.1 测试契约

2026-08-06 在 Windows Release 构建上完成的全量复验：

| 项目 | 值 |
|---|---|
| 操作系统 | Windows 11 Pro 64-bit，10.0.26200 |
| CPU / 内存 | Intel Core Ultra 9 285H，16 核 / 32 GiB |
| 编译器 | MSVC 19.44（VS 2022） |
| 稀疏后端 | SuiteSparse/CHOLMOD、MKL PARDISO |
| 基准程序 | `tests/Release/netlib_solver_benchmark.exe` |
| 数据集 | `tests/data` 中有官方参考目标的 90 个 NETLIB MPS 案例 |
| 隔离方式 | 每个案例、每次重复使用独立 benchmark 子进程 |
| 时限 | 求解器 15 s，隔离进程硬时限 20 s |
| 迭代上限 | 100,000 |

验收对象是**完全绕过 HiGHS presolve 的 Native IPM direct 路径**：模型加载后不调用 HiGHS presolve，预处理、IPM 迭代、Newton 系统选择、停止判定和解后审计均由 Native 路径完成。CHOLMOD 与 PARDISO 只是本地链接的数值线性代数内核，不执行 HiGHS 模型变换。计时只包含求解调用，不含 MPS 加载。

### 9.3.2 正确性审计

benchmark 在原始、未 presolve 的 LP 上重新计算目标和可行性，不信任求解器的 `Optimal` 字符串。同时满足以下条件才记为 `accurate`：

1. `success=true`；
2. `abs(f(x)-f_ref) / max(1, abs(f_ref)) <= 1e-5`；
3. `max(row_violation, bound_violation) / max(1, abs(rhs)) <= 1e-7`。

阈值对 Native 与 HiGHS 完全相同。Native 90 个解中最大目标相对误差 `1.250e-6`、最大归一化原始可行性违反 `8.689e-8`，均在发布阈值内。

### 9.3.3 主要结论

| 指标 | Native IPM direct | HiGHS IPM |
|---|---:|---:|
| success | **90/90** | 89/90 |
| accurate | **90/90** | 88/90 |
| 全集几何均值 | **12.5773 ms** | 15.6772 ms |
| 全集中位数 | **8.7098 ms** | 13.1550 ms |
| 全量总求解时间 | 10,129.4127 ms | **5,300.4629 ms** |
| 配对几何均值（88 个共同 accurate 案例） | **11.7404 ms** | 15.2844 ms |
| 配对加速 | **1.302x** | 1.000x |

配对加速的定义：先在 88 个双方都 accurate 的同名案例上分别计算运行时间几何均值，再取 `HiGHS / Native`——失败或错误结果不能靠快速返回获得性能优势。结论可表述为"全部通过、配对典型性能更快且结果更准"，但**不能**宣称全量总时间或尾延迟领先。

同一批 90 个原始 MPS 上对 SCIP in-process direct reader 的单进程对照：

| solver | success | accurate | total | geometric mean | median |
|---|---:|---:|---:|---:|---:|
| Native IPM direct | 90/90 | 90/90 | 9.7944 s | 12.4018 ms | 9.8578 ms |
| SCIP direct MPS | 90/90 | 89/90 | 30.1542 s | 35.7976 ms | 26.7293 ms |

该轮与隔离 HiGHS 报告计时协议不同（单进程 vs 独立子进程），两个表的绝对时间不能混合计算。

### 9.3.4 失败分析

HiGHS IPM 未通过原模型审计的两个案例（说明 `Optimal` 状态不等于解正确）：

| 案例 | success / 状态 | 目标相对误差 | 归一化原始违反 |
|---|---|---:|---:|
| `ganges` | true / `Optimal` | `1.801e-2` | `4.898e-6` |
| `greenbea` | false / `Unknown` | `5.285e-4` | `1.432e-2` |

SCIP 唯一未通过的是 `forplan`：其 MPS reader 对 BOUNDS 段连续报警并返回目标 0，而参考目标为 `-664.21896127`；状态虽为 Optimal，仍被原模型审计正确拒绝。

Native 剩余的性能长尾（决定全量总时间差距）：

| 案例 | Native ms | HiGHS IPM ms | Native 结果 |
|---|---:|---:|---|
| `dfl001` | 2,930.2415 | 1,124.3165 | accurate |
| `greenbea` | 1,621.1659 | 155.6581 | accurate；HiGHS 不 accurate |
| `maros-r7` | 1,280.6878 | 262.8354 | accurate |
| `pilot87` | 978.2399 | 815.2292 | accurate |

后续性能工作应直接压缩这些结构性长尾，而不是继续调整全局容差。

## 9.4 复现基准

### 9.4.1 准备数据与构建 benchmark

90 个 NETLIB 案例（含官方参考目标元数据）由仓库内脚本抓取：

```powershell
python tools/fetch_netlib.py
```

构建基准可执行文件（输出在 `tests/Release/` 下）：

```powershell
cmake --build build/windows-msvc-release --config Release `
  --target netlib_solver_benchmark miplib2017_benchmark --parallel 4
```

### 9.4.2 运行 NETLIB 基准

`netlib_solver_benchmark` 的常用选项：

```
--data-dir DIR       tests/data directory
--case TEXT          run matching instance names only
--cases A,B          run an exact comma-separated case list
--solvers A,B        native-ipm-direct, native-dual-direct, highs-ipm 等
--repeat N           repetitions per case/algorithm
--time-limit SEC     supported backend wall limit
--max-iterations N   iterative algorithm limit
--csv PATH           raw result CSV
--json PATH          raw and summary JSON
```

复现 Native IPM direct 全量运行：

```powershell
tests\Release\netlib_solver_benchmark.exe `
  --data-dir tests\data --solvers native-ipm-direct `
  --time-limit 15 --max-iterations 100000 `
  --csv reports\netlib_native_ipm_repro.csv `
  --json reports\netlib_native_ipm_repro.json
```

HiGHS IPM 对照只改 `--solvers highs-ipm` 和输出路径。

原文档的 90/90 报告采用"每案例独立子进程"的隔离协议，以避免单次崩溃污染整轮结果并施加进程级硬时限。原一次性审计脚本 `tools/run_netlib_isolated.py` 已从仓库移除；如需同等隔离，可用 PowerShell 逐案例调用 benchmark（每个案例一个进程），等价于原协议：

```powershell
$cases = Get-ChildItem tests\data\netlib -Filter *.mps | ForEach-Object BaseName
foreach ($c in $cases) {
  $p = Start-Process -PassThru -NoNewWindow `
    -FilePath tests\Release\netlib_solver_benchmark.exe `
    -ArgumentList "--data-dir tests\data --case $c --solvers native-ipm-direct",
                  "--time-limit 15 --max-iterations 100000",
                  "--csv reports\iso_$c.csv --json reports\iso_$c.json"
  if (-not $p.WaitForExit(20000)) { $p.Kill(); "TIMEOUT: $c" }  # 20 s 进程硬时限
}
```

> 若只是回归验证而非出报告，直接跑带 `-L integration` 的 `test_netlib_regression`（见 9.1.2）即可覆盖基准数据集的自动化正确性检查。

### 9.4.3 MIPLIB 2017 基准

`miplib2017_benchmark` 递归扫描 `.mps[.gz]` 目录并用 `.solu` 参考文件审计：

模型导入与优化分别计为 `read_ms`、`solve_ms`；后端的 `--time-limit` 在导入
完成后才应用于优化阶段，避免大 MPS 解析挤占搜索预算。

```
--data-dir DIR       recursively scan DIR for .mps[.gz]
--solu FILE          MIPLIB .solu reference file
--solvers A,B        cplex-mip,highs-mip,scip-mip,native-highs-lp 等
--case A,B           instance-name substring filters
--limit N / --sample N / --repeat N / --time-limit SEC
--native-threads N   Native B&C 请求线程数；JSON 同时记录实际线程与调度原因
```

示例：

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir <miplib2017 数据集目录> --solu <miplib2017.solu> `
  --solvers highs-mip,native-highs-lp --sample 20 --repeat 3 --time-limit 300
```

启用 CPLEX 的 Windows 构建可加入 `cplex-mip`。必须先把 Studio 的 DLL 目录
加入 `PATH`（见第 2 章），然后用相同实例、线程、seed、gap 和时限运行：

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir <miplib2017 数据集目录> --solu <miplib2017.solu> `
  --solvers cplex-mip,highs-mip,native-highs-lp `
  --seeds 0 --repeat 1 --time-limit 3 --native-threads 1 --case mas74,sct2 `
  --json reports\miplib_cplex_pilot.json --csv reports\miplib_cplex_pilot.csv
```

Native 的 `native_parallel` 结果对象同时给出请求线程数、实际有效线程数、
explorer 数、并行树是否启动及调度原因。比较 1/4 线程时必须核对这些字段；
严格 HiGHS 根流水线和等待 incumbent 的调度均可能使一次运行保持串行。

2026-09-12 的 Windows/MSVC Release 小样本中，CPLEX 在 `mas74`、`sct2`
的三秒最终 gap 分别为 10.3008% 和 0.02690%，HiGHS 为 17.4666% 和
23.4491%，Native/HiGHS-LP 为 39.6664% 和 3.1466%；六个 incumbent 均通过
原模型审计。CPLEX 模型导入分别只占 import+optimize 的 0.00548% 和
0.02509%。但六次运行均未证明最优，PAR-10 均为 30 秒；两个实例、一个 seed
不足以支持“比肩 Gurobi”或总体领先结论。Native 在 `sct2` 上还出现三秒配置却
运行 8.739 秒的 deadline 失真，因此这轮只能作为接口正确性和短时 gap pilot。
完整命令、build flags、基线 commit 与预测对照见
[CPLEX 集成记录](../archive/cplex_callable_library_integration_2026-09-12.md)。

随后对本机已有的全部 12 个样本按相同 seed/gap/三秒设置复跑 CPLEX 与
HiGHS，并修正 HiGHS time limit 曾在 `readModel` 前生效的计时口径错误。修正后
CPLEX 为 1 个 proven、8 个 feasible、0 个审计失败，HiGHS 为 0、5、0；双方
都有 incumbent 的 5 例中，CPLEX 最终 gap 较小 4 例，HiGHS 较小 1 例。
CPLEX 的 PAR-10 shifted geomean 为 23460.1 ms，HiGHS 为 30000.0 ms；CPLEX
最大 import/(import+optimize) 为 4.6122%，通过预注册的 5% 门。

该扩展仍不是严格等墙钟排名：Windows runner 没有进程级 hard deadline，记录到
CPLEX 最大 `solve_ms=3712`、HiGHS 最大 `solve_ms=10033`。它也只有一个 seed，
且本构建关闭 Gurobi，因此只能视为 CPLEX 的有利短预算信号，不能证明通用 MILP
性能已与 Gurobi 比肩。机器可读结果为 `reports/miplib_cplex_12case.json` 和
`reports/miplib_cplex_12case.csv`（`reports/` 默认被 Git 忽略）。

### 9.4.4 Release 冒烟基准

`native_kernel_comparison --smoke --check --time-limit 60` 用于快速确认各求解路径目标一致、gap 为 0。其单次样本太小、未重复，只能作正确性烟雾测试，不能作为性能排名依据。

## 9.5 结果判读注意事项

- 性能结论只来自 Release 构建 + 固定数据 + 固定线程 + 重复次数；单元测试 wall time 不能当求解器性能。
- `success` 只反映求解器状态字符串；正确性以 9.3.2 的原模型审计为准。
- 配对几何均值、全集几何均值、全量总时间回答三个不同问题，不能互相替代（见 9.3.3）。
- 隔离子进程运行与单进程运行的绝对时间不可混算。
- 测试失败排查见 [故障排查](10-troubleshooting.md)。


## Windows/main integration validation

The integration combines Windows parent `75c6e192` with main `5eac6be0`.
The model remains the original NLP KKT system. For main's augmented congruence
`T = diag(I, I, sqrt(mu/s))`, solve `T K T z = T rhs` and recover `dx,dy = T z`.
The Windows common KKT solve retains its backward-error certificate; only the
coordinate transformation is added. References: Waechter--Biegler (2006),
Sections 2.2 and 3.1; chapter 6, scaling, inertia and iterative refinement.

Conflict policy: retain Windows exact-first regularization, representable
positive floors, caller-coordinate feasibility and strict termination gates,
and guarded boolean KLU refactorization. Combine Ipopt acceptable-iteration
control with main's adaptive barrier and complete primal-dual warm-start API.
Main's optional active-set polish must also pass the Windows strict gates.
Retain Windows Newton-direction/corrector policy instead of reinstating main's
removed extra centrality-correction loop. First-attempt/restoration budgets
and diagnostics are imported from main. This is not bit-identical to either
parent and historical timings do not certify the merge.

Prediction fixed before validation: zero changes to requested accuracy gates;
all existing admitted regression assertions must pass. An explicit primary
budget of N allows at most N initial iterations. Congruence adds O(m_ineq)
scaling work and storage; no wall-time speedup is predicted on a shared machine.
Assumptions: positive finite interior iterates and matching matrix/vector
index spaces. Validate engine API, IPM, numerical stability, linear algebra,
LP/MILP and AML suites, then Simulation resilience, market pricing and OPF
integration. Preserve failures and investigate the implementation before
relaxing any mathematical gate. Build and measured results follow below.

The first configure required the existing staged static ZLIB prefix. The first
compile exposed duplicate base/KLU refactor declarations and a duplicate KLU
definition produced by Git's automatic merge. Removed only the duplicate
main entries, retaining the Windows pattern validation and transactional
failure path. This is an integration error, not a change to the numerical
model or a reason to relax its acceptance gates.

The next build exposed a truncated warm-start test caused by conflict
resolution. Restored main's complete near-feasible and rejected-start tests
alongside the Windows audit case; no assertions or tolerances were removed.
The first SDK consumer configure exposed an unresolved `MIPSolvers::MKL`
link target from exported `ipopt_local`. The package config now recreates that
target from its already validated bundled static MKL archives. This preserves
the existing sequential/LP64 link model and predicts zero external MKL paths
required by a consumer; the downstream Release link verifies this contract.

Initial IPM regression: 50/54 cases passed, with 10 assertions in four imported
cases failing. Inspection against both parents showed test-assumption drift:
Windows `mu_init=0` means automatic initialization, so constructing supplied
dual vectors from that default creates inadmissible zero multipliers. Its
central-start tolerances inherit strict caller tolerances, whereas these main
fixtures intentionally start with 0.005 bound violation and 0.01 slack residual.
The fixtures now explicitly supply main's intended `mu_init=0.1` and audit
tolerances (0.01 primal, 0.5 relative centrality); production defaults and final
KKT gates are unchanged. The one-variable quadratic diagnostic also enters a
Windows full-derivative certification path once; its oracle now expects that
one evaluation instead of main's zero. All other assertions are retained.

Final standalone CTest coverage: all 21 registrations passed (20 in one run,
IPM separately after the fixture correction: 54 cases, 368 assertions).
The SDK consumer then exposed missing SuiteSparse public headers despite
exported UMFPACK/KLU libraries. The full SDK install now includes their public
component headers and exports a package-relative include directory. This is a
packaging closure fix; no factorization implementation is changed.

Reproduce the standalone configuration with Visual Studio 17 2022, x64,
`MIPSOLVERS_BUILD_TESTS=ON`, `MIPSOLVERS_BUILD_SCUC=ON`,
`MIPSOLVERS_BUILD_LOCAL_IPOPT=ON`, `MIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl`,
`MIPSOLVERS_MKL_ROOT=<local static MKL prefix>`, `MIPSOLVERS_USE_MKL=ON`,
`MIPSOLVERS_USE_GUROBI=OFF`, `MIPSOLVERS_USE_PAPILO=OFF`,
`MIPSOLVERS_ENABLE_IPO=OFF`, `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF`,
`MIPSOLVERS_BUILD_BUNDLED_ARCHIVE=OFF`, `ZLIB_ROOT=<local static ZLIB prefix>`
and `ZLIB_USE_STATIC_LIBS=ON`; build with `--config Release --parallel 3`.
The optional GNU MRI bundled-archive target is disabled for this MSVC SDK
validation; individual exported static libraries are the validated artifact.
The successful ALL_BUILD follows a transient executable lock when a test was
still running during relink; serializing the final build cleared that lock.

[Machine-readable evidence](windows-integration-evidence.json) records all
21 commands/results and their complete test summaries. The 20-test run took
296.26 s wall; final IPM took 11.75 s (12.23 s CTest wall). These ran alongside
compilation and cannot rank solver speed. Gurobi and PaPILO were disabled at
configuration and are outside this coverage. The accuracy prediction holds
for the admitted assertions; no cross-platform or wall-time speedup was tested.

### Downstream result and publication boundary

Simulation `b0b852a9` built its full core and four test executables with this
installed SDK, full dependency profile, ETAP and Ipopt enabled, MSVC Release
without native-architecture/IPO optimizations. Source compilation used `/MP3`;
the final numerical tests ran sequentially after compilation. Commands:
`cmake --build <simulation-build> --config Release --parallel 3 --target
test_resilience_assessment test_southern_market test_opf_solver_backends
test_three_phase_hybrid_opf`, then each executable with
`--reporter junit --out <report.xml>`.

| Suite | Assertions | Failed assertions | Skipped | JUnit seconds |
|---|---:|---:|---:|---:|
| resilience assessment | 584 | 0 | 0 | 1.649 |
| Southern market | 29404 | 0 | 6 | 74.859 |
| OPF solver backends | 721 | 6 (one case) | 0 | 40.913 |
| three-phase hybrid OPF | 164 | 1 (one case) | 0 | 0.287 |

Two downstream cases therefore violate the initial all-regressions-pass
prediction. Both pass when rerun against the preserved pure-main `5eac6be0`
build (6/6 and 28/28 assertions). The merged build reports:

- GUI Auto showcase `ieee24_3area_acdc_expanded`: Ipopt iteration limit,
  primal `3.41241842849e-6`, stationarity `0.00352618561813499`.
- GUI Auto showcase `multiscale_comprehensive_acdc`: Ipopt iteration limit,
  primal `1.36737322097e-6`, stationarity `0.01203573543262182`.
- Native graph-reduced primal-dual transport: accepted-step collapse,
  primal `8.008e-7`, dual `0.289586`, complementarity `0.1`.

The conflicting Windows strict-bound/termination policies and Native
globalization choices differ from main; this paired comparison establishes
regression but does not isolate a single causal change. No tolerance was
relaxed and no failed result relabeled successful. Publish as a source-branch
integration with these explicit limitations. Simulation remains pinned to
pure main `5eac6be0`; this merge is not a validated drop-in replacement.
Neither a new installer nor a repeated all-module performance matrix is
certified by this publication. Raw logs are under the Simulation workspace's
`build/windows-branch-publication/`; retained downstream values and failure
messages are included in the linked machine-readable evidence.

## 9.6 Windows 离线复评（2026-09-11）

以下是修复前提交 `75c6e1922839d4775c27eacda833df386e95f476` 的基线快照，Windows 11 / i9-12900H / MSVC 19.44 Release，本地 sequential oneMKL，未联网。完整协议、命令、构建选项、失败复核及优化建议见 [离线评估记录](../archive/windows_offline_evaluation_2026-09-11.md)；修复与线程实验须使用相应工作区补丁，不能仅凭相同 HEAD 复现。

- CTest：19/19 通过，14 unit + 3 integration + 2 benchmark；SCUC/Python 未启用。
- NETLIB 90 × 3：原生 IPM direct 和 HiGHS simplex 均 270/270 准确；关闭 crossover 的 HiGHS IPM 为 264/270，问题为 ganges / greenbea。
- Native-Auto 单独批次为 264/270 准确：greenbea 超时，tuff 返回不准确的 Optimal；tuff 在原生双单纯形加 HiGHS presolve 路径也复现。不能仅凭 CTest 通过或 success 字段发布自动求解结果。
- HS071：本地 Ipopt 5/5 准确，禁用 fallback 的原生 NLP 0/5；SOCP/SDP quick 为 24/24 optimal；MILP 小型根节点冒烟为 21/21 成功，不代表大规模分支树性能。
- 该基线 CMake 对 Windows MKL 构建无条件要求 SEQUENTIAL；INTEL 线程层命令会被 SDK 检查拒绝。完整功能 SDK 的外部消费者还因缺失 `MIPSolvers::MKL` 导出目标生成失败；bundled 目标存在 MSVC 不支持 GNU `ar -M` 的产物缺失问题。

原生 IPM 三个长尾占 90 例中位数耗时总和约 63.08%。若未来能将它们各降时一半，条件预测全集降时约 31.54%；这不是本次已实施的加速结果。原始数据保存在 `reports/windows_eval_20260911/`。

## 9.7 LP 分解计时与 INTEL 线程稳定性协议

理论依据与预先固定的验收规则见 [Windows 修复推导 R6](../archive/windows_remediation_2026-09-11.md)。在同一 Release 构建上执行：

```powershell
python tools/windows_lp_stability.py --stage overhead --output reports/windows_stability_20260911
python tools/windows_lp_stability.py --stage stability --output reports/windows_stability_20260911
python tools/windows_lp_stability.py --stage gates --output reports/windows_stability_20260911
```

各阶段按上述顺序执行，不同时运行其他求解器或构建。脚本设置 `OMP_NUM_THREADS=1`、`MKL_DYNAMIC=FALSE`，分别验证 `MKL_NUM_THREADS=2/4`，并从基准 JSON 的 `mkl_max_threads` 核对运行时设置。完整日志、原模型准确性、进程峰值工作集与二进制 SHA256 均保存在输出目录。复测应使用新的输出目录，以免覆盖原始证据。

`stability` 对 dfl001、greenbea、maros-r7 做 20 组独立进程测量，奇数组 2/4、偶数组 4/2；预热进程不计入样本。报告中位数、nearest-rank P95（20 个样本的第 19 个）、最大值及 IQR/中位数。三案例总时间先在每个区组内相加，再计算分布；它不等于三个案例的中位数之和。P95 是本工作站的样本估计，不是跨机器的尾延迟保证；每个新进程仍包含首次库调用。

LP 计时默认关闭，手动诊断时设置 `MIPSOLVERS_LP_FACTOR_TIMING=1`。每个 `solve_lp_impl` 变体在标准错误输出一行 `LP-FACTOR` JSON，包括失败后重跑的变体。字段含义：

| 字段 | 含义 |
|---|---|
| `assembly_ms` | 已覆盖的矩阵装配、图结构与散射索引准备，扣除嵌套分析和分解 |
| `symbolic_ms` | 后端符号分析 API，含分解内部触发的延迟分析 |
| `numeric_ms` | 数值分解 API，含该 API 内部的格式准备，扣除符号分析 |
| `other_ms` | 变体总耗时减前三项，含回代、残差、全局化及未单列准备 |
| `variant_ms` | 单次变体墙钟时间，等于前四项之和 |
| `retry_ms` / `retries` | LP 正则化重分解的时间与次数；时间与上述分类重叠，不能再次加总；不包含外层变体重跑和后端自适应尝试 |
| `*_calls` | 相应已插桩 API 区间的调用次数，不等同于 MKL 内部调用次数 |

仅统计调用线程的区间墙钟，不将 MKL 工作线程的 CPU 时间相加。该诊断不覆盖 macOS Accelerate 内部的分解细分；本次验收范围为 Windows 后端。旧 `setup_sub_normal_only` 输出只覆盖普通方程部分路径，不能据此判断 augmented 路径未发生分解。

### 9.7.1 本轮实测与运行建议

本节是修复分支 `9652ddb9` 的历史测量。随后合入Windows release `1d31f0eb` 的复测见 [合并验收报告](../archive/windows_release_merge_2026-09-11.md)，两个版本的样本不可混算。

完整数字、源码/二进制身份、预测偏差调查见 [INTEL/2、INTEL/4 稳定性报告](../archive/windows_lp_stability_2026-09-11.md)。两档每例20次，均60/60准确：

| 案例 | 2线程中位数/P95 ms | 4线程中位数/P95 ms |
|---|---:|---:|
| dfl001 | 3340.85 / 3386.57 | 2230.12 / 2263.49 |
| greenbea | 2695.12 / 4482.84 | 2675.41 / 4151.26 |
| maros-r7 | 1091.88 / 1131.19 | 933.15 / 974.09 |
| 区组三例合计 | 7182.50 / 8874.88 | 5853.92 / 7301.95 |

4线程区组合计中位数降18.50%，P95降17.72%，通过预定性能门；本机LP direct推荐INTEL/4。4对2的原预测为5–15%，超出预测的调查已写回R6。greenbea迭代轨迹仍不稳定，不能声称消除了该长尾。

关闭计时后，两档NETLIB IPM/Auto各90×3均准确，合计1080/1080；HS071原生/Ipopt各档5次，合计20/20准确。全集IPM总时间降4.53%，Auto反增10.44%，因此Auto/混合负载先保留2线程；本次没有证明4线程对所有模块普遍有益。两档计时开关三对观测差异中位数+0.609%/+1.338%，但同配置也出现轨迹差异，尚不能从中隔离纯计时开销。

最终全目标重链接后，在INTEL/4、OMP=1、计时关闭下串行运行完整CTest，**20/20通过**（14 unit、4 integration、2 benchmark），305.41秒。SCUC/Python未启用。源码与测试保留于本地工作区，完整证据目录见报告。

## 9.8 Windows release 合并验收

合并提交 `1d31f0eb` 将 `9652ddb9` 的修复合入Windows release父提交 `2c400fcf`，保留既有API与数值更新。macOS main不随本次修改。完整记录见 [合并验收报告](../archive/windows_release_merge_2026-09-11.md) 和 [机器可读证据](windows-release-merge-evidence.json)。

Windows完整构建及 **CTest 20/20** 通过（240.85秒）；NETLIB两档全集 **1080/1080**、HS071 **20/20**、长尾重复 **120/120** 准确，重定位SDK消费者目标值 **9.000000**。20组长尾中，4比2线程区组合计中位数降24.60%、P95降22.41%，LP direct推荐4线程，Auto尚无对应20组稳定性验收，保守2线程建议不变。

本批绝对耗时高于历史批次且波动明显，不能把线程间24.60%解释为合并相对旧版本的加速；原因尚未唯一定位。release父提交已知的Simulation下游失败没有在本轮重新验证，仍保留相应限制。SCUC/Python未启用，HS071的CTest注册项为1线程，其2/4线程准确性由独立NLP阶段覆盖。
