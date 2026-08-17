# 第 9 章 测试与性能基准

> 本章整合自: docs/archive/testing.md, docs/archive/netlib_benchmark.md, docs/archive/windows_ci_validation.md

本章面向需要在现场验证构建正确性、复现性能数字的工程师，回答四个问题：测试怎么跑、NETLIB 基准结论是什么、Windows 平台当前验证状态如何、基准如何复现。求解算法本身的原理见 [求解器与引擎](05-solvers-engines.md) 和 [数值方法](06-numerical-methods.md)，构建选项见 [安装与部署](02-installation-deploy.md)。

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

```
--data-dir DIR       recursively scan DIR for .mps[.gz]
--solu FILE          MIPLIB .solu reference file
--solvers A,B        highs-mip,scip-mip,native-highs-lp,native-native-lp 等
--case A,B           instance-name substring filters
--limit N / --sample N / --repeat N / --time-limit SEC
```

示例：

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir <miplib2017 数据集目录> --solu <miplib2017.solu> `
  --solvers highs-mip,native-highs-lp --sample 20 --repeat 3 --time-limit 300
```

### 9.4.4 Release 冒烟基准

`native_kernel_comparison --smoke --check --time-limit 60` 用于快速确认各求解路径目标一致、gap 为 0。其单次样本太小、未重复，只能作正确性烟雾测试，不能作为性能排名依据。

## 9.5 结果判读注意事项

- 性能结论只来自 Release 构建 + 固定数据 + 固定线程 + 重复次数；单元测试 wall time 不能当求解器性能。
- `success` 只反映求解器状态字符串；正确性以 9.3.2 的原模型审计为准。
- 配对几何均值、全集几何均值、全量总时间回答三个不同问题，不能互相替代（见 9.3.3）。
- 隔离子进程运行与单进程运行的绝对时间不可混算。
- 测试失败排查见 [故障排查](10-troubleshooting.md)。
