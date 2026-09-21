# 测试与基准结果

本文记录当前工作树的可复现验证，不收录历史阶段报告中的数字。

## 1. 统计口径

- 源码测试定义：`tests/*.cpp` 中的 Catch2 `TEST_CASE`。
- 构建测试目标：`ctest --test-dir <build-dir> -N` 实际列出的目标。
- 通过率：只统计本次真正执行并返回的 CTest 目标。
- 性能：必须使用 Release 构建、固定数据、固定线程和重复次数；普通单元测试
  的 wall time 不能当作求解器性能基准。

## 2. 本次环境

| 项目 | 值 |
|---|---|
| 日期 | 2026-08-06 |
| 平台 | Windows，PowerShell |
| 工作分支 | `release/windows-self-contained` |
| 测试提交 | `5a43072477be` |
| 构建目录 | `build/windows-vcpkg-release` |
| 生成器/编译器 | Ninja Multi-Config / MSVC 19.44（VS 2022 17.14） |
| 配置 | Release，HiGHS/SCIP/Ipopt-PardisoMKL/OpenMP/PaPILO/SuiteSparse 开启 |
| 不可用数值后端 | MUMPS、SuperLU |
| Gurobi | 关闭 |
| 源码 `TEST_CASE` 数 | 328 |
| 当前 CTest 覆盖的 `TEST_CASE` 数 | 309 |
| CTest 目标数 | 18 |

## 3. 本次结果

执行命令：

```powershell
ctest --test-dir build/windows-vcpkg-release -C Release -L unit --output-on-failure -j 2
ctest --test-dir build/windows-vcpkg-release -C Release -L integration --output-on-failure -j 2
ctest --test-dir build/windows-vcpkg-release -C Release -L benchmark --output-on-failure -j 1
```

| 标签 | 目标数 | 通过 | 失败 | 实际耗时 |
|---|---:|---:|---:|---:|
| `unit` | 13 | 12 | 1 | 4.98 s |
| `integration` | 3 | 2 | 1 | 1.55 s |
| `benchmark` | 2 | 2 | 0 | 0.85 s |
| 合计 | 18 | 16 | 2 | 4.81 s wall time |

标签行是 CTest 报告的累计 processor time；合计是 `-j 2` 并行执行的真实墙钟
时间，因此不等于三行相加。

### 3.1 失败项

1. `test_milp_solver`：54 个用例中 53 个通过。失败用例
   `production optimum matches exhaustive binary oracle` 的求解器结果和用例内部
   穷举 oracle 都为 `-10`，但最后两个断言仍硬编码为 `-6`。这是测试期望与
   当前测试数据不一致，不能据此判定求解器答案错误。
2. `test_ipopt_parameter_stability`：近相关等式用例返回
   `x=(1.001262, 0.998738)`，未满足逐变量 `2e-5` 的断言。其余 5 个实际执行
   用例通过；该项反映近秩亏等式下的解分量稳定性风险。

`test_netlib_regression` 在部署标准数据后通过，耗时 0.71 s，不再属于失败项。

`test_scuc_module.cpp` 和 `test_market_simulation.cpp` 共 19 个 `TEST_CASE` 未进入
当前 CTest 配置，不计为通过。发布前需要在启用 SCUC 测试目标的构建中补跑。

### 3.2 NETLIB 全面基准

已使用 90 个带官方数值参考目标的标准 NETLIB LP，对 14 个算法配置完成
1,260 次全量求解，并对 5 个重点算法完成 450 次分层重复求解。统一门槛为
相对目标误差 `<=1e-5`、归一化原始可行性违反 `<=1e-7`。

核心结果为 HiGHS simplex 87/90 accurate、HiGHS IPM 86/90、SCIP direct
85/90、Native dual/Structural DSE/Exact DSE 各 84/90。详细的规模分层、
失败案例、DSE 遥测、原始结果文件和复现命令见
[NETLIB 线性规划求解器全面基准](netlib_benchmark.md)。

Native IPM 的 P1 理论增强另做了同一二进制 A/B：动态互补缓冲步长和标准
Mehrotra 中心参数为 78/90 accurate，legacy 固定步长为 76/90；无准确案例
回退，76 个共同准确案例上配对几何平均快 1.199 倍。该改动的方程、源码锚点
和 HSD/IP-PMM 后续门槛见
[Native LP 内点法理论设计](native_ipm_design.md)。

P2a 进一步加入原模型 primal/dual/gap 审计和相对候选停止，最终为 83/90
success、81/90 accurate；相对 P1 新增 `d6cube`、`degen3`、`shell`，无 accurate
回退。`shell` 从 2,000 次失败降至 108 次、约 41.6 ms，并通过原模型 KKT。

上述 P1/P2a 数字是阶段基线。当前严格正确性验收使用库默认
`IPMLPOptions::max_iter=200`，而不是把迭代上限提高到 1,000：

```powershell
python tools/run_netlib_isolated.py `
  --executable tests/Release/netlib_solver_benchmark.exe `
  --data-dir tests/data --solver native-ipm `
  --solver-time-limit 40 --hard-timeout 50 --max-iterations 200 `
  --csv reports/netlib_native_ipm_residual_regularization_full90_2026-08-06.csv `
  --json reports/netlib_native_ipm_residual_regularization_full90_2026-08-06.json `
  --log reports/netlib_native_ipm_residual_regularization_full90_2026-08-06.log
```

结果为 90/90 success、90/90 accurate，无超时、崩溃或隔离子进程异常；总求解
时间 47.497 s，中位数 21.717 ms。准确门槛保持相对目标误差 `<=1e-5` 和归一化
原始可行性违反 `<=1e-7`。`test_engine_api` 为 31/31 用例、138 个断言通过；
`test_numerical_stability` 为 23/23 用例、446 个断言通过。

合入后的定向测试为：`test_numerical_stability` 22/22 用例、442 个断言通过；
`test_netlib_regression` 3/3 用例、224 个断言通过。重新执行全部 18 个 CTest
仍为 16 通过、2 失败，失败项及数值与 3.1 节相同，没有出现新的跨模块回归。

### 3.3 Release smoke 基准

`native_kernel_comparison --smoke --check --time-limit 60` 的单次结果如下。所有
路径目标均为 `993830.1496`、gap 为 0；这些数字只用于正确性烟雾测试，样本太小
且未重复，不能作为稳定的性能排名。

| 路径 | 时间 | 迭代/节点 | 最大行违反 |
|---|---:|---:|---:|
| Native B&C + 原生对偶单纯形 | 15.0 ms | 1 LP，根闭合 | `5.68e-14` |
| Native B&C + 原生 IPM 根 | 57.5 ms | 1 LP，根闭合 | `5.68e-14` |
| Native B&C + HiGHS 合约 | 10.5 ms | 1 节点 | `0` |
| HiGHS direct | 8.8 ms | 1 节点，37 LP 迭代 | `0` |

同一 6-bus LP relaxation 上：原生对偶单纯形为 1.2 ms/64 次迭代，原生 IPM
为 2.3 ms/30 次迭代，HiGHS-LP 为 8.9 ms/37 次迭代；三者目标一致，报告最大
行违反分别为 `5.68e-14`、`1.96e-12`、`0`。

### 3.4 构建状态

核心库和上述 18 个测试/基准目标均在当前工作树重新编译成功。此前独立基准
`native_dual_bfrt_simd_benchmark.cpp:277` 在 MSVC 下使用 GCC 专有
`__VERSION__` 宏导致的构建失败已修复（MSVC 改用 `_MSC_FULL_VER` 报告编译器
版本，与 `miplib2017_benchmark.cpp`、`netlib_solver_benchmark.cpp` 中既有
的守护方式一致），当前无目标过滤的全量构建可通过。

### 3.5 CPLEX cross-platform integration (2026-09-13)

The CPLEX-only capability from Windows commit `78939619` was ported onto main
baseline `aef3be07` without its native MILP scheduling changes. The macOS
Release build used AppleClang 21.0.0, arm64, CPLEX Studio 22.1.1, one CPLEX
thread, Gurobi off and IPO off.

Focused CPLEX coverage passed 3 cases and 15 assertions. Full
`test_engine_api` passed 30/30 cases and 175 assertions; full
`test_milp_solver` passed 55/55 cases and 2,049 assertions. `otool -L` confirms
that the arm64 executable resolves `@rpath/libcplex2211.dylib`. Reconfiguring
the same source with `MIPSOLVERS_USE_CPLEX=OFF` rebuilt successfully, and its
negative registration test passed 1/1 assertion before the final ON rebuild.
The explicit `static_pic/libcplex.a` fallback also linked and passed all 15
focused assertions with the required `CoreFoundation` and `IOKit` frameworks.

The fixed three-second `mas74`/`sct2` CPLEX pilot returned incumbents for both,
and both passed the common original-model audit. Final gaps were 7.41084% and
0.0224079%; maximum row violations were `0` and `4.17444e-14`. Adapter import
was 0.100334 ms and 0.198709 ms, respectively 0.00334% and 0.00662% of import
plus optimize time. Both are below the pre-registered 5% threshold. Results
are in ignored local artifacts `reports/miplib_cplex_macos_pilot.{json,csv}`;
the complete derivation and Windows comparison are in the
[CPLEX Callable Library integration record](cplex_callable_library.md).

## 4. 推荐验证命令

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset windows-vcpkg-release
ctest --preset windows-vcpkg-release --output-on-failure
```

快速提交前检查可只运行 unit 标签；发布审核还应运行 integration、benchmark、
NETLIB/MIPLIB 数据集和 SCUC 场景。具体依赖与离线构建见
[构建与部署](build_and_deploy.md)。

## 5. 覆盖映射

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
