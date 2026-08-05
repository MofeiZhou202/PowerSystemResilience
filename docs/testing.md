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
| 日期 | 2026-08-05 |
| 平台 | Windows，PowerShell |
| 工作分支 | `release/windows-self-contained` |
| 构建目录 | `build/windows-vcpkg-release` |
| 生成器/编译器 | Ninja Multi-Config / MSVC 19.44（VS 2022 17.14） |
| 配置 | Release，HFactor/MKL/MUMPS/OpenMP/PaPILO/SuiteSparse/SuperLU 开启 |
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
| `unit` | 13 | 12 | 1 | 2.29 s |
| `integration` | 3 | 1 | 2 | 0.73 s |
| `benchmark` | 2 | 2 | 0 | 0.50 s |
| 合计 | 18 | 15 | 3 | 3.52 s |

### 3.1 失败项

1. `test_milp_solver`：54 个用例中 53 个通过。失败用例
   `production optimum matches exhaustive binary oracle` 的求解器结果和用例内部
   穷举 oracle 都为 `-10`，但最后两个断言仍硬编码为 `-6`。这是测试期望与
   当前测试数据不一致，不能据此判定求解器答案错误。
2. `test_ipopt_parameter_stability`：近相关等式用例返回
   `x=(1.001262, 0.998738)`，未满足逐变量 `2e-5` 的断言。其余 5 个实际执行
   用例通过；该项反映近秩亏等式下的解分量稳定性风险。
3. `test_netlib_regression`：3 个用例中 2 个通过；`grow22` 用例在读入数据时
   返回 `FilereaderRetcode=2`。同一次 benchmark 还报告 `afiro`、`adlittle`、
   `share2b`、`stocfor1`、`kb2` 不可读，因此这是数据集未部署，不是数值断言
   失败。

`test_scuc_module.cpp` 和 `test_market_simulation.cpp` 共 19 个 `TEST_CASE` 未进入
当前 CTest 配置，不计为通过。发布前需要在启用 SCUC 测试目标的构建中补跑。

### 3.2 Release smoke 基准

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

### 3.3 构建状态

核心库和上述 18 个测试/基准目标均在当前工作树重新编译成功。无目标过滤的
全量构建仍失败于独立基准 `native_dual_bfrt_simd_benchmark.cpp:277`：MSVC
下使用了 GCC 专有的 `__VERSION__` 宏。该问题不影响核心库和本次 CTest，仍是
发布构建需要修复的阻塞项。

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
