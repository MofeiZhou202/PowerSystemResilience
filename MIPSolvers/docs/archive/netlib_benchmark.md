# NETLIB 线性规划求解器全量基准

本文记录 2026-08-06 在 Windows Release 构建上完成的 90 例 NETLIB 全量
复验，验收对象是完全绕过 HiGHS presolve 的 Native IPM direct 路径，并以
同一基准程序中的 HiGHS IPM 作为外部对照。本文的最终结论只使用最终重建
二进制生成的两份隔离运行报告，不混用早期调试轮次。

## 1. 最终结论

- Native IPM direct：**90/90 success，90/90 accurate**。
- HiGHS IPM：89/90 success，88/90 accurate；`ganges` 和 `greenbea` 未通过
  同一原模型审计。
- 88 个双方都 accurate 的严格配对案例上，Native 几何均值为
  **11.7404 ms**，HiGHS IPM 为 **15.2844 ms**，Native 快 **1.302x**。
- Native 全集几何均值为 12.5773 ms，低于 HiGHS IPM 的 15.6772 ms；中位数
  也由 13.1550 ms 降至 8.7098 ms。
- Native 全量总耗时为 10.1294 s，较紧邻正式报告的 11.0762 s 降低 8.5%，
  但仍高于 HiGHS IPM 的 5.3005 s。该差距主要来自 `dfl001`、`greenbea`、
  `maros-r7` 和 `pilot87` 的长尾。因此当前已经实现“全部通过、
  配对典型性能更快且结果更准”，但尚不能宣称全量总时间或尾延迟全面领先。

这次提升依据 Newton 系统的后向误差、稀疏符号分解成本、填充和浮点停滞
判据实现；没有案例名路由、显式矩阵维度路由、审计阈值放宽或 HiGHS presolve。

### 1.1 同一原始 MPS 对 SCIP direct

补充报告 `reports/netlib_native_ipm_vs_scip_full90_2026-08-06.json` 使用同一
90 个原始 MPS，对比 `native-ipm-direct` 与 SCIP in-process direct reader：

| solver | success | accurate | total | geometric mean | median |
|---|---:|---:|---:|---:|---:|
| Native IPM direct | 90/90 | 90/90 | 9.7944 s | 12.4018 ms | 9.8578 ms |
| SCIP direct MPS | 90/90 | 89/90 | 30.1542 s | 35.7976 ms | 26.7293 ms |

Native 总时间快 `3.079x`，几何均值快 `2.887x`。SCIP 唯一未通过审计的是
`forplan`：SCIP MPS reader 对其 BOUNDS 段连续报警并返回目标 0，而参考目标为
`-664.21896127`；状态仍为 Optimal，因此被原模型审计正确拒绝。该轮是单进程
逐案例测量，与上文隔离 HiGHS 报告的计时协议不同，不能把两个表的绝对时间
混合计算。

## 2. 测试契约

| 项目 | 值 |
|---|---|
| 日期 | 2026-08-06 |
| 操作系统 | Windows 11 Pro 64-bit，10.0.26200 |
| CPU / 内存 | Intel Core Ultra 9 285H，16 核 / 32 GiB |
| 构建 | `build/windows-vcpkg-release`，Release，Ninja Multi-Config |
| 编译器 | MSVC 19.44（VS 2022） |
| 稀疏后端 | SuiteSparse/CHOLMOD、MKL PARDISO |
| 基准程序 | `tests/Release/netlib_solver_benchmark.exe` |
| 数据集 | `tests/data` 中有参考目标的 90 个 NETLIB MPS 案例 |
| 隔离方式 | 每个案例、每次重复使用独立 benchmark 子进程 |
| 时限 | 求解器 15 s，隔离进程硬时限 20 s |
| 迭代上限 | 100,000 |

“全 Native direct”严格表示：模型加载后不调用 HiGHS presolve；预处理、IPM
迭代、Newton 系统选择、停止判定和解后审计均由项目 Native 路径完成。
CHOLMOD 和 PARDISO 是本地编译链接的数值线性代数内核，不是 LP 求解器，
也不执行 HiGHS 模型变换。

两种求解器使用同一 benchmark 和同一测量契约。计时只包含求解调用，不包含
MPS 加载；报告中的 `success` 来自求解器状态，而 `accurate` 必须额外通过下述
原始 LP 审计。

## 3. 正确性审计

benchmark 在原始、未 presolve 的 LP 上重新计算目标和可行性，不直接信任
求解器的 `Optimal` 字符串。只有同时满足以下条件才记为 `accurate`：

1. `success=true`；
2. `abs(f(x)-f_ref) / max(1, abs(f_ref)) <= 1e-5`；
3. `max(row_violation, bound_violation) / max(1, abs(rhs)) <= 1e-7`。

Native 90 个解中最大目标相对误差为 `1.250e-6`，最大归一化原始可行性违反为
`8.689e-8`，均在发布阈值内。阈值与 HiGHS 完全相同，没有为 Native 单独放宽。

HiGHS IPM 的两个未通过案例为：

| 案例 | success / 状态 | 目标相对误差 | 归一化原始违反 |
|---|---|---:|---:|
| `ganges` | true / `Optimal` | `1.801e-2` | `4.898e-6` |
| `greenbea` | false / `Unknown` | `5.285e-4` | `1.432e-2` |

这说明 Native 的 90/90 不只是状态通过，而是对原模型重新审计后全部通过；
同时也解释了为什么 HiGHS 的 `ganges` 即使报告 `Optimal` 仍不能计为 accurate。

## 4. 最终性能结果

| 指标 | Native IPM direct | HiGHS IPM |
|---|---:|---:|
| success | **90/90** | 89/90 |
| accurate | **90/90** | 88/90 |
| 全集几何均值 | **12.5773 ms** | 15.6772 ms |
| 全集中位数 | **8.7098 ms** | 13.1550 ms |
| 全量总求解时间 | 10,129.4127 ms | **5,300.4629 ms** |
| 双方共同 accurate | 88 | 88 |
| 配对几何均值 | **11.7404 ms** | 15.2844 ms |
| 配对加速 | **1.302x** | 1.000x |

配对加速定义为：先在 88 个双方都 accurate 的同名案例上分别计算运行时间
几何均值，再计算 `HiGHS / Native`。这样失败或错误结果不能通过快速返回获得
性能优势。全量几何均值覆盖全部 90 次尝试，用于观察总体典型时间；总时间则
保留长尾成本。三项指标分别回答不同问题，不能互相替代。

Native 最慢的几个案例说明了当前剩余性能工作：

| 案例 | Native ms | HiGHS IPM ms | Native 结果 |
|---|---:|---:|---|
| `dfl001` | 2,930.2415 | 1,124.3165 | accurate |
| `greenbea` | 1,621.1659 | 155.6581 | accurate；HiGHS 不 accurate |
| `maros-r7` | 1,280.6878 | 262.8354 | accurate |
| `pilot87` | 978.2399 | 815.2292 | accurate |

下一阶段的性能目标应直接压缩这些结构性长尾，而不是继续调整全局容差；否则
即使几何均值继续改善，也不能实现全量总时间领先。

## 5. 理论驱动的实现

### 5.1 同坐标的 inexact Newton 判据

精化残差和 Newton 右端项现在都在相同的 Ruiz 缩放坐标中比较，目标为
`||r_linear||inf <= 0.1 ||rhs_Newton||inf`。此前把不同归一化坐标中的非线性
残差与原始线性残差直接比较，既不满足量纲一致性，也会造成无依据的过求解。

`0.1` 是保证局部收缩的 inexact-Newton forcing 常数。有限但暂未达到充分
局部界的方向交给中心性和正性步长做全局化，并由实际非线性进展及最终原模型
审计裁决；只有非有限方向立即拒绝。

### 5.2 Normal 与 augmented KKT 的结构路由

Auto 默认从 normal equations 开始，只有符号分析给出结构证据时才主动选择
augmented KKT。判据只使用 CHOLMOD 的 symbolic flop、`nnz(L)`、原矩阵非零
结构和由此得到的 factor intensity，不使用案例名称或显式行列数阈值。

- 稳定性路由：normal 填充离开 CHOLMOD 的 `5x` 稀疏经济区，同时 augmented
  的 flop 和 `nnz(L)` 都保持在 normal 的 `2x` 以内时，选择带主元 PARDISO。
- 性能路由：normal 模式完全稠密，或 factor intensity 超过
  `200 flop/nnz(L)`，且 augmented 的 flop 与 `nnz(L)` 都严格更低时，选择
  保持拟定结构的 augmented LDLT。

其中 `200 = 40 x 5`：`40 flop/nnz(L)` 来自 CHOLMOD supernodal crossover，
`5` 来自稀疏填充经济边界。它们表达的是内核计算强度与存储增长关系，而不是
针对 NETLIB 个例调出的规模参数。完全稠密 normal 的识别恢复了 `fit2p`，高
计算强度且 augmented 严格更便宜的路径改善了 `seba`。

### 5.3 后向误差优先于条件数猜测

`rcond(N)` 仍用于诊断，但不再作为方向的硬拒绝条件。normal matrix
`N = A Theta A^T` 会平方缩放算子的条件数；`rcond` 描述最坏前向误差上界，
不能替代对当前 Newton 方程实际后向误差的测量。实际接受依据是精化后的 KKT
残差以及原模型审计。

### 5.4 按失败类型恢复

normal 轨迹连续两次只能接受 `sqrt(epsilon)` 量级步长时，判定为浮点停滞并
重启 augmented KKT。恢复状态按失败发生位置划分：步长停滞和数值分解失败都
发生在坏 Newton 方向应用之前，因此可保留有限 primal 点；非有限方向或后向
误差失败可能已经失去可信方向，必须从调用者初始点冷重启。后端仍由失败类型
决定，且区分以下状态：

- `Normal equations stalled`：中心路径步长被舍入误差限制；
- `Normal equations rejected`：方向后向误差或有限性失败；
- `Cholesky failed`：数值分解失败。

### 5.5 原模型提前审计

当缩放坐标中的廉价判据达到发布容差时，立即执行原模型 KKT 审计；审计通过
即可结束，无需为更紧的内部残差继续迭代。负容差仍会关闭正常终止，以保留
既有测试契约。该改动将 `greenbea` 从约 14.7 s 降至约 2.5 s，同时没有改变
发布阈值。

### 5.6 固定列消除与并行 SPD 后端

固定变量在整个 barrier 轨迹中满足 `dx_j = 0`、`theta_j = 0`，因此不再进入
normal equations 的符号图；原始变量值仍保留在残差与最终审计中。对于符号
分析显示 `flop/nnz(L) > 200` 的高计算强度 SPD 法方程，数值分解改用 MKL
PARDISO LLT，而低强度稀疏图继续使用 CHOLMOD。该结构路由把 `pilot87` 从约
1.92 s 降到约 0.97 s，没有按案例名或矩阵维数选择后端。

### 5.7 稳定增广相位的单例列静态凝聚

对增广 KKT `[D G'; G -delta I]` 中有屏障曲率且只连接一个约束的列，Native
在数值内核内部做精确 Schur 凝聚：对偶对角加入 `-g_j^2 / D_j`，每个 RHS
加入对应修正，求解后解析恢复完整 `dx_j`。这不是模型 presolve，不改变
barrier 轨迹或原模型审计；新增回归测试直接验证带显式对偶对角的 KKT 装配
和回代残差。

凝聚仅用于 Auto 选择的稳定增广相位，该相位保持 `sqrt(epsilon)` 级主元分离。
因 normal 失败而启动的恢复相位需要正则化随 `mu` 消失，若凝聚会使
`1/(d+reg)` Schur 项在末期放大，因此保留完整拟定 KKT。该数值相位边界使
`dfl001` 的 1,370 个单例列从分解图退出，迭代数保持 48，时间从 4.421 s 降至
3.385 s；没有案例身份或维数特判。

### 5.8 同图冷启动与完全图闭式符号量

增广 least-squares 冷启动求解 `[I,-A';-A,-delta I]`，首个 Newton 系统则只
改变同一消元图上的对角数值。结构保持路径现在直接使用主迭代的 CHOLMOD
对象完成冷启动，并把已经得到的符号分解留给后续数值分解；需要主元的恢复
路径继续使用 PARDISO。该选择沿用既有 formulation 与后向误差契约，不新增
案例名、维数或经验阈值。它消除了“一次性 PARDISO 投影 + 再次 CHOLMOD
分析”的重复成本，同时保留 least-squares 初值带来的迭代数下降。

当 normal equations 的下三角模式已经完全稠密时，任意排序下的消元图仍是
完全图，因此 `nnz(L)=m(m+1)/2`、factor work 为
`sum(k^2)=m(m+1)(2m+1)/6`。路由现在使用这两个精确闭式量，不再调用 CHOLMOD
重新发现同一完全图；若最终仍选择 normal，则按需执行真实符号分析。该变换
将 `fit2p` 从紧邻改动前 full90 的 445.4136 ms 降到 334.2549 ms，迭代数保持
15，原模型审计结果不变。增广 least-squares 初值同时将 `dfl001` 从凝聚阶段
的 48 次迭代降到当前 38 次。

### 5.9 `greenbea`：数值秩事件与恢复轨迹

`greenbea` 的活动等式 SparseQR 结构秩为 `2196/2199`，缺少的 3 行正好是空
行；PARDISO 在末期报告的大量扰动主元因此不能解释为同等数量的结构秩亏，
而是屏障 KKT 的数值主元事件。SparseQR 本身约需 1.34 s，不适合进入热路径。

生产恢复现在使用更直接的事件证据：若 normal Cholesky 在新方向形成之前失败，
当前 primal 仍是合法中心路径状态，只重新构造 augmented 的 dual/slack/barrier
状态；warm start 的严格内点投影使用 `1e-6` 裕量，不再被冷启动的 `1.0` 单边界
位移覆盖。该改动把 full90 中 `greenbea` 从 2,636.2311 ms、176 次恢复迭代降到
1,621.1659 ms、44 次，原模型审计仍 accurate。非有限方向与后向误差拒绝不
共享该路径。

### 5.10 `dfl001` 与 `maros-r7`：因子图候选的独立裁决

KKT 内核新增显式稀疏 dual block 接口。消去连接行集合为 `R_j` 的对角 primal
列会精确加入 `-g_j g_j^T / D_j`，同步修正 RHS 并在求解后恢复 `dx_j`；模式
不变时继续 analyze-once/factorize-many。二度列局部上删除一个 primal 顶点和
两条边、最多增加一条 dual 边，但这只保证原始边数不增，不保证排序后的
`nnz(L)`、supernode 形状或 factor 时间下降。

- `dfl001` 的二度候选将 5,274 列凝聚后，CHOLMOD 与 PARDISO 静态统计都曾
  预测下降；但实际 PARDISO 首因子由约 47.8 ms 升到 63.8 ms，三次 A/B 几何
  均值由 2,992.7 ms 退化到 3,654.8 ms。因此生产鲁棒增广路径继续只凝聚
  1,370 个单例列，二度模块保留为经回归验证的候选，不进入该结构。
- `maros-r7` 单独比较 SPD normal 与“凝聚二度列、保留高度列”的 hybrid 图。
  normal 为 `491.296M flops / 1.188M nnz(L)`，hybrid 为
  `494.134M / 1.250M`，两项都更高，故在数值分解前拒绝 hybrid。后端 A/B 中
  PARDISO LLT 约 1.25 s，CHOLMOD 约 2.37 s，继续保留 PARDISO。

这两个结果固化了一条边界：度分布只负责生成候选变换，最终 formulation 与
后端必须由候选因子图在同一生产排序/后端上的总成本裁决，不能按案例名，也
不能把“边数下降”直接等同于“运行时间下降”。

## 6. 验证状态

与 Native IPM 和线性代数路径直接相关的测试全部通过：

| 测试 | 结果 |
|---|---|
| `test_ipm_solver` | 144 assertions / 24 cases，通过 |
| `test_numerical_stability` | 453 assertions / 25 cases，通过 |
| `test_netlib_regression` | 224 assertions / 3 cases，通过 |
| `test_engine_api` | 234 assertions / 43 cases，通过 |
| CTest 全集 | 16/18 targets 通过 |

完整 CTest 剩余两个失败与本次 Native LP/IPM 路径无关：

1. `test_ipopt_parameter_stability`：近相关等式案例返回
   `(1.001262, 0.998738)`，超出测试对各分量 `2e-5` 的固定 margin。
2. `test_milp_solver`：用例 112 的生产求解器和穷举 oracle 都得到 `-10`，但
   额外断言仍硬编码期望 `-6`。

默认 `all` 目标另受一个既有 MSVC 可移植性问题阻塞：
`benchmark/native_dual_bfrt_simd_benchmark.cpp:277` 使用了 MSVC 不定义的
`__VERSION__`。该 benchmark 不在 CTest 中，不影响本次最终 NETLIB 二进制和
报告，但不能将仓库状态表述为“全构建绿色”。

## 7. 最终报告与复现

最终 Native 报告：

- `reports/netlib_native_ipm_structural_features_full90_2026-08-06.csv`
- `reports/netlib_native_ipm_structural_features_full90_2026-08-06.json`
- `reports/netlib_native_ipm_structural_features_full90_2026-08-06.log`

同轮 HiGHS IPM 对照：

- `reports/netlib_highs_ipm_full90_candidate_2026-08-06.csv`
- `reports/netlib_highs_ipm_full90_candidate_2026-08-06.json`
- `reports/netlib_highs_ipm_full90_candidate_2026-08-06.log`

构建 benchmark：

```powershell
cmake --build build\windows-vcpkg-release --config Release `
  --target netlib_solver_benchmark --parallel 4
```

复现 Native 全量隔离运行：

```powershell
python tools\run_netlib_isolated.py `
  --executable tests\Release\netlib_solver_benchmark.exe `
  --data-dir tests\data --solver native-ipm-direct `
  --hard-timeout 20 --solver-time-limit 15 --max-iterations 100000 `
  --csv reports\netlib_native_ipm_structural_features_full90_2026-08-06.csv `
  --json reports\netlib_native_ipm_structural_features_full90_2026-08-06.json `
  --log reports\netlib_native_ipm_structural_features_full90_2026-08-06.log
```

复现 HiGHS IPM 对照时只将 `--solver` 改为 `highs-ipm`，并使用对应的三个
`netlib_highs_ipm_full90_candidate_2026-08-06` 输出路径。完整测试命令为：

```powershell
ctest --test-dir build\windows-vcpkg-release -C Release --output-on-failure
```
