# NETLIB 线性规划求解器全面基准

本文记录当前 Windows Release 构建对标准 NETLIB LP 集的可复现实测。原始
CSV、JSON 和日志位于本机 `reports/`，该目录按项目规则不提交 Git；本文只
保留审核所需的环境、口径、结果、异常分析和复现命令。

## 1. 测试范围

- 数据集：90 个具有 NETLIB 官方数值参考目标的 MPS 案例。
- 全量记录：14 个算法配置 x 90 案例，共 1,260 次求解。
- 重复测试：30 个按规模等距抽样的案例 x 5 个重点算法 x 3 次，共 450 次。
- 数据规模：25--6,072 行、32--13,525 列、88--151,120 个非零元；这里采用
  官方 README 的统计，求解器读入后的有效行数可能少一个目标行。
- 未纳入：`standgub`。NETLIB README 没有给出它的数值最优目标，无法应用
  本文统一的目标误差门槛。

数据由 `tools/fetch_netlib.py` 从固定镜像
`coin-or-tools/Data-Netlib@f1cc423067407d55d579c9c35fb01edf860dbc24`
获取。`tests/data/mps_manifest.csv` 保存官方规模、参考目标、来源 URL 和每个
解压 MPS 的 SHA-256；许可与再分发注意事项见
`tests/data/NETLIB_PROVENANCE.md`。

## 2. 测试环境

| 项目 | 值 |
|---|---|
| 日期 | 2026-08-05 |
| 分支/提交 | `release/windows-self-contained` / `5a43072477be` |
| 操作系统 | Windows 11 Pro 64-bit，10.0.26200 |
| CPU | Intel Core Ultra 9 285H，16 核/16 线程 |
| 内存 | 32 GiB |
| 构建 | `build/windows-vcpkg-release`，Release，Ninja Multi-Config |
| 编译器 | MSVC 19.44（VS 2022） |
| 外部后端 | embedded HiGHS 1.14.0、SCIP、Ipopt/PardisoMKL |
| 其他依赖 | PaPILO、SuiteSparse、OpenMP；Gurobi 关闭，MUMPS/SuperLU 不可用 |
| 可执行文件 | `tests/Release/netlib_solver_benchmark.exe` |

HiGHS direct 固定单线程。计时只包含求解调用，不包含 MPS 加载；隔离运行的
正常返回行也不包含子进程启动时间。

## 3. 正确性口径

每个候选解都在 benchmark 读入的原 LP 上重新计算目标、行残差和变量界残差，
不直接相信后端的 `Optimal`/`Solved` 字符串。只有同时满足以下条件才记为
`accurate`：

1. 后端返回 `success=true`；
2. 相对目标误差
   `abs(f(x)-f_ref) / max(1, abs(f_ref)) <= 1e-5`；
3. 归一化原始可行性违反
   `max(row_violation, bound_violation) / max(1, abs(rhs)) <= 1e-7`。

全量核心、SCIP 和 DSE 测试使用 15 秒后端时限、100,000 次最大迭代和单次
重复。Native PDLP 使用 20,000 次迭代。benchmark 内部把 Native LCQP 和
Ipopt 上限进一步限制为 2,000 次迭代，但这两条路径不能可靠执行统一墙钟
时限，因此通过 `tools/run_netlib_isolated.py` 逐案例启动子进程，并设置 30 秒
硬超时。硬超时按失败计入，且不会阻塞后续案例。

## 4. 全量结果

“典型加速比”只在该算法与 HiGHS simplex **都 accurate** 的共同案例上计算，
定义为 `HiGHS simplex 时间 / 当前算法时间` 的几何平均；小于 1 表示当前算法
更慢。失败路径的短耗时不参与这个加速比。

| 算法配置 | success | accurate | 中位数 ms | 几何均值 ms | 总求解 s | 共同准确案例 | 典型加速比 |
|---|---:|---:|---:|---:|---:|---:|---:|
| HiGHS simplex | 90/90 | 87/90 | 7.343 | 10.188 | 10.189 | 87 | 1.000 |
| HiGHS IPM | 89/90 | 86/90 | 11.473 | 14.534 | 5.238 | 86 | 0.684 |
| HiGHS PDLP | 19/90 | 14/90 | 17.970 | 14.623 | 1.864 | 14 | 0.537 |
| Native dual（默认） | 88/90 | 84/90 | 11.440 | 14.866 | 53.939 | 84 | 0.741 |
| Native IPM | 78/90 | 74/90 | 26.014 | 27.629 | 191.324 | 74 | 0.566 |
| SCIP direct MPS | 89/90 | 85/90 | 24.067 | 33.906 | 37.991 | 85 | 0.299 |
| SCIP LP adapter | 87/90 | 41/90 | 38.590 | 50.862 | 25.855 | 41 | 0.162 |
| Native dual Devex | 88/90 | 83/90 | 11.886 | 16.852 | 61.460 | 83 | 0.654 |
| Native dual Structural DSE | 88/90 | 84/90 | 12.319 | 16.274 | 56.756 | 84 | 0.675 |
| Native dual Exact DSE | 88/90 | 84/90 | 12.445 | 15.778 | 57.104 | 84 | 0.699 |
| Native dual Certified DSE | 81/90 | 78/90 | 56.030 | 71.464 | 235.659 | 78 | 0.181 |
| Native PDLP | 4/90 | 0/90 | 118.854 | 96.373 | 29.276 | 0 | 不适用 |
| Native LCQP | 37/90 | 36/90 | 3.811 | 8.576 | 103.181 | 36 | 0.171 |
| Ipopt（LP 转 NLP） | 75/90 | 64/90 | 687.937 | 1066.546 | 452.150 | 64 | 0.010 |

所有后端均在 90/90 案例上可用。Native LCQP 有 3 个 30 秒硬超时，Ipopt 有
9 个；它们的总求解时间包含这些硬超时。LCQP 的低中位数来自 49 个案例快速
报告“残差发散”，不能解释为有效求解速度。

### 4.1 重点算法的规模分层

规模按 benchmark 实际读入矩阵的非零元分为小型 `<5,000`（48 例）、中型
`5,000--19,999`（27 例）和大型 `>=20,000`（15 例）。表中为
`accurate/案例数；中位数毫秒`。

| 算法 | 小型 48 | 中型 27 | 大型 15 |
|---|---:|---:|---:|
| HiGHS simplex | 48/48；2.865 | 27/27；18.818 | 12/15；299.712 |
| HiGHS IPM | 48/48；6.146 | 26/27；25.441 | 12/15；176.685 |
| Native dual（默认） | 45/48；3.081 | 27/27；38.094 | 12/15；702.298 |
| Native Structural DSE | 45/48；3.681 | 27/27；41.189 | 12/15；825.284 |
| SCIP direct MPS | 47/48；10.776 | 27/27；56.207 | 11/15；486.336 |

HiGHS simplex 在小、中型案例上整体最好；HiGHS IPM 在大型组的中位数更低。
Native dual 在小型组接近 HiGHS simplex，但随规模增长差距明显扩大。

### 4.2 Native dual 定价与 DSE 遥测

| 变体 | accurate | 平均 pivot | DSE 初始化 ms | kernel ms/pivot |
|---|---:|---:|---:|---:|
| 默认 | 84/90 | 1097.4 | 0.003237 | 0.038521 |
| Devex | 83/90 | 1086.9 | 0.001718 | 0.038975 |
| Structural DSE | 84/90 | 1070.3 | 0.004444 | 0.042355 |
| Exact DSE | 84/90 | 1071.4 | 0.004217 | 0.041595 |
| Certified DSE | 78/90 | 506.0 | 0.004388 | 0.279220 |

Structural/Exact DSE 略减 pivot 数，但单 pivot 成本上升，当前全量几何均值没有
优于默认实现。Certified DSE 的单 pivot 内核成本约为默认的 7.2 倍，且因超时
和数值失败只通过 78 例；平均 pivot 较少主要是提前失败，不能视为收敛优势。

### 4.3 三次重复测试

30 个规模等距样本上的三次重复用于检查短时计时噪声。`attempts=90` 表示
30 例各重复 3 次。

| 算法 | accurate/attempts | 几何均值 ms | 相对 HiGHS simplex | 案例内中位 CV |
|---|---:|---:|---:|---:|
| HiGHS simplex | 87/90 | 10.111 | 1.000 | 5.3% |
| HiGHS IPM | 84/90 | 14.229 | 0.684 | 4.7% |
| Native dual（默认） | 81/90 | 13.607 | 0.746 | 4.2% |
| Native Structural DSE | 81/90 | 13.349 | 0.761 | 2.9% |
| SCIP direct MPS | 87/90 | 36.315 | 0.274 | 4.8% |

重复测试的典型案例内波动约 3%--5%，不改变全量排序；亚毫秒小案例仍不适合
用于绝对性能结论。

### 4.4 Native IPM：契约修复与理论增强

原始 14 算法表中的 Native IPM 是修改前基线。随后分两层改动，不能混成一次
“调参”结果：

| 阶段 | 算法语义 | success | accurate | 中位数 ms | 几何均值 ms | 总求解 s |
|---|---|---:|---:|---:|---:|---:|
| 初始基线 | 原始实现 | 78/90 | 74/90 | 26.014 | 27.629 | 191.324 |
| P0 契约修复 | 共享总时限、真实迭代/残差、原空间 ranged-row 审计 | 79/90 | 77/90 | 34.190 | 43.660 | 111.120 |
| P1 legacy 对照 | P0 + 固定 `0.9995` 步长、`sigma<=0.5` | 78/90 | 76/90 | 45.347 | 49.704 | 118.790 |
| **P1 中心路径控制** | P0 + 动态互补缓冲步长、标准 Mehrotra `sigma` | **80/90** | **78/90** | **32.627** | **41.374** | **94.557** |
| **P2a 审计停止** | P1 + 原模型 primal/dual/gap 审计、相对候选停止 | **83/90** | **81/90** | **28.009** | **41.141** | **88.004** |

P0 和初始基线来自不同构建轮次，用于说明契约变化；P1 两行来自同一二进制、
同一案例顺序和同一 15 秒预算，是判断算法效果的严格 A/B。P1 在 76 个共同
accurate 案例上相对 legacy 的配对几何平均加速为 **1.199 倍**，新增准确案例
为 `israel` 和 `maros-r7`，没有准确案例回退。`maros-r7` 从 legacy 的 15 秒
超时变为 27 次迭代、约 1.45 秒准确完成；`agg2` 从约 4.42 秒降至 0.26 秒，
`bandm` 从约 0.43 秒降至 4.58 毫秒。P2a 相对 P1 又新增 `d6cube`、`degen3`
和 `shell`，没有 accurate 回退；`shell` 从约 2.00 秒失败变为 108 次迭代、
41.6 毫秒准确完成。

P2a 后仍有 9 个不准确案例：

```text
boeing2 dfl001 e226 greenbea greenbeb lotfi pilot vtpbase wood1p
```

其中 `greenbeb`、`pilot` 返回可行 Optimal，但受官方参考目标精度门槛影响；
其余失败不能靠步长或停止机制解释，下一阶段按
[Native LP 内点法理论设计](native_ipm_design.md)推进 HSD、IP-PMM 正则化和
KKT 质量遥测。当前生产默认仍不改变。

## 5. 关键异常

1. **参考目标敏感案例**：`greenbea`、`greenbeb`、`pilot` 上，HiGHS simplex
   与 SCIP direct 得到几乎相同、且满足可行性门槛的解，但相对官方参考目标
   分别约差 `1.28e-3`、`2.62e-5`、`1.53e-4`。本文仍按统一严格门槛记为
   inaccurate，但不能仅据此断言两个独立后端都求错；应复核 NETLIB 参考值
   的精度和 MPS 解释。
2. **HiGHS IPM**：`ganges` 返回 `Optimal`，但目标误差 `1.80e-2`、归一化
   可行性违反 `4.90e-6`；`greenbea` 返回 `Unknown`。这两例需要保留 simplex
   回退和解后审计。
3. **Native dual**：`modszk1` 目标误差约 `3.40e2`，`tuff` 约 `1.88e-2`，
   `stair` 被误报 infeasible，`pilot` 超时；`greenbea` 的目标还与 HiGHS/SCIP
   明显不一致。这些是默认 Native 路径升为生产主路由前必须修复的回归案例。
4. **SCIP direct**：`dfl001` 达到时限；`forplan` 虽返回 `Optimal`，提取出的
   解为目标 0、归一化违反 1，说明 direct MPS 结果向统一 LP 向量映射存在问题。
5. **SCIP adapter**：87 例声称成功，但只有 41 例通过统一审计。多个案例在
   `Solved` 状态下仍有大目标误差或原始不可行性，当前不能作为可靠 LP 路径。
6. **一阶/实验 Native 路径**：HiGHS PDLP 仅 14/90 accurate；Native PDLP
   86 例达到迭代上限、4 例返回 Optimal，但 0 例通过双门槛；Native LCQP
   只有 36/90 accurate，并有 49 个残差发散和 3 个硬超时。
7. **Ipopt**：作为 LP 转 NLP 的通用途径为 64/90 accurate，且 9 个案例超过
   30 秒。它适合 NLP 能力验证，不适合作为纯 LP 的性能默认值。

## 6. 工程结论

- 当前 LP 默认路由应保持 **HiGHS simplex**；大型稀疏案例可尝试 HiGHS IPM，
  但必须在失败或解后审计不通过时回退 simplex。
- Native dual 可作为开发/影子验证路径。Structural 和 Exact DSE 没有提高
  accurate 数，当前不应替换默认定价；Certified DSE 只适合数值诊断。
- SCIP direct 可作为独立交叉验证后端；修复 `forplan` 的解向量提取前，不应
  无条件接受其状态。SCIP LP adapter 应先解决模型/解映射问题再进入路由。
- Native PDLP、Native LCQP 暂不进入生产自动选择；CI 应保留迭代上限和外部
  进程硬超时。
- 所有后端都应统一执行本文的解后审计。状态字符串不是正确性证明。

## 7. 复现

下载并校验数据：

```powershell
python tools\fetch_netlib.py --output-dir tests\data
```

构建 benchmark：

```powershell
cmake --build build\windows-vcpkg-release --config Release `
  --target netlib_solver_benchmark --parallel 4
```

核心、SCIP 和 Native DSE 全量命令的公共参数为：

```powershell
tests\Release\netlib_solver_benchmark.exe --data-dir tests\data `
  --repeat 1 --time-limit 15 --max-iterations 100000 `
  --solvers <逗号分隔算法键> --csv <结果.csv> --json <结果.json>
```

不可靠墙钟后端使用隔离运行器：

```powershell
python tools\run_netlib_isolated.py `
  --executable tests\Release\netlib_solver_benchmark.exe `
  --data-dir tests\data --solver native-lcqp --hard-timeout 30 `
  --solver-time-limit 15 --max-iterations 20000 `
  --csv reports\netlib_lcqp.csv --json reports\netlib_lcqp.json `
  --log reports\netlib_lcqp.log
```

本次主要原始结果文件为：

- `reports/netlib_windows_full_core_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_scip_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_native_dse_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_native_pdlp_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_native_lcqp_isolated_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_ipopt_isolated_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_stratified_core_repeat3_2026-08-05.{csv,json,log}`
- `reports/netlib_windows_full_native_ipm_after_contract_fix_2026-08-05.{csv,json,log}`
- `reports/netlib_native_ipm_centrality_ab_failures_2026-08-05.{csv,json}`
- `reports/netlib_native_ipm_centrality_ab_full90_2026-08-05.{csv,json}`
- `reports/netlib_native_ipm_audited_stop_full90_2026-08-05.{csv,json}`
- `reports/netlib_native_ipm_original_kkt_cohort_2026-08-05.{csv,json}`
