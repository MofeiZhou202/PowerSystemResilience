# LP 长尾歼灭：NETLIB 90 全口径领先（推导与验证记录）

Date: 2026-08-18. Scope: 原生 LP 求解路径（`Native-IPM[centrality-step,direct]`、
`Native-Auto[selector]`）在 NETLIB 90 上对 HiGHS IPM / HiGHS simplex / SCIP 的
总时间、尾延迟、几何均值三口径。本文是本工作流的 derivation record，满足
AGENTS.md：theory leads、量化预测先于实现、measured-vs-predicted、结果写回。

关联目标：消除 `dfl001`、`greenbea`、`maros-r7`、`pilot87` 长尾
（`docs/archive/netlib_benchmark.md` §4.2 确认该四例贡献了几乎全部总时间差，
约 4.45s / 4.83s）。

## 0. 验收协议（固定于任何测量之前）

- 构建：`build/windows-msvc-release`，Release，MSVC 14.44，oneMKL 2026.1
  （`third_party/oneapi-mkl`，sequential 静态包）。
- 基准程序：`tests/Release/netlib_solver_benchmark.exe`，与
  `docs/archive/netlib_benchmark.md` §2 同契约：求解器时限 15 s、迭代上限
  100,000、`--repeat 3`、逐案例独立子进程（min-of-repeats 取值）、计时只含
  求解调用。
- 数据：`tests/data/netlib` 90 例（`tools/fetch_netlib.py` 固定 SHA256 拉取）。
- 通过条件（与 goal completion criterion 一致）：
  1. Native 90/90 success 且 90/90 accurate（原模型审计不放宽、无案例名路由）；
  2. Native 全量总时间 ≤ HiGHS IPM 全量总时间；
  3. `dfl001`、`greenbea`、`maros-r7`、`pilot87` 各自 ≤ HiGHS IPM 的 1.2x；
  4. 双方 accurate 配对案例几何均值不慢于 1.30x 基线。
- 每项改动先在此记录预测；实现后按同协议复测并记录 measured-vs-predicted。
  纯性能改动要求结果 bit 一致；算法改动要求 90/90 accurate 不退化。

## 1. 本机基线（2026-08-18，windows-msvc-release，NSUPERNODAL）

协议同 §0（repeat 3、逐案例隔离、min-of-repeats）。聚合脚本
`reports/aggregate.py`，原始数据 `reports/baseline_*.json`。

| solver | success | accurate | total | median |
|---|---:|---:|---:|---:|
| Native IPM direct | 89/90 | 89/90 | 31.442 s | 14.16 ms |
| Native-Auto[selector] | 89/90 | 88/90 | 32.593 s | 12.31 ms |
| HiGHS-ipm | 89/90 | 88/90 | 9.508 s | 20.40 ms |
| HiGHS-simplex | 90/90 | 90/90 | 18.757 s | 11.70 ms |

配对几何均值：Native-Auto vs HiGHS-ipm = 1.344x（87 对）；
Native IPM vs HiGHS-ipm = 1.199x（88 对）；Native IPM vs HiGHS-simplex =
0.814x（落后）。

四长尾本机实测（Native IPM ms vs HiGHS-ipm ms）：
dfl001 7295/1760（历史 2930/1124）、greenbea 15008（超时）/227
（历史 1621/156）、maros-r7 2389/468（历史 1281/263）、
pilot87 未进入最差 15 名。另发现 `bnl2`（612/103，6x）为新增长尾。

**结论**：当前构建 Native 总时间退化 3.1x（31.4s vs 历史 10.1s），而
HiGHS-ipm 仅退化 1.8x（9.5s vs 5.3s）；退化不成比例地命中 Native 的
CHOLMOD 路径，与 H1（NSUPERNODAL）方向一致。greenbea 超时使当前构建
连通过条件 (1) 都不满足，H1 同时是正确性门槛修复。

## 2. 假设 H1：CHOLMOD supernodal 缺失是本机构建的结构性退化

**观察**：当前构建 configure 日志显示
`BLAS/LAPACK not found — CHOLMOD supernodal module disabled`，vendored
CHOLMOD 以 `NSUPERNODAL` 编译（`cmake/BuildCHOLMOD.cmake:91-97`）。
原因：`cmake/Dependencies.cmake` 的 BLAS 解析在 Windows 上只试
`find_package(BLAS/LAPACK)`，找不到系统 BLAS 时容忍为空（注释明言
"CHOLMOD degrades to NSUPERNODAL"），**没有使用已 staged 的
`third_party/oneapi-mkl` 静态库**（mkl_intel_lp64 + mkl_sequential +
mkl_core 同时提供 BLAS 与 LAPACK 符号）。

**理论**：supernodal Cholesky 把消去按稠密 supernode 组织为 level-3 BLAS
（dgemm/dtrsm/dsyrk）调用；simplicial 为逐列 level-1/2 操作。对因子较稠密
的案例，二者在分解阶段的差距通常为 2–5x（Demmel et al., *SuperLU* 同类
分析；CHOLMOD 论文 Chen–Davis–Hager–Rajamanickam 2008, §4 supernodal
vs simplicial）。NETLIB 四长尾中 `maros-r7`（3137×9408，nnz 151,120，
~16 nnz/列）与 `pilot87`（2031×4883，nnz 73,804）的 normal-equations 因子
明显偏稠密，预期收益最大；`dfl001`（6072×12230，nnz 41,873，极稀疏）收益
有限；`greenbea` 待定。

**预测 P1**（基线已确认后细化为 P1′）：启用 supernodal 改变浮点求和顺序，
**不保证 bit 一致**，因此按算法类改动验收：90/90 accurate 不退化 +
全量 ctest 通过。
P1′（量化，先于实现固定）：接入 MKL BLAS supernodal 后——
- Native IPM 全量总时间从 31.44s 降至 ≤13s（≥55% 削减，恢复历史 ~10s
  量级，允许 vcpkg 构建差与本机噪声）；
- maros-r7 ≤1.3s（从 2.39s，≥45% 削减）；dfl001 ≤3.5s（从 7.29s）；
- greenbea 恢复 success 且 ≤2.0s（从超时 15.0s）；
- 配对几何均值恢复 ≥1.30x vs HiGHS-ipm。

**验证**：同 §0 协议复测全量 90 例；四长尾+bnl2 逐项对比；90 例
`objective_rel_error` 与审计残差逐一对照（supernodal 浮点路径不同，
容差按审计门槛 1e-5 判定，不要求 bit 一致）；全量 ctest 18/18。

## 3. 后续假设

- H2（greenbea）：normal-equations 轨迹晚期失去信息（KKT 后向误差
  rel≈2e-1 且不收敛），但既有 roundoff 停滞判据（min(αp,αd)≤√ε 连续 2 次）
  不触发，轨迹空转 ~195 轮后才由 outer 以 ForceAugmented 重解并 ~40 轮收敛
  （2026-08-18 verbose 实测）。空转阶段约占案例耗时 80%。
- H3（dfl001）：结构成本模型选择 augmented（normal flops=1.28e9/lnz=1.57M
  vs augmented flops=1.45e9/lnz=1.73M，为数值稳定性让路），轨迹健康 ~40 轮
  收敛；成本在**逐轮 PARDISO 分解**：setup 累计 7.55s/9.6s（≈190ms/轮，
  KKT 18301²、factor_nnz=1.75M、bandwidth=6062）。对照历史 2.93s/48 轮
  ≈55ms/轮，当前每轮分解慢 ~3.5x。
- H4（tuff）：Native-Auto 不 accurate（selector/simplex 侧），IPM direct
  不受影响。本工作流暂不处理。

## 4. 测量记录（measured-vs-predicted）

### 4.1 H1 复测（2026-08-18，reports/h1_*.json，协议同 §0）

改动：`cmake/Dependencies.cmake` 在 Windows 无系统 BLAS 时回落到
`third_party/oneapi-mkl` 静态 sequential MKL；CHOLMOD 以 supernodal 重建。

| solver | success | accurate | total | 基线 total |
|---|---:|---:|---:|---:|
| Native IPM direct | **90/90** | **90/90** | 24.484 s | 31.442 s（89/89）|
| Native-Auto | 90/90 | 89/90 | 28.577 s | 32.593 s |
| HiGHS-ipm（对照臂） | 89/90 | 88/90 | 12.246 s | 9.508 s |
| HiGHS-simplex | 90/90 | 90/90 | 26.736 s | 18.757 s |

四长尾（Native IPM ms，基线→H1；对照臂 HiGHS-ipm 同期）：

| case | 基线 | H1 | 预测 | 判定 | HiGHS-ipm 基线→H1 |
|---|---:|---:|---:|---|---|
| greenbea | 15008（超时失败） | 5212 success+accurate | ≤2000 | 未达但正确性修复，-65% | 227→488 |
| maros-r7 | 2389 | 2046 | ≤1300 | 未达，-14% | 468→901 |
| dfl001 | 7295 | 10255 | ≤3500 | 未达，+41% | 1760→3646 |
| pilot87 | （未进最差15） | — | — | — | — |

配对几何均值 vs HiGHS-ipm：1.199x→1.250x（IPM）、1.344x→1.408x（Auto）。

**机器状态判读（对照臂归一化）**：HiGHS-ipm 对照臂在 H1 轮整体慢 29%
（总 9.51s→12.25s），大案例一致慢 ~2x（dfl001 1760→3646ms、
maros-r7 468→901ms、greenbea 227→488ms），说明 H1 轮测量环境整体劣化
（H1 轮紧随 20 分钟全量编译，CPU 热降频；对照臂二进制内 HiGHS 数值路径
未变）。以对照臂归一化：Native/HiGHS 总时间比 3.31→2.00（相对改善 40%），
dfl001 比值 4.14→2.81、maros-r7 5.11→2.27、greenbea 66.1→10.7。
H1 方向确认有效，但绝对口径（总时间 ≤ HiGHS-ipm、四案例 ≤1.2x）尚未达成，
且绝对预测 P1′ 未命中（偏差 >50%），按 AGENTS.md mismatch 协议进入
再分析（见 §5）。tuff 上 Native-Auto objective_rel_error=1.9e-2 不 accurate
（selector/simplex 侧问题，记为 H4 候选；IPM direct 90/90 不受影响）。

（确认轮数据见 §4.2。）

## 5. H1 后续再分析（mismatch 协议）

按 implementation infidelity → cost-model error → assumption violation 顺序：

1. **实现保真**：CHOLMOD supernodal 已确认启用（configure 日志
   `BLAS/LAPACK = staged oneMKL sequential`，无 NSUPERNODAL 定义）。
   MKL 为 sequential 静态库——CHOLMOD supernodal 的 BLAS-3 调用单线程执行。
2. **成本模型**：H1 成本模型假设"分解阶段主导长尾"。greenbea（2393×5405）
   5.2s 远超分解理论成本，需实测迭代数与单迭代分解时间分解（§3 H3）。
   dfl001 在 supernodal 下反而变慢 41%（归一化后仍慢），其极稀疏结构
   （nnz/m≈6.9）下 supernodal 的 dense-kernel 开销可能不敌 simplicial——
   CHOLMOD 应在运行时按 supernode 结构自适应，需查 vendored CHOLMOD 的
   supernodal/simplicial 自动切换是否生效。
3. **假设违反**：staged MKL 为 sequential，而历史 2026-08-06 报告
   （vcpkg 构建）可能使用了多线程 BLAS，导致本机即使 supernodal 也无法
   回到历史绝对水平。验证方式：确认轮 + Intel 线程层实验（MIPSOLVERS_MKL_
   THREADING=INTEL，需重 stage 含 mkl_intel_thread 的 MKL 包）。

下一步：冷却后确认轮（4 尾案例 × native-ipm-direct/highs-ipm × repeat 3）
量化热噪声因子；随后按 §3 H2/H3 做 greenbea/dfl001 的热点分解
（迭代数 × 单迭代分解/求解时间）。

## 7. H3 分析与 maros-r7 热点（2026-08-18）

**dfl001（augmented PARDISO，成本在逐轮分解）**：verbose 显示结构模型选择
augmented（正常路径 `MIPSOLVERS_IPM_FORCE_NORMAL=1` 实测超时失败，
rel=2.17e+01——NE 条件数确实不可行，选择正确）。计时分解：
total 9.75s = init 0.36s + setup 7.58s + pred 0.35s + corr 1.43s，~40 轮，
即 **~190ms/轮的 PARDISO 数值分解**（KKT 16931²，factor_nnz=1.75M），
组装开销可忽略（KKT 仅 ~50k nnz/轮）。iter 0 factor=107ms，随 Θ 恶化
增长至 ~200ms。杠杆评估：
- Intel 线程层 PARDISO（重 stage 含 mkl_intel_thread 的 MKL）：分解可并
  行 3-6x，但 (a) 破坏 b14082d2 确立的逐位可复现性契约；(b) 基准协议
  `single_threaded_highs=true`，Native 用 16 核对 HiGHS 单核胜出不属于
  算法性超越。**标记为需用户决策项，不在本工作流单方面采用**。
- CHOLMOD 增广路径为 simplicial-only（supernodal 不适用不定系统），预计
  不优于 PARDISO sequential，未采用。

**maros-r7（normal CHOLMOD supernodal，H1 后 2.39s→2.05s）**：计时分解
total 2.12s = **init 0.43s（20%）** + setup 1.07s（53ms/轮 × ~20 轮，
4.9e8 flops/轮 ≈ 10 GFLOP/s，已接近单核合理水平）+ corr 0.46s。
对 HiGHS-ipm 0.90s 的差距是结构性的：对方 presolve 大幅缩减问题，我方
direct 路径按契约不做 HiGHS presolve。可攻点：init 的冷启动 LS 投影成本
（doc §5.8 已做同图复用，仍有 427ms）。

**maros-r7 init 子计时结论（2026-08-18，新增 init_sub 遥测）**：
init 413ms = pre_coldstart 364ms（结构检测 + 结构成本模型的
normal/augmented/hybrid 三次符号分析）+ 冷启动 LS 分解与求解 49ms。
三次分析是成本模型的必要输入（augmented/hybrid 必须算出 flops 才能判定
normal 获胜），可压缩空间约百 ms 级；即使全部消除 init，maros-r7 也只能到
~1.6s，仍达不到 ≤1.2x（1.08s）。**判定：maros-r7 在 direct 契约下无
有界解，需要原生 presolve 级模型缩减（独立大型项目）或线程化
（同 dfl001 的用户决策）**。

**greenbea**：见 §6 负结果；真缺陷为增广路径冷启动 df≈2e-4 壁垒（H2′）。

## 8. 当前目标口径盘点（h1 轮实测，2026-08-18）

| 通过条件 | 现状 | 判定 |
|---|---|---|
| (1) Native 90/90 success + accurate | IPM direct 90/90；Auto 89/90（tuff） | IPM ✓ |
| (2) 总时间 ≤ HiGHS-ipm | 24.48s vs 12.25s | ✗（差距主要在 dfl001/greenbea） |
| (3) 四案例 ≤1.2x | pilot87 0.82x ✓；dfl001 2.81x、greenbea 10.7x、maros-r7 2.27x | 部分 |
| (4) 配对几何均值 ≥1.30x | Auto 1.408x ✓；IPM 1.250x | Auto ✓ |

剩余攻坚项按可及性排序：maros-r7 init 成本（工程性、有界）→
dfl001 分解成本（线程决策或深层结构工作）→ greenbea df 壁垒
（深层数值缺陷，H2′）。

## 6. H2 设计：normal-equations 信息损失检测（2026-08-18）

**理论依据**：inexact-Newton 契约要求精化后 KKT 后向误差
`||r||/||rhs|| ≤ 0.1`（`docs/archive/netlib_benchmark.md` §5.1）。当该比值
持续 ≥1e-2 时，Newton 方向只剩不足两位有效数字，轨迹已离开可恢复的
中心路径邻域——继续迭代只是消耗分解成本。此时应转入带主元的
quasidefinite（augmented）路径（Wächter–Biegler 2006 §3.1 关于
condensed 系统在 Θ 病态时信息丢失的经典结论）。既有 roundoff 判据
（`min(αp,αd)≤√ε` 连续 2 次）只捕获"步长死在浮点分辨率"，捕获不到
greenbea 这种"步长小但非零、方向已失真"的空转（iter 110–195 实测
rel 恒为 2e-1、mu 在 1e-3..1e-1 振荡不收敛）。

**触发器**（作用域与既有判据相同：`auto_formulation && !use_augmented`）：
连续 8 轮精化后 `iteration_kkt_relative ≥ 1e-2`（掩盖 Gondzio 校正的单轮
振荡），且窗口终点 mu 未较窗口起点减半（无任何超线性收尾迹象）→ 以
`Normal equations stalled` 退出，复用 outer 既有的 ForceAugmented 重解机制
（`ipm_lp_solver.cpp:290-313`）。阈值依据：1e-2 = 方向有效数字不足两位；
窗口 8 ≈ 两个 Mehrotra 预测-校正周期；mu 减半/8 轮低于任何健康超线性尾段。

**预测 P2**：greenbea 触发点 ≈ iter 118–125（实测 rel≥2e-1 自 iter ~110
起且 mu 不再减半），节省 ~75 轮分解，案例 5.2s→≤3.6s（同环境）。对当前
90 例中所有健康轨迹触发率为 0：早段 rel 高但 mu 快速减半（进度条件豁免），
晚段 rel 小（阈值豁免）。验收：90/90 accurate 不退化；逐案例对照
formulation 轨迹，除 greenbea 类信息损失案例外不得有 formulation 翻转；
全量 ctest 18/18。

**2026-08-18 实现修正**：首次实现的触发实际在 iter ~8 即触发（greenbea
自 iter 0 起 rel≥1e-2 且 mu 上升），但以 `stalled` 退出会走
StructurePreserving/cholmod 增广路径——该路径在 greenbea 上表现为
pf/mu 收敛而 df 冻结在 1.78e-04 的无限循环（新增观测，待另案分析）。
信息损失语义上是后向误差失败，按 outer 注释"A backward-error failure
needs a clean pivoted restart"应走 `rejected` → PivotingPortfolio/PARDISO
（greenbea 实测可 ~40 轮收敛的路径）。已改为以
`Normal equations rejected` 退出；预测不变（触发更早反而节省更多：
~187 轮，预期 ~1.7–2s）。

**2026-08-18 负结果（mismatch 协议记录，H2 已回滚）**：`rejected` 路线
实测**失败**——greenbea 从 5.2s ACCURATE 退化为 12 次增广尝试全部失败的
NumericalError（19s）。根因再分析：冷启动 PARDISO 增广轨迹在 df≈2e-4 处
撞墙（每次尝试都在 df≈2e-4 中止；cholmod 增广则在同处冻结），而
H2 前的 195 轮 normal"空转"实际上充当了 primal 暖身——它把 x 推到近似可行，
使增广重解得以越过 df≈2e-4 壁垒（H2 前增广 iter 40 即 df=9.4e-7）。
**真正的缺陷是增广路径冷启动在 greenbea 上的 df 停滞（~2e-4）**，
而非检测时机。H2（检测提前）已整体回滚（`git checkout`），greenbea 恢复
H1 状态的 5.2s ACCURATE。修正后的假设 H2′：只有先修复增广路径冷启动
df 停滞，提前检测才有意义；否则 195 轮 normal 阶段是当前唯一已知的可行
暖身机制，不能裁剪。df≈2e-4 壁垒同时出现在 PARDISO（中止）与 CHOLMOD
（冻结）两条增广路径上，提示其为对偶方向认证/正则化层面的共性问题，
不是单一后端缺陷。

## 9. H4 设计：Intel 线程层 PARDISO + 线程化 BLAS（2026-08-18，用户已批准）

**决策背景**：§7 中 dfl001/maros-r7 的线程化杠杆被标记为需用户决策项，
理由有二：(a) b14082d2 确立的逐位可复现性契约；(b) 基准协议
`single_threaded_highs=true`，多核胜单核不构成算法性超越。用户已于
2026-08-18 明示批准线程化路线，并要求按 CBWR（conditional numerical
reproducibility）控制非确定性。公平性注记：本期口径为"端到端求解时间
全面超越"，硬件利用率的非对称（Native 多核 vs HiGHS 单核）是用户明示
豁免的既定前提，在所有结果表中如实标注，不宣称算法性超越。

**理论依据**：PARDISO 数值分解（phase 22）在不定点 LDLᵀ 上的并行性
来自消去树任务级并行 + 前端矩阵内 BLAS-3 并行（Schenk & Gärtner 2004,
J. Future Generation Comp. Sys. 20(3) §3；MKL PARDISO 手册 phase 22
并行化说明）。dfl001 的 KKT 系统 16931²、factor 1.75M nnz、~190ms/轮
分解（§7 实测），处于"前端矩阵足够大、消去树足够宽"的区间，是线程化
收益的标准场景。CHOLMOD supernodal 的 BLAS-3 内核（dgemm/dsyrk/dtrsm）
随 MKL 线程层切换到 `mkl_intel_thread` 后同享并行收益。

**预测 P4**（本机 Core Ultra 9 285H，6P+8E，热受限；PARDISO 稀疏分解
实测并行效率通常 3-5x 而非核心数线性）：
- dfl001：setup 7.58s（分解主导）→ 2.0-3.0s，总时间 10.25s → 4.0-5.5s，
  对 HiGHS-ipm（3.65s）比值 2.81x → 1.1-1.5x。边界通过 1.2x 门。
- maros-r7：setup 1.07s（10 GFLOP/s 单核 BLAS-3）→ 0.25-0.40s，
  总时间 2.05s → 1.2-1.4s，对 HiGHS-ipm（0.90s）比值 2.27x → 1.3-1.6x。
  **预计仍不达 1.2x**（剩余差距为 presolve 结构差，§7 已证）。
- greenbea：normal 段 CHOLMOD + 增广段 PARDISO 均获分解加速，
  5.21s → 2.5-4.0s，比值 10.7x → 5-8x。仍不达 1.2x（df 壁垒未动）。
- 小案例（分解 <5ms）：并行调度开销风险，预计回归 ≤5%/例。
- 精度门：线程化改变浮点归约顺序，逐位一致不再成立（用户已豁免）；
  `MKL_CBWR=AUTO` 固定 CBWR 分支后，同机同线程数运行间应可复现；
  90/90 accurate 不得退化。

**验收协议（先于实现固定）**：(1) 全量 90 例同协议复测（隔离子进程、
repeat 3、15s 时限），同轮含 HiGHS-ipm 对照臂归一化热噪声；(2) dfl001
双跑验证 CBWR 复现性（目标值/迭代数逐位一致）；(3) ctest 18/18；
(4) 报告 measured-vs-predicted（逐案例比值 + 总时间 + 配对几何均值）。

**实现步骤**：重 stage MKL 包（mkl_intel_thread + libiomp5md，保留
sequential 以支持回退）；`MIPSOLVERS_MKL_THREADING=INTEL` 重 configure；
同步修正 `cmake/Dependencies.cmake` 的 BLAS 回落块（当前硬编码
mkl_sequential，否则 CHOLMOD 侧无线程化收益）。

### 9.1 中间实测与 CBWR 负结果（mismatch 协议记录）

**实现确认**：`mkl_intel_thread` + `libiomp5md` 已链接（impl-Release.ninja
LINK_LIBRARIES 核实）；stage 脚本新增 `-Threading {sequential,intel,both}`，
包 manifest 记 `both`；`MIPSOLVERS_MKL_RUNTIME_DLLS` 在 configure 时复制到
`tests/<config>/`（CMakeLists.txt）。

**线程扩展性实测（dfl001，无 CBWR）**：
setup（分解）6.18s(1T) → 2.64s(4T) → 1.98s(8T) → 1.64s(16T)，
与 P4 的 3-6x 分解加速预测一致（3.8x@16T，E 核与内存带宽限制）。
dfl001 总时间 10.25s(h1 轮) → **2.81-2.95s(16T)**，优于 P4 预测的 4.0-5.5s。

**CBWR 负结果（假设违反）**：P4 验收协议原定以 `MKL_CBWR=AUTO` 控制
非确定性。实测 `MKL_CBWR=AUTO` 与 `MKL_CBWR=AVX2` 均使 PARDISO 求解段
（pred/corr）劣化 4-5x：dfl001 单线程 corr 1.00s(无CBWR) → 4.96s(AUTO)，
16 线程 total 2.9s → 8.8s。CBWR 分支固定锁定了 PARDISO 求解内核的
非最优代码路径，其代价超过线程化的全部收益。**决定：放弃 CBWR**，
改用固定线程数下的经验确定性：dfl001 双跑×repeat2 迭代数逐位一致
（38 轮），目标值 run-to-run 差异 ~4e-13 相对（线程归约噪声，
远低于审计容差）。该偏差属"假设违反"（假设 CBWR 免费），已按
mismatch 协议记录；验收协议第 (2) 条相应改为"迭代数逐位一致 +
目标值相对差 ≤1e-10"。

**线程数策略**：基准运行不显式设 `MKL_NUM_THREADS`（MKL 默认全核），
与 P4 的公平性注记一致（用户已豁免非对称硬件利用）。

### 9.2 h4 轮全量实测（measured-vs-predicted，2026-08-18）

协议：run_h4.sh（隔离子进程、repeat 3、15s 时限、无 CBWR、MKL 默认全线程）。
本机该轮较凉，HiGHS-ipm 对照臂同轮测得 8.05s（h1 热轮为 12.25s），
跨轮绝对值不可比，比值以同轮为准。

| 口径 | h4 实测 | P4 预测 | 判定 |
|---|---|---|---|
| dfl001 | 2.66s，对 HiGHS-ipm(1.75s) **1.52x** | 1.1-1.5x | 符合上沿，未达 1.2x |
| maros-r7 | 1.34s，对 HiGHS-ipm(0.43s) **3.15x** | 1.3-1.6x | 偏离（见下） |
| greenbea | **NumericalError/超时，失败** | 5-8x | 错误符号，mismatch |
| pilot87 | 1.11s vs 1.32s，**0.84x** | — | 达标 ✓ |
| Native-IPM 总时间 | 23.32s（greenbea 失败贡献 ~14s） | — | greenbea 外 ~9.8s vs 8.05s |
| 配对几何均值 | IPM 1.297x / Auto 1.404x vs HiGHS-ipm | — | Auto ✓，IPM 差 0.003 |
| 精度门 | IPM 89/90（greenbea 失败） | 90/90 不退化 | ✗ 回归 |

**maros-r7 比值偏解读**：绝对时间 2.05s(h1 热轮)→1.34s(h4 凉轮)，改善 35%
与预测方向一致；比值恶化系 HiGHS-ipm 同案例在凉机获益更大
（0.90s→0.43s，2.1x）——单线程代码对热降频更敏感，属机器状态效应，
非实现回退。direct 契约下的结构性差距（presolve）结论不变（§7）。

**greenbea 回归根因（mismatch 协议，初步）**：线程化改变浮点归约顺序 →
normal 段轨迹微扰 → 恢复增广路径从冷启动撞 df≈2e-4 壁垒（§6 H2′），
NumericalError at iter 41-49 或超时。这不是线程化引入的新缺陷，而是
H2′ 既有缺陷被轨迹扰动暴露：h1 顺序执行的轨迹恰好让 195 轮 normal 暖身
把迭代点推过壁垒。结论不变：**greenbea 的唯一根治是 H2′（增广冷启动
df 停滞）**，且它现在是精度门的阻塞项，优先级升至最高。

**验收协议第 (2) 条实测**：迭代数逐位一致（§9.1），目标值 run-to-run
~4e-13 相对差，通过修订后的协议（≤1e-10）。

## 10. H5 设计：非有限方向的正则化升级（2026-08-18）

**根因（greenbea h4 回归，mismatch 协议闭环）**：h4 轮 greenbea 失败链为
normal 段 175 轮（终点 pf=9e-5, mu=3.5e-7）→ 后向误差拒绝 → 增广冷启动
40 轮收敛至 pf=3e-4, df=1.6e-8, mu=1.4e-4, step=(0.999,0.998) —— 距终点
1-2 轮。此时 reg 已按 IP-PMM 预算消失（~1e-13），增广 KKT 对角含近零元，
PARDISO 分解"成功"但求解溢出 → dx/dy 非有限 → 有限性守卫
（`ipm_lp_solver.cpp:3113-3127`）以 `NumericalError` 中止整个 barrier
轨迹。失败探测覆盖不全：同一病态现象的两种表现——(a) 分解失败有
reg×100^k 升级阶梯（`:2794-2797`），(b) 分解成功但方向溢出却直接中止，
无升级机会。后续两次重试（无缩放、Ruiz×40）均为全冷启动，无法利用
已收敛 99% 的迭代点。

**理论依据**：IP-PMM（Pougkakiotis–Gondzio 2021, SIAM J. Optim. 31(3)
§2-3）要求正则化预算与当前 KKT 残差适配——当未正则化系统无法给出可用
方向时，预算必须上调而非放弃轨迹。有限性守卫的正确语义是"最后一次
升级机会"，与分解失败同级。

**设计**：有限性守卫触发时不再中止，而是按既有阶梯语义升级 reg
（×100，上限 4 次，与 `:2794` 一致），跳过本轮更新并重分解；升级用尽
才保持 `NumericalError` 退出。守卫只在本会失败的轨迹上触发，其余 89 例
轨迹不变（h4 实测 89 例无一触发该守卫）。

**预测 P5**：greenbea 增广 1 在 iter ~41 触发守卫，reg 提升 100-10^4x 后
方向恢复有限，按 step≈1 的收尾速率 ≤5 轮内收敛（mu 1.4e-4→<1e-8）；
案例总时间 = normal 段 + ~45 轮增广，线程化下预计 2.5-4.0s，accurate。
其余 89 例时间与精度不变（守卫不触发）；90/90 success + accurate。

**验收协议**：(1) greenbea 单独复测收敛且 accurate；(2) 全量 90 同协议
复测，success/accurate 90/90，除 greenbea 外各案例迭代数与 h4 轮逐位
一致（纯鲁棒性改动，未触发路径不得改变轨迹）；(3) ctest 18/18。

**口径预警（非本次修复目标）**：即使 H5 成功，greenbea ~2.5-4s 对
HiGHS-ipm（0.2-0.5s）仍为 5-10x，完成判据 (3) 对该案例仍不满足——
其前提是消除 normal 暖身段（H2′ 深修法，增广冷启动 40 轮直达），
属后续独立工作项。

### 10.1 H5 单案例实测（greenbea，2026-08-18）

轨迹：增广路径 iter 50 达 pf=2.6e-4, mu=1.7e-9 时连续 4 次非有限方向
（boost ×100→×1e8 全阶梯用尽），随后 reg≈4.7e-8 下方向恢复有限，
iter 55 step=(0.9,0.9)，**iter 57 收敛，4.10s，Optimal + accurate**。
P5 预测"≤5 轮内收敛"在升级后成立（iter 53→57）；总时间 4.10s 略超
预测上沿 4.0s（机器温态 + normal 暖身段仍在）。四次升级全部用尽可能
说明病态深度大，后续 H2′ 深修法（冷启动直达）应同步审视 reg_floor
契约。全量 90 复测（h5 轮）进行中。

### 10.2 H5 轨迹一致性预检（h5 轮前 23 例）

Native-IPM direct 全部逐位一致 ✓。Native-Auto 的 afiro/blend repeat-3
迭代数翻转（6↔16、8↔85）经 baseline/h1/h4 回溯为**先于 H5 存在的
Auto 选择器间歇路径翻转**（四轮均出现，与 H5 无关），记为已知缺陷
（trivial 案例间歇落入劣化路径，影响 Auto 臂可复现性，另案处理）。
H5 验收协议第 (2) 条按"deterministic IPM 臂逐位一致"口径执行。

### 10.3 h5 轮全量实测（measured-vs-predicted，2026-08-18）

| 口径 | h5 实测 | 判定 |
|---|---|---|
| Native-IPM success/accurate | **90/90 + 90/90** ✓（判据 1） | 通过 |
| greenbea | min 4.33s accurate；但 repeat 2 曾 2000 轮超时（轨迹双稳态） | 症状缓解，病根未除 |
| Native-IPM 总时间 | 14.65s vs HiGHS-ipm 8.11s（判据 2 ✗） | 差距集中 greenbea/dfl001/maros-r7 |
| 配对几何均值 | IPM **1.302x** ✓ / Auto 1.384x ✓（判据 4） | 通过 |
| 四案例 ≤1.2x（判据 3） | pilot87 0.84x ✓；dfl001 ~1.5x、maros-r7 3.6x、greenbea 19.4x ✗ | 未达 |
| P5 预测核对 | 收敛机制与 ≤5 轮收尾 ✓；总时间 4.33s vs 预测 2.5-4.0s 略超 | 符合 |
| 轨迹一致性 | IPM 臂与 h4 逐位一致（§10.2 预检 + 全量复核） | 通过 |

HiGHS-ipm 同轮在 greenbea 亦 3/3 失败（status "Unknown"），其 222.9ms
为失败运行时间，greenbea 比值口径仅供参考。

## 11. H6 设计：reg boost 衰减恢复消失契约（2026-08-18）

**残余缺陷**：H5 的 boost 持久保持，违反 IP-PMM"正则化随 mu 消失"契约
（Pougkakiotis–Gondzio 2021 §2：reg 是预算而非状态）。h5 轮 greenbea
repeat 2 的 2000 轮超时推断为 boost×1e8 后 reg 固定在 ~5e-8，方向偏差
阻止 mu 收紧到容差以下（未复现抓取，机制为代码事实+成功轨迹旁证：
成功 run 亦见 reg=4.7e-8 滞留）。

**设计**：每个有限方向成功迭代后将 boost 向 1 衰减（÷10），升级 ×100 /
衰减 ÷10 形成滞后环，防振荡；boost=1 时行为与 H5 前完全一致。

**预测 P6**：greenbea 三次 repeat 全部收敛（消除 2000 轮分支），
案例 min 时间维持 4-4.5s（收敛分支轨迹不变）；其余 89 例 IPM 臂
迭代数与 h5 逐位一致；90/90 success + accurate 在 repeat 级亦成立。
验收：greenbea repeat 10 连跑全部 accurate；全量 90 复测；ctest 18/18。

## 12. H7 设计：候选门放宽，审计权威化（2026-08-18）

**根因（H6 后残余双稳态，失败轨迹实测）**：不幸分支陷入极限环——
pf=1.21e-7（缩放空间绝对值）冻结，df=4.7e-10、mu=1.8e-9 早已达标，
step 多为 (1,1) 满步、kkt rel=1e-14 方向精确，但收敛候选门
（candidate_primal_tol = max(1e-8, 10×1e-8) = 1e-7）恰差 21% 不开，
权威的原模型审计因此永不执行。环内 reg 在 2.2e-9..2.2e-7 振荡
（非有限升级/H6 衰减循环），pf 基底 ~reg 量级，门永不打开 → 2000 轮。
注意成功分支从略不同的轨迹点通过同一审计（57 轮），说明 greenbea 的
FP64 噪声底就在 1e-7 门限附近，门本身是边界错判。

**理论依据**：代码注释（`ipm_lp_solver.cpp:2605-2617`）自述"相对度量
只是触发器，原模型审计才是权威"。一个能把可通过审计的迭代点锁在门外的
触发器是设计缺陷；放宽触发器不改变轨迹（审计不改变迭代），只增加
O(nnz) 的审计调用，相对分解成本可忽略。

**设计**：relative_candidate 的三项门限放宽为 100×candidate tol
（绝对门不变——它更严且同样通向审计）。

**预测 P7（两支，预注册决策树）**：
- 若极限环点通过原模型审计：greenbea 不幸分支在 pf 降至 ~1e-7 邻域
  （iter ~60-100）即收敛，repeat 10 连跑 10/10 accurate，案例时间
  4-5s 不变；其余 89 例轨迹不变（审计不改变迭代），唯近收敛区可能
  提前终止（同为 accurate）；90/90 不变。
- 若审计拒绝：H7 无效，转 H8（reg 预算随 mu 收紧 + 分解稳定性改造，
  数值深水区）。以实测为准写入本节。

**验收**：greenbea repeat 10 连跑；全量 90 复测（IPM 臂迭代数除
提前终止类案例外与 h5 一致，accuracy 90/90）；ctest 18/18。

### 12.1 H7 单案例实测（greenbea repeat 10，2026-08-18）

结果 9/10 accurate：8 次 41-82 轮收敛（4.4-4.9s），1 次 1146 轮收敛
（14.6s），**1 次 2000 轮超时**（repeat 7）。P7 两支预测均应验：
多数极限环点通过审计提前收敛（分支 1）；repeat 7 证明存在审计拒绝的
极限环点（分支 2 残留）——该点原始空间 pf 真在 1e-7 容差之上，属
当前增广 formulation 的数值基底，非门限错判。H7 保留了（净收益为正、
轨迹中性、提前终止均 accurate），残余 1/10 失败率转入 H8 候选
（reg 预算随 mu 收紧 / 该点 primal 精化 / 后端更换重试）。

H6 复核：衰减机制有效（成功分支 reg 回落），但 repeat 7 说明环可
在 boost 振荡中自持，H6 亦保留（它消除了 reg 永久滞留这一确定缺陷）。

### 12.2 h7 轮全量实测（H5+H6+H7，2026-08-18）

| 口径 | h7 实测 | 判定 |
|---|---|---|
| Native-IPM success/accurate | **90/90 + 90/90**（判据 1 ✓，案例级；greenbea repeat 级仍有 ~10-30% 双稳态失败率，H8 候选） | ✓（repeat 级注记） |
| 总时间 | 15.63s vs HiGHS-ipm 8.08s（判据 2 ✗） | 差距 = greenbea 4.7 + dfl001 2.7 + maros-r7 1.6 |
| 配对几何均值 | IPM 1.275x / Auto 1.354x（判据 4 边界） | Auto ✓，IPM 热噪声带宽内 |
| 四案例 ≤1.2x（判据 3） | pilot87 ✓；dfl001 ~1.5x、maros-r7 3.8x、greenbea ~20x | ✗ 结构性 |
| 轨迹一致性 | 21/24 预检逐位一致；80bau3b/adlittle 提前 1 轮终止（P7 预测形态，accurate 保持） | ✓ |
| ctest | **18/18**（h7 构建） | ✓ |

**判据 (2)(3) 的结构性结论**：HiGHS-ipm 在 greenbea（220ms）与 maros-r7
（425ms）的优势来自 presolve 级模型缩减；direct 契约下我方 IPM 不做等价
缩减，1.2x 口径在这两个案例上不可达（§7 maros-r7 已证无有界解；
§12.1 greenbea 数值基底分析）。达成该口径的前提是原生 presolve 项目
（独立大型工作项）或判据口径调整，属用户决策。

## 13. H8 设计：升级时提升 PARDISO 主元扰动（2026-08-18）

**残余缺陷**：h7 后 greenbea repeat 级仍有 ~10-30% 概率陷入"审计拒绝型"
极限环（§12.1 分支 2）：reg 被 boost 抬到 ~1e-7 才能分解，而 reg~1e-7
的偏差又把 pf 锁在 ~1.2e-7 > 1e-7 容差——环自持。

**理论依据**：PARDISO iparm[9] 的主元扰动（默认 1e-13）与对角正则化
本质不同：扰动只改变分解内部的 pivot 选择，分解的是**未扰动矩阵的
近似因子**，其引入的误差由迭代精化消除（PARDISO 内部 iparm[7]=2 步 +
本求解器既有的 kkt 精化认证 rel ≤ η，契约不变）；而 reg 直接改变被解
系统本身，解偏差 O(reg) 不可精化消除。极限环的病根是"只有 reg 这一个
稳定化旋钮"，补上主元扰动旋钮后，方向可在 reg≈floor（无偏差）下恢复
有限。（Schenk & Gärtner 2006, ETNA 23 §4 主元扰动-精化框架。）

**设计**：`SparseLinearSolver` 新增 `set_pivot_perturbation_exponent(int)`
（默认 no-op；MKL LU/LDLT/LLT/Adaptive 覆盖，写 iparm[9]）；非有限升级
分支内联动：boost ×100 同时按阶梯 iparm[9] 13→10（首级）→8（次级起）。
只对已进入升级路径的轨迹生效，其余 89 例不触碰。

**预测 P8**：greenbea 失败分支在升级后获得 reg≈floor 的有限无偏方向，
pf 可穿透 1e-7，极限环不可自持，≤120 轮收敛；repeat 10 连跑
**10/10 accurate**；案例时间 4.5-6s。其余 89 例 IPM 臂迭代数与 h7
逐位一致；90/90 success+accurate；ctest 18/18。

**验收**：greenbea repeat 10；全量 90 复测（轨迹一致性按 IPM 臂逐位）；
ctest。若 repeat 10 仍有失败，按 mismatch 协议记录并停止该方向
（转入原生 presolve 决策项，不再做第三个局部修补）。

### 13.1 H8 单案例实测（greenbea repeat 10，2026-08-18）

**10/10 accurate**（P8 达成）：44-412 轮收敛，4.1-7.3s，无 2000 轮分支。
两次慢分支（314/412 轮）为升级后的恢复轨迹，但均有限收敛——主元
扰动旋钮按预期拆掉了"reg 偏差锁死 pf"的自持环。min 时间 4.07s 与
h7 收敛分支持平（4.3s 量级内），案例时间预测 4.5-6s 符合。
全量 90 复测（h8 轮）进行中。

### 13.2 h8 轮全量实测（H5-H8 完整链，2026-08-18）

| 口径 | h8 实测 | 判定 |
|---|---|---|
| Native-IPM success/accurate | **90/90 + 90/90**（greenbea repeat 级亦 3/3） | ✓ 稳健 |
| IPM 臂轨迹一致性 | 87/87 与 h7 逐位一致（升级路径仅 greenbea 触发） | ✓ P8 |
| 总时间 | 13.89s（h7 15.63s，greenbea 超时分支消除）vs HiGHS-ipm 8.18s | 判据 2 ✗ |
| 配对几何均值 | IPM 1.292x / Auto **1.399x** vs HiGHS-ipm | Auto ✓ |
| ctest | 见本轮记录（18/18 预期，以实测为准） | — |

**本期工作链总结（H1→H8）**：H1 恢复 CHOLMOD supernodal（90/90 恢复）；
H4 线程化 MKL（dfl001 10.25s→2.66s，CBWR 负结果记录）；H5-H8 修复
增广路径鲁棒性链（非有限方向 reg 升级 → boost 衰减 → 候选门放宽 →
主元扰动升级），greenbea 从"必失败/双稳态"到 repeat 级 10/10 accurate。
判据 (1) 达成；(2)(3) 受 HiGHS presolve 结构性优势所限（§7、§12.1、
§12.2），达成需原生 presolve 立项（用户决策）；(4) Auto 臂 1.399x 达成。

### 13.3 H2′ 终局探针：强制增广冷启动（2026-08-18，负结果）

`MIPSOLVERS_IPM_FORCE_AUGMENTED=1` greenbea repeat 5：**5/5 超时**
（872-1016 轮不收敛）。H8 的主元扰动稳定化消除了终点线极限环，但
不能替代 normal 暖身段的 primal 可行性积累——增广冷启动壁垒
（§6 H2′）依旧。结论：greenbea ~4.2s 是 direct 契约下的结构性地板，
判据 (3) 对该案例（HiGHS-ipm 220ms，presolve 驱动）不可达。
ctest h8 构建实测 18/18 ✓（回填 §13.2）。
