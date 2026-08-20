# 原生 LP Presolve 立项推导与设计（2026-08-18）

状态：**已立项**（用户决策，2026-08-18）。本文档是 AGENTS.md「理论先行」
条款要求的推导记录：§1-§3 在实现前固定，含量化预测与验收协议；§6 起按
阶段回填 measured-vs-predicted。

关联文档：`lp_tail_elimination_2026-08-18.md`（H1-H8 长尾歼灭工作链，
其 §7/§12.2/§13.2 已证明判据 (2)(3) 的差距根因是 presolve 级模型缩减，
direct 契约下结构性不可达）、`netlib_benchmark.md`（NETLIB 90 协议）。

---

## 1. 立项依据（实测事实，h8 轮 + 当日探针）

### 1.1 h8 全量基线（reports/h8_*.json，本机 Release，repeat 3，time-limit 15s）

| 口径 | Native | HiGHS-ipm | 比值 |
|---|---|---|---|
| 全量总时间（Auto 臂） | 14.862 s | 8.175 s | 1.82x |
| 配对几何均值（Auto，n=87） | 17.16 ms | 24.00 ms | **0.71x（领先）** |
| greenbea | 4126.6 ms | 220.4 ms | 18.7x |
| maros-r7 | 1281.7 ms | 454.2 ms | 2.82x |
| dfl001 | 2729.5 ms | 1890.7 ms | 1.44x |
| pilot87 | 1272.3 ms | 1417.4 ms | 0.90x（已达标） |
| bnl2 / cycle / perold / greenbeb | 373 / 386 / 212 / 682 ms | 104 / 125 / 80 / 245 ms | 2.6-3.6x |

判据 (1)（90/90 accurate）与 (4-Auto)（几何均值 ≥1.30x 领先）已达成；
判据 (2)（总时间）与 (3)（四案例 ≤1.2x）未达，差距集中在 presolve 敏感
案例。

### 1.2 关键新事实：HiGHS presolve 桥实质性死亡（2026-08-18 探针）

`native-ipm` 臂（`use_highs_presolve=true`）+ `MIPSOLVERS_PRESOLVE_VERBOSE=1`
单案例探针结果：

```
greenbea : [HIGHS-PRESOLVE] presolved_mapping_unavailable: rows 2392->0 ... use_reduced=0  38.0ms
maros-r7 : [HIGHS-PRESOLVE] presolved_mapping_unavailable: rows 3136->0 ... use_reduced=0  83.4ms
dfl001   : [HIGHS-PRESOLVE] presolved_mapping_unavailable: rows 6071->0 ... use_reduced=0  64.7ms
```

三个长尾案例**全部**在 `getPresolveSideState()`（vendored HiGHS 定制扩展）
处拿不到列映射 → `use_reduced=0` → 静默回落全尺寸 direct 求解，并白付
38-85ms presolve 开销。即「现有桥路径」对长尾案例零收益，所谓
"HiGHS-ipm 的 presolve 优势"目前完全无法经桥为我所用。这把
`lp_tail_elimination_2026-08-18.md` §13.2 的推测升级为实测结论：
**自研原生 presolve 是闭合判据 (2)(3) 的唯一路径**（修桥只能治标，
且 postsolve 仍依赖 HiGHS 实例驻留，与「原生内核全面超越」的目标矛盾）。

### 1.3 参照系规模（勘察结论）

HiGHS LP presolve 核心 ≈ 11.6k 行（`highs/presolve/HPresolve.cpp` 8511 +
`HighsPostsolveStack` 2629），深度耦合 HiGHS 内部设施，不可抽取，只能
照结构重写。LP 规则子集（剔除 MIP-only 分支后）是可自洽实现的，清单见 §4。

---

## 2. 理论框架

### 2.1 算法模型

LP presolve 的标准理论是 Andersen & Andersen, "Presolving in Linear
Programming", Mathematical Programming 71 (1995) §2-§4：在保持最优解
可恢复（postsolve）的前提下，迭代应用保最优性的约简规则缩小
(m, n, nnz)。规则目录以 Achterberg et al., "Presolve Reductions in
Mixed Integer Programming", INFORMS J. Computing 32(2) (2020) 的 LP
子集为准；双重ton/代入链的复杂度处理参照 Suhl & Szymanski,
"Supernode processing of mixed integer models", Math. Programming
(1994)。postsolve 协议（每条约简压栈、逆序 undo）参照
`highs/presolve/HighsPostsolveStack.h` 的 ReductionType 设计——这是
工程协议而非新理论。

对偶固定（dual fixing）的正确性条件：列 j 的目标系数 c_j 与全部
约束系数符号共同保证 x_j 恒取其某界不劣（A&A §3.2）；dominated column
判定同理。代入类规则（free column substitution）必须带 fill-in 保护，
否则 nnz 爆炸反噬分解时间（HiGHS `checkFillin` 同款机制，
`HPresolve.cpp:2684`）。

### 2.2 成本模型

IPM 总时间 ≈ N_iter × (t_factor + t_solve + t_etc)，其中 t_factor 对
KKT 维数 m 与 Cholesky 填充 nnz(L) 超线性。presolve 的收益链：

  规则约简 → m↓、n↓、nnz(A)↓ → nnz(L)↓ → t_factor↓（超线性）
           → 条件数改善 → N_iter 不增（通常略降）

h8 证据：HiGHS-ipm 在 greenbea 总耗时 220ms（含其 presolve），我们
direct 4127ms；两方 IPM 内核效率由全套件配对几何均值证明相当
（我们 1.29x 领先）。因此差距几乎完全由「求解的模型尺寸」解释，
而非内核效率。这正是 presolve 能闭合差距的理论根据。

### 2.3 量化预测（实现前固定）

记号：T_native(P4 完成、native-ipm 臂开原生 presolve）vs T_hipm（h8 实测）。

- **P-greenbea**：4127ms → **250-450ms**（比值 1.1-2.0x，目标 ≤1.2x=264ms
  标记为 stretch）。依据：HiGHS 全含 220ms；我们内核效率相当（§2.2），
  presolve 开销按 §2.4-A2 计 40-90ms；风险在迭代数与条件数不确定。
- **P-maros-r7**：1282ms → **≤550ms**（≤1.2x）。HiGHS 454ms 含 presolve。
- **P-dfl001**：2730ms → **≤2270ms**（≤1.2x）。dfl001 nnz 仅 35632 但
  分解重，缩减行数直接降 t_factor。
- **P-pilot87**：不得回退超过 1.2x（当前 0.90x）。风险：presolve 开销
  与规则误判。pilot87 若缩减率低，开销 ≤30ms 可控。
- **P-total**：Auto 臂总时间 14.86s → **≤8.2s**（判据 2）。greenbea
  单项贡献 -3.8s，maros-r7 -0.7s，bnl2/cycle/perold/greenbeb 合计 -1.3s，
  其余案例 presolve 开销 +0.3s 以内。
- **P-精度**：90/90 accurate 不回退。由既有原模型残差审计兜底
  （`ipm_lp_solver.cpp` postsolve 审计 + 失败回退 direct 的三级回退
  模式照搬到原生 presolve 路径）。

### 2.4 假设（逐条可检验）

- A1：HiGHS 在 NETLIB 上的缩减率可由标准规则集的忠实重实现复现
  （A&A 1995 + Achterberg 2020 LP 子集）。检验：P1-P4 每阶段记录
  rows/cols/nnz 缩减率并与 HiGHS 桥 verbose 数字对照。
- A2：我们的 presolve 运行时间 ≤ 1.5× HiGHS presolve（greenbea 38ms /
  maros-r7 83ms / dfl001 65ms 为锚）。检验：P0 遥测字段实测。
- A3：缩减模型上 IPM 迭代数 ≤ direct 迭代数。检验：每案例记录
  iterations 字段对照 h8。
- A4：postsolve 仅需 primal 恢复（IPM 路径无 crossover），审计容差
  沿用现有 `max(1e-10, 10*tol_primal)`。

### 2.5 证伪/重推导触发器

任一阶段验收出现以下情形，停止编码，回到本文档重推导（AGENTS.md §5）：
方向错误（开 presolve 后总时间变劣）；或实测偏离 §2.3 预测区间 >50%；
或 accurate 掉到 89/90 以下。

---

## 3. 架构设计

### 3.1 挂点与组件

- 新组件 `src/engine/presolve/lp_presolve.{hpp,cpp}`：输入输出均为
  `LPModel`（`include/mipsolvers/engine/problem_types.hpp:140`，Eigen
  CSC `A` + `Aeq` + 行双界 `row_lhs/b` + `vars[].lb/ub`），就地维护
  行/列邻接表（复用 `milp_presolve.cpp` 已验证的邻接表模式与
  postsolve 映射结构——它是本仓库内现成的正确性模板）。
- postsolve 栈：每条约简 push 一条变体记录（FixedCol / SingletonRow /
  ForcingRow / FreeColSubstitution / DoubletonEquation / DuplicateRow /
  DuplicateColumn / RedundantRow / LinearTransform ...），求解后逆序
  undo 恢复原始 x；协议照 HighsPostsolveStack，但只保留 LP 需要的类型。
- 开关：接管 `IPMLPOptions::presolve{true}`（当前死字段，
  `ipm_lp_solver.hpp:53`）作为原生 presolve 总开关；
  `use_highs_presolve` 保留为对照臂，验收完成后桥路径降级为
  回归对照，不作删除（另案处理）。
- 接线：`ipm_lp_solver.cpp` 在现 HiGHS 桥调用点（:349）之前先走原生
  presolve；失败/不可用再回落桥 → 回落 direct，三级回退语义不变。
  Auto 选择器 IPM 臂（`native_lp_selector.cpp:108`，当前用默认关）
  在 P4 验收后随默认值一并开启。

### 3.2 遥测（P0 先行）

`SolveStats`（`solver_adapter.hpp:12-79`）新增：
`presolve_ms`、`presolve_orig_rows/cols/nnz`、`presolve_reduced_rows/cols/nnz`、
`presolve_used`（枚举：none/native/highs/fallback）。当前 HiGHS 桥的
presolve_ms 不外发，无对照能力——这是 h8 轮的分析盲区，先补。

### 3.3 IPM 专属门控

IPM 无 basis postsolve（primal-only），因此 HiGHS 中需 basis 恢复的
规则禁用，无 basis 恢复的规则（行加法 sparsify 类）允许——对应
`HPresolve.cpp:5789-5797` 的 `lp_presolve_requires_basis_postsolve`
区分，我们照搬该语义。

---

## 4. 分阶段计划与验收协议（协议在实现前固定）

每阶段流程：实现 → 单测（每规则正/反例 + 随机 LP postsolve roundtrip）
→ ctest 全绿 → NETLIB 90（native-ipm 开原生 presolve vs highs-ipm，
repeat 3，time-limit 15，与 h8 同协议）→ 本文档 §6 回填
measured-vs-predicted → 触发器检查（§2.5）。

| 阶段 | 规则（引用） | 预期主要受益案例 |
|---|---|---|
| P0 | 遥测字段 + 骨架接入（恒等 presolve，默认关，env `MIPSOLVERS_NATIVE_PRESOLVE` 开） | —（基础设施） |
| P1 | 零填充规则：empty row/col、fixed col、redundant row、singleton row（A&A §2.1-2.3） | 全部中小案例；greenbea 已知 fixed_cols=103/singleton_rows=71 |
| P2 | 代入规则：singleton col、doubleton equation、free/implied-free col substitution（fill-in 保护：单次代入后行 nnz 不超过 max(原行 nnz + 列 nnz - 2, 2×原行 nnz)，总 nnz 净增 >5% 时锁定）（A&A §2.4；Suhl&Szymanski） | greenbea（singleton_cols=288）、maros-r7、dfl001 |
| P3 | 界传播与对偶规则：row implied bounds、dual fixing、dominated/forcing col（A&A §3；Achterberg §6.4 的 LP 子集） | bnl2、cycle、perold |
| P4 | 矩阵级：duplicate/parallel rows & cols（hash 签名）、aggregator（密度失控中止）（Achterberg §5；HiGHS `removeDoubletonEquations`/`aggregator` 结构） | greenbea、maros-r7 收尾 |

各阶段验收门（固定）：

- G1 正确性：新增单测全过；ctest 18/18 + 新增用例全绿。
- G2 精度：NETLIB 90 开原生 presolve 90/90 success + 90/90 accurate
  （原模型审计，不放宽）。
- G3 性能：P2 起每阶段总时间单调不劣于上一阶段（同轮对照臂，容热
  降频噪音 ±10%）；P4 达成 §2.3 全部预测区间。
- G4 回归：presolve 关闭路径与 h8 bit 一致（纯新增代码，不改 direct
  路径行为）。

工作量预估：P0 0.5 天；P1 2-3 天；P2 3-5 天；P3 2-3 天；P4 3-5 天；
含验收与文档回填共约 2-3 周（与用户「多周量级」预期一致）。

---

## 5. 风险登记

- R1 容差误判不可行：规则全部使用相对容差（沿用 HiGHS 量级：
  可行性 1e-7、零值 1e-11），不可行判定须双容差确认；单测含边界案例。
- R2 ranged 行（row_lhs/b 双侧有限）：代入规则遇 ranged 行先转松弛
  或跳过，postsolve 记录须含行侧恢复值。
- R3 fill-in 爆炸：P2 的 fill-in 锁 + 每轮规则迭代 nnz 上限（净增 5%
  锁定该规则本轮）。
- R4 postsolve bug → 错误解：由既有原模型残差审计 + 回退 direct 兜底
  （G2 保证审计不放宽）。
- R5 presolve 自身耗时失控：A2 检验 + presolve 内部时间盒
  （默认 2s，超盒返回未缩减模型）。

---

## 6. 阶段执行记录（measured-vs-predicted 回填区）

### P0（2026-08-18，未 commit）

- 预测（§4 G1/G4）：恒等 presolve 接入后 ctest 全绿；关闭路径与 h8
  bit 一致。
- 实测：ctest 19/19（18 + 新 `test_lp_presolve`，3 用例 34 断言）；
  agg（native-ipm-direct）objective=-35991767.286553584、iterations=86，
  与 reports/h8_agg.json 逐位一致；`MIPSOLVERS_NATIVE_PRESOLVE=1` 冒烟
  输出 `[NATIVE-PRESOLVE] identity_p0: rows 488->488 ...`，求解数值不变。
- 偏差：无。结论：P0 达标（G1/G4），进入 P1。
- 遗留小项：遥测字段外发到 benchmark JSON（P1 对照实验需要）；
  双臂同开时遥测 last-writer-wins（单臂实验不受影响）。

### P1（2026-08-18，未 commit）

- 预测（§4 G1/G2/G4 + §2.3）：零填充规则上线后单测全绿、ctest 全绿、
  冒烟案例 ACCURATE 且 objective 与 h8 在 1e-5 内一致、关闭路径 bit
  一致；greenbea 已知 fixed_cols=103/singleton_rows=71 应被捕获。
- mismatch 记录（AGENTS.md §5 触发：greenbea 首次接线的 postsolve 审计
  拒绝，偏离 §2.4-A4 假设）。按协议排查顺序定位：
  1. 实现保真度排查：行级追踪证明违规行（greenbea 行 72/96、等式行
     45/667）**存活**于 reduced 模型，即违规来自 reduced 求解本身而非
     postsolve/规则证书；删除规则证书（固定列代入精确、singleton 收紧
     单调、redundant 界含）经审查无缺陷。
  2. 成本模型错误：根因是 **reduced 模型的发布审计标尺通胀**。
     IPM 的发布审计（`publication_primal_tol = max(1e-10,10·tol_primal)`
     = 1e-7）是相对 reduced 模型全局行侧量级；fixed-col 代入把行侧
     平移到远超原模型任何行侧的量级（greenbea reduced 侧标尺 ≫ 原模型
     的 ~7e2），使 reduced 审计在原行绝对项下实质放宽。A4 假设
     「审计容差沿用 max(1e-10,10*tol_primal)」在原模型侧成立，但
     reduced 求解在同容差下发布的迭代点可以落在原模型审计边界外
     （实测 viol 7.2e-5，收紧 0.1 后仍 1.3e-5 > 1e-7·原标尺）。
  3. 处置（写回本文档，作为 §2.4-A4 的修正）：reduced 求解的发布容差
     按 `clamp(0.1 · scale_orig/scale_reduced, 1e-4, 1.0)` 收紧
     （`solve_lp_impl` 新增 `publication_tol_scale` 参数，默认 1.0 不
     改任何既有调用语义）；同时 presolve 内部采用双容差包络：删除松弛
     kDeleteTol=1e-8（严格在审计包络 1e-7 内一个 decade），不可行判定
     kInfeasTol=1e-7（容差外确认，§5 R1），singleton 界收紧无松弛
     全量应用（clamp 保持盒有效），含混带（两包络之间）的行保留不删。
  4. 修复后实测：greenbea native-ipm-direct 臂 presolve_used=1，
     1804.5ms / 167 iter / ACCURATE rel=4.70e-13（h8 direct 4126.6ms，
     2.29x；§2.3 P-greenbea 预测区间 250-450ms 是 P4 全规则目标，P1
     仅零填充规则，缩减 rows 2392->2315、cols 5405->5229、
     nnz 30877->30144，presolve 自身 1.8ms）。
- 规则实测缩减（verbose 汇总行）：
  - agg：rounds=3 redundant_rows=26 singleton_rows=30；
    rows 488->432 cols 163->163 nnz 2410->2304（0.2ms）。
  - greenbea：rounds=3 empty_rows=3 empty_cols=5 fixed_cols=171
    redundant_rows=1 singleton_rows=73；rows 2392->2315 cols 5405->5229
    nnz 30877->30144（1.8ms；IPM verbose 探针的 fixed_cols=103 是
    1e-9 绝对容差口径，本实现用相对容差口径得 171，singleton_rows
    71->73 同口径差异）。
  - share2b：rounds=2 singleton_rows=3；rows 96->93 cols 79->79
    nnz 694->691（0.1ms）。
- 验收：agg/greenbea/share2b 冒烟 ACCURATE（rel 1.18e-11 / 4.70e-13 /
  3.14e-08），objective 与 h8 基线在 1e-5 内一致；`env=0` 时 agg
  objective=-35991767.286553584 与 h8 逐位一致（G4）；benchmark JSON
  外发 8 个 presolve_* 字段（P0 遗留项关闭）。
- 与设计文档冲突的事实：`§2.4-A4` 按上文第 3 条修正；Auto 选择器 IPM
  臂本阶段显式 `presolve=false` 保持 P1 前路径（§3.1 的「P4 验收后随
  默认值一并开启」不变）。


### P1 验收失败与修复（2026-08-18，未 commit）

P1 全套件（reports/p1_*.json，三臂）验收触发 §2.5 重推导：四案例段错误
（bnl2/pilot/pilot87/gb_forceaug 无 json 产出）+ presolve 臂总 110.8s 对
direct 臂 12.1s 的系统性 100-1000x 慢。按 AGENTS.md §5 顺序排查，根因两
条均为**实现保真度**问题，与事先嫌疑排序（行侧哨兵数值机制居首）不同，
记录如下。

#### 段错误根因：me=0 空指针解引用（实现失真，非数值机制）

- 证伪嫌疑 A（行侧通胀越哨兵 → 标准形转换越界）：bnl2 的 reduced 模型在
  `MIPSOLVERS_IPM_FORCE_NORMAL` / `MIPSOLVERS_IPM_FORCE_AUGMENTED` 下均
  可解且 ACCURATE（307ms / 503ms），数值上完全健康。
- 真根因：P1 压实把全部存活行并入 `A`、`Aeq` 恒为空（me=0），而
  `ipm_lp_solver.cpp` 的 `Aeq_o/Aeq_i` 指针在 me=0 时为空（:725-727）。
  三处代码直接解引用它们（文件内其余十余处均有 `me > 0` 守卫）：
  Auto 构型的 augmented 符号探针（:1173）、hybrid 候选构建（:1247/:1275）、
  增广凝聚循环（:2031）。原模型等式行总在 `Aeq`（me>0），这些分支从未以
  me=0 被执行；reduced 模型是史上首个 me=0 输入 → 段错误。bnl2 命中
  探针，pilot/pilot87/gb_forceaug 命中凝聚（FORCE_AUGMENTED 不走凝聚，
  故 greenbea 崩溃发生在 auto 探针路径）。
- 修复：4 处补 `me > 0` 守卫。纯结构守卫，me>0 模型行为逐位不变（G4）。

#### 慢速根因：零宽松弛行破坏内核 slack 形式契约（实现失真）

- 实测证伪嫌疑 B/C 的主因地位：stocfor1/ship04s/brandy/blend 的
  pub_scale ≈ 0.05-0.11（行侧几乎无通胀），仍 50-500x 慢。
- 真根因：原模型等式行在 `Aeq`（无松弛），P1 压实后以 lhs==rhs 的
  ranged 行留在 `A` → 内核为其生成**零宽松弛**（ub-lb=0）。等式松弛的
  barrier 曲率 θ ~ z/g_u 随 g_u→0 爆炸，法方程/增广系统条件数崩溃。
  ship04s 实测轨迹：迭代 25-60 间 pf 在 4.5e-8~5.7e-7 振荡、KKT 残差
  爆至 1e39、步长 1e-14 —— 停滞水位恰在发布阈值附近。pub_scale 的 0.1
  十年边际（ship04s 比例 1.0 → 目标 1e-8）使停滞必然不收敛：它是放大器，
  不是根因。bore3d 则磨满 max_iter 后触发升级链三轮 ~2s 增广尝试再回退
  direct（presolve_used=3）。
- 修复 (a) 契约恢复：压实时把 lhs==rhs 的行回填 `Aeq`/`beq`（fixed-col
  代入对两侧等量平移，等式精确保持），恢复「`A` 只含真不等式」的内核
  设计契约。reduced 模型数学上恒等，仅表示形式变化。
- 修复 (b) 代入侧移上限：`kSideShiftCap = 1e2` × 原模型全局行侧标尺，
  超过即跳过该 fixed-col 约简（列保留，精确性无损）。理论依据：
  pub_scale = clamp(0.1·ratio, 1e-4, 1) 的正确性不变量是
  pub_scale ≤ ratio（reduced 发布蕴含原模型审计），地板 1e-4 在
  ratio < 1e-4 时破坏该不变量；且收紧目标的可达性要求所需绝对精度
  （~1e-8·scale_orig）高于线性代数噪声（~1e-13·scale_reduced）。κ=1e2
  使 ratio ≥ 1e-2（不变量余量 2 decade）且可达性余量 ~3 decade。
- 修复 (c) 哨兵/溢出硬保护（代码审查中/低三项一并关闭）：fixed-col 代入
  使任一有限行侧越过 1e19 哨兵或 a·v 溢出 → 跳过该约简；singleton 隐含界
  越过哨兵且会真正收紧盒 → 界不应用且**行不删**（原实现删行丢约束，靠
  审计兜底）；activity 累加溢出 → 该侧按无界处理（不错判不可行、不错删
  冗余行）。

#### 修复后实测（同协议：repeat 3 median，time-limit 15，max-iterations 100000）

段错误四案例（native-ipm 臂，修复前 exit 139 无产出）：

| 案例 | 结果 | 目标值 vs h8 基线 |
|---|---|---|
| bnl2 | 776.8ms ACCURATE | 1811.2365425 vs 1811.2365675（rel 1.4e-8）|
| pilot | 992.5ms ACCURATE | -557.4897284 vs -557.4897042（rel 4.3e-8）|
| pilot87 | 1139.4ms ACCURATE | 301.7103496 vs 301.7103511（rel 5.2e-9）|
| gb_forceaug | 不再段错误；与 h8 基线同等超时（h8_gb_forceaug direct 臂 5/5 Time limit，该探针案例两臂本就无法在 25s 内收敛；进程正常退出并产出结果行）| — |

五慢案例（presolve 臂 median vs reports/p1_*.json direct median）：

| 案例 | 修复前 | 修复后 | p1 direct | 比值（门 ≤2x）|
|---|---|---|---|---|
| bore3d | 6385.6ms | 12.7ms | 7.67ms | 1.65x |
| stocfor1 | 797.7ms | 2.7ms | 2.55ms | 1.06x |
| ship04s | 3264.8ms | 9.4ms | 6.03ms | 1.56x |
| brandy | 2140.1ms | 8.1ms | 4.35ms | 1.87x |
| blend | 445.0ms | 1.6ms | 1.07ms | 1.52x |

greenbea auto 臂（修复 (b) 的直接受益）：reduced 侧标尺 11954→98，
pub_scale 1e-4→1.0e-3，耗时从 2.6s~62s 大幅波动（偶发超时 FAIL）稳定为
2.8-3.1s ACCURATE（rel 1.85e-12；objective -72555248.12986551 vs h8
-72555248.1299674，rel 1.4e-12）。

回归：ctest 19/19 全绿；test_lp_presolve 15 用例 894 断言全过（新增三
用例：代入越哨兵/溢出跳过、singleton 界越哨兵不删行、activity 溢出契约）；
禁用标记 grep 为空；presolve 关闭路径未触及（G4，守卫均为纯新增分支）。

#### 修正的设计事实

- §3.1/P1 记录的压实输出约定「全部行并入 `A`、`Aeq` 为空」作废，改为
  「lhs==rhs 的行回填 `Aeq`/`beq`」。HiGHS 桥的同构约定
  （convert_highs_presolved_lp_to_native）从未真正被求解过（桥
  mapping_unavailable），不能作为正确性依据；该约定同样值得另案审查。
- §6 P1 mismatch 第 3 条的 pub_scale 机制保留，但其成立前提（ratio 不
  低于地板）现由修复 (b) 的 kSideShiftCap 在 presolve 侧保证。
- P2-P4 阶段计划不变；新增规则若引入非等量行侧变换，须重新检验
  kSideShiftCap 不变量与 `Aeq` 回填契约。

### P1 第二轮 mismatch 修复（2026-08-18，未 commit）

修复后全套件（reports/p1_*.json，repeat 3）：presolve 臂 90/90 success +
90/90 accurate，但总时 22.2s 对 direct 臂 12.9s，两个病态案例 modszk1
（4004ms 对 direct 69ms）与 fffff800（5535ms 对 direct 114ms）触发 §2.5
重推导。按 AGENTS.md §5 顺序排查，事先嫌疑全部证伪，根因与记录如下。

#### 嫌疑证伪记录

- **嫌疑 1（列侧零宽区间，近固定列毁曲率）证伪**：verbose 探针直方图
  显示两案例 reduced 模型均无任何双侧有限列（带宽计数全零）、无近零
  宽行；modszk1 reduced 685 行全部为精确等式（回填 `Aeq`）。第一轮同
  构机制不存在于列侧。
- **嫌疑 2（reduced-to-empty 且 postsolve 点不过审计）证伪**：fffff800
  遥测 reduced 0/0/0 是 **HiGHS 桥遥测覆盖伪影**——原生臂 reduced 求解
  失败后接线继续跑桥臂（native-ipm 臂 use_highs_presolve=true），桥的
  mapping_unavailable 结果（rows 524->0）以 last-writer-wins 覆盖了原
  生遥测（P0 遗留小项）。实际缩减为 524->476 行、854->817 列，且
  postsolve 点本身通过原模型审计（最终解 ACCURATE 来自回退后的 direct
  求解）；「524 行全删是否合法」之问不存在。
- **嫌疑 3（Aeq 回填契约失效）证伪**：两案例 reduced 模型的等式拆分
  正确（modszk1 685/685 行在 Aeq）。

#### 真根因：等价模型上的 barrier 轨迹盆地翻转 + 回退成本失控

两案例共同的因果链是「缩减改变了求解轨迹，而非模型正确性」：

- **modszk1**（仅删 2 行 1 列，0.3% 行）：reduced 与原模型前 3 次迭代
  逐位相同，但 reduced normal 轨迹在 pf=1.38e-8/mu=2.3e-7 处钉死
  （step~1e-9，blocking 分量在两索引间振荡），触发 stalled 判定后进升
  级链：增广轮非有限方向磨 3.8s 失败、第三轮 93ms 成功——而原模型
  direct 轨迹在 iter 76-80 从 mu=7e-2 俯冲到 1e-9 发布（69ms/80 iters）。
  退化模型上轨迹盆地对微小结构扰动敏感；reduced 的 rcond 反而更好
  （7.5e-5 对 5.0e-6），即这不是条件数机制。0.3% 的缩减本就无收益，
  属「不该解的 reduced 模型」。
- **fffff800**（删 9.2% 行）：reduced normal 在 iter 0-2 步长即 ~1e-7
  （冷启动结构失败，17ms 退出）；升级链三轮增广各磨满 max_iter=2000
  （step 钉 1e-14、同一 blocking 分量对 (224,223)、kkt 爆至 1e14）共
  ~5.4s 后回退 direct（98ms 成功；原模型 rcond 2.7e-9 与 reduced
  4.4e-9 同量级）。增广轮曾在 iter 75 逼近发布（pf=1.4e-10）但 df 卡
  1e-6 后轨迹崩溃。属「明显失败后回退成本失控」。
- **放大器（非根因）**：pub_scale 旧公式 `clamp(0.1*ratio, 1e-4, 1)`
  在零通胀（ratio=1）时无条件收紧十年。理论不变量实为
  `pub_scale ≤ ratio`：P1 postsolve 对存活列精确，存活行残差在两模型
  中逐位相同，且内核发布审计与接线原模型审计同为「全局侧标尺相对量」
  （audit_ipm_lp_optimality 的 relative = viol/max(1, scale)）；被删行
  的松弛由 kDeleteTol=1e-8 独立保证，与求解容差无关。

#### 修复（四条）

- **(a) pub_scale 策略函数**：新 `lp_presolve_publication_tol_scale()` =
  `clamp(0.9*scale_orig/scale_reduced, 1e-4, 1.0)`（lp_presolve.cpp，
  不变量推导在实现注释）。0.9 边际覆盖两条独立残差求值路径的末位 ulp
  差（~1e-13，余量 12 个数量级），不再对零通胀模型施加不可达收紧；
  kSideShiftCap 保证 ratio ≥ 1e-2，地板 1e-4 永不破坏不变量（防御性）。
- **(b) 微量缩减拒用门（presolve 侧）**：行、列缩减率均 <1% 时
  use_reduced=false（status="meager_reduction"，维度照常上报）。成本
  模型：IPM 分解成本对尺寸超线性，<1% 缩减的期望收益 <2% direct 时间，
  而一次失败的 reduced 尝试成本与 direct 同阶——期望值为负（modszk1
  实证）。greenbea（3.2% 行）等正常案例不受影响。
- **(c) 投机求解早失败短路（接线侧）**：`direct_solve` 新增
  `speculative` 形参（仅 reduced 调用点置 true）；首轮 ≤8 迭代失败判
  为冷启动结构失败，跳过增广升级链直接回退原模型全升级链——升级链以
  同一冷起点重试同一结构，fffff800 实证磨 3×2000 迭代不恢复。阈值 8
  远高于此类失败的 0-2 迭代、远低于任何成功轨迹（NETLIB 20-80+）；
  greenbea（首轮 ~175 迭代后失败、增广轮成功）不受影响。
- **(d) 遥测守卫**：桥臂仅在原生臂未运行（presolve_used==0）时覆盖
  presolve_* 遥测字段，消除 (b) 类伪影。

#### 修复后实测（同协议 repeat 3 median，time-limit 15，max-iterations 100000）

| 案例 | 修复前 | 修复后 | direct（同轮） | 比值（门 ≤2x）|
|---|---|---|---|---|
| modszk1 | 3988.1ms | 60.4ms（meager 门拒用，use_reduced=0）| 62.3ms | 0.97x |
| fffff800 | 5535.2ms | 119.6ms（早失败短路后回退，presolve_used=3）| 95.4ms | 1.25x |

两案例 3/3 ACCURATE；objective 与 h8 基线一致（modszk1
320.6197290636 vs 320.61972906，fffff800 555679.5688789 vs
555679.56482，rel ≤7.3e-9）。

回归（native-ipm 臂 vs 同轮 direct median；本机当日负载波动大，单发读
数曾虚高 2-4x，以 median 与同轮比值为准）：

| 案例 | presolve 臂 | direct 臂 | 比值 | 绝对门 |
|---|---|---|---|---|
| bore3d | 5.52ms | 5.58ms | 0.99x | ≤20ms ✓ |
| stocfor1 | 2.06ms | 2.76ms | 0.75x | ≤6ms ✓ |
| ship04s | 5.56ms | 7.01ms | 0.79x | ≤15ms ✓ |
| brandy | 7.08ms | 5.23ms | 1.35x | ≤12ms ✓ |
| blend | 1.45ms | 1.18ms | 1.22x | ≤4ms ✓ |

bnl2 4806.9ms / pilot 1289.7ms / pilot87 1305.1ms 均 ACCURATE（repeat 1）；
greenbea presolve 臂 ACCURATE（4.1-11.5s，见遗留风险）。
ctest 19/19 全绿；test_lp_presolve 17 用例 910 断言全过（新增两用例：
pub_scale 不变量五组边界、微量拒用门正/反例）；禁用标记 grep 为空。

#### 修正的设计事实与遗留风险

- §2.4-A4 第二次修正：发布容差公式由「无条件十年边际」改为
  `0.9*ratio`（不变量 `pub_scale ≤ ratio` 的推导见
  `lp_presolve_publication_tol_scale` 实现注释）；第一轮 §6 修正记录第
  3 条中 clamp 的 0.1 系数同时作废（kSideShiftCap 与地板论证不变）。
- **遗留风险 1（greenbea 增广轮轨迹混沌）**：reduced 增广轮在同模型同
  参数下 0.76s-9.0s 波动（PARDISO-Adaptive 线程调度非确定性 + 机器负
  载放大）；高负载下曾观测单次后端调用越过 15s deadline 磨至 54s
  FAIL（pinned step=1e-14 磨满 max_iter）。此为内核增广路径的既有脆
  弱性（第一轮修复前即有 2.6-62s 波动记录），本轮改动不改变其代码路
  径（greenbea 首轮 ~175 迭代，不触早失败短路；pub_scale 只影响停止
  判据不影响轨迹）。建议另案：增广路径停滞检测（normal_tiny_step_
  streak 仅覆盖法方程路径）+ 单次后端调用超时保护。
- **遗留风险 2**：modszk1 类「轨迹盆地翻转」无法先验预测；微量门与
  早失败短路是成本控制而非机制治愈。P2 代入规则会更大程度改变模型
  结构，届时须复验两门。

---

## P1 终审（2026-08-18，第三轮全套件 reports/p1_*.json，未 commit）

协议：§4 固定门。三臂（native-ipm 开原生 presolve / native-ipm-direct
对照 / highs-ipm），90 案例，repeat 3，time-limit 15s。

- G1 ✓ ctest 19/19（test_lp_presolve 17 用例 910 断言）。
- G2 ✓ presolve 臂 90/90 success + 90/90 accurate（原模型审计未放宽）。
- G3 ✓ presolve 臂总时 15.455s < direct 臂 16.777s；配对几何均值
  presolve 1.290x > direct 1.264x（均对 HiGHS-ipm 8.896s/24.63ms）。
  首轮 110.8s → 修复后 15.5s。
- G4 ✓ direct 臂 agg/maros-r7 目标值与迭代数与 h8 逐位一致。
- 与 §2.3 预测对照：P-greenbea 预测 250-450ms，实测 3342ms（13.7x）——
  未达，符合阶段预期（greenbea 大缩减依赖 P2 代入规则，kSideShiftCap
  使 P1 仅缩减 ~3% 行）；P-maros-r7/dfl001 同理留待 P2-P4。
  P-pilot87 ✓（1188ms vs HiGHS 1417ms，0.84x）。
- 遗留观察（P2 复验点）：bnl2 presolve 臂 834ms 反而慢于 direct
  426ms（缩减后模型轨迹更难，§6 第二轮同类现象）；greenbea 仍 13.7x；
  HiGHS 桥在原生臂成功后仍白跑（maros-r7 80ms/dfl001 54ms），
  P4 默认开启时须在原生成功后跳过桥。

P1 判定：**通过**，进入 P2。

---

## P2 首次实现 mismatch：singleton 列不是 free 列（2026-08-19，未 commit）

实现前固定预测为 greenbea 至少捕获既有探针锚点中的约 288 个
singleton 列，行/列缩减率相较 P1 各再提升约 5%，耗时进入 1.0-2.5s；
maros-r7 ≤900ms、dfl001 ≤2.5s。验证命令（HEAD `8d89b1cd`，MSVC Release，
MKL/PARDISO，repeat 1 首探针）：

```
MIPSOLVERS_NATIVE_PRESOLVE=1 MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE=1 \
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data \
  --cases greenbea,maros-r7,dfl001 --solvers native-ipm --repeat 1 \
  --time-limit 15 --max-iterations 100000
```

实测：greenbea P2 三类计数全部为 0（`doubleton_eq=0 singleton_cols=0
free_col_subst=0`），仍为 2392→2315 行、5405→5276 列、
30877→30231 nnz，presolve 3.3ms；maros-r7 与 dfl001 均 no_reduction。
greenbea 求解落入既有增广轨迹长尾并在 21.6s 返回 Time limit；另两例
分别 1476ms/3772ms accurate。结构缩减方向与预测相反，触发 §2.5。

按 AGENTS.md §5 顺序重推导：

1. **实现保真度错误（根因）**：首版只实现 doubleton equation 与
   `isImpliedFree` 成立后的 equality substitution。附录 A.3 已记录 HiGHS
   对 singleton column 的顺序是 dual fixing/stuffing/隐含界更新在先，
   free substitution 在后；NETLIB MPS 列通常带默认下界 0，故
   `singleton_cols=288` 从来不等价于 288 个 free/implied-free 候选。
   把探针结构计数直接当作 free 代入计数是实现前语义映射错误。
2. **成本模型**：未进入任何 P2 变换，无法检验缩减后的 factor 成本；
   greenbea 的超时属于文档 §6 已登记的增广轨迹双稳态，不是 presolve
   运行时间（3.3ms）或 fill-in 成本。
3. **假设修正**：singleton 等式列即使带显式界仍可精确消元，但必须把
   该界投影到等式其余变量。若
   `a_j x_j + s = rhs` 且 `l_j ≤ x_j ≤ u_j`，则新行为
   `rhs - max(a_j l_j,a_j u_j) ≤ s ≤
   rhs - min(a_j l_j,a_j u_j)`；无穷端保持无穷。该式是对
   `x_j=(rhs-s)/a_j` 的直接区间像推导（A&A 1995 §2.4），不需要
   implied-free 假设，且用原等式行承载 ranged 约束时不增加行数或 nnz。
   postsolve 仍按原等式快照恢复 `x_j`。

修正后的 P2 预测（实现前重新固定）：greenbea 捕获的 singleton 等式列
应为 200-288（允许部分候选因侧移哨兵/数值保护拒绝），列再降 3.7-5.3%，
行数不保证同比下降；presolve ≤30ms。仅列缩减对 KKT 的收益弱于原预测，
greenbea P2 耗时目标修正为 1.5-4.0s；maros-r7 ≤1.3s、dfl001 ≤3.5s。
若 singleton 计数仍 <200，或任一 original-model 审计失败，再次停止并按
同一顺序重推导。

### P2 singleton 修正后二次 mismatch（2026-08-19，未 commit）

按上式实现 bounded singleton equality projection 后，21 个规则用例、945
断言全绿；但同一三案例探针仍为 `doubleton_eq=0 singleton_cols=0
free_col_subst=0`，greenbea 结构仍 2392→2315 / 5405→5276 /
30877→30231（3.6ms）。三例均 accurate，耗时 greenbea 4485ms、
maros-r7 1697ms、dfl001 3324ms。singleton 计数低于重新固定的 200 下界，
再次触发本节门。

重排查结论：

1. **实现保真度**：bounded singleton 的代数与正/反例 postsolve 均通过，
   但 P1 输出上没有满足前置条件的候选；不是变换实现拒绝了 288 个候选。
2. **成本/探针语义错误（根因）**：此前 `singleton_cols=288` 来自 HiGHS
   完整 presolve 的规则计数，是界传播、dual fixing、stuffing 等规则级联
   后动态产生的机会，不是原模型或 P1 输出的静态列度数。将完整 presolve
   的末态计数拆给 P2 当独立输入锚点，违反了附录 A.3 已记录的规则顺序。
3. **阶段依赖修正**：P2 三种代入规则保留并以单元/roundtrip 正确性验收，
   但取消其独立长尾性能预测；P3 必须先实现 row implied bounds 与保守
   dual fixing，再与 P2 共同迭代到 fixed point，才能检验 greenbea 候选
   是否出现。先增加只读 verbose 计数（active singleton columns、其中
   equality/ranged 分布及拒绝原因）固定真实基线，再为 P3 重新预测。

本轮没有放宽总体验收目标；只是撤销“P2 单独即可产生 288 个候选”的错误
分阶段假设。P3/P2 联合验收仍受 §2.3 最终时间与 90/90 accuracy 门约束。

### P3 实现前算法卡与量化预测（2026-08-19）

只读遥测确认 greenbea/maros-r7/dfl001 的 P1 输出均为：active singleton
columns=0、singleton equality/ranged=0、doubleton equality rows=0、
free/implied-free equality columns=0。因此 P3 先实现 A&A (1995) §3 的
row implied bounds，并与 P1/P2 共同迭代。

对行 `lhs_i ≤ Σ a_ik x_k ≤ rhs_i`，令去掉 j 后其余项的活动区间为
`[L_-j,U_-j]`。若 `a_ij>0`，则
`x_j ≥ (lhs_i-U_-j)/a_ij`、`x_j ≤ (rhs_i-L_-j)/a_ij`；若
`a_ij<0` 两侧交换。无穷残余活动不产生对应界；界冲突在 1e-7 相对包络外
才判 infeasible，1e-8 含混带内不收紧。实现用每行有限和+无穷贡献计数，
把朴素 `O(Σ degree_i²)` 降为 `O(nnz)` 每轮（Achterberg et al. 2020
§6.4 的活动界传播框架）。

固定预测：greenbea 至少 100 次有效界收紧，并经 fixed/empty/singleton
级联额外删除 ≥50 列，P2 候选从 0 变为正数；maros-r7/dfl001 至少一例
达到 ≥1% 行或列缩减，否则 P3 本轮判定无性能价值。presolve 时间预算
greenbea ≤80ms、另外两例 ≤150ms；三例须 accurate。联合耗时目标仍为
greenbea 1.5-4.0s、maros-r7 ≤1.3s、dfl001 ≤3.5s。若收紧/删除方向错误或
计数偏离上述下界 >50%，再次执行 mismatch 协议。

### P3 首次 mismatch：残余活动无界（2026-08-19，未 commit）

实现 `O(nnz)` row implied bounds 后，23 个用例、951 断言全绿；三案例
短时结构探针却均为 `implied_bounds=0`，P1/P2 的尺寸与候选计数完全不变。
greenbea “≥100 次收紧、额外删除 ≥50 列”的预测方向错误，触发 §2.5。

排查顺序结论：

1. **实现保真度**：四种系数/行侧组合的精确正例与残余活动无界的反例
   均通过；现有 P1 行活动也在同一模型上报告大量无穷端。没有发现符号或
   residual-sum 错误。
2. **成本模型**：传播一轮仅把 greenbea presolve 保持在约 2-4ms，
   不是时间盒阻断；计数为 0 是候选不可证，而非执行未完成。
3. **假设违反（根因）**：三例为等式主导、变量多为单侧无界；对任一列，
   其余项活动区间至少一端无穷。P3 算法卡明确规定无穷残余端不能推出界，
   因而 “row implied bounds 会先制造 P2 候选” 的假设不成立。放宽容差
   无法改变无穷活动，且会破坏正确性。

处置：保留经过验证的 P3 规则，但取消本轮性能预测；不在该方向继续添加
启发式。按原 §4 转入 P4 duplicate/parallel rows and columns 的结构哈希
勘察，这类等价约简不依赖有限活动区间，且原计划明确以 greenbea、
maros-r7 为主要受益案例。先只读统计精确重复/平行候选，再固定 P4 数量与
性能预测；若候选同样为 0，需重新评估“标准规则子集足以复现 HiGHS 缩减”
的 A1，而不能继续盲写规则。

### P4 勘察 mismatch：A1 被证伪（2026-08-19，未 commit）

按 P3 处置先加入只读结构哈希：活动行/列按支撑与归一化系数（1e-12
量化，仅作候选计数，任何删除仍需二次精确验证）统计 parallel groups。
三案例结果一致：`parallel_rows=0 parallel_cols=0`；同时 P2 六类候选仍
全为 0。P4 原预测的 greenbea/maros-r7 结构收益没有输入基础。

这次按 §2.5 排查的结论不是某条实现 bug，而是 **§2.4-A1 被证伪**：
P1-P4 当前列出的标准局部规则子集不足以复现 HiGHS 在三个长尾模型上的
reduced-to-empty 行为。依次观察到 P2 候选为 0、P3 有限活动传播为 0、
P4 平行结构为 0 后，再实现 duplicate 删除或调容差不会改变模型，属于
没有理论输入的代码猜测，必须停止。

下一步改为协议勘察而非规则编码：检查 vendored HiGHS 是否能公开完整
presolve reduction stack 的 reduced LP 与 primal postsolve，而不依赖当前
失败的自定义列映射 `getPresolveSideState()`。若公共 `postsolve` 能接收
native 内核得到的 reduced primal，则可形成“HiGHS 负责已验证的完整等价
约简、native IPM 负责 reduced solve、原模型审计负责发布”的过渡闭环；
它不等同于原立项的完全自研 presolve，因此采用前必须在本文另立预测和
验收，并保留当前自研规则作为独立路径。若该协议也不可用，则本轮无法在
不扩展到完整 11.6k 行 presolve 重实现的情况下满足最终性能目标。

### HiGHS reduction-stack 过渡桥：实现前预测与验收（2026-08-19）

代码审计确认 `highs_presolve_lp` 已保留执行 `presolve()` 的 HiGHS 实例，
`highs_postsolve_primal` 直接调用公共 `Highs::postsolve(HighsSolution)`；
`Highs::postsolve` 明确接受 `kReduced/kReducedToEmpty` 且 primal-only solution
不需要 basis。当前阻断点仅是 `getPresolveSideState()` 的自定义仿射列映射
硬门，而冷启动 IPM/dual-simplex 的采用与 primal postsolve 都不读取该映射；
它只供 forward-map/warm-start 辅助使用。

变更协议：`kReduced` 时始终转换并保留 reduced LP；若 side-state mapping
缺失，状态记为 `reduced_primal_postsolve_only`，forward map 继续明确返回空，
但允许 retained reduction stack 做 primal postsolve。发布仍需原模型 residual
audit，任何 reduced solve/postsolve/audit 失败仍回退 direct，现有安全语义不变。
引用：HiGHS `Highs::postsolve` (`highs/lp_data/Highs.cpp:3533-3565`) 与
`HighsPostsolveStack::undo` (`highs/presolve/HighsPostsolveStack.h:649-796`)。

固定预测：关闭自研 presolve、只开桥的首探针中，greenbea/maros-r7/dfl001
均须 `use_reduced=1` 或 `solved_by_presolve=1`，original-model audit 3/3
通过；按 §1.2 的 presolve 38-85ms 锚和 reduced 模型 native solve，目标
greenbea ≤450ms、maros-r7 ≤600ms、dfl001 ≤2.2s。若 postsolve 失败或耗时
偏离 >50%，按实现保真度→reduced 内核成本→假设顺序重查，暂不改选择策略。

### 过渡桥首次 mismatch：转换器违反等式表示契约（2026-08-19）

关闭自研 presolve、只开桥的 repeat-1 探针：dfl001 6071×12230→
3657×9323（66.3ms presolve），2966ms accurate；maros-r7
3136×9408→2152×6605（161.8ms），936ms accurate；greenbea
2392×5405→952×2990（54.7ms），native reduced solve 15s Time limit。
桥采用与两个 primal postsolve 均成功，证明取消 mapping 硬门的协议正确；
但 greenbea 与 ≤450ms 预测方向相反，触发 §2.5。

实现保真度首查即命中已知缺陷：
`convert_highs_presolved_lp_to_native` 把所有 HiGHS 行放入 `A` 并用
`row_lhs==b` 表示等式，`Aeq` 为空。本文 §6 P1 第一轮 mismatch 已实证
zero-width slack 令 barrier 曲率 `theta ~ z/g_u` 在 `g_u→0` 时爆炸，
并已规定所有精确等式必须回填 `Aeq/beq`；当时还特别记录该 HiGHS
转换器“值得另案审查”。过渡桥是它第一次真正被 reduced solve 使用，
所以 greenbea 重现同类停滞。

处置固定：转换时按 `row_lower==row_upper` 精确拆入 `Aeq/beq`，其余行
保留 `A/row_lhs/b`；矩阵、目标和盒不变，仅恢复 native IPM 的表示契约。
修复后重复三例；greenbea 目标恢复 ≤450ms，maros-r7/dfl001 不得比本轮
936/2966ms 回退 >20%，三例须 accurate。若 greenbea 仍超预测 50%，再查
reduced 内核成本与轨迹，而不继续改转换语义。

### 等式拆分后二次 mismatch：约简规模不是收敛性的充分条件（2026-08-19，未 commit）

按上一节把 HiGHS 精确等式拆回 `Aeq/beq` 后，dfl001 为 2217ms、34 次
迭代且 accurate，maros-r7 为 827ms、12 次迭代且 accurate；两者相对修复前
分别改善约 25% 和 12%，没有触发 20% 回退门。greenbea 总时长却为
5389ms，`presolve_used=3`，仍超过 450ms 预测超过十倍，故再次触发 §2.5。

`MIPSOLVERS_IPM_VERBOSE=1` 将 greenbea 的两段成本拆开：HiGHS 约简后模型
为 952 行、2990 列、23457 nnz，native 标准形为 `m=952, nn=3157,
bw=825`；其 normal-equation 尝试只耗 524ms，但约 160 次迭代后 dual
residual 停在约 `1.78e-4`，primal residual 振荡，未通过发布门。随后原模型
normal/augmented 回退分别再耗约 3915/1093ms，构成总时长主体。

按规定顺序重查：

1. **实现保真度**：精确等式已进入 `Aeq/beq`，维数与 HiGHS presolved LP
   一致；失败是完整 barrier 轨迹的残差停滞，不是转换遗漏或 postsolve
   拒绝。
2. **成本模型修正**：约简内核 524ms 仅比 450ms 预测高 16%，因子成本预测
   基本成立；错误是默认这次尝试必然可发布，因而漏计了约 5s 原模型回退。
3. **假设违反（根因）**：等价约简保留最优解映射，但不保证保留 native
   barrier 冷启动所在的数值吸引域。尺寸缩小是每次 Newton 步更便宜的必要
   条件，不是 residual 单调下降或有限步通过发布门的充分条件。

处置：不调 IPM 容差、步长或迭代常数。只检验一个由当前结构遥测支持的
组合：HiGHS reduced LP 仍有 100 个 active singleton columns，先对它运行
已验证的 native P1-P3 等价规则，再做双层 primal postsolve。设 HiGHS 变换
为 `P -> P_H`、native 变换为 `P_H -> P_N`；求得 `x_N` 后严格按栈逆序执行
`x_H = T_N^{-1}(x_N)`、`x = T_H^{-1}(x_H)`，最后仍用原模型 residual audit
发布。该复合正确性来自 A&A (1995) §2 的逐规则等价变换，以及 HiGHS
`Highs::postsolve`/`HighsPostsolveStack::undo` 的逆序协议（上一节所列源码）。

实现前固定预测与门：第二层 native presolve 在 greenbea 上 <=25ms；只有
实际删除至少 1% 行或列时才采用（现有 meager gate），预期至少删除 30 列，
否则报告不采用且不得改变现有路径。若采用，结构变化应打破当前停滞轨迹，
reduced solve 必须成功、原模型 audit 必须通过，总时长 <=1.0s（相对 5.6s
回退路径至少改善 82%）。dfl001/maros-r7 若第二层不采用，时长与解保持现状；
若采用，三例均不得比各自 2217/827ms 回退超过 20%。若删除量、收敛方向或
时间偏离门限，停止并依次检查双层 postsolve 实现、实际因子成本、singleton
列可消去性假设，不用调参掩盖失配。

### 双层约简首次 mismatch：P2 扫描未在成本盒内完成（2026-08-19，未 commit）

按上一节接入双层栈并给第二层 50ms 硬盒后，greenbea repeat-1 为 7029ms
accurate；详细复测为 5048ms accurate。两次都没有 native presolve summary，
且内核结构仍为 HiGHS 输出的 `m=952, nn=3157, bw=825`，证明第二层返回
`time_box`、没有采用。详细复测的 reduced normal 为 531ms，原模型 normal
和 augmented 分别为 3564/855ms；轨迹与接入前相同，双层 postsolve 尚未执行。

这同时否定了“第二层 <=25ms”和“至少删除 30 列”两个预测，按顺序排查：

1. **实现保真度**：kernel LP 尺寸未变、最终 original audit accurate，说明
   `time_box -> use_reduced=false` 的旁路契约正确；没有错误地把半成品模型
   送入求解或 postsolve。
2. **成本模型失配（当前根因）**：P1-P3 在原 greenbea 上约 2-4ms 不能外推
   到 HiGHS reduced LP。后者虽 nnz 更少，却首次有 100 个 singleton-column
   候选，会进入 P2 代入候选/填充检查；50ms 未完成说明候选扫描成本或级联
   成本至少比预测高一倍。
3. **结构假设尚未检验**：由于 pass 未完成，不能据此断言这 100 列不可消去，
   也不能调 IPM 参数。先区分 build/P1/P3 与 P2 成本，再决定是否保留组合。

下一步仅增加诊断可观测性：`time_box` 早退也打印 status/elapsed，并让第二层
读取既有 `MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX`（master switch 仍由 HiGHS
桥所有，不让 `MIPSOLVERS_NATIVE_PRESOLVE=0` 关闭嵌套诊断）。固定诊断协议：
在 2s 上限下只跑一次 greenbea；若完成且删除 >=30 列，再以实测 presolve
成本重算总成本并验证轨迹；若仍超时或零删除，双层方案判无性能输入并撤回，
不继续扩时间盒。

### 双层约简终止结论：2s 仍超时，撤回执行路径（2026-08-19，未 commit）

诊断覆盖 `MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX=2` 后，greenbea 第二层明确
输出 `time_box: rows 952->952 cols 2990->2990 nnz 23457->23457 ...
2005.3ms`；最终虽 accurate，但总时长升至 8740ms。它既未完成也未产生可
采用的中间结果，偏离 <=25ms 预测约 80 倍，并超过一次 reduced solve 成本
约 3.8 倍。继续扩大 time box 的期望值为负。

按上一节预先固定的处置，撤回 IPM bridge 中的双层执行路径，保留通用
`time_box` verbose 行以便今后诊断。结构假设也据此作废：日志中的 100 个
singleton columns 只是 IPM 标准形（含 slack/bound 扩展）统计，不等于
`LPModel` presolve 的 100 个可代入原列；再次把下游标准形计数当上游规则
候选，与 P2 首次 mismatch 的计数层级错误同型。下一步回到已量化的真实
瓶颈：HiGHS reduced LP 的 531ms normal trajectory 在最优邻域附近不能同时
通过 primal/dual 发布门；先审计失败点的原坐标 primal 是否已经满足原模型
发布条件，再决定是终止判据缺少可发布解识别，还是确有对偶收敛缺陷。

### 分离 primal/dual 证书：实现前推导与预测（2026-08-19）

greenbea reduced normal 轨迹显示同一迭代联合 KKT 门过严地绑定了两个实际
独立的证书：约 iter 95 时 `pf=3.36e-8`，已有高精度 primal 候选但
`df=1.91e-4`；约 iter 155 时 `df=1.00e-10`，已有高精度 dual 候选但
当前 `pf=1.84e-3`。这不是放宽容差的理由，而是终止实现漏掉了 LP 弱对偶
允许的跨迭代组合。

对 minimization 形式，任意 primal 可行 `x_p` 给出上界 `c'x_p`，任意 dual
可行 `(y,z_l,z_u)` 给出下界 `d(y,z)`；二者无需来自同一个 barrier 点。
因此在 original-coordinate audit 通过各自发布容差时，保留最小 primal
objective 和最大 dual objective；若
`|c'x_p-d(y,z)|/(1+|c'x_p|+|d(y,z)|) <= gap_tol`，弱对偶性即认证 `x_p`
达到发布精度。maximization 先按现有 `sense_sign` 转成 minimization，公式
不变。引用：Nocedal & Wright, *Numerical Optimization*, 2nd ed., §13.1
（LP primal/dual 与 weak duality）；当前 `audit_ipm_lp_optimality` 已在原坐标
计算同一 primal residual、dual residual 和 objective gap，因此不引入新
数值定义。

实现协议：只在单边 scaled residual 进入现有 100x cheap gate 后执行原坐标
audit；primal/dual 各自通过严格 publication tolerance 才保存快照。联合
gap 通过后发布保存的 primal 与保存的 dual multipliers，再由函数末尾完整
KKT audit 复核；任何一侧缺失或 gap 不合格时轨迹与回退完全不变。

固定预测：greenbea 应在 <=165 次迭代用 iter≈95 的 primal 与 iter≈155 的
dual 组成证书，reduced kernel <=650ms、含 presolve/postsolve 总时长 <=0.9s，
相对当前约 5.0-5.6s 回退改善 >=82%；dfl001/maros-r7 若本来同迭代收敛则
路径与迭代数不变，耗时不得回退 >10%。三例 original-model audit 3/3 通过，
否则按快照坐标/符号实现→audit 成本→“两侧 objective 已闭合”假设重查。

### 分离证书首次 mismatch：两侧可行但 objective 未闭合（2026-08-19，未 commit）

实现严格的单边 original-coordinate audit 与快照后，greenbea 仍回退并以
6868ms accurate 结束，没有发布 `Optimal (split primal-dual certificate)`。
因此 <=165 次、<=0.9s 的预测方向错误。

当前顺序排查结论：实现仍由函数末尾完整 KKT audit fail-closed，且未出现
错误发布；轨迹时间增加主要是机器波动/原模型回退，不是少量 `O(nnz)` audit。
剩余假设缺口是把 `pf/df` 达标误当成 objective 自动闭合：弱对偶组合还要求
保存的 primal 上界与 dual 下界 gap 达标，而当前结果证明它没有发生。先在
verbose 下输出 `best_primal_residual/best_dual_residual/split_gap`，固定真实量级；
禁止放宽 gap tolerance。若 gap 远大于 1e-7，则撤回此性能路径（可保留理论上
正确但无收益的快照，或为零开销撤回），转查为何 normal 轨迹不能在 primal
可行性保持时继续改善 objective/dual，而不是误称证书完成。

### 分离证书终止结论：三例 gap 均不达标，撤回实现（2026-08-19，未 commit）

verbose 量化结果：greenbea 的最佳单边快照为 primal residual `4.905e-5`、
dual residual `5.923e-6`（二者各自在自身相对标尺下过 1e-7 门），但 split
gap=`1.908e-5`，是发布门的 191 倍。dfl001/maros-r7 的 split gap 分别为
`2.513e-7`/`2.657e-7`，也未过门。强制 augmented 形式求解 greenbea reduced
LP 则 15s time limit，不能作为 normal 失败后的低成本恢复。

因此“跨迭代证书即可消除回退”的 objective-闭合假设被三例共同否定。
分离快照在理论上成立，但目标 workload 没有收益且增加近收敛 `O(nnz)` audit，
按上一节预案撤回实现，避免把无效实验留在生产路径。保留文档记录；不放宽
gap，不把 benchmark 的 1e-5 objective 判据替代求解器 1e-7 KKT 发布契约。

### HiGHS bridge 二次根因：reduced side-scale 放大（2026-08-19）

新增 fail-only 遥测推翻了“reduced kernel 失败”的判断：greenbea reduced
solve 在 iter 163 返回 `Optimal (original KKT audit)`，scaled `pf=4.032e-6`、
其 reduced-LP 原坐标 absolute primal residual=`1.351e-5`、relative primal
residual=`1.130e-9`。HiGHS postsolve 后原模型分项为 row `4.571e-6`、
equality `1.351e-5`、bound `1.730e-6`；原模型 side scale=1，故 relative
residual=`1.351e-5`，被 1e-7 发布门正确拒绝。回退根因不是 postsolve
放大，而是同一 absolute equality residual 在 reduced 模型约 1.20e4 的
全局 side scale 下被掩盖。

这与 P1 native mismatch 的 tolerance-transfer 模型完全同型。已有
`lp_presolve_publication_tol_scale(orig,reduced)=0.9*s_orig/s_reduced` 正是保证
`tol*scale_reduced <= 0.9*tol*scale_orig`；但函数的 `1e-4` 防御下限只因 native
`kSideShiftCap=1e2` 才不会破坏不等式。HiGHS reduction 不受该 cap 约束，本例
真实 ratio 给出约 `7.5e-5`，而下限返回 `1e-4`，已违反函数声明的 transfer
invariant。修正不是新容差，而是移除不适用下限（改为仅防 underflow 的
`1e-12`），native 路径因 cap 保证其返回值不变；HiGHS bridge 把所得 scale
传入 reduced `direct_solve`。引用仍为本文 P1 audit-transfer 推导。

固定预测：greenbea reduced solve 在 <=220 次迭代把 absolute primal residual
压到 <=`9e-8`，HiGHS postsolve 后 original audit 通过，总时长 <=1.0s；
dfl001/maros-r7 original audit 保持通过，耗时不得比 2.0s/0.65s 基线回退
>20%。若 greenbea 轨迹在更严门下停滞或总时长 >1.5s，停止并按实现
（scale 是否实际传入）→内核额外迭代成本→global-scale transfer 是否足以
约束 eliminated-row roundoff 的顺序重查，不放宽 original audit。

### HiGHS bridge publication-transfer 验收（2026-08-19，未 commit）

修复后 greenbea repeat-1：`pub_scale=7.528e-5`，iter 165，reduced-LP
absolute primal residual `3.813e-8`；HiGHS postsolve 后 original audit 通过，
553.85ms accurate。相对修复前 4.4-6.9s 的 fallback 路径改善约 87-92%，
满足 <=220 iter、<=1.0s 和 <=`9e-8` 三项固定预测。

三案例 native-ipm vs highs-ipm、Release、repeat 3、time-limit 15s、commit
`8d89b1cd` + 本工作树，命令：

```
netlib_solver_benchmark --cases greenbea,maros-r7,dfl001 \
  --solvers native-ipm,highs-ipm --repeat 3 --time-limit 15 \
  --max-iterations 100000 --json reports/bridge_pubscale_tail_r3.json
```

| case | Native median | HiGHS-IPM median | accuracy | measured / fixed target |
|---|---:|---:|---|---|
| greenbea | 536.20ms | 213.51ms（3/3 fail） | native 3/3 | <=1.0s，达标；HiGHS 无 accurate 时间 |
| dfl001 | 2080.71ms | 1702.21ms | 双方 3/3 | 1.222x，略超 1.2x 总门 |
| maros-r7 | 634.80ms | 430.48ms | 双方 3/3 | 1.475x，未达 1.2x 总门 |

合计 native 9/9 accurate，HiGHS-IPM 6/9 accurate。dfl001/maros-r7 的 transfer
scale 为 0.866/0.9，迭代仍为 34/12，证明本修复没有用过严 tolerance 制造
剩余时间差；maros-r7 的 HiGHS presolve 单次约 132ms，native reduced kernel
约 512ms，剩余差距属于内核/完整 presolve 效率，不可再由本次 audit-transfer
模型解释。结论：publication correctness 与 greenbea 回退长尾已闭合，但
§2.3 的 dfl001/maros-r7 `<=1.2x` 及“各种性能全面超越”仍未完成，禁止把
9/9 vs 6/9 accuracy 优势表述成全部时间门通过。

正确性验收：`test_lp_presolve` 23 cases/954 assertions、
`test_branch_and_cut` 35/492、`test_numerical_stability` 27/479 均通过；完整
Release `ctest --output-on-failure` 为 19/19（最终二进制 208.38s）。因三案例性能门未
全部通过，按 §4 协议未启动 NETLIB 90 repeat-3。

---

## 7. 近期文献增强：线性求解预算优先的 Mehrotra 路径（2026-08-19）

### 7.1 文献与实现依据

Anjos、Lodi、Tanneau (2020)，*Design and implementation of a modular
interior-point solver for linear optimization*，Mathematical Programming
Computation 14，DOI `10.1007/s12532-020-00200-8`，§3.3、§4、§6.2，把多重
中心性修正明确建模为复用同一分解的额外线性求解，并把最大修正数作为可调实现
参数；其核心结论不是“修正越多越快”，而是线性代数成本必须与算法框架解耦后
按实例结构选择。HiGHS 所带 IPX 的公开实现进一步给出当前直接对照：
`highs/ipm/ipx/ipm.h:10-15` 规定 Mehrotra 变体每轮只需 predictor/corrector
两次线性求解，`ipm.cc:353-447` 给出两次右端构造。经典算法来源为 Mehrotra
(1992)，*On the implementation of a primal-dual interior point method*，
SIAM J. Optim. 2(4)，§2-3；多重修正来源为 Gondzio (1996)，*Multiple
centrality corrections in a primal-dual method for linear programming*，
Computational Optimization and Applications 6，§2-3。

### 7.2 成本模型与固定预测

令 `F` 为每轮数值分解成本、`S` 为一次回代及其右端构造成本、`k` 为 Gondzio
额外修正数，则迭代线性代价为 `F + (2+k)S`。2026-08-19 Release 单次遥测
（`reports/literature_baseline_profile_r1.{json,log}`，commit `a0322e38`，
MSVC 19.44，MKL/CHOLMOD Release）给出：

- `maros-r7`：12 轮，`factor=124.12ms`、`pred=45.51ms`、
  `corr=144.95ms`，多数记录 `gc=3`；故额外修正的观测预算约为
  `144.95-45.51=99.44ms`，占 IPM `550.61ms` 的 18.1%。
- `dfl001`：34 轮，`pred=173.82ms`、`corr=585.40ms`，额外修正预算约
  `411.58ms`，占 IPM `2218.88ms` 的 18.5%。

第一探针把 `IPMLPOptions::max_correctors` 默认值从 3 改为 0，保留显式非零
配置的完整 Gondzio 能力。固定预测：两例准确性门保持通过；`maros-r7` 迭代数
不超过 18、`dfl001` 不超过 51（均为基线的 1.5 倍）；若实际迭代数不超过
HiGHS 的 15/42，则分别回收至少 5%/8% 总时长。焦点 repeat-3 的最终目标仍为
native/HiGHS `<=1.2x`，即按上一轮 HiGHS 中位数 `maros-r7 <=516.58ms`、
`dfl001 <=2042.65ms`。命令固定为：

```
netlib_solver_benchmark --data-dir tests/data \
  --cases maros-r7,dfl001 --solvers native-ipm,highs-ipm --repeat 3 \
  --time-limit 15 --max-iterations 100000
```

若任一准确性门失败、迭代数超过 1.5 倍，或耗时改善符号错误/偏离上述条件预测
超过 50%，按实现忠实度（是否确实无额外回代）→分解/回代成本估计→中心邻域
假设→算法选择的顺序重推导，并先回写本节再改代码。

### 7.3 验收（2026-08-19，commit `a0322e38` + 本工作树）

MSVC 19.44、Release、MKL/CHOLMOD，命令与 §7.2 固定命令一致，报告
`reports/literature_no_gondzio_tail_r3.json`。结果：

| case | Native median / iter | HiGHS median / iter | 相对上一基线 | 验收 |
|---|---:|---:|---:|---|
| dfl001 | 1880.99ms / 41 | 1887.00ms / 42 | 2080.71→1880.99ms，-9.6% | native 3/3 accurate，0.997x HiGHS，达到 <=1.2x |
| maros-r7 | 575.91ms / 14 | 439.60ms / 15 | 634.80→575.91ms，-9.3% | native 3/3 accurate，1.310x HiGHS，仍未达到 <=1.2x |

两例迭代数均优于条件预测的 15/42，实测改善也超过固定的 5%/8%，方向和幅度
均匹配，因此保留默认 `max_correctors=0`。单次分桶复核
`reports/literature_no_gondzio_profile_r1.log`：`dfl001 corr 585.40→184.76ms`
（34→41 轮），`maros-r7 corr 144.95→46.39ms`（12→14 轮），证明收益确由
删除额外回代而非精度放宽；原模型 accuracy 门全部通过。`maros-r7` 剩余约
31.0% 差距仍属于初始化/符号构造与每轮数值分解，不得表述成总体性能目标完成。

### 7.4 全套件 mismatch 与重推导（2026-08-19）

焦点性能通过后，Release `test_numerical_stability` 的 27 个 case 中 1 个失败：
`Cached node LP matches fresh solve with Ruiz scaling on` 的约束对偶为 cached
`-1.3546e-6`、fresh `-1.5345e-4`，差 `1.5209e-4`，超过既定 `1e-4` 门；
其余 26 case/479 assertions 通过，`test_lp_presolve` 23/954 通过。故 §7.3
“保留全局默认 0”的结论撤销。实现忠实度无误（profile 全部 `gc=0`）；错误在
成本模型假设：两例大因子的结论被无条件外推到小型 fresh/cached 对照，而 cached
路径本来没有多重修正，两个不同迭代器的有限精度对偶代表会随 fresh 中心轨迹
改变。accuracy 虽未失败，但固定回归门禁止接受该漂移。

重推导：公开选项采用三态，`-1=Auto`、`0=禁用`、`>0=显式上限`。Auto 只在
所选 Cholesky 符号因子的**数值项数组**达到 4MiB 时令 `k=0`，否则保持历史
`k=3`。一次三角回代至少流读 `8*lnz(L)` 字节的数值项，实际还读索引并完成
前/后代，因此 4MiB 是保守的末级缓存流量下界而不是实例维数/名称阈值；
`maros-r7` 的 `lnz=512269`（3.91MiB）略低于 4MiB，故阈值改为十进制
`4,000,000` bytes 以覆盖实测边界，`dfl001` 为 9.72MiB。非 CHOLMOD 或无法
取得符号统计时回退 `k=3`。固定预测：两例仍为 41/14 轮、焦点性能相对 §7.3
波动不超过 10%；上述 cached/fresh dual 门恢复通过；显式 `max_correctors=0/3`
分别保持严格禁用/启用语义。若任一不满足，再按选中因子统计传递→阈值单位→
小模型轨迹的顺序检查，不放宽测试容差。

### 7.5 Auto 验收（2026-08-19，commit `a0322e38` + 本工作树）

`reports/literature_auto_corrector_tail_r3.json` 使用 §7.2 相同 Release flags 和
案例、仅运行 native 臂：`dfl001` 3/3 accurate、41 轮、中位 1812.04ms；
`maros-r7` 3/3 accurate、14 轮、中位 632.96ms。相对 §7.3 纯 `k=0` 探针分别
-3.7%/+9.9%，均在固定的 10% 复测波动带内，且迭代数逐次相同，证明 Auto 对
两个大因子选择了同一算法路径。Release `test_numerical_stability` 恢复为
27/27、480 assertions；此前失败的 cached/fresh dual 门通过，故 §7.4 的小因子
保留策略有效。最终性能结论仍以同批含 HiGHS 的 §7.3 为准：`dfl001` 已略快，
`maros-r7` 尚未达到 1.2x。

完整准确性门使用同一 MSVC 19.44、MKL/CHOLMOD Release 构建执行
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`：
19/19 tests passed，包含 `test_lp_presolve`、`test_numerical_stability` 与
`test_netlib_regression`，总实耗 223.27s。

## 8. 超大规模 LP：hypersparse PRICE 的密度相变（2026-08-19）

### 8.1 文献、现状与不可混淆的边界

Hall、McKinnon (2005)，*Hyper-sparsity in the revised simplex method and how
to exploit it*，Computational Optimization and Applications 32，说明 revised
simplex 的关键不是原矩阵整体密度，而是 FTRAN/BTRAN 结果和 PRICE 中间向量的
动态支持集。Huangfu、Hall (2018)，*Parallelizing the dual revised simplex
method*，Mathematical Programming Computation 10(1)，§2-4，把 dual revised
simplex 的 PRICE、FTRAN/BTRAN 与更新阶段分开，并明确指出稀疏性丢失后应转向
适合连续扫描/并行的内核。Hall 2024，*High performance computational
techniques for the simplex method*，沿用同一实现分区；它是近期实现材料，不替代
前两篇同行评审算法来源。

vendored HiGHS 给出本工作树的直接语义对照：`highs/util/HFactorConst.h:37-55`
用预期密度和 5% 当前 RHS 密度控制 hyper-sparse TRAN；
`highs/simplex/HEkkDual.cpp:1840-1852` 在 row-wise PRICE 结果变稠时切换路径。
Native 已有 `HFactorBackend::ftran_indexed/btran_indexed`、packed 支持集和
`PartitionedRowMatrix` 的非基前缀，故缺口不是“从零加入 hypersparse”，而是
PRICE 始终使用 row-wise 累加，没有利用已存在的 CSC 内核处理传播后的稠密区间。

### 8.2 成本模型、基线与固定预测

令 `rho = e_p^T B^-1`，`N` 为非基列集合。当前分区 row-wise PRICE 成本为

```
W_row = sum(i in supp(rho)) nnz(A[i,N]),
```

而 column-wise CSC PRICE 成本为 `W_csc = nnz(A) + O(m+n)`。前者避免基本列
且在 `rho` 超稀疏时严格占优；后者在支持扩散后以连续列流读换掉 timestamp
scatter 的随机写。选择依据必须是本轮实际支持传播和触及非零数，而不是原模型
维数或案例名。

MSVC 19.44、Release、commit `a0322e38` + §7 工作树，命令为

```
$env:MIPSOLVERS_DS_PROFILE=1
netlib_solver_benchmark --data-dir tests/data \
  --cases maros-r7,fit2d,pilot87,fit2p \
  --solvers native-dual-direct,highs-simplex --repeat 1 \
  --time-limit 20 --max-iterations 100000
```

报告 `reports/hypersparse_baseline_r1.{json,log}`。`fit2d` 为 178.16ms vs
HiGHS 177.59ms；`maros-r7` 为 5238.45ms vs 1337.45ms，`rowEP` 平均 12.05%、
PRICE 0.47s/总 5.22s；`pilot87` 为 16339.63ms 且最终 BTRAN 后向误差失败，
HiGHS 6478.24ms，`rowEP` 平均 61.11%、PRICE 2.82s/总 16.32s。故首个探针
只增加强制 CSC A/B，不改变默认路径。固定预测：`pilot87` PRICE 至少下降 20%、
总时长至少下降 3.5%；`maros-r7` 的低支持不得被未来 Auto 路径切到 CSC；
`fit2d` 的 PRICE 仅 0.03s，预期总收益小于 5%。accuracy 仍用原模型门；CSC 与
row-wise 的浮点求和顺序不同，故按算法改动跑完整 Release suite，不要求 pivot
路径 bit-identical。

若 `pilot87` 收益方向错误，或 PRICE/总收益相对预测偏离超过 50%，先依次检查
CSC 是否仍做全长 pack → `W_row` 是否已因非基分区低于密度代理 → cache/并行
成本假设 → 算法选择，并先写回本节再改默认路径。强制探针只有形成完整可诊断
A/B 合同才可进入代码；不能把未验证阈值发布为 Auto 默认。

### 8.3 强制 CSC mismatch 与重推导

`MIPSOLVERS_DS_FORCE_CSC_PRICE=1` 的完整 A/B 探针保留与生产路径相同的
“非基列 + leaving 列”输出语义；`reports/hypersparse_force_csc_r1.{json,log}`
使用 §8.2 相同构建和参数。结果与预测方向相反：`pilot87` PRICE
`2.82->3.34s`（+18.4%），总时长 `16.34->19.61s`（+20.0%）；`fit2d`
PRICE `0.03->0.04s`，总时长 `178.16->222.28ms`（+24.8%）；`maros-r7`
PRICE `0.47->0.51s`，总时长因其它分桶波动为 `5238.45->4977.39ms`。三例
主元数均与基线相同，原来成功的两例仍 accurate，`pilot87` 仍在同一最终 BTRAN
后向误差门失败，故不是 pivot-path 或精度变化造成的表象收益/损失。

按 §8.2 顺序重查：实现确实调用已有 CSC kernel，但该 kernel 必须扫描完整 `A`、
写全长 `product` 并做一次全列 pack，之后才能过滤基本列；生产
`PartitionedRowMatrix` 则只遍历 `supp(rho)` 所触及行的**非基前缀**。因此原模型
把 rowEP 密度当作 `W_row/W_csc` 代理不成立，且遗漏了 CSC 的 `O(n)` 初始化、
pack 和线程调度。修正后的必要条件应直接比较

```
sum(i in supp(rho)) nnz(A[i,N])
    versus nnz(A[:,N]) + dense-vector/pack cost,
```

而当前没有不先扫描 `N` 就能低成本取得右侧的持久非基 CSC 分区；为这一路径
再复制/动态维护一套 CSC 会增加 `O(nnz)` 内存，与超大规模目标相冲突。结论：
撤销强制探针，不发布 Auto CSC；当前 row-wise partitioned hypersparse PRICE 是
已测三例中的正确生产内核。下一步应针对 `pilot87` 的 BTRAN 后向误差与
FTRAN/BTRAN 支持扩散，以及 `maros-r7` 的主元数/状态更新成本，而非继续按
rowEP 密度切 PRICE。

### 8.4 `pilot87` 终止 BTRAN 的只读诊断合同

Higham (2002)，*Accuracy and Stability of Numerical Algorithms*，2nd ed.，
§7.2、§12.1，以缩放后向误差
`||c_B-B^T y||_inf / (||B^T||_inf ||y||_inf+||c_B||_inf)` 判断三角求解
是否与附近问题一致；Native 的 `BasisFactor::backward_error_acceptable`
使用同一分母和 `256 eps` 门，并在一次长双精度残差改进后拒绝不合格结果。
当前 `reconstruct` 丢弃了已计算的 residual、limit、因子更新数和 refinement 标志，
使“改进次数不足”和“fresh factor 本身不稳定”无法区分。

本探针只把 `last_solve_diagnostics()` 追加到既有 BTRAN 失败消息，不增加求解、
重分解或矩阵遍历。成本模型是成功路径额外工作为 0，失败路径仅格式化约 200 bytes；
固定预测为 `pilot87` 仍在同一 cleanup major reconstruction、相同 10024 总 pivot
处失败，消息必须给出 `initial_residual_inf/refinement_rhs_residual_inf/`
`residual_inf/correction_inf/limit/`
`ft_updates/refined/generation`；
`test_dual_simplex` 的 76 cases/23027 assertions 保持通过。若 pivot 数或失败阶段改变，
视为实现不忠实并撤回诊断改动，不据此设计数值修复。

诊断实测满足同一失败阶段与 10024 pivot 合同：fresh factor 的 `ft_updates=0`，
初始 residual 为 `1.9096317219581896e-11`，仅为 limit
`1.3256430733997605e-11` 的 1.44 倍；但一次 refinement 后 residual 反而为
`2.9983950515055473e-9`，放大 157 倍。更关键的是
`refinement_rhs_residual_inf` 与初始 residual 逐位相同。实现忠实度无误，错误在
平台假设：MSVC x64 的 `long double` 与 `double` 同为 64-bit，故
`residual_vector` 的类型转换没有提供扩展精度。继续增加 refinement 次数的预测
符号已经错误，必须停止该方向，先修正 defect 的计算精度。

### 8.5 失败慢路径的补偿残差

Ogita、Rump、Oishi (2005)，*Accurate Sum and Dot Product*，SIAM J. Sci.
Comput. 26(6)，Algorithms 3.1/5.2，以 `TwoSum` 与 FMA `TwoProduct` 累积乘法和
加法的舍入误差；其 `Dot2` 误差界把普通点积的一阶 `O(u)` 累积误差降为近似
`O(u^2)` 项加最终一次舍入。Higham (2002) §3.1、§12.1 给出残差精度决定经典
iterative refinement 有效性的同一误差模型。

实现仅替换 `refine_checked` 已经进入的失败慢路径 `residual_vector`：以
`std::fma(a,x,-a*x)` 回收乘法误差，再用 `TwoSum` 回收每次累加误差；首轮快速
验收、indexed TRAN、pivot 选择和因子更新均不变。每个触及非零增加常数次算术，
FTRAN defect 另需 `m` 个 compensation doubles；这只发生在原本将 refinement
或失败的求解。固定预测：`pilot87` 前 10024 pivot 必须相同；补偿初始 residual
应低于现有 `1.9096e-11`，并使初始解直接过 `1.3256e-11` 门或使一次修正过门；
案例最终必须 accurate，总时长相对同机原路径增加不超过 2%。若补偿 residual
不下降或仍失败，先检查 FMA/TwoSum 符号与 residual 定义，再检查 LU 条件，而不
放宽 `256 eps` 门。算法改动通过焦点案例后运行完整 Release 19-test suite。

实测 mismatch：补偿 residual 为 `1.9096316419359833e-11`，相对原值只下降
`4.2e-8`，仍为 limit 的 1.44 倍；修正后 residual 仍恶化到
`2.998536852979539e-9`，`pilot87` 保持相同 10024 pivot 和失败阶段。因此残差
求和不是主误差源，补偿内核撤回；问题归于 fresh LU 解。vendored HiGHS
`HEkk.cpp:3029-3050` 在数值 trouble 且 update count <10 时把 Markowitz pivot
threshold 从默认 0.1 提到最大 0.5 并重新 INVERT；Native backend 当前始终固定
0.1，缺少这一稳定性升级。

### 8.6 Markowitz 0.5 强制探针

依据 HiGHS `HFactorConst.h:23-29` 与 `HEkk.cpp:3029-3050`，首轮只加入环境变量
`MIPSOLVERS_HFACTOR_PIVOT_THRESHOLD`，允许 `[8e-4,0.5]`，未设置时严格保持
0.1；它用于完整求解 A/B，不形成 Auto 策略。阈值越大，Markowitz 选主元要求
候选相对列最大值更大，通常以更多 fill 换更小的 LU 增长因子（Higham 2002
§9.4）。固定预测：强制 0.5 后 `pilot87` 必须 accurate，后向误差失败消失；
pivot 数允许因舍入路径变化而变，但总时长不得超过 0.1 基线 20%。若仍失败，
说明静态全程阈值不能修复该 basis，撤回探针并研究终止时的定向 refactor；若
成功，再设计只在 fresh solve 失败时 0.1->0.5 的一次性重建，避免全程 fill 成本。

强制 0.5 实测通过：`pilot87` Native 为 `16359.69ms`、9925 dual pivot 加 742
primal cleanup pivot、accuracy `rel=1.26e-6`、归一化 primal violation
`2.80e-15`；同批 HiGHS 为 `6447.52ms`。相对 0.1 同批 Native
`14256.25ms`，增加 14.8%，在 20% 上限内；但全程更强 pivoting 改变路径并增加
597 个 dual pivot，不能作为默认值。强制环境变量探针在形成 Auto 后从生产源码
撤回。

### 8.7 fresh solve 失败后的单次稳定性升级

Auto 复刻 HiGHS `HEkk.cpp:3029-3050` 的单调阈值升级，但触发条件更窄：一次
major INVERT 后 `ft_updates==0`，若 reconstruction 明确因 FTRAN/BTRAN 后向误差
失败，才把 backend 阈值从 0.1 提到 0.5，对**同一 basis**再 INVERT 一次并重做
reconstruction；每个 solver 实例最多升级一次且随后保持 0.5。其它 reconstruction
错误不得触发，原始 pivot threshold 环境探针删除。

成本模型：正常路径只多一个失败分支判断，额外成本为 0；命中路径增加一次
`R_0.5` 分解。`pilot87` 当前单次 rebuild 约 20--47ms，故相对 14.26s 基线应小于
1%，即总时长上限固定为 14.97s（+5% 留出机器波动）。固定预测：升级前 dual
9328 + primal 696 = 10024 pivot 与 0.1 路径相同；升级后案例必须 accurate，且
不得重新从头求解。`maros-r7/fit2d` 不应命中升级，pivot 数须与 §8.2 基线相同。
若 `pilot87` 当前 basis 即使 0.5 仍失败，先检查阈值是否传入第二次 factor build，
再检查定向 basis 与全程 0.5 路径的条件差异，先回写 mismatch 后再改算法。

首版 Auto mismatch：升级误接在 dual `major_rebuild`，而基线失败实际来自 primal
cleanup 的 `rebuild_primal_state`。它在 dual 过程中提前触发并永久保持 0.5，产生
9595 dual pivot、1051 次 safety rebuild，最终转为 `fresh primal row/column pivot
identity failed`，总时长 19.86s；预测的 10024 pivot/单次额外 INVERT 均不满足。
按实现忠实度检查确认不是阈值理论失败，而是触发边界错误。修正合同：仅
`solver.cpp:1481` 的 unperturbed primal cleanup 调用允许 `rebuild_primal_state`
在 fresh reconstruction 后向误差失败时升级；普通 primal fallback、primal Phase I
与 dual `major_rebuild` 一律禁止。这样触发前应严格复现 9328 dual + 696 primal
pivot，且只在原失败点增加一次强 pivot INVERT。

边界修正后的实测为 `reports/hypersparse_pilot87_strong_retry_r1.json`：Native
`14268.36ms`、HiGHS `6289.04ms`，Native accurate（`rel=1.26e-6`，归一化
primal violation `3.36e-15`）。dual 路径严格保持 9328 pivot，说明升级未污染
超稀疏 dual 主循环；总时长相对同机失败基线 `14256.25ms` 仅 +0.08%，符合
成本预测。但 primal cleanup 为 892 pivot 而非 696，数量预测偏离 28.2%。重查
实现忠实度后，前 696 pivot 与原路径相同；0.5 factor 的可靠 reduced-cost 重构
暴露了原 0.1 factor 未能认证的剩余对偶缺陷，故需再做 196 pivot 才通过最终
原始成本 audit。修订验收：不得要求失败点后 pivot 为零；要求 dual pivot 严格
相同、总时间 <=14.97s、最终 accuracy 通过，并以完整 Release suite 排除额外
cleanup 对其它案例的回归。该修订不需要再改实现或放宽任何数值门。

非触发复核 `reports/hypersparse_strong_retry_nontrigger_r1.json`：`fit2d`
Native `158.71ms`/120 dual pivot、HiGHS `149.48ms`；`maros-r7` Native
`4029.46ms`/3255 dual pivot、HiGHS `1390.77ms`，均 accurate，pivot 数与 §8.2
基线逐例相同。性能结论不能外推：修复后 `pilot87` 仍为 HiGHS 的 2.27 倍，
`maros-r7` 仍为 2.90 倍；本节只关闭超大稀疏案例的数值失败。

构建为 commit `a0322e38` + 本工作树、MSVC 19.44、Release，焦点命令为
`netlib_solver_benchmark --data-dir tests/data --cases pilot87 --solvers
native-dual-direct,highs-simplex --repeat 1 --time-limit 60 --max-iterations
100000`。完整准确性门
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`
为 19/19，包含 dual simplex、presolve、numerical stability 与 NETLIB
regression，总实耗 337.47s。

### 8.8 稠密 DSE 到 Devex 的算法级 A/B

HiGHS `HEkkControl.cpp:78-143` 在 DSE 辅助列密度相对 `row_ep/col_aq/row_ap`
密度代价过高时允许切换到 Devex；算法基础为 Harris (1973) 的 Devex 与
Goldfarb、Reid (1977) 的 steepest-edge 更新，工程分析见 Huangfu、Hall (2018)
§2-4。Native `compute_dse_weights` 的 DSE 分支每 pivot 额外做一次 auxiliary
FTRAN，Devex 分支只按 pivotal row 的 reference set 更新；故这是真正与支持集
扩散相关的算法切换，而非存储格式切换。

首轮不实现阈值，直接使用已有 `native-dual-devex` 对 `pilot87` 做全程 A/B。
§8.7 Auto DSE 基线为 9328 dual pivot、DSE 2.38s、Native 总时长 14.27s。
固定预测：Devex 的 DSE bucket 至少下降 60%，dual pivot 增幅不超过 25%，最终
accuracy 通过；只有总时长下降至少 10% 才值得继续设计动态 switch。若 pivot
增幅超过 25% 或总时间方向错误，先检查 Devex framework restart/退化路径，
不把 HiGHS 的阈值常数直接复制成 Native 默认。

正交 key `native-dual-devex-direct` 排除 presolve 后，`pilot87` Devex
`10308.13ms` 对 DSE `16310.16ms`（-36.8%），DSE bucket `2.17->0.29s`
（-86.6%），dual pivot `9328->9069`（-2.8%）；`maros-r7` 为
`1799.78ms/2377` 对 `4931.15ms/3255`（-63.5%）；但小而宽的 `fit2d`
为 `243.49ms/190` 对 `155.62ms/120`（+56.5%）。三者均 accurate。
因此全局 Devex 被否决，但“大 basis 避免 auxiliary FTRAN”假设仍成立。

第二阶段 cohort 固定为标准形行数约 2000 以上的
`80bau3b,dfl001,fit2p,maros-r7,pilot87`，三臂为 direct Devex、direct DSE、
HiGHS simplex，repeat 1、40s time limit。只有 Devex 至少赢 DSE 4/5、五例
几何平均至少改善 15%、全部 accuracy 通过且任一案例 pivot 增幅不超过 30%，
才允许考虑 `m` gate；否则维数不是充分代理，保持显式 benchmark 选项并研究
在线 solve work。该 cohort 预测在查看结果前固定。

cohort mismatch：Devex 对 `80bau3b` 为 `436.49ms` vs `631.95ms`，
`fit2p` 为 `2376.55ms` vs `2403.64ms`，`maros-r7` 为 `3679.69ms` vs
`5300.63ms`，`pilot87` 为 `13609.00ms` vs `16542.08ms`；四例均 accurate。
五例报告几何平均 `4608.59ms` vs `5563.69ms`（Devex -17.2%）。但
`dfl001` 两臂均在 40s 超时：Devex 24470 pivot，DSE 21409 pivot，无法满足
“全部 accuracy”门，因此不发布 `m` gate。该删失结果同时表明 Devex 每秒处理
约 14.3% 更多 pivot，不能把超时直接解释成算法回归。

按 mismatch 协议，`dfl001` 单例延长至 120s，仍用相同未 presolve 模型、同一
MSVC Release 二进制。固定判据：两臂都必须 accurate；Devex 总时间至少下降
10%，且 pivot 增幅不超过 30%，才把 cohort 的 accuracy 缺口视为时间盒假象。
任一臂仍超时或 Devex 变慢则停止维数 gate，不继续调阈值。

延长结果明确否决 gate：`dfl001` Devex `65206.23ms/36482 pivot`，DSE
`48355.94ms/24617 pivot`，HiGHS `10887.05ms`，三者均 accurate；Devex
慢 34.9%、pivot 多 48.2%。额外 DSE profile 为 `43642.28ms`，其中 DSE
`6.89s/43.63s=15.8%`、rowEP 58.64%、pivot row 44.85%。这个占比与
`pilot87/maros-r7` 的约 13--17% 同量级，却产生相反的 Devex 总效果，证明规模、
支持密度和当前 DSE 时间份额都无法预测后续 pivot 数。结论：Production 不增加
自动 DSE->Devex；保留 `native-dual-devex-direct` 作为正交研究 key。超大规模的
下一主线是 presolve（同一 `pilot87` Devex + HiGHS presolve 已到 8.55s，距
HiGHS 6.28s 为 1.36x）及 BFRT/PRICE 内核，而不是未经预测器保护的权重切换。

### 8.9 分区行式 PRICE 的动态稠密累加（实现前算法卡）

Hall & McKinnon (2005) §3--4 的核心不是把所有 PRICE 固定为某一种稀疏格式，
而是在中间向量的支撑扩散后停止支付超稀疏集合维护成本。Hall (2024)，
*High performance computational techniques for the simplex method*（CO@Work
讲义，simplex computational kernels 部分）再次把 hyper-sparse 与 standard
kernel 的动态选择列为高性能 revised simplex 的基本机制。当前 HiGHS 实现
`highs/util/HighsSparseMatrix.cpp:1458-1545` 的
`priceByRowWithSwitch` 也在输出支撑超过 `kHyperPriceDensity` 后，从维护索引的
row PRICE 切到同一 row-wise 矩阵上的 dense-result 累加；其分区行矩阵只扫描
非基前缀。这与 §8.3 已否决的 CSC PRICE 不同：不会扫描 `nnz(A)`，仍只访问
`rho` 支撑命中的分区行。

Native 当前分区 PRICE 对命中行的每个非零都读 `stamp[col]` 和
`value[col]`。令已处理矩阵非零数为 `q`、列数为 `n`、切换后剩余非零数为
`q_r`。保持超稀疏到底的附加流量约为 `4q_r` bytes 的 stamp 读取；动态稠密化
先用 `O(n)` 写把本 epoch 尚未出现的 value 标为 NaN sentinel，随后只读写 value，
并继续按原 row/slot 顺序累加。切换条件采用 HiGHS 的 10% 输出密度：
`|support| > 0.1n`。sentinel 只区分“本 pivot 未触达”和“已触达但精确抵消为
0”，所以不会产生重复 support；首次触达仍执行 `0 + term`，每列浮点加法顺序、
首次触达顺序和最终 packed 顺序均不变，故这是 Class-P 纯性能变换。

固定预测与验收（查看 live A/B 前）：`pilot87` 的 pivot-row 平均密度 59.46%、
PRICE 2.82s/16.32s，预测 PRICE 降 10--25%，总时间降 2--5%；`maros-r7`
密度 18.62%，预测 PRICE 降 5--15%、总时间 0--2%；`fit2d` 密度 96.99% 但
`m=25`，`O(n)` sentinel 初始化会限制收益，预测 PRICE 降 0--15%、总时间不劣
超过 3%。三例 dual pivot 数必须与 §8.2/§8.7 逐例相同，原成功例与修复后的
`pilot87` 均须 accurate。命令固定为 Release、commit `a0322e38` + 本工作树：
`netlib_solver_benchmark --data-dir tests/data --cases fit2d,maros-r7,pilot87
--solvers native-dual-direct,highs-simplex --repeat 1 --time-limit 60
--max-iterations 100000`，并运行完整 Release ctest。若 PRICE/总时长方向错误，
或收益相对预测偏差超过 50%，按实现保真度 -> 缓存/带宽成本模型 -> 支撑扩散假设
顺序排查并先回写本节 mismatch；不通过时撤回动态切换。

实测 mismatch（`reports/hypersparse_dynamic_row_price_r1.json/.log`）：三例
dual pivot 分别保持 120/3255/9328，全部 accurate，说明行/列求和与 pivot 路径
合同满足；但 `pilot87` PRICE `2.82->4.18s`（+48.2%），总时长相对 §8.7
`14.27->17.84s`（+25.1%），方向错误。`fit2d` PRICE 约 0.03s、总时长
170.97ms；`maros-r7` PRICE 0.37s、总时长 4282.74ms，但单例 wall 波动不能
推翻焦点案例的负收益。

按协议排查：实现保真度由完全相同的 pivot 数和结果确认；错误在机器/成本模型。
NaN sentinel 为维持“精确抵消为零后也不重复 support”的逐位合同，每次切换须写
`O(n)` 个 value lane，随后每个剩余矩阵非零还执行 `isnan` 分类。原 stamp 仅增加
一个连续 4-byte 读取，且 `pilot87` 的 stamp/value 工作集能驻留缓存；所预测的
stamp 带宽节省不足以偿还 sentinel 写和浮点分类。HiGHS 的 `value0==0` 首触达
判据允许中间精确抵消后再次插入 index，不能在 Native 的 bit-identical packed
support 合同下直接复制。结论：撤回动态切换；PRICE 的下一安全方向必须把现有
epoch-stamped accumulator 与 factor/HVector resident workspace 合并，消除完整的
export/re-scatter 或状态流，而不是用另一套每 pivot 初始化状态替代 stamp。

### 8.10 `pilot87` 原生 presolve 的增量 nnz 成本模型（实现前算法卡）

把超大规模路径转到 presolved LP 前，原生 presolve 自身必须满足线性或局部增量
复杂度。`pilot87`（2030 行、4883 列、73152 非零）在默认 2s 及放宽 20s
时间盒下均返回 `time_box`，报告分别为
`reports/pilot87_native_presolve_probe_r1` 和
`pilot87_native_presolve_20s_probe_r1`。实现检查发现 P2 每次调用
`substitute_equality_column` 都以 `MutableAdjacency::nnz()` 遍历全部行 map，
再评估一个局部代入。令当前非零数为 `z`、尝试代入数为 `s`，该统计引入
`Theta(s(z+m))` 的节点/容器遍历；而一次 `set(i,j,v)` 已经知道条目由零到非零、
非零到零或只改值，可以用一个计数器把统计降为 `O(1)`，总成本回到局部
Markowitz/fill-in 检查的工作量。该变换不改变 map、遍历顺序、浮点值、规则选择或
postsolve 栈，是 Class-P 性能变换；依据 A&A (1995) §2.4 的局部代入模型和
Suhl & Szymanski (1994) 的稀疏消元复杂度原则。

固定预测：增量 nnz 把 `pilot87` presolve 从 `>20s` 降到默认 2s 时间盒内并
返回 reduced model；若仍超时，verbose 日志必须给出已应用的各规则计数，用来
区分 `nnz()` 成本与“每次成功后从 0 重扫行列”的候选调度成本。成功 reduced
solve 必须通过原模型 accuracy；P1/P2 规则单测结果完全不变。若仍超时，则按
实现保真度（计数是否随 set 精确变化）-> 成本模型（全扫调度）-> 结构假设顺序
排查并先回写 mismatch，再实现候选工作队列。

增量计数实测 mismatch（MSVC 19.44、Release、commit `a0322e38` + 本工作树，
`reports/pilot87_native_presolve_incremental_nnz_r1.{json,log}`）：
`test_lp_presolve` 保持 23 cases / 954 assertions 全通过，说明 `set()` 的零/非零
转移计数与既有规则语义相符；但 `pilot87` 在 `2006.8ms` 仍返回 `time_box`，
未产出 reduced model，故“降到 2s 内”的预测失败。超时点为 73 轮，累计
`implied_bounds=90783`，而结构代入只有 `doubleton_eq=6`、
`singleton_cols=11`、`free_col_subst=45`；其它结构规则合计 300 次。实现检查
确认 `nnz()` 已为单字段读取，剩余成本来自外层用 `counts.total()` 判定继续：
每个超过 `1e-8` 相对门的合法 bound tightening 都触发下一次完整
`row_sweep + O(nnz) implied_bound_sweep + col_sweep + P2`，即使这一轮没有任何
行、列或代入结构改变。故首要 mismatch 是成本模型遗漏了 P3 自身的数值不动点，
尚不能把主要责任归给 P2 候选扫描；先修正外层调度，再决定是否需要工作队列。

### 8.11 P3 结构轮调度（实现前算法卡）

A&A (1995) §3 与 Achterberg et al. (2020) §6.4 给出每一次 row-activity
projection 的合法 bound tightening，但 presolve 正确性不要求把这些 tightening
迭代到浮点数值不动点：任意前缀都只缩去由原约束证明不可行的 box 区域，未继续
传播只会漏掉后续可选 reduction。PaPILO 的公开调度同样把“only bound changes”
作为独立可终止轮次类别（`third_party/papilo/src/papilo/core/PresolveOptions.hpp`，
`max_consecutive_rounds_of_only_bound_changes`），说明 bound-only 进展不应与结构
消元无条件共用一个不动点合同。

本轮只把外层继续条件从 `counts.total()` 改为结构计数：empty/redundant/singleton
row、empty/fixed column 及 P2 substitution 任一增加才启动下一轮。P3 仍在每个
结构轮中按当前确定性行/列顺序完整执行一次，且 tightening 仍计入 telemetry；
因此结构变化产生的新邻接和新 box 至少会再传播一轮，而纯 P3 chaining 不再独自
触发 `O(nnz)` 全局重扫。P2 内部仍在一次调用中按 singleton equality column ->
doubleton equality -> general free/implied-free substitution 的原优先级跑到局部
不动点，postsolve 顺序不变。

成本预测固定如下：当前 `2006.8ms/73 = 27.5ms` 每轮；结构规则总计 362 次且
批量 row/column sweep 与单次 P2 sweep 各自消化同类候选，预测 `pilot87` 不超过
8 个外层轮、presolve 不超过 500ms，`implied_bounds < 20000`，并在默认 2s
时间盒内返回 reduced model。`test_lp_presolve` 必须保持 23/954；reduced solve
必须通过原模型 objective 与 primal residual accuracy 门。由于这是算法调度改变，
焦点成功后运行完整 Release suite。若仍超时或不产出 reduced model，先检查结构
计数分类和 P2 内循环是否确实收敛，再据 verbose 轮数决定实现增量候选队列；若
accuracy 失败，按 bound projection -> 结构轮停止假设 -> postsolve 顺序排查，
不得放宽容差或 publication audit。

实测验收（`reports/pilot87_native_presolve_structural_rounds_r1.{json,log}`）：
presolve 为 `162.7ms`、4 轮，产出 `2030x4883/73152 ->
1934x4560/70351` 的 reduced model；随后 Native IPM 在总 `2698.61ms` 内求解并
通过原模型 accuracy（objective relative error `1.25e-6`、normalized primal
violation `3.01e-14`）。轮数与 presolve 时间分别优于 `<=8`、`<=500ms` 预测，
`test_lp_presolve` 保持 23/954。`implied_bounds=21047` 比 `<20000` 点预测高
5.2%，但远低于前一轮 90783，且不构成约 50% 的模型失配；原因是首个结构轮的
Gauss-Seidel 行序在同一 sweep 内即可产生超过按 73 轮均值外推的 tightening，
不影响复杂度或正确性结论。保留结构轮调度。

### 8.12 Native dual presolve 接线（实现前算法卡）

Native dual 当前只有 HiGHS presolve bridge；Native presolve 已在 IPM 路径验证
`native reduce -> reduced solve without recursive presolve -> primal postsolve ->
original-model residual audit -> direct fallback` 的 fail-closed 合同（本文件 §3.1、
§6）。同一合同与 A&A (1995) §2 的 primal recovery 直接适用于 revised dual
simplex：presolve 只改变冷启动 LP，basis hint 仍严格留在原空间，不参与 reduced
solve。任何 reduced kernel、postsolve 维数或 original audit 失败都回到现有
HiGHS presolve（若启用）再回到 direct solve，不发布未经原模型审计的 primal。

首轮新增独立 `SimplexOptions::use_native_presolve` 和 benchmark key
`native-dual-devex-native-presolve`，保留现有 `native-dual-devex` 的 HiGHS bridge
作为同批对照。Native-only arm 关闭 HiGHS presolve，避免把 fallback 成功误记为
Native 收益；生产选项默认 false，因而所有既有调用与 pivot path 不变。reduced
simplex 的 feasibility/optimality/publication tolerance 乘以
`lp_presolve_publication_tol_scale`，其缩放推导见 §6；恢复后以
`escalation_residual_tol` 对原模型复核并用原始 `c.dot(x)` 计算目标。

成本模型：Native presolve 在 `pilot87` 为 162.7ms，并把列数降 6.6%、nnz 降
3.8%；未经 presolve 的 Devex 为 `10308.13ms/9069 pivot`，故仅按 nnz 比例保守
预测 Native-only 总时长下降 2--8%（`9.48--10.10s`），pivot 数不超过 direct
的 110%，但不预期首轮胜过 HiGHS-presolved `8553.42ms/8186 pivot`。验收要求
Native-only 返回 accurate、`presolve_used=1` 且尺寸与本节一致；若时长或 pivot
方向错误，先查 reduced tolerance 与模型结构，再决定增强 P3/P4，不把接线失败
归因于 factor 内核。接线属算法路径新增，焦点通过后运行完整 Release suite。

首轮接线 mismatch（`reports/pilot87_native_dual_native_presolve_r1.{json,log}`）：
Native-only 最终 accurate，但为 `14204.32ms/9069 pivot` 且
`presolve_used=3`，没有满足“采用 Native reduced model”和 `9.48--10.10s`
预测。同批 HiGHS-presolved Devex 为 `5869.97ms/8186 pivot`，HiGHS simplex
为 `3547.05ms`（机器频率与 §8.8 批次不同，结论只取同批相对值）。Native-only
最终成功臂的 `native_kernel=6823.26ms`，比总时长少 7.38s；结合 used=3 可知
实际路径是约 7.2s reduced 尝试失败后又执行 direct solve，而非 presolve 让每个
pivot 变慢。fail-closed 接线忠实地未发布失败结果，错误在成本模型假定 reduced
solve 会成功。下一步先用只读 verbose 记录 reduced status、postsolve 维数与原模型
audit 结果，区分 kernel failure 和 recovery failure；在完成该诊断前不增强 P3/P4，
也不改变默认路由。

只读诊断 `reports/pilot87_native_dual_native_presolve_diag_r1.{json,log}` 给出：
reduced simplex 本身为 `Optimal`、10876 iterations，postsolve 维数也正确
（4560 -> 4883），但 original audit 为 false；随后执行的 direct arm 才以 9069
pivot 成功。因此 failure 明确位于 primal recovery 的数值精度或记录语义，而非
reduced kernel 收敛。下一诊断固定为分别报告 inequality/equality/bound 最大违反
及索引；若只有已删除行失败，检查对应 postsolve rule，若 reduced 中保留的行也
失败，检查 reduced publication tolerance。诊断只读，不改变接受门。

分项残差诊断
`reports/pilot87_native_dual_native_presolve_residual_diag_r1.{json,log}`：
最大 inequality violation `1.315e-10`、equality violation `9.095e-12`，均远
低于 audit 门；唯一失败是原列 4823 的 bound violation `7.5141`（全局归一化
`1.156e-3`）。所以 reduced primal 与所有行代入在数值上成立，问题是某条 P2
记录恢复出的显式有界变量不满足原 box。下一步只追踪列 4823 对应的 postsolve
记录及其原界；若为 general implied-free substitution，则按附录 A.2 检查“隐含界
必须由其它行给出、不得用正在删除的 equality 自证”的前提，而不是放宽 bound
audit。

### 8.13 General implied-free 的来源闭包（实现前算法卡）

列 4823 的记录进一步给出 `type=free`、原界 `[0,+inf)`、恢复值
`-7.5140946`。这与附录 A.2 的 HiGHS 前提不符：`isImpliedFree(col)` 使用的
两侧隐含界必须由**其它行**维护，并排除将被删除的 substitution row。Native
当前 `implied_free_in_equality` 只对该 equality 做局部 interval projection，且
其它列界也可能来自随后被删除的 equality；多次代入可形成循环证明，最终一起
删除所有负责维持 box 的行/列。单次 interval 公式没有错，错误是缺少 bound
source/provenance 闭包，不能把局部充分条件跨 substitution 链外推。

在实现 source-row generation 和删除失效传播前，general equality substitution
只接受 `lb=-inf && ub=+inf` 的真正 free column。该条件使消去变量没有 primal
box 可丢，A&A (1995) §2.4 的代入保持等式与目标恒等；有界 singleton equality
仍由独立 projection 规则把 box 转成 ranged row，doubleton 仍把 subst box 交叉
收紧到 stay column，因此不受影响。`tests/test_lp_presolve.cpp` 已分别覆盖真正 free
正例、有界 singleton projection 正例和 bounded-not-implied-free 负例。

固定预测：`pilot87` 的 `free_col_subst` 从 45 降为 0，Native reduced model 约比
§8.11 多 45 行/列；reduced dual 必须成功 postsolve、`presolve_used=1`、原列
4823 保持活动且 original bound audit 通过。因前一 reduced 尝试约 7.2s、direct
成功约 6.6s，保守预测总时长 `6.5--8.5s`，不得出现二次 direct；pivot 数不超过
前一 reduced 的 10876。若仍有 bound violation，按 record 类型继续排查 singleton
projection 与 doubleton bound transfer；若仅性能超区间而 accuracy 通过，则记录
新结构的 pivot path，不恢复不安全的 implied-free 条件。

保守 free 条件实测（`reports/pilot87_native_dual_native_presolve_safe_free_r1`）：
`test_lp_presolve` 23/954 与 `test_dual_simplex` 76/23029 全通过，且
`free_col_subst=0`，证明不安全记录已移除；但 reduced dual 在 2400 iterations
以 `bound-side reconstruction failed` 停止（initial residual `6.299e-4`、limit
`5.85e-5`），随后 direct fallback 成功，故最终 `7622.99ms` accurate 但
`presolve_used=3`。总时长落在预测区间，然而“不得二次 direct”与 reduced 成功
预测失败。新模型为 `1968x4594/70417`、12 轮、`implied_bounds=39537`；失败从
postsolve bound violation 转移为 reduced kernel 的 bound-side reconstruction，
说明 §8.13 修复了正确性缺陷，但 P3 强化后的 box 与 Native simplex 数值路径仍
不兼容。

### 8.14 P3-off 正交探针（实现前算法卡）

P3 是当前唯一批量改写变量 box 的阶段，`pilot87` 有 39537 次 tightening；P1/P2
主要删除结构，singleton row 只贡献 8 次 bound projection。Native dual 的失败
消息又明确来自 canonical primal 的 bound-side reconstruction，而非 LU、PRICE
或 postsolve。故先通过现有 `LpPresolveConfig::propagate_bounds` 做正交 A/B，不改
任何 P3 公式或默认值：新增 `MIPSOLVERS_NATIVE_PRESOLVE_PROPAGATE_BOUNDS=0`
环境覆盖，仅用于完整 solve 探针。

固定预测：P3-off 的 `implied_bounds=0`、presolve <300ms；若病态 box 是原因，
reduced dual 必须成功、`presolve_used=1`、postsolve original audit 通过，总时长
在 `6.0--8.5s`。结构尺寸允许因缺少 bound-driven redundant/singleton reductions
而变大，pivot 不设改善门。若仍在相同 bound-side reconstruction 阶段失败，则
P3 不是主因，按 P2 matrix transformation -> standard-form scaling 排查；若成功，
再以小型/NETLIB cohort 比较 P3-on/off，不直接把 probe 设为全局默认。

P3-off mismatch（`reports/pilot87_native_dual_native_presolve_p3_off_r1`）：
`implied_bounds=0`、presolve `256.8ms`，但 reduced dual 仍失败；失败从 P3-on
的 2400 iterations bound reconstruction 移到 7000 iterations fresh-factor BTRAN
后向误差（initial residual `2.383e-11`、limit `1.542e-11`，refinement 后恶化到
`3.505e-9`）。最终 direct fallback accurate，总时长 `11811.45ms`、
`presolve_used=3`。因此“大量 tightened box 是主因”的预测被否决，且关闭 P3
反而浪费更多 reduced pivot；默认 P3 保持开启。

### 8.15 P1-only 正交探针（实现前算法卡）

P3-off 模型仍应用 22 次 doubleton、24 次 singleton-column substitution；这些
P2 规则改变 equality/row-side 结构和标准型列，而 P1 主要做固定列代入及零填充。
按 §8.14 的既定次序，新增
`MIPSOLVERS_NATIVE_PRESOLVE_SUBSTITUTIONS=0` 环境覆盖，并与 P3-off 同时使用，
形成 P1-only solve；默认配置不变。

固定预测：P1-only 的 `doubleton_eq=singleton_cols=free_col_subst=0`，presolve
<300ms；若 P2 matrix transformation 是主因，reduced dual 必须成功、
`presolve_used=1`、原模型 audit 通过，总时长 `6.0--8.5s`。若仍失败，则排除 P2，
下一层只剩固定列代入/compact 后的标准型缩放；在写回新 mismatch 前不修改这些
公式，也不改变 factor threshold。

P1-only 验收（`reports/pilot87_native_dual_native_presolve_p1_only_r1`）：
presolve `3.8ms`，4 轮，模型 `2030x4883/73152 -> 2010x4658/70639`；reduced
dual `Optimal`、9567 total iterations / 8749 dual pivots，postsolve original
audit 通过，最终 `6122.55ms` accurate 且 `presolve_used=1`。全部固定预测满足。
同机 §8.12 的 HiGHS-presolved Native Devex 为 `5869.97ms`，故 P1-only Native
presolve 只慢 4.3%；相对同批 direct 成功 kernel 约 `6.6s/9069 pivot`，P1-only
同时减少 pivot 与每 pivot 成本。这证明 P2/P3 的数值交互而非 P1 compact/fixed
substitution 是 reduced 失败源。

### 8.16 Native dual 的 P1-only 发布策略（实现前算法卡）

IPM 继续使用完整 P1/P2/P3，因为 §8.11 已在 `pilot87` 验证其 reduced solve 与
postsolve；Native dual 则采用 solver-specific capability envelope：
`use_native_presolve` 默认构造 `substitutions=false, propagate_bounds=false`，随后
再应用环境覆盖，保留显式 P2/P3 研究能力。该策略不是声称 P2/P3 数学规则普遍
无效，而是遵守已测的 kernel compatibility：P2/P3 reduced model 在 Native dual
分别触发 postsolve box 缺陷或数值 reconstruction 失败，P1-only 已通过原模型
审计。A&A (1995) §2.1--2.3 的 P1 规则和 reverse fixed-column recovery 保持完整。

固定验收：不设置实验环境变量时，benchmark key
`native-dual-devex-native-presolve` 必须复现 P1-only 尺寸、`presolve_used=1` 和
accuracy；repeat-3 中位数相对单次 `6122.55ms` 波动不超过 20%，且与同批
HiGHS-presolved Native Devex 的比值不超过 1.15。随后运行完整 Release suite；
任何回归先检查 solver-specific config 是否泄漏到 IPM/其它 SimplexOptions，禁止
以放宽原模型 audit 解决。

P1-only 发布策略验收（MSVC 19.44、Release、commit `a0322e38` + 本工作树，
`reports/pilot87_native_dual_p1_policy_r3.{json,log}`）：Native presolve 3 次均为
约 4ms、固定缩到 `2010x4658/70639`，reduced dual 每次 8749 pivot，3/3
accurate 且 `presolve_used=1`；总时长 `6134.72/6221.55/6063.71ms`，中位
`6134.72ms`，相对 §8.15 单次只波动 0.2%。同批 HiGHS-presolved Native Devex
中位 `5550.89ms/8186 pivot`，Native/HiGHS-presolve 比值 `1.105x`，满足
`<=1.15` 门。纯 HiGHS simplex 中位 `3309.73ms`，所以 Native P1-only 仍为
`1.854x`，不得表述为超越 HiGHS；本轮成果是把 Native presolve 从 >20s 超时及
错误回退推进到稳定可发布，并把与 HiGHS-presolved Native 的差距压到 10.5%。
完整 MSVC 19.44 Release 构建随后执行
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`，
19/19 全通过（含 dual simplex、LP presolve、numerical stability 与 NETLIB
regression），总实耗 237.87s。

### 8.17 Factor-resident pivotal BTRAN view（实现前性能卡）

模型与算法边界：dual revised simplex 的 pivotal row 是
`row_ep = B^{-T}e_p`，随后 PRICE 计算 `row_ep^T A`，DSE 用
Goldfarb--Reid recurrence 更新权重，BFRT 用同一行的无穷范数及坐标值构造
稳定性界。HFactor 已把该 BTRAN 结果和 Forrest--Tomlin 更新中间量保存在
`update_vec_ep`；当前 wrapper 随后仍逐项导出为 Native `IndexedVector`，而 PRICE、
BFRT 和辅助 DSE FTRAN 又对它做 lookup、scatter 或 RHS 重装填。依据
Forrest & Tomlin (1972) 的 product-form update 数据依赖、Hall & McKinnon
(2005) §2--3 的 hyper-sparse revised-simplex 数据流，以及 Huangfu & Hall
(2018) §2.2--2.3 的 HiGHS sparse-vector/PRICE 组织，只读消费者可直接遍历
factor-resident support；只要保持 HVector support 顺序和每行乘加顺序，结果与
先导出再遍历逐位相同。

实现限定为 generation-bound opaque view：Native 上层只见
`valid/count/index/value/value_at`，不见 `HVector`；view 绑定 factor serial，
INVERT 或 `update_captured` 后必须失效。首批迁移 pivotal PRICE、Devex/DSE 的当前
行权重、Phase-II BFRT 的 `row_ep_max_abs` 与只读坐标查询；普通 BTRAN、终局证书
和测试所需的 packed export 保持原 API。DSE auxiliary FTRAN 直接从 resident EP
装填 backend scratch，并仍只提取 captured AQ support。公式、容差、候选顺序、
Harris/Goldfarb--Reid 更新和 factor update 均不改变，属于 pure performance
改动。

成本模型：`pilot87` 既有 profiling 中 wrapper export 约 `0.515s`，其中 BTRAN
pack 约 `0.142s`、FTRAN conversion/reorder 约 `0.373s`。本阶段只覆盖 pivotal
BTRAN，预计 BTRAN export bucket 降 80--95%（剩余计时仅 view 验证/有限性扫描），
同时省掉 PRICE 前后的 lookup/scatter，故对 Native-P1-presolved 的约 `6.13s`
总时长固定预测下降 1.5--3.0%（repeat-3 中位 `5.95--6.04s`，不把机器波动计作
算法收益）；direct 路径方向相同。假设 `update_vec_ep.index` 是 BTRAN 的已压缩
非零 support，且 pivotal proposal 到 factor update 间没有 factor mutation。

验证协议固定如下：先用单元测试覆盖 view 的 external-row 坐标、serial 失效和
packed/resident 逐位等价；然后对 direct 与 Native-P1-presolved `pilot87` 各作
repeat-3 paired benchmark，记录 commit、MSVC Release flags、命令、总时长、
PRICE/BFRT/DSE/export buckets。两条路径的每次 pivot/iteration 数必须与改动前
完全相同，accuracy 全通过；BTRAN export bucket 至少下降 80%，P1-presolved
中位总时长须落入上述区间。最后运行完整 Release suite。若 pivot path 分叉，先
检查 internal/external row 语义和 support 中显式零；若收益方向错误或相对预测
偏差超过约 50%，先把 mismatch 写回本节，按“遗漏导出消费者 -> cache/计时噪声
-> support 密度假设 -> 成本模型错误”顺序复查，在此之前不迁移 FTRAN export 或
改变数值算法。

首轮 mismatch（MSVC 19.44 Release、commit `a0322e38` + 工作树，
`reports/pilot87_factor_resident_{p1_probe_r1,p1_direct_r3}.json`）：backend
等价/失效测试 26 assertions、PRICE 测试 41 assertions、完整 dual-simplex
77 cases / 23056 assertions 均通过；profile 探针保持 P1 尺寸、9567 iterations、
8749 dual pivots 和 accuracy，BTRAN export 从约 `0.142s` 降至 `0.0134s`
（-90.6%，满足 bucket 门）。但关闭 profile 的 P1 repeat-3 为
`8004.29/7919.37/7939.61ms`，中位 `7939.61ms`，不仅未进预测
`5.95--6.04s`，还相对 §8.16 同机历史中位 `6134.72ms` 反向 29.4%；paired
direct 为 `8528.73/8743.05/8309.22ms`、9069 pivots，说明本批机器整体也慢，
但不能解释 P1 绝对预测的错误，须停止扩展迁移。

按既定排查顺序，实现不忠实项优先：首版 `ResidentVectorView::index/value/at`
每次都 out-of-line 调 `valid()`，而 `Leaving::row_ep_*` 又在每个坐标重复选择
resident/packed；`pilot87` 平均 row EP 1174 nnz、8749 pivots，故 serial/pimpl
检查被放大到千万量级，违背成本模型中“每 pivot 一次 generation check”的假设。
修订实现固定为在 pivotal BTRAN 完成后设置不可变 resident selector，PRICE 前只做
一次 `valid()`；该 pivot 内的 `index/value/at` 是 unchecked read（factor mutation
仍只在所有消费者结束后的 commit），`finite/max/norm` 各自只校验一次。预测该修复
应消除首版新增开销并至少不慢于 paired direct 的每 pivot 成本；pivot/iteration
仍须逐次相同。若修复后 P1 相对新的 packed-control 仍不改善，则下一步才检查
out-of-line call 本身和当前机器频率，不继续迁移 FTRAN。诊断对照由
`MIPSOLVERS_DS_PACKED_ROW_EP=1` 恢复旧 `indexed_btran(..., capture=true)`；仅用于
同二进制相邻 A/B，默认 resident 路径及所有数值参数不变。

一次校验修复与 paired A/B 验收（同编译器/build/commit+工作树，
`reports/pilot87_{resident_row_ep,packed_row_ep_control}_r3.json`）：resident 为
`7192.45/7128.41/7169.11ms`，中位 `7169.11ms`；packed control 为
`7465.91/7478.52/7346.77ms`，中位 `7465.91ms`。两组均固定 9567 iterations、
8749 dual pivots、3/3 accurate，故 pivot-path 与数值门满足；resident 相对同二进制
control 快 3.98%。§8.17 的绝对 `5.95--6.04s` 门未满足，因为本批 packed control
自身已达 7.47s、paired direct 也达 8.53s，中间构建/测试期间机器频率状态与
§8.16 不同，绝对历史对照的恒频假设失效。

复现实验基线为 commit `a0322e38773b38be91c9b13d8e18c306b948f5e0`
加本节工作树，MSVC 19.44、Ninja Multi-Config Release，编译标志
`/O2 /Ob2 /DNDEBUG`。resident 命令为
`tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases pilot87 --solvers native-dual-devex-native-presolve --repeat 3 --time-limit 60 --max-iterations 100000 --json reports/pilot87_resident_row_ep_r3.json`；
packed control 在同一命令前设置 `MIPSOLVERS_DS_PACKED_ROW_EP=1`，并把输出改为
`reports/pilot87_packed_row_ep_control_r3.json`。两组相邻执行，除该诊断环境变量外
参数和二进制相同。

相对收益又高于 1.5--3.0% 预测上界（以 2.25% 中点计偏差约 77%），需修订成本
模型：`0.142s` backend BTRAN export 只计时 pack loop，未覆盖每 pivot 新建
`Leaving::IndexedVector` 的 capacity 分配、后续 lazy lookup table 和 DSE RHS 再装填；
resident 同时消除这些未归入 export bucket 的流量，因此实测 3.98% 合理。方向、
pivot path 和 bucket 门均正确，保留 resident 默认；但在固定频率/独占机器复测前
不声称绝对时长达到 5.95--6.04s，也不据此继续迁移 FTRAN export。

加入 packed-control 诊断开关后的最终当前二进制回归命令为
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`：
19/19 全通过，总实耗 3.51s。该轮只作为正确性门；性能结论仍仅取上述同二进制、
相邻执行的 repeat-3 A/B 数据。

### 8.18 超稀疏 root-to-node resident 数据流（实现前算法卡）

目标不是把 P2/P3 约化后的 basis 强行解释成原模型 basis，而是按所有权和坐标空间
把冷启动与热节点分开。冷启动 portfolio 的 Native IPM 臂可使用已经通过 primal
postsolve/original audit 的完整 P1/P2/P3；Native dual 继续使用 §8.16 的 P1
capability envelope，直至 P2/P3 reduced standard form 的 reconstruction 兼容性
通过。进入 B&C 后不再做结构消元：节点只在固定 `StandardFormLP` 上更新 bounds/RHS，
由 worker-local dispatcher 持有 mutable factor。只有 `structure_id`、矩阵对象、
basis indices 和 factor update generation 全部匹配时，下一热节点才可复用 factor；
跨 worker、兄弟节点已改变 factor、加行或 basis 投影一律重新 INVERT。

节点合同依据 Maros (2003) §9 的 revised-simplex reoptimization：只改 `b/l/u`
不会改变 `B`，所以相同 basis 的 LU/Forrest--Tomlin 表示仍是同一个线性算子；只需
用新 bounds/RHS 重建 primal/basic state 和 reduced-cost audit。dispatcher 而非队列
节点拥有 mutable factor，符合并行 B&B 的 worker-local solver state 模型；队列只
携带 immutable basis/status。该路径不改变 pivot 选择、容差或求和顺序，属于
Class-P。固定预测：构造同一 structure、同一 basis 的连续两个 warm solve，第二次
初始 reinversion 从 1 降为 0，objective/primal/basis 与强制冷重建逐位一致；basis
或 structure 不匹配时必须仍为 1。实际 B&C 先只报告 reuse hit/miss，不在看到命中率
前预测整树加速。

pivot 内的数据流沿用 Hall--McKinnon (2005) §2--4 和 Huangfu--Hall (2018)
§2--4：`row_ep=B^{-T}e_p`、`col_aq=B^{-1}a_q` 的 HVector support 是超稀疏
TRAN 的事实来源，`row_ap=row_ep^T A_N` 则属于 partitioned row PRICE 的输出。
因此 EP/AQ 应由 factor-resident generation-bound view 持有，而 row AP 应由
solver-state 的 epoch-stamped accumulator 持有；把 row AP 塞进 factor 会混淆矩阵
分区和 factor 所有权。PRICE、BFRT、DSE、basic-value transaction 和 edge-weight
update 在 factor update 前直接迭代 resident support；只有 terminal certificate、
外部 basis API 或诊断对照物化 packed 向量。保持 HVector support 顺序及每个 consumer
原有算术顺序，故默认路径必须与 packed control 有相同 pivot path。

§8.17 profile 中总 FTRAN conversion/reorder 约 `0.373s`，它包含 pivotal AQ、
BFRT RHS 和 DSE auxiliary FTRAN；本轮只移除每 pivot 一次的 pivotal AQ export，
保守预测该 export bucket 下降至少 50%，`pilot87` P1-presolved Devex 总时间相对
同二进制 `MIPSOLVERS_DS_PACKED_COL_AQ=1` 改善 2--5%。两臂必须保持 9567
iterations、8749 dual pivots、3/3 accurate；resident/packed support、坐标、范数和
factor update 后失效由单测逐项对照。若收益方向错误或相对预测偏差超过 50%，按
重复 validity 检查 -> internal/external row permutation -> 未计入的 BFRT/DSE export
-> cache/机器频率顺序排查并先回写本节。

P2/P3 接入 Auto IPM 是算法路由改变，不与上述 Class-P 验收混算。候选 cohort 固定为
`greenbea,maros-r7,dfl001,pilot87` 加一组小型 NETLIB 对照；比较 Auto 的 IPM 臂
`presolve=false/true`，要求所有成功结果通过原模型 accuracy、长尾四例几何平均不劣、
小型对照总退化不超过 5%，且 loser cancellation 后不得留下超过 presolve time-box
的后台工作。只有这些门满足才把 Auto IPM 默认切到完整 Native P1/P2/P3；否则保留
显式 Native-IPM presolve 路径，并根据 mismatch 先补 cancellation/增量候选队列。

首轮 resident AQ mismatch（MSVC 19.44 Release、commit `a0322e38` + 工作树）：
resident/packed FTRAN 坐标、support、norm 和 factor-update 失效的 focused test 通过，
但完整 `test_dual_simplex` 的 `BFRT warm sequence agrees with HiGHS` 在第一个 warm
solve 后发现 exported DSE weights 多行仍为 1，而当前 factor 的
`||e_i^T B^-1||_2^2` 为 2--4；79 cases 中 78 通过、该用例 46 个权重断言失败。
同一二进制设置 `MIPSOLVERS_DS_PACKED_COL_AQ=1` 后该用例 234 assertions 全通过，
因此不是 DSE recurrence、初始 cache 或 basis factor 本身，而是 resident AQ
consumer 的实现不忠实。虽然 pivot/objective/残差仍正确，这已违反 §8.18 的完整
accuracy gate，必须在 benchmark 前修复。固定排查顺序为：PivotalColumn support
selector 是否错误选择 packed 空向量 -> resident external-row 映射 -> 显式零压缩
是否遗漏 recurrence 所需坐标 -> edge-weight transaction 是否在 factor mutation 后
读取失效 view；修复验收为该 warm 用例 resident/packed 均 234 assertions 通过，再跑
完整 dual-simplex，之后才恢复 `pilot87` A/B。

根因与修复：DSE recurrence 在 factor update 前正确消费 resident AQ，但 edge-weight
值在 factor update 成功后才事务提交；原提交循环又从已经 generation-invalid 的 AQ
view 取 row support，故 `count()==0`，只写回 pivotal row。修复不延长 view 生命周期，
也不提前修改 solver state，而是在 `EdgeWeightUpdate` 中只保存受影响 row indices 和
计算后的 weights；AQ values 仍不导出。该所有权符合本节“factor 持有 EP/AQ、solver
epoch workspace 持有事务”的划分。修复后 warm BFRT resident 路径 234 assertions、
Native persistent tests 22 assertions、完整 dual-simplex 79 cases / 23094 assertions
全通过；packed control 同样通过。

`pilot87` paired A/B（MSVC 19.44 Release、commit `a0322e38` + 工作树，命令与本节
固定协议相同）保持每次 9567 iterations（8749 dual + 818 cleanup）、8749 dual
pivots、3/3 accurate。resident 为 `6240.42/6199.77/6190.69ms`，中位
`6199.77ms`；packed control 为 `6157.97/6241.17/6320.90ms`，中位
`6241.17ms`，resident 快 `0.66%`。profile 相邻探针中 FTRAN conversion/export
从 `0.1025s` 降至 `0.0349s`（-66.0%，满足 >=50% bucket 门），总 export 从
`0.1093s` 降至 `0.0417s`；pivot path 与 accuracy 门满足。

总收益低于 2--5% 预测且偏差超过 50%，成本模型修订如下：packed pivotal AQ export
在当前二进制中只有约 `0.0676s` 可消除流量，占 `5.05s` minor time 的约 1.34%，而非
此前把 BFRT/DSE auxiliary FTRAN 一并计入后的 `0.373s`；本轮并未迁移后两者。此外，
为保持 factor-update 事务性必须保存 AQ support 的 row indices，抵消部分 value export
收益。因此 0.66% 与实际可消除 bucket 同量级，方向正确但不能声称 2--5%。保留默认
resident AQ，因为完整门通过且相邻 A/B 为正；后续若继续优化，须分别测 BFRT RHS 与
DSE auxiliary FTRAN，而不能再用合并 bucket 外推 pivotal AQ 收益。

P2/P3 Auto cohort mismatch（MSVC 19.44 Release，`/O2 /Ob2 /DNDEBUG`，base
commit `a0322e38` + 工作树）：按本节预先固定的 7 例 cohort 运行
`tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases
greenbea,maros-r7,dfl001,pilot87,afiro,adlittle,sc50a --solvers
native-ipm,native-ipm-direct --repeat 1 --time-limit 30 --max-iterations 2000
--json reports/native_ipm_p23_auto_cohort_r1.json`。完整 P1/P2/P3 臂为 6/7
accurate，direct 为 7/7 accurate。唯一失败的 `dfl001` 从
`6071x12230/35632 nnz` 约到 `5895x10842/33838 nnz`，presolve `103.90ms`、
总时长 `2381.21ms/46` 轮，但恢复后的目标 `11266637.9060` 相对参考值误差
`2.147e-5`，而 direct 为 `2318.99ms/44` 轮、目标误差 `1.391e-8`；presolve
解的归一化 primal violation `4.293e-9` 仍通过当前只检查原模型可行性的 publication
audit，说明该 audit 不能证明 P2 目标变换/恢复的最优性。

性能预测也只部分成立：四个长尾案例的几何平均从 `2027.03ms` 降到
`1367.56ms`（-32.5%），但其中 `dfl001` 不准确，不能计入胜出；小型对照
`afiro,adlittle,sc50a` 累计从约 `1.90ms` 增到 `5.51ms`（+189%），超过预先固定的
5% 退化门。因此禁止把 P2/P3 打开到 Auto，`native_lp_selector.cpp` 继续保持
`opt.presolve=false`，显式 `native-ipm` 仅作为研究路径。

按 mismatch 顺序，当前首先判定为 implementation fidelity 未证实，而不是放宽数值
门：P3 只改变 box，P2 substitution 同时改写约化目标和 postsolve primal；可行但目标
错误优先指向 P2 的 objective coefficient/constant 传递或 substitution replay。下一轮
固定做同一 `dfl001` 二进制的 `P2-off/P3-on` 与 `P2-on/P3-off` 正交探针，并记录 P2
规则计数和 objective offset。若 P2-off 恢复 accuracy，则逐条核对 A&A (1995) §2.4
代入公式及附录 A 的目标更新；若 P3-off 才恢复，则核对 tightened bounds 是否在
postsolve 时被错误保留。修复验收固定为：7/7 原模型 accuracy、`dfl001` objective
relative error <= `1e-7`、其余 6 例不退化为错误；满足前不得改 Auto 路由。

正交探针结果确认 P2 为根因（同一 Release 二进制；报告
`reports/dfl001_native_ipm_p2_off_p3_on_r1.json` 与
`reports/dfl001_native_ipm_p2_on_p3_off_r1.json`）：`P2-off/P3-on` 的原模型目标
误差恢复为 `1.24e-8`、accuracy 通过；`P2-on/P3-off` 仍为 `4.96e-5`、accuracy
失败，两臂 primal violation 均在 `1e-8` 量级。因此排除 P3 tightened box，下一步只
审计 P2 doubleton/singleton substitution 的 objective coefficient、constant offset 和
反向 replay；在写回公式核对结论前不修改实现。

恒等式探针进一步排除 P2 公式错误：`dfl001` 的 postsolve 原目标与
`c_red*x_red + offset` 只差约 `1.1e-5` 绝对值，但 P2 累积的 objective offset 为
`8.0402367697e9`，远大于原目标约 `1.1267e7`。约化 IPM 的 KKT audit 使用
`|p_r-d_r|/(1+|p_r|+|d_r|)`；代入常数 `kappa` 后，原问题应使用
`|p_r-d_r|/(1+|p_r+kappa|+|d_r+kappa|)`。分子严格不变，但当前实现遗漏 `kappa`
导致大抵消问题按错误的超大分母提前终止。这是 audit wiring 的 implementation
infidelity，不是 substitution algebra 或 P3 错误。

修复前固定算法卡：给 `audit_ipm_lp_optimality` 增加原目标 sense 下的可选 constant
offset；审计内部转为 minimization sense 后同时加到 primal/dual objective，再计算
relative gap。native P2/P3 reduced solve 把 `nps.objective_offset` 一路传入每次迭代和
最终 audit；direct solve 的 offset 恒为 0，轨迹与结果不变。依据 A&A (1995) §2.4
的等价 substitution，常数只平移 primal/dual objective，不改变 stationarity、可行性
或 gap 分子。固定预测：`dfl001` 从 46 轮增至约 50--65 轮，总时长约
`2.6--3.5s`，objective relative error <= `1e-7`；其余 cohort 保持 accurate。
先加一个 cancellation-scale audit 单测，再跑 `dfl001` 和 7 例 cohort。若达到数值
地板或实测成本偏离预测超过 50%，先回写本节，不修改 Auto。

offset-aware gap 实测符合预测。新增 cancellation-scale audit 连同原有 audit 共
5 cases / 37 assertions 通过。`dfl001` 为 52 轮、`3102.53ms`，落在预先固定的
50--65 轮和 `2.6--3.5s` 区间；objective relative error 从 `2.147e-5` 降为
`2.921e-8`，normalized primal violation 为 `2.690e-11`。同一 7 例 cohort 报告
`reports/native_ipm_p23_offset_gap_fix_cohort_r1.json` 中 P1/P2/P3 与 direct 均为
7/7 accurate；四个长尾案例几何平均为 `1558.08ms` 对 direct `2402.80ms`
（P1/P2/P3 快 35.2%），但小型三例累计 `6.22ms` 对 `1.80ms`，且 `pilot87`
为 `1794.11ms` 对 direct `1340.36ms`。因此 correctness mismatch 已修复，性能
方向和 `dfl001` 成本预测成立；Auto 的“小型退化 <=5%”门仍失败，selector 继续
保持 `presolve=false`。

最终验证（base commit `a0322e38773b38be91c9b13d8e18c306b948f5e0` +
工作树，MSVC 19.44 Release，`/O2 /Ob2 /DNDEBUG /fp:fast`）：完整构建
`cmake --build build/windows-msvc-release --config Release --parallel 8` 成功；
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`
为 19/19 tests passed，real time `271.35s`。focused 验证为 LP presolve
23 cases / 954 assertions、dual simplex 79 cases / 23094 assertions、IPM audit
5 cases / 37 assertions，均通过。`git diff --check` 无错误，新增占位词扫描为空。

### 8.19 DSE auxiliary FTRAN 的 factor-resident 结果（2026-08-19）

理论依据仍为 Goldfarb--Reid (1977) 的 dual steepest-edge 递推，以及
Hall--McKinnon (2005) §2--4、Huangfu--Hall (2018) §2--4 的超稀疏
TRAN/HVector 数据流。令 `rho=B^{-1}row_ep`、`beta=B^{-1}a_q`；非主元行更新
只在 `supp(beta)` 上读取 `rho_i`。当前 backend 已把 `rho` 留在
`solve_vec_ftran`，但仍按 `supp(beta)` 将值复制到 `std::vector<double>`，Native
随后按同一 support 顺序读回。正确的所有权模型是：factor 持有完整 `rho` HVector，
用独立 workspace generation 约束只读 view；DSE 仍按 captured AQ 的原 support 顺序
迭代，并用 external row 映射直接读取 `rho_i`。下一次复用 `solve_vec_ftran`、INVERT
或 factor update 都必须使 view 失效。诊断开关
`MIPSOLVERS_DS_PACKED_DSE_FTRAN=1` 保留原 value-export 路径用于同二进制 A/B。

成本模型：改动不减少三角求解，只删除每个 DSE pivot 的 `nnz(beta)` 次 value copy、
`push_back` 和随后一次线性读取；DSE 递推的乘加及 AQ support 遍历不变。MSVC 19.44
Release、base commit `a0322e38` + 当前工作树的固定基线命令为
`tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases pilot87
--solvers native-dual-structural-dse --repeat 1 --time-limit 60 --max-iterations
100000`，并设置 `MIPSOLVERS_DS_PROFILE=1`；报告
`reports/pilot87_dse_resident_value_export_baseline_r1.{json,log}`。基线为 8572 dual
pivots、总时长 `9070.79ms`，DSE bucket `2.52s`；全部 indexed FTRAN conversion/export
仅 `0.0764s`，所以不能把整个 DSE bucket 当成可消除成本。

固定预测：resident 相对 packed control 的 DSE bucket 下降 0.5--2.0%，总时长下降
0.1--0.6%；两臂必须保持 8572 dual pivots、相同 iterations/objective/status 和原模型
accuracy。focused 单测逐坐标比较 resident `rho.at(row)` 与旧 packed values，并验证
下一次 auxiliary FTRAN 及 factor update 后旧 view 失效；随后跑完整 dual-simplex。
性能用相邻 repeat-3 A/B 的中位数判断。若方向错误，或以预测区间中点计偏差超过
50%，先按 workspace generation -> internal/external row permutation -> support 顺序 ->
机器频率排查并回写本节，再继续 BFRT resident 化。

首轮实现与 mismatch：resident/packed 单次 profile 都为 8572 pivots、相同 objective、
相同原模型残差且 accurate；resident 把 FTRAN conversion/export 从 `0.0771s` 降到
`0.0371s`，DSE bucket 从 `2.59s` 降到 `2.57s`（约 0.77%，符合 bucket 预测），但总时长
反而为 `9289.31ms` 对 `9213.97ms`。随后 resident-first 的相邻 repeat-3 为
`8843.97/8835.97/8935.21ms`（中位 `8843.97ms`），packed 为
`9292.25/9004.13/9319.41ms`（中位 `9292.25ms`），表面总收益 4.82%，远超 0.1--0.6%
预测且与单次探针方向冲突。完整 dual-simplex 仍为 79 cases / 23103 assertions 通过，
所以当前没有 implementation fidelity 或数值路径证据；可消除的约 40ms export 也无法
解释约 448ms 的中位差。按预定顺序将本次失配归入尚未排除的机器频率/执行顺序效应，
在反向 packed-first repeat-3 复核前不接受 4.82% 为算法收益，也不继续 BFRT 改动。

反向复核确认机器/顺序假设失效：packed-first 中位 `8706.33ms`，随后 resident 中位
`8739.52ms`，resident 反而慢 0.38%。进一步按 `R,P,P,R,R,P,P,R` 交错的四对结果中，
前三对 resident 相对差约 `+0.11%/-0.60%/-1.31%`，第四对外部状态突变为 packed
`7622.86ms`、resident `9434.00ms`，不能用于微秒级路径归因。因此本轮只接受可直接
归因且重复出现的 wrapper 指标：export/convert 减少约 `0.040s`，pivot path 和 accuracy
不变；不声称 wall-time 加速，也不以受污染的 4.82% 更新总体性能结论。实现保留是因为
它完成 factor-resident 所有权并删除确定存在的复制，而不是因为未复现的总时长收益。
后续总时长验收须在固定 affinity/频率的独占环境重新执行；当前成本模型的“约 40ms
可消除”成立，而“steady_clock 可在本机解析 0.1--0.6%”这一测量假设不成立。

最终验证（base commit `a0322e38773b38be91c9b13d8e18c306b948f5e0` + 工作树，
MSVC 19.44 Release）：resident 与 `MIPSOLVERS_DS_PACKED_DSE_FTRAN=1` 两臂的完整
`test_dual_simplex` 均为 79 cases / 23109 assertions 通过；完整
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure` 为 19/19
tests passed，real time `184.19s`。resident-view focused case 为 1 case / 57 assertions，
覆盖 support 坐标、value、`at(row)`、下一次 auxiliary FTRAN 失效和 factor update
失效。`git diff --check` 与新增占位词扫描均通过。

### 8.20 BFRT FTRAN resident 生命周期与 primal proposal 前移（2026-08-19）

依据 Harris (1973) 的两遍 ratio test、Maros (2003) §9 和 Huangfu--Hall (2018)
§2--4 的 BFRT/revised-simplex 数据流，bound flips 形成原行空间 RHS `r_flip`，其基本值
修正为 `d=B^{-1}r_flip`；随后 pivotal column `beta=B^{-1}a_q` 给出 transaction
`x_B' = x_B - d - beta*primal_step`。当前 `d` 已在 `solve_vec_ftran` 内产生，但先导出
packed `IndexedVector`；DSE auxiliary FTRAN 随后复用并覆盖同一 scratch，迫使 primal
transaction 在 DSE 后从 packed backing 构造。正确生命周期是：BFRT FTRAN 返回独立
workspace-generation view，先完成 leaving-row certificate 和完整 primal proposal，再运行
DSE 覆盖 scratch；proposal 只写 worker-local `MinorScratch`，factor/basis/move/x/reduced-cost
状态仍到所有证书通过后才 commit。因此前移不改变事务原子性，也不改变 BFRT support、
每行加法顺序或浮点表达式。`MIPSOLVERS_DS_PACKED_BFRT_FTRAN=1` 保留原路径。

成本模型：resident view 删除每个有 flip pivot 的 `nnz(d)` 次 internal-to-external index
转换、index/value push 和后续 packed backing 读取；primal epoch accumulator 与 `beta`
遍历不变。§8.19 resident DSE profile 后剩余 FTRAN conversion/export 约 `0.0371s`，其中
BFRT 是主循环中仍导出完整 FTRAN support 的路径。固定预测为该剩余 bucket 再下降
40--90%；受 §8.19 已确认的 20% 级机器波动影响，不把 0.2--0.4% 理论 wall-time 收益
设为当前机器的接受门。正确性硬门为 StructuralDSE `pilot87` 两臂均 8572 dual pivots、
相同 iterations/objective/status/原模型 accuracy，Devex P1-only 两臂均 8749 dual pivots；
focused test 逐 coordinate 比较 packed/resident support、value、`at(row)`，并验证 DSE
scratch 复用使旧 BFRT view 失效。完整 dual-simplex 两臂都必须通过。若 export bucket
下降不足 40%，先按 BFRT 无-flip占比 -> 其它 auxiliary export -> profile 计时边界回写，
再触碰 PRICE/BFRT 算术。

实测满足预测（MSVC 19.44 Release、base commit `a0322e38` + 工作树）。StructuralDSE
`pilot87` resident/packed 均为 8572 dual pivots、相同 objective/residual 且 accurate；
FTRAN conversion/export 从 `0.0757s` 降到 `0.0186s`（-75.4%）。Native-presolve
Devex 两臂均为 8749 dual pivots、相同 objective/residual 且 accurate；对应 bucket 从
`0.0690s` 降到 `0.0198s`（-71.3%）。两组均落在预先固定的 40--90% 区间。Devex
相邻 wall probe 为 packed `9953.40ms`、resident `9903.06ms`（resident 快 0.51%）；
StructuralDSE 为 packed `13420.31ms`、resident `13830.88ms`，与 profile 中整机各阶段
同步变慢一致，按 §8.19 不用于微优化归因。focused resident 生命周期为 1 case / 72
assertions；默认及 `MIPSOLVERS_DS_PACKED_BFRT_FTRAN=1` 完整 dual-simplex 均为
79 cases / 23124 assertions。接受依据是 bucket、pivot path 与 accuracy，不外推噪声 wall。

### 8.21 PRICE/BFRT 算术分解与连续元数据流（实现前算法卡，2026-08-19）

依据 Harris (1973) 的两遍 ratio test、Maros (2003) §9 以及
Huangfu--Hall (2018) §2--4，CHUZC 的必需数据流是：对 PRICE 产生的活动非基列按原
support 顺序计算带符号 `alpha` 与稳定性证书，按 breakpoint group 累积 boxed-column
capacity，再对未翻转后缀做 Harris pass 1/2，最后以同一活动列顺序审计更新后的对偶
可行区间。任何纯性能变换都必须保持候选插入顺序、breakpoint 分组、capacity 求和顺序、
Harris tie-break 和 BFRT RHS 行内求和顺序逐位不变。

MSVC 19.44 Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）、base commit
`a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树的 resident 基线报告
`reports/pilot87_bfrt_resident_devex_profile_r1.{json,log}` 为 8749 pivots：PRICE
`1.70s`、entering/BFRT `2.26s`，其中 order `0.1152s`、sort `0.0200s`；平均
1555.3、最大 2999 个 candidates，平均 1.14 个 flips。因此排序只占 entering 的约
6.0%，不能解释主成本；剩余约 `2.12s` 必须先分解为预筛、候选构造、Harris 两遍、
终端对偶区间扫描和 BFRT RHS 构造，才能选择优化对象。

第一步只增加由 `MIPSOLVERS_DS_PROFILE=1` 门控的细分计时，不改变生产路径算术。
固定预测：上述五个子 bucket 加 order 的和应解释 entering bucket 的至少 80%，计时
开销不超过 entering 的 2%；若不能解释，则先检查早退路径和计时边界。验证命令固定为
`tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases pilot87
--solvers native-dual-devex-native-presolve --repeat 1 --time-limit 60
--max-iterations 100000`；必须仍为 8749 pivots、相同 status/objective/原模型残差且
accurate。随后只优化最大的可消除子 bucket；Class-P 接受门为 StructuralDSE 8572 与
Devex 8749 pivots 均逐路径相同，目标与 accuracy 不变。性能预测和独立 A/B 门在看到
细分结果后追加到本节；若收益方向错误或相对预测偏差超过 50%，按实现保真度 ->
cache/indirection 成本模型 -> 支撑密度假设顺序排查并先回写 mismatch。

细分实测满足诊断预测：报告
`reports/pilot87_bfrt_split_devex_profile_r1.{json,log}` 为 8749 pivots、accurate，
entering `2.30s`；prefilter `0.4537s`、candidate `0.9522s`、order `0.1151s`、
Harris `0.0611s`、terminal scan `0.5277s`、RHS `0.0116s`，合计 `2.1215s`，
解释 entering 的 92.4%。计时后的 entering 相对无细分基线 `2.26s` 增约 1.8%，也在
2% 预设门内。结论是 BFRT RHS 和排序都不是下一主成本；首先消除 prefilter/candidate
对平均 1555.3 个活动列的重复间接访问。

固定 Class-P 变换：默认把 scalar prefilter 与 candidate classification 融合成单次按
`active_position` 顺序的循环；每列仍按原表达式计算 `direction`、`alpha`、
`cheap_error` 和四个 flags，立即交给与控制臂共享的分类逻辑。这样删除第二次
`active_position -> pivot_row -> state` 间接读取、`signed_alpha/flags` 工作区写回读回，
并复用同一 `direction/cheap_error`；候选插入及 capacity 加法顺序不变。
`MIPSOLVERS_DS_SPLIT_BFRT_SCAN=1` 保留原两遍路径。以两遍臂的
prefilter+candidate `1.406s` 为基数，预测融合 scan bucket 下降 15--35%
（约 `0.21--0.49s`），entering 下降 9--21%，总时长下降 2--5%。固定 A/B 仍用上述
Devex 命令，并补 StructuralDSE；两臂分别必须为 8749/8572 pivots、相同
status/objective/residual 且 accurate，完整 dual-simplex 通过。若偏差超过 50% 或方向
错误，先按实现保真度 -> cache/间接读取成本 -> 编译器内联/寄存器压力排查并回写。

融合实测满足 Class-P 正确性门，但算术收益位于预测下沿。MSVC 19.44 Release、base
commit `a0322e38` + 当前工作树的相邻 Devex A/B 报告为
`reports/pilot87_bfrt_fused_devex_profile_r1.{json,log}` 与
`reports/pilot87_bfrt_split_control_devex_profile_r2.{json,log}`：两臂均为 8749 pivots、
accurate，目标与原模型残差相同。split scan 为 `0.4525+1.0416=1.4941s`，fused 为
`1.3079s`，下降 12.5%；entering 从 `2.37s` 降到 `2.17s`，下降 8.4%；wall 从
`9989.75ms` 降到 `9673.83ms`，下降 3.16%。wall 落在预设 2--5% 区间；scan 与
entering 略低于 15%/9% 下沿，但相对区间中点的偏差未超过约 50% 停线门。修正后的
成本解释是：融合只删除元数据回写与第二遍间接读取，候选分类、breakpoint 算术和分支
仍完整保留；MSVC 的融合循环还增加 live range 与寄存器压力，因此不能把两遍时间差
全部视为可消除成本。

StructuralDSE 控制报告
`reports/pilot87_bfrt_fused_dse_profile_r1.{json,log}` 与
`reports/pilot87_bfrt_split_control_dse_profile_r1.{json,log}` 均为 8572 pivots、accurate，
目标与残差相同；scan 从 `0.4792+1.1094=1.5886s` 降到 `1.4489s`（-8.8%），entering
从 `2.57s` 降到 `2.48s`。wall 为 split `13279.63ms`、fused `13536.10ms`，但该次
profile 的 factor、DSE 等非目标阶段同步变慢，按 §8.19 的归因规则不把 wall 反向归于
本变换。默认 fused 与 `MIPSOLVERS_DS_SPLIT_BFRT_SCAN=1` 完整 dual-simplex 均为
79 cases / 23124 assertions。接受依据是目标 scan bucket 为正、pivot path 逐例一致及
accuracy 全通过；默认保留融合路径。

### 8.22 P1/P3-first staged P2 与 reduced-solve 摊销门（实现前算法卡，2026-08-19）

依据 Andersen--Andersen (1995) §2--3，P1 删除、P3 activity-bound projection 与 P2
substitution 都保持 LP 等价，但“规则正确”不等于“投机 reduced solve 有正收益”。对当前
稀疏 IPM，粗略成本模型为
`T = T_presolve + iterations * (T_factor(pattern) + T_solve(nnz))`；P2 的 mutable
`std::map` adjacency 还引入约 O(nnz log degree) 的构造/更新成本。因此 staged policy
先用零填充 P1/P3 收敛到结构稳定点，只在便宜结构证据预测后续 kernel 节省能摊销 P2
时才进入 substitution。该顺序也符合 Achterberg et al. (2020) §3--5 的 fast-to-
expensive presolve 分层思想；P2 启用后仍使用附录 A 的 Markowitz/fill/stability 门，数学
规则不变。

固定四例正交探针（MSVC 19.44 Release、base commit `a0322e38` + 当前工作树）排除了
按原始规模或仅按 P1 缩减率路由。P1-only 报告
`reports/native_ipm_p1_longtail_probe_r1.{json,log}`：`dfl001` 与 `maros-r7` 无缩减，
`greenbea` 为 `2392x5405/30877 -> 2315x5276/30231`，`pilot87` 为
`2030x4883/73152 -> 2010x4658/70639`。P1+P2 报告
`reports/native_ipm_p1_p2_longtail_probe_r1.{json,log}` 中 `greenbea` 虽约到
`1906x4733/28626` 却 30s 超时，证明 P3 对其 reduced IPM 数值轨迹是必要前置；P1+P3
报告 `reports/native_ipm_p1_p3_longtail_probe_r1.{json,log}` 则为 4/4 accurate，
`greenbea` 约到 `1951x4181/24142`、`pilot87` 约到 `1991x4622/70513`，另两例不发布。
完整 P1/P2/P3 的既有报告中 `greenbea/maros-r7` 获益，而 `dfl001/pilot87` 退化。

固定策略只在 IPM 的 adaptive staging 路径生效，强制控制臂仍可关闭 adaptive。令 P1/P3
稳定后的活动行列为 `m_a,n_a`，`r_m=1-m_a/m_0`、`r_n=1-n_a/n_0`，令 `s_eq` 为
一次 CSC 活动支撑扫描发现的 singleton equality columns。若 `m_0+n_0 < 4000`，在构造
presolve adjacency 前直接返回原模型；否则仅当
`max(r_m,r_n) >= 0.10` 或 `s_eq/n_a >= 0.25` 时进入 P2。P2 后若最终
`max(r_m,r_n) < 0.10`，不 compact、不启动 reduced solve。常数由摊销下界固定：当前
长尾 P2/compact 成本为约 74--477ms，至少 10% 的大模型结构下降才有覆盖空间；25%
singleton 密度捕获 `maros-r7` 的 66.7% singleton、44% 最终列删除，同时排除
`dfl001` 的约 11.2% 与 `pilot87` 的约 0.4%。这些是发布策略，不改变任何 reduction
公式；常数在实现处引用本节。

实现前固定预测与验证协议：adaptive 四长尾必须 4/4 accurate；`greenbea` 与
`maros-r7` 必须进入 P2，最终行或列缩减至少 10%，总时长分别落在 `0.9--2.0s` 与
`0.6--1.3s`；`dfl001` 必须跳过 P2 并不发布 reduced solve，`pilot87` 必须跳过 P2 或因
最终缩减不足不发布，两者相对同批 direct 退化不超过 5%。`afiro/adlittle/sc50a` 必须在
adjacency 构造前跳过，累计 wall 相对 direct 退化不超过 5%。另用 adaptive-off 复现旧
完整 P1/P2/P3 作为控制臂；focused LP presolve、IPM audit 和完整 suite 必须通过。若
任一模型路由与预测方向相反或性能收益偏差超过约 50%，先按 gate 实现保真度 -> 当前
机器/计时噪声 -> IPM pattern 成本假设 -> 阈值理论顺序调查并回写本节。

首轮实现触发顺序假设 mismatch。adaptive 报告
`reports/native_ipm_adaptive_staging_cohort_r1.{json,log}` 为 7/7 accurate，路由与预测
完全一致：`greenbea/maros-r7` 进入 P2，`dfl001/pilot87` 不发布 reduced solve，三个小例
在 adjacency 前返回；但 `greenbea` 先把 P1/P3 跑到稳定再原位继续 P2 后得到
`1539x3705/23232`、`3707.64ms`，超过预设 2.0s 上界。相同二进制的 adaptive-off
控制 `reports/native_ipm_eager_control_longtail_r1.{json,log}` 按旧 P1/P3/P2 交错顺序
得到 `1529x3682/23171`、`2135.51ms`；`maros-r7` 两序尺寸相同，分别
`1334.88/1262.18ms`。因此实现忠实执行了 staged 规则，错误位于“等价 reduction 的
阶段顺序不影响 reduced IPM 成本”假设：不同可交换规则的先后改变最终稀疏 pattern、
bound box 和 barrier 轨迹，`greenbea` 对此敏感。机器波动不能解释 73.6% 的同构建差距。

修正模型：P1/P3-first 只作为决策预跑；若 gate 拒绝 P2，预跑结果可继续用于最终
insufficient-reduction 判断；若 gate 接受 P2，必须丢弃预跑的工作区、postsolve stack
和计数，从原 LP 重建并恢复已验证的 eager P1/P3/P2 交错顺序。重建不改变任何公式，
且预跑成本由本轮 P1+P3 测得约 3--17ms。修正预测固定为：接受 P2 的两例最终尺寸与
规则计数逐项等于 adaptive-off 控制；预跑使 presolve 增量小于 20ms，wall 相对相邻
eager 控制不退化超过 5%。拒绝 P2 与小模型路径保持首轮路由和 accuracy。先实现该
replay，再重跑 7 例 cohort；若 `greenbea` 仍不恢复 eager 尺寸，优先检查工作区、
objective offset、postsolve stack 或 rule counts 是否未完全重置。

replay 修复后的首次 7 例报告暴露了第二个 measurement-fidelity mismatch：
`benchmark/netlib_solver_benchmark.cpp` 的 `native-ipm` key 同时设置
`opt.presolve=true` 与 `opt.use_highs_presolve=true`。因此 Native 返回 `small_model` 或
`insufficient_reduction` 后仍进入 HiGHS presolve second chance；`adlittle` 的
`9.06ms` 对 direct `0.95ms` 和 `pilot87` 的非 direct 轨迹都不能验证本节 adaptive
预测。生产 `IPMLPOptions::use_highs_presolve` 默认 false，故这是隔离基准不忠实，不是
生产路由缺陷。修复固定为 `native-ipm`/legacy key 只设置 Native master switch，结果
标签按 `opt.presolve` 判别；HiGHS bridge 仍可由 API/环境单独启用。重跑时
三个 small-model 与 `dfl001/pilot87` 必须直接进入原模型 kernel，iterations、目标与 direct
相同；差异只允许 Native 预跑的 0--20ms。若仍不同，检查环境覆盖或 IPM fallback 分支。

最终 Native-only 验收满足修正后的结构、准确性与长尾性能预测。报告
`reports/native_ipm_adaptive_native_only_cohort_r1.{json,log}` 为 7/7 accurate：
`greenbea` replay 后逐项复现 eager 的 `1529x3682/23171`、53 轮，`1199.76ms` 对
direct `5874.27ms`（-79.6%，落在 0.9--2.0s 预测）；`maros-r7` 复现
`2152x5288/98334`、15 轮，`708.07ms` 对 `1098.64ms`（-35.6%，落在 0.6--1.3s
预测）。两例的 rules/candidate counts 也与 adaptive-off 控制相同，证明预跑工作区、
stack、offset 与计数已经完全重置。adaptive 总 presolve 分别为 `103.3/241.0ms`；由于
相邻 eager 单次自身在 64--357ms 间波动，不能从非配对 wall 分离预跑增量，但独立
P1/P3 探针的 9--17ms 仍符合成本模型。

拒绝臂也满足预测：`dfl001` 为同 44 轮、`2374.21ms` 对 direct `2365.61ms`，Native
预跑 `3.08ms`、总退化 0.36%；`pilot87` 为同 34 轮、Native 预跑 `8.31ms`，不发布
reduced solve，wall `1042.43ms` 对 `1129.28ms` 的正差只作噪声、不归因。首个 cohort
中 `adlittle` 的 9.37ms 来自进程首次 IPM 初始化，不是 0.001ms 的 adaptive check；
反转 solver 顺序的 `reports/native_ipm_adaptive_small_reverse_r3.{json,log}` 把首次
`4.05ms` 转移到 direct，稳态 `adlittle` direct `0.73--0.76ms`、adaptive
`0.72--0.73ms`，`afiro/sc50a` 同样无退化。故小模型 adjacency-before-return 门成立，
原预设的单次累计 5% wall 门在亚毫秒模型上受一次性初始化支配，修正为相同 iterations、
目标与 repeat-3 稳态中位不退化；三例均满足。

最终验证环境为 MSVC 19.44 Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）、base commit
`a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树。完整构建命令
`cmake --build build/windows-msvc-release --config Release --parallel 8` 成功；
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure` 为 19/19
passed，real time `187.15s`。`git diff --check` 与新增占位词扫描均为空。

### 8.23 pilot87 PRICE/DSE 算术分解与扩大 presolve cohort（实现前算法卡，2026-08-19）

依据 revised dual simplex 的稀疏 PRICE 乘积
`alpha_N = A_N^T B^{-T} e_p`（Maros 2003 §9；Huangfu--Hall 2018
§2--4），partitioned row matrix 的工作量近似为
`W_price = sum_{i in supp(row_ep)} nnz_N(A_i)`，另加 leaving basic column 的
`nnz(A_q)` dot、`n_touched` 次 tiny filter/packing 和一次有限性审计。`pilot87`
的 row_ep/pivot-row 平均密度已经达到 58--66%，故它不是 hypersparse lookup 问题；
当前首要问题是确认上述四项中究竟哪一项消耗 PRICE 的 `1.68--1.99s`，再在不改变
row support、row-slot、touched 和 packed 顺序的前提下降低访存或重复算术。

StructuralDSE 继续使用 Goldfarb--Reid recurrence（Goldfarb--Reid 1977；Maros
2003 §9.3）
`beta_i' = beta_i - 2 (alpha_i/alpha_p) rho_i
                 + (alpha_i/alpha_p)^2 beta_p`，并保留现有四项 roundoff floor、
`long double` 表达式顺序及 transaction 写入顺序。其成本分为 auxiliary
`B^{-1}rho` FTRAN 和在 `nnz(A_q)` support 上的 recurrence/transaction；任何默认优化
必须逐位保持计算结果，不能用现有 `MIPSOLVERS_DS_LEAN_DSE` 的较弱 floor 代替。

第一步只增加 `MIPSOLVERS_DS_PROFILE` 门控的子计时。固定预测为 PRICE 的
accumulate/leaving-dot/pack/audit 合计解释顶层 PRICE 至少 90%，DSE 的 auxiliary
FTRAN/recurrence 合计解释 StructuralDSE 顶层 DSE 至少 90%；相对已有 profile，新增
计时使对应顶层 bucket 增长不超过 3%。固定命令为 `pilot87` 单例、repeat 1、60s、
100000 iterations，分别运行 `native-dual-devex-native-presolve` 和
`native-dual-dse-native-presolve`。必须仍为 8749/8572 dual pivots、相同
status/objective/原模型残差且 accurate。看到分解后只优化最大的可消除 bucket；
Class-P 的预测、A/B 控制开关和验收区间必须在实现前追加。若方向错误或相对预测偏差
超过 50%，按实现保真度 -> cache/indirection 机器成本 -> 稠密支撑假设 -> 理论模型
顺序排查，先回写 mismatch 再继续改代码。

Native presolve 的发布判断使用与上述微基准分离的 20 例中大型 cohort：
`fit2d,fit2p,wood1p,woodw,d6cube,d2q06c,80bau3b,greenbeb,degen3,cycle,
nesm,bnl2,pilot,pilotnov,ship12l,ship08l,fit1d,czprob,sierra,25fv47`。
MSVC Release 下以 `native-ipm,native-ipm-direct`、repeat 1、单例 30s、最多 2000
iterations 先筛选；对接近 5% 边界或受首次初始化影响的例再做反序 repeat 3。Auto
只在以下预先固定的门全部满足时打开：所有两臂成功例 accurate；Native presolve 不得
新增失败或超时；成功配对例 wall 几何平均不退化；任一确认的稳态 loser 不得退化超过
5%；至少三个不同结构族获得超过 10% 的稳态收益；§8.22 的小模型 early gate 反序
稳态仍不退化。否则保持 Auto 关闭，并记录 loser 的规模、缩减率、P2 决策、presolve
成本与 kernel iterations，作为下一轮 selector 特征，而不在本轮追调阈值。

算术细分满足解释率与路径预测。报告
`reports/pilot87_price_dse_split_devex_r1.{json,log}` 为 8749 pivots、accurate，
PRICE `1.13s` 中 accumulate `0.8982s`、leaving dot `0.0017s`、pack
`0.1737s`、audit `0.0528s`，合计解释 99.9%。报告
`reports/pilot87_price_dse_split_dse_r1.{json,log}` 为 8572 pivots、accurate，
PRICE `1.17s` 中四项为 `0.9385/0.0018/0.1677/0.0564s`，同样解释
99.9%；DSE `2.45s` 中 auxiliary FTRAN `0.6910s`、recurrence/transaction
`1.7473s`，解释 99.6%。因此 leaving dot、pack 和 export 不是本轮首要对象，最大可消除
成本是 PRICE accumulate 的逐 slot accessor，以及 resident DSE 对共享 captured-AQ
pattern 的重复内部坐标解码。

固定 Class-P 变换与预测如下。PRICE 由 `PartitionedRowMatrix` 返回连续 nonbasic row
slice，在完全相同的 row_ep、row slot 和 touched 顺序中用指针推进；
`MIPSOLVERS_DS_ACCESSOR_PRICE=1` 保留逐 slot accessor 控制。该变换每个 matrix entry
删除 `row_start/nonbasic_end/col_at/value_at` 的重复 vector 索引与 64 位 slot 转换，预测
accumulate 下降 5--15%，PRICE 下降 4--12%，dual wall 下降 0.6--2%。DSE 的 AQ 与
auxiliary rho view 引用同一个 captured-AQ index stream；默认每个 position 只解码一次
stored index，以它同时取得 external row、AQ value 和 rho value；
`MIPSOLVERS_DS_ACCESSOR_DSE=1` 保留三次 accessor 控制。该变换删除每个 updated row
两次相同的 index-stream load/decode，预测 recurrence/transaction 下降 10--25%，DSE
下降 7--18%，StructuralDSE wall 下降 2--5%。实现处引用本节；两项均不得改变
roundoff floor 或向量 push 顺序。

固定验证先分别 A/B：PRICE 用 Devex，DSE 用 StructuralDSE，仍使用上述单例命令；
控制臂只打开对应环境变量，默认臂紧邻运行。两臂必须分别保持 8749/8572 pivots、相同
status/objective/residual 且 accurate，完整 dual-simplex 通过。bucket 结果按上述区间
判定；受 §8.19 已证实的机器频率波动影响，wall 只在目标 bucket 同方向时作辅助证据。
若任一目标 bucket 方向错误或相对区间中点偏差超过 50%，先回写 mismatch，且只保留
通过预测的变换。

首轮 A/B 中 PRICE 满足预测，而 DSE 触发 mismatch。Devex 控制/连续 slice 报告
`reports/pilot87_price_accessor_control_devex_r1.{json,log}` 与
`reports/pilot87_price_slice_devex_r1.{json,log}` 均为 8749 pivots、accurate；accumulate
从 `0.8750s` 降至 `0.8027s`（-8.26%，落在 5--15% 区间），PRICE 从
`1.110s` 降至 `1.030s`（-7.2%，落在 4--12% 区间）。相邻 wall 从
`6116.63ms` 到 `5925.02ms`（-3.13%），超过预设 wall 上沿，但 factor、entering、
cleanup 同时变快，按 §8.19 只把 bucket 收益归于本变换。

StructuralDSE 控制/共享 pattern 报告
`reports/pilot87_dse_accessor_control_r1.{json,log}` 与
`reports/pilot87_dse_aligned_resident_r1.{json,log}` 均为 8572 pivots、accurate；recurrence
只从 `1.7708s` 到 `1.7467s`（-1.36%），显著低于 10--25% 预测，相对区间中点偏差
约 92%，因此停线。实现保真度证据是相同 pivot path、目标和残差，且默认确实进入
resident-aligned 分支；下一顺位假设是 MSVC 已把原 accessor 分支内联/公共子表达式化，
而本轮新分支与三个输出变量抵消了 index decode 节省，或 24ms 差异本身处于机器噪声。
在反序复核前不接受该 DSE 变换，也不继续改 recurrence 算术；若反序仍不足 5%，删除
该默认路径并把下一成本模型改为四项 roundoff-floor 算术与 transaction stores，而不是
resident accessor。

反序报告 `reports/pilot87_dse_aligned_reverse_r1.{json,log}` 与
`reports/pilot87_dse_accessor_control_reverse_r1.{json,log}` 再次保持 8572 pivots 与
accuracy，但 recurrence 为 `1.7641s/1.7821s`，aligned 仅快 1.01%；同时 wall 为
`8731.39/8649.10ms`，方向与首轮相反。故约 1% bucket 差不能从机器噪声中分离，
“三次 index decode 是主成本”的假设被否定，未达门的 aligned resident 路径删除。

修正后的 DSE Class-P 模型针对 MSVC 的 `sizeof(long double)==sizeof(double)`：当前
Goldfarb--Reid value 与四项 roundoff floor 分别形成相同的 cross term
`2*rho_i*ratio` 和 square term `ratio^2*beta_p`。显式各计算一次并按原左结合顺序组成
value，同时用嵌套 `std::max` 取同一四项集合；IEEE finite 正常路径的 value 与 floor
逐位相同，非 finite 仍由既有检查拒绝。其它平台保留 long-double 原式。控制变量
`MIPSOLVERS_DS_REPEATED_DSE_TERMS=1` 保留旧的重复表达式。若编译器没有跨
long-double/value/floor 边界公共子表达式消除，每行删除四次乘法及 initializer-list
循环，预测 recurrence/transaction 下降 8--20%、DSE 下降 6--14%；若 MSVC 已完全
CSE，则收益会低于 3%，按预测下界视为失败并删除。固定 A/B、8572 pivot path、目标、
残差和完整 suite 门不变。

首轮 shared-term A/B 的方向正确但幅度触发大幅正向 mismatch。控制报告
`reports/pilot87_dse_repeated_terms_control_r1.{json,log}` 为 recurrence `1.7665s`、
DSE `2.49s`；默认 `reports/pilot87_dse_reused_terms_r1.{json,log}` 为
`0.1014s/0.81s`，分别下降 94.3%/67.5%，远超 8--20%/6--14% 预测。两臂仍为
8572 pivots、相同 objective/residual 且 accurate，故实现保真度成立；其它 factor、
PRICE、entering bucket 只在约 0--4% 范围变化，不能解释约 1.67s。当前机器成本模型
漏掉的是 MSVC 对原 `std::max(initializer_list)` 和 long-double 表达式边界生成的
逐元素临时/比较代码：显式顺序 max 不只是删除四次乘法，而是消除了该抽象开销。
在默认-first 的反序 A/B 重现约 1.5s 以上 bucket 差前不接受该收益；若重现，则以
“等价顺序 max 的编译器代码生成”修正理论，而不再以标量 FLOP 数估计该路径。

反序 A/B 重现并接受修正后的 DSE 模型。默认/控制报告
`reports/pilot87_dse_reused_terms_reverse_r1.{json,log}` 与
`reports/pilot87_dse_repeated_terms_control_reverse_r1.{json,log}` 均为 8572 pivots、
相同 objective/residual 且 accurate；recurrence 为 `0.1027s/1.7537s`，稳定相差
`1.6510s`，DSE 为 `0.82s/2.47s`。wall 为 `6983.86/8611.16ms`，默认快 18.9%，
与目标 bucket 的 1.65s 差一致；其它主要 bucket 基本持平。最终接受依据是两种运行
顺序都重现 94% 级 recurrence 降幅、bit-identical pivot path 和 accuracy；原 FLOP
预测作废，根因修正为 MSVC initializer-list max 代码生成。默认只在
`long double==double` 时复用 terms，宽 long-double 平台继续走原公式。

扩大 Native-only presolve cohort 的筛选报告为
`reports/native_ipm_adaptive_large20_r1.{json,log}`。两臂均 20/20 success/accurate，
但 Native presolve 几何均值 `130.767ms` 对 direct `123.852ms`，整体退化 5.58%，
已直接失败“几何均值不退化”门。收益例包括 `80bau3b` -42.5%、`cycle` -50.1%、
`greenbeb` -32.6%、`d2q06c` -15.5%，说明策略确实覆盖多个结构族；但 loser 同时存在：
`czprob` +96.5%、`ship08l` +41.4%、`ship12l` +53.7%、`woodw` +133.8%、
`fit2p` +19.2%、`bnl2` +21.1%。

反序 repeat-3 报告 `reports/native_ipm_adaptive_losers_reverse_r3.{json,log}` 仍为
36/36 accurate，并确认主要 loser 不是首次初始化：按中位数，`czprob` 约
`58.62/33.43ms`（+75%）、`ship08l` `38.72/28.19ms`（+37%）、`ship12l`
`48.59/31.56ms`（+54%）、`fit2p` `598.84/509.24ms`（+18%）、`woodw`
`274.67/113.71ms`（+142%）；`bnl2` 两臂波动重叠，不用于判定。loser 特征分成两类：
`fit2p` 的 22.2% 列缩减仍花 `129ms` presolve，收益不足摊销；`czprob/ship*/woodw`
虽有 22--52% 结构缩减，但 presolve 本身占 `17--181ms`，且 `ship*/woodw` kernel
iterations 从 direct 的 `13/10/19` 增到 `14/13/27`，缩减后的 barrier 轨迹反而更差。
这不是单一 reduction-rate 阈值可以可靠区分的现象。

因此本轮明确保持 Auto Native-presolve 路由关闭：准确性门通过，但几何均值与稳态
loser 两个性能门失败。下一轮 selector 需要在运行前可得的结构特征之外加入 presolve
预算/中止机制，且对 ship/wood 型模型需要预测 reduced KKT 条件与 iteration inflation；
在该模型经独立 cohort 验证前，不用本轮 20 例反向拟合新阈值。

最终验证环境为 base commit
`a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树，MSVC 19.44
Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）。默认、
`MIPSOLVERS_DS_ACCESSOR_PRICE=1`、`MIPSOLVERS_DS_REPEATED_DSE_TERMS=1`
三条完整 `test_dual_simplex` 均为 79 cases / 23124 assertions；
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`
为 19/19 passed，real time `245.00s`。完整构建命令为先调用 VS 2022
`VsDevCmd.bat -arch=x64`，再执行
`cmake --build build/windows-msvc-release --config Release --parallel 8`。

### 8.24 P2 机会质量门与 P1/P3 直接发布（实现前算法卡，2026-08-19）

§8.23 的扩大 cohort 否定了“P1/P3 缩减大即可摊销 P2 replay”的成本模型。A&A
(1995) §2.4 保证 substitution 等价，但 IPM 成本近似为
`T_presolve + k * (T_factor(m,pattern) + T_assemble(nnz))`；只有 P2 能进一步降低行维度
或显著稀疏化 pattern 时，map-backed adjacency 与 barrier 轨迹分叉才值得承担。当前 gate
把 `max(row_cut,col_cut)>=10%` 当作 P2 收益证据，实际却只是 P1/P3 已经有效的证据：
`80bau3b/ship08l/ship12l` replay 后 P2 消元数均为零，`woodw` 只有
4 doubleton + 28 singleton 消元，却额外花约 148ms；这些模型应直接发布决策预跑产生的
P1/P3 reduced LP。

控制报告 `reports/native_ipm_p1p3_opportunity_probe_r1.{json,log}` 表明关闭 P2 后尺寸、
目标和 accuracy 保持有效，并把 `80bau3b/ship08l/ship12l/woodw` presolve 分别降到
`3.6/2.6/3.5/7.0ms`。反序 repeat-3
`reports/native_ipm_p1p3_losers_reverse_r3.{json,log}` 中，P1/P3-only 的中位 wall
相对 direct 为：`czprob 21.07/28.75ms`、`ship12l 44.94/48.76ms`、
`woodw 197.22/223.43ms`；`fit2p` 不发布 reduced solve，稳态与 direct 重合。
因此先前 loser 主要来自不必要的 P2，而不是 P1/P3。

新的固定策略把 P1/P3 缩减只用于最终 publication gate，不再触发 P2。P2 replay 仅在
P1/P3 fixpoint 后 active singleton-equality density `d=s_eq/n_a` 落在闭区间
`[0.25,0.90]` 时启用。25% 下界沿用 §8.22 的直接 substitution cohort：它保留
`maros-r7` 的 `d=66.7%`，该例 P2 将行/列/nnz 降低约 31/44/32%；低于下界的
`greenbea/greenbeb` 均先发布已证明有正收益的 P1/P3，避免用几乎相同的 5.7% density
错误预测相反 barrier 轨迹。90% 上界排除 `fit2p` 的 99.8% 饱和模式：13500 个
singleton columns 聚集在 3000 个宽 equality rows，P2 只删 3000 列、0 行和约 6% nnz，
不降低 normal-equation 维度，却花 129--189ms。该上界是 replay 策略，不改变任何
reduction 公式；控制环境 `MIPSOLVERS_NATIVE_PRESOLVE_ADAPTIVE_STAGING=0` 仍可强制
旧 eager P2。

实现前固定验证：扩大 20 例两臂必须全部 accurate，Native presolve 相对 direct 的 wall
几何均值必须从 §8.23 的 +5.58% 变为至少 -5%；`woodw/czprob/ship08l/ship12l/fit2p`
必须跳过 P2，反序稳态退化不超过 5%；`maros-r7` 必须继续 replay P2、最终尺寸保持
`2152x5288/98334`；`greenbea` 改走 P1/P3，仍须 accurate 且相对 direct 至少快 40%。
focused test 固定覆盖 100% singleton saturation 被拒和 50--80% cohort 被接受；完整
presolve/IPM/full suite 通过。若方向错误或相对预测偏差超过 50%，按 gate 实现保真度
-> P1/P3 compact/replay 所有权 -> barrier iteration inflation -> 阈值理论顺序调查并先回写。

实现后的扩大 cohort 报告为
`reports/native_ipm_p2_quality_large20_r1.{json,log}`：两臂均 20/20 success/accurate，
Native presolve 几何均值 `165.883ms` 对 direct `194.351ms`，快 14.65%，通过预设的
至少 5% 门。原稳态 loser 的反序报告
`reports/native_ipm_p2_quality_losers_reverse_r3.{json,log}` 为 30/30 accurate；
`czprob/ship08l/ship12l/woodw` 均不再退化，但 `fit2p` 中位
`437.00/403.59ms`（+8.28%）超过 5% 门，并相对预测上限偏差超过 50%，触发 mismatch
调查。

实现保真度检查排除了错误 replay：
`reports/native_ipm_p2_quality_fit2p_diag_r1.{json,log}` 明确记录 `p2=0`、singleton
density `0.998152`、`insufficient_reduction`，P1/P3 扫描仅 2.7ms；两臂保持同一 15
轮、目标与原模型残差，未进入 reduced solve。反向批次的退化也未在正向 repeat-3
`reports/native_ipm_p2_quality_fit2p_forward_r3.{json,log}` 重现：Native/direct 中位为
`458.54/452.07ms`（+1.43%）。两种运行顺序的中位比取对称几何平均约 +4.8%，落回
预设 5% 门内；而实际 presolve 扫描只占约 0.6--1.0%，其余差值来自同一 IPM 15 轮在
约 400--500ms 区间的机器频率/执行顺序波动。故不修改 90% 上界；最终 Auto 判定仍须
等待 `greenbea/maros-r7` 固定门和完整 suite，且报告中保留该边界噪声而不把单向
+8.28% 解释成算法收益或损失。

锚点报告 `reports/native_ipm_p2_quality_anchor_r3.{json,log}` 随后通过剩余固定门：
`greenbea` 走 P1/P3-only，3/3 accurate，中位 `3.854s` 对 direct `11.829s`，快
67.4%；`maros-r7` 走 P2 replay，3/3 accurate，最终尺寸逐项保持
`2152x5288/98334`，中位 `1.267s` 对 `1.959s`。至此 §8.23/§8.24 的 Auto
前置门全部满足。

Auto 激活仍使用并发 portfolio 的 `min(T_DSE,T_IPM)` 模型（§3.4），只把 IPM 臂从
direct 改为上述 adaptive Native presolve；小模型、超盒、无缩减和不足缩减均继续求解
原模型。按 20 例配对结果，预先固定预测为 Auto 的 IPM 赢家子集几何均值至少改善
5%，不新增 accuracy failure/timeout，任何确认的稳态 loser 不超过 5%；dual 赢家路径
不变。验证先跑 `native-auto` 的同一 20 例 repeat-1，再跑 selector/IPM/presolve 聚焦
测试与完整 `ctest`。若 Auto wall 因双线程内存带宽竞争未达到 5%，不反向调整 presolve
阈值；先区分 IPM 是否成为赢家、取消延迟和 portfolio 竞争成本，并回写后再决定回退
路由。

### 8.25 Auto 并发路径的只读机会预检（实现前算法卡，2026-08-19）

Auto 激活后的 20 例 A/B 主门方向正确：
`reports/native_auto_adaptive_large20_r1.{json,log}` 为 20/20 accurate、几何均值
`127.298ms`，旧 direct-IPM 控制
`reports/native_auto_direct_control_large20_r1.{json,log}` 为 20/20 accurate、
`146.605ms`，新路由快 13.17%。但 winner trace 的边界 repeat-3
`reports/native_auto_{adaptive,direct_control}_boundary_r3.{json,log}` 暴露两个稳态
loser：均由 Native IPM 获胜的 `d2q06c` 为 `354.68/330.35ms`（+7.4%），`fit2d`
为 `151.95/136.53ms`（+11.3%），违反“不超过 5%”门并触发 mismatch。

实现保真度与成本分解表明两例都没有进入 reduced solve：`d2q06c` P1/P3 只得到
3.4% row、0.2% column 缩减，`fit2d` 完全无缩减；两者保持与 direct 相同的
24/15 IPM iterations、目标和残差。损失来自在必然回退前构造完整 merged CSC+CSR
工作区并执行 fixpoint，presolve 遥测分别约 4--9ms；在 portfolio 中它与另一线程的
HiGHS presolve/dual kernel 争用内存带宽，因此 wall 放大到 7--11%。故根因是原成本
模型漏掉并发工作集竞争，而不是 substitution 公式、P1/P3 compact 所有权或 barrier
轨迹。

新策略依据 A&A (1995) §2.1--§2.4 在 `build_work` 前做一次只读原 CSC 预检，只统计
无需数值推演即可证明的初始机会：empty/singleton rows、empty/fixed columns，以及
singleton-equality columns。令
`s=max((n_empty_row+n_singleton_row)/m,
       (n_empty_col+n_fixed_col)/n)`；仅当 `s>=0.05`，或 singleton-equality density
仍落在 §8.24 的 `[0.25,0.90]` 时构造完整工作区。5% 是最终 10% publication 门的
一半，给级联消元保留 2 倍放大余量；它不是 reduction 正确性门，拒绝只会走原模型。
预检使用 exact-zero 结构度和现有 `kFixedTol`，不改变任何消元公式、容差或 postsolve。

固定预测与验收：只读预检耗时低于原完整构造的 20%；`d2q06c/fit2d` 报告
`low_opportunity`，Auto repeat-3 相对 direct-IPM 控制退化均不超过 5%；20 例 Auto
几何均值仍至少快 5% 且全部 accurate；`80bau3b/cycle/czprob/greenbeb/ship08l/
ship12l/woodw` 继续发布 reduced solve；`maros-r7` 继续 P2 replay 并保持
`2152x5288/98334`。若任一已知 winner 被预检拒绝，先检查 degree 统计是否与 merged
CSC 一致，再检查 2 倍级联余量假设；在回写 mismatch 前不降低阈值。

实现保真审计 `reports/native_ipm_opportunity_gate_audit_r1.{json,log}` 否定了该预检
成本模型。目标 loser 确实被廉价拒绝：`d2q06c` 为 seed 3.36%、0.285ms，`fit2d`
为 seed 0、1.015ms，相对原完整构造约 4.4/9.2ms，预检成本分别约 6.5%/11.0%，
满足成本预测。但已知 winner `greenbeb` 也以 seed 3.09% 被拒，`woodw` 更只有
0.09%；两者先前最终分别缩减约 18/23% 和 35/36% rows/columns。degree 统计与 merged
CSC 一致，错误来自 2 倍级联余量假设：这些缩减主要由 P3 implied-bound tightening
把非固定列变成固定列后级联产生，初始 empty/singleton/fixed 度不是其下界。

该偏差方向错误，故 5% 预检及 Auto 激活均撤回，不用本 cohort 反向降低阈值或拟合
singleton-density 特例。保留 §8.24 已通过的 standalone staged P2 质量门；Auto 的
IPM 臂恢复 direct。下一轮若重开 Auto，必须先推导一次只读 row-activity 投影对
P3 bound-fixing potential 的估计，并在独立 cohort 验证能同时区分 `greenbeb/woodw`
与 `d2q06c/fit2d`，再承担并发竞速成本。

本轮最终环境为 base commit
`a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树，MSVC 19.44 x86-64
Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）。20 例主命令为
`tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases
fit2d,fit2p,wood1p,woodw,d6cube,d2q06c,80bau3b,greenbeb,degen3,cycle,nesm,bnl2,
pilot,pilotnov,ship12l,ship08l,fit1d,czprob,sierra,25fv47 --solvers
native-ipm,native-ipm-direct --repeat 1 --time-limit 30 --max-iterations 2000`；Auto
A/B 使用同一 case/limit，控制臂设置 `MIPSOLVERS_NATIVE_PRESOLVE=0`。最终保留代码的
focused P2 policy 为 2 cases / 8 assertions；默认、PRICE accessor 控制、repeated-DSE
terms 控制三条 `test_dual_simplex` 均为 79 cases / 23124 assertions；完整
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure` 为 19/19，
real time 191.83s。`git diff --check` 与新增占位符扫描通过。

### 8.26 P3 row-activity/bound-fixing potential（实现前算法卡，2026-08-19）

§8.25 证明初始 structural degree 不是 P3 级联的下界。本轮不再用模型族或经验密度
拟合，而是直接计算 A&A (1995) §3、Achterberg et al. (2020) §6.4 的一次只读
Jacobi row projection。对原始 box `[l_j,u_j]`，第一遍 CSC 为每行计算有限活动和无穷
贡献计数 `L_i,U_i,n_i^L,n_i^U`；第二遍对每个 `a_ij` 用去掉 j 的残余活动
`L_{i,-j},U_{i,-j}`，按现有 P3 的同一公式生成候选：

`a_ij>0:  l^i_j=(lhs_i-U_{i,-j})/a_ij,
           u^i_j=(rhs_i-L_{i,-j})/a_ij`；
`a_ij<0` 时交换 row side。只有对应残余活动有限时才生成该侧。跨行只读聚合
`hat_l_j=max(l_j,l^i_j)`、`hat_u_j=min(u_j,u^i_j)`，不把候选写回其它行，因此结果
不依赖扫描顺序，也不改变 LP。若
`hat_u_j-hat_l_j <= kFixedTol*max(1,|hat_l_j|,|hat_u_j|)`，记为一个 projected fixed
column；候选越界或非有限只使 estimator 标记 invalid，不参与求解结论。

同时在同两遍中统计初始 empty/singleton rows、empty/fixed columns 和
singleton-equality columns。令
`q=max((empty_row+singleton_row)/m,
       (empty_col+fixed_col+projected_fixed)/n)`。
Auto 只在 `q>=0.05`，或 singleton-equality density 落在 §8.24 的 `[0.25,0.90]`
时为 IPM 臂开启完整 staged presolve；5% 仍是 10% publication 门的一半，只控制成本，
不改变任何 reduction/tolerance/postsolve。显式环境变量继续有最终覆盖权。

成本模型：estimator 只分配 `2 double+2 int` 每行和 `2 double` 每列，顺序读取原 CSC
两次；完整 `build_work` 还要复制 merged CSC、构造/填充 CSR 并运行 P1/P3。因此预测
estimator wall 不超过完整无收益 presolve 的 35%，`fit2d<=3.2ms`、`d2q06c<=1.6ms`。
分类预测固定为 `greenbeb/woodw q>=5%`、`d2q06c/fit2d q<5%`；`maros-r7` 由 P2
density 接受、`fit2p` 的 >90% 饱和密度拒绝。

验证先于 Auto 接线。独立 holdout 固定为未参与阈值形成的
`agg,boeing2,ganges,maros,perold,scfxm1,scfxm2,scfxm3,scsd8,sctap2,standmps,
stocfor1`。先用 benchmark telemetry 对 estimator 决策与完整 P1/P3 是否最终达到
10% publication cut 做混淆矩阵：四个目标模型必须全分对，holdout 不得出现“拒绝但
真实 row/column cut>=10%”的假阴性；允许假阳性，因为后置 publication gate 仍回退
direct。通过后才把 Auto IPM 臂设为 estimator 决策，并固定复测：边界 repeat-3 相对
direct 控制退化不超过 5%，20 例 Auto 几何均值至少快 5%，全例 accurate；完整 suite
通过。若 projected fixing 仍漏掉 `woodw/greenbeb`，依次检查 residual-infinity 计数、
row-side 合并、Jacobi 对 Gauss--Seidel 级联的保守性；先回写，不增加第三个经验特征。

首个目标探针先暴露实现忠实度缺陷：单侧无界列的 scale 为无穷，导致
`inf <= kFixedTol*inf` 被误判为 fixed；singleton-equality 统计还多做了第三遍 CSC。
修正为 original/projected 两侧均有限才判 fixed，并在第一遍保存 singleton row 后，
`reports/native_auto_p3_estimator_targets_fixed_r1.{json,log}` 的分类预测全部命中：
`greenbeb q=10.31%`、`woodw q=25.03%` 接受，`d2q06c q=3.36%`、
`fit2d q=0` 拒绝；`maros-r7` 由 66.67% P2 density 接受，`fit2p` 以
99.82% 饱和 density 拒绝。合成 opportunity 测试为 2 cases / 8 assertions。

但成本预测发生 >50% mismatch，因此 Auto 仍不得接线。目标单次探针中 `d2q06c`
为 1.202ms，满足 1.6ms 门；`fit2d` 为 5.709ms，超过 3.2ms 门 78.4%。重复五次报告
`reports/native_auto_p3_estimator_cost_repeat5_r1.{json,log}` 进一步给出 `d2q06c`
1.178--1.572ms、`fit2d` 6.375--7.900ms，排除冷启动噪声。实现审计确认修正后严格
只有两遍 CSC；两例耗时随 nnz 从 32417 到 129018 近似四倍增长。故失配位于机器/
成本模型：原模型只计入顺序读和工作区字节，遗漏第二遍每个非零的 residual-infinity
分类、row-side 分支、除法及 projected-bound 聚合；`fit2d` 的 25x10500 宽矩阵仍要
对 129018 个非零执行这些算术，估计器已接近 §8.25 的完整无收益 presolve 成本，
不适合直接放入并发 portfolio。下一步只能推导有证明的 row-level 候选上界以跳过
不可能产生 projected fixing 的行；在该上界、独立 holdout 和原性能门全部通过前，
保持 `opt.presolve=false`，不得用本目标集反向放宽成本门或拟合新阈值。

### 8.27 P3 row-span 必要条件（实现前算法卡，2026-08-19）

§8.26 的第二遍可先用同一 row-activity 公式导出逐行必要条件，而不引入经验特征。
对活动均有限的行，定义列 j 在该行上的 box activity span
`d_ij=|a_ij|(u_j-l_j)`、`D_i=max_j d_ij`。由 A&A (1995) §3 的 residual 公式，
上侧 `a_i x<=rhs_i` 对任一列产生严格收紧的必要条件为

`rhs_i-L_i < d_ij`，

因为 `L_i=L_{i,-j}+min(a_ij l_j,a_ij u_j)`；下侧同理为
`U_i-lhs_i < d_ij`。所以当 `rhs_i-L_i>D_i+tau_i` 时可证明上侧不会产生任何
candidate，当 `U_i-lhs_i>D_i+tau_i` 时下侧也不会产生 candidate；`tau_i` 使用现有
`row_tol(..., kDeleteTol)` 吸收活动求和舍入。等号或容差带内必须保守执行，以保留 forcing
row。若对应活动有一个无穷贡献，唯一贡献列仍可能得到有限 residual，保守执行；有两个
及以上无穷贡献时该侧对所有列都不可用，直接跳过。这只是从 §8.26 精确投影前删除已证明
为空的行，不改变公式、阈值、扫描顺序或 LP。

第一遍额外维护每行一个 `double D_i`，成本为每个非零一次绝对值、乘法和 max；随后只要
任一行可投影才进入第二遍 CSC，且第二遍对不可投影行立即返回。`fit2d` 的 MPS bounds
审计显示 10500 列均为有限 `[0,ub]`，完整 presolve 和 §8.26 均观测到 25 行、0 次
tightening，故固定预测为 projectable rows=0、完全跳过第二遍，repeat-5 estimator
中位不超过原 3.2ms 门。`d2q06c` 仍不超过 1.6ms；额外第一遍算术不得使其超过该门。
六个目标的 accept/reject、q、projected-fixed 计数必须与 §8.26 修正版相同；forcing
合成例 projectable rows=1000 且 projected fixed=4000，nonforcing 饱和例
projectable rows=0 且仍拒绝。若成本仍超门，先检查第二遍是否确实未进入，再检查第一遍
span 算术/计时占比；若任何分类或计数变化，视为必要条件实现不忠实并撤回，不调容差。
通过目标门后仍必须执行 §8.26 预注册的独立 holdout，才允许接通 Auto。

首轮 `reports/native_auto_p3_row_span_targets_r1.{json,log}` 满足实现忠实度：六例
`q/projected_fixed/tightening` 与 §8.26 逐项相同，`fit2d` projectable rows=0，forcing/
nonforcing 合成测试为 2 cases / 10 assertions。成本 repeat-5
`reports/native_auto_p3_row_span_cost_repeat5_r1.{json,log}` 中 `d2q06c` 中位
1.552ms，通过 1.6ms 门；`fit2d` 中位 3.445ms，仍比 3.2ms 门高 7.7%。方向正确且
偏差未达 50%，但严格成本门尚未通过。审计确认第二遍确实未进入，剩余偏差在第一遍：
代码先分别计算 `p_min=a*min_bound`、`p_max=a*max_bound` 用于活动和，又额外计算
`|a|(u-l)` 用于 span。由同一定义 `d_ij=|p_max-p_min|`，可直接复用前两个乘积，
每个有限 box 非零删除一次乘法且数学结果相同。固定预测：`fit2d` repeat-5 中位进入
2.8--3.2ms，`d2q06c<=1.6ms`，projectable rows 与所有分类/计数不变；若仍超门则停止
局部算术优化并保持 Auto 关闭。

### 8.28 Holdout 尺寸遮蔽与补充验证（结果前固定，2026-08-19）

§8.26 预注册的 12 例 holdout 在生产 staged 配置下全部命中既有
`m+n<4000` small-model gate，`reports/native_auto_p3_estimator_holdout_r1.{json,log}`
因此给出 12 个 `small=1/run=0` 和 0 最终缩减。按字面“最终缩减”没有假阴性，但这组
数据对 estimator 没有辨别力，不能作为打开 Auto 的证据。为审计标签而关闭 staging 的
`reports/native_p3_estimator_holdout_ground_truth_r1.{json,log}` 又证明不能把它们解释成
“无机会”：例如 `agg` 为 488x163 -> 175x106，`ganges` 为 1309x1681 -> 765x1064，
两者真实结构缩减均远超 10%。根因是验证 cohort 与早先独立制定的尺寸成本策略冲突，
不是 row-span 或 P3 residual 公式失效；不据此删除 small gate，也不把尺寸当新分类特征。

在查看任何新 estimator 结果前，补充 `m+n>=4000` cohort 固定为
`80bau3b,bnl2,cycle,czprob,degen3,dfl001,nesm,pilot,pilotnov,ship08l,ship12l,
stocfor2`。排除与目标同族的 `greenbea` 和反复优化过的 `pilot87`；其中前 10 例部分已
用于 §8.24 的整体 presolve 性能 cohort，故这是对当前 estimator 未调参的补充验证，
而不宣称全项目历史上的完全未见数据。使用生产 staged `native-ipm` verbose 作为最终
row/column cut 标签、`native-auto` debug 作为只读决策；任何实际 max(row cut,col cut)
达到 10% 而 estimator 拒绝的案例都否决 Auto 接线。不得根据该 cohort 调 5% 阈值、
row-span 容差或增加经验特征；若出现假阴性，只记录 residual/Jacobi 机制并保持 Auto
关闭。只有零假阴性、全部 accuracy、目标成本门仍通过，才进入边界 repeat-3 与 20 例
Auto A/B。

补充验证 `reports/native_auto_p3_estimator_large_holdout_r1.{json,log}` 使用 commit
`a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树、MSVC 19.44 Release、
repeat 1。达到 10% publication cut 的五例全部接受：`80bau3b` 最终 row/col cut
10.65%/6.16%，`cycle` 19.71%/11.45%，`czprob` 25.83%/21.37%，`ship08l`
33.16%/26.48%，`ship12l` 40.31%/22.17%。`dfl001` 的 0/0 和 `stocfor2` 的
1.30%/0.79% 被拒绝；`bnl2/pilot` 为允许的假阳性，最终分别只有 9.12%/1.63% 和
3.96%/7.37%。其余三例命中生产 small-model gate。故有标签的大模型为 0 假阴性，
两臂均 12/12 accurate；§8.27 的目标成本门也保持通过。按预注册流程，允许把
`valid && should_run` 接入 Auto IPM 臂的正常和同步 fallback 路径，环境变量仍在
`lp_presolve_config_from_env` 中拥有最终覆盖权。接线后的性能结论必须由边界 repeat-3
和 20 例 A/B 决定，不能用本 repeat-1 holdout wall 宣称收益。

### 8.29 Auto 接线验收（2026-08-19）

最终实现把 estimator 的 `valid && should_run` 同时接入 Auto IPM worker 与线程创建失败
时的同步 IPM fallback；`lp_presolve_config_from_env` 仍在内部最后执行，因此显式
`MIPSOLVERS_NATIVE_PRESOLVE=0/1` 保持最终覆盖。有限 bound 的 row activity 求和溢出或
投影候选越出可表示 row-side 域时 estimator fail closed（`valid=false`），不静默丢弃
candidate；focused overflow/forcing/nonforcing 为 3 cases / 12 assertions。

环境为 base commit `a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树，
MSVC 19.44 x86-64 Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）。§8.27 最终成本报告
`reports/native_auto_p3_row_span_reuse_cost_repeat5_r1.{json,log}`：`fit2d` 中位
2.297ms（预测 2.8--3.2ms，优于上界）、`d2q06c` 中位 1.362ms（预测 <=1.6ms）；
分类和 projected 计数与未剪枝的 §8.26 精确投影相同。最终单次目标复核
`reports/native_auto_p3_estimator_final_targets_r1.{json,log}` 仍为
`greenbeb/woodw/maros-r7` 接受、`d2q06c/fit2d/fit2p` 拒绝，6/6 accurate。

关闭全部 debug 的公平边界 A/B 使用同一二进制、repeat 3、30s、2000 iterations，控制
臂仅设置 `MIPSOLVERS_NATIVE_PRESOLVE=0`。报告
`reports/native_auto_p3_estimator_{,direct_control_}boundary_clean_r3.{json,log}` 的
默认/控制中位为 `d2q06c 309.96/325.74ms`、`fit2d 131.91/132.52ms`、
`fit2p 472.85/513.23ms`、`sierra 31.61/31.05ms`；最差退化 1.8%，满足预测的 <=5%，
12/12 accurate。带 debug 的早期边界报告只用于确认决策，因两臂观测开销不对称不用于
性能结论。

固定 20 例 Auto 主门用相同 flags、repeat 1：
`reports/native_auto_p3_estimator_large20_r1.{json,log}` 为 20/20 accurate、几何均值
135.381ms；同二进制 direct 控制
`reports/native_auto_p3_estimator_direct_control_large20_r1.{json,log}` 为 20/20 accurate、
154.396ms。Auto 改善 12.3%，超过预注册的至少 5% 门。故本节结论是：只读 P3/P2
机会估计器已通过分类、成本、holdout、边界与主 cohort 门并在 Auto 中启用；该结论只
说明 Native presolve 路由相对本机 direct Auto 的改进，不等价于“所有 LP 上 Native
均快于 HiGHS”。

最终完整 Release 构建后执行
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure`：19/19
通过，real time 205.85s；包含 `test_lp_presolve`、`test_numerical_stability` 和
`test_netlib_regression`。默认、`MIPSOLVERS_DS_ACCESSOR_PRICE=1`、
`MIPSOLVERS_DS_REPEATED_DSE_TERMS=1` 三条完整 dual-simplex 控制均为
79 cases / 23124 assertions，focused P2 policy 为 2 cases / 8 assertions。

### 8.30 Estimator row activity 直达增量 P3（实现前算法卡，2026-08-19）

**理论模型。** 对活动行 `i` 保存 Andersen & Andersen (1995) §3 与
Achterberg et al. (2020) §6.4 的四元组
`(L_i^f,U_i^f,n_i^L,n_i^U)`：前两项分别是有限最小/最大贡献之和，后两项
是对应方向的无穷贡献数。列 `j` 的 box 从 `(l,u)` 变为 `(l',u')` 时，对
每个 CSC incidence `(i,a_ij)` 定义
`q_L(a,l,u)=a*(a>0?l:u)`、`q_U(a,l,u)=a*(a>0?u:l)`；有限贡献执行
`L_i^f += q_L' - q_L`、`U_i^f += q_U' - q_U`，有限/无穷跨越则在和与计数
之间转移。P3 排除本列后的 residual 仍按 §8.26：仅当
`n_i^L-ownInf_L==0` / `n_i^U-ownInf_U==0` 时可用。该状态只缓存 estimator
第一遍 CSC 的**原始 box activity**；estimator 第二遍 Jacobi 投影仍是路由证据，
不能提前写入正式 bounds。

**生命周期与正确性。** Auto 将同一次估计所得 activity snapshot 随 IPM arm
传入 `lp_presolve_run`。接收端核对行列数、结构非零数以及原始列 bounds；不匹配
就放弃 snapshot 并从已建 CSR 初始化，不把陈旧缓存用于约简。`row_sweep` 和 P3
每次正式收紧一列 bounds 后立即沿该列 CSC 更新所有活动行；`col_sweep` 删除固定列
时在行侧平移后移除该列贡献。P2 会改系数、fill 和 row side，因此
`commit_mutable_adjacency` 后无条件从新 CSR 重建 activity，避免对 fill-in 做未经验证
的局部维护。行删除不改变别行 activity。公式与 infinity-count 转移均直接来自上述
row activity 定义；代码现场引用本节。

**数值语义。** estimator 第一遍按列递增、每列内按行递增；`build_work` 生成的
CSR 每行同样按列递增，因此初始有限和的加法次序一致。后续差分更新改变舍入历史，
故这是算法数值变更而非纯性能变更，必须走完整 original-model accuracy gate；P3
投影的冲突与 tightening tolerance 不变。任何差分溢出把该方向标为不可投影，并在
下次结构重建时恢复，而不从非有限值推断可行性。

**成本模型与固定预测。** 旧 P3 每次 sweep 的 activity build 与 projection 各读
一次活动 nnz，成本近似 `2*N_s`；新路径以 estimator 已付出的第一遍代替首次 build，
正式 presolve 只付 projection `N_s`，之后每个真正变化列付 incidence 总和 `D`，
P2 后至多付一次 `N_s` 重建。对 Auto 接受且 P2 不触发的 greenbeb/woodw/maros-r7，
预测 P3 activity-build 读流减少至少 80%，端到端 `estimator + presolve` 中位数减少
5--15%；reduced 尺寸、规则计数和路由分类不变。对 P2 replay cohort，允许因一次
重建仅减少 0--8%，但不得慢于旧路径超过 3%。

**结果前固定的验证协议。** (1) focused P3 测试增加跨 `-inf/finite`、
`finite/+inf`、同列连续收紧及 P2 后重建情形，并比较冷初始化与 snapshot 注入的
status、约简维度、规则计数可见结果及 postsolve primal；(2) Release 下运行
`test_lp_presolve`、`test_numerical_stability`、`test_netlib_regression` 与完整 `ctest`；
(3) 固定 Auto cohort 和 greenbeb/woodw/maros-r7 重测，报告命令、MSVC flags、commit
与 measured-vs-predicted；(4) snapshot mismatch 必须自动回退且结果等同冷路径；
(5) 禁止标记扫描与 `git diff --check`。若方向相反或偏离预测超过 50%，按
implementation fidelity -> machine/cost model -> assumption -> theory 的顺序定位，
先把原因写回本节，再继续做 presolve/kernel 矩阵重复物化。

### 8.31 Presolve 到 IPM kernel 的 resident CSC/CSR（实现前性能卡，2026-08-20）

**数据模型。** P1/P2/P3 fixpoint 后，活动列已经按原列顺序遍历，且每列的活动行
按原 row id 递增；`row_map_a/row_map_eq` 又各自保持序。因此 reduced `A/Aeq` 的
CSC 可用 Eigen 的 ordered `startVec + insertBackByOuterInner` 直接构造，无需
`Triplet` 暂存、通用排序和 duplicate merge。与此同时从 `PresolveWork::csr_*`
按活动行顺序压紧两块 CSR，列号经 `orig_to_reduced_col` 映射。该 CSR 作为
`LpPresolveResult` 的 immutable shared workspace 随 reduced model 进入 IPM。

**kernel 语义。** 未缩放、未翻转行时直接引用 resident `row_ptr/col_idx/value`。
Ruiz scaling 不改变稀疏结构，故仍直接引用 `row_ptr/col_idx`，只按
`v'_ij = dr_i * v_ij * dc_j` 生成 values；lower-only inequality normalization 同理
只额外乘该行的 `-1`。此式来自 §7 的对角等价变换，也与
`ruiz_equilibrate` 当前逐 entry 更新的最终值定义相同。workspace 接收端核对
`rows/cols/nnz` 与 CSC；任何不匹配退回现有 `build_csr`，绝不影响求解正确性。

**成本模型与预测。** 旧路径在 compaction 产生约 `N_r` 个 Triplet，再由 Eigen
读写/排序为 CSC；kernel 又执行 CSR count + prefix + scatter（约 `2*N_r + m` 的
结构流）。新路径 compaction 写一次 CSC 与一次 CSR，kernel 无缩放时零结构复制，
缩放时仅写 `N_r` values。预测 Auto 接受 cohort 的 presolve compaction bucket
下降 20--45%，kernel CSR bucket 下降至少 70%，端到端 `estimator + presolve +
kernel setup` 中位数再下降 3--10%；reduced CSC 的 outer/inner/value 序列、目标值、
迭代数和 original-model audit 必须不变。若端到端方向相反或收益偏离预测超过 50%，
先按 §8.30 的 mismatch 顺序写回原因，不继续扩大复用范围。

**验证协议。** 新增测试逐项比较 direct-ordered CSC/CSR 与 reduced Eigen matrices，
并覆盖 ranged/equality split、P2 fill 和空 block；运行完整 Release presolve、数值稳定、
NETLIB 与 `ctest`。性能报告记录命令、MSVC Release flags、commit，并对
greenbeb/woodw/maros-r7 报 measured-vs-predicted；最后执行禁止标记与 diff 检查。

**首次 mismatch（2026-08-20，implementation fidelity）。** focused presolve 与
numerical-stability 均通过，但第一次增量全量链接后的 `milp_benchmark`、
`native_kernel_comparison` 和 `test_engine_api` 分别以 access violation / heap
corruption 退出。原因不是 resident CSR 算术：实现曾把 activity `shared_ptr` 插入公开
`IPMLPOptions` 中部，而 build graph 未重编所有按值传递该 struct 的对象，跨对象文件
出现新旧布局 ABI 不一致。修复是恢复 `IPMLPOptions` 布局，改用显式
`NativeIPMLPAdapter::solve_lp(prob,snapshot)` 单次调用 overload；这也把 snapshot
所有权限制在 Auto->IPM->presolve 生命周期。按 mismatch 协议，完成全量重新编译并
先重跑上述三个用例后才恢复其余验收。

**干净构建与正确性验收（2026-08-20）。** Ninja/MSVC 的 `/showIncludes` 输出在本机为
本地化前缀 `注意: 包含文件:`，增量构建没有可靠地重编所有包含公开 struct 的对象；
`LpPresolveResult` 增加 workspace 后，旧对象还使一个 MILP telemetry 用例崩溃。执行
`cmake --build build/windows-msvc-release --config Release --clean-first -j 8` 后，该用例
为 1 case / 37 assertions；随后
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure` 为 19/19
通过，real time 191.98s。默认、`MIPSOLVERS_DS_ACCESSOR_PRICE=1` 和
`MIPSOLVERS_DS_REPEATED_DSE_TERMS=1` 三条完整 dual-simplex 控制各为 79 cases /
23124 assertions。实现新增的 snapshot 跨有限/无穷贡献转换、陈旧矩阵拒绝、P2
失效重建及 resident CSC/CSR permutation 测试也全部包含在该干净构建结果中。环境为
base commit `a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树、MSVC 19.44
x86-64 Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）。

**性能实测与第二次 mismatch（2026-08-20，machine/cost model -> assumption）。**
目标组命令为
`netlib_solver_benchmark.exe --data-dir tests/data --cases greenbeb,woodw,maros-r7
--solvers native-auto,highs-ipm --repeat 3 --time-limit 30 --max-iterations 2000
--json reports/native_p3_resident_matrix_targets_r3.json`。Native/HiGHS 中位数分别为
`greenbeb 394.395/258.595ms`（1.525x）、`woodw 137.738/153.436ms`（0.898x）、
`maros-r7 948.529/495.116ms`（1.916x）；Native 三例几何均值 364.159ms，HiGHS
281.587ms。与接线前单次报告
`reports/native_auto_p3_estimator_final_targets_r1.json` 的 Native
`343.063/139.118/885.642ms` 相比，`greenbeb/maros-r7` 方向相反，故不能用这组三例
宣称达到 §8.30 的 5--15% 或 §8.31 的 3--10% 端到端预测。

按既定排查顺序，implementation fidelity 证据为：snapshot/cold 两臂的约简维度和
迭代数逐例相同；resident workspace 的 ranged/equality CSC、CSR 与 permutation
逐 entry 测试通过；完整 original-model accuracy 为 20/20。机器/成本模型失配则很
明显：同一二进制三次 `maros-r7` presolve 为 216.009--330.291ms，波动已大于预计
节省；Auto 的 presolve 又与并行 dual arm 竞争资源，不能拿独立 `native-ipm` 冷臂作
严格 wall-time 因果对照。原先“不改变约简尺寸和迭代数”的假设也被增量浮点历史否定：
`greenbeb` 从旧报告的 1949 reduced rows / 59 iterations 变为稳定的 1952 / 58；这是
§8.30 已预警的数值语义变化，不是 snapshot 与冷初始化之间的分叉，且原模型 audit
仍通过。

固定 20 例 Auto cohort 用原 §8.29 命令重测为
`reports/native_p3_resident_matrix_large20_r1.json`：20/20 accurate，几何均值
131.934ms；接线前为 135.381ms，即改善 2.55%。方向正确，但仅能支持“重复物化已被
消除且整体无回退”的保守结论，不能把 0.45 个百分点的低于 §8.31 下界解释成稳定的
kernel setup 收益。冷 activity 控制
`reports/native_p3_cold_activity_control_targets_r3.json` 中，`woodw` presolve 中位
10.409ms 对 snapshot 路径 7.481ms（减少 28.1%），`maros-r7` 为
289.008/282.867ms（减少 2.1%），`greenbeb` 为 11.590/13.166ms（增加 13.6%）；因
两臂执行拓扑不同，这些数字只作命中/量级诊断。后续若继续优化，必须先增加同一 Auto
执行拓扑下的只读 bucket telemetry 或构建期 A/B，分别计量 activity build、compaction
和 kernel CSR structure/value scatter，不能再从总 wall time 反推各 bucket。

### 8.32 PaPILO-style resident P1 degrees/activity（实现前算法卡，2026-08-20）

**近期文献结论。** Gleixner、Gottwald、Hoen 的 PaPILO 论文 v3（2024，
arXiv:2206.10709）§2.1 将 fast presolver 的每轮复杂度写成关于“自上次调用后发生变化的
非零数” `n` 的 `O(n log n)`，并明确长期保存、更新每个约束的 minimum/maximum
activity，以缩小 activity-based presolver 的输入；§2.2 再用 read-only presolver +
deterministic transaction validation 解决并行 reductions 的冲突，而不是复制多份问题或
给细粒度任务加锁。对本仓库的直接含义是：在单线程 fast P1/P3 仍重复全矩阵扫描时，
先引入并行 transaction 层会放大错误的工作量；应先把已 resident 的 activity 与活动度数
用于 P1。Applegate et al.（NeurIPS 2021，arXiv:2106.04756）说明超大 LP 的另一条路线
是以 SpMV 为核心的 PDHG/PDLP，并配合 diagonal preconditioning、presolve、adaptive
steps 和 restart；这支持后续为真正超大且一阶法适用的模型建立 Auto 路由，但不改变
本节对精确 presolve 数据流的选择。HiGHS 的 hypersparse revised-simplex 基础仍来自
Huangfu--Hall (2018)，其核心同样是让工作量随 active support 而非矩阵总规模增长。

**算法与不变量。** 在 `PresolveWork` 中保存 `row_degree[i]` 和 `col_degree[j]`，定义为
当前 active bipartite matrix graph 中端点的度数。初始构建和每次 P2
`commit_mutable_adjacency` 后由 committed CSC 重建；删除活动行 `i` 时，沿其 CSR 对
每个活动列执行 `col_degree[j]--`，再把 `row_degree[i]=0`；删除活动列 `j` 时，沿其 CSC
对每个活动行执行 `row_degree[i]--`，再把 `col_degree[j]=0`。所有递减必须发生在 active
flag 翻转之前且只发生一次，因此度数永不为负，并始终等于 active incidence count。

`row_sweep` 首先保证 resident activity 已初始化。degree 0 直接走 empty-row；degree 1
才扫描该行以取得唯一 `(j,a_ij)`；degree >1 的 infeasibility/redundancy 判定直接读取
`(L_i^f,U_i^f,n_i^L,n_i^U,overflow_i)`，不再重新乘加整行。`col_sweep` 直接读取
`col_degree[j]`；只有实际 fixed-column substitution 才扫描该列做 side safety 和 shift。
公式、容差和删除顺序仍是 A&A (1995) §2.1--2.3；resident activity 的定义与更新为
§8.30。P2 fill 改变图和系数，故 commit 后同时失效 activity、重建 degrees，下一 P1
从 committed graph 继续。该改动把 PaPILO 的“resident activity + changed support”原则
落到现有串行数据流，不在本轮引入 reduction transactions 或并行顺序变化。

**成本模型与固定预测。** 设一轮开始时活动非零为 `N_r`，行列数为 `m_r,n_r`，本轮
被删除端点的 incidence 总数为 `D_r`。旧 P1 row/column passes 至少读取约 `2N_r`，P3
另读 `N_r`；新 P1 的非 P3 工作为 `O(m_r+n_r+D_r)`，仅 singleton/fixed safety 额外读
其实际 support。对多轮且 `D_r << N_r` 的 greenbeb/woodw，预测 presolve 中位减少
12--25%；对 P2/compaction 占比更高的 maros-r7 减少 8--20%；端到端 Auto 中位减少
1--5%。snapshot 与 cold 路径在同一当前二进制上必须给出相同 reduced dimensions、
rule counts 和 IPM iterations。由于 row_sweep 改为读取增量 activity，和旧版逐轮重新
求和的舍入历史不同，本节仍按算法数值变更处理，不要求跨旧二进制 pivot/path 相同。

**结果前固定的验证协议。** (1) focused 测试覆盖 row deletion -> empty column、column
deletion -> singleton/empty row、连续 bound tightening 以及 P2 commit 后 degree/activity
重建；检查 degree 语义通过最终 reductions 和 postsolve，而不暴露可变内部结构；
(2) 运行 `test_lp_presolve`、`test_numerical_stability`、`test_netlib_regression` 和完整
Release CTest，外加三条 dual-simplex 控制；(3) 同一干净 Release 二进制重跑
`greenbeb,woodw,maros-r7` 的 `native-auto` repeat-3，并以
`reports/native_p3_resident_matrix_targets_r3.json` 为接线前机器基线；冷路径以
`reports/native_p3_cold_activity_control_targets_r3.json` 为基线；(4) 重跑固定 20 例
Auto cohort，要求全例 accurate；(5) 报告 base commit、flags、命令以及
measured-vs-predicted，最后执行禁止标记与 diff 检查。若 presolve 中位方向相反或收益
偏离预测超过 50%，先按 degree update fidelity -> activity rounding -> concurrent Auto
machine noise -> cost-model assumption 的顺序回写，再决定是否继续做 dirty-row queue。

**冷路径实测与 wall-time mismatch（2026-08-20）。** 最终 clean Release 二进制的
`native-ipm` repeat-3 报告 `reports/native_p1_resident_degree_cold_targets_r3.json` 相对
本节预注册基线的 presolve 中位为：`greenbeb 11.590->6.899ms`（减少 40.5%，超过
12--25% 预测），`maros-r7 289.008->244.058ms`（减少 15.6%，落在 8--20% 预测内），
`woodw 10.409->8.849ms`（减少 15.0%，落在 12--25% 预测内）。三例
reduced rows/cols/nnz 和 IPM iterations 与基线逐项相同；最终迭代数仍分别为 58、15、
26，故 degree update fidelity 与当前数值路径通过检查。

同一对跨时段报告的端到端中位改善 `greenbeb 29.7%`、`maros-r7 16.7%`、`woodw
17.0%`，全部明显超出预注册的 1--5%；`greenbeb` 的 presolve bucket 也超过预测上界，
按仓库定义属于 >50% prediction mismatch，不能当作算法收益放大。按既定顺序排查：
结构、rule outcome、迭代数和完整测试排除了 implementation fidelity；保存的基线和新
报告不是同一交错执行轮，factorization/kernel 温度与并行背景负载改变，同时
`presolve_ms` 自身也表现出跨轮机器噪声。因此可接受的结论只取三例 presolve 方向一致、
`maros-r7/woodw` 量级命中和结构不变；不能从跨时段 wall time 精确归因本轮收益。后续
性能改动必须增加同一执行拓扑的 build-time A/B 或 bucket telemetry，不能继续把冷、
snapshot 或不同时段报告互作严格对照。

**最终验收与当前竞争位置（2026-08-20）。** clean build 命令为
`cmake --build build/windows-msvc-release --config Release --clean-first -j 8`；随后
`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure` 为 19/19
通过，real time 188.07s。默认、`MIPSOLVERS_DS_ACCESSOR_PRICE=1`、
`MIPSOLVERS_DS_REPEATED_DSE_TERMS=1` 三条完整 dual-simplex 控制各为 79 cases /
23124 assertions。focused `[resident-degree]` 为 1 case / 9 assertions，完整
`test_lp_presolve` 为 34 cases / 1048 assertions，`test_numerical_stability` 为
27 cases / 484 assertions，`test_netlib_regression` 为 3 cases / 294 assertions。
环境是 base commit `a0322e38773b38be91c9b13d8e18c306b948f5e0` + 当前工作树，
MSVC 19.44 x86-64 Release（`/O2 /Ob2 /DNDEBUG /fp:fast`）。

固定 20 例命令为
`netlib_solver_benchmark.exe --data-dir tests/data --cases fit2d,fit2p,wood1p,woodw,d6cube,d2q06c,80bau3b,greenbeb,degen3,cycle,nesm,bnl2,pilot,pilotnov,ship12l,ship08l,fit1d,czprob,sierra,25fv47 --solvers native-auto --repeat 1 --time-limit 30 --max-iterations 2000 --json reports/native_p1_resident_degree_large20_r1.json`；结果 20/20 accurate，
Native 几何均值 110.736ms。目标组三次重复报告
`reports/native_p1_resident_degree_auto_targets_r3.json` 的 Native/HiGHS 中位为：
`greenbeb 331.017/243.603ms`（Native 1.36x），`maros-r7 771.128/441.619ms`
（1.75x），`woodw 102.796/143.730ms`（0.72x）；三例几何均值分别为
303.012/252.430ms（Native 1.20x）。因此 resident P1 方向有效且保持准确性，但 Native
尚未在该目标组总体超过 HiGHS。下一步应先给 `maros-r7` 增加同拓扑 P2、compaction、
Ruiz/value scatter 与 factorization buckets，定位剩余 329.5ms 中可由 presolve 数据流
消除的部分，再决定 dirty-row queue、parallel transaction 或 PDLP Auto route；不能仅凭
矩阵规模先打开这些路径。

### 8.33 PaPILO changed-support dirty queues（实现前算法卡，2026-08-20）

**选择与理论依据。** 用户要求直接引入 parallel transaction、dirty queue 或 PDLP Auto
route。仓库已有 Native PDLP，但 `test_netlib_regression.cpp` 仍把它列为 report-only，
尚无可支持 Auto 接管的鲁棒性门；当前 LP presolver 也没有独立 read-only presolver
调度层，直接增加 parallel transaction 会先支付复制、冲突验证和同步成本。故本轮选择
依赖已经具备且可单独验收的 dirty queue。Gleixner、Gottwald、Hoen (2024)，PaPILO v3
arXiv:2206.10709 §2.1，把 fast-presolver 一轮的输入限制为自上次执行后变化的 support，
给出关于变化非零数 `n_delta` 的 `O(n_delta log n_delta)` 工作模型；Achterberg et al.
(2020) §3.1、§6.4 的活动维护说明，删除端点或改变列界只会使其 incidence 邻域中的规则
状态失效。由此不需要重新检查未接触的 P1 行列。

**算法、不变量与顺序。** `PresolveWork` 保存去重的 `dirty_rows/dirty_cols` 以及 membership
bits。初始时全部活动端点 dirty；删除行 `i` 时只把其活动邻列置脏，删除列 `j` 或改变
其界时只把活动邻行置脏。每次 P1 row/column pass 原子地取走当前集合、按原索引升序
排序并清除 membership bit，因此同一候选集内仍保持旧版升序 reduction 顺序；pass 中
产生的新 dirty item 留给下一次对应 pass。P3 仍保持原有全活动行 sweep，避免本轮把
性能变换同时变成传播闭包算法。P2 commit 可能改变系数和 fill，不能证明局部邻域完整，
故与 resident activity 失效相同，重建 degree 后把全部活动端点重新置脏。任一活动端点
若不在 dirty 集合中，则自上次检查后它的 degree、相邻活动状态和相关列界均未改变，
所以 A&A (1995) §2.1--2.3 的 empty/singleton/redundancy/fixed 判定结果不变。

**成本模型与固定预测。** 旧 P1 每个结构轮访问 `m_r+n_r` 个活动端点；新路径访问
`d_r+d_c` 个去重 dirty 端点，置脏成本为删除/界变化 incidence 总数 `D_r`，排序成本为
`O((d_r+d_c) log(d_r+d_c))`。不改变仍为 `O(N_r)` 的 P3 或 P2/compaction。对有多轮局部
级联的 greenbeb/woodw，预测 P1 端点访问减少 50--90%、presolve 中位减少 10--25%；
对 P2/compaction 占主导的 maros-r7，预测 presolve 中位减少 2--8%；端到端 Auto 只预测
0--3%。无 reduction 的单轮模型允许轻微 queue 初始化开销，但目标是不超过 presolve
中位 3%。

**结果前固定的验证协议。** (1) 新测试构造非连续索引的 row deletion -> dirty column ->
column deletion -> dirty singleton row 级联，检查完整 postsolve；已有 resident-degree、
incremental-P3、P2 commit 测试必须继续通过；(2) focused `test_lp_presolve`，随后
`test_numerical_stability`、`test_netlib_regression`、完整 Release CTest 和三条
dual-simplex controls；(3) clean Release 上以 §8.32 最终报告为机器基线，重跑
greenbeb/woodw/maros-r7 cold repeat-3、Auto/HiGHS repeat-3 和固定 20 例；结构、迭代和
准确性必须一致；(4) 报告 measured-vs-predicted、base commit 和 flags，执行禁止标记与
diff 检查。若 presolve 方向错误或收益偏离预测超过 50%，按 queue completeness ->
排序/reduction 顺序 -> P2 全量失效 -> machine noise -> cost-model assumption 的顺序
调查并先回写本节。

**首次 mismatch（2026-08-20，queue completeness）。** focused tests、完整 presolve、
numerical-stability 和 NETLIB regression 均通过，但首轮 cold repeat-3 相对 §8.32 基线的
presolve 中位为 `greenbeb 6.899->7.897ms`（增加 14.5%）、`maros-r7
244.058->238.115ms`（减少 2.4%）、`woodw 8.849->6.461ms`（减少 27.0%）；三例约简
维度相同，但 greenbeb IPM iterations 从 58 变为 57，违反本节固定的结构/迭代一致门。
按既定排查顺序定位到失效图遗漏：列界变化会使 incidence 邻行的 activity/singleton
结论失效，同时也会使该列自己的 fixed-column predicate 从 false 变为 true；初版
`update_column_activity` 只置脏相邻行，没有重新置脏列本身。该遗漏可改变 fixed-column
被消费的轮次，从而改变 side shift/rounding 和后续 IPM 轨迹。修正失效关系为
`bound(j) change -> dirty column j + incident dirty rows` 后，必须从 focused tests 和同一
cold benchmark 重新开始验收；首轮性能数字作诊断，不作收益结论。

**修正后复测与第二次 mismatch（2026-08-20，cost-model assumption）。** 补全
`bound(j) change -> dirty j` 后，focused/full presolve、numerical-stability 与 NETLIB
regression 再次全部通过；cold repeat-3 的三例约简维度、目标值和 IPM iterations 均与
§8.32 基线逐项相同。presolve 中位为 `greenbeb 6.899->7.130ms`（增加 3.4%）、
`maros-r7 244.058->239.682ms`（减少 1.8%）、`woodw 8.849->6.018ms`（减少 32.0%）。
greenbeb 仍为错误方向，按协议继续排查：queue completeness、排序顺序、P2 rebuild 和
数值路径已有结构/迭代一致证据；剩余假设错误是把“多轮结构变化”直接等同于“P1 扫描
占 presolve 显著比例”。resident activity 已使每个一般行的 P1 检查接近常数成本，
greenbeb 的 0.231ms 差值可能由 queue 初始化/排序或机器噪声覆盖，而 woodw 证实局部
级联 cohort 有收益。不能据此继续微调 queue 阈值。下一步加入同一二进制的只读
`MIPSOLVERS_NATIVE_PRESOLVE_FULL_P1_SCAN=1` 控制和 P1 row/column visit telemetry；默认
路径不变，用邻近 A/B 重估实际 P1 share 后才决定保留或按模型路由 dirty queue。

**同二进制配对结论（2026-08-20，预测重导）。** verbose telemetry 的 dirty/full P1
visits 为：greenbeb `7490/10761 rows`、`8760/22710 cols`（减少 30.4%/61.4%）；woodw
`2196/2906`、`8405/19115`（减少 24.4%/56.0%）；maros-r7 两臂均为
`8424/20968`，因为每轮 P2 commit 按不变量全量失效。随后同一二进制执行 6 组 AB/BA
交替单次配对，报告为 `reports/native_p1_dirty_queue_pair_{1..6}_{full,dirty}.json`。
paired presolve 百分比差的中位为 greenbeb `+2.8%`、maros-r7 `-1.6%`、woodw
`-0.9%`；三例独立中位分别为 `6.420/6.540ms`、`260.525/258.515ms`、
`9.110/8.935ms`（full/dirty）。单对噪声范围很大，不能解释小于约 3% 的 wall 差异。

因此原预测中“P1 visits 减少 50--90%”在列侧大体成立，但据此预测 presolve 减少
10--25% 是 cost-model 错误：resident activity 后 P1 endpoint predicate 太便宜，P3、P2、
compaction 和容器维护才是主要成本。默认保留 dirty queue 的依据不是已测 wall 加速，
而是 changed-support 工作量确定下降、结构/迭代完全一致，且观测到的最大 paired 中位
开销 2.8% 仍在预注册 3% 上限内；它也是后续把更多 fast presolver 接入增量数据流的
必要失效基础。当前不得宣称 dirty queue 单独带来端到端加速，maros-r7 的下一步仍必须
消除 P2 commit 全量失效或降低 P2/compaction 成本。

## 附录 A：P2 代入规则算法卡（HiGHS 语义提取，2026-08-18）

来源：vendored HiGHS（`highs/presolve/HPresolve.cpp/.h`、
`HighsPostsolveStack.{h,cpp}`、`HighsOptions.h`）只读勘察。重实现参照用，
不照抄代码。理论归属：A&A (1995) §2.4；Achterberg et al. (2020)；
Gamrath et al. (2015)（stuffing）。

### A.1 Doubleton equation 代入（`HPresolve::doubletonEq` :3026）

- 前置：行恰 2 非零且**精确**等式（`row_lower==row_upper`，浮点 ==）；
  隐含等式（isImpliedEquationAtLower/Upper :3560-3578）转换的不等式也
  放行，rowType 记 kGeq/kLeq。真 ranged 行不进入。
- 消去列选择（colAtPos1Better :3042-3091）：一整数一连续→消连续；
  两整数→消 |coef| 小者（差 ≤ small_matrix_value 视为等，再比稀疏度）；
  两连续→colsize==1 优先；系数比 ≤2 且稀疏度不同→消更稀疏者；否则消
  |coef| 大者（数值稳定：除数大）。LP-only 实现只需连续分支。
- 变换：行 `a_s·x_s + a_t·x_t = rhs` ⇒ `x_s = rhs/a_s − (a_t/a_s)·x_t`，
  即 substitute(subst, stay, offset=rhs/a_s, scale=−a_t/a_s)
  （:6726-6766）：subst 列出现的每行 i（系数 v）：有限行侧
  `−= v·offset`（**无穷界不传播**），stay 列 `+= scale·v`；目标
  `offset_ += c_s·offset`、`c_t += scale·c_s`（|c_t| ≤ small_matrix_value
  归零）、`c_s = 0`。代入前用隐含界交叉收紧 stay（:3126-3173）：
  **异号时 stay 下界来自 subst 下界，同号时来自 subst 上界**；扩展精度
  （HighsCDouble）；严格超 primal_feastol 才改界。
- postsolve：`DoubletonEquation`（HighsPostsolveStack.h:103）+
  subst 整列快照；primal undo `x_s = (rhs − a_t·x_t)/a_s`。
  LP-only primal 恢复只需要 coef/coefSubst/rhs/两列索引。

### A.2 Free column substitution（`substituteFreeCol` :3410，消元 :2881-2966）

- 前置（LP-only 子集）：`isImpliedFree(col)` = 列两侧界都被**其它行**
  隐含（implColLower/Upper，不含本行贡献，:200-208）；
  `isDualImpliedFree(row)` = 行是等式，或上界有限且 implRowDualUpper ≤
  dual_tol，或下界有限且 implRowDualLower ≥ −dual_tol（:210-216）。
  rhs/rowType 由对偶隐含侧确定（:218-235）。
- 变换：`scale_i = v_i·(−1/a_rc)`；行 i 两侧 `+= scale_i·rhs`；代入行
  整行（除 col）以 `scale_i·a_rj` 加到行 i；目标
  `offset_ −= c_col·(−1/a_rc)·rhs`，`c_j += c_col·(−1/a_rc)·a_rj`；
  **代入行整行删除**（ranged 行含另一侧界一起删，不转松弛——正确性
  完全依赖上面两条隐含条件）。
- 数值保护：Markowitz 主元阈值 `|a_rc| ≥ 0.01·max(rowMax, colMax)`
  （presolve_pivot_threshold，HighsOptions.h:1617-1621；行/列稀疏侧
  先比，两次失败才跳过，:6678-6688）；fill-in 上限
  `presolve_substitution_maxfillin = 10`（HighsOptions.h:1652-1656），
  计数式 `fillin = −(rowsize+colsize−1) + Σ countFillin(i)`（:6691-6697，
  countFillin :2675 用代入前**行快照**判定存在性）；rowsize==2 或
  colsize==2 跳过主元与 fill-in 检查直接代入（:6667-6676）；
  连续 3 次超限终止本轮 aggregator（:6699-6705）。
- postsolve：`FreeColSubstitution`（HighsPostsolveStack.h:90）+
  **代入行整行快照**（必须在删列前 storeRow）；primal undo
  `x_col = (rhs − Σ_{j≠col} a_rj·x_j)/a_rc`。

### A.3 Singleton column（`:3338`；stuffing :4976；与 dualFixing 顺序）

- 处理顺序（:3342-3407）：行也 singleton→转 singletonRow；
  否则 detectDominatedCol → dualFixing（可能直接固定删列）→ stuffing
  → 更新隐含界 → 若 isDualImpliedFree(row)&&isImpliedFree(col) 走 A.2
  连行带列消去。**代入是最后手段**；列被 dualFixing 删后不再代入。
- dualFixing（:4660，A&A §3.2）：0 方向锁→直接 fix 到对应界
  （:4911-4916）；单锁且锁行等式→handleSingleEquation（Achterberg §6.1）；
  否则仅界强化（拒收 |newBound| > primal_feastol/kHighsTiny 的巨大界）。
- stuffing（Gamrath 2015，:4976-5148）：仅非 ranged 行；把行内其它
  singleton 列按 cost/coef 有利方向（a·c<0）排序，逐个在活动上界
  不超行侧的前提下固定到有利界。不删行。LP-only 可暂缓（P3 候选）。
- LP-only 简化注意：没有整数分支；isImpliedFree 的隐含界维护
  （implColLower/Upper、updateColImpliedBounds :667/:742）是 A.2/A.3
  的共同地基，须先行实现。

### A.4 重实现陷阱（勘察记录）

1. 符号：singletonRow 负系数交换 lower/upper 来源；doubletonEq 隐含界
   按两系数是否异号取 subst 界的交叉来源。
2. 两条 substitute 路径行侧符号约定相反（−=v·offset vs +=scale·rhs），
   等价但勿混。
3. 无穷传播：界更新先判 ±inf 才算术；隐含界遇 inf 直接短路。
4. 等式判定是精确 ==；容差等式需先显式转换（记 rowType）。
5. 代入后 |c_stay| ≤ small_matrix_value 归零（postsolve 不依赖该小值，
   但保留它会与参照数值路径分叉）。
6. fill-in 计数含负基准项 −(rowsize+colsize−1)；countFillin 用代入前
   行快照，已存在位置（即使值为 0）不算 fill-in。
7. 主元阈值是双重比较（稀疏侧 max 先比），非单边式。
8. 删除顺序：先 markRow/ColDeleted 再遍历消元；行快照必须先于删列。
9. ranged 行代入删整行不转松弛；正确性前提 isImpliedFree(col) ∧
   isDualImpliedFree(row) 缺一不可。
10. postsolve 两段性：row==-1（dualFixing 复用记录）时仅 primal。
11. 本仓库既有约束叠加：kSideShiftCap=1e2（§6 第一轮修复）对代入侧移
    同样适用；Aeq 回填契约（lhs==rhs 精确等式行回 Aeq）必须在代入
    产生新等式/删除后维持；微量门与投机早失败短路在 P2 后须复验。
