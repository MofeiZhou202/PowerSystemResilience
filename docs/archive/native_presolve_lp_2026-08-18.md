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
