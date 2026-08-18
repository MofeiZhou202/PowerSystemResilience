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
