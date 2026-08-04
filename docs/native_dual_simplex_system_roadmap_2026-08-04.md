# Native Dual Simplex 系统路线（2026-08-04）

## 0. 文档地位

本文是 `src/engine/kernel/lp_kernel/native_dual/` 后续正确性、架构和性能工作的主路线。它固定工作顺序、理论合同、实验门槛和停止条件；现有文档继续作为推导、实验和实现历史：

- `native_dual_simplex_rewrite.md`
- `native_dual_simplex_performance_theory.md`
- `native_dual_simplex_optimization_2026-07-31.md`
- `native_dual_simplex_kernel_speedup_plan.md`

若旧建议与本文冲突，以本文为准。任何生产变更必须先映射到本文的一个阶段和一个成本项，并写明它保持或改变的数学合同。不能以单案例变快代替理论依据，不能以 benchmark 成功代替正确性证明。

路线按版本演进，但原则不能被一次实验静默修改。改变阶段顺序、正确性合同、事务边界或晋级门槛，必须在本文追加决策记录，包含新证据、反例和替代方案；失败实验只进入实验日志，不反向塑造 case selector。

## 1. 总目标和统一成本模型

目标是在相同正确性、相同资源和可复现实验条件下超过 HiGHS，而不是只在若干训练案例上取得局部胜利。

统一总成本为

\[
T = T_{\mathrm{setup}}
  + K C_{\mathrm{critical\mbox{-}path}}
  + \sum_r R_r
  + T_{\mathrm{cleanup}}
  + T_{\mathrm{cert}}.
\]

- \(T_{\mathrm{setup}}\)：标准形、basis、权重、factor 和工作区初始化。
- \(K\)：全部 phase 与 cleanup 的 pivot 数，不只统计主 dual Phase II。
- \(C_{\mathrm{critical\mbox{-}path}}\)：单 pivot 数据依赖图的关键路径成本；必须进一步分解为求解、PRICE、BFRT、DSE、状态提交、动态指令、有效字节和 stall。
- \(R_r\)：第 \(r\) 次 reinversion/rebuild 的实际成本，包括因 fill、稳定性和状态重建引起的工作。
- \(T_{\mathrm{cleanup}}\)：恢复原成本和最终可发布状态的代价。
- \(T_{\mathrm{cert}}\)：数值证书、残差审计、ray/certificate 构造和必要回退。

每个提案必须明确减少上述哪一项，以及是否可能增加其他项。只报告总时间、只报告 pivot 数或只报告 cache bytes 都不足以支持生产晋级。

reinversion 的基准模型为

\[
C(q)=\frac{R}{q}+C_0+\frac{u(q-1)}{2},
\qquad
q^*=\sqrt{\frac{2R}{u}},
\]

其中 \(q\) 是更新周期，\(R\) 是一次 reinversion 成本，\(u\) 是每次 basis update 带来的平均后续求解成本增量。这个公式只给出测量候选；数值稳定性、fill、growth 和 residual 触发器始终优先。

## 2. 当前证据和问题排序

最近固定二进制测量显示：

| Case | Native | HiGHS | Native / HiGHS pivot |
|---|---:|---:|---:|
| `d2q06c` | 856.8 ms | 326.4 ms | 5609 / 5243 |
| `degen3` | 150.9 ms | 74.2 ms | 2082 / 2042 |
| `25fv47` | 181.3 ms | 70.8 ms | 3014 / 2583 |

最新 24-case × 3-repeat gate：Native 4.359 ms，HiGHS 3.568 ms，速度比 0.819x；双方均为 72/72 accurate。

因此当前主要结论是：代表性大案例的首要差距是 per-pivot 架构和关键路径，不是单纯 pivot count。`25fv47` 另有可见的路径差异，但它仍不能解释约 2.56 倍总时间差。先解决贯穿每个 pivot 的数据运动和依赖，再讨论新的全局 pivot 规则。

预期收益映射如下：

| 工作 | 首要收益对象 | 预期作用 |
|---|---|---|
| S0 边界域正确性 | sentinel、warm、certificate/ray | 消除错误状态和伪性能数据 |
| S2/S3 驻留工作区与 pivot DAG | `d2q06c`、`degen3` | 降低每 pivot 指令、访存和容器转换 |
| S2/S3 + cleanup 数据流 | `25fv47` | 同时降低 per-pivot 与终止路径成本 |
| setup/workspace 复用 | 24-case fleet geomean | 降低小模型固定成本 |
| S5 warm state | MIP node、cut、strong branching | 避免重复 factor、norm 和 basis 工作 |
| S4 SIP | 等线程数对照中的宽 pivot DAG | 改善并行比较，不用于解释单线程基线 |
| S6 reinversion | workspace 改造后的 factor 路径 | 重新平衡 \(R\) 与 \(u\) |

## 3. 不可跳级的阶段路线

```text
S0 Bound-domain correctness
  -> S1 Measurement contract
  -> S2 Factor-resident pivot workspace
  -> S3 Proposal / certification / commit pipeline
  -> S4 Serial + SIP executors
  -> S5 Warm solver state
  -> S6 Reinversion remeasurement
  -> S7 Algorithmic and degeneracy work
```

一个阶段可以进行只读研究和 benchmark harness 准备，但其生产优化不得先于前置阶段的完成门槛。S0 的 correctness 修复可独立合入；S1-S3 应优先形成单线程基线；S4 不得掩盖 S2/S3 的串行效率问题。

## 4. S0：Bound-domain correctness

### 4.1 根因

当前标准形同时出现数学无穷 `+inf` 和有限 sentinel `1e20`。`std::isfinite(upper)` 只能回答浮点表示问题，不能回答“是否存在真实上界”。把 sentinel 当作真实端点会把非基变量放到 `1e20`，污染 RHS、basic solution 和 `Ax=b` 审计。

工作树已包含过渡修复：

- `state.cpp` 中的 `has_real_upper(upper) := isfinite(upper) && upper < 1e19`；
- `normalize_nonbasic_moves`、`decide_cost_shifted_dual_start`、`initialize_cost_shifted_dual_start` 使用同一判断；
- warm path 在 exact normalization 失败后接入 shifted dual start。

这能让 sentinel 变量执行 shift `-reduced_cost` 并停在 lower，而不是重新停到 `1e20`。但它还不是完整域模型：`state.cpp`、`certificate.cpp` 和 primal 路径仍存在直接的 `std::isfinite(upper)` 判断。

### 4.2 最终合同

必须选择并贯彻一个全局表示：

1. 在 `StandardForm/Bounds` 边界把所有 `upper >= 1e19` 规范化成数学 `+inf`；或
2. 引入显式 `BoundKind`，数值字段只保存真实有限端点。

首选方案 1，因为它让现有数值算法可以使用标准 `isfinite` 语义并缩小审计面。若输入/输出需要保留原 sentinel，应把表示转换限制在 adapter 层，solver core 不得观察到 sentinel。

必须满足以下不变量：

- lower-only 变量永远不能选择 upper side；
- cold、warm、rebuild、fallback、cleanup 使用同一 bound classification；
- certificate/ray/primal/dual 对有限、单侧、free、fixed 的分类一致；
- 内部状态中不存在把“无界”当作巨大可行数值参与算术的路径；
- `1e20` 与 `+inf` 两种外部编码产生等价的状态转换、终态和证书。

### 4.3 完成门槛

- 全局审计所有 bound 检查并消除 solver-core 的 sentinel 特判扩散；
- 增加 `1e20` / `+inf` metamorphic equivalence tests，覆盖 cold、warm、rebuild、certificate 和 ray；
- native 数学测试、NETLIB regression、SCUC audit 全部通过；
- 对失败案例保留最小复现和状态 trace，不用性能开关绕过。

## 5. S1：Measurement contract

在修改 pivot 架构前固定可复现测量合同：

- 同一编译器、Release flags、CPU affinity/电源条件、线程数和 time limit；
- Native 与 HiGHS 使用相同 presolve 与资源口径，串行对照固定为 one-thread vs one-thread；
- 记录 setup、各 phase pivot、cleanup、reinversion、certificate、wall 和每 pivot 成本；
- 记录 PRICE/BFRT/DSE/solve/commit 的时间、调用数、support、动态指令和有效读写字节；
- 同时记录 memory-stall normalized time 与 non-memory-stalling normalized time，避免把频率、分支或 cache 现象误判成访存收益；
- 固定 path trace：pivot、rebuild、phase transition、BFRT flip/shift 和 termination class；
- 所有报告保存 binary/config hash、原始结果和聚合脚本版本。

独立内核只在动态指令和访存量同时下降时通过结构 gate。微基准必须使用生产分布的 support、density、bound class 和数值范围；uniform synthetic 只能做机制验证，不能直接晋级生产。

## 6. S2：Factor-resident pivot workspace

### 6.1 目标

让一个 pivot 的向量在 factor backend 中一次生成、保持驻留并被所有消费者直接读取，消除 export/reimport、重复 clear/scatter、稀疏 hash lookup 和跨层临时容器。

factor-owned named workspace：

```text
row_ep | col_aq | col_dse | col_bfrt
```

HFactor backend 对每个 workspace 同时暴露：

- dense coordinate view：按 row/column 直接寻址；
- ordered packed support：稳定顺序的 `(index, value)` 流；
- generation/lifetime token：阻止消费者读取过期 view；
- ownership 状态：proposal 可写，certification 只读，commit 后回收。

PRICE、BFRT certification、DSE update 和 pivot verification 必须直接消费这些 view。workspace API 的价值由整个 pivot 生命周期减少的 bytes 和 instructions 证明，不以减少某个局部 `vector` 分配作为充分理由。

### 6.2 结构约束

- packed support 顺序是数值合同的一部分，Class P 变更不得改变 reduction/merge 顺序；
- dense 与 packed view 必须引用同一份数值事实，禁止双向复制形成两个权威版本；
- workspace 容量按 solve/factor 生命周期复用，setup 和 warm solve 不重复分配；
- backend 不泄漏算法决策，只提供 factor 结果和稳定 view；
- 先建立串行正确实现和独立内核证据，再接 executor。

### 6.3 完成门槛

- pivot 生命周期中每个向量的 owner、producer、consumer 和失效点均有表格或断言；
- Class P path contract 全等；
- 独立 benchmark 同时降低动态指令和 bytes；
- 完整 24×3 A/B/A 不回退正确性，且改善与微基准的成本模型方向一致。

## 7. S3：Proposal / certification / commit

统一 pivot 数据依赖图：

```text
CHUZR -> BTRAN
            |-> DSE FTRAN
            |-> PRICE -> BFRT -> aq FTRAN
                                -> BFRT FTRAN
                                -> dual proposal
JOIN -> scalar certification -> atomic factor/basis/state commit
```

pipeline 分为三个语义阶段：

1. **Proposal**：在只读 solver state 和私有 workspace 上产生候选 pivot、flip/shift、reduced-cost 更新和 DSE 更新提案。
2. **Certification**：按原标量顺序验证 ratio、pivot identity、误差界、有限性、feasibility budget 和 factor update 前置条件。
3. **Commit**：只在全部证书通过后，原子地提交 factor、basis、bounds side、cost journal、reduced costs、primal values、weights 和统计；失败丢弃 proposal，不发布半更新状态。

该分层使性能内核可替换，而正确性边界不随 SIMD/任务调度改变。标量 certification 和事务边界不得为性能而弱化。

S3 的首个重点是 PRICE 无阈值地只输出 active BFRT support；cheap prefilter 只能删除由解析上界证明不可能进入 active support 的项。精确 `dot_error_bound`、原 PRICE 顺序和 merge 顺序保留在 certification 路径中。收益必须是所有候选的结构性质，不能依赖 case、density 或 wall-time selector。

## 8. S4：Serial + SIP executors

串行和 SIP executor 必须执行同一个 pivot DAG、同一 proposal schema、同一 certification 和同一 commit。并行版本只能改变合法独立节点的调度，不能形成第二套算法。

规则：

- serial executor 是 one-thread HiGHS 比较的权威基线；
- SIP 只并行 DAG 中已证明无共享写的节点；
- 每个 worker 拥有私有 scratch，join 后才进入标量 certification；
- 线程数是用户资源配置，不是 case/size/density/wall-time selector；
- equal-thread Native vs HiGHS 是 SIP 的晋级 gate；
- 若调度开销超过独立工作，serial executor 自然获胜，不增加经验阈值切换算法。

HiGHS 的 serial dual pipeline、SIP 以及 PAMI/suboptimization 可作为实现参照，但其经验 cutoff 不能直接成为本项目理论。PAMI 只有在得到基于候选持续性、冲突概率和摊销成本的规则后才进入生产实验。

## 9. S5：Warm solver state

参考 SCIP/SoPlex 的 basis 与 pricing norm 持久化、strong-branch parent basis 恢复，warm snapshot 必须包含：

```text
basis + nonbasic sides + DSE norms + cost journals + factor/refactor metadata
```

状态所有权合同：

- parent snapshot immutable；
- 每个 worker 拥有可变 factor 和 pivot workspace；
- append cut 时新 slack 初始为 basic，旧 basis/norm 的合法部分不重算；
- strong branching 分支结束后恢复 parent basis、sides、norm 和 cost journal；
- factor 是否可复用由版本、矩阵增量和 residual certificate 决定，不靠 case selector；
- snapshot 必须携带 bound-domain 版本，禁止重新引入 sentinel 语义差异。

warm benchmark 单独分为 root、node reoptimization 和 strong-branch cohort，报告首次 solve 与后续 solve；fleet cold geomean 不能代替 warm 收益证明。

## 10. S6：架构改变后重测 reinversion

当前独立 reinversion 调参关闭，保留现有

```text
max(50, min(200, m/4))
```

此前按 \(q^*=\sqrt{2R/u}\) 得到的候选周期与现策略性能差最大约 1.8x，未通过预设 2x gate。S2-S5 会改变 workspace、solve update、warm factor 和 certificate 的 \(R,u\)，因此只有这些架构变化完成后才重新测量。

重测流程：固定 factor 状态 cohort，分别测 \(R\)、每次 update 后的 solve 成本斜率 \(u\)、fill/growth/residual；由公式生成候选，再在完整 solve 中验证。safety/numerical/fill triggers 永远覆盖经济周期。

## 11. S7：算法与退化

只有当 S2/S3 的 per-pivot 架构已稳定，才能把剩余差距可信地归因于 pivot count。届时按以下顺序：

1. 分离 cold start、dual I、dual II、original-cost cleanup 和 fallback 的 pivot bill；
2. 以 benchmark-only 三路对照当前 Devex、结构 exact DSE、完整 exact DSE，记录初始化时间、pivot 数、每 pivot 成本和总时间；
3. 对退化问题建立 objective progress、zero-step run、tie population、weight error 和 rebuild 相关性；
4. 只有存在跨 cohort 的可预测状态量与理论机制时，才提出 Class A 策略；
5. 对新路径重新建立 correctness proof、path contract 基线和 out-of-sample gate。

算法优化不能用于补偿错误 bound domain，也不能以降低 pivot 数为由接受更差的总成本或证书质量。

## 12. 明确禁止的方向

以下方向没有新的理论和跨 cohort 证据前不得进入生产：

- case-name selector；
- dimension、density 或 wall-time selector；
- 将 hardcoded pricing thresholds 作为性能 policy；
- PRICE/CSC switching；
- BFRT RHS layout retries；
- CertifiedExact CHUZR；
- CHUZR fusion；
- PRICE absolute-dot fusion；
- prevalidated reduced-cost stream；
- 在没有 proof-derived candidate-persistence rule 前引入 production PAMI；
- 弱化 scalar certification 或 proposal/commit transaction boundary；
- S2-S5 前独立重调 reinversion；
- 用线程数或并行调度掩盖单线程关键路径；
- 由单一目标案例结果晋级生产。

失败实验不是永久禁令，但重开必须指出旧实验为何不再适用：新的理论、不同的架构成本项或新的硬件事实至少满足一项。单纯换 threshold 不构成新证据。

## 13. 验证与晋级协议

### 13.1 Correctness gate

- full native tests；
- NETLIB 72/72 accurate；
- SCUC numerical audit；
- sentinel metamorphic tests；
- certificate/ray、cold/warm、rebuild/fallback 覆盖；
- 无新增未解释 numerical failure、cleanup 或 termination class。

### 13.2 变更分类

**Class P：路径等价优化。** 必须保持 pivot、rebuild、phase、BFRT flip/shift、termination trace 一致；浮点 reduction 顺序若改变，则不再自动属于 Class P。

**Class A：算法变更。** 必须给出明确数学合同、允许改变的 trace、新路径的终止/数值论证和重新建立的基线，不能借用 Class P 的较低证据成本。

### 13.3 性能 gate

- 独立内核：动态指令和有效 bytes 同时下降；
- profiler：memory-stall 与 non-memory-stalling normalized time 同时报告；
- serial：固定二进制 24×3 A/B/A，对比 one-thread HiGHS；
- parallel：相同线程数 A/B/A；
- warm：root/node/strong-branch cohort；
- 报告 geomean、代表性大案例、pivot 数、per-pivot、setup、cleanup、reinversion 和证书成本；
- targeted case 只用于诊断，不用于生产晋级。

晋级结论只能是 `retain`、`diagnostic-only`、`reject` 或 `blocked-by-prerequisite`。`retain` 必须说明收益来自成本模型的哪一项，并确认没有把成本转移到未测阶段。

## 14. 实验提案模板

```markdown
### Experiment ID / date

Stage: S0-S7
Class: P or A
Hypothesis:
Cost-model term:
Invariant/theorem:
Producer -> consumer dataflow changed:
Expected instruction reduction:
Expected byte reduction:
Possible regressions or shifted costs:
Independent-kernel design:
Correctness cohort:
Performance cohort:
Path-trace expectation:
Promotion gate:
Rollback/removal condition:
```

任何缺少 `Invariant/theorem` 或 `Cost-model term` 的性能提案不开始写生产代码。

## 15. 结果日志模板

```markdown
### Experiment ID / binary hash / config hash

Result: retain | diagnostic-only | reject | blocked-by-prerequisite
Correctness: tests / NETLIB / SCUC / metamorphic
Trace: pivot / rebuild / phase / BFRT / termination
Kernel: instructions / bytes / branch misses / cache misses
Profile: memory-stall / non-memory-stalling normalized time
Serial A/B/A: Native / HiGHS / ratio
Parallel equal-thread A/B/A:
Warm cohorts:
Setup / pivots / per-pivot / rebuild / cleanup / cert:
Representative cases:
Theory confirmed or falsified:
Unexpected cost transfer:
Artifacts:
Next authorized step:
```

## 16. 固定执行顺序与当前下一步

当前只授权以下顺序：

1. 完成 S0 全局 bound-domain 审计和 metamorphic tests；
2. 固定 S1 测量 schema、trace 与基线；
3. 设计 S2 factor-owned workspace 的 lifetime/ownership API，并做独立内核 benchmark；
4. 将 PRICE/BFRT/DSE/pivot verification 迁移到驻留 view；
5. 用 S3 proposal/certification/commit 重组 pivot transaction；
6. 在同一 DAG 上实现 serial，然后实现 SIP；
7. 建立完整 warm snapshot 和 MIP reoptimization cohort；
8. 重新测量 \(R,u\)，再决定 reinversion 周期；
9. 最后重新评估 exact DSE、退化和 pivot-count 算法。

这条路线的核心判断是：先统一状态语义，再固定证据，再消除 pivot DAG 上的结构性数据运动，最后才让并行与算法策略建立在稳定内核之上。后续工作不再按失败案例逐个打补丁，而按共同状态模型、共同数据流和共同成本模型推进。

## 17. 外部参照

- HiGHS dual serial/SIP pipeline：`highs/simplex/HEkkDual.cpp`。
- HiGHS factor workspace：`highs/util/HFactor.cpp`、`HFactorConst.h`。
- HiGHS 1.15.1：<https://github.com/ERGO-Code/HiGHS/releases/tag/v1.15.1>。
- Parallel dual revised simplex：<https://arxiv.org/abs/1503.01889>。
- Basis update：<https://doi.org/10.1007/s10589-014-9689-1>。
- SCIP/SoPlex basis、pricing norm 与 strong-branch state persistence：`scip/lpi/lpi_spx.cpp`。
- SCIP 10.0.3 memory-stall-separated measurement：<https://github.com/scipopt/scip/releases/tag/v10.0.3>。

## 18. 决策记录

### DR-1（2026-08-04）：S2 驻留视图摊销证据 → 优先 S3 候选削减

**触发**：执行 §16 step 4（PRICE/BFRT/DSE/pivot verification 迁移到驻留 view）。

**新证据**（两个生产密度独立内核，源码已入库；见 `native_dual_simplex_s2_workspace_design.md` §10–§11）：

- `native_dual_price_resident_benchmark`（row_ep → `dot_error_bound`）：去掉每 pivot 的 `dense_row_ep` clear+scatter（≈25.7 kB/pivot）在候选 dot-scan 上被摊销。候选数 2 / 8 / 32 / 128 / all → materialize÷resident = 1.083 / 1.078 / 1.025 / 1.006 / 1.000x。大案例每 pivot 认证候选达数百（d2q06c BFRT ≈957 候选），落在 ≥128 摊销区 → wall 收益 <0.6%，低于分辨率。判定 **diagnostic-only**。
- `native_dual_colaq_pivot_benchmark`（col_aq `column_pivot`）：单次 `.at()` 建 O(support) 哈希 → 驻留 dense O(1) 读，微内核 35–105x，指令与字节（`lookup_slot` ≈8·support B/pivot）同时下降，过 §13.3 结构 gate；但仅占 153 µs pivot 的约 0.3%，低于 wall 分辨率，按结构主导保留（已实现 `a54dd2b`，bit-identical，72/72）。

**核心结论**：目标大案例每 pivot 的主导成本是候选**认证 dot-scan（计算）**，不是 S2 去除的 pivot 周边数据运动。驻留视图价值是**消费者形态相关**的：单次读消费者（col_aq `column_pivot`）有结构收益；被多候选摊销的消费者（row_ep dot-scan）没有。

**反例/边界**：col_aq 证明驻留视图并非无用——结论不是"驻留视图无价值"，而是"其价值不触及主导项"。故保留 S2 已落地部分（col_bfrt 复用缓冲 `2e47c74`、col_aq 驻留读 `a54dd2b`）与两个 harness（作为后续 workspace 提案的摊销 gate）。

**决策**：在 S1–S3 单线程基线块内，不再穷尽 S2 step 4 的低价值剩余迁移（row_ep dense-read 接线、col_bfrt fuller），转入 S3（step 5）。S3 首个重点"PRICE 无阈值只输出 active BFRT support + 解析上界 cheap prefilter"直接攻击已测得的主导项（认证候选数）。此举不越过任何阶段的正确性前置：S2 的 design + 独立内核 gate（step 3）已完成，S3 是既定下一步。

**替代方案**：
- (a) 穷尽 S2 驻留迁移：拒绝——harness 证明 ≤0.3% 且被摊销，为低于分辨率的收益承担热循环风险。
- (b) 直接跳 S7（pivot-count/退化）：拒绝——§11 要求 S2/S3 per-pivot 架构先稳定，且 S7 明确最后。
- (c) 优先 S3 候选削减（**采纳**）：收益须是所有候选的结构性质（非 case/density/wall selector），符合 §7。

**保持不变量**：精确 `dot_error_bound`、原 PRICE 顺序、merge 顺序、scalar certification、proposal/commit 边界不变（§7、§12）。**下一步**：先测量当前认证路径每 pivot 的 exact-dot（needs-exact）候选数，量化可削减空间，再提出结构性收紧的解析上界。

### DR-2（2026-08-04）：延后 S4/SIP，优先 S5 warm state

**触发**：S3 完成（proposal/certification/commit 已分层，全部 bit-identical，24×3 仍 72/72，0.863x）；执行 §16 step 6（S4 serial + SIP）。

**证据/理论**：
- serial executor 已就绪：`minor_iteration` 现为 proposal（`detail::` 调用 + `build_primal_transaction`）/ certification（`certify_bfrt_*`）/ atomic commit 的清晰分层，三个抽出函数均以 scratch/scalar 传参，具备 per-worker 私有 scratch 条件。
- SIP 唯一可测的 per-pivot 独立并行是 DSE-FTRAN（factor solve，≈17.8 µs/pivot）‖ PRICE（`A_row` 矩阵乘，≈37.4 µs/pivot）：二者从 `BTRAN(row_ep)` 分叉且资源不同（factor vs `A_row`），可并发；`aq`/`BFRT` FTRAN 共享 factor workspace，不可并发。完美 2-thread 重叠上界 ≈ min = 17.8 µs/pivot ≈ 11.6% wall，减每 pivot ≈3 µs 任务开销后 ≈9%。
- 该 ≈9% 只在 2-thread 且须胜过 2-thread HiGHS（成熟 PAMI）；对 1v1 主 gate（当前 0.863x）无贡献（§2 value map：SIP 只改善并行对照，不解释单线程基线）。此前 1/2/4-thread run 在 `d2q06c`/`degen3` 相同，佐证细粒度数据并行无收益。

**决策**：延后 S4/SIP。serial executor 保持 1v1 权威基线；SIP 仅在出现明确 equal-thread 需求且 DSE‖PRICE task overlap 通过 §13.3 gate 时重启。转入 S5 warm state——native dual simplex 的真实产品用途是 native B&C（MIP）内的 node reoptimization 与 strong branching，cold geomean 非该用途的正确度量（DR-1：cold per-pivot 近地板）。

**替代方案**：
- (a) 现在实现 SIP：拒绝——正交于近地板 1v1 gate，equal-thread 对 HiGHS PAMI 收益不确定，热循环成本高。
- (b) 跳 S6/S7：拒绝——S6 需 S2–S5 架构先定，S7 明确最后。
- (c) 优先 S5 warm state（**采纳**）：产品价值最高，且 S3 分层已为 warm snapshot 提供事务边界。

**保持不变量**：serial executor 是 1v1 权威；SIP 与 serial 共享同一 DAG/proposal/certification/commit；warm snapshot 必须携带 bound-domain 版本（禁止重新引入 sentinel，§9）。**下一步**：审计当前 warm-start（`solve(sf, options, hint)`）携带的状态，对照 §9 要求的 basis + sides + DSE norms + cost journals + factor metadata，补齐缺失项（当前 hint 仅含 basis + at_upper，DSE norms 未 warm-start）。

**S4 代码级确认（2026-08-04）**：实测 `minor_iteration` 的 pivot 链为顺序 `BTRAN → PRICE → BFRT → FTRAN → DSE → commit`。`compute_dse_weights` 需 `direction`（pivotal-column FTRAN 输出），故 DSE 在 FTRAN 下游，**非与 PRICE 并行**（理想 DAG 的 `DSE‖PRICE` 与当前实现不符）。唯一名义独立对 `col_aq FTRAN`（`update_vec_aq`）‖ `col_bfrt FTRAN`（`solve_vec_ftran`）共享同一 HFactor 的 mutable HVector workspace，且 `col_bfrt` 仅在 `has_flips`（少数 pivot）出现。PRICE 数据并行为已关闭死路（§12）。故 real SIP 需 per-worker HFactor 副本（§9）或将 DSE 重构为从 BTRAN 分叉（Class A），皆为大改动且对 1v1 gate 正交、收益边际。**结论**：serial executor 确认为 S4 1v1 权威基线；不为边际正交收益向已验证热循环引入未验证 HFactor 并发（数据竞争风险）。SIP 仅在完成 Class-A DSE-from-BTRAN 重构且 per-worker factor 就绪后，凭 §13.3 equal-thread gate 重启。

**S5 warm state 进展（2026-08-04）**：
- **bound-domain 版本标签（commit 2a93314）**：`SimplexBasis` 携带 `kBoundDomainVersion=1`（S0 canonical +inf 表示）+ per-snapshot `bound_domain_version`。两个 producer（`export_basis`、LPModel wrapper）打标；warm DSE-inheritance guard 增加版本匹配检查 → 旧 sentinel-domain 的 reduced cost / DSE norm 无法被静默复用（§9 要求）。falsifiability 测试 `[bound_domain]`：匹配版本逐位继承 distinctive DSE norm（0 pivot），失配版本拒绝并重初始化。bit-identical Class-P（d2q06c 5609/31；fleet 72/72 639.0 pivot）。
- **warm 再优化 cohort（commit d987c9e）**：`netlib_solver_benchmark --warm-cohort` 对每个 NETLIB case 求解 root，收紧若干 basic structural 变量上界（合成 B&C branch），再 cold vs warm（root basis + 继承 DSE norm）求解 node，报告 warm/cold pivot 与 wall 比。kernel 级（presolve-free），比值隔离 warm-start 效应。结果（24 case，4-var/0.5 branch，repeat 3）：17/24 usable（7 node 在合成 cut 下不可行，诚实跳过），0 objective mismatch，geomean warm/cold pivot 0.146（**6.85x 更少**）、wall 0.278（**3.60x 更快**）、总 pivot 削减 90.0%；每个 warm 求解 `dse_initialization_solves=0`（DSE norm 继承，无 re-init BTRAN）。将 §9 warm 价值从 unit-scale 证明提升为可报告 cohort。
- **剩余 S5**：strong-branching parent-basis restore 覆盖；warm snapshot 的 cost journal / factor metadata 补齐（当前仅 basis + at_upper + DSE norm）。

**S5 完成（2026-08-04）**：
- **strong-branch parent restore 覆盖**：test `[strong_branch]` 固定 §9 kernel 级不变量——不可变 parent snapshot（basis + sides + DSE norms + version）跨多个 probe 复用：probe A warm → 中间 probe B warm → 再 probe A warm，第二次 A 与第一次 A **逐位相同**（pivots + objective + basis），证明中间 probe 未改动 parent；每个 warm probe `dse_init=0` 且与 cold 同 optimum；parent 字段全程不变。
- **snapshot 完备性判定（理论导向，避免 dead field）**：实测 `native_dual` warm-init（state.cpp `initialize_snapshot`）仅消费 hint 的 `basis_indices`（basis）、`at_upper`（sides）、`cached_dse_weights`+version（DSE norms）。它 **重建全新 `BasisFactor` 并 rebuild**（state.cpp:1345-1350），**不消费** hint 的 `cached_sparse_basis`（factor metadata 对 kernel warm 路径无关；factor 复用是更高层 `dual_simplex.cpp` `resolve_same_structure` / bc 的优化，已在该层处理），且 cost_shift/perturbation 每次 fresh 重置（state.cpp:1307-1308）。故 **kernel 级 warm snapshot 在 basis+sides+DSE-norms+bound-domain-version 处已完备**；§9 列出的 factor/refactor metadata 属更高层职责，cost journal 继承为投机优化（warm 路径已用 `decide_cost_shifted_dual_start` 按需重算），无证据前不实现（DR-1/DR-2 纪律）。
- S5 状态：**完成**（version tag + falsifiability + warm cohort 6.85x + strong-branch restore 覆盖 + snapshot 完备性判定）。下一步 S6（架构改变后重测 reinversion，测量性）或 S7（algorithmic，真正 1v1 杠杆）。

### DR-3（2026-08-04）：S7 证据——cold 处于算法地板，无 Class-A pivot 杠杆

**触发**：S5 完成后执行 §16 step 7（S7 algorithmic）。§11 要求 evidence-first：分离 pivot bill → 3-way DSE → 退化相关性 → **仅在跨 cohort 可预测状态量+机制时**提 Class-A（§13.2 要求数学合同 + 重建基线）。

**证据（repeat-3 fleet + d2q06c 深挖，测量性，无代码变更）**：
1. **pivot bill**（d2q06c DS-PHASES）：dualII 主导 0.756s/4186 pivot（75% pivot、93% wall）；dualI 0.035s/1423；cleanup 9。
2. **3-way DSE（repeat 3, 72 solves）**：Devex 824.7 pivot（0.804x）；StructuralDSE 与 ExactDSE **均 639.0 pivot**（per-pivot wall 微差为噪声，pivot 路径相同）。DSE 质量已到 exact，Devex→exact 已采纳；**DSE 非 S7 杠杆**。
3. **native vs HiGHS（d2q06c，同 HFactor 后端）**：pivot **几乎相同**（native 5618 vs HiGHS 5243，差 ~7%）；wall native 816ms vs HiGHS 334ms → **per-pivot 2.28x**（native 0.145 vs HiGHS 0.064 ms/pivot）。**gap 是 per-pivot 成本，非 pivot count。**
4. **PRICE 固有**：DS-DENSITY rowEP **40.16%**、pivotRow **40.30%** dense（远超 ~10% hypersparse 交叉点）→ hypersparse/row-wise PRICE 只会更慢；PRICE 成本是 40%-dense pivot 几何的固有代价，非算法低效（理论证伪 hypersparse-PRICE 杠杆，非试错）。
5. **无退化杠杆**：degen_dual=216/5618（**3.8%**）、degen_primal=0、cycles=0、bound_flips=4002（71%，BFRT 已高效承担）。退化极低，无 anti-degeneracy Class-A 空间。

**结论**：S7 在本 cohort **无 Class-A 算法杠杆**——pivot count 已与 HiGHS 持平、DSE 已 exact、退化极低（3.8%）、PRICE 在 40% 密度下固有。残余 **2.28x per-pivot gap** 分布于 native PRICE/BFRT/DSE vs HiGHS 成熟内核，属 SIMD/cache/fused-loop 微优化对标十年调优竞品，与本路线图算法范围正交、边际收益递减、且触碰 §12 禁止项（PRICE/CSC switching、absolute-dot fusion）。**cold 确认处于本架构算法地板**（第三次印证 DR-1/DR-2）。

**决策**：不提 Class-A 热循环变更（§11 step 4 未满足：无可预测状态量+机制；pivot count 已持平即证无 pivot-reduction 空间）。S7 算法层收敛为"已达算法地板"结论。重开须满足 §12 条件（新理论/新架构成本项/新硬件事实），例如针对 40%-dense PRICE 的可验证 SIMD 独立 kernel（动态指令 AND bytes 同降，§13.3 gate），而非热循环投机。

**替代方案**：
- (a) 强推 PRICE/BFRT/DSE SIMD 微优化：拒绝——触 §12、正交算法范围、对标成熟竞品收益递减、已验证热循环高风险。
- (b) 追 pivot count：拒绝——已与 HiGHS 持平（5618 vs 5243），无空间。
- (c) 收敛为"算法地板"结论 + 记录证据（**采纳**）：诚实、evidence-backed、符合 §11/§13.2 纪律。

**产品含义**：native dual simplex 的价值在 warm/B&C（S5 已证 6.85x node-reopt），非 cold 1v1 geomean。DR-1/DR-2/DR-3 三重独立印证 cold 近地板：S2 摊销、S4 顺序 DAG、S7 per-pivot 竞品对标。路线图 S0–S7 主线达成稳定内核 + 已证 warm 价值 + 诚实 cold 边界。







