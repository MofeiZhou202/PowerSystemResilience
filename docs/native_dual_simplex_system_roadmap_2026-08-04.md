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

