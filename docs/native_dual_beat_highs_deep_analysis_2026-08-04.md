# Beating HiGHS：per-pivot 数据结构与 cleanup 政策深度分析（2026-08-04）

## 0. 结论先行

**DR-3 的"算法地板"结论在 pivot count / pricing 层面正确，但在 per-pivot 成本层面不成立。**
本次分析用新鲜 24×3 fleet 数据 + 采样 profile + native/HEkkDual 逐机制对照，把 2.28x per-pivot gap 分解为**六个有名有姓的结构性税**（非分布式微噪声），并发现一个此前被 `dual_pivots` 指标掩盖的 **primal cleanup 尾巴**（在 6 个 case 上占 kernel 30–100%）。

量化后的翻盘条件（用当日 24×3 数据反推）：

| kernel 时间削减 | gate（geomean vs HiGHS） |
|---|---|
| 0%（现状） | 0.855x |
| 10% | 0.940x |
| **18%** | **≈1.00x** |
| 30% | 1.171x |
| 50% | 1.558x |

即：**不需要追平 HiGHS 的 2.28x per-pivot，只需削掉 ~20% kernel 时间即可 >1.0x**；下文清单中的结构性税总量远超此数。

## 1. 新鲜证据（fixed binary, 24×3, best-of-3）

### 1.1 Fleet 全景

| case | H_ms | N_ms | N/H | H_iters | N_dual_pivots | pivot 比 |
|---|---:|---:|---:|---:|---:|---:|
| 25fv47 | 68.8 | 169.1 | 2.46 | 2583 | 2181 | 0.84 |
| d2q06c | 315.3 | 771.8 | 2.45 | 5243 | 5609 | 1.07 |
| lotfi | 0.95 | 2.21 | 2.34 | 103 | 265 | 2.57 |
| degen2 | 6.50 | 13.94 | 2.14 | 481 | 604 | 1.26 |
| sc205 | 0.74 | 1.51 | 2.05 | 88 | 0(+224 primal) | — |
| degen3 | 69.6 | 138.9 | 2.00 | 2042 | 2082 | 1.02 |
| scsd8 | 18.9 | 36.2 | 1.92 | 1080 | 1017 | 0.94 |
| pilot4 | 15.8 | 30.0 | 1.90 | 760 | 356(+693 primal) | 0.47 |
| bandm | 3.91 | 6.93 | 1.78 | 382 | 280(+192 primal) | 0.73 |
| …小 case（afiro/recipe/blend/kb2/stocfor1/adlittle/agg/ship04s/fit1p）native 全胜 | | | | | | |

**三个推翻既有叙事的事实：**

1. **native 的 pivot count 在 fleet 上已优于 HiGHS**（25fv47 0.84x、pilot4 0.47x、grow22 0.42x）。若 native 被换成 HiGHS 的迭代数，gate 反而降到 0.829x。→ pivot count 全域无杠杆（DR-3 该点被 fleet 数据加强）。
2. **native 在所有 5 个微型 case 上赢**（setup 无问题），**在所有 9 个有实质 pivoting 的 case 上输 ~2x** → 差距 100% 是 per-pivot kernel 成本。
3. **`dual_pivots` 指标掩盖了 cleanup**：pilot4 "356 vs 760" 实为 356 dual + 693 primal cleanup = 1049 total。

### 1.2 cleanup 尾巴（DS-PHASES, repeat-1）

| case | dual 阶段 | primal cleanup | cleanup 占 kernel | startShifts |
|---|---|---|---:|---:|
| pilot4 | 3.7ms/356 | 23.0ms/693 | **86%** | 4 |
| fit1p | ~0/0 | 13.3ms/541 | **~100%** | 13 |
| sc205 | ~0/0 | 1.6ms/224 | **~100%** | 1 |
| bandm | 2.8ms/280 | 3.6ms/192 | **56%** | 131 |
| grow22 | 22.3ms/810 | 27.4ms/533 | **55%** | 66 |
| 25fv47 | 119ms/2181 | 50.5ms/833 | **30%** | 136 |
| d2q06c | 741ms/5609 | 2.7ms/9 | 0.4% | 0 |

机制：cost-shifted dual start 把 shifted 问题解到最优，再用 original-cost primal simplex 重解残差。HiGHS 用**有界扰动 + 逐 pivot `shiftBack`**（HEkkDual.cpp:2215-2222）+ rebuild 时 `correctDualInfeasibilities`，从不支付第二次 solve。§2 value map 早已预言 "S2/S3 + cleanup 数据流 → 25fv47"，此发现证实该假设并推广到 6 个 case。

仅消除 cleanup 尾巴（保守按砍 80%）：gate 0.855 → **≈0.98**。

### 1.3 d2q06c 采样 profile（top-of-stack, 3-repeat 窗口）

`multiply_AT_indexed_impl`(PRICE) 263 · `choose_entering_bfrt` 204 · HFactor solves(ftranU+solveHyper+btranU+ftranL+btranL) 281 · `push_leaving_row` 44 · `certify_bfrt_dual_feasibility` 34 · bridge(`ftran_indexed*`/`btran_indexed`) 44 · `compute_dse_weights` 24 · `build_primal_transaction` 21 · `std::vector::push_back` 10。

DS-PROFILE（0.72s minor / 5609 pivots ≈128µs/pivot）：PRICE 34µs、ratio/BFRT 27µs、BTRAN 12.5µs、FTRAN 12.5µs、DSE 16µs、rcUpdate 7µs、postcond 3.6µs、OTHER(update/copy) 14µs。HiGHS 全 solve 60µs/pivot。

## 2. 六个结构性税（native ↔ HEkkDual 逐机制对照）

### T1 容器模型：IndexedVector（packed-only + 惰性哈希）vs HVector（dense array + index + pack）

- native `IndexedVector`（model.hpp:571-618）只有 `(index[], value[])` + 惰性 `lookup_slot` 哈希；**没有 dense 视图** → 随机访问 = 哈希探测；每次 solve 结果都要重建两个 vector + 哈希表。
- HiGHS `HVector`（util/HVectorBase.h:40-74）：持久 dense `array` + `index/count` + 独立 `packIndex/packValue`，`clear()` 只擦上次 support（count≤30% 时，HVectorBase.cpp:47-64），一次 setup 终身复用，循环内零分配。
- **后果**：这一个选型差异同时向 PRICE（见 T3 的 stamp/epoch 补偿机制）、solve 桥（T2 的导出/重建）、DSE（`at()` 哈希探测）、BFRT（`active_position` 侧通道 + `dense_row_ep` 重散射）征税。

### T2 solve 桥：每次 FTRAN/BTRAN 的 O(m) 置换 gather + 错误的 expected_density

`hfactor_backend.cpp:600-651`（ftran；btran 同构）每次 solve：

1. **O(m) 全长置换回拷**：`for external in 0..m: result[external] = vector.array[external_to_internal[...]]` —— 与 support 无关，每 pivot 3–4 次 solve × O(m)。HiGHS 无此层：HVector 就是工作向量，全程 internal 序。
2. **expected_density 传的是 RHS 密度**（单位向量 ≈1/m≈0.0006），而 HFactor 的 kernel 选择需要的是**结果密度**（HiGHS 传 0.95/0.05 运行均值：row_ep/col_aq/row_DSE density，HEkkDual.cpp:1499/1962/2044；阈值 kHyperFtranL=0.15、kHyperFtranU=0.10）。在 d2q06c（rowEP 实测 40% dense）上，近零 expected_density 强制走 `solveHyper` 的 DFS+拓扑表机制，而 40% 密度下普通 skip-scan kernel 更快——**共享同一 HFactor 却被喂错参数**。
3. pattern 导出 push_back + 调用方再重建 IndexedVector（+哈希）。
4. HiGHS 还把 factor-update 输入当作 solve 副产品免费捕获（row_ep pack 在 btranU 内部、col_aq pack 在 ftranU 内部：HFactor.cpp:1895-1900、1706-1712）；DSE FTRAN 直接 **in-place 复用 row_ep**（HEkkDual.cpp:1225），无一份拷贝。

### T3 PRICE：全行扫描（含 basic 列）+ 三遍成型 + 每元素间接寻址

`multiply_AT_indexed_impl`（pricing.cpp:457-516），d2q06c 上 34µs/pivot vs ~13µs 的 flop 下界：

1. 内层 `InnerIterator` 遍历**整行所有非零，包括 basic 列**（约 27-50% 的元素被算完即弃）。HiGHS 的 `ar_matrix_` 是**行内分区**的行存副本——nonbasic 前缀 `start_..p_end_`，PRICE 只触 nonbasic 段（HighsSparseMatrix.cpp:1319-1388、1487-1491），分区由每 pivot O(nnz(a_in)+nnz(a_out)) 的 swap 维护（1573-1603）。
2. `InnerIterator::value()` = `values_[source_position(position)]`（dual_simplex.hpp:514-515）——**每个矩阵非零一次间接寻址**，HiGHS 是平铺 value 流。
3. stamp/epoch 累加器（3 条内存流：value/stamp/touched + per-column push_back）补偿"无 dense 视图"；HiGHS 直接散射进 `row_ap.array` 并 in-flight 收集 index（`if (value0==0) index[count++]`），超过 10% 密度时中途切 dense-result 模式再一遍 n-scan 收尾。
4. 结束后**第二遍** touched 过滤 tiny + 重建 packed 结果，**第三遍**逻辑生成 `active_position`。HiGHS 一遍 + `tight()`。

### T4 ratio test：多遍候选扫描 + 重型 per-candidate 状态

`choose_entering_bfrt`（pricing.cpp:741-1100+），d2q06c 957 候选/pivot、27µs/pivot：

- `prefilter_active_phase_two` 先整遍算 signed_alpha+flags；主扫描再整遍：4 个 capacity 累加器（带 isfinite 分支）、逐候选 breakpoint 除法、taboo 查询、40B `Candidate` push_back；needs-exact 候选再走 `dot_error_bound`（57.5 次/pivot，触发 `dense_row_ep` O(m) 散射）。
- HiGHS `HEkkDualRow`：`chooseMakepack` 一次拷进连续 workset，`choosePossible` 单遍（乘、比、append），Harris 分组扫描只在**不断缩短的前缀**上进行（chooseFinal, HEkkDualRow.cpp:117-315），无 per-candidate 误差界（固定容差 Ta + shift 兜底），只在最后对小 flip 表 pdqsort（为内存序 gather A 列）。

### T5 post-commit：每 pivot 一遍全 support 的对偶可行性复核

`certify_bfrt_dual_feasibility`（solver.cpp:536-586）：commit 前对 **pivot_row 全 support（d2q06c ~2588 项）再走一遍**，重算每个 reduced cost 更新并检查符号——之后 rcUpdate 再做一遍同样的更新。HiGHS 靠 ratio test 语义 + shift 保证对偶可行，**每迭代零复核**（dual infeasibility 计数在 rebuild 才重建，HEkkDual.cpp:1121-1124）。
合计 native 每 pivot 在 pivot-row support 上走 **~5 遍**（PRICE 过滤、prefilter、主扫描、postcond、rcUpdate）；HiGHS ~2 遍（pack、updateDual，且 updateDual 与 dual objective 增量融合）。

### T6 cleanup 政策（见 §1.2）——非 kernel 而是相位结构

## 3. 行动方案（按 gate 期望收益排序）

### P0：消灭 cleanup 尾巴（Class A，政策变更，单项 ≈ +0.12 gate）
把"大额 start shift + 事后 original-cost primal 全量重解"换成 HiGHS 式**有界小扰动 + 逐 pivot shiftBack + rebuild 时 correctDualInfeasibilities**；或至少让 cleanup 从 dual 最优基出发用 **dual**（原成本）而非 primal 重解，并审计 pilot4（4 次 shift 却 693 个 cleanup pivot——说明 shifted 最优离原问题最优系统性偏远，值得单独归因）。验证：DS-PHASES cleanup 份额 + 24×3 gate + 72/72。

### P1：HVector 化的 factor-resident workspace（Class A，一揽子结构变更）
即 S2 原设计的**整体**落地，而不是 DR-1 那种逐消费者微迁移：
- 四个命名向量 row_ep/row_ap/col_aq/col_bfrt 直接以 dense array + index + pack 形态**常驻 factor 后端**，全程 internal 行序（把 external/internal 置换折叠进矩阵一次性重排或仅在 rebuild/输出边界转换），删除 hfactor_backend 的 O(m) 置换 gather 与 IndexedVector 重建。
- expected_density 改传 per-向量类 0.95/0.05 运行均值（结果密度），修正 solveHyper/普通 kernel 的选择。
- pack 捕获对齐 HiGHS：factor update 输入由 solve 副产品提供，删除 luUpdate 前的重提取。

### P2：分区行存 PRICE（依赖 P1 的 dense row_ap）
nonbasic-前缀分区 + 单遍散射 + in-flight index 收集 + 平铺 value 流（消 source_position 间接寻址）+ >10% 密度中途切 dense-result。目标 34µs→~15µs（d2q06c minor 的 ~15%）。分区维护成本 O(nnz(a_in)+nnz(a_out))/pivot，已被 HiGHS 证明可数值中立（不改 reduction 顺序时是 Class P 候选，但建议并入 P1 的 Class A 基线一起重建）。

### P3：ratio test 与认证的单遍化
prefilter+主扫描合一遍；**postcondition 融进 rcUpdate 的同一循环**（边更新边检查——同样的数、少一遍 support 走查，不弱化任何证书）；capacity 累加器去 isfinite 分支（无穷 range 预分类）。

### P4：lotfi 类小退化 case 的 pivot 膨胀（2.57x）
唯一真的 pivot-count 缺口；量级小（gate 影响 ~1-2%），放最后，与 P0 的扰动政策一并处理。

**合成估算**：P0 ×1.15；P1+P2+P3 保守砍 kernel 25%（PRICE 15µs + 桥/拷贝 ~10µs + 认证/扫描合并 ~8µs，对 128µs/pivot ≈ 26%）→ ×1.35 → **gate ≈ 1.3x**。即便只兑现一半也过 1.0x。

## 4. 为什么此前的流程没找到（方法论修正）

1. **逐 diff 墙钟门槛否决分布式收益**：2x gap 由 6 个各占 5-15% 的税构成；DR-1 对 S2 的"摊销证据"只测了单个消费者的迁移（<0.6%、0.3%），必然全部 "低于分辨率"。修正：P1-P3 作为**一个** Class-A 实验整体重建基线（§13.2 本就允许），gate 用 24×3 A/B/A + 72/72，而非逐 diff bit-identity。
2. **`dual_pivots` 不含 cleanup pivots**：让 pivot "持平" 的表象掩盖了第二次 solve。修正：报告 total pivots（dual+primal cleanup）与 cleanup 墙钟份额，纳入 §15 模板。
3. **SIMD gate（§13.3 bytes 同降）设计正确但目标选错**：真正的 bytes 削减不在 SIMD，而在 T1-T5（少走 3 遍 support、删 O(m) gather、砍 basic 列扫描——全是实打实的 bytes 下降，恰好能过该 gate）。
4. §12 重开条件（"新架构成本项"）由本清单满足：这些不是换 threshold，是命名了此前未计量的成本项。

## 5. 验证清单（沿用现有合同）

- 每项落地后：full native tests、NETLIB 72/72、SCUC audit、sentinel metamorphic；
- 24×3 A/B/A（one-thread vs one-thread）报 geomean + 代表 case + total pivots（含 cleanup）+ per-pivot + DS-PHASES cleanup 份额；
- P1/P2 附独立内核证据：动态指令与有效 bytes 同降（本方案天然满足——删的是整遍整遍的内存走查）；
- warm/B&C cohort 回归不允许倒退（S5 资产保持）。

## 6. 本次分析的原始数据

- fleet 24×3 CSV：/tmp/nd_percase.csv（gate 0.855x best-of-3；全量 run 0.866x）
- d2q06c DS-PROFILE/DS-DENSITY/DS-BFRT、6-case DS-PHASES：见本文 §1
- 采样 profile：/tmp/d2q_sample.txt
- HEkkDual 机制清单（file:line 详表）：本文 §2 内嵌（源自 highs/simplex/HEkkDual.cpp、HEkkDualRow.cpp、HEkkDualRHS.cpp、highs/util/HVectorBase.*、HFactor.cpp 逐行核对）

---

## 7. 实施决策记录（2026-08-05）

**基线勘误**：本文 §1/§6 引用的 d2q06c "5609 pivots / 769ms" 来自 8-04 的陈旧二进制。当前源码树（含已入库的 P4 密度均值）从头构建的真实基线为 **5476 pivots / 31 rebuilds / ~693-748ms**。本节所有对照均以重建基线为准。

### DR-A（P0）：shifted-start vs dual-phase-I A/B —— 混合结果，保留现策略（diagnostic-only）

零代码实验（`MIPSOLVERS_DUAL_SHIFT_START=off` 走 dual Phase I 路径，best-of-3）：
pilot4 31.0→15.8、grow22 54.0→33.5、25fv47 166.7→138.0（OFF 大胜）；但 fit1p 15.7→43.7、scsd8 34.3→43.3、sc205 1.7→2.2（ON 大胜）。翻转默认会令 fleet gate 变差（≈0.845x）；per-case selector 触 §12。**结论**：现阈值策略在现有组件下已局部最优；cleanup 尾巴的真正解法是"更强的 dual Phase I 或更快的 cleanup"（Class-A 项目），而非策略翻转。§1.2 的 cleanup 份额数据保留为该项目的立项证据。

### DR-B（P3）：certify+commit 单次计算融合 —— bytes 中性，测量后否决（已回退）

实现了认证期计算 + 提交期发布（scratch 传递，保持事务边界），d2q06c 路径合同保持（5476/31 逐位一致）。但测量显示 postcond/rcUpdate 桶均无变化（0.03/0.07s）：物化认证值（写+回读 ~23kB/pivot scratch）恰好抵消重算的开销——**融合按构造 bytes 中性，不过 §13.3 gate**。与 DR-1 的摊销结论同构：被同一趟 pass 顺带完成的计算不是可削减成本。附带教训：认证检查与提交更新历史上使用不同的浮点结合序（FMA 收缩敏感），跨表达式重构必须显式保持两种 shape。

### DR-C（P2a）：PRICE 平铺行序值流 —— **retain**

`StandardRowMatrix` 增加 `flat_values_`（rebuild/rebind 时物化；`ruiz_scale_standard_form` 原位缩放后显式 `refresh_flat_values()` 重同步）。PRICE 内层从"顺序 4B position 读 + 随机 8B CSC gather（整 cache line）"变为 12B/nnz 全顺序流。
- **成本模型项**：C_critical-path 的 PRICE 有效 bytes 与动态指令同降（§13.3 双降满足）。
- **测量**：d2q06c PRICE 桶 0.20→0.15s（−25%），wall 748→676ms（−9.5%），路径合同逐位保持（5476/31，DS-BFRT 计数全同）；fleet 24×3 gate 0.866→**0.875x**，72/72 accurate，kernel ms/pivot 均值 0.0204→0.0184（−10%）。
- **正确性**：test_dual_simplex 75/75（1 个断言原测隐式别名合同，已按新合同更新）、test_netlib_regression、test_branch_and_cut 23/23、test_scuc_module 5/5 全绿。
- **内存代价**：行视图 +8B/nnz（16B/nnz 总计），仍低于完整行拷贝（值+64 位索引）。
- **合同变化**：行视图对 CSC 原位改值不再隐式可见；唯一生产原位改值点（Ruiz）已接 resync，合同写入类注释与测试。

### DR-D（P4）：结果密度运行均值 —— 已在树内实现，验证通过

分析 §2-T2 的"expected_density 传 RHS 密度"缺陷在当前树中已被修复（`update_solve_density_mean` + 每类 `density_mean_ftran/btran/aq/ep`，全部 solve 入口以 `max(RHS 密度, 运行均值)` 传入）。这解释了重建基线相对 8-04 二进制的路径/性能差异。无需新改动。

### 剩余主线

P1（HVector 常驻工作区一揽子改造）仍是最大未实施项：indexed 热路径导出已 support-bounded，但每 solve 仍付 packed 导出/重导入 + internal↔external 置换查表 + IndexedVector 惰性哈希。按 §13.2 作为单个 Class-A 实验立项（新基线：d2q06c 5476/31）。cleanup 尾巴（§1.2）的 Class-A 解法（强化 dual Phase I）为第二立项。
