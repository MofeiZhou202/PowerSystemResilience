# R3 恢复轨迹与发布阻断：后续工作记录

记录日期：2026-09-13。状态：暂停研究，后续完善；R3 未获发布批准。
本文件记录待解决的问题，不替代当前实现文档或既定验收合同。

## 当前交接状态

- 分支：`release/windows-self-contained`；基准 HEAD：
  `1e61b2434b07f2bb5a71839048e71fea97ccf586`。本轮改动尚未提交，已有工作树修改保留。
- bounded recovery cap 已放到默认关闭的实验开关
  `MIPSOLVERS_EXPERIMENTAL_LP_RECOVERY_CAP2=1` 后；默认恢复继承调用方线程预算。
- observational telemetry 与策略分离，诊断关闭时不执行诊断 seed 遍历。
- schema-v2 gate 已建立；schema-v1 仅是旧 2T 回归检查，不能批准 R3。
- R3-C2 未实施。用户决定暂存问题，后续再完善。

## 两项未解决的发布失败

1. **greenbea 的 4T 性能方向仍错误。** 固定 20-block 中，2T median/P95
   为 2586.80/4426.60 ms，4T 为 3096.12/4545.22 ms。4T median 慢约
   19.69%，配对 median 差为 +556.41 ms。虽然满足原有 [3000,3300] ms
   区间，但方向合同独立失败；aggregate 收益不能豁免。
2. **广泛 LP 语料出现原模型精度失败。** 首个 4T 候选进程中，greenbea
   在 15010.5702 ms 返回 Time limit，原模型 primal residual 为
   `1.2712008694366573e-7`，超过固定 `1e-7`。采集在首 block 的
   51/52 准确结果后停止，未完成要求的 20-block 广泛语料验证。
   返回 iterations=2000；现有 benchmark 对 Native-IPM 每 variant 的有效
   上限为 2000，该返回值不是所有 fallback variant 的迭代总和。

按事前固定的诊断协议，另执行了三个独立 4T baseline/candidate 配对重放。
两者各 39/39 准确，但候选 greenbea 轨迹仍明显更长。重放没有复现超时，
也没有证明问题消失；这些结果不替换首次失败批次。

相同 seed 指纹和相同 backend sequence 下仍有恢复迭代离散。PARDISO 内部
并行归约、调度，以及其他进程内状态仍是待检验解释，尚未完成因果定位。
不能把“源码未改变数学公式”当作执行轨迹等价的证明，也不能直接断言是机器噪声。

## 已有通过项及其边界

- 三个目标案例的 20-block：120/120 原模型准确；dfl001 每次 44 iterations，
  maros-r7 每次 21 iterations；遥测 schema、配对与计时检查通过。
- 13 案例 × 2T/4T 的实际跨二进制 pivot 比较：26 对全部逐字节相同，
  每个二进制共 83,398 条记录，52/52 求解准确。基准仅加入相同观测探针。
- 完整 Release CTest 串行 21/21，gate 测试 19/19；四个指定程序独立执行。
  `native_kernel_comparison` 虽退出零，极端缩放诊断仍有失败行，不能解读为
  所有诊断问题准确。
- Python 编译、文档锚点、差异空白与新增禁用标记检查通过。

这些通过项不覆盖上面的性能方向失败、广泛语料精度失败或尚未完成的验证。

## 后续恢复工作的约束

1. 先阅读下列 derivation 与失败原始记录，按 implementation fidelity →
   machine/cost model → assumptions → theory 的顺序继续调查。
2. 修改数值算法或资源策略前，在原 derivation 中补充新 RATIONALE：状态模型、
   机器成本模型、定量预测、假设/引用及事前固定的验证协议。
3. 不按案例名或固定矩阵维度路由策略；不延长失败批次的时限、放宽精度阈值、
   替换基线或用重跑成功覆盖失败。
4. 若继续 R3-C2，先验证每个相同 retained seed 在 serial recovery 下至少
   三个独立进程的 recovery iterations 完全固定，再进入完整 20-block。
   其曾预测的 maros-r7 4T 约 14% 回退仍需解决，不能由 aggregate 收益掩盖。
5. 新候选必须重过 schema-v2、固定广泛语料、每案例回归合同、真实 pivot trace
   与完整 Release 验证。所有新测量使用新目录，保留现有失败证据。

## 证据入口与身份

- [持续维护的 derivation 与有序复核](../archive/general_solver_performance_program_2026-09-13.md)
- [完整报告、命令与实测对照](../../reports/r3_release_isolation_20260913/report.md)
- [schema-v2 拒绝判定](../../reports/r3_release_isolation_20260913/evidence.gate.json)
- [实际 pivot 比较](../../reports/r3_release_isolation_20260913/pivot-comparison.json)
- [固定合同](../../benchmark/windows_lp_r3_contract.json)
- [测试与发布手册](../manual/09-testing-benchmarks.md)

原始记录目录：`reports/r3_release_isolation_20260913/` 下的 `stability/`、
`broad/`、`mismatch-replay/` 与 `trace/`。报告目录未纳入 Git；后续迁移机器前
应同时保存这些原始产物。已纳入仓库的参考分布位于 `benchmark/r3_reference/`。

| 身份 | SHA256 |
|---|---|
| 基准二进制 + 同一探针 | `73dcc86e8d49b75551d4a6b189526e823ba30a777951ebe5fb52cfdfa4f6c874` |
| 候选二进制 | `4b520f6840f0689e64d11768c1cbc269a53cfa718a22a25bbd5a35b8d7c94f04` |

构建配置：MSVC 19.44.35228.0 Release x64，`/O2 /Ob2 /Oi /fp:fast`，
oneMKL INTEL，IPO/PGO/native-arch off。隔离基准 checkout 位于
`C:\Users\matri\Codes\MIPSolvers-r3-baseline`。
