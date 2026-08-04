# LP Kernel 审计整改与性能验收（2026-08-02）

## 1. 结论

本轮完成了 `lp_kernel_math_model_and_audit.tex` 后续建议中的 1、2、3、5：

1. PRICE/BFRT 大工作量路径并行化，并评估显式 SIMD 的可行性；
2. 标准型 CSC/CSR 索引宽度自适应，普通模型使用 32 位，超限模型提升为 64 位；
3. `cost_shift` 和 `cost_perturbation` 改为稀疏优先、自适应稠密化 journal；
4. 增加 `-march=native`、LTO 和 PGO 的可控构建路径并完成 A/B。

正确性测试、ASan/UBSan 和 TSan 的定向并发测试通过。合成微基准证明并行内核在达到门槛时有效，但当前 24 个 NETLIB 模型没有达到保守并行门槛，因此不能把合成加速比宣称为 NETLIB 端到端加速。构建 A/B 也显示 native、LTO、PGO 在当前机器和训练负载上没有稳定总体收益，均不应默认开启。

原始审计如实保留了两个失败：`grow22` 被 canonical residual 审计
fail-closed；MILP 随机差分 case 112 返回错误最优值。2026-08-04 的后续
修复已解决 `grow22` 的 reconstruction 依赖问题并达到 NETLIB 72/72；后者
仍位于并行 LP kernel 之外的现有 MILP presolve/branch-and-cut 工作区。

## 2. 实现

### 2.1 PRICE/BFRT 并行化

- 稀疏 pivotal row 继续使用 CSR scatter，避免为了并行先做不划算的稠密化。
- 当 pivotal row 足够稠密、列数至少 4096、非零元至少 65536 时，PRICE 改用 CSC 列分区。
- BFRT 在 scan 项至少 8192 时并行计算逐列分类。
- BFRT 的容量求和、计数和候选插入仍按原始 PRICE 顺序串行合并，因此 worker 数不改变浮点累加顺序或候选顺序。
- 使用进程级持久线程池，pivot 内不创建和销毁线程。
- `SimplexOptions::lp_kernel_threads` 控制 worker 上限；`MIPSOLVERS_LP_KERNEL_THREADS` 可覆盖它。`1` 强制串行，`0` 选择不超过 4 个 worker 的硬件感知默认值。

显式 SIMD 未加入。这里的主循环是稀疏 CSC 间接索引 gather；当前 arm64/NEON 没有适合这一访问模式的通用 gather 指令。强行添加 SIMD pragma 既不能证明向量化，也可能损害数值复现性。当前可证实收益来自列级并行和编译器标量优化。AVX-512 gather 或 SVE 平台应另做目标平台 A/B。

### 2.2 自适应 CSC/CSR

- `StandardColumnMatrix` 以 `variant` 保存 Eigen 32 位或 64 位 CSC。
- 行数、列数和预计 nnz 都在有符号 32 位范围内时使用 32 位 CSC；`reserve()` 检测到超限时提升为 64 位。
- `StandardRowMatrix` 不再复制 value 数组，只保存 CSR 行偏移、列索引和 CSC value-position 映射。
- CSR 的三类索引分别选择 32/64 位；常见路径的 row view 成本为约 `4 bytes/row + 8 bytes/nnz`。
- copy/move 后显式重新绑定 row view 的 value 指针，避免悬空引用。
- HFactor 可直接绑定 32 位或 64 位压缩 CSC；普通路径不再为每次 INVERT 复制索引和值。

测试覆盖普通模型选择 32 位、copy/move 后 value 共享语义，以及 `rows = INT_MAX + 1` 时无需巨量 nnz 即自动选择 64 位 CSC。千万行单位阵探针保持 32 位 row offsets，并共享 CSC values：

```text
MIPSOLVERS_LP_SCALE_ROWS=10000000
10 assertions passed
elapsed_sec=0.090904
row_index_bytes=120000004
```

该探针验证的是千万级常见路径，不等同于实际分配超过 `UINT32_MAX` 个非零元；后者仍需要分块/惰性生成器或高内存机器验证。

### 2.3 稀疏 cost journal

- `cost_shift` 与 `cost_perturbation` 从两个 dense `n` 向量改为稀疏优先 journal。
- 稀疏状态用紧凑开放寻址表保存 column 到 slot 的映射，并保持确定性插入遍历顺序。
- 已分配稀疏 slot 达到约 `n/8` 时自动转换为 dense，避免高密度哈希查找。
- `set(column, 0)` 和加法抵消会正确减少当前非零计数；取消后重新加入不会绕过存储膨胀门槛。
- `size()`、`empty()`、`max_abs()` 和非零遍历在 sparse/dense 两种状态下语义一致。

### 2.4 native、LTO、PGO

CMake 新增：

```text
MIPSOLVERS_ENABLE_NATIVE_ARCH=ON|OFF
MIPSOLVERS_ENABLE_IPO=ON|OFF
MIPSOLVERS_PGO_MODE=OFF|GENERATE|USE
MIPSOLVERS_PGO_PROFILE=<Clang .profdata 或 GCC profile 目录>
```

Clang/AppleClang 使用 instrumentation PGO，GCC 使用 generate/use profile。`USE` 会在配置期检查 profile 是否存在；PGO 只作用于项目核心库，第三方大库不参与训练。默认保持 PGO 关闭。

## 3. 性能结果

测试环境：macOS 26.5.2、arm64、16 个在线逻辑处理器、Apple Clang 21.0.0。Release，24 个 NETLIB case，每个重复 3 次。表中的变化均相对原始 baseline；正号表示变慢。

| 构建 | 成功/准确 | 中位数 ms | 中位数变化 | 几何均值 ms | 几何均值变化 |
|---|---:|---:|---:|---:|---:|
| 原始 baseline | 69/72 | 4.200 | 基准 | 4.068 | 基准 |
| 最终源码，native/LTO/PGO 关闭 | 69/72 | 4.192 | -0.2% | 4.088 | +0.5% |
| `-march=native` | 69/72 | 4.322 | +2.9% | 4.262 | +4.8% |
| `-march=native` + LTO | 69/72 | 4.154 | -1.1% | 4.207 | +3.4% |
| PGO | 69/72 | 4.320 | +2.8% | 4.210 | +3.5% |

最终源码与 baseline 的差异落在短任务计时噪声范围内。native-only、native+LTO 和当前 PGO profile 都没有一致改善中位数与几何均值，不能作为默认配置。

强制命中门槛的 Release 微基准，500 次、4 worker、三轮：

| 内核 | 第 1 轮 | 第 2 轮 | 第 3 轮 |
|---|---:|---:|---:|
| PRICE | 3.684x | 3.680x | 3.764x |
| BFRT | 1.086x | 1.097x | 1.091x |

PRICE 的工作划分足以摊薄线程池调度；BFRT 仅并行分类，确定性合并仍串行，因此收益约 9%。真实 NETLIB 模型未达到当前保守门槛，不能据此声称 NETLIB 获得上述加速。

原始结果：

```text
/private/tmp/netlib_baseline.json
/private/tmp/netlib_final.json
/private/tmp/netlib_final.csv
/private/tmp/netlib_native.json
/private/tmp/netlib_native_lto.json
/private/tmp/netlib_pgo.json
/private/tmp/mipsolvers-pgo-data/netlib.profdata
```

## 4. 功能与内存安全验收

| 验证 | 结果 |
|---|---|
| `test_dual_simplex` Release | 5495 assertions / 57 cases，通过 |
| `test_lp_solver` Release | 41 assertions / 6 cases，通过 |
| `test_numerical_stability` Release | 409 assertions / 16 cases，通过 |
| `native_kernel_comparison --smoke --check --time-limit 60` | 0 must-pass failures |
| PRICE 串行/4 worker 等价 | 4096 列逐列完全相等 |
| BFRT 串行/4 worker 等价 | entering、容量、计数、Harris、事务完全相等 |
| ASan + UBSan 定向整改测试 | 4150 assertions，通过；可选千万行测试按设计跳过 |
| ThreadSanitizer PRICE | 4098 assertions，通过，无 race 报告 |
| ThreadSanitizer BFRT | 15 assertions，通过，无 race 报告 |
| macOS `leaks` | 0 leaks / 0 leaked bytes |
| `git diff --check` | 通过 |

## 5. 未通过项

1. ~~NETLIB `grow22` 三次都 fail-closed~~：已于 2026-08-04 修复。
   audit cache 原先只按 factor generation 去重，未包含 nonbasic side
   vector；side classification 后的 primal reconstruction 因而跳过 defect
   correction。修复后保持原 `1e-08` 上限，NETLIB 为 72/72。
2. 完整 MILP 固定种子 `2858143525` 的随机差分 case 112 失败：solver 报告 `Optimal (tree exhausted)`、目标 `-5`，穷举 oracle 为 `-6`。当前为 702/703 assertions。该路径位于现有 native MILP presolve/branch-and-cut 修改中，本轮 LP kernel 文件没有以放宽 oracle 或降级审计的方式掩盖它。

## 6. 后续性能建议

优先级从高到低：

1. 增加生产 telemetry：记录 PRICE row density、列数/nnz、BFRT scan size、每次并行是否命中、并行耗时占比。收集真实分布后再调 4096/65536/8192 门槛。
2. 将 LP kernel worker budget 与上层并行 B&C 联动。多个节点 LP 同时各取 4 worker 会过订阅；建议由全局并行预算器给每个 node solve 发 token，外层并行饱和时强制 LP kernel 串行。
3. 对标准型矩阵引入只读结构版本号或 immutable handle，让 CSC、CSR 和 HFactor 绑定共享一个所有权对象。这比裸 value pointer 更容易证明生命周期，也可用版本号 O(1) 拒绝错误 rebind。
4. 对超大模型采用分块构造器：先用 64 位计数 pass 计算每列 nnz，再一次性分配 32/64 位 CSC。这样可以避免逐项插入重分配，也能在不常驻 triplet 副本的情况下验证 `>UINT32_MAX` nnz 路径。
5. PGO 必须用真实生产 LP/MILP 根松弛训练，并在 CI 记录 compiler version、源码 fingerprint 和 profile fingerprint。只有在独立验收集上中位数、几何均值、尾延迟同时不退化时才启用。
6. 在 AVX-512 gather 或 SVE 目标机上重新评估显式 SIMD；arm64/NEON 当前不建议强制向量化稀疏 gather。
7. 对更大的 BFRT scan，可尝试每 worker 构造局部候选数组后按区间稳定拼接，同时保留容量求和的原序串行语义。只有 profiling 证明 merge 成为瓶颈时才值得增加复杂度。
