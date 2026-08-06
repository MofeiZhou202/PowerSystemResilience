# Native IPM 结构化性能提升与等价基准（2026-08-06）

## 1. 范围与审计合同

本轮覆盖 Filter-NLP IPM 与 LP/SOCP/SDP Conic IPM；NETLIB LP 的 90/90
结果继续以 `docs/netlib_benchmark.md` 为准。所有自动决策只使用矩阵结构、
符号分解、后端惯性和同坐标后向误差，不使用案例名、维数表或放宽后的终止
条件。

对比只在数学模型等价时计入性能结论：NLP 对 Ipopt；LP 对 SCIP/HiGHS；
SCIP 没有原生 SDP 锥接口，SOCP/SDP 经非线性或割平面重构只能标记为
`reformulation`，不能作为 Conic IPM 内核速度结论。

## 2. NLP：结构到公式与后端

对每个 NLP Newton 系统构造两个消元图：

$$
K_c=\begin{bmatrix}H+J_h^T(M/S)J_h&J_g^T\\J_g&0\end{bmatrix},\qquad
K_a=\begin{bmatrix}H&J_g^T&J_h^T\\J_g&0&0\\J_h&0&-SM^{-1}\end{bmatrix}.
$$

`NewtonFormulation::Auto` 对两个下三角图各做一次 CHOLMOD symbolic；只有
augmented 的 flops 与 `nnz(L)` 同时严格更低才采用它。显式
`Condensed/Augmented` 保留用于诊断。`IPMDetail` 返回候选维数、原始 nnz、
symbolic flops/`nnz(L)` 以及实际 symbolic/factor/solve 次数。

稳定性约束先于成本：augmented 使用 PARDISO LDLT 或 MUMPS 的直接惯性，目标
为 `(n, m_eq+m_ineq, 0)`；等式双对角使用 Wächter-Biegler $\delta_C$ stripe，
避免把秩亏零模态交给 threshold pivot 隐式扰动。PARDISO 的
`iparm[21]/[22]` 已通过统一线性后端接口导出。condensed 在已证明 `J_g` 满行
秩的 reduced-space certificate 后也使用对称 PARDISO LDLT，不再退回通用 KLU。

### OPF 结构探针：必须保留的负结果

400-bus 探针中 condensed 为 `2.974M flops / 47,180 nnz(L)`，augmented 为
`2.987M / 52,660`，Auto 因此选择 condensed。对称后端接通前 condensed 热运行
约 212 ms；接通后为 25.7–32.7 ms，factor 从错误重试的 54 次恢复为 6 次，
solve 从 22 次降到 16 次。Ipopt 为 15.4–17.5 ms，且该模型初始点几乎已是
解，Ipopt 只需 1 次迭代，Native 需 7 次。因此本例仍不能声称 Native 全面领先；
同时 Ipopt 报告的 complementarity `2.23e-6` 高于本次共同审计阈值 `1e-6`，
Native 为 `9.97e-7`。

## 3. 开放 NLP 案例：HS071

`benchmark/nlp_open_benchmark.cpp` 编码 Ipopt 官方教程的 HS071，Native 与
Ipopt 共用同一初值、边界、目标/约束回调、精确 Lagrangian Hessian 和
`1e-6` 审计。命令：

```powershell
.\tests\Release\nlp_open_benchmark.exe 5
```

去除每进程首次 MKL 初始化后的 4 次热运行：

| solver | success/accurate | iterations | time range | objective error | primal | dual | complementarity |
|---|---:|---:|---:|---:|---:|---:|---:|
| Native | 4/4 | 11 | 0.295–0.402 ms | `5.11e-7` | `5.87e-10` | `1.76e-9` | `3.12e-7` |
| Ipopt | 4/4 | 8 | 2.037–2.273 ms | `1.68e-8` | `6.77e-10` | `8.75e-7` | `1.00e-11` |

该小而稠密案例中 Native 热运行约快 5.1–7.7 倍，且对偶残差更小；首次运行
Native 约 101 ms，主要是 MKL/PARDISO 进程初始化，必须与热运行分开报告。
单个 HS071 只证明机制有效，不代表 CUTEst 全集结论。

### 3.1 开放 LP 对 SCIP

同一 90 例 NETLIB 原始 MPS 的 `native-ipm-direct`/`scip-direct` 单进程运行中，
Native 为 90/90 accurate、总计 9.7944 s、几何均值 12.4018 ms；SCIP 为
89/90 accurate、总计 30.1542 s、几何均值 35.7976 ms。Native 总时间快
3.079 倍、几何均值快 2.887 倍。SCIP 的 `forplan` reader 返回目标 0 并被原
模型审计拒绝。原始结果为
`reports/netlib_native_ipm_vs_scip_full90_2026-08-06.{csv,json}`。

## 4. Conic：后向误差驱动的精化

原实现对每个 affine/corrector 方向固定多做一次 3x3 KKT 回代。现在计算

$$
\eta=\max_i\frac{\|f_i\|_\infty}
 {1+\sum_j\|K_{ij}d_j\|_\infty+\|r_i\|_\infty},
$$

只在 $\eta>0.1$（Dembo-Eisenstat-Steihaug 收缩充分条件）时精化，并且仅提交
严格降低 $\eta$ 的校正。quick SOCP/SDP 四例全部为 optimal，迭代数保持
`10/5/5/5`，实际 refinement 全为 0，线性回代分别为 `20/10/10/10`；最大
初始后向误差在 `1.8e-8` 到 `1.44e-7`，已经远低于 forcing gate。结果见
`reports/conic_native_structural_final_2026-08-06.json`。

## 5. 当前边界与下一批开放数据

- NLP：已接 HS071；尚需用可校验转换器接 CUTEst，而不是手工扩写案例。
- SOCP：需接 DIMACS/Maros-Meszaros 中可等价转换的集合。
- SDP：需接 SDPLIB，并记录原始文件校验和及许可。
- SCIP：只在原始 LP/MINLP 数学表达完全等价时比较；SDP 不做伪等价横评。
- Cached/node LP：本轮未改其公式组合，NETLIB 非缓存 90/90 不能替代节点热
  启动工作负载；需要单独报告 prepare、首节点和后续节点分位数。

因此本轮可以确认“结构画像、惯性证书、后向误差门控”已跨 LP/NLP/Conic
复用并产生可测收益，但不能据此宣称全部开放套件已经完成或 Native 在所有
NLP/Conic 上领先 Ipopt/SCIP。
