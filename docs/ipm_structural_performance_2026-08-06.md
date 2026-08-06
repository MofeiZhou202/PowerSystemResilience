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

### 3.1 Hybrid AC/DC 项目的同模型直接桥接

HybridACDCDistributionSystemsSimulation 现在通过显式
`ACOPFSolverBackend::MIPSolversNativeIPM`，把构造一次的
`parity::NLPModel` 直接交给 `NativeIPMAdapter`。Ipopt 使用同一变量、初值、
边界、目标/约束/Jacobian 回调和精确 Lagrangian Hessian；运行时字符串为
`mipsolvers_native`。旧 `ParityIPM` 保持项目内求解器语义，不能与本后端混称。
Native 桥接强制 `allow_external_fallback=false`，基准还会检查实际后端标签，
因此 Ipopt fallback 不能冒充 Native 成功。

末端新增的是证书型 active-set KKT crossover，而非乘子裁剪。严格互补下，
活跃约束满足 $s_i=O(\mu), z_i=\Omega(1)$，非活跃约束满足相反渐近关系，故用
$\sqrt{\mu}$ 分离强活跃候选。等式行必须保留；候选不等式在等式行空间的补
空间内按数值秩筛成独立工作集；相关行的 $J_h^Tz$ 先投影到该基，并通过乘子
符号门。之后用精确 Lagrangian Hessian 解 active KKT。只有原尺度 primal、
dual、complementarity 三项全部通过才提交。

该 crossover 仅在 primal 与 dual 已通过、唯一剩余失败为 complementarity 时
触发。case30/118 的候选曾分别产生 `144 x 132` 超定行集和数值相关行集，说明
主轨迹尚未给出可靠极限活跃集；强行 polish 会恶化 stationarity 并增加排序/
分解成本，因此已由残差证书门控掉，而不是按案例名禁用。

对照现在显式区分求解器自身终止与共同原模型审计。默认工程 OPF 合同为
`p<=1e-6, d<=1e-5, c<=1e-6`，`--strict-kkt` 保留
`(1e-6,1e-6,1e-8)` 研究门。这个拆分是必要的：Ipopt 有 scaled aggregate 与
acceptable 终止，而 Native 原先只有绝对分量严格门。工程合同下 case30 Native
以 `p=8.5e-11,d=3.6e-6,c=1.8e-7` 通过，热运行 6.9 ms，Ipopt 9.5 ms；
case118 Native 也通过但约 233 ms，仍慢于 Ipopt 约 30 ms。

两个结构优化已经落地：精确 Hessian 模型 Filter 失败后不再无条件重跑 Merit，
case300 从约 2.06 s 降到约 1.0 s；Parity Lagrangian Hessian 即使当前乘子为零
也保留固定因子图位置，使 symbolic analysis 降到 case30/hybrid2000 的 1 次和
case118/300 的 2 次。hybrid2000 从 8.22 s 降到 7.11 s，但仍以
`p=1.5e-3,d=9.7e1,c=1.1e1` 失败；Ipopt 约 2.74 s 且通过。完整复现合同见
Hybrid 项目的
`docs/native_ipm_vs_ipopt.md`。这些数据证明桥接和 case9 crossover 有效，但
不支持“Native 已普遍快于 Ipopt”的结论。

### 3.2 开放 LP 对 SCIP

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

## 6. OPF 下一阶段的理论合同

OPF 的等式/控制分区、固定变量精确消元、数值分解复用下界以及首步方向质量
正则化，已在 [opf_native_ipm_structural_derivation.md](opf_native_ipm_structural_derivation.md)
中给出完整推导。该合同明确纠正两种不充分做法：有限等式松弛不能提交为严格
原模型解；目标惯性正确也不代表非凸首步方向有用。后续实现与 hybrid2000
验收均以该文档中的结构匹配、原空间 postsolve、RHS-only solve 和四重方向证书
为准。

### 6.1 本轮已验证的 OPF 轨迹改动

- 冷启动的模型 nonlinear multiplier 不再使用绝对 `1e-4` 下限；1188 个
  hybrid2000 非活跃非线性行恢复精确 `s_i*mu_i=mu_bar`，初始最大互补由
  `0.0700122` 降为 `0.001`。生成 box rows 保留 bound-dual 稳定化下限。
- `mu_init/mu_min`、终止 tolerance 与 warm start 由统一函数映射到缩放坐标；
  restoration retry 不再遗漏 barrier 参数变换。
- restoration 候选先回到原模型坐标比较，禁止不同 `sf/sf2` 的内部 merit
  直接覆盖。
- `IPMDetail` 新增主 factor、惯性重试、惯性证书、restoration、retry、
  active-set polish 分项；总 factor 覆盖被拒绝的恢复链路。

未缩放 `mu_init` 的实验轨迹曾达到约
`5.65 s, (1.02e-5,16.93,0.2828)`，但 case30 同时从约 7 次迭代退化到
177 次/153 ms，故该规则已撤销，不能列为有效最终结果。局部 tangent 投影/
联合精化也未能把正 barrier slope 变负，相关实验代码已撤销并作为负结果记录
在理论文档第 8.4 节。统一局部 KKT 接受门、corrector 后重算步长和绝对 KKT
best-iterate 排名也因跨案例回归撤销。最终保留版本的跨案例数据以重新构建后的
同模型 benchmark 为准。

最终保留版本的单轮结果为：case30 通过；case118 以 dual `1.1088e-5` 略高于
`1e-5`；case300 为 `(p,d,c)=(2.11e-13,1.48e-3,4.17e-6)`；hybrid2000
为 `(1.94e2,1.57e4,1.06e-1)`，4.17 s。hybrid2000 完整账本为 232 次 KKT
factor，其中 201 次 primary、31 次 inertia retry；phase 标签中 restoration
占 22 次、post-restoration retry 占 117 次。后两者是 primary 的阶段子集，
不应重复相加。主要成本已定位为恢复轨迹重复构造 Newton 矩阵，而非
predictor/corrector 的 RHS-only solve。
