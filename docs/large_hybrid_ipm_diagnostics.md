# 大型富混合 AC/DC IPM 内部诊断与求解器调用链

## 1. 调用链

大型 AC/DC OPF 的原生求解路径为：

1. `solve_ac_opf()`：校验拓扑、提升外部电网为可调边界电源并选择后端；
2. `solve_with_parity_ipm()`：构造统一 AC/DC 全空间问题；
3. `parity::build_problem()`：生成变量、等式/不等式、上下界和元件映射；
4. `parity::solve_primal_dual_ipm()`：执行 Mehrotra 预测—校正内点法；
5. `factor_kkt_sparse()`：组装并进行 Ruiz 对称均衡的稀疏 KKT；
6. SuiteSparse UMFPACK、KLU 或 Eigen SparseLU：数值分解与回代；
7. `solve_with_parity_ipm()`：还原 Vm、Va、Vdc、Pg/Qg、换流器和 DER 结果；
8. `verify_opf_result()` / PF：进行独立物理审计和潮流回放。

可以通过环境变量
`HACDCPF_OPF_LINEAR_SOLVER=umfpack|klu|eigen|dense`
强制底层线性代数后端，用于交叉诊断。

## 2. KKT 数学结构

不等式采用松弛变量，凝聚后的预测—校正方程为

\[
\begin{bmatrix}
W+\delta I & J_g^T\\
J_g & -\delta I
\end{bmatrix}
\begin{bmatrix}\Delta x\\\Delta\lambda\end{bmatrix}
=
\begin{bmatrix}r_d\\r_p\end{bmatrix},
\]

其中

\[
W=\nabla^2_{xx}\mathcal L+J_h^T
\operatorname{diag}(\mu/z)J_h.
\]

KKT 在分解前执行对称 Ruiz 均衡
\(\widehat K=DKD\)，并采用逐级正则化和迭代改进。UMFPACK、KLU、
Eigen SparseLU 与 Dense LU 使用同一模型、同一 KKT 和同一停止判据。

## 3. case300_acdc 的根因

故障不是模型不可行，也不是某个 LU 库单独失效：

- Ipopt 可将最大原始约束残差降到约 \(2.5\times10^{-9}\)，证明模型可行；
- 修复前 KLU、Eigen SparseLU 和 Dense LU 均在 1600 次迭代后停留在约 3%
  的原始残差；
- UMFPACK 轨迹在第 723 次迭代附近进入病态区域，随后所有稀疏分解均失败。

原生 IPM 在得到预测—校正方向后直接执行全步更新，只用
fraction-to-boundary 保护松弛变量和乘子为正，没有检查更新后真实的非线性
AC/DC 残差。局部 Newton 方向因此可能连续恶化全局残差，最终使
\(J_g\) 和凝聚 KKT 数值退化。底层分解失败是该轨迹失控的结果，而非首因。

## 4. 修复

原生 IPM 对维数不小于 512 的大型 KKT 增加残差 filter 回溯。每个候选步重新计算完整 AC/DC 等式、
不等式、Jacobian、目标梯度和拉格朗日梯度。候选点至少要在以下一条轴上
取得充分进展，同时受另外两条轴的保护：

1. 原始可行性；
2. 对偶平稳性；
3. 互补性。

不满足 filter 的步长按 0.5 回溯，最多 14 次。小型 KKT 保留原有快速全步路径，
避免对已经稳定的小系统引入不必要的严格全局化。该机制位于
`src/optimal_power_flow/parity_ipm.cpp`，在线性求解完成后、状态更新前执行，
因此覆盖 UMFPACK、KLU、Eigen SparseLU 和 Dense LU 全部底层后端。

同时把 KKT 分解次数、回代次数、接受步和拒绝步写入 `ACOPFProfiling`，测试和
GUI/API 调试时可以直接区分“线性代数失败”和“全局化拒绝”。

## 5. 修复后的内部证据

`case300_acdc` 使用原生 parity IPM + UMFPACK 时：

- 42 次迭代收敛；
- 最大约束残差约 \(1.74\times10^{-7}\)；
- 最大平稳性残差约 \(2.77\times10^{-7}\)；
- 41 次接受步、17 次回溯拒绝；
- 41 次 KKT 分解、158 次预测/校正/Gondzio 回代。

强制 UMFPACK、KLU、Eigen SparseLU 和 Dense LU 四种后端均通过同一大型
富混合算例，说明收敛不再依赖单一线性求解库。

## 6. 时序热启动

`ACOPFOptions::warm_start` 可接收上一时段的兼容 `ACOPFResult`。结果中除
Vm/Va、Pg/Qg、Vdc、VSC、DER、储能、DC/DC、柔性负荷和能量路由器等物理
变量外，还保留原生 IPM 的等式乘子、不等式乘子和松弛变量。维数和正性检查
全部通过时，下一次求解复用完整原始—对偶中心路径状态；任何状态块不兼容时，
自动回退到物理初值和冷启动乘子。

在相同 `case300_acdc` 上，冷启动为 42 次迭代，完整原始—对偶热启动为 2 次，
初始原始残差从约 (4.46\times10^{-2}) 降至 (1.23\times10^{-6})。时序 OPF
已自动把上一时段收敛结果传给下一时段。

## 7. Ipopt 参数透传

`MIPSolvers::NLPModel` 现在携带 `NLPSolverOptions`，嵌入式
`IpoptAdapter` 使用其中的 `max_iterations`、`tolerance` 和
`acceptable_tolerance`，不再固定为 500、$10^{-8}$ 和 $10^{-6}$。
`solve_parity_with_ipopt()` 将 `ACOPFOptions` 形成的总 IPM 迭代上限直接传给
`max_iterations`，将原始、对偶和互补容差中的最小值作为严格 `tol`、最大值
作为 `acceptable_tol`。这样原生 IPM 与 Ipopt 接收同一组调用方预算和停止精度。

macOS arm64 的默认组合为“仓库内嵌 Ipopt + Homebrew MUMPS 动态库”。通过
`MIPSOLVERS_FORCE_BUILD_MUMPS=ON` 可以从已缓存的 MUMPS 5.7.3 源码构建本地
静态库；该配置已能完成编译和链接，但运行适配器测试时在第 1 次迭代进入
`Ipopt restoration failed`，而相同内嵌 Ipopt 配合 Homebrew MUMPS 可在 11 次
迭代收敛。因此在修复 arm64 静态 MUMPS 的运行时数值或 ABI 问题前，不把本地
MUMPS 设为默认后端。

## 8. 2000 节点富混合系统实测

使用仓库内真实 `case_ACTIVSg2000`，并通过 `build_case2000_acdc()` 附加多端
直流网络和 8 个 VSC。Release 构建下的单次冷启动结果为：

| 后端 | 预算/迭代 | 墙钟时间 | 最大约束违反 | 平稳性残差 | 结果 |
|---|---:|---:|---:|---:|---|
| 原生 parity IPM + sparse UMFPACK | 90 次 | 7.07 s | $3.49\times10^{-9}$ | $9.03\times10^{-7}$ | 收敛 |
| 内嵌 Ipopt + Homebrew MUMPS | 100 次预算 | 48.15 s | $4.96\times10^{-4}$ | 1.49 | 未收敛 |

原生 IPM 共执行 89 次 KKT 分解和 346 次预测、校正及 Gondzio 回代。Ipopt 的
100 次有界基准保持数值有限，但在耗时约为原生路径 6.8 倍后仍未达到原始或
对偶停止条件。因此当前 2000 节点富混合系统应优先使用利用网络稀疏结构的
原生 IPM；Ipopt 保留为小中型问题后端和独立交叉验证路径。

## 9. 边界

该修复解决的是普通经济 AC/DC OPF 的原生 IPM 全局化。RPO 的电压偏差/网损
目标目前仍优先使用嵌入式 Ipopt filter-line-search 作为连续 NLP 后端；其离散
层仍是局部搜索，不构成全局 MINLP 证明。三相不平衡单体 RPO 和富混合设备的
完整 authored-space PF 写回仍按覆盖审计标记为部分支持。
