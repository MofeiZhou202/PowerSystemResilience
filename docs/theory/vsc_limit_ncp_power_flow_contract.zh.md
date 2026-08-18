> 本文档为 [vsc_limit_ncp_power_flow_contract.md](vsc_limit_ncp_power_flow_contract.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# VSC 限流 NCP 潮流契约

本活文档契约定义了生产统一 Newton 潮流路径所使用的平衡正序 VSC（电压源换流器）
限流模型。源码与已注册测试仍为最终权威依据。

## 适用范围

该模型通过在役的 `VSCConverter` 以
`enable_limit_ncp=true` 且 `i_ac_max_pu>0` 按设备启用。它支持并网或
无平衡节点（slackless）AC 孤岛中的 `PQ_MODE`、`VDC_Q`
和 `AC_GRID_FORMING` 模式。每台准入换流器拥有固定的
标幺局部状态

\[
z_c=(P_{ac},Q_{ac},P_{dc},E_r,E_i,\lambda)_c.
\]

这些功率直接进入 AC-P、AC-Q 与 DC-P 平衡方程。结果使用稳定的换流器
`.index`，绝不使用向量位置。`PQ_MODE` 与 `VDC_Q` 使用
恒等行 `E_r=E_i=0`；构网型（GFM, grid-forming）则将这些槽位用于虚拟阻抗后的
内部电压。这是一个稳态/准稳态的限流后运行点，
而非快速内环电流环、PWM 或 EMT 开关暂态。

## 功率端口方程

对于 `PQ_MODE`，`P_ref=P_set/S_base` 且 `Q_ref=Q_set/S_base`。对于 `VDC_Q`，

\[
P_{raw}=k_{vdc}(V_{dc}^2-(V_{dc}^{set})^2),\qquad
P_{ref}=\operatorname{clip}(P_{raw},P_{min},P_{max}).
\]

只要 `droop_p_min_mw` 或 `droop_p_max_mw` 中任一非零就使用该下垂对；
仅当下垂对全为零时才回退到 `pmin_mw/pmax_mw`
（`effective_lower_mw`/`effective_upper_mw`，
`src/power_flow/vsc_limit_ncp.cpp:90-102`）。仅当有效上限大于下限时才执行
clip；无序或零宽的有效上下限对会禁用钳制，`P_ref=P_raw` 原样通过
（`vsc_limit_command`，`src/power_flow/vsc_limit_ncp.cpp:205-210`）。
饱和导数在开区间之外为零，在区间之内为 `2*k_vdc*Vdc`。

### 面向内置基准数据的下垂参数整定

对于生产的电压平方律，一台预期在电压幅值下垂带宽 `Delta V`
内调节 `P_rated_pu` 的换流站采用

\[
k_{vdc}=\frac{P_{rated,pu}}{1-(1-\Delta V)^2}.
\]

`Defaults::kVdcDroopRatedVoltageDeviation=0.05` 是标准内置数据带宽的
唯一来源。在 `case2000_acdc` 中，四个定 P 换流站各吸收
`0.5 pu`，四个 Vdc-Q 换流站分担该转移功率，因此每个调节站
采用 `k_vdc=0.5/[1-(1-0.05)^2]`。换流器与线路损耗可能使解出的
电压略超名义 5% 点，而已注册用例仍
要求所有直流电压保持在生产 `[0.9,1.1] pu`
合格区间内。该整定属于用例数据构造，
而非依赖迭代的求解器覆写。

AC 电流圆盘为

\[
g(P,Q,V_m)=(V_m I_{max})^2-P^2-Q^2\ge0.
\]

幅值优先采用

\[
(1+\lambda)P-P_{ref}=0,\quad
(1+\lambda)Q-Q_{ref}=0,\quad
\Phi_{FB}(\lambda,g)=0.
\]

P 优先与 Q 优选对剩余电流圆半径使用中位数残差。
在零半径并列处选择一个有限的广义 Jacobian 分支。
所有优先级共享

\[
P_{dc}+P_{ac}+P_{loss}(P_{ac},V_{dc})=0,
\]

损耗导数取自已选生产 `LossModelType` 的解析表达式。

目标互补函数是精确的 Fischer-Burmeister 映射

\[
\Phi_{FB}(a,b)=\sqrt{a^2+b^2}-a-b.
\]

特别地，`Phi_FB(0,0)=0`；没有任何隐藏的 epsilon 扰动双激活
交点。在原点处，实现选取对称的 Clarke
广义导数
`(1/sqrt(2)-1, 1/sqrt(2)-1)`。这是用于
最终证书的默认半光滑方程。

## 构网型方程族

对于端电压 `V=Vm*exp(j*Va)`、内部电压 `E=Er+j*Ei` 与
虚拟阻抗 `Zv=Rv+j*Xv`，

\[
I=(E-V)/Z_v,\qquad S_{ac}=VI^*=P_{ac}+jQ_{ac}.
\]

`gfm_internal_voltage_set_pu`、`gfm_internal_angle_set_deg`、
`gfm_virtual_r_pu` 与 `gfm_virtual_x_pu` 由潮流（PF）、OPF 适配器与
暂态初始化共享。为零的著录值保留遗留的
`v_ac_set_pu`、`v_ac_angle_set_deg`、`r_conv_ac_pu` 与 `x_sc_pu` 回退值。
所有消费方调用 `model::resolve_gfm_norton_parameters`；完整的
跨模块优先级、单位、恒等式与结果规则定义于
`docs/developer/model_data_semantics_contract.md`。

幅值限制使用二维回退形式

\[
(1+\lambda)(E-V)=E^0-V,\qquad
0\le\lambda\perp I_{max}^2-|I|^2\ge0.
\]

P 优先与 Q 优先在端电压同步 dq 坐标系中运行。
对全部六个局部状态、端 `Vm`、
端 `Va` 以及 `Vdc` 的广义导数均为解析式。

## 光滑延拓

当启用 `RobustNonlinearOptions::enable_smooth_ncp` 时，VSC 块对
幅值优先使用 Kanzow 光滑化半径

\[
\Phi_\mu(a,b)=\sqrt{a^2+b^2+2\mu^2}-a-b
\]

并对 P 优先与 Q 优先使用 CHKS 光滑化的 min/max 算子。
该选项独立于遗留的 `enable_semi_smooth_newton` PV/PQ 开关作用于
VSC 限流块。当发电机与换流器互补方程同时存在时，
共享同一条 `mu` 调度。

`mu` 从 `ncp_mu0` 递减至 `ncp_mu_min`；每次递减后，
残差与 Jacobian 在不变的状态处重新装配。延拓
只改变数值，因此六状态 VSC 布局与稀疏坐标保持
不变。公开的 `VSCTransfer`/`VSCLimitState` 证书以
`mu=0` 重新评估，防止光滑代理残差被报告为精确的
互补性。

## 稀疏与结构契约

网络残差为 `spec-calc`，而存储矩阵遵循
既有的 `d(calc-spec)/dx` 约定。局部方程存储 `dF/dx` 并使用
右端项 `-F`。每台启用的换流器恰好增加六行六
列。切换激活状态只改变数值，绝不改变坐标，因此一次符号
分析可复用于多次数值重分解。

装配成本保持为

\[
O(nnz(Y_{ac})+nnz(G_{dc})+n_{VSC}),
\]

而分解成本取决于填充元。本契约未实现 GPU 装配或 GPU 稀疏
分解。

### 精确局部块 Schur 消元

对于 `m` 个准入的 VSC 块，所选广义 Newton 矩阵被
划分为网络变量 `x` 与换流器局部变量 `z`：

\[
J=\begin{bmatrix}A&B\\C&D\end{bmatrix},\qquad
D=\operatorname{blkdiag}(D_1,\ldots,D_m),\quad D_i\in\mathbb R^{6\times6}.
\]

`VSCLocalSchurSolver` 计算

\[
S=A-BD^{-1}C,\qquad
\widehat r_x=r_x-BD^{-1}r_z,
\]

求解 `S dx = rhat_x`，并恢复

\[
dz=D^{-1}(r_z-Cdx).
\]

不构造任何稠密逆。每个 6×6 分解求解四个右端项
`[C_i,r_i]`。每个块仅耦合其端 `Va/Vm/Vdc` 列
与其 AC-P/AC-Q/DC-P 行，因此一个块最多贡献九个 Schur 更新
坐标。约简稀疏模式固定，且只做一次符号分析。

在局部尺寸 `s=6`、至多 `c=3` 个端网络坐标下，局部
工作量为

\[
O\!\left(ms^3+ms^2(c+1)\right)=O(m),
\]

且约简维数恰好为 `n_x` 而非 `n_x+6m`。每台换流器的完整
固定模式含 36 个局部、18 个局部到网络、3 个网络到局部非零元。
约简更新最多增加 9 个非零元，因此输入稀疏
结构至少减少 `48m` 个非零元。稀疏 LU 填充与工作量仍然
依赖图结构与排序，以实测为准，而不赋予 `O(n^1.5)`
的论断。

生产路径要求 `network_nvar >=
Defaults::kVSCSchurMinNetworkDimension`（当前为 1000）。将该阈值
设为零属于研究性消融实验。启用标志、准入阈值、局部
`rcond` 与后向误差容差均可通过 C++
`PowerFlowOptions` 与生产潮流 HTTP `robust_nonlinear` 对象配置；
生效值由 GUI 后端返回。机器 epsilon 本身是一个
只读的表示常量。无效的无量纲证书容差在每个生产边界处
恢复其具名默认值。

该守卫是基于证据的：在
case300 上强制 Schur 将维数 `640 -> 604`、结构非零元 `4820 -> 4502`
降低，但因局部证书开销在小型 KLU
分解中占主导，按当前固定协议的中位线性/墙钟时间反而增加
`31.36%/32.94%`。
ACTIVSg2000 将维数与非零元降低为 `4054 -> 4006` 与 `29806 -> 29382`，中位
线性/墙钟时间降低 `13.75%/10.55%`；它被生产默认值
准入。相对此前的 `13.85%/11.99%` 结果，当前
大算例加速比低 0.10/1.44 个百分点；这一微小
回退不改变符号，也未触发固定的 50% 偏差
重新推导阈值。

每个候选步检查每个所选 `D_i` 的逆条件数估计、
约简后的范数型后向误差，以及重构完整系统的
后向误差。`Defaults::kVSCSchurLocalRcondTol` 由全局
`NumericalConstants::kSqrtMachineEpsilon` 导出；完整步默认值为
`Defaults::kVSCSchurBackwardErrorTol`。被拒绝的局部块、约简求解
或重构步将回退到普通的全稀疏 LU。公开的性能
画像区分尝试次数、接受步数、拒绝原因、维数、
结构非零元、接受块的最小 `rcond`、接受步的最大
后向误差，以及半光滑残差速率商。

### BD 正则性与局部速率

对于一个所选广义 Jacobian 元素，由块 Gauss 消元可知，所有 `D_i`
与 `S` 的非奇异性蕴含 `J` 的非奇异性。这是
Schur 路径所检查的可执行充分条件。它本身
并非 BD 正则性：BD 正则性要求解邻域内
B-次微分的所有相关元素均非奇异。在该更强
条件与半光滑残差下，半光滑 Newton 局部
超线性收敛；在强半光滑性下，标准局部速率为二次。

运行时报告 `||F_{k+1}||/||F_k||` 与
`||F_{k+1}||/||F_k||^2` 作为实测局部速率证据。它不会把
有限样本或单个所选 Jacobian 证书转化为对整个
Clarke 集合的证明。退化的 P/Q 优先级交点可能使所选 `D_i`
病态；这些迭代预期使用全 LU 回退。

当电流限值非正、控制
模式不受支持、GFM 虚拟阻抗为零、所需 AC-P/AC-Q/DC-P
行缺失，或结构扫描发现空行/空列时，准入以失败关闭
方式终止。并网 GFM 端仍是一个被求解的 PQ 母线。在无
端 SLACK/外部电网的 AC 孤岛中，所有母线 `Vm/Va` 状态与所有 P/Q 平衡仍
保留在 Newton 系统中。Norton 方程内部的著录固定内部相量
`E*exp(j*delta*)` 打破了全局旋转零模；不固定任何端
角度，也不创建合成 SLACK 母线。多个固定 Norton
源可共享同一孤岛：其内部相量与虚拟阻抗
决定环流与功率分配，因此它们并非多个刚性
端参考。已被类型化为 `SLACK` 的 GFM 端将被拒绝。

`IslandInfo::has_ac_slack` 保留其端参考含义。
`has_ac_angle_reference` 额外识别固定 GFM 内部相量，
而 `gfm_reference_vsc_indices` 携带稳定的著录换流器 ID。
自适应孤岛提取保留这些 ID 与母线类型。严格混合
协调另需一个物理的直流电压/功率平衡源；
仅有 `DC_V` 母线标签是不够的。

## 跨模块语义

`PowerFlowResult::vsc_limit_states` 与 `VSCTransfer` 暴露稳定 ID、
功率/电流数据、优先级、激活与下垂状态、互补
残差、`gfm_norton_model`、内部电压实/虚分量，以及
端电流实/虚分量。GFM 求解在
`ConverterModelScope` 中设置
`vsc_gfm_norton_modelled` 与 `vsc_gfm_priority_limit_enforced`。
无平衡节点求解额外设置
`vsc_gfm_island_reference_modelled` 并返回稳定的
`gfm_island_reference_vsc_indices` 诊断信息。

平衡 OPF 将 GFM 换流器视为自由 P/Q 端口，并施加其选定的
工程不等式。内部电压、虚拟阻抗与优先级
NCP 不是 OPF KKT 约束。时间序列与 GUI `post_pf` 重放保留
著录的 GFM 模式并返回生产潮流证书；该重放
不得被描述为内生的 OPF GFM-NCP 优化。

暂态初始化消费同一份稳定 ID 潮流证书，并在动态网络
重解之前检查内部电压、电流与功率 Norton 种子。
后续的全动态配平可能移动到另一个平衡点，因为
动态源与负荷不必精确复现潮流平衡节点的调度。

## 验证阈值

`tests/test_vsc_limit_ncp.cpp` 与 `tests/test_transient_dynamics.cpp` 强制要求：

- 局部/预言机（oracle）与非约束根误差不超过 `1e-9`；
- 互补、电流圆与能量残差不超过 `1e-9 pu`；
- 解析 GFM 广义 Jacobian 的有限差分误差不超过 `5e-7`；
- 精确 FB 原点残差等于零，双激活整块残差不
  超过 `2e-15`；
- 光滑 FB 与光滑化优先级 Jacobian 的有限差分误差不
  超过 `5e-7`；
- 布局固定且 `jacobian_pattern_rebuilds == 1`；
- Schur/全 LU 预言机步相对误差不超过 `1e-10`，完整系统
  后向误差不超过 `1e-12`；
- 奇异所选 `D_i` 的拒绝与生产全 LU 回退；
- case300/case2000 的 Schur 与全 LU 解的 `Vm/Va/Vdc` 差异不超过 `1e-8`；
- case300/case2000 的精确 Schur 维数降低为 36/48，结构非零元
  降低至少为 288/384；
- 潮流到暂态的 Norton 种子误差不超过 `1e-8`；
- OPF 到潮流重放证书残差不超过 `1e-9`。

该测试套件覆盖所有优先级、两个同时生效的 GFM 限值、退化、
弱电网与不可行用例、准稳态重放、OPF 重放、暂态
初始化、JSON/JPC 往返、稳定 ID、单个无平衡节点 GFM 孤岛、
一个无平衡节点孤岛中的多个 Norton 源，以及不制造
SLACK 母线的自适应多孤岛提取。光滑延拓集成
覆盖全部三种功率端口优先级与全部三种限流 GFM
优先级，最终 `mu <= 1e-12` 且精确证书残差不
超过 `1e-9`。

这些是围绕已认证高电压解的局部半光滑 Newton 保证。
非线性孤岛方程还可能存在低电压数学
根；足够远的旋转种子已展示出一个不同的吸引域。
因此本契约不声称全局唯一性或全局收敛性。

OpenDSS 验证有两个证据层级。在非约束点，独立求解一个
方程等价的双源 Thevenin 电路，容差为
`Vm <= 1e-4 pu`、角度 `<= 0.02 deg`、`P/Q <= 1e-3 pu`。在
约束点，OpenDSS 没有等价的优先级 NCP 控制器，因此冻结 HySim
内部电压并由 OpenDSS 独立重解该电路；
其原生端功率 KCL 残差必须不超过 `1e-6 pu`。后者是
冻结电路验证，而非独立的模式选择预言机。

`build_gfm_norton_limit_demo()` 是并网平启动的生产/GUI 用例。它具有
独立的端 AC 角度参考、一个提供物理直流
电压/能量支撑的 VDC-Q 换流站，以及一个显式的 GFM Norton/限流
NCP 换流站。

`build_case300_acdc_vsc_limit_ncp()` 与
`build_case2000_acdc_vsc_limit_ncp()` 仍是版本化的 MTDC/PQ-Vdc-Q CPU 稀疏
基准：300 AC + 6 DC + 6 VSC 与 2000 AC + 8 DC + 8 VSC，各有四个
同时生效的限值。`build_case300_acdc_gfm_limit_ncp()` 与
`build_case2000_acdc_gfm_limit_ncp()` 增加一个非约束 GFM Norton 块，
并从相应已解出的 MTDC 状态出发作为显式
延拓数据集进行认证。它们防止纯 AC 可扩展性证据被
误报，但不证明平启动鲁棒性、大规模同时
GFM 约束生效，或 GPU 加速。

普通的 `build_case2000_acdc()` 内置用例单独针对
非 NCP 生产混合 Newton 路径注册。在与 GUI 等价的 `tol=1e-8`、
`max_iter=100`、启用换流器协调且显式禁用自动回退
的条件下，它必须以不超过 `1e-8` 的残差收敛，且所有直流
电压位于 `[0.9,1.1] pu`。纯 AC 收敛不被接受为该
证书。

`vsc_schur_benchmark` 是非 CTest 的 Release 基准。其固定协议为
两次预热加五次全量/Schur 交替重复。它将 case300 标记为
强制性研究消融，并记录生产准入排除它的原因。
在仓库 `1773aa0e75d7`、MIPSolvers `3bf1e66749e3` 的当前
macOS 26.5.2 / Apple M4 Max 干净 Release/KLU 构建上，case300 测得
全量/Schur 线性时间 `0.524958/0.689582 ms`，
全量/Schur 墙钟时间 `2.522875/3.353792 ms`。ACTIVSg2000 测得线性时间
`31.979500/27.581291 ms`，墙钟时间
`47.372583/42.373500 ms`，其中 33 次尝试、19 次接受、14 次
回退 Schur 步。
当前 KLU 适配器对分解非零元/工作量返回 `-1`，因此该基准
设置 `factor_statistics_available=false` 而非编造填充数据。

## 数学参考文献

- F. Facchinei and J.-S. Pang, *Finite-Dimensional Variational Inequalities
  and Complementarity Problems*, Vol. I, Springer, 2003, Sec. 9.1.
- L. Qi and J. Sun, "A nonsmooth version of Newton's method," *Mathematical
  Programming*, 58, 1993, pp. 353-367.
- A. Fischer, "A special Newton-type optimization method," *Optimization*,
  24, 1992, pp. 269-284.
- C. Kanzow, "Some noninterior continuation methods for linear
  complementarity problems," *SIAM Journal on Matrix Analysis and
  Applications*, 17(4), 1996, pp. 851-868.
- G. H. Golub and C. F. Van Loan, *Matrix Computations*, 4th ed., Johns
  Hopkins University Press, 2013, block Gaussian elimination.
- T. A. Davis, *Direct Methods for Sparse Linear Systems*, SIAM, 2006.
- N. J. Higham, *Accuracy and Stability of Numerical Algorithms*, 2nd ed.,
  SIAM, 2002, Secs. 2.2 and 7.1.
