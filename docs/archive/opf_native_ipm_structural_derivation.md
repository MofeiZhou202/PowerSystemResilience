# OPF Native IPM 结构化加速推导

本文把 `docs/todo/OPF定制化加速技术方案.md` 中可用的方向收敛为可证明、
可证伪、可直接映射到代码的算法合同。目标不是按案例调整参数，而是由 OPF
的变量块、等式因子图、数值秩、当前缩放和方向质量自动选择实现模块。

## 1. 问题、精度与禁止项

考虑原模型

$$
\min_x f(x),\qquad g(x)=0,\qquad h(x)\le 0,\qquad l\le x\le u.
$$

Parity OPF 当前实现的原始变量顺序为 19 个连续块（早期手册中的“17 块”
计数未包含后来追加的 energy-router 两块）：

$$
x=[\theta,V,P^g,Q^g,V^{dc},P^{ac},Q^{ac},P^{dc},
\Delta P^d,\Delta Q^d,P^{ren},Q^{ren},P^{stor},Q^{stor},
P^{stor,dc},P^{dcdc},P^{flex},P^{er},Q^{er}],
$$

等式顺序为

$$
g=[g^P_{ac},g^Q_{ac},g^P_{dc},g^{conv},g^{dcdc},g^{er},
g^{\theta ref},g^{Vdc ref}].
$$

其中 `g_dcdc` 在当前实现中长度恒为零；$P^{dcdc}$ 直接进入输入、输出两端的
DC 节点平衡。分区算法不得为一个只存在于布局注释中的空块制造 pivot 或空行。

所有成功结论必须在恢复后的原变量、原约束尺度上满足共同 OPF 合同

$$
\|r_p\|_\infty\le10^{-6},\quad
\|r_d\|_\infty\le10^{-5},\quad
\|r_c\|_\infty\le10^{-6}.
$$

案例名、节点数查表、随机扰动、放宽原模型证书和外部 fallback 均不属于算法。
LiftedKKT 的有限等式松弛会改变可行域，只能作为预条件或早期近似方向，不能
直接提交为严格 OPF 解。本阶段不采用该变形。

## 2. 等式/控制变量分区

### 2.1 分区定义

令 $m=\dim g$，$n=\dim x$。从列集合中选择 $m$ 个状态列 $S$，其余
$n-m$ 个列为独立控制列 $C$：

$$
x=(x_s,x_c),\qquad J_g=[J_s\;J_c],\qquad J_s\in\mathbb R^{m\times m}.
$$

当 $J_s$ 非奇异时，隐函数定理给出局部状态映射 $x_s=\psi(x_c)$，以及

$$
\frac{\partial x_s}{\partial x_c}=-J_s^{-1}J_c,qquad
Z=\begin{bmatrix}-J_s^{-1}J_c\\I\end{bmatrix},qquad J_gZ=0.
$$

$Z$ 是等式切空间的一组基。于是只应在该空间检查非凸曲率：

$$
W_r=Z^TWZ.
$$

若 $J_g$ 满行秩，则总存在某个非奇异的 $m\times m$ 列子式；反之任何分区
都无法得到非奇异 $J_s$。因此“存在合法分区”等价于当前点满足 LICQ 的等式
部分。分区提示只负责更快地找到好的列子式，不能替代数值秩判断。

### 2.2 为什么不能按变量块生硬切分

纯 AC 模型有 $2n_{ac}$ 个 P/Q 平衡行和每个导电岛一个角参考行，而
$(\theta,V)$ 只有 $2n_{ac}$ 列。因此只取电压状态列必然不是方阵。每个 AC
岛至少还需要一个能影响该岛守恒行的执行器列，例如一台在线发电机的 $P^g$
或等价有功控制列。

混合模型还包含以下相同现象：

- DC 平衡与 $V^{dc}$ 参考同时存在时，仅用 $V^{dc}$ 列不足；
- 换流器等式同时连接 $P^{ac},P^{dc},V,V^{dc}$，必须在耦合分量中保留
  至少一个可消元的换流器功率列；
- energy-router 守恒行必须由某个端口功率列取得 pivot；
- 显式 reference row 与同一变量的固定 bound 若同时存在，会制造重复定义，
  必须先做固定变量消元/重复行检查，再形成最终分区。

所以 OPF 分区采用因子图上的带优先级最大匹配，而不是固定块清单。

### 2.3 结构匹配规则

构造二部图 $\mathcal G=(R,X,E)$：左侧是等式行，右侧是未固定变量，若
$J_{g,ij}$ 的数学稀疏模式中存在边则 $(i,j)\in E$。寻找覆盖全部等式行的
匹配 $M$。匹配列组成 $S$，未匹配列组成 $C$。

匹配优先级由方程物理角色确定：

1. P/Q 平衡优先匹配本岛的 $\theta,V,V^{dc}$ 状态列；
2. reference row 优先匹配它直接定义的参考状态列；若该列已被消元，则该行
   应代入后删除，而不是再次匹配；
3. converter/DC-DC/router 守恒优先匹配相应耦合功率列；
4. 只有为补足每个守恒分量的自由度时，才把 $P^g,Q^g,P^{ac},Q^{ac}$ 等
   执行器移入 $S$；其余执行器留在 $C$，成为真正优化控制自由度；
5. 同一层内以预计消元填充和列尺度作为代价，禁止使用案例身份。

这是一种结构提示，不是证书。数值阶段必须依次通过：

$$
\operatorname{rank}(J_s)=m,qquad
\frac{\|J_gZ\|_\infty}
 {1+\|J_g\|_\infty\|Z\|_\infty}
 \le c_Z\epsilon_{mach},
$$

并用 QR 的最小对角或一范数条件估计拒绝近奇异 $J_s$。$c_Z$ 随维度采用
$O(m)$ 的舍入误差界，而不是案例阈值。提示失败时回退到通用带列主元 QR；
若 QR 也判定 $J_g$ 行秩不足，则进入 $\delta_C$/restoration 路径，不能伪造
null-space certificate。

### 2.4 与算法模块的匹配

- $n-m$ 小且 $J_s$ 条件良好：显式构造稠密小型 $W_r$，精确给出切空间
  最小曲率和所需 $\delta_W$；
- $n-m$ 大：不显式形成 $Z$，使用增广 LDLT 的直接惯性；分区仍可用于排序、
  预条件和控制/状态尺度；
- $J_s$ 近奇异但 $J_g$ 满行秩：丢弃物理提示并 QR 重选列；
- $J_g$ 数值行秩不足：保留原等式，使用有证书的双正则化或可行性恢复。

## 3. 固定变量精确消元

### 3.1 约化模型

定义固定集合

$$
F=\{i:l_i=u_i\},\qquad R=\{1,\ldots,n\}\setminus F,
$$

并令 $x_F=\bar x_F$。只对严格相等的有限 bounds，或满足模型输入精度合同
的相等 bounds 消元；不能把“很窄”擅自视为固定。约化 NLP 为

$$
f_R(x_R)=f(P_Rx_R+P_F\bar x_F),
$$

$$
g_R(x_R)=g(P_Rx_R+P_F\bar x_F),\qquad
h_R(x_R)=h(P_Rx_R+P_F\bar x_F).
$$

链式法则直接给出

$$
\nabla f_R=P_R^T\nabla f,\quad
J_{g,R}=J_gP_R,\quad J_{h,R}=J_hP_R,
$$

$$
\nabla^2_{RR}L_R=P_R^T\nabla^2_{xx}L P_R.
$$

因此实现只是 callback 输入扩展、向量 gather 和稀疏矩阵取子行列，没有模型
近似。固定列不再产生上下界两条 barrier inequality，也不进入 KKT 图。

### 3.2 行代入与冗余

列消元后，若某个等式/不等式完全不含自由变量：

- 常数等式残差在原尺度容差内，删除该行并记录映射；否则原模型不可行；
- 常数不等式满足，删除；否则原模型不可行。

本阶段首先实现列消元；行删除只有在 callback pattern 和原尺度残差均已验证
时执行。特别地，reference row 在参考变量已固定后应成为可验证常数行，避免
同时以 equality 和双 bound 重复进入系统。

### 3.3 乘子与原空间证书恢复

约化求解得到 $x_R,\lambda,\nu$ 后，先恢复完整原始点。自由变量 stationarity
就是约化 stationarity。固定变量的拉格朗日梯度记为

$$
q_F=\nabla_F f+J_{g,F}^T\lambda+J_{h,F}^T\nu.
$$

固定点的 normal cone 是整个实轴，bound dual 不唯一。采用最小一范数分裂

$$
z^L_i=\max(q_i,0),\qquad z^U_i=\max(-q_i,0),
$$

在约定 stationarity
$\nabla f+J_g^T\lambda+J_h^T\nu-z^L+z^U=0$ 下恰好消去 $q_i$。
该恢复不改变目标或可行性；固定 bounds 的互补积恒为零。审计仍重新调用原始
callbacks，不能用约化残差替代原空间残差。

### 3.4 所属层

固定变量消元是通用 NLP 等价变换，应位于 MIPSolvers `NativeIPMAdapter`
入口，而不是仅在 Parity 中复制。Parity 只提供物理分区提示；通用层维护
`reduced_to_original/original_to_reduced`、callback 包装和 postsolve。这样同一
NLPModel 送入 Ipopt 时仍保持原模型，Native 的加速也不会改变公平比较边界。

Parity 当前还存在三类需要明确区分的 bounds：退运机组使用 `p0/q0 +/- 1e-8`，
无导电支路的 DC 电压使用约 `1e-6` 的锚定带，单时段储能调度使用
`1e-4/baseMVA` 的窄带。这些都不是数值意义上的 `lb==ub`，Native 通用层不得
猜测消元。若建模语义确实声明“不可优化”（例如退运机组、静态充电策略），
Parity 应把它改写成精确相等 bounds；若窄带代表允许的真实调节范围，则必须
保留为两条不等式。这个决定来自组件语义，不来自区间宽度阈值。

## 4. 数值分解次数和复用下界

### 4.1 Newton 矩阵生命周期

对固定迭代点 $(x,s,\lambda,\mu)$ 和固定正则化 $(\delta_W,\delta_C)$，Newton
矩阵 $K$ 不变。一次数值分解后，predictor、centering/corrector、SOC 和线性
精化只改变 RHS。因此正确成本合同是

```text
matrix values changed       -> numeric factorization
only RHS changed            -> triangular backsolve
CSC pattern hash changed    -> symbolic analysis + numeric factorization
pattern unchanged           -> reuse ordering/symbolic, refill values only
```

在不需要惯性重试的常规 IPM 主迭代中，数值分解次数的下界是每个不同 Newton
矩阵一次，而不是每个方向一次。若一次迭代中 predictor/corrector/SOC 各自触发
factor，属于实现错误或矩阵实际上被修改，必须在遥测中明确归因。

### 4.2 合法的额外 factor 来源

仅以下事件允许增加 numeric factor：

1. 惯性或方向质量证书失败后改变 $\delta_W/\delta_C$；
2. condensed 精度证书失败，切换到数学等价的 augmented 公式；
3. restoration 产生新的目标/Hessian 或新迭代点；
4. 接受 trial point 后进入下一主迭代，导数数值变化；
5. active-set polish 改变工作集和 KKT 图。

原尺度 residual refinement、predictor/corrector、SOC 与 watchdog trial 只允许
增加 solve。symbolic pattern 改变必须单独记录；Parity Hessian 已固定数学
pattern，正常主轨迹不应再次 symbolic analysis。

### 4.3 分解成本模型

对排序 $p$，符号阶段给出 $nnz(L_p)$ 和估计 flops $F_p$。运行时近似成本为

$$
T\approx N_{sym}T_{sym}(p)+\sum_k N_{fac,k}T_{fac,k}(p)
+N_{solve}T_{tri}(p)+T_{eval}+T_{asm}.
$$

因此 hybrid2000 的优化目标首先是减少失败轨迹造成的 $N_{fac}$，其次才是减少
单次 $T_{fac}$。所有 benchmark 必须同时报告 `iterations/symbolic/factor/solve`
及 factor 总耗时；只报告总墙钟无法区分轨迹问题和排序问题。

## 5. 首步惯性正则化与方向质量

### 5.1 惯性只是必要条件

对等式 KKT

$$
K(\delta_W,\delta_C)=
\begin{bmatrix}W+\delta_WD_x&J_g^T\\J_g&-\delta_CD_g\end{bmatrix},
$$

当 $J_g$ 满行秩且 $Z^T(W+\delta_WD_x)Z\succ0$ 时，目标惯性成立。但该定理
只说明局部二次模型在等式切空间有下降曲率，不保证：

- 线性系统求解足够准确；
- $\|D_x^{-1}d_x\|$ 位于局部模型可信区域；
- slack/dual 的 fraction-to-boundary 不坍缩；
- 非线性约束在该步长上仍由一阶模型准确描述。

hybrid2000 已出现“惯性正确但 $\|d_x\|_\infty=O(10^5)$、可行步长
$O(10^{-8})$”的反例，所以必须增加方向质量门。

### 5.2 四重方向证书

在首次或任何强非凸迭代中，方向被线搜索使用前必须满足：

1. **惯性证书**：直接惯性正确，或 $J_s/Z$ 的数值证书与
   $\lambda_{min}(Z^TWZ)+\delta_W>0$ 同时成立；
2. **线性后向误差**：
   $$
   \eta_K=\frac{\|Kd-r\|_\infty}
   {1+\|K\|_\infty\|d\|_\infty+\|r\|_\infty}
   \le c_K\epsilon_{mach};
   $$
3. **barrier 模型下降**：投影梯度与方向满足
   $$
   q'(0)=\nabla\varphi_\mu(x,s)^Td
   \le-\eta_d\|D^{-1}r_d\|_2\|Dd\|_2;
   $$
   若梯度接近舍入误差，则用二次模型预测下降替代，不能除以近零量；
4. **信赖域/边界门**：定义尺度化方向
   $$
   \rho_d=\|D_x^{-1}d_x\|_\infty/\Delta,
   $$
   并计算 fraction-to-boundary $\alpha_{ftb}$。若 $\rho_d>1$ 或
   $\alpha_{ftb}\rho_d$ 仍远大于 1，说明 Newton 模型超出可信域。

$\eta_d$、$c_K$ 由浮点误差界和维度确定；$D_x$ 来自现有变量缩放。
$\Delta$ 不是案例参数：初值由当前 scaled residual 与 Jacobian 线性化误差确定，
接受步后按实际/预测下降比更新。

### 5.3 正则化更新的推导

若线性后向误差失败，先做同一 factor 的 refinement；失败后切换公式或后端，
不能用更大 $\delta_W$ 掩盖线性求解错误。

若惯性通过但信赖域/下降门失败，解约束 trust-region 子问题的最小标量近似：

$$
(W+\delta_WD_x^TD_x)d+J_g^Td_\lambda=-r_d.
$$

由 $\|D_x^{-1}d\|$ 对 $\delta_W$ 单调下降，采用 bracket + 几何/二分更新
$\delta_W$，直到方向进入 $\Delta$。每次只 refill 对角并重做 numeric factor；
symbolic 必须复用。初始 bracket 由切空间最小曲率和
$\|D_x^{-1}r_d\|/\Delta$ 给出，不依赖案例名称。

首步特别重要：若初始原始 infeasibility 很大，barrier Hessian 尚不能代表可行
流形，先以较小 $\Delta$ 限制电压/功率的尺度化变化；随着滤波器实际下降比
稳定，再放大 $\Delta$。这比在首步固定设置 `delta_w=0.01` 有理论依据，也避免
“惯性刚好通过但方向巨大”的轨迹。

## 6. 算法状态机

```text
original NLP
  -> exact fixed-column elimination and constant-row audit
  -> OPF weighted structural matching
  -> numeric Js/rank/nullspace certificate (fallback: pivoted QR)
  -> assemble one Newton matrix
  -> inertia certificate
  -> one numeric factor + solve
  -> backward-error gate
  -> descent/trust-region/fraction-to-boundary gate
       failed: update delta_W, reuse symbolic, refactor
       passed: filter line search
  -> corrector/SOC/refinement: RHS-only solves
  -> accepted iterate or restoration
  -> original-variable postsolve and KKT audit
```

## 7. 实现映射与验收指标

| 理论对象 | 实现位置 | 必须新增的可证伪指标 |
|---|---|---|
| 固定变量消元 | MIPSolvers Native NLP 入口 | 原/约化维数、固定列数、恢复后 KKT |
| OPF 分区提示 | Parity `NLPModel` bridge | 匹配覆盖率、$\kappa(J_s)$ 估计、$\|J_gZ\|$ |
| 分区证书/回退 | MIPSolvers KKT cache | hint accepted/rejected、QR fallback 原因 |
| 分解复用 | KKT factor cache | 每类 factor 原因、factor/solve 比、pattern hash |
| 首步方向门 | Filter IPM | $\delta_W,\eta_K,q'(0),\rho_d,\alpha_{ftb}$ |
| 原空间审计 | adapter postsolve/benchmark | primal/dual/complementarity 三分量 |

hybrid2000 当前首步数据为

```text
iter 0: primal=14.67, dual=100.10, delta_w=1e-2,
        |dx|inf=6048, alpha_pri=7.74e-5
iter 1: delta_w=1.70667, |dx|inf=467512,
        |dmu|inf=1.77e6, alpha_pri=5.75e-8
```

它同时违反尺度化 trust-region 与 fraction-to-boundary 方向门，不能仅凭正确惯性
进入 line search。当前约 93 次 numeric factor 中相当一部分来自随后 200 次
restoration/retry 轨迹。第一阶段验收不是直接宣称 1 秒，而是：

1. 固定变量精确消元后原模型审计不退化；
2. 分区提示通过数值证书，失败时可观察到 QR 回退；
3. predictor/corrector/SOC 不增加 numeric factor；
4. hybrid2000 首步不再产生尺度化巨步和 $10^{-8}$ 级边界步长；
5. numeric factor 次数、总 factor 时间和最终原尺度残差同时下降。

只有这些结构指标成立并且内置 OPF 全部通过共同审计后，才评价与 Ipopt 的
总时间领先关系。

## 8. 2026-08-06 轨迹修正：中心性、缩放与局部接受门

### 8.1 不活跃不等式不能使用绝对乘子下限

冷启动松弛取 $s_i=\max(-h_i(x_0),s_{min})$ 时，中心路径要求

$$
\mu_i=\bar\mu/s_i,\qquad s_i\mu_i=\bar\mu.
$$

旧实现把每个 $\mu_i$ 截断到 `[1e-4,1e4]`。hybrid2000 中 6420 个非线性
不等式有 1188 个落到下限，使该块最大互补从目标 `0.001` 变成 `0.0700122`；
lower/upper bound 两块仍为 `0.001`。这说明误差来自大松弛的非活跃非线性行，
不是统一 `mu_init` 太小。绝对下限还违反目标缩放协变性：目标缩放改变
$\bar\mu$ 后，下限不变，故 $s_i\mu_i$ 不再按同一比例变化。

实现现在对模型 nonlinear rows 只用严格内部数值下限 `2*kMinPositive`，
可表示时精确保持 $s_i\mu_i=\bar\mu`；生成的 box rows 保留 bound-dual
稳定化下限。这是按约束来源分块，不是案例路由。对应单元测试同时验证大松弛
非线性行的中心乘积和上下界行的稳定化乘积。

**2026-08-18 修订（去超参数化后续）**：`133bf359`/`b14082d2` 的冷启动重写
用 `initialize_cold_primal_dual_state` 取代了上述分块规则。两处方差异需要
记录：(1) $\bar\mu$ 不再取 `max(mu_init, 0.1*tol_complementarity)` 的抬升值，
`mu_init>0` 时逐行保持 $s_i\mu_i=\bar\mu=$ `mu_init`（仅含 $O(\sqrt\epsilon)$
的 product-weight 摆动），这比旧分块乘积更中心；(2) box rows 的 `[1e-4,1e4]`
截断也被移除，所有行统一使用 stationarity-invisible 可表示性下限
（`sqrt(min_normal)` 量级），§8.1 对绝对下限破坏缩放协变性的论证同样适用于
盒行下限。`test_numerical_stability` 的 centrality 用例已更新为验证均匀乘积
$s_i\mu_i=$ `mu_init` 与非线性行无下限抬升（`mu_ineq[0] < 1e-4`）。

### 8.2 路径入口与恢复候选必须在一致坐标中处理

缩放模型为 $f_s=s_f f$。`mu_init`、`mu_min` 与 complementarity tolerance
均按 $s_f$ 映射到缩放坐标，primal-dual warm start 还要包含对应行缩放。
restoration retry 必须调用同一变换函数，不能把未变换 options 直接用于新的
scaled model。不同 `sf/sf2` 产生的 retry merit 也不可直接比较；采纳前必须
映射回同一个原模型，要求 primal 与 `max(dual,complementarity)` 同时改善。

曾测试把默认 `mu_init=0.1` 直接解释为 active-coordinate 参数。它把
hybrid2000 推到更低 primal，但使 case30 从约 7 次迭代退化到 177 次、153 ms。
随后测试解析规则
$\arg\min_{\bar\mu}\|\nabla f+J_h^TS^{-1}e\bar\mu\|_2$，结果为 case30
选择 `0.0537`、case118/300 选择 `0.1`，hybrid2000 反而只选 `0.00201`。
两项实验均已撤销。说明路径入口还必须包含等式流形/normal-step 信息，不能只
依据目标缩放、primal 标量或未投影 stationarity。

**2026-08-18 观测（自动选择器的退化情形）**：`b14082d2` 把默认
`mu_init` 改为 0（自动 stationarity 选择器）后，若冷启动点恰好是
stationarity 退化点（`grad(x0)=0` 且存在被违反的不等式），投影 stationarity
无信号，选择器给出可表示性下限量级的 $\bar\mu\approx 10^{-154}$。无屏障压力
时被违反行的松弛在第一步跌出内部，fraction-to-boundary 把步长冻结在
$10^{-8}$ 量级，restoration 关闭时 filter 报 accepted-step collapse
（`test_numerical_stability` 的 augmented/condensed 用例实测复现；显式
`mu_init=0.1` 时两种 Newton 形式均正常收敛且一致）。生产路径依赖
restoration phase 从此类坍缩中恢复；该用例已改为显式固定 `mu_init=0.1`，
因为它的目的是验证 augmented 与 condensed 形式等价，而不是检验自动选择器。
自动选择器是否需要纳入 normal-step 信息仍是上文未决的设计问题。

### 8.3 Filter 的全局区与局部区必须使用不同接受信息

标准 filter 在全局不可行区只比较 $(\theta,\varphi_\mu)$，这是建立可行性的
必要机制。但 hybrid2000 在 `mu=0.00282843` 的切换步中，旧接受门允许

```text
primal: 2.67e-5 -> 9.06e-3
dual:   1.63e-1 -> 3.29e1
```

只因为 barrier objective 下降。曾在 `theta <= theta_min` 时要求候选额外重算
完整 primal-dual residual，但它使 case30 进一步退化到 134 次、case118 达到
400 次仍未通过；corrector 后统一重算 fraction-to-boundary 也未恢复小案例
性能。两项实验均已撤销。正确的局部门必须识别 normal/tangential step 与
barrier 切换事件，不能对所有局部 Filter 候选套同一个分量单调门。

### 8.4 Phase-I 到 Phase-II 的 central warm-start 合同

`IPMOptions::central_warm_start` 与最终可行的
`primal_feasible_start` 是两个不同合同。前者允许 Phase I 在有限预算内只进入
中心邻域，但必须同时提供完整的 $x,s,\lambda,z$，并在任何 KKT 分解前通过

$$
\max\{\|g(x)\|_\infty,\|h(x)+s\|_\infty\}\le\epsilon_p,
\qquad
\max_i\left|\frac{s_i z_i}{\mu_0}-1\right|\le\eta_c.
$$

实现从提交的逐行互补乘积恢复
$\mu_0=\operatorname{mean}_i(s_i z_i)$，而不覆盖为固定障碍参数。
$\epsilon_p$ 由调用者显式给出；非正值回退到同一求解合同的
`tol_primal`。$\eta_c$ 也可显式给出；非正值按
$\eta_c=\epsilon_c/\mu_0$ 从 `tol_complementarity` 推导。原坐标 callback
给出的约束违反量和变量 bounds 还要分别通过 $\epsilon_p$。完整 dual residual
$\|\nabla f+J_g^T\lambda+J_h^Tz\|_\infty$ 被记录但不作为门槛，因为 OPF
Phase I 有意保留控制空间 reduced gradient 给 Phase II。

乘子幅值不设隐藏上限。只有调用者给出正的
`central_warm_start_max_inequality_dual` 时，才把它作为显式策略检查
$\|z\|_\infty$；默认非正值禁用该策略。任一检查失败时，全部 warm vectors
一起丢弃并进入普通 interiorization；不会部分消费 Phase-I 状态。

固定变量消元保持非线性行不变，并从 lower-bound、upper-bound 两块中删去固定
列对应行后再递归审计。这样原模型 warm-vector 维度不会在约化 NLP 中静默失配，
递归诊断也回传到原坐标结果。审计仅需一次模型/导数评估和稀疏矩阵乘法，成本
$O(n+m+\operatorname{nnz}J)$，不增加 KKT 分解。

该合同沿用 Wächter--Biegler (2006) 第 2--3 节的 filter/barrier 邻域口径；
近期 warm-start 研究脉络参见 Chen--Goulart--Jones (2025,
arXiv:2512.00693)、WARP (2026, arXiv:2605.05728) 与
Taheri--Molzahn (2026, arXiv:2606.08984)。这些工作支持把 Phase I 视为有预算的
邻域构造器，而不是复制一遍高精度 IPM。

### 8.5 保留的负结果

未缩放 `mu_init` 的诊断轨迹曾把 hybrid2000 推进到缩放坐标
`(p,d,c)=(6.31e-7,2.09e-1,2.83e-3)` 后，MUMPS 报告目标惯性且
`delta_w=29.774`，但 barrier slope 为 `+4.96e-3`。同一因子的高精度联合精化
和基于 OPF state/control 匹配的 $J_gd=0$ 投影分别得到
`||J_gd||_inf=1.43e-10` 与 `5.98e-17`，下降内积仍为正。该实验分支已撤销。

结论是直接惯性计数、常规后向残差和事后 tangent 投影仍不足以构造可靠下降
方向；下一实现应在 state/control 分区上直接解约化系统或切换到能提供更强
数值证书的因子后端，不能翻转方向、放宽 filter 或反复增大统一正则化。

## 9. 线性约束 QP 的 primal 结构初值合同

`QPModel::x0` 是与上述 NLP central warm start 分离的轻量合同。NativeLCQP
只在 `x0.size()==n` 且全部有限时消费；否则使用完全相同的确定性盒中心冷
初始化。消费前先按有限变量界投影，再用 Ruiz 列尺度映射
$x_s=D^{-1}x_0$。对一般不等式增广出的 slack 使用
$s_0=\max(b-Ax_0,0)$，从而保持原始变量部分不变。

Warm-start bound slack 按实际距离初始化并施加 $10^{-2}$ 正下限（冷启动
仍保留原有单位下限）：

$$
s_i^L=\max(x_i-l_i,10^{-2}),\qquad
s_i^U=\max(u_i-x_i,10^{-2}).
$$

这使 equality-feasible 的结构点不会被人为的单位 slack residual 淹没，同时
仍保证对数障碍有定义。`SolveStats::warm_start_used` 和
`initial_primal_feas` 报告实际消费及第 0 次迭代残差。该合同不接受外部 dual/
slack，不声称 central-path warm start；OPF 的连通分量配平和约化 Laplacian
投影属于调用方，NativeLCQP 只审计、缩放并执行 QP Phase II。
