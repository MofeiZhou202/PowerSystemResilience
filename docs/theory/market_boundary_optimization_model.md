# 周尺度电力市场边界优化数学模型

本文定义代理模型与 Bayesian Optimization/CMA-ES 使用的研究问题。它是外层研究模型，不改变当前 SCUC/SCED、LMP、AC 或 N-1 求解器的生产契约；最终候选必须由真实市场链复核。当前出清方程和结果口径见[市场模拟数学模型](market_simulation_mathematical_models.md)与[南方执行契约](../modules/market/southern_execution_contract.md)。

## 1. 随机双层问题

单个周样本的输入为

\[
x=(b,\xi,s_0),
\]

其中 $b$ 是待优化的市场边界，$\xi$ 是新能源、负荷、来水、报价和故障场景，$s_0$ 是周初机组、储能和水库状态。内层市场出清为

\[
y^*(b,\xi,s_0)\in\arg\min_y f(y;b,\xi)
\quad\text{s.t.}\quad y\in\mathcal F(b,\xi,s_0).
\]

周内状态按日递推：

\[
s_{d+1}=T(s_d,y_d^*,\xi_d),\qquad d=1,\ldots,7.
\]

因此外层问题为

\[
b^*\in\arg\min_{b\in\mathcal B}J(b),
\]

其中 $J$ 由场景集合 $\{(\xi_s,w_s)\}_{s=1}^N$ 估计，$\sum_sw_s=1$。逐日独立运行不得替代上述状态递推。

## 2. 边界变量与场景

第一阶段只优化低维、具工程含义的参数：

\[
b=[\alpha^{ren},\rho^{res},m^{line},s^{SOC},h^{hydro},\tau^{scen}],
\]

分别表示新能源有效出力系数、备用需求倍率、线路/断面安全裕度、储能 SOC 安全带、水电边界和场景筛选阈值。必须满足

\[
b_{min}\le b\le b_{max},\qquad |b_{d+1}-b_d|\le\Delta_b.
\]

场景应保留时间相关性和权重：

\[
\xi_{t+1}\sim P(\xi_{t+1}\mid\xi_t),
\qquad \xi=[\xi^{wind},\xi^{solar},\xi^{load},\xi^{inflow},\xi^{bid},\xi^{fault}].
\]

## 3. 出清输出与周尺度指标

内层变量包括机组组合 $u_{g,t}$、出力 $P_{g,t}$、备用 $R_{g,t}$、节点负荷损失 $D_{n,t}$、新能源弃电 $K_{r,t}$、线路潮流 $F_{\ell,t}$、储能 SOC、水位和节点价格 $\lambda_{n,t}$。

构造输出向量

\[
o=[o^{cost},o^{service},o^{security},o^{renewable},o^{price},o^{compute}].
\]

### 经济与服务

\[
C_{week}=\sum_{d=1}^7C_d,
\qquad E^{loss}=\sum_{n,t}D_{n,t}\Delta t,
\]

\[
D^{max}=\max_{n,t}D_{n,t},
\qquad T^{loss}=\sum_t\mathbf1(\sum_nD_{n,t}>0)\Delta t.
\]

### 线路安全

\[
O_{\ell,t}=\max\left(0,\frac{|F_{\ell,t}|}{F^{max}_\ell}-1\right)F^{max}_\ell,
\]

\[
E^{over}=\sum_{\ell,t}O_{\ell,t}\Delta t,
\quad O^{max}=\max_{\ell,t}\frac{O_{\ell,t}}{F^{max}_\ell},
\quad T^{over}=\sum_t\mathbf1(\max_\ell O_{\ell,t}>0)\Delta t.
\]

AC MVA 越限、线性有功越限和 N-1 越限必须分开保存。

### 新能源利用率

\[
\eta^{ren}=\frac{\sum_{r,t}P^{ren}_{r,t}\Delta t}{\sum_{r,t}P^{ren,av}_{r,t}\Delta t},
\qquad E^{curt}=\sum_{r,t}(P^{ren,av}_{r,t}-P^{ren}_{r,t})\Delta t.
\]

分母为零时为不适用，不能返回零；不能对逐时利用率做简单平均。

### 价格尖峰

给定事先固定的阈值 $\lambda^{thr}$：

\[
I^{spike}_{n,t}=\mathbf1(\lambda_{n,t}>\lambda^{thr}),
\]

\[
p^{spike}=\sum_{n,t}\omega_{n,t}I^{spike}_{n,t},
\qquad
T^{spike}=\sum_t\mathbf1(\max_n\lambda_{n,t}>\lambda^{thr})\Delta t.
\]

同时记录 $Q_{0.95}(\lambda)$、$Q_{0.99}(\lambda)$ 和 $CVaR_{0.99}(\lambda)$。价格只有在最终 SCED 组合和定价模型有效时才可作为正式标签。

## 4. 风险敏感外层目标

对场景输出使用加权期望：

\[
\mathbb E[Z(b)]\approx\sum_sw_sZ(b,\xi_s).
\]

对尾部风险使用

\[
CVaR_\alpha(Z)=\min_\eta\left[\eta+\frac{1}{1-\alpha}\sum_sw_s(Z_s-\eta)_+\right].
\]

推荐的第一版目标为

\[
\begin{aligned}
J(b)=&\ w_C\widetilde{\mathbb E[C_{week}]}
+w_L\widetilde{CVaR_{0.95}(E^{loss})}
+w_O\widetilde{CVaR_{0.95}(E^{over})}\\
&+w_R\widetilde{\mathbb E[E^{curt}]}
+w_P\widetilde{CVaR_{0.99}(\lambda)}
+w_B\|b-b_0\|_2^2.
\end{aligned}
\]

波浪号表示相对于基准边界 $b_0$ 的固定尺度归一化。硬约束为

\[
\Pr(E^{loss}>0)\le\varepsilon_L,
\quad \Pr(O^{max}>0)\le\varepsilon_O,
\quad \Pr(\text{infeasible})\le\varepsilon_F,
\quad \mathbb E[\eta^{ren}]\ge\eta_{min}.
\]

在权重尚未确定时，应先报告上述多目标的 Pareto 前沿，不应隐含选择权重。

## 5. 代理模型

代理模型近似外层指标及约束，而不是取代市场出清：

\[
\hat o_\phi(b,\xi,s_0)\approx o^{oracle}(b,\xi,s_0),
\qquad
\hat J_\phi(b),\hat g_{\phi,j}(b).
\]

连续输出采用加权回归损失，离散状态采用分类损失：

\[
L(\phi)=\sum_{k\in\mathcal C}\alpha_k\ell_k^{reg}
+\sum_{j\in\mathcal D}\beta_j\ell_j^{cls}.
\]

超时、无解、AC/N-1 失败和模型范围不足必须作为状态标签保留，不能当作零成本。代理候选必须经过真实 SCUC/SCED 以及适用的 AC/N-1 复核。

## 6. Bayesian Optimization 与 CMA-ES

Bayesian Optimization 将 $J(b)$ 视为昂贵黑盒，用后验均值和方差选择候选。带约束的采集函数可写为

\[
CEI(b)=EI(b)\prod_j\Pr(g_j(b)\le0).
\]

CMA-ES 在代理目标上采样

\[
b_i=m+\sigma\mathcal N(0,C),
\]

按候选排序更新 $m,C$，适合 SCUC 整数跳变造成的非光滑目标。变量必须投影到 $\mathcal B$，严重不可行候选应拒绝或施加大惩罚。

实际循环为：

```text
初始场景与边界 → Oracle 出清 → 训练代理
→ Bayesian/CMA-ES 产生候选 → 代理筛选
→ 完整 Oracle 复核 → 更新数据集与代理
```

该文档定义的是研究问题和标签口径；具体权重、阈值、边界维度、代理结构和 Oracle 抽样比例，应在实验协议中另行固定并记录。

## 7. 首期标签实现的口径补充

实现和实验协议见[智能仿真标签与价格代理](../modules/market/intelligent_simulation.md)。
SCUC/SCED为内层原变量；LMP由最终方案对应的定价LP对偶产生，不是SCUC的自由决策变量。
当前Oracle是逐日98点滚动链，不是全周联合最优；98点目标合计不能当作只执行96点的周成本。
第一版固定边界和周初状态，仅学习合成场景响应，尚未学习外层边界动作。
场景采样/缩减阈值属于计算设计参数；评价分布和样本权重应固定，不能通过删除恶劣场景改善物理目标。
本地市场LMP为内生输出；报价等才是其外生扰动，不同时将该LMP作为已知价格输入。
节点—时段价格频率与跨独立周事件概率分开；所有未知标签不填零。
