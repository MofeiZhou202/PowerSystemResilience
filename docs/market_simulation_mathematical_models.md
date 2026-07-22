# 市场模拟数学模型（实现审核稿）

本文从当前生产实现
`include/hacdcpf/market/market_simulation.hpp`、
`src/market/market_simulation.cpp` 与
`src/market/participant_behavior.cpp` 反向整理数学模型。其用途是审核“代码当前求解的模型”，
不是未来功能设计。若本文与代码冲突，以代码与注册测试为准。

当前市场链路为：参与者报价 -> 网络约束 SCUC -> 固定启停 SCED/LMP ->
可选 AC 支路 LODF 预防性 N-1 与全元件纠正式 SCED 校核 -> 非线性交直流潮流校核 -> 日前结算 ->
固定启停实时再调度与双结算 -> 可选重复局部最优响应博弈。

## 1. 适用范围与索引

当前商业模型同时覆盖交流网络与直流网络。直流侧支持 DC 母线、DC 支路、DC 负荷、
固定静态电源/PV、可优化 DC 储能、VSC 和 DC/DC 变换器。外部电网和能量路由器端口仍返回
`unsupported_hybrid_market_assets`，不会静默忽略。

支持的市场对象包括：

- 投运 AC 常规发电机；
- AC 母线和投运 AC 支路；
- DC 母线和投运 DC 支路；
- 作为固定外生注入的 AC/DC 静态发电、可再生能源、PV、AC 储能和不满足优化准入条件的 DC 储能；
- 投运、可控、充电策略非 `static` 且额定能量有效的两类 DC 储能；
- AC/DC 负荷；柔性负荷在本模型中不是可优化需求响应；
- VSC 与 DC/DC 的双向传输；
- AC/DC 负荷削减与正向外生注入弃电。

索引集合：

| 符号 | 含义 |
|---|---|
| $g\in\mathcal G$ | 投运 AC 发电机，内部按 active-generator position 排列 |
| $b\in\mathcal B$ | authored AC bus vector position |
| $\ell\in\mathcal L$ | 投运 AC 支路，内部按 active-branch position 排列 |
| $d\in\mathcal D$ | authored DC bus vector position |
| $m\in\mathcal M$ | 投运 DC 支路，内部按 active-DC-branch position 排列 |
| $v\in\mathcal V$ | 投运 VSC，内部按 active-VSC position 排列 |
| $q\in\mathcal Q$ | 投运 DC/DC，内部按 active-DC/DC position 排列 |
| $h\in\mathcal H$ | 可优化 DC 储能；`dc.storage` 在前、`dc.dc_storage` 在后 |
| $k\in\mathcal K=\{1,\ldots,K\}$ | 分段能量报价段 |
| $t\in\mathcal T=\{1,\ldots,T\}$ | 市场时段 |
| $i\in\mathcal I$ | 市场参与者 |

所有功率变量单位为 MW，时段长度 $\Delta t$ 单位为 h，能量为 MWh，价格为
currency/MWh，固定启停费用为 currency/event。

## 2. 时段需求与外生注入

对母线 $b$、时段 $t$，实现先构造：

$$
D_{bt}=D^{bus}_{bt}+D^{load}_{bt}+D^{flex}_{bt},
$$

$$
X_{bt}=P^{static}_{bt}+P^{renewable}_{bt}+P^{PV}_{bt}+P^{storage}_{bt},
$$

$$
d_{bt}=D_{bt}-X_{bt}.
$$

其中 $D_{bt}$ 是非负毛需求；$X_{bt}$ 保留外生设备的有符号注入，因此固定储能充电等负注入会
增加净需求。只有 $X_{bt}>0$ 的部分可被变量 $c_{bt}$ 削减：

$$
0\le s_{bt}\le D_{bt},\qquad
0\le c_{bt}\le \max(0,X_{bt}),
$$

其中 $s_{bt}$ 为失负荷，$c_{bt}$ 为外生正向注入弃电。

直流侧独立构造同口径的 $D^{dc}_{dt}$、$X^{dc}_{dt}$ 和
$d^{dc}_{dt}=D^{dc}_{dt}-X^{dc}_{dt}$，并配置 $s^{dc}_{dt}$、
$c^{dc}_{dt}$。实现始终通过 AC/DC 分域 bus map 聚合，允许 AC 与 DC 使用相同整数 bus ID。
进入 $\mathcal H$ 的 DC 储能不再计入 $X^{dc}_{dt}$，其净注入由优化变量单独进入节点平衡。

## 3. 参与者报价模型

### 3.1 物理成本与报价参数

发电机真实运行成本为：

$$
C_g(p)=c_{2g}p^2+c_{1g}p+c_{0g}.
$$

市场入口拒绝 $c_{2g}<0$ 的投运机组。定义：

$$
\underline P_g=\max(0,P_g^{min}),\qquad
P_g^{phys}=\max(\underline P_g,P_g^{max}).
$$

参与者的能量加价率 $m_i$、容量扣留率 $w_i$、启停加价率 $m_i^c$ 在提交时分别截断为：

$$
0\le m_i\le10,
\qquad 0\le w_i\le0.95,
\qquad 0\le m_i^c\le10.
$$

容量扣留只作用于最小稳定出力以上的灵活容量：

$$
\overline P_g^{offer}
=\underline P_g+(P_g^{phys}-\underline P_g)(1-w_i).
$$

### 3.2 分段能量报价

每段宽度为：

$$
\Delta P_{gk}=\frac{(P_g^{phys}-\underline P_g)(1-w_i)}{K}.
$$

令第 $k$ 段左右端点为 $a_{gk}$、$b_{gk}=a_{gk}+\Delta P_{gk}$，
当前实现采用二次成本弦斜率并加价：

$$
\pi^E_{gk}
=\left[c_{1g}+\max(0,c_{2g})(a_{gk}+b_{gk})\right](1+m_i).
$$

最小出力报价、空载报价、启停报价和上调备用报价分别为：

$$
C_g^{min,offer}
=\left(c_{2g}\underline P_g^2+c_{1g}\underline P_g\right)(1+m_i),
$$

$$
C_g^{NL,offer}=c_{0g}(1+m_i^c),
$$

$$
C_g^{SU,offer}=C_g^{SU}(1+m_i^c),\qquad
C_g^{SD,offer}=C_g^{SD}(1+m_i^c),
$$

$$
\pi_g^R=\max(0,\pi_{i,input}^R).
$$

每台未被显式归属的投运机组会自动获得一个独立、成本报价型参与者；重复归属会被拒绝。

## 4. 日前 SCUC：报价驱动的混合整数线性模型

### 4.1 决策变量

| 变量 | 类型与范围 | 含义 |
|---|---|---|
| $p_{gt}$ | 连续 | 发电出力 |
| $u_{gt}$ | $\{0,1\}$ | 开机状态 |
| $y_{gt}$ | $\{0,1\}$ | 启动事件 |
| $z_{gt}$ | $\{0,1\}$ | 停机事件 |
| $r_{gt}$ | 连续非负 | 上调备用 |
| $q_{gtk}$ | 连续非负 | 第 $k$ 段接受量 |
| $\theta_{bt}$ | $[-\pi,\pi]$ | 交流 DC 潮流近似的相角；authored SLACK 母线固定为 0 |
| $f_{\ell t}$ | 连续 | 有功支路潮流 |
| $s_{bt}$ | 连续非负 | 负荷削减 |
| $c_{bt}$ | 连续非负 | 外生注入削减 |
| $V^{dc}_{dt}$ | 连续 | 直流母线电压，范围取 authored $[V_d^{min},V_d^{max}]$ |
| $f^{dc}_{mt}$ | 连续 | 直流支路有功潮流 |
| $s^{dc}_{dt},c^{dc}_{dt}$ | 连续非负 | 直流负荷削减、外生注入削减 |
| $x^{AD}_{vt},x^{DA}_{vt}$ | 连续非负 | VSC 的 AC->DC、DC->AC 方向输入功率 |
| $z^V_{vt}$ | $\{0,1\}$ | VSC 方向，1 表示 AC->DC |
| $x^F_{qt},x^R_{qt}$ | 连续非负 | DC/DC 正向、反向输入功率 |
| $z^Q_{qt}$ | $\{0,1\}$ | DC/DC 方向，1 表示 bus_in->bus_out |
| $p^{ch}_{ht},p^{dis}_{ht}$ | 连续非负 | DC 储能充、放电功率 |
| $e_{ht}$ | 连续 | 时段末 DC 储能能量，MWh |
| $z^H_{ht}$ | $\{0,1\}$ | DC 储能方向，1 表示充电 |

### 4.2 目标函数

SCUC 最小化提交报价、备用、失负荷和弃电成本：

$$
\begin{aligned}
\min\quad
&\sum_{t,g}\Delta t\Big[
  \big(\widehat C_g^{min,offer}+\widehat C_g^{NL,offer}\big)u_{gt}
  +\widehat\pi_g^R r_{gt}
  +\sum_k\widehat\pi^E_{gk}q_{gtk}
\Big]\\
&+\sum_{t,g}\left(
  \widehat C_g^{SU,offer}y_{gt}
  +\widehat C_g^{SD,offer}z_{gt}
\right)\\
&+\sum_{t,b}\Delta t\left(
  VOLL\,s_{bt}+\pi^{curt}c_{bt}
\right)
+\sum_{t,d}\Delta t\left(
  VOLL\,s^{dc}_{dt}+\pi^{curt}c^{dc}_{dt}
\right)
+\sum_{t,h}\Delta t\left(
  \pi_h^{ch}p_{ht}^{ch}+\pi_h^{dis}p_{ht}^{dis}
\right),
\end{aligned}
$$

其中帽号表示实现对进入 SCUC 目标的报价取 $\max(0,\cdot)$；
$VOLL=\max(1,VOLL_{input})$，$\pi^{curt}=\max(0,\pi^{curt}_{input})$。

### 4.3 交流节点平衡和交流 DC 潮流近似

令 $A_{b\ell}=1$ 表示支路流入母线，$A_{b\ell}=-1$ 表示支路流出母线，则：

$$
\sum_{g\in\mathcal G_b}p_{gt}
+s_{bt}-c_{bt}
+\sum_{\ell}A_{b\ell}f_{\ell t}
=d_{bt},
\quad \forall b,t.
$$

对支路 $\ell=(i,j)$：

$$
f_{\ell t}=\beta_\ell
(\theta_{it}-\theta_{jt}-\phi_\ell),
\qquad
\beta_\ell=\frac{S_{base}}{x_\ell\tau_\ell},
$$

其中 $\phi_\ell$ 为弧度制固定移相角，$\tau_\ell$ 为变比。若 $|x_\ell|<10^{-12}$，
实现用带原符号的 $10^{-6}$ 替代；若变比近零，则取 1。

启用网络约束且 `rate_a_mva > 0` 时：

$$
-F_\ell^A\le f_{\ell t}\le F_\ell^A.
$$

这里把 MVA 额定值直接作为交流有功 MW 上限。关闭网络约束或缺少额定值时，上限取
$10^9$ MW，而不是删除潮流方程。

#### 4.3.1 精确 AC 热限约束生成

大型 SCUC 不再要求首轮把全部 $2LT$ 个交流热限方向交给 MILP 求解器。令有限额定值的
支路-时段候选集为

$$
\mathcal C=\{(\ell,t):0<F_\ell^A<10^9\}.
$$

当 `enable_scuc_network_constraint_generation=true` 且
$|\mathcal C|\ge$ `scuc_network_constraint_generation_min_candidates`（默认 10000）时，首轮只放宽
$f_{\ell t}$ 的热限变量界；所有节点平衡、相角-潮流等式、机组时序、备用、DC 网络、换流器和
储能约束仍完整保留。第 $k$ 轮主问题得到 $x^k$ 后，对全部候选执行精确分离：

$$
v_{\ell t}^k=
\max\{0,-F_\ell^A-f_{\ell t}^k,
f_{\ell t}^k-F_\ell^A\}.
$$

将 $v_{\ell t}^k>\epsilon$ 的候选按超限量降序恢复为真实双边界；每轮最多加入
`scuc_network_constraint_generation_max_new_per_iteration` 个，值为 0 时加入本轮全部违反项。
初始 MIP start 也会在第一次 MILP 前执行同一全候选分离；命中的真实变量界先恢复，再固定其
整数部分解一次当前约束集 LP，修复成功后才送入求解器。后续外层轮次采用相同的固定整数 LP
修复。默认 $\epsilon=10^{-5}$ MW、最多 8 轮，且 `scuc_time_limit_sec` 是热启动、预分离和
全部外层轮次共享的总预算。

该过程不是忽略网络的启发式。若终止时对所有未激活候选都有
$v_{\ell t}^k\le\epsilon$，当前解满足完整热限模型；受限主问题的下界不高于完整模型下界，
因此最后一轮报告的 MIP gap 仍是完整模型的保守有效界。若达到轮数或时间上限后仍有超限，
`network_constraint_generation_converged=false` 且 SCUC 必须返回不可行，不能进入 SCED/LMP。
当前仅 AC 热限采用该外层生成；DC 支路限额仍在每轮完整保留。

#### 4.3.2 搜索树内热限分离与精确回退

`enable_scuc_in_solve_network_constraint_generation=true` 且使用结构化 StrictHiGHS 时，热限
分离器通过动态节点割回调运行。回调取得节点 LP 在原模型列空间恢复的 $f_{\ell t}$，对全部
尚未提交的候选计算 $v_{\ell t}$。对支路 $\ell=(i,j)$，提交与潮流方程等价的相角差双边
全局割：

$$
-F_\ell^A+b_\ell\phi_\ell
\le b_\ell(\theta_{it}-\theta_{jt})
\le F_\ell^A+b_\ell\phi_\ell,
\qquad
b_\ell=\frac{S_{base}}{x_\ell\tau_\ell}.
$$

这些割在节点 LP 求解之后、接受 incumbent/剪枝/分支之前提交；能够通过 presolve 精确投影并
被接受的割可留在当前树中。`in_solve_network_constraints_submitted` 只统计提交数，不冒充求解器
接受数。最终解仍由外层全候选扫描复核。若原列无法从 presolve 空间可靠恢复、动态割被拒绝
或最终仍有超限，代码恢复对应真实变量界并进入跨轮精确回退，而不是接受未认证解。非
StrictHiGHS 后端继续使用外层生成。

#### 4.3.3 跨轮根状态复用

`enable_scuc_cross_round_solver_state_reuse=true` 时，结构化 StrictHiGHS 在第 $k$ 轮结束后
导出三类原模型空间 artifact，并在第 $k+1$ 轮注入：根节点有效割、与这些割配套的 simplex
root basis，以及每个原始整数列的上下分支伪成本与样本统计。

约束生成只把部分 $f_{\ell t}$ 变量界从 $[-10^9,10^9]$ 收紧到
$[-F_\ell^A,F_\ell^A]$，故 $\mathcal X_{k+1}\subseteq\mathcal X_k$；上一轮对
$\mathcal X_k$ 全局有效的根割对新可行域仍有效。root basis 只作为数值初始基，并且仅在
列数及“原行数 + 根割数”完全匹配时注入。伪成本仅在原始列数匹配时注入。任何 artifact
缺失或维度不匹配都会独立降级为冷启动，不影响精确性。

开放节点队列不跨轮恢复。新增变量界会改变节点 LP、节点下界和剪枝证书，直接沿用旧树并不
安全；当前 HiGHS 接口也不提供带逐节点重新认证的树 checkpoint。结果因而显式返回
`search_tree_rebuilt=true`，不能把 artifact reuse 描述成整棵搜索树恢复。

### 4.4 直流网络、VSC 与 DC/DC

对直流支路 $m=(i,j)$，商业模型采用电压差线性式：

$$
f^{dc}_{mt}=\frac{S_{base}^{dc}}{r_m}
(V^{dc}_{it}-V^{dc}_{jt}).
$$

若 $|r_m|<10^{-12}$，实现取 $10^{-6}$。启用网络约束时，
`rate_a_mva` 优先、否则 `s_max_mva` 作为 MW 上限；两者均无效时取 $10^9$ MW。
该商业方程不含 $I^2R$ 网损。

VSC 效率记为 $\eta_v\in[0.01,1]$。其对两域节点平衡的注入为：

$$
P^{ac}_{vt}=-x^{AD}_{vt}+\eta_vx^{DA}_{vt},
\qquad
P^{dc}_{vt}=\eta_vx^{AD}_{vt}-x^{DA}_{vt}.
$$

方向二进制禁止同一时段双向同时流动：

$$
0\le x^{AD}_{vt}\le \overline P_v^{AD}z^V_{vt},
\qquad
0\le x^{DA}_{vt}\le \overline P_v^{DA}(1-z^V_{vt}).
$$

DC/DC 同理：

$$
P^{in,withdraw}_{qt}=x^F_{qt}-\eta_qx^R_{qt},
\qquad
P^{out,inject}_{qt}=\eta_qx^F_{qt}-x^R_{qt},
$$

并由 $z^Q_{qt}$ 限制方向。Power/不可控模式固定传输功率；Voltage 模式固定输出母线
$V^{dc}_{out,t}=V_q^{ref}$；Droop 模式施加传输功率与输出电压的线性下垂等式。

因此直流节点平衡为：

$$
s^{dc}_{dt}-c^{dc}_{dt}
+\sum_{v\in\mathcal V_d}P^{dc}_{vt}
+\sum_{q\in\mathcal Q_d}P^{dc/dc}_{qt}
+\sum_{h\in\mathcal H_d}(p^{dis}_{ht}-p^{ch}_{ht})
+\sum_m A^{dc}_{dm}f^{dc}_{mt}
=d^{dc}_{dt}.
$$

每个由 DC 支路形成的投运直流电气岛必须至少具有一个 `DC_V` 母线、DC 成网 VSC，或
Voltage/Droop 型 DC/DC 的输出侧参考，否则入口返回 `invalid_dc_reference`。VSC 和 DC/DC
在本阶段视为受监管网络设备，没有独立报价或单独结算。

### 4.5 DC 储能跨期模型

有效容量先按健康状态折减：$E_h^{cap}=E_h^{rated}\operatorname{clip}(SOH_h,0,1)$；
若结果非正则回退到额定容量。能量上下限和初值均在该有效容量上构造并截断。令
$\kappa_h=\operatorname{clip}(1-self\_discharge\_pct_h/100,0,1)$，则：

$$
0\le p^{ch}_{ht}\le \overline P_h^{ch}z^H_{ht},\qquad
0\le p^{dis}_{ht}\le \overline P_h^{dis}(1-z^H_{ht}),
$$

$$
e_{h1}=\kappa_he_h^0+\eta_h^{ch}p^{ch}_{h1}\Delta t
-\frac{p^{dis}_{h1}\Delta t}{\eta_h^{dis}},
$$

$$
e_{ht}=\kappa_he_{h,t-1}+\eta_h^{ch}p^{ch}_{ht}\Delta t
-\frac{p^{dis}_{ht}\Delta t}{\eta_h^{dis}},\quad t>1,
$$

$$
\underline E_h\le e_{ht}\le\overline E_h.
$$

`enforce_terminal_dc_storage_soc=true` 时还固定 $e_{hT}=e_h^0$。注意当前
`self_discharge_pct` 被解释为每个市场时段一次的保留率，并未按 $\Delta t$ 指数换算。
对每个以 $\operatorname{round}(24/\Delta t)$ 划分的日块 $\mathcal T_j$，若日循环上限
$N_h^{cycle}>0$：

$$
\sum_{t\in\mathcal T_j}(p^{ch}_{ht}+p^{dis}_{ht})\Delta t
\le 2E_h^{cap}N_h^{cycle}.
$$

### 4.6 出力、报价段和备用

$$
p_{gt}=\underline P_gu_{gt}+\sum_k q_{gtk},
$$

$$
0\le q_{gtk}\le\Delta P_{gk}u_{gt},
$$

$$
p_{gt}+r_{gt}\le\overline P_g^{offer}u_{gt},
$$

$$
\sum_g r_{gt}\ge R_t,
\qquad
R_t=\rho\left(\sum_bD_{bt}+\sum_dD^{dc}_{dt}\right),
\qquad \rho=\max(0,\rho_{input}).
$$

备用需求按毛需求计算，不按净需求计算。

### 4.7 爬坡与备用可交付性

令
$RU_g=60\Delta t\,ramp\_up\_mw\_min_g$、
$RD_g=60\Delta t\,ramp\_dn\_mw\_min_g$。
参数非正时不施加相应爬坡约束。初始出力
$p_g^0=\operatorname{clip}(pg_g,0,\overline P_g^{offer})$：

$$
p_{g1}+r_{g1}\le p_g^0+RU_g,
\qquad
p_{g1}\ge p_g^0-RD_g,
$$

$$
p_{gt}+r_{gt}-p_{g,t-1}\le RU_g,
\qquad t>1,
$$

$$
p_{g,t-1}-p_{gt}\le RD_g,
\qquad t>1.
$$

### 4.8 启停逻辑

初始状态由 `pg_mw > 1e-6` 判定为开机：

$$
u_{g1}-y_{g1}+z_{g1}=u_g^0,
$$

$$
u_{gt}-u_{g,t-1}-y_{gt}+z_{gt}=0,
\qquad t>1,
$$

$$
y_{gt}+z_{gt}\le1.
$$

令 $U_g=\lceil T_g^{up}/\Delta t\rceil$、
$D_g=\lceil T_g^{down}/\Delta t\rceil$：

$$
\sum_{\tau=\max(1,t-U_g+1)}^t y_{g\tau}\le u_{gt},
$$

$$
\sum_{\tau=\max(1,t-D_g+1)}^t z_{g\tau}\le1-u_{gt}.
$$

每个从时域起点开始的 24 h 分块还满足：

$$
\sum_{t\in day}y_{gt}\le N_g^{SU,max},\qquad
\sum_{t\in day}z_{gt}\le N_g^{SD,max},
$$

但只有对应上限为正时才启用。

### 4.9 SCUC 专用热启动、分支与停止证书

市场构造器不再把 SCUC 作为无结构 MILP 直接提交。它向求解器提供 `UCGenHint`：包括
$p/u/y/z$ 列映射、机组母线、初始状态、最小启停时间、爬坡、出力上下限、备用需求和报价段。
分支优先级按以下原则生成：启停状态高于换流/储能方向，早时段高于晚时段，大容量机组高于
小容量机组。该优先级由结构化 HiGHS、Native B&C 或支持相同契约的商业后端消费。

多时段热启动先求连续松弛，按松弛启停值、报价 merit order、负荷、备用和最大事故容量修复启停状态；
随后固定所有整数变量并重新求解完整网络 LP。只有该固定整数 LP 成功，完整解向量才写入
`initial_solution`，因此 `scuc_mip_start_provided=true` 表示已经通过全部当前 SCUC 线性约束，
而不是未经验证的舍入向量。
单时段不存在可利用的启停时间链，默认跳过该热启动以避免额外 LP 开销。

当二进制数达到 `structured_scuc_min_binary_variables`（默认 1000）时，`highs/auto` 启用
结构化分支、根节点分离、可靠伪成本、对称检测和 MIP start；小模型保留轻量 HiGHS，避免
根节点策略的固定开销。默认相对 gap 为 1%，时限 120 s：

$$
gap=\frac{|UB-LB|}{\max(1,|UB|)}.
$$

`scuc_mip_gap_target_met` 仅表示 $gap\le gap_{target}$；`scuc_optimality_proven` 只有在求解器
报告最优且实际 gap 不超过 $10^{-9}$ 时为 true。达到时限但有 incumbent 时必须保留求解器
状态和实际 gap，不能声明严格最优。

## 5. 固定启停多时段 SCED 与价格

SCED 固定 SCUC 得到的 $u_{gt}=u_{gt}^*$，仅优化连续变量。其目标为：

$$
\min\ \sum_{t,g,k}\Delta t\,\pi^E_{gk}q_{gtk}
+\sum_{t,g}\Delta t\,\pi_g^Rr_{gt}
+\sum_{t,b}\Delta t(VOLL\,s_{bt}+\pi^{curt}c_{bt})
+\sum_{t,d}\Delta t(VOLL\,s^{dc}_{dt}+\pi^{curt}c^{dc}_{dt})
+\sum_{t,h}\Delta t(\pi_h^{ch}p_{ht}^{ch}+\pi_h^{dis}p_{ht}^{dis}).
$$

SCED 不再计入最小出力、空载、启动或停机报价，因为启停已固定；这些成本通过后续
make-whole uplift 处理。SCED 保留第 4 节的 AC/DC 节点平衡、网络、出力分解、容量和爬坡约束，
并固定 SCUC 得到的 VSC、DC/DC 与 DC 储能方向。储能的连续充放电、SOC、效率、自放电、
终端 SOC 和日循环约束均被保留。其他实现差异如下：

$$
\sum_gr_{gt}=R_t,
$$

即 SCED 的备用是等式，而 SCUC 中是下界；单机备用上界还受一个时段上调能力限制：

$$
0\le r_{gt}\le
u_{gt}^*\min(\overline P_g^{offer}-\underline P_g,RU_g).
$$

实时调用可另外施加 $p_{gt}\ge P_{gt}^{instruction,min}$ 或
$p_{gt}=P_{gt}^{actual,fixed}$；这些边界必须与启停状态和报价容量相容。

### 5.1 LMP 与备用价格

记 AC、DC 节点平衡等式的最优对偶变量为 $\mu^{ac}_{bt}$、$\mu^{dc}_{dt}$，
系统备用等式的最优对偶变量为 $\nu_t$。
代码报告：

$$
LMP^{ac}_{bt}=\frac{\mu^{ac}_{bt}}{\Delta t},
\qquad
LMP^{dc}_{dt}=\frac{\mu^{dc}_{dt}}{\Delta t},
\qquad
\pi_t^{R,clear}=\frac{\nu_t}{\Delta t}.
$$

成功的 simplex 定价结果将 `MarketModelScope.energy_prices_valid` 置为 true。若 LP 后端
未返回完整对偶量，当前结果向量仍可能保留初始化值 0；该布尔量尚不是逐节点 dual-presence 证明。
变量数不超过 `pricing_native_max_variables`（默认 5000）的 LP 先使用原生 simplex；若其
Phase I 或数值过程失败，则自动回退到嵌入式 HiGHS LP。更大 LP 直接路由到 HiGHS，避免
已知昂贵的原生失败尝试；阈值为 0 表示所有定价 LP 均直达 HiGHS。HiGHS 成功且返回完整
行对偶时仍可形成 LMP。结果分别用 `performance.pricing_solver_fallback_used` 和
`performance.pricing_large_model_direct_highs_used` 标记失败回退与规模路由。

## 6. 混合口径全元件 N-1 模型

### 6.1 PTDF/LODF

令 $A$ 为支路-母线关联矩阵，$B_f=\operatorname{diag}(\beta_\ell)$：

$$
B_{bus}=A^TB_fA.
$$

删除参考母线后求解支路 $o$ 两端的单位转移，得到
$PTDF_{\ell o}$。对不导致孤岛的故障支路：

$$
LODF_{\ell o}=\frac{PTDF_{\ell o}}{1-PTDF_{oo}},
\qquad LODF_{oo}=-1.
$$

若分母绝对值小于 $10^{-7}$，该故障被视为孤岛故障并跳过。LODF 灵敏度忽略固定移相器
注入，但基态 SCED 潮流仍包含移相角。

### 6.2 安全约束生成

故障 $o$ 后监视支路 $\ell$ 的估计潮流：

$$
f_{\ell t}^{(o)}=f_{\ell t}+LODF_{\ell o}f_{ot}.
$$

目标安全域为：

$$
\left|f_{\ell t}+LODF_{\ell o}f_{ot}\right|
\le\gamma F_\ell^A,
$$

其中 $\gamma$ 为 `n1_emergency_rating_multiplier`。实现先解无 N-1 约束的基准 SCED，
筛选超限方向，然后逐轮加入单边线性割：

$$
\sigma_{\ell ot}
\left(f_{\ell t}+LODF_{\ell o}f_{ot}\right)
\le\gamma F_\ell^A,
\quad \sigma_{\ell ot}\in\{-1,1\}.
$$

直到无超限、达到迭代上限或达到每轮割上限。最终割直接属于定价 LP，因此会进入最终
LMP。按 `n1_max_contingencies` 截断时，候选故障按基态时域最大绝对潮流降序选择。

### 6.3 发电机能力约束与全元件纠正式校核

启用 N-1 后，SCUC 对每台故障机组 $o$、每时段要求其余机组上调能力覆盖。为避免在每个
故障行重复写入全部备用变量，先定义辅助变量 $R_t$：

$$
R_t=\sum_g r_{gt},
\qquad
p_{ot}+r_{ot}\le R_t,
$$

它与 $p_{ot}\le\sum_{g\ne o}r_{gt}$ 完全等价，但非零元由 $O(TG^2)$ 降为 $O(TG)$。

最终日前组合和换流/储能方向固定后，代码枚举投运的 AC 发电机、AC 支路、DC 支路、
VSC、DC/DC、`dc.storage` 与 `dc.dc_storage`。每个故障在整个市场时域持续存在，并重新求解
一个允许纠正性再调度的多时段 SCED。令基态失负荷为 $LS^0$、故障结果为 $LS^{(o)}$：

$$
secure_o = feasible_o\ \land\
\max(0,LS^{(o)}-LS^0)\le\epsilon_{N-1}\Delta t.
$$

因此当前是三层组合口径：AC 支路 LODF 预防式切平面、发电机预防式能力采购、所有覆盖
元件的固定组合纠正式 SCED。只有 AC 支路切平面进入最终定价 LP，其他故障校核不会产生
事故约束影子价格。`n1_max_contingencies=0` 表示全覆盖；非零值按统一 authored 类型/位置顺序
截断全元件纠正式故障清单，而 AC LODF 候选独立按基态最大绝对潮流排序后取同一数量预算。
母线、负荷、并联元件、开关设备与保护装置不在当前市场 N-1 元件集合内。

## 7. 非线性交直流校核（不参与出清）

SCED 是交流 DC 潮流近似加直流电压线性式的商业模型。可选校核把已出清发电、AC/DC
负荷削减、AC/DC 外生弃电和换流器计划写回时段快照，求解统一非线性交直流潮流。交流侧为：

$$
P_i=V_i\sum_jV_j(G_{ij}\cos\theta_{ij}+B_{ij}\sin\theta_{ij}),
$$

$$
Q_i=V_i\sum_jV_j(G_{ij}\sin\theta_{ij}-B_{ij}\cos\theta_{ij}).
$$

直流支路物理校核使用：

$$
I_{ij}^{dc}=\frac{V_i^{dc}-V_j^{dc}}{r_{ij}},\qquad
P_{ij}^{from}=V_i^{dc}I_{ij}^{dc}S_{base},\qquad
P_{ij}^{to}=-V_j^{dc}I_{ij}^{dc}S_{base},
$$

因而包含电压相关的直流支路损耗。VSC/DC-DC 使用统一换流器物理模型，包括结果中声明的
损耗、控制方程和有效设备限制。节点削减按 authored bus 上各类负荷/正向外生注入同比例
分配。校核条件包括：

$$
V_b^{min}-\epsilon_V\le V_b\le V_b^{max}+\epsilon_V,
$$

$$
\max(|S_{\ell,from}|,|S_{\ell,to}|)\le F_\ell^A+\epsilon_S,
$$

$$
V_d^{dc,min}-\epsilon_V\le V_d^{dc}\le V_d^{dc,max}+\epsilon_V,
$$

$$
\max(|P_{m,from}^{dc}|,|P_{m,to}^{dc}|)\le F_m^{dc}+\epsilon_S.
$$

$$
P_{slack}^{min}-\epsilon_P
\le P_{slack}^{schedule}+\Delta P_{loss}
\le P_{slack}^{max}+\epsilon_P.
$$

其中：

$$
\Delta P_{loss}
=P^{served}+P^{branch\ loss}-P^{scheduled\ gen}-P^{exogenous}.
$$

每个交流连通分量独立选择一台在线平衡机；结果中的旧单值
`slack_generator_position` 仅保留兼容语义。混合校核只认证，不把网损平衡调整回写到商业
出清或结算。可选 AC 故障校核还会逐时断开
选定支路，并用 $\gamma F_\ell^A$ 检查故障后视在功率；它同样不执行纠正性再调度。

## 8. 日前结算模型

### 8.1 发电机收入与成本

发电机能量与备用收入：

$$
Rev_g^E=\sum_tLMP_{b(g),t}p_{gt}\Delta t,
$$

$$
Rev_g^R=\sum_t\pi_t^{R,clear}r_{gt}\Delta t.
$$

真实成本：

$$
TC_g=\sum_{t:u_{gt}=1}
(c_{2g}p_{gt}^2+c_{1g}p_{gt}+c_{0g})\Delta t
+\sum_t(C_g^{SU}y_{gt}+C_g^{SD}z_{gt}).
$$

报价成本由最小出力报价、按段接受能量、空载、备用及启停报价重构。代码不直接读取
SCED 的 $q_{gtk}$ 进行结算，而是按出力从低段到高段重新分配：

$$
\widetilde q_{gtk}(p_{gt})=
\min\left\{\Delta P_{gk},
\max\left[0,p_{gt}-\underline P_g-\sum_{h<k}\Delta P_{gh}\right]\right\}.
$$

因此：

$$
BC_g=\sum_{t:u_{gt}=1}\Delta t\left[
C_g^{min,offer}+\sum_k\pi^E_{gk}\widetilde q_{gtk}(p_{gt})
+C_g^{NL,offer}+\pi_g^Rr_{gt}\right]
+\sum_t(C_g^{SU,offer}y_{gt}+C_g^{SD,offer}z_{gt}).
$$

make-whole uplift 与利润：

$$
U_g=\max(0,BC_g-Rev_g^E-Rev_g^R),
$$

$$
\Pi_g^{DA}=Rev_g^E+Rev_g^R+U_g-TC_g.
$$

注意 uplift 按报价成本而不是真实成本补足。

### 8.2 用户、外生资源与现金流

$$
Pay^{load,E}=\sum_{t,b}LMP^{ac}_{bt}(D_{bt}-s_{bt})\Delta t
+\sum_{t,d}LMP^{dc}_{dt}(D^{dc}_{dt}-s^{dc}_{dt})\Delta t,
$$

$$
Rev^{exo,E}=\sum_{t,b}LMP^{ac}_{bt}(X_{bt}-c_{bt})\Delta t
+\sum_{t,d}LMP^{dc}_{dt}(X^{dc}_{dt}-c^{dc}_{dt})\Delta t,
$$

可优化 DC 储能按所在 DC 节点的净注入结算：

$$
Rev_h^{E}=\sum_tLMP^{dc}_{d(h),t}
(p^{dis}_{ht}-p^{ch}_{ht})\Delta t,
$$

$$
BC_h=\sum_t(\pi_h^{ch}p^{ch}_{ht}+\pi_h^{dis}p^{dis}_{ht})\Delta t,
\qquad \Pi_h=Rev_h^E-BC_h.
$$

储能当前没有 uplift；`DCStorageSettlement` 分别报告两种 authored 容器中的稳定组件 ID、
充放电量、净注入、初末 SOC、收入、报价成本与利润。

$$
Pay^{load,R}=\sum_t\pi_t^{R,clear}R_t\Delta t,
\qquad
Pay^{load,U}=\sum_gU_g.
$$

总账同时给出 `customer_ac_energy_payment`、`customer_dc_energy_payment`、
`resource_ac_energy_revenue` 和 `resource_dc_energy_revenue`。VSC/DC/DC 是受监管网络资产，
不单独收付款；其效率损耗通过两域 LMP 与节点注入进入组合拥塞/损耗租金。资源能量收入
包含常规机组、可优化 DC 储能与 AC/DC 外生资源。拥塞租金和审计残差定义为：

$$
CR^{DA}=Pay^{load,E}-Rev^{resource,E},
$$

$$
Residual^{DA}=Pay^{load,total}-Rev^{resource,total}-CR^{DA}.
$$

按上述定义，备用收费与备用收入、uplift 收费与 uplift 收入分别抵消，残差应接近 0。

### 8.3 市场力指标

对参与者 $i$ 的发电量份额 $s_i^E$ 和市场收入份额 $s_i^{Rev}$（单位均为百分数）：

$$
HHI_E=\sum_i(s_i^E)^2,
\qquad
HHI_{Rev}=\sum_i(s_i^{Rev})^2.
$$

实现还报告 Top-3 发电量份额、最大份额、最大利润、总扣留容量和按机组 action 简单平均的
能量加价率。收入份额只使用非负收入。

## 9. 实时市场、备用履约与双结算

实时市场固定日前启停 $u_{gt}^{DA}$，把备用需求率设为 0，并在实际时序上重新运行连续
SCED；报价仍由日前参与者行为参数重新生成。

### 9.1 备用激活

日前与实时净需求分别为 $D_t^{net,DA}$、$D_t^{net,RT}$，两者均为 AC 与 DC
各节点净需求之和：

$$
A_t=\max(0,D_t^{net,RT}-D_t^{net,DA}).
$$

若日前总备用获配 $R_t^{award}=\sum_gr_{gt}^{DA}>0$，机组指令为：

$$
a_{gt}=\min\left(r_{gt}^{DA},
A_t\frac{r_{gt}^{DA}}{R_t^{award}}\right).
$$

第一遍实时 SCED 施加：

$$
p_{gt}^{instr}\ge p_{gt}^{DA}+a_{gt}.
$$

给定履约系数 $\eta_g\in[0,1]$：

$$
d_{gt}^{reserve}=\eta_ga_{gt},
\qquad
h_{gt}^{reserve}=(1-\eta_g)a_{gt}.
$$

第二遍平衡 SCED 对收到备用指令的机组固定：

$$
p_{gt}^{actual}=p_{gt}^{instr}-h_{gt}^{reserve},
$$

其余机组可在原 SCED 约束内再调度补足缺口。若 $A_t$ 超过日前总备用，所有备用最多激活到
各自获配值，超出部分没有独立的“备用不足”松弛变量，而由平衡 SCED 的一般再调度、失负荷
或不可行状态处理。

### 9.2 发电机双结算

$$
\Delta E_{gt}=(p_{gt}^{RT}-p_{gt}^{DA})\Delta t,
$$

$$
Rev_g^{RTdev}=\sum_tLMP_{b(g),t}^{RT}\Delta E_{gt}.
$$

设实际相对 ISO 实时指令的偏差为：

$$
e_{gt}^{gen}=p_{gt}^{RT}-p_{gt}^{instr},
$$

容忍带与计罚偏差为：

$$
tol_{gt}^{gen}=\alpha_G\max(|p_{gt}^{instr}|,1),
$$

$$
e_{gt}^{pen}=\max(0,|e_{gt}^{gen}|-tol_{gt}^{gen}).
$$

辅助服务净调整：

$$
Adj_g^{AS}=\sum_t\Delta t\left[
\pi^{perf}d_{gt}^{reserve}
-\pi^{nonperf}h_{gt}^{reserve}
-\pi^{imb,G}e_{gt}^{pen}
\right].
$$

总收入和利润为：

$$
Rev_g^{2set}=Rev_g^{DA,E}+Rev_g^{DA,R}+U_g
+Rev_g^{RTdev}+Adj_g^{AS},
$$

$$
\Pi_g^{2set}=Rev_g^{2set}-TC_g(p^{RT},u^{DA}).
$$

这里真实成本按实际实时出力和固定启停重新计算。

### 9.3 用户偏差与总账

以下公式对 $n\in\mathcal B\cup\mathcal D$ 分域计算；$LMP_{nt}$ 必须取该节点所属域的
AC 或 DC LMP。母线已供负荷偏差和外生已交付注入偏差分别为：

$$
\Delta P_{bt}^{served}
=(D_{bt}^{RT}-s_{bt}^{RT})-(D_{bt}^{DA}-s_{bt}^{DA}),
$$

$$
\Delta X_{bt}^{delivered}
=(X_{bt}^{RT}-c_{bt}^{RT})-(X_{bt}^{DA}-c_{bt}^{DA}).
$$

$$
Pay^{RTdev}=\sum_{t,n}LMP_{nt}^{RT}\Delta P_{nt}^{served}\Delta t,
$$

$$
Rev^{RTdev}_{exo}=\sum_{t,n}LMP_{nt}^{RT}
\Delta X_{nt}^{delivered}\Delta t.
$$

优化 DC 储能的实时偏差收入独立计入资源侧：

$$
Rev^{RTdev}_{storage}=\sum_{t,h}LMP_{d(h),t}^{dc,RT}
\left[(p_{ht}^{dis,RT}-p_{ht}^{ch,RT})
-(p_{ht}^{dis,DA}-p_{ht}^{ch,DA})\right]\Delta t.
$$

实时与日前必须采用相同的 DC 储能优化策略，否则返回
`real_time_dc_storage_policy_mismatch`。该收入进入
`RealTimePeriod::dc_storage_deviation_revenue` 和双结算现金流审计。

负荷偏差罚金使用毛需求偏差：

$$
e_{bt}^{load}=D_{bt}^{RT}-D_{bt}^{DA},
$$

$$
e_{bt}^{load,pen}=\max\left(0,|e_{bt}^{load}|
-\alpha_L\max(|D_{bt}^{DA}|,1)\right).
$$

系统运营者辅助服务余额：

$$
SO^{AS}=Penalty^{load}+Charge^{gen,imb}+Charge^{reserve,nonperf}
-Payment^{reserve,perf}.
$$

组合现金流审计为：

$$
Residual^{2set}=Pay^{customer,2set}-Rev^{resource,2set}
-CR^{DA}-CR^{RTdev}-SO^{AS}.
$$

## 10. 重复参与者博弈

重复博弈不是连续优化或强化学习，而是确定性的同步离散局部最优响应。参与者 $i$ 的策略为：

$$
x_i=(m_i,w_i),
$$

其收益为日前利润 $\Pi_i^{DA}$，或启用实时市场时的双结算利润 $\Pi_i^{2set}$。

每轮在其他参与者策略固定时评估：

$$
\mathcal N_i(x_i)=\left\{
(m_i\pm\delta_m,w_i),
(m_i,w_i\pm\delta_w)
\right\},
$$

并投影至：

$$
0\le m_i\le m^{max},\qquad
0\le w_i\le w^{max}\le0.95.
$$

若最佳候选满足：

$$
\Pi_i(x_i',x_{-i})>\Pi_i(x_i,x_{-i})+\epsilon_\Pi,
$$

则接受该响应。所有参与者的新策略同步应用。没有参与者存在超过容差的四邻域改进时，报告
`converged`；因此这里得到的是网格上的局部均衡，不是全局 Nash 均衡。当前博弈只调整能量
加价和容量扣留，不学习启停加价或备用报价。

## 11. 当前实现边界与审核重点

以下不是推测，而是当前代码口径：

1. **当前是线性交直流商业模型。** AC 使用 DC 潮流近似，DC 使用电压差线性式；无交流无功决策。
2. **DC 支路商业模型无损。** $I^2R$ 损耗只在非线性认证中出现，不回写商业结算。
3. **VSC/DC-DC 仅建模方向与常效率。** VSC 固定/二次损耗、电流、容量圆、调制限制尚未成为商业约束。
4. **换流器是受监管网络资产。** 没有换流器报价、租金分配或独立结算。
5. **仅 DC 储能进入跨期优化。** AC 储能及不满足准入条件的 DC 储能仍是固定外生注入；DC 储能没有备用产品或 uplift。
6. **外部电网和能量路由器仍拒绝。** 结果列出稳定组件 ID 和拒绝原因。
7. **全元件 N-1 是混合口径。** AC 支路使用价格可见的预防式 LODF 割，其他覆盖元件使用价格不可见的纠正式 SCED；故障持续整个时域，且不覆盖母线、负荷、并联元件、开关设备和保护装置。
8. **支路 MVA 在商业模型中当作 MW。** 非线性认证才对 AC 两端视在功率和 DC 物理端功率检查。
9. **柔性负荷不是需求响应变量。** 它只进入固定毛需求，唯一需求侧松弛是高成本失负荷。
10. **SCUC 备用为 $\ge R_t$，SCED 备用为 $=R_t$。** 这会影响退化最优解及备用对偶解释。
11. **初始历史信息有限。** 初始启停只由 `pg_mw` 判断；没有时域开始前已开/停持续时间，
   因而不能完整执行跨边界剩余最小开停机时间。
12. **机组无终端状态约束。** 时域末端不要求终端启停状态，也不计时域外的后续启停成本；DC 储能默认要求终端 SOC 回到初值。
13. **报价段依靠凸成本顺序自然填充。** 没有显式“先填低价段”约束；负边际成本等异常数据
   可能暴露 SCUC 对报价取非负、SCED 保留原报价的差异。
14. **价格有效性是求解级标志。** `energy_prices_valid` 表示 simplex 定价成功，但不是逐节点对偶存在性证明。
15. **实时备用激活是确定性比例规则。** 不与能量共同优化激活量，也没有下调备用产品。
16. **uplift 按报价成本补足。** 战略加价可进入 uplift，而非只补真实成本。
17. **重复博弈仅保证离散四邻域局部稳定。** 同步更新可能存在循环；达到轮数上限时不声称收敛。

建议审核时优先确认第 2、3、4、7、10、13、14、15、16 项是否符合目标市场规则；这些属于模型选择，
不是单纯的软件实现细节。

## 12. 模型规模、复杂度与省级系统适用边界

### 12.1 精确变量数量

令 $G,B,L,D,M,V,Q,H,T,K$ 分别表示投运机组、AC 母线、AC 支路、DC 母线、DC 支路、
VSC、DC/DC、可优化 DC 储能、时段和报价段数。按当前变量布局，SCUC 变量总数为：

$$
N_{var}^{SCUC}=T\left[(5+K)G+3B+L+3D+M+3V+3Q+4H+I_{N-1}\right],
$$

其中 $I_{N-1}$ 在启用 N-1 时为 1，否则为 0，对应每时段一个备用总量辅助变量。

其中二进制变量数为：

$$
N_{bin}^{SCUC}=T(G+V+Q+H).
$$

机组仅保留启停状态 $u_{gt}$ 为二进制。启动/停机辅助量 $v_{gt},w_{gt}\in[0,1]$ 保持连续：
由于相邻 $u$ 为二进制，真实状态变化会由转移等式把对应辅助量固定为 1；额外的同时启动与
停机只会增加非负成本并收紧最小启停时间、每日次数等约束，不可能改善当前目标。因此该松弛
不改变投影到启停、出力和成本空间的最优解，但把机组二进制数量从 $3TG$ 降至 $TG$。

固定方向后的 SCED 连续变量数为：

$$
N_{var}^{SCED}=T\left[(2+K)G+3B+L+3D+M+2V+2Q+3H\right].
$$

这些值由 `MarketPerformanceProfile` 在每次运行中直接返回，而不是由 GUI 重新猜测。
百台机组并不自动意味着不可解；真正决定难度的是 $T$、机组启停耦合、报价退化性、网络规模、
储能/换流方向二进制和求解器。MILP 时间不能由变量数线性外推。

### 12.2 基态热限活跃集规模

完整 AC 热限候选数为 $LT$ 个双边变量界。约束生成首轮不向求解器施加这些有限界，最终只
激活实际超限候选的双边界。若收敛时激活数为 $A\ll LT$，MILP 主问题中的有限热限从 $LT$
降为 $A$；潮流变量和 $LT$ 条相角-潮流等式不会减少，所以这是针对拥塞组合搜索的稀疏化，
不是网络降阶。最坏情况下 $A=LT$，计算量可能高于一次性完整模型，但结果口径不变。

性能画像返回候选数、激活数、MILP 外层轮数、剩余违反数和最坏超限 MW。只有
`scuc_network_constraint_generation_converged=true` 才允许把约束生成结果标记为可行。

### 12.3 当前实现中的非线性增长点

启用发电机 N-1 能力约束后共有 $GT$ 条三系数能力约束，另有 $T$ 条备用总量定义，每条
含 $G+1$ 个系数，因此该部分非零元数量为 $O(TG)$。AC 支路筛选对每个时段、候选故障和
监视支路计算事故后潮流，复杂度为 $O(TN_oL)$。

旧版 `build_lodf_model` 的稠密内存基线为：

$$
Memory_{LODF,major}\gtrsim8\left(LB+L^2+B^2+(B-1)^2\right)\ bytes.
$$

当前实现不再分配这些稠密网络矩阵：它直接由支路端点构造稀疏降阶 $B_{bus}$，执行一次
`SparseLU` 分解，并对按基态最大绝对潮流排序的事故逐列回代。设降阶矩阵及其 LU 因子的
非零元数为 $nnz(B_r)$、$nnz(LU)$，实际请求并成功计算的事故列数为 $C$，主要内存变为：

$$
Memory_{LODF,sparse}=O\left(nnz(B_r)+nnz(LU)+LC\right).
$$

当 `n1_max_contingencies>0` 时，$C$ 受候选预算约束；`0` 仍严格执行所有有效事故，此时
$C\approx L$，仍保留 $O(L^2)$ 的 LODF 列存储。`estimated_lodf_dense_bytes` 是旧稠密路径
对照值，`estimated_lodf_sparse_bytes` 是当前组装矩阵、拓扑、列存储和 LU 填充的规划估算，
实际 allocator 占用依赖稀疏排序与平台。

全元件纠正式校核对每个候选元件独立重建并求解一个完整多时段 SCED。若覆盖元件数为
$N_c$，当前实现仍执行 $N_c$ 次，但以有界 worker 池并行。设实际 worker 数为 $W$：

$$
Time_{component\ N-1}\gtrsim
\frac{1}{W}\sum_{o=1}^{N_c}Time\left(SCED^{(o)}_{T}\right),
\qquad
Memory_{peak}=O\left(W\,Memory(SCED_T)\right).
$$

每个任务使用独立系统副本、LP 模型和求解器实例，结果写入预分配槽位后按 authored 元件顺序
归并。默认 $W\le4$；显式设置 `component_n1_parallel_workers=0` 才使用硬件并发数。

非线性基态认证还需 $T$ 次混合潮流；可选 AC 故障认证约需
$T\,N_o^{AC}$ 次潮流。重复博弈会把完整日前/实时市场调用再乘以轮数、参与者数和候选策略数，
不适合在大系统上与全 N-1 无限制叠加。

### 12.4 当前工程判断

当前代码可以用于百机级基础出清的基准、研究和受控运行，但还不能承诺“任意省级系统、
24 h、全元件 N-1、非线性认证和重复博弈”具有可接受的生产时延。主要缺口是：

1. SCUC 已暴露时限、MIP gap、节点上限、结构化分支、验证后 incumbent 和精确 AC 热限约束生成；约束生成轮次复用 root basis、根割与伪成本，但新增热限后仍须重建并重新认证搜索树；
2. 全元件纠正式 SCED 已有界并行，但尚无按影响度筛选、basis/模型复用或早停策略；
3. PTDF/LODF 已采用稀疏因子复用和按需列；全覆盖仍有 $O(L^2)$ 列存储，尚无流式筛选或分区；
4. HTTP 市场调用仍为同步长任务，没有作业队列、取消和进度接口；
5. 没有注册的多规模市场性能回归与硬件无关预算，当前测试主要证明正确性而非 SLA。

省级应用的保守运行方式应分层：先用精确热限约束生成运行基础 SCUC/SCED；再用有限候选集执行 AC 支路安全割；
最后离线或批处理全元件纠正式校核。交互界面中 `n1_max_contingencies` 应设为明确预算，
`0` 的全覆盖只适合已完成规模评估的算例。求解器优先采用 HiGHS/SCIP 或已授权的商业 MILP
后端；原生 B&C 主要用于小规模可重复验证。

下一阶段的生产化优化顺序应为：异步作业、进度和取消；跨运行根割/伪成本缓存；再引入滚动
时域、候选风险排序、流式安全筛选或场景分解。未完成这些工作前，
不应把单时段或小算例耗时外推为省级系统性能承诺。

### 12.5 可观测性能契约

`MarketPerformanceProfile` 返回模型规模、SCUC/SCED 求解器、SCUC gap/终止状态/结构提示/
MIP start/分支路径、定价 fallback/规模直达、旧稠密 LODF 基线、当前稀疏工作内存估算、
SCUC 跨轮根状态复用/搜索树重建状态、热限生成轮数/候选数/激活数/剩余超限、实际 LODF 列数、事故 LP/worker 数，以及报价、SCUC、基础
SCED、LODF、切平面、全元件 N-1、非线性认证、结算和总耗时。计时为当前进程的 wall-clock
秒数，只用于定位瓶颈，不构成跨硬件 SLA。

### 12.6 当前实现的规模参考

下表来自同一本地 macOS release 构建和服务进程，用于验证优化方向，不是跨硬件 SLA。关闭项
按表中说明执行；耗时会随数据、求解器版本、CPU、内存带宽和后台负载变化。

| 算例与设置 | 优化前 | 当前实现 | 主要变化 |
|---|---:|---:|---|
| ACTIVSg500，56 台投运机组，6 时段，4 报价段，无 N-1 | 总计 16.81 s；SCED 6.38 s | 总计 4.25 s；SCUC 4.16 s | 二进制压缩、可行 MIP start、大 LP 直达 HiGHS |
| ACTIVSg500，同上，24 时段、1% gap | 超过 120 s | 总计 2.13 s；SCUC 1.74 s；实际 gap 0.760% | 结构化分支与显式 gap 契约 |
| ACTIVSg2000，432 台投运机组，3206 支路，1 时段，1 报价段，无 N-1 | 总计 28.52 s；SCED 24.70 s | 总计 4.05 s；SCED 0.092 s | 大 LP 直达 HiGHS |
| ACTIVSg2000，同上，10 个 N-1 候选、0 轮 AC 割 | 总计 26.29 s；SCUC 25.90 s | 总计 7.97 s；SCUC 7.59 s | 发电机能力矩阵 $O(TG^2)\to O(TG)$ |
| ACTIVSg2000，24 时段，1 报价段，启用热限，无 N-1，1% gap / 120 s | 全量热限超过 135 s；旧生成路径 2 轮、120.92 s 未完成 | MIP start 预分离激活 34 / 76944；1 轮、剩余超限 0、无树重建；120.83 s 仍未达到 MIP gap | 网络约束已闭合，瓶颈转为单次 284520 变量 / 10560 二进制 SCUC |
| ACTIVSg500，1 时段，10 个纠正式事故 | 事故 SCED 串行 0.961 s | 4-worker 0.367 s | 有界并行，约 2.6 倍阶段加速 |

在 ACTIVSg2000 的 10 列 LODF 运行中，旧稠密基线估算约 197.49 MB，当前稀疏工作估算约
0.704 MB，LODF 构建约 0.003 s。若设置全覆盖，列存储会重新增长到 $O(L^2)$，不能用该
10 列结果推断全覆盖内存与时延。

ACTIVSg2000 复测进一步排除了网络生成重启：MIP start 在第一次 MILP 前命中并恢复 34 个
热限，最后一份 incumbent 的 76944 个候选全扫描无超限，`search_tree_rebuilt=false`；但
StrictHiGHS 仍在单次主问题上耗尽 120 s，未给出满足 1% 目标的 gap。该运行没有第二轮，故
root cuts、root basis 和伪成本没有可复用机会；这不等于复用接口失效，而是说明此算例的当前
主瓶颈已经是单次大规模 SCUC 的根松弛、启发式和树搜索。下一性能任务应优先做滚动时域/时段
分解、根松弛计时剖析和可认证 incumbent/dual-bound 改进，而不是继续优化外层热限扫描。

## 13. 代码与测试对应关系

| 模型部分 | 当前代码入口 |
|---|---|
| 报价、加价、扣留 | `submit_participant_offers` in `src/market/participant_behavior.cpp` |
| SCUC MILP | `build_market_commitment_model` in `src/market/market_simulation.cpp` |
| 精确 AC 热限约束生成 | `solve_market_commitment`, `repair_market_scuc_fixed_integers` |
| 跨轮根割/basis/伪成本复用 | `market_scuc_bc_options`, `solve_market_commitment` |
| 固定启停 SCED/LMP | `build_pricing_model`, `extract_pricing` |
| DC 储能跨期模型 | `active_market_dc_storages`, `build_market_commitment_model`, `build_pricing_model` |
| LODF、安全割与全元件校核 | `build_lodf_model`, `screen_n1`, `run_full_component_n1_validation` |
| 逐岛参考与换流器方向 | `dc_reference_coverage_error`, `select_online_slacks_by_island` |
| 交直流基态与 AC 故障校核 | `summarize_ac_validation`, `run_ac_contingency_validation` |
| 日前结算 | `settle_market`, `aggregate_participant_settlement` |
| 实时双结算与辅助服务 | `run_real_time_market` |
| 重复博弈 | `run_repeated_market_game` |
| 大规模定价回退与性能画像 | `solve_market_pricing_lp`, `MarketPerformanceProfile` |

主要回归契约位于 `tests/test_market_simulation.cpp`，覆盖 24 h SCUC-SCED-LMP-ACPF-结算、
报价完整性、弃电、失负荷收费、混合 AC/DC 出清与认证、逐 DC 岛参考、DC 实时偏差结算、
战略行为、DC 储能套利/SOC/实时偏差结算、全元件 N-1、AC 故障校核、爬坡可交付性、
精确热限生成与未完成拒绝、辅助服务总账和重复博弈。
