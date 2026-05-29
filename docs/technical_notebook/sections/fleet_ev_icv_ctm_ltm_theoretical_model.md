# 微观车辆行为 — Fleet 聚合 — CTM/LTM 动态传播 — 充放电反馈理论模型

## 1. 模型目标与总体结构

本文建立一个用于混合燃油车与电动车交通系统分析的完整理论模型。模型遵循如下分析逻辑：

1. 车辆层采用微观行为模型；
2. 由于车辆数量巨大，将单车行为聚合为 OD-fleet 模型；
3. ICV 采用时间最短模型；
4. EV 采用经济性最优模型；
5. fleet demand 被加载到道路网络；
6. 使用 CTM 或 LTM 计算交通传播；
7. 充电站作为内生服务节点，影响 EV 排队、充放电、停留和离站；
8. 交通传播结果反馈到车辆行为层，形成动态均衡或系统优化问题。

整体结构为：

$$
\text{Vehicle behaviour}
\rightarrow
\text{Fleet aggregation}
\rightarrow
\text{Dynamic network loading}
\rightarrow
\text{Station service}
\rightarrow
\text{Cost feedback}
\rightarrow
\text{Equilibrium / optimization}
$$

其中：

- 车辆层回答“车怎么选”；
- fleet 层回答“很多车如何聚合”；
- CTM/LTM 层回答“这些车如何在路网上传播”；
- 站点层回答“EV 如何排队、充放电、离站”；
- 反馈层回答“选择和传播如何相互作用”。

---

# 2. 基本集合、索引与网络定义

## 2.1 时间集合

设离散时间集合为：

$$
\mathcal{K}=\{0,1,\ldots,T\}
$$

时间步长为：

$$
\Delta t
$$

若 $$\Delta t$$ 以小时计，则流量单位 veh/h 乘以 $$\Delta t$$ 后得到每个时间步内车辆数。

---

## 2.2 道路网络

道路网络为有向图：

$$
\mathcal{G}^{R}
=
\left(
\mathcal{N}^{R},
\mathcal{A}^{R}
\right)
$$

其中：

- $$\mathcal{N}^{R}$$ 为道路节点集合；
- $$\mathcal{A}^{R}$$ 为道路有向 link 集合。

每条道路 link $$a\in\mathcal{A}^{R}$$ 具有参数：

$$
L_a,\quad
v^{ff}_a,\quad
w_a,\quad
\bar q_a,\quad
N^{jam}_a
$$

分别表示：

- link 长度；
- 自由流速度；
- 后向拥堵波速度；
- 通行能力；
- jam storage。

---

## 2.3 OD 集合与需求

OD 对集合为：

$$
\mathcal{W}
$$

其中一个 OD 对记为：

$$
w=(o,d)\in\mathcal{W}
$$

总 OD demand 为：

$$
d_{w,k}
$$

EV 渗透率为：

$$
p^{EV}_{w,k}\in[0,1]
$$

则 ICV 和 EV demand 分别为：

$$
d^{F}_{w,k}
=
\left(1-p^{EV}_{w,k}\right)d_{w,k}
$$

$$
d^{E}_{w,k}
=
p^{EV}_{w,k}d_{w,k}
$$

车辆类别集合为：

$$
\mathcal{C}=\{\mathrm{F},\mathrm{E}\}
$$

其中：

- $$\mathrm{F}$$ 表示燃油车；
- $$\mathrm{E}$$ 表示电动车。

---

## 2.4 路径与活动计划集合

ICV 的路径集合为：

$$
\mathcal{R}_{w}
$$

其中 $$r\in\mathcal{R}_{w}$$ 表示 OD 对 $$w$$ 的一条道路路径。

EV 的活动计划集合记为：

$$
\Omega_{w,k}
$$

一个 EV 活动计划定义为：

$$
\omega=
\left(
r,
s,
\chi
\right)
$$

其中：

- $$r$$ 为道路路径；
- $$s$$ 为选择的充电站，若不进站则 $$s=\varnothing$$；
- $$\chi$$ 为充放电策略，包括充电量、放电量、充电功率、放电功率和站内停留时间等。

因此：

- ICV 的选择对象是路径 $$r$$；
- EV 的选择对象是活动计划 $$\omega$$。

---

## 2.5 充电站集合

充电站集合为：

$$
\mathcal{S}
$$

每个站点 $$s\in\mathcal{S}$$ 具有参数：

$$
\bar B_s,\quad
\bar p^{ch}_s,\quad
\bar p^{dis}_s,\quad
\bar Q_s,\quad
\bar H_s
$$

分别表示：

- 插枪数量；
- 最大充电功率；
- 最大放电功率；
- 最大排队容量；
- 最大 dwell 容量。

站点连接到电网 bus：

$$
\mathrm{bus}(s)
$$

---

# 3. 建模假设

## 3.1 交通流假设

### 假设 1：道路交通满足一阶流体交通流理论

道路 link 上的宏观流满足 LWR kinematic wave theory。三角形 fundamental diagram 为：

$$
q_a(k)=
\begin{cases}
v^{ff}_a k_a, & k_a\le k^c_a,\\
w_a\left(k^j_a-k_a\right), & k_a>k^c_a.
\end{cases}
$$

其中：

$$
k^c_a=
\frac{\bar q_a}{v^{ff}_a}
$$

$$
k^j_a=
k^c_a+
\frac{\bar q_a}{w_a}
=
\bar q_a
\left(
\frac{1}{v^{ff}_a}
+
\frac{1}{w_a}
\right)
$$

后向波速满足：

$$
w_a=
\frac{\bar q_a}{k^j_a-k^c_a}
=
\frac{v^{ff}_a k^c_a}{k^j_a-k^c_a}
$$

---

### 假设 2：道路 link 上满足 FIFO

即先进入同一 link 的车辆不会晚于后进入车辆离开。

若车辆 $$i$$ 早于车辆 $$j$$ 进入 link $$a$$：

$$
k^{in}_{i,a}\le k^{in}_{j,a}
$$

则有：

$$
k^{out}_{i,a}\le k^{out}_{j,a}
$$

---

### 假设 3：ICV 与 EV 对道路容量的占用相同

在基础模型中：

$$
\mathrm{PCE}^{F}=\mathrm{PCE}^{E}=1
$$

如果后续考虑车辆类型差异，可以扩展为 class-dependent PCE。

---

### 假设 4：道路传播由 CTM 或 LTM 给出

CTM 和 LTM 只负责 traffic propagation，不负责车辆路径、站点或充放电行为选择。

车辆选择由 vehicle behaviour layer 决定。

---

## 3.2 车辆行为假设

### 假设 5：ICV 以时间最短为主要目标

ICV 的行为目标为：

$$
\min_{r\in\mathcal{R}_{w}} T_{r,k}
$$

若考虑燃油成本，可扩展为时间和燃油成本的广义成本。

---

### 假设 6：EV 以经济性最优为目标

EV 不只考虑时间，还考虑：

- 道路旅行时间；
- 站点等待时间；
- 充电时间；
- 放电收益；
- 电价；
- 电池退化；
- SOC 安全；
- 绕行成本；
- 站点可达性。

EV 选择活动计划：

$$
\omega^*_{w,k}
=
\arg\min_{\omega\in\Omega_{w,k}}
C^{E}_{w,\omega,k}
$$

---

### 假设 7：车辆数量大，可由 fleet flow 表示

不逐车求解所有车辆，而是将相同行为、相同 OD、相同出发时段的车辆聚合为 fleet flow：

$$
x^{F}_{w,r,k}
$$

$$
x^{E}_{w,\omega,k}
$$

---

## 3.3 EV 能量与充放电假设

### 假设 8：EV 电池能量动态满足离散 SOC 方程

对 EV $$v$$：

$$
e_{v,k+1}
=
e_{v,k}
-
E^{drive}_{v,k}
+
\eta^{ch}_v p^{ch}_{v,k}\Delta t
-
\frac{p^{dis}_{v,k}\Delta t}{\eta^{dis}_v}
$$

其中：

- $$e_{v,k}$$ 为电池能量；
- $$E^{drive}_{v,k}$$ 为行驶能耗；
- $$p^{ch}_{v,k}$$ 为充电功率；
- $$p^{dis}_{v,k}$$ 为放电功率；
- $$\eta^{ch}_v$$ 为充电效率；
- $$\eta^{dis}_v$$ 为放电效率。

---

### 假设 9：EV 必须满足 mobility feasibility

EV 在任何时刻都不能因为充放电导致无法完成剩余行程：

$$
e_{v,k}
-
E^{rem}_{v,k}
\ge
e^{res}_v
$$

其中：

- $$E^{rem}_{v,k}$$ 为从当前位置到目的地的剩余能耗；
- $$e^{res}_v$$ 为安全 reserve energy。

---

### 假设 10：充放电不能同时进行

对任意 EV：

$$
I^{ch}_{v,k}+I^{dis}_{v,k}\le 1
$$

其中：

$$
I^{ch}_{v,k},I^{dis}_{v,k}\in\{0,1\}
$$

功率约束为：

$$
0\le p^{ch}_{v,k}\le \bar p^{ch}_v I^{ch}_{v,k}
$$

$$
0\le p^{dis}_{v,k}\le \bar p^{dis}_v I^{dis}_{v,k}
$$

---

## 3.4 充电站服务假设

### 假设 11：充电站是道路网络中的 service node

EV 到达站点后依次经历：

$$
\text{arrival}
\rightarrow
\text{queueing}
\rightarrow
\text{charging/discharging}
\rightarrow
\text{dwell}
\rightarrow
\text{egress}
$$

---

### 假设 12：站点服务满足容量约束

站点同时占用 plug 的车辆数量不超过插枪数：

$$
B_{s,k}\le \bar B_s
$$

如果车辆处于充电、放电或插枪 idle dwell 状态，都占用 plug，则：

$$
B_{s,k}
=
\sum_{v\in\mathcal{V}_{s,k}}
\left(
I^{ch}_{v,k}
+
I^{dis}_{v,k}
+
I^{idle}_{v,k}
\right)
$$

---

# 4. 微观车辆模型

## 4.1 ICV 微观模型

ICV $$v$$ 的状态为：

$$
x^F_{v,k}
=
\left(
l_{v,k},
p_v
\right)
$$

其中：

- $$l_{v,k}$$ 为车辆位置；
- $$p_v$$ 为所选路径。

ICV 在 OD 对 $$w$$、出发时段 $$k$$ 选择路径：

$$
r^*_{v,k}
=
\arg\min_{r\in\mathcal{R}_w}
T_{r,k}
$$

路径旅行时间由动态 link travel time 递推得到。

若路径为：

$$
r=(a_1,a_2,\ldots,a_m)
$$

车辆从 $$k_1=k$$ 进入第一条 link。对每条 link：

$$
k_{j+1}
=
k_j
+
\left\lceil
\frac{T_{a_j,k_j}}{\Delta t}
\right\rceil
$$

路径旅行时间为：

$$
T_{r,k}
=
\sum_{j=1}^{m}T_{a_j,k_j}
$$

若采用 Wardrop 时间均衡：

$$
T_{r,k}=T^{\min}_{w,k}
\quad
\text{if}
\quad
x^F_{w,r,k}>0
$$

$$
T_{r,k}\ge T^{\min}_{w,k}
\quad
\text{if}
\quad
x^F_{w,r,k}=0
$$

其中：

$$
T^{\min}_{w,k}
=
\min_{r\in\mathcal{R}_w}T_{r,k}
$$

---

## 4.2 EV 微观模型

EV $$v$$ 的状态为：

$$
x^E_{v,k}
=
\left(
l_{v,k},
e_{v,k},
\sigma_{v,k},
p_v
\right)
$$

其中：

- $$l_{v,k}$$ 为位置；
- $$e_{v,k}$$ 为电池能量；
- $$\sigma_{v,k}$$ 为行为状态；
- $$p_v$$ 为活动计划。

行为状态集合可定义为：

$$
\sigma_{v,k}
\in
\{
\textit{driving},
\textit{accessing\_station},
\textit{queueing},
\textit{charging},
\textit{discharging},
\textit{dwelling},
\textit{egressing},
\textit{completed}
\}
$$

---

## 4.3 EV 行驶能耗

若采用距离线性能耗模型：

$$
E^{drive}_{v,a}
=
\xi_v L_a
$$

其中 $$\xi_v$$ 为单位里程能耗。

若采用速度相关能耗模型：

$$
E^{drive}_{v,a,k}
=
g_v
\left(
L_a,
T_{a,k},
\bar v_{a,k}
\right)
$$

平均速度为：

$$
\bar v_{a,k}
=
\frac{L_a}{T_{a,k}}
$$

因此拥堵不仅增加时间，也可能影响能耗。

---

## 4.4 EV SOC 动态

EV 在任意时段满足：

$$
e_{v,k+1}
=
e_{v,k}
-
E^{drive}_{v,k}
+
\eta^{ch}_v p^{ch}_{v,k}\Delta t
-
\frac{p^{dis}_{v,k}\Delta t}{\eta^{dis}_v}
$$

SOC 上下界为：

$$
e^{min}_v
\le
e_{v,k}
\le
e^{max}_v
$$

出行安全约束为：

$$
e_{v,k}
-
E^{rem}_{v,k}
\ge
e^{res}_v
$$

该约束非常关键，因为它说明：

> V2G 放电不能只看电网收益，必须保证车辆后续仍然可完成出行。

---

## 4.5 EV 充电触发

基础安全触发条件为：

$$
e_{v,k}
-
E^{rem}_{v,k}
<
e^{res}_v
$$

若满足该条件，EV 必须选择可达站点充电。

引入经济性优化后，EV 即使 SOC 足够，也可能因低电价、V2G 收益或未来电价预期而主动进站。

因此可将触发条件扩展为：

$$
\min_{\omega\in\Omega^{stop}_{v,k}}
C^E_{v,\omega,k}
<
C^E_{v,\mathrm{no\ stop},k}
$$

即如果包含站点活动的最优计划成本低于不进站成本，则 EV 选择进站。

---

# 5. EV 经济性最优模型

## 5.1 EV 活动计划

对 OD 对 $$w$$、出发时段 $$k$$，EV 活动计划为：

$$
\omega=
\left(
r,
s,
p^{ch}_{\omega,\ell},
p^{dis}_{\omega,\ell},
E^{ch}_{\omega},
E^{dis}_{\omega},
T^{dwell}_{\omega}
\right)
$$

其中 $$\ell$$ 表示活动计划中的服务时段。

若不进站，则：

$$
s=\varnothing
$$

且：

$$
p^{ch}_{\omega,\ell}=p^{dis}_{\omega,\ell}=0
$$

---

## 5.2 EV 经济成本函数

EV 活动计划成本可定义为：

$$
C^E_{w,\omega,k}
=
C^{travel}_{w,\omega,k}
+
C^{wait}_{w,\omega,k}
+
C^{service}_{w,\omega,k}
+
C^{energy}_{w,\omega,k}
+
C^{deg}_{w,\omega,k}
+
C^{schedule}_{w,\omega,k}
+
\phi^{soc}_{w,\omega,k}
$$

### 5.2.1 道路旅行时间成本

$$
C^{travel}_{w,\omega,k}
=
\alpha_t T_{\omega,k}
$$

其中：

- $$\alpha_t$$ 为时间价值；
- $$T_{\omega,k}$$ 为活动计划对应的道路旅行时间。

### 5.2.2 站点等待时间成本

若活动计划包含站点 $$s$$：

$$
C^{wait}_{w,\omega,k}
=
\alpha_w W_{s,k^{arr}_{s}}
$$

其中：

- $$W_{s,k^{arr}_{s}}$$ 为到达站点时的预计排队时间；
- $$k^{arr}_{s}$$ 为车辆到达站点的时间步。

若不进站，则该项为 0。

### 5.2.3 服务时间成本

充放电服务时间为：

$$
T^{service}_{\omega}
=
T^{ch}_{\omega}
+
T^{dis}_{\omega}
+
T^{switch}_{\omega}
$$

充电时间为：

$$
T^{ch}_{\omega}
=
\frac{E^{ch}_{\omega}}
{\eta^{ch}\bar p^{ch}_{s}}
$$

放电时间为：

$$
T^{dis}_{\omega}
=
\frac{E^{dis}_{\omega}\eta^{dis}}
{\bar p^{dis}_{s}}
$$

对应服务时间成本为：

$$
C^{service}_{w,\omega,k}
=
\alpha_s T^{service}_{\omega}
$$

### 5.2.4 能源成本

设充电价格为：

$$
\pi^{ch}_{s,\ell}
$$

放电补偿价格为：

$$
\pi^{dis}_{s,\ell}
$$

则净能源成本为：

$$
C^{energy}_{w,\omega,k}
=
\sum_{\ell\in\mathcal{K}_{\omega}^{s}}
\left(
\pi^{ch}_{s,\ell}p^{ch}_{\omega,\ell}
-
\pi^{dis}_{s,\ell}p^{dis}_{\omega,\ell}
\right)\Delta t
$$

其中：

- 充电为正成本；
- 放电收益为负成本。

### 5.2.5 电池退化成本

可采用线性退化成本：

$$
C^{deg}_{w,\omega,k}
=
c^{deg}
\left(
E^{ch}_{\omega}
+
E^{dis}_{\omega}
\right)
$$

也可采用循环深度相关模型：

$$
C^{deg}_{w,\omega,k}
=
f^{deg}
\left(
\mathrm{DoD}_{\omega},
E^{throughput}_{\omega}
\right)
$$

其中：

$$
E^{throughput}_{\omega}
=
E^{ch}_{\omega}
+
E^{dis}_{\omega}
$$

### 5.2.6 到达时间惩罚

若考虑期望到达时间 $$t^*_w$$，可定义 schedule delay：

$$
C^{schedule}_{w,\omega,k}
=
\gamma_e
\max
\left(
0,
t^*_w-t^{arr}_{w,\omega,k}
\right)
+
\gamma_l
\max
\left(
0,
t^{arr}_{w,\omega,k}-t^*_w
\right)
$$

其中：

- $$\gamma_e$$ 为早到惩罚；
- $$\gamma_l$$ 为迟到惩罚。

### 5.2.7 SOC 不可行惩罚

如果活动计划违反 SOC 安全约束，则赋予大惩罚：

$$
\phi^{soc}_{w,\omega,k}
=
M
\cdot
\mathbf{1}
\left[
\exists \ell:
e_{\omega,\ell}
-
E^{rem}_{\omega,\ell}
<
e^{res}
\right]
$$

其中 $$M$$ 为足够大的惩罚系数。

在优化模型中，更推荐直接将 SOC 可行性作为硬约束。

---

## 5.3 EV 最优行为

EV 选择：

$$
\omega^*_{w,k}
=
\arg\min_{\omega\in\Omega_{w,k}}
C^E_{w,\omega,k}
$$

若使用 fleet equilibrium 形式，则满足：

$$
C^E_{w,\omega,k}
=
C^{E,\min}_{w,k}
\quad
\text{if}
\quad
x^E_{w,\omega,k}>0
$$

$$
C^E_{w,\omega,k}
\ge
C^{E,\min}_{w,k}
\quad
\text{if}
\quad
x^E_{w,\omega,k}=0
$$

其中：

$$
C^{E,\min}_{w,k}
=
\min_{\omega\in\Omega_{w,k}}
C^E_{w,\omega,k}
$$

---

# 6. Fleet 聚合模型

## 6.1 ICV fleet 聚合

ICV route flow 为：

$$
x^F_{w,r,k}\ge 0
$$

OD 守恒：

$$
\sum_{r\in\mathcal{R}_w}
x^F_{w,r,k}
=
d^F_{w,k}
$$

所有 ICV fleet flow 构成：

$$
x^F=
\{x^F_{w,r,k}\}
$$

---

## 6.2 EV fleet 聚合

EV activity-plan flow 为：

$$
x^E_{w,\omega,k}\ge 0
$$

OD 守恒：

$$
\sum_{\omega\in\Omega_{w,k}}
x^E_{w,\omega,k}
=
d^E_{w,k}
$$

所有 EV fleet flow 构成：

$$
x^E=
\{x^E_{w,\omega,k}\}
$$

---

## 6.3 Fleet 到 link inflow 的映射

总 link inflow 为：

$$
u_{a,k}
=
u^F_{a,k}
+
u^E_{a,k}
+
u^{egress}_{a,k}
$$

其中 ICV 部分：

$$
u^F_{a,k}
=
\sum_{w}
\sum_{r\in\mathcal{R}_w}
\delta^{F}_{a,w,r,k}
x^F_{w,r,k}
$$

EV 部分：

$$
u^E_{a,k}
=
\sum_{w}
\sum_{\omega\in\Omega_{w,k}}
\delta^{E}_{a,w,\omega,k}
x^E_{w,\omega,k}
$$

其中：

- $$\delta^{F}_{a,w,r,k}$$ 表示 ICV path flow 在时段 $$k$$ 是否进入 link $$a$$；
- $$\delta^{E}_{a,w,\omega,k}$$ 表示 EV activity-plan flow 在时段 $$k$$ 是否进入 link $$a$$；
- 这些动态 incidence 由 CTM/LTM propagation 决定。

---

## 6.4 Fleet 到站点到达流的映射

站点 $$s$$ 的 EV 到达流为：

$$
u^S_{s,k}
=
\sum_{w}
\sum_{\omega\in\Omega_{w,k}:s\in\omega}
\delta^{arr}_{s,w,\omega,k}
x^E_{w,\omega,k}
$$

其中 $$\delta^{arr}_{s,w,\omega,k}$$ 表示活动计划 $$\omega$$ 的 EV 是否在时间 $$k$$ 到达站点 $$s$$。

---

## 6.5 Fleet 到站点离站流的映射

站点离站流为：

$$
o_{s,k}
=
\sum_{w}
\sum_{\omega\in\Omega_{w,k}:s\in\omega}
\delta^{dep}_{s,w,\omega,k}
x^E_{w,\omega,k}
$$

该流进入 station egress link，成为下游路网输入。

---

# 7. CTM 动态传播模型

CTM 适用于 cell-level propagation。

## 7.1 Cell 离散化

将 link $$a$$ 分为 $$M_a$$ 个 cell，cell 长度为：

$$
\delta_a=
\frac{L_a}{M_a}
$$

CTM CFL 条件要求：

$$
v^{ff}_a\Delta t
\le
\delta_a
$$

因此在给定 $$\delta_a$$ 时，需要选择满足条件的时间步长：

$$
\Delta t
\le
\frac{\delta_a}{v^{ff}_a}
$$

对全网：

$$
\Delta t
\le
\min_{a\in\mathcal{A}^{R}}
\frac{\delta_a}{v^{ff}_a}
$$

注意：如果 $$L_a<v^{ff}_a\Delta t$$，则无论如何划分 cell，都无法在该 link 上满足 classical CTM CFL 条件，此时必须减小 $$\Delta t$$ 或使用 sub-stepping。

---

## 7.2 Cell 状态

Cell occupancy 为：

$$
n_{a,m,k}
$$

其中：

- $$a$$ 为 link；
- $$m=0,\ldots,M_a-1$$ 为 cell index；
- $$k$$ 为时间步。

若追踪车辆类别：

$$
n_{a,m,k}
=
n^F_{a,m,k}
+
n^E_{a,m,k}
$$

---

## 7.3 CTM sending function

Cell sending 为：

$$
S_{a,m,k}
=
\min
\left(
\frac{v^{ff}_a}{\delta_a}n_{a,m,k},
\bar q_a
\right)\Delta t
$$

该式表示当前 cell 在一个时间步内最多能送出的车辆数。

---

## 7.4 CTM receiving function

Cell receiving 为：

$$
R_{a,m,k}
=
\min
\left(
\bar q_a,
\frac{w_a}{\delta_a}
\left(
N^{jam}_{a,m}
-
n_{a,m,k}
\right)
\right)\Delta t
$$

其中：

$$
N^{jam}_{a,m}
=
k^j_a\delta_a
$$

---

## 7.5 Inter-cell flow

从 cell $$m$$ 到 cell $$m+1$$ 的流量为：

$$
y_{a,m\rightarrow m+1,k}
=
\min
\left(
S_{a,m,k},
R_{a,m+1,k}
\right)
$$

---

## 7.6 Cell occupancy update

Cell 状态更新为：

$$
n_{a,m,k+1}
=
n_{a,m,k}
+
y_{a,m-1\rightarrow m,k}
-
y_{a,m\rightarrow m+1,k}
$$

对第一个 cell：

$$
y_{a,-1\rightarrow 0,k}
=
u_{a,k}
$$

对最后一个 cell：

$$
y_{a,M_a-1\rightarrow M_a,k}
=
v_{a,k}
$$

---

## 7.7 CTM 节点模型

在节点 $$i$$，设 incoming links 为：

$$
\mathcal{A}^{in}(i)
$$

outgoing links 为：

$$
\mathcal{A}^{out}(i)
$$

节点转向流为：

$$
y_{a,b,k}
$$

满足发送约束：

$$
\sum_{b\in\mathcal{A}^{out}(i)}
y_{a,b,k}
\le
S^{exit}_{a,k}
$$

接收约束：

$$
\sum_{a\in\mathcal{A}^{in}(i)}
y_{a,b,k}
\le
R^{entry}_{b,k}
$$

其中：

$$
S^{exit}_{a,k}=S_{a,M_a-1,k}
$$

$$
R^{entry}_{b,k}=R_{b,0,k}
$$

若给定 turning proportion：

$$
y_{a,b,k}
=
\beta_{a,b,k}v_{a,k}
$$

且：

$$
\sum_{b}
\beta_{a,b,k}=1
$$

节点模型可进一步设为最大吞吐问题：

$$
\max_{y_{a,b,k}}
\sum_{a,b}y_{a,b,k}
$$

subject to sending、receiving、turning、priority constraints。

---

# 8. LTM 动态传播模型

LTM 适用于 link-level cumulative propagation。

## 8.1 Cumulative curves

定义 link $$a$$ 的累计进入和累计离开车辆数：

$$
N^{in}_{a,k}
$$

$$
N^{out}_{a,k}
$$

时间步流量为：

$$
u_{a,k}
=
N^{in}_{a,k+1}
-
N^{in}_{a,k}
$$

$$
v_{a,k}
=
N^{out}_{a,k+1}
-
N^{out}_{a,k}
$$

link occupancy 为：

$$
n_{a,k}
=
N^{in}_{a,k}
-
N^{out}_{a,k}
$$

---

## 8.2 LTM 时间延迟

自由流延迟为：

$$
\tau^{ff}_a
=
\left\lceil
\frac{L_a}{v^{ff}_a\Delta t}
\right\rceil
$$

后向波延迟为：

$$
\tau^{bw}_a
=
\left\lceil
\frac{L_a}{w_a\Delta t}
\right\rceil
$$

---

## 8.3 LTM sending function

$$
S_{a,k}
=
\min
\left(
N^{in}_{a,k-\tau^{ff}_a}
-
N^{out}_{a,k},
\bar q_a\Delta t
\right)
$$

该式表示：

- 至少在自由流时间之前进入 link 的车辆才可能离开；
- 离开量不能超过 link capacity。

---

## 8.4 LTM receiving function

$$
R_{a,k}
=
\min
\left(
N^{out}_{a,k-\tau^{bw}_a}
+
N^{jam}_a
-
N^{in}_{a,k},
\bar q_a\Delta t
\right)
$$

该式表示：

- link 的可接收空间由 jam storage 和后向波传播决定；
- 进入量不能超过 capacity。

---

## 8.5 Boundary flow constraints

边界流满足：

$$
0\le v_{a,k}\le S_{a,k}
$$

$$
0\le u_{a,k}\le R_{a,k}
$$

累计曲线更新为：

$$
N^{in}_{a,k+1}
=
N^{in}_{a,k}
+
u_{a,k}
$$

$$
N^{out}_{a,k+1}
=
N^{out}_{a,k}
+
v_{a,k}
$$

---

## 8.6 LTM 节点模型

节点转向流满足：

$$
\sum_{b\in\mathcal{A}^{out}(i)}
y_{a,b,k}
\le
S_{a,k}
$$

$$
\sum_{a\in\mathcal{A}^{in}(i)}
y_{a,b,k}
\le
R_{b,k}
$$

link exit 与 entry 满足：

$$
v_{a,k}
=
\sum_{b\in\mathcal{A}^{out}(i)}
y_{a,b,k}
$$

$$
u_{b,k}
=
\sum_{a\in\mathcal{A}^{in}(i)}
y_{a,b,k}
+
d^{orig}_{b,k}
+
o_{s\rightarrow b,k}
$$

其中 $$o_{s\rightarrow b,k}$$ 为站点离站进入 link $$b$$ 的流。

---

# 9. 充电站服务节点模型

## 9.1 站点状态

每个站点 $$s$$ 定义三个聚合状态：

$$
Q_{s,k}
$$

$$
B_{s,k}
$$

$$
H_{s,k}
$$

分别表示：

- 排队车辆数；
- 正在充放电服务或占用 plug 的车辆数；
- 服务完成后仍在 dwell 的车辆数。

---

## 9.2 站点动态方程

站点到达流为：

$$
u^S_{s,k}
$$

进入服务流为：

$$
b_{s,k}
$$

服务完成流为：

$$
c_{s,k}
$$

离站流为：

$$
o_{s,k}
$$

动态方程为：

$$
Q_{s,k+1}
=
Q_{s,k}
+
u^S_{s,k}
-
b_{s,k}
$$

$$
B_{s,k+1}
=
B_{s,k}
+
b_{s,k}
-
c_{s,k}
$$

$$
H_{s,k+1}
=
H_{s,k}
+
c_{s,k}
-
o_{s,k}
$$

---

## 9.3 进入服务约束

进入服务流受排队车辆和 plug 可用容量限制：

$$
b_{s,k}
\le
Q_{s,k}
+
u^S_{s,k}
$$

$$
B_{s,k}
+
b_{s,k}
\le
\bar B_s
$$

---

## 9.4 服务完成约束

服务完成流受在服务车辆和服务能力限制：

$$
c_{s,k}\le B_{s,k}
$$

$$
c_{s,k}\le \mu_{s,k}
$$

其中 $$\mu_{s,k}$$ 可由平均服务时间或个体充放电计划聚合得到。

若使用个体充放电计划，则：

$$
c_{s,k}
=
\sum_{v\in\mathcal{V}_{s}}
\mathbf{1}
\left[
k^{end}_{v,s}=k
\right]
$$

---

## 9.5 离站约束

离站流受 dwell population 和下游道路 receiving capacity 限制：

$$
o_{s,k}
\le
H_{s,k}
+
c_{s,k}
$$

$$
o_{s,k}
\le
R^{egress}_{s,k}
$$

如果站点下游 link 已拥堵，则 EV 即使服务结束也不能全部离站。

---

## 9.6 站点对道路的 spillback

若站点排队容量有限：

$$
Q_{s,k}\le \bar Q_s
$$

当站点队列接近容量时，access link 的接收能力下降。可设：

$$
R^{access}_{s,k}
=
R^{0}_{s,k}
\left(
1-
\frac{Q_{s,k}}{\bar Q_s}
\right)_+
$$

其中：

$$
(x)_+=\max(x,0)
$$

若：

$$
Q_{s,k}\ge \bar Q_s
$$

则：

$$
R^{access}_{s,k}=0
$$

这表示站点队列 spillback 到道路网络。

---

# 10. 充放电如何影响单车与交通流

## 10.1 对单车 SOC 的影响

充电提高 SOC：

$$
\Delta e^{ch}_{v,k}
=
\eta^{ch}_v p^{ch}_{v,k}\Delta t
$$

放电降低 SOC：

$$
\Delta e^{dis}_{v,k}
=
\frac{p^{dis}_{v,k}\Delta t}{\eta^{dis}_v}
$$

因此充放电改变后续可达路径集合：

$$
\Omega_{v,k}
=
\left\{
\omega:
e_{\omega,\ell}
-
E^{rem}_{\omega,\ell}
\ge
e^{res}_v,\ \forall \ell
\right\}
$$

放电过多会缩小可行计划集合，甚至强制车辆再次充电。

---

## 10.2 对进站行为的影响

EV 进站不再只是由低 SOC 触发，而是由经济最优触发：

$$
\omega^*
=
\arg\min_{\omega}
C^E_{\omega}
$$

如果最优计划包含站点 $$s$$，则车辆进站。低价充电和高价放电补偿都可能导致额外进站需求。

因此充放电优化会改变站点到达流：

$$
u^S_{s,k}
$$

---

## 10.3 对站内停留时间的影响

充电量越大，服务时间越长：

$$
T^{ch}_{\omega}
=
\frac{E^{ch}_{\omega}}
{\eta^{ch}\bar p^{ch}_s}
$$

放电量越大，服务时间也越长：

$$
T^{dis}_{\omega}
=
\frac{E^{dis}_{\omega}\eta^{dis}}
{\bar p^{dis}_s}
$$

因此：

$$
T^{station}_{\omega}
=
W_{s,k}
+
T^{ch}_{\omega}
+
T^{dis}_{\omega}
+
T^{dwell}_{\omega}
$$

离站时间为：

$$
k^{dep}_{v,s}
=
k^{arr}_{v,s}
+
\left\lceil
\frac{T^{station}_{\omega}}{\Delta t}
\right\rceil
$$

于是充放电行为直接改变站点离站流：

$$
o_{s,k}
$$

---

## 10.4 对路径选择的影响

EV 的路径选择由经济成本决定，而经济成本包含站点价格和服务状态。

例如，若路径 $$r_1$$ 时间最短，但路径 $$r_2$$ 上有低价充电站，则可能出现：

$$
T_{r_1,k}<T_{r_2,k}
$$

但：

$$
C^E_{r_2,s,k}<C^E_{r_1,\varnothing,k}
$$

此时 EV 会选择更长但经济性更优的路径。

因此充放电行为改变路径流：

$$
x^E_{w,\omega,k}
$$

进一步改变 link inflow：

$$
u_{a,k}
$$

最终影响交通拥堵。

---

## 10.5 对道路传播的影响

充放电行为通过三类流影响路网。

### 第一类：改变普通道路 link flow

$$
u^E_{a,k}
=
\sum_{w,\omega}
\delta^{E}_{a,w,\omega,k}
x^E_{w,\omega,k}
$$

### 第二类：改变站点 access flow

$$
u^S_{s,k}
=
\sum_{w,\omega:s\in\omega}
\delta^{arr}_{s,w,\omega,k}
x^E_{w,\omega,k}
$$

### 第三类：改变站点 egress flow

$$
o_{s,k}
=
\sum_{w,\omega:s\in\omega}
\delta^{dep}_{s,w,\omega,k}
x^E_{w,\omega,k}
$$

这些变量进入 CTM/LTM 后，改变：

$$
N^{in}_{a,k},\quad
N^{out}_{a,k},\quad
n_{a,k},\quad
T_{a,k}
$$

再反馈到车辆行为层。

---

# 11. 动态均衡模型

## 11.1 Dynamic network loading operator

定义 DNL 算子：

$$
\mathcal{D}
:
(x^F,x^E)
\mapsto
(T,W,Q,o,N^{in},N^{out})
$$

其中：

- 输入为 fleet flow；
- 输出为旅行时间、等待时间、站点状态、累计曲线等。

---

## 11.2 ICV choice operator

ICV choice operator 为：

$$
\mathcal{A}^{F}
:
T
\mapsto
x^F
$$

若采用 all-or-nothing：

$$
\hat x^F_{w,r,k}
=
\begin{cases}
d^F_{w,k}, & r\in\arg\min_{r'}T_{r',k},\\
0, & \text{otherwise}.
\end{cases}
$$

---

## 11.3 EV choice operator

EV choice operator 为：

$$
\mathcal{A}^{E}
:
(T,W,\pi,Q)
\mapsto
x^E
$$

若采用 all-or-nothing：

$$
\hat x^E_{w,\omega,k}
=
\begin{cases}
d^E_{w,k}, & \omega\in\arg\min_{\omega'}C^E_{w,\omega',k},\\
0, & \text{otherwise}.
\end{cases}
$$

若采用 logit：

$$
P^E_{w,\omega,k}
=
\frac{
\exp
\left(
-\theta C^E_{w,\omega,k}
\right)
}{
\sum_{\omega'\in\Omega_{w,k}}
\exp
\left(
-\theta C^E_{w,\omega',k}
\right)
}
$$

$$
x^E_{w,\omega,k}
=
d^E_{w,k}P^E_{w,\omega,k}
$$

---

## 11.4 Fixed-point equilibrium

系统均衡满足：

$$
(x^{F,*},x^{E,*})
=
\mathcal{A}
\left(
\mathcal{D}
(x^{F,*},x^{E,*})
\right)
$$

其中：

$$
\mathcal{A}
=
(\mathcal{A}^{F},\mathcal{A}^{E})
$$

直观解释是：

> 给定 fleet flow，可以算出拥堵和排队；给定拥堵和排队，车辆会重新选择；均衡时，车辆不再有动力改变选择。

---

## 11.5 Wardrop-like 条件

ICV 满足：

$$
T_{w,r,k}=T^{\min}_{w,k}
\quad
\text{if}
\quad
x^F_{w,r,k}>0
$$

$$
T_{w,r,k}\ge T^{\min}_{w,k}
\quad
\text{if}
\quad
x^F_{w,r,k}=0
$$

EV 满足：

$$
C^E_{w,\omega,k}=C^{E,\min}_{w,k}
\quad
\text{if}
\quad
x^E_{w,\omega,k}>0
$$

$$
C^E_{w,\omega,k}\ge C^{E,\min}_{w,k}
\quad
\text{if}
\quad
x^E_{w,\omega,k}=0
$$

---

# 12. 求解算法

## 12.1 MSA 迭代框架

初始化：

$$
x^{F,(0)},x^{E,(0)}
$$

第 $$i$$ 次迭代包括如下步骤。

### Step 1：DNL propagation

$$
(T^{(i)},W^{(i)},Q^{(i)},o^{(i)})
=
\mathcal{D}
\left(
x^{F,(i)},x^{E,(i)}
\right)
$$

### Step 2：ICV shortest-time assignment

$$
\hat x^{F,(i)}
=
\mathcal{A}^{F}
\left(
T^{(i)}
\right)
$$

### Step 3：EV economic assignment

计算：

$$
C^{E,(i)}_{w,\omega,k}
$$

并得到：

$$
\hat x^{E,(i)}
=
\mathcal{A}^{E}
\left(
T^{(i)},W^{(i)},Q^{(i)},\pi
\right)
$$

### Step 4：MSA 更新

步长为：

$$
\lambda_i=
\frac{1}{i+1}
$$

更新：

$$
x^{F,(i+1)}
=
x^{F,(i)}
+
\lambda_i
\left(
\hat x^{F,(i)}
-
x^{F,(i)}
\right)
$$

$$
x^{E,(i+1)}
=
x^{E,(i)}
+
\lambda_i
\left(
\hat x^{E,(i)}
-
x^{E,(i)}
\right)
$$

---

## 12.2 收敛指标

ICV relative gap：

$$
\epsilon^F_i
=
\frac{
\sum_{w,k,r}
x^{F,(i)}_{w,r,k}
\left(
T^{(i)}_{w,r,k}
-
T^{\min,(i)}_{w,k}
\right)
}{
\sum_{w,k,r}
x^{F,(i)}_{w,r,k}
T^{\min,(i)}_{w,k}
}
$$

EV relative gap：

$$
\epsilon^E_i
=
\frac{
\sum_{w,k,\omega}
x^{E,(i)}_{w,\omega,k}
\left(
C^{E,(i)}_{w,\omega,k}
-
C^{E,\min,(i)}_{w,k}
\right)
}{
\sum_{w,k,\omega}
x^{E,(i)}_{w,\omega,k}
C^{E,\min,(i)}_{w,k}
}
$$

总 gap：

$$
\epsilon_i
=
\max
\left(
\epsilon^F_i,
\epsilon^E_i
\right)
$$

若：

$$
\epsilon_i<\epsilon^{tol}
$$

则停止。

---

# 13. 系统最优扩展

用户均衡描述个体最优行为，而系统最优用于整体调度和控制。

## 13.1 系统目标

系统目标可以包括：

- 总旅行时间；
- 总排队时间；
- 总充电成本；
- 电池退化成本；
- 电网购电成本；
- V2G 收益；
- 站点拥堵惩罚。

目标函数可写为：

$$
\min
J
=
\alpha_T
\sum_{a,k}
n_{a,k}\Delta t
+
\alpha_Q
\sum_{s,k}
Q_{s,k}\Delta t
+
\sum_{s,k}
\pi^{ch}_{s,k}P^{ch}_{s,k}\Delta t
-
\sum_{s,k}
\pi^{dis}_{s,k}P^{dis}_{s,k}\Delta t
+
\sum_{v,k}
C^{deg}_{v,k}
$$

subject to：

- fleet demand conservation；
- CTM 或 LTM propagation；
- station service dynamics；
- EV SOC constraints；
- charging/discharging power constraints；
- road capacity constraints；
- plug capacity constraints。

---

## 13.2 站点净负荷

站点总充电功率：

$$
P^{ch}_{s,k}
=
\sum_{v\in\mathcal{V}_{s,k}}
p^{ch}_{v,k}
$$

站点总放电功率：

$$
P^{dis}_{s,k}
=
\sum_{v\in\mathcal{V}_{s,k}}
p^{dis}_{v,k}
$$

站点净负荷：

$$
P^{net}_{s,k}
=
P^{ch}_{s,k}
-
P^{dis}_{s,k}
$$

映射到电网 bus：

$$
P^{EV}_{b,k}
=
\sum_{s:\mathrm{bus}(s)=b}
P^{net}_{s,k}
$$

---

# 14. 理论讨论

## 14.1 为什么需要微观车辆行为层？

CTM 和 LTM 本身只描述 traffic propagation，不能回答：

- EV 是否进站；
- 选择哪个站；
- 充多少电；
- 是否参与 V2G；
- 是否因为电价绕行；
- 放电后是否需要再次充电。

这些行为由单车约束和经济性目标决定，因此必须先建立微观行为模型。

但逐车模拟在大规模网络上计算昂贵，所以需要再聚合为 fleet flow：

$$
\text{micro behaviour}
\rightarrow
\text{fleet aggregation}
$$

---

## 14.2 为什么 ICV 和 EV 要用不同选择模型？

ICV 的约束较少，主要关注旅行时间。EV 具有额外能量约束：

$$
e_{v,k}
-
E^{rem}_{v,k}
\ge
e^{res}_v
$$

并且 EV 的 generalized cost 中包含能源价格和充电站排队。因此 EV 的选择不是简单最短时间，而是经济性最优。

---

## 14.3 充放电为何会反过来影响交通流？

充放电行为影响：

1. 是否进站；
2. 选择哪个站；
3. 站内停留多久；
4. 离站时间；
5. 是否因 SOC 变化改变后续路径；
6. 是否产生二次充电需求；
7. 是否造成 station access spillback。

这些都会改变：

$$
u_{a,k},\quad
u^S_{s,k},\quad
o_{s,k}
$$

而这些正是 CTM/LTM 的输入。因此充放电行为会改变交通流。

---

## 14.4 CTM 与 LTM 的角色差异

| 内容 | CTM | LTM |
|---|---|---|
| 状态变量 | cell occupancy | cumulative curves |
| 空间精度 | 高 | 中等 |
| 计算量 | 较大 | 较小 |
| FIFO 轨迹重构 | 近似 | 自然 |
| spillback | 强 | 强 |
| 适合用途 | 系统级传播、shockwave | EV 轨迹和站点耦合 |
| 对实现要求 | 需满足 CFL | 需处理时间延迟索引 |

建议：

- 若要分析 shockwave 和 cell-level spillback，用 CTM；
- 若要大规模 EV 行为、FIFO 轨迹、站点交互，用 LTM；
- 两者可以并存，CTM 做 benchmark，LTM 做主仿真。

---

## 14.5 用户均衡与系统最优的区别

用户均衡是每个车辆选择自身最优：

$$
C_{\text{used}}=C_{\min}
$$

系统最优是整体成本最小：

$$
\min J
$$

二者不一定一致。EV 经济性最优可能导致：

- 大量车辆涌向低价站点；
- 站点排队爆炸；
- access link 拥堵；
- 社会总成本上升。

因此需要价格或调度机制协调个体最优与系统最优。

---

# 15. 数值验证思路

数值验证应按照：

$$
\text{单模块验证}
\rightarrow
\text{耦合验证}
\rightarrow
\text{敏感性分析}
\rightarrow
\text{对照实验}
$$

的顺序进行。

---

## 15.1 验证一：ICV 最短时间路径选择

### 目的

验证 ICV fleet 能根据动态旅行时间选择最短路径。

### 网络

双路径网络：

- 路径 1：短但容量低；
- 路径 2：长但容量高。

### 实验

给定 OD demand，从低到高逐步增加。

### 预期结果

- 低需求时，全部或大部分车辆选择路径 1；
- 高需求时，路径 1 拥堵，部分车辆转向路径 2；
- 均衡时，已使用路径旅行时间接近相等。

验证指标：

$$
\left|T_{r_1,k}-T_{r_2,k}\right|
$$

relative gap：

$$
\epsilon^F_i
$$

---

## 15.2 验证二：EV SOC 可行性

### 目的

验证 EV 不会选择 SOC 不可行路径。

### 网络

设置三条路径：

1. 短路径，无充电站；
2. 长路径，有充电站；
3. 中等路径，但能耗高。

### 实验

改变 EV 初始 SOC：

$$
e^{0}_v
$$

### 预期结果

- 高 SOC：EV 选择经济成本最低路径；
- 中 SOC：EV 可能选择有站点路径；
- 低 SOC：无充电站路径被排除；
- 若所有路径不可行，出现 infeasible 标记。

验证条件：

$$
e_{v,k}
-
E^{rem}_{v,k}
\ge
e^{res}_v
$$

---

## 15.3 验证三：充电价格对路径和站点选择的影响

### 目的

验证 EV 经济性最优模型有效。

### 网络

两个站点：

- 站点 A：近，但电价高；
- 站点 B：远，但电价低。

### 实验

逐步降低站点 B 的电价：

$$
\pi^{ch}_{B,k}
$$

### 预期结果

存在阈值价格，使 EV 从 A 转向 B。

理论阈值由：

$$
C^E_{A}=C^E_{B}
$$

决定，即：

$$
\alpha_t
\left(
T_B-T_A
\right)
+
\alpha_w
\left(
W_B-W_A
\right)
+
\left(
\pi^{ch}_{B}E^{ch}_{B}
-
\pi^{ch}_{A}E^{ch}_{A}
\right)
=0
$$

当：

$$
C^E_B<C^E_A
$$

EV 选择 B。

---

## 15.4 验证四：V2G 放电对二次充电需求的影响

### 目的

验证放电会降低 SOC，并可能导致后续再次充电。

### 实验

设置一条较长 OD 路径，中途站点允许 V2G。

比较：

1. 不允许 V2G；
2. 允许 V2G 但放电少；
3. 允许 V2G 且放电多。

### 预期结果

- V2G 收益增加时，更多 EV 延长 dwell 并放电；
- 放电越多，后续 SOC 越低；
- 当放电超过阈值，EV 需要二次充电；
- 二次充电增加站点流量和旅行时间。

验证指标：

$$
E^{dis}_{v}
$$

$$
e_{v,k}
$$

$$
\#\text{charging stops}
$$

$$
T^{travel}_v
$$

---

## 15.5 验证五：站点排队与 plug 容量

### 目的

验证站点服务模型符合排队规律。

### 实验

固定充电需求，改变插枪数：

$$
\bar B_s
$$

或改变 EV 渗透率：

$$
p^{EV}
$$

### 预期结果

- 插枪数增加，平均等待时间下降；
- EV 渗透率增加，排队长度上升；
- 当到达率超过服务率，队列持续增长。

若平均服务时间为：

$$
\bar T^{service}_s
$$

服务能力近似为：

$$
\mu_s=
\frac{\bar B_s}{\bar T^{service}_s}
$$

当：

$$
\lambda_s>\mu_s
$$

队列不稳定。

验证指标：

$$
Q^{max}_s
$$

$$
\bar W_s
$$

$$
\frac{B_{s,k}}{\bar B_s}
$$

---

## 15.6 验证六：站点 spillback 对交通流的影响

### 目的

验证站点队列会反向影响道路交通。

### 实验

设置 station access link，给定有限站点排队容量：

$$
\bar Q_s
$$

当站点队列满时：

$$
R^{access}_{s,k}=0
$$

### 预期结果

- 站点队列满后，access link 入口受限；
- 上游 link occupancy 增加；
- link travel time 上升；
- 部分 EV 改选其他站点或绕行。

验证指标：

$$
Q_{s,k}
$$

$$
R^{access}_{s,k}
$$

$$
n_{a,k}
$$

$$
T_{a,k}
$$

---

## 15.7 验证七：CTM/LTM propagation 正确性

### 实验 A：自由流传播

单 link，输入脉冲 demand。

预期：

- outflow 相对 inflow 延迟自由流时间；
- 无容量约束时，形状保持。

LTM 下延迟为：

$$
\tau^{ff}_a\Delta t
$$

具体实现中若采用同步更新，可能出现额外一步延迟，需要单独说明为 implementation convention。

### 实验 B：容量约束

输入 demand 高于容量：

$$
d_{a,k}>\bar q_a
$$

预期：

$$
v_{a,k}\le \bar q_a\Delta t
$$

并且 link occupancy 上升。

### 实验 C：spillback

下游 receiving capacity 降低。

预期：

- 上游 outflow 受限；
- occupancy 增大；
- 队列向上游传播；
- jam storage 不被违反：

$$
n_{a,k}\le N^{jam}_a
$$

---

## 15.8 验证八：均衡收敛

### 目的

验证 MSA 或 logit 更新可以得到稳定结果。

### 实验

使用小型网络，多 OD、多路径、多个站点，运行迭代。

### 预期结果

- relative gap 下降；
- 路径流趋于稳定；
- used alternatives 的成本接近最小成本；
- 站点队列不再大幅振荡。

指标：

$$
\epsilon^F_i
$$

$$
\epsilon^E_i
$$

$$
\left\|x^{(i+1)}-x^{(i)}\right\|_1
$$

---

# 16. 推荐数值实验体系

## Case 1：基础 ICV-only 动态交通分配

目的：验证 CTM/LTM 和 ICV 最短时间行为。

输出：

- link inflow/outflow；
- link occupancy；
- path travel time；
- ICV assignment gap。

---

## Case 2：EV without charging optimization

目的：验证 EV SOC 和固定充电规则。

EV 规则：

$$
e_{v,k}-E^{rem}_{v,k}<e^{res}_v
$$

则进站。

输出：

- SOC profile；
- charging demand；
- station queue；
- EV travel time。

---

## Case 3：EV with economic charging optimization

目的：验证价格、等待时间、充电时间共同影响站点选择。

改变：

- 电价；
- plug 数；
- EV penetration；
- value of time。

输出：

- station market share；
- average charging cost；
- average waiting time；
- route switching rate。

---

## Case 4：EV with V2G

目的：验证放电对交通和能量的双向影响。

改变：

- V2G 补偿价格；
- minimum dwell time；
- reserve SOC；
- discharge power。

输出：

- V2G potential；
- actual discharged energy；
- secondary charging events；
- total travel time；
- total station delay；
- grid-side net load。

---

# 17. 关键输出指标

## 17.1 交通指标

总车辆在网时间：

$$
\mathrm{TTT}
=
\sum_{a,k}
n_{a,k}\Delta t
$$

总系统旅行时间：

$$
\mathrm{TSTT}
=
\sum_{w,r,k}
x^F_{w,r,k}T_{w,r,k}
+
\sum_{w,\omega,k}
x^E_{w,\omega,k}T_{w,\omega,k}
$$

平均速度：

$$
\bar v_{a,k}
=
\frac{L_a}{T_{a,k}}
$$

throughput：

$$
\mathrm{Throughput}
=
\sum_{a\in\mathcal{A}^{sink}}
N^{out}_{a,T}
$$

---

## 17.2 站点指标

最大队列：

$$
Q^{max}_s
=
\max_k Q_{s,k}
$$

平均等待时间：

$$
\bar W_s
=
\frac{
\sum_k Q_{s,k}\Delta t
}{
\sum_k u^S_{s,k}
}
$$

plug utilization：

$$
\rho_s
=
\frac{
\sum_k B_{s,k}
}{
\bar B_s T
}
$$

---

## 17.3 EV 能量指标

总充电量：

$$
E^{ch}_{tot}
=
\sum_{s,k}
P^{ch}_{s,k}\Delta t
$$

总放电量：

$$
E^{dis}_{tot}
=
\sum_{s,k}
P^{dis}_{s,k}\Delta t
$$

净负荷能量：

$$
E^{net}_{tot}
=
E^{ch}_{tot}
-
E^{dis}_{tot}
$$

SOC violation count：

$$
N^{viol}
=
\sum_{v,k}
\mathbf{1}
\left[
e_{v,k}
-
E^{rem}_{v,k}
<
e^{res}_v
\right]
$$

理论上应有：

$$
N^{viol}=0
$$

---

## 17.4 经济指标

EV 平均广义成本：

$$
\bar C^E
=
\frac{
\sum_{w,\omega,k}
x^E_{w,\omega,k}
C^E_{w,\omega,k}
}{
\sum_{w,k}d^E_{w,k}
}
$$

总能源成本：

$$
C^{energy}_{tot}
=
\sum_{s,k}
\left(
\pi^{ch}_{s,k}P^{ch}_{s,k}
-
\pi^{dis}_{s,k}P^{dis}_{s,k}
\right)\Delta t
$$

电池退化成本：

$$
C^{deg}_{tot}
=
\sum_{v,k}
C^{deg}_{v,k}
$$

---

# 18. 模型正确性检查清单

## 18.1 车辆守恒

对任意 link：

$$
N^{out}_{a,k}\le N^{in}_{a,k}
$$

对全网：

$$
\text{departed vehicles}
=
\text{arrived vehicles}
+
\text{vehicles in network}
+
\text{vehicles in stations}
$$

---

## 18.2 容量约束

道路容量：

$$
u_{a,k}\le \bar q_a\Delta t
$$

$$
v_{a,k}\le \bar q_a\Delta t
$$

站点 plug：

$$
B_{s,k}\le \bar B_s
$$

---

## 18.3 Jam storage

$$
n_{a,k}\le N^{jam}_a
$$

---

## 18.4 SOC 可行

$$
e^{min}_v\le e_{v,k}\le e^{max}_v
$$

$$
e_{v,k}-E^{rem}_{v,k}\ge e^{res}_v
$$

---

## 18.5 FIFO

若车辆 $$i$$ 比车辆 $$j$$ 更早进入同一 link：

$$
k^{in}_{i,a}\le k^{in}_{j,a}
$$

则应有：

$$
k^{out}_{i,a}\le k^{out}_{j,a}
$$

---

# 19. 结论

该理论模型可以总结为：

1. 车辆行为层建立 ICV 和 EV 的微观决策模型；
2. fleet 聚合层将大量车辆转化为 OD-时段-车型-路径或活动计划流；
3. ICV 采用最短时间或 Wardrop 时间均衡；
4. EV 采用经济性最优模型，成本包括旅行时间、电价、排队、服务时间、电池退化、V2G 收益和 SOC 约束；
5. CTM/LTM 负责将 fleet flow 动态传播到路网，计算拥堵、spillback、link travel time 和 FIFO 轨迹；
6. 充电站模型将 EV 的站点到达、排队、充放电、dwell 和离站内生化；
7. 充放电行为通过 SOC、进站选择、服务时间、离站时间、路径切换影响 traffic flow；
8. 动态均衡是车辆选择和网络传播之间的 fixed point；
9. 数值验证应从单模块到全耦合逐层验证，包括 SOC 可行性、站点排队、V2G 反馈、CTM/LTM propagation 和 equilibrium convergence。

一句话概括：

> ICV 主要通过时间最短路径选择影响交通流；EV 则通过经济性最优活动计划影响交通流，其中充放电行为改变 SOC、站点选择、停留时间和离站流，进而通过 CTM/LTM 改变道路传播、拥堵和动态均衡。