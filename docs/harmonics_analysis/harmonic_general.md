> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: design/analysis reference; confirm behavior against current implementation before adopting conclusions.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# 基于 Becker 等论文的混合 AC/DC 谐波潮流模型完善方案

你上传的论文 **“Harmonic Power-Flow Study of Hybrid AC/DC Grids with Converter-Interfaced Distributed Energy Resources”** 对前面我们讨论的模型有一个非常重要的提升：

> 它不是简单把电力电子设备等效成给定谐波电流源，而是把 CIDER / NIC 的控制器、滤波器、DC-link、AC/DC 耦合都纳入频域响应模型，并通过 Newton-Raphson 求解完整的谐波潮流不匹配方程。

结合你已有的**统一迭代交直流混合潮流模块**，建议把现有基波 AC/DC Newton 潮流模块扩展为一个更一般的：

> **统一迭代混合 AC/DC 多频率谐波潮流模块**

其核心思想是：

```text
已有模块：
    AC/DC 基波潮流统一 Newton 求解

扩展模块：
    AC/DC 多频率谐波潮流统一 Newton 求解
    +
    CIDER / NIC 频域响应模型
    +
    AC/DC 频率耦合 Jacobian
```

---

# 1. 论文的核心贡献应如何吸收到你的模型中？

该论文相对于普通谐波潮流的关键点有四个：

## 1.1 将资源分为单端口和双端口

论文将资源分为：

| 类型 | 含义 | 示例 |
|---|---|---|
| Single-port resource | 只连接一个子系统 | CIDER、负荷、电源、阻抗负荷 |
| Two-port resource | 连接两个子系统 | NIC，Network-Interfacing Converter |

在混合 AC/DC 系统中：

- 普通分布式电源、负荷、光伏逆变器可以看成单端口资源；
- AC/DC 变流器、VSC、LCC、储能 PCS、EV 充电站前端等可以看成双端口资源；
- NIC 是连接 AC 子系统和 DC 子系统的典型双端口资源。

---

## 1.2 将节点分为 grid-forming 与 grid-following

论文对每个子系统 $$j$$ 的节点集合进行划分：

$$
\mathcal{N}^{j}
=
\mathcal{S}^{j}
\cup
\mathcal{R}^{j}
$$

$$
\mathcal{S}^{j}
\cap
\mathcal{R}^{j}
=
\emptyset
$$

其中：

- $$\mathcal{S}^{j}$$：grid-forming 资源连接节点；
- $$\mathcal{R}^{j}$$：grid-following 资源连接节点。

并进一步细分为：

$$
\mathcal{R}^{j}
=
\mathcal{R}^{j}_{1}
\cup
\mathcal{R}^{j}_{2}
$$

$$
\mathcal{S}^{j}
=
\mathcal{S}^{j}_{1}
\cup
\mathcal{S}^{j}_{2}
$$

其中：

- 下标 $$1$$ 表示单端口资源；
- 下标 $$2$$ 表示双端口资源。

对于典型 AC/DC 混合网：

```text
AC subsystem:
    R_AC_1 : AC 侧 grid-following CIDER
    R_AC_2 : NIC 的 AC 端口

DC subsystem:
    S_DC_2 : NIC 的 DC 端口
    R_DC_1 : DC 侧负荷、电流源、DC DER
```

也就是说，典型 NIC 往往表现为：

```text
AC 侧：grid-following
DC 侧：grid-forming
```

这与实际 VSC/NIC 控制一致：

- AC 侧控制注入电流、有功无功；
- DC 侧控制直流母线电压。

---

## 1.3 采用“grid 方程”和“resource 方程”的不匹配

论文的 HPF 不是直接只写：

$$
\mathbf{Y}_h\mathbf{V}_h=\mathbf{I}_h
$$

而是写成：

```text
grid 看到的节点电压/电流
    -
resource 模型给出的节点电压/电流
    =
0
```

也就是通过不匹配方程求解：

$$
\Delta \hat{\mathbf{V}} = 0
$$

$$
\Delta \hat{\mathbf{I}} = 0
$$

这里的帽号 $$\hat{}$$ 表示**频域 Fourier 系数向量**，也就是把多个谐波的相量堆叠在一起。

---

## 1.4 Jacobian 写成资源 Jacobian 与网络 Jacobian 之差

论文给出：

$$
\hat{\mathbf{J}}
=
\hat{\mathbf{J}}_{RSC}
-
\hat{\mathbf{J}}_{GRD}
$$

其中：

- $$\hat{\mathbf{J}}_{GRD}$$：网络方程 Jacobian；
- $$\hat{\mathbf{J}}_{RSC}$$：资源模型 Jacobian；
- NIC 导致 AC/DC 子系统之间出现非零耦合块。

这点非常适合你的现有模块，因为你已经有了统一 AC/DC Newton 潮流框架，现在只需要将其扩展为**多频率复数 Newton 框架**。

---

# 2. 与你已有统一 AC/DC 潮流模块的对应关系

你现有模块的基波 AC/DC 潮流状态向量大致为：

$$
x_{PF}
=
\begin{bmatrix}
\theta_{\mathcal{N}_{ac}\setminus\mathcal{N}_{sl}} \\
v_{\mathcal{N}_{PQ}} \\
v^{dc}_{\mathcal{N}_{dc}\setminus\mathcal{N}_{sl}^{dc}}
\end{bmatrix}
$$

对应残差：

$$
F(x)=0
$$

包括：

- AC 有功不平衡；
- AC 无功不平衡；
- DC 功率不平衡；
- VSC AC/DC 耦合；
- DCDC 耦合；
- ZIP 负荷；
- converter mode switching。

而论文中的 HPF 模型可以看成把这个框架扩展为：

$$
\hat{x}_{HPF}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

其中每一个变量不再是单个基波标量，而是一个谐波向量，例如：

$$
\hat{V}
=
\begin{bmatrix}
V_{-H} \\
\vdots \\
V_{-1} \\
V_0 \\
V_1 \\
\vdots \\
V_H
\end{bmatrix}
$$

工程实现中，如果只考虑正频率，也可以用：

$$
\hat{V}
=
\begin{bmatrix}
V_0 \\
V_1 \\
V_2 \\
\vdots \\
V_H
\end{bmatrix}
$$

对 AC 系统一般取：

$$
h \in \{1,5,7,11,13,17,19,23,\ldots\}
$$

对 DC 系统一般取：

$$
r \in \{0,2,6,12,18,24,\ldots\}
$$

---

# 3. 建议的总体架构升级

你已有统一交直流潮流模块，建议不要重写，而是增加一个新的上层模块：

```text
src/harmonic_power_flow/
    harmonic_solver.cpp
    harmonic_network_builder.cpp
    harmonic_resource_models.cpp
    harmonic_converter_models.cpp
    harmonic_jacobian.cpp
    harmonic_postprocess.cpp
```

同时在模型层增加：

```text
include/hacdcpf/harmonics/
    harmonic_options.hpp
    harmonic_result.hpp
    harmonic_spectrum.hpp
    harmonic_resource.hpp
    harmonic_converter.hpp
    harmonic_frequency_set.hpp
```

建议保留原有基波 PF 模块作为：

```text
Step 0: operating point solver
```

即：

```text
solve_power_flow(system, options)
    ↓
base operating point
    ↓
solve_harmonic_power_flow(system, base_result, harmonic_options)
```

---

# 4. 扩展后的核心数学模型

## 4.1 多频率变量定义

定义 AC 谐波频率集合：

$$
\mathcal{H}_{ac}
=
\{h_1,h_2,\ldots,h_{n_h}\}
$$

对应频率：

$$
f_h = h f_1
$$

定义 DC 纹波频率集合：

$$
\mathcal{H}_{dc}
=
\{r_1,r_2,\ldots,r_{n_r}\}
$$

其中 DC 包含稳态分量：

$$
r=0
$$

以及纹波分量：

$$
r=2,6,12,\ldots
$$

对任一 AC 相节点 $$i$$：

$$
\hat{V}^{AC}_i
=
\begin{bmatrix}
V^{AC}_{i,h_1} \\
V^{AC}_{i,h_2} \\
\vdots \\
V^{AC}_{i,h_{n_h}}
\end{bmatrix}
$$

对任一 DC 节点 $$k$$：

$$
\hat{V}^{DC}_k
=
\begin{bmatrix}
V^{DC}_{k,r_1} \\
V^{DC}_{k,r_2} \\
\vdots \\
V^{DC}_{k,r_{n_r}}
\end{bmatrix}
$$

---

## 4.2 AC 网络频域方程

如果忽略频率间耦合，AC 网络为块对角：

$$
\hat{\mathbf{I}}^{AC}_{grid}
=
\hat{\mathbf{Y}}^{AC}_{grid}
\hat{\mathbf{V}}^{AC}
$$

其中：

$$
\hat{\mathbf{Y}}^{AC}_{grid}
=
\begin{bmatrix}
\mathbf{Y}^{AC}_{h_1} & 0 & \cdots & 0 \\
0 & \mathbf{Y}^{AC}_{h_2} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & \mathbf{Y}^{AC}_{h_{n_h}}
\end{bmatrix}
$$

每个 $$\mathbf{Y}^{AC}_{h}$$ 由线路、变压器、电容器、负荷阻尼、滤波器等在频率 $$h f_1$$ 下装配得到。

---

## 4.3 DC 网络频域方程

DC 纹波网络类似：

$$
\hat{\mathbf{I}}^{DC}_{grid}
=
\hat{\mathbf{Y}}^{DC}_{grid}
\hat{\mathbf{V}}^{DC}
$$

其中：

$$
\hat{\mathbf{Y}}^{DC}_{grid}
=
\begin{bmatrix}
\mathbf{Y}^{DC}_{r_1} & 0 & \cdots & 0 \\
0 & \mathbf{Y}^{DC}_{r_2} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & \mathbf{Y}^{DC}_{r_{n_r}}
\end{bmatrix}
$$

DC 线路频域阻抗：

$$
Z^{DC}_{line,r}
=
R_{dc,r}
+
j2\pi f_r L_{dc}
$$

DC 电容导纳：

$$
Y^{DC}_{C,r}
=
j2\pi f_r C_{dc}
$$

恒功率 DC 负荷小信号导纳：

$$
Y^{DC}_{CPL}
=
-\frac{P_{dc}}{V_{dc,0}^2}
$$

---

# 5. 引入论文中的 Hybrid 参数形式

论文中对每个子系统 $$j$$ 使用 hybrid 参数写网络方程：

$$
\hat{\mathbf{V}}^{j}_{S}
=
\hat{\mathbf{H}}^{j}_{S \times S}
\hat{\mathbf{I}}^{j}_{S}
+
\hat{\mathbf{H}}^{j}_{S \times R}
\hat{\mathbf{V}}^{j}_{R}
$$

$$
\hat{\mathbf{I}}^{j}_{R}
=
\hat{\mathbf{H}}^{j}_{R \times S}
\hat{\mathbf{I}}^{j}_{S}
+
\hat{\mathbf{H}}^{j}_{R \times R}
\hat{\mathbf{V}}^{j}_{R}
$$

这个形式很适合 HPF，因为 unknown 选为：

$$
\hat{x}^{j}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{j}_{S} \\
\hat{\mathbf{V}}^{j}_{R}
\end{bmatrix}
$$

也就是说：

- grid-forming 节点的未知量是注入电流；
- grid-following 节点的未知量是节点电压。

这和资源控制逻辑一致：

| 资源类型 | 资源已控制 | 未知量 |
|---|---|---|
| Grid-forming | 电压 | 电流 |
| Grid-following | 电流 | 电压 |

---

## 5.1 如何由节点导纳矩阵得到 hybrid 矩阵

对某一频率或多频率堆叠系统，有节点方程：

$$
\begin{bmatrix}
\hat{\mathbf{I}}_{S} \\
\hat{\mathbf{I}}_{R}
\end{bmatrix}
=
\begin{bmatrix}
\hat{\mathbf{Y}}_{SS} & \hat{\mathbf{Y}}_{SR} \\
\hat{\mathbf{Y}}_{RS} & \hat{\mathbf{Y}}_{RR}
\end{bmatrix}
\begin{bmatrix}
\hat{\mathbf{V}}_{S} \\
\hat{\mathbf{V}}_{R}
\end{bmatrix}
$$

需要表达成：

$$
\begin{bmatrix}
\hat{\mathbf{V}}_{S} \\
\hat{\mathbf{I}}_{R}
\end{bmatrix}
=
\begin{bmatrix}
\hat{\mathbf{H}}_{SS} & \hat{\mathbf{H}}_{SR} \\
\hat{\mathbf{H}}_{RS} & \hat{\mathbf{H}}_{RR}
\end{bmatrix}
\begin{bmatrix}
\hat{\mathbf{I}}_{S} \\
\hat{\mathbf{V}}_{R}
\end{bmatrix}
$$

由第一行：

$$
\hat{\mathbf{I}}_S
=
\hat{\mathbf{Y}}_{SS}\hat{\mathbf{V}}_S
+
\hat{\mathbf{Y}}_{SR}\hat{\mathbf{V}}_R
$$

若 $$\hat{\mathbf{Y}}_{SS}$$ 可逆，则：

$$
\hat{\mathbf{V}}_S
=
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{I}}_S
-
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{Y}}_{SR}
\hat{\mathbf{V}}_R
$$

因此：

$$
\hat{\mathbf{H}}_{SS}
=
\hat{\mathbf{Y}}_{SS}^{-1}
$$

$$
\hat{\mathbf{H}}_{SR}
=
-
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{Y}}_{SR}
$$

第二行：

$$
\hat{\mathbf{I}}_R
=
\hat{\mathbf{Y}}_{RS}\hat{\mathbf{V}}_S
+
\hat{\mathbf{Y}}_{RR}\hat{\mathbf{V}}_R
$$

代入 $$\hat{\mathbf{V}}_S$$：

$$
\hat{\mathbf{I}}_R
=
\hat{\mathbf{Y}}_{RS}
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{I}}_S
+
\left(
\hat{\mathbf{Y}}_{RR}
-
\hat{\mathbf{Y}}_{RS}
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{Y}}_{SR}
\right)
\hat{\mathbf{V}}_R
$$

所以：

$$
\hat{\mathbf{H}}_{RS}
=
\hat{\mathbf{Y}}_{RS}
\hat{\mathbf{Y}}_{SS}^{-1}
$$

$$
\hat{\mathbf{H}}_{RR}
=
\hat{\mathbf{Y}}_{RR}
-
\hat{\mathbf{Y}}_{RS}
\hat{\mathbf{Y}}_{SS}^{-1}
\hat{\mathbf{Y}}_{SR}
$$

---

## 5.2 编码建议

不要显式求逆，使用稀疏求解：

```cpp
H_SS = solve(Y_SS, I)
H_SR = -solve(Y_SS, Y_SR)
H_RS = Y_RS * H_SS
H_RR = Y_RR + Y_RS * H_SR
```

即：

$$
\hat{\mathbf{H}}_{SR}
=
-\operatorname{solve}
\left(
\hat{\mathbf{Y}}_{SS},
\hat{\mathbf{Y}}_{SR}
\right)
$$

---

# 6. 资源模型的统一表达

论文将资源响应写成：

## 6.1 单端口 grid-forming 资源

$$
s \in \mathcal{S}^{j}_{1}:
\quad
\hat{\mathbf{V}}^{j}_{s}
=
\hat{\mathbf{Y}}_{s}
\left(
\hat{\mathbf{I}}^{j}_{s},
\hat{\mathbf{W}}_{\sigma,s},
\hat{\mathbf{Y}}_{o,s}
\right)
$$

这里的 $$\hat{\mathbf{Y}}_s$$ 不是导纳矩阵，而是论文中泛化的资源响应函数。为了避免与网络导纳混淆，编码中建议命名为：

```cpp
ResourceResponse
```

或者数学上写为：

$$
\hat{\mathbf{V}}^{j}_{s}
=
\hat{\mathbf{F}}^{V}_{s}
\left(
\hat{\mathbf{I}}^{j}_{s},
\hat{\mathbf{w}}_{s},
\hat{\mathbf{y}}_{o,s}
\right)
$$

---

## 6.2 单端口 grid-following 资源

$$
r \in \mathcal{R}^{j}_{1}:
\quad
\hat{\mathbf{I}}^{j}_{r}
=
\hat{\mathbf{F}}^{I}_{r}
\left(
\hat{\mathbf{V}}^{j}_{r},
\hat{\mathbf{w}}_{r},
\hat{\mathbf{y}}_{o,r}
\right)
$$

例如：

- PQ 型逆变器；
- 恒功率负荷；
- 谐波电流源；
- Norton 谐波模型；
- CIDER。

---

## 6.3 双端口 NIC 资源

论文中 NIC 的典型形式：

$$
m=(r,s)
\in
\mathcal{M}
\subseteq
\mathcal{R}^{AC}_{2}
\times
\mathcal{S}^{DC}_{2}
$$

即：

- AC 端口在 $$\mathcal{R}^{AC}_{2}$$ 中，是 grid-following；
- DC 端口在 $$\mathcal{S}^{DC}_{2}$$ 中，是 grid-forming。

NIC 响应为：

$$
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{r} \\
\hat{\mathbf{V}}^{DC}_{s}
\end{bmatrix}
=
\hat{\mathbf{F}}_{m}
\left(
\hat{\mathbf{V}}^{AC}_{r},
\hat{\mathbf{I}}^{DC}_{s},
\hat{\mathbf{w}}_{m},
\hat{\mathbf{y}}_{o,m}
\right)
$$

这正是你之前想扩展的 AC/DC converter 模型的更严格形式。

---

# 7. 完整 HPF mismatch 方程

## 7.1 AC 子系统 grid 方程

$$
\hat{\mathbf{V}}^{AC}_{S}
=
\hat{\mathbf{H}}^{AC}_{SS}
\hat{\mathbf{I}}^{AC}_{S}
+
\hat{\mathbf{H}}^{AC}_{SR}
\hat{\mathbf{V}}^{AC}_{R}
$$

$$
\hat{\mathbf{I}}^{AC}_{R}
=
\hat{\mathbf{H}}^{AC}_{RS}
\hat{\mathbf{I}}^{AC}_{S}
+
\hat{\mathbf{H}}^{AC}_{RR}
\hat{\mathbf{V}}^{AC}_{R}
$$

---

## 7.2 DC 子系统 grid 方程

$$
\hat{\mathbf{V}}^{DC}_{S}
=
\hat{\mathbf{H}}^{DC}_{SS}
\hat{\mathbf{I}}^{DC}_{S}
+
\hat{\mathbf{H}}^{DC}_{SR}
\hat{\mathbf{V}}^{DC}_{R}
$$

$$
\hat{\mathbf{I}}^{DC}_{R}
=
\hat{\mathbf{H}}^{DC}_{RS}
\hat{\mathbf{I}}^{DC}_{S}
+
\hat{\mathbf{H}}^{DC}_{RR}
\hat{\mathbf{V}}^{DC}_{R}
$$

---

## 7.3 单端口 grid-forming mismatch

对 $$s \in \mathcal{S}^{j}_{1}$$：

$$
\Delta \hat{\mathbf{V}}^{j}_{s}
=
\hat{\mathbf{F}}^{V}_{s}
\left(
\hat{\mathbf{I}}^{j}_{s}
\right)
-
\hat{\mathbf{V}}^{j,grid}_{s}
=
0
$$

其中：

$$
\hat{\mathbf{V}}^{j,grid}_{s}
=
\hat{\mathbf{H}}^{j}_{sS}
\hat{\mathbf{I}}^{j}_{S}
+
\hat{\mathbf{H}}^{j}_{sR}
\hat{\mathbf{V}}^{j}_{R}
$$

---

## 7.4 单端口 grid-following mismatch

对 $$r \in \mathcal{R}^{j}_{1}$$：

$$
\Delta \hat{\mathbf{I}}^{j}_{r}
=
\hat{\mathbf{F}}^{I}_{r}
\left(
\hat{\mathbf{V}}^{j}_{r}
\right)
-
\hat{\mathbf{I}}^{j,grid}_{r}
=
0
$$

其中：

$$
\hat{\mathbf{I}}^{j,grid}_{r}
=
\hat{\mathbf{H}}^{j}_{rS}
\hat{\mathbf{I}}^{j}_{S}
+
\hat{\mathbf{H}}^{j}_{rR}
\hat{\mathbf{V}}^{j}_{R}
$$

---

## 7.5 双端口 NIC mismatch

对 NIC $$m=(r,s)$$：

AC 侧 mismatch：

$$
\Delta \hat{\mathbf{I}}^{AC}_{r}
=
\hat{\mathbf{F}}^{I,AC}_{m}
\left(
\hat{\mathbf{V}}^{AC}_{r},
\hat{\mathbf{I}}^{DC}_{s}
\right)
-
\hat{\mathbf{I}}^{AC,grid}_{r}
=
0
$$

DC 侧 mismatch：

$$
\Delta \hat{\mathbf{V}}^{DC}_{s}
=
\hat{\mathbf{F}}^{V,DC}_{m}
\left(
\hat{\mathbf{V}}^{AC}_{r},
\hat{\mathbf{I}}^{DC}_{s}
\right)
-
\hat{\mathbf{V}}^{DC,grid}_{s}
=
0
$$

这两个方程引入 AC/DC 交叉偏导数。

---

# 8. Jacobian 结构完善

论文中 Jacobian 写成：

$$
\hat{\mathbf{J}}
=
\hat{\mathbf{J}}_{RSC}
-
\hat{\mathbf{J}}_{GRD}
$$

这非常适合模块化实现。

---

## 8.1 状态向量

建议谐波潮流状态向量定义为：

$$
\hat{x}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

其中每个元素是复数多频率向量。

如果求解器只支持实数，可以展开为：

$$
x_{real}
=
\begin{bmatrix}
\Re(\hat{x}) \\
\Im(\hat{x})
\end{bmatrix}
$$

对于 DC 的零频分量 $$r=0$$，虚部可省略或固定为 0。为了实现简洁，可以先保留虚部但约束其为 0。

---

## 8.2 Grid Jacobian

网络 Jacobian 为块对角结构：

$$
\hat{\mathbf{J}}_{GRD}
=
\begin{bmatrix}
\hat{\mathbf{H}}^{AC}_{SS} & \hat{\mathbf{H}}^{AC}_{SR} & 0 & 0 \\
\hat{\mathbf{H}}^{AC}_{RS} & \hat{\mathbf{H}}^{AC}_{RR} & 0 & 0 \\
0 & 0 & \hat{\mathbf{H}}^{DC}_{SS} & \hat{\mathbf{H}}^{DC}_{SR} \\
0 & 0 & \hat{\mathbf{H}}^{DC}_{RS} & \hat{\mathbf{H}}^{DC}_{RR}
\end{bmatrix}
$$

注意：

> 网络自身没有 AC/DC 耦合，AC/DC 耦合全部来自 NIC 或 converter 资源模型。

这与论文一致。

---

## 8.3 Resource Jacobian

资源 Jacobian 包含：

1. 单端口 AC 资源偏导；
2. 单端口 DC 资源偏导；
3. NIC AC 侧对 AC 电压的偏导；
4. NIC AC 侧对 DC 电流的偏导；
5. NIC DC 侧对 AC 电压的偏导；
6. NIC DC 侧对 DC 电流的偏导。

对 NIC：

$$
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{r} \\
\hat{\mathbf{V}}^{DC}_{s}
\end{bmatrix}
=
\hat{\mathbf{F}}_{m}
\left(
\hat{\mathbf{V}}^{AC}_{r},
\hat{\mathbf{I}}^{DC}_{s}
\right)
$$

其偏导为：

$$
\frac{\partial \hat{\mathbf{I}}^{AC}_{r}}
{\partial \hat{\mathbf{V}}^{AC}_{r}}
$$

$$
\frac{\partial \hat{\mathbf{I}}^{AC}_{r}}
{\partial \hat{\mathbf{I}}^{DC}_{s}}
$$

$$
\frac{\partial \hat{\mathbf{V}}^{DC}_{s}}
{\partial \hat{\mathbf{V}}^{AC}_{r}}
$$

$$
\frac{\partial \hat{\mathbf{V}}^{DC}_{s}}
{\partial \hat{\mathbf{I}}^{DC}_{s}}
$$

这些就是 AC/DC 耦合项。

---

# 9. 与你现有 VSC 模型的衔接

你现有基波模型中 VSC 支持：

- PQ_MODE；
- VDC_Q；
- VDC_VAC；
- 损耗模型；
- AC/DC 耦合 Jacobian；
- converter mode switching。

对于 HPF，需要把 VSC/NIC 的模型扩展为频域响应。

建议保留现有基波 VSC 模型作为 operating point model，然后新增：

```cpp
class HarmonicConverterModel {
public:
    virtual void initialize_from_base_pf(
        const VSCConverter& conv,
        const PowerFlowResult& base
    ) = 0;

    virtual void eval_response(
        const HarmonicStateView& x,
        HarmonicResidual& residual
    ) const = 0;

    virtual void stamp_resource_jacobian(
        const HarmonicStateView& x,
        SparseMatrix& Jrsc
    ) const = 0;
};
```

---

# 10. NIC 模型建议

结合论文，NIC 是最关键的双端口设备。

建议定义：

```cpp
enum class NICControlMode {
    VDC_Q,
    PQ,
    VDC_VAC,
    GRID_FORMING_AC_GRID_FOLLOWING_DC
};
```

典型论文场景主要有：

```text
AC side: grid-following
DC side: grid-forming
```

即：

```text
input:
    V_AC at AC port
    I_DC at DC port

output:
    I_AC at AC port
    V_DC at DC port
```

数学接口：

$$
\begin{bmatrix}
\hat{\mathbf{I}}_{ac} \\
\hat{\mathbf{V}}_{dc}
\end{bmatrix}
=
\hat{\mathbf{F}}_{NIC}
\left(
\hat{\mathbf{V}}_{ac},
\hat{\mathbf{I}}_{dc},
\hat{\mathbf{w}},
\hat{\mathbf{y}}_o
\right)
$$

---

## 10.1 Level 1 NIC：频谱 + Norton 模型

初期最容易实现：

$$
\hat{\mathbf{I}}_{ac}
=
\hat{\mathbf{I}}^{src}_{ac}
-
\hat{\mathbf{Y}}^{out}_{ac}
\hat{\mathbf{V}}_{ac}
+
\hat{\mathbf{K}}_{ad}
\hat{\mathbf{I}}_{dc}
$$

$$
\hat{\mathbf{V}}_{dc}
=
\hat{\mathbf{V}}^{src}_{dc}
-
\hat{\mathbf{Z}}^{out}_{dc}
\hat{\mathbf{I}}_{dc}
+
\hat{\mathbf{K}}_{da}
\hat{\mathbf{V}}_{ac}
$$

其中：

- $$\hat{\mathbf{Y}}^{out}_{ac}$$：AC 侧输出导纳；
- $$\hat{\mathbf{Z}}^{out}_{dc}$$：DC 侧输出阻抗；
- $$\hat{\mathbf{K}}_{ad}$$：DC 对 AC 的耦合；
- $$\hat{\mathbf{K}}_{da}$$：AC 对 DC 的耦合。

对应 Jacobian：

$$
\frac{\partial \hat{\mathbf{I}}_{ac}}{\partial \hat{\mathbf{V}}_{ac}}
=
-\hat{\mathbf{Y}}^{out}_{ac}
$$

$$
\frac{\partial \hat{\mathbf{I}}_{ac}}{\partial \hat{\mathbf{I}}_{dc}}
=
\hat{\mathbf{K}}_{ad}
$$

$$
\frac{\partial \hat{\mathbf{V}}_{dc}}{\partial \hat{\mathbf{V}}_{ac}}
=
\hat{\mathbf{K}}_{da}
$$

$$
\frac{\partial \hat{\mathbf{V}}_{dc}}{\partial \hat{\mathbf{I}}_{dc}}
=
-\hat{\mathbf{Z}}^{out}_{dc}
$$

这个模型已经能表达论文中的 AC/DC coupling Jacobian 结构。

---

## 10.2 Level 2 NIC：LCL + DC-link 小信号模型

更接近论文的模型包含：

```text
AC side:
    LCL filter

DC side:
    DC-link capacitor

Control:
    VDC/Q control
    or P/Q control
```

可建立小信号状态空间：

$$
\dot{x}(t)
=
A(t)x(t)
+
B(t)u(t)
$$

$$
y(t)
=
C(t)x(t)
+
D(t)u(t)
$$

由于系统在稳态下是周期时变的，论文将其通过 Fourier 变换转为谐波域 Toeplitz 矩阵。

频域中：

$$
s\hat{x}
=
\hat{\mathbf{A}}\hat{x}
+
\hat{\mathbf{B}}\hat{u}
$$

其中微分算子对应：

$$
\hat{\mathbf{D}}
=
j\Omega
$$

$$
\Omega
=
\operatorname{diag}
\left(
\ldots,
-2\omega_1,
-\omega_1,
0,
\omega_1,
2\omega_1,
\ldots
\right)
$$

因此：

$$
\left(
j\Omega
-
\hat{\mathbf{A}}
\right)
\hat{x}
=
\hat{\mathbf{B}}\hat{u}
$$

输出：

$$
\hat{y}
=
\hat{\mathbf{C}}\hat{x}
+
\hat{\mathbf{D}}\hat{u}
$$

消去状态：

$$
\hat{y}
=
\left[
\hat{\mathbf{C}}
\left(
j\Omega-\hat{\mathbf{A}}
\right)^{-1}
\hat{\mathbf{B}}
+
\hat{\mathbf{D}}
\right]
\hat{u}
$$

这就是谐波传递函数。

---

# 11. Toeplitz 谐波矩阵实现建议

论文提到通过 Fourier 分析和 Toeplitz 矩阵将线性周期时变状态空间模型变换到频域。

如果某个周期量：

$$
a(t)
=
\sum_{k=-K}^{K}
a_k e^{jk\omega_1 t}
$$

它与另一个周期量相乘：

$$
y(t)=a(t)x(t)
$$

在频域中变为卷积：

$$
Y_h
=
\sum_k
a_k X_{h-k}
$$

矩阵形式为 Toeplitz：

$$
\hat{\mathbf{y}}
=
\mathcal{T}(\hat{\mathbf{a}})
\hat{\mathbf{x}}
$$

Toeplitz 矩阵元素：

$$
\mathcal{T}(\hat{\mathbf{a}})_{h,l}
=
a_{h-l}
$$

---

## 11.1 编码函数

建议新增：

```cpp
SparseMatrix build_toeplitz(
    const std::vector<int>& harmonic_orders,
    const std::unordered_map<int, Complex>& coeffs
);
```

伪代码：

```cpp
for row_h in harmonics:
    for col_l in harmonics:
        k = row_h - col_l;
        if coeffs.contains(k):
            T(row, col) = coeffs[k];
```

---

# 12. 多频率资源响应的统一接口

建议定义：

```cpp
struct HarmonicPortVariables {
    ComplexVector voltage;  // stacked harmonics
    ComplexVector current;  // stacked harmonics
};

struct HarmonicResourceResponse {
    ComplexVector output;
    SparseMatrix jacobian_wrt_local_inputs;
};
```

对于单端口 grid-following：

```cpp
I_hat = resource.eval_current(V_hat)
dI_dV = resource.jacobian_current_wrt_voltage(V_hat)
```

对于单端口 grid-forming：

```cpp
V_hat = resource.eval_voltage(I_hat)
dV_dI = resource.jacobian_voltage_wrt_current(I_hat)
```

对于 NIC：

```cpp
[Iac_hat, Vdc_hat] = nic.eval(Vac_hat, Idc_hat)

dIac_dVac
dIac_dIdc
dVdc_dVac
dVdc_dIdc
```

---

# 13. 建议新增的数据结构

## 13.1 HarmonicFrequencySet

```cpp
struct HarmonicFrequencySet {
    double fundamental_hz = 50.0;

    std::vector<int> ac_orders;      // e.g. {1,5,7,11,13}
    std::vector<int> dc_orders;      // e.g. {0,2,6,12}

    bool include_negative_orders = false;
    bool include_zero_sequence = true;
};
```

---

## 13.2 HarmonicOptions

```cpp
struct HarmonicPowerFlowOptions {
    HarmonicFrequencySet frequency_set;

    int max_iter = 20;
    double tol = 1e-8;
    double damping = 1.0;

    bool use_hybrid_parameter_form = true;
    bool use_resource_jacobian = true;
    bool use_acdc_coupling = true;
    bool use_converter_toeplitz_model = false;

    bool initialize_from_base_pf = true;
    bool solve_base_pf_first = true;

    bool real_expanded_system = true;
    bool enable_line_search = true;
};
```

---

## 13.3 HarmonicSpectrum

```cpp
struct HarmonicSpectrumPoint {
    int order = 1;
    double magnitude_percent = 100.0;
    double angle_deg = 0.0;
};

struct HarmonicSpectrum {
    std::vector<HarmonicSpectrumPoint> points;
};
```

---

## 13.4 HarmonicResult

```cpp
struct HarmonicPowerFlowResult {
    bool converged = false;
    int iterations = 0;
    double residual = 0.0;

    PowerFlowResult base_pf;

    std::unordered_map<int, ComplexVector> ac_voltage_by_order;
    std::unordered_map<int, ComplexVector> ac_current_by_order;

    std::unordered_map<int, ComplexVector> dc_voltage_by_order;
    std::unordered_map<int, ComplexVector> dc_current_by_order;

    std::vector<double> ac_voltage_thd;
    std::vector<double> ac_current_thd;

    std::vector<double> dc_voltage_ripple;
    std::vector<double> dc_current_ripple;
};
```

---

# 14. 建议新增组件字段

你现有模型已经有丰富的 AC/DC/VSC/DCDC/PV/Storage/ChargingStation 数据。为了支持谐波潮流，建议在以下组件上新增可选字段。

---

## 14.1 ACBranch / DCBranch

```cpp
struct HarmonicLineData {
    bool frequency_dependent = true;
    double skin_effect_alpha = 0.5;
    double capacitance_nf_per_km = 0.0;
    double inductance_mh_per_km = 0.0;

    bool use_distributed_parameter_model = false;
};
```

---

## 14.2 Load / DCLoad

```cpp
enum class HarmonicLoadModel {
    ConstantImpedance,
    ParallelRL,
    SeriesRL,
    CIGRE,
    ConstantPowerSmallSignal,
    UserDefined
};

struct HarmonicLoadData {
    HarmonicLoadModel model;
    HarmonicSpectrum injected_current_spectrum;
    bool is_nonlinear = false;
};
```

---

## 14.3 VSCConverter

```cpp
enum class HarmonicConverterModelType {
    SpectrumCurrentSource,
    NortonOutputImpedance,
    LCL_DCLinkSmallSignal,
    ToeplitzClosedLoop,
    UserDefined
};

struct VSCConverterHarmonicData {
    HarmonicConverterModelType model_type;

    HarmonicSpectrum ac_current_spectrum;
    HarmonicSpectrum dc_ripple_spectrum;

    double filter_l1_pu;
    double filter_l2_pu;
    double filter_c_pu;
    double filter_r_pu;
    double dc_link_capacitance_pu;

    bool enable_acdc_coupling = true;
    bool enable_frequency_coupling = false;
};
```

---

## 14.4 DCDCConverter

```cpp
struct DCDCHarmonicData {
    bool enable_ripple_model = true;
    HarmonicSpectrum input_ripple_spectrum;
    HarmonicSpectrum output_ripple_spectrum;

    double input_capacitance_pu;
    double output_capacitance_pu;
    double equivalent_inductance_pu;
    double switching_frequency_hz;
};
```

---

# 15. 算法流程：基于现有统一 AC/DC PF 的 HPF

下面给出推荐流程。

---

## 15.1 总流程

```text
Algorithm HPF-Hybrid-ACDC

Input:
    HybridPowerSystem system
    PowerFlowOptions pf_options
    HarmonicPowerFlowOptions hpf_options

Output:
    HarmonicPowerFlowResult

Step 1:
    若 hpf_options.solve_base_pf_first = true
    调用已有 solve_power_flow(system, pf_options)

Step 2:
    从基波潮流结果提取 operating point:
        AC 节点基波电压
        DC 节点稳态电压
        VSC / DCDC / EnergyRouter 运行点
        负荷、电源、变流器注入
        控制模式

Step 3:
    建立谐波频率索引:
        AC harmonic orders
        DC ripple orders
        频率到全局变量位置的映射

Step 4:
    对 AC 子系统建立多频率导纳矩阵 Yhat_AC
    对 DC 子系统建立多频率导纳矩阵 Yhat_DC

Step 5:
    根据节点资源类型划分:
        S_AC, R_AC, S_DC, R_DC
        进一步划分 single-port 和 two-port

Step 6:
    从 Yhat_AC 构造 Hhat_AC
    从 Yhat_DC 构造 Hhat_DC

Step 7:
    初始化谐波状态 xhat:
        使用频谱源给初值
        或所有非基波初始化为 0
        基波由 PF 结果初始化

Step 8:
    Newton 迭代:
        8.1 计算 grid response
        8.2 计算 resource response
        8.3 mismatch = resource response - grid response
        8.4 装配 J = Jrsc - Jgrd
        8.5 求解 J Δx = -mismatch
        8.6 线搜索/阻尼更新 x
        8.7 判断收敛

Step 9:
    从 xhat 恢复所有节点电压和电流

Step 10:
    计算支路谐波电流、THD、DC 纹波率、换流器谐波贡献

Step 11:
    返回结果
```

---

## 15.2 Newton 迭代伪代码

````markdown
```cpp
HarmonicPowerFlowResult solve_harmonic_power_flow(
    const HybridPowerSystem& system,
    const PowerFlowOptions& pf_opt,
    const HarmonicPowerFlowOptions& hpf_opt)
{
    HarmonicPowerFlowResult result;

    // Step 1: base operating point
    PowerFlowResult base;
    if (hpf_opt.solve_base_pf_first) {
        base = solve_power_flow(system, pf_opt);
        if (!base.converged) {
            result.converged = false;
            return result;
        }
    }

    // Step 2: build harmonic indexing
    HarmonicIndex index = build_harmonic_index(system, hpf_opt.frequency_set);

    // Step 3: classify nodes
    ResourcePartition partition = classify_harmonic_nodes(system, base);

    // Step 4: build multi-frequency network matrices
    SparseMatrix Yac_hat = build_ac_multifrequency_ybus(system, index, base, hpf_opt);
    SparseMatrix Ydc_hat = build_dc_multifrequency_ybus(system, index, base, hpf_opt);

    // Step 5: build hybrid matrices
    HybridMatrix Hac = build_hybrid_matrix(Yac_hat, partition.ac_S, partition.ac_R);
    HybridMatrix Hdc = build_hybrid_matrix(Ydc_hat, partition.dc_S, partition.dc_R);

    // Step 6: initialize resource models
    std::vector<std::unique_ptr<HarmonicResource>> resources =
        build_harmonic_resources(system, base, index, hpf_opt);

    // Step 7: initialize unknown vector
    ComplexVector x = initialize_harmonic_state(system, base, index, partition);

    for (int iter = 0; iter < hpf_opt.max_iter; ++iter) {
        ComplexVector F;
        SparseMatrix Jgrid;
        SparseMatrix Jrsc;

        // Grid response and grid Jacobian
        evaluate_grid_response_and_jacobian(
            x, Hac, Hdc, partition, F.grid, Jgrid);

        // Resource response and resource Jacobian
        evaluate_resource_response_and_jacobian(
            x, resources, partition, F.resource, Jrsc);

        ComplexVector mismatch = F.resource - F.grid;

        double norm_inf = inf_norm(mismatch);
        if (norm_inf < hpf_opt.tol) {
            result.converged = true;
            result.iterations = iter;
            break;
        }

        SparseMatrix J = Jrsc - Jgrid;

        ComplexVector dx = solve_sparse_complex(J, -mismatch);

        double alpha = hpf_opt.damping;
        if (hpf_opt.enable_line_search) {
            alpha = line_search_hpf(x, dx, mismatch, resources, Hac, Hdc);
        }

        x += alpha * dx;
    }

    recover_harmonic_solution(result, x, system, base, index, partition, Hac, Hdc);
    compute_harmonic_indices(result, system, index, base);

    return result;
}
```
````

---

# 16. 如果暂时不想实现 hybrid 参数法怎么办？

如果你的现有潮流模块更习惯直接用节点导纳形式，也可以先实现一个**直接节点电压形式**的 HPF：

$$
\hat{\mathbf{Y}}_{net}\hat{\mathbf{V}}
-
\hat{\mathbf{I}}_{resource}(\hat{\mathbf{V}})
=
0
$$

其中：

$$
\hat{\mathbf{V}}
=
\begin{bmatrix}
\hat{\mathbf{V}}^{AC} \\
\hat{\mathbf{V}}^{DC}
\end{bmatrix}
$$

残差：

$$
\hat{\mathbf{F}}
=
\begin{bmatrix}
\hat{\mathbf{Y}}^{AC}\hat{\mathbf{V}}^{AC} \\
\hat{\mathbf{Y}}^{DC}\hat{\mathbf{V}}^{DC}
\end{bmatrix}
-
\hat{\mathbf{I}}_{resource}
\left(
\hat{\mathbf{V}}^{AC},
\hat{\mathbf{V}}^{DC}
\right)
=
0
$$

Jacobian：

$$
\hat{\mathbf{J}}
=
\begin{bmatrix}
\hat{\mathbf{Y}}^{AC} & 0 \\
0 & \hat{\mathbf{Y}}^{DC}
\end{bmatrix}
-
\frac{\partial \hat{\mathbf{I}}_{resource}}
{\partial \hat{\mathbf{V}}}
$$

如果资源都是 Norton 形式，这种模型非常容易实现。

但它的缺点是：

- grid-forming 资源的处理不如论文自然；
- 电压源节点、DC 电压控制节点处理复杂；
- 不如 hybrid 参数法统一。

因此推荐路线是：

```text
短期：直接 YV-I(V)=0 形式
中期：加入 Norton 和 AC/DC 耦合
长期：实现论文 hybrid 参数 HPF
```

---

# 17. 如何复用你现有 AC/DC Newton PF 的能力？

你现有模块有很多可直接复用的内容。

## 17.1 可复用内容

| 现有模块 | HPF 中的用途 |
|---|---|
| HybridPowerSystem | 直接作为输入系统 |
| ACSystem / DCSystem | 构造 AC/DC 谐波网络 |
| VSCConverter | 作为 NIC / converter 资源 |
| DCDCConverter | 作为 DC/DC 纹波资源 |
| SolverData | 可扩展为 HarmonicSolverData |
| ybus / gdc builder | 扩展为频率相关 Ybus builder |
| PowerFlowResult | HPF operating point |
| SparseLU / sparse pattern cache | HPF Newton 求解 |
| robust nonlinear pipeline | HPF Newton 阻尼与线搜索 |
| converter mode switching | 可作为 HPF 初始化状态 |

---

## 17.2 需要新增内容

| 新增内容 | 说明 |
|---|---|
| HarmonicIndex | 多频率变量索引 |
| HarmonicNetworkBuilder | 频率相关 AC/DC 矩阵 |
| HarmonicResourceModel | CIDER、NIC、负荷、源响应 |
| HarmonicConverterModel | VSC/NIC/LCC/DCDC 频域模型 |
| ToeplitzBuilder | 周期时变系统频域矩阵 |
| HarmonicJacobianBuilder | 资源 Jacobian 与网络 Jacobian |
| HarmonicPostprocessor | THD、纹波率、支路谐波电流 |

---

# 18. 建议的实现路线

## 阶段 1：与你现有 PF 最兼容的版本

实现目标：

```text
基于已有 AC/DC PF operating point
构造 AC/DC 多频率网络
变流器采用给定 spectrum
AC/DC 谐波解耦求解
```

方程：

$$
\mathbf{Y}^{AC}_{h}\mathbf{V}^{AC}_{h}
=
\mathbf{I}^{AC}_{h}
$$

$$
\mathbf{Y}^{DC}_{r}\mathbf{V}^{DC}_{r}
=
\mathbf{I}^{DC}_{r}
$$

这是最小可运行版本。

---

## 阶段 2：加入 Norton converter 模型

实现：

$$
\mathbf{I}^{AC}_{conv,h}
=
\mathbf{I}^{AC}_{N,h}
-
\mathbf{Y}^{AC}_{out,h}
\mathbf{V}^{AC}_{h}
$$

$$
\mathbf{I}^{DC}_{conv,r}
=
\mathbf{I}^{DC}_{N,r}
-
\mathbf{Y}^{DC}_{out,r}
\mathbf{V}^{DC}_{r}
$$

矩阵变为：

$$
\left(
\mathbf{Y}^{AC}_{h}
+
\mathbf{Y}^{AC}_{out,h}
\right)
\mathbf{V}^{AC}_{h}
=
\mathbf{I}^{AC}_{N,h}
$$

$$
\left(
\mathbf{Y}^{DC}_{r}
+
\mathbf{Y}^{DC}_{out,r}
\right)
\mathbf{V}^{DC}_{r}
=
\mathbf{I}^{DC}_{N,r}
$$

---

## 阶段 3：加入 AC/DC coupling

实现论文中的 NIC 耦合结构：

$$
\begin{bmatrix}
\mathbf{Y}^{AC}_{h} & -\mathbf{K}_{ad,hr} \\
-\mathbf{K}_{da,rh} & \mathbf{Y}^{DC}_{r}
\end{bmatrix}
\begin{bmatrix}
\mathbf{V}^{AC}_{h} \\
\mathbf{V}^{DC}_{r}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{I}^{AC}_{h} \\
\mathbf{I}^{DC}_{r}
\end{bmatrix}
$$

这是与论文最接近的直接节点形式。

---

## 阶段 4：实现 hybrid 参数 HPF

实现论文完整形式：

$$
\hat{\mathbf{J}}
=
\hat{\mathbf{J}}_{RSC}
-
\hat{\mathbf{J}}_{GRD}
$$

并使用状态：

$$
\hat{x}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

---

## 阶段 5：实现 Toeplitz closed-loop converter

实现论文中基于 LTP state-space 的 converter 谐波响应：

$$
\hat{y}
=
\left[
\hat{\mathbf{C}}
\left(
j\Omega-\hat{\mathbf{A}}
\right)^{-1}
\hat{\mathbf{B}}
+
\hat{\mathbf{D}}
\right]
\hat{u}
$$

适合研究：

- 控制器参数对谐波传播的影响；
- LCL 滤波器谐振；
- AC/DC 纹波耦合；
- 多谐波间耦合。

---

# 19. 与论文验证指标一致的输出

论文使用 harmonic phasor 误差指标：

$$
e_{abs}(X_h)
=
\max_p
\left|
|X_{h,p}^{HPF}|
-
|X_{h,p}^{TDS}|
\right|
$$

$$
e_{arg}(X_h)
=
\max_p
\left|
\angle X_{h,p}^{HPF}
-
\angle X_{h,p}^{TDS}
\right|
$$

你可以在结果结构中增加：

```cpp
struct HarmonicValidationKPI {
    std::unordered_map<int, double> voltage_abs_error;
    std::unordered_map<int, double> voltage_angle_error;
    std::unordered_map<int, double> current_abs_error;
    std::unordered_map<int, double> current_angle_error;
};
```

如果未来要和 EMT / Simulink / PSCAD 对比，这个指标非常有用。

---

# 20. 推荐的最终统一模型表达

结合你已有 PF 模块和该论文，推荐最终文档中将 HPF 写成如下统一形式。

---

## 20.1 基波 operating point

$$
F_{PF}
\left(
x_{PF}
\right)
=
0
$$

得到：

$$
x_{0}
=
\left(
\mathbf{V}^{AC}_{1},
\mathbf{V}^{DC}_{0},
\mathbf{I}^{AC}_{1},
\mathbf{I}^{DC}_{0},
\mathbf{u}_{conv}
\right)
$$

---

## 20.2 谐波潮流未知量

$$
\hat{x}_{HPF}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

---

## 20.3 Grid response

$$
\hat{g}
\left(
\hat{x}_{HPF}
\right)
=
\hat{\mathbf{J}}_{GRD}
\hat{x}_{HPF}
$$

其中 $$\hat{\mathbf{J}}_{GRD}$$ 由 AC/DC hybrid matrix 构造。

---

## 20.4 Resource response

$$
\hat{r}
\left(
\hat{x}_{HPF},
x_0,
\hat{w}
\right)
=
\begin{bmatrix}
\hat{\mathbf{F}}^{AC}_{single} \\
\hat{\mathbf{F}}^{AC/DC}_{NIC} \\
\hat{\mathbf{F}}^{DC}_{single}
\end{bmatrix}
$$

---

## 20.5 Mismatch

$$
\hat{F}_{HPF}
\left(
\hat{x}_{HPF}
\right)
=
\hat{r}
\left(
\hat{x}_{HPF}
\right)
-
\hat{g}
\left(
\hat{x}_{HPF}
\right)
=
0
$$

---

## 20.6 Newton step

$$
\hat{\mathbf{J}}
\Delta \hat{x}
=
-
\hat{F}_{HPF}
$$

其中：

$$
\hat{\mathbf{J}}
=
\hat{\mathbf{J}}_{RSC}
-
\hat{\mathbf{J}}_{GRD}
$$

更新：

$$
\hat{x}^{k+1}
=
\hat{x}^{k}
+
\alpha
\Delta \hat{x}
$$

---

# 21. 对你现有文档的建议修改点

你已有 `03_power_flow.tex` 主要描述基波 PF。建议新增一章：

```text
04_harmonic_power_flow.tex
```

结构建议：

```text
1. Scope and Architecture
2. Frequency-Domain Variables
3. AC/DC Harmonic Network Models
4. Resource Classification
5. Single-Port Resource Responses
6. Network-Interfacing Converter Responses
7. Hybrid Parameter Formulation
8. HPF Mismatch Equations
9. HPF Jacobian Structure
10. Solver Algorithm
11. Relation to Existing Hybrid PF Solver
12. Result Metrics: THD, Ripple, KPIs
13. Implementation Roadmap
```

---

# 22. 简化版实现和论文完整版的关系

你可以在代码中提供三种求解路径：

```cpp
enum class HarmonicSolverMode {
    SequentialLinear,
    CoupledLinear,
    UnifiedNewtonHybrid
};
```

对应：

## SequentialLinear

```text
AC 和 DC 各频率分开求解
适合 spectrum 电流源模型
```

## CoupledLinear

```text
AC/DC 通过 converter coupling block 联合求解
适合 Norton + linear coupling converter
```

## UnifiedNewtonHybrid

```text
使用论文的 resource-grid mismatch
适合 nonlinear / closed-loop / frequency-coupled converter
```

---

# 23. 一句话总结

基于该论文，你的模型应从：

```text
给定谐波源 + 频率相关 Ybus 求解
```

升级为：

```text
基于 operating point 的多频率资源响应模型
+
AC/DC 子系统 hybrid network response
+
NIC 引起的 AC/DC 耦合 Jacobian
+
Newton-Raphson 求解 resource-grid mismatch
```

最关键的统一公式是：

$$
\hat{F}_{HPF}
=
\hat{r}
\left(
\hat{x}
\right)
-
\hat{g}
\left(
\hat{x}
\right)
=
0
$$

以及：

$$
\hat{\mathbf{J}}
=
\hat{\mathbf{J}}_{RSC}
-
\hat{\mathbf{J}}_{GRD}
$$

这与你已有的统一 AC/DC Newton 潮流模块非常契合。你可以将现有模块作为**基波 operating point 求解器**，再在其上增加一个**多频率 harmonic Newton layer**，逐步从线性谐波源模型升级到论文中的 NIC/CIDER 闭环频域响应模型。