# 锥规划（LP/SOCP/SDP）内点求解器：推导与设计

本文档记录 MIPSolvers 原生锥规划求解器 `NativeConicIPM` 的数学推导与工程设计：它以 CVXOPT `conelp` 算法为蓝本，在统一的 cvxopt 标准型下用 Nesterov–Todd 缩放（Nesterov–Todd scaling）的 Mehrotra 预测-校正原始-对偶内点法（primal-dual interior-point method）求解 LP / SOCP / SDP。阅读对象是需要修改该内核的工程师——每节给出问题、推导与实现锚点。

目录
- [1. 概述](#1-概述)
- [2. 标准型与对偶](#2-标准型与对偶)
- [3. 对数障碍与中心路径](#3-对数障碍与中心路径)
- [4. Nesterov–Todd 缩放](#4-nesterovtodd-缩放)
- [5. Newton 方程与 Schur 补](#5-newton-方程与-schur-补)
- [6. 主算法（Mehrotra 预测-校正）](#6-主算法mehrotra-预测-校正)
- [7. KKT 线性代数](#7-kkt-线性代数)
- [8. 每迭代复杂度](#8-每迭代复杂度)
- [9. 并行设计](#9-并行设计)
- [10. 使用示例](#10-使用示例)
- [11. 已知限制与后续方向](#11-已知限制与后续方向)
- [参考](#参考)

---

## 1. 概述

### 1.1 能力矩阵

求解器接受 cvxopt 风格的锥笛卡尔积 $\mathcal{K} = \mathbb{R}^l_+ \times \prod_i \mathcal{Q}^{q_i} \times \prod_j \mathcal{S}^{s_j}_+$，三类锥统一由 `ConeDims{l, q, s}` 描述（`include/mipsolvers/engine/problem_types.hpp:182`）：

| 锥块 | 数学形式 | `ConeDims` 字段 | 占用行数 | 锥度 $\nu$ 贡献 |
|---|---|---|---|---|
| 非负象限（nonnegative orthant） | $u \ge 0$，$u \in \mathbb{R}^l$ | `l` | $l$ | $l$ |
| 二阶锥（second-order cone, SOC / Lorentz cone） | $u_0 \ge \|u_1\|_2$，$u \in \mathcal{Q}^{k}$ | `q[i] = k` | $k$ | 每锥 1 |
| 半定锥（positive-semidefinite cone） | $U \succeq 0$，$U \in \mathcal{S}^{p}$ | `s[j] = p` | $p(p+1)/2$（svec 打包） | $p$ |

LP 是 `q`、`s` 均空的特例；SOCP 是 `s` 为空的特例。`ConeDims::total()` 给出锥行总数 $m = l + \sum_i q_i + \sum_j s_j(s_j+1)/2$；尺寸为 1 的 SOC 块在内部被折叠为象限行处理（见 §4）。

### 1.2 架构位置

```
AML 建模层            add_soc_constraint / add_rotated_soc_constraint / add_psd_constraint
  (src/aml/model.cpp  compile_conic(): 组装 cvxopt 标准型 ConicModel)
        │
        ▼
Engine 问题类型       engine::ConicModel + ProblemClass::CONIC
  (include/mipsolvers/engine/problem_types.hpp)
        │
        ▼
适配器                NativeConicIPMAdapter（name = "NativeConicIPM"）
  (src/engine/solver/native/conic_ipm_adapter.cpp)
        │
        ▼
内核                  ConicIPMSolver —— Mehrotra 预测-校正 IPM
  (src/engine/kernel/ipm/conic_ipm_solver.cpp)
   ├─ 锥代数 ConeNtScaling / max_step / svec        (src/engine/kernel/ipm/cones.cpp)
   └─ KKT 后端  CHOLMOD 超节点 LLᵀ（SPD 快路径）→ MUMPS 对称不定 LDLᵀ（回退）
```

### 1.3 文件地图

| 路径 | 内容 |
|---|---|
| `include/mipsolvers/engine/problem_types.hpp` | `ConeDims`、`ConicModel`（cvxopt 标准型） |
| `include/mipsolvers/engine/kernel/ipm/cones.hpp` / `src/engine/kernel/ipm/cones.cpp` | svec/smat、SOC Jordan 代数、NT 缩放、max_step、内部点检验 |
| `include/mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp` / `src/engine/kernel/ipm/conic_ipm_solver.cpp` | `ConicIPMOptions`、`ConicIPMResult`、主迭代、KKT 装配与分解 |
| `src/engine/solver/native/conic_ipm_adapter.cpp` | `NativeConicIPMAdapter`，`constraint_duals = [z | y]` |
| `include/mipsolvers/aml/model.hpp` / `src/aml/model.cpp` | AML 锥约束 API 与 `compile_conic()` |
| `include/mipsolvers/aml/solve_result.hpp` | `conic_dual_vals` 布局约定 |
| `tests/test_conic_ipm.cpp` | 内核单元/端到端测试（含解析最优解） |
| `tests/test_aml_conic.cpp` | AML 层测试（SOC / 旋转锥 / PSD / 对偶布局） |

---

## 2. 标准型与对偶

### 2.1 原始-对偶对

内核求解 cvxopt 标准型锥线性规划（conic linear program）：

$$
\begin{aligned}
\text{(P)}\quad & \min_{x}\; c^\top x
  & \text{(D)}\quad & \max_{y,z}\; -h^\top z - b^\top y \\
\text{s.t.}\quad & Gx + s = h,\; Ax = b,\; s \in \mathcal{K}
  & \text{s.t.}\quad & G^\top z + A^\top y + c = 0,\; z \in \mathcal{K}
\end{aligned}
$$

其中 $G \in \mathbb{R}^{m \times n}$、$A \in \mathbb{R}^{m_{eq} \times n}$，$G$ 与 $h$ 的行按锥块排序 `[l | q blocks | svec-packed s blocks]`（`problem_types.hpp:204`）。对偶间隙（duality gap）为

$$
c^\top x - (-h^\top z - b^\top y) = (h - Gx)^\top z + (Ax - b)^\top y = s^\top z,
$$

在可行点处恰等于互补松弛积 $s^\top z$，它也是算法实际跟踪的量（`conic_ipm_solver.cpp:760`）。Maximize 模型在进入内核前整体取负，结果在出口处还原符号。

### 2.2 svec 约定与内积保持

半定块以 svec（symmetric vectorization）打包：$p \times p$ 对称矩阵按**列主序下三角**展开为长度 $p(p+1)/2$ 的向量，且**非对角元乘以 $\sqrt{2}$**：

$$
\operatorname{svec}(X) = \big(X_{11}, \sqrt{2}X_{21}, \ldots, \sqrt{2}X_{p1}, X_{22}, \sqrt{2}X_{32}, \ldots, X_{pp}\big)^\top .
$$

**内积保持证明。** 对对称 $S, Z$，Frobenius 内积 $\operatorname{tr}(SZ) = \sum_i S_{ii}Z_{ii} + 2\sum_{i>j} S_{ij}Z_{ij}$。而

$$
\operatorname{svec}(S)^\top \operatorname{svec}(Z) = \sum_i S_{ii}Z_{ii} + \sum_{i>j} (\sqrt{2}S_{ij})(\sqrt{2}Z_{ij}) = \sum_i S_{ii}Z_{ii} + 2\sum_{i>j}S_{ij}Z_{ij} = \operatorname{tr}(SZ).
$$

因此打包空间中的普通点积就是矩阵内积，锥成员性（$S \succeq 0$）、障碍梯度、NT 算子都可直接在打包向量上表达；这正是 §5 中 SDP Schur 元素能写成 $\operatorname{tr}(X_i M X_j M)$ 的原因。实现：`cones.cpp:67`（`svec`）、`cones.cpp:79`（`smat`，逆变换，非对角除以 $\sqrt{2}$）；测试锚点 `tests/test_conic_ipm.cpp:135`。

### 2.3 三类锥的自对偶性

标准型 (D) 中 $z \in \mathcal{K}$ 与原始 $s \in \mathcal{K}$ 同锥——这依赖三类锥均自对偶（self-dual，$\mathcal{K}^* = \mathcal{K}$）：

- **$\mathbb{R}^l_+$**：$z^\top s \ge 0\ \forall s \ge 0 \iff z \ge 0$（取 $s = e_i$ 即得必要性）。
- **$\mathcal{Q}^k$**：若 $z_0 \ge \|z_1\|$ 且 $s_0 \ge \|s_1\|$，则 $z^\top s = z_0 s_0 + z_1^\top s_1 \ge z_0 s_0 - \|z_1\|\|s_1\| \ge 0$（Cauchy–Schwarz）。反之，若 $z_0 < \|z_1\|$，取 $s = (\|z_1\|, -z_1) \in \mathcal{Q}^k$，则 $z^\top s = z_0\|z_1\| - \|z_1\|^2 < 0$。
- **$\mathcal{S}^p_+$**：$S, Z \succeq 0$ 时 $\operatorname{tr}(SZ) = \operatorname{tr}(Z^{1/2} S Z^{1/2}) \ge 0$；反之若 $S$ 有负特征值 $\lambda(v) < 0$，取 $Z = vv^\top \succeq 0$ 得 $\operatorname{tr}(SZ) = v^\top S v < 0$。

自对偶 + 自尺度（self-scaled）是 NT 缩放存在（§4）与同一内点框架统一处理三锥的理论基础。

> 实现锚点：`include/mipsolvers/engine/problem_types.hpp:176-213`（ConeDims/ConicModel 与行序约定）、`src/engine/kernel/ipm/cones.cpp:97`（`ConeLayout::build`：块偏移、锥度 $\nu$、单位向量 $e$）。

---

## 3. 对数障碍与中心路径

每类锥配备对数齐次障碍函数（logarithmically homogeneous barrier）：

| 锥 | 障碍 $f(u)$ | 齐次度 |
|---|---|---|
| $\mathbb{R}^l_+$ | $-\sum_{i=1}^l \ln u_i$ | $l$ |
| $\mathcal{Q}^k$ | $-\ln\big(u_0^2 - \|u_1\|^2\big)$ | 2（算法锥度按每锥 1 计，见下） |
| $\mathcal{S}^p_+$ | $-\ln \det U$ | $p$ |

以 $\mu > 0$ 为障碍参数，原始-对偶**中心路径**（central path）由扰动 KKT 条件定义：

$$
Gx + s = h, \qquad Ax = b, \qquad G^\top z + A^\top y + c = 0, \qquad s \circ z = \mu e, \qquad s, z \succ_{\mathcal{K}} 0,
$$

其中 $\circ$ 为各锥对应的 Jordan 积（Euclidean Jordan algebra 乘积；象限为逐分量乘积，SOC 见 §4.2，半定锥为对称积 $S \circ Z = \tfrac{1}{2}(SZ + ZS)$），$e$ 为 Jordan 代数单位元（象限全 1；SOC $(1, 0)$；半定锥 $\operatorname{svec}(I)$）。代码中算法的锥度取

$$
\nu = l + N_q + \sum_j p_j
$$

（$N_q$ 为 SOC 块数，`cones.cpp:112`），并令 $\mu = s^\top z / \nu$（`conic_ipm_solver.cpp:761`）。$\mu \to 0$ 时中心路径收敛到原始-对偶最优对；Mehrotra 算法的全部工作就是在保持严格内部的同时沿路径把 $\mu$ 压下去。

> 实现锚点：`cones.cpp:116-130`（单位向量 $e$ 的打包构造）、`conic_ipm_solver.cpp:760-761`（gap 与 $\mu$）。

---

## 4. Nesterov–Todd 缩放

原始-对偶内点法每步在缩放空间（scaled space）中取 Newton 方向。Nesterov–Todd（NT）缩放为每对严格内部点 $(s, z)$ 选取锥的自尺度变换，使原始点与对偶点映到**同一点** $\lambda$（缩放点，scaling point）：

$$
\lambda = P\, s = D\, z,
$$

其中 $P$（原始缩放）、$D$（对偶缩放）逐锥块作用，Newton 矩阵中只出现组合算子 $H = P^{-1} D$ 与其逆（§5）。以下逐锥给出推导；算子与 `cones.cpp` 中 `ConeNtScaling` 的六个 `apply_*` 成员一一对应。

### 4.1 非负象限块 $\mathbb{R}^l_+$

逐分量取

$$
d_i = \sqrt{s_i / z_i}, \qquad \lambda_i = \sqrt{s_i z_i}, \qquad W = \operatorname{diag}(d).
$$

则 $W z = \operatorname{diag}(d) z$ 与 $W^{-1} s$ 的第 $i$ 分量都是 $\sqrt{s_i z_i}$，即 $\lambda = Wz = W^{-1}s$。组合算子为对角阵

$$
H = W^\top W = \operatorname{diag}(d_i^2) = \operatorname{diag}(s_i / z_i), \qquad
H^{-1} = \operatorname{diag}(1 / d_i^2).
$$

尺寸为 1 的 SOC 块 $\mathcal{Q}^1 = \mathbb{R}_+$ 在代码中折叠为「额外的象限行」（`l_extra_`，`cones.cpp:393-401`），与 $l$ 块共用同一套标量算子。

### 4.2 二阶锥块 $\mathcal{Q}^k$（$k \ge 2$）

**Jordan 代数。** 取 Minkowski 符号矩阵 $J = \operatorname{diag}(1, -1, \ldots, -1)$，定义

$$
\det(u) = u_0^2 - \|u_1\|^2, \qquad
u \circ v = \big(u^\top v,\; u_0 v_1 + v_0 u_1\big), \qquad
u^{-1} = \frac{Ju}{\det(u)},
$$

单位元 $e = (1, 0)$。$u \succ_{\mathcal{Q}} 0 \iff u_0 > 0 \land \det(u) > 0$。内部点的 Jordan 平方根为

$$
v = \sqrt[\circ]{u}: \quad v_0 = \sqrt{\tfrac{u_0 + \rho}{2}}, \quad v_1 = \frac{u_1}{2 v_0}, \qquad \rho = \sqrt{\det(u)},
$$

可直接验证 $v \circ v = u$（实现 `q_jordan_sqrt`，`cones.cpp:156`；恒等式测试 `tests/test_conic_ipm.cpp:157`）。

**NT 点构造。** 记 $\rho(u) = \sqrt{\det(u)}$，归一化 $\bar{s} = s / \rho(s)$、$\bar{z} = z / \rho(z)$（此时 $\det(\bar{s}) = \det(\bar{z}) = 1$），再令

$$
\gamma = \sqrt{\frac{1 + \bar{s}^\top \bar{z}}{2}}, \qquad
\bar{w} = \frac{\bar{s} + J\bar{z}}{2\gamma}, \qquad
v = \sqrt[\circ]{\bar{w}}, \qquad
\beta = \sqrt{\frac{\rho(s)}{\rho(z)}}.
$$

$\bar{w}$ 即归一化 NT 缩放点（$\det(\bar{w}) = 1$）。缩放算子取 $v$ 的二次表示（quadratic representation）：

$$
W x = \beta\,\big(2 v (v^\top x) - J x\big), \qquad
W^{-1} x = \beta^{-1}\,\big(2 (Jv)\big((Jv)^\top x\big) - J x\big),
$$

满足定义恒等式 $\lambda = W z = W^{-1} s$（`cones.cpp:324-343`；测试 `tests/test_conic_ipm.cpp:170`）。

**H 与 H⁻¹。** $W$ 对称，故 $H = W^\top W = W^2 = \beta^2 (2vv^\top - J)^2$。利用恒等式

$$
\big(2vv^\top - J\big)^2 = 2\,(v \circ v)(v \circ v)^\top - J, \qquad v \circ v = \bar{w},
$$

得到闭式

$$
H = \beta^2\,\big(2 \bar{w}\bar{w}^\top - J\big), \qquad
H^{-1} = \beta^{-2}\,\big(2 (J\bar{w})(J\bar{w})^\top - J\big).
$$

两者都是「$J$ 的低秩修正」，作用一次只需 $O(k)$：$Hx = \beta^2\big(2\bar{w}(\bar{w}^\top x) - Jx\big)$，$H^{-1}x = \beta^{-2}\big(2(J\bar{w})((J\bar{w})^\top x) - Jx\big)$（实现 `apply_h` / `apply_h_inv`，`cones.cpp:497-521`；互逆性测试 `tests/test_conic_ipm.cpp:196-214`）。§5 将看到，$G^\top H^{-1} G$ 的 SOC 贡献因此是「稀疏部分 $-\beta^{-2} G_q^\top J G_q$ + 秩 1 修正」。

### 4.3 半定锥块 $\mathcal{S}^p_+$

在矩阵空间推导（打包空间由 svec 内积保持一一对应）。设严格内部对 $S \succ 0$、$Z \succ 0$ 的 Cholesky 分解

$$
S = L_s L_s^\top, \qquad Z = L_z L_z^\top,
$$

对 $L_z^\top L_s$ 做奇异值分解（SVD）：

$$
L_z^\top L_s = U \Sigma V^\top, \qquad \Sigma = \operatorname{diag}(\sigma_1, \ldots, \sigma_p),\; \sigma_i > 0,
$$

并定义 NT 合同变换（congruence）矩阵与其逆转置

$$
R = L_s V \Sigma^{-1/2}, \qquad R^{-T} = L_s^{-T} V \Sigma^{1/2}.
$$

**核心恒等式** $R^\top Z R = R^{-1} S R^{-T} = \Sigma$：

$$
R^\top Z R = \Sigma^{-1/2} V^\top L_s^\top L_z L_z^\top L_s V \Sigma^{-1/2}
           = \Sigma^{-1/2} V^\top (V \Sigma U^\top)(U \Sigma V^\top) V \Sigma^{-1/2} = \Sigma,
$$

$$
R^{-1} S R^{-T} = \big(L_s^{-T}V\Sigma^{1/2}\big)^\top L_s L_s^\top \big(L_s^{-T}V\Sigma^{1/2}\big)
                = \Sigma^{1/2} V^\top L_s^{-1} L_s L_s^\top L_s^{-T} V \Sigma^{1/2} = \Sigma.
$$

即同一合同把 $S$、$Z$ 同时对角化为 $\Sigma$，缩放点为对角阵：

$$
\lambda = \operatorname{svec}\big(\operatorname{diag}(\Sigma)\big).
$$

代码中 $R^{-T}$ 不显式求逆，而是用三角求解 $L_s^\top X = V \Sigma^{1/2}$ 得到（`cones.cpp:373-377`；恒等式测试 `tests/test_conic_ipm.cpp:220`）。

**算子（以代码为准）。** `ConeNtScaling` 对半定块实现为合同变换（`cones.cpp:427-528`）：

| 算子 | 矩阵空间作用 | 代码变量 |
|---|---|---|
| `apply_w`（$P^{-1}$，原始去缩放） | $X \mapsto R X R^\top$ | `r` |
| `apply_w_inv`（$P$，原始缩放） | $X \mapsto Rti^\top X\, Rti$（即 $R^{-1} X R^{-T}$） | `rti` |
| `apply_wt`（$D$，对偶缩放） | $X \mapsto R^\top X R$ | `r` |
| `apply_wt_inv`（$D^{-1}$，对偶去缩放） | $X \mapsto Rti\, X\, Rti^\top$ | `rti` |
| `apply_h`（$H = P^{-1}D$） | $X \mapsto (RR^\top)\, X\, (RR^\top)$ | `h_cong = R R^\top` |
| `apply_h_inv`（$H^{-1} = D^{-1}P$） | $X \mapsto M X M$，$M = Rti \cdot Rti^\top$ | `h_inv_cong = Rti\, Rti^\top` |

注意 $M = R^{-T}R^{-1} = (RR^\top)^{-1}$，故 $H$ 与 $H^{-1}$ 的矩阵形式互逆；$H$（或 $H^{-1}$）是 $\mathbb{R}^{p(p+1)/2}$ 上的稠密线性算子，代码从不显式成形，而以两次合同矩阵乘实现（互逆与 $H = P^{-1}D$ 的往返测试见 `tests/test_conic_ipm.cpp:254-273`）。

### 4.4 锥度

中心路径参数按锥度归一：

$$
\nu = l + N_q + \sum_j p_j, \qquad \mu = \frac{s^\top z}{\nu}.
$$

每个 SOC 块不论尺寸均计 1（代码 `degree = l + q.size() + Σp`，`cones.cpp:112-115`）；尺寸-1 块虽走象限算子路径，其锥度贡献不变。

> 实现锚点：`cones.cpp:279-410`（`ConeNtScaling::compute`，q/s 块可并行）、`cones.hpp:118-131`（算子语义注释）。

---

## 5. Newton 方程与 Schur 补

### 5.1 3×3 系统

定义当前点的三类残差

$$
r_c = G^\top z + A^\top y + c \;(\text{对偶}), \qquad
r_x = Ax - b \;(\text{原始等式}), \qquad
r_s = Gx + s - h \;(\text{原始锥行}).
$$

固定当前 NT 缩放 $W$，在缩放空间 $\lambda = Wz = W^{-1}s$ 中线性化互补条件 $\lambda \circ \lambda = \mu e$。以象限为例：$z_i\, ds_i + s_i\, dz_i = r_i$ 两边除以 $z_i$ 得 $ds_i + (s_i/z_i)\, dz_i = r_i / z_i$，系数正是 $H_{ii} = s_i/z_i$，右端即 $W r_\lambda$ 的第 $i$ 项；CVXOPT 的 Jordan 代数推导表明同一结构对三锥统一成立。代入 $ds = -r_s - G\,dx$（锥行方程的 Newton 方程），得到每次迭代要求解的 3×3 系统

$$
\begin{bmatrix} 0 & A^\top & G^\top \\ A & 0 & 0 \\ G & 0 & -H \end{bmatrix}
\begin{bmatrix} dx \\ dy \\ dz \end{bmatrix}
=
\begin{bmatrix} -r_c \\ -r_x \\ -r_s - W\, r_\lambda \end{bmatrix},
$$

其中 $r_\lambda$ 为缩放空间的右端（预测步 $r_\lambda = -\lambda$；校正步见 §6.4）。这就是 `conic_ipm_solver.cpp:6-9` 头注释中的系统。

### 5.2 消元到 2×2

由第三行 $dz = H^{-1}(G\,dx - t)$，$t = -r_s - W r_\lambda$；代回第一行得

$$
\underbrace{\begin{bmatrix} G^\top H^{-1} G + \delta I & A^\top \\ A & -\delta I \end{bmatrix}}_{K}
\begin{bmatrix} dx \\ dy \end{bmatrix}
=
\begin{bmatrix} -r_c + G^\top H^{-1} t \\ -r_x \end{bmatrix},
\qquad
dz = H^{-1}(G\,dx - t), \quad ds = -r_s - G\,dx .
$$

$\delta I$ 与 $-\delta I$ 是拟定（quasi-definite）正则化：$\delta > 0$ 保证无等式时 $K$ 对称正定（SPD），有等式时保持稳定的对称不定结构（δ 策略见 §7）。回代见 `solve_direction` lambda（`conic_ipm_solver.cpp:837-853`）。

### 5.3 $B = G^\top H^{-1} G$ 的分块贡献

$B$ 是每迭代的装配对象。按锥块分解：

**$l$ 行与尺寸-1 SOC 行（稀疏对角加权）。** $H^{-1}$ 为对角 $\operatorname{diag}(1/d_r^2)$，故

$$
B_l = G_l^\top \operatorname{diag}(1/d^2)\, G_l = \sum_r \frac{1}{d_r^2}\, g_r g_r^\top,
$$

逐行稀疏外积经 scatter map 累加（`conic_ipm_solver.cpp:374-388`），保持稀疏。

**SOC 块（稀疏 + 秩 1）。** 由 §4.2 的 $H^{-1} = \beta^{-2}\big(2(J\bar w)(J\bar w)^\top - J\big)$，

$$
B_q = G_q^\top H^{-1} G_q
    = \beta^{-2}\Big(2\, u u^\top - G_q^\top J G_q\Big), \qquad
u = G_q^\top (J \bar w).
$$

$u$ 只在该块触及的变量团（clique）上稠密；$G_q^\top J G_q$ 为稀疏 scatter，秩 1 项 $2\beta^{-2}uu^\top$ 按团内上三角位置直接写 CSC（`conic_ipm_solver.cpp:357-408`）。

**半定块（稠密团）。** 设 $X_j = \operatorname{smat}(G_s$ 的第 $j$ 列$)$，$M = Rti \cdot Rti^\top$。利用 svec 内积保持（§2.2），

$$
(B_s)_{ij} = \operatorname{svec}(X_i)^\top H^{-1} \operatorname{svec}(X_j)
           = \operatorname{svec}(X_i)^\top \operatorname{svec}(M X_j M)
           = \operatorname{tr}\big(X_i\, M\, X_j\, M\big).
$$

实现按列 $j$ 先算稠密合同 $P_j = M X_j M$，再与团内各 $i \ge j$ 列做点积（`conic_ipm_solver.cpp:419-449`）。同一锥块触及的所有变量构成一个稠密团——这是 §8 中 SDP 装配成本的来源。

> 实现锚点：`conic_ipm_solver.cpp:85-138`（`KktAssembler` 数据结构）、`conic_ipm_solver.cpp:341-467`（`assemble` / `apply_delta`）。

---

## 6. 主算法（Mehrotra 预测-校正）

整体遵循 CVXOPT `conelp` 的 Mehrotra 预测-校正框架；容差语义即 `ConicIPMOptions`（`conic_ipm_solver.hpp:24-33`）：`abstol = 1e-7`、`reltol = 1e-6`、`feastol = 1e-7`、`max_iterations = 100`、`refinement = 1`（仅当存在 q/s 块时启用）、`num_threads`、`verbose`。

### 6.1 初始点

$$
x = 0, \qquad y = 0, \qquad s = h, \qquad z = e \;(\text{Jordan 单位元}).
$$

若 $s = h$ 不严格内部，做投影式平移 $s \leftarrow s + (1 + \alpha) e$，其中 $\alpha$ 为触界最小平移量（逐锥：象限 $\max_i(-u_i)$；SOC $\max(0, \|u_1\| - u_0)$；半定 $\max(0, -\lambda_{\min}(U))$，实现 `cone_shift`，`conic_ipm_solver.cpp:609-623`）。$z = e$ 恒严格内部。

### 6.2 残差与终止准则

每迭代计算 $r_x, r_s, r_c$（§5.1）与归一化量

$$
\text{pres} = \max\Big(\frac{\|r_x\|_2}{1 + \|b\|_2},\, \frac{\|r_s\|_2}{1 + \|h\|_2}\Big), \qquad
\text{dres} = \frac{\|r_c\|_2}{1 + \|c\|_2}, \qquad
\text{gap} = s^\top z, \qquad
\text{relgap} = \begin{cases} \text{gap}/(-\text{pobj}), & \text{pobj} < 0, \\ \text{gap}/\text{dobj}, & \text{dobj} > 0, \\ +\infty, & \text{否则}. \end{cases}
$$

最优终止（`status = "optimal"`）：

$$
\text{pres} \le \text{feastol} \;\land\; \text{dres} \le \text{feastol} \;\land\;
\big(\text{gap} \le \text{abstol} \;\lor\; \text{relgap} \le \text{reltol}\big).
$$

**不可行证书。** 若 $h^\top z + b^\top y < 0$ 且归一化残差

$$
\frac{\|G^\top z + A^\top y\|_\infty}{-(h^\top z + b^\top y)} \le \text{feastol},
$$

则 $(y, z)$ 是原始不可行（primal infeasible）的近似 Farkas 证书；若 $c^\top x < 0$ 且

$$
\frac{\max\big(\|Ax\|_\infty,\, \|Gx + s\|_\infty\big)}{-c^\top x} \le \text{feastol},
$$

则 $x$ 给出对偶不可行（dual infeasible，原始无界）证书（`conic_ipm_solver.cpp:792-813`；测试 `tests/test_conic_ipm.cpp:435-465`）。

### 6.3 仿射（预测）方向与 $\sigma$

以 $r_\lambda = -\lambda$ 解 §5 的方向方程，得仿射方向 $(dx_a, dy_a, dz_a, ds_a)$。映射到缩放空间

$$
d\lambda_s = W^{-1} ds_a, \qquad d\lambda_z = W^\top dz_a,
$$

求不越出锥的最大步 $a_{aff} = \min(1, \max\text{\_step}(\lambda, d\lambda_s))$、$b_{aff} = \min(1, \max\text{\_step}(\lambda, d\lambda_z))$，以及仿射外推间隙

$$
\mu_{aff} = \frac{(s + a_{aff} ds_a)^\top (z + b_{aff} dz_a)}{\nu}, \qquad
\sigma = \operatorname{clamp}\Big(\big(\mu_{aff}/\mu\big)^3,\, 0,\, 1\Big).
$$

$\mu_{aff} \ll \mu$（仿射方向进展好）时 $\sigma \to 0$，方向偏纯下降；反之 $\sigma \to 1$ 加强中心化（`conic_ipm_solver.cpp:881-901`）。

### 6.4 组合（校正 + 中心化）方向

以 Mehrotra 二阶校正右端再解同一分解：

$$
r_\lambda = -\lambda - \lambda \,\backslash\, (d\lambda_s \circ d\lambda_z) + \sigma\mu\, \lambda^{-1},
$$

其中 $\lambda \,\backslash\, X := L(\lambda)^{-1} X$ 是 **Jordan 除法**（解 $\lambda \circ Y = X$ 的箭头阵系统），而 $\sigma\mu\,\lambda^{-1}$ 中的 $\lambda^{-1} = J\lambda/\det(\lambda)$ 是 Jordan 逆元。注意对非结合 Jordan 代数（SOC、半定）$L(\lambda)^{-1} \neq L(\lambda^{-1})$——把校正项写成 $(d\lambda_s \circ d\lambda_z) \circ \lambda^{-1}$ 是非结合误差量级的**错误**公式（CVXOPT 的 `sinv` 即 $L(\lambda)^{-1}$，见 `misc_solvers.c`）；正确实现下 $\lambda \circ r_\lambda = -\lambda \circ \lambda - d\lambda_s \circ d\lambda_z + \sigma\mu e$ 精确成立。

逐锥实现（`combined_lambda_rhs`，`conic_ipm_solver.cpp:625-684`）：

- 象限 / 尺寸-1 块：逐分量 $-\lambda_i - d\lambda_{s,i} d\lambda_{z,i}/\lambda_i + \sigma\mu/\lambda_i$（标量代数下两种除法一致）；
- SOC：$-\lambda - L(\lambda)^{-1}\!\big(d\lambda_s \circ d\lambda_z\big) + \sigma\mu\, \lambda^{-1}$，箭头阵 $\big[\begin{smallmatrix} \lambda_0 & \lambda_1^\top \\ \lambda_1 & \lambda_0 I \end{smallmatrix}\big]$ 解析求解（`q_jordan_solve`，`cones.cpp:214-229`）；
- 半定块：在矩阵空间取**对称积** $\text{prod} = \tfrac{1}{2}(dS\, dZ + dZ\, dS)$；由于 $\Lambda = \operatorname{diag}(\Sigma)$ 对角，Jordan 除法逐元素为 $(\Lambda \,\backslash\, X)_{ij} = 2 X_{ij} / (\sigma_i + \sigma_j)$（**不是** $\tfrac{1}{2}(\sigma_i^{-1} + \sigma_j^{-1})$），对角元再加 $-\sigma_i + \sigma\mu/\sigma_i$。

对组合方向重算 $d\lambda_s, d\lambda_z$ 与最大步，取阻尼步

$$
\alpha_p = \min\big(1,\; 0.99 \cdot \max\text{\_step}(\lambda, d\lambda_s)\big), \qquad
\alpha_d = \min\big(1,\; 0.99 \cdot \max\text{\_step}(\lambda, d\lambda_z)\big),
$$

更新 $(x, s) \mathrel{+}= \alpha_p (dx, ds)$、$(y, z) \mathrel{+}= \alpha_d (dy, dz)$。方向求解带可选的迭代精化（iterative refinement，对 3×3 KKT 残差做多至 `refinement` 步，`conic_ipm_solver.cpp:854-877`）。

### 6.5 各锥 max_step 公式

$\max\text{\_step}(u, du) = \sup\{\alpha \ge 0 : u + \alpha\, du \in \operatorname{cl}\mathcal{K}\}$，$u$ 严格内部；射线不出锥时返回 $+\infty$。

- **象限**（`l_max_step`，`cones.cpp:192`）：$\alpha = \min_{i:\, du_i < 0} (-u_i / du_i)$。
- **SOC**（`q_max_step`，`cones.cpp:202`）：边界条件 $\det(u + \alpha\, du) = a\alpha^2 + 2b\alpha + c = 0$，其中 $c = \det(u) > 0$、$a = \det(du)$、$b = u_0 du_0 - u_1^\top du_1$。分情形取最小正根：
  - $a \ne 0$ 且判别式 $b^2 - ac \ge 0$（涵盖 $a < 0$ 的开口向下抛物线与 $a > 0, b < 0$ 的开口向上情形）：取数值稳定形式 $\alpha = c/(-b + \sqrt{b^2 - ac})$（$b < 0$ 时）或 $\alpha = (-b - \sqrt{b^2 - ac})/a$（$b \ge 0$ 时），为正则采纳；
  - $a = 0,\ b < 0$（线性退化）：$\alpha = -c/(2b)$；
  - 首分量约束：$du_0 < 0$ 时再与 $-u_0/du_0$ 取小。
  
  与二分搜索的数值对照见 `tests/test_conic_ipm.cpp:279`。
- **半定锥**（`s_max_step`，`cones.cpp:231`）：$X = LL^\top$，构造 $E = L^{-1} dX\, L^{-T}$（对舍入对称化），则 $X + \alpha\, dX \succeq 0 \iff I + \alpha E \succeq 0$，故

$$
\alpha = \begin{cases} -1/\lambda_{\min}(E), & \lambda_{\min}(E) < 0, \\ +\infty, & \lambda_{\min}(E) \ge 0. \end{cases}
$$

全锥步长为各块步长的最小值（`cone_max_step`，`conic_ipm_solver.cpp:558-588`）。

> 实现锚点：`conic_ipm_solver.cpp:694-931`（主循环）、`cones.cpp:188-273`（max_step 与 shift）。

---

## 7. KKT 线性代数

### 7.1 Analyze once, factorize many

$K$ 的稀疏模式逐迭代**不变**：$G$、$A$ 固定，每迭代只有 $H^{-1}$ 的数值与 $\delta$ 变化。`KktAssembler` 因此遵循与 LP-IPM 相同的 analyze-once/factorize-many 契约（参见 `docs/numerical_methods.md` §4）：

- `build()`（每问题一次）：从 $G^\top$ 的行表、各锥块变量团与 $A$ 的列表生成 $K$ 下三角 CSC 模式，并为每类贡献预计算 **scatter map**（源条目 → CSC 值数组位置），包括 SOC 秩 1 项与 SDP Schur 项的位置表（`conic_ipm_solver.cpp:140-339`）；
- `assemble()`（每迭代）：`values` 清零后按缓存的条目表直接累加数值，零分配、无模式重建；
- `apply_delta()`：只改对角（$B$ 对角加 $\delta$、等式块置 $-\delta$），不动非对角——δ 升级时免重装配（`conic_ipm_solver.cpp:459-467`）。

### 7.2 SPD 快路径：CHOLMOD 超节点 LLᵀ

无等式约束（$m_{eq} = 0$）时 $K = G^\top H^{-1} G + \delta I$ 对称正定：$H^{-1} \succ 0$（三锥的 $H^{-1}$ 均为正定算子——象限正对角、SOC 的 $\beta^{-2}(2(J\bar w)(J\bar w)^\top - J)$ 在 $\det(\bar w) = 1$ 时正定、半定块的合同算子），加 $\delta I$ 保证数值正定。此时走 CHOLMOD 超节点 LLᵀ 快路径（`CholmodLDLT`，默认 auto 模式）：符号分解 analyze 一次，数值分解每迭代一次。分解失败（近奇异）时 δ 按 $\times 10$ 升级重试：$\delta_0 = 10^{-9} \to 10^{-8} \to 10^{-7} \to 10^{-6}$（`kDelta0` / `kDeltaMax`，`conic_ipm_solver.cpp:514-526`）。

### 7.3 准定快路径与不定回退：simplicial LDLᵀ / MUMPS

有等式行时 $K = \big[\begin{smallmatrix} B + \delta I & A^\top \\ A & -\delta I \end{smallmatrix}\big]$ 是**准定（quasidefinite）**鞍点系统：惯性恰含 $m_{eq}$ 个负特征值，但 Vanderbei 定理保证其对任意对称排列都存在无主元交换的 $LDL^\top$ 分解（负主元全部落在等式块）——这正是 IPM 鞍点系统的标准处理方式。此时 `KktBackend` 用 `CholmodLDLT::set_simplicial(true)` 强制 **CHOLMOD simplicial LDLᵀ**（`cholmod_ldlt.cpp:165-173`，`conic_ipm_solver.cpp:485-498`）：超节点 LLᵀ 只接受 SPD，而 simplicial LDLᵀ 原样接纳负 $D$ 主元。基准验证（`conic_benchmark --with-equalities`，$m_{eq} = n/10$ 随机等式）：惯性 $(n, m_{eq})$ 正确，解残差 $\|Kx - b\| \approx 10^{-9}$，SOCP/SDP 全用例 5–11 迭代收敛。

simplicial LDLᵀ 遇到（近）零主元仍可能失败：δ 升级到上限后**永久切换**到 MUMPS 对称不定 LDLᵀ（编译期宏 `MIPSOLVERS_HAVE_MUMPS` 存在时）：`analyze_pattern` 一次，`factorize` 每迭代。LDLᵀ 是合同变换，由 Sylvester 惯性定律，$D$ 的负主元数即 $K$ 的负特征值数（MUMPS `INFOG(12)`，`MumpsSolver::negative_eigenvalues()`）——惯性是免费的，为后续惯性控制（如 Wächter–Biegler 风格的正则化）预留了接口。MUMPS 不可用（或链接 ABI 不匹配，见 §11）时回退失败，求解以 `status = "unknown"` 退出。

> 实现锚点：`conic_ipm_solver.cpp:469-542`（`KktBackend`）、`include/mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp`。

---

## 8. 每迭代复杂度

记 $n$ 为变量数，$m_{eq}$ 为等式数；对第 $b$ 个半定块，$p_b$ 为阶、$n_b$ 为其变量团大小、$\mathrm{nnz}_b$ 为 $G_s$ 在该块团内的非零数。

| 阶段 | 代价 | 备注 |
|---|---|---|
| NT 缩放 | $l$ 块 $O(l)$；SOC 每块 $O(k)$；半定每块 2 次 Cholesky + 1 次 SVD $O(p_b^3)$ | q/s 块可并行（§9） |
| $B$ 装配：标量行 | $O\big(\sum_j \sum_{r \ni j} \mathrm{nnz}(g_r)\big)$ | 稀疏 scatter |
| $B$ 装配：SOC 块 | 稀疏部分同量级 + 秩 1 $O(|{\rm clique}_b|^2)$ | 每块 |
| $B$ 装配：半定块 | $O\big(n_b p_b^3 + n_b^2 \cdot \overline{\mathrm{nnz}}_b\big)$ | 每列稠密合同 $M X_j M$ 为 $O(p_b^3)$；成对点积按团内列对 |
| KKT 分解 | CHOLMOD 超节点 LLᵀ（SPD）或 MUMPS LDLᵀ（不定），问题相关 | 符号分解摊还，数值分解每迭代 |
| max_step | SOC $O(k)$；半定每块 Cholesky + 特征分解 $O(p_b^3)$ | 每迭代调用 4 次（仿射/组合 × s/z） |
| 迭代精化 | 每次 1 回代 + 数次 SpMV | 仅 q/s 块存在时 |

**说明。** 半定块的 Schur 补对变量团是**稠密**的（同一 $\mathcal{S}^p$ 块触及的变量两两耦合），$O(n_b^2)$ 个元素、每元素来自 $O(p_b^3)$ 合同——这是 NT 框架下 SDP 的固有成本，CVXOPT 的 `sdp` 求解器同级；它决定了单个大阶半定块主导全部迭代时间。可行的缓解（chordal decomposition 等）见 §11。

---

## 9. 并行设计

并行全部基于 OpenMP，编译期由 `MIPSOLVERS_USE_OPENMP` 宏守卫（未启用时为空宏，完全串行）；运行期线程数由 `ConicIPMOptions::num_threads` 经 RAII guard（`OmpThreadsGuard`）设置并在求解结束恢复（`conic_ipm_solver.cpp:62-79`）。三处并行区：

1. **锥块级并行——NT 缩放**（`cones.cpp:299-307`）：$q$/$s$ 块合并为一个 `parallel for`，每块的缩放计算（SOC 的 $\bar w, v, \beta$；半定的 Cholesky/SVD/合同因子）相互独立、写入各自的输出槽。工作量门控：块数 $> 1$ 且立方标度工作量估计 $\sum q_i + \sum p_i^3 > 4096$ 才并行。
2. **锥块级并行——max_step**（`conic_ipm_solver.cpp:558-588`）：同一门控，各块步长写入独立数组后取最小值；障碍内部性检查与 shift 同理逐块独立。
3. **半定块 Schur 按列并行**（`conic_ipm_solver.cpp:419-449`）：对团内列 $j$ 并行计算 $P_j = M X_j M$ 并做点积；每列只写自己的 CSC 位置段（`schur_pos[j]`），无共享写。

稠密核（合同乘、特征分解、SVD）由 Eigen/BLAS 提供。

**确定性保证。** 所有并行循环都满足「每次迭代写互不相交的输出」，归约只发生在并行区之后的串行取 $\min$；因此**线程数不改变数值结果**。该性质被测试锁定：混合锥问题在 `num_threads = 1` 与 `4` 下目标值一致（容差 $10^{-10}$，`tests/test_conic_ipm.cpp:479-491`）。

---

## 10. 使用示例

### 10.1 Engine 级：手工组装 `ConicModel` 解 SOCP

求解 $\min t \;\; \text{s.t.} \;\; \|x\|_2 \le t,\ 3x_1 + 4x_2 = 1$（解析解 $t^* = 1/5$，$x^* = (0.12, 0.16)$；取自 `tests/test_conic_ipm.cpp:375`，`make_sparse` 为该测试文件中的三元组组装辅助函数）：

```cpp
#include "mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp"
using namespace mipsolvers::engine;

ConicModel cm;
cm.c.resize(3);
cm.c << 1.0, 0.0, 0.0;                    // min t
// q 块 (t, x1, x2)：s = h - Gx = (t, x1, x2) ∈ Q^3，取 G = -I, h = 0
cm.G = make_sparse(3, 3, {{0,0,-1.0},{1,1,-1.0},{2,2,-1.0}});
cm.h = Eigen::VectorXd::Zero(3);
cm.A = make_sparse(1, 3, {{0,1,3.0},{0,2,4.0}});   // 3 x1 + 4 x2 = 1
cm.b.resize(1);
cm.b << 1.0;
cm.dims.q = {3};                          // 一个 Q^3 块

ConicIPMOptions opt;                      // abstol=1e-7, reltol=1e-6, feastol=1e-7
const ConicIPMResult res = ConicIPMSolver(opt).solve(cm);
// res.status == "optimal", res.primal_objective ≈ 0.2, res.x ≈ (0.2, 0.12, 0.16)
```

锥行编码规则：$s = h - Gx$ 必须逐行等于目标仿射表达式 $e(x) = c_0 + \sum_j c_j x_j$，故 $h_i = c_0$、$G_{ij} = -c_j$。也可经 `SolverEngine::solve_conic(cm)` 走适配器（`solver_name == "NativeConicIPM"`），此时对偶在 `Result::constraint_duals` 中按 `[z（m 个锥行）| y（m_eq 个等式）]` 布局（`conic_ipm_adapter.cpp:53-66`）。

### 10.2 AML 级：`add_soc_constraint` / `add_rotated_soc_constraint` / `add_psd_constraint`

**标准 SOC**：$\|x - a\|_2 \le t$（`tests/test_aml_conic.cpp:61`）：

```cpp
Model m("soc_projection");
auto& S = m.add_set("S", {"i0"});
auto& I = m.add_set("I", {"i1", "i2"});
auto& tv = m.add_var("t", S, VarType::Continuous);
auto& xv = m.add_var("x", I, VarType::Continuous);
m.add_soc_constraint(tv("i0"), {xv("i1") - 3.0, xv("i2") + 4.0}, "shifted_norm");
m.minimize(tv("i0"));
const SolveResult r = m.solve();   // t* = 0, x* = (3, -4)
```

**旋转锥与 Markowitz 换算**：$\tfrac{1}{2} x^\top \Sigma x \le \theta$ 的图中表示（epigraph）。分解 $\Sigma = L L^\top$（Cholesky），令 $u = L^\top x$，则

$$
\tfrac{1}{2} x^\top \Sigma x \le \theta
\iff \|u\|_2^2 \le 2\theta \cdot 1
\iff (\theta,\ 1,\ u) \in \mathcal{Q}_r \;(\text{旋转锥})
\iff \big(\theta + 1,\ \theta - 1,\ \sqrt{2}\, u\big) \in \mathcal{Q}^{k+2},
$$

最后一步是 AML 在 `compile_conic` 中自动完成的标准化（`src/aml/model.cpp:452-464`）。用户只需写（`tests/test_aml_conic.cpp:132`）：

```cpp
std::vector<LinearExpr> u(3);
for (int k = 0; k < 3; ++k)
  for (int i = 0; i < 3; ++i)
    if (kL[i][k] != 0.0)
      u[k] += LinearExpr::from_var(xs(kAssets[i]).id(), kL[i][k]);  // u = L'x
ms.add_rotated_soc_constraint(ths("t0"), LinearExpr::const_expr(1.0), u, "risk");
// 目标：min theta - rho * mu'x —— 与 QP 路径同解（测试交叉验证 1e-5）
```

**PSD**：$F(x) = \begin{bmatrix} X_{11} & 2 \\ 2 & 1 \end{bmatrix} \succeq 0$（下三角逐行给出，`tests/test_aml_conic.cpp:193`）：

```cpp
m.add_psd_constraint(2, {{X11},
                         {LinearExpr::const_expr(2.0),
                          LinearExpr::const_expr(1.0)}}, "psd2");
m.minimize(X11);   // X11* = 4
```

**锥对偶读取**（`include/mipsolvers/aml/solve_result.hpp:96-105`）：`SolveResult::conic_dual_vals` 按 `add_*` 返回的 `ConId` 索引，每项布局为——标准 SOC（$k$ 个 xs）：$k+1$ 个对偶，锥头在前；旋转 SOC：编译后标准块 $(a+b, a-b, \sqrt{2}x)$ 的 $k+2$ 个对偶；PSD（阶 $p$）：$p(p+1)/2$ 个 svec 打包对偶（列主序下三角、非对角乘 $\sqrt{2}$）。例如 $p = 2$ 时 $(v_0, v_1, v_2) \mapsto Z = \begin{bmatrix} v_0 & v_1/\sqrt{2} \\ v_1/\sqrt{2} & v_2 \end{bmatrix} \succeq 0$（布局与成员性测试 `tests/test_aml_conic.cpp:249-269`）。锥对偶**不**混入线性行对偶表 `dual_vals`。

---

## 11. 已知限制与后续方向

- **仅线性目标（v1）**。内核标准型目标是 $c^\top x$；AML 侧二次目标 + 锥约束、整数变量 + 锥约束、非线性 + 锥约束均在 `compile_conic` 显式拒绝（`std::invalid_argument`，`src/aml/model.cpp:381-400`）。凸 QP 目标请改走 SOC 图中表示（§10.2 的旋转锥换算）或不带锥的 QP 路径。
- **无指数锥 / 幂锥**：三锥以外的锥需要新的自尺度障碍与 NT 缩放推导，非局部改动。
- **无 chordal decomposition**：单个大阶半定块的稠密 Schur（§8）目前无解耦手段；聚合型 SDP（如电网 SDP 松弛的团树结构）是下一步的主要性能项。
- **外部锥求解器未接线**：AML 能力标志 `supports_socp` / `supports_sdp` 已定义（`include/mipsolvers/aml/solver_capabilities.hpp:16-17`），但尚无适配器（Clarabel、SCS 等）声明它们；`NativeConicIPM` 是当前唯一的 CONIC 后端。
- **Python 绑定未暴露**：锥路径目前只有 C++ API。
- **Homebrew MUMPS 与内置头文件版本错位（构建层已知问题）**：`cmake/BuildMUMPS.cmake` 的 brew 路径（`MIPSOLVERS_FORCE_BUILD_MUMPS=OFF`）用**内置 5.7.3 头文件**链接 **Homebrew Ipopt 自带的 libdmumps 5.6.2**，`DMUMPS_STRUC_C` 布局跨版本不一致，导致 `MumpsSolver` 的 solve 返回错误解（`INFOG(12)` 惯性读取也错位）——不只是锥路径，任何走 MUMPS 回退的模块都受影响。规避：以 `-DMIPSOLVERS_FORCE_BUILD_MUMPS=ON` 重新配置（内置 5.7.3 源码构建，同矩阵实测解残差 $8\times10^{-13}$）；锥 KKT 自 §7.3 起默认走 simplicial LDLᵀ，正常情形不再触碰 MUMPS。
- **超稀疏 $G$ 的 SDP 病态**：$G$ 列均仅 ~1 非零时 Schur 补 $G^\top H^{-1} G$ 条件数极差，$p = 10$ 个例观测到 gap 在 $\sim 2\times 10^{-2}$ 处停滞、步长顶到锥边界后发散（`conic_benchmark` 因此对 SDP 块设了 ≥5 非零/列的构造下限）。未做算法层修复；候选缓解（未实现）：最小步长下限或中心化参数下限。

---

## 参考

- CVXOPT User's Guide, *Cone Programming*（`conelp`/`sdp` 算法描述与本实现的标准型、$\sigma$ 规则、终止准则）：https://cvxopt.org/userguide/coneprog.html
- L. Vandenberghe, *The CVXOPT linear and quadratic cone program solvers*, 2010（NT 缩放与 Schur 补装配的推导蓝本）。
- Yu. Nesterov, M. J. Todd, *Self-scaled barriers and interior-point methods for convex programming*, Mathematics of Operations Research 22(1), 1997（自尺度障碍与 NT 缩放点的原始理论）。
