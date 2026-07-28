# LP 求解栈梳理、一阶方法（PDLP/PDHG）文献综述、理论推导与改进路线

**日期：** 2026-07-28
**范围：** 本仓库 LP 求解能力全景 + 大规模 LP 一阶方法（FOM）文献 + 收敛性理论推导 + 分阶段改进建议
**方法：** 代码静态梳理（`src/`、`include/`、`cmake/`、`benchmark/`）+ 文献检索（arXiv / NeurIPS / Math. Program. / SIAM 等）

---

## TL;DR（结论先行）

1. 本仓库已有**三层 LP 求解栈**：原生对偶单纯形、原生 Mehrotra IPM（CHOLMOD）、原生 PDLP（PDHG 一阶），外嵌 HiGHS（simplex/IPX）与 SCIP（经 `lpi_highs`）。**PDLP 已有可用雏形，且 HiGHS 内嵌的 cuPDLP-C 已被编译进库但从未被调用**——这是零成本的对比基线。
2. 文献上，PDLP（restarted PDHG + 一组工程增强）已是成熟方向；**2024 年以后的最前沿是 Restarted Halpern PDHG（rHPDHG）与 HPR-LP**，GPU 实现（cuPDLP-C/x、NVIDIA cuOpt）在超大规模 LP 上已与商业求解器同档。理论上：遍历 $O(1/K)$ → sharpness 条件下 restart 后**线性收敛**，Halpern 锚定给出最优的 $O(1/K)$ 不动点残差。
3. 对本框架的建议：**不是**用 PDLP 替换单纯形/IPM，而是补第三根支柱——负责「分解法跑不动」的超大规模 LP（巨型 MILP 根节点、SCUC 市场出清 LP）。改进分三阶段：Phase 0 工程补齐（测试、停机判据、不可行证书、接入 B&C 根节点与基准）、Phase 1 算法升级（Halpern restart、linesearch、feasibility polish、多线程 SpMV）、Phase 2 平台化（GPU、crossover 衔接单纯形、L2O 调参）。

---

## 1. 本项目 LP 求解栈现状

### 1.1 全景与调用关系

```
应用层    SCUC(机组组合/市场出清LP+LMP对偶; MILP)   AML建模   L2O(模型指纹→求解配置策略; 不碰数值内核)
            │  SolverEngine::solve_lp / solve_milp
引擎层    StrategyDispatcher（按 ProblemClass 分发，支持 preferred_solver / 顺序回退）
            LP 默认优先级: NativeIPMLP → NativePDLP → NativeLCQP → HiGHS   (dispatcher.cpp:72-73)
            MILP 优先级:   StrictHiGHS → HiGHS → NativeBranchAndCut
─────────────────────────────────────────────────────────────────────────────
原生内核    对偶单纯形 (vendored HFactor / UMFPACK LU + eta 更新; B&C 节点内核、LP 快路径)
            NativeIPMLP (Mehrotra 预估-校正; 稀疏正规方程 CHOLMOD → Accelerate → Eigen)
            NativePDLP  (PDHG 一阶; CSC/CSR SpMV; presets: root_lp / milp_node / high_precision)
            NativeLCQP / ConicIPM (CHOLMOD; KKT 不定时回退 MUMPS)
外部库      HiGHS 1.14 内嵌 (simplex / IPX+crossover / ★cuPDLP-C 已编译(CUPDLP_CPU=ON)但从未启用)
            SCIP 9.0 内嵌 (MILP/MINLP; LP 松弛经 lpi_highs → 内嵌 HiGHS)
            Ipopt (仅 NLP; MUMPS 线性代数)      Gurobi (默认 OFF)
预处理      PaPILO (仅 MILP presolve; strategy 层 PresolveManager 仍是 stub)
```

关键事实（详细出处见各文件）：

- **HiGHS 的 PDLP 已编译、零暴露**：`cmake/BuildHiGHS.cmake:40` 硬编码 `CUPDLP_CPU ON`，即 cuPDLP-C 在库里；但全项目没有任何代码设置 `solver="pdlp"`（仅在 `adapters.cpp:558` 读 `pdlp_iteration_count` 统计），`BCOptions::highs_mip_lp_solver` 只允许 `simplex/ipm/choose`。
- **原生 B&C 的 LP 松弛内核**默认走 HiGHS（`BCOptions::lp_kernel_backend`），可选 `ExperimentalNative`；StrictHiGHS 在「大模型根节点」自动切 IPM+crossover（`native_adapters.cpp:422-462`）——这正是 PDLP 的典型插入点（见 §4）。
- **SCUC 出清 LP 需要约束对偶做 LMP 定价**，求解器偏好 `{"NativeBranchAndCut","NativeIPMLP"}`（`src/scuc/scuc.cpp:2440-2444`）。PDLP 天然返回对偶，具备进入该偏好列表的资格。
- **L2O 不解 LP**，只做「按指纹推荐配置/暖启动」，是将来调 PDLP 超参的天然挂载点。

### 1.2 原生 NativePDLP 现状（对照文献）

已实现（`include/mipsolvers/engine/solver/native/lp/pdlp_solver.hpp:9-61`）：Ruiz 均衡（早停）、Pock–Chambolle 对角预条件 + 原始权重平衡、自适应重启（归一化对偶间隙）、遍历平均停机、IP-PMM 步长正则化（Pougkakiotis–Gondzio）、裸 CSC/CSR SpMV、三档预设。**骨架与 PDLP 论文（§2.3）同代**。

已知差距（`docs/code_quality_evaluation.md:118-120` + 本文复核）：

| 差距 | 文献对应方案 | 出处 |
|---|---|---|
| 无单元测试 | — | `code_quality_evaluation.md:115` |
| 停机判据混用绝对 ∞-范数与相对间隙；inf gap 视为收敛 | PDLP 期刊版 §4 的相对 KKT 残差定义 | `pdlp_solver.cpp:455-456, :593` |
| 无不可行证书 | Applegate et al. 发散射线理论（§3.7） | [7] |
| 无 Malitsky–Pock linesearch | [3]（PDLP 消融显示自适应步长已接近其效果，[4] Table 4） | [3][4] |
| 未接入 B&C 根节点 / 基准 | `root_solve.cpp:5` 注释预留；`milp_benchmark_runner` 有根节点消融框架 | — |
| CSC+CSR 双驻留内存 ~2.4 GB @1e8 nnz | cuPDLP 单驻留 + 转置 SpMV | `pdlp_solver.cpp:252-269` |
| 遍历平均 restart（上一代） | **rHPDHG / HPR-LP（2024+ 前沿）** | [11][18] |

---

## 2. 文献综述

### 2.1 为什么需要一阶方法：三类 LP 算法的定位

| | 单纯形 | IPM（内点） | 一阶（PDHG 系） |
|---|---|---|---|
| 每迭代成本 | ~$O(\mathrm{nnz})$ 级（基分解更新） | 一次稀疏分解 $O(\mathrm{nnz}^{\sim 1.5+})$，内存大头 | **两次 SpMV，$O(\mathrm{nnz})$** |
| 迭代数 | 实践近线性，最坏指数 | $O(\sqrt{n}\log\frac1\varepsilon)$，实践 ~30–80 | 遍历 $O(1/\varepsilon)$；restart+sharpness 下 $O(\frac{1}{\eta\alpha}\log\frac1\varepsilon)$ |
| 输出 | 顶点解 + 基证书 | 内点（crossover 得基） | 内点（无基），天然含原始+对偶 |
| 暖启动 | **极好**（B&C 节点唯一现实选择） | 差 | 中等（重启点） |
| 精度 | 机器精度 | $10^{-8}$ 常规 | $10^{-4}\!\sim\!10^{-8}$，病态问题吃力 |
| 适用 | 中小规模、B&C 树内 | 中大、病态、高精度 | **超大（nnz≥1e7–1e9）、分解内存不可行、GPU** |

一阶方法复兴的直接动因：Google 内部实例（网络流量、供应链、营销组合）达到 $10^8$–$6.3\times10^9$ 非零元，IPM 分解内存爆掉、单纯形迭代次数爆掉 [5][39]；GPU 的 SpMV 带宽红利无法被分解类算法利用 [8][29]。

### 2.2 PDHG 基础

- **起源**：Esser–Zhang–Chan [41] 提出 primal-dual hybrid gradient；Chambolle–Pock [1] 系统化为凸问题一阶原始-对偶算法并证明遍历 $O(1/K)$（强凸情形下 $O(1/K^2)$/线性）；Pock–Chambolle [2] 给出**对角预条件**（行/列范数定步长），是后来 PDLP 实用化的关键组件。
- **Malitsky–Pock linesearch** [3]：免谱范数估计的自适应步长回溯。PDLP 消融（[4] Table 4）显示 PDLP 自己的自适应步长规则略优于全局调参的 MP linesearch——**这不是当前最高优先级**，但可作为病态实例的保险。
- **ADMM/DRS 等价**：PDHG 等价于对偶问题上的（预条件）Douglas–Rachford 分裂，也与线性化 ADMM 互为镜像——这是 §2.5 HPR-LP 统一关系的理论入口 [19]。

### 2.3 PDLP 主线（Google）

- **会议版** [4]（NeurIPS 2021，Beale–Orchard–Hays 奖）：PDHG + 五项增强 = PDLP：**预处理（presolve）、Ruiz 均衡、Pock–Chambolle 对角预条件 + primal weight 自适应、自适应步长、基于归一化对偶间隙的自适应 restart**。在 383 个 MIPLIB 2017 松弛上，$10^{-6}$/$10^{-8}$ 精度全面超过 SCS [22]。
- **期刊版** [5]（2025）：新增 **feasibility polishing**（收尾阶段投影提纯）；C++ 实现开源在 OR-Tools 并多线程化；提出 11 个 $1.25\times10^8$–$6.3\times10^9$ nnz 的超大规模测试集，8/11 在 6 天内解到 1% 间隙（原始/对偶可行误差 $<10^{-8}$）。
- **Google 官方博客** [39] 即用户提到的链接：通俗版总结（OR-Tools 可用、相对 KKT 残差停机、restart 机制）。

### 2.4 收敛理论线（本项目理论推导 §3 的出处）

1. **Restart + sharpness ⇒ 线性收敛**：Applegate–Hinder–Lu–Lubin [6] 证明 LP 的归一化对偶间隙满足 sharpness（常数 $\alpha$ 与 Hoffman 常数 [26] 相关），自适应 restart 的 PDHG 达 $\tilde O(1/(\eta\alpha))$ 线性收敛；Hinder [38] 给出唯一最优解下更易算的复杂度界。
2. **几何与精细速率**：Lu–Yang [12] 证明 vanilla PDHG 的**两阶段行为**（先次线性「识别」后局部线性），[13] 用 infimal sub-differential size 刻画局部度量次正则性。
3. **Error ratio 体系**：Xiong–Freund [14][15] 用 limiting error ratio 替代 Hoffman 常数，给出更紧且与 sharpness 等价的保证；[16] 提出**中心路径 Hessian 重缩放**显著提升 PDHG 收敛；[17] 给出 restart PDHG 的高概率多项式时间复杂度。
4. **不可行检测**：Applegate–Díaz–Lu–Lubin [7] 证明不可行时迭代沿 minimal displacement vector 发散，其极限给出 Farkas 原始/对偶不可行证书（§3.7）。
5. **在线预条件**：[31] 在求解过程中动态更新预条件矩阵，进一步减少迭代。

### 2.5 Halpern 加速与算法关系统一（2024+ 前沿）

- **Halpern 锚定**：对非扩张算子 $T$，Halpern 迭代 [25] $z^{k+1}=\frac{k+1}{k+2}Tz^k+\frac{1}{k+2}z^0$ 的不动点残差 $\|z^K-Tz^K\|=O(1/K)$（Lieder [24]，常数 $2\|z^0-z^*\|$；Kim [23] 的锚定 PPM 同阶），优于 KM 迭代的 $O(1/\sqrt K)$。
- **rHPDHG**（Lu–Yang [11]）：把 PDLP 的「平均点 restart」换成「Halpern restart」，理论常数更优、实践全面提速，并解释了 PDHG 的螺旋动态。
- **HPR-LP**（Chen–Sun–Yuan–Zhang–Zhao [18]，SIAM J. Optim. 2025）：Halpern 加速的 semi-proximal **Peaceman–Rachford**，KKT 残差 $O(1/K)$，GPU(Julia) 上比 PDLP 快 **2.4–5.7×**（SGM10，$10^{-8}$）。
- **关系统一** [19]：**cuPDLPx（见下）的基础算法是 HPR-LP 基础算法的特例**；确定活跃集后 HPR-LP 与 EPR-LP 等价。含义：新一代实现应以 HPR/锚定 PR 为基座，而非经典 PDHG 平均。
- **Anderson 加速 PDHG** [35]：另一条加速支线（2025）。

### 2.6 GPU 与产业化

| 实现 | 状态 | 备注 |
|---|---|---|
| cuPDLP.jl [8]（Oper. Res. 2025） | 开源 Julia | 首次证明「GPU 解 LP 有用」，标准基准上与 Gurobi 同档 |
| cuPDLP-C [9] | **已并入 HiGHS（本仓库已编译未启用）** | C 强化实现 |
| cuPDLPx [10]（2025） | 开源 | Halpern 化 + 增强，当前 GPU LP 标杆之一 |
| **NVIDIA cuOpt** [29] | 开源（CUDA） | PDLP + GPU barrier(cuDSS) + CPU 对偶单纯形 **三路并发**；Hopper 上号称最高数千倍加速（特定实例）；社区报告精度/收敛仍偶有短板 |
| COPT / FICO Xpress / HiGHS | 商业/开源 | 均已内置 (cu)PDLP 变体 [5] |
| D-PDLP [33] | 2026 | 多 GPU 分布式 PDLP |
| PDCS [32] / PDHCG | 2025 | 锥规划/市场均衡的 GPU 一阶推广 |

### 2.7 其他一阶/混合路线（对照与备选）

- **SCS** [22]：齐次自对偶嵌入 + ADMM（共轭梯度解线性系统），锥规划通用；PDLP 论文的 baseline，LP 上已被超越。
- **ABIP** [20][21]：ADMM 近似求解 IPM 屏障子问题（一阶-二阶混合），自带不可行证书；LEAVES/COPT 采用。
- **ECLIPSE** [27]：Web 级 LP（$10^{12}$ 变量）的加速梯度法，说明分布式/投影友好的另一条路。
- **Hybridizing PDHG–IPM** [34]（2026）：PDHG 预热 + IPM 收尾的正式混合框架，与本文 §4.4「PDLP→crossover/单纯形衔接」思路一致。
- **QP 推广**：Lu–Yang [37] 把 restarted PDHG 推广到凸 QP（最优复杂度）；对 NativeLCQP 有参考价值。

### 2.8 交叉（crossover）与下游衔接

- **PDHG→基**：[30] 利用 PDHG 螺旋动态做 crossover，从 PDLP 内点高效恢复最优基——**这是 PDLP 接入 B&C 树（需要暖启动基）和精确 LMP 定价的关键补丁**。

---

## 3. 理论推导

记 LP 标准形（推导用等式形，不等式/盒约束同理）：

$$\min_{x\ge 0}\ c^\top x \quad \text{s.t.}\ Ax=b,\qquad A\in\mathbb R^{m\times n}.$$

### 3.1 鞍点形式与单调算子

拉格朗日函数 $\mathcal L(x,y)=c^\top x+y^\top(b-Ax)$，LP $\iff$ 鞍点问题 $\min_{x\ge0}\max_{y}\mathcal L(x,y)$。写 $z=(x,y)$，KKT 算子

$$F(z)=\begin{pmatrix}c-A^\top y\\ b-Ax\end{pmatrix}=\underbrace{\begin{pmatrix}0&-A^\top\\ A&0\end{pmatrix}}_{M\ (\text{反对称})}z+\begin{pmatrix}c\\ b\end{pmatrix}.$$

$M$ 反对称 $\Rightarrow \langle z-z',F(z)-F(z')\rangle=0$，即 $F$ **单调**；求鞍点 $=$ 求单调包含 $0\in F(z)+N_Z(z)$ 的零点（$N_Z$ 为法锥，编码 $x\ge0$ 等约束）。

### 3.2 PDHG = 预条件不动点迭代；步长条件

取原始权重 $\omega>0$、步长 $\eta>0$（$\tau=\eta/\omega,\ \sigma=\eta\omega$），PDHG 迭代：

$$x^{k+1}=\Pi_{X}\big(x^k-\tau(c-A^\top y^k)\big),\qquad y^{k+1}=y^k+\sigma\big(b-A(2x^{k+1}-x^k)\big).$$

它是 $z^{k+1}=T(z^k)$ 的不动点迭代，$T$ 可写成「前向-后向」分裂：$z^{k+1}=(P+\tilde F)^{-1}(P z^k-\text{外推项})$，$P=\mathrm{diag}(\frac1\tau I,\frac1\sigma I)$。

**收敛条件** $\tau\sigma\|A\|^2=\eta^2\|A\|^2<1$：此时度量矩阵

$$Q=\begin{pmatrix}\frac1\tau I & -A^\top\\ -A & \frac1\sigma I\end{pmatrix}\succ 0,$$

$T$ 在 $Q$-范数下**非扩张**（Fejér 单调：$\|z^{k+1}-z^*\|_Q\le\|z^k-z^*\|_Q$），KM 迭代收敛到不动点集 $Z^*$（最优点集）。Pock–Chambolle 预条件 [2] 把 $\tau,\sigma$ 换成对角阵 $T=\mathrm{diag}(\tau_j),\Sigma=\mathrm{diag}(\sigma_i)$，$\tau_j=\eta/(\omega\sum_i|A_{ij}|)$、$\sigma_i=\eta\omega/\sum_j|A_{ij}|$，使 $\|T^{1/2}A\Sigma^{1/2}\|\le1$ 自动近似成立——**这就是 PDLP 能用近单位步长的原因**，也是本仓库 `pdlp_solver.cpp` 现有实现。

### 3.3 遍历 $O(1/K)$（Chambolle–Pock）推导纲要

**单步不等式**（下降引理）：由 $x^{k+1}$ 的投影最优性条件与 $y^{k+1}$ 的显式更新，对任意 $z=(x,y)\in Z$，

$$\mathcal L(x^{k+1},y)-\mathcal L(x,y^{k+1})\ \le\ \tfrac{1}{2\eta}\big(\|z-z^k\|_\omega^2-\|z-z^{k+1}\|_\omega^2\big)-\tfrac{1}{2\eta}\Delta_k,$$

其中 $\|z\|_\omega^2=\frac1\omega\|x\|^2+\omega\|y\|^2$，$\Delta_k=\|z^{k+1}-z^k\|_\omega^2-\|z^k-z^{k-1}\|_\omega^2+2\langle z^{k+1}-z^k,\,H(z^k-z^{k-1})\rangle$，$H=\big(\begin{smallmatrix}0&\eta A^\top\\-\eta A&0\end{smallmatrix}\big)$。

步长条件保证 $\|\cdot\|_\omega^2\pm H$ 交叉项构成范数，故 $\Delta_k$ **伸缩相消**（telescope）后余项非负。对 $k=0..K-1$ 求和，用 $\mathcal L$ 对 $x$ 凸、对 $y$ 凹（Jensen）取遍历平均 $\bar z^K=\frac1K\sum_k z^{k+1}$：

$$\boxed{\ \mathcal L(\bar x^K,y)-\mathcal L(x,\bar y^K)\ \le\ \frac{1}{2\eta K}\big(\|z-z^0\|_\omega^2-\|z-z^K\|_\omega^2\big)\quad \forall z\in Z.\ }$$

右端对 $z=z^*$ 给出对偶间隙型残差 $O(1/K)$。**注意两点**：(i) 速率遍历（平均点）而非末迭代；(ii) 界依赖 $\|z^0-z^*\|$——这两点分别解释了 PDLP 为何在平均点上停机、以及为什么 restart「换锚点」能加速。

### 3.4 归一化对偶间隙与自适应 restart

**定义**（[4][6]）：对 $r>0$，$W_r(z)=\{\hat z:\|\hat z-z\|\le r\}$，

$$\rho_r(z)\ :=\ \frac{1}{r}\sup_{\hat z\in W_r(z)}\big[\mathcal L(x,\hat y)-\mathcal L(\hat x,y)\big].$$

性质：(i) 上确界是线性函数在球上的最大值，**有闭式解、可 $O(\mathrm{nnz})$ 计算**；(ii) $\rho_r(z)\ge0$，且（$r>0$ 时）$\rho_r(z)=0\iff z\in Z^*$；(iii) $\rho_r$ 控制缩放后的 KKT 残差（原始可行、对偶可行、互补）。由 §3.3 的界直接得到

$$\rho_r(\bar z^K)\ \le\ \frac{(r+\|\bar z^K-z^0\|)^2}{2\eta K\,r}\ =\ O\!\Big(\frac{r}{\eta K}\Big)\quad(\text{取 } r\ge\|\bar z^K-z^0\|).$$

即：**epoch 内 $K$ 次迭代把归一化间隙按 $1/K$ 压低**。PDLP 的 restart 规则：当 $\rho(\bar z^{k})\le\beta\,\rho(z^{n,0})$（$\beta\in(0,1)$，另与末迭代比较取优）时，以当前平均（或末迭代）为新锚点 $z^{n+1,0}$。

### 3.5 Sharpness ⇒ 线性收敛：逐 epoch 收缩推导

**定义（sharpness，[6]）**：存在 $\alpha>0$ 使

$$\alpha\,\mathrm{dist}(z,Z^*)\ \le\ \rho_{\mathrm{dist}(z,Z^*)}(z)\qquad\forall z\notin Z^*.$$

LP 必满足（$\alpha$ 与 KKT 系统的 Hoffman 常数 [26] 同阶；Xiong–Freund [14][15] 给出与 limiting error ratio 的等价刻画）。

**收缩论证**（简化自 [6] §4）：设 epoch 锚点 $z^0$，$z^*=\Pi_{Z^*}(z^0)$，取 $r:=2\|z^0-z^*\|$。

1. Fejér 单调（§3.2）$\Rightarrow\|\bar z^K-z^0\|\le \|\bar z^K-z^*\|+\|z^*-z^0\|\le 2\|z^0-z^*\|=r$。
2. 遍历界（§3.4）：对 $\hat z\in W_r(\bar z^K)$ 有 $\|\hat z-z^0\|\le 2r$，故
   $\rho_r(\bar z^K)\le\frac{(2r)^2}{2\eta K r}=\dfrac{2r}{\eta K}=\dfrac{4\|z^0-z^*\|}{\eta K}$。
3. 又 $\mathrm{dist}(\bar z^K,Z^*)\le\|\bar z^K-z^*\|\le\|z^0-z^*\|=r/2\le r$，由 $\rho_\cdot$ 对半径的单调性与 sharpness：
   $\alpha\,\mathrm{dist}(\bar z^K,Z^*)\le\rho_{\mathrm{dist}}(\bar z^K)\le\rho_r(\bar z^K)\le\dfrac{4\|z^0-z^*\|}{\eta K}$。
4. 取 epoch 长度 $K\ge \dfrac{8}{\eta\alpha}$，得 $\mathrm{dist}(\bar z^K,Z^*)\le\tfrac12\,\mathrm{dist}(z^0,Z^*)$。

**结论**：每个 restart epoch 距离减半 $\Rightarrow$ $\mathrm{dist}(z^{n,0},Z^*)\le 2^{-n}\mathrm{dist}(z^{0,0},Z^*)$，总迭代复杂度

$$\boxed{\ K_{\text{total}}=O\!\Big(\frac{1}{\eta\alpha}\log\frac{1}{\varepsilon}\Big)\ \text{次 SpMV}.\ }$$

对照未 restart 的 $O(1/\varepsilon)$：当 $\alpha$ 不太小时这是**指数级改进**。实际实现无需知道 $\alpha$——自适应 restart 准则（§3.4）隐式地逼近最优 epoch 长度 [6]。

### 3.6 Halpern 锚定：最优 $O(1/K)$ 残差与 restart 线性化

对非扩张 $T$ 与 Halpern 迭代 $z^{k+1}=\frac{k+1}{k+2}Tz^k+\frac{1}{k+2}z^0$（$\beta_k=\frac1{k+2}$ 为锚定权重）：

**定理**（Lieder [24]；Kim [23] 锚定 PPM）：$\displaystyle\|z^K-Tz^K\|\le\frac{2\|z^0-z^*\|}{K+1}$。

证明骨架：归纳证明势能 $P_k=\frac{(k+1)(k+2)}{2}\|z^{k+1}-z^k\|^2+\lambda_k\langle z^0-z^k,\ z^{k+1}-z^k\rangle$ 单调不增且 $P_0\le\|z^0-z^*\|^2$；残差 $r_k=z^k-Tz^k$ 满足 $\|r_k\|$ 递减且 $P_K\ge\frac{(K+1)^2}{2}\|r_K\|^2\cdot\frac{1}{2}$，整理即得。$O(1/K)$ 残差对一般非扩张算子**不可改进**（与 §3.3 遍历速率匹配但作用在更强的残差度量上）。

**rHPDHG** [11]：PDHG 算子在 §3.2 的 $Q$-范数下非扩张，故可直接嵌入 Halpern 框架；不动点残差同样驱动归一化间隙，沿用 §3.5 的 sharpness 收缩得线性收敛，且省掉遍历平均、常数更小、天然解释 PDHG 的螺旋动态（残差单调 $\Rightarrow$ 尾部线性）。HPR-LP [18] 把同一锚定思想用在 Peaceman–Rachford 上；[19] 证明 cuPDLPx 基算法 $\subset$ HPR 基算法——**「Halpern/锚定」已是新一代一阶 LP 求解器的统一设计语言**。

### 3.7 不可行时的发散射线理论（不可行证书）

LP 不可行 $\Rightarrow$ $T$ 无不动点，迭代发散。经典结果（Bauschke 等）：存在 **minimal displacement vector** $v=\Pi_{\overline{\mathrm{range}}(T-I)}(0)\ne0$，且

$$z^{k+1}-z^k\to v,\qquad \frac{z^k-z^0}{k}\to v.$$

[7] 证明：对 LP 的 PDHG，$v=(v_x,v_y)$ 恰给出证书——原始不可行时 $v_y$ 是 Farkas 射线（$A^\top v_y\le0,\ b^\top v_y>0$ 型），对偶不可行（无界）时 $v_x$ 给出对偶 Farkas 证书。实践中用**迭代差分** $z^{k+1}-z^k$（或归一化 $z^k/k$）逼近 $v$ 并归一化输出。**这是 NativePDLP 目前缺失、且实现成本很低的一块**（监控既有量即可）。

### 3.8 复杂度与工程对照小结

| 方法 | 停机度量 | 迭代复杂度 | 每迭代 | 基/暖启动 |
|---|---|---|---|---|
| 对偶单纯形（本仓库原生/HiGHS） | 最优基 | 最坏指数 | 基分解更新 | **有** |
| IPM（NativeIPMLP/HiGHS IPX） | 互补间隙 | $O(\sqrt n\log\frac1\varepsilon)$ | 稀疏 LDL$^\top$/Cholesky | 差（需 crossover） |
| PDHG 遍历 | KKT/间隙 | $O(1/\varepsilon)$ | 2×SpMV | 中 |
| restarted PDHG（PDLP/NativePDLP） | 归一化间隙 | $O(\frac{1}{\eta\alpha}\log\frac1\varepsilon)$ | 2×SpMV | 中 |
| rHPDHG / HPR（前沿） | 不动点残差 | 同阶更优常数 + 两阶段 | 2×SpMV | 中 |

---

## 4. 对本项目的改进路线

### 4.1 定位判断（先把「该不该用 PDLP」说清楚）

- **PDLP 不替代单纯形/IPM**：B&C 树内节点 LP 依赖基的暖启动，单纯形不可替代；中小规模、需要顶点解/精确基的场合 IPM+单纯形仍是正解。
- **PDLP 补的是第三根支柱**：(a) 巨型 MILP 的**根节点 LP**（本仓库 StrictHiGHS 已对「大根节点」自动切 IPM+crossover，`native_adapters.cpp:422-462`——PDLP 是该策略在 IPM 分解内存不可行时的下一档）；(b) **SCUC 大规模市场出清 LP**（天然返回对偶，满足 LMP 定价需求 `src/scuc/scuc.cpp:2440-2444`）；(c) 将来 GPU 化的唯一现实路径（SpMV 友好，分解类算法不友好）。
- **风险/边界**：病态问题精度天花板、无基证书（需 crossover [30] 补丁）、停机判据必须严格相对化否则结果不可比。

### 4.2 Phase 0 — 工程补齐（先做，1–2 周量级，全部低成本）

1. **测试**：把 NativePDLP 纳入 `tests/test_lp_solver.cpp` 与 `tests/test_netlib_regression.cpp`（NETLIB 已知最优值，afiro/adlittle/share2b/stocfor1/kb2 直接复用）；加 $10^{-4},10^{-6},10^{-8}$ 三档容差用例与不可行/无界用例。
2. **停机判据对齐文献**：改为 PDLP 期刊版 [5] §4 的**相对 KKT 残差**（原始可行 $\frac{\|Ax-b\|_\infty}{1+\|b\|_\infty}$、对偶可行 $\frac{\|c-A^\top y-s\|_\infty}{1+\|c\|_\infty}$、间隙 $\frac{|c^\top x-b^\top y|}{1+|c^\top x|+|b^\top y|}$），修复 `pdlp_solver.cpp:455-456` 的绝对/相对混用与 `:593` 的 inf-gap 判收敛。
3. **不可行证书**：按 §3.7 实现迭代差分监控 + Farkas 证书输出（[7]，纯增量代码）。
4. **零成本基线**：HiGHS adapter 暴露 `solver="pdlp"` 选项（库里 cuPDLP-C 已在，`BuildHiGHS.cmake:40`），`BCOptions::highs_mip_lp_solver` 允许 `"pdlp"`——立刻获得与 Google 实现的对照。
5. **接入基准**：在 `benchmark/native_kernel_comparison.cpp` 与 `milp_benchmark_runner` 的根节点消融（现有 IPM±crossover 框架）中加入 NativePDLP / HiGHS-PDLP 两行。
6. **内存**：CSC+CSR 双驻留（`pdlp_solver.cpp:252-269`）改单驻留 + 转置 SpMV（1e8 nnz 省 ~2.4 GB）。

### 4.3 Phase 1 — 算法升级（对照 §2.5，1–3 月）

1. **Halpern restart（rHPDHG）**：把平均点 restart 换成锚定迭代 restart（§3.6）。改动集中在主循环与 restart 判定，预条件/步长/停机全部复用；文献报告对 restart PDHG 的全面提速 [11]，且为将来对齐 HPR-LP/cuPDLPx 铺路。
2. **Feasibility polishing**：[5] 的收尾投影提纯，低成本提高最终可行精度。
3. **步长**：保留现有自适应规则（PDLP 消融显示其不劣于 MP linesearch [4]），把 Malitsky–Pock [3] 作为病态实例的可选保险。
4. **（可选）中心路径重缩放** [16] 与**在线预条件** [31]：进一步降迭代数，建议在 L2O 框架下做 A/B。
5. **多线程 SpMV**：仓库已有 `MIPSOLVERS_USE_OPENMP=ON`；PDLP 的 SpMV 并行化是 GPU 化前的必经步骤（cuPDLP-C 经验：CPU 多线程已能拿下相当一部分加速 [9]）。
6. **接入 B&C 根节点策略**：在「大根节点」分档中加入 PDLP 档（IPM 分解预估内存超阈值时），用 `pdlp_presets::root_lp()` 起步，结果经现有 IPM 点→crash basis 管道（`milp_benchmark_runner` 已有该消融）或直接接 dual simplex 暖启动。

### 4.4 Phase 2 — 平台化（3–12 月）

1. **GPU**：优先评估直接链接/复用 cuPDLPx [10] 或 HPR-LP [18]（均开源），而非自研 CUDA 内核；[19] 的统—关系说明 HPR 基座覆盖 cuPDLPx。注意 NVIDIA cuOpt [29] 采用「GPU PDLP + GPU barrier + CPU 对偶单纯形」**并发三跑**的产品形态，与 StrategyDispatcher 的回退机制天然契合。
2. **Crossover 补丁**：实现/借鉴 [30]（PDHG 螺旋动态→最优基），打通「PDLP 解根节点 → 基 → 原生对偶单纯形进树」全链路；同时服务 SCUC 精确 LMP。
3. **L2O 联动**：用现有 `model_fingerprint`/`solver_config_policy` 学习 restart 阈值、primal weight、步长缩放等按实例族的配置；文献已有 PDHG-unrolled L2O 先例 [36]。
4. **更大尺度**：D-PDLP [33]（多 GPU 分布式）与 PDLP–IPM 混合 [34] 作为远景跟踪。

### 4.5 验证基准建议

- **正确性**：NETLIB 子集（已有基础设施）+ 不可行/无界实例（Netlib infeasible 集）。
- **性能**：MIPLIB 2017 根节点松弛集（PDLP 论文标准集 [4]）、Google 超大规模 11 实例的子集 [5]、本仓库 SCUC 出清 LP 与对抗性缩放 NETLIB（`native_kernel_comparison.cpp` 已有）。
- **报告口径**：SGM10 位移几何均值（HPR-LP/cuPDLP 论文通用），按 $10^{-4}/10^{-6}/10^{-8}$ 分档。

---

## 5. 参考文献

**PDLP/PDHG 主线**
1. Chambolle & Pock, *A first-order primal-dual algorithm for convex problems with applications to imaging*, JMIV 40:120–145, 2011.
2. Pock & Chambolle, *Diagonal preconditioning for first order primal-dual algorithms*, ICCV 2011.
3. Malitsky & Pock, *A first-order primal-dual algorithm with linesearch*, SIAM J. Optim. 28(1), 2018. ([arXiv:1608.08883](https://arxiv.org/abs/1608.08883))
4. Applegate, Díaz, Hinder, Lu, Lubin, O'Donoghue, Schudy, *Practical large-scale linear programming using primal-dual hybrid gradient*, NeurIPS 2021. ([arXiv:2106.04756](https://arxiv.org/abs/2106.04756))
5. 同上, *PDLP: A practical first-order method for large-scale linear programming*（期刊版，含 feasibility polishing、OR-Tools C++、超大规模集）, 2025. ([arXiv:2501.07018](https://arxiv.org/abs/2501.07018))
6. Applegate, Hinder, Lu, Lubin, *Faster first-order primal-dual methods for linear programming using restarts and sharpness*, Math. Program., 2024. ([arXiv:2105.12715](https://arxiv.org/abs/2105.12715))
7. Applegate, Díaz, Lu, Lubin, *Infeasibility detection with primal-dual hybrid gradient for large-scale linear programs*, 2024. ([arXiv:2102.04592](https://arxiv.org/abs/2102.04592))
8. Lu & Yang, *cuPDLP.jl: A GPU implementation of restarted primal-dual hybrid gradient for linear programming in Julia*, Oper. Res. 73(6), 2025. ([arXiv:2311.12180](https://arxiv.org/abs/2311.12180))
9. Lu, Yang, Hu, Huangfu, Liu, Liu, Ye, Zhang, Ge, *cuPDLP-C: A strengthened implementation of cuPDLP for LP by C language*, 2023. ([arXiv:2312.14832](https://arxiv.org/abs/2312.14832))
10. Lu, Peng, Yang, *cuPDLPx: A further enhanced GPU-based first-order solver for linear programming*, 2025. ([arXiv:2507.14051](https://arxiv.org/abs/2507.14051))
11. Lu & Yang, *Restarted Halpern PDHG for linear programming*, 2024. ([arXiv:2407.16144](https://arxiv.org/abs/2407.16144))
12. Lu & Yang, *On the geometry and refined rate of primal-dual hybrid gradient for linear programming*, Math. Program. 212:349–387, 2025. ([arXiv:2307.03664](https://arxiv.org/abs/2307.03664))
13. Lu & Yang, *On the infimal sub-differential size of primal-dual hybrid gradient method and beyond*, 2022. ([arXiv:2206.12061](https://arxiv.org/abs/2206.12061))

**理论与复杂度**
14. Xiong & Freund, *Computational guarantees for restarted PDHG for LP based on "limiting error ratios" and LP sharpness*, Math. Program., 2026. ([arXiv:2312.14774](https://arxiv.org/abs/2312.14774))
15. Xiong & Freund, *On the relation between LP sharpness and limiting error ratio and complexity implications for restarted PDHG*, 2023. ([arXiv:2312.13773](https://arxiv.org/abs/2312.13773))
16. Xiong & Freund, *The role of level-set geometry on the performance of PDHG for conic linear optimization*（含中心路径重缩放）, 2024. ([arXiv:2406.01942](https://arxiv.org/abs/2406.01942))
17. Xiong, *High-probability polynomial-time complexity of restarted PDHG for linear programming*, 2025. ([arXiv:2501.00728](https://arxiv.org/abs/2501.00728))
18. Chen, Sun, Yuan, Zhang, Zhao, *HPR-LP: An implementation of an HPR method for solving linear programming*, SIAM J. Optim., 2025. ([arXiv:2408.12179](https://arxiv.org/abs/2408.12179))
19. Chen, Sun, Yuan, Zhang, Zhao, *On the relationships among GPU-accelerated first-order methods for solving linear programming*, 2025. ([arXiv:2509.23903](https://arxiv.org/abs/2509.23903))
20. Lin, Ma, Ye, Zhang, *An ADMM-based interior-point method for large-scale linear programming*, OMS 36(2-3), 2021. ([arXiv:1805.12344](https://arxiv.org/abs/1805.12344))
21. Deng et al., *An enhanced ADMM-based interior point method for linear and conic optimization*, 2024. ([arXiv:2209.01793](https://arxiv.org/abs/2209.01793))
22. O'Donoghue, Chu, Parikh, Boyd, *Conic optimization via operator splitting and homogeneous self-dual embedding*, JOTA 169, 2016.（SCS）
23. Kim, *Accelerated proximal point method for maximally monotone operators*, Math. Program. 190:57–87, 2021.
24. Lieder, *On the convergence rate of the Halpern-iteration*, Optim. Lett. 15:405–418, 2021.
25. Halpern, *Fixed points of nonexpanding maps*, Bull. AMS 73:957–961, 1967.
26. Hoffman, *On approximate solutions of systems of linear inequalities*, J. Res. NBS 49:263–265, 1952.
27. Basu, Ghoting, Mazumder, Pan, *ECLIPSE: An extreme-scale linear program solver for web-applications*, ICML 2020.
28. Lu 等, *First-order methods for linear programming*（SIAG Views & News 综述）, 2024. ([arXiv:2403.14535](https://arxiv.org/abs/2403.14535))

**实现、推广与产业落地**
29. NVIDIA cuOpt 文档与开源仓库 ([docs.nvidia.com/cuopt](https://docs.nvidia.com/cuopt/user-guide/26.04.00/introduction.html), [github.com/NVIDIA/cuopt](https://github.com/NVIDIA/cuopt))
30. *A new crossover algorithm for LP inspired by the spiral dynamic of PDHG*, 2024. ([arXiv:2409.14715](https://arxiv.org/abs/2409.14715))
31. *Enhanced PDHG for linear programming with online preconditioning*, 2025. ([arXiv:2506.17650](https://arxiv.org/abs/2506.17650))
32. Lin, Xiong, Ge, Ye, *PDCS: A primal-dual large-scale conic programming solver with GPU enhancements*, 2025. ([arXiv:2505.00311](https://arxiv.org/abs/2505.00311))
33. *D-PDLP: Scaling PDLP to distributed multi-GPU systems*, 2026. ([arXiv:2601.07628](https://arxiv.org/abs/2601.07628))
34. *Hybridizing PDHG and interior-point methods*, 2026. ([arXiv:2603.03150](https://arxiv.org/abs/2603.03150))
35. *Anderson accelerated primal-dual hybrid gradient for solving LP*, 2025. ([arXiv:2508.08062](https://arxiv.org/abs/2508.08062))
36. *PDHG-unrolled learning-to-optimize method for large-scale linear programming*, 2024. ([arXiv:2406.01908](https://arxiv.org/abs/2406.01908))
37. Lu & Yang, *A practical and optimal first-order method for large-scale convex quadratic programming*, Math. Program., 2025. ([arXiv:2311.07710](https://arxiv.org/abs/2311.07710))
38. Hinder, *Accessible theoretical complexity of the restarted primal-dual hybrid gradient method for linear programs with unique optima*, 2024. ([arXiv:2410.04043](https://arxiv.org/abs/2410.04043))
39. Lu & Applegate, *Scaling up linear programming with PDLP*, Google Research Blog, 2024. ([link](https://research.google/blog/scaling-up-linear-programming-with-pdlp/))
40. Esser, Zhang, Chan, *A general framework for a class of first order primal-dual algorithms for convex optimization*, SIAM J. Imaging Sci. 3(4), 2010.
41. Pougkakiotis & Gondzio, *An interior point–proximal method of multipliers for convex quadratic programming*（IP-PMM 正则化，NativePDLP 已借鉴）, 2021.
