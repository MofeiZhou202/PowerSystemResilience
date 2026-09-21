# 面向 GPU 的电力系统优化方法论深度报告
## ——凝缩空间内点法与"分解—稠密化"的完备理论、算法实现与研究路线

> **报告定位**：本报告面向拟在电力系统（EE）与高性能计算（CS）交叉领域开展系统研究的读者，围绕一个核心方法论命题——**"当目标硬件上没有高效求解器时，应在算法层把问题变换为有好求解器的形式，而不是等待求解器进步"**——给出从 AC-OPF 问题形式到凝缩空间内点法（HyKKT/LiftedKKT）、从两阶段 SCOPF 分解到批处理稠密化的**完备理论推导、误差分析、参考实现（可运行代码 + 数值实验）与系统研究路线**。所有关键公式均自行推导并与原始文献核对；所有性能数据均标注来源。

---

## 1 总论：EE×CS 交叉点上的方法论转向

### 1.1 问题的计算解剖：时间花在哪里

交流最优潮流（AC-OPF）及其安全约束扩展（SCOPF）是电网运行优化的计算内核。现代求解器几乎无一例外地采用**原对偶内点法**（primal-dual interior-point method, IPM）：其每次迭代的主体工作是组装并求解一个**大规模、稀疏、对称不定、且随迭代趋于极度病态**的 Karush–Kuhn–Tucker（KKT）线性系统。文献反复确认这一线性求解占 NLP 求解总时间的 **80% 以上**[^198^]；因此"电力系统优化的高性能化"在计算层面等价于"KKT 系统的高性能求解"。

CPU 上的黄金标准是 **LBLᵀ（Bunch–Kaufman）稀疏分解配精细数值主元**（HSL MA27/MA57、PARDISO），其鲁棒性来自主元策略对不定性与病态的驯服[^158^]。然而数值主元本质上是**数据依赖的串行重排**：它导致不规则内存访问与全局同步，与 GPU 的 SIMT 吞吐量模型根本冲突。2021 年面向电网优化 KKT 矩阵的五款 GPU 求解器基准评测的结论是冷酷的：**没有任何一款 GPU 稀疏直接求解器在该类问题上取得显著加速**[^6^]；截至 2023 年，"不存在能直接高效处理大规模 NLP 稀疏不定 KKT 系统的 GPU 直接求解器"仍是该领域的基本判断[^143^]。

### 1.2 方法论命题：重构问题，而非等待硬件/求解器

正是在这一僵局下，2021—2024 年间涌现出四条"**在算法层重构问题结构以适配 GPU**"的路线，其核心思想可统一表述为：**把线性代数变换为 GPU 喜欢的三种形态——(i) 稀疏对称正定（Cholesky 无需数值主元）；(ii) 小型稠密（批处理分解占满流式多处理器）；(iii) 纯矩阵—向量乘与向量运算（迭代法）**。

| 路线 | 重构机制 | 线性代数形态 | 代表实现 | 来源 |
|---|---|---|---|---|
| A. 凝缩空间 IPM | 不定 KKT → 对称正定（凝缩/正则化） | 稀疏 SPD Cholesky | MadNLP+HyKKT/LiftedKKT+cuDSS | [^143^][^75^] |
| B. 分解—稠密化 | 大稀疏问题 → 主问题 + 大量小稠密子问题 | 批处理稠密 LU/Cholesky | ExaSGD: ExaGO+HiOp（Frontier） | [^4^][^5^][^3^] |
| C. 拉氏分解批处理 | 大 NLP → 数万个微型 NLP | 逐 warp 小稠密分解 + 向量更新 | ExaADMM/ProxAL/ExaTron | [^197^][^198^][^180^] |
| D. 零空间/降维 | 消去等式约束 → 稠密降阶系统 | 稠密约化 Hessian | reduced-space 实时 OPF | [^143^][^190^] |

四条路线在 2023—2025 年集中收获了标志性成果：路线 A 在 PGLIB 大规模 AC-OPF 上取得相对 Ipopt+MA27 **约一个数量级**的加速[^143^][^89^]；路线 B 在 Frontier 超算 **9,000 节点上 20 分钟完成 10 万+预想事故与天气场景的 SCOPF**，是调度员惯常 50–100 个手工事故集的三个数量级跃升[^4^]；路线 C 在 Summit 单节点 6 GPU 上相对 40 CPU 核取得 **9–35 倍**加速并展示 70,000 母线系统的秒级跟踪能力[^197^][^198^]；同一时期 GPU 原生稀疏直接求解库 cuDSS 的成熟[^143^]与 Julia 生态 MadSuite（ExaModels/MadNLP/MadIPM/MadNCL/ExaModelsPower）的成型[^199^]，使"全 GPU 驻留优化栈"从原型变为可用基础设施。

### 1.3 报告结构

第 2 章给出 IPM 与 KKT 系统的完备推导与病态理论；第 3、4 章分别推导两条凝缩空间路线（LiftedKKT、HyKKT）及其误差分析；第 5 章展开分解—稠密化路线（ExaSGD/HiOp 两阶段 SCOPF、ExaTron/ExaADMM 批处理）；第 6 章剖析支撑栈（SIMD 自动微分、全 GPU 驻留、MadSuite 生态）；第 7 章给出 NumPy 参考实现与四组数值实验；第 8 章汇总性能证据；第 9 章提出面向 EE×CS 系统研究的开放问题清单与复现路径。

---

## 2 内点法与 KKT 系统：从 AC-OPF 到牛顿方程

### 2.1 从 AC-OPF 到标准 NLP 形式

以极坐标形式为例，AC-OPF 的决策变量取 $x=(v,\theta,p_g,q_g)$（节点电压幅值/相角、发电机有功/无功），目标为发电成本 $\min \sum_g c_{2g}p_g^2+c_{1g}p_g+c_{0g}$，约束包括节点有功/无功平衡等式、线路潮流方程、以及电压/出力/线路容量的界约束。一个重要事实是：**全部约束都可归入"等式 + 界"的形式**——一般不等式 $h(x)\le 0$ 可通过引入辅助变量 $s_h=h(x)$ 转为等式加界 $s_h\le 0$。因此不失一般性，研究对象为

$$
\min_{x^\flat\le x\le x^\sharp}\; f(x)\quad \text{s.t.}\quad g(x)=0,\qquad f:\mathbb{R}^n\to\mathbb{R},\; g:\mathbb{R}^n\to\mathbb{R}^m. \tag{2.1}
$$

AC 潮流模型特别适合后文将反复利用的一个结构性质：其模型方程由**少数几种随网络规模不增长的计算模式**（发电成本、参考角约束、线路首末端有功/无功潮流、相角差约束、视在潮流上限、节点功率平衡等共约 15 种）按索引集重复构成[^75^]，这既是第 6 章 SIMD 自动微分的基础，也决定了雅可比 $A=\nabla_x g$ 与拉格朗日 Hessian $W=\nabla^2_{xx}\mathcal{L}$ 的**图诱导稀疏性**：每个节点的方程只耦合其电气邻域，$A$ 的每行非零元数 $O(1)$。

### 2.2 对数障碍与原对偶 KKT 条件

IPM 用对数障碍光滑化界约束，引入障碍参数 $\mu>0$ 的等式约束子问题：

$$
\min_{x}\; f(x)-\mu\mathbf{1}^{\!\top}\log(x-x^\flat)-\mu\mathbf{1}^{\!\top}\log(x^\sharp-x)\quad\text{s.t.}\quad g(x)=0. \tag{2.2}
$$

记拉格朗日函数 $\mathcal{L}=f(x)-y^\top g(x)-z^{\flat\top}(x-x^\flat)-z^{\sharp\top}(x^\sharp-x)$，其中 $y\in\mathbb{R}^m$ 为等式乘子，$z^\flat,z^\sharp\ge 0$ 为界乘子。障碍问题的一阶最优性条件（原对偶 KKT 条件）为

$$
F_\mu(w)=
\begin{bmatrix}
\nabla f(x)-A(x)^\top y-z^\flat+z^\sharp\\
g(x)\\
Z^\flat(x-x^\flat)-\mu\mathbf{1}\\
Z^\sharp(x^\sharp-x)-\mu\mathbf{1}
\end{bmatrix}=0,\qquad
A(x):=\nabla_x g(x), \tag{2.3}
$$

其中 $Z^\flat=\mathrm{diag}(z^\flat)$ 等沿用对角阵记号。当 $\mu\searrow 0$ 时，$F_\mu$ 的解轨迹（中心路径）趋向原问题 KKT 点。IPM 的本质即**用牛顿法追踪中心路径**：在当前迭代点 $w_k$ 解线性化牛顿方程 $\nabla_w F_\mu(w_k)\,d_k=-F_\mu(w_k)$，再以线搜索（如 Wächter–Biegler 滤波线搜索[^24^]）确定步长，周期性下调 $\mu$。

### 2.3 牛顿系统的逐层块消去

写出 (2.3) 的牛顿方程并消去恒可逆的对角块，是理解一切 KKT 重构的基础。完整推导如下。

**第 0 层（全空间）**。未知量 $(\Delta x,\Delta y,\Delta z^\flat,\Delta z^\sharp)$，维数 $3n+m$：

$$
\begin{bmatrix}
W & -A^\top & -I & I\\
A & 0 & 0 & 0\\
Z^\flat & 0 & X-X^\flat & 0\\
-Z^\sharp & 0 & 0 & X^\sharp-X
\end{bmatrix}
\begin{bmatrix}\Delta x\\ \Delta y\\ \Delta z^\flat\\ \Delta z^\sharp\end{bmatrix}
=
\begin{bmatrix}-r_x\\ -r_y\\ -r_4\\ -r_5\end{bmatrix}, \tag{2.4}
$$

其中 $W=\nabla^2_{xx}\mathcal{L}$，$r_x=\nabla f-A^\top y-z^\flat+z^\sharp$，$r_y=g(x)$，$r_4=Z^\flat(x-x^\flat)-\mu\mathbf 1$，$r_5=Z^\sharp(x^\sharp-x)-\mu\mathbf 1$。IPM 保证迭代严格位于界内部，故 $X-X^\flat$、$X^\sharp-X$ 恒为可逆对角阵。

**第 1 层（增广系统）**。由后两行解出 $\Delta z^\flat=(X-X^\flat)^{-1}(-r_4-Z^\flat\Delta x)$、$\Delta z^\sharp=(X^\sharp-X)^{-1}(-r_5+Z^\sharp\Delta x)$ 并回代第一行，得 $(n+m)$ 维**增广（augmented）系统**

$$
\underbrace{\begin{bmatrix} K & A^\top\\ A & 0\end{bmatrix}}_{M_{aug}}
\begin{bmatrix}\Delta x\\ -\Delta y\end{bmatrix}
=
\begin{bmatrix} b_x\\ b_y\end{bmatrix},\qquad
\begin{aligned}
K &:= W+\Sigma_x+\delta_w I,\\
\Sigma_x &:= (X-X^\flat)^{-1}Z^\flat+(X^\sharp-X)^{-1}Z^\sharp,
\end{aligned} \tag{2.5}
$$

$b_x=-r_x-(X-X^\flat)^{-1}r_4+(X^\sharp-X)^{-1}r_5$，$b_y=-r_y$，$\delta_w\ge0$ 为惯性修正正则项。$M_{aug}$ 是**对称不定拟定（quasi-definite）鞍点系统**——$K$ 在对角扰动下正定、$(2,2)$ 块为零（或 $-\delta_c I$），这就是 CPU 时代 LBLᵀ 分解的用武之地[^158^]。

**第 2 层（凝缩系统）**。若进一步消去 $\Delta y$（Schur 补），得到 $n$ 维**凝缩（condensed）系统**

$$
\underbrace{\left(K+A^\top \Sigma_s A\right)}_{M_{cond}}\Delta x=\tilde b_x \tag{2.6}
$$

的一般形式（$\Sigma_s$ 的来源见第 3 章的松弛构造；此处先指出结构）。凝缩把不定性"压进" $A^\top\Sigma_s A$：只要约化 Hessian 正定，$M_{cond}$ 即**对称正定**——这是全部凝缩空间方法的支点，其严格论证见 3.3 节的 Sylvester 惯性律推导。

**表 2  三层 KKT 系统对比**

| 层级 | 维数 | 定性 | 标准解法 | 数值主元 | GPU 适配 |
|---|---|---|---|---|---|
| 全空间 (2.4) | $3n+m$（含松弛 $3n{+}4m$） | 对称不定 | 从不直接使用 | — | 差 |
| 增广 (2.5) | $n+m$（含松弛 $n{+}2m$） | 对称不定（拟定） | LBLᵀ（MA27/MA57/PARDISO）[^158^] | **必需** | 差（主元串行）[^6^] |
| 凝缩 (2.6) | $n$ | **对称正定** | Cholesky / 固定主元重分解 | **不需要** | **好** |

### 2.4 为什么 LBLᵀ 难以移植 GPU

LBLᵀ 将对称不定矩阵分解为 $PKP^\top=LBL^\top$（$L$ 单位下三角、$B$ 为 $1\times1$ 与 $2\times2$ 对角块、$P$ 为置换）。Bunch–Kaufman 主元在消去每一步根据数值增长因子选择 $1\times1$ 或 $2\times2$ 主元块，是鲁棒性的来源[^177^]；但每一次主元选择都引入**数据依赖的分支与行列交换**，使消去树结构在运行时才确定，无法预分配内存与调度并行任务。这正是 GPU 的"反模式"：GPU 要求规则的批量线程执行与可预测的访存。实证上，电网优化 KKT 矩阵上的 GPU 稀疏不定分解相对 CPU 基线全面落后[^6^]；Ipopt 十款线性求解器的独立评测亦显示 OPF KKT 上串行 MA27/MA57 已接近最优，并行化开销反而拖累性能[^158^]。结论：**不定性（主元）而非稀疏性本身，才是 GPU 化的根本障碍**——这正是凝缩空间方法选择"消灭不定性"的逻辑起点。

### 2.5 病态的必然性与结构化

即使消灭不定性，病态也无法回避。随 $\mu\to0$，对活跃界约束有 $x_i\to x^\sharp_i$（或 $x^\flat_i$）而 $z_i\to\hat z_i>0$，故 $\Sigma_x$ 的对角元按 $\Theta(1/\mu)$ 发散；非活跃约束的对偶 $z_i\to0$ 使相应对角元按 $\Theta(\mu)$ 消失。Wright（1998）的经典分析表明[^146^]：

- **增广系统**：特征值分为 $\Theta(1)$ 与 $\Theta(\mu)$ 两支，$\kappa(M_{aug})=\Theta(1/\mu)$；
- **凝缩系统**：发散的 $\Sigma$ 直接进入 $A^\top\Sigma A$，$\lambda_{\max}=\Theta(1/\mu)$、$\lambda_{\min}=\Theta(\mu)$，故 $\kappa(M_{cond})=\Theta(1/\mu^2)$。

凝缩以**平方级病态**换取正定性——这曾使凝缩路线在 CPU 时代被判定为"不划算"而长期沉寂[^75^]。但两个现代事实改变了权衡：其一，Wright 式分析进一步揭示病态是**结构化的**——发散特征值集中于活跃约束对应的低维子空间，而 KKT 残差恰位于良态子空间内，使得直接解的精度损失远小于最坏情形估计；Pacaud 等将这一理论推广到含等式约束的凝缩矩阵 $K_\gamma$、$K_\tau$，证明了"**结构受控的病态**"（structured ill-conditioning）并给出 Richardson 迭代精化的误差界[^143^][^146^]（详见 3.4、4.3 节）。其二，GPU 上 Cholesky 的吞吐量优势（无主元、可批处理、可达机器峰值的高比例）足以"买下"这一病态溢价——**用可控的数值风险换取硬件利用率的数量级提升，正是 EE×CS 方法论的核心交易**。

![图D1 KKT 系统的逐层块消去](images2/d1_kkt_condensation.png)

---

## 3 凝缩空间方法 I：LiftedKKT（不等式松弛 + 凝缩 Cholesky）

LiftedKKT 由 Shin–Anitescu–Pacaud 在 GPU-OPF 框架中首次完整提出[^75^]，随后在 Pacaud 等的误差分析工作中被系统化[^143^][^146^]。本章给出其完整推导链条。

### 3.1 不等式松弛：把等式"抬升"为界约束

凝缩 (2.6) 的障碍在于：等式约束 $g(x)=0$ 的存在使 $(2,2)$ 块为零，无法直接 Schur 消去。LiftedKKT 的第一步是在算法开始时对等式施加**不等式松弛**（inequality relaxation，亦称 lifting）：引入松弛变量 $s\in\mathbb{R}^m$，

$$
g(x)-s=0,\qquad s^\flat\le s\le s^\sharp,\qquad s^\flat=-\tau\mathbf 1,\; s^\sharp=+\tau\mathbf 1, \tag{3.1}
$$

其中松弛宽度取为 IPM 容差 $\tau=\varepsilon_{tol}$。其合理性在于：IPM 本来只把不等式满足到 $\pm\varepsilon_{tol}$ 精度，允许等式残差落入宽度 $2\tau$ 的"走廊"不改变实际可达精度[^143^]；论文实现的默认精度即 $\varepsilon_{tol}=\varepsilon_{mach}^{1/4}\approx10^{-4}$[^75^]。松弛的回报是结构性的：问题 (2.1) 变为

$$
\min_{x^\flat\le x\le x^\sharp,\; s^\flat\le s\le s^\sharp}\; f(x)\quad\text{s.t.}\quad g(x)-s=0, \tag{3.2}
$$

其拉格朗日函数中 $s$ 只以**线性形式**出现（$\mathcal{L}$ 含 $+y^\top s$ 项），这将在牛顿系统中制造出一个可被廉价求逆的对角 $(2,2)$ 块——凝缩的全部秘密。

### 3.2 从全空间到凝缩系统：完整代数推导

对 (3.2) 的障碍问题写 KKT 条件（沿用 2.2 节记号，新增 $s$ 的界乘子 $z_s^\flat,z_s^\sharp$）：

$$
\begin{aligned}
\nabla f(x)-A^\top y-z_x^\flat+z_x^\sharp&=0, &
Z_x^\flat(x-x^\flat)&=\mu\mathbf 1,\\
y-z_s^\flat+z_s^\sharp&=0, &
Z_x^\sharp(x^\sharp-x)&=\mu\mathbf 1,\\
g(x)-s&=0, &
Z_s^\flat(s-s^\flat)&=\mu\mathbf 1,\\
&& Z_s^\sharp(s^\sharp-s)&=\mu\mathbf 1.
\end{aligned} \tag{3.3}
$$

**全空间牛顿系统**（未知量 $(\Delta x,\Delta s,\Delta y,\Delta z_x^\flat,\Delta z_x^\sharp,\Delta z_s^\flat,\Delta z_s^\sharp)$，维数 $3n+4m$）与 (2.4) 同构。对四组界乘子行逐一解出（对角求逆），回代得 $(n+2m)$ 维**增广系统**

$$
\underbrace{\begin{bmatrix}
W+\Sigma_x+\delta_w I & 0 & A^\top\\
0 & \Sigma_s+\delta_w I & -I\\
A & -I & -\delta_c I
\end{bmatrix}}_{M_{aug}}
\begin{bmatrix}\Delta x\\ \Delta s\\ \Delta y\end{bmatrix}
=
\begin{bmatrix}q_x\\ q_s\\ q_y\end{bmatrix}, \tag{3.4}
$$

其中 $\Sigma_s:=(S-S^\flat)^{-1}Z_s^\flat+(S^\sharp-S)^{-1}Z_s^\sharp\succ0$，$q_x,q_s,q_y$ 为吸收了对偶残差的右端项（与 2.3 节推导完全相同的形式）[^75^]。

**凝缩**：$M_{aug}$ 的右下 $2\times2$ 块 $\begin{bmatrix}\Sigma_s+\delta_wI&-I\\-I&-\delta_cI\end{bmatrix}$ 恒可逆（其 Schur 补为对角正定阵），消去 $(\Delta s,\Delta y)$ 得 $n$ 维凝缩系统

$$
\boxed{\;
\underbrace{\Big(W+\delta_w I+\Sigma_x+A^\top D A\Big)}_{M_{cond}\;\in\;\mathbb{R}^{n\times n}}\Delta x
= q_x+A^\top\big(Cq_s+Dq_y\big)\;} \tag{3.5}
$$

其中 $C:=\big(\delta_c\Sigma_s+(1+\delta_c\delta_w)I\big)^{-1}$，$D:=(\Sigma_s+\delta_wI)C$，**均为对角阵**，故 $C,D$ 的求值与 $A^\top DA$ 的组装都是逐元素廉价运算[^75^]。未正则化（$\delta_w=\delta_c=0$）时 $D=\Sigma_s$、(3.5) 右端退化为 $q_x+A^\top(q_s+\Sigma_s q_y)$。对偶与松弛步由对角运算回代：

$$
\Delta s:=C\big(\delta_c q_s-(q_y+A\Delta x)\big),\qquad
\Delta y:=(\Sigma_s+\delta_wI)\Delta s-q_s, \tag{3.6}
$$

界对偶步 $\Delta z$ 由 (2.3) 型对角公式恢复。整条消去链中**唯一需要分解的矩阵是 $n\times n$ 的 $M_{cond}$**。

### 3.3 正定性：Sylvester 惯性律论证

IPM 的下降性要求牛顿方向满足惯性条件 $\mathrm{inertia}(M_{aug})=(n+m,0,m)$（即 $n{+}m$ 个正特征值、$m$ 个负特征值）；惯性修正通过增大 $\delta_w,\delta_c$ 直至该条件满足。关键论证：块消去等价于合同变换（congruence），由 **Sylvester 惯性律**，合同变换不改变惯性，故

$$
\mathrm{inertia}(M_{aug})=(n+m,0,m)\iff \mathrm{inertia}(M_{cond})=(n,0,0)\iff M_{cond}\succ0. \tag{3.7}
$$

换言之，**凡满足下降性要求的正则化，都自动使凝缩矩阵对称正定**；反之，只需试探性地增大 $\delta_w$ 直到 $M_{cond}$ 的 Cholesky 分解成功，即等价完成了惯性控制——无需任何特征值估计。正定性的直接推论是：$M_{cond}$ 可用**无主元 Cholesky 或固定主元序列的重分解（refactorization）**求解，符号分解（排序+符号消去）在整个 IPM 过程中只做一次[^75^]。这正是 GPU 梦寐以求的结构：cuSOLVER/cuDSS 的稀疏 Cholesky/LDLᵀ 可直接承接[^143^]。

### 3.4 结构化病态与误差分析

凝缩的病态代价（2.5 节）在 LiftedKKT 中有更精细的图景。随 $\mu\to0$：活跃 $x$ 界对应 $\Sigma_x$ 对角元发散；松弛变量收敛到走廊边缘 $|s^*|\le 2\tau$，使 $\Sigma_s$ 中出现 $\hat z/\tau$ 量级的巨大对角元。实测 $K_\tau$（LiftedKKT 凝缩阵）的条件数在紧容差下**可超过 $10^{18}$**，矩阵在浮点意义下近乎奇异[^143^]。然而 Pacaud 等的误差分析（推广 Wright 1998 至含等式的凝缩阵）证明了三层保护机制[^143^][^146^]：

1. **残差位于良态子空间**：设 $\Xi$ 为发散特征值尺度参数，可证右端扰动满足 $O(\Xi^{-1}\mathbf u)$ 且落在活跃雅可比的值域内，从而使 $|\widehat d_x-d_x|=O(\Xi\mathbf u)$——解的**绝对**误差被控制在机器精度 $\mathbf u$ 的 $\Xi$ 倍而非 $\kappa$ 倍；
2. **误差回代不放大**：$\widehat d_s=d_s+O(\varepsilon_K)$、活跃/非活跃分量的对偶误差均有界，其中 $\varepsilon_K$ 为线性解的绝对精度[^143^]；
3. **作用于全系统的 Richardson 精化**：精化残差在**增广系统** $M_{aug}$（而非凝缩系统）上计算，每次精化用同一 Cholesky 因子做一次廉价回代，即可把相对残差压回可接受水平——实测多次 Richardson 迭代成功驯服了 $\kappa>10^{18}$ 的近奇异系统，使 cuDSS 在约 20 秒内将大型算例解至最优[^143^]。

这一理论的实践含义是凝缩 IPM 的**精度上限约为 $\varepsilon_{mach}^{1/4}\approx10^{-4}$**[^75^]：对在线安全分析、调度滚动优化而言足够，对需要 $10^{-8}$ 紧容差的场合则应切换 MadNCL 等替代路线[^193^]。

### 3.5 稀疏性与复杂度

$M_{cond}=W+\Sigma_x+A^\top DA$ 并非必然稠密：$W$ 与 $A$ 为图诱导带状/邻接稀疏时，$A^\top DA$ 的稀疏模式是 $A$ 列关联图的"二跳闭包"——对电网这类低度图仍高度稀疏[^75^]。其浮点成本与全空间 LBLᵀ 同阶，差异在于常数因子与规则性。但文献同时明确警告：**单个稠密行（如全耦合约束）会使凝缩阵稠密化**，这是 CUTEst 通用 NLP 基准上凝缩法性能波动的主因[^143^]；电力 NLP 的图结构恰好规避了这一陷阱，这是凝缩路线与 OPF"天作之合"的结构原因。

---

## 4 凝缩空间方法 II：HyKKT（Golub–Greif 正则化 + 混合直接-迭代）

HyKKT 由 Regev、Chiang、Darve、Petra、Saunders 等提出（2023）[^24^]，其思想源头是 Golub & Greif（2003）的增广拉格朗日 KKT 重构[^176^]，后被移植进 MadNLP 的全 GPU 栈[^143^][^192^]。与 LiftedKKT"改问题"不同，HyKKT **"改系统"**：直接对凝缩 KKT 系统做等价变换。

### 4.1 $K_\gamma$ 正则化与正定性定理

考虑凝缩 KKT 系统（记 $G$ 为雅可比、$K$ 为含 $\Sigma_x$ 的 $(1,1)$ 块，$\delta_c=0$）的鞍点形式

$$
\begin{bmatrix}K & G^\top\\ G & 0\end{bmatrix}\begin{bmatrix}d_x\\ d_y\end{bmatrix}=\begin{bmatrix}\bar r_1\\ \bar r_2\end{bmatrix}. \tag{4.1}
$$

将第二行左乘 $\gamma G^\top$（$\gamma>0$）后加到第一行——此为等价变换，解不变——得

$$
\begin{bmatrix}K_\gamma & G^\top\\ G & 0\end{bmatrix}\begin{bmatrix}d_x\\ d_y\end{bmatrix}=\begin{bmatrix}\bar r_1+\gamma G^\top\bar r_2\\ \bar r_2\end{bmatrix},\qquad
K_\gamma:=K+\gamma G^\top G. \tag{4.2}
$$

**正定性定理**：设 $G$ 行满秩、$Z$ 为 $\mathrm{null}(G)$ 的基。若约化 Hessian $Z^\top KZ\succ0$（由 IPM 惯性修正保证），则存在阈值 $\underline\gamma$，使 $\forall\gamma>\underline\gamma$ 有 $K_\gamma\succ0$[^24^][^176^]。

**证明梗概**：将 $\mathbb{R}^n=\mathrm{range}(G^\top)\oplus\mathrm{null}(G)$ 正交分解，对 $v=G^\top u+Zw$ 有 $v^\top K_\gamma v=\underbrace{u^\top GKG^\top u+2u^\top GKZw+w^\top Z^\top KZw}_{\text{交叉项有界}}+\gamma\|GG^\top u\|^2$。在 $\mathrm{range}(G^\top)$ 上 $\gamma G^\top G$ 的最小特征值 $\ge\gamma\sigma_{\min}(G)^2\to\infty$；在 $\mathrm{null}(G)$ 上 $K_\gamma$ 退化为 $Z^\top KZ\succ0$；交叉二阶项被 $\gamma$ 主导，故整体正定。$\square$

结合 Sylvester 论证：当惯性修正使增广系统惯性为 $(n{+}m,0,m)$ 时，必存在这样的 $\underline\gamma$[^176^]。

### 4.2 混合求解：Cholesky + Schur 补 CG

$K_\gamma\succ0$ 后，HyKKT 用三步混合直接-迭代法解 (4.2)[^24^][^143^]：

1. **组装并稀疏 Cholesky 分解** $K_\gamma=LL^\top$（无主元，GPU/cuDSS 友好；符号分解全程复用）；
2. **Schur 补 CG 求对偶步**：对 $S_\gamma:=GK_\gamma^{-1}G^\top$ 解
$$
S_\gamma\,d_y=GK_\gamma^{-1}(\bar r_1+\gamma G^\top\bar r_2)-\bar r_2, \tag{4.3}
$$
$S_\gamma$ 稠密但**从不显式形成**——以"Gᵀv → Cholesky 三角回代 → G(·)"的隐式矩阵—向量乘喂给共轭梯度（CG）；
3. **回代原步**：$K_\gamma d_x=\bar r_1+\gamma G^\top\bar r_2-G^\top d_y$（复用同一因子）。

CG 的可行性建立在**特征值聚集定理**上：$S_\gamma$ 的全部特征值随 $\gamma$ 增大收敛到 $1/\gamma$，从而 $\lim_{\gamma\to\infty}\kappa_2(S_\gamma)=1$（Regev et al., Theorem 4）[^24^]。直觉上，$K_\gamma^{-1}\approx(\gamma G^\top G)^{-1}$ 在 $\mathrm{range}(G^\top)$ 上占主导，使 $S_\gamma\approx\frac1\gamma I$。由于 CG 的收敛速度由相异特征值个数与聚集度决定，**$\gamma$ 越大 CG 越快**；工程实践取 $\gamma=10^7$，CG 平均迭代数 **<10 次且无需预条件子**[^192^]。

### 4.3 $\gamma$ 的双刃剑与结构化病态

$\gamma$ 的代价是 $\kappa(K_\gamma)$ 随 $\gamma$ **线性增长**（本报告实验 E3 实测：$\gamma$ 从 $10^0$ 增至 $10^8$，$\kappa(K_\gamma)$ 从 $10^5$ 增至 $10^{12}$，CG 迭代同步下降）。精度为何没有崩溃？误差分析给出答案[^143^][^146^]：在精确算术下 Schur 补具有结构

$$
S_\gamma=GY\,\Sigma_L^{-1}Y^\top G^\top+O(\Xi^2), \tag{4.4}
$$

其中 $Y$ 为良态基、$\Sigma_L$ 为大特征值对角阵、$\Xi$ 为病态尺度；同时 $GU_S=O(\Xi)$（$U_S$ 为小特征向量阵），即**病态方向几乎正交于 $G$ 的作用域**。Cholesky 分解的舍入误差 $\Gamma=O(\Xi^{-1}\mathbf u)$ 经 $K_\gamma^{-1}$ 放大后，投影到 $S_\gamma$ 上仅为 $O(\Xi\mathbf u)$ 量级——**病态被限制在与 CG 迭代空间几乎正交的方向上**，因此 CG 的精度损失有限。这与 LiftedKKT 的 Wright 式保护机制同源，共同构成"结构化病态"理论的两翼。

### 4.4 HyKKT vs LiftedKKT：机制对比

**表 3  两条凝缩路线的对比**

| 维度 | HyKKT | LiftedKKT |
|---|---|---|
| 重构对象 | KKT 系统（等价变换 (4.2)） | 优化问题（松弛 (3.1)） |
| SPD 化机制 | $K_\gamma=K+\gamma G^\top G$ | $M_{cond}=W+\Sigma_x+A^\top D A$ |
| 线性求解 | 稀疏 Cholesky + **Schur 补 CG**（混合） | 稀疏 Cholesky/LDLᵀ **直解** |
| 额外误差源 | CG 截断误差 $\varepsilon_K$ | 松弛扰动（解的是扰动问题） |
| 关键参数 | $\gamma$（实践 $10^7$）[^192^] | $\tau=\varepsilon_{tol}$[^143^] |
| 精度上限 | CG 容差控制，可达较高精度 | $\varepsilon_{mach}^{1/4}\approx10^{-4}$[^75^] |
| 每 IPM 迭代成本 | 1 次分解 + 数次三角回代（CG 内） | 1 次分解 + 精化回代 |
| 共同基础设施 | \multicolumn{2}{c}{MadNLP + cuDSS Cholesky + 全系统 Richardson 精化[^143^]} |

实测在 A100 上（tol=1e-6），两者在 PGLIB 大算例上性能接近：如 78484_epigrids 算例，Ipopt+MA27 总时 207.8s（线性求解 179.3s），LiftedKKT+cuDSS 总时 18.0s，HyKKT+cuDSS 总时 18.9s——**相对 CPU 黄金标准约 11 倍加速**[^89^]。

---

## 5 分解—稠密化路线：从 ExaSGD 两阶段 SCOPF 到批处理 NLP

凝缩空间方法在"单个 NLP 的 KKT 求解"层面重构问题；而 ExaSGD 项目面对的是一个更高层级的矛盾：**SCOPF 的天然形式（大规模稀疏不规则）与百亿亿次 GPU 机器的口味（稠密、规则、批量）不匹配**。其解法是在**算法结构层**做分解与稠密化。

### 5.1 SCOPF 的扩展形式与块结构

预想事故约束最优潮流（SCOPF）要求确定基态发电设定点，使任一预想事故 $c\in\mathcal C$ 发生后系统仍安全。其扩展形式为两阶段随机规划[^1^]：

$$
\begin{aligned}
\min\;&\textstyle\sum_{c\in\mathcal C} f(x_c)\\
\text{s.t.}\;& g(x_c)=0,\quad h(x_c)\le 0,\quad x^-\le x_c\le x^+,\\
& -\delta_c x\le x_c-x_0\le \delta_c x,\quad c\neq0,
\end{aligned} \tag{5.1}
$$

其中 $x_0$ 为基态、$x_c$ 为事故态，爬坡约束（最后一行）是**唯一的耦合项**——目标与约束均按事故可分。计入风电/天气场景 $\pi_s$ 后得随机变体 $\min\sum_s\pi_s\sum_c f(x_{s,c})$[^1^]。问题的计算需求由三个维度乘积决定：电网规模 × 预想事故数 × 天气场景数[^5^]；按北美 NERC 运行标准，调度决策窗口仅约 30 分钟[^181^]。

### 5.2 HiOp 两阶段分解与稠密化压缩

ExaSGD 的两阶段架构[^3^]：**第一阶段**用高精度 NLP 求解器（Ipopt/HiOp，CPU）解基态主问题，产出基态设定点 $x_0^\star$；**第二阶段**固定 $x_0^\star$，把 $N_c$ 个事故校正 OPF（recourse）子问题分发给 GPU 集群并行求解——事故间无通信，呈尴尬并行（embarrassingly parallel）[^3^]。但每个子问题本身仍是一个万节点级稀疏 NLP，其 IPM 仍卡在稀疏不定 KKT 上。项目的应对是坦率的：经穷尽调研确认"**没有现成的 GPU 稀疏线性求解器可用**"后，团队选择"**把电网表示尽可能稠密化（densify）**"——接受更差的渐近复杂度，换取 GPU 协处理器的高利用率，形成混合稠密/稀疏（mixed dense/sparse）表示[^5^]。

其机制是**线性代数压缩**：利用事故子问题与基态共享绝大部分雅可比结构的事实，把每个子问题的牛顿系统压缩为**小型稠密线性系统**，转由 GPU 上成熟的批处理稠密求解器（MAGMA batched LU）处理；配合定制化稠密—稀疏混合核函数，项目期内 HiOp 的 GPU 求解速度提升约 **100 倍**[^4^]。最终于 2023 年 4–5 月在 Frontier（世界首台百亿亿次超算）**9,000 节点**上完成迄今最大规模 SCOPF：**10 万余预想事故 × 天气场景，20 分钟**；对照工业实践为 50–100 个手工事故与 5–10 个场景[^4^]。结果经 PNNL 用工业标准工具验证：所求基态设定点能以极小运行成本增量大幅削减事故后停电[^4^]。

值得强调的是该项目的自我扬弃：ExaSGD 团队随后与求解器社区合作发展了加速器上的稀疏数学库（即 cuDSS 等所代表的 GPU 原生稀疏直接法生态），把 ExaGO/HiOp 重构回**原生稀疏表示**——其收益是"整个北美电网模型可装入单个加速器"、运行时间进一步缩短[^5^]。这一"先稠密化绕开、再稀疏化回归"的螺旋，恰好佐证了方法论命题的时效性：**结构重构是在求解器能力边界上的动态适配，而非一次性终局**。

![图D2 ExaSGD/HiOp 两阶段 SCOPF 分解](images2/d2_exasgd_decomposition.png)

### 5.3 拉氏分解批处理：ExaTron / ExaADMM / ProxAL

第三条路线把分解做到更细粒度。ExaADMM 将 ACOPF 按**电网元件**（发电机、母线、线路）分解为数以万计的微型非凸 NLP（19402_goc 算例每次 ADMM 迭代含 **34,704** 个支路子问题），子问题经增广拉格朗日技巧化为**界约束 NLP**，由批处理求解器 ExaTron 在 GPU 上批量求解[^197^][^198^]。ExaTron 的实现哲学是"**全部驻留 GPU、零主机往返**"：每个子问题（$n\le32$）映射到一个线程束（warp），以**完全稠密 Cholesky** 作信赖域牛顿的预条件（小子问题下稠密线性代数与稀疏同样高效，且预条件更精确），共享内存管理使批处理性能再翻倍[^197^]。在 Summit 单节点上，ExaTron 对批量规模与 GPU 数呈**线性扩展**，6 GPU 相对 40 CPU 核加速 **9–35 倍**[^197^]；ExaADMM 全栈（含共识变量与乘子更新，均有闭式）运行于 GPU，在 70,000 母线系统上展示了**热启动跟踪能力：13k 母线 <1 秒、70k 母线 <30 秒**[^198^]。更大尺度上，ProxAL 以近端增广拉格朗日 + 纯 Jacobi 更新分解多时段/多场景耦合（$N$ 可达数千），MPI 分发到多节点多 GPU，形成"ProxAL 外层分解 + ExaADMM/ExaTron 内层求解"的两级并行[^180^]；同一框架已被推广到机组组合（UC-ACOPF），其 UC 子问题以动态规划在 GPU 上批处理[^201^]。

必须如实记录该路线的精度边界：ADMM 类方法在 AC-OPF 上**无法可靠收敛到 $10^{-3}$ 以下**[^143^]，其定位是快速近似解与热启动提供者，而非高精度替代——这与 ML 代理模型的角色相似，构成"近似初筛 + 精确复核"分工中的前端。

### 5.4 零空间/降维路线及其他变体

第四条路线直接消去等式约束：以零空间基 $Z$（$\mathrm{null}(A)$）把 KKT 系统降到约化空间，得到**稠密的小型约化 Hessian 系统**，天然映射到 GPU 的稠密线性代数[^143^]。面向实时校正 OPF 的可行降空间法（feasible reduced-space method）即属此类[^190^]。此外，锥优化领域出现了同源设计：QOCO-GPU 以凝缩 KKT + cuDSS 做 LDLᵀ 分解，报告了 50–70 倍加速[^13^]；MadIPM 将 Mehrotra 预估—校正 IPM 全 GPU 化，在大规模 LP 上已可与 Gurobi 竞争[^200^][^179^]；面向病态/退化 NLP 的 MadNCL（增广拉格朗日外套 IPM）在 $10^{-8}$ 紧容差下反超凝缩路线[^177^][^193^]。这些共同表明：**"凝缩/分解/降维三件套"正从 OPF 特例演变为 GPU 优化求解器的通用设计语言**。

![图D3 “重构问题结构适配硬件”方法图谱](images2/d3_methodology_map.png)

---

## 6 支撑栈：SIMD 自动微分、全 GPU 驻留与 MadSuite 生态

求解器只是冰山一角。Shin–Pacaud–Anitescu 的 GPU-OPF 论文之所以成为范式性工作，在于它同时解决了另外两个常被忽视的瓶颈：**自动微分（AD）与数据驻留**[^75^]。

### 6.1 AD 是被低估的半壁江山

在 AC-OPF 这类问题上，使用现成 AD（AMPL、JuMP）时**导数求值常占求解总时间一半以上**[^75^]。原因是一般 AD 框架无法自动发现模型方程中的可并行重复结构。ExaModels.jl 的对策是 **SIMD 抽象**：要求用户以"指令 + 数据索引集"的生成器（Generator）形式声明模型，

$$
\min_{x^\flat\le x\le x^\sharp}\;\sum_{l\in[L]}\sum_{i\in[I_l]} f^{(l)}(x;p^{(l)}_i)\quad
\text{s.t.}\quad \big[g^{(m)}(x;q_j)\big]_{j\in[J_m]}+\sum_{n,k} h^{(n)}(x;s^{(n)}_k)=0, \tag{6.1}
$$

使目标与约束天然落在三种 GPU 高效计算模式上（独立映射 map、幺半归约 mapreduce、可交换复合）[^75^]。ACOPF 只需约 **15 种计算模式**即可完整表达，且模式数不随电网规模增长[^75^]；于是（i）导数核函数可按模式编译、按数据并行发射；（ii）稀疏模式分析对每种指令做一次再按索引展开，免去对百万级表达式逐一分析。效果：**GPU 导数求值相对 CPU 实现 >10 倍，相对 AMPL/JuMP 约 500 倍**[^75^]。这一设计的本质是"以代数建模层的结构保留换取编译期的并行性可知"，后被 ExaModelsPower.jl 继承为电力专用建模库（极坐标/直角坐标两套 ACOPF、含储能的多时段 MPOPF）[^193^]。

### 6.2 全 GPU 驻留原则

零散的 GPU 卸载会被主机—设备往返传输拖垮；GPU-OPF 框架的原则是**问题数据与求解器中间数据全部驻留设备内存，绝大多数操作在 GPU 上闭环**[^75^]。为此 MadNLP 把 IPM 主体（滤波线搜索、二阶校正、恢复相、自动缩放——与 Ipopt 数学等价）实现为对 KKT 数据结构抽象的 Julia 高层代码，底层按 `SparseKKTSystem`/`DenseKKTSystem`/`DenseCondensedKKTSystem`/`SparseCondensedKKTSystem` 四种类型分派到主机或设备核函数[^75^]；KKT 组装、残差范数、凝缩组装、线搜索等全部以 KernelAbstractions.jl 写成设备无关内核[^200^]。最终形态是 **ExaModels（GPU AD）→ MadNLP（GPU IPM）→ cuDSS（GPU 稀疏 Cholesky/LDLᵀ）** 的全 GPU 驻留栈[^143^]。

### 6.3 MadSuite 生态与 2025 现状

该栈已产品化为 **MadSuite**（madsuite.org）：ExaModels.jl（建模/AD）、MadNLP.jl（滤波线搜索 IPM）、MadIPM.jl（Mehrotra 预估—校正 LP/QP，大规模 LP 上可比肩 Gurobi）、MadNCL.jl（增广拉格朗日 + IPM，病态/退化 NLP 与 $10^{-8}$ 紧容差）、ExaModelsPower.jl（电力建模）、CUDSS.jl（稀疏直接法绑定）[^199^][^200^]。ExaModelsPower 的 2025 年基准给出了迄今最完整的画像[^193^]：

- 静态 OPF：**大规模算例上 GPU 全面最快**；中等容差下 MadNLP+LiftedKKT 最优，紧容差 $10^{-8}$ 下 MadNCL 更优；小算例 CPU 仍占优（GPU 固定开销）；
- 多时段 OPF：GPU 相对 Ipopt+MA27 在可解的最大算例上取得**接近两个数量级**加速，含储能的大算例**只有 GPU 能在时限内求解**；
- 已知短板：紧容差下 GPU 求解器失败率上升；LiftedKKT/MadNCL 的约束违反量偏大[^193^]。

同期，二阶 LP/NLP 求解器 GPU 实现的整体方法论（凝缩 KKT、cuDSS 集成、与 cuPDLP/ADMM 等一阶路线的分工）已被系统化总结[^179^]；NVIDIA 与 ANL 的合作案例则把 cuDSS+MadNLP 定位为"消除美国电网优化障碍"的工业路径[^179^]。

### 6.4 复现路径（工程入口）

- **快速体验**（Julia）：`ExaModelsPower.jl + MadNLP.jl + MadNLPGPU.CUDSSSolver`，或 JuMP 层 `Model(MadIPM.Optimizer)` + `set_optimizer_attribute(model, "array_type", CuVector{Float64})`[^200^]；
- **论文基准**：GPU-OPF 复现脚本 `github.com/sshin23/opf-on-gpu`（PGLIB 的 goc/pegase 算例，对照 Ipopt+AMPL/JuMP+MA27 四配置）[^75^]；
- **SCOPF**：LLNL 的 HiOp（开源，含两阶段分解引擎）[^189^]与 PNNL 的 ExaGO（SCOPFLOW/SOPFLOW/TCOPFLOW 建模层）[^1^]。

---

## 7 参考实现与数值实验

为使上述理论可触可验，本报告附带一个 **NumPy 教学参考实现**（`condensed_ipm_demo.py`，约 600 行，仅依赖 NumPy/SciPy/Matplotlib），实现了与 MadNLP+LiftedKKT **数学等价**的完整流程，并内置 HyKKT 与全空间求解器用于三路对照。GPU 版本与本实现的差别仅在核函数层面（稠密 Cholesky ↔ cuDSS 稀疏 Cholesky；NumPy 向量运算 ↔ GPU 内核），数学流程逐行对应。

### 7.1 模块结构

```
NLP 数据结构          f, ∇f, g, A=∇g, W=∇²xxL, 界 [x♭,x♯]
feasibility_phase()   Phase-0 可行性恢复（LM 法 min‖g(x)‖）
condensed_ipm()       LiftedKKT 主求解器：
                      松弛(3.1) → 障碍KKT(3.3) → 全空间(3n+4m)
                      → 增广(3.4) → 凝缩(3.5) Cholesky
                      → 回代(3.6) → Richardson 精化(作用于增广系统)
                      → fraction-to-boundary + 回溯线搜索/恢复相
fullspace_ipm()       全空间对照（mode="full": 增广直接解, 模拟 LBLᵀ 角色;
                      mode="hykkt": K_γ Cholesky + Schur 补 CG）
hykkt_step(), cg()    HyKKT 单步与无预条件 CG
build_dcopf()         5 节点 DC-OPF（凸 QP，13 变量 11 等式）
build_hs071()         HS071 非凸标准题（Ipopt 经典测试，f*≈17.014）
```

核心凝缩例程（与 (3.5)(3.6) 一一对应）：

```python
# 凝缩: M_cond = K + A^T Σs A, 右端 = bx + A^T bs + A^T (Σs * by)
M_cond = K + A.T @ (Sig_s[:, None] * A)          # 唯一需要分解的矩阵
L = cholesky(M_cond + delta_w * I)               # 惯性修正: 失败则 δw 增大重试

def cond_solve(rhs):                              # 用同一因子回代增广系统
    dx = solve(L.T, solve(L, rhs[0] + A.T @ rhs[1] + A.T @ (Sig_s * rhs[2])))
    ds = A @ dx - rhs[2]                          # (3.6): Δs 回代
    dy = rhs[1] - Sig_s * ds                      # (3.6): Δy 回代
    return dx, ds, dy

# Richardson 精化: 残差在增广系统上计算（3.4 节保护机制之③）
for _ in range(n_refine):
    r = b_aug - M_aug @ d
    d += np.concatenate(cond_solve((r[:n], r[n:n+m], r[n+m:])))
```

### 7.2 正确性验证

**表 4  参考实现验证结果**

| 测试 | 结果 | 参照 | 结论 |
|---|---|---|---|
| DC-OPF（凝缩, tol=1e-4） | 8 次 IPM 迭代，f = 9.870215 | SciPy SLSQP f = 9.871111 | 偏差 9×10⁻⁴ ≈ 松弛宽度 τ=10⁻⁴，与理论一致 |
| DC-OPF（全空间 / HyKKT, tol=1e-8） | 各 12 次迭代，f = 9.871111 | 同上 | 与 SLSQP 完全一致；HyKKT 平均 CG 1.8 次 |
| HS071 非凸（凝缩, tol=1e-6） | f = 17.0140150，x\*=[1, 4.743, 3.821, 1.379, 25] | 文献 f\* = 17.0140174 | 非凸问题收敛到已知全局最优（误差 2×10⁻⁶） |

教学实现与工业求解器的差距（诚实声明）：未稀疏化、无二阶校正与完整滤波器，故 HS071 需 232 次迭代（Ipopt 约 10 次）；凝缩路线对非凸通用 NLP 的鲁棒性折损与文献在 CUTEst 上的观察一致[^143^]。

### 7.3 实验 E1：凝缩的病态代价（图E1）

在 DC-OPF 的 IPM 轨迹上同时记录 $\kappa_2(M_{aug})$ 与 $\kappa_2(M_{cond})$。随 $\mu$ 从 1 降至 10⁻⁷ 量级，$\kappa(M_{aug})$ 升至约 10¹¹ 后回落（峰值对应 $\Sigma$ 发散初期），$\kappa(M_{cond})$ 全程高出 **1–2 个数量级**——直观印证 2.5 节 $\Theta(1/\mu)\to\Theta(1/\mu^2)$ 的病态放大。这是凝缩换取正定性（GPU 可分解性）所支付的价格。

![图E1 凝缩加剧病态](images2/e1_conditioning.png)

### 7.4 实验 E2：迭代精化的修复力（图E2）

同一轨迹上比较"Cholesky 直接回代"与"≤3 次 Richardson 精化"后的增广系统相对残差 $\|M_{aug}d-b\|/\|b\|$：直接回代残差随病态加剧升至 10⁻¹¹–10⁻¹⁰，而精化后**全程压在 10⁻¹⁶ 机器精度附近**，每次精化仅一次三角回代的代价。这定量验证了 3.4 节保护机制③：凝缩造成的精度损失是**可用廉价精化赎回**的，而非不可控误差。

![图E2 迭代精化](images2/e2_refinement.png)

### 7.5 实验 E3：HyKKT 的 $\gamma$ 权衡（图E3）

在收敛点附近的真实 KKT 系统上扫描 $\gamma\in[10^0,10^8]$：Schur 补 CG 迭代数从 7 单调降至 2（特征值向 $1/\gamma$ 聚集，4.2 节定理），同时 $\kappa(K_\gamma)$ 从 10⁵ 线性升至 10¹²。两条曲线的交叉正是 HyKKT 参数设计的全部内容：**$\gamma$ 取到 CG 足够快即可，不必更大**；文献实践 $\gamma=10^7$、CG<10 次[^192^]。

![图E3 HyKKT gamma 权衡](images2/e3_hykkt_gamma.png)

### 7.6 实验 E4：三条路线收敛轨迹（图E4）

同一 DC-OPF 上，全空间直接法（LDLᵀ 角色）、HyKKT、LiftedKKT 的 KKT 残差轨迹几乎重合（三者数学上等价，同为牛顿方向），差异只在**可抵达的精度终端**：凝缩路线在 10⁻⁴ 停机（设计精度），全空间/HyKKT 继续收敛至 10⁻⁸。该图浓缩了全篇的论点：**三条路线共享同一个 IPM 外壳，竞争发生在内层线性系统的"硬件可分解性"上**。

![图E4 三路收敛轨迹](images2/e4_convergence.png)

---

## 8 性能证据汇总

**表 5  凝缩/分解路线的代表性性能数据（按来源分组）**

| 基准/系统 | 配置 | 结果 | 来源 |
|---|---|---|---|
| 13659_pegase (AC-OPF) | Ipopt+MA27 vs HyKKT+cuDSS (A100, tol=1e-6) | 10.1s → 2.5s（线解 7.2s → 0.9s） | [^89^] |
| 19402_goc | 同上 | 36.9s → 4.3s（线解 31.7s → 1.9s） | [^89^] |
| 20758_epigrids | 同上 | 18.2s → 3.5s | [^89^] |
| 78484_epigrids | 同上 | **207.8s → 18.9s（约 11×）** | [^89^] |
| PGLIB + CUTEst 大算例 | MadNLP+ExaModels+cuDSS vs Ipopt | 最高 10×；稠密行可致性能波动 | [^143^] |
| AC-OPF 自动微分 | ExaModels GPU vs AMPL/JuMP (CPU) | 约 500×；相对自身 CPU 版 >10× | [^75^] |
| 最大 PGLIB 算例 | MadNLP+ExaModels GPU vs Ipopt+JuMP | 数量级加速（GPU vs 自身 CPU 4×） | [^75^] |
| 校正 SCOPF（初步） | MadNCL on GPU vs CPU | 收敛时约 10×；鲁棒性仍逊于 CPU | [^89^] |
| MPOPF（含储能） | ExaModelsPower GPU vs Ipopt+MA27 | 最大可解算例近 **100×**；大算例仅 GPU 可解 | [^193^] |
| 大规模 LP | MadIPM (GPU, cuDSS) vs Gurobi | 大规模 LP 上有竞争力 | [^200^] |
| 锥优化 | QOCO-GPU（凝缩 LDLᵀ + cuDSS） | 50–70× | [^13^] |
| 批量微型 NLP | ExaTron, 6 GPU vs 40 CPU 核（Summit） | 9–35×，随批量/GPU 数线性扩展 | [^197^] |
| AC-OPF 热启动跟踪 | ExaADMM | 13k 母线 <1s；70k 母线 <30s | [^198^] |
| SCOPF 穷举 | ExaGO+HiOp, Frontier 9,000 节点 | 10 万+ 事故 × 10 场景，20 分钟 | [^4^] |
| HiOp GPU 求解 | 线性代数压缩（项目期） | 约 100× | [^4^] |

读表须知：各结果测试平台、容差与基线各异，不可直接横向比较；但其**一致的方向性**是明确的——凡满足"大规模 + 稀疏图结构 + 多次重分解/批量"特征的电力优化负载，结构重构路线的 GPU 加速稳定落在 **10×–100×** 区间。

---

## 9 研究路线图：开放问题与切入点

### 9.1 理论开放问题

**T1. 结构化病态的完整刻画与利用。** 现有误差分析（Wright 1998 → Pacaud et al.[^146^]）证明凝缩解精度损失受控，但"残差位于良态子空间"的论证依赖活跃集识别与 LICQ；对退化/秩亏电网问题（病态雅可比、岛屿、零注入），保护机制何时失效缺乏完整刻画。切入点：把 HyKKT 的 $GU_S=O(\Xi)$ 型估计推广到松弛走廊边缘情形，给出 $\tau$、$\gamma$ 的可证选择规则。

**T2. 凝缩 Cholesky 的多 GPU/分布式扩展。** 单 GPU 凝缩 Cholesky 已成熟，但 SCOPF 级 KKT（数千万变量）需要多 GPU 分解；3D 通信规避算法（SuperLU_DIST 9.x 的 GPU 化路径[^139^]）能否移植到凝缩 SPD 阵并保持"一次符号分解、千次重分解"的摊销结构，是直接的系统工程问题。

**T3. 稠密行/稠密列的混合凝缩策略。** CUTEst 上"单个稠密行毁掉凝缩阵"的失败模式[^143^]呼唤自适应方案：对检测到的高耦合约束局部切换 Schur 补/增广处理。这在电力域同样相关（联络线汇聚约束、HVDC、FACTS 的稠密耦合）。

**T4. 混合精度凝缩求解。** 以 FP32/FP16 完成凝缩 Cholesky、以全系统 Richardson 精化恢复 FP64 精度——结构化病态理论（精度损失可控）恰好为低精度分解提供了理论背书；HPL-MxP 的稠密经验[^49^]向稀疏凝缩场景的迁移几乎空白。

**T5. 分解—稠密化的最优粒度理论。** HiOp 的稠密化压缩"以更差渐近复杂度换硬件利用率"[^5^]，但压缩到多小为最优？建立"稀疏模式 × GPU 占用率 × 分解复杂度"的量化模型，可指导任意块角结构问题（多时段 OPF、随机规划）的自动分块。

### 9.2 EE×CS 系统研究方向

**S1. SCOPF 的凝缩化。** 现有 GPU-OPF 成果集中于单快照 AC-OPF；把 LiftedKKT/HyKKT 推广到 (5.1) 的块角结构（PIPS 式 Schur 分解 + 凝缩内层[^186^]），结合 MadNCL 对校正 SCOPF 的初步成功[^89^][^177^]，是通向"单节点 SCOPF"的直接路线。

**S2. 批处理 IPM。** MadSuite 团队已把批量求解列为下一目标[^200^]；把 ExaTron 的批处理思想（逐 warp 小稠密分解）与凝缩 IPM 结合，服务海量场景初筛，可与 ML 代理形成"代理热启动 → 批量 IPM 复核"流水线。

**S3. 市场出清与安全约束经济调度（SCED/SCUC）的 GPU 化。** LP/QP 内点法 GPU 化（MadIPM[^200^]、QOCO-GPU[^13^]）已就绪，把节点电价出清、SCED 这类"结构更规则但时效更苛刻"的问题移植到 GPU 栈，工业价值直接。

**S4. 动态安全与多时段耦合。** MPOPF+储能的 GPU 结果（近 100×[^193^]）提示暂态稳定约束 OPF、模型预测控制式滚动调度是下一个规模增长点；GPU-NMPC 的全栈实现（ExaModels+MadNLP，HyKKT 平均 CG<10 次[^192^]）已给出模板。

**S5. 国产软硬件栈适配。** 方法论（凝缩/分解/稠密化）本身与供应商无关；在国产 GPU/NPU 上复现 cuDSS 等价的稀疏 Cholesky/LDLᵀ（参考 NICSLU 系列在 CPU 上的并行 LU 经验[^72^]），并与 CloudPSS/ADPSS 等国产仿真体系对接，兼具科学与工程价值。

### 9.3 建议的入门路径

1. **理论**：按本报告第 2–4 章推一遍三层消去与两条凝缩路线；精读三篇原始文献[^75^][^143^][^24^]；
2. **动手**：跑通附带的 `condensed_ipm_demo.py`（复现 E1–E4），再用 Julia 栈在 PGLIB 上复现 10× 加速[^75^][^200^]；
3. **扩展**：选一个 9.1/9.2 的问题（推荐 T4 混合精度或 S1 SCOPF 凝缩化），以小算例建立基线再放大。

---

## 10 结语

"改变问题结构优于等待求解器进步"之所以成为 GPU 时代电力优化的方法论内核，是因为它把矛盾从**硬件不擅长的算法步骤**（不定稀疏分解的数值主元）转移到**硬件擅长的形态**（SPD Cholesky、小稠密批处理、SpMV 迭代），并用两层理论保证了这笔交易的安全性：**Sylvester 惯性律**保证凝缩阵正定可无主元分解，**结构化病态理论**保证平方级病态下的精度损失可控且可用廉价迭代精化赎回。LiftedKKT 与 HyKKT 是"凝缩"的两种实现，ExaSGD 的两阶段分解与 ExaADMM 的拉氏批处理是同一哲学在算法结构层的投影，而 ExaSGD"先稠密化绕开、后稀疏化回归"的螺旋则提醒我们：结构重构不是终局，而是算法与硬件能力边界上的持续动态适配。对 EE×CS 研究者而言，这一领域的迷人之处恰在于：**电网的图结构、内点法的数值分析、GPU 的体系结构三者在同一组公式里相遇**——理解这组公式的人，将定义下一代电网优化软件的形态。

---

*本报告所有性能数据均引自公开文献/官方资料（标注于文内），测试条件各异，横向比较请注意基线差异；附带的 `condensed_ipm_demo.py` 为教学参考实现，可在本目录直接运行复现第 7 章全部实验。报告仅供研究参考。*
