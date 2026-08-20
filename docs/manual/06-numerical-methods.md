# 第 6 章 数值方法

> 本章整合自: docs/archive/numerical_methods.md, docs/archive/conic_sdp.md

本章面向需要理解求解器数值行为的高级用户与算法工程师：当你需要解读一次失败的迭代轨迹、调整正则化策略、或为新的内点内核选择分解后端时，这里给出可直接使用的约定、契约与判据。各节先讲"怎么用 / 看什么指标"，再讲背后的推导要点。建模与 API 用法见 [建模与 AML](04-modeling-aml.md) 与 [API 参考](07-api-reference.md)，求解器整体清单见 [求解器与引擎](05-solvers-engines.md)。

---

## 6.1 缩放（scaling）约定

### 6.1.1 输入规范化：单边约束行的翻号

`LPModel` 允许双边行 `lhs ≤ aᵀx ≤ b`。右端为 `+∞` 的单边行（`G` 行，`aᵀx ≥ lhs`）会直接毒化 IPM-LP 内核：松弛 `s = b − aᵀx = +∞`，障碍间隙与法方程对角 `θ = z_l/g_l + z_u/g_u` 相继变负，Cholesky 在非正定矩阵上"成功"，迭代点变成 NaN。

约定：**规范化只发生在求解器输入边**——把 `G` 行整体取负为等价的 `L` 行（`−aᵀx ≤ −lhs`），松弛恰好是剩余量 `aᵀx − lhs ≥ 0`；取解时把该行的 `constraint_duals` 符号还原。这一处理实现于 `solve_lp`、`prepare_for_node_solves`（缓存 B&C 路径），以及单纯形侧的 `build_standard_form_lp`。**不要在迭代内部对单边行做特例分支**：规范化一次，迭代保持均匀。

### 6.1.2 Ruiz 均衡（Ruiz equilibration）

当 `A` 的行/列量级跨越 `1e−10 … 1e10` 时，法矩阵 `N = A·Θ·Aᵀ` 在双精度下实际奇异。LP 在相似变换下不变：取正对角 `D_r, D_c`，解

```
min (D_c c)ᵀx̂   s.t.  (D_r A D_c) x̂ + ŝ = D_r b,  ŝ ≥ 0,
                       lb/D_c ≤ x̂ ≤ ub/D_c,
```

与原问题是同一个 LP，解的还原关系为 `x = D_c x̂`、`y = D_r ŷ`、`z = ẑ/D_c`。Ruiz 迭代交替做行、列 ∞-范数均衡（每轮乘以 `1/√‖·‖∞`），使每行/列的 ∞-范数几何收敛到 1。实现要点：**累积因子 `D_r, D_c` 每行/列只乘一次**——按非零元逐个乘会把 `D_r` 每轮平方，悄悄改变问题（这正是数值测试抓到的 Phase-1 bug）。

使用约定（用户视角）：

- 缩放**默认开启**，并带 **scaling fallback**：缩放后的求解不收敛时，以 rounds = 0 重试（实现于 `NativeIPMLPAdapter::solve_lp`）。
- 均衡是启发式预条件子，不是免费午餐：退化问题（如 NETLIB `stocfor1`）上激进的轮数会产生条件数更差的缩放系统（障碍参数 μ 冲到 `1e13+`）；而 `afiro` 不缩放会偏离最优 1%。**修改默认缩放前必须在 NETLIB 子集上同时检查两种形态**。
- OPF/IPM 路径上，均衡**每次求解计算一次并冻结**。逐迭代重算 `D` 会使有效 Newton 方向随 `D` 抖动、在病态 KKT 上极限环（IEEE24 混合 AC/DC 算例曾因此 640 迭代不收敛；冻结后 36 迭代收敛）。旧行为可用环境变量 `HACDCPF_OPF_RESCALE_PER_ITER=1` 恢复用于 A/B 对比。

---

## 6.2 KKT 系统组装：analyze-once / factorize-many 契约

内点法每次迭代都要组装 Newton 矩阵。朴素的三元组组装（`setFromTriplets`）代价是 `O(nnz·log nnz)` 外加反复分配，还会重做符号分解。关键观察：所有 Newton 矩阵都有形式

```
K(v) = P·diag(v)·Pᵀ + R
```

其中 `P`（Jacobian、松弛单位阵）**稀疏模式固定**，逐迭代变化的只有数值（障碍 `Θ`、正则化 `δ_W`、Hessian）。因此契约是：

1. **Analyze once**（每个问题一次）：fill-reducing 排序 + 符号分解；组装侧预计算 **scatter map**——每个源条目 `k` 在因子 CSC 值数组中的目标位置 `pos[k]`。
2. **Factorize many**（每次迭代）：数值按 `values[pos[k]] += v[k]` 直接累加，`O(nnz)`、零分配；符号分解复用，数值分解每迭代一次。

该契约已在 LP-IPM（banded/sparse 路径）、KKT 路径（`assemble_augmented_kkt_cached`）、增广 Newton 组装器以及锥规划 `KktAssembler`（见 §6.6）中实现。为新的 IPM 式内核写组装代码时请遵守同一契约。

**已知陷阱**：Eigen 的 `InnerIterator::index()` 是*行*索引而非存储偏移，不能当作 CSC 值数组下标用；直接用 `outerIndexPtr` 风格的 CSC 循环。值索引混淆是这类代码最常见的静默错误来源。

相关的内存纪律（同源问题）：长生命周期缓存（scatter map、符号分解、CHOLMOD analyze）每问题建一次、逐迭代复用；B&C 节点工作区共享并增量修改（`update_standard_form_bounds` 只应用变化的界），绝不默认逐节点整模型复制。

---

## 6.3 稀疏直接分解后端

按矩阵类型选择后端类：

- **Simplicial LDLᵀ**（Eigen、CHOLMOD simplicial）：逐列、BLAS-1，受内存延迟限制；小规模/极稀疏可用，规模化无望。在锥 KKT 的准定（quasidefinite）路径上另有专门用途，见 §6.6。
- **Supernodal**（CHOLMOD supernodal，本项目 SPD 默认后端）：把列聚成稠密面板用 BLAS-3（`dgemm`/`dsyrk`/`dtrsm`）分解；analyze-once 符号 + 逐迭代数值，无厂商锁定。
- **Multifrontal / Bunch–Kaufman LDLᵀ**（MUMPS、MA57、Pardiso 一类）：大规模*不定* KKT 系统（§6.5 的增广 Newton 形）的必需品类。MUMPS 是开源对应物，vendored 在树内（离线构建），同时接入内嵌 Ipopt 与 parity-IPM OPF 路径的 `MumpsSolver` 后端；Bunch–Kaufman 1×1/2×2 主元（阈值 `(1+√17)/8`）在不定系统上后向稳定。Pardiso/MA57 属同一算法类，但本仓库当前实际接线的开源后端是 MUMPS——不要假设存在 Pardiso 适配器。
- **Apple Accelerate** sparse Cholesky：Apple Silicon 上实测优于 CHOLMOD（法方程约 2.7×），macOS 上保持首选；其余平台优先级为 `Accelerate > CHOLMOD > Eigen`（实测序）。

**惯性是 LDLᵀ 的免费副产品。** `K = LDLᵀ` 是合同变换，由 Sylvester 惯性定律 `inertia(K) = inertia(D)`：负主元计数即负特征值数（MUMPS `INFOG(12)`，经 `MumpsSolver::negative_eigenvalues()` 暴露）。LU 类后端完全无法提供惯性——用它们时 IPM 只能盲正则化。对称不定分解带来的逐迭代免费惯性是一阶算法优势，不是实现细节；它是 §6.5 惯性校正的前提。

**后端选择的一个非对称性（重要）**：parity-IPM 的 OPF KKT 经 Ruiz 均衡 + `δ_W` 后是良态的，默认后端为 MUMPS；而*通用* IPM 的 `δ_C` 升级逻辑依赖分解器的**奇异标志**驱动——MUMPS 会吸收零/小主元（`CNTL(3)`、`ICNTL(24)`）而不报错，反而使该机制失效，所以 `make_default_sparse_solver()` 保持 `UMFPACK > KLU > …`，MUMPS 只在 KKT 已知良态处显式选用。这是算法性质的后果，不是偏好。

**KLU 固定模式数值重分解**。缓存型电力潮流 Newton 在首次完整 KLU 分解后，可对后续同模式 Jacobian 使用 `klu_refactor`。该路径固定首次主元顺序且不重新选主元，因此只在压缩列 `Ap/Ai` 逐项相同时启用；模式变化、奇异或固定主元失败均重新执行符号分析和完整分解。一次性 LE 和通用非缓存 NLE 不使用此优化。完整契约与成本模型见 [KLU numeric refactor 推导](../archive/klu_numeric_refactor_2026-08-20.md)。

**索引宽度**。2³¹ 的天花板不是维度 `n`（`n = 10⁶` 轻松装下），而是**因子非零数**：百万阶基矩阵的 `nnz(L)+nnz(U)` 可超过 `int` 上限并*静默*回绕。约定：分解接口先升到 64 位——UMFPACK 用 `umfpack_dl_*`、HiGHS 用 `HIGHSINT64`、CHOLMOD 用 `cholmod_l_*`；内部 CCS/CSR 缓存的*索引值*可保持 32 位（均 `< n`），但一切*计数*与接口数组必须 64 位，一切窄化转换必须显式守卫（响亮失败，不截断）。

---

## 6.4 迭代改进（iterative refinement）

分解 `Â ≈ A` 的前向误差满足 `‖x − x*‖ ≲ κ(A)·‖r‖`；`κ ~ 1e12` 时双精度每次回代丢约 4 位有效数字，Forrest–Tomlin 更新链与病态 KKT 会累积它。Wilkinson 改进：已算得 `x₀`，令 `r₀ = b − A x₀`，用**同一个**廉价分解解 `A δ = r₀`，置 `x₁ = x₀ + δ`，则每步误差乘以

```
q ≈ κ(A)·u   （u ≈ 2.2e−16 为机器精度）
```

`q < 1` 时逐步收敛到工作精度；每步代价为一次 SpMV 加一次回代，相对分解可忽略。

使用约定：

- **残差门控，而非常开**：先评估 `‖r‖∞ ≤ τ·max(1,‖b‖∞)`（τ ≈ 1e−12），只有残差需要时才付回代价。KKT 路径（≤2 步）、LP-IPM 法方程、UMFPACK 的 `IRSTEP=2` 都是这个策略。
- **发散守卫**：`q ≥ 1`（矩阵太病态）时改进不收敛，继续精化只会放大近零空间噪声。KKT 路径只在校正**严格减小原组装系统残差**时才提交它；若不存在减小残差的校正，应把它当作正则化问题处理（升级 `δ_W`/`dyn_reg`），而不是再要更多精化轮次。OPF 轨迹中曾实测到无守卫精化把方向放大到 `‖d‖ ~ 1e85`。
- 锥路径的精化同样不再是固定轮次：`ConicIPMOptions::refinement` 是**最大校正次数**，求解器在 NT 缩放的 3×3 坐标中计算分块后向误差，仅当其超过 inexact-Newton 收缩门槛（0.1）才回代，并拒绝不严格降低误差的校正。结果字段 `kkt_linear_solves`、`kkt_refinements`、`max_*_kkt_backward_error` 使该决策可审计。

---

## 6.5 惯性校正与正则化

### 6.5.1 凝聚形 vs 增广形 Newton 系统

原始-对偶 NLP IPM 把不等式乘子凝聚进 Hessian：`W = H + J_hᵀ·diag(μ/s)·J_h`，再解 `[W, J_gᵀ; J_g, 0]`。凝聚形的三个问题：每迭代一次稀疏三重积；`J_hᵀJ_h` 填充可达行数的平方；把 Jacobian 条件数**平方**。保留 `dμ` 为未知量则得到增广对称不定系统：

```
[ H + δ_W I   J_gᵀ     J_hᵀ ] [dx ]   [ −r_d                   ]
[ J_g        −δ_C I     0   ] [dλ ] = [ −r_eq                  ]
[ J_h          0      −SM⁻¹ ] [dμ ]   [ −r_ineq − M⁻¹(μ̄e − Sμ) ]
```

(3,3) 块是负对角 `−diag(s_i/μ_i)`，永不显式成形三重积。由块合同，

```
inertia(增广) = inertia(凝聚) + (0, m_ineq, 0)
```

即凝聚形上的下降条件 `inertia = (n, m_eq, 0)` 恰好对应增广形上的 `(n, m_eq + m_ineq, 0)`——同一套 Wächter–Biegler `δ_W` 校正原样适用。选择准则（`IPMOptions::use_augmented_newton`）：`J_h` 宽/稠密或 `nnz(J_hᵀJ_h) ≫ nnz(J_h)` 时用增广形；窄带状 `J_h`、额外的 `m_ineq` 维分解代价占主导时用凝聚形。OPF 实测：凝聚形在 stall 点 `κ(W) ≳ 1e10`，方向只剩 5–6 位有效数字（"解得准但方向错"——KKT 残差小、步毫无价值，任何线性求解器都救不了）；换增广形后 case1354pegase 由失败转为收敛，目标值与 Ipopt 吻合到 4e-6。

### 6.5.2 Wächter–Biegler δ_W 惯性校正环

简约 Hessian 正定时 `inertia(K_aug) = (n, m_eq + m_ineq, 0)`。LDLᵀ 分解免费报告负主元数（§6.3）；当 `negevals ≠ m_eq + m_ineq` 时简约 Hessian 不定，步不保证下降，于是升级 `δ_W` 并重新分解：

- 起步踢量 `1e-8·‖Lxx‖`，每次重试 ×8，上限 `1e-2·‖Lxx‖`。

这取代了旧的盲正则化阶梯（旧逻辑接受第一个"没失败"的分解，包括惯性错误的分解——这正是非下降步被接受的路径）。`δ_C`（等式块正则化）则按奇异*标志*升级，与上节的后端非对称性联动：走 MUMPS 的良态 KKT 用惯性驱动，走 UMFPACK/KLU 的通用路径用奇异标志驱动。

### 6.5.3 大规模案例的失败机理备忘（OPF 实测层级）

当求解在 OPF 规模实例上失败时，按以下顺序排查（三个渐近机制叠加，只修线性求解器总耗时移动不到 10%）：

1. **稠密热启动**：`build_initial_point` 的 DC 热启动若以稠密 `(nb−1)²` 矩阵解网络 Laplacian，在 10⁴ 母线规模占全程 94%（1.5 GB 内存、`O(n³)` flops）。`B'` 是近平面图 Laplacian，sparse LU + COLAMD 排序代价 `~O(n^{3/2})`。
2. **凝聚形病态**（可解性壁垒）：见 §6.5.1；换增广形 + `δ_W` 惯性环。
3. **分解后端类**：同一填充图上对称 LDLᵀ 的 flops 与填充约为非对称 LU 的一半；近平面图嵌套分割给出填充 `O(n log n)`、工作 `O(n^{3/2})`。case13659 实测 MUMPS 对 UMFPACK 在分解主导段约 1.75×，与 ~2× 对称性预测一致。

修复三者后仍不收敛的大规模案例，失败通常已移出线性代数、进入全局化（filter 拒绝、方向爆炸、接受准则被污染）；相应的 SOC（second-order correction）、re-centering、catastrophe guard 与可接受终止约定见 [求解器与引擎](05-solvers-engines.md)。

---

## 6.6 锥内点法专题（LP / SOCP / SDP）

原生锥规划内核 `ConicIPMSolver`（适配器 `NativeConicIPMAdapter`，名称 `"NativeConicIPM"`）以 CVXOPT `conelp` 为蓝本，在统一的 cvxopt 标准型下用 Nesterov–Todd 缩放的 Mehrotra 预测-校正原始-对偶 IPM 求解 LP / SOCP / SDP：

```
(P)  min cᵀx   s.t.  Gx + s = h,  Ax = b,  s ∈ K
(D)  max −hᵀz − bᵀy   s.t.  Gᵀz + Aᵀy + c = 0,  z ∈ K
```

锥笛卡尔积 `K = R^l_+ × ∏ Q^{q_i} × ∏ S^{s_j}_+` 由 `ConeDims{l, q, s}` 描述；可行点处对偶间隙恰为互补积 `sᵀz`，是算法实际跟踪的量。半定块用 svec 打包（列主序下三角、非对角乘 `√2`），打包空间的普通点积即矩阵 Frobenius 内积——这是 SDP Schur 元素能写成 `tr(X_i M X_j M)` 的原因。

### 6.6.1 Nesterov–Todd 方向

每步在缩放空间取 Newton 方向。NT 缩放为每对严格内部点 `(s, z)` 选自尺度变换，使原始点与对偶点映到同一缩放点 `λ = P s = D z`；Newton 矩阵中只出现组合算子 `H = P⁻¹D` 与其逆。逐锥要点：

- **非负象限**（及被折叠为象限行的尺寸-1 SOC 块）：`H = diag(s_i/z_i)`，对角。
- **二阶锥 `Q^k`**：记 Jordan 行列式 `det(u) = u₀² − ‖u₁‖²`、`ρ(u) = √det(u)`，归一化 `s̄ = s/ρ(s)`、`z̄ = z/ρ(z)`，取 `γ = √((1 + s̄ᵀz̄)/2)`、`w̄ = (s̄ + Jz̄)/(2γ)`、`β = √(ρ(s)/ρ(z))`，则闭式

  ```
  H = β²(2 w̄w̄ᵀ − J),        H⁻¹ = β⁻²(2(Jw̄)(Jw̄)ᵀ − J)
  ```

  两者都是 `J` 的低秩修正，作用一次 `O(k)`；`GᵀH⁻¹G` 的 SOC 贡献因而是"稀疏部分 `−β⁻²G_qᵀJG_q` + 秩 1 修正"。
- **半定锥 `S^p_+`**：由 `S = L_sL_sᵀ`、`Z = L_zL_zᵀ` 与 `L_zᵀL_s = UΣVᵀ` 构造合同矩阵 `R = L_sVΣ^{−1/2}`，核心恒等式 `RᵀZR = R⁻¹SR⁻ᵀ = Σ`——同一合同把 `S`、`Z` 同时对角化，缩放点为 `svec(diag(Σ))`。`H`/`H⁻¹` 是打包空间上的稠密线性算子，代码从不显式成形，以两次合同矩阵乘实现（`apply_h`: `X ↦ (RRᵀ)X(RRᵀ)`，`apply_h_inv`: `X ↦ MXM`，`M = (RRᵀ)⁻¹`）。

Mehrotra 主循环遵循 CVXOPT：仿射方向定 `σ = clamp((μ_aff/μ)³, 0, 1)`，再以组合右端 `r_λ = −λ − λ∖(dλ_s∘dλ_z) + σμλ⁻¹` 解同一分解，阻尼步 `α = min(1, 0.99·max_step)`。注意 `λ∖X := L(λ)⁻¹X` 是 **Jordan 除法**而非逐元素除——对非结合 Jordan 代数（SOC、半定）`L(λ)⁻¹ ≠ L(λ⁻¹)`，把校正项写成 `∘λ⁻¹` 是错误公式；半定块因 `Λ` 对角，除法逐元素为 `2X_ij/(σ_i + σ_j)`。

容差语义（`ConicIPMOptions` 默认值）：`abstol = 1e-7`、`reltol = 1e-6`、`feastol = 1e-7`、`max_iterations = 100`、`refinement = 1`（仅存在 q/s 块时启用，语义见 §6.4）。终止条件为归一化原始/对偶残差 ≤ `feastol` 且（`gap ≤ abstol` 或 `relgap ≤ reltol`）；不可行与无界各有归一化 Farkas 证书判据。

### 6.6.2 KKT 稀疏模式不变性与 Schur 装配

3×3 方向系统

```
[ 0   Aᵀ   Gᵀ ] [dx]   [ −r_c              ]
[ A   0    0  ] [dy] = [ −r_x              ]
[ G   0   −H  ] [dz]   [ −r_s − W·r_λ      ]
```

消去 `dz` 后得到每迭代实际求解的 2×2 系统

```
K [dx; dy] = rhs,   K = [ GᵀH⁻¹G + δI   Aᵀ ]
                        [ A            −δI ]
```

`δI`/`−δI` 是准定正则化。**`K` 的稀疏模式逐迭代不变**：`G`、`A` 固定，只有 `H⁻¹` 的数值与 `δ` 变化——`KktAssembler` 因此遵循 §6.2 的 analyze-once/factorize-many 契约：`build()` 每问题一次（生成下三角 CSC 模式并为象限外积、SOC 秩 1 项、SDP Schur 项预计算 scatter map）；`assemble()` 每迭代按缓存条目直接累加数值；`apply_delta()` 只动对角，`δ` 升级时免重装配。

`B = GᵀH⁻¹G` 按锥块装配：标量行为逐行稀疏外积 `Σ g_rg_rᵀ/d_r²`；SOC 块为稀疏 scatter 加团内秩 1；半定块为团内稠密 `(B_s)_ij = tr(X_i M X_j M)`，按块自适应选择稠密合同路径或超稀疏解析核。同一锥块触及的所有变量构成一个稠密团——这是 SDP 装配与分解成本的来源，也是弦分解（chordal decomposition）要拆的对象：聚合图稀疏的大阶 SDP 经 Agler 分解 `S(x) = Σ_k E_Ckᵀ S_k E_Ck` 加一致性等式**精确**改写为小团 PSD 块（门控参数 `chordal_decomposition`、`chordal_min_order=32`、`chordal_max_clique_ratio=0.75`、`chordal_max_expansion_ratio=2.0`，不通过门控的块保留原路径）。

### 6.6.3 锥 KKT 的分解后端链

- **无等式**（`m_eq = 0`）：`K = GᵀH⁻¹G + δI` 对称正定（三类锥的 `H⁻¹` 均为正定算子），走 CHOLMOD 超节点 LLᵀ 快路径；分解失败（近奇异）时 `δ` 按 ×10 升级重试：`10⁻⁹ → 10⁻⁸ → 10⁻⁷ → 10⁻⁶`。
- **有等式**：`K` 为准定鞍点系统，Vanderbei 定理保证其对任意对称排列存在无主元交换的 LDLᵀ（负主元全部落在等式块），惯性恰含 `m_eq` 个负特征值。此时强制 CHOLMOD **simplicial LDLᵀ**（超节点 LLᵀ 只接受 SPD，simplicial 原样接纳负 `D` 主元）。
- **不定回退**：simplicial 遇（近）零主元、`δ` 升到上限后**永久切换**到 MUMPS 对称不定 LDLᵀ（需编译期宏 `MIPSOLVERS_HAVE_MUMPS`）；`analyze_pattern` 一次、`factorize` 每迭代，惯性经 `MumpsSolver::negative_eigenvalues()` 免费可得，为 Wächter–Biegler 风格惯性控制预留接口。MUMPS 不可用时回退失败，求解以 `status = "unknown"` 退出——这是响亮失败，不是退化结果。

> 构建层已知坑（影响一切走 MUMPS 回退的模块）：brew 路径会把内置 5.7.3 头文件与 Homebrew Ipopt 自带的 libdmumps 5.6.2 链接，`DMUMPS_STRUC_C` 布局跨版本不一致，导致 MUMPS solve 返回错误解。规避：以 `-DMIPSOLVERS_FORCE_BUILD_MUMPS=ON` 重新配置（树内 5.7.3 源码构建）。

### 6.6.4 已知数值限制

- **超稀疏 `G` 的 SDP 病态**：`G` 列均仅约 1 个非零时 Schur 补 `GᵀH⁻¹G` 条件数极差，观测到 `p = 10` 个例中 gap 在 `~2×10⁻²` 处停滞并触锥边界发散。算法层未修复；基准构造因此对 SDP 块设了 ≥5 非零/列的下限。
- **无指数锥/幂锥**：需要新的自尺度障碍与 NT 缩放推导。
- 并行（OpenMP，`ConicIPMOptions::num_threads`）只作用于 NT 缩放、max_step 与 SDP Schur 按列装配，且所有并行循环写互不相交输出——**线程数不改变数值结果**（测试锁定到 1e-10）。

---

## 6.7 速查：数值问题排查路径

- **迭代变 NaN / Cholesky "成功"在非 PD 矩阵上** → 检查单边行是否在输入边规范化（§6.1.1）。
- **不收敛且 μ 爆炸** → 缩放形态问题：用 rounds = 0 的 fallback 对照（§6.1.2）；逐迭代重均衡的抖动用冻结 `D` 排除。
- **KKT 残差小但步无进展（"解得准方向错"）** → 系统本身病态：凝聚形换增广形，检查 `δ_W` 惯性环（§6.5）。
- **精化越修越坏** → 确认残差门控与发散守卫在位；`q = κ·u ≥ 1` 时转正则化（§6.4）。
- **后端行为费解（MUMPS 不报奇异 / 惯性读数异常）** → 后端与路径的匹配（§6.3、§6.5.2）及 MUMPS 头库版本（§6.6.3）。
- **锥求解 status = "unknown"** → 不定回退链末端失败（MUMPS 不可用或 ABI 不匹配），属响亮失败（§6.6.3）。

理论出处与文献清单见 [理论参考](11-theory-references.md)。
