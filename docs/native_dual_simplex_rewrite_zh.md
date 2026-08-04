# Native Revised Dual Simplex 重写算法决策

> 仅限开发：生产模块默认使用 `LpKernelBackend::HiGHS`。运行本文的原生内核测试与
> 基准时必须显式选择 `ExperimentalNative`；生产路径不存在 HiGHS 到 native 的回退。

> 状态：理论设计冻结；旧 native driver 已退出入口，cold revised-primal
> Phase I/II、warm revised-dual、受检 Farkas 证书、exact-breakpoint BFRT
> 和稀疏 Forrest-Tomlin basis update 已启用。HySim 3655 contingencies 已在
> 无 LP fallback 条件下通过；同机 100x 性能门槛仍待完成。
> 英文形式化契约见 `docs/native_dual_simplex_rewrite.md`。
> major/minor 稳定化闭环见 `docs/native_dual_stabilization_zh.md`；该闭环现为
> 完成重写的强制组成部分。

## 目标

彻底替换旧 `dual_simplex.cpp` 中的 warm reopt、rollback、pivot blacklist、
crash-repair retry、cold fallback 和多套重复状态。HiGHS 只作为算法参考与
差分测试 oracle，不能作为 native LP 的运行时 fallback。

最终验收包括：全部 LP/MILP/Netlib 测试、HySim 可靠性测试，以及
`ACTIVSg2000 / 2000 AC + 8 DC` 的 3655 个 FMEA contingencies；过程中不得
出现旧 `[PATH A]`、`Simplex Phase I failed` 或 warm LP 的 cold recovery。

## 统一数学模型

内部问题固定为：

```text
max c^T x
s.t. A x = b
     0 <= x_j <= u_j
```

`u_j` 可以为无穷。artificial 和零宽变量的有效界为 `[0,0]`。

非基本变量方向：

```text
d_j = +1：位于下界，可以向上移动
d_j = -1：位于有限上界，可以向下移动
d_j =  0：固定或禁止入基
```

给定基 `B=A[:,beta]`：

```text
x_B = B^-1 (b-Nx_N)
y   = B^-T c_B
r   = c-A^T y
```

最大化约定下的 dual feasibility 是 `d_j r_j <= tau_d`。所有模块只能使用
这一套符号；最小化符号转换只允许出现在公共 API 边界。

## 单次 revised-dual pivot 推导

设第 `p` 个基本变量违反边界：

```text
Delta_p = x_B[p] - violated_bound
s       = sign(Delta_p)  // 低于下界为 -1，高于上界为 +1
pi      = B^-T e_p
a_bar_j = pi^T A_j
```

CHUZC 候选必须满足：

```text
alpha_j = s d_j a_bar_j > 0
mu_j    = -d_j r_j >= 0
theta_j = mu_j / alpha_j
```

dual step 为：

```text
y'       = y-s theta pi
r'_j     = r_j+s theta a_bar_j
d_j r'_j = -mu_j+theta alpha_j
```

因此，进入列必须满足 `r'_q=0`；所有 breakpoint 小于最终 `theta` 的有限
范围变量必须翻转 bound side。离基变量落到被违反的边界后自动满足
`d_leave r'_leave=-theta<=0`。

浮点环境下不使用固定“小 pivot”经验阈值。定价点积同时计算误差界
`eta_j`，只有 `s d_j a_bar_j-eta_j>0` 才能确认候选符号；无法确认时 fresh
factorization 后重新定价，仍无法确认则返回 numerical status。

行定价与列 FTRAN 得到的 pivot 不使用固定 `1024 eps` 比较。令
`r_v=A_q-Bv`、`r_pi=e_p-B^T pi`，则有精确后验恒等式

```text
pi^T A_q-e_p^T v = pi^T r_v-r_pi^T v.
```

实现用扩展精度计算两个实际 solve residual，并只在该恒等式给出的误差包络包含
行列差异时接受。这个证据随实际求解误差缩放，不放宽 feasibility 或 optimality
阈值。

## 算法分层

### 第一层：正确性核心

1. fresh rank-revealing sparse factorization；
2. FTRAN/BTRAN 必须通过真实 `B` 的 backward-error 检查；
3. 每次 pivot 后 exact reconstruction；
4. CHUZR 使用 DSE merit，并对被选中行重新计算精确权重；
5. CHUZC 的正确性核心使用 Harris two-pass，生产路径与 BFRT prefix 组合；
6. 每次 pivot 前后强制检查 `Ax=b`、basic reduced cost 和 dual feasibility；
7. 没有证书不得返回 infeasible/unbounded，任何阶段不得发布伪 Optimal。

### DSE 的推导与状态所有权

设第 `p` 个基列由进入列替换，`v=B^-1 A_q`、`alpha=v_p`。则
`B'=BE`，其中 `E` 的第 `p` 列为 `v`，从而 `B'^-1=E^-1B^-1`。若
`pi_i^T=e_i^TB^-1`，有

```text
pi'_p = pi_p / alpha
pi'_i = pi_i - (v_i/alpha) pi_p,  i != p
```

因此令 `w_i=||pi_i||_2^2`、
`rho_i=pi_i^T pi_p=(B^-1B^-T)_(i,p)`，得到精确的
Goldfarb-Reid 递推：

```text
w'_p = w_p / alpha^2
w'_i = w_i - 2(v_i/alpha)rho_i + (v_i/alpha)^2 w_p
```

三项递推发生严重相消时，负值或不可认证的小值不是 LP failure。实现对该行执行
`pi_i=B^-T e_i`，再以扩展精度直接计算
`||pi_i-(v_i/alpha)pi_p||_2^2`。这是交换后 DSE weight 的定义值，不是截断、
放宽阈值或启发式替代。

DSE cache 只归 revised-dual 阶段所有。Cold Phase I/II 使用
revised-primal pricing，不读取行 edge weight；在 primal pivot 中维护 DSE
既多出一次 FTRAN 和一次 `O(m)` 递推，又会把消去相消造成的 cache 误差错误地
升级为求解失败。primal driver 仍计算 Forrest-Tomlin 基交换所必需的 BTRAN，但不
数值维护 `edge_weight`；一旦基交换就使 cache 失效，导出的 basis 也不携带该
cache。warm revised-dual 入口只允许复用 basis-matched DSE cache，或按稳定化契约
建立 Devex reference framework，因此不存在把 primal 阶段陈旧 cache 带入
CHUZR 的路径。

### 第二层：BFRT

BFRT 作为独立 transaction，不在 Harris 中扩大阈值实现。初版学习 HiGHS 的
完整分组和 BFRT FTRAN，但不照搬 10 倍扩组及“向前回扫大 pivot”启发式。

对每个 error-certified 候选定义 `sigma_j=d_j r_j`、精确断点
`t_j=max(0,-sigma_j)/alpha_j` 和有限跨度 `range_j=upper_j-lower_j`，按
`(t_j,column_j)` 排序；只有浮点值完全相同的断点属于同一组，不用容差合并。
若 `F` 是所选组之前完整组中的有限跨度变量，翻转后对偶可行的充要区间是：

```text
max_(j in F) (-tau_d-sigma_j)/alpha_j <= theta
theta <= min_(j not in F) ( tau_d-sigma_j)/alpha_j.
```

从所选精确断点组取 `theta=t_q`，则已翻转变量的断点都不大于 `theta`，未翻转
变量的断点都不小于 `theta`，所以区间由构造保证。实现仍显式检查该区间；检查
失败直接返回 numerical status，不试另一个 pivot。

边界翻转 `Delta x_j=d_j range_j` 导致
`Delta x_B=-B^-1 A_j Delta x_j`，其 leaving 分量为
`-s alpha_j range_j`。因此算法为：

1. 按精确断点形成完整有序分组；
2. 按 `sum alpha_j range_j` 累积可用 primal change；
3. 遇到首个含无限跨度候选的组时必须在该组进入；否则选择第一个加入后可覆盖
   leaving violation 的有限组；有限跨度总量不足时不选伪 pivot，转交行区间证书；
4. Harris 第一遍用全部剩余的正 signed coefficient 计算
   `(tau_d-sigma_j)/alpha_j` 上界，包括小到不能进入的系数；第二遍从断点不超过
   该上界的 error-certified 候选中选择最大稳定 pivot；
5. 翻转所选组之前完整组中的所有有限跨度变量；
6. 构造 `sum A_j d_j range_j` 并执行一次 BFRT FTRAN；
7. 物化完整 post-pivot vector；若增量 residual 达到固定 `Ax=b` 门限，则应用捕获
   的 FT update；否则只提交已验证的基交换，立即对新基 INVERT 和 reconstruction，
   然后才能继续 pricing；
8. analytical dual interval、逐列 dual postcondition 和 post-pivot audit
   全部通过后提交；终止发布前再做一次 exact reconstruction 与完整 audit。

防循环 signature 包含有序 basis 与全部 nonbasic bound side。状态重复时，对已
记录的 outgoing exchange 施加有限寿命 taboo；若某 leaving row 的全部
Harris-eligible exchange 都是 taboo，则 CHUZR taboo 该 row；只有全部 infeasible
row 都被 taboo 时才释放最早到期的 row。算法不会从失败 row 搜索备选 pivot。

循环入口的受检状态是归纳不变量。每个 transaction 都物化完整候选 primal
vector，检查全部候选 nonbasic reduced-cost signs，并把新 basis 的 basic reduced
costs 按定义置零；随后或者提交受检 FT transition，或者提交基交换并立即执行新基
INVERT/reconstruction。超过固定 `Ax=b` 门限的增量 vector 永远不会进入下一轮。
发布前仍执行 exact reconstruction 与完整 audit。

HiGHS 的扩组和向前回扫只有在单独证明及差分测试通过后才能加入。

### 第三层：稀疏 Forrest-Tomlin update

当前启用 HFactor 的稀疏 Forrest-Tomlin 表示，它与 `B'=BE_p(v)` 的
`E_p(v)^-1`/`E_p(v)^-T` product-form 算子代数等价。INVERT 会把 caller basis
position 置换到 pivot-row position，因此 wrapper 把 leaving position 与最终
FTRAN 向量映射到内部基坐标；最终 BTRAN 向量保持在物理约束行坐标，并在交换前
同步 HFactor 的 `basic_index`。

FT update 不能把最终 `B^-1 A_q` 与 `B^-T e_p` 重新打包。pivotal-column pack
必须在 L solve 之后、U solve 之前捕获；pivotal-row pack 必须在转置 U solve
之后、转置 L solve 之前捕获。旧 raw FT 从第二次交换起分叉的根因就是把最终
向量误当成这两个中间量。新 wrapper 在原 FTRAN/BTRAN 内同时捕获最终解和中间
pack，并以 factor generation 绑定。若 pivotal solve 需要 iterative refinement，
原始捕获的 pack 已失效；实现不使用该 pack，先提交已验证的交换，再由 driver 立即
对新基执行 INVERT 与完整 reconstruction。若物化后的增量 transaction 未达到原
canonical `Ax=b` 门限，也使用同一新基协议。超限状态不会进入 pricing，且不改变
basis decision、bound side、pivot、阈值、容差或迭代预算。

reconstruction 还显式闭合 factor backward stability 与 canonical primal contract
之间的差距：用扩展精度累加 `r=b-Ax`，仅在原门限要求时执行单调 iterative
defect correction `B Delta x_B=r`，每次更新后都按相同门限复核。只有 residual
无穷范数严格下降时才能继续；停滞仍是 numerical failure，不允许放宽 feasibility
tolerance。由于 `x_B=B^-1(b-A_N x_N)` 同时依赖 factor generation `B` 与
nonbasic side vector `s`，canonical audit 只有在 `(B,s)` 都不变时才能复用；
boxed-column side classification 即使没有 INVERT，也会使此前 audit 失效。

强制差分从非对角乱序基开始，连续执行 10 次交换；每一步都把 FT FTRAN/BTRAN
同时与 fresh HFactor 和 dense LU 比较，并覆盖同一 backend 连续 factorize。
单位基或对角基无法覆盖三种坐标空间及中间 pack 契约，因此不能作为充分证据。

native 私有 HFactor port 在 triangular solve、FT replay 和 LU fill cancellation
中不执行 HiGHS 的 `kHighsTiny` 稀疏化策略，只删除精确为零的值；嵌入式 HiGHS
本体保持不变。因为 native kernel 要对真实显式 `B` 验证 solve，静默丢弃非零项
会改变所表示的线性系统，不能靠增大 backward-error 阈值来合理化。

kernel 内部的 sparse matrix-vector product 使用确定性的串行 CSC 循环。目标 LP
约万列但只有数万非零，每次 PRICE/残差乘法都启动 OpenMP team 的调度成本高于
算术本身且引入抖动；串行 CSC 不改变任何求和项或 acceptance bound。

性能版每次迭代的目标工作量是：一次 sparse BTRAN、一次 sparse PRICE、一次
sparse FTRAN、可选的一次 BFRT FTRAN、一次 FT update；reinversion 由可测的
backward error、growth 和 fill 触发，不按失败重试次数触发。

## Cold Phase I 与策略选择

此前方案只复制 HiGHS 的 dual-phase-one bounds，然后复用 Phase-II dual pivot。
这是不成立的：HiGHS 还同时维护独立的 Phase-I dual objective、专用 ratio 逻辑、
cost perturbation/shift cleanup 与 rebuild 状态转换。脱离这些机制，phase-one
bounds 既不能保证 dual feasibility，辅助 bounds 下的 primal feasibility 也不能
推出原问题 dual feasibility。

correctness kernel 采用标准且确定的策略：

```text
具有合法 dual-feasible bound side 的 warm basis
    -> revised dual Phase II

cold logical basis
    -> 可证明的 singleton crash 替换
    -> 精确的 bound-side dual-feasibility 分类
       -> 成立：revised dual Phase II
       -> 对该 basis 不可能：revised primal artificial-objective Phase I，
          再进入原 cost 下的 revised primal Phase II
```

对固定 basis，这个分类是必要充分的：boxed nonbasic 可以按 reduced-cost 符号
选择兼容的端点；lower-only nonbasic 没有可选上界，因此正 reduced cost 正好是
唯一阻碍。分类发生在任何 pivot 之前，不是一次试运行；进入 revised-dual 分支
后的失败直接终止，绝不回退到 primal Phase I。

primal Phase I 中，artificial bounds 为 `[0,+inf)`、maximization cost 为 `-1`；
其他变量保留原 bounds、cost 为零。标准形构造保证 RHS 非负，因此 logical basis
primal feasible。只有通过 primal/dual audit 的 Phase-I 最优点才能分类：

在两个 revised-primal 阶段内部，pivot 是基坐标的增量变换，不是重新求解整个
LP。设 entering move 为 `d`、步长为 `t`、`v=B^-1A_q`、
`z=B^-Te_p`、进入 reduced cost 为 `r_q`、`alpha=v_p`，则

```text
Delta x_q = d t
x'_B      = x_B - v Delta x_q，第 p 个坐标改为 x_q+Delta x_q
r'        = r - (r_q/alpha) A^T z，并令 r'_q 精确为 0
objective'= objective + r_q Delta x_q
```

第一式来自 `Bx_B+A_qx_q=b`；取 `y'=y+(r_q/alpha)z` 后进入列 reduced
cost 为零，得到第二式。bound flip 不改变 basis 和 dual vector，只应用 primal
与 objective 更新。下一次 invariant audit 仍使用原有 `Ax=b`、active bounds、
basic reduced-cost 阈值；没有放宽阈值或 pivot 规则。完整 reconstruction 只属于
phase transition 与 reinversion，不再是每次 primal pivot 的两次 solve。

```text
sum artificial = 0  -> 原 canonical system feasible
sum artificial > 0  -> 原 canonical system infeasible
```

Phase-I→II 恢复原 cost，将 artificial 固定为零，在同一 basis 上 exact
reconstruction，并重新验证 primal feasibility；任何辅助 objective 都不会发布。

singleton crash 不是尝试性搜索：只有某 original column 恰有一个非零、位于行
`i`，且 `b_i/a_ij` 落在原 bounds 内时，才替换行 `i` 的 logical。各替换是相互
独立的对角替换，因此按构造保持满秩；候选顺序完全确定。

无法由合法原 bound side 形成 dual feasibility 的 warm basis 返回
`DualInfeasibleStart`，不得丢弃后 cold retry。cold/warm 共享 basis ownership、
factorization、exact reconstruction、audit、iteration budget 与结果发布契约；
不存在旧 dense driver、Big-M、数值 escalation 或 native 内的 HiGHS fallback。

## 证书

CHUZC 无候选只是证书候选。令 `pi=B^-T e_p`、`h=pi^T b`、
`a_j=pi^T A_j`，计算 bounds 下 `sum a_j x_j` 的可达区间 `[L,U]`。只有
`h<L` 或 `h>U` 且分离 margin 超过累计浮点误差界，才能报告 primal
infeasible。证书保存 `pi`、区间端点、误差界和 margin。

Phase-I 非零 merit 本身不能直接发布 dual-infeasible/unbounded 结论。

## 代码边界

```text
native_dual/model.*          bounds、move、basis membership
native_dual/factor.*         rank repair、FTRAN/BTRAN、update 验证
native_dual/state.*          exact reconstruction、invariant audit
native_dual/pricing.*        CHUZR、PRICE、Harris、BFRT transaction
native_dual/primal.*         cold primal Phase I/II、Harris ratio test
native_dual/certificate.*    primal/dual certificates
native_dual/solver.*         phase controller、limits、result publication
```

旧 `sparse_dual_simplex_reoptimize`、rollback、blacklist、crash-repair retry、
重复 warm/cold orchestration 在替代实现通过准入测试后删除，不保留 fallback。

## 准入顺序

1. 单 pivot 代数测试：上下界离基、解析 update 与 exact reconstruction 一致；
2. Harris：退化、tie、lower-only、boxed、fixed、数值不确定 pivot；
3. Phase I：lower-only dual infeasibility、boxed side、artificial basic、phase transition；
4. 随机小 LP/warm basis 与 HiGHS 做 status/objective/KKT 差分；
5. 现有 dual simplex、LP、numerical、MILP、Netlib 测试；
6. BFRT 三组以上 breakpoint、精确 flip set 和 BFRT RHS；
7. HySim reliability case-system，旧路径完全禁用；
8. ACTIVSg2000 全部 3655 FMEA；
9. FT update chain 差分测试；
10. 同机同构建性能验收：相对旧 native 旗舰路径至少提升 100 倍。

任何一步失败都回到对应数学模块修正，不通过放宽阈值、增加预算或启用 fallback
绕过。
