# 最严苛评估：这是一个“方向正确但证明链过强、工程闭环过重、若不收缩范围很容易失控”的方案

整体判断：

> 该设计的核心思想——**低保真优化器给下界，高保真仿真给上界，ML 只做提案，最终由框架认证**——是非常好的，也比单纯 L2O、单纯 MILP 规划、单纯仿真校验都更严谨。  
> 但从最严苛角度看，当前文档中存在一个主要问题：**理论叙述的确定性和工程可实现性之间仍有较大张力**。  
> 特别是你把“仿真评估”“上下界认证”“有限终止”“DAgger/L2O 泛化”“高保真最优运行成本”放在了同一闭环里，这使得系统在论文层面很有吸引力，但在真实实现、验证和答辩审查中会遭遇多处硬质疑。

我会按以下维度评估：

1. **总体优点**
2. **最严重的理论风险**
3. **最严重的工程风险**
4. **ML/L2O 部分的严苛批评**
5. **接口设计风险**
6. **收敛与上下界认证风险**
7. **仿真 oracle 的可信性风险**
8. **建议如何收缩成一个可落地、可发表、可实现版本**
9. **最终结论**

---

# 1. 总体优点：核心架构是有价值的

先说优点。方案中最强的点有四个。

## 1.1 “ML proposes, framework certifies” 是正确的治理原则

你没有让 ML 直接决定最终方案，而是把 ML 限制为：

- warm-start；
- incumbent proposal；
- surrogate correction；
- scenario prioritization；
- cut ranking。

这非常重要。因为规划问题涉及：

- 安全；
- 电压约束；
- 可靠性；
- 韧性；
- 碳约束；
- 大额投资；

任何“神经网络直接输出规划方案”的方法，在工程上都很难被接受。你现在的 R1 原则：

> ML proposes, the framework certifies.

是该设计最正确、最应该保留的核心。

---

## 1.2 下界/上界的职责划分很清楚

设计中明确：

- **MIPSolvers / low-fidelity relaxation** 产生 LB；
- **hacdcpf / high-fidelity simulation oracle** 产生 UB；
- surrogate 不参与有效下界；
- L2O 不参与有效上界，除非经过 oracle 验证。

这个边界划分是严谨框架的基础。

如果这点被破坏，整个设计会退化成“启发式仿真优化”，很难宣称有理论保证。

---

## 1.3 将现有资产组合成闭环，工程复用价值高

你已经有：

- Julia 简化规划原型；
- C++ 高保真 `hacdcpf`；
- MIPSolvers；
- NN-as-MILP；
- reliability / resilience / carbon 模块。

这个方案不是凭空造系统，而是试图把已有模块组合成闭环。这是很大的优势。

---

## 1.4 接口契约意识很好

`simulation_based_planning_interfaces.md` 明确规定：

- `Plan`；
- `Scenario`；
- `Metrics`；
- `Verdict`；
- `ViolationCertificate`；
- `Operator`；
- `Oracle`；
- `RelaxationBuilder`；
- `Cut`；

这是严肃系统设计必须有的东西。尤其是：

- infeasibility 必须带 certificate；
- relaxation builder 必须打 tag；
- dual/ray 是 Benders 必需；
- oracle 必须 reproducible。

这些要求都非常专业。

---

# 2. 最大理论风险：你把“仿真 oracle”当成了“真实 recourse optimizer”

这是最核心、最危险的问题。

文档里多次把 `hacdcpf` 定义为高保真 oracle，并说：

> if feasible, metrics.op_cost is an achievable cost, hence gives a valid upper bound.

这一点用于 UB 是可以的。

但是你又在多个地方写：

- `Q_H(x,ω)` 是高保真 operation cost；
- `x^*(i)=argmin F_i(x)`；
- `R_i(x)=F_i(x)-F_i(x^*)`；
- `𝓛_DF` 使用 surrogate approximation of high-fidelity objective；
- DAgger expert 是 `𝓟` solved to certified optimality；
- simulator returns the best operation the simulator can find。

最严苛问题是：

> `hacdcpf` 到底是“高保真可行运行方案生成器”，还是“高保真全局最优 recourse solver”？

这两者差别极大。

如果 `hacdcpf` 只是仿真/启发式/局部优化：

- 它给出的可行运行成本可以作为某个可行方案的成本；
- 因此可作为 UB；
- 但它不等于 $$Q_H(x,\omega)$$ 的最优值；
- 那么你不能把它当成真实 recourse optimum；
- 也不能严格定义 regret；
- 也不能说 `x^*(i)` 是真最优；
- 也不能说 DAgger expert 是 certified optimum。

如果 `hacdcpf` 真能求解高保真 recourse 全局最优：

- 那它本身就是非线性、混合整数、多时段、可靠性、韧性问题；
- 其计算代价和全局性证明非常困难；
- 你必须说明用什么求解器、什么全局最优证书、什么容差。

当前文本在这点上有混用。

## 严苛建议

必须把 `Q_H` 定义分成两层：

### 层 A：真实但不可计算的理想高保真 recourse

$$
Q_H^{\mathrm{opt}}(x,\omega)
$$

表示真正高保真物理模型下的最优运行成本。

### 层 B：仿真器实际返回的可行成本

$$
\widehat Q_H(x,\omega)
$$

表示 `hacdcpf` 返回的可行运行成本。

只要 `hacdcpf` 可行，则：

$$
\widehat Q_H(x,\omega) \ge Q_H^{\mathrm{opt}}(x,\omega)
$$

因此：

$$
c^\top x + \sum_s p_s \widehat Q_H(x,\omega_s)
$$

是一个有效 UB。

但是不要轻易说：

$$
\widehat Q_H(x,\omega) = Q_H(x,\omega)
$$

除非你能证明 simulator 内部求了全局最优。

---

# 3. 第二大理论风险：LB 的有效性依赖于非常强的 relaxation invariant

你设计中最重要的理论条件是：

$$
\mathcal{Y}_H(x,\omega) \subseteq \mathcal{Y}_L(x,\omega)
$$

以及：

$$
Q_L(x,\omega) \le Q_H(x,\omega)
$$

这个条件非常强。

在电力系统中，想让 LinDistFlow / DC / 简化 converter model 成为严格 relaxation，并不容易。

## 3.1 线性化潮流未必天然是 relaxation

LinDistFlow 有时候是近似模型，不一定严格外包络 AC 可行域。

例如：

- 电压幅值平方近似；
- 忽略损耗；
- 角度差；
- 无功功率；
- converter 控制模式；
- thermal limit；
- phase imbalance；
- meshed topology；
- reverse power flow；
- SOP/VSC 耦合。

这些简化有可能：

- 放松真实约束；
- 也有可能意外收紧真实约束；
- 更常见的是既不是 relaxation，也不是 restriction，只是 approximation。

如果某个约束意外收紧，则：

$$
Q_L(x,\omega) \le Q_H(x,\omega)
$$

可能不成立，下界失效。

## 3.2 “tag exact / relaxation / dropped” 不足以证明 relaxation

接口里要求每个约束打 tag：

- exact；
- relaxation；
- dropped。

这有助于审计，但不是数学证明。

最严苛审稿人会问：

> 你如何证明每一个 `Relaxation` 约束确实是高保真可行域的必要条件，或者至少不排除任何高保真可行点？

仅靠 tag 不够。需要：

- 每类约束的 relaxation lemma；
- 参数范围；
- topology assumption；
- voltage range assumption；
- line model assumption；
- converter model assumption；
- loss treatment；
- load shedding treatment。

## 严苛建议

把 R2 从“builder tag discipline”升级为“relaxation certificate registry”。

每种约束模板需要一个证明条目：

| Constraint template | High-fidelity counterpart | Relaxation relation | Assumptions | Failure if violated |
|---|---|---|---|---|
| LinDistFlow voltage drop | AC branch flow | outer approximation | radial, balanced, small angle | not certified |
| thermal ampacity | AC current limit | relaxation or exact? | voltage lower bound | may tighten |
| converter capacity | PQ capability circle | polyhedral outer approximation | known rating | valid if outer |
| ESS SoC | same linear dynamics | exact | fixed efficiency model | exact |
| load shedding | same or relaxed | relaxation | VOLL finite | complete recourse |

并在理论中声明：

> 下界只对通过 relaxation certificate registry 证明的低保真模型成立。

---

# 4. 第三大风险：有限终止结论过强

你提出 finite termination 依赖：

- compact/integer-bounded first stage；
- finite critical set；
- no-good cuts；
- sampled finite scenario set；
- binary x。

这个思路可以成立，但当前规划变量包含大量连续容量：

- PV capacity；
- ESS power/energy；
- converter rating；
- SOP rating；
- wind capacity；
- shared ESS sizing。

如果存在连续变量，简单 no-good cut 无法有限枚举所有方案。

## 4.1 Binary no-good cut 不适用于连续容量

对纯二进制变量，可以排除一个离散计划：

$$
\sum_{j\in S}(1-x_j) + \sum_{j\notin S}x_j \ge 1
$$

但如果计划还有连续 sizing：

$$
C_{\mathrm{ess}} = 3.1729
$$

下一轮可以返回：

$$
C_{\mathrm{ess}} = 3.1730
$$

几乎相同的不可行计划，no-good cut 不能有效有限终止。

## 4.2 Integer-bounded 不等于连续容量离散化

文档有时说：

- `x ∈ ℤ^p × ℝ^q`；
- 也说 cheap lattice；
- 也说 binary no-good；
- 也说 finite convergence。

这三者必须统一。

如果要有限终止，你必须做其中之一：

### 方案 A：所有投资变量离散化

例如容量只能是 unit multiples：

$$
C_k = n_k \Delta_k,\quad n_k \in \mathbb{Z}_+
$$

这样第一阶段是有限集合，no-good cut 有意义。

### 方案 B：不宣称 finite termination，只宣称 ε-convergence / practical termination

对混合整数连续问题，可以给出：

- monotone LB；
- nonincreasing UB；
- termination when gap small；
- but no finite guarantee unless additional assumptions hold。

### 方案 C：结构性 cut 足够强

如果 oracle certificate 总能生成强结构性 cut，例如：

$$
\sum_{b\in Z} E_b^{\mathrm{ess}} \ge \tau_Z
$$

那可以处理连续变量，但必须证明这些 cut 有效且有限收敛，这很难。

## 严苛建议

当前版本应删除或弱化“finite termination”表述，改为：

> finite termination holds only under a fully discretized first-stage design space and finite scenario catalogue.  
> For mixed-integer continuous sizing, the loop is an anytime certification algorithm with valid LB/UB but no general finite termination guarantee beyond solver tolerances and imposed discretization.

这会更可信。

---

# 5. 第四大风险：UB 和 LB 可能不是同一个问题的上下界

你定义：

- LB 来自 sample set $$\Xi^k$$ 上的 low-fidelity relaxation；
- UB 来自 `hacdcpf` over $$\Xi^k$$ plus held-out audit set；
- scenario set 可不断扩大；
- resilience/reliability events 可能 sampled；
- SAA 被引用。

严苛问题：

> LB 和 UB 是针对同一个目标函数吗？

如果 LB 用的是当前 scenario set：

$$
\Xi^k
$$

而 UB 用的是更大的：

$$
\Xi_{\mathrm{full}}
$$

或 held-out audit set，那么它们不是同一 SAA 问题的上下界。

如果目标是原始期望：

$$
\mathbb{E}_{\omega\sim \mathbb{P}}[Q_H(x,\omega)]
$$

那么有限样本上的 LB/UB 也不是原问题的确定性上下界，而是 SAA 估计。

## 必须分清三种 gap

### 1. Optimization gap on current SAA

$$
UB_{\Xi} - LB_{\Xi}
$$

这是对固定样本集的优化 gap。

### 2. Generalization / sampling gap

$$
F(x;\mathbb{P}) - F(x;\Xi)
$$

这是样本外误差。

### 3. Fidelity gap

$$
Q_H - Q_L
$$

这是高低保真差异。

现在文档中这三种 gap 有时被合并成一个“UB-LB gap”，这在严格审查下会被质疑。

## 严苛建议

显式定义：

- **certified SAA gap**；
- **out-of-sample audit metrics**；
- **statistical confidence interval**；
- **fidelity certification gap**。

例如：

$$
\mathrm{Gap}_{\mathrm{SAA}} =
UB_{\Xi}^{H} - LB_{\Xi}^{L}
$$

而对真实分布只报告：

$$
\widehat F_{\mathrm{audit}}(x) \pm z_{\alpha}\frac{\hat\sigma}{\sqrt{N}}
$$

不能把 audit set 结果直接并入 deterministic UB/LB，除非 LB 也对应同一个 set。

---

# 6. 第五大风险：oracle infeasibility certificate 要求过高

接口规定：

> Every infeasibility from `𝓔` is accompanied by a machine-actionable certificate sufficient to construct at least a no-good cut.

这是理想，但在高保真 AC/DC 仿真中很难保证。

## 6.1 非线性不可行问题的 certificate 很难稳定生成

比如：

- AC power flow 不收敛；
- OPF solver local infeasible；
- voltage collapse；
- converter saturation；
- islanding；
- restoration MIP timeout；
- reliability Monte Carlo failure；
- numerical divergence。

这些情况不一定能产生清晰的 certificate。

可能只是：

- solver failed；
- local infeasible；
- max iterations exceeded；
- NLP restoration failed；
- nonconvex infeasible uncertain。

这无法直接变成有效 cut。

## 6.2 Indeterminate 状态未被算法充分处理

接口里有：

```cpp
enum class Status { Feasible, Infeasible, Indeterminate };
```

但理论闭环基本围绕 feasible/infeasible 展开。

严苛问题：

> 如果 oracle 返回 Indeterminate，算法怎么办？

不能：

- 把它当 infeasible，加 cut，可能误删真可行解；
- 把它当 feasible，可能得到假 UB；
- 忽略它，可能循环卡住。

## 严苛建议

增加明确三态逻辑：

| Oracle status | UB update | Cut allowed | Next action |
|---|---:|---:|---|
| Feasible | yes | no | update incumbent |
| Infeasible with valid cert | no | yes | add certified cut |
| Indeterminate | no | no certified cut | escalate fidelity / retry / local diagnostics / mark unresolved |

并承认：

> finite convergence guarantee excludes Indeterminate oracle returns.

---

# 7. 第六大风险：high-fidelity “feasible” 的语义需要非常严格

你说：

> if verdict.status == Feasible then metrics.op_cost is achievable.

这是 UB 的基础。

但 `hacdcpf` 里可能有多个模块：

- annual production；
- AC OPF；
- reliability；
- resilience；
- carbon；
- lifecycle aging；
- restoration MIP；
- Monte Carlo reliability。

这些模块是否在同一个物理计划、同一个时序、同一个设备约束下联合可行？

例如：

- annual production finds feasible dispatch；
- reliability module separately estimates EENS；
- resilience module solves restoration；
- carbon tracing post-processes；
- AC power flow snapshot checks selected time points；

这并不等于整个 8760 小时 + contingencies + resilience event 的联合 feasible recourse。

## 严苛问题

> `Full` fidelity 的 feasibility 是“所有场景逐一可行”，还是“统一时序策略可行”？  
> annual simulation 的 dispatch 是否满足 AC feasibility at every time step？  
> reliability estimates 是否与 investment-dependent operation strategy consistent？  
> resilience restoration 是否共享同一 ESS energy state assumptions？

如果这些没有统一，UB 的“achievable”语义会被削弱。

## 建议

定义 `Full` fidelity 的最低标准：

1. 对每个 scenario，必须 materialize a concrete dispatch/restoration trajectory。
2. 每个 trajectory 必须满足所有物理约束。
3. carbon/reliability/resilience metrics 必须从该 trajectory 或一致模型派生。
4. 如果某模块只是 statistical/post-process estimate，应明确不能单独作为 UB ingredient。

---

# 8. 第七大风险：L2O 理论目前写得过于自信

`simulation_based_planning_amortized_l2o.md` 的方向是对的，但严苛看存在几类问题。

## 8.1 Regret 定义依赖 expert optimum，实际很难拿到

你定义：

$$
R_i(x)=F_i(x)-F_i(x^*(i))
$$

但是每个 instance 的 $$x^*(i)$$ 来自完整闭环 certified optimum。

这在大规模配网规划中非常昂贵。你后面也承认 demonstration cost 高。

因此，BC 训练集很可能不是“optimal demonstrations”，而是：

- low-fidelity optima；
- time-limited incumbents；
- heuristic plans；
- partially certified solutions。

那么 BC 学到的不是最优 solution map，而是 expert policy map。

建议把理论改成：

$$
x_E(i)
$$

expert-labeled plan，而不是一开始就写 $$x^*(i)$$。

并区分：

- imitation error to expert；
- regret to optimum；
- certification gap of expert itself。

---

## 8.2 DAgger guarantee 不可直接平移

你写：

> Standard DAgger analysis; here the trajectory is the instance stream rather than a control rollout.

这句话会被严格审稿人质疑。

DAgger 的核心是 sequential decision distribution shift，policy affects state distribution。你这里的 instance stream $$i\sim \mathcal{D}$$ 并不由 policy 生成，除非你把“规划 loop 内部访问的 regions/plans”定义成 states。

如果只是 instance-level supervised learning，那么：

- fresh instances 仍来自 $$\mathcal{D}$$；
- policy 不改变 instance distribution；
- DAgger 的必要性不如控制任务明显；
- “trajectory” 类比较弱。

更准确的说法是：

- active learning / dataset aggregation；
- hard-instance mining；
- expert relabeling；
- counterexample-guided data augmentation；
- amortized optimization with adaptive demonstration set。

如果要用 DAgger，需要定义状态为：

$$
(i, x_t, cuts_t, scenarios_t)
$$

并说明 policy 的 proposal 会诱导 optimizer/oracle 访问不同状态分布。

否则“DAgger guarantee”最好弱化。

---

## 8.3 Decision-focused loss 使用 surrogate，可能优化错误方向

你定义：

$$
\mathcal{L}_{DF}(\theta)=
\mathbb{E}_i[\tilde F_i^\theta(\pi_\theta(i))]
$$

其中：

$$
\tilde F = Q_L + \hat e_\phi
$$

问题：

- $$\hat e_\phi$$ 的误差可能使 policy 学会利用 surrogate 漏洞；
- $$Q_L$$ 是 optimistic，不代表真实 feasibility；
- ReLU residual 即使非负，也未必保序；
- decision-focused fine-tuning 可能降低真实 oracle performance。

你说这“不影响 correctness”，因为后面会 certify。是对的。但它会影响效率，甚至使 policy 变成坏 warm-start。

建议加入：

- surrogate uncertainty penalty；
- trust region；
- held-out oracle audit；
- residual nonnegativity constraint；
- conservative residual upper confidence bound for proposal only；
- top-k diversity decoding。

---

## 8.4 GNN size-generalization 不能保证

“30-bus train, 2000-node evaluate” 是 GNN 常见卖点，但在电网规划中不应写得过满。

原因：

- 2000-node system 的 voltage/control regimes 不同；
- topology diameter 变大，message passing over-smoothing；
- candidate density不同；
- budget scale不同；
- failure modes不同；
- graph distribution shift严重；
- feeder hierarchy和相间不平衡可能不在小系统中出现。

建议改为：

> GNN architecture permits variable-size evaluation and permutation equivariance; empirical generalization across sizes must be tested and is not guaranteed by architecture alone.

---

# 9. 第八大风险：surrogate embedded as MILP 可能工程上爆炸

设计中提到：

> ReLU-network-as-MILP embedded into master.

这在小网络中可行，但在规划 master 里可能非常贵。

如果每个 scenario、每个 candidate、每个 iteration 都嵌入 NN：

- binary variables 激增；
- big-M bounds 难 tight；
- branch-and-bound 变慢；
- 下界因为 surrogate 不参与 certification，又不能作为 valid LB；
- 反而拖慢主问题。

尤其你还说 LB 必须来自 pure relaxation，而 solve 用 corrected surrogate $$Q_C$$。这会导致一个微妙问题：

- master objective 用 $$Q_C$$；
- LB 用 $$Q_L$$ at solution；
- 但求解 $$Q_C$$ 的 optimum 并不等于求解 $$Q_L$$ 的 optimum；
- 因此 $$c^\top \hat x + Q_L(\hat x)$$ 不一定是 valid lower bound on $$P_L$$ optimum。

这是非常重要的漏洞。

## 9.1 关键问题：在 corrected surrogate master 上得到的点，不能直接给 pure relaxation LB

文档算法中写：

> solve surrogate with $$Q_C$$;  
> LB ← max(LB, relaxation value $$c^\top \hat x^k + \sum p_s Q_L(\hat x^k,\omega_s)$$)

但如果 $$\hat x^k$$ 是 $$Q_C$$ 的最优解，通常有：

$$
c^\top \hat x^k + Q_L(\hat x^k)
\ge
\min_x \left(c^\top x + Q_L(x)\right)
$$

这是一个 feasible objective value of relaxation，不是 lower bound。对于 minimization，任意 feasible objective value 是 upper bound on relaxation optimum，不是 lower bound。

真正的 lower bound应该是：

$$
LB_L =
\min_x c^\top x + \sum_s p_s Q_L(x,\omega_s)
$$

或者 solver dual bound for pure relaxation master with all valid cuts.

因此你不能在只求 $$Q_C$$ 的情况下，把 $$Q_L(\hat x)$$ 当 LB。

## 这是当前方案里最严重的数学漏洞之一。

### 修正方式

必须维护两个 master：

#### Master-L：纯 relaxation，用于 LB

$$
\min_x c^\top x + \sum_s p_s Q_L(x,\omega_s)
$$

with only valid cuts.

求得：

$$
LB = \text{dual bound of Master-L}
$$

#### Master-C：corrected surrogate，用于 proposal

$$
\min_x c^\top x + \sum_s p_s (Q_L+\hat e)(x,\omega_s)
$$

用于生成 candidate，但不产出 LB。

或者：

- 在同一模型中可切换 objective；
- 先求 LB master；
- 再用 surrogate heuristic / warm-start 产生 incumbents。

这必须改。

---

# 10. 第九大风险：Benders/cuts 与 integer recourse 的边界还需更严格

你已经注意到：

- LP recourse 用 dual cuts；
- MILP recourse 用 integer L-shaped；
- high-fidelity oracle cut 不来自 dual。

这是对的。

但风险在于实际 recourse 可能包含：

- commitment；
- switching；
- restoration；
- topology reconfiguration；
- load priority；
- mobile ESS routing；
- microgrid formation。

这些几乎都会引入 integer recourse。

如果大量 recourse 是 MILP，则经典 Benders 的效率和理论都变弱。

## Integer L-shaped 的 cut 很弱

Laporte–Louveaux integer L-shaped cut 通常依赖 binary first-stage，并且 cut 可能很弱，收敛慢。

对于连续 sizing，它更不直接。

## CCG 会导致模型膨胀

如果不断添加 scenario blocks，master 会越来越大。对于 8760 / reliability / resilience，规模很容易失控。

## 建议

第一阶段应明确一个 MVP 限制：

- recourse 先用 LP；
- switching/restoration integer logic 暂不嵌入 Benders recourse；
- high-fidelity resilience 仅作 oracle verification；
- 对 infeasible plan 加 structural cuts或 scenario penalties；
- 不一开始就做 full MILP recourse decomposition。

---

# 11. 第十大风险：scenario refinement 的统计意义不清

方案中 scenario set 包括：

- typical；
- reliability；
- resilience；
- audit；
- adversarially discovered failures。

如果把失败场景不断加入训练/优化集，会偏向 worst-case/robust planning。这个很好，但目标函数需要清楚：

- 是期望最小化？
- 是 chance-constrained？
- 是 robust？
- 是 CVaR？
- 是 expected cost + reliability penalty？
- 新增 scenario 权重如何分配？
- failure scenario 是来自原分布还是 adversarial stress test？

如果新增场景没有概率权重，或权重任意，就会改变原优化问题。

## 建议

将场景分成三类：

| Scenario type | Role | Weight in objective | Certification use |
|---|---|---:|---|
| SAA sample | estimate expectation | probability weight | SAA objective |
| Critical constraints | must-pass feasibility | constraint, not probability | feasibility screen |
| Audit sample | out-of-sample evaluation | not in optimization | generalization report |

不要把所有场景都混进同一个 $$\sum_s p_s Q$$。

---

# 12. 工程实现风险：当前范围过大

当前方案包含：

- stochastic planning；
- AC/DC hybrid；
- PV/wind/ESS/VSC/SOP/lines/shared ESS/microgrids；
- typical/reliability/resilience scenarios；
- Benders；
- CCG；
- NN surrogate as MILP；
- L2O GNN；
- DAgger；
- carbon tracing；
- lifecycle aging；
- high-fidelity verification；
- finite convergence certificates。

这不是一个项目，是多个博士课题的总和。

最严苛判断：

> 如果按当前全量方案推进，极可能无法在合理时间内形成稳定系统。  
> 必须收缩为“证明闭环有效”的最小版本，再逐层扩展。

---

# 13. 建议的 MVP：把系统削到可证明、可实现、可发表

我建议分三阶段。

---

## Phase 1：无 ML 的认证闭环

目标：先证明 core loop。

### 保留

- first-stage investment：
  - PV；
  - ESS；
  - maybe line upgrade。
- scenario：
  - representative days；
  - small contingency set。
- low-fidelity recourse：
  - LP LinDistFlow；
  - no integer recourse。
- high-fidelity oracle：
  - AC power flow / AC OPF verification；
  - annual production subset。
- certification：
  - pure relaxation LB；
  - oracle feasible UB；
  - no ML。

### 不做

- GNN L2O；
- surrogate-as-MILP；
- resilience MIP；
- mobile ESS；
- microgrid formation；
- DAgger；
- carbon lifecycle full model。

### 产出

证明：

1. low-fidelity relaxation valid under stated assumptions；
2. LB/UB sandwich；
3. oracle feasibility cuts；
4. practical convergence log。

这是最基础、最有说服力的版本。

---

## Phase 2：加入 residual surrogate，但只做 proposal

目标：验证 ML acceleration。

### 加入

- residual model $$\hat e$$；
- corrected objective for candidate generation；
- but LB still from pure relaxation master；
- oracle still certifies UB。

### 必须改

不要把 $$Q_L(\hat x_C)$$ 当 LB。维护两个 objectives：

- pure objective for bound；
- corrected objective for heuristic proposal。

### 评估指标

- oracle calls reduced；
- time-to-first-feasible；
- time-to-gap；
- surrogate error；
- no change in final certificate validity。

---

## Phase 3：加入 L2O/GNN amortization

目标：跨 instances warm-start。

### 加入

- GNN policy；
- top-k plan proposal；
- cheap repair；
- oracle validation；
- warm-start master。

### 不要过度承诺

只宣称：

- faster incumbent discovery；
- improved warm-start；
- transfer empirically evaluated。

不要宣称：

- guaranteed generalization；
- DAgger optimality unless严格证明；
- L2O alone solves planning。

---

# 14. 必须修改的关键文本点

下面这些点建议立即改。

---

## 14.1 修改算法中 LB 更新

当前类似：

$$
LB \leftarrow \max(LB, c^\top \hat x^k + \sum_s p_s Q_L(\hat x^k,\omega_s))
$$

这是错误或至少不严谨的，除非 $$\hat x^k$$ 是 pure relaxation master 的全局最优或其 dual bound 来源于 pure relaxation solve。

应改为：

$$
LB \leftarrow \max(LB, \mathrm{dualBound}(M_L^k))
$$

其中 $$M_L^k$$ 是只含 valid relaxation 和 valid cuts 的 master。

---

## 14.2 区分 `Q_H` 与 simulator cost

建议引入：

$$
Q_H^{\mathrm{opt}}(x,\omega)
$$

和：

$$
\widehat Q_H^{\mathcal{E}}(x,\omega)
$$

并说明：

$$
\widehat Q_H^{\mathcal{E}}(x,\omega)
$$

只要可行，就是 achievable upper-bound cost，不必等于最优 recourse cost。

---

## 14.3 弱化 finite termination

改成：

> Finite termination is guaranteed only for fully discretized first-stage spaces, finite scenario sets, no Indeterminate oracle returns, and valid no-good/structural cuts. Otherwise the method is an anytime bound-certified algorithm.

---

## 14.4 弱化 DAgger guarantee

将：

> expected regret bounded by best-in-class plus $$O(1/\sqrt r)$$

改成：

> Under a formal reduction to online no-regret learning with expert relabeling, one may obtain DAgger-style bounds; in the present planning setting we treat dataset aggregation primarily as an empirical distribution-shift mitigation mechanism unless the induced state distribution is explicitly defined.

---

## 14.5 明确 Indeterminate 处理

算法中加入：

```text
if oracle returns Indeterminate:
    do not update UB
    do not add validity-critical cut
    escalate fidelity / retry / run diagnostics
    if unresolved, mark case as uncertified
```

---

## 14.6 明确 scenario sets

建议定义：

- $$\Xi_{\mathrm{opt}}$$；
- $$\Xi_{\mathrm{verify}}$$；
- $$\Xi_{\mathrm{audit}}$$；
- $$\Xi_{\mathrm{stress}}$$。

并分别说明它们是否进入 objective、constraints、reports。

---

# 15. 对四个文档的逐项严苛评价

## 15.1 `simulation_based_planning_design.md`

### 优点

- 架构直观；
- 与现有 repo 关系清楚；
- 模块复用路径明确；
- 闭环思想表达好。

### 问题

- 范围极大；
- Fidelity A/B/C 边界还可以，但 certification 语义不够严格；
- “simulator verdict agrees” 的标准需要形式化；
- NN-as-MILP 容易导致 master 规模爆炸；
- 多目标/碳/可靠性的处理在 objective 和 constraint 之间切换较随意。

### 建议

把它变成 roadmap 文档，不要让它承担理论保证。

---

## 15.2 `simulation_based_planning_theory.md`

### 优点

- 数学结构清晰；
- LB/UB sandwich 是核心贡献；
- 注意到 integer recourse 和 Benders dual 的边界；
- 明确 ML 不影响 validity。

### 严重问题

- LB 更新逻辑需要修；
- relaxation invariant 过强，需要具体证明库；
- finite termination 对连续变量不成立；
- high-fidelity oracle 与 true optimum 混用；
- SAA gap 与真实期望 gap 混用风险。

### 建议

这是最需要修的文档。修完后整套方案可信度会显著提高。

---

## 15.3 `simulation_based_planning_interfaces.md`

### 优点

- 非常有价值；
- 契约化设计强；
- R1-R5 很好；
- `ViolationCertificate` 是关键接口；
- `RelaxationBuilder` 的 audit 机制方向正确。

### 问题

- `ViolationCertificate` 可能要求过高；
- `Indeterminate` 未在 loop 中制度化；
- `OracleReturn` 的 `metrics.op_cost` 需要区分 feasible trajectory cost vs optimal cost；
- dual/ray exactness 现实中受 solver tolerance 影响，应表述为 tolerance-valid；
- lazy cuts validity必须配套 cut validator。

### 建议

增加：

```cpp
enum class CertificateValidity { Validated, Heuristic, None };
```

只有 `Validated` certificate 才能产生 validity-critical cut。

---

## 15.4 `simulation_based_planning_amortized_l2o.md`

### 优点

- 把 L2O 放回 framework 内部，这是正确方向；
- GNN policy 适合 variable-size networks；
- BC + DF schedule 合理；
- top-k proposal 与 certification 逻辑匹配；
- symmetry pathology 认识到位。

### 问题

- regret/optimum 假设过强；
- DAgger guarantee 类比过强；
- decision-focused surrogate 可能被 policy exploit；
- “size-generalization” 表述过满；
- L2O 的 evaluation metric 应该是 speed-up，而不是 final optimality。

### 建议

把 L2O 文档定位为“acceleration module theory pass”，不要与主理论的 correctness 绑定。

---

# 16. 最严苛的答辩问题清单

如果我是审稿人/答辩委员，我会问：

1. **你的 low-fidelity model 真的是 high-fidelity model 的 relaxation 吗？逐条证明在哪里？**
2. **LinDistFlow 对 AC feasible set 是外近似、内近似，还是普通近似？**
3. **如果 surrogate master 的解不是 pure relaxation 的最优解，为什么它的 $$Q_L$$ 值能作为 lower bound？**
4. **`hacdcpf` 返回的是全局最优 operation cost，还是一个可行 dispatch cost？**
5. **如果 oracle 返回 solver failure / nonconvergence，你如何避免错误 cut？**
6. **连续 sizing 变量存在时，你如何保证 finite termination？**
7. **你的 UB 和 LB 是针对同一个 scenario sample set 吗？**
8. **新增 critical scenarios 的概率权重如何定义？**
9. **reliability/resilience/carbon 模块的指标是否来自同一个可行运行轨迹？**
10. **NN-as-MILP 嵌入后 master 规模如何控制？**
11. **GNN 从 30-bus 到 2000-bus 的泛化依据是什么？**
12. **DAgger 的 state distribution 在你的规划问题里具体是什么？**
13. **如果 learned policy 给出 infeasible plan，如何保证不会污染 bounds？**
14. **你的 lower-bound sequence 是否单调？在添加 cuts/columns 和切换 surrogate 时是否仍然有效？**
15. **SAA statistical error 如何报告？**

如果你能逐条回答，上述方案会非常强；如果不能，建议先收缩范围。

---

# 17. 最终建议：保留核心，削弱承诺，强化证书

我建议你把整个设计的定位调整为：

> 一个 **bound-certified, simulation-verified, ML-accelerated planning framework**。

而不是：

> 一个同时具有完整高保真全局最优、有限终止、跨规模 L2O 泛化保证的系统。

更具体地：

## 必须保留

- ML proposes, framework certifies；
- pure relaxation produces LB；
- oracle feasible trajectory produces UB；
- invalid oracle/surrogate never affects correctness；
- interface contracts；
- high-fidelity feedback loop。

## 必须修改

- LB 更新；
- `Q_H` vs simulator cost；
- finite termination；
- continuous variable no-good cuts；
- DAgger guarantee；
- Indeterminate oracle；
- scenario-set semantics。

## 必须补充

- relaxation proof registry；
- cut validity validator；
- oracle status handling；
- two-master architecture；
- SAA vs true-distribution gap；
- MVP roadmap。

---

# 18. 简短结论

从最严苛角度看：

> 这是一个很有潜力的框架设计，但当前文本的理论保证写得比实际可证明范围更强。最大的问题不是方向错误，而是“有效下界、有效上界、有限终止、仿真最优性、L2O 泛化”这几条保证之间的边界还不够硬。

如果修正以下三点，可信度会大幅提高：

1. **LB 只能来自 pure relaxation master 的最优值/dual bound，不能来自 corrected surrogate solution 的 $$Q_L$$ 值。**
2. **`hacdcpf` 成本必须定义为 achievable UB cost，而不默认等于 true high-fidelity optimal recourse。**
3. **finite termination 只在完全离散化、有限场景、无 Indeterminate、valid cuts 的条件下成立；否则是 anytime certified algorithm。**

最终评价：

| 维度 | 评分 | 评价 |
|---|---:|---|
| 核心思想 | 9/10 | 非常好，方向正确 |
| 理论严谨性 | 6/10 | 框架好，但若干证明链过强 |
| 工程可落地性 | 5/10 | 当前范围过大，需 MVP |
| ML 使用方式 | 8/10 | 放在 acceleration path 上是正确的 |
| 接口设计 | 8/10 | 契约意识强，但 certificate 要求需现实化 |
| 可发表潜力 | 8/10 | 修正后很有潜力 |
| 当前整体成熟度 | 6.5/10 | 适合作为 v0.1 theory/design，但不能直接声称完整保证 |

一句话总结：

> **保留“优化给下界、仿真给上界、ML 只提案”的主线；砍掉过强承诺；把所有 guarantee 限定到可证明的模型、样本集和证书条件内。这样它会从一个宏大的系统设想，变成一个真正严谨、可实现、可扩展的规划框架。**