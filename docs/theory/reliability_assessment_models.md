# 可靠性评估经典方法与执行契约

> 最后核实：2026-08-20。
>
> 本文是当前实现的中文摘要，不保存历史缺陷清单或研究路线图。完整推导、保护配合、信息系统、
> 在线动态、参数影响、方法对比和验证算例见
> `docs/modules/reliability/reliability_manual.tex`。运行行为以 `include/`、`src/`、`tests/`
> 和已注册测试为准。

## 1. 统一参数与概率模型

所有评估入口通过统一解析器把元件数据归一为运行时间故障强度、日历年事件频率、平均修复时间、
不可用度和平均无故障时间。设报告年小时数为 $H$、运行时间故障强度为 $\lambda_{up}$、
修复时间为 $r$，则

$$
\mu=H/r,\qquad
U=\frac{\lambda_{up}}{\lambda_{up}+\mu}
=\frac{\lambda_{up}r}{H+\lambda_{up}r},\qquad
f_{cal}=(1-U)\lambda_{up}.
$$

年度 EENS、LOLE 和 LOLF 使用 $f_{cal}$，不得把运行时间强度直接当作日历年事件频率。
`StrictCaseDataOnly` 不补造缺失数据；模板策略由同一逐元件解析器供非序贯 MC、序贯 MC、FMEA
和精确灵敏度共同使用。解析结果保留数据来源与有效性诊断。

## 2. 状态后果模型

物理后果的完整独立审计见中文手册
`docs/modules/reliability/chapters/physical_consequence_models.tex`。该章统一审计主网 HL-II、混合
AC/DC、配网三阶段和运行拓扑重构的可行域；本摘要只保留执行契约，避免把指标聚合与物理后果混为一谈。

纯交流 HL-II 状态采用最小切负荷 DC-OPF；混合交直流状态采用 AC/DC 有功网络 LP。设
$d_i$ 为毛负荷，$s_i$ 为负荷削减，$p_g$ 为可调机组出力，$\bar p_i^{fix}$ 为固定正注入，
$p_i^{gc}$ 为固定注入削减，则交流节点平衡为

$$
\sum_{g\in\mathcal G_i}p_g+\bar p_i^{fix}-p_i^{gc}+s_i-d_i
=\sum_{e\in\delta(i)}K_{ie}f_e,
$$

并满足

$$
0\le p_g\le a_g\bar P_g,\quad
0\le s_i\le d_i,\quad
0\le p_i^{gc}\le\bar p_i^{fix},\quad
-a_e\bar f_e\le f_e\le a_e\bar f_e.
$$

每个交流连通分量任取一个角度参考。输入 `SLACK` 只是可复用的角度规范，不是供电能力，也不是
可行性条件。HL-II 允许机组从零出力重新调度；固定注入允许显式削减。因此

$$
p=0,\qquad f=0,\qquad \theta=0,\qquad s=d,\qquad
p^{gc}=\bar p^{fix}
$$

是构造性可行点。该模型出现未收敛或后验原始可行性认证失败时，只能视为构模或数值求解缺陷，
状态评估立即失败，禁止以“全切负荷”伪造 EENS。

目标采用严格两阶段字典序：先最小化 $\sum_i s_i$，再固定其最优值并最小化发电和固定注入削减成本。
单一有限 VOLL 只有在证明支配界和求解容差后才与字典序等价，当前可靠性后果入口不依赖这种近似。

## 3. 拓扑与物理边不变量

拓扑分析、随机状态和功率流约束必须使用同一物理边集。MATPOWER 导入形成的
`Transformer2W(source_branch_idx>0)` 是原始 `ACBranch` 的参数与归因元数据，不是第二条物理边，
不进入图、LP 或独立随机元件集合。其可用状态由所链接的原始支路唯一决定。

这项不变量消除了旧实现中“拓扑仍连通、B 矩阵已经断开”的表示矛盾。无源岛的全负荷削减由同一个
优化模型内生得到，不经过 slack/source 预筛，也不经过失败回退。

## 4. 非序贯与序贯蒙特卡洛

非序贯 MC 对独立二态元件抽样

$$
p(x)=\prod_iU_i^{x_i}(1-U_i)^{1-x_i},
$$

并以状态后果 $S(x)$ 估计 EDNS、EENS、LOLE 和 PLC。优势比扭曲重要抽样使用精确似然比
$w(x)=p(x)/q(x)$，结果返回权重诊断和有效样本量

$$
N_{eff}=\frac{(\sum_n w_n)^2}{\sum_nw_n^2}.
$$

非序贯尾部风险先把状态样本按报告年聚合成合成年损失，再计算 VaR 与经验期望短缺；不把单个随机小时
乘以全年小时数冒充年度分布。

序贯 MC 按失效与修复指数时钟生成逐小时状态，并消费负荷时间曲线。对同一小时健康状态 $x_0$，定义

$$
S^0_h=S(x_0,h),\qquad
S^{inc}_{y,h}=\max\{0,S(x_{y,h},h)-S^0_h\}.
$$

标准原始指标、健康基线和故障增量分别为

$$
\mathrm{EENS}^{raw}=\frac1{N_y}\sum_y\sum_hS(x_{y,h},h),
$$

$$
\mathrm{EENS}^{0}=\frac1{N_y}\sum_y\sum_hS^0_h,\qquad
\mathrm{EENS}^{inc}=\frac1{N_y}\sum_y\sum_hS^{inc}_{y,h}.
$$

`eens_mwh_yr` 和 `annual_eens` 使用原始量；`baseline_eens_mwh_yr` 单列健康基线；
`incremental_eens_mwh_yr` 单列故障增量。LOLE 和 LOLF 同样由原始逐小时失负荷标志计算。
健康状态已有缺额时，不能静默扣除后仍把结果称为标准 EENS。

`ReliabilityResult::converged` 在 MC 中表示统计 CoV 停止条件是否满足，不表示单个 OPF 状态是否求解成功。
单状态求解失败会立即抛错；`opf_failed_probability` 与 `opf_failed_eens_mwh_yr` 在有效运行中必须为零。

## 5. FMEA、联合故障与重要度

单故障模式先形成显式后果补丁，再调用共同物理后果引擎。一级 FMEA 聚合日历事件频率、阶段持续时间
与切负荷。二阶联合故障使用

$$
\mathbb E[S]\approx S_0+\sum_iU_i(S_i-S_0)
+\sum_{i<j}U_iU_j(S_{ij}-S_i-S_j+S_0),
$$

并报告计算完整性、跳过原因和二阶交互项。规模守卫内还可精确枚举全部状态，计算 Birnbaum、
Fussell--Vesely 和 EENS 对不可用度的解析导数；有限差分只作验证，不替代理论定义。

物理与信息成功路径按共享元件保持的容斥计算联合可用率，最小割集通过极小击中集求解并返回稳定组件 ID。
信息功能支持共享通信依赖、QoS、共同原因和互斥功能类，不能把各功能可用率简单相乘。

## 6. 三阶段恢复与保护信息物理链

三阶段恢复 MILP 依次求解故障隔离、运行拓扑重构和修复窗口。故障元件在全部阶段保持停运；第三阶段继承
第二阶段已接受拓扑。模型覆盖 AC LinDistFlow、DC 有功平衡、辐射森林、热限、开关动作、双向 VSC/DC--DC、
DER、微网和储能跨阶段能量。每个阶段有 $0\le p^{sh}\le P^d$；对 $P^d>0$ 的负荷采用
$q^{sh}=(Q^d/P^d)p^{sh}$，对 $P^d=0,Q^d\ne0$ 的纯无功负荷另设
$\min(0,Q^d)\le q^{sh}\le\max(0,Q^d)$，所以 $p^{sh}=P^d,q^{sh}=Q^d$ 是显式削减可行点。MILP 求解失败或后验
认证失败立即抛出异常，不以全切负荷伪造 EENS。保护联锁阻止的是恢复动作准入，不等价于数学不可行。
LCC 与多端口能量路由器在入口明确拒绝，不返回伪造的零影响结果。

保护执行链实际计算 CT/PT 动态与饱和、定时限/反时限、方向、距离、差动、主后备配合、断路器失灵、
重合器—熔断器—分段器序列、自动保护拓扑、DER 穿越与闭锁、微网同步窗以及信息共因和备用电池。
三级方法对照可消费给定轨迹，也可在线调用 Mass-Matrix DAE；在线模式把保护动作反馈到网络并重新计算轨迹。

保护在线模型是正序网络和单相故障输入，不认证三相 EMT、行波保护或 CT 磁滞；年度后果采用三个时间窗口。
这些是当前模型边界，不是未实现功能。

## 7. 结果解释与 IEEE RTS-24 基准

结果必须同时读取 `model_scope`、`validity`、`model_limitations`、求解器状态和后验可行性证书。
不同后果模型的 EENS 不能脱离状态空间、负荷轨迹、故障集合和运行语义直接比较。

IEEE RTS-24 当前逐行映射 33 台机组和 38 条物理支路。确定性 N-0/N-1/N-2 共 2557 个状态全部通过：
无求解失败、无非有限结果、无负削减、无超总负荷削减，最大原始约束违反为 0 MW。当前固定种子诊断值为：

| 方法 | EENS/(MWh/yr) | LOLE/(h/yr) | CoV | 状态求解失败 |
|---|---:|---:|---:|---:|
| 非序贯 MC，20000 状态 | 123156.84 | 738.468 | 0.0314 | 0 |
| 序贯 MC，400 年 | 1222.5778 | 10.5375 | 0.1152 | 0 |

两次 MC 均未满足各自预设统计停止条件，因此只能称为诊断值，不能称为最终收敛基准。经典 RTS-24
约 1200 MWh/yr、9.4 h/yr 的结果通常是只抽样机组、忽略网络故障的 HL-I 口径；与 HL-II 网络约束
结果比较时必须采用相同机组状态和负荷轨迹，计算反事实网络增量，不能把两组数直接相减后归因。

## 8. 验证入口

- `test_reliability_resolver`：参数解析、MC/FMEA、混合 AC/DC 后果、固定注入削减、RTS-24 状态扫描。
- `test_three_stage_reliability`：三阶段恢复、交直流耦合、保护动作与求解证书。
- `test_intelligent_cyber_physical_reliability`：有限 POMDP、检测、隔离和风险约束恢复。
- `reliability_workflow_e2e`、`reliability_configuration_e2e`：HTTP/GUI、在线 DAE、参数保存与移动视口。
- `pf_doc_benchmark rel`：RTS-24 F&D、非序贯 MC 和序贯 MC 固定种子诊断。

任何新增设备、后果模型或结果字段必须同步修改完整中文手册、本文、运行接口和注册测试；不得只在理论中规划。
