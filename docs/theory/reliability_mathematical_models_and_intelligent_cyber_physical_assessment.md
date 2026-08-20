# 可靠性数学模型与智能信息物理评估执行契约

> 最后核实：2026-08-20。
>
> 本文只描述当前代码实际执行的模型，不保存研究路线图。完整中文推导、参数影响、保护配合、在线动态、
> 三级对照与验证案例见 `docs/modules/reliability/reliability_manual.tex`。运行行为以
> `include/hacdcpf/reliability/`、`include/hacdcpf/analysis/three_stage_reliability.hpp`、
> `src/reliability/`、`tests/run_gui_server.cpp` 和注册测试为准。

## 1. 结果口径与共同原则

可靠性结果统一按

$$
\text{年度风险}=\sum_k \text{日历年事件频率}_k\times
\text{条件后果}_k\times\text{持续时间}_k
$$

聚合。稳定组件身份采用“组件类型 + `.index`”，数组位置只作为内部索引；交流和直流母线映射始终分域。
近似、回退、求解时限、模型覆盖不足和输入不适用必须通过 `model_scope`、`validity`、
`model_limitations` 或失败结果显式返回，不能用零影响值代替。

本模块执行四类后果口径：

| 入口 | 物理后果口径 | 主要输出 |
|---|---|---|
| 非序贯/序贯 MC、FMEA | 纯交流 `ac-only-dcopf`；混合系统 `hybrid-acdc-network-lp` | EENS、LOLE、LOLF、节点和元件归因 |
| 三阶段恢复 | `ac-lindistflow-milp` 或 `coupled-acdc-lindistflow-restoration-milp` | 隔离、转供、修复窗切负荷与恢复动作 |
| 物理/信息成功路径 | 独立二态元件的精确容斥和最小击中集 | 可用率、失效概率、稳定 ID 最小割集 |
| 保护—信息物理对照 | 给定轨迹或在线 Mass-Matrix DAE 的互斥事件树 | 静态 FMEA、仅保护、联合 EENS/LOLE/LOLF |

## 2. 可修元件、故障率口径与年度频率

### 2.1 运行时间强度

设报告年小时数为 $H$、设备运行期间的年故障强度为 $\lambda_{up}$、平均修复时间为 $r$，二态交替更新模型给出

$$
\mu=H/r,\qquad
U=\frac{\lambda_{up}}{\lambda_{up}+\mu}
=\frac{\lambda_{up}r}{H+\lambda_{up}r},\qquad
\mathrm{MTTF}=H/\lambda_{up}.
$$

MTBF 的解释由输入约定决定：若指运行时间均值，则 `MTTF=MTBF`；若指含修复的完整周期，则
`MTTF=MTBF-MTTR`，不一致数据由解析器校验。

### 2.2 日历年事件频率

若输入是日历暴露下观测到的年事件频率 $f_{cal}$，则不能直接当作运行时间强度。精确关系为

$$
U=\frac{f_{cal}r}{H},\qquad
\lambda_{up}=\frac{f_{cal}}{1-U},\qquad
f_{cal}=(1-U)\lambda_{up}.
$$

`failure_rate_basis=calendar_time` 执行上述反演；`operating_time` 执行运行时间强度公式。未知枚举、
$H\le0$ 或 $f_{cal}r\ge H$ 明确失败，不采用稀有事件近似。解析结果同时返回
`calendar_frequency_per_year`，年度 EENS/LOLE/LOLF 一律使用日历频率。

## 3. 精确状态、蒙特卡洛、FMEA 与灵敏度

### 3.1 独立二态状态概率

对元件不可用度 $U_i$ 和状态 $x_i\in\{0,1\}$，精确状态概率为

$$
p(x)=\prod_i U_i^{x_i}(1-U_i)^{1-x_i},\qquad \sum_xp(x)=1.
$$

非序贯 MC 按该分布采样随机状态；序贯 MC 按失效/修复转移时钟推进，并记录持续时间和年度事件次数。
重要抽样使用偏置分布 $q(x)$，每个样本按似然比

$$
w(x)=p(x)/q(x)
$$

还原目标分布，并返回权重方差和有效样本量

$$
N_{eff}=\frac{(\sum_nw_n)^2}{\sum_nw_n^2}.
$$

### 3.2 精确枚举与重要度

规模守卫内可枚举全部状态，直接计算

$$
\mathrm{EENS}=H\sum_xp(x)S(x).
$$

对元件 $i$，Birnbaum 边际和 EENS 导数使用同一状态对：

$$
B_i=\Pr(L\mid x_i=1)-\Pr(L\mid x_i=0),
$$

$$
\frac{\partial\mathrm{EENS}}{\partial U_i}
=H\left[\mathbb E(S\mid x_i=1)-\mathbb E(S\mid x_i=0)\right].
$$

Fussell--Vesely 重要度按包含元件失效的风险贡献除以总风险计算。解析导数由有限差分和闭式小系统共同验证。

### 3.3 FMEA 与二阶联合停运

单故障模式先通过后果补丁改变可用容量、拓扑、保护区或持续时间，再调用物理后果引擎。两模式联合项采用二阶交互展开

$$
\mathbb E[S]\approx S_0+\sum_iU_i(S_i-S_0)
+\sum_{i<j}U_iU_j(S_{ij}-S_i-S_j+S_0).
$$

结果返回已计算/跳过的模式对和二阶完整性标志；三阶及以上联合状态不伪装为已计算。

### 3.4 频率—持续时间与容量状态表

发电充裕度采用未分箱的精确停运容量状态递推。相邻容量相同的状态合并概率但不改变容量值，LOLP、LOLE、LOLF、
LOLD 由状态概率与上下穿越频率计算，并返回概率和、负概率、频率一致性等有效性诊断。

## 4. 物理网络成功路径与最小割集

输入元件可用率 $A_i$ 和完整成功路径族 $\mathcal P$。系统成功事件为

$$
\mathcal S=\bigcup_{p\in\mathcal P}\bigcap_{i\in p}\{X_i=1\}.
$$

程序先删除包含其他成功路径的超集，再对保留路径作共享元件保持的容斥：

$$
\Pr(\mathcal S)=
\sum_{\emptyset\ne J\subseteq\mathcal P}
(-1)^{|J|+1}\prod_{i\in\cup_{p\in J}p}A_i.
$$

最小割集是成功路径族的极小击中集。实现逐路径扩展候选击中集并做吸收约简，对外返回稳定 ID，不以数组位置冒充元件身份。
元件状态独立、路径完整和单调相干是该精确模型的前提；重复 ID、非法概率、越界索引或超过规模守卫时明确拒绝。

闭式验收例：路径 $\{1,2\}$ 与 $\{1,3\}$，可用率 $(0.9,0.8,0.7)$，则

$$
A=0.9[1-(1-0.8)(1-0.7)]=0.846,\qquad Q=0.154,
$$

最小割集为 $\{1\}$ 和 $\{2,3\}$。

## 5. 三阶段交直流恢复 MILP

### 5.1 阶段与目标

每个故障依次计算隔离、开关恢复、修复窗三个阶段；故障元件在三段内保持开断，第三阶段保持第二阶段已接受的联络方案。
目标为交流和直流切负荷之和，另加 $10^{-8}$ 的储能/换流器简并破除项：

$$
\min\sum_{i\in\mathcal B^{ac}}p_i^{sh}
+\sum_{d\in\mathcal B^{dc}}p_d^{sh}+10^{-8}\Phi.
$$

源容量、带电状态、交流/直流节点平衡、电压降、开关预算、故障开断、储能跨阶段能量、VSC/DC-DC 双向效率和严格辐射森林
在同一模型内执行。交流与直流母线使用分域映射。

### 5.2 视在功率内接多边形

对每条带电交流支路施加偶数 $J\ge4$ 个半空间：

$$
P_{ij}\cos\theta_j+Q_{ij}\sin\theta_j
\le z_{ij}\overline S_{ij}\cos(\pi/J),\qquad
\theta_j=(2j+1)\pi/J.
$$

$\cos(\pi/J)$ 使多边形内接于视在功率圆，因此可行点必满足
$\sqrt{P_{ij}^2+Q_{ij}^2}\le\overline S_{ij}$。默认 $J=16$；结果返回边数、约束执行标志和最大
`|S|/额定值`。

### 5.3 N-0 反事实与年度聚合

故障与健康反事实从相同初始 SOC、负荷、可再生可用性和阶段时长出发，分别演化储能轨迹。可靠性增量为

$$
s_{i,k,s}^{inc}=\max(0,s_{i,k,s}^{fault}-s_{i,k,s}^{N0}),
$$

$$
\mathrm{EENS}_i=\sum_k f_{k}^{cal}\sum_s s_{i,k,s}^{inc}\Delta t_{k,s}.
$$

### 5.4 最优性证书与间隙语义

阶段求解先保存后端原始终止状态和原始相对 gap，再形成认证状态。若后端明确返回 `HiGHS optimal`、
`StrictHiGHS Optimal run=0` 或原生分支定界的明确最优终止状态，则最优性证书优先于零/近零目标下可能失真的相对 gap，
认证 gap 规范为 0；否则仅当有限 gap 不超过 $10^{-6}+10^{-9}$ 时认证最优。任何变量界、整数性、等式、
不等式、热限或倒闸后验检查失败仍将阶段置为失败，最优状态不能掩盖约束违规。

HTTP 每个故障返回：

- `stage1/2/3_status`：归一后的成功、近似或失败状态；
- `stage1/2/3_solver_status`：后端原始终止状态；
- `stage1/2/3_mip_gap`：认证间隙；
- `stage1/2/3_solver_reported_mip_gap`：后端原始间隙。

## 6. 保护测量、判据与配合

### 6.1 CT/PT 动态与饱和

测量链按采样步推进一阶动态，CT 在磁通过限后限制二次电流并记录饱和，PT 按量程和时间常数形成二次电压。保护判据只消费测量链输出，
不直接读取理想一次量，因此饱和、延迟和失真会改变动作时刻和选择性。

### 6.2 过流、方向、距离与差动

定时限在 $I\ge I_{set}$ 持续达到整定延时后动作。反时限采用曲线

$$
t_{op}=TMS\left(\frac{A}{M^P-1}+B\right),\qquad M=I/I_{set}>1.
$$

方向门以极化电压和电流的相角关系判定正向故障。距离保护同时执行 mho 圆和四边形区域：

$$
|Z-Z_c|\le R_c,\qquad Z=V/I,
$$

或按四边形的电阻、电抗边界判定。差动保护采用带制动斜率的动作区：

$$
I_{diff}\ge I_{pickup}+kI_{bias}.
$$

主保护、后备保护、断路器固有延时和失灵后备按事件时钟执行；配合裕度由实际清除时刻之差校核，不用静态标签代替。

### 6.3 重合器、熔断器和分段器

重合器执行多次快速/慢速动作、死区、重合成功或闭锁；熔断器按时间—电流曲线累计热效应；分段器在上游无流窗口计数并在整定次数后开断。
序列引擎同时校核熔断器保护/熔断器节省策略、动作先后、故障区隔离和重合后拓扑。断路器失灵会扩大停运保护区并进入年度后果。

### 6.4 保护误动

无故障判别窗内，测量误差、通道状态、保护安全性和断路器按需动作形成误动事件频率。每次误动通过实际停运区和恢复时间计算
EENS/LOLE/LOLF 增量，并返回频率分解残差；不能把误动只作为日志而不进入年度指标。

## 7. 信息系统与智能决策

### 7.1 共享依赖、QoS 与共因

信息元件状态在检测、跳闸、隔离和恢复多个功能之间共享，同一控制中心或通信链路只抽样一次。路径只有在可用且同时满足时延、抖动、丢包率和带宽门限时才成功。
环境状态用于表达共因组失效；各环境概率必须归一。物理母线失电会切断所供通信设备，除非电池自治时间覆盖事件窗口。

对不超过 20 个二态信息元件的输入，程序枚举全部状态：

$$
\Pr(X=x)=\sum_e p_e\prod_u q_{u|e}^{x_u}(1-q_{u|e})^{1-x_u}.
$$

检测、主跳、后备跳、隔离和恢复函数的路径在同一状态上求值，避免错误地相乘边际可用率。超出规模守卫时明确拒绝。

### 7.2 检测、隔离、恢复与有限 POMDP

智能检测返回正确检测、漏检和误报互斥结果及延迟。隔离动作通过稳定组件 ID 作用于保护区和开关拓扑；风险约束恢复只接受满足辐射、供电能力、
电压/热限和动作风险上界的方案。有限 POMDP 使用显式状态、观测、转移、奖励和有限时域信念树精确求解；输入超过状态/动作/观测/时域守卫时拒绝，
不返回启发式伪最优策略。

## 8. 在线 Mass-Matrix DAE 与 DER-FRT 闭环

### 8.1 两遍在线执行

在线入口按给定故障位置和时间执行两遍 Mass-Matrix DAE：第一遍生成未动作的测量轨迹并驱动 CT/PT、主后备保护、重合器/熔断器/分段器和断路器事件；
第二遍把已确定的跳闸、重合、闭锁和保护区拓扑动作反馈到网络方程。每次拓扑变化重新形成网络代数约束，失去电源的径向岛负荷退出。

质量矩阵形式为

$$
M\dot x=f(t,x,z),\qquad 0=g(t,x,z,\sigma),
$$

其中 $x$ 为动态状态、$z$ 为代数量、$\sigma$ 为保护和开关离散状态。结果返回积分器、步长、轨迹点数、重解次数、动作反馈和残差诊断。

### 8.2 DER 穿越与保护反馈

DER-FRT 对电压/频率分区分别累计穿越计时器，状态包括连续运行、瞬时闭锁、跳闸、重连资格和恢复爬坡。保护动作改变端电压轨迹，DER 状态又改变故障后可用功率，
二者在第二遍网络求解中闭环。微网重连需同时满足电压、频率和相角同步窗。

在线入口当前采用正序网络和单相故障输入，年度后果采用故障清除、开关恢复、修复三个窗口；这些是结果边界，不是待实现功能。

## 9. 三级对照与因果解释

同一初始故障集合上计算三条路径：

1. 静态 FMEA：不执行动态保护和信息条件；
2. 仅保护：执行测量、保护、开关和 DER-FRT，不施加信息系统失败；
3. 信息物理联合：在同一保护事件树上叠加共享信息状态、QoS、共因、物理供电和电池自治。

三条路径使用同一日历年事件频率和三窗口后果映射，因此

$$
\Delta R_{prot}=R_{prot}-R_{static},\qquad
\Delta R_{cyber}=R_{joint}-R_{prot}
$$

可解释为保护执行和信息系统条件化的增量，而不是把不同求解器的无关结果硬比较。闭式基准为：

| 路径 | EENS（MWh/年） | LOLE（h/年） |
|---|---:|---:|
| 静态 FMEA | 200.0000000000 | 20.0000000000 |
| 仅保护 | 0.4007777778 | 0.2000777778 |
| 信息物理联合 | 21.5217333333 | 4.1600622222 |

保护误动闭式基准为频率 `0.4 次/年`、EENS 增量 `0.2 MWh/年`、LOLE 增量 `0.1 h/年`、
LOLF 增量 `0.4 次/年`，分解残差为 0。

## 10. 明确拒绝的模型边界

下列对象不属于可靠性入口定义的结果空间，遇到时必须拒绝或清除有效标志，不能返回伪造结果：

- 三阶段恢复不接受 LCC 换相方程、多端口能量路由器端口平衡和固定/二次换流损耗；
- 三阶段 LinDistFlow 不认证非线性交流潮流的完整无功/电压可行性；
- 在线保护入口不声称三相不平衡 EMT、行波保护或电弧微分方程；
- 物理/信息最小割集精确枚举不接受不完整路径、相关元件却未给共因环境或超过规模守卫的输入；
- 有限 POMDP 不接受超过显式状态空间守卫的输入；
- 纯交流 `ac-only-dcopf` 结果不包含直流负荷后果；混合 `hybrid-acdc-network-lp` 结果不认证非线性交流电压。

这些边界只说明当前模型能够回答的问题范围。

## 11. 实现、接口与验证对应

| 理论对象 | C++ 实现 | 主要验证 |
|---|---|---|
| 故障率口径、解析与精确灵敏度 | `src/reliability/reliability_assessment.cpp` | `test_reliability_resolver` |
| 失效模式目录和二阶联合停运 | `src/reliability/failure_mode.cpp` | `test_reliability_resolver` |
| 物理/信息成功路径、最小割集和共享状态 | `src/reliability/reliability_assessment.cpp` | `test_reliability_resolver`、`test_intelligent_cyber_physical_reliability` |
| CT/PT、保护判据、设备序列和误动 | `src/reliability/online_protection.cpp`、`src/reliability/protection_frt.cpp` | `test_intelligent_cyber_physical_reliability` |
| 智能检测、隔离、恢复、有限 POMDP | `src/reliability/intelligent_cyber_physical.cpp` | `test_intelligent_cyber_physical_reliability` |
| 在线 DAE、保护反馈和 DER-FRT | `src/reliability/online_protection.cpp` | `test_intelligent_cyber_physical_reliability`、浏览器 E2E |
| 三阶段交直流恢复 MILP | `src/reliability/three_stage_reliability.cpp` | `test_three_stage_reliability` |
| HTTP/GUI 契约 | `tests/run_gui_server.cpp`、`web/js/app.js` | `reliability_configuration_e2e`、`reliability_workflow_e2e` |

提交验收同时要求：C++ 专项全绿、两个可靠性 E2E 全绿、JavaScript 语法检查通过、差异中不存在未完成标记、中文手册不少于 100 页且最新版逐页渲染无空白、乱码、裁切和重叠。
