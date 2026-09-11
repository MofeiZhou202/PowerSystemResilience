# Windows 修复与性能验证（2026-09-11）

基线提交 75c6e1922839d4775c27eacda833df386e95f476，证据见 windows_offline_evaluation_2026-09-11.md。用户授权实施全部三个阶段；允许同步远端 main。已从 b42a0540 读取完整 theory-guided-coding 技能，后续实现遵守其三阶段契约，依赖继续本地化。

## R1：安装依赖闭包与 MSVC 归档

RATIONALE

- Model/algorithm: 静态库链接的有向依赖图；每个导出的命名节点必须在消费作用域中可解析，归档合并必须保留所有输入成员。
- Claim: 恢复 MIPSolvers::MKL imported interface，MSVC 使用 lib.exe /OUT，消除 missing-target 和缺失归档。
- Cost model: 配置 O(依赖数)，合并 O(输入归档字节)，不改变数值运算。
- Prediction: 两个已复现交付失败变为通过；求解内核运算变化为 0。
- Assumptions: 本地 staged MKL 文件和相同 ABI 工具链可用；分别验证 sequential 与 INTEL 闭包。
- References: 本地 cmake/BuildIpopt.cmake 的 MIPSolvers::MKL 契约；CMakeLists.txt 的导出图；MSVC lib.exe 本机 /?。
- Validation: cmake build Release；声明归档存在且 lib /LIST 可读；cmake install 后在不同路径配置 cmake/consumer-example，编译执行并检查 objective=9；全量 CTest。不得仅以 build exit=0 判定归档成功。

后续数值修复在定位根因后分别追加完整 rationale，再实施。

## R2：无下界变量的标准型等价性

RATIONALE

- Model/algorithm: 标准型非负分解。自由 x=p-q；仅有上界 u 时 x=u-q（p 固定为零）；p,q>=0。相应矩阵列为 A_j 与 -A_j，目标系数为 c_j 与 -c_j。原变量顺序不变，负列追加在所有既有列之后。
- Claim: 当前构造把无下界变量错误限制为非负；引入负列恢复原模型可行域的满射，并保留目标及 recession ray。tuff 两个 FR 变量因此允许取负值。
- Cost model: 新增 k 个列及这些列原非零数之和，O(nnz_free) 带宽成本；所有有限下界模型 k=0，结构与数值运算不变。
- Prediction: tuff 从 0/3 准确变为 3/3；有有限下界的小模型 pivot 路径不变。该修复不承诺 tuff 加速，额外列可能增加求解成本。
- Assumptions: 非负标准型核的既有正确性；原模型界哨兵遵守 dual_simplex_api.cpp 契约。分裂列存在时增量结构 API 可返回 false 请求完整重建，不能在不兼容布局上继续更新。
- References: 本节代数恒等式；docs/manual/06-numerical-methods.md 的单纯形标准型；基线 tuff_direct_before.json；MPS FR UH2...BW / U9R.BLBW。
- Validation: 小模型要求负自由变量、上界单侧负最优解、无界射线和目标更新；tuff --solvers native-dual-direct,native-auto --repeat 3；NETLIB 原生 IPM/Auto 全集各 3 次，仍用 1e-5 / 1e-7 审计；全量 CTest。

## R3：冷启动 NLP 的惯性证书

RATIONALE

- Model/algorithm: 等式约束 Newton 方向的 bordered-Hessian 惯性条件 inertia(K)=(n,m_eq,0)，不可用的惯性不是该条件的证据。
- Claim: 无惯性信息的稀疏 LU 不能绕过既有惯性校正；应交由现有 LDLT / reduced-space 证书路径处理。
- Cost model: 可能多一次失败的候选分解及一次惯性校正，O(factor work)；与原来错误方向引起的 restoration 重试比较。
- Prediction: 所有接受的 exact-condensed 方向都有实际惯性证书；HS071 由 0/5 改善为 5/5 是本阶段验收目标。若仍失败，回到初始化与全局化推导，不能以此宣布修复完成。
- Assumptions: 现有惯性校正实现符合本地推导；不改变调用者已提供 warm-start 的意义。
- References: docs/archive/opf_native_ipm_structural_derivation.md；kkt_system.cpp 的 bordered-Hessian 同余分解；HS071 trace 的 negative=-1 / deficiency=-1。
- Validation: nlp_open_benchmark 5，原生无 fallback、1e-6 目标绝对误差及 primal 审计；test_ipm_solver、test_numerical_stability 及全量 CTest。

### R2 首轮验证偏差：旧对象文件的 ABI 混用

新加自由变量单元测试返回 standard-form dimensions are invalid，tuff 进程未产生完整结果。按偏差协议先查实现保真：native_dual/state.cpp.obj 时间为 2026-08-20，而公共结构头已于 2026-09-11 改动；ninja -t deps 显示该旧对象依赖数为 0。当前重编的 dual_simplex_api.cpp.obj 依赖数为 254，说明当前配置已能解析头依赖，缺陷是继承的旧缓存没有有效依赖图，尚不能归因于现在的语言设置。先清除项目自身旧对象并重编，检查头依赖跟踪，再重新执行固定验收。此前数值结果无效，不调整数学模型。

### R3 首轮偏差与 R3a：冷启动的有限内点距离

惯性检查收紧后 HS071 仍 0/5，第三步失败，目标 16.8730016761，与原失败轨迹相同。实现确实拒绝缺失的惯性，惯性校正仍不能消除极差的冷启动尺度；这属于“惯性足以恢复收敛”的假设不成立。边界点仅 nextafter 内移，slack 接近舍入分辨率，mu/s 产生巨大的屏障曲率。成本模型并非本次主要原因。

RATIONALE

- Model/algorithm: Wächter–Biegler (2006), §3.6 initialization：将冷启动盒内点与边界保持有限距离，p_l=min(k1*max(1,|l|),k2*(u-l))，p_u 同理；单侧界只用第一项；k1=k2=0.01。
- Claim: 完整冷启动使用上述投影，保留已恢复可行点和显式 preserve_initial_point 的既有策略。严格内点仍以 nextafter 处理不可表示的窄区间。
- Cost model: O(n) 标量运算；初始化后不增加每步成本；非凸全局收敛不能由一次投影保证。
- Prediction: HS071 5/5 达到原定目标/原始残差阈值；初始盒 slack 至少约 0.01（当前约 1e-7），预计相同乘子下 mu/s 至少降低 1e5 倍。运行时间不作加速承诺。
- Assumptions: 冷启动允许移动 x0；边界宽度非零；原局部 NLP 模型与导数正确。
- References: Wächter and Biegler, Mathematical Programming 106 (2006), §3.6；本地 ipopt 初始化 bound_push/bound_frac 默认值；本节推导与 hs071_inertia.log。
- Validation: 与 R3 完全相同；额外检查 preserve_initial_point/Phase-I 相关 test_ipm_solver。

## R4：Auto 的直接 IPM 保障路径与协作式截止时间

RATIONALE

- Model/algorithm: 两算法竞争调度，已知可靠的直接 IPM 与原生双单纯形并发，首次成功后协作取消并回收线程。预算统一为调用入口的 steady_clock deadline。
- Claim: Auto 不再将唯一 IPM 分支消耗于预处理后不可靠的约化问题；直接 IPM 自身仍保留有数值审计的鲁棒性重试。返回前回收所有工作线程，消除持续运行的 detached 任务。
- Cost model: T_auto≈min(T_dual,T_direct)+T_cancel，两个工作线程；取消延迟至下一个检查点，MKL/外部同步分解不能被强行中断，因此不声称硬实时截止。
- Prediction: greenbea 由 0/3 准确变为 3/3；预计 3–6 秒（原直接约 2.73 秒，给竞争及取消预留约一倍），在 15 秒预算内完成。所有模型无需推测预处理机会。有限预算到期不再开始下一轮 IPM 重试。
- Assumptions: 当前直接 IPM 全集 270/270 的成功率能保持；两个线程资源竞争不超过上述区间。操作系统调度与库调用具有不可中断时间段。
- References: docs/archive/lp_kernel_selector_2026-08-11.md 的 portfolio 模型；Windows 基线 greenbea_auto 独立进程和直接 IPM 数据；本节截止时间模型。
- Validation: greenbea --solvers native-auto --repeat 3 --time-limit 15；全集 native-auto 90×3，阈值 1e-5/1e-7；极短预算测试确认有限返回、线程回收、不把立即失败错误改写为超时；全量 CTest。若时间预测不符，按协议重新分析取消点及分解成本。

### R3a 偏差与 R3b：恢复滤子的 h-type 步合同

有限内点投影后仍 0/5，但对偶残差从约 6.4e4 降到 1.86，初始化尺度得到改善。trace 第 4 步 Newton 四块残差均小于 1e-15，原始残差已降至缩放坐标 0.122448，代码却在尝试线搜索之前仅因 barrier slope 非负而直接返回 accepted-step collapse。按顺序检查，这是全局化实现不忠实于滤子模型，而非浮点成本问题。

RATIONALE

- Model/algorithm: Wächter–Biegler (2006), §2 filter line search：不可行点的 h-type 步通过约束违反量 theta 的充分下降被接受；仅 f-type switching 步要求障碍函数 Armijo 下降。
- Claim: 删除进入线搜索前的 slope>=0 提前失败；保留已有 filter、switching、Armijo、SOC 及线搜索失败后的 restoration。正 slope 不能推出线搜索必然失败。
- Cost model: 每个原来提前中止的方向增加实际 trial 评估 O(callback cost)，可能避免完整 restoration 子问题；不改变因子分解。
- Prediction: HS071 由 0/5 改善为 5/5，目标误差与原始残差门槛不变；不预测时间加速。
- Assumptions: 剩余 h-type 判据符合约束下降合同；回归需覆盖此前 OPF handoff 和线搜索审计。
- References: 上述论文 §2；本地 barrier_descent_slope、parameterized_switching_filter 和 filter_accepts 实现；hs071_boundpush_trace.log 第 39–44 行。
- Validation: R3 相同；全量 test_ipm_solver 与 CTest，不能用返回 success 替代原坐标审计。

## R5：分离验证 MKL 线程与 MSVC 编译选项

RATIONALE

- Model/algorithm: Amdahl T(p)=T_serial+T_parallel/p；LTCG 跨编译单元内联与去虚化；PGO 以训练分布指导布局与分支优化；AVX2 为 x64 向量运算提供显式可选目标。
- Claim: 各选项保持默认关闭/原线程选择；每次只改变一个选项，对同一修复版本复测，不把条件预测当实测收益。PGO 使用每个最终可执行文件独立的 PGD，USE 缺失 profile 必须失败。
- Cost model: MKL 线程只能降低实际进入并行分解/BLAS 的部分，不能降低 Eigen/图构建串行开销；编译优化只作用项目核心，不重编或声称优化已提供的 MKL 二进制。
- Prediction: 4 线程相对 INTEL/1 的三长尾合计时间降低 25%（假设约 1/3 工作有效并行）；LTCG 全集合计降低 3%；AVX2 降低 5%；PGO 在独立 holdout 降低 5%。这些都是待验证假设，非交付保证。原“长尾各减半则全集减 31.54%”保持条件推算。
- Assumptions: 电源策略不变、无同时运行的求解任务、训练覆盖目标分支但与 holdout 案例隔离；AVX2 仅在支持 AVX2 的 x64 机器使用。
- References: 本节 Amdahl 代数模型；本机 link /? 的 /LTCG、/GENPROFILE:PGD=filename、/USEPROFILE:PGD=filename；cl /arch:AVX2；基线 profile 分解占比。
- Validation: 线程 1/2/4/8，每档长尾与 NETLIB 原生 IPM 90×3，保持 1e-5 目标/1e-7 原始残差；编译项同一顺序独立比较且记录命令和构建标识。PGO 训练 afiro,adlittle,blend,sc50a,sc105；holdout 是其余 85 例。NLP 与全量 CTest 是候选默认值的额外必要门槛。未通过准确率或收益不明确的配置不升级为默认。

### R3b 验证：KKT 收敛已恢复，目标误差检查尚差 0.6%

HS071 原生现在 5/5 返回成功，12 次迭代，原始残差 6.449e-12，对偶残差 1.721e-12，互补性 5.029e-7；目标误差 1.006e-6，仍未通过预先规定的 1e-6 绝对目标误差门槛（0/5 accurate）。不能把 success 等同 accurate，也不放宽目标门槛。

偏差顺序：实现通过 Newton 残差与实际 KKT 门槛；成本不解释该差异；不成立的假设是“逐项 1e-6 KKT 容差推出 1e-6 目标误差”。即使凸问题，目标差控制量也是互补积之和而非最大单项；一般非凸问题更无此保证。

R3c 验证方案（不改变求解器默认容差）：为 HS071 benchmark 增加显式第二参数求解容差，并同时施加到 Native 与 Ipopt。以 1e-7 求解、仍以原 1e-6 目标误差/原始残差验收；预期 5/5 accurate，约多 1–5 步。命令 nlp_open_benchmark.exe 5 1e-7。原 1e-6 结果保留并报告，不与不同容差的原始基线直接作速度比。引用：本节互补积总量解释、Wächter–Biegler §2 的 KKT 停止条件。CTest 用更严格求解容差固定数值回归。

### R2 全量回归的合同调整

首轮 CTest 为 19/20；唯一失败为旧 differential transaction 测试要求自由变量边界变化仍在原矩阵上增量更新。正确的 x=p-q 转换在有限下界出现后可能改变列数，与该旧测试假设冲突。R2 预先已规定此时增量 API 返回 false 请求完整重建，因此不是接受阈值下降：差分测试改为有限下界模型，另保留自由变量 transaction 必须拒绝且不改状态的断言。负自由变量/上界单侧/无界射线/完整重建检查仍覆盖原问题。

### R1 Intel 运行时发现的闭包修正

INTEL 首轮配置发现 BuildIpopt 的备用 MKL 发现分支仅在系统 oneAPI 中寻找 OpenMP runtime，即使 MIPSOLVERS_MKL_ROOT 已指定。Dependency 分支也未完全排除既有缓存。必须由显式根控制 libiomp5md.lib 与 bin/libiomp5md.dll，缺失时失败。系统与 staged 文件 SHA256 完全相同（DLL EE527C978999653EB18EDC3EF372B4E95D4AD9A7691B6D08817BE9D38C7D0A02，LIB F282D5C65DD65FF089DE7047BD5ADF901AD2356E14A9333F7BB122859F8F1EBF），因此当前线程测量数值内容有效；交付验证仍需重新配置并确认引用来自 staged 根。

## R6：LP 分解计时与 Windows 2/4 线程稳定性（实施前协议）

RATIONALE

- Model/algorithm: steady_clock 的嵌套区间分解；装配、符号分析、数值分解采用 exclusive elapsed time，嵌套子项从父项扣除；retry 是 inclusive 子集，不再与前三项相加。每个 solve_lp_impl 变体独立计数，含失败变体及冷初始化。
- Claim: 补齐 Windows normal/augmented 路径、稀疏后端内部延迟分析与重试计时；不改变矩阵、求解顺序、容差和停止规则。记录的是后端 API 的工作时间（含格式准备），不是 MKL 内部 FLOP 时间。未覆盖的求解/残差/全局化时间归 other，不伪称为分解。
- Cost model: 开启时每个分析/装配/分解/重试边界 O(1) 时钟及累加；关闭时只检查 thread_local 指针，不读时钟、不输出。预测开启开销低于 2%，相同线程下目标、残差和迭代数一致；不预测算法加速。
- Assumptions: 在单一 Windows 工作站上串行运行测量进程；设置 OMP_NUM_THREADS=1、MKL_DYNAMIC=FALSE，INTEL 2 或 4；电源计划保持原设置。无额外 affinity 或 CBWR 改动，避免混入多项干预。
- References: 本节区间分解恒等式 T_parent_exclusive=T_parent_elapsed-sum(T_children_elapsed)；R5 Amdahl 成本模型；基线 dfl001 setup=4532ms 而 fill=factor=0 的计时缺口。
- Validation: 同一 Release 二进制，先各线程预热 1 次，再进行 20 个配对区组，奇数组按 2/4、偶数组按 4/2，每进程 --cases dfl001,greenbea,maros-r7 --solvers native-ipm-direct --repeat 1 --time-limit 15 --max-iterations 100000；开启 MIPSOLVERS_LP_FACTOR_TIMING。得到每案例每线程 20 个独立进程样本，首次预热单列，不能声称每进程 first-call 是完全 warm。
- 对照：每档另外 3 次 timing-off/on 交错配对，检查观测开销及数值一致性。全 NETLIB 各线程 IPM/Auto 各 3 次，目标相对误差<=1e-5、归一化原始违反<=1e-7；NLP HS071 5 次、求解容差1e-7、目标绝对误差<=1e-6。CTest 在最终候选下运行。
- 统计：每案例 median、nearest-rank P95=第19个/20、max、IQR/median；区组的三案例耗时和同样计算，报告 peak working set 与进程墙钟（不混入 kernel runtime）。稳定性优先：候选须零失败/超时，所有案例 P95 不劣于另一档 10%；若 4 线程在此门下 aggregate median 至少快 5% 才推荐 4，否则推荐占用更少的 2。所有结论限于本机和当前构建，不修改全局默认线程数。
- 原 R5 4线程对1线程降25%的预测需先根据已保存数据对照，偏差写回本节；本次 4 对2 的速度假设为 5–15% 降时，不作收益保证。31.54% 始终是“三案例各减半”的条件预测。

### R5 实测偏差复核（R6 前置）

已保存的 INTEL 1/2/4/8 数据，三长尾的各例中位数之和依次为 11362.09 / 6916.27 / 6723.73 / 7125.53 ms。4 对1降时40.82%，高于原预测25%，偏差超过预测的50%。先核对实现：相同算法与准确门槛均通过；成本模型中可并行占比估计偏低，dfl001 的 PARDISO 分解由6318.94降到2166.58ms（65.71%），说明三分之一的并行占比假设不适用。另 greenbea 4线程比2线程更慢（3566.53 vs2504.21ms），是需要配对重复验证的资源/轨迹现象。R6 不据三次样本确定默认值，以预先固定的20次协议复验。

### R6 开关计时对照的偏差调查

同2线程的3个配对，三例总时间变化为 +0.162%、+1.105%、+0.609%，中位数+0.609%。但逐位目标/残差与迭代一致的预测未成立。先查实现：新增 scope 只读 steady_clock 并写 thread_local 统计，不写矩阵/选路阈值；数值表达式与调用顺序保留。再查机器与假设：同样 timing=off 的3个独立进程，greenbea 迭代数已经分别为46/50/46，故相同线程配置本身不保证相同轨迹；线程运行时归约/调度以及已有自适应后端的时间比较是待区分来源，现有观测不足以单独归因。4线程首对 on 比 off 快5.69%，也不能解读为计时带来加速。

修正解释：开关对照报告“观测差异”，不声称已隔离出纯探针成本，也不以数值逐位相等验收多线程运行。保留原先准确门槛与20次稳定性分布验收，记录每例迭代数集合。若需位级复现，必须作为独立 CBWR/固定后端实验，不能在当前2/4比较中偷偷加入设置。本次不更改求解算法或线程归约方式。

### R6 二十组结果及预测偏差复核

同一二进制下两档各60/60准确；三例区组总时间中位数 2线程7182.49915ms、4线程5853.91935ms，实测降18.497%，高于5–15%预测区间。相对区间中点10%的偏差超过50%，按保守口径触发复核，不修改验收门槛。

1. 实现保真：实际 MKL 最大线程数分别为2/4，二进制及算法相同，所有原模型审计通过，计时分类闭合；没有混入编译项或放宽容差。
2. 机器/成本模型：dfl001 数值分解中位数2426.32→1402.82ms，减少约1023.50ms；三例区组中位数差1328.58ms。这一量级表明收益主要来自本来占72.79%时间的分解并行段，而非全部工作均匀加速。各项中位数不可直接加和作精确贡献分摊。
3. 假设复核：greenbea 每次两变体，2/4线程返回的迭代数范围46–419/43–302，固定工作量假设不成立；该例中位数仅降0.73%，P95降7.40%。dfl001与maros-r7迭代数分别固定44/21，更适合分析固定工作并行成本。
4. 修正模型：按案例采用 T(p)=T_assembly+T_symbolic+T_numeric(p)+T_other(p,trajectory)，并把变体及迭代工作量单列。原5–15%粗估低估dfl001数值分解的有效可并行占比；greenbea波动来源尚未唯一定位，不能宣称已解决数值非确定性。

4线程各案例P95均未超过2线程的1.1倍，区组合计中位数收益超过5%，因此通过性能候选门。最终推荐还须本轮两档全集、NLP及候选配置CTest完成。开关计时4线程三对观测差异为−5.687%、+3.362%、+1.338%，中位数+1.338%；结合2线程+0.609%，仍不能在轨迹噪声中证明纯插桩开销低于2%。

### R6 全集的适用性复核

两档 NETLIB IPM/Auto 各270/270准确，HS071 Native/Ipopt各5/5准确。但关闭计时的全集中，4对2线程IPM总耗时仅降4.53%，Auto总耗时增加10.44%。预先性能预测针对三长尾direct区组，不针对Auto，不能把18.50%推广为全集或Auto收益。

先核对实现与测量：二进制SHA256未变，实际线程数正确，全集是同进程每例3次且先2后4，与长尾交替独立进程协议不同。再核对成本/假设：Auto中greenbea三次合计增加6655.17ms，其中一次4线程为9416.82ms，返回迭代44；其另两次为3213.22/2517.82ms。Auto竞争线程回收与资源争用、内部恢复轨迹都可能使返回的迭代数不能解释墙钟。当前未记录Auto分解及取消等待，不能把9416.82ms唯一归因于取消延迟。结论限定为本机LP direct的INTEL/4建议；Auto保持2线程的保守运行建议，不修改全局默认。后续Auto分解/取消时间和并发吞吐应另立协议。

最终全目标重链接成功，基准二进制SHA256不变；INTEL/4、OMP=1、MKL_DYNAMIC=FALSE、计时关闭，`ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 1` 为20/20通过、305.41秒。R6规定的本轮验收全部完成；运行建议及未解决的轨迹/开销问题见 `windows_lp_stability_2026-09-11.md`。

## R7：合并到 Windows release（实施前协议）

- Model/algorithm: 将9652ddb9的R1–R6修改合入Windows release父提交2c400fcf，保持release已有NLP KKT同余缩放、warm-start/API和带验证的KLU排序/重分解。KLU置换控制与计时scope正交，保留环境排序设置并仅包围实际analyzePattern；两边SDK命名依赖节点实现相同，只合并注释和运行时部署。
- Claim: 不删除任一父提交的公开API或回归断言，Windows专用release接收修复；不修改macOS main引用。保留两边的历史验证记录，不把父版本结果冒充合并结果。
- Cost model: 数值语义为两个已记录改动集的组合；计时成本沿用R6，新增合并本身不引入求解循环。不能从Git自动合并推导数值等价。
- Prediction: 0个丢失API、0个放宽验收阈值；Windows完整构建及注册CTest全部通过、NETLIB两档各540/540准确、HS071每档10/10准确；不预测合并比父版本更快。
- Assumptions: 同一本地MSVC/MKL依赖闭包；release既有下游Simulation失败记录仍保留，当前任务不宣称修复未复测的下游案例。
- References: R1–R6；手册9章Windows/main integration validation的KKT变换、边界/终止合同；KLU现有排序与固定模式refactor合同。
- Validation: 重建全部Release目标；tools/windows_lp_stability.py的stability及gates阶段，独立目录reports/windows_release_merge_20260911；候选线程下完整CTest；新前缀cmake install、不同目录consumer-example配置/链接/执行并检查objective=9。对2/4推荐沿用R6固定P95和5%门槛，旧18.50%及31.54%不作为合并实测。失败先调查实现与合并保真，不能放宽门槛。
