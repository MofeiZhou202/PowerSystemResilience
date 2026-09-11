# Windows 离线评估：预注册协议与实测记录（2026-09-11）

## 结论

本机可以依靠已有本地源码、工具链和数值库完成 Release 增量构建及运行，但当前完整功能配置尚不具备可靠的离线 SDK 交付条件。19/19 CTest 通过；进一步基准发现 Native-Auto 的错误最优解与超时、原生 NLP 的 HS071 失败，以及完整 SDK 的消费失败。性能优化应先修复这些正确性与交付问题，再集中处理原生 LP 的三个长尾。不能把历史机器的多线程成绩作为当前 Windows 配置的性能承诺。

本次所有操作均在本机进行，未访问外部网络，未安装或下载依赖。没有更改求解算法或放宽容差。新增评估文档并将摘要链接加入手册；评估使用的构建缓存已从 INTEL 调整为 SEQUENTIAL。

## 范围与约束

用户要求系统评估整个 MIPSolvers 在 Windows 上的表现并提出性能增强意见，全部代码本地化，不访问外部网络。本次不改变算法、容差或求解器实现。只使用本地工具链、源码、静态数值库和数据；不运行下载工具，不使用 vcpkg 自动安装，不启用 Gurobi。oneMKL 为本地闭源二进制，不代表全部依赖都有可重编译源码。

起始提交：`75c6e1922839d4775c27eacda833df386e95f476`，初始工作树干净。规定的 `~/.agents/skills/theory-guided-coding/SKILL.md` 不存在，在本机 `.agents` 与 `.codex` 搜索未找到。本次不开展依赖该技能的算法实现；评估遵守仓库明示的理论、预测、准确性和报告约束。

## 测量前固定的协议

- 平台：本机 Windows 11、MSVC x64 Release；记录 CPU、内存、编译选项和二进制哈希。
- 先构建当前源码并执行已注册全量 CTest，逐个运行，默认单测试 180 秒；CTest 时间只用于正确性回归，不用于求解器排名。
- LP：本地 NETLIB 90，主比较 `highs-simplex,highs-ipm,native-auto,native-ipm-direct`，每例每方法重复 3 次，求解时限 15 秒，最多 100000 次迭代。若方法键不支持，以程序明确支持的公共 auto 路径替代并记录。计时包含求解调用，不包含数据加载；固定 `OMP_NUM_THREADS=1`、`MKL_NUM_THREADS=1`、`MKL_DYNAMIC=FALSE`。所有性能任务串行，不与编译或其他基准重叠。
- 正确性采用 `docs/manual/09-testing-benchmarks.md` §9.3.2：原模型目标相对误差 ≤ 1e-5、归一化原始可行性违反 ≤ 1e-7，同时要求 success。保留失败、超时与不准确结果，不以退出码替代审计。
- 性能汇总：每例 3 次中位数，分别给出准确配对集合的几何均值速度比、所有尝试的总耗时以及长尾；毫秒级样本只作参考，不外推工业规模。
- NLP：本地 HS071、native 与本地 Ipopt，交替执行顺序，共 5 次；沿用程序原有准确性审计。锥：已有 quick suite；MILP：已有 smoke 及本地可用数据。覆盖不足必须注明。
- 不更换浮点模式，不宣称未经实际 A/B 的优化收益；双单纯形后续纯性能改动需保持位一致 pivot path，算法变化需全套准确性验收。

## 理论假设、量化预测及验收

参考仅使用本地文档，不联网查文献：`docs/manual/06-numerical-methods.md`、`docs/manual/09-testing-benchmarks.md`、`docs/archive/lp_tail_elimination_2026-08-18.md`、`docs/archive/ipm_structural_performance_2026-08-06.md`。

1. **线程配置**：串行 MKL 不会因设置 MKL_NUM_THREADS=4 自动成为并行 MKL，预测 MKL 核本身收益为 0%。若未来提供线程化构建，使用 Amdahl 模型 T(p)/T(1)=(1-f)+f/p+h；例如实测可并行占比 f=0.6、p=4、忽略额外开销 h 时，条件预测降时 45%，不是本次已证实结果。
2. **稀疏结构与长尾**：分解重用、预处理缩减和稀疏访存优化应优先针对占比最大的阶段。若某阶段占总时 40%，阶段降时 25%，条件预测整体降时 10%。须先 profile 确认占比；不能从既有历史机器的加速比直接推断本机收益。
3. **构建可重复性**：两个 build 目录共用源码树 tests/Release，预测交错构建会使测试指向最近一次覆盖的二进制。未来将输出隔离到各 build 目录，应把跨配置产物覆盖降为 0；这是可靠性目标，不是运行时间收益。
4. **数值结果**：无算法变更，预期测试正确性不降低。任何失败先检查构建/实现一致性，再检查机器成本模型、假设和理论，记录后才能考虑改代码；不得调宽容差掩盖失败。

## 已确认的配置事实

原 `build/windows-msvc-release` 缓存选择 INTEL 线程层。当前 CMakeLists.txt 的 Windows SDK 检查无条件要求 SEQUENTIAL，原配置增量构建因而失败。随后显式设置 SEQUENTIAL、PREBUILT_THIRD_PARTY=OFF、GUROBI=OFF，在本机离线构建成功。保留第一次失败与后续成功日志。

原始证据目录：`reports/windows_eval_20260911/`。后续实测、限制和建议在完成测量后追加。

### 执行前协议修正：Native-Auto 的资源口径

源码检查确认 `NativeAutoLPAdapter` 会创建两个 detached worker，返回赢家时不等待输家结束。它并非单线程方法，且同进程连续求解可能存在取消滞后。为避免污染其他方法，主 LP 性能比较改为 `highs-simplex,highs-ipm,native-ipm-direct`；Native-Auto 单独进程运行、单独报告，不能称为等核数加速，也不与主组的同进程负载混合。此修正在运行 LP 性能数据前固定。

### 测量中异常记录：自动竞速不保证等于最快路径

主组中 greenbea 的直接原生 IPM 三次均准确，中位数 2728.50 ms；单独运行的 Native-Auto 首次 greenbea 却在 22489.42 ms 返回 Time limit（配置 15 秒）。不能继续把 auto 理解为直接求解器耗时的无开销最小值。

按失配协议先检查实现：auto 的 IPM worker 带机会估计驱动的原生预处理，与本次 presolve-free direct 对照并非相同工作；两个 worker 为 detached，赢家发布不等于输家已停止。其次检查成本模型：存在双核竞争、模型复制及取消滞后，理想 min(T1,T2) 模型缺少这些成本；当前 MKL 为串行且机器为混合核心。然后检查假设：等核数、同算法配置和硬时限均不能从 auto 名称推定。尚不能仅由日志断定哪一项是主因。后续做单案例独立进程复核，保留失效记录，不修改算法或容差。

## 1. 实测环境与可复现性

| 项目 | 实际配置 |
|---|---|
| 提交 | `75c6e1922839d4775c27eacda833df386e95f476` |
| 系统 | Windows 11 专业版，10.0.26200 |
| CPU | i9-12900H，14 核 / 20 逻辑处理器；平衡电源计划 |
| 内存 | 系统可见约 31.67 GiB |
| 工具链 | VS 2022 Build Tools，MSVC 19.44，Ninja Multi-Config，C++20 |
| 核心优化 | `/O2 /Ob2 /Oi /fp:fast /DNDEBUG`，动态 MSVC 运行时 `/MD`；IPO、PGO、native arch 均关闭 |
| 数值依赖 | 仓库本地 oneMKL 2026.0.1，LP64 + sequential + core 静态库；本地 SuiteSparse/CHOLMOD/KLU/UMFPACK |
| 求解后端 | 本地 HiGHS 1.14.0、SCIP、Ipopt、PaPILO；Gurobi 关闭 |
| 线程 | 主 LP、NLP、MILP 的 OMP/MKL 环境均为 1；MKL_DYNAMIC=FALSE；Native-Auto 自建两个 worker |
| 构建覆盖 | 当前源码增量重配置及构建，32 个构建动作，22.816 秒；不是清空缓存的全量编译时间 |

本次配置的 OpenMP 已启用，但 MKL 仍是 sequential；两者不可混称。没有启用 `MKL_CBWR`。当前 CHOLMOD 编译文件未出现 `NSUPERNODAL`，configure 已选用本地 MKL，不能把本次长尾直接归因于历史的 supernodal 缺失问题。

完整缓存、机器信息、二进制 SHA256、数据哈希审计、退出码与日志位于 `reports/windows_eval_20260911/`。`netlib_main.json` 的 provenance 包含实际命令、提交、MSVC 1944、Release/NDEBUG 和 x86_64。保留所有失败行。

## 2. 正确性与性能结果

### 2.1 全量 CTest

**19/19 通过**：unit 14、integration 3、benchmark 2，串行总 wall time 281.28 秒。未启用 SCUC 和 Python；不能据此声明这些配置通过。MUMPS 与 SuperLU 在本次 configure 中不可用。

单进程测试的端到端耗时约 2.45–20.63 秒，不能当作求解内核耗时。后续热态 `netlib_solver_benchmark --help` 三次 wall time 19.33 / 20.79 / 14.79 毫秒，说明早先的秒级开销不能归结为稳定的启动成本。没有采集 ETW，冷缓存、文件扫描、系统负载等具体贡献仍未归因。该程序的帮助分支按现有实现退出 1，日志正常打印 Usage；这不是求解失败。

### 2.2 NETLIB 90：每例每方法三次

下表“中位数之和”是先对每例三次取中位数，再相加；“全部运行耗时”保留所有三次尝试。均为程序内部求解计时，不包含加载与进程启动。准确例数要求该例三次全部通过原模型审计。

| 路径 | 准确运行 | 准确例数 | 90 例中位数之和 | 全部运行耗时 |
|---|---:|---:|---:|---:|
| HiGHS simplex | 270/270 | 90/90 | 15.675 s | 47.127 s |
| HiGHS IPM，关闭 crossover | 264/270 | 88/90 | 8.162 s | 24.530 s |
| 原生 IPM direct | 270/270 | 90/90 | 15.096 s | 45.452 s |
| Native-Auto，独立批次、双 worker | 264/270 | 88/90 | 36.831 s | 108.600 s |

HiGHS IPM 的总时间含失败/不准确案例，不能将其解释为完成 90 个准确解所需时间。Native-Auto 资源、执行批次和取消行为不同，不纳入主组等资源排名。其成功案例较短的时间不能抵消错误结果。

原生 IPM direct 相对 HiGHS IPM，在双方三次均准确的 **88 对**上，配对中位数的几何平均速度比 `T_HiGHS / T_native = 1.354x`；相对 HiGHS simplex 的 90 对为 **0.862x**。原生 IPM 在许多中小例上较快，但长尾使其总时间仍较高。其最大目标相对误差 `1.250e-6`、最大归一化原始违反 `8.689e-8`，通过既定阈值。

| 原生 IPM 长尾 | 三次中位数 | 占原生 IPM 中位数总和 |
|---|---:|---:|
| dfl001 | 5294.89 ms | 35.08% |
| greenbea | 2728.50 ms | 18.07% |
| maros-r7 | 1499.49 ms | 9.93% |
| pilot87 | 1197.93 ms | 7.94% |
| pilot | 633.47 ms | 4.20% |

前三例合计 **63.08%**，前五例合计 **75.22%**。前三例的迭代数在三次运行中分别稳定为 44、44、21；本轮长尾不是重复运行迭代次数剧烈波动造成的。

### 2.3 失败的独立复核与边界

- **Native-Auto / tuff**：三次都返回 Optimal，但目标相对误差 `1.88246e-2`；原始可行性违反仅 `2.728e-12`。独立 tuff 批次再次 0/3 准确，debug 日志三次 winner 都是 NativeSimplex。直接选择 `native-dual-simplex`（含 HiGHS presolve）也 0/3 准确；`native-ipm-direct` 为 3/3 准确。问题已缩小到原生双单纯形/HiGHS 预处理组合及发布门，尚未判定更底层的具体根因。原始可行性合格不能证明最优性，发布赢家前需要原模型的最优性证书审计。
- **Native-Auto / greenbea**：全组 22.489 / 25.559 / 26.003 秒均超时；独立进程仍在 **21.232 秒**返回 Time limit，配置均为 15 秒。独立 debug 显示机会估计启用原生 presolve，最终发布的是 NativeIPMLP 失败结果。主组记录 presolve_ms 约 7–9 ms，因此不能把全部延迟归为“预处理本身耗时”；应检查变换后的系统、重试及取消检查粒度。源码显示 IPM 的取消/时限主要在迭代边界检查，自动选择器使用无时限的条件变量等待发布；这说明硬时限并未由外层强制保证。
- **HiGHS IPM**：ganges 三次返回 Optimal，但目标相对误差 `1.80139e-2`、归一化原始违反 `4.898e-6`；greenbea 三次 Unknown。结论仅针对 benchmark 的关闭 crossover 配置，不泛化到所有 HiGHS 路径。
- **原生 NLP / HS071**：禁用外部 fallback，5/5 次失败，均在 3 次迭代后返回，目标约 16.8730、原始违反约 0.8125。不能把失败路径的 3.211 ms 中位时间计为“快”。本地 Ipopt 5/5 次准确，目标约 17.0140174709，中位 2.633 ms，8 次迭代。原生 LP 与原生 NLP 是不同路径，前者 NETLIB 成功不替后者背书。当前输出不足以归因到某一种 KKT、正则化或全局化缺陷。

### 2.4 锥规划与 MILP

SOCP/SDP quick suite，4 个模型，线程参数 1/4，独立批次重复三次，**24/24 返回 optimal**。最大相对 gap `4.247e-7`；最大最终 KKT backward error `1.444e-7`。此处是锥基准的状态/gap 口径，不等同于 NETLIB 的目标参考值审计。

| 模型 | 1 线程中位数 | 4 线程中位数 | 速度比 |
|---|---:|---:|---:|
| socp_n100 | 1.455 ms | 1.216 ms | 1.196x |
| socp_n400 | 6.805 ms | 6.790 ms | 1.002x |
| sdp_p10 | 2.598 ms | 2.475 ms | 1.050x |
| sdp_p20 | 32.417 ms | 27.299 ms | 1.187x |

设置的是锥求解器的 OpenMP 参数，MKL 始终串行；这些小样本未呈现接近 4x 的扩展性。顺序固定为 1 后 4，且样本很短，缓存和计时噪声可能影响数字，不能据此发布线程数自适应阈值。未覆盖等式行、chordal SDP 或百万变量 sparse_lp 专项。

MILP 使用本地 UC_6bus_3G_4T（212 变量、28 等式、384 不等式），7 个既有配置 × 3 次，**21/21 成功**，目标均约 993830.14961442，最大 gap `1.172e-15`。Baseline 中位 30.708 ms，含 cuts / A/B 的若干配置约 16.9–17.1 ms。**所有运行均在根节点结束，nodes=0**，所以不能证明 B&B 大树性能或 MIPLIB2017 级能力；这些仅是根节点冒烟结果。

## 3. 离线本地化与 Windows 交付审计

| 检查 | 实测结论 |
|---|---|
| 数据本地化 | 90/90 NETLIB SHA256 与本地清单匹配 |
| MKL 本地化 | SHA256SUMS 的 240/240 文件匹配；实际链接选用仓库内静态包 |
| 下载行为 | 本次未运行 fetch、pip、vcpkg、联网文档查询或外部服务；现行主依赖解析为本地路径 |
| 独立预编译包 | 本机 `third_party/install` 不存在；windows-offline-release 默认要求该包，不能认为仅选此 preset 就可构建 |
| 文件分发完整性 | `*.lib` 被 .gitignore 全局忽略，MKL manifest 虽在版本控制中，mkl_core.lib 不在；仅 Git checkout 不等于可离线构建交付包 |
| 完整 SDK 安装 | 当前配置 `cmake --install` 成功，安装后目录改名仍能找到配置；723 个文件，约 982.58 MB |
| 外部消费者 | 生成失败：导出的 `mipsolvers::ipopt_local` 引用不存在的 `MIPSolvers::MKL`。需要修复导出依赖闭包并重新验收 |
| 合并静态库 | 默认 bundled 目标把 GNU `ar -M` 交给 MSVC lib.exe，产生 LNK4044，构建退出 0 却没有 `libmipsolvers_bundled.a` |
| 线程配置 | CMakeLists.txt 的 SDK 检查无条件要求 SEQUENTIAL；现行 INTEL 配置被拒绝，手册的线程化用法已与当前提交不一致 |

本地化不等于拥有全部第三方源码：oneMKL 是本地闭源二进制。若部署还要求全部依赖能够从源码重建，需要另行验收本地 Fortran + MUMPS + reference BLAS/LAPACK 路径；不能承诺其性能与优化 MKL 相同。本次没有测试全新断网机器，也未通过系统防火墙抓包证明所有可选第三方组件都无联网能力；结论范围是实际执行的本地配置与操作。

本次 netlib EXE 为 123.93 MB。主 LP 运行中的一次 OS 采样显示此前 peak working set 约 66.13 MB，auto 的后续采样约 329.93 MB；采样时点不同，不能据此给出严格峰值内存速度比，需在后续隔离每例时采集完整 peak private bytes / working set 曲线。

## 4. 性能增强建议：按优先级与验收条件排序

### P0：先保证正确性、时限和可交付

1. 修复 tuff 的原生双单纯形/预处理组合，并让 portfolio 只发布经原模型原始/对偶可行性与 gap 审计的结果。不能只使用 success 字段。门槛：独立复现通过、90 例每例三次准确、全量 CTest 通过；使用结构/数值判据，禁止按案例名绕行。
2. 针对 greenbea 重建 portfolio 成本模型：包含机会估计、模型复制、变换后系统成本、并发竞争、取消滞后和 worker 生命周期。将统一 deadline 传到长初始化、预处理、稀疏分解前后及重试边界。若接口宣称进程级硬时限，则需可终止的隔离执行策略；线程内不能承诺任意外部库调用都即时中断。验收记录超限量与返回后的后台 CPU，并确保没有已结束调用持续堆积 worker。
3. 将 HS071 的纯原生 NLP 路径纳入回归，补足失败状态/方向/KKT/全局化诊断，依原推导定位后修复；在该能力通过前，已有本地 Ipopt 是本次验证过的 HS071 可用路径。
4. 修复 MKL imported target 的重建和 MSVC 合并库生成；新增能实际链接、运行并验证目标值的安装后消费者验收，以及声明产物存在性检查。分离构建目录下的 EXE，避免共享 tests/Release 覆盖。此类工作目标是消费成功率与产物一致性，不宣称求解加速。

### P1：围绕实测长尾优化数值阶段

单次额外 verbose profile 仅用于诊断，不混入三次主排名：

| 案例 | profile 分解 | 含义 |
|---|---|---|
| dfl001 | total 5417.01 ms，setup 4532.34 ms（83.67%），init 367.66 ms | 增广系统 setup 为第一热点；优先细分并优化装配/分解/重分解 |
| maros-r7 | total 1425.82 ms，factor 766.10 ms（53.73%），init 309.84 ms（21.73%） | 分解与初始化/符号分析比普通向量循环更值得投入 |
| greenbea | SPARSE 2129.81 ms，随后 AUGMENTED 698.49 ms | 首阶段的预测/校正和路径切换成本明显；应分析重试策略及原模型稳定性 |

增广路径当前打印 `fill=0 factor=0`，但 setup 数秒；这表明细分计时没有完整覆盖，**不能解释为分解免费**。先完善计时契约，再比较复用结构、符号分析、KKT 路径选择、稀疏数据布局等改动。已有实现包含若干重用机制，需先证明具体调用仍有冗余，而非重复实现缓存。

量化预测均为后续工作的条件预测，**本次没有实施这些优化**：

- 若前三个长尾各降时 50%，其他案例不变，则总中位数时间预计从 15.096 s 降到 **10.334 s**，降时 **31.54%**，约 **1.46x**。它是收益场景，不是已经测得的增强。
- 若只把 dfl001 的 setup 耗时减半，假设 profile 阶段占比可迁移，则该例预计降时 41.84%，整个 NETLIB 中位数总和降时约 **14.67%**。
- 若只把 maros-r7 的 factor 耗时减半，该例预计降时 26.87%，全集降时约 **2.67%**。
- 恢复可选本地 INTEL MKL 构建时，先解除“任意构建都必须串行”的打包耦合，明确随包携带 Intel OpenMP runtime，再用 1/2/4/8 线程实测。采用 Amdahl 模型，收益由实际可并行占比与额外开销决定；不能因 CPU 有 20 个逻辑处理器就期待 20x。

验收：固定源码、二进制、线程、数据与供电状态，交错 A/B 顺序，每例至少三次；统计中位数、尾部、全部尝试时间和准确率。若收益方向相反或与预测偏差超过约 50%，按实现一致性 → 成本模型 → 假设 → 理论检查并写回推导记录。原生双单纯形纯性能改动保持位一致 pivot path；算法变更执行完整准确性门。

### P2：编译器、x64 SIMD 与部署成本

- MSVC 当前核心已有 `/O2 /Oi /fp:fast`；native-arch 选项只作用于非 MSVC 分支，PGO 在 MSVC 分支明确拒绝，不能通过照搬 GCC 选项得到加速。可在保持浮点语义契约的前提下验证 MSVC IPO/LTCG 和独立 PGO 工作流，使用训练/留出问题集，达到至少 5% 可重复收益且准确性不退化才推广；5% 是建议验收门槛，不是收益预测。
- PRICE/BFRT 的部分实验 SIMD harness 是 AArch64/NEON，不是 Windows x64 AVX2 实现。先测 x64 热点的有效字节、带宽和分支成本，再决定 AVX2 是否值得；若瓶颈是随机访存，应优先布局和批处理，不能仅靠扩大 SIMD 宽度。归约顺序改变应按算法变化验收。
- 固定同一求解批次的线程预算，避免 Native-Auto 的两个 worker 再叠加多线程 MKL，或多任务并发产生过度订阅。混合核心机器需用实际线程/亲和性实验确定策略，本次没有修改电源计划或亲和性。
- 为现场交付建立本地依赖清单，包含源码、二进制、许可证、SHA256、工具链版本和实际安装消费测试。常驻进程与批量求解是否改善延迟，应分开测试冷态/热态启动、数据装载与内部 solve；不将本次 CTest 冷态耗时直接外推为服务启动成本。

## 5. 复现命令与证据索引

在已初始化的 VS 2022 x64 开发 shell 中，从仓库根目录执行；这些命令不需要网络。下面 `reports/windows_eval_20260911` 是本次原始输出路径，重跑时应改用新目录，避免覆盖证据。

```powershell
cmake -S . -B build/windows-msvc-release -DMIPSOLVERS_MKL_THREADING=SEQUENTIAL -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF -DMIPSOLVERS_USE_GUROBI=OFF
cmake --build build/windows-msvc-release --config Release --parallel 6
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
$env:MKL_DYNAMIC='FALSE'
$env:MIPSOLVERS_BENCH_GIT_COMMIT='75c6e1922839d4775c27eacda833df386e95f476'
ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 1 --timeout 180
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --solvers highs-simplex,highs-ipm,native-ipm-direct --repeat 3 --time-limit 15 --max-iterations 100000 --csv reports/windows_eval_20260911/netlib_main.csv --json reports/windows_eval_20260911/netlib_main.json
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --solvers native-auto --repeat 3 --time-limit 15 --max-iterations 100000 --json reports/windows_eval_20260911/netlib_auto.json
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --case greenbea --solvers native-auto --repeat 1 --time-limit 15 --max-iterations 100000
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --case tuff --solvers native-auto,native-dual-simplex,native-ipm-direct --repeat 3 --time-limit 15 --max-iterations 100000
$env:MIPSOLVERS_IPM_VERBOSE='1'
tests/Release/netlib_solver_benchmark.exe --data-dir tests/data --cases dfl001,greenbea,maros-r7 --solvers native-ipm-direct --repeat 1 --time-limit 15 --max-iterations 100000
Remove-Item Env:MIPSOLVERS_IPM_VERBOSE
tests/Release/nlp_open_benchmark.exe 5
foreach ($r in 1..3) {
  tests/Release/conic_benchmark.exe --quick --suite all --threads-list 1,4 --json "reports/windows_eval_20260911/conic_r$r.json"
  tests/Release/milp_benchmark_runner.exe --smoke --no-highs-adapter --json "reports/windows_eval_20260911/milp_r$r.json"
}
```

完整初始缓存保存于 `CMakeCache.txt`；上面的 configure 是针对既有缓存的本次调整，不是忽略其他配置的全新机器构建配方。ZLIB 使用 `third_party/zlib-install` 的本地静态库。原始执行顺序为 NLP → 全部 conic 重复 → 全部 MILP 重复；重跑复核性能时保持这一顺序。

安装消费复核使用 `cmake --install build/windows-msvc-release --config Release --prefix reports/windows_eval_20260911/sdk`，将该目录改名为 `sdk_relocated` 后，用 `cmake -S cmake/consumer-example -B build/windows-eval-consumer -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=<sdk_relocated绝对路径>`；失败信息保存在 `consumer_configure.log`。这是同机改路径的消费验证，不是全新机器部署认证。

主要证据：`ctest.xml` / `ctest.log`；`netlib_main.json` / `netlib_main_analysis.json`；`netlib_auto.json` / `netlib_auto_analysis.json`；`auto_greenbea_isolated.json`；`tuff_diagnosis.json`；`ipm_tail_profile.log`；`nlp_hs071.log`；`conic_r1..3.json`；`milp_r1..3.json`；`additional_analysis.json`；`build.log` / `build_sequential.log` / `consumer_configure.log`；`data_hash_audit.json` / `mkl_hash_audit.json`。
