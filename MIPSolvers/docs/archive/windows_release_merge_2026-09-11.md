# Windows release 合并验收（2026-09-11）

## 身份与分支范围

用户指定 `release/windows-self-contained` 为Windows分支、`main` 为macOS分支。本次将已推送的Windows修复分支合入release，不更新main，不回退release已有更新。

- 合并提交：`1d31f0ebcd1b90f2f1f5a0a579402203eef87998`。
- 第一父提交：Windows release `2c400fcf`；第二父提交：Windows修复及LP计时 `9652ddb9`。
- macOS `origin/main` 在合并前为 `b42a054052c223e8dfdf3871c667bfc8fe36a09f`。
- 二进制：`netlib_solver_benchmark.exe` SHA256 `49298e6c335fde0b221758811a4f5831d3a637b5c5ad44e5283ddb4cc41daea1`。
- 机器：Windows 11 / i9-12900H / 约32GB；MSVC 19.44 x64 Release，Ninja Multi-Config，`/O2 /Ob2 /Oi /fp:fast /MD`，本地oneMKL 2026.0.1 INTEL线程层。
- CMake：本地 `third_party/oneapi-mkl`；SuiteSparse、PaPILO、C++ OpenMP开启；SCUC、Python、IPO、PGO、native architecture关闭。环境 `OMP_NUM_THREADS=1`、`MKL_DYNAMIC=FALSE`、MKL线程分别2/4；不设置CBWR/affinity，保留Balanced电源计划。
- 原始证据：`reports/windows_release_merge_20260911/`（Git忽略；须单独留存）。包含构建日志、CMakeCache、合并/二进制身份及API保留审计。

本报告验证源码合并后的Windows配置，不将旧 `9652ddb9` 的18.50%收益或其他分支测试数字作为本提交结果。31.54%仍只是“三个原基线长尾各减半”的条件预测。

## 合并保真与冲突处理

实施前理论、预测和验收门槛固定于 [Windows修复推导R7](windows_remediation_2026-09-11.md)。预测为零API丢失、零准确门槛放宽、全部指定回归通过；不承诺速度提升。

三个显式冲突：

1. KLU延迟符号分析：保留release读取环境排序并设置KLU ordering的逻辑，再以R6计时scope包围 `analyzePattern`。
2. SDK `MIPSolvers::MKL`：两边实现相同，合并理论引用及INTEL运行时部署。
3. 测试手册：两边追加的记录完整保留，不删除release原有下游失败证据。

自动合并的NLP改动按R3逐项复核：惯性证书、冷启动内点距离及h-type滤子修复与release的KKT同余变换、warm-start、预算/API同时保留。release的公共problem/options/adapters头和完整 `test_ipm_solver.cpp` 与父提交逐文件差分为空。

## 复现命令

在x64 VS 2022开发者环境，从仓库根顺序执行：

```powershell
cmake --build build/windows-msvc-release --config Release --parallel 6
python tools/windows_lp_stability.py --stage gates --output reports/windows_release_merge_20260911
python tools/windows_lp_stability.py --stage stability --output reports/windows_release_merge_20260911
```

构建成功，含MSVC bundled archive。求解阶段不与编译或其他求解阶段并发。两档全集为每例3次×90例×IPM/Auto，HS071每档Native/Ipopt各5次；长尾阶段沿用20组交替顺序、P95与5%中位数收益门。

## 已知范围限制

release父提交的Simulation集成曾报告GUI Auto showcase和Native graph-reduced transport失败，证据仍在手册9章及 `windows-integration-evidence.json`。本次独立求解器/SDK验收不能证明这些下游失败已修复；Simulation不是本轮重测对象。SCUC/Python也不在当前构建覆盖范围。

## 全集与 NLP 实测

两档NETLIB均540/540准确，共1080/1080，无失败或超时；每档Native-IPM direct、Native-Auto分别270/270。原模型目标相对误差最大约1.250e-6，归一化原始违反最大9.978e-8，未放宽1e-5/1e-7验收门槛。

| 求解器 | 2线程270次总耗时 ms | 4线程270次总耗时 ms | 2线程各例中位数之和 ms | 4线程各例中位数之和 ms |
|---|---:|---:|---:|---:|
| Native-IPM direct | 82369.87 | 77202.28 | 26095.06 | 26802.05 |
| Native-Auto | 76493.25 | 65095.52 | 25077.86 | 21338.55 |

全集只按2后4顺序各运行一个进程、每案例重复3次，主要用于准确性验收。IPM总时间降低但各例中位数之和增加，说明不同统计口径不可混用；不据此独立决定P95或跨版本加速。

HS071每档原生/Ipopt各5/5准确，共20/20。原生16次迭代、目标绝对误差约1.908e-12；Ipopt8次迭代、误差约1.815e-8。两者求解容差1e-7，验收目标绝对误差/原始违反仍为1e-6。

本批绝对LP耗时明显高于9652ddb9的早先测量，例如2线程direct的dfl001中位数3281.88→7672.81ms，maros-r7为1078.90→2452.18ms，而迭代数仍为44/21。已核对两批CMakeCache中的MKL/BLAS/OpenMP/IPO/PGO/架构及优化开关相同，当前未观察到并发编译/其他求解进程；电源计划仍Balanced，电池状态记录为2、100%。现有证据不足以区分重新构建、机器运行状态和实现组合的贡献，不声称该差异已解释，也不通过改变电源/affinity掩盖它。R7没有速度改善预测，本次合并验收和运行建议只以其明确口径为准；旧18.50%不升级为本提交承诺。

## 二十组长尾与分解计时

两档每例20次，合计120/120准确，无失败或超时；预热单列。下表单位ms，P95为20个样本中的第19个。三例合计先在各区组内相加，再计算分布。

| 案例 | 2线程中位数 | 2线程P95 | 2线程最大 | 4线程中位数 | 4线程P95 | 4线程最大 |
|---|---:|---:|---:|---:|---:|---:|
| dfl001 | 4493.30 | 5030.45 | 5549.65 | 2531.94 | 3758.96 | 5038.82 |
| greenbea | 5775.81 | 9207.86 | 10227.81 | 4656.99 | 8184.25 | 8352.06 |
| maros-r7 | 2421.94 | 2715.52 | 2734.81 | 1958.19 | 2078.30 | 2176.64 |
| 区组三例合计 | 12310.39 | 16395.67 | 17431.63 | 9281.44 | 12722.21 | 12913.92 |

**本轮4对2线程区组合计中位数降低24.60%，P95降低22.41%**；各案例P95均不劣于2线程的1.1倍，满足R6/R7固定门槛，故本机LP direct仍推荐INTEL/4。Auto仅通过全集准确性验收，没有对应20组Auto尾延迟/并发吞吐实验，保守2线程建议保持不变。上述收益不是相对合并前提交的收益，R7也未对该比较给出速度预测。

2/4线程合计IQR/中位数为27.65%/21.55%；greenbea为56.52%/24.86%。峰值工作集中位数113.61/117.34MiB，最大114.27/118.10MiB。样本波动明显，不声称具有跨机器的尾延迟保证。

计时分类均非负且互斥分类之和闭合；正则化重试时间仍是重叠子集。dfl001固定44迭代、45次数值分解、5次符号分析；maros-r7固定21迭代、22次数值分解、4次符号分析；greenbea两变体、2/4线程分别79/65次正则化重试，返回迭代数范围43–368/43–355。数值分解中位数如下：

| 案例 | 2线程数值分解 ms | 4线程数值分解 ms |
|---|---:|---:|
| dfl001 | 3405.33 | 1590.87 |
| greenbea（各变体相加） | 2013.42 | 1530.39 |
| maros-r7 | 1247.84 | 839.90 |

新批次dfl001的固定工作量仍伴随明显耗时变化；greenbea还叠加轨迹变化。保留分解日志，以供后续在受控机器状态下区分因素，本次不额外修改算法。

## SDK 重定位验收

`cmake --install build/windows-msvc-release --config Release --prefix <evidence>/sdk-stage` 成功；随后将该完整目录移动为 `sdk-relocated`。复制 `cmake/consumer-example` 到独立目录，清除当前shell的MKLROOT，关闭CMake用户/系统包注册表搜索，仅给出移动后SDK的 `CMAKE_PREFIX_PATH`，以Ninja/MSVC Release构建并运行。

消费者目标值 **9.000000**，退出码0；`mipsolvers_deploy_runtime`将 `libiomp5md.dll` 部署到EXE旁。`dumpbin /DEPENDENTS`显示Intel运行时、Windows系统及MSVC/UCRT依赖；`lib /LIST`可以读取本轮bundled archive。完整命令在证据目录 `validate_sdk.ps1`，日志为 `sdk-install.log`、`consumer-configure.log`、`consumer-build.log`、`consumer-run.log`、`consumer-dependencies.log`。

这证明本机SDK最小消费者的重定位/链接闭包；没有声称在全新无开发环境VM上验证过全部功能。第三方许可与闭源交付条件仍须遵守手册2.7.1。

## CTest 与最终结论

SDK验收结束后顺序执行：

```powershell
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='4'
$env:MKL_DYNAMIC='FALSE'
$env:MIPSOLVERS_LP_FACTOR_TIMING='0'
ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 1
```

**20/20通过**，14 unit + 4 integration + 2 benchmark，总墙钟240.85秒。HS071的CTest注册项显式覆盖MKL线程为1；该案例的2/4线程验证另由本轮NLP阶段完成。包括release新增的54项IPM测试所在目标、自由变量/事务回归及数值稳定性目标。完整日志 `ctest.log`，注册结果摘要见 [机器可读证据](../manual/windows-release-merge-evidence.json)。

R7的构建、准确性、注册回归、API保留及SDK消费预测全部满足，未删除断言或放宽容差。合并后的实现相对合并提交无进一步代码修改，后续提交仅记录验收文档；raw报告保留对应合并提交和二进制哈希。本次可以发布到Windows release源码分支；仍不把它称为已通过所有Simulation下游场景或完成干净VM二进制发行认证。跨批绝对性能差异的原因仍未唯一定位。
