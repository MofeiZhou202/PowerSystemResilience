> 本文档为 [cross_platform_build.md](cross_platform_build.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

> 文档同步（2026-07-26）
> 范围：macOS、Linux 与 Windows 上的离线源码构建。
> 状态：有实现支撑的参考文档。
> 事实来源：若本文档与 `CMakeLists.txt`、`CMakePresets.json` 及
> `cmake/Dependencies.cmake` 发生分歧，以这些文件为准。

# 跨平台离线构建

常规构建不会下载依赖。`MIPSolvers` 提供以下组件的完整本地源码副本：
Eigen 3.4.1、fmt、nlohmann/json、HiGHS、SCIP、MUMPS、Ipopt、SuiteSparse、
Catch2、PaPILO 3.0.0，以及 PaPILO 所需的 Boost 头文件。本仓库自带
OpenXLSX，用于 ETAP Excel 读写。Eigen 同时包含核心源码树与 `unsupported/`，
其中包括 `unsupported/Eigen/MatrixFunctions`。

## 源码包目录布局

默认布局为：

```text
package/
  HybridACDCDistributionSystemsSimulation/
  MIPSolvers/
```

也可以将 `MIPSolvers` 目录放在主仓库内部，或显式指定其位置：

```bash
cmake -S . -B build/release \
  -DMIPSOLVERS_SOURCE_DIR=/absolute/path/to/MIPSolvers
```

Release 与 CI 构建会拒绝处于脏状态（dirty）的 MIPSolvers Git 检出。不含
`.git` 元数据的分发源码归档包可以接受，并会被如实报告为此类情形；归档
包的制作方负责保证所固定内容的完整性。

离线包中不得遗漏以下路径：

```text
MIPSolvers/third_party/eigen/
MIPSolvers/third_party/fmt/
MIPSolvers/third_party/nlohmann_json/
MIPSolvers/third_party/catch2/
MIPSolvers/third_party/papilo/
MIPSolvers/third_party/boost_papilo/
MIPSolvers/highs/  MIPSolvers/scip/  MIPSolvers/mumps/
MIPSolvers/ipopt/  MIPSolvers/suitesparse/
MIPSolvers/third_party/install/
HybridACDCDistributionSystemsSimulation/third_party/OpenXLSX-master/
```

在 Windows 上，`MIPSolvers/third_party/install` 由预制的 oneMKL 静态捆绑包
生成，且与 ABI（应用二进制接口）绑定。它必须与其清单文件及 oneMKL 许可
声明一并保留；不得跨 MSVC 工具集、处理器架构、运行时库模式或
Release/Debug 配置复用。

## 工具链前置条件

前置条件仅包括构建工具与平台运行时。每个平台都需要 CMake 3.20 或更高
版本以及支持 C++20 的编译器。

### macOS

使用 Xcode Command Line Tools。默认的嵌入式 Ipopt/MUMPS 配置档还需要一个
Fortran 编译器，例如 `gfortran`。Homebrew 安装的库不是构建依赖；但仍可用
Homebrew 安装 CMake 或编译器。

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

### Linux

使用 GCC 或 Clang 配合标准 C/C++ 构建工具。Linux 主配置档默认关闭嵌入式
Ipopt，因此默认配置档不需要 Fortran 编译器。

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

### Windows

使用 Visual Studio 2022 的 x64 Native Tools 命令提示符或 Developer
PowerShell。先在一台联网的 Windows 预制作（staging）机器上一次性准备
MIPSolvers：

```powershell
cd ..\MIPSolvers
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force
.\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
cd ..\HybridACDCDistributionSystemsSimulation
```

然后将两个仓库整体转移到隔离（sealed）环境中，包括被 git 忽略的
`MIPSolvers/third_party/install` 产物。预制作脚本会记录 SHA-256 哈希值与
许可材料；该预构建包导出 Ipopt 以及一个可重定位的静态 `MIPSolvers::MKL`
目标。其安装后消费方检查仅执行 configure（配置）阶段，因此无需再次编译
或链接即可验证目标的作用域与路径。`build_third_party.ps1` 与 Windows 主
构建配置档将并行度上限设为 8，并保持 IPO（跨过程优化）关闭。

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

该配置档使用 MSVC 动态运行时（Release 下为 `/MD`），要求使用与之兼容的
MIPSolvers 预构建包，并启用嵌入式 Ipopt 以及顺序执行的静态 PardisoMKL
后端。`windows-vcpkg-release` 仍是面向刻意提供额外软件包的站点的兼容
配置档；它继承相同的 Ipopt/预构建包契约。

## 封闭环境验证（Hermetic Verification）

请从一个全新的构建目录和一份不含 `.git` 元数据的 MIPSolvers 源码副本出发
验证发布版本。下面的开关可以杜绝任何意外的 FetchContent 网络操作；不加
该开关时，构建同样应当成功。

```bash
cmake -S . -B build/offline \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_SOURCE_DIR=/path/to/MIPSolvers-source-archive \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build build/offline --parallel
ctest --test-dir build/offline --output-on-failure --parallel 4
```

configure 日志必须将 Eigen、fmt、nlohmann/json、Catch2、OpenXLSX、HiGHS、
SCIP、PaPILO、启用时的 MUMPS/Ipopt 以及 SuiteSparse 标识为 vendored（随包
内置）源码。缺少任何必需的源码树时，必须在 configure 阶段直接失败，而
不得触发下载。

Gurobi 检测默认启用，但始终保持可选，且从不随包捆绑。当 Gurobi 安装在
非标准位置时，请在配置前设置 `GUROBI_HOME`。运行时若许可证不可用/过期、
环境初始化失败或求解失败，将回退到随包内置的 HiGHS 与原生求解器。实际
产生结果的后端由所报告的求解器名称与直流最优潮流（DC OPF）的
`solver_chain` 字段标识。

GridLAB-D、Julia、OpenDSS、Chromium 与 Playwright 等外部对比工具属于运行
时测试集成，而非 C++ 构建依赖。当对应可执行文件缺失时，相关测试会自动
跳过。如需启用 OpenDSS 桥接，请提供本地的 DSS C-API 捆绑包并设置
`HACDCPF_ENABLE_OPENDSS=ON`。

## 最优潮流线性求解器选择

parity 最优潮流内点法（OPF IPM）支持：

```text
HACDCPF_OPF_LINEAR_SOLVER=auto|dense|mumps|umfpack|klu|eigen
```

`auto` 对小型 KKT 系统使用带主元的稠密 LU 分解，对较大系统使用本地稀疏
后端。`umfpack` 与 `klu` 使用 MIPSolvers 随包内置的 SuiteSparse 目标；
`eigen` 强制使用随包内置的 Eigen SparseLU 回退方案。该环境开关仅供诊断
使用，不是稳定的应用层 API。

## 项目选项

| 选项 | 默认值 | 含义 |
|---|---:|---|
| `HACDCPF_DEPENDENCY_PROFILE` | `portable` | 构建应用所需的求解器子集；`full` 还会构建 MIPSolvers 的开发者目标。 |
| `HACDCPF_USE_SUITESPARSE` | `ON` | 使用随包内置的 UMFPACK/KLU；`OFF` 选择 Eigen SparseLU 回退方案。 |
| `HACDCPF_ENABLE_IPOPT` | macOS/Windows 配置档 `ON`，Linux 配置档 `OFF` | 启用嵌入式 Ipopt；Windows 消费本地的 oneMKL 预构建包。 |
| `HACDCPF_ENABLE_ETAP` | `ON` | 基于随包内置的 OpenXLSX 构建。 |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | 需要另行提供的本地 DSS C-API。 |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | 启用针对宿主机的 CPU 指令；要产出可移植二进制请保持 `OFF`。 |
| `HACDCPF_USE_GUROBI` | `ON` | 检测并优先使用已安装/已授权的 Gurobi；缺失时不视为错误。 |
| `HACDCPF_USE_PAPILO` | `ON` | 使用 MIPSolvers 捆绑的 header-only PaPILO；`minimal` 配置档或 `OFF` 时使用原生预处理（presolve）。 |

发布前仍必须在全部三个目标操作系统上执行原生构建：在 macOS 上配置
Windows 配置档并不能验证 MSVC ABI 或运行时打包。请使用 `otool -L`、
`ldd` 或 `dumpbin /DEPENDENTS` 检查最终可执行文件，以确定部署时必须一并
携带的编译器与平台运行时库。
