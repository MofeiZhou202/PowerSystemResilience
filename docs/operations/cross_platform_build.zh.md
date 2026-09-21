> 本文档为 [cross_platform_build.md](cross_platform_build.md) 的中文译文，供 GUI
> 帮助中心使用；两者冲突时以原文与代码为准。
>
> 文档同步：2026-09-19。事实来源为 `CMakeLists.txt`、`CMakePresets.json`、
> `cmake/Dependencies.cmake` 与 `cmake/MIPSolvers.lock.json`。

# 跨平台离线构建

常规构建不会下载依赖。仓库内锁定导入的 `MIPSolvers/` 提供 Eigen 3.4.1、
fmt、nlohmann/json、HiGHS、SCIP、MUMPS、Ipopt、SuiteSparse、Catch2、
PaPILO 3.0.0 及其所需 Boost 头文件的完整本地源码。本仓库自带用于 ETAP
Excel 读写的 OpenXLSX。Eigen 同时包含核心树和 `unsupported/`。

## 锁定的源码布局

默认和正式发布使用：

```text
PowerSystemResilience/
  MIPSolvers/
  cmake/MIPSolvers.lock.json
```

当前导入锁定到 `Matrixeigs/MIPSolvers` 的 `windows` 分支提交
`c6f77f297350b357ff30cc96d9234b2031fd316c`，完整导入 Git tree 也记录在 lock
中。前缀导入没有嵌套 `.git`，因此 CMake 仍会校验关键源码 SHA-256；正式打包
还会校验已提交的 `HEAD:MIPSolvers` 完整树，并拒绝该目录下的待提交改动。

开发者可以显式指定另一份检出：

```bash
cmake -S . -B build/release \
  -DMIPSOLVERS_SOURCE_DIR=/absolute/path/to/MIPSolvers
```

独立 Git 检出必须位于锁定提交；Release/CI 还要求工作树干净。系统不再隐式
回退到 `../MIPSolvers`，避免机器上的兄弟目录悄悄改变构建内容。

离线源码包不得遗漏：

```text
MIPSolvers/third_party/eigen/
MIPSolvers/third_party/fmt/
MIPSolvers/third_party/nlohmann_json/
MIPSolvers/third_party/catch2/
MIPSolvers/third_party/papilo/
MIPSolvers/third_party/boost_papilo/
MIPSolvers/highs/  MIPSolvers/scip/  MIPSolvers/mumps/
MIPSolvers/ipopt/  MIPSolvers/suitesparse/
MIPSolvers/third_party/zlib-1.3.1/
third_party/OpenXLSX-master/
```

Git 源码树对于源码消费是完整的，但 Windows 生成的 `.lib`、`.dll` 不受版本
控制。目录中存在 oneMKL 头文件、manifest 和哈希清单，不代表已经存在可链接的
oneMKL 包。机器相关产物统一生成到已忽略的 `build/windows-dependencies/`；不得
写回锁定的 `MIPSolvers/` 子树。

## 工具链前置条件

每个平台都需要 CMake 3.20 或更高版本以及支持 C++20 的编译器。GridLAB-D、
Julia、OpenDSS、Chromium 与 Playwright 属于运行时测试集成，不是核心 C++ 构建
依赖。

### macOS

使用 Xcode Command Line Tools。启用嵌入式 Ipopt/MUMPS 时还需 `gfortran` 等
Fortran 编译器。

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

### Linux

使用 GCC 或 Clang。Linux preset 默认关闭嵌入式 Ipopt，因此默认路径不需要
Fortran 编译器。

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

### Windows 依赖准备

使用 Visual Studio 2022 的 x64 Native Tools Prompt 或 Developer PowerShell。
安装 oneMKL 或先初始化 `MKLROOT`，然后在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/prepare_windows_dependencies.ps1
```

该脚本会：

1. 从 `MIPSolvers/third_party/zlib-1.3.1` 构建 zlib；
2. 使用 MIPSolvers 的 manifest/hash 逻辑从 `MKLROOT` staging sequential oneMKL；
3. 将全部产物留在 `build/windows-dependencies/`。

完成后可使用源码依赖 preset：

```powershell
cmake --preset windows-source-release
cmake --build --preset windows-source-release
ctest --preset windows-source-release
```

若要准备 `windows-msvc-release` 所需、与 ABI 绑定的完整预编译第三方包，加
`-BuildPrebuiltPackage`：

```powershell
powershell -ExecutionPolicy Bypass -File tools/prepare_windows_dependencies.ps1 `
  -BuildPrebuiltPackage
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

生成目录为：

```text
build/windows-dependencies/zlib/
build/windows-dependencies/oneapi-mkl/
build/windows-dependencies/mipsolvers-third-party/
```

`windows-source-release` 从锁定源码构建其他开源依赖，只消费 staging 的 zlib 和
oneMKL；`windows-msvc-release` 消费完整预编译第三方包。两者在 Release 使用
`/MD`，正式分发关闭 Gurobi 和 IPO，并在启用 Ipopt 时要求 sequential
PardisoMKL。生成包不可跨 MSVC 工具集、架构、运行库模式或 Release/Debug 复用。

### Resilience 与 Trial 版本

两个 edition 开关互斥。Resilience preset 继承源码依赖路径并启用 fail-closed
能力边界：

```powershell
cmake --preset windows-resilience-release
cmake --build --preset windows-resilience-release --target run_gui_server
ctest --preset windows-resilience-release -L edition --output-on-failure
```

Resilience 专用包使用显式的资源与运行时 DLL 允许清单，并提供干净解压验证器：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_resilience_windows.ps1
```

该脚本强制使用仓库内锁定源码依赖，构建并运行 edition 单元/API 门禁，拒绝仅服务于
禁用能力的资源和非允许清单 DLL，写入逐文件清单与 ZIP SHA-256，并在隔离 `PATH`
下调用 `tools/verify_resilience_windows_release.ps1`。脚本定义的是打包契约；成功配置
或脚本已经落盘本身不能证明 Windows 压缩包已通过。已执行证据见
[Resilience Edition 契约](resilience_edition_design.md)与持续更新的开发状态。现有
Trial 路径保持为：

```powershell
cmake --preset windows-trial-release
cmake --build --preset windows-trial-release --target run_gui_server
ctest --preset windows-trial-release -L trial --output-on-failure
```

### Full Edition Windows 二进制包

以下通用命令生成 Full Edition 包，不是 Resilience 打包路径。正式包只能从干净仓库生成，且已提交的 `MIPSolvers/` 子树必须与 lock 一致：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_windows.ps1
```

加 `-SourceDependencies` 可选择 `windows-source-release`。脚本会验证 lock、已提交
子树、依赖模式和发布不变量，构建并测试服务器，用 `dumpbin /DEPENDENTS` 审计
运行时依赖，复制数据、GUI、文档、DLL 与许可，从暂存目录独立启动服务器，并生成
逐文件清单和 ZIP SHA-256。

打包后在新的含空格目录和隔离 `PATH` 中复核：

```powershell
powershell -ExecutionPolicy Bypass -File tools/verify_windows_release.ps1
```

该门禁小于完整 Windows CTest；发布结论必须同时参考持续更新的开发状态。

## 封闭环境验证

不含嵌套 MIPSolvers Git 元数据的源码归档仍可通过来源锁中的源码哈希验证：

```bash
cmake -S . -B build/offline \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build build/offline --parallel
ctest --test-dir build/offline --output-on-failure --parallel 4
```

配置日志必须标识锁定的仓库内导入及本地 vendored 依赖。关键文件缺失或被修改时
必须直接失败，不得触发网络下载。

## OPF 线性求解器选择

Parity OPF IPM 支持：

```text
HACDCPF_OPF_LINEAR_SOLVER=auto|dense|mumps|umfpack|klu|eigen
```

`auto` 对小型 KKT 使用带主元稠密 LU，对大系统使用本地稀疏后端；`umfpack`、
`klu` 来自 MIPSolvers 的 SuiteSparse，`eigen` 强制使用 Eigen SparseLU。该环境
变量是诊断开关，不是稳定应用 API。

## 项目选项

| 选项 | 默认值 | 含义 |
|---|---:|---|
| `HACDCPF_DEPENDENCY_PROFILE` | `portable` | 构建应用所需求解器子集；`full` 还构建 MIPSolvers 开发者目标。 |
| `HACDCPF_TRIAL_EDITION` | `OFF` | 启用 Trial 能力配置；与 Resilience 互斥。 |
| `HACDCPF_RESILIENCE_EDITION` | `OFF` | 启用 Resilience 能力配置；与 Trial 互斥。 |
| `HACDCPF_USE_SUITESPARSE` | `ON` | 使用 vendored UMFPACK/KLU；关闭时使用 Eigen SparseLU。 |
| `HACDCPF_ENABLE_IPOPT` | macOS/Windows 默认 `ON`，Linux preset `OFF` | 启用 Ipopt；Windows 支持路径要求 staging 的 sequential oneMKL/PardisoMKL。 |
| `HACDCPF_ENABLE_ETAP` | `ON` | 使用 vendored OpenXLSX。 |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | 需显式提供本地 DSS C-API。 |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | 启用宿主 CPU 指令；便携二进制应保持关闭。 |
| `HACDCPF_USE_GUROBI` | 裸配置默认 `ON`；Windows 发布 preset `OFF` | 自定义构建可使用本机授权 Gurobi；正式包不分发它。 |
| `HACDCPF_USE_PAPILO` | `ON` | 使用 MIPSolvers 的 header-only PaPILO；关闭时用原生 presolve。 |

发布前仍需在各目标操作系统上执行原生构建，并用 `otool -L`、`ldd` 或
`dumpbin /DEPENDENTS` 审计最终运行时依赖。
