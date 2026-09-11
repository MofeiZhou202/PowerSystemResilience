# 第 2 章 安装与部署

> 本章整合自: docs/archive/build_and_deploy.md, docs/archive/windows_ci_validation.md

本章面向现场工程师与集成商，说明 MIPSolvers 的依赖模型、各平台构建步骤、离线自封包（sealed offline package）流程，以及安装后的消费与验证方法。

**核心原则：仓库一旦拷贝到构建机，默认构建全程不需要网络。** configure 阶段不会下载任何依赖；缺失 vendored 源码树是致命的 checkout 错误，而不是降级行为。

> 注意：不要在源码根目录直接 configure，请始终使用独立的构建目录（预设默认输出到 `build/<preset名>`）。

## 2.1 支持的平台与工具链

| 平台 | 经过验证的工具链 | 数值库选择顺序 |
|---|---|---|
| macOS | Xcode command-line tools + Fortran 编译器（如 `brew install gcc`） | Accelerate → vendored reference BLAS/LAPACK |
| Linux | GCC/Clang + `gfortran`，CMake ≥ 3.20 | 系统 BLAS/LAPACK → vendored reference BLAS/LAPACK |
| Windows | **Visual Studio 2022 x64**（唯一测试过的原生配置），从 `x64 Native Tools Command Prompt` 或经 `VsDevCmd.bat` 初始化的 PowerShell 启动 | 可发现的系统 BLAS/LAPACK；默认 Ipopt 配置使用 oneMKL |

所有平台共同要求：**CMake 3.20 或更新版本，以及 C++20 编译器**。

Windows 上严禁混用 MSVC 库与 MinGW C++ 构建。Windows 有两个受支持的构建配置：

1. **MSVC + oneAPI MKL（默认，风险最低）**：内嵌 Ipopt 使用 PardisoMKL，不编译 MUMPS。
2. **纯源码数值配置**：Ipopt 后端切到 `mumps`，强制使用 vendored reference BLAS/LAPACK，并提供 Intel oneAPI `ifx`/`ifort` 或一套自洽的 MinGW `gfortran` 工具链。

## 2.2 依赖模型

以下源码树随仓库分发并在本地编译，configure 阶段不会联网下载：

- HiGHS、SCIP、Ipopt、MUMPS 5.7.3、SuiteSparse
- 完整 Eigen 3.4.1（含 `unsupported/`）、fmt 12.1.0、nlohmann/json 3.11.3、Catch2 3.7.1
- PaPILO 3.0.0 及其所需的 Boost 头文件子集
- reference BLAS 与 LAPACK 3.12.1，作为最终数值兜底

要点：

- PaPILO 集成是 **header-only** 的：不需要 GMP、TBB、LUSOL、Boost 二进制库、OpenBLAS 或 Fortran。`MIPSOLVERS_USE_PAPILO=OFF` 时原生 presolver 作为显式回退保留。
- configure 会检查 `third_party/eigen/unsupported/Eigen/MatrixFunctions`，防止被裁剪过的 Eigen 头文件子集"静默通过"离线依赖检查。
- reference BLAS/LAPACK 只是可靠的离线兜底。**生产环境若数值核性能重要，请使用 OpenBLAS、MKL 或其他优化实现**。
- Gurobi 永不随包分发。构建期检测到其头文件与库、且 `GRBloadenv()` 能初始化有效运行许可时，它是首选 LP/QP/MILP 后端；缺失安装、许可缺失/过期或求解失败时，在 `SolveOptions::allow_fallback` 为 true（默认）的情况下自动回退到内嵌 HiGHS 与原生求解器。需要不含 Gurobi 的包时设 `MIPSOLVERS_USE_GUROBI=OFF`。

### oneMKL 的本地化 staging（Windows 默认配置必需）

正常 CMake configure **永远不会下载 oneMKL**。需要在一台联网的 staging 机器上初始化一次 oneAPI 环境，并创建仓库本地的静态库 bundle：

```powershell
cmd /k '"C:\Program Files (x86)\Intel\oneAPI\setvars.bat" intel64'
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force
```

生成的 `third_party/oneapi-mkl` 目录包含完整头文件树、`mkl_intel_lp64.lib`、`mkl_sequential.lib`、`mkl_core.lib`、许可证材料、版本清单和 `SHA256SUMS`。该目录被 Git 忽略，必须作为受控二进制依赖或内部 artifact 传输。staging 出的 bundle 刻意只含 sequential 线程层；如需本地共享内存 PARDISO 构建，须显式选择 Intel 线程层（见 2.6 节）。

## 2.3 CMake 预设一览

预设定义见仓库根目录 `CMakePresets.json`。所有预设继承隐藏基座 `base`（`CMAKE_BUILD_TYPE=Release`、构建测试、关闭 SCUC/Python、开启 SuiteSparse/SuperLU/MKL/PaPILO、关闭 Gurobi），构建目录均为 `build/<preset名>`。

| 预设 | 生成器 | 关键差异 | 适用场景 |
|---|---|---|---|
| `macos-release` | Unix Makefiles | 启用本地 Ipopt | macOS 本机构建 |
| `linux-release` | Unix Makefiles | 启用本地 Ipopt | Linux 本机构建 |
| `windows-msvc-release` | Ninja Multi-Config | `pardisomkl` 后端、IPO 关闭、动态 MSVC 运行时 | Windows 默认开发/发布构建 |
| `windows-offline-release` | 同上（继承） | `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON`，`MIPSOLVERS_MKL_ROOT=third_party/oneapi-mkl` | 无网密封机的自封包发布构建 |
| `windows-vcpkg-release` | 同上（继承） | 挂接 vcpkg toolchain 与 `x64-windows` triplet | 依赖由 vcpkg 管理的开发机（密封机上不要使用） |
| `portable-no-suitesparse` | Unix Makefiles | 关闭 SuiteSparse、SuperLU、MKL | 需要最大可移植性、可接受精简线性求解器的场景 |
| `full-dev` | Unix Makefiles | `RelWithDebInfo`，开启 SCUC 与 case builder | 全功能开发者构建 |

`windows-msvc-release` 预设不含任何机器特定的编译器或 SDK 路径——它完全使用当前 shell 中的编译器、Ninja、Windows SDK 和 oneAPI 环境。

## 2.4 构建、测试与安装

### macOS / Linux

准备依赖：

```bash
# macOS
xcode-select --install
brew install cmake ninja gcc libomp        # libomp 可选，缺省时数值核串行运行

# Ubuntu / Debian
sudo apt update
sudo apt install -y build-essential cmake ninja-build gfortran
sudo apt install -y libopenblas-dev liblapack-dev   # 可选，生产环境推荐优化 BLAS
```

从源码构建并运行单元测试：

```bash
cmake -S . -B build/release \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF
cmake --build build/release --parallel
ctest --test-dir build/release -L unit --output-on-failure --parallel 4
```

如需强制使用内嵌 reference BLAS/LAPACK（而非 Accelerate 或系统库）：

```bash
cmake -S . -B build/reference-blas \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF \
  -DMIPSOLVERS_FORCE_VENDORED_BLAS=ON
cmake --build build/reference-blas --parallel
```

说明：`MIPSOLVERS_USE_VENDORED_BLAS=ON`（默认）是"允许兜底"，`MIPSOLVERS_FORCE_VENDORED_BLAS=ON` 是"跳过系统探测"，后者用于兜底路径验证。

安装：

```bash
cmake --install build/release --prefix /path/to/prefix
```

### Windows 默认配置（MSVC + oneMKL）

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release -L unit
```

等价的显式配置（便于 CI 脚本化）：

```powershell
cmake -S . -B build/windows-msvc -G "Ninja Multi-Config" `
  -DMIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl `
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF
cmake --build build/windows-msvc --config Release --parallel 8
ctest --test-dir build/windows-msvc -C Release -L unit --output-on-failure
```

### Windows 纯源码数值配置

在选定 Fortran 编译器可用的 shell 中执行；oneAPI 自动探测不足时显式传入 `ifx.exe`：

```powershell
cmake -S . -B build/windows-source -G "Ninja Multi-Config" `
  -DMIPSOLVERS_FORTRAN_COMPILER="C:/Program Files (x86)/Intel/oneAPI/compiler/latest/bin/ifx.exe" `
  -DMIPSOLVERS_IPOPT_LINEAR_SOLVER=mumps `
  -DMIPSOLVERS_FORCE_VENDORED_BLAS=ON `
  -DMIPSOLVERS_USE_MKL=OFF
cmake --build build/windows-source --config Release --parallel 8
```

此路径由 CMake 逻辑支持，但必须在部署包实际使用的 Visual Studio、oneAPI、Windows SDK 与架构组合上重新验证。

### 实测注意事项（2026-08-18，Windows）

`miplib2017_benchmark` 目标需要 ZLIB。Windows 上没有系统级 ZLIB，需自行编译 zlib（静态库），并在 configure 时显式传入路径，否则 configure 报 `Could NOT find ZLIB`：

```powershell
cmake --preset windows-msvc-release `
  -DZLIB_LIBRARY=<path>/zlibstatic.lib `
  -DZLIB_INCLUDE_DIR=<path>/include
```

仓库内虽带有 `third_party/zlib-1.3.1` 源码，但该 benchmark 目标的 ZLIB 查找不自动使用它，需要上述显式参数。

## 2.5 预编译第三方包

第三方库可以只编译一次、供后续开发构建复用。生成的包**绑定于其编译器、SDK、架构、运行时与构建配置，不是通用二进制缓存**。

### macOS / Linux

```bash
third_party/build_third_party.sh --jobs 8 --build-type Release
```

默认输出 `third_party/build/` 与 `third_party/install/`（均被 Git 忽略）。`--fresh` 仅丢弃本地第三方构建目录后重建。

### Windows

```powershell
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force   # 仅创建/更新 oneMKL bundle 时需要
.\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
```

`build_third_party.ps1` 会按 `SHA256SUMS` 校验每个 staged 文件，以 IPO 关闭方式构建，并安装可重定位的 `MIPSolvers::MKL` target 与 `ipopt_local`；随后运行一次仅 configure 的消费者契约检查（验证父工程 target 可见性与包相对 MKL 路径，不重复编译链接）。此后密封主构建只需要 Visual Studio、`third_party/install` 包和源码树，**不再需要机器级 oneAPI 安装**。

### 复用策略

`MIPSOLVERS_USE_PREBUILT_THIRD_PARTY` 接受三个值：

- `AUTO`（默认）：有兼容的本地清单则复用，否则警告并在树内构建 vendored 源码
- `ON`：强制要求兼容包，缺失或不兼容即 configure 失败
- `OFF`：永远在树内构建依赖

清单记录依赖可用性、公共编译定义与工具链身份；多配置生成器被限制在包内已安装的配置上，防止 Release 依赖包被链进 Debug MSVC 构建。

## 2.6 离线自封包构建流程

以下流程只能在联网 staging 机器上的 **x64 Visual Studio 2022 开发者 shell** 中执行（oneMKL 不由本项目从源码编译，`stage_onemkl.ps1` 负责拷贝）：

```powershell
cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 && set'
cmd /c '"C:\Program Files (x86)\Intel\oneAPI\setvars.bat" intel64 && set'
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force
.\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
cmake --preset windows-offline-release
cmake --build --preset windows-offline-release
ctest --preset windows-offline-release -L unit
cmake --install build/windows-offline-release --config Release --prefix artifacts/windows-offline
```

可复用依赖二进制位于 `third_party/install`；安装好的 MIPSolvers 包位于上例的 `artifacts/windows-offline`。传输前记录哈希：

```powershell
Get-ChildItem third_party\install, artifacts\windows-offline -Recurse -File |
  Get-FileHash -Algorithm SHA256 |
  Sort-Object Path |
  Format-Table Hash, Path -AutoSize
```

受控离线传输时，从 staging 机器归档以下内容：

```text
third_party/oneapi-mkl/
third_party/install/
```

将 `third_party/oneapi-mkl/SHA256SUMS` 与已安装的 `mipsolvers-third-party` 清单随发布记录一并保存。**在密封机上使用 `cmake --preset windows-offline-release`，不要使用 vcpkg 预设，也不要依赖 `%MKLROOT%`**。该预设刻意要求已传输的预编译包——缺失或不兼容的依赖是 configure 错误，而不是隐式的树内重建或网络探测。

验收包之前，用 `dumpbin /DEPENDENTS` 检查每个随包可执行文件与 DLL。默认 oneMKL 形态是 **LP64、sequential、static**；显式 INTEL 配置使用静态线程层及动态 `libiomp5md.dll`，必须把该 DLL 纳入离线产物。任何未随包交付的非系统运行时依赖都意味着依赖闭包未完成。

### Intel 线程层（显式本地配置）

```powershell
cmake --preset windows-msvc-release `
  -DMIPSOLVERS_MKL_ROOT=third_party/oneapi-mkl `
  -DMIPSOLVERS_MKL_THREADING=INTEL `
  -DMIPSOLVERS_USE_OPENMP=ON `
  -DMIPSOLVERS_ENABLE_IPO=OFF `
  -DMIPSOLVERS_ENABLE_NATIVE_ARCH=OFF `
  -DMIPSOLVERS_PGO_MODE=OFF
```

该配置链接 `mkl_intel_thread` 与 `libiomp5md`，运行时 DLL 记录在 `MIPSOLVERS_MKL_RUNTIME_DLLS` 中并部署到可执行文件旁。staging 时使用 `stage_onemkl.ps1 -Threading both` 可同时提供串行与线程层；显式 `MIPSOLVERS_MKL_ROOT` 控制本地库及运行时发现，缺失时配置失败，不回退到系统 oneAPI。

上述开关与 2026-09-11 的重复实验一致：保留 C++ OpenMP 编译支持，但运行时设置 `OMP_NUM_THREADS=1`，由 `MKL_NUM_THREADS` 单独控制 oneMKL。关闭 C++ OpenMP 是另一个构建配置，不能直接套用本次数字。不要默认添加 CBWR 或 CPU affinity：本次均未设置，跨线程的数值轨迹也不保证逐位一致；需要可复现归约时应单独验证其准确性与性能。

本机 i9-12900H 的 LP direct 推荐 INTEL/4：20组长尾合计中位数降18.50%，P95降17.72%，两档全集及NLP均通过准确性验收。Auto的全集总时间在4线程下增加10.44%，因此Auto/混合负载先保留2线程，不统一提高线程数。详细范围和原始证据见 [稳定性报告](../archive/windows_lp_stability_2026-09-11.md)。启动进程前设置：

```powershell
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='4'  # LP direct；Auto先用2
$env:MKL_DYNAMIC='FALSE'
$env:MIPSOLVERS_LP_FACTOR_TIMING='0'
```

以上是显式运行建议，不改变CMake默认线程层。LTCG/PGO/AVX2在本轮未单独验证，继续关闭；多任务并发的吞吐需另行测试。

## 2.7 SDK 产物布局与消费者链接

安装前缀（如 `artifacts/windows-offline`）导出的内容包括：

- `mipsolvers::mipsolvers` CMake target（经 `find_package(mipsolvers CONFIG REQUIRED)` 使用）
- vendored Eigen 与 nlohmann/json 头文件；构建过 vendored fmt/SuiteSparse target 时一并导出
- oneMKL 许可证材料，位于 `share/mipsolvers-third-party/licenses/oneapi-mkl`（部署声明中必须保留）
- 启用 Intel 线程层时，`libiomp5md.dll` 安装在包内可执行文件旁，并同时记录在包配置与清单中；声明的运行时文件缺失会导致包加载和预编译消费者冒烟测试失败

消费者的两种集成方式：

```cmake
# 方式一：使用安装好的包
find_package(mipsolvers CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

```cmake
# 方式二：源码级集成
set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MIPSOLVERS_BUILD_SCUC OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/MIPSolvers EXCLUDE_FROM_ALL)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

安装包不会无条件搜索 Homebrew 路径。

### 2.7.1 只分发编译版本

当前 `mipsolvers` 是 C++20 静态库。近期建议发布可重定位的 **Windows x64 / MSVC v143 / Release / MD 二进制 SDK**，分发 `cmake --install` 生成的整个目录，而不是只拷贝一个 `.lib` 或构建目录中的 bundled archive。消费者编译自己的应用，不需要编译本项目；C++ 接口依赖的公开头文件与模板头仍须随包提供。

在已通过验证的 x64 VS 开发者环境中：

```powershell
cmake --build build/windows-msvc-release --config Release --parallel 6
cmake --install build/windows-msvc-release --config Release --prefix artifacts/mipsolvers-sdk-win-x64
```

交付目录应包含 `include/`、`lib/`、`lib/cmake/mipsolvers/`、`bin/` 中必需的运行时、第三方许可证/通知、最小消费者示例、版本与构建清单。核心实现的 `src/`、仓库 `.git/`、开发构建缓存、测试数据和内部调试符号不作为常规 SDK 内容；第三方源码交付义务单独按许可证落实，不能因使用二进制 SDK 就忽略。

消费者最小配置：

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_app LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
find_package(mipsolvers CONFIG REQUIRED)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
mipsolvers_deploy_runtime(my_app)
```

配置时用 `-DCMAKE_PREFIX_PATH=<SDK解压目录>`，构建 Release。当前 C++ ABI 的支持范围应固定到经过验收的工具链/架构/运行时组合；Debug、MinGW、ARM64 和跨版本编译器兼容性不能由此自动推断。随产品提供匹配的 MSVC 运行时安装要求，Intel 线程层使用包内 `libiomp5md.dll`。

正式发布前将 SDK 复制到另一个路径，在没有源码树、oneAPI 开发环境和开发机 PATH 的干净 Windows VM 上，仅使用 SDK 构建/运行消费者。除最小 LP 外，还需按交付功能覆盖 NLP、MILP、锥求解；核对最终 EXE/DLL 的 `dumpbin /DEPENDENTS`，并校验解压后的逐文件 SHA256。当前20项CTest和最小消费者验证不能替代这项完整发布验收。

每份发行包固定版本、Git提交、编译器/Windows SDK版本、CMake选项、第三方版本、最低系统要求及数值验收结果。建议文件名包括版本和ABI，例如 `mipsolvers-<version>-windows-x64-msvc143-release-md.zip`，另附SHA256清单和变更记录。公开发行物放在Release附件或制品库；Git保留源码、配置与报告。PDB另存内部符号包以便根据用户崩溃信息定位问题。运行线程数由应用按负载设置，本轮LP direct推荐4、Auto保守选2，不在SDK中固定所有调用者的进程环境。

**闭源交付必须先处理现有依赖许可。** 当前vendored `UMFPACK/Include/umfpack.h` 标注GPL-2.0+；`CHOLMOD/Include/cholmod.h` 列出Supernodal等GPL模块以及其他LGPL模块。不能假设把它们静态合并或改成DLL后就可无条件闭源分发。应核对实际链接模块及授权，选择满足对应源码/重链接等义务、取得合适商业授权，或形成替换/裁剪依赖的发行变体；后一种方案需重新执行数值与性能验收，不能沿用本轮SuiteSparse构建的数字。另须落实Ipopt、Eigen、oneMKL及其他实际依赖的通知、许可证和再分发条款，并明确本项目自身的使用授权。

长期若要支持Python/C#/不同C++工具链，建议另设稳定的C ABI DLL：不透明句柄、固定宽度类型/数组、显式创建与销毁、错误码和版本查询；异常、STL/Eigen对象及跨CRT的内存释放不穿过DLL边界。再在该接口之上提供薄C++/Python封装。当前项目没有完成这层ABI交付，不能仅设置 `BUILD_SHARED_LIBS=ON` 就获得它；DLL本身也不等于算法不可被逆向。

## 2.8 运行时部署要点

- **统一 C/C++ 运行时模型。** Windows 预设使用动态 MSVC 运行时（Release `/MD`、Debug `/MDd`），随应用分发对应的 MSVC redistributable。
- **最终可执行文件才是判据。** 静态库包本身不能证明运行时依赖已闭合：
  - macOS：`otool -L path/to/application`——vendored MUMPS 需要 Fortran 运行时，构建优先静态 GCC Fortran 归档；不可用时最终可执行文件会依赖匹配的 `libgfortran`/`libquadmath` dylib。启用 OpenMP 时需随应用分发兼容的 `libomp.dylib` 并修正 install name/RPATH（macOS 一般不支持静态链接 OpenMP）。
  - Linux：`ldd path/to/application`。可用 `MIPSOLVERS_STATIC_LIBGFORTRAN=ON` 换取更自足的部署，但需确认目标发行版允许静态链接 `libgcc`/`libgfortran`。
  - Windows：`dumpbin /DEPENDENTS application.exe`。
- 更换 MSVC 工具集、目标架构、oneAPI 编译器/运行时或 MSVC 运行时模型时，**必须重新生成预编译第三方 `.lib`**。

## 2.9 发布候选验证清单

1. 在全新构建目录以 `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF` configure，确认无任何下载活动。
2. 以默认（内嵌 PaPILO 开启）构建并运行 `ctest -L unit`。
3. 再以 `MIPSOLVERS_USE_PAPILO=OFF` 重复，验证原生 presolve 回退。
4. 以 `MIPSOLVERS_FORCE_VENDORED_BLAS=ON` 构建并测试一次。
5. 在 Windows staging 上运行 `stage_onemkl.ps1` 与 `build_third_party.ps1`，保留 `SHA256SUMS`、许可证材料与已安装的第三方清单。
6. 将源码树、`third_party/oneapi-mkl`、`third_party/install` 传输到无网机器，用 `windows-offline-release` configure 并运行单元测试。
7. 用 `otool -L` / `ldd` / `dumpbin /DEPENDENTS` 检查最终可执行文件（仅检查静态库是不够的）。
8. 在部署工具链上重复 Windows 构建——从 macOS/Linux 做跨平台 CMake configure 不能验证 MSVC/ifx 的 ABI 行为。

## 2.10 Windows CI 回归验证

Windows 侧回归面集中在共享求解器分发路径，涉及五个测试：`test_engine_api`、`test_milp_solver`、`test_ipm_solver`、`test_scuc_module`、`test_market_simulation`。已修复的 Windows 特有失败包括：子进程重定向硬编码 `/dev/null`（应为 `NUL`）、调试 JSON 输出硬编码 `/tmp`（应使用 `std::filesystem::temp_directory_path()`）、空稀疏系统进入 Eigen SparseLU 除零（`0x0` 系统应在调用 Eigen/SuiteSparse 后端前短路）。

在 `x64 Native Tools Command Prompt for VS 2022`（或 PATH 上带 MSVC 工具链的 PowerShell）中，从仓库根执行：

```powershell
# 1. configure（test_scuc_module / test_market_simulation 仅在 SCUC 开启时注册）
cmake -S . -B build-win -A x64 `
  -DMIPSOLVERS_BUILD_TESTS=ON `
  -DMIPSOLVERS_BUILD_PYTHON=OFF `
  -DMIPSOLVERS_BUILD_SCUC=ON `
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF `
  -DMIPSOLVERS_ENABLE_WERROR=OFF

# 2. 只构建受影响的五个目标
cmake --build build-win --config Release `
  --target test_engine_api test_milp_solver test_ipm_solver test_scuc_module test_market_simulation `
  --parallel 4

# 3. 运行 Windows 回归切片
ctest --test-dir build-win --build-config Release `
  -R "test_(engine_api|milp_solver|ipm_solver|scuc_module|market_simulation)" `
  --output-on-failure --parallel 2

# 4. 可选：全量测试
ctest --test-dir build-win --build-config Release --output-on-failure --parallel 2
```

通过判据：configure 与构建不要求 Gurobi；五个测试全部通过；失败输出中不含 `/dev/null` 或 `/tmp` 路径假设；`test_market_simulation` 将调试 JSON 写入合法 Windows 临时目录；`test_engine_api` 确认 `EigenSparseLUSolver` 接受空 `0x0` 系统而不崩溃。

对应的 GitHub Actions 回归 job 应：运行在 `windows-latest`；以"测试开启、Python 关闭、SCUC 开启"配置；只构建这五个测试目标；用定向 `ctest -R` 运行。这样 Windows job 聚焦精确的回归面，同时覆盖当初导致失败的共享求解器分发路径。

补充说明：本仓库无需外部 HiGHS 或 SCIP 可执行文件即可构建，内嵌源码树足以覆盖 CI；若 VS Code CMake Tools 无法 configure，以上述终端 `cmake`/`ctest` 命令为准。

## 2.11 常用 CMake 选项速查

| 选项 | 默认值 | 用途 |
|---|---:|---|
| `MIPSOLVERS_BUILD_TESTS` | `ON` | 构建单元与集成测试 |
| `MIPSOLVERS_THIRD_PARTY_ONLY` | `OFF` | 只构建/安装依赖 |
| `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY` | `AUTO` | `AUTO`/`ON`/`OFF` 复用策略 |
| `MIPSOLVERS_USE_VENDORED_BLAS` | `ON` | 允许 reference BLAS/LAPACK 兜底 |
| `MIPSOLVERS_FORCE_VENDORED_BLAS` | `OFF` | 强制 reference BLAS/LAPACK |
| `MIPSOLVERS_FORCE_BUILD_MUMPS` | `ON` | 构建 vendored MUMPS 而非 Homebrew MUMPS |
| `MIPSOLVERS_STATIC_LIBGFORTRAN` | `OFF` | Linux 上静态 GNU Fortran 运行时 |
| `MIPSOLVERS_IPOPT_LINEAR_SOLVER` | 平台相关 | `mumps`，Windows 上可为 `pardisomkl` |
| `MIPSOLVERS_MKL_ROOT` | 空 | 权威本地静态 oneMKL bundle；设置后无系统回退 |
| `MIPSOLVERS_MKL_THREADING` | Windows: `SEQUENTIAL` | 仅 Windows：`SEQUENTIAL` 或显式本地 bundle 的 `INTEL` 线程 |
| `MIPSOLVERS_USE_PAPILO` | `ON` | 使用内嵌 PaPILO；原生 presolve 仍保留 |
| `MIPSOLVERS_PAPILO_SOURCE_DIR` | 内嵌 | 覆盖为其他本地 PaPILO 源码树 |
| `MIPSOLVERS_PAPILO_BOOST_DIR` | 内嵌 | 覆盖 PaPILO 的本地 Boost include 根 |
| `MIPSOLVERS_PAPILO_ROOT` | 空 | 无源码树时指定本地已安装 PaPILO 前缀 |
| `MIPSOLVERS_USE_GUROBI` | `ON` | 探测已安装 Gurobi；缺失不致命 |
| `MIPSOLVERS_USE_SYSTEM_FMT` | `OFF` | 选择使用系统 fmt |
| `MIPSOLVERS_USE_OPENMP` | `ON` | 检测到 OpenMP 时启用 |

> 注：`CMakePresets.json` 的 `base` 预设将 `MIPSOLVERS_USE_GUROBI` 显式置为 `OFF`，即通过预设构建时默认不探测 Gurobi；如需启用请在 configure 时显式传 `-DMIPSOLVERS_USE_GUROBI=ON`。

下一步阅读：[快速上手](03-quickstart.md) 构建第一个模型，或参阅 [测试与基准](09-testing-benchmarks.md) 了解完整的测试标签与基准流程。
