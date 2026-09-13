# 第 10 章 故障排查

> 本章整合自: docs/archive/build_and_deploy.md, docs/archive/windows_ci_validation.md, docs/archive/testing.md, docs/archive/user_manual.md

本章按 FAQ 组织，每条给出「现象 → 原因 → 解决」三步。排查前请先确认

你使用的是与部署目标一致的工具链和构建目录；很多"奇怪"的报错只是因为在错误的 shell 或过期的 build 目录里执行命令。相关章节：[安装与部署](02-installation-deploy.md)、[测试与基准](09-testing-benchmarks.md)。

## 10.1 构建与配置

### configure 报 `Could NOT find ZLIB`（Windows）

- **现象**：`cmake --preset windows-msvc-release` 或显式 `cmake -S . -B ...` 在 configure 阶段失败，报 `Could NOT find ZLIB (missing: ZLIB_LIBRARY ZLIB_INCLUDE_DIR)`。
- **原因**：Windows 平台没有系统级 zlib；某些构建路径（如依赖 zlib 的第三方组件）会调用 `find_package(ZLIB)`，但 configure 阶段不会替你下载依赖。
- **解决**（2026-08-18 实测）：自编译 zlib 静态库，并在 configure 时显式传入路径：

  ```powershell
  cmake -S . -B build/windows-msvc -G "Ninja Multi-Config" `
    -DZLIB_LIBRARY="C:/path/to/zlib-build/zlibstatic.lib" `
    -DZLIB_INCLUDE_DIR="C:/path/to/zlib"
  ```

  仓库内自带 `third_party/zlib-1.3.1` 源码和 `third_party/zlib-build` 构建目录，可优先复用；自编译时用与主工程一致的 MSVC toolset 和运行时模型（`/MD`）。

### Windows 编译器找不到 `stdio.h`

- **现象**：MSVC 编译大量报 `Cannot open include file: 'stdio.h'` 或类似的 Windows SDK 头文件缺失错误。
- **原因**：只把 `cl.exe` 加入了 `PATH`，但没有初始化 Visual Studio 开发环境，Windows SDK 和标准库的 include 路径未设置。
- **解决**：从 `x64 Native Tools Command Prompt for VS 2022` 启动构建，或在当前 shell 中先调用 `VsDevCmd.bat`：

  ```powershell
  cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 && set'
  ```

  `windows-msvc-release` preset 本身不含机器特定路径，依赖当前 shell 提供编译器、Ninja、Windows SDK 环境。

### 安装 VS Build Tools 后 `vcvars64.bat` 报"系统找不到指定的路径"

- **现象**：刚装完 Visual Studio Build Tools，调用 `vcvars64.bat` / `VsDevCmd.bat` 报"系统找不到指定的路径"（`The system cannot find the path specified`）。
- **原因**：安装器尚未完成环境注册（注册表项、组件清单未写入完成），当前系统会话拿到的仍是旧路径。
- **解决**（2026-08-18 实测）：重启 Windows 让安装完全注册，再重新打开 shell 调用 `vcvars64.bat`。重启后仍失败则检查 Build Tools 安装是否完整（C++ 工作负载、Windows SDK 组件）。

### 独立 benchmark 源文件在 MSVC 下编译失败：`__VERSION__` 未定义

- **现象**：全量构建时 `benchmark/native_dual_bfrt_simd_benchmark.cpp`（`main` 中打印编译器版本处）报编译错误，GCC 专有宏 `__VERSION__` 在 MSVC 下不存在。
- **原因**：该文件用 `__VERSION__` 打印编译器版本，是 GCC/Clang 专有宏。
- **解决**：已在源码中修复为 `_MSC_FULL_VER` 守护（MSVC 用 `_MSC_FULL_VER` 报告版本，与 `miplib2017_benchmark.cpp`、`netlib_solver_benchmark.cpp` 中既有的守护方式一致），当前无目标过滤的全量构建可通过。若你手上有该文件的本地改动覆盖了修复，请对齐到同一守护模式。

### `windows-offline-release` 配置直接失败

- **现象**：在无网络机器上 `cmake --preset windows-offline-release` 报依赖缺失/不兼容的配置错误。
- **原因**：这是预设的**预期行为**——离线 preset 要求 `third_party/install`（预编译第三方包）和 `third_party/oneapi-mkl`（仓库本地 oneMKL 静态包）已经由联网暂存机传过来，宁可提前失败也不静默探测网络或回退到本机构建。
- **解决**：在联网暂存机上完成 staging 后，把源树连同下列本地制品一起归档传输，再在无网机上运行 `cmake --preset windows-offline-release`：

  ```text
  third_party/oneapi-mkl/
  third_party/install/
  ```

  传输前记录哈希（见 [安装与部署](02-installation-deploy.md)），不要用 vcpkg preset 或依赖 `%MKLROOT%`。

### `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON` 报 manifest 不兼容

- **现象**：配置了 `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON` 后 configure 报第三方包不兼容；或 Debug 构建链接了 Release 的第三方包出错。
- **原因**：预编译第三方包**不是通用二进制缓存**，它绑定编译器、SDK、架构、运行时和配置。manifest 记录工具链身份；多配置生成器被限制为包所安装的单个配置，Release 依赖包不能链进 Debug MSVC 构建。
- **解决**：更换 MSVC toolset、目标架构、oneAPI 编译器/运行时或 MSVC 运行时模型后，必须重新生成预编译 `.lib`：

  ```powershell
  .\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
  ```

  如果只是想让配置继续，用 `AUTO`（默认，不匹配时警告并回退到源码构建）或 `OFF`（总是源码内构建）。

### 在源码根目录直接 configure 产生混乱

- **现象**：源码根目录出现 `CMakeCache.txt`、`CMakeFiles`，后续构建行为异常。
- **原因**：在源码根做 in-source configure 是仓库明确不支持的做法。
- **解决**：删除根目录的 `CMakeCache.txt` 和 `CMakeFiles/`，改用独立 build 目录（`cmake --preset ...` 或 `cmake -S . -B build/...`）。

### VS Code CMake Tools 无法 configure

- **现象**：VS Code 的 CMake Tools 插件配置失败，但命令行正常。
- **解决**：以终端版 `cmake` / `ctest` 命令为准（仓库文档的验证路径均基于终端命令）。把插件的 generator、环境变量设置与终端对齐，或干脆用终端流程。

### 缺失 vendored 源码树报错

- **现象**：configure 报某个 `third_party/` 或内嵌源码目录缺失，例如检查 `third_party/eigen/unsupported/Eigen/MatrixFunctions` 失败。
- **原因**：configure 不下载依赖，缺失的内嵌源码树被视为**致命的 checkout 错误**（防止裁剪过的 Eigen 头文件子集"静默通过"离线检查）。
- **解决**：重新做完整的仓库拷贝/检出，确保 `third_party/eigen` 等目录完整，不要用裁剪过的源码包。

## 10.2 依赖下载与网络

### curl 报 `CRYPT_E_REVOCATION_OFFLINE`（Windows schannel）

- **现象**：Windows 上用 `curl` 下载文件失败，报 schannel 错误 `CRYPT_E_REVOCATION_OFFLINE`。
- **原因**：Windows 自带的 schannel TLS 栈默认要求证书吊销列表（CRL）在线检查，目标机的吊销服务器不可达时直接拒绝连接。
- **解决**（2026-08-18 实测）：加 `--ssl-no-revoke` 跳过吊销检查：

  ```bash
  curl --ssl-no-revoke -L -o file.tar.gz https://example.org/file.tar.gz
  ```

### GitHub 直连不通

- **现象**：从 GitHub（`github.com` / `raw.githubusercontent.com`）下载依赖超时或连接重置。
- **解决**（2026-08-18 实测）：改用软件官方源，例如 zlib 用 <https://zlib.net> 而不是 GitHub 镜像。注意本仓库的 configure 本身不下载依赖——这只是在你需要补第三方源码（如 zlib、oneMKL）时的取数建议；正式发布仍应走离线 staging 流程（见 [安装与部署](02-installation-deploy.md)）。

### NETLIB 测试读文件失败

- **现象**：`test_netlib_regression` 或 NETLIB 基准报数据文件不存在/读取失败。
- **原因**：回归数据集不随源码仓库部署。注意读取失败与数值不收敛是两类问题，先确认是哪一类。
- **解决**：下载并生成 manifest 后再跑回归：

  ```powershell
  python tools\fetch_netlib.py --output-dir tests\data
  ```

## 10.3 求解器与结果异常

### 找不到求解器 / `preferred_solver` 不生效

- **现象**：指定的求解器名被拒绝，或实际执行的适配器不是预期后端。
- **原因**：外部后端只有在编译支持、可执行文件/动态库和许可证都可用时才注册；名称**区分大小写**，必须严格匹配 `list_solvers` 的返回字符串；`allow_fallback=true`（默认）时失败会切到其他适配器，`solver_name` 可能与你指定的不同。
- **解决**：先查询当前构建实际注册了哪些后端：

  ```cpp
  SolverEngine engine;
  for (const auto& name : engine.list_solvers(ProblemClass::LP)) {
    std::cout << name << '\n';
  }
  ```

  ```python
  print(mipsolvers.engine.list_solvers("LP"))
  ```

  需要可重复审计时，显式选择求解器并设 `allow_fallback=false`。

### `success=false` 但 `x` 非空

- **现象**：求解返回失败，但解向量里有数。
- **原因**：一些迭代法会返回最佳已知点；MILP 达到 time limit 时也可能有 incumbent。它不是自动可接受的结果。
- **解决**：检查 `status`、可行度和 gap，按业务规则决定是否使用：

  ```cpp
  if (!result.stats.success) {
    // 根据 status 区分 infeasible、unbounded、limit、numerical failure。
  }
  if (result.x.size() != expected_n || !result.x.allFinite()) {
    // 不使用尺寸错误或非有限的候选解。
  }
  if (is_milp && result.stats.mip_gap > required_gap) {
    // 可能有 incumbent，但未达到业务证明要求。
  }
  ```

  不要只检查 `x` 是否非空；生产代码至少检查 `success`、`status`、`solver_name` 和相应的可行度/gap。

### 没有对偶变量

- **现象**：`constraint_duals`、`box_dual_lb/ub` 为空，或 LMP 读不出来。
- **原因**：MILP 本身通常不返回有意义的行对偶；外部文件型适配器也可能只解析原始解。
- **解决**：固定整数决策后重解 LP/SCED，在 LP 路径上读取对偶；或选择明确支持对偶返回的 LP 后端。需要 LMP 时尤其如此。

### NLP 返回 `unknown` 或数值失败

- **现象**：`status` 为 `unknown` 或报告数值失败。
- **原因**：初值、函数定义域、导数或尺度问题。`unknown` **不能解释为不可行**。
- **解决**：检查初值是否落在 `log/sqrt/division` 的非法点；用有限差分独立核对解析梯度/Jacobian；核对约束方向（NLP 为 `g(x)=0, h(x)<=0`）；对数量级悬殊的变量/约束做物理单位归一化。NLP 成功必须同时检查原始可行度、对偶可行度和互补度。

### 近秩亏等式下解分量不稳定

- **现象**：近相关（近秩亏）等式约束的 NLP 用例解分量偏离预期（参考用例返回 `x=(1.001262, 0.998738)`，逐变量 `2e-5` 断言未过）。
- **原因**：这是 `test_ipopt_parameter_stability` 记录在案的已知风险项，反映近秩亏等式下解分量本身的条件数敏感，不一定是求解器 bug。
- **解决**：建模层面消除冗余等式（合并线性相关行）、改善尺度；判读结果时关注残差而非逐变量硬阈值。

### Windows 上调试输出/子进程路径异常（历史回归，已修复）

- **现象**：外部求解器子进程输出重定向失败、调试 JSON 写不到磁盘。
- **原因**：旧代码硬编码了 `/dev/null` 和 `/tmp`，Windows 没有这两个路径。
- **解决**：已修复——子进程输出在 Windows 重定向到 `NUL`，临时调试 JSON 改用 `std::filesystem::temp_directory_path()`。若日志里再出现 `/dev/null` 或 `/tmp` 字样，说明跑的是旧代码。回归验证命令：

  ```powershell
  ctest --test-dir build-win --build-config Release `
    -R "test_(engine_api|milp_solver|ipm_solver|scuc_module|market_simulation)" `
    --output-on-failure --parallel 2
  ```

## 10.4 测试与回归

### `test_milp_solver` 有一个用例失败：`production optimum matches exhaustive binary oracle`

- **现象**：54 个用例中 53 个通过，该用例断言失败，但求解器结果和用例内部穷举 oracle 都是 `-10`。
- **原因**：用例最后两个断言硬编码为 `-6`，是**测试期望与当前测试数据不一致**，不能据此判定求解器答案错误。
- **解决**：这是记录在案的已知测试问题（截至 2026-08-06 测试基线），不是求解器缺陷；发布判读时单独剔除该项。

### `test_ipopt_parameter_stability` 失败

- **现象**：近相关等式用例逐变量 `2e-5` 断言未过，其余 5 个实际执行用例通过。
- **原因**：见 10.3 节"近秩亏等式下解分量不稳定"——属于已知的数值稳定性风险项。
- **解决**：与上一条同样处理：单独记录，不按整体回归失败处理，但发布前应评估你的业务模型是否含近秩亏等式。

### SCUC 相关测试没跑 / `test_scuc_module` 未出现在 CTest 列表

- **现象**：`test_scuc_module`、`test_market_simulation`（共 19 个 `TEST_CASE`）不在当前 CTest 配置中。
- **原因**：这两个目标只在 `MIPSOLVERS_BUILD_SCUC=ON` 时注册。
- **解决**：configure 时启用 SCUC：

  ```powershell
  cmake -S . -B build-win -A x64 `
    -DMIPSOLVERS_BUILD_TESTS=ON `
    -DMIPSOLVERS_BUILD_PYTHON=OFF `
    -DMIPSOLVERS_BUILD_SCUC=ON `
    -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF `
    -DMIPSOLVERS_ENABLE_WERROR=OFF
  ```

  注意 `test_market_simulation` 已直接链接 `src/scuc/case_builder.cpp`，不需要单独构建 `scuc_case_builder` 可执行文件。未跑 SCUC 目标的构建不算完整发布验证。

### Eigen SparseLU 在空稀疏系统上崩溃/除零

- **现象**：`0x0` 空稀疏系统进入 Eigen SparseLU 时在 `SparseLU_Memory.h` 内除零崩溃。
- **原因**：旧代码未对空系统短路。
- **解决**：已修复——空 `0x0` 稀疏系统在调用 Eigen 或 SuiteSparse 后端之前短路返回。`test_engine_api` 覆盖了该回归（确认 `EigenSparseLUSolver` 接受空系统不崩溃）。

### 单元测试的 wall time 被当成性能基准

- **现象**：用 `ctest` 输出的时间对比求解器快慢。
- **原因**：CTest 标签行是累计 processor time，合计是并行执行的墙钟时间，两者口径不同，且单元测试未固定数据/线程/重复次数。
- **解决**：性能结论只使用 Release 构建、固定数据、固定线程和重复次数的基准目标（`-L benchmark`、NETLIB/MIPLIB 基准），见 [测试与基准](09-testing-benchmarks.md)。

## 10.5 部署与运行时

### 部署机上 DLL / 运行时缺失

- **现象**：可执行文件在开发机能跑，部署机报缺 DLL 或加载失败。
- **原因**：静态库检查通过不代表运行时依赖封闭。Windows preset 使用动态 MSVC 运行时（Release `/MD`、Debug `/MDd`）；oneMKL 构建还有按链接模型决定的 oneAPI 运行时 DLL。
- **解决**：对最终可执行文件（不是 `.lib`）逐一检查实际依赖：

  ```powershell
  dumpbin /DEPENDENTS application.exe
  ```

  ```bash
  ldd path/to/application      # Linux
  otool -L path/to/application # macOS
  ```

  预期 oneMKL 剖面为 LP64、sequential、static；若出现 `mkl_intel_thread.dll`、`mkl_sequential.dll` 等意外 oneAPI DLL，在链接模型修正或运行时 DLL 显式纳入离线制品前，该包不算密封。记得随应用分发 MSVC redistributable。

### macOS 部署报 `libgfortran` / `libomp` 缺失

- **现象**：目标机上动态链接器找不到 `libgfortran`、`libquadmath` 或 `libomp.dylib`。
- **原因**：vendored MUMPS 需要 Fortran 运行时；静态 GCC Fortran runtime 不可用时最终可执行文件会依赖对应 dylib。OpenMP 在 macOS 上一般不能静态链接。
- **解决**：随应用分发兼容的 `libomp.dylib` 并修正 install name/RPATH，或在目标机安装 `libomp`；用 `otool -L` 检查最终可执行文件确认。

### Linux 部署希望更自包含

- **解决**：开启 `MIPSOLVERS_STATIC_LIBGFORTRAN=ON` 静态链入 GNU Fortran 运行时，并确认目标发行版许可静态 `libgcc`/`libgfortran` 链接；用 `ldd` 复核最终 ELF 依赖。

### Gurobi 没有被使用

- **现象**：装了 Gurobi 但 `list_solvers` 里没有，或求解走了 HiGHS。
- **原因**：Gurobi 从不随仓库打包；只有当编译期检测到其头文件和库、且 `GRBloadenv()` 能初始化有效运行许可证时才注册为首选 LP/QP/MILP 后端。安装缺失、许可证缺失/过期或求解失败都会在 `allow_fallback=true`（默认）时自动回退到 HiGHS 和原生求解器。
- **解决**：确认编译环境能找到 Gurobi、运行机许可证有效；若发布包不允许链接 Gurobi，显式设 `MIPSOLVERS_USE_GUROBI=OFF`。

### 混用 MSVC 与 MinGW 产物链接失败

- **现象**：Windows 上链接报一堆无法解析的符号或 ABI 错误。
- **原因**：MSVC 库与 MinGW C++ 构建的库 ABI 不兼容。
- **解决**：不要混用。MSVC 构建配 MSVC 工具链的全部产物；若选 MinGW `gfortran` 做 source-only 数值剖面，整条链都用一致的 MinGW 工具链。

## 10.6 Python 接口

### Python 无法导入模块

- **现象**：`import mipsolvers` 报 `ModuleNotFoundError` 或 DLL 加载失败。
- **原因**：`PYTHONPATH` 未指向扩展文件所在目录；或 Python 版本/体系结构/ABI 与编译扩展时不一致；Windows 上还可能是依赖 DLL 不在扩展同目录或 `PATH`。
- **解决**：确认 `PYTHONPATH`、Python 版本、架构和 MSVC/libstdc++ ABI 与构建一致；**不能**把另一个 Python 版本生成的 `.pyd`/`.so` 直接复制过来用。验证：

  ```bash
  python -c "import mipsolvers; print(mipsolvers.engine.list_solvers('LP'))"
  ```

### 后端列表在不同机器上不一样

- **现象**：两台机器 `list_solvers` 返回的顺序/内容不同。
- **原因**：返回顺序是当前构建的候选优先级，取决于编译选项和本机可用后端（Gurobi、MKL 等），本来就不能假设所有机器完全相同。
- **解决**：发布前在目标部署机上调用 `list_solvers` 并保存实际可用后端清单，作为发布记录的一部分。

## 10.7 排查信息清单

向支持方或上游报告问题时，建议附上：

1. 平台、编译器版本（如 MSVC 19.44 / VS 2022 17.14）与构建 preset。
2. 完整 configure 命令与 CMake 选项。
3. `list_solvers` 的实际输出。
4. 失败的 `ctest --output-on-failure` 完整输出。
5. 求解问题的 `success / status / solver_name / primal_feas / mip_gap` 等字段值。
6. 部署问题的 `dumpbin /DEPENDENTS` / `ldd` / `otool -L` 输出。
7. 软件提交哈希与构建配置，保证问题可复现。
