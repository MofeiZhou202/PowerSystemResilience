# MIPSolvers 0.1.0 Windows x64 SDK

构建日期：2026-08-05  
源码分支：`release/windows-self-contained`  
基准提交：`2a78c3c`  
源码状态：包含该提交之后当前工作树中尚未提交的修改。

## 包含内容

- `lib/mipsolvers.lib`：MIPSolvers C++20 静态库；
- `lib/highs.lib`、`lib/fmt.lib`：随导出目标提供的静态依赖；
- `include/`：MIPSolvers、Eigen、nlohmann/json 与 fmt 公开头文件；
- `lib/cmake/mipsolvers/`：供 `find_package(mipsolvers CONFIG REQUIRED)` 使用；
- `bin/scuc_solve.exe`、`bin/scuc_case_builder.exe`：SCUC 命令行工具；
- `share/mipsolvers/docs/`：中文用户手册、算法说明和测试记录。

## 构建配置

- Windows x64、Visual Studio 2022 MSVC 19.44、Release、C++20；
- 动态 MSVC 运行库 `/MD`，部署机需要相应的 Microsoft Visual C++
  Redistributable；
- 内置求解后端：HiGHS、StrictHiGHS、NativeBranchAndCut 及其他原生求解器；
- 为保证 SDK 不携带本机专有路径，本包未启用 SCIP、Ipopt、Gurobi、MKL、
  SuiteSparse、SuperLU、MUMPS、OpenMP 和 Python 扩展。

## CMake 接入

```cmake
find_package(mipsolvers CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

配置消费工程时把本目录传给 CMake：

```powershell
cmake -S . -B build -DCMAKE_PREFIX_PATH=C:/path/to/MIPSolvers-0.1.0-windows-x64-sdk
cmake --build build --config Release
```

已使用包外 `cmake/consumer-example` 完成配置、编译、链接和 LP 求解验证，
目标值为 `9.000000`。
