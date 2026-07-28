# LP 数值内核选择与参数传播契约

## 目标

MIPSolvers 的原生模块可以继续拥有分支定界树、割、启发式和业务语义，
但生产 LP 数值计算统一由内置 HiGHS 完成。实验性 native dual simplex 只允许测试和基准显式选择。

## 唯一公共参数

```cpp
enum class LpKernelBackend {
  HiGHS,
  ExperimentalNative,
};
```

- `SimplexOptions::lp_kernel_backend`：单次 LP 求解的最终后端选择。
- `BCOptions::lp_kernel_backend`：原生 B&C 拥有的全部 LP 松弛的后端选择。
- 两者默认均为 `LpKernelBackend::HiGHS`。
- `ExperimentalNative` 不表示 fallback，只表示一次明确的开发求解。

旧的 `allow_vendored_highs_sf_backend`、`require_vendored_highs_sf_backend` 和
`use_vendored_highs_lp_kernel` C++ 字段已删除。Python 的
`use_vendored_highs_lp_kernel` 仅是兼容属性，直接映射到枚举，不保存第二份状态。

## 参数流

```text
BCOptions::lp_kernel_backend
  -> DispatcherConfig::lp_kernel_backend
  -> SimplexOptions::lp_kernel_backend
  -> solve_lp_from_sf_impl()
       HiGHS              -> run once; rejection is terminal
       ExperimentalNative -> run native dual once
```

所有绕过 `SolverDispatcher` 的 B&C 辅助 LP（CGLP、修复、pump、diving、probing、
并行节点）也必须从所属 `BCOptions` 复制同一个枚举。禁止使用环境变量、线程局部开关
或 allow/require 布尔组合改变实际后端；环境变量也不得切换 direct-LP、live-append
或 root-frontier 路径。环境变量仅允许控制不改变算法行为的 trace/diagnostic 输出。

## 与完整 MIP 策略的边界

`lp_kernel_backend = HiGHS` 只把 LP 数值计算交给 HiGHS，不改变 presolve、branching、
node selection、tree ownership、cuts、conflict learning、heuristics 或证明策略。

`BCOptions::strict_highs_mip_contract` 是独立的显式开关。只有该开关可以启用
StrictHiGHS 完整 MIP 策略及其 policy overrides。设置 LP 后端不得隐式设置它；设置它
则会把 LP 后端规范化为 HiGHS。

## 不变量与测试

1. 默认 `SimplexOptions{}` 的 `solver_name` 必须是 `VendoredHighsLpKernel`。
2. 默认 `BCOptions{}` 使用 HiGHS LP，同时 `strict_highs_mip_contract == false`。
3. native dual 测试必须显式写 `ExperimentalNative`。
4. HiGHS 返回拒绝或数值失败后，不得继续执行 native dual。
5. LP 后端选择不得改写任何非 LP 的 B&C 配置字段。

主要门禁：`test_dual_simplex`、`test_lp_solver` 和 `test_milp_solver`。
