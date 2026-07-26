# 通用 Branch-and-Benders-Cut 模块

实现位置：

- `include/mipsolvers/engine/decomposition/benders.hpp`
- `src/engine/decomposition/benders.cpp`

该模块属于 `mipsolvers::engine`，不依赖 SCUC、机组、时段或电网数据结构。
SCUC 只是通过公共接口选择主问题列，并将返回的列映射还原为业务结果。

## 两种接入方式

对于一个单体 `MIPModel`，调用方只需给出一阶段变量列：

```cpp
std::vector<int> first_stage = {/* original column indices */};
auto partition = engine::partition_benders_model(
    std::move(monolithic_model), first_stage);
if (!partition.success) {
  // partition.status contains the rejected model contract.
}

engine::BendersOptions options;
auto result = engine::solve_benders(std::move(partition.model), options);
```

`partition_benders_model()` 自动完成：

- 将纯一阶段约束放入紧凑主问题；
- 将包含 recourse 列的行放入连续子问题；
- 从混合行提取稀疏 RHS 耦合矩阵；
- 构造 `theta` 及由 recourse 变量边界得到的有效下界；
- 返回 `master_to_original` 和 `subproblem_to_original` 列映射；
- 保留不等式、等式及 ranged-row 的上下界语义。

已有显式分块的应用也可以直接构造 `BendersModel`。公共模型约定为：

```text
min  f'x + theta
s.t. x satisfies master constraints

Q(x) = min c'y
       s.t. A y <= b - B x
            E y  = d - D x
```

其中 `coupling_ineq = B`、`coupling_eq = D`，二者的列数必须等于紧凑
master 的总列数；`theta_col` 对应的耦合列必须为零。

## 模型契约

- 当前 master 和 recourse 都必须是最小化问题。
- recourse 必须全部为连续变量；整数 recourse 会在分块或求解验证阶段被拒绝。
- master 可以包含二进制、整数或连续变量。
- `theta` 必须是连续变量，且目标系数为 1。
- 自动分块时，一阶段列顺序由调用方决定；recourse 列保持原模型顺序。
- `coefficient_tolerance` 会从分块矩阵中删除绝对值不超过该阈值的系数，默认
  为零，避免未经调用方授权改变模型。

## 求解与正确性

`solve_benders()` 统一负责主问题迭代、上下界、gap 和割管理。最优 recourse
使用 LP 对偶生成最优性割；不可行 recourse 优先使用 Farkas ray 生成全局有效
可行性割。只有后端明确报告 LP 不可行时才允许 fallback；时间限制、数值失败
或残差审计失败不会被误判为不可行。

当除 `theta` 外的所有 master 变量均为二进制时，模块可以自动生成 no-good
fallback。对于更一般的 master，调用方可提供 `infeasible_fallback`，但返回割
必须对完整模型全局有效；无法取得有效证书时模块停止，且不会宣称收敛。

## 大规模路径

- 主问题矩阵持久化，每轮只追加一条规范 CSC 稀疏割；
- 主问题复用原生 B&C 伪成本，并可选择验证后的 incumbent 热启动；
- recourse 只构造一次 standard form 和 Ruiz 缩放；
- master 取值变化时只更新受耦合非零元影响的 RHS；
- recourse basis 和稀疏分解由 dual-simplex reoptimization 复用；
- API 按值接收大模型，调用方可用 `std::move` 避免保留矩阵副本。

该路径使用项目原生 B&C 与 LP kernel，不依赖 Gurobi。百万或千万变量是否可行
仍取决于 recourse 稀疏度、耦合非零元、主问题树规模、内存和有效割数量；模块
提供线性稀疏存储和增量重优化路径，但不把某个未实测规模写成性能保证。

## 遥测

`BendersResult::stats` 报告迭代和割数量、耦合列/非零元、增量 RHS 更新次数、
主问题节点与 LP 次数、incumbent/伪成本复用、子问题 basis 复用、单纯形迭代和
两侧耗时。应用层应使用这些字段定位瓶颈，而不是依赖领域专用日志。
