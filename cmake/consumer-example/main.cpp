/**
 * consumer_example — 演示如何在外部工程中使用 MIPSolvers 静态库。
 *
 * 示例：用 AML (代数建模层) 建立并求解一个最小 LP
 *
 *   min   2x + 3y
 *   s.t.  x + y  >= 4          (resource)
 *         x      >= 1          (lb_x)
 *             y  >= 1          (lb_y)
 *         x, y continuous, unconstrained upper bound
 *
 * 最优解：x=1, y=3 → obj=11 (或 x=3,y=1 → obj=9 — 实际最优 x=1,y=3 → 2+9=11
 * 不对，让我重算：obj = 2*1 + 3*3 = 11 vs 2*3 + 3*1 = 9，最优是 x=3,y=1 → 9)
 * 实际：KKT 最优在 x+y=4 边界上，令 d(2x+3y)/dy=3>2，所以 y 尽量小 → y=1,x=3，obj=9。
 */

#include <cassert>
#include <cmath>
#include <cstdio>

#include "mipsolvers/aml/aml.hpp"
#include "mipsolvers/aml/model.hpp"
#include "mipsolvers/aml/variable.hpp"

int main() {
    // 创建模型
    mipsolvers::aml::Model model("lp_demo");

    // 定义单元素集合（AML 变量必须定义在集合上）
    auto& S = model.add_set("S", {"i0"});

    // 定义决策变量：x, y ∈ [0, +∞)
    auto& xv = model.add_var("x", S, mipsolvers::aml::VarType::Continuous, 0.0, 1e30);
    auto& yv = model.add_var("y", S, mipsolvers::aml::VarType::Continuous, 0.0, 1e30);

    const auto x = xv("i0");
    const auto y = yv("i0");

    // 约束
    model.add_constraint(x + y >= 4.0, "resource");
    model.add_constraint(x >= 1.0,     "lb_x");
    model.add_constraint(y >= 1.0,     "lb_y");

    // 目标：min 2x + 3y
    model.minimize(2.0 * x + 3.0 * y);

    // 求解（使用默认求解器，通常为内嵌 HiGHS）
    mipsolvers::aml::SolveOptions opts;
    opts.verbosity = 1;
    auto result = model.solve(opts);

    // 检验结果
    if (result.termination_status != mipsolvers::aml::TerminationStatus::Optimal) {
        std::printf("ERROR: 求解未达到最优状态\n");
        return 1;
    }

    double obj = result.objective_value;
    std::printf("最优目标值: %.6f  (期望: 9.000000)\n", obj);
    assert(std::abs(obj - 9.0) < 1e-4);

    std::printf("consumer_example 运行成功。\n");
    return 0;
}
