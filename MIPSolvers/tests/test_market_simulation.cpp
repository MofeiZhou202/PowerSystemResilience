/// test_market_simulation.cpp
///
/// SCUC 市场模拟基准测试与结果可视化
///
/// 功能：
///   1. 对所有已注册的可用 MILP 求解器
///      在 3-bus、6-bus、IEEE 39-bus 算例上分别测速，输出对比表格
///   2. ASCII 图形化输出：机组组合 Gantt 图、调度出力时序图、LMP 热图
///   3. 物理一致性验证：功率平衡、备用满足、SOC 边界

#include <catch2/catch_all.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/scuc/case_builder.hpp"
#include "mipsolvers/scuc/scuc.hpp"

using namespace mipsolvers::scuc;
using namespace mipsolvers::engine;
namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// 计时辅助
// ─────────────────────────────────────────────────────────────────────────────

struct WallTimer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point t0{Clock::now()};
    double elapsed_sec() const {
        return std::chrono::duration<double>(Clock::now() - t0).count();
    }
};

struct BenchResult {
    std::string solver_name;
    std::string case_name;
    bool        converged{false};
    double      wall_sec{0.0};
    double      solver_sec{0.0};
    double      objective{0.0};
    double      mip_gap{0.0};
    int         n_cuts{0};
};

static fs::path debug_output_path(const std::string& filename) {
    std::error_code ec;
    fs::path dir = fs::temp_directory_path(ec);
    if (ec || dir.empty()) {
        ec.clear();
        dir = fs::current_path(ec);
    }
    if (ec || dir.empty()) {
        return fs::path(filename);
    }
    return dir / filename;
}

// ─────────────────────────────────────────────────────────────────────────────
// ASCII 可视化工具
// ─────────────────────────────────────────────────────────────────────────────

namespace viz {

// ── 工具函数 ────────────────────────────────────────────────────────────────

static std::string repeat(char c, int n) {
    return n > 0 ? std::string(static_cast<size_t>(n), c) : "";
}

static std::string pad_right(const std::string& s, int width) {
    if (static_cast<int>(s.size()) >= width) return s.substr(0, static_cast<size_t>(width));
    return s + repeat(' ', width - static_cast<int>(s.size()));
}

static std::string pad_left(const std::string& s, int width) {
    if (static_cast<int>(s.size()) >= width) return s.substr(0, static_cast<size_t>(width));
    return repeat(' ', width - static_cast<int>(s.size())) + s;
}

static std::string fmt_f(double v, int w, int d) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(d) << std::setw(w) << v;
    return ss.str();
}

// ── 水平条形图 ───────────────────────────────────────────────────────────────
// 打印求解器耗时对比图

static void bar_chart(const std::string& title,
                      const std::vector<std::string>& labels,
                      const std::vector<double>& values,
                      const std::string& unit = "s",
                      int bar_width = 38) {
    double max_val = *std::max_element(values.begin(), values.end());
    if (max_val <= 0.0) max_val = 1.0;

    int lw = 0;
    for (auto& l : labels) lw = std::max(lw, static_cast<int>(l.size()));
    lw = std::min(lw + 1, 20);

    std::cout << "\n" << title << "\n" << repeat('=', lw + bar_width + 14) << "\n";
    for (size_t i = 0; i < labels.size(); ++i) {
        int filled = static_cast<int>(values[i] / max_val * bar_width);
        filled = std::clamp(filled, 0, bar_width);
        std::cout << pad_right(labels[i], lw) << " |"
                  << repeat('#', filled) << repeat('-', bar_width - filled)
                  << "| " << fmt_f(values[i], 7, 3) << " " << unit << "\n";
    }
    std::cout << repeat('-', lw + bar_width + 14) << "\n";
}

// ── 对比表格 ────────────────────────────────────────────────────────────────

static void comparison_table(const std::string& title,
                              const std::vector<std::string>& headers,
                              const std::vector<std::vector<std::string>>& rows) {
    // 计算列宽
    std::vector<int> widths(headers.size(), 0);
    for (size_t c = 0; c < headers.size(); ++c)
        widths[c] = static_cast<int>(headers[c].size());
    for (auto& row : rows)
        for (size_t c = 0; c < row.size() && c < widths.size(); ++c)
            widths[c] = std::max(widths[c], static_cast<int>(row[c].size()));
    for (auto& w : widths) w += 2;

    // 水平分隔线
    auto hline = [&]() {
        std::cout << "+";
        for (int w : widths) std::cout << repeat('-', w) << "+";
        std::cout << "\n";
    };

    std::cout << "\n" << title << "\n";
    hline();
    std::cout << "|";
    for (size_t c = 0; c < headers.size(); ++c)
        std::cout << " " << pad_right(headers[c], widths[c] - 2) << " |";
    std::cout << "\n";
    hline();
    for (auto& row : rows) {
        std::cout << "|";
        for (size_t c = 0; c < headers.size(); ++c) {
            std::string cell = c < row.size() ? row[c] : "";
            std::cout << " " << pad_right(cell, widths[c] - 2) << " |";
        }
        std::cout << "\n";
    }
    hline();
}

// ── 机组组合 Gantt 图 ────────────────────────────────────────────────────────
// commitment[g][t] ∈ {0,1}；██ = 在线，░░ = 离线

static void gantt_chart(const std::string& title,
                        const std::vector<std::string>& gen_names,
                        const Matrix2D& commitment,
                        int T) {
    const int ng = static_cast<int>(gen_names.size());
    int nw = 0;
    for (auto& n : gen_names) nw = std::max(nw, static_cast<int>(n.size()));
    nw = std::min(nw + 1, 12);

    // t 轴刻度（每 6 格打一个时刻）
    auto t_axis = [&]() {
        std::cout << pad_right("", nw) << "  ";
        for (int t = 0; t < T; ++t) {
            if (t % 6 == 0)
                std::cout << pad_left(std::to_string(t), 1);
            else
                std::cout << " ";
        }
        std::cout << "\n";
    };

    std::cout << "\n" << title << "  (■=在线  ░=离线)\n";
    std::cout << pad_right("", nw + 2) << repeat('-', T + 2) << "\n";
    for (int g = 0; g < ng && g < static_cast<int>(commitment.size()); ++g) {
        std::cout << pad_right(gen_names[static_cast<size_t>(g)], nw) << " |";
        for (int t = 0; t < T && t < static_cast<int>(commitment[static_cast<size_t>(g)].size()); ++t) {
            bool on = commitment[static_cast<size_t>(g)][static_cast<size_t>(t)] > 0.5;
            std::cout << (on ? "\u2588" : "\u2591");  // █ / ░
        }
        std::cout << "|\n";
    }
    std::cout << pad_right("", nw + 2) << repeat('-', T + 2) << "\n";
    t_axis();
}

// ── 时序折线图 ───────────────────────────────────────────────────────────────
// 支持最多 5 条曲线，使用不同符号

static void time_series(const std::string& title,
                        const std::vector<std::string>& names,
                        const std::vector<std::vector<double>>& series,
                        const std::string& ylabel = "MW",
                        int height = 10,
                        int width  = 60) {
    if (series.empty()) return;
    const int n_series = static_cast<int>(series.size());
    const int T = static_cast<int>(series[0].size());
    if (T == 0) return;

    // 求全局 min/max
    double gmin = 1e18, gmax = -1e18;
    for (auto& s : series)
        for (double v : s) { gmin = std::min(gmin, v); gmax = std::max(gmax, v); }
    if (std::abs(gmax - gmin) < 1e-9) gmax = gmin + 1.0;

    // 建字符画布
    std::vector<std::string> canvas(static_cast<size_t>(height),
                                    std::string(static_cast<size_t>(width), ' '));
    const char symbols[] = {'*', 'o', '+', 'x', '#'};

    auto y_to_row = [&](double v) {
        int r = static_cast<int>((gmax - v) / (gmax - gmin) * (height - 1) + 0.5);
        return std::clamp(r, 0, height - 1);
    };
    auto t_to_col = [&](int t) {
        return std::clamp(static_cast<int>(static_cast<double>(t) / (T - 1) * (width - 1)), 0, width - 1);
    };

    for (int si = 0; si < std::min(n_series, 5); ++si) {
        char sym = symbols[si];
        for (int t = 0; t < T; ++t) {
            int r = y_to_row(series[static_cast<size_t>(si)][static_cast<size_t>(t)]);
            int c = (T == 1) ? 0 : t_to_col(t);
            canvas[static_cast<size_t>(r)][static_cast<size_t>(c)] = sym;
        }
    }

    // 打印标题和图
    const int ylabel_w = 8;
    std::cout << "\n" << title << "  (" << ylabel << ")\n";
    for (int r = 0; r < height; ++r) {
        double val = gmax - (gmax - gmin) * r / (height - 1);
        std::cout << pad_left(fmt_f(val, 6, 1), ylabel_w) << " |" << canvas[static_cast<size_t>(r)] << "|\n";
    }
    // t 轴
    std::cout << repeat(' ', ylabel_w + 2) << "+" << repeat('-', width) << "+\n";
    std::cout << repeat(' ', ylabel_w + 3);
    for (int mark = 0; mark <= 4; ++mark) {
        int t = mark * (T - 1) / 4;
        int c = (T == 1) ? 0 : t_to_col(t);
        int pos = c - (mark > 0 ? static_cast<int>(std::to_string(t).size()) : 0);
        (void)pos;
        std::cout << pad_left(std::to_string(t), width / 5);
    }
    std::cout << "   t\n";
    // 图例
    std::cout << "    图例: ";
    for (int si = 0; si < std::min(n_series, 5); ++si)
        std::cout << symbols[si] << "=" << names[static_cast<size_t>(si)] << "  ";
    std::cout << "\n";
}

// ── LMP 热图 ────────────────────────────────────────────────────────────────
// 按电价区间着色：低=. 中=: 高=# 极高=!

static void lmp_heatmap(const std::string& title,
                        const Matrix2D& nodal_lmp,
                        int max_buses   = 8,
                        int max_periods = 24) {
    if (nodal_lmp.empty()) return;
    const int nb = static_cast<int>(std::min(static_cast<int>(nodal_lmp.size()), max_buses));
    const int T  = static_cast<int>(std::min(
        static_cast<int>(nodal_lmp[0].size()), max_periods));

    double lmin = 1e18, lmax = -1e18;
    for (int b = 0; b < nb; ++b)
        for (int t = 0; t < T; ++t) {
            lmin = std::min(lmin, nodal_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)]);
            lmax = std::max(lmax, nodal_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)]);
        }
    if (lmax - lmin < 1e-9) lmax = lmin + 1.0;

    auto lmp_char = [&](double v) -> char {
        double ratio = (v - lmin) / (lmax - lmin);
        if (ratio < 0.25) return '.';
        if (ratio < 0.50) return ':';
        if (ratio < 0.75) return '#';
        return '!';
    };

    const int bw = 7;  // bus label width
    std::cout << "\n" << title << "\n";
    std::cout << "  (" << fmt_f(lmin, 5, 1) << "$/MWh=. "
              << fmt_f(lmin + (lmax-lmin)*0.25, 5, 1) << "=: "
              << fmt_f(lmin + (lmax-lmin)*0.50, 5, 1) << "=# "
              << fmt_f(lmax, 5, 1) << "=!)\n";
    // 时段 header
    std::cout << pad_right("Bus", bw) << " |";
    for (int t = 0; t < T; ++t)
        std::cout << (t % 6 == 0 ? std::to_string(t % 10) : " ");
    std::cout << "|\n" << repeat('-', bw + T + 3) << "\n";
    for (int b = 0; b < nb; ++b) {
        std::cout << pad_right("Bus" + std::to_string(b), bw) << " |";
        for (int t = 0; t < T; ++t)
            std::cout << lmp_char(nodal_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)]);
        std::cout << "|\n";
    }
    std::cout << repeat('-', bw + T + 3) << "\n";
}

// ── 求解器性能对比报告 ────────────────────────────────────────────────────────

static void print_benchmark_report(const std::string& title,
                                   const std::vector<BenchResult>& results) {
    // 按耗时排序（收敛优先）
    auto sorted = results;
    std::sort(sorted.begin(), sorted.end(), [](const BenchResult& a, const BenchResult& b) {
        if (a.converged != b.converged) return a.converged > b.converged;
        return a.wall_sec < b.wall_sec;
    });

    // 条形图
    std::vector<std::string> labels;
    std::vector<double> times;
    for (auto& r : sorted) {
        labels.push_back(r.solver_name + (r.converged ? "" : "(未收敛)"));
        times.push_back(r.converged ? r.wall_sec : 0.0);
    }
    bar_chart(title + " — 耗时对比", labels, times, "s", 36);

    // 详细表格
    std::vector<std::string> headers = {"求解器", "收敛", "耗时(s)", "目标值($)", "MIP间隙", "割平面"};
    std::vector<std::vector<std::string>> rows;
    for (auto& r : sorted) {
        rows.push_back({
            r.solver_name,
            r.converged ? "YES" : "NO",
            fmt_f(r.wall_sec, 7, 3),
            r.converged ? fmt_f(r.objective, 10, 1) : "N/A",
            r.converged ? fmt_f(r.mip_gap * 100.0, 6, 3) + "%" : "N/A",
            std::to_string(r.n_cuts)
        });
    }
    comparison_table(title + " — 详细结果", headers, rows);
}

}  // namespace viz

// ─────────────────────────────────────────────────────────────────────────────
// 获取所有可用 MILP 求解器名称
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<std::string> available_milp_solvers() {
    SolverEngine eng;
    return eng.list_solvers(ProblemClass::MILP);
}

// 使用指定求解器求解，返回 BenchResult
static BenchResult solve_with(const std::string& solver_name,
                              const std::string& case_name,
                              SCUCInput inp) {
    inp.config.solver         = solver_name;
    inp.config.allow_fallback = false;  // 不回退，准确测试该求解器
    inp.config.solve_sced     = false;  // 仅测 MILP 阶段耗时
    inp.config.solve_lmp      = false;

    BenchResult res;
    res.solver_name = solver_name;
    res.case_name   = case_name;

    WallTimer timer;
    try {
        SCUCOutput out = scuc_solve(inp);
        res.wall_sec   = timer.elapsed_sec();
        res.converged  = out.scuc.converged;
        res.objective  = out.scuc.objective;
        res.mip_gap    = out.scuc.mip_gap;
        res.solver_sec = out.scuc.solve_time_sec;
        res.n_cuts     = out.scuc.n_cuts_added;
    } catch (...) {
        res.wall_sec  = timer.elapsed_sec();
        res.converged = false;
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 1: 3-bus 全求解器性能基准
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 3-bus 全求解器性能基准 (T=6)", "[market][benchmark]") {
    const auto solvers = available_milp_solvers();
    REQUIRE_FALSE(solvers.empty());

    SCUCInput base = build_3bus_case(6, 1.0);

    std::vector<BenchResult> results;
    for (const auto& s : solvers) {
        DYNAMIC_SECTION("求解器=" << s) {
            auto res = solve_with(s, "3bus-T6", base);
            results.push_back(res);
            // 只要能列出就算通过（即使不收敛也记录）
            SUCCEED();
        }
    }

    // 验证 Auto 模式必须收敛
    {
        SCUCInput inp = base;
        inp.config.solver         = "Auto";
        inp.config.allow_fallback = true;
        inp.config.solve_sced     = false;
        inp.config.solve_lmp      = false;
        SCUCOutput out = scuc_solve(inp);
        REQUIRE(out.scuc.converged);

        BenchResult auto_res;
        auto_res.solver_name = "Auto(回退)";
        auto_res.case_name   = "3bus-T6";
        auto_res.converged   = out.scuc.converged;
        auto_res.objective   = out.scuc.objective;
        auto_res.mip_gap     = out.scuc.mip_gap;
        auto_res.n_cuts      = out.scuc.n_cuts_added;
        results.push_back(auto_res);
    }

    viz::print_benchmark_report("3-bus T=6", results);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 2: 6-bus 含风电储能 全求解器基准
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 6-bus 风电+储能 全求解器基准 (T=8)", "[market][benchmark]") {
    const auto solvers = available_milp_solvers();
    REQUIRE_FALSE(solvers.empty());

    SCUCInput base = build_6bus_case(8, 1.0, /*wind=*/true, /*storage=*/true);

    std::vector<BenchResult> results;
    for (const auto& s : solvers) {
        DYNAMIC_SECTION("求解器=" << s) {
            auto res = solve_with(s, "6bus-T8-wind-sto", base);
            results.push_back(res);
            SUCCEED();
        }
    }

    viz::print_benchmark_report("6-bus T=8 (风电+储能)", results);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3: IEEE 39-bus 快速基准 (T=4, 只用最快求解器)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: IEEE 39-bus 快速基准 (T=4)", "[market][benchmark][ieee39]") {
    const auto solvers = available_milp_solvers();
    REQUIRE_FALSE(solvers.empty());

    SCUCInput base = build_ieee39_case(4, 1.0, /*wind=*/true, /*solar=*/false);
    base.config.mip_gap      = 0.01;  // 放宽间隙以加速
    base.config.time_limit_sec = 120.0;

    std::vector<BenchResult> results;
    for (const auto& s : solvers) {
        DYNAMIC_SECTION("求解器=" << s) {
            auto res = solve_with(s, "ieee39-T4", base);
            results.push_back(res);
            SUCCEED();
        }
    }

    viz::print_benchmark_report("IEEE 39-bus T=4", results);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3b: IEEE 39-bus 24h 风电+光伏 全求解器基准
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: IEEE 39-bus 24h 风电+光伏 全求解器基准", "[market][benchmark][ieee39]") {
    const auto solvers = available_milp_solvers();
    REQUIRE_FALSE(solvers.empty());

    SCUCInput base = build_ieee39_case(24, 1.0, /*wind=*/true, /*solar=*/true);
    base.config.mip_gap        = 0.01;
    base.config.time_limit_sec = 300.0;

    std::vector<BenchResult> results;
    for (const auto& s : solvers) {
        DYNAMIC_SECTION("求解器=" << s) {
            auto res = solve_with(s, "ieee39-T24-wind-solar", base);
            results.push_back(res);
            SUCCEED();
        }
    }

    viz::print_benchmark_report("IEEE 39-bus T=24 (风电+光伏)", results);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3c: IEEE 39-bus 24h 全资源 (风+光+储) 全求解器基准
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: IEEE 39-bus 24h 全资源 (风+光+储) 基准", "[market][benchmark][ieee39]") {
    const auto solvers = available_milp_solvers();
    REQUIRE_FALSE(solvers.empty());

    SCUCInput base = build_ieee39_case(24, 1.0, /*wind=*/true, /*solar=*/true);
    base.config.mip_gap        = 0.01;
    base.config.time_limit_sec = 300.0;

    // 添加 2 个网侧大型储能（接入主要负荷母线）
    {
        StorageUnit batt1;
        batt1.bus                 = 3;     // bus 3: 500 MW 负荷母线
        batt1.pmax_charge         = 200.0;
        batt1.pmax_discharge      = 200.0;
        batt1.energy_capacity_mwh = 800.0;
        batt1.efficiency          = 0.90;
        batt1.soc_init            = 0.50;
        batt1.charge_bid_price    = 5.0;
        batt1.discharge_bid_price = 20.0;
        base.storage.push_back(batt1);

        StorageUnit batt2;
        batt2.bus                 = 19;    // bus 19: 628 MW 负荷母线
        batt2.pmax_charge         = 150.0;
        batt2.pmax_discharge      = 150.0;
        batt2.energy_capacity_mwh = 600.0;
        batt2.efficiency          = 0.90;
        batt2.soc_init            = 0.50;
        batt2.charge_bid_price    = 5.0;
        batt2.discharge_bid_price = 20.0;
        base.storage.push_back(batt2);

        base.initial_status.storage_soc = {0.5, 0.5};
    }

    std::vector<BenchResult> results;
    for (const auto& s : solvers) {
        DYNAMIC_SECTION("求解器=" << s) {
            auto res = solve_with(s, "ieee39-T24-wind-solar-sto", base);
            results.push_back(res);
            SUCCEED();
        }
    }

    viz::print_benchmark_report("IEEE 39-bus T=24 (风+光+储)", results);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 4: 3-bus 24h 机组组合 Gantt 图可视化
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 3-bus 24h 机组组合 Gantt 图", "[market][viz]") {
    SCUCInput inp = build_3bus_case(24, 1.0);
    inp.config.solver = "Auto";

    SCUCOutput out = scuc_solve(inp);
    REQUIRE(out.scuc.converged);
    REQUIRE_FALSE(out.scuc.commitment.empty());

    const int ng = static_cast<int>(inp.generators.size());
    const int T  = inp.config.num_periods;

    std::vector<std::string> names;
    for (auto& g : inp.generators) names.push_back(g.name);

    viz::gantt_chart("3-bus 24h 机组组合", names, out.scuc.commitment, T);

    // 检查：每时段至少一台机组在线
    for (int t = 0; t < T; ++t) {
        double sum_on = 0.0;
        for (int g = 0; g < ng; ++g)
            sum_on += out.scuc.commitment[static_cast<size_t>(g)][static_cast<size_t>(t)];
        CHECK(sum_on >= 1.0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 5: 3-bus 24h 出力时序图 + SCED 对比
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 3-bus 24h 出力时序图（SCUC vs SCED）", "[market][viz]") {
    SCUCInput inp = build_3bus_case(24, 1.0);
    inp.config.solver    = "Auto";
    inp.config.solve_sced = true;
    inp.config.solve_lmp  = false;

    SCUCOutput out = scuc_solve(inp);
    REQUIRE(out.scuc.converged);

    const int T  = inp.config.num_periods;
    const int ng = static_cast<int>(inp.generators.size());

    // 汇总每时段系统总出力
    std::vector<double> scuc_total(static_cast<size_t>(T), 0.0);
    std::vector<double> sced_total(static_cast<size_t>(T), 0.0);
    std::vector<double> load_total(static_cast<size_t>(T), 0.0);

    for (int t = 0; t < T; ++t) {
        for (int g = 0; g < ng; ++g) {
            if (!out.scuc.dispatch.empty())
                scuc_total[static_cast<size_t>(t)] +=
                    out.scuc.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
            if (out.sced.converged && !out.sced.dispatch.empty())
                sced_total[static_cast<size_t>(t)] +=
                    out.sced.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
        }
        for (size_t d = 0; d < inp.loads.size(); ++d) {
            double pu = (!inp.profiles.load.empty() && d < inp.profiles.load.size() &&
                         t < static_cast<int>(inp.profiles.load[d].size()))
                        ? inp.profiles.load[d][static_cast<size_t>(t)]
                        : 1.0;
            load_total[static_cast<size_t>(t)] += inp.loads[d].p_mw * pu;
        }
    }

    // 逐台机组出力时序
    std::vector<std::string> gen_names;
    std::vector<std::vector<double>> gen_dispatch;
    for (int g = 0; g < ng; ++g) {
        gen_names.push_back(inp.generators[static_cast<size_t>(g)].name + "(SCUC)");
        if (!out.scuc.dispatch.empty())
            gen_dispatch.push_back(out.scuc.dispatch[static_cast<size_t>(g)]);
    }
    if (out.sced.converged)
        gen_names.push_back("Total-SCED");
    gen_dispatch.push_back(sced_total);
    gen_names.push_back("Load");
    gen_dispatch.push_back(load_total);

    viz::time_series("3-bus 24h 调度出力时序", gen_names, gen_dispatch, "MW");

    // 每台机每时段单独打印
    std::cout << "\n机组逐时段出力（SCUC 结果）:\n";
    std::cout << std::setw(6) << "时段";
    for (int g = 0; g < ng; ++g)
        std::cout << std::setw(10) << inp.generators[static_cast<size_t>(g)].name;
    std::cout << std::setw(12) << "总出力MW" << std::setw(12) << "负荷MW" << "\n";
    std::cout << viz::repeat('-', 6 + ng * 10 + 24) << "\n";
    for (int t = 0; t < T; ++t) {
        std::cout << std::setw(6) << t;
        for (int g = 0; g < ng; ++g) {
            double d = out.scuc.dispatch.empty() ? 0.0
                       : out.scuc.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
            std::cout << std::setw(10) << viz::fmt_f(d, 8, 1);
        }
        std::cout << std::setw(12) << viz::fmt_f(scuc_total[static_cast<size_t>(t)], 10, 1)
                  << std::setw(12) << viz::fmt_f(load_total[static_cast<size_t>(t)], 10, 1)
                  << "\n";
    }

    // 功率平衡验证（dispatch + load_shedding ≈ load，误差 < 5 MW）
    for (int t = 0; t < T; ++t) {
        double ls = (!out.scuc.load_shedding.empty())
                    ? out.scuc.load_shedding[static_cast<size_t>(t)] : 0.0;
        double gc = (!out.scuc.gen_curtailment.empty())
                    ? out.scuc.gen_curtailment[static_cast<size_t>(t)] : 0.0;
        CHECK(std::abs(scuc_total[static_cast<size_t>(t)] + ls - gc
                       - load_total[static_cast<size_t>(t)]) < 5.0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 6: 6-bus 含风电储能 24h LMP 热图
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 6-bus 24h LMP 热图 + 节点电价对比", "[market][viz]") {
    SCUCInput inp = build_6bus_case(24, 1.0, /*wind=*/true, /*storage=*/false);
    inp.config.solver    = "Auto";
    inp.config.solve_sced = true;
    inp.config.solve_lmp  = true;

    SCUCOutput out = scuc_solve(inp);
    REQUIRE(out.scuc.converged);

    if (!out.lmp.converged) {
        WARN("LMP 未收敛，跳过 LMP 可视化");
        return;
    }

    viz::lmp_heatmap("6-bus 24h 节点边际电价热图 ($/MWh)", out.lmp.nodal_lmp);

    // LMP 统计汇总表
    const int nb = static_cast<int>(inp.num_buses);
    const int T  = inp.config.num_periods;

    std::vector<std::string> headers = {"母线", "均值($/MWh)", "最大($/MWh)", "最小($/MWh)", "峰谷差"};
    std::vector<std::vector<std::string>> rows;
    for (int b = 0; b < nb && b < static_cast<int>(out.lmp.nodal_lmp.size()); ++b) {
        auto& lmp_b = out.lmp.nodal_lmp[static_cast<size_t>(b)];
        double avg = 0.0, mx = -1e18, mn = 1e18;
        for (double v : lmp_b) { avg += v; mx = std::max(mx, v); mn = std::min(mn, v); }
        avg /= lmp_b.size();
        rows.push_back({
            "Bus" + std::to_string(b),
            viz::fmt_f(avg, 7, 2),
            viz::fmt_f(mx, 7, 2),
            viz::fmt_f(mn, 7, 2),
            viz::fmt_f(mx - mn, 7, 2)
        });
    }
    viz::comparison_table("6-bus 节点电价统计", headers, rows);

    // 打印系统级结果
    std::cout << "\n系统统计: "
              << "平均LMP=" << viz::fmt_f(out.lmp.avg_lmp, 6, 2) << " $/MWh  "
              << "最大=" << viz::fmt_f(out.lmp.max_lmp, 6, 2) << "  "
              << "最小=" << viz::fmt_f(out.lmp.min_lmp, 6, 2) << "\n";

    // 物理一致性：LMP 范围合理（上限为 voll，切负荷时 LMP = voll）
    CHECK(out.lmp.avg_lmp >= 0.0);
    CHECK(out.lmp.avg_lmp <= inp.config.voll + 1.0);
    CHECK(out.lmp.min_lmp <= out.lmp.avg_lmp + 1e-9);
    CHECK(out.lmp.avg_lmp <= out.lmp.max_lmp + 1e-9);
    REQUIRE(static_cast<int>(out.lmp.nodal_lmp.size()) == nb);
    for (auto& row : out.lmp.nodal_lmp)
        REQUIRE(static_cast<int>(row.size()) == T);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 7: 储能 SOC 时序可视化 + 循环约束验证
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 6-bus 储能 SOC 时序图", "[market][viz]") {
    SCUCInput inp = build_6bus_case(24, 1.0, /*wind=*/false, /*storage=*/true);
    inp.config.solver    = "Auto";
    inp.config.solve_sced = false;
    inp.config.solve_lmp  = false;

    SCUCOutput out = scuc_solve(inp);
    REQUIRE(out.scuc.converged);
    if (out.scuc.storage_soc.empty()) {
        WARN("无储能结果");
        return;
    }

    const int ns = static_cast<int>(inp.storage.size());
    const int T  = inp.config.num_periods;

    // SOC 时序图
    std::vector<std::string> soc_names;
    for (auto& s : inp.storage) soc_names.push_back(s.name + " SOC");
    viz::time_series("储能 SOC 时序 (MWh)", soc_names, out.scuc.storage_soc, "MWh");

    // 充放电功率时序
    if (!out.scuc.storage_charging.empty()) {
        std::vector<std::string> pnames;
        std::vector<std::vector<double>> pseries;
        for (int s = 0; s < ns; ++s) {
            pnames.push_back(inp.storage[static_cast<size_t>(s)].name + " 充");
            pseries.push_back(out.scuc.storage_charging[static_cast<size_t>(s)]);
            pnames.push_back(inp.storage[static_cast<size_t>(s)].name + " 放");
            pseries.push_back(out.scuc.storage_discharging[static_cast<size_t>(s)]);
        }
        viz::time_series("储能充放电功率时序 (MW)", pnames, pseries, "MW");
    }

    // 逐时段 SOC 打印
    std::cout << "\n储能 SOC 逐时段 (MWh):\n";
    std::cout << std::setw(6) << "时段";
    for (auto& s : inp.storage)
        std::cout << std::setw(14) << s.name << "(SOC)" << std::setw(6) << "充MW" << std::setw(6) << "放MW";
    std::cout << "\n" << viz::repeat('-', 6 + ns * 26) << "\n";
    for (int t = 0; t < T; ++t) {
        std::cout << std::setw(6) << t;
        for (int s = 0; s < ns; ++s) {
            double soc = out.scuc.storage_soc[static_cast<size_t>(s)][static_cast<size_t>(t)];
            double pch = out.scuc.storage_charging.empty() ? 0.0
                         : out.scuc.storage_charging[static_cast<size_t>(s)][static_cast<size_t>(t)];
            double pdi = out.scuc.storage_discharging.empty() ? 0.0
                         : out.scuc.storage_discharging[static_cast<size_t>(s)][static_cast<size_t>(t)];
            std::cout << std::setw(14) << viz::fmt_f(soc, 8, 1)
                      << std::setw(8) << viz::fmt_f(pch, 6, 1)
                      << std::setw(8) << viz::fmt_f(pdi, 6, 1);
        }
        std::cout << "\n";
    }

    // SOC 物理约束检查
    for (int s = 0; s < ns; ++s) {
        double e_cap = inp.storage[static_cast<size_t>(s)].energy_capacity_mwh;
        double soc_min_frac = inp.storage[static_cast<size_t>(s)].soc_min < 0.0
                              ? 0.10 : inp.storage[static_cast<size_t>(s)].soc_min;
        double e_min = soc_min_frac * e_cap;
        for (int t = 0; t < T; ++t) {
            double soc = out.scuc.storage_soc[static_cast<size_t>(s)][static_cast<size_t>(t)];
            CHECK(soc >= e_min - 1.0);   // 允许 1 MWh 数值误差
            CHECK(soc <= e_cap + 1.0);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 8: 备用约束满足验证（含负备用、PFR）
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: 备用约束验证", "[market][physics]") {
    SCUCInput inp = build_3bus_case(6, 1.0);
    inp.config.solver              = "Auto";
    inp.config.spinning_reserve_req = 0.10;
    inp.config.regulation_up_req    = 0.05;
    inp.config.regulation_down_req  = 0.05;
    inp.config.neg_reserve_req      = 0.10;
    inp.config.pfr_reserve_req_mw   = 10.0;  // G1: 0.05 * pmax200 = 10 MW max
    // 设置 PFR 系数（G1 参与 PFR）
    inp.generators[0].pfr_alpha = 0.05;
    inp.config.solve_sced = false;
    inp.config.solve_lmp  = false;

    SCUCOutput out = scuc_solve(inp);
    REQUIRE(out.scuc.converged);

    const int T  = inp.config.num_periods;
    const int ng = static_cast<int>(inp.generators.size());
    const double dt = inp.config.period_length_hr;

    std::cout << "\n备用约束验证（T=" << T << " 时段）:\n";
    std::cout << std::setw(6) << "时段"
              << std::setw(12) << "负荷(MW)"
              << std::setw(14) << "旋转备用(MW)"
              << std::setw(14) << "调频上(MW)"
              << std::setw(14) << "调频下(MW)"
              << "\n" << viz::repeat('-', 60) << "\n";

    for (int t = 0; t < T; ++t) {
        double load_t = 0.0;
        for (size_t d = 0; d < inp.loads.size(); ++d) {
            double pu = (!inp.profiles.load.empty() && d < inp.profiles.load.size()
                         && t < static_cast<int>(inp.profiles.load[d].size()))
                        ? inp.profiles.load[d][static_cast<size_t>(t)] : 1.0;
            load_t += inp.loads[d].p_mw * pu;
        }
        double spin = 0.0, rup = 0.0, rdn = 0.0;
        for (int g = 0; g < ng; ++g) {
            if (!out.scuc.spinning_reserve.empty())
                spin += out.scuc.spinning_reserve[static_cast<size_t>(g)][static_cast<size_t>(t)];
            if (!out.scuc.regulation_up.empty())
                rup  += out.scuc.regulation_up[static_cast<size_t>(g)][static_cast<size_t>(t)];
            if (!out.scuc.regulation_down.empty())
                rdn  += out.scuc.regulation_down[static_cast<size_t>(g)][static_cast<size_t>(t)];
        }
        std::cout << std::setw(6) << t
                  << std::setw(12) << viz::fmt_f(load_t, 8, 1)
                  << std::setw(14) << viz::fmt_f(spin, 8, 1)
                  << std::setw(14) << viz::fmt_f(rup, 8, 1)
                  << std::setw(14) << viz::fmt_f(rdn, 8, 1)
                  << "\n";

        // 旋转备用 >= req * load
        if (!out.scuc.spinning_reserve.empty())
            CHECK(spin >= inp.config.spinning_reserve_req * load_t - 1.0);
    }
    (void)dt;
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 9: IEEE 39-bus 24h 完整流程 + 调试图表 + JSON 导出
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: IEEE 39-bus 24h 完整流程", "[market][viz][ieee39]") {
    SCUCInput inp = build_ieee39_case(24, 1.0, /*wind=*/true, /*solar=*/false);
    inp.config.solver          = "Auto";
    inp.config.mip_gap         = 0.01;
    inp.config.time_limit_sec  = 180.0;
    inp.config.solve_sced      = true;
    inp.config.solve_lmp       = true;

    const int T  = inp.config.num_periods;
    const int ng = static_cast<int>(inp.generators.size());
    const int nb = static_cast<int>(inp.num_buses);
    const int nl = static_cast<int>(inp.branches.size());

    // ── 导出输入 JSON ────────────────────────────────────────────────────────
    {
        std::string js = scuc_input_to_json(inp, 2);
        const fs::path input_path = debug_output_path("ieee39_scuc_input.json");
        std::ofstream f(input_path);
        if (f) { f << js; std::cout << "\n[调试] 输入已写入 " << input_path.string() << "\n"; }
    }

    // ── 求解前: 容量裕度检查 ──────────────────────────────────────────────────
    {
        double total_pmax = 0.0, total_pmin = 0.0;
        for (auto& g : inp.generators) { total_pmax += g.pmax; total_pmin += g.pmin; }
        // 初始在线容量
        double init_cap = 0.0;
        for (int g = 0; g < ng; ++g)
            if (!inp.initial_status.commitment.empty() &&
                inp.initial_status.commitment[static_cast<size_t>(g)] > 0.5)
                init_cap += inp.generators[static_cast<size_t>(g)].pmax;
        // 系统总负荷基准
        double base_load = 0.0;
        for (auto& l : inp.loads) base_load += l.p_mw;
        double peak_load   = base_load * 1.0;   // 负荷曲线峰值系数 = 1.0
        double valley_load = base_load * 0.65;  // 谷值系数 = 0.65

        std::cout << "\n╔══════════════════════════════════════════╗\n";
        std::cout <<   "║  IEEE 39-bus 算例 — 求解前容量摘要        ║\n";
        std::cout <<   "╠══════════════════════════════════════════╣\n";
        std::cout <<   "║  发电机: " << std::setw(3) << ng
                  << "  母线: " << std::setw(3) << nb
                  << "  支路: " << std::setw(3) << nl
                  << "  时段: " << std::setw(3) << T << "           ║\n";
        std::cout <<   "║  总Pmax:      " << std::setw(8) << viz::fmt_f(total_pmax,   8, 0) << " MW               ║\n";
        std::cout <<   "║  总Pmin:      " << std::setw(8) << viz::fmt_f(total_pmin,   8, 0) << " MW               ║\n";
        std::cout <<   "║  初始在线容量: " << std::setw(8) << viz::fmt_f(init_cap,    8, 0) << " MW               ║\n";
        std::cout <<   "║  峰值负荷:    " << std::setw(8) << viz::fmt_f(peak_load,   8, 0) << " MW               ║\n";
        std::cout <<   "║  谷值负荷:    " << std::setw(8) << viz::fmt_f(valley_load, 8, 0) << " MW               ║\n";
        double margin = (total_pmax - peak_load) / peak_load * 100.0;
        std::cout <<   "║  备用裕度:    " << std::setw(8) << viz::fmt_f(margin,       7, 1) << " %                ║\n";
        // 初始缺额 (仅在线机组 vs 谷负荷)
        double init_deficit = valley_load - init_cap;
        if (init_deficit > 0.0)
            std::cout << "║  ⚠ 初始在线容量不足谷负荷: " << viz::fmt_f(init_deficit, 7, 0) << " MW 缺口  ║\n";
        std::cout <<   "╚══════════════════════════════════════════╝\n";

        // 逐台机初始状态表
        std::cout << "\n初始机组状态:\n";
        std::cout << std::setw(8) << "机组" << std::setw(8) << "母线"
                  << std::setw(8) << "Pmin" << std::setw(8) << "Pmax"
                  << std::setw(8) << "初状态" << std::setw(10) << "初出力MW" << "\n"
                  << viz::repeat('-', 52) << "\n";
        for (int g = 0; g < ng; ++g) {
            auto& gn = inp.generators[static_cast<size_t>(g)];
            int   cm = (!inp.initial_status.commitment.empty())
                       ? static_cast<int>(inp.initial_status.commitment[static_cast<size_t>(g)] + 0.5) : 0;
            double dp = (!inp.initial_status.dispatch.empty())
                        ? inp.initial_status.dispatch[static_cast<size_t>(g)] : 0.0;
            std::cout << std::setw(8) << (gn.name.empty() ? "G" + std::to_string(g+1) : gn.name)
                      << std::setw(8) << gn.bus
                      << std::setw(8) << viz::fmt_f(gn.pmin, 6, 0)
                      << std::setw(8) << viz::fmt_f(gn.pmax, 6, 0)
                      << std::setw(8) << (cm ? "在线" : "离线")
                      << std::setw(10) << viz::fmt_f(dp, 7, 1) << "\n";
        }
    }

    // ── 求解 ──────────────────────────────────────────────────────────────────
    WallTimer timer;
    SCUCOutput out = scuc_solve(inp);
    double wall_total = timer.elapsed_sec();

    // ── 导出输出 JSON ────────────────────────────────────────────────────────
    {
        std::string js = scuc_output_to_json(out, inp, 2);
        const fs::path output_path = debug_output_path("ieee39_scuc_output.json");
        std::ofstream f(output_path);
        if (f) { f << js; std::cout << "[调试] 输出已写入 " << output_path.string() << "\n"; }
    }

    // ── 求解耗时 ──────────────────────────────────────────────────────────────
    {
        std::vector<std::string> hdr = {"阶段", "耗时(s)", "状态", "目标值($)", "MIP间隙"};
        std::vector<std::vector<std::string>> rows;
        rows.push_back({"SCUC",
            viz::fmt_f(out.scuc.solve_time_sec, 7, 2),
            out.scuc.converged ? "收敛" : "未收敛/超时",
            out.scuc.converged ? viz::fmt_f(out.scuc.objective, 12, 0) : "N/A",
            out.scuc.converged ? viz::fmt_f(out.scuc.mip_gap * 100.0, 5, 3) + "%" : "N/A"});
        if (out.sced.converged || out.sced.solve_time_sec > 0.0)
            rows.push_back({"SCED",
                viz::fmt_f(out.sced.solve_time_sec, 7, 2),
                out.sced.converged ? "收敛" : "未收敛",
                out.sced.converged ? viz::fmt_f(out.sced.objective, 12, 0) : "N/A", "-"});
        rows.push_back({"墙上总计", viz::fmt_f(wall_total, 7, 2), "-", "-", "-"});
        viz::comparison_table("IEEE 39-bus 求解耗时分解", hdr, rows);
        std::cout << "  求解器: " << out.scuc.solver_name << "  割平面: " << out.scuc.n_cuts_added << "\n";
    }

    REQUIRE(out.scuc.converged);

    const int nw = static_cast<int>(inp.wind.size());

    // ── 机组组合 Gantt 图 ────────────────────────────────────────────────────
    std::vector<std::string> gen_names;
    for (auto& g : inp.generators)
        gen_names.push_back(g.name.empty() ? "G?" : g.name);
    viz::gantt_chart("IEEE 39-bus 24h 机组组合", gen_names, out.scuc.commitment, T);

    // ── 系统级出力时序 ────────────────────────────────────────────────────────
    std::vector<double> total_thermal(static_cast<size_t>(T), 0.0);
    std::vector<double> total_wind(static_cast<size_t>(T),    0.0);
    std::vector<double> total_shedding(static_cast<size_t>(T), 0.0);
    std::vector<double> load_profile(static_cast<size_t>(T),  0.0);

    for (int t = 0; t < T; ++t) {
        for (int g = 0; g < ng; ++g)
            if (!out.scuc.dispatch.empty())
                total_thermal[static_cast<size_t>(t)] +=
                    out.scuc.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
        for (int w = 0; w < nw; ++w)
            if (!out.scuc.wind_generation.empty())
                total_wind[static_cast<size_t>(t)] +=
                    out.scuc.wind_generation[static_cast<size_t>(w)][static_cast<size_t>(t)];
        if (!out.scuc.load_shedding.empty())
            total_shedding[static_cast<size_t>(t)] =
                out.scuc.load_shedding[static_cast<size_t>(t)];
        for (size_t d = 0; d < inp.loads.size(); ++d) {
            double pu = (!inp.profiles.load.empty() && d < inp.profiles.load.size()
                         && t < static_cast<int>(inp.profiles.load[d].size()))
                        ? inp.profiles.load[d][static_cast<size_t>(t)] : 1.0;
            load_profile[static_cast<size_t>(t)] += inp.loads[d].p_mw * pu;
        }
    }

    viz::time_series("IEEE 39-bus 24h 出力时序",
                     {"火电总出力", "风电总出力", "系统负荷"},
                     {total_thermal, total_wind, load_profile}, "MW");

    // ── 切负荷分析 ────────────────────────────────────────────────────────────
    {
        double total_shed = std::accumulate(total_shedding.begin(), total_shedding.end(), 0.0);
        std::cout << "\n切负荷统计: 总计 " << viz::fmt_f(total_shed, 8, 1)
                  << " MWh, 峰值 "
                  << viz::fmt_f(*std::max_element(total_shedding.begin(), total_shedding.end()), 8, 1)
                  << " MW\n";
        if (total_shed > 1.0) {
            // 切负荷逐时段条形图
            viz::bar_chart("各时段切负荷量 (MW)", [&]{
                std::vector<std::string> labels;
                for (int t = 0; t < T; ++t) labels.push_back("t=" + std::to_string(t));
                return labels;
            }(), total_shedding, "MW", 36);
        }
    }

    // ── 逐时段功率平衡诊断表 ─────────────────────────────────────────────────
    {
        std::cout << "\n逐时段功率平衡（火电+风电+切负荷应≈负荷）:\n";
        std::cout << std::setw(5)  << "时段"
                  << std::setw(10) << "负荷MW"
                  << std::setw(10) << "火电MW"
                  << std::setw(10) << "风电MW"
                  << std::setw(10) << "切负荷MW"
                  << std::setw(10) << "误差MW" << "\n"
                  << viz::repeat('-', 55) << "\n";
        for (int t = 0; t < T; ++t) {
            double gen = total_thermal[static_cast<size_t>(t)]
                       + total_wind[static_cast<size_t>(t)];
            double ls  = total_shedding[static_cast<size_t>(t)];
            double lod = load_profile[static_cast<size_t>(t)];
            double err = gen + ls - lod;
            std::cout << std::setw(5)  << t
                      << std::setw(10) << viz::fmt_f(lod, 8, 1)
                      << std::setw(10) << viz::fmt_f(total_thermal[static_cast<size_t>(t)], 8, 1)
                      << std::setw(10) << viz::fmt_f(total_wind[static_cast<size_t>(t)], 8, 1)
                      << std::setw(10) << viz::fmt_f(ls, 8, 1)
                      << std::setw(10) << viz::fmt_f(err, 8, 3)
                      << (std::abs(err) > 5.0 ? "  ⚠" : "") << "\n";
        }
    }

    // ── 网络拥塞分析 ──────────────────────────────────────────────────────────
    if (!out.scuc.line_flows.empty()) {
        struct CongEvent { int line, t; double flow, rating, util; };
        std::vector<CongEvent> events;
        for (int l = 0; l < nl && l < static_cast<int>(out.scuc.line_flows.size()); ++l) {
            double rating = inp.branches[static_cast<size_t>(l)].rating_mw;
            for (int t = 0; t < T; ++t) {
                double flow = std::abs(out.scuc.line_flows[static_cast<size_t>(l)][static_cast<size_t>(t)]);
                double util = (rating > 0.0) ? flow / rating : 0.0;
                if (util > 0.85) events.push_back({l, t, flow, rating, util});
            }
        }
        if (!events.empty()) {
            std::sort(events.begin(), events.end(),
                      [](const CongEvent& a, const CongEvent& b){ return a.util > b.util; });
            std::cout << "\n高负载支路 (利用率 > 85%):\n";
            std::vector<std::string> hdr = {"支路", "从-到", "时段", "潮流MW", "额定MW", "利用率%"};
            std::vector<std::vector<std::string>> rows;
            int shown = 0;
            for (auto& ev : events) {
                if (shown++ > 15) break;  // 最多显示前16条
                int f = inp.branches[static_cast<size_t>(ev.line)].from;
                int t = inp.branches[static_cast<size_t>(ev.line)].to;
                rows.push_back({std::to_string(ev.line),
                    std::to_string(f) + "-" + std::to_string(t),
                    std::to_string(ev.t),
                    viz::fmt_f(ev.flow, 7, 1),
                    viz::fmt_f(ev.rating, 7, 0),
                    viz::fmt_f(ev.util * 100.0, 5, 1)});
            }
            viz::comparison_table("网络拥塞事件 (前" + std::to_string(std::min(shown, 16)) + "条)", hdr, rows);
        } else {
            std::cout << "\n无支路利用率 > 85% — 网络畅通\n";
        }
    }

    // ── LMP 热图 ─────────────────────────────────────────────────────────────
    if (out.lmp.converged)
        viz::lmp_heatmap("IEEE 39-bus 24h 节点电价热图", out.lmp.nodal_lmp, 10, 24);

    // ── 费用分解 ──────────────────────────────────────────────────────────────
    {
        std::vector<std::string> hdr = {"费用项目", "金额($)", "占比(%)"};
        std::vector<std::vector<std::string>> rows;
        auto add = [&](const std::string& n, double v) {
            rows.push_back({n, viz::fmt_f(v, 12, 0),
                viz::fmt_f(out.scuc.total_cost > 0 ? v / out.scuc.total_cost * 100.0 : 0.0, 5, 1)});
        };
        add("能量费用", out.scuc.energy_cost);
        add("启动费用", out.scuc.startup_cost);
        add("空载费用", out.scuc.no_load_cost);
        add("备用费用", out.scuc.reserve_cost);
        add("惩罚费用", out.scuc.penalty_cost);
        add("合计",     out.scuc.total_cost);
        viz::comparison_table("IEEE 39-bus 运营费用分解", hdr, rows);
    }

    // ── 物理验证 ──────────────────────────────────────────────────────────────
    // 切负荷：拓扑修复后应接近零；允许轻度网络拥塞导致的少量切负荷（< 50 MW/period）
    double total_pmax = 0.0;
    for (auto& g : inp.generators) total_pmax += g.pmax;
    double max_shed = *std::max_element(total_shedding.begin(), total_shedding.end());
    if (max_shed > 50.0)
        WARN("峰值切负荷 " << max_shed << " MW 超过 50 MW — 可能存在网络拥塞或容量不足");
    for (int t = 0; t < T; ++t)
        CHECK(total_shedding[static_cast<size_t>(t)] < total_pmax);
    // 功率平衡：误差 < 5 MW
    for (int t = 0; t < T; ++t) {
        double gc = (!out.scuc.gen_curtailment.empty())
                    ? out.scuc.gen_curtailment[static_cast<size_t>(t)] : 0.0;
        double balance = total_thermal[static_cast<size_t>(t)]
                       + total_wind[static_cast<size_t>(t)]
                       + total_shedding[static_cast<size_t>(t)]
                       - gc
                       - load_profile[static_cast<size_t>(t)];
        CHECK(std::abs(balance) < 5.0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 10: MIP Gap 对求解速度影响分析（3-bus, 默认 Auto 求解器）
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Market: MIP Gap 收敛速度分析", "[market][benchmark]") {
    const std::vector<double> gaps = {0.10, 0.05, 0.01, 0.005, 0.001};
    SCUCInput base = build_3bus_case(24, 1.0);
    base.config.solver        = "Auto";
    base.config.allow_fallback = true;
    base.config.solve_sced    = false;
    base.config.solve_lmp     = false;

    std::vector<std::string>              labels;
    std::vector<double>                   times;
    std::vector<std::vector<std::string>> rows;

    for (double gap : gaps) {
        SCUCInput inp = base;
        inp.config.mip_gap = gap;

        WallTimer timer;
        SCUCOutput out = scuc_solve(inp);
        double elapsed = timer.elapsed_sec();

        std::string gap_str = viz::fmt_f(gap * 100.0, 4, 1) + "%";
        labels.push_back(gap_str);
        times.push_back(out.scuc.converged ? elapsed : 0.0);
        rows.push_back({
            gap_str,
            out.scuc.converged ? "YES" : "NO",
            viz::fmt_f(elapsed, 7, 3),
            out.scuc.converged ? viz::fmt_f(out.scuc.objective, 12, 1) : "N/A",
            out.scuc.converged ? viz::fmt_f(out.scuc.mip_gap * 100.0, 6, 3) + "%" : "N/A"
        });
    }

    viz::bar_chart("MIP Gap vs 耗时（3-bus T=24, Auto）", labels, times, "s", 36);
    viz::comparison_table("MIP Gap 收敛分析", {"间隙要求", "收敛", "耗时(s)", "目标值($)", "实际间隙"}, rows);

    // 验证最严间隙下也应收敛
    SCUCInput strict = base;
    strict.config.mip_gap = 0.001;
    CHECK(scuc_solve(strict).scuc.converged);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST: IEEE 39-bus 24h — StrictHiGHS root IPM + crossover
//
// Builds the 39-bus 24T SCUC MIP directly, then solves it via StrictHiGHS with
// an explicit root-node IPM + crossover policy.  Verifies:
//   (1) Crossover runs at least one iteration (basis was produced).
//   (2) StrictHiGHS returns a finite, feasible objective (no bound-zero failure).
//   (3) The IPM-root result agrees with a plain simplex-root solve within 0.5%.
// ─────────────────────────────────────────────────────────────────────────────
#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"

TEST_CASE("StrictHiGHS IEEE 39-bus 24h: root IPM + crossover correctness",
          "[market][strict_highs][ipm_root][ieee39]") {
    using namespace mipsolvers::engine;

    SCUCInput inp = build_ieee39_case(24, 1.0, /*wind=*/true, /*solar=*/true);
    inp.config.solve_sced = false;
    inp.config.solve_lmp  = false;

    MIPModel mip = build_scuc_mip(inp);
    const int n_cols = static_cast<int>(mip.linear_part.vars.size());
    const int n_rows = static_cast<int>(mip.linear_part.A.rows() +
                                        mip.linear_part.Aeq.rows());

    // ── Simplex-root baseline ─────────────────────────────────────────────
    BCOptions simplex_opt;
    simplex_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
    simplex_opt.auto_highs_root_pipeline     = true;
    simplex_opt.highs_mip_lp_solver          = "choose";  // simplex at root
    simplex_opt.gap_tol                      = 1e-3;
    simplex_opt.time_limit_sec               = 120.0;

    BCResult simplex_res = solve_milp_bc(mip, simplex_opt);
    REQUIRE(simplex_res.stats.success);
    const double simplex_obj = simplex_res.stats.objective;

    // ── IPM-root + crossover ──────────────────────────────────────────────
    BCOptions ipm_opt;
    ipm_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
    ipm_opt.auto_highs_root_pipeline     = true;
    ipm_opt.highs_mip_lp_solver          = "ipm";
    ipm_opt.highs_mip_root_crossover     = "on";
    ipm_opt.gap_tol                      = 1e-3;
    ipm_opt.time_limit_sec               = 120.0;

    BCResult ipm_res = solve_milp_bc(mip, ipm_opt);

    // Report problem size and solve outcomes for visibility.
    std::cout << "\n[IPM-ROOT-TEST] cols=" << n_cols << " rows=" << n_rows
              << " simplex_obj=" << simplex_obj
              << " ipm_obj=" << ipm_res.stats.objective
              << " ipm_success=" << ipm_res.stats.success
              << " ipm_status=" << ipm_res.stats.status << "\n";

    // (1) Must succeed — IPM+crossover must give a feasible solve.
    REQUIRE(ipm_res.stats.success);

    // (2) Objective must be finite and positive.
    CHECK(std::isfinite(ipm_res.stats.objective));
    CHECK(ipm_res.stats.objective > 0.0);

    // (3) IPM-root result must be within 0.5% of simplex baseline.
    const double rel_diff = std::abs(ipm_res.stats.objective - simplex_obj) /
                            std::max(1.0, std::abs(simplex_obj));
    CHECK(rel_diff < 0.005);
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST: IEEE 39-bus 24h — pure IPM (root + all nodes, no crossover)
//
// Sets mip_lp_solver="ipm" + run_crossover="off".  Since no basis is ever
// produced, HiGHS falls through to the IPM solver path for EVERY node LP, not
// just the root.  This test documents the performance cost: without warm-start
// each child node LP starts cold, so the number of B&C nodes solvable within
// the budget collapses.
//
// Expected observations vs IPM+crossover:
//   - Root LP time ≈ same (same IPM, no crossover overhead)
//   - B&C node LP solves: each starts cold → dramatically fewer nodes explored
//   - Objective gap: worse or similar (fewer nodes → weaker bound)
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("StrictHiGHS IEEE 39-bus 24h: pure IPM no-crossover (root+nodes)",
          "[market][strict_highs][ipm_root][ieee39][pure_ipm]") {
    using namespace mipsolvers::engine;

    SCUCInput inp = build_ieee39_case(24, 1.0, /*wind=*/true, /*solar=*/true);
    inp.config.solve_sced = false;
    inp.config.solve_lmp  = false;

    MIPModel mip = build_scuc_mip(inp);
    const int n_cols = static_cast<int>(mip.linear_part.vars.size());
    const int n_rows = static_cast<int>(mip.linear_part.A.rows() +
                                        mip.linear_part.Aeq.rows());

    // ── IPM root + crossover (baseline) ───────────────────────────────────
    BCOptions xover_opt;
    xover_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
    xover_opt.auto_highs_root_pipeline     = true;
    xover_opt.highs_mip_lp_solver          = "ipm";
    xover_opt.highs_mip_root_crossover     = "on";   // crossover → basis
    xover_opt.gap_tol                      = 1e-3;
    xover_opt.time_limit_sec               = 120.0;
    const auto t_xover0 = std::chrono::steady_clock::now();
    BCResult xover_res = solve_milp_bc(mip, xover_opt);
    const double t_xover = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_xover0).count();

    // ── Pure IPM: no crossover → IPM for every node LP ────────────────────
    BCOptions pure_ipm_opt;
    pure_ipm_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
    pure_ipm_opt.auto_highs_root_pipeline     = true;
    pure_ipm_opt.highs_mip_lp_solver          = "ipm";
    pure_ipm_opt.highs_mip_root_crossover     = "off";  // no basis → cold IPM at every node
    pure_ipm_opt.gap_tol                      = 1e-3;
    pure_ipm_opt.time_limit_sec               = 120.0;
    const auto t_pure0 = std::chrono::steady_clock::now();
    BCResult pure_res = solve_milp_bc(mip, pure_ipm_opt);
    const double t_pure = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_pure0).count();

    std::cout << "\n[PURE-IPM-TEST] cols=" << n_cols << " rows=" << n_rows
              << "\n  ipm+xover  : obj=" << xover_res.stats.objective
              << " success=" << xover_res.stats.success
              << " wall=" << t_xover << "s"
              << "\n  pure-ipm   : obj=" << pure_res.stats.objective
              << " success=" << pure_res.stats.success
              << " wall=" << t_pure << "s"
              << "\n  (pure IPM eliminates warm-starting; each child node LP"
                 " costs a full IPM solve instead of O(1) warm-start pivots)\n";

    // Both must produce a finite objective — pure IPM should converge on this
    // small problem even without warm-starting.
    REQUIRE(xover_res.stats.success);
    REQUIRE(pure_res.stats.success);
    CHECK(std::isfinite(pure_res.stats.objective));
    CHECK(pure_res.stats.objective > 0.0);

    // Objectives must agree within 0.5%: both solve the same MIP.
    const double rel_diff =
        std::abs(pure_res.stats.objective - xover_res.stats.objective) /
        std::max(1.0, std::abs(xover_res.stats.objective));
    CHECK(rel_diff < 0.005);
}
