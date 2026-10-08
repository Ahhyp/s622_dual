// test/test_qp_benchmark.cpp
// C2.R1 + C2.R4:
//   - the OSQP workspace must be PERSISTENT: osqp_setup() exactly once
//   - 100k-cycle statistics: mean / median / P95 / P99 / max, iterations, failures
//
// The problem is assembled by DualArmQp every cycle (same structure, moving
// state), i.e. the full control-path pipeline, not just a frozen QP.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_qp_core/osqp_solver.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;
using dual_arm_qp::OsqpSolver;

const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng, double shrink) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        const double mid = 0.5 * (kLower[i] + kUpper[i]);
        const double half = 0.5 * (kUpper[i] - kLower[i]) * shrink;
        q[i] = mid + (2.0 * unit(rng) - 1.0) * half;
    }
    return q;
}

double percentile(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t idx = std::min(v.size() - 1,
                                static_cast<size_t>(p * static_cast<double>(v.size() - 1)));
    return v[idx];
}

}  // namespace

TEST(QpBenchmark, PersistentWorkspace100k) {
    DualArmKinematics kin;
    DualArmQpParams params;
    params.dt = 0.008;
    params.kp_rel = 10.0;
    params.w_slack = 1e3;

    auto solver = std::make_shared<OsqpSolver>();
    DualArmQp qp(kin, params, solver);

    std::mt19937 rng(20261008u);
    const Q12 q_home = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
    const dual_arm::Iso3 target = kin.leftTright(q_home);

    constexpr int kN = 100000;
    std::vector<double> solver_ms, loop_ms;
    solver_ms.reserve(kN);
    loop_ms.reserve(kN);
    int failures = 0;
    long long iter_sum = 0;
    int iter_max = 0;

    // warm-up: performs the ONE-TIME osqp_setup() (excluded from the stats)
    qp.solve(q_home, target);

    auto loop_start = std::chrono::steady_clock::now();
    for (int k = 0; k < kN; ++k) {
        // slowly moving state -> the problem changes every cycle
        Q12 q = q_home;
        const double t = k * 0.008;
        q[1] += 0.02 * std::sin(t);
        q[7] -= 0.02 * std::sin(1.3 * t);
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = qp.solve(q, target);
        const auto t1 = std::chrono::steady_clock::now();
        solver_ms.push_back(r.solve_time_ms);
        loop_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        iter_sum += r.iterations;
        iter_max = std::max(iter_max, r.iterations);
        if (!r.converged) ++failures;
    }
    const double total_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count();

    const double mean = std::accumulate(solver_ms.begin(), solver_ms.end(), 0.0) / kN;
    const double med = percentile(solver_ms, 0.50);
    const double p95 = percentile(solver_ms, 0.95);
    const double p99 = percentile(solver_ms, 0.99);
    const double mx = *std::max_element(solver_ms.begin(), solver_ms.end());
    const double loop_p99 = percentile(loop_ms, 0.99);

    std::printf("[C2.R4] %d cycles in %.1fs (%.0f Hz)\n", kN, total_s, kN / total_s);
    std::printf("[C2.R4] solver ms: mean=%.4f median=%.4f P95=%.4f P99=%.4f max=%.4f\n", mean,
                med, p95, p99, mx);
    std::printf("[C2.R4] full pipeline (assemble+solve) P99=%.4f ms\n", loop_p99);
    std::printf("[C2.R4] iters: mean=%.1f max=%d | failures=%d | osqp_setup calls=%d\n",
                static_cast<double>(iter_sum) / kN, iter_max, failures, solver->setupCount());

    EXPECT_EQ(failures, 0);
    EXPECT_EQ(solver->setupCount(), 1) << "workspace was rebuilt in the control path!";
    EXPECT_LT(p99, 0.5);
    EXPECT_LT(loop_p99, 0.5);
}
