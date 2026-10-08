// test/test_perf_report_c28.cpp
// C2.8: formal 125 Hz performance report.
//   - five controller configurations (relative / dual-task / +CBF / +nullspace
//     / all-on), 20000 cycles each with a slowly moving state
//   - solver time AND full-cycle (assembly + optional collision + solve) time
//   - sub-module micro benchmarks (kinematics, collision)
//   Budget at 125 Hz: 8 ms.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

#ifdef DUAL_ARM_HAVE_COLLISION
#include "dual_arm_collision/dual_arm_collision.hpp"
#endif

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;

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

struct Stats {
    double mean = 0, median = 0, p95 = 0, p99 = 0, max = 0;
};

Stats statsOf(std::vector<double>& v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / v.size();
    auto pct = [&](double p) {
        return v[std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1)))];
    };
    s.median = pct(0.50);
    s.p95 = pct(0.95);
    s.p99 = pct(0.99);
    s.max = v.back();
    return s;
}

struct ConfigResult {
    std::string name;
    Stats solver, cycle;
    double iter_mean = 0;
    int iter_max = 0;
    int failures = 0;
    int cycles = 0;
};

#ifdef DUAL_ARM_HAVE_COLLISION
using dual_arm_collision::DualArmCollisionModel;
#endif

}  // namespace

TEST(C28Perf, FiveConfigurations) {
    DualArmKinematics kin;
    std::mt19937 rng(20261020u);
    const Q12 q_home = DualArmKinematics::stack(randomQ(rng, 0.4), randomQ(rng, 0.4));
    const Iso3 rel0 = kin.leftTright(q_home);
    const Iso3 obj0 = kin.worldTobject(q_home);

    constexpr int kN = 20000;
    std::vector<ConfigResult> results;

    auto bench = [&](const std::string& name, DualArmQpParams params, bool cbf) {
        DualArmQp qp(kin, params);
#ifdef DUAL_ARM_HAVE_COLLISION
        std::unique_ptr<DualArmCollisionModel> model;
        if (cbf) model = std::make_unique<DualArmCollisionModel>(kin, 0.06);
#else
        (void)cbf;
#endif
        std::vector<double> solver_ms, cycle_ms;
        solver_ms.reserve(kN);
        cycle_ms.reserve(kN);
        ConfigResult cr;
        cr.name = name;
        long long iter_sum = 0;
        for (int k = 0; k < kN; ++k) {
            Q12 q = q_home;
            const double t = k * params.dt;
            q[1] += 0.02 * std::sin(t);
            q[7] -= 0.02 * std::sin(1.3 * t);
            Iso3 target_obj = obj0;
            target_obj.translation().x() += 0.01 * std::sin(0.7 * t);
#ifdef DUAL_ARM_HAVE_COLLISION
            if (model != nullptr) {
                const auto info = model->distance(q);
                const dual_arm_collision::Vec12 g = model->distanceGradient(q);
                params.C_ineq = Eigen::MatrixXd::Zero(1, 24);
                for (int i = 0; i < 12; ++i) params.C_ineq(0, i) = -g[i];
                params.d_ineq =
                    Eigen::VectorXd::Constant(1, 3.0 * (info.distance - 0.05) * params.dt);
                qp.setParams(params);
            }
#endif
            const auto t0 = std::chrono::steady_clock::now();
            const auto r = qp.solve(q, rel0, target_obj);
            const auto t1 = std::chrono::steady_clock::now();
            solver_ms.push_back(r.solve_time_ms);
            cycle_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            iter_sum += r.iterations;
            cr.iter_max = std::max(cr.iter_max, r.iterations);
            if (!r.converged) ++cr.failures;
        }
        cr.solver = statsOf(solver_ms);
        cr.cycle = statsOf(cycle_ms);
        cr.iter_mean = static_cast<double>(iter_sum) / kN;
        cr.cycles = kN;
        results.push_back(cr);
    };

    auto base = [&]() {
        DualArmQpParams p;
        p.dt = 0.008;
        p.kp_rel = 20.0;
        p.kp_obj = 20.0;
        p.w_rel = 1.0;
        p.w_slack = 1e3;
        p.w_center = 1e-2;
        p.w_reg = 1e-6;
        return p;
    };

    DualArmQpParams pA = base(); pA.w_obj = 0.0;
    DualArmQpParams pB = base(); pB.w_obj = 1.0; pB.w_slack_obj = 1e3;
    DualArmQpParams pC = pB;
    DualArmQpParams pD = base(); pD.w_obj = 0.0;
    pD.w_null_margin = pD.w_null_manip = pD.w_null_clear = 1.0;
    DualArmQpParams pE = pB;
    pE.w_null_margin = pE.w_null_manip = pE.w_null_clear = 1.0;
    pE.enable_singularity = true;

    bench("A relative only", pA, false);
    bench("B dual task", pB, false);
#ifdef DUAL_ARM_HAVE_COLLISION
    bench("C dual task + CBF", pC, true);
#else
    bench("C dual task + CBF (SKIPPED: no collision pkg)", pC, false);
#endif
    bench("D relative + nullspace", pD, false);
    bench("E all on (dual+null+sing+CBF)", pE, true);

    std::printf("\n[C2.8] %d cycles per config, dt = 8 ms (125 Hz), budget = 8000 us\n", kN);
    std::printf("[C2.8] %-34s | %-38s | %-38s | iters | fail\n", "config",
                "solver ms mean/med/P95/P99/max", "cycle  ms mean/med/P95/P99/max");
    for (const auto& r : results) {
        std::printf("[C2.8] %-34s | %6.3f %6.3f %6.3f %6.3f %7.3f | %6.3f %6.3f %6.3f %6.3f %7.3f | %5.1f | %d\n",
                    r.name.c_str(), r.solver.mean, r.solver.median, r.solver.p95, r.solver.p99,
                    r.solver.max, r.cycle.mean, r.cycle.median, r.cycle.p95, r.cycle.p99,
                    r.cycle.max, r.iter_mean, r.failures);
    }
    std::printf("[C2.8] budget usage (cycle P99 / 8 ms):\n");
    for (const auto& r : results) {
        std::printf("[C2.8]   %-34s %6.1f us = %5.2f%%\n", r.name.c_str(), r.cycle.p99 * 1000.0,
                    r.cycle.p99 / 8.0 * 100.0);
    }

    for (const auto& r : results) {
        EXPECT_EQ(r.failures, 0);
        EXPECT_LT(r.cycle.p99, 8.0);  // P99 must fit the 125 Hz budget
    }
    EXPECT_GE(results.size(), 4u);
}

TEST(C28Perf, SubModuleMicroBenchmarks) {
    DualArmKinematics kin;
    std::mt19937 rng(20261021u);
    const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.4), randomQ(rng, 0.4));
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);

    auto timeit = [](const char* name, int n, const std::function<double()>& f) {
        auto t0 = std::chrono::steady_clock::now();
        double sink = 0.0;
        for (int i = 0; i < n; ++i) sink += f();
        auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / n;
        std::printf("[C2.8] %-34s %8.3f us/call (sink=%.3f)\n", name, us, sink);
    };

    timeit("kin : worldTgrasp", 200000, [&] { return kin.worldTgrasp(dual_arm::Arm::Left, ql).matrix().norm(); });
    timeit("kin : relativeJacobian(6x12)", 200000, [&] {
        return kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR).norm();
    });
    timeit("kin : objectJacobianBody(6x12)", 200000, [&] { return kin.objectJacobianBody(q).norm(); });
    timeit("kin : relativeError", 200000, [&] {
        return kin.relativeError(q, kin.leftTright(q)).norm();
    });
#ifdef DUAL_ARM_HAVE_COLLISION
    {
        dual_arm_collision::DualArmCollisionModel model(kin, 0.06);
        timeit("collision : distance(36 pairs)", 20000, [&] { return model.distance(q).distance; });
        timeit("collision : distanceGradient", 20000, [&] { return model.distanceGradient(q).norm(); });
    }
#endif
}
