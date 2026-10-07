// test/test_solver_cross_check.cpp
// ADMM reference backend vs OSQP: same problems, same answers.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cstdio>
#include <limits>
#include <random>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/admm_solver.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_qp_core/osqp_solver.hpp"

namespace {

using dual_arm_qp::AdmmSolver;
using dual_arm_qp::OsqpSolver;
using dual_arm_qp::QpProblem;

constexpr double kInf = std::numeric_limits<double>::infinity();

QpProblem boxProblem(int n, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) b[i] = 3.0 * gauss(rng);
    QpProblem p;
    p.P = 2.0 * Eigen::MatrixXd::Identity(n, n);
    p.q = -2.0 * b;
    p.A = Eigen::MatrixXd::Identity(n, n);
    p.l = Eigen::VectorXd::Constant(n, -0.7);
    p.u = Eigen::VectorXd::Constant(n, 0.9);
    return p;
}

QpProblem inequalityProblem(int n, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    QpProblem p;
    p.P = 2.0 * Eigen::MatrixXd::Identity(n, n);
    p.q = Eigen::VectorXd::Zero(n);
    p.A.resize(2, n);
    for (int j = 0; j < n; ++j) p.A(0, j) = gauss(rng);
    p.A.row(1).setOnes();
    p.l = Eigen::VectorXd::Constant(2, -kInf);
    p.u = Eigen::VectorXd(2);
    p.u << 0.5, 1.0;
    return p;
}

}  // namespace

TEST(SolverCrossCheck, BoxAndInequality) {
    AdmmSolver admm;
    OsqpSolver osqp;
    double max_diff = 0.0;
    for (unsigned seed = 1; seed <= 20; ++seed) {
        for (int n : {4, 12}) {
            const QpProblem pb = boxProblem(n, seed);
            const auto xa = admm.solve(pb);
            const auto xo = osqp.solve(pb);
            ASSERT_TRUE(xa.converged);
            ASSERT_TRUE(xo.converged);
            max_diff = std::max(max_diff, (xa.x - xo.x).cwiseAbs().maxCoeff());

            const QpProblem pi = inequalityProblem(n, seed);
            const auto ya = admm.solve(pi);
            const auto yo = osqp.solve(pi);
            ASSERT_TRUE(ya.converged);
            ASSERT_TRUE(yo.converged);
            max_diff = std::max(max_diff, (ya.x - yo.x).cwiseAbs().maxCoeff());
        }
    }
    std::printf("[XP] ADMM vs OSQP (box + inequality): max |dx| = %.3e\n", max_diff);
    EXPECT_LT(max_diff, 1e-4);
}

TEST(SolverCrossCheck, DualArmRegulationBothBackends) {
    using dual_arm::DualArmKinematics;
    using dual_arm::Q12;
    using dual_arm_qp::DualArmQp;
    using dual_arm_qp::DualArmQpParams;

    DualArmKinematics kin;
    DualArmQpParams params;
    params.dt = 0.008;

    const double lo[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
    const double hi[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};
    std::mt19937 rng(2026u);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Q12 q0;
    for (int a = 0; a < 2; ++a)
        for (int i = 0; i < 6; ++i)
            q0[a * 6 + i] = lo[i] + unit(rng) * (hi[i] - lo[i]);

    const dual_arm::Iso3 target = kin.leftTright(q0);
    Q12 q_pert = q0;
    std::normal_distribution<double> gauss(0.0, 0.02);
    for (int i = 0; i < 12; ++i) q_pert[i] += gauss(rng);

    auto run = [&](dual_arm_qp::QpSolverPtr solver, Q12 q, double* first, double* last,
                   double* ms) {
        DualArmQp qp(kin, params, solver);
        *first = kin.relativeError(q, target).norm();
        double total = 0.0;
        for (int k = 0; k < 400; ++k) {
            const auto r = qp.solve(q, target);
            total += r.solve_time_ms;
            q += r.dq;
        }
        *last = kin.relativeError(q, target).norm();
        *ms = total / 400.0;
    };

    double a_first, a_last, a_ms, o_first, o_last, o_ms;
    run(std::make_shared<AdmmSolver>(), q_pert, &a_first, &a_last, &a_ms);
    run(std::make_shared<OsqpSolver>(), q_pert, &o_first, &o_last, &o_ms);

    std::printf("[XP] dual-arm regulation: ADMM %.3e -> %.3e (%.3f ms) | OSQP %.3e -> %.3e (%.3f ms)\n",
                a_first, a_last, a_ms, o_first, o_last, o_ms);
    EXPECT_LT(a_last, 1e-5);
    EXPECT_LT(o_last, 1e-5);
}

TEST(SolverCrossCheck, TimingReport) {
    const QpProblem p = boxProblem(18, 7u);
    AdmmSolver admm;
    OsqpSolver osqp;
    const auto a = admm.solve(p);
    const auto o = osqp.solve(p);
    std::printf("[XP] timing (18 vars, box): ADMM %.3f ms / %d iters, OSQP %.3f ms / %d iters\n",
                a.solve_time_ms, a.iterations, o.solve_time_ms, o.iterations);
    EXPECT_TRUE(a.converged);
    EXPECT_TRUE(o.converged);
}
