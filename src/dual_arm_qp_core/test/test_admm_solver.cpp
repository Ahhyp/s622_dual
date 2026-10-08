// test/test_admm_solver.cpp
// Backend sanity checks for the ADMM reference QP solver.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdio>
#include <limits>

#include "dual_arm_qp_core/admm_solver.hpp"
#include "dual_arm_qp_core/qp_types.hpp"

namespace {

using dual_arm_qp::AdmmOptions;
using dual_arm_qp::AdmmSolver;
using dual_arm_qp::QpProblem;

constexpr double kInf = std::numeric_limits<double>::infinity();

QpProblem makeLeastSquares(const Eigen::VectorXd& b) {
    const int n = static_cast<int>(b.size());
    QpProblem p;
    p.P = 2.0 * Eigen::MatrixXd::Identity(n, n);
    p.q = -2.0 * b;
    p.A = Eigen::MatrixXd::Identity(n, n);
    p.l = Eigen::VectorXd::Constant(n, -kInf);
    p.u = Eigen::VectorXd::Constant(n, kInf);
    return p;
}

}  // namespace

TEST(AdmmSolver, UnconstrainedLeastSquares) {
    const Eigen::VectorXd b = (Eigen::VectorXd(3) << 1.0, -2.0, 0.5).finished();
    const QpProblem p = makeLeastSquares(b);
    AdmmSolver solver;
    const auto sol = solver.solve(p);
    ASSERT_TRUE(sol.converged) << "iter=" << sol.iterations << " pri=" << sol.primal_res
                               << " dua=" << sol.dual_res;
    EXPECT_LT((sol.x - b).cwiseAbs().maxCoeff(), 1e-6);
    std::printf("[QP] unconstrained: iters=%d pri=%.2e dua=%.2e\n", sol.iterations,
                sol.primal_res, sol.dual_res);
}

TEST(AdmmSolver, BoxClampedSolution) {
    const Eigen::VectorXd b = (Eigen::VectorXd(3) << 2.0, -3.0, 0.0).finished();
    QpProblem p = makeLeastSquares(b);
    const Eigen::VectorXd lo = (Eigen::VectorXd(3) << -1.0, -1.0, -1.0).finished();
    const Eigen::VectorXd hi = (Eigen::VectorXd(3) << 1.0, 1.0, 1.0).finished();
    p.l = lo;
    p.u = hi;
    AdmmSolver solver;
    const auto sol = solver.solve(p);
    ASSERT_TRUE(sol.converged);
    const Eigen::VectorXd expected = b.cwiseMax(lo).cwiseMin(hi);
    EXPECT_LT((sol.x - expected).cwiseAbs().maxCoeff(), 1e-6);
    std::printf("[QP] box clamp: iters=%d x=[%.4f %.4f %.4f]\n", sol.iterations, sol.x[0],
                sol.x[1], sol.x[2]);
}

TEST(AdmmSolver, LinearInequality) {
    // min ||x||^2  s.t.  x1 + x2 >= 1, optimum (0.5, 0.5).
    QpProblem p;
    p.P = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    p.q = Eigen::VectorXd::Zero(2);
    p.A = Eigen::MatrixXd(1, 2);
    p.A << -1.0, -1.0;  // -x1 - x2 <= -1
    p.l = Eigen::VectorXd::Constant(1, -kInf);
    p.u = Eigen::VectorXd::Constant(1, -1.0);
    AdmmSolver solver;
    const auto sol = solver.solve(p);
    ASSERT_TRUE(sol.converged);
    EXPECT_NEAR(sol.x[0], 0.5, 1e-6);
    EXPECT_NEAR(sol.x[1], 0.5, 1e-6);
    std::printf("[QP] linear ineq: iters=%d x=[%.6f %.6f]\n", sol.iterations, sol.x[0], sol.x[1]);
}

TEST(AdmmSolver, DefaultFactoryIsAvailable) {
    const auto solver = dual_arm_qp::makeDefaultSolver();
    ASSERT_TRUE(solver != nullptr);
    std::printf("[QP] default backend = %s (osqp built-in = %d)\n", solver->name().c_str(),
                static_cast<int>(dual_arm_qp::hasOsqpBackend()));
}
