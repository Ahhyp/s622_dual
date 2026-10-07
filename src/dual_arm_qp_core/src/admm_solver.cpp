// src/admm_solver.cpp
// OSQP-style ADMM on  min 0.5 x'Px + q'x  s.t. l <= Ax <= u.
//
// Iterations:
//   x <- (P + sigma I + rho A'A)^-1 (sigma x + rho A'(z - y) - q)
//   z <- clamp(A x + y, l, u)
//   y <- y + A x - z
// with an optional adaptive rho. The KKT matrix is constant between rho
// changes, so it is factorized once (dense LLT -- fine for ~18 vars).

#include "dual_arm_qp_core/admm_solver.hpp"

#ifdef DUAL_ARM_QP_HAVE_OSQP
#include "dual_arm_qp_core/osqp_solver.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace dual_arm_qp {

bool QpProblem::valid() const {
    if (P.rows() == 0 || P.rows() != P.cols()) return false;
    if (q.size() != P.rows()) return false;
    if (A.cols() != P.rows()) return false;
    if (A.rows() != l.size() || A.rows() != u.size()) return false;
    for (int i = 0; i < l.size(); ++i) {
        if (l[i] > u[i]) return false;
    }
    return true;
}

AdmmSolver::AdmmSolver(AdmmOptions options) : options_(options) {}

QpSolution AdmmSolver::solve(const QpProblem& problem) {
    QpSolution solution;
    if (!problem.valid()) return solution;

    const int n = problem.num_vars();
    const int m = problem.num_constraints();
    const auto t_start = std::chrono::steady_clock::now();

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    if (problem.x0.size() == n) x = problem.x0;  // warm start
    Eigen::VectorXd z = (problem.A * x).cwiseMax(problem.l).cwiseMin(problem.u);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd z_prev = z;

    double rho = options_.rho;
    const Eigen::MatrixXd At = problem.A.transpose();
    const Eigen::MatrixXd AAt = At * problem.A;
    const Eigen::MatrixXd sigma_I = options_.sigma * Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd K;
    Eigen::LLT<Eigen::MatrixXd> llt;
    auto factorize = [&]() {
        K = problem.P + sigma_I + rho * AAt;
        llt.compute(K);
    };
    factorize();
    if (llt.info() != Eigen::Success) {
        // Fall back to a tiny regularization bump (P should already be PD).
        factorize();
    }

    bool converged = false;
    int iter = 0;
    double primal_res = 0.0;
    double dual_res = 0.0;

    for (iter = 1; iter <= options_.max_iter; ++iter) {
        z_prev = z;

        const Eigen::VectorXd rhs =
            options_.sigma * x - problem.q + rho * (At * (z - y));
        x = llt.solve(rhs);

        const Eigen::VectorXd Ax = problem.A * x;
        z = (Ax + y).cwiseMax(problem.l).cwiseMin(problem.u);
        y += Ax - z;

        primal_res = (Ax - z).lpNorm<Eigen::Infinity>();
        dual_res = rho * (At * (z - z_prev)).lpNorm<Eigen::Infinity>();

        const double eps_pri =
            options_.eps_abs +
            options_.eps_rel * std::max(Ax.lpNorm<Eigen::Infinity>(), z.lpNorm<Eigen::Infinity>());
        const double eps_dual =
            options_.eps_abs + options_.eps_rel * (At * y).lpNorm<Eigen::Infinity>();

        if (primal_res <= eps_pri && dual_res <= eps_dual) {
            converged = true;
            break;
        }

        if (options_.adaptive_rho && iter % 25 == 0) {
            double next = rho;
            if (primal_res > 10.0 * dual_res) {
                next = rho * 2.0;
            } else if (dual_res > 10.0 * primal_res) {
                next = rho * 0.5;
            }
            next = std::min(options_.rho_max, std::max(options_.rho_min, next));
            if (next != rho) {
                rho = next;
                factorize();
            }
        }
    }

    solution.x = x;
    solution.converged = converged;
    solution.iterations = iter;
    solution.primal_res = primal_res;
    solution.dual_res = dual_res;
    solution.solve_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start)
            .count();
    return solution;
}

QpSolverPtr makeDefaultSolver() {
#ifdef DUAL_ARM_QP_HAVE_OSQP
    return std::make_shared<OsqpSolver>();
#else
    return std::make_shared<AdmmSolver>();
#endif
}

bool hasOsqpBackend() {
#ifdef DUAL_ARM_QP_HAVE_OSQP
    return true;
#else
    return false;
#endif
}

}  // namespace dual_arm_qp
