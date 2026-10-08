// src/osqp_solver.cpp
// OSQP (0.6.x) backend.  P is passed as its upper triangle (OSQP convention),
// A and q as dense->CSC.  Warm start uses QpProblem::x0.

#include "dual_arm_qp_core/osqp_solver.hpp"

#include <chrono>
#include <cstdlib>
#include <vector>

#include "cs.h"
#include "osqp.h"

namespace dual_arm_qp {

namespace {

/// Dense -> CSC, upper triangle only (for P).
csc* denseUpperToCsc(const Eigen::MatrixXd& M) {
    const int n = static_cast<int>(M.rows());
    std::vector<c_int> p(n + 1, 0);
    std::vector<c_int> rows;
    std::vector<double> vals;
    rows.reserve(n * 2);
    vals.reserve(n * 2);
    for (int j = 0; j < n; ++j) {
        p[j] = static_cast<c_int>(rows.size());
        for (int i = 0; i <= j; ++i) {
            const double v = M(i, j);
            if (v != 0.0) {
                rows.push_back(static_cast<c_int>(i));
                vals.push_back(v);
            }
        }
    }
    p[n] = static_cast<c_int>(rows.size());
    const int nnz = static_cast<int>(rows.size());
    csc* out = csc_spalloc(n, n, nnz > 0 ? nnz : 1, 1, 0);
    for (int j = 0; j <= n; ++j) out->p[j] = p[j];
    for (int k = 0; k < nnz; ++k) {
        out->i[k] = rows[k];
        out->x[k] = vals[k];
    }
    out->nz = -1;
    return out;
}

/// Dense -> CSC, full matrix (for A).
csc* denseFullToCsc(const Eigen::MatrixXd& M) {
    const int m = static_cast<int>(M.rows());
    const int n = static_cast<int>(M.cols());
    std::vector<c_int> p(n + 1, 0);
    std::vector<c_int> rows;
    std::vector<double> vals;
    for (int j = 0; j < n; ++j) {
        p[j] = static_cast<c_int>(rows.size());
        for (int i = 0; i < m; ++i) {
            const double v = M(i, j);
            if (v != 0.0) {
                rows.push_back(static_cast<c_int>(i));
                vals.push_back(v);
            }
        }
    }
    p[n] = static_cast<c_int>(rows.size());
    const int nnz = static_cast<int>(rows.size());
    csc* out = csc_spalloc(m, n, nnz > 0 ? nnz : 1, 1, 0);
    for (int j = 0; j <= n; ++j) out->p[j] = p[j];
    for (int k = 0; k < nnz; ++k) {
        out->i[k] = rows[k];
        out->x[k] = vals[k];
    }
    out->nz = -1;
    return out;
}

}  // namespace

OsqpSolver::OsqpSolver(OsqpOptions options) : options_(options) {}

QpSolution OsqpSolver::solve(const QpProblem& problem) {
    QpSolution solution;
    if (!problem.valid()) return solution;

    const int n = problem.num_vars();
    const int m = problem.num_constraints();
    const auto t_start = std::chrono::steady_clock::now();

    csc* P = denseUpperToCsc(problem.P);
    csc* A = denseFullToCsc(problem.A);
    c_float* q = static_cast<c_float*>(std::malloc(sizeof(c_float) * n));
    c_float* l = static_cast<c_float*>(std::malloc(sizeof(c_float) * m));
    c_float* u = static_cast<c_float*>(std::malloc(sizeof(c_float) * m));
    for (int i = 0; i < n; ++i) q[i] = static_cast<c_float>(problem.q[i]);
    for (int i = 0; i < m; ++i) {
        l[i] = static_cast<c_float>(problem.l[i]);
        u[i] = static_cast<c_float>(problem.u[i]);
    }

    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = P;
    data.A = A;
    data.q = q;
    data.l = l;
    data.u = u;

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = options_.verbose ? 1 : 0;
    settings.eps_abs = static_cast<c_float>(options_.eps_abs);
    settings.eps_rel = static_cast<c_float>(options_.eps_rel);
    settings.max_iter = options_.max_iter;
    settings.polish = options_.polish ? 1 : 0;
    settings.warm_start = options_.warm_start ? 1 : 0;

    OSQPWorkspace* work = nullptr;
    const c_int setup_rc = osqp_setup(&work, &data, &settings);
    if (setup_rc == 0 && work != nullptr) {
        if (options_.warm_start && problem.x0.size() == n) {
            c_float* x0 = static_cast<c_float*>(std::malloc(sizeof(c_float) * n));
            for (int i = 0; i < n; ++i) x0[i] = static_cast<c_float>(problem.x0[i]);
            osqp_warm_start_x(work, x0);
            std::free(x0);
        }
        const c_int solve_rc = osqp_solve(work);
        OSQPInfo* info = work->info;
        const bool solved = (info->status_val == OSQP_SOLVED ||
                             info->status_val == OSQP_SOLVED_INACCURATE);
        if (solve_rc == 0 && solved) {
            solution.x.resize(n);
            for (int i = 0; i < n; ++i) {
                solution.x[i] = static_cast<double>(work->solution->x[i]);
            }
            solution.converged = true;
        }
        solution.iterations = static_cast<int>(info->iter);
        solution.primal_res = static_cast<double>(info->pri_res);
        solution.dual_res = static_cast<double>(info->dua_res);
        osqp_cleanup(work);
    }

    csc_spfree(P);
    csc_spfree(A);
    std::free(q);
    std::free(l);
    std::free(u);

    solution.solve_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start)
            .count();
    return solution;
}

}  // namespace dual_arm_qp
