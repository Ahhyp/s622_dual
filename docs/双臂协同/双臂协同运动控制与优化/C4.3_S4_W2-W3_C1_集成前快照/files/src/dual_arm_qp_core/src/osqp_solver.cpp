// src/osqp_solver.cpp
// Persistent-workspace OSQP backend (C2.R1).
//
// Steady state (structure unchanged): update P/A values + q/l/u, warm start,
// osqp_solve().  osqp_setup()/osqp_cleanup() and malloc/free happen only when
// the problem structure (n, m) changes.
//
// Sparsity patterns are FIXED:
//   P = dense upper triangle (n(n+1)/2 entries, explicit zeros allowed)
//   A = dense              (m*n entries)
// which keeps osqp_update_P_A valid for every cycle.

#include "dual_arm_qp_core/osqp_solver.hpp"

#include <chrono>
#include <cstdlib>

#include "cs.h"
#include "osqp.h"

namespace dual_arm_qp {

struct OsqpSolver::Impl {
    OsqpOptions options;

    OSQPWorkspace* work = nullptr;
    csc* P = nullptr;
    csc* A = nullptr;
    c_float* q = nullptr;
    c_float* l = nullptr;
    c_float* u = nullptr;
    c_float* x0 = nullptr;
    int n = 0;
    int m = 0;
    int setup_count = 0;

    ~Impl() { reset(); }

    void reset() {
        if (work != nullptr) {
            osqp_cleanup(work);
            work = nullptr;
        }
        if (P != nullptr) {
            csc_spfree(P);
            P = nullptr;
        }
        if (A != nullptr) {
            csc_spfree(A);
            A = nullptr;
        }
        std::free(q);
        q = nullptr;
        std::free(l);
        l = nullptr;
        std::free(u);
        u = nullptr;
        std::free(x0);
        x0 = nullptr;
        n = 0;
        m = 0;
    }

    bool allocStructure(int n_in, int m_in) {
        reset();
        n = n_in;
        m = m_in;
        const int p_nnz = n * (n + 1) / 2;
        const int a_nnz = m * n;

        P = csc_spalloc(n, n, p_nnz, 1, 0);
        A = csc_spalloc(m, n, a_nnz, 1, 0);
        q = static_cast<c_float*>(std::malloc(sizeof(c_float) * n));
        l = static_cast<c_float*>(std::malloc(sizeof(c_float) * m));
        u = static_cast<c_float*>(std::malloc(sizeof(c_float) * m));
        x0 = static_cast<c_float*>(std::malloc(sizeof(c_float) * n));
        if (P == nullptr || A == nullptr || q == nullptr || l == nullptr || u == nullptr ||
            x0 == nullptr) {
            reset();
            return false;
        }

        // P upper triangle, column-major (fixed pattern)
        int k = 0;
        for (int j = 0; j < n; ++j) {
            P->p[j] = k;
            for (int i = 0; i <= j; ++i) {
                P->i[k] = i;
                P->x[k] = 0.0;
                ++k;
            }
        }
        P->p[n] = k;
        P->nz = -1;

        // A dense, column-major (fixed pattern)
        k = 0;
        for (int j = 0; j < n; ++j) {
            A->p[j] = k;
            for (int i = 0; i < m; ++i) {
                A->i[k] = i;
                A->x[k] = 0.0;
                ++k;
            }
        }
        A->p[n] = k;
        A->nz = -1;

        return true;
    }

    bool osqpSetup() {
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
        settings.verbose = options.verbose ? 1 : 0;
        settings.eps_abs = static_cast<c_float>(options.eps_abs);
        settings.eps_rel = static_cast<c_float>(options.eps_rel);
        settings.max_iter = options.max_iter;
        settings.polish = options.polish ? 1 : 0;
        settings.warm_start = options.warm_start ? 1 : 0;

        if (osqp_setup(&work, &data, &settings) != 0) {
            work = nullptr;
            reset();
            return false;
        }
        ++setup_count;
        return true;
    }

    void updateValues(const QpProblem& p) {
        int k = 0;
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i <= j; ++i) {
                P->x[k++] = static_cast<c_float>(p.P(i, j));
            }
        }
        k = 0;
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < m; ++i) {
                A->x[k++] = static_cast<c_float>(p.A(i, j));
            }
        }
        for (int i = 0; i < n; ++i) q[i] = static_cast<c_float>(p.q[i]);
        for (int i = 0; i < m; ++i) {
            l[i] = static_cast<c_float>(p.l[i]);
            u[i] = static_cast<c_float>(p.u[i]);
        }
    }
};

OsqpSolver::OsqpSolver(OsqpOptions options) : impl_(std::make_unique<Impl>()) {
    impl_->options = options;
}

OsqpSolver::~OsqpSolver() = default;

void OsqpSolver::setOptions(const OsqpOptions& options) { impl_->options = options; }

OsqpOptions OsqpSolver::options() const { return impl_->options; }

int OsqpSolver::setupCount() const { return impl_->setup_count; }

void OsqpSolver::reset() { impl_->reset(); }

QpSolution OsqpSolver::solve(const QpProblem& problem) {
    QpSolution solution;
    if (!problem.valid()) return solution;

    const int n = problem.num_vars();
    const int m = problem.num_constraints();
    const auto t_start = std::chrono::steady_clock::now();

    if (impl_->work == nullptr || impl_->n != n || impl_->m != m) {
        // Structure (re)setup: allocate the fixed patterns, fill the REAL
        // numerical data, then call osqp_setup() once.
        if (!impl_->allocStructure(n, m)) return solution;
        impl_->updateValues(problem);
        if (!impl_->osqpSetup()) return solution;
    } else {
        impl_->updateValues(problem);
    }
    if (osqp_update_P_A(impl_->work, impl_->P->x, nullptr, 0, impl_->A->x, nullptr, 0) != 0 ||
        osqp_update_lin_cost(impl_->work, impl_->q) != 0 ||
        osqp_update_lower_bound(impl_->work, impl_->l) != 0 ||
        osqp_update_upper_bound(impl_->work, impl_->u) != 0) {
        impl_->reset();  // force a clean setup next time
        return solution;
    }

    if (impl_->options.warm_start && problem.x0.size() == n) {
        for (int i = 0; i < n; ++i) impl_->x0[i] = static_cast<c_float>(problem.x0[i]);
        osqp_warm_start_x(impl_->work, impl_->x0);
    }

    const c_int rc = osqp_solve(impl_->work);
    OSQPInfo* info = impl_->work->info;
    const bool solved = (info->status_val == OSQP_SOLVED ||
                         info->status_val == OSQP_SOLVED_INACCURATE);
    if (rc == 0 && solved) {
        solution.x.resize(n);
        for (int i = 0; i < n; ++i) {
            solution.x[i] = static_cast<double>(impl_->work->solution->x[i]);
        }
        solution.converged = true;
    }
    solution.iterations = static_cast<int>(info->iter);
    solution.primal_res = static_cast<double>(info->pri_res);
    solution.dual_res = static_cast<double>(info->dua_res);
    solution.solve_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start)
            .count();
    return solution;
}

}  // namespace dual_arm_qp
