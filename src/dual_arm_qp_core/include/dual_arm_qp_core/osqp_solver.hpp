// include/dual_arm_qp_core/osqp_solver.hpp
// OSQP backend (compiled in only when the osqp package is found).
//
// C2.R1: the OSQP workspace is PERSISTENT.  osqp_setup() is called once per
// problem structure (n, m); every subsequent solve() only updates the numerical
// data (P/A values, q, l, u) and re-solves with warm start.  No malloc/free and
// no osqp_setup/cleanup in the steady-state control path.
//
// The OSQP types are hidden behind a PIMPL so this header stays free of the
// (private) OSQP dependency.

#pragma once

#include <memory>
#include <string>

#include "dual_arm_qp_core/qp_types.hpp"

namespace dual_arm_qp {

struct OsqpOptions {
    double eps_abs = 1e-7;
    double eps_rel = 1e-6;
    int max_iter = 4000;
    bool polish = true;
    bool warm_start = true;
    bool verbose = false;
};

class OsqpSolver : public QpSolver {
public:
    explicit OsqpSolver(OsqpOptions options = {});
    ~OsqpSolver() override;
    OsqpSolver(const OsqpSolver&) = delete;
    OsqpSolver& operator=(const OsqpSolver&) = delete;

    QpSolution solve(const QpProblem& problem) override;
    std::string name() const override { return "osqp"; }

    void setOptions(const OsqpOptions& options);
    OsqpOptions options() const;

    /// Number of osqp_setup() calls so far.  Must stay 1 while the problem
    /// structure (n, m) is unchanged -- this is what C2.R1 is about.
    int setupCount() const;

    /// Drop the cached workspace (the next solve() will re-setup).
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dual_arm_qp
