// include/dual_arm_qp_core/qp_types.hpp
// Solver-agnostic QP data types (decision D6: replaceable backend).

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace dual_arm_qp {

/// minimize  0.5 x' P x + q' x
/// subject to  l <= A x <= u
struct QpProblem {
    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    Eigen::MatrixXd A;
    Eigen::VectorXd l;
    Eigen::VectorXd u;

    /// Optional warm start (size n); ignored if empty or wrong size.
    Eigen::VectorXd x0;

    int num_vars() const { return static_cast<int>(P.rows()); }
    int num_constraints() const { return static_cast<int>(A.rows()); }
    bool valid() const;
};

struct QpSolution {
    Eigen::VectorXd x;
    bool converged = false;
    int iterations = 0;
    double primal_res = 0.0;
    double dual_res = 0.0;
    double solve_time_ms = 0.0;
};

class QpSolver {
public:
    virtual ~QpSolver() = default;
    virtual QpSolution solve(const QpProblem& problem) = 0;
    virtual std::string name() const = 0;
};

using QpSolverPtr = std::shared_ptr<QpSolver>;

/// Backend factory: returns the best available solver (OSQP if compiled in,
/// otherwise the built-in ADMM reference backend).
QpSolverPtr makeDefaultSolver();

/// True if the library was built with the OSQP backend.
bool hasOsqpBackend();

}  // namespace dual_arm_qp
