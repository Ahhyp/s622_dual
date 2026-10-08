// include/dual_arm_qp_core/osqp_solver.hpp
// OSQP backend (compiled in only when the osqp package is found).
// Requires: ros-humble-osqp-vendor (provides libosqp + cmake config).

#pragma once

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

    QpSolution solve(const QpProblem& problem) override;
    std::string name() const override { return "osqp"; }

    void setOptions(const OsqpOptions& options) { options_ = options; }
    const OsqpOptions& options() const { return options_; }

private:
    OsqpOptions options_;
};

}  // namespace dual_arm_qp
