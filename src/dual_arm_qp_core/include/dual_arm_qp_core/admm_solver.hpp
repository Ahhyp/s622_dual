// include/dual_arm_qp_core/admm_solver.hpp
// Built-in ADMM (OSQP-style operator splitting) reference backend.
// Dependency-free, dense, intended for the small problems of this project
// (18 vars, ~12-30 constraints) and as a cross-check for OSQP.

#pragma once

#include "dual_arm_qp_core/qp_types.hpp"

namespace dual_arm_qp {

struct AdmmOptions {
    double rho = 1.0;
    double sigma = 1e-6;
    double eps_abs = 1e-7;
    double eps_rel = 1e-6;
    int max_iter = 1500;
    bool adaptive_rho = true;
    double rho_min = 1e-6;
    double rho_max = 1e6;
};

class AdmmSolver : public QpSolver {
public:
    explicit AdmmSolver(AdmmOptions options = {});

    QpSolution solve(const QpProblem& problem) override;
    std::string name() const override { return "admm"; }

    void setOptions(const AdmmOptions& options) { options_ = options; }
    const AdmmOptions& options() const { return options_; }

private:
    AdmmOptions options_;
};

}  // namespace dual_arm_qp
