// C4.3 S3 -- W6: the mock closed loop with TWO-PHASE COMMIT (§3.3) and the §9
// failure-code state machine.
//
// The one thing to keep in mind while reading this file: a candidate state is
// COMPUTED, then checked by an independent collision judge, and only then
// committed.  A vetoed candidate is recorded (committed == false) and q does not
// move.  Nothing downstream may treat "dq was computed" as "dq was executed":
// `CycleRecord::committed` is the only such signal (§3.3 discipline 1).
//
// The collision judge (TransportChecker) is NOT part of the QP.  It cannot be
// traded off against tracking error, which is exactly why §3.3 calls it an
// independent referee.
#ifndef DUAL_ARM_TRANSPORT_MOCK__MOCK_LOOP_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__MOCK_LOOP_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_transport_mock/transport_checker.hpp"

namespace dual_arm_transport_mock {

// C4.3 S4 W0: the anchored object task now lives in dual_arm_object_task (it
// moved out of this mock package so the production controller can use it).
// A using-directive -- NOT a second definition: there is exactly one
// AnchoredObjectTaskBuilder in src/, enforced by
// dual_arm_object_task/cmake/check_single_object_task_definition.cmake.
using namespace dual_arm_object_task;  // NOLINT(build/namespaces)

/// §2.3.2 pre-check constant: the stacked [J_rel; J_O] must be full rank AND not
/// merely near-singular.  Below this the 12-DOF stack cannot track both tasks and
/// the closed loop degrades to "the QP picks something"; aborting is honest.
inline constexpr double kSigmaMinStackMin = 1.0e-6;
/// Relative-singular-value threshold that defines the numeric rank.
inline constexpr double kRankTolRatio = 1.0e-9;

struct MockLoopOptions {
    dual_arm_qp::DualArmQpParams qp;  ///< w_obj / kp_obj / w_slack_obj set HERE
    CheckOptions check;
    bool use_object_feedforward = true;
    bool use_object_task = true;  ///< false => relative task only (the control arm)
    bool enforce_rank_precheck = true;
    double sigma_min_stack_min = kSigmaMinStackMin;

    double s_rel_max = 1.0e-3;  ///< [m / rad] mixed norm
    double s_obj_max = 5.0e-3;
    double terminal_pos_tol_m = 1.0e-3;
    double terminal_rot_tol_rad = 1.0e-3;
    /// Budget for the two anchored estimates to disagree.  The gap IS the
    /// relative-task violation propagated through the grasp geometry (measured:
    /// 1.08e-4 m while |e_rel| was 1.11e-4 during S3-T6's fast transport), so a
    /// 1e-9 budget would fire on legitimate tracking error -- the first draft did
    /// exactly that.  1e-3 matches the relative-task tolerance we actually promise
    /// and is still two orders below the 0.12 m sample, so it catches a broken or
    /// mis-wired anchor (which produces centimetres) without chasing round-off.
    /// Refuse to act when ||logSO3(R_d^T R_hat)|| gets this close to pi.  The anchored
    /// row is exact at any ARM-RELATIVE rotation, but the object-attitude error is a
    /// logSO3 coordinate whose derivative degrades near pi (measured in S3-T21:
    /// 3e-05 rel at pi-1e-2, 20 % at pi-1e-4, 115 % at pi-1e-5).  Default = pi - 0.05,
    /// i.e. 500x away from the measured onset of the breakdown.
    double obj_rot_error_refuse_rad = 3.0915926535897933;  // pi - 0.05

    double anchor_pos_gap_max_m = 1.0e-3;
    double anchor_rot_gap_max_rad = 1.0e-3;

    /// Optional solver override (S3-T13 injection: a solver that never converges).
    /// nullptr keeps the default backend.
    dual_arm_qp::QpSolverPtr solver;

    int solver_fail_streak_max = 10;
    int limit_saturation_streak_max = 25;
    int object_infeasible_streak_max = 25;
};

/// Numeric structure of the stacked task matrix (12 rows x 12 cols).
struct StackRank {
    int rank = 0;
    double sigma_max = 0.0;
    double sigma_min = 0.0;
    double cond = 0.0;
    bool full_rank() const { return rank == 12; }
};

/// SVD of [J_rel; J_obj].  Rank tolerance is `kRankTolRatio * sigma_max`.
StackRank computeStackRank(const dual_arm::Mat6x12& J_rel, const dual_arm::Mat6x12& J_obj,
                          double tau_ratio = kRankTolRatio);

/// One control cycle, exactly the §8 column set.  `q_prev` / `q_cand` /
/// `committed` together preserve "where it meant to go and why it did not".
struct CycleRecord {
    int k = 0;
    double t = 0.0;
    std::string object_source;

    dual_arm::Q12 q_prev = dual_arm::Q12::Zero();
    dual_arm::Q12 dq = dual_arm::Q12::Zero();
    dual_arm::Q12 q_cand = dual_arm::Q12::Zero();
    bool committed = false;
    std::string failure_code{"ok"};

    dual_arm::Iso3 world_T_object_desired = dual_arm::Iso3::Identity();
    dual_arm::Iso3 T_hat_left = dual_arm::Iso3::Identity();
    dual_arm::Iso3 T_hat_right = dual_arm::Iso3::Identity();

    /// Tracking error of the actual object estimate against the desired pose.
    double e_pos_m = 0.0;
    double e_rot_rad = 0.0;
    /// Relative-task error components [m; rad] (right-grasp frame).
    double e_rel_pos_m = 0.0;
    double e_rel_rot_rad = 0.0;
    /// Object-task error components (the QP row's e = -e_O).
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    /// QP slacks.
    double slack_rel_pos_m = 0.0;
    double slack_rel_rot_rad = 0.0;
    double slack_obj_pos_m = 0.0;
    double slack_obj_rot_rad = 0.0;
    /// Disagreement between the two independent anchored estimates (should be 0).
    double anchor_pos_gap_m = 0.0;
    double anchor_rot_gap_rad = 0.0;
    /// Kinematic closure residual (translation / rotation).
    double closure_pos_m = 0.0;
    double closure_rot_rad = 0.0;
    /// Conditioning of the stacked task.
    double sigma_min_rel = 0.0;
    double sigma_min_stack = 0.0;
    double cond_stack = 0.0;
    int rank_stack = 0;
    /// Box-constraint attribution on the CANDIDATE.
    int active_limit_count = 0;
    /// Smallest distance from the CANDIDATE configuration to any joint limit [rad].
    /// Measured after the step (that is the state the robot would be in), not at q.
    double min_limit_margin_rad = 0.0;
    /// Self-collision scopes (layer 1) on the CANDIDATE.
    double whole_robot_m = 0.0;
    double arm_arm_m = 0.0;
    double arm_proximal_m = 0.0;
    double legacy_margin_rad = 0.0;

    bool converged = false;
    int iterations = 0;
    double solve_time_ms = 0.0;
    StateCheck state_check;
    bool swept_ok = false;
    /// The same segment with the sample carried along the grasp (S3-T22).  `false`
    /// unless the tracking sweep actually ran; `sweep_tracked_code == "not_applicable"`
    /// marks the `desired` source, which has no configuration to be anchored to.
    bool sweep_tracked_ok = false;
    double sweep_tracked_min_clearance_m = -1.0;
    std::string sweep_tracked_code = "not_run";
    /// One entry per §8 object source, always all three (S3-T20).  A cycle is
    /// only "checked" when every source has been evaluated and none failed;
    /// discarding the passing sources would make the log unable to answer
    /// "which placement was the one that clipped the table?".
    std::vector<SweepCheck> sweeps;
};

struct RunSummary {
    bool success = false;
    std::string failure_code{"ok"};
    std::string failure_detail;
    int failure_cycle = -1;
    int steps = 0;
    double duration_s = 0.0;
    double max_e_rel_norm = 0.0;
    double max_e_obj_norm = 0.0;
    double max_s_rel_norm = 0.0;
    double max_s_obj_norm = 0.0;
    /// A2 asks for these at run level, not only per cycle: the anchor-disagreement
    /// audit is the one check that can pass cycle by cycle and still be wrong in
    /// aggregate, so the maximum must be in runs.csv next to the other maxima.
    double max_anchor_pos_gap_m = 0.0;
    double max_anchor_rot_gap_rad = 0.0;
    double terminal_pos_err_m = 0.0;
    double terminal_rot_err_rad = 0.0;
    double min_clearance_m = 0.0;
    int min_clearance_cycle = -1;
    std::string min_clearance_links;
    int collision_cycles = 0;
    int swept_failures = 0;
    int solver_failures = 0;
    int limit_saturation_cycles = 0;
    int committed_cycles = 0;
    int vetoed_cycles = 0;
    double max_step_norm_rad = 0.0;
    double legacy_margin_min_rad = 0.0;

    std::string to_string() const;
};

/// The outcome of one two-phase step: the record plus the raw QP result, so a
/// test can inspect both the decision and what the solver returned.
struct CycleOutcome {
    CycleRecord record;
    dual_arm_qp::DualArmQpResult qp;
    bool abort = false;  ///< "stop the run", not "veto this cycle"
};

class MockTransportLoop {
public:
    MockTransportLoop(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                      const dual_arm_transport::GraspConfig& cfg,
                      const dual_arm_transport::TransportBaseline& baseline,
                      const TrajectorySpec& trajectory, const MockLoopOptions& opt = {});

    RunSummary run(std::vector<CycleRecord>* log = nullptr);

    /// The thin adapter of §5 question 5: build the row, solve, return.  No
    /// checks, no commit, no hidden tuning -- that is what makes the T5
    /// equivalence test meaningful.
    dual_arm_qp::DualArmQpResult cycle(const dual_arm::Q12& q,
                                      const dual_arm::Iso3& world_T_object_desired,
                                      const dual_arm::Vec6& v_ff);

    /// The full §3.3 step, used by `run()` and directly assertable by tests.
    CycleOutcome step(const dual_arm::Q12& q, double t, int k,
                      const dual_arm::Iso3& world_T_object_desired,
                      const dual_arm::Vec6& v_ff);

    const dual_arm::Iso3& leftTrightTarget() const { return left_T_right_target_; }
    const dual_arm::Q12& qGrasp() const { return q_grasp_; }
    const AnchoredObjectTaskBuilder& taskBuilder() const { return *task_; }
    const TransportChecker& checker() const { return *checker_; }
    const FrozenObjectTrajectory& trajectory() const { return *trajectory_; }
    const MockLoopOptions& options() const { return opt_; }
    /// The three §8 object sources, in the fixed order used for the candidate check.
    static const std::vector<std::string>& objectSources();

private:
    /// World placement of the sample for one object source, evaluated at `task`.
    dual_arm::Iso3 sourcePose(const AnchoredObjectTask& task,
                             const dual_arm::Iso3& desired,
                             const std::string& source) const;

    std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene_;
    dual_arm_transport::GraspConfig cfg_;
    dual_arm_transport::TransportBaseline baseline_;
    MockLoopOptions opt_;
    std::unique_ptr<dual_arm::DualArmKinematics> kin_;
    std::unique_ptr<dual_arm_qp::DualArmQp> qp_;
    std::unique_ptr<AnchoredObjectTaskBuilder> task_;
    std::unique_ptr<TransportChecker> checker_;
    std::unique_ptr<FrozenObjectTrajectory> trajectory_;

    dual_arm::Iso3 left_T_right_target_ = dual_arm::Iso3::Identity();
    dual_arm::Q12 q_grasp_ = dual_arm::Q12::Zero();
    dual_arm_qp::Vec12 q_min_ = dual_arm_qp::Vec12::Zero();
    dual_arm_qp::Vec12 q_max_ = dual_arm_qp::Vec12::Zero();
    dual_arm_qp::Vec12 v_max_ = dual_arm_qp::Vec12::Zero();
};

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__MOCK_LOOP_HPP_
