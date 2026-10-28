// include/dual_arm_qp_controller/transport_task.hpp
// C4.3 S4 W1: cooperative-transport task state for the QP controller.
//
// This class owns the *policy* of one transport cycle (S4 plan §4.2/§4.2.1/
// §4.2.2/§4.4) and is deliberately free of ROS so it can be unit tested
// without a ros2_control harness (S4-T3/T4/T5).  The controller is a thin
// shell: it reads state, asks this class for one cycle, and writes q_cmd.
//
// It does NOT re-implement any object-task math: the anchored task comes from
// dual_arm_object_task::AnchoredObjectTaskBuilder, the QP from dual_arm_qp_core.
// The object frame convention remains owned by dual_arm_kinematics.
//
// SIGN CONTRACT (S3 §2.3.1(c)(d), frozen): the S1 anchored triple gives
// J_O = d(e_O)/dq with e_O = current - desired, so the QP row is
//     row.J = J_O,  row.e = -e_O,  row.body_twist_ff = +v_d
#ifndef DUAL_ARM_QP_CONTROLLER__TRANSPORT_TASK_HPP_
#define DUAL_ARM_QP_CONTROLLER__TRANSPORT_TASK_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/frozen_baseline.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace dual_arm_qp_controller {

/// S4 plan §4.3 state machine, restricted to the transport sub-state
/// (APPROACH/GRASP/LIFT/LOWER/RELEASE are driven by the executor, W3+).
enum class TransportState : int {
    Idle = 0,    ///< no trajectory latched
    Running = 1, ///< playing; t_traj advances only on committed cycles
    Hold = 2,    ///< frozen; needs ~/resume (never auto-recovers)
    Abort = 3,   ///< frozen; needs a new ~/start_transport
    Fault = 4,   ///< latched hardware/math fault; needs ~/reset
    Complete = 5 ///< t_traj reached duration; object task holds the final pose
};

/// C7-A Rev.2 (B-3-1): who asked for the current HOLD.  This lives in the task
/// rather than in the controller because the pre-solver refusals raised inside
/// step() (near-pi / invalid row) are HOLDs too, and the controller has no call
/// site there -- tagging per call site could only ever cover 4 of the 5 sources.
enum class HoldOrigin : std::uint8_t {
    None = 0,       ///< never held
    Controller = 1, ///< own safety layer, or a pre-solver refusal in step()
    Executor = 2    ///< explicit ~/hold_transport request
};

const char* holdOriginName(HoldOrigin o) noexcept;

const char* transportStateName(TransportState s) noexcept;

/// S2 frozen baseline `q_grasp12` (transport_baseline.yaml:66).  It is the
/// canonical pre-transport configuration: the anchored start pose of a
/// transport is T_hat_{O,L}(q_grasp), so asking for a start while the arms are
/// somewhere else would silently change what "the object is here" means.
dual_arm::Q12 defaultHomeQ();

struct TransportTaskConfig {
    /// Frozen S2 baseline grasp transforms G_L / G_R (never measured, never
    /// re-derived).  The controller fills these from its parameters; the
    /// defaults come from frozen_baseline.hpp so a direct user of this struct
    /// cannot silently get an identity grasp.
    dual_arm::Iso3 object_T_left_tcp = frozenObjectTLeftTcp();
    dual_arm::Iso3 object_T_right_tcp = frozenObjectTRightTcp();

    /// S4 plan §4.7.1 E6: refuse before the QP when the object attitude error
    /// exceeds pi - 0.05.  Frozen by user decision (C4.3 S3 收口, m06104).
    double obj_rot_error_refuse_rad = 3.0915926535897933;

    double dt = 0.008;
    int solve_fail_streak_max = 10;
    /// §4.2.2: a second failed resume turns the HOLD into an ABORT.
    int resume_fail_max = 2;
    double resume_pos_tol_m = 5.0e-3;                  ///< 5 mm
    double resume_rot_tol_rad = 2.0 * 0.017453292519943295;  ///< 2 deg

    /// Pre-transport standing configuration and how far the arms may be from it
    /// when a transport is requested (max |q - home| over the 12 joints).
    /// 0 or negative tolerance disables the check (not recommended).
    dual_arm::Q12 home_q = defaultHomeQ();
    double start_pose_tolerance_rad = 0.05;
};

/// §4.7.1 protection coverage, as a machine-checkable statement rather than a
/// claim.  "implemented" means the controller itself detects and latches the
/// condition; "degraded" means a partial/indirect detector; "missing" means the
/// controller does NOT detect it at all (and must say so in ~/status).
enum class ProtectionCoverage : int {
    Implemented = 0,
    Degraded = 1,   ///< partial or indirect detection only
    Missing = 2
};

struct ProtectionItem {
    const char* id;          ///< "e1_joint_bounds" ... "e8_executor_heartbeat"
    ProtectionCoverage coverage;
    const char* note;        ///< why, in one sentence
};

struct ProtectionReport {
    bool complete = false;
    /// e.g. "PARTIAL(e1,e3,e4,e5,e6,e7) degraded(e2) missing(e8)"
    std::string summary;
    std::vector<ProtectionItem> items;
};

/// Fills in the coverage of one controller instance.  Pure, allocation is a
/// configure-time concern only.
ProtectionReport buildProtectionReport(bool e7_arm_gap_enabled, bool e8_heartbeat_enabled);

/// Pure E8 decision (§4.7.1): true => the executor heartbeat is stale, so both
/// arms must hold.  Kept out of the control loop so the rule can be unit tested
/// without a ros2_control fixture; the loop only applies the verdict.
bool executorHeartbeatStale(bool seen, double last_heartbeat_s, double now_s, double timeout_s);

/// Everything one cycle needs to publish / log.  No allocation.
struct TransportSample {
    bool active = false;
    TransportState state = TransportState::Idle;
    dual_arm_qp::ObjectTaskRow row;
    dual_arm::Iso3 T_d = dual_arm::Iso3::Identity();
    double t_traj = 0.0;
    double duration = 0.0;
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    /// §2.5 closed-chain audit: disagreement of the two anchored estimates.
    double anchor_pos_gap_m = 0.0;
    double anchor_rot_gap_rad = 0.0;
    /// "" when the row may be solved; otherwise the §4.7.1 refusal code
    /// ("object_task_row_invalid" / "object_error_near_pi").
    const char* refusal_code = "";
};

/// Result of one control cycle.  `failure_code == ""` means "no new failure".
struct CycleOutcome {
    bool active = false;
    bool committed = false;
    bool converged = false;
    dual_arm_qp::Vec12 dq = dual_arm_qp::Vec12::Zero();
    int iterations = 0;
    double solve_time_ms = 0.0;
    const char* failure_code = "";
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    double t_traj = 0.0;
    /// true when the QP consumed an external object row (never the legacy one).
    bool object_task_external = false;
};

class TransportTask {
public:
    TransportTask(const dual_arm::DualArmKinematics& kin, const TransportTaskConfig& cfg);

    /// Latch a parsed trajectory.  `start_from_baseline` inside the spec has
    /// already been resolved by the caller (start pose = anchored estimate at
    /// the current q).  Allocates once, at a state transition -- never per cycle.
    bool latch(std::shared_ptr<const dual_arm_object_task::FrozenObjectTrajectory> traj,
               std::string* err);

    /// Convenience: resolve `start_from_baseline` from q, then latch.
    bool start(const dual_arm_object_task::TrajectorySpec& spec, const dual_arm::Q12& q,
               std::string* err);

    /// Pure: no state change.  Applies the resume shift (if any) to T_d only;
    /// v_ff is translation invariant, so it stays bit-identical (§4.2.2).
    TransportSample sample(const dual_arm::Q12& q) const;

    /// One cycle.  Mutates state (t_traj, streaks, HOLD/FAULT latches) but
    /// never allocates.
    CycleOutcome step(const dual_arm::Q12& q, dual_arm_qp::DualArmQp& qp);

    /// §4.2.1: advance the trajectory clock iff this cycle was committed.
    void advance(bool committed);

    /// origin defaults to Controller so every pre-existing call site keeps
    /// its exact meaning; only ~/hold_transport passes Executor.
    void hold(const char* code, HoldOrigin origin = HoldOrigin::Controller);
    void fault(const char* code);
    void abort(const char* code);

    /// §4.2.2: HOLD -> RUNNING.  On a failed check the remaining path is
    /// translated by (p_hat - p_d) -- a pure pose offset, so v_ff is unchanged.
    /// A second consecutive failure raises ABORT.
    bool resume(const dual_arm::Q12& q);

    /// FAULT/ABORT -> Idle (clears the latch).  Requires an explicit call.
    void reset();

    TransportState state() const { return state_; }
    bool active() const { return traj_ != nullptr; }
    bool running() const { return state_ == TransportState::Running; }
    const char* failureCode() const { return failure_code_; }
    HoldOrigin holdOrigin() const { return hold_origin_; }
    const dual_arm::Iso3& leftTrightTarget() const { return left_T_right_; }
    double tTraj() const { return t_traj_; }
    double duration() const { return traj_ ? traj_->duration() : 0.0; }

    // ---- W3-R2 (approved D5-1): task-phase observation ----------------------
    // The phase table is the FROZEN spec the controller was started with; these
    // accessors only expose it, they never reinterpret it.  phaseIndexAt() uses
    // the same half-open convention as the executor (last phase closed), so both
    // sides of the wire agree on boundary ownership.
    int phaseCount() const {
        return traj_ ? static_cast<int>(traj_->spec().phases.size()) : 0;
    }
    const char* phaseName(int i) const {
        if (traj_ == nullptr) return "";
        const auto& phases = traj_->spec().phases;
        if (i < 0 || static_cast<size_t>(i) >= phases.size()) return "";
        return phases[static_cast<size_t>(i)].name.c_str();
    }
    double phaseDuration(int i) const {
        if (traj_ == nullptr) return 0.0;
        const auto& phases = traj_->spec().phases;
        if (i < 0 || static_cast<size_t>(i) >= phases.size()) return 0.0;
        return phases[static_cast<size_t>(i)].duration;
    }
    /// Index of the phase containing t (clamped into [0, duration]); -1 if no
    /// trajectory is latched.
    int phaseIndexAt(double t) const {
        if (traj_ == nullptr) return -1;
        const auto& b = traj_->boundaryTimes();
        if (b.size() < 2) return 0;
        const int n = static_cast<int>(b.size()) - 1;
        if (!(t > b.front())) return 0;
        for (int i = 0; i < n; ++i) {
            if (t < b[static_cast<size_t>(i) + 1]) return i;
        }
        return n - 1;  // last phase is closed at the end of the trajectory
    }
    double phaseStartTime(int i) const {
        if (traj_ == nullptr) return 0.0;
        const auto& b = traj_->boundaryTimes();
        if (i < 0 || static_cast<size_t>(i + 1) >= b.size()) return 0.0;
        return b[static_cast<size_t>(i)];
    }
    int solveFailStreak() const { return solve_fail_streak_; }
    int resumeFailCount() const { return resume_fail_count_; }
    /// Accumulated pose offset applied by resume (identity when never shifted).
    const Eigen::Vector3d& resumeShift() const { return resume_shift_; }

    /// max |q - home| (rad) measured at the last accepted start().
    double startPoseDeviation() const { return start_pose_deviation_; }
    bool startedFromHome() const { return started_from_home_; }

    const dual_arm_object_task::AnchoredObjectTaskBuilder& builder() const { return builder_; }
    const TransportTaskConfig& config() const { return cfg_; }

private:
    dual_arm_object_task::AnchoredObjectTaskBuilder builder_;
    TransportTaskConfig cfg_;

    std::shared_ptr<const dual_arm_object_task::FrozenObjectTrajectory> traj_;
    double t_traj_ = 0.0;
    dual_arm::Iso3 left_T_right_ = dual_arm::Iso3::Identity();

    TransportState state_ = TransportState::Idle;
    const char* failure_code_ = "";
    HoldOrigin hold_origin_ = HoldOrigin::None;
    int solve_fail_streak_ = 0;
    int resume_fail_count_ = 0;

    /// Pure translation offset added to T_d (resume re-planning).  v_ff untouched.
    Eigen::Vector3d resume_shift_ = Eigen::Vector3d::Zero();

    double start_pose_deviation_ = 0.0;
    bool started_from_home_ = false;
};

}  // namespace dual_arm_qp_controller

#endif  // DUAL_ARM_QP_CONTROLLER__TRANSPORT_TASK_HPP_
