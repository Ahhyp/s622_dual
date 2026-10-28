// src/transport_task.cpp
// C4.3 S4 W1: transport-cycle policy (no ROS, no allocation on the control path).
#include "dual_arm_qp_controller/transport_task.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace dual_arm_qp_controller {

namespace {
constexpr const char* kCodeOk = "";
constexpr const char* kCodeRowInvalid = "object_task_row_invalid";
constexpr const char* kCodeNearPi = "object_error_near_pi";
constexpr const char* kCodeSolverFailed = "qp_solver_failed";
constexpr const char* kCodeResumeFailed = "resume_failed";
}  // namespace

const char* transportStateName(TransportState s) noexcept {
    switch (s) {
        case TransportState::Idle: return "IDLE";
        case TransportState::Running: return "RUNNING";
        case TransportState::Hold: return "HOLD";
        case TransportState::Abort: return "ABORT";
        case TransportState::Fault: return "FAULT";
        case TransportState::Complete: return "COMPLETE";
    }
    return "UNKNOWN";
}

dual_arm::Q12 defaultHomeQ() {
    // S2 frozen baseline, transport_baseline.yaml:66 (q_grasp12).
    dual_arm::Q12 q;
    q << 2.795036909814, -1.706950857914, 1.279963524342, -1.143808993223,
        -1.570796326795, -0.346555743776, 0.346555743776, -1.434641795676,
        -1.279963524342, -1.997783660367, 1.570796326795, -2.795036909814;
    return q;
}

bool executorHeartbeatStale(bool seen, double last_heartbeat_s, double now_s,
                           double timeout_s) {
    if (!seen) return true;                       // never heard from the executor
    if (!std::isfinite(last_heartbeat_s)) return true;
    if (!std::isfinite(now_s)) return false;
    const double age = now_s - last_heartbeat_s;
    if (age < 0.0) return false;                   // clock step; not evidence of loss
    return age > timeout_s;
}

ProtectionReport buildProtectionReport(bool e7_arm_gap_enabled, bool e8_heartbeat_enabled) {
    ProtectionReport r;
    r.items = {
        {"e1_joint_bounds", ProtectionCoverage::Implemented,
         "q_state outside q_min/q_max latches HOLD before any math"},
        {"e2_state_staleness", ProtectionCoverage::Degraded,
         "ros2_control hands values without timestamps; a failed read returns ERROR, "
         "so a frozen-but-finite source is NOT detectable at this layer"},
        {"e3_solve_streak", ProtectionCoverage::Implemented,
         "consecutive QP solve failures latch HOLD"},
        {"e4_deadline", ProtectionCoverage::Implemented,
         "measured update() period > 2*dt latches HOLD (single overrun, stricter than "
         "the planned 5-consecutive rule)"},
        {"e5_row_valid", ProtectionCoverage::Implemented,
         "ObjectTaskRowCheck != Ok is refused before the solver"},
        {"e6_near_pi", ProtectionCoverage::Implemented,
         "object attitude error > pi-0.05 is refused before the solver"},
        {"e7_arm_gap", e7_arm_gap_enabled ? ProtectionCoverage::Implemented
                                          : ProtectionCoverage::Missing,
         e7_arm_gap_enabled ? "arm-arm distance < collision_d_stop latches HOLD"
                            : "collision_d_stop <= 0 disables the arm-arm interlock"},
        {"e8_executor_heartbeat", e8_heartbeat_enabled ? ProtectionCoverage::Implemented
                                                       : ProtectionCoverage::Missing,
         e8_heartbeat_enabled ? "executor heartbeat age > timeout latches HOLD"
                              : "require_executor_heartbeat=false (no executor in S4.1 yet)"},
    };
    std::string ok, deg, miss;
    for (const auto& it : r.items) {
        std::string* dst = (it.coverage == ProtectionCoverage::Implemented)  ? &ok
                           : (it.coverage == ProtectionCoverage::Degraded)    ? &deg
                                                                             : &miss;
        if (!dst->empty()) *dst += ",";
        *dst += it.id;
    }
    r.complete = deg.empty() && miss.empty();
    r.summary = std::string(r.complete ? "COMPLETE" : "PARTIAL") + "(" + ok + ")";
    if (!deg.empty()) r.summary += " degraded(" + deg + ")";
    if (!miss.empty()) r.summary += " missing(" + miss + ")";
    return r;
}

TransportTask::TransportTask(const dual_arm::DualArmKinematics& kin,
                             const TransportTaskConfig& cfg)
    : builder_(kin, cfg.object_T_left_tcp, cfg.object_T_right_tcp), cfg_(cfg) {}

bool TransportTask::start(const dual_arm_object_task::TrajectorySpec& spec,
                          const dual_arm::Q12& q, std::string* err) {
    dual_arm_object_task::TrajectorySpec s = spec;
    if (s.dt <= 0.0) s.dt = cfg_.dt;
    // §4.4: the start pose is the *measured* anchored estimate T_hat_{O,L}(q),
    // not the nominal baseline pose -- the two differ by whatever relative
    // error the arms currently carry.
    // B (S4 W1 review): a transport may only start from a defined standing
    // configuration.  Otherwise the anchored start pose silently becomes
    // "wherever the arms happen to be", and the whole trajectory is relative to
    // an arbitrary pose.
    start_pose_deviation_ = 0.0;
    started_from_home_ = false;
    hold_origin_ = HoldOrigin::None;
    if (cfg_.start_pose_tolerance_rad > 0.0) {
        int worst = 0;
        for (int i = 0; i < 12; ++i) {
            const double d = std::fabs(q[i] - cfg_.home_q[i]);
            if (d > start_pose_deviation_) {
                start_pose_deviation_ = d;
                worst = i;
            }
        }
        if (!(start_pose_deviation_ <= cfg_.start_pose_tolerance_rad)) {
            std::ostringstream os;
            os << "start pose deviation " << start_pose_deviation_ << " rad at joint " << worst
               << " (j" << (worst % 6 + 1) << " of the " << (worst < 6 ? "left" : "right")
               << " arm) exceeds " << cfg_.start_pose_tolerance_rad
               << " rad: move both arms to the standing configuration first";
            if (err != nullptr) *err = os.str();
            return false;
        }
        started_from_home_ = true;
    }
    if (s.start_from_baseline) {
        s.start_world_T_object = builder_.leftEstimate(q);
        s.start_from_baseline = false;
    }
    std::shared_ptr<const dual_arm_object_task::FrozenObjectTrajectory> traj;
    try {
        traj = std::make_shared<const dual_arm_object_task::FrozenObjectTrajectory>(s);
    } catch (const std::exception& e) {
        if (err != nullptr) *err = e.what();
        return false;
    }
    return latch(std::move(traj), err);
}

bool TransportTask::latch(std::shared_ptr<const dual_arm_object_task::FrozenObjectTrajectory> traj,
                          std::string* err) {
    if (!traj) {
        if (err != nullptr) *err = "null trajectory";
        return false;
    }
    if (!(traj->duration() > 0.0)) {
        if (err != nullptr) *err = "trajectory duration must be > 0";
        return false;
    }
    traj_ = std::move(traj);
    // §4.4: the relative target is the frozen grasp chain, computed once from
    // the baseline grasps -- never re-derived from measurements.
    left_T_right_ = builder_.leftTrightGrasp();
    t_traj_ = 0.0;
    state_ = TransportState::Running;
    failure_code_ = kCodeOk;
    solve_fail_streak_ = 0;
    resume_fail_count_ = 0;
    resume_shift_.setZero();
    if (err != nullptr) err->clear();
    return true;
}

TransportSample TransportTask::sample(const dual_arm::Q12& q) const {
    TransportSample s;
    s.active = active();
    s.state = state_;
    if (!traj_) return s;

    s.duration = traj_->duration();
    s.t_traj = t_traj_;

    dual_arm::Vec6 v_ff = dual_arm::Vec6::Zero();
    s.T_d = traj_->at(t_traj_, &v_ff);
    // §4.2.2: a resume shift is a pure world translation of the remaining path.
    // v_d = [R_d^T p_d_dot; w] does not see it, so the feed-forward is unchanged.
    s.T_d.translation() += resume_shift_;

    const dual_arm_object_task::AnchoredObjectTask task = builder_.build(q, s.T_d, v_ff);
    // SIGN CONTRACT: the QP row is (J_O, -e_O, +v_d).
    s.row.J = task.J;
    s.row.e = -task.e;
    s.row.body_twist_ff = task.v_ff;
    // valid comes from inputs_ok ONLY -- never from anchor_ok (§2.5 rationale).
    s.row.valid = task.inputs_ok;

    s.e_obj_pos_m = task.e.head<3>().norm();
    s.e_obj_rot_rad = task.e.tail<3>().norm();
    s.anchor_pos_gap_m = task.anchor_gap.pos_m;
    s.anchor_rot_gap_rad = task.anchor_gap.rot_rad;

    if (!task.inputs_ok) {
        s.refusal_code = kCodeRowInvalid;
    } else if (s.e_obj_rot_rad > cfg_.obj_rot_error_refuse_rad) {
        // §4.7.1 E6: the error-to-velocity map degrades near |e_rot| ~ pi
        // (measured: 20% at pi-1e-4, 100% at pi-1e-6), so refuse *before* the
        // solver rather than trust a near-pi linearisation.
        s.refusal_code = kCodeNearPi;
    }
    return s;
}

void TransportTask::advance(bool committed) {
    if (!traj_ || state_ != TransportState::Running || !committed) return;
    t_traj_ = std::min(t_traj_ + cfg_.dt, traj_->duration());
    if (t_traj_ >= traj_->duration()) state_ = TransportState::Complete;
}

CycleOutcome TransportTask::step(const dual_arm::Q12& q, dual_arm_qp::DualArmQp& qp) {
    CycleOutcome out;
    out.active = active();
    out.t_traj = t_traj_;
    out.failure_code = failure_code_;

    // HOLD / ABORT / FAULT / IDLE: frozen, nothing is solved, nothing is written.
    if (state_ != TransportState::Running && state_ != TransportState::Complete) {
        return out;
    }

    const TransportSample s = sample(q);
    out.e_obj_pos_m = s.e_obj_pos_m;
    out.e_obj_rot_rad = s.e_obj_rot_rad;
    out.t_traj = s.t_traj;

    if (s.refusal_code[0] != '\0') {
        // Refuse BEFORE the solver: no QP call, so iterations == 0 downstream.
        hold(s.refusal_code);
        out.failure_code = failure_code_;
        return out;
    }

    out.object_task_external = true;
    const dual_arm_qp::DualArmQpResult r = qp.solve(q, left_T_right_, s.row);
    out.iterations = r.iterations;
    out.solve_time_ms = r.solve_time_ms;
    out.converged = r.converged;
    out.dq = r.dq;

    if (!r.converged) {
        out.dq.setZero();
        if (++solve_fail_streak_ >= cfg_.solve_fail_streak_max) {
            fault(kCodeSolverFailed);
        }
        out.failure_code = failure_code_;
        return out;
    }
    solve_fail_streak_ = 0;
    out.committed = true;
    advance(true);
    out.t_traj = t_traj_;
    return out;
}

void TransportTask::hold(const char* code, HoldOrigin origin) {
    state_ = TransportState::Hold;
    failure_code_ = (code != nullptr) ? code : "hold";
    hold_origin_ = origin;
}

const char* holdOriginName(HoldOrigin o) noexcept {
    switch (o) {
        case HoldOrigin::None: return "";
        case HoldOrigin::Controller: return "controller";
        case HoldOrigin::Executor: return "executor";
    }
    return "";
}

void TransportTask::fault(const char* code) {
    state_ = TransportState::Fault;
    failure_code_ = (code != nullptr) ? code : "fault";
}

void TransportTask::abort(const char* code) {
    state_ = TransportState::Abort;
    failure_code_ = (code != nullptr) ? code : "abort";
}

bool TransportTask::resume(const dual_arm::Q12& q) {
    // C (S4 W1 review): ~/resume is a HOLD-only command.  FAULT/ABORT are
    // deliberately NOT resumable: FAULT needs ~/reset, ABORT needs a new
    // ~/start_transport.  Silently resuming from either would erase the reason
    // the machine stopped.
    if (state_ == TransportState::Fault || state_ == TransportState::Abort) {
        failure_code_ = (state_ == TransportState::Fault) ? "fault_requires_reset"
                                                          : "abort_requires_start";
        return false;
    }
    if (state_ != TransportState::Hold) {
        failure_code_ = "resume_requires_hold";
        return false;
    }
    const TransportSample s = sample(q);
    const bool pos_ok = s.e_obj_pos_m <= cfg_.resume_pos_tol_m;
    const bool rot_ok = s.e_obj_rot_rad <= cfg_.resume_rot_tol_rad;
    const bool row_ok = s.refusal_code[0] == '\0';

    if (pos_ok && rot_ok && row_ok) {
        state_ = TransportState::Running;
        failure_code_ = kCodeOk;
        resume_fail_count_ = 0;
        return true;
    }

    if (++resume_fail_count_ >= cfg_.resume_fail_max) {
        abort(kCodeResumeFailed);
        return false;
    }

    // §4.2.2: translate the remaining path onto the current anchored estimate.
    // Only the pose shifts; the velocity profile (hence v_ff) is untouched.
    const dual_arm_object_task::AnchoredObjectTask task =
        builder_.build(q, s.T_d, dual_arm::Vec6::Zero());
    resume_shift_ += task.T_hat_left.translation() - s.T_d.translation();
    state_ = TransportState::Running;
    failure_code_ = kCodeOk;
    return true;
}

void TransportTask::reset() {
    hold_origin_ = HoldOrigin::None;
    traj_.reset();
    t_traj_ = 0.0;
    left_T_right_ = dual_arm::Iso3::Identity();
    state_ = TransportState::Idle;
    failure_code_ = kCodeOk;
    solve_fail_streak_ = 0;
    resume_fail_count_ = 0;
    resume_shift_.setZero();
}

}  // namespace dual_arm_qp_controller
