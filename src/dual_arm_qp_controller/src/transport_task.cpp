// src/transport_task.cpp
// C4.3 S4 W1: transport-cycle policy (no ROS, no allocation on the control path).
#include "dual_arm_qp_controller/transport_task.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

void TransportTask::hold(const char* code) {
    state_ = TransportState::Hold;
    failure_code_ = (code != nullptr) ? code : "hold";
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
    if (state_ != TransportState::Hold) return false;

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
