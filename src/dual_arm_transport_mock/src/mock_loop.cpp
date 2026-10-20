// C4.3 S3 -- W6 implementation.  See mock_loop.hpp for the two-phase contract.
//
// Reading order: step() is §3.3 steps 3-13 verbatim; run() adds the streak
// counters (7, 9, 8) and the terminal criterion (14).
#include "dual_arm_transport_mock/mock_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "dual_arm_kinematics/closure_residual.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_transport_mock {

namespace {

using dual_arm::Iso3;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_qp::DualArmQpResult;
using dual_arm_qp::ObjectTaskRow;

double n3(const Vec6& v) { return v.head<3>().norm(); }
double n3r(const Vec6& v) { return v.tail<3>().norm(); }

std::string fmt(double v, int prec = 9) {
    std::ostringstream os;
    os.precision(prec);
    os << v;
    return os.str();
}

}  // namespace

const std::vector<std::string>& MockTransportLoop::objectSources() {
    static const std::vector<std::string> kSources{kObjectSourceLeftAnchor,
                                                   kObjectSourceDesired,
                                                   kObjectSourceRightAnchor};
    return kSources;
}

StackRank computeStackRank(const Mat6x12& J_rel, const Mat6x12& J_obj, double tau_ratio) {
    Eigen::Matrix<double, 12, 12> M;
    M.topRows<6>() = J_rel;
    M.bottomRows<6>() = J_obj;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(M);
    StackRank out;
    out.sigma_max = svd.singularValues()(0);
    out.sigma_min = svd.singularValues()(11);
    out.cond = (out.sigma_min > 0.0) ? out.sigma_max / out.sigma_min : 0.0;
    const double tol = tau_ratio * out.sigma_max;
    int rank = 0;
    for (int i = 0; i < 12; ++i) {
        if (svd.singularValues()(i) > tol) ++rank;
    }
    out.rank = rank;
    return out;
}

std::string RunSummary::to_string() const {
    std::ostringstream os;
    os << "success=" << (success ? 1 : 0) << " failure=" << failure_code;
    if (!failure_detail.empty()) os << " (" << failure_detail << ")";
    os << " failure_cycle=" << failure_cycle << " steps=" << steps
       << " duration_s=" << fmt(duration_s, 6) << " committed=" << committed_cycles
       << " vetoed=" << vetoed_cycles << " max_e_rel=" << fmt(max_e_rel_norm)
       << " max_e_obj=" << fmt(max_e_obj_norm) << " max_s_rel=" << fmt(max_s_rel_norm)
       << " max_anchor_gap=" << fmt(max_anchor_pos_gap_m)
       << " max_s_obj=" << fmt(max_s_obj_norm)
       << " terminal_pos_err_m=" << fmt(terminal_pos_err_m)
       << " terminal_rot_err_rad=" << fmt(terminal_rot_err_rad)
       << " min_clearance_m=" << fmt(min_clearance_m)
       << " min_clearance_cycle=" << min_clearance_cycle
       << " collisions=" << collision_cycles << " swept_failures=" << swept_failures
       << " solver_failures=" << solver_failures
       << " limit_saturation=" << limit_saturation_cycles
       << " max_step_norm_rad=" << fmt(max_step_norm_rad)
       << " legacy_margin_min_rad=" << fmt(legacy_margin_min_rad);
    return os.str();
}

MockTransportLoop::MockTransportLoop(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                                     const dual_arm_transport::GraspConfig& cfg,
                                     const dual_arm_transport::TransportBaseline& baseline,
                                     const TrajectorySpec& trajectory,
                                     const MockLoopOptions& opt)
    : scene_(std::move(scene)), cfg_(cfg), baseline_(baseline), opt_(opt) {
    if (scene_ == nullptr) throw std::invalid_argument("MockTransportLoop: null scene");
    if (opt_.check.sweep_samples < 2) {
        throw std::invalid_argument("MockTransportLoop: check.sweep_samples must be >= 2");
    }
    kin_ = std::make_unique<dual_arm::DualArmKinematics>();
    q_grasp_ = baseline_.q_grasp;
    // Frozen relative grasp G_L^-1 G_R.  Constant for a run -- re-deriving it from
    // the current state would turn the relative task into "hold what you have",
    // which tracks nothing.
    left_T_right_target_ = baseline_.object_T_left_tcp.inverse() * baseline_.object_T_right_tcp;

    task_ = std::make_unique<AnchoredObjectTaskBuilder>(*kin_, baseline_.object_T_left_tcp,
                                                        baseline_.object_T_right_tcp);
    checker_ = std::make_unique<TransportChecker>(scene_, cfg_, opt_.check);
    trajectory_ = std::make_unique<FrozenObjectTrajectory>(trajectory);

    auto params = opt_.qp;
    if (!opt_.use_object_task) {
        params.w_obj = 0.0;
        params.w_slack_obj = 1e-6;
    } else if (params.w_obj > 0.0 && params.w_slack_obj < 1e2) {
        // With w_obj > 0 but a nearly-free s_obj, s_obj absorbs the whole residual
        // at negligible cost and the object task has no teeth (§2.3.1, S3-T5h).
        throw std::invalid_argument(
            "MockTransportLoop: w_obj > 0 requires w_slack_obj >= 1e2 (got " +
            fmt(params.w_slack_obj) + "); see S3-T5h");
    }
    qp_ = std::make_unique<dual_arm_qp::DualArmQp>(
        *kin_, params,
        opt_.solver != nullptr ? opt_.solver : dual_arm_qp::makeDefaultSolver());

    // One source of truth for the limits.  `DualArmQpParams`' own constructor already
    // fills the S622 table, so calling defaultS622Limits here would (a) be redundant and
    // (b) silently DISCARD a caller override while still handing the override to the
    // solver -- after which `active_limit_count` would be counting a different box than
    // the one that was actually solved.  That is the same class of bug W0 found in the
    // controller (declared parameters nobody reads), so it is worth not having twice.
    q_min_ = opt_.qp.q_min;
    q_max_ = opt_.qp.q_max;
    v_max_ = opt_.qp.v_max;
}

Iso3 MockTransportLoop::sourcePose(const AnchoredObjectTask& task, const Iso3& desired,
                                   const std::string& source) const {
    if (source == kObjectSourceDesired) return desired;
    if (source == kObjectSourceLeftAnchor) return task.T_hat_left;
    if (source == kObjectSourceRightAnchor) return task.T_hat_right;
    throw std::invalid_argument("MockTransportLoop: unknown object_source '" + source + "'");
}

DualArmQpResult MockTransportLoop::cycle(const Q12& q, const Iso3& world_T_object_desired,
                                         const Vec6& v_ff) {
    const AnchoredObjectTask task = task_->build(q, world_T_object_desired, v_ff);
    ObjectTaskRow row;
    row.J = task.J;
    row.e = -task.e;                // §2.3.1(c): the one minus sign
    row.body_twist_ff = task.v_ff;  // §2.3.1(d): same convention, +v_d
    row.valid = task.inputs_ok;
    return qp_->solve(q, left_T_right_target_, row);
}

CycleOutcome MockTransportLoop::step(const Q12& q, double t, int k,
                                     const Iso3& world_T_object_desired, const Vec6& v_ff) {
    CycleOutcome out;
    CycleRecord& rec = out.record;
    rec.k = k;
    rec.t = t;
    rec.object_source = opt_.check.object_source;
    rec.q_prev = q;
    rec.q_cand = q;  // provisional: only a committed step moves it
    rec.world_T_object_desired = world_T_object_desired;
    rec.failure_code = "ok";

    // ---- 1b. is the state I am in even legal? ----
    // Without this, a state outside the joint limits reaches the solver as an
    // INFEASIBLE BOX (lower > upper) and comes back as "solver failed", i.e. the
    // failure is misattributed to the solver instead of to the limits.  The loop
    // starts inside the limits, so this is a guard, not a normal exit.
    {
        double margin = std::numeric_limits<double>::max();
        int worst_joint = -1;
        for (int i = 0; i < 12; ++i) {
            const double m = std::min(q[i] - q_min_[i], q_max_[i] - q[i]);
            if (m < margin) {
                margin = m;
                worst_joint = i;
            }
        }
        rec.min_limit_margin_rad = margin;
        if (margin < 0.0) {
            rec.failure_code = "joint_limit_saturation";
            rec.state_check.code = "joint_limit_saturation";
            rec.state_check.detail = "joint " + std::to_string(worst_joint) + " is " +
                                     fmt(-margin) + " rad outside its limits";
            out.abort = true;
            return out;
        }
    }

    // ---- 3. the object task row (e = -e_O, ff = +v_d) ----
    const AnchoredObjectTask task = task_->build(q, world_T_object_desired, v_ff);
    rec.T_hat_left = task.T_hat_left;
    rec.T_hat_right = task.T_hat_right;
    rec.e_obj_pos_m = n3(task.e);
    rec.e_obj_rot_rad = n3r(task.e);
    rec.anchor_pos_gap_m = task.anchor_gap.pos_m;
    rec.anchor_rot_gap_rad = task.anchor_gap.rot_rad;
    rec.legacy_margin_rad = task.legacy_margin_rad;
    rec.e_pos_m = (task.T_hat_left.translation() - world_T_object_desired.translation()).norm();
    rec.e_rot_rad = dual_arm::logSO3(world_T_object_desired.linear().transpose() *
                                     task.T_hat_left.linear())
                        .norm();
    {
        const Vec6 e_rel = kin_->relativeError(q, left_T_right_target_);
        rec.e_rel_pos_m = n3(e_rel);
        rec.e_rel_rot_rad = n3r(e_rel);
    }

    // ---- 4-5. independent judge on the CURRENT state ----
    checker_->applyWorld(sourcePose(task, world_T_object_desired, opt_.check.object_source),
                         opt_.check.object_source);
    const StateCheck now = checker_->checkState(q);
    rec.state_check = now;
    rec.whole_robot_m = now.whole_robot;
    rec.arm_arm_m = now.arm_arm;
    rec.arm_proximal_m = now.arm_proximal;
    if (now.code == "collision" || now.code == "insufficient_clearance") {
        rec.failure_code = now.code;
        out.abort = true;
        return out;
    }

    // ---- 6. rank pre-check on the stacked task (§2.3.2) ----
    const Mat6x12 J_rel = kin_->relativeJacobian(q);
    const StackRank rk = computeStackRank(J_rel, task.J);
    rec.sigma_min_rel = Eigen::JacobiSVD<Mat6x12>(J_rel).singularValues()(5);
    rec.rank_stack = rk.rank;
    rec.sigma_min_stack = rk.sigma_min;
    rec.cond_stack = rk.cond;
    if (opt_.enforce_rank_precheck &&
        (!rk.full_rank() || rk.sigma_min < opt_.sigma_min_stack_min)) {
        rec.failure_code = "task_stack_singular";
        out.abort = true;
        return out;
    }

    // ---- 7. the QP ----
    ObjectTaskRow row;
    row.J = task.J;
    row.e = -task.e;
    row.body_twist_ff = task.v_ff;
    row.valid = task.inputs_ok;
    out.qp = qp_->solve(q, left_T_right_target_, row);
    rec.converged = out.qp.converged;
    rec.iterations = out.qp.iterations;
    rec.solve_time_ms = out.qp.solve_time_ms;
    rec.dq = out.qp.dq;
    rec.slack_rel_pos_m = n3(out.qp.slack);
    rec.slack_rel_rot_rad = n3r(out.qp.slack);
    rec.slack_obj_pos_m = n3(out.qp.slack_object);
    rec.slack_obj_rot_rad = n3r(out.qp.slack_object);

    if (out.qp.object_task_check != dual_arm_qp::ObjectTaskRowCheck::Ok) {
        rec.failure_code = "object_task_row_invalid";
        out.abort = true;
        return out;
    }
    if (!out.qp.converged) {
        rec.failure_code = "qp_solver_failed";  // veto; streak counted in run()
        return out;
    }

    // ---- 9. closure / slack / anchor judges, before the candidate may move ----
    const dual_arm::ClosureResidual closure = dual_arm::closureResidual(task.T_hat_left,
                                                                       task.T_hat_right);
    rec.closure_pos_m = closure.translation_m;
    rec.closure_rot_rad = closure.rotation_rad;
    if (out.qp.slack.norm() > opt_.s_rel_max) {
        rec.failure_code = "closure_violated";
        out.abort = true;
        return out;
    }
    if (out.qp.slack_object.norm() > opt_.s_obj_max) {
        rec.failure_code = "object_task_infeasible";  // veto; streak counted in run()
        return out;
    }
    if (rec.anchor_pos_gap_m > opt_.anchor_pos_gap_max_m ||
        rec.anchor_rot_gap_rad > opt_.anchor_rot_gap_max_rad) {
        rec.failure_code = "anchor_disagreement";
        out.abort = true;
        return out;
    }

    // ---- 10. candidate: compute only, never commit here ----
    const Q12 q_cand = q + out.qp.dq;
    rec.q_cand = q_cand;
    {
        int active = 0;
        double margin = std::numeric_limits<double>::max();
        for (int i = 0; i < 12; ++i) {
            margin = std::min(margin, std::min(q_cand[i] - q_min_[i], q_max_[i] - q_cand[i]));
            const double lo = std::max(-v_max_[i] * opt_.qp.dt, q_min_[i] - q[i]);
            const double hi = std::min(v_max_[i] * opt_.qp.dt, q_max_[i] - q[i]);
            if (out.qp.dq[i] <= lo + 1e-12 || out.qp.dq[i] >= hi - 1e-12) ++active;
        }
        rec.active_limit_count = active;
        rec.min_limit_margin_rad = margin;
    }

    // ---- 11. independent judge on the CANDIDATE (the substance of "two-phase") ----
    checker_->applyWorld(sourcePose(task, world_T_object_desired, opt_.check.object_source),
                         opt_.check.object_source);
    const StateCheck cand = checker_->checkState(q_cand);
    rec.state_check = cand;
    rec.whole_robot_m = cand.whole_robot;
    rec.arm_arm_m = cand.arm_arm;
    rec.arm_proximal_m = cand.arm_proximal;
    if (cand.code == "collision" || cand.code == "insufficient_clearance") {
        rec.failure_code = cand.code;
        out.abort = true;
        return out;
    }

    // Every source is a different placement of the sample, i.e. a different world,
    // so the sweep runs once per source (§3.3 discipline 3).
    //
    // Only the ADOPTED source gates the commit.  The other two are what-ifs about a
    // sample pose the controller does not believe in: with the object task active all
    // three coincide to within the tracking error, but on the control arm (relative
    // task only) the object never moves while the desired pose walks away, and
    // gating on `desired` would abort the control arm for a sample that is not there.
    // The what-ifs are still recorded and still compared (S3-T20) -- they are just not
    // allowed to veto a motion that is safe under the pose the controller uses.
    bool swept_ok = true;
    std::string sweep_code;
    std::string sweep_detail;
    for (const std::string& source : objectSources()) {
        checker_->applyWorld(sourcePose(task, world_T_object_desired, source), source);
        SweepCheck swept = checker_->checkSweep(q, q_cand);
        if (swept.object_source.empty()) swept.object_source = source;
        // Record every source even after a failure: the log must show all three
        // placements, not just the one that happened to fail first (S3-T20).
        if (source == opt_.check.object_source) {
            rec.swept_ok = swept.ok;
            if (!swept.ok) {
                swept_ok = false;
                sweep_code = swept.code;
                sweep_detail = source + ": " + swept.detail;
            }
        }
        rec.sweeps.push_back(swept);
    }
    if (!swept_ok) {
        rec.failure_code =
            (sweep_code == "insufficient_clearance") ? "insufficient_clearance" : "swept_collision";
        rec.state_check.detail = sweep_detail;
        out.abort = true;
        return out;
    }

    // ---- 12. commit ----
    rec.committed = true;
    rec.failure_code = "ok";
    return out;
}

RunSummary MockTransportLoop::run(std::vector<CycleRecord>* log) {
    RunSummary sum;
    const double dt = opt_.qp.dt;
    const int steps = static_cast<int>(trajectory_->steps());
    const double duration = trajectory_->duration();
    sum.steps = steps;
    sum.duration_s = duration;
    sum.min_clearance_m = std::numeric_limits<double>::max();
    sum.legacy_margin_min_rad = std::numeric_limits<double>::max();

    Q12 q = q_grasp_;
    int solver_streak = 0;
    int obj_streak = 0;
    int limit_streak = 0;

    // A run that stops early still has a position, and that position is the whole
    // point of the control arm's report ("the object did not move, the target did").
    // Leaving the terminal error at 0 for failed runs made the W8 table say the control
    // arm had landed perfectly, which is the opposite of what happened.
    auto set_terminal = [&sum](const AnchoredObjectTask& task, const Iso3& T_des) {
        sum.terminal_pos_err_m = (task.T_hat_left.translation() - T_des.translation()).norm();
        sum.terminal_rot_err_rad =
            dual_arm::logSO3(T_des.linear().transpose() * task.T_hat_left.linear()).norm();
    };
    auto stop = [&](const std::string& code, const std::string& detail, int cycle, const Q12& qq,
                    const Iso3& T_des) {
        sum.success = false;
        sum.failure_code = code;
        sum.failure_detail = detail;
        sum.failure_cycle = cycle;
        set_terminal(task_->build(qq, T_des, Vec6::Zero()), T_des);
    };

    for (int k = 0; k < steps; ++k) {
        const double t = static_cast<double>(k) * dt;
        Vec6 v_ff = Vec6::Zero();
        const Iso3 T_d = trajectory_->at(t, &v_ff);
        if (!opt_.use_object_feedforward) v_ff.setZero();

        CycleOutcome outcome = step(q, t, k, T_d, v_ff);
        CycleRecord& rec = outcome.record;

        // streak bookkeeping (steps 7, 9 and 8 of §3.3)
        solver_streak = (rec.failure_code == "qp_solver_failed") ? solver_streak + 1 : 0;
        obj_streak = (rec.failure_code == "object_task_infeasible") ? obj_streak + 1 : 0;
        limit_streak = (rec.active_limit_count >= 12) ? limit_streak + 1 : 0;
        if (rec.failure_code == "qp_solver_failed") sum.solver_failures += 1;
        if (rec.active_limit_count >= 12) sum.limit_saturation_cycles += 1;

        sum.max_e_rel_norm = std::max(sum.max_e_rel_norm,
                                      std::hypot(rec.e_rel_pos_m, rec.e_rel_rot_rad));
        sum.max_e_obj_norm = std::max(sum.max_e_obj_norm,
                                      std::hypot(rec.e_obj_pos_m, rec.e_obj_rot_rad));
        sum.max_s_rel_norm = std::max(sum.max_s_rel_norm,
                                      std::hypot(rec.slack_rel_pos_m, rec.slack_rel_rot_rad));
        sum.max_s_obj_norm = std::max(sum.max_s_obj_norm,
                                      std::hypot(rec.slack_obj_pos_m, rec.slack_obj_rot_rad));
        sum.max_anchor_pos_gap_m = std::max(sum.max_anchor_pos_gap_m, rec.anchor_pos_gap_m);
        sum.max_anchor_rot_gap_rad = std::max(sum.max_anchor_rot_gap_rad, rec.anchor_rot_gap_rad);
        sum.legacy_margin_min_rad = std::min(sum.legacy_margin_min_rad, rec.legacy_margin_rad);
        sum.max_step_norm_rad = std::max(sum.max_step_norm_rad, rec.dq.cwiseAbs().maxCoeff());
        const double clearance = std::min(rec.whole_robot_m, rec.state_check.robot_world);
        if (clearance < sum.min_clearance_m) {
            sum.min_clearance_m = clearance;
            sum.min_clearance_cycle = k;
            sum.min_clearance_links = rec.state_check.link_a + "<->" + rec.state_check.link_b;
        }
        if (rec.state_check.code == "collision") sum.collision_cycles += 1;
        // Only cycles that actually reached the sweep can have failed it; an
        // aborted cycle leaves swept_ok at its default false.
        if (!rec.sweeps.empty() && !rec.swept_ok) sum.swept_failures += 1;
        if (rec.committed) {
            sum.committed_cycles += 1;
        } else {
            sum.vetoed_cycles += 1;
        }

        if (outcome.abort) {
            if (log != nullptr) log->push_back(rec);
            stop(rec.failure_code,
                 rec.state_check.detail.empty() ? rec.failure_code : rec.state_check.detail, k,
                 q, T_d);
            return sum;
        }
        if (solver_streak > opt_.solver_fail_streak_max) {
            if (log != nullptr) log->push_back(rec);
            stop("qp_solver_failed",
                 "solver did not converge for " + std::to_string(solver_streak) + " cycles", k,
                 q, T_d);
            return sum;
        }
        if (obj_streak > opt_.object_infeasible_streak_max) {
            if (log != nullptr) log->push_back(rec);
            stop("object_task_infeasible",
                 "object slack above s_obj_max for " + std::to_string(obj_streak) + " cycles", k,
                 q, T_d);
            return sum;
        }
        if (limit_streak > opt_.limit_saturation_streak_max) {
            if (log != nullptr) log->push_back(rec);
            stop("joint_limit_saturation",
                 "box constraints saturated for " + std::to_string(limit_streak) + " cycles", k,
                 q, T_d);
            return sum;
        }
        if (rec.committed) q = rec.q_cand;
        if (log != nullptr) log->push_back(rec);
    }

    // ---- 14. terminal criterion ----
    Vec6 v_end = Vec6::Zero();
    const Iso3 T_end = trajectory_->at(duration, &v_end);
    set_terminal(task_->build(q, T_end, Vec6::Zero()), T_end);
    if (sum.terminal_pos_err_m > opt_.terminal_pos_tol_m ||
        sum.terminal_rot_err_rad > opt_.terminal_rot_tol_rad) {
        sum.success = false;
        sum.failure_code = "terminal_pose_error";
        sum.failure_detail = "terminal pos err " + fmt(sum.terminal_pos_err_m) + " m, rot err " +
                             fmt(sum.terminal_rot_err_rad) + " rad";
        sum.failure_cycle = steps - 1;
        return sum;
    }
    sum.success = true;
    sum.failure_code = "ok";
    sum.failure_cycle = -1;
    return sum;
}

}  // namespace dual_arm_transport_mock
