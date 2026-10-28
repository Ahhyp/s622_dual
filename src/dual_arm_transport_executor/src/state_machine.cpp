#include "dual_arm_transport_executor/state_machine.hpp"

#include <cstddef>
#include <cstring>
#include <utility>

#include "dual_arm_transport_executor/collision_guard.hpp"
#include "dual_arm_transport_executor/precheck.hpp"

namespace dual_arm_transport_executor
{

namespace
{
// Forward order of the executable states; used to decide whether a controller
// phase is ahead of the executor (advance) or behind it (desync).
int stateOrdinal(State s)
{
  switch (s) {
    case State::Idle: return 0;
    case State::Approach: return 1;
    case State::Grasp: return 2;
    case State::Lift: return 3;
    case State::Transport: return 4;
    case State::Lower: return 5;
    case State::Release: return 6;
    case State::Retreat: return 7;
    case State::Complete: return 8;
    default: return -1;
  }
}

// Only the four trajectory phases R2 owns map onto executor states.  Settle
// phases carry no state of their own: the executor stays in the state it is in
// and the entry is recorded as an observation, not a transition.
State stateForPhaseName(const std::string & name)
{
  if (name == "approach") {return State::Approach;}
  if (name == "lift") {return State::Lift;}
  if (name == "transport") {return State::Transport;}
  if (name == "lower") {return State::Lower;}
  return State::Idle;  // sentinel: no state mapping
}
}  // namespace

const char * toString(State s)
{
  switch (s) {
    case State::Idle: return "IDLE";
    case State::Approach: return "APPROACH";
    case State::Grasp: return "GRASP";
    case State::Lift: return "LIFT";
    case State::Transport: return "TRANSPORT";
    case State::Lower: return "LOWER";
    case State::Release: return "RELEASE";
    case State::Retreat: return "RETREAT";
    case State::Complete: return "COMPLETE";
    case State::Hold: return "HOLD";
    case State::Fault: return "FAULT";
    case State::Abort: return "ABORT";
  }
  return "UNKNOWN";
}

bool isActive(State s)
{
  return s == State::Approach || s == State::Grasp || s == State::Lift ||
         s == State::Transport || s == State::Lower || s == State::Release ||
         s == State::Retreat;
}

bool isLatched(State s)
{
  return s == State::Hold || s == State::Fault || s == State::Abort;
}

bool isTerminal(State s) {return s == State::Idle || s == State::Complete;}

// Registered == the code already exists in an approved contract.
// Unregistered codes are CONTRACT GAPS: W3-R1 needs them, no approved document
// defines them, and they must be registered before being called official.
const std::vector<FailureCodeInfo> & failureCodes()
{
  static const std::vector<FailureCodeInfo> kCodes = {
    {"ok", true, "no failure"},
    // S3 twelve (mock transport)
    {"qp_solver_failed", true, "S3 / plan E3"},
    {"object_task_infeasible", true, "S3"},
    {"closure_violated", true, "S3"},
    {"joint_limit_saturation", true, "S3 / plan E1"},
    {"task_stack_singular", true, "S3"},
    {"collision", true, "S3 / plan E7"},
    {"swept_collision", true, "S3"},
    {"insufficient_clearance", true, "S3"},
    {"terminal_pose_error", true, "S3"},
    {"anchor_disagreement", true, "S3"},
    {"trajectory_invalid", true, "S3"},
    // plan §4.8 additions
    {"grasp_check_failed", true, "plan §4.8"},
    {"attach_failed", true, "plan §4.8"},
    {"release_failed", true, "plan §4.8"},
    {"precheck_failed", true, "plan §4.8"},
    {"object_env_contact", true, "plan §4.8"},
    {"object_table_clearance", true, "plan §4.8"},
    {"follower_divergence", true, "plan §4.8"},
    {"stale_command", true, "plan §4.8"},
    {"stale_state", true, "plan §4.8 / E2"},
    // controller-side protection codes (W1 implementation, plan §4.7.1)
    {"deadline_missed", true, "plan E4"},
    {"non_finite_state", true, "W1 stream health"},
    {"tracking_error", true, "W1 stream health"},
    {"stall_injected", true, "W1 stream health"},
    {"stream_unhealthy", true, "W1 stream health"},
    {"object_error_near_pi", true, "plan E6"},
    {"object_task_row_invalid", true, "plan E5"},
    {"executor_lost", true, "plan E8"},
    {"joint_state_out_of_bounds", true, "plan E1"},
    {"resume_failed", true, "plan §4.2.2"},
    // ---- contract gaps (registered == false) ----
    {"status_never_received", false, "W3-R1 needs it; no approved contract defines it"},
    {"status_unparseable", false, "W3-R1 needs it; no approved contract defines it"},
    {"hold_invariant_violated", false, "W3-R1 needs it; no approved contract defines it"},
    {"phase_timeout", false, "plan says HOLD on phase timeout but gives no code"},
    {"abort_requested", false, "operator abort without a specific contract code"},
    {"phase_desync", false, "W3-R2 needs it; no approved contract defines phase desync"},
    {"phase_name_mismatch", false, "W3-R2 needs it; no approved contract defines phase names"},
    {"collision_arm_arm", false, "W3-R6 guard code; no approved contract defines it"},
    {"collision_object", false, "W3-R6 guard code; no approved contract defines it"},
    {"collision_sweep", false, "W3-R6 guard code; no approved contract defines it"},
    {"abort_not_confirmed", false, "P1-4: controller did not confirm the ABORT within the deadline"},
    {"abort_status_unknown", false, "P1-4: no fresh controller status while confirming an ABORT"},
  };
  return kCodes;
}

const FailureCodeInfo * findFailureCode(const std::string & id)
{
  for (const auto & c : failureCodes()) {
    if (id == c.id) {return &c;}
  }
  return nullptr;
}

Protection protectionOf(const std::string & code)
{
  if (code.empty() || code == "ok") {return Protection::None;}
  static const char * kHold[] = {
    "stale_state", "stale_command", "deadline_missed", "qp_solver_failed",
    "object_task_row_invalid", "object_error_near_pi", "joint_limit_saturation",
    "joint_state_out_of_bounds", "executor_lost", "tracking_error",
    "non_finite_state", "stall_injected", "stream_unhealthy",
    "status_never_received", "status_unparseable", "phase_timeout",
    "collision", "object_task_infeasible", "insufficient_clearance",
    "terminal_pose_error", "phase_desync", "phase_name_mismatch",
    // W3-R6 collision-guard verdict codes: a real geometric rejection is a HOLD
    // (the plant is stopped and a human decides), never an auto-clearable state.
    "collision_arm_arm", "collision_object", "collision_sweep",
  };
  static const char * kFault[] = {
    "grasp_check_failed", "attach_failed", "release_failed", "precheck_failed",
    "hold_invariant_violated", "resume_failed", "trajectory_invalid",
  };
  static const char * kAbort[] = {
    "closure_violated", "object_env_contact", "object_table_clearance",
    "follower_divergence", "swept_collision", "anchor_disagreement",
    "task_stack_singular", "abort_requested",
  };
  for (const char * c : kHold) {if (code == c) {return Protection::Hold;}}
  for (const char * c : kFault) {if (code == c) {return Protection::Fault;}}
  for (const char * c : kAbort) {if (code == c) {return Protection::Abort;}}
  // Unknown code: stay conservative.  An unrecognised controller failure must
  // not be treated as "no protection".
  return Protection::Hold;
}

TransportStateMachine::TransportStateMachine(Config cfg)
: cfg_(std::move(cfg))
{
  record(0.0, "init", "NONE", toString(state_), "", "constructed");
}

void TransportStateMachine::record(double now_s, const std::string & kind,
                                   const std::string & from, const std::string & to,
                                   const std::string & code, const std::string & detail)
{
  Event e;
  e.t = now_s;
  e.kind = kind;
  e.from = from;
  e.to = to;
  e.code = code;
  e.detail = detail;
  events_.push_back(std::move(e));
}

bool TransportStateMachine::heartbeatStale(double now_s) const
{
  if (!hb_seen_) {return true;}
  const double age = now_s - last_hb_;
  if (age < 0.0) {return false;}  // clock stepped backwards: do not trip
  return age > cfg_.heartbeat_timeout_s;
}

void TransportStateMachine::onHeartbeat(double now_s)
{
  last_hb_ = now_s;
  hb_seen_ = true;
}

void TransportStateMachine::captureFreeze(const ControllerStatus & st)
{
  if (!st.received) {return;}
  frozen_q_cmd_ = st.q_cmd;
  frozen_t_traj_ = st.t_traj;
  has_frozen_ = true;
}

void TransportStateMachine::enterProtection(Protection p, const std::string & code,
                                            double now_s, const std::string & detail)
{
  // Never downgrade, never auto-clear.
  if (state_ == State::Fault || state_ == State::Abort) {
    record(now_s, "protection", toString(state_), toString(state_), code,
           "ignored: already latched at a stricter level; " + detail);
    return;
  }
  if (state_ == State::Hold && p == Protection::Hold) {
    record(now_s, "protection", toString(state_), toString(state_), code,
           "additional hold reason; " + detail);
    if (failure_code_.empty()) {failure_code_ = code;}
    return;
  }
  const State from = state_;
  if (from != State::Hold) {
    resume_state_ = from;
    captureFreeze(last_status_);
    // Provisional snapshot: re-baselined by the first status that arrives while
    // latched (see tick()), because the controller freezes at drain time and the
    // status we latched on can be up to one publish period old.
    freeze_settled_ = false;
  }
  switch (p) {
    case Protection::Hold: state_ = State::Hold; break;
    case Protection::Fault: state_ = State::Fault; break;
    case Protection::Abort: state_ = State::Abort; resume_state_ = State::Idle; break;
    case Protection::None: return;
  }
  failure_code_ = code;
  state_since_ = now_s;
  record(now_s, "protection", toString(from), toString(state_), code, detail);
}

void TransportStateMachine::advance(double now_s, const std::string & detail)
{
  State next = state_;
  switch (state_) {
    case State::Approach: next = State::Grasp; break;
    case State::Grasp: next = State::Lift; break;
    case State::Lift: next = State::Transport; break;
    case State::Transport: next = State::Lower; break;
    case State::Lower: next = State::Release; break;
    case State::Release: next = State::Retreat; break;
    case State::Retreat: next = State::Complete; break;
    default:
      record(now_s, "reject", toString(state_), toString(state_), "invalid_transition",
             "advance() from a non-advancing state; " + detail);
      return;
  }
  record(now_s, "advance", toString(state_), toString(next), "", detail);
  state_ = next;
  state_since_ = now_s;
}

bool TransportStateMachine::start(double now_s, const StartRequest & req, std::string * why)
{
  if (state_ == State::Fault) {
    if (why) {*why = "fault_requires_reset";}
    record(now_s, "reject", toString(state_), toString(state_), "fault_requires_reset",
           "start() refused: FAULT is latched, explicit reset required");
    return false;
  }
  if (state_ == State::Hold) {
    if (why) {*why = "hold_requires_resume";}
    record(now_s, "reject", toString(state_), toString(state_), "hold_requires_resume",
           "start() refused: HOLD must be resumed or aborted first");
    return false;
  }
  if (isActive(state_)) {
    if (why) {*why = "already_active";}
    record(now_s, "reject", toString(state_), toString(state_), "already_active",
           "start() refused: a task is already running");
    return false;
  }
  // C4.3 S4 W7: with an attach client wired in, a task may not start while the
  // physical attach state is unknown or still welded.  Fortress 6.16 creates the
  // startup weld 2.165-2.201 s after the server comes up (W6-0), so "started
  // before the guard verified" is exactly the race that would teleport the
  // sample.  Refusing here keeps that decision out of the caller's hands.
  if (weld_ != nullptr && !weld_->startupVerified()) {
    if (why) {*why = "weld_startup_release_pending";}
    record(now_s, "reject", toString(state_), toString(state_),
           "weld_startup_release_pending",
           "start() refused: the physical attach state has not been released and verified");
    return false;
  }
  const PrecheckResult pc = checkStart(cfg_, last_status_, has_status_, now_s);
  if (!pc.ok) {
    if (why) {*why = pc.code;}
    enterProtection(Protection::Fault, "precheck_failed", now_s, pc.detail);
    return false;
  }
  const State from = state_;
  state_ = State::Approach;
  state_since_ = now_s;
  duration_ = req.duration_s;
  failure_code_.clear();
  resume_fail_ = 0;
  resume_state_ = State::Idle;
  has_frozen_ = false;
  has_last_guard_q_ = false;  // a new task has no previous observation to sweep
  no_status_since_ = -1.0;
  record(now_s, "start", toString(from), toString(state_), "",
         "trajectory duration " + std::to_string(req.duration_s) + " s");
  return true;
}

bool TransportStateMachine::abortTransport(const std::string & code, double now_s,
                                           std::string * why)
{
  if (state_ == State::Abort) {
    if (why) {*why = "already_aborted";}
    return false;
  }
  if (state_ == State::Fault) {
    if (why) {*why = "fault_requires_reset";}
    return false;
  }
  if (isTerminal(state_) && state_ != State::Complete) {
    if (why) {*why = "abort_not_active";}
    record(now_s, "reject", toString(state_), toString(state_), "abort_not_active",
           "abort() refused: nothing is running");
    return false;
  }
  const std::string c = code.empty() ? std::string("abort_requested") : code;
  enterProtection(Protection::Abort, c, now_s, "operator / executor abort");
  if (why) {why->clear();}
  return true;
}

// P1-4: an ABORT the controller never confirmed is not a safe termination.  The
// executor escalates it to a latching FAULT with its own code so the record
// cannot be mistaken for "we stopped the robot".
bool TransportStateMachine::escalateFault(const std::string & code, double now_s,
                                          const char * detail, std::string * why)
{
  std::string c = code.empty() ? std::string("abort_not_confirmed") : code;
  enterProtection(Protection::Fault, c, now_s,
                  detail != nullptr ? detail : "escalated by the executor");
  if (why != nullptr) {*why = "escalated to FAULT: " + c;}
  return true;
}

bool TransportStateMachine::resume(double now_s, std::string * why)
{
  if (state_ == State::Fault) {
    if (why) {*why = "fault_requires_reset";}
    record(now_s, "reject", toString(state_), toString(state_), "fault_requires_reset",
           "resume() refused");
    return false;
  }
  if (state_ == State::Abort) {
    if (why) {*why = "abort_requires_start";}
    record(now_s, "reject", toString(state_), toString(state_), "abort_requires_start",
           "resume() refused");
    return false;
  }
  if (state_ != State::Hold) {
    if (why) {*why = "resume_requires_hold";}
    record(now_s, "reject", toString(state_), toString(state_), "resume_requires_hold",
           "resume() refused");
    return false;
  }

  std::string reason;
  const ControllerStatus & st = last_status_;
  if (!has_status_) {
    reason = "no_status";
  } else if (!st.parse_ok) {
    reason = "status_unparseable";
  } else if (now_s - st.stamp_s > cfg_.status_stale_timeout_s) {
    reason = "stale_state";
  } else if (!st.failure_code.empty() && st.failure_code != "ok") {
    reason = "controller_failure_active:" + st.failure_code;
  } else if (st.e_obj_pos_m > cfg_.resume_pos_tol_m) {
    reason = "object_pos_error";
  } else if (st.e_obj_rot_rad > cfg_.resume_rot_tol_rad) {
    reason = "object_rot_error";
  } else if (st.active_limit_count > 0) {
    reason = "active_limit";
  } else if (st.solve_fail_streak >= cfg_.solve_fail_streak_limit) {
    reason = "solve_streak";
  } else if (st.executor_heartbeat_required && heartbeatStale(now_s)) {
    reason = "executor_lost";
  }

  if (!reason.empty()) {
    ++resume_fail_;
    record(now_s, "reject", toString(state_), toString(state_), "resume_failed_check",
           reason + " (failure " + std::to_string(resume_fail_) + ")");
    if (resume_fail_ > cfg_.resume_fail_max) {
      enterProtection(Protection::Abort, "resume_failed", now_s,
                      "resume refused " + std::to_string(resume_fail_) + " times; last: " + reason);
      if (why) {*why = "resume_failed";}
      return false;
    }
    if (why) {*why = reason;}
    return false;
  }

  const State from = state_;
  state_ = resume_state_;
  state_since_ = now_s;
  failure_code_.clear();
  resume_fail_ = 0;
  has_frozen_ = false;
  record(now_s, "resume", toString(from), toString(state_), "",
         "resume preconditions satisfied");
  if (why) {why->clear();}
  return true;
}

bool TransportStateMachine::reset(double now_s, std::string * why)
{
  if (state_ == State::Abort) {
    if (why) {*why = "abort_requires_start";}
    record(now_s, "reject", toString(state_), toString(state_), "abort_requires_start",
           "reset() refused: ABORT is cleared only by a new start()");
    return false;
  }
  if (state_ != State::Fault) {
    if (why) {*why = "reset_requires_fault";}
    record(now_s, "reject", toString(state_), toString(state_), "reset_requires_fault",
           "reset() refused");
    return false;
  }
  const State from = state_;
  state_ = State::Idle;
  state_since_ = now_s;
  failure_code_.clear();
  resume_state_ = State::Idle;
  resume_fail_ = 0;
  has_frozen_ = false;
  has_last_guard_q_ = false;
  record(now_s, "reset", toString(from), toString(state_), "", "explicit reset");
  if (why) {why->clear();}
  return true;
}

bool TransportStateMachine::phaseDone(double now_s, std::string * why)
{
  // C4.3 S4 W7: GRASP may not be declared done while the weld is unconfirmed, no
  // matter which side asks (controller phase or explicit step completion).  The
  // gripper feedback and the attach are both prerequisites, never shortcuts.
  if (weld_ != nullptr && state_ == State::Grasp && !weld_attached_) {
    if (why) {*why = "weld_not_confirmed";}
    record(now_s, "reject", "GRASP", "GRASP", "weld_not_confirmed",
           "phaseDone() refused: GRASP cannot advance before the attach is confirmed");
    return false;
  }
  // RELEASE must actually let go before retreating (S4 :838 order: support
  // confirmed -> detach -> gripper -> retreat).  A still-welded release that
  // "advances" would drag the sample away with the arm.
  if (weld_ != nullptr && state_ == State::Release && weld_attached_ && !weld_release_done_) {
    if (why) {*why = "weld_not_confirmed";}
    record(now_s, "reject", "RELEASE", "RELEASE", "weld_not_confirmed",
           "phaseDone() refused: RELEASE cannot advance while the sample is still welded");
    return false;
  }
  if (!isActive(state_)) {
    if (why) {*why = "phase_done_requires_active";}
    record(now_s, "reject", toString(state_), toString(state_), "phase_done_requires_active",
           "phaseDone() refused");
    return false;
  }
  advance(now_s, "simulated physical step completion (W3-R1: not a real actuator)");
  if (why) {why->clear();}
  return true;
}

namespace
{
// The guard's own bounds are the contract; never silently ask for a sweep that
// the guard would clamp or refuse.
static_assert(TransportStateMachine::kGuardSweepSamples >= CollisionGuard::kMinSamples &&
  TransportStateMachine::kGuardSweepSamples <= CollisionGuard::kMaxSamples,
  "kGuardSweepSamples must stay within CollisionGuard's sample bounds");
}  // namespace

void TransportStateMachine::setCollisionObjectPose(const dual_arm::Iso3 & object_pose)
{
  object_pose_ = object_pose;
  has_object_pose_ = true;
}

void TransportStateMachine::clearCollisionObjectPose() {has_object_pose_ = false;}

bool TransportStateMachine::checkCollisionGuard(double now_s, const ControllerStatus & st)
{
  // Every number below is measured by CollisionGuard
  // (DualArmCollisionModel::distance); this function only selects the frame and
  // turns a !ok verdict into a latch.  It never reads ControllerStatus::
  // failure_code, so the resulting code can only come from the guard call.
  dual_arm::Q12 q;
  for (std::size_t i = 0; i < 12; ++i) {q[i] = st.q_state[i];}
  ++collision_checks_;

  // Check 1: arm-arm clearance at the configuration the controller reports.
  const CollisionVerdict arm = collision_guard_->checkArmArm(q);
  if (!arm.ok) {
    enterProtection(Protection::Hold, arm.code, now_s,
                    "collision guard checkArmArm: clearance " +
                    std::to_string(arm.clearance_m) + " m (measured on q_state)");
    return true;
  }

  // Check 2: the optional object pose against the guard's exclusion spheres.
  if (has_object_pose_) {
    const CollisionVerdict obj = collision_guard_->checkObjectPose(object_pose_);
    if (!obj.ok) {
      enterProtection(Protection::Hold, obj.code, now_s,
                      "collision guard checkObjectPose: clearance " +
                      std::to_string(obj.clearance_m) + " m");
      return true;
    }
  }

  // Check 3: the segment between the two most recent observations.  A period
  // with no other real mid-period evidence must not hide a crossing that only
  // collides in between two individually safe configurations.
  if (has_last_guard_q_) {
    dual_arm::Q12 previous;
    for (std::size_t i = 0; i < 12; ++i) {previous[i] = last_guard_q_[i];}
    const CollisionVerdict sweep =
      collision_guard_->checkSweep(previous, q, kGuardSweepSamples);
    if (!sweep.ok) {
      enterProtection(Protection::Hold, sweep.code, now_s,
                      "collision guard checkSweep: clearance " +
                      std::to_string(sweep.clearance_m) + " m over " +
                      std::to_string(kGuardSweepSamples) + " samples");
      return true;
    }
  }
  last_guard_q_ = st.q_state;
  has_last_guard_q_ = true;
  return false;
}

void TransportStateMachine::checkTimeout(double now_s)
{
  const double elapsed = now_s - state_since_;
  double limit = -1.0;
  Protection p = Protection::Hold;
  std::string code;
  switch (state_) {
    case State::Approach:
      limit = cfg_.approach_timeout_s; p = Protection::Fault; code = "grasp_check_failed";
      break;
    case State::Grasp:
      limit = cfg_.grasp_timeout_s; p = Protection::Fault; code = "attach_failed";
      break;
    case State::Lift:
    case State::Transport:
    case State::Lower:
      limit = duration_ > 0.0 ? duration_ * cfg_.phase_timeout_scale : cfg_.phase_timeout_scale;
      p = Protection::Hold; code = "phase_timeout";
      break;
    case State::Release:
    case State::Retreat:
      limit = cfg_.release_timeout_s; p = Protection::Fault; code = "release_failed";
      break;
    default:
      return;
  }
  if (limit > 0.0 && elapsed > limit) {
    enterProtection(p, code, now_s,
                    "state timeout: elapsed " + std::to_string(elapsed) +
                    " s > limit " + std::to_string(limit) + " s");
  }
}

bool TransportStateMachine::tickWeld(double now_s)
{
  if (weld_ == nullptr) {return false;}

  if (state_ == State::Grasp && !weld_attached_) {
    if (!weld_attach_requested_ && !weld_->failed() &&
      weld_->state() == WeldState::Detached)
    {
      weld_attach_requested_ = true;
      weld_->requestAttach(now_s);
      record(now_s, "weld", "GRASP", "GRASP", "attach_requested",
             "welding the gripper to the sample (startup guard verified)");
    }
    const WeldAction a = weld_->tick(now_s);
    // Only an action produced by OUR attach request counts; the sticky
    // Confirmed left over from the startup release must never be read as one.
    if (a == WeldAction::Confirmed && weld_attach_requested_) {
      weld_attached_ = true;
      record(now_s, "weld", "GRASP", "GRASP", "attached", weld_->reason());
      return false;  // let the normal GRASP logic run on the next tick
    }
    if (a == WeldAction::Failed) {
      enterProtection(Protection::Hold, "attach_failed", now_s,
                      weld_->outcome() + ": " + weld_->reason());
      return true;
    }
    return true;  // still waiting for a confirmed attach
  }

  if (state_ == State::Release && weld_attached_ && !weld_release_done_) {
    if (!weld_detach_requested_ && weld_->state() == WeldState::Attached) {
      weld_detach_requested_ = true;
      weld_->requestDetach(now_s);
      record(now_s, "weld", "RELEASE", "RELEASE", "detach_requested",
             "releasing the sample before opening the gripper (S4 :838 order)");
    }
    const WeldAction a = weld_->tick(now_s);
    if (a == WeldAction::Confirmed) {
      weld_release_done_ = true;
      weld_attached_ = false;
      record(now_s, "weld", "RELEASE", "RELEASE", "detached", weld_->reason());
      return false;
    }
    if (a == WeldAction::Failed) {
      enterProtection(Protection::Fault, "release_failed", now_s,
                      weld_->outcome() + ": " + weld_->reason());
      return true;
    }
    return true;  // still waiting for a confirmed detach
  }
  return false;
}

void TransportStateMachine::tick(double now_s, const ControllerStatus * st_in)
{
  if (st_in != nullptr && st_in->received) {
    last_status_ = *st_in;
    has_status_ = true;
  }

  if (isTerminal(state_)) {return;}

  if (state_ == State::Hold || state_ == State::Fault || state_ == State::Abort) {
    // Latched: no progression and no auto-clear.  HOLD additionally audits that
    // the two frozen quantities really are frozen.
    if (state_ != State::Hold || st_in == nullptr || !st_in->received) {return;}
    if (!freeze_settled_) {
      // C7 review P1-3: the snapshot taken at latch time can predate the
      // controller's freeze (status is 10 Hz, the freeze is per control tick),
      // so a hold drained one cycle later used to look like a violation.
      // Re-baseline ONLY on a status that actually confirms OUR freeze
      // (hold_request_result==applied for a real request).  Without that
      // confirmation the original snapshot stands and the audit runs
      // immediately -- otherwise a controller that simply ignored the hold
      // would be re-baselined into looking healthy.
      freeze_settled_ = true;
      const bool answers_our_request = expected_hold_seq_ != 0u &&
        st_in->hold_request_seq == expected_hold_seq_;
      if (st_in->hold_request_result == 1 && answers_our_request) {
        captureFreeze(*st_in);
        return;
      }
    }
    if (has_frozen_) {
      if (std::memcmp(st_in->q_cmd.data(), frozen_q_cmd_.data(), sizeof(double) * 12) != 0) {
        state_ = State::Fault;
        failure_code_ = "hold_invariant_violated";
        state_since_ = now_s;
        record(now_s, "protection", "HOLD", "FAULT", "hold_invariant_violated",
               "q_cmd changed while the executor was holding (bitwise mismatch)");
        return;
      }
      if (st_in->t_traj != frozen_t_traj_) {
        state_ = State::Fault;
        failure_code_ = "hold_invariant_violated";
        state_since_ = now_s;
        record(now_s, "protection", "HOLD", "FAULT", "hold_invariant_violated",
               "t_traj advanced while the executor was holding");
        return;
      }
    }
    return;
  }

  // ---- active ----
  if (st_in == nullptr || !st_in->received) {
    if (no_status_since_ < 0.0) {no_status_since_ = now_s;}
    if (now_s - no_status_since_ > cfg_.status_stale_timeout_s) {
      enterProtection(Protection::Hold, "status_never_received", now_s,
                      "no controller status since " + std::to_string(no_status_since_));
      return;
    }
    checkTimeout(now_s);
    return;
  }
  no_status_since_ = -1.0;

  if (!st_in->parse_ok) {
    enterProtection(Protection::Hold, "status_unparseable", now_s, "status could not be parsed");
    return;
  }
  const double age = now_s - st_in->stamp_s;
  if (age > cfg_.status_stale_timeout_s) {
    enterProtection(Protection::Hold, "stale_state", now_s,
                    "status age " + std::to_string(age) + " s");
    return;
  }
  if (!st_in->failure_code.empty() && st_in->failure_code != "ok") {
    const Protection p = protectionOf(st_in->failure_code);
    if (p != Protection::None) {
      enterProtection(p, st_in->failure_code, now_s, "reported by the controller");
      return;
    }
  }
  if (st_in->solve_fail_streak >= cfg_.solve_fail_streak_limit) {
    enterProtection(Protection::Hold, "qp_solver_failed", now_s,
                    "solve_fail_streak " + std::to_string(st_in->solve_fail_streak));
    return;
  }
  if (st_in->active_limit_count > 0) {
    enterProtection(Protection::Hold, "joint_limit_saturation", now_s,
                    "active_limit_count " + std::to_string(st_in->active_limit_count));
    return;
  }
  if (st_in->executor_heartbeat_required && heartbeatStale(now_s)) {
    enterProtection(Protection::Hold, "executor_lost", now_s,
                    "executor heartbeat older than " +
                    std::to_string(cfg_.heartbeat_timeout_s) + " s");
    return;
  }

  // W3-R6: dry-run collision guard.  When the executor owns one, a real
  // geometric verdict latches HOLD with the guard's own code before any phase
  // bookkeeping -- a collision is not a phase event.
  if (collision_guard_ != nullptr && checkCollisionGuard(now_s, *st_in)) {return;}
  if (tickWeld(now_s)) {return;}

  // W3-R2 task-phase driving.  When a driver is attached the controller clock is
  // turned into a phase index and the executor walks APPROACH -> LIFT ->
  // TRANSPORT -> LOWER with it.  A jump, a regression or an unknown name is a
  // contract violation and latches HOLD; it is never silently absorbed.
  const bool phase_driven = (phase_ != nullptr && phase_->enabled());
  if (phase_driven && st_in->transport_active) {
    // W3-R2 (D5-1): when the controller reports a phase index, that is the
    // authority -- the executor must not believe its own clock over it.  The
    // local clock is only the fallback for a controller that does not publish
    // phases (index < 0), so both paths stay available and testable.
    const bool external = st_in->phase_index >= 0 && !st_in->phase_name.empty();
    const PhaseObservation po = external
      ? phase_->observeExternal(st_in->phase_index, st_in->phase_name, st_in->t_traj)
      : phase_->observeLocal(st_in->t_traj);
    switch (po.event) {
      case PhaseEvent::Desync:
        enterProtection(Protection::Hold, "phase_desync", now_s,
                        "phase index " + std::to_string(po.index) +
                        " (t_traj " + std::to_string(st_in->t_traj) + " s)");
        return;
      case PhaseEvent::UnknownName:
        // Report the name the CONTROLLER sent, not po.name: on a mismatch
        // po.name is the previously accepted phase, which made the log read
        // "unknown phase name 'approach'" when 'transport' was the offender.
        enterProtection(Protection::Hold, "phase_name_mismatch", now_s,
                        "phase name mismatch at index " + std::to_string(st_in->phase_index) +
                        ": controller reported '" + st_in->phase_name + "', expected '" +
                        (st_in->phase_index >= 0 && st_in->phase_index < phase_->count()
                        ? phase_->phases()[static_cast<std::size_t>(st_in->phase_index)].name
                        : std::string("<out of range>")) + "'");
        return;
      case PhaseEvent::Entered: {
        const State target = stateForPhaseName(po.name);
        if (target == State::Idle) {
          record(now_s, "phase", toString(state_), toString(state_), "",
                 "'" + po.name + "' entered (no executor state mapping; stays in place)");
        } else {
          int guard = 0;
          while (isActive(state_) && state_ != target &&
            stateOrdinal(state_) < stateOrdinal(target) && guard < 8)
          {
            advance(now_s, "controller phase '" + po.name + "'");
            ++guard;
          }
          if (state_ != target && stateOrdinal(state_) > stateOrdinal(target)) {
            enterProtection(Protection::Hold, "phase_desync", now_s,
                            "executor state " + std::string(toString(state_)) +
                            " is ahead of controller phase '" + po.name + "'");
            return;
          }
        }
        break;
      }
      case PhaseEvent::NotStarted:
      case PhaseEvent::None:
        break;
    }
  }

  // Trajectory-driven completion.  LIFT / TRANSPORT / LOWER are driven by the
  // controller trajectory, so their completion is a real signal, not simulated.
  // With a phase driver attached the phase index owns the transitions; the
  // end-of-trajectory rule stays as the fallback for the un-phase-driven case.
  if (!phase_driven &&
    (state_ == State::Lift || state_ == State::Transport || state_ == State::Lower)) {
    if (st_in->transport_active && st_in->duration > 0.0 &&
        st_in->t_traj >= st_in->duration - 1e-12)
    {
      advance(now_s, "controller reported the trajectory reached its end");
      return;
    }
  }
  checkTimeout(now_s);
}

}  // namespace dual_arm_transport_executor
