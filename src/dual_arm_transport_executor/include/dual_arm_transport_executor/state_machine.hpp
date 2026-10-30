// C4.3 S4.1 W3-R1 — transport task executor state machine.
//
// Design rules (task book §3 W3-R1-01 / -02):
//   * The core depends on NO ROS clock, NO Gazebo and NO real actuator.
//   * Time, controller status, command emission and artifact writing are all
//     injected, so the whole thing is testable on a fixed simulated timeline.
//   * Protection states latch.  Nothing in this file may clear HOLD / FAULT /
//     ABORT as a side effect of a status message, a timeout retry or a restart.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__STATE_MACHINE_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__STATE_MACHINE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_transport_executor/attach_client.hpp"
#include "dual_arm_transport_executor/phase_driver.hpp"

namespace dual_arm_transport_executor
{

class CollisionGuard;  // C4.3 S4 W3-R6 dry-run hook (collision_guard.hpp); NOT owned

enum class State
{
  Idle,
  Approach,
  Grasp,
  Lift,
  Transport,
  Lower,
  Release,
  Retreat,
  Complete,
  Hold,
  Fault,
  Abort,
};

const char * toString(State s);
bool isActive(State s);    // Approach .. Retreat
bool isLatched(State s);   // Hold, Fault, Abort
bool isTerminal(State s);  // Idle, Complete

// A failure code is "registered" only when an approved contract already defines
// it (plan §4.7.1 / §4.8 / the S3 twelve).  Registered == false means: W3-R1
// needs the code but no approved contract defines it yet, so it is a CONTRACT
// GAP and must not be presented as an official code.
struct FailureCodeInfo
{
  const char * id;
  bool registered;
  const char * source;
};

const std::vector<FailureCodeInfo> & failureCodes();
const FailureCodeInfo * findFailureCode(const std::string & id);

// Which protection state a code drives.  None means "not a protection code".
enum class Protection {None, Hold, Fault, Abort};
Protection protectionOf(const std::string & failure_code);

// Mirror of the fields the executor needs from
// dual_arm_qp_controller/msg/TransportStatus (single source = that message; this
// struct exists so the core stays ROS-free).
struct ControllerStatus
{
  bool received = false;
  bool parse_ok = true;
  double stamp_s = 0.0;
  std::string state;
  std::string failure_code;
  bool transport_active = false;
  bool committed = false;
  double t_traj = 0.0;
  double duration = 0.0;
  std::array<double, 12> q_state{};
  std::array<double, 12> q_cmd{};
  std::array<double, 12> dq{};
  int solve_fail_streak = 0;
  int active_limit_count = 0;
  // Distance from the nearest joint *position* limit, in rad (signed: negative
  // means past the limit).  This is what "the arm is running out of travel"
  // actually looks like; active_limit_count is a velocity-cap saturation
  // counter and must not be read as a position-limit violation.
  double min_limit_margin_rad = 0.0;
  double e_obj_pos_m = 0.0;
  double e_obj_rot_rad = 0.0;
  double sigma_min_stack = 0.0;
  double executor_heartbeat_age_s = -1.0;
  // C4.3 S4.1 W3-R1a-2: the C7-A Rev.2 hold-ack triple.  Kept together because
  // the executor must judge (seq, state, origin) jointly -- see HoldRequester.
  std::string hold_origin;
  std::uint32_t hold_request_seq = 0;
  std::uint8_t hold_request_result = 0;
  // W3-R2 (approved D5-1): controller-reported task phase.  -1 = no trajectory.
  int phase_index = -1;
  int phase_count = 0;
  std::string phase_name;
  double phase_elapsed_s = 0.0;
  double phase_duration_s = 0.0;
  bool executor_heartbeat_seen = false;
  bool executor_heartbeat_required = false;
};

struct Event
{
  double t = 0.0;
  std::string kind;    // start / advance / protection / resume / reset / reject / phase / heartbeat
  std::string from;
  std::string to;
  std::string code;
  std::string detail;
};

struct Config
{
  double dt = 0.008;
  // Plan E2: status older than three control periods is stale.
  double status_stale_timeout_s = 0.024;
  // Plan E8, frozen at 100 ms.
  double heartbeat_timeout_s = 0.100;
  // [待标定] in the plan -> carried here as an explicit, overridable value.
  double approach_timeout_s = 5.0;
  double grasp_timeout_s = 3.0;
  double release_timeout_s = 3.0;
  /// S4-T16: when true, GRASP must wait for the node's gripper-close and
  /// grasp-relative-check verdicts before it may request attach.  Default false
  /// preserves the pre-S4-T16 software tests and the mock/dry-run profiles; the
  /// C4.3 Gazebo launch sets it true.
  bool require_grasp_check = false;
  double phase_timeout_scale = 1.5;
  // Plan §4.2.2: resume requires e_O <= 5 mm / 2 deg.
  double resume_pos_tol_m = 0.005;
  double resume_rot_tol_rad = 0.03490658503988659;
  double v_max = 2.5;
  int resume_fail_max = 2;
  int solve_fail_streak_limit = 10;
  // Joint-limit pressure protection: HOLD only after this many *consecutive*
  // status samples show the arm either at a velocity cap or inside
  // limit_margin_rad of a position limit.  One saturated cycle is a normal
  // transient of a transport start, not a fault (W7-B finding, 2026-10-28).
  int limit_saturation_cycles = 25;
  // How close to a position limit counts as "running out of room".  Default
  // 0.0 = only an actual violation (margin < 0); a positive band is a profile
  // choice (the Gazebo profile sets 0.05 rad).
  double limit_margin_rad = 0.0;
};

struct StartRequest
{
  double duration_s = 0.0;
};

class TransportStateMachine
{
public:
  explicit TransportStateMachine(Config cfg = Config());

  // ---- commands (non-realtime side) ----
  bool start(double now_s, const StartRequest & req, std::string * why);
  bool abortTransport(const std::string & code, double now_s, std::string * why);
  /// P1-3/C7: which ack seq would confirm OUR hold request.  The freeze baseline
  /// is only re-taken on a status that answers this exact seq -- an old applied
  /// ack from a previous request must not be mistaken for this one.
  void expectHoldAck(std::uint32_t seq) { expected_hold_seq_ = seq; }
  /// P1-4: escalate an unconfirmed ABORT into a latching FAULT (own code).
  bool escalateFault(const std::string & code, double now_s,
                     const char * detail, std::string * why);
  bool resume(double now_s, std::string * why);
  bool reset(double now_s, std::string * why);
  void onHeartbeat(double now_s);

  // C4.3 S4.1 W3-R2: optional task-phase driver.  nullptr (the default) keeps
  // the W3-R1 behaviour exactly: LIFT/TRANSPORT/LOWER advance only when the
  // controller reports the end of the trajectory.  Attaching a driver makes the
  // executor follow APPROACH -> LIFT -> TRANSPORT -> LOWER phase by phase and
  // HOLD on any index desync.  The driver is NOT owned by the state machine.
  // C4.3 S4 W7: optional physical-attach client (W6).  nullptr (the default)
  // keeps every existing behaviour bit for bit.  When one is attached:
  //   * start() is refused until the startup weld has been released and the
  //     release confirmed (a startup weld is never evidence of a grasp);
  //   * GRASP does not advance until the weld is CONFIRMED, and a failed attach
  //     is a HOLD carrying `attach_failed` + the client's own code;
  //   * RELEASE detaches before it advances; a failed detach is a FAULT
  //     (`release_failed`, the code this state already uses for release faults).
  // The client is NOT owned by the state machine.
  void attachWeldClient(AttachClient * client) {weld_ = client;}
  AttachClient * weldClient() const {return weld_;}
  bool weldAttached() const {return weld_attached_;}
  // C4.3 S4 S4-T16: two injected GRASP prerequisites.  The node owns the ROS
  // action/pose plumbing; the state machine only consumes the verdicts.  A
  // failed verdict latches FAULT(grasp_check_failed) and attach is never sent.
  void noteGripperClosed(double now_s, bool ok, const std::string & detail);
  void noteGraspCheck(double now_s, bool ok, const std::string & detail);
  bool gripperReady() const {return gripper_ready_;}
  bool graspCheckDone() const {return grasp_checked_;}
  bool graspOk() const {return grasp_ok_;}
  void attachPhaseDriver(PhaseDriver * driver) {phase_ = driver;}
  PhaseDriver * phaseDriver() const {return phase_;}
  int phaseIndex() const {return phase_ != nullptr ? phase_->index() : -1;}

  // ---- injected driver ----
  // One control-period observation.  `st` may be nullptr (nothing received yet).
  void tick(double now_s, const ControllerStatus * st);
  // Simulated completion of a physical step that W3-R1 does not implement.
  bool phaseDone(double now_s, std::string * why);

  // ---- C4.3 S4 W3-R6: collision guard hook (dry-run mode) ----
  // When a guard is attached, tick() consults it on every ACTIVE-state
  // observation, against the controller-reported q_state.  All collision
  // geometry and every threshold live in CollisionGuard
  // (DualArmCollisionModel::distance); this class contains no collision math and
  // merely turns a !ok verdict into HOLD carrying the guard's OWN code
  // (collision_arm_arm / collision_object / collision_sweep).  The guard is NOT
  // owned; nullptr (the default) means "no collision guard", which is what the
  // real executor node builds.  A guard verdict is not a status field: nothing
  // here inspects ControllerStatus::failure_code to produce it.
  void setCollisionGuard(CollisionGuard * guard) {collision_guard_ = guard;}
  CollisionGuard * collisionGuard() const {return collision_guard_;}
  // Check 2 (object pose) needs an object frame the executor does not own, so it
  // is only consulted after setCollisionObjectPose().
  void setCollisionObjectPose(const dual_arm::Iso3 & object_pose);
  void clearCollisionObjectPose();
  bool hasCollisionObjectPose() const {return has_object_pose_;}
  // Number of guard consultations (arm-arm, once per active tick).  This is the
  // evidence that a guard failure code came from the guard call and not from a
  // ControllerStatus field.
  int collisionChecks() const {return collision_checks_;}
  // Fixed policy for the per-tick sweep check (check 3); within
  // [CollisionGuard::kMinSamples, CollisionGuard::kMaxSamples].
  static constexpr int kGuardSweepSamples = 8;

  // ---- observers ----
  State state() const { return state_; }
  const std::string & failureCode() const { return failure_code_; }
  std::uint32_t expectedHoldSeq() const { return expected_hold_seq_; }
  bool latched() const { return isLatched(state_); }
  State resumeState() const { return resume_state_; }
  int resumeFailures() const { return resume_fail_; }
  bool hasFrozenSnapshot() const { return has_frozen_; }
  const std::array<double, 12> & frozenQcmd() const { return frozen_q_cmd_; }
  double frozenTTraj() const { return frozen_t_traj_; }
  double stateSince() const { return state_since_; }
  const std::vector<Event> & events() const { return events_; }
  const Config & config() const { return cfg_; }

private:
  // C4.3 S4 W7: drives the attach client for GRASP / RELEASE.  Returns true when
  // it has taken over this tick (state is still latched or still waiting).
  bool tickWeld(double now_s);
  bool limitPressure(const ControllerStatus & st) const;
  int limit_streak_ = 0;
  void record(double now_s, const std::string & kind, const std::string & from,
              const std::string & to, const std::string & code, const std::string & detail);
  void advance(double now_s, const std::string & detail);
  void enterProtection(Protection p, const std::string & code, double now_s,
                       const std::string & detail);
  void checkTimeout(double now_s);
  bool heartbeatStale(double now_s) const;
  void captureFreeze(const ControllerStatus & st);
  // W3-R6: consult the guard against this observation.  Returns true when the
  // verdict latched a protection state (the caller then stops for this tick).
  bool checkCollisionGuard(double now_s, const ControllerStatus & st);

  Config cfg_;
  State state_ = State::Idle;
  State resume_state_ = State::Idle;
  std::string failure_code_;
  std::vector<Event> events_;
  ControllerStatus last_status_;
  bool has_status_ = false;
  double state_since_ = 0.0;
  double duration_ = 0.0;
  double no_status_since_ = -1.0;
  bool has_frozen_ = false;
  /// C7 review P1-3: the snapshot taken at entry may come from a status that
  /// predates the controller's freeze (status is 10 Hz, the freeze is per-tick).
  /// The first status observed *after* entering HOLD re-baselines the audit, so
  /// a one-frame-late freeze is not misreported as hold_invariant_violated.
  bool freeze_settled_ = false;
  std::uint32_t expected_hold_seq_ = 0;
  std::array<double, 12> frozen_q_cmd_{};
  double frozen_t_traj_ = 0.0;
  int resume_fail_ = 0;
  double last_hb_ = 0.0;
  bool hb_seen_ = false;
  PhaseDriver * phase_ = nullptr;  // W3-R2 task-phase driver (not owned)
  // W3-R6 dry-run collision guard (not owned).  nullptr = no guard.
  // C4.3 S4 W7: physical attach client (not owned).
  AttachClient * weld_ = nullptr;
  bool weld_attached_ = false;
  bool weld_release_done_ = false;
  // "Requested" is tracked here because AttachClient keeps its last action
  // (Confirmed is sticky by design): without this flag the startup guard's
  // confirmed *detach* would read as a confirmed attach.
  bool weld_attach_requested_ = false;
  bool weld_detach_requested_ = false;
  // S4-T16: see noteGripperClosed()/noteGraspCheck().  Reset by start().
  bool gripper_ready_ = false;
  bool grasp_checked_ = false;
  bool grasp_ok_ = false;

  CollisionGuard * collision_guard_ = nullptr;
  dual_arm::Iso3 object_pose_ = dual_arm::Iso3::Identity();
  bool has_object_pose_ = false;
  bool has_last_guard_q_ = false;
  std::array<double, 12> last_guard_q_{};
  int collision_checks_ = 0;
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__STATE_MACHINE_HPP_
