// C4.3 S4.1 W3-R1 — thin ROS adapter.  It owns communication and lifecycle and
// NOTHING else: every decision lives in TransportStateMachine (pure C++).
//
// Explicitly out of scope for W3-R1: Gazebo, real actuators, gripper actions,
// attach/detach.  The trajectory itself is started by the caller through the
// controller's existing ~/start_transport service; this node does not resend it.
#include "dual_arm_transport_executor/executor_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dual_arm_transport_executor
{

TransportExecutorNode::TransportExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("c43_s4_transport_executor", options)
{
  Config cfg;
  cfg.dt = declare_parameter("dt", cfg.dt);
  cfg.status_stale_timeout_s = declare_parameter("status_stale_timeout_s", cfg.status_stale_timeout_s);
  cfg.heartbeat_timeout_s = declare_parameter("heartbeat_timeout_s", cfg.heartbeat_timeout_s);
  cfg.approach_timeout_s = declare_parameter("approach_timeout_s", cfg.approach_timeout_s);
  cfg.grasp_timeout_s = declare_parameter("grasp_timeout_s", cfg.grasp_timeout_s);
  cfg.release_timeout_s = declare_parameter("release_timeout_s", cfg.release_timeout_s);
  cfg.phase_timeout_scale = declare_parameter("phase_timeout_scale", cfg.phase_timeout_scale);
  cfg.resume_pos_tol_m = declare_parameter("resume_pos_tol_m", cfg.resume_pos_tol_m);
  cfg.resume_rot_tol_rad = declare_parameter("resume_rot_tol_rad", cfg.resume_rot_tol_rad);
  cfg.resume_fail_max = declare_parameter("resume_fail_max", cfg.resume_fail_max);
  cfg.solve_fail_streak_limit = declare_parameter("solve_fail_streak_limit", cfg.solve_fail_streak_limit);
  sm_ = std::make_unique<TransportStateMachine>(cfg);

  artifacts_dir_ = declare_parameter<std::string>("artifacts_dir", "/tmp/c43_w3r1");
  run_id_ = declare_parameter<std::string>("run_id", "executor_run");
  trajectory_duration_s_ = declare_parameter("trajectory_duration_s", 5.1);
  heartbeat_period_s_ = declare_parameter("heartbeat_period_s", 0.02);

  // ---- C4.3 S4 W7: physical attach (Gazebo DetachableJoint) ----
  // Topic names are the ones the plugin logs at load time (W6-0 evidence); the
  // gz<->ros bridge in the W7 launch maps them onto these ROS topics.
  AttachClientConfig wcfg;
  wcfg.attach_topic = declare_parameter<std::string>("attach_topic", wcfg.attach_topic);
  wcfg.detach_topic = declare_parameter<std::string>("detach_topic", wcfg.detach_topic);
  wcfg.state_topic = declare_parameter<std::string>("weld_state_topic", wcfg.state_topic);
  wcfg.state_stale_s = declare_parameter("weld_state_stale_s", wcfg.state_stale_s);
  wcfg.transition_timeout_s =
    declare_parameter("weld_transition_timeout_s", wcfg.transition_timeout_s);
  wcfg.max_attach_attempts =
    declare_parameter("weld_max_attach_attempts", wcfg.max_attach_attempts);
  wcfg.startup_release_timeout_s =
    declare_parameter("weld_startup_release_timeout_s", wcfg.startup_release_timeout_s);
  wcfg.max_snap_m = declare_parameter("weld_max_snap_m", wcfg.max_snap_m);
  weld_.configure(wcfg);
  weld_parent_frame_ = declare_parameter<std::string>("weld_parent_frame", "left_grasp_frame");
  weld_child_frame_ = declare_parameter<std::string>(
    "weld_child_frame", "c43_sample::base_link");
  const std::string weld_pose_topic =
    declare_parameter<std::string>("weld_pose_topic", "/c43/weld_pose");

  // The controller publishes its status on ITS OWN private topic, so the topic
  // must be addressed explicitly rather than with this node's "~/".
  const std::string status_topic = declare_parameter<std::string>(
    "status_topic", "/dual_arm_qp_controller/transport_status");
  const std::string hb_topic = declare_parameter<std::string>(
    "controller_heartbeat_topic", "/dual_arm_qp_controller/heartbeat");

  const auto qos = rclcpp::QoS(10);
  status_sub_ = create_subscription<dual_arm_qp_controller::msg::TransportStatus>(
    status_topic, qos,
    [this](dual_arm_qp_controller::msg::TransportStatus::SharedPtr m) {onStatus(std::move(m));});

  hold_cli_ = create_client<dual_arm_qp_controller::srv::HoldTransport>(
    declare_parameter<std::string>("hold_service",
      "/dual_arm_qp_controller/hold_transport"));
  // P1-4: the executor decides ABORT, the controller performs the stop and owns
  // the authoritative state.  Hence a client for ~/abort_transport.
  abort_cli_ = create_client<dual_arm_qp_controller::srv::AbortTransport>(
    declare_parameter<std::string>("abort_service",
      "/dual_arm_qp_controller/abort_transport"));
  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
      std::string why;
      StartRequest req;
      req.duration_s = trajectory_duration_s_;
      r->success = sm_->start(nowSeconds(), req, &why);
      r->message = why.empty() ? std::string("accepted") : why;
      afterTransition();
    });
  abort_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/abort", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
      std::string why;
      const bool ok = sm_->abortTransport("abort_requested", nowSeconds(), &why);
      // P1-4: a local ABORT is a REQUEST.  Report exactly that; the confirmed
      // stop is reported later through the status/artifact trail.
      r->success = ok;
      r->message = ok
        ? std::string("abort requested; controller confirmation pending")
        : why;
      afterTransition();
    });
  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/resume", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
      std::string why;
      r->success = sm_->resume(nowSeconds(), &why);
      r->message = why.empty() ? std::string("resumed") : why;
      afterTransition();
    });
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
      std::string why;
      r->success = sm_->reset(nowSeconds(), &why);
      if (r->success) {abort_req_.reset(); abort_propagation_open_ = false;}
      r->message = why.empty() ? std::string("reset") : why;
      afterTransition();
    });

  heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>(hb_topic, qos);

  // The plugin subscribes to attach/detach (it does not advertise them, see W6-0),
  // so these publishers are the only way to (re)build or drop the weld.
  weld_attach_pub_ = create_publisher<std_msgs::msg::Empty>(wcfg.attach_topic, qos);
  weld_detach_pub_ = create_publisher<std_msgs::msg::Empty>(wcfg.detach_topic, qos);
  weld_state_sub_ = create_subscription<std_msgs::msg::String>(
    wcfg.state_topic, qos,
    [this](std_msgs::msg::String::SharedPtr m) {onWeldState(std::move(m));});
  weld_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    weld_pose_topic, rclcpp::QoS(10),
    [this](tf2_msgs::msg::TFMessage::SharedPtr m) {onPoses(std::move(m));});

  // Fortress 6.16 welds at startup whether we ask or not (W6-0), so the guard is
  // armed immediately; start() is refused until it reports a confirmed release.
  weld_.beginStartupGuard(nowSeconds());
  sm_->attachWeldClient(&weld_);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(cfg.dt), [this]() {onTimer();});
  hb_timer_ = create_wall_timer(
    std::chrono::duration<double>(heartbeat_period_s_), [this]() {
      // W3-R2 B3: publishing is not enough.  TransportStateMachine::heartbeatStale()
      // stays true until onHeartbeat() has been called at least once, and the first
      // active status would otherwise latch HOLD executor_lost and freeze the run.
      heartbeat_pub_->publish(std_msgs::msg::Empty());
      sm_->onHeartbeat(nowSeconds());
    });

  RCLCPP_INFO(get_logger(),
    "c43_s4_transport_executor ready: dt=%.4f s, stale=%.3f s, heartbeat=%.3f s, dry_run=%s",
    cfg.dt, cfg.status_stale_timeout_s, cfg.heartbeat_timeout_s, "true (no actuator writes)");
}

double TransportExecutorNode::nowSeconds() {return get_clock()->now().seconds();}

void TransportExecutorNode::onStatus(dual_arm_qp_controller::msg::TransportStatus::SharedPtr m)
{
  ControllerStatus & s = status_;
  s.received = true;
  s.parse_ok = true;
  s.stamp_s = nowSeconds();
  s.state = m->state;
  s.failure_code = m->failure_code;
  s.transport_active = m->active;
  s.committed = m->committed;
  s.t_traj = m->t_traj;
  s.duration = m->t_traj_duration;
  for (int i = 0; i < 12; ++i) {
    s.q_state[static_cast<size_t>(i)] = m->q_state[static_cast<size_t>(i)];
    s.q_cmd[static_cast<size_t>(i)] = m->q_cmd[static_cast<size_t>(i)];
    s.dq[static_cast<size_t>(i)] = m->dq[static_cast<size_t>(i)];
  }
  s.solve_fail_streak = m->solve_fail_streak;
  s.active_limit_count = m->active_limit_count;
  s.e_obj_pos_m = m->e_obj_pos_m;
  s.e_obj_rot_rad = m->e_obj_rot_rad;
  s.sigma_min_stack = m->sigma_min_stack;
  s.executor_heartbeat_age_s = m->executor_heartbeat_age_s;
  s.executor_heartbeat_seen = m->executor_heartbeat_seen;
  s.executor_heartbeat_required = m->executor_heartbeat_required;
  s.hold_origin = m->hold_origin;
  s.hold_request_seq = m->hold_request_seq;
  s.hold_request_result = m->hold_request_result;
  s.phase_index = m->phase_index;
  s.phase_count = m->phase_count;
  s.phase_name = m->phase_name;
  s.phase_elapsed_s = m->phase_elapsed_s;
  s.phase_duration_s = m->phase_duration_s;
  learnPhaseTable(*m);
}

// W3-R2 B1: the controller builds its phase table from the start_transport
// request, so the executor cannot know it up front.  It is learned here, slot by
// slot, from the status stream; the driver is attached once every slot is known.
// A status without phase fields (phase_count == 0 / phase_index < 0) leaves the
// table untouched and the state machine keeps using the local trajectory clock.
void TransportExecutorNode::learnPhaseTable(
  const dual_arm_qp_controller::msg::TransportStatus & m)
{
  if (m.phase_count <= 0 || m.phase_index < 0) {
    return;
  }
  if (phase_table_count_ != m.phase_count) {
    phase_table_count_ = m.phase_count;
    phase_table_known_ = 0;
    phase_driver_attached_ = false;
    sm_->attachPhaseDriver(nullptr);
    if (!phase_driver_.setPhaseCount(m.phase_count)) {
      RCLCPP_ERROR(get_logger(), "controller reports phase_count=%d; executor cannot size its "
        "phase table (shrinking tables are a contract violation)", m.phase_count);
      phase_table_count_ = 0;
      return;
    }
  }
  const int idx = m.phase_index;
  if (idx < phase_table_count_ && phase_driver_.phases()[static_cast<std::size_t>(idx)].name.empty()) {
    if (!phase_driver_.setPhase(idx, m.phase_name, m.phase_duration_s)) {
      RCLCPP_WARN(get_logger(), "phase slot %d: cannot learn '%s' (%.3f s); keeping the local clock",
        idx, m.phase_name.c_str(), m.phase_duration_s);
      return;
    }
    ++phase_table_known_;
  }
  if (!phase_driver_attached_ && phase_table_known_ >= phase_table_count_) {
    sm_->attachPhaseDriver(&phase_driver_);
    phase_driver_attached_ = true;
    RCLCPP_INFO(get_logger(), "phase table learned from the controller (%d phases, total %.3f s); "
      "auditing the external phase index", phase_table_count_, phase_driver_.totalDuration());
  }
}

void TransportExecutorNode::onTimer()
{
  // W7: advance the attach client FIRST, so that a request produced by this tick
  // is published exactly once and the state machine sees the resulting action.
  driveWeld();
  sm_->tick(nowSeconds(), status_.received ? &status_ : nullptr);
  afterTransition();
  driveAbortPropagation();
  driveHoldRequest();
}

void TransportExecutorNode::beginHoldRequest(const std::string & reason)
{
  if (hold_request_open_ || hold_confirmed_) {return;}
  hold_request_open_ = true;
  next_hold_seq_ = (next_hold_seq_ == 0) ? 1 : next_hold_seq_;
  const std::uint32_t seq = next_hold_seq_++;
  // The state machine must know which ack answers THIS request, otherwise an
  // old applied ack would re-baseline its freeze audit (P1-3 isolation).
  sm_->expectHoldAck(seq);
  hold_req_.begin(nowSeconds(), seq, reason.c_str());
  RCLCPP_WARN(get_logger(), "requesting controller HOLD (seq=%u): %s",
    static_cast<unsigned>(hold_req_.seq()), reason.c_str());
}

// Pump the P-2 state machine.  All judgement lives in HoldRequester (unit tested);
// this function only moves messages.
void TransportExecutorNode::driveHoldRequest()
{
  // The state machine entering HOLD is the trigger: from then on the executor
  // insists that the controller freeze, and escalates if it cannot confirm it.
  if (sm_->state() == State::Hold && !hold_request_open_ && !hold_confirmed_) {
    beginHoldRequest(sm_->failureCode());
  }
  if (!hold_request_open_) {return;}

  const int reply = hold_reply_.exchange(0);
  if (reply == 1) {hold_req_.onServiceReply(true, "ok");}
  else if (reply == 2) {hold_req_.onServiceReply(false, "busy");}

  HoldObservation obs;
  obs.now_s = nowSeconds();
  obs.have_status = status_.received;
  obs.status_age_s = status_.received ? (obs.now_s - status_.stamp_s) : 1.0e9;
  obs.state = status_.state;
  obs.hold_origin = status_.hold_origin;
  obs.ack_seq = status_.hold_request_seq;
  obs.ack_result = status_.hold_request_result;

  const HoldStep step = hold_req_.onObservation(obs);
  if (step.action == HoldAction::Resend || hold_req_.needSend()) {
    if (!hold_cli_->service_is_ready()) {return;}
    auto req = std::make_shared<dual_arm_qp_controller::srv::HoldTransport::Request>();
    req->reason = sm_->failureCode();
    req->seq = hold_req_.seq();
    hold_req_.markSent(obs.now_s);
    hold_cli_->async_send_request(req, [this](rclcpp::Client<
        dual_arm_qp_controller::srv::HoldTransport>::SharedFuture f) {
        hold_reply_.store(f.get()->accepted ? 1 : 2);
      });
    return;
  }
  if (step.action == HoldAction::Done) {
    hold_confirmed_ = true;
    hold_request_open_ = false;
    // D3-a: distinguish "the controller froze because of MY request" from
    // "the controller was already frozen by a safety layer".  Both are safe,
    // only the first one may be reported as our own action.
    if (hold_req_.outcome() == "frozen_by_controller") {
      RCLCPP_WARN(get_logger(),
        "controller already HOLD (origin=controller): adopting the safety hold, "
        "not claiming it as ours (seq=%u)", static_cast<unsigned>(hold_req_.seq()));
    } else {
      RCLCPP_INFO(get_logger(), "controller HOLD confirmed: %s (seq=%u, attempts=%d)",
        hold_req_.outcome().c_str(), static_cast<unsigned>(hold_req_.seq()),
        hold_req_.attempts());
    }
    return;
  }
  if (step.action == HoldAction::Abort) {
    // Conservative terminal.  Never reported as "frozen".
    hold_request_open_ = false;
    std::string why;
    (void)sm_->abortTransport("hold_not_confirmed:" + step.reason, obs.now_s, &why);
    beginAbortPropagation(std::string("hold_not_confirmed:") + step.reason);
    RCLCPP_ERROR(get_logger(), "controller HOLD NOT confirmed (%s): %s",
      step.reason.c_str(), why.c_str());
  }
}

void TransportExecutorNode::beginAbortPropagation(const std::string & reason)
{
  if (abort_propagation_open_ || abort_req_.confirmed()) {return;}
  abort_propagation_open_ = true;
  abort_req_.begin(nowSeconds(), reason.c_str());
  RCLCPP_WARN(get_logger(), "propagating ABORT to the controller: %s", reason.c_str());
}

// P1-4: pump the AbortRequester.  It owns every judgement (confirmation,
// retries, deadline, escalation); this function only moves messages.  Nothing
// here may report "aborted safely" before Confirmed.
void TransportExecutorNode::driveAbortPropagation()
{
  const State st = sm_->state();
  if (!abort_propagation_open_ && !abort_req_.started() &&
      (st == State::Abort))
  {
    beginAbortPropagation(sm_->failureCode());
  }
  if (!abort_req_.started() || abort_req_.confirmed()) {return;}

  const int reply = abort_reply_.exchange(0);
  if (reply == 1) {abort_req_.onServiceReply(true, "queued");}
  else if (reply == 2) {abort_req_.onServiceReply(false, "rejected");}

  AbortObservation obs;
  obs.now_s = nowSeconds();
  obs.have_status = status_.received;
  obs.status_age_s = status_.received ? (obs.now_s - status_.stamp_s) : 1.0e9;
  obs.state = status_.state;

  const AbortStep step = abort_req_.onObservation(obs);
  if (step.action == AbortAction::Resend || abort_req_.needSend()) {
    if (!abort_cli_->service_is_ready()) {return;}
    auto req = std::make_shared<dual_arm_qp_controller::srv::AbortTransport::Request>();
    req->reason = abort_req_.reason();
    abort_req_.markSent(obs.now_s);
    abort_cli_->async_send_request(req, [this](rclcpp::Client<
        dual_arm_qp_controller::srv::AbortTransport>::SharedFuture f) {
        abort_reply_.store(f.get()->accepted ? 1 : 2);
      });
    return;
  }
  if (step.action == AbortAction::Confirmed) {
    RCLCPP_WARN(get_logger(), "controller stop confirmed (%s) after %d attempt(s)",
      abort_req_.outcome().c_str(), abort_req_.attempts());
    abort_propagation_open_ = false;
    return;
  }
  if (step.action == AbortAction::Escalate) {
    // Never claim safety.  Escalate to a latching safety fault with its own code.
    abort_propagation_open_ = false;
    std::string why;
    (void)sm_->escalateFault(abort_req_.outcome(), obs.now_s,
      "controller did not confirm the ABORT", &why);
    RCLCPP_ERROR(get_logger(),
      "ABORT NOT CONFIRMED by the controller (%s); escalating to a safety fault",
      abort_req_.outcome().c_str());
  }
}

void TransportExecutorNode::afterTransition()
{
  const State st = sm_->state();
  if (st != last_state_) {
    RCLCPP_INFO(get_logger(), "state %s -> %s (code '%s')",
      toString(last_state_), toString(st), sm_->failureCode().c_str());
    last_state_ = st;
  }
  if (artifacts_written_) {return;}
  if (st == State::Complete || st == State::Fault || st == State::Abort) {
    // Non-realtime context only: file IO must never run in the control loop.
    writeArtifacts();
    artifacts_written_ = true;
  }
}

void TransportExecutorNode::writeArtifacts()
{
  ArtifactWriter w;
  std::string err;
  const std::string dir = artifacts_dir_ + "/" + run_id_;
  if (!w.open(dir, &err)) {
    RCLCPP_ERROR(get_logger(), "cannot open artifact dir: %s", err.c_str());
    return;
  }
  Manifest m;
  m.run_id = run_id_;
  m.git_head = C43_GIT_HEAD;
  m.package_version = "0.1.0";
  m.build_type = C43_BUILD_TYPE;
  m.mode = "bench";
  m.env_class = "host";
  m.input_config_sha256 = "unset";
  m.started_utc = "1970-01-01T00:00:00Z";
  (void)w.writeManifest(m, &err);
  (void)w.writeEvents(sm_->events(), &err);
  (void)w.writeCycles(cycles_, &err);
  (void)w.writeSummary(
    buildSummaryJson(run_id_,
      sm_->state() == State::Complete ? "COMPLETE" : sm_->failureCode(),
      sm_->events(), cycles_, {}), &err);
  (void)w.writeCommands({"c43_s4_executor"}, &err);
  (void)w.copyTestResults("", &err);
  RCLCPP_INFO(get_logger(), "artifacts written to %s", dir.c_str());
}

namespace {
// q_rel = conj(q_parent) * q_child, normalised, written as x y z w.
void relativeQuat(const double * qp, const double * qc, double * out)
{
  const double px = -qp[0], py = -qp[1], pz = -qp[2], pw = qp[3];
  const double cx = qc[0], cy = qc[1], cz = qc[2], cw = qc[3];
  const double w = pw * cw - px * cx - py * cy - pz * cz;
  const double x = pw * cx + px * cw + py * cz - pz * cy;
  const double y = pw * cy - px * cz + py * cw + pz * cx;
  const double z = pw * cz + px * cy - py * cx + pz * cw;
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n > 0.0) {
    out[0] = x / n; out[1] = y / n; out[2] = z / n; out[3] = w / n;
  } else {
    out[0] = 0.0; out[1] = 0.0; out[2] = 0.0; out[3] = 1.0;
  }
}

// Angle between two orientations, in radians: 2*acos(|dot|) with unit quaternions.
double quatAngle(const double * a, const double * b)
{
  const double d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  const double c = d > 1.0 ? 1.0 : d;
  return 2.0 * std::acos(c);
}
}  // namespace

void TransportExecutorNode::driveWeld()
{
  const double now = nowSeconds();
  // Two calls, two jobs (P0-1): tick() advances deadlines/decisions, consumeSend()
  // returns the armed Request* exactly once and is what publishes it.  Publishing
  // off tick()'s return value left `sent_` false forever, so the retry cadence
  // never re-armed and a single dropped message failed the startup guard.
  weld_.tick(now);
  const WeldAction a = weld_.consumeSend(now);
  if (a == WeldAction::RequestAttach) {
    // Snapshot the parent->child vector immediately before the weld is created:
    // the snap is the CHANGE of that vector across the attach, so a measurement
    // taken only afterwards would prove nothing.
    have_pre_rel_ = have_parent_pose_ && have_child_pose_;
    if (have_pre_rel_) {
      for (int i = 0; i < 3; ++i) {pre_rel_[i] = child_pos_[i] - parent_pos_[i];}
    }
    have_pre_rel_q_ = have_parent_pose_ && have_child_pose_;
    if (have_pre_rel_q_) {relativeQuat(parent_quat_, child_quat_, pre_rel_q_);}
    weld_attach_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_INFO(get_logger(), "weld: attach requested (attempt %d)", weld_.attempts());
  } else if (a == WeldAction::RequestDetach) {
    weld_detach_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_INFO(get_logger(), "weld: detach requested");
  } else if (weld_.failed() && !weld_failure_logged_) {
    // Once: a terminal weld failure is reported every 8 ms otherwise, which buries
    // every other line in the log and makes the first failure hard to find.
    weld_failure_logged_ = true;
    RCLCPP_ERROR(get_logger(), "weld: %s (%s)", weld_.outcome().c_str(),
      weld_.reason().c_str());
  }
  if (weld_.startupVerified() && !startup_logged_) {
    startup_logged_ = true;
    RCLCPP_INFO(get_logger(),
      "weld: startup weld released and confirmed -- the executor may now start");
  }
}

void TransportExecutorNode::onWeldState(std_msgs::msg::String::SharedPtr m)
{
  const std::string & v = m->data;
  WeldState st = WeldState::Unknown;
  if (v == "attached") {st = WeldState::Attached;}
  else if (v == "detached") {st = WeldState::Detached;}
  weld_.observe(st, nowSeconds());
  if (st == WeldState::Attached && have_pre_rel_) {
    const double rel[3] = {
      child_pos_[0] - parent_pos_[0],
      child_pos_[1] - parent_pos_[1],
      child_pos_[2] - parent_pos_[2]};
    double d2 = 0.0;
    for (int i = 0; i < 3; ++i) {const double d = rel[i] - pre_rel_[i]; d2 += d * d;}
    // Rotation jump across the attach (§14.3 C3: <= 0.5 deg).  The criterion is on
    // the CHANGE of the parent->child orientation, so both relative quaternions are
    // compared; a translation-only check would accept a weld that spun the sample.
    double rot_rad = std::nan("");
    if (have_pre_rel_q_) {
      double rel_q_now[4];
      relativeQuat(parent_quat_, child_quat_, rel_q_now);
      rot_rad = quatAngle(pre_rel_q_, rel_q_now);
    }
    weld_.setMeasuredSnap(std::sqrt(d2), rot_rad);
  }
}

void TransportExecutorNode::onPoses(tf2_msgs::msg::TFMessage::SharedPtr m)
{
  for (const auto & t : m->transforms) {
    const std::string & f = t.child_frame_id;
    if (f == weld_parent_frame_) {
      parent_pos_[0] = t.transform.translation.x;
      parent_pos_[1] = t.transform.translation.y;
      parent_pos_[2] = t.transform.translation.z;
      parent_quat_[0] = t.transform.rotation.x;
      parent_quat_[1] = t.transform.rotation.y;
      parent_quat_[2] = t.transform.rotation.z;
      parent_quat_[3] = t.transform.rotation.w;
      have_parent_pose_ = true;
    } else if (f == weld_child_frame_) {
      child_pos_[0] = t.transform.translation.x;
      child_pos_[1] = t.transform.translation.y;
      child_pos_[2] = t.transform.translation.z;
      child_quat_[0] = t.transform.rotation.x;
      child_quat_[1] = t.transform.rotation.y;
      child_quat_[2] = t.transform.rotation.z;
      child_quat_[3] = t.transform.rotation.w;
      have_child_pose_ = true;
    }
  }
}

}  // namespace dual_arm_transport_executor
