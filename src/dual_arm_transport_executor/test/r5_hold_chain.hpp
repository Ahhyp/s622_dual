// C4.3 S4 W3-R6 -- R5 end-to-end hold chain harness (executor side, no ROS).
//
// Scope honesty (same rule as test_fault_chain.cpp): no ROS node, no clock, no
// DDS, no actuator.  What this file does model faithfully is the two halves of
// the HOLD request/acknowledge contract and the SAFETY STATE they leave behind,
// so that R5 can be tested without a real-time rig:
//
//   * ControllerHoldModel mirrors src/dual_arm_qp_controller's hold drain and
//     TransportStatus publication.  Every branch below is a line-for-line
//     restatement of that controller's code (quoted in the comments); it is a
//     transport-status model, not a second implementation of the transport:
//       - one request slot, "busy" while another request owns it;
//       - a replay of the seq that was already decided is idempotent and never
//         freezes twice, and its ack describes the CURRENT state;
//       - the ack is a persistent echo carried by every status sample;
//       - hold_origin distinguishes the executor's request from a controller
//         safety layer, and a refusal of the request does not move the freeze.
//   * ExecutorHoldChain is the executor's own side: the real
//     TransportStateMachine (which latches HOLD and audits its frozen state) plus
//     the real HoldRequester (retry / idempotency / timeout policies), wired the
//     way ExecutorNode wires them: status in -> tick -> request/retry.
//
// The two test-only links are named where they occur: the service reply is
// injected as a bool (the real path crosses ~/hold_transport), and the ack can
// be blacked out with setAckDelivery(false) to model a status window in which no
// published ack reached the executor.  Nothing else is faked: the state machine,
// the requester and the phase driver are the production classes.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__TEST__R5_HOLD_CHAIN_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__TEST__R5_HOLD_CHAIN_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "dual_arm_transport_executor/hold_requester.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"

namespace dual_arm_transport_executor
{
namespace test
{

// The controller half of the contract, restricted to what a HOLD request can
// observe: the request slot, the drain, the ack echo and the freeze.
class ControllerHoldModel
{
public:
  // dual_arm_qp_controller: kAckApplied / kAckRefused.
  static constexpr std::uint8_t kAckApplied = 1;
  static constexpr std::uint8_t kAckRefused = 2;

  // ~/start_transport acknowledged: RUNNING, trajectory at t=0.  q_cmd starts at
  // the first sample of the frozen trajectory and q_state follows it exactly
  // (this model has no tracking error).
  void startTransport(double duration_s)
  {
    state_ = "RUNNING";
    failure_code_ = "ok";
    hold_origin_.clear();
    active_ = true;
    running_ = true;
    duration_ = duration_s;
    t_traj_ = 0.0;
    for (std::size_t i = 0; i < 12; ++i) {
      q_cmd_[i] = 0.01 * static_cast<double>(i);
    }
    q_state_ = q_cmd_;
    queued_seq_ = 0;
    last_seq_ = 0;
    ack_seq_ = 0;
    ack_result_ = 0;
    request_freezes_ = 0;
  }

  // A controller-side safety layer stops the trajectory (origin "controller").
  // This is NOT an executor request and must never be counted as one.
  void holdForSafety(const std::string & code) {freeze(code, "controller");}

  // ~/resume_transport accepted.  Only valid from HOLD; FAULT/ABORT are not
  // resumable on the controller side either.
  bool resume()
  {
    if (state_ != "HOLD") {return false;}
    state_ = "RUNNING";
    failure_code_ = "ok";
    hold_origin_.clear();
    running_ = active_;
    return true;
  }

  // The controller's phase counter, used to provoke a real phase desync on the
  // executor side.  -1 == no trajectory (the executor then falls back to its
  // local clock, which is the mode R2 can run today).
  void setPhase(int index, const std::string & name)
  {
    phase_index_ = index;
    phase_name_ = name;
  }

  // One control period of the plant: only while the trajectory is running.
  void advancePlant(double dt)
  {
    if (!active_ || !running_) {return;}
    for (std::size_t i = 0; i < 12; ++i) {
      q_cmd_[i] += 0.001 * static_cast<double>(i + 1);
    }
    q_state_ = q_cmd_;
    t_traj_ = std::min(t_traj_ + dt, duration_);
    if (t_traj_ >= duration_) {running_ = false;}
  }

  // ~/hold_transport service call: the single slot.  Returns false when the
  // controller refuses the request object itself (seq 0 / slot busy), which is
  // distinct from an accepted request that the drain later refuses.
  bool requestHold(std::uint32_t seq, std::string * why)
  {
    if (seq == 0) {
      if (why != nullptr) {*why = "seq_zero";}
      return false;
    }
    if (queued_seq_ != 0) {
      if (why != nullptr) {*why = "busy";}
      return false;
    }
    queued_seq_ = seq;
    return true;
  }

  // The controller's update() drains the slot exactly once per period, BEFORE the
  // control law runs (dual_arm_qp_controller: applyPendingStart() then the hold
  // drain then the safety/control pipeline).  That ordering is why a queued hold
  // freezes q_cmd without one extra sample of motion.
  void drain()
  {
    const std::uint32_t seq = queued_seq_;
    if (seq == 0) {return;}
    queued_seq_ = 0;
    if (seq == last_seq_) {
      // Replay of an already-decided seq: never freeze again, and the ack
      // describes the CURRENT state (still stopped => applied, otherwise refused).
      const bool still_stopped = active_ && !running_;
      ack_result_ = still_stopped ? kAckApplied : kAckRefused;
    } else {
      const bool hold_ok = active_ && running_;
      if (hold_ok) {
        freeze("hold_by_executor", "executor");
        ++request_freezes_;
      }
      ack_result_ = hold_ok ? kAckApplied : kAckRefused;
      last_seq_ = seq;
    }
    ack_seq_ = seq;
  }

  // TransportStatus as the executor sees it.  hold_request_seq/result are the
  // ack echo; `deliver_ack_ == false` makes them read 0/0, i.e. this sample did
  // not carry the ack even though the controller had already applied the hold.
  ControllerStatus status(double now_s) const
  {
    ControllerStatus st;
    st.received = true;
    st.parse_ok = true;
    st.stamp_s = now_s;
    st.state = state_;
    st.failure_code = failure_code_;
    st.transport_active = active_;
    st.committed = running_;
    st.t_traj = t_traj_;
    st.duration = duration_;
    st.q_state = q_state_;
    st.q_cmd = q_cmd_;
    if (deliver_ack_) {
      st.hold_request_seq = ack_seq_;
      st.hold_request_result = ack_result_;
    }
    st.hold_origin = hold_origin_;
    st.phase_index = phase_index_;
    st.phase_name = phase_name_;
    return st;
  }

  // --- test-only link control -------------------------------------------
  void setAckDelivery(bool on) {deliver_ack_ = on;}
  bool ackDelivered() const {return deliver_ack_;}

  // --- observers --------------------------------------------------------
  const std::string & state() const {return state_;}
  const std::string & failureCode() const {return failure_code_;}
  const std::string & holdOrigin() const {return hold_origin_;}
  bool active() const {return active_;}
  bool running() const {return running_;}
  double tTraj() const {return t_traj_;}
  const std::array<double, 12> & qCmd() const {return q_cmd_;}
  std::uint32_t ackSeq() const {return ack_seq_;}
  std::uint8_t ackResult() const {return ack_result_;}
  std::uint32_t queuedSeq() const {return queued_seq_;}
  std::uint32_t lastSeq() const {return last_seq_;}
  // Number of times a REQUEST caused a freeze (never the controller's own).
  int requestFreezes() const {return request_freezes_;}

private:
  void freeze(const std::string & code, const char * origin)
  {
    state_ = "HOLD";
    failure_code_ = code;
    hold_origin_ = origin;
    running_ = false;
  }

  std::string state_ = "IDLE";
  std::string failure_code_ = "ok";
  std::string hold_origin_;
  bool active_ = false;
  bool running_ = false;
  double duration_ = 0.0;
  double t_traj_ = 0.0;
  std::array<double, 12> q_state_{};
  std::array<double, 12> q_cmd_{};
  std::uint32_t queued_seq_ = 0;
  std::uint32_t last_seq_ = 0;
  std::uint32_t ack_seq_ = 0;
  std::uint8_t ack_result_ = 0;
  int request_freezes_ = 0;
  int phase_index_ = -1;
  std::string phase_name_;
  bool deliver_ack_ = true;
};

// The executor half: the real state machine plus the real hold requester, wired
// the way the node wires them.  One call per control period, after the status
// sample for that period has been produced.
class ExecutorHoldChain
{
public:
  ExecutorHoldChain(TransportStateMachine & sm, ControllerHoldModel & ctl,
    HoldRequesterConfig cfg = HoldRequesterConfig())
  : sm_(sm), ctl_(ctl), hold_(cfg) {}

  void tick(double now_s, const ControllerStatus & st)
  {
    sm_.tick(now_s, &st);
    if (sm_.state() != State::Hold) {return;}

    // The first tick that sees HOLD starts the request: asking the controller to
    // freeze is part of latching the protection, not a later phase.
    if (!hold_.started()) {beginNewRequest(now_s, "executor_hold:" + sm_.failureCode());}
    if (hold_.done()) {return;}

    if (hold_.needSend()) {
      std::string why;
      last_request_accepted_ = ctl_.requestHold(hold_.seq(), &why);
      last_request_reply_ = last_request_accepted_ ? "queued" : why;
      hold_.onServiceReply(last_request_accepted_, last_request_reply_.c_str());
      if (last_request_accepted_) {hold_.markSent(now_s);}
    }

    HoldObservation obs;
    obs.now_s = now_s;
    obs.have_status = true;
    obs.status_age_s = 0.0;  // st was stamped now_s by the model
    obs.state = st.state;
    obs.hold_origin = st.hold_origin;
    obs.ack_seq = st.hold_request_seq;
    obs.ack_result = st.hold_request_result;
    last_step_ = hold_.onObservation(obs);

    if (last_step_.action == HoldAction::Done) {
      // Only a hold whose origin is ours (or the controller's own) is accepted,
      // and the requester's outcome string is what the report carries.
      confirmed_ = true;
      outcome_ = hold_.outcome();
    } else if (last_step_.action == HoldAction::Abort) {
      // The ack budget is exhausted: the executor must not pretend the plant is
      // frozen.  Record the failure as ABORT, keep the safety state, and never
      // claim "frozen".
      outcome_ = hold_.outcome();
      std::string why;
      sm_.abortTransport("hold_not_confirmed:" + last_step_.reason, now_s, &why);
    }
  }

  std::uint32_t beginNewRequest(double now_s, const std::string & reason)
  {
    const std::uint32_t seq = next_seq_++;
    hold_.begin(now_s, seq, reason.c_str());
    confirmed_ = false;
    outcome_.clear();
    return seq;
  }

  // --- observers --------------------------------------------------------
  bool confirmed() const {return confirmed_;}
  const std::string & outcome() const {return outcome_;}
  const HoldRequester & hold() const {return hold_;}
  const HoldStep & lastStep() const {return last_step_;}
  bool lastRequestAccepted() const {return last_request_accepted_;}
  const std::string & lastRequestReply() const {return last_request_reply_;}
  std::uint32_t nextSeq() const {return next_seq_;}

private:
  TransportStateMachine & sm_;
  ControllerHoldModel & ctl_;
  HoldRequester hold_;
  HoldStep last_step_;
  std::string outcome_;
  std::string last_request_reply_;
  bool last_request_accepted_ = false;
  bool confirmed_ = false;
  std::uint32_t next_seq_ = 1;
};

}  // namespace test
}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__TEST__R5_HOLD_CHAIN_HPP_
