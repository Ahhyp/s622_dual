// C4.3 S4 D5-2 / R3 -- software-only GRASP/RELEASE attachment adapter.
//
// SCOPE (R3, SOFTWARE ONLY)
//   * Pure logic: no ROS, no clock, no actuator.  The caller passes `now_s`,
//     so the completion and timeout rules are unit-testable and reproducible.
//   * SIMULATED COMPLETION, EXPLICITLY LABELLED: tick() moves Requested to
//     AttachedSimulated.  No physical attachment is performed or claimed.
//     artifactTag() reports "simulated" in that state and never anything
//     stronger, so no report built on this adapter can be read as evidence of
//     a real grasp.
//
// STATE CONTRACT
//   Unattached  -- no request; request() is accepted.
//   Requested   -- accepted but not completed.  There is exactly ONE slot: a
//                  second request() is "attach_busy", never queued.  Only in
//                  this state do tick() and injectFailure() act.
//   AttachedSimulated -- simulated completion.  request() is "already_attached";
//                  the only exit is reset().
//   Failed      -- a timeout or an injected failure.  request() is allowed and
//                  clears the failure code, so a retry is explicit rather than
//                  implicit.
//
// NO ALLOCATION IN tick(): the timeout code is a static literal assigned by
// pointer; only an injected code is owned (std::string) on the object.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_ADAPTER_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_ADAPTER_HPP_

#include <string>

namespace dual_arm_transport_executor
{

enum class AttachState
{
  Unattached,
  Requested,
  AttachedSimulated,
  Failed,
};

struct AttachResult
{
  bool ok = false;
  std::string reason;  // "" on success
};

class AttachAdapter
{
public:
  AttachAdapter() = default;

  // Timeout for a Requested request.  Non-positive / non-finite values keep the
  // documented default so the wait cannot be disabled by accident.
  void configure(double request_timeout_s);

  void reset();

  // Unattached/Failed -> Requested.  Requested -> {false,"attach_busy"}.
  // AttachedSimulated -> {false,"already_attached"}.
  AttachResult request(double now_s);

  // Requested -> AttachedSimulated (simulated completion, deterministic), or
  // Requested -> Failed("attach_timeout") once strictly longer than the timeout.
  // No-op in any other state.
  void tick(double now_s);

  // Requested -> Failed(code) and records the code.  No-op anywhere else: no
  // state change, no record.
  void injectFailure(const std::string & code);

  AttachState state() const { return state_; }
  const std::string & failureCode() const;
  bool awaitingCompletion() const { return state_ == AttachState::Requested; }

  // Timestamp of the most recent accepted request (0.0 before one / after reset).
  double requestedAtS() const { return requested_at_s_; }

  // "simulated" once AttachedSimulated, otherwise "none".
  std::string artifactTag() const;

private:
  double request_timeout_s_ = 0.5;
  AttachState state_ = AttachState::Unattached;
  double requested_at_s_ = 0.0;
  // Set only to a static literal ("attach_timeout"), never to owned storage.
  const char * failure_literal_ = nullptr;
  // Owned copy: the caller's buffer may die right after injectFailure().
  std::string injected_failure_;
  // Built once at construction so the tick() timeout path allocates nothing.
  std::string timeout_code_ = "attach_timeout";
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_ADAPTER_HPP_
