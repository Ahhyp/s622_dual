// C4.3 S4.1 W3-R1a-2 -- executor-side consumer of ~/hold_transport (C7-A Rev.2.1 P-2)
//
// WHY THIS EXISTS
//   The controller can only answer "did I queue your seq, and did I enter HOLD".
//   It cannot answer "is the trajectory frozen because of YOUR request".  That
//   judgement needs the joint triple (seq, state, hold_origin) and belongs here,
//   on the executor side, where a wrong answer is allowed to be conservative
//   instead of fatal.
//
// THE FOUR THINGS THAT MUST NOT BE CONFLATED (Gate, W3-R1a FAIL)
//   result=2          is NOT "not frozen"  -- it may mean "already HOLD".
//   ack timeout       is NOT success.
//   a historical ack  is NOT the current state -- every decision below gates on
//                     a status sample younger than `status_stale_s`.
//   origin conflict   is NOT L3/L4 -- a controller-origin HOLD is somebody
//                     else's safety action and must never be claimed as ours.
//
// The class is pure logic: no ROS, no time source of its own.  The caller passes
// `now_s` and the observed status, so every rule below is unit-testable without
// a bench, and the retry/timeout behaviour is evidence rather than narration.

#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__HOLD_REQUESTER_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__HOLD_REQUESTER_HPP_

#include <cstdint>
#include <string>

namespace dual_arm_transport_executor
{

struct HoldRequesterConfig
{
    /// Attempts (sends) before giving up.  A resend keeps the SAME seq: seq is the
    /// idempotency key, so a duplicate cannot be mistaken for a second request.
    int max_attempts = 3;
    /// Time to wait for the ack echo before resending.
    double ack_timeout_s = 0.3;
    /// A status sample older than this is not evidence of anything.
    double status_stale_s = 0.1;
};

struct HoldObservation
{
    double now_s = 0.0;
    bool have_status = false;
    double status_age_s = 0.0;
    std::string state;          ///< IDLE / RUNNING / HOLD / COMPLETE / ABORT / FAULT
    std::string hold_origin;    ///< "" / "controller" / "executor"
    std::uint32_t ack_seq = 0;  ///< TransportStatus.hold_request_seq
    std::uint8_t ack_result = 0;///< 0 none / 1 applied / 2 refused
    bool service_reply_received = false;
    bool service_accepted = false;
    std::string service_message;
};

enum class HoldAction
{
    Wait,    ///< nothing to do yet
    Resend,  ///< send the SAME seq again
    Done,    ///< L3/L4 established for OUR request
    Abort,   ///< give up: conservative terminal, never treated as success
};

struct HoldStep
{
    HoldAction action = HoldAction::Wait;
    std::uint32_t seq = 0;
    std::string reason;
};

class HoldRequester
{
public:
    explicit HoldRequester(HoldRequesterConfig cfg = HoldRequesterConfig{});

    /// Start a new request.  `seq` must be non-zero (0 is the protocol's
    /// "no request" sentinel); the caller owns allocation so it can keep the
    /// sequence monotonic across restarts.
    void begin(double now_s, std::uint32_t seq, const char *reason);

    /// Record the service reply.  `accepted=false` is retried (busy slot), it is
    /// not a failure of the hold itself.
    void onServiceReply(bool accepted, const char *message);

    /// Feed one observation and get the next action.
    HoldStep onObservation(const HoldObservation &obs);

    /// True when the caller should call the service now (before markSent()).
    bool needSend() const { return started_ && !done_ && !awaiting_reply_ && resend_pending_; }
    void markSent(double now_s);

    std::uint32_t seq() const { return seq_; }
    bool started() const { return started_; }
    bool done() const { return done_; }
    int attempts() const { return attempts_; }
    /// For the audit trail: how the request ended ("" while running).
    const std::string &outcome() const { return outcome_; }

private:
    HoldRequesterConfig cfg_;
    bool started_ = false;
    bool done_ = false;
    bool awaiting_reply_ = false;
    bool resend_pending_ = false;
    std::uint32_t seq_ = 0;
    int attempts_ = 0;
    double last_send_s_ = 0.0;
    std::string outcome_;
};

/// Wrap-safe "a is at or after b" for the 32-bit ack sequence space.
bool seqAtOrAfter(std::uint32_t a, std::uint32_t b);

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__HOLD_REQUESTER_HPP_
