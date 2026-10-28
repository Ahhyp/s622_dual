// C4.3 S4 W3 -- executor-side propagation of a task ABORT to the controller.
//
// WHY THIS EXISTS (P1-4)
//   The executor decides that a task must ABORT.  The CONTROLLER owns motion
//   termination and the authoritative state.  Before this unit existed the
//   executor only wrote ABORT into its own state machine: the controller kept
//   running until the trajectory ended (the heartbeat watchdog cannot fire while
//   the executor is alive and still ticking), so "executor says ABORT" and
//   "robot stopped" were two different facts.
//
// THE RULE THIS CLASS ENFORCES
//   A local ABORT is a request, never a result.  Safe termination may be
//   reported ONLY after the controller's authoritative status says the motion
//   stopped (ABORT, or COMPLETE when the trajectory ended on its own first).
//   Everything else -- rejection, timeout, missing/stale status, an unknown
//   state string -- ends in Escalate, which the caller must turn into a latching
//   safety fault.  Escalate is never success.
//
// Deliberately pure logic: no ROS, no clock of its own.  The caller passes
// `now_s` and the observed status so every rule below is unit-testable.

#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__ABORT_REQUESTER_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__ABORT_REQUESTER_HPP_

#include <string>

namespace dual_arm_transport_executor
{

struct AbortRequesterConfig
{
    /// Sends of ~/abort_transport before giving up.  The service is idempotent
    /// (pending_cmd_ is a plain store), so resending is safe.
    int max_attempts = 3;
    /// The controller applies the abort on its next control cycle and publishes
    /// status at ~10 Hz, so 0.5 s is several status periods.
    double confirm_timeout_s = 0.5;
    /// A status sample older than this is not evidence of anything.
    double status_stale_s = 0.1;
};

struct AbortObservation
{
    double now_s = 0.0;
    bool have_status = false;
    double status_age_s = 0.0;
    std::string state;            ///< IDLE / RUNNING / HOLD / COMPLETE / ABORT / FAULT
    bool service_reply_received = false;
    bool service_accepted = false;
    std::string service_message;
};

enum class AbortAction
{
    Wait,       ///< nothing to do yet
    Resend,     ///< send the SAME abort again
    Confirmed,  ///< the controller's authoritative state says motion stopped
    Escalate,   ///< never assume safety: caller must latch a safety fault
};

struct AbortStep
{
    AbortAction action = AbortAction::Wait;
    std::string reason;
};

class AbortRequester
{
public:
    explicit AbortRequester(AbortRequesterConfig cfg = AbortRequesterConfig{});

    /// Start propagating `reason`.  Calling it again -- e.g. a second operator
    /// abort while confirmation is pending -- keeps the same request alive and
    /// does NOT reset the confirmation state.
    void begin(double now_s, const char * reason);

    /// Clear the request (used by an explicit operator reset).  Confirmation is
    /// NOT carried across a reset: a new run must re-establish its own evidence.
    void reset();

    /// Record the service reply.  `accepted=false` is retried, not fatal.
    void onServiceReply(bool accepted, const char * message);

    /// Feed one observation and get the next action.
    AbortStep onObservation(const AbortObservation & obs);

    /// True when the caller should call the service now (before markSent()).
    bool needSend() const { return started_ && !confirmed_ && !escalated_ && !awaiting_reply_ && resend_pending_; }
    void markSent(double now_s);

    bool started() const { return started_; }
    bool confirmed() const { return confirmed_; }
    bool escalated() const { return escalated_; }
    int attempts() const { return attempts_; }
    double elapsed(double now_s) const { return started_ ? now_s - begun_s_ : 0.0; }
    /// "confirmed_by_controller" / "already_complete" / "abort_not_confirmed" /
    /// "abort_status_unknown" ("" while in flight).
    const std::string & outcome() const { return outcome_; }
    const std::string & reason() const { return reason_; }

private:
    AbortRequesterConfig cfg_;
    bool started_ = false;
    bool confirmed_ = false;
    bool escalated_ = false;
    bool awaiting_reply_ = false;
    bool resend_pending_ = false;
    int attempts_ = 0;
    double begun_s_ = 0.0;
    double last_send_s_ = 0.0;
    bool saw_stale_or_missing_ = false;
    std::string reason_;
    std::string outcome_;
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__ABORT_REQUESTER_HPP_
