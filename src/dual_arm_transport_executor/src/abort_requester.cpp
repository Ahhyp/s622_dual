// C4.3 S4 W3 -- AbortRequester implementation (see header for the contract).

#include "dual_arm_transport_executor/abort_requester.hpp"

#include <cstdio>

namespace dual_arm_transport_executor
{

AbortRequester::AbortRequester(AbortRequesterConfig cfg)
: cfg_(cfg)
{
#if defined(__cpp_exceptions)
  try {
#endif
    // Pre-size the two strings that the RT-adjacent path may touch so a resend
    // never has to grow a buffer while the control loop is waiting on it.
    reason_.reserve(96);
    outcome_.reserve(64);
#if defined(__cpp_exceptions)
  } catch (...) {
  }
#endif
}

void AbortRequester::begin(double now_s, const char * reason)
{
  const std::string why = (reason != nullptr && *reason != '\0')
    ? std::string("abort: ") + reason
    : std::string("abort_requested");
  reason_ = why;  // assignment into the reserved buffer: no growth for short codes
  if (started_) {
    // A second abort while one is in flight is the same intent: keep the
    // deadline and the confirmation state, but DO send again.  An operator
    // pressing abort twice must not be silently swallowed; ABORT is idempotent
    // on the controller (a plain pending_cmd_ store), and the attempt budget
    // still bounds the traffic.
    if (!confirmed_ && !escalated_) {
      resend_pending_ = true;
      awaiting_reply_ = false;
    }
    return;
  }
  started_ = true;
  begun_s_ = now_s;
  resend_pending_ = true;
  awaiting_reply_ = false;
  attempts_ = 0;
  saw_stale_or_missing_ = false;
  confirmed_ = false;
  escalated_ = false;
  outcome_.clear();
}

void AbortRequester::reset()
{
  started_ = false;
  confirmed_ = false;
  escalated_ = false;
  awaiting_reply_ = false;
  resend_pending_ = false;
  attempts_ = 0;
  begun_s_ = 0.0;
  last_send_s_ = 0.0;
  saw_stale_or_missing_ = false;
  reason_.clear();
  outcome_.clear();
}

void AbortRequester::onServiceReply(bool accepted, const char * message)
{
  (void)message;
  awaiting_reply_ = false;
  if (!accepted) {
    // Busy / unknown service: retry (bounded by max_attempts and the deadline).
    resend_pending_ = true;
    return;
  }
  // Accepted only means "queued on the next control cycle".  Never a result.
  resend_pending_ = false;
}

void AbortRequester::markSent(double now_s)
{
  ++attempts_;
  last_send_s_ = now_s;
  awaiting_reply_ = true;
  resend_pending_ = false;
}

AbortStep AbortRequester::onObservation(const AbortObservation & obs)
{
  AbortStep step;
  if (!started_ || confirmed_ || escalated_) {
    step.action = confirmed_ ? AbortAction::Confirmed
      : (escalated_ ? AbortAction::Escalate : AbortAction::Wait);
    step.reason = outcome_;
    return step;
  }

  const bool fresh = obs.have_status && obs.status_age_s <= cfg_.status_stale_s;
  if (!fresh) {
    saw_stale_or_missing_ = true;
  }

  // 1. Authoritative termination already observable?
  if (fresh) {
    if (obs.state == "ABORT") {
      confirmed_ = true;
      outcome_ = "confirmed_by_controller";
      step.action = AbortAction::Confirmed;
      step.reason = outcome_;
      return step;
    }
    if (obs.state == "COMPLETE") {
      // The trajectory ended before the abort landed.  Motion is stopped and
      // that is the fact we needed; reporting it as a failure would be wrong,
      // reporting it as "our abort worked" would be a lie -- hence its own tag.
      confirmed_ = true;
      outcome_ = "already_complete";
      step.action = AbortAction::Confirmed;
      step.reason = outcome_;
      return step;
    }
  }

  // 2. Deadline / attempts exhausted -> never claim safety.
  const double elapsed_s = obs.now_s - begun_s_;
  const bool out_of_attempts = attempts_ >= cfg_.max_attempts;
  const bool out_of_time = elapsed_s >= cfg_.confirm_timeout_s;
  if (out_of_attempts || out_of_time) {
    escalated_ = true;
    outcome_ = saw_stale_or_missing_ ? "abort_status_unknown" : "abort_not_confirmed";
    step.action = AbortAction::Escalate;
    step.reason = outcome_;
    return step;
  }

  // 3. Still within the window: (re)send if the previous attempt has settled.
  if (!awaiting_reply_) {
    resend_pending_ = true;
    step.action = AbortAction::Resend;
    step.reason = "no confirmation yet";
    return step;
  }

  step.action = AbortAction::Wait;
  return step;
}

}  // namespace dual_arm_transport_executor
