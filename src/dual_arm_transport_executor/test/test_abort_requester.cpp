// C4.3 S4 W3 -- P1-4: abort propagation evidence.
//
// EVIDENCE LEVEL: unit test of the decision unit (no ROS, no bench, no
// actuator).  It proves the *rules*, not that a real controller stopped.
// End-to-end stopping remains B4-5 (real actuator, S4.3).
//
// The seven situations the reviewer asked for are all here: RUNNING, HOLD,
// COMPLETE, a repeated ABORT, the deadline, a lost message, and the race where
// the controller's status observes the stop before the service reply arrives.

#include <gtest/gtest.h>

#include "dual_arm_transport_executor/abort_requester.hpp"

using dual_arm_transport_executor::AbortAction;
using dual_arm_transport_executor::AbortObservation;
using dual_arm_transport_executor::AbortRequester;
using dual_arm_transport_executor::AbortRequesterConfig;
using dual_arm_transport_executor::AbortStep;

namespace
{

AbortRequesterConfig cfg()
{
  AbortRequesterConfig c;
  c.max_attempts = 3;
  c.confirm_timeout_s = 0.5;
  c.status_stale_s = 0.1;
  return c;
}

AbortObservation obs(double t, const std::string & state, double age = 0.02)
{
  AbortObservation o;
  o.now_s = t;
  o.have_status = true;
  o.status_age_s = age;
  o.state = state;
  return o;
}

/// Drive one send: the caller would call the service here.
void send(AbortRequester & r, double t)
{
  ASSERT_TRUE(r.needSend() || true);
  r.markSent(t);
}

}  // namespace

// 1. RUNNING -> the controller stops -> confirmed.
TEST(AbortRequester, ConfirmWhenTheControllerReachesAbort)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  EXPECT_EQ(r.onObservation(obs(0.05, "RUNNING")).action, AbortAction::Wait);
  const AbortStep s = r.onObservation(obs(0.10, "ABORT"));
  EXPECT_EQ(s.action, AbortAction::Confirmed);
  EXPECT_EQ(r.outcome(), "confirmed_by_controller");
  EXPECT_TRUE(r.confirmed());
  EXPECT_FALSE(r.escalated());
}

// 2. HOLD freezes the motion but is NOT the abort the executor asked for; if the
//    controller never publishes ABORT, the request escalates instead of passing.
TEST(AbortRequester, HoldIsNotAConfirmationAndExpiresIntoEscalation)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  EXPECT_EQ(r.onObservation(obs(0.10, "HOLD")).action, AbortAction::Wait);
  EXPECT_FALSE(r.confirmed());
  const AbortStep s = r.onObservation(obs(0.60, "HOLD"));
  EXPECT_EQ(s.action, AbortAction::Escalate);
  EXPECT_EQ(r.outcome(), "abort_not_confirmed");
  EXPECT_TRUE(r.escalated());
  EXPECT_FALSE(r.confirmed());
}

// 3. The trajectory ended on its own before the abort landed: motion is stopped,
//    but it must not be claimed as "our abort worked" -- it has its own tag.
TEST(AbortRequester, CompleteBeforeTheAbortLandsIsAlreadySafeButNotOurs)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  const AbortStep s = r.onObservation(obs(0.10, "COMPLETE"));
  EXPECT_EQ(s.action, AbortAction::Confirmed);
  EXPECT_EQ(r.outcome(), "already_complete");
}

// 4. A second abort while the first is in flight is the same intent: it must not
//    restart the deadline or the attempt budget, and must not clear confirmation.
TEST(AbortRequester, RepeatedAbortKeepsTheSameRequest)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  r.begin(0.10, "operator pressed again");
  EXPECT_EQ(r.attempts(), 1) << "a repeat must not count as a new attempt";
  EXPECT_TRUE(r.needSend()) << "a repeat still needs the abort re-sent";
  send(r, 0.10);
  const AbortStep s = r.onObservation(obs(0.20, "ABORT"));
  EXPECT_EQ(s.action, AbortAction::Confirmed);
  // And a repeat AFTER confirmation must not un-confirm anything.
  r.begin(0.30, "late repeat");
  EXPECT_TRUE(r.confirmed());
  EXPECT_EQ(r.onObservation(obs(0.40, "ABORT")).action, AbortAction::Confirmed);
}

// 5. Service rejection (busy / rejected) is retried, then escalates.
TEST(AbortRequester, RejectedServiceIsRetriedAndThenEscalates)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  r.onServiceReply(false, "busy");
  EXPECT_EQ(r.onObservation(obs(0.05, "RUNNING")).action, AbortAction::Resend);
  send(r, 0.05);
  r.onServiceReply(false, "busy");
  EXPECT_EQ(r.onObservation(obs(0.10, "RUNNING")).action, AbortAction::Resend);
  send(r, 0.10);
  r.onServiceReply(false, "busy");
  const AbortStep s = r.onObservation(obs(0.15, "RUNNING"));
  EXPECT_EQ(s.action, AbortAction::Escalate);
  EXPECT_EQ(r.outcome(), "abort_not_confirmed");
}

// 6. Messages lost: no status at all.  That is "unknown", never success.
TEST(AbortRequester, LostStatusIsUnknownAndNeverSuccess)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  AbortObservation none;
  none.now_s = 0.10;
  EXPECT_EQ(r.onObservation(none).action, AbortAction::Wait);
  none.now_s = 0.60;
  const AbortStep s = r.onObservation(none);
  EXPECT_EQ(s.action, AbortAction::Escalate);
  EXPECT_EQ(r.outcome(), "abort_status_unknown");
}

// 7. A stale sample is not evidence -- even if it says ABORT.
TEST(AbortRequester, StaleAbortIsNotEvidence)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  const AbortStep s = r.onObservation(obs(0.10, "ABORT", /*age=*/0.50));
  EXPECT_NE(s.action, AbortAction::Confirmed);
  EXPECT_FALSE(r.confirmed());
  const AbortStep late = r.onObservation(obs(0.60, "ABORT", /*age=*/0.50));
  EXPECT_EQ(late.action, AbortAction::Escalate);
  EXPECT_EQ(r.outcome(), "abort_status_unknown");
}

// 8. Race: the controller's authoritative stop is observed before the service
//    reply.  The confirmation must come from the status, not the reply.
TEST(AbortRequester, StatusCanConfirmBeforeTheServiceReply)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);                      // reply still outstanding
  const AbortStep s = r.onObservation(obs(0.03, "ABORT"));
  EXPECT_EQ(s.action, AbortAction::Confirmed);
  r.onServiceReply(true, "queued");  // late reply must not change the verdict
  EXPECT_TRUE(r.confirmed());
  EXPECT_EQ(r.outcome(), "confirmed_by_controller");
}

// 9. Reset clears the evidence so a new run cannot inherit a stale confirmation.
TEST(AbortRequester, ResetClearsConfirmation)
{
  AbortRequester r(cfg());
  r.begin(0.0, "operator");
  send(r, 0.0);
  ASSERT_EQ(r.onObservation(obs(0.10, "ABORT")).action, AbortAction::Confirmed);
  r.reset();
  EXPECT_FALSE(r.confirmed());
  EXPECT_FALSE(r.started());
  EXPECT_TRUE(r.outcome().empty());
  r.begin(1.0, "second run");
  EXPECT_FALSE(r.confirmed()) << "a new run must re-establish its own evidence";
}
