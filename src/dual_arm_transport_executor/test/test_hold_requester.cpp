// C4.3 S4.1 W3-R1a-2 -- P-2 joint judgement, retry and timeout (pure logic).
//
// These tests are the evidence for the rules the Gate called "阻断": each one
// pins a way the executor could wrongly conclude that the trajectory is frozen.

#include <gtest/gtest.h>

#include "dual_arm_transport_executor/hold_requester.hpp"

namespace {

using dual_arm_transport_executor::HoldAction;
using dual_arm_transport_executor::HoldObservation;
using dual_arm_transport_executor::HoldRequester;
using dual_arm_transport_executor::HoldRequesterConfig;
using dual_arm_transport_executor::seqAtOrAfter;

HoldObservation running(double now) {
    HoldObservation o;
    o.now_s = now;
    o.have_status = true;
    o.status_age_s = 0.01;
    o.state = "RUNNING";
    return o;
}

HoldRequesterConfig cfg() {
    HoldRequesterConfig c;
    c.max_attempts = 3;
    c.ack_timeout_s = 0.3;
    c.status_stale_s = 0.1;
    return c;
}

TEST(HoldRequesterC43S3, T_SeqComparisonSurvivesWrap) {
    EXPECT_TRUE(seqAtOrAfter(5u, 5u));
    EXPECT_TRUE(seqAtOrAfter(6u, 5u));
    EXPECT_FALSE(seqAtOrAfter(4u, 5u));
    // Across the uint32 wrap: 0 is the next value after 0xFFFFFFFF.
    EXPECT_TRUE(seqAtOrAfter(0u, 0xFFFFFFFFu));
    EXPECT_FALSE(seqAtOrAfter(0xFFFFFFFFu, 0u));
}

TEST(HoldRequesterC43S3, T_H8_BusyReplyIsRetriedWithTheSameSeq) {
    HoldRequester r(cfg());
    r.begin(0.0, 41, "test");
    ASSERT_TRUE(r.needSend());
    r.markSent(0.0);

    r.onServiceReply(false, "busy: hold request seq=7 is still queued");
    auto step = r.onObservation(running(0.01));
    EXPECT_EQ(step.action, HoldAction::Resend);
    EXPECT_EQ(step.seq, 41u);
    EXPECT_FALSE(r.done());
    r.markSent(0.01);

    // Second attempt succeeds and is acked.
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.02);
    obs.state = "HOLD";
    obs.hold_origin = "executor";
    obs.ack_seq = 41;
    obs.ack_result = 1;
    EXPECT_EQ(r.onObservation(obs).action, HoldAction::Done);
    EXPECT_EQ(r.seq(), 41u);
    EXPECT_EQ(r.attempts(), 2);
}

TEST(HoldRequesterC43S3, T_H16_LostAckIsNeverTreatedAsSuccess) {
    HoldRequester r(cfg());
    r.begin(0.0, 9, "test");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");

    // No ack echo yet: must NOT be Done, and before the timeout it just waits.
    EXPECT_EQ(r.onObservation(running(0.1)).action, HoldAction::Wait);
    EXPECT_FALSE(r.done());

    // Timeout -> resend the same seq.
    auto step = r.onObservation(running(0.35));
    EXPECT_EQ(step.action, HoldAction::Resend);
    EXPECT_EQ(step.reason, "ack_timeout");
    EXPECT_FALSE(r.done());
    r.markSent(0.35);

    // The ack finally arrives.
    auto obs = running(0.4);
    obs.state = "HOLD";
    obs.hold_origin = "executor";
    obs.ack_seq = 9;
    obs.ack_result = 1;
    EXPECT_EQ(r.onObservation(obs).action, HoldAction::Done);
    // Rule 1 (the triple) fires before the plain "applied" rule, which is the
    // point: the claim is that OUR request froze the trajectory.
    EXPECT_EQ(r.outcome(), "frozen_by_executor_request");
}

TEST(HoldRequesterC43S3, T_H16b_ExhaustedAttemptsAbortAndAreNeverDone) {
    HoldRequester r(cfg());
    r.begin(0.0, 3, "test");
    dual_arm_transport_executor::HoldStep step;
    for (int i = 0; i < 3; ++i) {
        r.markSent(0.0 + 0.5 * i);
        r.onServiceReply(true, "hold queued");   // queued, but the ack never comes
        step = r.onObservation(running(0.5 * i + 0.4));
    }
    EXPECT_EQ(step.action, HoldAction::Abort) << "attempts exhausted";
    EXPECT_EQ(step.reason, "hold_ack_timeout");
    EXPECT_FALSE(r.outcome() == "applied_and_frozen");
}

TEST(HoldRequesterC43S3, T_H4_RefusalWhileExecututorHoldsStillMeansFrozen) {
    HoldRequester r(cfg());
    r.begin(0.0, 12, "retry-after-drain");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.02);
    obs.state = "HOLD";
    obs.hold_origin = "executor";
    obs.ack_seq = 12;
    obs.ack_result = 2;   // refused: already HOLD
    const auto step = r.onObservation(obs);
    EXPECT_EQ(step.action, HoldAction::Done)
        << "result==2 while HOLD(executor) must not be read as 'not frozen'";
    EXPECT_EQ(r.outcome(), "frozen_by_executor_request");
}

TEST(HoldRequesterC43S3, T_H5_ControllerOriginHoldIsNeverClaimed) {
    HoldRequester r(cfg());
    r.begin(0.0, 13, "test");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.02);
    obs.state = "HOLD";
    obs.hold_origin = "controller";   // a safety action owns the hold
    obs.ack_seq = 13;
    obs.ack_result = 2;
    const auto step = r.onObservation(obs);
    // C4.3 S4 D3-a changed this from Abort to Done: the robot IS frozen (by a
    // safety layer), so aborting turned a safe state into a reported failure.
    // The attribution rule is unchanged -- it is never reported as OUR request.
    EXPECT_EQ(step.action, HoldAction::Done)
        << "a controller-origin HOLD must be adopted, not escalated to ABORT";
    EXPECT_EQ(r.outcome(), "frozen_by_controller");
    EXPECT_NE(r.outcome(), "frozen_by_executor_request")
        << "a safety-layer hold must never be claimed as the executor's own";
}

TEST(HoldRequesterC43S3, T_H12_StaleStatusProvesNothing) {
    HoldRequester r(cfg());
    r.begin(0.0, 21, "test");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.05);
    obs.state = "HOLD";
    obs.hold_origin = "executor";
    obs.ack_seq = 21;
    obs.ack_result = 1;
    obs.status_age_s = 0.5;           // older than status_stale_s
    EXPECT_EQ(r.onObservation(obs).action, HoldAction::Wait)
        << "a stale sample must not be used to declare the trajectory frozen";
    EXPECT_FALSE(r.done());
}

}  // namespace

// C7 independent review (P1-1): a safety layer can re-attribute the SAME tick's
// hold to itself, so the status the executor sees is
//   state=HOLD + ack_result=applied + hold_origin=controller
// The old code demanded origin=="executor" for result==1 and then fell into an
// unconditional Wait that bypassed the ack-timeout path entirely: a genuinely
// frozen robot left the executor waiting forever.  Success is the FREEZE, not
// the ownership.
TEST(HoldRequesterC43S3, T_H22_AppliedAckWithSafetyOriginIsStillASuccess) {
    HoldRequester r(cfg());
    r.begin(0.0, 14, "same-tick safety override");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.02);
    obs.state = "HOLD";
    obs.hold_origin = "controller";   // collision / D7 / E8 / E1 won the tick
    obs.ack_seq = 14;
    obs.ack_result = 1;               // applied: the plant IS frozen
    const auto step = r.onObservation(obs);
    EXPECT_EQ(step.action, HoldAction::Done)
        << "applied + HOLD must complete regardless of which layer owns the hold";
    EXPECT_EQ(r.outcome(), "applied_hold_owned_by_controller");
    EXPECT_NE(r.outcome(), "applied_and_frozen")
        << "the safety layer's hold must not be claimed as the executor's own";
}

// C7 review P1-1, second half: an applied ack whose state has not settled must
// still be bounded by ack_timeout -- not an unbounded Wait.
TEST(HoldRequesterC43S3, T_H23_AppliedButUnsettledIsBoundedByAckTimeout) {
    HoldRequester r(cfg());
    r.begin(0.0, 15, "ack before state");
    r.markSent(0.0);
    r.onServiceReply(true, "hold queued");
    auto obs = running(0.02);
    obs.state = "RUNNING";
    obs.ack_seq = 15;
    obs.ack_result = 1;               // "applied" but still RUNNING
    auto step = r.onObservation(obs);
    EXPECT_EQ(step.action, HoldAction::Wait);
    EXPECT_FALSE(r.done());
    // Past the timeout the requester must not keep waiting silently.
    obs.now_s = 0.40;                 // > ack_timeout_s (0.3)
    step = r.onObservation(obs);
    EXPECT_TRUE(step.action == HoldAction::Resend || step.action == HoldAction::Abort)
        << "an unsettled applied ack must reach the timeout/retry path";
}
