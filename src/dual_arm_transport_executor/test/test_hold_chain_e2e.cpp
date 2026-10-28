// C4.3 S4 W3-R6 -- R5 end-to-end fault chain, executor side (in-process).
//
// What is exercised here: the whole HOLD request path as the executor runs it --
// TransportStateMachine (real) -> HoldRequester (real) -> request slot -> drain
// -> ack echo -> next status sample -- against a transport-status model that
// restates dual_arm_qp_controller's drain and publication semantics one line at a
// time (see test/r5_hold_chain.hpp).  Every assertion checks a SAFETY STATE
// (controller state/origin/frozen q_cmd/t_traj, executor state/failure_code/
// frozen snapshot), never merely "an ack message arrived":
//
//   * an ack that was published but not delivered is not success, and a re-send
//     of the same seq is idempotent (the plant is not frozen twice);
//   * a NEW seq is accepted by the request slot even while the plant is already
//     frozen, and the replay still answers "still applied" without a new freeze;
//   * a duplicate seq replay is idempotent, and the same replay after a resume is
//     NOT "frozen" -- the executor must not claim a freeze it does not have (and
//     the state machine faults if the plant really moved under it);
//   * an exhausted ack budget records the failure (ABORT hold_not_confirmed:...)
//     instead of claiming success, while the controller really is frozen;
//   * a controller-origin freeze is never re-attributed to the executor's
//     request, and an executor-origin freeze is attributed to it.
//
// Scope honesty: no ROS, no DDS, no clock, no actuator.  This is evidence class
// "in-process end-to-end"; it is NOT a real-time result and NOT the S4-T21
// hardware verdict.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include "dual_arm_transport_executor/hold_requester.hpp"
#include "dual_arm_transport_executor/phase_driver.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "r5_hold_chain.hpp"

using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::HoldAction;
using dual_arm_transport_executor::HoldObservation;
using dual_arm_transport_executor::HoldRequester;
using dual_arm_transport_executor::HoldRequesterConfig;
using dual_arm_transport_executor::HoldStep;
using dual_arm_transport_executor::PhaseDriver;
using dual_arm_transport_executor::PhaseSpec;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::State;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::test::ControllerHoldModel;
using dual_arm_transport_executor::test::ExecutorHoldChain;

namespace
{

constexpr double kDt = 0.008;
constexpr double kTrajectoryDuration = 4.80;

std::vector<PhaseSpec> fourPhaseTrajectory()
{
  return {
    {"approach", 0.30},
    {"lift", 1.50},
    {"transport", 1.50},
    {"lower", 1.50},
  };
}

struct Rig
{
  ControllerHoldModel ctl;
  PhaseDriver driver{fourPhaseTrajectory()};
  TransportStateMachine sm;
  double now = 0.0;
  std::string why;
  bool started = false;

  Rig()
  {
    // 1. The controller is IDLE: start_transport is allowed by the precheck.
    ControllerStatus st = ctl.status(now);
    sm.tick(now, &st);
    sm.attachPhaseDriver(&driver);
    StartRequest req;
    req.duration_s = kTrajectoryDuration;
    started = sm.start(now, req, &why);
    // 2. The controller acknowledges start_transport and starts the trajectory.
    ctl.startTransport(kTrajectoryDuration);
    st = ctl.status(now);
    sm.tick(now, &st);
  }

  // One control period, in the controller's own order: drain a queued hold, then
  // run the plant, then publish.  A request queued by the executor during period
  // k is therefore decided at the start of period k+1, before the plant can move
  // again -- the same window the real controller has.
  void period(ExecutorHoldChain & chain)
  {
    now += kDt;
    ctl.drain();
    ctl.advancePlant(kDt);
    chain.tick(now, ctl.status(now));
  }

  // The period in which the controller reports a phase jump.  The executor
  // latches HOLD and issues the hold request in the SAME callback, so its frozen
  // snapshot matches a controller that has not frozen yet (a request sent one
  // period later would be a real invariant violation, not a test artifact).
  void desyncPeriod(ExecutorHoldChain & chain)
  {
    now += kDt;
    ctl.setPhase(2, "transport");
    ctl.drain();
    ctl.advancePlant(kDt);
    chain.tick(now, ctl.status(now));
  }
};

}  // namespace

TEST(HoldChainE2E, R5E2E_AckLossThenNewSeqRequestIsAcceptedAndFreezeIsNotRepeated)
{
  Rig r;
  ExecutorHoldChain chain(r.sm, r.ctl);
  ASSERT_TRUE(r.started) << r.why;
  ASSERT_EQ(r.sm.state(), State::Approach);

  // The ack is published by the controller but does not reach the executor in
  // this window (blacked out at the link, not at the controller).
  r.ctl.setAckDelivery(false);
  r.desyncPeriod(chain);
  const double send_time = r.now;

  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_EQ(r.sm.failureCode(), "phase_desync");
  ASSERT_EQ(chain.hold().seq(), 1u);
  EXPECT_TRUE(chain.lastRequestAccepted()) << chain.lastRequestReply();
  EXPECT_EQ(r.ctl.queuedSeq(), 1u) << "the request is in the controller's slot";

  // Next period: the controller applies the hold and publishes ack (1, applied).
  // The sample the executor receives carries no ack at all.
  r.period(chain);
  ASSERT_EQ(r.ctl.state(), "HOLD");
  ASSERT_EQ(r.ctl.holdOrigin(), "executor");
  ASSERT_EQ(r.ctl.ackSeq(), 1u);
  ASSERT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckApplied);
  EXPECT_FALSE(r.ctl.ackDelivered());
  ASSERT_EQ(r.ctl.requestFreezes(), 1);
  const std::array<double, 12> frozen_q = r.ctl.qCmd();
  const double frozen_t = r.ctl.tTraj();
  ASSERT_TRUE(r.sm.hasFrozenSnapshot());
  EXPECT_EQ(r.sm.frozenQcmd(), frozen_q);
  EXPECT_DOUBLE_EQ(r.sm.frozenTTraj(), frozen_t);

  // An ack we never saw is not success: the requester waits, then re-sends the
  // SAME seq (the idempotency key) once the budget is spent.
  EXPECT_FALSE(chain.confirmed()) << "an undelivered ack must not be success";
  EXPECT_EQ(chain.hold().attempts(), 1);
  int spins = 0;
  while (chain.lastStep().action != HoldAction::Resend && spins++ < 200) {
    r.period(chain);
  }
  ASSERT_EQ(chain.lastStep().action, HoldAction::Resend) << chain.outcome();
  EXPECT_EQ(chain.lastStep().reason, "ack_timeout");
  EXPECT_EQ(chain.hold().seq(), 1u) << "a retry keeps the seq: it is the idempotency key";
  EXPECT_GT(r.now - send_time, 0.3);
  EXPECT_FALSE(chain.confirmed());

  // The re-send goes out as the same seq; the controller replays it and does not
  // freeze a second time.
  r.period(chain);
  EXPECT_TRUE(chain.lastRequestAccepted());
  EXPECT_EQ(chain.hold().seq(), 1u);
  r.period(chain);
  EXPECT_EQ(r.ctl.ackSeq(), 1u);
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckApplied);
  EXPECT_EQ(r.ctl.requestFreezes(), 1) << "a replay of the same seq is idempotent";
  EXPECT_EQ(r.ctl.qCmd(), frozen_q);
  EXPECT_DOUBLE_EQ(r.ctl.tTraj(), frozen_t);

  // The link is healthy again: retry with a NEW seq.  The request object is
  // accepted (the slot is free, not busy), and because the trajectory is already
  // stopped the drain refuses to freeze it again -- while the executor can still
  // verify from hold_origin that the freeze is its own.
  r.ctl.setAckDelivery(true);
  const std::uint32_t seq2 = chain.beginNewRequest(r.now, "retry_after_ack_loss");
  EXPECT_EQ(seq2, 2u) << "a NEW seq is required for a new request";
  r.period(chain);
  EXPECT_TRUE(chain.lastRequestAccepted()) << chain.lastRequestReply();
  EXPECT_EQ(r.ctl.queuedSeq(), 2u);
  r.period(chain);
  ASSERT_TRUE(chain.confirmed()) << chain.outcome();
  EXPECT_EQ(chain.outcome(), "frozen_by_executor_request");
  EXPECT_EQ(r.ctl.ackSeq(), 2u);
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckRefused)
    << "already frozen: the drain must not freeze twice";
  EXPECT_EQ(r.ctl.requestFreezes(), 1);

  // SAFETY STATE: the plant is frozen, the executor holds, nothing completed.
  EXPECT_EQ(r.ctl.state(), "HOLD");
  EXPECT_EQ(r.ctl.holdOrigin(), "executor");
  EXPECT_EQ(r.ctl.failureCode(), "hold_by_executor");
  EXPECT_FALSE(r.ctl.running());
  EXPECT_EQ(r.ctl.qCmd(), frozen_q);
  EXPECT_DOUBLE_EQ(r.ctl.tTraj(), frozen_t);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_desync");
  EXPECT_NE(r.sm.state(), State::Complete);
  EXPECT_EQ(r.sm.frozenQcmd(), r.ctl.qCmd());
}

TEST(HoldChainE2E, R5E2E_DuplicateSeqReplayIsIdempotentAndDoesNotFreezeTwice)
{
  Rig r;
  ExecutorHoldChain chain(r.sm, r.ctl);
  ASSERT_TRUE(r.started) << r.why;

  // Our own request stops the plant.
  r.desyncPeriod(chain);
  r.period(chain);
  ASSERT_TRUE(chain.confirmed()) << chain.outcome();
  ASSERT_EQ(chain.outcome(), "frozen_by_executor_request");
  ASSERT_EQ(r.ctl.holdOrigin(), "executor");
  ASSERT_EQ(r.ctl.requestFreezes(), 1);
  const std::array<double, 12> frozen_q = r.ctl.qCmd();
  const double frozen_t = r.ctl.tTraj();
  const std::uint32_t seq = chain.hold().seq();

  // Duplicate request: the same seq again (the service call was retried by the
  // caller, so the controller sees the same idempotency key).
  std::string why;
  ASSERT_TRUE(r.ctl.requestHold(seq, &why)) << why;
  r.ctl.drain();
  EXPECT_EQ(r.ctl.ackSeq(), seq);
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckApplied);
  EXPECT_EQ(r.ctl.requestFreezes(), 1) << "a duplicate seq must not freeze twice";
  EXPECT_EQ(r.ctl.qCmd(), frozen_q);
  EXPECT_DOUBLE_EQ(r.ctl.tTraj(), frozen_t);

  // A restarted executor retries the same seq: still frozen, still attributed to
  // the executor's request, still no second freeze.
  HoldRequester replay;
  replay.begin(r.now, seq, "duplicate_replay");
  HoldObservation obs;
  obs.now_s = r.now;
  obs.have_status = true;
  obs.status_age_s = 0.0;
  obs.state = r.ctl.state();
  obs.hold_origin = r.ctl.holdOrigin();
  obs.ack_seq = r.ctl.ackSeq();
  obs.ack_result = r.ctl.ackResult();
  const HoldStep step = replay.onObservation(obs);
  ASSERT_EQ(step.action, HoldAction::Done);
  EXPECT_EQ(replay.outcome(), "frozen_by_executor_request");
  EXPECT_EQ(r.ctl.requestFreezes(), 1);

  // The same replay AFTER a resume is not "still frozen": the controller refuses
  // it and the executor must not claim a freeze it does not have.
  ASSERT_TRUE(r.ctl.resume());
  r.now += kDt;
  r.ctl.advancePlant(kDt);
  ASSERT_GT(r.ctl.tTraj(), frozen_t);
  ASSERT_TRUE(r.ctl.requestHold(seq, &why)) << why;
  r.ctl.drain();
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckRefused);
  EXPECT_EQ(r.ctl.requestFreezes(), 1);

  HoldRequester after_resume;
  after_resume.begin(r.now, seq, "replay_after_resume");
  obs.now_s = r.now;
  obs.state = r.ctl.state();
  obs.hold_origin = r.ctl.holdOrigin();
  obs.ack_seq = r.ctl.ackSeq();
  obs.ack_result = r.ctl.ackResult();
  const HoldStep step2 = after_resume.onObservation(obs);
  ASSERT_EQ(step2.action, HoldAction::Abort);
  EXPECT_EQ(after_resume.outcome(), "not_running:RUNNING");
  EXPECT_EQ(after_resume.outcome().find("frozen"), std::string::npos)
    << "a silent refusal after a resume must never be reported as frozen";
  EXPECT_EQ(r.ctl.state(), "RUNNING");

  // And the executor refuses to keep believing the freeze: a q_cmd that moved
  // while it is holding is a fault, not a quiet success.
  ControllerStatus st = r.ctl.status(r.now);
  r.sm.tick(r.now, &st);
  EXPECT_EQ(r.sm.state(), State::Fault);
  EXPECT_EQ(r.sm.failureCode(), "hold_invariant_violated");
  EXPECT_NE(r.sm.state(), State::Complete);
}

TEST(HoldChainE2E, R5E2E_HoldAckTimeoutRecordsFailureWithoutClaimingSuccess)
{
  Rig r;
  HoldRequesterConfig cfg;
  cfg.max_attempts = 3;
  cfg.ack_timeout_s = 0.3;
  ExecutorHoldChain chain(r.sm, r.ctl, cfg);
  ASSERT_TRUE(r.started) << r.why;

  // Every ack is published and none is delivered: the executor cannot know that
  // the controller froze, so it must not assume it did.
  r.ctl.setAckDelivery(false);
  r.desyncPeriod(chain);
  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_EQ(r.sm.failureCode(), "phase_desync");

  int spins = 0;
  while (!chain.hold().done() && spins++ < 2000) {
    r.period(chain);
  }
  ASSERT_TRUE(chain.hold().done()) << "the ack budget must expire";
  ASSERT_EQ(chain.lastStep().action, HoldAction::Abort);
  EXPECT_EQ(chain.outcome(), "hold_ack_timeout");
  EXPECT_EQ(chain.hold().attempts(), cfg.max_attempts);
  EXPECT_FALSE(chain.confirmed()) << "a timeout is not success";

  // The failure is recorded as an executor-side ABORT, keeping the safety state,
  // rather than as a completed transport or an unconfirmed "frozen".
  EXPECT_EQ(r.sm.state(), State::Abort);
  EXPECT_EQ(r.sm.failureCode(), "hold_not_confirmed:hold_ack_timeout");
  EXPECT_NE(r.sm.state(), State::Complete);
  EXPECT_NE(r.sm.state(), State::Hold);

  // Meanwhile the controller really did freeze: the ack existed, it was simply
  // never delivered to this executor.  The executor's frozen snapshot still
  // matches the controller's frozen command.
  EXPECT_EQ(r.ctl.state(), "HOLD");
  EXPECT_EQ(r.ctl.holdOrigin(), "executor");
  EXPECT_EQ(r.ctl.ackSeq(), 1u);
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckApplied);
  EXPECT_FALSE(r.ctl.ackDelivered());
  EXPECT_EQ(r.ctl.requestFreezes(), 1);
  EXPECT_FALSE(r.ctl.running());
  ASSERT_TRUE(r.sm.hasFrozenSnapshot());
  EXPECT_EQ(r.sm.frozenQcmd(), r.ctl.qCmd());
  EXPECT_DOUBLE_EQ(r.sm.frozenTTraj(), r.ctl.tTraj());
}

TEST(HoldChainE2E, R5E2E_ControllerOriginHoldIsNotClaimedAsOurs)
{
  Rig r;
  ExecutorHoldChain chain(r.sm, r.ctl);
  ASSERT_TRUE(r.started) << r.why;
  for (int i = 0; i < 3; ++i) {r.period(chain);}
  ASSERT_FALSE(r.sm.latched());

  // A controller safety layer stops the trajectory BEFORE the executor asks for
  // anything.  "collision" is the R6 HOLD code, not a FAULT: the machine keeps
  // the reason and stops.
  r.ctl.holdForSafety("collision");
  ASSERT_EQ(r.ctl.holdOrigin(), "controller");
  ASSERT_EQ(r.ctl.requestFreezes(), 0);
  const std::array<double, 12> frozen_q = r.ctl.qCmd();
  const double frozen_t = r.ctl.tTraj();

  // The executor sees the controller's own HOLD and does ask for a hold; the
  // controller refuses because the trajectory is not running.  The freeze is
  // real, and it is NOT ours.
  r.period(chain);
  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_EQ(r.sm.failureCode(), "collision");
  EXPECT_TRUE(chain.lastRequestAccepted()) << chain.lastRequestReply();
  // The request object is accepted by the slot; the drain decides it in the next
  // period, and that decision is a refusal: there is nothing left to freeze.
  r.period(chain);
  ASSERT_TRUE(chain.confirmed()) << chain.outcome();
  EXPECT_EQ(chain.outcome(), "frozen_by_controller");
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckRefused);
  EXPECT_EQ(r.ctl.holdOrigin(), "controller") << "the origin must not be rewritten";
  EXPECT_EQ(r.ctl.failureCode(), "collision");
  EXPECT_EQ(r.ctl.requestFreezes(), 0) << "this freeze is not the executor's";

  // SAFETY STATE: frozen plant, executor holding, nothing complete.
  EXPECT_EQ(r.ctl.qCmd(), frozen_q);
  EXPECT_DOUBLE_EQ(r.ctl.tTraj(), frozen_t);
  EXPECT_EQ(r.ctl.state(), "HOLD");
  EXPECT_FALSE(r.ctl.running());
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_NE(r.sm.state(), State::Complete);
  ASSERT_TRUE(r.sm.hasFrozenSnapshot());
  EXPECT_EQ(r.sm.frozenQcmd(), r.ctl.qCmd());
  EXPECT_DOUBLE_EQ(r.sm.frozenTTraj(), r.ctl.tTraj());
}

TEST(HoldChainE2E, R5E2E_ExecutorOriginHoldIsAttributedToOurRequest)
{
  Rig r;
  ExecutorHoldChain chain(r.sm, r.ctl);
  ASSERT_TRUE(r.started) << r.why;
  for (int i = 0; i < 3; ++i) {r.period(chain);}
  ASSERT_EQ(r.sm.state(), State::Approach);
  ASSERT_FALSE(r.sm.latched());

  // Executor-origin HOLD: the executor's own phase audit catches a jump.
  r.desyncPeriod(chain);
  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_EQ(r.sm.failureCode(), "phase_desync");
  ASSERT_TRUE(chain.lastRequestAccepted()) << chain.lastRequestReply();

  // The controller applies it: result 1, origin executor, freeze counted as ours.
  r.period(chain);
  ASSERT_TRUE(chain.confirmed()) << chain.outcome();
  EXPECT_EQ(chain.outcome(), "frozen_by_executor_request");
  EXPECT_EQ(r.ctl.ackSeq(), 1u);
  EXPECT_EQ(r.ctl.ackResult(), ControllerHoldModel::kAckApplied);
  EXPECT_EQ(r.ctl.holdOrigin(), "executor");
  EXPECT_EQ(r.ctl.failureCode(), "hold_by_executor");
  EXPECT_EQ(r.ctl.requestFreezes(), 1);
  EXPECT_FALSE(r.ctl.running());

  // SAFETY STATE: the plant stopped, and the executor's own frozen snapshot is
  // exactly the controller's frozen command (no drift, no second freeze).
  EXPECT_EQ(r.ctl.state(), "HOLD");
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_NE(r.sm.state(), State::Complete);
  ASSERT_TRUE(r.sm.hasFrozenSnapshot());
  EXPECT_EQ(r.sm.frozenQcmd(), r.ctl.qCmd());
  EXPECT_DOUBLE_EQ(r.sm.frozenTTraj(), r.ctl.tTraj());
  const std::array<double, 12> frozen_q = r.ctl.qCmd();
  const double frozen_t = r.ctl.tTraj();

  // It stays frozen: another second of periods must not move the plant and must
  // not unlatch the HOLD.
  for (int i = 0; i < 125; ++i) {
    r.period(chain);
    ASSERT_EQ(r.sm.state(), State::Hold);
    ASSERT_EQ(r.ctl.qCmd(), frozen_q);
    ASSERT_DOUBLE_EQ(r.ctl.tTraj(), frozen_t);
  }
  EXPECT_EQ(r.ctl.requestFreezes(), 1);
  EXPECT_EQ(chain.outcome(), "frozen_by_executor_request");
}
