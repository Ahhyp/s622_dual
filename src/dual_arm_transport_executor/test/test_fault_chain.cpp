// C4.3 S4 W3-R5 -- fault and safety chain matrix (executor side).
//
// Scope honesty: every case here exercises the EXECUTOR state machine against a
// fake controller.  It is evidence class "unit test"; it is NOT a real-time
// result, NOT an actuator-safety result, and NOT an end-to-end T-H15/T-H16
// verdict (those need a real executor or S4.3).  The controller-side half of the
// same matrix lives in
// src/dual_arm_qp_controller/test/test_hold_transport.cpp (BenchHold.*).
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

using dual_arm_transport_executor::Config;
using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::State;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::test::FakeController;

namespace
{

constexpr double kDt = 0.008;

struct Rig
{
  Config cfg;
  TransportStateMachine sm;
  FakeController fake;
  double now = 0.0;

  explicit Rig(Config c = Config()) : cfg(c), sm(c) {}

  void primeAndStart(double duration_s = 0.05)
  {
    fake.init(now, kDt);
    sm.tick(now, fake.ptr());
    StartRequest req;
    req.duration_s = duration_s;
    std::string why;
    ASSERT_TRUE(sm.start(now, req, &why)) << why;
    tick(1);
  }
  void tick(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      now += kDt;
      fake.step(now);
      sm.tick(now, fake.ptr());
    }
  }
  // Apply a status patch, then run real ticks (the fake still stamps and
  // advances the commanded joints, so the executor sees a live controller).
  void tickWith(const ControllerStatus & patch, int n = 1)
  {
    fake.status() = patch;
    tick(n);
  }
};

TEST(FaultChain, R5_1_AbortLatchesAndResumeIsRefused)
{
  Rig r;
  r.primeAndStart();
  std::string why;
  ASSERT_TRUE(r.sm.abortTransport("operator_abort", r.now, &why)) << why;
  ASSERT_EQ(r.sm.state(), State::Abort);

  // A resume attempt must be refused and must not leave ABORT.
  EXPECT_FALSE(r.sm.resume(r.now += kDt, &why));
  EXPECT_EQ(r.sm.state(), State::Abort);
  EXPECT_EQ(r.sm.failureCode(), "operator_abort");

  // Ten more ticks with a healthy heartbeat change nothing.
  r.tick(10);
  EXPECT_EQ(r.sm.state(), State::Abort);
  EXPECT_TRUE(r.sm.latched());
}

TEST(FaultChain, R5_2_LatchesAreOnlyClearedByAnExplicitReset)
{
  Rig r;
  r.primeAndStart();
  ControllerStatus st = r.fake.status();
  st.failure_code = "attach_failed";
  r.tickWith(st);
  ASSERT_TRUE(r.sm.latched());
  const State latched = r.sm.state();
  const std::string code = r.sm.failureCode();

  r.tick(25);  // a healthy run of ticks must not silently clear a latch
  EXPECT_EQ(r.sm.state(), latched);
  EXPECT_EQ(r.sm.failureCode(), code);

  std::string why;
  ASSERT_TRUE(r.sm.reset(r.now += kDt, &why)) << why;
  EXPECT_EQ(r.sm.state(), State::Idle);
  EXPECT_FALSE(r.sm.latched());
  bool saw_reset = false;
  for (const auto & e : r.sm.events()) {
    if (e.kind == "reset") {saw_reset = true;}
  }
  EXPECT_TRUE(saw_reset);
}

TEST(FaultChain, R5_3_ControllerFailureCodePropagatesWithItsOwnCode)
{
  Rig r;
  r.primeAndStart();
  ControllerStatus st = r.fake.status();
  st.state = "FAULT";
  st.failure_code = "follower_divergence";
  r.tickWith(st);
  EXPECT_TRUE(r.sm.latched());
  EXPECT_EQ(r.sm.failureCode(), "follower_divergence");
  EXPECT_EQ(r.sm.state(), State::Abort);   // per the contract table, not invented here

  // The executor must not have claimed success anywhere.
  EXPECT_NE(r.sm.state(), State::Complete);
}

TEST(FaultChain, R5_4_HoldFreezesTheCommandEvenWhenTheControllerKeepsPublishing)
{
  Rig r;
  r.primeAndStart();
  ControllerStatus st = r.fake.status();
  st.failure_code = "qp_solver_failed";
  // The executor has asked for hold with seq 7; only an ack answering 7 may be
  // used as the new baseline.
  r.sm.expectHoldAck(7u);
  r.tickWith(st);
  ASSERT_EQ(r.sm.state(), State::Hold);
  EXPECT_TRUE(r.sm.hasFrozenSnapshot());
  const auto frozen = r.sm.frozenQcmd();
  const double frozen_t = r.sm.frozenTTraj();

  // The controller keeps advancing t_traj; the frozen snapshot must not move.
  for (int i = 0; i < 5; ++i) {
    r.now += kDt;
    r.fake.step(r.now);
    r.sm.tick(r.now, r.fake.ptr());
    for (int j = 0; j < 12; ++j) {
      EXPECT_EQ(r.sm.frozenQcmd()[j], frozen[j]) << "joint " << j << " moved";
    }
    EXPECT_EQ(r.sm.frozenTTraj(), frozen_t);
  }
}

TEST(FaultChain, R5_5_StaleStatusNeverReadsAsHealthyAndNeverClearsAHold)
{
  Rig r;
  r.primeAndStart();
  // Enter HOLD through a controller refusal, then stop publishing entirely.
  ControllerStatus st = r.fake.status();
  st.failure_code = "qp_solver_failed";
  // The executor has asked for hold with seq 7; only an ack answering 7 may be
  // used as the new baseline.
  r.sm.expectHoldAck(7u);
  r.tickWith(st);
  ASSERT_EQ(r.sm.state(), State::Hold);
  const std::string code = r.sm.failureCode();

  for (int i = 0; i < 60; ++i) {
    r.now += kDt;
    r.sm.tick(r.now, nullptr);  // nothing arrives
  }
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_TRUE(r.sm.latched());
  // The reason may be upgraded to stale_state, but a hold never reads as healthy.
  EXPECT_FALSE(r.sm.failureCode().empty());
  EXPECT_FALSE(code.empty());
}

}  // namespace

// C7 independent review (P1-3): the freeze snapshot was taken from the status in
// hand when HOLD latched.  Status is published at 10 Hz while the controller
// freezes per control tick, so a hold drained one cycle later compared the
// frozen q_cmd against a PRE-freeze snapshot and reported a bogus FAULT
// hold_invariant_violated.  The first status seen while latched now re-baselines
// the audit; the audit itself must stay live afterwards.
TEST(FaultChain, R5_6_FreezeAuditRebaselinesOnTheFirstLatchedStatus)
{
  Rig r;
  r.primeAndStart(2.0);

  // Latch HOLD through a safety reason while the controller is still publishing
  // a pre-freeze status.
  ControllerStatus st = r.fake.status();
  st.failure_code = "qp_solver_failed";
  // The executor has asked for hold with seq 7; only an ack answering 7 may be
  // used as the new baseline.
  r.sm.expectHoldAck(7u);
  r.tickWith(st);
  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_TRUE(r.sm.hasFrozenSnapshot());

  // The controller has now frozen one tick later: the first latched status
  // carries a *different* q_cmd than the snapshot taken at latch time, and it
  // confirms our freeze (hold_request_result == applied).
  ControllerStatus frozen = r.fake.status();
  frozen.failure_code = "qp_solver_failed";
  frozen.hold_request_seq = 7u;
  frozen.hold_request_result = 1u;
  for (auto & q : frozen.q_cmd) {q += 1e-3;}
  r.tickWith(frozen);
  EXPECT_EQ(r.sm.state(), State::Hold)
    << "a one-frame-late freeze must not be reported as hold_invariant_violated";
  EXPECT_EQ(r.sm.failureCode(), "qp_solver_failed");

  // The audit must still be live: a further change is a real violation.
  ControllerStatus moved = frozen;
  for (auto & q : moved.q_cmd) {q += 1e-3;}
  r.tickWith(moved);
  EXPECT_EQ(r.sm.state(), State::Fault)
    << "the freeze audit must keep running after the re-baseline";
  EXPECT_EQ(r.sm.failureCode(), "hold_invariant_violated");
}


// P1-3 isolation (a): an ack from an OLDER request must never serve as the new
// baseline.  Otherwise a stale "applied" would hide the very motion the audit
// exists to catch.
TEST(FaultChain, R5_7_OldAckSeqDoesNotRebaselineTheAudit)
{
  Rig r;
  r.primeAndStart(2.0);
  ControllerStatus st = r.fake.status();
  st.failure_code = "qp_solver_failed";
  r.sm.expectHoldAck(9u);
  r.tickWith(st);
  ASSERT_EQ(r.sm.state(), State::Hold);

  ControllerStatus old_ack = st;
  old_ack.hold_request_seq = 8u;   // answers a DIFFERENT (previous) request
  old_ack.hold_request_result = 1u;
  for (auto & q : old_ack.q_cmd) {q += 1e-3;}
  r.tickWith(old_ack);
  EXPECT_EQ(r.sm.state(), State::Fault)
    << "an applied ack for seq 8 must not excuse motion observed under request 9";
  EXPECT_EQ(r.sm.failureCode(), "hold_invariant_violated");
}

// P1-3 isolation (b): a status that answers nothing (result == 0) can never be
// a baseline -- the executor would be trusting a sample it cannot match.
TEST(FaultChain, R5_8_UnansweredStatusNeverRebaselines)
{
  Rig r;
  r.primeAndStart(2.0);
  ControllerStatus st = r.fake.status();
  st.failure_code = "qp_solver_failed";
  r.sm.expectHoldAck(5u);
  r.tickWith(st);
  ASSERT_EQ(r.sm.state(), State::Hold);

  ControllerStatus silent = st;
  silent.hold_request_seq = 5u;
  silent.hold_request_result = 0u;   // no verdict at all
  for (auto & q : silent.q_cmd) {q += 1e-3;}
  r.tickWith(silent);
  EXPECT_EQ(r.sm.state(), State::Fault);
  EXPECT_EQ(r.sm.failureCode(), "hold_invariant_violated");
}
