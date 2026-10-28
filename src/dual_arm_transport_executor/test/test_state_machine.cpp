// R1-T02 (legal transitions), R1-T03 (illegal rejected), R1-T04 (HOLD freeze),
// R1-T05 (HOLD resume), R1-T06 (FAULT/ABORT latching), R1-T10 (determinism).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

using dual_arm_transport_executor::Config;
using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::Event;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::State;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::test::FakeController;

namespace
{

constexpr double kDt = 0.008;

struct Bench
{
  Config cfg;
  TransportStateMachine sm;
  FakeController fake;
  double now = 0.0;

  explicit Bench(Config c = Config())
  : cfg(c), sm(c) {}

  void prime()
  {
    fake.init(now, kDt);
    sm.tick(now, fake.ptr());
  }
  void step(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      now += kDt;
      fake.step(now);
      sm.tick(now, fake.ptr());
    }
  }
  void sync() {sm.tick(now, fake.ptr());}
  bool start()
  {
    std::string why;
    StartRequest req;
    req.duration_s = 1.0;
    return sm.start(now, req, &why);
  }
  // Run a trajectory phase until the executor leaves it (or timeout).
  bool runPhaseUntilNot(State s, double duration, int max_ticks = 2000)
  {
    fake.beginPhase(duration);
    for (int i = 0; i < max_ticks; ++i) {
      step();
      if (sm.state() != s) {return true;}
    }
    return false;
  }
};

TEST(StateMachine, T02_LegalForwardPath)
{
  Bench b;
  b.prime();
  ASSERT_TRUE(b.start());
  EXPECT_EQ(b.sm.state(), State::Approach);

  std::string why;
  ASSERT_TRUE(b.sm.phaseDone(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Grasp);
  ASSERT_TRUE(b.sm.phaseDone(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Lift);

  EXPECT_TRUE(b.runPhaseUntilNot(State::Lift, 0.05));
  EXPECT_EQ(b.sm.state(), State::Transport);
  EXPECT_TRUE(b.runPhaseUntilNot(State::Transport, 0.05));
  EXPECT_EQ(b.sm.state(), State::Lower);
  EXPECT_TRUE(b.runPhaseUntilNot(State::Lower, 0.05));
  EXPECT_EQ(b.sm.state(), State::Release);

  b.fake.endTransport();
  b.step();   // deliver the "transport finished" observation to the executor
  ASSERT_TRUE(b.sm.phaseDone(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Retreat);
  ASSERT_TRUE(b.sm.phaseDone(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Complete);

  // A completed task may be started again.
  std::string why2;
  dual_arm_transport_executor::StartRequest req2;
  req2.duration_s = 1.0;
  ASSERT_TRUE(b.sm.start(b.now, req2, &why2)) << "why=" << why2 << " detail=" << b.sm.events().back().detail;
  EXPECT_EQ(b.sm.state(), State::Approach);
}

TEST(StateMachine, T03_IllegalTransitionsAreRejected)
{
  Bench b;
  b.prime();
  std::string why;

  EXPECT_FALSE(b.sm.resume(b.now, &why));
  EXPECT_EQ(why, "resume_requires_hold");
  EXPECT_FALSE(b.sm.reset(b.now, &why));
  EXPECT_EQ(why, "reset_requires_fault");
  EXPECT_FALSE(b.sm.phaseDone(b.now, &why));
  EXPECT_FALSE(b.sm.abortTransport("", b.now, &why));
  EXPECT_TRUE(b.sm.events().size() >= 4u);

  ASSERT_TRUE(b.start());
  EXPECT_FALSE(b.start()) << "a second start() while running must be refused";

  // FAULT is only cleared by reset; ABORT only by start.
  ASSERT_TRUE(b.sm.abortTransport("closure_violated", b.now, &why));
  EXPECT_EQ(b.sm.state(), State::Abort);
  EXPECT_FALSE(b.sm.reset(b.now, &why));
  EXPECT_EQ(why, "abort_requires_start");
  EXPECT_FALSE(b.sm.resume(b.now, &why));
  EXPECT_EQ(why, "abort_requires_start");
  ASSERT_TRUE(b.start());
  EXPECT_EQ(b.sm.state(), State::Approach);
}

TEST(StateMachine, T04_HoldFreezesAndAuditsTheFreeze)
{
  Bench b;
  b.prime();
  ASSERT_TRUE(b.start());
  b.step(5);

  // Stale status -> HOLD.
  b.now += 1.0;
  b.sm.tick(b.now, b.fake.ptr());
  ASSERT_EQ(b.sm.state(), State::Hold);
  EXPECT_EQ(b.sm.failureCode(), "stale_state");
  ASSERT_TRUE(b.sm.hasFrozenSnapshot());
  const auto frozen_q = b.sm.frozenQcmd();
  const double frozen_t = b.sm.frozenTTraj();

  // An obedient controller keeps the two frozen quantities bitwise constant.
  b.fake.obeyHold();
  for (int i = 0; i < 20; ++i) {
    b.now += kDt;
    b.fake.status().stamp_s = b.now;   // fresh again
    b.sm.tick(b.now, b.fake.ptr());
  }
  EXPECT_EQ(b.sm.state(), State::Hold);
  EXPECT_EQ(b.sm.frozenTTraj(), frozen_t);
  for (int i = 0; i < 12; ++i) {EXPECT_EQ(b.sm.frozenQcmd()[static_cast<size_t>(i)], frozen_q[static_cast<size_t>(i)]);}

  // A controller that keeps advancing while the executor holds is a safety
  // violation, not a benign race.
  b.fake.disobeyHold();
  b.now += kDt;
  b.fake.step(b.now);
  b.sm.tick(b.now, b.fake.ptr());
  EXPECT_EQ(b.sm.state(), State::Fault);
  EXPECT_EQ(b.sm.failureCode(), "hold_invariant_violated");
}

TEST(StateMachine, T05_ResumeChecksThenEscalatesToAbort)
{
  Bench b;
  b.prime();
  ASSERT_TRUE(b.start());
  b.step(3);

  // Force HOLD via a controller-reported code.
  b.fake.status().failure_code = "stale_command";
  b.step(1);
  ASSERT_EQ(b.sm.state(), State::Hold);
  b.fake.obeyHold();

  // 1) object position error above 5 mm blocks resume.
  b.fake.status().failure_code = "ok";
  b.fake.status().e_obj_pos_m = 0.020;
  b.sync();
  std::string why;
  EXPECT_FALSE(b.sm.resume(b.now += kDt, &why));
  EXPECT_EQ(why, "object_pos_error");

  // 2) rotation error above 2 deg blocks resume.
  b.fake.status().e_obj_pos_m = 0.001;
  b.fake.status().e_obj_rot_rad = 0.10;
  b.sync();
  EXPECT_FALSE(b.sm.resume(b.now += kDt, &why));
  EXPECT_EQ(why, "object_rot_error");

  b.fake.status().e_obj_rot_rad = 0.001;
  b.sync();
  EXPECT_TRUE(b.sm.resume(b.now += kDt, &why)) << why;
  EXPECT_EQ(b.sm.state(), State::Approach) << "resume returns to the interrupted state";
}

TEST(StateMachine, T05b_ThirdFailedResumeEscalatesToAbort)
{
  Bench b;
  b.prime();
  ASSERT_TRUE(b.start());
  b.fake.status().failure_code = "stale_command";
  b.step(1);
  ASSERT_EQ(b.sm.state(), State::Hold);
  b.fake.obeyHold();
  b.fake.status().failure_code = "ok";
  b.fake.status().e_obj_pos_m = 0.5;   // never satisfied
  b.sync();

  std::string why;
  EXPECT_FALSE(b.sm.resume(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Hold);
  EXPECT_FALSE(b.sm.resume(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Hold);
  EXPECT_FALSE(b.sm.resume(b.now += kDt, &why));
  EXPECT_EQ(b.sm.state(), State::Abort);
  EXPECT_EQ(b.sm.failureCode(), "resume_failed");
}

TEST(StateMachine, T06_FaultAndAbortNeverAutoClear)
{
  Config c;
  c.approach_timeout_s = 0.05;   // force a FAULT quickly
  Bench b(c);
  b.prime();
  ASSERT_TRUE(b.start());
  b.step(20);
  ASSERT_EQ(b.sm.state(), State::Fault);
  EXPECT_EQ(b.sm.failureCode(), "grasp_check_failed");

  // Healthy status must not clear it.
  b.fake.status().failure_code = "ok";
  b.step(50);
  EXPECT_EQ(b.sm.state(), State::Fault);

  std::string why;
  EXPECT_TRUE(b.sm.reset(b.now, &why)) << why;
  EXPECT_EQ(b.sm.state(), State::Idle);
}

TEST(StateMachine, T10_DeterministicReplay)
{
  auto run = []() {
      Bench b;
      b.prime();
      b.start();
      b.fake.obeyHold();
      for (int i = 0; i < 120; ++i) {
        b.step();
        if (i == 20) {b.fake.status().failure_code = "deadline_missed";}
        if (i == 40) {b.fake.status().failure_code = "ok";}
        if (i == 60) {b.fake.status().active_limit_count = 1;}
      }
      std::vector<std::string> sig;
      for (const auto & e : b.sm.events()) {
        sig.push_back(e.kind + "|" + e.from + "|" + e.to + "|" + e.code);
      }
      sig.push_back(std::string("final:") + dual_arm_transport_executor::toString(b.sm.state()));
      return sig;
    };
  const auto a = run();
  const auto b2 = run();
  ASSERT_EQ(a.size(), b2.size());
  for (size_t i = 0; i < a.size(); ++i) {EXPECT_EQ(a[i], b2[i]) << "event " << i;}
}

}  // namespace
