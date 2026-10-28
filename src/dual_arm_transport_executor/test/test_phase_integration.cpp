// C4.3 S4.1 W3-R2 — phase driving integration (APPROACH -> LIFT -> TRANSPORT
// -> LOWER) on top of the W3-R1 state machine.
//
// Everything runs on an injected clock and an injected controller: no ROS, no
// Gazebo, no actuator.  The legacy W3-R1 behaviour (no driver attached) is
// asserted unchanged in the last two cases.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dual_arm_transport_executor/phase_driver.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

using dual_arm_transport_executor::Config;
using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::PhaseDriver;
using dual_arm_transport_executor::PhaseSpec;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::State;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::test::FakeController;

namespace
{

constexpr double kDt = 0.008;

std::vector<PhaseSpec> fivePhaseTrajectory()
{
  return {
    {"approach", 0.30},
    {"lift", 1.50},
    {"transport", 1.50},
    {"lower", 1.50},
    {"settle", 0.30},
  };
}

struct Rig
{
  Config cfg;
  TransportStateMachine sm;
  FakeController fake;
  PhaseDriver driver;
  double now = 0.0;

  explicit Rig(std::vector<PhaseSpec> phases = fivePhaseTrajectory())
  : sm(cfg), driver(phases) {}

  void prime()
  {
    fake.init(now, kDt);
    sm.tick(now, fake.ptr());
  }
  bool start(double duration = 5.10)
  {
    fake.beginPhase(duration);
    StartRequest req;
    req.duration_s = duration;
    std::string why;
    return sm.start(now, req, &why);
  }
  void step(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      now += kDt;
      fake.step(now);
      sm.tick(now, fake.ptr());
    }
  }
  // Advance until the trajectory clock passes `t`, or the executor latches.
  void runUntil(double t)
  {
    while (now < t && !sm.latched()) {
      step();
    }
  }
};

std::vector<State> stateSequence(const TransportStateMachine & sm)
{
  std::vector<State> seq;
  for (const auto & e : sm.events()) {
    if (e.kind == "advance" || e.kind == "start" || e.kind == "protection") {
      const std::string & to = e.to;
      State s = State::Idle;
      if (to == "APPROACH") {s = State::Approach;}
      else if (to == "GRASP") {s = State::Grasp;}
      else if (to == "LIFT") {s = State::Lift;}
      else if (to == "TRANSPORT") {s = State::Transport;}
      else if (to == "LOWER") {s = State::Lower;}
      else if (to == "RELEASE") {s = State::Release;}
      else if (to == "HOLD") {s = State::Hold;}
      else {continue;}
      if (seq.empty() || seq.back() != s) {seq.push_back(s);}
    }
  }
  return seq;
}

}  // namespace

TEST(PhaseIntegration, WalksApproachToLowerWithTheControllerClock)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  EXPECT_EQ(r.sm.state(), State::Approach);

  r.runUntil(4.95);  // past the last boundary (4.80), inside settle
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  EXPECT_EQ(r.sm.state(), State::Lower);
  EXPECT_EQ(r.sm.phaseIndex(), 4);

  const std::vector<State> seq = stateSequence(r.sm);
  const std::vector<State> expected = {
    State::Approach, State::Grasp, State::Lift, State::Transport, State::Lower};
  EXPECT_EQ(seq, expected);
}

TEST(PhaseIntegration, PhaseJumpLatchesHold)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  r.runUntil(0.10);
  ASSERT_EQ(r.sm.phaseIndex(), 0);

  // Controller clock jumps two phases ahead: 0 -> 2.
  r.fake.status().t_traj = 2.00;
  r.fake.status().stamp_s = r.now;
  r.sm.tick(r.now, r.fake.ptr());

  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_desync");
}

TEST(PhaseIntegration, UnknownPhaseNameLatchesHold)
{
  Rig r({{"approach", 0.30}, {"teleport", 0.50}});
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start(0.80));

  r.runUntil(0.40);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_name_mismatch");
}

TEST(PhaseIntegration, SettlePhaseIsRecordedButDoesNotTransition)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());

  r.runUntil(4.90);  // inside the trailing settle phase
  ASSERT_FALSE(r.sm.latched());
  EXPECT_EQ(r.sm.state(), State::Lower);

  bool recorded = false;
  for (const auto & e : r.sm.events()) {
    if (e.kind == "phase" && e.detail.find("settle") != std::string::npos) {
      recorded = true;
    }
  }
  EXPECT_TRUE(recorded);
}

TEST(PhaseIntegration, PhaseNamesMustMatchTheExecutorState)
{
  // "lift" as the first phase: the executor is in APPROACH, so it advances
  // through GRASP to LIFT -- never skipping the order.
  Rig r({{"lift", 0.30}, {"transport", 0.30}});
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start(0.60));
  r.runUntil(0.10);
  EXPECT_EQ(r.sm.state(), State::Lift);
  r.runUntil(0.40);
  EXPECT_EQ(r.sm.state(), State::Transport);
}

// ---- W3-R2 (D5-1): controller-reported phase is the authority --------------
namespace {
void publishExternal(Rig & r, int idx, const std::string & name)
{
  r.fake.status().phase_index = idx;
  r.fake.status().phase_name = name;
  r.fake.status().phase_count = 5;
  r.fake.status().phase_duration_s = 1.5;
  r.fake.status().stamp_s = r.now;
}
}  // namespace

TEST(PhaseIntegration, ExternalPhaseIndexDrivesTheWalk)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  r.runUntil(0.10);

  // The controller reports its own progress; the executor must follow it even
  // though the local clock has not reached those boundaries yet.
  publishExternal(r, 1, "lift");
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Lift) << r.sm.failureCode();

  publishExternal(r, 2, "transport");
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Transport) << r.sm.failureCode();
  EXPECT_EQ(r.sm.phaseIndex(), 2);
  EXPECT_FALSE(r.sm.latched());
}

TEST(PhaseIntegration, ExternalPhaseJumpLatchesHold)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  r.runUntil(0.10);

  publishExternal(r, 2, "transport");   // 0 -> 2: one phase skipped
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_desync");
}

TEST(PhaseIntegration, ExternalPhaseRegressionLatchesHold)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  r.runUntil(0.10);
  publishExternal(r, 1, "lift");
  r.sm.tick(r.now, r.fake.ptr());
  ASSERT_EQ(r.sm.phaseIndex(), 1);

  publishExternal(r, 0, "approach");    // the controller went backwards
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_desync");
}

TEST(PhaseIntegration, ExternalUnknownPhaseNameLatchesHold)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  r.runUntil(0.10);

  publishExternal(r, 1, "teleport");
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_name_mismatch");
}

TEST(PhaseIntegration, ControllerWithoutPhaseFieldsFallsBackToTheLocalClock)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  ASSERT_TRUE(r.start());
  ASSERT_EQ(r.fake.status().phase_index, -1);   // controller publishes no phases
  // Local fallback: 0.40 s falls in phase 1 ("lift", boundaries {0, 0.30, 1.80,
  // ...}), and the forward walk passes through GRASP on the way.
  r.runUntil(0.40);
  EXPECT_EQ(r.sm.state(), State::Lift) << r.sm.failureCode();
  EXPECT_FALSE(r.sm.latched());
}

TEST(PhaseIntegration, DetachingTheDriverRestoresTheW3R1Behaviour)
{
  Rig r;
  r.prime();
  r.sm.attachPhaseDriver(&r.driver);
  r.sm.attachPhaseDriver(nullptr);
  EXPECT_EQ(r.sm.phaseDriver(), nullptr);
  EXPECT_EQ(r.sm.phaseIndex(), -1);

  ASSERT_TRUE(r.start(0.05));
  r.fake.beginPhase(0.05);
  std::string why;
  ASSERT_TRUE(r.sm.phaseDone(r.now += kDt, &why));   // APPROACH -> GRASP
  ASSERT_TRUE(r.sm.phaseDone(r.now += kDt, &why));   // GRASP -> LIFT
  EXPECT_EQ(r.sm.state(), State::Lift);
  // Legacy rule: LIFT / TRANSPORT / LOWER still advance when the trajectory ends.
  for (int i = 0; i < 50 && r.sm.state() == State::Lift; ++i) {r.step();}
  EXPECT_EQ(r.sm.state(), State::Transport);
}
