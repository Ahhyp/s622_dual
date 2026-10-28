// C4.3 S4 W7 -- full-flow integration of the executor with the physical attach
// client (W6) and the existing W3-R5 fault chain.
//
// Evidence class: unit test against a fake controller.  It is NOT a Gazebo run,
// NOT a real-time result, NOT actuator safety, and NOT an S4-T13 PASS.  What it
// does prove is that the wiring has the following properties:
//   * a startup weld is never treated as a grasp: start() is refused until the
//     startup guard saw a confirmed release;
//   * GRASP cannot advance (by controller phase OR by phaseDone) before the
//     attach is confirmed;
//   * an attach that is confirmed without a snap measurement, or with a snap
//     beyond the frozen 2 mm criterion, is a HOLD (fail closed), not a success;
//   * RELEASE detaches before advancing, and a failed detach is a FAULT;
//   * a complete run reaches COMPLETE with weld attach at GRASP and detach at
//     RELEASE, and the event trail is deterministic for identical input.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "dual_arm_transport_executor/attach_client.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

using dual_arm_transport_executor::AttachClient;
using dual_arm_transport_executor::AttachClientConfig;
using dual_arm_transport_executor::Config;
using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::State;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::WeldState;
using dual_arm_transport_executor::test::FakeController;

namespace
{

constexpr double kDt = 0.008;

struct Rig
{
  Config cfg;
  TransportStateMachine sm;
  FakeController fake;
  AttachClient weld;
  double now = 0.0;
  // The plugin really welds at startup in Gazebo; whether we saw the message or
  // not, the guard has to prove the weld is gone before anything may start.
  WeldState plugin_state = WeldState::Unknown;

  Rig()
  : sm(cfg)
  {
    AttachClientConfig wcfg;
    wcfg.state_stale_s = 0.25;
    wcfg.transition_timeout_s = 0.05;
    wcfg.startup_release_timeout_s = 0.5;
    wcfg.max_attach_attempts = 1;
    wcfg.max_snap_m = 0.002;
    weld.configure(wcfg);
    sm.attachWeldClient(&weld);
  }

  void prime(double duration_s = 0.05)
  {
    fake.init(now, kDt);
    // The plugin's startup weld is already there (Gazebo creates it ~2.2 s after
    // boot); the guard starts by assuming it may be welded and must prove it is
    // gone.  The executor node calls this from its constructor.
    weld.beginStartupGuard(now);
    sm.tick(now, fake.ptr());
    duration_ = duration_s;
  }

  bool start(std::string * why = nullptr)
  {
    StartRequest req;
    req.duration_s = duration_;
    return sm.start(now, req, why);
  }

  // One control period: advance the fake controller, feed the weld observation
  // the plugin would publish, then tick the state machine.
  void tick(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      now += kDt;
      fake.step(now);
      weld.observe(plugin_state, now);
      // The node's driveWeld() pumps the client every control cycle and publishes
      // whatever request it produces; here the "publish" is the plugin_state the
      // next tick feeds back in.
      weld.tick(now);
      sm.tick(now, fake.ptr());
    }
  }

  // Complete the startup guard: the plugin reports "detached" (its detach was
  // requested by the client) and the guard confirms it.
  void releaseStartupWeld()
  {
    plugin_state = WeldState::Detached;
    tick(3);
    ASSERT_TRUE(weld.startupVerified());
  }

  std::vector<std::string> trail() const
  {
    std::vector<std::string> out;
    for (const auto & e : sm.events()) {
      out.push_back(e.kind + ":" + e.code + ":" + e.from + ">" + e.to);
    }
    return out;
  }

  double duration_ = 0.05;
};

// Walk the executor to GRASP and perform a confirmed attach with the given snap.
void graspWithConfirmedAttach(Rig & r, double snap_m)
{
  while (r.sm.state() != State::Grasp && !r.sm.latched()) {
    r.sm.phaseDone(r.now, nullptr);
    r.tick(1);
  }
  ASSERT_EQ(r.sm.state(), State::Grasp);
  // The client refuses to attach while the plugin still reports "attached"
  // (startup weld) -- we are detached here, so the attach may proceed.
  r.plugin_state = WeldState::Detached;
  r.tick(1);                       // requestAttach -> RequestAttach -> publish
  r.weld.setMeasuredSnap(snap_m);   // the node measures this from bridged poses
  r.plugin_state = WeldState::Attached;
  r.tick(1);   // confirm; if that latches HOLD, do not tick again (the freeze
               // audit would turn a *hold* into a fault purely because the fake
               // controller keeps moving)
}

TEST(FullFlow, W7_StartIsRefusedUntilTheStartupWeldIsReleased)
{
  Rig r;
  r.prime();
  std::string why;
  EXPECT_FALSE(r.start(&why));
  EXPECT_EQ(why, "weld_startup_release_pending");
  EXPECT_EQ(r.sm.state(), State::Idle);

  // The plugin reports "attached" for the whole window: the guard must fail, and
  // a weld that was never released must never become a grasp.
  r.plugin_state = WeldState::Attached;
  r.tick(80);                       // 0.64 s > 0.5 s startup window
  EXPECT_TRUE(r.weld.failed());
  EXPECT_EQ(r.weld.outcome(), "weld_not_released");
  EXPECT_FALSE(r.start(&why));
  EXPECT_EQ(why, "weld_startup_release_pending");
}

TEST(FullFlow, W7_StartIsAcceptedOnceTheReleaseIsConfirmed)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  std::string why;
  EXPECT_TRUE(r.start(&why)) << why;
  EXPECT_EQ(r.sm.state(), State::Approach);
}

TEST(FullFlow, W7_GraspCannotAdvanceBeforeTheAttachIsConfirmed)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  while (r.sm.state() != State::Grasp) {r.sm.phaseDone(r.now, nullptr); r.tick(1);}
  ASSERT_EQ(r.sm.state(), State::Grasp);

  // Explicit "gripper closed" request from outside: refused while unwelded.
  std::string why;
  EXPECT_FALSE(r.sm.phaseDone(r.now, &why));
  EXPECT_EQ(why, "weld_not_confirmed");
  EXPECT_EQ(r.sm.state(), State::Grasp);

  // Ticking alone does not advance either: the weld is requested but unconfirmed.
  r.plugin_state = WeldState::Detached;
  r.tick(5);
  EXPECT_EQ(r.sm.state(), State::Grasp);
  EXPECT_FALSE(r.sm.weldAttached());
}

TEST(FullFlow, W7_ConfirmedAttachWithInCriterionSnapUnlocksGrasp)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  graspWithConfirmedAttach(r, 0.0005);
  EXPECT_TRUE(r.sm.weldAttached());
  EXPECT_FALSE(r.sm.latched());
  EXPECT_EQ(r.sm.state(), State::Grasp);
  EXPECT_TRUE(r.sm.phaseDone(r.now, nullptr));
  EXPECT_EQ(r.sm.state(), State::Lift);
}

TEST(FullFlow, W7_UnmeasuredSnapFailsClosed)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  while (r.sm.state() != State::Grasp) {r.sm.phaseDone(r.now, nullptr); r.tick(1);}
  r.plugin_state = WeldState::Detached;
  r.tick(1);
  // No setMeasuredSnap(): the node has no pose evidence.
  r.plugin_state = WeldState::Attached;
  r.tick(1);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "attach_failed");
  EXPECT_EQ(r.weld.outcome(), "attach_snap_unmeasured");
}

TEST(FullFlow, W7_SnapBeyondTheFrozenCriterionHoldsInsteadOfContinuing)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  graspWithConfirmedAttach(r, 0.0021);   // 2.1 mm > 2 mm
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "attach_failed");
  EXPECT_EQ(r.weld.outcome(), "attach_snap_exceeded");
  EXPECT_FALSE(r.sm.weldAttached());
}

TEST(FullFlow, W7_FullRunReachesCompleteAndDetachesAtRelease)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  graspWithConfirmedAttach(r, 0.0001);

  const State order[] = {
    State::Grasp, State::Lift, State::Transport, State::Lower, State::Release, State::Retreat};
  for (State want : order) {
    ASSERT_EQ(r.sm.state(), want) << "before advancing from " << dual_arm_transport_executor::toString(want);
    if (want == State::Release) {
      // RELEASE: the detach must be requested, then confirmed, before advancing.
      std::string why;
      EXPECT_FALSE(r.sm.phaseDone(r.now, &why));
      EXPECT_EQ(why, "weld_not_confirmed");
      r.plugin_state = WeldState::Attached;
      r.tick(1);                        // requestDetach -> RequestDetach
      r.plugin_state = WeldState::Detached;
      r.tick(2);                        // detach confirmed
      EXPECT_FALSE(r.sm.weldAttached());
    }
    EXPECT_TRUE(r.sm.phaseDone(r.now, nullptr))
      << "task could not advance out of " << dual_arm_transport_executor::toString(want);
    r.tick(1);
  }
  EXPECT_EQ(r.sm.state(), State::Complete);
  EXPECT_FALSE(r.sm.latched());
}

TEST(FullFlow, W7_DetachThatNeverConfirmsIsAFault)
{
  Rig r;
  r.prime();
  r.releaseStartupWeld();
  ASSERT_TRUE(r.start(nullptr));
  graspWithConfirmedAttach(r, 0.0001);
  while (r.sm.state() != State::Release) {
    ASSERT_FALSE(r.sm.latched());
    r.sm.phaseDone(r.now, nullptr);
    r.tick(1);
  }
  // The plugin keeps reporting "attached": the physical grasp is never released.
  r.plugin_state = WeldState::Attached;
  r.tick(20);
  EXPECT_EQ(r.sm.state(), State::Fault);
  EXPECT_EQ(r.sm.failureCode(), "release_failed");
}

TEST(FullFlow, W7_IdenticalInputProducesAnIdenticalTrail)
{
  auto run = []() {
      Rig r;
      r.prime();
      r.releaseStartupWeld();
      if (!r.start(nullptr)) {return std::vector<std::string>{};}
      graspWithConfirmedAttach(r, 0.0001);
      for (int i = 0; i < 6; ++i) {r.sm.phaseDone(r.now, nullptr); r.tick(1);}
      return r.trail();
    };
  const auto a = run();
  const auto b = run();
  ASSERT_FALSE(a.empty());
  EXPECT_EQ(a, b);   // same input -> same decisions (no hidden clock/randomness)
}

}  // namespace
