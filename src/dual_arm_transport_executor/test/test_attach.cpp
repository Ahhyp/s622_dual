// C4.3 S4.1 W6 -- unit tests for the weld client (no simulator, no ROS).
//
// What these tests are evidence for: the *decisions* the executor makes about the
// Gazebo weld.  They are NOT evidence that the plugin behaves as measured in
// W6-0 (that is the W6-0 report), and NOT evidence of an end-to-end grasp.

#include <cmath>
#include <limits>

#include "dual_arm_transport_executor/attach_client.hpp"
#include "gtest/gtest.h"

using dual_arm_transport_executor::AttachClient;
using dual_arm_transport_executor::AttachClientConfig;
using dual_arm_transport_executor::WeldAction;
using dual_arm_transport_executor::WeldState;

namespace
{

AttachClientConfig cfg()
{
  AttachClientConfig c;
  c.state_stale_s = 0.25;
  c.transition_timeout_s = 0.5;
  c.max_attach_attempts = 3;
  c.startup_release_timeout_s = 5.0;
  c.max_snap_m = 0.002;
  return c;
}

/// Drive the startup guard to a confirmed detach at t = 0.1 s.
AttachClient releasedAt(double t = 0.1)
{
  AttachClient c(cfg());
  c.beginStartupGuard(0.0);
  EXPECT_EQ(c.tick(0.0), WeldAction::RequestDetach);
  c.observe(WeldState::Detached, t);
  EXPECT_EQ(c.tick(t), WeldAction::Confirmed);
  EXPECT_TRUE(c.startupVerified());
  return c;
}

}  // namespace

// A pre-guard sample cannot prove the release: beginStartupGuard() forgets it.
TEST(AttachClient, StartupGuardIgnoresWeldStateObservedBeforeItStarted)
{
  AttachClient c(cfg());
  c.observe(WeldState::Detached, 0.0);
  c.beginStartupGuard(1.0);
  EXPECT_EQ(c.tick(1.0), WeldAction::RequestDetach)
    << "the plugin may have welded the sample after that stale sample was taken";
  EXPECT_EQ(c.tick(1.1), WeldAction::Wait);
  EXPECT_FALSE(c.startupVerified());
}

// W6-0: the startup weld is real and must be released before anything else.
TEST(AttachClient, StartupWeldMustBeReleasedAndConfirmed)
{
  AttachClient c(cfg());
  c.beginStartupGuard(0.0);
  EXPECT_EQ(c.tick(0.0), WeldAction::RequestDetach);
  EXPECT_FALSE(c.startupVerified());
  c.observe(WeldState::Detached, 0.2);
  EXPECT_EQ(c.tick(0.2), WeldAction::Confirmed);
  EXPECT_TRUE(c.startupVerified());
  EXPECT_EQ(c.outcome(), "startup_released");
}

// The weld never releases: fail, do not proceed on the assumption that it did.
TEST(AttachClient, StartupWeldThatNeverReleasesFails)
{
  AttachClient c(cfg());
  c.beginStartupGuard(0.0);
  c.observe(WeldState::Attached, 0.1);
  c.observe(WeldState::Attached, 4.9);
  EXPECT_EQ(c.tick(5.1), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "weld_not_released");
}

// No state at all: "unknown" must never be read as "detached".
TEST(AttachClient, StartupWithNoWeldStateFailsAsUnknown)
{
  AttachClient c(cfg());
  c.beginStartupGuard(0.0);
  EXPECT_EQ(c.tick(5.1), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "weld_state_unknown");
}

// Before the guard passes there is no legal attach at all.
TEST(AttachClient, AttachIsRefusedBeforeTheStartupRelease)
{
  AttachClient c(cfg());
  c.beginStartupGuard(0.0);
  c.observe(WeldState::Detached, 0.1);
  c.requestAttach(0.1);   // guard has not been ticked to confirmed yet
  EXPECT_EQ(c.tick(0.1), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "attach_refused_before_startup_release");
}

// A confirmed release with a small snap is a good attach.
TEST(AttachClient, AttachWithSnapInsideTheFrozenCriterionIsConfirmed)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  EXPECT_EQ(c.tick(0.2), WeldAction::RequestAttach);
  c.setMeasuredSnap(0.001);
  c.observe(WeldState::Attached, 0.3);
  EXPECT_EQ(c.tick(0.3), WeldAction::Confirmed);
  EXPECT_EQ(c.outcome(), "attached");
  EXPECT_TRUE(c.done());
}

// Boundary of the frozen 2 mm criterion (inclusive).
TEST(AttachClient, SnapExactlyAtTheCriterionIsAccepted)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  c.tick(0.2);
  c.setMeasuredSnap(0.002);
  c.observe(WeldState::Attached, 0.3);
  EXPECT_EQ(c.tick(0.3), WeldAction::Confirmed);
}

// W6-0 measured a 0.475 m teleport when the startup weld is relied on; any snap
// beyond the criterion must be a failure, and must require a detach before retry.
TEST(AttachClient, AttachThatSnapsBeyondTheCriterionFailsAndRequiresDetach)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  c.tick(0.2);
  c.setMeasuredSnap(0.475);
  c.observe(WeldState::Attached, 0.3);
  EXPECT_EQ(c.tick(0.3), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "attach_snap_exceeded");
  // Retrying without detaching is refused, and stays refused.
  c.requestAttach(0.4);
  EXPECT_TRUE(c.failed());
  EXPECT_EQ(c.outcome(), "attach_snap_exceeded");
}

// "Not measured" is not "no snap".
TEST(AttachClient, AttachWithoutASnapMeasurementFails)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  c.tick(0.2);
  c.observe(WeldState::Attached, 0.3);
  EXPECT_EQ(c.tick(0.3), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "attach_snap_unmeasured");
}

// An unconfirmed attach is retried a bounded number of times, then failed.
TEST(AttachClient, AttachThatIsNeverConfirmedFailsAfterBoundedRetries)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  EXPECT_EQ(c.tick(0.2), WeldAction::RequestAttach);
  WeldAction a = WeldAction::Idle;
  double t = 0.2;
  for (int i = 0; i < 20 && !c.failed(); ++i) {
    t += 0.1;
    a = c.tick(t);
  }
  EXPECT_EQ(a, WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "attach_not_confirmed");
  EXPECT_EQ(c.attempts(), 3);
}

// A stale "attached" sample cannot support a confirmation: the client keeps
// waiting (and retrying) instead of accepting evidence it cannot trust.
TEST(AttachClient, StaleAttachedSampleDoesNotConfirm)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  c.tick(0.2);
  c.setMeasuredSnap(0.0005);
  c.observe(WeldState::Attached, 0.25);
  EXPECT_EQ(c.tick(0.9), WeldAction::Wait) << "0.65 s old sample is not evidence";
  EXPECT_FALSE(c.done());
  EXPECT_NE(c.outcome(), "attached");
}

// Detach is confirmed only by a fresh "detached".
TEST(AttachClient, DetachNeedsAFreshConfirmation)
{
  AttachClient c = releasedAt();
  c.requestDetach(1.0);
  EXPECT_EQ(c.tick(1.0), WeldAction::RequestDetach);
  c.observe(WeldState::Detached, 1.2);
  EXPECT_EQ(c.tick(1.2), WeldAction::Confirmed);
  EXPECT_EQ(c.outcome(), "detached");
}

// A failed detach must not be reported as a release.
TEST(AttachClient, DetachThatNeverConfirmsFails)
{
  AttachClient c = releasedAt();
  c.requestDetach(1.0);
  c.tick(1.0);
  c.observe(WeldState::Attached, 1.4);
  EXPECT_EQ(c.tick(1.6), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "detach_not_confirmed");
  EXPECT_TRUE(c.startupVerified());
}

// Attach requires a known-detached weld, not merely "no attached message yet".
TEST(AttachClient, AttachIsRefusedWhenTheWeldStateIsStale)
{
  AttachClient c = releasedAt(0.1);
  c.requestAttach(1.0);   // last sample was at 0.1 s, stale_s = 0.25
  EXPECT_EQ(c.tick(1.0), WeldAction::Failed);
  EXPECT_EQ(c.outcome(), "attach_refused_weld_state_unknown");
}

// reset() must not leave a verified release or a live attempt behind.
TEST(AttachClient, ResetClearsVerificationAndAttempts)
{
  AttachClient c = releasedAt();
  c.requestAttach(0.2);
  c.tick(0.2);
  c.reset();
  EXPECT_FALSE(c.startupVerified());
  EXPECT_EQ(c.attempts(), 0);
  EXPECT_EQ(c.state(), WeldState::Unknown);
  EXPECT_FALSE(c.done());
}

// The default configuration carries the frozen criterion verbatim.
TEST(AttachClientConfig, DefaultsMatchTheFrozenCriterion)
{
  AttachClientConfig c;
  EXPECT_DOUBLE_EQ(c.max_snap_m, 0.002) << "C4.3_S4_执行规划.md 14.3 C3";
  EXPECT_EQ(c.attach_topic, "/c43/attach");
  EXPECT_EQ(c.detach_topic, "/c43/detach");
  EXPECT_EQ(c.state_topic, "/c43/weld_state");
}
