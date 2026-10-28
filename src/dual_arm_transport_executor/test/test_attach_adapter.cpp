// C4.3 S4 D5-2 / R3 -- attach adapter tests (software-only simulation).
//
// No clock is read here: every case passes an explicit now_s, so the completion
// and timeout results are reproducible.  The adapter never claims a physical
// attachment, and these tests pin that the only success tag is "simulated".
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport_executor/attach_adapter.hpp"

namespace dae = dual_arm_transport_executor;

TEST(AttachAdapter, StartsUnattached)
{
  dae::AttachAdapter a;
  EXPECT_EQ(a.state(), dae::AttachState::Unattached);
  EXPECT_FALSE(a.awaitingCompletion());
  EXPECT_EQ(a.failureCode(), "");
  EXPECT_EQ(a.requestedAtS(), 0.0);
  EXPECT_EQ(a.artifactTag(), "none");
}

TEST(AttachAdapter, HappyPathReachesAttachedSimulated)
{
  dae::AttachAdapter a;
  const dae::AttachResult r = a.request(1.0);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.reason, "");
  EXPECT_EQ(a.state(), dae::AttachState::Requested);
  EXPECT_TRUE(a.awaitingCompletion());
  EXPECT_EQ(a.requestedAtS(), 1.0);

  a.tick(1.1);
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);
  EXPECT_FALSE(a.awaitingCompletion());
  EXPECT_EQ(a.failureCode(), "");
}

TEST(AttachAdapter, ArtifactTagIsSimulatedOnlyWhenAttached)
{
  dae::AttachAdapter a;
  EXPECT_EQ(a.artifactTag(), "none");
  ASSERT_TRUE(a.request(0.0).ok);
  EXPECT_EQ(a.artifactTag(), "none");  // requested is not attached
  a.tick(0.1);
  EXPECT_EQ(a.artifactTag(), "simulated");

  a.reset();
  ASSERT_TRUE(a.request(1.0).ok);
  a.injectFailure("gripper_no_contact");
  EXPECT_EQ(a.artifactTag(), "none");  // a failure is never an attachment
}

TEST(AttachAdapter, SecondRequestWhileRequestedIsBusy)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(2.0).ok);
  const dae::AttachResult r = a.request(2.1);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "attach_busy");
  EXPECT_EQ(a.state(), dae::AttachState::Requested);
  EXPECT_EQ(a.requestedAtS(), 2.0);  // the pending request is untouched
}

TEST(AttachAdapter, RequestWhileAttachedIsRejected)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(0.0).ok);
  a.tick(0.1);
  ASSERT_EQ(a.state(), dae::AttachState::AttachedSimulated);

  const dae::AttachResult r = a.request(0.2);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "already_attached");
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);
  a.tick(9.0);  // late tick must not turn an attachment into a timeout
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);
}

TEST(AttachAdapter, TimeoutIsStrictlyGreaterThanConfigured)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(0.0).ok);
  a.tick(0.5);  // exactly the default timeout is still allowed to complete
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);

  a.reset();
  ASSERT_TRUE(a.request(10.0).ok);
  a.tick(10.5001);
  EXPECT_EQ(a.state(), dae::AttachState::Failed);
  EXPECT_EQ(a.failureCode(), "attach_timeout");
  EXPECT_FALSE(a.awaitingCompletion());
  EXPECT_EQ(a.artifactTag(), "none");
  EXPECT_EQ(a.requestedAtS(), 10.0);  // kept for the audit trail
}

TEST(AttachAdapter, InjectionWhileRequestedFailsWithCode)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(3.0).ok);
  a.injectFailure("gripper_no_contact");
  EXPECT_EQ(a.state(), dae::AttachState::Failed);
  EXPECT_EQ(a.failureCode(), "gripper_no_contact");
  EXPECT_FALSE(a.awaitingCompletion());
  a.tick(3.1);  // cannot resurrect a failed request
  EXPECT_EQ(a.state(), dae::AttachState::Failed);
  EXPECT_EQ(a.failureCode(), "gripper_no_contact");
}

TEST(AttachAdapter, InjectionOutsideRequestedIsNoOp)
{
  dae::AttachAdapter a;
  a.injectFailure("too_early");
  EXPECT_EQ(a.state(), dae::AttachState::Unattached);
  EXPECT_EQ(a.failureCode(), "");

  ASSERT_TRUE(a.request(0.0).ok);
  a.tick(0.1);
  a.injectFailure("too_late");
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);
  EXPECT_EQ(a.failureCode(), "");

  a.reset();  // the attached state rejects requests, so reset before retrying
  ASSERT_TRUE(a.request(1.0).ok);
  a.tick(1.6);  // timeout failure
  ASSERT_EQ(a.state(), dae::AttachState::Failed);
  a.injectFailure("overwrite_attempt");
  EXPECT_EQ(a.failureCode(), "attach_timeout");  // original code preserved
}

TEST(AttachAdapter, RetryAfterFailureClearsCode)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(5.0).ok);
  a.injectFailure("gripper_no_contact");
  ASSERT_EQ(a.failureCode(), "gripper_no_contact");

  const dae::AttachResult r = a.request(5.5);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.reason, "");
  EXPECT_EQ(a.state(), dae::AttachState::Requested);
  EXPECT_EQ(a.failureCode(), "");
  EXPECT_EQ(a.requestedAtS(), 5.5);

  a.tick(5.6);
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);
}

TEST(AttachAdapter, ResetClearsEverything)
{
  dae::AttachAdapter a;
  ASSERT_TRUE(a.request(2.0).ok);
  a.injectFailure("gripper_no_contact");
  ASSERT_EQ(a.state(), dae::AttachState::Failed);

  a.reset();
  EXPECT_EQ(a.state(), dae::AttachState::Unattached);
  EXPECT_EQ(a.failureCode(), "");
  EXPECT_EQ(a.requestedAtS(), 0.0);
  EXPECT_FALSE(a.awaitingCompletion());
  EXPECT_EQ(a.artifactTag(), "none");
  a.tick(2.1);  // no pending request, nothing to complete
  EXPECT_EQ(a.state(), dae::AttachState::Unattached);
}

TEST(AttachAdapter, ConfigureChangesTheTimeout)
{
  dae::AttachAdapter a;
  a.configure(0.25);
  ASSERT_TRUE(a.request(1.0).ok);
  a.tick(1.2499);
  EXPECT_EQ(a.state(), dae::AttachState::AttachedSimulated);

  a.reset();
  a.configure(0.25);
  ASSERT_TRUE(a.request(2.0).ok);
  a.tick(2.26);
  EXPECT_EQ(a.state(), dae::AttachState::Failed);
  EXPECT_EQ(a.failureCode(), "attach_timeout");

  // A non-positive timeout must not disable the wait.
  a.reset();
  a.configure(0.0);
  ASSERT_TRUE(a.request(3.0).ok);
  a.tick(3.6);
  EXPECT_EQ(a.state(), dae::AttachState::Failed);
}
