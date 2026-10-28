// C4.3 S4 W3 -- HOLD ack policy, covered in a hooks=OFF build.
//
// WHY: the readState-failure branch cannot be reached end-to-end without a fault
// injection, and fault injection is compiled out of the production library.  The
// production code calls exactly these functions, so this test pins the decision
// in the same build that ships (C43_ENABLE_TEST_HOOKS=OFF).  T_H17/T_H17a remain
// the wiring evidence in a hooks=ON build; neither replaces the other.

#include <gtest/gtest.h>

#include <cstdint>

#include "dual_arm_qp_controller/hold_ack_policy.hpp"

using dual_arm_qp_controller::ackForReadStateFailure;
using dual_arm_qp_controller::kAckApplied;
using dual_arm_qp_controller::kAckRefused;
using dual_arm_qp_controller::packHoldAck;
using dual_arm_qp_controller::replayAckResult;

TEST(HoldAckPolicy, AckCodesAreDistinctAndReserved)
{
  // 0 is the protocol's "no ack yet" sentinel: neither verdict may collide with it.
  EXPECT_EQ(kAckApplied, 1u);
  EXPECT_EQ(kAckRefused, 2u);
  EXPECT_NE(kAckApplied, kAckRefused);
}

TEST(HoldAckPolicy, NoQueuedRequestMeansNoAckAtAll)
{
  const auto a = ackForReadStateFailure(0u);
  EXPECT_FALSE(a.publish) << "an empty slot must not fabricate an ack";
  EXPECT_EQ(a.seq, 0u);
  EXPECT_EQ(a.result, 0u);
}

TEST(HoldAckPolicy, QueuedRequestIsDrainedAndRefused)
{
  const auto a = ackForReadStateFailure(4242u);
  EXPECT_TRUE(a.publish);
  EXPECT_EQ(a.seq, 4242u) << "the ack must echo the seq so the executor can match it";
  EXPECT_EQ(a.result, kAckRefused) << "nothing was frozen: a refused ack is the only honest answer";
}

TEST(HoldAckPolicy, RefusedAckIsNeverPackableAsApplied)
{
  const auto a = ackForReadStateFailure(7u);
  const std::uint64_t packed = packHoldAck(a.seq, a.result);
  EXPECT_EQ(static_cast<std::uint32_t>(packed & 0xFFFFFFFFu), kAckRefused);
  EXPECT_EQ(static_cast<std::uint32_t>(packed >> 32), 7u);
}

TEST(HoldAckPolicy, ReplayIsAppliedOnlyWhileTheTrajectoryIsHeld)
{
  EXPECT_EQ(replayAckResult(/*still_held=*/true), kAckApplied);
  EXPECT_EQ(replayAckResult(/*still_held=*/false), kAckRefused)
    << "COMPLETE stops step() but keeps committing q_cmd: refused, not applied";
}
