// C4.3 S4.1 W3-R2 — phase driver tests.
//
// Fixed-step, no clock read: every case drives an explicit t_traj, so the
// pass/fail result is reproducible on any machine.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dual_arm_transport_executor/phase_driver.hpp"

namespace due = dual_arm_transport_executor;

namespace
{

std::vector<due::PhaseSpec> fourPhaseTrajectory()
{
  // 0.30 + 1.50 + 1.50 + 1.50 + 0.30 = 5.10 s, the S4 frozen transport shape.
  return {
    {"approach", 0.30},
    {"lift", 1.50},
    {"transport", 1.50},
    {"lower", 1.50},
    {"settle", 0.30},
  };
}

}  // namespace

TEST(PhaseDriver, DisabledDriverReportsNotStarted)
{
  due::PhaseDriver d;
  EXPECT_FALSE(d.enabled());
  EXPECT_EQ(d.count(), 0);
  EXPECT_EQ(d.index(), -1);
  EXPECT_EQ(d.indexAt(0.0), -1);
  EXPECT_EQ(d.totalDuration(), 0.0);
  const auto o = d.observeLocal(1.0);
  EXPECT_EQ(o.event, due::PhaseEvent::NotStarted);
  EXPECT_EQ(o.index, -1);
}

TEST(PhaseDriver, BoundariesAreCumulative)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  ASSERT_TRUE(d.enabled());
  EXPECT_EQ(d.count(), 5);
  EXPECT_DOUBLE_EQ(d.startOf(0), 0.0);
  EXPECT_DOUBLE_EQ(d.startOf(1), 0.30);
  EXPECT_DOUBLE_EQ(d.startOf(2), 1.80);
  EXPECT_DOUBLE_EQ(d.startOf(3), 3.30);
  EXPECT_DOUBLE_EQ(d.startOf(4), 4.80);
  EXPECT_DOUBLE_EQ(d.startOf(5), 5.10);
  EXPECT_DOUBLE_EQ(d.totalDuration(), 5.10);
}

TEST(PhaseDriver, IndexAtUsesHalfOpenIntervals)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.indexAt(-1.0), -1);
  EXPECT_EQ(d.indexAt(0.0), 0);
  EXPECT_EQ(d.indexAt(0.299), 0);
  EXPECT_EQ(d.indexAt(0.30), 1);   // boundary belongs to the next phase
  EXPECT_EQ(d.indexAt(1.80), 2);
  EXPECT_EQ(d.indexAt(3.30), 3);
  EXPECT_EQ(d.indexAt(4.80), 4);
  EXPECT_EQ(d.indexAt(5.10), 4);   // past the end: stay on the last phase
  EXPECT_EQ(d.indexAt(99.0), 4);
}

TEST(PhaseDriver, LocalAdvanceEmitsOneEnteredPerPhase)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  const std::vector<double> samples = {0.0, 0.1, 0.30, 0.5, 1.80, 2.0, 3.30, 3.5, 4.80, 5.0, 5.1};
  std::vector<int> entered;
  for (double t : samples) {
    const auto o = d.observeLocal(t);
    if (o.event == due::PhaseEvent::Entered) {
      entered.push_back(o.index);
      EXPECT_FALSE(o.name.empty());
      EXPECT_DOUBLE_EQ(o.duration_s, d.phases()[static_cast<std::size_t>(o.index)].duration_s);
    } else {
      EXPECT_EQ(o.event, due::PhaseEvent::None);
    }
  }
  EXPECT_EQ(entered, (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_EQ(d.index(), 4);
}

TEST(PhaseDriver, SameTimeTwiceEmitsNoSecondEvent)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::None);
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::None);
  EXPECT_EQ(d.index(), 0);
}

TEST(PhaseDriver, ForwardJumpIsDesync)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
  const auto o = d.observeLocal(1.90);  // 0 -> 2, skipping phase 1
  EXPECT_EQ(o.event, due::PhaseEvent::Desync);
  EXPECT_EQ(o.index, 2);
  EXPECT_EQ(d.index(), 2);
}

TEST(PhaseDriver, RegressionIsDesync)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeLocal(0.3).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeLocal(1.8).event, due::PhaseEvent::Entered);
  const auto o = d.observeLocal(0.1);
  EXPECT_EQ(o.event, due::PhaseEvent::Desync);
  EXPECT_EQ(o.index, 0);
}

TEST(PhaseDriver, JoinMidTrajectoryIsDesync)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  const auto o = d.observeLocal(2.0);  // first observation already in phase 2
  EXPECT_EQ(o.event, due::PhaseEvent::Desync);
  EXPECT_EQ(o.index, 2);
}

TEST(PhaseDriver, UnknownPhaseNameIsReported)
{
  std::vector<due::PhaseSpec> phases = {{"approach", 0.3}, {"teleport", 0.5}};
  due::PhaseDriver d(phases);
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
  const auto o = d.observeLocal(0.4);
  EXPECT_EQ(o.event, due::PhaseEvent::UnknownName);
  EXPECT_EQ(o.name, "teleport");
  EXPECT_TRUE(due::isKnownPhaseName("lower"));
  EXPECT_FALSE(due::isKnownPhaseName("teleport"));
}

TEST(PhaseDriver, ExternalIndexAdvancesOnlyByOne)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeExternal(0, "approach", 0.0).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeExternal(0, "approach", 0.1).event, due::PhaseEvent::None);
  EXPECT_EQ(d.observeExternal(1, "lift", 0.3).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeExternal(2, "transport", 1.8).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.index(), 2);
}

TEST(PhaseDriver, ExternalJumpAndBackwardsAreDesync)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeExternal(0, "approach", 0.0).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeExternal(3, "lower", 3.4).event, due::PhaseEvent::Desync);
  EXPECT_EQ(d.observeExternal(1, "lift", 3.4).event, due::PhaseEvent::Desync);
  EXPECT_EQ(d.index(), 1);
}

TEST(PhaseDriver, ExternalNameMustMatchTheLocalTable)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeExternal(0, "approach", 0.0).event, due::PhaseEvent::Entered);
  // index 1 is "lift" locally; a controller that calls it "teleport" is lying.
  EXPECT_EQ(d.observeExternal(1, "teleport", 0.3).event, due::PhaseEvent::UnknownName);
  EXPECT_EQ(d.index(), 0);  // the index was not adopted
  // A known name at the wrong slot is also a mismatch, not an advance.
  EXPECT_EQ(d.observeExternal(1, "transport", 0.3).event, due::PhaseEvent::UnknownName);
  EXPECT_EQ(d.observeExternal(1, "lift", 0.3).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.index(), 1);
}

TEST(PhaseDriver, ExternalOutOfRangeIsDesyncWithoutAdoption)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeExternal(0, "approach", 0.0).event, due::PhaseEvent::Entered);
  const auto o = d.observeExternal(7, "lower", 0.5);
  EXPECT_EQ(o.event, due::PhaseEvent::Desync);
  EXPECT_EQ(d.index(), 0);  // refused to adopt an impossible index
}

TEST(PhaseDriver, ResetClearsIndexAndAllowsReplay)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
  EXPECT_EQ(d.observeLocal(0.3).event, due::PhaseEvent::Entered);
  d.reset();
  EXPECT_EQ(d.index(), -1);
  EXPECT_EQ(d.observeLocal(0.0).event, due::PhaseEvent::Entered);
}

TEST(PhaseDriver, ElapsedIsClampedToThePhase)
{
  due::PhaseDriver d(fourPhaseTrajectory());
  const auto o = d.observeLocal(0.29);
  EXPECT_EQ(o.event, due::PhaseEvent::Entered);
  EXPECT_NEAR(o.elapsed_s, 0.29, 1e-12);
  EXPECT_DOUBLE_EQ(o.duration_s, 0.30);
  EXPECT_EQ(o.count, 5);
}
