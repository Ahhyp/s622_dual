// R1-T07 (seven core faults), R1-T08 (E2 / E7 / E8 paths), R1-T09 (stale,
// out-of-order, duplicate and unparseable status).
//
// Scope honesty: this file verifies that the EXECUTOR reacts correctly to a
// fault EVENT.  It does NOT verify the fault DETECTOR in the controller (E7's
// detector is still Missing there), and it is evidence class "unit test" /
// "bench behaviour" only - never "real time" or "real actuator safety".
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport_executor/precheck.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

using dual_arm_transport_executor::Config;
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

  void primeAndStart()
  {
    fake.init(now, kDt);
    sm.tick(now, fake.ptr());
    StartRequest req;
    req.duration_s = 0.05;
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
  void tickNoStatus(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      now += kDt;
      sm.tick(now, nullptr);
    }
  }
};

TEST(FaultInjection, T07_SevenCoreFaultsMapToTheContractState)
{
  struct Case
  {
    const char * name;
    const char * code;
    bool via_status;
    State expect;
  };
  const Case cases[] = {
    {"attach failed", "attach_failed", true, State::Fault},
    {"sample position deviation", "grasp_check_failed", true, State::Fault},
    {"object hits the table", "object_table_clearance", true, State::Abort},
    {"follower overrun", "follower_divergence", true, State::Abort},
    {"solve failure streak", "qp_solver_failed", true, State::Hold},
    {"status expired", "stale_state", false, State::Hold},
    {"controller lost", "status_never_received", false, State::Hold},
  };

  for (const auto & c : cases) {
    Rig r;
    r.primeAndStart();
    ASSERT_EQ(r.sm.state(), State::Approach) << c.name;
    if (c.via_status) {
      r.fake.status().failure_code = c.code;
      r.tick(1);
    } else if (std::string(c.code) == "stale_state") {
      r.now += 1.0;   // let the status age; do NOT let the fake refresh its stamp
      r.sm.tick(r.now, r.fake.ptr());
    } else {
      r.tickNoStatus(10);
    }
    EXPECT_EQ(r.sm.state(), c.expect) << c.name;
    EXPECT_EQ(r.sm.failureCode(), c.code) << c.name;
    // Every fault must be reproducible and explicit.
    EXPECT_TRUE(r.sm.latched()) << c.name;
  }
}

TEST(FaultInjection, T08a_E2_StatusStalenessIsDetectedByTheExecutor)
{
  Rig r;
  r.primeAndStart();
  // Still inside the three-period window: no protection.
  r.now += 0.016;
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Approach);

  // Beyond it: HOLD.  (The controller reports E2 as Degraded; this is the
  // executor's own, independent check and is what makes it non-vacuous.)
  r.now += 0.050;
  r.sm.tick(r.now, r.fake.ptr());
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "stale_state");
}

TEST(FaultInjection, T08b_E7_CollisionCodeIsPropagatedButTheDetectorIsStillMissing)
{
  Rig r;
  r.primeAndStart();
  r.fake.status().failure_code = "collision";
  r.tick(1);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "collision");
  // Documented gap: plan §4.7.1 E7 is reported as "Missing" by the controller
  // (config has collision_d_stop = 0.0), so this only proves propagation.
}

TEST(FaultInjection, T08c_E8_ExecutorHeartbeatLossHoldsTheTask)
{
  Rig r;
  r.primeAndStart();
  // On heartbeat loss the CONTROLLER freezes itself (that is what E8 means), so
  // the stand-in must obey; a controller that kept advancing would be a
  // different (and worse) fault, covered by T04.
  r.fake.obeyHold();
  r.fake.status().executor_heartbeat_required = true;

  // A fresh heartbeat keeps the task running.
  r.sm.onHeartbeat(r.now);
  r.tick(3);
  EXPECT_EQ(r.sm.state(), State::Approach);

  // No heartbeat for longer than 100 ms -> HOLD.
  r.tick(20);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "executor_lost");
}

TEST(FaultInjection, T08d_HeartbeatRecoveryDoesNotAutoResume)
{
  Rig r;
  r.primeAndStart();
  r.fake.obeyHold();
  r.fake.status().executor_heartbeat_required = true;
  r.tick(20);
  ASSERT_EQ(r.sm.state(), State::Hold);

  r.sm.onHeartbeat(r.now);
  r.tick(5);
  EXPECT_EQ(r.sm.state(), State::Hold) << "HOLD must need an explicit resume";

  std::string why;
  EXPECT_TRUE(r.sm.resume(r.now, &why)) << why;
  EXPECT_EQ(r.sm.state(), State::Approach);
}

TEST(FaultInjection, T09_StaleOutOfOrderDuplicateAndUnparseable)
{
  {
    Rig r;
    r.primeAndStart();
    // Duplicate (same stamp, still fresh): no state change.
    r.sm.tick(r.now, r.fake.ptr());
    r.sm.tick(r.now, r.fake.ptr());
    EXPECT_EQ(r.sm.state(), State::Approach);
  }
  {
    Rig r;
    r.primeAndStart();
    // Out-of-order: an older stamp looks stale to the freshness check.
    r.fake.status().stamp_s = r.now - 0.5;
    r.sm.tick(r.now, r.fake.ptr());
    EXPECT_EQ(r.sm.state(), State::Hold);
    EXPECT_EQ(r.sm.failureCode(), "stale_state");
  }
  {
    Rig r;
    r.primeAndStart();
    r.fake.status().parse_ok = false;
    r.sm.tick(r.now, r.fake.ptr());
    EXPECT_EQ(r.sm.state(), State::Hold);
    EXPECT_EQ(r.sm.failureCode(), "status_unparseable");
  }
}

TEST(FaultInjection, T09b_PrecheckRefusalEntersFaultNotSilentContinue)
{
  Rig r;
  r.fake.init(r.now, kDt);
  r.sm.tick(r.now, r.fake.ptr());
  r.fake.status().active_limit_count = 2;   // precheck must refuse
  r.sm.tick(r.now, r.fake.ptr());           // deliver that observation first
  std::string why;
  StartRequest req;
  EXPECT_FALSE(r.sm.start(r.now, req, &why));
  EXPECT_EQ(r.sm.state(), State::Fault);
  EXPECT_EQ(r.sm.failureCode(), "precheck_failed");
  EXPECT_EQ(why, "precheck_failed");
}

TEST(FaultInjection, T09c_UnknownControllerCodeIsTreatedConservativelyAsHold)
{
  Rig r;
  r.primeAndStart();
  r.fake.status().failure_code = "some_future_code_not_in_the_contract";
  r.tick(1);
  EXPECT_EQ(r.sm.state(), State::Hold);
}

}  // namespace
