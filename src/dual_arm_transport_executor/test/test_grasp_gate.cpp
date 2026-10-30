// C4.3 S4 S4-T16 -- GRASP prerequisite gate (pure state machine).
//
// The node closes the gripper and checks the sample; this test pins the ordering
// the state machine must enforce: no attach request until BOTH verdicts are in,
// and a negative grasp verdict is FAULT(grasp_check_failed), never a weld.
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport_executor/state_machine.hpp"

namespace dae = dual_arm_transport_executor;

namespace
{
dae::ControllerStatus graspStatus()
{
  dae::ControllerStatus st;
  st.received = true;
  st.parse_ok = true;
  st.transport_active = true;
  st.duration = 16.0;
  st.t_traj = 3.0;
  st.phase_index = 1;
  st.phase_count = 6;
  st.phase_name = "grasp";
  st.phase_duration_s = 2.0;
  return st;
}

/// start() refuses without a recent, non-active status (precheck contract).
void primeIdle(dae::TransportStateMachine & sm)
{
  dae::ControllerStatus idle = graspStatus();
  idle.transport_active = false;
  idle.phase_index = -1;
  idle.phase_name.clear();
  idle.t_traj = 0.0;
  idle.stamp_s = 0.0;
  sm.tick(0.0, &idle);
}
}  // namespace

TEST(GraspGate, AttachIsNotRequestedUntilBothVerdictsArrive)
{
  dae::Config cfg;
  cfg.status_stale_timeout_s = 10.0;  // the test drives synthetic timestamps
  cfg.require_grasp_check = true;
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  ASSERT_TRUE(sm.phaseDone(0.1, &why)) << why;
  ASSERT_EQ(sm.state(), dae::State::Grasp);

  const dae::ControllerStatus st = graspStatus();
  sm.tick(0.2, &st);
  EXPECT_EQ(sm.state(), dae::State::Grasp);
  EXPECT_FALSE(sm.weldAttached());

  sm.noteGripperClosed(0.3, true, "fingers at target");
  EXPECT_TRUE(sm.gripperReady());
  sm.tick(0.3, &st);
  EXPECT_EQ(sm.state(), dae::State::Grasp) << "grasp check still pending";
  EXPECT_FALSE(sm.weldAttached());

  sm.noteGraspCheck(0.4, true, "pos=0.001 m rot=0.001 rad");
  EXPECT_TRUE(sm.graspCheckDone());
  EXPECT_TRUE(sm.graspOk());
  EXPECT_EQ(sm.state(), dae::State::Grasp);
}

TEST(GraspGate, FiveMillimetreLiftLatchesFaultWithoutAttach)
{
  dae::Config cfg;
  cfg.status_stale_timeout_s = 10.0;  // the test drives synthetic timestamps
  cfg.require_grasp_check = true;
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  ASSERT_TRUE(sm.phaseDone(0.1, &why)) << why;
  ASSERT_EQ(sm.state(), dae::State::Grasp);

  const dae::ControllerStatus st = graspStatus();
  sm.tick(0.2, &st);
  sm.noteGripperClosed(0.3, true, "fingers at target");
  sm.noteGraspCheck(0.4, false, "pos=0.005 m (tol 0.003)");

  EXPECT_EQ(sm.state(), dae::State::Fault);
  EXPECT_EQ(sm.failureCode(), "grasp_check_failed");
  EXPECT_FALSE(sm.weldAttached());
  EXPECT_EQ(sm.weldClient(), nullptr);
}

TEST(GraspGate, GripperFailureAlsoLatchesFault)
{
  dae::Config cfg;
  cfg.status_stale_timeout_s = 10.0;  // the test drives synthetic timestamps
  cfg.require_grasp_check = true;
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  ASSERT_TRUE(sm.phaseDone(0.1, &why)) << why;

  sm.noteGripperClosed(0.2, false, "JTC failed");
  EXPECT_EQ(sm.state(), dae::State::Fault);
  EXPECT_EQ(sm.failureCode(), "grasp_check_failed");
}

TEST(GraspGate, LegacyModeStillAdvancesWithoutVerdicts)
{
  dae::Config cfg;  // require_grasp_check == false
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  EXPECT_TRUE(sm.gripperReady());
  EXPECT_TRUE(sm.graspCheckDone());
  EXPECT_TRUE(sm.graspOk());
}

TEST(GraspGate, SingleJointVelocityCapDuringGraspDoesNotHold)
{
  dae::Config cfg;
  cfg.status_stale_timeout_s = 10.0;
  cfg.limit_saturation_cycles = 3;
  cfg.limit_margin_rad = 0.05;
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  ASSERT_TRUE(sm.phaseDone(0.1, &why)) << why;
  ASSERT_EQ(sm.state(), dae::State::Grasp);

  dae::ControllerStatus st = graspStatus();
  st.active_limit_count = 1;          // finger contact, one joint at the cap
  st.min_limit_margin_rad = 1.0;      // far from any position limit
  for (int i = 0; i < 20; ++i) {
    const double t = 0.2 + 0.008 * i;
    st.stamp_s = t;
    sm.tick(t, &st);
  }
  EXPECT_EQ(sm.state(), dae::State::Grasp)
    << "a single velocity-capped joint during GRASP is not a T1 runaway";
}

TEST(GraspGate, PositionLimitDuringGraspStillHolds)
{
  dae::Config cfg;
  cfg.status_stale_timeout_s = 10.0;
  cfg.limit_saturation_cycles = 3;
  cfg.limit_margin_rad = 0.05;
  dae::TransportStateMachine sm(cfg);
  dae::StartRequest req;
  req.duration_s = 16.0;
  std::string why;
  primeIdle(sm);
  ASSERT_TRUE(sm.start(0.01, req, &why)) << why;
  ASSERT_TRUE(sm.phaseDone(0.1, &why)) << why;
  ASSERT_EQ(sm.state(), dae::State::Grasp);

  dae::ControllerStatus st = graspStatus();
  st.active_limit_count = 0;
  st.min_limit_margin_rad = 0.01;     // inside the 0.05 position band
  for (int i = 0; i < 10; ++i) {
    const double t = 0.2 + 0.008 * i;
    st.stamp_s = t;
    sm.tick(t, &st);
  }
  EXPECT_EQ(sm.state(), dae::State::Hold);
  EXPECT_EQ(sm.failureCode(), "joint_limit_saturation");
}
