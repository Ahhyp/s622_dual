#include <gtest/gtest.h>
#include "dual_arm_transport_executor/state_machine.hpp"
using namespace dual_arm_transport_executor;

struct ContactTest : testing::Test {
  Config config() {Config c;c.physical_contact=true;c.require_grasp_check=true;
    c.approach_timeout_s=45;c.grasp_timeout_s=40;return c;}
  TransportStateMachine sm{config()};ContactObservation o;
  double now=1;
  void observe() {o.stamp_s=o.contact_stamp_s=now;sm.noteContactObservation(o);}
  void ready() {o.ready=true;o.source_support=true;o.robot_clear=true;o.arms_jtc_active=true;observe();}
  void grasp() {
    ready();ASSERT_TRUE(sm.start(now,{27},nullptr));o.command_phase="APPROACH";o.step_done=true;
    observe();ASSERT_TRUE(sm.phaseDone(now,nullptr));
    o.command_phase="GRASP";o.all_four=true;o.robot_clear=false;o.arms_jtc_active=false;o.qp_active=true;
    observe();sm.noteGripperClosed(now,true,"four contacts");sm.noteGraspCheck(now,true,"ownership");
  }
  void lower() {
    grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));
    static PhaseDriver driver;driver.configure({{"lift",6},{"settle",5},{"transport",8},{"lower",6},{"settle_post",2}});sm.attachPhaseDriver(&driver);
    for(int i=0;i<4;++i){now+=.01;observe();ControllerStatus s;s.received=true;s.parse_ok=true;s.stamp_s=now;
      s.transport_active=true;s.state="RUNNING";s.phase_index=i;s.phase_name=driver.phases()[i].name;
      s.t_traj=driver.startOf(i);s.duration=27;s.min_limit_margin_rad=1;
      sm.tick(now,&s);}
    ASSERT_EQ(sm.state(),State::Lower);
  }
  ControllerStatus hold(unsigned seq=17) {ControllerStatus s;s.received=true;s.parse_ok=true;s.stamp_s=now;
    s.transport_active=true;s.state="HOLD";s.failure_code="hold_by_executor";
    s.hold_origin="executor";s.hold_request_seq=seq;s.hold_request_result=1;return s;}
};
TEST_F(ContactTest,NoReadinessNoStart) {EXPECT_FALSE(sm.start(now,{27},nullptr));}
TEST_F(ContactTest,FourContactsRequiredEvenForExplicitPhaseDone) {
  grasp();o.all_four=false;observe();EXPECT_FALSE(sm.phaseDone(now,nullptr));EXPECT_EQ(sm.state(),State::Grasp);
}
TEST_F(ContactTest,OwnershipConflictCannotAdvance) {
  grasp();o.arms_jtc_active=true;observe();EXPECT_FALSE(sm.phaseDone(now,nullptr));
}
TEST_F(ContactTest,WrongPhaseCompletionCannotAdvance) {
  grasp();o.command_phase="APPROACH";observe();EXPECT_FALSE(sm.phaseDone(now,nullptr));
}
TEST_F(ContactTest,ControllerCompleteIsNotSupportProof) {
  lower();ControllerStatus s;s.received=true;s.stamp_s=now;s.state="COMPLETE";
  sm.tick(now,&s);EXPECT_EQ(sm.state(),State::Fault);EXPECT_EQ(sm.failureCode(),"release_failed");
}
TEST_F(ContactTest,WrongAckCannotReleaseOrAutoClear) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold(16);sm.tick(now,&s);EXPECT_EQ(sm.state(),State::Hold);
  s=hold(17);sm.tick(now,&s);EXPECT_NE(sm.state(),State::Release);
}
TEST_F(ContactTest,ControllerOriginHoldCannotAuthorizeRelease) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold();s.hold_origin="controller";sm.tick(now,&s);EXPECT_NE(sm.state(),State::Release);
}
TEST_F(ContactTest,StaleSupportCannotAuthorizeRelease) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  now+=.2;auto s=hold();sm.tick(now,&s);EXPECT_EQ(sm.state(),State::Hold);
}
TEST_F(ContactTest,SupportedAckThenOpenThenRetreat) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold();sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Release);
  EXPECT_FALSE(sm.phaseDone(now,nullptr));
  o.command_phase="RELEASE";o.fingers_open=true;o.qp_active=false;o.arms_jtc_active=true;observe();
  ASSERT_TRUE(sm.phaseDone(now,nullptr));ASSERT_EQ(sm.state(),State::Retreat);
  EXPECT_FALSE(sm.phaseDone(now,nullptr));o.command_phase="RETREAT";o.robot_clear=true;observe();
  EXPECT_TRUE(sm.phaseDone(now,nullptr));EXPECT_EQ(sm.state(),State::Complete);
}
TEST_F(ContactTest,MissingSupportCannotReleaseOnCorrectAck) {
  lower();o.destination_support=false;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold();sm.tick(now,&s);EXPECT_NE(sm.state(),State::Release);
}
TEST_F(ContactTest,UnparseableAckCannotAuthorizeRelease) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold();s.parse_ok=false;sm.tick(now,&s);EXPECT_EQ(sm.state(),State::Hold);
}
TEST_F(ContactTest,FutureAckCannotAuthorizeRelease) {
  lower();o.destination_support=true;o.in_target=true;observe();sm.expectSupportHoldAck(17);
  auto s=hold();s.stamp_s=now+1;sm.tick(now,&s);EXPECT_NE(sm.state(),State::Release);
}
TEST_F(ContactTest,LostContactDuringCarryFaults) {
  lower();o.all_four=false;observe();auto s=hold();
  sm.tick(now,&s);EXPECT_EQ(sm.state(),State::Fault);EXPECT_EQ(sm.failureCode(),"grasp_check_failed");
}

TEST_F(ContactTest,ExplicitHoldRequiresAppliedRunningAckToResume) {
  grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));sm.expectHoldAck(17);
  auto s=hold();s.min_limit_margin_rad=1;sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Hold);
  std::string why;EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,&why));
  s.state="RUNNING";s.failure_code="";s.hold_request_seq=16;
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,&why));
  s.hold_request_seq=17;EXPECT_TRUE(sm.resumeAfterControllerAck(now,s,&why));EXPECT_EQ(sm.state(),State::Lift);
}
TEST_F(ContactTest,PhysicalResumeRejectsMissingContact) {
  grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));sm.expectHoldAck(17);
  auto s=hold();s.min_limit_margin_rad=1;sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Hold);
  o.all_four=false;observe();s.state="RUNNING";s.failure_code="";
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));EXPECT_EQ(sm.state(),State::Hold);
}
TEST_F(ContactTest,PhysicalResumeRejectsNonfiniteAndFutureStatus) {
  grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));sm.expectHoldAck(17);
  auto s=hold();s.min_limit_margin_rad=1;sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Hold);
  s.state="RUNNING";s.failure_code="";s.e_obj_pos_m=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));s.e_obj_pos_m=0;s.stamp_s=now+1;
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));
}
TEST_F(ContactTest,PhysicalResumeRejectsForeignOrRefusedAck) {
  grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));sm.expectHoldAck(17);
  auto s=hold();s.min_limit_margin_rad=1;sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Hold);
  s.state="RUNNING";s.failure_code="";s.hold_origin="controller";
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));
  s.hold_origin="executor";s.hold_request_result=2;
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));
}
TEST_F(ContactTest,PhysicalResumeRejectsStaleContactsAndOwnershipConflict) {
  grasp();ASSERT_TRUE(sm.phaseDone(now,nullptr));sm.expectHoldAck(17);
  auto s=hold();s.min_limit_margin_rad=1;sm.tick(now,&s);ASSERT_EQ(sm.state(),State::Hold);
  s.state="RUNNING";s.failure_code="";now+=.2;s.stamp_s=now;
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));
  observe();o.hands_jtc_active=true;observe();
  EXPECT_FALSE(sm.resumeAfterControllerAck(now,s,nullptr));
}
