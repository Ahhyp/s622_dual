// C4.3 S4 W1 review (A/B/C) regression tests.
//
// Three review items are covered here, and each one is a NEGATIVE-controlable
// statement about what the controller refuses to do:
//   A  the protection report must never claim completeness while a §4.7.1
//      criterion is not actually detected by this controller;
//   B  a transport may not start from an arbitrary pose (the anchored start pose
//      would silently become "wherever the arms are");
//   C  ~/resume is HOLD-only: HOLD -> ABORT after two failed resumes, FAULT
//      needs ~/reset, ABORT needs a new ~/start_transport -- never automatic.
#include <gtest/gtest.h>

#include <cstdio>
#include <limits>
#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm_qp_controller::buildProtectionReport;
using dual_arm_qp_controller::defaultHomeQ;
using dual_arm_qp_controller::executorHeartbeatStale;
using dual_arm_qp_controller::ProtectionCoverage;
using dual_arm_qp_controller::ProtectionReport;
using dual_arm_qp_controller::TransportState;
using dual_arm_qp_controller::TransportTask;
using dual_arm_qp_controller::TransportTaskConfig;

constexpr double kPi = 3.14159265358979323846;

Iso3 objectTLeftTcp() {
    Iso3 T = Iso3::Identity();
    T.linear() << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    T.translation() << 0.0497, 0.0, 0.025;
    return T;
}
Iso3 objectTRightTcp() {
    Iso3 T = Iso3::Identity();
    T.linear() << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    T.translation() << -0.0497, 0.0, 0.025;
    return T;
}

dual_arm_object_task::TrajectorySpec specTowards(const Iso3& start, double dz, double yaw_deg = 0.0) {
    dual_arm_object_task::TrajectorySpec s;
    s.dt = 0.008;
    s.time_scaling = "smoothstep";
    s.start_world_T_object = start;
    s.start_from_baseline = false;
    s.phases.push_back({"move", 2.0, 0.0, 0.0, dz});
    s.yaw_sweep_deg = {yaw_deg};
    return s;
}

class TransportLatches : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_.object_T_left_tcp = objectTLeftTcp();
        cfg_.object_T_right_tcp = objectTRightTcp();
        cfg_.dt = 0.008;
        kin_ = std::make_unique<dual_arm::DualArmKinematics>();
        q_home_ = defaultHomeQ();
    }
    std::unique_ptr<dual_arm::DualArmKinematics> kin_;
    TransportTaskConfig cfg_;
    Q12 q_home_ = Q12::Zero();
};

// ---------------------------------------------------------------------------
// A: the protection report is a measurement, not a claim.
// ---------------------------------------------------------------------------
TEST_F(TransportLatches, A_ProtectionReportNeverClaimsWhatIsNotWatched) {
    // S4.1 default: arm-arm interlock enabled, executor heartbeat not required.
    const ProtectionReport rep = buildProtectionReport(/*e7=*/true, /*e8=*/false);
    std::printf("A: default protection = %s\n", rep.summary.c_str());

    EXPECT_FALSE(rep.complete) << "S4.1 must not report COMPLETE: E2 is degraded and E8 is absent";
    EXPECT_NE(rep.summary.find("PARTIAL"), std::string::npos);
    EXPECT_NE(rep.summary.find("degraded(e2_state_staleness)"), std::string::npos);
    EXPECT_NE(rep.summary.find("missing(e8_executor_heartbeat)"), std::string::npos);

    // Every criterion the plan lists must be present in the report either way,
    // so nothing can be dropped silently.
    ASSERT_EQ(rep.items.size(), 8u);
    int implemented = 0, degraded = 0, missing = 0;
    for (const auto& it : rep.items) {
        std::printf("A:   %-24s %s\n", it.id,
                    it.coverage == ProtectionCoverage::Implemented
                        ? "implemented"
                        : (it.coverage == ProtectionCoverage::Degraded ? "degraded" : "missing"));
        if (it.coverage == ProtectionCoverage::Implemented) ++implemented;
        if (it.coverage == ProtectionCoverage::Degraded) ++degraded;
        if (it.coverage == ProtectionCoverage::Missing) ++missing;
    }
    EXPECT_EQ(degraded, 1);   // e2
    EXPECT_EQ(missing, 1);    // e8
    EXPECT_EQ(implemented + degraded + missing, 8);

    // With both interlocks enabled, E7/E8 become implemented; E2 stays degraded,
    // so the report is STILL not complete -- that is the honest answer.
    const ProtectionReport all_on = buildProtectionReport(true, true);
    std::printf("A: all interlocks on = %s\n", all_on.summary.c_str());
    EXPECT_FALSE(all_on.complete) << "e2 cannot be implemented at the controller layer";
    EXPECT_EQ(all_on.summary.find("missing("), std::string::npos);

    // And with the arm-arm interlock disabled it must say so.
    const ProtectionReport no_gap = buildProtectionReport(false, false);
    EXPECT_NE(no_gap.summary.find("missing(e7_arm_gap,e8_executor_heartbeat)"), std::string::npos);
}

// ---------------------------------------------------------------------------
// A (E8): the heartbeat rule itself, so "e8 implemented" is a tested claim.
// ---------------------------------------------------------------------------
TEST_F(TransportLatches, A2_ExecutorHeartbeatStaleRule) {
    const double t = 100.0;
    // Never heard from the executor -> stale (fail closed, not fail open).
    EXPECT_TRUE(executorHeartbeatStale(/*seen=*/false, -1.0, t, 0.1));
    // Fresh heartbeat -> not stale.
    EXPECT_FALSE(executorHeartbeatStale(true, t - 0.05, t, 0.1));
    // Exactly at the timeout is still acceptable (the plan says "> 100 ms").
    EXPECT_FALSE(executorHeartbeatStale(true, t - 0.1, t, 0.1));
    // Past the timeout -> stale.
    EXPECT_TRUE(executorHeartbeatStale(true, t - 0.101, t, 0.1));
    // A backwards clock step is not evidence of loss.
    EXPECT_FALSE(executorHeartbeatStale(true, t + 5.0, t, 0.1));
    // Non-finite stamps fail closed.
    EXPECT_TRUE(executorHeartbeatStale(true, std::numeric_limits<double>::quiet_NaN(), t, 0.1));
}

// ---------------------------------------------------------------------------
// B: a transport may not start from an arbitrary pose.
// ---------------------------------------------------------------------------
TEST_F(TransportLatches, B_StartIsRefusedAwayFromTheStandingConfiguration) {
    TransportTask task(*kin_, cfg_);
    const Iso3 start = task.builder().leftEstimate(q_home_);

    std::string err;
    ASSERT_TRUE(task.start(specTowards(start, 0.02), q_home_, &err)) << err;
    EXPECT_TRUE(task.startedFromHome());
    EXPECT_EQ(task.startPoseDeviation(), 0.0);

    // 0.2 rad on one joint is 4x the 0.05 rad tolerance.
    Q12 q_bad = q_home_;
    q_bad[3] += 0.2;
    TransportTask strict(*kin_, cfg_);
    err.clear();
    const bool ok = strict.start(specTowards(start, 0.02), q_bad, &err);
    std::printf("B: refused=%d deviation=%.6f err=%s\n", static_cast<int>(!ok),
                strict.startPoseDeviation(), err.c_str());
    EXPECT_FALSE(ok);
    EXPECT_NE(err.find("start pose deviation"), std::string::npos);
    EXPECT_NE(err.find("standing configuration"), std::string::npos);
    EXPECT_GT(strict.startPoseDeviation(), cfg_.start_pose_tolerance_rad);

    // A tolerance of 0 disables the check (documented escape hatch, not default).
    TransportTaskConfig loose = cfg_;
    loose.start_pose_tolerance_rad = 0.0;
    TransportTask task2(*kin_, loose);
    err.clear();
    EXPECT_TRUE(task2.start(specTowards(start, 0.02), q_bad, &err)) << err;
    EXPECT_FALSE(task2.startedFromHome());
}

// ---------------------------------------------------------------------------
// C: latch semantics -- HOLD-only resume, two failures abort, FAULT needs reset.
// ---------------------------------------------------------------------------
TEST_F(TransportLatches, C_ResumeIsHoldOnlyAndTwoFailuresAbort) {
    // Idle -> resume must be refused.
    TransportTask task(*kin_, cfg_);
    EXPECT_NE(std::string(task.state() == TransportState::Idle ? "idle" : "?"), "?");
    EXPECT_FALSE(task.resume(q_home_));
    EXPECT_STREQ(task.failureCode(), "resume_requires_hold");

    // Latch a trajectory whose yaw is 30 deg away from the current anchored
    // estimate: the resume shift only translates, so this error never heals.
    TransportTask task2(*kin_, cfg_);
    const Iso3 start = task2.builder().leftEstimate(q_home_);
    ASSERT_TRUE(task2.start(specTowards(start, 0.02, /*yaw_deg=*/30.0), q_home_, nullptr));
    EXPECT_EQ(task2.state(), TransportState::Running);

    // First failed resume: translates the remaining path, returns to RUNNING.
    task2.hold("test");
    ASSERT_EQ(task2.state(), TransportState::Hold);
    const bool r1 = task2.resume(q_home_);
    std::printf("C: resume#1 = %d state=%d fail_count=%d\n", static_cast<int>(r1),
                static_cast<int>(task2.state()), task2.resumeFailCount());
    EXPECT_TRUE(task2.resumeFailCount() >= 1);

    // Second consecutive failure -> ABORT (not HOLD, not RUNNING).
    task2.hold("test");
    const bool r2 = task2.resume(q_home_);
    std::printf("C: resume#2 = %d state=%d code=%s\n", static_cast<int>(r2),
                static_cast<int>(task2.state()), task2.failureCode());
    EXPECT_FALSE(r2);
    EXPECT_EQ(task2.state(), TransportState::Abort);
    EXPECT_STREQ(task2.failureCode(), "resume_failed");

    // ABORT is not resumable; only a new start (or reset) clears it.
    EXPECT_FALSE(task2.resume(q_home_));
    EXPECT_STREQ(task2.failureCode(), "abort_requires_start");
    EXPECT_EQ(task2.state(), TransportState::Abort);

    // FAULT is not resumable either: it needs ~/reset_transport.
    task2.fault("unit_test_fault");
    EXPECT_FALSE(task2.resume(q_home_));
    EXPECT_STREQ(task2.failureCode(), "fault_requires_reset");
    EXPECT_EQ(task2.state(), TransportState::Fault);

    task2.reset();
    EXPECT_EQ(task2.state(), TransportState::Idle);
    EXPECT_STREQ(task2.failureCode(), "");
    EXPECT_FALSE(task2.active());
}

}  // namespace
