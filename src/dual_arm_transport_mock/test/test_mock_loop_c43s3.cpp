// C4.3 S3 -- W6 tests: the two-phase closed loop (S3-T6 / T7 / T19 / T20).
//
// What these tests are actually defending:
//   * T6  -- the loop transports the object AND the frozen relative grasp holds,
//            i.e. the coordination frame is real and not a per-cycle re-derivation.
//   * T7  -- the feed-forward is doing work: removing it must make the peak object
//            tracking error strictly worse.  A "test" that passes with ff == 0
//            would prove nothing, so the comparison IS the assertion.
//   * T19 -- a candidate that was COMPUTED but REJECTED must never be counted as
//            executed.  This is the whole point of the two-phase commit and it is
//            the one failure mode that silently corrupts an experiment log.
//   * T20 -- all three object-pose sources are evaluated and reported, and a
//            failure names the source that failed.
//
// TIMING NOTE.  The frozen experiment trajectory is 5.1 s = 638 cycles; one cycle
// costs ~50 collision queries, so a full run is minutes.  These are unit tests, so
// they use SHORTENED specs (same dt, same smoothstep, same three-source checking)
// with a comparable per-step object speed.  The full-length scenarios are W8's job.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport_mock/log_writer.hpp"
#include "dual_arm_transport_mock/mock_loop.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_transport_mock/transport_checker.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_pose_optimization::DualArmScene;
using dual_arm_pose_optimization::SceneLoadOptions;
using dual_arm_transport::GraspConfig;
using dual_arm_transport::TransportBaseline;
using dual_arm_transport::loadGraspConfig;
using dual_arm_transport_mock::CycleRecord;
using dual_arm_transport_mock::MockLoopOptions;
using dual_arm_transport_mock::MockTransportLoop;
using dual_arm_transport_mock::RunSummary;
using dual_arm_object_task::TrajectoryPhase;
using dual_arm_object_task::TrajectorySpec;
using dual_arm_object_task::loadTrajectorySpec;

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot read " + path);
    std::ostringstream os;
    os << f.rdbuf();
    return os.str();
}

class MockLoopC43S3 : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        SceneLoadOptions so;
        so.urdf_path = C43_URDF;
        so.srdf_path = C43_SRDF;
        so.require_meshes = true;
        scene = DualArmScene::load(so);
        cfg = loadGraspConfig(std::string(C43_S2_CONFIG_DIR) + "/object_and_grasps.yaml");
        baseline = TransportBaseline::fromYaml(
            readFile(std::string(C43_S2_CONFIG_DIR) + "/transport_baseline.yaml"));
        spec = loadTrajectorySpec(std::string(C43_CONFIG_DIR) + "/s3_transport_trajectory.yaml");
        if (spec.start_from_baseline) spec.start_world_T_object = baseline.world_T_object;
    }

    void SetUp() override {
        ASSERT_TRUE(scene != nullptr);
        // A throwing SetUpTestSuite does not fail the tests, it silently leaves the
        // statics empty -- so assert the config actually loaded.
        ASSERT_GT(cfg.grip_thickness_m, 0.0) << "grasp config not loaded (C43_S2_CONFIG_DIR)";
        ASSERT_FALSE(baseline.q_grasp.isZero()) << "baseline not loaded";
        ASSERT_FALSE(spec.phases.empty()) << "trajectory spec not loaded";
    }

    static MockLoopOptions options() {
        MockLoopOptions opt;
        opt.qp.dt = 0.008;
        opt.qp.kp_rel = 10.0;
        opt.qp.w_rel = 1.0;
        opt.qp.w_slack = 1e3;
        // w_center is a STEP-SIZE regulariser, not a task weight, but it trades
        // against task accuracy at first order: measured on this very spec,
        //   w_center 1e-2 -> max|e_rel| 1.67e-3, terminal 1.99e-3 (FAILS 1 mm)
        //   w_center 1e-3 -> 2.06e-4 / 6.2e-5
        //   w_center 1e-4 -> 1.67e-5 / 9.4e-5   <-- chosen
        //   w_center 0    -> 1.11e-5 / 1.06e-4
        // while max|dq| moves by 0.1% (4.574e-3 -> 4.569e-3), i.e. the residual
        // is bought with essentially no authority.  1e-4 keeps the tracking error
        // two orders below the 1 mm promise; the C2 default 1e-2 cannot.
        opt.qp.w_center = 1e-4;
        opt.qp.w_reg = 1e-6;
        // The object task is ENABLED here explicitly: the production controller
        // never wires kp_obj / w_obj / w_slack_obj (W0 finding), so the mock must
        // set them itself or it would silently test the legacy path.
        opt.qp.kp_obj = 20.0;
        opt.qp.w_obj = 1.0;
        opt.qp.w_slack_obj = 1e3;
        opt.qp.enable_singularity = false;
        opt.check.min_clearance_m = 0.002;
        // 4 rather than the experiment's 16: the endpoint check on the candidate
        // is what refuses a bad step here, and T9c already proved that 16 samples
        // detect a mid-sweep clip.  Keeping 16 would make these tests take minutes.
        opt.check.sweep_samples = 4;
        return opt;
    }

    /// A short transport with the same shape and per-step object speed as the real
    /// one: +2 cm up, -2 cm across, in 0.5 s.
    static TrajectorySpec fastSpec() {
        TrajectorySpec s = spec;
        s.start_from_baseline = false;
        s.start_world_T_object = baseline.world_T_object;
        s.phases.clear();
        s.phases.push_back(TrajectoryPhase{"lift_fast", 0.5, -0.020, 0.0, 0.020});
        s.yaw_sweep_deg.assign(1, 0.0);
        return s;
    }

    /// Drive the sample DOWN: guaranteed to reduce the robot-world clearance, which
    /// is what T19 needs in order to separate "the state I am in" from "the state
    /// my candidate would put me in".
    static TrajectorySpec descendSpec() {
        Vec6 d = Vec6::Zero();
        d[2] = -0.050;
        TrajectorySpec s = spec;
        s.start_from_baseline = false;
        s.start_world_T_object = baseline.world_T_object * dual_arm::expSE3(d);
        s.phases.clear();
        s.phases.push_back(TrajectoryPhase{"descend", 0.5, 0.0, 0.0, 0.0});
        s.yaw_sweep_deg.assign(1, 0.0);
        return s;
    }

    static std::shared_ptr<DualArmScene> scene;
    static GraspConfig cfg;
    static TransportBaseline baseline;
    static TrajectorySpec spec;
};

std::shared_ptr<DualArmScene> MockLoopC43S3::scene = nullptr;
GraspConfig MockLoopC43S3::cfg;
TransportBaseline MockLoopC43S3::baseline;
TrajectorySpec MockLoopC43S3::spec;

/// A compact, greppable window on the run.  Printed on failure only: a passing
/// test should not fill the log with numbers nobody reads.
void dumpCycles(const std::vector<CycleRecord>& log, int k0, int k1) {
    std::printf("%4s %1s %-18s %12s %12s %12s %12s %9s %9s %3s %2s %12s\n", "k", "c", "code",
                "e_pos", "e_rot", "e_rel", "e_obj", "s_rel", "s_obj", "rk", "al", "|dq|inf");
    for (const CycleRecord& r : log) {
        if (r.k < k0 || r.k > k1) continue;
        std::printf("%4d %1d %-18s %12.4e %12.4e %12.4e %12.4e %9.2e %9.2e %3d %2d %12.4e\n", r.k,
                    r.committed ? 1 : 0, r.failure_code.c_str(), r.e_pos_m, r.e_rot_rad,
                    std::hypot(r.e_rel_pos_m, r.e_rel_rot_rad),
                    std::hypot(r.e_obj_pos_m, r.e_obj_rot_rad),
                    std::hypot(r.slack_rel_pos_m, r.slack_rel_rot_rad),
                    std::hypot(r.slack_obj_pos_m, r.slack_obj_rot_rad), r.rank_stack,
                    r.active_limit_count, r.dq.cwiseAbs().maxCoeff());
    }
}

// --------------------------------------------------------------------------- T5a
// The loop wraps the core QP; the wrapper must not add, drop or reorder anything.
// A one-line difference here would invalidate every number W8 reports.
TEST_F(MockLoopC43S3, T5a_ThinAdapterProducesSameDqAsTheDirectCoreCall) {
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), options());

    const Q12 q = baseline.q_grasp;
    const Iso3 T_d =
        baseline.world_T_object * dual_arm::expSE3(Vec6(0.012, -0.008, 0.020, 0.0, 0.0, 0.04));
    const Vec6 v = Vec6(0.0, 0.0, 0.02, 0.0, 0.0, 0.01);

    const dual_arm_qp::DualArmQpResult via_loop = loop.cycle(q, T_d, v);

    // The same computation, spelled out by hand: build the row, call the core overload.
    MockLoopOptions opt = options();
    dual_arm::DualArmKinematics kin;
    dual_arm_object_task::AnchoredObjectTaskBuilder builder(kin, baseline.object_T_left_tcp,
                                                             baseline.object_T_right_tcp);
    const dual_arm_object_task::AnchoredObjectTask task = builder.build(q, T_d, v);
    dual_arm_qp::ObjectTaskRow row;
    row.J = task.J;
    row.e = -task.e;  // 2.3.1(c)
    row.body_twist_ff = v;
    row.valid = task.inputs_ok;
    dual_arm_qp::DualArmQp core(kin, opt.qp, dual_arm_qp::makeDefaultSolver());
    const dual_arm_qp::DualArmQpResult direct = core.solve(q, loop.leftTrightTarget(), row);

    ASSERT_TRUE(via_loop.converged);
    ASSERT_TRUE(direct.converged);
    const double diff = (via_loop.dq - direct.dq).cwiseAbs().maxCoeff();
    std::printf("\n[ S3-T5a ] loop vs direct core: max|ddq| = %.3e\n", diff);
    EXPECT_LE(diff, 1e-12);
    EXPECT_LE((via_loop.slack - direct.slack).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LE((via_loop.slack_object - direct.slack_object).cwiseAbs().maxCoeff(), 1e-12);
}

// --------------------------------------------------------------------------- T6
TEST_F(MockLoopC43S3, T6_MockLoopTransportsTheObjectWithinTolerance) {
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), options());
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    if (!sum.success) dumpCycles(log, std::max(0, sum.failure_cycle - 6), sum.failure_cycle + 1);
    EXPECT_TRUE(sum.success) << sum.to_string();
    EXPECT_EQ(sum.failure_code, "ok") << sum.to_string();
    EXPECT_GT(sum.committed_cycles, 0);
    EXPECT_EQ(sum.collision_cycles, 0);
    EXPECT_EQ(sum.swept_failures, 0);

    EXPECT_LE(sum.terminal_pos_err_m, 1e-3) << sum.to_string();
    EXPECT_LE(sum.terminal_rot_err_rad, 1e-3) << sum.to_string();
    EXPECT_LE(sum.max_s_rel_norm, 1e-3) << sum.to_string();

    // The frozen relative grasp is what makes this a transport and not two arms
    // each chasing the sample: it must stay at the grasp value for the whole run.
    ASSERT_FALSE(log.empty());
    EXPECT_LE(sum.max_e_rel_norm, 1e-3) << sum.to_string();

    // Every committed cycle must carry the candidate it executed, and the next
    // cycle must start exactly there -- the other half of the two-phase contract
    // (T19 covers the refused half).
    for (std::size_t i = 0; i < log.size(); ++i) {
        const CycleRecord& r = log[i];
        if (!r.committed) continue;
        EXPECT_LT((r.q_cand - (r.q_prev + r.dq)).cwiseAbs().maxCoeff(), 1e-15) << "cycle " << r.k;
        if (i + 1 < log.size()) {
            EXPECT_EQ((log[i + 1].q_prev - r.q_cand).cwiseAbs().maxCoeff(), 0.0)
                << "cycle " << r.k << ": q did not land on the committed candidate";
        }
    }

    // And the two independent estimates of the sample must agree -- if they do
    // not, the anchored form is holding an inconsistent pair of poses.
    for (const CycleRecord& r : log) {
        EXPECT_LE(r.anchor_pos_gap_m, 1e-3) << "cycle " << r.k;
        EXPECT_LE(r.anchor_rot_gap_rad, 1e-3) << "cycle " << r.k;
    }
    std::printf("[ S3-T6 ] %s\n", sum.to_string().c_str());
}

// --------------------------------------------------------------------------- T7
TEST_F(MockLoopC43S3, T7_FeedforwardRemovesTheTrackingLag) {
    // The terminal tolerance is deliberately loosened for BOTH runs: the claim here
    // is the comparison, and the no-ff run is *expected* to miss the 1 mm promise --
    // that is the effect being measured.  Tightening it would make the no-ff run
    // abort before the end and compare two different trajectories.
    MockLoopOptions with_ff = options();
    with_ff.use_object_feedforward = true;
    with_ff.terminal_pos_tol_m = 5e-2;
    with_ff.terminal_rot_tol_rad = 5e-2;
    MockTransportLoop loop_ff(scene, cfg, baseline, fastSpec(), with_ff);
    const RunSummary sum_ff = loop_ff.run();

    MockLoopOptions no_ff = options();
    no_ff.use_object_feedforward = false;
    no_ff.terminal_pos_tol_m = 5e-2;
    no_ff.terminal_rot_tol_rad = 5e-2;
    MockTransportLoop loop_noff(scene, cfg, baseline, fastSpec(), no_ff);
    const RunSummary sum_noff = loop_noff.run();

    ASSERT_TRUE(sum_ff.success) << sum_ff.to_string();
    ASSERT_TRUE(sum_noff.success) << sum_noff.to_string();
    // The same run, judged by the promise it is supposed to keep: ff makes it,
    // no-ff does not.  This is the second, independent witness of the same effect.
    EXPECT_LE(sum_ff.terminal_pos_err_m, 1e-3) << sum_ff.to_string();
    EXPECT_GT(sum_noff.terminal_pos_err_m, 1e-3) << sum_noff.to_string();
    ASSERT_GT(sum_ff.max_e_obj_norm, 0.0) << "degenerate: nothing to track";
    ASSERT_GT(sum_noff.max_e_obj_norm, 0.0) << "degenerate: nothing to track";
    // Strictly better, not "within tolerance": with the analytic v_d the lag is
    // first order in dt, without it the loop is a pure P tracker whose error is
    // proportional to the object speed.
    EXPECT_LT(sum_ff.max_e_obj_norm, sum_noff.max_e_obj_norm)
        << "ff: " << sum_ff.max_e_obj_norm << " no-ff: " << sum_noff.max_e_obj_norm;
    const double ratio = sum_ff.max_e_obj_norm / sum_noff.max_e_obj_norm;
    EXPECT_LT(ratio, 1.0);
    std::printf("[ S3-T7 ] peak |e_O| ff=%.6e no_ff=%.6e ratio=%.4f\n",
                sum_ff.max_e_obj_norm, sum_noff.max_e_obj_norm, ratio);
}

// -------------------------------------------------------------------------- T22
// What the fixed-pose sweep can and cannot say, measured rather than asserted.
//
// checkSweep() interpolates the arm configurations but tests the sample at ONE
// painted world pose.  For a CARRIED object the sample's pose is a function of the
// configuration, so a segment has a different world at every sample.  Three
// measurements pin that down:
//   1. IDENTITY  the tracked sweep's minima equal a manual per-sample recomputation
//                (so it really re-paints the world at every sample);
//   2. BLINDNESS the fixed sweep's minima come from ONE painting: changing only the
//                painting changes its answer, with the arms and the segment fixed;
//   3. VERDICT   a table over candidate segments, printed, so that "does a
//                fixed-passes/tracked-fails pair exist in this scene" is answered
//                by data instead of by hope.
// Step 3 is deliberately NOT asserted as a flip.  A rigidly grasped sample still
// drags the FINGERS along, so the fixed sweep does see most of the motion, and the
// remaining gap is the sample's own pose -- which the world layer only measures
// against ROBOT links (the sample's resting contact with the support is a contact,
// not a collision).  That limitation is recorded in C4.3_S3_验收记录.md instead of
// being hidden behind a test that happens to pass.
TEST_F(MockLoopC43S3, T22_TheFixedPoseSweepCannotSeeTheObjectBeingCarried) {
    using dual_arm::Arm;
    using dual_arm_transport_mock::CheckOptions;
    using dual_arm_transport_mock::TransportChecker;

    const dual_arm::DualArmKinematics kin;
    const Iso3 G_L = baseline.object_T_left_tcp;
    auto objectAt = [&](const Q12& q) {
        dual_arm::JointConfig ql;
        for (int j = 0; j < 6; ++j) ql[j] = q[j];
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, ql), G_L);
    };

    CheckOptions co;
    co.sweep_samples = 8;
    co.object_source = "left_anchor";
    TransportChecker checker(scene, cfg, co);
    const Q12 q0 = baseline.q_grasp;

    struct Row {
        int joint;
        double delta;
        double travel_m;
        bool fixed_ok;
        bool tracked_ok;
        bool end_ok;
        double fixed_world;
        double tracked_world;
        double end_world;
    };
    std::vector<Row> rows;
    double worst_paint_gap = 0.0;
    int worst_row = -1;

    // Small deltas on purpose: a large one simply self-collides the two arms and then
    // every check agrees on "collision", which would prove nothing about the model.
    for (int joint : {0, 1, 3, 5}) {
        for (double delta : {0.04, -0.04}) {
            Q12 q1 = q0;
            q1[joint] += delta;
            const double travel =
                (objectAt(q1).translation() - objectAt(q0).translation()).norm();

            checker.applyWorld(objectAt(q0), "left_anchor");
            const auto fixed = checker.checkSweep(q0, q1);

            checker.applyWorld(objectAt(q1), "left_anchor");  // same arms, other world
            const auto at_end = checker.checkSweep(q0, q1);

            checker.applyWorld(objectAt(q0), "left_anchor");
            const auto tracked = checker.checkSweepTracking(q0, q1, G_L, Arm::Left);

            rows.push_back({joint, delta, travel, fixed.ok, tracked.ok, at_end.ok,
                            fixed.min_robot_world_m, tracked.min_robot_world_m,
                            at_end.min_robot_world_m});
            const double gap = std::fabs(fixed.min_robot_world_m - tracked.min_robot_world_m);
            if (gap > worst_paint_gap) {
                worst_paint_gap = gap;
                worst_row = static_cast<int>(rows.size()) - 1;
            }
        }
    }

    std::printf("\n[ S3-T22 ] fixed(=start painting) vs tracked(carried) vs at_end(same segment, "
                "end painting)\n");
    for (const Row& r : rows) {
        std::printf(
            "  j=%d d=%+.2f travel=%.4f m | fixed(ok=%d world=%.6f) tracked(ok=%d world=%.6f) "
            "at_end(ok=%d world=%.6f)\n",
            r.joint, r.delta, r.travel_m, r.fixed_ok ? 1 : 0, r.fixed_world, r.tracked_ok ? 1 : 0,
            r.tracked_world, r.end_ok ? 1 : 0, r.end_world);
    }

    ASSERT_GT(rows.size(), 0u);
    ASSERT_GE(worst_row, 0) << "no candidate moved the painted world at all";

    // ---- 1. IDENTITY: the tracked sweep is per-sample re-painting, nothing else ----
    {
        const Row& r = rows[static_cast<std::size_t>(worst_row)];
        Q12 q1 = q0;
        q1[r.joint] += r.delta;
        double manual_world = std::numeric_limits<double>::max();
        double manual_clear = std::numeric_limits<double>::max();
        for (int i = 0; i < co.sweep_samples; ++i) {
            const double a = static_cast<double>(i) / static_cast<double>(co.sweep_samples - 1);
            const Q12 q = q0 + a * (q1 - q0);
            checker.applyWorld(objectAt(q), "left_anchor@manual");
            const auto sc = checker.checkState(q);
            manual_world = std::min(manual_world, sc.robot_world);
            manual_clear = std::min(manual_clear, sc.min_clearance);
        }
        checker.applyWorld(objectAt(q0), "left_anchor");
        const auto tracked = checker.checkSweepTracking(q0, q1, G_L, Arm::Left);
        EXPECT_NEAR(tracked.min_robot_world_m, manual_world, 1e-12);
        EXPECT_NEAR(tracked.min_clearance_m, manual_clear, 1e-12);
        EXPECT_EQ(tracked.object_source, "left_anchor@tracked");
    }

    // ---- 2. BLINDNESS: the fixed sweep's answer is a property of the painting ----
    EXPECT_GT(worst_paint_gap, 1e-9)
        << "if the fixed sweep's world minimum does not move when only the painting moves, "
           "it is not measuring the transport at all";
    {
        const Row& r = rows[static_cast<std::size_t>(worst_row)];
        EXPECT_GT(std::fabs(r.fixed_world - r.end_world), 1e-9)
            << "same arms, same segment, different object paint -> different world minimum";
    }

    // ---- 3. VERDICT table: report, do not manufacture ----
    std::size_t flips = 0;
    for (const Row& r : rows) {
        if (r.fixed_ok && !r.tracked_ok) ++flips;
    }
    std::printf("[ S3-T22 ] worst paint gap %.6f m (joint %d, delta %+.2f); %zu/%zu segments pass "
                "the fixed sweep yet fail the carried sweep\n",
                worst_paint_gap, rows[static_cast<std::size_t>(worst_row)].joint,
                rows[static_cast<std::size_t>(worst_row)].delta, flips, rows.size());
}

// -------------------------------------------------------------------------- T20
TEST_F(MockLoopC43S3, T20_AllThreeObjectPoseSourcesAreReported) {
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), options());
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);
    ASSERT_TRUE(sum.success) << sum.to_string();
    ASSERT_FALSE(log.empty());

    const auto& sources = MockTransportLoop::objectSources();
    const std::set<std::string> expected(sources.begin(), sources.end());
    std::size_t checked_cycles = 0;
    for (const CycleRecord& r : log) {
        // Aborted cycles return before the sweep, so only fully-checked ones carry
        // the three reports.
        if (r.sweeps.empty()) continue;
        ++checked_cycles;
        // 3 fixed-pose placements + 1 carried-sample (tracked) sweep of the adopted
        // source -- the tracked one is what actually says the transport is safe
        // (S3-T22); the other three say the arms may move at that one world.
        ASSERT_EQ(r.sweeps.size(), 4u) << "cycle " << r.k;
        std::set<std::string> got;
        for (const auto& sc : r.sweeps) {
            got.insert(sc.object_source);
            EXPECT_TRUE(std::isfinite(sc.min_clearance_m)) << sc.object_source;
            EXPECT_LE(sc.first_bad_sample, 3);  // -1 (ok) .. sweep_samples-1
        }
        for (const std::string& src : expected) {
            EXPECT_EQ(got.count(src), 1u) << "cycle " << r.k << " missed source " << src;
        }
        EXPECT_EQ(got.count("left_anchor@tracked"), 1u) << "cycle " << r.k;
        EXPECT_TRUE(r.sweep_tracked_ok) << "cycle " << r.k;
        EXPECT_EQ(r.sweep_tracked_code, "ok") << "cycle " << r.k;
        // The emitted rows carry one entry per source, in the fixed source order,
        // followed by the tracked placement.
        const auto rows = dual_arm_transport_mock::sweepRows(r);
        ASSERT_EQ(rows.size(), 4u);
        for (std::size_t i = 0; i < 3; ++i) EXPECT_EQ(rows[i][1], sources[i]);
        EXPECT_EQ(rows[3][1], "left_anchor@tracked");
    }
    EXPECT_GT(checked_cycles, 10u);
    const std::string header = dual_arm_transport_mock::sweepsCsvHeader();
    EXPECT_NE(header.find("object_source"), std::string::npos);
    EXPECT_NE(header.find("first_bad_sample"), std::string::npos);
    std::printf("[ S3-T20 ] %zu cycles x 3 sources reported\n", checked_cycles);
}

// -------------------------------------------------------------------------- T19
TEST_F(MockLoopC43S3, T19_UncommittedCandidateIsNeverCountedAsExecuted) {
    // Two-phase commit under a refusal that is GUARANTEED to fire every cycle:
    // squeeze the object-slack budget below anything the solver can deliver, so
    // step 9 refuses AFTER the solve and BEFORE the commit.  Every cycle therefore
    // computes a real motion and must execute none of it.
    //
    // Why not the geometric route (drive the sample down into the table)?  Measured,
    // not assumed: min_clearance = min(whole_robot, robot_world) is PINNED at
    // 0.0131174 m by the constant left_wrist3 <-> wrist_camera self pair, and the
    // carried sample keeps a constant finger <-> sample distance, so no descent makes
    // the CANDIDATE violate clearance before the CURRENT STATE does -- the test then
    // measures nothing.  The deterministic refusal is the honest way to exercise the
    // invariant, and it is strictly harder: the loop is asked to move every cycle.
    MockLoopOptions opt = options();
    // 1e-12, not 1e-9: the object task must already be *asking* for something.  It
    // is not at t = 0 when the sample sits exactly at the grasp (e_O == 0 -> s_obj
    // ~ 1e-20 -> the first cycles would commit), so the spec starts 2 cm out; the
    // refusal then holds for every cycle with ~400x margin (measured s_obj 4e-10).
    opt.s_obj_max = 1e-12;
    TrajectorySpec spec_gap = fastSpec();
    Vec6 up = Vec6::Zero();
    up[2] = 0.020;
    spec_gap.start_world_T_object = baseline.world_T_object * dual_arm::expSE3(up);
    MockTransportLoop loop(scene, cfg, baseline, spec_gap, opt);
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    ASSERT_FALSE(log.empty());
    EXPECT_FALSE(sum.success) << "the refusal streak was expected to abort the run\n"
                              << sum.to_string();
    EXPECT_EQ(sum.failure_code, "object_task_infeasible") << sum.to_string();
    EXPECT_EQ(sum.committed_cycles, 0) << "a refused candidate was committed\n" << sum.to_string();
    EXPECT_GT(sum.vetoed_cycles, 0);

    const Q12 q0 = log.front().q_prev;
    for (std::size_t i = 0; i < log.size(); ++i) {
        const CycleRecord& r = log[i];
        // 1. never counted as executed
        ASSERT_FALSE(r.committed) << "cycle " << r.k;
        ASSERT_EQ(r.failure_code, "object_task_infeasible") << "cycle " << r.k;
        // 2. the QP really ran and really produced motion.  This is what makes the
        //    test non-vacuous: dq == 0 would mean "nothing happened", not "something
        //    was refused".
        ASSERT_GT(r.dq.norm(), 0.0) << "cycle " << r.k << ": there was nothing to refuse";
        // 3. ORDERING, asserted rather than assumed: the slack judge is step 9 and the
        //    candidate is formed at step 10, so a refused cycle never even produces a
        //    candidate -- q_cand stays at q_prev.  That is stronger than "the candidate
        //    was discarded": there is no candidate to leak.  (The step-11 geometric
        //    refusal is the case where q_cand DOES differ; it is covered by T9c at the
        //    checker level, because in this scene min_clearance is pinned by the
        //    constant wrist3 <-> wrist_camera pair and no descent moves it -- measured,
        //    see the note above.)
        ASSERT_EQ((r.q_cand - r.q_prev).cwiseAbs().maxCoeff(), 0.0)
            << "cycle " << r.k << ": step 10 ran even though step 9 refused";
        // 4. q never advanced, not by one bit -- and where it WOULD have gone is still
        //    in the record, which is the whole reason q_cand is logged.
        ASSERT_EQ((r.q_prev - q0).cwiseAbs().maxCoeff(), 0.0) << "cycle " << r.k;
        if (i + 1 < log.size()) {
            const double drift = (log[i + 1].q_prev - r.q_prev).cwiseAbs().maxCoeff();
            ASSERT_EQ(drift, 0.0) << "vetoed cycle " << r.k << " advanced q by " << drift;
        }
    }
    EXPECT_EQ(sum.vetoed_cycles, static_cast<int>(log.size()));
    std::printf("[ S3-T19 ] %zu cycles all solved and all refused; q never advanced; "
                "failure=%s at cycle %d\n",
                log.size(), sum.failure_code.c_str(), sum.failure_cycle);
}

}  // namespace
