// C4.3 S3 -- W6: the failure policy (S3-T13) and the rank precheck (S3-T18).
//
// S3-T13 exists because a transport loop that "keeps going" when its solver fails or
// its object task becomes infeasible produces a log that looks successful while the
// robot is doing nothing.  Every failure class must therefore (a) appear under its
// documented code, (b) take the documented action (hold vs abort), and (c) leave a
// record.  Each case below injects ONE failure and asserts exactly that.
//
// S3-T18 is the numeric precondition the whole design leans on: the stacked system
// [J_rel; J_O] is 12x12, so "no redundancy" and "the object task is satisfiable" hold
// ONLY when it is full rank.  The plan asserts rank every cycle; this test proves the
// assertion is live by making it fail.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport_mock/mock_loop.hpp"
#include "dual_arm_transport_mock/object_task.hpp"
#include "dual_arm_transport_mock/sha256.hpp"
#include "dual_arm_transport_mock/trajectory.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec3;
using dual_arm::Vec6;
using dual_arm_pose_optimization::DualArmScene;
using dual_arm_pose_optimization::SceneLoadOptions;
using dual_arm_transport::GraspConfig;
using dual_arm_transport::TransportBaseline;
using dual_arm_transport::loadGraspConfig;
using dual_arm_transport_mock::AnchoredObjectTaskBuilder;
using dual_arm_transport_mock::CheckOptions;
using dual_arm_transport_mock::CycleRecord;
using dual_arm_transport_mock::MockLoopOptions;
using dual_arm_transport_mock::MockTransportLoop;
using dual_arm_transport_mock::RunSummary;
using dual_arm_transport_mock::StackRank;
using dual_arm_transport_mock::TrajectoryPhase;
using dual_arm_transport_mock::TrajectorySpec;
using dual_arm_transport_mock::loadTrajectorySpec;

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot read " + path);
    std::ostringstream os;
    os << f.rdbuf();
    return os.str();
}

/// A solver that never converges.  Injecting it is the only honest way to exercise
/// the qp_solver_failed path: the real backend converges on every reachable problem,
/// so a "test" that waits for it to fail tests nothing.
class NeverConverges : public dual_arm_qp::QpSolver {
public:
    dual_arm_qp::QpSolution solve(const dual_arm_qp::QpProblem& problem) override {
        dual_arm_qp::QpSolution s;
        s.x = Eigen::VectorXd::Zero(problem.num_vars());
        s.converged = false;
        s.iterations = 0;
        s.solve_time_ms = 0.0;
        return s;
    }
    std::string name() const override { return "never-converges"; }
};

class FailurePolicyC43S3 : public ::testing::Test {
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
        ASSERT_GT(cfg.grip_thickness_m, 0.0) << "grasp config not loaded (C43_S2_CONFIG_DIR)";
        ASSERT_FALSE(baseline.q_grasp.isZero());
        ASSERT_FALSE(spec.phases.empty());
    }

    static MockLoopOptions options() {
        MockLoopOptions opt;
        opt.qp.dt = 0.008;
        opt.qp.kp_rel = 10.0;
        opt.qp.w_rel = 1.0;
        opt.qp.w_slack = 1e3;
        opt.qp.w_center = 1e-4;
        opt.qp.w_reg = 1e-6;
        opt.qp.kp_obj = 20.0;
        opt.qp.w_obj = 1.0;
        opt.qp.w_slack_obj = 1e3;
        opt.qp.enable_singularity = false;
        opt.check.min_clearance_m = 0.002;
        opt.check.sweep_samples = 4;
        return opt;
    }

    static TrajectorySpec fastSpec() {
        TrajectorySpec s = spec;
        s.start_from_baseline = false;
        s.start_world_T_object = baseline.world_T_object;
        s.phases.clear();
        s.phases.push_back(TrajectoryPhase{"lift_fast", 0.5, -0.020, 0.0, 0.020});
        s.yaw_sweep_deg.assign(1, 0.0);
        return s;
    }

    static TrajectorySpec descendSpec(double drop_m, double duration) {
        TrajectorySpec s = spec;
        s.start_from_baseline = false;
        s.start_world_T_object = baseline.world_T_object;
        s.phases.clear();
        s.phases.push_back(TrajectoryPhase{"descend", duration, 0.0, 0.0, -drop_m});
        s.yaw_sweep_deg.assign(1, 0.0);
        return s;
    }

    /// The two-phase invariant, checked on every cycle of every failing run: a cycle
    /// that was not committed must not have moved the robot.
    static void expectNoSilentAdvance(const std::vector<CycleRecord>& log) {
        for (std::size_t i = 0; i + 1 < log.size(); ++i) {
            const CycleRecord& a = log[i];
            const CycleRecord& b = log[i + 1];
            if (a.committed) {
                EXPECT_EQ(b.q_prev, a.q_cand) << "cycle " << a.k << ": committed step was not "
                                                 "the state the next cycle started from";
            } else {
                EXPECT_EQ(b.q_prev, a.q_prev)
                    << "cycle " << a.k << ": a REFUSED candidate moved the robot";
            }
        }
    }

    static std::shared_ptr<DualArmScene> scene;
    static GraspConfig cfg;
    static TransportBaseline baseline;
    static TrajectorySpec spec;
};

std::shared_ptr<DualArmScene> FailurePolicyC43S3::scene = nullptr;
GraspConfig FailurePolicyC43S3::cfg;
TransportBaseline FailurePolicyC43S3::baseline;
TrajectorySpec FailurePolicyC43S3::spec;

}  // namespace

// ------------------------------------------------------------------- S3-T13.1
TEST_F(FailurePolicyC43S3, T13a_SolverFailureIsReportedAndRecorded) {
    MockLoopOptions opt = options();
    opt.solver = std::make_shared<NeverConverges>();

    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    std::printf("\n[ S3-T13.1 ] solver never converges: %s at cycle %d after %zu recorded cycles\n",
                sum.failure_code.c_str(), sum.failure_cycle, log.size());
    EXPECT_FALSE(sum.success);
    EXPECT_EQ(sum.failure_code, "qp_solver_failed");
    // Documented action: a veto (hold), not an immediate abort -- the streak decides.
    EXPECT_EQ(sum.failure_cycle, opt.solver_fail_streak_max);
    EXPECT_EQ(sum.committed_cycles, 0);
    EXPECT_EQ(sum.vetoed_cycles, static_cast<int>(log.size()));
    ASSERT_FALSE(log.empty());
    for (const CycleRecord& r : log) {
        EXPECT_EQ(r.failure_code, "qp_solver_failed") << "cycle " << r.k;
        EXPECT_FALSE(r.committed) << "cycle " << r.k;
    }
    expectNoSilentAdvance(log);
}

// ------------------------------------------------------------------- S3-T13.2
TEST_F(FailurePolicyC43S3, T13b_ObjectInfeasibilityIsReportedAndAborts) {
    MockLoopOptions opt = options();
    opt.s_obj_max = 1e-9;  // every cycle's object slack exceeds this

    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    std::printf("[ S3-T13.2 ] object slack over budget: %s at cycle %d after %zu cycles\n",
                sum.failure_code.c_str(), sum.failure_cycle, log.size());
    EXPECT_FALSE(sum.success);
    EXPECT_EQ(sum.failure_code, "object_task_infeasible");
    // The first cycles CAN commit: at t = 0 the object task is satisfied (e_O = 0,
    // s_obj ~ 1e-20), so it takes a few cycles of motion before the slack exceeds the
    // budget.  What must hold is that the abort is the END of a full unbroken streak of
    // refusals -- one refusal fewer and the run would still be going.
    ASSERT_GE(log.size(), static_cast<std::size_t>(opt.object_infeasible_streak_max));
    const std::size_t streak = static_cast<std::size_t>(opt.object_infeasible_streak_max);
    for (std::size_t i = log.size() - streak; i < log.size(); ++i) {
        EXPECT_EQ(log[i].failure_code, "object_task_infeasible") << "cycle " << log[i].k;
        EXPECT_FALSE(log[i].committed) << "cycle " << log[i].k;
    }
    // Cycles before the streak may well include a stray refusal -- a refusal only
    // RESETS the streak counter, it does not end the run.  What must hold is that the
    // loop transported normally first (the injected budget is not hit from cycle 0).
    EXPECT_GT(sum.committed_cycles, 0);
    EXPECT_EQ(sum.vetoed_cycles + sum.committed_cycles, static_cast<int>(log.size()));
    EXPECT_EQ(sum.failure_cycle, log.back().k);
    expectNoSilentAdvance(log);
}

// ------------------------------------------------------------------- S3-T13.3
TEST_F(FailurePolicyC43S3, T13c_OutOfLimitStateIsReportedAsSuch) {
    MockLoopOptions opt = options();
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);

    // Joint 1's upper limit is 1.4835 rad; 3.0 rad is outside it by 1.5 rad.
    Q12 q = baseline.q_grasp;
    q[1] = 3.0;
    const dual_arm_transport_mock::CycleOutcome out =
        loop.step(q, 0.0, 0, baseline.world_T_object, Vec6::Zero());

    std::printf("[ S3-T13.3 ] out-of-limit state: %s (%s), abort=%d, committed=%d\n",
                out.record.failure_code.c_str(), out.record.state_check.detail.c_str(),
                out.abort ? 1 : 0, out.record.committed ? 1 : 0);
    // The failure must be attributed to the LIMITS.  Without the step-1b guard the
    // out-of-limit state reaches the solver as an infeasible box (lower > upper) and
    // comes back as "qp_solver_failed", i.e. the wrong subsystem gets blamed.
    EXPECT_EQ(out.record.failure_code, "joint_limit_saturation");
    EXPECT_TRUE(out.abort);
    EXPECT_FALSE(out.record.committed);
    EXPECT_LT(out.record.min_limit_margin_rad, 0.0);
}

// ------------------------------------------------------------------- S3-T13.4
TEST_F(FailurePolicyC43S3, T13d_CollisionIsReportedAndAbortsImmediately) {
    MockLoopOptions opt = options();
    // Drive the sample 0.35 m down: the arms have to follow it through the table, so
    // the candidate state eventually leaves the clearance budget.  Both the state
    // check and the swept check are legitimately allowed to be the one that fires.
    // The START pose is the frozen grasp (so the relative task starts satisfied); the
    // commanded pose travels 0.35 m down, which the arms cannot follow through the
    // table.  Moving the START pose down instead would open the grasp immediately and
    // abort on anchor_disagreement at cycle 1, testing nothing about collisions.
    MockTransportLoop loop(scene, cfg, baseline, descendSpec(0.35, 1.0), opt);
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    std::printf("[ S3-T13.4 ] drive into the table: %s at cycle %d (%s), %d committed\n",
                sum.failure_code.c_str(), sum.failure_cycle, sum.failure_detail.c_str(),
                sum.committed_cycles);
    EXPECT_FALSE(sum.success);
    EXPECT_TRUE(sum.failure_code == "insufficient_clearance" || sum.failure_code == "collision" ||
                sum.failure_code == "swept_collision")
        << "unexpected failure code: " << sum.failure_code;
    EXPECT_GT(sum.committed_cycles, 0) << "the run must make progress before it is stopped";
    ASSERT_FALSE(log.empty());
    // The aborting cycle is recorded and NOT committed (it is the refused candidate).
    EXPECT_FALSE(log.back().committed);
    expectNoSilentAdvance(log);
}

// ------------------------------------------------------------------- S3-T18
TEST_F(FailurePolicyC43S3, T18a_StackIsFullRankAtTheFrozenGrasp) {
    const dual_arm::DualArmKinematics kin;
    const AnchoredObjectTaskBuilder builder(kin, baseline.object_T_left_tcp,
                                            baseline.object_T_right_tcp);
    const Q12 q = baseline.q_grasp;
    const dual_arm_transport_mock::AnchoredObjectTask task =
        builder.build(q, baseline.world_T_object, Vec6::Zero());
    const Mat6x12 J_rel = kin.relativeJacobian(q);
    const StackRank rk = dual_arm_transport_mock::computeStackRank(J_rel, task.J);

    std::printf("\n[ S3-T18 ] q_grasp12 stack: rank=%d sigma_max=%.6e sigma_min=%.6e cond=%.6e\n",
                rk.rank, rk.sigma_max, rk.sigma_min, rk.cond);
    EXPECT_EQ(rk.rank, 12) << "the stacked system must be full rank at the frozen grasp, or the "
                              "object task is not independently satisfiable";
    EXPECT_TRUE(rk.full_rank());
    EXPECT_GE(rk.sigma_min, dual_arm_transport_mock::kSigmaMinStackMin);
    EXPECT_GT(rk.cond, 0.0);
    EXPECT_LT(rk.cond, 1e6) << "a condition number this large would make every solve a coin flip";
}

TEST_F(FailurePolicyC43S3, T18b_RankLossIsDetectedNotAbsorbed) {
    // A stack with two identical rows is rank 11.  The precheck must say so rather
    // than hand a singular system to the solver and let it return a confident answer.
    Eigen::Matrix<double, 12, 12> M = Eigen::Matrix<double, 12, 12>::Identity();
    M.row(3) = M.row(2);  // rows 2 and 3 identical => rank 11, not 12
    Mat6x12 J_rel = M.topRows<6>();
    Mat6x12 J_obj = M.bottomRows<6>();

    const StackRank rk = dual_arm_transport_mock::computeStackRank(J_rel, J_obj);
    std::printf("[ S3-T18 ] deliberately rank-deficient stack: rank=%d (of 12), cond=%.3e\n",
                rk.rank, rk.cond);
    EXPECT_LT(rk.rank, 12);
    EXPECT_FALSE(rk.full_rank());
}

TEST_F(FailurePolicyC43S3, T18c_LoopRefusesWhenTheRankPrecheckFails) {
    MockLoopOptions opt = options();
    // Raise the required sigma_min above anything physical: the guard must stop the
    // run instead of solving anyway.
    opt.sigma_min_stack_min = 1e3;

    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);
    std::vector<CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    std::printf("[ S3-T18 ] sing guard tripped: %s at cycle %d (%s)\n", sum.failure_code.c_str(),
                sum.failure_cycle, sum.failure_detail.c_str());
    EXPECT_FALSE(sum.success);
    EXPECT_EQ(sum.failure_code, "task_stack_singular");
    EXPECT_EQ(sum.committed_cycles, 0);
    EXPECT_EQ(sum.failure_cycle, 0) << "the precondition is checked before the first commit";
    ASSERT_EQ(log.size(), 1u);
    EXPECT_FALSE(log[0].committed);
}

// ------------------------------------------------------------------- S3-T15
// "joint limit saturation" is only a useful failure code if the attribution is
// right: WHICH joint, HOW far outside, and HOW MANY joints are on the box.  Here a
// velocity ceiling of 1e-2 rad/s (8e-5 rad/cycle) is far below the 4.5e-3 rad the
// task asks for, so every joint is legitimately pinned to its box and the count must
// be 12 -- not "some joints, probably".
TEST_F(FailurePolicyC43S3, T15_LimitAttributionIsCorrect) {
    MockLoopOptions opt = options();
    opt.qp.v_max = dual_arm_qp::Vec12::Constant(1e-2);
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);

    const Q12 q = baseline.q_grasp;
    // The commanded pose must be DISPLACED: at the grasp itself e_O = 0, the optimal
    // answer is dq = 0, and no box is ever touched (that is what the first version of
    // this test measured -- |dq|_inf = 1.05e-13 against a box of 8e-5).  A 2 mm demand
    // is far below the object slack budget (5e-3) yet far above what an 8e-5 rad budget
    // can deliver, so the box is what limits the step.
    const Iso3 T_d =
        baseline.world_T_object * dual_arm::expSE3(Vec6(0.002, 0.0, 0.002, 0.0, 0.0, 0.01));
    const dual_arm_transport_mock::CycleOutcome out = loop.step(q, 0.0, 0, T_d, Vec6::Zero());

    // Independent recomputation of the margin reported by the loop.  The record's
    // margin is measured at the CANDIDATE (after the step), not at q -- so this test
    // doubles as documentation of that choice.
    double expect_margin = std::numeric_limits<double>::max();
    for (int i = 0; i < 12; ++i) {
        const double lo = out.record.q_cand[i] - opt.qp.q_min[i];
        const double hi = opt.qp.q_max[i] - out.record.q_cand[i];
        expect_margin = std::min(expect_margin, std::min(lo, hi));
    }
    std::printf("\n[ S3-T15 ] all joints on the velocity box: active=%d min_margin=%.6e rad\n",
                out.record.active_limit_count, out.record.min_limit_margin_rad);
    std::printf("           |dq|inf=%.6e (box = %.6e)  converged=%d  iters=%d\n",
                out.record.dq.cwiseAbs().maxCoeff(), opt.qp.v_max[0] * opt.qp.dt,
                out.qp.converged ? 1 : 0, out.qp.iterations);
    for (int i = 0; i < 12; ++i) std::printf("   dq[%2d]=%12.6e  box=%9.3e\n", i, out.record.dq[i],
                                             opt.qp.v_max[i] * opt.qp.dt);
    // The count must equal an independent recount against the SAME box.  Note it is
    // NOT necessarily 12: only the joints whose demand exceeds the budget land on it
    // (7 of 12 here), and a test that insists on 12 would be asserting the shape of
    // the demand, not the bookkeeping.
    int expect_active = 0;
    for (int i = 0; i < 12; ++i) {
        const double box = opt.qp.v_max[i] * opt.qp.dt;
        if (out.record.dq[i] >= box - 1e-12 || out.record.dq[i] <= -box + 1e-12) ++expect_active;
    }
    EXPECT_EQ(out.record.active_limit_count, expect_active);
    EXPECT_GE(expect_active, 1) << "the demand must actually saturate something";
    EXPECT_NEAR(out.record.min_limit_margin_rad, expect_margin, 1e-12);
    // The step is bounded by v_max*dt, i.e. the box is what limited it.
    EXPECT_LE(out.record.dq.cwiseAbs().maxCoeff(), 1e-2 * opt.qp.dt + 1e-12);
    EXPECT_TRUE(out.record.committed);
}

// ------------------------------------------------------------------- S3-T16
// S2's frozen baseline is INPUT to S3.  If it ever changes underneath us, every
// number in the acceptance record silently refers to a different robot/trajectory,
// so the hash is pinned here rather than trusted.
TEST_F(FailurePolicyC43S3, T16_S2BaselineIsUnmodified) {
    const std::string path = std::string(C43_S2_CONFIG_DIR) + "/transport_baseline.yaml";
    const std::string got = dual_arm_transport_mock::sha256FileHex(path);
    std::printf("\n[ S3-T16 ] transport_baseline.yaml sha256 = %s\n", got.c_str());
    EXPECT_EQ(got, "8587f3647f270dee09c65aa127f173b7f0fc45cbaaff5ba06cd2f77c16276615")
        << "the S2 frozen baseline changed; S3's results are no longer comparable";
}

// ------------------------------------------------------------------- S3-T21b
// The other half of S3-T21: when the object-attitude error is inside the band where
// the row stops being the derivative, the loop must REFUSE.  A wrong direction that
// looks like a normal cycle is worse than a stopped run.
TEST_F(FailurePolicyC43S3, T21b_LoopRefusesWhenTheObjectRotationErrorIsNearPi) {
    MockLoopOptions opt = options();
    MockTransportLoop loop(scene, cfg, baseline, fastSpec(), opt);
    const Q12 q = baseline.q_grasp;
    const double pi = 3.14159265358979323846;

    Iso3 T_bad = baseline.world_T_object;
    T_bad.linear() = baseline.world_T_object.linear() * dual_arm::expSO3(Vec3(0.0, 0.0, pi - 0.01));
    const dual_arm_transport_mock::CycleOutcome bad = loop.step(q, 0.0, 0, T_bad, Vec6::Zero());

    Iso3 T_ok = baseline.world_T_object;
    T_ok.linear() = baseline.world_T_object.linear() * dual_arm::expSO3(Vec3(0.0, 0.0, pi - 0.2));
    const dual_arm_transport_mock::CycleOutcome ok = loop.step(q, 0.0, 0, T_ok, Vec6::Zero());

    std::printf("\n[ S3-T21b ] pi-0.01: code=%s abort=%d committed=%d | pi-0.2: code=%s\n",
                bad.record.failure_code.c_str(), bad.abort ? 1 : 0, bad.record.committed ? 1 : 0,
                ok.record.failure_code.c_str());
    EXPECT_EQ(bad.record.failure_code, "object_error_near_pi");
    EXPECT_TRUE(bad.abort);
    EXPECT_FALSE(bad.record.committed);
    EXPECT_TRUE(bad.record.dq.cwiseAbs().maxCoeff() == 0.0)
        << "the guard must fire before the QP, not after it produced something";
    EXPECT_NE(ok.record.failure_code, "object_error_near_pi")
        << "0.2 rad inside pi must still be allowed, or the guard is eating the workspace";
}
