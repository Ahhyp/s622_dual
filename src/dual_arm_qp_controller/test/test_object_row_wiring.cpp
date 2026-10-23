// C4.3 S4 W1 -- S4-T3 / S4-T4 / S4-T5.
//
// What this test is FOR (and what it is not):
//   * It proves the CONTROLLER's wiring of the object task is correct: the row
//     it hands to the QP equals the row assembled from the single definition
//     (dual_arm_object_task) with the frozen sign contract, the relative target
//     is the frozen grasp chain G_L^-1 G_R (not re-measured), and the
//     trajectory clock advances only on committed cycles.
//   * It does NOT re-derive the object-task math and does NOT run Gazebo.
//   * Cross-package bitwise identity of the row itself is already established
//     by S3-T5a (mock cycle() vs a hand-built row); this test chains onto it.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;
using dual_arm_qp::DualArmQpResult;
using dual_arm_qp::ObjectTaskRow;
using dual_arm_qp_controller::CycleOutcome;
using dual_arm_qp_controller::TransportState;
using dual_arm_qp_controller::TransportTask;
using dual_arm_qp_controller::TransportTaskConfig;

constexpr double kPi = 3.14159265358979323846;
/// Frozen by the C4.3 S3 close-out decision (pi - 0.05).
constexpr double kRefuseRad = 3.0915926535897933;

/// S2 frozen baseline, transport_baseline.yaml:66 (q_grasp12).
Q12 graspQ() {
    Q12 q;
    q << 2.795036909814, -1.706950857914, 1.279963524342, -1.143808993223,
        -1.570796326795, -0.346555743776, 0.346555743776, -1.434641795676,
        -1.279963524342, -1.997783660367, 1.570796326795, -2.795036909814;
    return q;
}

/// S2 frozen baseline, transport_baseline.yaml:72/73.
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

/// The parameters S3 used for its own baselines (mock options(): w_center 1e-4).
DualArmQpParams s3Params() {
    DualArmQpParams p;
    p.dt = 0.008;
    p.kp_rel = 10.0;
    p.w_rel = 1.0;
    p.w_slack = 1e3;
    p.w_center = 1e-4;
    p.w_reg = 1e-6;
    p.kp_obj = 20.0;
    p.w_obj = 1.0;
    p.w_slack_obj = 1e3;
    return p;
}

dual_arm_object_task::TrajectorySpec shortSpec(const Iso3& start) {
    dual_arm_object_task::TrajectorySpec s;
    s.dt = 0.008;
    s.time_scaling = "smoothstep";
    s.start_world_T_object = start;
    s.start_from_baseline = false;
    s.phases.push_back({"lift_short", 0.5, 0.0, 0.0, 0.020});
    s.yaw_sweep_deg = {0.0};
    return s;
}

class ObjectRowWiring : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_.object_T_left_tcp = objectTLeftTcp();
        cfg_.object_T_right_tcp = objectTRightTcp();
        cfg_.obj_rot_error_refuse_rad = kRefuseRad;
        cfg_.dt = 0.008;
    }

    dual_arm::DualArmKinematics kin_;
    TransportTaskConfig cfg_;
};

// ---------------------------------------------------------------------------
// S4-T3: row assembly + solve match the single definition, bit for bit.
// ---------------------------------------------------------------------------
TEST_F(ObjectRowWiring, T3_RowAndSolveMatchTheSingleDefinition) {
    const Q12 q0 = graspQ();
    TransportTask task(kin_, cfg_);
    const Iso3 start = task.builder().leftEstimate(q0);

    std::string err;
    ASSERT_TRUE(task.start(shortSpec(start), q0, &err)) << err;
    ASSERT_EQ(task.state(), TransportState::Running);

    // The relative target must be the FROZEN grasp chain G_L^-1 G_R, not the
    // current measured chain (which differs by whatever the arms carry).
    const Iso3 ltr_grasp = task.builder().leftTrightGrasp();
    EXPECT_EQ((task.leftTrightTarget().matrix() - ltr_grasp.matrix()).cwiseAbs().maxCoeff(),
              0.0);
    // At the frozen grasp configuration the measured chain coincides with the
    // frozen one -- that IS the invariant the S2 baseline encodes.  The point
    // of using G_L^-1 G_R is that it stays correct once the arms move away.
    EXPECT_LT((task.leftTrightTarget().matrix() - kin_.leftTright(q0).matrix()).cwiseAbs().maxCoeff(),
              1e-9);

    // Independent reference: same builder, same QP, but driven by its own clock.
    dual_arm_object_task::AnchoredObjectTaskBuilder ref_builder(kin_, cfg_.object_T_left_tcp,
                                                                cfg_.object_T_right_tcp);
    const dual_arm_object_task::FrozenObjectTrajectory ref_traj(shortSpec(start));

    DualArmQp qp(kin_, s3Params());
    DualArmQp qp_ref(kin_, s3Params());
    qp.setWarmStart(false);
    qp_ref.setWarmStart(false);

    Q12 q = q0;
    Q12 q_ref = q0;
    double t_ref = 0.0;
    int cycles = 0;
    int committed = 0;
    double worst_j = 0.0, worst_e = 0.0, worst_dq = 0.0;

    for (int k = 0; k < 40; ++k) {
        // Sample BEFORE step(): step() advances t_traj on commit, so a sample
        // taken afterwards would describe the NEXT cycle.
        const auto smp = task.sample(q);
        const CycleOutcome out = task.step(q, qp);
        ASSERT_EQ(out.failure_code[0], '\0') << "unexpected failure at cycle " << k << ": "
                                            << out.failure_code;
        ASSERT_TRUE(out.converged) << "QP did not converge at cycle " << k;
        ASSERT_TRUE(out.committed) << "cycle " << k << " did not commit";

        // ---- reference path, assembled from scratch ----
        Vec6 ff_ref = Vec6::Zero();
        const Iso3 T_d_ref = ref_traj.at(t_ref, &ff_ref);
        const auto ref_task = ref_builder.build(q_ref, T_d_ref, ff_ref);
        ObjectTaskRow row_ref;
        // SIGN CONTRACT: J = J_O, e = -e_O, ff = +v_d.
        row_ref.J = ref_task.J;
        row_ref.e = -ref_task.e;
        row_ref.body_twist_ff = ref_task.v_ff;
        row_ref.valid = ref_task.inputs_ok;
        const DualArmQpResult ref = qp_ref.solve(q_ref, ltr_grasp, row_ref);

        // ---- compare ----
        const auto row_got = smp.row;
        worst_j = std::max(worst_j, (row_got.J - row_ref.J).cwiseAbs().maxCoeff());
        worst_e = std::max(worst_e, (row_got.e - row_ref.e).cwiseAbs().maxCoeff());
        worst_dq = std::max(worst_dq, (out.dq - ref.dq).cwiseAbs().maxCoeff());
        EXPECT_EQ(out.converged, ref.converged) << "cycle " << k;

        q = q + out.dq;
        q_ref = q_ref + ref.dq;
        if (ref.converged) t_ref = std::min(t_ref + cfg_.dt, ref_traj.duration());
        ++cycles;
        ++committed;
    }

    std::printf("T3: cycles=%d committed=%d max|dJ|=%.3e max|de|=%.3e max|ddq|=%.3e t_traj=%.4f\n",
                cycles, committed, worst_j, worst_e, worst_dq, task.tTraj());
    // Bit-identical: the plan forbids loosening this (1 ULP must be explained,
    // not tolerated).
    EXPECT_EQ(worst_j, 0.0) << "row.J differs from the single definition";
    EXPECT_EQ(worst_e, 0.0) << "row.e differs from -e_O";
    EXPECT_EQ(worst_dq, 0.0) << "dq differs from the reference solve";
    EXPECT_EQ(committed, cycles);
}

// ---------------------------------------------------------------------------
// S4-T4: feed-forward sign and linearity (with the plan's stated preconditions:
// no saturation, fixed target, time-scaling not frozen).
// ---------------------------------------------------------------------------
TEST_F(ObjectRowWiring, T4_FeedForwardEntersTheRowWithTheFrozenSign) {
    const Q12 q = graspQ();
    dual_arm_object_task::AnchoredObjectTaskBuilder builder(kin_, cfg_.object_T_left_tcp,
                                                            cfg_.object_T_right_tcp);
    const Iso3 T_d = builder.leftEstimate(q);  // fixed target, e_O == 0 direction
    const Iso3 ltr = builder.leftTrightGrasp();

    const Vec6 ff_unit = (Vec6() << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

    auto solve_with = [&](const Vec6& ff) {
        DualArmQpParams p = s3Params();
        DualArmQp qp(kin_, p);
        qp.setWarmStart(false);
        const auto task = builder.build(q, T_d, ff);
        ObjectTaskRow row;
        row.J = task.J;
        row.e = -task.e;
        row.body_twist_ff = task.v_ff;
        row.valid = task.inputs_ok;
        const DualArmQpResult r = qp.solve(q, ltr, row);
        EXPECT_TRUE(r.converged);
        EXPECT_FALSE(r.object_task_external == false) << "the external row must be consumed";
        return r;
    };

    const DualArmQpResult zero = solve_with(Vec6::Zero());
    const DualArmQpResult pos = solve_with(ff_unit);
    const DualArmQpResult neg = solve_with(-ff_unit);
    const DualArmQpResult dbl = solve_with(2.0 * ff_unit);

    // The feed-forward must actually move the solution (otherwise the wiring is
    // inert and the test would pass vacuously).
    const double moved = (pos.dq - zero.dq).norm();
    std::printf("T4: |dq(ff)-dq(0)|=%.6e |dq(-ff)+dq(ff)-2dq(0)|=%.6e "
                "|dq(2ff)-dq(0)-2(dq(ff)-dq(0))|=%.6e\n",
                moved, ((pos.dq + neg.dq) - 2.0 * zero.dq).norm(),
                ((dbl.dq - zero.dq) - 2.0 * (pos.dq - zero.dq)).norm());
    EXPECT_GT(moved, 1e-9) << "feed-forward had no effect on dq";

    // Anti-symmetry and doubling (affine in the bias, no active box => exact to
    // solver tolerance; the *structural* statement is sign/scale, not bitwise).
    EXPECT_LT(((pos.dq + neg.dq) - 2.0 * zero.dq).norm(), 1e-6);
    EXPECT_LT(((dbl.dq - zero.dq) - 2.0 * (pos.dq - zero.dq)).norm(), 1e-6);
    // ... and the direction actually flips.
    EXPECT_LT((pos.dq - zero.dq).dot(neg.dq - zero.dq), 0.0);
}

// ---------------------------------------------------------------------------
// S4-T5: the pi - 0.05 refusal boundary, and that a refusal never reaches the
// solver (iterations == 0) and never commits.
// ---------------------------------------------------------------------------
TEST_F(ObjectRowWiring, T5_NearPiIsRefusedBeforeTheSolver) {
    // The frozen constant must be exactly pi - 0.05.
    EXPECT_DOUBLE_EQ(cfg_.obj_rot_error_refuse_rad, kPi - 0.05);
    EXPECT_DOUBLE_EQ(kRefuseRad, kPi - 0.05);

    const Q12 q = graspQ();
    TransportTask task(kin_, cfg_);
    const Iso3 T_hat = task.builder().leftEstimate(q);

    // FrozenObjectTrajectory derives the object rotation purely from
    // yaw_sweep_deg (trajectory.cpp:219 `T_d.linear() = rotZ(yaw)`); the start
    // pose contributes its translation only.  So the attitude error is injected
    // as a yaw of (pi - delta), which yields e_rot = pi - delta about z here
    // because the anchored estimate at the grasp pose has identity rotation.
    auto with_error = [&](double delta) {
        dual_arm_object_task::TrajectorySpec s;
        // Keep the translation of the measured estimate, rotate elsewhere.
        s = shortSpec(T_hat);
        s.phases = {{"hold", 0.5, 0.0, 0.0, 0.0}};
        s.yaw_sweep_deg = {(kPi - delta) * 180.0 / kPi};
        return s;
    };

    // delta 0.04 => e_rot = pi-0.04 > pi-0.05 => refuse.
    // delta 0.06 => e_rot = pi-0.06 < pi-0.05 => allow.
    struct Case {
        double delta;
        bool refuse;
    };
    const Case cases[] = {{0.04, true}, {0.049999, true}, {0.050001, false}, {0.06, false}};

    for (const Case& c : cases) {
        dual_arm_object_task::TrajectorySpec s = with_error(c.delta);
        std::string err;
        TransportTask t(kin_, cfg_);
        ASSERT_TRUE(t.start(s, q, &err)) << err;

        const auto sample = t.sample(q);
        const bool refused = sample.refusal_code[0] != '\0';
        std::printf("T5: delta=%.6f e_rot=%.17g refused=%d code=%s\n", c.delta,
                    sample.e_obj_rot_rad, static_cast<int>(refused),
                    refused ? sample.refusal_code : "-");
        EXPECT_EQ(refused, c.refuse) << "delta=" << c.delta;
        if (c.refuse) {
            EXPECT_STREQ(sample.refusal_code, "object_error_near_pi");
        }

        DualArmQp qp(kin_, s3Params());
        qp.setWarmStart(false);
        const CycleOutcome out = t.step(q, qp);
        if (c.refuse) {
            EXPECT_FALSE(out.committed);
            EXPECT_EQ(out.dq.cwiseAbs().maxCoeff(), 0.0);
            // Proof the solver was never called.
            EXPECT_EQ(out.iterations, 0);
            EXPECT_FALSE(out.converged);
            EXPECT_STREQ(out.failure_code, "object_error_near_pi");
            EXPECT_EQ(t.state(), TransportState::Hold);
        } else {
            // Outside the refusal band the solver must actually run; whether it
            // commits is a feasibility question, not a boundary question.
            EXPECT_GT(out.iterations, 0);
            EXPECT_STRNE(out.failure_code, "object_error_near_pi");
        }
    }
}

}  // namespace
