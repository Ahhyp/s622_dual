// C4.3 S4 -- W0 migration dump (acceptance clause W0-2).
//
// Purpose: prove that moving the anchored object task + trajectory units from
// `dual_arm_transport_mock` into `dual_arm_object_task` is numerically inert.
// Same source, same inputs, same compiler flags; only the include path and the
// namespace alias change.  Every number is printed as %a (hex float) so the two
// outputs can be compared with `cmp`, not with a tolerance.
//
// Built twice by tools/w0_migration_check.sh.
#ifdef C43_DUMP_OLD_LAYOUT
// Pre-migration layout (commit 627a761): the units lived in the mock package.
#include "dual_arm_transport_mock/object_task.hpp"
#include "dual_arm_transport_mock/trajectory.hpp"
namespace ns = dual_arm_transport_mock;
#else
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
namespace ns = dual_arm_object_task;
#endif

#include <cmath>
#include <cstdio>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "legacy_cases.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm::Vec3;
using dual_arm::Vec6;

// Frozen S2 grasp transforms, transcribed from
// src/dual_arm_transport/config/transport_baseline.yaml:
//   object_T_left_tcp  = [0 1 0  0.0497; 1 0 0  0; 0 0 -1  0.0250; 0 0 0 1]
//   object_T_right_tcp = [0 1 0 -0.0497; 1 0 0  0; 0 0 -1  0.0250; 0 0 0 1]
// A transcription error would be caught by the self-check below, which asserts
// T_hat_{O,L}(q_grasp) == world_T_object_star (S3-T2).
Iso3 graspTransform(double px, double py, double pz) {
    Iso3 T = Iso3::Identity();
    T.linear() << 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0,
                  0.0, 0.0, -1.0;
    T.translation() = Vec3(px, py, pz);
    return T;
}

void dumpRaw(const char* tag, const double* v, int n) {
    std::printf(" %s=", tag);
    for (int i = 0; i < n; ++i) std::printf(i ? ",%a" : "%a", v[i]);
}

void dumpIso(const char* tag, const Iso3& T) {
    std::printf(" %s=", tag);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) std::printf("%a,", T.linear()(r, c));
        std::printf("%a,", T.translation()(r));
    }
}

Vec6 rot(double x, double y, double z) {
    Vec6 v;
    v << 0.0, 0.0, 0.0, x, y, z;
    return v;
}

}  // namespace

int main() {
    const dual_arm::DualArmKinematics kin;
    const Iso3 G_L = graspTransform(0.0497, 0.0, 0.0250);
    const Iso3 G_R = graspTransform(-0.0497, 0.0, 0.0250);
    const ns::AnchoredObjectTaskBuilder builder(kin, G_L, G_R);

    std::printf("# C4.3 S4 W0 migration dump -- hex floats, compare with cmp\n");

    // ---- self-check: the frozen grasp must reproduce world_T_object_star ----
    const Q12 q_grasp = legacy_golden::graspQ();
    {
        const Iso3 T_hat = builder.leftEstimate(q_grasp);
        const double dp = (T_hat.translation() - Vec3(0.0, 0.0, 0.275)).norm();
        const double dr = dual_arm::logSO3(T_hat.linear().transpose() * Eigen::Matrix3d::Identity()).norm();
        std::printf("# self_check_grasp dp=%a dr=%a\n", dp, dr);
        if (!(dp < 1e-9) || !(dr < 1e-9)) {
            std::printf("# FATAL: G_L transcription wrong (dp=%g dr=%g)\n", dp, dr);
            return 1;
        }
        // Frozen relative grasp G_L^-1 G_R must be a pure translation.
        const Iso3 ltr = builder.leftTrightGrasp();
        std::printf("# left_T_right_grasp");
        dumpIso("T", ltr);
        std::printf("\n");
    }

    // ---- q grid: the frozen grasp plus three off-grasp configurations ----
    Q12 qs[4];
    qs[0] = q_grasp;
    qs[1] = q_grasp;
    for (int i = 0; i < 12; ++i) qs[1][i] += (i % 2 == 0 ? 0.01 : -0.008) * (1.0 + 0.1 * i);
    qs[2] = q_grasp;
    for (int i = 0; i < 12; ++i) qs[2][i] += 0.02 * std::sin(1.7 * i);
    qs[3] = q_grasp;
    for (int i = 0; i < 6; ++i) qs[3][i] -= 0.03;
    for (int i = 6; i < 12; ++i) qs[3][i] += 0.03;

    // ---- desired pose grid ----
    struct DesiredCase {
        const char* name;
        Iso3 T;
    };
    DesiredCase ds[5];
    ds[0].name = "star";
    ds[0].T = Iso3::Identity();
    ds[0].T.translation() = Vec3(0.0, 0.0, 0.275);
    ds[1].name = "rot_y";
    ds[1].T = ds[0].T;
    ds[1].T.linear() = ds[1].T.linear() * dual_arm::expSO3(rot(0.0, 0.3, 0.0).tail<3>());
    ds[2].name = "trans";
    ds[2].T = ds[0].T;
    ds[2].T.translation() += Vec3(0.03, -0.02, 0.012);
    ds[3].name = "near_pi";
    ds[3].T = ds[0].T;
    ds[3].T.linear() = ds[3].T.linear() * dual_arm::expSO3(rot(0.0, 0.0, M_PI - 0.01).tail<3>());
    ds[4].name = "mid_rot";
    ds[4].T = ds[0].T;
    ds[4].T.linear() = ds[4].T.linear() * dual_arm::expSO3(rot(0.1, 0.2, -0.25).tail<3>()).inverse();

    const Vec6 ffs[3] = {Vec6::Zero(),
                         (Vec6() << 0.01, -0.02, 0.0, 0.0, 0.0, 0.05).finished(),
                         (Vec6() << -0.03, 0.004, 0.02, 0.02, -0.01, 0.0).finished()};

    for (int qi = 0; qi < 4; ++qi) {
        for (const DesiredCase& d : ds) {
            for (int fi = 0; fi < 3; ++fi) {
                const ns::AnchoredObjectTask t = builder.build(qs[qi], d.T, ffs[fi]);
                std::printf("task qi=%d d=%s ff=%d", qi, d.name, fi);
                dumpRaw("J", t.J.data(), 72);
                dumpRaw("e", t.e.data(), 6);
                dumpRaw("vff", t.v_ff.data(), 6);
                dumpIso("Tl", t.T_hat_left);
                dumpIso("Tr", t.T_hat_right);
                dumpRaw("gap", &t.anchor_gap.pos_m, 2);
                dumpRaw("margin", &t.legacy_margin_rad, 1);
                std::printf(" inp=%d aok=%d\n", t.inputs_ok ? 1 : 0, t.anchor_ok ? 1 : 0);
            }
        }
    }

    // ---- trajectory grid (the second moved unit) ----
    for (int variant = 0; variant < 2; ++variant) {
        ns::TrajectorySpec spec;
        spec.dt = 0.008;
        spec.time_scaling = (variant == 0) ? "smoothstep" : "linear";
        spec.start_from_baseline = false;
        spec.start_world_T_object = ds[0].T;
        spec.phases = {{"a", 0.30, 0.0, 0.0, 0.0},
                       {"b", 1.50, 0.0, 0.0, 0.050},
                       {"c", 1.50, -0.060, 0.0, 0.0}};
        spec.yaw_sweep_deg = (variant == 0) ? std::vector<double>{0.0}
                                            : std::vector<double>{0.0, 10.0, -5.0, 4.0};
        const ns::FrozenObjectTrajectory traj(spec);
        std::printf("traj variant=%d duration=%a steps=%zu\n", variant,
                    traj.duration(), traj.steps());
        const double ts[11] = {-0.1, 0.0, 0.15, 0.30, 0.9, 1.8, 2.4, 3.3, 3.2999, 3.3, 4.0};
        for (int i = 0; i < 11; ++i) {
            Vec6 ff = Vec6::Constant(0.123);
            const Iso3 T = traj.at(ts[i], &ff);
            std::printf("  traj t=%a", ts[i]);
            dumpIso("T", T);
            dumpRaw("vff", ff.data(), 6);
            std::printf("\n");
        }
        // dense sweep over the full duration at exactly dt
        for (std::size_t k = 0; k <= traj.steps(); k += 7) {
            Vec6 ff;
            const Iso3 T = traj.at(static_cast<double>(k) * spec.dt, &ff);
            std::printf("  dense k=%zu", k);
            dumpIso("T", T);
            dumpRaw("vff", ff.data(), 6);
            std::printf("\n");
        }
    }

    std::printf("# end\n");
    return 0;
}
