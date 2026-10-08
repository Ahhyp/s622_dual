// include/dual_arm_workspace_analysis/workspace_sampler.hpp
// C4.1: dual-arm workspace / manipulability sampler.
//
// Pipeline
//   Halton sample q12 in the joint box
//     -> reject if the arm-arm capsule distance < d_min      (C2.7 model)
//     -> FK: TCP positions of both arms, object frame, ^L T_R
//     -> per-arm geometric Jacobian -> w = prod(sigma), sigma_min
//     -> relative Jacobian (3 frames) -> w_rel, sigma_min, numerical rank
//   One binary record per accepted sample (see kRecordFields).
//
// Conventions
//   * Relative quantities are reported for three frames because Ad(T) is NOT
//     orthogonal for the [v; w] twist convention: rank / dim ker are
//     frame-invariant, but sigma_i(J_rel) and w_rel are NOT.  The main field is
//     BodyR (matches C2.3 relativeError); the other two exist for the S4
//     comparison table.
//   * w = sqrt(det(J J^T)) = prod(sigma_i), computed as exp(sum log sigma_i)
//     so it degrades gracefully instead of underflowing to 0.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace dual_arm_workspace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm::RelativeFrame;

/// Joint limits (rad), identical for both arms.  Source: docs/机械臂参数.md
/// (= URDF <limit> tags).
struct JointLimits {
    std::array<double, 6> lower{-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
    std::array<double, 6> upper{3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};
};

struct SamplerParams {
    std::uint64_t num_samples = 10'000'000ULL;
    int num_threads = 0;  ///< 0 => std::thread::hardware_concurrency()
    double link_radius = 0.06;  ///< capsule radius, same as C2.7
    double d_min = 0.02;        ///< reject a sample if arm-arm distance < d_min
    double rank_tol = 1e-6;     ///< numerical rank: sigma_i > rank_tol * sigma_1
    std::uint64_t seed_offset = 0;  ///< Halton start offset (reproducibility)
    JointLimits limits{};

    /// Ground / work-plane constraint.  `dual_arm_world.sdf` has a ground plane
    /// at z = 0 (the table was deliberately removed -- the board sits on the
    /// ground), so any configuration whose DH frame origins or TCP dip below
    /// z_floor is physically unreachable.  Without this filter a large part of
    /// the sampled workspace is *below the floor*, which would make the common
    /// reachable set useless for choosing handover poses.
    ///
    /// The check uses frame ORIGINS, not the C2.7 capsule radius: that radius is
    /// a conservative proxy for arm-arm clearance, not the real link geometry,
    /// and applying it to the floor would reject the base link outright.
    bool use_z_floor = true;
    double z_floor = 0.0;  ///< [m]
};

/// One accepted (self-collision-free) sample.  Field order is frozen and shared
/// with the Python pipeline -- see kRecordFields / kRecordFloats.
struct SampleRecord {
    Q12 q;                    ///< 12 joint angles
    Eigen::Vector3d p_left;   ///< world TCP (grasp frame) of the left arm
    Eigen::Vector3d p_right;
    Eigen::Vector3d p_object;  ///< world SE(3) midpoint of the two grasps
    Eigen::Vector3d p_rel;     ///< ^L T_R translation
    Eigen::Vector3d rel_rotvec;  ///< logSO3(R_rel), for the fig2 orientation slices
    double d_arms = 0.0;         ///< min arm-arm capsule distance
    double w_left = 0.0;         ///< sqrt(det(J_L J_L^T))
    double w_right = 0.0;
    double smin_left = 0.0;  ///< sigma_min(J_L)
    double smin_right = 0.0;
    double w_rel_body = 0.0;   ///< sqrt(det(J_rel J_rel^T)) in BodyR (MAIN)
    double smin_rel_body = 0.0;
    double w_rel_left = 0.0;   ///< same, expressed in LeftL
    double w_rel_world = 0.0;  ///< same, expressed in WorldSpatial
    double rank_rel = 0.0;     ///< numerical rank of J_rel (frame-invariant)
};

/// 12 (q) + 12 (4 x Vec3 positions) + 3 (rel rotvec) + 10 scalars = 37.
constexpr int kRecordFloats = 37;
/// Comma-separated column names, same order as encode().
extern const char* const kRecordFields;

/// Pack a record into kRecordFloats floats (row-major, no padding).
void encodeRecord(const SampleRecord& r, float* out);

/// Singular-value summary of a Jacobian.
struct SvdMetrics {
    double w = 0.0;      ///< prod(sigma_i)
    double smin = 0.0;   ///< sigma_min
    int rank = 0;        ///< #{sigma_i > tol * sigma_1}
    int num_sv = 0;      ///< number of singular values (min(rows, cols))
};

SvdMetrics svdMetrics(const Eigen::MatrixXd& J, double rank_tol);

struct SamplerStats {
    std::uint64_t requested = 0;
    std::uint64_t evaluated = 0;
    std::uint64_t rejected_floor = 0;     ///< DH frame origin or TCP below z_floor
    std::uint64_t rejected_distance = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rank_defect = 0;  ///< accepted samples with rank_rel < 6
    double wall_seconds = 0.0;
};

/// Evaluate a single configuration.  Returns false when rejected by the
/// arm-arm distance filter (out is untouched in that case).
class WorkspaceSampler {
public:
    explicit WorkspaceSampler(const SamplerParams& params);

    bool evaluate(const Q12& q, SampleRecord* out) const;

    /// Ground-plane test alone (used to attribute rejections).
    bool aboveFloor(const Q12& q) const;

    const SamplerParams& params() const { return params_; }
    const DualArmKinematics& kinematics() const { return kin_; }
    const dual_arm_collision::DualArmCollisionModel& collision() const { return collision_; }

private:
    SamplerParams params_;
    DualArmKinematics kin_;
    dual_arm_collision::DualArmCollisionModel collision_;
};

/// Run the full (multithreaded) sampling.
/// Writes  <out_prefix>.bin          float32 records, kRecordFloats per sample
///         <out_prefix>.json         params + stats + schema
///         <out_prefix>_preview.csv  first 1000 accepted records, human readable
SamplerStats runSampling(const SamplerParams& params, const std::string& out_prefix);

/// Default output prefix used by the CLI.
std::string defaultOutputPrefix();

}  // namespace dual_arm_workspace
