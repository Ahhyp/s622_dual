// include/dual_arm_pose_optimization/feasibility.hpp
// C4.2 / S3: the layered feasibility ladder S0 -> S4.
//
//   S0  both TCPs are points the corresponding arm has been observed to reach
//       (C4.1 voxel masks)                       -- "point reachable"
//   S1  each arm ALONE has a target-conformant IK solution that passes the
//       selector's hard filters                 -- "pose reachable, one arm"
//   S2  both arms simultaneously                -- "pose reachable, dual arm"
//   S3  S2 AND the relative grasp constraint is met
//   S4  S3 AND there is a solution PAIR that is collision free (self + floor +
//       rod), evaluated over the hard-passing branches of BOTH arms
//
// Two modelling decisions are recorded here because they change what S3 means:
//
//  (A) "procedural"  : the right target keeps `receive_quat` and takes a
//                      world-axis offset from the rod centre (what the M4 BT
//                      literally does).  Only correct at the nominal pose.
//  (B) "rigid grasp" : the relative grasp pose is FIXED by the two grasp
//                      geometries, so T_R^des = T_L^des * T_rel^B.  This is the
//                      physically right model for a rigid rod held by two
//                      grippers, and it is what is implemented.
//
//  (A) and (B) agree exactly at the nominal M4 pose and under V0 (orientation
//  fixed); they diverge as soon as the object orientation is perturbed (V1),
//  where (A) would silently break the rigid grasp.
//
// Consequence, reported honestly rather than hidden: under (B) the H4 relative
// grasp residual is satisfied BY CONSTRUCTION (it only carries the IK
// residual), so |S3|/|S2| is 100% as a structural identity, not as a finding.
// A unit test asserts exactly that, which is what makes the identity checkable.

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/candidate_pool.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/ik_layer.hpp"
#include "dual_arm_pose_optimization/m4_baseline.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace dual_arm_pose_optimization {

/// A candidate object pose.  V0: R_O is the M4 orientation, only p_O varies.
struct Candidate {
  Eigen::Vector3d p_O = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_O = Eigen::Matrix3d::Identity();
  std::string pool = "P1_handover";

  Eigen::Matrix4d T_O() const;
};

struct LayerFlags {
  bool s0_point_reachable = false;
  bool s1_left = false;
  bool s1_right = false;
  bool s2_dual = false;
  bool s3_grasp = false;
  bool s4_collision_free = false;
};

struct CandidateResult {
  Candidate candidate;
  LayerFlags layer;

  Eigen::Vector3d p_L = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_R = Eigen::Vector3d::Zero();
  bool s0_left = false;   ///< diagnostic split of S0
  bool s0_right = false;

  IkOutcome left_ik;
  IkOutcome right_ik;

  std::size_t pair_attempts = 0;        ///< (qL,qR) combinations actually tested
  double rel_grasp_pos_err = -1.0;      ///< H4 residual of the chosen pair [m]
  double rel_grasp_rot_err = -1.0;      ///< H4 residual of the chosen pair [rad]
  double mesh_arm_arm = -1.0;           ///< cross-arm mesh clearance of the chosen pair
  double capsule_arm_arm = -1.0;
  std::string mesh_pair_a, mesh_pair_b;  ///< which link pair realises that clearance
  bool has_chosen_pair = false;
  dual_arm::JointConfig q_left_chosen = dual_arm::JointConfig::Zero();
  dual_arm::JointConfig q_right_chosen = dual_arm::JointConfig::Zero();

  // ---- raw objective inputs (filled for S4 candidates) -------------------
  // Kept RAW here; normalisation belongs to the optimizer (S4).
  double smin_left = 0.0;    ///< sigma_min(J_L) of the chosen solution
  double smin_right = 0.0;   ///< sigma_min(J_R)
  double w_rel = 0.0;        ///< relative manipulability (BodyR, frame-invariant)
  double joint_margin = 0.0; ///< min over both arms of min(q-lo, hi-q) [rad]
  double deviation = 0.0;    ///< ||q12 - q12_baseline||  (limit-normalised span)
  double continuity = 0.0;   ///< IKSelector S1 of the chosen solution [0,1]
  double arm_proximal_clearance = -1.0;  ///< cross-arm min with >=1 proximal link [m]
  std::string arm_proximal_pair_a, arm_proximal_pair_b;
};

struct FeasibilityOptions {
  double eps_rel_pos = 1e-3;                    ///< H4 epsilon [m]
  double eps_rel_rot = 0.5 * M_PI / 180.0;      ///< H4 epsilon [rad]
  std::size_t max_pairs = 64;                   ///< safety cap on pair enumeration
  bool measure_clearance_of_chosen = true;
  bool also_report_capsule = true;
};

class FeasibilityPipeline {
public:
  FeasibilityPipeline(const std::shared_ptr<DualArmScene>& scene,
                      const M4Params& m4, const BaselinePose& baselineB,
                      const FeasibilityOptions& options = FeasibilityOptions{});

  /// Evaluate one candidate.  `index` (optional) supplies the S0 masks; when
  /// null, S0 is treated as satisfied (useful for unit tests).
  CandidateResult evaluate(const Candidate& c, const WorkspaceIndex* index = nullptr);

  /// Desired grasps for a candidate object pose (world frame).
  void desiredGrasps(const Candidate& c, Eigen::Matrix4d* T_L, Eigen::Matrix4d* T_R) const;

  const M4Params& m4() const { return m4_; }
  const BaselinePose& baseline() const { return baseline_; }
  const FeasibilityOptions& options() const { return opts_; }

private:
  std::shared_ptr<DualArmScene> scene_;
  M4Params m4_;
  BaselinePose baseline_;
  FeasibilityOptions opts_;
  IkLayer ik_;
  std::unique_ptr<MoveItCollisionBackend> collision_;
  Eigen::Matrix4d T_rel_fixed_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d world_T_left_base_inv_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d world_T_right_base_inv_ = Eigen::Matrix4d::Identity();
};

/// Move the (already added) rod to a new world pose.  Requires addRodToScene()
/// to have run once; no-op if the object does not exist yet is NOT assumed.
void setRodPose(const std::shared_ptr<DualArmScene>& scene,
                const dual_arm::Iso3& world_T_object, const M4Params& params);

}  // namespace dual_arm_pose_optimization
