// include/dual_arm_pose_optimization/ik_layer.hpp
// C4.2 / S2: the IK layer -- analytic multi-solution IK + branch selection.
//
// Per the plan (额外要求 2) we must NOT take "the first IK solution" per hand.
// `FairinoIK` returns up to 8 branches; `IKSelector` scores every surviving
// candidate on four already-normalised dimensions (continuity / manipulability /
// posture / joint safety) and applies hard filters that include per-chain ground
// clearance (the "don't judge the floor from DH origins alone" requirement).
//
// This wrapper exists to make the two-step protocol explicit and testable:
//     solves()  -> raw analytic branches
//     select()  -> hard-filtered, scored, seeded branch choice

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "fairino_planning_core/ik/fairino_ik.h"
#include "fairino_planning_core/ik/ik_selector.h"

namespace dual_arm_pose_optimization {

using fairino_planning::AnalyticalIKParams;
using fairino_planning::IKResult;
using fairino_planning::IKSelectParams;
using fairino_planning::IKSelectionRequest;
using fairino_planning::IKSelectionResult;
using fairino_planning::IKTaskProfile;
using fairino_planning::ToolModel;
using fairino_planning::JointConfig;

/// Outcome of one "solve then select" query for a single arm.
struct IkOutcome {
  bool ok = false;                 ///< a branch was selected
  ToolModel tool_model = ToolModel::GRIPPER;
  Eigen::Matrix4d target_base = Eigen::Matrix4d::Identity();

  // raw analytic stage
  bool solve_success = false;
  int total_branches = 0;          ///< branches the solver enumerated (8)
  int survive_q1 = 0, survive_q5 = 0, survive_q23 = 0;
  int survive_fk_verify = 0, survive_unique = 0, survive_joint_limits = 0;
  std::string failure_stage{"none"};
  std::string failure_code{"none"};
  std::vector<JointConfig> solutions;

  // selection stage
  std::optional<JointConfig> selected;
  std::size_t n_candidates = 0;    ///< candidates the selector scored
  std::size_t n_hard_pass = 0;     ///< candidates that passed the hard filters
  double selected_cost = 0.0;

  /// Per-candidate reject reasons, as "reason: count" (for the S2 report).
  std::vector<std::pair<std::string, int>> reject_histogram;

  /// Residual of the chosen solution against the target (position [m], rotation [rad]).
  double residual_pos = 0.0;
  double residual_rot = 0.0;
};

/// Analytic IK + selector for ONE arm (the S622 is a 6-DOF arm; the dual-arm
/// layer is built by simply running this twice).
class IkLayer {
public:
  IkLayer();

  /// Raw analytic solutions for a target expressed in the ARM BASE frame.
  IKResult solves(const Eigen::Matrix4d& target_base,
                  ToolModel model = ToolModel::GRIPPER) const;

  /// Solve + select.  `seed` steers the continuity term; pass the previous /
  /// nominal configuration (e.g. right_receive_base when rebuilding M4).
  IkOutcome solveAndSelect(const Eigen::Matrix4d& target_base, const JointConfig& seed,
                           ToolModel model = ToolModel::GRIPPER,
                           IKTaskProfile profile = IKTaskProfile::Grasp) const;

  const IKSelectParams& selectParams() const { return select_params_; }
  IKSelectParams& selectParams() { return select_params_; }
  const AnalyticalIKParams& solveParams() const { return solve_params_; }

  /// Forward kinematics residual of a configuration against a base-frame target.
  static void residual(const Eigen::Matrix4d& target_base, const JointConfig& q,
                       ToolModel model, double* pos_err, double* rot_err);

private:
  AnalyticalIKParams solve_params_;
  IKSelectParams select_params_;
  fairino_planning::FairinoIK ik_;
  fairino_planning::IKSelector selector_;
};

/// Human-readable name for a reject reason (for reports).
std::string rejectReasonName(int reason);

}  // namespace dual_arm_pose_optimization
