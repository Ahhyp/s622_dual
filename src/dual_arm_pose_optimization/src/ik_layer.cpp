// src/ik_layer.cpp
#include "dual_arm_pose_optimization/ik_layer.hpp"

#include <algorithm>
#include <map>

#include "dual_arm_kinematics/se3.hpp"
#include "fairino_planning_core/dh_kinematics.h"

namespace dual_arm_pose_optimization {

namespace {

fairino_planning::DHKinematics& dhGripper() {
  static fairino_planning::DHKinematics dh(fairino_planning::DHParams{},
                                           fairino_planning::ToolParams::gripper());
  return dh;
}

}  // namespace

std::string rejectReasonName(int reason) {
  return fairino_planning::toString(static_cast<fairino_planning::IKRejectReason>(reason));
}

IkLayer::IkLayer() : ik_(solve_params_), selector_(select_params_) {}

IKResult IkLayer::solves(const Eigen::Matrix4d& target_base, ToolModel model) const {
  return ik_.solve(target_base, model);
}

void IkLayer::residual(const Eigen::Matrix4d& target_base, const JointConfig& q,
                       ToolModel model, double* pos_err, double* rot_err) {
  const Eigen::Matrix4d fk =
      (model == ToolModel::GRIPPER) ? dhGripper().fkine(q, ToolModel::GRIPPER)
                                    : dhGripper().fkine(q, ToolModel::FLANGE);
  const Eigen::Matrix4d d = target_base.inverse() * fk;
  if (pos_err != nullptr) {
    *pos_err = d.block<3, 1>(0, 3).norm();
  }
  if (rot_err != nullptr) {
    const dual_arm::Vec3 w = dual_arm::logSO3(d.block<3, 3>(0, 0));
    *rot_err = w.norm();
  }
}

IkOutcome IkLayer::solveAndSelect(const Eigen::Matrix4d& target_base, const JointConfig& seed,
                                  ToolModel model, IKTaskProfile profile) const {
  IkOutcome out;
  out.tool_model = model;
  out.target_base = target_base;

  const IKResult raw = ik_.solve(target_base, model);
  out.solve_success = raw.success;
  out.total_branches = raw.total_branches;
  out.survive_q1 = raw.survive_q1;
  out.survive_q5 = raw.survive_q5;
  out.survive_q23 = raw.survive_q23;
  out.survive_fk_verify = raw.survive_fk_verify;
  out.survive_unique = raw.survive_unique;
  out.survive_joint_limits = raw.survive_joint_limits;
  out.failure_stage = raw.failure_stage;
  out.failure_code = raw.failure_code;
  out.solutions = raw.solutions;

  IKSelectionRequest req;
  req.solutions = &out.solutions;
  req.seed = seed;
  req.target_pose = target_base;
  req.tool_model = model;
  req.task_profile = profile;

  const IKSelectionResult sel = selector_.select(req);
  out.selected = sel.selected;
  out.n_candidates = sel.diagnostics.size();

  std::map<std::string, int> hist;
  std::vector<std::pair<double, JointConfig>> passing;
  for (const auto& d : sel.diagnostics) {
    if (d.passed_hard_filter) {
      ++out.n_hard_pass;
      passing.emplace_back(d.total_cost, d.q);
    }
    if (d.selected) out.selected_cost = d.total_cost;
    hist[rejectReasonName(static_cast<int>(d.reject_reason))] += 1;
  }
  std::sort(passing.begin(), passing.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  for (auto& kv : passing) out.hard_pass_solutions.push_back(kv.second);
  for (const auto& kv : hist) out.reject_histogram.emplace_back(kv.first, kv.second);

  if (sel.selected.has_value()) {
    out.ok = true;
    residual(target_base, *sel.selected, model, &out.residual_pos, &out.residual_rot);
  }
  return out;
}

}  // namespace dual_arm_pose_optimization
