// src/m4_baseline.cpp
#include "dual_arm_pose_optimization/m4_baseline.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

namespace dual_arm_pose_optimization {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;

namespace {

DualArmKinematics& kin() {
  static DualArmKinematics k;
  return k;
}

Eigen::Matrix4d toMatrix(const Iso3& T) { return T.matrix(); }

}  // namespace

dual_arm::Iso3 M4Params::objectInLeftGrasp() const {
  Iso3 T = Iso3::Identity();
  T.translation() = object_in_left_grasp_pos;
  T.linear() = object_in_left_grasp_quat.normalized().toRotationMatrix();
  return T;
}

dual_arm::Q12 BaselinePose::q12() const {
  return DualArmKinematics::stack(q_left, q_right);
}

std::vector<double> BaselinePose::q12Vector() const {
  const dual_arm::Q12 q = q12();
  std::vector<double> v(12);
  for (int i = 0; i < 12; ++i) v[static_cast<std::size_t>(i)] = q(i);
  return v;
}

Eigen::Matrix4d m4ReceiveTargetWorld(const Iso3& world_T_left, const M4Params& params) {
  // rod centre in world = FK_left * (^G_L T_O)
  const Iso3 world_T_object = world_T_left * params.objectInLeftGrasp();

  // bb.receive_offset is declared frame: "world" -> a plain world-axis offset
  // added to the rod centre (NOT rotated by the rod orientation).
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
  target.block<3, 3>(0, 0) = params.receive_quat.normalized().toRotationMatrix();
  target.block<3, 1>(0, 3) =
      world_T_object.translation() + params.receive_offset_world;
  return target;
}

BaselinePose rebuildBaselineB(const std::shared_ptr<DualArmScene>& scene,
                              const M4Params& params) {
  BaselinePose bp;
  bp.id = "M4_baseline_B";
  bp.official = true;
  bp.provisional = true;  // Q1..Q4 unresolved -> "基于当前配置的临时重建基线"
  bp.rebuilt_via_ik = true;

  bp.q_left = params.left_hold_joints;
  bp.world_T_left = kin().worldTgrasp(Arm::Left, bp.q_left);

  // ---- M4 RECEIVE target, world frame -----------------------------------
  bp.target_world = m4ReceiveTargetWorld(bp.world_T_left, params);

  // Express it in the RIGHT arm's base frame for the analytic solver.
  const Eigen::Matrix4d world_T_right_base = kin().worldTbase(Arm::Right).matrix();
  bp.target_right_base = world_T_right_base.inverse() * bp.target_world;

  // ---- how wrong would the un-rebuilt base point be? ---------------------
  {
    const Eigen::Matrix4d fk_base =
        kin().dh(Arm::Right).fkine(params.right_receive_base, fairino_planning::ToolModel::GRIPPER);
    const Eigen::Matrix4d fk_world = world_T_right_base * fk_base;
    const Eigen::Matrix4d d = bp.target_world.inverse() * fk_world;
    bp.pre_rebuild_pos_err = d.block<3, 1>(0, 3).norm();
    bp.pre_rebuild_rot_err = dual_arm::logSO3(d.block<3, 3>(0, 0)).norm();
  }

  // ---- IK + branch selection --------------------------------------------
  IkLayer ik;
  bp.right_ik = ik.solveAndSelect(bp.target_right_base, params.right_receive_base,
                                  fairino_planning::ToolModel::GRIPPER,
                                  fairino_planning::IKTaskProfile::Grasp);
  if (!bp.right_ik.ok || !bp.right_ik.selected.has_value()) {
    throw SceneLoadError(
        "rebuildBaselineB: no IK branch selected for the M4 RECEIVE target "
        "(solve_success=" + std::string(bp.right_ik.solve_success ? "true" : "false") +
        ", solutions=" + std::to_string(bp.right_ik.solutions.size()) +
        ", hard_pass=" + std::to_string(bp.right_ik.n_hard_pass) + ")");
  }
  bp.q_right = *bp.right_ik.selected;
  bp.post_rebuild_pos_err = bp.right_ik.residual_pos;
  bp.post_rebuild_rot_err = bp.right_ik.residual_rot;

  // ---- re-derive everything by FK ---------------------------------------
  bp.world_T_right = kin().worldTgrasp(Arm::Right, bp.q_right);
  bp.world_T_object = bp.world_T_left * params.objectInLeftGrasp();
  bp.left_T_right = kin().leftTright(bp.q12());
  return bp;
}

BaselinePose baselineAHistorical() {
  BaselinePose bp;
  bp.id = "baseline_A_historical";
  bp.official = false;
  bp.provisional = false;

  bp.q_left = (JointConfig() << -0.5963, -1.6969, -2.2265, 0.7825, 2.1677, 1.5715)
                  .finished();
  bp.q_right = (JointConfig() << -2.5432, -1.4419, 2.2229, -3.9223, -2.1697, 1.5713)
                   .finished();
  bp.world_T_left = kin().worldTgrasp(Arm::Left, bp.q_left);
  bp.world_T_right = kin().worldTgrasp(Arm::Right, bp.q_right);
  bp.world_T_object = bp.world_T_left;  // no object is defined for the annex
  bp.left_T_right = kin().leftTright(bp.q12());
  return bp;
}

collision_detection::AllowedCollisionMatrix addRodToScene(
    const std::shared_ptr<DualArmScene>& scene, const BaselinePose& pose,
    const M4Params& params) {
  moveit_msgs::msg::CollisionObject co;
  co.id = params.object_id;
  co.header.frame_id = scene->robotModel()->getModelFrame();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {params.object_size.x(), params.object_size.y(), params.object_size.z()};
  geometry_msgs::msg::Pose p;
  p.position.x = pose.world_T_object.translation().x();
  p.position.y = pose.world_T_object.translation().y();
  p.position.z = pose.world_T_object.translation().z();
  const Eigen::Quaterniond q(pose.world_T_object.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  co.primitives.push_back(box);
  co.primitive_poses.push_back(p);
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene->planningScene()->processCollisionObjectMsg(co);

  // NOTE: PlanningScene::isStateColliding() consults the SCENE's ACM, so the
  // exemptions must be installed back into the scene -- merely returning a
  // modified copy leaves every gripper/object contact reported as a collision.
  collision_detection::AllowedCollisionMatrix& acm =
      scene->planningScene()->getAllowedCollisionMatrixNonConst();
  for (const std::string& link : params.object_touch_links) {
    if (scene->robotModel()->getLinkModel(link) != nullptr) {
      acm.setEntry(link, params.object_id, true);
    }
  }
  return acm;
}

}  // namespace dual_arm_pose_optimization
