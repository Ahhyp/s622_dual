// src/feasibility.cpp
#include "dual_arm_pose_optimization/feasibility.hpp"

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

moveit_msgs::msg::CollisionObject rodMessage(const std::string& frame_id,
                                             const Iso3& world_T_object,
                                             const M4Params& params, int operation) {
  moveit_msgs::msg::CollisionObject co;
  co.id = params.object_id;
  co.header.frame_id = frame_id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {params.object_size.x(), params.object_size.y(), params.object_size.z()};
  geometry_msgs::msg::Pose p;
  p.position.x = world_T_object.translation().x();
  p.position.y = world_T_object.translation().y();
  p.position.z = world_T_object.translation().z();
  const Eigen::Quaterniond q(world_T_object.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  co.primitives.push_back(box);
  co.primitive_poses.push_back(p);
  co.operation = static_cast<std::int8_t>(operation);
  return co;
}

}  // namespace

void setRodPose(const std::shared_ptr<DualArmScene>& scene, const Iso3& world_T_object,
                const M4Params& params) {
  scene->planningScene()->processCollisionObjectMsg(
      rodMessage(scene->robotModel()->getModelFrame(), world_T_object, params,
                 moveit_msgs::msg::CollisionObject::MOVE));
}

Eigen::Matrix4d Candidate::T_O() const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R_O;
  T.block<3, 1>(0, 3) = p_O;
  return T;
}

FeasibilityPipeline::FeasibilityPipeline(const std::shared_ptr<DualArmScene>& scene,
                                         const M4Params& m4, const BaselinePose& baselineB,
                                         const FeasibilityOptions& options)
    : scene_(scene), m4_(m4), baseline_(baselineB), opts_(options) {
  if (!scene_) throw SceneLoadError("FeasibilityPipeline: null scene");
  if (!baseline_.rebuilt_via_ik) {
    throw SceneLoadError("FeasibilityPipeline: baseline B must be the IK-rebuilt one");
  }

  EnvironmentOptions env;
  env.add_floor = true;
  collision_ = std::make_unique<MoveItCollisionBackend>(scene_, env);

  // Install the rod once (ADD + object<->gripper ACM exemptions).
  addRodToScene(scene_, baseline_, m4_);

  // Fixed relative grasp pose, from the rebuilt baseline.
  T_rel_fixed_ = baseline_.left_T_right.matrix();

  world_T_left_base_inv_ = kin().worldTbase(Arm::Left).matrix().inverse();
  world_T_right_base_inv_ = kin().worldTbase(Arm::Right).matrix().inverse();
}

void FeasibilityPipeline::desiredGrasps(const Candidate& c, Eigen::Matrix4d* T_L,
                                        Eigen::Matrix4d* T_R) const {
  const Eigen::Matrix4d T_O = c.T_O();
  const Eigen::Matrix4d G_L_T_O = m4_.objectInLeftGrasp().matrix();
  const Eigen::Matrix4d TL = T_O * G_L_T_O.inverse();   // rigid-grasp model (B)
  if (T_L != nullptr) *T_L = TL;
  if (T_R != nullptr) *T_R = TL * T_rel_fixed_;
}

CandidateResult FeasibilityPipeline::evaluate(const Candidate& c,
                                              const WorkspaceIndex* index) {
  CandidateResult r;
  r.candidate = c;

  // ---- desired grasps ---------------------------------------------------
  Eigen::Matrix4d T_L = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_R = Eigen::Matrix4d::Identity();
  desiredGrasps(c, &T_L, &T_R);
  r.p_L = T_L.block<3, 1>(0, 3);
  r.p_R = T_R.block<3, 1>(0, 3);

  // ---- S0: point reachability from C4.1's masks -------------------------
  r.s0_left = (index == nullptr) ? true : index->leftHas(r.p_L);
  r.s0_right = (index == nullptr) ? true : index->rightHas(r.p_R);
  r.layer.s0_point_reachable = r.s0_left && r.s0_right;

  // ---- S1: per-arm pose reachability -----------------------------------
  const Eigen::Matrix4d tl_base = world_T_left_base_inv_ * T_L;
  const Eigen::Matrix4d tr_base = world_T_right_base_inv_ * T_R;

  r.left_ik = ik_.solveAndSelect(tl_base, baseline_.q_left,
                                 fairino_planning::ToolModel::GRIPPER,
                                 fairino_planning::IKTaskProfile::Grasp);
  r.right_ik = ik_.solveAndSelect(tr_base, baseline_.q_right,
                                  fairino_planning::ToolModel::GRIPPER,
                                  fairino_planning::IKTaskProfile::Grasp);
  r.layer.s1_left = r.left_ik.ok;
  r.layer.s1_right = r.right_ik.ok;
  r.layer.s2_dual = r.layer.s1_left && r.layer.s1_right;
  if (!r.layer.s2_dual) return r;

  // ---- S3: relative grasp constraint (H4) on the selected solutions ------
  {
    const Iso3 wl = kin().worldTgrasp(Arm::Left, *r.left_ik.selected);
    const Iso3 wr = kin().worldTgrasp(Arm::Right, *r.right_ik.selected);
    const Eigen::Matrix4d T_err = T_rel_fixed_.inverse() * (wl.matrix().inverse() * wr.matrix());
    r.rel_grasp_pos_err = T_err.block<3, 1>(0, 3).norm();
    r.rel_grasp_rot_err = dual_arm::logSO3(T_err.block<3, 3>(0, 0)).norm();
    r.layer.s3_grasp = r.rel_grasp_pos_err <= opts_.eps_rel_pos &&
                       r.rel_grasp_rot_err <= opts_.eps_rel_rot;
  }
  if (!r.layer.s3_grasp) return r;

  // ---- S4: collision-free solution PAIR (all hard-passing branches) ------
  // The rod sits at the CANDIDATE object pose -- it is a property of the
  // candidate, not of the solution pair (S3 already asserted that the chosen
  // pair reproduces that pose to within the IK residual).
  {
    Iso3 T_O_iso = Iso3::Identity();
    T_O_iso.matrix() = c.T_O();
    setRodPose(scene_, T_O_iso, m4_);
  }

  const std::vector<JointConfig>& cand_L = r.left_ik.hard_pass_solutions;
  const std::vector<JointConfig>& cand_R = r.right_ik.hard_pass_solutions;

  for (const JointConfig& ql : cand_L) {
    for (const JointConfig& qr : cand_R) {
      if (r.pair_attempts >= opts_.max_pairs) break;
      ++r.pair_attempts;
      const dual_arm::Q12 q12 = DualArmKinematics::stack(ql, qr);
      const auto st = scene_->makeStateQ12(
          std::vector<double>(q12.data(), q12.data() + 12));
      if (collision_->isStateColliding(st)) continue;

      r.layer.s4_collision_free = true;
      r.has_chosen_pair = true;
      r.q_left_chosen = ql;
      r.q_right_chosen = qr;
      if (opts_.measure_clearance_of_chosen) {
        const DistanceReading d = collision_->distances(
            st, opts_.also_report_capsule ? &q12 : nullptr);
        r.mesh_arm_arm = d.arm_arm;
        r.capsule_arm_arm = d.capsule_arm_arm;
        r.mesh_pair_a = d.arm_arm_link_a;
        r.mesh_pair_b = d.arm_arm_link_b;
      }
      return r;
    }
  }
  return r;
}

}  // namespace dual_arm_pose_optimization
