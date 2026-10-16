// src/ik_enum.cpp -- W2: full branch enumeration + independent FK verification.
#include "dual_arm_transport/ik_enum.hpp"

#include <cmath>
#include <sstream>

#include "dual_arm_pose_optimization/ik_layer.hpp"

namespace dual_arm_transport {
namespace {

const dual_arm_pose_optimization::IkLayer& layer() {
  static const dual_arm_pose_optimization::IkLayer l;
  return l;
}

/// Independent rotation-angle error between two rotations, via the SE3 log.
double rotationError(const dual_arm::Mat3& R_a, const dual_arm::Mat3& R_b) {
  dual_arm::Iso3 E = dual_arm::Iso3::Identity();
  E.linear() = R_a.transpose() * R_b;
  return dual_arm::logSE3(E).tail<3>().norm();
}

}  // namespace

DualGraspTargets graspTargets(const GraspConfig& cfg) {
  DualGraspTargets t;
  t.world_T_object = cfg.grasp.world_T_object_star;
  t.world_T_left_tcp = cfg.grasp.world_T_object_star * cfg.grasp.object_T_left_tcp;
  t.world_T_right_tcp = cfg.grasp.world_T_object_star * cfg.grasp.object_T_right_tcp;
  return t;
}

std::pair<double, double> fingerStrokes(const GraspConfig& cfg, double pad_clearance_m) {
  const double d = cfg.grip_thickness_m / 2.0 + pad_clearance_m;
  // Measured from the URDF: pad gap = 2 * |q|; finger1 opens towards +q and
  // finger2 towards -q (the two prismatic joints have opposite sign conventions).
  return {+d, -d};
}

std::string BranchSet::to_string() const {
  std::ostringstream os;
  os << arm << " branch set:\n"
     << "  target(world) = " << target_world.translation().transpose() << "\n"
     << "  ok=" << (ok ? "true" : "false") << " branches=" << total_branches << " (q1 "
     << survive_q1 << " -> q5 " << survive_q5 << " -> q23 " << survive_q23 << " -> fk_verify "
     << survive_fk_verify << " -> unique " << survive_unique << " -> limits "
     << survive_joint_limits << ")\n"
     << "  returned " << solutions.size() << " solution(s)";
  if (!ok) os << "  failure: " << failure_stage << "/" << failure_code;
  for (std::size_t i = 0; i < solutions.size(); ++i) {
    os << "\n    [" << i << "] fk_pos_err=" << fk_pos_err[i] << " fk_rot_err=" << fk_rot_err[i];
  }
  return os.str();
}

BranchSet enumerateBranches(const GraspConfig& cfg, const dual_arm::DualArmKinematics& kin,
                            const dual_arm::Iso3& target_world, const std::string& arm,
                            double fk_pos_tol, double fk_rot_tol) {
  (void)cfg;
  const dual_arm::Arm a = (arm == "left") ? dual_arm::Arm::Left : dual_arm::Arm::Right;
  BranchSet bs;
  bs.arm = arm;
  bs.target_world = target_world;
  bs.target_base = kin.worldTbase(a).inverse() * target_world;

  const fairino_planning::IKResult r =
      layer().solves(bs.target_base.matrix(), fairino_planning::ToolModel::GRIPPER);

  bs.total_branches = r.total_branches;
  bs.survive_q1 = r.survive_q1;
  bs.survive_q5 = r.survive_q5;
  bs.survive_q23 = r.survive_q23;
  bs.survive_fk_verify = r.survive_fk_verify;
  bs.survive_unique = r.survive_unique;
  bs.survive_joint_limits = r.survive_joint_limits;
  bs.failure_stage = r.failure_stage;
  bs.failure_code = r.failure_code;

  // Independent FK re-check.  A branch is kept only if OUR kinematics reproduces
  // the target -- the solver's own fk_verify is not accepted as evidence.
  for (const dual_arm::JointConfig& q : r.solutions) {
    const dual_arm::Iso3 fk = kin.worldTgrasp(a, q);
    const double pe = (fk.translation() - target_world.translation()).norm();
    const double re = rotationError(fk.linear(), target_world.linear());
    if (!(pe <= fk_pos_tol) || !(re <= fk_rot_tol)) continue;
    bs.solutions.push_back(q);
    bs.fk_pos_err.push_back(pe);
    bs.fk_rot_err.push_back(re);
  }

  bs.ok = r.success && !bs.solutions.empty();
  return bs;
}

}  // namespace dual_arm_transport
