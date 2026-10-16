// src/path_check.cpp -- W5/W6 implementation.
#include "dual_arm_transport/path_check.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_transport {
namespace {

std::string num(double v, int p = 6) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(p) << v;
  return os.str();
}

}  // namespace

PathChecker::PathChecker(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                         const GraspConfig& cfg, const PathCheckOptions& opt)
    : scene_(std::move(scene)), cfg_(cfg), opt_(opt) {
  if (scene_ == nullptr) throw GraspConfigError("PathChecker: null scene");
  dual_arm_pose_optimization::EnvironmentOptions env;
  env.add_floor = true;
  backend_ = std::make_unique<dual_arm_pose_optimization::MoveItCollisionBackend>(scene_, env);
}

moveit::core::RobotState PathChecker::stateOf(const dual_arm::Q12& q12) const {
  moveit::core::RobotState st =
      scene_->makeStateQ12(std::vector<double>(q12.data(), q12.data() + 12));
  // ★ The standby motion and the approach BOTH happen with the fingers OPEN;
  // only the final closure brings the pads onto the sample.  `makeStateQ12`
  // leaves the prismatic finger joints at their default 0.0, which is the
  // CLOSED position -- with the sample present every such state is in collision
  // with it, which is what made every joint segment look like a collision.
  const double open = cfg_.gripper_max_opening_m / 2.0;
  st.setVariablePosition("left_finger1_joint", +open);
  st.setVariablePosition("left_finger2_joint", -open);
  st.setVariablePosition("right_finger1_joint", +open);
  st.setVariablePosition("right_finger2_joint", -open);
  st.update();
  return st;
}

bool PathChecker::withinJointLimits(const dual_arm::Q12& q12, std::string* detail) const {
  const moveit::core::RobotModel& model = *scene_->robotModel();
  const std::vector<std::string>& names = scene_->armJointNames();
  for (std::size_t k = 0; k < names.size() && k < 12; ++k) {
    const moveit::core::JointModel* jm = model.getJointModel(names[k]);
    if (jm == nullptr || jm->getVariableCount() != 1) continue;
    // NOTE: JointModel::getVariableBounds() is overloaded on the variable NAME
    // (std::string), so `getVariableBounds(0)` compiles -- 0 converts to a null
    // const char* -- and then throws std::logic_error
    // "basic_string::_M_construct null not valid" at run time.  Always look the
    // bound up by the variable's own name.
    const std::vector<std::string>& var_names = jm->getVariableNames();
    if (var_names.size() != 1) continue;
    const moveit::core::VariableBounds& bounds = jm->getVariableBounds(var_names.front());
    const double lo = bounds.min_position_;
    const double hi = bounds.max_position_;
    const double v = q12[static_cast<Eigen::Index>(k)];
    if (v < lo - 1e-9 || v > hi + 1e-9) {
      if (detail != nullptr) {
        *detail = names[k] + " = " + num(v) + " outside [" + num(lo) + ", " + num(hi) + "]";
      }
      return false;
    }
  }
  return true;
}

SegmentResult PathChecker::jointSegment(const dual_arm::Q12& a, const dual_arm::Q12& b) const {
  SegmentResult r;
  std::string why;
  if (!withinJointLimits(a, &why) || !withinJointLimits(b, &why)) {
    r.code = "joint_limit";
    r.detail = "endpoint outside joint limits: " + why;
    return r;
  }
  const int n = std::max(2, opt_.samples);
  r.n_samples = static_cast<std::size_t>(n + 1);
  r.min_clearance_m = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= n; ++k) {
    const double s = static_cast<double>(k) / static_cast<double>(n);
    const dual_arm::Q12 q = a + s * (b - a);
    const moveit::core::RobotState st = stateOf(q);
    const auto reading = backend_->distances(st, &q);
    r.min_clearance_m = std::min(r.min_clearance_m, reading.whole_robot);
    if (reading.in_collision) {
      r.code = "collision";
      r.first_bad_sample = k;
      r.q_bad = q;
      r.detail = "collision at sample " + std::to_string(k) + "/" + std::to_string(n) + " (" +
                 reading.whole_link_a + "<->" + reading.whole_link_b + ")";
      return r;
    }
    if (reading.whole_robot < opt_.min_clearance_m) {
      r.code = "insufficient_clearance";
      r.first_bad_sample = k;
      r.q_bad = q;
      r.detail = "clearance " + num(reading.whole_robot) + " m < " + num(opt_.min_clearance_m) +
                 " m at sample " + std::to_string(k) + "/" + std::to_string(n);
      return r;
    }
  }
  r.ok = true;
  r.code = "ok";
  return r;
}

Iso3Pair PathChecker::pregraspTargets(const Iso3Pair& grasp_world) const {
  Iso3Pair out;
  // The tool z axis points from the TCP into the sample (see grasp_config.cpp),
  // so backing off is -z in the TCP frame.
  const Eigen::Vector3d back(0.0, 0.0, -cfg_.d_approach_m);
  out.left = grasp_world.left;
  out.left.translation() += grasp_world.left.linear() * back;
  out.right = grasp_world.right;
  out.right.translation() += grasp_world.right.linear() * back;
  return out;
}

SegmentResult PathChecker::approachSegment(const dual_arm::Q12& q_back, const dual_arm::Q12& q_grasp,
                                           const Iso3Pair& task_world) const {
  SegmentResult r;
  std::string why;
  if (!withinJointLimits(q_back, &why) || !withinJointLimits(q_grasp, &why)) {
    r.code = "joint_limit";
    r.detail = "endpoint outside joint limits: " + why;
    return r;
  }

  dual_arm::DualArmKinematics kin;
  const int n = std::max(2, opt_.samples);
  r.n_samples = static_cast<std::size_t>(n + 1);
  r.min_clearance_m = std::numeric_limits<double>::infinity();

  const Iso3Pair pre = pregraspTargets(task_world);
  const Eigen::Vector3d p0l = pre.left.translation();
  const Eigen::Vector3d p1l = task_world.left.translation();
  const Eigen::Vector3d p0r = pre.right.translation();
  const Eigen::Vector3d p1r = task_world.right.translation();

  for (int k = 0; k <= n; ++k) {
    const double s = static_cast<double>(k) / static_cast<double>(n);
    const dual_arm::Q12 q = q_back + s * (q_grasp - q_back);

    // Straight-line tracking of BOTH TCPs.
    {
      const Eigen::Vector3d wl = kin.worldTgrasp(dual_arm::Arm::Left, q.head<6>()).translation();
      const Eigen::Vector3d wr = kin.worldTgrasp(dual_arm::Arm::Right, q.tail<6>()).translation();
      const double dl = ((p0l + s * (p1l - p0l)) - wl).norm();
      const double dr = ((p0r + s * (p1r - p0r)) - wr).norm();
      r.max_line_dev_m = std::max(r.max_line_dev_m, std::max(dl, dr));
    }

    const moveit::core::RobotState st = stateOf(q);
    const auto reading = backend_->distances(st, &q);
    r.min_clearance_m = std::min(r.min_clearance_m, reading.whole_robot);
    if (reading.in_collision) {
      r.code = "collision";
      r.first_bad_sample = k;
      r.q_bad = q;
      r.detail = "collision at approach sample " + std::to_string(k) + " (" +
                 reading.whole_link_a + "<->" + reading.whole_link_b + ")";
      return r;
    }
    if (reading.whole_robot < opt_.min_clearance_m) {
      r.code = "insufficient_clearance";
      r.first_bad_sample = k;
      r.q_bad = q;
      r.detail = "clearance " + num(reading.whole_robot) + " m < " + num(opt_.min_clearance_m) +
                 " m at approach sample " + std::to_string(k);
      return r;
    }
  }

  if (opt_.require_straight_approach && r.max_line_dev_m > opt_.approach_line_tol_m) {
    r.code = "approach_not_straight";
    r.detail = "max Cartesian departure " + num(r.max_line_dev_m) + " m > " +
               num(opt_.approach_line_tol_m) +
               " m (joint interpolation does not realize a straight-line approach)";
    return r;
  }
  r.ok = true;
  r.code = "ok";
  return r;
}

ReachabilityResult PathChecker::check(const dual_arm::Q12& q_standby, const dual_arm::Q12& q_grasp,
                                      const Iso3Pair& task_world) const {
  ReachabilityResult out;

  // (a) standby -> grasp in joint space
  out.standby_segment = jointSegment(q_standby, q_grasp);
  if (!out.standby_segment.ok) {
    out.code = "standby_segment_" + out.standby_segment.code;
    out.detail = out.standby_segment.detail;
    return out;
  }

  // (b) the final approach, after solving IK at the backed-off pose
  const Iso3Pair pre = pregraspTargets(task_world);
  dual_arm::DualArmKinematics kin;
  const auto solveOne = [&](dual_arm::Arm a, const dual_arm::Iso3& target,
                            const dual_arm::JointConfig& seed, dual_arm::JointConfig* out_q,
                            std::string* err) {
    const dual_arm::Iso3 target_base = kin.worldTbase(a).inverse() * target;
    const fairino_planning::IKResult res = ik_layer_.solves(target_base.matrix());
    if (!res.success || res.solutions.empty()) {
      *err = std::string("ik failed at pre-grasp (") + res.failure_code + ")";
      return false;
    }
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const auto& s : res.solutions) {
      const double d = (s - seed).norm();
      if (d < best) {
        best = d;
        *out_q = s;
        found = true;
      }
    }
    if (!found) {
      *err = "ik returned no usable pre-grasp solution";
      return false;
    }
    return true;
  };

  std::string err;
  if (!solveOne(dual_arm::Arm::Left, pre.left, q_grasp.head<6>(), &out.q_left_pregrasp, &err) ||
      !solveOne(dual_arm::Arm::Right, pre.right, q_grasp.tail<6>(), &out.q_right_pregrasp, &err)) {
    out.code = "pregrasp_ik_failed";
    out.detail = err;
    return out;
  }

  dual_arm::Q12 q_back = dual_arm::Q12::Zero();
  q_back.head<6>() = out.q_left_pregrasp;
  q_back.tail<6>() = out.q_right_pregrasp;

  out.approach_segment = approachSegment(q_back, q_grasp, task_world);
  if (!out.approach_segment.ok) {
    out.code = "approach_segment_" + out.approach_segment.code;
    out.detail = out.approach_segment.detail;
    return out;
  }

  out.ok = true;
  out.code = "ok";
  out.detail = "standby->grasp clearance " + num(out.standby_segment.min_clearance_m) +
               " m; approach clearance " + num(out.approach_segment.min_clearance_m) +
               " m; approach line deviation " + num(out.approach_segment.max_line_dev_m) + " m";
  return out;
}

std::string SegmentResult::to_string() const {
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << (ok ? "OK" : "FAIL") << " code=" << code << " samples=" << n_samples
     << " min_clearance=" << min_clearance_m << " line_dev=" << max_line_dev_m;
  if (first_bad_sample >= 0) os << " first_bad=" << first_bad_sample;
  if (!detail.empty()) os << " (" << detail << ")";
  return os.str();
}

}  // namespace dual_arm_transport
