// src/collision_backend.cpp
#include "dual_arm_pose_optimization/collision_backend.hpp"

#include <algorithm>
#include <sstream>

#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "dual_arm_collision/dual_arm_collision.hpp"

namespace dual_arm_pose_optimization {

namespace {

constexpr const char* kDualArmGroup = "dual_arm";

using collision_detection::AllowedCollisionMatrix;
using AcType = collision_detection::AllowedCollision::Type;
using collision_detection::DistanceRequest;
using collision_detection::DistanceResult;

/// The single distance query used everywhere: ACM is mandatory.
double queryDistance(const collision_detection::CollisionEnvConstPtr& env,
                     const moveit::core::RobotState& state,
                     const AllowedCollisionMatrix& acm, DistanceResult* out,
                     bool self) {
  DistanceRequest req;
  req.enable_nearest_points = true;
  req.acm = &acm;
  DistanceResult res;
  if (self) {
    env->distanceSelf(req, res, state);
  } else {
    env->distanceRobot(req, res, state);
  }
  if (out != nullptr) *out = res;
  return res.minimum_distance.distance;
}

}  // namespace

std::string DistanceReading::to_string() const {
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(5);
  os << "whole_robot=" << whole_robot << " m (" << whole_link_a << " <-> "
     << whole_link_b << ")";
  os << "  arm_arm=" << arm_arm << " m (" << arm_arm_link_a << " <-> "
     << arm_arm_link_b << ")";
  if (capsule_arm_arm >= 0.0) os << "  capsule_arm_arm=" << capsule_arm_arm << " m";
  os << "  colliding=" << (in_collision ? "true" : "false");
  return os.str();
}

MoveItCollisionBackend::MoveItCollisionBackend(const std::shared_ptr<DualArmScene>& scene,
                                               const EnvironmentOptions& env)
    : scene_(scene), env_(env) {
  if (!scene_) throw SceneLoadError("MoveItCollisionBackend: null scene");

  const planning_scene::PlanningScenePtr& ps = scene_->planningScene();
  srdf_acm_ = ps->getAllowedCollisionMatrix();
  robot_world_acm_ = srdf_acm_;
  robot_world_acm_no_exemption_ = srdf_acm_;

  // ---- environment ------------------------------------------------------
  if (env_.add_floor) {
    moveit_msgs::msg::CollisionObject co;
    co.id = env_.floor_id;
    co.header.frame_id = scene_->robotModel()->getModelFrame();
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {env_.floor_size_xy, env_.floor_size_xy, env_.floor_thickness};
    geometry_msgs::msg::Pose p;
    p.position.x = 0.0;
    p.position.y = 0.0;
    // top surface at floor_top_z, extending downward
    p.position.z = env_.floor_top_z - 0.5 * env_.floor_thickness;
    p.orientation.w = 1.0;
    co.primitives.push_back(box);
    co.primitive_poses.push_back(p);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    ps->processCollisionObjectMsg(co);
    floor_added_ = true;

    // trap 4: the arms are bolted to the floor, so base_link touching it is not
    // a collision.  Exempt explicitly instead of relying on FCL contact
    // semantics.
    for (const std::string& link : env_.floor_exempt_links) {
      if (scene_->robotModel()->getLinkModel(link) == nullptr) continue;
      robot_world_acm_.setEntry(link, env_.floor_id, true);
    }
  }

  cross_arm_acm_ = buildCrossArmAcm();
}

AllowedCollisionMatrix MoveItCollisionBackend::buildCrossArmAcm() const {
  AllowedCollisionMatrix acm = srdf_acm_;

  // Definition: the cross-arm scope contains EXACTLY the (left, right) link
  // pairs.  Everything else -- same-arm pairs, and any link that belongs to
  // neither arm (world, grasp frames, and notably wrist_camera_link, which is
  // mounted on the RIGHT wrist but is not named right_*) -- is disabled.
  //
  // Deriving the scope from pair membership rather than from a name prefix is
  // deliberate: prefix matching silently leaked the wrist camera into the
  // "cross-arm" minimum and made arm_arm == whole_robot.
  const std::vector<std::string>& all = scene_->robotModel()->getLinkModelNames();
  const std::vector<std::string>& left = scene_->leftLinks();
  const std::vector<std::string>& right = scene_->rightLinks();

  auto inSet = [](const std::vector<std::string>& v, const std::string& n) {
    return std::find(v.begin(), v.end(), n) != v.end();
  };

  std::size_t checked = 0;
  for (std::size_t i = 0; i < all.size(); ++i) {
    const bool a_left = inSet(left, all[i]);
    const bool a_right = inSet(right, all[i]);
    for (std::size_t j = i + 1; j < all.size(); ++j) {
      const bool b_left = inSet(left, all[j]);
      const bool b_right = inSet(right, all[j]);
      const bool cross = (a_left && b_right) || (a_right && b_left);
      // allowed == true means "ignore this pair"
      acm.setEntry(all[i], all[j], !cross);
      if (cross) ++checked;
    }
  }
  cross_arm_pairs_ = checked;
  return acm;
}

bool MoveItCollisionBackend::isStateColliding(const moveit::core::RobotState& state) const {
  return scene_->planningScene()->isStateColliding(state, kDualArmGroup);
}

bool MoveItCollisionBackend::isStateCollidingWholeRobot(
    const moveit::core::RobotState& state) const {
  return scene_->planningScene()->isStateColliding(state, "");
}

bool MoveItCollisionBackend::isSegmentValid(const moveit::core::RobotState& a,
                                            const moveit::core::RobotState& b,
                                            int samples) const {
  if (samples < 2) samples = 2;
  const moveit::core::RobotModelConstPtr& model = scene_->robotModel();
  moveit::core::RobotState probe(model);
  for (int i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    a.interpolate(b, t, probe);   // NOTE: const method, writes into `probe`
    probe.update();
    if (isStateColliding(probe)) return false;
  }
  return true;
}

double MoveItCollisionBackend::robotWorldDistance(
    const moveit::core::RobotState& state) const {
  return robotWorldDistanceWith(state, robot_world_acm_);
}

double MoveItCollisionBackend::robotWorldDistanceWith(
    const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm) const {
  return queryDistance(scene_->planningScene()->getCollisionEnv(), state, acm, nullptr,
                       /*self=*/false);
}

DistanceReading MoveItCollisionBackend::distances(const moveit::core::RobotState& state,
                                                  const dual_arm::Q12* q12) const {
  DistanceReading out;
  const collision_detection::CollisionEnvConstPtr& env =
      scene_->planningScene()->getCollisionEnv();

  out.in_collision = isStateColliding(state);

  // scope 1: whole robot (SRDF ACM)
  DistanceResult whole;
  out.whole_robot = queryDistance(env, state, srdf_acm_, &whole, /*self=*/true);
  out.whole_link_a = whole.minimum_distance.link_names[0];
  out.whole_link_b = whole.minimum_distance.link_names[1];

  // scope 2: cross-arm only
  DistanceResult arm;
  out.arm_arm = queryDistance(env, state, cross_arm_acm_, &arm, /*self=*/true);
  out.arm_arm_link_a = arm.minimum_distance.link_names[0];
  out.arm_arm_link_b = arm.minimum_distance.link_names[1];
  out.arm_arm_point_a = arm.minimum_distance.nearest_points[0];
  out.arm_arm_point_b = arm.minimum_distance.nearest_points[1];

  // scope 3: independent capsule proxy (C2.7)
  if (q12 != nullptr) {
    if (!kin_) kin_ = std::make_unique<dual_arm::DualArmKinematics>();
    if (!capsule_) {
      capsule_ = std::make_unique<dual_arm_collision::DualArmCollisionModel>(*kin_, 0.06);
    }
    out.capsule_arm_arm = capsule_->distance(*q12).distance;
  }

  return out;
}

}  // namespace dual_arm_pose_optimization
