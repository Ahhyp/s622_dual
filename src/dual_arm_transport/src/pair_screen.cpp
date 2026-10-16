// src/pair_screen.cpp -- W3/W4: cartesian-product screening with attribution.
#include "dual_arm_transport/pair_screen.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/msg/pose.h>
#include <moveit_msgs/msg/collision_object.h>
#include <shape_msgs/msg/solid_primitive.h>

#include "dual_arm_kinematics/closure_residual.hpp"
#include "dual_arm_kinematics/object_frame.hpp"

namespace dual_arm_transport {
namespace {

using dual_arm::Iso3;

geometry_msgs::msg::Pose toPose(const Iso3& T) {
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond q(T.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

}  // namespace

PairScreener::PairScreener(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                           const GraspConfig& cfg, const ScreenOptions& opt)
    : scene_(std::move(scene)), cfg_(cfg), opt_(opt) {
  if (scene_ == nullptr) throw GraspConfigError("PairScreener: null scene");
  dual_arm_pose_optimization::EnvironmentOptions env;
  env.add_floor = true;
  backend_ = std::make_unique<dual_arm_pose_optimization::MoveItCollisionBackend>(scene_, env);
}

void PairScreener::clearObjects() {
  for (const std::string& id : {cfg_.support.id, std::string("c43_sample")}) {
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.header.frame_id = scene_->robotModel()->getModelFrame();
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene_->planningScene()->processCollisionObjectMsg(co);
  }
}

void PairScreener::applyWorld(const Iso3& world_T_object) {
  world_T_object_ = world_T_object;
  world_applied_ = true;

  // ---- the sample, as a box exactly as configured ----
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = "c43_sample";
    co.header.frame_id = scene_->robotModel()->getModelFrame();
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {cfg_.grasp.size_xyz.x(), cfg_.grasp.size_xyz.y(), cfg_.grasp.size_xyz.z()};
    co.primitives.push_back(box);
    co.primitive_poses.push_back(toPose(world_T_object));
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene_->planningScene()->processCollisionObjectMsg(co);
  }

  // ---- the candidate support, directly under the sample ----
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = cfg_.support.id;
    co.header.frame_id = scene_->robotModel()->getModelFrame();
    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = {cfg_.support.height_m, cfg_.support.radius_m};
    co.primitives.push_back(cyl);
    geometry_msgs::msg::Pose p;
    p.position.x = world_T_object.translation().x();
    p.position.y = world_T_object.translation().y();
    p.position.z = cfg_.support.height_m / 2.0;  // candidate top at height_m
    p.orientation.w = 1.0;
    co.primitive_poses.push_back(p);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene_->planningScene()->processCollisionObjectMsg(co);
  }
}

moveit::core::RobotState PairScreener::graspState(const dual_arm::Q12& q12) const {
  moveit::core::RobotState st =
      scene_->makeStateQ12(std::vector<double>(q12.data(), q12.data() + 12));
  const std::pair<double, double> l = fingerStrokes(cfg_, opt_.pad_clearance_m);
  st.setVariablePosition("left_finger1_joint", l.first);
  st.setVariablePosition("left_finger2_joint", l.second);
  st.setVariablePosition("right_finger1_joint", l.first);
  st.setVariablePosition("right_finger2_joint", l.second);
  st.update();
  return st;
}

Iso3 PairScreener::anchoredObject(const dual_arm::JointConfig& q, const std::string& arm) const {
  const dual_arm::Arm a = (arm == "left") ? dual_arm::Arm::Left : dual_arm::Arm::Right;
  const Iso3 world_T_tcp = kin_.worldTgrasp(a, q);
  const Iso3& object_T_tcp =
      (arm == "left") ? cfg_.grasp.object_T_left_tcp : cfg_.grasp.object_T_right_tcp;
  return dual_arm::anchoredObjectPose(world_T_tcp, object_T_tcp);
}

ScreenReport PairScreener::screen(const BranchSet& left, const BranchSet& right) {
  ScreenReport rep;
  const Iso3& T_task = world_applied_ ? world_T_object_ : cfg_.grasp.world_T_object_star;

  const auto poseGap = [](const Iso3& a, const Iso3& b, double* pos, double* rot) {
    *pos = (a.translation() - b.translation()).norm();
    Iso3 e = Iso3::Identity();
    e.linear() = a.linear().transpose() * b.linear();
    *rot = dual_arm::logSE3(e).tail<3>().norm();
  };

  for (std::size_t i = 0; i < left.solutions.size(); ++i) {
    for (std::size_t j = 0; j < right.solutions.size(); ++j) {
      ++rep.n_pairs;
      PairCandidate c;
      c.left_index = static_cast<int>(i);
      c.right_index = static_cast<int>(j);
      c.q_left = left.solutions[i];
      c.q_right = right.solutions[j];
      c.q12.head<6>() = c.q_left;
      c.q12.tail<6>() = c.q_right;

      // ---- independent anchors (S2-T8) ----
      c.object_from_left = anchoredObject(c.q_left, "left");
      c.object_from_right = anchoredObject(c.q_right, "right");
      poseGap(c.object_from_left, c.object_from_right, &c.anchor_pos_gap_m, &c.anchor_rot_gap_rad);

      c.object_midpoint = kin_.worldTobject(c.q12);
      poseGap(c.object_midpoint, T_task, &c.midpoint_vs_task_pos_m, &c.midpoint_vs_task_rot_rad);

      // ---- closure residual (S2-T10) ----
      const dual_arm::ClosureResidual cr = dual_arm::closureResidual(
          c.object_from_left, c.object_from_right);
      c.closure_pos_m = cr.translation_m;
      c.closure_rot_rad = cr.rotation_rad;
      c.closure_dir_ambiguous = cr.rotation_direction_ambiguous;

      // ---- conditioning ----
      {
        const Eigen::Matrix<double, 6, 12> J = kin_.objectJacobianBody(c.q12);
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 12>> svd(J);
        c.sigma_min = svd.singularValues().minCoeff();
      }

      // ---- collision ----
      const moveit::core::RobotState st = graspState(c.q12);
      const auto reading = backend_->distances(st, &c.q12);
      c.min_clearance_m = reading.whole_robot;
      c.collides = reading.in_collision;
      if (!c.collides) {
        c.collision_kind = "none";
      } else if (reading.arm_arm <= 0.0 || reading.arm_proximal <= 0.0) {
        c.collision_kind = (reading.arm_arm <= 0.0) ? "arm_arm" : "arm_proximal";
        c.collision_detail = reading.arm_arm_link_a + "<->" + reading.arm_arm_link_b;
        if (reading.arm_proximal <= 0.0) {
          c.collision_detail += " / " + reading.arm_proximal_link_a + "<->" +
                                reading.arm_proximal_link_b;
        }
      } else {
        // Neither arm-to-arm nor arm-to-proximal is touching, yet the pair
        // collides.  Attribute it correctly (S2-T13): two links of the SAME arm
        // are a within-arm SELF collision, not a collision with the world.  Only
        // a pair spanning the object/support/floor is a world collision.
        auto on_arm = [](const std::string& link, const char* prefix) {
          return link.rfind(prefix, 0) == 0;
        };
        const bool same_arm = (on_arm(reading.whole_link_a, "left_") &&
                               on_arm(reading.whole_link_b, "left_")) ||
                              (on_arm(reading.whole_link_a, "right_") &&
                               on_arm(reading.whole_link_b, "right_"));
        c.collision_kind = same_arm ? "self" : "world";
        c.collision_detail = reading.whole_link_a + "<->" + reading.whole_link_b;
      }

      // ---- verdict, one code per rejection (S2-T12/T13) ----
      if (!left.ok || !right.ok) {
        c.reject_code = "ik_branch_unavailable";
        c.reject_detail = "one arm produced no verified branch";
      } else if (c.anchor_pos_gap_m > opt_.anchor_pos_tol_m ||
                 c.anchor_rot_gap_rad > opt_.anchor_rot_tol_rad) {
        c.reject_code = "independent_anchor_disagreement";
        std::ostringstream os;
        os << "anchor gap " << c.anchor_pos_gap_m << " m / " << c.anchor_rot_gap_rad
           << " rad exceeds tolerance";
        c.reject_detail = os.str();
      } else if (c.closure_pos_m > opt_.closure_pos_tol_m ||
                 c.closure_rot_rad > opt_.closure_rot_tol_rad) {
        c.reject_code = "closure_residual";
        c.reject_detail = "frozen grasp constraint violated";
      } else if (c.sigma_min < opt_.min_sigma) {
        c.reject_code = "singular_configuration";
        c.reject_detail = "sigma_min below threshold";
      } else if (c.collides) {
        c.reject_code = "collision_" + c.collision_kind;
        c.reject_detail = c.collision_detail;
      } else if (c.min_clearance_m < opt_.min_clearance_m) {
        c.reject_code = "insufficient_clearance";
        std::ostringstream os;
        os << "clearance " << c.min_clearance_m << " m < " << opt_.min_clearance_m << " m";
        c.reject_detail = os.str();
      } else {
        c.feasible = true;
      }

      if (!c.feasible) rep.reject_histogram[c.reject_code] += 1;
      else ++rep.n_feasible;
      if (opt_.keep_all || c.feasible) rep.candidates.push_back(c);
    }
  }
  return rep;
}

std::string ScreenReport::to_string(std::size_t max_rows) const {
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << "screen: pairs=" << n_pairs << " feasible=" << n_feasible << "\n";
  for (const auto& kv : reject_histogram) {
    os << "  reject " << kv.first << ": " << kv.second << "\n";
  }
  const std::size_t rows = std::min(max_rows, candidates.size());
  for (std::size_t k = 0; k < rows; ++k) {
    const PairCandidate& c = candidates[k];
    os << "  [" << c.left_index << "," << c.right_index << "] "
       << (c.feasible ? "FEASIBLE" : "reject:" + c.reject_code) << "  clearance=" << c.min_clearance_m
       << " sigma_min=" << c.sigma_min << " anchor_gap=" << c.anchor_pos_gap_m
       << " midpoint_err=" << c.midpoint_vs_task_pos_m;
    if (!c.reject_detail.empty()) os << "  (" << c.reject_detail << ")";
    os << "\n";
  }
  if (candidates.size() > rows) os << "  ... " << (candidates.size() - rows) << " more\n";
  return os.str();
}

}  // namespace dual_arm_transport
