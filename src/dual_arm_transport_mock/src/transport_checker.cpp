// C4.3 S3 -- W5 implementation.  See transport_checker.hpp for the contract.
#include "dual_arm_transport_mock/transport_checker.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <geometry_msgs/msg/pose.h>
#include <moveit_msgs/msg/collision_object.h>
#include <shape_msgs/msg/solid_primitive.h>

#include "dual_arm_transport/ik_enum.hpp"

namespace dual_arm_transport_mock {

namespace {

using dual_arm::Iso3;

constexpr const char* kSampleId = "c43_sample";

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

TransportChecker::TransportChecker(
    std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
    const dual_arm_transport::GraspConfig& cfg, const CheckOptions& opt)
    : scene_(std::move(scene)), cfg_(cfg), opt_(opt) {
    if (scene_ == nullptr) throw dual_arm_transport::GraspConfigError("TransportChecker: null scene");
    if (opt_.sweep_samples < 2) throw std::invalid_argument("TransportChecker: sweep_samples must be >= 2");
    dual_arm_pose_optimization::EnvironmentOptions env;
    env.add_floor = true;
    backend_ = std::make_unique<dual_arm_pose_optimization::MoveItCollisionBackend>(scene_, env);

    // Start from the backend's robot-world ACM (floor exempted for the bases) and
    // add ONLY the intended pad contact.  Everything else stays enabled, so a
    // wrist swinging through the sample is still a collision.
    world_acm_ = backend_->robotWorldAcm();
    for (const std::string& link : opt_.grasp_contact_links) {
        world_acm_.setEntry(kSampleId, link, true);
    }
    object_source_ = opt_.object_source;
}

void TransportChecker::clearObjects() {
    for (const std::string& id : {cfg_.support.id, std::string(kSampleId)}) {
        moveit_msgs::msg::CollisionObject co;
        co.id = id;
        co.header.frame_id = scene_->robotModel()->getModelFrame();
        co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        scene_->planningScene()->processCollisionObjectMsg(co);
    }
    world_applied_ = false;
}

void TransportChecker::applyWorld(const Iso3& world_T_object, const std::string& source) {
    world_T_object_ = world_T_object;
    world_applied_ = true;
    object_source_ = source.empty() ? opt_.object_source : source;

    // ---- the sample, as a box exactly as configured (same as S2) ----
    {
        moveit_msgs::msg::CollisionObject co;
        co.id = kSampleId;
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
        p.position.z = cfg_.support.height_m / 2.0;
        p.orientation.w = 1.0;
        co.primitive_poses.push_back(p);
        co.operation = moveit_msgs::msg::CollisionObject::ADD;
        scene_->planningScene()->processCollisionObjectMsg(co);
    }
}

moveit::core::RobotState TransportChecker::graspState(const dual_arm::Q12& q) const {
    moveit::core::RobotState st =
        scene_->makeStateQ12(std::vector<double>(q.data(), q.data() + 12));
    const std::pair<double, double> stroke =
        dual_arm_transport::fingerStrokes(cfg_, opt_.pad_clearance_m);
    st.setVariablePosition("left_finger1_joint", stroke.first);
    st.setVariablePosition("left_finger2_joint", stroke.second);
    st.setVariablePosition("right_finger1_joint", stroke.first);
    st.setVariablePosition("right_finger2_joint", stroke.second);
    st.update();
    return st;
}

StateCheck TransportChecker::checkState(const dual_arm::Q12& q) const {
    StateCheck out;
    out.object_source = object_source_;

    if (!q.allFinite()) {
        out.ok = false;
        out.in_collision = true;
        out.code = "collision";
        out.detail = "non-finite q";
        return out;
    }

    const moveit::core::RobotState st = graspState(q);

    // layer 1: robot self-collision + the three clearance scopes
    const auto reading = backend_->distances(st, &q);
    out.whole_robot = reading.whole_robot;
    out.arm_arm = reading.arm_arm;
    out.arm_proximal = reading.arm_proximal;
    out.capsule_arm_arm = reading.capsule_arm_arm;
    out.link_a = reading.whole_link_a;
    out.link_b = reading.whole_link_b;

    // layer 2: robot vs world (sample / support / floor), pad contact exempted
    out.robot_world = backend_->robotWorldDistanceWith(st, world_acm_);

    // `reading.in_collision` (PlanningScene::isStateColliding) is deliberately NOT
    // used: it folds self and world together and consults the planning-scene ACM,
    // so the intended pad contact on the sample would make it permanently true.
    // Layer separation is what lets §9 name the offending layer.
    const bool self_contact = reading.whole_robot <= 0.0;
    const bool world_contact = world_applied_ && out.robot_world <= 0.0;
    out.in_collision = self_contact || world_contact;
    out.min_clearance = std::min(out.whole_robot, out.robot_world);

    if (self_contact) {
        out.code = "collision";
        out.detail = "self-collision " + reading.whole_link_a + "<->" + reading.whole_link_b;
        out.ok = false;
        return out;
    }
    if (world_contact) {
        out.code = "collision";
        out.detail = "robot-world contact";
        out.ok = false;
        return out;
    }
    if (out.min_clearance < opt_.min_clearance_m) {
        std::ostringstream os;
        os << "clearance " << out.min_clearance << " m < " << opt_.min_clearance_m
           << " m (whole=" << out.whole_robot << ", world=" << out.robot_world << ")";
        out.code = "insufficient_clearance";
        out.detail = os.str();
        out.ok = false;
        return out;
    }
    out.code = "ok";
    out.ok = true;
    return out;
}

SweepCheck TransportChecker::checkSweep(const dual_arm::Q12& q0, const dual_arm::Q12& q1) const {
    SweepCheck out;
    out.object_source = object_source_;
    out.min_clearance_m = -1.0;

    if (!opt_.swept_check_enabled) {
        out.ok = true;
        out.code = "ok";
        out.detail = "swept check disabled";
        return out;
    }
    if (!q0.allFinite() || !q1.allFinite()) {
        out.ok = false;
        out.code = "swept_collision";
        out.detail = "non-finite endpoint";
        out.first_bad_sample = 0;
        return out;
    }

    const int n = opt_.sweep_samples;
    for (int i = 0; i < n; ++i) {
        const double a = static_cast<double>(i) / static_cast<double>(n - 1);
        const dual_arm::Q12 q = q0 + a * (q1 - q0);
        const StateCheck sc = checkState(q);
        if (out.min_clearance_m < 0.0 || sc.min_clearance < out.min_clearance_m) {
            out.min_clearance_m = sc.min_clearance;
        }
        if (!sc.ok) {
            out.ok = false;
            out.code = (sc.code == "collision") ? "swept_collision" : "insufficient_clearance";
            out.detail = "sample " + std::to_string(i) + "/" + std::to_string(n) + ": " + sc.detail;
            out.first_bad_sample = i;
            return out;
        }
    }
    out.ok = true;
    out.code = "ok";
    return out;
}

}  // namespace dual_arm_transport_mock
