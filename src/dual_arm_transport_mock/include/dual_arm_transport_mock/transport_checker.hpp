// C4.3 S3 -- W5: per-cycle state check + swept check, with the sample placed at
// three independent object-pose sources (§3.3 step 11, S3-T9 / S3-T19 / S3-T20).
//
// Two layers, deliberately separate:
//   * robot self-collision / clearance  -- MoveItCollisionBackend::distances()
//     (SRDF ACM + cross-arm + arm-proximal scopes, C4.2's three-trap fixes)
//   * robot vs WORLD                    -- the sample, the support, the floor.
//     C4.3 S2's PairScreener only ever asked for `distances()` (self-only), so the
//     world layer is genuinely new here and S3-T9 is what proves it is live.
//
// The intended grasp contact must NOT count as a collision: the sample is held by
// the fingers.  That is expressed as an explicit ACM exemption (sample <-> finger
// links only), never as a blanket "ignore the sample" -- otherwise a wrist
// through the sample would pass.
#ifndef DUAL_ARM_TRANSPORT_MOCK__TRANSPORT_CHECKER_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__TRANSPORT_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <moveit/collision_detection/collision_matrix.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"

namespace dual_arm_transport_mock {

/// The three object-pose sources of §8.  A cycle is only "checked" when all three
/// have been evaluated (S3-T20); the selected one decides commit/abort.
inline constexpr const char* kObjectSourceLeftAnchor = "left_anchor";
inline constexpr const char* kObjectSourceRightAnchor = "right_anchor";
inline constexpr const char* kObjectSourceDesired = "desired";

struct CheckOptions {
    double min_clearance_m = 0.002;   ///< same value S2 used
    /// Finger stroke stands off `grip_thickness/2` by this much.  MUST equal
    /// dual_arm_transport::PairScreenerOptions::pad_clearance_m (1e-4): at exactly
    /// 0.0 the pads sit ON the sample surface and FCL reports the intended grasp
    /// contact as a penetration, which would make every grasp "colliding".
    double pad_clearance_m = 1e-4;
    int sweep_samples = 16;
    bool swept_check_enabled = true;
    std::string object_source = kObjectSourceLeftAnchor;
    /// Links allowed to touch the grasped sample (the pad contact).
    std::vector<std::string> grasp_contact_links{
        "left_finger1", "left_finger2", "right_finger1", "right_finger2"};
};

struct StateCheck {
    bool ok = false;              ///< !in_collision && min_clearance >= min_clearance_m
    /// Either layer reports contact/penetration.  Derived from the two distance
    /// queries below, NOT from PlanningScene::isStateColliding -- that call mixes
    /// self and world and uses the planning-scene ACM instead of world_acm_.
    bool in_collision = false;
    std::string code;             ///< "ok" | "collision" | "insufficient_clearance"
    std::string detail;
    double whole_robot = 0.0;     ///< [m] min over every robot link pair
    double arm_arm = 0.0;         ///< [m] cross-arm only
    double arm_proximal = 0.0;    ///< [m] cross-arm with >= 1 proximal link
    double capsule_arm_arm = 0.0; ///< [m] C2.7 capsule proxy (independent reading)
    double robot_world = 0.0;     ///< [m] robot vs sample/support/floor; <= 0 = contact
    double min_clearance = 0.0;   ///< [m] min(whole_robot, robot_world)
    std::string link_a;
    std::string link_b;
    std::string object_source;    ///< which painted-object pose produced this result
};

struct SweepCheck {
    bool ok = false;
    std::string code;             ///< "ok" | "swept_collision" | "insufficient_clearance"
    std::string detail;
    int first_bad_sample = -1;
    double min_clearance_m = 0.0;
    /// The WORLD layer alone (robot vs sample/support/floor).  Reported separately
    /// because min_clearance_m is usually dominated by a constant self-collision pair
    /// and therefore cannot show that the painted world changed at all (S3-T22).
    double min_robot_world_m = -1.0;
    std::string object_source;
};

class TransportChecker {
public:
    TransportChecker(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                     const dual_arm_transport::GraspConfig& cfg,
                     const CheckOptions& opt = CheckOptions{});

    /// Insert/refresh the sample at `world_T_object` (and the candidate support).
    /// `source` tags the result so S3-T20 can prove which pose was checked.
    void applyWorld(const dual_arm::Iso3& world_T_object, const std::string& source = "");
    void clearObjects();

    /// Fingers closed on the sample, then self + world distance queries.
    StateCheck checkState(const dual_arm::Q12& q) const;

    /// Discretised straight-line joint interpolation q0 -> q1, each sample checked
    /// with the full two-layer check.  Returns the first offending sample.
    SweepCheck checkSweep(const dual_arm::Q12& q0, const dual_arm::Q12& q1) const;

    /// The same segment, but with the sample CARRIED ALONG the grasp (S3-T22).
    ///
    /// WHY THIS EXISTS (review).  checkSweep() interpolates the arm configurations and
    /// tests the sample at ONE FIXED world pose -- the pose it had when applyWorld()
    /// was last called.  That models an object that stays put while the arms move, so
    /// it is evidence for "the arms can travel between these two configurations with
    /// the object where it started", NOT for "the object is collision-free along the
    /// way".  In a real transport the object is held: at sample a its pose is the
    /// anchored grasp pose of q(a), i.e. a different world for every sample.
    ///
    /// `object_T_tcp` is the frozen grasp G_arm and `anchor` the arm it is anchored to
    /// (Left for left_anchor, Right for right_anchor), so the check tests exactly the
    /// pose the adopted source claims.  There is no general form for `desired`: that
    /// source is a prescription, not a measurable pose, so callers must not pretend it
    /// tracks a configuration.
    ///
    /// Non-const: every sample re-paints the collision world.
    SweepCheck checkSweepTracking(const dual_arm::Q12& q0, const dual_arm::Q12& q1,
                                  const dual_arm::Iso3& object_T_tcp, dual_arm::Arm anchor);

    const CheckOptions& options() const { return opt_; }
    const std::string& currentObjectSource() const { return object_source_; }
    const dual_arm::Iso3& currentWorldObject() const { return world_T_object_; }
    const dual_arm_pose_optimization::MoveItCollisionBackend& backend() const { return *backend_; }

private:
    moveit::core::RobotState graspState(const dual_arm::Q12& q) const;

    std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene_;
    dual_arm_transport::GraspConfig cfg_;
    CheckOptions opt_;
    std::unique_ptr<dual_arm_pose_optimization::MoveItCollisionBackend> backend_;
    /// backend_->robotWorldAcm() + sample <-> grasp_contact_links.
    collision_detection::AllowedCollisionMatrix world_acm_;
    dual_arm::Iso3 world_T_object_ = dual_arm::Iso3::Identity();
    std::string object_source_;
    bool world_applied_ = false;
};

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__TRANSPORT_CHECKER_HPP_
