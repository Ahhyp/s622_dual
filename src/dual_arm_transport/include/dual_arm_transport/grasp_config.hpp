// include/dual_arm_transport/grasp_config.hpp
// C4.3 S2 / W1 -- the strict grasp/sample/support configuration.
//
// FRAME DISCIPLINE (user m02331 D2)
// ---------------------------------
// Every grasp-region quantity below is expressed in the OBJECT LOCAL frame
// (`dual_arm::GraspGeometry::object_frame`) and must be INVARIANT under any rigid
// transform of the object.  Only `world_T_object_star` is a world-frame quantity.
// Writing a grasp region as a world-frame constant is forbidden: with the two
// arms mounted facing each other, a world-frame constant is wrong for every
// object orientation except the one it was measured at.
//
// 0.061 m IS A STROKE, NOT A PROOF (user m02331 D1)
// -------------------------------------------------
// `gripper_max_opening_m` bounds the CLOSING STROKE.  It says nothing about the
// gripper BODY envelope, so it is a necessary and NOT a sufficient condition for
// a dual-end grasp.  `GraspConfig::validate()` therefore never treats
// `grip_thickness_m <= gripper_max_opening_m` as acceptance; the envelope check
// is a collision query (pair_screen.cpp), and gripperGeometry() below measures
// the real finger/wrist extents out of the loaded robot model instead of
// assuming them.
#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/grasp_geometry.hpp"

// Only a reference parameter below; the .cpp pulls in the full definition.
namespace dual_arm_pose_optimization {
class DualArmScene;
}  // namespace dual_arm_pose_optimization

namespace dual_arm_transport {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// One grasp region, in the OBJECT LOCAL frame (D1/D2).
struct GraspRegion {
  std::string name;   ///< "left" | "right"
  /// Grasp-face polygon vertices, object local [m], in order (>= 4 vertices).
  std::vector<Vec3> polygon_local;
  /// Outward normal of the grasped END face: points from the object CENTRE out
  /// to the gripper.  It is a region marker and the machine-checked statement of
  /// WHICH end is grasped.  It is deliberately NOT the tool approach direction:
  /// the tool comes down onto the end from above, which is encoded in and
  /// enforced through `object_T_*_tcp` (see makeEndGraspTransform below).
  Vec3 approach_dir_local = Vec3::UnitX();
  /// The two finger/object contact segments (each a pair of endpoints), object
  /// local [m].  These are the edges the pads actually touch.
  std::vector<std::pair<Vec3, Vec3>> contact_faces;
  /// Distance from the grasped face to the object centre, object local [m].
  double grasp_depth_m = 0.0;
};

/// The sample geometry actually used.  `variant != "box"` requires an explicit,
/// recorded geometry change (user m02331 D1), never a silent edit.
struct SampleGeometry {
  std::string variant{"box"};          ///< "box" | "parameterized_rigid"
  std::string reason;                  ///< why this variant, empty for "box"
  std::string diff_vs_box;             ///< how it differs from the plain box
};

/// Finger/object interference numbers for ONE gripper (D1).
struct FingerObjectClearance {
  double intrusion_m = 0.0;  ///< [m] how far the finger envelope enters the object
  double margin_m = 0.0;     ///< [m] smallest finger-to-object separation (>= 0 good)
  std::string note;
};

/// Sample support.  `top_z_m` is a CANDIDATE until screening confirms it
/// (user m02331 D4); the status field is what the acceptance record reports.
struct SupportSpec {
  std::string id{"c43_support"};
  std::string shape{"cylinder"};
  double radius_m = 0.05;
  double height_m = 0.30;
  double top_z_m = 0.30;  ///< candidate top surface height [m]
  /// "candidate" | "screened" | "rejected".  Never silently promoted.
  std::string top_z_status{"candidate"};
};

/// The whole W1 input, loaded from `config/object_and_grasps.yaml`.
struct GraspConfig {
  /// Object body + the frozen grasp transforms G_L / G_R, from C4.3 S1.
  /// Carries `size_xyz`, `mass`, `inertia`, `world_T_object_star`,
  /// `object_T_left_tcp`, `object_T_right_tcp` and S1's own `validate()`.
  dual_arm::GraspGeometry grasp;

  SampleGeometry sample_geometry;
  std::string provenance{"provisional simulation sample"};
  std::vector<std::string> collision_links{"object_link"};

  GraspRegion left_region;
  GraspRegion right_region;

  /// Grip thickness = the object extent along the grasp axis [m].
  double grip_thickness_m = 0.0;
  FingerObjectClearance left_clearance;
  FingerObjectClearance right_clearance;

  SupportSpec support;
  double d_approach_m = 0.08;  ///< CANDIDATE pre-grasp retreat (user m02331 D7)

  /// Closing-stroke bound measured from the URDF (see C4.3_S2_实施方案.md §3.2).
  double gripper_max_opening_m = 0.061;
};

/// Empty string => valid, otherwise a human-readable reason.
///
/// Rejects: non-positive / non-finite sizes, non-orthonormal transforms, empty
/// or colliding frame ids, degenerate G_L == G_R, a grasp region whose polygon
/// is not planar / not on the declared face, an approach direction that is not
/// unit-norm, a `parameterized_rigid` variant without a recorded reason, and a
/// grip thickness above the stroke.  NEVER clamps or repairs.
std::string validateGraspConfig(const GraspConfig& cfg);

/// Strict YAML load.  Throws `GraspConfigError` on any problem, including any
/// UNKNOWN KEY (typo protection is part of the contract, not a nicety).
GraspConfig loadGraspConfig(const std::string& path);

/// Thrown by loadGraspConfig / dump helpers.
class GraspConfigError : public std::runtime_error {
public:
  explicit GraspConfigError(const std::string& what) : std::runtime_error(what) {}
};

/// Solid-box inertia about the object origin, from size and mass.
Mat3 boxInertia(const Vec3& size_xyz, double mass);

/// Object-local grasp frame builders (D2): place the TCP just above the centre
/// of the `+x` (or `-x`) END face and orient the tool so it comes DOWN onto the
/// end -- `-z_tcp = +z_obj`, and the pads close along `+y_obj`.
///
/// A horizontal approach along the rod axis was measured to be unreachable: this
/// arm carries its TCP 0.2168 m in front of the wrist, so reaching inward puts
/// each wrist centre ~0.073 m from its own base axis and the analytic solver
/// fails at the grasp itself (`rho_sq`).  `tcp_offset_along_normal_m` shifts the
/// TCP further along +z (0 by default).
dual_arm::Iso3 makeEndGraspTransform(const Vec3& size_xyz, int sign,
                                     double tcp_offset_along_normal_m = 0.0);

// ---------------------------------------------------------------------------
// Measured gripper geometry (D1).  Nothing here is assumed: every number is
// read back out of the loaded MoveIt model.
// ---------------------------------------------------------------------------

/// Where one gripper's pads and body actually are.
struct GripperMeasurement {
  std::string prefix;                       ///< "left" | "right"
  std::string wrist_link;
  std::string tcp_frame;
  /// Bounding box, in the TCP frame, of the two finger links' collision shapes,
  /// at the CLOSED configuration (both prismatic joints at 0).
  Vec3 fingers_min_tcp = Vec3::Zero();
  Vec3 fingers_max_tcp = Vec3::Zero();
  /// Bounding box, in the TCP frame, of the wrist3 link's collision shapes.
  Vec3 wrist_min_tcp = Vec3::Zero();
  Vec3 wrist_max_tcp = Vec3::Zero();
  std::size_t n_finger_shapes = 0;
  std::size_t n_wrist_shapes = 0;
  /// Gap between the two pad faces at the closed configuration [m].
  double closed_pad_gap_m = 0.0;
  /// Gap at the fully open configuration [m].
  double open_pad_gap_m = 0.0;
  /// TCP-frame z of the finger pad region's near/far edge [m].
  double pad_z_near_m = 0.0;
  double pad_z_far_m = 0.0;
  std::string to_string() const;
};

/// Measure both grippers from a loaded scene.  Throws GraspConfigError when a
/// link has no collision geometry (that would make every later query silently
/// answer "no collision").
GripperMeasurement measureGripper(const dual_arm_pose_optimization::DualArmScene& scene,
                                  const std::string& prefix);

}  // namespace dual_arm_transport
