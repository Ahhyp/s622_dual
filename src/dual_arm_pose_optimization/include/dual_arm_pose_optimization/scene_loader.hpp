// include/dual_arm_pose_optimization/scene_loader.hpp
// C4.2 / S1: build a MoveIt PlanningScene OFFLINE (no ROS node) from URDF+SRDF
// using the real STL collision meshes, and PROVE that they loaded.
//
// Why this exists
// ---------------
// The C2.7 collision model is a capsule proxy.  C4.2 needs the real collision
// geometry (extra requirement 4: "地面碰撞不能只靠 DH 坐标系原点的高度判断").
// MoveIt gives us that for free -- but only if the meshes resolve:
// a missing ament index entry makes RobotModel silently DROP the collision
// geometry of every affected link, and then every collision query returns
// "false" with no error.  That failure mode is invisible unless it is asserted
// against, so this loader does two independent checks:
//
//   Check A (pre-flight, this code):  every  package://  mesh URI in the URDF
//                                     resolves to an existing, non-empty file
//   Check B (post-load):              the number of collision shapes actually
//                                     loaded into the RobotModel equals the
//                                     number of <collision> elements the URDF
//                                     declares, and mesh vertex counts are > 0
//
// Either check failing raises SceneLoadError.  Nothing here ever "returns
// anyway with an empty scene".

#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace dual_arm_pose_optimization {

/// Thrown for any unrecoverable scene-construction problem.  The message always
/// names the offending file / URI so a failure is actionable.
class SceneLoadError : public std::runtime_error {
public:
  explicit SceneLoadError(const std::string& what) : std::runtime_error(what) {}
};

struct SceneLoadOptions {
  std::string urdf_path;
  std::string srdf_path;
  /// Check A: refuse to build a scene whose meshes cannot be resolved.
  bool require_meshes = true;
  /// Prefix used to classify links into arms ("left_"/"right_").
  std::string left_prefix = "left_";
  std::string right_prefix = "right_";
};

/// Per-link collision geometry actually present in the RobotModel.
struct LinkGeometry {
  std::string link_name;
  std::string arm;  ///< "left" | "right" | "other"
  std::size_t n_shapes = 0;
  std::vector<std::string> shape_types;  ///< "mesh" | "box" | "cylinder" | ...
  std::size_t mesh_vertices = 0;
  std::size_t mesh_triangles = 0;
};

/// Everything S1 is required to report as evidence.
struct GeometryReport {
  // ---- provenance -------------------------------------------------------
  std::string urdf_path;
  std::size_t urdf_bytes = 0;
  std::string srdf_path;
  std::size_t srdf_bytes = 0;
  std::string model_name;
  std::string model_frame;
  std::size_t urdf_links = 0;   ///< links declared in the URDF
  std::size_t urdf_joints = 0;
  std::size_t urdf_collision_elements = 0;  ///< <collision> blocks declared
  std::vector<std::string> joint_model_groups;
  std::size_t srdf_disable_collisions = 0;
  std::string collision_detector_name;

  // ---- Check A: mesh resource pre-flight --------------------------------
  std::size_t urdf_mesh_refs = 0;      ///< mesh references (deduplicated by URI count)
  std::size_t meshes_resolved = 0;
  std::vector<std::string> resolved_mesh_paths;  ///< unique absolute paths
  std::vector<std::string> failed_meshes;        ///< "uri -> reason"

  // ---- Check B: geometry actually loaded --------------------------------
  std::vector<LinkGeometry> links;  ///< only links with >= 1 collision shape
  std::size_t total_collision_shapes = 0;
  std::map<std::string, std::size_t> shapes_per_arm;
  std::map<std::string, std::size_t> shapes_per_type;
  std::size_t total_mesh_vertices = 0;
  std::size_t total_mesh_triangles = 0;
  std::vector<std::string> urdf_links_without_collision;

  /// Human-readable multi-line report (what the CLI prints).
  std::string to_string() const;

  /// Convenience accessors.
  std::size_t shapesOfArm(const std::string& arm) const;
  const LinkGeometry* link(const std::string& name) const;
};

class DualArmScene {
public:
  /// Loads and validates.  Throws SceneLoadError on any failure.
  static std::shared_ptr<DualArmScene> load(const SceneLoadOptions& options);

  const moveit::core::RobotModelConstPtr& robotModel() const { return robot_model_; }
  const planning_scene::PlanningScenePtr& planningScene() const { return scene_; }
  const GeometryReport& geometry() const { return geometry_; }

  /// Link names whose name starts with the given prefix, in RobotModel order.
  std::vector<std::string> linksWithPrefix(const std::string& prefix) const;
  std::vector<std::string> linksOfArm(const std::string& arm) const;  ///< "left"/"right"

  /// 12 values in the order  left_j1..j6, right_j1..j6  (i.e. C2's q12 layout).
  moveit::core::RobotState makeStateQ12(const std::vector<double>& q12) const;

  /// Names of the 12 arm joints in the same order as makeStateQ12().
  const std::vector<std::string>& armJointNames() const { return arm_joint_names_; }

  /// Arm link names (all links of both arms), used to build the cross-arm ACM.
  const std::vector<std::string>& leftLinks() const { return left_links_; }
  const std::vector<std::string>& rightLinks() const { return right_links_; }
  const std::vector<std::string>& otherLinks() const { return other_links_; }

private:
  DualArmScene() = default;

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr scene_;
  GeometryReport geometry_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> left_links_;
  std::vector<std::string> right_links_;
  std::vector<std::string> other_links_;
};

/// Resolve a  package://<pkg>/<rel>  URI to an absolute filesystem path.
/// Returns false and fills `reason` when it cannot be resolved.
/// Exposed for the negative tests.
bool resolvePackageUri(const std::string& uri, std::string* absolute_path, std::string* reason);

/// Read a file fully; returns false when unreadable.  Exposed for tests.
bool readFileBytes(const std::string& path, std::string* out);

}  // namespace dual_arm_pose_optimization
