// src/scene_loader.cpp
#include "dual_arm_pose_optimization/scene_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shapes.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace dual_arm_pose_optimization {

namespace fs = std::filesystem;

namespace {

std::string shapeTypeName(shapes::ShapeType t) {
  switch (t) {
    case shapes::UNKNOWN_SHAPE: return "unknown";
    case shapes::SPHERE: return "sphere";
    case shapes::CYLINDER: return "cylinder";
    case shapes::CONE: return "cone";
    case shapes::BOX: return "box";
    case shapes::PLANE: return "plane";
    case shapes::MESH: return "mesh";
    case shapes::OCTREE: return "octree";
    default: return "other";
  }
}

std::size_t fileSize(const std::string& path, bool* ok) {
  std::error_code ec;
  const auto sz = fs::file_size(path, ec);
  if (ec) {
    *ok = false;
    return 0;
  }
  *ok = true;
  return static_cast<std::size_t>(sz);
}

std::string armOf(const std::string& link, const std::string& lp, const std::string& rp) {
  if (link.rfind(lp, 0) == 0) return "left";
  if (link.rfind(rp, 0) == 0) return "right";
  return "other";
}

}  // namespace

bool readFileBytes(const std::string& path, std::string* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::ostringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

bool resolvePackageUri(const std::string& uri, std::string* absolute_path,
                       std::string* reason) {
  const std::string scheme = "package://";
  if (uri.rfind(scheme, 0) != 0) {
    // file:// or a plain path
    if (uri.rfind("file://", 0) == 0) {
      const std::string p = uri.substr(7);
      if (fs::exists(p)) {
        *absolute_path = p;
        return true;
      }
      *reason = "file:// target does not exist: " + p;
      return false;
    }
    if (fs::exists(uri)) {
      *absolute_path = uri;
      return true;
    }
    *reason = "not a package:// URI and no such file: " + uri;
    return false;
  }

  const std::string rest = uri.substr(scheme.size());
  const std::size_t slash = rest.find('/');
  if (slash == std::string::npos) {
    *reason = "malformed package:// URI (no '/' after the package name): " + uri;
    return false;
  }
  const std::string pkg = rest.substr(0, slash);
  const std::string rel = rest.substr(slash + 1);

  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory(pkg);
  } catch (const std::exception& e) {
    *reason = "package '" + pkg + "' not found in the ament index (" + e.what() +
              "). Did you `source install/setup.bash`?";
    return false;
  }

  const fs::path full = fs::path(share) / rel;
  std::error_code ec;
  if (!fs::exists(full, ec)) {
    *reason = "resolved package '" + pkg + "' to " + share +
              " but the file is missing: " + full.string();
    return false;
  }
  bool ok = false;
  const std::size_t sz = fileSize(full.string(), &ok);
  if (!ok || sz == 0) {
    *reason = "resolved to " + full.string() + " but it is empty or unreadable";
    return false;
  }
  *absolute_path = full.string();
  return true;
}

std::size_t GeometryReport::shapesOfArm(const std::string& arm) const {
  const auto it = shapes_per_arm.find(arm);
  return it == shapes_per_arm.end() ? 0u : it->second;
}

const LinkGeometry* GeometryReport::link(const std::string& name) const {
  for (const auto& l : links) {
    if (l.link_name == name) return &l;
  }
  return nullptr;
}

std::string GeometryReport::to_string() const {
  std::ostringstream os;
  os << "=== C4.2 S1 geometry report ===\n";
  os << "[provenance]\n";
  os << "  urdf            : " << urdf_path << "  (" << urdf_bytes << " bytes)\n";
  os << "  srdf            : " << srdf_path << "  (" << srdf_bytes << " bytes)\n";
  os << "  model           : " << model_name << "   frame=" << model_frame << "\n";
  os << "  urdf links/joints: " << urdf_links << " / " << urdf_joints << "\n";
  os << "  <collision> blks : " << urdf_collision_elements << "\n";
  os << "  groups          :";
  for (const auto& g : joint_model_groups) os << " " << g;
  os << "\n  srdf disable_collisions : " << srdf_disable_collisions << "\n";
  os << "  collision detector      : " << collision_detector_name << "\n";

  os << "[check A] mesh pre-flight\n";
  os << "  mesh refs       : " << urdf_mesh_refs << "\n";
  os << "  resolved        : " << meshes_resolved << "\n";
  os << "  failed          : " << failed_meshes.size() << "\n";
  for (const auto& f : failed_meshes) os << "      FAIL " << f << "\n";
  os << "  unique mesh files (" << resolved_mesh_paths.size() << "):\n";
  for (const auto& p : resolved_mesh_paths) os << "      " << p << "\n";

  os << "[check B] collision geometry loaded into RobotModel\n";
  os << "  links with collision : " << links.size() << "\n";
  os << "  total shapes         : " << total_collision_shapes << "\n";
  os << "  per arm              :";
  for (const auto& kv : shapes_per_arm) os << " " << kv.first << "=" << kv.second;
  os << "\n  per type             :";
  for (const auto& kv : shapes_per_type) os << " " << kv.first << "=" << kv.second;
  os << "\n  mesh verts/tris      : " << total_mesh_vertices << " / "
     << total_mesh_triangles << "\n";
  os << "  per-link:\n";
  for (const auto& l : links) {
    os << "      " << l.link_name << " [" << l.arm << "] shapes=" << l.n_shapes
       << " types=";
    for (std::size_t i = 0; i < l.shape_types.size(); ++i) {
      os << (i ? "," : "") << l.shape_types[i];
    }
    if (l.mesh_vertices > 0) {
      os << " mesh(v=" << l.mesh_vertices << ",t=" << l.mesh_triangles << ")";
    }
    os << "\n";
  }
  if (!urdf_links_without_collision.empty()) {
    os << "  urdf links WITHOUT collision (expected: world, grasp_frame, ...):\n      ";
    for (std::size_t i = 0; i < urdf_links_without_collision.size(); ++i) {
      os << (i ? ", " : "") << urdf_links_without_collision[i];
    }
    os << "\n";
  }
  return os.str();
}

std::shared_ptr<DualArmScene> DualArmScene::load(const SceneLoadOptions& options) {
  auto scene = std::shared_ptr<DualArmScene>(new DualArmScene());
  GeometryReport& rep = scene->geometry_;
  rep.urdf_path = options.urdf_path;
  rep.srdf_path = options.srdf_path;

  // ---- read the files (fail loudly, naming the path) --------------------
  std::string urdf_xml, srdf_xml;
  if (!readFileBytes(options.urdf_path, &urdf_xml)) {
    throw SceneLoadError("SceneLoadError: cannot read URDF '" + options.urdf_path + "'");
  }
  rep.urdf_bytes = urdf_xml.size();
  if (!readFileBytes(options.srdf_path, &srdf_xml)) {
    throw SceneLoadError("SceneLoadError: cannot read SRDF '" + options.srdf_path + "'");
  }
  rep.srdf_bytes = srdf_xml.size();
  if (rep.urdf_bytes == 0) {
    throw SceneLoadError("SceneLoadError: URDF '" + options.urdf_path + "' is empty");
  }
  if (rep.srdf_bytes == 0) {
    throw SceneLoadError("SceneLoadError: SRDF '" + options.srdf_path + "' is empty");
  }

  // ---- parse URDF / SRDF ------------------------------------------------
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_xml);
  if (!urdf) {
    throw SceneLoadError("SceneLoadError: urdf::parseURDF failed for '" +
                         options.urdf_path + "'");
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, srdf_xml)) {
    throw SceneLoadError("SceneLoadError: srdf::Model::initString failed for '" +
                         options.srdf_path + "'");
  }

  rep.model_name = urdf->getName();
  rep.urdf_links = urdf->links_.size();
  rep.urdf_joints = urdf->joints_.size();
  rep.srdf_disable_collisions = srdf->getDisabledCollisionPairs().size();

  // ---- Check A: mesh pre-flight ----------------------------------------
  {
    std::set<std::string> unique_uris;
    for (const auto& lk : urdf->links_) {
      if (!lk.second || !lk.second->collision || !lk.second->collision->geometry) continue;
      const auto g = lk.second->collision->geometry;
      if (g->type != urdf::Geometry::MESH) continue;
      const auto mesh = std::static_pointer_cast<urdf::Mesh>(g);
      unique_uris.insert(mesh->filename);
    }
    rep.urdf_mesh_refs = unique_uris.size();

    std::set<std::string> unique_paths;
    for (const auto& uri : unique_uris) {
      std::string path, reason;
      if (resolvePackageUri(uri, &path, &reason)) {
        unique_paths.insert(path);
      } else {
        rep.failed_meshes.push_back(uri + "  ->  " + reason);
      }
    }
    rep.resolved_mesh_paths.assign(unique_paths.begin(), unique_paths.end());
    rep.meshes_resolved = rep.resolved_mesh_paths.size();

    if (options.require_meshes && !rep.failed_meshes.empty()) {
      std::ostringstream os;
      os << "SceneLoadError: " << rep.failed_meshes.size()
         << " mesh resource(s) could not be resolved; refusing to build a scene "
            "with silently-empty collision geometry.\n";
      for (const auto& f : rep.failed_meshes) os << "  - " << f << "\n";
      os << "Hint: `source install/setup.bash` so that the ament index contains "
            "the mesh packages.";
      throw SceneLoadError(os.str());
    }
  }

  // ---- build RobotModel + PlanningScene ---------------------------------
  moveit::core::RobotModelPtr model;
  try {
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  } catch (const std::exception& e) {
    throw SceneLoadError(std::string("SceneLoadError: RobotModel construction failed: ") +
                         e.what());
  }
  scene->robot_model_ = model;
  rep.model_frame = model->getModelFrame();
  rep.joint_model_groups = model->getJointModelGroupNames();

  try {
    scene->scene_ = std::make_shared<planning_scene::PlanningScene>(model);
  } catch (const std::exception& e) {
    throw SceneLoadError(std::string("SceneLoadError: PlanningScene construction failed: ") +
                         e.what());
  }
  rep.collision_detector_name = scene->scene_->getCollisionDetectorName();

  // ---- Check B: count what actually loaded ------------------------------
  for (const std::string& name : model->getLinkModelNames()) {
    const moveit::core::LinkModel* lm = model->getLinkModel(name);
    if (lm == nullptr) continue;
    const auto& shapes_vec = lm->getShapes();

    LinkGeometry lg;
    lg.link_name = name;
    lg.arm = armOf(name, options.left_prefix, options.right_prefix);
    lg.n_shapes = shapes_vec.size();
    for (const auto& sh : shapes_vec) {
      if (!sh) continue;
      lg.shape_types.push_back(shapeTypeName(sh->type));
      rep.shapes_per_type[shapeTypeName(sh->type)] += 1;
      if (sh->type == shapes::MESH) {
        const auto mesh = std::static_pointer_cast<const shapes::Mesh>(sh);
        lg.mesh_vertices += mesh->vertex_count;
        lg.mesh_triangles += mesh->triangle_count;
      }
    }
    rep.total_collision_shapes += lg.n_shapes;
    rep.total_mesh_vertices += lg.mesh_vertices;
    rep.total_mesh_triangles += lg.mesh_triangles;
    rep.shapes_per_arm[lg.arm] += lg.n_shapes;
    if (lg.n_shapes > 0) rep.links.push_back(lg);

    if (lg.arm == "left") scene->left_links_.push_back(name);
    else if (lg.arm == "right") scene->right_links_.push_back(name);
    else scene->other_links_.push_back(name);
  }

  // Expected number of collision elements, straight from the URDF.
  for (const auto& lk : urdf->links_) {
    if (!lk.second) continue;
    if (lk.second->collision && lk.second->collision->geometry) {
      rep.urdf_collision_elements += 1;
    } else {
      rep.urdf_links_without_collision.push_back(lk.first);
    }
  }

  if (options.require_meshes) {
    if (rep.shapes_per_arm["left"] == 0 || rep.shapes_per_arm["right"] == 0) {
      std::ostringstream os;
      os << "SceneLoadError: collision geometry is missing for an arm "
            "(left=" << rep.shapes_per_arm["left"]
         << ", right=" << rep.shapes_per_arm["right"]
         << "). This is the silent-failure mode: meshes were not resolvable at "
            "RobotModel construction time.";
      throw SceneLoadError(os.str());
    }
    if (rep.total_mesh_vertices == 0) {
      throw SceneLoadError(
          "SceneLoadError: no mesh vertices were loaded; collision geometry is "
          "empty even though the URDF declares meshes.");
    }
  }

  // ---- arm joint names (C2 q12 order) -----------------------------------
  for (const char* p : {"left_", "right_"}) {
    for (int i = 1; i <= 6; ++i) {
      const std::string jn = std::string(p) + "j" + std::to_string(i);
      if (model->getJointModel(jn) == nullptr) {
        throw SceneLoadError("SceneLoadError: expected joint '" + jn +
                             "' is not in the RobotModel");
      }
      scene->arm_joint_names_.push_back(jn);
    }
  }

  return scene;
}

std::vector<std::string> DualArmScene::linksWithPrefix(const std::string& prefix) const {
  std::vector<std::string> out;
  for (const std::string& n : robot_model_->getLinkModelNames()) {
    if (n.rfind(prefix, 0) == 0) out.push_back(n);
  }
  return out;
}

std::vector<std::string> DualArmScene::linksOfArm(const std::string& arm) const {
  if (arm == "left") return left_links_;
  if (arm == "right") return right_links_;
  return other_links_;
}

moveit::core::RobotState DualArmScene::makeStateQ12(const std::vector<double>& q12) const {
  if (q12.size() != arm_joint_names_.size()) {
    throw SceneLoadError("makeStateQ12 expects 12 joint values, got " +
                         std::to_string(q12.size()));
  }
  moveit::core::RobotState st(robot_model_);
  st.setToDefaultValues();
  for (std::size_t i = 0; i < arm_joint_names_.size(); ++i) {
    st.setJointPositions(arm_joint_names_[i], &q12[i]);
  }
  st.update();
  return st;
}

}  // namespace dual_arm_pose_optimization
