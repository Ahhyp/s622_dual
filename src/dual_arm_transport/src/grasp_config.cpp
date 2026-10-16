// src/grasp_config.cpp -- W1: strict grasp/sample/support config + measured
// gripper geometry.  See grasp_config.hpp for the frame discipline and for why
// 0.061 m is a stroke and not a proof.
#include "dual_arm_transport/grasp_config.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>

#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>
#include <yaml-cpp/yaml.h>

#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace dual_arm_transport {
namespace {

constexpr double kTol = 1e-9;

bool isFinite(const Vec3& v) { return v.allFinite(); }

// ---- strict YAML key checking ---------------------------------------------
// A typo'd key must fail the load, never be silently ignored: a silently
// ignored key is an unvalidated assumption, which is exactly what D1 forbids.
std::string checkKeys(const YAML::Node& n, const std::set<std::string>& allowed,
                      const std::string& where) {
  if (!n.IsMap()) return where + ": expected a mapping";
  std::string bad;
  int nbad = 0;
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    if (allowed.count(k) == 0) {
      if (nbad++) bad += ", ";
      bad += k;
    }
  }
  if (nbad > 0) {
    return where + ": unknown key(s) [" + bad + "] (typo protection is part of the contract)";
  }
  return {};
}

Vec3 readVec3(const YAML::Node& n, const std::string& where) {
  if (!n || !n.IsSequence() || n.size() != 3) {
    throw GraspConfigError(where + ": expected a 3-element sequence");
  }
  Vec3 v;
  for (int i = 0; i < 3; ++i) v[i] = n[i].as<double>();
  if (!isFinite(v)) throw GraspConfigError(where + ": non-finite component");
  return v;
}

Mat3 rpyToMat3(double roll, double pitch, double yaw) {
  // Fixed-axis XYZ (URDF convention): R = Rz(yaw) * Ry(pitch) * Rx(roll).
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3::UnitX()))
      .toRotationMatrix();
}

// ---- AABB helpers (geometric_shapes) --------------------------------------

/// Local-frame AABB of one collision shape.
bool shapeAabb(const shapes::Shape* shape, bodies::AABB& out) {
  if (shape == nullptr) return false;
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
  if (body == nullptr) return false;
  body->computeBoundingBox(out);
  return true;
}

struct Box3 {
  Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
  Vec3 hi = Vec3::Constant(-std::numeric_limits<double>::infinity());
  bool valid() const { return lo.x() <= hi.x(); }
  void extend(const Vec3& p) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
};

/// AABB of `links`, expressed in `tcp_frame`, at the given robot state.
Box3 aabbInFrame(const moveit::core::RobotState& state, const moveit::core::RobotModel& model,
                 const std::vector<std::string>& links, const std::string& tcp_frame,
                 std::size_t* n_shapes) {
  Box3 box;
  const Eigen::Isometry3d T_tcp = state.getGlobalLinkTransform(tcp_frame);
  const Eigen::Isometry3d T_tcp_inv = T_tcp.inverse();
  for (const std::string& link : links) {
    const moveit::core::LinkModel* lm = model.getLinkModel(link);
    if (lm == nullptr) continue;
    const std::vector<shapes::ShapeConstPtr>& shapes_vec = lm->getShapes();
    if (shapes_vec.empty()) continue;
    const EigenSTL::vector_Isometry3d& origins = lm->getCollisionOriginTransforms();
    const Eigen::Isometry3d& T_link = state.getGlobalLinkTransform(lm);
    for (std::size_t i = 0; i < shapes_vec.size(); ++i) {
      bodies::AABB local;
      if (!shapeAabb(shapes_vec[i].get(), local)) continue;
      Eigen::Isometry3d T = T_tcp_inv * T_link;
      if (i < origins.size()) T = T * origins[i];
      const Eigen::Vector3d lo = local.min();
      const Eigen::Vector3d hi = local.max();
      for (int k = 0; k < 8; ++k) {
        const Eigen::Vector3d c((k & 1) ? hi.x() : lo.x(), (k & 2) ? hi.y() : lo.y(),
                                (k & 4) ? hi.z() : lo.z());
        box.extend(T * c);
      }
      if (n_shapes != nullptr) ++(*n_shapes);
    }
  }
  return box;
}

/// Separation along TCP x between two boxes; negative means interpenetration.
double gapAlongX(const Box3& a, const Box3& b) {
  if (!a.valid() || !b.valid()) return std::numeric_limits<double>::quiet_NaN();
  const double a_lo = a.lo.x(), b_lo = b.lo.x();
  const Box3* left = (a_lo <= b_lo) ? &a : &b;
  const Box3* right = (a_lo <= b_lo) ? &b : &a;
  return right->lo.x() - left->hi.x();
}

std::string fmtVec(const Vec3& v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(6) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return os.str();
}

}  // namespace

Mat3 boxInertia(const Vec3& size_xyz, double mass) {
  const double x = size_xyz.x(), y = size_xyz.y(), z = size_xyz.z();
  Mat3 I = Mat3::Zero();
  I(0, 0) = mass * (y * y + z * z) / 12.0;
  I(1, 1) = mass * (x * x + z * z) / 12.0;
  I(2, 2) = mass * (x * x + y * y) / 12.0;
  return I;
}

dual_arm::Iso3 makeEndGraspTransform(const Vec3& size_xyz, int sign,
                                     double tcp_offset_along_normal_m) {
  if (sign != 1 && sign != -1) {
    throw GraspConfigError("makeEndGraspTransform: sign must be +1 or -1");
  }
  const double s = static_cast<double>(sign);
  // Grasp region = the +/-X END of the sample, approached from ABOVE.
  //   z_tcp = -z_obj  (points from the TCP into the sample, i.e. downward)
  //   x_tcp = +y_obj  (the pad CLOSING axis; pad faces are perpendicular to it)
  //   y_tcp = z_tcp x x_tcp = +x_obj
  // Columns of R are the images of e_x, e_y, e_z, so det(R) = +1 and R is a
  // proper rotation.  G_L and G_R differ ONLY in the x translation, which is what
  // makes the two ends distinct.
  Mat3 R;
  R.col(0) = Vec3(0.0, 1.0, 0.0);
  R.col(1) = Vec3(1.0, 0.0, 0.0);
  R.col(2) = Vec3(0.0, 0.0, -1.0);
  dual_arm::Iso3 T = dual_arm::Iso3::Identity();
  T.linear() = R;
  // Centre the 20.6 mm wide pads on the end face instead of overhanging it.
  const double kPadHalfAlongRod = 0.0103;
  const double along = std::max(size_xyz.x() / 2.0 - kPadHalfAlongRod, 0.0);
  T.translation() =
      Vec3(s * along, 0.0, size_xyz.z() / 2.0 + tcp_offset_along_normal_m);
  return T;
}

std::string validateGraspConfig(const GraspConfig& cfg) {
  const dual_arm::GraspGeometry& g = cfg.grasp;

  // ---- object body ----
  if (!isFinite(g.size_xyz) || g.size_xyz.minCoeff() <= 0.0) {
    return "object.size_xyz must be finite and strictly positive";
  }
  if (!(std::isfinite(g.mass) && g.mass > 0.0)) return "object.mass must be finite and > 0";
  if (!g.inertia.allFinite()) return "inertia is not finite";
  if ((g.inertia - g.inertia.transpose()).cwiseAbs().maxCoeff() > 1e-12) {
    return "inertia is not symmetric";
  }
  Eigen::LLT<Mat3> llt(g.inertia);
  if (llt.info() != Eigen::Success) return "inertia is not positive definite";

  // ---- frames and transforms ----
  const std::string gv = g.validate();
  if (!gv.empty()) return "GraspGeometry::validate(): " + gv;
  if (!dual_arm::isRigidTransform(g.world_T_object_star)) return "world_T_object_star is not rigid";
  if (!dual_arm::isRigidTransform(g.object_T_left_tcp)) return "object_T_left_tcp is not rigid";
  if (!dual_arm::isRigidTransform(g.object_T_right_tcp)) return "object_T_right_tcp is not rigid";
  const double glr = (g.object_T_left_tcp.matrix() - g.object_T_right_tcp.matrix()).cwiseAbs().maxCoeff();
  if (glr <= kTol) return "G_L and G_R are identical: a dual-end grasp needs two distinct ends";

  // ---- sample variant must declare any geometry change (D1) ----
  if (cfg.sample_geometry.variant != "box") {
    if (cfg.sample_geometry.reason.empty() || cfg.sample_geometry.diff_vs_box.empty()) {
      return "sample_geometry.variant '" + cfg.sample_geometry.variant +
             "' requires non-empty reason and diff_vs_box (D1: a geometry change must be "
             "recorded)";
    }
    // A non-box variant cannot be checked against size_xyz by this function.
    return {};
  }

  // ---- box: regions must sit exactly on the declared faces ----
  const double hx = g.size_xyz.x() / 2.0;
  const struct {
    const GraspRegion* r;
    int sign;
    const char* name;
  } sides[2] = {{&cfg.left_region, +1, "left"}, {&cfg.right_region, -1, "right"}};

  for (const auto& s : sides) {
    const GraspRegion& r = *s.r;
    const std::string where = std::string("regions.") + s.name;
    if (r.polygon_local.size() < 4) return where + ".polygon_local needs >= 4 vertices";
    if (r.contact_faces.empty()) return where + ".contact_faces is empty";
    if (!(std::isfinite(r.grasp_depth_m) && r.grasp_depth_m > 0.0)) {
      return where + ".grasp_depth_m must be finite and > 0";
    }
    if (std::abs(r.grasp_depth_m - hx) > 1e-9) {
      return where + ".grasp_depth_m must equal size_x/2 (the grasped face is a face of the box)";
    }
    if (!isFinite(r.approach_dir_local)) return where + ".approach_dir_local is not finite";
    if (std::abs(r.approach_dir_local.norm() - 1.0) > 1e-9) {
      return where + ".approach_dir_local must be unit norm";
    }
    const Vec3 expect(s.sign, 0.0, 0.0);
    if ((r.approach_dir_local - expect).cwiseAbs().maxCoeff() > 1e-9) {
      return where + ".approach_dir_local must be " + fmtVec(expect) +
             " (outward normal of the grasped face)";
    }
    // Planarity + exact face plane.
    const Vec3 ref = r.polygon_local.front();
    const Vec3 e1 = r.polygon_local[1] - ref;
    const Vec3 e2 = r.polygon_local[2] - ref;
    const Vec3 nrm = e1.cross(e2).normalized();
    for (const Vec3& p : r.polygon_local) {
      if (!isFinite(p)) return where + ".polygon_local has a non-finite vertex";
      if (std::abs(nrm.dot(p - ref)) > 1e-9) return where + ".polygon_local is not planar";
      if (std::abs(p.x() - s.sign * hx) > 1e-9) {
        return where + ".polygon_local is not on the "+std::string(s.sign > 0 ? "+x" : "-x")+" face plane";
      }
    }
    if (std::abs(nrm.dot(expect)) < 1.0 - 1e-9) {
      return where + ".polygon_local winding does not match the face normal";
    }
  }

  // ---- the tool frame must be the one this package builds (D1/D2) ----
  // The pads must close along the object's Y axis and the tool must come down
  // from above.  Checking it here means a hand-edited transform cannot quietly
  // describe a different grasp than the one the screening measured.
  for (const auto& pr : {std::make_pair(std::string("left"), g.object_T_left_tcp),
                         std::make_pair(std::string("right"), g.object_T_right_tcp)}) {
    const Vec3 zt = pr.second.linear().col(2);
    const Vec3 xt = pr.second.linear().col(0);
    if ((zt - Vec3(0.0, 0.0, -1.0)).cwiseAbs().maxCoeff() > 1e-9) {
      return "object_T_" + pr.first + "_tcp: -z_tcp must be +z_obj (approach from above)";
    }
    if ((xt - Vec3(0.0, 1.0, 0.0)).cwiseAbs().maxCoeff() > 1e-9) {
      return "object_T_" + pr.first + "_tcp: the pads must close along +y_obj";
    }
  }

  // ---- grip thickness vs the closing STROKE (necessary, not sufficient) ----
  if (!(std::isfinite(cfg.grip_thickness_m) && cfg.grip_thickness_m > 0.0)) {
    return "grip_thickness_m must be finite and > 0";
  }
  // The closing axis is +y_obj, so the pad separation is the object's Y extent,
  // NOT its X extent.  size_x is the ROD LENGTH between the two grasped ends.
  if (std::abs(cfg.grip_thickness_m - g.size_xyz.y()) > 1e-9) {
    return "grip_thickness_m must equal size_y (the pad closing axis of this grasp)";
  }
  if (!(std::isfinite(cfg.gripper_max_opening_m) && cfg.gripper_max_opening_m > 0.0)) {
    return "gripper_max_opening_m must be finite and > 0";
  }
  if (cfg.grip_thickness_m > cfg.gripper_max_opening_m + 1e-12) {
    return "grip_thickness_m exceeds the closing stroke; the fingers physically cannot close";
  }

  // ---- D1: the envelope question is NEVER answered by grep-ing clearances ----
  // clearance entries are measurements; a non-zero intrusion is a hard failure.
  for (const auto& c : {std::make_pair(std::string("left"), cfg.left_clearance),
                        std::make_pair(std::string("right"), cfg.right_clearance)}) {
    if (!std::isfinite(c.second.intrusion_m) || !std::isfinite(c.second.margin_m)) {
      return "clearance." + c.first + " is not finite";
    }
    if (c.second.intrusion_m > 1e-9) {
      return "clearance." + c.first + ".intrusion_m > 0: the finger envelope enters the sample";
    }
  }

  // ---- support (D4) ----
  if (!(std::isfinite(cfg.support.top_z_m))) return "support.top_z_m is not finite";
  if (!(std::isfinite(cfg.support.radius_m) && cfg.support.radius_m > 0.0)) {
    return "support.radius_m must be finite and > 0";
  }
  if (!(std::isfinite(cfg.support.height_m) && cfg.support.height_m > 0.0)) {
    return "support.height_m must be finite and > 0";
  }
  if (cfg.support.top_z_status != "candidate" && cfg.support.top_z_status != "screened" &&
      cfg.support.top_z_status != "rejected") {
    return "support.top_z_status must be candidate|screened|rejected";
  }
  if (cfg.support.top_z_status != "candidate" &&
      std::abs(cfg.support.top_z_m - cfg.support.height_m) > 1e-9) {
    return "a non-candidate support.top_z_status requires top_z_m == height_m";
  }

  // ---- D7 ----
  if (!(std::isfinite(cfg.d_approach_m) && cfg.d_approach_m > 0.0)) {
    return "d_approach_m must be finite and > 0 (it is a candidate, but it must be positive)";
  }

  if (cfg.collision_links.empty()) return "collision_links is empty";
  return {};
}

GraspConfig loadGraspConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw GraspConfigError("cannot read " + path + ": " + e.what());
  }
  if (!root.IsMap()) throw GraspConfigError(path + ": root must be a mapping");

  std::string err = checkKeys(root,
                              {"object", "world_object_star", "grasps", "regions", "grip_thickness_m",
                               "clearance", "support", "d_approach_m", "gripper_max_opening_m"},
                              "root");
  if (!err.empty()) throw GraspConfigError(err);

  GraspConfig cfg;

  // ---- object ----
  const YAML::Node obj = root["object"];
  err = checkKeys(obj, {"frame", "size_xyz", "mass", "provenance", "collision_links",
                        "sample_geometry"},
                  "object");
  if (!err.empty()) throw GraspConfigError(err);
  cfg.grasp.object_frame = obj["frame"].as<std::string>();
  cfg.grasp.size_xyz = readVec3(obj["size_xyz"], "object.size_xyz");
  cfg.grasp.mass = obj["mass"].as<double>();
  cfg.grasp.inertia = boxInertia(cfg.grasp.size_xyz, cfg.grasp.mass);
  if (obj["provenance"]) cfg.provenance = obj["provenance"].as<std::string>();
  if (obj["collision_links"]) {
    cfg.collision_links = obj["collision_links"].as<std::vector<std::string>>();
  }
  const YAML::Node sg = obj["sample_geometry"];
  err = checkKeys(sg, {"variant", "reason", "diff_vs_box"}, "object.sample_geometry");
  if (!err.empty()) throw GraspConfigError(err);
  if (sg) {
    cfg.sample_geometry.variant = sg["variant"].as<std::string>();
    if (sg["reason"]) cfg.sample_geometry.reason = sg["reason"].as<std::string>();
    if (sg["diff_vs_box"]) cfg.sample_geometry.diff_vs_box = sg["diff_vs_box"].as<std::string>();
  }

  // ---- world_object_star ----
  const YAML::Node wo = root["world_object_star"];
  err = checkKeys(wo, {"frame", "translation", "rpy"}, "world_object_star");
  if (!err.empty()) throw GraspConfigError(err);
  cfg.grasp.world_frame = wo["frame"].as<std::string>();
  const Vec3 t = readVec3(wo["translation"], "world_object_star.translation");
  const Vec3 rpy = readVec3(wo["rpy"], "world_object_star.rpy");
  cfg.grasp.world_T_object_star = dual_arm::Iso3::Identity();
  cfg.grasp.world_T_object_star.linear() = rpyToMat3(rpy.x(), rpy.y(), rpy.z());
  cfg.grasp.world_T_object_star.translation() = t;

  // ---- grasps (G_L / G_R built from sign + offset, never pasted in) ----
  const YAML::Node gr = root["grasps"];
  err = checkKeys(gr, {"left", "right"}, "grasps");
  if (!err.empty()) throw GraspConfigError(err);
  struct Side {
    const char* key;
    dual_arm::Iso3* T;
    std::string* frame;
    int* sign;
  };
  int sign_l = 0, sign_r = 0;
  const Side sides[2] = {{"left", &cfg.grasp.object_T_left_tcp, &cfg.grasp.left_tcp_frame, &sign_l},
                         {"right", &cfg.grasp.object_T_right_tcp, &cfg.grasp.right_tcp_frame,
                          &sign_r}};
  for (const Side& s : sides) {
    const YAML::Node n = gr[s.key];
    if (!n) throw GraspConfigError(std::string("grasps.") + s.key + " is missing");
    err = checkKeys(n, {"tcp_frame", "sign", "tcp_offset_along_normal_m"},
                    std::string("grasps.") + s.key);
    if (!err.empty()) throw GraspConfigError(err);
    *s.frame = n["tcp_frame"].as<std::string>();
    *s.sign = n["sign"].as<int>();
    const double off = n["tcp_offset_along_normal_m"]
                           ? n["tcp_offset_along_normal_m"].as<double>()
                           : 0.0;
    *s.T = makeEndGraspTransform(cfg.grasp.size_xyz, *s.sign, off);
  }

  // ---- regions ----
  const YAML::Node rg = root["regions"];
  err = checkKeys(rg, {"left", "right"}, "regions");
  if (!err.empty()) throw GraspConfigError(err);
  struct RSide {
    const char* key;
    GraspRegion* r;
    int sign;
  };
  const RSide rsides[2] = {{"left", &cfg.left_region, sign_l}, {"right", &cfg.right_region, sign_r}};
  for (const RSide& s : rsides) {
    const YAML::Node n = rg[s.key];
    if (!n) throw GraspConfigError(std::string("regions.") + s.key + " is missing");
    err = checkKeys(n, {"polygon_local", "approach_dir_local", "contact_faces", "grasp_depth_m"},
                    std::string("regions.") + s.key);
    if (!err.empty()) throw GraspConfigError(err);
    GraspRegion& r = *s.r;
    r.name = s.key;
    for (const YAML::Node& p : n["polygon_local"]) {
      r.polygon_local.push_back(readVec3(p, std::string("regions.") + s.key + ".polygon_local[]"));
    }
    r.approach_dir_local = readVec3(n["approach_dir_local"],
                                    std::string("regions.") + s.key + ".approach_dir_local");
    for (const YAML::Node& seg : n["contact_faces"]) {
      if (!seg.IsSequence() || seg.size() != 2) {
        throw GraspConfigError(std::string("regions.") + s.key +
                               ".contact_faces[] must be a pair of points");
      }
      r.contact_faces.emplace_back(
          readVec3(seg[0], std::string("regions.") + s.key + ".contact_faces[][0]"),
          readVec3(seg[1], std::string("regions.") + s.key + ".contact_faces[][1]"));
    }
    r.grasp_depth_m = n["grasp_depth_m"].as<double>();
  }

  // ---- scalars, clearance, support ----
  cfg.grip_thickness_m = root["grip_thickness_m"].as<double>();
  cfg.gripper_max_opening_m = root["gripper_max_opening_m"].as<double>();
  cfg.d_approach_m = root["d_approach_m"].as<double>();

  const YAML::Node cl = root["clearance"];
  err = checkKeys(cl, {"left", "right"}, "clearance");
  if (!err.empty()) throw GraspConfigError(err);
  struct CSide {
    const char* key;
    FingerObjectClearance* c;
  };
  const CSide csides[2] = {{"left", &cfg.left_clearance}, {"right", &cfg.right_clearance}};
  for (const CSide& s : csides) {
    const YAML::Node n = cl[s.key];
    err = checkKeys(n, {"intrusion_m", "margin_m", "note"}, std::string("clearance.") + s.key);
    if (!err.empty()) throw GraspConfigError(err);
    if (n["intrusion_m"]) s.c->intrusion_m = n["intrusion_m"].as<double>();
    if (n["margin_m"]) s.c->margin_m = n["margin_m"].as<double>();
    if (n["note"]) s.c->note = n["note"].as<std::string>();
  }

  const YAML::Node su = root["support"];
  err = checkKeys(su, {"id", "shape", "radius_m", "height_m", "top_z_m", "top_z_status"}, "support");
  if (!err.empty()) throw GraspConfigError(err);
  cfg.support.id = su["id"].as<std::string>();
  cfg.support.shape = su["shape"].as<std::string>();
  cfg.support.radius_m = su["radius_m"].as<double>();
  cfg.support.height_m = su["height_m"].as<double>();
  cfg.support.top_z_m = su["top_z_m"].as<double>();
  cfg.support.top_z_status = su["top_z_status"].as<std::string>();

  const std::string v = validateGraspConfig(cfg);
  if (!v.empty()) throw GraspConfigError(path + ": " + v);
  return cfg;
}

// ---------------------------------------------------------------------------
// Measured gripper geometry (D1)
// ---------------------------------------------------------------------------

std::string GripperMeasurement::to_string() const {
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << prefix << " gripper @ " << tcp_frame << " (wrist " << wrist_link << ")\n";
  os << "  finger AABB in TCP: min " << fmtVec(fingers_min_tcp) << " max " << fmtVec(fingers_max_tcp)
     << "  (" << n_finger_shapes << " shapes)\n";
  os << "  wrist  AABB in TCP: min " << fmtVec(wrist_min_tcp) << " max " << fmtVec(wrist_max_tcp)
     << "  (" << n_wrist_shapes << " shapes)\n";
  os << "  pad z window in TCP: [" << pad_z_near_m << ", " << pad_z_far_m << "]\n";
  os << "  finger gap along TCP x: closed " << closed_pad_gap_m << " m, open " << open_pad_gap_m
     << " m";
  return os.str();
}

GripperMeasurement measureGripper(const dual_arm_pose_optimization::DualArmScene& scene,
                                  const std::string& prefix) {
  const moveit::core::RobotModelConstPtr& model = scene.robotModel();
  GripperMeasurement m;
  m.prefix = prefix;
  m.wrist_link = prefix + "_wrist3_link";
  m.tcp_frame = prefix + "_grasp_frame";

  if (model->getLinkModel(m.tcp_frame) == nullptr) {
    throw GraspConfigError("measureGripper: no link '" + m.tcp_frame + "' in the model");
  }
  if (model->getLinkModel(m.wrist_link) == nullptr) {
    throw GraspConfigError("measureGripper: no link '" + m.wrist_link + "' in the model");
  }

  const std::string j1 = prefix + "_finger1_joint";
  const std::string j2 = prefix + "_finger2_joint";
  const moveit::core::JointModel* jm1 = model->getJointModel(j1);
  const moveit::core::JointModel* jm2 = model->getJointModel(j2);
  if (jm1 == nullptr || jm2 == nullptr) {
    throw GraspConfigError("measureGripper: missing prismatic joint '" + j1 + "' or '" + j2 + "'");
  }
  const double stroke1 = jm1->getVariableBounds()[0].max_position_;
  const double stroke2 = jm2->getVariableBounds()[0].min_position_;

  const std::vector<std::string> fingers = {prefix + "_finger1", prefix + "_finger2"};
  const std::vector<std::string> wrist = {m.wrist_link};

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  const auto setFingers = [&](double q1, double q2) {
    state.setVariablePosition(j1, q1);
    state.setVariablePosition(j2, q2);
    state.update();
  };

  // --- closed configuration (both prismatic joints at 0) ---
  setFingers(0.0, 0.0);
  std::size_t nf = 0, nw = 0;
  Box3 f1 = aabbInFrame(state, *model, {fingers[0]}, m.tcp_frame, &nf);
  Box3 f2 = aabbInFrame(state, *model, {fingers[1]}, m.tcp_frame, &nf);
  Box3 fall = aabbInFrame(state, *model, fingers, m.tcp_frame, nullptr);
  const Box3 wbox = aabbInFrame(state, *model, wrist, m.tcp_frame, &nw);

  if (!fall.valid() || !f1.valid() || !f2.valid()) {
    throw GraspConfigError("measureGripper: '" + prefix +
                           "' finger links have no usable collision geometry "
                           "(every later collision query would silently answer 'no collision')");
  }
  if (!wbox.valid()) {
    throw GraspConfigError("measureGripper: '" + m.wrist_link + "' has no collision geometry");
  }
  m.n_finger_shapes = nf;
  m.n_wrist_shapes = nw;
  m.fingers_min_tcp = fall.lo;
  m.fingers_max_tcp = fall.hi;
  m.wrist_min_tcp = wbox.lo;
  m.wrist_max_tcp = wbox.hi;
  m.pad_z_near_m = fall.lo.z();
  m.pad_z_far_m = fall.hi.z();
  m.closed_pad_gap_m = gapAlongX(f1, f2);

  // --- fully open configuration ---
  setFingers(stroke1, stroke2);
  const Box3 o1 = aabbInFrame(state, *model, {fingers[0]}, m.tcp_frame, nullptr);
  const Box3 o2 = aabbInFrame(state, *model, {fingers[1]}, m.tcp_frame, nullptr);
  m.open_pad_gap_m = gapAlongX(o1, o2);

  state.setToDefaultValues();
  return m;
}

}  // namespace dual_arm_transport
