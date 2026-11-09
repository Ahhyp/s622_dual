// C4.3 S4 / experiment 2.5-D3: OFFLINE X/Z offset scan of the left gripper against
// the 50 mm sample, using the same MoveIt/FCL scene as c43_path_check.
//
// For every (dx, dz) sample offset it reports
//   * onset q per finger  : bisected finger joint value at which that finger first
//                           touches the sample (whole-robot FCL distance == 0,
//                           the other finger held fully open so it cannot mask it)
//   * pad/face overlap    : intersection of the transformed finger mesh bounding
//                           box with the sample's +/-y face rectangle, in x and z
//   * overhang            : how far the pad sticks out past the +x end of the sample
//
// A symmetric face grasp shows onset_q1 == onset_q2 and a large overlap on both
// pads; a corner/edge contact shows up as an onset asymmetry and/or overhang.
//
// Read-only: builds an offline scene, never publishes or moves anything.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;

// q_grasp12, identical to the last row of c43_path_check's DESCENT table.
static const std::vector<double> Q_GRASP{
  2.795036910, -1.706950858, 1.279963524, -1.143808993, -1.570796327, -0.346555744,
  0.346555744, -1.434641796, -1.279963524, -1.997783660, 1.570796327, -2.795036910};

// Finger collision meshes are the STL bounding boxes, in the link frame, metres.
// Measured from finger1.STL / finger2.STL (32 triangles each).
struct MeshBox { double x0, x1, y0, y1, z0, z1; };
static const MeshBox PAD1{-0.01009, 0.02519, 0.02000, 0.06150, -0.01022, 0.01009};
static const MeshBox PAD2{-0.02519, 0.01009, 0.02000, 0.06150, -0.01009, 0.01022};

static const double SAMPLE_X = 0.120, SAMPLE_Y = 0.050, SAMPLE_Z = 0.050;
static const double SAMPLE_Z0 = 0.275;
// 2.5-C mesh fit: inner pad gap [mm] = 2000*q[m] - 0.18
static double gap_mm(double q) { return 2000.0 * std::fabs(q) - 0.18; }

struct Box {
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
  void add(const Eigen::Vector3d& p) {
    xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
    ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
    zmin = std::min(zmin, p.z()); zmax = std::max(zmax, p.z());
  }
};

static Box transformedBox(const moveit::core::RobotState& st, const std::string& link,
                          const MeshBox& m) {
  const Eigen::Isometry3d T = st.getGlobalLinkTransform(link);
  Box b;
  for (int i = 0; i < 8; ++i) {
    Eigen::Vector3d p((i & 1) ? m.x1 : m.x0, (i & 2) ? m.y1 : m.y0, (i & 4) ? m.z1 : m.z0);
    b.add(T * p);
  }
  return b;
}

static double overlap(double a0, double a1, double b0, double b1) {
  return std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
}

struct Row {
  double dx_mm, dz_mm;
  double q1_mm, q2_mm;       // |onset q| in mm; <0 => never touched
  double g1_mm, g2_mm;       // onset inner gap; <0 => never touched
  double asym_um;            // (q1 - q2) in um, positive => finger2 reaches later
  double c1x, c1z, c2x, c2z; // onset contact point in the SAMPLE frame [mm]
  double ov1_x, ov1_z, ov1_over;   // mm (whole-finger AABB; coarse, not the pad face)
  double ov2_x, ov2_z, ov2_over;
};

struct Scanner {
  std::shared_ptr<c42::DualArmScene> scene;
  std::unique_ptr<c42::MoveItCollisionBackend> backend;
  std::shared_ptr<collision_detection::World> world;

  explicit Scanner(const std::string& urdf, const std::string& srdf) {
    c42::SceneLoadOptions opt; opt.urdf_path = urdf; opt.srdf_path = srdf;
    scene = c42::DualArmScene::load(opt);
    c42::EnvironmentOptions env; env.add_floor = true; env.floor_top_z = 0.0;
    backend.reset(new c42::MoveItCollisionBackend(scene, env));
    world = scene->planningScene()->getWorldNonConst();
  }

  moveit::core::RobotState make(double q1, double q2) {
    auto st = scene->makeStateQ12(Q_GRASP);
    st.setVariablePosition("left_finger1_joint", q1);
    st.setVariablePosition("left_finger2_joint", q2);
    st.setVariablePosition("right_finger1_joint", 0.0305);
    st.setVariablePosition("right_finger2_joint", -0.0305);
    st.update();
    return st;
  }

  void placeSample(double dx, double dz) {
    world->removeObject("c43_sample");
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(dx, 0.0, SAMPLE_Z0 + dz);
    world->addToObject("c43_sample", std::make_shared<shapes::Box>(SAMPLE_X, SAMPLE_Y, SAMPLE_Z), p);
  }

  // Contact points (model frame) between `link` and the sample at one finger value.
  std::vector<Eigen::Vector3d> contactsAt(const std::string& link, bool positive, double mag) {
    const double q = positive ? mag : -mag;
    auto st = make(0.0305, -0.0305);
    st.setVariablePosition(link + "_joint", q);
    st.update();
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 20;
    collision_detection::CollisionResult res;
    scene->planningScene()->getCollisionEnv()->checkRobotCollision(
        req, res, st, backend->robotWorldAcm());
    std::vector<Eigen::Vector3d> out;
    static int dbg = 0;
    if (dbg < 3 && std::getenv("C43_SCAN_DEBUG")) {
      ++dbg;
      std::fprintf(stderr, "[dbg] contact_count=%zu pairs=%zu\n", res.contact_count, res.contacts.size());
      for (const auto& kv : res.contacts)
        for (const auto& c : kv.second)
          std::fprintf(stderr, "[dbg] '%s' | '%s' pos=(%.4f %.4f %.4f) d=%.5f\n",
                       c.body_name_1.c_str(), c.body_name_2.c_str(),
                       c.pos.x(), c.pos.y(), c.pos.z(), c.depth);
    }
    for (const auto& kv : res.contacts) {
      for (const auto& c : kv.second) {
        const bool touches_sample =
            (c.body_name_1 == "c43_sample") != (c.body_name_2 == "c43_sample");
        if (!touches_sample) continue;
        const std::string& other = (c.body_name_1 == "c43_sample") ? c.body_name_2 : c.body_name_1;
        if (other != link) continue;
        out.push_back(c.pos);
      }
    }
    return out;
  }

  // Smallest |q| at which `joint` touches the sample.  <0 when it never does.
  double onset(const std::string& joint, bool positive, double q_open, double q_closed) {
    auto dist = [&](double mag) {
      const double q = positive ? mag : -mag;
      auto st = make(q_open, -q_open);   // both fingers open
      st.setVariablePosition(joint, q);  // close only the probed one
      st.update();
      return backend->robotWorldDistance(st);
    };
    if (dist(q_open) <= 1e-9) return q_open;        // already touching when open
    if (dist(q_closed) > 1e-9) return -1.0;         // never reaches the sample
    double lo = q_open, hi = q_closed;              // lo: open, hi: touching
    for (int i = 0; i < 40; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (dist(mid) <= 1e-9) hi = mid; else lo = mid;
    }
    return 0.5 * (lo + hi);
  }

  Row scan(double dx_mm, double dz_mm) {
    const double dx = dx_mm / 1000.0, dz = dz_mm / 1000.0;
    placeSample(dx, dz);
    Row r{};
    r.dx_mm = dx_mm; r.dz_mm = dz_mm;
    const double q1 = onset("left_finger1_joint", true, 0.0305, 0.0145);
    const double q2 = onset("left_finger2_joint", false, 0.0305, 0.0145);
    r.q1_mm = q1 < 0 ? -1 : q1 * 1000.0;
    r.q2_mm = q2 < 0 ? -1 : q2 * 1000.0;
    r.g1_mm = q1 < 0 ? -1 : gap_mm(q1);
    r.g2_mm = q2 < 0 ? -1 : gap_mm(q2);
    r.asym_um = (q1 < 0 || q2 < 0) ? -1 : (q1 - q2) * 1e6;

    // Where the pad actually touches, in the SAMPLE frame (decisive for D3:
    // side face vs the +x end edge).
    auto centre = [&](const std::string& link, bool positive, double q, double& cx, double& cz) {
      cx = cz = 999.0;
      if (q < 0) return;
      const double probe = std::getenv("C43_SCAN_PROBE_Q")
                               ? std::atof(std::getenv("C43_SCAN_PROBE_Q")) : q - 0.0002;
      // Larger |q| = more OPEN, so "deeper" is q - margin.
      const auto pts = contactsAt(link, positive, probe > 0 ? probe : q - 0.0002);
      if (pts.empty()) return;
      Eigen::Vector3d m = Eigen::Vector3d::Zero();
      for (const auto& p : pts) m += p;
      m /= static_cast<double>(pts.size());
      cx = (m.x() - dx) * 1000.0;
      cz = (m.z() - (SAMPLE_Z0 + dz)) * 1000.0;
    };
    centre("left_finger1", true, q1, r.c1x, r.c1z);
    centre("left_finger2", false, q2, r.c2x, r.c2z);

    // Geometry at the nominal closing position (the C2 stop value ~0.025089).
    auto st = make(0.025089, -0.025089);
    const Box b1 = transformedBox(st, "left_finger1", PAD1);
    const Box b2 = transformedBox(st, "left_finger2", PAD2);
    const double sx0 = dx - SAMPLE_X / 2, sx1 = dx + SAMPLE_X / 2;
    const double sz0 = SAMPLE_Z0 + dz - SAMPLE_Z / 2, sz1 = SAMPLE_Z0 + dz + SAMPLE_Z / 2;
    r.ov1_x = overlap(b1.xmin, b1.xmax, sx0, sx1) * 1000.0;
    r.ov1_z = overlap(b1.zmin, b1.zmax, sz0, sz1) * 1000.0;
    r.ov1_over = (b1.xmax - sx1) * 1000.0;
    r.ov2_x = overlap(b2.xmin, b2.xmax, sx0, sx1) * 1000.0;
    r.ov2_z = overlap(b2.zmin, b2.zmax, sz0, sz1) * 1000.0;
    r.ov2_over = (b2.xmax - sx1) * 1000.0;
    return r;
  }
};

int main(int argc, char** argv) {
  std::string urdf = "/workspace/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "/workspace/src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string out = "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/grasp_scan_2_5d3.csv";
  std::string mode = "xz";
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto nxt = [&]() { if (i + 1 >= argc) std::exit(2); return std::string(argv[++i]); };
    if (a == "--urdf") urdf = nxt();
    else if (a == "--srdf") srdf = nxt();
    else if (a == "--out") out = nxt();
    else if (a == "--mode") mode = nxt();
    else { std::cerr << "unknown " << a << "\n"; return 2; }
  }
  Scanner sc(urdf, srdf);
  std::vector<Row> rows;
  if (mode == "x" || mode == "xz") {
    for (int dx = -25; dx <= 15; dx += 5) rows.push_back(sc.scan(dx, 0.0));
  }
  if (mode == "z" || mode == "xz") {
    for (int dz = -20; dz <= 20; dz += 5) rows.push_back(sc.scan(0.0, dz));
  }
  std::ofstream f(out);
  f << "dx_mm,dz_mm,q1_mm,q2_mm,g1_mm,g2_mm,asym_um,"
       "contact1_x_mm,contact1_z_mm,contact2_x_mm,contact2_z_mm\n";
  std::printf("%7s %7s | %8s %8s | %8s %8s | %9s | %10s %10s | %10s %10s\n",
              "dx_mm", "dz_mm", "q1_mm", "q2_mm", "g1_mm", "g2_mm", "asym_um",
              "c1_x", "c1_z", "c2_x", "c2_z");
  for (const auto& r : rows) {
    std::printf("%7.1f %7.1f | %8.4f %8.4f | %8.3f %8.3f | %9.2f | %10.2f %10.2f | %10.2f %10.2f\n",
                r.dx_mm, r.dz_mm, r.q1_mm, r.q2_mm, r.g1_mm, r.g2_mm, r.asym_um,
                r.c1x, r.c1z, r.c2x, r.c2z);
    f << r.dx_mm << "," << r.dz_mm << "," << r.q1_mm << "," << r.q2_mm << ","
      << r.g1_mm << "," << r.g2_mm << "," << r.asym_um << ","
      << r.c1x << "," << r.c1z << "," << r.c2x << "," << r.c2z << "\n";
  }
  std::printf("\nWROTE %s\n", out.c_str());
  return 0;
}
