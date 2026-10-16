// src/baseline.cpp -- W7/W8 implementation.
#include "dual_arm_transport/baseline.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "dual_arm_kinematics/closure_residual.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_transport {
namespace {

using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;

std::string f6(double v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(9) << v;
  return os.str();
}

std::string mat4ToString(const Iso3& T) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(12);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (r < 3 || c < 3) os << T(r, c);
      else if (c == 3) os << 1.0;
      os << (c == 3 ? (r == 3 ? "" : ";") : ",");
    }
  }
  return os.str();
}

Eigen::Matrix4d mat4FromString(const std::string& s) {
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  std::stringstream ss(s);
  std::string row;
  int r = 0;
  while (std::getline(ss, row, ';') && r < 4) {
    std::stringstream rs(row);
    std::string cell;
    int c = 0;
    while (std::getline(rs, cell, ',') && c < 4) {
      if (!cell.empty()) M(r, c) = std::stod(cell);
      ++c;
    }
    ++r;
  }
  return M;
}

Iso3 iso3FromString(const std::string& s) { return Iso3(mat4FromString(s)); }

std::string vecToString(const Eigen::VectorXd& v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(12);
  for (Eigen::Index i = 0; i < v.size(); ++i) os << (i ? "," : "") << v[i];
  return os.str();
}

Eigen::VectorXd vecFromString(const std::string& s, Eigen::Index n) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero(n);
  std::stringstream ss(s);
  std::string cell;
  Eigen::Index i = 0;
  while (std::getline(ss, cell, ',') && i < n) {
    if (!cell.empty()) v[i] = std::stod(cell);
    ++i;
  }
  return v;
}

/// Accept BOTH forms the artifact may carry: the `[a, b, ...]` YAML sequence the
/// writer emits, and a legacy comma-separated scalar string.  Reading only the
/// scalar form made every written baseline unreadable ("bad conversion").
Eigen::VectorXd vecFromNode(const YAML::Node& n, Eigen::Index expected) {
  Eigen::VectorXd v;
  if (n.IsSequence()) {
    v.resize(static_cast<Eigen::Index>(n.size()));
    for (std::size_t k = 0; k < n.size(); ++k) v(static_cast<Eigen::Index>(k)) = n[k].as<double>();
  } else {
    v = vecFromString(n.as<std::string>(), expected);
  }
  if (expected > 0 && v.size() != expected) {
    throw GraspConfigError("vecFromNode: expected " + std::to_string(expected) + " values, got " +
                           std::to_string(v.size()));
  }
  return v;
}

std::string segToYaml(const std::string& indent, const std::string& key,
                      const SegmentResult& s) {
  std::ostringstream os;
  os << indent << key << ":\n";
  os << indent << "  ok: " << (s.ok ? "true" : "false") << "\n";
  os << indent << "  code: \"" << s.code << "\"\n";
  os << indent << "  detail: \"" << s.detail << "\"\n";
  os << indent << "  samples: " << s.n_samples << "\n";
  os << indent << "  first_bad_sample: " << s.first_bad_sample << "\n";
  os << indent << "  min_clearance_m: " << f6(s.min_clearance_m) << "\n";
  os << indent << "  max_line_dev_m: " << f6(s.max_line_dev_m) << "\n";
  os << indent << "  max_angle_dev_rad: " << f6(s.max_angle_dev_rad) << "\n";
  os << indent << "  q_bad: [" << vecToString(s.q_bad) << "]\n";
  return os.str();
}

SegmentResult segFromYaml(const YAML::Node& n) {
  SegmentResult s;
  if (!n) return s;
  s.ok = n["ok"] && n["ok"].as<bool>();
  s.code = n["code"] ? n["code"].as<std::string>() : "unknown";
  s.detail = n["detail"] ? n["detail"].as<std::string>() : "";
  s.n_samples = n["samples"] ? n["samples"].as<std::size_t>() : 0;
  s.first_bad_sample = n["first_bad_sample"] ? n["first_bad_sample"].as<int>() : -1;
  s.min_clearance_m = n["min_clearance_m"] ? n["min_clearance_m"].as<double>() : 0.0;
  s.max_line_dev_m = n["max_line_dev_m"] ? n["max_line_dev_m"].as<double>() : 0.0;
  s.max_angle_dev_rad = n["max_angle_dev_rad"] ? n["max_angle_dev_rad"].as<double>() : 0.0;
  if (n["q_bad"]) {
    s.q_bad = vecFromNode(n["q_bad"], 12);
  } else {
    s.q_bad = Q12::Zero();
  }
  return s;
}

}  // namespace

Q12 c42M4HandoverQ12() {
  Q12 q;
  q << 0.3353, -1.5739, -0.9095, -2.2290, 1.5708, -1.2355,      // left  (C4.2 M4 A6)
      2.28189, -1.90764, 1.82378, -0.0225579, 0.718457, -1.46959;  // right
  return q;
}

// ---------------------------------------------------------------- solver ----

BaselineSolver::BaselineSolver(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                               const GraspConfig& cfg)
    : scene_(std::move(scene)), cfg_(cfg) {
  if (scene_ == nullptr) throw GraspConfigError("BaselineSolver: null scene");
}

std::vector<StandbyCandidate> BaselineSolver::searchStandby() { return searchStandbyFor(cfg_); }

std::vector<StandbyCandidate> BaselineSolver::searchStandbyFor(const GraspConfig& cfg,
                                                                const dual_arm::Q12* seed) {
  std::vector<StandbyCandidate> out;
  const dual_arm::DualArmKinematics kin;
  const DualGraspTargets g = graspTargets(cfg);

  dual_arm_pose_optimization::EnvironmentOptions env;
  env.add_floor = true;
  dual_arm_pose_optimization::MoveItCollisionBackend backend(scene_, env);

  // Standby candidates are the grasp TCP poses RETREATED along -z_tcp.  They are
  // derived here for C4.3 and are deliberately NOT C4.2's A6 (user m02331 D3).
  for (const double d : {0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.22, 0.28}) {
    StandbyCandidate c;
    std::ostringstream id;
    id << "c43_standby_retreat_" << std::fixed << std::setprecision(2) << d;
    c.id = id.str();
    Q12 q = Q12::Zero();
    bool ok = true;
    for (int side = 0; side < 2; ++side) {
      const dual_arm::Arm a = (side == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
      Iso3 target = (side == 0) ? g.world_T_left_tcp : g.world_T_right_tcp;
      target.translation() += target.linear() * Eigen::Vector3d(0.0, 0.0, -d);
      const Iso3 target_base = kin.worldTbase(a).inverse() * target;
      const fairino_planning::IKResult res = ik_layer_.solves(target_base.matrix());
      if (!res.success || res.solutions.empty()) {
        c.code = "ik_failed";
        c.detail = std::string("standby IK failed for arm ") + (side == 0 ? "left" : "right") +
                   " at retreat " + std::to_string(d) + " (" + res.failure_code + ")";
        ok = false;
        break;
      }
      dual_arm::JointConfig pick = res.solutions.front();
      if (seed != nullptr) {
        const dual_arm::JointConfig want = seed->segment<6>(side * 6);
        double best = std::numeric_limits<double>::infinity();
        for (const auto& s : res.solutions) {
          const double dd = (s - want).norm();
          if (dd < best) {
            best = dd;
            pick = s;
          }
        }
      }
      q.segment<6>(side * 6) = pick;
    }
    if (!ok) {
      out.push_back(c);
      continue;
    }
    const moveit::core::RobotState st =
        scene_->makeStateQ12(std::vector<double>(q.data(), q.data() + 12));
    const auto reading = backend.distances(st, &q);
    c.q12 = q;
    c.min_clearance_m = reading.whole_robot;
    std::ostringstream dbg;
    dbg << "whole=" << f6(reading.whole_robot) << " arm_arm=" << f6(reading.arm_arm)
        << " arm_prox=" << f6(reading.arm_proximal) << " hit=" << reading.whole_link_a << "<->"
        << reading.whole_link_b;
    if (reading.in_collision) {
      c.code = "collision";
      c.detail = dbg.str();
    } else if (reading.whole_robot < 0.002) {
      c.code = "insufficient_clearance";
      c.detail = dbg.str();
    } else {
      c.valid = true;
      c.code = "ok";
      c.detail = dbg.str();
    }
    out.push_back(c);
  }
  return out;
}

TransportBaseline BaselineSolver::solve() {
  TransportBaseline b;
  b.provenance = provenance_;
  b.object_frame = cfg_.grasp.object_frame;
  b.size_xyz = cfg_.grasp.size_xyz;
  b.mass = cfg_.grasp.mass;
  b.d_approach_m = cfg_.d_approach_m;

  // ---- D4: neither the support height nor the exact sample placement may be
  // ASSUMED.  Both are swept, and the screened result is what gets recorded.
  // The sweep is ordered so the most balanced placement is tried first.
  const double x0 = cfg_.grasp.world_T_object_star.translation().x();
  const double z0 = cfg_.grasp.world_T_object_star.translation().z();
  const double half_z = cfg_.grasp.size_xyz.z() / 2.0;
  std::vector<double> xs;
  for (double dx : {0.0, -0.025, 0.025, -0.05, 0.05, -0.075}) xs.push_back(x0 + dx);
  std::vector<double> zs;
  for (double dz : {0.0, 0.025, 0.05, 0.075, 0.10, 0.125, 0.15, 0.20, 0.25}) {
    zs.push_back(z0 + dz);
  }

  const dual_arm::DualArmKinematics kin;
  PathChecker paths(scene_, cfg_);

  TransportBaseline best;
  bool have = false;

  for (const double x : xs) {
    for (const double z : zs) {
      GraspConfig cfg = cfg_;
      cfg.grasp.world_T_object_star.translation().x() = x;
      cfg.grasp.world_T_object_star.translation().z() = z;

      PairScreener screener(scene_, cfg);
      screener.applyWorld(cfg.grasp.world_T_object_star);
      const DualGraspTargets targets = graspTargets(cfg);
      const BranchSet left = enumerateBranches(cfg, kin, targets.world_T_left_tcp, "left");
      const BranchSet right = enumerateBranches(cfg, kin, targets.world_T_right_tcp, "right");
      const ScreenReport rep = screener.screen(left, right);
      last_screen_ = rep;

      std::ostringstream log;
      log << "placement x=" << f6(x) << " z=" << f6(z) << " pairs=" << rep.n_pairs
          << " feasible=" << rep.n_feasible << " ik=(" << left.solutions.size() << ","
          << right.solutions.size() << ")";
      if (left.solutions.empty()) log << " left_fail=" << left.failure_stage << "/" << left.failure_code;
      if (right.solutions.empty()) log << " right_fail=" << right.failure_stage << "/" << right.failure_code;
      for (const auto& kv : rep.reject_histogram) log << " " << kv.first << "=" << kv.second;
      b.placement_log.push_back(log.str());
      last_placement_log_ = b.placement_log;

      if (rep.n_feasible == 0) continue;

      // Rank the feasible pairs by clearance, then by conditioning, and
      // path-check them in that order.
      std::vector<const PairCandidate*> feas;
      for (const auto& c : rep.candidates) {
        if (c.feasible) feas.push_back(&c);
      }
      std::sort(feas.begin(), feas.end(), [](const PairCandidate* a, const PairCandidate* c) {
        if (a->min_clearance_m != c->min_clearance_m) return a->min_clearance_m > c->min_clearance_m;
        return a->sigma_min > c->sigma_min;
      });

      bool placed = false;
      for (std::size_t k = 0; k < feas.size() && k < 8; ++k) {
        const PairCandidate& c = *feas[k];

        // ---- W7 standby for THIS candidate (not C4.2's A6, user m02331 D3) ----
        // Seeded with the candidate's own grasp configuration so the standby and
        // the grasp share one IK branch.
        const std::vector<StandbyCandidate> standby = searchStandbyFor(cfg, &c.q12);
        std::string standby_id;
        Q12 q_standby = Q12::Zero();
        for (const auto& sc : standby) {
          if (sc.valid) {
            q_standby = sc.q12;
            standby_id = sc.id;
            break;
          }
        }
        if (standby_id.empty()) {
          b.placement_log.back() += " | pair(" + std::to_string(c.left_index) + "," +
                                    std::to_string(c.right_index) + ") no collision-free standby";
          continue;
        }

        const ReachabilityResult r = paths.check(q_standby, c.q12, tcpPairOf(targets));
        if (!r.ok) {
          b.placement_log.back() += " | pair(" + std::to_string(c.left_index) + "," +
                                    std::to_string(c.right_index) + ") path:" + r.code;
          continue;
        }

        best = b;
        best.world_T_object = cfg.grasp.world_T_object_star;
        best.support = cfg.support;
        best.support.top_z_m = z - half_z;
        best.support.top_z_status = "screened";
        best.q_standby = q_standby;
        best.q_standby_id = standby_id;
        best.standby_candidates = standby;
        best.left_index = c.left_index;
        best.right_index = c.right_index;
        best.q_grasp = c.q12;
        best.grasp_thickness_m = cfg.grip_thickness_m;
        best.left_tcp_frame = cfg.grasp.left_tcp_frame;
        best.right_tcp_frame = cfg.grasp.right_tcp_frame;
        best.world_T_left_tcp = targets.world_T_left_tcp;
        best.world_T_right_tcp = targets.world_T_right_tcp;
        best.object_T_left_tcp = cfg.grasp.object_T_left_tcp;
        best.object_T_right_tcp = cfg.grasp.object_T_right_tcp;
        best.clearance_m = c.min_clearance_m;
        best.sigma_min = c.sigma_min;
        best.anchor_pos_gap_m = c.anchor_pos_gap_m;
        best.anchor_rot_gap_rad = c.anchor_rot_gap_rad;
        best.closure_pos_m = c.closure_pos_m;
        best.closure_rot_rad = c.closure_rot_rad;
        best.legacy_midpoint_err_pos_m = c.midpoint_vs_task_pos_m;
        best.legacy_midpoint_err_rot_rad = c.midpoint_vs_task_rot_rad;
        best.standby_segment = r.standby_segment;
        best.approach_segment = r.approach_segment;
        best.n_pairs = rep.n_pairs;
        best.n_feasible = rep.n_feasible;
        best.reject_histogram = rep.reject_histogram;
        have = true;
        placed = true;
        break;
      }
      if (placed) return best;
    }
  }

  if (!have) {
    throw GraspConfigError(
        "BaselineSolver: screening found no feasible C4.3 transport baseline; "
        "refusing to write an empty baseline");
  }
  return best;
}

// ------------------------------------------------------------- W8: YAML ----

void writeBaseline(const std::string& path, const TransportBaseline& b) {
  std::ofstream os(path);
  if (!os) throw GraspConfigError("writeBaseline: cannot open " + path);
  os << b.toYaml();
}

std::string TransportBaseline::toYaml() const {
  std::ostringstream os;
  os << "# " << provenance << "\n";
  // Also written as a real KEY: the comment above is for humans, but the artifact
  // must survive a round trip, and a comment is not data.
  os << "provenance: \"" << provenance << "\"\n";
  os << "schema: \"" << schema << "\"\n";
  os << "stage: \"" << stage << "\"\n";
  os << "object:\n";
  os << "  frame: \"" << object_frame << "\"\n";
  os << "  size_xyz: [" << f6(size_xyz.x()) << ", " << f6(size_xyz.y()) << ", "
     << f6(size_xyz.z()) << "]\n";
  os << "  mass: " << f6(mass) << "\n";
  os << "  world_T_object: \"" << mat4ToString(world_T_object) << "\"\n";
  os << "support:\n";
  os << "  id: \"" << support.id << "\"\n";
  os << "  shape: \"" << support.shape << "\"\n";
  os << "  radius_m: " << f6(support.radius_m) << "\n";
  os << "  height_m: " << f6(support.height_m) << "\n";
  os << "  top_z_m: " << f6(support.top_z_m) << "\n";
  os << "  top_z_status: \"" << support.top_z_status << "\"\n";
  os << "standby:\n";
  os << "  id: \"" << q_standby_id << "\"\n";
  os << "  source: \"C4.3 independent standby search (not C4.2 A6)\"\n";
  os << "  q12: [" << vecToString(q_standby) << "]\n";
  os << "  candidates:\n";
  for (const auto& c : standby_candidates) {
    os << "    - id: \"" << c.id << "\"\n";
    os << "      code: \"" << c.code << "\"\n";
    os << "      valid: " << (c.valid ? "true" : "false") << "\n";
    os << "      min_clearance_m: " << f6(c.min_clearance_m) << "\n";
    os << "      detail: \"" << c.detail << "\"\n";
  }
  os << "grasp:\n";
  os << "  left_index: " << left_index << "\n";
  os << "  right_index: " << right_index << "\n";
  os << "  grip_thickness_m: " << f6(grasp_thickness_m) << "\n";
  os << "  q_grasp12: [" << vecToString(q_grasp) << "]\n";
  os << "  frames:\n";
  os << "    left_tcp: \"" << left_tcp_frame << "\"\n";
  os << "    right_tcp: \"" << right_tcp_frame << "\"\n";
  os << "  world_T_left_tcp: \"" << mat4ToString(world_T_left_tcp) << "\"\n";
  os << "  world_T_right_tcp: \"" << mat4ToString(world_T_right_tcp) << "\"\n";
  os << "  object_T_left_tcp: \"" << mat4ToString(object_T_left_tcp) << "\"\n";
  os << "  object_T_right_tcp: \"" << mat4ToString(object_T_right_tcp) << "\"\n";
  os << "  d_approach_m: " << f6(d_approach_m) << "\n";
  os << "measured:\n";
  os << "  clearance_m: " << f6(clearance_m) << "\n";
  os << "  sigma_min: " << f6(sigma_min) << "\n";
  os << "  anchor_pos_gap_m: " << f6(anchor_pos_gap_m) << "\n";
  os << "  anchor_rot_gap_rad: " << f6(anchor_rot_gap_rad) << "\n";
  os << "  closure_pos_m: " << f6(closure_pos_m) << "\n";
  os << "  closure_rot_rad: " << f6(closure_rot_rad) << "\n";
  os << "  legacy_midpoint_err_pos_m: " << f6(legacy_midpoint_err_pos_m) << "\n";
  os << "  legacy_midpoint_err_rot_rad: " << f6(legacy_midpoint_err_rot_rad) << "\n";
  os << "search:\n";
  os << "  n_pairs: " << n_pairs << "\n";
  os << "  n_feasible: " << n_feasible << "\n";
  os << "  reject_histogram:\n";
  for (const auto& kv : reject_histogram) {
    os << "    " << kv.first << ": " << kv.second << "\n";
  }
  os << "reachability:\n";
  os << segToYaml("  ", "standby_segment", standby_segment);
  os << segToYaml("  ", "approach_segment", approach_segment);
  os << "placement_log:\n";
  if (placement_log.empty()) {
    os << "  []\n";
  } else {
    for (const auto& l : placement_log) os << "  - \"" << l << "\"\n";
  }
  os << "not_covered:\n";
  for (const auto& s : not_covered) os << "  - \"" << s << "\"\n";
  return os.str();
}

TransportBaseline TransportBaseline::fromYaml(const std::string& text) {
  TransportBaseline b;
  const YAML::Node n = YAML::Load(text);
  if (n["schema"]) b.schema = n["schema"].as<std::string>();
  if (n["stage"]) b.stage = n["stage"].as<std::string>();
  if (n["provenance"]) b.provenance = n["provenance"].as<std::string>();
  if (n["object"]) {
    const YAML::Node o = n["object"];
    if (o["frame"]) b.object_frame = o["frame"].as<std::string>();
    if (o["size_xyz"]) {
      const auto v = o["size_xyz"].as<std::vector<double>>();
      if (v.size() == 3) b.size_xyz = Eigen::Vector3d(v[0], v[1], v[2]);
    }
    if (o["mass"]) b.mass = o["mass"].as<double>();
    if (o["world_T_object"]) {
      b.world_T_object = iso3FromString(o["world_T_object"].as<std::string>());
    }
  }
  if (n["support"]) {
    const YAML::Node s = n["support"];
    if (s["id"]) b.support.id = s["id"].as<std::string>();
    if (s["shape"]) b.support.shape = s["shape"].as<std::string>();
    if (s["radius_m"]) b.support.radius_m = s["radius_m"].as<double>();
    if (s["height_m"]) b.support.height_m = s["height_m"].as<double>();
    if (s["top_z_m"]) b.support.top_z_m = s["top_z_m"].as<double>();
    if (s["top_z_status"]) b.support.top_z_status = s["top_z_status"].as<std::string>();
  }
  if (n["standby"]) {
    const YAML::Node s = n["standby"];
    if (s["id"]) b.q_standby_id = s["id"].as<std::string>();
    if (s["q12"]) b.q_standby = vecFromNode(s["q12"], 12);
    if (s["candidates"]) {
      for (const auto& c : s["candidates"]) {
        StandbyCandidate sc;
        if (c["id"]) sc.id = c["id"].as<std::string>();
        if (c["code"]) sc.code = c["code"].as<std::string>();
        if (c["valid"]) sc.valid = c["valid"].as<bool>();
        if (c["min_clearance_m"]) sc.min_clearance_m = c["min_clearance_m"].as<double>();
        if (c["detail"]) sc.detail = c["detail"].as<std::string>();
        b.standby_candidates.push_back(sc);
      }
    }
  }
  if (n["grasp"]) {
    const YAML::Node g = n["grasp"];
    if (g["left_index"]) b.left_index = g["left_index"].as<int>();
    if (g["right_index"]) b.right_index = g["right_index"].as<int>();
    if (g["q_grasp12"]) b.q_grasp = vecFromNode(g["q_grasp12"], 12);
    if (g["d_approach_m"]) b.d_approach_m = g["d_approach_m"].as<double>();
    if (g["grip_thickness_m"]) b.grasp_thickness_m = g["grip_thickness_m"].as<double>();
    if (g["frames"]) {
      const YAML::Node fr = g["frames"];
      if (fr["left_tcp"]) b.left_tcp_frame = fr["left_tcp"].as<std::string>();
      if (fr["right_tcp"]) b.right_tcp_frame = fr["right_tcp"].as<std::string>();
    }
    if (g["world_T_left_tcp"])
      b.world_T_left_tcp = iso3FromString(g["world_T_left_tcp"].as<std::string>());
    if (g["world_T_right_tcp"])
      b.world_T_right_tcp = iso3FromString(g["world_T_right_tcp"].as<std::string>());
    if (g["object_T_left_tcp"])
      b.object_T_left_tcp = iso3FromString(g["object_T_left_tcp"].as<std::string>());
    if (g["object_T_right_tcp"])
      b.object_T_right_tcp = iso3FromString(g["object_T_right_tcp"].as<std::string>());
  }
  if (n["measured"]) {
    const YAML::Node m = n["measured"];
    auto get = [&](const char* k, double* dst) {
      if (m[k]) *dst = m[k].as<double>();
    };
    get("clearance_m", &b.clearance_m);
    get("sigma_min", &b.sigma_min);
    get("anchor_pos_gap_m", &b.anchor_pos_gap_m);
    get("anchor_rot_gap_rad", &b.anchor_rot_gap_rad);
    get("closure_pos_m", &b.closure_pos_m);
    get("closure_rot_rad", &b.closure_rot_rad);
    get("legacy_midpoint_err_pos_m", &b.legacy_midpoint_err_pos_m);
    get("legacy_midpoint_err_rot_rad", &b.legacy_midpoint_err_rot_rad);
  }
  if (n["search"]) {
    const YAML::Node s = n["search"];
    if (s["n_pairs"]) b.n_pairs = s["n_pairs"].as<int>();
    if (s["n_feasible"]) b.n_feasible = s["n_feasible"].as<int>();
    if (s["reject_histogram"]) {
      for (const auto& kv : s["reject_histogram"]) {
        b.reject_histogram[kv.first.as<std::string>()] = kv.second.as<int>();
      }
    }
  }
  if (n["reachability"]) {
    b.standby_segment = segFromYaml(n["reachability"]["standby_segment"]);
    b.approach_segment = segFromYaml(n["reachability"]["approach_segment"]);
  }
  b.placement_log.clear();
  if (n["placement_log"]) {
    for (const auto& s2 : n["placement_log"]) b.placement_log.push_back(s2.as<std::string>());
  }
  b.not_covered.clear();
  if (n["not_covered"]) {
    for (const auto& s : n["not_covered"]) b.not_covered.push_back(s.as<std::string>());
  }
  return b;
}

TransportBaseline readBaseline(const std::string& path) {
  std::ifstream is(path);
  if (!is) throw GraspConfigError("readBaseline: cannot open " + path);
  std::stringstream ss;
  ss << is.rdbuf();
  return TransportBaseline::fromYaml(ss.str());
}

}  // namespace dual_arm_transport
