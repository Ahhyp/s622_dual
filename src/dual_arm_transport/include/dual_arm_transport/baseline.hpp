// include/dual_arm_transport/baseline.hpp
// C4.3 S2 / W7 + W8 -- the independent C4.3 `transport_baseline`.
//
// This baseline is NOT a copy of C4.2's A6.  It is produced by an explicit
// search (standby candidate -> grasp branches -> cartesian product -> collision
// and reachability screening) and it RECORDS what the search actually measured,
// including the rejection histogram and the legacy-midpoint error, so the
// record can defend why the pair was chosen (user m02331 执行要求).
//
// What it deliberately does NOT contain: any post-grasp transport trajectory.
// A joint interpolation between two grasp configurations does not satisfy a
// fixed-grasp constraint and must not be presented as a transport path
// (user m02331 D7).
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"
#include "dual_arm_transport/pair_screen.hpp"
#include "dual_arm_transport/path_check.hpp"

namespace dual_arm_transport {

struct StandbyCandidate {
  std::string id;
  dual_arm::Q12 q12 = dual_arm::Q12::Zero();
  bool valid = false;
  std::string code;    ///< "ok" | "ik_failed" | "collision" | ...
  std::string detail;
  double min_clearance_m = 0.0;
};

struct TransportBaseline {
  std::string schema{"c43_s2_transport_baseline/v1"};
  std::string stage{"C4.3-S2"};
  std::string provenance;

  // ---- object + support ----
  std::string object_frame;
  dual_arm::Iso3 world_T_object = dual_arm::Iso3::Identity();
  dual_arm::Vec3 size_xyz = dual_arm::Vec3::Zero();
  double mass = 0.0;
  SupportSpec support;

  // ---- standby (S2-T8): chosen independently, never C4.2's A6 ----
  dual_arm::Q12 q_standby = dual_arm::Q12::Zero();
  std::string q_standby_id;
  std::vector<StandbyCandidate> standby_candidates;

  // ---- the selected grasp pair ----
  int left_index = -1;
  int right_index = -1;
  dual_arm::Q12 q_grasp = dual_arm::Q12::Zero();
  double grasp_thickness_m = 0.0;
  std::string left_tcp_frame;
  std::string right_tcp_frame;
  dual_arm::Iso3 world_T_left_tcp = dual_arm::Iso3::Identity();
  dual_arm::Iso3 world_T_right_tcp = dual_arm::Iso3::Identity();
  dual_arm::Iso3 object_T_left_tcp = dual_arm::Iso3::Identity();
  dual_arm::Iso3 object_T_right_tcp = dual_arm::Iso3::Identity();
  double clearance_m = 0.0;
  double sigma_min = 0.0;
  double anchor_pos_gap_m = 0.0;
  double anchor_rot_gap_rad = 0.0;
  double closure_pos_m = 0.0;
  double closure_rot_rad = 0.0;

  // ---- the defense against the legacy midpoint (S1 F-S1-1 / S2-T7) ----
  double legacy_midpoint_err_pos_m = 0.0;
  double legacy_midpoint_err_rot_rad = 0.0;

  // ---- reachability (S2-T14/T15) ----
  SegmentResult standby_segment;
  SegmentResult approach_segment;
  double d_approach_m = 0.08;

  // ---- search provenance ----
  int n_pairs = 0;
  int n_feasible = 0;
  std::map<std::string, int> reject_histogram;
  /// One line per (x, z) placement swept, in the order tried (D4).
  std::vector<std::string> placement_log;

  // ---- explicit scope exclusions ----
  std::vector<std::string> not_covered{
      "post-grasp transport trajectory (S3)",
      "grasp force / contact dynamics",
      "real-hardware execution",
  };

  std::string toYaml() const;
  static TransportBaseline fromYaml(const std::string& text);
};

/// Runs the whole W2->W6 chain and produces the baseline (W7).
class BaselineSolver {
public:
  BaselineSolver(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
                 const GraspConfig& cfg);

  /// Standby search for the configured object placement (S2-T8).
  std::vector<StandbyCandidate> searchStandby();
  /// Same, for an explicit placement (the sweep re-targets the object).
  /// Standby candidates for `cfg`.  When `seed` is given, each arm's IK solution
  /// is chosen to be the one CLOSEST to that arm's seed configuration, so the
  /// standby lands on the same IK branch as the grasp it belongs to.  Without
  /// this the standby can sit on an elbow-flip branch and the joint-space motion
  /// from standby to grasp sweeps the arm through a collision even though both
  /// endpoints are collision free.
  std::vector<StandbyCandidate> searchStandbyFor(const GraspConfig& cfg,
                                                 const dual_arm::Q12* seed = nullptr);

  /// Full pipeline.  Throws GraspConfigError when no pair is feasible -- an
  /// empty baseline must never be written as if it had succeeded.
  TransportBaseline solve();

  const ScreenReport& lastScreen() const { return last_screen_; }
  const std::vector<std::string>& lastPlacementLog() const { return last_placement_log_; }
  void setProvenance(const std::string& p) { provenance_ = p; }

private:
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene_;
  GraspConfig cfg_;
  dual_arm_pose_optimization::IkLayer ik_layer_;
  ScreenReport last_screen_;
  /// The placement log of the sweep as it was at the last iteration (kept as a
  /// member so a FAILED sweep can still explain itself -- solve() throws before
  /// it can return the baseline that would otherwise carry the log).
  std::vector<std::string> last_placement_log_;
  std::string provenance_;
};

/// W8: write/read the baseline as YAML.
void writeBaseline(const std::string& path, const TransportBaseline& b);
TransportBaseline readBaseline(const std::string& path);

/// R1 regression defense: the C4.2 M4 handover configuration, as recorded in
/// C4.2_S2_验收记录.md, must be rejected by this screen's closure criterion.
dual_arm::Q12 c42M4HandoverQ12();

}  // namespace dual_arm_transport
