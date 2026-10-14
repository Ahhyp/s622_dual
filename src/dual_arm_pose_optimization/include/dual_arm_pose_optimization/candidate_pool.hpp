// include/dual_arm_pose_optimization/candidate_pool.hpp
// C4.2 / S3: candidate object poses, and the C4.1 sampling proxy for
// "TCP point reachable" (the S0 layer).
//
// C4.1 produced 2.78e6 accepted joint-space samples and recorded, per sample,
// the two TCP positions and the object-frame midpoint.  That file is the S0
// oracle: a point is "point reachable by arm A" iff some sample put arm A's TCP
// in that 2 cm voxel.
//
// Two pools are offered, deliberately different in character (plan §4.1):
//   P1 (handover pool) : object centres taken from C4.1 samples whose TCP pair
//                        is already close enough to hand over.  Point reachable
//                        BY CONSTRUCTION -> S0 is ~100% and the interesting
//                        ratios start at S1.
//   P3 (grid pool)     : a uniform grid over the CRW bounding box, NOT filtered
//                        by the mask.  Many points are not point reachable at
//                        all, which is what makes |S1|/|S0| informative and
//                        guards against P1's sampling bias.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace dual_arm_pose_optimization {

/// One C4.1 record, reduced to what S3 needs.
struct C41Sample {
  dual_arm::JointConfig q_left = dual_arm::JointConfig::Zero();
  Eigen::Vector3d p_left;
  Eigen::Vector3d p_right;
  Eigen::Vector3d p_object;
  double d_tcp = 0.0;
};

/// 2 cm voxel occupancy of the two arms' TCP sets, built from C4.1's records.
class WorkspaceIndex {
public:
  struct Options {
    std::string c41_bin;
    double voxel = 0.02;
    /// Handover samples kept for P1 (strided subsample); 0 = keep all.
    std::size_t max_handover_samples = 30000;
    double handover_max_d = 0.45;
  };

  static WorkspaceIndex load(const Options& options);

  bool leftHas(const Eigen::Vector3d& p) const;
  bool rightHas(const Eigen::Vector3d& p) const;
  bool dualHas(const Eigen::Vector3d& p) const;

  std::size_t countLeft() const { return n_left_; }
  std::size_t countRight() const { return n_right_; }
  std::size_t countDual() const { return n_dual_; }
  std::uint64_t totalRecords() const { return total_records_; }
  double voxel() const { return voxel_; }
  const Eigen::Vector3d& origin() const { return origin_; }
  const Eigen::Vector3d& bboxMin() const { return bbox_min_; }
  const Eigen::Vector3d& bboxMax() const { return bbox_max_; }

  const std::vector<C41Sample>& handoverSamples() const { return handover_; }

  /// Uniform grid of candidate OBJECT CENTRES over the CRW bounding box.
  /// Not filtered by the mask (that is the point of this pool).
  std::vector<Eigen::Vector3d> gridObjectCentres(double step) const;

private:
  std::size_t index(int i, int j, int k) const;
  bool has(const std::vector<std::uint8_t>& m, const Eigen::Vector3d& p) const;

  double voxel_ = 0.02;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bbox_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bbox_max_ = Eigen::Vector3d::Zero();
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<std::uint8_t> left_, right_;
  std::size_t n_left_ = 0, n_right_ = 0, n_dual_ = 0;
  std::uint64_t total_records_ = 0;
  std::vector<C41Sample> handover_;
};

}  // namespace dual_arm_pose_optimization
