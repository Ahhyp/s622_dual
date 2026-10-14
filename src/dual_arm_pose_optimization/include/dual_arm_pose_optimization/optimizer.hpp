// include/dual_arm_pose_optimization/optimizer.hpp
// C4.2 / S5: the pose refiner (V0 / V1) on top of the S3/S4 candidate table.
//
// V0 : optimise the object centre p_O with the object orientation FIXED at the
//      M4 value                       -> "keep M4's handover direction, pick a
//                                          better handover position"
// V1 : p_O + a rotation-vector perturbation about R_O^M4, constrained by its
//      EUCLIDEAN NORM:   R_O = R_O^M4 * exp([dtheta]x),  ||dtheta||_2 <= theta_max
//      -> "additionally allow a small attitude adjustment"
//
// The norm constraint is the one agreed for C4.2 (plan §3.2): it is NOT the
// per-axis |dtheta_i| <= 10 deg box, which would allow up to sqrt(3)*10 deg.
//
// Every objective is normalised to [0,1] against a scale that is DECLARED up
// front (passed in from the S4 analysis), then weighted.  Infeasible poses are
// not discarded silently: they get a layered penalty so the simplex can walk
// back into the feasible set.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_pose_optimization/candidate_pool.hpp"
#include "dual_arm_pose_optimization/feasibility.hpp"

namespace dual_arm_pose_optimization {

/// The eight objectives, in the S4 order.
struct ObjectiveSpec {
  /// Normalisation scales, in the S4 order
  /// J1 smin_left, J2 smin_right, J3 w_rel, J4 arm_arm,
  /// J5 joint_margin, J6 deviation, J7 continuity, J8 arm_proximal.
  double scale[8] = {0.153739, 0.143295, 1.151829, 0.05,
                     0.5,      1.053581, 1.0,      0.05};
  double weight[8] = {1, 1, 1, 1, 1, 1, 1, 1};

  static ObjectiveSpec named(const std::string& set);
};

/// Normalised objective values of a result (order as above).
Eigen::Matrix<double, 8, 1> normalisedObjectives(const CandidateResult& r,
                                                 const ObjectiveSpec& spec);

/// Weighted score in [0,1] for feasible poses; negative (layered penalty) for
/// infeasible ones.  Never NaN.
double scoreResult(const CandidateResult& r, const ObjectiveSpec& spec);

/// A pose parameterisation of the object frame.
struct PoseParams {
  Eigen::Vector3d p_O = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotvec = Eigen::Vector3d::Zero();  ///< about R_O^M4
};

struct RefineOptions {
  int max_evals = 600;
  double tol_x = 1e-5;   ///< simplex size tolerance [m / rad]
  double tol_f = 1e-7;
  double theta_max = 10.0 * M_PI / 180.0;
  bool allow_orientation = false;  ///< false -> V0
};

struct RefineResult {
  PoseParams best;
  double score = -1e9;
  bool feasible = false;
  int evals = 0;
  CandidateResult result;
};

/// Nelder-Mead with an optional projection onto the rotation-vector ball.
class PoseRefiner {
public:
  PoseRefiner(FeasibilityPipeline* pipeline, const WorkspaceIndex* index,
              const ObjectiveSpec& spec, const RefineOptions& options);

  /// `R_O_base` is the M4 object orientation (R_O^M4).
  RefineResult refine(const PoseParams& start, const Eigen::Matrix3d& R_O_base);

  const ObjectiveSpec& spec() const { return spec_; }
  const RefineOptions& options() const { return opts_; }
  int totalEvals() const { return total_evals_; }

private:
  double evaluate(const PoseParams& p, const Eigen::Matrix3d& R_O_base,
                  CandidateResult* out);
  void project(PoseParams* p) const;

  FeasibilityPipeline* pipeline_;
  const WorkspaceIndex* index_;
  ObjectiveSpec spec_;
  RefineOptions opts_;
  int total_evals_ = 0;
};

/// Joint-space straight-line feasibility, the S5 (Phase 1) proxy for motion
/// planning.  Reported as "segment feasibility", never as "planning success".
struct SegmentReport {
  double success_rate = 0.0;
  std::size_t tested = 0;
  std::size_t feasible = 0;
  int samples = 32;
};

}  // namespace dual_arm_pose_optimization
