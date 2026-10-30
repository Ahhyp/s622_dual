// C4.3 S4 S4-T16 -- pure grasp-relative check.
//
// The executor measures the sample relative to the left wrist from the
// Gazebo pose bridge and compares it with the FROZEN grasp chain
// (object_T_left_tcp, S2 baseline).  This file contains only the arithmetic:
// no ROS, no clock, no thresholds hidden in a caller.
//
// A "grasp" is NOT a weld: this check only says the sample is where the frozen
// grasp geometry says it should be.  The weld confirmation is a separate gate.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__GRASP_CHECK_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__GRASP_CHECK_HPP_

#include <Eigen/Geometry>

namespace dual_arm_transport_executor
{

struct GraspTolerances
{
  /// [待标定] plan §4.3: suggested 3 mm / 1 deg.  These are DEFAULTS for the
  /// Gazebo profile, not approved acceptance thresholds.
  double pos_m = 0.003;
  double rot_rad = 0.017453292519943295;  // 1 deg
};

struct GraspCheckResult
{
  bool ok = false;
  double pos_m = 0.0;
  double rot_rad = 0.0;
};

/// Compare two rigid transforms expressed in the SAME frame (here:
/// left_wrist3_link -> sample).  Rotation is the geodesic angle of
/// R_measured^T R_expected; translation is the Euclidean difference.
GraspCheckResult checkGraspRelative(const Eigen::Matrix4d & measured,
                                    const Eigen::Matrix4d & expected,
                                    const GraspTolerances & tol);

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__GRASP_CHECK_HPP_
