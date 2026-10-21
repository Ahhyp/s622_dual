// C4.3 S3 -- W4: frozen piecewise SE(3) object trajectory + body-twist feed-forward.
//
// The trajectory is *data*, not code: it comes from config/s3_transport_trajectory.yaml.
// This module only evaluates it analytically (no integration, no filtering), so the
// feed-forward is exact by construction rather than by tuning -- that is what makes
// S3-T4 assertable at 1e-6.
//
// Convention (C4.3_S3_实施方案.md §2.4): the returned twist is the DESIRED object BODY
// twist v_d = [R_dᵀ ṗ_d ; ω_body], and it enters the QP as
//     row.body_twist_ff = +v_d      (paired with row.e = -e_O, see §2.3.1(c)(d))
#ifndef DUAL_ARM_TRANSPORT_MOCK__TRAJECTORY_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_transport_mock {

/// Thrown for any malformed spec (missing key, negative duration, yaw node count
/// mismatch, ...).  A trajectory that cannot be parsed must never silently become
/// a different trajectory.
class TrajectoryError : public std::runtime_error {
public:
    explicit TrajectoryError(const std::string& what) : std::runtime_error(what) {}
};

struct TrajectoryPhase {
    std::string name;
    double duration = 0.0;   ///< s
    double dx = 0.0;         ///< m
    double dy = 0.0;         ///< m
    double dz = 0.0;         ///< m
};

struct TrajectorySpec {
    double dt = 0.008;                        ///< = DualArmQpParams::dt (125 Hz)
    std::string time_scaling = "smoothstep";  ///< "smoothstep" | "linear"
    dual_arm::Iso3 start_world_T_object = dual_arm::Iso3::Identity();
    /// YAML said `start: from_baseline`: the caller must replace the pose above.
    bool start_from_baseline = true;
    std::vector<TrajectoryPhase> phases;
    /// Yaw (deg, about world z) at each phase boundary: size 1 (constant, no
    /// rotation) or size phases.size()+1 (per-boundary node).  Per-phase smoothstep
    /// keeps ẏaw == 0 at every boundary, so v_ff is continuous and vanishes there.
    std::vector<double> yaw_sweep_deg{0.0};
};

/// Parse config/s3_transport_trajectory.yaml.  Throws TrajectoryError.
TrajectorySpec loadTrajectorySpec(const std::string& path);

class FrozenObjectTrajectory {
public:
    explicit FrozenObjectTrajectory(const TrajectorySpec& spec);

    double duration() const { return duration_; }
    /// Number of control periods: ceil(duration / dt).
    std::size_t steps() const { return steps_; }
    /// Phase boundary times, size phases.size()+1, starting at 0.
    const std::vector<double>& boundaryTimes() const { return boundary_times_; }
    const TrajectorySpec& spec() const { return spec_; }

    /// Analytic evaluation at time t (clamped to [0, duration]).
    /// `body_twist_ff`, if non-null, receives v_d = [R_dᵀ ṗ_d ; ω_body] (eq. (8)).
    /// Returns T_d(t).  A time beyond the end returns the final pose and a ZERO
    /// twist (the object has stopped; it is not coasting).
    dual_arm::Iso3 at(double t, dual_arm::Vec6* body_twist_ff) const;

private:
    void locate(double t, std::size_t* phase, double* tau) const;

    TrajectorySpec spec_;
    double duration_ = 0.0;
    std::size_t steps_ = 0;
    std::vector<double> boundary_times_;
    /// World-frame position at each phase start (size phases.size()+1).
    std::vector<Eigen::Vector3d> phase_start_p_;
};

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__TRAJECTORY_HPP_
