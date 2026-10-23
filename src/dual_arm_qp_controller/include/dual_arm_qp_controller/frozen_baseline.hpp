// C4.3 S4: the frozen S2 baseline constants the controller must never re-derive.
//
// Why a single header: both the ROS parameter defaults and the pure
// TransportTaskConfig defaults need G_L / G_R.  While the two were written out
// separately, TransportTaskConfig silently defaulted to Identity, which produced
// a plausible-looking but *wrong* object frame (anchor gap 0.0994 m, attitude
// error pi) instead of a visible failure.  One definition cannot drift.
//
// Source of truth: src/dual_arm_transport/config/transport_baseline.yaml
// (`object_T_left_tcp` / `object_T_right_tcp`).  The W2 test
// FrozenBaseline.MatchesS2BaselineYaml re-reads that file and fails if this
// header is ever edited without the baseline.
#ifndef DUAL_ARM_QP_CONTROLLER__FROZEN_BASELINE_HPP_
#define DUAL_ARM_QP_CONTROLLER__FROZEN_BASELINE_HPP_

#include <array>
#include <vector>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_qp_controller {

/// object_T_left_tcp, row-major 4x4 (S2 frozen baseline).
inline constexpr std::array<double, 16> kFrozenObjectTLeftTcp = {
    0.0, 1.0,  0.0, 0.0497,
    1.0, 0.0,  0.0, 0.0,
    0.0, 0.0, -1.0, 0.025,
    0.0, 0.0,  0.0, 1.0};

/// object_T_right_tcp, row-major 4x4 (S2 frozen baseline).
inline constexpr std::array<double, 16> kFrozenObjectTRightTcp = {
    0.0, 1.0,  0.0, -0.0497,
    1.0, 0.0,  0.0,  0.0,
    0.0, 0.0, -1.0,  0.025,
    0.0, 0.0,  0.0,  1.0};

inline dual_arm::Iso3 iso3FromRowMajor16(const std::array<double, 16>& a) {
    dual_arm::Iso3 T = dual_arm::Iso3::Identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) T(r, c) = a[static_cast<std::size_t>(4 * r + c)];
    }
    return T;
}

inline dual_arm::Iso3 frozenObjectTLeftTcp() { return iso3FromRowMajor16(kFrozenObjectTLeftTcp); }
inline dual_arm::Iso3 frozenObjectTRightTcp() { return iso3FromRowMajor16(kFrozenObjectTRightTcp); }

inline std::vector<double> frozenObjectTLeftTcpVector() {
    return {kFrozenObjectTLeftTcp.begin(), kFrozenObjectTLeftTcp.end()};
}
inline std::vector<double> frozenObjectTRightTcpVector() {
    return {kFrozenObjectTRightTcp.begin(), kFrozenObjectTRightTcp.end()};
}

}  // namespace dual_arm_qp_controller

#endif  // DUAL_ARM_QP_CONTROLLER__FROZEN_BASELINE_HPP_
