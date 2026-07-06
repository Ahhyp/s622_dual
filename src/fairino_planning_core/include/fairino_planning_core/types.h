#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fairino_planning
{

    constexpr int DOF = 6;

    using JointArray = std::array<double, DOF>;
    using JointPath = std::vector<JointArray>;
    using Pose = Eigen::Isometry3d;

    struct JointLimits
    {
        JointArray lower = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
        JointArray upper = { 3.0543,  1.4835,  2.8274,  1.4835,  3.0543,  3.0543};
        JointArray velocity = {3.15, 3.15, 3.15, 3.2, 3.2, 3.2};
        JointArray acceleration = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

        bool isWithin(const JointArray &q, double tol = 1e-6) const
        {
            for (int i = 0; i < DOF; ++i)
                if (q[i] < lower[i] - tol || q[i] > upper[i] + tol)
                    return false;
            return true;
        }

        JointArray clamp(const JointArray& q) const {
            JointArray result;
            for (int i = 0; i < DOF; ++i)
                result[i] = std::clamp(q[i], lower[i], upper[i]);
            return result;
        }
    };

    struct PlanRequest
    {
        JointArray start;
        JointArray goal;
        JointLimits limits;
        double step_size{0.05};
        double goal_tolerance{0.02};
        int max_iterations{5000};
    };

    struct PlanResult
    {
        bool success{false};
        JointPath path;
        double planning_time{0.0};
        std::string message;
    };

    //FK/IK 第一步就会用到角度归一化
    inline double wrapToPi(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }
    inline JointArray wrapToPi(const JointArray& q) {
        JointArray result;
        for (int i = 0; i < DOF; ++i)
            result[i] = wrapToPi(q[i]);
        return result;
    }



} // namespace fairino_planning