#include "fairino_planning_core/ik/fairino_ik.h"
#include <cmath>
#include <algorithm>

namespace fairino_planning
{

    FairinoIK::FairinoIK(const std::array<double, DOF> &d,
                         const std::array<double, DOF> &a,
                         const std::array<double, DOF> &alpha)
        : d_(d), a_(a), alpha_(alpha), fk_([](
            const std::array<double, DOF>& dd,
            const std::array<double, DOF>& aa,
            const std::array<double, DOF>& al) {
            std::array<DHParam, DOF> dh;
            for (int i = 0; i < DOF; ++i)
                dh[i] = {aa[i], al[i], dd[i], 0.0};
            return dh;
        }(d, a, alpha))
    {
    }

    Eigen::Matrix3d FairinoIK::R03(double q1, double q2, double q3)
    {
        const double c1 = std::cos(q1), s1 = std::sin(q1);
        const double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);
        Eigen::Matrix3d R;
        R << c1 * c23, -c1 * s23, s1,
            s1 * c23, -s1 * s23, -c1,
            s23, c23, 0;
        return R;
    }

    std::vector<JointArray> FairinoIK::solve(const Pose &target) const
    {
        // 提取常量（取绝对值用于三角形几何法）
        const double d1 = d_[0], d4 = d_[3], d5 = d_[4], d6 = d_[5];
        const double L2 = std::abs(a_[1]); // 0.280
        const double L3 = std::abs(a_[2]); // 0.240

        const Eigen::Vector3d p = target.translation();
        const Eigen::Matrix3d R = target.rotation();
        const Eigen::Vector3d a = R.col(2); // 末端 z 轴
        const double ax = a.x(), ay = a.y(), az = a.z();

        // Step 1: 腕前点 WCP = p - d6 * a
        const double xw = p.x() - d6 * ax;
        const double yw = p.y() - d6 * ay;
        const double zw = p.z() - d6 * az;

        // Step 2: q1 候选（两解：肩左 / 肩右）
        const double rho_sq = xw * xw + yw * yw - d4 * d4;
        if (rho_sq < -1e-10)
            return {};

        const double rho = std::sqrt(std::max(rho_sq, 0.0));
        const double q1_cands[2] = {
            std::atan2(yw, xw) - std::atan2(-d4, rho),
            std::atan2(yw, xw) - std::atan2(-d4, -rho)};

        std::vector<JointArray> candidates;

        for (double q1 : q1_cands)
        {
            q1 = wrapToPi(q1);
            const double c1 = std::cos(q1), s1 = std::sin(q1);

            // Step 3: q5, q234
            double c5 = s1 * ax - c1 * ay;
            c5 = std::clamp(c5, -1.0, 1.0);
            const double s5_abs = std::sqrt(std::max(0.0, 1.0 - c5 * c5));
            if (s5_abs < 1e-8)
                continue; // 腕部奇异

            const double C = c1 * ax + s1 * ay;

            for (double s5 : {s5_abs, -s5_abs})
            {
                const double q5 = std::atan2(s5, c5);
                const double c234 = -C / s5;
                const double s234 = -az / s5;
                const double q234 = std::atan2(s234, c234);

                // Step 4: 2R 平面三角形解 q2, q3
                const double Xp = c1 * xw + s1 * yw - d5 * s234;
                const double Zp = zw - d1 + d5 * c234;
                const double Xg = -Xp, Zg = -Zp;

                double D = (Xg * Xg + Zg * Zg - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
                if (D < -1.0 - 1e-10 || D > 1.0 + 1e-10)
                    continue;
                D = std::clamp(D, -1.0, 1.0);
                const double s3_abs = std::sqrt(std::max(0.0, 1.0 - D * D));

                for (double s3 : {s3_abs, -s3_abs})
                {
                    const double q3 = std::atan2(s3, D);
                    const double q2 = std::atan2(Zg, Xg) - std::atan2(L3 * s3, L2 + L3 * D);
                    const double q4 = wrapToPi(q234 - q2 - q3);

                    // Step 5: q6
                    const Eigen::Matrix3d R03_ = R03(q1, q2, q3);
                    const Eigen::Matrix3d R36 = R03_.transpose() * R;
                    const double q6 = std::atan2(-R36(2, 1), R36(2, 0));

                    JointArray q{q1, q2, q3, q4, q5, q6};
                    q = wrapToPi(q);

                    // FK 回代验证
                    Pose T_check = fk_.forward(q);
                    double pos_err = (T_check.translation() - p).norm();
                    double rot_err = (T_check.rotation() - R).norm();

                    if (pos_err < 1e-4 && rot_err < 1e-4)
                        candidates.push_back(q);
                }
            }
        }

        return uniqueSolutions(candidates);
    }

    std::vector<JointArray> FairinoIK::uniqueSolutions(
        const std::vector<JointArray> &sols, double tol)
    {
        std::vector<JointArray> unique;
        for (const auto &q : sols)
        {
            bool dup = false;
            for (const auto &u : unique)
            {
                double sq_dist = 0.0;
                for (int i = 0; i < DOF; ++i)
                {
                    double dq = wrapToPi(q[i] - u[i]);
                    sq_dist += dq * dq;
                }
                if (std::sqrt(sq_dist) < tol)
                {
                    dup = true;
                    break;
                }
            }
            if (!dup)
                unique.push_back(q);
        }
        return unique;
    }

} // namespace fairino_planning
