#include "fairino_planning_core/ik/ik_selector.h"
#include <cmath>
#include <limits>

namespace fairino_planning
{

    IKSelector::IKSelector() : params_(), limits_() {}
    IKSelector::IKSelector(const IKSelectParams &p) : params_(p), limits_() {}

    double IKSelector::computeCost(const JointArray &q,
                                   const JointArray &q_current) const
    {
        // 1. 关节运动代价
        double J_move = 0.0;
        for (int i = 0; i < DOF; ++i)
        {
            double dq = wrapToPi(q[i] - q_current[i]);
            J_move += params_.move_weights[i] * dq * dq;
        }

        // 2. 限位居中代价
        double J_limit = 0.0;
        for (int i = 0; i < DOF; ++i)
        {
            double mid = 0.5 * (limits_.lower[i] + limits_.upper[i]);
            double half = 0.5 * (limits_.upper[i] - limits_.lower[i]);
            double eta = (q[i] - mid) / half;
            J_limit += eta * eta;
        }

        return params_.w_move * J_move +
               params_.w_limit * J_limit;
    }

    std::optional<JointArray> IKSelector::select(
        const std::vector<JointArray> &solutions,
        const JointArray &q_current) const
    {
        double best_cost = std::numeric_limits<double>::infinity();
        std::optional<JointArray> best;

        for (const auto &q : solutions)
        {
            JointArray qw = wrapToPi(q);
            if (!limits_.isWithin(qw))
                continue;

            double cost = computeCost(qw, q_current);
            if (cost < best_cost)
            {
                best_cost = cost;
                best = qw;
            }
        }
        return best;
    }

} // namespace fairino_planning