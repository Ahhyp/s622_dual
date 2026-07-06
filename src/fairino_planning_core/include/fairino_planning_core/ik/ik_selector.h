#pragma once
#include "fairino_planning_core/types.h"
#include <optional>

namespace fairino_planning {

struct IKSelectParams {
    // W_move: 关节 1-3 权重大（大关节少动），4-6 权重小（腕部多动）
    std::array<double, DOF> move_weights{5.0, 2.5, 2.0, 1.5, 1.0, 1.0};
    double w_move  = 3.0;
    double w_limit = 2.0;
};

class IKSelector {
public:
    IKSelector();
    explicit IKSelector(const IKSelectParams& params);

    std::optional<JointArray> select(
        const std::vector<JointArray>& solutions,
        const JointArray& q_current) const;

private:
    IKSelectParams params_;
    JointLimits    limits_;

    double computeCost(const JointArray& q, const JointArray& q_current) const;
};

}  // namespace fairino_planning