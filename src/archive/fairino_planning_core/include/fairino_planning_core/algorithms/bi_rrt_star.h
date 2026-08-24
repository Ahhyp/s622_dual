#pragma once

#include "fairino_planning_core/algorithms/planning_algorithm.h"
#include "fairino_planning_core/tree/rrt_tree.h"
#include "fairino_planning_core/collision/collision_interface.h"
#include <memory>
#include <random>

namespace fairino_planning
{

class BiRRTStar : public PlanningAlgorithm
{
public:
	explicit BiRRTStar(std::shared_ptr<CollisionInterface> collision);

	PlanResult plan(const PlanRequest& request) override;

private:
	std::shared_ptr<CollisionInterface> collision_;
	std::mt19937 rng_{std::random_device{}()};

	JointArray sample(const JointLimits& limits, const JointArray& goal);
	JointArray steer(const JointArray& from, const JointArray& to, double step);
	double distance(const JointArray& a, const JointArray& b) const;
	double computeRewireRadius(int n) const;

	struct ConnResult
	{
		bool connected = false;
		double edge_cost = 0.0;
		int opp_idx = -1;
	};

	ConnResult tryConnect(const JointArray& q_new, RRTTree& other_tree,
												double max_step);
};

}  // namespace fairino_planning
