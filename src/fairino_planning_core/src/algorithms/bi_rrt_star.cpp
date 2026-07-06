#include "fairino_planning_core/algorithms/bi_rrt_star.h"
#include <chrono>
#include <algorithm>

namespace fairino_planning
{

BiRRTStar::BiRRTStar(std::shared_ptr<CollisionInterface> collision)
	: collision_(std::move(collision))
{
}

// ---------------------------------------------------------------------------
JointArray BiRRTStar::sample(const JointLimits& limits, const JointArray& goal)
{
	// 10% 目标偏置
	std::uniform_real_distribution<double> bias(0.0, 1.0);
	if (bias(rng_) < 0.10)
		return goal;

	JointArray q;
	for (int i = 0; i < DOF; ++i) {
		std::uniform_real_distribution<double> dist(limits.lower[i], limits.upper[i]);
		q[i] = dist(rng_);
	}
	return q;
}

// ---------------------------------------------------------------------------
JointArray BiRRTStar::steer(const JointArray& from, const JointArray& to,
														 double step)
{
	double d = distance(from, to);
	if (d < 1e-12) return from;

	double ratio = std::min(step, d) / d;
	JointArray result;
	for (int i = 0; i < DOF; ++i)
		result[i] = from[i] + ratio * (to[i] - from[i]);
	return result;
}

// ---------------------------------------------------------------------------
double BiRRTStar::distance(const JointArray& a, const JointArray& b) const
{
	double sq = 0.0;
	for (int i = 0; i < DOF; ++i) {
		double d = a[i] - b[i];
		sq += d * d;
	}
	return std::sqrt(sq);
}

// ---------------------------------------------------------------------------
double BiRRTStar::computeRewireRadius(int n) const
{
	if (n < 2) return 1.0;
	const double gamma = 1.5;
	return gamma * std::pow(std::log(static_cast<double>(n)) / n, 1.0 / DOF);
}

// ---------------------------------------------------------------------------
BiRRTStar::ConnResult BiRRTStar::tryConnect(const JointArray& q_new,
																							RRTTree& other_tree,
																							double max_step)
{
	ConnResult res;
	int idx = other_tree.nearest(q_new);
	const JointArray& q_opp = other_tree.node(idx).state;
	double d = distance(q_new, q_opp);

	if (d < max_step * 1.5) {
		if (collision_->isMotionValid(q_new, q_opp, 0.05)) {
			res.connected = true;
			res.edge_cost = d;
			res.opp_idx = idx;
		}
	}
	return res;
}

// ---------------------------------------------------------------------------
PlanResult BiRRTStar::plan(const PlanRequest& request)
{
	auto t_start = std::chrono::steady_clock::now();
	PlanResult result;

	const int max_iter = request.max_iterations;
	const double step  = request.step_size;
	const double tol   = request.goal_tolerance;

	// 双树初始化
	const int max_n = max_iter / 2 + 10;
	RRTTree treeA(max_n), treeB(max_n);
	treeA.addNode(request.start, -1, 0.0);
	treeB.addNode(request.goal,  -1, 0.0);

	// 交替标志
	bool grow_a = true;

	// 最佳连接记录
	double best_cost = std::numeric_limits<double>::infinity();
	int best_a = -1, best_b = -1;

	// ───────── 主循环 ─────────
	for (int it = 0; it < max_iter; ++it)
	{
		RRTTree& cur = grow_a ? treeA : treeB;
		RRTTree& opp = grow_a ? treeB : treeA;

		// (1) 采样（向对方树的最后一个节点偏置）
		int opp_last = opp.size() - 1;
		JointArray q_target = (opp_last >= 0) ? opp.node(opp_last).state : request.goal;
		JointArray q_rand = sample(request.limits, q_target);

		// (2) 最近 + 步进
		int idx_near = cur.nearest(q_rand);
		JointArray q_near = cur.node(idx_near).state;
		JointArray q_new  = steer(q_near, q_rand, step);

		// (3) 碰撞检测
		if (!collision_->isStateValid(q_new))        { grow_a = !grow_a; continue; }
		if (!collision_->isMotionValid(q_near, q_new, 0.05)) { grow_a = !grow_a; continue; }

		// (4) 邻域搜索 + 选最优父节点
		double rewire_r = computeRewireRadius(cur.size());
		auto near_set = cur.nearRadius(q_new, rewire_r);
		if (near_set.empty())
			near_set.push_back(idx_near);

		int    best_par  = -1;
		double best_c2n  = std::numeric_limits<double>::infinity();
		for (int ic : near_set) {
			double e  = distance(cur.node(ic).state, q_new);
			double cc = cur.node(ic).cost + e;
			if (cc < best_c2n && collision_->isMotionValid(cur.node(ic).state, q_new, 0.05)) {
				best_par = ic;
				best_c2n = cc;
			}
		}
		if (best_par < 0) {
			// fallback: 用最近节点
			if (!collision_->isMotionValid(q_near, q_new, 0.05)) { grow_a = !grow_a; continue; }
			best_par = idx_near;
			best_c2n = cur.node(idx_near).cost + distance(q_near, q_new);
		}

		// (5) 添加节点
		int new_idx = cur.addNode(q_new, best_par, best_c2n);

		// (6) 重布线：遍历邻域，通过 q_new 到某节点代价更低则重接
		for (int j : near_set) {
			if (j == best_par || j == new_idx) continue;
			double e_j  = distance(q_new, cur.node(j).state);
			double c_via = best_c2n + e_j;
			if (c_via + 1e-12 >= cur.node(j).cost) continue;
			if (!collision_->isMotionValid(q_new, cur.node(j).state, 0.05)) continue;

			// 更新 j 的父节点为 new_idx
			cur.node(j).parent = new_idx;
			cur.node(j).cost   = c_via;
			cur.node(new_idx).children.push_back(j);
			cur.propagateCost(j);
		}

		// (7) 尝试连接两棵树
		auto conn = tryConnect(q_new, opp, step);
		if (conn.connected) {
			double total = best_c2n + conn.edge_cost + opp.node(conn.opp_idx).cost;
			if (total < best_cost) {
				best_cost = total;
				best_a    = grow_a ? new_idx : conn.opp_idx;
				best_b    = grow_a ? conn.opp_idx : new_idx;
			}
		}

		// (8) 检查是否到达对方树的根节点附近
		int opp_root = 0;  // 对方树的根节点（goal 或 start）
		double d_to_root = distance(q_new, opp.node(opp_root).state);
		if (d_to_root < tol) {
			double total = best_c2n + d_to_root + opp.node(opp_root).cost;
			if (total < best_cost) {
				best_cost = total;
				best_a    = grow_a ? new_idx : opp_root;
				best_b    = grow_a ? opp_root : new_idx;
			}
		}

		grow_a = !grow_a;
	}

	// ───────── 路径组装 ─────────
	if (best_a < 0) {
		result.success = false;
		result.message = "BiRRTStar: no connection found after " +
										 std::to_string(max_iter) + " iterations";
		return result;
	}

	auto pathA = treeA.backtrack(best_a);
	auto pathB = treeB.backtrack(best_b);
	std::reverse(pathB.begin(), pathB.end());

	result.path.clear();
	result.path.insert(result.path.end(), pathA.begin(), pathA.end());
	// 跳过 pathB[0]（和 pathA 最后一个重复）
	if (!pathB.empty()) pathB.erase(pathB.begin());
	result.path.insert(result.path.end(), pathB.begin(), pathB.end());

	auto t_end = std::chrono::steady_clock::now();
	result.success       = true;
	result.planning_time = std::chrono::duration<double>(t_end - t_start).count();
	result.message       = "BiRRTStar: path found, " +
												 std::to_string(result.path.size()) + " waypoints";

	return result;
}

}  // namespace fairino_planning
