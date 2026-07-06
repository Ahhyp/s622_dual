#include "fairino_planning_core/types.h"
#include "fairino_planning_core/dh_kinematics.h"
#include "fairino_planning_core/ik/fairino_ik.h"
#include "fairino_planning_core/ik/ik_selector.h"
#include "fairino_planning_core/algorithms/bi_rrt_star.h"
#include "fairino_planning_core/collision/collision_interface.h"
#include <iostream>
#include <cmath>
#include <iomanip>

namespace fairino_planning
{

// Mock 碰撞检测器：无障碍物，永远通过
class MockCollision : public CollisionInterface
{
public:
	bool isStateValid(const JointArray&) const override { return true; }
	bool isMotionValid(const JointArray&, const JointArray&, double) const override { return true; }
};

}  // namespace fairino_planning

static std::array<double, 6> makeDH_d()  { return {0.140, 0, 0, 0.102, 0.102, 0.100}; }
static std::array<double, 6> makeDH_a()  { return {0, -0.280, -0.240, 0, 0, 0}; }
static std::array<double, 6> makeDH_al() { return {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0}; }

static fairino_planning::JointLimits makeLimits()
{
	fairino_planning::JointLimits lim;
	lim.lower       = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
	lim.upper       = { 3.0543,  1.4835,  2.8274,  1.4835,  3.0543,  3.0543};
	lim.velocity    = {3.15, 3.15, 3.15, 3.2, 3.2, 3.2};
	lim.acceleration = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	return lim;
}

#define TEST(name) std::cout << "\n=== " << name << " ===" << std::endl
#define PASS()     std::cout << "  PASS" << std::endl
#define FAIL(msg)  std::cout << "  FAIL: " << msg << std::endl

int main()
{
	using namespace fairino_planning;
	std::cout << std::fixed << std::setprecision(4);

	// ═══════════════════════════════════════
	//  Test 1: FK 零位
	// ═══════════════════════════════════════
	TEST("1. FK 正运动学 — 零位");
	{
		auto d = makeDH_d(), a = makeDH_a(), al = makeDH_al();
		std::array<DHParam, DOF> dh;
		for (int i = 0; i < DOF; ++i)
			dh[i] = {a[i], al[i], d[i], 0.0};
		DHKinematics fk(dh);

		JointArray q_zero = {0, 0, 0, 0, 0, 0};
		Pose T = fk.forward(q_zero);

		Eigen::Vector3d p = T.translation();
		Eigen::Matrix3d R = T.rotation();

		std::cout << "  end-effector pos: [" << p.x() << ", "
		          << p.y() << ", " << p.z() << "]" << std::endl;
		double det = R.determinant();
		std::cout << "  det(R) = " << det << " (expect ~1.0)" << std::endl;
		if (std::abs(det - 1.0) < 0.01) PASS(); else FAIL("det(R) not ~1");
	}

	// ═══════════════════════════════════════
	//  Test 2: FK 非零位 — 不同输入得不同输出
	// ═══════════════════════════════════════
	TEST("2. FK 正运动学 — 非零关节角");
	{
		auto d = makeDH_d(), a = makeDH_a(), al = makeDH_al();
		std::array<DHParam, DOF> dh;
		for (int i = 0; i < DOF; ++i)
			dh[i] = {a[i], al[i], d[i], 0.0};
		DHKinematics fk(dh);

		JointArray q1 = {0.5, -0.3, 1.0, 0.2, -0.7, 0.0};
		Pose T1 = fk.forward(q1);
		std::cout << "  q1 pos: [" << T1.translation().x() << ", "
		          << T1.translation().y() << ", "
		          << T1.translation().z() << "]" << std::endl;

		JointArray q2 = {1.0, 0.8, -0.5, -1.0, 0.3, 1.5};
		Pose T2 = fk.forward(q2);
		std::cout << "  q2 pos: [" << T2.translation().x() << ", "
		          << T2.translation().y() << ", "
		          << T2.translation().z() << "]" << std::endl;

		double pos_diff = (T1.translation() - T2.translation()).norm();
		std::cout << "  pos diff = " << pos_diff << " (expect >0)" << std::endl;
		if (pos_diff > 1e-6) PASS(); else FAIL("different q gives same pose");
	}

	// ═══════════════════════════════════════
	//  Test 3: IK — FK(IK(pose)) ≈ pose
	// ═══════════════════════════════════════
	TEST("3. IK 解析逆运动学");
	{
		auto d = makeDH_d(), a = makeDH_a(), al = makeDH_al();
		FairinoIK ik(d, a, al);

		std::array<DHParam, DOF> dh;
		for (int i = 0; i < DOF; ++i)
			dh[i] = {a[i], al[i], d[i], 0.0};
		DHKinematics fk(dh);

		// 用 FK 构造可达目标位姿
		JointArray q_ref = {0.2, -0.5, 0.8, -0.3, 0.6, 0.1};
		Pose target = fk.forward(q_ref);

		auto sols = ik.solve(target);
		std::cout << "  IK returned " << sols.size() << " solutions (expect >0)" << std::endl;

		if (sols.empty()) {
			FAIL("no IK solutions for reachable pose");
		} else {
			PASS();
			for (size_t s = 0; s < sols.size(); ++s) {
				Pose T = fk.forward(sols[s]);
				double err = (T.translation() - target.translation()).norm();
				std::cout << "    sol[" << s << "]: pos_err=" << err
				          << (err < 1e-3 ? " OK" : " FAIL") << std::endl;
			}
		}
	}

	// ═══════════════════════════════════════
	//  Test 4: IK 选择器
	// ═══════════════════════════════════════
	TEST("4. IK 选择器");
	{
		auto d = makeDH_d(), a = makeDH_a(), al = makeDH_al();
		FairinoIK ik(d, a, al);
		IKSelector selector;

		std::array<DHParam, DOF> dh;
		for (int i = 0; i < DOF; ++i)
			dh[i] = {a[i], al[i], d[i], 0.0};
		DHKinematics fk(dh);
		JointArray q_ref = {0.2, -0.5, 0.8, -0.3, 0.6, 0.1};
		Pose target = fk.forward(q_ref);

		auto sols = ik.solve(target);
		if (sols.empty()) {
			std::cout << "  skipped (no IK solutions)" << std::endl;
		} else {
			JointArray q_cur = {0, 0, 0, 0, 0, 0};
			auto best = selector.select(sols, q_cur);
			if (best.has_value()) {
				std::cout << "  selected: [" << (*best)[0] << ", "
				          << (*best)[1] << ", " << (*best)[2] << ", "
				          << (*best)[3] << ", " << (*best)[4] << ", "
				          << (*best)[5] << "]" << std::endl;
				PASS();
			} else {
				FAIL("selector returned nullopt");
			}
		}
	}

	// ═══════════════════════════════════════
	//  Test 5: BiRRT* 简单路径
	// ═══════════════════════════════════════
	TEST("5. BiRRT* 规划 — 无障碍简单路径");
	{
		auto collision = std::make_shared<MockCollision>();
		BiRRTStar planner(collision);

		PlanRequest req;
		req.start          = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		req.goal           = {0.5, -0.3, 0.8, -0.2, 0.4, 0.6};
		req.limits         = makeLimits();
		req.step_size      = 0.1;
		req.goal_tolerance = 0.1;
		req.max_iterations = 3000;

		std::cout << "  max_iter=" << req.max_iterations
		          << " step=" << req.step_size
		          << " tol=" << req.goal_tolerance << std::endl;

		PlanResult result = planner.plan(req);

		std::cout << "  success: " << (result.success ? "true" : "false") << std::endl;
		std::cout << "  waypoints: " << result.path.size() << std::endl;
		std::cout << "  time: " << result.planning_time << "s" << std::endl;
		std::cout << "  message: " << result.message << std::endl;

		if (result.success && result.path.size() > 1) {
			PASS();
		} else {
			FAIL("no path found or path too short");
		}
	}

	// ═══════════════════════════════════════
	//  Test 6: BiRRT* 远距离路径
	// ═══════════════════════════════════════
	TEST("6. BiRRT* 规划 — 远距离路径");
	{
		auto collision = std::make_shared<MockCollision>();
		BiRRTStar planner(collision);

		PlanRequest req;
		req.start          = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0};
		req.goal           = {1.0, -1.0, 1.5, -0.8, 1.0, 1.5};
		req.limits         = makeLimits();
		req.step_size      = 0.15;
		req.goal_tolerance = 0.15;
		req.max_iterations = 5000;

		std::cout << "  max_iter=" << req.max_iterations
		          << " step=" << req.step_size
		          << " tol=" << req.goal_tolerance << std::endl;

		PlanResult result = planner.plan(req);

		std::cout << "  success: " << (result.success ? "true" : "false") << std::endl;
		std::cout << "  waypoints: " << result.path.size() << std::endl;
		std::cout << "  time: " << result.planning_time << "s" << std::endl;
		std::cout << "  message: " << result.message << std::endl;

		if (result.success && result.path.size() > 1) {
			PASS();
		} else {
			FAIL("no path found");
		}
	}

	std::cout << "\n=== 全部测试完成 ===" << std::endl;
	return 0;
}
