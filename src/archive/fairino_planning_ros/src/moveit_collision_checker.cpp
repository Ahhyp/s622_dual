#include "fairino_planning_ros/moveit_collision_checker.h"

#include <algorithm>
#include <cmath>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/joint_model_group.h>

namespace fairino_planning_ros
{

    MoveItCollisionChecker::MoveItCollisionChecker(
        planning_scene::PlanningSceneConstPtr planning_scene, // 场景指针
        const std::string &group_name)                        // 规划组名
        : planning_scene_(planning_scene), group_name_(group_name)
    {
        if (!planning_scene_)
        {
            throw std::invalid_argument(
                "MoveItCollisionChecker: planning_scene is null");
        }

        // 缓存 JointModelGroup 指针，避免每次查找
        jmg_ = planning_scene_->getRobotModel()->getJointModelGroup(group_name_);
        if (!jmg_)
        {
            throw std::invalid_argument(
                "MoveItCollisionChecker: group '" + group_name_ + "' not found");
        }
    }

    // 单状态碰撞检测
    bool MoveItCollisionChecker::isStateValid(const fairino_planning::JointArray &q) const
    {
        // ① 在 PlanningScene 当前状态基础上构造 RobotState
        //    用 getCurrentState() 拿到完整状态（含 group 之外的关节，如夹爪）
        moveit::core::RobotState state = planning_scene_->getCurrentState();

        // ② 只设置目标 group 的关节角
        std::vector<double> q_vec(q.begin(), q.end());
        state.setJointGroupPositions(jmg_, q_vec);
        state.update(); // 触发 FK，更新所有 link 位姿

        // ③ 关节限位检查（MoveIt 的检查比手写更全：包含连续关节、mimic 等）
        if (!state.satisfiesBounds(jmg_))
        {
            return false;
        }

        // ④ 碰撞检测（含自碰撞 + 环境碰撞）
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = group_name_;
        req.contacts = false; // 不需要接触点细节，省时间
        req.cost = false;
        req.distance = false;

        planning_scene_->checkCollision(req, res, state);
        if (res.collision)
        {
            return false;
        }

        // ⑤ 路径约束（如方向约束、可见性约束等，规划时由上层注入）
        //    没有约束就跳过；有约束就检查
        //    可选：planning_scene_->isStateConstrained(state, ...)

        return true;
    }

    // 一段运动的碰撞检测：在关节空间沿直线离散采样
    bool MoveItCollisionChecker::isMotionValid(
        const fairino_planning::JointArray &from,
        const fairino_planning::JointArray &to,
        double resolution) const
    {
        // ① 确定离散步数：以最大关节移动量除以分辨率
        double max_delta = 0.0;
        for (int i = 0; i < fairino_planning::DOF; ++i)
        {
            max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
        }

        // 完全没动，只检查端点
        if (max_delta < 1e-9)
        {
            return isStateValid(from);
        }

        // resolution 默认例如 0.05 rad ≈ 3°
        const int steps = std::max(
            1, static_cast<int>(std::ceil(max_delta / resolution)));

        // ② 复用一个 RobotState（避免每步重建）
        moveit::core::RobotState state = planning_scene_->getCurrentState();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = group_name_;
        req.contacts = false;

        // ③ 沿直线插值采样，每个点都做完整碰撞检测
        //    采样顺序：先两端，再中点二分——更早发现碰撞，提前退出
        //    简化做法：从头到尾顺序采样
        std::vector<double> q_vec(fairino_planning::DOF);

        for (int k = 0; k <= steps; ++k)
        {
            const double t = static_cast<double>(k) / steps;

            for (int i = 0; i < fairino_planning::DOF; ++i)
            {
                q_vec[i] = from[i] + t * (to[i] - from[i]);
            }

            state.setJointGroupPositions(jmg_, q_vec);
            state.update();

            if (!state.satisfiesBounds(jmg_))
            {
                return false;
            }

            res.clear(); // 复用 res 必须 clear
            planning_scene_->checkCollision(req, res, state);
            if (res.collision)
            {
                return false;
            }
        }

        return true;
    }

} // namespace fairino_planning_ros