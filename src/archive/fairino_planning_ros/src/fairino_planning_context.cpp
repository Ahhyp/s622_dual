#include "fairino_planning_ros/fairino_planning_context.h"
#include "fairino_planning_ros/moveit_collision_checker.h"

#include <chrono>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>            // ⭐ 这一行
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "fairino_planning_core/algorithms/bi_rrt_star.h"

namespace fairino_planning_ros
{

    FairinoPlanningContext::FairinoPlanningContext(
        const std::string &name,
        const std::string &group,
        const moveit::core::RobotModelConstPtr &robot_model)
        : planning_interface::PlanningContext(name, group),
          robot_model_(robot_model), jmg_(robot_model->getJointModelGroup(group))
    {
    }

    bool FairinoPlanningContext::solve(planning_interface::MotionPlanResponse &res)
    {
        /*
        solve() 是 PlanningContext 的核心：六步走——提取起点、提取目标、构造碰撞器、组装 core 请求、
        调用算法、转 RobotTrajectory + TOTG。关键是用 MoveIt 的工具函数提取状态/限位，而不是硬编码，
        这样换型号、换 group 都不用改代码。
        */
        const auto t_start = std::chrono::steady_clock::now();
        terminated_ = false;

        // ---- ① 提取起点 ----
        fairino_planning::JointArray q_start;
        if (!extractStartState(q_start))
        {
            res.error_code_.val =
                moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
            return false;
        }

        // ---- ② 提取目标 ----
        fairino_planning::JointArray q_goal;
        if (!extractGoalState(q_goal))
        {
            res.error_code_.val =
                moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
            return false;
        }

        // ---- ③ 构造碰撞检测器 ----
        auto checker = std::make_shared<MoveItCollisionChecker>(
            planning_scene_, getGroupName());

        // ---- ④ 构造 core 请求 ----
        fairino_planning::PlanRequest core_req;
        core_req.start = q_start;
        core_req.goal = q_goal;
        core_req.limits = extractJointLimits();
        core_req.step_size = 0.05;
        core_req.goal_tolerance = 0.02;
        core_req.max_iterations = 5000;

        // ---- ⑤ 调用 core 算法 ----
        fairino_planning::BiRRTStar planner(checker);
        // planner.setCollisionChecker(checker);

        // 中途取消支持（如果你的算法支持，传入一个标志）
        // planner.setTerminationFlag(&terminated_);

        fairino_planning::PlanResult core_res = planner.plan(core_req);

        if (!core_res.success)
        {
            res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
            res.planning_time_ = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - t_start)
                                     .count();
            return false;
        }

        // ---- ⑥ 路径转 RobotTrajectory + 时间参数化 ----
        convertPathToRobotTrajectory(core_res.path, res);

        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        res.planning_time_ = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - t_start)
                                 .count();
        return true;
    }

    // ============================================================================
    // 详细响应（一般转发给简单版本即可）
    // ============================================================================
    bool FairinoPlanningContext::solve(
        planning_interface::MotionPlanDetailedResponse &res)
    {
        planning_interface::MotionPlanResponse simple_res;
        const bool ok = solve(simple_res);

        res.trajectory_.push_back(simple_res.trajectory_);
        res.description_.push_back("fairino_planning");
        res.processing_time_.push_back(simple_res.planning_time_);
        res.error_code_ = simple_res.error_code_;
        return ok;
    }

    // ============================================================================
    // 清理 / 终止
    // ============================================================================
    void FairinoPlanningContext::clear()
    {
        terminated_ = false;
    }

    bool FairinoPlanningContext::terminate()
    {
        terminated_ = true;
        return true;
    }

    // ============================================================================
    // 提取起始关节角
    // ============================================================================
    bool FairinoPlanningContext::extractStartState(
        fairino_planning::JointArray &q) const
    {
        // 用 PlanningScene 当前状态作为底，再叠加 request 的 start_state
        moveit::core::RobotState state = planning_scene_->getCurrentState();
        moveit::core::robotStateMsgToRobotState(
            planning_scene_->getTransforms(),
            request_.start_state, state);
        state.update();

        std::vector<double> values;
        state.copyJointGroupPositions(jmg_, values);
        if (values.size() != fairino_planning::DOF)
        {
            return false;
        }
        std::copy(values.begin(), values.end(), q.begin());
        return true;
    }

    // ============================================================================
    // 提取目标关节角
    //   - JointConstraint：直接读取
    //   - PositionConstraint + OrientationConstraint：调 IK
    // ============================================================================
    bool FairinoPlanningContext::extractGoalState(
        fairino_planning::JointArray &q) const
    {
        if (request_.goal_constraints.empty())
        {
            return false;
        }

        // 取第一组约束（可扩展为遍历多组）
        const auto &goal = request_.goal_constraints.front();

        // ---- 情况 1：JointConstraint（最常见）----
        if (!goal.joint_constraints.empty())
        {
            if (goal.joint_constraints.size() != fairino_planning::DOF)
            {
                return false;
            }

            // joint_constraints 的顺序未必和 group 关节顺序一致，按名字索引
            const auto &joint_names = jmg_->getActiveJointModelNames();
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                bool found = false;
                for (const auto &jc : goal.joint_constraints)
                {
                    if (jc.joint_name == joint_names[i])
                    {
                        q[i] = jc.position;
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return false;
            }
            return true;
        }

        // ---- 情况 2：Pose Goal（位置 + 姿态约束）----
        if (!goal.position_constraints.empty() &&
            !goal.orientation_constraints.empty())
        {
            // 用 MoveIt 的 RobotState::setFromIK，它会调用 kinematics_solver
            moveit::core::RobotState state = planning_scene_->getCurrentState();

            // 提取目标 pose
            geometry_msgs::msg::Pose target_pose;
            target_pose.position = goal.position_constraints.front()
                                       .constraint_region.primitive_poses.front()
                                       .position;
            target_pose.orientation = goal.orientation_constraints.front().orientation;

            if (!state.setFromIK(jmg_, target_pose, 0.1))
            {
                return false;
            }

            std::vector<double> values;
            state.copyJointGroupPositions(jmg_, values);
            std::copy(values.begin(), values.end(), q.begin());
            return true;
        }

        return false;
    }

    // ============================================================================
    // 提取关节限位
    // ============================================================================
    fairino_planning::JointLimits FairinoPlanningContext::extractJointLimits() const
    {
        fairino_planning::JointLimits lim{};

        const auto &joint_names = jmg_->getActiveJointModelNames();
        for (size_t i = 0; i < joint_names.size() && i < fairino_planning::DOF; ++i)
        {
            const auto *jm = robot_model_->getJointModel(joint_names[i]);
            const auto &bounds = jm->getVariableBounds()[0];

            lim.lower[i] = bounds.position_bounded_ ? bounds.min_position_ : -M_PI;
            lim.upper[i] = bounds.position_bounded_ ? bounds.max_position_ : M_PI;
            lim.velocity[i] = bounds.velocity_bounded_ ? bounds.max_velocity_ : 3.14;
            lim.acceleration[i] = bounds.acceleration_bounded_
                                      ? bounds.max_acceleration_
                                      : 10.0;
        }
        return lim;
    }

    // ============================================================================
    // 路径 → RobotTrajectory + 时间参数化
    // ============================================================================
    void FairinoPlanningContext::convertPathToRobotTrajectory(
        const fairino_planning::JointPath &path,
        planning_interface::MotionPlanResponse &res) const
    {
        auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
            robot_model_, getGroupName());

        // ① 用当前状态为模板，逐个 waypoint 设置关节角
        moveit::core::RobotState reference = planning_scene_->getCurrentState();

        for (const auto &q : path)
        {
            moveit::core::RobotState state(reference);
            std::vector<double> values(q.begin(), q.end());
            state.setJointGroupPositions(jmg_, values);
            state.update();

            // 时间参数化前，每个 waypoint 的 dt 写 0，TOTG 会重算
            trajectory->addSuffixWayPoint(state, 0.0);
        }

        // ② 时间参数化（最优时间，受关节速度/加速度限制）
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        const double vel_scale =
            request_.max_velocity_scaling_factor > 0.0
                ? request_.max_velocity_scaling_factor
                : 1.0;
        const double acc_scale =
            request_.max_acceleration_scaling_factor > 0.0
                ? request_.max_acceleration_scaling_factor
                : 1.0;

        if (!totg.computeTimeStamps(*trajectory, vel_scale, acc_scale))
        {
            // 失败也不致命，发一个粗略时间戳的轨迹也行
        }

        res.trajectory_ = trajectory;
    }
} // namespace fairino_planning_ros