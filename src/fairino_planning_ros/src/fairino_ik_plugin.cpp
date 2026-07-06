#include "fairino_planning_ros/fairino_ik_plugin.h"
#include "fairino_planning_core/types.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fairino_planning_ros
{
    bool loadDHParam(const rclcpp::Node::SharedPtr &node,
                     const std::string &name,
                     std::array<double, fairino_planning::DOF> &out)
    {
        std::vector<double> v;
        node->declare_parameter(name, std::vector<double>{});
        if (!node->get_parameter(name, v) || v.size() != fairino_planning::DOF)
        {
            return false;
        }
        std::copy(v.begin(), v.end(), out.begin());
        return true;
    }

    bool FairinoIKPlugin::initialize(
        const rclcpp::Node::SharedPtr &node,         // ROS 节点（用于读参数、日志）
        const moveit::core::RobotModel &robot_model, // MoveIt 解析好的整个机器人模型（URDF）
        const std::string &group_name,               // planning group，例如 "manipulator"
        const std::string &base_frame,               // 基坐标系，例如 "base_link"
        const std::vector<std::string> &tip_frames,  // 末端坐标系列表，通常 1 个，例如 {"tool0"}
        double search_discretization)                // 搜索步长（冗余轴扫描用，6 轴一般用不上）
    {
        node_ = node;
        // ① 把参数存到基类（基类成员函数会用到这些）
        storeValues(robot_model, group_name, base_frame, tip_frames,
                    search_discretization);

        // 校验 planning group：必须是 6 关节
        const moveit::core::JointModelGroup *jmg = robot_model.getJointModelGroup(group_name);
        if (!jmg)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Group '%s' not found in robot model.", group_name.c_str());
            return false;
        }

        const auto &active_joints = jmg->getActiveJointModelNames();
        if (active_joints.size() != fairino_planning::DOF)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Fairino IK requires %d joints, but group '%s' has %zu.",
                         fairino_planning::DOF, group_name.c_str(),
                         active_joints.size());
            return false;
        }

        // 校验末端坐标系：只支持单末端
        if (tip_frames.size() != 1)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Fairino IK supports exactly 1 tip frame, got %zu.",
                         tip_frames.size());
            return false;
        }

        // ④ 缓存关节名/连杆名（基类的 getJointNames/getLinkNames 要用）
        joint_names_ = active_joints;
        link_names_.clear();
        for (const auto *lm : jmg->getLinkModels())
        {
            link_names_.push_back(lm->getName());
        }

        // 读取 DH 参数（从 ROS 参数服务器或 yaml）
        // 形式： fairino_ik.dh.a / .alpha / .d / .theta_offset 各 6 个 double
        std::array<double, fairino_planning::DOF> a{}, alpha{}, d{}, theta_off{};
        const std::string ns = "fairino_ik.dh.";
        if (!loadDHParam(node_, ns + "a", a) ||
            !loadDHParam(node_, ns + "alpha", alpha) ||
            !loadDHParam(node_, ns + "d", d) ||
            !loadDHParam(node_, ns + "theta_offset", theta_off))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to load DH parameters under '%s'.", ns.c_str());
            return false;
        }

        // ==========================================================
        // ⑥ 构造 core 的解析 IK 求解器
        // ==========================================================
        ik_ = std::make_unique<fairino_planning::FairinoIK>(d, a, alpha);

        // ==========================================================
        // ⑦ 构造 core 的 FK（getPositionFK 要用）
        // ==========================================================
        std::array<fairino_planning::DHParam, fairino_planning::DOF> dh_array;
        for (int i = 0; i < fairino_planning::DOF; ++i)
        {
            dh_array[i] = {a[i], alpha[i], d[i], theta_off[i]};
        }
        fk_ = std::make_unique<fairino_planning::DHKinematics>(dh_array);

        // ==========================================================
        // ⑧ 提取关节限位（用于过滤越限解）
        // ==========================================================
        for (size_t i = 0; i < active_joints.size(); ++i)
        {
            const auto *jm = robot_model.getJointModel(active_joints[i]);
            const auto &bounds = jm->getVariableBounds()[0];
            joint_lower_[i] = bounds.position_bounded_ ? bounds.min_position_ : -M_PI;
            joint_upper_[i] = bounds.position_bounded_ ? bounds.max_position_ : M_PI;
        }

        // ==========================================================
        // ⑨ 计算 base_frame 与 robot_model 根之间的变换（如有）
        //    URDF 的 base_link 和 DH 第一关节坐标系如果不一致，需要补偿
        // ==========================================================
        // 通常法奥的 base_link 与 DH 基一致，这里留接口
        base_offset_ = Eigen::Isometry3d::Identity();
        tip_offset_ = Eigen::Isometry3d::Identity();

        // ==========================================================
        // ⑩ 日志
        // ==========================================================
        RCLCPP_INFO(node_->get_logger(),
                    "FairinoIKPlugin initialized: group='%s', base='%s', tip='%s', joints=%zu",
                    group_name.c_str(), base_frame.c_str(),
                    tip_frames[0].c_str(), joint_names_.size());

        initialized_ = true;
        return true;
    }

    fairino_planning::JointArray FairinoIKPlugin::pickClosest(
        const std::vector<fairino_planning::JointArray> &sols,
        const fairino_planning::JointArray &seed) const
    {
        auto wrapDiff = [](double a)
        {
            a = std::fmod(a + M_PI, 2.0 * M_PI);
            if (a <= 0)
                a += 2.0 * M_PI;
            return a - M_PI;
        };

        double best_dist = std::numeric_limits<double>::infinity();
        fairino_planning::JointArray best = sols.front();

        for (const auto &q : sols)
        {
            double d2 = 0.0;
            for (int i = 0; i < fairino_planning::DOF; ++i)
            {
                const double diff = wrapDiff(q[i] - seed[i]);
                d2 += diff * diff;
            }
            if (d2 < best_dist)
            {
                best_dist = d2;
                best = q;
            }
        }
        return best;
    }

    bool FairinoIKPlugin::withinLimits(
        const fairino_planning::JointArray &q) const
    {
        for (int i = 0; i < fairino_planning::DOF; ++i)
        {
            if (q[i] < joint_lower_[i] - 1e-6 ||
                q[i] > joint_upper_[i] + 1e-6)
            {
                return false;
            }
        }
        return true;
    }

    fairino_planning::JointArray FairinoIKPlugin::reboundToSeed(
        const fairino_planning::JointArray &q,
        const fairino_planning::JointArray &seed) const
    {
        fairino_planning::JointArray out = q;
        for (int i = 0; i < fairino_planning::DOF; ++i)
        {
            while (out[i] - seed[i] > M_PI)
                out[i] -= 2.0 * M_PI;
            while (out[i] - seed[i] < -M_PI)
                out[i] += 2.0 * M_PI;
            // 重绑后再次检查限位
            if (out[i] < joint_lower_[i] || out[i] > joint_upper_[i])
            {
                out[i] = q[i]; // 还原（限位优先）
            }
        }
        return out;
    }

    bool FairinoIKPlugin::getPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,        // 目标末端位姿（在 base_frame 中表达）
        const std::vector<double> &ik_seed_state,       // 种子关节角（6 个 double，多解时选最近的）
        std::vector<double> &solution,                  // 输出参数：求得的关节角
        moveit_msgs::msg::MoveItErrorCodes &error_code, // 输出参数：错误码
        const kinematics::KinematicsQueryOptions &options) const
    {
        // IK 插件核心函数： 给一个目标位姿 + 种子关节角，返回一组关节解。
        // 调用的是 fairino_planning_core 中的函数

        // ① 前置检查：插件是否已初始化
        if (!initialized_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "FairinoIKPlugin not initialized.");
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return false;
        }

        // ② 输入校验：种子必须是 6 维
        if (ik_seed_state.size() != fairino_planning::DOF)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Seed state size %zu != DOF %d",
                         ik_seed_state.size(), fairino_planning::DOF);
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        // 将 ik_pose 转换为 Pose(Eigen::Isometry3d) 类型
        // 接着求解 正运动学
        Eigen::Isometry3d target;
        tf2::fromMsg(ik_pose, target);

        // ============================================================
        // ④ 坐标系修正：base/tip offset
        //    用户给的是 base_frame → tip_frame 的变换
        //    core 的 IK 期望 DH 第一轴坐标系 → DH 末端坐标系 的变换
        // ============================================================
        const Eigen::Isometry3d target_dh =
            base_offset_.inverse() * target * tip_offset_.inverse();

        std::vector<fairino_planning::JointArray> sols = ik_->solve(target_dh);

        if (sols.empty())
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        // ============================================================
        // ⑥ 关节限位过滤
        // ============================================================
        std::vector<fairino_planning::JointArray> valid;
        valid.reserve(sols.size());
        for (const auto &q : sols)
        {
            if (withinLimits(q))
            {
                valid.push_back(q);
            }
        }

        if (valid.empty())
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        // ============================================================
        // ⑦ 多解选优：选离 seed 最近的（关节空间 L2 距离）
        //    考虑关节周期性（角度差需归一化到 [-pi, pi]）
        // ============================================================
        fairino_planning::JointArray seed;
        std::copy(ik_seed_state.begin(), ik_seed_state.end(), seed.begin());

        fairino_planning::JointArray best = pickClosest(valid, seed);

        // 工业上常常需要：解出 q = -3.0 rad 但 seed = 3.0 rad，
        // 两者数值上差 6 rad，但实际只差 0.28 rad（一圈）。要让机械臂走最短路径：
        best = reboundToSeed(best, seed);
        solution.assign(best.begin(), best.end());

        // ============================================================
        // ⑧ （可选）调用用户回调：碰撞/约束检查
        //    在 searchPositionIK 里更常用，这里也可以支持
        // ============================================================
        // 此处略，getPositionIK 不带 solution_callback 参数

        // ============================================================
        // ⑨ FK 自检：把解算回末端位姿，与目标对比，确保数值精度
        //    （core 已经做过 FK 校验，这里通常可以跳过）
        // ============================================================
        // 略

        // ============================================================
        // ⑩ 写出结果
        // ============================================================
        solution.assign(best.begin(), best.end());
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // 重载版本
    bool FairinoIKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        // 解析 IK 在 ms 内完成，timeout 参数对我们无意义，直接转发
        (void)timeout;

        return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
    }

    // ─────────────────────────────────────────────────────────────
    // 重载 2：带 consistency_limits（限制解距离 seed 的范围）
    // ─────────────────────────────────────────────────────────────
    bool FairinoIKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        static const IKCallbackFn empty_cb;
        return searchPositionIK(ik_pose, ik_seed_state, timeout,
                                consistency_limits, solution,
                                empty_cb, error_code, options);
    }

    // ─────────────────────────────────────────────────────────────
    // 重载 3：带 solution_callback（回调用于碰撞/约束检查）
    // ─────────────────────────────────────────────────────────────
    bool FairinoIKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        static const std::vector<double> empty_limits;
        return searchPositionIK(ik_pose, ik_seed_state, timeout,
                                empty_limits, solution,
                                solution_callback, error_code, options);
    }

    // ─────────────────────────────────────────────────────────────
    // 重载 4：完整版（所有参数）—— 真正的实现在这里 ⭐
    // ─────────────────────────────────────────────────────────────
    bool FairinoIKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        // 返回第一个通过碰撞检测的解，可能不是最近的
        (void)timeout; // 解析 IK 不需要

        // ① 前置检查
        if (!initialized_)
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return false;
        }
        if (ik_seed_state.size() != fairino_planning::DOF)
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        // ② ROS Pose → Eigen
        Eigen::Isometry3d target;
        tf2::fromMsg(ik_pose, target);
        const Eigen::Isometry3d target_dh =
            base_offset_.inverse() * target * tip_offset_.inverse();

        // ③ 解析 IK（最多 8 解）
        std::vector<fairino_planning::JointArray> sols = ik_->solve(target_dh);
        if (sols.empty())
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        // ④ seed 转换
        fairino_planning::JointArray seed;
        std::copy(ik_seed_state.begin(), ik_seed_state.end(), seed.begin());

        // ⑤ 过滤 + 重绑 + 排序：按"距 seed 远近"排好
        struct Candidate
        {
            fairino_planning::JointArray q;
            double dist;
        };
        std::vector<Candidate> cands;
        cands.reserve(sols.size());

        for (auto q : sols)
        {
            // 重绑到 seed 周期
            q = reboundToSeed(q, seed);

            // 限位过滤
            if (!withinLimits(q))
                continue;

            // 一致性约束检查
            if (!consistency_limits.empty())
            {
                bool ok = true;
                for (int i = 0; i < fairino_planning::DOF; ++i)
                {
                    if (std::abs(q[i] - seed[i]) > consistency_limits[i])
                    {
                        ok = false;
                        break;
                    }
                }
                if (!ok)
                    continue;
            }

            // 计算距离
            double d2 = 0.0;
            for (int i = 0; i < fairino_planning::DOF; ++i)
            {
                const double diff = q[i] - seed[i];
                d2 += diff * diff;
            }
            cands.push_back({q, d2});
        }

        if (cands.empty())
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return false;
        }

        std::sort(cands.begin(), cands.end(),
                  [](const Candidate &a, const Candidate &b)
                  {
                      return a.dist < b.dist;
                  });

        // ⑥ 按距离顺序逐个尝试，第一个通过 callback 的胜出
        // callback 一般都是 检测路径是否可行，会不会撞墙。
        // 这个是和 getPositionIK 的最大的区别， getPositionIK 不能检测
        for (const auto &c : cands)
        {
            std::vector<double> sol_vec(c.q.begin(), c.q.end());

            if (solution_callback)
            {
                moveit_msgs::msg::MoveItErrorCodes cb_err;
                solution_callback(ik_pose, sol_vec, cb_err);
                if (cb_err.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                {
                    continue; // 这个解不通过（碰撞/约束失败），试下一个
                }
            }

            solution = sol_vec;
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            return true;
        }

        // 所有解都被 callback 拒绝
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    const std::vector<std::string> &FairinoIKPlugin::getJointNames() const
    {
        return joint_names_;
    }
    const std::vector<std::string> &FairinoIKPlugin::getLinkNames() const
    {
        return link_names_;
    }

    // ─────────────────────────────────────────────────────────
    // getPositionFK：给定关节角，计算指定 link 的位姿
    // ─────────────────────────────────────────────────────────
    bool FairinoIKPlugin::getPositionFK(
        const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::msg::Pose> &poses) const
    {
        // ① 前置检查
        if (!initialized_)
        {
            return false;
        }
        if (joint_angles.size() != fairino_planning::DOF)
        {
            return false;
        }

        // ② 把 vector<double> 转成 JointArray
        fairino_planning::JointArray q;
        std::copy(joint_angles.begin(), joint_angles.end(), q.begin());

        // ③ 调 core 算 FK：返回所有连杆位姿（含 base，共 DOF+1 个）
        auto link_transforms = fk_->linkTransforms(q);

        // ④ 按请求的 link_names 取出对应位姿
        poses.clear();
        poses.reserve(link_names.size());

        for (const auto &name : link_names)
        {
            // 在 link_names_ 里找索引
            auto it = std::find(link_names_.begin(), link_names_.end(), name);
            if (it == link_names_.end())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "FK: link '%s' not in plugin's link list.", name.c_str());
                return false;
            }
            const size_t idx = std::distance(link_names_.begin(), it);

            // ⑤ idx 要小于 link_transforms.size()
            //    通常 link_names_[0] = base_link，对应 link_transforms[0]
            //    后续每个关节后的连杆依次对应
            if (idx >= link_transforms.size())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "FK: link '%s' index out of range.", name.c_str());
                return false;
            }

            // ⑥ 应用 base / tip 偏移
            Eigen::Isometry3d T_world = base_offset_ * link_transforms[idx];

            // ⑦ Eigen → ROS msg
            poses.push_back(tf2::toMsg(T_world));
        }

        return true;
    }
}
// 导出宏
PLUGINLIB_EXPORT_CLASS(fairino_planning_ros::FairinoIKPlugin, kinematics::KinematicsBase)
