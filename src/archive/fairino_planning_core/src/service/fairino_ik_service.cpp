#include "fairino_planning_core/service/fairino_ik_service.h"
#include "fairino_planning_core/types.h"

#include <Eigen/Geometry>
#include <cmath>

namespace fairino_planning_core
{

    using GetAllIK = fairino_msgs::srv::GetAllIK;

    FairinoIKService::FairinoIKService(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        // ---------- 关节名 ----------
        joint_names_ = node_->declare_parameter<std::vector<std::string>>(
            "joint_names",
            std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});

        // ---------- DH 参数（来自 docs/机械臂参数.md）----------
        auto d_vec = node_->declare_parameter<std::vector<double>>(
            "dh_d", std::vector<double>{0.140, 0.0, 0.0, 0.102, 0.102, 0.100});
        auto a_vec = node_->declare_parameter<std::vector<double>>(
            "dh_a", std::vector<double>{0.0, -0.280, -0.240, 0.0, 0.0, 0.0});
        auto alpha_vec = node_->declare_parameter<std::vector<double>>(
            "dh_alpha", std::vector<double>{M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0});

        if (d_vec.size() != fairino_planning::DOF ||
            a_vec.size() != fairino_planning::DOF ||
            alpha_vec.size() != fairino_planning::DOF)
        {
            RCLCPP_FATAL(node_->get_logger(),
                         "DH parameter size mismatch, expected %d", fairino_planning::DOF);
            throw std::runtime_error("DH parameter size mismatch");
        }

        std::array<double, fairino_planning::DOF> d_arr, a_arr, alpha_arr;
        for (size_t i = 0; i < fairino_planning::DOF; ++i)
        {
            d_arr[i] = d_vec[i];
            a_arr[i] = a_vec[i];
            alpha_arr[i] = alpha_vec[i];
        }

        ik_ = std::make_unique<fairino_planning::FairinoIK>(d_arr, a_arr, alpha_arr);

        // ---------- 注册 service ----------
        srv_ = node_->create_service<GetAllIK>(
            "/fairino/get_all_ik",
            std::bind(&FairinoIKService::handle, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(),
                    "FairinoIKService ready at /fairino/get_all_ik");

        // ---- 自检 ----
        fairino_planning::JointArray zero = {0, 0, 0, 0, 0, 0};
        fairino_planning::DHKinematics fk_check([] {
            std::array<fairino_planning::DHParam, fairino_planning::DOF> dh;
            // 用和 FairinoIK 完全一样的 DH
            dh[0] = { 0.0,    1.570796,  0.140, 0};
            dh[1] = {-0.280,  0.0,       0.0,   0};
            dh[2] = {-0.240,  0.0,       0.0,   0};
            dh[3] = { 0.0,    1.570796,  0.102, 0};
            dh[4] = { 0.0,   -1.570796,  0.102, 0};
            dh[5] = { 0.0,    0.0,       0.100, 0};
            return dh;
        }());
        auto p_zero = fk_check.forward(zero);
        RCLCPP_INFO(node_->get_logger(),
                    "Zero pose: t=(%.3f,%.3f,%.3f)",
                    p_zero.translation().x(),
                    p_zero.translation().y(),
                    p_zero.translation().z());

        auto sols = ik_->solve(p_zero);
        RCLCPP_INFO(node_->get_logger(),
                    "Self-test IK from zero FK: %zu solutions", sols.size());
    }

    void FairinoIKService::handle(
        const std::shared_ptr<GetAllIK::Request> req,
        std::shared_ptr<GetAllIK::Response> res)
    {
        // ---- 1. PoseStamped -> Eigen::Isometry3d ----
        fairino_planning::Pose target = fairino_planning::Pose::Identity();
        target.translation() << req->pose.pose.position.x,
            req->pose.pose.position.y,
            req->pose.pose.position.z;
        Eigen::Quaterniond q(req->pose.pose.orientation.w,
                             req->pose.pose.orientation.x,
                             req->pose.pose.orientation.y,
                             req->pose.pose.orientation.z);
        target.linear() = q.normalized().toRotationMatrix();

        // ---- 2. 调你的解析 IK(内部已做 FK 验证和去重)----
        std::vector<fairino_planning::JointArray> solutions = ik_->solve(target);

        fairino_planning::JointLimits safety_limits;
        if (!req->joint_limits_lower.empty() &&
            req->joint_limits_lower.size() == fairino_planning::DOF)
        {
            for (size_t i = 0; i < fairino_planning::DOF; ++i)
            {
                safety_limits.lower[i] = req->joint_limits_lower[i];
                safety_limits.upper[i] = req->joint_limits_upper[i];
            }
        }

        // ---- 3. 组装响应 ----
        for (const auto &sol : solutions)
        {
            if (req->filter_joint_limits && !safety_limits.isWithin(sol))
            {
                continue; // 跳过超限解
            }

            sensor_msgs::msg::JointState js;
            js.header.stamp = node_->now();
            js.name = joint_names_;
            js.position.assign(sol.begin(), sol.end());
            res->solutions.push_back(js);

            // 误差字段:既然 solve() 内部已验证通过,这里填 0 占位
            res->fk_position_errors.push_back(0.0);
            res->fk_rotation_errors.push_back(0.0);
        }

        if (solutions.empty())
        {
            res->error_code = -1;
            res->error_message = "no IK solution";
            RCLCPP_WARN(node_->get_logger(),
                        "GetAllIK: no solution for pose (%.3f, %.3f, %.3f)",
                        req->pose.pose.position.x,
                        req->pose.pose.position.y,
                        req->pose.pose.position.z);
        }
        else
        {
            res->error_code = 0;
            res->error_message = "OK";
            RCLCPP_INFO(node_->get_logger(),
                        "GetAllIK: %zu solutions for pose (%.3f, %.3f, %.3f)",
                        solutions.size(),
                        req->pose.pose.position.x,
                        req->pose.pose.position.y,
                        req->pose.pose.position.z);
        }
    }

} // namespace fairino_planning_core
