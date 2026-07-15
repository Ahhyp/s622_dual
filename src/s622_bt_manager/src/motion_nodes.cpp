#include "s622_bt_manager/motion_nodes.hpp"
#include <cmath>
#include <string>
namespace s622_bt
{

    // ---- helper: build action name from prefix ----
    static std::string action_name_for(const std::string &prefix,
                                       const std::string &base)
    {
        if (prefix.empty())
            return base;
        return "/" + prefix + "/" + base;
    }

    MoveToPoseAction::MoveToPoseAction(const std::string &name,
                                       const BT::NodeConfig &config,
                                       rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node))
    {
        // client_ = rclcpp_action::create_client<ActionT>(node_, "move_to_pose");
    }

    BT::NodeStatus MoveToPoseAction::onStart()
    {
        geometry_msgs::msg::PoseStamped target;
        std::string named_pose, arm_prefix;
        float v_scale = 0.2f, a_scale = 0.2f, timeout = 20.0f;
        bool stop_servo = true;

        getInput("arm_prefix", arm_prefix);
        getInput("named_pose", named_pose);
        if (named_pose.empty())
        {
            if (!getInput("target_pose", target))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "MoveToPose: target_pose required when named_pose empty");
                return BT::NodeStatus::FAILURE;
            }
        }
        getInput("velocity_scale", v_scale);
        getInput("acceleration_scale", a_scale);
        getInput("timeout_sec", timeout);
        getInput("ensure_servo_stopped", stop_servo);
        timeout_sec_ = timeout;

        // ---- 懒建/重建 client ----
        if (arm_prefix != client_arm_prefix_)
        {
            const auto action_name = action_name_for(arm_prefix, "move_to_pose");
            client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
            client_arm_prefix_ = arm_prefix;
            RCLCPP_INFO(node_->get_logger(),
                        "MoveToPose: bound to action '%s'", action_name.c_str());
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "move_to_pose action server not available (arm=%s)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        ActionT::Goal goal_msg;
        goal_msg.target_pose = target;
        goal_msg.named_pose = named_pose;
        goal_msg.velocity_scale = v_scale;
        goal_msg.acceleration_scale = a_scale;
        goal_msg.timeout_sec = timeout;
        goal_msg.ensure_servo_stopped = stop_servo;

        if (named_pose.empty())
        {
            double qx = target.pose.orientation.x;
            double qy = target.pose.orientation.y;
            double qz = target.pose.orientation.z;
            double qw = target.pose.orientation.w;
            // quaternion -> RPY
            double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            double pitch = std::asin(2.0 * (qw * qy - qz * qx));
            double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            RCLCPP_INFO(node_->get_logger(),
                        "MoveToPose: sending goal (v=%.2f, a=%.2f, to=%.1fs) "
                        "xyz (%.4f, %.4f, %.4f) rpy (%.3f, %.3f, %.3f)",
                        v_scale, a_scale, timeout,
                        target.pose.position.x, target.pose.position.y, target.pose.position.z,
                        roll, pitch, yaw);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "MoveToPose[arm=%s]: sending goal (named=%s, v=%.2f, a=%.2f, to=%.1fs)",
                        arm_prefix.c_str(), named_pose.c_str(),
                        v_scale, a_scale, timeout);
        }

        rclcpp_action::Client<ActionT>::SendGoalOptions opts;
        goal_handle_future_ = client_->async_send_goal(goal_msg, opts);

        goal_handle_.reset();
        result_received_ = false;
        start_time_ = node_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveToPoseAction::onRunning()
    {
        // 超时
        if ((node_->now() - start_time_).seconds() > timeout_sec_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "MoveToPose: timeout after %.1fs", timeout_sec_);
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        // 等待 goal handle
        if (!goal_handle_)
        {
            if (goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                goal_handle_ = goal_handle_future_.get();
                if (!goal_handle_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "MoveToPose: goal rejected");
                    return BT::NodeStatus::FAILURE;
                }
                result_future_ = client_->async_get_result(goal_handle_);
                RCLCPP_INFO(node_->get_logger(), "MoveToPose: goal accepted");
            }
            return BT::NodeStatus::RUNNING;
        }

        // 等待结果
        if (result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto wrapped = result_future_.get();
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result && wrapped.result->success)
            {
                RCLCPP_INFO(node_->get_logger(), "MoveToPose: SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::string err = wrapped.result ? wrapped.result->error_msg : "no result";
                RCLCPP_ERROR(node_->get_logger(),
                             "MoveToPose: FAILED (code=%d, msg=%s)",
                             static_cast<int>(wrapped.code), err.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void MoveToPoseAction::onHalted()
    {
        if (goal_handle_)
        {
            RCLCPP_WARN(node_->get_logger(), "MoveToPose: halted, cancelling goal");
            client_->async_cancel_goal(goal_handle_);
        }
    }

    DualMoveToJointStateAction::DualMoveToJointStateAction(const std::string &name,
                                                           const BT::NodeConfig &config,
                                                           rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node))
    {
        client_ = rclcpp_action::create_client<DualActionT>(node_, "/dual/move_to_joint_state");
    }

    BT::NodeStatus DualMoveToJointStateAction::onStart()
    {
        std::vector<double> left_positions, right_positions;
        std::string named_pose;
        float v_scale = 0.2f, a_scale = 0.2f, timeout = 20.0f;

        getInput("named_pose", named_pose);
        if (named_pose.empty())
        {
            if (!getInput("left_positions", left_positions) || !getInput("right_positions", right_positions))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "DualMoveToJointState: positions required when named_pose empty");
                return BT::NodeStatus::FAILURE;
            }
        }
        getInput("velocity_scale", v_scale);
        getInput("acceleration_scale", a_scale);
        getInput("timeout_sec", timeout);
        timeout_sec_ = timeout;

        if (!client_->wait_for_action_server(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "move_to_joint_state action server not available");
            return BT::NodeStatus::FAILURE;
        }

        DualActionT::Goal goal_msg;
        goal_msg.named_pose = named_pose;
        goal_msg.left_positions = left_positions;
        goal_msg.right_positions = right_positions;
        goal_msg.velocity_scale = v_scale;
        goal_msg.acceleration_scale = a_scale;
        goal_msg.timeout_sec = timeout;

        if (named_pose.empty())
        {
            RCLCPP_INFO(node_->get_logger(),
                        "DualMoveToJointState: sending goal (v=%.2f, a=%.2f, to=%.1fs) "
                        "left positions(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f), right positions(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                        v_scale, a_scale, timeout,
                        left_positions[0], left_positions[1], left_positions[2], left_positions[3], left_positions[4], left_positions[5],
                        right_positions[0], right_positions[1], right_positions[2], right_positions[3], right_positions[4], right_positions[5]);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "DualMoveToJointState: sending goal (named=%s, v=%.2f, a=%.2f, to=%.1fs)",
                        named_pose.c_str(), v_scale, a_scale, timeout);
        }

        rclcpp_action::Client<DualActionT>::SendGoalOptions opts;
        goal_handle_future_ = client_->async_send_goal(goal_msg, opts);

        goal_handle_.reset();
        result_received_ = false;
        start_time_ = node_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus DualMoveToJointStateAction::onRunning()
    {
        // 超时
        if ((node_->now() - start_time_).seconds() > timeout_sec_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "DualMoveToJointState: timeout after %.1fs", timeout_sec_);
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        // 等待 goal handle
        if (!goal_handle_)
        {
            if (goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                goal_handle_ = goal_handle_future_.get();
                if (!goal_handle_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "DualMoveToJointState: goal rejected");
                    return BT::NodeStatus::FAILURE;
                }
                result_future_ = client_->async_get_result(goal_handle_);
                RCLCPP_INFO(node_->get_logger(), "DualMoveToJointState: goal accepted");
            }
            return BT::NodeStatus::RUNNING;
        }

        // 等待结果
        if (result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto wrapped = result_future_.get();
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result && wrapped.result->success)
            {
                RCLCPP_INFO(node_->get_logger(), "DualMoveToJointState: SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::string err = wrapped.result ? wrapped.result->error_msg : "no result";
                RCLCPP_ERROR(node_->get_logger(),
                             "DualMoveToJointState: FAILED (code=%d, msg=%s)",
                             static_cast<int>(wrapped.code), err.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void DualMoveToJointStateAction::onHalted()
    {
        if (goal_handle_)
        {
            RCLCPP_WARN(node_->get_logger(), "DualMoveToJointState: halted, cancelling goal");
            client_->async_cancel_goal(goal_handle_);
        }
    }

    void registerMotionNodes(BT::BehaviorTreeFactory &factory,
                             rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<MoveToPoseAction>(
            "MoveToPose",
            [node](const std::string &name, const BT::NodeConfig &config)
            {
                return std::make_unique<MoveToPoseAction>(name, config, node);
            });
        factory.registerBuilder<DualMoveToJointStateAction>(
            "DualMoveToJointState",
            [node](const std::string &name, const BT::NodeConfig &config)
            {
                return std::make_unique<DualMoveToJointStateAction>(name, config, node);
            });
    }

} // namespace s622_bt