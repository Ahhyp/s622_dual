#include "s622_bt_manager/motion_nodes.hpp"
#include <cmath>

namespace s622_bt
{

    MoveToPoseAction::MoveToPoseAction(const std::string &name,
                                       const BT::NodeConfig &config,
                                       rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node))
    {
        client_ = rclcpp_action::create_client<ActionT>(node_, "move_to_pose");
    }

    BT::NodeStatus MoveToPoseAction::onStart()
    {
        geometry_msgs::msg::PoseStamped target;
        std::string named_pose;
        float v_scale = 0.2f, a_scale = 0.2f, timeout = 20.0f;
        bool stop_servo = true;

        // target_pose 仅在 named_pose 为空时必填
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

        if (!client_->wait_for_action_server(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "move_to_pose action server not available");
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
            double roll  = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            double pitch = std::asin(2.0 * (qw * qy - qz * qx));
            double yaw   = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
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
                        "MoveToPose: sending goal (named=%s, v=%.2f, a=%.2f, to=%.1fs)",
                        named_pose.c_str(), v_scale, a_scale, timeout);
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

    void registerMotionNodes(BT::BehaviorTreeFactory &factory,
                             rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<MoveToPoseAction>(
            "MoveToPose",
            [node](const std::string &name, const BT::NodeConfig &config)
            {
                return std::make_unique<MoveToPoseAction>(name, config, node);
            });
    }

} // namespace s622_bt