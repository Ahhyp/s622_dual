#include "s622_bt_manager/servo_nodes.hpp"
#include <chrono>
namespace s622_bt
{

    using namespace std::chrono_literals;

    // ---- helper: build action/service name from prefix ----
    static std::string action_name_for(const std::string &prefix,
                                       const std::string &base)
    {
        if (prefix.empty())
            return base;
        return "/" + prefix + "/" + base;
    }
    static std::string service_name_for(const std::string &prefix,
                                        const std::string &base)
    {
        if (prefix.empty())
            return base; // base 已包含开头 "/"
        // base like "/servo_node/stop_servo" -> "/left/servo_node/stop_servo"
        return "/" + prefix + base;
    }

    VisualAlignAction::VisualAlignAction(const std::string &name,
                                         const BT::NodeConfig &config,
                                         rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node))
    {
        // client_ = rclcpp_action::create_client<ActionT>(node_, "visual_align");
        // 不再在构造时建 client, 因为要按 tick 时的 arm_prefix 决定
        client_arm_prefix_ = "__UNSET__"; // sentinel
    }

    BT::NodeStatus VisualAlignAction::onStart()
    {
        std::string mode;
        if (!getInput("mode", mode))
            return BT::NodeStatus::FAILURE;

        std::string arm_prefix = "";
        getInput("arm_prefix", arm_prefix);

        // 若 prefix 变化(或首次), 重建 client
        if (arm_prefix != client_arm_prefix_)
        {
            const auto action_name = action_name_for(arm_prefix, "visual_align");
            client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
            client_arm_prefix_ = arm_prefix;
            RCLCPP_INFO(node_->get_logger(),
                        "VisualAlign: bound to action '%s'", action_name.c_str());
        }

        double t_x = 0.0, t_y = 0.0;
        double tol_m = 0.005f;
        double t_yaw = 0.0f, tol_rad = 0.05f;
        double dist = 0.0f, speed = 0.04f, timeout = 25.0f;
        bool ess = true;
        getInput("target_x_base", t_x);
        getInput("target_y_base", t_y);
        getInput("tolerance_m", tol_m);
        getInput("target_yaw", t_yaw);
        getInput("tolerance_rad", tol_rad);
        getInput("distance", dist);
        getInput("speed", speed);
        getInput("timeout_sec", timeout);
        getInput("ensure_servo_started", ess);
        timeout_sec_ = timeout;

        if (!client_->wait_for_action_server(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "visual_align server unavailable (arm=%s)",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        ActionT::Goal g;
        g.mode = mode;
        g.target_x_base = static_cast<float>(t_x);
        g.target_y_base = static_cast<float>(t_y);
        g.tolerance_m = static_cast<float>(tol_m);
        g.target_yaw = static_cast<float>(t_yaw);
        g.tolerance_rad = static_cast<float>(tol_rad);
        g.distance = static_cast<float>(dist);
        g.speed = static_cast<float>(speed);
        g.timeout_sec = static_cast<float>(timeout);
        g.ensure_servo_started = static_cast<float>(ess);

        RCLCPP_INFO(node_->get_logger(),
                    "VisualAlign[%s arm=%s]: dist=%.3f speed=%.3f to=%.1f "
                    "target_xy=(%.3f,%.3f) tol_m=%.3f target_yaw=%.3f tol_rad=%.3f",
                    mode.c_str(), arm_prefix.c_str(),
                    dist, speed, timeout, t_x, t_y, tol_m, t_yaw, tol_rad);
        rclcpp_action::Client<ActionT>::SendGoalOptions opts;
        goal_future_ = client_->async_send_goal(g, opts);
        goal_handle_.reset();
        start_time_ = node_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus VisualAlignAction::onRunning()
    {
        if ((node_->now() - start_time_).seconds() > timeout_sec_ + 5.0f)
        {
            onHalted();
            return BT::NodeStatus::FAILURE;
        }
        if (!goal_handle_)
        {
            if (goal_future_.wait_for(0ms) == std::future_status::ready)
            {
                goal_handle_ = goal_future_.get();
                if (!goal_handle_)
                    return BT::NodeStatus::FAILURE;
                result_future_ = client_->async_get_result(goal_handle_);
            }
            return BT::NodeStatus::RUNNING;
        }
        if (result_future_.wait_for(0ms) == std::future_status::ready)
        {
            auto wrapped = result_future_.get();
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result && wrapped.result->success)
            {
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(node_->get_logger(), "VisualAlign FAILED: %s",
                         wrapped.result ? wrapped.result->error_msg.c_str() : "no result");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void VisualAlignAction::onHalted()
    {
        if (goal_handle_)
            client_->async_cancel_goal(goal_handle_);
    }

    StopServoNode::StopServoNode(const std::string &name,
                                 const BT::NodeConfig &config,
                                 rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(std::move(node))
    {
        // client_ = node_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
        client_arm_prefix_ = "__UNSET__";
    }

    BT::NodeStatus StopServoNode::tick()
    {
        std::string arm_prefix = "";
        getInput("arm_prefix", arm_prefix);
        if (arm_prefix != client_arm_prefix_)
        {
            const auto srv_name = service_name_for(arm_prefix, "/servo_node/stop_servo");
            client_ = node_->create_client<std_srvs::srv::Trigger>(srv_name);
            client_arm_prefix_ = arm_prefix;
            RCLCPP_INFO(node_->get_logger(),
                        "StopServo: bound to service '%s'", srv_name.c_str());
        }
        if (!client_->wait_for_service(2s))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "stop_servo service unavailable (arm=%s)",
                        arm_prefix.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(req);
        auto deadline = node_->now() + rclcpp::Duration::from_seconds(3.0);
        while (rclcpp::ok() && node_->now() < deadline)
        {
            if (future.wait_for(0ms) == std::future_status::ready)
                break;
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(node_->get_logger(), "stop_servo called (arm=%s)", arm_prefix.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void registerServoNodes(BT::BehaviorTreeFactory &factory,
                            rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<VisualAlignAction>(
            "VisualAlign",
            [node](const std::string &n, const BT::NodeConfig &c)
            {
                return std::make_unique<VisualAlignAction>(n, c, node);
            });
        factory.registerBuilder<StopServoNode>(
            "StopServo",
            [node](const std::string &n, const BT::NodeConfig &c)
            {
                return std::make_unique<StopServoNode>(n, c, node);
            });
    }

} // namespace s622_bt