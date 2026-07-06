#pragma once

#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "s622_bt_manager/action/visual_align.hpp"

namespace s622_bt
{

    class VisualAlignAction : public BT::StatefulActionNode
    {
    public:
        using ActionT = s622_bt_manager::action::VisualAlign;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

        VisualAlignAction(const std::string &name,
                          const BT::NodeConfig &config,
                          rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("mode"),
                BT::InputPort<double>("timeout_sec", 25.0, ""),
                BT::InputPort<bool>("ensure_servo_started", true, ""),
                // mode=xy
                BT::InputPort<double>("target_x_base", 0.0, ""),
                BT::InputPort<double>("target_y_base", 0.0, ""),
                BT::InputPort<double>("tolerance_m", 0.005, ""),
                // mode=yaw
                BT::InputPort<double>("target_yaw", 0.0, ""),
                BT::InputPort<double>("tolerance_rad", 0.05, ""),
                // mode=descend/lift/retreat
                BT::InputPort<double>("distance", 0.0, ""),
                BT::InputPort<double>("speed", 0.04, ""),
            };
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ActionT>::SharedPtr client_;
        std::shared_future<GoalHandle::SharedPtr> goal_future_;
        GoalHandle::SharedPtr goal_handle_;
        std::shared_future<GoalHandle::WrappedResult> result_future_;
        rclcpp::Time start_time_;
        float timeout_sec_ = 25.0f;
    };

    class StopServoNode : public BT::SyncActionNode
    {
    public:
        StopServoNode(const std::string &name, const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    };

    void registerServoNodes(BT::BehaviorTreeFactory &factory,
                            rclcpp::Node::SharedPtr node);

} // namespace s622_bt