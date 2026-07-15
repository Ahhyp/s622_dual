#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "s622_bt_manager/action/move_to_pose.hpp"
#include "s622_bt_manager/action/dual_move_to_joint_state.hpp"

namespace s622_bt
{

    class MoveToPoseAction : public BT::StatefulActionNode
    {
    public:
        using ActionT = s622_bt_manager::action::MoveToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

        MoveToPoseAction(const std::string &name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
                BT::InputPort<std::string>("named_pose", "", "if non-empty, use named pose"),
                BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right'"),
                BT::InputPort<float>("velocity_scale", 0.2f, ""),
                BT::InputPort<float>("acceleration_scale", 0.2f, ""),
                BT::InputPort<float>("timeout_sec", 20.0f, ""),
                BT::InputPort<bool>("ensure_servo_stopped", true, ""),
            };
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ActionT>::SharedPtr client_;

        std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
        GoalHandle::SharedPtr goal_handle_;
        std::shared_future<GoalHandle::WrappedResult> result_future_;
        std::string client_arm_prefix_ = "__UNSET__";

        bool result_received_ = false;
        GoalHandle::WrappedResult last_result_;

        rclcpp::Time start_time_;
        float timeout_sec_ = 20.0f;
    };

    class DualMoveToJointStateAction : public BT::StatefulActionNode
    {
    public:
        using DualActionT = s622_bt_manager::action::DualMoveToJointState;
        using GoalHandle = rclcpp_action::ClientGoalHandle<DualActionT>;

        DualMoveToJointStateAction(const std::string &name,
                                   const BT::NodeConfig &config,
                                   rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return
            {
                // BT::InputPort<std::string>("left_positions", "", "");
                // BT::InputPort<std::string>("right_positions", "", "");
                BT::InputPort<std::vector<double>>("left_positions", {}, ""),
                BT::InputPort<std::vector<double>>("right_positions", {}, ""),
                BT::InputPort<std::string>("named_pose", "", "'' | 'dual_home'"),
                BT::InputPort<float>("velocity_scale", 0.2f, ""),
                BT::InputPort<float>("acceleration_scale", 0.2f, ""),
                BT::InputPort<float>("timeout_sec", 20.0f, ""),
            };
        }
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<DualActionT>::SharedPtr client_;
        
        std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
        GoalHandle::SharedPtr goal_handle_;
        std::shared_future<GoalHandle::WrappedResult> result_future_;
        bool result_received_ = false;
        rclcpp::Time start_time_;
        float timeout_sec_ = 20.0f;
    };

    void
    registerMotionNodes(BT::BehaviorTreeFactory &factory,
                        rclcpp::Node::SharedPtr node);

} // namespace s622_bt