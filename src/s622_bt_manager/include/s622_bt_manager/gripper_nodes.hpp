#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "s622_bt_manager/srv/set_gripper.hpp"

namespace s622_bt
{

    class SetGripperNode : public BT::SyncActionNode
    {
    public:
        SetGripperNode(const std::string &name,
                       const BT::NodeConfig &config,
                       rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("command", "open|close"),
                BT::InputPort<float>("timeout_sec", 5.0f, ""),
                BT::OutputPort<float>("finger_position"),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<s622_bt_manager::srv::SetGripper>::SharedPtr client_;
    };

    class VerifyGraspNode : public BT::SyncActionNode
    {
    public:
        VerifyGraspNode(const std::string &name,
                        const BT::NodeConfig &config,
                        rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<float>("finger_min_position", 0.005f,
                                     "below this -> empty grasp -> FAILURE"),
                BT::InputPort<std::string>("feedback_joint", "finger1_joint", ""),
                BT::InputPort<float>("timeout_sec", 2.0f, ""),
                BT::OutputPort<float>("finger_position"),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::mutex mu_;
        sensor_msgs::msg::JointState::SharedPtr latest_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    };

    void registerGripperNodes(BT::BehaviorTreeFactory &factory,
                              rclcpp::Node::SharedPtr node);

} // namespace s622_bt