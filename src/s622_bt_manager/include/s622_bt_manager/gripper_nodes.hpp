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
                BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right'"),
                BT::InputPort<std::string>("command", "open|close"),
                BT::InputPort<float>("timeout_sec", 5.0f, ""),
                BT::OutputPort<float>("finger_position"),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<s622_bt_manager::srv::SetGripper>::SharedPtr client_;
        std::string client_arm_prefix_ = "__UNSET__";   
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
                // BT::InputPort<std::string>("feedback_joint", "finger1_joint", ""),
                BT::InputPort<std::string>("feedback_joint", "", ""),
                BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right'"),  
                BT::InputPort<float>("timeout_sec", 2.0f, ""),
                // [M4 BT 回填] 真实咬合判据：|f1|+|f2| ≈ 指板间隙（比单指 f1 稳）。
                // gap_min/gap_max 均 >0 时启用；否则沿用旧的三段 f1 判定。
                BT::InputPort<float>("gap_min", 0.0f, "|f1|+|f2| 下限(m)，>0 启用"),
                BT::InputPort<float>("gap_max", 0.0f, "|f1|+|f2| 上限(m)"),
                // 力控夹爪停下后会缓慢漂移 → 稳定阈值可放宽（默认保留旧值 5e-4）
                BT::InputPort<float>("settle_tol", 5e-4f, "帧间变化阈值(m)"),
                BT::InputPort<float>("min_wait_sec", 1.5f, "最小等待(sim s)"),
                BT::OutputPort<float>("finger_position"),
                BT::OutputPort<float>("finger_gap", "[M4] |f1|+|f2|"),
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