#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "s622_bt_manager/srv/attach_object.hpp"
#include "s622_bt_manager/srv/detach_object.hpp"
#include "s622_bt_manager/srv/transfer_object.hpp"
#include "s622_bt_manager/perception_nodes.hpp"

namespace s622_bt
{
    class AttachObjectNode : public BT::SyncActionNode
    {
    public:
        AttachObjectNode(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_name", "cube", ""),
                // link_name 空串时按 arm_prefix 自动拼 '{arm_prefix}_grasp_frame'
                // arm_prefix="" 且 link_name="" 时 fallback 到 'grasp_frame' (M1.7 兼容)
                BT::InputPort<std::string>("link_name", "", ""),
                BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right'"),
                BT::InputPort<double>("size_x", 0.04, ""),
                BT::InputPort<double>("size_y", 0.04, ""),
                BT::InputPort<double>("size_z", 0.04, ""),
                BT::InputPort<double>("offset_z", 0.02,
                                      "pose.z in link frame (cube center below TCP)"),
                BT::InputPort<double>("timeout_sec", 3.0, ""),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<s622_bt_manager::srv::AttachObject>::SharedPtr client_;
    };

    class DetachObjectNode : public BT::SyncActionNode
    {
    public:
        DetachObjectNode(const std::string &name, const BT::NodeConfig &config,
                         RosContextPtr ros);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_name", "cube", ""),
                BT::InputPort<bool>("put_back_in_world", true, ""),
                BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right'"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>(
                    "drop_pose", "where to put it back (base_link frame)"),
                BT::InputPort<double>("timeout_sec", 3.0, ""),
            };
        }
        BT::NodeStatus tick() override;

    private:
        RosContextPtr ros_;
        rclcpp::Client<s622_bt_manager::srv::DetachObject>::SharedPtr client_;
    };

    class TransferObjectNode : public BT::SyncActionNode
    {
    public:
        TransferObjectNode(const std::string &name, const BT::NodeConfig &config,
                           rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_name", "cube", ""),
                // 目标 arm: 从 old_arm_prefix 转到 new_arm_prefix
                BT::InputPort<std::string>("new_arm_prefix", "",
                                           "'left' | 'right' - derives new_link_name"),
                // 或直接给 new_link_name (覆盖派生)
                BT::InputPort<std::string>("new_link_name", "", ""),
                BT::InputPort<double>("offset_z", 0.02,
                                      "cube pose z in new link frame"),
                BT::InputPort<double>("timeout_sec", 3.0, ""),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<s622_bt_manager::srv::TransferObject>::SharedPtr client_;
    };

    void registerSceneNodes(BT::BehaviorTreeFactory &factory,
                            RosContextPtr ros);

} // namespace s622_bt