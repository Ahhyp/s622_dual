#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "s622_bt_manager/srv/attach_object.hpp"
#include "s622_bt_manager/srv/detach_object.hpp"

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
                BT::InputPort<std::string>("link_name", "grasp_frame", ""),
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
                         rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_name", "cube", ""),
                BT::InputPort<bool>("put_back_in_world", true, ""),
                BT::InputPort<geometry_msgs::msg::PoseStamped>(
                    "drop_pose", "where to put it back (base_link frame)"),
                BT::InputPort<double>("timeout_sec", 3.0, ""),
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<s622_bt_manager::srv::DetachObject>::SharedPtr client_;
    };

    void registerSceneNodes(BT::BehaviorTreeFactory &factory,
                            rclcpp::Node::SharedPtr node);

} // namespace s622_bt