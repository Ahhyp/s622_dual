#pragma once

#include <map>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "s622_bt_manager/perception_nodes.hpp"

namespace s622_bt
{

    class GenerateGraspCandidate : public BT::SyncActionNode
    {
    public:
        GenerateGraspCandidate(const std::string &name,
                               const BT::NodeConfig &config,
                               RosContextPtr ros);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        RosContextPtr ros_;
    };

    void registerGraspNodes(BT::BehaviorTreeFactory &factory, RosContextPtr ros);
    
    class GeneratePlaceCandidate : public BT::SyncActionNode
    {
    public:
        GeneratePlaceCandidate(const std::string &name,
                               const BT::NodeConfig &config,
                               rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("place_pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("pre_place_pose"),
                BT::InputPort<std::string>("arm_prefix", "",
                    "'' -> place_*; 'left' -> left_place_*"),  // ← 新增
            };
        }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        // ← 改为按 arm_prefix 缓存 pose (支持一个 BT process 里多个 arm)
        std::map<std::string, geometry_msgs::msg::PoseStamped> place_cache_;
        std::map<std::string, geometry_msgs::msg::PoseStamped> pre_place_cache_;

        // helper: 按 arm_prefix 加载并缓存
        void ensure_loaded(const std::string &arm_prefix);
        
    };
} // namespace s622_bt