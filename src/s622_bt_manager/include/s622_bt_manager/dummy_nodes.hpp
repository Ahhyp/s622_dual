#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

namespace s622_bt
{

    // 2026-08-27 S3：从 dummy 改为真检查——调 /controller_manager/list_controllers，
    // 要求所有已加载 controller 全 active（双臂 5 个 / 单臂 3 个），
    // 否则 FAILURE（避免 controller 未就绪时 BT 继续执行白跑）。
    class CheckSystemReady : public BT::SyncActionNode
    {
    public:
        CheckSystemReady(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_;
    };

    class PrintMessage : public BT::SyncActionNode
    {
    public:
        PrintMessage(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("msg")};
        }
        BT::NodeStatus tick() override;
    };

    class Wait : public BT::SyncActionNode
    {
    public:
        Wait(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<double>("sec")};
        }
        BT::NodeStatus tick() override;
    };

    void registerDummyNodes(BT::BehaviorTreeFactory &factory, rclcpp::Node::SharedPtr node);

} // namespace s622_bt