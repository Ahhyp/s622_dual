#pragma once
#include <behaviortree_cpp/bt_factory.h>

namespace s622_bt
{

    class CheckSystemReady : public BT::SyncActionNode
    {
    public:
        CheckSystemReady(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
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

    void registerDummyNodes(BT::BehaviorTreeFactory &factory);

} // namespace s622_bt