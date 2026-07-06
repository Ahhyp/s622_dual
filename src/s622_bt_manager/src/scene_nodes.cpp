#include "s622_bt_manager/scene_nodes.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace s622_bt
{

    AttachObjectNode::AttachObjectNode(const std::string &name,
                                       const BT::NodeConfig &config,
                                       rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<s622_bt_manager::srv::AttachObject>("attach_object");
    }

    BT::NodeStatus AttachObjectNode::tick()
    {
        double timeout = getInput<double>("timeout_sec").value_or(3.0);

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(node_->get_logger(), "attach_object service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::AttachObject::Request>();
        req->object_name = getInput<std::string>("object_name").value_or("cube");
        req->link_name = getInput<std::string>("link_name").value_or("grasp_frame");
        req->size.x = getInput<double>("size_x").value_or(0.04);
        req->size.y = getInput<double>("size_y").value_or(0.04);
        req->size.z = getInput<double>("size_z").value_or(0.04);
        req->pose_in_link.position.x = 0.0;
        req->pose_in_link.position.y = 0.0;
        req->pose_in_link.position.z = getInput<double>("offset_z").value_or(0.02);
        req->pose_in_link.orientation.w = 1.0;
        // touch_links 留空，server 用 default_touch_links

        auto future = client_->async_send_request(req);
        if (future.wait_for(std::chrono::duration<double>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "attach_object timeout");
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "attach failed: %s", resp->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "attached %s", req->object_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    DetachObjectNode::DetachObjectNode(const std::string &name,
                                       const BT::NodeConfig &config,
                                       rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<s622_bt_manager::srv::DetachObject>("detach_object");
    }

    BT::NodeStatus DetachObjectNode::tick()
    {
        double timeout = getInput<double>("timeout_sec").value_or(3.0);

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(node_->get_logger(), "detach_object service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::DetachObject::Request>();
        req->object_name = getInput<std::string>("object_name").value_or("cube");
        req->put_back_in_world = getInput<bool>("put_back_in_world").value_or(true);

        auto drop_ps = getInput<geometry_msgs::msg::PoseStamped>("drop_pose");
        if (drop_ps.has_value())
        {
            req->drop_pose = drop_ps.value().pose;
        }
        else
        {
            // 默认放在 base_link 原点附近（不太有意义，但避免 crash）
            req->drop_pose.orientation.w = 1.0;
        }

        auto future = client_->async_send_request(req);
        if (future.wait_for(std::chrono::duration<double>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "detach_object timeout");
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "detach failed: %s", resp->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "detached %s", req->object_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void registerSceneNodes(BT::BehaviorTreeFactory &factory,
                            rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<AttachObjectNode>(
            "AttachObject",
            [node](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<AttachObjectNode>(name, config, node); });
        factory.registerBuilder<DetachObjectNode>(
            "DetachObject",
            [node](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<DetachObjectNode>(name, config, node); });
    }

} // namespace s622_bt