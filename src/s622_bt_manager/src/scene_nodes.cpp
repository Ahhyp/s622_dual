#include "s622_bt_manager/scene_nodes.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace s622_bt
{

    static std::string resolve_link_name(const std::string &explicit_link,
                                         const std::string &arm_prefix,
                                         const std::string &suffix)
    {
        if (!explicit_link.empty())
            return explicit_link;
        if (arm_prefix.empty())
            return suffix;                // M1.7 fallback: "grasp_frame"
        return arm_prefix + "_" + suffix; // "left_grasp_frame"
    }

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
        std::string arm_prefix = getInput<std::string>("arm_prefix").value_or("");
        std::string explicit_link = getInput<std::string>("link_name").value_or("");
        const std::string link_name = resolve_link_name(explicit_link, arm_prefix,
                                                        "grasp_frame");

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(node_->get_logger(), "attach_object service unavailable (arm=%s)",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::AttachObject::Request>();
        req->object_name = getInput<std::string>("object_name").value_or("cube");
        req->link_name = link_name;
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
            RCLCPP_ERROR(node_->get_logger(), "attach_object timeout(arm=%s)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "attach failed(arm = %s): %s", arm_prefix.c_str(), resp->error_msg.c_str());
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
        std::string arm_prefix = getInput<std::string>("arm_prefix").value_or("");

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(node_->get_logger(), "detach_object service unavailable (arm=%s)",
                         arm_prefix.c_str());
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
            RCLCPP_ERROR(node_->get_logger(),
                         "detach_object timeout (arm=%s)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "detach failed (arm=%s): %s",
                         arm_prefix.c_str(), resp->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "detached %s (arm=%s)",
                    req->object_name.c_str(), arm_prefix.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    TransferObjectNode::TransferObjectNode(const std::string &name,
                                           const BT::NodeConfig &config,
                                           rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<s622_bt_manager::srv::TransferObject>(
            "transfer_object");
    }

    BT::NodeStatus TransferObjectNode::tick()
    {
        double timeout = getInput<double>("timeout_sec").value_or(3.0);
        std::string explicit_link = getInput<std::string>("new_link_name").value_or("");
        std::string new_arm = getInput<std::string>("new_arm_prefix").value_or("");

        // 派生 new_link_name
        std::string new_link;
        if (!explicit_link.empty())
            new_link = explicit_link;
        else if (!new_arm.empty())
            new_link = new_arm + "_grasp_frame";
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "TransferObject: need new_arm_prefix or new_link_name");
            return BT::NodeStatus::FAILURE;
        }

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "transfer_object service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::TransferObject::Request>();
        req->object_name = getInput<std::string>("object_name").value_or("cube");
        req->new_link_name = new_link;
        req->pose_in_new_link.position.x = 0.0;
        req->pose_in_new_link.position.y = 0.0;
        req->pose_in_new_link.position.z = getInput<double>("offset_z").value_or(0.02);
        req->pose_in_new_link.orientation.w = 1.0;
        // touch_links 留空 -> server 用 default_touch_links (双臂 launch 里已配)

        auto future = client_->async_send_request(req);
        if (future.wait_for(std::chrono::duration<double>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "transfer_object timeout");
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "transfer failed: %s", resp->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "transferred %s -> %s",
                    req->object_name.c_str(), new_link.c_str());
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
        factory.registerBuilder<TransferObjectNode>(
            "TransferObject",
            [node](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<TransferObjectNode>(name, config, node); });
        }

} // namespace s622_bt