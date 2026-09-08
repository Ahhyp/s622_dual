#include "s622_bt_manager/scene_nodes.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace s622_bt
{

    // [M4 rod] 解析 'touch_links' 字符串端口：空格/逗号分隔 → vector<string>
    static std::vector<std::string> split_touch_links(const std::string &raw)
    {
        std::vector<std::string> out;
        std::stringstream ss(raw);
        std::string tok;
        while (ss >> tok)
        {
            std::stringstream tok_ss(tok);
            std::string item;
            while (std::getline(tok_ss, item, ','))
            {
                if (!item.empty())
                    out.push_back(item);
            }
        }
        return out;
    }

    // [M4 rod] 组装物体在 link 系中的 pose：pos_x/y/z 优先；pos_z 缺失(NaN)时回退 offset_z；
    // 四元数默认 identity，非单位时归一化。
    static geometry_msgs::msg::Pose object_pose_from_ports(
        BT::TreeNode &node, const std::string &offset_port)
    {
        geometry_msgs::msg::Pose p;
        double ox = node.getInput<double>(offset_port).value_or(0.02);
        double x = node.getInput<double>("pos_x").value_or(0.0);
        double y = node.getInput<double>("pos_y").value_or(0.0);
        double z = node.getInput<double>("pos_z").value_or(ox);
        if (std::isnan(z))
            z = ox; // 未给 pos_z → 旧 offset_z 语义
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.x = node.getInput<double>("quat_x").value_or(0.0);
        p.orientation.y = node.getInput<double>("quat_y").value_or(0.0);
        p.orientation.z = node.getInput<double>("quat_z").value_or(0.0);
        p.orientation.w = node.getInput<double>("quat_w").value_or(1.0);
        double n = std::sqrt(p.orientation.x * p.orientation.x +
                             p.orientation.y * p.orientation.y +
                             p.orientation.z * p.orientation.z +
                             p.orientation.w * p.orientation.w);
        if (n < 1e-9)
            p.orientation.w = 1.0; // 全零 → identity
        else if (std::abs(n - 1.0) > 1e-6)
        {
            p.orientation.x /= n;
            p.orientation.y /= n;
            p.orientation.z /= n;
            p.orientation.w /= n;
        }
        return p;
    }

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
        req->pose_in_link = object_pose_from_ports(*this, "offset_z");
        auto touch_raw = getInput<std::string>("touch_links").value_or("");
        if (!touch_raw.empty())
            req->touch_links = split_touch_links(touch_raw);
        // touch_links 留空 → server 用 default_touch_links

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
                                       RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(ros)
    {
        client_ = ros_->node->create_client<s622_bt_manager::srv::DetachObject>(
            "detach_object");
    }

    BT::NodeStatus DetachObjectNode::tick()
    {
        double timeout = getInput<double>("timeout_sec").value_or(3.0);
        std::string arm_prefix = getInput<std::string>("arm_prefix").value_or("");

        if (!client_->wait_for_service(std::chrono::milliseconds(
                static_cast<int>(timeout * 1000))))
        {
            RCLCPP_ERROR(ros_->node->get_logger(),
                         "detach_object service unavailable (arm=%s)",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::DetachObject::Request>();
        req->object_name = getInput<std::string>("object_name").value_or("cube");
        req->put_back_in_world = getInput<bool>("put_back_in_world").value_or(true);

        auto drop_ps = getInput<geometry_msgs::msg::PoseStamped>("drop_pose");
        if (drop_ps.has_value())
        {
            // drop_pose 是 'left_base_link'/'right_base_link' 系（GeneratePlaceCandidate 输出），
            // 而 planning_scene_service 以 world 为 frame 发布 → 必须变换到 world。
            try
            {
                geometry_msgs::msg::PoseStamped world_ps =
                    ros_->tf_buffer->transform(drop_ps.value(), "world",
                                               tf2::durationFromSec(0.5));
                req->drop_pose = world_ps.pose;
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(ros_->node->get_logger(),
                            "detach drop_pose transform failed (%s), using raw pose",
                            e.what());
                req->drop_pose = drop_ps.value().pose;
            }
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
            RCLCPP_ERROR(ros_->node->get_logger(),
                         "detach_object timeout (arm=%s)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(ros_->node->get_logger(),
                         "detach failed (arm=%s): %s",
                         arm_prefix.c_str(), resp->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(ros_->node->get_logger(),
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
        req->pose_in_new_link = object_pose_from_ports(*this, "offset_z");
        auto touch_raw = getInput<std::string>("touch_links").value_or("");
        if (!touch_raw.empty())
            req->touch_links = split_touch_links(touch_raw);
        // touch_links 留空 → server 用 default_touch_links

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
                            RosContextPtr ros)
    {
        factory.registerBuilder<AttachObjectNode>(
            "AttachObject",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<AttachObjectNode>(name, config, ros->node); });
        factory.registerBuilder<DetachObjectNode>(
            "DetachObject",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<DetachObjectNode>(name, config, ros); });
        factory.registerBuilder<TransferObjectNode>(
            "TransferObject",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<TransferObjectNode>(name, config, ros->node); });
        }

} // namespace s622_bt