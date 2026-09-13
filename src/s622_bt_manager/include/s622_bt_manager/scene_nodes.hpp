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
                // [M4 rod] 物体在 link 系中的完整位姿。长条物体抓取点可在中心之外、
                // 长轴也未必与 link 轴一致 → 需要 pos_x/y/z + 四元数。
                // 兼容旧用法：pos_z 未给出(NaN)时回退到 offset_z。
                BT::InputPort<double>("pos_x", 0.0, "object center x in link frame"),
                BT::InputPort<double>("pos_y", 0.0, "object center y in link frame"),
                BT::InputPort<double>("pos_z", std::numeric_limits<double>::quiet_NaN(),
                                      "object center z in link frame (default: offset_z)"),
                BT::InputPort<double>("quat_x", 0.0, "object orientation (w=1 identity)"),
                BT::InputPort<double>("quat_y", 0.0, ""),
                BT::InputPort<double>("quat_z", 0.0, ""),
                BT::InputPort<double>("quat_w", 1.0, ""),
                BT::InputPort<double>("offset_z", 0.02,
                                      "legacy: pose.z in link frame (cube center below TCP)"),
                // [M4 BT 回填] 运行期算出来的完整位姿（如 GenerateHandoverCandidate 的输出）。
                // 给了这个端口就优先用它，忽略上面的 pos_*/quat_*/offset_z。
                BT::InputPort<geometry_msgs::msg::Pose>("pose_in_link_full", ""),
                // 空格/逗号分隔的 link 名列表；空 = server 默认(按 link 臂前缀过滤)。
                // 交接阶段需要放行右指接触时显式传左右手指/grasp links。
                BT::InputPort<std::string>("touch_links", "",
                                           "space/comma separated link names"),
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
                // [M4 rod] 物体在新 link 系中的完整位姿(缺省 x=y=0, z=offset_z, identity)
                BT::InputPort<double>("pos_x", 0.0, "object center x in new link frame"),
                BT::InputPort<double>("pos_y", 0.0, "object center y in new link frame"),
                BT::InputPort<double>("pos_z", std::numeric_limits<double>::quiet_NaN(),
                                      "object center z in new link frame (default: offset_z)"),
                BT::InputPort<double>("quat_x", 0.0, "object orientation (w=1 identity)"),
                BT::InputPort<double>("quat_y", 0.0, ""),
                BT::InputPort<double>("quat_z", 0.0, ""),
                BT::InputPort<double>("quat_w", 1.0, ""),
                BT::InputPort<double>("offset_z", 0.02,
                                      "legacy: cube pose z in new link frame"),
                // [M4 BT 回填] 运行期完整位姿 {}^{G_R} T_O（GenerateHandoverCandidate 输出）
                BT::InputPort<geometry_msgs::msg::Pose>("pose_in_new_link_full", ""),
                BT::InputPort<std::string>("touch_links", "",
                                           "space/comma separated link names"),
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