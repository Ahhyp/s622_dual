#include "s622_bt_manager/gripper_nodes.hpp"

#include <chrono>

namespace s622_bt
{

    using namespace std::chrono_literals;

    static std::string service_name_for(const std::string &prefix,
                                        const std::string &base)
    {
        if (prefix.empty())
            return base;
        return "/" + prefix + "/" + base;
    }

    SetGripperNode::SetGripperNode(const std::string &name,
                                   const BT::NodeConfig &config,
                                   rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(std::move(node))
    {
        // client_ = node_->create_client<s622_bt_manager::srv::SetGripper>("set_gripper");
    }

    BT::NodeStatus SetGripperNode::tick()
    {
        std::string cmd, arm_prefix;
        if (!getInput("command", cmd))
        {
            RCLCPP_ERROR(node_->get_logger(), "SetGripper: missing command");
            return BT::NodeStatus::FAILURE;
        }
        getInput("arm_prefix", arm_prefix);
        float timeout = 5.0f;
        getInput("timeout_sec", timeout);

        // ---- 懒建/重建 client ----
        if (arm_prefix != client_arm_prefix_)
        {
            const auto srv_name = service_name_for(arm_prefix, "set_gripper");
            client_ = node_->create_client<s622_bt_manager::srv::SetGripper>(srv_name);
            client_arm_prefix_ = arm_prefix;
            RCLCPP_INFO(node_->get_logger(),
                        "SetGripper: bound to service '%s'", srv_name.c_str());
        }

        if (!client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "set_gripper service unavailable (arm=%s)",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<s622_bt_manager::srv::SetGripper::Request>();
        req->command = cmd;
        auto future = client_->async_send_request(req);

        auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout);
        while (rclcpp::ok() && node_->now() < deadline)
        {
            if (future.wait_for(0ms) == std::future_status::ready)
                break;
            std::this_thread::sleep_for(20ms);
        }
        if (future.wait_for(0ms) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "SetGripper: timeout (arm=%s)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto res = future.get();
        setOutput("finger_position", static_cast<float>(res->finger_position));
        if (!res->success)
        {
            RCLCPP_ERROR(node_->get_logger(), "SetGripper failed: %s",
                         res->error_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "SetGripper[arm=%s cmd=%s] -> finger=%.4f",
                    arm_prefix.c_str(), cmd.c_str(), res->finger_position);

        return BT::NodeStatus::SUCCESS;
    }

    VerifyGraspNode::VerifyGraspNode(const std::string &name,
                                     const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(std::move(node))
    {
        sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(mu_);
                latest_ = msg;
            });
    }

    BT::NodeStatus VerifyGraspNode::tick()
    {
        float min_pos = 0.005f, timeout = 5.0f;
        std::string fb_joint_raw, arm_prefix;
        getInput("finger_min_position", min_pos);
        getInput("feedback_joint", fb_joint_raw);
        getInput("arm_prefix", arm_prefix);
        getInput("timeout_sec", timeout);

        std::string fb_joint = fb_joint_raw;
        if (fb_joint.empty())
        {
            fb_joint = arm_prefix.empty()
                           ? "finger1_joint"
                           : arm_prefix + "_finger1_joint";
        }

        auto t_start = node_->now();
        auto deadline = t_start + rclcpp::Duration::from_seconds(timeout);
        const double min_wait = 1.5; // 最少等 1.5s 让 finger 完成闭合
        float last_pos = -1.0f;
        int stable_count = 0;

        while (rclcpp::ok() && node_->now() < deadline)
        {
            sensor_msgs::msg::JointState::SharedPtr latest;
            {
                std::lock_guard<std::mutex> lk(mu_);
                latest = latest_;
            }
            if (latest)
            {
                auto it = std::find(latest->name.begin(), latest->name.end(), fb_joint);
                if (it != latest->name.end())
                {
                    size_t idx = std::distance(latest->name.begin(), it);
                    float pos = static_cast<float>(latest->position[idx]);
                    setOutput("finger_position", pos);

                    double elapsed = (node_->now() - t_start).seconds();
                    if (elapsed < min_wait)
                    {
                        last_pos = pos;
                        std::this_thread::sleep_for(50ms);
                        continue; // 还在最小等待期内, 不判定
                    }

                    // 等待 finger 停止运动（连续 5 帧变化 < 0.0005）
                    if (last_pos >= 0.0f && std::abs(pos - last_pos) < 0.0005f)
                    {
                        stable_count++;
                        if (stable_count >= 5)
                        {
                            // 三段判定: 完全闭合=空, 几乎没动=未接触, 中间=抓到
                            const float max_open = 0.022f;
                            if (pos < min_pos)
                            {
                                RCLCPP_WARN(node_->get_logger(),
                                            "VerifyGrasp[arm=%s]: EMPTY (%s=%.4f < %.4f)",
                                            arm_prefix.c_str(), fb_joint.c_str(),
                                            pos, min_pos);
                                return BT::NodeStatus::FAILURE;
                            }
                            else if (pos > max_open)
                            {
                                RCLCPP_WARN(node_->get_logger(),
                                            "VerifyGrasp[arm=%s]: MISS (%s=%.4f > %.4f)",
                                            arm_prefix.c_str(), fb_joint.c_str(),
                                            pos, max_open);
                                return BT::NodeStatus::FAILURE;
                            }
                            else
                            {
                                RCLCPP_INFO(node_->get_logger(),
                                            "VerifyGrasp[arm=%s]: GRASPED (%.4f in [%.4f,%.4f])",
                                            arm_prefix.c_str(), pos, min_pos, max_open);
                                return BT::NodeStatus::SUCCESS;
                            }
                        }
                    }
                    else
                    {
                        stable_count = 0;
                    }
                    last_pos = pos;
                }
            }
            std::this_thread::sleep_for(50ms);
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "VerifyGrasp[arm=%s]: timeout reading %s",
                     arm_prefix.c_str(), fb_joint.c_str());
        return BT::NodeStatus::FAILURE;
    }

    void registerGripperNodes(BT::BehaviorTreeFactory &factory,
                              rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<SetGripperNode>(
            "SetGripper",
            [node](const std::string &n, const BT::NodeConfig &c)
            {
                return std::make_unique<SetGripperNode>(n, c, node);
            });
        factory.registerBuilder<VerifyGraspNode>(
            "VerifyGrasp",
            [node](const std::string &n, const BT::NodeConfig &c)
            {
                return std::make_unique<VerifyGraspNode>(n, c, node);
            });
    }

} // namespace s622_bt