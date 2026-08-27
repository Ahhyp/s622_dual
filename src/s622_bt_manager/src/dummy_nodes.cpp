#include "s622_bt_manager/dummy_nodes.hpp"

#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace s622_bt {

CheckSystemReady::CheckSystemReady(const std::string &name, const BT::NodeConfig &config,
                                   rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus CheckSystemReady::tick() {
  if (!client_) {
    client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
  }

  // controller_manager 可能还在启动，等它出现（最多 2s）
  if (!client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(rclcpp::get_logger("CheckSystemReady"),
                "controller_manager service '/controller_manager/list_controllers' "
                "not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  // Humble rclcpp::Client 无同步 call()：用 async + future 等待（响应由主 executor spin 处理，
  // BT tick 线程只等待，不阻塞 executor）
  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(rclcpp::get_logger("CheckSystemReady"),
                "list_controllers service call failed/timeout");
    return BT::NodeStatus::FAILURE;
  }
  auto response = future.get();

  const auto &controllers = response->controller;
  if (controllers.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CheckSystemReady"), "no controllers loaded");
    return BT::NodeStatus::FAILURE;
  }

  // 要求所有已加载 controller 全 active（双臂 5 个 / 单臂 3 个）
  for (const auto &c : controllers) {
    if (c.state != "active") {
      RCLCPP_WARN(rclcpp::get_logger("CheckSystemReady"),
                  "controller '%s' not active (state=%s)",
                  c.name.c_str(), c.state.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("CheckSystemReady"),
              "System ready: %zu controllers active", controllers.size());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PrintMessage::tick() {
  auto msg = getInput<std::string>("msg");
  RCLCPP_INFO(rclcpp::get_logger("PrintMessage"), "%s",
              msg.value_or(std::string{"(no msg)"}).c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Wait::tick() {
  double sec = 1.0;
  getInput("sec", sec);
  RCLCPP_INFO(rclcpp::get_logger("Wait"), "Waiting %.2f sec", sec);
  std::this_thread::sleep_for(std::chrono::duration<double>(sec));
  return BT::NodeStatus::SUCCESS;
}

void registerDummyNodes(BT::BehaviorTreeFactory &factory, rclcpp::Node::SharedPtr node) {
  factory.registerNodeType<CheckSystemReady>("CheckSystemReady", node);
  factory.registerNodeType<PrintMessage>("PrintMessage");
  factory.registerNodeType<Wait>("Wait");
}

}  // namespace s622_bt