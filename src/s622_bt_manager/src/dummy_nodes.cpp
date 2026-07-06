#include "s622_bt_manager/dummy_nodes.hpp"

#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace s622_bt {

BT::NodeStatus CheckSystemReady::tick() {
  RCLCPP_INFO(rclcpp::get_logger("CheckSystemReady"), "System ready (dummy)");
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

void registerDummyNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<CheckSystemReady>("CheckSystemReady");
  factory.registerNodeType<PrintMessage>("PrintMessage");
  factory.registerNodeType<Wait>("Wait");
}

}  // namespace s622_bt