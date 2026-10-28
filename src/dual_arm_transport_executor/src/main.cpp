#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dual_arm_transport_executor/executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dual_arm_transport_executor::TransportExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
