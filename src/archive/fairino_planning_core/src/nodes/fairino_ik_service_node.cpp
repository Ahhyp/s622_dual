#include <rclcpp/rclcpp.hpp>
#include "fairino_planning_core/service/fairino_ik_service.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fairino_ik_service");
  fairino_planning_core::FairinoIKService service(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
