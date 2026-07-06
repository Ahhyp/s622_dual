#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <fairino_msgs/srv/get_all_ik.hpp>

#include "fairino_planning_core/ik/fairino_ik.h"

namespace fairino_planning_core {

class FairinoIKService {
public:
  explicit FairinoIKService(rclcpp::Node::SharedPtr node);

private:
  void handle(
    const std::shared_ptr<fairino_msgs::srv::GetAllIK::Request> req,
    std::shared_ptr<fairino_msgs::srv::GetAllIK::Response> res);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<fairino_msgs::srv::GetAllIK>::SharedPtr srv_;

  std::unique_ptr<fairino_planning::FairinoIK> ik_;
  std::vector<std::string> joint_names_;
};

}  // namespace fairino_planning_core
