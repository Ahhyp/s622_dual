#pragma once

#include <memory>
#include <string>
#include <vector>

#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "rclcpp/rclcpp.hpp"

namespace fairino_planning_ros
{

    class FairinoPlannerManager : public planning_interface::PlannerManager
    {
    public:
        // 初始化
        bool initialize(
            const moveit::core::RobotModelConstPtr &model,
            const rclcpp::Node::SharedPtr &node,
            const std::string &parameter_namespace) override;

        bool canServiceRequest(
            const moveit_msgs::msg::MotionPlanRequest &req) const override;

        std::string getDescription() const override;

        void getPlanningAlgorithms(
            std::vector<std::string> &algs) const override;

        planning_interface::PlanningContextPtr getPlanningContext(
            const planning_scene::PlanningSceneConstPtr &planning_scene,
            const planning_interface::MotionPlanRequest &req,
            moveit_msgs::msg::MoveItErrorCodes &error_code) const override;

        void setPlannerConfigurations(
            const planning_interface::PlannerConfigurationMap &pcs) override;

    private:
        moveit::core::RobotModelConstPtr robot_model_;
        rclcpp::Node::SharedPtr node_;
    };

} // namespace fairino_planning_ros