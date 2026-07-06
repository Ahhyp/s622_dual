#include "fairino_planning_ros/fairino_planner_manager.h"
#include "fairino_planning_ros/fairino_planning_context.h"

#include "pluginlib/class_list_macros.hpp"

namespace fairino_planning_ros
{

    bool FairinoPlannerManager::initialize(
        const moveit::core::RobotModelConstPtr &model,
        const rclcpp::Node::SharedPtr &node,
        const std::string &)
    {
        robot_model_ = model;
        node_ = node;
        return true;
    }

    bool FairinoPlannerManager::canServiceRequest(
        const moveit_msgs::msg::MotionPlanRequest &req) const
    {
        return !req.group_name.empty();
    }

    std::string FairinoPlannerManager::getDescription() const
    {
        return "Fairino custom planner manager";
    }

    void FairinoPlannerManager::getPlanningAlgorithms(
        std::vector<std::string> &algs) const
    {
        algs = {
            "RRTStar",
            "BiRRTStar"};
    }

    planning_interface::PlanningContextPtr
    FairinoPlannerManager::getPlanningContext(
        const planning_scene::PlanningSceneConstPtr &planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::msg::MoveItErrorCodes &error_code) const
    {
        (void)planning_scene;

        auto context = std::make_shared<FairinoPlanningContext>(
            "fairino_planning_context",
            req.group_name,
            robot_model_);

        context->setMotionPlanRequest(req);
        context->setPlanningScene(planning_scene);

        error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        return context;
    }

    void FairinoPlannerManager::setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap &pcs)
    {
        // 这个是基类给的成员变量，保护
        config_settings_ = pcs;
    }


} // namespace fairino_planning_ros

PLUGINLIB_EXPORT_CLASS(
    fairino_planning_ros::FairinoPlannerManager,
    planning_interface::PlannerManager)