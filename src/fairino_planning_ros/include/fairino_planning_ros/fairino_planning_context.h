#include <memory>
#include <string>

#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/planning_scene/planning_scene.h"

#include "fairino_planning_core/algorithms/planning_algorithm.h"
#include "fairino_planning_ros/moveit_collision_checker.h"

namespace fairino_planning_ros
{

    class FairinoPlanningContext : public planning_interface::PlanningContext
    {
    public:
        FairinoPlanningContext(
            const std::string &name,
            const std::string &group,
            const moveit::core::RobotModelConstPtr &robot_model);

        bool solve(planning_interface::MotionPlanResponse &res) override;
        bool solve(planning_interface::MotionPlanDetailedResponse &res) override;
        void clear() override;
        bool terminate() override;

    private:
        bool extractStartState(fairino_planning::JointArray &q) const;
        bool extractGoalState(fairino_planning::JointArray &q) const;
        fairino_planning::JointLimits extractJointLimits() const;

        void convertPathToRobotTrajectory(
            const fairino_planning::JointPath &path,
            planning_interface::MotionPlanResponse &res) const;

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup *jmg_ = nullptr;
        std::atomic<bool> terminated_{false};
    };
}