#pragma once

#include <memory>

#include "fairino_planning_core/collision/collision_interface.h"

#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_state/robot_state.h"

namespace fairino_planning_ros
{
    /*
    把 MoveIt2 的 PlanningScene 包装成 core 期望的 CollisionInterface，
    让 RRT 等算法能无差别使用 MoveIt 的碰撞场景。
    */
    class MoveItCollisionChecker : public fairino_planning::CollisionInterface
    {
    public:
        MoveItCollisionChecker(
            planning_scene::PlanningSceneConstPtr planning_scene,
            const std::string &group_name);

        bool isStateValid(const fairino_planning::JointArray &q) const override;

        bool isMotionValid(
            const fairino_planning::JointArray &from,
            const fairino_planning::JointArray &to,
            double resolution) const override;

    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
        std::string group_name_;
        const moveit::core::JointModelGroup *jmg_;
    };

} // namespace fairino_planning_ros