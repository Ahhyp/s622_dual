// fairino_ik_plugin.h
#pragma once

#include <string>
#include <vector>

#include "moveit/kinematics_base/kinematics_base.h"
#include "fairino_planning_core/ik/fairino_ik.h"

namespace fairino_planning_ros
{

    class FairinoIKPlugin : public kinematics::KinematicsBase
    {
    public:
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const moveit::core::RobotModel &robot_model,
            const std::string &group_name,
            const std::string &base_frame,
            const std::vector<std::string> &tip_frames,
            double search_discretization) override;

        bool getPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            std::vector<double> &solution,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options =
                kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            std::vector<double> &solution,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options =
                kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            const std::vector<double> &consistency_limits,
            std::vector<double> &solution,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options =
                kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            std::vector<double> &solution,
            const IKCallbackFn &solution_callback,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options =
                kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            const std::vector<double> &consistency_limits,
            std::vector<double> &solution,
            const IKCallbackFn &solution_callback,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options =
                kinematics::KinematicsQueryOptions()) const override;

        Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose &ik_pose);

        fairino_planning::JointArray pickClosest(
            const std::vector<fairino_planning::JointArray> &sols,
            const fairino_planning::JointArray &seed) const;

        bool withinLimits(
            const fairino_planning::JointArray &q) const;

        fairino_planning::JointArray reboundToSeed(
            const fairino_planning::JointArray &q,
            const fairino_planning::JointArray &seed) const;

        bool getPositionFK(
            const std::vector<std::string> &link_names,
            const std::vector<double> &joint_angles,
            std::vector<geometry_msgs::msg::Pose> &poses) const override;

        const std::vector<std::string> &getJointNames() const override;
        const std::vector<std::string> &getLinkNames() const override;

    private:
        std::string group_name_;
        std::string base_frame_;
        std::vector<std::string> tip_frames_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> link_names_;
        std::unique_ptr<fairino_planning::FairinoIK> ik_;
        std::unique_ptr<fairino_planning::DHKinematics> fk_;
        // 限位
        std::array<double, fairino_planning::DOF> joint_lower_{};
        std::array<double, fairino_planning::DOF> joint_upper_{};
        // 坐标系修正
        Eigen::Isometry3d base_offset_;
        Eigen::Isometry3d tip_offset_;
        // ROS
        rclcpp::Node::SharedPtr node_;

        // 标志
        bool initialized_ = false;
    };

} // namespace fairino_planning_ros