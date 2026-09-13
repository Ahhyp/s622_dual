#pragma once

// ============================================================================
// M4：「长条物体双臂松耦合交接」BT 回填节点（规划 §5/§6/§7/§9）
//
//   GenerateHandoverCandidate : 运行期生成 Right 接物候选位姿
//                               {}^W T_{G_R}^* = {}^W T_O · {}^O T_{G_R}^*
//                               物体位姿优先取 PlanningScene 里 attached object（= AttachObject
//                               真实写入的那个位姿，随轮次变化），YAML 端口为兜底。
//   PlanToJoints              : 单臂关节空间规划+执行（OMPL，含碰撞检查）——
//                               对应脚本 R_move()（R→PRE / L 退让到 home）
//   CartesianApproach         : 世界空间直线进给（沿线逐点 FK 反解 → 一条 JTC 轨迹）——
//                               对应脚本 R_move_cart_line()（§17：关节空间插值 TCP 轨迹是弯的，
//                               横向余量被吃掉 → 5 轮里 2 轮撞飞；直线进给让余量恒定）
//
// 这三个节点把已验证脚本里的动作原语搬进 BT，使 M4 主流程可以在行为树里跑，
// 而不是只存在于 tmp/m4_round/*.py。
// ============================================================================

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "s622_bt_manager/perception_nodes.hpp"

namespace s622_bt
{

    using Vec3 = std::array<double, 3>;

    // ------------------------------------------------------------------
    // 共享 helper：单臂 FK 服务 + /joint_states 缓存 + 阻尼最小二乘位置解算
    // （与 tmp/m4_round/ho_verify.py 的 fk_grasp/solve_receive 等价）
    // ------------------------------------------------------------------
    class ArmFkHelper
    {
    public:
        ArmFkHelper(const rclcpp::Node::SharedPtr &node,
                    const std::string &fk_service);

        void setArmPrefix(const std::string &arm) { arm_ = arm; }
        const std::string &armPrefix() const { return arm_; }

        // 等 /joint_states 里该臂 6 个关节到齐
        bool waitJoints(double timeout_sec);
        std::vector<double> currentJoints() const { return q_; }
        bool hasJoints() const { return q_.size() == 6; }

        // FK: 该臂 6 关节 → <arm>_grasp_frame 在 world 下的位姿
        bool fk(const std::vector<double> &q,
                geometry_msgs::msg::Pose *out,
                double timeout_sec);

        // 世界位移 dv → 只调 j1..j3（j4..j6 固定），阻尼最小二乘 + 4 次迭代
        bool solveDelta(const std::vector<double> &q_base,
                        const Vec3 &dv,
                        double tol,
                        std::vector<double> *q_out,
                        Vec3 *achieved);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_cli_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        std::string arm_;
        std::vector<double> q_;
    };

    // 动作节点公共基类：helper + 控制器 action client
    class HandoverMotionBase
    {
    public:
        explicit HandoverMotionBase(const rclcpp::Node::SharedPtr &node);
        virtual ~HandoverMotionBase() = default;

    protected:
        // 绑定臂：创建 FK helper + 控制器 action client（同一条臂只建一次）
        void bindArm(const std::string &arm_prefix, const std::string &fk_service);

        bool sendTrajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                            double timeout_sec,
                            std::string *err);

        rclcpp::Node::SharedPtr node_;
        std::string arm_;
        std::shared_ptr<ArmFkHelper> fk_;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr ctrl_;
    };

    // ------------------------------------------------------------------
    // GenerateHandoverCandidate (SyncActionNode)
    // ------------------------------------------------------------------
    class GenerateHandoverCandidateNode : public BT::SyncActionNode
    {
    public:
        GenerateHandoverCandidateNode(const std::string &name,
                                      const BT::NodeConfig &config,
                                      RosContextPtr ros);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        bool objectPoseFromScene(const std::string &object_name,
                                 const std::string &left_link,
                                 geometry_msgs::msg::Pose *out,
                                 std::string *err);
        RosContextPtr ros_;
        rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_cli_;
    };

    // ------------------------------------------------------------------
    // PlanToJoints (StatefulActionNode)：OMPL 关节空间规划 + 执行
    // ------------------------------------------------------------------
    class PlanToJointsNode : public BT::StatefulActionNode, public HandoverMotionBase
    {
    public:
        PlanToJointsNode(const std::string &name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_cli_;
        std::vector<double> q_target_;
        double timeout_sec_ = 90.0;
    };

    // ------------------------------------------------------------------
    // CartesianApproach (StatefulActionNode)：世界空间直线进给
    // ------------------------------------------------------------------
    class CartesianApproachNode : public BT::StatefulActionNode, public HandoverMotionBase
    {
    public:
        CartesianApproachNode(const std::string &name,
                              const BT::NodeConfig &config,
                              rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        double timeout_sec_ = 90.0;
    };

    // ------------------------------------------------------------------
    // MeasureObjectInGrasp：物体中心相对当前 TCP 的世界偏移（诊断 / §14 指标）
    // ------------------------------------------------------------------
    class MeasureObjectInGraspNode : public BT::SyncActionNode
    {
    public:
        MeasureObjectInGraspNode(const std::string &name,
                                 const BT::NodeConfig &config,
                                 RosContextPtr ros);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        RosContextPtr ros_;
    };

    void registerHandoverNodes(BT::BehaviorTreeFactory &factory,
                               RosContextPtr ros,
                               rclcpp::Node::SharedPtr node);

} // namespace s622_bt
