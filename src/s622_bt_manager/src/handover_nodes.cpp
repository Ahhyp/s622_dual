// ============================================================================
// M4 BT 回填：GenerateHandoverCandidate / PlanToJoints / CartesianApproach
// 见 include/s622_bt_manager/handover_nodes.hpp 顶部说明
// ============================================================================
#include "s622_bt_manager/handover_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace s622_bt
{

    namespace
    {
        std::string arm_joint_name(const std::string &arm, int i)
        {
            return arm + "_j" + std::to_string(i);
        }

        std::vector<std::string> arm_joint_names(const std::string &arm)
        {
            std::vector<std::string> v;
            for (int i = 1; i <= 6; ++i)
                v.push_back(arm_joint_name(arm, i));
            return v;
        }

        // 轮询 future。BT 线程里不能用 spin_until_future_complete：
        // 这个 node 已经挂在 MultiThreadedExecutor 上，回包由 executor 线程处理。
        template <typename FutureT>
        bool wait_ready(FutureT &fut, double timeout_sec)
        {
            const auto t0 = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
                    return true;
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >
                    timeout_sec)
                    return false;
                std::this_thread::sleep_for(2ms);
            }
            return false;
        }

        // 3x3 线性方程求解（列主元高斯消元），用于阻尼最小二乘
        bool solve3(double a[3][3], double b[3], double x[3])
        {
            double m[3][4];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                    m[i][j] = a[i][j];
                m[i][3] = b[i];
            }
            for (int c = 0; c < 3; ++c)
            {
                int piv = c;
                for (int r = c + 1; r < 3; ++r)
                    if (std::abs(m[r][c]) > std::abs(m[piv][c]))
                        piv = r;
                if (std::abs(m[piv][c]) < 1e-12)
                    return false;
                if (piv != c)
                    for (int j = 0; j < 4; ++j)
                        std::swap(m[c][j], m[piv][j]);
                for (int r = 0; r < 3; ++r)
                {
                    if (r == c)
                        continue;
                    const double f = m[r][c] / m[c][c];
                    for (int j = c; j < 4; ++j)
                        m[r][j] -= f * m[c][j];
                }
            }
            for (int i = 0; i < 3; ++i)
                x[i] = m[i][3] / m[i][i];
            return true;
        }

        geometry_msgs::msg::Pose pose_from_tf(const tf2::Transform &t)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = t.getOrigin().x();
            p.position.y = t.getOrigin().y();
            p.position.z = t.getOrigin().z();
            p.orientation.x = t.getRotation().x();
            p.orientation.y = t.getRotation().y();
            p.orientation.z = t.getRotation().z();
            p.orientation.w = t.getRotation().w();
            return p;
        }

        bool pose_is_identity(const geometry_msgs::msg::Pose &p)
        {
            return std::abs(p.position.x) < 1e-9 && std::abs(p.position.y) < 1e-9 &&
                   std::abs(p.position.z) < 1e-9 && std::abs(p.orientation.x) < 1e-9 &&
                   std::abs(p.orientation.y) < 1e-9 && std::abs(p.orientation.z) < 1e-9;
        }
    } // namespace

    // ==================================================================
    // ArmFkHelper
    // ==================================================================
    ArmFkHelper::ArmFkHelper(const rclcpp::Node::SharedPtr &node,
                             const std::string &fk_service)
        : node_(node)
    {
        fk_cli_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(fk_service);
        js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                if (arm_.empty())
                    return;
                std::vector<double> q(6, 0.0);
                for (int i = 0; i < 6; ++i)
                {
                    const auto nm = arm_joint_name(arm_, i + 1);
                    auto it = std::find(msg->name.begin(), msg->name.end(), nm);
                    if (it == msg->name.end())
                        return; // 还没到齐
                    q[i] = msg->position[std::distance(msg->name.begin(), it)];
                }
                q_ = q;
            });
    }

    bool ArmFkHelper::waitJoints(double timeout_sec)
    {
        const auto t0 = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            if (hasJoints())
                return true;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >
                timeout_sec)
                return false;
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    bool ArmFkHelper::fk(const std::vector<double> &q,
                         geometry_msgs::msg::Pose *out,
                         double timeout_sec)
    {
        if (q.size() != 6)
            return false;
        if (!fk_cli_->service_is_ready() && !fk_cli_->wait_for_service(2s))
            return false;

        auto req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
        req->header.frame_id = "world";
        req->header.stamp = node_->now();
        req->fk_link_names = {arm_ + "_grasp_frame"};
        req->robot_state.joint_state.name = arm_joint_names(arm_);
        req->robot_state.joint_state.position.assign(q.begin(), q.end());

        auto fut = fk_cli_->async_send_request(req);
        if (!wait_ready(fut, timeout_sec))
            return false;
        auto res = fut.get();
        if (!res || res->error_code.val != 1 || res->pose_stamped.empty())
            return false;
        *out = res->pose_stamped[0].pose;
        return true;
    }

    bool ArmFkHelper::solveDelta(const std::vector<double> &q_base,
                                 const Vec3 &dv,
                                 double tol,
                                 std::vector<double> *q_out,
                                 Vec3 *achieved)
    {
        geometry_msgs::msg::Pose p_base;
        if (!fk(q_base, &p_base, 5.0))
            return false;
        const Vec3 b{p_base.position.x, p_base.position.y, p_base.position.z};

        // Jacobian 3x6（中心差分 d=0.05 rad）。与 ho_verify.py 一致：只在基点算一次。
        const double d = 0.05;
        double J[3][6];
        for (int i = 0; i < 6; ++i)
        {
            std::vector<double> qp = q_base, qm = q_base;
            qp[i] += d;
            qm[i] -= d;
            geometry_msgs::msg::Pose pp, pm;
            if (!fk(qp, &pp, 5.0) || !fk(qm, &pm, 5.0))
                return false;
            J[0][i] = (pp.position.x - pm.position.x) / (2 * d);
            J[1][i] = (pp.position.y - pm.position.y) / (2 * d);
            J[2][i] = (pp.position.z - pm.position.z) / (2 * d);
        }

        std::vector<double> q = q_base;
        const double lam = 1e-4;
        for (int iter = 0; iter < 5; ++iter)
        {
            Vec3 err = dv;
            if (iter > 0)
            {
                geometry_msgs::msg::Pose pc;
                if (!fk(q, &pc, 5.0))
                    return false;
                err = {dv[0] - (pc.position.x - b[0]),
                       dv[1] - (pc.position.y - b[1]),
                       dv[2] - (pc.position.z - b[2])};
                const double n = std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
                if (n < tol)
                    break;
            }
            // A = J3^T J3 + lam I ；rhs = J3^T err（只调 j1..j3，J3 = J[:3, :3]）
            double A[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            double rhs[3] = {0, 0, 0};
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    for (int k = 0; k < 3; ++k)
                        A[r][c] += J[k][r] * J[k][c];
                    if (r == c)
                        A[r][c] += lam;
                }
                for (int k = 0; k < 3; ++k)
                    rhs[r] += J[k][r] * err[k]; // (J3^T · err)
            }
            double x[3];
            if (!solve3(A, rhs, x))
                return false;
            for (int i = 0; i < 3; ++i)
                q[i] += x[i];
        }

        geometry_msgs::msg::Pose pf;
        if (!fk(q, &pf, 5.0))
            return false;
        const Vec3 ach{pf.position.x - b[0], pf.position.y - b[1], pf.position.z - b[2]};
        // 收敛自检：偏差 > 5mm 视为反解失败（否则会把垃圾关节角送给规划/控制器）
        const double res = std::sqrt(std::pow(ach[0] - dv[0], 2) +
                                     std::pow(ach[1] - dv[1], 2) +
                                     std::pow(ach[2] - dv[2], 2));
        if (res > 5e-3)
            return false;
        *q_out = q;
        if (achieved)
            (*achieved) = ach;
        return true;
    }

    // ==================================================================
    // HandoverMotionBase
    // ==================================================================
    HandoverMotionBase::HandoverMotionBase(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
    }

    void HandoverMotionBase::bindArm(const std::string &arm_prefix,
                                     const std::string &fk_service)
    {
        if (arm_ == arm_prefix && fk_ && ctrl_)
            return;
        arm_ = arm_prefix;
        fk_ = std::make_shared<ArmFkHelper>(node_, fk_service);
        fk_->setArmPrefix(arm_prefix);
        ctrl_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, "/" + arm_prefix + "_arm_controller/follow_joint_trajectory");
        RCLCPP_INFO(node_->get_logger(),
                    "[handover] arm=%s bound (fk=%s, ctrl=/%s_arm_controller)",
                    arm_prefix.c_str(), fk_service.c_str(), arm_prefix.c_str());
    }

    bool HandoverMotionBase::sendTrajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                                            double timeout_sec,
                                            std::string *err)
    {
        if (!ctrl_->wait_for_action_server(5s))
        {
            if (err)
                *err = "controller action server unavailable";
            return false;
        }
        control_msgs::action::FollowJointTrajectory::Goal goal;
        goal.trajectory = traj;
        goal.trajectory.header.stamp.sec = 0;
        goal.trajectory.header.stamp.nanosec = 0;

        auto gh_fut = ctrl_->async_send_goal(goal);
        if (!wait_ready(gh_fut, 10.0))
        {
            if (err)
                *err = "send_goal timeout";
            return false;
        }
        auto gh = gh_fut.get();
        if (!gh)
        {
            if (err)
                *err = "goal rejected";
            return false;
        }
        auto res_fut = ctrl_->async_get_result(gh);
        // sim 时间预算 → 墙钟预算（RTF≈0.25，×3 + 60s 余量）
        const double wall = timeout_sec * 3.0 + 60.0;
        if (!wait_ready(res_fut, wall))
        {
            ctrl_->async_cancel_goal(gh);
            if (err)
                *err = "execute timeout";
            return false;
        }
        auto wr = res_fut.get();
        if (wr.code != rclcpp_action::ResultCode::SUCCEEDED ||
            (wr.result && wr.result->error_code != 0 &&
             wr.result->error_code != wr.result->SUCCESSFUL))
        {
            if (err)
                *err = "controller error_code=" +
                       std::to_string(wr.result ? wr.result->error_code : -1);
            return false;
        }
        return true;
    }

    // ==================================================================
    // GenerateHandoverCandidate
    // ==================================================================
    BT::PortsList GenerateHandoverCandidateNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_name", "handover_bar", "attached object id"),
            BT::InputPort<std::string>("left_link", "left_grasp_frame", "物体挂在这条 link 上"),
            BT::InputPort<std::string>("world_frame", "world", ""),
            BT::InputPort<bool>("use_scene_pose", true,
                                "true=从 PlanningScene 的 attached object 取运行期位姿"),
            // —— YAML 兜底（use_scene_pose=false 或场景里查不到时用）——
            BT::InputPort<double>("object_pos_x", 0.0, "object center in left_link"),
            BT::InputPort<double>("object_pos_y", 0.0, ""),
            BT::InputPort<double>("object_pos_z", 0.0, ""),
            BT::InputPort<double>("object_quat_x", 0.0, ""),
            BT::InputPort<double>("object_quat_y", 0.0, ""),
            BT::InputPort<double>("object_quat_z", 0.0, ""),
            BT::InputPort<double>("object_quat_w", 1.0, ""),
            // —— 抓取变换 ——
            // receive_offset: RECEIVE 相对物体中心的偏移；坐标系由 *_frame 指定
            //   world  (默认, 已验证配方 §17): (-0.0086, -0.054, +0.0009) —— 世界轴对齐偏移，
            //          杆有 ±5° 倾斜时仍把偏移按世界轴加，与脚本 R_move_cart_line 完全一致
            //   object: 偏移在物体系表达（(-0.054,0.0086,0.0009)），姿态随物体倾斜
            BT::InputPort<std::vector<double>>(
                "receive_offset", std::vector<double>{-0.0086, -0.054, 0.0009},
                "RECEIVE 相对物体中心的偏移(m)"),
            BT::InputPort<std::string>("receive_offset_frame", "world", "world | object"),
            BT::InputPort<std::vector<double>>(
                "approach_axis", std::vector<double>{1.0, 0.0, 0.0},
                "进给方向(单位向量)：PRE = RECEIVE - d·轴"),
            BT::InputPort<std::string>("approach_axis_frame", "world", "world | object"),
            BT::InputPort<double>("approach_distance", 0.060, "RECEIVE 之前多远生成 PRE(m)"),
            BT::InputPort<std::vector<double>>(
                "receive_quat", std::vector<double>{0.0087, 0.6819, 0.0060, 0.7314},
                "接物姿态(世界系四元数 x;y;z;w)＝右爪接物固定姿态(FK(R_SOLVED))"),
            BT::InputPort<std::vector<double>>(
                "receive_quat_in_object", std::vector<double>{},
                "可选：{}^O q_{G_R}（给空则用 receive_quat 的世界姿态）"),
            BT::InputPort<double>("timeout_sec", 5.0, "查场景/TF 的超时(s)"),
            // —— 输出 ——
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("right_receive_pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("right_receive_pre_pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("object_pose_world"),
            BT::OutputPort<geometry_msgs::msg::Pose>("object_pose_in_right_grasp",
                                                     "{}^{G_R} T_O（给 TransferObject 用）"),
            BT::OutputPort<geometry_msgs::msg::Pose>(
                "object_pose_in_left_grasp_rt",
                "{}^{G_L} T_O（运行期实际用的那个，给第二次 AttachObject 用）"),
            BT::OutputPort<bool>("used_scene_pose", "位姿是否来自 PlanningScene"),
        };
    }

    GenerateHandoverCandidateNode::GenerateHandoverCandidateNode(
        const std::string &name, const BT::NodeConfig &config, RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(ros)
    {
        scene_cli_ = ros_->node->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/move_group_fairino/get_planning_scene");
    }

    bool GenerateHandoverCandidateNode::objectPoseFromScene(
        const std::string &object_name, const std::string &left_link,
        geometry_msgs::msg::Pose *out, std::string *err)
    {
        if (!scene_cli_->service_is_ready() && !scene_cli_->wait_for_service(3s))
        {
            if (err)
                *err = "get_planning_scene unavailable";
            return false;
        }
        auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        req->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
            moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
        auto fut = scene_cli_->async_send_request(req);
        if (!wait_ready(fut, 5.0))
        {
            if (err)
                *err = "get_planning_scene timeout";
            return false;
        }
        auto res = fut.get();
        if (!res)
        {
            if (err)
                *err = "get_planning_scene null";
            return false;
        }
        for (const auto &aco : res->scene.robot_state.attached_collision_objects)
        {
            if (aco.object.id != object_name)
                continue;
            // 只认挂在 left_link 上的那一个（场景里可能残留别的臂上的同名/其它杆）
            if (!aco.link_name.empty() && aco.link_name != left_link)
                continue;
            const auto &o = aco.object;
            // MoveIt 把几何位姿归一化进 object.pose（实测：primitive_poses[0] 变 identity）
            if (!pose_is_identity(o.pose))
                *out = o.pose;
            else if (!o.primitive_poses.empty())
                *out = o.primitive_poses[0];
            else
                *out = o.pose;
            return true;
        }
        if (err)
            *err = "attached object '" + object_name + "' not found in scene";
        return false;
    }

    BT::NodeStatus GenerateHandoverCandidateNode::tick()
    {
        std::string object_name, left_link, world_frame;
        bool use_scene = true;
        double timeout = 5.0, approach_distance = 0.060;
        std::vector<double> receive_offset, approach_axis, recv_quat, recv_quat_obj;
        std::string recv_frame = "world", appr_frame = "world";
        getInput("object_name", object_name);
        getInput("left_link", left_link);
        getInput("world_frame", world_frame);
        getInput("use_scene_pose", use_scene);
        getInput("timeout_sec", timeout);
        getInput("receive_offset", receive_offset);
        getInput("receive_offset_frame", recv_frame);
        getInput("approach_axis", approach_axis);
        getInput("approach_axis_frame", appr_frame);
        getInput("approach_distance", approach_distance);
        getInput("receive_quat", recv_quat);
        getInput("receive_quat_in_object", recv_quat_obj);

        const auto log = ros_->node->get_logger();

        // ---- 1. 物体在 left_link 系里的位姿 ----
        geometry_msgs::msg::Pose l_pose;
        bool from_scene = false;
        if (use_scene)
        {
            std::string err;
            from_scene = objectPoseFromScene(object_name, left_link, &l_pose, &err);
            if (!from_scene)
                RCLCPP_WARN(log, "GenerateHandoverCandidate: 场景取位姿失败(%s)，回落 YAML 端口",
                            err.c_str());
        }
        if (!from_scene)
        {
            getInput("object_pos_x", l_pose.position.x);
            getInput("object_pos_y", l_pose.position.y);
            getInput("object_pos_z", l_pose.position.z);
            getInput("object_quat_x", l_pose.orientation.x);
            getInput("object_quat_y", l_pose.orientation.y);
            getInput("object_quat_z", l_pose.orientation.z);
            getInput("object_quat_w", l_pose.orientation.w);
        }

        // ---- 2. {}^W T_O = {}^W T_{G_L} · {}^{G_L} T_O ----
        geometry_msgs::msg::TransformStamped w_t_l;
        try
        {
            w_t_l = ros_->tf_buffer->lookupTransform(world_frame, left_link,
                                                     tf2::TimePointZero,
                                                     tf2::durationFromSec(timeout));
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(log, "GenerateHandoverCandidate: TF %s<-%s 失败: %s",
                         world_frame.c_str(), left_link.c_str(), e.what());
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::PoseStamped w_o;
        w_o.header = w_t_l.header;
        tf2::doTransform(l_pose, w_o.pose, w_t_l);

        // ---- 3. {}^W T_{G_R}^* = {}^W T_O · {}^O T_{G_R}^* ----
        if (receive_offset.size() != 3 || approach_axis.size() != 3)
        {
            RCLCPP_ERROR(log,
                         "GenerateHandoverCandidate: receive_offset/approach_axis 必须是 3 个数"
                         " (得到 %zu/%zu)",
                         receive_offset.size(), approach_axis.size());
            return BT::NodeStatus::FAILURE;
        }
        tf2::Quaternion q_o(w_o.pose.orientation.x, w_o.pose.orientation.y,
                            w_o.pose.orientation.z, w_o.pose.orientation.w);
        q_o.normalize();
        tf2::Matrix3x3 R_o(q_o);
        const tf2::Vector3 recv_off(receive_offset[0], receive_offset[1], receive_offset[2]);
        const tf2::Vector3 appr_axis(approach_axis[0], approach_axis[1], approach_axis[2]);
        const bool recv_local_frame = (recv_frame == "object");
        const bool appr_local_frame = (appr_frame == "object");
        const tf2::Vector3 recv_local = recv_local_frame ? (R_o * recv_off) : recv_off;
        const tf2::Vector3 appr_world = appr_local_frame ? (R_o * appr_axis) : appr_axis;

        geometry_msgs::msg::PoseStamped recv;
        recv.header = w_o.header;
        recv.pose.position.x = w_o.pose.position.x + recv_local.x();
        recv.pose.position.y = w_o.pose.position.y + recv_local.y();
        recv.pose.position.z = w_o.pose.position.z + recv_local.z();

        // 接物姿态：优先用 {}^O T_{G_R} 给出的相对姿态；否则用世界系固定姿态
        if (recv_quat_obj.size() == 4)
        {
            tf2::Quaternion q_rel(recv_quat_obj[0], recv_quat_obj[1],
                                  recv_quat_obj[2], recv_quat_obj[3]);
            q_rel.normalize();
            const tf2::Quaternion q_world = q_o * q_rel;
            recv.pose.orientation.x = q_world.x();
            recv.pose.orientation.y = q_world.y();
            recv.pose.orientation.z = q_world.z();
            recv.pose.orientation.w = q_world.w();
        }
        else if (recv_quat.size() == 4)
        {
            recv.pose.orientation.x = recv_quat[0];
            recv.pose.orientation.y = recv_quat[1];
            recv.pose.orientation.z = recv_quat[2];
            recv.pose.orientation.w = recv_quat[3];
        }
        else
        {
            RCLCPP_WARN(log, "GenerateHandoverCandidate: 无 receive_quat，回落 object 姿态");
            recv.pose.orientation = w_o.pose.orientation;
        }

        geometry_msgs::msg::PoseStamped pre = recv;
        // PRE = RECEIVE - d·n̂（脚本: tgt_pre = tgt - (0.06,0,0)）
        pre.pose.position.x -= appr_world.x() * approach_distance;
        pre.pose.position.y -= appr_world.y() * approach_distance;
        pre.pose.position.z -= appr_world.z() * approach_distance;

        // ---- 4. {}^{G_R} T_O = ({}^W T_O)^{-1} · {}^W T_{G_R}^* ----
        tf2::Transform W_T_O, W_T_GR;
        tf2::fromMsg(w_o.pose, W_T_O);
        tf2::fromMsg(recv.pose, W_T_GR);
        const geometry_msgs::msg::Pose obj_in_right =
            pose_from_tf(W_T_O.inverseTimes(W_T_GR));

        setOutput("right_receive_pose", recv);
        setOutput("right_receive_pre_pose", pre);
        setOutput("object_pose_world", w_o);
        setOutput("object_pose_in_left_grasp_rt", l_pose);
        setOutput("object_pose_in_right_grasp", obj_in_right);
        setOutput("used_scene_pose", from_scene);

        RCLCPP_INFO(log,
                    "GenerateHandoverCandidate: obj_src=%s W_T_O=(%.4f, %.4f, %.4f) "
                    "| receive=(%.4f, %.4f, %.4f) pre=(%.4f, %.4f, %.4f)",
                    from_scene ? "scene" : "yaml",
                    w_o.pose.position.x, w_o.pose.position.y, w_o.pose.position.z,
                    recv.pose.position.x, recv.pose.position.y, recv.pose.position.z,
                    pre.pose.position.x, pre.pose.position.y, pre.pose.position.z);
        return BT::NodeStatus::SUCCESS;
    }

    // ==================================================================
    // PlanToJoints
    // ==================================================================
    BT::PortsList PlanToJointsNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("arm_prefix", "right", "left | right"),
            BT::InputPort<std::string>("log_tag", "", "日志前缀"),
            BT::InputPort<std::vector<double>>("joint_positions", "6 个关节目标（与 target_pose 二选一）"),
            BT::InputPort<std::vector<double>>(
                "base_joint_positions",
                "反解基点（默认当前关节）；给 R_SOLVED 时 j4..j6 固定为其值，与脚本等价"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose",
                                                           "给姿态时用 FK+雅可比反解 j1..j3"),
            BT::InputPort<double>("position_tol", 1e-4, "反解位置收敛容差(m)"),
            BT::InputPort<float>("velocity_scale", 0.06f, ""),
            BT::InputPort<float>("acceleration_scale", 0.06f, ""),
            BT::InputPort<double>("planning_time", 12.0, "OMPL allowed_planning_time (sim s)"),
            BT::InputPort<int>("num_attempts", 20, ""),
            BT::InputPort<double>("timeout_sec", 120.0, "执行超时(sim s)"),
            BT::InputPort<std::string>("fk_service", "/move_group_fairino/compute_fk", ""),
            BT::InputPort<std::string>("planner_service",
                                       "/move_group_fairino/plan_kinematic_path", ""),
            BT::OutputPort<std::vector<double>>("solved_joint_positions"),
        };
    }

    PlanToJointsNode::PlanToJointsNode(const std::string &name,
                                       const BT::NodeConfig &config,
                                       rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), HandoverMotionBase(node)
    {
    }

    BT::NodeStatus PlanToJointsNode::onStart()
    {
        std::string arm_prefix, fk_service, planner_service, tag;
        std::vector<double> q_in, q_base_in;
        geometry_msgs::msg::PoseStamped target;
        float v_scale = 0.06f, a_scale = 0.06f;
        double planning_time = 12.0, timeout = 120.0, pos_tol = 1e-4;
        int attempts = 20;
        getInput("arm_prefix", arm_prefix);
        getInput("log_tag", tag);
        getInput("joint_positions", q_in);
        getInput("base_joint_positions", q_base_in);
        const bool has_pose = getInput("target_pose", target).has_value();
        getInput("velocity_scale", v_scale);
        getInput("acceleration_scale", a_scale);
        getInput("planning_time", planning_time);
        getInput("num_attempts", attempts);
        getInput("timeout_sec", timeout);
        getInput("position_tol", pos_tol);
        getInput("fk_service", fk_service);
        getInput("planner_service", planner_service);
        timeout_sec_ = timeout;

        bindArm(arm_prefix, fk_service);
        if (!fk_->waitJoints(15.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: /joint_states 未就绪",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        const auto q_now = fk_->currentJoints();

        if (q_in.size() == 6)
        {
            q_target_ = q_in;
        }
        else if (has_pose)
        {
            // 反解基点：默认当前关节；给了 base_joint_positions 就用它（j4..j6 取基点值）
            const std::vector<double> q_base = (q_base_in.size() == 6) ? q_base_in : q_now;
            geometry_msgs::msg::Pose p_base;
            if (!fk_->fk(q_base, &p_base, 5.0))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: FK 失败", arm_prefix.c_str());
                return BT::NodeStatus::FAILURE;
            }
            const Vec3 dv{target.pose.position.x - p_base.position.x,
                          target.pose.position.y - p_base.position.y,
                          target.pose.position.z - p_base.position.z};
            Vec3 achieved{};
            if (!fk_->solveDelta(q_base, dv, pos_tol, &q_target_, &achieved))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: 位置反解失败",
                             arm_prefix.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "PlanToJoints[%s]: 反解 dv=(%.4f,%.4f,%.4f) → 达成 (%.4f,%.4f,%.4f)",
                        arm_prefix.c_str(), dv[0], dv[1], dv[2],
                        achieved[0], achieved[1], achieved[2]);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanToJoints[%s]: 需要 joint_positions(6) 或 target_pose",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        setOutput("solved_joint_positions", q_target_);

        if (!plan_cli_)
            plan_cli_ = node_->create_client<moveit_msgs::srv::GetMotionPlan>(planner_service);
        if (!plan_cli_->service_is_ready() && !plan_cli_->wait_for_service(5s))
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanToJoints: %s 不可用", planner_service.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
        auto &mpr = req->motion_plan_request;
        mpr.group_name = arm_prefix + "_arm";
        mpr.planner_id = "RRTConnect";
        mpr.pipeline_id = "ompl";
        mpr.num_planning_attempts = attempts;
        mpr.allowed_planning_time = planning_time;
        mpr.max_velocity_scaling_factor = v_scale;
        mpr.max_acceleration_scaling_factor = a_scale;
        const auto names = arm_joint_names(arm_prefix);
        mpr.start_state.joint_state.name = names;
        mpr.start_state.joint_state.position.assign(q_now.begin(), q_now.end());
        mpr.start_state.is_diff = false;
        moveit_msgs::msg::Constraints cc;
        for (size_t i = 0; i < names.size(); ++i)
        {
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name = names[i];
            jc.position = q_target_[i];
            jc.tolerance_above = 0.001;
            jc.tolerance_below = 0.001;
            jc.weight = 1.0;
            cc.joint_constraints.push_back(jc);
        }
        mpr.goal_constraints.push_back(cc);

        auto fut = plan_cli_->async_send_request(req);
        if (!wait_ready(fut, planning_time * 3.0 + 20.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: 规划服务超时",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto res = fut.get();
        const int code = res ? res->motion_plan_response.error_code.val : -999;
        if (code != 1)
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: 规划失败 code=%d",
                         arm_prefix.c_str(), code);
            return BT::NodeStatus::FAILURE;
        }
        const auto &jt = res->motion_plan_response.trajectory.joint_trajectory;
        RCLCPP_INFO(node_->get_logger(), "PlanToJoints[%s]: 规划 OK (%zu 点)",
                    arm_prefix.c_str(), jt.points.size());
        std::string err;
        if (!sendTrajectory(jt, timeout_sec_, &err))
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanToJoints[%s]: 执行失败 (%s)",
                         arm_prefix.c_str(), err.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "PlanToJoints[%s]%s: 完成", arm_prefix.c_str(),
                    tag.empty() ? "" : ("[" + tag + "]").c_str());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus PlanToJointsNode::onRunning() { return BT::NodeStatus::SUCCESS; }
    void PlanToJointsNode::onHalted() {}

    // ==================================================================
    // CartesianApproach
    // ==================================================================
    BT::PortsList CartesianApproachNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("arm_prefix", "right", "left | right"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "世界系目标 TCP 姿态"),
            BT::InputPort<std::vector<double>>("delta",
                                               "相对当前 TCP 的世界位移 dxyz(m)（与 target_pose 二选一）"),
            BT::InputPort<std::vector<double>>(
                "base_joint_positions",
                "反解基点（默认当前关节）；给 R_SOLVED 时 j4..j6 固定为其值，与脚本等价"),
            BT::InputPort<int>("npts", 24, "直线上采样点数"),
            BT::InputPort<double>("secs", 6.0, "轨迹时长(sim s)"),
            BT::InputPort<double>("timeout_sec", 120.0, "执行超时(sim s)"),
            BT::InputPort<double>("position_tol", 1e-5, "逐点反解收敛容差(m)"),
            BT::InputPort<std::string>("fk_service", "/move_group_fairino/compute_fk", ""),
            BT::InputPort<std::string>("log_tag", "", "日志前缀"),
            BT::OutputPort<std::vector<double>>("start_tcp", "起始 TCP 位置（world）"),
            BT::OutputPort<std::vector<double>>("goal_tcp", "目标 TCP 位置（world）"),
        };
    }

    CartesianApproachNode::CartesianApproachNode(const std::string &name,
                                                 const BT::NodeConfig &config,
                                                 rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), HandoverMotionBase(node)
    {
    }

    BT::NodeStatus CartesianApproachNode::onStart()
    {
        std::string arm_prefix, fk_service, tag;
        geometry_msgs::msg::PoseStamped target;
        std::vector<double> delta, q_base_in;
        int npts = 24;
        double secs = 6.0, timeout = 120.0, pos_tol = 1e-5;
        getInput("arm_prefix", arm_prefix);
        getInput("log_tag", tag);
        getInput("base_joint_positions", q_base_in);
        const bool has_pose = getInput("target_pose", target).has_value();
        const bool has_delta = getInput("delta", delta).has_value();
        getInput("npts", npts);
        getInput("secs", secs);
        getInput("timeout_sec", timeout);
        getInput("position_tol", pos_tol);
        getInput("fk_service", fk_service);
        timeout_sec_ = timeout;

        bindArm(arm_prefix, fk_service);
        if (!fk_->waitJoints(15.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "CartesianApproach[%s]: /joint_states 未就绪",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        const auto q_from = fk_->currentJoints();
        // 反解基点：默认当前关节；给 base_joint_positions 时 j4..j6 固定为其值
        const std::vector<double> q_solve = (q_base_in.size() == 6) ? q_base_in : q_from;
        geometry_msgs::msg::Pose p_from, p_base;
        if (!fk_->fk(q_from, &p_from, 5.0) || !fk_->fk(q_solve, &p_base, 5.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "CartesianApproach[%s]: 起始 FK 失败",
                         arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }
        // dv 一律相对反解基点算（与脚本 solve_receive 的 base 一致）
        const Vec3 base_pos{p_base.position.x, p_base.position.y, p_base.position.z};

        const Vec3 p_start{p_from.position.x, p_from.position.y, p_from.position.z};
        Vec3 p_to = p_start;
        if (has_pose)
        {
            p_to = {target.pose.position.x, target.pose.position.y, target.pose.position.z};
        }
        else if (has_delta && delta.size() == 3)
        {
            p_to = {p_start[0] + delta[0], p_start[1] + delta[1], p_start[2] + delta[2]};
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "CartesianApproach[%s]: 需要 target_pose 或 delta(3)", arm_prefix.c_str());
            return BT::NodeStatus::FAILURE;
        }

        setOutput("start_tcp", std::vector<double>{p_start[0], p_start[1], p_start[2]});
        setOutput("goal_tcp", std::vector<double>{p_to[0], p_to[1], p_to[2]});

        // 沿线逐点 FK 反解 → 一条 JTC 轨迹（关节空间插值的 TCP 会画弧, 见 M4_2 §17）
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = arm_joint_names(arm_prefix);
        const std::string label = tag.empty() ? "" : ("[" + tag + "]");
        for (int k = 0; k <= npts; ++k)
        {
            const double t = static_cast<double>(k) / npts;
            const Vec3 tgt{p_start[0] + (p_to[0] - p_start[0]) * t,
                           p_start[1] + (p_to[1] - p_start[1]) * t,
                           p_start[2] + (p_to[2] - p_start[2]) * t};
            const Vec3 dv{tgt[0] - base_pos[0], tgt[1] - base_pos[1], tgt[2] - base_pos[2]};
            std::vector<double> q;
            Vec3 ach{};
            if (!fk_->solveDelta(q_solve, dv, pos_tol, &q, &ach))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "CartesianApproach[%s]%s: 第 %d/%d 点反解失败",
                             arm_prefix.c_str(), label.c_str(), k, npts);
                return BT::NodeStatus::FAILURE;
            }
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.assign(q.begin(), q.end());
            pt.velocities.assign(6, 0.0);
            const double tt = secs * t;
            pt.time_from_start.sec = static_cast<int32_t>(tt);
            pt.time_from_start.nanosec = static_cast<uint32_t>((tt - static_cast<int>(tt)) * 1e9);
            traj.points.push_back(pt);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "CartesianApproach[%s]%s: 直线 (%.4f,%.4f,%.4f)→(%.4f,%.4f,%.4f) %d 点, %.1fs",
                    arm_prefix.c_str(), label.c_str(),
                    p_start[0], p_start[1], p_start[2], p_to[0], p_to[1], p_to[2],
                    npts + 1, secs);

        std::string err;
        if (!sendTrajectory(traj, timeout_sec_, &err))
        {
            RCLCPP_ERROR(node_->get_logger(), "CartesianApproach[%s]%s: 执行失败 (%s)",
                         arm_prefix.c_str(), label.c_str(), err.c_str());
            return BT::NodeStatus::FAILURE;
        }
        // 完成后再 FK 一次，报告"命令位移 vs 实际达成位移"（跟随率/是否被挡住的直接证据）
        geometry_msgs::msg::Pose p_end;
        if (fk_->fk(fk_->currentJoints(), &p_end, 5.0))
        {
            const double dz_cmd = p_to[2] - p_start[2];
            const double dz_ach = p_end.position.z - p_start[2];
            RCLCPP_INFO(node_->get_logger(),
                        "CartesianApproach[%s]%s: 完成  Δz 命令=%.1fmm 实际=%.1fmm  "
                        "Δxyz 实际=(%.1f, %.1f, %.1f)mm",
                        arm_prefix.c_str(), label.c_str(), dz_cmd * 1000, dz_ach * 1000,
                        (p_end.position.x - p_start[0]) * 1000,
                        (p_end.position.y - p_start[1]) * 1000, dz_ach * 1000);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "CartesianApproach[%s]%s: 完成",
                        arm_prefix.c_str(), label.c_str());
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus CartesianApproachNode::onRunning() { return BT::NodeStatus::SUCCESS; }
    void CartesianApproachNode::onHalted() {}

    // ==================================================================
    // ==================================================================
    // MeasureObjectInGrasp：量"当前 TCP 相对物体中心"的世界偏移（诊断 + 指标）
    //   tcp_minus_rod = p_tcp - p_rod (world)
    //   期望 ≈ receive_offset（world 模式）→ 偏差即"没夹到"的真实几何原因
    // ==================================================================
    BT::PortsList MeasureObjectInGraspNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_name", "handover_bar", ""),
            BT::InputPort<std::string>("left_link", "left_grasp_frame", ""),
            BT::InputPort<std::string>("world_frame", "world", ""),
            BT::InputPort<std::string>("arm_prefix", "right", "量哪条臂的 TCP"),
            BT::InputPort<std::string>("fk_service", "/move_group_fairino/compute_fk", ""),
            BT::InputPort<std::vector<double>>("expected_offset", std::vector<double>{},
                                               "期望的 rod_minus_tcp（给空则不比对）"),
            BT::InputPort<double>("tolerance", 0.010, "偏差告警阈值(m)"),
            BT::InputPort<std::string>("log_tag", "", "日志前缀"),
            BT::OutputPort<std::vector<double>>("tcp_minus_rod",
                                               "p_tcp - p_rod (world)（到位时应 ≈ receive_offset）"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("object_pose_world_measured"),
            BT::OutputPort<bool>("offset_ok", "偏差是否在容差内"),
        };
    }

    MeasureObjectInGraspNode::MeasureObjectInGraspNode(const std::string &name,
                                                       const BT::NodeConfig &config,
                                                       RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(ros)
    {
    }

    BT::NodeStatus MeasureObjectInGraspNode::tick()
    {
        std::string object_name, left_link, world_frame, arm, fk_service, tag;
        std::vector<double> expected;
        double tol = 0.010;
        getInput("object_name", object_name);
        getInput("left_link", left_link);
        getInput("world_frame", world_frame);
        getInput("arm_prefix", arm);
        getInput("fk_service", fk_service);
        getInput("expected_offset", expected);
        getInput("tolerance", tol);
        getInput("log_tag", tag);
        const auto log = ros_->node->get_logger();

        // 物体世界位姿：场景 attached object（挂在 left_link 上）+ TF
        auto scene_cli = ros_->node->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/move_group_fairino/get_planning_scene");
        if (!scene_cli->service_is_ready() && !scene_cli->wait_for_service(3s))
        {
            RCLCPP_ERROR(log, "MeasureObjectInGrasp: get_planning_scene 不可用");
            return BT::NodeStatus::FAILURE;
        }
        auto sreq = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        sreq->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
            moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
        auto sfut = scene_cli->async_send_request(sreq);
        if (!wait_ready(sfut, 5.0))
        {
            RCLCPP_ERROR(log, "MeasureObjectInGrasp: 场景查询超时");
            return BT::NodeStatus::FAILURE;
        }
        auto sres = sfut.get();
        bool found = false;
        geometry_msgs::msg::Pose l_pose;
        if (sres)
        {
            for (const auto &aco : sres->scene.robot_state.attached_collision_objects)
            {
                if (aco.object.id != object_name)
                    continue;
                if (!aco.link_name.empty() && aco.link_name != left_link)
                    continue;
                l_pose = pose_is_identity(aco.object.pose)
                             ? (aco.object.primitive_poses.empty()
                                    ? aco.object.pose
                                    : aco.object.primitive_poses[0])
                             : aco.object.pose;
                found = true;
                break;
            }
        }
        if (!found)
        {
            RCLCPP_WARN(log, "MeasureObjectInGrasp: 场景里没有 %s（挂 %s）",
                        object_name.c_str(), left_link.c_str());
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::TransformStamped w_t_l;
        try
        {
            w_t_l = ros_->tf_buffer->lookupTransform(world_frame, left_link,
                                                     tf2::TimePointZero,
                                                     tf2::durationFromSec(3.0));
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(log, "MeasureObjectInGrasp: TF 失败 %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::PoseStamped w_o;
        w_o.header = w_t_l.header;
        tf2::doTransform(l_pose, w_o.pose, w_t_l);

        // 当前 TCP
        ArmFkHelper fk(ros_->node, fk_service);
        fk.setArmPrefix(arm);
        if (!fk.waitJoints(15.0))
        {
            RCLCPP_ERROR(log, "MeasureObjectInGrasp: /joint_states 未就绪");
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::Pose tcp;
        if (!fk.fk(fk.currentJoints(), &tcp, 5.0))
        {
            RCLCPP_ERROR(log, "MeasureObjectInGrasp: FK 失败");
            return BT::NodeStatus::FAILURE;
        }
        const std::vector<double> d{
            tcp.position.x - w_o.pose.position.x,
            tcp.position.y - w_o.pose.position.y,
            tcp.position.z - w_o.pose.position.z};
        bool ok = true;
        std::string cmp;
        if (expected.size() == 3)
        {
            const double err = std::sqrt(std::pow(d[0] - expected[0], 2) +
                                         std::pow(d[1] - expected[1], 2) +
                                         std::pow(d[2] - expected[2], 2));
            ok = err <= tol;
            std::ostringstream os;
            os << " 期望=(" << expected[0] << "," << expected[1] << "," << expected[2]
               << ") 偏差=" << err * 1000 << "mm";
            cmp = os.str();
        }
        setOutput("tcp_minus_rod", d);
        setOutput("object_pose_world_measured", w_o);
        setOutput("offset_ok", ok);
        RCLCPP_INFO(log, "MeasureObjectInGrasp[%s]%s: tcp-rod=(%.4f, %.4f, %.4f)%s",
                    arm.c_str(), tag.empty() ? "" : ("[" + tag + "]").c_str(),
                    d[0], d[1], d[2], cmp.c_str());
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // ==================================================================
    // GenerateLeftGraspCandidate
    // ==================================================================
    BT::PortsList GenerateLeftGraspCandidateNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("world_frame", "world", ""),
            // 输出坐标系：move_to_pose_server **不解析 header.frame_id**（当 base 系用），
            // 所以这里必须用 TF 把 world 目标转成 <arm>_base_link。
            BT::InputPort<std::string>("output_frame", "left_base_link", ""),
            BT::InputPort<std::vector<double>>(
                "object_pos_world", std::vector<double>{0.04, 0.0, 0.0155},
                "长条物体中心的世界坐标（YAML 标定；后续换视觉/场景来源）"),
            BT::InputPort<double>("pregrasp_height", 0.165,
                                  "pregrasp 相对物体中心的高度(m)（配方 0.18-0.0155）"),
            BT::InputPort<std::vector<double>>(
                "pregrasp_quat", std::vector<double>{1.0, 0.0, 0.0, 0.0},
                "pregrasp 姿态（world 四元数 x;y;z;w，左爪朝下 = roll π）"),
            BT::InputPort<double>("descend_distance", 0.153,
                                  "servo descend 距离(m)（力控下重标值）"),
            BT::InputPort<std::vector<double>>(
                "object_pose_in_left_grasp", std::vector<double>{-0.0148, -0.0002, 0.0139},
                "物体中心在 left_grasp_frame 下的位置（YAML 标定，给 AttachObject）"),
            BT::InputPort<std::vector<double>>(
                "object_quat_in_left_grasp", std::vector<double>{-0.7040, -0.7101, 0.0005, 0.0079},
                "同上姿态四元数"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pregrasp_pose"),
            BT::OutputPort<double>("descend_distance_out"),
            BT::OutputPort<geometry_msgs::msg::Pose>("object_pose_in_left_grasp_pose"),
        };
    }

    GenerateLeftGraspCandidateNode::GenerateLeftGraspCandidateNode(
        const std::string &name, const BT::NodeConfig &config, RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(ros)
    {
    }

    BT::NodeStatus GenerateLeftGraspCandidateNode::tick()
    {
        std::string world_frame, out_frame;
        std::vector<double> obj, quat, lpos, lquat;
        double h = 0.165, descend = 0.153;
        getInput("world_frame", world_frame);
        getInput("output_frame", out_frame);
        getInput("object_pos_world", obj);
        getInput("pregrasp_height", h);
        getInput("pregrasp_quat", quat);
        getInput("descend_distance", descend);
        getInput("object_pose_in_left_grasp", lpos);
        getInput("object_quat_in_left_grasp", lquat);
        const auto log = ros_->node->get_logger();
        if (obj.size() != 3 || quat.size() != 4)
        {
            RCLCPP_ERROR(log, "GenerateLeftGraspCandidate: object_pos_world(3)/pregrasp_quat(4) 缺失");
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::PoseStamped pg;
        pg.header.frame_id = world_frame;
        pg.header.stamp = ros_->node->now();
        pg.pose.position.x = obj[0];
        pg.pose.position.y = obj[1];
        pg.pose.position.z = obj[2] + h;
        pg.pose.orientation.x = quat[0];
        pg.pose.orientation.y = quat[1];
        pg.pose.orientation.z = quat[2];
        pg.pose.orientation.w = quat[3];
        // world → output_frame（move_to_pose_server 只认 base 系坐标）
        try
        {
            const auto tf = ros_->tf_buffer->lookupTransform(
                out_frame, world_frame, tf2::TimePointZero, tf2::durationFromSec(3.0));
            geometry_msgs::msg::PoseStamped pg_out;
            pg_out.header = tf.header;
            tf2::doTransform(pg.pose, pg_out.pose, tf);
            pg = pg_out;
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(log, "GenerateLeftGraspCandidate: TF %s<-%s 失败: %s",
                         out_frame.c_str(), world_frame.c_str(), e.what());
            return BT::NodeStatus::FAILURE;
        }
        setOutput("pregrasp_pose", pg);
        setOutput("descend_distance_out", descend);
        geometry_msgs::msg::Pose lp;
        if (lpos.size() == 3)
        {
            lp.position.x = lpos[0];
            lp.position.y = lpos[1];
            lp.position.z = lpos[2];
        }
        if (lquat.size() == 4)
        {
            lp.orientation.x = lquat[0];
            lp.orientation.y = lquat[1];
            lp.orientation.z = lquat[2];
            lp.orientation.w = lquat[3];
        }
        setOutput("object_pose_in_left_grasp_pose", lp);
        RCLCPP_INFO(log,
                    "GenerateLeftGraspCandidate: obj_world=(%.4f, %.4f, %.4f) → pregrasp=(%.4f, %.4f, %.4f) "
                    "descend=%.3f  {}^{G_L}T_O=(%.4f, %.4f, %.4f)",
                    obj[0], obj[1], obj[2], pg.pose.position.x, pg.pose.position.y,
                    pg.pose.position.z, descend, lp.position.x, lp.position.y, lp.position.z);
        return BT::NodeStatus::SUCCESS;
    }

    void registerHandoverNodes(BT::BehaviorTreeFactory &factory,
                               RosContextPtr ros,
                               rclcpp::Node::SharedPtr node)
    {
        factory.registerBuilder<GenerateHandoverCandidateNode>(
            "GenerateHandoverCandidate",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<GenerateHandoverCandidateNode>(name, config, ros); });
        factory.registerBuilder<PlanToJointsNode>(
            "PlanToJoints",
            [node](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<PlanToJointsNode>(name, config, node); });
        factory.registerBuilder<CartesianApproachNode>(
            "CartesianApproach",
            [node](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<CartesianApproachNode>(name, config, node); });
        factory.registerBuilder<GenerateLeftGraspCandidateNode>(
            "GenerateLeftGraspCandidate",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<GenerateLeftGraspCandidateNode>(name, config, ros); });
        factory.registerBuilder<MeasureObjectInGraspNode>(
            "MeasureObjectInGrasp",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<MeasureObjectInGraspNode>(name, config, ros); });
    }

} // namespace s622_bt
