#include "s622_bt_manager/grasp_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace s622_bt
{

    namespace
    {
        // 5x5 中值采样深度，单位米
        double samplePixelDepth(const sensor_msgs::msg::Image &depth_img,
                                int u, int v, int kernel = 5)
        {
            if (depth_img.encoding != "32FC1")
                return std::nan("");
            std::vector<float> samples;
            int half = kernel / 2;
            int w = depth_img.width;
            int h = depth_img.height;
            const float *data = reinterpret_cast<const float *>(depth_img.data.data());
            for (int dy = -half; dy <= half; ++dy)
            {
                for (int dx = -half; dx <= half; ++dx)
                {
                    int x = u + dx, y = v + dy;
                    if (x < 0 || x >= w || y < 0 || y >= h)
                        continue;
                    float z = data[y * w + x];
                    if (std::isfinite(z) && z > 0.01f && z < 5.0f)
                        samples.push_back(z);
                }
            }
            if (samples.empty())
                return std::nan("");
            std::nth_element(samples.begin(),
                             samples.begin() + samples.size() / 2,
                             samples.end());
            return samples[samples.size() / 2];
        }

        double normalizeAngle(double a)
        {
            while (a > M_PI)
                a -= 2 * M_PI;
            while (a < -M_PI)
                a += 2 * M_PI;
            return a;
        }

        // cube 4 个等价 yaw 中选离 current_yaw 最近的
        double selectBestYaw(double obj_yaw, double current_yaw)
        {
            double best = normalizeAngle(obj_yaw);
            double best_err = std::abs(normalizeAngle(best - current_yaw));
            for (int k = 1; k < 4; ++k)
            {
                double cand = normalizeAngle(obj_yaw + k * M_PI / 2.0);
                double err = std::abs(normalizeAngle(cand - current_yaw));
                if (err < best_err)
                {
                    best_err = err;
                    best = cand;
                }
            }
            return best;
        }

        geometry_msgs::msg::Pose makePose(double x, double y, double z,
                                          double roll, double pitch, double yaw)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = x;
            p.position.y = y;
            p.position.z = z;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            p.orientation = tf2::toMsg(q);
            return p;
        }

        std::string derive_frame(const std::string &explicit_frame,
                                 const std::string &arm_prefix,
                                 const std::string &suffix,
                                 const std::string &m17_default)
        {
            if (!explicit_frame.empty())
                return explicit_frame;
            if (arm_prefix.empty())
                return m17_default;
            return arm_prefix + "_" + suffix;
        }

    } // namespace

    GenerateGraspCandidate::GenerateGraspCandidate(const std::string &name,
                                                   const BT::NodeConfig &config,
                                                   RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(std::move(ros)) {}

    BT::PortsList GenerateGraspCandidate::providedPorts()
    {
        return {
            BT::InputPort<double>("u"),
            BT::InputPort<double>("v"),
            BT::InputPort<double>("object_yaw"),
            BT::InputPort<std::string>("arm_prefix", "", "'' | 'left' | 'right', derives base/ee frames if not given"),
            BT::InputPort<double>("table_z", 0.0, "table z in base frame"),
            BT::InputPort<double>("grasp_height_above_table", 0.030, ""),
            BT::InputPort<double>("pregrasp_height_above_table", 0.16, ""),
            BT::InputPort<double>("pregrasp_camera_offset", 0.03, ""),
            // base_frame/ee_frame 空时按 arm_prefix 派生
            BT::InputPort<std::string>("base_frame", "", ""),
            BT::InputPort<std::string>("ee_frame", "", ""),
            BT::InputPort<double>("camera_x_in_base", 0.3825, ""),
            BT::InputPort<double>("camera_y_in_base", 0.4838, ""),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("tcp_grasp_pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("tcp_pregrasp_pose"),
            BT::OutputPort<double>("grasp_yaw"),
            BT::OutputPort<double>("grasp_pose_x"),
            BT::OutputPort<double>("grasp_pose_y"),
            // BT::OutputPort<double>("tcp_grasp_pose_x"),
            // BT::OutputPort<double>("tcp_grasp_pose_y"),
        };
    }

    BT::NodeStatus GenerateGraspCandidate::tick()
    {
        double u, v, obj_yaw;
        if (!getInput("u", u) || !getInput("v", v) || !getInput("object_yaw", obj_yaw))
        {
            RCLCPP_ERROR(ros_->node->get_logger(), "GenerateGraspCandidate: missing inputs");
            return BT::NodeStatus::FAILURE;
        }
        double table_z, grasp_dz, pregrasp_dz, cam_offset;
        double cam_x_base, cam_y_base;
        std::string base_frame_raw, ee_frame_raw, arm_prefix;
        getInput("table_z", table_z);
        getInput("grasp_height_above_table", grasp_dz);
        getInput("pregrasp_height_above_table", pregrasp_dz);
        getInput("pregrasp_camera_offset", cam_offset);
        getInput("base_frame", base_frame_raw);
        getInput("arm_prefix", arm_prefix);
        getInput("ee_frame", ee_frame_raw);
        getInput("camera_x_in_base", cam_x_base);
        getInput("camera_y_in_base", cam_y_base);

        // ---- 按 arm_prefix 派生 frame ----
        const std::string base_frame = derive_frame(base_frame_raw, arm_prefix,
                                                    "base_link", "base_link");
        const std::string ee_frame = derive_frame(ee_frame_raw, arm_prefix,
                                                  "grasp_frame", "grasp_frame");

        // 1) camera_info
        sensor_msgs::msg::CameraInfo::SharedPtr caminfo;
        {
            std::lock_guard<std::mutex> lk(ros_->caminfo_mutex);
            caminfo = ros_->latest_caminfo;
        }
        if (!caminfo)
        {
            RCLCPP_WARN(ros_->node->get_logger(), "no camera_info yet");
            return BT::NodeStatus::FAILURE;
        }
        double fx = caminfo->k[0], fy = caminfo->k[4];
        double cx = caminfo->k[2], cy = caminfo->k[5];
        std::string camera_frame = caminfo->header.frame_id;
        if (camera_frame.empty())
            camera_frame = "camera_color_optical_frame";

        // 2) 获取相机在 base 系的位姿（TF）
        geometry_msgs::msg::TransformStamped cam_tf;
        try
        {
            cam_tf = ros_->tf_buffer->lookupTransform(
                base_frame, camera_frame, tf2::TimePointZero,
                tf2::durationFromSec(0.5));
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN(ros_->node->get_logger(),
                        "TF %s->%s failed (arm=%s): %s, fallback to depth sensor",
                        base_frame.c_str(), camera_frame.c_str(),
                        arm_prefix.c_str(), e.what());
            goto use_depth;
        }

        // 3) 桌面平面交线法: 射线与 z=table_z+grasp_dz 平面求交
        //    绕过深度传感器对小目标的穿透问题
        {
            double cam_z = cam_tf.transform.translation.z;
            double dx = (u - cx) / fx; // 像素射线方向 (相机光轴系)
            double dy = (v - cy) / fy;

            // 相机光轴系 → base 系的旋转
            tf2::Quaternion q;
            tf2::fromMsg(cam_tf.transform.rotation, q);
            tf2::Matrix3x3 R(q);

            // 射线方向在 base 系
            double rx = R[0][0] * dx + R[0][1] * dy + R[0][2] * 1.0;
            double ry = R[1][0] * dx + R[1][1] * dy + R[1][2] * 1.0;
            double rz = R[2][0] * dx + R[2][1] * dy + R[2][2] * 1.0;

            double target_z = table_z + grasp_dz; // 方块顶面高度
            if (std::abs(rz) < 1e-6)
                goto use_depth; // 射线平行桌面, 回退深度传感器

            double t = (target_z - cam_z) / rz;
            if (t <= 0)
                goto use_depth; // 交点在相机后方, 回退

            double obj_x = cam_tf.transform.translation.x + t * rx;
            double obj_y = cam_tf.transform.translation.y + t * ry;

            RCLCPP_INFO(ros_->node->get_logger(),
                        "Table-plane[arm=%s]: t=%.3f -> base=(%.3f,%.3f)",
                        arm_prefix.c_str(), t, obj_x, obj_y);

            // 5) 当前 EE yaw
            double current_ee_yaw = 0.0;
            try
            {
                auto tf = ros_->tf_buffer->lookupTransform(base_frame, ee_frame,
                                                           tf2::TimePointZero,
                                                           tf2::durationFromSec(0.2));
                tf2::Quaternion eq;
                tf2::fromMsg(tf.transform.rotation, eq);
                double r, p, y;
                tf2::Matrix3x3(eq).getRPY(r, p, y);
                current_ee_yaw = y;
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(ros_->node->get_logger(),
                            "TF %s->%s failed (arm=%s), default ee_yaw=0: %s",
                            base_frame.c_str(), ee_frame.c_str(),
                            arm_prefix.c_str(), e.what());
            }

            double best_yaw = selectBestYaw(obj_yaw, current_ee_yaw);
            double obj_z_grasp = table_z + grasp_dz;

            // 6) grasp pose
            geometry_msgs::msg::PoseStamped grasp;
            grasp.header.frame_id = base_frame;
            grasp.header.stamp = ros_->node->now();
            grasp.pose = makePose(obj_x, obj_y, obj_z_grasp, M_PI, 0.0, best_yaw);

            // 7) pregrasp
            double ddx = obj_x - cam_tf.transform.translation.x;
            double ddy = obj_y - cam_tf.transform.translation.y;
            double norm = std::hypot(ddx, ddy);
            double off_x = norm > 1e-6 ? ddx / norm * cam_offset : 0.0;
            double off_y = norm > 1e-6 ? ddy / norm * cam_offset : 0.0;

            geometry_msgs::msg::PoseStamped pregrasp;
            pregrasp.header.frame_id = base_frame;
            pregrasp.header.stamp = grasp.header.stamp;
            pregrasp.pose = makePose(obj_x + off_x, obj_y + off_y,
                                     table_z + pregrasp_dz, M_PI, 0.0, best_yaw);

            setOutput("tcp_grasp_pose", grasp);
            setOutput("tcp_pregrasp_pose", pregrasp);
            setOutput("grasp_yaw", best_yaw);
            setOutput("grasp_pose_x", obj_x);
            setOutput("grasp_pose_y", obj_y);

            RCLCPP_INFO(ros_->node->get_logger(),
                        "Grasp[arm=%s plane]: uv=(%.1f,%.1f) -> base=(%.3f,%.3f,%.3f) "
                        "yaw=%.3f pregrasp=(%.3f,%.3f,%.3f)",
                        arm_prefix.c_str(), u, v, obj_x, obj_y, obj_z_grasp,
                        best_yaw,
                        pregrasp.pose.position.x, pregrasp.pose.position.y,
                        pregrasp.pose.position.z);

            // 8) 可视化
            if (ros_->grasp_viz_pub)
            {
                geometry_msgs::msg::PoseArray arr;
                arr.header = pregrasp.header;
                arr.poses.push_back(pregrasp.pose);
                arr.poses.push_back(grasp.pose);
                ros_->grasp_viz_pub->publish(arr);
            }

            return BT::NodeStatus::SUCCESS;
        }

    use_depth:
        // === 深度传感器回退路径 ===
        {
            sensor_msgs::msg::Image::SharedPtr depth;
            {
                std::lock_guard<std::mutex> lk(ros_->depth_mutex);
                depth = ros_->latest_depth;
            }
            if (!depth)
            {
                RCLCPP_WARN(ros_->node->get_logger(), "no depth yet");
                return BT::NodeStatus::FAILURE;
            }
            double z_cam = samplePixelDepth(*depth, int(std::round(u)), int(std::round(v)));
            if (!std::isfinite(z_cam) || z_cam <= 0)
            {
                RCLCPP_WARN(ros_->node->get_logger(), "invalid depth at (%.1f,%.1f)", u, v);
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_WARN(ros_->node->get_logger(),
                        "Using depth fallback[arm=%s]: z_cam=%.3f",
                        arm_prefix.c_str(), z_cam);

            // 3) pixel → camera frame
            double x_cam = (u - cx) * z_cam / fx;
            double y_cam = (v - cy) * z_cam / fy;

            // 4) camera → base
            geometry_msgs::msg::PointStamped p_cam, p_base;
            p_cam.header.frame_id = camera_frame;
            p_cam.header.stamp = depth->header.stamp;
            p_cam.point.x = x_cam;
            p_cam.point.y = y_cam;
            p_cam.point.z = z_cam;
            try
            {
                p_base = ros_->tf_buffer->transform(p_cam, base_frame,
                                                    tf2::durationFromSec(0.2));
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(ros_->node->get_logger(), "TF %s->%s failed: %s",
                            camera_frame.c_str(), base_frame.c_str(), e.what());
                return BT::NodeStatus::FAILURE;
            }
            double obj_x = p_base.point.x, obj_y = p_base.point.y;

            // 5) 当前 EE yaw
            double current_ee_yaw = 0.0;
            try
            {
                auto tf = ros_->tf_buffer->lookupTransform(base_frame, ee_frame,
                                                           tf2::TimePointZero,
                                                           tf2::durationFromSec(0.2));
                tf2::Quaternion q;
                tf2::fromMsg(tf.transform.rotation, q);
                double r, p, y;
                tf2::Matrix3x3(q).getRPY(r, p, y);
                current_ee_yaw = y;
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(ros_->node->get_logger(),
                            "TF %s->%s failed (arm=%s), default ee_yaw=0: %s",
                            base_frame.c_str(), ee_frame.c_str(),
                            arm_prefix.c_str(), e.what());
            }

            double best_yaw = selectBestYaw(obj_yaw, current_ee_yaw);

            // 6) grasp pose（z 用配置，不用反投影）
            geometry_msgs::msg::PoseStamped grasp;
            grasp.header.frame_id = base_frame;
            grasp.header.stamp = ros_->node->now();
            grasp.pose = makePose(obj_x, obj_y, table_z + grasp_dz, M_PI, 0.0, best_yaw);

            // 7) pregrasp：XY 沿"远离相机"方向偏移
            double dx = obj_x - cam_x_base, dy = obj_y - cam_y_base;
            double norm = std::hypot(dx, dy);
            double off_x = norm > 1e-6 ? dx / norm * cam_offset : 0.0;
            double off_y = norm > 1e-6 ? dy / norm * cam_offset : 0.0;

            geometry_msgs::msg::PoseStamped pregrasp;
            pregrasp.header.frame_id = base_frame;
            pregrasp.header.stamp = grasp.header.stamp;
            pregrasp.pose = makePose(obj_x + off_x, obj_y + off_y,
                                     table_z + pregrasp_dz, M_PI, 0.0, best_yaw);

            setOutput("tcp_grasp_pose", grasp);
            setOutput("tcp_pregrasp_pose", pregrasp);
            setOutput("grasp_yaw", best_yaw);
            setOutput("grasp_pose_x", obj_x);
            setOutput("grasp_pose_y", obj_y);
            // setOutput("tcp_grasp_pose_x", obj_x);
            // setOutput("tcp_grasp_pose_y", obj_y);

            RCLCPP_INFO(ros_->node->get_logger(),
                        "Grasp[arm=%s depth]: uv=(%.1f,%.1f) z_cam=%.3f -> "
                        "base=(%.3f,%.3f,%.3f) yaw=%.3f",
                        arm_prefix.c_str(), u, v, z_cam,
                        obj_x, obj_y, table_z + grasp_dz, best_yaw);
            // 8) 可视化
            if (ros_->grasp_viz_pub)
            {
                geometry_msgs::msg::PoseArray arr;
                arr.header = pregrasp.header;
                arr.poses.push_back(pregrasp.pose);
                arr.poses.push_back(grasp.pose);
                ros_->grasp_viz_pub->publish(arr);
            }

            return BT::NodeStatus::SUCCESS;
        } // use_depth fallback block
    } // tick()

    void registerGraspNodes(BT::BehaviorTreeFactory &factory, RosContextPtr ros)
    {
        factory.registerBuilder<GenerateGraspCandidate>(
            "GenerateGraspCandidate",
            [ros](const std::string &name, const BT::NodeConfig &config)
            {
                return std::make_unique<GenerateGraspCandidate>(name, config, ros);
            });

        factory.registerBuilder<GeneratePlaceCandidate>(
            "GeneratePlaceCandidate",
            [ros](const std::string &name, const BT::NodeConfig &config)
            { return std::make_unique<GeneratePlaceCandidate>(name, config, ros->node); });
    }

    // helper: rpy -> quaternion
    static void rpy_to_quat(double r, double p, double y, geometry_msgs::msg::Quaternion &q)
    {
        double cr = std::cos(r * 0.5), sr = std::sin(r * 0.5);
        double cp = std::cos(p * 0.5), sp = std::sin(p * 0.5);
        double cy = std::cos(y * 0.5), sy = std::sin(y * 0.5);
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
    }

    // ---- 改写: 按 arm_prefix 决定参数前缀 ----
    static geometry_msgs::msg::PoseStamped
    load_pose_from_params(rclcpp::Node::SharedPtr node,
                          const std::string &arm_prefix,
                          const std::string &pose_name)
    {
        // arm_prefix="" -> "place_*"; "left" -> "left_place_*"
        const std::string p = arm_prefix.empty()
                                  ? pose_name
                                  : arm_prefix + "_" + pose_name;
        const std::string default_frame = arm_prefix.empty()
                                              ? "base_link"
                                              : arm_prefix + "_base_link";

        // 幂等 declare (has_parameter check)
        if (!node->has_parameter(p + "_frame_id"))
            node->declare_parameter<std::string>(p + "_frame_id", default_frame);
        if (!node->has_parameter(p + "_position"))
            node->declare_parameter<std::vector<double>>(
                p + "_position", {0.0, 0.0, 0.0});
        if (!node->has_parameter(p + "_rpy"))
            node->declare_parameter<std::vector<double>>(
                p + "_rpy", {0.0, 0.0, 0.0});

        auto frame = node->get_parameter(p + "_frame_id").as_string();
        auto pos = node->get_parameter(p + "_position").as_double_array();
        auto rpy = node->get_parameter(p + "_rpy").as_double_array();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame;
        ps.pose.position.x = pos.size() > 0 ? pos[0] : 0.0;
        ps.pose.position.y = pos.size() > 1 ? pos[1] : 0.0;
        ps.pose.position.z = pos.size() > 2 ? pos[2] : 0.0;
        rpy_to_quat(rpy.size() > 0 ? rpy[0] : 0.0,
                    rpy.size() > 1 ? rpy[1] : 0.0,
                    rpy.size() > 2 ? rpy[2] : 0.0,
                    ps.pose.orientation);
        RCLCPP_INFO(node->get_logger(),
                    "loaded %s: frame=%s (%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
                    p.c_str(), frame.c_str(),
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                    rpy[0], rpy[1], rpy[2]);
        return ps;
    }

    GeneratePlaceCandidate::GeneratePlaceCandidate(
        const std::string &name, const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        // place_pose_ = load_pose_from_params(node_, "place");
        // pre_place_pose_ = load_pose_from_params(node_, "pre_place");
    }

    void GeneratePlaceCandidate::ensure_loaded(const std::string &arm_prefix)
    {
        if (place_cache_.count(arm_prefix) == 0)
        {
            place_cache_[arm_prefix] = load_pose_from_params(node_, arm_prefix, "place");
            pre_place_cache_[arm_prefix] = load_pose_from_params(node_, arm_prefix, "pre_place");
        }
    }

    BT::NodeStatus GeneratePlaceCandidate::tick()
    {
        std::string arm_prefix;
        getInput("arm_prefix", arm_prefix);
        ensure_loaded(arm_prefix);

        auto stamp = node_->now();
        auto place_pose = place_cache_[arm_prefix];
        auto pre_place_pose = pre_place_cache_[arm_prefix];
        place_pose.header.stamp = stamp;
        pre_place_pose.header.stamp = stamp;
        setOutput("place_pose", place_pose);
        setOutput("pre_place_pose", pre_place_pose);
        return BT::NodeStatus::SUCCESS;
    }
} // namespace s622_bt