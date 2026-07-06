#include "s622_bt_manager/perception_nodes.hpp"

namespace s622_bt
{
    void RosContext::initYoloSubscription(const std::string &topic)
    {
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = perception_cb_group;

        yolo_sub = node->create_subscription<yolov8_obb_msgs::msg::Yolov8Inference>(
            topic, rclcpp::SensorDataQoS(),
            [this](yolov8_obb_msgs::msg::Yolov8Inference::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(yolo_mutex);
                latest_yolo = msg;
                latest_yolo_recv_time = node->now(); // 用 node 时钟 (收 use_sim_time 影响)
            },
            opts);
    }

    void RosContext::initDepthSubscription(const std::string &topic)
    {
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = perception_cb_group;
        depth_sub = node->create_subscription<sensor_msgs::msg::Image>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(depth_mutex);
                latest_depth = msg;
            },
            opts);
    }

    void RosContext::initCameraInfoSubscription(const std::string &topic)
    {
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = perception_cb_group;
        caminfo_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(caminfo_mutex);
                latest_caminfo = msg;
            },
            opts);
    }

    void RosContext::initTf()
    {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    void RosContext::initGraspVizPublisher(const std::string &topic)
    {
        grasp_viz_pub = node->create_publisher<geometry_msgs::msg::PoseArray>(topic, 10);
    }

    DetectObject::DetectObject(const std::string &name,
                               const BT::NodeConfig &config,
                               RosContextPtr ros)
        : BT::SyncActionNode(name, config), ros_(std::move(ros)) {}

    BT::PortsList DetectObject::providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_name", "cube", "target class name"),
            BT::InputPort<double>("min_confidence", 0.10, "minimum confidence"),
            BT::InputPort<double>("max_age_sec", 1.0, "max age of latest detection"),
            BT::OutputPort<double>("output_u"),
            BT::OutputPort<double>("output_v"),
            BT::OutputPort<double>("output_yaw"),
            BT::OutputPort<double>("output_confidence"),
        };
    }

    BT::NodeStatus DetectObject::tick()
    {
        std::string object_name = "cube";
        double min_conf = 0.05;
        double max_age = 1.0;
        getInput("object_name", object_name);
        getInput("min_confidence", min_conf);
        getInput("max_age_sec", max_age);

        yolov8_obb_msgs::msg::Yolov8Inference::SharedPtr latest;
        rclcpp::Time recv_time;
        {
            std::lock_guard<std::mutex> lock(ros_->yolo_mutex);
            latest = ros_->latest_yolo;
            recv_time = ros_->latest_yolo_recv_time;
        }

        if (!latest)
        {
            RCLCPP_WARN_THROTTLE(ros_->node->get_logger(),
                                 *ros_->node->get_clock(), 2000,
                                 "DetectObject: no YOLO message received yet");
            return BT::NodeStatus::FAILURE;
        }

        // 用本地接收时刻判断 age，避免 sim time / system time 混淆
        auto now = ros_->node->now();
        double age = (now - recv_time).seconds();
        if (age < 0 || age > max_age)
        {
            RCLCPP_WARN(ros_->node->get_logger(),
                        "DetectObject: latest YOLO recv was %.2fs ago (> %.2f)", age, max_age);
            return BT::NodeStatus::FAILURE;
        }
        // auto msg_time = rclcpp::Time(latest->header.stamp);
        // RCLCPP_WARN(ros_->node->get_logger(), "msg_time = %.2f", msg_time.seconds());
        // double age = (now - msg_time).seconds();
        if (age > max_age)
        {
            RCLCPP_WARN(ros_->node->get_logger(),
                        "DetectObject: latest YOLO is %.2fs old (> %.2f)", age, max_age);
            return BT::NodeStatus::FAILURE;
        }

        const yolov8_obb_msgs::msg::InferenceResult *best = nullptr;
        for (const auto &r : latest->results)
        {
            if (r.class_name != object_name)
                continue;
            if (r.confidence < min_conf)
                continue;
            if (!best || r.confidence > best->confidence)
                best = &r;
        }

        if (!best)
        {
            RCLCPP_WARN(ros_->node->get_logger(),
                        "DetectObject: no '%s' with conf>=%.2f in %zu results",
                        object_name.c_str(), min_conf, latest->results.size());
            return BT::NodeStatus::FAILURE;
        }

        setOutput("output_u", static_cast<double>(best->center_x));
        setOutput("output_v", static_cast<double>(best->center_y));
        setOutput("output_yaw", static_cast<double>(best->angle));
        setOutput("output_confidence", static_cast<double>(best->confidence));

        RCLCPP_INFO(ros_->node->get_logger(),
                    "DetectObject [%s]: uv=(%.1f,%.1f) yaw=%.3f conf=%.2f age=%.2fs",
                    object_name.c_str(),
                    best->center_x, best->center_y, best->angle, best->confidence, age);

        return BT::NodeStatus::SUCCESS;
    }

    void registerPerceptionNodes(BT::BehaviorTreeFactory &factory, RosContextPtr ros)
    {
        BT::NodeBuilder builder =
            [ros](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<DetectObject>(name, config, ros);
        };
        factory.registerBuilder<DetectObject>("DetectObject", builder);

        factory.registerNodeType<LockTargetPixel>("LockTargetPixel");
        factory.registerBuilder<AlignLockCheck>(
            "AlignLockCheck",
            [ros](const std::string &n, const BT::NodeConfig &c)
            {
                return std::make_unique<AlignLockCheck>(n, c, ros);
            });
    }

    BT::NodeStatus LockTargetPixel::tick()
    {
        double u, v;
        if (!getInput("input_u", u) || !getInput("input_v", v))
            return BT::NodeStatus::FAILURE;
        setOutput("locked_u", u);
        setOutput("locked_v", v);
        RCLCPP_INFO(rclcpp::get_logger("LockTargetPixel"),
                    "Locked target at (%.1f, %.1f)", u, v);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus AlignLockCheck::tick()
    {
        double lu, lv, tol = 12.0, min_conf = 0.05;
        std::string obj_name = "cube";
        getInput("locked_u", lu);
        getInput("locked_v", lv);
        getInput("tolerance_px", tol);
        getInput("min_confidence", min_conf);
        getInput("object_name", obj_name);

        yolov8_obb_msgs::msg::Yolov8Inference::SharedPtr latest;
        {
            std::lock_guard<std::mutex> lk(ros_->yolo_mutex);
            latest = ros_->latest_yolo;
        }
        if (!latest)
        {
            RCLCPP_WARN(ros_->node->get_logger(), "AlignLockCheck: no YOLO");
            return BT::NodeStatus::FAILURE;
        }
        const yolov8_obb_msgs::msg::InferenceResult *best = nullptr;
        for (const auto &r : latest->results)
        {
            if (r.class_name != obj_name)
                continue;
            if (r.confidence < min_conf)
                continue;
            if (!best || r.confidence > best->confidence)
                best = &r;
        }
        if (!best)
        {
            RCLCPP_WARN(ros_->node->get_logger(), "AlignLockCheck: no %s",
                        obj_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        double du = best->center_x - lu;
        double dv = best->center_y - lv;
        double err = std::hypot(du, dv);
        if (err > tol)
        {
            RCLCPP_WARN(ros_->node->get_logger(),
                        "AlignLockCheck: drift %.1f px > %.1f", err, tol);
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(ros_->node->get_logger(),
                    "AlignLockCheck: err=%.1f px (ok)", err);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace s622_bt