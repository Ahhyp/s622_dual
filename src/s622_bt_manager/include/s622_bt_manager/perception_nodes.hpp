#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <yolov8_obb_msgs/msg/yolov8_inference.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace s622_bt
{

    struct RosContext
    {
        rclcpp::Node::SharedPtr node;
        rclcpp::CallbackGroup::SharedPtr perception_cb_group;

        // YOLO
        std::mutex yolo_mutex;
        yolov8_obb_msgs::msg::Yolov8Inference::SharedPtr latest_yolo;
        rclcpp::Time latest_yolo_recv_time;
        rclcpp::Subscription<yolov8_obb_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub;

        // Depth
        std::mutex depth_mutex;
        sensor_msgs::msg::Image::SharedPtr latest_depth;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;

        // CameraInfo
        std::mutex caminfo_mutex;
        sensor_msgs::msg::CameraInfo::SharedPtr latest_caminfo;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;

        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        // 可视化
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr grasp_viz_pub;

        void initYoloSubscription(const std::string &topic);
        void initDepthSubscription(const std::string &topic);
        void initCameraInfoSubscription(const std::string &topic);
        void initTf();
        void initGraspVizPublisher(const std::string &topic);
    };

    using RosContextPtr = std::shared_ptr<RosContext>;

    class DetectObject : public BT::SyncActionNode
    {
    public:
        DetectObject(const std::string &name,
                     const BT::NodeConfig &config,
                     RosContextPtr ros);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        RosContextPtr ros_;
    };
    class LockTargetPixel : public BT::SyncActionNode
    {
    public:
        LockTargetPixel(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<double>("input_u"),
                BT::InputPort<double>("input_v"),
                BT::OutputPort<double>("locked_u"),
                BT::OutputPort<double>("locked_v"),
            };
        }
        BT::NodeStatus tick() override;
    };

    class AlignLockCheck : public BT::SyncActionNode
    {
    public:
        AlignLockCheck(const std::string &name,
                       const BT::NodeConfig &config,
                       RosContextPtr ros)
            : BT::SyncActionNode(name, config), ros_(std::move(ros)) {}
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<double>("locked_u"),
                BT::InputPort<double>("locked_v"),
                BT::InputPort<std::string>("object_name", "cube", ""),
                BT::InputPort<double>("min_confidence", 0.05, ""),
                BT::InputPort<double>("tolerance_px", 12.0, ""),
            };
        }
        BT::NodeStatus tick() override;

    private:
        RosContextPtr ros_;
    };
    
    void registerPerceptionNodes(BT::BehaviorTreeFactory &factory, RosContextPtr ros);

} // namespace s622_bt