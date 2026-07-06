#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include "s622_bt_manager/dummy_nodes.hpp"
#include "s622_bt_manager/perception_nodes.hpp"
#include "s622_bt_manager/grasp_nodes.hpp"
#include "s622_bt_manager/bt_executor.hpp"
#include "s622_bt_manager/motion_nodes.hpp"
#include "s622_bt_manager/gripper_nodes.hpp"
#include "s622_bt_manager/servo_nodes.hpp"
#include "s622_bt_manager/scene_nodes.hpp"

using namespace std::chrono_literals;

class BTExecutor : public rclcpp::Node
{
public:
    BTExecutor() : rclcpp::Node("bt_executor")
    {
        declare_parameter<std::string>("tree_file", "dummy_tree.xml");
        declare_parameter<std::string>("tree_id", "DummyTree");
        declare_parameter<int>("tick_rate_hz", 10);
        declare_parameter<bool>("auto_start", false);
        declare_parameter<int>("groot2_port", 1667);
        declare_parameter<std::string>("yolo_topic", "/yolov8/obb_detections");
        declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
        declare_parameter<std::string>("caminfo_topic", "/camera/color/camera_info");
        declare_parameter<std::string>("grasp_viz_topic", "/grasp_visualization");

        tick_rate_hz_ = get_parameter("tick_rate_hz").as_int();

        trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/bt_trigger", 10,
            std::bind(&BTExecutor::onTrigger, this, std::placeholders::_1));
    }

    ~BTExecutor()
    {
        stop_request_ = true;
        if (bt_thread_.joinable())
            bt_thread_.join();
    }

    void init()
    {
        ros_ctx_ = std::make_shared<s622_bt::RosContext>();
        ros_ctx_->node = shared_from_this();
        ros_ctx_->perception_cb_group =
            create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        ros_ctx_->initYoloSubscription(get_parameter("yolo_topic").as_string());
        ros_ctx_->initDepthSubscription(get_parameter("depth_topic").as_string());
        ros_ctx_->initCameraInfoSubscription(get_parameter("caminfo_topic").as_string());
        ros_ctx_->initTf();
        ros_ctx_->initGraspVizPublisher(get_parameter("grasp_viz_topic").as_string());

        s622_bt::registerDummyNodes(factory_);
        s622_bt::registerPerceptionNodes(factory_, ros_ctx_);
        s622_bt::registerGraspNodes(factory_, ros_ctx_);
        s622_bt::registerMotionNodes(factory_, shared_from_this());
        s622_bt::registerGripperNodes(factory_, shared_from_this());
        s622_bt::registerServoNodes(factory_, shared_from_this());
        s622_bt::registerSceneNodes(factory_, shared_from_this());
        
        auto pkg_share = ament_index_cpp::get_package_share_directory("s622_bt_manager");
        auto tree_path = pkg_share + "/behavior_trees/" + get_parameter("tree_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading tree: %s", tree_path.c_str());
        factory_.registerBehaviorTreeFromFile(tree_path);
        
        if (get_parameter("auto_start").as_bool())
        {
            startTreeAsync();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Waiting for trigger on /bt_trigger ...");
        }
        
    }

private:
    void onTrigger(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;
        if (running_.load())
        {
            RCLCPP_WARN(get_logger(), "Tree already running, ignoring trigger");
            return;
        }
        startTreeAsync();
    }

    void startTreeAsync()
    {
        if (bt_thread_.joinable())
            bt_thread_.join();
        running_ = true;
        bt_thread_ = std::thread(&BTExecutor::runTree, this);
    }

    void runTree()
    {
        auto tree_id = get_parameter("tree_id").as_string();
        auto groot_port = get_parameter("groot2_port").as_int();

        RCLCPP_INFO(get_logger(), "=== Starting BT [%s] ===", tree_id.c_str());
        auto tree = factory_.createTree(tree_id);
        BT::Groot2Publisher groot_pub(tree, groot_port);
        BT::StdCoutLogger cout_logger(tree);
        BT::TreeObserver observer(tree);

        auto t_start = now();
        auto status = BT::NodeStatus::RUNNING;
        rclcpp::Rate rate(tick_rate_hz_);
        while (rclcpp::ok() && !stop_request_.load() && status == BT::NodeStatus::RUNNING)
        {
            status = tree.tickOnce();
            rate.sleep();
        }

        if (status == BT::NodeStatus::FAILURE) {
            std::string failed_node = "unknown";
            for (const auto& [uid, path] : observer.uidToPath()) {
                const auto& stats = observer.getStatistics(uid);
                if (stats.last_result == BT::NodeStatus::FAILURE) {
                    failed_node = path;
                }
            }
            tree.rootBlackboard()->set("last_failure_node", failed_node);
            RCLCPP_ERROR(get_logger(),
                         "BT FAILURE at node: %s", failed_node.c_str());
        }

        double duration = (now() - t_start).seconds();
        RCLCPP_INFO(get_logger(), "=== BT finished: %s ===", BT::toStr(status).c_str());
        RCLCPP_INFO(get_logger(), "[TRIAL_RESULT] status=%s duration=%.1f",
                    BT::toStr(status).c_str(), duration);
        running_ = false;
    }

    BT::BehaviorTreeFactory factory_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    s622_bt::RosContextPtr ros_ctx_;
    std::thread bt_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_request_{false};
    int tick_rate_hz_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}