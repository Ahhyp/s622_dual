#include <atomic>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
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
#include "s622_bt_manager/handover_nodes.hpp"

using namespace std::chrono_literals;

class BTExecutor : public rclcpp::Node
{
public:
    explicit BTExecutor(const rclcpp::NodeOptions &options)
        : rclcpp::Node("bt_executor", options)
    {
        // auto-declare(from overrides) 打开后，YAML/CLI 给过的参数已经 declared，
        // 再 declare 会抛 ParameterAlreadyDeclaredException → 统一走这个 helper。
        auto declare_if_absent = [this](const auto &name, const auto &def)
        { if (!this->has_parameter(name)) this->declare_parameter(name, def); };
        declare_if_absent("tree_file", "dummy_tree.xml");
        declare_if_absent("tree_id", "DummyTree");
        declare_if_absent("tick_rate_hz", 10);
        declare_if_absent("auto_start", false);
        declare_if_absent("groot2_port", 1667);
        declare_if_absent("yolo_topic", "/yolov8/obb_detections");
        declare_if_absent("depth_topic", "/camera/depth/image_raw");
        declare_if_absent("caminfo_topic", "/camera/color/camera_info");
        declare_if_absent("grasp_viz_topic", "/grasp_visualization");
        declare_if_absent("arm", "");
        // [M4 BT 回填] 把 "<prefix>.*" 参数灌进 blackboard，让 YAML（config/m4_handover.yaml）
        // 成为交接几何/运动参数的唯一来源，XML 里用 {var} 引用。
        declare_if_absent("blackboard_prefix", "bb");
        // 逗号分隔的额外 XML（只提供 <SubTree> 定义，供主树 <SubTree ID=.../> 引用）
        declare_if_absent("subtree_files", "handover_rod.xml");

        tick_rate_hz_ = get_parameter("tick_rate_hz").as_int();

        trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/bt_trigger", 10,
            std::bind(&BTExecutor::onTrigger, this, std::placeholders::_1));
        // [M4 BT 回填] trial 结果话题（统计 harness / 自动化 runner 用）
        rclcpp::QoS res_qos(1);
        res_qos.transient_local().reliable();
        trial_result_pub_ = create_publisher<std_msgs::msg::String>("/bt_trial_result", res_qos);
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

        s622_bt::registerDummyNodes(factory_, shared_from_this());
        s622_bt::registerPerceptionNodes(factory_, ros_ctx_);
        s622_bt::registerGraspNodes(factory_, ros_ctx_);
        s622_bt::registerMotionNodes(factory_, shared_from_this());
        s622_bt::registerGripperNodes(factory_, shared_from_this());
        s622_bt::registerServoNodes(factory_, shared_from_this());
        s622_bt::registerSceneNodes(factory_, ros_ctx_);
        s622_bt::registerHandoverNodes(factory_, ros_ctx_, shared_from_this());

        auto pkg_share = ament_index_cpp::get_package_share_directory("s622_bt_manager");
        // 先注册子树库（逗号分隔），再注册主树文件
        {
            std::stringstream ss(get_parameter("subtree_files").as_string());
            std::string item;
            while (std::getline(ss, item, ','))
            {
                if (item.empty())
                    continue;
                const auto p = pkg_share + "/behavior_trees/" + item;
                RCLCPP_INFO(get_logger(), "Loading subtree file: %s", p.c_str());
                try
                {
                    factory_.registerBehaviorTreeFromFile(p);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "subtree file %s 跳过: %s", item.c_str(), e.what());
                }
            }
        }
        auto tree_path = pkg_share + "/behavior_trees/" + get_parameter("tree_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading tree: %s", tree_path.c_str());
        factory_.registerBehaviorTreeFromFile(tree_path);

        const auto arm = get_parameter("arm").as_string();
        if (arm.empty())
        {
            RCLCPP_INFO(get_logger(), "arm parameter: '' (single-arm compat mode)");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "arm parameter: '%s'", arm.c_str());
        }

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

    // [M4 BT 回填] YAML → blackboard：<prefix>.foo_bar → 变量 foo_bar。
    //
    // 两个必须注意的点（实测）：
    //  1) 依赖 NodeOptions.automatically_declare_parameters_from_overrides(true)，
    //     否则 YAML 里的参数不会出现在 list_parameters 里；
    //  2) 值一律存成 std::string —— BT.CPP 在解析 XML 时会把 "{var}" 引用的 blackboard
    //     条目预声明成 string，这里如果再 set(double) 会报 "type shall not change"。
    //     存成字符串后，端口按自己的类型解析（vector<double> 用 ';' 分隔，已单独验证）。
    static void loadBlackboardFromParams(rclcpp::Node *node, const std::string &prefix,
                                         BT::Blackboard::Ptr bb)
    {
        if (prefix.empty() || !bb)
            return;
        const std::string dotted = prefix + ".";
        const auto names = node->list_parameters({prefix}, 0).names;
        auto num = [](double x)
        {
            std::ostringstream os;
            os << std::setprecision(12) << x;
            return os.str();
        };
        auto join = [&](const auto &v)
        {
            std::string s;
            for (size_t i = 0; i < v.size(); ++i)
                s += (i ? ";" : "") + num(static_cast<double>(v[i]));
            return s;
        };
        int loaded = 0;
        for (const auto &name : names)
        {
            if (name.rfind(dotted, 0) != 0 || !node->has_parameter(name))
                continue;
            const std::string key = name.substr(dotted.size());
            const auto p = node->get_parameter(name);
            std::string text;
            try
            {
                switch (p.get_type())
                {
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    text = p.as_bool() ? "true" : "false";
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    text = std::to_string(p.as_int());
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    text = num(p.as_double());
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING:
                    text = p.as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
                    text = join(p.as_double_array());
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
                    text = join(p.as_integer_array());
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
                {
                    const auto v = p.as_string_array();
                    for (size_t i = 0; i < v.size(); ++i)
                        text += (i ? ";" : "") + v[i];
                    break;
                }
                case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
                {
                    const auto v = p.as_bool_array();
                    for (size_t i = 0; i < v.size(); ++i)
                        text += (i ? ";" : "") + std::string(v[i] ? "true" : "false");
                    break;
                }
                default:
                    RCLCPP_WARN(node->get_logger(), "blackboard: 跳过不支持的类型 %s",
                                name.c_str());
                    continue;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node->get_logger(), "blackboard['%s'] <- %s 失败: %s",
                            key.c_str(), name.c_str(), e.what());
                continue;
            }
            try
            {
                bb->set(key, text);
                ++loaded;
                RCLCPP_DEBUG(node->get_logger(), "blackboard['%s'] = '%s'", key.c_str(),
                             text.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node->get_logger(), "blackboard['%s'] 设置失败: %s", key.c_str(),
                            e.what());
            }
        }
        RCLCPP_INFO(node->get_logger(), "blackboard: 从 '%s.*' 载入 %d 个参数", prefix.c_str(),
                    loaded);
    }

    void runTree()
    {
        auto tree_id = get_parameter("tree_id").as_string();
        auto groot_port = get_parameter("groot2_port").as_int();
        auto arm = get_parameter("arm").as_string();

        RCLCPP_INFO(get_logger(), "=== Starting BT [%s] ===", tree_id.c_str());
        auto tree = factory_.createTree(tree_id);

        tree.rootBlackboard()->set("arm", arm);
        loadBlackboardFromParams(this, get_parameter("blackboard_prefix").as_string(),
                                 tree.rootBlackboard());

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

        if (status == BT::NodeStatus::FAILURE)
        {
            std::string failed_node = "unknown";
            for (const auto &[uid, path] : observer.uidToPath())
            {
                const auto &stats = observer.getStatistics(uid);
                if (stats.last_result == BT::NodeStatus::FAILURE)
                {
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
        {
            std_msgs::msg::String msg;
            std::string failed_node_bb;
            tree.rootBlackboard()->get("last_failure_node", failed_node_bb);
            std::ostringstream os;
            os << "status=" << BT::toStr(status) << " tree=" << tree_id
               << " duration=" << duration << " failure_node=" << failed_node_bb;
            msg.data = os.str();
            trial_result_pub_->publish(msg);
        }
        running_ = false;
    }

    BT::BehaviorTreeFactory factory_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trial_result_pub_;
    s622_bt::RosContextPtr ros_ctx_;
    std::thread bt_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_request_{false};
    int tick_rate_hz_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // [M4 BT 回填] YAML 里的参数默认不会自动 declare（list_parameters 看不到），
    // 打开这个选项后才能把 config/m4_handover.yaml 的 bb.* 灌进 blackboard。
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<BTExecutor>(options);
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}