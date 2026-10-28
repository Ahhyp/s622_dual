// C4.3 S4.1 W3-R1 — thin ROS adapter declaration.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__EXECUTOR_NODE_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__EXECUTOR_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "dual_arm_qp_controller/srv/abort_transport.hpp"
#include "dual_arm_qp_controller/srv/hold_transport.hpp"
#include "dual_arm_transport_executor/artifacts.hpp"
#include "dual_arm_transport_executor/attach_client.hpp"
#include "dual_arm_transport_executor/abort_requester.hpp"
#include "dual_arm_transport_executor/hold_requester.hpp"
#include "dual_arm_transport_executor/phase_driver.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"

namespace dual_arm_transport_executor
{

class TransportExecutorNode : public rclcpp::Node
{
public:
  explicit TransportExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double nowSeconds();
  void onStatus(dual_arm_qp_controller::msg::TransportStatus::SharedPtr m);
  void onTimer();
  void afterTransition();
  void writeArtifacts();
  // C4.3 S4.1 W3-R1a-2: executor-driven HOLD request (C7-A Rev.2).
  // C4.3 S4.1 W3-R2 B1: learn the controller phase table from TransportStatus.
  void learnPhaseTable(const dual_arm_qp_controller::msg::TransportStatus & m);
  void beginHoldRequest(const std::string & reason);
  void driveHoldRequest();
  // C4.3 S4 W7: pump the physical-attach client and move its requests onto the
  // bridged gz topics.  All judgement lives in AttachClient (unit tested).
  void driveWeld();
  void onWeldState(std_msgs::msg::String::SharedPtr m);
  void onPoses(tf2_msgs::msg::TFMessage::SharedPtr m);
  /// P1-4: propagate a task ABORT to the controller and wait for its
  /// authoritative confirmation before anything is reported as safe.
  void beginAbortPropagation(const std::string & reason);
  void driveAbortPropagation();


  std::unique_ptr<TransportStateMachine> sm_;
  ControllerStatus status_;
  State last_state_ = State::Idle;
  std::vector<CycleRow> cycles_;
  bool artifacts_written_ = false;
  std::string artifacts_dir_;
  std::string run_id_;
  double trajectory_duration_s_ = 5.1;
  double heartbeat_period_s_ = 0.02;

  rclcpp::Subscription<dual_arm_qp_controller::msg::TransportStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::Client<dual_arm_qp_controller::srv::HoldTransport>::SharedPtr hold_cli_;
  rclcpp::Client<dual_arm_qp_controller::srv::AbortTransport>::SharedPtr abort_cli_;

  // ---- C4.3 S4 W7: Gazebo DetachableJoint weld (W6) ----
  AttachClient weld_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr weld_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr weld_detach_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr weld_state_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr weld_pose_sub_;
  std::string weld_parent_frame_;
  std::string weld_child_frame_;
  bool have_parent_pose_ = false;
  bool have_child_pose_ = false;
  double parent_pos_[3] = {0.0, 0.0, 0.0};
  double child_pos_[3] = {0.0, 0.0, 0.0};
  bool startup_logged_ = false;
  bool weld_failure_logged_ = false;
  bool have_pre_rel_ = false;
  double pre_rel_[3] = {0.0, 0.0, 0.0};
  HoldRequester hold_req_;
  AbortRequester abort_req_;
  std::atomic<int> abort_reply_{0};   // 0 none / 1 accepted / 2 rejected
  bool abort_propagation_open_ = false;
  // W3-R2 B1: the phase table is owned by the controller; this node learns it
  // from the status stream and attaches the driver once the table is complete.
  PhaseDriver phase_driver_;
  int phase_table_count_ = 0;
  int phase_table_known_ = 0;
  bool phase_driver_attached_ = false;
  std::uint32_t next_hold_seq_ = 1;
  std::atomic<int> hold_reply_{0};   // 0 none / 1 accepted / 2 busy
  bool hold_request_open_ = false;
  bool hold_confirmed_ = false;

};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__EXECUTOR_NODE_HPP_
