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
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

  // ---- C4.3 S4 S4-T16: GRASP prerequisites (gripper + grasp check) ----------
  // The node owns the action/pose plumbing; the state machine owns the verdict
  // consumption (noteGripperClosed / noteGraspCheck).  The check itself is in
  // grasp_check.cpp (pure arithmetic).
  using GripperAction = control_msgs::action::FollowJointTrajectory;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;
  void driveGripper();
  void startGripper(double now_s);
  void resetGripperState();
  bool fingerPositionsAtTarget() const;
  /// Returns true when a verdict was sent to the state machine.  False means
  /// "pose not available yet"; the state machine's GRASP timeout is the bound.
  bool computeGraspCheck(double now_s);
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
  // S4-T16: the grasp check must use the sample pose AT GRASP ENTRY (before the
  // fingers move).  Closing the fingers can tilt a loosely supported sample; that
  // is a physical grasp issue, not the "sample was raised before GRASP" case the
  // check is for.
  bool grasp_snapshot_ok_ = false;
  double grasp_snapshot_at_s_ = -1.0;
  double grasp_parent_pos_[3] = {0.0, 0.0, 0.0};
  double grasp_child_pos_[3] = {0.0, 0.0, 0.0};
  double grasp_parent_quat_[4] = {0.0, 0.0, 0.0, 1.0};
  double grasp_child_quat_[4] = {0.0, 0.0, 0.0, 1.0};
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
  // §14.3 C3 also bounds the rotation jump, so the same snapshot keeps the
  // parent->child relative orientation (as a quaternion, x y z w).
  double parent_quat_[4] = {0.0, 0.0, 0.0, 1.0};
  double child_quat_[4] = {0.0, 0.0, 0.0, 1.0};
  double pre_rel_q_[4] = {0.0, 0.0, 0.0, 1.0};
  bool have_pre_rel_q_ = false;
  // S4-T16 gripper/grasp-check state (node side; the SM holds the verdicts).
  rclcpp_action::Client<GripperAction>::SharedPtr gripper_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  bool require_grasp_check_ = false;
  std::string gripper_action_;
  std::vector<std::string> gripper_joint_names_;
  std::vector<double> gripper_close_positions_;
  double gripper_motion_s_ = 1.0;
  double gripper_position_tol_ = 0.002;
  double gripper_timeout_s_ = 2.5;
  double grasp_pos_tol_m_ = 0.003;
  double grasp_rot_tol_rad_ = 0.017453292519943295;
  double grasp_snapshot_max_age_s_ = 5.0;
  double wrist3_to_grasp_z_ = 0.2168;
  bool gripper_started_ = false;
  bool gripper_failed_ = false;
  bool gripper_result_ready_ = false;
  bool gripper_result_ok_ = false;
  bool gripper_goals_sent_ = false;
  double gripper_start_at_s_ = -1.0;
  std::vector<double> gripper_positions_;
  std::vector<bool> gripper_position_seen_;
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
