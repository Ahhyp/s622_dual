#pragma once
#include <array>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "dual_arm_transport_executor/msg/contact_command.hpp"
#include "dual_arm_transport_executor/msg/contact_evidence.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "dual_arm_transport_executor/hold_requester.hpp"
#include "dual_arm_transport_executor/abort_requester.hpp"
#include "dual_arm_transport_executor/artifacts.hpp"
#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "dual_arm_qp_controller/srv/start_transport.hpp"
#include "dual_arm_qp_controller/srv/hold_transport.hpp"
#include "dual_arm_qp_controller/srv/abort_transport.hpp"

namespace dual_arm_transport_executor {
// Separate ROS plant adapter, same production core. The legacy weld adapter
// remains the default path of c43_s4_executor.
class ContactExecutorAdapter {
public:
  explicit ContactExecutorAdapter(rclcpp::Node & node);
private:
  double now() const {return n_.get_clock()->now().seconds();}
  bool start(std::string * why);
  void tick();
  void evidence(const msg::ContactEvidence & m);
  void status(const dual_arm_qp_controller::msg::TransportStatus & m);
  void command(const std::string & phase);
  void requestStart();
  void pumpHold();
  bool requestOperatorHold(std::string * why);
  bool requestOperatorResume(std::string * why);
  bool resumeGeometryReady(std::string * why) const;
  void pumpOperatorHold();
  void finish();
  rclcpp::Node & n_;
  Config cfg_;
  std::unique_ptr<TransportStateMachine> sm_;
  PhaseDriver phases_;
  ControllerStatus status_;
  ContactObservation observation_;
  HoldRequester hold_;
  HoldRequester operator_hold_;
  AbortRequester abort_;
  ArtifactWriter writer_;
  std::vector<CycleRow> cycles_;
  std::ofstream protocol_;
  std::string run_id_;
  std::uint64_t task_id_=0,command_id_=0,last_evidence_seq_=0,consumed_command_=0;
  std::string command_phase_;
  bool auto_start_=false,auto_started_=false,qp_start_sent_=false,finished_=false,stop_sent_=false;
  bool hold_service_inflight_=false;
  bool abort_service_inflight_=false,stop_ack_=false;
  double stop_since_=-1, last_abort_poll_=-1, command_since_=-1;
  bool inject_retreat_abort_=false;
  bool hold_resume_enabled_=false,operator_hold_active_=false,operator_hold_confirmed_=false;
  bool operator_hold_inflight_=false,resume_pending_=false;
  std::uint32_t next_hold_seq_=1;
  double operator_hold_since_=0,operator_hold_confirmed_since_=0,resume_since_=0;
  std::array<double,3> sample_position_{},held_sample_position_{};
  std::array<double,4> sample_orientation_{},held_sample_orientation_{};
  double seat_overtravel_=0.0002;
  rclcpp::Publisher<msg::ContactCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<msg::ContactEvidence>::SharedPtr evidence_sub_;
  rclcpp::Subscription<dual_arm_qp_controller::msg::TransportStatus>::SharedPtr status_sub_;
  rclcpp::Client<dual_arm_qp_controller::srv::StartTransport>::SharedPtr start_cli_;
  rclcpp::Client<dual_arm_qp_controller::srv::HoldTransport>::SharedPtr hold_cli_;
  rclcpp::Client<dual_arm_qp_controller::srv::AbortTransport>::SharedPtr abort_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_cli_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_,abort_srv_,reset_srv_,hold_srv_,resume_srv_;
  rclcpp::TimerBase::SharedPtr timer_,heartbeat_timer_;
};
}
