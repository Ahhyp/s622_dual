#include "dual_arm_transport_executor/contact_executor_adapter.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

namespace dual_arm_transport_executor {
ContactExecutorAdapter::ContactExecutorAdapter(rclcpp::Node & n):n_(n) {
  cfg_.physical_contact=true;cfg_.require_grasp_check=true;
  cfg_.approach_timeout_s=n_.declare_parameter("approach_timeout_s",45.0);
  cfg_.grasp_timeout_s=n_.declare_parameter("grasp_timeout_s",40.0);
  cfg_.release_timeout_s=n_.declare_parameter("release_timeout_s",12.0);
  cfg_.retreat_timeout_s=n_.declare_parameter("retreat_timeout_s",60.0);
  cfg_.limit_min_saturated_joints=6;cfg_.limit_margin_rad=.05;
  cfg_.contact_stale_timeout_s=n_.declare_parameter("contact_stale_timeout_s",.1);
  hold_resume_enabled_=n_.declare_parameter("enable_physical_hold_resume",false);
  auto_start_=n_.declare_parameter("auto_start",false);
  inject_retreat_abort_=n_.declare_parameter("inject_retreat_abort",false);
  seat_overtravel_=n_.declare_parameter("seat_overtravel_m",.0002);
  if (!(seat_overtravel_>0 && seat_overtravel_<=.0005)) throw std::invalid_argument("invalid seating bound");
  sm_=std::make_unique<TransportStateMachine>(cfg_);
  phases_.configure({{"lift",6},{"settle",5},{"transport",8},{"lower",6},{"settle_post",2}});
  sm_->attachPhaseDriver(&phases_);
  run_id_=n_.declare_parameter<std::string>("run_id","contact_transport");
  const auto dir=n_.declare_parameter<std::string>("artifacts_dir","/tmp/c43_contact_executor");
  std::string err;if(!writer_.open(dir,&err))throw std::runtime_error(err);
  protocol_.open(dir+"/contact_protocol.jsonl");
  Manifest manifest;manifest.run_id=run_id_;manifest.mode="physical_contact_sim";
  manifest.env_class="container";manifest.git_head=C43_GIT_HEAD;manifest.dry_run=false;
  writer_.writeManifest(manifest,&err);
  command_pub_=n_.create_publisher<msg::ContactCommand>("~/contact_command",10);
  state_pub_=n_.create_publisher<std_msgs::msg::String>("~/task_state",rclcpp::QoS(1).transient_local());
  result_pub_=n_.create_publisher<std_msgs::msg::String>("~/task_result",rclcpp::QoS(1).transient_local());
  heartbeat_pub_=n_.create_publisher<std_msgs::msg::Empty>("/dual_arm_qp_controller/heartbeat",10);
  evidence_sub_=n_.create_subscription<msg::ContactEvidence>("~/contact_evidence",1,
    [this](msg::ContactEvidence::SharedPtr m){evidence(*m);});
  status_sub_=n_.create_subscription<dual_arm_qp_controller::msg::TransportStatus>(
    "/dual_arm_qp_controller/transport_status",1,[this](dual_arm_qp_controller::msg::TransportStatus::SharedPtr m){status(*m);});
  start_cli_=n_.create_client<dual_arm_qp_controller::srv::StartTransport>("/dual_arm_qp_controller/start_transport");
  hold_cli_=n_.create_client<dual_arm_qp_controller::srv::HoldTransport>("/dual_arm_qp_controller/hold_transport");
  abort_cli_=n_.create_client<dual_arm_qp_controller::srv::AbortTransport>("/dual_arm_qp_controller/abort_transport");
  resume_cli_=n_.create_client<std_srvs::srv::Trigger>("/dual_arm_qp_controller/resume_transport");
  hold_srv_=n_.create_service<std_srvs::srv::Trigger>("~/hold",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){r->success=requestOperatorHold(&r->message);});
  resume_srv_=n_.create_service<std_srvs::srv::Trigger>("~/resume",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){r->success=requestOperatorResume(&r->message);});
  start_srv_=n_.create_service<std_srvs::srv::Trigger>("~/start",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){r->success=start(&r->message);});
  abort_srv_=n_.create_service<std_srvs::srv::Trigger>("~/abort",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){r->success=sm_->abortTransport("abort_requested",now(),&r->message);});
  reset_srv_=n_.create_service<std_srvs::srv::Trigger>("~/reset",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){r->success=false;r->message="physical profile recovery not validated; restart the isolated experiment after inspecting the latched result";});
  heartbeat_timer_=n_.create_wall_timer(std::chrono::milliseconds(20),[this](){heartbeat_pub_->publish(std_msgs::msg::Empty());sm_->onHeartbeat(now());});
  timer_=n_.create_wall_timer(std::chrono::milliseconds(8),[this](){tick();});
  RCLCPP_INFO(n_.get_logger(),"physical_contact adapter ready; shared C++ state machine, no weld");
}

bool ContactExecutorAdapter::start(std::string * why) {
  if(task_id_!=0){if(why)*why="one task per physical experiment; restart for another run";return false;}
  if(!sm_->start(now(),{27},why))return false;
  task_id_=static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  command_id_=0;consumed_command_=0;finished_=false;qp_start_sent_=false;stop_sent_=false;
  command("APPROACH");return true;
}

void ContactExecutorAdapter::command(const std::string & phase) {
  command_phase_=phase;command_since_=now();++command_id_;msg::ContactCommand m;
  m.task_id=task_id_;m.command_id=command_id_;m.phase=phase;command_pub_->publish(m);
  protocol_<<"{\"event\":\"command\",\"sim\":"<<now()<<",\"task_id\":"<<task_id_<<",\"command_id\":"<<command_id_<<",\"phase\":\""<<phase<<"\"}\n";protocol_.flush();
}

void ContactExecutorAdapter::evidence(const msg::ContactEvidence & m) {
  if(finished_)return;
  if(m.evidence_seq<=last_evidence_seq_)return;
  if((task_id_==0 && m.task_id!=0)||(task_id_!=0 && m.task_id!=task_id_))return;
  if(task_id_!=0 && (m.command_id!=command_id_ || m.command_phase!=command_phase_))return;
  for(double x:m.sample_position)if(!std::isfinite(x))return;
  for(double x:m.sample_orientation)if(!std::isfinite(x))return;
  sample_position_=m.sample_position;sample_orientation_=m.sample_orientation;
  last_evidence_seq_=m.evidence_seq;
  protocol_<<"{\"event\":\"evidence\",\"sim\":"<<now()
    <<",\"stamp\":"<<rclcpp::Time(m.stamp).seconds()
    <<",\"contact_stamp\":"<<rclcpp::Time(m.contact_stamp).seconds()
    <<",\"evidence_seq\":"<<m.evidence_seq<<",\"command_id\":"<<m.command_id<<"}\n";

  observation_.stamp_s=rclcpp::Time(m.stamp).seconds();observation_.contact_stamp_s=rclcpp::Time(m.contact_stamp).seconds();
  observation_.command_phase=m.command_phase;observation_.ready=m.ready;observation_.step_done=m.step_done;
  observation_.all_four=m.all_four;observation_.source_support=m.source_support;
  observation_.destination_support=m.destination_support;observation_.robot_clear=m.robot_clear;
  observation_.arms_jtc_active=m.arms_jtc_active;observation_.qp_active=m.qp_active;
  observation_.hands_jtc_active=m.hands_jtc_active;observation_.fingers_open=m.fingers_open;
  const double d2=m.sample_position[0]*m.sample_position[0]+std::pow(m.sample_position[1]-.12,2)+std::pow(m.sample_position[2]-.275,2);
  observation_.in_target=d2<=.01*.01;sm_->noteContactObservation(observation_);
  if(task_id_!=0 && !stop_sent_ && (sm_->state()!=State::Fault && sm_->state()!=State::Abort) && now()-observation_.stamp_s>=-.01 && now()-observation_.stamp_s<=cfg_.contact_stale_timeout_s &&
     now()-observation_.contact_stamp_s>=-.01 && now()-observation_.contact_stamp_s<=cfg_.contact_stale_timeout_s &&
     (m.safety_failure_code=="object_env_contact"||m.safety_failure_code=="follower_divergence")){
    protocol_<<"{\"event\":\"physical_safety_abort\",\"sim\":"<<now()<<",\"code\":\""<<m.safety_failure_code<<"\"}\n";protocol_.flush();
    sm_->abortTransport(m.safety_failure_code,now(),nullptr);return;
  }
  if(task_id_==0 || !m.step_done || consumed_command_==command_id_)return;
  consumed_command_=command_id_;
  if(command_phase_=="STOP"){stop_ack_=m.ok;return;}
  protocol_<<"{\"event\":\"step_result\",\"sim\":"<<now()<<",\"command_id\":"<<command_id_<<",\"ok\":"<<(m.ok?"true":"false")<<"}\n";protocol_.flush();
  std::string why;
  if(!m.ok){sm_->escalateFault(sm_->state()==State::Release?"release_failed":"grasp_check_failed",now(),m.detail.c_str(),&why);return;}
  if(sm_->state()==State::Grasp){sm_->noteGripperClosed(now(),m.all_four,"measured four-finger contact");sm_->noteGraspCheck(now(),m.qp_active&&!m.arms_jtc_active&&!m.hands_jtc_active,"verified command resource ownership");}
  if(!sm_->phaseDone(now(),&why))sm_->escalateFault(sm_->state()==State::Release?"release_failed":"grasp_check_failed",now(),why.c_str(),nullptr);
}

void ContactExecutorAdapter::status(const dual_arm_qp_controller::msg::TransportStatus & m) {
  status_.received=true;status_.stamp_s=now();status_.state=m.state;status_.failure_code=m.failure_code;
  status_.transport_active=m.active;status_.committed=m.committed;status_.t_traj=m.t_traj;status_.duration=m.t_traj_duration;
  status_.q_state=m.q_state;status_.q_cmd=m.q_cmd;status_.dq=m.dq;
  status_.solve_fail_streak=m.solve_fail_streak;status_.active_limit_count=m.active_limit_count;
  status_.min_limit_margin_rad=m.min_limit_margin_rad;status_.e_obj_pos_m=m.e_obj_pos_m;status_.e_obj_rot_rad=m.e_obj_rot_rad;
  status_.hold_origin=m.hold_origin;status_.hold_request_seq=m.hold_request_seq;status_.hold_request_result=m.hold_request_result;
  status_.phase_index=m.phase_index;status_.phase_count=m.phase_count;status_.phase_name=m.phase_name;
  status_.phase_duration_s=m.phase_duration_s;status_.phase_elapsed_s=m.phase_elapsed_s;
  status_.executor_heartbeat_required=m.executor_heartbeat_required;status_.executor_heartbeat_seen=m.executor_heartbeat_seen;
}

void ContactExecutorAdapter::requestStart() {
  if(!start_cli_->service_is_ready())return;
  qp_start_sent_=true;auto r=std::make_shared<dual_arm_qp_controller::srv::StartTransport::Request>();
  r->phase_name={"lift","settle","transport","lower","settle_post"};r->phase_duration={6.,5.,8.,6.,2.};
  r->phase_dx={0,0,0,0,0};r->phase_dy={0,0,.12,0,0};r->phase_dz={.06,0,0,-.06,-seat_overtravel_};r->dt=.008;r->time_scaling="smoothstep";
  start_cli_->async_send_request(r,[this](rclcpp::Client<dual_arm_qp_controller::srv::StartTransport>::SharedFuture f){auto r=f.get();if(!r->accepted)sm_->escalateFault("trajectory_invalid",now(),r->message.c_str(),nullptr);});
}

void ContactExecutorAdapter::pumpHold() {
  if(!hold_.started())return;
  HoldObservation o;o.now_s=now();o.have_status=status_.received;o.status_age_s=now()-status_.stamp_s;
  o.state=status_.state;o.hold_origin=status_.hold_origin;o.ack_seq=status_.hold_request_seq;o.ack_result=status_.hold_request_result;
  const auto step=hold_.onObservation(o);
  if(step.action==HoldAction::Abort){sm_->abortTransport("release_failed",now(),nullptr);return;}
  if(hold_.needSend() && !hold_service_inflight_ && hold_cli_->service_is_ready()){
    auto r=std::make_shared<dual_arm_qp_controller::srv::HoldTransport::Request>();r->seq=hold_.seq();r->reason="physical destination support confirmed";
    hold_.markSent(now());hold_service_inflight_=true;
    hold_cli_->async_send_request(r,[this](rclcpp::Client<dual_arm_qp_controller::srv::HoldTransport>::SharedFuture f){auto r=f.get();hold_service_inflight_=false;hold_.onServiceReply(r->accepted,r->message.c_str());});
  }
}

bool ContactExecutorAdapter::resumeGeometryReady(std::string * why) const {
  auto fail=[why](const char * reason){if(why)*why=reason;return false;};
  const double age=now()-observation_.stamp_s,ca=now()-observation_.contact_stamp_s;
  if(!std::isfinite(age)||age<0||age>cfg_.contact_stale_timeout_s||!std::isfinite(ca)||ca<0||ca>cfg_.contact_stale_timeout_s||
     !observation_.all_four||!observation_.qp_active||observation_.arms_jtc_active||observation_.hands_jtc_active)
    return fail("fresh four contacts and exclusive QP ownership required");
  const double sa=now()-status_.stamp_s;
  if(!status_.received||!std::isfinite(sa)||sa<0||sa>cfg_.status_stale_timeout_s||
     !std::isfinite(status_.e_obj_pos_m)||!std::isfinite(status_.e_obj_rot_rad)||
     status_.e_obj_pos_m>cfg_.resume_pos_tol_m||status_.e_obj_rot_rad>cfg_.resume_rot_tol_rad||
     !std::isfinite(status_.min_limit_margin_rad)||status_.min_limit_margin_rad<cfg_.limit_margin_rad||
     status_.solve_fail_streak>=cfg_.solve_fail_streak_limit)return fail("controller resume bounds invalid");
  if(operator_hold_confirmed_){
    double d=0,dot=0,a=0,b=0;
    for(int i=0;i<3;++i)d+=std::pow(sample_position_[i]-held_sample_position_[i],2);
    for(int i=0;i<4;++i){dot+=sample_orientation_[i]*held_sample_orientation_[i];a+=sample_orientation_[i]*sample_orientation_[i];b+=held_sample_orientation_[i]*held_sample_orientation_[i];}
    if(a<.9||a>1.1||b<.9||b>1.1||std::sqrt(d)>cfg_.resume_pos_tol_m||
       2*std::acos(std::clamp(std::abs(dot)/std::sqrt(a*b),0.,1.))>cfg_.resume_rot_tol_rad)
      return fail("measured payload drift exceeds resume bounds");
  }
  return true;
}

bool ContactExecutorAdapter::requestOperatorHold(std::string * why) {
  if(!hold_resume_enabled_||operator_hold_active_||finished_||
     (sm_->state()!=State::Lift&&sm_->state()!=State::Transport)||status_.state!="RUNNING"){
    if(why){*why="explicit hold unavailable in current state";}return false;}
  if(!resumeGeometryReady(why))return false;
  operator_hold_active_=true;operator_hold_confirmed_=false;operator_hold_since_=now();
  operator_hold_.begin(now(),next_hold_seq_++,"explicit operator hold");sm_->expectHoldAck(operator_hold_.seq());
  protocol_<<"{\"event\":\"operator_hold_requested\",\"sim\":"<<now()<<",\"seq\":"<<operator_hold_.seq()<<"}\n";protocol_.flush();
  if(why){*why="hold queued; wait for applied controller acknowledgement";}return true;
}

bool ContactExecutorAdapter::requestOperatorResume(std::string * why) {
  if(!hold_resume_enabled_||!operator_hold_active_||!operator_hold_confirmed_||resume_pending_||sm_->state()!=State::Hold||
     now()-operator_hold_confirmed_since_<.1||status_.state!="HOLD"||status_.hold_origin!="executor"||
     status_.hold_request_seq!=operator_hold_.seq()||status_.hold_request_result!=1||!resume_cli_->service_is_ready()){
    if(why){*why="confirmed explicit hold required";}return false;}
  if(!resumeGeometryReady(why))return false;
  resume_pending_=true;resume_since_=now();
  protocol_<<"{\"event\":\"operator_resume_requested\",\"sim\":"<<now()<<"}\n";protocol_.flush();
  resume_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){auto r=f.get();if(!r->success)sm_->escalateFault("resume_failed",now(),r->message.c_str(),nullptr);});
  if(why){*why="resume queued; RUNNING acknowledgement still required";}return true;
}

void ContactExecutorAdapter::pumpOperatorHold() {
  if(!operator_hold_active_)return;
  std::string why;
  if(!resumeGeometryReady(&why)){sm_->escalateFault("resume_failed",now(),why.c_str(),nullptr);return;}
  if(resume_pending_){
    if(status_.state=="RUNNING"){
      if(!sm_->resumeAfterControllerAck(now(),status_,&why)){sm_->escalateFault("resume_failed",now(),why.c_str(),nullptr);return;}
      protocol_<<"{\"event\":\"operator_resume_applied\",\"sim\":"<<now()<<",\"seq\":"<<operator_hold_.seq()<<"}\n";protocol_.flush();
      operator_hold_active_=false;operator_hold_confirmed_=false;resume_pending_=false;return;
    }
    if(now()-resume_since_>1.)sm_->escalateFault("resume_failed",now(),"resume acknowledgement timeout",nullptr);
    return;
  }
  HoldObservation o;o.now_s=now();o.have_status=status_.received;o.status_age_s=now()-status_.stamp_s;
  o.state=status_.state;o.hold_origin=status_.hold_origin;o.ack_seq=status_.hold_request_seq;o.ack_result=status_.hold_request_result;
  const auto step=operator_hold_.onObservation(o);
  if(step.action==HoldAction::Abort){sm_->escalateFault("resume_failed",now(),"operator hold unconfirmed",nullptr);return;}
  if(status_.state=="HOLD"&&!operator_hold_confirmed_){
    if(status_.hold_origin!="executor"||status_.hold_request_seq!=operator_hold_.seq()||status_.hold_request_result!=1){sm_->escalateFault("resume_failed",now(),"hold ownership mismatch",nullptr);return;}
    operator_hold_confirmed_=true;operator_hold_confirmed_since_=now();held_sample_position_=sample_position_;held_sample_orientation_=sample_orientation_;
    protocol_<<"{\"event\":\"operator_hold_applied\",\"sim\":"<<now()<<",\"seq\":"<<operator_hold_.seq()<<"}\n";protocol_.flush();
  }
  if(operator_hold_.needSend()&&!operator_hold_inflight_&&hold_cli_->service_is_ready()){
    auto r=std::make_shared<dual_arm_qp_controller::srv::HoldTransport::Request>();r->seq=operator_hold_.seq();r->reason="explicit operator hold";
    operator_hold_.markSent(now());operator_hold_inflight_=true;
    hold_cli_->async_send_request(r,[this](rclcpp::Client<dual_arm_qp_controller::srv::HoldTransport>::SharedFuture f){auto r=f.get();operator_hold_inflight_=false;operator_hold_.onServiceReply(r->accepted,r->message.c_str());});
  }
}

void ContactExecutorAdapter::tick() {
  if(finished_)return;
  if(auto_start_&&!auto_started_&&observation_.ready&&now()>0&&command_pub_->get_subscription_count()>0){
    // Wait for fresh evidence; a failed precheck is not retried automatically.
    if(now()-observation_.stamp_s<=cfg_.contact_stale_timeout_s){auto_started_=true;std::string why;start(&why);}
  }
  if(sm_->state()==State::Lower && status_.phase_name=="settle_post" && status_.phase_elapsed_s>=1.0 &&
    observation_.destination_support&&observation_.in_target&&!hold_.started()){
    hold_.begin(now(),next_hold_seq_++,"destination seating");sm_->expectSupportHoldAck(hold_.seq());
  }
  if(inject_retreat_abort_ && sm_->state()==State::Retreat &&
    command_phase_=="RETREAT" && now()-command_since_>=1.0)
    sm_->abortTransport("abort_requested",now(),nullptr);
  pumpOperatorHold();pumpHold();sm_->tick(now(),status_.received?&status_:nullptr);
  if(sm_->state()==State::Hold&&!operator_hold_active_)sm_->abortTransport(sm_->failureCode(),now(),nullptr);
  const State state=sm_->state();std_msgs::msg::String text;text.data=toString(state);state_pub_->publish(text);
  if(state==State::Grasp&&command_phase_!="GRASP")command("GRASP");
  if(state==State::Lift&&!qp_start_sent_)requestStart();
  if(state==State::Release&&command_phase_!="RELEASE")command("RELEASE");
  if(state==State::Retreat&&command_phase_!="RETREAT")command("RETREAT");
  CycleRow row;row.k=cycles_.size();row.t=now();row.state=toString(state);row.failure_code=sm_->failureCode();row.committed=status_.committed;row.t_traj=status_.t_traj;row.q_cmd=status_.q_cmd;cycles_.push_back(row);
  if(isLatched(state)&&!(state==State::Hold&&operator_hold_active_)){
    if(!stop_sent_){stop_sent_=true;stop_since_=now();command("STOP");
      if(qp_start_sent_ && observation_.qp_active)abort_.begin(now(),sm_->failureCode().c_str());}
    if(abort_.started() && (last_abort_poll_<0 || now()-last_abort_poll_>=.05)){
      last_abort_poll_=now();
      AbortObservation o;o.now_s=now();o.have_status=status_.received;o.status_age_s=now()-status_.stamp_s;o.state=status_.state;
      const auto a=abort_.onObservation(o,false);
      if(a.action==AbortAction::Escalate)sm_->escalateFault(a.reason,now(),"controller stop unconfirmed",nullptr);
      if(abort_.needSend()&&!abort_service_inflight_&&abort_cli_->service_is_ready()){
        auto r=std::make_shared<dual_arm_qp_controller::srv::AbortTransport::Request>();r->reason=sm_->failureCode();
        abort_.markSent(now());abort_service_inflight_=true;
        abort_cli_->async_send_request(r,[this](rclcpp::Client<dual_arm_qp_controller::srv::AbortTransport>::SharedFuture f){auto r=f.get();abort_service_inflight_=false;abort_.onServiceReply(r->accepted,r->message.c_str());});
      }
    }
    if(stop_ack_&&(!abort_.started()||abort_.confirmed()||abort_.escalated()))finish();
    else if(now()-stop_since_>3){sm_->escalateFault("abort_not_confirmed",now(),"plant STOP acknowledgement missing",nullptr);finish();}
  } else if(state==State::Complete)finish();
}

void ContactExecutorAdapter::finish() {
  protocol_<<"{\"event\":\"terminal_stop\",\"sim\":"<<now()
    <<",\"plant_ack\":"<<(stop_ack_?"true":"false")
    <<",\"controller_abort_started\":"<<(abort_.started()?"true":"false")
    <<",\"controller_stop_confirmed\":"<<(abort_.confirmed()?"true":"false")
    <<",\"controller_stop_outcome\":\""<<abort_.outcome()
    <<"\",\"controller_state\":\""<<status_.state<<"\"}\n";
  // A stop-confirmation failure may escalate after this tick's cycle row.
  // Keep the artifact's final_state consistent with the published result.
  if(cycles_.empty() || cycles_.back().state!=toString(sm_->state())){
    CycleRow row;row.k=cycles_.size();row.t=now();row.state=toString(sm_->state());
    row.failure_code=sm_->failureCode();row.committed=status_.committed;
    row.t_traj=status_.t_traj;row.q_cmd=status_.q_cmd;cycles_.push_back(row);
  }
  std::string err;writer_.writeEvents(sm_->events(),&err);writer_.writeCycles(cycles_,&err);
  writer_.writeSummary(buildSummaryJson(run_id_,toString(sm_->state()),sm_->events(),cycles_,
    {"physical_contact_sim; same production core; full W7 fault/safety acceptance not claimed"},false,{},"physical_contact"),&err);
  finished_=true;protocol_.flush();std_msgs::msg::String result;result.data=toString(sm_->state());result_pub_->publish(result);
  RCLCPP_INFO(n_.get_logger(),"physical task ended: %s / %s",toString(sm_->state()),sm_->failureCode().c_str());
}
}
