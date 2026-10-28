// C4.3 S4.1 W3-R2: live phase integration probe (test fixture, NOT production).
//
// Why this file exists: the production adapter (src/executor_node.cpp) never
// constructs or attaches a PhaseDriver, so the phase fields it parses are dead
// (see the W3-R2 report, bug B1).  This probe therefore builds the same two
// pieces the executor is *supposed* to build -- the real TransportStateMachine
// and the real PhaseDriver from this package's core library -- and drives them
// with the status messages that actually cross DDS from the real controller.
// It proves what the core does with the wire data; the production node is
// checked separately by the Python harness on the same stream.
//
// It runs two state machines side by side on the SAME status stream:
//   * ext   : observes the controller phase index / name (observeExternal)
//   * local : the controller phase fields are masked out and t_traj is scaled
//             by --local-scale, i.e. a deliberately slow local clock.  This is
//             the "executor clock disagrees with the controller" case: if the
//             executor believed its own clock it would stay far behind.
//
// Output: JSONL, one line per state-machine event, flushed immediately, so the
// Python orchestrator can assert on it even if the probe is killed.
//
// Argv (positional, consumed before rclcpp::init):
//   1 out_path   2 phases ("approach:1.0,lift:1.2,...")   3 stale_s
//   4 local_scale   5 tag (used in the node name, so two probes can coexist)
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "dual_arm_transport_executor/phase_driver.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using dual_arm_transport_executor::Config;
using dual_arm_transport_executor::ControllerStatus;
using dual_arm_transport_executor::PhaseDriver;
using dual_arm_transport_executor::PhaseSpec;
using dual_arm_transport_executor::StartRequest;
using dual_arm_transport_executor::TransportStateMachine;
using dual_arm_transport_executor::toString;

std::string jsonEscape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 8);
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::vector<PhaseSpec> parsePhases(const std::string & csv)
{
  std::vector<PhaseSpec> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) {continue;}
    const std::size_t colon = item.find(':');
    PhaseSpec p;
    if (colon == std::string::npos) {
      p.name = item;
      p.duration_s = 0.0;
    } else {
      p.name = item.substr(0, colon);
      p.duration_s = std::stod(item.substr(colon + 1));
    }
    out.push_back(p);
  }
  return out;
}

class Probe : public rclcpp::Node
{
public:
  Probe(const std::string & tag, const std::string & out, std::vector<PhaseSpec> phases,
    double stale_s, double local_scale)
  : rclcpp::Node("w3r2_probe_" + tag),
    out_path_(out), phases_(std::move(phases)), local_scale_(local_scale)
  {
    Config cfg;
    // The probe isolates the phase logic from the production staleness
    // threshold (which cannot work against a 10 Hz publisher -- bug B2), so it
    // uses a threshold wider than the ~104 ms publish period.
    cfg.status_stale_timeout_s = stale_s;
    // The local-clock arm is deliberately slow; its APPROACH timeout must not
    // fire before the controller's 4-5 s trajectory is over.
    cfg.approach_timeout_s = 30.0;
    ext_ = std::make_unique<TransportStateMachine>(cfg);
    local_ = std::make_unique<TransportStateMachine>(cfg);
    ext_driver_.configure(phases_);
    local_driver_.configure(phases_);
    ext_->attachPhaseDriver(&ext_driver_);
    local_->attachPhaseDriver(&local_driver_);
    total_duration_s_ = 0.0;
    for (const auto & p : phases_) {total_duration_s_ += p.duration_s;}

    file_.open(out_path_, std::ios::out | std::ios::app);
    if (!file_) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", out_path_.c_str());
    }

    sub_ = create_subscription<dual_arm_qp_controller::msg::TransportStatus>(
      "/dual_arm_qp_controller/transport_status", rclcpp::QoS(10),
      [this](dual_arm_qp_controller::msg::TransportStatus::SharedPtr m) {
        onStatus(std::move(m));
      });
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        if (started_) {
          r->success = false;
          r->message = "already started";
          return;
        }
        // Arm the start and perform it from the next status callback: at that
        // instant the status age is ~0, so the start precheck (which the
        // production executor applies) cannot lose a race with the 10 Hz wire.
        want_start_ = true;
        r->success = true;
        r->message = "armed; starting on the next status message";
      });
    // The mock bench runs with require_executor_heartbeat:=true, so a live
    // executor must keep /dual_arm_qp_controller/heartbeat fed while a
    // transport is active.  The production adapter does this at 50 Hz; the
    // probe does the same so that the phase-publication run is driven by two
    // independent processes (a WSL2 scheduling hiccup in one of them cannot
    // trip the controller's E8 interlock and freeze the trajectory).
    hb_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/dual_arm_qp_controller/heartbeat", rclcpp::QoS(10));
    hb_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      hb_pub_->publish(std_msgs::msg::Empty());
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(8), [this]() {onTick();});

    std::ostringstream boot;
    boot << "{\"kind\":\"boot\",\"node\":\"" << get_name() << "\",\"stale_s\":" << stale_s
         << ",\"local_scale\":" << local_scale << ",\"total_duration_s\":"
         << total_duration_s_ << ",\"phases\":[";
    for (std::size_t i = 0; i < phases_.size(); ++i) {
      if (i != 0) {boot << ",";}
      boot << "{\"name\":\"" << jsonEscape(phases_[i].name) << "\",\"duration_s\":"
           << phases_[i].duration_s << "}";
    }
    boot << "]}";
    writeLine(boot.str());
  }

private:
  double nowSeconds()
  {
    return get_clock()->now().seconds();
  }

  void writeLine(const std::string & line)
  {
    if (!file_.is_open()) {return;}
    file_ << line << '\n';
    file_.flush();
  }

  void fill(ControllerStatus & dst, const dual_arm_qp_controller::msg::TransportStatus & m,
    double now_s, bool mask_phase, double t_scale)
  {
    dst.received = true;
    dst.parse_ok = true;
    dst.stamp_s = now_s;
    dst.state = m.state;
    dst.failure_code = m.failure_code;
    dst.transport_active = m.active;
    dst.committed = m.committed;
    dst.t_traj = m.t_traj * t_scale;
    dst.duration = m.t_traj_duration;
    for (int i = 0; i < 12; ++i) {
      dst.q_state[static_cast<std::size_t>(i)] = m.q_state[static_cast<std::size_t>(i)];
      dst.q_cmd[static_cast<std::size_t>(i)] = m.q_cmd[static_cast<std::size_t>(i)];
      dst.dq[static_cast<std::size_t>(i)] = m.dq[static_cast<std::size_t>(i)];
    }
    dst.solve_fail_streak = m.solve_fail_streak;
    dst.active_limit_count = m.active_limit_count;
    dst.e_obj_pos_m = m.e_obj_pos_m;
    dst.e_obj_rot_rad = m.e_obj_rot_rad;
    dst.sigma_min_stack = m.sigma_min_stack;
    dst.executor_heartbeat_age_s = m.executor_heartbeat_age_s;
    dst.executor_heartbeat_seen = m.executor_heartbeat_seen;
    dst.executor_heartbeat_required = m.executor_heartbeat_required;
    dst.hold_origin = m.hold_origin;
    dst.hold_request_seq = m.hold_request_seq;
    dst.hold_request_result = m.hold_request_result;
    if (mask_phase) {
      dst.phase_index = -1;
      dst.phase_count = 0;
      dst.phase_name.clear();
      dst.phase_elapsed_s = 0.0;
      dst.phase_duration_s = 0.0;
    } else {
      dst.phase_index = m.phase_index;
      dst.phase_count = m.phase_count;
      dst.phase_name = m.phase_name;
      dst.phase_elapsed_s = m.phase_elapsed_s;
      dst.phase_duration_s = m.phase_duration_s;
    }
  }

  void onStatus(dual_arm_qp_controller::msg::TransportStatus::SharedPtr m)
  {
    const double now = nowSeconds();
    raw_phase_index_ = m->phase_index;
    raw_phase_name_ = m->phase_name;
    raw_t_traj_ = m->t_traj;
    have_status_ = true;
    fill(ext_status_, *m, now, false, 1.0);
    fill(local_status_, *m, now, true, local_scale_);
    if (want_start_) {
      want_start_ = false;
      // The start precheck reads the last status the machine has *ticked* with,
      // so hand the fresh sample to both machines before calling start().  At
      // this instant the status age is ~0, exactly like the executor's own
      // start path on a live stream.
      ext_->tick(now, &ext_status_);
      local_->tick(now, &local_status_);
      doStart(now);
    }
  }

  void doStart(double now_s)
  {
    StartRequest req;
    req.duration_s = total_duration_s_;
    startResults("ext", *ext_, req, now_s);
    startResults("local", *local_, req, now_s);
    started_ = true;
  }

  void startResults(const std::string & tag, TransportStateMachine & sm,
    const StartRequest & req, double now_s)
  {
    std::string why;
    const bool ok = sm.start(now_s, req, &why);
    std::ostringstream os;
    os << "{\"kind\":\"" << (ok ? "start_ok" : "start_failed") << "\",\"sm\":\"" << tag
       << "\",\"wall\":" << now_s << ",\"why\":\"" << jsonEscape(why)
       << "\",\"state\":" << "\"" << toString(sm.state()) << "\"}";
    writeLine(os.str());
  }

  void onTick()
  {
    const double now = nowSeconds();
    // The production executor publishes ~/heartbeat at 50 Hz; the probe models
    // a live executor by feeding the same signal into both state machines.
    ext_->onHeartbeat(now);
    local_->onHeartbeat(now);
    ext_->tick(now, have_status_ ? &ext_status_ : nullptr);
    local_->tick(now, have_status_ ? &local_status_ : nullptr);
    emitNewEvents("ext", *ext_, ext_driver_, ext_status_, ext_emitted_);
    emitNewEvents("local", *local_, local_driver_, local_status_, local_emitted_);
  }

  void emitNewEvents(const std::string & tag, TransportStateMachine & sm,
    PhaseDriver & driver, const ControllerStatus & st, std::size_t & emitted)
  {
    const auto & events = sm.events();
    for (; emitted < events.size(); ++emitted) {
      const auto & e = events[emitted];
      std::ostringstream os;
      os << "{\"kind\":\"" << jsonEscape(e.kind) << "\",\"sm\":\"" << tag
         << "\",\"from\":\"" << jsonEscape(e.from) << "\",\"to\":\"" << jsonEscape(e.to)
         << "\",\"code\":\"" << jsonEscape(e.code) << "\",\"detail\":\""
         << jsonEscape(e.detail) << "\",\"state_now\":\"" << toString(sm.state())
         << "\",\"latched\":" << (sm.latched() ? "true" : "false")
         << ",\"failure_code\":\"" << jsonEscape(sm.failureCode()) << "\""
         << ",\"probe_phase_index\":" << driver.index()
         << ",\"ctrl_phase_index\":" << raw_phase_index_
         << ",\"ctrl_phase_name\":\"" << jsonEscape(raw_phase_name_) << "\""
         << ",\"ctrl_t_traj\":" << raw_t_traj_
         << ",\"used_t_traj\":" << st.t_traj
         << ",\"used_active\":" << (st.transport_active ? "true" : "false") << "}";
      writeLine(os.str());
    }
  }

  std::string out_path_;
  std::vector<PhaseSpec> phases_;
  double local_scale_ = 0.15;
  double total_duration_s_ = 0.0;
  std::ofstream file_;
  std::unique_ptr<TransportStateMachine> ext_;
  std::unique_ptr<TransportStateMachine> local_;
  PhaseDriver ext_driver_;
  PhaseDriver local_driver_;
  ControllerStatus ext_status_;
  ControllerStatus local_status_;
  std::size_t ext_emitted_ = 0;
  std::size_t local_emitted_ = 0;
  bool have_status_ = false;
  bool want_start_ = false;
  bool started_ = false;
  int raw_phase_index_ = -1;
  std::string raw_phase_name_;
  double raw_t_traj_ = 0.0;
  rclcpp::Subscription<dual_arm_qp_controller::msg::TransportStatus>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 6) {
    std::fprintf(stderr,
      "usage: live_phase_probe <out.jsonl> <name:dur,...> <stale_s> <local_scale> <tag>\n");
    return 2;
  }
  const std::string out = argv[1];
  const std::string phases_csv = argv[2];
  const double stale_s = std::stod(argv[3]);
  const double local_scale = std::stod(argv[4]);
  const std::string tag = argv[5];

  // Consume the positional arguments before rclcpp::init so it does not treat
  // them as ROS arguments.
  char * only_argv[] = {argv[0], nullptr};
  int only_argc = 1;
  rclcpp::init(only_argc, only_argv);
  auto node = std::make_shared<Probe>(tag, out, parsePhases(phases_csv), stale_s, local_scale);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
