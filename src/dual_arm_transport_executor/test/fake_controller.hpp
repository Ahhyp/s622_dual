// Deterministic stand-in for the QP controller, used only by tests.
// It obeys HOLD (freezes q_cmd and t_traj bitwise) and can be told to misbehave
// so that the executor's HOLD invariant audit can be exercised.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__TEST__FAKE_CONTROLLER_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__TEST__FAKE_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <string>

#include "dual_arm_transport_executor/state_machine.hpp"

namespace dual_arm_transport_executor
{
namespace test
{

class FakeController
{
public:
  void init(double now_s, double dt)
  {
    dt_ = dt;
    st_ = ControllerStatus();
    st_.received = true;
    st_.parse_ok = true;
    st_.stamp_s = now_s;
    st_.state = "IDLE";
    st_.failure_code = "ok";
    st_.t_traj = 0.0;
    st_.duration = 0.0;
    st_.committed = false;
    st_.transport_active = false;
    st_.q_state.fill(0.0);
    st_.q_cmd.fill(0.0);
    st_.dq.fill(0.0);
    holding_ = false;
  }

  void obeyHold() {holding_ = true;}
  void disobeyHold() {holding_ = false;}

  void beginPhase(double duration_s)
  {
    st_.transport_active = true;
    st_.duration = duration_s;
    st_.t_traj = 0.0;
    st_.committed = true;
    for (int i = 0; i < 12; ++i) {
      st_.q_cmd[static_cast<size_t>(i)] = 0.01 * static_cast<double>(i);
      st_.q_state[static_cast<size_t>(i)] = 0.01 * static_cast<double>(i);
    }
  }

  void endTransport()
  {
    st_.transport_active = false;
    st_.committed = false;
  }

  // One control period.  Call before sm.tick().
  void step(double now_s)
  {
    st_.stamp_s = now_s;
    if (holding_) {return;}  // HOLD: nothing may change
    for (int i = 0; i < 12; ++i) {
      st_.q_cmd[static_cast<size_t>(i)] =
        st_.q_cmd[static_cast<size_t>(i)] + 0.001 * static_cast<double>(i + 1);
      st_.q_state[static_cast<size_t>(i)] = st_.q_cmd[static_cast<size_t>(i)];
    }
    if (st_.transport_active && st_.duration > 0.0) {
      st_.t_traj = std::min(st_.t_traj + dt_, st_.duration);
      st_.committed = st_.t_traj < st_.duration;
    }
  }

  ControllerStatus & status() {return st_;}
  const ControllerStatus & status() const {return st_;}
  const ControllerStatus * ptr() const {return &st_;}

private:
  ControllerStatus st_;
  double dt_ = 0.008;
  bool holding_ = false;
};

}  // namespace test
}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__TEST__FAKE_CONTROLLER_HPP_
