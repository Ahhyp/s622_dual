// C4.3 S4.1 W3-R1 — pre-start check (layer B stub).
//
// Scope note (must stay honest): the real geometric pre-start check is W5+
// (finite-geometry table/support contact, FCL sweep).  What is implemented here
// is only what W3-R1 can actually verify from the controller status: the
// controller is alive, fresh, parsed, idle, and not already in a protection
// state.  Anything stronger must not be claimed.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__PRECHECK_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__PRECHECK_HPP_

#include <string>

#include "dual_arm_transport_executor/state_machine.hpp"

namespace dual_arm_transport_executor
{

struct PrecheckResult
{
  bool ok = true;
  std::string code;     // failure code to record; empty when ok
  std::string detail;   // human readable, goes into artifacts
};

// `now_s` is the injected clock; `st` the most recent controller status.
PrecheckResult checkStart(const Config & cfg, const ControllerStatus & st,
                          bool have_status, double now_s);

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__PRECHECK_HPP_
