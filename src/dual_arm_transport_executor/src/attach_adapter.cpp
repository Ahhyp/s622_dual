#include "dual_arm_transport_executor/attach_adapter.hpp"

namespace dual_arm_transport_executor
{
namespace
{
// Static literals: assigning one of these performs no allocation and no copy.
constexpr const char * kTimeoutCode = "attach_timeout";
constexpr const char * kSimulatedTag = "simulated";
constexpr const char * kNoneTag = "none";
constexpr double kDefaultTimeoutS = 0.5;
}  // namespace

void AttachAdapter::configure(double request_timeout_s)
{
  // A non-positive (or NaN) timeout would fail every request on the first tick;
  // keep the documented default instead of silently disabling the wait.
  request_timeout_s_ =
    (request_timeout_s > 0.0) ? request_timeout_s : kDefaultTimeoutS;
}

void AttachAdapter::reset()
{
  state_ = AttachState::Unattached;
  requested_at_s_ = 0.0;
  failure_literal_ = nullptr;
  injected_failure_.clear();
}

AttachResult AttachAdapter::request(double now_s)
{
  switch (state_) {
    case AttachState::Requested:
      return {false, "attach_busy"};
    case AttachState::AttachedSimulated:
      return {false, "already_attached"};
    case AttachState::Unattached:
    case AttachState::Failed:
      break;
  }
  // Unattached or Failed: a retry is accepted and clears the previous code.
  state_ = AttachState::Requested;
  requested_at_s_ = now_s;
  failure_literal_ = nullptr;
  injected_failure_.clear();
  return {true, ""};
}

void AttachAdapter::tick(double now_s)
{
  if (state_ != AttachState::Requested) {
    return;
  }
  if (now_s - requested_at_s_ > request_timeout_s_) {
    // Static literal only: no string is constructed on this path.
    failure_literal_ = kTimeoutCode;
    state_ = AttachState::Failed;
    return;
  }
  // SIMULATED completion: no physical attachment is performed or claimed.
  state_ = AttachState::AttachedSimulated;
}

void AttachAdapter::injectFailure(const std::string & code)
{
  if (state_ != AttachState::Requested) {
    return;
  }
  injected_failure_ = code;  // owned copy, so the caller's buffer may die
  failure_literal_ = nullptr;
  state_ = AttachState::Failed;
}

const std::string & AttachAdapter::failureCode() const
{
  if (failure_literal_ != nullptr) {
    return timeout_code_;
  }
  return injected_failure_;  // empty unless a failure was injected
}

std::string AttachAdapter::artifactTag() const
{
  return state_ == AttachState::AttachedSimulated ? kSimulatedTag : kNoneTag;
}

}  // namespace dual_arm_transport_executor
