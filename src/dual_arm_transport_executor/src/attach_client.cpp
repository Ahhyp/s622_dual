// C4.3 S4.1 W6 -- see attach_client.hpp for the measured W6-0 facts this encodes.

#include "dual_arm_transport_executor/attach_client.hpp"

#include <cmath>
#include <utility>

namespace dual_arm_transport_executor
{

const char * toString(WeldState s)
{
  switch (s) {
    case WeldState::Attached: return "attached";
    case WeldState::Detached: return "detached";
    case WeldState::Unknown:
    default: return "unknown";
  }
}

const char * toString(WeldAction a)
{
  switch (a) {
    case WeldAction::Idle: return "idle";
    case WeldAction::Wait: return "wait";
    case WeldAction::RequestDetach: return "request_detach";
    case WeldAction::RequestAttach: return "request_attach";
    case WeldAction::Confirmed: return "confirmed";
    case WeldAction::Failed: return "failed";
    default: return "?";
  }
}

void AttachClient::configure(const AttachClientConfig & cfg)
{
  cfg_ = cfg;
  reset();
}

void AttachClient::reset()
{
  mode_ = Mode::Idle;
  state_ = WeldState::Unknown;
  state_at_s_ = -1.0;
  deadline_s_ = -1.0;
  pending_snap_m_ = std::nan("");
  attempts_ = 0;
  sent_ = false;
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  startup_verified_ = false;
  detach_required_ = false;
  action_ = WeldAction::Idle;
  outcome_.clear();
  reason_.clear();
}

bool AttachClient::stateKnown(double now_s) const
{
  if (state_ == WeldState::Unknown || state_at_s_ < 0.0) { return false; }
  return (now_s - state_at_s_) <= cfg_.state_stale_s;
}

bool AttachClient::snapWithinCriterion(double snap_m) const
{
  return std::isfinite(snap_m) && std::fabs(snap_m) <= cfg_.max_snap_m;
}

bool AttachClient::snapWithinCriterion(double snap_m, double snap_rad) const
{
  // §14.3 C3: pose jump across the attach is bounded in BOTH translation and
  // rotation.  A translation-only check would accept a weld that spun the sample.
  return snapWithinCriterion(snap_m) && std::isfinite(snap_rad) &&
         std::fabs(snap_rad) <= cfg_.max_snap_rad;
}

void AttachClient::beginStartupGuard(double now_s)
{
  // Deliberately unconditional: 6.16 welds at startup whether we asked or not, so
  // there is no "probably not welded" branch to take.
  mode_ = Mode::Startup;
  attempts_ = 0;
  sent_ = false;
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  startup_verified_ = false;
  detach_required_ = false;
  state_ = WeldState::Unknown;   // a pre-guard sample cannot prove the release
  state_at_s_ = -1.0;
  deadline_s_ = now_s + cfg_.startup_release_timeout_s;
  action_ = WeldAction::RequestDetach;
  outcome_ = "startup_release_pending";
  reason_ = "the plugin cannot start unwelded (W6-0); releasing the startup weld";
}

void AttachClient::requestAttach(double now_s)
{
  // A recorded failure is terminal: re-requesting must not overwrite the reason the
  // executor will report, and must not turn a snap violation into a fresh attempt.
  if (action_ == WeldAction::Failed) { return; }
  if (!startup_verified_) {
    fail("attach_refused_before_startup_release",
      "the startup weld has not been released yet; a weld at this point is not a grasp");
    return;
  }
  if (detach_required_) {
    fail("attach_refused_weld_still_on",
      "the previous attach exceeded the snap criterion and must be detached first");
    return;
  }
  // T2 fix (2026-10-29): /c43/weld_state is EVENT-DRIVEN -- the plugin publishes
  // only when the weld changes (W6-0).  Ageing a "detached" sample out after
  // 0.25 s therefore refused every attach that was not issued within 250 ms of
  // the startup release.  What actually proves the state is current is
  // startupVerified() (a confirmed release) plus the absence of any later
  // transition; liveness is checked by transition_timeout_s while waiting for
  // "attached".  Freshness is still required to CONFIRM a transition below.
  if (state_ != WeldState::Detached) {
    fail("attach_refused_weld_state_unknown",
      std::string("state=") + toString(state_));
    return;
  }
  mode_ = Mode::Attaching;
  attempts_ = 0;      // counted when the request is armed, see tick()
  sent_ = false;
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  deadline_s_ = now_s + cfg_.transition_timeout_s;
  pending_snap_m_ = std::nan("");
  action_ = WeldAction::RequestAttach;
  outcome_ = "attach_pending";
  reason_.clear();
}

void AttachClient::requestDetach(double now_s)
{
  mode_ = Mode::Detaching;
  sent_ = false;
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  deadline_s_ = now_s + cfg_.transition_timeout_s;
  action_ = WeldAction::RequestDetach;
  outcome_ = "detach_pending";
  reason_.clear();
}

void AttachClient::observe(WeldState s, double now_s)
{
  state_ = s;
  state_at_s_ = now_s;
}

void AttachClient::fail(const std::string & code, const std::string & why)
{
  mode_ = Mode::Idle;
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  action_ = WeldAction::Failed;
  outcome_ = code;
  reason_ = why;
}

WeldAction AttachClient::tick(double now_s)
{
  // A failure is terminal until reset(): the executor must not keep going on a
  // guess, and it must not silently retry a snap violation.
  if (action_ == WeldAction::Failed) { return action_; }

  switch (mode_) {
    case Mode::Startup: {
        if (state_ == WeldState::Detached && stateKnown(now_s)) {
          startup_verified_ = true;
          mode_ = Mode::Idle;
          action_ = WeldAction::Confirmed;
          outcome_ = "startup_released";
          reason_ = "detach confirmed; the startup weld no longer exists";
          return action_;
        }
        if (now_s > deadline_s_) {
          if (!stateKnown(now_s)) {
            fail("weld_state_unknown",
              "no fresh weld-state sample within the startup release window; "
              "cannot prove the startup weld is gone");
          } else {
            fail("weld_not_released",
              std::string("state stayed ") + toString(state_) +
              " for the whole startup release window");
          }
          return action_;
        }
        // A one-shot publish can be dropped before the plugin subscribes (W6-0
        // measured a 2.17 s attach delay).  Keep the Request pending on a bounded
        // cadence while no state sample has arrived; the deadline above still fails
        // the guard, and consumeSend() bounds the attempt count.
        if (!armed_ &&
          (last_send_s_ < 0.0 || (now_s - last_send_s_) >= cfg_.startup_retry_s))
        {
          armed_ = true;
          armed_action_ = WeldAction::RequestDetach;
          sent_ = false;   // the previous publish is stale; re-arm the same request
          attempts_ += 1;
          return action_ = WeldAction::RequestDetach;
        }
        return action_ = WeldAction::Wait;
      }

    case Mode::Attaching: {
        if (state_ == WeldState::Attached && stateKnown(now_s)) {
          if (!std::isfinite(pending_snap_m_)) {
            detach_required_ = true;
            fail("attach_snap_unmeasured",
              "the weld was confirmed but no snap measurement was reported; "
              "'not measured' is not 'no snap'");
            return action_;
          }
          if (snapWithinCriterion(pending_snap_m_, pending_snap_rad_)) {
            mode_ = Mode::Idle;
            action_ = WeldAction::Confirmed;
            outcome_ = "attached";
            reason_ = "weld confirmed with snap within the frozen criterion";
            return action_;
          }
          detach_required_ = true;
          fail("attach_snap_exceeded",
            "attached but the measured snap exceeds the frozen 2 mm criterion; "
            "the executor must detach and re-approach instead of continuing "
            "(translation <= 2 mm and rotation <= 0.5 deg, §14.3 C3)");
          return action_;
        }
        if (now_s > deadline_s_) {
          if (attempts_ < cfg_.max_attach_attempts) {
            sent_ = false;
            armed_ = false;
            armed_action_ = WeldAction::Idle;
            deadline_s_ = now_s + cfg_.transition_timeout_s;
            return action_ = WeldAction::Wait;   // next tick re-issues the attach
          }
          fail("attach_not_confirmed",
            std::string("state=") + toString(state_) + " after " +
            std::to_string(attempts_) + " attach(es); an unconfirmed attach is a failure");
          return action_;
        }
        if (!armed_ && !sent_) {
          armed_ = true;
          armed_action_ = WeldAction::RequestAttach;
          attempts_ += 1;
          return action_ = WeldAction::RequestAttach;
        }
        return action_ = WeldAction::Wait;
      }

    case Mode::Detaching: {
        if (state_ == WeldState::Detached && stateKnown(now_s)) {
          mode_ = Mode::Idle;
          action_ = WeldAction::Confirmed;
          outcome_ = "detached";
          reason_ = "detach confirmed";
          return action_;
        }
        if (now_s > deadline_s_) {
          if (!stateKnown(now_s)) {
            fail("weld_state_unknown", "no fresh weld-state sample while detaching");
          } else {
            fail("detach_not_confirmed",
              std::string("state stayed ") + toString(state_) + " while detaching");
          }
          return action_;
        }
        if (!armed_ && !sent_) {
          armed_ = true;
          armed_action_ = WeldAction::RequestDetach;
          attempts_ += 1;
          return action_ = WeldAction::RequestDetach;
        }
        return action_ = WeldAction::Wait;
      }

    case Mode::Idle:
    default:
        if (action_ == WeldAction::Confirmed) { return action_; }
        return action_ = WeldAction::Idle;
  }
}

WeldAction AttachClient::consumeSend(double now_s)
{
  if (sent_ || !armed_) { return WeldAction::Wait; }
  // Use the dedicated slot: action_ may already have been overwritten by the
  // state machine's tickWeld() between arming and this call (T2 fix).
  const WeldAction a = armed_action_;
  if (a != WeldAction::RequestAttach && a != WeldAction::RequestDetach) {
    return WeldAction::Wait;
  }
  armed_ = false;
  armed_action_ = WeldAction::Idle;
  sent_ = true;
  last_send_s_ = now_s;
  return a;
}

}  // namespace dual_arm_transport_executor
