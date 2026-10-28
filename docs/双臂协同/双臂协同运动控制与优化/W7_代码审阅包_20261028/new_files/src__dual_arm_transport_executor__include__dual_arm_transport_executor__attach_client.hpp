// C4.3 S4.1 W6 -- executor-side client for the Gazebo DetachableJoint weld.
//
// WHY THIS EXISTS  (measured, see C4.3_S4_W6-0_DetachableJoint实测报告_20261028.md)
//   Fortress 6.16's DetachableJoint plugin, as built, CANNOT be configured to
//   start unwelded: the `initial_attach` element is ignored (0 byte occurrences
//   in the plugin and in every libignition-gazebo6 library; adding it changes
//   nothing).  Every run therefore starts with a physical constraint between the
//   gripper and the sample.  In 4 measured runs that startup weld was created
//   2.165-2.201 s after the server came up, was a no-op twice, and twice
//   teleported the sample 0.475 m back to its SDF spawn pose.
//
//   Consequences encoded below:
//     * a startup weld is NEVER evidence of a grasp -- it must be released
//       (`startupGuard`) and the release must be confirmed before the executor
//       may move or claim anything;
//     * "attached" is only accepted when the state message is fresh AND the
//       measured snap is within the frozen 2 mm criterion; a bigger snap is a
//       failure of the attach, not a success with a caveat;
//     * an attach that is never confirmed is a failure, not a silent no-op.
//
// The class is pure logic: no ROS, no ignition-transport, no clock of its own.
// The caller passes `now_s`, the observed weld state, and the measured snap, so
// every rule is unit-testable without a simulator, and the timeouts are evidence
// instead of narration.  The runtime wiring (ROS topics bridged to the gz topics
// by ros_gz_bridge) belongs to W7; the topic names below are what that bridge
// will use.

#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_CLIENT_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_CLIENT_HPP_

#include <string>

namespace dual_arm_transport_executor
{

/// What the plugin last reported on its output topic (`ignition.msgs.StringMsg`,
/// payload "attached" / "detached", bridged to std_msgs/String).
enum class WeldState
{
  Unknown = 0,
  Attached = 1,
  Detached = 2,
};

const char * toString(WeldState s);

struct AttachClientConfig
{
    /// Topics used by the plugin (W6-0 evidence: the plugin logs exactly these
    /// three names; attach/detach are subscriber-only, so they must be configured
    /// here rather than discovered).
    std::string attach_topic = "/c43/attach";
    std::string detach_topic = "/c43/detach";
    std::string state_topic = "/c43/weld_state";

    /// A weld-state sample older than this is not evidence of anything.
    double state_stale_s = 0.25;
    /// How long a requested attach/detach may stay unconfirmed before retry/fail.
    double transition_timeout_s = 0.5;
    /// Sends before an attach is declared failed (a retry re-issues the same attach,
    /// which the plugin treats as "Already attached" once the first one landed).
    int max_attach_attempts = 3;
    /// Startup release window.  The startup weld appears ~2.2 s after the server
    /// starts (W6-0), so this must cover server start-up, not just one tick.
    double startup_release_timeout_s = 5.0;
    /// Frozen criterion, C4.3_S4_执行规划.md 14.3 C3: attach must not translate
    /// the sample by more than 2 mm / 0.5 deg within 100 ms.  This client only
    /// checks translation; rotation is the caller's measurement.
    double max_snap_m = 0.002;
};

/// What the caller must do on the transport, this cycle.
enum class WeldAction
{
  Idle = 0,
  Wait = 1,
  RequestDetach = 2,
  RequestAttach = 3,
  Confirmed = 4,
  Failed = 5,
};

const char * toString(WeldAction a);

class AttachClient
{
public:
  AttachClient() = default;
  explicit AttachClient(const AttachClientConfig & cfg) : cfg_(cfg) { reset(); }

  void configure(const AttachClientConfig & cfg);
  void reset();

  /// Step 1 of W6: the weld that exists at startup must be released and the
  /// release confirmed before the executor may advance.  Nothing else is allowed
  /// until this passes.
  void beginStartupGuard(double now_s);
  bool startupGuardActive() const { return mode_ == Mode::Startup; }
  bool startupVerified() const { return startup_verified_; }

  /// Requests an attach.  Refused (Failed) unless the startup guard passed and the
  /// weld is currently known-detached: welding an already-welded sample would hide
  /// the very failure this client exists to report.
  void requestAttach(double now_s);
  void requestDetach(double now_s);

  /// The sample's translation change across the attach, measured by the caller
  /// (pose topic) within 100 ms of the attach request.  Must be reported before the
  /// tick that observes `attached`, otherwise the client judges a 0 m snap, which
  /// would turn "we did not measure" into "we measured no snap".
  void setMeasuredSnap(double snap_m) { pending_snap_m_ = snap_m; }

  /// A fresh state sample arrived from the plugin.
  void observe(WeldState s, double now_s);

  /// Called every control cycle with the current time; returns the transport action.
  WeldAction tick(double now_s);

  WeldState state() const { return state_; }
  /// What the client is currently doing (Idle/Wait/Request*/Confirmed/Failed).
  /// Callers need this to know whether an attach is still in flight; without it
  /// they would re-send the same request every control cycle.
  WeldAction action() const { return action_; }
  bool stateKnown(double now_s) const;
  int attempts() const { return attempts_; }
  /// Human-readable classification of the finished (or in-progress) operation.
  const std::string & outcome() const { return outcome_; }
  const std::string & reason() const { return reason_; }
  bool done() const { return mode_ == Mode::Idle && action_ == WeldAction::Confirmed; }
  bool failed() const { return action_ == WeldAction::Failed; }

  /// Snap check used both when an attach is confirmed and (for evidence) by tests.
  bool snapWithinCriterion(double snap_m) const;

private:
  enum class Mode
  {
    Idle = 0,
    Startup = 1,
    Attaching = 2,
    Detaching = 3,
  };

  void fail(const std::string & code, const std::string & why);

  AttachClientConfig cfg_{};
  Mode mode_ = Mode::Idle;
  WeldState state_ = WeldState::Unknown;
  double state_at_s_ = -1.0;
  double deadline_s_ = -1.0;
  double pending_snap_m_ = 0.0;
  int attempts_ = 0;
  bool sent_ = false;
  bool startup_verified_ = false;
  bool detach_required_ = false;   ///< after a snap violation, no retry until detach
  WeldAction action_ = WeldAction::Idle;
  std::string outcome_;
  std::string reason_;
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__ATTACH_CLIENT_HPP_
