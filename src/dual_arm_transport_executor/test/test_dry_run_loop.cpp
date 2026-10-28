// C4.3 S4 W3-R6 -- SOFTWARE DRY-RUN closed loop.
//
// This file is evidence for the "no physical device, no ROS controller" claim:
// it drives the real TransportStateMachine, the real PhaseDriver, the real
// CollisionGuard, the real AttachAdapter, the real HoldRequester and the real
// artifact writer against test/fake_controller.hpp only.  There is no rclcpp
// node, no publisher, no service client, no Gazebo and no actuator write.
//
// What it proves is deliberately narrow and is named as such in the test names:
//   * the happy path visits every executor state once, in contract order, with
//     the four R2 phases entered in order;
//   * a collision-guard verdict becomes a HOLD carrying the guard's own code,
//     produced by the guard the state machine owns (setCollisionGuard) -- not by
//     writing the code into ControllerStatus.failure_code;
//   * the HoldRequester round-trip is recorded with its reason;
//   * an injected attach failure becomes a FAULT, never COMPLETE;
//   * a frozen trajectory clock raises phase_timeout;
//   * summary.json carries mode=SOFTWARE_ONLY, dry_run=true, the ordered phase
//     list and the final state.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_transport_executor/artifacts.hpp"
#include "dual_arm_transport_executor/attach_adapter.hpp"
#include "dual_arm_transport_executor/collision_guard.hpp"
#include "dual_arm_transport_executor/hold_requester.hpp"
#include "dual_arm_transport_executor/phase_driver.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"
#include "fake_controller.hpp"

namespace due = dual_arm_transport_executor;
namespace fs = std::filesystem;

using due::AttachAdapter;
using due::AttachState;
using due::CollisionGuard;
using due::Config;
using due::CycleRow;
using due::HoldAction;
using due::HoldObservation;
using due::HoldRequester;
using due::PhaseDriver;
using due::PhaseSpec;
using due::StartRequest;
using due::State;
using due::TransportStateMachine;
using due::test::FakeController;

namespace
{

constexpr double kDt = 0.008;

std::vector<PhaseSpec> fourPhaseTrajectory()
{
  return {
    {"approach", 0.30},
    {"lift", 1.50},
    {"transport", 1.50},
    {"lower", 1.50},
  };
}

Config dryRunConfig()
{
  Config cfg;
  cfg.dt = kDt;
  // Keep the defaults visible: this is the executor's ordinary watchdog set,
  // not a relaxed test configuration.
  return cfg;
}

// Everything the dry run needs, with no ROS type in sight.
struct Rig
{
  Config cfg;
  TransportStateMachine sm;
  FakeController fake;
  PhaseDriver driver;
  AttachAdapter attach;
  double now = 0.0;
  std::string why;
  std::vector<CycleRow> cycles;
  std::vector<std::string> phase_entries;
  int last_phase = -1;

  explicit Rig(Config c = dryRunConfig(), std::vector<PhaseSpec> phases = fourPhaseTrajectory())
  : cfg(c), sm(cfg), driver(phases) {}

  void prime()
  {
    fake.init(now, kDt);
    sm.tick(now, fake.ptr());
    capture();
  }

  bool start(double duration_s)
  {
    // Mirror the real bring-up order: the executor starts the trajectory first,
    // then the controller's first status carries transport_active=true.
    fake.beginPhase(duration_s);
    sm.attachPhaseDriver(&driver);
    StartRequest req;
    req.duration_s = duration_s;
    return sm.start(now, req, &why);
  }

  void capture()
  {
    CycleRow row;
    row.k = static_cast<int>(cycles.size());
    row.t = now;
    row.state = due::toString(sm.state());
    row.failure_code = sm.failureCode();
    row.committed = fake.status().committed;
    row.t_traj = fake.status().t_traj;
    row.q_cmd = fake.status().q_cmd;
    cycles.push_back(row);
  }

  void notePhase()
  {
    const int idx = sm.phaseIndex();
    if (idx != last_phase) {
      if (idx >= 0 && idx < driver.count() && idx > last_phase) {
        phase_entries.push_back(driver.phases()[static_cast<size_t>(idx)].name);
      }
      last_phase = idx;
    }
  }

  void step()
  {
    now += kDt;
    fake.step(now);
    sm.tick(now, fake.ptr());
    notePhase();
    capture();
  }

  // Keep the controller alive (fresh sample) but freeze its trajectory clock.
  void frozenStep()
  {
    now += kDt;
    fake.status().stamp_s = now;
    sm.tick(now, fake.ptr());
    notePhase();
    capture();
  }
};

void feedFailure(Rig & r, const std::string & code)
{
  r.fake.status().failure_code = code;
  r.fake.status().stamp_s = r.now;
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();
}

std::string artifactRoot()
{
  const char * env = std::getenv("C43_W3R6_ARTIFACT_DIR");
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
  return (fs::temp_directory_path() / "c43_s4_w3r6_dry_run").string();
}

std::string readText(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  std::ostringstream body;
  body << in.rdbuf();
  return body.str();
}

// Write one complete six-file artifact bundle for a scenario.  The summary is
// produced by the real buildSummaryJson(), not by a hand-written expectation.
std::string writeRun(
  const std::string & name, const TransportStateMachine & sm,
  const std::vector<CycleRow> & cycles, const std::string & result,
  const std::vector<std::string> & phases, const std::vector<std::string> & extra)
{
  const std::string dir = artifactRoot() + "/" + name;
  std::error_code ec;
  fs::remove_all(dir, ec);

  due::ArtifactWriter w;
  std::string err;
  EXPECT_TRUE(w.open(dir, &err)) << err;

  due::Manifest m;
  m.run_id = name;
  m.git_head = "dry-run-no-git";
  m.package_version = "0.1.0";
  m.build_type = "dry-run";
  m.mode = "SOFTWARE_ONLY";
  m.env_class = "host";
  m.input_config_sha256 = "c43_s4_dry_run.yaml";
  m.started_utc = "1970-01-01T00:00:00Z";
  m.dry_run = true;
  EXPECT_TRUE(w.writeManifest(m, &err)) << err;
  EXPECT_TRUE(w.writeEvents(sm.events(), &err)) << err;
  EXPECT_TRUE(w.writeCycles(cycles, &err)) << err;
  EXPECT_TRUE(w.writeSummary(
    due::buildSummaryJson(name, result, sm.events(), cycles, extra,
      true, phases, "SOFTWARE_ONLY"), &err)) << err;
  EXPECT_TRUE(w.writeCommands({"scripts/run_dry_run.sh", name}, &err)) << err;
  EXPECT_TRUE(w.copyTestResults("", &err)) << err;
  return dir;
}

std::string jsonString(const std::string & body, const std::string & key)
{
  const std::string needle = "\"" + key + "\": \"";
  const size_t begin = body.find(needle);
  if (begin == std::string::npos) {
    return "";
  }
  const size_t first = begin + needle.size();
  const size_t end = body.find('"', first);
  if (end == std::string::npos) {
    return "";
  }
  return body.substr(first, end - first);
}

std::vector<std::string> jsonStringArray(const std::string & body, const std::string & key)
{
  std::vector<std::string> out;
  const std::string needle = "\"" + key + "\": [";
  const size_t begin = body.find(needle);
  if (begin == std::string::npos) {
    return out;
  }
  size_t at = begin + needle.size();
  const size_t end = body.find(']', at);
  if (end == std::string::npos) {
    return out;
  }
  while (at < end) {
    const size_t q1 = body.find('"', at);
    if (q1 == std::string::npos || q1 >= end) {
      break;
    }
    const size_t q2 = body.find('"', q1 + 1);
    if (q2 == std::string::npos || q2 > end) {
      break;
    }
    out.push_back(body.substr(q1 + 1, q2 - q1 - 1));
    at = q2 + 1;
  }
  return out;
}

State stateFromName(const std::string & name)
{
  if (name == "APPROACH") {return State::Approach;}
  if (name == "GRASP") {return State::Grasp;}
  if (name == "LIFT") {return State::Lift;}
  if (name == "TRANSPORT") {return State::Transport;}
  if (name == "LOWER") {return State::Lower;}
  if (name == "RELEASE") {return State::Release;}
  if (name == "RETREAT") {return State::Retreat;}
  if (name == "COMPLETE") {return State::Complete;}
  if (name == "HOLD") {return State::Hold;}
  if (name == "FAULT") {return State::Fault;}
  if (name == "ABORT") {return State::Abort;}
  return State::Idle;
}

std::vector<State> enteredStates(const TransportStateMachine & sm)
{
  std::vector<State> seq;
  for (const auto & e : sm.events()) {
    if (e.kind == "start" || e.kind == "advance") {
      seq.push_back(stateFromName(e.to));
    }
  }
  return seq;
}

dual_arm::Q12 bothArms(double j2)
{
  dual_arm::JointConfig q;
  q << 0.0, j2, 0.0, 0.0, 0.0, 0.0;
  return dual_arm::DualArmKinematics::stack(q, q);
}

std::array<double, 12> toArray(const dual_arm::Q12 & q)
{
  std::array<double, 12> out{};
  for (std::size_t i = 0; i < 12; ++i) {out[i] = q[i];}
  return out;
}

// Independent per-arm j2; needed for the swept-segment case, where each endpoint
// is individually safe but the straight segment between them is not.
dual_arm::Q12 armsAt(double j2_left, double j2_right)
{
  dual_arm::JointConfig l;
  l << 0.0, j2_left, 0.0, 0.0, 0.0, 0.0;
  dual_arm::JointConfig r;
  r << 0.0, j2_right, 0.0, 0.0, 0.0, 0.0;
  return dual_arm::DualArmKinematics::stack(l, r);
}

bool hasLineWith(const std::string & csv, const std::string & a, const std::string & b)
{
  std::istringstream in(csv);
  std::string line;
  while (std::getline(in, line)) {
    if (line.find(a) != std::string::npos && line.find(b) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(DryRunLoop, HappyPathVisitsEveryStateOnceAndWritesSoftwareOnlyArtifacts)
{
  Rig r;
  r.prime();
  ASSERT_EQ(r.sm.state(), State::Idle);
  ASSERT_TRUE(r.start(4.80)) << r.why;
  ASSERT_EQ(r.sm.state(), State::Approach);

  bool attach_requested = false;
  for (int i = 0; i < 2000 && !r.sm.latched() && r.sm.state() != State::Lower; ++i) {
    r.step();
    if (!attach_requested && r.sm.state() == State::Lift) {
      // GRASP is a discrete executor state crossed on the phase boundary, so the
      // first observable post-GRASP tick is LIFT.  This is the software grasp.
      ASSERT_TRUE(r.attach.request(r.now).ok);
      r.attach.tick(r.now + 0.05);
      ASSERT_EQ(r.attach.state(), AttachState::AttachedSimulated);
      attach_requested = true;
    }
  }
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  ASSERT_EQ(r.sm.state(), State::Lower);
  ASSERT_TRUE(attach_requested);

  // Drive the whole LOWER phase, not just its entry.
  while (!r.sm.latched() && r.fake.status().t_traj < 4.80 - 1e-12) {
    r.step();
  }
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  ASSERT_EQ(r.sm.state(), State::Lower);
  ASSERT_EQ(r.sm.phaseIndex(), 3);

  const std::vector<std::string> expected_phases = {
    "approach", "lift", "transport", "lower"};
  EXPECT_EQ(r.phase_entries, expected_phases);

  // RELEASE -> RETREAT -> COMPLETE are outside the four R2 motion phases; the
  // real executor advances them with phaseDone() once the plant reports done.
  r.attach.reset();
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  r.capture();
  ASSERT_EQ(r.sm.state(), State::Release);
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  r.capture();
  ASSERT_EQ(r.sm.state(), State::Retreat);
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  r.capture();

  EXPECT_EQ(r.sm.state(), State::Complete);
  EXPECT_FALSE(r.sm.latched());
  EXPECT_EQ(r.sm.failureCode(), "");

  const std::vector<State> expected_states = {
    State::Approach, State::Grasp, State::Lift, State::Transport,
    State::Lower, State::Release, State::Retreat, State::Complete};
  EXPECT_EQ(enteredStates(r.sm), expected_states);
  // "exactly once" is implied by the exact vector equality above, but make the
  // intent unmissable for a reader scanning test output.
  for (size_t i = 1; i < expected_states.size(); ++i) {
    EXPECT_NE(expected_states[i], expected_states[i - 1]);
  }

  const std::string dir = writeRun(
    "happy_path", r.sm, r.cycles, "COMPLETE", r.phase_entries,
    {"attach=simulated", "release=simulated"});
  ASSERT_TRUE(fs::exists(dir + "/manifest.json"));
  ASSERT_TRUE(fs::exists(dir + "/summary.json"));
  ASSERT_TRUE(fs::exists(dir + "/events.csv"));
  ASSERT_TRUE(fs::exists(dir + "/cycles.csv"));
  ASSERT_TRUE(fs::exists(dir + "/test_results.xml"));
  ASSERT_TRUE(fs::exists(dir + "/commands.txt"));

  const std::string manifest = readText(dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"mode\": \"SOFTWARE_ONLY\""), std::string::npos);
  EXPECT_NE(manifest.find("\"dry_run\": true"), std::string::npos);

  const std::string summary = readText(dir + "/summary.json");
  EXPECT_EQ(jsonString(summary, "mode"), "SOFTWARE_ONLY");
  EXPECT_EQ(jsonString(summary, "final_state"), "COMPLETE");
  EXPECT_NE(summary.find("\"dry_run\": true"), std::string::npos);
  EXPECT_EQ(jsonStringArray(summary, "phases"), expected_phases);

  const std::string events = readText(dir + "/events.csv");
  EXPECT_TRUE(hasLineWith(events, "start", "APPROACH"));
  EXPECT_TRUE(hasLineWith(events, "advance", "COMPLETE"));
}

// ---------------------------------------------------------------------------
// W3-R6 real collision-guard trigger.
//
// Before this revision the collision case was proved by copying the guard's
// code into ControllerStatus.failure_code (feedFailure) -- that is code
// injection, not a guard verdict.  These tests attach the real CollisionGuard to
// the real state machine (setCollisionGuard) and leave failure_code at "ok" for
// the whole scenario, so the only possible source of the HOLD is the guard call
// inside TransportStateMachine::tick().
//
// Measured geometry (CollisionGuard, link radius 0.06 m, required arm-arm gap
// 0.10 m): both arms at j2 = -2.4 put the link capsules 0.12 m inside each
// other, i.e. distance() = -0.12 m and clearance = -0.22 m.  The tests assert
// those numbers by calling the same guard the executor calls, and the state
// machine's protection event must carry the very same clearance.
// ---------------------------------------------------------------------------
constexpr double kPenetratingJ2 = -2.4;
constexpr double kRequiredArmArmGap = 0.10;

TEST(DryRunLoop, CollisionGuardHookLatchesHoldWithTheGuardsOwnCodeAndNeverCompletes)
{
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  // The executor owns the guard in dry-run mode.  No collision math is written
  // in the executor: every number comes from checkArmArm()/checkSweep().
  CollisionGuard guard;
  guard.configure(kRequiredArmArmGap, true);
  r.sm.setCollisionGuard(&guard);
  ASSERT_EQ(r.sm.collisionGuard(), &guard);

  // What the controller reports as its measured configuration.  This is the only
  // thing that changes: no failure_code, no event, no injected verdict.
  const dual_arm::Q12 penetrating = bothArms(kPenetratingJ2);
  const due::CollisionVerdict expected = guard.checkArmArm(penetrating);
  ASSERT_FALSE(expected.ok);
  ASSERT_EQ(expected.code, "collision_arm_arm");
  EXPECT_NEAR(expected.clearance_m, -0.220000, 2e-4);
  ASSERT_EQ(r.fake.status().failure_code, "ok");

  r.fake.status().q_state = toArray(penetrating);
  r.fake.status().stamp_s = r.now;
  const int checks_before = r.sm.collisionChecks();
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();

  // 1. The guard really was consulted on this tick...
  EXPECT_EQ(r.sm.collisionChecks(), checks_before + 1);
  // 2. ...and its verdict is what latched the state machine.  failure_code is
  //    still "ok": the code cannot have come from the status message.
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_TRUE(r.sm.latched());
  EXPECT_EQ(r.sm.failureCode(), "collision_arm_arm");
  EXPECT_EQ(r.fake.status().failure_code, "ok");

  // 3. The protection event quotes the guard's own measurement, so the number in
  //    the artifact is traceable to the guard call and not to a literal.
  bool guard_event = false;
  for (const auto & e : r.sm.events()) {
    if (e.kind == "protection" && e.code == "collision_arm_arm" &&
      e.detail.find(std::to_string(expected.clearance_m)) != std::string::npos)
    {
      guard_event = true;
    }
  }
  EXPECT_TRUE(guard_event) << "the protection event must quote the guard's clearance";

  // 4. A guard HOLD is a stop, not a detour: with the clock running and status
  //    samples staying fresh and frozen, the task must never complete.
  for (int i = 0; i < 200; ++i) {
    r.frozenStep();
    ASSERT_EQ(r.sm.state(), State::Hold);
    ASSERT_EQ(r.sm.failureCode(), "collision_arm_arm");
  }
  EXPECT_NE(r.sm.state(), State::Complete);
  EXPECT_EQ(r.fake.status().failure_code, "ok");

  const std::string dir = writeRun(
    "collision_guard_arm_arm", r.sm, r.cycles, "HOLD", {},
    {"collision_trigger=collision_guard_hook", "guard_code=collision_arm_arm",
      "clearance_m=" + std::to_string(expected.clearance_m),
      "required_gap_m=" + std::to_string(kRequiredArmArmGap),
      "guard_checks=" + std::to_string(r.sm.collisionChecks())});
  const std::string events = readText(dir + "/events.csv");
  EXPECT_TRUE(hasLineWith(events, "protection", "collision_arm_arm"));
  const std::string summary = readText(dir + "/summary.json");
  EXPECT_EQ(jsonString(summary, "final_state"), "HOLD");
  EXPECT_EQ(jsonString(summary, "mode"), "SOFTWARE_ONLY");
  EXPECT_NE(summary.find("\"dry_run\": true"), std::string::npos);
}

TEST(DryRunLoop, SameGeometryWithAPassingGuardThresholdRunsToComplete)
{
  // Counterfactual for the test above, and the proof that the guard is the
  // trigger: the geometry, the status, the clock and the phase driver are
  // identical; the ONLY difference is the guard's own required gap.
  //
  // Direction of the comparison: clearance = distance - required_gap, so a
  // TIGHTER gap can only make a verdict worse.  "Raise it so the scenario
  // passes" therefore means relaxing the required gap to -0.30 m (a deliberately
  // permissive threshold that accepts up to 0.30 m of penetration).  The
  // measured distance stays -0.12 m in both tests.
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  CollisionGuard strict;
  strict.configure(kRequiredArmArmGap, true);
  const due::CollisionVerdict strict_verdict = strict.checkArmArm(bothArms(kPenetratingJ2));
  ASSERT_FALSE(strict_verdict.ok);

  CollisionGuard lenient;
  lenient.configure(-0.30, true);
  const due::CollisionVerdict lenient_verdict = lenient.checkArmArm(bothArms(kPenetratingJ2));
  ASSERT_TRUE(lenient_verdict.ok);
  EXPECT_NEAR(lenient_verdict.clearance_m, 0.180000, 2e-4);

  r.sm.setCollisionGuard(&lenient);
  r.fake.status().q_state = toArray(bothArms(kPenetratingJ2));
  r.fake.status().stamp_s = r.now;
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();

  ASSERT_GE(r.sm.collisionChecks(), 1);
  EXPECT_FALSE(r.sm.latched());
  EXPECT_EQ(r.sm.state(), State::Approach);
  EXPECT_EQ(r.sm.failureCode(), "");

  // The same scenario now runs the whole script to COMPLETE with the guard
  // attached the entire time (it is consulted on every tick).
  while (!r.sm.latched() && r.sm.state() != State::Lower) {
    r.step();
  }
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  while (!r.sm.latched() && r.fake.status().t_traj < 4.80 - 1e-12) {
    r.step();
  }
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  ASSERT_EQ(r.sm.state(), State::Lower);
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  ASSERT_TRUE(r.sm.phaseDone(r.now, &r.why)) << r.why;
  r.capture();

  EXPECT_EQ(r.sm.state(), State::Complete);
  EXPECT_FALSE(r.sm.latched());
  EXPECT_GT(r.sm.collisionChecks(), 100)
    << "the guard must have been consulted throughout the run";

  const std::string dir = writeRun(
    "collision_guard_threshold_passes", r.sm, r.cycles, "COMPLETE", r.phase_entries,
    {"collision_trigger=collision_guard_hook", "guard_code=none",
      "clearance_m=" + std::to_string(lenient_verdict.clearance_m),
      "required_gap_m=-0.300000",
      "guard_checks=" + std::to_string(r.sm.collisionChecks())});
  const std::string summary = readText(dir + "/summary.json");
  EXPECT_EQ(jsonString(summary, "final_state"), "COMPLETE");
  EXPECT_EQ(jsonString(summary, "mode"), "SOFTWARE_ONLY");
  const std::string events = readText(dir + "/events.csv");
  EXPECT_FALSE(hasLineWith(events, "protection", "collision"));
}

TEST(DryRunLoop, CollisionGuardObjectCheckLatchesHoldWithItsOwnCode)
{
  // Check 2 through the same hook: the arm-arm check passes, the object pose is
  // what the guard refuses.  Again failure_code stays "ok".
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  CollisionGuard guard;
  guard.configure(kRequiredArmArmGap, true);
  guard.addExclusionSphere(Eigen::Vector3d::Zero(), 1.0);
  const dual_arm::Iso3 pose = dual_arm::Iso3::Identity();
  const due::CollisionVerdict expected = guard.checkObjectPose(pose);
  ASSERT_FALSE(expected.ok);
  ASSERT_EQ(expected.code, "collision_object");

  r.sm.setCollisionGuard(&guard);
  r.sm.setCollisionObjectPose(pose);
  ASSERT_TRUE(r.sm.hasCollisionObjectPose());

  const int checks_before = r.sm.collisionChecks();
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();

  EXPECT_EQ(r.sm.collisionChecks(), checks_before + 1);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "collision_object");
  EXPECT_EQ(r.fake.status().failure_code, "ok");
  EXPECT_NE(r.sm.state(), State::Complete);

  bool quoted = false;
  for (const auto & e : r.sm.events()) {
    if (e.kind == "protection" && e.code == "collision_object" &&
      e.detail.find(std::to_string(expected.clearance_m)) != std::string::npos)
    {
      quoted = true;
    }
  }
  EXPECT_TRUE(quoted);

  const std::string dir = writeRun(
    "collision_guard_object", r.sm, r.cycles, "HOLD", {},
    {"collision_trigger=collision_guard_hook", "guard_code=collision_object",
      "clearance_m=" + std::to_string(expected.clearance_m)});
  EXPECT_TRUE(hasLineWith(readText(dir + "/events.csv"), "protection", "collision_object"));
  EXPECT_EQ(jsonString(readText(dir + "/summary.json"), "final_state"), "HOLD");
}

TEST(DryRunLoop, CollisionGuardSweepCheckCatchesASegmentThatOnlyCollidesInBetween)
{
  // Check 3 through the same hook.  Both endpoints pass checkArmArm, so a
  // point-in-time test would let this motion through; only the swept segment
  // fails.  Measured by the guard (required gap 0.10 m):
  //   A (j2l=-2.80, j2r=-1.50): clearance +0.00184 m
  //   B (j2l=-1.70, j2r=-2.50): clearance +0.02095 m
  //   checkSweep(A, B, 8)     : -0.06113 m -> collision_sweep
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  CollisionGuard guard;
  guard.configure(kRequiredArmArmGap, true);
  r.sm.setCollisionGuard(&guard);

  const dual_arm::Q12 a = armsAt(-2.80, -1.50);
  const dual_arm::Q12 b = armsAt(-1.70, -2.50);
  const due::CollisionVerdict va = guard.checkArmArm(a);
  const due::CollisionVerdict vb = guard.checkArmArm(b);
  const due::CollisionVerdict vsweep =
    guard.checkSweep(a, b, TransportStateMachine::kGuardSweepSamples);
  ASSERT_TRUE(va.ok) << va.clearance_m;
  ASSERT_TRUE(vb.ok) << vb.clearance_m;
  EXPECT_NEAR(va.clearance_m, 0.00184, 2e-4);
  EXPECT_NEAR(vb.clearance_m, 0.02095, 2e-4);
  ASSERT_FALSE(vsweep.ok);
  ASSERT_EQ(vsweep.code, "collision_sweep");
  EXPECT_NEAR(vsweep.clearance_m, -0.06113, 5e-3);

  // First observation: the guard stores the baseline and stays quiet.
  r.fake.status().q_state = toArray(a);
  r.fake.status().stamp_s = r.now;
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  EXPECT_EQ(r.sm.state(), State::Approach);

  // Second observation: both configurations are individually safe, the segment
  // is not, so the executor must stop.
  r.fake.status().q_state = toArray(b);
  r.fake.status().stamp_s = r.now;
  r.sm.tick(r.now, r.fake.ptr());
  r.capture();

  EXPECT_EQ(r.sm.collisionChecks(), 2);
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "collision_sweep");
  EXPECT_EQ(r.fake.status().failure_code, "ok");
  EXPECT_NE(r.sm.state(), State::Complete);

  bool quoted = false;
  for (const auto & e : r.sm.events()) {
    if (e.kind == "protection" && e.code == "collision_sweep" &&
      e.detail.find(std::to_string(vsweep.clearance_m)) != std::string::npos)
    {
      quoted = true;
    }
  }
  EXPECT_TRUE(quoted) << "the protection event must quote the swept clearance";

  const std::string dir = writeRun(
    "collision_guard_sweep", r.sm, r.cycles, "HOLD", {},
    {"collision_trigger=collision_guard_hook", "guard_code=collision_sweep",
      "clearance_m=" + std::to_string(vsweep.clearance_m),
      "endpoint_a_clearance_m=" + std::to_string(va.clearance_m),
      "endpoint_b_clearance_m=" + std::to_string(vb.clearance_m)});
  EXPECT_TRUE(hasLineWith(readText(dir + "/events.csv"), "protection", "collision_sweep"));
  EXPECT_EQ(jsonString(readText(dir + "/summary.json"), "final_state"), "HOLD");
}

TEST(DryRunLoop, HoldRequesterRoundTripIsRecordedWithReason)
{
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  // A controller-side refusal the executor must never ignore.
  feedFailure(r, "stale_command");
  ASSERT_EQ(r.sm.state(), State::Hold);
  ASSERT_EQ(r.sm.failureCode(), "stale_command");

  HoldRequester hold;
  const std::uint32_t seq = 7;
  hold.begin(r.now + 0.01, seq, "stale_command");
  ASSERT_TRUE(hold.needSend());
  hold.markSent(r.now + 0.01);
  hold.onServiceReply(true, "queued");

  HoldObservation obs;
  obs.now_s = r.now + 0.02;
  obs.have_status = true;
  obs.status_age_s = 0.001;
  obs.state = "HOLD";
  obs.hold_origin = "executor";
  obs.ack_seq = seq;
  obs.ack_result = 1;
  const due::HoldStep step = hold.onObservation(obs);

  EXPECT_EQ(step.action, HoldAction::Done);
  EXPECT_TRUE(hold.done());
  EXPECT_NE(hold.outcome().find("frozen"), std::string::npos);

  const std::string dir = writeRun(
    "hold_round_trip", r.sm, r.cycles, "HOLD", {},
    {"hold_reason=stale_command", "hold_outcome=" + hold.outcome()});

  const std::string events = readText(dir + "/events.csv");
  EXPECT_TRUE(hasLineWith(events, "protection", "HOLD"));
  EXPECT_TRUE(hasLineWith(events, "protection", "stale_command"));

  const std::string summary = readText(dir + "/summary.json");
  const std::vector<std::string> extra = jsonStringArray(summary, "extra");
  EXPECT_NE(std::find(extra.begin(), extra.end(), "hold_reason=stale_command"), extra.end());
  EXPECT_EQ(jsonString(summary, "final_state"), "HOLD");
}

TEST(DryRunLoop, InjectedAttachFaultBecomesFaultAndNeverCompletes)
{
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(4.80)) << r.why;

  AttachAdapter attach;
  ASSERT_TRUE(attach.request(r.now).ok);
  ASSERT_TRUE(attach.awaitingCompletion());
  attach.injectFailure("attach_failed");
  ASSERT_EQ(attach.state(), AttachState::Failed);
  ASSERT_EQ(attach.failureCode(), "attach_failed");
  EXPECT_EQ(attach.artifactTag(), "none");

  // The adapter is pure logic; the executor learns of the failure through the
  // controller status.  Do not invent a second failure path.
  feedFailure(r, attach.failureCode());
  EXPECT_EQ(r.sm.state(), State::Fault);
  EXPECT_EQ(r.sm.failureCode(), "attach_failed");
  EXPECT_TRUE(r.sm.latched());
  EXPECT_NE(r.sm.state(), State::Complete);

  const std::string dir = writeRun(
    "attach_fault", r.sm, r.cycles, "FAULT", {},
    {"attach_failure=attach_failed", "attach_tag=none"});
  EXPECT_NE(readText(dir + "/summary.json").find("\"final_state\": \"FAULT\""),
    std::string::npos);
}

TEST(DryRunLoop, FrozenTrajectoryClockRaisesPhaseTimeout)
{
  Rig r;
  r.prime();
  ASSERT_TRUE(r.start(1.00)) << r.why;

  // Reach LIFT (phase index 1) using the real phase driver.
  for (int i = 0; i < 300 && !r.sm.latched() && r.sm.state() != State::Lift; ++i) {
    r.step();
  }
  ASSERT_FALSE(r.sm.latched()) << r.sm.failureCode();
  ASSERT_EQ(r.sm.state(), State::Lift);
  ASSERT_EQ(r.sm.phaseIndex(), 1);
  const double lift_since = r.sm.stateSince();
  const double frozen_t = r.fake.status().t_traj;

  // Freeze t_traj while the executor clock keeps moving: the phase watchdog
  // must fire even though status samples stay fresh.
  while (!r.sm.latched() && r.now < lift_since + 2.00) {
    r.frozenStep();
  }

  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_timeout");
  EXPECT_EQ(r.fake.status().t_traj, frozen_t);
  EXPECT_NE(r.sm.state(), State::Complete);

  // A latched HOLD must survive further frozen samples without changing code.
  r.frozenStep();
  EXPECT_EQ(r.sm.state(), State::Hold);
  EXPECT_EQ(r.sm.failureCode(), "phase_timeout");

  const std::string dir = writeRun(
    "phase_timeout", r.sm, r.cycles, "HOLD", {},
    {"frozen_t_traj=" + std::to_string(frozen_t), "timeout=phase_timeout"});
  EXPECT_NE(readText(dir + "/summary.json").find("\"final_state\": \"HOLD\""),
    std::string::npos);
}
