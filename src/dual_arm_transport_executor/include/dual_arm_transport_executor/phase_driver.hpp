// C4.3 S4.1 W3-R2 — task phase driver (APPROACH -> LIFT -> TRANSPORT -> LOWER).
//
// Scope note (R2, SOFTWARE ONLY):
//   * This component turns a trajectory clock into a *phase index* and audits
//     that the index only ever advances by exactly one step.  It performs no
//     physical work and touches no actuator.
//   * Two observation modes exist because the controller-side phase counter is
//     not published yet (msg/TransportStatus has no phase field):
//       - observeLocal(t_traj): derive the index from the phase durations that
//         were used to build the trajectory.  This is the mode R2 can run today;
//         it is a *local inference* and must never be described as the
//         controller's own phase state.
//       - observeExternal(index, name, t_traj): audit a controller-reported
//         index.  Switching to this mode is a one-line change once the field
//         exists, and the desync rules are identical.
//   * Deterministic: no clock is read here.  t_traj is injected by the caller,
//     which is what makes the fixed-step tests reproducible.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__PHASE_DRIVER_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__PHASE_DRIVER_HPP_

#include <string>
#include <vector>

namespace dual_arm_transport_executor
{

// One phase of the transport trajectory: a display name and its nominal
// duration.  Order in the vector is the execution order.
struct PhaseSpec
{
  std::string name;
  double duration_s = 0.0;
};

// The four phases R2 owns, plus the settle phases the frozen trajectory already
// contains.  Names are compared case-sensitively against the controller's
// start_transport phase_name values.
bool isKnownPhaseName(const std::string & name);

enum class PhaseEvent
{
  None,        // no change since the previous observation
  Entered,     // index advanced by exactly one (the normal case)
  Desync,      // index jumped, or went backwards: caller must HOLD
  UnknownName, // index advanced but the phase name is not a known phase
  NotStarted,  // t_traj < 0 or the driver has no phases configured
};

struct PhaseObservation
{
  PhaseEvent event = PhaseEvent::None;
  int index = -1;           // -1 == not started
  std::string name;
  double elapsed_s = 0.0;   // time spent inside this phase (>= 0)
  double duration_s = 0.0;  // nominal duration of this phase (0 == unknown)
  int count = 0;            // number of configured phases
};

class PhaseDriver
{
public:
  PhaseDriver() = default;
  explicit PhaseDriver(std::vector<PhaseSpec> phases);

  // Replaces the phase list and resets the index.  Empty list == disabled.
  void configure(std::vector<PhaseSpec> phases);

  // Incremental "learn from the controller stream" variant (W3-R2, defect B1).
  // The executor never sees the controller's phase table ahead of time - it is
  // built inside the controller from the start_transport request - so the only
  // place to learn it is TransportStatus (phase_count + phase_name +
  // phase_duration_s).  setPhaseCount() grows the table to the reported size,
  // setPhase() fills one slot when that slot is first seen.  Unlike configure()
  // they do NOT reset the observed index, so a table can be completed while a
  // run is already in progress.
  // setPhase() returns false and changes nothing for an out-of-range index, a
  // negative/non-finite duration, or a non-known phase name (the same contract
  // check observeExternal() applies).
  bool setPhaseCount(int count);
  bool setPhase(int index, const std::string & name, double duration_s);

  void reset();

  bool enabled() const { return !phases_.empty(); }
  int count() const { return static_cast<int>(phases_.size()); }
  const std::vector<PhaseSpec> & phases() const { return phases_; }

  // Cumulative start time of `index`; startOf(count()) == totalDuration().
  double startOf(int index) const;
  double totalDuration() const;

  // Current index (-1 before the first observation).
  int index() const { return index_; }


  // Which phase a trajectory clock falls into; -1 when disabled / t < 0.
  int indexAt(double t_traj) const;

  // --- observation modes -------------------------------------------------
  PhaseObservation observeLocal(double t_traj);
  // Same rules as observeLocal, plus name verification: the controller-reported
  // name must be a known phase name AND must agree with the local table at that
  // index.  A reported "teleport" at index 1 is not silently accepted as "lift";
  // it is reported as UnknownName and the index is not adopted.
  // ONE deliberate difference: the controller is the authority on which phase the
  // trajectory is in, and the adapter may attach mid-run (the phase table is
  // learned from the status stream), so the FIRST external observation adopts the
  // reported index and reports Entered.  Every later jump/regression is Desync.
  // (observeLocal keeps the strict "first index must be 0" rule because a local
  // clock inferring a mid-run start has no authority to claim it.)
  PhaseObservation observeExternal(
    int reported_index, const std::string & reported_name, double t_traj);

private:
  PhaseObservation make(PhaseEvent e, int idx, double t_traj) const;
  void recomputeStarts();

  std::vector<PhaseSpec> phases_;
  std::vector<double> starts_;  // size count()+1, monotone non-decreasing
  int index_ = -1;
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__PHASE_DRIVER_HPP_
