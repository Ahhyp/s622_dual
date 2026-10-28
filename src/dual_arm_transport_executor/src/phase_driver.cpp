// C4.3 S4.1 W3-R2 — task phase driver implementation.
//
// Pure C++17, no allocation after configure(), no clock read: every input is
// injected.  See phase_driver.hpp for the scope note.
#include "dual_arm_transport_executor/phase_driver.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm_transport_executor
{

namespace
{
constexpr double kEps = 1e-9;
}  // namespace

bool isKnownPhaseName(const std::string & name)
{
  // The frozen trajectory (dual_arm_object_task/trajectory.hpp) uses these
  // names; anything else is a contract mismatch, not a new phase.
  return name == "approach" || name == "lift" || name == "transport" ||
         name == "lower" || name == "settle" || name == "settle2" ||
         name == "settle_pre" || name == "settle_post";
}

PhaseDriver::PhaseDriver(std::vector<PhaseSpec> phases)
{
  configure(std::move(phases));
}

void PhaseDriver::configure(std::vector<PhaseSpec> phases)
{
  phases_ = std::move(phases);
  recomputeStarts();
  index_ = -1;
}

void PhaseDriver::recomputeStarts()
{
  starts_.assign(phases_.size() + 1, 0.0);
  double acc = 0.0;
  for (std::size_t i = 0; i < phases_.size(); ++i) {
    starts_[i] = acc;
    const double d = phases_[i].duration_s;
    acc += (std::isfinite(d) && d > 0.0) ? d : 0.0;
  }
  starts_[phases_.size()] = acc;
}

bool PhaseDriver::setPhaseCount(int count)
{
  // Grow-only: a shrinking report is a contract violation, not a new table.
  if (count < 0 || count < static_cast<int>(phases_.size())) {
    return false;
  }
  phases_.resize(static_cast<std::size_t>(count));
  recomputeStarts();
  return true;
}

bool PhaseDriver::setPhase(int index, const std::string & name, double duration_s)
{
  if (index < 0 || index >= static_cast<int>(phases_.size())) {
    return false;
  }
  if (!isKnownPhaseName(name) || !std::isfinite(duration_s) || duration_s < 0.0) {
    return false;
  }
  PhaseSpec & spec = phases_[static_cast<std::size_t>(index)];
  // A slot is learned once; a later disagreement is a contract mismatch that
  // observeExternal() will report, not something to paper over here.
  if (!spec.name.empty() && spec.name != name) {
    return false;
  }
  spec.name = name;
  spec.duration_s = duration_s;
  recomputeStarts();
  return true;
}

void PhaseDriver::reset()
{
  index_ = -1;
}

double PhaseDriver::startOf(int index) const
{
  if (index < 0 || starts_.empty()) {
    return 0.0;
  }
  const int n = static_cast<int>(phases_.size());
  if (index > n) {
    return starts_[static_cast<std::size_t>(n)];
  }
  return starts_[static_cast<std::size_t>(index)];
}

double PhaseDriver::totalDuration() const
{
  return starts_.empty() ? 0.0 : starts_.back();
}

int PhaseDriver::indexAt(double t_traj) const
{
  if (!enabled() || !std::isfinite(t_traj) || t_traj < 0.0) {
    return -1;
  }
  const int n = static_cast<int>(phases_.size());
  for (int i = 0; i < n; ++i) {
    // Half-open interval [start_i, start_{i+1}); the last phase is closed.
    const double end = starts_[static_cast<std::size_t>(i + 1)];
    if (t_traj < end || (i == n - 1 && t_traj >= starts_[static_cast<std::size_t>(i)])) {
      if (t_traj + kEps < starts_[static_cast<std::size_t>(i)]) {
        // Cannot happen for a monotone starts_ table; kept as a guard.
        return -1;
      }
      return i;
    }
  }
  return n - 1;
}

PhaseObservation PhaseDriver::make(PhaseEvent e, int idx, double t_traj) const
{
  PhaseObservation o;
  o.event = e;
  o.index = idx;
  o.count = count();
  if (idx >= 0 && idx < count()) {
    const auto & p = phases_[static_cast<std::size_t>(idx)];
    o.name = p.name;
    o.duration_s = p.duration_s;
    o.elapsed_s = t_traj - starts_[static_cast<std::size_t>(idx)];
    if (o.elapsed_s < 0.0) {
      o.elapsed_s = 0.0;
    }
    if (p.duration_s > 0.0 && o.elapsed_s > p.duration_s) {
      o.elapsed_s = p.duration_s;
    }
  }
  return o;
}

PhaseObservation PhaseDriver::observeLocal(double t_traj)
{
  const int idx = indexAt(t_traj);
  if (idx < 0) {
    if (index_ != -1) {
      // The clock went backwards past the start of the trajectory.
      index_ = -1;
      return make(PhaseEvent::Desync, -1, t_traj);
    }
    return make(PhaseEvent::NotStarted, -1, t_traj);
  }
  if (idx == index_) {
    return make(PhaseEvent::None, index_, t_traj);
  }
  if (index_ == -1) {
    if (idx != 0) {
      // Joined mid-trajectory: the executor cannot prove phases 0..idx-1 ran.
      index_ = idx;
      return make(PhaseEvent::Desync, idx, t_traj);
    }
    index_ = 0;
    return make(isKnownPhaseName(phases_[0].name) ? PhaseEvent::Entered : PhaseEvent::UnknownName,
                0, t_traj);
  }
  if (idx < index_) {
    index_ = idx;
    return make(PhaseEvent::Desync, idx, t_traj);
  }
  const bool single_step = (idx == index_ + 1);
  index_ = idx;
  if (!single_step) {
    return make(PhaseEvent::Desync, idx, t_traj);
  }
  return make(isKnownPhaseName(phases_[static_cast<std::size_t>(idx)].name) ?
         PhaseEvent::Entered : PhaseEvent::UnknownName, idx, t_traj);
}

PhaseObservation PhaseDriver::observeExternal(
  int reported_index, const std::string & reported_name, double t_traj)
{
  if (!enabled()) {
    return make(PhaseEvent::NotStarted, -1, t_traj);
  }
  if (reported_index < 0) {
    if (index_ != -1) {
      index_ = -1;
      return make(PhaseEvent::Desync, -1, t_traj);
    }
    return make(PhaseEvent::NotStarted, -1, t_traj);
  }
  if (reported_index >= count()) {
    // Out of contract range: treat as desync, do not adopt an impossible index.
    return make(PhaseEvent::Desync, index_, t_traj);
  }
  // Name verification comes first: an index we can place but a name we cannot
  // vouch for means the controller and this executor disagree about what the
  // phase is, which must latch a hold rather than advance.
  if (!isKnownPhaseName(reported_name) ||
    reported_name != phases_[static_cast<std::size_t>(reported_index)].name)
  {
    return make(PhaseEvent::UnknownName, index_, t_traj);
  }
  if (reported_index == index_) {
    return make(PhaseEvent::None, index_, t_traj);
  }
  if (index_ == -1) {
    // First observation: adopt the controller's index (mid-run attach).  The
    // state machine's Entered handler walks forward one state at a time, so a
    // join at index 1 still passes through LIFT rather than teleporting.
    index_ = reported_index;
    return make(PhaseEvent::Entered, reported_index, t_traj);
  }
  const bool single_step = (reported_index == index_ + 1);
  const bool backwards = (reported_index < index_);
  index_ = reported_index;
  if (backwards || !single_step) {
    return make(PhaseEvent::Desync, reported_index, t_traj);
  }
  return make(PhaseEvent::Entered, reported_index, t_traj);
}

}  // namespace dual_arm_transport_executor
