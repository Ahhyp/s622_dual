// C4.3 S4.1 W3-R1 — deterministic artifact writer (non-realtime only).
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__ARTIFACTS_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__ARTIFACTS_HPP_

#include <array>
#include <string>
#include <vector>

#include "dual_arm_transport_executor/state_machine.hpp"

namespace dual_arm_transport_executor
{

// The six mandatory files of task book §3 W3-R1-06.
struct ArtifactNames
{
  static const char * manifest() { return "manifest.json"; }
  static const char * events() { return "events.csv"; }
  static const char * cycles() { return "cycles.csv"; }
  static const char * summary() { return "summary.json"; }
  static const char * testResults() { return "test_results.xml"; }
  static const char * commands() { return "commands.txt"; }
};

struct Manifest
{
  std::string run_id;
  std::string git_head;
  std::string package_version;
  std::string build_type;
  std::string mode;         // "unit" | "bench" | "replay"
  std::string env_class;    // "host" | "container" | "ci"
  std::string input_config_sha256;
  std::string started_utc;
  // W3-R6: explicit evidence marker.  `mode` says what kind of run this is;
  // dry_run says whether a physical plant/actuator participated.  It is never
  // inferred from `mode`, so an old writer default stays false.
  bool dry_run = false;
};

struct CycleRow
{
  int k = 0;
  double t = 0.0;
  std::string state;
  std::string failure_code;
  bool committed = false;
  double t_traj = 0.0;
  std::array<double, 12> q_cmd{};
};

class ArtifactWriter
{
public:
  bool open(const std::string & dir, std::string * err);
  const std::string & dir() const { return dir_; }

  bool writeManifest(const Manifest & m, std::string * err) const;
  bool writeEvents(const std::vector<Event> & events, std::string * err) const;
  bool writeCycles(const std::vector<CycleRow> & rows, std::string * err) const;
  bool writeSummary(const std::string & json_body, std::string * err) const;
  bool writeCommands(const std::vector<std::string> & argv, std::string * err) const;
  // Copies an existing gtest xml into the artifact directory.
  bool copyTestResults(const std::string & xml_path, std::string * err) const;

private:
  std::string dir_;
};

// Deterministic summary body: no wall-clock, no temp paths.  Two runs with the
// same inputs must produce the same bytes.
// W3-R6 additions are optional and additive: `dry_run` marks the evidence as
// software-only, `phases` is the ordered phase list actually entered, and
// `mode` is emitted only when the caller supplies it.
std::string buildSummaryJson(const std::string & run_id, const std::string & result,
                             const std::vector<Event> & events,
                             const std::vector<CycleRow> & cycles,
                             const std::vector<std::string> & extra_lines,
                             bool dry_run = false,
                             const std::vector<std::string> & phases = {},
                             const std::string & mode = "");

const char * toString(Protection p);

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__ARTIFACTS_HPP_
