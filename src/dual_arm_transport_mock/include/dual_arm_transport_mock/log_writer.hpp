// C4.3 S3 -- W6: the two §8 artifacts.
//
// cycles.csv keeps q_prev / q_cand / committed side by side ON PURPOSE.  A log
// that stores only the committed state cannot answer "why did cycle 812 not
// move?", which is the only question anyone asks of a failed run.  The vetoed
// candidate is in the same row as the state it was rejected from.
#ifndef DUAL_ARM_TRANSPORT_MOCK__LOG_WRITER_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__LOG_WRITER_HPP_

#include <string>
#include <vector>

#include "dual_arm_transport_mock/mock_loop.hpp"

namespace dual_arm_transport_mock {

/// The §8 column order.  Stable on purpose: analysis scripts index by position.
const std::vector<std::string>& cycleColumnNames();
/// The §8 runs.csv column order.
const std::vector<std::string>& runColumnNames();

/// Header line only (no trailing newline beyond the one terminating the row).
std::string cyclesCsvHeader();
/// Header line only.
std::string runsCsvHeader();
/// sweeps.csv header: three rows per cycle, one per object source.  Kept as its
/// own table because a cycle has three sweep results; folding them into
/// cycles.csv would need three copies of every column.
std::string sweepsCsvHeader();
/// The sweeps.csv rows belonging to one cycle, in `objectSources()` order.
std::vector<std::vector<std::string>> sweepRows(const CycleRecord& rec);

/// One CSV row, values in `cycleColumnNames()` order.
std::vector<std::string> cycleRow(const CycleRecord& rec);
/// Header + one row per record.
std::string cyclesCsv(const std::vector<CycleRecord>& log);

/// One CSV row (no header, no trailing newline), values in `runColumnNames()` order.
std::vector<std::string> runRow(const RunSummary& sum, const std::string& scenario,
                                const std::string& object_source);
/// Header + a single row.
std::string runsCsv(const RunSummary& sum, const std::string& scenario,
                    const std::string& object_source);

/// Write `text` to `path`.  Throws `std::runtime_error` on any I/O failure --
/// a silently missing log is worse than a crash, because the run looks clean.
void writeTextFile(const std::string& path, const std::string& text);

// ---------------------------------------------------------------- W7 artifacts

/// clearance.csv: one row per cycle, the four clearance readings that mean
/// different things side by side (whole robot / cross arm / proximal / capsule)
/// plus the world layer, which is the one S3 added.
std::string clearanceCsvHeader();
std::string clearanceCsv(const std::vector<CycleRecord>& log);
/// sweeps.csv: three rows per cycle, one per object source.
std::string sweepsCsv(const std::vector<CycleRecord>& log);

/// cycles.jsonl: one JSON object per line, same fields as cycles.csv.  `%.17g`
/// again, so the machine-readable copy is the same evidence as the human one.
std::string cyclesJsonl(const std::vector<CycleRecord>& log);

/// One scenario as it was asked and as it came back.
struct ScenarioRun {
    std::string scenario;      ///< "yaw_+15.00deg" / "w_obj_1.0e-01"
    std::string object_source; ///< the source that decided commit/abort
    std::string note;          ///< e.g. "control arm (object task off)"
    RunSummary summary;
};

/// What produced the numbers.  `files` is name -> path; the hashes are computed
/// HERE so a caller cannot forget one, and a missing input file is an error rather
/// than an empty string in the provenance block.
struct Provenance {
    std::string git_head;
    std::string package_version;
    std::string build_stamp;
    std::vector<std::pair<std::string, std::string>> files;
};

/// Full parameter snapshots (§8 requires ALL fields, not the interesting ones).
std::string qpParamsJson(const dual_arm_qp::DualArmQpParams& p);
std::string checkOptionsJson(const CheckOptions& o);

/// summary.json: runs + trajectory + every parameter + provenance.
std::string summaryJson(const std::vector<ScenarioRun>& runs, const TrajectorySpec& spec,
                        const MockLoopOptions& opt, const Provenance& prov);

/// report.md (§8): the human-readable view of the SAME values as summary.json.
/// Generated, never hand-written -- a report whose numbers were typed by hand is a
/// second source of truth that silently drifts from the log.
std::string summaryReport(const std::vector<ScenarioRun>& runs, const TrajectorySpec& spec,
                          const MockLoopOptions& opt, const Provenance& prov);

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__LOG_WRITER_HPP_
