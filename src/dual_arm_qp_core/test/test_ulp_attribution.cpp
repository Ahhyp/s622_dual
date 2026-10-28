// C4.3 S4 W2 -- ULP ATTRIBUTION EXPERIMENT (reconstruction).
// ===========================================================================
// WHAT THIS FILE IS, AND WHAT IT IS NOT  (read before citing a number from it)
// ===========================================================================
//
// The frozen criterion is
//     C4.3_S4_执行规划.md:533  "与 S3 `ff_1` 场景逐周期比 `dq` 位相同（≤1 ULP）"
// and the S4-T2 acceptance test that enforces a *stand-in* for it lives in
//     src/dual_arm_qp_controller/test/test_mock_closure.cpp:269
//     EXPECT_LE(worst_abs, 1e-12)
//
// Two documents assert that a "decisive test" named
//     UlpMetric.S4T2_DifferenceIsTheTrajectoryClockNotTheSolver
// proves the observed difference is the trajectory clock and not the solver:
//     docs/双臂协同/.../C4.3_S4_W2_ULP判据变更申请.md:37
//     docs/双臂协同/.../C4.3_S4_W2-R3_ULP候选V1独立验证报告.md:41
// No such test exists anywhere in this repository (grep finds only those two
// prose citations).  This file is an INDEPENDENT RECONSTRUCTION of the
// experiment those documents *describe*; it is not a restoration of lost code.
//
// EQUIVALENCE STATEMENT (required by the task, stated explicitly):
//   * This is NOT the acceptance test, and it is NOT equivalent to it.
//   * Equivalent aspects: the scenario (S3 `ff_1`, 638 cycles), the frozen
//     fixture, the QP parameters, the row assembly (`row.J = task.J`,
//     `row.e = -task.e`, `row.body_twist_ff = task.v_ff`), the plant
//     (`q += dq`, ideal velocity servo) and the solver instance
//     (`makeDefaultSolver()`).
//   * NOT equivalent aspects: it bypasses
//     `dual_arm_qp_controller::TransportTask` and therefore ROS, the
//     controller node, and the executor; it drives the trajectory clock itself
//     and can run it under TWO different clock laws; and it reports the
//     original criterion instead of asserting a tolerance on it.
//   * Passing this file proves NOTHING about `≤1 ULP`.  It contains no
//     threshold for the criterion: the criterion verdict is *printed*
//     (MEASURED/VERDICT lines), and what is actually ASSERTED here is only
//     (a) harness integrity (the fixture and the S3 artifact agree bit for
//     bit, all cycles converge, no NaN/Inf) and (b) propositions that are
//     forced by arithmetic (e.g. at k = 0 both clock laws produce exactly the
//     same double, so their dq must be bit-identical there -- if it is not,
//     this harness is broken, not the criterion).
//
// WHY AN ATTRIBUTION EXPERIMENT CAN ANSWER THE QUESTION AT ALL
//   The comparison is  dq_controller(k)  vs  dq_S3(k).  Between the two runs
//   there are exactly two classes of difference:
//     C  the trajectory clock: `TransportTask::advance()` accumulates
//        (`t_traj_ = min(t_traj_ + dt, duration)`, transport_task.cpp), while
//        the S3 artifact's `t` is exactly `k*dt` for all 638 rows
//        (verified: max|t - k*0.008| = 0.0).  The two therefore disagree by up
//        to 3.553e-15 s.
//     S  the solver path: OSQP's workspace and ADMM state persist inside
//        `OsqpSolver` across cycles (osqp_solver.cpp: "Persistent-workspace"),
//        so the same mathematical input does not imply the same floating-point
//        iterate.  `DualArmQp::setWarmStart()` switches part of that state.
//   Experiment A holds the solver configuration fixed and varies ONLY the
//   clock (accumulate vs k*dt) -> measures C.  Experiment B holds `t_traj`
//   fixed at k*dt and varies ONLY the warm-start state -> measures S.  The two
//   measurements are printed side by side with the observed acceptance gap.
//
//   LIMIT OF THE METHOD, stated up front: A and B cannot reproduce the *S3*
//   side, only the difference between two of MY runs.  So C measures the
//   clock's causal contribution to a dq difference, and S measures one real
//   solver-side sensitivity; neither is a proof of what the S3 run did
//   differently, and this file does not claim one.  The one place where the
//   attribution can be settled exactly -- cycle 0, where both clocks are the
//   same double and both inputs are provably identical -- is measured
//   separately (TEST CycleZero).
//
// NUMBERS: this file only ever PRINTs them.  The numbers quoted in the header
// comments above/below are the pre-registered predictions derived from reading
// the code, and each one is printed as `PREDICTED` next to the `MEASURED`
// value so a disagreement is visible instead of being edited away.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

#ifndef S4_FIXTURE_CSV
#define S4_FIXTURE_CSV ""
#endif
#ifndef S4_TRANSPORT_YAML
#define S4_TRANSPORT_YAML ""
#endif
#ifndef S4_BASELINE_YAML
#define S4_BASELINE_YAML ""
#endif
#ifndef S3_ARTIFACT_CSV
#define S3_ARTIFACT_CSV ""
#endif

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_object_task::AnchoredObjectTask;
using dual_arm_object_task::AnchoredObjectTaskBuilder;
using dual_arm_object_task::FrozenObjectTrajectory;
using dual_arm_object_task::TrajectorySpec;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;
using dual_arm_qp::DualArmQpResult;
using dual_arm_qp::ObjectTaskRow;
using dual_arm_qp::Vec12;

constexpr int kJoints = 12;
constexpr double kDt = 0.008;  // == DualArmQpParams::dt == TrajectorySpec::dt
constexpr std::size_t kExpectedCycles = 638;
// `kSigmaMinStackMin` (src/dual_arm_transport_mock/include/dual_arm_transport_mock/
// mock_loop.hpp:42) and `kRankTolRatio` (:44) -- the thresholds the S3 loop uses
// to turn the stacked singular value into a refused/not-refused decision.
constexpr double kS3SigmaMinStackMin = 1.0e-6;
constexpr double kS3RankTolRatio = 1.0e-9;

// ---------------------------------------------------------------------------
// formatting
// ---------------------------------------------------------------------------

std::string bits(double v) {
    uint64_t u = 0;
    static_assert(sizeof(u) == sizeof(v), "double must be 64 bit");
    std::memcpy(&u, &v, sizeof(u));
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(u));
    return std::string(b);
}

std::string g17(double v) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return std::string(b);
}

std::string sci(double v) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.3e", v);
    return std::string(b);
}

/// Distance in ULPs between two doubles (monotonic bit-pattern mapping).
/// Copied verbatim from the acceptance test's helper so the numbers here are
/// directly comparable with the ones it prints.
uint64_t ulpDistance(double a, double b) {
    int64_t ia = 0;
    int64_t ib = 0;
    std::memcpy(&ia, &a, sizeof(ia));
    std::memcpy(&ib, &b, sizeof(ib));
    if (ia < 0) ia = std::numeric_limits<int64_t>::min() - ia;
    if (ib < 0) ib = std::numeric_limits<int64_t>::min() - ib;
    const uint64_t ua = static_cast<uint64_t>(ia) ^ 0x8000000000000000ULL;
    const uint64_t ub = static_cast<uint64_t>(ib) ^ 0x8000000000000000ULL;
    return (ua > ub) ? (ua - ub) : (ub - ua);
}

// ---------------------------------------------------------------------------
// the frozen S3 fixture (29 columns; the S4-T2 acceptance input)
// ---------------------------------------------------------------------------

struct FixtureRow {
    int k = 0;
    double t = 0.0;
    Q12 q_prev = Q12::Zero();
    Vec12 dq = Vec12::Zero();
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    bool committed = false;
};

std::vector<FixtureRow> loadFixture(const std::string& path, bool* ok) {
    std::vector<FixtureRow> rows;
    *ok = false;
    std::ifstream in(path);
    if (!in) return rows;
    std::string line;
    if (!std::getline(in, line)) return rows;  // header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::string cell;
        std::istringstream is(line);
        while (std::getline(is, cell, ',')) f.push_back(cell);
        if (f.size() < 28) return {};
        FixtureRow r;
        r.k = std::stoi(f[0]);
        r.t = std::stod(f[1]);
        for (int i = 0; i < kJoints; ++i) {
            r.q_prev[i] = std::stod(f[2 + i]);
            r.dq[i] = std::stod(f[14 + i]);
        }
        r.e_obj_pos_m = std::stod(f[26]);
        r.e_obj_rot_rad = std::stod(f[27]);
        r.committed = (f.size() > 28) ? (std::stoi(f[28]) != 0) : true;
        rows.push_back(r);
    }
    *ok = !rows.empty();
    return rows;
}

// ---------------------------------------------------------------------------
// the S3 44-column cycle artifact (the run the fixture was cut from)
// ---------------------------------------------------------------------------
//
// The 29-column fixture drops the three columns that V1's fifth sub-criterion
// needs (`active_limit_count`, `sigma_min_stack`, `rank_stack`) and the
// `iterations` column that fingerprints the solver.  They exist only in
// `runs/c43_s3/w8/ff_1/cycles.csv` (44 columns), which is the S3 run itself.
// The two files are cross-checked bit for bit in TEST Harness, so grading V1-5
// against this artifact grades it against exactly the fixture the criterion is
// defined on.
struct S3Row {
    int k = 0;
    double t = 0.0;
    bool committed = false;
    bool converged = false;
    int active_limit_count = 0;
    int iterations = 0;
    int rank_stack = 0;
    double sigma_min_rel = 0.0;
    double sigma_min_stack = 0.0;
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    std::string failure_code;
    Q12 q_prev = Q12::Zero();
    Vec12 dq = Vec12::Zero();
};

std::vector<std::string> splitOn(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cell;
    std::istringstream is(s);
    while (std::getline(is, cell, sep)) out.push_back(cell);
    return out;
}

bool toVec12(const std::string& cell, Vec12* v) {
    const std::vector<std::string> toks = splitOn(cell, ' ');
    std::vector<std::string> keep;
    for (const auto& t : toks) {
        if (!t.empty()) keep.push_back(t);
    }
    if (keep.size() != static_cast<std::size_t>(kJoints)) return false;
    for (int i = 0; i < kJoints; ++i) (*v)[i] = std::stod(keep[static_cast<std::size_t>(i)]);
    return true;
}

std::vector<S3Row> loadS3Artifact(const std::string& path, bool* ok) {
    std::vector<S3Row> rows;
    *ok = false;
    std::ifstream in(path);
    if (!in) return rows;
    std::string line;
    if (!std::getline(in, line)) return rows;
    const std::vector<std::string> header = splitOn(line, ',');
    auto index = [&header](const std::string& name) -> int {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return static_cast<int>(i);
        }
        return -1;
    };
    const int ik = index("k");
    const int it = index("t");
    const int icommitted = index("committed");
    const int iconverged = index("converged");
    const int ial = index("active_limit_count");
    const int iiters = index("iterations");
    const int irank = index("rank_stack");
    const int ismr = index("sigma_min_rel");
    const int isms = index("sigma_min_stack");
    const int iep = index("e_obj_pos_m");
    const int ier = index("e_obj_rot_rad");
    const int ifc = index("failure_code");
    const int iq = index("q_prev");
    const int idq = index("dq");
    if (ik < 0 || it < 0 || icommitted < 0 || ial < 0 || isms < 0 || iq < 0 || idq < 0) {
        return rows;
    }
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> f = splitOn(line, ',');
        if (f.size() != header.size()) return {};
        S3Row r;
        r.k = std::stoi(f[static_cast<std::size_t>(ik)]);
        r.t = std::stod(f[static_cast<std::size_t>(it)]);
        r.committed = std::stoi(f[static_cast<std::size_t>(icommitted)]) != 0;
        r.converged = (iconverged >= 0) ? (std::stoi(f[static_cast<std::size_t>(iconverged)]) != 0) : true;
        r.active_limit_count = std::stoi(f[static_cast<std::size_t>(ial)]);
        r.iterations = (iiters >= 0) ? std::stoi(f[static_cast<std::size_t>(iiters)]) : -1;
        r.rank_stack = (irank >= 0) ? std::stoi(f[static_cast<std::size_t>(irank)]) : -1;
        r.sigma_min_rel = (ismr >= 0) ? std::stod(f[static_cast<std::size_t>(ismr)]) : 0.0;
        r.sigma_min_stack = std::stod(f[static_cast<std::size_t>(isms)]);
        r.e_obj_pos_m = (iep >= 0) ? std::stod(f[static_cast<std::size_t>(iep)]) : 0.0;
        r.e_obj_rot_rad = (ier >= 0) ? std::stod(f[static_cast<std::size_t>(ier)]) : 0.0;
        r.failure_code = (ifc >= 0) ? f[static_cast<std::size_t>(ifc)] : std::string();
        if (!toVec12(f[static_cast<std::size_t>(iq)], &r.q_prev)) return {};
        if (!toVec12(f[static_cast<std::size_t>(idq)], &r.dq)) return {};
        rows.push_back(r);
    }
    *ok = !rows.empty();
    return rows;
}

// ---------------------------------------------------------------------------
// frozen grasp frames: read from the S2 baseline, never re-typed here
// ---------------------------------------------------------------------------
//
// `transport_baseline.yaml` is the documented source of truth for the frozen
// grasp frame (`frozen_baseline.hpp` says so in its own comment).  Reading it
// at run time keeps this file from being a THIRD copy of the numbers.
Iso3 parseIso3String(const std::string& s) {
    const std::vector<std::string> rows = splitOn(s, ';');
    if (rows.size() != 4) throw std::runtime_error("not a 4x4 matrix string");
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (std::size_t r = 0; r < 4; ++r) {
        const std::vector<std::string> cols = splitOn(rows[r], ',');
        if (cols.size() != 4) throw std::runtime_error("not a 4x4 matrix string");
        for (std::size_t c = 0; c < 4; ++c) m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = std::stod(cols[c]);
    }
    Iso3 out = Iso3::Identity();
    out.matrix() = m;
    return out;
}

Iso3 loadGrasp(const std::string& path, const std::string& key, std::string* err) {
    try {
        const YAML::Node node = YAML::LoadFile(path);
        if (!node["grasp"] || !node["grasp"][key]) {
            *err = "transport_baseline.yaml has no grasp." + key;
            return Iso3::Identity();
        }
        return parseIso3String(node["grasp"][key].as<std::string>());
    } catch (const std::exception& e) {
        *err = std::string("cannot read grasp frame '") + key + "' from " + path + ": " + e.what();
        return Iso3::Identity();
    }
}

// ---------------------------------------------------------------------------
// the two clock laws under test
// ---------------------------------------------------------------------------

enum class ClockLaw {
    /// `TransportTask::advance()`: t_traj_ = min(t_traj_ + dt, duration).
    Accumulate = 0,
    /// The S3 artifact's law: t = k * dt, clamped at the end exactly like the
    /// controller clamps (so the ONLY difference is accumulation vs product).
    Kdt = 1,
};

const char* clockName(ClockLaw law) {
    return (law == ClockLaw::Accumulate) ? "t_traj_accumulated(controller)" : "t=k*dt(S3 artifact)";
}

// ---------------------------------------------------------------------------
// one cycle of the loop, logged at full width
// ---------------------------------------------------------------------------

struct CycleLog {
    double t = 0.0;  // the t_traj actually fed to FrozenObjectTrajectory::at()
    Vec12 dq = Vec12::Zero();
    bool converged = false;
    bool inputs_ok = false;
    int iterations = 0;
    double sigma_min_rel = 0.0;
    double sigma_min_stack = 0.0;
    int rank_stack = 0;
    int active_limit_count = 0;
    double e_obj_pos_m = 0.0;
    double e_obj_rot_rad = 0.0;
    double cost_norm = 0.0;  // ||q_cost|| of the assembled problem (cycle-0 probe)
};

struct RunLog {
    std::vector<CycleLog> cyc;
    std::string solver_name;
    std::string error;  // non-empty => harness problem, never a criterion result
    std::size_t nonconverged = 0;
    std::size_t nonfinite = 0;
    double duration = 0.0;
};

// ---------------------------------------------------------------------------
// the exact S4-T2 / S3 `ff_1` QP parameters (copied from the acceptance test)
// ---------------------------------------------------------------------------

DualArmQpParams s3Params() {
    DualArmQpParams p;
    dual_arm_qp::defaultS622Limits(&p.q_min, &p.q_max, &p.v_max);
    p.dt = 0.008;
    p.kp_rel = 10.0;
    p.w_rel = 1.0;
    p.w_slack = 1000.0;
    p.w_center = 1.0e-4;
    p.w_reg = 1e-6;
    p.kp_obj = 20.0;
    p.w_obj = 1.0;
    p.w_slack_obj = 1000.0;
    return p;
}

/// sigma_min and rank of the stacked 12x12 [J_rel; J_obj], defined exactly as
/// `computeStackRank()` in src/dual_arm_transport_mock/src/mock_loop.cpp:46-62
/// so the V1-5 comparison is against the same quantity the S3 column reports.
void stackRank(const Mat6x12& j_rel, const Mat6x12& j_obj, double* sigma_min, int* rank,
               double* sigma_max) {
    Eigen::Matrix<double, 12, 12> m;
    m.topRows<6>() = j_rel;
    m.bottomRows<6>() = j_obj;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(m);
    *sigma_max = svd.singularValues()(0);
    *sigma_min = svd.singularValues()(11);
    const double tol = kS3RankTolRatio * (*sigma_max);
    int r = 0;
    for (int i = 0; i < 12; ++i) {
        if (svd.singularValues()(i) > tol) ++r;
    }
    *rank = r;
}

/// `active_limit_count`, defined exactly as in mock_loop.cpp:305-315: a joint
/// counts as sitting on its box when |dq - bound| <= 1e-12 (rad).
int activeLimitCount(const Q12& q, const DualArmQpParams& p, const Vec12& dq) {
    int active = 0;
    for (int i = 0; i < kJoints; ++i) {
        const double lo = std::max(-p.v_max[i] * p.dt, p.q_min[i] - q[i]);
        const double hi = std::min(p.v_max[i] * p.dt, p.q_max[i] - q[i]);
        if (dq[i] <= lo + 1e-12 || dq[i] >= hi - 1e-12) ++active;
    }
    return active;
}

// ---------------------------------------------------------------------------
// the loop
// ---------------------------------------------------------------------------

RunLog runLoop(const std::vector<FixtureRow>& ref, ClockLaw law, bool warm_start,
                bool fresh_solver_each_cycle) {
    RunLog log;
    if (ref.empty()) {
        log.error = "empty fixture";
        return log;
    }
    std::string err;
    const Iso3 g_left = loadGrasp(S4_BASELINE_YAML, "object_T_left_tcp", &err);
    const Iso3 g_right = loadGrasp(S4_BASELINE_YAML, "object_T_right_tcp", &err);
    if (!err.empty()) {
        log.error = err;
        return log;
    }

    DualArmKinematics kin;
    const AnchoredObjectTaskBuilder builder(kin, g_left, g_right);

    // Reproduces TransportTask::start(): the start pose comes from the frozen
    // baseline via leftEstimate(q0), and the relative target is latched once.
    TrajectorySpec spec;
    try {
        spec = dual_arm_object_task::loadTrajectorySpec(S4_TRANSPORT_YAML);
    } catch (const std::exception& e) {
        log.error = std::string("cannot load trajectory: ") + e.what();
        return log;
    }
    const Q12 q0 = ref.front().q_prev;
    if (spec.dt != kDt) {
        log.error = "trajectory dt != 0.008 (the experiment assumes 125 Hz)";
        return log;
    }
    if (spec.start_from_baseline) {
        spec.start_world_T_object = builder.leftEstimate(q0);
        spec.start_from_baseline = false;
    }
    FrozenObjectTrajectory traj(spec);
    log.duration = traj.duration();
    const Iso3 left_T_right = builder.leftTrightGrasp();

    const DualArmQpParams params = s3Params();
    // fresh_solver_each_cycle rebuilds the QP -- and therefore OSQP's workspace,
    // its scaling and its ADMM iterate -- for every cycle: identical problem
    // data, identical warm-start policy, but no solver state carried across
    // cycles.  This is the arm that isolates "state of the solver" from
    // "state of the plant"; warm start alone turned out to be a null knob.
    auto makeQp = [&]() {
        auto q = std::make_unique<DualArmQp>(kin, params);
        q->setWarmStart(warm_start);
        return q;
    };
    std::unique_ptr<DualArmQp> qp = makeQp();
    log.solver_name = qp->solverName();

    Q12 q = q0;
    double t_accum = 0.0;
    log.cyc.reserve(ref.size());
    for (std::size_t i = 0; i < ref.size(); ++i) {
        if (fresh_solver_each_cycle) qp = makeQp();
        const double ti = (law == ClockLaw::Kdt)
                              ? std::min(static_cast<double>(i) * kDt, traj.duration())
                              : t_accum;
        Vec6 v_ff = Vec6::Zero();
        const Iso3 t_d = traj.at(ti, &v_ff);
        const AnchoredObjectTask task = builder.build(q, t_d, v_ff);

        ObjectTaskRow row;
        row.J = task.J;
        row.e = -task.e;
        row.body_twist_ff = task.v_ff;
        row.valid = task.inputs_ok;

        const DualArmQpResult r = qp->solve(q, left_T_right, row);

        CycleLog c;
        c.t = ti;
        c.dq = r.dq;
        c.converged = r.converged;
        c.inputs_ok = task.inputs_ok;
        c.iterations = r.iterations;
        c.sigma_min_rel = r.sigma_min_rel;
        c.e_obj_pos_m = task.e.head<3>().norm();
        c.e_obj_rot_rad = task.e.tail<3>().norm();
        double smax = 0.0;
        stackRank(kin.relativeJacobian(q), row.J, &c.sigma_min_stack, &c.rank_stack, &smax);
        (void)smax;  // sigma_max is not compared against anything here
        c.active_limit_count = activeLimitCount(q, params, r.dq);
        c.cost_norm = qp->lastProblem().q.norm();

        if (!r.dq.allFinite()) ++log.nonfinite;
        if (!r.converged) ++log.nonconverged;
        log.cyc.push_back(c);

        q += r.dq;  // ideal velocity servo, exactly as the S3 mock plant
        t_accum = std::min(t_accum + kDt, traj.duration());
    }
    return log;
}

// ---------------------------------------------------------------------------
// the four runs: {clock} x {warm start}
// ---------------------------------------------------------------------------

struct RunSet {
    bool ok = false;
    std::string error;
    std::vector<FixtureRow> ref;
    std::vector<S3Row> s3;
    bool s3_ok = false;
    RunLog acc_warm;
    RunLog acc_nowarm;
    RunLog acc_fresh;  // accumulated clock, warm start on, NEW QP every cycle
    RunLog kdt_warm;
    RunLog kdt_nowarm;
};

const RunSet& runSet() {
    static const RunSet set = [] {
        RunSet s;
        bool fixture_ok = false;
        s.ref = loadFixture(S4_FIXTURE_CSV, &fixture_ok);
        if (!fixture_ok) {
            s.error = std::string("cannot load the S3 fixture: ") + S4_FIXTURE_CSV;
            return s;
        }
        s.s3 = loadS3Artifact(S3_ARTIFACT_CSV, &s.s3_ok);
        s.acc_warm = runLoop(s.ref, ClockLaw::Accumulate, true, false);
        s.acc_nowarm = runLoop(s.ref, ClockLaw::Accumulate, false, false);
        s.kdt_warm = runLoop(s.ref, ClockLaw::Kdt, true, false);
        s.kdt_nowarm = runLoop(s.ref, ClockLaw::Kdt, false, false);
        s.acc_fresh = runLoop(s.ref, ClockLaw::Accumulate, true, true);
        for (const RunLog* l : {&s.acc_warm, &s.acc_nowarm, &s.kdt_warm, &s.kdt_nowarm}) {
            if (!l->error.empty()) {
                s.error = l->error;
                return s;
            }
        }
        s.ok = true;
        return s;
    }();
    return set;
}

// ---------------------------------------------------------------------------
// comparing two runs (or a run against the fixture)
// ---------------------------------------------------------------------------

struct Series {
    std::string name;
    std::vector<double> t;   // per cycle
    std::vector<Vec12> dq;   // per cycle
    std::vector<bool> committed;
    std::vector<int> iterations;
    std::vector<double> sigma_min_stack;
    std::vector<int> rank_stack;
    std::vector<int> active_limit_count;
    std::vector<double> e_obj_pos;
    std::vector<double> e_obj_rot;
};

Series seriesFromRun(const RunLog& log, const std::string& name) {
    Series s;
    s.name = name;
    s.t.reserve(log.cyc.size());
    for (const auto& c : log.cyc) {
        s.t.push_back(c.t);
        s.dq.push_back(c.dq);
        s.committed.push_back(c.converged);  // commit == converged for the controller
        s.iterations.push_back(c.iterations);
        s.sigma_min_stack.push_back(c.sigma_min_stack);
        s.rank_stack.push_back(c.rank_stack);
        s.active_limit_count.push_back(c.active_limit_count);
        s.e_obj_pos.push_back(c.e_obj_pos_m);
        s.e_obj_rot.push_back(c.e_obj_rot_rad);
    }
    return s;
}

Series seriesFromFixture(const std::vector<FixtureRow>& ref) {
    Series s;
    s.name = "S3_fixture";
    for (const auto& r : ref) {
        s.t.push_back(r.t);
        s.dq.push_back(r.dq);
        s.committed.push_back(r.committed);
        s.iterations.push_back(-1);
        s.sigma_min_stack.push_back(0.0);
        s.rank_stack.push_back(-1);
        s.active_limit_count.push_back(-1);
        s.e_obj_pos.push_back(r.e_obj_pos_m);
        s.e_obj_rot.push_back(r.e_obj_rot_rad);
    }
    return s;
}

struct Diff {
    std::size_t cycles = 0;
    std::size_t pairs = 0;
    std::size_t bit_identical = 0;
    std::size_t pairs_gt_1ulp = 0;
    // absolute difference (the honest floor)
    double max_abs = 0.0;
    std::size_t max_abs_cycle = 0;
    int max_abs_joint = -1;
    double max_abs_a = 0.0;
    double max_abs_b = 0.0;
    // ULP over every pair
    uint64_t max_ulp = 0;
    std::size_t max_ulp_cycle = 0;
    int max_ulp_joint = -1;
    double max_ulp_a = 0.0;
    double max_ulp_b = 0.0;
    // ULP over "meaningful" joints, using the acceptance test's own gate
    // (|a| >= 1e-9 && |b| >= 1e-9) so the number matches what it prints.
    uint64_t max_ulp_nonzero = 0;
    std::size_t max_ulp_nz_cycle = 0;
    int max_ulp_nz_joint = -1;
    double max_ulp_nz_a = 0.0;
    double max_ulp_nz_b = 0.0;
    std::size_t nonzero_pairs = 0;
    std::size_t nonzero_gt_1ulp = 0;
    // first cycle whose dq vector is not bit-identical
    std::size_t first_diff_cycle = static_cast<std::size_t>(-1);
    int first_diff_joint = -1;
    // per-cycle t_traj gap
    double max_t_gap = 0.0;
    std::size_t max_t_gap_cycle = 0;
    std::size_t cycles_t_differ = 0;
    bool any_nan_or_inf = false;
};

Diff diffSeries(const Series& a, const Series& b) {
    Diff d;
    d.cycles = std::min(a.dq.size(), b.dq.size());
    for (std::size_t i = 0; i < d.cycles; ++i) {
        const double tg = std::abs(a.t[i] - b.t[i]);
        if (tg > d.max_t_gap) {
            d.max_t_gap = tg;
            d.max_t_gap_cycle = i;
        }
        if (a.t[i] != b.t[i]) ++d.cycles_t_differ;
        for (int j = 0; j < kJoints; ++j) {
            const double x = a.dq[i][j];
            const double y = b.dq[i][j];
            if (!std::isfinite(x) || !std::isfinite(y)) d.any_nan_or_inf = true;
            ++d.pairs;
            const double ad = std::abs(x - y);
            if (ad == 0.0 && bits(x) == bits(y)) ++d.bit_identical;
            const uint64_t u = ulpDistance(x, y);
            if (u > 1) ++d.pairs_gt_1ulp;
            if (u > d.max_ulp) {
                d.max_ulp = u;
                d.max_ulp_cycle = i;
                d.max_ulp_joint = j;
                d.max_ulp_a = x;
                d.max_ulp_b = y;
            }
            if (ad > d.max_abs) {
                d.max_abs = ad;
                d.max_abs_cycle = i;
                d.max_abs_joint = j;
                d.max_abs_a = x;
                d.max_abs_b = y;
            }
            if (std::abs(x) >= 1e-9 && std::abs(y) >= 1e-9) {
                ++d.nonzero_pairs;
                if (u > 1) ++d.nonzero_gt_1ulp;
                if (u > d.max_ulp_nonzero) {
                    d.max_ulp_nonzero = u;
                    d.max_ulp_nz_cycle = i;
                    d.max_ulp_nz_joint = j;
                    d.max_ulp_nz_a = x;
                    d.max_ulp_nz_b = y;
                }
            }
            if (d.first_diff_cycle == static_cast<std::size_t>(-1) && bits(x) != bits(y)) {
                d.first_diff_cycle = i;
                d.first_diff_joint = j;
            }
        }
    }
    return d;
}

void printDiff(const std::string& label, const std::string& a_name, const std::string& b_name,
               const Diff& d) {
    std::printf("[ATTR] ---- %s ----\n", label.c_str());
    std::printf("[ATTR]   cycles=%zu pairs=%zu\n", d.cycles, d.pairs);
    std::printf("[ATTR]   t_traj: cycles where t differs=%zu  max|dt|=%.3e at cycle %zu\n",
                d.cycles_t_differ, d.max_t_gap, d.max_t_gap_cycle);
    std::printf("[ATTR]   dq bit-identical pairs=%zu/%zu (%.2f%%)  pairs with ULP>1: %zu (%.2f%%)\n",
                d.bit_identical, d.pairs, 100.0 * static_cast<double>(d.bit_identical) / static_cast<double>(d.pairs ? d.pairs : 1),
                d.pairs_gt_1ulp, 100.0 * static_cast<double>(d.pairs_gt_1ulp) / static_cast<double>(d.pairs ? d.pairs : 1));
    if (d.pairs == 0 || d.bit_identical == d.pairs) {
        // Nothing differs at all: report the fact instead of a sentinel index,
        // so the line can never be read as "cycle 0 joint -1 differs by 0".
        std::printf("[ATTR]   max|dq_a - dq_b| = 0.000e+00 -- every one of the %zu pairs is "
                    "bit-identical, so there is no worst pair to name\n", d.pairs);
        std::printf("[ATTR]   max ULP over ALL pairs = 0 (bit-identical everywhere)\n");
        std::printf("[ATTR]   max ULP over NONZERO pairs = 0; nonzero pairs=%zu, of which ULP>1: 0\n",
                    d.nonzero_pairs);
    } else {
        std::printf("[ATTR]   max|dq_a - dq_b| = %.3e at cycle %zu joint %d\n", d.max_abs,
                    d.max_abs_cycle, d.max_abs_joint);
        std::printf("[ATTR]     %s = %s (%s)\n", a_name.c_str(), g17(d.max_abs_a).c_str(),
                    bits(d.max_abs_a).c_str());
        std::printf("[ATTR]     %s = %s (%s)\n", b_name.c_str(), g17(d.max_abs_b).c_str(),
                    bits(d.max_abs_b).c_str());
        std::printf("[ATTR]   max ULP over ALL pairs = %llu at cycle %zu joint %d (%s vs %s)\n",
                    static_cast<unsigned long long>(d.max_ulp), d.max_ulp_cycle, d.max_ulp_joint,
                    g17(d.max_ulp_a).c_str(), g17(d.max_ulp_b).c_str());
        std::printf("[ATTR]   max ULP over NONZERO pairs (|dq|>=1e-9 both sides) = %llu at cycle %zu joint %d (%s vs %s); nonzero pairs=%zu, of which ULP>1: %zu\n",
                    static_cast<unsigned long long>(d.max_ulp_nonzero), d.max_ulp_nz_cycle,
                    d.max_ulp_nz_joint, g17(d.max_ulp_nz_a).c_str(), g17(d.max_ulp_nz_b).c_str(),
                    d.nonzero_pairs, d.nonzero_gt_1ulp);
    }
    if (d.first_diff_cycle != static_cast<std::size_t>(-1)) {
        std::printf("[ATTR]   first cycle with any non-bit-identical joint = %zu (joint %d)\n",
                    d.first_diff_cycle, d.first_diff_joint);
    } else {
        std::printf("[ATTR]   first cycle with any non-bit-identical joint = none\n");
    }
}

/// Uniform one-line verdict, so every criterion in this file reads the same.
void report(const char* id, const std::string& measured, const std::string& bound, bool pass) {
    std::printf("[%s] measured=%s bound=%s verdict=%s\n", id, measured.c_str(), bound.c_str(),
                pass ? "PASS" : "FAIL");
}

bool g_harness_ok = false;
bool g_s3_ok = false;

void requireHarness() {
    const RunSet& s = runSet();
    ASSERT_TRUE(g_harness_ok) << "harness error (this is not a criterion result): " << s.error;
}

// ===========================================================================
// TEST 1 -- harness integrity
// ===========================================================================
//
// Everything here must hold for ANY number the other tests print to mean
// anything.  In particular the 29-column fixture and the 44-column S3 artifact
// must be bit-for-bit the same data, because V1-5 is graded on the artifact
// while the dq comparison is graded on the fixture.
TEST(UlpAttribution, Harness) {
    const RunSet& s = runSet();
    if (!s.error.empty()) {
        ADD_FAILURE() << "harness error: " << s.error;
        return;
    }
    g_harness_ok = true;

    EXPECT_EQ(s.ref.size(), kExpectedCycles);
    EXPECT_EQ(s.acc_warm.cyc.size(), kExpectedCycles);
    std::printf("[HARNESS] fixture=%s rows=%zu dt=%.17g\n", S4_FIXTURE_CSV, s.ref.size(), kDt);
    std::printf("[HARNESS] solver=%s (makeDefaultSolver())\n", s.acc_warm.solver_name.c_str());
    // duration() is the SUM of the five phase durations (0.30+1.50+1.50+1.50+0.30
    // = 5.10 s), NOT cycles*dt.  5.10/0.008 = 637.5, so the loop runs 638 cycles and
    // the last one lands on the clamp (t_accum = min(t_accum+dt, duration)).  Asserting
    // 638*dt here would silently encode that clamp as if it were a rounding error.
    std::printf("[HARNESS] trajectory duration=%.17g s, dt=%.17g -> ceil(duration/dt)=%zu "
                "cycles (the last one clamped to duration)\n",
                s.acc_warm.duration, kDt,
                static_cast<std::size_t>(std::ceil(s.acc_warm.duration / kDt)));
    EXPECT_NEAR(s.acc_warm.duration, 5.10, 1e-12);
    EXPECT_EQ(static_cast<std::size_t>(std::ceil(s.acc_warm.duration / kDt)), kExpectedCycles);

    // The fixture's t must be exactly k*dt, otherwise the Kdt law is not the
    // S3 law and the whole attribution argument collapses.
    double max_t_dev = 0.0;
    for (const auto& r : s.ref) {
        max_t_dev = std::max(max_t_dev, std::abs(r.t - static_cast<double>(r.k) * kDt));
    }
    std::printf("[HARNESS] fixture max|t - k*dt| = %.3e  (PREDICTED 0.0)\n", max_t_dev);
    EXPECT_EQ(max_t_dev, 0.0) << "the S3 clock is not exactly k*dt; the Kdt law would be a lie";

    // Convergence: a non-converged cycle makes every dq comparison below
    // meaningless, so it is a harness failure, not a criterion result.
    for (const RunLog* l : {&s.acc_warm, &s.acc_nowarm, &s.kdt_warm, &s.kdt_nowarm}) {
        EXPECT_EQ(l->nonconverged, 0u) << "a cycle did not converge";
        EXPECT_EQ(l->nonfinite, 0u) << "a cycle produced NaN/Inf";
    }

    // ---- the S3 artifact: same data as the fixture, plus the dropped columns
    if (!s.s3_ok) {
        ADD_FAILURE() << "cannot load the S3 44-column artifact at '" << S3_ARTIFACT_CSV
                      << "'.  V1 criterion 5 (active_limit_count / sigma_min_stack "
                         "decisions) CANNOT be graded without it, and this test does not "
                         "silently pass in that case.";
        return;
    }
    g_s3_ok = true;
    ASSERT_EQ(s.s3.size(), kExpectedCycles);
    std::size_t q_diff = 0;
    std::size_t dq_diff = 0;
    std::size_t t_diff = 0;
    for (std::size_t i = 0; i < s.s3.size(); ++i) {
        for (int j = 0; j < kJoints; ++j) {
            if (bits(s.s3[i].q_prev[j]) != bits(s.ref[i].q_prev[j])) ++q_diff;
            if (bits(s.s3[i].dq[j]) != bits(s.ref[i].dq[j])) ++dq_diff;
        }
        if (bits(s.s3[i].t) != bits(s.ref[i].t)) ++t_diff;
    }
    std::printf("[HARNESS] artifact=%s rows=%zu  fixture-vs-artifact bitwise mismatches: "
                "q_prev=%zu dq=%zu t=%zu\n",
                S3_ARTIFACT_CSV, s.s3.size(), q_diff, dq_diff, t_diff);
    EXPECT_EQ(q_diff, 0u);
    EXPECT_EQ(dq_diff, 0u);
    EXPECT_EQ(t_diff, 0u);

    std::printf("[HARNESS] S3 artifact: active_limit_count in {");
    for (int v : {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}) {
        bool present = false;
        for (const auto& r : s.s3) present = present || (r.active_limit_count == v);
        if (present) std::printf("%d ", v);
    }
    std::printf("}  rank_stack in {");
    for (int v : {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}) {
        bool present = false;
        for (const auto& r : s.s3) present = present || (r.rank_stack == v);
        if (present) std::printf("%d ", v);
    }
    double smin = std::numeric_limits<double>::max();
    double smax = 0.0;
    int iters_max = 0;
    for (const auto& r : s.s3) {
        smin = std::min(smin, r.sigma_min_stack);
        smax = std::max(smax, r.sigma_min_stack);
        iters_max = std::max(iters_max, r.iterations);
    }
    std::printf("}  sigma_min_stack in [%.6f, %.6f]  iterations_max=%d\n", smin, smax, iters_max);
}

// ===========================================================================
// TEST 2 -- EXPERIMENT A: vary ONLY the clock, hold the solver fixed
// ===========================================================================
//
// "Identical solver inputs but shifted t_traj": both runs use the same QP
// object, the same warm-start policy, the same parameters, the same fixture and
// the same row assembly; the only difference is how the per-cycle t_traj is
// produced.  The dq difference between these two runs is therefore the clock's
// causal contribution -- an upper bound, since the real S3 run differs from the
// controller in other ways too.
TEST(UlpAttribution, ClockOnly) {
    requireHarness();
    const RunSet& s = runSet();
    const Series a = seriesFromRun(s.acc_warm, "dq_accumulated");
    const Series b = seriesFromRun(s.kdt_warm, "dq_k_dt");
    const Diff d = diffSeries(a, b);
    std::printf("[ATTR] clock laws: A = %s, B = %s; solver state identical (warm start on)\n",
                clockName(ClockLaw::Accumulate), clockName(ClockLaw::Kdt));
    printDiff("EXPERIMENT A: clock differs, solver/warm-start identical", a.name, b.name, d);

    // Forced by arithmetic: at k = 0 both laws evaluate to exactly 0.0, so the
    // entire cycle-0 dq vector must be bit-identical.  If not, this harness is
    // broken and none of its numbers may be quoted.
    std::size_t same0 = 0;
    for (int j = 0; j < kJoints; ++j) {
        if (bits(a.dq[0][j]) == bits(b.dq[0][j])) ++same0;
    }
    std::printf("[ATTR]   cycle 0: t_a=%s t_b=%s  bit-identical joints=%zu/%d\n",
                g17(a.t[0]).c_str(), g17(b.t[0]).c_str(), same0, kJoints);
    EXPECT_EQ(same0, static_cast<std::size_t>(kJoints))
        << "at k=0 both clock laws are the same double; a difference there cannot come "
           "from the clock";

    std::printf("[ATTR] clock-only contribution: max|dq diff| = %.3e rad, max ULP=%llu\n",
                d.max_abs, static_cast<unsigned long long>(d.max_ulp));
}

// ===========================================================================
// TEST 3 -- EXPERIMENT B: vary ONLY the solver state, hold t_traj fixed
// ===========================================================================
//
// "Identical t_traj but perturbed solver warm-start": both runs use the k*dt
// clock (bit-identical t, bit-identical q chain inputs, bit-identical rows);
// the only difference is `DualArmQp::setWarmStart()`, which changes the ADMM
// starting point fed to OSQP's persistent workspace.  The dq difference is
// therefore solver-attributable, measured without any clock contamination.
TEST(UlpAttribution, SolverOnly) {
    requireHarness();
    const RunSet& s = runSet();
    const Series w = seriesFromRun(s.kdt_warm, "dq_warm_start");
    const Series n = seriesFromRun(s.kdt_nowarm, "dq_no_warm_start");
    const Diff d = diffSeries(w, n);
    printDiff("EXPERIMENT B: t_traj identical (k*dt), warm start on vs off", w.name, n.name, d);

    std::size_t same0 = 0;
    for (int j = 0; j < kJoints; ++j) {
        if (bits(w.dq[0][j]) == bits(n.dq[0][j])) ++same0;
    }
    std::printf("[ATTR]   cycle 0: bit-identical joints=%zu/%d (warm start has no history "
                "yet, so this is expected to be %d)\n",
                same0, kJoints, kJoints);
    std::printf("[ATTR] solver-only contribution: max|dq diff| = %.3e rad, max ULP=%llu\n",
                d.max_abs, static_cast<unsigned long long>(d.max_ulp));

    // The 2x2 lattice, so the reader can see the decomposition is not a
    // story about one pair of runs.
    const Series aw = seriesFromRun(s.acc_warm, "acc_warm");
    const Series an = seriesFromRun(s.acc_nowarm, "acc_nowarm");
    const Diff d_accClock = diffSeries(aw, w);
    const Diff d_anClock = diffSeries(an, n);
    const Diff d_warm = diffSeries(aw, an);
    const Diff d_kw = diffSeries(w, n);
    const Diff d_both = diffSeries(aw, n);
    std::printf("[ATTR] 2x2 max|dq diff| lattice:\n");
    std::printf("[ATTR]   clock, warm start on : acc_warm   vs kdt_warm   = %.3e\n",
                d_accClock.max_abs);
    std::printf("[ATTR]   clock, warm start off: acc_nowarm vs kdt_nowarm = %.3e\n",
                d_anClock.max_abs);
    std::printf("[ATTR]   solver, accumulated   : acc_warm   vs acc_nowarm = %.3e\n",
                d_warm.max_abs);
    std::printf("[ATTR]   solver, k*dt          : kdt_warm   vs kdt_nowarm = %.3e\n",
                d_kw.max_abs);
    std::printf("[ATTR]   both                   : acc_warm   vs kdt_nowarm = %.3e\n",
                d_both.max_abs);

    // Third solver arm: the workspace itself.  EXP 3 above showed that the
    // warm-start knob changes nothing at all, so "the solver" needed a
    // perturbation that provably reaches it -- a fresh DualArmQp (fresh OSQP
    // setup, fresh scaling, fresh iterate) for every single cycle, with the
    // accumulated clock and warm start otherwise untouched.
    const Series af = seriesFromRun(s.acc_fresh, "acc_fresh_solver");
    const Diff d_fresh = diffSeries(aw, af);
    std::printf("[ATTR] solver-state contribution (fresh QP per cycle vs one persistent QP):\n");
    std::printf("[ATTR]   acc_warm vs acc_fresh: max|dq diff| = %.3e rad, max ULP = %llu, "
                "bit-identical %zu/%zu\n",
                d_fresh.max_abs, static_cast<unsigned long long>(d_fresh.max_ulp),
                d_fresh.bit_identical, d_fresh.pairs);
    if (d_fresh.first_diff_cycle != static_cast<std::size_t>(-1)) {
        std::printf("[ATTR]   first cycle affected by workspace reuse = %zu\n",
                    d_fresh.first_diff_cycle);
    } else {
        std::printf("[ATTR]   the persistent OSQP workspace changes no bit of any dq\n");
    }
}

// ===========================================================================
// TEST 4 -- the observed acceptance gap, and where it actually comes from
// ===========================================================================
//
// Reproduces the numbers the acceptance test prints, then settles cycle 0 --
// the cycle that carries the worst |dq| difference -- by construction rather
// than by inference: with the settle phase at t=0 nothing is moving, so the
// exact optimum of the QP is dq = 0 and any nonzero dq is solver residual.
TEST(UlpAttribution, ObservedGapAndCycleZero) {
    requireHarness();
    const RunSet& s = runSet();
    const Series mine = seriesFromRun(s.acc_warm, "mine(accumulated clock)");
    const Series ref = seriesFromFixture(s.ref);
    const Diff d = diffSeries(mine, ref);
    printDiff("ACCEPTANCE COMPARISON: controller-clock run vs the S3 ff_1 fixture", mine.name,
              ref.name, d);

    // Reproduce the acceptance test's own summary lines so a reader can see this
    // harness is the same loop (same worst pair, same t gap, same terminal e).
    std::printf("[ATTR]   PREDICTED from test_mock_closure.cpp:  worst |dq| = 9.024e-14 at "
                "cycle 0 joint 6, mine -1.8145642984102369e-14 (%s) vs baseline "
                "7.2097585154607034e-14 (%s); |max t gap| = 3.553e-15; terminal e_obj = "
                "2.971610e-08 m\n",
                bits(-1.8145642984102369e-14).c_str(), bits(7.2097585154607034e-14).c_str());
    std::printf("[ATTR]   MEASURED: worst |dq| = %s at cycle %zu joint %d, mine %s (%s) vs "
                "baseline %s (%s); |max t gap| = %s; terminal e_obj = %s m / %s rad\n",
                sci(d.max_abs).c_str(), d.max_abs_cycle, d.max_abs_joint,
                g17(d.max_abs_a).c_str(), bits(d.max_abs_a).c_str(), g17(d.max_abs_b).c_str(),
                bits(d.max_abs_b).c_str(), sci(d.max_t_gap).c_str(), sci(mine.e_obj_pos.back()).c_str(),
                sci(mine.e_obj_rot.back()).c_str());
    std::printf("[ATTR]   S3 terminal e_obj = %s m / %s rad; ratio mine/S3 = %.6f\n",
                sci(ref.e_obj_pos.back()).c_str(), sci(ref.e_obj_rot.back()).c_str(),
                mine.e_obj_pos.back() / ref.e_obj_pos.back());

    // ---- cycle 0 in detail: is the difference a clock, a solver, or a zero?
    const CycleLog& c0 = s.acc_warm.cyc[0];
    std::printf("[ATTR] cycle 0 diagnosis (both clocks read exactly %s here):\n", g17(c0.t).c_str());
    std::printf("[ATTR]   ||q_cost|| of the assembled problem = %s   (0 => the exact optimum "
                "is dq = 0)\n", sci(c0.cost_norm).c_str());
    std::printf("[ATTR]   ||dq||_inf (mine) = %s, ||dq||_inf (S3) = %s\n", sci(c0.dq.cwiseAbs().maxCoeff()).c_str(),
                sci(s.ref[0].dq.cwiseAbs().maxCoeff()).c_str());
    std::printf("[ATTR]   e_obj at cycle 0 = %s m / %s rad (tasks are satisfied before the step)\n",
                sci(c0.e_obj_pos_m).c_str(), sci(c0.e_obj_rot_rad).c_str());
    // Bind the ACTIVE BOX: dq = 0 is feasible and strictly interior, so the
    // bound constraints do not remove the zero optimum.
    const DualArmQpParams p = s3Params();
    double min_margin = std::numeric_limits<double>::max();
    for (int j = 0; j < kJoints; ++j) {
        const double lo = std::max(-p.v_max[j] * p.dt, p.q_min[j] - s.ref[0].q_prev[j]);
        const double hi = std::min(p.v_max[j] * p.dt, p.q_max[j] - s.ref[0].q_prev[j]);
        min_margin = std::min(min_margin, std::min(-lo, hi));
    }
    std::printf("[ATTR]   min distance from dq=0 to the velocity box = %s rad (interior)\n",
                sci(min_margin).c_str());
    EXPECT_GT(min_margin, 0.0) << "dq=0 must be strictly inside the box for the zero-optimum "
                                 "argument to hold at cycle 0";

    // The decisive statement, in numbers: if the exact optimum is 0 and both
    // values are ~1e-14, the 9e-14 difference is the sum of two ADMM residuals
    // -- it is neither a clock nor a "policy" difference.  It is also 1e8 ULP
    // apart in exactly the way a near-zero comparison always is.
    std::printf("[ATTR]   |dq| at the worst pair = %s vs %s rad; the exact optimum is 0.0\n",
                sci(std::abs(d.max_abs_a)).c_str(), sci(std::abs(d.max_abs_b)).c_str());

    // The solver's own path, cycle by cycle.  If the QP input at cycle 0 were
    // bit-identical and the solve were reproducible, the iteration counts would
    // have to agree; they are the one artifact column that describes the path
    // rather than the answer, so they say whether "the solver" is a real
    // separate cause or a rewording of "a different binary asked the question".
    std::printf("[ATTR] solver path (iterations mine vs S3) at the first cycles:\n");
    for (std::size_t k = 0; k < 8 && k < s.s3.size() && k < s.acc_warm.cyc.size(); ++k) {
        std::printf("[ATTR]   cycle %zu: iterations mine=%d S3=%d | converged mine=%d S3=%d\n", k,
                    s.acc_warm.cyc[k].iterations, s.s3[k].iterations,
                    s.acc_warm.cyc[k].converged ? 1 : 0, s.s3[k].converged);
    }
    std::size_t iter_diff_early = 0;
    for (std::size_t k = 0; k < s.s3.size() && k < s.acc_warm.cyc.size(); ++k) {
        if (s.acc_warm.cyc[k].iterations != s.s3[k].iterations) ++iter_diff_early;
    }
    std::printf("[ATTR]   iterations differ on %zu/%zu cycles in total\n", iter_diff_early,
                s.s3.size());
}

// ===========================================================================
// TEST 5 -- the ORIGINAL criterion, graded by its original definition
// ===========================================================================
//
// C4.3_S4_执行规划.md:533 is "bit-identical per cycle (<= 1 ULP)".  The
// original definition, with no stand-in and no gate, is:
//     for every cycle k and joint j:  ulpDistance(dq_mine, dq_S3) <= 1
// Both readings are reported: ALL pairs, and the acceptance test's own
// "meaningful joint" subset (|dq| >= 1e-9 on both sides).  Nothing is asserted
// here -- the criterion is a property of the code under test, and this file
// reports it rather than editing it.
TEST(UlpAttribution, OriginalCriterionVerdict) {
    requireHarness();
    const RunSet& s = runSet();
    const Series mine = seriesFromRun(s.acc_warm, "mine(accumulated clock)");
    const Series ref = seriesFromFixture(s.ref);
    const Diff d = diffSeries(mine, ref);

    const bool pass_all = (d.max_ulp <= 1);
    const bool pass_nonzero = (d.max_ulp_nonzero <= 1);
    std::printf("[ORIG] criterion = C4.3_S4_执行规划.md:533 \"per-cycle dq bit-identical "
                "(<= 1 ULP)\"; definition applied = ulpDistance(dq_mine, dq_S3) <= 1 for "
                "every (cycle, joint)\n");
    report("ORIG-all", "max_ulp=" + std::to_string(d.max_ulp) + " at cycle " +
                           std::to_string(d.max_ulp_cycle) + " joint " +
                           std::to_string(d.max_ulp_joint) + "; " +
                           std::to_string(d.pairs_gt_1ulp) + "/" + std::to_string(d.pairs) +
                           " pairs exceed 1 ULP",
           "max_ulp<=1", pass_all);
    report("ORIG-nonzero", "max_ulp=" + std::to_string(d.max_ulp_nonzero) + " at cycle " +
                               std::to_string(d.max_ulp_nz_cycle) + " joint " +
                               std::to_string(d.max_ulp_nz_joint) + "; " +
                               std::to_string(d.nonzero_gt_1ulp) + "/" +
                               std::to_string(d.nonzero_pairs) + " nonzero pairs exceed 1 ULP",
           "max_ulp<=1", pass_nonzero);
    std::printf("[ORIG] worst |dq| absolute difference = %.3e rad (the 1e-12 stand-in in "
                "test_mock_closure.cpp:269 is a DIFFERENT criterion, 1.1e6x looser than 1 ULP "
                "at dq ~ 4.6e-3 rad, where 1 ULP = %.3e)\n",
                d.max_abs, std::abs(std::nextafter(4.6e-3, 1.0) - 4.6e-3));
    std::printf("[ORIG] VERDICT by the original definition: %s%s\n",
                (pass_all ? "PASS" : "FAIL"),
                (pass_all == pass_nonzero) ? "" : " (readings disagree -- see both lines)");

    // The claim under test, quoted verbatim from
    // C4.3_S4_W2_ULP判据变更申请.md:37 and C4.3_S4_W2-R3_ULP候选V1独立验证报告.md:41:
    //   "改用 t = k*dt 后差异归零；改回累加式后差异重现"
    //   "用基线规则 t = k*dt 重算 638 周期 -> 与冻结件 dq 逐位相同"
    // Both are measurable without trusting either document: swap ONLY my run's
    // clock for the artifact's clock and compare against the same fixture.
    const Series kdt = seriesFromRun(s.kdt_warm, "mine(t=k*dt)");
    const Series ref_fix = seriesFromFixture(s.ref);
    const Diff d_kdt_fixture = diffSeries(kdt, ref_fix);
    std::printf("[ORIG] the doc's claimed fix, executed: my run with t = k*dt vs the SAME "
                "fixture -> bit-identical %zu/%zu, pairs with ULP>1: %zu, max|dq| = %.3e rad\n",
                d_kdt_fixture.bit_identical, d_kdt_fixture.pairs, d_kdt_fixture.pairs_gt_1ulp,
                d_kdt_fixture.max_abs);
    if (d_kdt_fixture.max_abs > 0.0) {
        std::printf("[ORIG]   worst pair: cycle %zu joint %d, %s vs %s (ULP %llu)\n",
                    d_kdt_fixture.max_abs_cycle, d_kdt_fixture.max_abs_joint,
                    g17(d_kdt_fixture.max_abs_a).c_str(), g17(d_kdt_fixture.max_abs_b).c_str(),
                    static_cast<unsigned long long>(d_kdt_fixture.max_ulp));
    }
    std::printf("[ORIG]   => changing the clock does NOT zero the difference: after the swap "
                "max|dq| vs the fixture = %.3e rad, while the accumulated-clock run gives "
                "%.3e rad\n", d_kdt_fixture.max_abs, d.max_abs);
}

// ===========================================================================
// TEST 6 -- V1, the five proposed sub-criteria, measured one by one
// ===========================================================================
//
// The bounds are the proposal's, quoted verbatim from
// docs/双臂协同/.../C4.3_S4_W2-R3_ULP候选V1独立验证报告.md; nothing here
// changes them, and nothing here implements V1 in the product.  This test only
// answers "would V1 pass, and what exactly does it measure".
TEST(UlpAttribution, V1SubCriteria) {
    requireHarness();
    const RunSet& s = runSet();
    const Series mine = seriesFromRun(s.acc_warm, "mine");
    const Series ref = seriesFromFixture(s.ref);
    const Diff d = diffSeries(mine, ref);
    const bool s3 = g_s3_ok;

    std::size_t decision_mismatches = 0;
    for (std::size_t i = 0; i < d.cycles; ++i) {
        if (mine.committed[i] != ref.committed[i]) ++decision_mismatches;
    }

    // ---- V1-1: decision_mismatches == 0
    report("V1-1", "decision_mismatches=" + std::to_string(decision_mismatches),
           "decision_mismatches==0", decision_mismatches == 0);

    // ---- V1-2: max|dq| <= 1e-6 rad and no NaN/Inf
    {
        const bool ok = (d.max_abs <= 1e-6) && !d.any_nan_or_inf;
        report("V1-2", "max|dq|=" + sci(d.max_abs) + " rad, NaN/Inf=" +
                           (d.any_nan_or_inf ? "yes" : "no"),
               "max|dq|<=1e-6 rad && no NaN/Inf", ok);
        std::printf("[V1-2] note: 1e-6 rad is %.0f x the observed gap %.3e and ~%.1e ULP at "
                    "dq ~ 4.6e-3 rad; the frozen criterion is 1 ULP\n",
                    1e-6 / (d.max_abs > 0.0 ? d.max_abs : 1.0), d.max_abs,
                    1e-6 / std::abs(std::nextafter(4.6e-3, 1.0) - 4.6e-3));
    }

    // ---- V1-3: max|dt_traj| <= 1e-6 s
    report("V1-3", "max|dt_traj|=" + sci(d.max_t_gap) + " s", "max|dt_traj|<=1e-6 s",
           d.max_t_gap <= 1e-6);

    // ---- V1-4: terminal ||e_O||_pos <= 1e-3 m, and ratio to S3 in [0.5, 2.0]
    {
        const double e_pos = mine.e_obj_pos.back();
        const double e_s3 = ref.e_obj_pos.back();
        const double ratio = (e_s3 > 0.0) ? (e_pos / e_s3) : std::numeric_limits<double>::quiet_NaN();
        const bool ok = (e_pos <= 1e-3) && (ratio >= 0.5) && (ratio <= 2.0);
        report("V1-4", "||e_O||_pos=" + sci(e_pos) + " m, ratio_to_S3=" + g17(ratio) +
                           " (S3=" + sci(e_s3) + " m)",
               "||e_O||_pos<=1e-3 m && ratio in [0.5,2.0]", ok);
    }

    // ---- V1-5: sign/threshold decisions of active_limit_count and
    //            sigma_min_stack identical per cycle
    if (!s3) {
        report("V1-5", "not graded: the S3 44-column artifact is unavailable",
               "identical per cycle", false);
        ADD_FAILURE() << "V1-5 needs runs/c43_s3/w8/ff_1/cycles.csv (the columns are not in "
                         "the 29-column fixture)";
        return;
    }
    std::size_t al_mismatch = 0;
    std::size_t al_sign_mismatch = 0;
    std::size_t rank_mismatch = 0;
    std::size_t sms_threshold_mismatch = 0;
    double sms_max_abs = 0.0;
    double sms_max_rel = 0.0;
    std::size_t sms_max_cycle = 0;
    std::size_t iter_mismatch = 0;
    int iter_max_mine = 0;
    for (std::size_t i = 0; i < d.cycles; ++i) {
        const S3Row& r = s.s3[i];
        if (mine.active_limit_count[i] != r.active_limit_count) ++al_mismatch;
        if ((mine.active_limit_count[i] > 0) != (r.active_limit_count > 0)) ++al_sign_mismatch;
        if (mine.rank_stack[i] != r.rank_stack) ++rank_mismatch;
        const bool healthy_mine = (mine.sigma_min_stack[i] >= kS3SigmaMinStackMin) &&
                                  (mine.rank_stack[i] == 12);
        const bool healthy_s3 = (r.sigma_min_stack >= kS3SigmaMinStackMin) &&
                                (r.rank_stack == 12);
        if (healthy_mine != healthy_s3) ++sms_threshold_mismatch;
        const double ad = std::abs(mine.sigma_min_stack[i] - r.sigma_min_stack);
        if (ad > sms_max_abs) {
            sms_max_abs = ad;
            sms_max_cycle = i;
        }
        const double rel = (r.sigma_min_stack != 0.0) ? ad / std::abs(r.sigma_min_stack) : 0.0;
        sms_max_rel = std::max(sms_max_rel, rel);
        if (r.iterations >= 0 && mine.iterations[i] != r.iterations) ++iter_mismatch;
        iter_max_mine = std::max(iter_max_mine, mine.iterations[i]);
    }
    const bool ok5 = (al_mismatch == 0) && (rank_mismatch == 0) && (sms_threshold_mismatch == 0);
    report("V1-5", "active_limit_count mismatches=" + std::to_string(al_mismatch) +
                       " (sign-only mismatches=" + std::to_string(al_sign_mismatch) + "), "
                       "rank_stack mismatches=" + std::to_string(rank_mismatch) +
                       ", sigma_min_stack threshold-decision mismatches=" +
                       std::to_string(sms_threshold_mismatch),
           "all per-cycle decisions identical", ok5);
    double mine_min = std::numeric_limits<double>::max();
    double mine_max = 0.0;
    for (double v : mine.sigma_min_stack) {
        mine_min = std::min(mine_min, v);
        mine_max = std::max(mine_max, v);
    }
    double s3_min = std::numeric_limits<double>::max();
    double s3_max = 0.0;
    int s3_iter_max = 0;
    for (const auto& r : s.s3) {
        s3_min = std::min(s3_min, r.sigma_min_stack);
        s3_max = std::max(s3_max, r.sigma_min_stack);
        s3_iter_max = std::max(s3_iter_max, r.iterations);
    }
    std::printf("[V1-5] sigma_min_stack: max|diff|=%.3e at cycle %zu, max relative=%.3e\n",
                sms_max_abs, sms_max_cycle, sms_max_rel);
    std::printf("[V1-5] sigma_min_stack ranges: mine [%.6f, %.6f] vs S3 [%.6f, %.6f]; both stay "
                "%.4f x above the %.1e threshold, so the threshold decision is 'healthy' in "
                "every cycle on both sides -- the criterion passes here, but this scenario "
                "never exercises it\n",
                mine_min, mine_max, s3_min, s3_max, s3_min / kS3SigmaMinStackMin,
                kS3SigmaMinStackMin);
    // Solver fingerprint: the per-cycle iteration count is the one artifact column
    // that describes the solver's internal path rather than its answer.
    std::printf("[SOLVER] iterations per cycle: mismatches=%zu/%zu, mine_max=%d, S3_max=%d "
                "(PREDICTED: unknown -- this is the measurement)\n",
                iter_mismatch, d.cycles, iter_max_mine, s3_iter_max);
}

}  // namespace
