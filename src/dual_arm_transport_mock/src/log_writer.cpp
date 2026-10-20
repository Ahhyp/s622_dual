// C4.3 S3 -- W6: CSV emission.  No formatting surprises: everything numeric goes
// through %.17g so a log round-trips to the exact double that produced it.
#include "dual_arm_transport_mock/log_writer.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "dual_arm_transport_mock/sha256.hpp"

namespace dual_arm_transport_mock {

namespace {

/// Exact round-trip for doubles; the log is evidence, not a screenshot.
std::string g17(double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

std::string i2s(int v) { return std::to_string(v); }

std::string q12(const dual_arm::Q12& q) {
    std::string out;
    for (int i = 0; i < 12; ++i) {
        if (i != 0) out += ' ';
        out += g17(q[i]);
    }
    return out;
}

std::string csvField(const std::string& s) {
    // Only failure_detail and link names can contain commas, but quoting
    // unconditionally would break naive `cut -d,` readers, so quote on demand.
    if (s.find_first_of(",\"\n") == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string jstr(const std::string& s) { return "\"" + jsonEscape(s) + "\""; }

/// True when the whole string is one number (so it can be emitted unquoted).
bool looksNumeric(const std::string& v) {
    if (v.empty()) return false;
    char* end = nullptr;
    std::strtod(v.c_str(), &end);
    return end != nullptr && *end == '\0' && end != v.c_str();
}

/// JSON has no inf/nan; emitting them would produce a file no parser accepts.
std::string jnum(double v) { return std::isfinite(v) ? g17(v) : std::string("null"); }

/// 12 numbers as a JSON array.
std::string jq12(const dual_arm::Q12& q) {
    std::string out = "[";
    for (int i = 0; i < 12; ++i) {
        if (i != 0) out += ',';
        out += jnum(q[i]);
    }
    out += ']';
    return out;
}

std::string jvec(const std::vector<double>& v) {
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i != 0) out += ',';
        out += jnum(v[i]);
    }
    out += ']';
    return out;
}

void appendRow(std::string& out, const std::vector<std::string>& row) {
    for (std::size_t i = 0; i < row.size(); ++i) {
        if (i != 0) out += ',';
        out += csvField(row[i]);
    }
    out += '\n';
}

}  // namespace

const std::vector<std::string>& cycleColumnNames() {
    static const std::vector<std::string> kCols{
        "k", "t", "object_source", "committed", "failure_code",
        // state before the step, the step, and where it meant to go
        "q_prev", "dq", "q_cand",
        "e_pos_m", "e_rot_rad",
        "e_rel_pos_m", "e_rel_rot_rad",
        "e_obj_pos_m", "e_obj_rot_rad",
        "slack_rel_pos_m", "slack_rel_rot_rad",
        "slack_obj_pos_m", "slack_obj_rot_rad",
        "anchor_pos_gap_m", "anchor_rot_gap_rad",
        "closure_pos_m", "closure_rot_rad",
        "sigma_min_rel", "sigma_min_stack", "cond_stack", "rank_stack",
        "active_limit_count", "min_limit_margin_rad",
        "whole_robot_m", "arm_arm_m", "arm_proximal_m", "legacy_margin_rad",
        "converged", "iterations", "solve_time_ms",
        // the independent judge's own verdict on the candidate
        "state_code", "robot_world_m", "arm_arm_check_m", "in_collision",
        "min_clearance_m", "swept_ok",
        "sweep_tracked_ok", "sweep_tracked_min_clearance_m", "sweep_tracked_code",
        "world_T_object_desired_px", "world_T_object_desired_py", "world_T_object_desired_pz"};
    return kCols;
}

const std::vector<std::string>& runColumnNames() {
    static const std::vector<std::string> kCols{
        "scenario", "object_source", "success", "failure_code", "failure_detail",
        "failure_cycle", "steps", "duration_s", "committed_cycles", "vetoed_cycles",
        "max_e_rel_norm", "max_e_obj_norm", "max_s_rel_norm", "max_s_obj_norm",
        "max_anchor_pos_gap_m", "max_anchor_rot_gap_rad",
        "terminal_pos_err_m", "terminal_rot_err_rad",
        "min_clearance_m", "min_clearance_cycle", "min_clearance_links",
        "collision_cycles", "swept_failures", "solver_failures", "limit_saturation_cycles",
        "max_step_norm_rad", "legacy_margin_min_rad"};
    return kCols;
}

std::vector<std::string> cycleRow(const CycleRecord& r) {
    const dual_arm::Iso3& T = r.world_T_object_desired;
    return {i2s(r.k),
            g17(r.t),
            r.object_source,
            r.committed ? "1" : "0",
            r.failure_code,
            q12(r.q_prev),
            q12(r.dq),
            q12(r.q_cand),
            g17(r.e_pos_m),
            g17(r.e_rot_rad),
            g17(r.e_rel_pos_m),
            g17(r.e_rel_rot_rad),
            g17(r.e_obj_pos_m),
            g17(r.e_obj_rot_rad),
            g17(r.slack_rel_pos_m),
            g17(r.slack_rel_rot_rad),
            g17(r.slack_obj_pos_m),
            g17(r.slack_obj_rot_rad),
            g17(r.anchor_pos_gap_m),
            g17(r.anchor_rot_gap_rad),
            g17(r.closure_pos_m),
            g17(r.closure_rot_rad),
            g17(r.sigma_min_rel),
            g17(r.sigma_min_stack),
            g17(r.cond_stack),
            i2s(r.rank_stack),
            i2s(r.active_limit_count),
            g17(r.min_limit_margin_rad),
            g17(r.whole_robot_m),
            g17(r.arm_arm_m),
            g17(r.arm_proximal_m),
            g17(r.legacy_margin_rad),
            r.converged ? "1" : "0",
            i2s(r.iterations),
            g17(r.solve_time_ms),
            r.state_check.code,
            g17(r.state_check.robot_world),
            g17(r.state_check.arm_arm),
            r.state_check.in_collision ? "1" : "0",
            g17(r.state_check.min_clearance),
            r.swept_ok ? "1" : "0",
            r.sweep_tracked_ok ? "1" : "0",
            g17(r.sweep_tracked_min_clearance_m),
            r.sweep_tracked_code,
            g17(T.translation().x()),
            g17(T.translation().y()),
            g17(T.translation().z())};
}

std::string cyclesCsv(const std::vector<CycleRecord>& log) {
    std::string out = cyclesCsvHeader();
    for (const CycleRecord& r : log) appendRow(out, cycleRow(r));
    return out;
}

std::string cyclesCsvHeader() {
    std::string out;
    appendRow(out, cycleColumnNames());
    return out;
}

std::string runsCsvHeader() {
    std::string out;
    appendRow(out, runColumnNames());
    return out;
}

std::string sweepsCsvHeader() {
    std::string out;
    appendRow(out, {"k", "object_source", "ok", "code", "detail", "first_bad_sample",
                    "min_clearance_m"});
    return out;
}

std::vector<std::vector<std::string>> sweepRows(const CycleRecord& r) {
    std::vector<std::vector<std::string>> rows;
    rows.reserve(r.sweeps.size());
    for (const SweepCheck& sc : r.sweeps) {
        rows.push_back({i2s(r.k), sc.object_source, sc.ok ? "1" : "0", sc.code, sc.detail,
                        i2s(sc.first_bad_sample), g17(sc.min_clearance_m)});
    }
    return rows;
}

std::vector<std::string> runRow(const RunSummary& s, const std::string& scenario,
                                const std::string& object_source) {
    return {scenario,
            object_source,
            s.success ? "1" : "0",
            s.failure_code,
            s.failure_detail,
            i2s(s.failure_cycle),
            i2s(s.steps),
            g17(s.duration_s),
            i2s(s.committed_cycles),
            i2s(s.vetoed_cycles),
            g17(s.max_e_rel_norm),
            g17(s.max_e_obj_norm),
            g17(s.max_s_rel_norm),
            g17(s.max_s_obj_norm),
            g17(s.max_anchor_pos_gap_m),
            g17(s.max_anchor_rot_gap_rad),
            g17(s.terminal_pos_err_m),
            g17(s.terminal_rot_err_rad),
            g17(s.min_clearance_m),
            i2s(s.min_clearance_cycle),
            s.min_clearance_links,
            i2s(s.collision_cycles),
            i2s(s.swept_failures),
            i2s(s.solver_failures),
            i2s(s.limit_saturation_cycles),
            g17(s.max_step_norm_rad),
            g17(s.legacy_margin_min_rad)};
}

std::string runsCsv(const RunSummary& sum, const std::string& scenario,
                    const std::string& object_source) {
    std::string out = runsCsvHeader();
    appendRow(out, runRow(sum, scenario, object_source));
    return out;
}

void writeTextFile(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("cannot open for writing: " + path);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    f.flush();
    if (!f) throw std::runtime_error("write failed: " + path);
}

// ---------------------------------------------------------------- W7 artifacts

std::string clearanceCsvHeader() {
    return "k,whole_robot_m,arm_arm_m,arm_proximal_m,capsule_arm_arm_m,robot_world_m,"
           "min_clearance_m,link_a,link_b,object_source,swept_ok,sweep_min_clearance_m\n";
}

std::string clearanceCsv(const std::vector<CycleRecord>& log) {
    std::string out = clearanceCsvHeader();
    for (const CycleRecord& r : log) {
        double sweep_min = 0.0;
        for (const SweepCheck& sc : r.sweeps) {
            sweep_min = std::min(sweep_min == 0.0 ? sc.min_clearance_m : sweep_min,
                                 sc.min_clearance_m);
        }
        appendRow(out, {i2s(r.k), g17(r.whole_robot_m), g17(r.arm_arm_m), g17(r.arm_proximal_m),
                        g17(r.state_check.capsule_arm_arm), g17(r.state_check.robot_world),
                        g17(r.state_check.min_clearance), r.state_check.link_a,
                        r.state_check.link_b, r.object_source, r.swept_ok ? "1" : "0",
                        g17(sweep_min)});
    }
    return out;
}

std::string sweepsCsv(const std::vector<CycleRecord>& log) {
    std::string out = sweepsCsvHeader();
    for (const CycleRecord& r : log) {
        for (const std::vector<std::string>& row : sweepRows(r)) appendRow(out, row);
    }
    return out;
}

std::string cyclesJsonl(const std::vector<CycleRecord>& log) {
    std::string out;
    for (const CycleRecord& r : log) {
        const std::vector<std::string> cols = cycleColumnNames();
        const std::vector<std::string> vals = cycleRow(r);
        out += '{';
        for (std::size_t i = 0; i < cols.size(); ++i) {
            if (i != 0) out += ',';
            out += jstr(cols[i]) + ':';
            // The 12-wide fields are arrays, everything else was already numeric or
            // textual -- so re-derive the type from the record, not from the string.
            if (cols[i] == "q_prev") {
                out += jq12(r.q_prev);
            } else if (cols[i] == "dq") {
                out += jq12(r.dq);
            } else if (cols[i] == "q_cand") {
                out += jq12(r.q_cand);
            } else if (cols[i] == "committed" || cols[i] == "active_limit_count" ||
                       cols[i] == "rank_stack" || cols[i] == "iterations" || cols[i] == "k") {
                out += vals[i];
            } else if (looksNumeric(vals[i])) {
                out += vals[i];
            } else {
                // Anything that is not a number must be QUOTED.  The first version of
                // this branch used a hand-maintained list of textual columns and forgot
                // one, which emitted `"state_code":ok` -- a file no JSON parser accepts.
                // Deciding by content instead of by name cannot forget a column.
                out += jstr(vals[i]);
            }
        }
        out += "}\n";
    }
    return out;
}

std::string qpParamsJson(const dual_arm_qp::DualArmQpParams& p) {
    std::ostringstream o;
    o << "  \"dt\": " << jnum(p.dt) << ",\n"
      << "  \"kp_rel\": " << jnum(p.kp_rel) << ",\n"
      << "  \"w_rel\": " << jnum(p.w_rel) << ",\n"
      << "  \"w_slack\": " << jnum(p.w_slack) << ",\n"
      << "  \"w_center\": " << jnum(p.w_center) << ",\n"
      << "  \"w_reg\": " << jnum(p.w_reg) << ",\n"
      << "  \"kp_obj\": " << jnum(p.kp_obj) << ",\n"
      << "  \"w_obj\": " << jnum(p.w_obj) << ",\n"
      << "  \"w_slack_obj\": " << jnum(p.w_slack_obj) << ",\n"
      << "  \"w_null_margin\": " << jnum(p.w_null_margin) << ",\n"
      << "  \"w_null_manip\": " << jnum(p.w_null_manip) << ",\n"
      << "  \"w_null_clear\": " << jnum(p.w_null_clear) << ",\n"
      << "  \"null_step\": " << jnum(p.null_step) << ",\n"
      << "  \"enable_singularity\": " << (p.enable_singularity ? "true" : "false") << ",\n"
      << "  \"sigma_critical\": " << jnum(p.sigma_critical) << ",\n"
      << "  \"sigma_safe\": " << jnum(p.sigma_safe) << ",\n"
      << "  \"damp_max\": " << jnum(p.damp_max) << ",\n"
      << "  \"slack_relax\": " << jnum(p.slack_relax) << ",\n"
      << "  \"vel_scale_min\": " << jnum(p.vel_scale_min) << ",\n"
      << "  \"e_safe\": " << jnum(p.e_safe) << ",\n"
      << "  \"e_critical\": " << jnum(p.e_critical) << ",\n"
      << "  \"q_min\": " << jvec(std::vector<double>(p.q_min.data(), p.q_min.data() + 12))
      << ",\n"
      << "  \"q_max\": " << jvec(std::vector<double>(p.q_max.data(), p.q_max.data() + 12))
      << ",\n"
      << "  \"v_max\": " << jvec(std::vector<double>(p.v_max.data(), p.v_max.data() + 12))
      << ",\n"
      << "  \"dq_center\": " << jvec(std::vector<double>(p.dq_center.data(),
                                                         p.dq_center.data() + 12))
      << ",\n"
      << "  \"C_ineq_rows\": " << p.C_ineq.rows() << ",\n"
      << "  \"C_ineq_cols\": " << p.C_ineq.cols() << ",\n"
      << "  \"d_ineq_size\": " << p.d_ineq.size() << "\n";
    return o.str();
}

std::string checkOptionsJson(const CheckOptions& c) {
    std::ostringstream o;
    o << "  \"min_clearance_m\": " << jnum(c.min_clearance_m) << ",\n"
      << "  \"pad_clearance_m\": " << jnum(c.pad_clearance_m) << ",\n"
      << "  \"sweep_samples\": " << c.sweep_samples << ",\n"
      << "  \"swept_check_enabled\": " << (c.swept_check_enabled ? "true" : "false") << ",\n"
      << "  \"object_source\": " << jstr(c.object_source) << ",\n"
      << "  \"grasp_contact_links\": [";
    for (std::size_t i = 0; i < c.grasp_contact_links.size(); ++i) {
        if (i != 0) o << ", ";
        o << jstr(c.grasp_contact_links[i]);
    }
    o << "]\n";
    return o.str();
}

std::string summaryJson(const std::vector<ScenarioRun>& runs, const TrajectorySpec& spec,
                        const MockLoopOptions& opt, const Provenance& prov) {
    std::ostringstream o;
    o << "{\n";
    o << "  \"package\": \"dual_arm_transport_mock\",\n";
    o << "  \"version\": " << jstr(prov.package_version) << ",\n";
    o << "  \"build_stamp\": " << jstr(prov.build_stamp) << ",\n";
    o << "  \"git_head\": " << jstr(prov.git_head) << ",\n";
    o << "  \"inputs\": {\n";
    for (std::size_t i = 0; i < prov.files.size(); ++i) {
        const auto& f = prov.files[i];
        o << "    " << jstr(f.first) << ": {\"path\": " << jstr(f.second)
          << ", \"sha256\": " << jstr(sha256FileHex(f.second)) << "}";
        o << (i + 1 == prov.files.size() ? "\n" : ",\n");
    }
    o << "  },\n";
    o << "  \"trajectory\": {\n"
      << "    \"dt\": " << jnum(spec.dt) << ",\n"
      << "    \"time_scaling\": " << jstr(spec.time_scaling) << ",\n"
      << "    \"start_from_baseline\": " << (spec.start_from_baseline ? "true" : "false")
      << ",\n"
      << "    \"yaw_sweep_deg\": " << jvec(spec.yaw_sweep_deg) << ",\n"
      << "    \"phases\": [";
    for (std::size_t i = 0; i < spec.phases.size(); ++i) {
        const TrajectoryPhase& p = spec.phases[i];
        if (i != 0) o << ", ";
        o << "{\"name\": " << jstr(p.name) << ", \"duration\": " << jnum(p.duration)
          << ", \"dx\": " << jnum(p.dx) << ", \"dy\": " << jnum(p.dy)
          << ", \"dz\": " << jnum(p.dz) << "}";
    }
    o << "]\n  },\n";
    o << "  \"loop_options\": {\n"
      << "    \"use_object_feedforward\": " << (opt.use_object_feedforward ? "true" : "false")
      << ",\n"
      << "    \"use_object_task\": " << (opt.use_object_task ? "true" : "false") << ",\n"
      << "    \"enforce_rank_precheck\": " << (opt.enforce_rank_precheck ? "true" : "false")
      << ",\n"
      << "    \"sigma_min_stack_min\": " << jnum(opt.sigma_min_stack_min) << ",\n"
      << "    \"s_rel_max\": " << jnum(opt.s_rel_max) << ",\n"
      << "    \"s_obj_max\": " << jnum(opt.s_obj_max) << ",\n"
      << "    \"terminal_pos_tol_m\": " << jnum(opt.terminal_pos_tol_m) << ",\n"
      << "    \"terminal_rot_tol_rad\": " << jnum(opt.terminal_rot_tol_rad) << ",\n"
      << "    \"anchor_pos_gap_max_m\": " << jnum(opt.anchor_pos_gap_max_m) << ",\n"
      << "    \"anchor_rot_gap_max_rad\": " << jnum(opt.anchor_rot_gap_max_rad) << ",\n"
      << "    \"solver_fail_streak_max\": " << opt.solver_fail_streak_max << ",\n"
      << "    \"limit_saturation_streak_max\": " << opt.limit_saturation_streak_max << ",\n"
      << "    \"object_infeasible_streak_max\": " << opt.object_infeasible_streak_max << "\n"
      << "  },\n";
    o << "  \"qp_params\": {\n" << qpParamsJson(opt.qp) << "  },\n";
    o << "  \"check_options\": {\n" << checkOptionsJson(opt.check) << "  },\n";
    o << "  \"runs\": [\n";
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const ScenarioRun& r = runs[i];
        const RunSummary& s = r.summary;
        o << "    {\"scenario\": " << jstr(r.scenario) << ", \"object_source\": "
          << jstr(r.object_source) << ", \"note\": " << jstr(r.note) << ",\n"
          << "     \"success\": " << (s.success ? "true" : "false")
          << ", \"failure_code\": " << jstr(s.failure_code)
          << ", \"failure_detail\": " << jstr(s.failure_detail)
          << ", \"failure_cycle\": " << s.failure_cycle << ",\n"
          << "     \"steps\": " << s.steps << ", \"duration_s\": " << jnum(s.duration_s)
          << ", \"committed_cycles\": " << s.committed_cycles
          << ", \"vetoed_cycles\": " << s.vetoed_cycles << ",\n"
          << "     \"max_e_rel_norm\": " << jnum(s.max_e_rel_norm)
          << ", \"max_e_obj_norm\": " << jnum(s.max_e_obj_norm)
          << ", \"max_s_rel_norm\": " << jnum(s.max_s_rel_norm)
          << ", \"max_s_obj_norm\": " << jnum(s.max_s_obj_norm) << ",\n"
          << "    \"max_anchor_pos_gap_m\": " << jnum(s.max_anchor_pos_gap_m) << ",\n"
          << "    \"max_anchor_rot_gap_rad\": " << jnum(s.max_anchor_rot_gap_rad) << ",\n"
          << "     \"terminal_pos_err_m\": " << jnum(s.terminal_pos_err_m)
          << ", \"terminal_rot_err_rad\": " << jnum(s.terminal_rot_err_rad) << ",\n"
          << "     \"min_clearance_m\": " << jnum(s.min_clearance_m)
          << ", \"min_clearance_cycle\": " << s.min_clearance_cycle
          << ", \"min_clearance_links\": " << jstr(s.min_clearance_links) << ",\n"
          << "     \"collision_cycles\": " << s.collision_cycles
          << ", \"swept_failures\": " << s.swept_failures
          << ", \"solver_failures\": " << s.solver_failures
          << ", \"limit_saturation_cycles\": " << s.limit_saturation_cycles << ",\n"
          << "     \"max_step_norm_rad\": " << jnum(s.max_step_norm_rad)
          << ", \"legacy_margin_min_rad\": " << jnum(s.legacy_margin_min_rad) << "}";
        o << (i + 1 == runs.size() ? "\n" : ",\n");
    }
    o << "  ]\n";
    o << "}\n";
    return o.str();
}

std::string summaryReport(const std::vector<ScenarioRun>& runs, const TrajectorySpec& spec,
                          const MockLoopOptions& opt, const Provenance& prov) {
    std::ostringstream o;
    o << "# C4.3 S3 mock transport -- run report\n\n";
    o << "Generated by `c43_s3_mock_main` from the same values written to `summary.json`.\n";
    o << "Do not edit by hand; re-run the executable instead.\n\n";
    o << "- package: `dual_arm_transport_mock` " << prov.package_version << "\n";
    o << "- git HEAD: `" << prov.git_head << "`\n";
    o << "- build stamp: `" << prov.build_stamp << "`\n";
    o << "- dt: " << spec.dt << " s, time scaling: `" << spec.time_scaling << "`\n\n";

    o << "## Inputs\n\n| input | path | SHA-256 |\n|---|---|---|\n";
    for (const auto& f : prov.files) {
        o << "| " << f.first << " | `" << f.second << "` | `" << sha256FileHex(f.second)
          << "` |\n";
    }

    o << "\n## Trajectory\n\n| phase | duration [s] | dx | dy | dz |\n|---|---|---|---|---|\n";
    double total = 0.0;
    for (const TrajectoryPhase& p : spec.phases) {
        total += p.duration;
        o << "| " << p.name << " | " << p.duration << " | " << p.dx << " | " << p.dy << " | "
          << p.dz << " |\n";
    }
    o << "\nScheduled duration " << total << " s (" << static_cast<long>(total / spec.dt + 0.5)
      << " cycles at dt=" << spec.dt << " s).\n";

    o << "\n## Runs\n\n| scenario | verdict | failure | steps | committed | vetoed | "
         "terminal pos [m] | terminal rot [rad] | max e_rel | max e_obj | "
         "min clearance [m] |\n";
    o << "|---|---|---|---|---|---|---|---|---|---|---|\n";
    for (const ScenarioRun& r : runs) {
        const RunSummary& s = r.summary;
        o << "| `" << r.scenario << "` | " << (s.success ? "**pass**" : "**fail**") << " | `"
          << s.failure_code << "`" << (s.failure_cycle >= 0 ? " @" + std::to_string(s.failure_cycle)
                                                            : std::string())
          << " | " << s.steps << " | " << s.committed_cycles << " | " << s.vetoed_cycles << " | "
          << s.terminal_pos_err_m << " | " << s.terminal_rot_err_rad << " | " << s.max_e_rel_norm
          << " | " << s.max_e_obj_norm << " | " << s.min_clearance_m << " |\n";
    }

    o << "\n## Verdict against the acceptance tolerances\n\n";
    o << "terminal position <= " << opt.terminal_pos_tol_m << " m, terminal rotation <= "
      << opt.terminal_rot_tol_rad << " rad, relative closure <= " << opt.s_rel_max
      << ", object slack <= " << opt.s_obj_max << ", clearance floor "
      << opt.check.min_clearance_m << " m.\n\n";
    for (const ScenarioRun& r : runs) {
        const RunSummary& s = r.summary;
        o << "- `" << r.scenario << "`: ";
        if (s.success) {
            o << "pass -- terminal " << s.terminal_pos_err_m << " m / "
              << s.terminal_rot_err_rad << " rad, " << s.committed_cycles << " committed cycles";
            if (s.vetoed_cycles > 0) o << ", " << s.vetoed_cycles << " refused (never executed)";
            o << ".\n";
        } else {
            o << "fail -- `" << s.failure_code << "` at cycle " << s.failure_cycle;
            if (!s.failure_detail.empty()) o << ": " << s.failure_detail;
            o << ".\n";
        }
    }
    if (!runs.empty()) {
        o << "\nThe numbers above are the ones the controller actually produced; the "
             "per-cycle evidence is in `cycles.csv`, `cycles.jsonl`, `sweeps.csv` and "
             "`clearance.csv`.\n";
    }
    return o.str();
}

}  // namespace dual_arm_transport_mock
