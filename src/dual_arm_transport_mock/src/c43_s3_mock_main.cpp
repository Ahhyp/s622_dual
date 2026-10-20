// C4.3 S3 -- W7: the executable that produces one §8 artifact directory.
//
// One directory == one run.  W8 sweeps by invoking this repeatedly, which keeps
// every summary.json self-consistent (its inputs/params are the ones that actually
// produced the numbers in that directory) instead of one giant file whose blocks
// have to be matched up by hand.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport_mock/log_writer.hpp"
#include "dual_arm_transport_mock/mock_loop.hpp"
#include "dual_arm_transport_mock/trajectory.hpp"
#include "dual_arm_transport_mock/version.hpp"

#ifndef C43_GIT_HEAD
#define C43_GIT_HEAD "unknown"
#endif
#ifndef C43_DEFAULT_URDF
#define C43_DEFAULT_URDF ""
#endif
#ifndef C43_DEFAULT_SRDF
#define C43_DEFAULT_SRDF ""
#endif
#ifndef C43_DEFAULT_S2_CONFIG
#define C43_DEFAULT_S2_CONFIG ""
#endif
#ifndef C43_DEFAULT_CONFIG
#define C43_DEFAULT_CONFIG ""
#endif

namespace {

using dual_arm::Vec6;
using dual_arm_pose_optimization::DualArmScene;
using dual_arm_pose_optimization::SceneLoadOptions;
using dual_arm_transport::GraspConfig;
using dual_arm_transport::TransportBaseline;
using dual_arm_transport::loadGraspConfig;
using dual_arm_transport_mock::MockLoopOptions;
using dual_arm_transport_mock::MockTransportLoop;
using dual_arm_transport_mock::RunSummary;
using dual_arm_transport_mock::ScenarioRun;
using dual_arm_transport_mock::TrajectorySpec;
using dual_arm_transport_mock::loadTrajectorySpec;

void usage() {
    std::printf(
        "c43_s3_mock_main -- C4.3 S3 mock dual-arm transport\n"
        "\n"
        "  --out DIR                required: artifact directory (created)\n"
        "  --urdf PATH              default: the shared dual-arm fixture\n"
        "  --srdf PATH              default: the shared dual-arm SRDF\n"
        "  --baseline PATH          default: S2's transport_baseline.yaml\n"
        "  --grasps PATH            default: S2's object_and_grasps.yaml\n"
        "  --trajectory PATH        default: S3's s3_transport_trajectory.yaml\n"
        "  --yaw-deg X              start-pose yaw (default: first sweep entry)\n"
        "  --scenario NAME          label written into runs.csv/summary.json\n"
        "  --w-obj X                object-task weight (0 disables the task)\n"
        "  --w-slack-obj X          object slack penalty (>=1e3 for a live task)\n"
        "  --kp-obj X               object-task gain [1/s]\n"
        "  --w-rel X                relative-task weight\n"
        "  --w-center X             step-size regulariser\n"
        "  --sweep-samples N        swept-check samples per edge\n"
        "  --min-clearance X        [m] clearance floor\n"
        "  --object-source S        left_anchor | right_anchor | desired\n"
        "  --no-ff                  disable the object feed-forward\n"
        "  --no-object-task         relative task only (the control arm)\n"
        "  --quiet                  do not print the per-run line\n"
        "  --help                   this text\n");
}

[[noreturn]] void fail(int code, const std::string& msg) {
    std::fprintf(stderr, "c43_s3_mock_main: %s\n", msg.c_str());
    std::exit(code);
}

double parseDouble(const char* flag, const char* text) {
    char* end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0') fail(2, std::string("bad number for ") + flag + ": " + text);
    return v;
}

long parseLong(const char* flag, const char* text) {
    char* end = nullptr;
    const long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') fail(2, std::string("bad integer for ") + flag + ": " + text);
    return v;
}

std::string readFileOrFail(const std::string& path) {
    std::ifstream f(path);
    if (!f) fail(2, "cannot read " + path);
    std::ostringstream os;
    os << f.rdbuf();
    return os.str();
}

}  // namespace

int main(int argc, char** argv) {
    std::string out_dir;
    std::string urdf = C43_DEFAULT_URDF;
    std::string srdf = C43_DEFAULT_SRDF;
    std::string baseline_path = std::string(C43_DEFAULT_S2_CONFIG) + "/transport_baseline.yaml";
    std::string grasps_path = std::string(C43_DEFAULT_S2_CONFIG) + "/object_and_grasps.yaml";
    std::string trajectory_path = std::string(C43_DEFAULT_CONFIG) + "/s3_transport_trajectory.yaml";
    std::string scenario = "default";
    bool has_yaw = false;
    double yaw_deg = 0.0;
    bool quiet = false;

    MockLoopOptions opt;
    // The production controller never wires the object task (W0 finding), so this
    // executable -- not the controller -- is what turns it on.
    opt.qp.kp_obj = 20.0;
    opt.qp.w_obj = 1.0;
    opt.qp.w_slack_obj = 1e3;
    opt.qp.w_center = 1e-4;  // measured: see the note in test_mock_loop_c43s3.cpp

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) fail(2, std::string(flag) + " needs a value");
            return argv[++i];
        };
        if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else if (a == "--out") {
            out_dir = next("--out");
        } else if (a == "--urdf") {
            urdf = next("--urdf");
        } else if (a == "--srdf") {
            srdf = next("--srdf");
        } else if (a == "--baseline") {
            baseline_path = next("--baseline");
        } else if (a == "--grasps") {
            grasps_path = next("--grasps");
        } else if (a == "--trajectory") {
            trajectory_path = next("--trajectory");
        } else if (a == "--scenario") {
            scenario = next("--scenario");
        } else if (a == "--yaw-deg") {
            yaw_deg = parseDouble("--yaw-deg", next("--yaw-deg"));
            has_yaw = true;
        } else if (a == "--w-obj") {
            opt.qp.w_obj = parseDouble("--w-obj", next("--w-obj"));
        } else if (a == "--w-slack-obj") {
            opt.qp.w_slack_obj = parseDouble("--w-slack-obj", next("--w-slack-obj"));
        } else if (a == "--kp-obj") {
            opt.qp.kp_obj = parseDouble("--kp-obj", next("--kp-obj"));
        } else if (a == "--w-rel") {
            opt.qp.w_rel = parseDouble("--w-rel", next("--w-rel"));
        } else if (a == "--w-center") {
            opt.qp.w_center = parseDouble("--w-center", next("--w-center"));
        } else if (a == "--sweep-samples") {
            opt.check.sweep_samples = static_cast<int>(parseLong("--sweep-samples",
                                                                next("--sweep-samples")));
        } else if (a == "--min-clearance") {
            opt.check.min_clearance_m = parseDouble("--min-clearance", next("--min-clearance"));
        } else if (a == "--object-source") {
            opt.check.object_source = next("--object-source");
        } else if (a == "--no-ff") {
            opt.use_object_feedforward = false;
        } else if (a == "--no-object-task") {
            opt.use_object_task = false;
            opt.qp.w_obj = 0.0;
        } else if (a == "--quiet") {
            quiet = true;
        } else {
            fail(2, "unknown flag: " + a + " (try --help)");
        }
    }
    if (out_dir.empty()) {
        usage();
        fail(2, "--out is required");
    }
    if (opt.use_object_task && opt.qp.w_obj > 0.0 && opt.qp.w_slack_obj < 1e2) {
        fail(2, "w_obj > 0 with w_slack_obj < 1e2 makes the object task toothless (S3-T5h)");
    }

    SceneLoadOptions so;
    so.urdf_path = urdf;
    so.srdf_path = srdf;
    so.require_meshes = true;
    std::shared_ptr<DualArmScene> scene;
    try {
        scene = DualArmScene::load(so);
    } catch (const std::exception& e) {
        fail(2, std::string("scene load failed: ") + e.what());
    }

    GraspConfig cfg;
    TransportBaseline baseline;
    TrajectorySpec spec;
    try {
        cfg = loadGraspConfig(grasps_path);
        baseline = TransportBaseline::fromYaml(readFileOrFail(baseline_path));
        spec = loadTrajectorySpec(trajectory_path);
    } catch (const std::exception& e) {
        fail(2, std::string("config load failed: ") + e.what());
    }
    if (spec.start_from_baseline) spec.start_world_T_object = baseline.world_T_object;
    if (!has_yaw && !spec.yaw_sweep_deg.empty()) yaw_deg = spec.yaw_sweep_deg.front();

    // Apply the yaw to the start pose exactly as the trajectory spec defines it.
    if (yaw_deg != 0.0) {
        const double rad = yaw_deg * M_PI / 180.0;
        dual_arm::Iso3 yaw = dual_arm::Iso3::Identity();
        yaw.linear() = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        spec.start_world_T_object = yaw * spec.start_world_T_object;
    }
    spec.dt = opt.qp.dt;

    MockTransportLoop loop(scene, cfg, baseline, spec, opt);
    std::vector<dual_arm_transport_mock::CycleRecord> log;
    const RunSummary sum = loop.run(&log);

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) fail(2, "cannot create " + out_dir + ": " + ec.message());

    try {
        using namespace dual_arm_transport_mock;
        writeTextFile(out_dir + "/cycles.csv", cyclesCsv(log));
        writeTextFile(out_dir + "/cycles.jsonl", cyclesJsonl(log));
        writeTextFile(out_dir + "/sweeps.csv", sweepsCsv(log));
        writeTextFile(out_dir + "/clearance.csv", clearanceCsv(log));

        ScenarioRun run;
        run.scenario = scenario;
        run.object_source = opt.check.object_source;
        run.note = opt.use_object_task ? (opt.use_object_feedforward ? "object task + ff"
                                                                     : "object task, no ff")
                                       : "control arm (object task off)";
        run.summary = sum;
        writeTextFile(out_dir + "/runs.csv", runsCsv(sum, scenario, opt.check.object_source));

        Provenance prov;
        prov.git_head = C43_GIT_HEAD;
        prov.package_version = packageVersion();
        prov.build_stamp = buildStamp();
        prov.files = {{"urdf", urdf},
                      {"srdf", srdf},
                      {"baseline", baseline_path},
                      {"grasps", grasps_path},
                      {"trajectory", trajectory_path}};
        writeTextFile(out_dir + "/summary.json", summaryJson({run}, spec, opt, prov));
        writeTextFile(out_dir + "/report.md", summaryReport({run}, spec, opt, prov));
    } catch (const std::exception& e) {
        fail(2, std::string("artifact write failed: ") + e.what());
    }

    if (!quiet) {
        std::printf("[ c43_s3 ] scenario=%s source=%s success=%d committed=%d vetoed=%d "
                    "failure=%s@%d terminal=%.3e m max_e_rel=%.3e m max_e_obj=%.3e\n",
                    scenario.c_str(), opt.check.object_source.c_str(), sum.success ? 1 : 0,
                    sum.committed_cycles, sum.vetoed_cycles, sum.failure_code.c_str(),
                    sum.failure_cycle, sum.terminal_pos_err_m, sum.max_e_rel_norm,
                    sum.max_e_obj_norm);
    }
    return sum.success ? 0 : 1;
}
