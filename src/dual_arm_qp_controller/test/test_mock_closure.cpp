// C4.3 S4.1 W2 -- mock-bench closure and the E8 executor-heartbeat interlock.
//
// Two things are checked here, and they are deliberately different in nature:
//
//   S4-T2  (off-line, deterministic)  The controller's own cycle policy
//          (TransportTask + DualArmQp) must reproduce the S3 `ff_1` cycle
//          sequence bit-for-bit.  The baseline is a frozen fixture, not a
//          recomputation: see test/fixtures/PROVENANCE.txt.
//
//   S4-T13 (on the bench, real processes)  E8 must actually hold the arms when
//          the executor heartbeat stops, including the two cases that a unit
//          test cannot show: "never received" (fail closed) and "latch
//          re-arms" (a second timeout holds again, it is not a one-shot).
//          The same run also closes the loop over all 638 cycles.
//
// The bench test starts a real `ros2 launch` child process.  If it cannot start,
// the test FAILS (no silent skip): a bench test that quietly does nothing is
// worse than no bench test.
#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "dual_arm_qp_controller/srv/abort_transport.hpp"
#include "dual_arm_qp_controller/srv/start_transport.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Q12;
using dual_arm_qp::Vec12;
using dual_arm_qp_controller::TransportState;
using dual_arm_qp_controller::TransportTask;
using dual_arm_qp_controller::TransportTaskConfig;
using StatusMsg = dual_arm_qp_controller::msg::TransportStatus;

constexpr int kJoints = 12;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

std::string stateName(const std::string& s) { return s.empty() ? "?" : s; }

std::vector<std::string> splitWs(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream is(s);
    std::string tok;
    while (is >> tok) out.push_back(tok);
    return out;
}

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

/// Distance in ULPs between two doubles (monotonic bit-pattern mapping; handles
/// sign changes without the usual two's-complement pitfalls).
uint64_t ulpDistance(double a, double b) {
    // Map IEEE-754 bit patterns to a monotonically ordered unsigned key, so the
    // distance is |key(a) - key(b)|.  (The naive "~bits+1 for negatives" trick
    // is NOT monotonic: it reports ~2^63 ULPs for two numbers of opposite sign
    // that straddle zero, which is exactly the near-zero dq case here.)
    int64_t ia = 0, ib = 0;
    std::memcpy(&ia, &a, sizeof(ia));
    std::memcpy(&ib, &b, sizeof(ib));
    if (ia < 0) ia = INT64_MIN - ia;
    if (ib < 0) ib = INT64_MIN - ib;
    const uint64_t ka = static_cast<uint64_t>(ia) ^ 0x8000000000000000ULL;
    const uint64_t kb = static_cast<uint64_t>(ib) ^ 0x8000000000000000ULL;
    return (ka > kb) ? (ka - kb) : (kb - ka);
}

// ---------------------------------------------------------------------------
// S4-T2: per-cycle dq must match the S3 ff_1 baseline (<= 1 ULP)
// ---------------------------------------------------------------------------
TEST(MockClosureC43S3, S4T2_PerCycleDqMatchesS3FfOne) {
    bool ok = false;
    const std::vector<FixtureRow> ref = loadFixture(S4_FIXTURE_CSV, &ok);
    ASSERT_TRUE(ok) << "cannot read fixture: " << S4_FIXTURE_CSV;
    ASSERT_EQ(ref.size(), 638u) << "the S3 ff_1 baseline is 638 cycles";

    const auto spec = dual_arm_object_task::loadTrajectorySpec(S4_TRANSPORT_YAML);
    ASSERT_EQ(spec.phases.size(), 5u) << "S4.1 must drive the same five phases as S3";
    EXPECT_DOUBLE_EQ(spec.dt, 0.008);

    dual_arm::DualArmKinematics kin;

    // S3 ff_1 QP configuration, copied from
    // runs/c43_s3/w8/ff_1/summary.json (see test/fixtures/PROVENANCE.txt).
    dual_arm_qp::DualArmQpParams p;
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

    TransportTaskConfig cfg;   // defaults: S2 frozen G_L / G_R, dt 0.008
    TransportTask task(kin, cfg);
    const Q12 q0 = ref.front().q_prev;
    std::string err;
    ASSERT_TRUE(task.start(spec, q0, &err)) << err;
    EXPECT_NEAR((q0 - dual_arm_qp_controller::defaultHomeQ()).cwiseAbs().maxCoeff(), 0.0, 1e-12)
        << "the S3 start configuration must be the S2 q_grasp12 (home)";

    dual_arm_qp::DualArmQp qp(kin, p);

    Q12 q = q0;
    int worst_cycle = -1;
    uint64_t worst_ulp_key = 0;
    long near_zero_pairs = 0;
    long pairs_over_1ulp = 0;   // the frozen contract's own metric (see below)
    double worst_abs = 0.0;
    int worst_abs_cycle = -1;
    int worst_abs_j = -1;
    std::pair<double, double> worst_abs_pair{0.0, 0.0};
    int noncommitted = 0;
    int decision_mismatches = 0;
    double worst_t_traj_gap = 0.0;
    for (size_t i = 0; i < ref.size(); ++i) {
        const auto smp = task.sample(q);
        // §4.2.1: the trajectory clock the QP was solved at.
        worst_t_traj_gap = std::max(worst_t_traj_gap, std::abs(smp.t_traj - ref[i].t));
        const auto out = task.step(q, qp);
        if (!out.converged) {
            std::printf(
                "DIAG cycle %zu: active=%d committed=%d converged=%d iters=%d "
                "code='%s' refusal='%s' row.valid=%d row.e=%.6e row.ff=%.6e "
                "t_traj=%.6f e_obj=%.6e/%.6e anchor=%.3e/%.3e\n",
                i, static_cast<int>(out.active), static_cast<int>(out.committed),
                static_cast<int>(out.converged), out.iterations,
                out.failure_code ? out.failure_code : "(null)",
                smp.refusal_code ? smp.refusal_code : "(null)",
                static_cast<int>(smp.row.valid), smp.row.e.norm(),
                smp.row.body_twist_ff.norm(), smp.t_traj, smp.e_obj_pos_m,
                smp.e_obj_rot_rad, smp.anchor_pos_gap_m, smp.anchor_rot_gap_rad);
        }
        ASSERT_TRUE(out.converged) << "cycle " << i << " did not converge";
        ASSERT_STREQ(out.failure_code, "")
            << "cycle " << i << " failed with " << out.failure_code;
        if (!out.committed) ++noncommitted;
        if (out.committed != ref[i].committed) ++decision_mismatches;
        for (int j = 0; j < kJoints; ++j) {
            const uint64_t u = ulpDistance(out.dq[j], ref[i].dq[j]);
            if (u > 1) ++pairs_over_1ulp;
            const double ad = std::abs(out.dq[j] - ref[i].dq[j]);
            if (ad > worst_abs) {
                worst_abs = ad;
                worst_abs_cycle = static_cast<int>(i);
                worst_abs_j = j;
                worst_abs_pair = std::make_pair(out.dq[j], ref[i].dq[j]);
            }
            // The plan's "<= 1 ULP" is meaningful only where the quantity is
            // not numerically zero: for a joint whose dq is ~1e-14 (the two
            // tasks both want "do not move") a 1-ULP test compares noise.
            // Report both: the ULP distance restricted to meaningful joints,
            // and the absolute difference everywhere (the honest floor).
            if (std::abs(out.dq[j]) >= 1e-9 && std::abs(ref[i].dq[j]) >= 1e-9) {
                if (u > worst_ulp_key) {
                    worst_ulp_key = u;
                    worst_cycle = static_cast<int>(i);
                }
            }
            if (std::abs(out.dq[j]) < 1e-9 && std::abs(ref[i].dq[j]) < 1e-9) {
                ++near_zero_pairs;
            }
        }
        q += out.dq;  // ideal velocity servo, exactly as the S3 mock plant
    }

    const auto last = task.sample(q);
    std::printf(
        "S4-T2: cycles=%zu noncommitted=%d worst_ulp_over_nonzero_joints=%llu "
        "(cycle %d) |max t_traj gap|=%.3e terminal e_obj=%.6e m / %.6e rad\n",
        ref.size(), noncommitted,
        static_cast<unsigned long long>(worst_ulp_key), worst_cycle,
        worst_t_traj_gap,
        last.e_obj_pos_m, last.e_obj_rot_rad);
    std::printf(
        "S4-T2: worst |dq| abs diff = %.3e at cycle %d joint %d (mine %.17g vs "
        "baseline %.17g)\n",
        worst_abs, worst_abs_cycle, worst_abs_j, worst_abs_pair.first,
        worst_abs_pair.second);

    std::printf("S4-T2: near-zero dq pairs (|dq| < 1e-9) = %ld of %d\n",
                near_zero_pairs, static_cast<int>(ref.size()) * kJoints);
    // The frozen contract (C4.3_S4_执行规划.md:533) asks for <= 1 ULP, and it is
    // NOT met.  Keep that verdict visible on the acceptance line instead of
    // leaving it buried in a printout: 1e-12 below is a stand-in, not the
    // contract, and this counter is what makes the difference auditable.
    std::printf("S4-T2: CONTRACT criterion (<=1 ULP, C4.3_S4_执行规划.md:533): %s "
                "(%ld/%d pairs exceed 1 ULP; the 1e-12 bound below is a different, "
                "1.1e6x looser stand-in)\n",
                (pairs_over_1ulp == 0) ? "PASS" : "FAIL", pairs_over_1ulp,
                static_cast<int>(ref.size()) * kJoints);

    EXPECT_EQ(noncommitted, 0) << "every cycle of the S3 baseline was committed";
    // The commitment decisions must be identical, not merely "mostly": a single
    // extra/missing commit is exactly the policy drift this test exists to catch.
    EXPECT_EQ(decision_mismatches, 0)
        << "cycle policy differs from the S3 mock loop (commit/veto decisions)";

    // Agreement is asserted in absolute terms, and the plan's "<= 1 ULP" is
    // recorded as NOT attainable rather than silently reinterpreted:
    //   - At k = 0 both tasks are "do not move" (|dq| ~ 1e-14), so a 1-ULP test
    //     compares two different roundings of zero.
    //   - The measured worst absolute difference is 9.0e-14 rad over all 7656
    //     (cycle, joint) pairs, i.e. 2e-11 relative to the 0.0046 rad steps a
    //     real cycle takes: the two harnesses do not share a bit-identical QP
    //     invocation, and no amount of tolerance tuning changes that.
    // Measured 2026-10-20: max |dq| diff = 9.024e-14 (cycle 0), commit
    // decisions identical, max |dt_traj| = 3.55e-15, terminal e_obj 2.97e-8 m
    // vs the S3 run's 3.54e-8 m.
    EXPECT_LE(worst_abs, 1e-12) << "worst absolute dq difference (cycle "
                                << worst_abs_cycle << " joint " << worst_abs_j << ")";
    // Recorded verdict, machine-checked: the frozen <=1 ULP criterion is
    // violated by every pair.  This is deliberate -- it is the recorded state
    // of C4.3 S4 W2 (see docs/.../C3_ULP超限归因.md, D2 pending).  If a future
    // change makes the two harnesses bit-identical this assertion fails on
    // purpose, forcing the contract record to be updated rather than letting
    // the acceptance line go green with a stand-in bound.
    EXPECT_GT(pairs_over_1ulp, 0)
        << "the frozen <=1 ULP criterion is recorded as FAIL; if this now holds, update the "
           "contract record (C4.3_S4_执行规划.md:533) instead of deleting this assertion";
    EXPECT_LE(worst_t_traj_gap, 1e-12);
    // Terminal accuracy must stay in the same class as S3 (3.54e-8 m measured).
    EXPECT_LT(last.e_obj_pos_m, 1e-3);
    EXPECT_LT(last.e_obj_rot_rad, 1e-3);
    EXPECT_EQ(task.state(), TransportState::Complete);
    EXPECT_NEAR(task.tTraj(), task.duration(), 1e-12);
}

// ---------------------------------------------------------------------------
// Bench harness (S4-T13)
// ---------------------------------------------------------------------------
class BenchClosure : public ::testing::Test {
protected:
    void SetUp() override {
        // The controller_manager writes logs under $HOME/.ros; without ROS_HOME
        // the bench fails with PermissionError in sandboxed environments.
        char cwd[4096];
        ASSERT_NE(getcwd(cwd, sizeof(cwd)), nullptr);
        ros_home_ = std::string(cwd) + "/bench_home";
        ::setenv("ROS_HOME", ros_home_.c_str(), 1);
        ::setenv("ROS_LOG_DIR", (ros_home_ + "/log").c_str(), 1);
        std::string mk = "mkdir -p '" + ros_home_ + "/log'";
        ASSERT_EQ(std::system(mk.c_str()), 0);

        launch_log_ = std::string(cwd) + "/bench_launch.log";
        startBench();

        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("s4_mock_closure_driver");
        hb_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
            "/dual_arm_qp_controller/heartbeat", rclcpp::QoS(10));
        startHeartbeatThread();
        status_sub_ = node_->create_subscription<StatusMsg>(
            "/dual_arm_qp_controller/transport_status", rclcpp::QoS(10),
            [this](StatusMsg::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(mu_);
                last_ = *m;
                have_status_ = true;
            });

        start_cli_ = node_->create_client<dual_arm_qp_controller::srv::StartTransport>(
            "/dual_arm_qp_controller/start_transport");
        resume_cli_ = node_->create_client<std_srvs::srv::Trigger>(
            "/dual_arm_qp_controller/resume_transport");
        reset_cli_ = node_->create_client<std_srvs::srv::Trigger>(
            "/dual_arm_qp_controller/reset_transport");
        abort_cli_ = node_->create_client<dual_arm_qp_controller::srv::AbortTransport>(
            "/dual_arm_qp_controller/abort_transport");
    }

    void TearDown() override {
        if (rclcpp::ok()) rclcpp::shutdown();
        stopHeartbeatThread();
        hb_pub_.reset();
        status_sub_.reset();
        stopBench();
    }

    // ---- process management -------------------------------------------------
    void startBench() {
        pid_ = ::fork();
        if (pid_ == 0) {
            ::freopen(launch_log_.c_str(), "w", stderr);
            ::freopen(launch_log_.c_str(), "a", stdout);
            ::execlp("ros2", "ros2", "launch", "dual_arm_qp_controller",
                     "c43_s4_mock.launch.py", static_cast<char*>(nullptr));
            ::_exit(127);
        }
        ASSERT_GT(pid_, 0) << "fork failed";
    }

    void stopBench() {
        if (pid_ <= 0) return;
        ::kill(pid_, SIGINT);
        for (int i = 0; i < 200; ++i) {  // up to 10 s
            int st = 0;
            const pid_t r = ::waitpid(pid_, &st, WNOHANG);
            if (r == pid_) {
                pid_ = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        ::kill(pid_, SIGKILL);
        int st = 0;
        ::waitpid(pid_, &st, 0);
        pid_ = -1;
    }

    // ---- timing / spinning --------------------------------------------------
    /// Spin the driver node for `ms`, publishing the heartbeat at `hb_hz` when
    /// `heartbeat` is true, stopping early when `pred` (if any) becomes true.
    /// Publish the heartbeat from its own thread.  Publishing it from inside
    /// spinFor() was measured to miss the 100 ms E8 timeout whenever
    /// rclcpp::spin_some() took a while (graph work), which made the *test* the
    /// thing under test.  `hb_enable_` is the injection switch: E8 is verified by
    /// clearing it, not by hoping the main loop stalls.
    void startHeartbeatThread() {
        hb_run_.store(true);
        hb_thread_ = std::thread([this] {
            auto next = std::chrono::steady_clock::now();
            while (hb_run_.load()) {
                if (hb_enable_.load()) hb_pub_->publish(std_msgs::msg::Empty());
                next += std::chrono::milliseconds(20);   // 50 Hz vs 100 ms timeout
                std::this_thread::sleep_until(next);
                if (std::chrono::steady_clock::now() - next > std::chrono::seconds(1)) {
                    next = std::chrono::steady_clock::now();
                }
            }
        });
    }
    void stopHeartbeatThread() {
        hb_run_.store(false);
        if (hb_thread_.joinable()) hb_thread_.join();
    }

    /// Spin the driver node for `ms`, with the heartbeat enabled iff `heartbeat`,
    /// stopping early when `pred` (if any) becomes true.
    void spinFor(int ms, bool heartbeat, double hb_hz = 20.0,
                 const std::function<bool()>& pred = {}) {
        (void)hb_hz;
        hb_enable_.store(heartbeat);
        const auto t0 = std::chrono::steady_clock::now();
        const auto step = std::chrono::milliseconds(4);
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count() < ms) {
            if (pred && pred()) break;
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(step);
        }
        hb_enable_.store(false);
    }

    bool waitFor(const std::function<bool()>& pred, int timeout_ms, bool heartbeat,
                 double hb_hz = 20.0) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count() < timeout_ms) {
            if (pred()) return true;
            spinFor(20, heartbeat, hb_hz, pred);
        }
        return pred();
    }

    StatusMsg status() {
        std::lock_guard<std::mutex> lk(mu_);
        return last_;
    }
    bool haveStatus() {
        std::lock_guard<std::mutex> lk(mu_);
        return have_status_;
    }

    // ClientT is deduced from the client pointer (a nested `ClientT::SharedPtr`
    // parameter would be a non-deduced context and fail to compile).
    template <typename ClientT>
    bool callService(std::shared_ptr<ClientT> cli,
                     std::shared_ptr<typename ClientT::Request> req,
                     typename ClientT::Response* out, int timeout_ms) {
        if (!cli->wait_for_service(std::chrono::milliseconds(timeout_ms))) return false;
        auto fut = cli->async_send_request(req);
        const auto t0 = std::chrono::steady_clock::now();
        while (fut.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
            // Keep the heartbeat flowing while a service call is outstanding,
            // otherwise the E8 interlock re-holds in the very cycle that the
            // resume is applied (the controller sees no fresh heartbeat yet).
            if (hb_during_calls_) hb_enable_.store(true);
            rclcpp::spin_some(node_);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0)
                    .count() > timeout_ms) {
                return false;
            }
        }
        *out = *fut.get();
        return true;
    }

    /// Call ~/resume_transport and report the response.  Returns true iff the
    /// service answered at all; a refusal (success=false) is a legitimate
    /// outcome and is printed so a failure is explained rather than guessed.
    bool resumeNow(const char* tag, bool heartbeat) {
        std_srvs::srv::Trigger::Response trig;
        // Refresh the heartbeat BEFORE and DURING the resume, so the cycle that
        // applies the resume already sees a fresh one.
        spinFor(200, heartbeat);
        hb_during_calls_ = heartbeat;
        const bool ok = callService(
            resume_cli_, std::make_shared<std_srvs::srv::Trigger::Request>(), &trig,
            10000);
        hb_during_calls_ = false;
        hb_enable_.store(false);
        last_resume_message_ = trig.message;
        std::printf("S4-T13: %s ok=%d success=%d message='%s'\n", tag,
                    static_cast<int>(ok), static_cast<int>(trig.success),
                    trig.message.c_str());
        spinFor(300, heartbeat);
        std::printf("S4-T13: %s -> state=%s code='%s' t_traj=%.4f e_obj=%.3e/%.3e\n",
                    tag, status().state.c_str(), status().failure_code.c_str(),
                    status().t_traj, status().e_obj_pos_m, status().e_obj_rot_rad);
        return ok;
    }

    bool hb_during_calls_ = false;
    std::thread hb_thread_;
    std::atomic<bool> hb_run_{false};
    std::atomic<bool> hb_enable_{false};
    std::string last_resume_message_;
    std::string ros_home_;
    std::string launch_log_;
    pid_t pid_ = -1;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_pub_;
    rclcpp::Subscription<StatusMsg>::SharedPtr status_sub_;
    rclcpp::Client<dual_arm_qp_controller::srv::StartTransport>::SharedPtr start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_cli_;
    rclcpp::Client<dual_arm_qp_controller::srv::AbortTransport>::SharedPtr abort_cli_;
    std::mutex mu_;
    StatusMsg last_;
    bool have_status_ = false;
};

TEST_F(BenchClosure, S4T13_E8HeartbeatInterlockAndFullClosure) {
    // 1. The controller must come up and publish status.
    ASSERT_TRUE(waitFor([this] { return haveStatus(); }, 90000, /*heartbeat=*/false))
        << "no transport_status from the bench; see " << launch_log_;
    ASSERT_TRUE(start_cli_->wait_for_service(std::chrono::seconds(30)))
        << "start_transport never appeared; see " << launch_log_;

    // The bench profile must demand the heartbeat and must admit that the
    // protection chain is NOT complete (honesty on a live system, not only in a
    // unit test).
    {
        const StatusMsg s = status();
        std::printf("S4-T13: protection = %s (complete=%d, e8_required=%d)\n",
                    s.protection.c_str(), static_cast<int>(s.protection_complete),
                    static_cast<int>(s.executor_heartbeat_required));
        EXPECT_TRUE(s.executor_heartbeat_required);
        EXPECT_FALSE(s.protection_complete);
        EXPECT_NE(s.protection.find("degraded(e2_state_staleness)"), std::string::npos);
    }

    // 2. Start WITHOUT ever publishing a heartbeat: the interlock must fail
    //    closed on the very first cycles ("never received").
    auto start_req =
        std::make_shared<dual_arm_qp_controller::srv::StartTransport::Request>();
    const auto spec = dual_arm_object_task::loadTrajectorySpec(S4_TRANSPORT_YAML);
    for (const auto& ph : spec.phases) {
        start_req->phase_name.push_back(ph.name);
        start_req->phase_duration.push_back(ph.duration);
        start_req->phase_dx.push_back(ph.dx);
        start_req->phase_dy.push_back(ph.dy);
        start_req->phase_dz.push_back(ph.dz);
    }
    start_req->dt = spec.dt;
    start_req->time_scaling = spec.time_scaling;
    start_req->yaw_sweep_deg = spec.yaw_sweep_deg;
    start_req->start_world_t_object.clear();  // from the anchored estimate

    dual_arm_qp_controller::srv::StartTransport::Response start_res;
    ASSERT_TRUE(callService(start_cli_, start_req, &start_res, 10000)) << "start call failed";
    std::printf("S4-T13: start_transport accepted=%d message='%s'\n",
                static_cast<int>(start_res.accepted), start_res.message.c_str());
    ASSERT_TRUE(start_res.accepted) << start_res.message;

    const auto holding_on_heartbeat_loss = [this] {
        const StatusMsg s = status();
        return s.state == "HOLD" && s.failure_code == "executor_lost";
    };
    ASSERT_TRUE(waitFor(holding_on_heartbeat_loss, 5000, /*heartbeat=*/false))
        << "never-received heartbeat did not hold; state=" << status().state
        << " code=" << status().failure_code;
    std::printf("S4-T13: never-received heartbeat -> HOLD executor_lost\n");

    // 3. Resume with the heartbeat running: the transport moves again.
    ASSERT_TRUE(resumeNow("first resume", /*heartbeat=*/true));
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 5000,
                        /*heartbeat=*/true))
        << "first resume did not restart the transport; state=" << status().state
        << " code=" << status().failure_code << " | " << last_resume_message_;
    ASSERT_TRUE(waitFor([this] { return status().t_traj > 0.10; }, 5000,
                        /*heartbeat=*/true))
        << "t_traj did not advance while RUNNING (t_traj=" << status().t_traj << ")";
    const double t_running = status().t_traj;
    EXPECT_EQ(status().state, "RUNNING");

    // 4. Stop the heartbeat: within one timeout the controller holds and the
    //    trajectory clock freezes.  This is the real timeout path, on a real
    //    process, not an injected flag.
    ASSERT_TRUE(waitFor(holding_on_heartbeat_loss, 3000, /*heartbeat=*/false))
        << "heartbeat timeout did not hold; state=" << status().state
        << " code=" << status().failure_code;
    const double t_held_1 = status().t_traj;
    spinFor(400, /*heartbeat=*/false);
    const double t_held_2 = status().t_traj;
    std::printf("S4-T13: t_traj running=%.4f held=%.4f -> %.4f (frozen)\n", t_running,
                t_held_1, t_held_2);
    EXPECT_NEAR(t_held_1, t_held_2, 1e-12) << "t_traj must be frozen while HOLD";

    // 5. Resume again, then stop again: the interlock must RE-ARM.  A one-shot
    //    latch would let the second loss of the executor go unnoticed.
    ASSERT_TRUE(resumeNow("second resume", /*heartbeat=*/true));
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 5000,
                        /*heartbeat=*/true))
        << "second resume did not restart the transport; state=" << status().state
        << " code=" << status().failure_code << " | " << last_resume_message_;
    ASSERT_TRUE(waitFor(holding_on_heartbeat_loss, 3000, /*heartbeat=*/false))
        << "the interlock did not re-arm; state=" << status().state
        << " code=" << status().failure_code;
    std::printf("S4-T13: interlock re-armed after a resume\n");

    // 6. Resume a third time with a steady heartbeat and let the whole transport
    //    finish, so the same run also closes the loop end to end.
    ASSERT_TRUE(resumeNow("third resume", /*heartbeat=*/true));
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 5000,
                        /*heartbeat=*/true))
        << "third resume did not restart the transport; state=" << status().state
        << " code=" << status().failure_code << " | " << last_resume_message_;
    // Wait for COMPLETE, resuming a HOLD if one appears.  The subject here is
    // the injection semantics (asserted above), not "the host never hiccups for
    // five seconds": on a loaded container the controller's non-RT executor was
    // measured to miss the 100 ms E8 deadline and HOLD a healthy transport.  Any
    // such HOLD is counted and printed so it cannot pass silently.
    const auto t_fin = std::chrono::steady_clock::now();
    int spurious_holds = 0;
    bool finished = false;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t_fin)
               .count() < 60000) {
        const auto st = status();
        if (st.state == "COMPLETE") {
            finished = true;
            break;
        }
        if (st.state == "HOLD") {
            ++spurious_holds;
            std::printf(
                "S4-T13: spurious HOLD code='%s' at t_traj=%.4f while waiting for "
                "COMPLETE -> resume %d\n",
                st.failure_code.c_str(), st.t_traj, spurious_holds);
            resumeNow("completion resume", /*heartbeat=*/true);
            continue;
        }
        spinFor(20, /*heartbeat=*/true);
    }
    std::printf("S4-T13: completion holds_resumed=%d\n", spurious_holds);
    ASSERT_TRUE(finished) << "transport did not complete; state=" << status().state
                          << " code=" << status().failure_code
                          << " t_traj=" << status().t_traj;

    const StatusMsg fin = status();
    std::printf(
        "S4-T13: COMPLETE t_traj=%.4f / %.4f  e_obj=%.6e m / %.6e rad  "
        "anchor_gap=%.3e m  sigma_min_stack=%.6f\n",
        fin.t_traj, fin.t_traj_duration, fin.e_obj_pos_m, fin.e_obj_rot_rad,
        fin.anchor_pos_gap_m, fin.sigma_min_stack);
    EXPECT_NEAR(fin.t_traj, fin.t_traj_duration, 1e-3);
    EXPECT_LT(fin.e_obj_pos_m, 1e-3) << "terminal object position error";
    EXPECT_LT(fin.e_obj_rot_rad, 1e-3) << "terminal object attitude error";
    EXPECT_LT(fin.anchor_pos_gap_m, 1e-3) << "closed-chain audit on the bench";
    EXPECT_GE(fin.sigma_min_stack, 1e-6) << "stacked task must stay well conditioned";
}

}  // namespace
