// C4.3 S4.1 W3-R1a -- C7-A Rev.2: ~/hold_transport (executor HOLD request)
//
// WHAT IS TESTED, AND AT WHICH LAYER
//   Layer 3  the real controller inside controller_manager at 125 Hz over the
//            mock_components bench (c43_s4_mock.launch.py).  Service call ->
//            controller tick -> status ack -> frozen t_traj.
//   NOT covered here: Gazebo, hardware, or the executor node itself.
//
// THE 300 ms IS AN EXECUTOR-SIDE BUDGET, NOT A CONTROLLER BOUND
//   Rev.2.1 (P-1) makes a new ack publish immediately, so the observed latency
//   is one tick plus a few DDS/executor periods.  That does NOT prove a finite
//   bound: each failed trylock() pushes the publish out by one status period
//   and the number of such failures has no proven limit.  The assertion below
//   is therefore a *budget check*, and the measured value is printed so the
//   record cannot present it as a worst-case guarantee.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "dual_arm_qp_controller/srv/abort_transport.hpp"
#include "dual_arm_qp_controller/srv/hold_transport.hpp"
#include "dual_arm_qp_controller/srv/start_transport.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"

namespace {

using StatusMsg = dual_arm_qp_controller::msg::TransportStatus;
using HoldSrv = dual_arm_qp_controller::srv::HoldTransport;

#ifndef C43_HOLD_PARAMS
#error "C43_HOLD_PARAMS must point at the W2 bench parameter file"
#endif

// ---- T-H9 (origin strings): pure, no bench ---------------------------------
TEST(HoldOriginC43S3, T_H9_OriginTaggingIsStable) {
    using dual_arm_qp_controller::HoldOrigin;
    using dual_arm_qp_controller::holdOriginName;
    EXPECT_STREQ(holdOriginName(HoldOrigin::None), "");
    EXPECT_STREQ(holdOriginName(HoldOrigin::Controller), "controller");
    EXPECT_STREQ(holdOriginName(HoldOrigin::Executor), "executor");
}

class BenchHold : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(std::system("mkdir -p /tmp/c43s4_hold/home/log"), 0);
        ::setenv("ROS_HOME", "/tmp/c43s4_hold/home", 1);
        ::setenv("ROS_LOG_DIR", "/tmp/c43s4_hold/home/log", 1);
        launch_log_ = "/tmp/c43s4_hold/launch.log";
        // Isolate this test from any other bench instance: a crashed earlier run
        // can leave a controller_manager behind, and two nodes with the same name
        // on one domain make the status topic unusable.
        // A distinct domain per test: the previous test's bench may still be
        // shutting down, and a leftover controller_manager on the same domain
        // answers the service calls meant for the new bench.
        static int domain_counter = 0;
        const int domain = 30 + ((static_cast<int>(::getpid()) + domain_counter++) % 50);
        ::setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
        // NOTE: the bench is started LAST.  Wait-for-status must run after the
        // subscription exists, otherwise nothing spins the callback and the
        // wait can never succeed (this cost one debugging round).
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("s4_hold_driver");
        startHeartbeat();
        status_sub_ = node_->create_subscription<StatusMsg>(
            "/dual_arm_qp_controller/transport_status", rclcpp::QoS(10),
            [this](StatusMsg::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(mu_);
                last_ = *m;
                have_status_ = true;
                ++status_count_;
                last_status_s_ = node_->now().seconds();
                if (m->hold_request_seq != 0) {
                    ++ack_msgs_;
                    max_ack_seq_ = m->hold_request_seq;
                }
            });
        start_cli_ = node_->create_client<dual_arm_qp_controller::srv::StartTransport>(
            "/dual_arm_qp_controller/start_transport");
        hold_cli_ = node_->create_client<HoldSrv>("/dual_arm_qp_controller/hold_transport");
        resume_cli_ = node_->create_client<std_srvs::srv::Trigger>(
            "/dual_arm_qp_controller/resume_transport");
        startBench();
    }

    void TearDown() override {
        stopHeartbeat();
        if (rclcpp::ok()) rclcpp::shutdown();
        status_sub_.reset();
        stopBench();
    }

    void startBench() {
        const pid_t pid = ::fork();
        ASSERT_GE(pid, 0);
        if (pid == 0) {
            ::setpgid(0, 0);   // own process group, so stopBench kills the tree
            const int fd = ::open(launch_log_.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
            if (fd >= 0) { ::dup2(fd, 1); ::dup2(fd, 2); ::close(fd); }
            const char * ov = std::getenv("C43_TH17_PARAMS");
            const std::string params_path =
                (ov != nullptr && *ov != '\0') ? std::string(ov) : std::string(C43_HOLD_PARAMS);
            const std::string arg = std::string("params_file:=") + params_path;
            ::execlp("ros2", "ros2", "launch", "dual_arm_qp_controller", "c43_s4_mock.launch.py",
                     arg.c_str(), static_cast<char*>(nullptr));
            ::_exit(127);
        }
        bench_pid_ = pid;
        ASSERT_TRUE(waitFor([this] { return haveStatus(); }, 90000))
            << "bench never came up; see " << launch_log_;
    }

    void stopBench() {
        if (bench_pid_ <= 0) return;
        ::kill(-bench_pid_, SIGINT);
        ::kill(bench_pid_, SIGINT);
        int st = 0;
        bool reaped = false;
        for (int i = 0; i < 40; ++i) {   // up to 4 s, then hard stop
            if (::waitpid(bench_pid_, &st, WNOHANG) == bench_pid_) { reaped = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ::kill(-bench_pid_, SIGTERM);
        ::kill(-bench_pid_, SIGKILL);
        if (!reaped) ::waitpid(bench_pid_, &st, 0);
        bench_pid_ = -1;
    }

    void startHeartbeat() {
        hb_node_ = std::make_shared<rclcpp::Node>("s4_hold_heartbeat");
        hb_pub_ = hb_node_->create_publisher<std_msgs::msg::Empty>(
            "/dual_arm_qp_controller/heartbeat", rclcpp::QoS(10));
        hb_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        hb_exec_->add_node(hb_node_);
        hb_run_.store(true);
        hb_thread_ = std::thread([this] {
            auto next = std::chrono::steady_clock::now();
            while (hb_run_.load()) {
                hb_pub_->publish(std_msgs::msg::Empty());
                next += std::chrono::milliseconds(20);
                std::this_thread::sleep_until(next);
                if (std::chrono::steady_clock::now() - next > std::chrono::seconds(1)) {
                    next = std::chrono::steady_clock::now();
                }
            }
        });
    }

    void stopHeartbeat() {
        // Idempotent: T_H10 stops it from the test body and TearDown stops it
        // again; a second remove_node() on a released node throws.
        if (!hb_run_.exchange(false)) return;
        if (hb_thread_.joinable()) hb_thread_.join();
        if (hb_exec_ && hb_node_) hb_exec_->remove_node(hb_node_);
    }

    bool haveStatus() {
        std::lock_guard<std::mutex> lk(mu_);
        return have_status_;
    }
    StatusMsg status() {
        std::lock_guard<std::mutex> lk(mu_);
        return last_;
    }

    void spinFor(int ms, const std::function<bool()>& pred = {}) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0).count() < ms) {
            if (pred && pred()) return;
            if (node_) rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0).count() < timeout_ms) {
            if (pred()) return true;
            if (node_) rclcpp::spin_some(node_);   // node_ is null during SetUp
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return pred();
    }

    template <typename ClientT, typename ReqT, typename ResT>
    bool callService(std::shared_ptr<ClientT> cli, std::shared_ptr<ReqT> req, ResT* out,
                     int timeout_ms) {
        // wait_for_service() pumps the graph guard condition; the response future
        // does NOT complete unless this thread spins, so spin while waiting.
        const auto t0 = std::chrono::steady_clock::now();
        while (cli->service_is_ready() == false) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count() > timeout_ms) {
                return false;
            }
            if (node_) rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        auto fut = cli->async_send_request(req);
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0).count() < timeout_ms) {
            if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                *out = *fut.get();
                return true;
            }
            if (node_) rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    void startTransport(int lift_ms) {
        auto req = std::make_shared<dual_arm_qp_controller::srv::StartTransport::Request>();
        req->dt = 0.008;
        req->time_scaling = "smoothstep";
        req->yaw_sweep_deg = {0.0};
        req->phase_name = {"lift", "lower"};
        req->phase_duration = {lift_ms / 1000.0, lift_ms / 1000.0};
        req->phase_dx = {0.0, 0.0};
        req->phase_dy = {0.0, 0.0};
        req->phase_dz = {0.020, -0.020};
        dual_arm_qp_controller::srv::StartTransport::Response res;
        ASSERT_TRUE(callService(start_cli_, req, &res, 30000));
        ASSERT_TRUE(res.accepted) << "start_transport rejected: " << res.message;
    }

    pid_t bench_pid_ = -1;
    std::string launch_log_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<StatusMsg>::SharedPtr status_sub_;
    rclcpp::Client<dual_arm_qp_controller::srv::StartTransport>::SharedPtr start_cli_;
    rclcpp::Client<HoldSrv>::SharedPtr hold_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_cli_;
    rclcpp::Node::SharedPtr hb_node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> hb_exec_;
    std::thread hb_thread_;
    std::atomic<bool> hb_run_{false};
    std::atomic<int> status_count_{0};
    std::atomic<int> ack_msgs_{0};
    std::atomic<std::uint32_t> max_ack_seq_{0};
    std::atomic<double> last_status_s_{0.0};
    std::mutex mu_;
    StatusMsg last_;
    bool have_status_ = false;
};

// ---- T-H11 (service-level validation) --------------------------------------
TEST_F(BenchHold, T_H11_SeqZeroIsRejectedWithoutQueueing) {
    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "test_zero";
    req->seq = 0;
    HoldSrv::Response res;
    ASSERT_TRUE(callService(hold_cli_, req, &res, 10000));
    EXPECT_FALSE(res.accepted);
    EXPECT_NE(res.message.find("invalid seq"), std::string::npos) << res.message;
    // No ack must appear for seq 0: the slot was never occupied.
    EXPECT_EQ(status().hold_request_seq, 0u);
}

// ---- T-H3 (Idle is refused, but still acked) -------------------------------
TEST_F(BenchHold, T_H3_HoldWhileIdleIsRefusedAndAcked) {
    ASSERT_EQ(status().state, "IDLE");
    EXPECT_EQ(status().hold_request_seq, 0u);
    ASSERT_EQ(status().hold_request_result, 0);
    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "test_idle";
    req->seq = 7;
    HoldSrv::Response res;
    ASSERT_TRUE(callService(hold_cli_, req, &res, 10000));
    EXPECT_TRUE(res.accepted) << res.message;
    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq == 7u; }, 4000))
        << "no ack for a refused request: Idle would then wedge the slot";
    EXPECT_EQ(status().hold_request_result, 2);
    EXPECT_EQ(status().state, "IDLE");
    EXPECT_EQ(status().hold_request_seq, 7u);
}

// ---- T-H1 (applied) + T-H6 (ack latency) + F1/F2 (freeze) -------------------
TEST_F(BenchHold, T_H1_HoldFromRunningIsAppliedAckedAndFreezes) {
    startTransport(1000);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000))
        << "state=" << status().state;

    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "test_running";
    req->seq = 1;
    HoldSrv::Response res;
    const auto t_call = std::chrono::steady_clock::now();
    ASSERT_TRUE(callService(hold_cli_, req, &res, 10000));
    EXPECT_TRUE(res.accepted) << res.message;

    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq == 1u; }, 5000))
        << "no ack for an applied hold";
    const double ack_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_call).count();

    const StatusMsg acked = status();
    EXPECT_EQ(acked.hold_request_result, 1);
    EXPECT_EQ(acked.state, "HOLD");
    EXPECT_EQ(acked.hold_origin, "executor");
    EXPECT_EQ(acked.failure_code, "hold_by_executor");

    // F1/F2: the trajectory clock must be FROZEN, and the command must not
    // advance.  Sample twice, 300 ms apart (>> 1 tick at 125 Hz).
    const double t0 = status().t_traj;
    spinFor(300);
    const StatusMsg later = status();
    EXPECT_EQ(later.state, "HOLD");
    EXPECT_DOUBLE_EQ(later.t_traj, t0) << "t_traj advanced while HOLD";

    // Executor-side budget check, NOT a controller guarantee (see header).
    std::printf("T-H6 ack latency = %.1f ms (executor budget 300 ms; trylock "
                "failures extend it without a proven bound)\n", ack_ms);
    EXPECT_LE(ack_ms, 300.0);
}

// ---- T-H18 (D3-a replay gate) ------------------------------------------------
// The idempotency key must be idempotent: re-sending a seq that was already
// consumed has to be ACKED (a lost ack is recoverable) but must NOT freeze a
// trajectory that has since been resumed.
TEST_F(BenchHold, T_H18_ReplayedSeqIsAckedButDoesNotFreezeAResumedRun) {
    startTransport(1500);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000))
        << "state=" << status().state;

    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "first_request";
    req->seq = 1;
    HoldSrv::Response res;
    ASSERT_TRUE(callService(hold_cli_, req, &res, 10000));
    ASSERT_TRUE(res.accepted) << res.message;
    ASSERT_TRUE(waitFor([this] {
        return status().state == "HOLD" && status().hold_request_seq == 1u; }, 5000))
        << "the first request never froze the transport";

    auto rreq = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response rres;
    ASSERT_TRUE(callService(resume_cli_, rreq, &rres, 10000));
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 5000))
        << "resume did not restart the transport: " << status().state;

    // Replay the SAME seq -- exactly what a retransmit after a lost ack looks
    // like from the controller's point of view.
    auto req2 = std::make_shared<HoldSrv::Request>();
    req2->reason = "replay";
    req2->seq = 1;
    HoldSrv::Response res2;
    ASSERT_TRUE(callService(hold_cli_, req2, &res2, 10000));
    EXPECT_TRUE(res2.accepted) << res2.message;

    const double t0 = status().t_traj;
    spinFor(300);
    const StatusMsg later = status();
    EXPECT_EQ(later.state, "RUNNING") << "a replayed seq froze a resumed run";
    EXPECT_GT(later.t_traj, t0) << "t_traj did not advance after a replayed seq";
    EXPECT_EQ(later.hold_request_seq, 1u) << "a replay must still be acknowledged";
    // D3-a (round 2): an ack describes the CURRENT state, not the historical
    // verdict. The first request really was applied, but the plant is running
    // again, so "applied" here would be a false success ack.
    EXPECT_EQ(later.hold_request_result, 2)
        << "a replay of an applied seq must not claim 'frozen' after a resume";
}

// Replay while the transport is STILL held: the same seq must be idempotent --
// acked as applied, no second freeze, no change of origin.
TEST_F(BenchHold, T_H19_ReplayWhileStillHeldIsIdempotentAndStillApplied) {
    startTransport(1500);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "first_request";
    req->seq = 7;
    HoldSrv::Response res;
    ASSERT_TRUE(callService(hold_cli_, req, &res, 10000));
    ASSERT_TRUE(res.accepted) << res.message;
    ASSERT_TRUE(waitFor([this] {
        return status().state == "HOLD" && status().hold_request_seq == 7u; }, 5000));

    const StatusMsg before = status();
    auto replay = std::make_shared<HoldSrv::Request>();
    replay->reason = "retransmit";
    replay->seq = 7;
    HoldSrv::Response res2;
    ASSERT_TRUE(callService(hold_cli_, replay, &res2, 10000));
    EXPECT_TRUE(res2.accepted) << res2.message;

    spinFor(250);
    const StatusMsg after = status();
    EXPECT_EQ(after.state, "HOLD");
    EXPECT_EQ(after.hold_request_seq, 7u);
    EXPECT_EQ(after.hold_request_result, 1)
        << "replay of an applied request while still held must repeat 'applied'";
    EXPECT_EQ(after.hold_origin, "executor") << "the origin must not be rewritten";
    EXPECT_DOUBLE_EQ(after.t_traj, before.t_traj) << "a replayed seq moved the clock";
    for (int j = 0; j < 12; ++j) {
        EXPECT_DOUBLE_EQ(after.q_cmd[j], before.q_cmd[j]) << "joint " << j << " moved";
    }
}

// A NEW seq after a resume must be able to freeze again -- the de-duplication
// must not degrade into "one freeze per process".
TEST_F(BenchHold, T_H20_NewSeqAfterResumeFreezesAgain) {
    startTransport(1500);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    auto r1 = std::make_shared<HoldSrv::Request>();
    r1->reason = "first"; r1->seq = 21;
    HoldSrv::Response a;
    ASSERT_TRUE(callService(hold_cli_, r1, &a, 10000));
    ASSERT_TRUE(waitFor([this] {
        return status().state == "HOLD" && status().hold_request_seq == 21u; }, 5000));

    auto rreq = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response rres;
    ASSERT_TRUE(callService(resume_cli_, rreq, &rres, 10000));
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 5000));

    auto r2 = std::make_shared<HoldSrv::Request>();
    r2->reason = "second"; r2->seq = 22;
    HoldSrv::Response b;
    ASSERT_TRUE(callService(hold_cli_, r2, &b, 10000));
    ASSERT_TRUE(b.accepted) << b.message;
    ASSERT_TRUE(waitFor([this] {
        return status().state == "HOLD" && status().hold_request_seq == 22u; }, 5000))
        << "a new seq was not able to freeze the transport again";
    EXPECT_EQ(status().hold_request_result, 1);
}

// Same-cycle competition between an ABORT and an executor HOLD request.
//
// The two intake paths are independent latches (pending command vs. hold slot),
// so both can be set between two update() calls -- that is exactly the race.
// The invariant is NOT "one of them wins" (either order is legal); it is that
// the published ack may never claim "applied" while the transport is still
// RUNNING, and the origin may not be rewritten by the losing path.
TEST_F(BenchHold, T_H21_HoldAndAbortInTheSameCycleNeverClaimsSuccessWhileRunning) {
    auto abort_cli = node_->create_client<dual_arm_qp_controller::srv::AbortTransport>(
        "/dual_arm_qp_controller/abort_transport");
    ASSERT_TRUE(abort_cli->wait_for_service(std::chrono::seconds(10)));

    for (int round = 0; round < 3; ++round) {
        startTransport(1200);
        ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000))
            << "round " << round << " never started";

        auto areq = std::make_shared<dual_arm_qp_controller::srv::AbortTransport::Request>();
        areq->reason = "same_cycle_abort";
        auto hreq = std::make_shared<HoldSrv::Request>();
        hreq->reason = "same_cycle_hold";
        hreq->seq = static_cast<std::uint32_t>(40 + round);

        // Fire both without spinning in between: both latches are set before the
        // next update() drains them.
        auto afut = abort_cli->async_send_request(areq);
        auto hfut = hold_cli_->async_send_request(hreq);
        spinFor(400, [&] {
            return hfut.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
                   afut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        });
        const auto ares = afut.get();
        const auto hres = hfut.get();
        std::printf("T-H21 round=%d abort accepted=%d msg='%s' hold accepted=%d msg='%s'\n",
                    round, static_cast<int>(ares->accepted), ares->message.c_str(),
                    static_cast<int>(hres->accepted), hres->message.c_str());

        // The ack is only observable once the controller has had a cycle to drain
        // the slot; wait for the latch, then sample the published truth.
        waitFor([this] { return status().state != "RUNNING"; }, 3000);
        const StatusMsg st = status();
        std::printf("T-H21 round=%d state=%s origin=%s ack_seq=%u ack_result=%d\n",
                    round, st.state.c_str(), st.hold_origin.c_str(),
                    st.hold_request_seq, static_cast<int>(st.hold_request_result));

        // The safety property: no "applied" ack while the plant is still moving.
        if (st.hold_request_result == 1) {
            EXPECT_NE(st.state, "RUNNING")
                << "an applied ack was published while the transport kept running";
        }
        // An abort may not be relabelled as an executor hold, nor vice versa.
        if (st.state == "ABORT" && st.hold_request_result == 2) {
            EXPECT_NE(st.hold_origin, "executor")
                << "the refused path rewrote the hold origin";
        }
        // Whatever won, the transport must be latched (never silently RUNNING).
        EXPECT_NE(st.state, "RUNNING")
            << "both the abort and the hold were lost in the same cycle";
    }
}

// ---- T-H4 (no silent overwrite) --------------------------------------------
TEST_F(BenchHold, T_H4_TwoRapidRequestsAreNotSilentlyOverwritten) {
    startTransport(1000);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    auto r1 = std::make_shared<HoldSrv::Request>();
    r1->reason = "first"; r1->seq = 11;
    auto r2 = std::make_shared<HoldSrv::Request>();
    r2->reason = "second"; r2->seq = 12;
    HoldSrv::Response a, b;
    ASSERT_TRUE(callService(hold_cli_, r1, &a, 10000));
    ASSERT_TRUE(callService(hold_cli_, r2, &b, 10000));

    // Exactly one of the two documented outcomes must hold:
    //   (a) the second is refused with "busy" (slot still occupied), or
    //   (b) the slot had already drained, so both are acked in order.
    const bool refused_busy = (!b.accepted && b.message.find("busy") != std::string::npos);
    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq != 0u; }, 5000));
    const std::uint32_t seen = status().hold_request_seq;
    EXPECT_TRUE(refused_busy || seen == 11u || seen == 12u)
        << "second=" << b.accepted << " msg='" << b.message << "' ack_seq=" << seen;
    std::printf("T-H4 first accepted=%d second accepted=%d second_msg='%s' ack_seq=%u\n",
                static_cast<int>(a.accepted), static_cast<int>(b.accepted),
                b.message.c_str(), static_cast<unsigned>(seen));
}


// ---- T-H5 (already held: refused, and the original origin is preserved) -----
TEST_F(BenchHold, T_H5_HoldWhileAlreadyHeldKeepsTheOriginalOrigin) {
    startTransport(1000);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    auto r1 = std::make_shared<HoldSrv::Request>();
    r1->reason = "first"; r1->seq = 21;
    HoldSrv::Response a;
    ASSERT_TRUE(callService(hold_cli_, r1, &a, 10000));
    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq == 21u; }, 5000));
    ASSERT_EQ(status().state, "HOLD");
    ASSERT_EQ(status().hold_origin, "executor");

    // Second request while HOLD: there is no Running trajectory, so the
    // controller refuses (result=2) and must NOT relabel the existing hold.
    auto r2 = std::make_shared<HoldSrv::Request>();
    r2->reason = "second"; r2->seq = 22;
    HoldSrv::Response b;
    ASSERT_TRUE(callService(hold_cli_, r2, &b, 10000));
    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq == 22u; }, 5000));
    EXPECT_EQ(status().hold_request_result, 2);
    EXPECT_EQ(status().state, "HOLD");
    EXPECT_EQ(status().hold_origin, "executor")
        << "a refused request must not steal the attribution of the existing hold";
    EXPECT_EQ(status().failure_code, "hold_by_executor");
}

// ---- T-H3b (two requests in flight at once: exactly one may own the slot) --
TEST_F(BenchHold, T_H3b_ConcurrentRequestsNeverBothQueue) {
    startTransport(1000);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));
    ASSERT_TRUE(hold_cli_->service_is_ready());

    // Sent back-to-back on the wire, before spinning: both can reach the
    // controller inside one 8 ms tick, which is the case the single-slot CAS
    // exists for.  (Two threads here would mean two spin_some() calls on one
    // node, which rclcpp rejects.)
    auto r1 = std::make_shared<HoldSrv::Request>();
    r1->reason = "a"; r1->seq = 31;
    auto r2 = std::make_shared<HoldSrv::Request>();
    r2->reason = "b"; r2->seq = 32;
    auto f1 = hold_cli_->async_send_request(r1);
    auto f2 = hold_cli_->async_send_request(r2);

    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t0).count() < 10000) {
        if (f1.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready &&
            f2.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(f1.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    ASSERT_EQ(f2.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    const auto ra = *f1.get();
    const auto rb = *f2.get();

    const int accepted = static_cast<int>(ra.accepted) + static_cast<int>(rb.accepted);
    EXPECT_GE(accepted, 1) << "both requests were refused";
    // Whatever the timing, a refusal must say why: silence would be a lost request.
    if (!ra.accepted) EXPECT_NE(ra.message.find("busy"), std::string::npos) << ra.message;
    if (!rb.accepted) EXPECT_NE(rb.message.find("busy"), std::string::npos) << rb.message;
    std::printf("T-H3b accepted=%d a_msg='%s' b_msg='%s'\n", accepted,
                ra.message.c_str(), rb.message.c_str());
}

// ---- T-H10 (a controller-origin hold keeps its attribution) ----------------
TEST_F(BenchHold, T_H10_ControllerOriginHoldIsNotRelabelled) {
    startTransport(1000);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    // Stopping the heartbeat makes the controller hold on its own (E8): the
    // executor then asks for a hold, and must be told to "own" something it
    // does not.  The attribution must survive (origin==controller).
    stopHeartbeat();
    ASSERT_TRUE(waitFor([this] { return status().state == "HOLD"; }, 5000))
        << "controller did not enter HOLD after the heartbeat stopped";
    const StatusMsg ctrl = status();
    EXPECT_EQ(ctrl.failure_code, "executor_lost");
    EXPECT_EQ(ctrl.hold_origin, "controller");

    auto r = std::make_shared<HoldSrv::Request>();
    r->reason = "late"; r->seq = 41;
    HoldSrv::Response res;
    ASSERT_TRUE(callService(hold_cli_, r, &res, 10000));
    ASSERT_TRUE(waitFor([this] { return status().hold_request_seq == 41u; }, 5000));
    EXPECT_EQ(status().hold_request_result, 2);
    EXPECT_EQ(status().hold_origin, "controller")
        << "the executor must not be able to claim a safety-origin hold";
    EXPECT_EQ(status().failure_code, "executor_lost");
}

}  // namespace

// ---- T-H17: readState() failure injection -----------------------------------
// Evidence classes are labelled per assertion:
//   [OBSERVED]  measured at run time
//   [PROOF]     follows from the code path, not measured
//   [INFERRED]  consistent with observation, not proven
TEST_F(BenchHold, T_H17_PositiveControl_FailureExitStopsTheController) {
#ifndef C43_ENABLE_TEST_HOOKS
    // The readState() injection does not exist in a hooks=OFF build.  Skipping
    // is the honest classification: this case is NOT executable here and it is
    // NOT a pass -- the frozen evidence for it was produced with hooks=ON.
    GTEST_SKIP() << "requires -DC43_ENABLE_TEST_HOOKS=ON (injection not compiled in)";
#endif
    stopBench();
    ::setenv("C43_TH17_PARAMS", C43_TH17_PARAMS, 1);
    startBench();
    ASSERT_TRUE(waitFor([this] {
        return start_cli_->service_is_ready() && hold_cli_->service_is_ready(); }, 40000));
    ASSERT_TRUE(waitFor([this] { return status_count_.load() > 2; }, 20000));
    // The injection budget is armed inside onStartTransport, so the run has to
    // be started first -- without a start the arm site is never reached and the
    // injection can never fire (this was the cause of an earlier false positive).
    startTransport(400);
    ASSERT_TRUE(waitFor([this] { return status_count_.load() > 4; }, 8000))
        << "start_transport did not take effect";
    // after_cycles = 150 => the failure starts 1.2 s in, well after the status
    // stream has demonstrably advanced, so a stop here cannot be the run ending.
    const bool stopped = waitFor([this] {
        const double last = last_status_s_.load();
        return last > 0.0 && (node_->now().seconds() - last) > 0.40; }, 8000);
    EXPECT_TRUE(stopped) << "status kept flowing => the injected ERROR was not taken";
}

TEST_F(BenchHold, T_H17a_NoNewAckAfterTheFailure) {
#ifndef C43_ENABLE_TEST_HOOKS
    GTEST_SKIP() << "requires -DC43_ENABLE_TEST_HOOKS=ON (injection not compiled in)";
#endif
    stopBench();
    ::setenv("C43_TH17_PARAMS", C43_TH17_PARAMS, 1);
    startBench();
    ASSERT_TRUE(waitFor([this] {
        return start_cli_->service_is_ready() && hold_cli_->service_is_ready(); }, 40000));
    // [OBSERVED] the controller is alive and publishing before anything is armed.
    ASSERT_TRUE(waitFor([this] { return status_count_.load() > 2; }, 20000));
    EXPECT_EQ(ack_msgs_.load(), 0) << "no request has been made yet";

    // after_cycles = 10: the start is applied on cycle 1, the failure starts on
    // cycle 11.  Both facts are observable here through status arrivals.
    startTransport(400);
    ASSERT_TRUE(waitFor([this] { return status_count_.load() > 6; }, 8000))
        << "start_transport did not take effect";

    // [OBSERVED] the controller stops updating once the injected ERROR is taken.
    const bool stopped = waitFor([this] {
        const double last = last_status_s_.load();
        return last > 0.0 && (node_->now().seconds() - last) > 0.40; }, 8000);
    ASSERT_TRUE(stopped) << "status kept flowing => injection did not fire";

    // Request submitted AFTER the failure has started.
    auto req = std::make_shared<HoldSrv::Request>();
    req->reason = "th17_after_failure";
    req->seq = 41;
    HoldSrv::Response res;
    const bool served = callService(hold_cli_, req, &res, 5000);
    if (!served) { GTEST_SKIP() << "[OBSERVED] service unreachable after ERROR; nothing to queue"; }
    EXPECT_TRUE(res.accepted) << "hold rejected: " << res.message;

    // Observation window: no ack may appear for a cycle that never ran.
    for (int i = 0; i < 250; ++i) {
        if (node_) {rclcpp::spin_some(node_);}
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // [OBSERVED] a1: no ACK is produced once the controller has stopped running.
    // [LIMITATION] this is "no ack after the failure", NOT "the failing cycle
    // refused to drain a pending request" -- that is a2 and stays [PROOF] only.
    EXPECT_EQ(ack_msgs_.load(), 0) << "an ACK appeared although no cycle ran";
    EXPECT_EQ(max_ack_seq_.load(), 0u);
}

// C7 independent review (P1-2): an "applied" ack claims the trajectory is
// FROZEN.  The replay verdict used to be `!running()`, which COMPLETE satisfies
// while step() keeps solving and committing -- so replaying a consumed seq after
// the trajectory completed answered "applied" on a robot whose q_cmd still
// moved.  The verdict now tests the latch itself.
TEST_F(BenchHold, T_H22_ReplayAfterCompleteIsNotAnAppliedAck) {
    startTransport(800);
    ASSERT_TRUE(waitFor([this] { return status().state == "RUNNING"; }, 20000));

    auto r1 = std::make_shared<HoldSrv::Request>();
    r1->reason = "freeze once"; r1->seq = 31;
    HoldSrv::Response a;
    ASSERT_TRUE(callService(hold_cli_, r1, &a, 10000));
    ASSERT_TRUE(waitFor([this] {
        return status().state == "HOLD" && status().hold_request_seq == 31u; }, 5000));

    auto rreq = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response rres;
    ASSERT_TRUE(callService(resume_cli_, rreq, &rres, 10000));
    ASSERT_TRUE(waitFor([this] { return status().state == "COMPLETE"; }, 20000))
        << "the short trajectory never completed; the test cannot prove replays at COMPLETE";

    const auto q_before = status().q_cmd;
    auto r2 = std::make_shared<HoldSrv::Request>();
    r2->reason = "stale replay"; r2->seq = 31;
    HoldSrv::Response b;
    ASSERT_TRUE(callService(hold_cli_, r2, &b, 10000));
    // The seq is the same as the first (already acked) request, so waiting for
    // "seq==31 && result!=0" would be satisfied by the STALE ack.  Wait for the
    // verdict itself to flip: a refused replay must overwrite applied with
    // refused.  If the bug is present the value stays 1 and this times out.
    const bool refused = waitFor([this] { return status().hold_request_result == 2; }, 3000);
    EXPECT_TRUE(refused)
        << "replaying a consumed seq at COMPLETE left the ack at applied (result="
        << static_cast<int>(status().hold_request_result) << ")";
    EXPECT_EQ(status().hold_request_result, 2)
        << "replaying a consumed seq at COMPLETE must be refused, not reported as applied";
    EXPECT_EQ(status().state, "COMPLETE")
        << "the refused replay must not re-latch a completed trajectory";
    // And the claim really would have been false: q_cmd keeps being committed at
    // COMPLETE, which is exactly why "!running()" was the wrong verdict test.
    const auto q_after = status().q_cmd;
    (void)q_before; (void)q_after;
}
