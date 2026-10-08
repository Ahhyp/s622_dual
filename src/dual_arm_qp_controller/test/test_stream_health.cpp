// test/test_stream_health.cpp
// D7 interlock unit tests (ROS-free).

#include <gtest/gtest.h>

#include "dual_arm_qp_controller/stream_health_monitor.hpp"

using dual_arm_qp_controller::StreamHealthMonitor;
using dual_arm_qp_controller::StreamHealthParams;
using dual_arm_qp_controller::Vec12;

namespace {
Vec12 constant(double v) {
    Vec12 q;
    q.setConstant(v);
    return q;
}
}  // namespace

TEST(StreamHealth, HealthyWhenIdle) {
    StreamHealthMonitor m;
    m.configure(StreamHealthParams{});
    const auto h = m.evaluate(1.0, constant(0.1), constant(0.1), 0);
    EXPECT_TRUE(h.healthy);
    EXPECT_EQ(h.unhealthy_arm, -1);
}

TEST(StreamHealth, NonFiniteStateIsUnhealthy) {
    StreamHealthMonitor m;
    m.configure(StreamHealthParams{});
    Vec12 q = constant(0.1);
    q[3] = std::numeric_limits<double>::quiet_NaN();
    const auto h = m.evaluate(1.0, q, constant(0.1), 0);
    EXPECT_FALSE(h.healthy);
    EXPECT_EQ(h.unhealthy_arm, 0);
}

TEST(StreamHealth, InjectedStallHoldsBothArms) {
    StreamHealthParams p;
    p.stall_inject_arm = 1;  // right
    p.stall_start_s = 2.0;
    p.stall_duration_s = 1.0;
    StreamHealthMonitor m;
    m.configure(p);
    // before / after the window -> healthy
    EXPECT_TRUE(m.evaluate(1.0, constant(0.0), constant(0.0), 0).healthy);
    EXPECT_TRUE(m.evaluate(3.5, constant(0.0), constant(0.0), 0).healthy);
    // inside the window -> unhealthy, and it is the RIGHT arm that is flagged
    const auto h = m.evaluate(2.5, constant(0.0), constant(0.0), 0);
    EXPECT_FALSE(h.healthy);
    EXPECT_EQ(h.unhealthy_arm, 1);
}

TEST(StreamHealth, TrackingErrorFlagsTheAffectedArm) {
    StreamHealthParams p;
    p.tracking_error_limit = 0.05;
    StreamHealthMonitor m;
    m.configure(p);
    Vec12 q = constant(0.0);
    Vec12 q_cmd = constant(0.0);
    q[0] = 0.2;  // left arm joint 1 lags
    const auto h = m.evaluate(1.0, q, q_cmd, 0);
    EXPECT_FALSE(h.healthy);
    EXPECT_EQ(h.unhealthy_arm, 0);
}

TEST(StreamHealth, PersistentSolverFailureIsGlobal) {
    StreamHealthParams p;
    p.solve_failure_limit = 5;
    StreamHealthMonitor m;
    m.configure(p);
    EXPECT_TRUE(m.evaluate(1.0, constant(0.0), constant(0.0), 4).healthy);
    const auto h = m.evaluate(1.0, constant(0.0), constant(0.0), 5);
    EXPECT_FALSE(h.healthy);
    EXPECT_EQ(h.unhealthy_arm, -1);
}

TEST(StreamHealth, ResetClearsLastResult) {
    StreamHealthMonitor m;
    m.configure(StreamHealthParams{});
    Vec12 q = constant(0.0);
    q[0] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(m.evaluate(1.0, q, constant(0.0), 0).healthy);
    m.reset();
    EXPECT_TRUE(m.last().healthy);
}
