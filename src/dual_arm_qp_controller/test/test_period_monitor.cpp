// test/test_period_monitor.cpp  (C2.R2)

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "dual_arm_qp_controller/period_monitor.hpp"

using dual_arm_qp_controller::PeriodMonitor;

TEST(PeriodMonitor, NominalPeriodIsNotAFault) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    for (int i = 0; i < 100; ++i) EXPECT_FALSE(m.update(0.008));
    EXPECT_EQ(m.faults(), 0);
    EXPECT_NEAR(m.meanMs(), 8.0, 1e-9);
    EXPECT_NEAR(m.percentileMs(0.99), 8.0, 1e-9);
}

TEST(PeriodMonitor, OverrunIsAFault) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);  // fault threshold 16 ms
    EXPECT_FALSE(m.update(0.010));
    EXPECT_TRUE(m.update(0.020));
    EXPECT_EQ(m.faults(), 1);
    EXPECT_NEAR(m.maxMs(), 20.0, 1e-9);
}

TEST(PeriodMonitor, PercentilesOnAMixedWindow) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    for (int i = 1; i <= 1000; ++i) m.update(0.001 * i);  // 1..1000 ms
    EXPECT_NEAR(m.percentileMs(0.50), 500.0, 2.0);
    EXPECT_NEAR(m.percentileMs(0.95), 950.0, 2.0);
    EXPECT_NEAR(m.percentileMs(0.99), 990.0, 2.0);
    EXPECT_NEAR(m.maxMs(), 1000.0, 1e-9);
}

TEST(PeriodMonitor, NonFiniteFallsBackToNominal) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    EXPECT_FALSE(m.update(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(m.update(0.0));
    EXPECT_NEAR(m.lastMs(), 8.0, 1e-9);
}

TEST(PeriodMonitor, DifferentiatesAbsoluteTime) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    // first call: no previous time -> fallback
    EXPECT_FALSE(m.updateFromTime(10.000, 0.008));
    EXPECT_FALSE(m.lastWasMeasured());
    // monotonic timestamps -> measured
    EXPECT_FALSE(m.updateFromTime(10.008, 0.008));
    EXPECT_TRUE(m.lastWasMeasured());
    EXPECT_NEAR(m.lastMs(), 8.0, 1e-9);
    // a real overrun is detected from the measured delta (22 ms > 16 ms)
    EXPECT_TRUE(m.updateFromTime(10.030, 0.008));
    EXPECT_NEAR(m.lastMs(), 22.0, 1e-9);
    EXPECT_EQ(m.faults(), 1);
}

TEST(PeriodMonitor, BackwardsClockFallsBackToNominal) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    m.updateFromTime(10.000, 0.008);
    m.updateFromTime(10.008, 0.008);
    EXPECT_FALSE(m.updateFromTime(9.000, 0.008));  // clock jumped backwards
    EXPECT_FALSE(m.lastWasMeasured());
    EXPECT_NEAR(m.lastMs(), 8.0, 1e-9);
}

TEST(PeriodMonitor, NonFiniteTimeFallsBack) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    m.updateFromTime(10.0, 0.008);
    EXPECT_FALSE(m.updateFromTime(std::numeric_limits<double>::quiet_NaN(), 0.008));
    EXPECT_FALSE(m.lastWasMeasured());
    EXPECT_NEAR(m.lastMs(), 8.0, 1e-9);
}

TEST(PeriodMonitor, WindowIsBounded) {
    PeriodMonitor m;
    m.configure(0.008, 2.0);
    for (int i = 0; i < 10000; ++i) m.update(0.008);
    EXPECT_EQ(m.count(), 10000);
    // mean/percentiles only reflect the bounded window; still 8 ms
    EXPECT_NEAR(m.meanMs(), 8.0, 1e-6);
}
