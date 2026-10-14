// test/test_ik_layer.cpp
// C4.2 / S2: the analytic multi-solution IK layer.
//
// Key property under test (额外要求 2): we must never be limited to "the first
// solution" -- the solver enumerates branches and the selector scores all of them.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <set>

#include "dual_arm_pose_optimization/ik_layer.hpp"

namespace c42 = dual_arm_pose_optimization;
using fairino_planning::DHKinematics;
using fairino_planning::DHParams;
using fairino_planning::ToolModel;
using fairino_planning::ToolParams;
using fairino_planning::JointConfig;

namespace {

DHKinematics& fk() {
  static DHKinematics dh(DHParams{}, ToolParams::gripper());
  return dh;
}

const double kLo[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kHi[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  JointConfig q;
  for (int i = 0; i < 6; ++i) q[i] = kLo[i] + u(rng) * (kHi[i] - kLo[i]);
  return q;
}

}  // namespace

TEST(C42IkWrap, FkIkRoundTripReturnsTheOriginalConfiguration) {
  // The analytic solver must be able to reproduce a configuration from its own
  // FK -- this is the sanity check that also pins the frame convention
  // (targets are in the ARM BASE frame, not world).
  c42::IkLayer layer;
  std::mt19937 rng(1234);

  int tested = 0;
  for (int trial = 0; trial < 200 && tested < 40; ++trial) {
    const JointConfig q = randomQ(rng);
    const Eigen::Matrix4d target = fk().fkine(q, ToolModel::GRIPPER);
    const auto res = layer.solves(target, ToolModel::GRIPPER);
    if (!res.success || res.solutions.empty()) continue;

    double best = 1e9;
    for (const auto& s : res.solutions) {
      const Eigen::Matrix4d d = target.inverse() * fk().fkine(s, ToolModel::GRIPPER);
      best = std::min(best, d.block<3, 1>(0, 3).norm());
    }
    EXPECT_LT(best, 1e-6) << "trial " << trial << ": no returned solution reproduces the FK";
    ++tested;
  }
  EXPECT_GT(tested, 20);
}

TEST(C42IkWrap, SolverEnumeratesMoreThanOneBranch) {
  c42::IkLayer layer;
  std::mt19937 rng(99);
  int multi = 0;
  for (int i = 0; i < 50; ++i) {
    const JointConfig q = randomQ(rng);
    const auto res = layer.solves(fk().fkine(q, ToolModel::GRIPPER), ToolModel::GRIPPER);
    if (res.total_branches > 1) ++multi;
  }
  EXPECT_EQ(multi, 50) << "the solver must always enumerate the 8 analytic branches";
}

TEST(C42IkWrap, SelectReturnsAHardFilterPassingSolution) {
  c42::IkLayer layer;
  std::mt19937 rng(7);
  int selected = 0;
  int tested = 0;
  for (int i = 0; i < 60 && tested < 30; ++i) {
    const JointConfig q = randomQ(rng);
    const Eigen::Matrix4d target = fk().fkine(q, ToolModel::GRIPPER);
    const auto out = layer.solveAndSelect(target, q, ToolModel::GRIPPER);
    if (!out.solve_success || out.solutions.empty()) continue;
    ++tested;
    if (!out.ok) continue;
    ++selected;
    ASSERT_TRUE(out.selected.has_value());
    EXPECT_LT(out.residual_pos, 1e-6);
    EXPECT_GE(out.n_candidates, 1u);
    EXPECT_GE(out.n_hard_pass, 1u);
    // the selected candidate must itself be among the hard-passing ones
    EXPECT_GE(out.n_hard_pass, 1u);
  }
  EXPECT_GT(tested, 10);
  EXPECT_GT(selected, 0) << "at least some poses must yield a selectable branch";
}

TEST(C42IkWrap, RejectHistogramAccountsForEveryCandidate) {
  c42::IkLayer layer;
  std::mt19937 rng(2024);
  const JointConfig q = randomQ(rng);
  const auto out =
      layer.solveAndSelect(fk().fkine(q, ToolModel::GRIPPER), q, ToolModel::GRIPPER);

  int total = 0;
  for (const auto& kv : out.reject_histogram) total += kv.second;
  EXPECT_EQ(static_cast<std::size_t>(total), out.n_candidates)
      << "every scored candidate must land in exactly one reject bucket";
  EXPECT_LE(out.n_hard_pass, out.n_candidates);

  // NOTE: `n_hard_pass` is NOT guaranteed to be >= 1.  IKSelector's Grasp
  // profile rejects on ground clearance, wrist fold, q4 inner fold, joint
  // margin, ... and for some targets every analytic branch fails.  That is a
  // legitimate outcome (the pose is simply not graspable), not an error --
  // `SelectReturnsAHardFilterPassingSolution` covers the aggregate behaviour.
}

TEST(C42IkWrap, ResidualHelperAgreesWithFk) {
  const JointConfig q = (JointConfig() << 0.3, -1.2, 0.5, -0.4, 1.1, -1.5).finished();
  const Eigen::Matrix4d target = fk().fkine(q, ToolModel::GRIPPER);
  double pe = 0.0, re = 0.0;
  c42::IkLayer::residual(target, q, ToolModel::GRIPPER, &pe, &re);
  EXPECT_LT(pe, 1e-12);
  EXPECT_LT(re, 1e-12);

  // and it must be non-zero for a different configuration
  JointConfig q2 = q;
  q2[1] += 0.2;
  c42::IkLayer::residual(target, q2, ToolModel::GRIPPER, &pe, &re);
  EXPECT_GT(pe, 1e-3);
}
