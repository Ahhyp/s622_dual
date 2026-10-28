// R1-T11 — the six mandatory artifacts are produced, are stable across runs and
// a missing required file is impossible to overlook.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_transport_executor/artifacts.hpp"
#include "dual_arm_transport_executor/state_machine.hpp"

using namespace dual_arm_transport_executor;  // NOLINT

#ifndef C43_W3R1_ARTIFACT_DIR
#error "C43_W3R1_ARTIFACT_DIR must be defined by CMake"
#endif

namespace
{

std::string readAll(const std::string & p)
{
  std::ifstream f(p, std::ios::binary);
  std::ostringstream s;
  s << f.rdbuf();
  return s.str();
}

std::string makeRun(const std::string & dir)
{
  ArtifactWriter w;
  std::string err;
  EXPECT_TRUE(w.open(dir, &err)) << err;

  Manifest m;
  m.run_id = "unit_replay";
  m.git_head = "test-head";
  m.package_version = "0.1.0";
  m.build_type = "RelWithDebInfo";
  m.mode = "unit";
  m.env_class = "host";
  m.input_config_sha256 = "0000";
  m.started_utc = "1970-01-01T00:00:00Z";
  EXPECT_TRUE(w.writeManifest(m, &err)) << err;

  TransportStateMachine sm;
  std::vector<CycleRow> cycles;
  for (int k = 0; k < 5; ++k) {
    CycleRow r;
    r.k = k;
    r.t = 0.008 * k;
    r.state = toString(sm.state());
    r.failure_code = sm.failureCode();
    r.committed = (k % 2) == 0;
    r.t_traj = 0.008 * k;
    r.q_cmd.fill(0.01 * k);
    cycles.push_back(r);
  }
  EXPECT_TRUE(w.writeCycles(cycles, &err)) << err;
  EXPECT_TRUE(w.writeEvents(sm.events(), &err)) << err;
  EXPECT_TRUE(w.writeSummary(buildSummaryJson("unit_replay", "PASS", sm.events(), cycles, {"k=5"}), &err))
    << err;
  EXPECT_TRUE(w.writeCommands({"c43_s4_executor", "--mode", "unit"}, &err)) << err;
  EXPECT_TRUE(w.copyTestResults("", &err)) << err;
  return dir;
}

TEST(Artifacts, T11_AllSixMandatoryFilesExist)
{
  const std::string base = std::string(C43_W3R1_ARTIFACT_DIR) + "/unit";
  const std::string dir = makeRun(base);
  for (const char * f : {ArtifactNames::manifest(), ArtifactNames::events(),
      ArtifactNames::cycles(), ArtifactNames::summary(),
      ArtifactNames::testResults(), ArtifactNames::commands()})
  {
    const std::string p = dir + "/" + f;
    EXPECT_TRUE(std::filesystem::exists(p)) << p;
    EXPECT_GT(std::filesystem::file_size(p), 0u) << p;
  }
  EXPECT_NE(readAll(dir + "/cycles.csv").find("k,t,state,failure_code,committed,t_traj,q_cmd"),
    std::string::npos);
  EXPECT_NE(readAll(dir + "/events.csv").find("t,kind,from,to,code,detail"), std::string::npos);
  EXPECT_NE(readAll(dir + "/manifest.json").find("\"input_config_sha256\""), std::string::npos);
  EXPECT_NE(readAll(dir + "/manifest.json").find("\"env_class\""), std::string::npos);
}

TEST(Artifacts, T11b_TwoRunsProduceIdenticalBytes)
{
  const std::string a = makeRun(std::string(C43_W3R1_ARTIFACT_DIR) + "/det_a");
  const std::string b = makeRun(std::string(C43_W3R1_ARTIFACT_DIR) + "/det_b");
  for (const char * f : {ArtifactNames::manifest(), ArtifactNames::events(),
      ArtifactNames::cycles(), ArtifactNames::summary(),
      ArtifactNames::testResults(), ArtifactNames::commands()})
  {
    EXPECT_EQ(readAll(a + "/" + f), readAll(b + "/" + f)) << f;
  }
}

TEST(Artifacts, T11c_SummaryHasNoWallClockOrTempPaths)
{
  std::vector<CycleRow> cycles;
  const std::string s = buildSummaryJson("r", "PASS", {}, cycles, {});
  EXPECT_EQ(s.find("/tmp/"), std::string::npos);
  EXPECT_EQ(s.find("2026"), std::string::npos);
}

}  // namespace
