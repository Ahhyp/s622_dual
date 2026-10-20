// W1 skeleton smoke test: the package builds, links and exposes its stamp.
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport_mock/version.hpp"

TEST(Version, NamesPackageAndVersion) {
  EXPECT_STREQ(dual_arm_transport_mock::packageName(), "dual_arm_transport_mock");
  EXPECT_STREQ(dual_arm_transport_mock::packageVersion(), "0.1.0");
}

TEST(Version, ProvenanceLineNamesPackageVersionAndBuild) {
  const std::string line = dual_arm_transport_mock::provenanceLine();
  EXPECT_NE(line.find("pkg=dual_arm_transport_mock"), std::string::npos) << line;
  EXPECT_NE(line.find("version=0.1.0"), std::string::npos) << line;
  EXPECT_NE(line.find("build="), std::string::npos) << line;
}
