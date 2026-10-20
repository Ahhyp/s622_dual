#include "dual_arm_transport_mock/version.hpp"

namespace dual_arm_transport_mock {
namespace {
constexpr const char kPackageName[] = "dual_arm_transport_mock";
constexpr const char kPackageVersion[] = "0.1.0";
constexpr const char kBuildStamp[] = __DATE__ " " __TIME__;
}  // namespace

const char* packageName() { return kPackageName; }
const char* packageVersion() { return kPackageVersion; }
const char* buildStamp() { return kBuildStamp; }

std::string provenanceLine() {
  std::string s;
  s.reserve(96);
  s += "pkg=";
  s += kPackageName;
  s += " version=";
  s += kPackageVersion;
  s += " build=";
  s += kBuildStamp;
  return s;
}

}  // namespace dual_arm_transport_mock
