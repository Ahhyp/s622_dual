// C4.3 S3 -- package provenance stamp.
//
// Why this exists: every artifact written by S3 (§8 of
// C4.3_S3_实施方案.md) must be attributable to the exact build that produced
// it.  A run whose log cannot name its own build cannot be used as evidence.
#ifndef DUAL_ARM_TRANSPORT_MOCK__VERSION_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__VERSION_HPP_

#include <string>

namespace dual_arm_transport_mock {

/// "dual_arm_transport_mock"
const char* packageName();

/// Semantic version from package.xml, e.g. "0.1.0".
const char* packageVersion();

/// Compile-time stamp of this translation unit, e.g. "Aug 30 2025 14:03:11".
const char* buildStamp();

/// One-line header written into every CSV/JSONL artifact:
///   "pkg=dual_arm_transport_mock version=0.1.0 build=<stamp>"
std::string provenanceLine();

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__VERSION_HPP_
