// C4.3 S3 -- W7: SHA-256 for the §8 provenance block.
//
// Self-contained on purpose: the only thing this package needs from a hash is "is
// this byte-for-byte the config that produced these numbers", and pulling libssl
// into a mock-transport executable to get it would add a dependency to every
// downstream build for 70 lines of code.  Verified against `sha256sum` in
// test_artifacts_c43s3.cpp (known-answer tests), not against itself.
#ifndef DUAL_ARM_TRANSPORT_MOCK__SHA256_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__SHA256_HPP_

#include <string>

namespace dual_arm_transport_mock {

/// Lower-case hex digest of `bytes`.
std::string sha256Hex(const std::string& bytes);

/// Lower-case hex digest of a file's contents.  Throws `std::runtime_error` if the
/// file cannot be read: a provenance block with a silently empty hash is worse than
/// no run at all.
std::string sha256FileHex(const std::string& path);

}  // namespace dual_arm_transport_mock

#endif  // DUAL_ARM_TRANSPORT_MOCK__SHA256_HPP_
