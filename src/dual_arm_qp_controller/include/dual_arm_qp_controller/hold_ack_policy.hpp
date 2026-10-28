// C4.3 S4 W3 -- HOLD ack policy, extracted so the readState-failure rule can be
// tested in a hooks=OFF production build.
//
// WHY THIS FILE EXISTS
//   The readState-failure branch of DualArmQpController::update() can only be
//   exercised end-to-end with a fault injection, and fault injection is a test
//   hook (compiled out of the production library).  So the *decision* it makes
//   -- "drain the queued seq, refuse it, publish the ack, freeze nothing" -- is
//   expressed here as pure functions that the production code calls verbatim.
//   test_hold_ack_policy.cpp therefore covers the decision in an OFF build,
//   while T_H17/T_H17a keep covering the wiring in a hooks=ON build.
//
// No ROS, no state, no allocation: safe to call from the control loop.

#ifndef DUAL_ARM_QPCONTROLLER__HOLD_ACK_POLICY_HPP_
#define DUAL_ARM_QPCONTROLLER__HOLD_ACK_POLICY_HPP_

#include <cstdint>

namespace dual_arm_qp_controller
{

/// TransportStatus.hold_request_result values (0 = no ack yet).
constexpr std::uint8_t kAckApplied = 1;
constexpr std::uint8_t kAckRefused = 2;

/// Packed ack as published in TransportStatus (seq in the high 32 bits).
inline std::uint64_t packHoldAck(std::uint32_t seq, std::uint8_t result)
{
    return (static_cast<std::uint64_t>(seq) << 32) | static_cast<std::uint64_t>(result);
}

struct ReadStateFailureAck
{
    bool publish = false;      ///< false => nothing was queued, nothing to say
    std::uint32_t seq = 0;     ///< seq to echo
    std::uint8_t result = 0;   ///< always kAckRefused today
};

/// Rule for the readState()-failed cycle: a request queued for this tick is
/// consumed and REFUSED (it was not applied -- the joints could not be read),
/// and the ack is published before ERROR propagates so the executor learns
/// "refused: nothing was frozen" instead of timing out and inventing an ABORT.
inline ReadStateFailureAck ackForReadStateFailure(std::uint32_t pending_seq)
{
    ReadStateFailureAck a;
    if (pending_seq == 0u) return a;
    a.publish = true;
    a.seq = pending_seq;
    a.result = kAckRefused;
    return a;
}

/// Rule for a replayed (already consumed) seq: "applied" only if the trajectory
/// is latched in HOLD right now.  `!running()` is NOT enough: COMPLETE also
/// stops stepping the trajectory through hold(), yet q_cmd keeps advancing.
inline std::uint8_t replayAckResult(bool still_held)
{
    return still_held ? kAckApplied : kAckRefused;
}

}  // namespace dual_arm_qp_controller

#endif  // DUAL_ARM_QPCONTROLLER__HOLD_ACK_POLICY_HPP_
