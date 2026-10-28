#include "dual_arm_transport_executor/hold_requester.hpp"

#include <cstdint>

namespace dual_arm_transport_executor
{

bool seqAtOrAfter(std::uint32_t a, std::uint32_t b)
{
    // Signed difference: correct across the uint32 wrap (2^31 window).
    return static_cast<std::int32_t>(a - b) >= 0;
}

HoldRequester::HoldRequester(HoldRequesterConfig cfg) : cfg_(cfg) {}

void HoldRequester::begin(double now_s, std::uint32_t seq, const char *reason)
{
    started_ = true;
    done_ = false;
    awaiting_reply_ = false;
    resend_pending_ = true;   // the first send happens immediately
    seq_ = seq;
    attempts_ = 0;
    last_send_s_ = now_s;
    outcome_.clear();
    (void)reason;             // the caller logs the human-readable reason
}

void HoldRequester::onServiceReply(bool accepted, const char *message)
{
    if (!started_ || done_) return;
    awaiting_reply_ = false;
    if (accepted) {
        // Queued.  The ack echo is now expected in the status stream.
        resend_pending_ = false;
        return;
    }
    // "busy: ..." -- another request owns the single slot.  Retry the SAME seq.
    resend_pending_ = true;
    outcome_ = message != nullptr ? message : "rejected";
}

void HoldRequester::markSent(double now_s)
{
    ++attempts_;
    last_send_s_ = now_s;
    awaiting_reply_ = true;
    resend_pending_ = false;
}

HoldStep HoldRequester::onObservation(const HoldObservation &obs)
{
    HoldStep step;
    step.seq = seq_;
    if (!started_ || done_) {
        step.action = HoldAction::Wait;
        return step;
    }

    const bool fresh = obs.have_status && obs.status_age_s <= cfg_.status_stale_s;
    if (fresh) {
        // P-2 rule 1: our own hold, judged by the triple, not by `result` alone.
        const bool origin_ours = obs.hold_origin == "executor";
        if (obs.state == "HOLD" && origin_ours && seqAtOrAfter(obs.ack_seq, seq_) &&
            obs.ack_seq != 0)
        {
            done_ = true;
            outcome_ = "frozen_by_executor_request";
            step.action = HoldAction::Done;
            step.reason = outcome_;
            return step;
        }
        if (obs.ack_seq == seq_) {
            if (obs.ack_result == 1) {
                if (obs.state == "HOLD") {
                    // The ack says "applied" and the controller reports HOLD:
                    // the plant IS frozen.  Ownership is NOT part of the
                    // success condition -- a safety layer (collision / D7 / E8
                    // / E1) can re-attribute the same tick's hold to itself, so
                    // demanding origin_ours here used to leave the executor
                    // waiting forever on a genuinely frozen robot (independent
                    // C7 review, P1-1).  Report the freeze, name the owner.
                    done_ = true;
                    outcome_ = origin_ours ? "applied_and_frozen"
                                           : ("applied_hold_owned_by_" + obs.hold_origin);
                    step.action = HoldAction::Done;
                    step.reason = outcome_;
                    return step;
                }
                // Applied but the state has not settled yet: fall through to the
                // ack-timeout machinery below instead of waiting unconditionally
                // -- an unbounded Wait was the second half of P1-1.
            }
            if (obs.ack_result == 2) {
                // Refused.  Two very different meanings:
                if (obs.state == "HOLD") {
                    // Somebody else holds.  C4.3 S4 D3-a: a controller-origin
                    // HOLD still means the trajectory IS frozen -- by a safety
                    // layer we do not own.  Reporting Abort here used to turn a
                    // safe state into a failure, and (before the same patch)
                    // could leave the executor ABORT while the controller kept
                    // RUNNING.  So: adopt the freeze, attribute it honestly, and
                    // never claim it as our own request.
                    done_ = true;
                    outcome_ = origin_ours ? "frozen_by_executor_request"
                                           : "frozen_by_controller";
                    step.action = HoldAction::Done;
                    step.reason = outcome_;
                    return step;
                }
                // Nothing to hold (IDLE / COMPLETE / ABORT / FAULT): the request
                // is refused because there is no running trajectory.
                done_ = true;
                outcome_ = "not_running:" + obs.state;
                step.action = (obs.state == "COMPLETE") ? HoldAction::Done : HoldAction::Abort;
                step.reason = outcome_;
                return step;
            }
        }
    }

    // P-2 rule 2: a reply of "busy" retries the same seq.
    if (resend_pending_) {
        if (attempts_ >= cfg_.max_attempts) {
            done_ = true;
            outcome_ = "hold_request_rejected";
            step.action = HoldAction::Abort;
            step.reason = outcome_;
            return step;
        }
        step.action = HoldAction::Resend;
        step.reason = "busy";
        return step;
    }

    // P-2 rule 3: a missing ack is NEVER success.
    if (awaiting_reply_ || attempts_ > 0) {
        const double age = obs.now_s - last_send_s_;
        if (age > cfg_.ack_timeout_s) {
            if (attempts_ >= cfg_.max_attempts) {
                done_ = true;
                outcome_ = "hold_ack_timeout";
                step.action = HoldAction::Abort;
                step.reason = outcome_;
                return step;
            }
            awaiting_reply_ = false;
            resend_pending_ = true;
            step.action = HoldAction::Resend;
            step.reason = "ack_timeout";
            return step;
        }
    }
    step.action = HoldAction::Wait;
    return step;
}

}  // namespace dual_arm_transport_executor
