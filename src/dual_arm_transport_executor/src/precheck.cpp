#include "dual_arm_transport_executor/precheck.hpp"

namespace dual_arm_transport_executor
{

PrecheckResult checkStart(const Config & cfg, const ControllerStatus & st,
                          bool have_status, double now_s)
{
  PrecheckResult r;
  if (!have_status) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "no controller status has ever been received";
    return r;
  }
  if (!st.parse_ok) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "latest controller status could not be parsed";
    return r;
  }
  if (now_s - st.stamp_s > cfg.status_stale_timeout_s) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "latest controller status is stale (age " +
               std::to_string(now_s - st.stamp_s) + " s)";
    return r;
  }
  if (!st.failure_code.empty() && st.failure_code != "ok") {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "controller is already reporting '" + st.failure_code + "'";
    return r;
  }
  if (st.transport_active) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "the controller already reports an active transport";
    return r;
  }
  if (st.solve_fail_streak > 0) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "solve_fail_streak is " + std::to_string(st.solve_fail_streak);
    return r;
  }
  if (st.active_limit_count > 0) {
    r.ok = false;
    r.code = "precheck_failed";
    r.detail = "active_limit_count is " + std::to_string(st.active_limit_count);
    return r;
  }
  return r;
}

}  // namespace dual_arm_transport_executor
