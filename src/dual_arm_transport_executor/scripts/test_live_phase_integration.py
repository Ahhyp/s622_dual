#!/usr/bin/env python3
"""C4.3 S4.1 W3-R2 live integration: phase fields over the real DDS path.

This is NOT a unit test.  It forks the real mock bench

    ros2 launch dual_arm_qp_controller c43_s4_mock.launch.py params_file:=<yaml>

publishes the E8 heartbeat at 50 Hz, calls the real ``~/start_transport`` and
records every ``/dual_arm_qp_controller/transport_status`` sample that actually
arrives over DDS.  It then runs the real executor core against the same wire
data through ``test/live_phase_probe.cpp`` (real TransportStateMachine + real
PhaseDriver), and finally the real ``c43_s4_executor`` adapter node.

Legs
  I*  idle publish defaults before the first start and again after reset.
  W*  run A -- controller publication over DDS: +1 monotonic index, expected
      name sequence, count==4, 0 <= elapsed <= duration+0.02, durations equal
      to the request, old fields coherent, COMPLETE at t_traj==duration.
  X*  run A, probe (the real TransportStateMachine + PhaseDriver, fed by the
      real status stream): the SM advances APPROACH -> GRASP -> LIFT ->
      TRANSPORT -> LOWER at the *controller's* phase index, while a second SM
      fed a deliberately slowed local clock (t_traj*0.15) stays in APPROACH.
  O*  run A, second subscription that reads only the pre-W3-R2 fields.
  SB* run B -- 5 phases with a `settle` phase: publication is correct and the
      settle phase causes no executor state transition.
  MC* run C -- controller runs 2 phases [approach, transport] against a
      4-phase executor table: the skipped name must latch phase_name_mismatch,
      not silently advance.
  N*  the production `c43_s4_executor` adapter on the same wire (defect
      detector: B3 -- heartbeat never fed to the SM; B1 -- no PhaseDriver).
  T*  production staleness threshold (24 ms) vs the 10 Hz status period
      (defect detector: B2).

Exit codes
  0  every assertion passed and no defect was reproduced  -> R2 can be DONE
  1  an assertion failed (harness or product)
  3  every assertion passed but the defect detectors fired -> R2 is PARTIAL
"""
import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time

TOPIC = "/dual_arm_qp_controller/transport_status"
HB_TOPIC = "/dual_arm_qp_controller/heartbeat"
START_SRV = "/dual_arm_qp_controller/start_transport"
RESET_SRV = "/dual_arm_qp_controller/reset_transport"
EXECUTOR_SRV = "/c43_s4_transport_executor/start"
PROBE_PKG = "dual_arm_transport_executor"
PROBE_EXE = "live_phase_probe"
EXECUTOR_EXE = "c43_s4_executor"
DT = 0.008

RESULTS = []
# Defects reproduced by this harness.  R2 can only be called DONE when the
# publication side AND the production adapter are both clean, i.e. when this
# list is empty.  Legs N and T are *defect detectors*: their assertions pass
# when the defect is present, so a green run with a non-empty BUGS means the
# integration is still broken.
BUGS = []


def check(aid, ok, msg, detail=""):
    RESULTS.append({"id": aid, "ok": bool(ok), "msg": msg, "detail": detail})
    print(("PASS %-3s " % aid if ok else "FAIL %-3s " % aid) + msg
          + ((" | " + detail) if detail else ""), flush=True)
    return bool(ok)


def bug(bid, where, desc, detail=""):
    BUGS.append({"id": bid, "where": where, "desc": desc, "detail": detail})
    print("BUG  %-3s %s  [%s]  %s" % (bid, desc, where, detail), flush=True)


def info(msg):
    print("[harness] " + msg, flush=True)


# ---------------------------------------------------------------------------
# process / environment plumbing
# ---------------------------------------------------------------------------
class Child:
    def __init__(self, name, argv, env, log_path):
        self.name = name
        self.log_path = log_path
        self._log = open(log_path, "w")
        info("spawn %s: %s" % (name, " ".join(argv)))
        self.proc = subprocess.Popen(
            argv, env=env, stdout=self._log, stderr=subprocess.STDOUT,
            start_new_session=True)

    def alive(self):
        return self.proc.poll() is None

    def stop(self, sig=signal.SIGINT, timeout=10.0):
        if self.proc.poll() is not None:
            self._log.close()
            return self.proc.returncode
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except ProcessLookupError:
            pass
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            rc = self.proc.wait(timeout=5)
        self._log.close()
        return rc


def child_env(base, ros_home, ros_log):
    env = dict(base)
    env["ROS_DOMAIN_ID"] = "77"
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_HOME"] = ros_home
    env["ROS_LOG_DIR"] = ros_log
    return env


def make_params(out_dir):
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("dual_arm_qp_controller")
    src = os.path.join(share, "config", "c43_s4_mock_params.yaml")
    with open(src) as fh:
        text = fh.read()
    if not text.endswith("\n"):
        text += "\n"
    # Append-only override under `dual_arm_qp_controller: ros__parameters:`.
    text += (
        "    # W3-R2 live harness: timing recorder off, default status cadence.\n"
        "    timing_enable: false\n"
        "    transport_status_rate_hz: 10.0\n"
        "    # The phase stream is observed at 10 Hz; a WSL2 scheduling hiccup of\n"
        "    # ~0.1 s must not latch the controller's E8 hold in the middle of the\n"
        "    # run (declared default is 0.1 s -- this harness deliberately widens\n"
        "    # the tolerance so the measurement is not aborted by host jitter;\n"
        "    # the runtime view used by the executor node is unchanged).\n"
        "    executor_heartbeat_timeout_s: 0.3\n"
    )
    path = os.path.join(out_dir, "w3r2_mock_params.yaml")
    with open(path, "w") as fh:
        fh.write(text)
    return path, src


# ---------------------------------------------------------------------------
# rclpy driver
# ---------------------------------------------------------------------------
def build_driver(rclpy, TransportStatus, Empty, Trigger, StartTransport,
                 SingleThreadedExecutor, MultiThreadedExecutor):
    """Create the harness node.  Defined inside a function so the module can be
    imported (--help) without a sourced ROS environment."""
    import rclpy.node

    class Driver(rclpy.node.Node):
        def __init__(self):
            super().__init__("w3r2_harness")
            self.samples = []
            self.old_only = []
            self.create_subscription(TransportStatus, TOPIC, self.on_status, 10)
            # Second, independent consumer that NEVER touches the new fields:
            # this is the "old consumer still works" leg.
            self.create_subscription(TransportStatus, TOPIC, self.on_old_status, 10)
            self.hb = self.create_publisher(Empty, HB_TOPIC, 10)
            self.create_timer(0.02, lambda: self.hb.publish(Empty()))
            self.start_cli = self.create_client(StartTransport, START_SRV)
            self.reset_cli = self.create_client(Trigger, RESET_SRV)

        def on_status(self, m):
            self.samples.append({
                "t": time.monotonic(),
                "state": m.state,
                "failure_code": m.failure_code,
                "active": bool(m.active),
                "committed": bool(m.committed),
                "t_traj": float(m.t_traj),
                "t_traj_duration": float(m.t_traj_duration),
                "q_len": len(m.q_state),
                "q_state0": float(m.q_state[0]),
                "phase_index": int(m.phase_index),
                "phase_count": int(m.phase_count),
                "phase_name": m.phase_name,
                "phase_elapsed_s": float(m.phase_elapsed_s),
                "phase_duration_s": float(m.phase_duration_s),
                "hb_seen": bool(m.executor_heartbeat_seen),
                "hb_age": float(m.executor_heartbeat_age_s),
            })

        def on_old_status(self, m):
            # OLD-SCHEMA CONSUMER: only fields that existed before W3-R2.
            self.old_only.append({
                "t": time.monotonic(),
                "state": m.state,
                "active": bool(m.active),
                "t_traj": float(m.t_traj),
                "t_traj_duration": float(m.t_traj_duration),
                "q_len": len(m.q_state),
            })

    return Driver()


class Harness:
    def __init__(self, node_cls, rclpy_mod, Empty, Trigger, StartTransport,
                 SingleThreadedExecutor):
        self.rclpy = rclpy_mod
        self.Empty = Empty
        self.Trigger = Trigger
        self.StartTransport = StartTransport
        self.node = node_cls
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)

    def spin_once(self, timeout=0.002):
        self.exec.spin_once(timeout_sec=timeout)

    def spin_until(self, pred, timeout, desc, poll_extra=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin_once()
            if poll_extra is not None:
                poll_extra()
            if pred():
                return True
        if poll_extra is not None:
            poll_extra()
        return pred()

    def spin_for(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.spin_once()

    def wait_service(self, cli, timeout, name):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin_once()
            if cli.service_is_ready():
                return True
        return cli.service_is_ready()

    def call(self, cli, req, timeout=15.0):
        if not self.wait_service(cli, timeout, "service"):
            return None
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin_once()
            if fut.done():
                return fut.result()
        return None

    def start_transport(self, phases, timeout=20.0):
        req = self.StartTransport.Request()
        req.dt = DT
        req.time_scaling = "smoothstep"
        req.yaw_sweep_deg = [0.0]
        for name, dur in phases:
            req.phase_name.append(name)
            req.phase_duration.append(dur)
            req.phase_dx.append(0.0)
            req.phase_dy.append(0.0)
            req.phase_dz.append(0.0)
        return self.call(self.node.start_cli, req, timeout)

    def trigger(self, cli, timeout=10.0):
        return self.call(cli, self.Trigger.Request(), timeout)

    def reset_controller(self, timeout=10.0):
        res = self.trigger(self.node.reset_cli, timeout)
        ok = self.spin_until(
            lambda: self.node.samples and not self.node.samples[-1]["active"]
            and self.node.samples[-1]["phase_index"] == -1, 8.0, "idle after reset")
        self.spin_for(0.35)
        return res, ok


# ---------------------------------------------------------------------------
# JSONL helpers
# ---------------------------------------------------------------------------
def read_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def sm_events(records, sm):
    # Probe JSONL carries two record families: state-machine events (with
    # from/to) and service-result records (kind start_ok/start_failed, no
    # from/to).  Only the former is a transition event.
    return [r for r in records
            if r.get("sm") == sm and "kind" in r and "from" in r and "to" in r]


def find_event(records, sm, **kw):
    for r in sm_events(records, sm):
        if all(r.get(k) == v for k, v in kw.items()):
            return r
    return None


def events_sorted(records, sm):
    ev = sm_events(records, sm)
    # Probe writes in order; keep file order, it is append-only and single-node.
    return ev


# ---------------------------------------------------------------------------
# assertions
# ---------------------------------------------------------------------------
def fmt(s):
    return ("state=%s active=%s idx=%d/%d name=%r t=%.3f/%.3f el=%.3f/%.3f"
            % (s["state"], int(s["active"]), s["phase_index"], s["phase_count"],
               s["phase_name"], s["t_traj"], s["t_traj_duration"],
               s["phase_elapsed_s"], s["phase_duration_s"]))


def fmt_old(o):
    if o is None:
        return "None"
    return ("state=%s active=%s t=%.6f dur=%.6f q_len=%d"
            % (o["state"], int(o["active"]), o["t_traj"], o["t_traj_duration"],
               o["q_len"]))


def check_active_stream(aid_prefix, samples, phases, terminal_state="COMPLETE"):
    """W2..W7 / S1: every active sample of one transport run."""
    names = [p[0] for p in phases]
    durs = [p[1] for p in phases]
    n = len(phases)
    total = sum(durs)
    active = [s for s in samples if s["active"]]
    ok_all = True

    ok = len(active) >= 5
    ok_all &= check(aid_prefix + "1", ok, "active stream recorded",
                    "%d active samples, %d total" % (len(active), len(samples)))

    if not active:
        return False

    first, last = active[0], active[-1]
    info("first active: " + fmt(first))
    info("last  active: " + fmt(last))

    # W2: count is the frozen phase count, and the index is in range, always.
    bad = [s for s in active if s["phase_count"] != n or not (0 <= s["phase_index"] < n)]
    ok_all &= check(aid_prefix + "2", not bad,
                    "phase_count == %d and 0 <= phase_index < %d on every active sample"
                    % (n, n),
                    "first=%d/%d last=%d/%d"
                    % (first["phase_count"], first["phase_index"],
                       last["phase_count"], last["phase_index"]))
    within = [s for s in active if 0 <= s["phase_index"] < n]

    # W3: monotonic +1 (never jumps, never goes back).
    bad = []
    seq = [s["phase_index"] for s in active]
    for i in range(1, len(seq)):
        d = seq[i] - seq[i - 1]
        if d not in (0, 1):
            bad.append((i, seq[i - 1], seq[i]))
    ok_all &= check(aid_prefix + "3", not bad,
                    "phase_index advances by +1 or 0 (no jumps/regressions)",
                    "deltas ok; distinct sequence=%s" % sorted(set(seq)))

    seen = []
    for v in seq:
        if not seen or seen[-1] != v:
            seen.append(v)
    ok = seen == list(range(n))
    ok_all &= check(aid_prefix + "4", ok,
                    "all phases observed in order 0..%d" % (n - 1),
                    "observed order=%s counts=%s"
                    % (seen, {i: seq.count(i) for i in range(n)}))

    # W4: names match the request per index.
    bad = [(s["phase_index"], s["phase_name"]) for s in within
           if s["phase_name"] != names[s["phase_index"]]]
    ok_all &= check(aid_prefix + "5", not bad,
                    "phase_name matches the requested name at its index",
                    "names=%s" % names)

    # W5: durations match the request, elapsed in [0, duration+20 ms].
    bad_dur = [s for s in within
               if abs(s["phase_duration_s"] - durs[s["phase_index"]]) > 1e-9]
    bad_el = [s for s in within
              if not (0.0 <= s["phase_elapsed_s"] <= s["phase_duration_s"] + 0.02)]
    ok_all &= check(aid_prefix + "6", not bad_dur and not bad_el,
                    "phase_duration matches request; 0 <= elapsed <= duration+0.02",
                    "max elapsed=%.4f" % max(s["phase_elapsed_s"] for s in within))

    # W6: elapsed is derived from the same t_traj (cross-check new vs old field).
    starts = [0.0]
    for d in durs[:-1]:
        starts.append(starts[-1] + d)
    bad = [s for s in within
           if abs(s["phase_elapsed_s"]
                  - (s["t_traj"] - starts[s["phase_index"]])) > 1e-6]
    ok_all &= check(aid_prefix + "7", not bad,
                    "phase_elapsed_s == t_traj - phase_start (same snapshot)",
                    "idx=%d elapsed=%.4f t=%.4f" % (last["phase_index"],
                                                    last["phase_elapsed_s"], last["t_traj"]))

    # W7: old fields are still coherent (state / duration / clock).
    bad = [s for s in active
           if s["t_traj_duration"] != total or s["q_len"] != 12
           or not (-1e-9 <= s["t_traj"] <= total + 1e-9)
           or s["state"] not in ("RUNNING", terminal_state)]
    ok_all &= check(aid_prefix + "8", not bad,
                    "old fields coherent (duration=%.3f, t_traj in range, state RUNNING/%s)"
                    % (total, terminal_state),
                    "states seen=%s" % sorted(set(s["state"] for s in active)))

    # W7b: terminal sample.
    ok = (last["state"] == terminal_state and abs(last["t_traj"] - total) <= 1.5 * 0.104
          and last["phase_index"] == n - 1)
    ok_all &= check(aid_prefix + "9", ok,
                    "run finished %s at t_traj==duration, final phase %d"
                    % (terminal_state, n - 1),
                    fmt(last))
    return ok_all


# ---------------------------------------------------------------------------
# scenarios
# ---------------------------------------------------------------------------
def run_scenario(h, children, out_dir, probe_tag, phases, expect, timeout=20.0):
    """Start a fresh probe, start it, start the controller, wait for `expect`
    (predicate over the probe JSONL), then stop the probe."""
    probe_jsonl = os.path.join(out_dir, "probe_%s.jsonl" % probe_tag)
    if os.path.exists(probe_jsonl):
        os.remove(probe_jsonl)
    probe = Child(
        "probe_" + probe_tag,
        ["ros2", "run", PROBE_PKG, PROBE_EXE, probe_jsonl,
         ",".join("%s:%s" % (n, d) for n, d in phases), "0.3", "0.15", probe_tag],
        child_env(os.environ, h.node_ros_home, h.node_ros_log),
        os.path.join(out_dir, "probe_%s.log" % probe_tag))
    children.append(probe)

    start_cli = h.node.create_client(h.Trigger, "/w3r2_probe_%s/start" % probe_tag)
    ok = h.wait_service(start_cli, 20.0, "probe start")
    if not ok:
        return None, probe
    res = h.call(start_cli, h.Trigger.Request(), 5.0)
    info("probe %s start -> success=%s msg=%r"
         % (probe_tag, None if res is None else res.success,
            None if res is None else res.message))

    got_start = h.spin_until(
        lambda: find_event(read_jsonl(probe_jsonl), "ext", kind="start") is not None,
        5.0, "probe start event")
    # The controller must be started after (or with) the probe; callers do it.
    return {"jsonl": probe_jsonl, "probe": probe, "started": got_start,
            "start_cli": start_cli}, probe


def wait_probe(h, probe_info, pred, timeout):
    path = probe_info["jsonl"]
    return h.spin_until(lambda: any(pred(r) for r in read_jsonl(path)), timeout,
                        "probe condition")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="/tmp/w3r2_live")
    ap.add_argument("--keep", action="store_true", help="keep the run directory (default)")
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out_dir, "run_" + stamp)
    os.makedirs(out_dir, exist_ok=True)
    ros_home = os.path.join(out_dir, "ros_home")
    ros_log = os.path.join(out_dir, "ros_log")
    os.makedirs(ros_home, exist_ok=True)
    os.makedirs(ros_log, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = "77"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_HOME"] = ros_home
    os.environ["ROS_LOG_DIR"] = ros_log

    params_file, params_src = make_params(out_dir)
    info("out_dir   = " + out_dir)
    info("params    = " + params_file + "  (from " + params_src + ")")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Empty
    from std_srvs.srv import Trigger
    from dual_arm_qp_controller.msg import TransportStatus
    from dual_arm_qp_controller.srv import StartTransport

    rclpy.init()
    driver = build_driver(rclpy, TransportStatus, Empty, Trigger, StartTransport,
                          SingleThreadedExecutor, None)
    h = Harness(driver, rclpy, Empty, Trigger, StartTransport, SingleThreadedExecutor)
    h.node_ros_home = ros_home
    h.node_ros_log = ros_log
    env = child_env(os.environ, ros_home, ros_log)
    children = []
    controller = None

    try:
        controller = Child(
            "controller",
            ["ros2", "launch", "dual_arm_qp_controller", "c43_s4_mock.launch.py",
             "params_file:=" + params_file],
            env, os.path.join(out_dir, "controller.log"))
        children.append(controller)

        up = h.spin_until(lambda: len(driver.samples) > 0, 150.0, "controller status")
        if not up:
            check("W0", False, "controller published /transport_status")
            tail = open(os.path.join(out_dir, "controller.log")).read()[-4000:]
            info("controller log tail:\n" + tail)
            return finish(out_dir, None)
        check("W0", True, "controller published /transport_status",
              "%d samples before start" % len(driver.samples))

        h.spin_for(1.2)
        idle = driver.samples[-1]
        info("idle sample: " + fmt(idle))
        ok = (idle["state"] == "IDLE" and not idle["active"]
              and idle["phase_index"] == -1 and idle["phase_count"] == 0
              and idle["phase_name"] == "" and idle["phase_elapsed_s"] == 0.0
              and idle["phase_duration_s"] == 0.0 and idle["t_traj"] == 0.0)
        check("I1", ok, "idle publish carries -1 / 0 / \"\" / 0 / 0", fmt(idle))

        # ---- run A: the canonical 4-phase transport ----------------------
        phases_a = [("approach", 1.0), ("lift", 1.2), ("transport", 1.4),
                    ("lower", 1.0)]
        total_a = sum(d for _, d in phases_a)

        # NOTE: the production adapter is NOT attached during runs A/B/C.  It
        # latches HOLD "executor_lost" on the first active status (bug B3) and
        # its hold request freezes the controller's trajectory, which would
        # destroy the very stream these runs measure.  Leg N measures the
        # adapter in isolation instead.
        def run_a_attempt():
            a0 = len(driver.samples)
            probe, _ = run_scenario(h, children, out_dir, "a", phases_a, None)
            if probe is None:
                return None
            sres = h.start_transport(phases_a, 20.0)
            accepted = sres is not None and sres.accepted
            recs = None
            if accepted:
                h.spin_until(
                    lambda: len(driver.samples) > a0
                    and driver.samples[-1]["state"]
                    in ("COMPLETE", "FAULT", "ABORT", "HOLD"),
                    20.0, "run A terminal")
                h.spin_for(0.45)
                recs = read_jsonl(probe["jsonl"])
            probe["probe"].stop()
            return {"accepted": accepted,
                    "msg": None if sres is None else sres.message,
                    "samples": driver.samples[a0:] if accepted else [],
                    "records": recs or []}

        # A host freeze (WSL2) can desynchronise the observer and the
        # controller for seconds; the phase assertions then see a jump that is
        # real over the wire but says nothing about the feature under test.
        # One clean retry keeps the harness deterministic.
        res_a = None
        attempts_used = 0
        for attempt in (1, 2):
            attempts_used = attempt
            res_a = run_a_attempt()
            if res_a is None:
                break
            bad = [r for r in sm_events(res_a["records"], "ext")
                   if r.get("kind") == "protection"]
            if res_a["accepted"] and not bad:
                break
            info("run A attempt %d unusable: accepted=%s protections=%s"
                 % (attempt, res_a["accepted"],
                    [(b.get("code"), b.get("detail")) for b in bad]))
            if attempt == 1:
                h.reset_controller()
        if res_a is None:
            check("A0", False, "probe A came up")
            return finish(out_dir, None)
        check("A0", True, "probe A came up and started from the real status stream",
              "attempts used: %d" % attempts_used)
        check("A1", res_a["accepted"], "controller accepted the 4-phase start_transport",
              "message=%r" % (res_a["msg"],))
        if not res_a["accepted"]:
            return finish(out_dir, None)
        samples_a = res_a["samples"]
        write_jsonl(os.path.join(out_dir, "samples_a.jsonl"), samples_a)
        probe_records = res_a["records"]
        check_active_stream("W", samples_a, phases_a)
        check("W10", all(not s["active"] or s["hb_seen"] for s in samples_a),
              "controller saw the 50 Hz executor heartbeat while active")

        # ---- X: probe = the real executor core, phase driver attached ----
        ext = events_sorted(probe_records, "ext")
        loc = events_sorted(probe_records, "local")
        info("probe ext events: %s"
             % ([(e["kind"], e["from"], e["to"], e["ctrl_phase_index"]) for e in ext],))
        info("probe local events: %s"
             % ([(e["kind"], e["from"], e["to"], e["ctrl_phase_index"]) for e in loc],))

        def has(sm_events_, **kw):
            return any(all(e.get(k) == v for k, v in kw.items()) for e in sm_events_)

        check("X1", has(ext, kind="start"),
              "probe ext started (IDLE -> APPROACH) from the real start service",
              "start events=%s" % [e["kind"] for e in ext if e["kind"] == "start"])
        e_lift = find_event(probe_records, "ext", to="LIFT")
        e_trans = find_event(probe_records, "ext", to="TRANSPORT")
        e_lower = find_event(probe_records, "ext", to="LOWER")
        check("X2", e_lift is not None and e_lift["ctrl_phase_index"] == 1,
              "ext enters LIFT when the controller reports phase_index==1",
              "event=%s" % str(None if e_lift is None else
                            (e_lift["from"], e_lift["to"], e_lift["ctrl_phase_index"],
                             e_lift["ctrl_phase_name"], round(e_lift["ctrl_t_traj"], 3))))
        check("X3", e_trans is not None and e_trans["ctrl_phase_index"] == 2,
              "ext enters TRANSPORT when the controller reports phase_index==2",
              "event=%s" % str(None if e_trans is None else
                            (e_trans["from"], e_trans["to"], e_trans["ctrl_phase_index"],
                             e_trans["ctrl_phase_name"], round(e_trans["ctrl_t_traj"], 3))))
        check("X4", e_lower is not None and e_lower["ctrl_phase_index"] == 3,
              "ext enters LOWER when the controller reports phase_index==3",
              "event=%s" % str(None if e_lower is None else
                            (e_lower["from"], e_lower["to"], e_lower["ctrl_phase_index"],
                             e_lower["ctrl_phase_name"], round(e_lower["ctrl_t_traj"], 3))))
        prot = [e for e in ext if e["kind"] == "protection"]
        check("X5", not prot, "ext never latched a protection state",
              "protection events=%s" % [(e["code"]) for e in prot])

        # X6: the local clock disagrees (t_traj * 0.15) and must NOT drive it.
        loc_final = loc[-1]["state_now"] if loc else "?"
        loc_up = [e for e in loc if e["to"] in ("LIFT", "TRANSPORT", "LOWER")]
        check("X6", loc_final == "APPROACH" and not loc_up,
              "slow local clock (t_traj*0.15) stays APPROACH: the external index wins",
              "local final=%s, local advances=%s, ext LOWER at ctrl t=%.3f"
              % (loc_final, [(e["from"], e["to"]) for e in loc_up],
                 e_lower["ctrl_t_traj"] if e_lower else -1.0))

        # ---- I2: idle again after the controller reset ------------------
        res, ok = h.reset_controller()
        idle2 = driver.samples[-1]
        ok2 = (idle2["state"] == "IDLE" and not idle2["active"]
               and idle2["phase_index"] == -1 and idle2["phase_count"] == 0
               and idle2["phase_name"] == "" and idle2["phase_elapsed_s"] == 0.0
               and idle2["phase_duration_s"] == 0.0)
        check("I2", ok2, "after ~/reset_transport the publish is idle again",
              fmt(idle2))

        # ---- O: old-field-only consumer --------------------------------
        # Same wire, but the callback never reads phase_index/count/name/
        # elapsed/duration.  It must observe exactly the pre-W3-R2 view of the
        # run, which doubles as a backwards-compatibility check on the
        # publisher (the extra fields must not disturb the old ones).
        old = driver.old_only
        old_ok = len(old) > 20 and all(o["q_len"] == 12 for o in old)
        newest_old = old[-1] if old else None
        check("O1", old_ok,
              "consumer that reads only the pre-W3-R2 fields still works",
              "%d old-schema samples, last state=%s" %
              (len(old), None if newest_old is None else newest_old["state"]))
        old_run = [o for o in old if o["state"] in ("RUNNING", "COMPLETE")
                   and o["t_traj_duration"] > 0.0]
        old_dur = old_run[0]["t_traj_duration"] if old_run else None
        check("O2", old_dur is not None and abs(old_dur - total_a) < 1e-9
              and newest_old is not None and newest_old["state"] == "IDLE"
              and newest_old["t_traj_duration"] == 0.0,
              "old consumer saw duration=%.3f during the run and idle after reset"
              % total_a,
              "old-run duration=%s, newest=%s" % (old_dur, fmt_old(newest_old)))

        # ---- run B: settle phase must not transition --------------------
        phases_b = [("approach", 0.8), ("settle", 0.5), ("lift", 1.0),
                    ("transport", 1.2), ("lower", 0.8)]
        b0 = len(driver.samples)
        probe_b, _ = run_scenario(h, children, out_dir, "b", phases_b, None)
        if probe_b is None:
            check("SBp", False, "probe B came up")
            return finish(out_dir, None)
        sres = h.start_transport(phases_b, 20.0)
        ok = sres is not None and sres.accepted
        check("SB0", ok, "controller accepted the 5-phase settle start",
              "message=%r" % (None if sres is None else sres.message))
        h.spin_until(lambda: len(driver.samples) > b0
                     and driver.samples[-1]["state"] == "COMPLETE", 20.0, "run B")
        h.spin_for(0.45)
        probe_b["probe"].stop()
        samples_b = driver.samples[b0:]
        write_jsonl(os.path.join(out_dir, "samples_b.jsonl"), samples_b)
        check_active_stream("SB", samples_b, phases_b)
        rec_b = read_jsonl(probe_b["jsonl"])
        settle_ev = find_event(rec_b, "ext", kind="phase")
        check("SB10", settle_ev is not None and settle_ev["from"] == settle_ev["to"]
              and settle_ev["ctrl_phase_index"] == 1
              and settle_ev["ctrl_phase_name"] == "settle",
              "settle phase produces NO executor state transition",
              "event=%s" % str(None if settle_ev is None else
                            (settle_ev["kind"], settle_ev["from"], settle_ev["to"],
                             settle_ev["ctrl_phase_index"], settle_ev["ctrl_phase_name"],
                             settle_ev["detail"][:60])))
        b_lift = find_event(rec_b, "ext", to="LIFT")
        b_lower = find_event(rec_b, "ext", to="LOWER")
        check("SB11", b_lift is not None and b_lift["ctrl_phase_index"] == 2
              and b_lower is not None and b_lower["ctrl_phase_index"] == 4,
              "after settle, LIFT is entered at index 2 and LOWER at index 4",
              "lift@%s lower@%s" % (None if b_lift is None else b_lift["ctrl_phase_index"],
                                    None if b_lower is None else b_lower["ctrl_phase_index"]))
        h.reset_controller()

        # ---- run C: 2-phase run against the 4-phase table ---------------
        phases_c = [("approach", 0.6), ("transport", 1.4)]
        # The probe is configured with the canonical 4-phase table on purpose:
        # the controller now skips index 1 ("lift") and reports "transport",
        # which must be caught as a name mismatch, not silently followed.
        probe_table_c = [("approach", 0.6), ("lift", 1.4), ("transport", 1.4),
                         ("lower", 1.0)]
        c0 = len(driver.samples)
        probe_c, _ = run_scenario(h, children, out_dir, "c", probe_table_c, None)
        if probe_c is None:
            check("MCp", False, "probe C came up")
            return finish(out_dir, None)
        sres = h.start_transport(phases_c, 20.0)
        ok = sres is not None and sres.accepted
        check("MC0", ok, "controller accepted the 2-phase (skipped lift) start",
              "message=%r" % (None if sres is None else sres.message))
        h.spin_until(
            lambda: any(r.get("kind") == "protection"
                        for r in read_jsonl(probe_c["jsonl"])), 8.0, "probe C hold")
        # The probe is a passive observer (it does not drive the controller's
        # hold service), so the controller keeps running to COMPLETE even
        # though the executor core has latched HOLD.  Record the whole run so
        # the publication assertions cover it; the mismatch is judged below.
        h.spin_until(lambda: len(driver.samples) > c0
                     and driver.samples[-1]["state"] == "COMPLETE", 8.0, "run C terminal")
        h.spin_for(0.25)
        probe_c["probe"].stop()
        samples_c = driver.samples[c0:]
        write_jsonl(os.path.join(out_dir, "samples_c.jsonl"), samples_c)
        check_active_stream("MC", samples_c, phases_c)
        rec_c = read_jsonl(probe_c["jsonl"])
        prot_c = [r for r in rec_c if r.get("sm") == "ext" and r.get("kind") == "protection"]
        ok = bool(prot_c) and prot_c[0]["code"] == "phase_name_mismatch" \
            and prot_c[0]["ctrl_phase_index"] == 1 \
            and prot_c[0]["ctrl_phase_name"] == "transport"
        check("MC10", ok,
              "skipped phase (name 'transport' at index 1) latches phase_name_mismatch",
              "events=%s" % ([(p["code"], p["ctrl_phase_index"], p["ctrl_phase_name"],
                               p["detail"][:70]) for p in prot_c],))
        check("MC11", bool(prot_c) and prot_c[0]["state_now"] == "HOLD",
              "mismatch lands in HOLD, not in progress", "")

        # The controller names are checked before the index, so a +gap index is
        # observed as a name mismatch.  Report that explicitly.
        info("NOTE: a +2 index jump is not producible by the real controller "
             "(phaseIndexAt is contiguous); its integration-observable form is the "
             "skipped phase caught here by the name-first check.  A synthetic +2 "
             "jump would be a unit test, not this live harness.")
        h.reset_controller()
        # Wait for the reset to be observable in the stream: the adapter's
        # precheck refuses to start unless the controller reports IDLE.
        h.spin_until(lambda: bool(driver.samples) and driver.samples[-1]["state"] == "IDLE",
                     6.0, "controller IDLE after reset")

        # ---- N: the shipped adapter, on the real wire (regression) ------
        # History: this leg originally DETECTED two adapter defects (B1: no
        # PhaseDriver attached; B3: heartbeat published but never fed to the SM,
        # so heartbeatStale() was permanently true).  Both are fixed, so the leg
        # now asserts the fixed behaviour: the adapter must consume the E8
        # heartbeat, learn the phase table from the status stream, and advance
        # APPROACH -> LIFT -> ... without ever requesting a HOLD on a healthy run.
        phases_n = [("approach", 0.8), ("lift", 0.8)]
        total_n = sum(d for _, d in phases_n)
        n0 = len(driver.samples)
        exec_n_log = os.path.join(out_dir, "executor_n.log")
        executor_n = Child(
            "executor_n",
            ["ros2", "run", PROBE_PKG, EXECUTOR_EXE, "--ros-args",
             "-p", "status_stale_timeout_s:=0.5",
             "-p", "approach_timeout_s:=30.0",
             "-p", "trajectory_duration_s:=%s" % total_n,
             "-p", "artifacts_dir:=" + os.path.join(out_dir, "artifacts"),
             "-p", "run_id:=run_n"],
            env, exec_n_log)
        children.append(executor_n)
        ex_cli = h.node.create_client(Trigger, EXECUTOR_SRV)
        ex_ok = h.wait_service(ex_cli, 25.0, "executor start")
        if not ex_ok:
            info("executor ~/start never appeared: alive=%s rc=%s log=%r"
                 % (executor_n.alive(), executor_n.proc.poll(),
                    (open(exec_n_log).read()[-1200:] if os.path.exists(exec_n_log) else "")))
        h.spin_for(0.35)
        ex_res = h.trigger(ex_cli, 10.0) if ex_ok else None
        info("executor start -> success=%s msg=%r"
             % (None if ex_res is None else ex_res.success,
                None if ex_res is None else ex_res.message))
        sres = h.start_transport(phases_n, 20.0)
        check("N0", sres is not None and sres.accepted,
              "controller accepted the 2-phase adapter leg",
              "message=%r" % (None if sres is None else sres.message))
        h.spin_until(lambda: len(driver.samples) > n0
                     and driver.samples[-1]["state"] in ("COMPLETE", "FAULT", "ABORT", "HOLD"),
                     12.0, "adapter leg terminal")
        h.spin_for(0.6)
        executor_n.stop()
        log_n = open(exec_n_log).read() if os.path.exists(exec_n_log) else ""
        trans_n = re.findall(r"state (\w+) -> (\w+) \(code '([^']*)'\)", log_n)
        info("executor_n transitions: %s" % (trans_n,))
        lost = [t for t in trans_n if t[1] == "HOLD" and t[2] in ("executor_lost", "stale_state")]
        check("N1", not lost,
              "adapter consumes the E8 heartbeat and does not stall on a stale-status false positive",
              "transitions=%s" % (trans_n,))
        advanced = any(to in ("LIFT", "TRANSPORT", "LOWER", "COMPLETE") for _, to, _ in trans_n)
        check("N2", advanced,
              "adapter advances phases from the controller-reported index (PhaseDriver attached)",
              "transitions=%s" % (trans_n,))
        learned = "phase table learned from the controller" in log_n
        check("N3", learned,
              "adapter learns the controller's phase table from TransportStatus instead of duplicating it",
              "learned=%s" % (learned,))
        held = "requesting controller HOLD" in log_n
        check("N4", not held,
              "a healthy run never asks the controller to freeze",
              "hold_requested=%s" % (held,))
        h.reset_controller()

        # ---- T: status wire period vs the staleness threshold -----------
        deltas = [samples_a[i]["t"] - samples_a[i - 1]["t"]
                  for i in range(1, len(samples_a))]
        med = statistics.median(deltas) if deltas else 0.0
        check("T1", med > 0.024,
              "status period (%.3f s) is far above the OLD 24 ms threshold" % med,
              "min=%.4f median=%.4f max=%.4f n=%d"
              % (min(deltas) if deltas else 0, med,
                 max(deltas) if deltas else 0, len(deltas)))

        # T2: the shipped config file must be usable as-is (B2 regression guard).
        cfg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "config", "c43_s4_executor.yaml")
        executor2 = Child(
            "executor_prod",
            ["ros2", "run", PROBE_PKG, EXECUTOR_EXE, "--ros-args",
             "--params-file", cfg_path,
             "-p", "trajectory_duration_s:=4.6",
             "-p", "artifacts_dir:=" + os.path.join(out_dir, "artifacts"),
             "-p", "run_id:=run_prod_yaml"],
            env, os.path.join(out_dir, "executor_prod.log"))
        children.append(executor2)
        cli2 = h.node.create_client(Trigger, EXECUTOR_SRV)
        h.wait_service(cli2, 20.0, "executor2")
        h.spin_for(0.4)
        res2 = h.trigger(cli2, 10.0)
        h.spin_for(0.6)
        executor2.stop()
        prod_log = open(os.path.join(out_dir, "executor_prod.log")).read()
        prod_trans = re.findall(r"state (\w+) -> (\w+) \(code '([^']*)'\)", prod_log)
        prod_ok = (res2 is not None and res2.success
                   and not any(code == "precheck_failed" for _, _, code in prod_trans))
        check("T2", prod_ok,
              "the shipped config file (status_stale_timeout_s=0.3) tracks a 10 Hz status stream",
              "start success=%s msg=%r transitions=%s cfg=%s"
              % (None if res2 is None else res2.success,
                 None if res2 is None else res2.message, prod_trans, cfg_path))

        # T3: re-enact the OLD 24 ms value so the defect stays reproducible.
        executor3 = Child(
            "executor_stale024",
            ["ros2", "run", PROBE_PKG, EXECUTOR_EXE, "--ros-args",
             "--params-file", cfg_path,
             "-p", "status_stale_timeout_s:=0.024",
             "-p", "trajectory_duration_s:=4.6",
             "-p", "artifacts_dir:=" + os.path.join(out_dir, "artifacts"),
             "-p", "run_id:=run_prod_stale"],
            env, os.path.join(out_dir, "executor_stale024.log"))
        children.append(executor3)
        cli3 = h.node.create_client(Trigger, EXECUTOR_SRV)
        h.wait_service(cli3, 20.0, "executor3")
        h.spin_for(0.4)
        res3 = h.trigger(cli3, 10.0)
        h.spin_for(0.6)
        executor3.stop()
        stale_log = open(os.path.join(out_dir, "executor_stale024.log")).read()
        stale_trans = re.findall(r"state (\w+) -> (\w+) \(code '([^']*)'\)", stale_log)
        symptoms3 = (res3 is not None and not res3.success) or any(
            code in ("stale_state", "precheck_failed") for _, _, code in stale_trans)
        check("T3", symptoms3,
              "B2 re-enactment: a 24 ms threshold still fails against the 10 Hz stream",
              "start success=%s msg=%r transitions=%s"
              % (None if res3 is None else res3.success,
                 None if res3 is None else res3.message, stale_trans))

        return finish(out_dir, driver)
    finally:
        for ch in reversed(children):
            try:
                ch.stop()
            except Exception as exc:  # noqa: BLE001 - cleanup must never mask results
                info("cleanup %s: %s" % (ch.name, exc))
        try:
            h.exec.shutdown()
            h.node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def write_jsonl(path, rows):
    with open(path, "w") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")


def finish(out_dir, driver):
    fails = [r for r in RESULTS if not r["ok"]]
    report = {
        "out_dir": out_dir,
        "assertions": RESULTS,
        "passed": len(RESULTS) - len(fails),
        "failed": len(fails),
        "bugs": BUGS,
        "r2_done": len(fails) == 0 and len(BUGS) == 0,
        "failed_ids": [r["id"] for r in fails],
        "bug_ids": [b["id"] for b in BUGS],
    }
    with open(os.path.join(out_dir, "report.json"), "w") as fh:
        json.dump(report, fh, indent=1)
    print("\n=== %d passed, %d failed ===" % (report["passed"], report["failed"]),
          flush=True)
    if fails:
        print("FAILED: " + ", ".join("%s (%s)" % (r["id"], r["msg"]) for r in fails),
              flush=True)
    if fails or BUGS:
        print("R2 verdict: PARTIAL"
              + (" (assertion failures: %s)" % report["failed_ids"] if fails else "")
              + (" (defects reproduced: %s)" % report["bug_ids"] if BUGS else ""),
              flush=True)
        return 1 if fails else 3
    print("R2 verdict: DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
