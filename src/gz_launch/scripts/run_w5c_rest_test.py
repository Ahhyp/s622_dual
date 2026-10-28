#!/usr/bin/env python3
"""W5-C 静置仿真验收（C4.3 S4 / W5-C 交付物）

在 C4.3 专用 world 中，让 c43_sample 在 c43_support 上静置，按**仿真时间**采集
至少 5.0 s 的姿态观测，输出漂移/误差与原始证据。

设计约束（W5 v1.1 §4 C）：
  * 不运行机械臂控制、不发 attach/detach、不锁定样件位姿；
  * 不关闭重力、不暂停物理；
  * 以 Gazebo 仿真时钟（/world/<world>/stats 的 sim_time）判定测试区间，
    而不是墙钟；
  * 保留原始观测与命令，`≤ 2 mm` 是**未批准建议值**，本脚本只报告实测值，
    不产生任何"正式 PASS"结论。

用法：
  python3 src/gz_launch/scripts/run_w5c_rest_test.py \
      --outdir docs/双臂协同/双臂协同运动控制与优化/W5-C_静置仿真证据_20261028 \
      --duration 5.0

退出码：0=观测完成且区间≥duration（不代表阈值 PASS）；2=环境/加载失败；3=观测不足。
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import tempfile
import signal
import subprocess
import sys
import time

WORLD_NAME = "c43_s4_world"
SAMPLE = "c43_sample"
SUPPORT = "c43_support"
TABLE = "c43_table"
POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
STATS_TOPIC = f"/world/{WORLD_NAME}/stats"
CONTACTS_TOPIC = f"/world/{WORLD_NAME}/contacts"


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def run(cmd, env, timeout=20):
    try:
        p = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "TIMEOUT"


def stats_sample(env, timeout=8):
    """返回 (sim_time_s, real_time_s, iterations, rtf) 或 None。"""
    rc, out = run(["ign", "topic", "-e", "-t", STATS_TOPIC, "-n", "1"], env, timeout)
    if rc != 0 or "sim_time" not in out:
        return None
    def _t(block):
        # protobuf 文本输出会省略零值字段：sec=0 时没有 `sec:` 行；
        # 且 `sec:` 会命中 `nsec:` 的子串，必须用 (?<!n) 排除。
        sec = re.search(r"(?<!n)sec:\s*(\d+)", block)
        nsec = re.search(r"nsec:\s*(\d+)", block)
        if not nsec:
            return None
        return (int(sec.group(1)) if sec else 0) + int(nsec.group(1)) / 1e9
    sim = re.search(r"sim_time\s*\{(.*?)\}", out, re.S)
    real = re.search(r"real_time\s*\{(.*?)\}", out, re.S)
    it = re.search(r"iterations:\s*(\d+)", out)
    rtf = re.search(r"real_time_factor:\s*([0-9.eE+-]+)", out)
    return {
        "sim_time_s": _t(sim.group(1)) if sim else None,
        "real_time_s": _t(real.group(1)) if real else None,
        "iterations": int(it.group(1)) if it else None,
        "real_time_factor": float(rtf.group(1)) if rtf else None,
    }


def model_names(env, timeout=20):
    rc, out = run(["ign", "model", "--list"], env, timeout)
    if rc != 0:
        return []
    return [ln.strip()[2:].strip() for ln in out.splitlines() if ln.strip().startswith("- ")]


def parse_pose_info(text):
    """解析 `ign topic -e` 的 protobuf 文本输出。

    每条 pose/info 消息以 `header { stamp {...} }` 开头，随后是若干
    `pose { name: "..." position {...} orientation {...} }` 块。
    返回 [{"t": sim_time_s, "name": ..., x,y,z,qw,qx,qy,qz}]。
    """
    import math  # noqa: F401  (保持函数自足)
    stamped = []
    for msg in re.split(r"(?=header\s*\{)", text):
        st = re.search(r"stamp\s*\{(.*?)\}", msg, re.S)
        if not st:
            continue
        sec = re.search(r"(?<!n)sec:\s*(\d+)", st.group(1))
        nsec = re.search(r"nsec:\s*(\d+)", st.group(1))
        if not nsec:
            continue
        t = (int(sec.group(1)) if sec else 0) + int(nsec.group(1)) / 1e9
        for m in re.finditer(r"pose\s*\{(.*?)\n\}", msg, re.S):
            block = m.group(1)
            name = re.search(r'name:\s*"([^"]+)"', block)
            if not name:
                continue
            pos = re.search(r"position\s*\{(.*?)\}", block, re.S)
            ori = re.search(r"orientation\s*\{(.*?)\}", block, re.S)
            pos = pos.group(1) if pos else None
            ori = ori.group(1) if ori else None

            def _f(blk, key, default=0.0):
                if not blk:
                    return default
                mm = re.search(key + r":\s*([-0-9.eE+]+)", blk)
                return float(mm.group(1)) if mm else default

            stamped.append({
                "t": t, "name": name.group(1),
                "x": _f(pos, "x"), "y": _f(pos, "y"), "z": _f(pos, "z"),
                "qw": _f(ori, "w", 1.0), "qx": _f(ori, "x"),
                "qy": _f(ori, "y"), "qz": _f(ori, "z"),
            })
    return stamped


def quat_angle(qw, qx, qy, qz, rw, rx, ry, rz):
    dot = abs(qw * rw + qx * rx + qy * ry + qz * rz)
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * __import__("math").acos(dot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=None)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=0.5, help="先等待的仿真秒数（接触稳定）")
    ap.add_argument("--sim-timeout", type=float, default=180.0)
    ap.add_argument("--gz-home", default=None, help="Gazebo HOME（默认临时目录，避免写仓库）")
    args = ap.parse_args()

    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    world = args.world or os.path.join(repo, "src", "gz_launch", "worlds", "c43_s4_world.sdf")
    models_dir = os.path.join(repo, "src", "gz_launch", "models")
    os.makedirs(args.outdir, exist_ok=True)
    # Gazebo 的 HOME 下会写 ~/.ignition 日志；默认放临时目录，避免污染仓库证据目录。
    home = args.gz_home or tempfile.mkdtemp(prefix="w5c_gz_home_")
    os.makedirs(home, exist_ok=True)

    env = dict(os.environ)
    env["HOME"] = home
    env["IGN_GAZEBO_RESOURCE_PATH"] = models_dir + ":" + env.get("IGN_GAZEBO_RESOURCE_PATH", "")

    gz_ver_rc, gz_ver = run(["ign", "gazebo", "--versions"], env)
    kernel = subprocess.run(["uname", "-r"], capture_output=True, text=True).stdout.strip()
    cmdline = ""
    try:
        with open("/proc/cmdline") as fh:
            cmdline = fh.read().strip()
    except OSError:
        pass

    hashes = {"world": sha256_file(world)}
    for name in ("c43_sample", "c43_support", "table"):
        for f in ("model.config", "model.sdf"):
            p = os.path.join(models_dir, name, f)
            hashes[f"{name}/{f}"] = sha256_file(p) if os.path.exists(p) else "MISSING"
    hashes["script"] = sha256_file(os.path.abspath(__file__))

    server_log = os.path.join(args.outdir, "gazebo_server.log")
    server = subprocess.Popen(
        ["ign", "gazebo", "-s", "-r", "-v", "3", world],
        env=env, stdout=open(server_log, "w"), stderr=subprocess.STDOUT,
    )
    result = {
        "schema": "c43s4.w5c.rest_test.v1", "world": world, "duration_requested_s": args.duration,
        "gazebo": {"versions": gz_ver.strip(), "versions_rc": gz_ver_rc},
        "machine": {"kernel": kernel, "wsl": "microsoft-standard" in kernel, "cmdline": cmdline},
        "hashes": hashes, "commands": [], "status": "INIT",
    }
    try:
        t0 = time.time()
        names = []
        while time.time() - t0 < 60.0:
            names = model_names(env)
            if all(n in names for n in (SAMPLE, SUPPORT, TABLE)):
                break
            time.sleep(0.5)
        if not all(n in names for n in (SAMPLE, SUPPORT, TABLE)):
            result.update(status="LOAD_FAILED", models=names)
            return 2
        result["models_loaded"] = names

        # 接触稳定等待（仿真时间）
        st = stats_sample(env)
        while st and st["sim_time_s"] is not None and st["sim_time_s"] < args.settle:
            time.sleep(0.2)
            st = stats_sample(env)
        t_start = stats_sample(env)["sim_time_s"]

        raw_path = os.path.join(args.outdir, "pose_info_raw.txt")
        rec = subprocess.Popen(["ign", "topic", "-e", "-t", POSE_TOPIC],
                               env=env, stdout=open(raw_path, "w"), stderr=subprocess.DEVNULL)
        result["commands"].append(f"ign topic -e -t {POSE_TOPIC} > pose_info_raw.txt")

        sim = t_start
        wall_deadline = time.time() + args.sim_timeout
        while sim is not None and sim - t_start < args.duration and time.time() < wall_deadline:
            time.sleep(0.2)
            s = stats_sample(env)
            if s and s["sim_time_s"] is not None:
                sim = s["sim_time_s"]
        t_end = sim
        time.sleep(0.3)
        rec.send_signal(signal.SIGINT)
        try:
            rec.wait(timeout=10)
        except subprocess.TimeoutExpired:
            rec.kill()

        with open(raw_path, "r", errors="replace") as fh:
            text = fh.read()
        poses = parse_pose_info(text)
        window = [p for p in poses if t_start - 0.05 <= p["t"] <= (t_end or 0) + 0.05]
        sample_poses = sorted([p for p in window if p["name"] == SAMPLE], key=lambda p: p["t"])
        support_poses = sorted([p for p in window if p["name"] == SUPPORT], key=lambda p: p["t"])
        table_poses = sorted([p for p in window if p["name"] == TABLE], key=lambda p: p["t"])

        if len(sample_poses) < 3:
            result.update(status="INSUFFICIENT_SAMPLES", n_sample_poses=len(sample_poses),
                          t_start_sim=t_start, t_end_sim=t_end)
            return 3

        values = sample_poses
        xs = [p["x"] for p in values]; ys = [p["y"] for p in values]; zs = [p["z"] for p in values]
        ref = values[0]
        angles = [quat_angle(p["qw"], p["qx"], p["qy"], p["qz"], ref["qw"], ref["qx"], ref["qy"], ref["qz"])
                  for p in values]
        n_samples = len(values)
        steps = [values[i + 1]["t"] - values[i]["t"] for i in range(n_samples - 1)]
        speeds = []
        for i in range(n_samples - 1):
            dt = steps[i]
            if dt <= 0:
                continue
            dx = values[i + 1]["x"] - values[i]["x"]
            dy = values[i + 1]["y"] - values[i]["y"]
            dz = values[i + 1]["z"] - values[i]["z"]
            speeds.append(((dx * dx + dy * dy + dz * dz) ** 0.5) / dt)
        nominal = (0.0, 0.0, 0.275)
        summary = {
            "n_sample_poses": n_samples,
            "sim_window": {"start_s": t_start, "end_s": t_end, "span_s": (t_end - t_start) if t_end else None},
            "sample_rate_hz": (n_samples / (t_end - t_start)) if t_end and t_end > t_start else None,
            "sample_rate_note": "pose/info 发布频率（非人工采样）",
            "position": {
                "first": {"x": xs[0], "y": ys[0], "z": zs[0]},
                "last": {"x": xs[-1], "y": ys[-1], "z": zs[-1]},
                "x_range": [min(xs), max(xs)], "y_range": [min(ys), max(ys)], "z_range": [min(zs), max(zs)],
                "max_horizontal_drift_m": max((x * x + y * y) ** 0.5 for x, y in zip(xs, ys)),
                "total_horizontal_displacement_m": ((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2) ** 0.5,
                "max_abs_z_error_m": max(abs(z - nominal[2]) for z in zs),
                "min_abs_z_error_m": min(abs(z - nominal[2]) for z in zs),
            },
            "orientation": {
                "max_angle_from_first_rad": max(angles),
                "max_angle_from_first_deg": max(angles) * 180.0 / 3.141592653589793,
            },
            "velocity": {
                "max_speed_estimate_m_s": max(speeds) if speeds else None,
                "mean_speed_estimate_m_s": (sum(speeds) / len(speeds)) if speeds else None,
                "note": "由相邻 pose/info 消息的中心位移除以时间戳差估算，不含角速度",
            },
            "support_pose_variation": {
                "n": len(support_poses),
                "max_abs_dz": max((abs(p["z"]) for p in support_poses), default=None),
                "max_abs_dxy": max((abs(p["x"]) + abs(p["y"]) for p in support_poses), default=None),
            },
            "table_pose": ({"x": table_poses[0]["x"], "z": table_poses[0]["z"]} if table_poses else None),
            "nominal_center_m": nominal,
        }

        csv_path = os.path.join(args.outdir, "pose_samples.csv")
        with open(csv_path, "w") as fh:
            fh.write("sim_time_s,x,y,z,qw,qx,qy,qz\n")
            for p in values:
                fh.write(f"{p['t']:.6f},{p['x']:.9f},{p['y']:.9f},{p['z']:.9f},"
                         f"{p['qw']:.9f},{p['qx']:.9f},{p['qy']:.9f},{p['qz']:.9f}\n")
        result["commands"].append(f"ign gazebo -s -r -v 3 {world}")
        result["stats"] = stats_sample(env)
        result["summary"] = summary
        result["evidence"] = {"pose_samples_csv": "pose_samples.csv", "raw_pose_info": "pose_info_raw.txt",
                              "server_log": "gazebo_server.log"}
        span = summary["sim_window"]["span_s"]
        result["status"] = "OBSERVED" if (span is not None and span >= args.duration) else "WINDOW_TOO_SHORT"
        with open(os.path.join(args.outdir, "summary.json"), "w") as fh:
            json.dump(result, fh, indent=2, ensure_ascii=False)
        print(json.dumps({"status": result["status"], "sim_window": summary["sim_window"],
                          "n_sample_poses": n_samples,
                          "position": summary["position"], "orientation": summary["orientation"],
                          "velocity": summary["velocity"]}, indent=2, ensure_ascii=False))
        return 0 if result["status"] == "OBSERVED" else 3
    finally:
        if server.poll() is None:
            server.send_signal(signal.SIGINT)
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill()
        result["server_returncode"] = server.returncode
        try:
            with open(os.path.join(args.outdir, "summary.json"), "w") as fh:
                json.dump(result, fh, indent=2, ensure_ascii=False)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
