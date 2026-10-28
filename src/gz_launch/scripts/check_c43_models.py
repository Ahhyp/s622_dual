#!/usr/bin/env python3
"""W5-A 静态校核：c43_sample / c43_support 与 S2 冻结基线的一致性。

只做静态检查，不启动 Gazebo、不进入物理仿真。分层报告：

  [XML]      两个 model.config 与两个 model.sdf 的 XML 可解析性
  [SDFORMAT] 若本机存在 `ign sdf -k` / `gz sdf -k`，用它做 SDFormat 结构校验
  [FROZEN]   与 src/dual_arm_transport/config/transport_baseline.yaml 的数值对照
  [MODEL]    动态/静态、碰撞几何、惯量自洽、模型坐标系约定
  [URI]      是否存在 model:// 之类的外部资源依赖（本轮两个模型都不应有）

Usage:
  python3 src/gz_launch/scripts/check_c43_models.py [--models DIR] [--baseline FILE]
                                                    [--share DIR] [--json]

Exit codes: 0 = 全部通过；1 = 有 FAIL；2 = 输入缺失（模型或基线不存在）。
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT = Path(__file__).resolve()


def find_repo_root(start: Path) -> Path:
    for cand in [start, *start.parents]:
        if (cand / "docs/双臂协同").is_dir() and (cand / "src").is_dir():
            return cand
    return start.parents[3]


REPO = find_repo_root(SCRIPT)
DEFAULT_MODELS = REPO / "src/gz_launch/models"
DEFAULT_BASELINE = REPO / "src/dual_arm_transport/config/transport_baseline.yaml"

ABS_TOL = 1e-9
REL_TOL = 1e-6

results: list[tuple[str, str, str]] = []  # (layername, PASS/FAIL/SKIP, message)


def record(layer: str, ok: bool | None, msg: str) -> None:
    if ok is None:
        results.append((layer, "SKIP", msg))
    else:
        results.append((layer, "PASS" if ok else "FAIL", msg))


def close(a: float, b: float, rel: float = REL_TOL, abs_tol: float = ABS_TOL) -> bool:
    return math.isclose(a, b, rel_tol=rel, abs_tol=abs_tol)


def parse_yaml_scalars(path: Path) -> dict:
    """极简解析：只取本检查需要的几个标量/序列，避免引入 PyYAML 之外的契约依赖。

    仓库测试环境已有 PyYAML（W4 台账检查器在用），优先用它。
    """
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:  # pragma: no cover
        raise SystemExit("PyYAML is required (already used by the W4 ledger checker)")
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def layer_xml(models: Path) -> None:
    for rel in ("c43_sample/model.config", "c43_sample/model.sdf",
                "c43_support/model.config", "c43_support/model.sdf"):
        p = models / rel
        if not p.is_file():
            record("XML", False, f"missing file {p}")
            continue
        try:
            ET.parse(p)
            record("XML", True, f"{rel}: well-formed XML")
        except ET.ParseError as exc:
            record("XML", False, f"{rel}: XML parse error: {exc}")

    # model.config 的字段与被引用 SDF 的存在性
    for name in ("c43_sample", "c43_support"):
        cfg = models / name / "model.config"
        if not cfg.is_file():
            continue
        root = ET.parse(cfg).getroot()
        declared = (root.findtext("name") or "").strip()
        sdf_ref = (root.findtext("sdf") or "").strip()
        record("XML", declared == name, f"{name}/model.config <name> == directory name ({declared!r})")
        sdf_path = models / name / sdf_ref if sdf_ref else None
        record("XML", bool(sdf_path and sdf_path.is_file()),
               f"{name}/model.config -> {sdf_ref!r} exists")


def layer_sdformat(models: Path) -> None:
    exe = shutil.which("ign") or shutil.which("gz")
    if not exe:
        record("SDFORMAT", None, "no `ign`/`gz` binary: XML check only, SDFormat check NOT RUN")
        return
    for name in ("c43_sample", "c43_support"):
        sdf = models / name / "model.sdf"
        proc = subprocess.run([exe, "sdf", "-k", str(sdf)], capture_output=True, text=True)
        ok = proc.returncode == 0 and "Valid" in (proc.stdout + proc.stderr)
        record("SDFORMAT", ok,
               f"{name}: `{Path(exe).name} sdf -k` rc={proc.returncode} "
               f"{(proc.stdout + proc.stderr).strip().splitlines()[-1] if (proc.stdout + proc.stderr).strip() else ''}")


def sample_numbers(models: Path) -> dict:
    root = ET.parse(models / "c43_sample/model.sdf").getroot()
    model = root.find("model")
    link = model.find("link")
    inertial = link.find("inertial")
    size = [float(v) for v in link.find("collision/geometry/box/size").text.split()]
    mass = float(inertial.findtext("mass"))
    inertia = {k: float(inertial.findtext(f"inertia/{k}") or 0.0)
               for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    return {
        "static": (model.findtext("static") or "false").strip().lower() == "true",
        "size": size,
        "mass": mass,
        "inertia": inertia,
        "collision_size": size,
        "visual_size": [float(v) for v in link.find("visual/geometry/box/size").text.split()],
        "link_pose": [float(v) for v in (link.findtext("pose") or "0 0 0 0 0 0").split()],
    }


def support_numbers(models: Path) -> dict:
    root = ET.parse(models / "c43_support/model.sdf").getroot()
    model = root.find("model")
    link = model.find("link")
    pose = [float(v) for v in (link.findtext("pose") or "0 0 0 0 0 0").split()]
    radius = float(link.find("collision/geometry/cylinder/radius").text)
    length = float(link.find("collision/geometry/cylinder/length").text)
    vradius = float(link.find("visual/geometry/cylinder/radius").text)
    vlength = float(link.find("visual/geometry/cylinder/length").text)
    return {
        "static": (model.findtext("static") or "false").strip().lower() == "true",
        "link_pose_z": pose[2],
        "radius": radius,
        "length": length,
        "visual_radius": vradius,
        "visual_length": vlength,
        "has_inertial": link.find("inertial") is not None,
    }


def layer_frozen(models: Path, baseline: Path) -> None:
    base = parse_yaml_scalars(baseline)
    obj = base["object"]
    sup = base["support"]

    s = sample_numbers(models)
    record("FROZEN", all(close(a, b) for a, b in zip(s["size"], obj["size_xyz"])),
           f"sample size {s['size']} == S2 object.size_xyz {obj['size_xyz']}")
    record("FROZEN", close(s["mass"], float(obj["mass"])),
           f"sample mass {s['mass']} == S2 object.mass {obj['mass']}")

    # 均匀密度长方体惯量（S2 未给出惯量，这是导出值）
    a, b, c = s["size"]
    m = s["mass"]
    want = {"ixx": m / 12.0 * (b * b + c * c),
            "iyy": m / 12.0 * (a * a + c * c),
            "izz": m / 12.0 * (a * a + b * b)}
    for k, v in want.items():
        record("FROZEN", close(s["inertia"][k], v),
               f"sample inertia {k} = {s['inertia'][k]:.9e} (uniform box {v:.9e})")

    u = support_numbers(models)
    record("FROZEN", close(u["radius"], float(sup["radius_m"])),
           f"support radius {u['radius']} == S2 {sup['radius_m']}")
    record("FROZEN", close(u["length"], float(sup["height_m"])),
           f"support height {u['length']} == S2 {sup['height_m']}")
    record("FROZEN", close(u["visual_radius"], u["radius"]) and close(u["visual_length"], u["length"]),
           "support visual geometry == collision geometry")

    # 名义 z 链（world 侧放置：support 以 pose 0 0 0 引用）
    top_z = u["link_pose_z"] + u["length"] / 2.0
    record("FROZEN", close(top_z, float(sup["top_z_m"])),
           f"support top z = link_pose_z + h/2 = {top_z:.9f} == S2 top_z_m {sup['top_z_m']}")

    world_T = obj["world_T_object"]
    obj_z = float(world_T.split(";")[2].split(",")[3])
    record("FROZEN", close(obj_z, float(sup["top_z_m"]) + s["size"][2] / 2.0),
           f"nominal object centre z {obj_z} == support top {sup['top_z_m']} + h/2 {s['size'][2] / 2.0}")
    record("FROZEN", s["size"][2] == obj["size_xyz"][2],
           f"object bottom (centre - h/2) == support top: "
           f"{obj_z - s['size'][2] / 2.0:.9f} == {float(sup['top_z_m']):.9f}")


def layer_model(models: Path) -> None:
    s = sample_numbers(models)
    record("MODEL", not s["static"], "sample is a dynamic rigid body (no <static>true</static>)")
    record("MODEL", close(s["collision_size"][0], s["visual_size"][0]) and
           close(s["collision_size"][1], s["visual_size"][1]) and
           close(s["collision_size"][2], s["visual_size"][2]),
           "sample collision box size == visual box size")
    record("MODEL", all(abs(v) < 1e-12 for v in
                        (s["inertia"]["ixy"], s["inertia"]["ixz"], s["inertia"]["iyz"])),
           "sample inertia products of inertia are zero (axis-aligned box)")
    principal = [s["inertia"][k] for k in ("ixx", "iyy", "izz")]
    record("MODEL", all(v > 0.0 for v in principal) and s["mass"] > 0.0,
           f"sample mass {s['mass']} kg and principal inertias "
           f"{['%.3e' % v for v in principal]} are positive")

    u = support_numbers(models)
    record("MODEL", u["static"], "support is static (fixture/stand, not a free body)")
    record("MODEL", not u["has_inertial"],
           "support declares no <inertial> (static links are not given fabricated mass values)")
    record("MODEL", close(u["link_pose_z"], u["length"] / 2.0),
           f"support model frame = base centre (link z {u['link_pose_z']} = h/2 {u['length'] / 2.0})")


def layer_uri(models: Path) -> None:
    for name in ("c43_sample", "c43_support"):
        text = (models / name / "model.sdf").read_text(encoding="utf-8")
        has_uri = "model://" in text or "<uri>" in text
        record("URI", not has_uri, f"{name}/model.sdf has no external <uri> dependency")


def layer_share(models: Path, share: Path | None) -> None:
    if share is None:
        record("MODEL", None, "no --share given: skipped install-path visibility check")
        return
    for name in ("c43_sample", "c43_support"):
        p = share / "models" / name / "model.sdf"
        record("MODEL", p.exists(), f"installed model visible: {p}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", default=str(DEFAULT_MODELS))
    ap.add_argument("--baseline", default=str(DEFAULT_BASELINE))
    ap.add_argument("--share", default=None,
                    help="gz_launch share dir (…/share/gz_launch) to check installed model paths")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    models = Path(args.models)
    baseline = Path(args.baseline)
    if not (models / "c43_sample/model.sdf").is_file() or not (models / "c43_support/model.sdf").is_file():
        print(f"INPUT_ERROR: models not found under {models}", file=sys.stderr)
        return 2
    if not baseline.is_file():
        print(f"INPUT_ERROR: frozen baseline not found: {baseline}", file=sys.stderr)
        return 2

    layer_xml(models)
    layer_sdformat(models)
    layer_frozen(models, baseline)
    layer_model(models)
    layer_uri(models)
    layer_share(models, Path(args.share) if args.share else None)

    failed = [r for r in results if r[1] == "FAIL"]
    if args.json:
        print(json.dumps({
            "schema": "c43s4.w5a.model_check.v1",
            "models": str(models),
            "baseline": str(baseline),
            "results": [{"layer": l, "status": s, "detail": m} for l, s, m in results],
            "passed": sum(1 for r in results if r[1] == "PASS"),
            "failed": len(failed),
            "skipped": sum(1 for r in results if r[1] == "SKIP"),
        }, ensure_ascii=False, indent=2))
    else:
        width = max(len(r[2]) for r in results)
        for layer, status, msg in results:
            print(f"[{layer:<9}] {status}  {msg}")
        print("-" * 78)
        print(f"checks={len(results)} passed={sum(1 for r in results if r[1] == 'PASS')} "
              f"failed={len(failed)} skipped={sum(1 for r in results if r[1] == 'SKIP')}")
        if failed:
            print("VERDICT: W5A_MODEL_CHECK_FAIL — 这是静态检查，不是 Gazebo 物理加载/静置验证")
        else:
            print("VERDICT: W5A_MODEL_CHECK_PASS — 仅静态结构与冻结参数一致性；"
                  "不代表 Gazebo 加载或 5 s 静置通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
