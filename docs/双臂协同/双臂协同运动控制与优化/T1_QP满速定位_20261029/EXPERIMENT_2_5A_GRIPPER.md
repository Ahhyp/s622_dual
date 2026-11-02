# Experiment 2.5-A — left gripper control, no sample (2026-10-31)

Status: **2.5-A PASS; 2.5-B1 PASS; 2.5-B2: NO_CONTACT at ±0.0251 (not a grasp).**

## Setup

- Diagnostic world without sample: `c43_s4_world_no_sample.sdf` (support/table kept).
- Homing: fixed 2.4-C right-first, h=0.05, 11-point descent; QP inactive.
- Arms hold q_grasp12; left/right fingers open.
- No attach/detach, no weld, no transport, no sample.
- Direct action: `/left_hand_controller/follow_joint_trajectory`.
- 100 Hz recorder: `gripper_2_5a.csv` (1113 rows).

## Commands and results

All four goals returned:

```text
status=4 (SUCCEEDED), error_code=0, error_string='Goal successfully reached!'
```

Actual left finger positions:

| command | actual f1 | actual f2 | result |
|---|---:|---:|---|
| initial/open | 0.0305000 | -0.0304995 | open |
| OPEN 0.0305 | 0.0304724 | -0.0304713 | SUCCEEDED |
| CLOSE 0.0251 | **0.0251002** | **-0.0251001** | SUCCEEDED |
| CLOSE 0.0200 | **0.0201013** | **-0.0201013** | SUCCEEDED |
| REOPEN 0.0305 | 0.0305000 | -0.0304986 | SUCCEEDED |

The controller error is ~0.0002 mm at 0.0251 (0.0251002 - 0.0251 = 2e-7 m) and ~0.1013 mm at 0.0200; it can reach
`0.0251` and `0.0200` without a sample. Therefore the earlier `0.0251 -> 0.0271`
behaviour is **not** a JTC tracking limitation.

## Root cause found in executor

### 1. The close target is the open target

`src/gz_launch/launch/c43_s4_full_flow.launch.py`:

```python
"gripper_close_positions": [0.0305, -0.0305],
```

This is identical to the open position. `startGripper()` sends exactly this
trajectory. Thus GRASP can log "closing left fingers" while the goal is a no-op;
Experiment 3's `±0.0305` after "closed" is explained by this configuration.

### 2. The 0.0271 acceptance is the executor tolerance

`src/dual_arm_transport_executor/src/executor_node.cpp`:

```cpp
gripper_position_tol_ = declare_parameter("gripper_position_tol", 0.002);
...
if (std::fabs(gripper_positions_[i] - targets[i]) > gripper_position_tol_) return false;
```

`fingerPositionsAtTarget()` accepts any position within **2 mm** of the close
target. With target `0.0251`, `0.0271` is exactly within tolerance, so the executor
can report "fingers at close targets" before the JTC reaches the commanded value.
The JTC itself continues to be capable of reaching `0.0251`.

## Conclusion

- Left gripper JTC: **PASS** (open, close 0.0251, close 0.020, reopen all SUCCEEDED).
- Fault 1: executor close configuration is a no-op (`0.0305 == open`).
- Fault 2: executor acceptance tolerance 2 mm is too loose for a 50 mm sample
  (4 mm total gap ambiguity); it can accept 0.0271 for 0.0251.
- No sample was involved; no weld/attach.

## Recommendation for 2.5-B

1. Set `gripper_close_positions` to the intended close position, e.g.
   `[0.0251, -0.0251]` for the 50 mm sample (50.02 mm inner gap).
2. Tighten `gripper_position_tol` for the contact experiment (e.g. 0.0005–0.001 m),
   or separate "JTC command tolerance" from "grasp acceptance".
3. Only then add the sample and run the slow contact test; do not auto-attach weld.
4. Keep homing/right-first and QP untouched; this is a GRASP-only change.

## Evidence / rollback

- `gripper_2_5a.csv` (100 Hz, 1113 rows)
- `c43_gripper_probe.py`
- `c43_s4_world_no_sample.sdf`
- `/home/kz/s622_backup_20261031_5/uncommitted.diff`
- `/home/kz/s622_backup_20261031_5/untracked.tgz`

No commit / no push. Regression remains 190 tests / 0 failures from 2.4-C.


## 2.5-B1 / 2.5-B2 results (same day)

### Fixes applied

- `gripper_close_positions` default is now `[0.0251, -0.0251]`; a launch-time
  guard raises if close == open (or lengths differ).
- Executor completion now requires **all** of:
  1. JTC action result `SUCCEEDED`;
  2. actual positions within `gripper_position_tol` (now 0.0005);
  3. positions stable for `gripper_stable_window_s` (0.2 s).
  A loose tolerance match alone no longer completes GRASP.
- No homing/QP/weld/sample/GRASP target changes beyond the close target and
  acceptance logic.

### 2.5-B1 — executor close, no sample in the fingers

World: `c43_s4_world_sample_away_prod.sdf` (sample moved to 0.30,0.30 so the
startup weld chain still exists; fingers free), right-first homing, QP active,
executor-driven GRASP.

Executor log:

```text
state APPROACH -> GRASP
S4-T16: closing left fingers via /left_hand_controller/follow_joint_trajectory
S4-T16: gripper goal accepted
S4-T16: gripper goal finished: SUCCEEDED
S4-T16: fingers at close targets (0.025130, -0.025132)
state GRASP -> LIFT
```

Result: **PASS** — executor really sent 0.0251, waited for JTC SUCCEEDED and the
stable window, and the actual fingers reached `0.025130/-0.025132`.

### 2.5-B2 — sample present, slow close to ±0.0251

World: `c43_s4_world_contact.sdf` (original 50 mm sample at center), right-first
homing, QP inactive, **direct slow left JTC** (no attach/weld), 5 s close.

```text
OPEN 0.0305: SUCCEEDED
SLOW_CLOSE 0.0251: SUCCEEDED, error_code=0
actual finger plateau: +0.0251749 / -0.0251749
inner gap: 2 * 0.0251749 = 50.3498 mm
sample width: 50.00 mm
robot-sample contacts: 0
sample pose before/after: (0,0,0.274999773) -> (0,0,0.274999773)
```

Result: **NO_CONTACT**. The actual inner gap is ~0.35 mm larger than the sample,
so the pads do not touch; the sample does not move. This is **not** a grasp
success, and the target was not reduced further (no 0.0245/0.0230 push).

The theoretical 50.02 mm at 0.0251 is not reached exactly: the measured plateau
is 50.35 mm (0.075 mm per side error). A contact-limited close would need a target
slightly below 0.0251, with contact detection and immediate cancellation.

### Updated status

| item | status |
|---|---|
| 2.5-A JTC | PASS |
| executor close parameter | FIXED |
| executor completion judgment | FIXED (JTC result + tight tolerance + stable window) |
| 2.5-B1 executor close, no sample | PASS |
| 2.5-B2 sample contact at ±0.0251 | **NO_CONTACT** (sample stable; no attach) |
| real friction grasp | NOT_EVALUATED |
| W7 integration | INTEGRATION_HOLD |

Regression: **190 tests / 0 failures**.

Evidence: `gripper_2_5b.csv`, `/tmp/b2_contact.txt`, `/tmp/b1_world.log`,
`/tmp/b2_world.log`.
