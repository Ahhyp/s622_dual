# Experiment 2.4 — vertical pre-grasp homing (2026-10-31)

Status: **2.4-C PASS: sample contact 3/3; full collision 0 illegal; min cross-arm 19.36 mm; min world 5.39 mm**.

## 1. q_grasp12 TCP (FK)

`fairino_planning_core::DHKinematics` with `ToolModel::GRIPPER`:

| arm | TCP in arm base (m) | TCP in world (m) | orientation |
|---|---|---|---|
| left | `(0.3003, 0, 0.3000)` | `(0.0497, 0, 0.3000)` | unchanged |
| right | `(0.3003, 0, 0.3000)` | `(-0.0497, 0, 0.3000)` | unchanged |

World base: left `x=+0.35, yaw=π`; right `x=-0.35, yaw=0`.

## 2. h and pre_grasp

Candidate `h = 0.05 m` (TCP world Z + 0.05, X/Y/orientation unchanged).

IK: `FairinoIK::solve(Transform4d, ToolModel::GRIPPER)`, choose solution closest to
q_grasp12. All h ∈ {0.03, 0.05, 0.07, 0.10} were reachable; h=0.05 chosen for the
first collision test. Joint distance to q_grasp12: **0.420 rad** per arm.

```text
left  pre_grasp = [2.795036910, -1.604022235, 0.945042433, -0.911816525, -1.570796327, -0.346555744]
right pre_grasp = [0.346555744, -1.537570419, -0.945042433, -2.229776129, 1.570796327, -2.795036910]
```

Pre-grasp TCP world = q_grasp12 TCP + `(0,0,0.05)`.

Targets are written by the homing script to
`pre_grasp_targets.json` at send time (JTC controller_state did not expose reference,
so this is the explicit target record).

## 3. Two-stage homing

- Stage 1: `dual_home → pre_grasp`, JTC, 8 s.
- Stage 2: `pre_grasp → q_grasp12`, **11 IK waypoints** at 5 mm Z steps, JTC, 6 s.
  This is an IK-sampled Cartesian descent, not a claim that joint interpolation is straight.
- Fingers open throughout; no QP, no attach/detach, no weld.
- `C43_PRE_GRASP_HOMING=1 C43_HOMING_NO_QP=1`.
- Diagnostic world: `c43_s4_world_contact.sdf`; sample model `c43_sample_contact`
  with contact sensor. Sample name in the diagnostic world is `c43_sample_contact`
  so the startup DetachableJoint cannot attach to `c43_sample`; this removes the
  startup weld. Consequence: executor logs `weld_state_unknown`; no weld exists.
  Evidence limitation: no live weld-state sample; absence of weld is from the
  naming/command setup and the sample not following the arm.

## 4. Results (3 runs)

All runs: stage-1 and stage-2 JTC logs `Goal reached, success!`; fingers open;
robot–sample contact count from the sample contact sensor = **0**.

| run | sample initial (m) | sample final (m) | robot contacts | JTC | notes |
|---|---|---|---|---|---|
| pre2 | `(0,0,0.274999773)` | `(0,0,0.274999773)` | 0 | both stages success | 100 Hz trace: 2996 sample rows, 1 unique pose |
| pre3 | `(0,0,0.274999773)` | `(0,0,0.274999773)` | 0 | both stages success | final pose queried from Gazebo |
| pre4 | `(0,0,0.274999773)` | `(0,0,0.274999773)` | 0 | both stages success | final joints match q_grasp12 |

Final joint check (pre4, actual / q_grasp12):

```text
left  = 2.795036910, -1.706950858, 1.279963524, -1.143808993, -1.570796327, -0.346555744
right = 0.346555744, -1.434641796, -1.279963524, -1.997783660, 1.570796327, -2.795036910
```

Fingers: `+0.0305 / -0.0305` on both hands.

Sample offset over the 100 Hz pre2 trace: **0** (all 2996 sampled poses identical).
Support contacts (normal resting contacts) were present; they are not
robot–sample contacts.

## 5. What is verified and what is not

Verified:

- pre_grasp generation/FK/IK reachability and continuity.
- Two-stage JTC execution and final q_grasp12 accuracy.
- Fingers open.
- **0 robot–sample contacts** in 3 runs via the sample contact sensor.
- Sample remains at `(0,0,0.274999773)`; no observed drift.

NOT verified / failed:

- The offline MoveIt/FCL checker **was** implemented (`c43_path_check`) using the
  C4.3 Gazebo xacro URDF, SRDF, sample, support and floor. Fingers are explicitly
  opened to ±0.0305 in the RobotState (MoveIt's default finger value is 0.0,
  which produced false sample collisions until fixed).
- h=0.05 (the Gazebo-validated path): **2 arm-arm collision samples** at
  `stage1 89/100`, pair `right_finger2 <-> left_finger1`; world collisions 0;
  min robot-world distance 5.39 mm.
- h=0.10 (offline alternate): arm-arm collisions 0, but min arm-arm clearance
  only **0.170 mm** at the same stage1 89/100 pair; still not a defensible margin.
- The whole-robot self-collision count (891) is a known false-positive scope:
  `isStateCollidingWholeRobot` includes adjacent/attached link pairs not covered by
  the dual_arm SRDF ACM; the meaningful cross-arm query gives the 2/0 values above.
  We did not expand the SRDF exemptions.
- Therefore 2.4 is **not** a full PASS; the failed/near-contact segment is stage1
  near pre_grasp, not the sample and not stage2.

## 6. Code/evidence changes

New/modified:

- `src/gz_launch/scripts/c43_homing.py`
  - `C43_PRE_GRASP_HOMING=1` two-stage mode
  - pre_grasp constants, 11-waypoint descent, target JSON, action result logs
- `src/gz_launch/models/c43_sample_contact/` (diagnostic sample copy + contact sensor)
- `src/gz_launch/worlds/c43_s4_world_contact.sdf` (diagnostic world)
- `src/gz_launch/launch/c43_s4_full_flow.launch.py`
  - `C43_S4_WORLD_FILE` diagnostic world override
  - `C43_DISABLE_HOMING` / `C43_HOMING_NO_QP` previously added
- `docs/.../gen_pre_grasp.cpp` (FK/IK + descent generator)
- `docs/.../homing_trace_probe.py` (100 Hz trace; sample frame accepts `c43_sample*`)
- `docs/.../pre_grasp_targets.json` (sent targets)
- `docs/.../pre2_trace.csv`, `pre3_world.log`, `pre4_world.log`, contact captures
- `docs/.../EXPERIMENT_2_4_PRE_GRASP.md` (this file)

Not modified: sample size, support geometry, weld attach/detach logic, GRASP close
target, QP algorithm, frozen public interfaces.

## 7. Regression / rollback

- `colcon test` on `dual_arm_transport_executor`: **190 tests, 0 errors, 0 failures**.
- Uncommitted work snapshot:
  - `/home/kz/s622_backup_20261031_2/uncommitted.diff`
  - `/home/kz/s622_backup_20261031_2/untracked.tgz`
- No commit / no push.

## 8. Missing capability and minimal next validation

To close the acceptance table, add an offline whole-robot collision check over both
stages:

- Use the existing `dual_arm_pose_optimization::MoveItCollisionBackend`
  (MoveIt/FCL mesh collision + SRDF ACM) or `MoveItCollisionChecker`, with the
  sample/support/table as collision objects.
- Sample each JTC segment (stage1 and stage2) at ≥100 points; require
  `isSegmentValid` true, `robotWorldDistance > 0`, and record minimum clearance.
- Alternatively, add Gazebo contact sensors to all robot links and the support/table;
  this is heavier but directly observes contacts.

Until then, this experiment's defensible claim is:
**pre_grasp h=0.05 removes the observed left_finger1/sample collision and keeps the
sample centered in 3/3 runs, but full robot–environment/self collision clearance is
still unmeasured.**


## 9. Experiment 2.4-B — full-robot FCL check (2026-10-31)

Checker: `src/dual_arm_pose_optimization/src/c43_path_check_main.cpp`
(`c43_path_check`), using:
- `/tmp/c43_gazebo.urdf` generated from the same C4.3 xacro used by Gazebo;
- `s622_moveit_config/config/s622_dual_arm.srdf` (no ACM expansion);
- sample box `0.12 x 0.05 x 0.05` at `(0,0,0.275)`;
- support cylinder `r=0.04, h=0.25` at `(0,0,0.125)`;
- floor top z=0; left/right base links exempted from the floor only;
- fingers set OPEN (`+0.0305/-0.0305`) in every RobotState.

Path checked: stage1 `dual_home -> pre` 101 states; stage2 11 IK points with
20 sub-samples per segment; actual pre2 100 Hz feedback every 5th row.

Results:

| path | arm-arm collision samples | robot-world collision samples | min arm-arm | min world |
|---|---:|---:|---:|---:|
| h=0.05 | 2 (stage1 89/100, right_finger2 <-> left_finger1) | 0 | 0.000 mm | 5.39 mm |
| h=0.10 (offline) | 0 | 0 | 0.170 mm | 5.39 mm |

The h=0.05 path is the one validated 3/3 in Gazebo for **sample** contact; it still
has an arm-arm contact at the end of stage1, so it cannot be accepted as a full
collision-free homing.

**Conclusion:** keep the sample-contact result, do not claim full 2.4. The next
change should be a small path/safety fix at stage1 near pre_grasp (or a q_safe /
lateral separation), then rerun the same FCL checker + 3 Gazebo contact runs.


## 10. Experiment 2.4-C — right-first stage1 (2026-10-31)

Only the stage1 ordering changed. Targets, h=0.05, the 11-point descent, fingers,
sample/support/GRASP/weld/QP were unchanged.

### Offline comparison (same C4.3 URDF/SRDF, open fingers)

| stage1 mode | cross-arm collisions | worst cross-arm depth | min cross-arm | min world |
|---|---:|---:|---:|---:|
| simultaneous | 21 samples (left_finger1 <-> right_finger2) | 12.14 mm | 0.00 mm | 5.39 mm |
| left_first | 4 pairs, 35–54 samples each | 42.37 mm | 0.00 mm | 5.39 mm |
| **right_first** | **0** | — | **19.36 mm** | **5.39 mm** |

`right_first` = right arm `dual_home -> pre_grasp` first (left held at home), then
left arm `dual_home -> pre_grasp` (right held at pre), then the unchanged descent.

The 891 whole-robot self contacts were audited by unique pair. In `right_first`
they are only:
- same-arm adjacent/fixed pairs: `*_shoulder <-> *_upperarm`,
  `*_wrist1 <-> *_wrist2`, `*_wrist2 <-> *_wrist3`,
  `*_forearm <-> *_wrist1`, `*_finger* <-> *_wrist3`, `*_base <-> *_shoulder`;
- `c42_floor <-> left/right_base_link` (depth ~1e-15; floor exemption);
- **no `left_* <-> right_*` pair**.

No SRDF ACM was expanded. These are normal adjacent/fixed structure contacts, not
illegal collisions.

### Gazebo right-first validation (3/3)

| run | sample initial | sample final | robot contacts | JTC |
|---|---|---|---|---|
| rf1 | `(0,0,0.274999773)` | `(0,0,0.274999773)` | 0 | right→left→descent success |
| rf2 | same | same | 0 | success |
| rf3 | same | same | 0 | success |

`rf1` 100 Hz actual-feedback FCL check:
- cross-arm collisions **0**;
- world collisions **0**;
- min cross-arm **19.376 mm** (`right_wrist3 <-> left_wrist3`, actual row 3800);
- min world **19.782 mm** (actual row 4000).

### 2.4-C conclusion

The 50 mm vertical pre_grasp and original descent are kept. Changing only stage1
to **right-first** removes the `left_finger1 <-> right_finger2` collision.
All illegal collision counts are 0; the smallest clearance is the target-path
world clearance 5.39 mm, above the 3 mm candidate. Sample contact is 0 in 3/3
Gazebo runs and the sample remains at `(0,0,0.274999773)`.

Evidence:
- `collision_2c_simultaneous.txt`, `collision_2c_left_first.txt`,
  `collision_2c_right_first.txt`
- `collision_2c_rf1_actual.txt`
- `rf1_trace.csv`, `/tmp/rf{1,2,3}_contact.txt`, `/tmp/rf{1,2,3}_world.log`
- checker source: `src/dual_arm_pose_optimization/src/c43_2c_sequence_check.cpp`
