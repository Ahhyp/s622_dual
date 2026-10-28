# C3 / D2-B baseline fixture provenance — FORMAL (approved 2026-10-27)

contract: **`D2B_APPROVED_CONTRACT` v1** (approved by the project approver on 2026-10-27;
see `docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-C3_D2B_审批材料.md` §7 and
`docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_审批记录.md`, row D2-B)

This baseline is a **same-build regression fixture**, NOT a numerical-correctness proof.
The historical cross-generation input (`src/dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv`)
and its `7656/7656` FAIL are retained unchanged and are never overwritten by this fixture.

## Files

| file | sha256 | rows |
|---|---|---|
| `c3_d2b_candidate_baseline.csv` | `5489dfdd348883bf672c3cc26942c560f36db3fbf4a8c3fdbb1155ec0b075f26` | 638 |
| `c3_d2b_candidate_baseline.rerun.csv` | `5489dfdd348883bf672c3cc26942c560f36db3fbf4a8c3fdbb1155ec0b075f26` | 638 |
| `c3_d2b_admm.csv` | `05b5fd14979c562bbf2ecbc18507b5172e9b3b4c77dabe062c1e2506fa580274` | 638 |

Columns: 29 (`k,t,q_prev0..11,dq0..11,e_obj_pos_m,e_obj_rot_rad,committed`).

## Generation (historical record — the run itself was 2026-10-27T17:52Z)

- generator: `/tmp/c3d2b/replay.cpp` (independent driver: `loadTrajectorySpec` →
  `TransportTask` → `DualArmQp`; shares no code with
  `src/dual_arm_qp_core/test/tools/generate_s3_fixture.cpp`)
- generator source sha256 (the exact source that produced this baseline):
  `29fc6e769538791eac2ac61abd9e05fda666e95f4a09af68f406640bb6a5351c`
  **That exact revision was not retained verbatim** (it lived under `/tmp`). The copy kept
  here as `replay.cpp.txt` (sha256 `eb6dbcddd243f11c5f12138130f28903e554f09404c972fe0d10286c08c47105`)
  is the SAME driver **plus three extra output columns** (`des_px/des_py/des_pz`) added later
  for the input-comparison experiment; those columns do not change `dq`, and the run with
  them reproduced byte-identical `dq` values. This asymmetry is disclosed rather than hidden:
  the baseline bytes are frozen by sha256 above, so the fixture stays verifiable even though
  its generator revision is only reproducible modulo the disclosed three-column addition.
- compiler: g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0
- flags: `-std=c++17 -O3 -I install/include -I /usr/include/eigen3`
- link: `-L install/lib -ldual_arm_qp_controller -ldual_arm_object_task -ldual_arm_qp_core
  -ldual_arm_kinematics -lyaml-cpp -losqp`
- measured build: `libdual_arm_qp_core.so` sha256
  `ff95750afe76b49b7738c35eb1264f94791f81004513510e2c8ca939e90a4636` (D1-A integrated)
- trajectory: `src/dual_arm_qp_controller/config/c43_s4_transport.yaml`
- QP params: S3 `ff_1` (kp_rel 10, w_rel 1, w_slack 1000, w_center 1e-4, w_reg 1e-6,
  kp_obj 20, w_obj 1, w_slack_obj 1000, dt 0.008)
- start pose: `dual_arm_qp_controller::defaultHomeQ()` (S2 frozen `q_grasp12`)
- two runs: `cmp` byte-identical (`two_runs_identical: yes`)

## What this fixture is NOT

- It is **not** a proof of numerical correctness. D2-B is a **same-build regression**
  criterion (plus cross-algorithm agreement and physical invariants): it can show that
  the solver still produces what this frozen baseline recorded, and that an independent
  solver agrees to 1e-6 rad -- it cannot show that the baseline itself is physically right.
- It does **not** satisfy, and must never be cited against, the historical
  cross-generation `<=1 ULP` criterion. That criterion stays **FAIL (7656/7656)** and is
  labelled `LEGACY_CROSS_GENERATION_CONTRACT_FAIL_RETAINED` in the gate banner and in
  `C4.3_S4_执行规划.md` §5. D2-B replaced its **acceptance use**, not its result.

## How the gate uses it

`src/dual_arm_qp_core/scripts/c3_d2b_gate.py` (contract `D2B_APPROVED_CONTRACT` v1) checks:

| id | check | threshold |
|---|---|---|
| D2-B.1 | baseline vs rerun row count + byte-identical dq | equality |
| D2-B.2 | same-build `max|Δdq|` | ≤ 1e-12 rad |
| D2-B.3 | every row `committed` | all committed |
| D2-B.5 | OSQP vs ADMM `max|Δdq|` | ≤ 1e-6 rad |
| D2-B.6 | terminal `e_obj_pos` / `e_obj_rot` | ≤ 1e-6 m / 1e-6 rad |
| D2-B.7 | historical `≤1 ULP` FAIL still reported | printed by the gtest, never overridden |

The gate **fails closed**: a missing/unreadable input, an unexpected schema, a provenance
hash that does not match the library being graded, or a threshold breach all yield a
non-zero exit code. There is no "missing input means pass" path.

## What this does NOT mean

- It does not make the historical `≤1 ULP` result pass; that result stays FAIL
  (`LEGACY_CROSS_GENERATION_CONTRACT_FAIL_RETAINED`).
- Passing this gate does not mean the cross-generation interface change is compatible;
  that requires a separate review (see the §533 annotation).
- It does not replace a physical/real-time acceptance.
