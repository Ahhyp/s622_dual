# dual_arm_kinematics

Step 0 deliverable of the dual-arm motion-control plan
(`docs/双臂协同/双臂协同运动控制与优化/架构决策单_D1-D7与Step0.md`).

It proves that the project's **own DH kinematics** is trustworthy and builds a
unified dual-arm relative kinematics on top of it — so the later QP core does
not depend on a third-party kinematics library.

## What it provides

- Per-arm FK / geometric Jacobian from `fairino_planning::DHKinematics`
  (base frame, grasp point, `[linear; angular]` twist).
- World-frame composition via the URDF ground-truth base poses.
- Relative pose `left_T_right` and relative Jacobian `J_rel` (6x12) in three
  frames (right-grasp body / left-grasp / world-spatial), assembled with SE(3)
  adjoints — never by naively stacking `[-J_left, J_right]`.
- Pose error twist and a first-order error-rate Jacobian.

## Tests (Step 0 acceptance)

`test_step0_fk_oracle.cpp` (S0.1, S0.2, S0.5)
- DH FK vs **KDL** oracle, per-arm base frame and world frame
- DH analytical Jacobian vs KDL Jacobian and vs finite differences
- tool-chain consistency (`d6 = 0.1` + gripper `0.1168` == URDF `0.2168`)

`test_step0_relative.cpp` (S0.3, S0.4, S0.5)
- `J_rel` vs finite differences of the relative pose (all three frames)
- invariance under a common rigid motion of both bases
- "same body twist on both arms -> relative twist = 0" (adjoint check)
- error-rate Jacobian consistency

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select dual_arm_kinematics --merge-install --symlink-install
colcon test  --packages-select dual_arm_kinematics --event-handlers console_direct+
```

## Oracle fixture

`test/fixtures/s622_dual_arm.urdf` is generated from
`src/gz_launch/config/s622_dual_arm_gazebo.urdf.xacro`. Regenerate with
`scripts/regen_fixture.sh` (needs the workspace built and sourced).
