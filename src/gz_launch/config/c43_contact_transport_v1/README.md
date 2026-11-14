# C43 contact_force5_jtc_reference_v1

Versioned Gazebo physical-contact input, packaged on 2026-11-12 from the
audited 2026-11-11 reference. This is a simulation diagnostic, not formal W7
or hardware acceptance. Geometry and the effective contact configuration are
retained; only resource paths in the frozen URDF were made relocatable.

- `robot.urdf`: full dual robot, initial open/home state, original STL;
  no force plugins, FT declarations or weld. The runner instruments a copy.
- `world.sdf`: source sample/support/table scene; the runner adds the
  destination support at Y=0.12 m.
- `controllers.yaml`: reference parameter snapshot. Arm JTCs own the arms in
  the reference; hand JTCs are not activated with native finger forces.
- `approach.json`, `transport.json`: previously checked candidate/path,
  X=±25 mm, outward tilt 15°, lift 60 mm, translate Y=120 mm, place/retreat.
  The path flags refer to discrete FCL evidence, not continuous certification.
- `task.json`: explicit diagnostic settings, simulation-time physical waits,
  and wall-clock stall watchdog. These are not approved formal safety limits.
- `qp_task.json`: bounded 0.2 mm destination seating overtravel; continuous
  support contact requests the existing HOLD service and verifies its ACK.
- `provenance.json`: original input paths and hashes plus packaged-file hashes.
  Historical paths here are provenance text; they are not read at runtime.

Build `gz_launch` and source the installation. From any working directory:

```bash
ros2 run gz_launch c43_run_contact_transport.py --prepare-only --out /tmp/c43_prepare_NEW
ros2 run gz_launch c43_run_contact_transport.py --out /tmp/c43_jtc_NEW
ros2 run gz_launch c43_run_contact_transport.py --backend qp-smoke --out /tmp/c43_qp_smoke_NEW
ros2 run gz_launch c43_run_contact_transport.py --backend qp-carry --out /tmp/c43_qp_carry_NEW
```

Output directories must not already exist. `--prepare-only` resolves installed
meshes/configuration, generates the actual URDF/SDF/world and metadata without
starting Gazebo. `--gui --keep-open --start-delay 20` enables the desktop view
when display authentication is configured. The host GUI wrapper configures X11.

`jtc` is the reference. `qp-smoke` and `qp-carry` use the existing QP controller,
versioned grasp transforms, strict resource switching, and recorded status.
They do not run the formal transport executor or its full ACK/fault matrix.
The source reference controller parameters and S2 frozen baseline are not edited.
The full QP trajectory uses diagnostic `hold`/`seat` phase names. These are not
accepted by the formal executor PhaseDriver and require an explicit adapter
before formal W7 integration. Current 5 ms physics / 8 ms QP settings yield
about 0.8 controller trajectory seconds per simulation second; measure both
clocks rather than treating configured duration as physical elapsed time.

For the explicit scheduling-alignment experiment, add `--physics-step-ms 2`
to the installed entry. It changes only the generated world, retains the
packaged 5 ms reference, and records the actual step in metadata. Frequency
inference is per simulation second; wall-time RTF and hardware real-time
acceptance are separate. See the 2026-11-12 scheduling validation report.

Dependencies are installed ROS packages and their model resources, not an old
`log/` directory or a particular `/workspace` prefix. Force publication is a
compiled installed executable. Every actual run saves resolved inputs, script
snapshots, checksums, native contacts/poses, telemetry, and timing conditions.

The opt-in `qp-executor` backend uses the existing `c43_s4_executor` executable
and shared C++ task core with `plant_profile=physical_contact`; Python only
executes measured steps. Build `dual_arm_transport_executor` as well. This
backend defaults to an explicit 2 ms world copy, enables executor heartbeat,
and uses legal `settle` / `settle_post` phases. Destination support and an exact
executor-origin HOLD ACK are required before opening. Its new contact profile
has separate validation; it does not constitute the full original W7/W9 matrix.

```bash
ros2 run gz_launch c43_run_contact_transport.py --backend qp-executor --out /tmp/c43_executor_NEW
```

`--inject-executor missing-grasp` and `--inject-executor missing-support` are
explicit negative-test observation masks; raw native contact data remain intact.
A latched physical run does not support online reset/recovery yet. Inspect its
artifacts and start a new isolated experiment. See the state-machine integration
report dated 2026-11-12 for measured results and limitations.
`--inject-executor retreat-abort` explicitly aborts one simulation second into
RETREAT; its STOP confirmation waits for both arm JTC goal results to terminate.
