# Experiment 2.5-D — finger/sample geometry audit (2026-11-02)

Status: **read-only measurement; no TCP/GRASP/homing/weld changes.**
The first contact is on the **sample top edge/corner**, not the intended side
faces. `left_finger2` is only ~3.6 µm from the sample at the C1 contact state;
the C2 FCL state reports both left fingers in collision, while the first-contact
log only recorded `left_finger1`. Dual contact is **not** established.

## Method

- C4.3 Gazebo xacro URDF (`/tmp/c43_gazebo.urdf`), original SRDF.
- q_grasp12 arms; finger joints set to the C1/C2 first-contact values:
  - C1: `+0.025089375 / -0.025089374`
  - C2: `+0.0250825 / -0.0250825`
- Sample collision box: `0.12 x 0.05 x 0.05` at `(0,0,0.275)`.
- FCL BVH mesh vs box distance with nearest points (`c43_geometry_probe`).
- FCL returns `-1` for an intersecting pair; positive values are clearances.

## 1. Finger distances at the first-contact states

| state | link | distance | nearest sample-local point | face |
|---|---|---:|---|---|
| C1 | left_finger1 | **-1 (intersecting)** | — | collision |
| C1 | **left_finger2** | **0.003593 mm (3.6 µm)** | `(0.059911, 0.025000, 0.025000)` | x-end + y-side + z-top |
| C1 | right_finger1 | 5.409 mm | `(-0.059958, -0.025000, 0.014504)` | x-end |
| C1 | right_finger2 | 5.414 mm | `(-0.039426, 0.025000, 0.025000)` | y-side/top |
| C2 | left_finger1 | -1 (intersecting) | — | collision |
| C2 | left_finger2 | -1 (intersecting) | — | collision |
| C2 | right_finger1 | 5.409 mm | — | no contact |
| C2 | right_finger2 | 5.414 mm | — | no contact |

So the answer to "how far is left_finger2 at first contact" is:
**C1: ~0.0036 mm; C2: FCL says intersecting (0 / -1 sentinel).**
The difference between C1 and C2 is small; both are at the same top-corner
geometry.

## 2. Closure centre / sample centre offset

At q_grasp12 + C1 fingers:

```text
left_grasp_frame = (0.049692, 0.000003, 0.300004) m
sample centre    = (0.000000, 0.000000, 0.275000) m
TCP delta        = (+49.692, +0.003, +25.004) mm

left finger-origin midpoint = (0.049692, 0.000003, 0.351004)
midpoint delta               = (+49.692, +0.003, +76.004) mm

grasp-frame vs sample identity rotation = pi rad (180 deg)
```

- Lateral Y offset is only ~0.003 mm; the earlier 0.0895 mm sample-Y clue is not
  the cause here.
- The dominant offsets are **X ~49.7 mm** (the left gripper is at the sample end)
  and **Z 25 mm at the grasp frame / 76 mm at the finger origins**.
- The gripper is oriented 180 deg relative to the sample identity frame; the pads
  approach the top region, not the mid-height side faces.

## 3. What the first contact actually is

C1's nearest point on the sample is:

```text
sample-local = (0.059911, 0.025000, 0.025000) m
             = +X end, +Y side, +Z top
```

That is a **top edge/corner** of the 0.12 x 0.05 x 0.05 box, not the flat
`y = ±0.025` side face at mid-height. The left finger is touching the sample's
top perimeter while closing. This explains why forcing the same target further
does not create a clean two-sided side grasp.

## 4. Sensor coverage caveat

The sample contact sensor is attached to the sample collision and can report
both finger collisions; the C1 raw first contact was
`s622_dual_arm::left_finger1::left_finger1_collision` only.
C2 FCL says `left_finger2` is also intersecting at the captured contact state,
but the first-contact parser did not record a `left_finger2` event before
cancel. Therefore:

- C1: left_finger2 is not in contact (3.6 µm gap); single-left is real.
- C2: **dual contact is unresolved**; FCL says both intersect, but the sensor
  first-event log only names left_finger1. Do not claim dual until an
  all-pairs contact capture is done.

## 5. Time-base caveat

The recorded values use two clocks:

```text
C1: contact sim = 125.390 s; contact wall-relative = 10.008 s
C2: contact sim =  57.200 s; contact wall-relative = 10.342 s
```

Cancel-to-stop (0.22 s) is valid on the wall clock, but contact-to-cancel and
contact-to-stop end-to-end delays are **not** yet on one unified clock. The next
run must record both `/world/.../clock` and a monotonic wall clock for every
event, plus finger speed at first contact.

## 6. Conclusion for D3/D4

The problem is now localized:

- Not a JTC tracking failure.
- Not a large lateral Y centering error (Y ~0.003 mm).
- The gripper is closing at the **top edge/corner**, 25 mm above the sample
  centre at the grasp frame; the intended side faces are not engaged.
- `left_finger2` is nearly touching in C1 (3.6 µm), so the one-sided event can be
  a tiny asymmetry at a corner, not a 2 mm side gap.

D3 should therefore be a **vertical/top-edge alignment** question, not a random
XY nudge. Because q_grasp12 is frozen, the correction must be expressed as a
documented TCP/grasp-frame offset or a new diagnostic pre-grasp pose, then
revalidated with FCL and contact capture. Do not change the frozen baseline
blindly.

D4 must first capture **all contact pairs** in the same sensor message before
cancel; otherwise dual contact cannot be distinguished from a missed second
event.

Evidence: `c43_geometry_probe.cpp`, output captured in this report; C1/C2
`contact_stop_*.json` from 2.5-C.
No commit / no push.
