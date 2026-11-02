# Experiment 2.5-C — contact-limited close diagnostic (2026-11-01)

Status: **C0 cancel/stop PASS; C1/C2 first-contact stop works, but contact is
SINGLE-LEFT only; dual contact not established; no attach.**

## Setup

- Independent diagnostic only; no executor state-machine change, no weld/attach.
- Direct `/left_hand_controller/follow_joint_trajectory`.
- Right-first homing to q_grasp12; QP inactive; fingers open.
- Contact sensor: `c43_sample_contact` → `left_finger1/2`, `right_finger1/2`.
- Probe: `src/gz_launch/scripts/c43_contact_stop_probe.py`.
- Cancel is sent on the FIRST finger contact; then the probe measures the
  physical stop and sample displacement.

## C0 — cancel/stop without sample

No sample; close target 0.0249; cancel at 1.5 s after close start.

```text
cancel sent:      3.551 s
cancel confirmed: 3.554 s
JTC result:       status 5 (CANCELED), error_code 0
physical stop:    3.780 s (0.23 s after cancel)
final fingers:    0.029506 / -0.029504
extra close:      ~1.0 mm
```

**PASS**: cancel reaches the JTC, the physical finger motion stops, and a
CANCELED result is not treated as a failure.

## C1 — sample, target ±0.0250

```text
first contact sim: 125.390 s
collision2:        s622_dual_arm::left_finger1::left_finger1_collision
sides:             left only
fingers at contact: 0.025089/-0.025089
cancel sent/confirmed: 10.010/10.011 s (wall-relative)
JTC result: status 5 (CANCELED)
physical stop: 10.230 s (0.22 s after cancel)
final fingers: 0.0250890/-0.0250890
extra close: ~0.0065 mm total
sample displacement: ~0.018 mm 3D
```

## C2 — sample, target ±0.0249

```text
first contact sim: 57.200 s
collision2:        s622_dual_arm::left_finger1::left_finger1_collision
sides:             left only
fingers at contact: 0.025082/-0.025082
cancel sent/confirmed: 10.346/10.347 s
JTC result: status 5 (CANCELED)
physical stop: 10.570 s (0.22 s after cancel)
final fingers: 0.025082/-0.025082
extra close: ~0.0004 mm total
sample displacement: ~0.055 mm 3D
```

The first contact occurs at essentially the same actual finger position in C1
and C2 (~0.02508–0.02509); changing the command from 0.0250 to 0.0249 does not
reach a second (right) contact before the first contact cancel.

## Findings

1. Contact sensor correctly identifies the left finger collision; contact time
   and cancel/stop timing were recorded.
2. First-contact cancel works: 0.22 s cancel-to-stop; extra closing ≤0.0065 mm.
3. The sample displacement is small (<0.055 mm) and below the 0.5 mm threshold.
4. Contact is **single-left**, not dual. Right finger never contacted. Therefore
   this is a controlled single-contact stop, not a valid two-sided grasp.
5. No weld/attach was performed.

## Status

| item | status |
|---|---|
| C0 cancel/stop | PASS |
| C1 ±0.0250 first-contact stop | SINGLE_LEFT_CONTACT |
| C2 ±0.0249 first-contact stop | SINGLE_LEFT_CONTACT |
| dual contact | NOT_ESTABLISHED |
| real friction grasp | NOT_EVALUATED |
| executor integration | NOT_CHANGED (not yet) |

Next: do not keep closing to force the right side. The next investigation is the
left/right finger geometry or independent finger control; executor integration
only after dual-contact/repeatability is understood.

Evidence: `contact_stop_c0.json`, `contact_stop_c1.json`, `contact_stop_c2.json`,
`contact_stop_c{0,1,2}.csv`.
No commit / no push.
