#!/usr/bin/env python3
"""Summarize the five-case stopping pilot and hash its retained raw evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from c43_stopping_calibration import CASES


def summarize(root):
    result = dict(scope='fixed-orientation translation, five-case simulation pilot',
                  stopping_reserve_calibration_approved=False, base_clearance_mm=2,
                  experimental_stopping_reserve_mm=8, cases=[])
    for name, mass, speed in CASES:
        run = root / name
        read = lambda f: json.loads((run / f).read_text())
        required = ('finite_runtime_audit.json', 'w9_audit.json', 'w9_fault_capture.json')
        missing = [f for f in required if not (run / f).exists()]
        if missing:
            result['cases'].append(dict(name=name, passed=False, missing=missing, log_path=str(run)))
            continue
        finite, stop, capture = (read(f) for f in (
            'finite_runtime_audit.json', 'w9_audit.json', 'w9_fault_capture.json'))
        first = finite['first_violation_status_sim']
        frames = []
        for line in (run / 'native_poses.jsonl').open():
            frame = json.loads(line)
            if first is not None and first - .22 <= frame['sim'] <= capture['observe_end_sim']:
                if frame['ages_s'].get('c43_sample', 1) <= 1e-9:
                    frames.append((frame['sim'], frame['poses']['c43_sample'][1]))
        before = [(t, y) for t, y in frames if first - .22 <= t <= first - .02]
        after = [(t, y) for t, y in frames if t >= first]
        # Same-clock native samples; status receipt time is not the kernel detection time.
        velocity = None
        if len(before) > 2:
            mean_t = sum(t for t, _ in before) / len(before)
            mean_y = sum(y for _, y in before) / len(before)
            variance = sum((t - mean_t) ** 2 for t, _ in before)
            if variance:
                velocity = 1000 * sum((t - mean_t) * (y - mean_y) for t, y in before) / variance
        hashes = {}
        for filename in ('world.sdf', 'task.json', 'controllers.yaml', 'executor.yaml', 'calibration_variant.json',
                         'qp_status.jsonl', 'native_poses.jsonl', 'contacts.jsonl',
                         'c43_force_lab.py', 'c43_run_contact_transport.py', 'c43_contact_plant.py',
                         'w9_fault_capture.json', 'w9_audit.json', 'finite_runtime_audit.json',
                         'executor/summary.json', 'cleanup.json'):
            digest = hashlib.sha256()
            with (run / filename).open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            hashes[filename] = digest.hexdigest()
        inertial = ET.parse(run / 'world.sdf').getroot().find("world/model[@name='c43_sample']/link/inertial")
        duration = float(re.search(r'transport_duration_s:\s*([\d.]+)', (run / 'executor.yaml').read_text())[1])
        variant_checks = dict(world_mass_matches=math.isclose(float(inertial.findtext('mass')), mass),
                              task_mass_matches=math.isclose(read('task.json')['object']['mass_kg'], mass),
                              transport_duration_matches=math.isclose(duration, 8 / speed))
        gap = finite['post_stop_min_native_box_y_gap_m']
        launch = json.loads((root / (name + '.result.json')).read_text())
        result['cases'].append(dict(name=name, log_path=str(run), mass_kg=mass,
            command=launch['command'], launch_exit=launch['launch_exit'],
            transport_speed_scale=speed, variant=read('calibration_variant.json'),
            passed=finite['passed'] and stop['passed'] and all(variant_checks.values()),
            variant_checks=variant_checks,
            min_post_stop_native_y_gap_mm=1000 * gap if gap is not None else None,
            native_obstacle_contact_messages=finite['native_obstacle_contact_messages'],
            observed_pre_stop_y_velocity_mm_s=velocity,
            observed_post_violation_forward_travel_mm=1000 * (max(y for _, y in after) - after[0][1]) if after else None,
            first_violation_source=finite['first_violation_source'],
            stop_code=stop['first_failure_code'],
            frozen_left_command_span_rad=stop['left_command_span_rad'],
            frozen_right_command_span_rad=stop['right_command_span_rad'],
            stop_observation_sim_s=stop['post_stop_window_sim_s'],
            geometry_check_us_observed_max=finite['geometry_check_us_observed_max'],
            native_age_s_observed_max=finite['native_age_s_observed_max'],
            finite_checks=finite['checks'], stopping_checks=stop['checks'], sha256=hashes))
    result['passed'] = all(r['passed'] for r in result['cases'])
    result['timing_note'] = 'Velocity uses native simulation stamps. Forward travel starts at first observed violation, not exact kernel detection; no stopping latency or certified braking-distance claim.'
    result['limitations'] = ['Five cases do not estimate statistical reliability.',
        'No rotated trajectory, failed gripper retention, hardware, or full RT-cycle acceptance.',
        'The 8 mm reserve remains an experimental candidate within this scene.']
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summarize(args.root), indent=2) + '\n')
