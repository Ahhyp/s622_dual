#!/usr/bin/env python3
"""Summarize candidate transfer runs without interpreting completion as grip success."""
import csv
import json
from pathlib import Path
import sys


def analyze(root):
    result = {}
    for run in sorted(root.iterdir()):
        if not (run / 'summary.json').exists():
            continue
        summary = json.loads((run / 'summary.json').read_text())
        record = {k: summary.get(k) for k in (
            'error', 'mode', 'single_robot', 'wrist_lift_mm', 'sample_lift_mm',
            'vertical_slip_mm', 'contacts_by_phase')}
        rows = list(csv.DictReader((run / 'telemetry.csv').open()))
        pre = [r for r in rows if r['phase'] == 'settle' and r['sample_y']]
        hold = [r for r in rows if r['phase'] == 'hold_lift' and r['sample_y']]
        if pre and hold:
            y0 = float(pre[-1]['sample_y'])
            record['sample_final_y_displacement_mm'] = (float(hold[-1]['sample_y']) - y0)*1000
            record['sample_max_abs_y_displacement_in_hold_mm'] = max(abs(float(r['sample_y']) - y0) for r in hold)*1000
            record['hold_sim_duration_s'] = float(hold[-1]['sim']) - float(hold[0]['sim'])
            record['right_arm_max_home_error_rad'] = max(abs(float(r[f'right_j{i}']) - (-1.5708 if i == 2 else 0)) for r in hold for i in range(1, 7))
        for name in ('native_escape_audit', 'candidate_achieved', 'actual_path_collision_audit'):
            path = run / (name + '.json')
            if path.exists():
                record[name] = json.loads(path.read_text())
        # Include all robot/sample pairs, not only finger pairs.
        contacts = run / 'contacts.jsonl'
        if contacts.exists():
            pairs = [json.loads(line) for line in contacts.open()]
            record['robot_sample_contact_messages_by_phase'] = {
                phase: sum(bool(c.get('robot_links', c['fingers'])) for c in pairs if c['phase'] == phase)
                for phase in sorted({c['phase'] for c in pairs})}
        result[run.name] = record
    (root / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    for name, record in result.items():
        print(name, json.dumps({k: record.get(k) for k in (
            'error', 'sample_lift_mm', 'vertical_slip_mm',
            'sample_final_y_displacement_mm', 'sample_max_abs_y_displacement_in_hold_mm',
            'right_arm_max_home_error_rad')}, ensure_ascii=False))


if __name__ == '__main__':
    analyze(Path(sys.argv[1]))
