#!/usr/bin/env python3
"""Audit the positive combined-guard run without certifying thresholds or RT collision."""
import argparse,json,math
from pathlib import Path
from c43_dual_transport_analyze import audit


def analyze(run):
    if not (run/'transport_audit.json').exists():audit(run)
    physical=json.loads((run/'transport_audit.json').read_text())
    summary=json.loads((run/'summary.json').read_text());formal=json.loads((run/'executor/summary.json').read_text())
    conditions=json.loads((run/'conditions.json').read_text())
    rows=[json.loads(x) for x in (run/'qp_status.jsonl').open()];running=[x for x in rows if x['state']=='RUNNING']
    checks=dict(opt_in_active=conditions.get('C43_ENABLE_CONTACT_SAFETY')=='1',physical_audit_pass=physical['diagnostic_pass'] is True,
        formal_complete=formal['result']=='COMPLETE',no_false_safety_trigger=not any(e['event']=='physical_safety_violation' for e in summary['events']),
        running_follow_within_experimental_bounds=bool(running) and all(math.isfinite(r['anchor_pos_gap_m']) and math.isfinite(r['anchor_rot_gap_rad']) and r['anchor_pos_gap_m']<=.002 and r['anchor_rot_gap_rad']<=math.radians(.5) for r in running),
        cleanup=json.loads((run/'cleanup.json').read_text())['clean'] is True)
    result=dict(scope='combined opt-in contact/follow guards in physical simulation; thresholds pending calibration',passed=all(checks.values()),checks=checks,
        final_destination_error_mm=physical['final_destination_error_mm'],carry_max_object_angle_deg=physical['carry_max_object_angle_deg'],
        same_stamp_native_frames=physical['same_stamp_native_frames'],peak_anchor_pos_gap_m=max(r['anchor_pos_gap_m'] for r in running),
        peak_anchor_rot_gap_rad=max(r['anchor_rot_gap_rad'] for r in running),full_w9_acceptance=False)
    (run/'contact_safety_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);raise SystemExit(not analyze(p.parse_args().run)['passed'])
