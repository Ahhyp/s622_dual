#!/usr/bin/env python3
"""Audit opt-in RT analytical guard, native truth age and actual post-stop contact."""
import argparse,json,math
import xml.etree.ElementTree as ET
import numpy as np
from c43_contact_safety import rotation
from pathlib import Path


def analyze(run):
    rows=[json.loads(x) for x in (run/'qp_status.jsonl').open()]
    checked=[r for r in rows if r.get('finite_geometry_checked')]
    violated=[r for r in checked if r.get('finite_geometry_violation')]
    contacts=[json.loads(x) for x in (run/'contacts.jsonl').open()]
    native_hits=[r['sim'] for r in contacts if 'c43_w9_obstacle' in json.dumps(r['sample_collision_pairs'])]
    import yaml
    config=yaml.safe_load((run/'controllers.yaml').read_text())['dual_arm_qp_controller']['ros__parameters']
    mode=json.loads((run/'conditions.json').read_text()).get('C43_EXECUTOR_INJECT','')
    formal=json.loads((run/'executor/summary.json').read_text());cleanup=json.loads((run/'cleanup.json').read_text())
    native_checked=[r for r in checked if 'finite_geometry_native_fresh' in r]
    durations=[r['finite_geometry_check_us'] for r in checked]
    checks=dict(command_candidate_guard_exercised=any(r.get('finite_geometry_source',-1)>=4 for r in checked),guard_exercised=bool(checked) and all(r.get('finite_geometry_guard_enabled') for r in checked),
        native_truth_fresh=bool(native_checked) and all(r['finite_geometry_native_fresh'] and -.01<=r['finite_geometry_native_age_s']<=.1 for r in native_checked),
        geometry_timing_recorded=bool(durations) and all(math.isfinite(x) and x>=0 for x in durations),
        cleanup=cleanup['clean'] is True)
    native_gaps=[]
    if mode=='finite-clearance':
        stop=json.loads((run/'w9_audit.json').read_text())
        capture=json.loads((run/'w9_fault_capture.json').read_text())
        obstacle=ET.parse(run/'world.sdf').getroot().find("world/model[@name='c43_w9_obstacle']")
        obstacle_pose=[float(x) for x in obstacle.findtext('pose').split()]
        size=[float(x) for x in obstacle.findtext('link/collision/geometry/box/size').split()]
        if any(abs(x)>1e-12 for x in obstacle_pose[3:]):raise ValueError('unsupported rotated diagnostic obstacle')
        front_y=obstacle_pose[1]-size[1]/2
        begin=violated[0]['sim']-.01 if violated else float('inf')
        for line in (run/'native_poses.jsonl').open():
            frame=json.loads(line)
            if not begin<=frame['sim']<=capture['observe_end_sim'] or frame['ages_s'].get('c43_sample',1)>1e-9:continue
            sample=frame['poses']['c43_sample'];extent=float(np.abs(rotation(sample[3:])[1])@np.array([.06,.025,.025]))
            native_gaps.append(front_y-sample[1]-extent)
        checks.update(native_contact_record_covers_stop_observation=bool(contacts) and contacts[-1]['sim']>=capture['observe_end_sim']-.01,
            stopping_audit=stop['passed'] is True,analytical_violation_identified=bool(violated) and violated[0]['finite_geometry_obstacle']=='c43_w9_obstacle::base_link::collision',
            command_or_native_truth_trigger=bool(violated) and violated[0]['finite_geometry_source'] in (3,4,5,6),no_native_obstacle_contact=not native_hits,
            native_box_y_gap_above_base_clearance=bool(native_gaps) and min(native_gaps)>=.002)
    else:
        resume=json.loads((run/'w9_resume_audit.json').read_text());physical=json.loads((run/'contact_safety_audit.json').read_text())
        checks.update(formal_complete=formal['result']=='COMPLETE',resume_audit=resume['passed'] is True,physical_audit=physical['passed'] is True,no_false_geometry_violation=not violated,
            scene_preflight_pass=json.loads((run/'preflight_gate.json').read_text())['passed'] is True)
    duration_sorted=sorted(durations);p99=duration_sorted[min(len(duration_sorted)-1,int(.99*len(duration_sorted)))] if duration_sorted else None
    result=dict(scope='sampled RT object/environment guard plus native truth; not full controller timing or continuous safety acceptance',passed=all(checks.values()),checks=checks,
        experimental_stopping_reserve_m=config.get('finite_geometry_stop_reserve_m',0),base_geometry_clearance_m=.002,stopping_reserve_calibration_approved=False,controller_status_rows=len(rows),checked_status_rows=len(checked),geometry_check_us_observed_max=max(durations) if durations else None,
        geometry_check_us_observed_p99=p99,geometry_block_observed_over_8ms=any(x>=8000 for x in durations),native_age_s_observed_max=max(r['finite_geometry_native_age_s'] for r in native_checked) if native_checked else None,
        first_violation_status_sim=violated[0]['sim'] if violated else None,first_violation_source=violated[0]['finite_geometry_source'] if violated else None,
        native_obstacle_contact_messages=len(native_hits),post_stop_native_gap_samples=len(native_gaps),post_stop_min_native_box_y_gap_m=min(native_gaps) if native_gaps else None,
        gap_note='exact native box projection to diagnostic obstacle front Y face; sufficient separating-axis gap, not general Euclidean closest distance',first_native_obstacle_contact_sim=min(native_hits) if native_hits else None,
        timing_note='status samples can be dropped and ABORT repeats the last geometry result; rows are not cycle counts, durations are status-observed geometry blocks, not full RT-cycle Gate measurements',full_w9_acceptance=False)
    (run/'finite_runtime_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);raise SystemExit(not analyze(p.parse_args().run)['passed'])
