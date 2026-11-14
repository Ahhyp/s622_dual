#!/usr/bin/env python3
"""Summarize calibration integrity and completed/aborted force comparisons."""
import csv
import json
import math
from pathlib import Path
import sys
import numpy as np

root=Path(sys.argv[1]);calibrations={};runs=[];points=[]
for p in root.glob('*/calibration.json'):
    rows=json.loads(p.read_text());by_pose={}
    for r in rows:
        points.append(dict(run=p.parent.name,pose=r['pose'],label=r['label'],
            world_force=r['world_force'],delta_sensor_force=r['delta'][:3],expected_sensor_force=r['expected_child_force'],
            vector_error_N=float(np.linalg.norm(np.array(r['delta'][:3])-r['expected_child_force'])),
            return_force_N=float(np.linalg.norm(r['return_delta'][:3])),joint_motion_m=r['joint_motion_m']))
    for pose in sorted({r['pose'] for r in rows}):
        rs=[r for r in rows if r['pose']==pose]
        by_pose[pose]=dict(points=len(rs),
            max_force_vector_error_N=max(float(np.linalg.norm(np.array(r['delta'][:3])-r['expected_child_force'])) for r in rs),
            max_return_force_N=max(float(np.linalg.norm(r['return_delta'][:3])) for r in rs),
            stroke_motion_points=[dict(label=r['label'],motion_m=r['joint_motion_m']) for r in rs if abs(r['joint_motion_m'])>.001])
    calibrations[p.parent.name]=by_pose
for p in sorted(root.glob('*/instrumented_summary.json')):
    d=json.loads(p.read_text());rows=list(csv.DictReader((p.parent/'telemetry.csv').open()))
    pre=[r for r in rows if r['phase']=='settle' and r['sample_z'] and r['wrist_z']]
    lifted=[r for r in rows if r['phase'] in ('lift','hold_lift') and r['sample_z'] and r['wrist_z']]
    rec=dict(run=p.parent.name,mode=d.get('drive_mode'),target_m=d['target_m'],clamp_command_N=d.get('clamp_command_N'),
        error=d['error'],completed=(d['error'] is None and any(r['phase']=='hold_lift' for r in rows)),
        protocol=('superseded' if p.parent.name in ('force_2N_r1','force_5N_r1','force_2N_sync_r1','force_5N_sync_r1') else 'final'),
        reaction_settle=d['estimated_external_normal_stats'].get('settle'),
        reaction_hold=d['estimated_external_normal_stats'].get('hold_lift'),
        contact_phases=d['contacts_by_phase'])
    conditions=json.loads((p.parent/'conditions.json').read_text())
    rec.update(scene_dx_m=float(conditions.get('C43_LAB_SCENE_DX','0')),
        collision_profile=conditions.get('C43_LAB_COLLISION','mesh'),
        contact_profile=conditions.get('C43_LAB_CONTACT_PROFILE','unchanged'),
        hold_lift_duration_s=d.get('hold_lift_duration_s',5))
    if pre and lifted:
        a=pre[-1];b=lifted[-1];z0=float(a['sample_z']);w0=float(a['wrist_z'])
        rec.update(wrist_lift_mm=(float(b['wrist_z'])-w0)*1000,
            prelift_joint_positions_m={j:float(a[j]) for j in ('left_finger1_joint','left_finger2_joint')},
            sample_final_lift_mm=(float(b['sample_z'])-z0)*1000,
            sample_peak_lift_mm=max((float(r['sample_z'])-z0)*1000 for r in lifted),
            min_total_joint_opening_mm=min((abs(float(r['left_finger1_joint']))+abs(float(r['left_finger2_joint'])))*1000 for r in lifted))
    runs.append(rec)
(root/'calibration_audit.json').write_text(json.dumps(calibrations,indent=2)+'\n')
(root/'calibration_points.json').write_text(json.dumps(points,indent=2)+'\n')
(root/'comparison.json').write_text(json.dumps(runs,indent=2)+'\n')
cols=['run','protocol','mode','scene_dx_m','collision_profile','contact_profile','target_m','clamp_command_N','hold_lift_duration_s','completed','wrist_lift_mm','sample_peak_lift_mm','sample_final_lift_mm','min_total_joint_opening_mm','error']
with (root/'comparison.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');w.writeheader();w.writerows(runs)
print(json.dumps(calibrations,indent=2))
for r in runs:print(json.dumps({k:r.get(k) for k in cols},ensure_ascii=False))
