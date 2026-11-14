#!/usr/bin/env python3
"""Recompute grip A/B motion and contact metrics from saved raw records."""
import csv
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
import sys

root=Path(sys.argv[1])
results=[]
for path in sorted(root.glob('*/summary.json')):
    d=json.loads(path.read_text())
    if d.get('error') or 'sample_lift_mm' not in d:
        continue
    rows=list(csv.DictReader((path.parent/'telemetry.csv').open()))
    pre=[r for r in rows if r['phase']=='settle' and r['sample_z'] and r['wrist_z']]
    moving=[r for r in rows if r['phase'] in ('lift','hold_lift') and r['sample_z'] and r['wrist_z']]
    if not pre or not moving: continue
    z0=float(pre[-1]['sample_z']); w0=float(pre[-1]['wrist_z'])
    peak=max(moving,key=lambda r:float(r['sample_z']))
    rec={k:d[k] for k in ('width_m','mass_kg','mode','target_m','wrist_lift_mm','sample_lift_mm','follow_ratio')}
    rec.update(run=path.parent.name,peak_sample_lift_mm=1000*(float(peak['sample_z'])-z0),
               single_robot=d.get('single_robot',False),
               wrist_lift_at_peak_mm=1000*(float(peak['wrist_z'])-w0))
    phases={}
    right_touch=0
    for line in (path.parent/'contacts.jsonl').open():
        c=json.loads(line); names=' '.join(c['fingers'])
        right_touch+=int('right_finger' in names)
        p=phases.setdefault(c['phase'],dict(messages=0,both=0,finger1=0,finger2=0))
        p['messages']+=1
        p['finger1']+=int('left_finger1' in names)
        p['finger2']+=int('left_finger2' in names)
        p['both']+=int('left_finger1' in names and 'left_finger2' in names)
    rec['right_finger_contact_messages']=right_touch
    rec['contacts_by_phase']=phases
    robot=ET.parse(path.parent/'robot.urdf').getroot()
    rec['hardware_components']=[c.get('name') for c in robot.findall('ros2_control')]
    sdf=(path.parent/'robot.sdf').read_text()
    assert 'DetachableJoint' not in sdf and 'ApplyJointForce' not in sdf
    world=ET.parse(path.parent/'world.sdf').getroot()
    actual_mass=float(world.find(".//model[@name='c43_sample']/link/inertial/mass").text)
    assert abs(actual_mass-rec['mass_kg'])<1e-9
    collision_names=set()
    with (path.parent/'contacts_raw.txt').open() as raw:
        for line in raw:
            if 'name:' in line: collision_names.update(re.findall(r'name:\s*"([^"]+)"',line))
    rec['sample_contact_collision_names']=sorted(collision_names)
    rec['other_robot_contact_links']=sorted(n for n in collision_names if 's622_dual_arm::' in n and 'left_finger' not in n)
    rec['unconfounded_left_only']=not bool(rec['other_robot_contact_links'])
    results.append(rec)
(root/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
columns=['run','single_robot','width_m','mass_kg','mode','target_m','wrist_lift_mm','peak_sample_lift_mm','sample_lift_mm','follow_ratio','right_finger_contact_messages','unconfounded_left_only']
with (root/'comparison.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');w.writeheader();w.writerows(results)
for r in results:
    print(f"{r['run']}: wrist={r['wrist_lift_mm']:.3f} mm, object peak={r['peak_sample_lift_mm']:.3f} mm, final={r['sample_lift_mm']:.3f} mm; clean_left_only={r['unconfounded_left_only']}; other robot contacts={r['other_robot_contact_links']}")
