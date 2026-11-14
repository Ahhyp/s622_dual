#!/usr/bin/env python3
"""Audit actual pick/carry/place: contact pairs and same-stamp native geometry."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E
import numpy as np
from c43_escape_audit import collision_vertices, rotation


def audit(run):
    summary = json.loads((run/'summary.json').read_text())
    contacts = [json.loads(line) for line in (run/'contacts.jsonl').open()]
    names = [f'{side}_finger{i}' for side in ('left','right') for i in (1,2)]
    by_phase = {}
    for phase in sorted({c['phase'] for c in contacts}):
        rows = [c for c in contacts if c['phase'] == phase]
        by_phase[phase] = dict(messages=len(rows), all_four=sum(all(n in ' '.join(c['fingers']) for n in names) for c in rows),
            source_support=sum('c43_support' in json.dumps(c.get('sample_collision_pairs',[])) for c in rows),
            destination_support=sum('c43_destination' in json.dumps(c.get('sample_collision_pairs',[])) for c in rows),
            robot_messages=sum(bool(c['robot_links']) for c in rows),
            nonfinger_robot_messages=sum(any('finger' not in n for n in c['robot_links']) for c in rows))
    model = E.parse(run/'robot.urdf').getroot()
    vertices = {f'{side}_finger{i}':collision_vertices(model,i,side) for side in ('left','right') for i in (1,2)}
    native = [json.loads(line) for line in (run/'native_poses.jsonl').open()]
    wanted = set(names+['c43_sample','left_wrist3_link','right_wrist3_link'])
    native = [r for r in native if wanted.issubset(r['poses']) and all(r['ages_s'].get(n,1)<1e-8 for n in wanted)]
    indexed = {round(c['sim']*1e9):c for c in contacts}
    data = []
    for row in native:
        contact = indexed.get(round(row['sim']*1e9))
        if contact is None:
            continue
        poses = row['poses']; sample = np.array(poses['c43_sample'][:3]); rs = rotation(poses['c43_sample'][3:])
        record = dict(sim=row['sim'], phase=contact['phase'], sample_xyz=sample.tolist(),
            angle_deg=math.degrees(2*math.acos(min(1,abs(poses['c43_sample'][6])))),
            all_four=all(n in ' '.join(contact['fingers']) for n in names),
            source_support='c43_support' in json.dumps(contact.get('sample_collision_pairs',[])),
            destination_support='c43_destination' in json.dumps(contact.get('sample_collision_pairs',[])))
        for n, mesh in vertices.items():
            f=poses[n]; world=(rotation(f[3:])@mesh.T).T+np.array(f[:3]); local=(rs.T@(world-sample).T).T
            record[n+'_inside_mm']=max(0,float((np.array([.06,.025,.025])-np.abs(local)).min(axis=1).max()))*1000
        for side in ('left','right'):
            wrist=poses[side+'_wrist3_link']; record[side+'_sample_relative_mm']=(rotation(wrist[3:]).T@(sample-np.array(wrist[:3]))*1000).tolist()
        data.append(record)
    carrying = [r for r in data if r['phase'] in ('lift','hold_lift','transfer','place')]
    result = dict(error=summary['error'], completed=any(e['event']=='physical_transport_complete' for e in summary['events']),
        contacts_by_phase=by_phase, same_stamp_native_frames=len(data), geometry_method='STL vertex inside box witness, not engine penetration depth',
        events=summary['events'])
    if carrying:
        result['carry_native_frames']=len(carrying)
        result['carry_max_vertex_inside_mm']={n:max(r[n+'_inside_mm'] for r in carrying) for n in names}
        result['carry_max_object_angle_deg']=max(r['angle_deg'] for r in carrying)
        result['sample_peak_z_m']=max(r['sample_xyz'][2] for r in carrying)
    if data:
        result['last_native']=data[-1]
    pre=next((e for e in summary['events'] if e['event']=='prelift'),None)
    final=next((e for e in summary['events'] if e['event']=='final'),None)
    if pre and final:
        result['final_displacement_mm']=((np.array(final['detail']['sample'][:3])-np.array(pre['detail']['sample'][:3]))*1000).tolist()
        result['final_destination_error_mm']=float(np.linalg.norm(np.array(final['detail']['sample'][:3])-np.array([0,.12,.275]))*1000)
    carry_phases=('lift','hold_lift','transfer','place')
    checks=dict(script_completed=result['completed'] and result['error'] is None,
        no_nonfinger_robot_sample_contacts=all(p['nonfinger_robot_messages']==0 for p in by_phase.values()),
        all_four_contacts_through_carry=all(by_phase.get(p,{}).get('messages',0)>0 and by_phase[p]['all_four']==by_phase[p]['messages'] for p in carry_phases),
        unsupported_during_hold_and_transfer=all(by_phase.get(p,{}).get('messages',0)>0 and by_phase[p]['source_support']==0 and by_phase[p]['destination_support']==0 for p in ('hold_lift','transfer')),
        final_supported_without_robot=by_phase.get('final',{}).get('messages',0)>0 and by_phase['final']['destination_support']==by_phase['final']['messages'] and by_phase['final']['robot_messages']==0,
        final_destination_within_10mm=result.get('final_destination_error_mm',float('inf'))<10,
        carry_vertex_overlap_witness_below_0_1mm=bool(carrying) and max(result['carry_max_vertex_inside_mm'].values())<.1,
        carry_object_rotation_below_5deg=bool(carrying) and result['carry_max_object_angle_deg']<5)
    result['diagnostic_checks']=checks
    result['diagnostic_pass']=all(checks.values())
    (run/'transport_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    with (run/'native_transport_geometry.jsonl').open('w') as f:
        for r in data:
            f.write(json.dumps(r)+'\n')
    print(json.dumps({k:result.get(k) for k in ('error','completed','diagnostic_pass','diagnostic_checks','same_stamp_native_frames','carry_max_vertex_inside_mm',
        'carry_max_object_angle_deg','final_displacement_mm','final_destination_error_mm','contacts_by_phase')},indent=2))


if __name__=='__main__':
    audit(Path(sys.argv[1]))
