#!/usr/bin/env python3
"""Independent same-native-timestamp geometry/contact audit; no ROS joint FK."""
import csv
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as E
import numpy as np
from c43_escape_audit import rotation,collision_vertices,vectors,rpy
from c43_native_pose_record import blocks,WANTED


def messages(path):
    buf=[]
    for line in path.open():
        if line.strip():buf.append(line)
        elif buf:yield ''.join(buf);buf=[]
    if buf:yield ''.join(buf)


def audit(path):
    summary=json.loads((path/'instrumented_summary.json').read_text())
    lifts=[e for e in summary['events'] if e['event']=='goal_accepted' and e['phase']=='lift']
    start=lifts[0]['sim_last'];cutoff=max(float(r['sim']) for r in csv.DictReader((path/'telemetry.csv').open()) if r['sim'])
    raw=[json.loads(l) for l in (path/'native_poses.jsonl').open()]
    raw=[r for r in raw if r['sim']<=cutoff and WANTED.issubset(r['poses']) and max(r['ages_s'].values())<1e-8]
    native={round(r['sim']*1e9):r for r in raw};contacts={}
    for text in messages(path/'contacts_raw.txt'):
        stamp=next(blocks(text,'stamp'),'');d=dict(re.findall(r'\b(sec|nsec):\s*(\d+)',stamp))
        key=int(d.get('sec',0))*1000000000+int(d.get('nsec',0))
        if key not in native:continue
        rec={}
        for b in blocks(text,'contact'):
            names=re.findall(r'name:\s*"([^"]+)"',b)
            if not any('c43_sample' in n for n in names):continue
            for kind in ('left_finger1','left_finger2','c43_support'):
                if any(kind in n for n in names):
                    item=rec.setdefault(kind,dict(points=[],normals=[],depth_count=0))
                    item['points'].extend(vectors(b,'position').tolist());item['normals'].extend(vectors(b,'normal').tolist())
                    item['depth_count']+=len(re.findall(r'\bdepth:',b))
        contacts[key]=rec
    urdf=E.parse(path/'robot.urdf').getroot();meshes={};joints={}
    for i in (1,2):
        meshes[i]=collision_vertices(urdf,i)
        joint=urdf.find(f"joint[@name='left_finger{i}_joint']");o=joint.find('origin')
        joints[i]=(np.fromstring(o.get('xyz'),sep=' '),rpy(np.fromstring(o.get('rpy'),sep=' ')),np.fromstring(joint.find('axis').get('xyz'),sep=' '))
    data=[]
    for key,n in native.items():
        if key not in contacts:continue
        ps=n['poses'];s=np.array(ps['c43_sample'][:3]);rs=rotation(ps['c43_sample'][3:]);wp=np.array(ps['left_wrist3_link'][:3]);rw=rotation(ps['left_wrist3_link'][3:]);c=contacts[key]
        rec=dict(sim=n['sim'],support_contact=('c43_support' in c),
            pitch_deg=math.degrees(math.asin(float(np.clip(-rs[2,0],-1,1)))),
            sample_x_mm=s[0]*1000,sample_z_mm=s[2]*1000,wrist_z_mm=wp[2]*1000,
            sample_in_wrist_z_mm=float((rw.T@(s-wp))[2]*1000),contact_detail={})
        total=0
        for i in (1,2):
            fp=np.array(ps[f'left_finger{i}'][:3]);rf=rotation(ps[f'left_finger{i}'][3:])
            world=(rf@meshes[i].T).T+fp;local=(rs.T@(world-s).T).T
            margin=(np.array([.06,.025,.025])-np.abs(local)).min(axis=1)
            rec[f'finger{i}_vertex_inside_box_mm']=max(0,float(margin.max()))*1000
            p,r,a=joints[i];delta=r.T@(rw.T@(fp-wp)-p);q=float(np.dot(delta,a)/np.dot(a,a));total+=abs(q)
            rec[f'finger{i}_native_q_m']=q
            rec[f'finger{i}_joint_axis_residual_mm']=float(np.linalg.norm(delta-a*q)*1000)
            points=np.array(c.get(f'left_finger{i}',{}).get('points',[])).reshape(-1,3)
            if len(points):
                loc=(rs.T@(points-s).T).T*1000
                rec['contact_detail'][f'finger{i}']=dict(min_local_mm=loc.min(0).tolist(),max_local_mm=loc.max(0).tolist(),count=len(points))
        rec['native_total_joint_opening_mm']=total*1000
        if 'c43_support' in c:
            p=np.array(c['c43_support']['points']);rec['support_x_range_mm']=[float(p[:,0].min()*1000),float(p[:,0].max()*1000)]
        data.append(rec)
    data.sort(key=lambda r:r['sim']);pre=[r for r in data if r['sim']<=start];lift=[r for r in data if r['sim']>start]
    ref=pre[-1];events=[]
    for label,check in [('pitch_1deg',lambda r:abs(r['pitch_deg']-ref['pitch_deg'])>1),
                        ('relative_z_slip_1mm',lambda r:abs(r['sample_in_wrist_z_mm']-ref['sample_in_wrist_z_mm'])>1),
                        ('no_support_contact',lambda r:not r['support_contact']),
                        ('native_opening_below_48mm',lambda r:r['native_total_joint_opening_mm']<48),
                        ('native_finger1_inside_1mm',lambda r:r['finger1_vertex_inside_box_mm']>1),
                        ('native_finger2_inside_1mm',lambda r:r['finger2_vertex_inside_box_mm']>1)]:
        hit=next((r for r in lift if check(r)),None)
        if hit:events.append(dict(event=label,**hit))
    result=dict(run=path.name,cutoff_sim=cutoff,start_lift_sim=start,matched_native_frames=len(data),
        native_pose_ages_s=0,prelift=ref,last=lift[-1],events=events,
        support_contact_frames_during_lift=sum(r['support_contact'] for r in lift),lift_frames=len(lift),
        max_vertex_inside_mm={str(i):max(r[f'finger{i}_vertex_inside_box_mm'] for r in lift) for i in (1,2)},
        max_joint_axis_residual_mm=max(r[f'finger{i}_joint_axis_residual_mm'] for r in data for i in (1,2)))
    (path/'native_escape_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    with (path/'native_escape_geometry.csv').open('w') as f:
        fields=[k for k in data[0] if k not in ('contact_detail','support_x_range_mm')]
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(data)
    print(json.dumps(dict(run=result['run'],frames=len(data),support=f"{result['support_contact_frames_during_lift']}/{len(lift)}",max_vertex_inside_mm=result['max_vertex_inside_mm'],events=[(e['event'],e['sim']) for e in events]),ensure_ascii=False))


if __name__=='__main__':audit(Path(sys.argv[1]))
