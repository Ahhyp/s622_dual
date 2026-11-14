#!/usr/bin/env python3
"""Offline escape audit: logged poses/joints + exact STL vertex witnesses.

Positive vertex_inside_box_mm is a geometric solid-overlap witness, NOT a
Gazebo penetration-depth measurement. ROS pose/time alignment is approximate.
"""
import bisect
import csv
import json
import math
from pathlib import Path
import re
import struct
import sys
import xml.etree.ElementTree as E
import numpy as np


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def rpy(v):
    x,y,z=v;cx,sx=math.cos(x),math.sin(x);cy,sy=math.cos(y),math.sin(y);cz,sz=math.cos(z),math.sin(z)
    return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])


def vectors(block,field):
    out=[]
    for m in re.finditer(r'\b'+field+r'\s*\{([^{}]*)\}',block):
        d=dict(re.findall(r'\b([xyz]):\s*(\S+)',m.group(1)))
        out.append([float(d.get(k,0)) for k in 'xyz'])
    return np.array(out).reshape(-1,3)


def load_mesh(path):
    b=path.read_bytes();n=struct.unpack_from('<I',b,80)[0]
    v=np.array([struct.unpack_from('<9f',b,84+50*k+12) for k in range(n)]).reshape(-1,3)
    return np.unique(v,axis=0)


def collision_vertices(model,i,side='left'):
    result=[]
    for collision in model.find(f"link[@name='{side}_finger{i}']").findall('collision'):
        origin=collision.find('origin');p=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
        r=rpy(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')) if origin is not None else np.eye(3)
        mesh=collision.find('geometry/mesh')
        if mesh is not None:
            file=mesh.get('filename');local=Path(file.replace('/workspace/install/share/s622_moveit_descriptions','src/s622_moveit_descriptions'))
            if not local.exists():local=Path(file)
            vertices=load_mesh(local)
        else:
            size=np.fromstring(collision.find('geometry/box').get('size'),sep=' ')
            vertices=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
        result.extend((r@vertices.T).T+p)
    return np.array(result)


def pose(row,prefix):
    return np.array([float(row[prefix+'_'+k]) for k in ('x','y','z')]),rotation([float(row[prefix+'_'+k]) for k in ('qx','qy','qz','qw')])


def audit(path):
    rows=[r for r in csv.DictReader((path/'telemetry.csv').open()) if r['sample_x'] and r['wrist_x'] and r['sim']]
    times=[float(r['sim']) for r in rows]
    model=E.parse(path/'robot.urdf').getroot();data=[];mesh={};joints={}
    for i in (1,2):
        node=model.find(f"joint[@name='left_finger{i}_joint']");o=node.find('origin')
        joints[i]=(np.fromstring(o.get('xyz'),sep=' '),rpy(np.fromstring(o.get('rpy'),sep=' ')),np.fromstring(node.find('axis').get('xyz'),sep=' '))
        mesh[i]=collision_vertices(model,i)
    for row in rows:
        sp,sr=pose(row,'sample');wp,wr=pose(row,'wrist')
        rec=dict(sim=float(row['sim']),phase=row['phase'],sample=sp.tolist(),
            wrist=wp.tolist(),sample_in_wrist_mm=(wr.T@(sp-wp)*1000).tolist(),
            object_rotation_deg=math.degrees(2*math.acos(min(1,abs(float(row['sample_qw']))))),
            total_joint_opening_mm=sum(abs(float(row[f'left_finger{i}_joint'])) for i in (1,2))*1000)
        for i in (1,2):
            p,r,a=joints[i];q=float(row[f'left_finger{i}_joint'])
            # URDF: T_world_child = T_world_parent * T_origin * T_axis(q).
            vertices=(wr@(r@(mesh[i]+a*q).T+p[:,None])).T+wp
            loc=(sr.T@(vertices-sp).T).T
            margins=np.array([.06,.025,.025])-np.abs(loc)
            inside=margins.min(axis=1)
            rec[f'finger{i}_vertex_inside_box_mm']=max(0,float(inside.max()))*1000
            rec[f'finger{i}_z_aabb_overlap_mm']=max(0,min(vertices[:,2].max(),sp[2]+.025)-max(vertices[:,2].min(),sp[2]-.025))*1000
        data.append(rec)
    contact=[]
    for line in (path/'contacts.jsonl').open():
        c=json.loads(line)
        if c['phase'] not in ('settle','lift','hold_lift'):continue
        j=bisect.bisect_left(times,c['sim']);candidates=[max(0,min(len(times)-1,k)) for k in (j-1,j)]
        ix=min(candidates,key=lambda k:abs(times[k]-c['sim']));row=rows[ix];sp,sr=pose(row,'sample')
        for pair in c['pairs']:
            points=vectors(pair['raw_block'],'position');normals=vectors(pair['raw_block'],'normal')
            if not len(points):continue
            local=(sr.T@(points-sp).T).T*1000
            contact.append(dict(sim=c['sim'],phase=c['phase'],finger=pair['finger'],
                telemetry_dt_s=times[ix]-c['sim'],n=len(points),
                local_min_mm=local.min(0).tolist(),local_max_mm=local.max(0).tolist(),
                normals_available=len(normals),depths_available=len(re.findall(r'\bdepth:',pair['raw_block']))))
    pre=[r for r in data if r['phase']=='settle'];lift=[r for r in data if r['phase'] in ('lift','hold_lift')]
    if not pre or not lift:return None
    ref=pre[-1];events=[]
    for label,predicate in [
        ('relative_z_slip_1mm',lambda r:abs(r['sample_in_wrist_mm'][2]-ref['sample_in_wrist_mm'][2])>1),
        ('opening_below_48mm',lambda r:r['total_joint_opening_mm']<48),
        ('finger1_vertex_inside_1mm',lambda r:r['finger1_vertex_inside_box_mm']>1),
        ('finger2_vertex_inside_1mm',lambda r:r['finger2_vertex_inside_box_mm']>1),
        ('sample_rotation_1deg',lambda r:r['object_rotation_deg']>1)]:
        hit=next((r for r in lift if predicate(r)),None)
        if hit:events.append(dict(event=label,**hit))
    report=dict(run=path.name,prelift=ref,last=lift[-1],events=events,
        max_vertex_inside_mm={str(i):max(r[f'finger{i}_vertex_inside_box_mm'] for r in lift) for i in (1,2)},
        contact_snapshots={})
    for label,t in [('prelift',ref['sim']),('last',lift[-1]['sim'])]+[(e['event'],e['sim']) for e in events]:
        report['contact_snapshots'][label]=[]
        for i in (1,2):
            cc=[c for c in contact if f'left_finger{i}' in c['finger']]
            if cc:report['contact_snapshots'][label].append(min(cc,key=lambda c:abs(c['sim']-t)))
    with (path/'escape_geometry.csv').open('w') as f:
        fields=['sim','phase','total_joint_opening_mm','object_rotation_deg','finger1_vertex_inside_box_mm','finger2_vertex_inside_box_mm','finger1_z_aabb_overlap_mm','finger2_z_aabb_overlap_mm']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(data)
    (path/'escape_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    for name in sys.argv[1:]:
        r=audit(Path(name))
        if r:print(json.dumps(dict(run=r['run'],max_vertex_inside_mm=r['max_vertex_inside_mm'],
            events=[(e['event'],round(e['sim'],4)) for e in r['events']]),ensure_ascii=False))
