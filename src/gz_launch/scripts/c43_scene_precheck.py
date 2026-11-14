#!/usr/bin/env python3
"""Non-RT full-scene discrete preflight. Static geometry is read from generated world SDF."""
import argparse,hashlib,json,math,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from c43_contact_safety import rotation


def transform(text):
    values=[float(x) for x in (text or '0 0 0 0 0 0').split()]
    if len(values)!=6 or not all(math.isfinite(x) for x in values):raise ValueError('invalid SDF pose')
    x,y,z,r,p,yaw=values;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(yaw),math.sin(yaw)
    out=np.eye(4);out[:3,3]=[x,y,z];out[:3,:3]=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return out


def pose7(t):
    # Input is SDF rotation; stable matrix -> quaternion via symmetric eigenproblem.
    r=t[:3,:3];q=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],1+np.trace(r)])
    if np.linalg.norm(q)<1e-8:
        i=int(np.argmax(np.diag(r)));xyz=np.zeros(3);xyz[i]=math.sqrt(max(0,(1+2*r[i,i]-np.trace(r))/4))
        j,k=(i+1)%3,(i+2)%3;xyz[j]=(r[j,i]+r[i,j])/(4*xyz[i]);xyz[k]=(r[k,i]+r[i,k])/(4*xyz[i]);q=np.r_[xyz,(r[k,j]-r[j,k])/(4*xyz[i])]
    q/=np.linalg.norm(q);return [*t[:3,3],*q]


def export_scene(run,out):
    share=Path(get_package_share_directory('gz_launch'));files={};objects=[];sample=None
    def read(path):
        files[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();return ET.parse(path).getroot()
    root=read(run/'world.sdf');world=root.find('world')
    def check_pose(node):
        p=node.find('pose')
        if p is not None and (p.get('relative_to') or p.get('rotation_format') or p.get('degrees')):raise ValueError('unsupported relative/encoded pose: '+ET.tostring(p).decode())
        return transform(node.findtext('pose'))
    def model(m,t,name):
        nonlocal sample
        if m.findall('include') or m.findall('model'):raise ValueError('nested models must be resolved explicitly')
        static=m.findtext('static','false')=='true'
        if name!='c43_sample' and not static:raise ValueError('unmodelled moving obstacle '+name)
        for link in m.findall('link'):
            lt=t@check_pose(link)
            for c in link.findall('collision'):
                ct=lt@check_pose(c);g=c.find('geometry')
                if len(g)!=1:raise ValueError('ambiguous collision geometry')
                shape=g[0];kind=shape.tag
                if kind=='box':dims=[float(x) for x in shape.findtext('size').split()]
                elif kind=='cylinder':dims=[float(shape.findtext('radius')),float(shape.findtext('length')),0]
                elif kind=='plane':dims=[float(x) for x in shape.findtext('normal').split()]
                else:raise ValueError('unsupported world collision geometry '+kind)
                obj=dict(id=f"{name}::{link.get('name')}::{c.get('name')}",model=name,kind=kind,dimensions=dims,pose=pose7(ct))
                if name=='c43_sample':
                    if sample is not None or kind!='box':raise ValueError('sample must have one box collision')
                    if not np.allclose(np.linalg.inv(t)@ct,np.eye(4),atol=1e-10,rtol=0):raise ValueError('sample collision offsets require explicit base-to-box transform support')
                    sample=obj
                else:objects.append(obj)
    for m in world.findall('model'):model(m,check_pose(m),m.get('name'))
    for inc in world.findall('include'):
        uri=inc.findtext('uri');name=inc.findtext('name')
        if not uri.startswith('model://'):raise ValueError('unsupported model URI '+uri)
        model_name=uri[8:];path=share/'models'/model_name/'model.sdf'
        if not path.is_file():raise ValueError('missing local model '+str(path))
        m=read(path).find('model');model(m,check_pose(inc) if inc.find('pose') is not None else check_pose(m),name or m.get('name'))
    if sample is None:raise ValueError('sample missing')
    scene=dict(schema='c43_static_scene/v1',sample=sample,objects=objects,source_sha256=files,
        mount_exemptions=['left_base_link/right_base_link vs ground_plane and c43_table::link::surface only'],
        world_planes='actual SDF physics planes; plane size is not converted to finite thickness',
        scope='sampled full robot and finite world; not continuous collision proof')
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    with (out/'scene.txt').open('w') as f:
        f.write('sample '+' '.join(map(str,sample['dimensions']))+'\n')
        for obj in objects:f.write(obj['id']+' '+obj['kind']+' '+' '.join(map(str,obj['dimensions']+obj['pose']))+'\n')
    return scene


def planned_states(run,count):
    path=json.loads((run/'transport.json').read_text());approach=json.loads((run/'approach.json').read_text());grasp=path['grasp_q12']
    if [x['name'] for x in path['stages']]!=['lift','transfer','place','retract','return_over_source','return_pre','home_left','home_right']:raise ValueError('reference phases missing or reordered')
    home=[0,-1.5708,0,0,0,0]*2
    previous=np.array(home+[.0305,-.0305]*2+[0,0,.275,0,0,0,1],dtype=float);rows=[]
    def segment(phase,q,object_pose,closed=False,steps=count):
        nonlocal previous
        if len(q)!=12 or len(object_pose)!=7:raise ValueError('invalid reference state dimensions')
        end=np.array(list(q)+([.0251,-.0251]*2 if closed else [.0305,-.0305]*2)+list(object_pose),dtype=float)
        if not np.isfinite(end).all():raise ValueError('nonfinite reference input')
        # Quaternion interpolation follows shortest arc; reference path has fixed orientation.
        for k in range(1,steps+1):
            t=k/steps;state=(1-t)*previous+t*end;qa=previous[19:23];qb=end[19:23]
            if qa@qb<0:qb=-qb
            state[19:23]=(1-t)*qa+t*qb;state[19:23]/=np.linalg.norm(state[19:23]);rows.append((phase,state.copy()))
        previous=end
    initial=[0,0,.275,0,0,0,1]
    segment('APPROACH',home,initial)
    segment('APPROACH',home[:6]+approach['right_pre'],initial)
    segment('APPROACH',approach['left_pre']+approach['right_pre'],initial)
    for q in approach['left_descent'][1:]:segment('APPROACH',q+approach['right_pre'],initial)
    for q in approach['right_descent'][1:]:segment('APPROACH',grasp[:6]+q,initial)
    segment('GRASP',grasp,initial,True)
    for stage in path['stages']:
        if not stage['q12'] or len(stage['q12'])!=len(stage['object_xyz']):raise ValueError('reference sample counts differ')
        phase={'lift':'LIFT','transfer':'TRANSPORT','place':'LOWER'}.get(stage['name'],'RETREAT')
        for q,obj in zip(stage['q12'],stage['object_xyz']):segment(phase,q,list(obj)+[0,0,0,1],stage['closed'])
        if stage['name']=='place':segment('RELEASE',stage['q12'][-1],[0,.12,.275,0,0,0,1],False)
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--out',type=Path);p.add_argument('--samples-per-segment',type=int,default=16);p.add_argument('--export-only',action='store_true');a=p.parse_args()
    if not 2<=a.samples_per_segment<=256:p.error('samples must be 2..256')
    out=a.out or a.run/'scene_precheck';out.mkdir(parents=True,exist_ok=False);scene=export_scene(a.run,out)
    # Only URI syntax is repaired in this offline copy. Geometry and SRDF are unchanged.
    urdf=ET.parse(a.run/'robot.urdf');mesh_hashes={}
    for mesh in urdf.getroot().iter('mesh'):
        uri=mesh.get('filename')
        if uri.startswith('/'):
            mesh_hashes[uri]=hashlib.sha256(Path(uri).read_bytes()).hexdigest();mesh.set('filename','file://'+uri)
        elif uri.startswith('file://'):mesh_hashes[uri]=hashlib.sha256(Path(uri[7:]).read_bytes()).hexdigest()
        else:raise ValueError('mesh URI must be resolved by scene preparation: '+uri)
    urdf.write(out/'robot.urdf',encoding='utf-8')
    srdf=Path(get_package_share_directory('s622_moveit_config'))/'config/s622_dual_arm.srdf'
    (out/'robot.srdf').write_bytes(srdf.read_bytes())
    if not np.allclose(scene['sample']['pose'],[0,0,.275,0,0,0,1],atol=1e-9,rtol=0) or not np.allclose(scene['sample']['dimensions'],[.12,.05,.05],atol=1e-12,rtol=0):raise ValueError('scene sample differs from versioned reference contract')
    rows=planned_states(a.run,a.samples_per_segment)
    with (out/'states.txt').open('w') as f:
        for i,(phase,state) in enumerate(rows):f.write(f'{i} {phase} '+' '.join(f'{x:.17g}' for x in state)+'\n')
    inputs=dict(kind='planned reference path preflight',samples=len(rows),samples_per_segment=a.samples_per_segment,
        robot_urdf_sha256=hashlib.sha256((a.run/'robot.urdf').read_bytes()).hexdigest(),srdf_sha256=hashlib.sha256(srdf.read_bytes()).hexdigest(),mesh_sha256=mesh_hashes,
        path_sha256={x:hashlib.sha256((a.run/x).read_bytes()).hexdigest() for x in ('transport.json','approach.json')},
        reference_note='versioned joint reference path, explicit finger opening/closing; not exact future QP feedback trajectory',geometry=scene)
    (out/'inputs.json').write_text(json.dumps(inputs,indent=2)+'\n')
    if a.export_only:return 0
    binary=Path(get_package_prefix('dual_arm_pose_optimization'))/'lib/dual_arm_pose_optimization/c43_scene_path_check'
    with (out/'checker.log').open('w') as log:
        rc=subprocess.call([str(binary),str(out/'robot.urdf'),str(out/'robot.srdf'),str(out/'scene.txt'),str(out/'states.txt'),str(out)],stdout=log,stderr=subprocess.STDOUT)
    print('full-scene precheck:',out,'exit',rc,flush=True);return rc
if __name__=='__main__':raise SystemExit(main())
