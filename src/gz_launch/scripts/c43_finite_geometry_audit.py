#!/usr/bin/env python3
"""Offline finite-table SAT/FCL cross-check. Saves all cases, never a continuous proof."""
import argparse,csv,hashlib,json,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from c43_contact_safety import obb_overlap


def main():
    p=argparse.ArgumentParser();p.add_argument('world',type=Path);p.add_argument('out',type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    model_path=Path(get_package_share_directory('gz_launch'))/'models/table/model.sdf'
    include=next(i for i in ET.parse(a.world).getroot().findall('world/include') if i.findtext('name')=='c43_table')
    pose=np.array([float(x) for x in include.findtext('pose').split()]);assert np.allclose(pose[3:],0)
    shape=ET.parse(model_path).getroot().find("model/link/collision[@name='surface']")
    local=np.array([float(x) for x in shape.findtext('pose').split()]);assert np.allclose(local[3:],0)
    center=pose[:3]+local[:3];half=np.array([float(x) for x in shape.findtext('geometry/box/size').split()])/2
    # Current physical world: table top is 0, pedestal top is .25. Do not conflate.
    assert abs(center[2]+half[2])<1e-6
    source=Path(__file__).with_name('c43_obb_fcl_check.cpp');binary=a.out/'obb_fcl_oracle'
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','fcl'],text=True).split()
    subprocess.run(['g++','-O2','-std=c++17',str(source),'-o',str(binary)]+flags,check=True)
    rng=np.random.default_rng(20261113);cases=[]
    fixed=[('outside_edge_descent',[.82,0,-.01],False),('edge_penetration',[.80,0,-.01],True),('above_table',[0,0,.04],False),('inside_table',[0,0,-.01],True)]
    for name,pos,expected in fixed:cases.append(dict(name=name,center=(np.asarray(pos)+np.array([center[0],center[1],0])).tolist(),rotation=np.eye(3).tolist(),expected=expected))
    for i in range(200):
        q=rng.normal(size=4);q/=np.linalg.norm(q)
        from c43_contact_safety import rotation
        cases.append(dict(name=f'random_{i:03}',center=rng.uniform([-.9,-.5,-.08],[.9,.5,.08]).tolist(),rotation=rotation(q).tolist()))
    lines=[]
    for case in cases:
        chunks=[np.asarray(case['center']),np.asarray(case['rotation']),np.array([.06,.025,.025]),center,np.eye(3),half]
        lines.append(' '.join(str(x) for arr in chunks for x in arr.flatten()))
        case['sat']=obb_overlap(chunks[0],chunks[1],chunks[2],center,np.eye(3),half)
    output=subprocess.check_output([str(binary)],input='\n'.join(lines)+'\n',text=True).splitlines();assert len(output)==len(cases)
    for case,value in zip(cases,output):case['fcl']=bool(int(value));case['agree']=case['sat']==case['fcl']
    result=dict(scope='offline finite tabletop only; not online RT or whole-scene collision acceptance',seed=20261113,cases=len(cases),disagreements=sum(not c['agree'] for c in cases),fixed_cases_correct=all(c['sat']==c['expected'] for c in cases if 'expected' in c),table_center=center.tolist(),table_half_size=half.tolist(),table_top_z=float(center[2]+half[2]),source_world=str(a.world),source_sdf_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),all_cases=cases)
    result['passed']=result['disagreements']==0 and result['fixed_cases_correct']
    (a.out/'finite_geometry_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='all_cases'},indent=2));return int(not result['passed'])
if __name__=='__main__':raise SystemExit(main())
