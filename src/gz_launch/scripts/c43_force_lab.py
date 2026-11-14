#!/usr/bin/env python3
"""Prepare and supervise an isolated single/dual-arm diagnostic in container."""
import json
import os
from pathlib import Path
import signal
import shutil
import shlex
import math
import copy
import subprocess
import sys
import hashlib
import re
import time
import xml.etree.ElementTree as E
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from inject_joint_ft_sensor import SENSOR_TMPL

SCRIPTS=Path(__file__).resolve().parent
PROFILE=SCRIPTS.parent/'config/c43_contact_transport_v1'


def resolve_packages(text):
    return re.sub(r'package://([^/]+)/([^\s<>";]+)',
        lambda m:str(Path(get_package_share_directory(m[1]))/m[2]),text)


def cleanup_world(out):
    # Some Gazebo children outlive ros2 launch's process group on shutdown.
    # Match only this exact world, never all Gazebo processes or a ROS domain.
    world=str(out/'world.sdf')
    def matching():
        result=[]
        for row in subprocess.check_output(['ps','-eo','pid=,args='],text=True).splitlines():
            p=row.strip().split(None,1)
            if len(p)==2 and p[1].startswith('ign gazebo ') and world in p[1].split():
                result.append(int(p[0]))
        return result
    escalation=[]
    for sig in (signal.SIGTERM,signal.SIGKILL):
        pids=matching()
        if not pids:break
        for pid in pids:
            try:os.kill(pid,sig);escalation.append(dict(pid=pid,signal=sig.name))
            except ProcessLookupError:pass
        deadline=time.monotonic()+2
        while matching() and time.monotonic()<deadline:time.sleep(.1)
    remaining=matching()
    (out/'cleanup.json').write_text(json.dumps(dict(world=world,escalation=escalation,remaining_world_pids=remaining,clean=not remaining),indent=2)+'\n')
    if remaining:raise RuntimeError('experiment world still running: '+repr(remaining))


def main():
    out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
    mode=sys.argv[2] if len(sys.argv)>2 else 'calibration'
    for name in ('c43_force_lab.py','c43_ft_load_probe.py','c43_instrumented_grip.py',
                 'c43_force_pair.cpp','c43_position_grip_probe.py','c43_position_single.launch.py','c43_native_pose_record.py','c43_dual_candidate_dry.py','c43_dual_force.cpp','c43_dual_transport.py','c43_qp_contact_smoke.py','c43_contact_plant.py','c43_homing.py','inject_joint_ft_sensor.py','c43_run_contact_transport.py','c43_dual_transport_analyze.py','c43_qp_contact_analyze.py','c43_escape_audit.py','c43_w9_analyze.py','c43_w9_resume_analyze.py','c43_contact_safety.py','c43_contact_safety_run_analyze.py','c43_finite_geometry_audit.py','c43_obb_fcl_check.cpp','c43_scene_precheck.py','c43_scene_precheck_selftest.py','c43_finite_runtime_analyze.py'):
        shutil.copyfile(SCRIPTS/name,out/name)
    dual=os.environ.get('C43_LAB_DUAL')=='1'
    default_seed=('/workspace/log/c43_position_ab_20261111/w50_fixed_contact_r1' if dual else '/workspace/log/c43_position_ab_20261111/single_w50_fixed_r2')
    default_seed=PROFILE if mode=='dual_transport' else default_seed
    seed=Path(os.environ.get('C43_LAB_SEED',str(default_seed)))
    task=None
    if mode=='dual_transport':
        task=json.loads((PROFILE/'task.json').read_text())
        provenance=json.loads((PROFILE/'provenance.json').read_text())
        for name,item in provenance['inputs'].items():
            if hashlib.sha256((PROFILE/name).read_bytes()).hexdigest()!=item['packaged_sha256']:
                raise RuntimeError('packaged input checksum mismatch: '+name)
        for name,digest in provenance.get('added_resource_sha256',{}).items():
            if hashlib.sha256((PROFILE/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('packaged task checksum mismatch: '+name)
        for name in ('task.json','approach.json','transport.json','provenance.json','controllers.yaml'):
            shutil.copyfile(PROFILE/name,out/name)
    robot=E.ElementTree(E.fromstring(resolve_packages((seed/'robot.urdf').read_text())));root=robot.getroot()
    if os.environ.get('C43_ARM_BACKEND','').startswith('qp-'):
        shutil.copyfile(PROFILE/'qp_task.json',out/'qp_task.json')
        config=yaml.safe_load((PROFILE/'controllers.yaml').read_text())
        params=config['dual_arm_qp_controller']['ros__parameters']
        approach=json.loads((PROFILE/'approach.json').read_text())
        path=json.loads((PROFILE/'transport.json').read_text())
        params['home_q']=[float(x) for x in path['grasp_q12']]
        if os.environ.get('C43_ARM_BACKEND')=='qp-executor':
            params['require_executor_heartbeat']=True
            params['transport_status_rate_hz']=125.0
            executor_config={'c43_s4_transport_executor':{'ros__parameters':dict(
                plant_profile='physical_contact',use_sim_time=True,auto_start=True,
                enable_physical_hold_resume=os.environ.get('C43_ENABLE_HOLD_RESUME')=='1' or os.environ.get('C43_EXECUTOR_INJECT')=='hold-resume',
                inject_retreat_abort=os.environ.get('C43_EXECUTOR_INJECT')=='retreat-abort',
                artifacts_dir=str(out/'executor'),run_id=out.name,
                seat_overtravel_m=json.loads((PROFILE/'qp_task.json').read_text())['seat_overtravel_m'])}}
            (out/'executor.yaml').write_text(yaml.safe_dump(executor_config,sort_keys=False))
        transforms={}
        for side in ('left','right'):
            p=approach[side+'_tcp_world'];x,y,z,w=p[3:]
            r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            g=np.eye(4);g[:3,:3]=r;g[:3,3]=np.array(p[:3])-np.array(task['object']['initial_xyz_m'])
            params['object_T_'+side+'_tcp']=g.flatten().tolist();transforms[side]=g.tolist()
        (out/'controllers.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        next(root.iter('parameters')).text=str(out/'controllers.yaml')
        (out/'qp_adapter.json').write_text(json.dumps(dict(home_q=params['home_q'],object_T_tcp=transforms,
            scope=('physical contact adapter using shared C++ executor core; full W7 acceptance not claimed' if os.environ.get('C43_ARM_BACKEND')=='qp-executor' else 'isolated controller physical probe; not formal W7 executor integration'),existing_qp_thresholds_modified=False),indent=2)+'\n')
    if os.environ.get('C43_LAB_CONTACT_PROFILE')=='link_intended':
        changed=[]
        for gazebo in root.findall('gazebo'):
            ref=gazebo.get('reference','')
            if ref in ('left_finger1_joint','left_finger2_joint') and gazebo.find('mu1') is not None:
                gazebo.set('reference',ref[:-6]);changed.append([ref,ref[:-6]])
        assert len(changed)==2
        (out/'friction_reference_fix.json').write_text(json.dumps(changed,indent=2))
    if os.environ.get('C43_LAB_COLLISION')=='boxes':
        # STL is an extrusion rotated +0.5 deg about local Y; keep its union
        # envelope with three cuboids, mass/inertia and visual mesh unchanged.
        angle=math.radians(.5);c,s=math.cos(angle),math.sin(angle)
        boxes=[(-.01,-.0072,.0365,.0615),(-.0072,-.0022,.026,.0615),(-.0072,.0251,.02,.026)]
        for i in (1,2):
            link=root.find(f"link[@name='left_finger{i}']")
            for old in list(link.findall('collision')):link.remove(old)
            for j,(lo,hi,bottom,top) in enumerate(boxes):
                x=(lo+hi)/2*(1 if i==1 else -1);y=(bottom+top)/2
                collision=E.SubElement(link,'collision',name=f'left_finger{i}_box{j}_collision')
                E.SubElement(collision,'origin',xyz=f'{c*x} {y} {-s*x}',rpy=f'0 {angle} 0')
                geometry=E.SubElement(collision,'geometry')
                E.SubElement(geometry,'box',size=f'{hi-lo} {top-bottom} .02')
    force_sides=('left','right') if mode=='dual_transport' else ('left',)
    for side in force_sides:
        control=next(c for c in root.findall('ros2_control') if c.get('name','').startswith(side+'_'))
        for i in (1,2):
            sensor=E.SubElement(control,'sensor',name=f'{side}_finger{i}_joint_ft')
            for field in ('force.x','force.y','force.z','torque.x','torque.y','torque.z'):
                E.SubElement(sensor,'state_interface',name=field)
    robot.write(out/'robot.urdf',encoding='utf-8')
    converted=subprocess.run(['ign','sdf','-p',str(out/'robot.urdf')],capture_output=True,text=True,check=True)
    (out/'conversion.log').write_text(converted.stderr)
    sdf=E.fromstring(converted.stdout)
    if os.environ.get('C43_LAB_CONTACT_PROFILE')=='link_intended':
        for i in (1,2):
            link=next(l for l in sdf.iter('link') if l.get('name')==f'left_finger{i}')
            for collision in link.findall('collision'):
                assert float(collision.findtext('surface/friction/ode/mu'))==500
    if os.environ.get('C43_LAB_COLLISION')=='boxes':
        original=E.parse(seed/'robot.sdf').getroot()
        for i in (1,2):
            link=next(l for l in sdf.iter('link') if l.get('name')==f'left_finger{i}')
            old=next(l for l in original.iter('link') if l.get('name')==f'left_finger{i}').find('collision')
            for collision in link.findall('collision'):
                for child in list(collision):
                    if child.tag not in ('geometry','pose'):collision.remove(child)
                for child in old:
                    if child.tag not in ('geometry','pose'):collision.append(copy.deepcopy(child))
    for side in force_sides:
        for i in (1,2):
            name=f'{side}_finger{i}_joint';joint=next(x for x in sdf.iter('joint') if x.get('name')==name)
            joint.append(E.fromstring(SENSOR_TMPL.format(joint=name)))
    E.ElementTree(sdf).write(out/'robot.sdf',encoding='utf-8')
    world=E.parse(seed/'world.sdf');w=world.getroot().find('world')
    if os.environ.get('C43_PHYSICS_STEP_S'):
        step=float(os.environ['C43_PHYSICS_STEP_S'])
        if step not in (.001,.002,.005):raise ValueError('unsupported diagnostic physics step')
        physics=w.find('physics');physics.set('name',f'{step*1000:g}ms')
        physics.find('max_step_size').text=str(step)
    dx=float(os.environ.get('C43_LAB_SCENE_DX','0'))
    if dx:
        for include in w.findall('include'):
            if include.findtext('name')=='c43_support':
                p=include.find('pose');v=[float(x) for x in p.text.split()];v[0]+=dx;p.text=' '.join(map(str,v))
        for m in w.findall('model'):
            if m.get('name')=='c43_sample':
                p=m.find('pose');v=[float(x) for x in p.text.split()];v[0]+=dx;p.text=' '.join(map(str,v))
    for m in list(w.findall('model')):
        if m.get('name')=='c43_sample' and mode=='calibration':w.remove(m)
    E.SubElement(w,'plugin',filename='ignition-gazebo6-forcetorque-system',name='ignition::gazebo::systems::ForceTorque')
    E.SubElement(w,'plugin',filename='ignition-gazebo-apply-link-wrench-system',name='ignition::gazebo::systems::ApplyLinkWrench')
    if mode=='dual_transport' and os.environ.get('C43_EXECUTOR_INJECT')!='destination-absent':
        include=E.SubElement(w,'include');E.SubElement(include,'uri').text='model://c43_support'
        E.SubElement(include,'name').text='c43_destination';E.SubElement(include,'pose').text='0 .12 0 0 0 0'
    if os.environ.get('C43_EXECUTOR_INJECT') in ('unexpected-contact','finite-clearance'):
        obstacle=E.SubElement(w,'model',name='c43_w9_obstacle');E.SubElement(obstacle,'static').text='true'
        E.SubElement(obstacle,'pose').text='0 .07 .335 0 0 0'
        link=E.SubElement(obstacle,'link',name='base_link');collision=E.SubElement(link,'collision',name='collision')
        box=E.SubElement(E.SubElement(collision,'geometry'),'box');E.SubElement(box,'size').text='.02 .006 .08'
    world.write(out/'world.sdf',encoding='utf-8')
    if os.environ.get('C43_ENABLE_FINITE_GUARD')=='1':
        from c43_scene_precheck import export_scene
        finite_dir=out/'finite_scene';finite_dir.mkdir();export_scene(out,finite_dir)
        config=yaml.safe_load((out/'controllers.yaml').read_text())
        config['dual_arm_qp_controller']['ros__parameters']['finite_geometry_scene']=str(finite_dir/'scene.txt')
        config['dual_arm_qp_controller']['ros__parameters']['finite_geometry_stop_reserve_m']=float(os.environ.get('C43_FINITE_STOP_RESERVE_M','0'))
        (out/'controllers.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    if mode in ('force','dual_transport'):
        model=next(m for m in sdf.iter('model') if m.get('name')=='s622_dual_arm')
        for side in force_sides:
            for i in (1,2):
                plugin=E.SubElement(model,'plugin',filename='ignition-gazebo-apply-joint-force-system',name='ignition::gazebo::systems::ApplyJointForce')
                E.SubElement(plugin,'joint_name').text=f'{side}_finger{i}_joint'
        E.ElementTree(sdf).write(out/'robot.sdf',encoding='utf-8')
    assert len(root.findall('ros2_control'))==(2 if dual else 1)
    assert 'DetachableJoint' not in E.tostring(sdf,encoding='unicode')
    env=os.environ.copy();env.update(C43_AB_OUT=str(out),C43_AB_WIDTH='.05',C43_AB_MASS='.35',
        C43_AB_SINGLE='0' if dual else '1',C43_AB_MODE=mode,C43_LAB_MODE=mode,
        C43_AB_TARGET=sys.argv[3] if len(sys.argv)>3 else '.0249',ROS_DOMAIN_ID=os.environ.get('C43_ROS_DOMAIN_ID','146'),IGN_PARTITION='c43-force-'+out.name)
    if task:
        env.update(C43_TASK_JSON=str(out/'task.json'),C43_WAIT_CLOCK='sim')
        env.setdefault('C43_CANDIDATE_JSON',str(out/'approach.json'))
        env.setdefault('C43_TRANSPORT_JSON',str(out/'transport.json'))
        env.setdefault('C43_CLAMP_N',str(task['clamp_per_finger_N']))
    if mode in ('force','dual_transport'):
        binary=out.parent/('c43_dual_force' if mode=='dual_transport' else 'c43_force_pair')
        source=SCRIPTS/(binary.name+'.cpp')
        installed=Path(get_package_prefix('gz_launch'))/'lib/gz_launch/c43_dual_force'
        if mode=='dual_transport' and installed.exists():
            binary=installed
        elif not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:
            flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','ignition-transport11','ignition-msgs8'],text=True))
            subprocess.run(['g++','-O2',str(source),'-o',str(binary)]+flags,check=True)
        env['C43_FORCE_PUBLISHER']=str(binary)
    (out/'conditions.json').write_text(json.dumps({k:v for k,v in env.items() if k.startswith('C43_') or k in ('ROS_DOMAIN_ID','IGN_PARTITION')},indent=2))
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
    mesh_files=sorted({x.get('filename') for x in root.iter('mesh')})
    for name in mesh_files:
        if not Path(name).is_file():raise RuntimeError('mesh resource missing: '+name)
    metadata=dict(profile=task['profile'] if task else 'legacy_diagnostic',seed=str(seed),
        wait_clock=env.get('C43_WAIT_CLOCK','wall'),arm_backend=env.get('C43_ARM_BACKEND','jtc'),physics_step_s=float(w.findtext('physics/max_step_size')),
        source_files_sha256=hashes,mesh_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in mesh_files},
        controllers_sha256=hashlib.sha256(Path(next(root.iter('parameters')).text).read_bytes()).hexdigest(),
        ros_domain_id=env['ROS_DOMAIN_ID'],ign_partition=env['IGN_PARTITION'],
        diagnostic_only=True,formal_gate_pass=False)
    commit=subprocess.run(['git','rev-parse','HEAD'],text=True,capture_output=True)
    metadata['git_commit']=commit.stdout.strip() if commit.returncode==0 else None
    dirty=subprocess.run(['git','status','--porcelain'],text=True,capture_output=True)
    metadata['git_worktree_dirty']=bool(dirty.stdout) if dirty.returncode==0 else None
    if task:metadata['profile_source_git_commit']=provenance.get('git_commit_at_packaging')
    if env.get('C43_FORCE_PUBLISHER'):
        metadata['force_executable_sha256']=hashlib.sha256(Path(env['C43_FORCE_PUBLISHER']).read_bytes()).hexdigest()
    if env.get('C43_ARM_BACKEND','').startswith('qp-'):
        qp_library=Path(get_package_prefix('dual_arm_qp_controller'))/'lib/libdual_arm_qp_controller.so'
        metadata['qp_controller_library_sha256']=hashlib.sha256(qp_library.read_bytes()).hexdigest() if qp_library.exists() else None
    if env.get('C43_ARM_BACKEND')=='qp-executor':
        executor_binary=Path(get_package_prefix('dual_arm_transport_executor'))/'lib/dual_arm_transport_executor/c43_s4_executor'
        metadata['executor_binary_sha256']=hashlib.sha256(executor_binary.read_bytes()).hexdigest()
    metadata['controller_config_update_rate_hz']=125
    metadata['controller_observed_update_rate_hz']=None
    metadata['qp_config_dt_s']=.008
    metadata['physics_step_variant_explicit']='C43_PHYSICS_STEP_S' in env
    metadata['physics_random_seed']='engine default; no explicit seed override'
    metadata['runtime_argv']=sys.argv
    metadata['gazebo_versions']=subprocess.run(['ign','gazebo','--versions'],text=True,capture_output=True,timeout=10).stdout.strip()
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    if env.get('C43_SCENE_PREFLIGHT')=='1':
        result=subprocess.run(['python3',str(out/'c43_scene_precheck.py'),str(out)],env=env)
        passed=result.returncode==0
        (out/'preflight_gate.json').write_text(json.dumps(dict(passed=passed,checker_exit=result.returncode,simulation_started=False,
            reason='ok' if passed else 'precheck_failed',scope='all versioned reference segments checked before launch; no continuous or exact QP rollout claim'),indent=2)+'\n')
        if not passed:
            print('scene preflight rejected launch; no simulator or actuator started',flush=True);return 3
    if os.environ.get('C43_LAB_PREPARE_ONLY')=='1':
        print('prepared installed resources:',out,flush=True);return 0
    with (out/'launch.log').open('w') as lf,(out/'probe.log').open('w') as pf:
        launch=subprocess.Popen(['ros2','launch',str(out/'c43_position_single.launch.py')],env=env,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True)
        if env.get('C43_SCENE_PREFLIGHT')=='1':
            gate_path=out/'preflight_gate.json';gate=json.loads(gate_path.read_text())
            gate.pop('simulation_started',None);gate.update(simulation_started_before_precheck=False,simulation_launch_requested=True,launch_pid=launch.pid)
            gate_path.write_text(json.dumps(gate,indent=2)+'\n')
        probe=None
        try:
            script=('c43_ft_load_probe.py' if mode=='calibration' else
                    'c43_dual_candidate_dry.py' if mode=='dual_dry' else
                    ('c43_contact_plant.py' if env.get('C43_ARM_BACKEND')=='qp-executor' else 'c43_qp_contact_smoke.py' if env.get('C43_ARM_BACKEND','').startswith('qp-') else 'c43_dual_transport.py') if mode=='dual_transport' else 'c43_instrumented_grip.py')
            probe=subprocess.Popen(['python3','-u',str(out/script)],env=env,stdout=pf,stderr=subprocess.STDOUT,start_new_session=True)
            rc=probe.wait(timeout=480 if env.get('C43_ARM_BACKEND')=='qp-executor' else 360 if mode=='dual_transport' else 210);print('probe_exit',rc,flush=True)
            if os.environ.get('C43_GUI_KEEP_OPEN')=='1' and (
                    os.environ.get('C43_LAB_GUI')=='1' or os.environ.get('C43_EXTERNAL_GUI')=='1'):
                print('Experiment finished; server stays open for the GUI. Ctrl+C closes this isolated run.',flush=True)
                launch.wait()
        finally:
            for proc in (probe,launch):
                if proc is None:continue
                try:os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:pass
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
            cleanup_world(out)
            print('isolated cleanup complete',flush=True)
    return rc

if __name__=='__main__':raise SystemExit(main())
