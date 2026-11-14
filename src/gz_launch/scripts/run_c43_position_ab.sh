#!/usr/bin/env bash
# Container-only, isolated simulation; usage: OUT WIDTH MODE TARGET [MASS].
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
export C43_AB_OUT="$1" C43_AB_WIDTH="$2" C43_AB_MODE="$3" C43_AB_TARGET="$4"
export C43_AB_MASS="${5:-0.35}"
export C43_AB_SINGLE="${C43_AB_SINGLE:-0}"
mkdir -p "$C43_AB_OUT"
cp src/gz_launch/scripts/c43_position_grip_probe.py "$C43_AB_OUT/probe_source.py"
cp src/gz_launch/scripts/run_c43_position_ab.sh "$C43_AB_OUT/runner_source.sh"
export ROS_DOMAIN_ID=145 IGN_PARTITION="c43-ab-$(basename "$C43_AB_OUT")"
export C43_DISABLE_HOMING=1 C43_GRIPPER_FORCE_PLUGINS=0
unset C43_FT_SENSOR
export C43_S4_WORLD_FILE="$C43_AB_OUT/world.sdf" C43_ROBOT_SDF="$C43_AB_OUT/robot.sdf"
qp_share=$(ros2 pkg prefix --share dual_arm_qp_controller)
desc_share=$(ros2 pkg prefix --share s622_moveit_descriptions)
xacro src/gz_launch/config/c43_s4_dual_arm_gazebo.urdf.xacro \
  arm_command_mode:=position arm_position_gain:=20.0 \
  include_wrist_camera:=false gripper_force_plugins:=false \
  controllers_yaml:="$qp_share/config/dual_arm_qp_gazebo_controllers.yaml" \
  > "$C43_AB_OUT/robot.xacro.urdf"
python3 - "$desc_share" <<'PY'
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as E
out=Path(os.environ['C43_AB_OUT']); width=float(os.environ['C43_AB_WIDTH']); mass=float(os.environ['C43_AB_MASS'])
robot=E.fromstring((out/'robot.xacro.urdf').read_text().replace('package://s622_moveit_descriptions',sys.argv[1]))
single=os.environ['C43_AB_SINGLE']=='1'
if single:
    for item in list(robot):
        remove=False
        if item.tag in ('link','ros2_control','transmission'):
            remove=item.get('name','').startswith('right_')
        elif item.tag=='joint':
            remove=any(e.get('link','').startswith('right_') for e in item if e.tag in ('parent','child'))
        elif item.tag=='gazebo':
            remove=item.get('reference','').startswith('right_')
        if remove: robot.remove(item)
    assert len(robot.findall('ros2_control'))==1
    assert not any(x.get('name','').startswith('right_') for x in robot.findall('link'))
for parent in robot.iter():
    for p in list(parent):
        if p.tag=='plugin' and ('DetachableJoint' in p.get('name','') or 'ApplyJointForce' in p.get('name','')):
            parent.remove(p)
E.ElementTree(robot).write(out/'robot.urdf',encoding='utf-8')
world=E.parse('src/gz_launch/worlds/c43_s4_world_contact.sdf'); w=world.getroot().find('world')
for include in list(w.findall('include')):
    if 'c43_sample' in include.findtext('uri',''): w.remove(include)
sample=E.parse('src/gz_launch/models/c43_sample_contact/model.sdf').getroot().find('model')
sample.set('name','c43_sample'); sample.find('pose').text='0 0 0.275 0 0 0'
for size in sample.findall('.//box/size'): size.text=f'0.12 {width} 0.05'
sample.find('.//inertial/mass').text=str(mass)
inertia=sample.find('.//inertia'); m=mass; x=.12; z=.05
for name,value in [('ixx',m*(width**2+z*z)/12),('iyy',m*(x*x+z*z)/12),('izz',m*(x*x+width**2)/12)]:
    inertia.find(name).text=str(value)
sample.find('.//sensor/topic').text='/c43/position_ab/contact'
w.append(sample); world.write(out/'world.sdf',encoding='utf-8')
(out/'conditions.json').write_text(__import__('json').dumps(dict(width=width,height=.05,length=.12,mass=mass,
    mode=os.environ['C43_AB_MODE'],target=float(os.environ['C43_AB_TARGET']),single_robot=single,weld=False,force_plugin=False,
    ros_domain_id=os.environ['ROS_DOMAIN_ID'],ign_partition=os.environ['IGN_PARTITION']),indent=2))
PY
ign sdf -p "$C43_AB_OUT/robot.urdf" > "$C43_ROBOT_SDF"
python3 - <<'PY'
import os,signal,subprocess,sys
from pathlib import Path
p=Path(os.environ['C43_AB_OUT'])
assert 'DetachableJoint' not in (p/'robot.sdf').read_text()
assert 'ApplyJointForce' not in (p/'robot.sdf').read_text()
with (p/'launch.log').open('w') as lf,(p/'probe.log').open('w') as pf:
    command=(['ros2','launch','src/gz_launch/scripts/c43_position_single.launch.py']
             if os.environ['C43_AB_SINGLE']=='1' else
             ['ros2','launch','gz_launch','c43_s4_full_flow.launch.py','gui:=false','executor:=false'])
    launch=subprocess.Popen(command,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True)
    probe=None
    try:
        probe=subprocess.Popen(['python3','-u','src/gz_launch/scripts/c43_position_grip_probe.py'],stdout=pf,stderr=subprocess.STDOUT,start_new_session=True)
        rc=probe.wait(timeout=150)
        print('probe_exit=',rc,flush=True)
    finally:
        for proc in (probe,launch):
            if proc is None: continue
            try: os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError: pass
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired: pass
            try: os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError: pass
        print('cleanup complete',flush=True)
    sys.exit(rc)
PY
