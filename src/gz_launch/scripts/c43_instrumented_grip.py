#!/usr/bin/env python3
"""Left grip position-preload / native drive experiment; optional parked right arm."""
import csv
import json
import os
import subprocess
import time
import numpy as np
import rclpy
from c43_position_grip_probe import Probe, OUT, FINGERS, TARGET
from c43_ft_load_probe import WrenchStream, quaternion_matrix
from c43_homing import LEFT_JOINTS, RIGHT_JOINTS, PRE_LEFT, DESCENT_LEFT

MODE=os.environ['C43_LAB_MODE']
CLAMP=float(os.environ.get('C43_CLAMP_N','2'))
HOLD_S=float(os.environ.get('C43_HOLD_LIFT_S','5'))
AXIS=np.array([.99996,0.,-.0087267]);AXIS/=np.linalg.norm(AXIS)
HAND_JOINTS=FINGERS[:2]


class Instrumented(Probe):
    def __init__(self):
        super().__init__()
        self.ft=[WrenchStream(f'/left_finger{i}_joint_ft',OUT/f'ft_finger{i}_raw.txt') for i in (1,2)]
        self.wrench_rows=[]
        self.v={}
        self.overtravel_since=None
        self.publisher=(subprocess.Popen([os.environ.get('C43_FORCE_PUBLISHER','/workspace/log/c43_force_20261111/c43_force_pair')],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) if MODE=='force' else None)
        self.native=None
        if os.environ.get('C43_CAPTURE_NATIVE_POSES')=='1':
            from c43_native_pose_record import NativePoseRecorder
            self.native=NativePoseRecorder(OUT)

    def force(self, value):
        us=self.force_pair(-value,value)
        self.event('joint_drive_N',closing_per_finger=value,pair_publish_us=us)

    def force_pair(self,a,b):
        self.publisher.stdin.write(f'{a} {b}\n');self.publisher.stdin.flush()
        answer=self.publisher.stdout.readline()
        if not answer:raise RuntimeError('native force publisher disconnected')
        return int(answer)

    def estimated_normal(self,i,wrench):
        pose=self.pose(f'left_finger{i}')
        if pose is None:return None
        gravity=quaternion_matrix(pose[3:]).T@np.array([0,0,-.008826*9.8])
        external=np.array(wrench[:3])-gravity
        return float((1 if i==1 else -1)*np.dot(external,AXIS))

    def on_js(self,msg):
        self.v=dict(zip(msg.name,msg.velocity))
        super().on_js(msg)
        if not hasattr(self,'ft'):return
        record=dict(mono=time.monotonic()-self.t0,sim=self.sim,phase=self.phase)
        for i,stream in enumerate(self.ft,1):
            with stream.lock:latest=stream.rows[-1] if stream.rows else None
            record[f'ft{i}']=latest
            record[f'estimated_normal{i}_N']=self.estimated_normal(i,latest['wrench']) if latest else None
        self.wrench_rows.append(record)
        if MODE=='force' and self.phase in ('close','settle','lift','hold_lift'):
            if sum(abs(self.q.get(j,.03)) for j in HAND_JOINTS)<.048:
                if self.overtravel_since is None:self.overtravel_since=self.sim
                elif self.sim is not None and self.sim-self.overtravel_since>=.03:
                    raise RuntimeError('diagnostic stop: total joint opening <48 mm for 30 ms sim')
            else:self.overtravel_since=None

    def snapshot(self,label):
        self.event(label,sample=self.pose('c43_sample'),q={j:self.q.get(j) for j in FINGERS},
            wrench=[s.sample() for s in self.ft])

    def run(self):
        candidate=None
        if os.environ.get('C43_CANDIDATE_JSON'):
            candidate=json.loads(open(os.environ['C43_CANDIDATE_JSON']).read())
            if not candidate['path_collision_free']:raise RuntimeError('candidate path did not pass collision screening')
        names=['left_arm_controller']+([] if MODE=='force' else ['left_hand_controller'])
        if os.environ.get('C43_LAB_DUAL')=='1':names+=['right_arm_controller','right_hand_controller']
        for name in names:
            if not self.action_clients[name].wait_for_server(timeout_sec=45):raise RuntimeError(name+' unavailable')
        self.spin(1)
        if os.environ.get('C43_LAB_DUAL')=='1':
            right_hold=candidate['right_hold'] if candidate else [0,-1.5708,0,0,0,0]
            self.trajectory('right_arm_controller',RIGHT_JOINTS,[right_hold],1)
            self.trajectory('right_hand_controller',FINGERS[2:],[[.0305,-.0305]],1.5)
        if MODE=='force':
            self.force(-2);self.spin(2)
            if any(abs(self.q.get(j,0))<.03 for j in HAND_JOINTS):raise RuntimeError('force-open did not reach open limit')
        else:self.trajectory('left_hand_controller',HAND_JOINTS,[[.0305,-.0305]],1.5)
        self.phase='homing'
        self.trajectory('left_arm_controller',LEFT_JOINTS,[candidate['left_pre'] if candidate else PRE_LEFT],8)
        self.trajectory('left_arm_controller',LEFT_JOINTS,candidate['left_descent'] if candidate else DESCENT_LEFT,6)
        self.phase='preclose';self.spin(2)
        with self.lock:
            if not self.contacts:raise RuntimeError('no native contact data')
            if any(c['fingers'] for c in self.contacts if c['phase']=='homing'):
                raise RuntimeError('sample touched during homing')
        self.snapshot('preclose')
        self.phase='close'
        if MODE=='force':
            # Slow velocity-feedback approach, then ramp constant drive only
            # after BOTH fingers appear in a full top-level Contacts message.
            start=time.monotonic();matched=False
            while time.monotonic()-start<25:
                with self.lock:
                    recent=[c for c in self.contacts[-60:] if c['phase']=='close']
                    matched=any('left_finger1' in ' '.join(c['fingers']) and
                                'left_finger2' in ' '.join(c['fingers']) for c in recent)
                if matched:break
                forces=[]
                for i,sign in ((1,-1),(2,1)):
                    velocity=self.v.get(FINGERS[i-1],float('nan'))
                    if not np.isfinite(velocity):raise RuntimeError('joint velocity missing for slow force approach')
                    p=self.pose(f'left_finger{i}')
                    gravity=float(np.dot(quaternion_matrix(p[3:]).T@np.array([0,0,-.008826*9.8]),AXIS))
                    forces.append(float(np.clip(.008826*10*(sign*.0007-velocity)-gravity,-CLAMP,CLAMP)))
                self.force_pair(*forces);self.spin(.02)
            if not matched:raise RuntimeError('bounded slow force approach did not establish two-finger contact')
            self.event('two_fingers_before_force_ramp')
            for level in (.25,.5,.75,1.):self.force(CLAMP*level);self.spin(1)
        else:self.trajectory('left_hand_controller',HAND_JOINTS,[[TARGET,-TARGET]],8)
        self.phase='settle';self.spin(3);self.snapshot('prelift')
        self.phase='lift'
        lift=candidate['left_lift'][1:] if candidate else list(reversed(DESCENT_LEFT))[1:]
        self.trajectory('left_arm_controller',LEFT_JOINTS,lift,8)
        self.phase='hold_lift';self.spin(HOLD_S);self.snapshot('lift_end')

    def finish(self,error):
        if MODE=='force':
            try:self.force(0)
            except Exception as exc:self.event('zero_drive_failed',message=repr(exc))
            self.publisher.stdin.close()
            try:self.publisher.wait(timeout=3)
            except subprocess.TimeoutExpired:self.publisher.terminate();self.publisher.wait()
        for stream in self.ft:stream.close()
        if self.native:self.native.close()
        self.save(error)
        with (OUT/'wrench_telemetry.jsonl').open('w') as f:
            for row in self.wrench_rows:f.write(json.dumps(row)+'\n')
        stats={}
        for phase in ('preclose','close','settle','lift','hold_lift'):
            stats[phase]={}
            for i in (1,2):
                values=[r[f'estimated_normal{i}_N'] for r in self.wrench_rows if r['phase']==phase and r[f'estimated_normal{i}_N'] is not None]
                if values:stats[phase][f'finger{i}']=dict(mean=float(np.mean(values)),min=float(min(values)),max=float(max(values)))
        d=json.loads((OUT/'summary.json').read_text());d.update(drive_mode=MODE,clamp_command_N=CLAMP if MODE=='force' else None,
            estimated_external_normal_stats=stats,hold_lift_duration_s=HOLD_S)
        (OUT/'instrumented_summary.json').write_text(json.dumps(d,indent=2)+'\n')


def main():
    rclpy.init();node=Instrumented();error=None
    try:node.run()
    except Exception as exc:error=repr(exc);node.event('error',message=error)
    finally:
        node.finish(error);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return int(error is not None)

if __name__=='__main__':raise SystemExit(main())
