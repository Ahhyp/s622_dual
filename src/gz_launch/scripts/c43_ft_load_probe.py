#!/usr/bin/env python3
"""Native joint-wrench calibration at mid-stroke, home and grasp orientation."""
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from c43_position_grip_probe import Probe, OUT, FINGERS
from c43_homing import LEFT_JOINTS, PRE_LEFT, DESCENT_LEFT
FINGER=int(os.environ.get('C43_FT_FINGER','1'))


class WrenchStream:
    def __init__(self, topic, output):
        self.rows=[]; self.lock=threading.Lock()
        self.raw=output.open('w')
        self.proc=subprocess.Popen(['ign','topic','-e','-t',topic],stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,text=True,bufsize=1)
        self.thread=threading.Thread(target=self.read,daemon=True); self.thread.start()

    def read(self):
        buf=[]
        for line in self.proc.stdout:
            self.raw.write(line)
            if line.strip(): buf.append(line); continue
            if not buf: continue
            text=''.join(buf); buf=[]; values=[]
            for kind in ('force','torque'):
                match=re.search(r'\b'+kind+r'\s*\{([^{}]*)\}',text)
                axes=dict(re.findall(r'\b([xyz]):\s*(\S+)',match.group(1))) if match else {}
                values.extend(float(axes.get(a,'0')) for a in 'xyz')
            stamp=re.search(r'stamp\s*\{([^{}]*)\}',text)
            axes=dict(re.findall(r'\b(sec|nsec):\s*(\d+)',stamp.group(1))) if stamp else {}
            row=dict(mono=time.monotonic(),sim=int(axes.get('sec',0))+1e-9*int(axes.get('nsec',0)),wrench=values)
            if not all(math.isfinite(x) for x in values): continue
            with self.lock: self.rows.append(row)

    def sample(self,sim_now=None):
        with self.lock:
            rows=[r for r in self.rows if (r['mono']>time.monotonic()-.5 if sim_now is None else sim_now-.5<r['sim']<=sim_now+.03)]
        if len(rows)<20 or rows[-1]['sim']<=rows[0]['sim']:
            raise RuntimeError('FT stream missing/stale or fewer than 20 fresh samples')
        if sim_now is not None and (sim_now-rows[-1]['sim']>.1 or time.monotonic()-rows[-1]['mono']>10):
            raise RuntimeError('FT stream stale relative to simulation/wall clock')
        a=np.array([r['wrench'] for r in rows])
        return dict(n=len(rows),first_sim=rows[0]['sim'],last_sim=rows[-1]['sim'],
                    mean=a.mean(axis=0).tolist(),std=a.std(axis=0).tolist())

    def close(self):
        self.proc.terminate()
        try: self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired: self.proc.kill(); self.proc.wait()
        self.thread.join(timeout=3); self.raw.close()


def quaternion_matrix(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class Calibration(Probe):
    def __init__(self):
        super().__init__(); self.loads=[]
        self.ft=WrenchStream(f'/left_finger{FINGER}_joint_ft',OUT/'ft_raw.txt')

    def publish(self, force=None):
        entity=f'name: "s622_dual_arm::left_finger{FINGER}" type: LINK'
        if force is None:
            topic='/world/c43_s4_world/wrench/clear'; kind='ignition.msgs.Entity'; msg=entity
        else:
            topic='/world/c43_s4_world/wrench/persistent'; kind='ignition.msgs.EntityWrench'
            msg='entity { '+entity+' } wrench { force { '+ ' '.join(f'{a}: {v}' for a,v in zip('xyz',force))+' } }'
        subprocess.run(['ign','topic','-t',topic,'-m',kind,'-p',msg],check=True,timeout=8)

    def state(self):
        return dict(q={j:self.q.get(j) for j in FINGERS},finger=self.pose(f'left_finger{FINGER}'),
                    wrist=self.pose('left_wrist3_link'))

    def one_load(self, pose_name, force, label):
        self.publish(); self.spin(1.0)
        baseline=self.ft.sample(); before=self.state()
        try:
            self.publish(force); self.spin(1.5)
            loaded=self.ft.sample(); during=self.state()
        finally:
            self.publish()
        self.spin(1.0); cleared=self.ft.sample(); after=self.state()
        record=dict(pose=pose_name,label=label,world_force=force,baseline=baseline,loaded=loaded,
            cleared=cleared,before=before,during=during,after=after,
            delta=(np.array(loaded['mean'])-baseline['mean']).tolist(),
            return_delta=(np.array(cleared['mean'])-baseline['mean']).tolist())
        if before['finger']:
            rotation=quaternion_matrix(before['finger'][3:])
            record['expected_child_force']= (rotation.T@np.array(force)).tolist()
        record['joint_motion_m']=during['q'][FINGERS[FINGER-1]]-before['q'][FINGERS[FINGER-1]]
        self.loads.append(record); self.event('load_result',**record)
        (OUT/'calibration.json').write_text(json.dumps(self.loads,indent=2)+'\n')

    def run(self):
        for name,cli in self.action_clients.items():
            if not cli.wait_for_server(timeout_sec=45): raise RuntimeError(name+' unavailable')
        self.spin(2)
        self.phase='midstroke'
        self.trajectory('left_hand_controller',FINGERS,[[.02,-.02]],2)
        for pose in ('home','grasp'):
            if pose=='home' and os.environ.get('C43_FT_GRASP_ONLY')=='1':continue
            if pose=='grasp':
                self.phase='homing'
                self.trajectory('left_arm_controller',LEFT_JOINTS,[PRE_LEFT],8)
                self.trajectory('left_arm_controller',LEFT_JOINTS,DESCENT_LEFT,6)
            self.phase=pose; self.spin(2)
            for axis in range(3):
                for magnitude in (2.,-2.,5.):
                    force=[0.,0.,0.];force[axis]=magnitude
                    self.one_load(pose,force,f'world_{"xyz"[axis]}_{magnitude:+g}N')
        fits={}
        for pose in ('home','grasp'):
            rows=[r for r in self.loads if r['pose']==pose]
            if not rows:continue
            a=np.array([r['world_force'] for r in rows]); b=np.array([r['delta'][:3] for r in rows])
            gain=np.linalg.lstsq(a,b,rcond=None)[0]
            fits[pose]=dict(world_force_to_child_force=gain.tolist(),singular_values=np.linalg.svd(gain,compute_uv=False).tolist(),
                max_return_force_norm=max(float(np.linalg.norm(r['return_delta'][:3])) for r in rows),
                max_joint_motion_m=max(abs(r['joint_motion_m']) for r in rows))
        (OUT/'calibration_fit.json').write_text(json.dumps(fits,indent=2)+'\n')
        print(json.dumps(fits,indent=2),flush=True)


def main():
    rclpy.init(); node=Calibration(); error=None
    try: node.run()
    except Exception as exc: error=repr(exc); node.event('error',message=error)
    finally:
        try: node.publish()
        finally:
            node.ft.close(); node.save(error); node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    return int(error is not None)

if __name__=='__main__':raise SystemExit(main())
