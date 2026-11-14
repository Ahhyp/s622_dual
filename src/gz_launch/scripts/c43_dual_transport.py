#!/usr/bin/env python3
"""Physical four-finger constant-drive pick/carry/place, no weld or FT feedback."""
import json
import math
import os
import subprocess
import time
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from c43_position_grip_probe import Probe, OUT, FINGERS
from c43_homing import LEFT_JOINTS, RIGHT_JOINTS
from c43_ft_load_probe import WrenchStream, quaternion_matrix
from c43_native_pose_record import NativePoseRecorder

AXIS = np.array([.99996, 0., -.0087267]); AXIS /= np.linalg.norm(AXIS)
CLAMP = float(os.environ.get('C43_CLAMP_N', '5'))


class Transport(Probe):
    def __init__(self):
        super().__init__()
        self.v = {}
        self.ft = [WrenchStream('/'+joint+'_ft', OUT/(joint+'_ft_raw.txt')) for joint in FINGERS]
        self.native = NativePoseRecorder(OUT)
        self.publisher = subprocess.Popen([os.environ['C43_FORCE_PUBLISHER']],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.ft_rows = []
        self.guard_since = None
        self.carry_offsets = None
        self.limit_warned = set()
        self.task=json.loads(open(os.environ['C43_TASK_JSON']).read()) if os.environ.get('C43_TASK_JSON') else {}
        self.waits=self.task.get('waits_sim_s',{})
        self.durations=self.task.get('durations_sim_s',{})
        self.last_clock_progress=time.monotonic()

    def on_clock(self,msg):
        previous=self.sim
        super().on_clock(msg)
        if self.sim!=previous:self.last_clock_progress=time.monotonic()

    def spin(self,seconds):
        if os.environ.get('C43_WAIT_CLOCK')!='sim':return super().spin(seconds)
        watchdog=self.task.get('clock_stall_watchdog_wall_s',10.)
        initial_wall=time.monotonic()
        while self.sim is None:
            rclpy.spin_once(self,timeout_sec=.01)
            if time.monotonic()-initial_wall>watchdog:raise RuntimeError('simulation clock unavailable')
        start=self.sim
        while self.sim-start<seconds:
            rclpy.spin_once(self,timeout_sec=.01)
            if self.sim<start:raise RuntimeError('simulation clock moved backwards')
            if time.monotonic()-self.last_clock_progress>watchdog:raise RuntimeError('simulation clock stalled (wall watchdog)')

    def wait_stage(self,name,default):
        self.spin(float(self.waits.get(name,default)))

    def drive(self, values, log=False):
        self.publisher.stdin.write(' '.join(str(float(v)) for v in values)+'\n')
        self.publisher.stdin.flush()
        answer = self.publisher.stdout.readline()
        if not answer:
            raise RuntimeError('four-finger publisher disconnected')
        if log:
            self.event('joint_drives_N', values=values, pair_publish_us=int(answer))

    def clamp(self, value):
        self.drive([-value, value, -value, value], log=True)

    def on_js(self, msg):
        self.v.update(zip(msg.name, msg.velocity))
        super().on_js(msg)
        if not hasattr(self, 'ft'):
            return
        latest = []
        for stream in self.ft:
            with stream.lock:
                latest.append(stream.rows[-1] if stream.rows else None)
        self.ft_rows.append(dict(sim=self.sim, phase=self.phase, ft=latest))
        if self.phase in ('close', 'settle', 'lift', 'hold_lift', 'transfer', 'place'):
            if all(j in self.q for j in FINGERS):
                narrow = any(abs(self.q[FINGERS[i]])+abs(self.q[FINGERS[i+1]]) < self.task.get('opening_stop_m',.048) for i in (0, 2))
                if narrow:
                    if self.guard_since is None:
                        self.guard_since = self.sim
                    elif self.sim-self.guard_since > self.task.get('opening_stop_duration_sim_s',.03):
                        raise RuntimeError('diagnostic stop: a gripper opening <48 mm for 30 ms')
                else:
                    self.guard_since = None
            if self.carry_offsets is not None:
                sample = np.array(self.pose('c43_sample')[:3])
                predicted = np.mean([np.array(self.pose(side+'_wrist3_link')[:3])+offset
                    for side, offset in zip(('left', 'right'), self.carry_offsets)], axis=0)
                if np.linalg.norm(sample-predicted) > self.task.get('carry_discrepancy_stop_m',.015):
                    raise RuntimeError('diagnostic stop: sample/carry pose discrepancy >15 mm')
        if self.phase == 'homing' and any(c.get('robot_links', c['fingers']) for c in self.contacts[-20:]):
            raise RuntimeError('unexpected sample contact during approach')

    def snapshot(self, label):
        self.event(label, sample=self.pose('c43_sample'),
            wrists={s:self.pose(s+'_wrist3_link') for s in ('left','right')},
            fingers={j:self.q.get(j) for j in FINGERS}, ft=[s.sample(sim_now=self.sim if os.environ.get('C43_WAIT_CLOCK')=='sim' else None) for s in self.ft])

    def synchronized(self, q12_points, duration):
        if getattr(self,'stop_requested',False):raise RuntimeError('executor STOP latched before synchronized goals')
        # Common simulation-clock start time; independent goals sent before waiting.
        if self.sim is None:
            raise RuntimeError('no simulation clock')
        start_time = self.sim+self.task.get('sync_lead_sim_s',.5)
        futures = []
        for side, joints, offset in [('left', LEFT_JOINTS, 0), ('right', RIGHT_JOINTS, 6)]:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            goal.trajectory.header.stamp.sec = int(start_time)
            goal.trajectory.header.stamp.nanosec = int((start_time-int(start_time))*1e9)
            for k, q in enumerate(q12_points):
                p = JointTrajectoryPoint(); p.positions = [float(v) for v in q[offset:offset+6]]
                t = duration*(k+1)/len(q12_points)
                p.time_from_start.sec = int(t); p.time_from_start.nanosec = int((t-int(t))*1e9)
                goal.trajectory.points.append(p)
            futures.append(self.action_clients[side+'_arm_controller'].send_goal_async(goal))
        handles = [self.wait_future(f, 10) for f in futures]
        if not all(h and h.accepted for h in handles):
            raise RuntimeError('synchronized arm goal rejected')
        self.event('synchronized_goals_accepted', start_sim=start_time, duration=duration, target=q12_points[-1])
        self.active_goal_handles=handles
        if getattr(self,'stop_requested',False):
            for handle in handles:self.cancel_for_stop(handle)
            raise RuntimeError('executor STOP during synchronized goal acceptance')
        results = [h.get_result_async() for h in handles]
        for result in results:
            if os.environ.get('C43_WAIT_CLOCK')=='sim':
                while not result.done():
                    rclpy.spin_once(self,timeout_sec=.01)
                    if getattr(self,'stop_requested',False):
                        for handle in handles:self.cancel_for_stop(handle)
                        raise RuntimeError('executor STOP: cancel both synchronized JTC goals')
                    if self.sim>start_time+duration+15:raise RuntimeError('arm result exceeded simulation deadline')
                    if time.monotonic()-self.last_clock_progress>self.task.get('clock_stall_watchdog_wall_s',10.):
                        raise RuntimeError('simulation clock stalled during synchronized trajectory')
                r=result.result()
            else:r = self.wait_future(result, duration+20)
            self.event('synchronized_action_result', status=r.status, error_code=r.result.error_code, detail=r.result.error_string)
            if r.status != 4 or r.result.error_code != 0:
                raise RuntimeError('synchronized trajectory failed')

        self.active_goal_handles=[]

    def recent_all_fingers(self):
        with self.lock:
            recent = [c for c in self.contacts[-30:] if c['phase'] == 'close']
            return any(all(j[:-6] in ' '.join(c['fingers']) for j in FINGERS) for c in recent)

    def run(self):
        path = json.loads(open(os.environ['C43_TRANSPORT_JSON']).read())
        approach = json.loads(open(os.environ['C43_CANDIDATE_JSON']).read())
        if not path['path_collision_free'] or not approach['path_collision_free']:
            raise RuntimeError('path did not pass collision screening')
        for side in ('left', 'right'):
            if not self.action_clients[side+'_arm_controller'].wait_for_server(timeout_sec=45):
                raise RuntimeError(side+' arm controller unavailable')
        self.approach(path,approach)
        self.grasp()
        if os.environ.get('C43_DUAL_CLOSE_ONLY') == '1':
            self.phase = 'hold_lift'; self.wait_stage('hold_lift',5); self.snapshot('close_only_end'); return
        self.carry(path)

    def approach(self,path,approach):
        self.wait_stage('startup',1);self.spin(float(os.environ.get('C43_GUI_START_DELAY_S','0')))
        self.clamp(-2);self.wait_stage('open',2)
        if any(abs(self.q.get(j,0)) < .03 for j in FINGERS):
            raise RuntimeError('four fingers did not open')
        self.phase = 'homing'
        # Use the newly checked 60/50 mm path at the same pregrasp branch.
        pre = path['pre_q12']
        self.trajectory('right_arm_controller', RIGHT_JOINTS, [pre[6:]], self.durations.get('pregrasp',8))
        self.trajectory('left_arm_controller', LEFT_JOINTS, [pre[:6]], self.durations.get('pregrasp',8))
        self.trajectory('left_arm_controller', LEFT_JOINTS, approach['left_descent'], self.durations.get('descent',6))
        self.trajectory('right_arm_controller', RIGHT_JOINTS, approach['right_descent'], self.durations.get('descent',6))
        self.phase = 'preclose'; self.wait_stage('preclose',2); self.snapshot('preclose')

    def grasp(self):
        self.phase = 'close'
        start = time.monotonic()
        start_sim=self.sim
        def close_elapsed():return self.sim-start_sim if os.environ.get('C43_WAIT_CLOCK')=='sim' else time.monotonic()-start
        while not self.recent_all_fingers() and close_elapsed() < self.task.get('close_timeout_sim_s',30.):
            forces = []
            for k, joint in enumerate(FINGERS):
                velocity = self.v.get(joint, float('nan'))
                pose = self.pose(joint[:-6])
                if not math.isfinite(velocity) or pose is None:
                    raise RuntimeError('missing finger pose/velocity')
                gravity = float(np.dot(quaternion_matrix(pose[3:]).T@np.array([0,0,-.008826*9.8]), AXIS))
                sign = -1 if k%2 == 0 else 1
                forces.append(float(np.clip(.008826*10*(sign*.0007-velocity)-gravity, -CLAMP, CLAMP)))
            self.drive(forces); self.spin(.02)
        if not self.recent_all_fingers():
            raise RuntimeError('four-finger contact not established within 30 s')
        self.event('all_four_fingers_before_force_ramp')
        for fraction in (.25,.5,.75,1.):
            self.clamp(CLAMP*fraction); self.wait_stage('ramp_step',1)
        self.phase = 'settle'; self.wait_stage('settle',3); self.snapshot('prelift')
        sample = np.array(self.pose('c43_sample')[:3])
        self.carry_offsets = [sample-np.array(self.pose(s+'_wrist3_link')[:3]) for s in ('left','right')]
    def carry(self,path):
        for stage in path['stages']:
            name = stage['name']
            if name == 'retract':
                self.phase = 'place_settle'; self.wait_stage('place_settle',3); self.snapshot('before_release')
                sample = self.pose('c43_sample')
                if np.linalg.norm(np.array(sample[:3])-np.array(path['destination_xyz'])) > self.task.get('before_release_tolerance_m',.015):
                    raise RuntimeError('sample not within destination tolerance before release')
                with self.lock:
                    supported = any('c43_destination' in json.dumps(c.get('sample_collision_pairs', [])) for c in self.contacts[-40:])
                if not supported:
                    raise RuntimeError('destination support contact missing before release')
                self.carry_offsets = None
                self.phase = 'release'; self.clamp(-1); self.wait_stage('release',2); self.snapshot('released')
            self.phase = name
            self.synchronized(stage['q12'], self.durations.get(name,8 if name in ('transfer','home_left','home_right') else 6))
            self.snapshot(name+'_end')
            if name == 'lift':
                self.phase = 'hold_lift'; self.wait_stage('hold_lift',5); self.snapshot('hold_lift_end')
        self.phase = 'final'; self.wait_stage('final',5); self.snapshot('final')
        sample = self.pose('c43_sample')
        if np.linalg.norm(np.array(sample[:3])-np.array(path['destination_xyz'])) > self.task.get('final_destination_tolerance_m',.01):
            raise RuntimeError('final sample outside 10 mm destination tolerance')
        with self.lock:
            if any(c['fingers'] for c in self.contacts[-60:]):
                raise RuntimeError('finger/sample contact persists after retreat')
            if not any('c43_destination' in json.dumps(c.get('sample_collision_pairs', [])) for c in self.contacts[-40:]):
                raise RuntimeError('final destination support contact missing')
        self.event('physical_transport_complete', destination=sample)

    def finish(self, error):
        try:
            self.drive([0,0,0,0], log=True)
        except Exception as exc:
            self.event('zero_drive_failed', detail=repr(exc))
        self.publisher.stdin.close()
        try:
            self.publisher.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.publisher.terminate(); self.publisher.wait()
        for stream in self.ft:
            stream.close()
        self.native.close(); self.save(error)
        with (OUT/'four_finger_ft.jsonl').open('w') as f:
            for row in self.ft_rows:
                f.write(json.dumps(row)+'\n')


def main():
    rclpy.init(); node=Transport(); error=None
    try:
        node.run()
    except Exception as exc:
        error=repr(exc); node.event('error', detail=error)
    finally:
        node.finish(error); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return int(error is not None)


if __name__ == '__main__':
    raise SystemExit(main())
