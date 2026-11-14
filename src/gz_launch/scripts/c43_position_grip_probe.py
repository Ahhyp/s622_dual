#!/usr/bin/env python3
"""Sim-only position grip A/B. No executor, weld, QP, or force commands.

Run with run_c43_position_ab.sh; writes raw Contacts messages, message-level
finger pairs, event-time telemetry and action outcomes into an isolated run.
"""
import csv
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from c43_homing import LEFT_JOINTS, RIGHT_JOINTS, PRE_LEFT, PRE_RIGHT, DESCENT_LEFT, DESCENT_RIGHT

OUT = Path(os.environ['C43_AB_OUT'])
WIDTH = float(os.environ['C43_AB_WIDTH'])
MASS = float(os.environ.get('C43_AB_MASS', '.35'))
SINGLE = os.environ.get('C43_AB_SINGLE', '0') == '1'
MODE = os.environ['C43_AB_MODE']
TARGET = float(os.environ['C43_AB_TARGET'])
CLOSE_S = max(8.0, (.0305-TARGET)/.0007)
TOPIC = '/world/c43_s4_world/model/c43_sample/link/base_link/sensor/sample_contact/contact'
FINGERS = [f'{side}_finger{i}_joint' for side in (('left',) if SINGLE else ('left','right')) for i in (1, 2)]
JOINTS = LEFT_JOINTS + ([] if SINGLE else RIGHT_JOINTS) + FINGERS


class Probe(Node):
    def __init__(self):
        super().__init__('c43_position_grip_probe')
        self.t0 = time.monotonic()
        self.phase = 'startup'
        self.sim = None
        self.q = {}
        self.poses = {}
        self.contacts = []
        self.events = []
        self.rows = []
        self.lock = threading.Lock()
        self.stop_reader = False
        self.raw = (OUT / 'contacts_raw.txt').open('w')
        self.contact_file = (OUT / 'contacts.jsonl').open('w')
        # Live executor guards need the latest state, not a buffered replay.
        live=os.environ.get('C43_ARM_BACKEND')=='qp-executor'
        self.create_subscription(JointState, '/joint_states', self.on_js, 1 if live else 100)
        self.create_subscription(Clock, '/world/c43_s4_world/clock', self.on_clock, 1 if live else 50)
        self.create_subscription(TFMessage, '/c43/weld_pose', self.on_pose, 1 if live else 100)
        self.action_clients = {name: ActionClient(self, FollowJointTrajectory,
            f'/{name}/follow_joint_trajectory') for name in
            (('left_arm_controller', 'left_hand_controller') if SINGLE else
             ('left_arm_controller', 'right_arm_controller', 'left_hand_controller', 'right_hand_controller'))}
        self.reader = subprocess.Popen(['ign', 'topic', '-e', '-t', TOPIC],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read_contacts, daemon=True)
        self.thread.start()

    def event(self, name, **detail):
        rec = dict(event=name, phase=self.phase, mono=time.monotonic()-self.t0,
                   sim_last=self.sim, detail=detail)
        self.events.append(rec)
        print(json.dumps(rec), flush=True)

    def on_clock(self, msg):
        self.sim = msg.clock.sec + 1e-9*msg.clock.nanosec

    def on_pose(self, msg):
        for t in msg.transforms:
            p, q = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def pose(self, name):
        for k, v in self.poses.items():
            if k == name or k.endswith('::'+name):
                return list(v)
        return None

    def on_js(self, msg):
        self.q.update(zip(msg.name, msg.position))
        if all(j in self.q for j in JOINTS):
            self.rows.append(dict(mono=time.monotonic()-self.t0, sim=self.sim,
                phase=self.phase, q={j:self.q[j] for j in JOINTS},
                sample=self.pose('c43_sample'), wrist=self.pose('left_wrist3_link')))

    def read_contacts(self):
        # `ign topic -e` separates complete top-level protobuf messages by a
        # blank line. Parse contact blocks only after the WHOLE message arrives.
        lines = []
        for line in self.reader.stdout:
            self.raw.write(line)
            if line.strip():
                lines.append(line)
            elif lines:
                self.process_message(''.join(lines))
                lines = []
        if lines:
            self.process_message(''.join(lines))

    def process_message(self, text):
        stamp = re.search(r'stamp\s*\{([^{}]*)\}', text)
        s = stamp.group(1) if stamp else ''
        sec = re.search(r'(?<!n)sec:\s*(\d+)', s)
        nsec = re.search(r'nsec:\s*(\d+)', s)
        sim = (int(sec.group(1)) if sec else 0) + 1e-9*(int(nsec.group(1)) if nsec else 0)
        pairs = []
        sample_collision_pairs = []
        sample_contact_blocks = []
        robot_links = set()
        for match in re.finditer(r'\bcontact\s*\{', text):
            pos = match.end(); depth = 1; end = pos
            while end < len(text) and depth:
                depth += (text[end] == '{') - (text[end] == '}'); end += 1
            block = text[pos:end-1]
            names = re.findall(r'name:\s*"([^"]+)"', block)
            if any('c43_sample' in n for n in names):
                sample_collision_pairs.append(names)
                sample_contact_blocks.append(dict(names=names,raw_block=block))
                robot_links.update(n for n in names if 's622_dual_arm::' in n)
                fingers = [n for n in names if 'left_finger' in n or 'right_finger' in n]
                for f in fingers:
                    pairs.append(dict(finger=f, names=names, raw_block=block))
        rec = dict(mono=time.monotonic()-self.t0, sim=sim, phase=self.phase,
                   fingers=sorted({x['finger'] for x in pairs}), pairs=pairs,
                   robot_links=sorted(robot_links), sample_collision_pairs=sample_collision_pairs, sample_contact_blocks=sample_contact_blocks)
        with self.lock:
            self.contacts.append(rec)
            self.contact_file.write(json.dumps(rec)+'\n')

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_future(self, future, timeout):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        if not future.done():
            raise TimeoutError('action future timed out')
        return future.result()

    def trajectory(self, controller, joints, points, duration, cancel_first=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        for i, q in enumerate(points):
            p = JointTrajectoryPoint(); p.positions = [float(value) for value in q]
            t = duration*(i+1)/len(points)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t-int(t))*1e9)
            goal.trajectory.points.append(p)
        handle = self.wait_future(self.action_clients[controller].send_goal_async(goal), 10)
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{controller}: goal rejected')
        self.active_goal_handle=handle
        self.event('goal_accepted', controller=controller, target=points[-1], duration=duration)
        result = handle.get_result_async()
        start = time.monotonic(); cancelled = False
        start_sim=self.sim
        def elapsed():
            return self.sim-start_sim if os.environ.get('C43_WAIT_CLOCK')=='sim' else time.monotonic()-start
        while not result.done() and elapsed() < duration+15:
            rclpy.spin_once(self, timeout_sec=0.005)
            if getattr(self,'stop_requested',False):
                self.cancel_for_stop(handle)
                raise RuntimeError('executor STOP: cancel active JTC goal')
            if os.environ.get('C43_WAIT_CLOCK')=='sim' and time.monotonic()-self.last_clock_progress>self.task.get('clock_stall_watchdog_wall_s',10.):
                raise RuntimeError('simulation clock stalled during trajectory')
            if cancel_first and not cancelled:
                with self.lock:
                    touched = any(c['fingers'] and c['phase']=='close' for c in self.contacts)
                if touched:
                    self.event('cancel_send', q=[self.q[j] for j in FINGERS[:2]])
                    reply = self.wait_future(handle.cancel_goal_async(), 5)
                    self.event('cancel_confirm', accepted=bool(reply.goals_canceling))
                    cancelled = True
        response = self.wait_future(result, 5)
        self.active_goal_handle=None
        self.event('action_result', controller=controller, status=response.status,
                   error_code=response.result.error_code, error_string=response.result.error_string)
        if response.status != 4 and not (cancelled and response.status == 5):
            raise RuntimeError(f'action did not succeed: {response.status}')
        return cancelled

    def run(self):
        for name, cli in self.action_clients.items():
            if not cli.wait_for_server(timeout_sec=45):
                raise RuntimeError(f'{name} unavailable')
        info=subprocess.run(['ign','topic','-i','-t',TOPIC],capture_output=True,text=True,timeout=8)
        (OUT/'contact_topic_info.txt').write_text(info.stdout+info.stderr)
        if 'Publishers [' not in info.stdout:
            raise RuntimeError('native contact topic has no publisher')
        self.spin(1)
        self.event('ready')
        self.phase = 'homing'
        self.trajectory('left_hand_controller', FINGERS[:2], [[.0305,-.0305]], 1.5)
        if not SINGLE:
            self.trajectory('right_hand_controller', FINGERS[2:], [[.0305,-.0305]], 1.5)
            self.trajectory('right_arm_controller', RIGHT_JOINTS, [PRE_RIGHT], 8)
        self.trajectory('left_arm_controller', LEFT_JOINTS, [PRE_LEFT], 8)
        self.trajectory('left_arm_controller', LEFT_JOINTS, DESCENT_LEFT, 6)
        if not SINGLE:
            self.trajectory('right_arm_controller', RIGHT_JOINTS, DESCENT_RIGHT, 6)
        self.phase = 'preclose'; self.spin(2)
        initial = self.pose('c43_sample')
        if initial is None or self.pose('left_wrist3_link') is None:
            raise RuntimeError('sample/wrist pose not received')
        self.event('homing_complete', sample=initial)
        with self.lock:
            home_touch = sum(bool(c['fingers']) for c in self.contacts if c['phase']=='homing')
            if not self.contacts or self.contacts[-1]['mono'] < time.monotonic()-self.t0-2:
                raise RuntimeError('contact acquisition missing or stale')
        if home_touch:
            raise RuntimeError(f'homing touched sample: {home_touch} messages')
        self.phase = 'close'
        cancelled = self.trajectory('left_hand_controller', FINGERS[:2], [[TARGET,-TARGET]],
                                    CLOSE_S, cancel_first=(MODE=='cancel_hold'))
        if MODE == 'cancel_hold':
            self.spin(.35)
            hold = [self.q[j] for j in FINGERS[:2]]
            self.phase = 'hold_command'
            self.trajectory('left_hand_controller', FINGERS[:2], [hold], .5)
            self.event('hold_measured_position', target=hold, cancelled=cancelled)
        self.phase = 'settle'; self.spin(3)
        self.event('prelift', sample=self.pose('c43_sample'), q=[self.q[j] for j in FINGERS[:2]])
        self.phase = 'lift'
        # Reverse the validated descent table: same branch, +50 mm world Z.
        self.trajectory('left_arm_controller', LEFT_JOINTS, list(reversed(DESCENT_LEFT))[1:], 8)
        self.phase = 'hold_lift'; self.spin(5)
        self.event('lift_end', sample=self.pose('c43_sample'), wrist=self.pose('left_wrist3_link'))

    def save(self, error=None):
        self.reader.terminate()
        try: self.reader.wait(timeout=3)
        except subprocess.TimeoutExpired: self.reader.kill(); self.reader.wait()
        self.thread.join(timeout=3)
        self.raw.close(); self.contact_file.close()
        with (OUT/'telemetry.csv').open('w') as f:
            w=csv.writer(f); w.writerow(['mono','sim','phase']+JOINTS+
                ['sample_'+k for k in ('x','y','z','qx','qy','qz','qw')]+
                ['wrist_'+k for k in ('x','y','z','qx','qy','qz','qw')])
            for r in self.rows:
                w.writerow([r['mono'],r['sim'],r['phase']]+[r['q'][j] for j in JOINTS]+
                           (r['sample'] or [None]*7)+(r['wrist'] or [None]*7))
        summary=dict(width_m=WIDTH,height_m=.05,mass_kg=MASS,mode=MODE,target_m=TARGET,
            single_robot=SINGLE,
            planned_close_duration_s=CLOSE_S,
            error=error,rows=len(self.rows),contact_messages=len(self.contacts),events=self.events)
        before=[r for r in self.rows if r['phase']=='settle' and r['sample'] and r['wrist']]
        after=[r for r in self.rows if r['phase']=='hold_lift' and r['sample'] and r['wrist']]
        if before and after:
            a,b=before[-1],after[-1]
            dz=b['wrist'][2]-a['wrist'][2]; oz=b['sample'][2]-a['sample'][2]
            summary.update(wrist_lift_mm=dz*1000,sample_lift_mm=oz*1000,
                follow_ratio=oz/dz if abs(dz)>.001 else None,
                vertical_slip_mm=(dz-oz)*1000,
                final_fingers_m={j:b['q'][j] for j in FINGERS},
                sample_prelift_m=a['sample'],sample_final_m=b['sample'])
        with self.lock:
            summary['contacts_by_phase']={}
            for phase in sorted({c['phase'] for c in self.contacts}):
                cs=[c for c in self.contacts if c['phase']==phase]
                counts={}
                both=0
                for c in cs:
                    for finger in c['fingers']: counts[finger]=counts.get(finger,0)+1
                    names=' '.join(c['fingers'])
                    both+=('left_finger1' in names and 'left_finger2' in names)
                summary['contacts_by_phase'][phase]=dict(messages=len(cs),finger_messages=counts,
                                                         both_left_same_top_level_message=both)
        (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2),flush=True)


def main():
    rclpy.init(); probe=Probe(); error=None
    try: probe.run()
    except Exception as exc:
        error=repr(exc); probe.event('error',message=error)
    finally:
        probe.save(error); probe.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 1 if error else 0

if __name__=='__main__':
    raise SystemExit(main())
