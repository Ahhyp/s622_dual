#!/usr/bin/env python3
"""Physical actuator/observer adapter; task progression belongs to C++ executor."""
import json
import gc
import math
import os
import time
import signal
import subprocess
import shlex
import threading
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from c43_contact_safety import contact_violation
import numpy as np
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers
from dual_arm_transport_executor.msg import ContactCommand,ContactEvidence
from c43_qp_contact_smoke import QpSmoke
from c43_position_grip_probe import OUT,FINGERS,JOINTS


class ContactPlant(QpSmoke):
    def __init__(self):
        super().__init__()
        self.command_queue=[];self.current_command=None;self.evidence_seq=0
        self.step_done=False;self.step_ok=True;self.step_detail='';self.ready=False
        self.owns_jtc=False;self.owns_qp=False;self.hands_jtc=False
        self.stop_requested=False;self.stop_futures=[];self.stop_goal_ids=set();self.stop_logged=False;self.step_running=False;self.terminal=None;self.executor_state='IDLE'
        self.last_evidence_sim=-1.
        self.w9_mode=os.environ.get('C43_EXECUTOR_INJECT','')
        self.environment_safety_enabled=os.environ.get('C43_ENABLE_CONTACT_SAFETY')=='1' or self.w9_mode=='unexpected-contact'
        self.following_safety_enabled=os.environ.get('C43_ENABLE_CONTACT_SAFETY')=='1' or self.w9_mode=='right-wrist-load'
        self.contact_safety_enabled=self.environment_safety_enabled or self.following_safety_enabled
        self.native_pose_pub=self.create_publisher(PoseStamped,'/dual_arm_qp_controller/native_object_pose',1) if os.environ.get('C43_ENABLE_FINITE_GUARD')=='1' else None
        self.contact_phase='IDLE'
        self.source_departed=False;self.safety_failure='';self.safety_detail='';self.last_status_sim=-1
        self.wrench_processes=[]
        self.supports=[]
        support=ET.parse(Path(get_package_share_directory('gz_launch'))/'models/c43_support/model.sdf').getroot().find('model')
        radius=float(support.findtext('link/collision/geometry/cylinder/radius'));height=float(support.findtext('link/collision/geometry/cylinder/length'))
        link_pose=[float(x) for x in support.findtext('link/pose').split()]
        for include in ET.parse(OUT/'world.sdf').getroot().findall('world/include'):
            name=include.findtext('name')
            if name in ('c43_support','c43_destination'):
                pose=[float(x) for x in include.findtext('pose').split()]
                if any(abs(x)>1e-12 for x in pose[3:5]):raise ValueError('tilted support not supported')
                self.supports.append(dict(name=name,center=pose[:3],radius=radius,top_z=pose[2]+link_pose[2]+height/2))
        self.w9_trigger=None;self.w9_frozen_stamp=None
        self.hold_test_stage='idle';self.hold_test_future=None;self.hold_test_since=0
        self.hold_test_clients={k:self.create_client(Trigger,'/c43_s4_transport_executor/'+k) for k in ('hold','resume')}
        self.paused_executor_pid=None;self.executor_resume_timer=None
        self.create_subscription(ContactCommand,'/c43_s4_transport_executor/contact_command',self.on_command,10)
        self.create_subscription(String,'/c43_s4_transport_executor/task_state',self.on_executor_state,10)
        self.create_subscription(String,'/c43_s4_transport_executor/task_result',self.on_result,10)
        self.evidence_pub=self.create_publisher(ContactEvidence,'/c43_s4_transport_executor/contact_evidence',1)
        # Contact evidence must not depend on JointState publication cadence.
        self.evidence_timer=self.create_timer(.02,self.publish_evidence)

    def restore_executor(self):
        if self.paused_executor_pid is None:return
        try:
            os.kill(self.paused_executor_pid,signal.SIGCONT)
            self.event('w9_executor_continued',pid=self.paused_executor_pid)
        except ProcessLookupError:pass
        self.paused_executor_pid=None

    def trigger_w9(self,facts):
        if self.w9_mode=='hold-resume':self.pump_hold_test()
        if self.w9_trigger is not None:return
        modes=('carry-contact-loss','carry-contact-loss-regrip','stale-contact','evidence-gap','executor-pause','hold-resume','unexpected-contact','right-wrist-load','finite-clearance')
        if self.w9_mode not in modes or self.status is None:return
        if not (self.status.state=='RUNNING' and self.status.phase_name=='settle' and
                self.status.phase_elapsed_s>=1.0 and facts['four']):return
        self.w9_trigger=dict(mode=self.w9_mode,sim=self.sim,mono=time.monotonic()-self.t0,
            t_traj=self.status.t_traj,phase=self.status.phase_name,raw_four=facts['four'],sample=self.pose('c43_sample'))
        self.event('w9_injection',**self.w9_trigger)
        if self.w9_mode=='hold-resume':
            self.hold_test_future=self.hold_test_clients['resume'].call_async(Trigger.Request());self.hold_test_stage='reject_early'
        if self.w9_mode in ('carry-contact-loss','carry-contact-loss-regrip'):
            # Actually open the left gripper; right native drive remains 5 N/finger.
            self.drive([2,-2,-5,5],log=True)
        elif self.w9_mode=='right-wrist-load':self.right_wrist_wrench(True)
        elif self.w9_mode=='stale-contact':self.w9_frozen_stamp=facts['stamp']
        elif self.w9_mode=='executor-pause':
            target=str(OUT/'executor.yaml');matches=[]
            for row in subprocess.check_output(['ps','-eo','pid=,args='],text=True).splitlines():
                parts=row.strip().split(None,1)
                if len(parts)!=2:continue
                argv=shlex.split(parts[1])
                if argv and argv[0].endswith('/c43_s4_executor') and target in argv:
                    matches.append(int(parts[0]))
            if len(matches)!=1:raise RuntimeError('W9 executor PID is not unique: '+repr(matches))
            self.paused_executor_pid=matches[0]
            os.kill(self.paused_executor_pid,signal.SIGSTOP)
            self.event('w9_executor_paused',pid=self.paused_executor_pid)
            self.executor_resume_timer=threading.Timer(2.0,self.restore_executor)
            self.executor_resume_timer.daemon=True;self.executor_resume_timer.start()

    def right_wrist_wrench(self,apply):
        entity='name: "s622_dual_arm::right_wrist3_link" type: LINK'
        topic='/world/c43_s4_world/wrench/'+('persistent' if apply else 'clear')
        kind='ignition.msgs.EntityWrench' if apply else 'ignition.msgs.Entity'
        message='entity { '+entity+' } wrench { torque { z: 50 } }' if apply else entity
        self.wrench_processes.append(subprocess.Popen(['ign','topic','-t',topic,'-m',kind,'-p',message],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
        self.event('w9_right_wrist_load_command',apply=apply,torque_z_Nm=50 if apply else 0)

    def physical_safety(self,facts,sample):
        if not self.contact_safety_enabled or self.stop_requested:return
        if self.executor_state in ('LIFT','TRANSPORT','LOWER') and not facts['source']:self.source_departed=True
        with self.lock:record=self.contacts[-1] if self.contacts else None
        if record is None:return
        try:detail=contact_violation(record,sample,self.contact_phase,not self.source_departed,self.supports) if self.environment_safety_enabled else ''
        except (ValueError,KeyError) as exc:detail='invalid contact safety evidence: '+str(exc)
        code='object_env_contact' if detail else ''
        if not code and self.following_safety_enabled and self.owns_qp and self.status is not None and 0<=self.sim-self.last_status_sim<=.1:
            pos=self.status.anchor_pos_gap_m;rot=self.status.anchor_rot_gap_rad
            if not math.isfinite(pos) or not math.isfinite(rot) or pos>.002 or rot>math.radians(.5):
                code='follower_divergence';detail='experimental plan bounds: 2mm / 0.5deg'
        if code and not self.safety_failure:
            self.safety_failure=code;self.safety_detail=detail
            self.event('physical_safety_violation',code=code,detail=detail,sample=sample,
                contact_stamp=record['sim'],sample_collision_pairs=record['sample_collision_pairs'],
                anchor_pos_gap_m=self.status.anchor_pos_gap_m if self.status else None,
                anchor_rot_gap_rad=self.status.anchor_rot_gap_rad if self.status else None)

    def pump_hold_test(self):
        if self.hold_test_stage=='idle':return
        future=self.hold_test_future
        if future is not None and future.done():
            response=future.result();self.event('w9_hold_service',stage=self.hold_test_stage,success=response.success,message=response.message)
            self.hold_test_future=None
            if self.hold_test_stage=='reject_early':
                if response.success:raise RuntimeError('resume accepted before explicit HOLD')
                self.hold_test_future=self.hold_test_clients['hold'].call_async(Trigger.Request());self.hold_test_stage='hold_request'
            elif self.hold_test_stage=='hold_request':
                if not response.success:raise RuntimeError('explicit hold rejected: '+response.message)
                self.hold_test_stage='wait_hold'
            elif self.hold_test_stage=='resume_request':
                if not response.success:raise RuntimeError('explicit resume rejected: '+response.message)
                self.hold_test_stage='wait_running'
        if self.hold_test_stage=='wait_hold' and self.executor_state=='HOLD' and self.status.state=='HOLD':
            self.hold_test_since=self.sim;self.hold_test_stage='dwell';self.event('w9_hold_dwell_begin')
        if self.hold_test_stage=='dwell' and self.sim-self.hold_test_since>=1.0:
            self.event('w9_hold_dwell_end');self.hold_test_future=self.hold_test_clients['resume'].call_async(Trigger.Request());self.hold_test_stage='resume_request'
        if self.hold_test_stage=='wait_running' and self.status.state=='RUNNING' and self.executor_state!='HOLD':
            self.hold_test_stage='done';self.event('w9_hold_resume_running')

    def capture_fault_stop(self):
        modes=('destination-absent','carry-contact-loss','carry-contact-loss-regrip','stale-contact','evidence-gap','executor-pause','unexpected-contact','right-wrist-load','finite-clearance')
        if self.w9_mode not in modes:return
        self.phase='fault_observe'
        start=self.sim;wall=time.monotonic();i=len(self.qp_rows)
        self.event('w9_stop_observation_begin',terminal=self.terminal)
        while self.sim-start<1.0:
            rclpy.spin_once(self,timeout_sec=.01)
            if time.monotonic()-wall>15:raise RuntimeError('W9 stopped observation clock watchdog')
        probes={}
        for endpoint in ('reset','start'):
            client=self.create_client(Trigger,'/c43_s4_transport_executor/'+endpoint)
            if not client.wait_for_service(timeout_sec=2):
                probes[endpoint]=dict(available=False);continue
            future=client.call_async(Trigger.Request());deadline=time.monotonic()+3
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.01)
            if future.done():
                response=future.result();probes[endpoint]=dict(available=True,success=response.success,message=response.message)
            else:probes[endpoint]=dict(available=True,timed_out=True)
        result=dict(mode=self.w9_mode,trigger=self.w9_trigger,terminal=self.terminal,
            observe_start_sim=start,observe_end_sim=self.sim,status_rows=self.qp_rows[i:],
            recovery_probes=probes,sample=self.pose('c43_sample'),fingers={j:self.q.get(j) for j in FINGERS},
            no_positive_resume_claim=True)
        (OUT/'w9_fault_capture.json').write_text(json.dumps(result,indent=2)+'\n')
        self.event('w9_stop_observation_end',terminal=self.terminal,recovery_probes=probes)

    def on_executor_state(self,msg):
        if msg.data!=self.executor_state:self.event('executor_state',state=msg.data)
        self.executor_state=msg.data
        if msg.data not in ('HOLD','FAULT','ABORT'):self.contact_phase=msg.data

    def on_result(self,msg):
        self.terminal=msg.data

    def on_command(self,msg):
        previous=self.current_command
        if previous is not None and msg.task_id==previous.task_id and msg.command_id<=previous.command_id:return
        if previous is not None and msg.task_id!=previous.task_id:
            self.event('rejected_foreign_task_command');return
        self.current_command=msg;self.step_done=False;self.step_ok=True;self.step_detail=''
        self.event('executor_command',task_id=msg.task_id,command_id=msg.command_id,command_phase=msg.phase)
        if msg.phase=='STOP':
            self.stop_requested=True
            self.phase='fault_stop'
            if self.w9_mode=='right-wrist-load':self.right_wrist_wrench(False)
            if self.owns_qp and self.w9_mode!='carry-contact-loss':
                # Reassert the existing 5 N command, never increase the force.
                # The old loss case models a persistent open-actuator fault;
                # the new regrip case tests a recoverable opening transient.
                self.clamp(5)
                self.event('grip_hold_reasserted',drive_N=5)
            handles=list(getattr(self,'active_goal_handles',[]))
            handle=getattr(self,'active_goal_handle',None)
            if handle:handles.append(handle)
            for handle in handles:
                self.cancel_for_stop(handle)
            return
        self.command_queue.append(msg)

    def cancel_for_stop(self,handle):
        key=bytes(handle.goal_id.uuid).hex()
        if key in self.stop_goal_ids:return
        self.stop_goal_ids.add(key);handle.cancel_goal_async()
        self.stop_futures.append(handle.get_result_async())

    def on_status(self,msg):
        super().on_status(msg)
        self.last_status_sim=self.sim if self.sim is not None else -1
        if msg.active and not self.stop_requested:
            mapped={'lift':'lift','settle':'hold_lift','transport':'transfer','lower':'place','settle_post':'place'}
            if msg.phase_name in mapped:self.phase=mapped[msg.phase_name]

    def spin(self,seconds):
        super().spin(seconds)
        if getattr(self,'stop_requested',False):raise RuntimeError('executor STOP latched')

    def switch(self,to_qp):
        super().switch(to_qp)
        self.owns_qp=to_qp;self.owns_jtc=not to_qp

    @staticmethod
    def stamp(target,value):
        if value<0:target.sec=-1;target.nanosec=0;return
        ns=int(round(value*1e9));target.sec=ns//1000000000;target.nanosec=ns%1000000000

    def contact_facts(self):
        with self.lock:recent=list(self.contacts[-5:])
        if not recent:return dict(stamp=-1,four=False,source=False,destination=False,clear=False)
        pair=lambda c,key:key in json.dumps(c.get('sample_collision_pairs',[]))
        return dict(stamp=recent[-1]['sim'],
            four=len(recent)==5 and all(all(j[:-6] in ' '.join(c['fingers']) for j in FINGERS) for c in recent),
            source=len(recent)==5 and all(pair(c,'c43_support') for c in recent),
            destination=len(recent)==5 and all(pair(c,'c43_destination') for c in recent),
            clear=all(not c['robot_links'] for c in recent))

    def publish_evidence(self):
        if self.sim is None:return
        f=self.contact_facts();sample=self.pose('c43_sample')
        if sample is None:return
        native_snapshot=self.native.latest_sample
        if self.native_pose_pub is not None and native_snapshot is not None:
            native_stamp,native_sample=native_snapshot
            native=PoseStamped();native.header.frame_id='world';self.stamp(native.header.stamp,native_stamp)
            native.pose.position.x,native.pose.position.y,native.pose.position.z=[float(x) for x in native_sample[:3]]
            native.pose.orientation.x,native.pose.orientation.y,native.pose.orientation.z,native.pose.orientation.w=[float(x) for x in native_sample[3:]]
            self.native_pose_pub.publish(native)
        self.trigger_w9(f)
        self.physical_safety(f,sample)
        if self.w9_mode=='evidence-gap' and self.w9_trigger is not None and not self.stop_requested:return
        if self.stop_requested and not self.step_running and all(f.done() for f in self.stop_futures):
            self.step_done=True;self.step_ok=True;self.step_detail='STOP: no outstanding arm action'
            if not self.stop_logged:
                self.stop_logged=True;self.event('plant_stop_confirmed',goals=len(self.stop_goal_ids),
                    terminal_statuses=[f.result().status for f in self.stop_futures])
        m=ContactEvidence();self.evidence_seq+=1;m.evidence_seq=self.evidence_seq
        self.stamp(m.stamp,max(self.sim,f['stamp']));self.stamp(m.contact_stamp,f['stamp'])
        if self.w9_frozen_stamp is not None:self.stamp(m.contact_stamp,self.w9_frozen_stamp)
        if self.current_command:
            m.task_id=self.current_command.task_id;m.command_id=self.current_command.command_id;m.command_phase=self.current_command.phase
        else:m.command_phase='READY'
        m.ready=self.ready and all(j in self.q and math.isfinite(self.q[j]) for j in JOINTS)
        m.step_done=self.step_done;m.ok=self.step_ok;m.detail=self.step_detail
        m.all_four=f['four'] and os.environ.get('C43_EXECUTOR_INJECT')!='missing-grasp'
        m.source_support=f['source'];m.destination_support=f['destination'] and os.environ.get('C43_EXECUTOR_INJECT')!='missing-support'
        m.robot_clear=f['clear'];m.fingers_open=all(abs(self.q.get(j,0))>.03 for j in FINGERS)
        m.arms_jtc_active=self.owns_jtc;m.qp_active=self.owns_qp;m.hands_jtc_active=self.hands_jtc
        m.sample_position=[float(x) for x in sample[:3]];m.sample_orientation=[float(x) for x in sample[3:]]
        m.safety_failure_code=self.safety_failure;m.safety_detail=self.safety_detail
        self.evidence_pub.publish(m)

    def release(self):
        if not self.contact_facts()['destination']:raise RuntimeError('plant independently refuses release without destination contact')
        self.phase='place';self.spin(2);self.snapshot('supported_hold_settled')
        if not self.contact_facts()['destination']:raise RuntimeError('destination support lost during stopped settling')
        result=self.service(self.reset_client,Trigger.Request())
        if not result.success:raise RuntimeError('QP reset refused')
        self.spin(.2);self.switch(False);self.carry_offsets=None
        self.phase='place_settle';self.wait_stage('place_settle',3);self.snapshot('before_release')
        if not self.contact_facts()['destination']:raise RuntimeError('destination support lost before opening')
        self.phase='release';self.clamp(-1);self.wait_stage('release',2);self.snapshot('released')
        if any(abs(self.q.get(j,0))<.03 for j in FINGERS):raise RuntimeError('finger open feedback missing')

    def retreat(self,path):
        for s in path['stages']:
            if s['name'] not in ('retract','return_over_source','return_pre','home_left','home_right'):continue
            self.phase=s['name'];self.synchronized(s['q12'],self.durations[s['name']])
        self.phase='final';self.wait_stage('final',5);self.snapshot('final')
        if not self.contact_facts()['clear']:raise RuntimeError('robot contact persists after retreat')
        if not self.contact_facts()['destination']:raise RuntimeError('sample not supported at destination')

    def run(self):
        path=json.loads((OUT/'transport.json').read_text());approach=json.loads((OUT/'approach.json').read_text())
        if not path['path_collision_free'] or not approach['path_collision_free']:raise RuntimeError('candidate precheck flag missing')
        for side in ('left','right'):
            if not self.action_clients[side+'_arm_controller'].wait_for_server(timeout_sec=45):raise RuntimeError(side+' arm unavailable')
        # An action server can appear before its controller is activated.
        # Retry the measured ownership precheck instead of caching that race
        # as ready=False forever, which prevented two concurrent W9 runs starting.
        deadline=time.monotonic()+45;attempt=0
        while True:
            controllers=self.service(self.list_client,ListControllers.Request()).controller
            active={c.name for c in controllers if c.state=='active'}
            self.owns_jtc={'left_arm_controller','right_arm_controller'}<=active
            self.owns_qp='dual_arm_qp_controller' in active
            self.hands_jtc=bool({'left_hand_controller','right_hand_controller'}&active)
            self.ready=self.owns_jtc and not self.owns_qp and not self.hands_jtc
            attempt+=1
            if self.ready:
                self.event('initial_ownership_ready',attempts=attempt,active=sorted(active));break
            if attempt==1:self.event('initial_ownership_pending',active=sorted(active))
            if time.monotonic()>deadline:raise RuntimeError('initial controller ownership did not become ready')
            self.spin(.05)
        started=time.monotonic()
        while self.terminal is None:
            rclpy.spin_once(self,timeout_sec=.01)
            if time.monotonic()-started>450:raise RuntimeError('executor result wall watchdog expired')
            if not self.command_queue or self.stop_requested:continue
            cmd=self.command_queue.pop(0);self.step_running=True
            try:
                if cmd.phase=='APPROACH':self.approach(path,approach)
                elif cmd.phase=='GRASP':
                    self.grasp();self.switch(True);self.spin(.2)
                elif cmd.phase=='RELEASE':self.release()
                elif cmd.phase=='RETREAT':self.retreat(path)
                else:raise RuntimeError('unknown physical command '+cmd.phase)
                if not self.stop_requested:self.step_done=True;self.step_ok=True;self.step_detail='measured step completed'
            except Exception as exc:
                if not self.stop_requested:self.step_done=True;self.step_ok=False;self.step_detail=repr(exc)
                self.event('plant_step_exception',detail=repr(exc))
            finally:self.step_running=False
            self.publish_evidence()
        if self.terminal!='COMPLETE':
            self.capture_fault_stop()
            raise RuntimeError('formal executor ended '+self.terminal)
        self.event('physical_transport_complete',destination=self.pose('c43_sample'))
        (OUT/'executor_integration_result.json').write_text(json.dumps(dict(executor_state=self.terminal,
            shared_cpp_state_machine=True,full_w7_acceptance=False),indent=2)+'\n')


def main():
    rclpy.init();gc_was_enabled=gc.isenabled()
    # This bounded experiment retains large acyclic telemetry lists. An implicit
    # whole-heap cyclic scan pauses every Python thread, including observation.
    gc.disable();node=ContactPlant();error=None
    node.event('python_gc_policy',automatic_cyclic_gc=False,scope='bounded physical experiment; restore after teardown')
    try:node.run()
    except Exception as exc:error=repr(exc);node.event('error',detail=error)
    finally:
        if node.executor_resume_timer is not None:node.executor_resume_timer.cancel()
        node.restore_executor()
        for process in node.wrench_processes:
            try:process.wait(timeout=2)
            except subprocess.TimeoutExpired:process.terminate()
        node.finish(error);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (OUT/'python_gc_policy.json').write_text(json.dumps(dict(
            automatic_cyclic_gc_during_run=False,previously_enabled=gc_was_enabled,
            final_allocation_counts=gc.get_count(),restored_after_teardown=gc_was_enabled),indent=2)+'\n')
        if gc_was_enabled:gc.enable()
    return int(error is not None)


if __name__=='__main__':raise SystemExit(main())
